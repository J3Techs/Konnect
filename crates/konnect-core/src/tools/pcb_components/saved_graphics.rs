//! Closed-board reads and edits of one graphic item inside a placed footprint.
//!
//! KiCad 10's typed API reaches a footprint's artwork only by sending the
//! whole `FootprintInstance` back, and `FOOTPRINT::Deserialize` re-creates
//! every pad from a message with no field for a pad's fabrication property
//! (see `footprint_fabrication`). On a board KiCad does not hold, this module
//! edits the saved file instead: it rewrites only the defining-point clauses
//! of the one item the UUID names — `(start …)`, `(mid …)`, `(end …)`,
//! `(center …)` or `(pts …)` — or cuts that item's block, and every other byte
//! of the board stays as it was. Pad clauses, fabrication properties, nets,
//! fields and UUIDs are untouched by construction, and the write is checked
//! byte for byte outside the planned spans.
//!
//! Points are footprint-local millimetres as the saved board shows them: the
//! offset from the footprint anchor in its unrotated frame, already mirrored
//! for a back-side footprint. `list_board_footprint_graphics` reports the same
//! numbers live and saved.

use super::reference_field::{footprints_by_reference, head_of, mm, round_nm};
use super::*;
use crate::tools::closed_board::only_spans_changed;
use crate::tools::footprint_fabrication::{footprint_pad_properties, PadProperty};
use konnect_sexp::SexpNode;
use serde::Serialize;
use serde_json::Value;

pub(crate) const OPERATION: &str = "edit_board_footprint_graphic";
const TOLERANCE: f64 = 5e-7;
const MAX_LOCAL_MM: f64 = 1000.0;

/// A footprint-local point, serialized as the live listing's `{x, y}`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub(crate) struct Xy {
    pub x: f64,
    pub y: f64,
}

/// One graphic item of a saved footprint, as `list_board_footprint_graphics`
/// reports it.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub(crate) struct SavedGraphic {
    pub uuid: String,
    pub kind: &'static str,
    pub layer: String,
    pub points: Vec<Xy>,
    pub outlines: usize,
    pub holes: usize,
    pub editable: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stroke_width_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub not_editable_reason: Option<String>,
    #[serde(skip)]
    span: (usize, usize),
}

/// What one request does to the item.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum GraphicChange {
    /// New defining points, in the order `list_board_footprint_graphics`
    /// reports them for the item's kind.
    Points(Vec<(f64, f64)>),
    Delete,
}

impl GraphicChange {
    pub(crate) fn json(&self) -> Value {
        match self {
            Self::Points(points) => json!({
                "points": points.iter().map(|&(x, y)| json!([round_nm(x), round_nm(y)])).collect::<Vec<_>>()
            }),
            Self::Delete => json!({ "delete": true }),
        }
    }
}

/// The saved-file tag of each kind, named as the live listing names it.
fn kind_of(tag: &str) -> Option<&'static str> {
    Some(match tag {
        "fp_line" => "segment",
        "fp_arc" => "arc",
        "fp_circle" => "circle",
        "fp_rect" => "rectangle",
        "fp_poly" => "polygon",
        "fp_curve" => "bezier",
        _ => return None,
    })
}

/// The clauses carrying each kind's defining points, in `points` order. The
/// order matches the live listing: a circle's second point is on the circle,
/// a rectangle's are opposite corners.
pub(crate) fn point_tags(kind: &str) -> &'static [&'static str] {
    match kind {
        "segment" | "rectangle" => &["start", "end"],
        "arc" => &["start", "mid", "end"],
        "circle" => &["center", "end"],
        _ => &[],
    }
}

/// How each kind's `points` are read, for tool descriptions and refusals.
pub(crate) const POINTS_CONVENTION: &str = "points by kind: polygon = its vertices (at least \
     3); segment = [start, end]; arc = [start, mid, end]; circle = [center, a point on the \
     circle]; rectangle = two opposite corners. Footprint-local millimetres as the saved board \
     shows them (offset from the anchor in the unrotated frame, already mirrored on a back-side \
     footprint).";

fn xy_of(node: &SexpNode, tag: &str) -> Option<Xy> {
    let clause = node.find(tag)?;
    Some(Xy {
        x: clause.get_f64(1)?,
        y: clause.get_f64(2)?,
    })
}

/// The single footprint `reference` names in the saved board.
fn footprint_span(source: &str, reference: &str) -> Result<(usize, usize), String> {
    let index = footprints_by_reference(source)?;
    match index.get(reference).map(Vec::as_slice).unwrap_or(&[]) {
        [] => Err(format!(
            "footprint '{reference}' was not found in the saved board"
        )),
        [span] => Ok(*span),
        many => Err(format!(
            "footprint reference '{reference}' appears {} times in the saved board",
            many.len()
        )),
    }
}

/// Every graphic item that is a direct child of the footprint `reference`
/// names, in file order.
pub(crate) fn saved_graphics(source: &str, reference: &str) -> Result<Vec<SavedGraphic>, String> {
    graphics_in(source, footprint_span(source, reference)?)
}

fn graphics_in(source: &str, footprint: (usize, usize)) -> Result<Vec<SavedGraphic>, String> {
    let block = &source[footprint.0..footprint.1];
    let mut out = Vec::new();
    for (s, e) in find_direct_child_blocks(block, "footprint") {
        let text = &block[s..e];
        let Some(kind) = kind_of(head_of(text)) else {
            continue;
        };
        let node = konnect_sexp::parse_sexp(text).map_err(|error| error.to_string())?;
        out.push(read_graphic(
            &node,
            kind,
            (footprint.0 + s, footprint.0 + e),
        ));
    }
    Ok(out)
}

fn read_graphic(node: &SexpNode, kind: &'static str, span: (usize, usize)) -> SavedGraphic {
    let uuid = node
        .find_str("uuid")
        .or_else(|| node.find_str("tstamp"))
        .unwrap_or_default()
        .to_string();
    let mut reason = None;
    let points = match kind {
        "polygon" | "bezier" => {
            let entries = node
                .find("pts")
                .and_then(SexpNode::children)
                .map(|children| &children[1..])
                .unwrap_or(&[]);
            let mut points = Vec::new();
            for entry in entries {
                match (entry.head(), entry.get_f64(1), entry.get_f64(2)) {
                    (Some("xy"), Some(x), Some(y)) => points.push(Xy { x, y }),
                    (head, _, _) => {
                        reason = Some(format!(
                            "its outline holds a '{}' entry, not only (xy …) vertices; replacing \
                             the outline would flatten it",
                            head.unwrap_or("?")
                        ))
                    }
                }
            }
            points
        }
        _ => point_tags(kind)
            .iter()
            .filter_map(|tag| xy_of(node, tag))
            .collect(),
    };
    if kind == "bezier" {
        reason = Some("a Bezier curve is reported, not edited".to_string());
    } else if kind != "polygon" && points.len() != point_tags(kind).len() {
        reason = Some("one of its defining points is missing or unreadable".to_string());
    }
    if uuid.is_empty() {
        reason = Some("it carries no UUID to select it by".to_string());
    }
    SavedGraphic {
        uuid,
        kind,
        layer: node.find_str("layer").unwrap_or_default().to_string(),
        outlines: usize::from(kind == "polygon"),
        holes: 0,
        editable: reason.is_none(),
        stroke_width_mm: node
            .find("stroke")
            .and_then(|stroke| stroke.find_f64("width"))
            .or_else(|| node.find_f64("width")),
        not_editable_reason: reason,
        points,
        span,
    }
}

fn same(a: (f64, f64), b: (f64, f64)) -> bool {
    (a.0 - b.0).abs() <= TOLERANCE && (a.1 - b.1).abs() <= TOLERANCE
}

/// Validate `points` for an item of `kind` and round them to KiCad's
/// nanometre grid. A polygon's repeated closing vertex is dropped, as KiCad
/// saves an outline without one.
pub(crate) fn check_points(kind: &str, points: &[(f64, f64)]) -> Result<Vec<(f64, f64)>, String> {
    if points.iter().any(|&(x, y)| {
        !x.is_finite() || !y.is_finite() || x.abs() > MAX_LOCAL_MM || y.abs() > MAX_LOCAL_MM
    }) {
        return Err(format!(
            "every point must be finite and within ±{MAX_LOCAL_MM} mm of the footprint anchor"
        ));
    }
    let mut points: Vec<(f64, f64)> = points
        .iter()
        .map(|&(x, y)| (round_nm(x), round_nm(y)))
        .collect();
    let count = |want: usize| {
        if points.len() == want {
            Ok(())
        } else {
            Err(format!(
                "a {kind} takes exactly {want} points, got {}; {POINTS_CONVENTION}",
                points.len()
            ))
        }
    };
    match kind {
        "polygon" => {
            if points.len() > 3 && same(points[0], points[points.len() - 1]) {
                points.pop();
            }
            if points.len() < 3 {
                return Err(format!(
                    "a polygon needs at least 3 distinct vertices, got {}",
                    points.len()
                ));
            }
            if (0..points.len()).any(|i| same(points[i], points[(i + 1) % points.len()])) {
                return Err("a polygon may not repeat a vertex consecutively".to_string());
            }
            let twice_area: f64 = (0..points.len())
                .map(|i| {
                    let (a, b) = (points[i], points[(i + 1) % points.len()]);
                    a.0 * b.1 - b.0 * a.1
                })
                .sum();
            if twice_area.abs() < 1e-9 {
                return Err("the polygon's vertices enclose no area".to_string());
            }
        }
        "segment" | "circle" => {
            count(2)?;
            if same(points[0], points[1]) {
                return Err(format!("a {kind}'s two points must differ"));
            }
        }
        "rectangle" => {
            count(2)?;
            if (points[0].0 - points[1].0).abs() <= TOLERANCE
                || (points[0].1 - points[1].1).abs() <= TOLERANCE
            {
                return Err("a rectangle's corners must differ in both x and y".to_string());
            }
        }
        "arc" => {
            count(3)?;
            let (a, b, c) = (points[0], points[1], points[2]);
            let cross = (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0);
            if same(a, b) || same(b, c) || same(a, c) || cross.abs() < 1e-9 {
                return Err(
                    "an arc's start, mid and end must be three distinct, non-collinear points"
                        .to_string(),
                );
            }
        }
        other => return Err(format!("a {other} cannot be edited here")),
    }
    Ok(points)
}

/// The planned write.
#[derive(Debug, Clone)]
pub(crate) struct Prepared {
    pub candidate: String,
    /// `(start, end, replacement_len)` against the original.
    pub edits: Vec<(usize, usize, usize)>,
    pub before: SavedGraphic,
    pub expected: Expected,
    pub pad_properties: Vec<PadProperty>,
}

impl Prepared {
    pub(crate) fn changed(&self) -> bool {
        !self.edits.is_empty()
    }
}

/// What the item must read back as.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum Expected {
    Points(&'static str, Vec<(f64, f64)>),
    Deleted,
}

fn one_child(block: &str, parent: &str, tag: &str) -> Result<(usize, usize), String> {
    let spans: Vec<_> = find_direct_child_blocks(block, parent)
        .into_iter()
        .filter(|&(s, e)| head_of(&block[s..e]) == tag)
        .collect();
    match spans.as_slice() {
        [span] => Ok(*span),
        [] => Err(format!("the item has no ({tag} …) clause")),
        _ => Err(format!("the item has more than one ({tag} …) clause")),
    }
}

/// The whitespace a clause at `start` of `block` is indented by, when it sits
/// on its own line.
fn indent_before(block: &str, start: usize) -> Option<&str> {
    let line_start = block[..start].rfind('\n').map_or(0, |i| i + 1);
    let indent = &block[line_start..start];
    indent
        .chars()
        .all(|c| c == ' ' || c == '\t')
        .then_some(indent)
}

fn pts_clause(points: &[(f64, f64)], indent: Option<&str>) -> String {
    let xys = points
        .iter()
        .map(|&(x, y)| format!("(xy {} {})", mm(x), mm(y)))
        .collect::<Vec<_>>()
        .join(" ");
    match indent {
        Some(indent) => format!("(pts\n{indent}\t{xys}\n{indent})"),
        None => format!("(pts {xys})"),
    }
}

/// Plan the edit of the item `uuid` inside the footprint `reference`, and
/// check the candidate exactly as an apply checks the written file.
pub(crate) fn prepare(
    source: &str,
    reference: &str,
    uuid: &str,
    change: &GraphicChange,
) -> Result<Prepared, String> {
    check_single_board_form(source)?;
    let footprint = footprint_span(source, reference)?;
    let graphics = graphics_in(source, footprint)?;
    let target = match graphics
        .iter()
        .filter(|graphic| graphic.uuid == uuid)
        .collect::<Vec<_>>()
        .as_slice()
    {
        [graphic] => (*graphic).clone(),
        [] => {
            return Err(format!(
                "graphic '{uuid}' is not a graphic item of '{reference}' in the saved board; \
                 list_board_footprint_graphics reports the UUIDs it has"
            ))
        }
        many => {
            return Err(format!(
                "graphic UUID '{uuid}' names {} items of '{reference}'",
                many.len()
            ))
        }
    };
    let pad_properties = footprint_properties(source, footprint)?;
    let (start, end) = target.span;
    let block = &source[start..end];
    let tag = head_of(block).to_string();
    let mut changes = Vec::new();
    let expected = match change {
        GraphicChange::Delete => {
            let elsewhere = source.matches(uuid).count() - block.matches(uuid).count();
            if elsewhere > 0 {
                return Err(format!(
                    "graphic '{uuid}' is named {elsewhere} more time(s) elsewhere in the board, \
                     such as a group's member list; deleting it would leave that reference \
                     dangling, so it is refused"
                ));
            }
            // The item and the whitespace that sets it on its own line.
            let cut_from = source[..start]
                .trim_end_matches([' ', '\t', '\r', '\n'])
                .len();
            changes.push(SexpEdit::delete(cut_from, end));
            Expected::Deleted
        }
        GraphicChange::Points(points) => {
            if let Some(reason) = &target.not_editable_reason {
                return Err(format!(
                    "{} '{uuid}' on '{reference}' cannot be edited: {reason}",
                    target.kind
                ));
            }
            let points = check_points(target.kind, points)?;
            let current: Vec<(f64, f64)> = target.points.iter().map(|p| (p.x, p.y)).collect();
            if target.kind == "polygon" {
                let unchanged = current.len() == points.len()
                    && current.iter().zip(&points).all(|(a, b)| same(*a, *b));
                if !unchanged {
                    let (s, e) = one_child(block, &tag, "pts")?;
                    changes.push(SexpEdit::replace(
                        start + s,
                        start + e,
                        pts_clause(&points, indent_before(block, s)),
                    ));
                }
            } else {
                for ((clause, &point), &now) in
                    point_tags(target.kind).iter().zip(&points).zip(&current)
                {
                    if same(point, now) {
                        continue;
                    }
                    let (s, e) = one_child(block, &tag, clause)?;
                    changes.push(SexpEdit::replace(
                        start + s,
                        start + e,
                        format!("({clause} {} {})", mm(point.0), mm(point.1)),
                    ));
                }
            }
            Expected::Points(target.kind, points)
        }
    };
    let edits: Vec<_> = changes
        .iter()
        .map(|edit| (edit.start, edit.end, edit.replacement.len()))
        .collect();
    let candidate = apply_edits(source.to_string(), changes);
    let prepared = Prepared {
        candidate,
        edits,
        before: target,
        expected,
        pad_properties,
    };
    verify(source, &prepared.candidate, &prepared, reference, uuid)?;
    Ok(prepared)
}

fn footprint_properties(
    source: &str,
    footprint: (usize, usize),
) -> Result<Vec<PadProperty>, String> {
    let node = konnect_sexp::parse_sexp(&source[footprint.0..footprint.1])
        .map_err(|error| error.to_string())?;
    Ok(footprint_pad_properties(&node))
}

/// The candidate (or persisted) board differs from the original only inside
/// the planned spans, the item reads back as planned, and the footprint's pad
/// fabrication properties are the ones it had.
pub(crate) fn verify(
    original: &str,
    candidate: &str,
    prepared: &Prepared,
    reference: &str,
    uuid: &str,
) -> Result<Value, String> {
    if !only_spans_changed(original, candidate, &prepared.edits) {
        return Err("bytes outside the planned graphic spans changed".to_string());
    }
    check_single_board_form(candidate)?;
    let footprint = footprint_span(candidate, reference)?;
    let graphics = graphics_in(candidate, footprint)?;
    let found: Vec<_> = graphics.iter().filter(|g| g.uuid == uuid).collect();
    match (&prepared.expected, found.as_slice()) {
        (Expected::Deleted, []) => {}
        (Expected::Deleted, _) => return Err(format!("graphic '{uuid}' is still present")),
        (Expected::Points(kind, points), [graphic]) => {
            let matches = graphic.kind == *kind
                && graphic.points.len() == points.len()
                && graphic
                    .points
                    .iter()
                    .zip(points)
                    .all(|(have, want)| same((have.x, have.y), *want));
            if !matches {
                return Err(format!("graphic '{uuid}' does not read back as planned"));
            }
        }
        (Expected::Points(..), _) => {
            return Err(format!("graphic '{uuid}' is no longer exactly one item"))
        }
    }
    let properties = footprint_properties(candidate, footprint)?;
    if properties != prepared.pad_properties {
        return Err(format!(
            "the pad fabrication properties of '{reference}' changed"
        ));
    }
    konnect_sexp::parse_sexp(candidate).map_err(|error| error.to_string())?;
    Ok(json!({
        "graphic": match &prepared.expected {
            Expected::Deleted => json!({ "uuid": uuid, "deleted": true }),
            Expected::Points(kind, points) => json!({
                "uuid": uuid,
                "kind": kind,
                "points": points.iter().map(|&(x, y)| json!({ "x": x, "y": y })).collect::<Vec<_>>(),
            }),
        },
        "pad_fabrication_properties": properties,
        "bytes_outside_planned_spans_unchanged": true,
    }))
}

/// The closed-board form of `edit_board_footprint_graphic`, reached only once
/// the live write gate has said no KiCad holds the board.
pub(super) async fn handle_closed(
    ctx: &ToolContext,
    board: &Path,
    reference: &str,
    uuid: &str,
    change: &GraphicChange,
    dry_run: bool,
    expected_revision: Option<&str>,
) -> anyhow::Result<CallToolResult> {
    let closure = match crate::tools::closed_board::prove_closed(ctx, board, OPERATION).await? {
        Ok(closure) => closure,
        Err(refusal) => return Ok(refusal),
    };
    let original = read_consistent(board)?;
    let prepared = match prepare(&original, reference, uuid, change) {
        Ok(prepared) => prepared,
        Err(reason) => {
            return Ok(CallToolResult::error_kind(
                crate::mcp::error::ToolErrorKind::PlanBlocked {
                    operation: OPERATION.to_string(),
                    reasons: vec![reason.clone()],
                },
                format!("No footprint graphic changed: {reason}. The board file was not modified."),
            ))
        }
    };
    let revision = crate::tools::closed_board::revision(
        OPERATION,
        &original,
        &json!({ "reference": reference, "uuid": uuid, "change": change.json() }),
    );
    if expected_revision.is_some_and(|expected| expected != revision) {
        return Ok(crate::tools::closed_board::stale_revision(board, OPERATION));
    }
    let changed = prepared.changed();
    let after = match &prepared.expected {
        Expected::Deleted => Value::Null,
        Expected::Points(kind, points) => json!({
            "uuid": uuid,
            "kind": kind,
            "layer": prepared.before.layer,
            "points": points.iter().map(|&(x, y)| json!({ "x": x, "y": y })).collect::<Vec<_>>(),
        }),
    };
    let mut body = json!({
        "status": match (changed, dry_run) {
            (false, _) => "noop",
            (true, true) => "ready",
            (true, false) => "applied",
        },
        "source": "saved",
        "edited": reference,
        "uuid": uuid,
        "kind": prepared.before.kind,
        "action": match change {
            GraphicChange::Delete => "delete",
            GraphicChange::Points(_) => "edit",
        },
        "revision": revision,
        "before": prepared.before,
        "after": after,
        "pad_fabrication_properties": prepared.pad_properties,
        "board_closure": closure,
        "other_footprint_clauses": "unchanged byte-for-byte; no whole-footprint typed update",
        "convention": POINTS_CONVENTION,
    });
    if changed && !dry_run {
        if let Err(refusal) =
            crate::tools::closed_board::prove_closed(ctx, board, OPERATION).await?
        {
            return Ok(refusal);
        }
        match crate::tools::closed_board::write_and_read_back(
            board,
            OPERATION,
            &original,
            &prepared.candidate,
            |text| verify(&original, text, &prepared, reference, uuid),
        ) {
            Ok(write) => body["write"] = write,
            Err(refusal) => return Ok(refusal),
        }
    }
    Ok(CallToolResult::json(&body))
}

/// The closed-board form of `list_board_footprint_graphics`.
pub(super) fn list_saved(board: &Path, reference: &str) -> Result<Vec<SavedGraphic>, String> {
    let source = read_consistent(board).map_err(|error| error.to_string())?;
    check_single_board_form(&source)?;
    saved_graphics(&source, reference)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tools::pcb_board::board_mock::ctx_talking_to;

    /// A QFN whose thermal pad carries `pad_prop_heatsink`, with a pin-1
    /// polygon, and a back-side diode whose outline is line segments, an arc,
    /// a circle and a rectangle; one segment is a group member.
    const BOARD: &str = r#"(kicad_pcb (version 20260206)
	(footprint "Lib:QFN" (layer "F.Cu") (uuid "11111111-1111-4111-8111-111111111111") (at 20 30 -90)
		(property "Reference" "U5901" (at 0 -3 90) (layer "F.SilkS") (uuid "21111111-1111-4111-8111-111111111111")
			(effects (font (size 0.8 0.8) (thickness 0.12))))
		(fp_poly
			(pts
				(xy -2.61 -1.25) (xy -2.94 -1.01) (xy -2.94 -1.49)
			)
			(stroke
				(width 0.12)
				(type solid)
			)
			(fill yes)
			(layer "F.SilkS")
			(uuid "9ae97e9c-d041-464b-b45e-4b1ac6862b9c")
		)
		(fp_poly (pts (xy 0 0) (arc (start 1 0) (mid 1.5 0.5) (end 1 1)) (xy 0 1)) (stroke (width 0.1) (type solid)) (fill no) (layer "F.Fab") (uuid "31111111-1111-4111-8111-111111111111"))
		(fp_curve (pts (xy 0 0) (xy 1 1) (xy 2 1) (xy 3 0)) (stroke (width 0.1) (type solid)) (layer "F.Fab") (uuid "41111111-1111-4111-8111-111111111111"))
		(pad "25" smd rect (at 0 0 -90) (size 2.5 2.5) (property pad_prop_heatsink) (layers "F.Cu" "F.Mask") (net 1 "GND") (uuid "51111111-1111-4111-8111-111111111111"))
	)
	(footprint "Lib:SMB" (layer "B.Cu") (uuid "12222222-2222-4222-8222-222222222222") (at 50 60 180)
		(property "Reference" "D6602" (at 0 2 180) (layer "B.SilkS") (uuid "22222222-2222-4222-8222-222222222222")
			(effects (font (size 1 1) (thickness 0.15)) (justify mirror)))
		(fp_line
			(start -1.61 0.85)
			(end -1.61 -0.85)
			(stroke
				(width 0.12)
				(type solid)
			)
			(layer "B.SilkS")
			(uuid "a31369d9-c93c-4c51-9638-0198706bcd7e")
		)
		(fp_line (start 1 1) (end 2 1) (stroke (width 0.12) (type solid)) (layer "B.SilkS") (uuid "a4444444-4444-4444-8444-444444444444"))
		(fp_arc (start 0 -1) (mid 1 0) (end 0 1) (stroke (width 0.12) (type solid)) (layer "B.SilkS") (uuid "a5555555-5555-4555-8555-555555555555"))
		(fp_circle (center 0 0) (end 0.5 0) (stroke (width 0.05) (type solid)) (fill no) (layer "B.CrtYd") (uuid "a6666666-6666-4666-8666-666666666666"))
		(fp_rect (start -2 -1) (end 2 1) (stroke (width 0.1) (type solid)) (fill no) (layer "B.Fab") (uuid "a7777777-7777-4777-8777-777777777777"))
		(group "" (uuid "a8888888-8888-4888-8888-888888888888") (members "a4444444-4444-4444-8444-444444444444"))
		(pad "1" smd rect (at -2 0 180) (size 1 1) (property pad_prop_testpoint) (layers "B.Cu" "B.Mask") (net 2 "A") (uuid "52222222-2222-4222-8222-222222222222"))
	)
	(gr_line (start 0 0) (end 1 1) (stroke (width 0.1) (type solid)) (layer "Edge.Cuts") (uuid "61111111-1111-4111-8111-111111111111"))
)
"#;

    const PIN1: &str = "9ae97e9c-d041-464b-b45e-4b1ac6862b9c";
    const SEGMENT: &str = "a31369d9-c93c-4c51-9638-0198706bcd7e";

    fn points(values: &[(f64, f64)]) -> GraphicChange {
        GraphicChange::Points(values.to_vec())
    }

    #[test]
    fn lists_each_kind_with_its_points_and_why_some_are_not_editable() {
        let qfn = saved_graphics(BOARD, "U5901").unwrap();
        assert_eq!(qfn.len(), 3);
        assert_eq!(qfn[0].kind, "polygon");
        assert_eq!(qfn[0].points.len(), 3);
        assert!(qfn[0].editable);
        assert_eq!(qfn[0].stroke_width_mm, Some(0.12));
        assert!(!qfn[1].editable, "an outline with an arc is not flattened");
        assert!(qfn[1]
            .not_editable_reason
            .as_deref()
            .unwrap()
            .contains("'arc'"));
        assert_eq!(qfn[2].kind, "bezier");
        assert!(!qfn[2].editable);

        let diode = saved_graphics(BOARD, "D6602").unwrap();
        let kinds: Vec<_> = diode.iter().map(|g| g.kind).collect();
        assert_eq!(kinds, ["segment", "segment", "arc", "circle", "rectangle"]);
        assert!(diode.iter().all(|g| g.editable));
        assert_eq!(
            diode[0].points,
            [Xy { x: -1.61, y: 0.85 }, Xy { x: -1.61, y: -0.85 }]
        );
        assert_eq!(diode[2].points.len(), 3);
        assert_eq!(diode[0].layer, "B.SilkS");
        assert!(saved_graphics(BOARD, "NOPE").is_err());
    }

    #[test]
    fn a_polygon_on_a_footprint_with_a_heatsink_pad_changes_only_its_pts() {
        let prepared = prepare(
            BOARD,
            "U5901",
            PIN1,
            &points(&[(-2.61, -1.1), (-2.94, -0.94), (-2.94, -1.26)]),
        )
        .unwrap();
        assert_eq!(prepared.edits.len(), 1);
        assert!(prepared
            .candidate
            .contains("(pts\n\t\t\t\t(xy -2.61 -1.1) (xy -2.94 -0.94) (xy -2.94 -1.26)\n\t\t\t)"));
        assert!(prepared
            .candidate
            .contains("(property pad_prop_heatsink) (layers \"F.Cu\" \"F.Mask\") (net 1 \"GND\")"));
        assert_eq!(
            prepared.pad_properties,
            [PadProperty {
                pad: "25".to_string(),
                property: "pad_prop_heatsink".to_string()
            }]
        );
        // Re-planned on the result, the same request is a no-op.
        let again = prepare(
            &prepared.candidate,
            "U5901",
            PIN1,
            &points(&[(-2.61, -1.1), (-2.94, -0.94), (-2.94, -1.26), (-2.61, -1.1)]),
        )
        .unwrap();
        assert!(!again.changed());
    }

    #[test]
    fn segment_arc_circle_and_rectangle_rewrite_only_the_moved_points() {
        let shortened = prepare(
            BOARD,
            "D6602",
            SEGMENT,
            &points(&[(-1.61, 0.5), (-1.61, -0.85)]),
        )
        .unwrap();
        assert_eq!(
            shortened.edits.len(),
            1,
            "the unchanged end is not rewritten"
        );
        assert!(shortened
            .candidate
            .contains("(start -1.61 0.5)\n\t\t\t(end -1.61 -0.85)"));

        let arc = prepare(
            BOARD,
            "D6602",
            "a5555555-5555-4555-8555-555555555555",
            &points(&[(0.0, -1.0), (-1.0, 0.0), (0.0, 1.0)]),
        )
        .unwrap();
        assert!(arc
            .candidate
            .contains("(fp_arc (start 0 -1) (mid -1 0) (end 0 1)"));

        let circle = prepare(
            BOARD,
            "D6602",
            "a6666666-6666-4666-8666-666666666666",
            &points(&[(0.0, 0.0), (0.75, 0.0)]),
        )
        .unwrap();
        assert!(circle.candidate.contains("(center 0 0) (end 0.75 0)"));

        let rect = prepare(
            BOARD,
            "D6602",
            "a7777777-7777-4777-8777-777777777777",
            &points(&[(-2.0, -1.0), (1.5, 1.0)]),
        )
        .unwrap();
        assert!(rect
            .candidate
            .contains("(fp_rect (start -2 -1) (end 1.5 1)"));
    }

    #[test]
    fn delete_cuts_the_item_and_its_line_and_refuses_a_group_member() {
        let deleted = prepare(BOARD, "D6602", SEGMENT, &GraphicChange::Delete).unwrap();
        assert!(!deleted.candidate.contains(SEGMENT));
        assert!(deleted
            .candidate
            .contains("(justify mirror)))\n\t\t(fp_line (start 1 1)"));
        assert!(prepare(&deleted.candidate, "D6602", SEGMENT, &GraphicChange::Delete).is_err());
        assert_eq!(
            saved_graphics(&deleted.candidate, "D6602").unwrap().len(),
            4
        );

        let grouped = prepare(
            BOARD,
            "D6602",
            "a4444444-4444-4444-8444-444444444444",
            &GraphicChange::Delete,
        )
        .unwrap_err();
        assert!(grouped.contains("group"), "{grouped}");
    }

    #[test]
    fn bad_requests_are_refused_before_planning() {
        for (uuid, change, needle) in [
            (SEGMENT, points(&[(0.0, 0.0)]), "exactly 2 points"),
            (SEGMENT, points(&[(1.0, 1.0), (1.0, 1.0)]), "must differ"),
            (PIN1, points(&[(0.0, 0.0), (1.0, 1.0)]), "at least 3"),
            (
                PIN1,
                points(&[(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)]),
                "no area",
            ),
            (
                "a5555555-5555-4555-8555-555555555555",
                points(&[(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)]),
                "non-collinear",
            ),
            (SEGMENT, points(&[(0.0, 0.0), (2000.0, 0.0)]), "within"),
            (
                "41111111-1111-4111-8111-111111111111",
                points(&[(0.0, 0.0)]),
                "Bezier",
            ),
            (
                "61111111-1111-4111-8111-111111111111",
                GraphicChange::Delete,
                "not a graphic item",
            ),
        ] {
            let reference = if uuid == PIN1 || uuid.starts_with('4') || uuid.starts_with('6') {
                "U5901"
            } else {
                "D6602"
            };
            let error = prepare(BOARD, reference, uuid, &change).unwrap_err();
            assert!(error.contains(needle), "{uuid}: {error}");
        }
    }

    fn body(result: &CallToolResult) -> Value {
        match &result.content[0] {
            crate::mcp::protocol::ToolContent::Text { text } => {
                serde_json::from_str(text).unwrap_or_else(|_| json!({ "text": text }))
            }
            other => panic!("expected text, got {other:?}"),
        }
    }

    async fn call(name: &str, args: Value) -> (bool, Value) {
        let def = tools().into_iter().find(|tool| tool.name == name).unwrap();
        assert!(def.input_validator.is_valid(&args), "schema refused {args}");
        let result = (def.handler)(&args, std::sync::Arc::new(ctx_talking_to(String::new())))
            .await
            .unwrap();
        (result.is_error, body(&result))
    }

    #[tokio::test]
    async fn the_closed_board_tool_dry_runs_applies_and_settles() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("silk.kicad_pcb");
        std::fs::write(&board, BOARD).unwrap();

        let (error, listed) = call(
            "list_board_footprint_graphics",
            json!({ "board": board, "reference": "D6602", "layer": "B.SilkS" }),
        )
        .await;
        assert!(!error, "{listed}");
        assert_eq!(listed["source"], "saved");
        assert_eq!(listed["count"], 3);
        assert_eq!(listed["graphics"][0]["uuid"], SEGMENT);
        assert_eq!(listed["graphics"][0]["editable"], true);

        let args = json!({
            "board": board, "reference": "U5901", "uuid": PIN1,
            "points": [{ "x": -2.61, "y": -1.1 }, { "x": -2.94, "y": -0.94 }, { "x": -2.94, "y": -1.26 }],
            "dry_run": true
        });
        let (error, plan) = call(OPERATION, args.clone()).await;
        assert!(!error, "{plan}");
        assert_eq!(plan["status"], "ready");
        assert_eq!(
            plan["pad_fabrication_properties"][0]["property"],
            "pad_prop_heatsink"
        );
        assert_eq!(std::fs::read_to_string(&board).unwrap(), BOARD);

        let mut apply = args.clone();
        apply["dry_run"] = json!(false);
        apply["expected_revision"] = json!("f".repeat(64));
        let (error, stale) = call(OPERATION, apply.clone()).await;
        assert!(error);
        assert_eq!(stale["error"]["reason"], "stale_revision");

        apply["expected_revision"] = plan["revision"].clone();
        let (error, applied) = call(OPERATION, apply).await;
        assert!(!error, "{applied}");
        assert_eq!(applied["status"], "applied");
        assert_eq!(
            applied["write"]["persisted_readback"]["pad_fabrication_properties"][0]["property"],
            "pad_prop_heatsink"
        );
        let (error, again) = call(OPERATION, args).await;
        assert!(!error);
        assert_eq!(again["status"], "noop");

        let (error, deleted) = call(
            OPERATION,
            json!({ "board": board, "reference": "D6602", "uuid": SEGMENT, "delete": true }),
        )
        .await;
        assert!(!error, "{deleted}");
        assert_eq!(deleted["status"], "applied");
        let written = std::fs::read_to_string(&board).unwrap();
        assert!(!written.contains(SEGMENT));
        assert!(written.contains("(property pad_prop_testpoint)"));
    }

    #[tokio::test]
    async fn requests_name_exactly_one_change() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("silk.kicad_pcb");
        std::fs::write(&board, BOARD).unwrap();
        for args in [
            json!({ "board": board, "reference": "D6602", "uuid": SEGMENT }),
            json!({ "board": board, "reference": "D6602", "uuid": SEGMENT, "delete": true,
                    "points": [{ "x": 0, "y": 0 }, { "x": 1, "y": 0 }] }),
            json!({ "board": board, "reference": "D6602", "uuid": SEGMENT, "delete": false }),
        ] {
            let (error, refused) = call(OPERATION, args).await;
            assert!(error, "{refused}");
            assert_eq!(refused["error"]["kind"], "invalid_argument", "{refused}");
        }
        assert_eq!(std::fs::read_to_string(&board).unwrap(), BOARD);
    }
}
