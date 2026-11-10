//! Closed-board repair of a placed footprint's child zones that are not where
//! the footprint is (`edit_component` with `library_zones`).
//!
//! KiCad saves a footprint's child zones in board coordinates (see
//! `child_zones`). A board written by an earlier saved-file placement, move,
//! rotate, flip or closed-board import can hold a footprint at one place and
//! its keepout at another; `audit_manufacturing` reports it as
//! `misplaced_footprint_zone`. This module puts such a zone back: the outline
//! of each `(polygon …)` becomes the library footprint's outline at the
//! instance's position, rotation and side, which is where KiCad saves it for
//! an instance placed there.
//!
//! Only a zone that is the library's zone somewhere else is touched, and in
//! it only the `(pts …)` of its outlines, its `(filled_polygon …)` blocks and
//! the `yes` of its `(fill yes …)`: a fill computed where the zone was is
//! stale, and without it KiCad shows the zone unfilled until its next refill.
//! A zone already in place, and a zone the library does not describe, are
//! reported and left byte for byte. So is everything else in the board, which
//! the write is checked for outside the planned spans.

use super::child_zones::{
    map_pts, outlines, placed_point, same_place, same_shape, Outlines, Point,
};
use super::reference_field::{footprints_by_reference, head_of};
use super::*;
use crate::tools::closed_board::only_spans_changed;
use konnect_sexp::SexpNode;
use serde_json::Value;

pub(crate) const OPERATION: &str = "edit_component library_zones";
const KEYS: [&str; 2] = ["restore", "expected_revision"];
/// An outline longer than this is counted, not listed.
const LISTED_POINTS: usize = 32;

/// What the plan does with one child zone.
#[derive(Debug, Clone)]
struct ZonePlan {
    uuid: String,
    kind: &'static str,
    layers: Vec<String>,
    /// `in_place`, `misplaced` or `not_in_library`.
    state: &'static str,
    before: Outlines,
    /// Where a misplaced zone goes.
    after: Option<Outlines>,
    fills_dropped: usize,
}

/// The planned write.
#[derive(Debug, Clone)]
pub(crate) struct Prepared {
    pub candidate: String,
    /// `(start, end, replacement_len)` against the original.
    pub edits: Vec<(usize, usize, usize)>,
    zones: Vec<ZonePlan>,
    footprint: Value,
    /// The library footprint's source, which the plan depends on.
    library: String,
}

impl Prepared {
    pub(crate) fn changed(&self) -> bool {
        !self.edits.is_empty()
    }
}

fn round(value: f64) -> f64 {
    let rounded = (value * 1e6).round() / 1e6;
    if rounded == 0.0 {
        0.0
    } else {
        rounded
    }
}

fn where_json(outline: &Outlines) -> Value {
    let points = || outline.iter().flatten();
    let low = |pick: fn(&Point) -> f64| points().map(pick).fold(f64::INFINITY, f64::min);
    let high = |pick: fn(&Point) -> f64| points().map(pick).fold(f64::NEG_INFINITY, f64::max);
    let (x1, y1, x2, y2) = (low(|p| p.0), low(|p| p.1), high(|p| p.0), high(|p| p.1));
    let mut out = json!({
        "center": { "x": round((x1 + x2) / 2.0), "y": round((y1 + y2) / 2.0) },
        "bbox": { "x1": round(x1), "y1": round(y1), "x2": round(x2), "y2": round(y2) },
        "points": points().count(),
    });
    if points().count() <= LISTED_POINTS {
        out["outline"] = json!(outline
            .iter()
            .map(|o| o
                .iter()
                .map(|p| json!({ "x": round(p.0), "y": round(p.1) }))
                .collect::<Vec<_>>())
            .collect::<Vec<_>>());
    }
    out
}

impl ZonePlan {
    fn json(&self) -> Value {
        json!({
            "uuid": self.uuid,
            "kind": self.kind,
            "layers": self.layers,
            "state": self.state,
            "before": where_json(&self.before),
            "after": self.after.as_ref().map(where_json),
            "filled_polygons_dropped": self.fills_dropped,
        })
    }
}

/// The `(start, end)` of every direct child `tag` of the block `parent`.
fn children(block: &str, parent: &str, tag: &str) -> Result<Vec<(usize, usize)>, String> {
    Ok(direct_children_with_tags(block, parent)
        .map_err(|error| format!("{error:#}"))?
        .into_iter()
        .filter(|(_, _, head)| head == tag)
        .map(|(start, end, _)| (start, end))
        .collect())
}

/// The `(pts …)` span of each `(polygon …)` of the zone block `zone`.
fn outline_spans(zone: &str) -> Result<Vec<(usize, usize)>, String> {
    children(zone, "zone", "polygon")?
        .into_iter()
        .map(
            |(start, end)| match children(&zone[start..end], "polygon", "pts")?.as_slice() {
                [(s, e)] => Ok((start + s, start + e)),
                _ => Err("a zone (polygon …) does not hold exactly one (pts …)".to_string()),
            },
        )
        .collect()
}

/// One zone of the library footprint, in the footprint's own frame.
struct LibraryZone {
    outline: Outlines,
    /// The `(pts …)` text of each of its polygons.
    pts: Vec<String>,
}

fn library_zones(library: &str) -> Result<Vec<LibraryZone>, String> {
    let library = library.trim();
    if head_of(library) != "footprint" {
        return Err("the library footprint is not a (footprint …) file".to_string());
    }
    let mut out = Vec::new();
    for (start, end) in children(library, "footprint", "zone")? {
        let block = &library[start..end];
        let node = konnect_sexp::parse_sexp(block).map_err(|error| error.to_string())?;
        let Some(outline) = outlines(&node) else {
            continue;
        };
        let pts = outline_spans(block)?
            .into_iter()
            .map(|(s, e)| block[s..e].to_string())
            .collect();
        out.push(LibraryZone { outline, pts });
    }
    Ok(out)
}

fn single_footprint(source: &str, reference: &str) -> Result<(usize, usize), String> {
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

/// The mapped library `(pts …)` in the layout of the clause it replaces, which
/// starts at `at` of `zone`: on its own lines when that one is, whatever the
/// library file's layout.
fn pts_clause(mapped: &str, zone: &str, at: usize, end: usize) -> String {
    let mapped = mapped.trim();
    let inner = mapped
        .strip_prefix("(pts")
        .and_then(|rest| rest.strip_suffix(')'))
        .unwrap_or(mapped);
    let entries = inner.split_whitespace().collect::<Vec<_>>().join(" ");
    let line_start = zone[..at].rfind('\n').map_or(0, |i| i + 1);
    let indent = &zone[line_start..at];
    let eol = if zone.contains("\r\n") { "\r\n" } else { "\n" };
    if zone[at..end].contains('\n') && indent.chars().all(|c| c == ' ' || c == '\t') {
        format!("(pts{eol}{indent}\t{entries}{eol}{indent})")
    } else {
        format!("(pts {entries})")
    }
}

/// `(fill yes …)` without its `yes`, or `None` when the fill is not marked.
fn unfilled(fill: &str) -> Option<String> {
    let rest = fill.strip_prefix("(fill")?;
    let marked = rest.trim_start().strip_prefix("yes")?;
    marked
        .starts_with(|c: char| c.is_whitespace() || c == ')')
        .then(|| format!("(fill{marked}"))
}

/// Plan the repair of the child zones of the footprint `reference`, and check
/// the candidate exactly as an apply checks the written file. `resolve` reads
/// the library source of a `Library:Footprint` identifier.
pub(crate) fn prepare(
    source: &str,
    reference: &str,
    resolve: &dyn Fn(&str) -> Result<String, String>,
) -> Result<Prepared, String> {
    check_single_board_form(source)?;
    let (start, end) = single_footprint(source, reference)?;
    let block = &source[start..end];
    let node = konnect_sexp::parse_sexp(block).map_err(|error| error.to_string())?;
    let lib_id = node.get(1).and_then(SexpNode::as_str).unwrap_or_default();
    let at = node
        .find("at")
        .ok_or_else(|| format!("footprint '{reference}' has no position"))?;
    let position = (
        at.get_f64(1)
            .ok_or("the footprint position is unreadable")?,
        at.get_f64(2)
            .ok_or("the footprint position is unreadable")?,
    );
    let rotation = at.get_f64(3).unwrap_or(0.0);
    let back = node
        .find_str("layer")
        .is_some_and(|layer| layer.starts_with("B."));
    let zone_spans = children(block, "footprint", "zone")?;
    if zone_spans.is_empty() {
        return Err(format!("footprint '{reference}' has no zone of its own"));
    }
    let library = resolve(lib_id).map_err(|reason| {
        format!("the library footprint '{lib_id}' of '{reference}' could not be read: {reason}")
    })?;
    let library_zones = library_zones(&library)?;
    let place = |p: Point| placed_point(p, position, rotation, back);
    let placed: Vec<Outlines> = library_zones
        .iter()
        .map(|zone| {
            zone.outline
                .iter()
                .map(|o| o.iter().map(|p| place(*p)).collect())
                .collect()
        })
        .collect();

    let mut changes = Vec::new();
    let mut zones = Vec::new();
    for (index, (zone_start, zone_end)) in zone_spans.into_iter().enumerate() {
        let zone = &block[zone_start..zone_end];
        let zone_node = konnect_sexp::parse_sexp(zone).map_err(|error| error.to_string())?;
        let uuid = zone_node
            .find_str("uuid")
            .or_else(|| zone_node.find_str("tstamp"))
            .unwrap_or_default()
            .to_string();
        let before = outlines(&zone_node)
            .ok_or_else(|| format!("zone '{uuid}' of '{reference}' has no readable outline"))?;
        let mut plan = ZonePlan {
            uuid,
            kind: if zone_node.find("keepout").is_some() {
                "rule_area"
            } else {
                "copper"
            },
            layers: zone_node
                .find("layers")
                .or_else(|| zone_node.find("layer"))
                .and_then(SexpNode::children)
                .unwrap_or_default()
                .iter()
                .skip(1)
                .filter_map(SexpNode::as_str)
                .map(str::to_string)
                .collect(),
            state: "not_in_library",
            before,
            after: None,
            fills_dropped: 0,
        };
        if placed.iter().any(|want| same_place(&plan.before, want)) {
            plan.state = "in_place";
            zones.push(plan);
            continue;
        }
        let moved = |k: &usize| same_shape(&plan.before, &library_zones[*k].outline);
        let Some(k) = Some(index)
            .filter(|k| *k < library_zones.len())
            .filter(moved)
            .or_else(|| (0..library_zones.len()).find(moved))
        else {
            zones.push(plan);
            continue;
        };
        plan.state = "misplaced";
        plan.after = Some(placed[k].clone());
        let base = start + zone_start;
        for ((s, e), pts) in outline_spans(zone)?.into_iter().zip(&library_zones[k].pts) {
            let mapped = map_pts(pts, &|x, y| place((x, y))).map_err(|e| format!("{e:#}"))?;
            changes.push(SexpEdit::replace(
                base + s,
                base + e,
                pts_clause(&mapped, zone, s, e),
            ));
        }
        for (s, e) in children(zone, "zone", "filled_polygon")? {
            // The block and the whitespace that sets it on its own line.
            let cut_from = zone[..s].trim_end_matches([' ', '\t', '\r', '\n']).len();
            changes.push(SexpEdit::delete(base + cut_from, base + e));
            plan.fills_dropped += 1;
        }
        for (s, e) in children(zone, "zone", "fill")? {
            if let Some(fill) = unfilled(&zone[s..e]) {
                changes.push(SexpEdit::replace(base + s, base + e, fill));
            }
        }
        zones.push(plan);
    }
    let edits: Vec<_> = changes
        .iter()
        .map(|edit| (edit.start, edit.end, edit.replacement.len()))
        .collect();
    let prepared = Prepared {
        candidate: apply_edits(source.to_string(), changes),
        edits,
        zones,
        footprint: json!({
            "reference": reference,
            "lib_id": lib_id,
            "uuid": node.find_str("uuid").or_else(|| node.find_str("tstamp")),
            "x": position.0,
            "y": position.1,
            "rotation": rotation,
            "side": if back { "back" } else { "front" },
        }),
        library,
    };
    verify(source, &prepared.candidate, &prepared, reference)?;
    Ok(prepared)
}

/// The candidate (or persisted) board differs from the original only inside
/// the planned spans, and every zone of the footprint reads back as planned:
/// a restored one at the library's place with no fill left, any other where
/// it was.
pub(crate) fn verify(
    original: &str,
    candidate: &str,
    prepared: &Prepared,
    reference: &str,
) -> Result<Value, String> {
    if !only_spans_changed(original, candidate, &prepared.edits) {
        return Err("bytes outside the planned zone spans changed".to_string());
    }
    check_single_board_form(candidate)?;
    konnect_sexp::parse_sexp(candidate).map_err(|error| error.to_string())?;
    let (start, end) = single_footprint(candidate, reference)?;
    let block = &candidate[start..end];
    let spans = children(block, "footprint", "zone")?;
    if spans.len() != prepared.zones.len() {
        return Err(format!("'{reference}' no longer has the same zones"));
    }
    for ((s, e), plan) in spans.into_iter().zip(&prepared.zones) {
        let zone = &block[s..e];
        let node = konnect_sexp::parse_sexp(zone).map_err(|error| error.to_string())?;
        let now = outlines(&node)
            .ok_or_else(|| format!("zone '{}' no longer has a readable outline", plan.uuid))?;
        let want = plan.after.as_ref().unwrap_or(&plan.before);
        if !same_place(&now, want) {
            return Err(format!(
                "zone '{}' does not read back as planned",
                plan.uuid
            ));
        }
        if plan.after.is_some()
            && (!children(zone, "zone", "filled_polygon")?.is_empty()
                || children(zone, "zone", "fill")?
                    .iter()
                    .any(|&(s, e)| unfilled(&zone[s..e]).is_some()))
        {
            return Err(format!("zone '{}' still carries its old fill", plan.uuid));
        }
    }
    Ok(json!({
        "zones": prepared.zones.iter().map(ZonePlan::json).collect::<Vec<_>>(),
        "bytes_outside_planned_spans_unchanged": true,
    }))
}

pub(super) async fn handle(args: &Value, ctx: &ToolContext) -> anyhow::Result<CallToolResult> {
    let board = get_path(args, "board")?;
    let reference = match require_str(args, "reference") {
        Ok(reference) => reference.to_string(),
        Err(result) => return Ok(result),
    };
    let Some(object) = args["library_zones"].as_object() else {
        return Ok(crate::tools::invalid_arg(
            "library_zones",
            "must be an object",
        ));
    };
    if let Some(key) = object.keys().find(|key| !KEYS.contains(&key.as_str())) {
        return Ok(crate::tools::invalid_arg(
            &format!("library_zones.{key}"),
            "is not one of restore, expected_revision",
        ));
    }
    let restore = match object.get("restore") {
        None | Some(Value::Null) => false,
        Some(Value::Bool(restore)) => *restore,
        Some(_) => {
            return Ok(crate::tools::invalid_arg(
                "library_zones.restore",
                "must be true or false",
            ))
        }
    };
    let expected_revision = match object.get("expected_revision") {
        None | Some(Value::Null) => None,
        Some(Value::String(revision)) => Some(revision.as_str()),
        Some(_) => {
            return Ok(crate::tools::invalid_arg(
                "library_zones.expected_revision",
                "must be a string",
            ))
        }
    };
    let closure = match crate::tools::closed_board::prove_closed(ctx, &board, OPERATION).await? {
        Ok(closure) => closure,
        Err(refusal) => return Ok(refusal),
    };
    let original = read_consistent(&board)?;
    let resolve = |lib_id: &str| {
        resolve_footprint_source(lib_id, &board).map_err(|error| format!("{error:#}"))
    };
    let prepared = match prepare(&original, &reference, &resolve) {
        Ok(prepared) => prepared,
        Err(reason) => {
            return Ok(CallToolResult::error_kind(
                crate::mcp::error::ToolErrorKind::PlanBlocked {
                    operation: OPERATION.to_string(),
                    reasons: vec![reason.clone()],
                },
                format!("No footprint zone changed: {reason}. The board file was not modified."),
            ))
        }
    };
    let revision = crate::tools::closed_board::revision(
        OPERATION,
        &original,
        &json!({
            "reference": reference,
            "library": crate::tools::closed_board::sha256_hex(prepared.library.as_bytes()),
        }),
    );
    if expected_revision.is_some_and(|expected| expected != revision) {
        return Ok(crate::tools::closed_board::stale_revision(
            &board, OPERATION,
        ));
    }
    let changed = prepared.changed();
    let dry_run = !restore;
    let count = |state: &str| prepared.zones.iter().filter(|z| z.state == state).count();
    let mut body = json!({
        "status": match (changed, dry_run) {
            (false, _) => "noop",
            (true, true) => "ready",
            (true, false) => "applied",
        },
        "source": "saved",
        "footprint": prepared.footprint,
        "revision": revision,
        "zones": prepared.zones.iter().map(ZonePlan::json).collect::<Vec<_>>(),
        "misplaced": count("misplaced"),
        "in_place": count("in_place"),
        "not_in_library": count("not_in_library"),
        "board_closure": closure,
        "other_footprint_clauses": "unchanged byte-for-byte; no whole-footprint typed update",
        "note": "A misplaced zone is the library footprint's zone somewhere else; restore moves its outline to where KiCad saves it for this position, rotation and side, and drops its fill (refill zones in KiCad). A zone that is not_in_library is not the library's shape and is left alone.",
    });
    if changed && dry_run {
        body["next"] = json!(
            "Repeat with library_zones {\"restore\": true} (and this revision as expected_revision) to write."
        );
    }
    if changed && !dry_run {
        if let Err(refusal) =
            crate::tools::closed_board::prove_closed(ctx, &board, OPERATION).await?
        {
            return Ok(refusal);
        }
        match crate::tools::closed_board::write_and_read_back(
            &board,
            OPERATION,
            &original,
            &prepared.candidate,
            |text| verify(&original, text, &prepared, &reference),
        ) {
            Ok(write) => body["write"] = write,
            Err(refusal) => return Ok(refusal),
        }
    }
    Ok(CallToolResult::json(&body))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tools::pcb_board::board_mock::{ctx_talking_to, spawn_kicad_holding_board};

    /// One pad and an L-shaped keepout, so that no rotation or mirror of the
    /// outline is the outline again.
    const LIBRARY: &str = r#"(footprint "Keepout_L"
	(version 20241229)
	(generator "pcbnew")
	(layer "F.Cu")
	(property "Reference" "REF**" (at 0 -2 0) (layer "F.SilkS"))
	(attr smd)
	(pad "1" smd rect (at -1 0.5) (size 1 1) (layers "F.Cu" "F.Mask" "F.Paste"))
	(zone
		(net 0)
		(net_name "")
		(layer "F.Cu")
		(hatch full 0.508)
		(keepout (tracks allowed) (vias not_allowed) (pads allowed) (copperpour not_allowed) (footprints allowed))
		(fill (thermal_gap 0.5) (thermal_bridge_width 0.5))
		(polygon
			(pts
				(xy 0 0) (xy 3 0) (xy 3 1)
				(xy 1 1) (xy 1 2) (xy 0 2)
			)
		)
	)
)
"#;
    const AT_ORIGIN: &str = "(xy 0 0) (xy 3 0) (xy 3 1) (xy 1 1) (xy 1 2) (xy 0 2)";
    // The keepout as pcbnew 10.0.7 saves it for an instance of `LIBRARY`:
    /// `(at 80 60 30)` on the front.
    const AT_80_60_30: &str = "(xy 80 60) (xy 82.598076 58.5) (xy 83.098076 59.366025) (xy 81.366025 60.366025) (xy 81.866025 61.232051) (xy 81 61.732051)";
    /// Set to -90 on the front, then flipped top to bottom: `(at 110 70 90)`.
    const BACK_AT_110_70_90: &str =
        "(xy 110 70) (xy 110 67) (xy 109 67) (xy 109 69) (xy 108 69) (xy 108 70)";
    /// Set to 200 on the front, then flipped left to right: `(at 140 80 -20)`.
    const BACK_AT_140_80_M20: &str = "(xy 140 80) (xy 142.819078 81.02606) (xy 143.161098 80.086368) (xy 141.281713 79.402328) (xy 141.623733 78.462635) (xy 140.68404 78.120615)";

    fn instance(reference: &str, at: &str, layer: &str, fill: &str, pts: &str) -> String {
        format!(
            r#"	(footprint "Probe:Keepout_L"
		(layer "{layer}")
		(uuid "f0000000-0000-4000-8000-0000000000{reference}")
		(at {at})
		(property "Reference" "{reference}" (at 0 -2 0) (layer "F.SilkS"))
		(attr smd)
		(pad "1" smd rect (at -1 0.5) (size 1 1) (layers "{layer}") (property pad_prop_testpoint))
		(zone
			(net 0)
			(net_name "")
			(layer "{layer}")
			(uuid "a0000000-0000-4000-8000-0000000000{reference}")
			(hatch full 0.508)
			(keepout (tracks allowed) (vias not_allowed) (pads allowed) (copperpour not_allowed) (footprints allowed))
			{fill}
			(polygon
				(pts
					{pts}
				)
			)
		)
	)
"#
        )
    }

    const FILL: &str = "(fill (thermal_gap 0.5) (thermal_bridge_width 0.5))";

    fn board(footprints: &[String]) -> String {
        format!(
            "(kicad_pcb\n\t(version 20260206)\n\t(generator \"pcbnew\")\n{})\n",
            footprints.concat()
        )
    }

    fn library(lib_id: &str) -> Result<String, String> {
        match lib_id {
            "Probe:Keepout_L" => Ok(LIBRARY.to_string()),
            other => Err(format!("library of '{other}' not found")),
        }
    }

    #[test]
    fn a_misplaced_zone_becomes_byte_for_byte_what_kicad_saves_for_that_placement() {
        for (at, layer, kicad) in [
            ("80 60 30", "F.Cu", AT_80_60_30),
            ("110 70 90", "B.Cu", BACK_AT_110_70_90),
            ("140 80 -20", "B.Cu", BACK_AT_140_80_M20),
        ] {
            let damaged = board(&[
                instance("01", "50 50", "F.Cu", FILL, "(xy 9 9)"),
                instance("02", at, layer, FILL, AT_ORIGIN),
            ]);
            let prepared = prepare(&damaged, "02", &library).unwrap();
            assert!(prepared.changed());
            assert_eq!(prepared.zones.len(), 1);
            assert_eq!(prepared.zones[0].state, "misplaced");
            assert_eq!(
                prepared.candidate,
                board(&[
                    instance("01", "50 50", "F.Cu", FILL, "(xy 9 9)"),
                    instance("02", at, layer, FILL, kicad),
                ]),
                "{at} {layer}"
            );
            // Settled: the repaired board has nothing left to restore.
            let again = prepare(&prepared.candidate, "02", &library).unwrap();
            assert!(!again.changed());
            assert_eq!(again.zones[0].state, "in_place");
        }
    }

    #[test]
    fn a_restored_zone_loses_its_stale_fill_and_nothing_else() {
        let filled = "(fill yes (thermal_gap 0.5) (thermal_bridge_width 0.5))";
        let damaged = board(&[instance("02", "80 60 30", "F.Cu", filled, AT_ORIGIN)]).replace(
            "\t\t)\n\t)\n",
            "\t\t\t(filled_polygon\n\t\t\t\t(layer \"F.Cu\")\n\t\t\t\t(pts\n\t\t\t\t\t(xy 0.1 0.1) (xy 2.9 0.1) (xy 2.9 0.9)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n\t)\n",
        );
        assert!(damaged.contains("(filled_polygon"));
        let prepared = prepare(&damaged, "02", &library).unwrap();
        assert_eq!(prepared.zones[0].fills_dropped, 1);
        assert_eq!(
            prepared.candidate,
            board(&[instance("02", "80 60 30", "F.Cu", FILL, AT_80_60_30)])
        );
    }

    #[test]
    fn a_zone_in_place_and_a_zone_the_library_does_not_describe_are_left_alone() {
        let reshaped = "(xy 0 0) (xy 3 0) (xy 3 2) (xy 0 2)";
        for (pts, state) in [(AT_80_60_30, "in_place"), (reshaped, "not_in_library")] {
            let source = board(&[instance("02", "80 60 30", "F.Cu", FILL, pts)]);
            let prepared = prepare(&source, "02", &library).unwrap();
            assert!(!prepared.changed());
            assert_eq!(prepared.candidate, source);
            assert_eq!(prepared.zones[0].state, state);
            assert!(prepared.zones[0].after.is_none());
        }
    }

    #[test]
    fn an_outline_arc_of_the_library_is_kept_as_an_arc() {
        let with_arc = |lib_id: &str| {
            library(lib_id).map(|source| {
                source.replace(
                    "(xy 0 0) (xy 3 0) (xy 3 1)\n\t\t\t\t(xy 1 1) (xy 1 2) (xy 0 2)",
                    "(xy 0 0) (arc (start 3 0) (mid 3.5 0.5) (end 3 1)) (xy 0 1)",
                )
            })
        };
        let damaged = board(&[instance(
            "02",
            "10 10",
            "F.Cu",
            FILL,
            "(xy 0 0) (arc (start 3 0) (mid 3.5 0.5) (end 3 1)) (xy 0 1)",
        )]);
        let prepared = prepare(&damaged, "02", &with_arc).unwrap();
        assert!(prepared
            .candidate
            .contains("(xy 10 10) (arc (start 13 10) (mid 13.5 10.5) (end 13 11)) (xy 10 11)\n"));
    }

    #[test]
    fn a_footprint_that_cannot_be_judged_is_refused_with_the_reason() {
        let source = board(&[instance("02", "80 60 30", "F.Cu", FILL, AT_ORIGIN)]);
        let unknown = source.replace("Probe:Keepout_L", "Gone:Keepout_L");
        let error = prepare(&unknown, "02", &library).unwrap_err();
        assert!(
            error.contains("'Gone:Keepout_L' of '02' could not be read"),
            "{error}"
        );
        let error = prepare(&source, "09", &library).unwrap_err();
        assert!(error.contains("'09' was not found"), "{error}");
        let plain = "(kicad_pcb (version 20260206)\n\t(footprint \"Probe:Keepout_L\" (layer \"F.Cu\") (at 1 2)\n\t\t(property \"Reference\" \"R1\" (at 0 0) (layer \"F.SilkS\"))\n\t)\n)\n";
        let error = prepare(plain, "R1", &library).unwrap_err();
        assert!(error.contains("has no zone of its own"), "{error}");
    }

    fn body(result: &CallToolResult) -> Value {
        match &result.content[0] {
            crate::mcp::protocol::ToolContent::Text { text } => {
                serde_json::from_str(text).unwrap_or_else(|_| json!({ "text": text }))
            }
            other => panic!("expected text, got {other:?}"),
        }
    }

    async fn call(ctx: ToolContext, args: Value) -> (bool, Value) {
        let def = tools()
            .into_iter()
            .find(|tool| tool.name == "edit_component")
            .unwrap();
        assert!(def.input_validator.is_valid(&args), "schema refused {args}");
        let result = (def.handler)(&args, std::sync::Arc::new(ctx))
            .await
            .unwrap();
        (result.is_error, body(&result))
    }

    /// A project directory whose own library table resolves `Probe:Keepout_L`,
    /// and a board in it whose one footprint has its keepout at the origin.
    fn project(dir: &Path) -> (PathBuf, String) {
        let pretty = dir.join("Probe.pretty");
        std::fs::create_dir_all(&pretty).unwrap();
        std::fs::write(pretty.join("Keepout_L.kicad_mod"), LIBRARY).unwrap();
        std::fs::write(
            dir.join("fp-lib-table"),
            format!(
                "(fp_lib_table\n\t(version 7)\n\t(lib (name \"Probe\") (type \"KiCad\") (uri \"{}\") (options \"\") (descr \"\"))\n)\n",
                pretty.to_string_lossy()
            ),
        )
        .unwrap();
        let source = board(&[instance("02", "110 70 90", "B.Cu", FILL, AT_ORIGIN)]);
        let path = dir.join("board.kicad_pcb");
        std::fs::write(&path, &source).unwrap();
        (path, source)
    }

    #[tokio::test]
    async fn edit_component_plans_by_default_applies_on_restore_and_settles() {
        let tmp = tempfile::tempdir().unwrap();
        let (path, damaged) = project(tmp.path());
        let closed = || ctx_talking_to(String::new());
        let args = json!({ "board": path, "reference": "02", "library_zones": {} });

        let (error, plan) = call(closed(), args.clone()).await;
        assert!(!error, "{plan}");
        assert_eq!(plan["status"], "ready");
        assert_eq!(plan["misplaced"], 1);
        assert_eq!(plan["footprint"]["side"], "back");
        assert_eq!(plan["zones"][0]["state"], "misplaced");
        assert_eq!(plan["zones"][0]["kind"], "rule_area");
        assert_eq!(plan["zones"][0]["layers"], json!(["B.Cu"]));
        assert_eq!(
            plan["zones"][0]["before"]["center"],
            json!({ "x": 1.5, "y": 1.0 })
        );
        assert_eq!(
            plan["zones"][0]["after"]["bbox"],
            json!({ "x1": 108.0, "y1": 67.0, "x2": 110.0, "y2": 70.0 })
        );
        assert_eq!(std::fs::read_to_string(&path).unwrap(), damaged);

        let mut apply = args.clone();
        apply["library_zones"] = json!({ "restore": true, "expected_revision": "f".repeat(64) });
        let (error, stale) = call(closed(), apply.clone()).await;
        assert!(error);
        assert_eq!(stale["error"]["reason"], "stale_revision");
        assert_eq!(std::fs::read_to_string(&path).unwrap(), damaged);

        apply["library_zones"]["expected_revision"] = plan["revision"].clone();
        let (error, applied) = call(closed(), apply.clone()).await;
        assert!(!error, "{applied}");
        assert_eq!(applied["status"], "applied");
        assert_eq!(
            applied["write"]["persisted_readback"]["bytes_outside_planned_spans_unchanged"],
            true
        );
        assert_eq!(
            std::fs::read_to_string(&path).unwrap(),
            board(&[instance("02", "110 70 90", "B.Cu", FILL, BACK_AT_110_70_90)])
        );

        let (error, again) = call(closed(), args).await;
        assert!(!error, "{again}");
        assert_eq!(again["status"], "noop");
        assert_eq!(again["in_place"], 1);
    }

    #[tokio::test]
    async fn an_open_board_a_lock_and_a_mixed_or_malformed_request_change_nothing() {
        let tmp = tempfile::tempdir().unwrap();
        let (path, damaged) = project(tmp.path());
        let restore =
            json!({ "board": path, "reference": "02", "library_zones": { "restore": true } });

        let server = spawn_kicad_holding_board(&path, |_| None);
        let (error, open) = call(
            ctx_talking_to(server.address().to_string()),
            restore.clone(),
        )
        .await;
        assert!(error, "{open}");
        assert_eq!(open["error"]["kind"], "plan_blocked");
        drop(server);

        let lock = konnect_sexp::writer::kicad_editor_lock_path(&path).unwrap();
        std::fs::write(&lock, "own-test-lock").unwrap();
        let (error, locked) = call(ctx_talking_to(String::new()), restore.clone()).await;
        assert!(error, "{locked}");
        std::fs::remove_file(lock).unwrap();

        let mut mixed = restore.clone();
        mixed["attributes"] = json!({});
        let (error, _) = call(ctx_talking_to(String::new()), mixed).await;
        assert!(error);

        let result = handle(
            &json!({ "board": path, "reference": "02", "library_zones": { "move": true } }),
            &ctx_talking_to(String::new()),
        )
        .await
        .unwrap();
        assert!(result.is_error);
        assert_eq!(body(&result)["error"]["field"], "library_zones.move");

        assert_eq!(std::fs::read_to_string(&path).unwrap(), damaged);
    }
}
