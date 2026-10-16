//! Closed-board presentation of placed footprints' Reference fields: local
//! position, text angle, character width and height, and stroke width.
//!
//! Only three spans of each requested field change — its `(at …)`, and the
//! `(size …)` and `(thickness …)` inside its `(effects (font …))` — so the
//! field's UUID, text, layer, mirror, visibility and every other clause of the
//! footprint (pads, fabrication properties, nets, models, schematic links)
//! stay byte for byte. No typed footprint rebuild is involved.
//!
//! Conventions, as KiCad 10 saves a footprint field
//! (`PCB_IO_KICAD_SEXPR::format( const PCB_TEXT* )`, read back by
//! `parsePCB_TEXT_effects`):
//!
//! - `local_x`/`local_y` are the saved `(at x y …)`: the offset from the
//!   footprint anchor in the footprint's own unrotated frame, KiCad's Y down.
//!   A back-side footprint's frame is the flipped one, so a local offset is
//!   already mirrored there.
//! - `board_x`/`board_y` = anchor + the local offset turned by the footprint
//!   angle θ: x' = x·cos θ + y·sin θ, y' = −x·sin θ + y·cos θ.
//! - `angle_deg` is the text's board angle as KiCad saves it;
//!   `relative_angle_deg` is that minus the footprint angle.
//! - `(size H W)` is height then width.

use super::*;
use konnect_sexp::SexpNode;
use serde_json::{Map, Value};

use crate::tools::closed_board::only_spans_changed;

const TOLERANCE: f64 = 5e-7;
const MAX_TEXT_MM: f64 = 100.0;
const MAX_STROKE_MM: f64 = 25.0;
const MAX_LOCAL_MM: f64 = 1000.0;
const MAX_BOARD_MM: f64 = 10_000.0;
pub(crate) const MAX_FIELDS: usize = 5000;

pub(crate) const CONVENTION: &str = "local_x/local_y: the saved (at …) offset from the \
     footprint anchor in the footprint's unrotated frame (Y down; already mirrored for a \
     back-side footprint). board_x/board_y = anchor + local turned by the footprint angle θ \
     (x' = x·cos θ + y·sin θ, y' = −x·sin θ + y·cos θ). angle_deg: the text's board angle as \
     KiCad saves it; relative_angle_deg = angle_deg − footprint angle. width_mm/height_mm: \
     character size, saved as (size height width); stroke_mm: (thickness …).";

/// One requested Reference field. Every value is optional; none at all is an
/// inspection.
#[derive(Debug, Clone, Default, PartialEq)]
pub(crate) struct FieldRequest {
    pub reference: String,
    pub local_x: Option<f64>,
    pub local_y: Option<f64>,
    pub board_xy: Option<(f64, f64)>,
    pub angle_deg: Option<f64>,
    pub relative_angle_deg: Option<f64>,
    pub width_mm: Option<f64>,
    pub height_mm: Option<f64>,
    pub stroke_mm: Option<f64>,
}

impl FieldRequest {
    pub(crate) fn json(&self) -> Value {
        json!({
            "reference": self.reference,
            "local_x": self.local_x, "local_y": self.local_y,
            "board_xy": self.board_xy.map(|(x, y)| [x, y]),
            "angle_deg": self.angle_deg, "relative_angle_deg": self.relative_angle_deg,
            "width_mm": self.width_mm, "height_mm": self.height_mm, "stroke_mm": self.stroke_mm,
        })
    }
}

pub(crate) const FIELD_KEYS: [&str; 9] = [
    "local_x",
    "local_y",
    "board_x",
    "board_y",
    "angle_deg",
    "relative_angle_deg",
    "width_mm",
    "height_mm",
    "stroke_mm",
];

/// `(field, reason)` for a value that is not a valid request.
pub(crate) type ArgError = (String, String);

/// Read one field's values from `object`; `at` prefixes field names in errors.
pub(crate) fn field_request(
    reference: &str,
    object: &Map<String, Value>,
    at: &str,
) -> Result<FieldRequest, ArgError> {
    let name = |key: &str| {
        if at.is_empty() {
            key.to_string()
        } else {
            format!("{at}.{key}")
        }
    };
    let number = |key: &str, limit: f64, positive: bool| -> Result<Option<f64>, ArgError> {
        match object.get(key) {
            None | Some(Value::Null) => Ok(None),
            Some(value) => match value.as_f64() {
                Some(n) if n.is_finite() && n.abs() <= limit && (!positive || n > 0.0) => {
                    Ok(Some(n))
                }
                _ => Err((
                    name(key),
                    if positive {
                        format!("must be a finite number greater than 0 and at most {limit}")
                    } else {
                        format!("must be a finite number within ±{limit}")
                    },
                )),
            },
        }
    };
    let request = FieldRequest {
        reference: reference.to_string(),
        local_x: number("local_x", MAX_LOCAL_MM, false)?,
        local_y: number("local_y", MAX_LOCAL_MM, false)?,
        board_xy: match (
            number("board_x", MAX_BOARD_MM, false)?,
            number("board_y", MAX_BOARD_MM, false)?,
        ) {
            (Some(x), Some(y)) => Some((x, y)),
            (None, None) => None,
            _ => {
                return Err((
                    name("board_x"),
                    "board_x and board_y go together".to_string(),
                ))
            }
        },
        angle_deg: number("angle_deg", 3600.0, false)?,
        relative_angle_deg: number("relative_angle_deg", 3600.0, false)?,
        width_mm: number("width_mm", MAX_TEXT_MM, true)?,
        height_mm: number("height_mm", MAX_TEXT_MM, true)?,
        stroke_mm: number("stroke_mm", MAX_STROKE_MM, true)?,
    };
    if request.board_xy.is_some() && (request.local_x.is_some() || request.local_y.is_some()) {
        return Err((
            name("board_x"),
            "give the position as local_x/local_y or as board_x/board_y, not both".to_string(),
        ));
    }
    if request.angle_deg.is_some() && request.relative_angle_deg.is_some() {
        return Err((
            name("angle_deg"),
            "give angle_deg or relative_angle_deg, not both".to_string(),
        ));
    }
    Ok(request)
}

/// What the saved board says about one Reference field.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Observed {
    pub footprint: Option<String>,
    pub footprint_uuid: Option<String>,
    pub footprint_xy: (f64, f64),
    pub footprint_angle: f64,
    pub footprint_layer: String,
    pub kind: &'static str,
    pub field_uuid: Option<String>,
    pub layer: Option<String>,
    pub mirrored: bool,
    pub hidden: bool,
    pub local: (f64, f64),
    pub angle_deg: f64,
    pub width_mm: Option<f64>,
    pub height_mm: Option<f64>,
    pub stroke_mm: Option<f64>,
}

fn round_nm(value: f64) -> f64 {
    let rounded = (value * 1e6).round() / 1e6;
    if rounded == 0.0 {
        0.0
    } else {
        rounded
    }
}

fn normalize(angle: f64) -> f64 {
    let wrapped = angle.rem_euclid(360.0);
    let wrapped = if wrapped > 180.0 {
        wrapped - 360.0
    } else {
        wrapped
    };
    round_nm(wrapped)
}

impl Observed {
    fn board(&self) -> (f64, f64) {
        let (x, y) = konnect_sexp::geometry::rotate_about(
            self.local.0,
            self.local.1,
            self.footprint_xy.0,
            self.footprint_xy.1,
            self.footprint_angle,
        );
        (round_nm(x), round_nm(y))
    }

    pub(crate) fn json(&self, reference: &str) -> Value {
        let (board_x, board_y) = self.board();
        json!({
            "reference": reference,
            "footprint": self.footprint,
            "footprint_uuid": self.footprint_uuid,
            "footprint_x": self.footprint_xy.0,
            "footprint_y": self.footprint_xy.1,
            "footprint_angle_deg": self.footprint_angle,
            "footprint_layer": self.footprint_layer,
            "side": if self.footprint_layer == "B.Cu" { "back" } else { "front" },
            "field": self.kind,
            "field_uuid": self.field_uuid,
            "layer": self.layer,
            "mirrored": self.mirrored,
            "hidden": self.hidden,
            "local_x": self.local.0,
            "local_y": self.local.1,
            "board_x": board_x,
            "board_y": board_y,
            "angle_deg": self.angle_deg,
            "relative_angle_deg": normalize(self.angle_deg - self.footprint_angle),
            "width_mm": self.width_mm,
            "height_mm": self.height_mm,
            "stroke_mm": self.stroke_mm,
        })
    }
}

/// The byte spans one field's edit can touch, relative to the board.
struct FieldSpans {
    at: (usize, usize),
    font: (usize, usize),
    size: Option<(usize, usize)>,
    thickness: Option<(usize, usize)>,
}

fn head_of(block: &str) -> &str {
    sexp_block_tokens(block)
        .first()
        .map_or("", |&(s, e)| &block[s..e])
}

fn tagged(source: &str, parent: &str, tag: &str) -> Vec<(usize, usize)> {
    find_direct_child_blocks(source, parent)
        .into_iter()
        .filter(|&(s, e)| head_of(&source[s..e]) == tag)
        .collect()
}

fn one(
    spans: Vec<(usize, usize)>,
    offset: usize,
    what: &str,
) -> Result<Option<(usize, usize)>, String> {
    match spans.as_slice() {
        [] => Ok(None),
        [(s, e)] => Ok(Some((offset + s, offset + e))),
        _ => Err(format!("has more than one {what}")),
    }
}

/// Footprint spans by Reference, in board order.
fn footprints_by_reference(source: &str) -> Result<BTreeMap<String, Vec<(usize, usize)>>, String> {
    check_single_board_form(source)?;
    let mut out: BTreeMap<String, Vec<(usize, usize)>> = BTreeMap::new();
    for (start, end) in find_direct_child_blocks(source, "kicad_pcb") {
        if head_of(&source[start..end]) != "footprint" {
            continue;
        }
        let node = konnect_sexp::parse_sexp(&source[start..end]).map_err(|e| e.to_string())?;
        if let Some(reference) = footprint_reference(&node) {
            out.entry(reference).or_default().push((start, end));
        }
    }
    Ok(out)
}

fn read_field(source: &str, footprint: (usize, usize)) -> Result<(Observed, FieldSpans), String> {
    let block = &source[footprint.0..footprint.1];
    let node = konnect_sexp::parse_sexp(block).map_err(|e| e.to_string())?;
    let children = find_direct_child_blocks(block, "footprint");
    let fields: Vec<(usize, usize, &'static str)> = children
        .iter()
        .filter_map(|&(s, e)| {
            let text = &block[s..e];
            let tokens = sexp_block_tokens(text);
            let key = tokens.get(1).map(|&(a, b)| sexp_token_value(&text[a..b]))?;
            match (head_of(text), key.as_str()) {
                ("property", "Reference") => Some((s, e, "property")),
                ("fp_text", "reference") => Some((s, e, "fp_text")),
                _ => None,
            }
        })
        .collect();
    let [(field_start, field_end, kind)] = fields.as_slice() else {
        return Err(format!(
            "has {} Reference fields; exactly one is required",
            fields.len()
        ));
    };
    let field_offset = footprint.0 + field_start;
    let field_text = &block[*field_start..*field_end];
    let field_node = konnect_sexp::parse_sexp(field_text).map_err(|e| e.to_string())?;

    let anchor = one(tagged(block, "footprint", "at"), 0, "footprint (at …)")?
        .ok_or("has no footprint (at …)")?;
    let (fx, fy, fa, _) =
        at_components(&block[anchor.0..anchor.1]).map_err(|e| format!("{e:#}"))?;
    let layer = one(
        tagged(block, "footprint", "layer"),
        0,
        "footprint (layer …)",
    )?
    .ok_or("has no footprint (layer …)")?;
    let footprint_layer = konnect_sexp::parse_sexp(&block[layer.0..layer.1])
        .ok()
        .and_then(|n| n.get(1).and_then(SexpNode::as_str).map(str::to_string))
        .ok_or("has an unreadable footprint (layer …)")?;

    let parent = *kind;
    let at = one(
        tagged(field_text, parent, "at"),
        field_offset,
        "Reference (at …)",
    )?
    .ok_or("has a Reference field with no (at …)")?;
    let (lx, ly, angle, _) =
        at_components(&source[at.0..at.1]).map_err(|e| format!("Reference {e:#}"))?;
    if !tagged(field_text, parent, "render_cache").is_empty() {
        return Err(
            "has a Reference field drawn in an outline font with a saved (render_cache …), \
             which a new size or position would leave stale; change it in KiCad"
                .to_string(),
        );
    }
    let effects = one(
        tagged(field_text, parent, "effects"),
        field_offset,
        "Reference (effects …)",
    )?
    .ok_or("has a Reference field with no (effects …)")?;
    let effects_text = &source[effects.0..effects.1];
    let font = one(
        tagged(effects_text, "effects", "font"),
        effects.0,
        "Reference (font …)",
    )?
    .ok_or("has a Reference field with no (effects (font …))")?;
    let font_text = &source[font.0..font.1];
    let size = one(
        tagged(font_text, "font", "size"),
        font.0,
        "Reference (size …)",
    )?;
    let thickness = one(
        tagged(font_text, "font", "thickness"),
        font.0,
        "Reference (thickness …)",
    )?;
    let size_values = match size {
        None => None,
        Some((s, e)) => {
            let node = konnect_sexp::parse_sexp(&source[s..e]).map_err(|e| e.to_string())?;
            match (node.get_f64(1), node.get_f64(2), node.get(3)) {
                (Some(h), Some(w), None) if h.is_finite() && w.is_finite() => Some((h, w)),
                _ => return Err("has a malformed Reference (size …)".to_string()),
            }
        }
    };
    let stroke = match thickness {
        None => None,
        Some((s, e)) => {
            let node = konnect_sexp::parse_sexp(&source[s..e]).map_err(|e| e.to_string())?;
            match (node.get_f64(1), node.get(2)) {
                (Some(t), None) if t.is_finite() => Some(t),
                _ => return Err("has a malformed Reference (thickness …)".to_string()),
            }
        }
    };
    let effects_node = konnect_sexp::parse_sexp(effects_text).map_err(|e| e.to_string())?;
    let mirrored = effects_node.find_all("justify").iter().any(|j| {
        j.children()
            .unwrap_or(&[])
            .iter()
            .any(|c| c.as_str() == Some("mirror"))
    });
    let hidden = field_node
        .find("hide")
        .is_some_and(|h| h.get(1).and_then(SexpNode::as_str) != Some("no"))
        || effects_node
            .find("hide")
            .is_some_and(|h| h.get(1).and_then(SexpNode::as_str) != Some("no"))
        || field_node
            .children()
            .unwrap_or(&[])
            .iter()
            .any(|c| matches!(c, SexpNode::Atom(a) if a == "hide"));
    let observed = Observed {
        footprint: node.get(1).and_then(SexpNode::as_str).map(str::to_string),
        footprint_uuid: crate::tools::footprint_fabrication::footprint_identity(&node)
            .map(str::to_string),
        footprint_xy: (fx, fy),
        footprint_angle: fa,
        footprint_layer,
        kind,
        field_uuid: field_node
            .find_str("uuid")
            .or_else(|| field_node.find_str("tstamp"))
            .map(str::to_string),
        layer: field_node.find_str("layer").map(str::to_string),
        mirrored,
        hidden,
        local: (lx, ly),
        angle_deg: angle,
        width_mm: size_values.map(|(_, w)| w),
        height_mm: size_values.map(|(h, _)| h),
        stroke_mm: stroke,
    };
    Ok((
        observed,
        FieldSpans {
            at,
            font,
            size,
            thickness,
        },
    ))
}

fn mm(value: f64) -> String {
    let text = format!("{:.6}", round_nm(value));
    let text = text.trim_end_matches('0').trim_end_matches('.');
    if text == "-0" {
        "0".to_string()
    } else {
        text.to_string()
    }
}

fn differs(a: Option<f64>, b: f64) -> bool {
    a.is_none_or(|a| (a - b).abs() > TOLERANCE)
}

/// The observation a request would leave.
fn requested(observed: &Observed, want: &FieldRequest) -> Observed {
    let mut after = observed.clone();
    if let Some((bx, by)) = want.board_xy {
        let (lx, ly) = konnect_sexp::geometry::rotate_about(
            bx - observed.footprint_xy.0,
            by - observed.footprint_xy.1,
            0.0,
            0.0,
            -observed.footprint_angle,
        );
        after.local = (round_nm(lx), round_nm(ly));
    }
    if let Some(x) = want.local_x {
        after.local.0 = round_nm(x);
    }
    if let Some(y) = want.local_y {
        after.local.1 = round_nm(y);
    }
    if let Some(angle) = want.angle_deg {
        after.angle_deg = normalize(angle);
    }
    if let Some(relative) = want.relative_angle_deg {
        after.angle_deg = normalize(relative + observed.footprint_angle);
    }
    if let Some(w) = want.width_mm {
        after.width_mm = Some(round_nm(w));
    }
    if let Some(h) = want.height_mm {
        after.height_mm = Some(round_nm(h));
    }
    if let Some(t) = want.stroke_mm {
        after.stroke_mm = Some(round_nm(t));
    }
    after
}

#[derive(Debug)]
pub(crate) struct Prepared {
    pub candidate: String,
    pub edits: Vec<(usize, usize, usize)>,
    pub expected: Vec<(String, Observed)>,
    pub before: Vec<(String, Observed)>,
}

impl Prepared {
    pub(crate) fn changed_references(&self) -> Vec<&str> {
        self.before
            .iter()
            .zip(&self.expected)
            .filter(|((_, a), (_, b))| a != b)
            .map(|((reference, _), _)| reference.as_str())
            .collect()
    }
}

/// Plan every requested field against the saved board. Any member that
/// cannot be planned refuses the whole batch.
pub(crate) fn prepare(source: &str, requests: &[FieldRequest]) -> Result<Prepared, Vec<String>> {
    let index = footprints_by_reference(source).map_err(|e| vec![e])?;
    let mut reasons = Vec::new();
    let mut seen = HashSet::new();
    let mut edits = Vec::new();
    let mut changes = Vec::new();
    let mut before = Vec::new();
    let mut expected = Vec::new();
    for want in requests {
        let reference = want.reference.as_str();
        if !seen.insert(reference) {
            reasons.push(format!("'{reference}' is requested more than once"));
            continue;
        }
        let span = match index.get(reference).map(Vec::as_slice) {
            None | Some([]) => {
                reasons.push(format!("no footprint on the saved board is '{reference}'"));
                continue;
            }
            Some([span]) => *span,
            Some(spans) => {
                reasons.push(format!(
                    "{} footprints on the saved board are '{reference}'",
                    spans.len()
                ));
                continue;
            }
        };
        let (observed, spans) = match read_field(source, span) {
            Ok(found) => found,
            Err(reason) => {
                reasons.push(format!("'{reference}' {reason}"));
                continue;
            }
        };
        let after = requested(&observed, want);
        let wants_size = want.width_mm.is_some() || want.height_mm.is_some();
        if wants_size && (after.width_mm.is_none() || after.height_mm.is_none()) {
            reasons.push(format!(
                "'{reference}' has no saved (size …); give both width_mm and height_mm"
            ));
            continue;
        }
        let moved = differs(Some(observed.local.0), after.local.0)
            || differs(Some(observed.local.1), after.local.1)
            || differs(Some(observed.angle_deg), after.angle_deg);
        if moved {
            let (_, _, _, suffix) =
                at_components(&source[spans.at.0..spans.at.1]).expect("read above");
            let text = format!(
                "(at {} {} {}{})",
                mm(after.local.0),
                mm(after.local.1),
                mm(after.angle_deg),
                suffix.iter().map(|s| format!(" {s}")).collect::<String>()
            );
            edits.push((spans.at.0, spans.at.1, text.len()));
            changes.push(SexpEdit::replace(spans.at.0, spans.at.1, text));
        }
        let (h, w) = (after.height_mm, after.width_mm);
        if wants_size
            && (differs(observed.height_mm, h.unwrap()) || differs(observed.width_mm, w.unwrap()))
        {
            let text = format!("(size {} {})", mm(h.unwrap()), mm(w.unwrap()));
            match spans.size {
                Some((s, e)) => {
                    edits.push((s, e, text.len()));
                    changes.push(SexpEdit::replace(s, e, text));
                }
                None => unreachable!("refused above"),
            }
        }
        if let Some(t) = after.stroke_mm.filter(|t| differs(observed.stroke_mm, *t)) {
            let text = format!("(thickness {})", mm(t));
            match spans.thickness {
                Some((s, e)) => {
                    edits.push((s, e, text.len()));
                    changes.push(SexpEdit::replace(s, e, text));
                }
                None => {
                    // After the size when there is one, as KiCad writes it.
                    let at = spans.size.map_or(spans.font.0 + "(font".len(), |(_, e)| e);
                    let text = format!(" {text}");
                    edits.push((at, at, text.len()));
                    changes.push(SexpEdit::insert(at, text));
                }
            }
        }
        before.push((reference.to_string(), observed));
        expected.push((reference.to_string(), after));
    }
    if !reasons.is_empty() {
        return Err(reasons);
    }
    let candidate = apply_edits(source.to_string(), changes);
    verify(source, &candidate, &edits, &expected).map_err(|e| vec![e])?;
    Ok(Prepared {
        candidate,
        edits,
        expected,
        before,
    })
}

/// The candidate (or persisted) board differs from the original only inside
/// the planned spans, and every requested field reads back as planned.
pub(crate) fn verify(
    original: &str,
    candidate: &str,
    edits: &[(usize, usize, usize)],
    expected: &[(String, Observed)],
) -> Result<Value, String> {
    if !only_spans_changed(original, candidate, edits) {
        return Err("bytes outside the planned Reference-field spans changed".to_string());
    }
    let index = footprints_by_reference(candidate)?;
    for (reference, want) in expected {
        let [span] = index.get(reference).map(Vec::as_slice).unwrap_or(&[]) else {
            return Err(format!("'{reference}' is no longer one footprint"));
        };
        let (have, _) = read_field(candidate, *span).map_err(|e| format!("'{reference}' {e}"))?;
        let close = |a: Option<f64>, b: Option<f64>| match (a, b) {
            (Some(a), Some(b)) => (a - b).abs() <= TOLERANCE,
            (None, None) => true,
            _ => false,
        };
        let same = close(Some(have.local.0), Some(want.local.0))
            && close(Some(have.local.1), Some(want.local.1))
            && close(Some(have.angle_deg), Some(want.angle_deg))
            && close(have.width_mm, want.width_mm)
            && close(have.height_mm, want.height_mm)
            && close(have.stroke_mm, want.stroke_mm)
            && have.field_uuid == want.field_uuid
            && have.layer == want.layer
            && have.mirrored == want.mirrored
            && have.hidden == want.hidden
            && have.footprint_uuid == want.footprint_uuid;
        if !same {
            return Err(format!("'{reference}' does not read back as planned"));
        }
    }
    konnect_sexp::parse_sexp(candidate).map_err(|e| e.to_string())?;
    Ok(json!({
        "fields_match_plan": expected.len(),
        "bytes_outside_planned_spans_unchanged": true,
    }))
}

/// The single-reference form of `edit_component`: inspect with an empty
/// object, otherwise one guarded write.
pub(super) async fn handle(args: &Value, ctx: &ToolContext) -> anyhow::Result<CallToolResult> {
    let board = get_path(args, "board")?;
    let reference = match require_str(args, "reference") {
        Ok(reference) => reference.to_string(),
        Err(result) => return Ok(result),
    };
    let Some(object) = args["reference_field"].as_object() else {
        return Ok(crate::tools::invalid_arg(
            "reference_field",
            "must be an object",
        ));
    };
    if let Some(key) = object
        .keys()
        .find(|key| !FIELD_KEYS.contains(&key.as_str()))
    {
        return Ok(crate::tools::invalid_arg(
            &format!("reference_field.{key}"),
            "is not a Reference-field presentation value",
        ));
    }
    let request = match field_request(&reference, object, "reference_field") {
        Ok(request) => request,
        Err((field, reason)) => return Ok(crate::tools::invalid_arg(&field, &reason)),
    };
    let operation = "edit_component reference_field";
    let closure = match crate::tools::closed_board::prove_closed(ctx, &board, operation).await? {
        Ok(closure) => closure,
        Err(refusal) => return Ok(refusal),
    };
    let original = read_consistent(&board)?;
    let prepared = match prepare(&original, std::slice::from_ref(&request)) {
        Ok(prepared) => prepared,
        Err(reasons) => {
            return Ok(CallToolResult::error_kind(
                crate::mcp::error::ToolErrorKind::PlanBlocked {
                    operation: operation.to_string(),
                    reasons: reasons.clone(),
                },
                format!(
                    "No Reference field changed: {}. The board file was not modified.",
                    reasons.join("; ")
                ),
            ))
        }
    };
    let changed = !prepared.edits.is_empty();
    let mut body = json!({
        "reference": reference,
        "status": if changed { "applied" } else { "noop" },
        "source": "saved",
        "before": prepared.before[0].1.json(&reference),
        "after": prepared.expected[0].1.json(&reference),
        "convention": CONVENTION,
        "board_closure": closure,
        "other_footprint_clauses": "unchanged byte-for-byte; no whole-footprint typed update",
    });
    if changed {
        if let Err(refusal) =
            crate::tools::closed_board::prove_closed(ctx, &board, operation).await?
        {
            return Ok(refusal);
        }
        let expected = prepared.expected.clone();
        let edits = prepared.edits.clone();
        match crate::tools::closed_board::write_and_read_back(
            &board,
            operation,
            &original,
            &prepared.candidate,
            |text| verify(&original, text, &edits, &expected),
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

    /// Rotated front and back footprints, an older fp_text Reference, a field
    /// with no stroke, and every ambiguity the setter must refuse. Pads carry
    /// fabrication properties, nets and UUIDs, and models carry offsets, so a
    /// byte that moved outside the planned spans would show.
    const BOARD: &str = r#"(kicad_pcb (version 20260206)
	(footprint "Test:Bga" (layer "F.Cu") (uuid "11111111-1111-4111-8111-111111111111") (at 20 30 37)
		(property "Reference" "U1" (at 0 -2 37) (layer "F.SilkS") (uuid "21111111-1111-4111-8111-111111111111")
			(effects (font (size 0.65 0.65) (thickness 0.1))))
		(property "Value" "BGA" (at 0 2 37) (layer "F.Fab") (uuid "31111111-1111-4111-8111-111111111111") (effects (font (size 1 1) (thickness 0.15))))
		(attr smd preserve_me)
		(path "/keep/identity")
		(pad "A1" smd custom (at 1 2 37) (size 0.3 0.3) (layers "F.Cu") (property pad_prop_bga) (net 1 "GND")
			(primitives (gr_poly (pts (xy -1 -1) (xy 1 -1) (xy 1 1)) (width 0) (fill yes)))
			(uuid "41111111-1111-4111-8111-111111111111"))
		(model "keep.step" (offset (xyz 1 2 3)) (rotate (xyz 0 0 37))))
	(footprint "Resistor_SMD:R_0402" (layer "B.Cu") (uuid "12222222-2222-4222-8222-222222222222") (at 50 60 90)
		(property "Reference" "R4001" (at 0 1.5 90) (unlocked yes) (layer "B.SilkS") (uuid "22222222-2222-4222-8222-222222222222")
			(effects (font (size 1 1) (thickness 0.15)) (justify mirror)))
		(pad "1" smd roundrect (at -0.5 0 90) (size 0.6 0.5) (layers "B.Cu" "B.Paste" "B.Mask") (net 2 "VCC") (uuid "42222222-2222-4222-8222-222222222222"))
		(pad "2" smd roundrect (at 0.5 0 90) (size 0.6 0.5) (layers "B.Cu" "B.Paste" "B.Mask") (net 1 "GND") (uuid "43333333-3333-4333-8333-333333333333")))
	(footprint "Old:Cap" (layer "F.Cu") (tstamp "13333333-3333-4333-8333-333333333333") (at 5 5)
		(fp_text reference "C7" (at 1 0) (layer "F.SilkS") (tstamp "23333333-3333-4333-8333-333333333333")
			(effects (font (size 0.8 0.8))))
		(pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu")))
	(footprint "Test:Dup" (layer "F.Cu") (at 1 1) (property "Reference" "DUP" (at 0 0 0) (effects (font (size 1 1) (thickness 0.1)))))
	(footprint "Test:Dup" (layer "F.Cu") (at 2 2) (property "Reference" "DUP" (at 0 0 0) (effects (font (size 1 1) (thickness 0.1)))))
	(footprint "Test:Two" (layer "F.Cu") (at 3 3)
		(property "Reference" "X1" (at 0 0 0) (effects (font (size 1 1) (thickness 0.1))))
		(property "Reference" "X1" (at 0 1 0) (effects (font (size 1 1) (thickness 0.1)))))
	(footprint "Test:Ttf" (layer "F.Cu") (at 4 4)
		(property "Reference" "T1" (at 0 0 0) (effects (font (face "Arial") (size 1 1) (thickness 0.1)))
			(render_cache "T1" 0 (polygon (pts (xy 0 0) (xy 1 0) (xy 1 1))))))
	(footprint "Test:Bad" (layer "F.Cu") (at 6 6)
		(property "Reference" "B1" (at 0 0 0) (effects (font (size 1 1) (size 2 2) (thickness 0.1)))))
	(segment (start 1 2) (end 3 4) (width 0.2) (layer "F.Cu") (net 1) (uuid "51111111-1111-4111-8111-111111111111"))
)
"#;

    fn request(reference: &str, values: Value) -> FieldRequest {
        field_request(reference, values.as_object().unwrap(), "").unwrap()
    }

    #[test]
    fn three_spans_change_and_everything_else_is_byte_for_byte() {
        let prepared = prepare(
            BOARD,
            &[request(
                "U1",
                json!({ "local_x": 1.25, "local_y": -2.5, "angle_deg": 127,
                        "width_mm": 1.0, "height_mm": 1.0, "stroke_mm": 0.15 }),
            )],
        )
        .unwrap();
        let expected = BOARD
            .replacen(
                "(at 0 -2 37) (layer \"F.SilkS\")",
                "(at 1.25 -2.5 127) (layer \"F.SilkS\")",
                1,
            )
            .replacen(
                "(effects (font (size 0.65 0.65) (thickness 0.1)))",
                "(effects (font (size 1 1) (thickness 0.15)))",
                1,
            );
        assert_eq!(prepared.candidate, expected);
        assert_eq!(prepared.edits.len(), 3);
        assert_eq!(prepared.changed_references(), ["U1"]);
        let after = prepared.expected[0].1.json("U1");
        assert_eq!(after["field_uuid"], "21111111-1111-4111-8111-111111111111");
        assert_eq!(
            after["footprint_uuid"],
            "11111111-1111-4111-8111-111111111111"
        );
        assert_eq!(after["relative_angle_deg"], 90.0);

        // The same request against its own result is a no-op.
        let again = prepare(
            &prepared.candidate,
            &[request(
                "U1",
                json!({ "local_x": 1.25, "local_y": -2.5, "relative_angle_deg": 90,
                        "width_mm": 1, "height_mm": 1, "stroke_mm": 0.15 }),
            )],
        )
        .unwrap();
        assert!(again.edits.is_empty());
        assert_eq!(again.candidate, prepared.candidate);
        assert!(again.changed_references().is_empty());
    }

    #[test]
    fn board_positions_turn_with_rotated_and_back_side_footprints() {
        // R4001: B.Cu at (50, 60, 90°). Board (52, 60) is local (0, 2): the
        // inverse of x' = x·cos θ + y·sin θ, y' = −x·sin θ + y·cos θ.
        let prepared = prepare(
            BOARD,
            &[request(
                "R4001",
                json!({ "board_x": 52, "board_y": 60, "relative_angle_deg": 0, "height_mm": 1.0, "width_mm": 0.9 }),
            )],
        )
        .unwrap();
        let after = prepared.expected[0].1.json("R4001");
        assert_eq!(after["local_x"], 0.0);
        assert_eq!(after["local_y"], 2.0);
        assert_eq!(after["board_x"], 52.0);
        assert_eq!(after["board_y"], 60.0);
        assert_eq!(after["angle_deg"], 90.0);
        assert_eq!(after["side"], "back");
        assert_eq!(
            after["mirrored"], true,
            "the mirror is the field's own and stays"
        );
        assert_eq!(after["layer"], "B.SilkS");
        // The (unlocked yes) clause and the justify mirror survive verbatim.
        assert!(prepared.candidate.contains(
            "(property \"Reference\" \"R4001\" (at 0 2 90) (unlocked yes) (layer \"B.SilkS\")"
        ));
        assert!(prepared
            .candidate
            .contains("(effects (font (size 1 0.9) (thickness 0.15)) (justify mirror))"));

        // U1 at 37°: its saved (0, -2) lands at the anchor plus the turned offset.
        let inspected = prepare(BOARD, &[request("U1", json!({}))]).unwrap();
        let before = inspected.before[0].1.json("U1");
        let theta = 37f64.to_radians();
        assert!((before["board_x"].as_f64().unwrap() - (20.0 - 2.0 * theta.sin())).abs() < 1e-6);
        assert!((before["board_y"].as_f64().unwrap() - (30.0 - 2.0 * theta.cos())).abs() < 1e-6);
        assert!(inspected.edits.is_empty());
    }

    #[test]
    fn an_older_fp_text_reference_and_a_missing_stroke_are_edited_in_place() {
        let prepared = prepare(
            BOARD,
            &[request("C7", json!({ "stroke_mm": 0.12, "local_x": 1.5 }))],
        )
        .unwrap();
        assert!(prepared.candidate.contains(
            "(fp_text reference \"C7\" (at 1.5 0 0) (layer \"F.SilkS\") (tstamp \"23333333-3333-4333-8333-333333333333\")"
        ));
        assert!(prepared
            .candidate
            .contains("(effects (font (size 0.8 0.8) (thickness 0.12))))"));
        assert_eq!(prepared.expected[0].1.json("C7")["field"], "fp_text");
    }

    #[test]
    fn one_bad_member_refuses_the_whole_batch() {
        let good = request("U1", json!({ "height_mm": 1, "width_mm": 1 }));
        for (bad, why) in [
            (request("NOPE", json!({ "height_mm": 1 })), "no footprint"),
            (request("DUP", json!({ "height_mm": 1 })), "2 footprints"),
            (
                request("X1", json!({ "height_mm": 1 })),
                "2 Reference fields",
            ),
            (request("T1", json!({ "height_mm": 1 })), "render_cache"),
            (
                request("B1", json!({ "height_mm": 1 })),
                "more than one Reference (size",
            ),
            (request("C7", json!({ "local_x": 0 })), "never"),
            (good.clone(), "more than once"),
        ] {
            let result = prepare(BOARD, &[good.clone(), bad.clone()]);
            match (result, why) {
                (Ok(_), "never") => {}
                (Ok(_), _) => panic!("{bad:?} was accepted"),
                (Err(reasons), why) => assert!(reasons.join(" ").contains(why), "{reasons:?}"),
            }
        }
        // An fp_text with no saved size cannot take only one dimension.
        let sizeless = BOARD.replace("(effects (font (size 0.8 0.8)))", "(effects (font))");
        assert!(
            prepare(&sizeless, &[request("C7", json!({ "height_mm": 1 }))]).unwrap_err()[0]
                .contains("no saved (size")
        );
    }

    #[test]
    fn malformed_or_conflicting_values_are_refused_before_planning() {
        for values in [
            json!({ "height_mm": 0 }),
            json!({ "width_mm": -1 }),
            json!({ "stroke_mm": 26 }),
            json!({ "local_x": 1e9 }),
            json!({ "board_x": 1 }),
            json!({ "board_x": 1, "board_y": 2, "local_x": 0 }),
            json!({ "angle_deg": 0, "relative_angle_deg": 0 }),
            json!({ "height_mm": "1" }),
        ] {
            assert!(
                field_request("U1", values.as_object().unwrap(), "").is_err(),
                "{values}"
            );
        }
    }

    // ─── Through the handlers ────────────────────────────────────────────────

    fn body(result: &CallToolResult) -> Value {
        match &result.content[0] {
            crate::mcp::protocol::ToolContent::Text { text } => {
                serde_json::from_str(text).unwrap_or_else(|_| json!({ "text": text }))
            }
            other => panic!("expected text, got {other:?}"),
        }
    }

    async fn batch(ctx: ToolContext, args: Value) -> (bool, Value) {
        let def = crate::tools::placement::tools()
            .into_iter()
            .find(|tool| tool.name == "set_reference_field_presentations")
            .unwrap();
        assert!(def.input_validator.is_valid(&args), "schema refused {args}");
        let result = (def.handler)(&args, std::sync::Arc::new(ctx))
            .await
            .unwrap();
        (result.is_error, body(&result))
    }

    #[tokio::test]
    async fn the_batch_dry_runs_applies_once_and_is_idempotent() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("labels.kicad_pcb");
        std::fs::write(&board, BOARD).unwrap();
        let fields = json!([
            { "reference": "U1", "height_mm": 1, "width_mm": 1, "stroke_mm": 0.15, "relative_angle_deg": 0 },
            { "reference": "R4001", "board_x": 52, "board_y": 60 },
            { "reference": "C7", "stroke_mm": 0.15, "height_mm": 0.8, "width_mm": 0.8 }
        ]);
        let args = json!({ "board": board, "fields": fields });
        let (error, plan) = batch(ctx_talking_to(String::new()), args.clone()).await;
        assert!(!error, "{plan}");
        assert_eq!(plan["status"], "ready");
        assert_eq!(plan["changed_count"], 3);
        assert_eq!(std::fs::read_to_string(&board).unwrap(), BOARD);

        let mut apply = args.clone();
        apply["dry_run"] = json!(false);
        apply["expected_revision"] = json!("f".repeat(64));
        let (error, stale) = batch(ctx_talking_to(String::new()), apply.clone()).await;
        assert!(error);
        assert_eq!(stale["error"]["reason"], "stale_revision");
        assert_eq!(std::fs::read_to_string(&board).unwrap(), BOARD);

        apply["expected_revision"] = plan["revision"].clone();
        let (error, applied) = batch(ctx_talking_to(String::new()), apply.clone()).await;
        assert!(!error, "{applied}");
        assert_eq!(applied["status"], "applied");
        assert_eq!(
            applied["write"]["persisted_readback"]["fields_match_plan"],
            3
        );
        let written = std::fs::read_to_string(&board).unwrap();
        for kept in [
            "(property pad_prop_bga) (net 1 \"GND\")",
            "(model \"keep.step\" (offset (xyz 1 2 3)) (rotate (xyz 0 0 37)))",
            "(property \"Value\" \"BGA\" (at 0 2 37)",
            "(uuid \"51111111-1111-4111-8111-111111111111\")",
        ] {
            assert!(written.contains(kept), "{kept}");
        }

        // Independent readback through edit_component's inspection form.
        let inspected = handle(
            &json!({ "board": board, "reference": "R4001", "reference_field": {} }),
            &ctx_talking_to(String::new()),
        )
        .await
        .unwrap();
        let inspected = body(&inspected);
        assert_eq!(inspected["status"], "noop", "{inspected}");
        assert_eq!(inspected["after"]["board_x"], 52.0);
        assert_eq!(inspected["after"]["local_y"], 2.0);
        assert_eq!(std::fs::read_to_string(&board).unwrap(), written);

        let (error, again) = batch(ctx_talking_to(String::new()), args).await;
        assert!(!error);
        assert_eq!(again["status"], "noop");
        assert_eq!(again["changed_count"], 0);
        assert_eq!(again["fields"], json!([]), "report 'changed' lists nothing");
    }

    #[tokio::test]
    async fn fields_file_and_report_file_carry_large_batches() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("labels.kicad_pcb");
        std::fs::write(&board, BOARD).unwrap();
        let fields = tmp.path().join("fields.json");
        std::fs::write(
            &fields,
            json!([{ "reference": "U1" }, { "reference": "R4001", "board_x": 52, "board_y": 60 }])
                .to_string(),
        )
        .unwrap();
        let report = tmp.path().join("report.json");
        let args = json!({ "board": board, "fields_file": fields, "report_file": report, "report": "all" });
        let (error, plan) = batch(ctx_talking_to(String::new()), args.clone()).await;
        assert!(!error, "{plan}");
        assert_eq!(
            plan["fields"],
            Value::Null,
            "the inline answer drops the list"
        );
        assert_eq!(plan["requested_count"], 2);
        let written: Value =
            serde_json::from_str(&std::fs::read_to_string(&report).unwrap()).unwrap();
        assert_eq!(written["fields"].as_array().unwrap().len(), 2);
        assert_eq!(written["revision"], plan["revision"]);

        let mut apply = args.clone();
        apply["dry_run"] = json!(false);
        apply["expected_revision"] = plan["revision"].clone();
        let (error, applied) = batch(ctx_talking_to(String::new()), apply).await;
        assert!(!error, "{applied}");
        assert_eq!(applied["status"], "applied");
        let written: Value =
            serde_json::from_str(&std::fs::read_to_string(&report).unwrap()).unwrap();
        assert_eq!(written["status"], "applied");

        let both =
            json!({ "board": board, "fields_file": fields, "fields": [{ "reference": "U1" }] });
        let (error, refused) = batch(ctx_talking_to(String::new()), both).await;
        assert!(error);
        assert!(refused.to_string().contains("fields_file"), "{refused}");
        std::fs::write(&fields, "[not json").unwrap();
        let (error, refused) = batch(
            ctx_talking_to(String::new()),
            json!({ "board": board, "fields_file": fields }),
        )
        .await;
        assert!(error);
        assert!(
            refused.to_string().contains("cannot be read as JSON"),
            "{refused}"
        );
    }

    #[tokio::test]
    async fn open_locked_or_partially_invalid_batches_write_nothing() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("labels.kicad_pcb");
        std::fs::write(&board, BOARD).unwrap();
        let args = json!({ "board": board, "fields": [{ "reference": "U1", "height_mm": 1, "width_mm": 1 }] });

        let server = spawn_kicad_holding_board(&board, |_| None);
        let (error, refused) =
            batch(ctx_talking_to(server.address().to_string()), args.clone()).await;
        assert!(error);
        assert_eq!(
            refused["error"]["reasons"][0], "board_open_in_kicad",
            "{refused}"
        );
        let single = handle(
            &json!({ "board": board, "reference": "U1", "reference_field": { "height_mm": 1 } }),
            &ctx_talking_to(server.address().to_string()),
        )
        .await
        .unwrap();
        assert!(single.is_error);
        drop(server);

        let lock = konnect_sexp::writer::kicad_editor_lock_path(&board).unwrap();
        std::fs::write(&lock, "own-test-lock").unwrap();
        let (error, refused) = batch(ctx_talking_to(String::new()), args.clone()).await;
        assert!(error);
        assert_eq!(
            refused["error"]["reason"], "kicad_lock_present",
            "{refused}"
        );
        std::fs::remove_file(&lock).unwrap();

        let mixed = json!({ "board": board, "fields": [
            { "reference": "U1", "height_mm": 1, "width_mm": 1 },
            { "reference": "DUP", "height_mm": 1 }
        ] });
        let (error, refused) = batch(ctx_talking_to(String::new()), mixed).await;
        assert!(error);
        assert_eq!(refused["error"]["kind"], "plan_blocked", "{refused}");
        let duplicate = json!({ "board": board, "fields": [
            { "reference": "U1", "height_mm": 1 }, { "reference": "U1", "width_mm": 1 }
        ] });
        let (error, _) = batch(ctx_talking_to(String::new()), duplicate).await;
        assert!(error);
        assert_eq!(
            std::fs::read_to_string(&board).unwrap(),
            BOARD,
            "nothing was written"
        );
    }
}
