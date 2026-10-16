//! `add_board_graphics` — many board-level lines, rectangles and texts on one
//! non-copper layer in a single write.
//!
//! Silkscreen group boxes, assembly notes and fabrication drawings are made of
//! hundreds of plain strokes. `add_board_text` places one string per call and
//! nothing else draws a line off Edge.Cuts, so this takes the whole batch,
//! inline or from a JSON file, validates every member before anything is
//! written, and creates it in one KiCad `CreateItems` (one undo step). With no
//! live KiCad holding the board it appends the same items to the saved file.

use crate::mcp::protocol::CallToolResult;
use crate::tool;
use crate::tools::pcb_board::{attempt_ipc_write, BoardWrite};
use crate::tools::{get_path, invalid_arg, ToolContext, ToolDef};
use konnect_ipc::builders;
use konnect_ipc::gen::kiapi::board::types::BoardLayer;
use konnect_sexp::writer::{apply_edits, new_uuid, write_atomic, SexpEdit};
use serde_json::json;

/// Members one call may carry, across lines, rectangles and texts.
pub(crate) const MAX_ITEMS: usize = 5000;
const DEFAULT_WIDTH_MM: f64 = 0.15;
const MAX_WIDTH_MM: f64 = 10.0;
const DEFAULT_TEXT_SIZE_MM: f64 = 1.0;
const MAX_TEXT_SIZE_MM: f64 = 50.0;
/// Font stroke as a fraction of the character height, as `add_board_text` uses.
const TEXT_STROKE_RATIO: f64 = 0.15;

#[derive(Debug, Clone, PartialEq)]
struct Segment {
    x1: f64,
    y1: f64,
    x2: f64,
    y2: f64,
}

#[derive(Debug, Clone, PartialEq)]
struct Text {
    text: String,
    x: f64,
    y: f64,
    size: f64,
    angle: f64,
}

/// A validated request: everything below is ready to write.
#[derive(Debug, Clone, PartialEq)]
struct Batch {
    layer: String,
    width: f64,
    lines: Vec<Segment>,
    rects: Vec<Segment>,
    texts: Vec<Text>,
}

impl Batch {
    fn mirrored(&self) -> bool {
        self.layer.starts_with("B.")
    }

    fn counts(&self) -> serde_json::Value {
        json!({
            "lines": self.lines.len(),
            "rects": self.rects.len(),
            "texts": self.texts.len(),
        })
    }

    fn stroke(size: f64) -> f64 {
        (size * TEXT_STROKE_RATIO * 1000.0).round() / 1000.0
    }

    fn ipc_items(&self) -> Vec<prost_types::Any> {
        const SHAPE: &str = "kiapi.board.types.BoardGraphicShape";
        let mut items = Vec::with_capacity(self.lines.len() + self.rects.len() + self.texts.len());
        for s in &self.lines {
            let shape = builders::board_segment(&self.layer, self.width, s.x1, s.y1, s.x2, s.y2);
            items.push(builders::pack_any(&shape, SHAPE));
        }
        for r in &self.rects {
            let shape =
                builders::board_rectangle(&self.layer, self.width, r.x1, r.y1, r.x2, r.y2, false);
            items.push(builders::pack_any(&shape, SHAPE));
        }
        for t in &self.texts {
            let text = builders::board_text_with_stroke_width(
                &self.layer,
                &t.text,
                t.x,
                t.y,
                t.size,
                Self::stroke(t.size),
                t.angle,
                self.mirrored(),
            );
            items.push(builders::pack_any(&text, "kiapi.board.types.BoardText"));
        }
        items
    }

    fn sexp(&self) -> String {
        let mut out = String::new();
        let (layer, w) = (&self.layer, self.width);
        for s in &self.lines {
            out.push_str(&format!(
                "\n  (gr_line\n    (start {} {})\n    (end {} {})\n    \
                 (stroke (width {w}) (type solid))\n    (layer \"{layer}\")\n    (uuid \"{}\")\n  )",
                s.x1,
                s.y1,
                s.x2,
                s.y2,
                new_uuid()
            ));
        }
        for r in &self.rects {
            out.push_str(&format!(
                "\n  (gr_rect\n    (start {} {})\n    (end {} {})\n    \
                 (stroke (width {w}) (type solid))\n    (fill no)\n    (layer \"{layer}\")\n    \
                 (uuid \"{}\")\n  )",
                r.x1,
                r.y1,
                r.x2,
                r.y2,
                new_uuid()
            ));
        }
        let justify = if self.mirrored() {
            " (justify mirror)"
        } else {
            ""
        };
        for t in &self.texts {
            let escaped = t.text.replace('\\', "\\\\").replace('"', "\\\"");
            out.push_str(&format!(
                "\n  (gr_text \"{escaped}\"\n    (at {} {} {})\n    (layer \"{layer}\")\n    \
                 (uuid \"{}\")\n    (effects (font (size {} {}) (thickness {})){justify})\n  )",
                t.x,
                t.y,
                t.angle,
                new_uuid(),
                t.size,
                t.size,
                Self::stroke(t.size)
            ));
        }
        out
    }
}

type Refusal = (String, String);

fn refuse(field: impl Into<String>, reason: impl Into<String>) -> Refusal {
    (field.into(), reason.into())
}

fn number(
    object: &serde_json::Map<String, serde_json::Value>,
    at: &str,
    key: &str,
) -> Result<f64, Refusal> {
    match object.get(key).and_then(serde_json::Value::as_f64) {
        Some(value) if value.is_finite() => Ok(value),
        _ => Err(refuse(format!("{at}.{key}"), "must be a finite number")),
    }
}

fn optional_number(
    object: &serde_json::Map<String, serde_json::Value>,
    at: &str,
    key: &str,
    default: f64,
) -> Result<f64, Refusal> {
    match object.get(key) {
        None | Some(serde_json::Value::Null) => Ok(default),
        Some(_) => number(object, at, key),
    }
}

fn members<'a>(
    source: &'a serde_json::Value,
    key: &str,
) -> Result<&'a [serde_json::Value], Refusal> {
    match source.get(key) {
        None | Some(serde_json::Value::Null) => Ok(&[]),
        Some(serde_json::Value::Array(items)) => Ok(items.as_slice()),
        Some(_) => Err(refuse(key, "must be an array")),
    }
}

fn object<'a>(
    value: &'a serde_json::Value,
    at: &str,
    allowed: &[&str],
) -> Result<&'a serde_json::Map<String, serde_json::Value>, Refusal> {
    let Some(object) = value.as_object() else {
        return Err(refuse(at, "must be an object"));
    };
    if let Some(key) = object.keys().find(|key| !allowed.contains(&key.as_str())) {
        return Err(refuse(format!("{at}.{key}"), "is not a known member key"));
    }
    Ok(object)
}

fn segments(source: &serde_json::Value, key: &str) -> Result<Vec<Segment>, Refusal> {
    let mut out = Vec::new();
    for (index, item) in members(source, key)?.iter().enumerate() {
        let at = format!("{key}[{index}]");
        let o = object(item, &at, &["x1", "y1", "x2", "y2"])?;
        let s = Segment {
            x1: number(o, &at, "x1")?,
            y1: number(o, &at, "y1")?,
            x2: number(o, &at, "x2")?,
            y2: number(o, &at, "y2")?,
        };
        let degenerate = if key == "rects" {
            s.x1 == s.x2 || s.y1 == s.y2
        } else {
            s.x1 == s.x2 && s.y1 == s.y2
        };
        if degenerate {
            let what = if key == "rects" {
                "zero area"
            } else {
                "zero length"
            };
            return Err(refuse(at, format!("has {what}")));
        }
        out.push(s);
    }
    Ok(out)
}

fn texts(source: &serde_json::Value) -> Result<Vec<Text>, Refusal> {
    let mut out = Vec::new();
    for (index, item) in members(source, "texts")?.iter().enumerate() {
        let at = format!("texts[{index}]");
        let o = object(item, &at, &["text", "x", "y", "size", "angle"])?;
        let text = match o.get("text").and_then(serde_json::Value::as_str) {
            Some(text) if !text.trim().is_empty() => text.to_string(),
            _ => return Err(refuse(format!("{at}.text"), "must be a non-empty string")),
        };
        let size = optional_number(o, &at, "size", DEFAULT_TEXT_SIZE_MM)?;
        if !(size > 0.0 && size <= MAX_TEXT_SIZE_MM) {
            return Err(refuse(
                format!("{at}.size"),
                format!("must be above 0 and at most {MAX_TEXT_SIZE_MM} mm"),
            ));
        }
        out.push(Text {
            text,
            x: number(o, &at, "x")?,
            y: number(o, &at, "y")?,
            size,
            angle: optional_number(o, &at, "angle", 0.0)?,
        });
    }
    Ok(out)
}

/// The layer must be one KiCad names and must not be copper: copper graphics
/// carry a net and are routing, which this tool does not do.
fn layer(args: &serde_json::Value) -> Result<String, Refusal> {
    let Some(name) = args.get("layer").and_then(serde_json::Value::as_str) else {
        return Err(refuse("layer", "is required"));
    };
    let id = builders::layer_from_name(name);
    if id == BoardLayer::BlUndefined {
        return Err(refuse(
            "layer",
            format!("'{name}' is not a KiCad board layer name"),
        ));
    }
    if name.ends_with(".Cu") {
        return Err(refuse(
            "layer",
            format!("'{name}' is copper; draw copper with the routing tools"),
        ));
    }
    Ok(name.to_string())
}

/// Parse and validate the whole request. `file` is the parsed `graphics_file`.
fn batch(args: &serde_json::Value, file: Option<&serde_json::Value>) -> Result<Batch, Refusal> {
    let layer = layer(args)?;
    let width = match args.get("width") {
        None | Some(serde_json::Value::Null) => DEFAULT_WIDTH_MM,
        Some(value) => match value.as_f64() {
            Some(w) if w > 0.0 && w <= MAX_WIDTH_MM => w,
            _ => {
                return Err(refuse(
                    "width",
                    format!("must be above 0 and at most {MAX_WIDTH_MM} mm"),
                ))
            }
        },
    };
    let inline = ["lines", "rects", "texts"]
        .iter()
        .any(|key| args.get(*key).is_some_and(|value| !value.is_null()));
    let source = match (file, inline) {
        (Some(_), true) => {
            return Err(refuse(
                "graphics_file",
                "give the members inline or in graphics_file, not both",
            ))
        }
        (Some(file), false) => {
            if !file.is_object() {
                return Err(refuse(
                    "graphics_file",
                    "must hold a JSON object with lines, rects and/or texts",
                ));
            }
            if let Some(key) = file.as_object().and_then(|o| {
                o.keys()
                    .find(|k| !["lines", "rects", "texts"].contains(&k.as_str()))
            }) {
                return Err(refuse(
                    "graphics_file",
                    format!("key '{key}' is not lines, rects or texts"),
                ));
            }
            file
        }
        (None, _) => args,
    };
    let batch = Batch {
        layer,
        width,
        lines: segments(source, "lines")?,
        rects: segments(source, "rects")?,
        texts: texts(source)?,
    };
    let total = batch.lines.len() + batch.rects.len() + batch.texts.len();
    if total == 0 {
        return Err(refuse(
            "lines",
            "at least one line, rect or text is required",
        ));
    }
    if total > MAX_ITEMS {
        return Err(refuse(
            "lines",
            format!("{total} members exceed the {MAX_ITEMS} one call may carry"),
        ));
    }
    Ok(batch)
}

pub(crate) fn tool() -> ToolDef {
    let point = |what: &str| json!({ "type": "number", "description": what });
    let segment = json!({
        "type": "object",
        "additionalProperties": false,
        "properties": {
            "x1": point("Start (or corner) X, mm"), "y1": point("Start (or corner) Y, mm"),
            "x2": point("End (or opposite corner) X, mm"), "y2": point("End (or opposite corner) Y, mm")
        },
        "required": ["x1", "y1", "x2", "y2"]
    });
    tool!(
        "add_board_graphics",
        "Add many board-level lines, unfilled rectangles and texts to one non-copper layer \
         (silkscreen, fab, courtyard, user layers, Edge.Cuts) in a single write: one KiCad \
         CreateItems and one undo step when KiCad holds the board, otherwise one append to the \
         saved file. Members come inline (lines/rects/texts) or from 'graphics_file', a JSON \
         object with the same keys, for batches too large to pass inline. Every member is \
         validated before anything is written; one bad member refuses the whole batch. Texts on \
         a B.* layer are mirrored, as KiCad shows back-side text. Text stroke is 15% of its \
         height. Copper is refused: use the routing tools.",
        json!({
            "type": "object",
            "properties": {
                "board": { "type": "string", "description": "Path to .kicad_pcb file" },
                "layer": { "type": "string", "description": "Non-copper layer, e.g. 'F.SilkS', 'B.Fab', 'User.Drawings'" },
                "width": { "type": "number", "exclusiveMinimum": 0, "maximum": MAX_WIDTH_MM, "default": DEFAULT_WIDTH_MM, "description": "Line and rectangle stroke width, mm" },
                "lines": { "type": "array", "items": segment.clone() },
                "rects": { "type": "array", "items": segment, "description": "Unfilled rectangles by opposite corners" },
                "texts": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "additionalProperties": false,
                        "properties": {
                            "text": { "type": "string" },
                            "x": point("Text anchor X (centre), mm"),
                            "y": point("Text anchor Y (centre), mm"),
                            "size": { "type": "number", "exclusiveMinimum": 0, "maximum": MAX_TEXT_SIZE_MM, "default": DEFAULT_TEXT_SIZE_MM, "description": "Character height and width, mm" },
                            "angle": { "type": "number", "default": 0, "description": "Rotation, degrees" }
                        },
                        "required": ["text", "x", "y"]
                    }
                },
                "graphics_file": { "type": "string", "description": "Path to a JSON object {lines, rects, texts} used instead of the inline members" }
            },
            "required": ["board", "layer"]
        }),
        |args, ctx| async move { handle_add_board_graphics(args, ctx).await }
    )
    .with_board_access(crate::tools::BoardAccess::LivePreferredWithFallback)
}

async fn handle_add_board_graphics(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let board_path = get_path(args, "board")?;
    let file = match args
        .get("graphics_file")
        .and_then(serde_json::Value::as_str)
    {
        None => None,
        Some(path) => {
            let parsed = std::fs::read_to_string(path)
                .map_err(|e| e.to_string())
                .and_then(|text| {
                    serde_json::from_str::<serde_json::Value>(&text).map_err(|e| e.to_string())
                });
            match parsed {
                Ok(value) => Some(value),
                Err(reason) => {
                    return Ok(invalid_arg(
                        "graphics_file",
                        &format!("cannot be read as JSON: {reason}"),
                    ))
                }
            }
        }
    };
    let batch = match batch(args, file.as_ref()) {
        Ok(batch) => batch,
        Err((field, reason)) => return Ok(invalid_arg(&field, &reason)),
    };

    let items = batch.ipc_items();
    let fallback = match attempt_ipc_write(ctx, &board_path, "board graphics", move |c| {
        c.create_items(items)
    })
    .await?
    {
        BoardWrite::Ipc(()) => {
            return Ok(CallToolResult::json(&json!({
                "layer": batch.layer,
                "width": batch.width,
                "created": batch.counts(),
                "source": "ipc"
            })))
        }
        BoardWrite::Refused(err) => return Ok(err),
        BoardWrite::File(reason) => reason,
    };

    let content = std::fs::read_to_string(&board_path)?;
    let close_pos = content.rfind(')').unwrap_or(content.len());
    let new_content = apply_edits(content, vec![SexpEdit::insert(close_pos, batch.sexp())]);
    write_atomic(&board_path, &new_content)?;

    Ok(CallToolResult::json(&json!({
        "layer": batch.layer,
        "width": batch.width,
        "created": batch.counts(),
        "source": "file",
        "fallback_reason": fallback.evidence(),
        "warning": fallback.warning()
    })))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tools::pcb_board::mounting_hole_tests::{
        blank_board, ctx_with_ipc, result_text, spawn_rejecting_kicad,
    };
    use konnect_sexp::parser::parse_sexp;

    fn args(board: &std::path::Path, extra: serde_json::Value) -> serde_json::Value {
        let mut base = json!({ "board": board.to_str().unwrap(), "layer": "F.SilkS" });
        for (key, value) in extra.as_object().unwrap() {
            base[key] = value.clone();
        }
        base
    }

    fn sample() -> serde_json::Value {
        json!({
            "lines": [{ "x1": 0, "y1": 0, "x2": 10, "y2": 0 }, { "x1": 10, "y1": 0, "x2": 10, "y2": 5 }],
            "rects": [{ "x1": 20, "y1": 20, "x2": 30, "y2": 25 }],
            "texts": [{ "text": "PT BANK A \"POWER\"", "x": 25, "y": 22, "size": 1.5 }]
        })
    }

    fn refusal_field(args: &serde_json::Value) -> String {
        batch(args, None).unwrap_err().0
    }

    #[test]
    fn copper_unknown_and_missing_layers_are_refused() {
        let p = std::path::Path::new("b.kicad_pcb");
        assert_eq!(
            refusal_field(&args(
                p,
                json!({ "layer": "F.Cu", "lines": sample()["lines"] })
            )),
            "layer"
        );
        assert_eq!(
            refusal_field(&args(
                p,
                json!({ "layer": "In2.Cu", "lines": sample()["lines"] })
            )),
            "layer"
        );
        assert_eq!(
            refusal_field(&args(
                p,
                json!({ "layer": "F.Silk", "lines": sample()["lines"] })
            )),
            "layer"
        );
        let no_layer = json!({ "board": "b.kicad_pcb", "lines": sample()["lines"] });
        assert_eq!(refusal_field(&no_layer), "layer");
    }

    #[test]
    fn bad_members_name_their_index() {
        let p = std::path::Path::new("b.kicad_pcb");
        let zero = json!({ "lines": [{ "x1": 1, "y1": 1, "x2": 1, "y2": 1 }] });
        assert_eq!(refusal_field(&args(p, zero)), "lines[0]");
        let flat = json!({ "rects": [{ "x1": 1, "y1": 1, "x2": 5, "y2": 1 }] });
        assert_eq!(refusal_field(&args(p, flat)), "rects[0]");
        let extra = json!({ "lines": [{ "x1": 0, "y1": 0, "x2": 1, "y2": 1, "w": 2 }] });
        assert_eq!(refusal_field(&args(p, extra)), "lines[0].w");
        let blank = json!({ "texts": [{ "text": " ", "x": 0, "y": 0 }] });
        assert_eq!(refusal_field(&args(p, blank)), "texts[0].text");
        let big = json!({ "texts": [{ "text": "A", "x": 0, "y": 0, "size": 60 }] });
        assert_eq!(refusal_field(&args(p, big)), "texts[0].size");
        let nan = json!({ "lines": [{ "x1": 0, "y1": "a", "x2": 1, "y2": 1 }] });
        assert_eq!(refusal_field(&args(p, nan)), "lines[0].y1");
        assert_eq!(
            refusal_field(&args(p, json!({ "width": 0, "lines": sample()["lines"] }))),
            "width"
        );
        assert_eq!(refusal_field(&args(p, json!({}))), "lines");
    }

    #[test]
    fn inline_and_file_members_are_exclusive() {
        let p = std::path::Path::new("b.kicad_pcb");
        let file = sample();
        let both = args(p, json!({ "lines": sample()["lines"] }));
        assert_eq!(batch(&both, Some(&file)).unwrap_err().0, "graphics_file");
        let from_file = batch(&args(p, json!({})), Some(&file)).unwrap();
        assert_eq!(
            from_file.counts(),
            json!({ "lines": 2, "rects": 1, "texts": 1 })
        );
        let stray = json!({ "lines": [], "layer": "F.Cu" });
        assert_eq!(
            batch(&args(p, json!({})), Some(&stray)).unwrap_err().0,
            "graphics_file"
        );
    }

    #[test]
    fn ipc_items_are_one_shape_per_member_on_the_named_layer() {
        use prost::Message;
        let p = std::path::Path::new("b.kicad_pcb");
        let b = batch(&args(p, sample()), None).unwrap();
        let items = b.ipc_items();
        assert_eq!(items.len(), 4);
        let shape = konnect_ipc::gen::kiapi::board::types::BoardGraphicShape::decode(
            items[2].value.as_slice(),
        )
        .unwrap();
        assert_eq!(shape.layer, BoardLayer::BlFSilkS as i32);
        assert!(items[3].type_url.ends_with("BoardText"));
    }

    #[test]
    fn back_side_text_is_mirrored_in_the_file() {
        let p = std::path::Path::new("b.kicad_pcb");
        let b = batch(
            &args(p, json!({ "layer": "B.SilkS", "texts": sample()["texts"] })),
            None,
        )
        .unwrap();
        let sexp = b.sexp();
        assert!(sexp.contains("(justify mirror)"), "{sexp}");
        assert!(sexp.contains("(thickness 0.225)"), "{sexp}");
        assert!(sexp.contains("PT BANK A \\\"POWER\\\""), "{sexp}");
    }

    #[tokio::test]
    async fn unreachable_kicad_appends_every_member_to_the_file() {
        let dir = tempfile::tempdir().unwrap();
        let board = blank_board(dir.path());
        let ctx = ctx_with_ipc(String::new());
        let res = handle_add_board_graphics(&args(&board, sample()), &ctx)
            .await
            .unwrap();
        assert!(!res.is_error, "{}", result_text(&res));
        let body: serde_json::Value = serde_json::from_str(&result_text(&res)).unwrap();
        assert_eq!(body["source"], "file");
        assert_eq!(
            body["created"],
            json!({ "lines": 2, "rects": 1, "texts": 1 })
        );
        let text = std::fs::read_to_string(&board).unwrap();
        assert_eq!(text.matches("(gr_line").count(), 2);
        assert_eq!(text.matches("(gr_rect").count(), 1);
        assert_eq!(text.matches("(gr_text").count(), 1);
        assert_eq!(text.matches("(layer \"F.SilkS\")").count(), 4);
        parse_sexp(&text).expect("the board stays valid S-expression");
    }

    #[tokio::test]
    async fn graphics_file_feeds_the_batch() {
        let dir = tempfile::tempdir().unwrap();
        let board = blank_board(dir.path());
        let file = dir.path().join("g.json");
        std::fs::write(&file, sample().to_string()).unwrap();
        let ctx = ctx_with_ipc(String::new());
        let a = args(&board, json!({ "graphics_file": file.to_str().unwrap() }));
        let res = handle_add_board_graphics(&a, &ctx).await.unwrap();
        assert!(!res.is_error, "{}", result_text(&res));
        assert_eq!(
            std::fs::read_to_string(&board)
                .unwrap()
                .matches("(gr_")
                .count(),
            4
        );

        std::fs::write(&file, "not json").unwrap();
        let res = handle_add_board_graphics(&a, &ctx).await.unwrap();
        assert!(res.is_error);
        assert!(result_text(&res).contains("graphics_file"));
    }

    #[tokio::test]
    async fn rejecting_kicad_leaves_the_file_alone() {
        let dir = tempfile::tempdir().unwrap();
        let board = blank_board(dir.path());
        let before = std::fs::read_to_string(&board).unwrap();
        let server = spawn_rejecting_kicad();
        let ctx = ctx_with_ipc(server.address().to_string());
        let res = handle_add_board_graphics(&args(&board, sample()), &ctx)
            .await
            .unwrap();
        assert!(res.is_error, "a rejection must not be reported as success");
        assert_eq!(std::fs::read_to_string(&board).unwrap(), before);
    }

    #[tokio::test]
    async fn an_invalid_batch_never_reaches_kicad_or_the_file() {
        let dir = tempfile::tempdir().unwrap();
        let board = blank_board(dir.path());
        let before = std::fs::read_to_string(&board).unwrap();
        let ctx = ctx_with_ipc(String::new());
        let bad = args(
            &board,
            json!({ "lines": [{ "x1": 0, "y1": 0, "x2": 0, "y2": 0 }] }),
        );
        let res = handle_add_board_graphics(&bad, &ctx).await.unwrap();
        assert!(res.is_error);
        assert_eq!(std::fs::read_to_string(&board).unwrap(), before);
    }
}
