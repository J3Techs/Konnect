//! Round and oval footprint drills, in pad-local millimetres.

use konnect_sexp::parser::{parse_sexp, SexpNode};
use konnect_sexp::writer::{apply_edits, find_direct_child_blocks, SexpEdit};
use serde_json::{json, Value};

pub(super) fn schema() -> Value {
    json!({
        "description": "Round drill diameter in mm, or {shape: 'oval', width, height} in pad-local mm. Omit to preserve the existing drill when editing. Existing drill offsets are preserved; pad rotation rotates the slot as well.",
        "oneOf": [
            { "type": "number", "exclusiveMinimum": 0 },
            {
                "type": "object",
                "properties": {
                    "shape": { "const": "oval" },
                    "width": { "type": "number", "exclusiveMinimum": 0 },
                    "height": { "type": "number", "exclusiveMinimum": 0 }
                },
                "required": ["shape", "width", "height"],
                "additionalProperties": false
            }
        ]
    })
}

#[derive(Clone, Copy, Debug)]
pub(super) enum Drill {
    Round(f64),
    Oval { width: f64, height: f64 },
}

impl Drill {
    pub(super) fn requested(value: Option<&Value>) -> Result<Option<Self>, String> {
        let Some(value) = value else { return Ok(None) };
        if let Some(diameter) = value.as_f64() {
            positive(diameter)?;
            return Ok(Some(Self::Round(diameter)));
        }
        let object = value.as_object().ok_or_else(|| {
            "expected a positive diameter or {shape: 'oval', width, height}".to_string()
        })?;
        if object.len() != 3 || object.get("shape").and_then(Value::as_str) != Some("oval") {
            return Err("oval drill requires exactly shape='oval', width and height".into());
        }
        let width = object
            .get("width")
            .and_then(Value::as_f64)
            .ok_or("oval drill width must be a positive number")?;
        let height = object
            .get("height")
            .and_then(Value::as_f64)
            .ok_or("oval drill height must be a positive number")?;
        positive(width)?;
        positive(height)?;
        Ok(Some(Self::Oval { width, height }))
    }

    pub(super) fn validate_pad_type(self, pad_type: &str) -> Result<(), String> {
        if !matches!(pad_type, "thru_hole" | "np_thru_hole") {
            return Err(format!(
                "drills require thru_hole or np_thru_hole pads, not '{pad_type}'"
            ));
        }
        Ok(())
    }

    pub(super) fn definition(self) -> String {
        self.with_offset("")
    }

    fn with_offset(self, offset: &str) -> String {
        match self {
            Self::Round(diameter) => format!("(drill {diameter}{offset})"),
            Self::Oval { width, height } => format!("(drill oval {width} {height}{offset})"),
        }
    }
}

fn positive(value: f64) -> Result<f64, String> {
    if !value.is_finite() || value <= 0.0 {
        return Err("drill dimensions must be finite and greater than zero".into());
    }
    Ok(value)
}

fn finite(node: &SexpNode, index: usize) -> Result<f64, String> {
    node.get_f64(index)
        .filter(|value| value.is_finite())
        .ok_or_else(|| {
            format!(
                "malformed {} coordinate/dimension",
                node.head().unwrap_or("pad")
            )
        })
}

fn inspect_drill(node: &SexpNode) -> Result<Value, String> {
    let oval = node.get(1).and_then(SexpNode::as_str) == Some("oval");
    let width = positive(finite(node, if oval { 2 } else { 1 })?)?;
    let height = if oval {
        positive(finite(node, 3)?)?
    } else {
        width
    };
    let remainder = &node.children().ok_or("malformed drill")?[if oval { 4 } else { 2 }..];
    let mut offset = json!({"x": 0.0, "y": 0.0});
    if !remainder.is_empty() {
        if remainder.len() != 1
            || remainder[0].head() != Some("offset")
            || remainder[0].children().map(<[_]>::len) != Some(3)
        {
            return Err("unsupported or duplicate drill attributes".into());
        }
        offset = json!({"x": finite(&remainder[0], 1)?, "y": finite(&remainder[0], 2)?});
    }
    Ok(
        json!({"shape": if oval { "oval" } else { "circle" }, "width": width, "height": height, "offset": offset}),
    )
}

/// Replace a complete direct-child drill, preserving a nested offset verbatim.
/// Searching for the first ')' would leave an extra parenthesis after offsets.
pub(super) fn replace_in_pad(source: &str, drill: Drill) -> Result<String, String> {
    let pad = parse_sexp(source).map_err(|error| error.to_string())?;
    drill.validate_pad_type(
        pad.get(2)
            .and_then(SexpNode::as_str)
            .ok_or("missing pad type")?,
    )?;
    let mut replacement = None;
    for (start, end) in find_direct_child_blocks(source, "pad") {
        let block = &source[start..end];
        let child = parse_sexp(block).map_err(|error| error.to_string())?;
        if child.head() != Some("drill") {
            continue;
        }
        if replacement.is_some() {
            return Err("pad contains duplicate drill definitions".into());
        }
        inspect_drill(&child)?;
        let mut offset = String::new();
        for (offset_start, offset_end) in find_direct_child_blocks(block, "drill") {
            offset.push(' ');
            offset.push_str(&block[offset_start..offset_end]);
        }
        replacement = Some(SexpEdit::replace(start, end, drill.with_offset(&offset)));
    }
    if let Some(edit) = replacement {
        Ok(apply_edits(source.to_string(), vec![edit]))
    } else {
        let end = source.rfind(')').ok_or("pad has no closing parenthesis")?;
        let mut result = source.to_string();
        result.insert_str(end, &format!(" {}", drill.definition()));
        Ok(result)
    }
}

/// Saved-file readback. Repeated pad numbers are separate physical pads.
pub(super) fn inspect_pads(footprint: &SexpNode) -> Result<Vec<Value>, String> {
    if footprint.head() != Some("footprint") {
        return Err("expected a footprint root".into());
    }
    footprint
        .find_all("pad")
        .into_iter()
        .map(|pad| {
            let number = pad
                .get(1)
                .and_then(SexpNode::as_str)
                .ok_or("missing pad number")?;
            let pad_type = pad
                .get(2)
                .and_then(SexpNode::as_str)
                .ok_or("missing pad type")?;
            let shape = pad
                .get(3)
                .and_then(SexpNode::as_str)
                .ok_or("missing pad shape")?;
            let at = pad.find("at").ok_or("pad has no position")?;
            let size = pad.find("size").ok_or("pad has no size")?;
            let layers = pad
                .find("layers")
                .and_then(SexpNode::children)
                .ok_or("pad has no layers")?
                .iter()
                .skip(1)
                .map(|layer| layer.as_str().ok_or("malformed pad layer"))
                .collect::<Result<Vec<_>, _>>()?;
            if layers.is_empty() {
                return Err("pad has no layers".into());
            }
            let drills = pad.find_all("drill");
            if drills.len() > 1 {
                return Err("pad contains duplicate drill definitions".into());
            }
            let drill = drills
                .first()
                .map(|node| inspect_drill(node))
                .transpose()?
                .unwrap_or(Value::Null);
            let mut result = json!({
                "number": number, "type": pad_type, "shape": shape,
                "x": finite(at, 1)?, "y": finite(at, 2)?,
                "rotation": if at.get(3).is_some() { finite(at, 3)? } else { 0.0 },
                "width": finite(size, 1)?, "height": finite(size, 2)?,
                "layers": layers, "drill": drill
            });
            if let Some(ratio) = pad.find("roundrect_rratio") {
                result["roundrect_rratio"] = json!(finite(ratio, 1)?);
            }
            Ok(result)
        })
        .collect()
}
