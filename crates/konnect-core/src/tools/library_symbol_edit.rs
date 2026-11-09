//! `edit_symbol`: change a symbol in a `.kicad_sym` library in place.
//!
//! Moving a few pins of a 65-pin symbol used to mean `delete_symbol` and
//! `create_symbol` with every pin restated, which loses whatever the library
//! carried beyond the pins: fields, graphics, alternates, descriptions. Here
//! each requested change is a token-level edit of the pin or rectangle block
//! it names — a pin's type, `(at …)`, `(length …)`, name or number; a
//! rectangle's corners — and every other byte of the file stays as it was.
//! The whole request is checked before anything is written, and the written
//! file is read back.
//!
//! Coordinates are the library's own: millimetres with Y pointing up, as the
//! symbol editor shows them. A pin's `(at x y angle)` is its connection
//! point; the angle is the direction from there toward the body: 0 points the
//! pin right, so it sits on the body's left edge.

use crate::mcp::{error::ToolErrorKind, protocol::CallToolResult};
use crate::tool;
use crate::tools::{get_path, invalid_arg, require_str, ToolContext, ToolDef};
use konnect_schematic_editor::types::fmt_f64;
use konnect_sexp::parser::{parse_sexp, SexpNode};
use konnect_sexp::writer::{
    apply_edits, find_direct_child_blocks, read_consistent, write_atomic_if_unchanged, SexpEdit,
};
use serde_json::{json, Value};
use std::collections::BTreeMap;
use std::path::Path;

/// KiCad's twelve pin electrical types.
const PIN_TYPES: [&str; 12] = [
    "input",
    "output",
    "bidirectional",
    "tri_state",
    "passive",
    "free",
    "unspecified",
    "power_in",
    "power_out",
    "open_collector",
    "open_emitter",
    "no_connect",
];

pub(crate) fn tool() -> ToolDef {
    let point = json!({
        "type": "object",
        "properties": { "x": { "type": "number" }, "y": { "type": "number" } },
        "required": ["x", "y"]
    });
    tool!(
        "edit_symbol",
        "Edit a symbol in a .kicad_sym library in place, without deleting and re-creating it: \
         per unit, move pins (position, orientation, length), rename their names and \
         numbers, change their electrical types, and resize body rectangles. Every other \
         property, field and graphic is kept byte for byte. Coordinates are the library's: \
         mm with Y pointing up; a pin's position is its connection point and its orientation \
         the direction toward the body. The whole request is checked first (each pin and \
         rectangle found exactly once, pin numbers still unique), nothing is written on any \
         refusal, and the written file is read back. Placed copies in schematics change only \
         when update_symbols_from_library re-embeds the symbol.",
        json!({
            "type": "object",
            "properties": {
                "library_path": { "type": "string", "description": "Path to the .kicad_sym library file" },
                "symbol_name": { "type": "string", "description": "The symbol's name in the library, e.g. 'MC33PT2001'" },
                "pins": {
                    "type": "array",
                    "description": "Pin edits, each naming a pin by its current number in a unit",
                    "items": {
                        "type": "object",
                        "properties": {
                            "number": { "type": "string", "minLength": 1, "description": "The pin's current number" },
                            "unit": { "type": "integer", "minimum": 0, "default": 1, "description": "The unit holding the pin: 1 for the first; 0 for pins common to every unit" },
                            "body_style": { "type": "integer", "minimum": 1, "default": 1, "description": "1 for the normal body, 2 for the De Morgan alternate" },
                            "x": { "type": "number", "description": "New connection-point X, mm" },
                            "y": { "type": "number", "description": "New connection-point Y, mm (Y up)" },
                            "dx": { "type": "number", "description": "Move the pin by this much in X, mm; with x, refused" },
                            "dy": { "type": "number", "description": "Move the pin by this much in Y, mm (positive is up); with y, refused" },
                            "orientation": { "type": "integer", "enum": [0, 90, 180, 270], "description": "Direction from the connection point toward the body, degrees" },
                            "length": { "type": "number", "exclusiveMinimum": 0, "description": "Pin length, mm" },
                            "new_name": { "type": "string", "description": "The pin's new name ('~' for none)" },
                            "new_number": { "type": "string", "minLength": 1, "description": "The pin's new number" },
                            "electrical_type": { "type": "string", "enum": PIN_TYPES }
                        },
                        "required": ["number"]
                    }
                },
                "rectangles": {
                    "type": "array",
                    "description": "Body rectangle edits: new corners, or edges moved",
                    "items": {
                        "type": "object",
                        "properties": {
                            "unit": { "type": "integer", "minimum": 0, "default": 1, "description": "The unit drawing the rectangle; 0 for graphics common to every unit" },
                            "body_style": { "type": "integer", "minimum": 1, "default": 1 },
                            "index": { "type": "integer", "minimum": 0, "default": 0, "description": "Which of the unit's rectangles, in file order from 0" },
                            "start": point.clone(),
                            "end": point,
                            "move_top": { "type": "number", "description": "Move the top edge (larger Y) by this much, mm; positive is up" },
                            "move_bottom": { "type": "number", "description": "Move the bottom edge (smaller Y) by this much, mm" },
                            "move_left": { "type": "number", "description": "Move the left edge (smaller X) by this much, mm" },
                            "move_right": { "type": "number", "description": "Move the right edge (larger X) by this much, mm" }
                        }
                    }
                },
                "dry_run": { "type": "boolean", "default": false, "description": "Report the changes without writing" }
            },
            "required": ["library_path", "symbol_name"]
        }),
        |args, ctx| async move { handle_edit_symbol(args, ctx).await }
    )
}

/// One direct child block of a unit: its byte range in the file and parse.
struct Block {
    start: usize,
    end: usize,
    node: SexpNode,
}

/// A pin as the library states it.
#[derive(Clone, Debug, PartialEq)]
struct PinState {
    electrical_type: String,
    x: f64,
    y: f64,
    orientation: f64,
    length: f64,
    name: String,
    number: String,
}

impl PinState {
    fn read(node: &SexpNode) -> Option<Self> {
        let at = node.find("at")?;
        let float = |node: Option<&SexpNode>| node.and_then(SexpNode::as_str)?.parse::<f64>().ok();
        Some(PinState {
            electrical_type: node.get(1)?.as_str()?.to_owned(),
            x: float(at.get(1))?,
            y: float(at.get(2))?,
            orientation: float(at.get(3)).unwrap_or(0.0),
            length: float(node.find("length").and_then(|length| length.get(1))).unwrap_or(0.0),
            name: node
                .find("name")
                .and_then(|name| name.get(1))
                .and_then(SexpNode::as_str)
                .unwrap_or("")
                .to_owned(),
            number: node
                .find("number")
                .and_then(|number| number.get(1))
                .and_then(SexpNode::as_str)
                .unwrap_or("")
                .to_owned(),
        })
    }

    fn json(&self) -> Value {
        json!({
            "electrical_type": self.electrical_type,
            "x": self.x,
            "y": self.y,
            "orientation": self.orientation,
            "length": self.length,
            "name": self.name,
            "number": self.number,
        })
    }
}

/// The unit blocks of one symbol, by `(unit, body_style)`.
struct SymbolBlocks {
    units: BTreeMap<(u32, u32), Vec<Block>>,
}

/// Find `name` among the library's top-level symbols and read its units.
fn symbol_blocks(content: &str, name: &str) -> Result<SymbolBlocks, String> {
    let mut found = None;
    for (start, end) in find_direct_child_blocks(content, "kicad_symbol_lib") {
        let block = &content[start..end];
        if !block.starts_with("(symbol") {
            continue;
        }
        let node =
            parse_sexp(block).map_err(|error| format!("a symbol does not parse: {error}"))?;
        if node.get(1).and_then(SexpNode::as_str) == Some(name) {
            if found.is_some() {
                return Err(format!("the library defines '{name}' twice"));
            }
            found = Some((start, end, node));
        }
    }
    let Some((start, end, node)) = found else {
        return Err(format!("the library has no symbol '{name}'"));
    };
    if let Some(parent) = node.find("extends").and_then(|extends| extends.get(1)) {
        return Err(format!(
            "'{name}' extends '{}' and has no pins of its own; edit '{}' instead",
            parent.as_str().unwrap_or("?"),
            parent.as_str().unwrap_or("?")
        ));
    }
    let symbol = &content[start..end];
    let mut units = BTreeMap::new();
    let prefix = format!("{name}_");
    for (unit_start, unit_end) in find_direct_child_blocks(symbol, "symbol") {
        let unit_block = &symbol[unit_start..unit_end];
        if !unit_block.starts_with("(symbol") {
            continue;
        }
        let unit_node = parse_sexp(unit_block).map_err(|error| error.to_string())?;
        let Some(unit_name) = unit_node.get(1).and_then(SexpNode::as_str) else {
            continue;
        };
        let Some((unit, style)) = unit_name
            .strip_prefix(&prefix)
            .and_then(|rest| rest.rsplit_once('_'))
            .and_then(|(unit, style)| {
                Some((unit.parse::<u32>().ok()?, style.parse::<u32>().ok()?))
            })
        else {
            continue;
        };
        let base = start + unit_start;
        let mut children = Vec::new();
        for (child_start, child_end) in find_direct_child_blocks(unit_block, "symbol") {
            let text = &unit_block[child_start..child_end];
            let Ok(child) = parse_sexp(text) else {
                continue;
            };
            children.push(Block {
                start: base + child_start,
                end: base + child_end,
                node: child,
            });
        }
        units.insert((unit, style), children);
    }
    Ok(SymbolBlocks { units })
}

/// The byte range of a block's `index`-th quoted string's contents.
fn quoted_range(text: &str, index: usize) -> Option<(usize, usize)> {
    let mut seen = 0;
    let mut open = None;
    let mut escaped = false;
    for (offset, character) in text.char_indices() {
        match character {
            _ if escaped => escaped = false,
            '\\' if open.is_some() => escaped = true,
            '"' => match open {
                None => open = Some(offset + 1),
                Some(start) => {
                    if seen == index {
                        return Some((start, offset));
                    }
                    seen += 1;
                    open = None;
                }
            },
            _ => {}
        }
    }
    None
}

fn escape(text: &str) -> String {
    text.replace('\\', "\\\\").replace('"', "\\\"")
}

/// The direct child of `block` whose head is `head`, as a range in the file.
fn child_range(content: &str, block: &Block, head: &str) -> Option<(usize, usize)> {
    let text = &content[block.start..block.end];
    find_direct_child_blocks(text, block.node.head().unwrap_or(""))
        .into_iter()
        .find(|(start, end)| {
            parse_sexp(&text[*start..*end])
                .ok()
                .is_some_and(|node| node.head() == Some(head))
        })
        .map(|(start, end)| (block.start + start, block.start + end))
}

fn number_arg(item: &Value, name: &str, field: &str) -> Result<Option<f64>, (String, String)> {
    match item.get(name) {
        None | Some(Value::Null) => Ok(None),
        Some(value) => value
            .as_f64()
            .filter(|number| number.is_finite())
            .map(Some)
            .ok_or_else(|| (format!("{field}{name}"), "must be a number".to_owned())),
    }
}

fn unit_arg(item: &Value, name: &str, field: &str, default: u32) -> Result<u32, (String, String)> {
    match item.get(name) {
        None | Some(Value::Null) => Ok(default),
        Some(value) => value
            .as_u64()
            .and_then(|unit| u32::try_from(unit).ok())
            .ok_or_else(|| {
                (
                    format!("{field}{name}"),
                    "must be a whole number".to_owned(),
                )
            }),
    }
}

/// Every edit the request makes, and what it reports.
struct Plan {
    edits: Vec<SexpEdit>,
    pin_changes: Vec<Value>,
    rectangle_changes: Vec<Value>,
    /// (unit, body style, final pin) for every pin of the symbol after the edit.
    pins_after: Vec<(u32, u32, PinState)>,
}

fn plan_edits(content: &str, name: &str, args: &Value) -> Result<Plan, (String, String)> {
    let blocks =
        symbol_blocks(content, name).map_err(|reason| ("symbol_name".to_owned(), reason))?;
    let mut edits = Vec::new();
    let mut pin_changes = Vec::new();
    let mut rectangle_changes = Vec::new();
    // Every pin, keyed by where it is, so the uniqueness check reads the
    // symbol as it will be.
    let mut pins: BTreeMap<(u32, u32, usize), PinState> = BTreeMap::new();
    for (&(unit, style), children) in &blocks.units {
        for (position, child) in children.iter().enumerate() {
            if child.node.head() == Some("pin") {
                if let Some(state) = PinState::read(&child.node) {
                    pins.insert((unit, style, position), state);
                }
            }
        }
    }
    let mut touched = std::collections::BTreeSet::new();

    let pin_items = args
        .get("pins")
        .and_then(Value::as_array)
        .cloned()
        .unwrap_or_default();
    for (index, item) in pin_items.iter().enumerate() {
        let field = format!("pins[{index}].");
        let number = item
            .get("number")
            .and_then(Value::as_str)
            .filter(|number| !number.is_empty())
            .ok_or_else(|| (format!("{field}number"), "missing".to_owned()))?;
        let unit = unit_arg(item, "unit", &field, 1)?;
        let style = unit_arg(item, "body_style", &field, 1)?;
        let children = blocks.units.get(&(unit, style)).ok_or_else(|| {
            (
                format!("{field}unit"),
                format!("'{name}' has no unit {unit} with body style {style}"),
            )
        })?;
        let matches: Vec<usize> = children
            .iter()
            .enumerate()
            .filter(|(_, child)| {
                child.node.head() == Some("pin")
                    && PinState::read(&child.node).is_some_and(|pin| pin.number == number)
            })
            .map(|(position, _)| position)
            .collect();
        let position = match matches.as_slice() {
            [only] => *only,
            [] => {
                return Err((
                    format!("{field}number"),
                    format!("unit {unit} of '{name}' has no pin numbered '{number}'"),
                ))
            }
            _ => {
                return Err((
                    format!("{field}number"),
                    format!(
                        "unit {unit} of '{name}' has {} pins numbered '{number}'",
                        matches.len()
                    ),
                ))
            }
        };
        if !touched.insert((unit, style, position)) {
            return Err((
                format!("{field}number"),
                format!("pin '{number}' of unit {unit} is edited twice"),
            ));
        }
        let block = &children[position];
        let before = PinState::read(&block.node).expect("matched above");
        let mut after = before.clone();
        let (x, y) = (
            number_arg(item, "x", &field)?,
            number_arg(item, "y", &field)?,
        );
        let (dx, dy) = (
            number_arg(item, "dx", &field)?,
            number_arg(item, "dy", &field)?,
        );
        if x.is_some() && dx.is_some() {
            return Err((format!("{field}dx"), "give x or dx, not both".to_owned()));
        }
        if y.is_some() && dy.is_some() {
            return Err((format!("{field}dy"), "give y or dy, not both".to_owned()));
        }
        after.x = x.unwrap_or(before.x + dx.unwrap_or(0.0));
        after.y = y.unwrap_or(before.y + dy.unwrap_or(0.0));
        // Keep KiCad's own precision: a move by 5.08 from 30.48 is 35.56,
        // not 35.559999999999995.
        after.x = konnect_sexp::geometry::round6(after.x);
        after.y = konnect_sexp::geometry::round6(after.y);
        if let Some(orientation) = number_arg(item, "orientation", &field)? {
            if ![0.0, 90.0, 180.0, 270.0].contains(&orientation) {
                return Err((
                    format!("{field}orientation"),
                    "must be 0, 90, 180 or 270".to_owned(),
                ));
            }
            after.orientation = orientation;
        }
        if let Some(length) = number_arg(item, "length", &field)? {
            if length <= 0.0 {
                return Err((format!("{field}length"), "must be positive".to_owned()));
            }
            after.length = length;
        }
        if let Some(value) = item.get("new_name").filter(|value| !value.is_null()) {
            after.name = value
                .as_str()
                .ok_or_else(|| (format!("{field}new_name"), "must be a string".to_owned()))?
                .to_owned();
        }
        if let Some(value) = item.get("new_number").filter(|value| !value.is_null()) {
            after.number = value
                .as_str()
                .filter(|number| !number.is_empty())
                .ok_or_else(|| {
                    (
                        format!("{field}new_number"),
                        "must be a non-empty string".to_owned(),
                    )
                })?
                .to_owned();
        }
        if let Some(value) = item.get("electrical_type").filter(|value| !value.is_null()) {
            let kind = value.as_str().unwrap_or("");
            if !PIN_TYPES.contains(&kind) {
                return Err((
                    format!("{field}electrical_type"),
                    format!("'{kind}' is not one of {}", PIN_TYPES.join(", ")),
                ));
            }
            after.electrical_type = kind.to_owned();
        }
        if after == before {
            continue;
        }

        let text = &content[block.start..block.end];
        if after.electrical_type != before.electrical_type {
            // `(pin TYPE STYLE …`: the type is the first atom after the head.
            let head_end = "(pin".len();
            let type_start = head_end
                + text[head_end..]
                    .find(|c: char| !c.is_whitespace())
                    .unwrap_or(0);
            let type_end = type_start
                + text[type_start..]
                    .find(|c: char| c.is_whitespace() || c == '(' || c == ')')
                    .unwrap_or(0);
            edits.push(SexpEdit::replace(
                block.start + type_start,
                block.start + type_end,
                after.electrical_type.clone(),
            ));
        }
        if (after.x, after.y, after.orientation) != (before.x, before.y, before.orientation) {
            let (start, end) = child_range(content, block, "at")
                .ok_or_else(|| (format!("{field}number"), "the pin has no (at …)".to_owned()))?;
            edits.push(SexpEdit::replace(
                start,
                end,
                format!(
                    "(at {} {} {})",
                    fmt_f64(after.x),
                    fmt_f64(after.y),
                    fmt_f64(after.orientation)
                ),
            ));
        }
        if after.length != before.length {
            let (start, end) = child_range(content, block, "length").ok_or_else(|| {
                (
                    format!("{field}length"),
                    "the pin has no (length …)".to_owned(),
                )
            })?;
            edits.push(SexpEdit::replace(
                start,
                end,
                format!("(length {})", fmt_f64(after.length)),
            ));
        }
        for (head, old, new, what) in [
            ("name", &before.name, &after.name, "new_name"),
            ("number", &before.number, &after.number, "new_number"),
        ] {
            if old == new {
                continue;
            }
            let (start, end) = child_range(content, block, head).ok_or_else(|| {
                (
                    format!("{field}{what}"),
                    format!("the pin has no ({head} …)"),
                )
            })?;
            let (open, close) = quoted_range(&content[start..end], 0).ok_or_else(|| {
                (
                    format!("{field}{what}"),
                    format!("the pin's {head} has no text"),
                )
            })?;
            edits.push(SexpEdit::replace(start + open, start + close, escape(new)));
        }
        pin_changes.push(json!({
            "unit": unit,
            "body_style": style,
            "before": before.json(),
            "after": after.json(),
        }));
        pins.insert((unit, style, position), after);
    }

    let rectangle_items = args
        .get("rectangles")
        .and_then(Value::as_array)
        .cloned()
        .unwrap_or_default();
    for (index, item) in rectangle_items.iter().enumerate() {
        let field = format!("rectangles[{index}].");
        let unit = unit_arg(item, "unit", &field, 1)?;
        let style = unit_arg(item, "body_style", &field, 1)?;
        let wanted = unit_arg(item, "index", &field, 0)? as usize;
        let children = blocks.units.get(&(unit, style)).ok_or_else(|| {
            (
                format!("{field}unit"),
                format!("'{name}' has no unit {unit} with body style {style}"),
            )
        })?;
        let rectangles: Vec<&Block> = children
            .iter()
            .filter(|child| child.node.head() == Some("rectangle"))
            .collect();
        let block = *rectangles.get(wanted).ok_or_else(|| {
            (
                format!("{field}index"),
                format!(
                    "unit {unit} of '{name}' has {} rectangle(s), so there is no index {wanted}",
                    rectangles.len()
                ),
            )
        })?;
        let corner = |head: &str| -> Option<(f64, f64)> {
            let node = block.node.find(head)?;
            Some((
                node.get(1)?.as_str()?.parse().ok()?,
                node.get(2)?.as_str()?.parse().ok()?,
            ))
        };
        let (Some(start), Some(end)) = (corner("start"), corner("end")) else {
            return Err((
                format!("{field}index"),
                "the rectangle has no corners".to_owned(),
            ));
        };
        let point = |name: &str| -> Result<Option<(f64, f64)>, (String, String)> {
            match item.get(name) {
                None | Some(Value::Null) => Ok(None),
                Some(value) => {
                    let x = value.get("x").and_then(Value::as_f64);
                    let y = value.get("y").and_then(Value::as_f64);
                    match (x, y) {
                        (Some(x), Some(y)) => Ok(Some((x, y))),
                        _ => Err((format!("{field}{name}"), "needs x and y".to_owned())),
                    }
                }
            }
        };
        let (mut new_start, mut new_end) = (
            point("start")?.unwrap_or(start),
            point("end")?.unwrap_or(end),
        );
        // Edges, by which corner holds them: Y points up in a library.
        let top_is_start = new_start.1 >= new_end.1;
        let left_is_start = new_start.0 <= new_end.0;
        if let Some(by) = number_arg(item, "move_top", &field)? {
            if top_is_start {
                new_start.1 += by
            } else {
                new_end.1 += by
            }
        }
        if let Some(by) = number_arg(item, "move_bottom", &field)? {
            if top_is_start {
                new_end.1 += by
            } else {
                new_start.1 += by
            }
        }
        if let Some(by) = number_arg(item, "move_left", &field)? {
            if left_is_start {
                new_start.0 += by
            } else {
                new_end.0 += by
            }
        }
        if let Some(by) = number_arg(item, "move_right", &field)? {
            if left_is_start {
                new_end.0 += by
            } else {
                new_start.0 += by
            }
        }
        let round = |(x, y): (f64, f64)| {
            (
                konnect_sexp::geometry::round6(x),
                konnect_sexp::geometry::round6(y),
            )
        };
        let (new_start, new_end) = (round(new_start), round(new_end));
        if new_start.0 == new_end.0 || new_start.1 == new_end.1 {
            return Err((
                format!("{field}index"),
                "the rectangle would have no area".to_owned(),
            ));
        }
        for (head, before, after) in [("start", start, new_start), ("end", end, new_end)] {
            if before == after {
                continue;
            }
            let (from, to) = child_range(content, block, head)
                .ok_or_else(|| (format!("{field}index"), format!("no ({head} …)")))?;
            edits.push(SexpEdit::replace(
                from,
                to,
                format!("({head} {} {})", fmt_f64(after.0), fmt_f64(after.1)),
            ));
        }
        if (new_start, new_end) != (start, end) {
            rectangle_changes.push(json!({
                "unit": unit,
                "body_style": style,
                "index": wanted,
                "before": { "start": { "x": start.0, "y": start.1 }, "end": { "x": end.0, "y": end.1 } },
                "after": { "start": { "x": new_start.0, "y": new_start.1 }, "end": { "x": new_end.0, "y": new_end.1 } },
            }));
        }
    }

    // A pin number names one pin of a body style across every unit, as a
    // package pin does: an edited pin may not end up sharing one. Duplicates
    // the library already had elsewhere are not this edit's to refuse.
    for key in &touched {
        let pin = &pins[key];
        let (_, style, _) = *key;
        let sharing: Vec<u32> = pins
            .iter()
            .filter(|((_, other_style, _), other)| {
                *other_style == style && other.number == pin.number
            })
            .map(|((unit, _, _), _)| *unit)
            .collect();
        if sharing.len() > 1 {
            return Err((
                "pins".to_owned(),
                format!(
                    "pin number '{}' would name {} pins of '{name}' (units {})",
                    pin.number,
                    sharing.len(),
                    sharing
                        .iter()
                        .map(u32::to_string)
                        .collect::<Vec<_>>()
                        .join(", ")
                ),
            ));
        }
    }
    Ok(Plan {
        edits,
        pin_changes,
        rectangle_changes,
        pins_after: pins
            .into_iter()
            .map(|((unit, style, _), pin)| (unit, style, pin))
            .collect(),
    })
}

async fn handle_edit_symbol(args: &Value, _ctx: &ToolContext) -> anyhow::Result<CallToolResult> {
    let path = get_path(args, "library_path")?;
    let name = match require_str(args, "symbol_name") {
        Ok(name) => name.to_owned(),
        Err(error) => return Ok(error),
    };
    let dry_run = args
        .get("dry_run")
        .and_then(Value::as_bool)
        .unwrap_or(false);
    if !path.is_file() {
        return Ok(CallToolResult::error_kind(
            ToolErrorKind::FileNotFound {
                path: path.display().to_string(),
            },
            format!("Library '{}' not found", path.display()),
        ));
    }
    let content = read_consistent(&path)?;
    let plan = match plan_edits(&content, &name, args) {
        Ok(plan) => plan,
        Err((field, reason)) => return Ok(invalid_arg(&field, &reason)),
    };
    let mut response = json!({
        "library_path": path.display().to_string(),
        "symbol_name": name,
        "dry_run": dry_run,
        "pin_changes": plan.pin_changes,
        "rectangle_changes": plan.rectangle_changes,
    });
    if plan.edits.is_empty() || dry_run {
        response["written"] = json!(false);
        return Ok(CallToolResult::json(&response));
    }
    let updated = apply_edits(content.clone(), plan.edits);
    parse_sexp(&updated).map_err(|error| {
        anyhow::anyhow!("the edited library would not parse, so nothing was written: {error}")
    })?;
    write_atomic_if_unchanged(&path, &content, &updated)?;

    // Read back: the written pins must be the planned ones.
    if let Err(reason) = confirm_written(&path, &name, &plan.pins_after) {
        return Ok(crate::tools::mutation_outcome_uncertain(
            &path,
            "edit_symbol",
            reason,
        ));
    }
    response["written"] = json!(true);
    Ok(CallToolResult::json(&response))
}

fn confirm_written(
    path: &Path,
    name: &str,
    expected: &[(u32, u32, PinState)],
) -> Result<(), String> {
    let written = read_consistent(path).map_err(|error| error.to_string())?;
    let blocks = symbol_blocks(&written, name)?;
    let mut found: Vec<(u32, u32, PinState)> = Vec::new();
    for (&(unit, style), children) in &blocks.units {
        for child in children {
            if child.node.head() == Some("pin") {
                if let Some(pin) = PinState::read(&child.node) {
                    found.push((unit, style, pin));
                }
            }
        }
    }
    if found.len() != expected.len()
        || found
            .iter()
            .zip(expected)
            .any(|(have, want)| have.0 != want.0 || have.1 != want.1 || have.2 != want.2)
    {
        return Err(format!(
            "the pins of '{name}' read back are not the ones written"
        ));
    }
    Ok(())
}

#[cfg(test)]
#[path = "library_symbol_edit_tests.rs"]
mod tests;
