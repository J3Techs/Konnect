//! `pcb_components` toolset — place, move, rotate, query, and array footprints on the PCB.
//!
//! Most operations use the KiCAD IPC API so they integrate with KiCAD's undo/redo
//! system and don't require a separate file-sync step. `get_board_2d_view` uses
//! kicad-cli to render a PNG.

use crate::mcp::protocol::CallToolResult;
use crate::tool;
use crate::tools::footprint_fabrication::{
    guard_live_rebuild,
    Remedy::{ClosedBoardPath, NoLosslessPath},
};
use crate::tools::library::{footprint_lib_nickname_for_dir, is_lib_id, resolve_footprint_path};
use crate::tools::pcb_board::{attempt_ipc_write, BoardWrite};
use crate::tools::{
    get_path, require_array, require_f64, require_str, require_u64, with_board_ipc_classified,
    ToolContext, ToolDef,
};
use anyhow::Context;
use konnect_ipc::client::KiCadIpcClient;
use konnect_sexp::writer::{
    apply_edits, find_balanced_block, find_block_starts, find_direct_child_blocks, new_uuid,
    read_consistent, write_atomic_if_unchanged,
};
use konnect_sexp::SexpEdit;
use prost::Message;
use serde_json::json;
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, HashMap, HashSet, VecDeque};
use std::path::{Path, PathBuf};

mod attributes;
pub(crate) mod reference_field;

macro_rules! ipc {
    ($ctx:expr, $args:expr, |$c:ident| $body:expr) => {{
        let requested_board = get_path($args, "board")?;
        match with_board_ipc_classified($ctx, &requested_board, move |$c, _| $body).await? {
            Ok(v) => v,
            // Only an unreachable transport justifies "KiCAD must be running".
            // This used to say it for every failure, so a tool that refused a
            // request on its merits — "a polygon needs at least 3 points" —
            // told the reader to start a KiCad that was already running, with
            // the actual reason parenthesised after a false statement.
            Err(konnect_ipc::IpcFailure::Unreachable(msg)) => {
                return Ok(CallToolResult::error(format!(
                    "KiCAD must be running with the board loaded (IPC error: {})",
                    msg
                )))
            }
            Err(konnect_ipc::IpcFailure::Target { error, .. }) => {
                return Ok(crate::tools::ipc_target_error_result(&error))
            }
            Err(konnect_ipc::IpcFailure::Rejected(msg)) => return Ok(CallToolResult::error(msg)),
            Err(konnect_ipc::IpcFailure::Recovered(msg)) => {
                return Ok(crate::tools::ipc_recovered_result(&msg))
            }
            Err(konnect_ipc::IpcFailure::Uncertain(msg)) => {
                return Ok(crate::tools::ipc_uncertain_result(&msg))
            }
        }
    }};
}

// ─── Footprint-library resolution ───────────────────────────────────────────

/// Read the library source of `lib_id` (`Library:Footprint`), resolving it
/// through the project's fp-lib-table (the board's directory), then the global
/// table, then the conventional KiCad library directories — the lookup that
/// `library::resolve_footprint_path` owns.
pub(crate) fn resolve_footprint_source(lib_id: &str, board: &Path) -> anyhow::Result<String> {
    let path = resolve_footprint_file(lib_id, board)?;
    std::fs::read_to_string(&path)
        .map_err(|error| anyhow::anyhow!("failed to read {}: {error}", path.display()))
}

/// The resolution half of [`resolve_footprint_source`], for a caller that
/// reports "does not resolve" and "cannot be read" as different failures.
pub(crate) fn resolve_footprint_file(lib_id: &str, board_path: &Path) -> anyhow::Result<PathBuf> {
    let (nickname, entry) = lib_id.split_once(':').ok_or_else(|| {
        anyhow::anyhow!("footprint must use Library:Footprint syntax, got '{lib_id}'")
    })?;
    if nickname.is_empty() || entry.is_empty() {
        anyhow::bail!("footprint must use a non-empty Library:Footprint identifier");
    }
    super::library::resolve_footprint_path(lib_id, board_path.parent())
        .map_err(|message| anyhow::anyhow!(message))
}

/// Structured rejection for any back-side (`B.*`) placement layer.
///
/// Placing on the back is not a layer rename: KiCAD's flip mirrors the
/// footprint's geometry (pad X positions negate, every front layer swaps with
/// its back counterpart per item). Until Konnect implements that mirror,
/// Read a `[{x, y}, …]` argument into footprint-local millimetre pairs.
///
/// Each rejection names the offending index, because "each point needs a
/// numeric 'x'" on a twelve-vertex courtyard tells the caller nothing about
/// which vertex to fix.
fn parse_points(value: &serde_json::Value) -> Result<Vec<(f64, f64)>, CallToolResult> {
    let invalid = |field: &str, reason: String| {
        CallToolResult::error_kind(
            crate::mcp::error::ToolErrorKind::InvalidArgument {
                field: field.to_string(),
                reason: reason.clone(),
            },
            format!("Argument '{field}' is invalid: {reason}"),
        )
    };

    let array = value
        .as_array()
        .ok_or_else(|| invalid("points", "missing or not an array of {x, y}".to_string()))?;

    let mut points = Vec::with_capacity(array.len());
    for (i, p) in array.iter().enumerate() {
        let x = p["x"]
            .as_f64()
            .ok_or_else(|| invalid("points", format!("point {i} has no numeric 'x'")))?;
        let y = p["y"]
            .as_f64()
            .ok_or_else(|| invalid("points", format!("point {i} has no numeric 'y'")))?;
        points.push((x, y));
    }
    Ok(points)
}

/// pretending to support `B.Cu` silently produces wrong copper, so the layer
/// is refused up front — before anything is resolved, sent, or written.
fn back_side_layer_error(layer: &str) -> Option<CallToolResult> {
    if !layer.starts_with("B.") {
        return None;
    }
    Some(CallToolResult::error_kind(
        crate::mcp::error::ToolErrorKind::InvalidArgument {
            field: "layer".to_string(),
            reason: format!("back-side placement on '{layer}' is not yet supported"),
        },
        format!(
            "Cannot place on '{layer}': back-side placement is not yet supported, \
             because a correct flip must mirror the footprint geometry rather than \
             just rename its layers. Place the footprint on F.Cu and flip it to the \
             back in KiCAD (select it and press F)."
        ),
    ))
}

fn escape_sexp_string(value: &str) -> String {
    value.replace('\\', "\\\\").replace('"', "\\\"")
}

fn replace_quoted_after(source: &mut String, marker: &str, value: &str) -> anyhow::Result<()> {
    let start = source
        .find(marker)
        .map(|offset| offset + marker.len())
        .ok_or_else(|| anyhow::anyhow!("footprint library data is missing {marker}"))?;
    let bytes = source.as_bytes();
    let mut escaped = false;
    let end = (start..bytes.len())
        .find(|index| {
            let byte = bytes[*index];
            if escaped {
                escaped = false;
                false
            } else if byte == b'\\' {
                escaped = true;
                false
            } else {
                byte == b'"'
            }
        })
        .ok_or_else(|| anyhow::anyhow!("unterminated quoted value after {marker}"))?;
    source.replace_range(start..end, &escape_sexp_string(value));
    Ok(())
}

fn replace_reference(source: &mut String, reference: &str) -> anyhow::Result<()> {
    replace_footprint_field(source, "Reference", reference)
}

/// Byte spans of the top-level tokens inside one balanced `(…)` block,
/// relative to the block, head first. Strings keep their quotes, nested lists
/// are one token each.
pub(crate) fn sexp_block_tokens(block: &str) -> Vec<(usize, usize)> {
    let bytes = block.as_bytes();
    let mut out = Vec::new();
    let end = block.len().saturating_sub(1);
    let mut i = 1;
    while i < end {
        match bytes[i] {
            b' ' | b'\t' | b'\r' | b'\n' => i += 1,
            b'"' => {
                let start = i;
                i += 1;
                while i < end {
                    match bytes[i] {
                        b'\\' => i += 2,
                        b'"' => break,
                        _ => i += 1,
                    }
                }
                i = (i + 1).min(end);
                out.push((start, i));
            }
            b'(' => match find_balanced_block(block, i) {
                Some((start, stop)) => {
                    out.push((start, stop));
                    i = stop;
                }
                None => break,
            },
            _ => {
                let start = i;
                while i < end && !matches!(bytes[i], b' ' | b'\t' | b'\r' | b'\n' | b'(' | b')') {
                    i += 1;
                }
                out.push((start, i));
            }
        }
    }
    out
}

/// A token's value: a quoted string unescaped as KiCad's reader does, or an
/// atom as written.
pub(crate) fn sexp_token_value(token: &str) -> String {
    match konnect_sexp::parse_sexp(token) {
        Ok(node) => node.as_str().map(str::to_string).unwrap_or_default(),
        Err(_) => token.to_string(),
    }
}

/// Byte span, quotes included, of a footprint field's value: the root's own
/// `(property "Name" "value" …)`, or — for Reference and Value, in files
/// older than KiCad 8 — `(fp_text reference|value "value" …)`. Nested
/// properties (a pad's, a group's) are never matched.
pub(crate) fn footprint_field_value_span(source: &str, name: &str) -> Option<(usize, usize)> {
    let fp_text = match name {
        "Reference" => Some("reference"),
        "Value" => Some("value"),
        _ => None,
    };
    let children = find_direct_child_blocks(source, "footprint");
    let find = |head: &str, key: &str| {
        children.iter().find_map(|&(start, end)| {
            let block = &source[start..end];
            let spans = sexp_block_tokens(block);
            let (head_start, head_end) = *spans.first()?;
            let (key_start, key_end) = *spans.get(1)?;
            let (value_start, value_end) = *spans.get(2)?;
            (&block[head_start..head_end] == head
                && sexp_token_value(&block[key_start..key_end]) == key)
                .then_some((start + value_start, start + value_end))
        })
    };
    find("property", name).or_else(|| fp_text.and_then(|kind| find("fp_text", kind)))
}

/// Set a footprint field's value through [`footprint_field_value_span`].
fn replace_footprint_field(source: &mut String, name: &str, value: &str) -> anyhow::Result<()> {
    let (start, end) = footprint_field_value_span(source, name).ok_or_else(|| {
        anyhow::anyhow!("footprint library data has no {name} property or fp_text")
    })?;
    source.replace_range(start..end, &format!("\"{}\"", escape_sexp_string(value)));
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn prepare_footprint_source(
    source: &str,
    lib_id: &str,
    reference: &str,
    value: Option<&str>,
    x: f64,
    y: f64,
    rotation: f64,
    layer: &str,
) -> anyhow::Result<String> {
    // No back-side placement: a correct F.Cu→B.Cu flip mirrors the geometry
    // (pad X positions negate, layers swap per item) the way KiCAD's own flip
    // does. A textual layer swap produces wrong copper, so it is refused
    // outright — see back_side_layer_error.
    if layer != "F.Cu" {
        anyhow::bail!(
            "footprints can only be placed on F.Cu (back-side placement is not yet \
             supported because a correct flip must mirror the geometry), got '{layer}'"
        );
    }
    let mut prepared = source.to_string();
    replace_quoted_after(&mut prepared, "(footprint \"", lib_id)?;
    replace_reference(&mut prepared, reference)?;
    if let Some(value) = value {
        replace_footprint_field(&mut prepared, "Value", value)?;
    }
    replace_quoted_after(&mut prepared, "(layer \"", layer)?;
    let layer_start = prepared
        .find("(layer \"")
        .context("footprint library data has no root layer")?;
    let layer_end = prepared[layer_start..]
        .find(')')
        .map(|offset| layer_start + offset + 1)
        .context("footprint root layer is unterminated")?;
    prepared.insert_str(layer_end, &format!("\n\t(at {x} {y} {rotation})"));
    konnect_sexp::parse_sexp(&prepared).context("prepared footprint is not valid S-expression")?;
    Ok(prepared)
}

pub(crate) fn extract_pad_definitions(
    source: &str,
) -> anyhow::Result<Vec<konnect_ipc::IpcPadDefinition>> {
    let footprint = konnect_sexp::parse_sexp(source)?;
    let format_version = sexp_format_version(&footprint);
    footprint
        .find_all("pad")
        .into_iter()
        .map(|pad| {
            let required = |index: usize, label: &str| {
                pad.get(index)
                    .and_then(konnect_sexp::SexpNode::as_str)
                    .ok_or_else(|| anyhow::anyhow!("footprint pad is missing {label}"))
            };
            let shape = required(3, "shape")?.to_string();
            let number = pad
                .get(1)
                .and_then(konnect_sexp::SexpNode::as_str)
                .unwrap_or_default();
            refuse_uncarried_pad_clauses(pad, &shape).with_context(|| {
                format!("pad '{number}' cannot be carried by KiCad 10's typed placement path")
            })?;
            let custom = if shape == "custom" {
                Some(parse_custom_pad(pad).with_context(|| {
                    format!("custom pad '{number}' cannot be carried by KiCad 10's typed placement path")
                })?)
            } else {
                None
            };
            let zone_connection = pad_zone_connection(pad)
                .with_context(|| format!("pad '{number}' has an unreadable zone connection"))?;
            let (thermal, _) = pad_thermal(pad, format_version).with_context(|| {
                format!("pad '{number}' cannot be carried by KiCad 10's typed placement path")
            })?;
            let at = pad
                .find("at")
                .context("footprint pad is missing its position")?;
            let size = pad
                .find("size")
                .context("footprint pad is missing its size")?;
            let layers = pad
                .find("layers")
                .context("footprint pad is missing its layer set")?
                .children()
                .unwrap_or_default()
                .iter()
                .skip(1)
                .filter_map(konnect_sexp::SexpNode::as_str)
                .map(str::to_string)
                .collect();
            let (drill_x, drill_y, drill_oval) = match pad.find("drill") {
                Some(drill)
                    if drill.get(1).and_then(konnect_sexp::SexpNode::as_str) == Some("oval") =>
                {
                    (
                        drill.get_f64(2),
                        drill.get_f64(3).or_else(|| drill.get_f64(2)),
                        true,
                    )
                }
                Some(drill) => (
                    drill.get_f64(1),
                    drill.get_f64(2).or_else(|| drill.get_f64(1)),
                    false,
                ),
                None => (None, None, false),
            };
            Ok(konnect_ipc::IpcPadDefinition {
                number: required(1, "number")?.to_string(),
                pad_type: required(2, "type")?.to_string(),
                shape,
                x: at
                    .get_f64(1)
                    .context("footprint pad has an invalid X position")?,
                y: at
                    .get_f64(2)
                    .context("footprint pad has an invalid Y position")?,
                rotation: at.get_f64(3).unwrap_or(0.0),
                size_x: size
                    .get_f64(1)
                    .context("footprint pad has an invalid width")?,
                size_y: size
                    .get_f64(2)
                    .context("footprint pad has an invalid height")?,
                drill_x,
                drill_y,
                drill_oval,
                layers,
                roundrect_ratio: pad.find_f64("roundrect_rratio").unwrap_or(0.0),
                custom,
                zone_connection,
                thermal,
            })
        })
        .collect()
}

/// A pad's `(zone_connect N)`: `None` when the pad leaves it out and so
/// inherits, the named connection otherwise. A repeated clause, a value that
/// is not one integer, or an integer KiCad does not define is refused rather
/// than read as inherited.
pub(crate) fn pad_zone_connection(
    pad: &konnect_sexp::SexpNode,
) -> anyhow::Result<Option<konnect_ipc::IpcPadZoneConnection>> {
    let clauses = pad.find_all("zone_connect");
    let clause = match clauses.as_slice() {
        [] => return Ok(None),
        [clause] => clause,
        _ => anyhow::bail!("the pad has {} (zone_connect …) clauses", clauses.len()),
    };
    let values = clause.children().unwrap_or_default();
    anyhow::ensure!(
        values.len() == 2,
        "(zone_connect …) must hold exactly one value"
    );
    let text = values[1]
        .as_str()
        .filter(|_| matches!(values[1], konnect_sexp::SexpNode::Atom(_)))
        .context("(zone_connect …) must hold a bare integer")?;
    let value: i64 = text
        .parse()
        .with_context(|| format!("zone_connect '{text}' is not an integer"))?;
    konnect_ipc::IpcPadZoneConnection::from_file_value(value)
        .map(Some)
        .with_context(|| {
            format!("zone_connect {value} is not a KiCad zone connection (-1, 0, 1, 2 or 3)")
        })
}

/// A file's `(version N)`, read from the root's own children. KiCad's parser
/// treats a missing one as 0, which callers pass on as `None`.
pub(crate) fn sexp_format_version(root: &konnect_sexp::SexpNode) -> Option<i64> {
    root.children()
        .unwrap_or_default()
        .iter()
        .find(|child| child.head() == Some("version"))
        .and_then(|version| version.get(1))
        .and_then(konnect_sexp::SexpNode::as_str)
        .and_then(|text| text.parse().ok())
}

/// A pad's thermal-relief clauses as KiCad 10's footprint parser reads them
/// (`PCB_IO_KICAD_SEXPR_PARSER::parsePAD`): `(thermal_bridge_width mm)` or
/// its legacy spelling `(thermal_width mm)`, `(thermal_gap mm)` and
/// `(thermal_bridge_angle degrees)`.
///
/// The width and gap stay `None` unless written, and an explicit zero stays
/// zero. The angle is always filled in, because the parser always sets one:
/// the written value, or its shape default (see
/// [`konnect_ipc::file_default_thermal_spoke_angle`]); the flag says whether
/// the file wrote it. A repeated clause, one that is not a single bare
/// number, and a value KiCad could not hold as written are refused.
pub(crate) fn pad_thermal(
    pad: &konnect_sexp::SexpNode,
    format_version: Option<i64>,
) -> anyhow::Result<(konnect_ipc::IpcPadThermal, bool)> {
    let value = |tags: &[&str], unit: &str| -> anyhow::Result<Option<f64>> {
        let clauses = pad
            .children()
            .unwrap_or_default()
            .iter()
            .filter(|child| child.head().is_some_and(|tag| tags.contains(&tag)))
            .collect::<Vec<_>>();
        let clause = match clauses.as_slice() {
            [] => return Ok(None),
            [clause] => clause,
            _ => anyhow::bail!(
                "the pad has {} ({} …) clauses",
                clauses.len(),
                tags.join(" …)/(")
            ),
        };
        let tag = clause.head().unwrap_or_default();
        let values = clause.children().unwrap_or_default();
        anyhow::ensure!(
            values.len() == 2 && matches!(values[1], konnect_sexp::SexpNode::Atom(_)),
            "({tag} …) must hold exactly one bare number of {unit}"
        );
        let text = values[1].as_str().unwrap_or_default();
        let number: f64 = text
            .parse()
            .with_context(|| format!("{tag} '{text}' is not a number"))?;
        anyhow::ensure!(number.is_finite(), "{tag} '{text}' is not finite");
        Ok(Some(number))
    };
    let shape = pad
        .get(3)
        .and_then(konnect_sexp::SexpNode::as_str)
        .unwrap_or_default();
    let anchor = pad
        .find("options")
        .and_then(|options| options.find("anchor"))
        .and_then(|anchor| anchor.get(1))
        .and_then(konnect_sexp::SexpNode::as_str)
        .and_then(|anchor| match anchor {
            "circle" => Some(konnect_ipc::IpcCustomPadAnchor::Circle),
            "rect" => Some(konnect_ipc::IpcCustomPadAnchor::Rect),
            _ => None,
        });
    let written_angle = value(&["thermal_bridge_angle"], "degrees")?;
    let thermal = konnect_ipc::IpcPadThermal {
        spoke_width: value(&["thermal_bridge_width", "thermal_width"], "mm")?,
        gap: value(&["thermal_gap"], "mm")?,
        spoke_angle: Some(written_angle.unwrap_or_else(|| {
            konnect_ipc::file_default_thermal_spoke_angle(shape, anchor, format_version)
        })),
    };
    thermal.validate()?;
    Ok((thermal, written_angle.is_some()))
}

/// Refuse a pad clause the typed placement path would drop. A pad is built
/// from its position, size, layers, drill, corner ratio, custom shape, zone
/// connection and thermal settings; anything else that changes the pad (a
/// fabrication property, mask or paste margins, clearance overrides, chamfers, a
/// trapezoid delta, a drill offset, removed unused layers, a padstack, …) has
/// no field it could travel in, so placing the pad without it would put a
/// different pad on the board than the library holds.
///
/// `uuid`/`tstamp` are identity KiCad assigns afresh, and `pinfunction`/
/// `pintype` are schematic data, not pad geometry.
fn refuse_uncarried_pad_clauses(pad: &konnect_sexp::SexpNode, shape: &str) -> anyhow::Result<()> {
    for child in pad.children().unwrap_or_default().iter().skip(4) {
        let Some(tag) = child.head() else {
            anyhow::bail!(
                "pad flag '{}' is not supported",
                child.as_str().unwrap_or("<list>")
            );
        };
        match tag {
            "at" | "size" | "layers" | "roundrect_rratio" | "zone_connect" | "uuid" | "tstamp"
            | "pinfunction" | "pintype" => {}
            // Read and validated by `pad_thermal`.
            "thermal_bridge_width" | "thermal_width" | "thermal_gap" | "thermal_bridge_angle" => {}
            "options" | "primitives" if shape == "custom" => {}
            "drill" => {
                if let Some(nested) = child
                    .children()
                    .unwrap_or_default()
                    .iter()
                    .skip(1)
                    .find_map(konnect_sexp::SexpNode::head)
                {
                    anyhow::bail!("drill clause '{nested}' is not supported");
                }
            }
            // The typed pad stack always keeps unused layers.
            "remove_unused_layers" => {
                let value = child.get(1).and_then(konnect_sexp::SexpNode::as_str);
                anyhow::ensure!(
                    value == Some("no"),
                    "'(remove_unused_layers {})' is not supported",
                    value.unwrap_or("")
                );
            }
            "property" => anyhow::bail!(
                "pad property '{}' is not supported: KiCad's API has no field for a pad's \
                 fabrication property",
                child
                    .get(1)
                    .and_then(konnect_sexp::SexpNode::as_str)
                    .unwrap_or("")
            ),
            other => anyhow::bail!("pad clause '{other}' is not supported"),
        }
    }
    Ok(())
}

/// Read a custom pad's `(options …)` and `(primitives …)` into the typed form
/// KiCad's API carries (`PadStackLayer::custom_anchor_shape` and
/// `custom_shapes`), keeping every coordinate pad-local.
///
/// Only what that form reproduces exactly is accepted. Everything else is
/// refused by name rather than dropped or guessed: a `convexhull` clearance
/// (the API has no field for it, and KiCad's default is `outline`), bezier,
/// annotation and vector primitives, polygon arc edges, rounded rectangles,
/// a primitive missing its width or fill, and any clause not listed here.
fn parse_custom_pad(pad: &konnect_sexp::SexpNode) -> anyhow::Result<konnect_ipc::IpcCustomPad> {
    use konnect_ipc::{IpcCustomPadAnchor, IpcPadPrimitive};
    use konnect_sexp::SexpNode;

    let options = pad.find("options").context(
        "it has no (options …), so its anchor shape is unknown; save the footprint \
         in KiCad 10 to write it",
    )?;
    let mut anchor = None;
    for option in options.children().unwrap_or_default().iter().skip(1) {
        let value = option.get(1).and_then(SexpNode::as_str);
        match (option.head(), value) {
            (Some("anchor"), Some("circle")) => anchor = Some(IpcCustomPadAnchor::Circle),
            (Some("anchor"), Some("rect")) => anchor = Some(IpcCustomPadAnchor::Rect),
            (Some("anchor"), other) => anyhow::bail!(
                "anchor '{}' is not one KiCad supports (circle or rect)",
                other.unwrap_or("")
            ),
            (Some("clearance"), Some("outline")) => {}
            (Some("clearance"), Some("convexhull")) => anyhow::bail!(
                "'(clearance convexhull)' cannot be sent: KiCad's API has no field for a \
                 custom pad's zone clearance shape, so the pad would arrive with the \
                 'outline' default. Change the pad to 'outline' clearance in the \
                 footprint editor if that is acceptable"
            ),
            (Some("clearance"), other) => anyhow::bail!(
                "clearance '{}' is not one KiCad supports (outline or convexhull)",
                other.unwrap_or("")
            ),
            (head, _) => anyhow::bail!(
                "custom pad option '{}' is not supported",
                head.unwrap_or("<atom>")
            ),
        }
    }
    let anchor = anchor.context("its (options …) names no anchor shape")?;

    let primitives_node = pad.find("primitives").context("it has no (primitives …)")?;
    let mut primitives = Vec::new();
    for (index, node) in primitives_node
        .children()
        .unwrap_or_default()
        .iter()
        .skip(1)
        .enumerate()
    {
        let primitive = parse_pad_primitive(node)
            .with_context(|| format!("primitive {index} ({})", node.head().unwrap_or("<atom>")))?;
        primitive
            .validate()
            .with_context(|| format!("primitive {index} ({})", primitive.kind()))?;
        primitives.push(primitive);
    }
    let custom = konnect_ipc::IpcCustomPad { anchor, primitives };
    custom.validate()?;
    return Ok(custom);

    fn parse_pad_primitive(node: &SexpNode) -> anyhow::Result<IpcPadPrimitive> {
        let kind = node.head().context("a primitive must be a list")?;
        let geometry: &[&str] = match kind {
            "gr_poly" => &["pts"],
            "gr_rect" => &["start", "end"],
            "gr_circle" => &["center", "end"],
            "gr_line" => &["start", "end"],
            "gr_arc" => &["start", "mid", "end"],
            "gr_curve" | "bezier" => anyhow::bail!("bezier primitives are not supported"),
            "gr_bbox" | "gr_vector" => anyhow::bail!(
                "'{kind}' annotation primitives are not supported: KiCad's API cannot \
                 mark them as annotation, so they would arrive as copper"
            ),
            // KiCad writes width and fill on each primitive; a list-level
            // default would have to be guessed onto them.
            "width" | "fill" => anyhow::bail!(
                "a list-level ({kind} …) in (primitives …) is not supported; save the \
                 footprint in KiCad 10 to write it on each primitive"
            ),
            other => anyhow::bail!("primitive type '{other}' is not supported"),
        };
        let closed = matches!(kind, "gr_poly" | "gr_rect" | "gr_circle");
        let mut width = None;
        let mut filled = None;
        let mut seen = HashSet::new();
        for child in node.children().unwrap_or_default().iter().skip(1) {
            let tag = child
                .head()
                .with_context(|| format!("unexpected bare token in {kind}"))?;
            anyhow::ensure!(seen.insert(tag), "{kind} has more than one ({tag} …)");
            match tag {
                _ if geometry.contains(&tag) => {}
                "width" => width = Some(number(child, 1, "width")?),
                "stroke" => {
                    for part in child.children().unwrap_or_default().iter().skip(1) {
                        match (part.head(), part.get(1).and_then(SexpNode::as_str)) {
                            (Some("width"), _) => width = Some(number(part, 1, "stroke width")?),
                            (Some("type"), Some("solid" | "default")) => {}
                            (Some("type"), other) => anyhow::bail!(
                                "stroke type '{}' is not supported",
                                other.unwrap_or("")
                            ),
                            (head, _) => anyhow::bail!(
                                "stroke clause '{}' is not supported",
                                head.unwrap_or("<atom>")
                            ),
                        }
                    }
                }
                "fill" => {
                    filled = Some(match child.get(1).and_then(SexpNode::as_str) {
                        Some("yes" | "solid") => true,
                        Some("no" | "none") => false,
                        other => anyhow::bail!(
                            "fill '{}' is not supported (yes or no)",
                            other.unwrap_or("")
                        ),
                    })
                }
                other => anyhow::bail!("{kind} clause '{other}' is not supported"),
            }
        }
        let width = width.with_context(|| format!("{kind} has no width"))?;
        let filled = if closed {
            filled.with_context(|| format!("{kind} has no (fill …), so its copper is ambiguous"))?
        } else {
            anyhow::ensure!(filled != Some(true), "a filled {kind} is not supported");
            false
        };
        let point = |tag: &str| -> anyhow::Result<(f64, f64)> {
            let node = node
                .find(tag)
                .with_context(|| format!("{kind} has no ({tag} …)"))?;
            anyhow::ensure!(
                node.children().map_or(0, <[SexpNode]>::len) == 3,
                "{kind} ({tag} …) must hold exactly X and Y"
            );
            Ok((number(node, 1, tag)?, number(node, 2, tag)?))
        };
        Ok(match kind {
            "gr_poly" => {
                let pts = node.find("pts").context("gr_poly has no (pts …)")?;
                let points = pts
                    .children()
                    .unwrap_or_default()
                    .iter()
                    .skip(1)
                    .map(|vertex| match vertex.head() {
                        Some("xy") => {
                            anyhow::ensure!(
                                vertex.children().map_or(0, <[SexpNode]>::len) == 3,
                                "gr_poly (xy …) must hold exactly X and Y"
                            );
                            Ok((number(vertex, 1, "xy")?, number(vertex, 2, "xy")?))
                        }
                        Some("arc") => anyhow::bail!(
                            "gr_poly has a curved (arc …) edge, which is not supported"
                        ),
                        other => anyhow::bail!(
                            "gr_poly point '{}' is not supported",
                            other.unwrap_or("<atom>")
                        ),
                    })
                    .collect::<anyhow::Result<Vec<_>>>()?;
                IpcPadPrimitive::Polygon {
                    points,
                    width,
                    filled,
                }
            }
            "gr_rect" => IpcPadPrimitive::Rect {
                start: point("start")?,
                end: point("end")?,
                width,
                filled,
            },
            "gr_circle" => IpcPadPrimitive::Circle {
                center: point("center")?,
                end: point("end")?,
                width,
                filled,
            },
            "gr_line" => IpcPadPrimitive::Segment {
                start: point("start")?,
                end: point("end")?,
                width,
            },
            _ => IpcPadPrimitive::Arc {
                start: point("start")?,
                mid: point("mid")?,
                end: point("end")?,
                width,
            },
        })
    }

    fn number(node: &SexpNode, index: usize, label: &str) -> anyhow::Result<f64> {
        let value = node
            .get_f64(index)
            .with_context(|| format!("{label} is missing a number"))?;
        anyhow::ensure!(value.is_finite(), "{label} value {value} is not finite");
        Ok(value)
    }
}

// ─── Footprint graphics extraction ───────────────────────────────────────────

/// `(start x y)`-style point child of a graphic node.
fn graphic_point(
    node: &konnect_sexp::SexpNode,
    tag: &str,
    kind: &str,
) -> anyhow::Result<(f64, f64)> {
    let point = node
        .find(tag)
        .ok_or_else(|| anyhow::anyhow!("footprint {kind} is missing its ({tag} …)"))?;
    Ok((
        point
            .get_f64(1)
            .ok_or_else(|| anyhow::anyhow!("footprint {kind} has an invalid {tag} X"))?,
        point
            .get_f64(2)
            .ok_or_else(|| anyhow::anyhow!("footprint {kind} has an invalid {tag} Y"))?,
    ))
}

fn graphic_layer(node: &konnect_sexp::SexpNode, kind: &str) -> anyhow::Result<String> {
    node.find_str("layer")
        .map(str::to_string)
        .ok_or_else(|| anyhow::anyhow!("footprint {kind} is missing its layer"))
}

/// Stroke width in mm: modern `(stroke (width w) …)`, legacy bare `(width w)`.
/// KiCad's default silkscreen line width stands in when neither is present.
fn graphic_stroke_width(node: &konnect_sexp::SexpNode) -> f64 {
    node.find("stroke")
        .and_then(|stroke| stroke.find_f64("width"))
        .or_else(|| node.find_f64("width"))
        .unwrap_or(0.12)
}

/// `(fill yes)` (KiCad 8+) or legacy `(fill solid)`.
fn graphic_filled(node: &konnect_sexp::SexpNode) -> bool {
    matches!(node.find_str("fill"), Some("yes") | Some("solid"))
}

/// `(hide yes)` (modern) or a bare `hide` atom (legacy).
fn text_hidden(node: &konnect_sexp::SexpNode) -> bool {
    node.find_str("hide") == Some("yes")
        || node
            .children()
            .unwrap_or_default()
            .iter()
            .any(|child| child.as_str() == Some("hide"))
}

/// `(effects (font (size h w)))` glyph size, defaulting to KiCad's 1 mm.
fn text_size(node: &konnect_sexp::SexpNode) -> f64 {
    node.find("effects")
        .and_then(|effects| effects.find("font"))
        .and_then(|font| font.find("size"))
        .and_then(|size| size.get_f64(1))
        .unwrap_or(1.0)
}

/// Explicit font stroke width, falling back to KiCad's 15%-of-size default.
fn text_stroke_width(node: &konnect_sexp::SexpNode) -> f64 {
    node.find("effects")
        .and_then(|effects| effects.find("font"))
        .and_then(|font| font.find_f64("thickness"))
        .unwrap_or_else(|| text_size(node) * 0.15)
}

/// Text position and angle from `(at x y [rot])`.
fn text_at(node: &konnect_sexp::SexpNode, kind: &str) -> anyhow::Result<((f64, f64), f64)> {
    let at = node
        .find("at")
        .ok_or_else(|| anyhow::anyhow!("footprint {kind} is missing its position"))?;
    Ok((
        (
            at.get_f64(1)
                .ok_or_else(|| anyhow::anyhow!("footprint {kind} has an invalid X position"))?,
            at.get_f64(2)
                .ok_or_else(|| anyhow::anyhow!("footprint {kind} has an invalid Y position"))?,
        ),
        at.get_f64(3).unwrap_or(0.0),
    ))
}

/// Parse a footprint's drawable children — `fp_line`, `fp_rect`, `fp_circle`,
/// `fp_arc`, `fp_poly` and visible `fp_text`/`property` texts — into
/// footprint-local [`konnect_ipc::IpcGraphicDefinition`]s.
///
/// The typed placement path previously shipped pads only, so a placed part had
/// no courtyard, silkscreen, or fab drawing: courtyard DRC had nothing to
/// check and KiCad's `lib_footprint_mismatch` flagged every placement.
///
/// `Reference` and `Value` properties are excluded — `build_footprint_item`
/// already carries those as first-class fields.
/// Footprint-local Reference/Value text anchors from the library source, so
/// placed parts keep the library's text layout (a synthesized offset put the
/// Reference on the part's own silkscreen — silk_overlap in live DRC).
pub(crate) fn extract_field_placement(source: &str) -> konnect_ipc::IpcFieldPlacement {
    let mut placement = konnect_ipc::IpcFieldPlacement::default();
    let Ok(footprint) = konnect_sexp::parse_sexp(source) else {
        return placement;
    };
    for prop in footprint.find_all("property") {
        let Some(name) = prop.get(1).and_then(|n| n.as_str()) else {
            continue;
        };
        let Some(at) = prop.find("at") else {
            continue;
        };
        let num = |i: usize| {
            at.get(i)
                .and_then(|v| v.as_str())
                .and_then(|s| s.parse::<f64>().ok())
        };
        let (x, y) = (num(1), num(2));
        let rot = num(3).unwrap_or(0.0);
        if let (Some(x), Some(y)) = (x, y) {
            match name {
                "Reference" => placement.reference_at = Some((x, y, rot)),
                "Value" => placement.value_at = Some((x, y, rot)),
                _ => {}
            }
        }
    }
    placement
}

pub(crate) fn extract_graphic_definitions(
    source: &str,
) -> anyhow::Result<Vec<konnect_ipc::IpcGraphicDefinition>> {
    extract_graphic_definitions_with_properties(source, true)
}

pub(crate) fn extract_graphic_definitions_without_properties(
    source: &str,
) -> anyhow::Result<Vec<konnect_ipc::IpcGraphicDefinition>> {
    extract_graphic_definitions_with_properties(source, false)
}

fn extract_graphic_definitions_with_properties(
    source: &str,
    include_properties: bool,
) -> anyhow::Result<Vec<konnect_ipc::IpcGraphicDefinition>> {
    use konnect_ipc::IpcGraphicDefinition as Graphic;
    let footprint = konnect_sexp::parse_sexp(source)?;
    let mut graphics = Vec::new();

    for line in footprint.find_all("fp_line") {
        graphics.push(Graphic::Line {
            start: graphic_point(line, "start", "fp_line")?,
            end: graphic_point(line, "end", "fp_line")?,
            layer: graphic_layer(line, "fp_line")?,
            width: graphic_stroke_width(line),
        });
    }
    for rect in footprint.find_all("fp_rect") {
        graphics.push(Graphic::Rect {
            start: graphic_point(rect, "start", "fp_rect")?,
            end: graphic_point(rect, "end", "fp_rect")?,
            layer: graphic_layer(rect, "fp_rect")?,
            width: graphic_stroke_width(rect),
            filled: graphic_filled(rect),
        });
    }
    for circle in footprint.find_all("fp_circle") {
        graphics.push(Graphic::Circle {
            center: graphic_point(circle, "center", "fp_circle")?,
            end: graphic_point(circle, "end", "fp_circle")?,
            layer: graphic_layer(circle, "fp_circle")?,
            width: graphic_stroke_width(circle),
            filled: graphic_filled(circle),
        });
    }
    for arc in footprint.find_all("fp_arc") {
        graphics.push(Graphic::Arc {
            start: graphic_point(arc, "start", "fp_arc")?,
            mid: graphic_point(arc, "mid", "fp_arc")?,
            end: graphic_point(arc, "end", "fp_arc")?,
            layer: graphic_layer(arc, "fp_arc")?,
            width: graphic_stroke_width(arc),
        });
    }
    for poly in footprint.find_all("fp_poly") {
        let pts = poly
            .find("pts")
            .ok_or_else(|| anyhow::anyhow!("footprint fp_poly is missing its (pts …)"))?;
        let points = pts
            .children()
            .unwrap_or_default()
            .iter()
            .filter(|node| node.head() == Some("xy"))
            .map(|node| {
                Ok((
                    node.get_f64(1)
                        .ok_or_else(|| anyhow::anyhow!("footprint fp_poly has an invalid X"))?,
                    node.get_f64(2)
                        .ok_or_else(|| anyhow::anyhow!("footprint fp_poly has an invalid Y"))?,
                ))
            })
            .collect::<anyhow::Result<Vec<_>>>()?;
        graphics.push(Graphic::Poly {
            points,
            layer: graphic_layer(poly, "fp_poly")?,
            width: graphic_stroke_width(poly),
            filled: graphic_filled(poly),
        });
    }
    for text in footprint.find_all("fp_text") {
        let kind = text.get(1).and_then(konnect_sexp::SexpNode::as_str);
        if matches!(kind, Some("reference" | "value")) || text_hidden(text) {
            continue;
        }
        let content = text
            .get(2)
            .and_then(konnect_sexp::SexpNode::as_str)
            .ok_or_else(|| anyhow::anyhow!("footprint fp_text is missing its text"))?;
        let (position, rotation) = text_at(text, "fp_text")?;
        graphics.push(Graphic::Text {
            text: content.to_string(),
            position,
            rotation,
            layer: graphic_layer(text, "fp_text")?,
            size: text_size(text),
            stroke_width_mm: text_stroke_width(text),
        });
    }
    let properties = if include_properties {
        footprint.find_all("property")
    } else {
        Vec::new()
    };
    for property in properties {
        let name = property.get(1).and_then(konnect_sexp::SexpNode::as_str);
        // Reference and Value travel as first-class fields; hidden built-ins
        // (Footprint, Datasheet, …) are not drawn.
        if matches!(name, Some("Reference") | Some("Value")) || text_hidden(property) {
            continue;
        }
        let Some(content) = property.get(2).and_then(konnect_sexp::SexpNode::as_str) else {
            continue;
        };
        let Ok((position, rotation)) = text_at(property, "property") else {
            continue;
        };
        let Ok(layer) = graphic_layer(property, "property") else {
            continue;
        };
        graphics.push(Graphic::Text {
            text: content.to_string(),
            position,
            rotation,
            layer,
            size: text_size(property),
            stroke_width_mm: text_stroke_width(property),
        });
    }
    Ok(graphics)
}

// ─── Library footprint → board footprint (file-editing fallback) ─────────────
//
// Used ONLY when the IPC transport is unreachable (unconfigured socket or
// failed dial/send): a live KiCad must never have the board file edited
// behind its back. Ported from emolitor's PR #66.

/// Build a board-ready `(footprint …)` block for `lib_id`.
///
/// A library `.kicad_mod` is a complete footprint definition sitting at the
/// origin with a `REF**` placeholder reference. Placing it on a board means
/// renaming it to the full `Library:Footprint` id, stamping in a position,
/// rotation and fresh UUID, and substituting the real reference designator.
///
/// KiCAD's own parser then handles the pads and graphics, which is why the
/// whole definition is forwarded rather than reconstructed.
fn board_footprint_sexp(
    lib_id: &str,
    x: f64,
    y: f64,
    rotation: f64,
    layer: &str,
    reference: Option<&str>,
    project_dir: Option<&Path>,
) -> Result<String, String> {
    let path = resolve_footprint_path(lib_id, project_dir)?;
    let content = std::fs::read_to_string(&path)
        .map_err(|e| format!("Cannot read footprint {}: {}", path.display(), e))?;

    let name_span = footprint_name_span(&content).ok_or_else(|| {
        format!(
            "{} does not start with a (footprint \"NAME\" …) block",
            path.display()
        )
    })?;

    // Board footprints carry the full library id, not the bare footprint name.
    // The declared name is the span without its surrounding quotes.
    let declared = &content[name_span.start + 1..name_span.end - 1];
    let mut out = String::with_capacity(content.len() + 128);
    out.push_str(&content[..name_span.start]);
    out.push_str(&quote_sexp_string(&board_lib_id(lib_id, &path, declared)));
    out.push_str(&format!(
        "\n\t(at {x} {y} {rotation})\n\t(uuid \"{}\")",
        new_uuid()
    ));
    out.push_str(&content[name_span.end..]);

    if rotation != 0.0 {
        out = apply_rotation_to_children(&out, rotation);
    }
    if let Some(reference) = reference {
        replace_reference(&mut out, reference).map_err(|error| error.to_string())?;
    }
    if layer != "F.Cu" {
        out = replace_footprint_layer(&out, layer);
    }

    Ok(out)
}

/// The name a board entry should carry for a footprint read from `path`.
///
/// `resolve_footprint_path` also accepts a bare filesystem path, which is
/// convenient for a caller holding a `.kicad_mod` directly. That path must not
/// reach the board file: `(footprint "C:\…\R_0805_2012Metric.kicad_mod")` is
/// not a library identifier, and KiCad reports the placed part as a broken
/// library link. This function is therefore total — every branch returns
/// something that is not a path.
///
/// Preference order, most authoritative first:
///
/// 1. The caller already gave a `Library:Footprint` id — use it verbatim.
/// 2. The fp-lib-table maps a nickname to the containing directory. Only the
///    table can answer this: KiCad lets any nickname point at any path, so
///    `MyParts` may well live in `vendor.pretty`, and guessing from the
///    directory would silently mislink the part.
/// 3. The conventional `<nickname>.pretty/` layout. The library is not
///    registered, so the link will be broken either way, but this is the
///    nickname the user gets when they do register it.
/// 4. Neither — fall back to a bare footprint name, which links to nothing but
///    is at least a valid name. The library file's own is used when it is not
///    itself path-like; otherwise the file stem, which cannot contain a
///    separator.
fn board_lib_id(reference: &str, path: &Path, declared: &str) -> String {
    if is_lib_id(reference) {
        return reference.to_string();
    }
    let stem = path
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_default();

    if let Some(dir) = path.parent() {
        if let Some(nick) = footprint_lib_nickname_for_dir(dir) {
            return format!("{nick}:{stem}");
        }
        if let Some(nick) = pretty_dir_nickname(dir) {
            return format!("{nick}:{stem}");
        }
    }

    if declared.is_empty() || declared.contains('/') || declared.contains('\\') {
        stem
    } else {
        declared.to_string()
    }
}

/// The nickname a conventional `<nickname>.pretty` directory implies.
///
/// Matched case-insensitively: KiCad's own libraries are lowercase `.pretty`,
/// but Windows and macOS filesystems are case-insensitive, so a `.Pretty` on
/// disk is the same directory to KiCad and should not change the answer.
fn pretty_dir_nickname(dir: &Path) -> Option<String> {
    let name = dir.file_name()?.to_string_lossy().into_owned();
    let cut = name.len().checked_sub(".pretty".len())?;
    name[cut..]
        .eq_ignore_ascii_case(".pretty")
        .then(|| name[..cut].to_string())
        .filter(|nick| !nick.is_empty())
}

/// Fold the footprint's placement rotation into its pads and text items.
///
/// KiCad stores each pad's and text item's *absolute* orientation while their
/// positions stay in unrotated footprint-local coordinates — a `C_0603` placed
/// at -90° keeps `(at -0.775 0 270)` on pad 1. Omitting this leaves the pad
/// shapes unrotated relative to the body and makes KiCad's
/// `lib_footprint_mismatch` check fire.
///
/// Text is additionally kept readable: KiCad flips an angle that would leave a
/// label upside down by 180°, so a -90° footprint carries `90` on its reference.
fn apply_rotation_to_children(content: &str, rotation: f64) -> String {
    let mut out = content.to_string();

    for tag in ["pad", "property", "fp_text"] {
        let readable = tag != "pad";
        // Rewrite back-to-front so earlier byte offsets stay valid.
        let starts: Vec<usize> = find_block_starts(&out, tag);
        for start in starts.into_iter().rev() {
            let Some((bstart, bend)) = find_balanced_block(&out, start) else {
                continue;
            };
            // The block's own `(at …)` is its first — nested ones (a pad's
            // `(primitives …)`, for instance) come later.
            let Some(at_start) = find_block_starts(&out[bstart..bend], "at")
                .first()
                .map(|i| bstart + i)
            else {
                continue;
            };
            let Some((astart, aend)) = find_balanced_block(&out, at_start) else {
                continue;
            };
            let Some(rewritten) = rotate_at_block(&out[astart..aend], rotation, readable) else {
                continue;
            };
            out.replace_range(astart..aend, &rewritten);
        }
    }
    out
}

/// Rewrite `(at x y [angle])`, adding `rotation` to the angle.
///
/// Returns `None` when the block does not look like a positional `at`.
fn rotate_at_block(block: &str, rotation: f64, readable: bool) -> Option<String> {
    let inner = block.strip_prefix('(')?.strip_suffix(')')?;
    let mut parts = inner.split_whitespace();
    if parts.next()? != "at" {
        return None;
    }
    let x: f64 = parts.next()?.parse().ok()?;
    let y: f64 = parts.next()?.parse().ok()?;
    let existing: f64 = parts.next().and_then(|a| a.parse().ok()).unwrap_or(0.0);
    if parts.next().is_some() {
        return None; // `(at …)` with unexpected extra tokens — leave alone.
    }

    let mut angle = (existing + rotation).rem_euclid(360.0);
    if readable && angle > 90.0 && angle <= 270.0 {
        angle -= 180.0;
    }
    Some(format_at(x, y, angle))
}

/// Normalise a footprint's *root* orientation the way KiCad's writer does,
/// to (-180, 180].
///
/// Measured, not assumed: rotating `R1` to 247.5° through the closed-board
/// path and then letting KiCad re-save the board, KiCad wrote `-112.5` for the
/// root `(at …)` and left both pad angles at `247.5` untouched. Same angle,
/// different spelling — but writing the un-normalised form means the user's
/// next save in KiCad produces a diff on a footprint nobody touched, and it
/// makes the file path disagree with the IPC path, which ends up normalised
/// because KiCad itself does it.
///
/// Deliberately not applied to children: KiCad does not normalise those.
fn normalize_root_angle(degrees: f64) -> f64 {
    let wrapped = degrees.rem_euclid(360.0);
    if wrapped > 180.0 {
        wrapped - 360.0
    } else {
        wrapped
    }
}

/// Render `(at x y angle)`, dropping a zero angle as KiCad's writer does and
/// trimming trailing zeros from the decimals.
fn format_at(x: f64, y: f64, angle: f64) -> String {
    let n = |v: f64| {
        let s = format!("{v:.6}");
        let s = s.trim_end_matches('0').trim_end_matches('.').to_string();
        if s == "-0" {
            "0".to_string()
        } else {
            s
        }
    };
    if angle == 0.0 {
        format!("(at {} {})", n(x), n(y))
    } else {
        format!("(at {} {} {})", n(x), n(y), n(angle))
    }
}

/// Byte range of the quoted name in the leading `(footprint "NAME"` header,
/// including the surrounding quotes.
fn footprint_name_span(content: &str) -> Option<std::ops::Range<usize>> {
    let block = *find_block_starts(content, "footprint").first()?;
    let after_tag = block + "(footprint".len();
    let rel = content[after_tag..].find('"')?;
    let start = after_tag + rel;
    let end = start + 1 + content[start + 1..].find('"')?;
    Some(start..end + 1)
}

/// Quote and escape `value` as an S-expression string literal.
fn quote_sexp_string(value: &str) -> String {
    format!("\"{}\"", escape_sexp_string(value))
}

/// Replace the footprint's own `(layer "…")` — the first `layer` block that is a
/// direct child of the footprint, not one belonging to a pad or graphic.
///
/// Note this only retargets the footprint; a true F.Cu↔B.Cu flip would also
/// have to mirror every child item, which is why back-side placement is
/// rejected before this code can run (see `back_side_layer_error`).
fn replace_footprint_layer(content: &str, layer: &str) -> String {
    let Some(name) = footprint_name_span(content) else {
        return content.to_string();
    };
    let Some(start) = find_block_starts(content, "layer")
        .into_iter()
        .find(|&i| i > name.end)
    else {
        return content.to_string();
    };
    let Some((bstart, bend)) = find_balanced_block(content, start) else {
        return content.to_string();
    };

    let mut out = String::with_capacity(content.len());
    out.push_str(&content[..bstart]);
    out.push_str(&format!("(layer {})", quote_sexp_string(layer)));
    out.push_str(&content[bend..]);
    out
}

/// Insert `blocks` just inside the board's closing paren and write it back,
/// refusing to write anything that is not one complete `(kicad_pcb …)` form.
///
/// The insert point is `rfind(')')`, which is only the right place if the file
/// really is a single closed form. Checking the result before committing it
/// means a board that was already truncated — or a footprint block that was —
/// fails loudly instead of being written back over the user's file in a state
/// KiCad can no longer open.
///
/// Like the rest of `konnect-sexp`, this treats parens as syntax everywhere: a
/// `#`-commented paren would be miscounted. KiCad does not write comments into
/// `.kicad_pcb`, and no reader in this workspace understands them either, so
/// the assumption is at least consistent.
fn insert_into_board(board_path: &Path, blocks: &[String]) -> anyhow::Result<()> {
    let content = read_consistent(board_path)?;
    let existing_references = footprint_references(&content)?;
    let mut inserted_references = HashSet::new();
    for block in blocks {
        for reference in footprint_references(block)? {
            if existing_references.contains(&reference)
                || !inserted_references.insert(reference.clone())
            {
                anyhow::bail!(
                    "Footprint reference '{}' already exists on the board",
                    reference
                );
            }
        }
    }
    // KiCad writes these files CRLF on Windows — its bundled .kicad_mod
    // libraries are CRLF throughout — so an inserted block joined with bare LF
    // would leave the board with two conventions in it.
    let eol = if content.contains("\r\n") {
        "\r\n"
    } else {
        "\n"
    };
    let close_pos = content.rfind(')').unwrap_or(content.len());
    let joined: String = blocks
        .iter()
        .map(|b| format!("{eol}{}", indent_block(b.trim_end(), "\t", eol)))
        .collect();
    let new_content = apply_edits(content.clone(), vec![SexpEdit::insert(close_pos, joined)]);

    if let Err(why) = check_single_board_form(&new_content) {
        anyhow::bail!(
            "Refusing to write {}: {}. The board file was left untouched.",
            board_path.display(),
            why
        );
    }

    persist_board_replacement(board_path, &content, &new_content)?;
    Ok(())
}

/// One prepared footprint placement shared by the IPC and guarded file paths.
///
/// Callers choose whether the definition came from a KiCad library or from
/// deliberately supplied inline geometry, but they do not orchestrate the
/// parser, IPC extraction, board serialization, and duplicate-safe insertion
/// helpers independently.
#[derive(Clone)]
pub(crate) struct PreparedFootprintPlacement {
    pub(crate) value: String,
    pub(crate) pads: Vec<konnect_ipc::IpcPadDefinition>,
    pub(crate) graphics: Vec<konnect_ipc::IpcGraphicDefinition>,
    pub(crate) fields: konnect_ipc::IpcFieldPlacement,
    /// The library's attributes, description, keywords and 3D models; `None`
    /// for inline geometry, which has no library to carry them.
    library: Option<super::library_footprint::LibraryData>,
    board_sexp: String,
}

impl PreparedFootprintPlacement {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn from_library(
        source: &str,
        lib_id: &str,
        reference: &str,
        value: Option<&str>,
        x: f64,
        y: f64,
        rotation: f64,
        layer: &str,
        project_dir: Option<&Path>,
    ) -> anyhow::Result<Self> {
        let prepared =
            prepare_footprint_source(source, lib_id, reference, value, x, y, rotation, layer)?;
        let pads = extract_pad_definitions(&prepared)?;
        let graphics = extract_graphic_definitions(&prepared)?;
        let fields = extract_field_placement(&prepared);
        let library =
            super::library_footprint::LibraryData::read(&konnect_sexp::parse_sexp(&prepared)?)
                .map_err(|error| {
                    anyhow::anyhow!("footprint '{lib_id}' library data cannot be placed: {error:#}")
                })?;
        let board_sexp =
            board_footprint_sexp(lib_id, x, y, rotation, layer, Some(reference), project_dir)
                .map_err(anyhow::Error::msg)?;
        let value = value.map(str::to_string).unwrap_or_else(|| {
            lib_id
                .split_once(':')
                .map_or(lib_id, |(_, entry)| entry)
                .to_string()
        });
        Ok(Self {
            value,
            pads,
            graphics,
            fields,
            library: Some(library),
            board_sexp,
        })
    }

    pub(crate) fn from_inline(
        value: String,
        pads: Vec<konnect_ipc::IpcPadDefinition>,
        graphics: Vec<konnect_ipc::IpcGraphicDefinition>,
        fields: konnect_ipc::IpcFieldPlacement,
        board_sexp: String,
    ) -> Self {
        Self {
            value,
            pads,
            graphics,
            fields,
            library: None,
            board_sexp,
        }
    }

    /// The typed item the IPC path creates: built from the pads, drawings and
    /// fields, then given the library's attributes, description, keywords and
    /// 3D models when there is a library. The library source is only read.
    pub(crate) fn build_item(
        &self,
        lib_id: &str,
        reference: &str,
        x: f64,
        y: f64,
        rotation: f64,
        layer: &str,
    ) -> anyhow::Result<prost_types::Any> {
        let item = KiCadIpcClient::build_footprint_item(
            lib_id,
            reference,
            &self.value,
            &self.pads,
            &self.graphics,
            &self.fields,
            x,
            y,
            rotation,
            layer,
        )?;
        let Some(library) = &self.library else {
            return Ok(item);
        };
        let mut footprint =
            konnect_ipc::gen::kiapi::board::types::FootprintInstance::decode(item.value.as_slice())
                .context("typed footprint builder returned an invalid item")?;
        library.apply(&mut footprint)?;
        Ok(konnect_ipc::builders::pack_any(
            &footprint,
            "kiapi.board.types.FootprintInstance",
        ))
    }

    /// Insert exactly this footprint through the same atomic, duplicate-safe
    /// path used by ordinary library placement.
    pub(crate) fn insert_into_board(&self, board_path: &Path) -> anyhow::Result<()> {
        insert_into_board(board_path, std::slice::from_ref(&self.board_sexp))
    }
}

fn footprint_references(content: &str) -> anyhow::Result<HashSet<String>> {
    let root = konnect_sexp::parse_sexp(content)?;
    let footprints: Vec<&konnect_sexp::SexpNode> = if root.head() == Some("footprint") {
        vec![&root]
    } else if root.head() == Some("kicad_pcb") {
        root.find_all("footprint")
    } else {
        anyhow::bail!("expected a footprint or kicad_pcb root");
    };

    Ok(footprints
        .into_iter()
        .filter_map(footprint_reference)
        .collect())
}

fn board_contains_reference(board_path: &Path, reference: &str) -> anyhow::Result<bool> {
    let content = read_consistent(board_path)?;
    if let Err(reason) = check_single_board_form(&content) {
        anyhow::bail!(
            "Refusing to read {}: {}. The board file is not balanced and was left untouched.",
            board_path.display(),
            reason
        );
    }
    Ok(footprint_references(&content)?.contains(reference))
}

pub(crate) fn footprint_reference(footprint: &konnect_sexp::SexpNode) -> Option<String> {
    footprint
        .find_all("property")
        .into_iter()
        .find(|property| {
            property.get(1).and_then(konnect_sexp::SexpNode::as_str) == Some("Reference")
        })
        .and_then(|property| property.get(2))
        .and_then(konnect_sexp::SexpNode::as_str)
        .or_else(|| {
            footprint
                .find_all("fp_text")
                .into_iter()
                .find(|text| {
                    text.get(1).and_then(konnect_sexp::SexpNode::as_str) == Some("reference")
                })
                .and_then(|text| text.get(2))
                .and_then(konnect_sexp::SexpNode::as_str)
        })
        .map(str::to_string)
}

fn persist_board_replacement(
    board_path: &Path,
    expected: &str,
    replacement: &str,
) -> Result<(), konnect_sexp::SexpError> {
    write_atomic_if_unchanged(board_path, expected, replacement)
}

#[derive(Clone, Copy)]
enum FootprintPlacementUpdate {
    Move { x: f64, y: f64 },
    Rotate { rotation: f64 },
    Set { x: f64, y: f64, rotation: f64 },
}

/// Why a closed-board placement update could not be applied.
///
/// The handlers have to tell a caller's mistake — a reference that is not on
/// this board — from a board Konnect declines to edit, from a genuine I/O
/// failure, and report each differently. Deciding that by matching on the
/// error's message text is exactly what [`with_board_ipc_classified`]'s contract
/// forbids two screens up, and it left every case except the missing
/// reference surfacing as an unstructured `handler_error` (#194's class).
#[derive(Debug)]
pub(crate) enum ClosedBoardError {
    /// No footprint on this board carries that reference.
    ReferenceNotFound(String),
    /// More than one does, so "the" footprint is ambiguous.
    ReferenceAmbiguous(String),
    /// The board is not a shape this tool will edit, before or after.
    Unusable(String),
    /// Reading or writing failed, or the file changed under us.
    Io(anyhow::Error),
}

impl ClosedBoardError {
    /// The result to hand back. Never `Err`: every one of these is something
    /// the caller can act on, and all of them leave the board untouched.
    pub(crate) fn into_result(self) -> CallToolResult {
        match self {
            Self::ReferenceNotFound(reference) => CallToolResult::error_kind(
                crate::mcp::error::ToolErrorKind::InvalidArgument {
                    field: "reference".to_string(),
                    reason: format!("no footprint '{reference}' on this board"),
                },
                format!(
                    "Footprint '{reference}' is not on this board, so there was nothing to \
                     update. The board file was not modified."
                ),
            ),
            Self::ReferenceAmbiguous(reference) => CallToolResult::error_kind(
                crate::mcp::error::ToolErrorKind::InvalidArgument {
                    field: "reference".to_string(),
                    reason: format!("'{reference}' appears more than once on this board"),
                },
                format!(
                    "Footprint reference '{reference}' appears more than once on the board, so \
                     it does not identify one footprint. The board file was not modified — \
                     give the duplicates distinct references first."
                ),
            ),
            Self::Unusable(reason) => CallToolResult::error(format!(
                "Refusing to edit the board: {reason}. The board file was not modified."
            )),
            Self::Io(error) => CallToolResult::error(format!("{error:#}")),
        }
    }
}

fn update_closed_board_footprint(
    board_path: &Path,
    reference: &str,
    update: FootprintPlacementUpdate,
) -> Result<(), ClosedBoardError> {
    let content = read_consistent(board_path).map_err(|e| ClosedBoardError::Io(e.into()))?;
    let updated = prepare_closed_board_footprint_update(&content, reference, update)?;
    persist_board_replacement(board_path, &content, &updated)
        .map_err(|e| ClosedBoardError::Io(e.into()))?;
    Ok(())
}

pub(crate) fn update_closed_board_footprints(
    board_path: &Path,
    placements: &[konnect_ipc::types::IpcFootprintPlacement],
) -> Result<Vec<konnect_ipc::types::IpcFootprintPlacement>, ClosedBoardError> {
    update_closed_board_footprints_on_sides(board_path, placements, &vec![None; placements.len()])
        .map(|(applied, _)| applied)
}

/// As [`update_closed_board_footprints`], first moving each footprint given
/// a side to it with the closed-board flip (which mirrors its geometry and
/// refuses anything it cannot mirror), then setting the requested position and
/// absolute rotation. One write for the whole batch, or none.
fn update_closed_board_footprints_on_sides(
    board_path: &Path,
    placements: &[konnect_ipc::types::IpcFootprintPlacement],
    sides: &[Option<String>],
) -> Result<
    (
        Vec<konnect_ipc::types::IpcFootprintPlacement>,
        Vec<serde_json::Value>,
    ),
    ClosedBoardError,
> {
    let content =
        read_consistent(board_path).map_err(|error| ClosedBoardError::Io(error.into()))?;
    let mut updated = content.clone();
    let mut layers = Vec::with_capacity(placements.len());
    for (placement, side) in placements.iter().zip(sides) {
        let mut flipped = false;
        if let Some(layer) = side {
            let (next, changed) =
                prepare_closed_board_footprint_side(&updated, &placement.reference, layer)?;
            updated = next;
            flipped = changed;
        }
        layers.push(json!({ "reference": placement.reference, "flipped": flipped }));
        updated = prepare_closed_board_footprint_update(
            &updated,
            &placement.reference,
            FootprintPlacementUpdate::Set {
                x: placement.x,
                y: placement.y,
                rotation: placement.rotation,
            },
        )?;
    }
    persist_board_replacement(board_path, &content, &updated)
        .map_err(|error| ClosedBoardError::Io(error.into()))?;
    // Report what was written, not what was asked: the file path normalizes
    // the root angle to KiCad's (-180, 180] (a requested 270 is stored as
    // -90), and the response has to say the number the file now holds.
    let mut applied = Vec::with_capacity(placements.len());
    for (index, placement) in placements.iter().enumerate() {
        let root_at = find_direct_child_blocks(&updated, "kicad_pcb")
            .into_iter()
            .filter_map(|(start, end)| konnect_sexp::parse_sexp(&updated[start..end]).ok())
            .filter(|node| node.head() == Some("footprint"))
            .find(|node| footprint_reference(node).as_deref() == Some(&placement.reference))
            .and_then(|node| {
                let at = node.find("at")?;
                layers[index]["layer"] = json!(node.find_str("layer"));
                Some((at.get_f64(1)?, at.get_f64(2)?, at.get_f64(3).unwrap_or(0.0)))
            })
            .ok_or_else(|| {
                ClosedBoardError::Unusable(format!(
                    "footprint '{}' was updated but cannot be read back from the written board",
                    placement.reference
                ))
            })?;
        applied.push(konnect_ipc::types::IpcFootprintPlacement {
            reference: placement.reference.clone(),
            x: root_at.0,
            y: root_at.1,
            rotation: root_at.2,
        });
    }
    Ok((applied, layers))
}

fn prepare_closed_board_footprint_update(
    content: &str,
    reference: &str,
    update: FootprintPlacementUpdate,
) -> Result<String, ClosedBoardError> {
    if let Err(reason) = check_single_board_form(content) {
        return Err(ClosedBoardError::Unusable(reason.to_string()));
    }

    let mut matched = None;
    for (start, end) in find_direct_child_blocks(content, "kicad_pcb") {
        let block = &content[start..end];
        let footprint = match konnect_sexp::parse_sexp(block) {
            Ok(node) if node.head() == Some("footprint") => node,
            _ => continue,
        };
        if footprint_reference(&footprint).as_deref() == Some(reference)
            && matched.replace((start, end)).is_some()
        {
            return Err(ClosedBoardError::ReferenceAmbiguous(reference.to_string()));
        }
    }

    let (start, end) =
        matched.ok_or_else(|| ClosedBoardError::ReferenceNotFound(reference.to_string()))?;
    let replacement = update_footprint_placement(&content[start..end], update)
        .map_err(|error| ClosedBoardError::Unusable(format!("{error:#}")))?;
    let updated = apply_edits(
        content.to_string(),
        vec![SexpEdit::replace(start, end, replacement)],
    );
    // Defensive, and deliberately kept despite having no reachable trigger:
    // `update_footprint_placement` already refuses anything whose root is not
    // a `footprint`, and `apply_edits` swaps one balanced block for another,
    // so there is no input that reaches here with a broken board. Neutering it
    // changes no test, which is the honest state of affairs — it exists to
    // make "we never write a board we did not just re-validate" true by
    // construction rather than by tracing every caller.
    if let Err(reason) = check_single_board_form(&updated) {
        return Err(ClosedBoardError::Unusable(format!(
            "updating '{reference}' would have produced {reason}"
        )));
    }
    Ok(updated)
}

fn update_footprint_placement(
    footprint: &str,
    update: FootprintPlacementUpdate,
) -> anyhow::Result<String> {
    let at_ranges: Vec<_> = find_direct_child_blocks(footprint, "footprint")
        .into_iter()
        .filter(|(start, end)| {
            konnect_sexp::parse_sexp(&footprint[*start..*end])
                .ok()
                .is_some_and(|node| node.head() == Some("at"))
        })
        .collect();
    let [(at_start, at_end)] = at_ranges.as_slice() else {
        anyhow::bail!("footprint must contain exactly one root placement (at ...) block");
    };
    let at = konnect_sexp::parse_sexp(&footprint[*at_start..*at_end])?;
    let old_x = at
        .get_f64(1)
        .context("footprint root placement has an invalid X position")?;
    let old_y = at
        .get_f64(2)
        .context("footprint root placement has an invalid Y position")?;
    let old_rotation = at.get_f64(3).unwrap_or(0.0);

    // A move preserves the existing orientation *exactly as spelled* — it is
    // not this tool's business to renormalise an angle the caller did not ask
    // to change. A rotation writes KiCad's spelling of the new one.
    let (x, y, rotation) = match update {
        FootprintPlacementUpdate::Move { x, y } => (x, y, old_rotation),
        FootprintPlacementUpdate::Rotate { rotation } => {
            (old_x, old_y, normalize_root_angle(rotation))
        }
        FootprintPlacementUpdate::Set { x, y, rotation } => (x, y, normalize_root_angle(rotation)),
    };

    // Replace the root `(at …)` FIRST, while `at_start`/`at_end` still index
    // the string they were measured against.
    //
    // `apply_rotation_to_children` rewrites the `(at …)` inside every pad,
    // property and fp_text, and those replacements change length — an angle
    // can gain digits, or reach zero and be dropped entirely. Rotating first
    // and splicing after left the root offsets stale by that delta whenever a
    // child `(at …)` preceded the root one, which is legal S-expression even
    // though KiCad's own writer does not emit it. A −45° property angle
    // rotating to 0 shortens by four bytes and lands the splice inside the
    // preceding block: `(at (at 10 20 75) (pad "1" …`.
    //
    // The two passes are safe in this order because they touch disjoint
    // blocks: the root `(at …)` is a direct child of `footprint`, and
    // `apply_rotation_to_children` only rewrites the first `(at …)` *inside* a
    // pad/property/fp_text — and it recomputes its own offsets against the
    // string it is given.
    let updated = apply_edits(
        footprint.to_string(),
        vec![SexpEdit::replace(
            *at_start,
            *at_end,
            format_at(x, y, rotation),
        )],
    );
    let updated = match update {
        FootprintPlacementUpdate::Move { .. } => updated,
        FootprintPlacementUpdate::Rotate { rotation }
        | FootprintPlacementUpdate::Set { rotation, .. } => {
            apply_rotation_to_children(&updated, rotation - old_rotation)
        }
    };
    let parsed = konnect_sexp::parse_sexp(&updated)?;
    if parsed.head() != Some("footprint") {
        anyhow::bail!("placement update changed the footprint root");
    }
    Ok(updated)
}

fn direct_children_with_tags(
    source: &str,
    parent_tag: &str,
) -> anyhow::Result<Vec<(usize, usize, String)>> {
    find_direct_child_blocks(source, parent_tag)
        .into_iter()
        .map(|(start, end)| {
            let node = konnect_sexp::parse_sexp(&source[start..end])?;
            let tag = node
                .head()
                .context("direct child has no S-expression tag")?
                .to_string();
            Ok((start, end, tag))
        })
        .collect()
}

/// Whether `source` has at least one direct `(child_tag …)`, without caring
/// how many or failing on a block that has none.
fn has_direct_child(source: &str, parent_tag: &str, child_tag: &str) -> bool {
    direct_children_with_tags(source, parent_tag)
        .map(|children| children.iter().any(|(_, _, tag)| tag == child_tag))
        .unwrap_or(false)
}

fn exactly_one_direct_child(
    source: &str,
    parent_tag: &str,
    child_tag: &str,
) -> anyhow::Result<(usize, usize)> {
    let matches: Vec<_> = direct_children_with_tags(source, parent_tag)?
        .into_iter()
        .filter_map(|(start, end, tag)| (tag == child_tag).then_some((start, end)))
        .collect();
    let [(start, end)] = matches.as_slice() else {
        anyhow::bail!("{parent_tag} must contain exactly one direct ({child_tag} ...) block");
    };
    Ok((*start, *end))
}

fn at_components(block: &str) -> anyhow::Result<(f64, f64, f64, Vec<String>)> {
    let at = konnect_sexp::parse_sexp(block)?;
    if at.head() != Some("at") {
        anyhow::bail!("expected an (at ...) block");
    }
    let x = at
        .get_f64(1)
        .context("(at ...) has an invalid X position")?;
    let y = at
        .get_f64(2)
        .context("(at ...) has an invalid Y position")?;
    let children = at.children().unwrap_or_default();
    let angle = at.get_f64(3).unwrap_or(0.0);
    let suffix_start = if at.get_f64(3).is_some() { 4 } else { 3 };
    let suffix = children
        .iter()
        .skip(suffix_start)
        .map(|child| {
            child
                .as_str()
                .map(str::to_string)
                .context("(at ...) contains a non-atomic suffix")
        })
        .collect::<anyhow::Result<Vec<_>>>()?;
    Ok((x, y, angle, suffix))
}

fn format_at_with_suffix(x: f64, y: f64, angle: f64, suffix: &[String]) -> String {
    let mut formatted = format_at(x, y, angle);
    if !suffix.is_empty() {
        formatted.insert_str(formatted.len() - 1, &format!(" {}", suffix.join(" ")));
    }
    formatted
}

fn format_xy(tag: &str, x: f64, y: f64) -> String {
    let at = format_at(x, y, 0.0);
    format!("({tag}{})", &at["(at".len()..at.len() - 1])
}

fn normalize_angle(angle: f64) -> f64 {
    angle.rem_euclid(360.0)
}

fn normalize_angle_180(angle: f64) -> f64 {
    let normalized = normalize_angle(angle);
    if normalized > 180.0 {
        normalized - 360.0
    } else {
        normalized
    }
}

fn flipped_layer(layer: &str) -> anyhow::Result<String> {
    const SIDE_PAIRS: &[(&str, &str)] = &[
        ("F.Cu", "B.Cu"),
        ("F.Adhes", "B.Adhes"),
        ("F.Paste", "B.Paste"),
        ("F.SilkS", "B.SilkS"),
        ("F.Mask", "B.Mask"),
        ("F.CrtYd", "B.CrtYd"),
        ("F.Fab", "B.Fab"),
    ];
    for (front, back) in SIDE_PAIRS {
        if layer == *front {
            return Ok((*back).to_string());
        }
        if layer == *back {
            return Ok((*front).to_string());
        }
    }
    if layer.starts_with("F.") || layer.starts_with("B.") {
        anyhow::bail!("unsupported side-specific KiCad layer '{layer}'");
    }
    Ok(layer.to_string())
}

fn flip_layer_block(block: &str) -> anyhow::Result<String> {
    let layer = konnect_sexp::parse_sexp(block)?;
    let name = layer
        .get(1)
        .and_then(konnect_sexp::SexpNode::as_str)
        .context("(layer ...) has no layer name")?;
    Ok(format!(
        "(layer {})",
        quote_sexp_string(&flipped_layer(name)?)
    ))
}

fn flip_layers_block(block: &str) -> anyhow::Result<String> {
    let layers = konnect_sexp::parse_sexp(block)?;
    let names = layers
        .children()
        .unwrap_or_default()
        .iter()
        .skip(1)
        .map(|child| {
            let name = child
                .as_str()
                .context("(layers ...) contains a non-atomic layer name")?;
            flipped_layer(name).map(|flipped| quote_sexp_string(&flipped))
        })
        .collect::<anyhow::Result<Vec<_>>>()?;
    Ok(format!("(layers {})", names.join(" ")))
}

fn toggle_text_mirror(effects: &str) -> anyhow::Result<String> {
    let justify = direct_children_with_tags(effects, "effects")?
        .into_iter()
        .filter_map(|(start, end, tag)| (tag == "justify").then_some((start, end)))
        .collect::<Vec<_>>();
    match justify.as_slice() {
        [] => Ok(apply_edits(
            effects.to_string(),
            vec![SexpEdit::insert(effects.len() - 1, " (justify mirror)")],
        )),
        [(start, end)] => {
            let node = konnect_sexp::parse_sexp(&effects[*start..*end])?;
            let mut values = node
                .children()
                .unwrap_or_default()
                .iter()
                .skip(1)
                .map(|child| {
                    child
                        .as_str()
                        .map(str::to_string)
                        .context("(justify ...) contains a non-atomic value")
                })
                .collect::<anyhow::Result<Vec<_>>>()?;
            if let Some(index) = values.iter().position(|value| value == "mirror") {
                values.remove(index);
            } else {
                values.push("mirror".to_string());
            }
            let replacement = if values.is_empty() {
                String::new()
            } else {
                format!("(justify {})", values.join(" "))
            };
            Ok(apply_edits(
                effects.to_string(),
                vec![SexpEdit::replace(*start, *end, replacement)],
            ))
        }
        _ => anyhow::bail!("text effects contain duplicate (justify ...) blocks"),
    }
}

fn flip_text_block(block: &str, tag: &str) -> anyhow::Result<String> {
    let (at_start, at_end) = exactly_one_direct_child(block, tag, "at")?;
    let (x, y, angle, suffix) = at_components(&block[at_start..at_end])?;
    let (layer_start, layer_end) = exactly_one_direct_child(block, tag, "layer")?;
    let layer_block = &block[layer_start..layer_end];
    let layer = konnect_sexp::parse_sexp(layer_block)?;
    let layer_name = layer
        .get(1)
        .and_then(konnect_sexp::SexpNode::as_str)
        .context("(layer ...) has no layer name")?;
    let flipped_layer_name = flipped_layer(layer_name)?;
    let (effects_start, effects_end) = exactly_one_direct_child(block, tag, "effects")?;
    let effects = if flipped_layer_name == layer_name {
        block[effects_start..effects_end].to_string()
    } else {
        toggle_text_mirror(&block[effects_start..effects_end])?
    };
    Ok(apply_edits(
        block.to_string(),
        vec![
            SexpEdit::replace(
                at_start,
                at_end,
                format_at_with_suffix(x, -y, normalize_angle(180.0 - angle), &suffix),
            ),
            SexpEdit::replace(
                layer_start,
                layer_end,
                format!("(layer {})", quote_sexp_string(&flipped_layer_name)),
            ),
            SexpEdit::replace(effects_start, effects_end, effects),
        ],
    ))
}

fn flip_graphic_block(block: &str, tag: &str) -> anyhow::Result<String> {
    let point_tags: &[&str] = match tag {
        "fp_line" | "fp_rect" => &["start", "end"],
        "fp_circle" => &["center", "end"],
        "fp_arc" => &["start", "mid", "end"],
        _ => anyhow::bail!("unsupported footprint graphic '{tag}'"),
    };
    let mut edits = Vec::new();
    let mut points = Vec::new();
    for point_tag in point_tags {
        let (start, end) = exactly_one_direct_child(block, tag, point_tag)?;
        let point = konnect_sexp::parse_sexp(&block[start..end])?;
        points.push((
            start,
            end,
            point
                .get_f64(1)
                .with_context(|| format!("({point_tag} ...) has an invalid X position"))?,
            -point
                .get_f64(2)
                .with_context(|| format!("({point_tag} ...) has an invalid Y position"))?,
        ));
    }
    let source_order: Vec<usize> = if tag == "fp_arc" {
        vec![2, 1, 0]
    } else {
        (0..points.len()).collect()
    };
    for (target_index, point_tag) in point_tags.iter().enumerate() {
        let (target_start, target_end, _, _) = points[target_index];
        let (_, _, x, y) = points[source_order[target_index]];
        edits.push(SexpEdit::replace(
            target_start,
            target_end,
            format_xy(point_tag, x, y),
        ));
    }
    let (layer_start, layer_end) = exactly_one_direct_child(block, tag, "layer")?;
    edits.push(SexpEdit::replace(
        layer_start,
        layer_end,
        flip_layer_block(&block[layer_start..layer_end])?,
    ));
    Ok(apply_edits(block.to_string(), edits))
}

fn flip_poly_block(block: &str) -> anyhow::Result<String> {
    let (pts_start, pts_end) = exactly_one_direct_child(block, "fp_poly", "pts")?;
    let pts = &block[pts_start..pts_end];
    let mut point_edits = Vec::new();
    for (start, end, tag) in direct_children_with_tags(pts, "pts")? {
        if tag != "xy" {
            anyhow::bail!("fp_poly contains unsupported point block '{tag}'");
        }
        let point = konnect_sexp::parse_sexp(&pts[start..end])?;
        let x = point.get_f64(1).context("(xy ...) has an invalid X")?;
        let y = point.get_f64(2).context("(xy ...) has an invalid Y")?;
        point_edits.push(SexpEdit::replace(start, end, format_xy("xy", x, -y)));
    }
    let mirrored_pts = apply_edits(pts.to_string(), point_edits);
    let (layer_start, layer_end) = exactly_one_direct_child(block, "fp_poly", "layer")?;
    Ok(apply_edits(
        block.to_string(),
        vec![
            SexpEdit::replace(pts_start, pts_end, mirrored_pts),
            SexpEdit::replace(
                layer_start,
                layer_end,
                flip_layer_block(&block[layer_start..layer_end])?,
            ),
        ],
    ))
}

fn contains_descendant_tag(node: &konnect_sexp::SexpNode, tag: &str) -> bool {
    node.children().is_some_and(|children| {
        children
            .iter()
            .any(|child| child.head() == Some(tag) || contains_descendant_tag(child, tag))
    })
}

fn flip_pad_block(block: &str) -> anyhow::Result<String> {
    let pad = konnect_sexp::parse_sexp(block)?;
    if pad.get(3).and_then(konnect_sexp::SexpNode::as_str) == Some("custom") {
        anyhow::bail!("custom pads are not supported by closed-board footprint flipping");
    }
    for unsupported in ["offset", "rect_delta", "chamfer_ratio", "primitives"] {
        if contains_descendant_tag(&pad, unsupported) {
            anyhow::bail!(
                "pad geometry containing ({unsupported} ...) is not supported by closed-board footprint flipping"
            );
        }
    }
    let (at_start, at_end) = exactly_one_direct_child(block, "pad", "at")?;
    let (x, y, angle, suffix) = at_components(&block[at_start..at_end])?;
    let (layers_start, layers_end) = exactly_one_direct_child(block, "pad", "layers")?;
    Ok(apply_edits(
        block.to_string(),
        vec![
            SexpEdit::replace(
                at_start,
                at_end,
                format_at_with_suffix(x, -y, normalize_angle(-angle), &suffix),
            ),
            SexpEdit::replace(
                layers_start,
                layers_end,
                flip_layers_block(&block[layers_start..layers_end])?,
            ),
        ],
    ))
}

/// Refuse a `(model …)` whose placement a flip would have to move.
///
/// This guard applies only to the closed-board file-fallback path. An open
/// board flips through KiCad's own native `FlipItems` IPC command (10.0.6+),
/// which transforms a model's Y offset and X/Y rotation correctly, so that
/// path never reaches this function. The closed-board path directly edits
/// the `.kicad_pcb` file and leaves `(model …)` untouched, which is silently
/// wrong for any model where those are non-zero. Rather than guess the
/// transform — reproducing KiCad's own `FOOTPRINT::Flip` 3D-model math is out
/// of scope for the file fallback (issue #604) — this refuses, consistent
/// with how the rest of this path treats geometry it cannot mirror.
///
/// The cost of refusing is close to nothing. Across all **14,818** footprints
/// in KiCad 10's standard libraries that carry a `(model …)`: `offset.y` is
/// non-zero in **3**, and `rotate.x`/`rotate.y` in **none**. The three are
/// `RaspberryPi_Pico_Common_THT` (-24.13 mm, badly wrong if ignored) and two
/// sub-0.04 mm cases. The 84 footprints with a non-zero `rotate.z` are
/// unaffected either way, since a flip does not touch Z.
fn refuse_model_a_flip_would_move(block: &str) -> anyhow::Result<()> {
    let model = konnect_sexp::parse_sexp(block)?;
    for (tag, fields) in [("offset", ["x", "y", "z"]), ("rotate", ["x", "y", "z"])] {
        let Some(node) = model.find_all(tag).into_iter().next() else {
            continue;
        };
        let Some(xyz) = node.find_all("xyz").into_iter().next() else {
            continue;
        };
        // A flip negates offset.y, rotate.x and rotate.y; Z and offset.x ride
        // along unchanged, so a non-zero value there is not a problem.
        let moved: &[usize] = if tag == "offset" { &[2] } else { &[1, 2] };
        for &index in moved {
            let value = xyz.get_f64(index).unwrap_or(0.0);
            if value != 0.0 {
                anyhow::bail!(
                    "the 3D model's {tag}.{} is {value}, and a flip would have to move it; \
                     flipping this footprint would leave the model where it was",
                    fields[index - 1]
                );
            }
        }
    }
    Ok(())
}

fn flip_footprint_block(footprint: &str) -> anyhow::Result<String> {
    let root = konnect_sexp::parse_sexp(footprint)?;
    if root.head() != Some("footprint") {
        anyhow::bail!("expected a footprint root");
    }
    let (root_at_start, root_at_end) = exactly_one_direct_child(footprint, "footprint", "at")?;
    let (x, y, angle, suffix) = at_components(&footprint[root_at_start..root_at_end])?;
    let (root_layer_start, root_layer_end) =
        exactly_one_direct_child(footprint, "footprint", "layer")?;
    let mut edits = vec![
        SexpEdit::replace(
            root_at_start,
            root_at_end,
            format_at_with_suffix(x, y, normalize_angle_180(-angle), &suffix),
        ),
        SexpEdit::replace(
            root_layer_start,
            root_layer_end,
            flip_layer_block(&footprint[root_layer_start..root_layer_end])?,
        ),
    ];

    for (start, end, tag) in direct_children_with_tags(footprint, "footprint")? {
        let block = &footprint[start..end];
        let replacement = match tag.as_str() {
            // A property with no `(at …)` carries no geometry, so there is
            // nothing to mirror and it passes through untouched.
            //
            // This is not an edge case. KiCad writes
            // `(property ki_fp_filters "R_* Resistor_*")` — a bare token, no
            // position, no layer — into every footprint it places from a
            // library: 779 of them across the 19 boards shipped in
            // `share/kicad/demos`. Requiring exactly one `(at …)` on every
            // property therefore refused practically every real board with
            // "property must contain exactly one direct (at ...) block",
            // which is what the first live run of this tool hit. The
            // synthetic fixture has only positioned properties, so nothing
            // offline could have caught it.
            "property" if !has_direct_child(block, "property", "at") => None,
            "property" | "fp_text" => Some(flip_text_block(block, &tag)?),
            "fp_line" | "fp_rect" | "fp_circle" | "fp_arc" => {
                Some(flip_graphic_block(block, &tag)?)
            }
            "fp_poly" => Some(flip_poly_block(block)?),
            "pad" => Some(flip_pad_block(block)?),
            "model" => {
                refuse_model_a_flip_would_move(block)?;
                None
            }
            unsupported if unsupported.starts_with("fp_") || unsupported == "zone" => {
                anyhow::bail!(
                    "unsupported footprint child '{unsupported}' prevents a safe closed-board flip"
                )
            }
            _ => None,
        };
        if let Some(replacement) = replacement {
            edits.push(SexpEdit::replace(start, end, replacement));
        }
    }

    let flipped = apply_edits(footprint.to_string(), edits);
    let parsed = konnect_sexp::parse_sexp(&flipped)?;
    if parsed.head() != Some("footprint") {
        anyhow::bail!("flip changed the footprint root");
    }
    Ok(flipped)
}

fn footprint_layer(footprint: &str) -> anyhow::Result<String> {
    let (start, end) = exactly_one_direct_child(footprint, "footprint", "layer")?;
    let layer = konnect_sexp::parse_sexp(&footprint[start..end])?;
    layer
        .get(1)
        .and_then(konnect_sexp::SexpNode::as_str)
        .map(str::to_string)
        .context("footprint layer has no name")
}

fn prepare_closed_board_footprint_side(
    content: &str,
    reference: &str,
    target_layer: &str,
) -> Result<(String, bool), ClosedBoardError> {
    if let Err(reason) = check_single_board_form(content) {
        return Err(ClosedBoardError::Unusable(reason.to_string()));
    }
    let mut matched = None;
    for (start, end, tag) in
        direct_children_with_tags(content, "kicad_pcb").map_err(ClosedBoardError::Io)?
    {
        if tag != "footprint" {
            continue;
        }
        let footprint = konnect_sexp::parse_sexp(&content[start..end])
            .map_err(|e| ClosedBoardError::Io(e.into()))?;
        if footprint_reference(&footprint).as_deref() == Some(reference)
            && matched.replace((start, end)).is_some()
        {
            return Err(ClosedBoardError::ReferenceAmbiguous(reference.to_string()));
        }
    }
    let (start, end) =
        matched.ok_or_else(|| ClosedBoardError::ReferenceNotFound(reference.to_string()))?;
    let block = &content[start..end];
    let current_layer = footprint_layer(block).map_err(ClosedBoardError::Io)?;
    if !matches!(current_layer.as_str(), "F.Cu" | "B.Cu") {
        return Err(ClosedBoardError::Unusable(format!(
            "footprint '{reference}' sits on root layer '{current_layer}', which is neither \
             side of the board"
        )));
    }
    if current_layer == target_layer {
        return Ok((content.to_string(), false));
    }
    let flipped = flip_footprint_block(block)
        .map_err(|error| ClosedBoardError::Unusable(format!("{error:#}")))?;
    if footprint_layer(&flipped).map_err(ClosedBoardError::Io)? != target_layer {
        return Err(ClosedBoardError::Unusable(format!(
            "flipping '{reference}' did not produce target layer '{target_layer}'"
        )));
    }
    let updated = apply_edits(
        content.to_string(),
        vec![SexpEdit::replace(start, end, flipped)],
    );
    if let Err(reason) = check_single_board_form(&updated) {
        return Err(ClosedBoardError::Unusable(format!(
            "flipping '{reference}' would have produced {reason}"
        )));
    }
    Ok((updated, true))
}

fn set_closed_board_footprint_side(
    board_path: &Path,
    reference: &str,
    target_layer: &str,
) -> Result<bool, ClosedBoardError> {
    let content = read_consistent(board_path).map_err(|e| ClosedBoardError::Io(e.into()))?;
    let (updated, changed) =
        prepare_closed_board_footprint_side(&content, reference, target_layer)?;
    if changed {
        persist_board_replacement(board_path, &content, &updated)
            .map_err(|e| ClosedBoardError::Io(e.into()))?;
    }
    Ok(changed)
}

/// Verify `content` is exactly one `(kicad_pcb …)` form and nothing else.
///
/// Checking only that *a* balanced block exists is too weak to back the promise
/// above: `find_balanced_block` skips whatever precedes the first paren, so
/// leading garbage would pass, as would a well-formed form that is not a board
/// at all.
pub(crate) fn check_single_board_form(content: &str) -> Result<(), String> {
    let trimmed = content.trim();
    let (start, end) = find_balanced_block(trimmed, 0)
        .ok_or_else(|| "the result is not a balanced S-expression".to_string())?;

    if start != 0 {
        return Err(format!(
            "{} bytes of content precede the opening paren",
            start
        ));
    }
    if end != trimmed.len() {
        return Err(format!(
            "{} bytes of content follow the closing paren",
            trimmed.len() - end
        ));
    }
    if !trimmed[1..].trim_start().starts_with("kicad_pcb") {
        return Err("the root expression is not (kicad_pcb …)".to_string());
    }
    Ok(())
}

/// Prefix every non-empty line with `indent`, joining them with `eol`.
pub(crate) fn indent_block(block: &str, indent: &str, eol: &str) -> String {
    // `lines()` strips a trailing \r along with the \n, so rejoining with `eol`
    // re-imposes one convention on a block that may have arrived with another —
    // a CRLF library footprint going into an LF board, or the reverse.
    block
        .lines()
        .map(|l| {
            if l.trim().is_empty() {
                String::new()
            } else {
                format!("{indent}{l}")
            }
        })
        .collect::<Vec<_>>()
        .join(eol)
}

/// The Reference-field presentation values `edit_component.reference_field`
/// and `set_reference_field_presentations` share; see `reference_field`.
pub(crate) fn reference_field_schema(description: &str) -> serde_json::Value {
    json!({
        "type": "object",
        "description": format!("{description} {}", reference_field::CONVENTION),
        "properties": {
            "local_x": { "type": "number", "description": "Saved (at …) X offset from the footprint anchor, footprint frame, mm" },
            "local_y": { "type": "number", "description": "Saved (at …) Y offset from the footprint anchor, footprint frame, mm" },
            "board_x": { "type": "number", "description": "Board-space X, mm; with board_y, instead of local_x/local_y" },
            "board_y": { "type": "number", "description": "Board-space Y, mm; with board_x" },
            "angle_deg": { "type": "number", "description": "Text board angle as KiCad saves it" },
            "relative_angle_deg": { "type": "number", "description": "Text angle relative to the footprint angle; instead of angle_deg" },
            "width_mm": { "type": "number", "exclusiveMinimum": 0, "maximum": 100 },
            "height_mm": { "type": "number", "exclusiveMinimum": 0, "maximum": 100 },
            "stroke_mm": { "type": "number", "exclusiveMinimum": 0, "maximum": 25 }
        }
    })
}

// ─── Tool definitions ─────────────────────────────────────────────────────────

pub fn tools() -> Vec<ToolDef> {
    let mut tools = vec![
        tool!(
            "place_component",
            "Place a footprint on the PCB. Uses live KiCAD IPC when reachable; otherwise safely \
             edits a closed board file with revision checks.",
            json!({
                "type": "object",
                "properties": {
                    "board":      { "type": "string" },
                    "footprint":  { "type": "string", "description": "Library:Footprint (e.g. 'Resistor_SMD:R_0402')" },
                    "reference":  { "type": "string", "description": "Reference designator" },
                    "x":          { "type": "number" },
                    "y":          { "type": "number" },
                    "rotation":   { "type": "number", "default": 0 },
                    "layer":      { "type": "string", "default": "F.Cu" }
                },
                "required": ["board", "footprint", "reference", "x", "y"]
            }),
            |args, ctx| async move { handle_place_component(args, ctx).await }
        )
        .with_board_access(crate::tools::BoardAccess::LivePreferredWithFallback),
        tool!(
            "move_component",
            "Move a placed footprint to a new X/Y position. Uses live KiCAD IPC when reachable; \
             otherwise safely edits a closed board file with revision checks.",
            json!({
                "type": "object",
                "properties": {
                    "board":     { "type": "string" },
                    "reference": { "type": "string" },
                    "x":         { "type": "number" },
                    "y":         { "type": "number" }
                },
                "required": ["board", "reference", "x", "y"]
            }),
            |args, ctx| async move { handle_move_component(args, ctx).await }
        )
        .with_board_access(crate::tools::BoardAccess::LivePreferredWithFallback),
        tool!(
            "rotate_component",
            "Set the rotation angle of a placed footprint. Uses live KiCAD IPC when reachable; \
             otherwise safely edits a closed board file with revision checks.",
            json!({
                "type": "object",
                "properties": {
                    "board":     { "type": "string" },
                    "reference": { "type": "string" },
                    "rotation":  { "type": "number", "description": "Rotation angle in degrees" }
                },
                "required": ["board", "reference", "rotation"]
            }),
            |args, ctx| async move { handle_rotate_component(args, ctx).await }
        )
        .with_board_access(crate::tools::BoardAccess::LivePreferredWithFallback),
        tool!(
            "set_component_placements",
            "Set X/Y positions and rotations for multiple existing footprints atomically. Uses one live KiCAD IPC update and one undo step when reachable; otherwise safely edits a closed board file once with revision checks. A placement may name a target layer (F.Cu or B.Cu) on a closed board only: that footprint is mirrored first, then placed; any unsupported footprint refuses the whole batch.",
            json!({
                "type": "object",
                "properties": {
                    "board": { "type": "string" },
                    "placements": {
                        "type": "array",
                        "minItems": 1,
                        "items": {
                            "type": "object",
                            "properties": {
                                "reference": { "type": "string" },
                                "x": { "type": "number", "description": "Target X coordinate in millimetres" },
                                "y": { "type": "number", "description": "Target Y coordinate in millimetres" },
                                "rotation": { "type": "number", "description": "Target absolute rotation in degrees" },
                                "layer": { "type": "string", "enum": ["F.Cu", "B.Cu"], "description": "Optional target side, closed board only: a footprint on the other side is first mirrored by the closed-board flip (which refuses custom pads, pad offsets/primitives, moving 3D models and unsupported children), then set to x/y/rotation. Any layer in the batch refuses the whole batch while KiCad holds the board." }
                            },
                            "required": ["reference", "x", "y", "rotation"]
                        }
                    }
                },
                "required": ["board", "placements"]
            }),
            |args, ctx| async move { handle_set_component_placements(args, ctx).await }
        )
        .with_board_access(crate::tools::BoardAccess::LivePreferredWithFallback),
        tool!(
            "flip_component",
            "Set a placed footprint to F.Cu or B.Cu. When the board is open in KiCad 10.0.6 or \
             newer, this uses KiCad's own native FlipItems IPC command — the same transform the \
             GUI's F key performs, including 3D-model offset/rotation — inside one KiCad undo \
             step. Older or unsupported KiCad reachable over IPC returns an explicit \
             unsupported_capability error rather than falling back to the file. Only when no \
             live KiCad holds this board does it fall back to a guarded closed-board file edit, \
             which flips supported geometry with revision checks and refuses any footprint whose \
             3D model would need a transform the file path cannot perform.",
            json!({
                "type": "object",
                "properties": {
                    "board":     { "type": "string" },
                    "reference": { "type": "string" },
                    "layer":     { "type": "string", "enum": ["F.Cu", "B.Cu"] }
                },
                "required": ["board", "reference", "layer"]
            }),
            |args, ctx| async move { handle_flip_component(args, ctx).await }
        )
        .with_board_access(crate::tools::BoardAccess::LivePreferredWithFallback),
        tool!(
            "delete_component",
            "Remove a footprint from the board via KiCAD IPC.",
            json!({
                "type": "object",
                "properties": {
                    "board":     { "type": "string" },
                    "reference": { "type": "string" }
                },
                "required": ["board", "reference"]
            }),
            |args, ctx| async move { handle_delete_component(args, ctx).await }
        )
        .with_board_access(crate::tools::BoardAccess::LiveOnly),
        tool!(
            "edit_component",
            "Update a footprint value through live KiCad IPC, or inspect/edit its assembly attributes or its Reference field's presentation in the saved board while KiCad does not hold it. Attribute and reference_field edits preserve every other footprint clause and use an atomic revision-checked write. Give only one of value, attributes and reference_field per call. For many Reference fields in one write use set_reference_field_presentations.",
            json!({
                "type": "object",
                "properties": {
                    "board":     { "type": "string" },
                    "reference": { "type": "string" },
                    "value":     { "type": "string", "description": "New value string (optional)" },
                    "attributes": {
                        "type": "object", "additionalProperties": false,
                        "description": "Requires the exact board closed. Set only named flags; omitted flags and unknown existing attributes are preserved. Empty object inspects saved attributes without a write.",
                        "properties": {
                            "board_only": { "type": "boolean" },
                            "exclude_from_bom": { "type": "boolean" },
                            "exclude_from_pos_files": { "type": "boolean" }
                        }
                    },
                    "reference_field": reference_field_schema(
                        "Requires the exact board closed. Set the saved Reference field's local position, text angle, character width/height and stroke; omitted values are kept, and an empty object inspects the saved values without a write."
                    )
                },
                "required": ["board", "reference"]
            }),
            |args, ctx| async move { handle_edit_component(args, ctx).await }
        )
        .with_board_access(crate::tools::BoardAccess::ApplyModeDependent),
        tool!(
            "repair_corrupted_footprints",
            "Repair the exact legacy corruption from Konnect issue #244: footprint drawing \
             shapes rewritten as anonymous pads with empty layer sets. Resolves each affected \
             footprint from its library, restores its pads and drawing shapes while preserving \
             placement, identity and pad nets, and applies every requested repair in one KiCad \
             undo commit. Defaults to a non-mutating dry run; apply requires the exact returned \
             plan revision. Requires KiCAD running with the requested board open.",
            json!({
                "type": "object",
                "properties": {
                    "board": { "type": "string" },
                    "references": {
                        "type": "array",
                        "description": "Optional reference-designator allowlist. Omit to scan every footprint.",
                        "items": { "type": "string" }
                    },
                    "dry_run": {
                        "type": "boolean",
                        "default": true,
                        "description": "Report the repair plan without changing the board."
                    },
                    "expected_plan_revision": {
                        "type": "string",
                        "description": "Exact plan_revision returned by a current dry run; required for apply."
                    }
                },
                "required": ["board"]
            }),
            |args, ctx| async move { handle_repair_corrupted_footprints(args, ctx).await }
        )
        .with_board_access(crate::tools::BoardAccess::LiveOnly),
        tool!(
            "find_component",
            "Find a footprint on the board by reference designator and return its position.",
            json!({
                "type": "object",
                "properties": {
                    "board":     { "type": "string" },
                    "reference": { "type": "string" }
                },
                "required": ["board", "reference"]
            }),
            |args, ctx| async move { handle_find_component(args, ctx).await }
        )
        .with_board_access(crate::tools::BoardAccess::LiveOnly),
        super::pcb_footprint_update::tool(),
        tool!(
            "list_board_footprint_graphics",
            "List the graphic items inside a footprint placed on the board — silkscreen, fabrication, and courtyard artwork — with the UUID needed to edit one. Points are footprint-local millimetres, as the .kicad_mod shows them. Each item reports 'editable', plus 'outlines' and 'holes' for polygons: 'points' covers the first outline only, so an item with more than one outline or any holes is reported but cannot be edited here. Requires KiCAD running with the board open.",
            json!({
                "type": "object",
                "properties": {
                    "board":     { "type": "string" },
                    "reference": { "type": "string", "description": "Reference designator, e.g. 'J2'" },
                    "layer":     { "type": "string", "description": "Only list items on this layer, e.g. 'F.SilkS' (optional)" }
                },
                "required": ["board", "reference"]
            }),
            |args, ctx| async move { handle_list_board_footprint_graphics(args, ctx).await }
        )
        .with_board_access(crate::tools::BoardAccess::LiveOnly),
        tool!(
            "edit_board_footprint_graphic",
            "Replace the vertices of a polygon inside a footprint placed on the board, selected by UUID. Points are footprint-local millimetres, as the .kicad_mod shows them. Use this to bring one placed instance in line with a library change without re-placing the part. Only a single-outline polygon with no holes can be replaced; anything else is refused by name rather than flattened. Requires KiCAD running with the board open.",
            json!({
                "type": "object",
                "properties": {
                    "board":     { "type": "string" },
                    "reference": { "type": "string", "description": "Reference designator, e.g. 'J2'" },
                    "uuid":      { "type": "string", "description": "UUID of the graphic item, from list_board_footprint_graphics" },
                    "points": {
                        "type": "array",
                        "description": "Replacement vertices in footprint-local millimetres; at least 3.",
                        "items": {
                            "type": "object",
                            "properties": {
                                "x": { "type": "number" },
                                "y": { "type": "number" }
                            },
                            "required": ["x", "y"]
                        }
                    }
                },
                "required": ["board", "reference", "uuid", "points"]
            }),
            |args, ctx| async move { handle_edit_board_footprint_graphic(args, ctx).await }
        )
        .with_board_access(crate::tools::BoardAccess::LiveOnly),
        tool!(
            "get_component_pads",
            "Return live board-space pad positions, effective rotations, physical geometry, drills, \
             copper-layer shapes, layers and net assignments for a footprint. \
             Reads the board open in KiCad when it is reachable and falls back to the file only \
             when no live KiCad holds this board — IPC unreachable, or that board not open — \
             'source' says which, so unsaved placements are visible without a save. \
             A pad's 'net' is its net name, \"\" if the pad carries no net \
             (unconnected), or — reading the file — null if the net node is present \
             but unreadable; treat null as an error, not as an unconnected pad. \
             Pad fabrication properties (pad_prop_bga, pad_prop_heatsink, …) are reported \
             only from the saved file, as each pad's 'fabrication_property' (\"none\" when the \
             saved pad has none); KiCad 10's live typed API cannot observe them, so a live read \
             says 'unobservable_via_live_typed_api' rather than reporting them absent. \
             source 'saved' reads the saved file even while KiCad holds the board: it shows the \
             last save, not unsaved edits.",
            json!({
                "type": "object",
                "properties": {
                    "board":     { "type": "string" },
                    "reference": { "type": "string" },
                    "source": {
                        "type": "string",
                        "enum": ["auto", "saved"],
                        "default": "auto",
                        "description": "auto: live board when KiCad holds it, else the saved file. saved: always the saved file, with pad fabrication properties."
                    },
                    "detail": {
                        "type": "string",
                        "enum": ["full", "fabrication", "connectivity"],
                        "default": "full",
                        "description": "fabrication returns each pad number and fabrication property plus the source/observability facts, omitting geometry. connectivity returns each physical pad's number, net, board-space x/y, rotation, pad type, UUID and fabrication property, plus the footprint's own pose, UUID, layer and library ID; no shape, polygon, paste or 3D data. Use either for large parts; full retains the existing response."
                    },
                    "pad_numbers": {
                        "type": "array",
                        "items": { "type": "string" },
                        "description": "Only pads whose number is exactly one of these. Every physical pad with a matching number is returned, duplicates included; numbers that match none are listed in unmatched_pad_numbers."
                    },
                    "nets": {
                        "type": "array",
                        "items": { "type": "string" },
                        "description": "Only pads whose net name is exactly one of these (\"\" for unconnected); nets that match none are listed in unmatched_nets."
                    }
                },
                "required": ["board", "reference"]
            }),
            |args, ctx| async move { handle_get_component_pads(args, ctx).await }
        )
        .with_board_access(crate::tools::BoardAccess::LivePreferredWithFallback),
        tool!(
            "get_pad_position",
            "Return the live board-space position, effective rotation, physical geometry, drill, \
             layers and net of a specific pad number on a footprint.",
            json!({
                "type": "object",
                "properties": {
                    "board":       { "type": "string" },
                    "reference":   { "type": "string" },
                    "pad_number":  { "type": "string" }
                },
                "required": ["board", "reference", "pad_number"]
            }),
            |args, ctx| async move { handle_get_pad_position(args, ctx).await }
        )
        .with_board_access(crate::tools::BoardAccess::LivePreferredWithFallback),
        tool!(
            "get_component_list",
            "List footprints on the board with their positions, rotations, layers, values, \
             library IDs and UUIDs. source 'live' (default) reads the board KiCad holds open, \
             with no file fallback; source 'saved' reads the saved file even while KiCad holds \
             the board and says it excludes unsaved editor state. 'references' limits the \
             answer to those exact references: a reference no footprint carries is listed in \
             unknown_references, and one several footprints carry is listed in \
             duplicate_references with every footprint kept.",
            json!({
                "type": "object",
                "properties": {
                    "board": { "type": "string" },
                    "references": {
                        "type": "array",
                        "items": { "type": "string" },
                        "description": "Only these exact references; omit for every footprint."
                    },
                    "source": {
                        "type": "string",
                        "enum": ["live", "saved"],
                        "default": "live",
                        "description": "live: the exact board open in KiCad. saved: the saved file only."
                    }
                },
                "required": ["board"]
            }),
            |args, ctx| async move { handle_get_component_list(args, ctx).await }
        )
        .with_board_access(crate::tools::BoardAccess::LivePreferredWithFallback),
        tool!(
            "place_component_array",
            "Place multiple copies of a footprint in a grid or line array via KiCAD IPC.",
            json!({
                "type": "object",
                "properties": {
                    "board":        { "type": "string" },
                    "footprint":    { "type": "string" },
                    "start_x":      { "type": "number" },
                    "start_y":      { "type": "number" },
                    "count_x":      { "type": "integer", "description": "Number of columns" },
                    "count_y":      { "type": "integer", "description": "Number of rows", "default": 1 },
                    "spacing_x":    { "type": "number", "description": "Column spacing in mm" },
                    "spacing_y":    { "type": "number", "description": "Row spacing in mm", "default": 0 },
                    "ref_prefix":   { "type": "string", "description": "Reference prefix (e.g. 'R')", "default": "U" },
                    "ref_start":    { "type": "integer", "description": "Starting reference number", "default": 1 }
                },
                "required": ["board", "footprint", "start_x", "start_y", "count_x", "spacing_x"]
            }),
            |args, ctx| async move { handle_place_array(args, ctx).await }
        )
        .with_board_access(crate::tools::BoardAccess::LiveOnly),
        tool!(
            "align_components",
            "Align multiple footprints along a common X or Y axis via KiCAD IPC.",
            json!({
                "type": "object",
                "properties": {
                    "board":       { "type": "string" },
                    "references":  { "type": "array", "items": { "type": "string" } },
                    "axis":        { "type": "string", "description": "'x' or 'y'", "default": "x" },
                    "value":       { "type": "number", "description": "Target coordinate to align to" }
                },
                "required": ["board", "references", "value"]
            }),
            |args, ctx| async move { handle_align_components(args, ctx).await }
        )
        .with_board_access(crate::tools::BoardAccess::LiveOnly),
        tool!(
            "duplicate_component",
            "Duplicate an existing footprint at a new position via KiCAD IPC.",
            json!({
                "type": "object",
                "properties": {
                    "board":         { "type": "string" },
                    "reference":     { "type": "string", "description": "Reference to duplicate" },
                    "new_reference": { "type": "string", "description": "New reference designator" },
                    "x":             { "type": "number" },
                    "y":             { "type": "number" }
                },
                "required": ["board", "reference", "new_reference", "x", "y"]
            }),
            |args, ctx| async move { handle_duplicate_component(args, ctx).await }
        )
        .with_board_access(crate::tools::BoardAccess::LiveOnly),
        tool!(
            "get_board_2d_view",
            "Render the board with kicad-cli and return it as a base64 PNG. Note this is              kicad-cli's 3-D board render viewed from the top, not a layer plot -- there is              no layer selection. Use export_svg for layer-aware 2-D output.",
            json!({
                "type": "object",
                "properties": {
                    "board":  { "type": "string" },
                    "width":  { "type": "integer", "default": 800, "description": "Render width in pixels, clamped to 100-4000 (kept small since the image lands in LLM context, raise it when detail matters)" },
                    "height": { "type": "integer", "default": 600, "description": "Render height in pixels, clamped to 100-4000" }
                },
                "required": ["board"]
            }),
            |args, ctx| async move { handle_get_board_2d_view(args, ctx).await }
        ),
    ];
    tools.push(super::placed_footprint_models::tool());
    tools
}

// ─── Handlers ─────────────────────────────────────────────────────────────────

async fn handle_place_component(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let board = get_path(args, "board")?;
    let footprint = match require_str(args, "footprint") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let reference = match require_str(args, "reference") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let x = match require_f64(args, "x") {
        Ok(v) => v,
        Err(e) => return Ok(e),
    };
    let y = match require_f64(args, "y") {
        Ok(v) => v,
        Err(e) => return Ok(e),
    };
    let rotation = args["rotation"].as_f64().unwrap_or(0.0);
    let layer = args["layer"].as_str().unwrap_or("F.Cu").to_string();
    if let Some(rejection) = back_side_layer_error(&layer) {
        return Ok(rejection);
    }
    let source = match resolve_footprint_source(&footprint, &board) {
        Ok(source) => source,
        Err(error) => return Ok(CallToolResult::error(error.to_string())),
    };
    let prepared = match PreparedFootprintPlacement::from_library(
        &source,
        &footprint,
        &reference,
        None,
        x,
        y,
        rotation,
        &layer,
        board.parent(),
    ) {
        Ok(prepared) => prepared,
        Err(error) => return Ok(CallToolResult::error(format!("{error:#}"))),
    };

    // Try IPC first. The fallback gate is the typed transport classification:
    // only when the transport is unreachable and this server has never
    // observed the requested board live is the guarded file path available.
    // A KiCad that answered — even with an error — fails closed.
    let footprint_ipc = footprint.clone();
    let reference_ipc = reference.clone();
    let layer_ipc = layer.clone();
    let requested_board = board.clone();
    let prepared_ipc = prepared.clone();
    let attempt = attempt_ipc_write(ctx, &board, "placement", move |c| {
        let item =
            prepared_ipc.build_item(&footprint_ipc, &reference_ipc, x, y, rotation, &layer_ipc)?;
        c.place_footprint_item(&requested_board, &reference_ipc, item)
    })
    .await?;

    match attempt {
        BoardWrite::Ipc(fp) => Ok(CallToolResult::json(&json!({
            "placed": fp.reference,
            "footprint": fp.footprint,
            "x": fp.position.x, "y": fp.position.y,
            "rotation": fp.rotation, "layer": fp.layer,
            "source": "ipc"
        }))),
        BoardWrite::Refused(result) => Ok(result),
        BoardWrite::File(reason) => {
            // No live KiCad on the other end of this transport: fall back to
            // editing the board file directly.
            if board_contains_reference(&board, &reference)? {
                return Ok(CallToolResult::error_kind(
                    crate::mcp::error::ToolErrorKind::InvalidArgument {
                        field: "reference".to_string(),
                        reason: format!(
                            "footprint reference '{reference}' already exists on the board"
                        ),
                    },
                    format!("Footprint reference '{reference}' already exists on the board"),
                ));
            }
            prepared.insert_into_board(&board)?;
            Ok(CallToolResult::json(&json!({
                "placed": reference,
                "footprint": footprint,
                "x": x, "y": y, "rotation": rotation, "layer": layer,
                "source": "file",
                "fallback_reason": reason.evidence(),
                "warning": reason.warning()
            })))
        }
    }
}

async fn handle_move_component(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let board = get_path(args, "board")?;
    let reference = match require_str(args, "reference") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let x = match require_f64(args, "x") {
        Ok(v) => v,
        Err(e) => return Ok(e),
    };
    let y = match require_f64(args, "y") {
        Ok(v) => v,
        Err(e) => return Ok(e),
    };

    let ref_ipc = reference.clone();
    let guard_board = board.clone();
    match attempt_ipc_write(ctx, &board, "move", move |c| {
        if let Err(refusal) = guard_live_rebuild(
            c,
            &guard_board,
            "move_component",
            &[&ref_ipc],
            ClosedBoardPath,
        )? {
            return Ok(Err(refusal));
        }
        c.move_footprint(&ref_ipc, x, y).map(Ok)
    })
    .await?
    {
        BoardWrite::Ipc(Ok(())) => Ok(CallToolResult::json(
            &json!({ "moved": reference, "x": x, "y": y, "source": "ipc" }),
        )),
        BoardWrite::Ipc(Err(refusal)) => Ok(refusal.into_result()),
        BoardWrite::Refused(result) => Ok(result),
        BoardWrite::File(reason) => {
            match update_closed_board_footprint(
                &board,
                &reference,
                FootprintPlacementUpdate::Move { x, y },
            ) {
                Ok(()) => Ok(CallToolResult::json(&json!({
                    "moved": reference,
                    "x": x,
                    "y": y,
                    "source": "file",
                    "fallback_reason": reason.evidence(),
                    "warning": reason.warning()
                }))),
                Err(error) => Ok(error.into_result()),
            }
        }
    }
}

async fn handle_list_board_footprint_graphics(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let reference = match require_str(args, "reference") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let layer_filter = args["layer"].as_str().map(|s| s.to_string());

    let ref_ipc = reference.clone();
    let graphics: Vec<konnect_ipc::types::IpcFootprintGraphic> =
        ipc!(ctx, args, |c| c.list_footprint_graphics(&ref_ipc));

    let graphics: Vec<_> = graphics
        .into_iter()
        .filter(|g| layer_filter.as_deref().is_none_or(|l| g.layer == l))
        .collect();

    Ok(CallToolResult::json(&json!({
        "count": graphics.len(),
        "reference": reference,
        "graphics": graphics,
    })))
}

async fn handle_edit_board_footprint_graphic(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let reference = match require_str(args, "reference") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let uuid = match require_str(args, "uuid") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    // A malformed `points` is the caller's mistake, so it gets the structured
    // `invalid_argument` the rest of this file returns — bubbling `anyhow`
    // here would surface it as a handler error with the offending field
    // flattened into prose, which is the class #194 is open against.
    let points = match parse_points(&args["points"]) {
        Ok(p) => p,
        Err(e) => return Ok(e),
    };

    let (ref_ipc, uuid_ipc, pts) = (reference.clone(), uuid.clone(), points.clone());
    let guard_board = get_path(args, "board")?;
    let kind: Result<String, _> = ipc!(ctx, args, |c| {
        if let Err(refusal) = guard_live_rebuild(
            c,
            &guard_board,
            "edit_board_footprint_graphic",
            &[&ref_ipc],
            NoLosslessPath,
        )? {
            return Ok(Err(refusal));
        }
        c.set_footprint_graphic_points(&ref_ipc, &uuid_ipc, &pts)
            .map(Ok)
    });
    let kind = match kind {
        Ok(kind) => kind,
        Err(refusal) => return Ok(refusal.into_result()),
    };

    Ok(CallToolResult::json(&json!({
        "edited": reference,
        "uuid": uuid,
        "kind": kind,
        "points": points.len(),
    })))
}

async fn handle_rotate_component(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let board = get_path(args, "board")?;
    let reference = match require_str(args, "reference") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let rotation = match require_f64(args, "rotation") {
        Ok(v) => v,
        Err(e) => return Ok(e),
    };

    let ref_ipc = reference.clone();
    let guard_board = board.clone();
    match attempt_ipc_write(ctx, &board, "rotation", move |c| {
        if let Err(refusal) = guard_live_rebuild(
            c,
            &guard_board,
            "rotate_component",
            &[&ref_ipc],
            ClosedBoardPath,
        )? {
            return Ok(Err(refusal));
        }
        c.rotate_footprint(&ref_ipc, rotation).map(Ok)
    })
    .await?
    {
        BoardWrite::Ipc(Ok(())) => Ok(CallToolResult::json(&json!({
            "rotated": reference,
            "rotation": rotation,
            "source": "ipc"
        }))),
        BoardWrite::Ipc(Err(refusal)) => Ok(refusal.into_result()),
        BoardWrite::Refused(result) => Ok(result),
        BoardWrite::File(reason) => {
            match update_closed_board_footprint(
                &board,
                &reference,
                FootprintPlacementUpdate::Rotate { rotation },
            ) {
                Ok(()) => Ok(CallToolResult::json(&json!({
                    "rotated": reference,
                    "rotation": rotation,
                    "source": "file",
                    "fallback_reason": reason.evidence(),
                    "warning": reason.warning()
                }))),
                Err(error) => Ok(error.into_result()),
            }
        }
    }
}

fn invalid_placement(field: String, reason: impl Into<String>) -> CallToolResult {
    let reason = reason.into();
    CallToolResult::error_kind(
        crate::mcp::error::ToolErrorKind::InvalidArgument {
            field: field.clone(),
            reason: reason.clone(),
        },
        format!("Argument '{field}' is invalid: {reason}"),
    )
}

/// The placements, and each one's requested side (`None` keeps its side).
type ParsedPlacements = (
    Vec<konnect_ipc::types::IpcFootprintPlacement>,
    Vec<Option<String>>,
);

fn parse_component_placements(
    args: &serde_json::Value,
) -> Result<ParsedPlacements, CallToolResult> {
    let values = require_array(args, "placements")?;
    if values.is_empty() {
        return Err(invalid_placement(
            "placements".to_string(),
            "must contain at least one placement",
        ));
    }

    let mut references = HashSet::new();
    let mut placements = Vec::with_capacity(values.len());
    let mut sides = Vec::with_capacity(values.len());
    for (index, value) in values.iter().enumerate() {
        let field = |name: &str| format!("placements[{index}].{name}");
        let Some(object) = value.as_object() else {
            return Err(invalid_placement(
                format!("placements[{index}]"),
                "must be an object",
            ));
        };
        let reference = object
            .get("reference")
            .and_then(serde_json::Value::as_str)
            .filter(|reference| !reference.is_empty())
            .ok_or_else(|| invalid_placement(field("reference"), "missing or empty"))?
            .to_string();
        if !references.insert(reference.clone()) {
            return Err(invalid_placement(
                field("reference"),
                format!("duplicate footprint reference '{reference}'"),
            ));
        }
        let number = |name: &str| {
            object
                .get(name)
                .and_then(serde_json::Value::as_f64)
                .ok_or_else(|| invalid_placement(field(name), "missing or not a number"))
        };
        sides.push(match object.get("layer") {
            None | Some(serde_json::Value::Null) => None,
            Some(serde_json::Value::String(layer)) if layer == "F.Cu" || layer == "B.Cu" => {
                Some(layer.clone())
            }
            Some(_) => return Err(invalid_placement(field("layer"), "must be F.Cu or B.Cu")),
        });
        placements.push(konnect_ipc::types::IpcFootprintPlacement {
            reference,
            x: number("x")?,
            y: number("y")?,
            rotation: number("rotation")?,
        });
    }
    Ok((placements, sides))
}

async fn handle_set_component_placements(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let board = get_path(args, "board")?;
    let (placements, sides) = match parse_component_placements(args) {
        Ok(parsed) => parsed,
        Err(error) => return Ok(error),
    };
    let changes_sides = sides.iter().any(Option::is_some);

    let placements_ipc = placements.clone();
    let guard_board = board.clone();
    match attempt_ipc_write(ctx, &board, "component placement batch", move |client| {
        // A side change is a closed-board edit only: refuse before anything
        // live is touched.
        if changes_sides {
            return Ok(Err(CallToolResult::error_kind(
                crate::mcp::error::ToolErrorKind::PlanBlocked {
                    operation: "set_component_placements".to_string(),
                    reasons: vec!["side_change_requires_closed_board".to_string()],
                },
                "A placement batch that sets 'layer' edits the saved board and requires it \
                 closed in KiCad; use flip_component for an open board. Nothing was changed.",
            )));
        }
        // The whole batch or nothing: one unsupported footprint refuses all.
        let references = placements_ipc
            .iter()
            .map(|placement| placement.reference.as_str())
            .collect::<Vec<_>>();
        if let Err(refusal) = guard_live_rebuild(
            client,
            &guard_board,
            "set_component_placements",
            &references,
            ClosedBoardPath,
        )? {
            return Ok(Err(refusal.into_result()));
        }
        client.set_footprint_placements(&placements_ipc).map(Ok)
    })
    .await?
    {
        BoardWrite::Ipc(Err(refusal)) => Ok(refusal),
        BoardWrite::Ipc(Ok(applied)) => Ok(CallToolResult::json(&json!({
            "count": applied.len(),
            "placements": applied,
            "source": "ipc",
            "undo": "One KiCad undo step reverses the whole placement batch."
        }))),
        BoardWrite::Refused(result) => Ok(result),
        BoardWrite::File(_)
            if changes_sides
                && !matches!(
                    crate::tools::live_board::editor_lock(&board),
                    crate::tools::live_board::EditorLock::Absent
                ) =>
        {
            Ok(CallToolResult::error_kind(
                crate::mcp::error::ToolErrorKind::UnsafeFileFallback {
                    path: board.display().to_string(),
                    reason: "kicad_lock_present".to_string(),
                },
                "The exact board's KiCad lock is present or unreadable; no side-changing \
                 placement was written.",
            ))
        }
        BoardWrite::File(reason) => {
            match update_closed_board_footprints_on_sides(&board, &placements, &sides) {
                Ok((applied, layers)) => {
                    let mut body = json!({
                        "count": applied.len(),
                        "placements": applied,
                        "source": "file",
                        "fallback_reason": reason.evidence(),
                        "warning": reason.warning()
                    });
                    if changes_sides {
                        body["sides"] = json!(layers);
                        body["side_change"] = json!(
                            "Each footprint given a layer was mirrored by the closed-board flip \
                             first, then set to the requested position and absolute rotation."
                        );
                    }
                    Ok(CallToolResult::json(&body))
                }
                Err(error) => Ok(error.into_result()),
            }
        }
    }
}

/// Minimum KiCad version whose native `FlipItems` command Konnect drives over
/// IPC (issue #604). Older reachable KiCad answers `AS_UNHANDLED`, which this
/// handler treats as authoritative capability evidence, never inferred from a
/// version string on its own.
const FLIP_ITEMS_MIN_KICAD_VERSION: &str = "10.0.6";

async fn handle_flip_component(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let board = get_path(args, "board")?;
    let reference = match require_str(args, "reference") {
        Ok(value) => value.to_string(),
        Err(error) => return Ok(error),
    };
    let layer = match require_str(args, "layer") {
        Ok(value) if matches!(value, "F.Cu" | "B.Cu") => value.to_string(),
        Ok(value) => {
            return Ok(CallToolResult::error_kind(
                crate::mcp::error::ToolErrorKind::InvalidArgument {
                    field: "layer".to_string(),
                    reason: format!("must be F.Cu or B.Cu, got '{value}'"),
                },
                format!("Footprints can only be flipped between F.Cu and B.Cu, got '{value}'"),
            ))
        }
        Err(error) => return Ok(error),
    };

    // KiCad 10.0.6 added a native FlipItems IPC command, so an open board now
    // flips through the same live-IPC-preferred-with-fallback shape as
    // set_component_placements, rather than refusing outright whenever KiCad
    // is reachable. The closed-board file fallback is unchanged (issue #604).
    let reference_ipc = reference.clone();
    let layer_ipc = layer.clone();
    match attempt_ipc_write(ctx, &board, "footprint flip", move |client| {
        client.flip_footprint(&reference_ipc, &layer_ipc)
    })
    .await?
    {
        BoardWrite::Ipc(outcome) => Ok(flip_outcome_result(outcome)),
        BoardWrite::Refused(result) => Ok(result),
        BoardWrite::File(reason) => {
            match set_closed_board_footprint_side(&board, &reference, &layer) {
                Ok(changed) => Ok(CallToolResult::json(&json!({
                    "flipped": reference,
                    "layer": layer,
                    "changed": changed,
                    "source": "file",
                    "fallback_reason": reason.evidence(),
                    "warning": reason.warning()
                }))),
                Err(error) => Ok(error.into_result()),
            }
        }
    }
}

/// Turn a [`konnect_ipc::IpcFlipOutcome`] into the tool's `CallToolResult`.
///
/// `Flipped` is the only case reported as a plain success. `Unsupported` and
/// `Uncertain` are both IPC-observed states — the board is open and
/// reachable — so neither one may fall back to the file path; each gets its
/// own structured result naming what the caller should do next.
fn flip_outcome_result(outcome: konnect_ipc::IpcFlipOutcome) -> CallToolResult {
    use konnect_ipc::IpcFlipOutcome;
    match outcome {
        IpcFlipOutcome::Flipped {
            reference,
            kiid,
            previous_layer,
            layer,
            already_on_layer,
            models,
        } => CallToolResult::json(&json!({
            "flipped": reference,
            "kiid": kiid,
            "previous_layer": previous_layer,
            "layer": layer,
            "already_on_layer": already_on_layer,
            "models": models,
            "source": "ipc",
            "undo": "One KiCad undo step reverses the flip."
        })),
        IpcFlipOutcome::Unsupported { kicad_version } => CallToolResult::error_kind(
            crate::mcp::error::ToolErrorKind::UnsupportedCapability {
                capability: "flip_component".to_string(),
                kicad_version: kicad_version.map(|v| v.full_version),
            },
            format!(
                "The running KiCad endpoint answered AS_UNHANDLED for the native footprint-flip \
                 command. Konnect requires KiCad {FLIP_ITEMS_MIN_KICAD_VERSION} or newer to flip \
                 a footprint on an open board over IPC. The board file was not modified."
            ),
        ),
        IpcFlipOutcome::Uncertain {
            reference,
            kiid,
            requested_layer,
            reason,
        } => CallToolResult::error_kind(
            crate::mcp::error::ToolErrorKind::MutationOutcomeUncertain {
                operation: "flip_component".to_string(),
                path: format!("{reference} ({kiid})"),
                reason: reason.clone(),
            },
            format!(
                "KiCad's footprint-flip mutation for '{reference}' toward {requested_layer} may \
                 or may not have taken effect: {reason}. Inspect the board in KiCad and reconcile \
                 before retrying — this may not be a no-op."
            ),
        ),
    }
}

async fn handle_delete_component(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let reference = match require_str(args, "reference") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };

    let ref_ipc = reference.clone();
    ipc!(ctx, args, |c| c.delete_footprint(&ref_ipc));
    Ok(CallToolResult::json(&json!({ "deleted": reference })))
}

async fn handle_edit_component(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let given = |key: &str| args.get(key).is_some_and(|value| !value.is_null());
    if given("reference_field") {
        if given("value") || given("attributes") {
            return Ok(CallToolResult::error(
                "Edit value, closed-board attributes and the Reference field in separate calls."
                    .to_string(),
            ));
        }
        return reference_field::handle(args, ctx).await;
    }
    if given("attributes") {
        if given("value") {
            return Ok(CallToolResult::error(
                "Edit value and closed-board attributes in separate calls.".to_string(),
            ));
        }
        return attributes::handle(args, ctx).await;
    }
    let reference = match require_str(args, "reference") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    if let Some(value) = args["value"].as_str() {
        let reference_for_ipc = reference.clone();
        let value_for_ipc = value.to_string();
        let guard_board = get_path(args, "board")?;
        let guarded = ipc!(ctx, args, |c| {
            if let Err(refusal) = guard_live_rebuild(
                c,
                &guard_board,
                "edit_component",
                &[&reference_for_ipc],
                NoLosslessPath,
            )? {
                return Ok(Err(refusal));
            }
            c.set_footprint_value(&reference_for_ipc, &value_for_ipc)
                .map(Ok)
        });
        if let Err(refusal) = guarded {
            return Ok(refusal.into_result());
        }
    }
    let lookup_reference = reference.clone();
    let fp = ipc!(ctx, args, |c| {
        c.get_footprint(&lookup_reference)?
            .ok_or_else(|| anyhow::anyhow!("Footprint '{}' not found", lookup_reference))
    });
    Ok(CallToolResult::json(&json!({
        "reference": fp.reference,
        "value": fp.value,
        "footprint": fp.footprint
    })))
}

fn footprint_instance_reference(
    footprint: &konnect_ipc::gen::kiapi::board::types::FootprintInstance,
) -> String {
    footprint
        .reference_field
        .as_ref()
        .and_then(|field| field.text.as_ref())
        .and_then(|text| text.text.as_ref())
        .map(|text| text.text.clone())
        .unwrap_or_default()
}

fn issue_244_phantom_pad(pad: &konnect_ipc::gen::kiapi::board::types::Pad) -> bool {
    use konnect_ipc::gen::kiapi;

    let layerless = pad
        .pad_stack
        .as_ref()
        .is_none_or(|stack| stack.layers.is_empty());
    // Once an affected board is saved, KiCad can materialise proto3's empty
    // pad defaults as a PTH pad on *.Cu with a 1 nm drill. That is still #244,
    // not a real hole. The plan additionally requires a one-for-one missing
    // library graphic and an exact match of every legitimate library pad, so
    // a genuine unnumbered mechanical pad cannot be removed by this test.
    let normalised_zero_drill = pad.r#type == kiapi::board::types::PadType::PtPth as i32
        && pad
            .pad_stack
            .as_ref()
            .and_then(|stack| stack.drill.as_ref())
            .and_then(|drill| drill.diameter.as_ref())
            .is_some_and(|diameter| {
                diameter.x_nm.unsigned_abs() <= 1_000 && diameter.y_nm.unsigned_abs() <= 1_000
            });
    pad.number.is_empty()
        // KiCad may normalise an absent proto net into an empty Net message
        // when the board is saved and reloaded. Neither form names copper.
        && pad.net.as_ref().is_none_or(|net| net.name.is_empty())
        && (layerless || normalised_zero_drill)
}

fn issue_244_counts(
    footprint: &konnect_ipc::gen::kiapi::board::types::FootprintInstance,
) -> anyhow::Result<(usize, usize, BTreeMap<String, usize>)> {
    use konnect_ipc::gen::kiapi;

    let definition = footprint
        .definition
        .as_ref()
        .context("board footprint has no library definition")?;
    let mut phantom_pads = 0usize;
    let mut graphic_shapes = 0usize;
    let mut legitimate_pad_numbers = BTreeMap::new();
    for child in &definition.items {
        if konnect_ipc::builders::any_is(child, "kiapi.board.types.Pad") {
            let pad = kiapi::board::types::Pad::decode(child.value.as_slice())
                .context("footprint declares a pad that cannot be decoded")?;
            if issue_244_phantom_pad(&pad) {
                phantom_pads += 1;
            } else {
                *legitimate_pad_numbers.entry(pad.number).or_insert(0) += 1;
            }
        } else if konnect_ipc::builders::any_is(child, "kiapi.board.types.BoardGraphicShape") {
            graphic_shapes += 1;
        }
    }
    Ok((phantom_pads, graphic_shapes, legitimate_pad_numbers))
}

/// Replace only a footprint definition's mixed child list with a clean copy
/// built from its library. Everything owned by the placed instance — KIID,
/// placement, symbol path, flags and fields — stays on the original message.
/// Named/legitimate pads keep their live KIID, net and lock state by number;
/// only #244's anonymous, layerless pads disappear.
fn merge_clean_footprint_children(
    current: &prost_types::Any,
    clean: &prost_types::Any,
) -> anyhow::Result<prost_types::Any> {
    use konnect_ipc::gen::kiapi;

    let mut current = kiapi::board::types::FootprintInstance::decode(current.value.as_slice())
        .context("KiCad returned an invalid current footprint")?;
    let mut clean = kiapi::board::types::FootprintInstance::decode(clean.value.as_slice())
        .context("the library produced an invalid clean footprint")?;
    let current_definition = current
        .definition
        .as_mut()
        .context("current footprint has no library definition")?;
    let clean_definition = clean
        .definition
        .as_mut()
        .context("clean footprint has no library definition")?;

    // #244 changed only BoardGraphicShape children into Pad children. BoardText
    // and any future non-shape child types survived and may contain deliberate
    // per-board customisation, so retain those exact live messages rather than
    // replacing the whole mixed child list from the library.
    let preserved_non_shapes = current_definition
        .items
        .iter()
        .filter(|child| {
            !konnect_ipc::builders::any_is(child, "kiapi.board.types.Pad")
                && !konnect_ipc::builders::any_is(child, "kiapi.board.types.BoardGraphicShape")
        })
        .cloned()
        .collect::<Vec<_>>();

    let mut current_pads: BTreeMap<String, VecDeque<kiapi::board::types::Pad>> = BTreeMap::new();
    for child in &current_definition.items {
        if !konnect_ipc::builders::any_is(child, "kiapi.board.types.Pad") {
            continue;
        }
        let pad = kiapi::board::types::Pad::decode(child.value.as_slice())
            .context("current footprint declares a pad that cannot be decoded")?;
        if !issue_244_phantom_pad(&pad) {
            current_pads
                .entry(pad.number.clone())
                .or_default()
                .push_back(pad);
        }
    }

    let mut clean_items = Vec::new();
    for mut child in std::mem::take(&mut clean_definition.items) {
        if !konnect_ipc::builders::any_is(&child, "kiapi.board.types.Pad") {
            if konnect_ipc::builders::any_is(&child, "kiapi.board.types.BoardGraphicShape") {
                clean_items.push(child);
            }
            continue;
        }
        let mut pad = kiapi::board::types::Pad::decode(child.value.as_slice())
            .context("clean footprint declares a pad that cannot be decoded")?;
        let preserved = current_pads
            .get_mut(&pad.number)
            .and_then(VecDeque::pop_front)
            .with_context(|| {
                format!(
                    "clean library footprint has pad '{}' that the board footprint does not",
                    pad.number
                )
            })?;
        pad.id = preserved.id;
        pad.net = preserved.net;
        pad.locked = preserved.locked;
        child = konnect_ipc::builders::pack_any(&pad, "kiapi.board.types.Pad");
        clean_items.push(child);
    }
    let leftovers = current_pads
        .iter()
        .filter(|(_, pads)| !pads.is_empty())
        .map(|(number, pads)| format!("'{number}' x{}", pads.len()))
        .collect::<Vec<_>>();
    if !leftovers.is_empty() {
        anyhow::bail!(
            "board footprint has legitimate pads absent from the library: {}",
            leftovers.join(", ")
        );
    }

    clean_items.extend(preserved_non_shapes);
    current_definition.items = clean_items;
    Ok(konnect_ipc::builders::pack_any(
        &current,
        "kiapi.board.types.FootprintInstance",
    ))
}

fn merge_corrupted_footprint_candidate(
    reference: &str,
    current: &prost_types::Any,
    clean: &prost_types::Any,
    diagnostics: &mut Vec<serde_json::Value>,
) -> Option<prost_types::Any> {
    match merge_clean_footprint_children(current, clean) {
        Ok(repaired) => Some(repaired),
        Err(error) => {
            diagnostics.push(json!({
                "reference": reference,
                "code": "repair_merge_failed",
                "message": format!("{error:#}")
            }));
            None
        }
    }
}

struct CorruptedFootprintRepair {
    reference: String,
    footprint: String,
    phantom_pads: usize,
    restored_graphics: usize,
    expected_graphics: usize,
    repaired_item: prost_types::Any,
}

fn repair_plan_revision(
    board: &Path,
    repairs: &[CorruptedFootprintRepair],
    source_digests: &[(String, Vec<u8>)],
) -> String {
    let mut hasher = Sha256::new();
    hasher.update(board.as_os_str().as_encoded_bytes());
    for repair in repairs {
        hasher.update(repair.reference.as_bytes());
        hasher.update(repair.footprint.as_bytes());
        hasher.update(&repair.repaired_item.value);
    }
    for (reference, source) in source_digests {
        hasher.update(reference.as_bytes());
        hasher.update(source);
    }
    format!("{:x}", hasher.finalize())
}

async fn handle_repair_corrupted_footprints(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    use konnect_ipc::gen::kiapi;

    let board = get_path(args, "board")?;
    let dry_run = args["dry_run"].as_bool().unwrap_or(true);
    let expected_revision = args["expected_plan_revision"].as_str().map(str::to_string);
    if !dry_run && expected_revision.is_none() {
        return Ok(CallToolResult::error_kind(
            crate::mcp::error::ToolErrorKind::InvalidArgument {
                field: "expected_plan_revision".to_string(),
                reason: "required when dry_run is false".to_string(),
            },
            "Apply requires the plan revision returned by a current dry run.",
        ));
    }

    let selected = if args
        .get("references")
        .is_none_or(serde_json::Value::is_null)
    {
        None
    } else {
        let values = match require_array(args, "references") {
            Ok(values) => values,
            Err(error) => return Ok(error),
        };
        let mut selected = HashSet::new();
        for (index, value) in values.iter().enumerate() {
            let Some(reference) = value.as_str().filter(|reference| !reference.is_empty()) else {
                return Ok(CallToolResult::error_kind(
                    crate::mcp::error::ToolErrorKind::InvalidArgument {
                        field: "references".to_string(),
                        reason: format!("item {index} must be a non-empty string"),
                    },
                    format!("Argument 'references' item {index} must be a non-empty string"),
                ));
            };
            if !selected.insert(reference.to_string()) {
                return Ok(CallToolResult::error_kind(
                    crate::mcp::error::ToolErrorKind::InvalidArgument {
                        field: "references".to_string(),
                        reason: format!("duplicate reference '{reference}'"),
                    },
                    format!("Argument 'references' contains duplicate '{reference}'"),
                ));
            }
        }
        Some(selected)
    };

    let board_for_ipc = board.clone();
    let expected_for_ipc = expected_revision.clone();
    let outcome = attempt_ipc_write(
        ctx,
        &board,
        "legacy footprint repair",
        move |client| {
            let document = client.find_open_board(&board_for_ipc)?;
            let summaries = client
                .list_footprints_in(document.clone())?
                .into_iter()
                .map(|footprint| (footprint.reference.clone(), footprint))
                .collect::<HashMap<_, _>>();
            let items = client.get_items_in(
                document.clone(),
                kiapi::common::types::KiCadObjectType::KotPcbFootprint,
            )?;
            let mut seen = HashSet::new();
            let mut repairs = Vec::new();
            let mut source_digests = Vec::new();
            let mut diagnostics = Vec::new();

            for item in items {
                let footprint = kiapi::board::types::FootprintInstance::decode(
                    item.value.as_slice(),
                )
                .context("KiCad returned an invalid footprint instance")?;
                let reference = footprint_instance_reference(&footprint);
                if reference.is_empty()
                    || selected
                        .as_ref()
                        .is_some_and(|selected| !selected.contains(&reference))
                {
                    continue;
                }
                seen.insert(reference.clone());
                let (phantom_pads, current_graphics, board_pad_numbers) =
                    issue_244_counts(&footprint)?;
                if phantom_pads == 0 {
                    continue;
                }

                let Some(summary) = summaries.get(&reference) else {
                    diagnostics.push(json!({
                        "reference": reference,
                        "code": "live_summary_missing",
                        "message": "KiCad returned the footprint item but not its summary"
                    }));
                    continue;
                };
                let source = match resolve_footprint_source(&summary.footprint, &board_for_ipc) {
                    Ok(source) => source,
                    Err(error) => {
                        diagnostics.push(json!({
                            "reference": reference,
                            "code": "library_resolution_failed",
                            "message": format!("{error:#}")
                        }));
                        continue;
                    }
                };
                let pads = match extract_pad_definitions(&source) {
                    Ok(pads) => pads,
                    Err(error) => {
                        diagnostics.push(json!({
                            "reference": reference,
                            "code": "library_pad_parse_failed",
                            "message": format!("{error:#}")
                        }));
                        continue;
                    }
                };
                let graphics = match extract_graphic_definitions(&source) {
                    Ok(graphics) => graphics,
                    Err(error) => {
                        diagnostics.push(json!({
                            "reference": reference,
                            "code": "library_graphic_parse_failed",
                            "message": format!("{error:#}")
                        }));
                        continue;
                    }
                };
                let expected_graphics = graphics
                    .iter()
                    .filter(|graphic| {
                        !matches!(graphic, konnect_ipc::IpcGraphicDefinition::Text { .. })
                    })
                    .count();
                let restored_graphics = expected_graphics.saturating_sub(current_graphics);
                let library_pad_numbers = pads.iter().fold(BTreeMap::new(), |mut counts, pad| {
                    *counts.entry(pad.number.clone()).or_insert(0) += 1;
                    counts
                });
                if phantom_pads != restored_graphics || board_pad_numbers != library_pad_numbers {
                    diagnostics.push(json!({
                        "reference": reference,
                        "code": "signature_mismatch",
                        "message": format!(
                            "found {phantom_pads} anonymous layerless pads but the library is missing {restored_graphics} drawing shapes; legitimate board pads and library pads must also match exactly"
                        )
                    }));
                    continue;
                }

                let clean = KiCadIpcClient::build_footprint_item(
                    &summary.footprint,
                    &summary.reference,
                    &summary.value,
                    &pads,
                    &graphics,
                    &extract_field_placement(&source),
                    summary.position.x,
                    summary.position.y,
                    summary.rotation,
                    &summary.layer,
                )?;
                let Some(repaired_item) = merge_corrupted_footprint_candidate(
                    &reference,
                    &item,
                    &clean,
                    &mut diagnostics,
                ) else {
                    continue;
                };
                source_digests.push((reference.clone(), source.into_bytes()));
                repairs.push(CorruptedFootprintRepair {
                    reference,
                    footprint: summary.footprint.clone(),
                    phantom_pads,
                    restored_graphics,
                    expected_graphics,
                    repaired_item,
                });
            }

            if let Some(selected) = &selected {
                for reference in selected.difference(&seen) {
                    diagnostics.push(json!({
                        "reference": reference,
                        "code": "reference_not_found",
                        "message": "the requested footprint reference is not present on the open board"
                    }));
                }
            }
            repairs.sort_by(|left, right| left.reference.cmp(&right.reference));
            source_digests.sort_by(|left, right| left.0.cmp(&right.0));
            let plan_revision = repair_plan_revision(&board_for_ipc, &repairs, &source_digests);
            let candidates = repairs
                .iter()
                .map(|repair| json!({
                    "reference": repair.reference,
                    "footprint": repair.footprint,
                    "phantom_pads_removed": repair.phantom_pads,
                    "drawing_shapes_restored": repair.restored_graphics,
                    "expected_drawing_shapes": repair.expected_graphics
                }))
                .collect::<Vec<_>>();

            if !diagnostics.is_empty() {
                return Ok(json!({
                    "status": "conflict",
                    "plan_revision": plan_revision,
                    "candidate_count": repairs.len(),
                    "candidates": candidates,
                    "diagnostics": diagnostics,
                    "applied": false
                }));
            }
            if repairs.is_empty() {
                return Ok(json!({
                    "status": "noop",
                    "plan_revision": plan_revision,
                    "candidate_count": 0,
                    "candidates": [],
                    "diagnostics": [],
                    "applied": false
                }));
            }
            if dry_run {
                return Ok(json!({
                    "status": "ready",
                    "plan_revision": plan_revision,
                    "candidate_count": repairs.len(),
                    "candidates": candidates,
                    "diagnostics": [],
                    "applied": false
                }));
            }
            if expected_for_ipc.as_deref() != Some(plan_revision.as_str()) {
                return Ok(json!({
                    "status": "conflict",
                    "plan_revision": plan_revision,
                    "candidate_count": repairs.len(),
                    "candidates": candidates,
                    "diagnostics": [{
                        "code": "stale_plan_revision",
                        "message": "The live board or footprint library changed; rerun dry run and apply its new plan revision."
                    }],
                    "applied": false
                }));
            }

            let repaired_items = repairs
                .iter()
                .map(|repair| repair.repaired_item.clone())
                .collect::<Vec<_>>();
            client.run_commit("Repair legacy-corrupted footprint graphics", |client| {
                client.update_items_in(document.clone(), repaired_items)
            })?;

            let updated = client.get_items_in(
                document,
                kiapi::common::types::KiCadObjectType::KotPcbFootprint,
            )?;
            let expected = repairs
                .iter()
                .map(|repair| (repair.reference.as_str(), repair.expected_graphics))
                .collect::<HashMap<_, _>>();
            let mut verified = HashSet::new();
            for item in updated {
                let footprint = kiapi::board::types::FootprintInstance::decode(
                    item.value.as_slice(),
                )
                .context("KiCad returned an invalid repaired footprint")?;
                let reference = footprint_instance_reference(&footprint);
                let Some(expected_graphics) = expected.get(reference.as_str()) else {
                    continue;
                };
                let (phantom_pads, graphic_shapes, _) = issue_244_counts(&footprint)?;
                if phantom_pads != 0 || graphic_shapes != *expected_graphics {
                    anyhow::bail!(
                        "KiCad accepted the repair for {reference} but read-back found {phantom_pads} phantom pads and {graphic_shapes}/{expected_graphics} drawing shapes; use Ctrl-Z and inspect the footprint"
                    );
                }
                verified.insert(reference);
            }
            if verified.len() != repairs.len() {
                anyhow::bail!(
                    "KiCad accepted {} repairs but only {} repaired footprints were found on read-back; use Ctrl-Z and inspect the board",
                    repairs.len(),
                    verified.len()
                );
            }

            Ok(json!({
                "status": "applied",
                "plan_revision": plan_revision,
                "candidate_count": repairs.len(),
                "repaired_count": repairs.len(),
                "candidates": candidates,
                "diagnostics": [],
                "applied": true,
                "undo": "Ctrl-Z reverses the complete repair."
            }))
        },
    )
    .await?;

    Ok(match outcome {
        BoardWrite::Ipc(value) => CallToolResult::json(&value),
        BoardWrite::Refused(result) => result,
        BoardWrite::File(reason) => CallToolResult::error(format!(
            "{} repair_corrupted_footprints is live-IPC-only and never edits the board file \
             directly. Open the requested board in KiCad and retry.",
            reason.premise()
        )),
    })
}

async fn handle_find_component(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let reference = match require_str(args, "reference") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let fp = ipc!(ctx, args, |c| {
        c.get_footprint(&reference)?
            .ok_or_else(|| anyhow::anyhow!("Footprint '{}' not found", reference))
    });
    Ok(CallToolResult::json(&json!({
        "reference": fp.reference,
        "value": fp.value,
        "footprint": fp.footprint,
        "x": fp.position.x, "y": fp.position.y,
        "rotation": fp.rotation, "layer": fp.layer
    })))
}

/// How many pads the saved file gives `reference`, or `None` when the file
/// has no footprint by that name.
fn saved_pad_count(board_path: &std::path::Path, reference: &str) -> Option<usize> {
    let content = std::fs::read_to_string(board_path).ok()?;
    let tree = konnect_sexp::parser::parse_sexp(&content).ok()?;
    tree.find_all("footprint")
        .into_iter()
        .find(|fp| {
            fp.find_all("property").iter().any(|p| {
                p.get(1).and_then(|n| n.as_str()) == Some("Reference")
                    && p.get(2).and_then(|n| n.as_str()) == Some(reference)
            })
        })
        .map(|fp| fp.find_all("pad").len())
}

fn file_pad_drill(pad: &konnect_sexp::SexpNode) -> Option<konnect_ipc::IpcPadDrill> {
    let drill = pad.find("drill")?;
    let oval = drill.get(1).and_then(konnect_sexp::SexpNode::as_str) == Some("oval");
    let first = if oval { 2 } else { 1 };
    let x = drill.get_f64(first)?;
    let y = drill.get_f64(first + 1).unwrap_or(x);
    Some(konnect_ipc::IpcPadDrill {
        shape: Some(if oval { "oval" } else { "circle" }.to_string()),
        size: Some(konnect_ipc::IpcVector2 { x, y }),
        // The serialized pad drill does not independently name a layer span.
        // Do not turn the pad's wildcard/copper layer list into measured drill
        // endpoints; live IPC provides those when KiCad exposes them.
        start_layer: None,
        end_layer: None,
    })
}

/// Parse the geometry KiCad saved for one pad and express it in the same typed
/// shape returned by the live IPC path.
///
/// `x`, `y`, and `rotation_deg` are already in the caller's declared coordinate
/// space: board-space for `get_component_pads`, footprint-local for library
/// inspection.  Keeping this serializer shared prevents the two file readers
/// from drifting as new native pad fields are exposed.
pub(crate) fn pad_info_from_sexp(
    pad: &konnect_sexp::SexpNode,
    x: f64,
    y: f64,
    rotation_deg: f64,
    net: String,
    format_version: Option<i64>,
) -> anyhow::Result<konnect_ipc::IpcPad> {
    let number = pad
        .get(1)
        .and_then(konnect_sexp::SexpNode::as_str)
        .context("pad has no readable number")?
        .to_string();
    let pad_type = pad
        .get(2)
        .and_then(konnect_sexp::SexpNode::as_str)
        .and_then(|kind| {
            match kind {
                "thru_hole" => Some("thru_hole"),
                "smd" => Some("smd"),
                "connect" => Some("edge_connector"),
                "np_thru_hole" => Some("np_thru_hole"),
                _ => None,
            }
            .map(str::to_string)
        });
    let shape = pad
        .get(3)
        .and_then(konnect_sexp::SexpNode::as_str)
        .and_then(|shape| {
            matches!(
                shape,
                "circle"
                    | "rect"
                    | "oval"
                    | "trapezoid"
                    | "roundrect"
                    | "chamfered_rect"
                    | "custom"
            )
            .then(|| shape.to_string())
        });
    let size = pad.find("size").and_then(|size| {
        Some(konnect_ipc::IpcVector2 {
            x: size.get_f64(1)?,
            y: size.get_f64(2)?,
        })
    });
    let layers: Vec<String> = pad
        .find("layers")
        .and_then(konnect_sexp::SexpNode::children)
        .unwrap_or_default()
        .iter()
        .skip(1)
        .filter_map(konnect_sexp::SexpNode::as_str)
        .map(str::to_string)
        .collect();
    let offset = pad.find("offset").and_then(|offset| {
        Some(konnect_ipc::IpcVector2 {
            x: offset.get_f64(1)?,
            y: offset.get_f64(2)?,
        })
    });
    let corner_rounding_ratio = pad
        .find("roundrect_rratio")
        .and_then(|node| node.get_f64(1));
    let chamfer_ratio = pad.find("chamfer_ratio").and_then(|node| node.get_f64(1));
    // The same reader placement uses, so the file and the live board describe
    // a custom pad alike; what it cannot read is said, not left empty.
    let custom = (shape.as_deref() == Some("custom")).then(|| parse_custom_pad(pad));
    let (custom_anchor_shape, custom_primitives, custom_primitives_unreadable) = match custom {
        Some(Ok(custom)) => (
            Some(custom.anchor.token().to_string()),
            Some(custom.primitives),
            None,
        ),
        Some(Err(error)) => (None, None, Some(format!("{error:#}"))),
        None => (None, None, None),
    };
    let copper_layers = layers
        .iter()
        .filter(|layer| layer.as_str() == "*.Cu" || layer.ends_with(".Cu"))
        .map(|layer| konnect_ipc::IpcPadLayerGeometry {
            layer: layer.clone(),
            shape: shape.clone(),
            size: size.clone(),
            offset: offset.clone(),
            corner_rounding_ratio,
            chamfer_ratio,
            custom_anchor_shape: custom_anchor_shape.clone(),
            custom_primitives: custom_primitives.clone(),
            custom_primitives_unreadable: custom_primitives_unreadable.clone(),
        })
        .collect();
    let uuid = pad
        .find_str("uuid")
        .filter(|uuid| !uuid.is_empty())
        .map(str::to_string);
    // The reader placement uses; a file pad without the clause inherits.
    let zone_connection = pad_zone_connection(pad)
        .with_context(|| format!("pad '{number}' has an unreadable zone connection"))?
        .unwrap_or(konnect_ipc::IpcPadZoneConnection::Inherited);
    // As KiCad's parser leaves the pad, so it compares with the live readback.
    let (thermal, thermal_spoke_angle_explicit) = pad_thermal(pad, format_version)
        .with_context(|| format!("pad '{number}' has unreadable thermal settings"))?;

    Ok(konnect_ipc::IpcPad {
        uuid,
        number,
        x,
        y,
        net,
        layers,
        pad_type,
        rotation_deg: Some(rotation_deg),
        shape,
        size,
        drill: file_pad_drill(pad),
        copper_layers,
        corner_rounding_ratio,
        zone_connection: Some(zone_connection),
        thermal: Some(thermal),
        thermal_spoke_angle_explicit: Some(thermal_spoke_angle_explicit),
    })
}

/// An optional array of exact strings, for the pad filters.
fn exact_strings(
    args: &serde_json::Value,
    key: &str,
) -> Result<Option<Vec<String>>, CallToolResult> {
    match args.get(key) {
        None | Some(serde_json::Value::Null) => Ok(None),
        Some(serde_json::Value::Array(items)) => items
            .iter()
            .map(|item| item.as_str().map(str::to_string))
            .collect::<Option<Vec<_>>>()
            .map(Some)
            .ok_or_else(|| crate::tools::invalid_arg(key, "must be an array of strings")),
        Some(_) => Err(crate::tools::invalid_arg(
            key,
            "must be an array of strings",
        )),
    }
}

/// The pad keys `detail: "connectivity"` keeps.
const CONNECTIVITY_PAD_KEYS: [&str; 8] = [
    "number",
    "net",
    "x",
    "y",
    "rotation_deg",
    "pad_type",
    "uuid",
    "fabrication_property",
];

async fn handle_get_component_pads(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let detail = match args["detail"].as_str().unwrap_or("full") {
        detail @ ("full" | "fabrication" | "connectivity") => detail,
        other => {
            return Ok(CallToolResult::error_kind(
                crate::mcp::error::ToolErrorKind::InvalidArgument {
                    field: "detail".to_string(),
                    reason: format!(
                        "expected 'full', 'fabrication' or 'connectivity', got '{other}'"
                    ),
                },
                "detail must be 'full', 'fabrication' or 'connectivity'.",
            ))
        }
    };
    let pad_numbers = match exact_strings(args, "pad_numbers") {
        Ok(filter) => filter,
        Err(refusal) => return Ok(refusal),
    };
    let nets = match exact_strings(args, "nets") {
        Ok(filter) => filter,
        Err(refusal) => return Ok(refusal),
    };
    let connectivity = detail == "connectivity";
    let mut result = read_component_pads(args, ctx, connectivity).await?;
    if result.is_error || (detail == "full" && pad_numbers.is_none() && nets.is_none()) {
        return Ok(result);
    }
    for content in &mut result.content {
        let crate::mcp::protocol::ToolContent::Text { text } = content else {
            continue;
        };
        let Ok(mut value) = serde_json::from_str::<serde_json::Value>(text) else {
            continue;
        };
        let Some(pads) = value
            .get_mut("pads")
            .and_then(serde_json::Value::as_array_mut)
        else {
            continue;
        };
        let total = pads.len();
        // The filters read the reported values, so they select exactly what
        // the unfiltered answer would have said.
        pads.retain(|pad| {
            pad_numbers.as_ref().is_none_or(|numbers| {
                pad["number"]
                    .as_str()
                    .is_some_and(|n| numbers.iter().any(|w| w == n))
            }) && nets.as_ref().is_none_or(|nets| {
                pad["net"]
                    .as_str()
                    .is_some_and(|n| nets.iter().any(|w| w == n))
            })
        });
        let unmatched = |wanted: &Option<Vec<String>>, key: &str, pads: &[serde_json::Value]| {
            wanted.as_ref().map(|wanted| {
                wanted
                    .iter()
                    .filter(|w| !pads.iter().any(|p| p[key].as_str() == Some(w.as_str())))
                    .cloned()
                    .collect::<Vec<_>>()
            })
        };
        let unmatched_numbers = unmatched(&pad_numbers, "number", pads);
        let unmatched_nets = unmatched(&nets, "net", pads);
        for pad in pads.iter_mut() {
            if let Some(fields) = pad.as_object_mut() {
                match detail {
                    "fabrication" => {
                        fields.retain(|name, _| name == "number" || name == "fabrication_property")
                    }
                    "connectivity" => {
                        fields.retain(|name, _| CONNECTIVITY_PAD_KEYS.contains(&name.as_str()))
                    }
                    _ => {}
                }
            }
        }
        let matched = pads.len();
        if detail != "full" {
            value["detail"] = json!(detail);
            value["geometry_omitted"] = json!(true);
        }
        if pad_numbers.is_some() || nets.is_some() {
            value["total_pad_count"] = json!(total);
            value["matched_pad_count"] = json!(matched);
            value["pad_count"] = json!(matched);
            value["filters"] = json!({ "pad_numbers": pad_numbers, "nets": nets });
            value["unmatched_pad_numbers"] = json!(unmatched_numbers);
            value["unmatched_nets"] = json!(unmatched_nets);
        }
        *text = value.to_string();
    }
    Ok(result)
}

/// `context` adds the footprint's own pose, UUID, layer and library ID, and
/// refuses a reference more than one saved footprint carries.
async fn read_component_pads(
    args: &serde_json::Value,
    ctx: &ToolContext,
    context: bool,
) -> anyhow::Result<CallToolResult> {
    let board_path = get_path(args, "board")?;
    let reference = match require_str(args, "reference") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let saved_only = match args["source"].as_str().unwrap_or("auto") {
        "auto" => false,
        "saved" => true,
        other => {
            return Ok(CallToolResult::error_kind(
                crate::mcp::error::ToolErrorKind::InvalidArgument {
                    field: "source".to_string(),
                    reason: format!("must be 'auto' or 'saved', got '{other}'"),
                },
                format!("Unknown source '{other}'. Expected 'auto' or 'saved'."),
            ))
        }
    };
    // What a live typed read can say about pad fabrication properties:
    // nothing, which is not the same as "none".
    let live_fabrication = json!({
        "observable": false,
        "status": crate::tools::footprint_fabrication::LIVE_TYPED_UNOBSERVABLE,
        "note": "KiCad 10's Pad protobuf has no fabrication-property field. Save the board and \
                 call again with source 'saved' to read them."
    });

    // The board open in KiCad first: a part placed but not yet saved has no
    // pads in the file at all, so reading the file would either error or
    // answer about a stale board while the writers in this toolset act on the
    // live one. The file stays the fallback for an offline session.
    let ipc_reference = reference.clone();
    let live = if saved_only {
        // Asked for the saved file: no live read, whatever KiCad holds.
        Err(konnect_ipc::IpcFailure::Unreachable(String::new()))
    } else {
        with_board_ipc_classified(ctx, &board_path, move |c, document| {
            let pads = c.get_footprint_pads_in(document.clone(), &ipc_reference)?;
            let poses = if context && pads.is_some() {
                c.list_footprints_in(document)?
                    .into_iter()
                    .filter(|fp| fp.reference == ipc_reference)
                    .collect()
            } else {
                Vec::new()
            };
            Ok(pads.map(|pads| (pads, poses)))
        })
        .await?
    };
    match live {
        // KiCad has the part and reports pads for it.
        Ok(Some((pads, poses))) if !pads.is_empty() => {
            let items: Vec<serde_json::Value> = pads
                .iter()
                .map(|pad| {
                    let mut value = serde_json::to_value(pad).expect("IpcPad is serializable");
                    // Unknown, never "none": the live API cannot carry it.
                    value["fabrication_property"] = serde_json::Value::Null;
                    value
                })
                .collect();
            let mut body = json!({
                "reference": reference,
                "pad_count": items.len(),
                "pads": items,
                "source": "ipc",
                "pad_fabrication_properties": live_fabrication
            });
            if context {
                body["footprint"] = live_footprint_context(&poses);
            }
            return Ok(CallToolResult::json(&body));
        }
        // KiCad has the part and reports no pads. A pad-less footprint is
        // legal — a logo, a mounting graphic — so this is not wrong by
        // itself. But "no pads" is also what an unread response shape would
        // look like, and it reads as a plausible answer rather than a
        // failure, so it is refused whenever the saved file disagrees.
        //
        // That leaves one deliberate false positive: delete a footprint's pads
        // in KiCad without saving and the file still has them, so a correct
        // "no pads" is refused until the next save. Preferring that to
        // silently reporting an unread response as an empty pad list is the
        // whole point — and the message says which key to press.
        Ok(Some((_, poses))) => {
            if saved_pad_count(&board_path, &reference).is_some_and(|count| count > 0) {
                return Ok(CallToolResult::error(format!(
                    "KiCad reports no pads for footprint '{reference}' while the saved file \
                     has some. Refusing to answer 'no pads' — save the board in KiCad and \
                     retry, and report this if it persists."
                )));
            }
            let mut body = json!({
                "reference": reference,
                "pad_count": 0,
                "pads": [],
                "source": "ipc",
                "pad_fabrication_properties": live_fabrication
            });
            if context {
                body["footprint"] = live_footprint_context(&poses);
            }
            return Ok(CallToolResult::json(&body));
        }
        // KiCad holds this board and does not have the part: the file may
        // still carry a footprint the user has deleted, so answering from it
        // would be answering about a board that no longer exists.
        Ok(None) => {
            return Ok(CallToolResult::error(format!(
                "Footprint '{reference}' not found on the board open in KiCad"
            )))
        }
        // Unreachable, or reachable and holding some other board: either way
        // no live KiCad can be answering about this one, so read the file.
        Err(konnect_ipc::IpcFailure::Unreachable(_)) => {}
        Err(konnect_ipc::IpcFailure::Target { error, .. }) if error.proves_not_open() => {}
        Err(konnect_ipc::IpcFailure::Target { error, .. }) => {
            return Ok(crate::tools::ipc_target_error_result(&error));
        }
        Err(konnect_ipc::IpcFailure::Rejected(message)) => {
            return Ok(CallToolResult::error(message));
        }
        Err(konnect_ipc::IpcFailure::Recovered(message)) => {
            return Ok(crate::tools::ipc_recovered_result(&message));
        }
        Err(konnect_ipc::IpcFailure::Uncertain(message)) => {
            return Ok(crate::tools::ipc_uncertain_result(&message));
        }
    }

    let content = std::fs::read_to_string(&board_path)?;
    let tree = konnect_sexp::parser::parse_sexp(&content)?;

    // Find the footprint with matching reference, as a property or the
    // older fp_text.
    let matching: Vec<&konnect_sexp::SexpNode> = tree
        .find_all("footprint")
        .into_iter()
        .filter(|fp| footprint_reference(fp).as_deref() == Some(reference.as_str()))
        .collect();
    if context && matching.len() > 1 {
        return Ok(CallToolResult::error_kind(
            crate::mcp::error::ToolErrorKind::AmbiguousTarget {
                target: reference.clone(),
                candidates: matching
                    .iter()
                    .map(|fp| {
                        crate::tools::footprint_fabrication::footprint_identity(fp)
                            .unwrap_or("<no uuid>")
                            .to_string()
                    })
                    .collect(),
            },
            format!(
                "{} saved footprints carry reference '{reference}'; their pads cannot be told \
                 apart by reference.",
                matching.len()
            ),
        ));
    }
    let fp_node = matching.first().copied();

    let fp_node = match fp_node {
        Some(n) => n,
        None => {
            return Ok(CallToolResult::error(format!(
                "Footprint '{}' not found",
                reference
            )))
        }
    };

    let fp_at = fp_node.find("at");
    let fp_x = fp_at.and_then(|a| a.get_f64(1)).unwrap_or(0.0);
    let fp_y = fp_at.and_then(|a| a.get_f64(2)).unwrap_or(0.0);
    let fp_rot = fp_at.and_then(|a| a.get_f64(3)).unwrap_or(0.0);
    // KiCad parses the board's footprints under the greater of the two.
    let format_version = sexp_format_version(&tree).max(sexp_format_version(fp_node));

    let mut pads = Vec::new();
    for pad in fp_node.find_all("pad") {
        let pad_number = pad
            .get(1)
            .and_then(konnect_sexp::SexpNode::as_str)
            .unwrap_or("<unreadable>");
        let Some(pad_at) = pad.find("at") else {
            return Ok(CallToolResult::error(format!(
                "Pad '{pad_number}' on footprint '{reference}' has no readable position"
            )));
        };
        let (Some(local_x), Some(local_y)) = (pad_at.get_f64(1), pad_at.get_f64(2)) else {
            return Ok(CallToolResult::error(format!(
                "Pad '{pad_number}' on footprint '{reference}' has an invalid position"
            )));
        };
        // Transform local pad coords to board space (rotation only).
        // Uses the canonical KiCAD transform — see konnect_sexp::geometry.
        let (board_x, board_y) =
            konnect_sexp::geometry::transform_pad(local_x, local_y, fp_x, fp_y, fp_rot);
        // Three outcomes, deliberately distinguishable. No (net …) node at
        // all is an unconnected pad, and "" says so. A node we can read
        // gives its name. A node that is present but unreadable gives
        // null — previously it gave "" too, so a fully connected KiCad 10
        // pad was indistinguishable from an unconnected one. See
        // konnect_sexp::net for the two shapes.
        let net = match pad.find("net") {
            None => json!(""),
            Some(node) => match konnect_sexp::net::net_name(node) {
                Some(name) => json!(name),
                None => serde_json::Value::Null,
            },
        };
        // In a saved board KiCad writes the pad's effective board-space
        // angle here. Unlike a library footprint, adding the footprint
        // angle again would double-rotate the reported geometry.
        let rotation_deg = pad_at.get_f64(3).unwrap_or(0.0);
        let pad_info = match pad_info_from_sexp(
            pad,
            board_x,
            board_y,
            rotation_deg,
            net.as_str().unwrap_or_default().to_string(),
            format_version,
        ) {
            Ok(info) => info,
            Err(error) => {
                return Ok(CallToolResult::error(format!(
                    "Could not read pad '{pad_number}' on footprint '{reference}': {error}"
                )))
            }
        };
        let mut value = serde_json::to_value(pad_info)?;
        // Preserve the established three-state net contract.  The shared
        // typed pad uses a string for known/unconnected values; malformed
        // saved net nodes remain an explicit JSON null here.
        value["net"] = net;
        // The saved pad's fabrication property, or "none" when the saved pad
        // has none: KiCad writes the clause only for a property other than
        // none, so its absence is the fact, not a gap in the read.
        value["fabrication_property"] = json!(
            crate::tools::footprint_fabrication::pad_fabrication_property(pad).unwrap_or("none")
        );
        pads.push(value);
    }

    let mut body = json!({
        "reference": reference,
        "pad_count": pads.len(),
        "pads": pads,
        "source": "file",
        "requested_source": if saved_only { "saved" } else { "auto" },
        "pad_fabrication_properties": {
            "observable": true,
            "status": "read_from_saved_file",
            "uuid": crate::tools::footprint_fabrication::footprint_identity(fp_node),
            "note": "The saved board as last written; unsaved edits in an open KiCad are not \
                     part of this read."
        }
    });
    if context {
        let mut pose = saved_footprint_json(fp_node);
        pose["source"] = json!("file");
        body["footprint"] = pose;
    }
    Ok(CallToolResult::json(&body))
}

/// A live footprint's pose and identity, as the compact readers report it.
fn live_footprint_context(poses: &[konnect_ipc::IpcFootprint]) -> serde_json::Value {
    match poses {
        [fp] => json!({
            "reference": fp.reference,
            "footprint": fp.footprint,
            "uuid": fp.uuid,
            "x": fp.position.x,
            "y": fp.position.y,
            "rotation": fp.rotation,
            "layer": fp.layer,
            "source": "ipc",
        }),
        _ => json!({
            "unavailable": format!(
                "KiCad's footprint list names {} footprints with this reference",
                poses.len()
            ),
            "source": "ipc",
        }),
    }
}

/// A saved footprint's pose, identity and value, as `get_component_list` and
/// the compact pad reader report it. Angles are the saved (at …) angle.
fn saved_footprint_json(fp: &konnect_sexp::SexpNode) -> serde_json::Value {
    let at = fp.find("at");
    let value = fp
        .find_all("property")
        .into_iter()
        .find(|p| p.get(1).and_then(konnect_sexp::SexpNode::as_str) == Some("Value"))
        .or_else(|| {
            fp.find_all("fp_text")
                .into_iter()
                .find(|t| t.get(1).and_then(konnect_sexp::SexpNode::as_str) == Some("value"))
        })
        .and_then(|p| p.get(2))
        .and_then(konnect_sexp::SexpNode::as_str);
    json!({
        "reference": footprint_reference(fp),
        "value": value,
        "footprint": fp.get(1).and_then(konnect_sexp::SexpNode::as_str),
        "uuid": crate::tools::footprint_fabrication::footprint_identity(fp),
        "x": at.and_then(|a| a.get_f64(1)),
        "y": at.and_then(|a| a.get_f64(2)),
        "rotation": at.map(|a| a.get_f64(3).unwrap_or(0.0)),
        "layer": fp.find_str("layer"),
    })
}

async fn handle_get_pad_position(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let pad_number = match require_str(args, "pad_number") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let pads_result = handle_get_component_pads(args, ctx).await?;
    if pads_result.is_error {
        return Ok(pads_result);
    }
    // Parse the result and filter for the specific pad number
    if let Some(crate::mcp::protocol::ToolContent::Text { text }) = pads_result.content.first() {
        if let Ok(parsed) = serde_json::from_str::<serde_json::Value>(text) {
            if let Some(pads) = parsed["pads"].as_array() {
                if let Some(pad) = pads
                    .iter()
                    .find(|p| p["number"].as_str() == Some(&pad_number))
                {
                    // Carry the pad list's source, so a caller measuring one
                    // pad can still tell whether it measured the live board.
                    let mut pad = pad.clone();
                    if let (Some(object), Some(source)) =
                        (pad.as_object_mut(), parsed.get("source"))
                    {
                        object.insert("source".to_string(), source.clone());
                    }
                    return Ok(CallToolResult::json(&pad));
                }
            }
        }
    }
    Ok(CallToolResult::error(format!(
        "Pad '{}' not found",
        pad_number
    )))
}

async fn handle_get_component_list(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let references = match exact_strings(args, "references") {
        Ok(filter) => filter,
        Err(refusal) => return Ok(refusal),
    };
    let saved = match args.get("source").and_then(serde_json::Value::as_str) {
        None | Some("live") => false,
        Some("saved") => true,
        Some(other) => {
            return Ok(crate::tools::invalid_arg(
                "source",
                &format!("must be 'live' or 'saved', got '{other}'"),
            ))
        }
    };
    let items: Vec<serde_json::Value> = if saved {
        let board = get_path(args, "board")?;
        let content = match read_consistent(&board) {
            Ok(content) => content,
            Err(error) => {
                return Ok(CallToolResult::error_kind(
                    crate::mcp::error::ToolErrorKind::FileNotFound {
                        path: board.display().to_string(),
                    },
                    format!("The saved board could not be read: {error}"),
                ))
            }
        };
        let tree = konnect_sexp::parser::parse_sexp(&content)?;
        tree.find_all("footprint")
            .into_iter()
            .map(saved_footprint_json)
            .collect()
    } else {
        let fps = ipc!(ctx, args, |c| c.list_footprints());
        fps.iter()
            .map(|fp| {
                json!({
                    "reference": fp.reference,
                    "value": fp.value,
                    "footprint": fp.footprint,
                    "x": fp.position.x, "y": fp.position.y,
                    "rotation": fp.rotation, "layer": fp.layer,
                    "uuid": fp.uuid
                })
            })
            .collect()
    };
    let mut body = match &references {
        None => json!({ "count": items.len(), "components": items }),
        Some(wanted) => {
            let kept: Vec<serde_json::Value> = items
                .into_iter()
                .filter(|item| {
                    item["reference"]
                        .as_str()
                        .is_some_and(|r| wanted.iter().any(|w| w == r))
                })
                .collect();
            let count_of = |reference: &str| {
                kept.iter()
                    .filter(|item| item["reference"].as_str() == Some(reference))
                    .count()
            };
            let unknown: Vec<&String> = wanted.iter().filter(|w| count_of(w) == 0).collect();
            let duplicate: Vec<&String> = wanted.iter().filter(|w| count_of(w) > 1).collect();
            json!({
                "count": kept.len(),
                "components": kept,
                "requested_count": wanted.len(),
                "unknown_references": unknown,
                "duplicate_references": duplicate,
            })
        }
    };
    if saved {
        let board = get_path(args, "board")?;
        body["source"] = json!("saved");
        body["source_evidence"] = json!({
            "board_state": crate::tools::board_source::FROM_SAVED_BOARD,
            "excludes_unsaved_editor_state": true,
            "reason": if ctx.board_session.was_observed_live(&board) {
                "explicitly_requested_after_live_observation"
            } else {
                "explicitly_requested"
            },
            "detail": "source 'saved' was requested; KiCad was not consulted. Angles are the \
                       saved (at …) angle.",
        });
    } else if references.is_some() {
        body["source"] = json!("ipc");
    }
    Ok(CallToolResult::json(&body))
}

async fn handle_place_array(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let board = get_path(args, "board")?;
    let footprint = match require_str(args, "footprint") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let start_x = match require_f64(args, "start_x") {
        Ok(v) => v,
        Err(e) => return Ok(e),
    };
    let start_y = match require_f64(args, "start_y") {
        Ok(v) => v,
        Err(e) => return Ok(e),
    };
    // `count_x` is schema-required and defaulted to 1, so a caller who meant a
    // 10x10 grid and mistyped the key got a single column of real footprints
    // committed to the live board, and `{"placed_count": N}` back. The zero
    // guard below could never fire on that path either — 1 is not 0 — so it
    // only ever caught an explicit zero, which it still does (#218).
    let count_x = match require_u64(args, "count_x") {
        Ok(v) => v,
        Err(e) => return Ok(e),
    };
    let count_y = args["count_y"].as_u64().unwrap_or(1);
    let Some(total_count) = count_x.checked_mul(count_y) else {
        return Ok(CallToolResult::error("Array dimensions overflow."));
    };
    if count_x == 0 || count_y == 0 || total_count > 10_000 {
        return Ok(CallToolResult::error(
            "Array dimensions must be non-zero and contain at most 10,000 components.",
        ));
    }
    let count_x = count_x as usize;
    let count_y = count_y as usize;
    let spacing_x = match require_f64(args, "spacing_x") {
        Ok(v) => v,
        Err(e) => return Ok(e),
    };
    let spacing_y = args["spacing_y"].as_f64().unwrap_or(spacing_x);
    let prefix = args["ref_prefix"].as_str().unwrap_or("U").to_string();
    let ref_start = args["ref_start"].as_u64().unwrap_or(1);
    if ref_start.checked_add(total_count - 1).is_none() {
        return Ok(CallToolResult::error("Reference number overflow."));
    }
    let source = match resolve_footprint_source(&footprint, &board) {
        Ok(source) => source,
        Err(error) => return Ok(CallToolResult::error(error.to_string())),
    };
    // Graphics are footprint-local and identical for every array instance, so
    // one extraction serves the whole batch.
    let graphics = match extract_graphic_definitions(&source) {
        Ok(graphics) => graphics,
        Err(error) => return Ok(CallToolResult::error(error.to_string())),
    };
    let fields = extract_field_placement(&source);

    let value = footprint
        .split_once(':')
        .map(|(_, entry)| entry)
        .unwrap_or(&footprint)
        .to_string();
    let mut planned = Vec::with_capacity(total_count as usize);
    for row in 0..count_y {
        for col in 0..count_x {
            let x = start_x + col as f64 * spacing_x;
            let y = start_y + row as f64 * spacing_y;
            let reference = format!("{prefix}{}", ref_start + planned.len() as u64);
            let prepared = match prepare_footprint_source(
                &source, &footprint, &reference, None, x, y, 0.0, "F.Cu",
            ) {
                Ok(prepared) => prepared,
                Err(error) => return Ok(CallToolResult::error(error.to_string())),
            };
            let pads = match extract_pad_definitions(&prepared) {
                Ok(pads) => pads,
                Err(error) => return Ok(CallToolResult::error(error.to_string())),
            };
            planned.push((reference, pads, x, y));
        }
    }

    let footprint_id = footprint.clone();
    let placed = match with_board_ipc_classified(ctx, &board, move |c, _| {
        let existing = c
            .list_footprints()?
            .into_iter()
            .map(|footprint| footprint.reference)
            .collect::<HashSet<_>>();
        let conflicts = planned
            .iter()
            .filter(|(reference, _, _, _)| existing.contains(reference))
            .map(|(reference, _, _, _)| reference.as_str())
            .collect::<Vec<_>>();
        if !conflicts.is_empty() {
            anyhow::bail!(
                "footprint references already exist on the board: {}",
                conflicts.join(", ")
            );
        }

        let items = planned
            .iter()
            .map(|(reference, pads, x, y)| {
                KiCadIpcClient::build_footprint_item(
                    &footprint_id,
                    reference,
                    &value,
                    pads,
                    &graphics,
                    &fields,
                    *x,
                    *y,
                    0.0,
                    "F.Cu",
                )
                .with_context(|| format!("failed to prepare {reference}"))
            })
            .collect::<anyhow::Result<Vec<_>>>()?;
        c.run_commit("Place footprint array", |c| c.create_items(items))?;

        let mut created = c
            .list_footprints()?
            .into_iter()
            .map(|footprint| (footprint.reference.clone(), footprint))
            .collect::<HashMap<_, _>>();
        planned
            .into_iter()
            .map(|(reference, _, _, _)| {
                let footprint = created.remove(&reference).with_context(|| {
                    format!("committed footprint '{reference}' was not found on the board")
                })?;
                Ok(json!({
                    "reference": reference,
                    "x": footprint.position.x,
                    "y": footprint.position.y
                }))
            })
            .collect::<anyhow::Result<Vec<_>>>()
    })
    .await?
    {
        Ok(placed) => placed,
        Err(error) => return Ok(CallToolResult::error(format!("IPC array error: {error}"))),
    };
    Ok(CallToolResult::json(
        &json!({ "placed_count": placed.len(), "components": placed }),
    ))
}

async fn handle_align_components(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let board = get_path(args, "board")?;
    let refs = match args["references"].as_array() {
        Some(references) if !references.is_empty() => references,
        _ => {
            return Ok(CallToolResult::error(
                "'references' must be a non-empty array.",
            ))
        }
    };
    let references = match refs
        .iter()
        .map(|reference| reference.as_str().map(String::from))
        .collect::<Option<Vec<_>>>()
    {
        Some(references) => references,
        None => return Ok(CallToolResult::error("Every reference must be a string.")),
    };
    let axis = args["axis"].as_str().unwrap_or("x").to_string();
    if axis != "x" && axis != "y" {
        return Ok(CallToolResult::error("'axis' must be either 'x' or 'y'."));
    }
    let value = match require_f64(args, "value") {
        Ok(v) => v,
        Err(e) => return Ok(e),
    };

    let guard_board = board.clone();
    let aligned = match with_board_ipc_classified(ctx, &board, move |c, _| {
        let names = references.iter().map(String::as_str).collect::<Vec<_>>();
        if let Err(refusal) =
            guard_live_rebuild(c, &guard_board, "align_components", &names, NoLosslessPath)?
        {
            return Ok(Err(refusal));
        }
        c.run_commit("Align footprints", |c| {
            references
                .iter()
                .map(|reference| {
                    let footprint = c
                        .get_footprint(reference)?
                        .with_context(|| format!("footprint '{reference}' not found"))?;
                    let (x, y) = if axis == "y" {
                        (footprint.position.x, value)
                    } else {
                        (value, footprint.position.y)
                    };
                    c.move_footprint(reference, x, y)?;
                    Ok(json!({ "reference": reference, "x": x, "y": y }))
                })
                .collect::<anyhow::Result<Vec<_>>>()
        })
        .map(Ok)
    })
    .await?
    {
        Ok(Ok(aligned)) => aligned,
        Ok(Err(refusal)) => return Ok(refusal.into_result()),
        Err(error) => return Ok(CallToolResult::error(format!("IPC align error: {error}"))),
    };
    Ok(CallToolResult::json(
        &json!({ "aligned_count": aligned.len(), "components": aligned }),
    ))
}

async fn handle_duplicate_component(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let board = get_path(args, "board")?;
    let reference = match require_str(args, "reference") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let new_reference = match require_str(args, "new_reference") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let x = match require_f64(args, "x") {
        Ok(v) => v,
        Err(e) => return Ok(e),
    };
    let y = match require_f64(args, "y") {
        Ok(v) => v,
        Err(e) => return Ok(e),
    };

    // Get the source footprint's footprint ID and rotation
    let ref_ipc = reference.clone();
    let src = ipc!(ctx, args, |c| {
        c.get_footprint(&ref_ipc)?
            .ok_or_else(|| anyhow::anyhow!("Footprint '{}' not found", ref_ipc))
    });
    if let Some(rejection) = back_side_layer_error(&src.layer) {
        return Ok(rejection);
    }
    let source = match resolve_footprint_source(&src.footprint, &board) {
        Ok(source) => source,
        Err(error) => return Ok(CallToolResult::error(error.to_string())),
    };
    let prepared = match prepare_footprint_source(
        &source,
        &src.footprint,
        &new_reference,
        Some(&src.value),
        x,
        y,
        src.rotation,
        &src.layer,
    ) {
        Ok(prepared) => prepared,
        Err(error) => return Ok(CallToolResult::error(error.to_string())),
    };
    let ipc_reference = new_reference.clone();
    let fp_id = src.footprint.clone();
    let fp_value = src.value.clone();
    let fp_layer = src.layer.clone();
    let fp_rotation = src.rotation;
    let pads = match extract_pad_definitions(&prepared) {
        Ok(pads) => pads,
        Err(error) => return Ok(CallToolResult::error(error.to_string())),
    };
    let graphics = match extract_graphic_definitions(&prepared) {
        Ok(graphics) => graphics,
        Err(error) => return Ok(CallToolResult::error(error.to_string())),
    };
    let fields = extract_field_placement(&prepared);
    let dup_board = board.clone();
    let fp = ipc!(ctx, args, |c| c.place_footprint(
        &dup_board,
        &fp_id,
        &ipc_reference,
        &fp_value,
        &pads,
        &graphics,
        &fields,
        x,
        y,
        fp_rotation,
        &fp_layer
    ));
    Ok(CallToolResult::json(&json!({
        "duplicated_from": reference,
        "new_reference": fp.reference,
        "x": fp.position.x, "y": fp.position.y
    })))
}

async fn handle_get_board_2d_view(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    use base64::Engine;
    let board_path = get_path(args, "board")?;
    let width = args["width"].as_u64().unwrap_or(800).clamp(100, 4000) as u32;
    let height = args["height"].as_u64().unwrap_or(600).clamp(100, 4000) as u32;

    let tmp = board_path.with_extension("render.png");
    super::cli::render_pcb_png(&ctx.config.kicad_cli, &board_path, &tmp, width, height).await?;
    let bytes = tokio::fs::read(&tmp).await?;
    let _ = tokio::fs::remove_file(&tmp).await;

    let b64 = base64::engine::general_purpose::STANDARD.encode(&bytes);
    Ok(CallToolResult::image(b64, "image/png"))
}

#[cfg(test)]
mod tests {
    use super::*;

    const FOOTPRINT: &str = r#"(footprint "R_0402"
  (version 20240108)
  (generator pcbnew)
  (layer "F.Cu")
  (property "Reference" "REF**" (at 0 -1 0) (layer "F.SilkS"))
  (property "Value" "R_0402" (at 0 1 0) (layer "F.Fab"))
  (pad "1" smd roundrect (at -0.5 0) (size 0.5 0.5)
    (layers "F.Cu" "F.Paste" "F.Mask")))"#;

    fn test_pad(number: &str, layers: Vec<i32>, id: &str, net: Option<&str>) -> prost_types::Any {
        use konnect_ipc::gen::kiapi;
        let pad = kiapi::board::types::Pad {
            id: Some(kiapi::common::types::Kiid {
                value: id.to_string(),
            }),
            number: number.to_string(),
            net: net.map(|name| kiapi::board::types::Net {
                code: None,
                name: name.to_string(),
            }),
            pad_stack: Some(kiapi::board::types::PadStack {
                layers,
                ..Default::default()
            }),
            ..Default::default()
        };
        konnect_ipc::builders::pack_any(&pad, "kiapi.board.types.Pad")
    }

    fn normalised_issue_244_pad() -> prost_types::Any {
        use konnect_ipc::gen::kiapi;
        let mut pad = kiapi::board::types::Pad::decode(
            test_pad("", vec![0], "normalised-phantom", None)
                .value
                .as_slice(),
        )
        .unwrap();
        pad.r#type = kiapi::board::types::PadType::PtPth as i32;
        pad.pad_stack.as_mut().unwrap().drill = Some(kiapi::board::types::DrillProperties {
            diameter: Some(konnect_ipc::builders::vec2(0.000_001, 0.000_001)),
            ..Default::default()
        });
        konnect_ipc::builders::pack_any(&pad, "kiapi.board.types.Pad")
    }

    #[test]
    fn issue_244_signature_distinguishes_real_unnumbered_pads() {
        use konnect_ipc::gen::kiapi;
        let footprint = kiapi::board::types::FootprintInstance {
            definition: Some(kiapi::board::types::Footprint {
                items: vec![
                    test_pad("1", vec![0], "named", None),
                    // A real NPTH/mechanical pad may be unnumbered, but it has
                    // an explicit layer set and must survive the repair.
                    test_pad("", vec![0, 31], "mechanical", None),
                    // #244's proto3 default pad has neither a number nor any
                    // layers. This exact signature is the only one repaired.
                    test_pad("", vec![], "phantom", None),
                    // A save/reload can normalise that same proto default into
                    // *.Cu with a one-nanometre PTH drill.
                    normalised_issue_244_pad(),
                    konnect_ipc::builders::pack_any(
                        &kiapi::board::types::BoardGraphicShape::default(),
                        "kiapi.board.types.BoardGraphicShape",
                    ),
                ],
                ..Default::default()
            }),
            ..Default::default()
        };

        let (phantoms, graphics, pads) = issue_244_counts(&footprint).unwrap();
        assert_eq!(phantoms, 2);
        assert_eq!(graphics, 1);
        assert_eq!(pads.get("1"), Some(&1));
        assert_eq!(pads.get(""), Some(&1));
    }

    #[test]
    fn repair_restores_graphics_and_preserves_live_pad_identity_and_net() {
        use konnect_ipc::gen::kiapi;

        let current = kiapi::board::types::FootprintInstance {
            id: Some(kiapi::common::types::Kiid {
                value: "placed-footprint".to_string(),
            }),
            symbol_path: Some(kiapi::common::types::SheetPath {
                path: vec![kiapi::common::types::Kiid {
                    value: "symbol-path".to_string(),
                }],
                path_human_readable: String::new(),
            }),
            definition: Some(kiapi::board::types::Footprint {
                items: vec![
                    test_pad("1", vec![0], "live-pad", Some("GND")),
                    test_pad("", vec![], "phantom", None),
                    konnect_ipc::builders::pack_any(
                        &kiapi::board::types::BoardText {
                            id: Some(kiapi::common::types::Kiid {
                                value: "custom-board-text".to_string(),
                            }),
                            ..Default::default()
                        },
                        "kiapi.board.types.BoardText",
                    ),
                ],
                ..Default::default()
            }),
            ..Default::default()
        };
        let clean = kiapi::board::types::FootprintInstance {
            definition: Some(kiapi::board::types::Footprint {
                items: vec![
                    test_pad("1", vec![0], "library-pad", None),
                    konnect_ipc::builders::pack_any(
                        &kiapi::board::types::BoardGraphicShape::default(),
                        "kiapi.board.types.BoardGraphicShape",
                    ),
                    konnect_ipc::builders::pack_any(
                        &kiapi::board::types::BoardText {
                            id: Some(kiapi::common::types::Kiid {
                                value: "library-text".to_string(),
                            }),
                            ..Default::default()
                        },
                        "kiapi.board.types.BoardText",
                    ),
                ],
                ..Default::default()
            }),
            ..Default::default()
        };
        let current =
            konnect_ipc::builders::pack_any(&current, "kiapi.board.types.FootprintInstance");
        let clean = konnect_ipc::builders::pack_any(&clean, "kiapi.board.types.FootprintInstance");

        let repaired = merge_clean_footprint_children(&current, &clean).unwrap();
        let repaired =
            kiapi::board::types::FootprintInstance::decode(repaired.value.as_slice()).unwrap();
        assert_eq!(repaired.id.as_ref().unwrap().value, "placed-footprint");
        assert_eq!(
            repaired.symbol_path.as_ref().unwrap().path[0].value,
            "symbol-path"
        );
        let (phantoms, graphics, pads) = issue_244_counts(&repaired).unwrap();
        assert_eq!((phantoms, graphics), (0, 1));
        assert_eq!(pads.get("1"), Some(&1));

        let pad = repaired
            .definition
            .as_ref()
            .unwrap()
            .items
            .iter()
            .find(|item| konnect_ipc::builders::any_is(item, "kiapi.board.types.Pad"))
            .map(|item| kiapi::board::types::Pad::decode(item.value.as_slice()).unwrap())
            .unwrap();
        assert_eq!(pad.id.as_ref().unwrap().value, "live-pad");
        assert_eq!(pad.net.as_ref().unwrap().name, "GND");
        let texts = repaired
            .definition
            .as_ref()
            .unwrap()
            .items
            .iter()
            .filter(|item| konnect_ipc::builders::any_is(item, "kiapi.board.types.BoardText"))
            .map(|item| {
                kiapi::board::types::BoardText::decode(item.value.as_slice())
                    .unwrap()
                    .id
                    .unwrap()
                    .value
            })
            .collect::<Vec<_>>();
        assert_eq!(texts, ["custom-board-text"]);
    }

    #[test]
    fn repair_merge_failure_is_scoped_to_one_footprint() {
        use konnect_ipc::gen::kiapi;

        let valid = kiapi::board::types::FootprintInstance {
            definition: Some(kiapi::board::types::Footprint::default()),
            ..Default::default()
        };
        let valid = konnect_ipc::builders::pack_any(&valid, "kiapi.board.types.FootprintInstance");
        let invalid = prost_types::Any {
            type_url: "type.googleapis.com/kiapi.board.types.FootprintInstance".to_string(),
            value: vec![0xff],
        };
        let mut diagnostics = Vec::new();
        let mut repaired = Vec::new();

        for (reference, clean) in [("R_BAD", &invalid), ("R_GOOD", &valid)] {
            if merge_corrupted_footprint_candidate(reference, &valid, clean, &mut diagnostics)
                .is_some()
            {
                repaired.push(reference);
            }
        }

        assert_eq!(repaired, ["R_GOOD"]);
        assert_eq!(diagnostics.len(), 1);
        assert_eq!(diagnostics[0]["reference"], "R_BAD");
        assert_eq!(diagnostics[0]["code"], "repair_merge_failed");
        assert!(diagnostics[0]["message"]
            .as_str()
            .unwrap()
            .contains("invalid clean footprint"));
    }

    #[test]
    fn prepares_complete_front_footprint() {
        let prepared = prepare_footprint_source(
            FOOTPRINT,
            "Resistor_SMD:R_0402",
            "R17",
            None,
            12.5,
            8.25,
            90.0,
            "F.Cu",
        )
        .unwrap();
        assert!(prepared.starts_with("(footprint \"Resistor_SMD:R_0402\""));
        assert!(prepared.contains("(property \"Reference\" \"R17\""));
        assert!(prepared.contains("(at 12.5 8.25 90)"));
        assert!(prepared.contains("(pad \"1\""));
        assert!(prepared.contains("(layers \"F.Cu\" \"F.Paste\" \"F.Mask\")"));
        let pads = extract_pad_definitions(&prepared).unwrap();
        assert_eq!(pads.len(), 1);
        assert_eq!(pads[0].number, "1");
        assert_eq!(pads[0].shape, "roundrect");
        assert_eq!(pads[0].layers, ["F.Cu", "F.Paste", "F.Mask"]);
    }

    #[test]
    fn back_side_placement_is_rejected_not_string_swapped() {
        // The old implementation did a blind "F. → "B. text swap over the whole
        // footprint, which corrupted property values starting with "F." and
        // left pad X positions unmirrored — wrong geometry presented as
        // success. Until a real mirror flip exists, B.Cu must be refused.
        let error = prepare_footprint_source(
            FOOTPRINT,
            "Resistor_SMD:R_0402",
            "R18",
            Some("10k"),
            1.0,
            2.0,
            0.0,
            "B.Cu",
        )
        .unwrap_err();
        assert!(error.to_string().contains("not yet supported"), "{error}");
    }

    #[test]
    fn rejects_non_outer_copper_layer() {
        let error = prepare_footprint_source(
            FOOTPRINT,
            "Resistor_SMD:R_0402",
            "R19",
            None,
            0.0,
            0.0,
            0.0,
            "In1.Cu",
        )
        .unwrap_err();
        assert!(error.to_string().contains("F.Cu"));
    }

    /// A footprint with the graphics KiCad's own libraries ship: courtyard
    /// rect, silkscreen lines, a fab outline and text, plus hidden built-in
    /// properties that must not be drawn.
    const GRAPHIC_FOOTPRINT: &str = r#"(footprint "R_0402"
  (version 20240108)
  (generator pcbnew)
  (layer "F.Cu")
  (property "Reference" "REF**" (at 0 -1 0) (layer "F.SilkS"))
  (property "Value" "R_0402" (at 0 1 0) (layer "F.Fab"))
  (property "Datasheet" "" (at 0 0 0) (layer "F.Fab") (hide yes))
  (fp_line (start -0.6 -0.5) (end 0.6 -0.5) (stroke (width 0.12) (type solid)) (layer "F.SilkS"))
  (fp_line (start -0.6 0.5) (end 0.6 0.5) (stroke (width 0.12) (type solid)) (layer "F.SilkS"))
  (fp_rect (start -0.8 -0.7) (end 0.8 0.7) (stroke (width 0.05) (type default)) (fill no) (layer "F.CrtYd"))
  (fp_circle (center 0 0) (end 0.25 0) (stroke (width 0.1) (type solid)) (fill yes) (layer "F.Fab"))
  (fp_arc (start -0.3 0) (mid 0 -0.3) (end 0.3 0) (stroke (width 0.12) (type solid)) (layer "F.SilkS"))
  (fp_poly (pts (xy -0.2 -0.2) (xy 0.2 -0.2) (xy 0.2 0.2)) (stroke (width 0.1) (type solid)) (fill yes) (layer "F.Fab"))
  (fp_text user "${REFERENCE}" (at 0 1.17 0) (layer "F.Fab") (effects (font (size 0.26 0.26) (thickness 0.04))))
  (fp_text user "secret" (at 0 0 0) (layer "F.Fab") (hide yes) (effects (font (size 0.26 0.26))))
  (pad "1" smd roundrect (at -0.5 0) (size 0.5 0.5)
    (layers "F.Cu" "F.Paste" "F.Mask")))"#;

    #[test]
    fn extracts_all_drawable_graphics_with_layers_and_widths() {
        use konnect_ipc::IpcGraphicDefinition as Graphic;
        let graphics = extract_graphic_definitions(GRAPHIC_FOOTPRINT).unwrap();

        let lines: Vec<_> = graphics
            .iter()
            .filter(|g| matches!(g, Graphic::Line { .. }))
            .collect();
        assert_eq!(lines.len(), 2);
        assert!(matches!(
            lines[0],
            Graphic::Line { layer, width, start, .. }
                if layer == "F.SilkS" && *width == 0.12 && *start == (-0.6, -0.5)
        ));

        let rect = graphics
            .iter()
            .find(|g| matches!(g, Graphic::Rect { .. }))
            .unwrap();
        assert!(matches!(
            rect,
            Graphic::Rect { layer, width, filled, start, end }
                if layer == "F.CrtYd" && *width == 0.05 && !*filled
                    && *start == (-0.8, -0.7) && *end == (0.8, 0.7)
        ));

        let circle = graphics
            .iter()
            .find(|g| matches!(g, Graphic::Circle { .. }))
            .unwrap();
        assert!(matches!(
            circle,
            Graphic::Circle { layer, filled, end, .. }
                if layer == "F.Fab" && *filled && *end == (0.25, 0.0)
        ));

        assert!(graphics
            .iter()
            .any(|g| matches!(g, Graphic::Arc { layer, mid, .. }
                if layer == "F.SilkS" && *mid == (0.0, -0.3))));

        let poly = graphics
            .iter()
            .find(|g| matches!(g, Graphic::Poly { .. }))
            .unwrap();
        assert!(matches!(
            poly,
            Graphic::Poly { points, filled, .. } if points.len() == 3 && *filled
        ));

        // Exactly one visible text: the fab ${REFERENCE}. The hidden fp_text,
        // the hidden Datasheet property, and the Reference/Value properties
        // (carried as first-class fields) are all excluded.
        let texts: Vec<_> = graphics
            .iter()
            .filter(|g| matches!(g, Graphic::Text { .. }))
            .collect();
        assert_eq!(texts.len(), 1, "{texts:?}");
        assert!(matches!(
            texts[0],
            Graphic::Text { text, layer, size, position, .. }
                if text == "${REFERENCE}" && layer == "F.Fab" && *size == 0.26
                    && *position == (0.0, 1.17)
        ));
    }

    #[test]
    fn legacy_reference_and_value_text_are_not_duplicated_as_graphics() {
        let source = r#"(footprint "Legacy"
  (layer "F.Cu")
  (fp_text reference "REF**" (at 0 -1) (layer "F.SilkS"))
  (fp_text value "Legacy" (at 0 1) (layer "F.Fab"))
  (fp_text user "visible" (at 0 0) (layer "F.Fab")))"#;

        let graphics = extract_graphic_definitions(source).unwrap();

        assert_eq!(graphics.len(), 1, "{graphics:?}");
        assert!(matches!(
            &graphics[0],
            konnect_ipc::IpcGraphicDefinition::Text { text, .. } if text == "visible"
        ));
    }

    #[test]
    fn a_bare_pads_only_footprint_extracts_no_graphics() {
        assert!(extract_graphic_definitions(FOOTPRINT).unwrap().is_empty());
    }

    fn test_ctx() -> ToolContext {
        ToolContext::new(
            crate::tools::ServerConfig {
                kicad_cli: String::new(),
                kicad_binary: String::new(),
                // No IPC address: any handler that reaches the IPC layer fails
                // with the socket-path configuration error, so a different
                // error proves the handler rejected before trying IPC.
                ipc_address: String::new(),
                project_dir: None,
                jlcpcb_db_path: None,
                auto_load_toolsets: false,
                eager_toolsets: false,
            },
            std::sync::Arc::new(crate::router::ToolRouter::new()),
        )
    }

    fn result_text(res: &CallToolResult) -> String {
        match res.content.first() {
            Some(crate::mcp::protocol::ToolContent::Text { text }) => text.clone(),
            other => panic!("expected text content, got {other:?}"),
        }
    }

    // ─── File-editing fallback (ported from emolitor's PR #66) ────────────────

    /// A library footprint in the exact shape KiCad ships: TAB-indented, name
    /// without a library prefix, `REF**` placeholder, no `(at …)`. CRLF, the
    /// way KiCad's bundled libraries are written.
    fn library_footprint() -> String {
        [
            "(footprint \"R_0805_2012Metric\"",
            "\t(version 20260206)",
            "\t(generator \"kicad-footprint-generator\")",
            "\t(layer \"F.Cu\")",
            "\t(descr \"Resistor SMD 0805\")",
            "\t(property \"Reference\" \"REF**\"",
            "\t\t(at 0 -1.65 0)",
            "\t\t(layer \"F.SilkS\")",
            "\t)",
            "\t(property \"Value\" \"R_0805_2012Metric\"",
            "\t\t(at 0 1.65 0)",
            "\t\t(layer \"F.Fab\")",
            "\t)",
            "\t(pad \"1\" smd roundrect",
            "\t\t(at -0.9125 0)",
            "\t\t(size 1.025 1.4)",
            "\t\t(layers \"F.Cu\" \"F.Paste\" \"F.Mask\")",
            "\t)",
            ")",
            "",
        ]
        .join("\r\n")
    }

    const EMPTY_BOARD: &str = "(kicad_pcb
\t(version 20260206)
\t(generator \"pcbnew\")
\t(net 0 \"\")
)
";

    /// A project directory holding a registered `Resistor_SMD.pretty` library
    /// with one footprint, plus an empty board. The project fp-lib-table makes
    /// `Resistor_SMD:R_0805_2012Metric` resolve hermetically — no global
    /// table, no environment.
    fn fallback_fixture(dir: &Path) -> std::path::PathBuf {
        let pretty = dir.join("Resistor_SMD.pretty");
        std::fs::create_dir_all(&pretty).unwrap();
        std::fs::write(
            pretty.join("R_0805_2012Metric.kicad_mod"),
            library_footprint(),
        )
        .unwrap();
        std::fs::write(
            dir.join("fp-lib-table"),
            format!(
                "(fp_lib_table\r\n\t(version 7)\r\n\t(lib (name \"Resistor_SMD\") (type \"KiCad\") (uri \"{}\") (options \"\") (descr \"\"))\r\n)\r\n",
                pretty.to_string_lossy()
            ),
        )
        .unwrap();
        let board = dir.join("b.kicad_pcb");
        std::fs::write(&board, EMPTY_BOARD).unwrap();
        board
    }

    fn legacy_fallback_fixture(dir: &Path) -> std::path::PathBuf {
        let pretty = dir.join("Legacy.pretty");
        std::fs::create_dir_all(&pretty).unwrap();
        std::fs::write(
            pretty.join("Socket.kicad_mod"),
            r#"(footprint "Socket" (version 20221018) (generator pcbnew)
  (layer "F.Cu")
  (attr exclude_from_pos_files)
  (fp_text reference "REF**" (at 0 -8.5) (layer "F.SilkS"))
  (fp_text value "Socket" (at 0 8.5) (layer "F.Fab"))
  (pad "1" thru_hole circle (at 0 0) (size 4 4) (drill 3) (layers "*.Cu" "*.Mask"))
  (model "../models/Socket.step"
    (offset (xyz 0 0 0))
    (scale (xyz 1 1 1))
    (rotate (xyz 0 0 0)))
)
"#,
        )
        .unwrap();
        std::fs::write(
            dir.join("fp-lib-table"),
            format!(
                "(fp_lib_table\n  (version 7)\n  (lib (name \"Legacy\") (type \"KiCad\") (uri \"{}\") (options \"\") (descr \"\"))\n)\n",
                pretty.to_string_lossy()
            ),
        )
        .unwrap();
        let board = dir.join("legacy.kicad_pcb");
        std::fs::write(&board, EMPTY_BOARD).unwrap();
        board
    }

    /// Net paren depth, ignoring anything inside quoted strings.
    fn count_parens(s: &str) -> i32 {
        let (mut depth, mut in_str, mut esc) = (0i32, false, false);
        for ch in s.chars() {
            match ch {
                _ if esc => esc = false,
                '\\' if in_str => esc = true,
                '"' => in_str = !in_str,
                '(' if !in_str => depth += 1,
                ')' if !in_str => depth -= 1,
                _ => {}
            }
        }
        depth
    }

    #[tokio::test]
    async fn unreachable_ipc_falls_back_to_writing_the_board_file() {
        // ipc_address is empty in test_ctx, which classifies as
        // transport-unreachable, and this fresh context has never observed the
        // board live, so the guarded file path is available.
        let tmp = tempfile::tempdir().unwrap();
        let board = fallback_fixture(tmp.path());

        let args = json!({
            "board": board.to_string_lossy(),
            "footprint": "Resistor_SMD:R_0805_2012Metric",
            "reference": "R7",
            "x": 50.0, "y": 60.0,
        });
        let res = handle_place_component(&args, &test_ctx()).await.unwrap();
        assert!(!res.is_error, "handler errored: {:?}", res.content);

        let out: serde_json::Value = serde_json::from_str(&result_text(&res)).unwrap();
        assert_eq!(out["source"], "file");
        assert_eq!(out["placed"], "R7");
        assert!(
            out["warning"]
                .as_str()
                .is_some_and(|w| w.contains("current Konnect server session")
                    && w.contains("crashed or was force-quit")),
            "the fallback must warn that the file was edited directly: {out}"
        );

        let written = std::fs::read_to_string(&board).unwrap();
        assert_eq!(
            written.matches("(footprint \"").count(),
            1,
            "no footprint:\n{written}"
        );
        assert!(
            written.contains("(footprint \"Resistor_SMD:R_0805_2012Metric\""),
            "board should carry the Library:Footprint id:\n{written}"
        );
        assert!(
            written.contains("(at 50 60 0)"),
            "placement missing:\n{written}"
        );
        assert!(
            written.contains("(property \"Reference\" \"R7\""),
            "{written}"
        );
        assert!(
            written.contains("(pad \"1\" smd roundrect"),
            "the full definition must be carried:\n{written}"
        );
        assert!(written.contains("(uuid \""), "board items need a uuid");
        assert_eq!(
            count_parens(&written),
            0,
            "board is no longer balanced:\n{written}"
        );
    }

    #[tokio::test]
    async fn a_previously_live_board_blocks_place_components_file_fallback() {
        let tmp = tempfile::tempdir().unwrap();
        let board = fallback_fixture(tmp.path());
        let before = std::fs::read(&board).unwrap();
        let ctx = test_ctx();
        ctx.board_session.observe_live(&board);
        let args = json!({
            "board": board.to_string_lossy(),
            "footprint": "Resistor_SMD:R_0805_2012Metric",
            "reference": "R7",
            "x": 50.0,
            "y": 60.0,
        });

        let result = handle_place_component(&args, &ctx).await.unwrap();

        assert!(result.is_error);
        assert_eq!(
            crate::mcp::error::extract_error_kind(&result).as_deref(),
            Some("unsafe_file_fallback")
        );
        assert_eq!(std::fs::read(&board).unwrap(), before);
    }

    #[tokio::test]
    async fn fallback_replaces_legacy_fp_text_reference_and_preserves_models_and_attributes() {
        let tmp = tempfile::tempdir().unwrap();
        let board = legacy_fallback_fixture(tmp.path());
        let args = json!({
            "board": board.to_string_lossy(),
            "footprint": "Legacy:Socket",
            "reference": "SW1",
            "x": 20.0,
            "y": 30.0,
        });

        let result = handle_place_component(&args, &test_ctx()).await.unwrap();

        assert!(!result.is_error, "{:?}", result.content);
        let written = std::fs::read_to_string(&board).unwrap();
        assert!(written.contains("(fp_text reference \"SW1\""), "{written}");
        assert!(!written.contains("REF**"), "{written}");
        assert!(
            written.contains("(attr exclude_from_pos_files)"),
            "{written}"
        );
        assert!(
            written.contains("(model \"../models/Socket.step\""),
            "{written}"
        );
        assert!(konnect_sexp::parse_sexp(&written).is_ok());
    }

    #[tokio::test]
    async fn fallback_rejects_a_duplicate_reference_without_writing() {
        let tmp = tempfile::tempdir().unwrap();
        let board = fallback_fixture(tmp.path());
        let args = json!({
            "board": board.to_string_lossy(),
            "footprint": "Resistor_SMD:R_0805_2012Metric",
            "reference": "R1",
            "x": 10.0,
            "y": 20.0,
        });
        let first = handle_place_component(&args, &test_ctx()).await.unwrap();
        assert!(!first.is_error, "{:?}", first.content);
        let before_duplicate = std::fs::read_to_string(&board).unwrap();

        let duplicate = handle_place_component(
            &json!({
                "board": board.to_string_lossy(),
                "footprint": "Resistor_SMD:R_0805_2012Metric",
                "reference": "R1",
                "x": 30.0,
                "y": 40.0,
            }),
            &test_ctx(),
        )
        .await
        .unwrap();

        assert!(duplicate.is_error);
        assert!(
            result_text(&duplicate).contains("already exists"),
            "{:?}",
            duplicate.content
        );
        assert_eq!(std::fs::read_to_string(board).unwrap(), before_duplicate);
    }

    /// A board whose footprint carries a `(property …)` *before* its root
    /// `(at …)`, with `angle` chosen so rotating changes the rendered width.
    ///
    /// KiCad's own writer emits `at` before properties and pads, so this shape
    /// never comes out of KiCad — but it is legal S-expression, KiCad loads it,
    /// and Konnect reads boards it did not write.
    fn board_with_a_property_before_the_root_at(property_angle: &str) -> String {
        format!(
            "(kicad_pcb (version 20241229) (generator \"pcbnew\")\n\
             \x20 (footprint \"Test:R\"\n\
             \x20   (layer \"F.Cu\")\n\
             \x20   (property \"Reference\" \"R1\"\n\
             \x20     (at 0 -1.65{property_angle})\n\
             \x20     (layer \"F.SilkS\")\n\
             \x20     (effects (font (size 1 1)))\n\
             \x20   )\n\
             \x20   (at 10 20 30)\n\
             \x20   (pad \"1\" smd roundrect (at -0.9125 0 30) (size 1 1) (layers \"F.Cu\"))\n\
             \x20 )\n\
             )"
        )
    }

    /// Rotating must not splice the root `(at …)` at offsets measured before
    /// the child rewrite moved them.
    ///
    /// `apply_rotation_to_children` reformats every pad/property/fp_text
    /// `(at …)`, and those replacements change length. Running it before the
    /// root splice left the root offsets stale by that delta.
    ///
    /// Both directions are covered because they failed differently and only
    /// one of them was obviously a failure:
    ///
    /// * shrinking — a −45° property rotating to exactly 0, which `format_at`
    ///   drops, moving the root four bytes: `(at (at 10 20 75) (pad "1" …`;
    /// * growing — a property with no angle at all gaining one, which tripped
    ///   the root-head check with "placement update changed the footprint
    ///   root" and gave no hint why.
    #[test]
    fn rotating_splices_the_root_at_before_the_children_move_it() {
        for (label, property_angle, target, expected_root, expected_property) in [
            ("shrinking", " -45", 75.0, "(at 10 20 75)", "(at 0 -1.65)"),
            // 0° + 240° of delta = 240°, which `rotate_at_block` folds to 60°
            // to keep the text readable. The root becomes -90, KiCad's
            // spelling of 270.
            ("growing", "", 270.0, "(at 10 20 -90)", "(at 0 -1.65 60)"),
        ] {
            let board = board_with_a_property_before_the_root_at(property_angle);
            let updated = prepare_closed_board_footprint_update(
                &board,
                "R1",
                FootprintPlacementUpdate::Rotate { rotation: target },
            )
            .unwrap_or_else(|error| panic!("{label}: {error:?}"));

            assert!(
                konnect_sexp::parse_sexp(&updated).is_ok(),
                "{label}: result must still parse\n{updated}"
            );
            assert!(
                updated.contains(expected_root),
                "{label}: root placement\n{updated}"
            );
            assert!(
                updated.contains(expected_property),
                "{label}: property angle\n{updated}"
            );
            assert!(
                !updated.contains("(at (at"),
                "{label}: a splice landed inside another block\n{updated}"
            );
        }
    }

    /// The root orientation is written the way KiCad writes it, and the
    /// children are not.
    ///
    /// Measured, not assumed. Rotating `R1` on the ecc83 demo to 247.5°
    /// through this path and then letting KiCad 10.0.5 re-save the board,
    /// KiCad rewrote the root `(at …)` as `-112.5` and left both pad angles at
    /// `247.5` exactly as Konnect had written them. Writing the un-normalised
    /// root means a footprint nobody touched shows a diff after the user's
    /// next save in KiCad, and it makes this path disagree with the IPC path —
    /// which ends up normalised because KiCad does it there.
    #[test]
    fn the_root_orientation_is_spelled_the_way_kicad_spells_it() {
        for (target, expected) in [
            (247.5, "(at 10 20 -112.5)"),
            (270.0, "(at 10 20 -90)"),
            (180.0, "(at 10 20 180)"),
            (181.0, "(at 10 20 -179)"),
            (90.0, "(at 10 20 90)"),
            (-450.0, "(at 10 20 -90)"),
            (360.0, "(at 10 20)"),
        ] {
            let board = board_with_a_property_before_the_root_at(" -45");
            let updated = prepare_closed_board_footprint_update(
                &board,
                "R1",
                FootprintPlacementUpdate::Rotate { rotation: target },
            )
            .unwrap_or_else(|error| panic!("{target}: {error:?}"));
            assert!(updated.contains(expected), "{target} =>\n{updated}");
        }

        // A move must not renormalise the angle it is preserving — that would
        // rewrite a root `(at …)` the caller did not ask to change. Checked on
        // a board whose root angle is already outside (-180, 180], which is
        // the only case where the two behaviours differ.
        let board = board_with_a_property_before_the_root_at(" -45")
            .replace("(at 10 20 30)", "(at 10 20 247.5)");
        let moved = prepare_closed_board_footprint_update(
            &board,
            "R1",
            FootprintPlacementUpdate::Move { x: 1.0, y: 2.0 },
        )
        .expect("move");
        assert!(moved.contains("(at 1 2 247.5)"), "{moved}");
    }

    /// A reference naming two footprints identifies none of them, and a board
    /// Konnect cannot parse is not one it will write to. Both refuse, both
    /// with a structured reason, and both leave the file byte-identical.
    ///
    /// Added because neutering each guard found nothing: the PR describes all
    /// three behaviours — missing reference, duplicate reference, unusable
    /// board — and only the missing one was exercised, for move or rotate.
    #[tokio::test]
    async fn a_duplicate_reference_or_an_unparseable_board_refuses_without_writing() {
        let tmp = tempfile::tempdir().unwrap();

        let footprint = "  (footprint \"Test:R\"\n    (layer \"F.Cu\")\n    (at 10 20 30)\n    \
             (property \"Reference\" \"R1\" (at 0 -1.65 30) (layer \"F.SilkS\"))\n    \
             (pad \"1\" smd roundrect (at -0.9125 0 30) (size 1 1) (layers \"F.Cu\"))\n  )";
        let duplicated = tmp.path().join("duplicate.kicad_pcb");
        std::fs::write(
            &duplicated,
            format!(
                "(kicad_pcb (version 20241229) (generator \"pcbnew\")\n{footprint}\n{footprint}\n)"
            ),
        )
        .unwrap();

        let truncated = tmp.path().join("truncated.kicad_pcb");
        std::fs::write(
            &truncated,
            "(kicad_pcb (version 20241229)\n  (footprint \"Test:R\"",
        )
        .unwrap();

        for (label, board, structured_reference) in [
            ("duplicate", &duplicated, true),
            ("truncated", &truncated, false),
        ] {
            let before = std::fs::read_to_string(board).unwrap();
            for tool in ["move", "rotate"] {
                let mut arguments = json!({
                    "board": board.to_string_lossy(),
                    "reference": "R1",
                });
                let result = if tool == "move" {
                    arguments["x"] = json!(40.0);
                    arguments["y"] = json!(50.0);
                    handle_move_component(&arguments, &test_ctx())
                        .await
                        .unwrap()
                } else {
                    arguments["rotation"] = json!(90.0);
                    handle_rotate_component(&arguments, &test_ctx())
                        .await
                        .unwrap()
                };

                assert!(result.is_error, "{label}/{tool} must refuse");
                let text = result_text(&result);
                if structured_reference {
                    assert!(
                        text.contains("invalid_argument") && text.contains("reference"),
                        "{label}/{tool}: {text}"
                    );
                } else {
                    assert!(
                        text.contains("Refusing to edit the board"),
                        "{label}/{tool}: {text}"
                    );
                }
                assert_eq!(
                    std::fs::read_to_string(board).unwrap(),
                    before,
                    "{label}/{tool} must leave the board byte-identical"
                );
            }
        }
    }

    /// The same board must survive a move, which does not rewrite children at
    /// all — pinned so a future refactor cannot reintroduce the coupling by
    /// making the move path share the rotate path's ordering.
    #[test]
    fn moving_keeps_every_child_untouched() {
        let board = board_with_a_property_before_the_root_at(" -45");
        let updated = prepare_closed_board_footprint_update(
            &board,
            "R1",
            FootprintPlacementUpdate::Move { x: 40.0, y: 50.0 },
        )
        .expect("move must succeed");

        assert!(updated.contains("(at 40 50 30)"), "{updated}");
        assert!(updated.contains("(at 0 -1.65 -45)"), "{updated}");
        assert!(updated.contains("(at -0.9125 0 30)"), "{updated}");
    }

    async fn placed_fallback_fixture(dir: &Path) -> std::path::PathBuf {
        let board = fallback_fixture(dir);
        let placed = handle_place_component(
            &json!({
                "board": board.to_string_lossy(),
                "footprint": "Resistor_SMD:R_0805_2012Metric",
                "reference": "R1",
                "x": 10.0,
                "y": 20.0,
                "rotation": 30.0,
            }),
            &test_ctx(),
        )
        .await
        .unwrap();
        assert!(!placed.is_error, "{:?}", placed.content);
        board
    }

    #[tokio::test]
    async fn unreachable_ipc_moves_an_existing_footprint_in_the_board_file() {
        let tmp = tempfile::tempdir().unwrap();
        let board = placed_fallback_fixture(tmp.path()).await;

        let moved = handle_move_component(
            &json!({
                "board": board.to_string_lossy(),
                "reference": "R1",
                "x": 40.0,
                "y": 50.0,
            }),
            &test_ctx(),
        )
        .await
        .unwrap();

        assert!(!moved.is_error, "{:?}", moved.content);
        let result: serde_json::Value =
            serde_json::from_str(&result_text(&moved)).expect("move result must be JSON");
        assert_eq!(result["source"], "file");
        let written = std::fs::read_to_string(board).unwrap();
        assert!(written.contains("(at 40 50 30)"), "{written}");
        assert!(written.contains("(pad \"1\" smd roundrect"), "{written}");
        assert!(konnect_sexp::parse_sexp(&written).is_ok());
    }

    #[tokio::test]
    async fn unreachable_ipc_rotates_an_existing_footprint_in_the_board_file() {
        let tmp = tempfile::tempdir().unwrap();
        let board = placed_fallback_fixture(tmp.path()).await;

        let rotated = handle_rotate_component(
            &json!({
                "board": board.to_string_lossy(),
                "reference": "R1",
                "rotation": 270.0,
            }),
            &test_ctx(),
        )
        .await
        .unwrap();

        assert!(!rotated.is_error, "{:?}", rotated.content);
        let result: serde_json::Value =
            serde_json::from_str(&result_text(&rotated)).expect("rotate result must be JSON");
        assert_eq!(result["source"], "file");
        let written = std::fs::read_to_string(board).unwrap();
        // The root orientation is normalised to (-180, 180] because that is
        // what KiCad writes; the pad and text angles are not, because KiCad
        // leaves those alone. Both measured against a real KiCad 10 re-save.
        assert!(written.contains("(at 10 20 -90)"), "{written}");
        assert!(written.contains("(at -0.9125 0 270)"), "{written}");
        assert!(written.contains("(at 0 -1.65 90)"), "{written}");
        assert!(konnect_sexp::parse_sexp(&written).is_ok());
    }

    #[tokio::test]
    async fn unreachable_ipc_sets_multiple_placements_with_one_file_write() {
        let tmp = tempfile::tempdir().unwrap();
        let board = fallback_fixture(tmp.path());
        for (reference, x) in [("R1", 10.0), ("R2", 20.0)] {
            let placed = handle_place_component(
                &json!({
                    "board": board.to_string_lossy(),
                    "footprint": "Resistor_SMD:R_0805_2012Metric",
                    "reference": reference,
                    "x": x,
                    "y": 20.0,
                    "rotation": 0.0,
                }),
                &test_ctx(),
            )
            .await
            .unwrap();
            assert!(!placed.is_error, "{reference}: {:?}", placed.content);
        }

        let result = handle_set_component_placements(
            &json!({
                "board": board.to_string_lossy(),
                "placements": [
                    {"reference": "R1", "x": 40.0, "y": 50.0, "rotation": 270.0},
                    {"reference": "R2", "x": 60.0, "y": 70.0, "rotation": 45.0}
                ]
            }),
            &test_ctx(),
        )
        .await
        .unwrap();

        assert!(!result.is_error, "{:?}", result.content);
        let response: serde_json::Value =
            serde_json::from_str(&result_text(&result)).expect("batch result must be JSON");
        assert_eq!(response["source"], "file");
        assert_eq!(response["count"], 2);
        let written = std::fs::read_to_string(&board).unwrap();
        assert!(written.contains("(at 40 50 -90)"), "{written}");
        assert!(written.contains("(at 60 70 45)"), "{written}");
        assert!(konnect_sexp::parse_sexp(&written).is_ok());

        // The response reports what the file now holds, not what was asked:
        // the requested 270 is stored (and therefore reported) as -90. An
        // echoed 270 here and a -90 in the file would be two different
        // answers for one final state.
        assert_eq!(response["placements"][0]["reference"], "R1");
        assert_eq!(response["placements"][0]["rotation"], -90.0, "{response}");
        assert_eq!(response["placements"][1]["rotation"], 45.0);
        assert_eq!(response["placements"][0]["x"], 40.0);
        assert_eq!(response["placements"][1]["y"], 70.0);
    }

    #[tokio::test]
    async fn placement_batch_is_all_or_nothing_for_missing_and_duplicate_references() {
        let tmp = tempfile::tempdir().unwrap();
        let board = placed_fallback_fixture(tmp.path()).await;
        let before = std::fs::read_to_string(&board).unwrap();

        let missing = handle_set_component_placements(
            &json!({
                "board": board.to_string_lossy(),
                "placements": [
                    {"reference": "R1", "x": 40.0, "y": 50.0, "rotation": 90.0},
                    {"reference": "R404", "x": 60.0, "y": 70.0, "rotation": 0.0}
                ]
            }),
            &test_ctx(),
        )
        .await
        .unwrap();
        assert!(missing.is_error);
        assert!(result_text(&missing).contains("R404"));
        assert_eq!(std::fs::read_to_string(&board).unwrap(), before);

        let duplicate = handle_set_component_placements(
            &json!({
                "board": board.to_string_lossy(),
                "placements": [
                    {"reference": "R1", "x": 40.0, "y": 50.0, "rotation": 90.0},
                    {"reference": "R1", "x": 60.0, "y": 70.0, "rotation": 0.0}
                ]
            }),
            &test_ctx(),
        )
        .await
        .unwrap();
        let text = result_text(&duplicate);
        assert!(duplicate.is_error);
        assert!(text.contains("invalid_argument") && text.contains("placements[1].reference"));
        assert_eq!(std::fs::read_to_string(&board).unwrap(), before);
    }

    #[tokio::test]
    async fn closed_board_move_and_rotate_reject_a_missing_reference_without_writing() {
        let tmp = tempfile::tempdir().unwrap();
        let board = placed_fallback_fixture(tmp.path()).await;
        let before = std::fs::read_to_string(&board).unwrap();

        let moved = handle_move_component(
            &json!({
                "board": board.to_string_lossy(),
                "reference": "R404",
                "x": 40.0,
                "y": 50.0,
            }),
            &test_ctx(),
        )
        .await
        .unwrap();
        assert!(moved.is_error);
        assert!(result_text(&moved).contains("R404"));
        // Not just "an error": a reference that names nothing on this board is
        // the caller's mistake, so it carries the structured field rather than
        // reaching them as an opaque handler_error (#194's class).
        assert!(
            result_text(&moved).contains("invalid_argument")
                && result_text(&moved).contains("\"field\":\"reference\""),
            "{}",
            result_text(&moved)
        );
        assert_eq!(std::fs::read_to_string(&board).unwrap(), before);

        let rotated = handle_rotate_component(
            &json!({
                "board": board.to_string_lossy(),
                "reference": "R404",
                "rotation": 90.0,
            }),
            &test_ctx(),
        )
        .await
        .unwrap();
        assert!(rotated.is_error);
        assert!(result_text(&rotated).contains("R404"));
        assert_eq!(std::fs::read_to_string(&board).unwrap(), before);
    }

    #[tokio::test]
    async fn reachable_rejection_prevents_placement_file_fallbacks() {
        let tmp = tempfile::tempdir().unwrap();
        let board = placed_fallback_fixture(tmp.path()).await;
        let before = std::fs::read_to_string(&board).unwrap();
        let server = spawn_rejecting_kicad();
        let ctx = ToolContext::new(
            crate::tools::ServerConfig {
                kicad_cli: String::new(),
                kicad_binary: String::new(),
                ipc_address: server.address().to_string(),
                project_dir: None,
                jlcpcb_db_path: None,
                auto_load_toolsets: false,
                eager_toolsets: false,
            },
            std::sync::Arc::new(crate::router::ToolRouter::new()),
        );

        let moved = handle_move_component(
            &json!({
                "board": board.to_string_lossy(),
                "reference": "R1",
                "x": 40.0,
                "y": 50.0,
            }),
            &ctx,
        )
        .await
        .unwrap();
        assert!(moved.is_error);
        assert!(result_text(&moved).contains("not modified"));
        assert_eq!(std::fs::read_to_string(&board).unwrap(), before);

        let rotated = handle_rotate_component(
            &json!({
                "board": board.to_string_lossy(),
                "reference": "R1",
                "rotation": 90.0,
            }),
            &ctx,
        )
        .await
        .unwrap();
        assert!(rotated.is_error);
        assert!(result_text(&rotated).contains("not modified"));
        assert_eq!(std::fs::read_to_string(&board).unwrap(), before);

        let batch = handle_set_component_placements(
            &json!({
                "board": board.to_string_lossy(),
                "placements": [
                    {"reference": "R1", "x": 40.0, "y": 50.0, "rotation": 90.0}
                ]
            }),
            &ctx,
        )
        .await
        .unwrap();
        assert!(batch.is_error);
        assert!(result_text(&batch).contains("not modified"));
        assert_eq!(std::fs::read_to_string(&board).unwrap(), before);
    }

    const FLIP_FOOTPRINT: &str = r#"(footprint "Test:Flip"
  (layer "F.Cu")
  (at 10 20 30)
  (property "Reference" "U1"
    (at 1 -2 40)
    (layer "F.SilkS")
    (effects (font (size 1 1)) (justify left))
  )
  (fp_line (start 1 2) (end 3 4)
    (stroke (width 0.1) (type solid))
    (layer "F.SilkS")
  )
  (fp_arc (start 1 2) (mid 3 4) (end 5 6)
    (stroke (width 0.1) (type solid))
    (layer "F.Fab")
  )
  (fp_poly (pts (xy 1 2) (xy 3 4) (xy 5 6))
    (stroke (width 0.1) (type solid))
    (fill no)
    (layer "F.CrtYd")
  )
  (pad "1" smd roundrect (at 2 3 50) (size 1 2)
    (layers "F.Cu" "F.Paste" "F.Mask")
    (roundrect_rratio 0.25)
  )
  (model "../models/Test.step"
    (offset (xyz 0 0 0))
    (scale (xyz 1 1 1))
    (rotate (xyz 0 0 90))
  )
)"#;

    fn flip_board(footprints: &[&str], eol: &str) -> String {
        let body = footprints
            .iter()
            .flat_map(|footprint| footprint.lines())
            .map(|line| format!("  {line}"))
            .collect::<Vec<_>>()
            .join(eol);
        format!(
            "(kicad_pcb{eol}  (version 20260206){eol}  (generator \"pcbnew\"){eol}  \
             (net 0 \"\"){eol}{body}{eol}){eol}"
        )
    }

    #[test]
    fn flip_footprint_matches_kicads_library_frame_transform() {
        let flipped = flip_footprint_block(FLIP_FOOTPRINT).unwrap();

        assert!(flipped.contains("(layer \"B.Cu\")"), "{flipped}");
        assert!(flipped.contains("(at 10 20 -30)"), "{flipped}");
        assert!(flipped.contains("(at 1 2 140)"), "{flipped}");
        assert!(flipped.contains("(layer \"B.SilkS\")"), "{flipped}");
        assert!(flipped.contains("(justify left mirror)"), "{flipped}");
        assert!(flipped.contains("(start 1 -2)"), "{flipped}");
        assert!(flipped.contains("(end 3 -4)"), "{flipped}");
        assert!(flipped.contains("(start 5 -6)"), "{flipped}");
        assert!(flipped.contains("(mid 3 -4)"), "{flipped}");
        assert!(flipped.contains("(end 1 -2)"), "{flipped}");
        assert!(flipped.contains("(xy 1 -2)"), "{flipped}");
        assert!(flipped.contains("(at 2 -3 310)"), "{flipped}");
        assert!(
            flipped.contains("(layers \"B.Cu\" \"B.Paste\" \"B.Mask\")"),
            "{flipped}"
        );
        // The model is carried through verbatim. That is only safe because a
        // model whose placement a flip would have to move is refused outright
        // — see `a_model_a_flip_would_move_is_refused`. `rotate.z` is not one
        // of those: a flip does not touch Z.
        assert!(
            flipped.contains("(offset (xyz 0 0 0))") && flipped.contains("(rotate (xyz 0 0 90))"),
            "a model a flip does not move must survive verbatim: {flipped}"
        );
        assert!(konnect_sexp::parse_sexp(&flipped).is_ok());
    }

    /// A property with no position is metadata, not geometry, and must not
    /// stop the flip.
    ///
    /// KiCad writes `(property ki_fp_filters "R_* Resistor_*")` — a bare
    /// token, no `(at …)`, no layer — into every footprint it places from a
    /// library. There are **779** of them across the 19 boards in
    /// `share/kicad/demos`. Requiring exactly one `(at …)` on every property
    /// therefore refused practically every real board with "property must
    /// contain exactly one direct (at ...) block", which is what the first
    /// live run of this tool hit on the stock ecc83 demo.
    ///
    /// Nothing offline could have caught it: the synthetic fixture has only
    /// positioned properties.
    #[test]
    fn a_positionless_property_does_not_block_the_flip() {
        let with_metadata = FLIP_FOOTPRINT.replace(
            "  (pad \"1\"",
            "  (property ki_fp_filters \"R_* Resistor_*\")\n  (pad \"1\"",
        );
        assert!(
            with_metadata.contains("ki_fp_filters"),
            "fixture must carry the metadata property"
        );

        let flipped = flip_footprint_block(&with_metadata)
            .expect("a positionless property must not block the flip");

        // Carried through untouched — it has no geometry to mirror.
        assert!(
            flipped.contains("(property ki_fp_filters \"R_* Resistor_*\")"),
            "{flipped}"
        );
        // And the positioned ones still flipped.
        assert!(flipped.contains("(layer \"B.Cu\")"), "{flipped}");
        assert!(konnect_sexp::parse_sexp(&flipped).is_ok());
    }

    /// A footprint that is not on either copper side has no "other side" to
    /// flip to, so it is refused rather than moved to one.
    ///
    /// Reachable on any board a user hand-edited or an older tool wrote — and
    /// the guard existed with nothing exercising it, which the neuter pass
    /// found.
    #[tokio::test]
    async fn a_footprint_on_neither_side_of_the_board_is_refused() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("inner.kicad_pcb");
        let stranded = FLIP_FOOTPRINT.replace("(layer \"F.Cu\")", "(layer \"In1.Cu\")");
        let before = flip_board(&[&stranded], "\n");
        std::fs::write(&board, &before).unwrap();

        let refusal = handle_flip_component(
            &json!({
                "board": board.to_string_lossy(),
                "reference": "U1",
                "layer": "B.Cu",
            }),
            &test_ctx(),
        )
        .await
        .unwrap();

        assert!(refusal.is_error, "{:?}", refusal.content);
        let text = result_text(&refusal);
        assert!(text.contains("In1.Cu"), "{text}");
        assert!(text.contains("neither side"), "{text}");
        assert_eq!(std::fs::read_to_string(board).unwrap(), before);
    }

    /// KiCad's own flip transforms a model's Y offset and its X/Y rotation.
    /// This path does not transform models at all, so rather than silently
    /// leave one behind it refuses — the same policy the rest of the flip
    /// applies to geometry it cannot mirror.
    ///
    /// Refusing costs almost nothing. Across all **14,818** footprints in
    /// KiCad 10's standard libraries carrying a `(model …)`, `offset.y` is
    /// non-zero in **3** and `rotate.x`/`rotate.y` in **none**; the worst is
    /// `RaspberryPi_Pico_Common_THT` at -24.13 mm, which would put its model
    /// roughly 48 mm out. The 84 with a non-zero `rotate.z` are unaffected.
    #[test]
    fn a_model_a_flip_would_move_is_refused() {
        for (label, replacement, needle) in [
            ("offset.y", "(offset (xyz 0 -24.13 0))", "offset.y"),
            ("rotate.x", "(rotate (xyz 90 0 0))", "rotate.x"),
            ("rotate.y", "(rotate (xyz 0 90 0))", "rotate.y"),
        ] {
            let source = FLIP_FOOTPRINT
                .replace("(offset (xyz 0 0 0))", replacement)
                .replace("(rotate (xyz 0 0 90))", replacement);
            let error = flip_footprint_block(&source).unwrap_err().to_string();
            assert!(error.contains(needle), "{label}: {error}");
            assert!(error.contains("would have to move it"), "{label}: {error}");
        }

        // The fields a flip leaves alone must not trip it.
        for untouched in ["(offset (xyz 8.89 0 0))", "(rotate (xyz 0 0 90))"] {
            let source = FLIP_FOOTPRINT
                .replace("(offset (xyz 0 0 0))", untouched)
                .replace("(rotate (xyz 0 0 90))", untouched);
            assert!(
                flip_footprint_block(&source).is_ok(),
                "{untouched} is not moved by a flip and must be accepted"
            );
        }
    }

    #[tokio::test]
    async fn unreachable_ipc_flips_an_existing_footprint_to_the_requested_side() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("flip.kicad_pcb");
        std::fs::write(
            &board,
            format!(
                "(kicad_pcb\n  (version 20260206)\n  (generator \"pcbnew\")\n  (net 0 \"\")\n{}\n)\n",
                FLIP_FOOTPRINT
                    .lines()
                    .map(|line| format!("  {line}"))
                    .collect::<Vec<_>>()
                    .join("\n")
            ),
        )
        .unwrap();

        let flipped = handle_flip_component(
            &json!({
                "board": board.to_string_lossy(),
                "reference": "U1",
                "layer": "B.Cu",
            }),
            &test_ctx(),
        )
        .await
        .unwrap();

        assert!(!flipped.is_error, "{:?}", flipped.content);
        let result: serde_json::Value =
            serde_json::from_str(&result_text(&flipped)).expect("flip result must be JSON");
        assert_eq!(result["source"], "file");
        assert_eq!(result["layer"], "B.Cu");
        let written = std::fs::read_to_string(&board).unwrap();
        assert!(written.contains("(layer \"B.Cu\")"), "{written}");
        assert!(written.contains("(layers \"B.Cu\" \"B.Paste\" \"B.Mask\")"));

        let repeated = handle_flip_component(
            &json!({
                "board": board.to_string_lossy(),
                "reference": "U1",
                "layer": "B.Cu",
            }),
            &test_ctx(),
        )
        .await
        .unwrap();
        assert!(!repeated.is_error, "{:?}", repeated.content);
        assert_eq!(std::fs::read_to_string(board).unwrap(), written);
    }

    #[tokio::test]
    async fn a_reachable_kicad_that_rejects_the_flip_never_touches_the_file() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("flip.kicad_pcb");
        let before = format!(
            "(kicad_pcb\n  (version 20260206)\n  (generator \"pcbnew\")\n  (net 0 \"\")\n{}\n)\n",
            FLIP_FOOTPRINT
                .lines()
                .map(|line| format!("  {line}"))
                .collect::<Vec<_>>()
                .join("\n")
        );
        std::fs::write(&board, &before).unwrap();
        let server = spawn_rejecting_kicad();
        let ctx = ToolContext::new(
            crate::tools::ServerConfig {
                kicad_cli: String::new(),
                kicad_binary: String::new(),
                ipc_address: server.address().to_string(),
                project_dir: None,
                jlcpcb_db_path: None,
                auto_load_toolsets: false,
                eager_toolsets: false,
            },
            std::sync::Arc::new(crate::router::ToolRouter::new()),
        );

        let flipped = handle_flip_component(
            &json!({
                "board": board.to_string_lossy(),
                "reference": "U1",
                "layer": "B.Cu",
            }),
            &ctx,
        )
        .await
        .unwrap();

        // Since #604, flip_component performs the write over IPC via
        // attempt_ipc_write, exactly like set_component_placements: a
        // reachable KiCad that answers but rejects the request may be alive
        // and holding this exact board, so the file must not be touched —
        // "Rejected" fails closed rather than falling back to the file
        // (mirrors `a_reachable_kicad_that_rejects_never_touches_the_file`).
        assert!(
            flipped.is_error,
            "a rejection must not be reported as success"
        );
        let text = result_text(&flipped);
        assert!(
            text.contains("rejected the footprint flip") && text.contains("not modified"),
            "the error must say the file was left alone: {text}"
        );
        assert_eq!(
            std::fs::read_to_string(&board).unwrap(),
            before,
            "a reachable KiCAD that says no must never trigger the file fallback"
        );
    }

    /// A rep0 endpoint playing a KiCad that holds `board` open with one
    /// footprint on it, answering `GetItems`/`BeginCommit`/`EndCommit`/
    /// `GetVersion`, and dispatching `FlipItems` to `flip_items`. Exercises
    /// `handle_flip_component`'s IPC path end-to-end through
    /// `attempt_ipc_write` (issue #604) — not just `flip_footprint` in
    /// isolation, which `flip_footprint_test.rs` already covers.
    fn spawn_flip_capable_kicad(
        board: &std::path::Path,
        footprint: std::sync::Arc<
            std::sync::Mutex<konnect_ipc::gen::kiapi::board::types::FootprintInstance>,
        >,
        flip_items: impl Fn(
                &konnect_ipc::gen::kiapi::board::commands::FlipItems,
                &mut konnect_ipc::gen::kiapi::board::types::FootprintInstance,
            ) -> konnect_ipc::gen::kiapi::board::commands::FlipItemsResponse
            + Send
            + 'static,
    ) -> crate::test_support::MockIpcServer {
        use konnect_ipc::gen::kiapi;
        use prost::Message;

        let board = board.to_string_lossy().to_string();
        crate::test_support::MockIpcServer::spawn("flip-capable-kicad", move |request| {
            let command = request.message.expect("a command");
            let body = if command.type_url.ends_with("GetOpenDocuments") {
                Some(konnect_ipc::builders::pack_any(
                    &kiapi::common::commands::GetOpenDocumentsResponse {
                        documents: vec![kiapi::common::types::DocumentSpecifier {
                            r#type: kiapi::common::types::DocumentType::DoctypePcb as i32,
                            project: None,
                            identifier: Some(
                                kiapi::common::types::document_specifier::Identifier::BoardFilename(
                                    board.clone(),
                                ),
                            ),
                        }],
                    },
                    "kiapi.common.commands.GetOpenDocumentsResponse",
                ))
            } else if command.type_url.ends_with("GetItems") {
                let fp = footprint.lock().unwrap().clone();
                Some(konnect_ipc::builders::pack_any(
                    &kiapi::common::commands::GetItemsResponse {
                        header: None,
                        status: kiapi::common::types::ItemRequestStatus::IrsOk as i32,
                        items: vec![konnect_ipc::builders::pack_any(
                            &fp,
                            "kiapi.board.types.FootprintInstance",
                        )],
                    },
                    "kiapi.common.commands.GetItemsResponse",
                ))
            } else if command.type_url.ends_with("BeginCommit") {
                Some(konnect_ipc::builders::pack_any(
                    &kiapi::common::commands::BeginCommitResponse {
                        id: Some(kiapi::common::types::Kiid {
                            value: "flip-commit".to_string(),
                        }),
                    },
                    "kiapi.common.commands.BeginCommitResponse",
                ))
            } else if command.type_url.ends_with("EndCommit") {
                Some(konnect_ipc::builders::pack_any(
                    &kiapi::common::commands::EndCommitResponse {},
                    "kiapi.common.commands.EndCommitResponse",
                ))
            } else if command.type_url.ends_with("kiapi.board.commands.FlipItems") {
                let flip =
                    kiapi::board::commands::FlipItems::decode(command.value.as_slice()).unwrap();
                let mut held = footprint.lock().unwrap();
                let response = flip_items(&flip, &mut held);
                Some(konnect_ipc::builders::pack_any(
                    &response,
                    "kiapi.board.commands.FlipItemsResponse",
                ))
            } else if command.type_url.ends_with("GetVersion") {
                Some(konnect_ipc::builders::pack_any(
                    &kiapi::common::commands::GetVersionResponse {
                        version: Some(kiapi::common::types::KiCadVersion {
                            major: 10,
                            minor: 0,
                            patch: 5,
                            full_version: "10.0.5".to_string(),
                        }),
                    },
                    "kiapi.common.commands.GetVersionResponse",
                ))
            } else {
                None
            };
            kiapi::common::ApiResponse {
                status: Some(kiapi::common::ApiResponseStatus {
                    status: kiapi::common::ApiStatusCode::AsOk as i32,
                    error_message: String::new(),
                }),
                header: None,
                message: body,
            }
        })
    }

    fn flip_test_footprint(
        kiid: &str,
        layer: konnect_ipc::gen::kiapi::board::types::BoardLayer,
    ) -> konnect_ipc::gen::kiapi::board::types::FootprintInstance {
        use konnect_ipc::gen::kiapi;
        kiapi::board::types::FootprintInstance {
            id: Some(kiapi::common::types::Kiid {
                value: kiid.to_string(),
            }),
            position: Some(konnect_ipc::builders::vec2(10.0, 20.0)),
            orientation: Some(kiapi::common::types::Angle { value_degrees: 0.0 }),
            layer: layer as i32,
            reference_field: Some(kiapi::board::types::Field {
                name: "Reference".to_string(),
                text: Some(kiapi::board::types::BoardText {
                    text: Some(kiapi::common::types::Text {
                        text: "U1".to_string(),
                        ..Default::default()
                    }),
                    ..Default::default()
                }),
                ..Default::default()
            }),
            definition: Some(kiapi::board::types::Footprint {
                ..Default::default()
            }),
            ..Default::default()
        }
    }

    #[tokio::test]
    async fn flip_over_ipc_succeeds_when_kicad_holds_this_exact_board() {
        use konnect_ipc::gen::kiapi;

        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("flip.kicad_pcb");
        std::fs::write(&board, "(kicad_pcb\n  (version 20260206)\n)\n").unwrap();
        let before = std::fs::read_to_string(&board).unwrap();

        let footprint = std::sync::Arc::new(std::sync::Mutex::new(flip_test_footprint(
            "U1-kiid",
            kiapi::board::types::BoardLayer::BlFCu,
        )));
        let server = spawn_flip_capable_kicad(&board, footprint, |flip, held| {
            assert_eq!(
                flip.direction(),
                kiapi::board::commands::BoardFlipDirection::BfdTopBottom
            );
            held.layer = kiapi::board::types::BoardLayer::BlBCu as i32;
            let item = konnect_ipc::builders::pack_any(held, "kiapi.board.types.FootprintInstance");
            kiapi::board::commands::FlipItemsResponse {
                header: None,
                status: kiapi::common::types::ItemRequestStatus::IrsOk as i32,
                flipped_items: vec![kiapi::board::commands::ItemFlipResult {
                    status: Some(kiapi::common::commands::ItemStatus {
                        code: kiapi::common::commands::ItemStatusCode::IscOk as i32,
                        error_message: String::new(),
                    }),
                    item: Some(item),
                }],
            }
        });
        let ctx = crate::tools::pcb_board::board_mock::ctx_talking_to(server.address().to_string());

        let result = handle_flip_component(
            &json!({"board": board, "reference": "U1", "layer": "B.Cu"}),
            &ctx,
        )
        .await
        .unwrap();

        assert!(!result.is_error, "{:?}", result.content);
        let body: serde_json::Value =
            serde_json::from_str(&result_text(&result)).expect("json result");
        assert_eq!(body["source"], "ipc");
        assert_eq!(body["layer"], "B.Cu");
        assert_eq!(body["flipped"], "U1");
        // The IPC path never touches the file.
        assert_eq!(std::fs::read_to_string(&board).unwrap(), before);
    }

    #[tokio::test]
    async fn flip_over_ipc_reports_unsupported_capability_on_as_unhandled() {
        use konnect_ipc::gen::kiapi;

        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("flip.kicad_pcb");
        std::fs::write(&board, "(kicad_pcb\n  (version 20260206)\n)\n").unwrap();
        let before = std::fs::read_to_string(&board).unwrap();

        let footprint = std::sync::Arc::new(std::sync::Mutex::new(flip_test_footprint(
            "U1-kiid",
            kiapi::board::types::BoardLayer::BlFCu,
        )));
        let board_for_mock = board.clone();
        let server =
            crate::test_support::MockIpcServer::spawn("flip-unsupported", move |request| {
                let command = request.message.expect("a command");
                let body = if command.type_url.ends_with("GetOpenDocuments") {
                    Some(konnect_ipc::builders::pack_any(
                        &kiapi::common::commands::GetOpenDocumentsResponse {
                            documents: vec![kiapi::common::types::DocumentSpecifier {
                            r#type: kiapi::common::types::DocumentType::DoctypePcb as i32,
                            project: None,
                            identifier: Some(
                                kiapi::common::types::document_specifier::Identifier::BoardFilename(
                                    board_for_mock.to_string_lossy().to_string(),
                                ),
                            ),
                        }],
                        },
                        "kiapi.common.commands.GetOpenDocumentsResponse",
                    ))
                } else if command.type_url.ends_with("GetItems") {
                    let fp = footprint.lock().unwrap().clone();
                    Some(konnect_ipc::builders::pack_any(
                        &kiapi::common::commands::GetItemsResponse {
                            header: None,
                            status: kiapi::common::types::ItemRequestStatus::IrsOk as i32,
                            items: vec![konnect_ipc::builders::pack_any(
                                &fp,
                                "kiapi.board.types.FootprintInstance",
                            )],
                        },
                        "kiapi.common.commands.GetItemsResponse",
                    ))
                } else if command.type_url.ends_with("BeginCommit") {
                    Some(konnect_ipc::builders::pack_any(
                        &kiapi::common::commands::BeginCommitResponse {
                            id: Some(kiapi::common::types::Kiid {
                                value: "flip-commit".to_string(),
                            }),
                        },
                        "kiapi.common.commands.BeginCommitResponse",
                    ))
                } else if command.type_url.ends_with("EndCommit") {
                    Some(konnect_ipc::builders::pack_any(
                        &kiapi::common::commands::EndCommitResponse {},
                        "kiapi.common.commands.EndCommitResponse",
                    ))
                } else if command.type_url.ends_with("GetVersion") {
                    Some(konnect_ipc::builders::pack_any(
                        &kiapi::common::commands::GetVersionResponse {
                            version: Some(kiapi::common::types::KiCadVersion {
                                major: 10,
                                minor: 0,
                                patch: 5,
                                full_version: "10.0.5".to_string(),
                            }),
                        },
                        "kiapi.common.commands.GetVersionResponse",
                    ))
                } else if command.type_url.ends_with("kiapi.board.commands.FlipItems") {
                    // AS_UNHANDLED: 10.0.5 predates the native FlipItems handler.
                    return kiapi::common::ApiResponse {
                        status: Some(kiapi::common::ApiResponseStatus {
                            status: kiapi::common::ApiStatusCode::AsUnhandled as i32,
                            error_message: "FlipItems is not handled".to_string(),
                        }),
                        header: None,
                        message: None,
                    };
                } else {
                    None
                };
                kiapi::common::ApiResponse {
                    status: Some(kiapi::common::ApiResponseStatus {
                        status: kiapi::common::ApiStatusCode::AsOk as i32,
                        error_message: String::new(),
                    }),
                    header: None,
                    message: body,
                }
            });
        let ctx = crate::tools::pcb_board::board_mock::ctx_talking_to(server.address().to_string());

        let result = handle_flip_component(
            &json!({"board": board, "reference": "U1", "layer": "B.Cu"}),
            &ctx,
        )
        .await
        .unwrap();

        assert!(
            result.is_error,
            "AS_UNHANDLED must not be reported as success"
        );
        assert_eq!(
            crate::mcp::error::extract_error_kind(&result).as_deref(),
            Some("unsupported_capability")
        );
        let parsed: serde_json::Value =
            serde_json::from_str(&result_text(&result)).expect("json error body");
        assert_eq!(parsed["error"]["kicad_version"], "10.0.5");
        assert_eq!(std::fs::read_to_string(&board).unwrap(), before);
    }

    #[tokio::test]
    async fn flip_over_ipc_reports_uncertain_outcome_when_readback_disagrees() {
        use konnect_ipc::gen::kiapi;

        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("flip.kicad_pcb");
        std::fs::write(&board, "(kicad_pcb\n  (version 20260206)\n)\n").unwrap();
        let before = std::fs::read_to_string(&board).unwrap();

        let footprint = std::sync::Arc::new(std::sync::Mutex::new(flip_test_footprint(
            "U1-kiid",
            kiapi::board::types::BoardLayer::BlFCu,
        )));
        // The mutation response claims success and a new layer, but the mock
        // never actually updates `held` — so the fresh readback GetItems that
        // follows still reports F.Cu. This is exactly the divergence the
        // readback guard exists to catch (issue #604 acceptance criteria).
        let server = spawn_flip_capable_kicad(&board, footprint, |_flip, held| {
            let mut echoed = held.clone();
            echoed.layer = kiapi::board::types::BoardLayer::BlBCu as i32;
            let item =
                konnect_ipc::builders::pack_any(&echoed, "kiapi.board.types.FootprintInstance");
            kiapi::board::commands::FlipItemsResponse {
                header: None,
                status: kiapi::common::types::ItemRequestStatus::IrsOk as i32,
                flipped_items: vec![kiapi::board::commands::ItemFlipResult {
                    status: Some(kiapi::common::commands::ItemStatus {
                        code: kiapi::common::commands::ItemStatusCode::IscOk as i32,
                        error_message: String::new(),
                    }),
                    item: Some(item),
                }],
            }
        });
        let ctx = crate::tools::pcb_board::board_mock::ctx_talking_to(server.address().to_string());

        let result = handle_flip_component(
            &json!({"board": board, "reference": "U1", "layer": "B.Cu"}),
            &ctx,
        )
        .await
        .unwrap();

        assert!(result.is_error, "an unconfirmed mutation is not a success");
        assert_eq!(
            crate::mcp::error::extract_error_kind(&result).as_deref(),
            Some("mutation_outcome_uncertain")
        );
        assert_eq!(std::fs::read_to_string(&board).unwrap(), before);
    }

    #[tokio::test]
    async fn flip_refuses_the_exact_open_board_without_touching_the_file() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("flip.kicad_pcb");
        let before = format!(
            "(kicad_pcb\n  (version 20260206)\n  (generator \"pcbnew\")\n  (net 0 \"\")\n{}\n)\n",
            FLIP_FOOTPRINT
                .lines()
                .map(|line| format!("  {line}"))
                .collect::<Vec<_>>()
                .join("\n")
        );
        std::fs::write(&board, &before).unwrap();
        let server =
            crate::tools::pcb_board::board_mock::spawn_kicad_holding_board(&board, |_| None);
        let ctx = crate::tools::pcb_board::board_mock::ctx_talking_to(server.address().to_string());

        let result = handle_flip_component(
            &json!({"board": board, "reference": "U1", "layer": "B.Cu"}),
            &ctx,
        )
        .await
        .unwrap();

        assert!(result.is_error);
        assert!(result_text(&result).contains("footprint flip"));
        assert_eq!(std::fs::read_to_string(&board).unwrap(), before);
    }

    #[tokio::test]
    async fn flip_proceeds_when_kicad_holds_a_different_board() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("flip.kicad_pcb");
        let other = tmp.path().join("other.kicad_pcb");
        let before = format!(
            "(kicad_pcb\n  (version 20260206)\n  (generator \"pcbnew\")\n  (net 0 \"\")\n{}\n)\n",
            FLIP_FOOTPRINT
                .lines()
                .map(|line| format!("  {line}"))
                .collect::<Vec<_>>()
                .join("\n")
        );
        std::fs::write(&board, &before).unwrap();
        std::fs::write(&other, "").unwrap();
        let server =
            crate::tools::pcb_board::board_mock::spawn_kicad_holding_board(&other, |_| None);
        let ctx = crate::tools::pcb_board::board_mock::ctx_talking_to(server.address().to_string());

        let result = handle_flip_component(
            &json!({"board": board, "reference": "U1", "layer": "B.Cu"}),
            &ctx,
        )
        .await
        .unwrap();

        assert!(!result.is_error, "{:?}", result.content);
        assert_ne!(std::fs::read_to_string(&board).unwrap(), before);
    }

    #[test]
    fn flip_refuses_custom_pad_geometry_instead_of_corrupting_it() {
        let custom = FLIP_FOOTPRINT.replace("roundrect (at 2 3 50)", "custom (at 2 3 50)");

        let error = flip_footprint_block(&custom).unwrap_err();

        assert!(error.to_string().contains("custom pads"));
    }

    #[test]
    fn flip_refuses_nested_drill_offsets_instead_of_mirroring_only_part_of_the_padstack() {
        let offset_drill = FLIP_FOOTPRINT.replace(
            "(roundrect_rratio 0.25)",
            "(roundrect_rratio 0.25)\n    (drill oval 0.4 0.8 (offset 0.2 0.1))",
        );

        let error = flip_footprint_block(&offset_drill).unwrap_err();

        assert!(error.to_string().contains("offset"), "{error}");
    }

    #[test]
    fn flip_does_not_mirror_text_on_a_non_side_specific_layer() {
        let user_text = FLIP_FOOTPRINT.replace(
            "(layer \"F.SilkS\")\n    (effects (font (size 1 1)) (justify left))",
            "(layer \"User.Drawings\")\n    (effects (font (size 1 1)) (justify left))",
        );

        let flipped = flip_footprint_block(&user_text).unwrap();

        assert!(flipped.contains("(layer \"User.Drawings\")"), "{flipped}");
        assert!(flipped.contains("(justify left)"), "{flipped}");
        assert!(!flipped.contains("(justify left mirror)"), "{flipped}");
    }

    #[test]
    fn supported_footprint_round_trip_restores_the_original_semantics() {
        let no_justify = FLIP_FOOTPRINT.replace(" (justify left)", "");
        let back = flip_footprint_block(&no_justify).unwrap();
        let front = flip_footprint_block(&back).unwrap();

        assert_eq!(
            konnect_sexp::parse_sexp(&front).unwrap(),
            konnect_sexp::parse_sexp(&no_justify).unwrap()
        );
    }

    #[test]
    fn non_cardinal_root_orientation_round_trips_without_drift() {
        let non_cardinal = FLIP_FOOTPRINT.replace("(at 10 20 30)", "(at 10 20 37.5)");

        let back = flip_footprint_block(&non_cardinal).unwrap();
        assert!(back.contains("(at 10 20 -37.5)"), "{back}");
        let front = flip_footprint_block(&back).unwrap();

        assert_eq!(
            konnect_sexp::parse_sexp(&front).unwrap(),
            konnect_sexp::parse_sexp(&non_cardinal).unwrap()
        );
    }

    #[test]
    fn through_hole_pad_layers_survive_a_flip_round_trip() {
        let through_hole = FLIP_FOOTPRINT.replace(
            "(pad \"1\" smd roundrect (at 2 3 50) (size 1 2)\n    \
             (layers \"F.Cu\" \"F.Paste\" \"F.Mask\")\n    (roundrect_rratio 0.25)",
            "(pad \"1\" thru_hole oval (at 2 3 50) (size 1 2)\n    \
             (drill oval 0.4 0.8)\n    (layers \"*.Cu\" \"*.Mask\")",
        );

        let back = flip_footprint_block(&through_hole).unwrap();
        assert!(back.contains("(layers \"*.Cu\" \"*.Mask\")"), "{back}");
        assert!(back.contains("(drill oval 0.4 0.8)"), "{back}");
        let front = flip_footprint_block(&back).unwrap();

        assert_eq!(
            konnect_sexp::parse_sexp(&front).unwrap(),
            konnect_sexp::parse_sexp(&through_hole).unwrap()
        );
    }

    #[test]
    fn hidden_property_stays_hidden_when_flipped() {
        let hidden = FLIP_FOOTPRINT.replace(
            "(effects (font (size 1 1)) (justify left))",
            "(effects (font (size 1 1)) (justify left))\n    (hide yes)",
        );

        let flipped = flip_footprint_block(&hidden).unwrap();

        assert!(flipped.contains("(hide yes)"), "{flipped}");
        assert!(flipped.contains("(justify left mirror)"), "{flipped}");
    }

    #[test]
    fn legacy_fp_text_reference_flips_and_round_trips() {
        let legacy = FLIP_FOOTPRINT.replace(
            "(property \"Reference\" \"U1\"",
            "(fp_text reference \"U1\"",
        );

        let back = flip_footprint_block(&legacy).unwrap();
        assert!(back.contains("(fp_text reference \"U1\""), "{back}");
        let front = flip_footprint_block(&back).unwrap();

        assert_eq!(
            konnect_sexp::parse_sexp(&front).unwrap(),
            konnect_sexp::parse_sexp(&legacy).unwrap()
        );
    }

    #[tokio::test]
    async fn invalid_flip_layer_is_structured_and_leaves_the_board_untouched() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("invalid-layer.kicad_pcb");
        let before = flip_board(&[FLIP_FOOTPRINT], "\n");
        std::fs::write(&board, &before).unwrap();

        let result = handle_flip_component(
            &json!({
                "board": board.to_string_lossy(),
                "reference": "U1",
                "layer": "In1.Cu",
            }),
            &test_ctx(),
        )
        .await
        .unwrap();

        assert!(result.is_error);
        assert_eq!(
            crate::mcp::error::extract_error_kind(&result).as_deref(),
            Some("invalid_argument")
        );
        assert_eq!(std::fs::read_to_string(board).unwrap(), before);
    }

    #[tokio::test]
    async fn missing_and_duplicate_flip_references_leave_the_board_untouched() {
        let tmp = tempfile::tempdir().unwrap();
        let missing_board = tmp.path().join("missing.kicad_pcb");
        let missing_before = flip_board(&[FLIP_FOOTPRINT], "\n");
        std::fs::write(&missing_board, &missing_before).unwrap();

        let missing = handle_flip_component(
            &json!({
                "board": missing_board.to_string_lossy(),
                "reference": "U404",
                "layer": "B.Cu",
            }),
            &test_ctx(),
        )
        .await
        .unwrap();
        assert!(missing.is_error);
        assert_eq!(
            std::fs::read_to_string(&missing_board).unwrap(),
            missing_before
        );

        let duplicate_board = tmp.path().join("duplicate.kicad_pcb");
        let duplicate_before = flip_board(&[FLIP_FOOTPRINT, FLIP_FOOTPRINT], "\n");
        std::fs::write(&duplicate_board, &duplicate_before).unwrap();
        let duplicate = handle_flip_component(
            &json!({
                "board": duplicate_board.to_string_lossy(),
                "reference": "U1",
                "layer": "B.Cu",
            }),
            &test_ctx(),
        )
        .await
        .unwrap();
        // A structured refusal naming the offending field, not a bubbled
        // `anyhow` the caller has to read prose out of.
        assert!(duplicate.is_error);
        let text = result_text(&duplicate);
        assert!(text.contains("invalid_argument"), "{text}");
        assert!(text.contains("\"field\":\"reference\""), "{text}");
        assert!(text.contains("more than once"), "{text}");
        assert_eq!(
            std::fs::read_to_string(duplicate_board).unwrap(),
            duplicate_before
        );
    }

    #[tokio::test]
    async fn closed_board_flip_preserves_crlf_line_endings() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("crlf.kicad_pcb");
        let before = flip_board(&[FLIP_FOOTPRINT], "\r\n");
        std::fs::write(&board, &before).unwrap();

        let result = handle_flip_component(
            &json!({
                "board": board.to_string_lossy(),
                "reference": "U1",
                "layer": "B.Cu",
            }),
            &test_ctx(),
        )
        .await
        .unwrap();
        assert!(!result.is_error, "{:?}", result.content);

        let written = std::fs::read_to_string(board).unwrap();
        assert_eq!(
            written
                .match_indices('\n')
                .filter(|(index, _)| *index == 0 || written.as_bytes()[index - 1] != b'\r')
                .count(),
            0,
            "{written:?}"
        );
    }

    #[test]
    fn stale_closed_board_flip_is_rejected_without_overwriting_newer_content() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("stale-flip.kicad_pcb");
        let expected = flip_board(&[FLIP_FOOTPRINT], "\n");
        let (replacement, changed) =
            prepare_closed_board_footprint_side(&expected, "U1", "B.Cu").unwrap();
        assert!(changed);
        let newer = expected.replace("(net 0 \"\")", "(net 0 \"\")\n  (net 1 \"GND\")");
        std::fs::write(&board, &newer).unwrap();

        let error = persist_board_replacement(&board, &expected, &replacement)
            .expect_err("a stale flip source must conflict");

        assert!(matches!(error, konnect_sexp::SexpError::Conflict { .. }));
        assert_eq!(std::fs::read_to_string(board).unwrap(), newer);
    }

    #[tokio::test]
    async fn unsupported_flip_geometry_returns_zero_writes() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("unsupported.kicad_pcb");
        let custom = FLIP_FOOTPRINT.replace("roundrect (at 2 3 50)", "custom (at 2 3 50)");
        let before = flip_board(&[&custom], "\n");
        std::fs::write(&board, &before).unwrap();

        let refusal = handle_flip_component(
            &json!({
                "board": board.to_string_lossy(),
                "reference": "U1",
                "layer": "B.Cu",
            }),
            &test_ctx(),
        )
        .await
        .unwrap();

        assert!(
            refusal.is_error,
            "unsupported pad geometry must fail closed"
        );
        let text = result_text(&refusal);
        assert!(text.contains("custom pads"), "{text}");
        assert!(text.contains("not modified"), "{text}");
        assert_eq!(std::fs::read_to_string(board).unwrap(), before);
    }

    #[test]
    fn stale_board_source_is_rejected_without_overwriting_newer_content() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("board.kicad_pcb");
        std::fs::write(&board, EMPTY_BOARD).unwrap();
        let replacement = EMPTY_BOARD.replace(
            "\n)\n",
            "\n\t(footprint \"Legacy:Socket\" (layer \"F.Cu\"))\n)\n",
        );
        let newer = EMPTY_BOARD.replace("(net 0 \"\")", "(net 0 \"\")\n\t(net 1 \"GND\")");
        std::fs::write(&board, &newer).unwrap();

        let error = persist_board_replacement(&board, EMPTY_BOARD, &replacement)
            .expect_err("a stale expected board must conflict");

        assert!(matches!(error, konnect_sexp::SexpError::Conflict { .. }));
        assert_eq!(std::fs::read_to_string(board).unwrap(), newer);
    }

    #[tokio::test]
    async fn stale_closed_board_move_is_rejected_without_overwriting_newer_content() {
        let tmp = tempfile::tempdir().unwrap();
        let board = placed_fallback_fixture(tmp.path()).await;
        let expected = std::fs::read_to_string(&board).unwrap();
        let replacement = prepare_closed_board_footprint_update(
            &expected,
            "R1",
            FootprintPlacementUpdate::Move { x: 40.0, y: 50.0 },
        )
        .unwrap();
        let newer = expected.replace("(net 0 \"\")", "(net 0 \"\")\n\t(net 1 \"GND\")");
        std::fs::write(&board, &newer).unwrap();

        let error = persist_board_replacement(&board, &expected, &replacement)
            .expect_err("a stale move source must conflict");

        assert!(matches!(error, konnect_sexp::SexpError::Conflict { .. }));
        assert_eq!(std::fs::read_to_string(board).unwrap(), newer);
    }

    #[tokio::test]
    async fn fallback_placement_rotation_reaches_the_pads() {
        // A rotated placement whose pads keep angle 0 trips KiCad's own
        // lib_footprint_mismatch check, so the rotation has to reach them.
        let tmp = tempfile::tempdir().unwrap();
        let board = fallback_fixture(tmp.path());
        let args = json!({
            "board": board.to_string_lossy(),
            "footprint": "Resistor_SMD:R_0805_2012Metric",
            "reference": "R1", "x": 10.0, "y": 20.0, "rotation": -90.0,
        });
        let res = handle_place_component(&args, &test_ctx()).await.unwrap();
        assert!(!res.is_error, "{:?}", res.content);

        let out = std::fs::read_to_string(&board).unwrap();
        assert!(out.contains("(at 10 20 -90)"), "footprint angle:\n{out}");
        assert!(out.contains("(at -0.9125 0 270)"), "pad angle:\n{out}");
        assert!(
            out.contains("(at 0 -1.65 90)"),
            "readable text angle:\n{out}"
        );
    }

    #[tokio::test]
    async fn a_truncated_board_is_refused_rather_than_rewritten() {
        // rfind(')') picks the insert point, so a board that is not one closed
        // (kicad_pcb …) form would silently gain a footprint outside the root
        // expression. Nothing should be written in that case.
        let tmp = tempfile::tempdir().unwrap();
        let board = fallback_fixture(tmp.path());
        let truncated = "(kicad_pcb (version 20241229) (generator \"test\")";
        std::fs::write(&board, truncated).unwrap();

        let args = json!({
            "board": board.to_string_lossy(),
            "footprint": "Resistor_SMD:R_0805_2012Metric",
            "reference": "R1", "x": 1.0, "y": 2.0,
        });
        let err = handle_place_component(&args, &test_ctx())
            .await
            .expect_err("a malformed board must not be written back");
        assert!(
            err.to_string().contains("balanced"),
            "error should explain why: {err}"
        );
        assert_eq!(
            std::fs::read_to_string(&board).unwrap(),
            truncated,
            "board must be left exactly as it was"
        );
    }

    /// Force LF, whatever the checkout did to this source file's literals.
    fn lf(s: &str) -> String {
        s.replace("\r\n", "\n")
    }

    /// Force CRLF, likewise.
    fn crlf(s: &str) -> String {
        lf(s).replace('\n', "\r\n")
    }

    #[tokio::test]
    async fn a_crlf_board_stays_crlf() {
        // KiCad writes these files CRLF on Windows, so placing into a CRLF
        // board must not leave two conventions in it.
        let tmp = tempfile::tempdir().unwrap();
        let board = fallback_fixture(tmp.path());
        std::fs::write(&board, crlf(EMPTY_BOARD)).unwrap();

        let args = json!({
            "board": board.to_string_lossy(),
            "footprint": "Resistor_SMD:R_0805_2012Metric",
            "reference": "R1", "x": 1.0, "y": 2.0,
        });
        let res = handle_place_component(&args, &test_ctx()).await.unwrap();
        assert!(!res.is_error, "handler errored: {:?}", res.content);

        let out = std::fs::read_to_string(&board).unwrap();
        assert!(
            out.contains("(pad \"1\" smd roundrect"),
            "footprint missing"
        );
        let bare_lf = out
            .match_indices('\n')
            .filter(|(i, _)| *i == 0 || out.as_bytes()[i - 1] != b'\r')
            .count();
        assert_eq!(
            bare_lf, 0,
            "a CRLF board gained {bare_lf} bare LF line endings:\n{out:?}"
        );
    }

    #[tokio::test]
    async fn an_lf_board_stays_lf() {
        // The reverse: a CRLF library footprint must not drag \r into an LF
        // board, which is the common case on Linux and macOS.
        let tmp = tempfile::tempdir().unwrap();
        let board = fallback_fixture(tmp.path());
        std::fs::write(&board, lf(EMPTY_BOARD)).unwrap();

        let args = json!({
            "board": board.to_string_lossy(),
            "footprint": "Resistor_SMD:R_0805_2012Metric",
            "reference": "R1", "x": 1.0, "y": 2.0,
        });
        let res = handle_place_component(&args, &test_ctx()).await.unwrap();
        assert!(!res.is_error, "handler errored: {:?}", res.content);

        let out = std::fs::read_to_string(&board).unwrap();
        assert!(
            out.contains("(pad \"1\" smd roundrect"),
            "footprint missing"
        );
        assert!(
            !out.contains('\r'),
            "a CRLF library footprint dragged \\r into an LF board:\n{out:?}"
        );
    }

    /// A rep0 endpoint that completes every round-trip with an error status —
    /// a live KiCAD saying no. Placement must fail closed: error out, and
    /// leave the board file alone.
    fn spawn_rejecting_kicad() -> crate::test_support::MockIpcServer {
        crate::test_support::MockIpcServer::spawn("rejecting-component", |_| {
            konnect_ipc::gen::kiapi::common::ApiResponse {
                status: Some(konnect_ipc::gen::kiapi::common::ApiResponseStatus {
                    status: konnect_ipc::gen::kiapi::common::ApiStatusCode::AsBadRequest as i32,
                    error_message: "mock rejects everything".to_string(),
                }),
                header: None,
                message: None,
            }
        })
    }

    #[tokio::test]
    async fn a_reachable_kicad_that_rejects_never_touches_the_file() {
        let tmp = tempfile::tempdir().unwrap();
        let board = fallback_fixture(tmp.path());
        let board_before = std::fs::read_to_string(&board).unwrap();

        let server = spawn_rejecting_kicad();
        let ctx = ToolContext::new(
            crate::tools::ServerConfig {
                kicad_cli: String::new(),
                kicad_binary: String::new(),
                ipc_address: server.address().to_string(),
                project_dir: None,
                jlcpcb_db_path: None,
                auto_load_toolsets: false,
                eager_toolsets: false,
            },
            std::sync::Arc::new(crate::router::ToolRouter::new()),
        );
        let args = json!({
            "board": board.to_string_lossy(),
            "footprint": "Resistor_SMD:R_0805_2012Metric",
            "reference": "R1", "x": 1.0, "y": 2.0,
        });
        let res = handle_place_component(&args, &ctx).await.unwrap();
        assert!(res.is_error, "a rejection must not be reported as success");
        let text = result_text(&res);
        assert!(
            text.contains("rejected the placement") && text.contains("not modified"),
            "the error must say the file was left alone: {text}"
        );
        assert_eq!(
            std::fs::read_to_string(&board).unwrap(),
            board_before,
            "a reachable KiCAD that says no must never trigger the file fallback"
        );
    }

    // ─── Reading pads: live board first, file second ──────────────────────────

    /// A board file whose R1 is stale — the state of the last save.
    const SAVED_BOARD_WITH_R1: &str = "(kicad_pcb\n\
        \t(version 20260206)\n\
        \t(footprint \"R_0805\"\n\
        \t\t(at 5 5 90)\n\
        \t\t(property \"Reference\" \"R1\" (at 0 -1 0) (layer \"F.SilkS\"))\n\
        \t\t(pad \"1\" smd roundrect (at -0.9 0 105) (size 2.432 1.524)\n\
        \t\t\t(roundrect_rratio 0.25) (layers \"F.Cu\" \"F.Paste\" \"F.Mask\")\n\
        \t\t\t(net \"SAVED\") (uuid \"11111111-1111-1111-1111-111111111111\"))\n\
        \t)\n\
        )\n";

    fn live_pad(number: &str, x: f64, y: f64, net: &str) -> prost_types::Any {
        konnect_ipc::builders::pack_any(
            &konnect_ipc::gen::kiapi::board::types::Pad {
                id: Some(konnect_ipc::gen::kiapi::common::types::Kiid {
                    value: "22222222-2222-2222-2222-222222222222".to_string(),
                }),
                number: number.to_string(),
                r#type: konnect_ipc::gen::kiapi::board::types::PadType::PtSmd as i32,
                position: Some(konnect_ipc::builders::vec2(x, y)),
                net: Some(konnect_ipc::gen::kiapi::board::types::Net {
                    code: None,
                    name: net.to_string(),
                }),
                pad_stack: Some(konnect_ipc::gen::kiapi::board::types::PadStack {
                    layers: vec![
                        konnect_ipc::gen::kiapi::board::types::BoardLayer::BlFCu as i32,
                        konnect_ipc::gen::kiapi::board::types::BoardLayer::BlFPaste as i32,
                        konnect_ipc::gen::kiapi::board::types::BoardLayer::BlFMask as i32,
                    ],
                    copper_layers: vec![konnect_ipc::gen::kiapi::board::types::PadStackLayer {
                        layer: konnect_ipc::gen::kiapi::board::types::BoardLayer::BlFCu as i32,
                        shape: konnect_ipc::gen::kiapi::board::types::PadStackShape::PssRoundrect
                            as i32,
                        size: Some(konnect_ipc::builders::vec2(2.432, 1.524)),
                        corner_rounding_ratio: 0.25,
                        ..Default::default()
                    }],
                    angle: Some(konnect_ipc::gen::kiapi::common::types::Angle {
                        value_degrees: 105.0,
                    }),
                    // KiCad reports zone settings on every pad, inherited
                    // ones included, and always the spoke angle.
                    zone_settings: Some(konnect_ipc::gen::kiapi::board::types::ZoneConnectionSettings {
                        zone_connection:
                            konnect_ipc::gen::kiapi::board::types::ZoneConnectionStyle::ZcsInherited
                                as i32,
                        thermal_spokes: Some(
                            konnect_ipc::gen::kiapi::board::types::ThermalSpokeSettings {
                                angle: Some(konnect_ipc::gen::kiapi::common::types::Angle {
                                    value_degrees: 90.0,
                                }),
                                ..Default::default()
                            },
                        ),
                    }),
                    ..Default::default()
                }),
                ..Default::default()
            },
            "kiapi.board.types.Pad",
        )
    }

    fn live_custom_stack_pad(number: &str, x: f64, y: f64) -> prost_types::Any {
        use konnect_ipc::gen::kiapi::board::types::{
            BoardLayer, DrillProperties, DrillShape, Pad, PadStack, PadStackLayer, PadStackShape,
            PadType,
        };

        let layer = |layer, shape, width, height| PadStackLayer {
            layer,
            shape,
            size: Some(konnect_ipc::builders::vec2(width, height)),
            ..Default::default()
        };
        konnect_ipc::builders::pack_any(
            &Pad {
                number: number.to_string(),
                r#type: PadType::PtPth as i32,
                position: Some(konnect_ipc::builders::vec2(x, y)),
                pad_stack: Some(PadStack {
                    layers: vec![
                        BoardLayer::BlFCu as i32,
                        BoardLayer::BlIn1Cu as i32,
                        BoardLayer::BlBCu as i32,
                    ],
                    copper_layers: vec![
                        layer(
                            BoardLayer::BlFCu as i32,
                            PadStackShape::PssCustom as i32,
                            3.0,
                            2.0,
                        ),
                        layer(
                            BoardLayer::BlIn1Cu as i32,
                            PadStackShape::PssCircle as i32,
                            1.5,
                            1.5,
                        ),
                        layer(
                            BoardLayer::BlBCu as i32,
                            PadStackShape::PssRectangle as i32,
                            2.0,
                            1.0,
                        ),
                    ],
                    drill: Some(DrillProperties {
                        start_layer: BoardLayer::BlFCu as i32,
                        end_layer: BoardLayer::BlBCu as i32,
                        diameter: Some(konnect_ipc::builders::vec2(0.8, 1.2)),
                        shape: DrillShape::DsOblong as i32,
                        ..Default::default()
                    }),
                    angle: Some(konnect_ipc::gen::kiapi::common::types::Angle {
                        value_degrees: 37.0,
                    }),
                    ..Default::default()
                }),
                ..Default::default()
            },
            "kiapi.board.types.Pad",
        )
    }

    fn live_footprint(reference: &str, pads: Vec<prost_types::Any>) -> prost_types::Any {
        use konnect_ipc::gen::kiapi;
        konnect_ipc::builders::pack_any(
            &kiapi::board::types::FootprintInstance {
                position: Some(konnect_ipc::builders::vec2(100.0, 100.0)),
                reference_field: Some(kiapi::board::types::Field {
                    name: "Reference".to_string(),
                    text: Some(kiapi::board::types::BoardText {
                        text: Some(kiapi::common::types::Text {
                            text: reference.to_string(),
                            ..Default::default()
                        }),
                        ..Default::default()
                    }),
                    ..Default::default()
                }),
                definition: Some(kiapi::board::types::Footprint {
                    items: pads,
                    ..Default::default()
                }),
                ..Default::default()
            },
            "kiapi.board.types.FootprintInstance",
        )
    }

    /// A rep0 endpoint playing a KiCad that holds `board` open with `items` on
    /// it — a live board carrying edits the file on disk has never seen.
    fn spawn_kicad_holding(
        board: &Path,
        items: Vec<prost_types::Any>,
    ) -> crate::test_support::MockIpcServer {
        use konnect_ipc::gen::kiapi;

        let board = board.to_string_lossy().to_string();
        crate::test_support::MockIpcServer::spawn("component-items", move |request| {
            let command = request.message.expect("a command");
            let body = if command.type_url.ends_with("GetOpenDocuments") {
                Some(konnect_ipc::builders::pack_any(
                    &kiapi::common::commands::GetOpenDocumentsResponse {
                        documents: vec![kiapi::common::types::DocumentSpecifier {
                            r#type: kiapi::common::types::DocumentType::DoctypePcb as i32,
                            project: None,
                            identifier: Some(
                                kiapi::common::types::document_specifier::Identifier::BoardFilename(
                                    board.clone(),
                                ),
                            ),
                        }],
                    },
                    "kiapi.common.commands.GetOpenDocumentsResponse",
                ))
            } else if command.type_url.ends_with("GetItems") {
                Some(konnect_ipc::builders::pack_any(
                    &kiapi::common::commands::GetItemsResponse {
                        header: None,
                        status: kiapi::common::types::ItemRequestStatus::IrsOk as i32,
                        items: items.clone(),
                    },
                    "kiapi.common.commands.GetItemsResponse",
                ))
            } else {
                None
            };
            kiapi::common::ApiResponse {
                status: Some(kiapi::common::ApiResponseStatus {
                    status: kiapi::common::ApiStatusCode::AsOk as i32,
                    error_message: String::new(),
                }),
                header: None,
                message: body,
            }
        })
    }

    fn ctx_talking_to(address: String) -> ToolContext {
        ToolContext::new(
            crate::tools::ServerConfig {
                kicad_cli: String::new(),
                kicad_binary: String::new(),
                ipc_address: address,
                project_dir: None,
                jlcpcb_db_path: None,
                auto_load_toolsets: false,
                eager_toolsets: false,
            },
            std::sync::Arc::new(crate::router::ToolRouter::new()),
        )
    }

    fn parsed(res: &CallToolResult) -> serde_json::Value {
        serde_json::from_str(&result_text(res)).expect("json result")
    }

    /// The compact live answers: every physical pad with a requested number
    /// (two share "1" here), the footprint's live pose, fabrication properties
    /// unknown rather than "none", and the list filter's unknown/duplicate split.
    #[tokio::test]
    async fn compact_live_pads_and_filtered_list_report_what_kicad_holds() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("b.kicad_pcb");
        std::fs::write(&board, SAVED_BOARD_WITH_R1).unwrap();
        let server = spawn_kicad_holding(
            &board,
            vec![
                live_footprint(
                    "R1",
                    vec![
                        live_pad("1", 101.0, 66.0, "/VBUS"),
                        live_pad("1", 102.0, 66.0, "/VBUS"),
                        live_pad("2", 103.0, 66.0, "GND"),
                    ],
                ),
                live_footprint("R2", vec![live_pad("1", 1.0, 1.0, "GND")]),
                live_footprint("R2", vec![live_pad("1", 2.0, 2.0, "GND")]),
            ],
        );
        let ctx = ctx_talking_to(server.address().to_string());
        let res = handle_get_component_pads(
            &json!({ "board": board.to_string_lossy(), "reference": "R1",
                     "detail": "connectivity", "pad_numbers": ["1", "9"] }),
            &ctx,
        )
        .await
        .unwrap();
        assert!(!res.is_error, "{}", result_text(&res));
        let body = parsed(&res);
        assert_eq!(body["source"], "ipc");
        assert_eq!(body["total_pad_count"], 3);
        assert_eq!(body["matched_pad_count"], 2);
        assert_eq!(body["unmatched_pad_numbers"], json!(["9"]));
        assert_eq!(body["pads"][1]["x"], 102.0);
        assert!(body["pads"][0]["fabrication_property"].is_null());
        assert_eq!(body["pad_fabrication_properties"]["observable"], false);
        assert!(body["pads"][0].get("copper_layers").is_none());
        assert_eq!(body["footprint"]["x"], 100.0);
        assert_eq!(body["footprint"]["source"], "ipc");

        // An explicit saved read answers from the file even while KiCad holds it.
        let saved = parsed(
            &handle_get_component_pads(
                &json!({ "board": board.to_string_lossy(), "reference": "R1",
                         "detail": "connectivity", "source": "saved" }),
                &ctx,
            )
            .await
            .unwrap(),
        );
        assert_eq!(saved["source"], "file");
        assert_eq!(saved["pads"][0]["net"], "SAVED");
        assert_eq!(saved["footprint"]["rotation"], 90.0);

        let list = parsed(
            &handle_get_component_list(
                &json!({ "board": board.to_string_lossy(), "references": ["R1", "R2", "R9"] }),
                &ctx,
            )
            .await
            .unwrap(),
        );
        assert_eq!(list["source"], "ipc");
        assert_eq!(list["count"], 3);
        assert_eq!(list["unknown_references"], json!(["R9"]));
        assert_eq!(list["duplicate_references"], json!(["R2"]));
    }

    #[tokio::test]
    async fn pads_come_from_the_board_kicad_holds_not_the_last_save() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("b.kicad_pcb");
        std::fs::write(&board, SAVED_BOARD_WITH_R1).unwrap();
        let server = spawn_kicad_holding(
            &board,
            vec![live_footprint(
                "R1",
                vec![live_pad("1", 101.155, 66.11, "/VBUS")],
            )],
        );

        let res = handle_get_component_pads(
            &json!({ "board": board.to_string_lossy(), "reference": "R1" }),
            &ctx_talking_to(server.address().to_string()),
        )
        .await
        .unwrap();

        assert!(!res.is_error, "{:?}", res.content);
        let body = parsed(&res);
        assert_eq!(body["source"], json!("ipc"));
        assert_eq!(body["pads"][0]["net"], json!("/VBUS"));
        assert_eq!(body["pads"][0]["x"], json!(101.155));
        assert_eq!(
            body["pads"][0]["uuid"],
            "22222222-2222-2222-2222-222222222222"
        );
        assert_eq!(body["pads"][0]["pad_type"], "smd");
        assert_eq!(body["pads"][0]["shape"], "roundrect");
        assert_eq!(body["pads"][0]["size"], json!({"x": 2.432, "y": 1.524}));
        assert_eq!(body["pads"][0]["rotation_deg"], 105.0);
        assert_eq!(body["pads"][0]["copper_layers"][0]["layer"], "F.Cu");
        assert_eq!(
            body["pads"][0]["layers"],
            json!(["F.Cu", "F.Paste", "F.Mask"])
        );
    }

    #[tokio::test]
    async fn a_part_deleted_in_kicad_is_not_answered_from_the_file() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("b.kicad_pcb");
        std::fs::write(&board, SAVED_BOARD_WITH_R1).unwrap();
        let server = spawn_kicad_holding(&board, vec![]);

        let res = handle_get_component_pads(
            &json!({ "board": board.to_string_lossy(), "reference": "R1" }),
            &ctx_talking_to(server.address().to_string()),
        )
        .await
        .unwrap();

        assert!(res.is_error, "the live board no longer has R1");
        assert!(result_text(&res).contains("open in KiCad"));
    }

    #[tokio::test]
    async fn no_pads_from_kicad_is_refused_when_the_file_has_some() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("b.kicad_pcb");
        std::fs::write(&board, SAVED_BOARD_WITH_R1).unwrap();
        // The part is there, its pads are not — the shape a response we
        // failed to read would also take.
        let server = spawn_kicad_holding(&board, vec![live_footprint("R1", vec![])]);

        let res = handle_get_component_pads(
            &json!({ "board": board.to_string_lossy(), "reference": "R1" }),
            &ctx_talking_to(server.address().to_string()),
        )
        .await
        .unwrap();

        assert!(res.is_error, "'no pads' must not pass as an answer here");
        assert!(result_text(&res).contains("no pads"));
    }

    #[tokio::test]
    async fn a_genuinely_pad_less_footprint_reads_as_zero() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("b.kicad_pcb");
        std::fs::write(&board, SAVED_BOARD_WITH_R1).unwrap();
        // The saved file has no LOGO1 to disagree, so KiCad's answer stands.
        let server = spawn_kicad_holding(&board, vec![live_footprint("LOGO1", vec![])]);

        let res = handle_get_component_pads(
            &json!({ "board": board.to_string_lossy(), "reference": "LOGO1" }),
            &ctx_talking_to(server.address().to_string()),
        )
        .await
        .unwrap();

        assert!(!res.is_error, "{:?}", res.content);
        let body = parsed(&res);
        assert_eq!(body["source"], json!("ipc"));
        assert_eq!(body["pad_count"], json!(0));
    }

    #[tokio::test]
    async fn pads_fall_back_to_the_file_when_kicad_is_unreachable() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("b.kicad_pcb");
        std::fs::write(&board, SAVED_BOARD_WITH_R1).unwrap();

        let res = handle_get_component_pads(
            &json!({ "board": board.to_string_lossy(), "reference": "R1" }),
            &test_ctx(),
        )
        .await
        .unwrap();

        assert!(!res.is_error, "{:?}", res.content);
        let body = parsed(&res);
        assert_eq!(body["source"], json!("file"));
        assert_eq!(body["pads"][0]["net"], json!("SAVED"));
        assert_eq!(
            body["pads"][0]["uuid"],
            "11111111-1111-1111-1111-111111111111"
        );
        assert_eq!(body["pads"][0]["pad_type"], "smd");
        assert_eq!(body["pads"][0]["shape"], "roundrect");
        assert_eq!(body["pads"][0]["size"], json!({"x": 2.432, "y": 1.524}));
        assert_eq!(body["pads"][0]["rotation_deg"], 105.0);
        assert_eq!(body["pads"][0]["copper_layers"][0]["layer"], "F.Cu");
        assert_eq!(
            body["pads"][0]["layers"],
            json!(["F.Cu", "F.Paste", "F.Mask"])
        );
    }

    #[tokio::test]
    async fn live_and_file_pad_geometry_use_the_same_response_shape() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("b.kicad_pcb");
        std::fs::write(&board, SAVED_BOARD_WITH_R1).unwrap();
        let server = spawn_kicad_holding(
            &board,
            vec![live_footprint("R1", vec![live_pad("1", 5.0, 5.9, "SAVED")])],
        );
        let args = json!({ "board": board.to_string_lossy(), "reference": "R1" });

        let live = handle_get_component_pads(&args, &ctx_talking_to(server.address().to_string()))
            .await
            .unwrap();
        let file = handle_get_component_pads(&args, &test_ctx()).await.unwrap();
        let live = parsed(&live);
        let file = parsed(&file);
        let live_keys: std::collections::BTreeSet<_> = live["pads"][0]
            .as_object()
            .unwrap()
            .keys()
            .cloned()
            .collect();
        let file_keys: std::collections::BTreeSet<_> = file["pads"][0]
            .as_object()
            .unwrap()
            .keys()
            .cloned()
            .collect();

        assert_eq!(live_keys, file_keys);
    }

    #[tokio::test]
    async fn live_custom_front_inner_back_stack_keeps_each_copper_geometry() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("b.kicad_pcb");
        std::fs::write(&board, SAVED_BOARD_WITH_R1).unwrap();
        let server = spawn_kicad_holding(
            &board,
            vec![live_footprint(
                "R1",
                vec![live_custom_stack_pad("1", 12.0, 34.0)],
            )],
        );

        let result = handle_get_component_pads(
            &json!({ "board": board.to_string_lossy(), "reference": "R1" }),
            &ctx_talking_to(server.address().to_string()),
        )
        .await
        .unwrap();

        assert!(!result.is_error, "{:?}", result.content);
        let body = parsed(&result);
        assert_eq!(body["pads"][0]["shape"], "custom");
        assert_eq!(body["pads"][0]["rotation_deg"], 37.0);
        assert_eq!(body["pads"][0]["drill"]["shape"], "oval");
        assert_eq!(
            body["pads"][0]["copper_layers"].as_array().unwrap().len(),
            3
        );
        assert_eq!(body["pads"][0]["copper_layers"][0]["layer"], "F.Cu");
        assert_eq!(body["pads"][0]["copper_layers"][0]["shape"], "custom");
        assert_eq!(body["pads"][0]["copper_layers"][1]["layer"], "In1.Cu");
        assert_eq!(body["pads"][0]["copper_layers"][1]["shape"], "circle");
        assert_eq!(body["pads"][0]["copper_layers"][2]["layer"], "B.Cu");
        assert_eq!(body["pads"][0]["copper_layers"][2]["shape"], "rect");
    }

    // ─── Custom-shape pads through the typed placement path ────────────────

    const TEXAS_CUSTOM_PADS: &str =
        include_str!("../../tests/fixtures/custom_pads_texas_rje0020a_kicad10.kicad_mod");
    const GENERIC_CUSTOM_PADS: &str =
        include_str!("../../tests/fixtures/custom_pads_vqfn20_kicad10.kicad_mod");

    use crate::test_support::without_heatsink_property as carried;

    /// The exposed pad's fabrication property has no field in KiCad 10's pad
    /// message, so placing the stock footprints would put a pad on the board
    /// that is not the library's. The shared parser refuses it by name before
    /// anything is built.
    #[test]
    fn byte_exact_custom_pad_fixtures_refuse_the_heatsink_property_by_name() {
        for source in [TEXAS_CUSTOM_PADS, GENERIC_CUSTOM_PADS] {
            let error = format!("{:#}", extract_pad_definitions(source).unwrap_err());
            assert!(
                error.contains("pad '21'") && error.contains("pad_prop_heatsink"),
                "{error}"
            );
        }
    }

    /// `(zone_connect 2)` on the fixtures' exposed pad is read, built into the
    /// pad stack's own zone settings (not the per-layer field the protocol
    /// reserves) and every other pad, which leaves it out, stays inherited:
    /// no zone settings are sent for it.
    #[test]
    fn fixture_zone_connect_reaches_the_pad_stack_and_absence_stays_inherited() {
        use konnect_ipc::gen::kiapi::board::types::{PadStackShape, ZoneConnectionStyle};
        for source in [TEXAS_CUSTOM_PADS, GENERIC_CUSTOM_PADS] {
            let pads = extract_pad_definitions(&carried(source)).unwrap();
            for pad in &pads {
                let want = (pad.number == "21").then_some(konnect_ipc::IpcPadZoneConnection::Solid);
                assert_eq!(pad.zone_connection, want, "pad '{}'", pad.number);
            }
            let built = built_pads(&build_at(&pads, 10.0, 20.0, 37.0).unwrap());
            for pad in &built {
                let stack = pad.pad_stack.as_ref().unwrap();
                assert!(stack
                    .copper_layers
                    .iter()
                    .all(|layer| layer.zone_settings.is_none()));
                // Every file pad goes out with the spoke angle KiCad's parser
                // gives it (none of these write one): 45° for a custom pad
                // on a circle anchor in a 20260206 file, 90° for a rounded
                // or plain rectangle. No width or gap override is invented.
                let settings = stack.zone_settings.as_ref().unwrap();
                assert_eq!(
                    settings.zone_connection,
                    if pad.number == "21" {
                        ZoneConnectionStyle::ZcsFull
                    } else {
                        ZoneConnectionStyle::ZcsInherited
                    } as i32,
                    "pad '{}'",
                    pad.number
                );
                let spokes = settings.thermal_spokes.unwrap();
                assert_eq!((spokes.width, spokes.gap), (None, None));
                let copper = &stack.copper_layers[0];
                let want_angle = if copper.shape == PadStackShape::PssCustom as i32 {
                    assert_eq!(copper.custom_anchor_shape, PadStackShape::PssCircle as i32);
                    45.0
                } else {
                    assert!(
                        [PadStackShape::PssRoundrect, PadStackShape::PssRectangle]
                            .map(|shape| shape as i32)
                            .contains(&copper.shape),
                        "pad '{}'",
                        pad.number
                    );
                    90.0
                };
                assert_eq!(
                    spokes.angle.map(|angle| angle.value_degrees),
                    Some(want_angle),
                    "pad '{}'",
                    pad.number
                );
            }
        }
    }

    fn thermal_footprint(version: &str, pad: &str) -> String {
        format!("(footprint \"X\" {version} (layer \"F.Cu\") {pad})")
    }

    fn thermal_roundrect(clauses: &str) -> String {
        format!(
            "(pad \"1\" smd roundrect (at 0 0) (size 1 1) (layers \"F.Cu\" \"F.Mask\") \
             (roundrect_rratio 0.25) {clauses} (uuid \"u\"))"
        )
    }

    fn thermal_custom(anchor: &str, clauses: &str) -> String {
        format!(
            "(pad \"2\" smd custom (at 0 0) (size 0.5 0.5) (layers \"F.Cu\" \"F.Mask\") \
             {clauses} (options (clearance outline) (anchor {anchor})) (primitives (gr_poly \
             (pts (xy -1 -1) (xy 1 -1) (xy 1 1)) (width 0) (fill yes))) (uuid \"v\"))"
        )
    }

    const KICAD10_FORMAT: &str = "(version 20260206)";

    /// The spokes one built pad carries.
    fn built_spokes(
        pads: &[konnect_ipc::IpcPadDefinition],
    ) -> konnect_ipc::gen::kiapi::board::types::ZoneConnectionSettings {
        built_pads(&build_at(pads, 10.0, 20.0, 37.0).unwrap())[0]
            .pad_stack
            .as_ref()
            .unwrap()
            .zone_settings
            .unwrap()
    }

    /// A pad's thermal clauses travel exactly as KiCad's parser holds them:
    /// an explicit zero stays zero, an unwritten width or gap stays unset,
    /// and the angle is the written one or the parser's shape default
    /// (`PCB_IO_KICAD_SEXPR_PARSER::parsePAD`), which also depends on the
    /// file format for a circle-anchored custom pad.
    #[test]
    fn pad_thermal_clauses_travel_exactly_and_absence_stays_unset() {
        use konnect_ipc::gen::kiapi::board::types::ZoneConnectionStyle;
        use konnect_ipc::gen::kiapi::common::types::Distance;

        let source = thermal_footprint(
            KICAD10_FORMAT,
            &thermal_roundrect(
                "(thermal_bridge_width 0) (thermal_gap 0.3) (thermal_bridge_angle 30)",
            ),
        );
        let pads = extract_pad_definitions(&source).unwrap();
        assert_eq!(
            pads[0].thermal,
            konnect_ipc::IpcPadThermal {
                spoke_width: Some(0.0),
                gap: Some(0.3),
                spoke_angle: Some(30.0),
            }
        );
        let settings = built_spokes(&pads);
        assert_eq!(
            settings.zone_connection,
            ZoneConnectionStyle::ZcsInherited as i32
        );
        let spokes = settings.thermal_spokes.unwrap();
        assert_eq!(spokes.width, Some(Distance { value_nm: 0 }));
        assert_eq!(spokes.gap, Some(Distance { value_nm: 300_000 }));
        assert_eq!(spokes.angle.map(|angle| angle.value_degrees), Some(30.0));

        // KiCad's legacy spelling of the width.
        let legacy = thermal_footprint(KICAD10_FORMAT, &thermal_roundrect("(thermal_width 0.25)"));
        assert_eq!(
            extract_pad_definitions(&legacy).unwrap()[0]
                .thermal
                .spoke_width,
            Some(0.25)
        );

        let circle = "(pad \"3\" thru_hole circle (at 0 0) (size 2 2) (drill 1) \
                      (layers \"*.Cu\" \"*.Mask\") (uuid \"w\"))";
        for (pad, version, angle) in [
            (thermal_roundrect(""), KICAD10_FORMAT, 90.0),
            (circle.to_string(), KICAD10_FORMAT, 45.0),
            (thermal_custom("circle", ""), KICAD10_FORMAT, 45.0),
            (thermal_custom("rect", ""), KICAD10_FORMAT, 90.0),
            (thermal_custom("circle", ""), "(version 20211014)", 90.0),
            (thermal_custom("circle", ""), "(version 20211015)", 45.0),
            (thermal_custom("circle", ""), "", 90.0),
        ] {
            let pads = extract_pad_definitions(&thermal_footprint(version, &pad)).unwrap();
            let thermal = pads[0].thermal;
            assert_eq!(
                (thermal.spoke_width, thermal.gap, thermal.spoke_angle),
                (None, None, Some(angle)),
                "{version} {pad}"
            );
            let spokes = built_spokes(&pads).thermal_spokes.unwrap();
            assert_eq!((spokes.width, spokes.gap), (None, None), "{pad}");
            assert_eq!(
                spokes.angle.map(|angle| angle.value_degrees),
                Some(angle),
                "{version} {pad}"
            );
        }

        // An explicit angle equal to the default is still the written one.
        let written = thermal_footprint(
            KICAD10_FORMAT,
            &thermal_custom("rect", "(thermal_bridge_angle 45)"),
        );
        assert_eq!(
            extract_pad_definitions(&written).unwrap()[0]
                .thermal
                .spoke_angle,
            Some(45.0)
        );
    }

    /// The file readback reports the thermal settings KiCad's parser holds,
    /// and says whether the angle was written or defaulted.
    #[test]
    fn file_pad_readback_reports_thermal_settings_and_their_source() {
        let readback = |source: &str| {
            let root = konnect_sexp::parse_sexp(source).unwrap();
            let pad = root.find("pad").unwrap();
            serde_json::to_value(
                pad_info_from_sexp(
                    pad,
                    0.0,
                    0.0,
                    0.0,
                    String::new(),
                    sexp_format_version(&root),
                )
                .unwrap(),
            )
            .unwrap()
        };
        let written = readback(&thermal_footprint(
            KICAD10_FORMAT,
            &thermal_roundrect(
                "(thermal_bridge_width 0) (thermal_gap 0.3) (thermal_bridge_angle 30)",
            ),
        ));
        assert_eq!(
            written["thermal"],
            json!({ "spoke_width": 0.0, "gap": 0.3, "spoke_angle": 30.0 })
        );
        assert_eq!(written["thermal_spoke_angle_explicit"], json!(true));

        let defaulted = readback(&thermal_footprint(
            KICAD10_FORMAT,
            &thermal_custom("rect", ""),
        ));
        assert_eq!(defaulted["thermal"], json!({ "spoke_angle": 90.0 }));
        assert_eq!(defaulted["thermal_spoke_angle_explicit"], json!(false));
    }

    /// Every malformed, repeated, non-finite or uncarriable thermal clause is
    /// refused by name before anything is built.
    #[test]
    fn malformed_or_uncarriable_thermal_clauses_are_refused_by_name() {
        for (clauses, named) in [
            (
                "(thermal_gap 0.3) (thermal_gap 0.3)",
                "2 (thermal_gap …) clauses",
            ),
            (
                "(thermal_width 0.2) (thermal_bridge_width 0.2)",
                "thermal_bridge_width",
            ),
            (
                "(thermal_bridge_angle 30) (thermal_bridge_angle 30)",
                "thermal_bridge_angle",
            ),
            ("(thermal_gap 0.3 0.4)", "exactly one bare number"),
            ("(thermal_gap (x 1))", "exactly one bare number"),
            ("(thermal_gap \"0.3\")", "exactly one bare number"),
            ("(thermal_gap)", "exactly one bare number"),
            ("(thermal_gap abc)", "not a number"),
            ("(thermal_bridge_angle nan)", "not finite"),
            ("(thermal_bridge_angle inf)", "not finite"),
            ("(thermal_bridge_width -0.1)", "negative"),
            ("(thermal_gap 5000)", "board-unit range"),
        ] {
            let source = thermal_footprint(KICAD10_FORMAT, &thermal_roundrect(clauses));
            let error = format!("{:#}", extract_pad_definitions(&source).unwrap_err());
            assert!(
                error.contains(named) && error.contains("pad '1'"),
                "{clauses}: {error}"
            );
        }
        // The other uncarried clauses are still refused beside them.
        let source = thermal_footprint(
            KICAD10_FORMAT,
            &thermal_roundrect("(thermal_gap 0.3) (property pad_prop_heatsink)"),
        );
        let error = format!("{:#}", extract_pad_definitions(&source).unwrap_err());
        assert!(error.contains("pad_prop_heatsink"), "{error}");
        let source = thermal_footprint(
            KICAD10_FORMAT,
            &thermal_custom("circle", "(thermal_bridge_angle 45)")
                .replace("(clearance outline)", "(clearance convexhull)"),
        );
        let error = format!("{:#}", extract_pad_definitions(&source).unwrap_err());
        assert!(error.contains("convexhull"), "{error}");
    }

    /// Each value KiCad writes, from the KiCad-written zone-connect fixtures,
    /// maps to its protocol style; malformed and undefined values are refused,
    /// not read as inherited.
    #[test]
    fn pad_zone_connect_maps_every_kicad_value_and_refuses_the_rest() {
        use konnect_ipc::gen::kiapi::board::types::ZoneConnectionStyle as Style;
        use konnect_ipc::IpcPadZoneConnection as Zone;
        let fixtures = [
            (
                include_str!("../../tests/fixtures/pad_zone_connect_kicad10.kicad_mod"),
                [Some(Zone::Solid); 3],
            ),
            (
                include_str!("../../tests/fixtures/pad_zone_connect_thermal_kicad10.kicad_mod"),
                [Some(Zone::Thermal); 3],
            ),
            (
                include_str!("../../tests/fixtures/pad_zone_connect_inherited_kicad10.kicad_mod"),
                [None; 3],
            ),
            (
                include_str!("../../tests/fixtures/pad_zone_connect_first_none_kicad10.kicad_mod"),
                [Some(Zone::None), None, None],
            ),
        ];
        for (source, want) in fixtures {
            let pads = extract_pad_definitions(source).unwrap();
            assert_eq!(
                pads.iter()
                    .map(|pad| pad.zone_connection)
                    .collect::<Vec<_>>(),
                want
            );
            // The file reader inspection shares reports the same, with
            // absence spelled out as inherited.
            let root = konnect_sexp::parse_sexp(source).unwrap();
            for (pad, want) in root.find_all("pad").into_iter().zip(want) {
                let info = pad_info_from_sexp(pad, 0.0, 0.0, 0.0, String::new(), None).unwrap();
                assert_eq!(info.zone_connection, Some(want.unwrap_or(Zone::Inherited)));
            }
        }

        for (value, zone, style) in [
            (-1, Zone::Inherited, Style::ZcsInherited),
            (0, Zone::None, Style::ZcsNone),
            (1, Zone::Thermal, Style::ZcsThermal),
            (2, Zone::Solid, Style::ZcsFull),
            (3, Zone::PthThermal, Style::ZcsPthThermal),
        ] {
            assert_eq!(Zone::from_file_value(value), Some(zone));
            assert_eq!(zone.file_value(), value);
            assert_eq!(zone.to_proto(), style);
            assert_eq!(Zone::from_proto(style as i32), Some(zone));
        }
        assert_eq!(Zone::from_proto(Style::ZcsUnknown as i32), None);

        let base = include_str!("../../tests/fixtures/pad_zone_connect_kicad10.kicad_mod");
        for bad in [
            "(zone_connect 4)",
            "(zone_connect -2)",
            "(zone_connect solid)",
            "(zone_connect \"2\")",
            "(zone_connect 2.0)",
            "(zone_connect)",
            "(zone_connect 2 2)",
            "(zone_connect 2) (zone_connect 2)",
        ] {
            let source = base.replacen("(zone_connect 2)", bad, 1);
            let error = format!("{:#}", extract_pad_definitions(&source).unwrap_err());
            assert!(error.contains("zone connection"), "{bad}: {error}");
            let root = konnect_sexp::parse_sexp(&source).unwrap();
            let pad = root.find("pad").unwrap();
            assert!(
                pad_info_from_sexp(pad, 0.0, 0.0, 0.0, String::new(), None).is_err(),
                "{bad} must not be inspected as a connection"
            );
        }
    }

    /// Pad clauses the typed path has no field for are refused by name, while
    /// the ones it carries or that are not pad semantics are accepted.
    #[test]
    fn pad_clauses_the_typed_path_cannot_carry_are_refused() {
        let pad = |clauses: &str| {
            format!(
                "(footprint \"X\" (layer \"F.Cu\") (pad \"1\" smd roundrect (at 0 0) \
                 (size 1 1) (layers \"F.Cu\" \"F.Mask\") (roundrect_rratio 0.25) {clauses} \
                 (uuid \"u\")))"
            )
        };
        assert!(extract_pad_definitions(&pad("(zone_connect 1) (pinfunction \"A\")")).is_ok());
        assert!(extract_pad_definitions(&pad("(remove_unused_layers no)")).is_ok());
        for (clause, named) in [
            ("(property pad_prop_bga)", "pad_prop_bga"),
            ("(solder_mask_margin 0.05)", "solder_mask_margin"),
            ("(solder_paste_margin -0.02)", "solder_paste_margin"),
            ("(clearance 0.2)", "clearance"),
            ("(chamfer_ratio 0.2)", "chamfer_ratio"),
            ("(remove_unused_layers yes)", "remove_unused_layers yes"),
            ("(options (anchor rect))", "options"),
            ("locked", "locked"),
        ] {
            let error = format!("{:#}", extract_pad_definitions(&pad(clause)).unwrap_err());
            assert!(error.contains(named), "{clause}: {error}");
        }
        let offset = "(footprint \"X\" (pad \"1\" thru_hole circle (at 0 0) (size 2 2) \
                      (drill 1 (offset 0.1 0)) (layers \"*.Cu\")))";
        let error = format!("{:#}", extract_pad_definitions(offset).unwrap_err());
        assert!(error.contains("drill clause 'offset'"), "{error}");
    }

    /// The pads of a built `FootprintInstance`, decoded.
    fn built_pads(item: &prost_types::Any) -> Vec<konnect_ipc::gen::kiapi::board::types::Pad> {
        use konnect_ipc::gen::kiapi;
        let footprint = kiapi::board::types::FootprintInstance::decode(item.value.as_slice())
            .expect("a FootprintInstance");
        footprint
            .definition
            .expect("a definition")
            .items
            .iter()
            .filter(|child| konnect_ipc::builders::any_is(child, "kiapi.board.types.Pad"))
            .map(|child| kiapi::board::types::Pad::decode(child.value.as_slice()).unwrap())
            .collect()
    }

    fn build_at(
        pads: &[konnect_ipc::IpcPadDefinition],
        x: f64,
        y: f64,
        rotation: f64,
    ) -> anyhow::Result<prost_types::Any> {
        KiCadIpcClient::build_footprint_item(
            "Lib:Part",
            "U1",
            "part",
            pads,
            &[],
            &Default::default(),
            x,
            y,
            rotation,
            "F.Cu",
        )
    }

    /// The polygon of a pad's single primitive, in nm, as sent.
    fn polygon_nm(
        shape: &konnect_ipc::gen::kiapi::board::types::BoardGraphicShape,
    ) -> Vec<(i64, i64)> {
        use konnect_ipc::gen::kiapi::common::types::{graphic_shape::Geometry, poly_line_node};
        let Some(Geometry::Polygon(set)) = shape.shape.as_ref().unwrap().geometry.as_ref() else {
            panic!("expected a polygon primitive, got {shape:?}");
        };
        assert_eq!(set.polygons.len(), 1);
        assert!(set.polygons[0].holes.is_empty());
        set.polygons[0]
            .outline
            .as_ref()
            .unwrap()
            .nodes
            .iter()
            .map(|node| match node.geometry.as_ref().unwrap() {
                poly_line_node::Geometry::Point(p) => (p.x_nm, p.y_nm),
                other => panic!("unexpected arc node {other:?}"),
            })
            .collect()
    }

    /// Both stock KiCad 10 fixtures, which #657 refused, now parse with every
    /// pad kept: eight custom corner pads each, their primitives exact.
    #[test]
    fn stock_custom_pad_footprints_parse_with_every_pad_and_primitive() {
        for (source, paste_only) in [(TEXAS_CUSTOM_PADS, 1), (GENERIC_CUSTOM_PADS, 4)] {
            let pads = extract_pad_definitions(&carried(source)).unwrap();
            assert_eq!(pads.len(), 21 + paste_only, "every pad, numbered or not");
            let custom: Vec<_> = pads.iter().filter(|pad| pad.shape == "custom").collect();
            assert_eq!(
                custom
                    .iter()
                    .map(|pad| pad.number.as_str())
                    .collect::<Vec<_>>(),
                ["1", "5", "6", "10", "11", "15", "16", "20"]
            );
            for pad in &custom {
                let shape = pad.custom.as_ref().expect("custom pads carry primitives");
                assert_eq!(shape.anchor, konnect_ipc::IpcCustomPadAnchor::Circle);
                assert_eq!(shape.primitives.len(), 1);
                assert!(matches!(
                    &shape.primitives[0],
                    konnect_ipc::IpcPadPrimitive::Polygon { points, filled: true, .. }
                        if points.len() == 5
                ));
                assert_eq!(pad.layers, ["F.Cu", "F.Mask", "F.Paste"]);
            }
            assert!(pads
                .iter()
                .filter(|pad| pad.shape != "custom")
                .all(|pad| pad.custom.is_none()));
            let paste: Vec<_> = pads.iter().filter(|pad| pad.number.is_empty()).collect();
            assert_eq!(paste.len(), paste_only);
            assert!(paste.iter().all(|pad| pad.layers == ["F.Paste"]));
        }

        let texas = extract_pad_definitions(&carried(TEXAS_CUSTOM_PADS)).unwrap();
        let pad1 = texas.iter().find(|pad| pad.number == "1").unwrap();
        assert_eq!(
            pad1.custom.as_ref().unwrap().primitives[0],
            konnect_ipc::IpcPadPrimitive::Polygon {
                points: vec![
                    (-0.4275, -0.09),
                    (0.253358, -0.09),
                    (0.4275, 0.084142),
                    (0.4275, 0.09),
                    (-0.4275, 0.09),
                ],
                width: 0.02,
                filled: true,
            }
        );
    }

    /// The real fixture through the actual `FootprintInstance` builder, placed
    /// translated and rotated: the pad moves and turns with the footprint, the
    /// primitive stays exactly pad-local (KiCad applies the pad transform to
    /// it, so applying it here too would move the copper twice), and the
    /// paste-only pads keep their single layer.
    #[test]
    fn stock_custom_pads_survive_the_typed_builder_pad_local() {
        use konnect_ipc::gen::kiapi::board::types::{BoardLayer, PadStackShape};
        let pads = extract_pad_definitions(&carried(TEXAS_CUSTOM_PADS)).unwrap();
        let item = build_at(&pads, 100.0, 50.0, 90.0).unwrap();
        let built = built_pads(&item);
        assert_eq!(built.len(), pads.len(), "no pad is dropped");

        let pad1 = built.iter().find(|pad| pad.number == "1").unwrap();
        let (bx, by) = konnect_sexp::geometry::transform_pad(-1.4375, -0.9, 100.0, 50.0, 90.0);
        let position = pad1.position.unwrap();
        assert_eq!(
            (position.x_nm, position.y_nm),
            (
                konnect_ipc::builders::mm_to_nm(bx),
                konnect_ipc::builders::mm_to_nm(by)
            )
        );
        let stack = pad1.pad_stack.as_ref().unwrap();
        assert_eq!(stack.angle.as_ref().unwrap().value_degrees, 90.0);
        assert_eq!(
            stack.layers,
            [
                BoardLayer::BlFCu as i32,
                BoardLayer::BlFMask as i32,
                BoardLayer::BlFPaste as i32
            ]
            .into_iter()
            .collect::<std::collections::BTreeSet<_>>()
            .into_iter()
            .collect::<Vec<_>>()
        );
        let [copper] = stack.copper_layers.as_slice() else {
            panic!("one copper entry for a normal stack");
        };
        assert_eq!(copper.layer, BoardLayer::BlFCu as i32);
        assert_eq!(copper.shape, PadStackShape::PssCustom as i32);
        assert_eq!(copper.custom_anchor_shape, PadStackShape::PssCircle as i32);
        assert_eq!(copper.size.unwrap().x_nm, 72_721);
        assert_eq!(copper.custom_shapes.len(), 1);
        assert_eq!(
            polygon_nm(&copper.custom_shapes[0]),
            [
                (-427_500, -90_000),
                (253_358, -90_000),
                (427_500, 84_142),
                (427_500, 90_000),
                (-427_500, 90_000)
            ],
            "pad-local and exact to the nanometre"
        );
        let attributes = copper.custom_shapes[0]
            .shape
            .as_ref()
            .unwrap()
            .attributes
            .as_ref()
            .unwrap();
        assert_eq!(
            attributes.stroke.as_ref().unwrap().width.unwrap().value_nm,
            20_000
        );
        assert_eq!(
            attributes.fill.as_ref().unwrap().fill_type,
            konnect_ipc::gen::kiapi::common::types::GraphicFillType::GftFilled as i32
        );

        // Another placement changes the pad, never its primitives.
        let elsewhere = built_pads(&build_at(&pads, -7.0, 3.0, 215.0).unwrap());
        let moved = elsewhere.iter().find(|pad| pad.number == "1").unwrap();
        assert_ne!(moved.position, pad1.position);
        assert_eq!(
            moved.pad_stack.as_ref().unwrap().copper_layers[0].custom_shapes,
            copper.custom_shapes
        );

        let paste: Vec<_> = built.iter().filter(|pad| pad.number.is_empty()).collect();
        assert_eq!(paste.len(), 1);
        assert_eq!(
            paste[0].pad_stack.as_ref().unwrap().layers,
            [BoardLayer::BlFPaste as i32]
        );

        // Ordinary pads are built as before: no primitives, anchor = shape.
        let pad2 = built.iter().find(|pad| pad.number == "2").unwrap();
        let ordinary = &pad2.pad_stack.as_ref().unwrap().copper_layers[0];
        assert_eq!(ordinary.shape, PadStackShape::PssRoundrect as i32);
        assert_eq!(
            ordinary.custom_anchor_shape,
            PadStackShape::PssRoundrect as i32
        );
        assert!(ordinary.custom_shapes.is_empty());
    }

    /// One custom pad of every supported primitive type, on a rect anchor, a
    /// rotated pad and a rotated, translated footprint.
    const MIXED_CUSTOM_PAD: &str = r#"(footprint "Mixed"
	(layer "F.Cu")
	(pad "7" smd custom
		(at 1.5 -2 30)
		(size 0.4 0.3)
		(layers "F.Cu" "F.Mask")
		(options
			(clearance outline)
			(anchor rect)
		)
		(primitives
			(gr_poly (pts (xy 0 0) (xy 1 0) (xy 1 0.5)) (width 0) (fill yes))
			(gr_rect (start -0.5 -0.25) (end 0.25 0.5) (stroke (width 0.05) (type solid)) (fill no))
			(gr_circle (center 0.1 0.2) (end 0.4 0.2) (width 0.03) (fill yes))
			(gr_line (start -1 0) (end -0.2 0.3) (width 0.1))
			(gr_arc (start 0 -0.6) (mid 0.3 -0.3) (end 0 0) (width 0.08))
		)
	)
	(pad "" smd custom
		(at 0 0)
		(size 0.2 0.2)
		(layers "F.Paste")
		(options (anchor circle))
		(primitives (gr_poly (pts (xy -0.3 -0.3) (xy 0.3 -0.3) (xy 0.3 0.3) (xy -0.3 0.3)) (width 0) (fill yes)))
	)
)"#;

    #[test]
    fn every_supported_primitive_is_carried_pad_local_on_a_rotated_pad() {
        use konnect_ipc::gen::kiapi::board::types::{BoardLayer, PadStackShape};
        use konnect_ipc::IpcPadPrimitive as P;

        let pads = extract_pad_definitions(MIXED_CUSTOM_PAD).unwrap();
        let expected = vec![
            P::Polygon {
                points: vec![(0.0, 0.0), (1.0, 0.0), (1.0, 0.5)],
                width: 0.0,
                filled: true,
            },
            P::Rect {
                start: (-0.5, -0.25),
                end: (0.25, 0.5),
                width: 0.05,
                filled: false,
            },
            P::Circle {
                center: (0.1, 0.2),
                end: (0.4, 0.2),
                width: 0.03,
                filled: true,
            },
            P::Segment {
                start: (-1.0, 0.0),
                end: (-0.2, 0.3),
                width: 0.1,
            },
            P::Arc {
                start: (0.0, -0.6),
                mid: (0.3, -0.3),
                end: (0.0, 0.0),
                width: 0.08,
            },
        ];
        let custom = pads[0].custom.as_ref().unwrap();
        assert_eq!(custom.anchor, konnect_ipc::IpcCustomPadAnchor::Rect);
        assert_eq!(custom.primitives, expected);

        let built = built_pads(&build_at(&pads, 10.0, 20.0, -45.0).unwrap());
        let stack = built[0].pad_stack.as_ref().unwrap();
        assert_eq!(stack.angle.as_ref().unwrap().value_degrees, -15.0);
        let (bx, by) = konnect_sexp::geometry::transform_pad(1.5, -2.0, 10.0, 20.0, -45.0);
        assert_eq!(
            built[0].position.unwrap(),
            konnect_ipc::builders::vec2(bx, by)
        );
        let copper = &stack.copper_layers[0];
        assert_eq!(
            copper.custom_anchor_shape,
            PadStackShape::PssRectangle as i32
        );
        // Read back with the inverse reader: every primitive unchanged.
        let round_trip = copper
            .custom_shapes
            .iter()
            .map(|shape| konnect_ipc::builders::pad_primitive_from_proto(shape).unwrap())
            .collect::<Vec<_>>();
        assert_eq!(round_trip, expected);

        // A paste-only custom pad keeps exactly its one layer and its copper.
        let paste = built[1].pad_stack.as_ref().unwrap();
        assert_eq!(paste.layers, [BoardLayer::BlFPaste as i32]);
        assert_eq!(paste.copper_layers[0].custom_shapes.len(), 1);
    }

    /// Everything the typed path cannot carry exactly is refused with a
    /// reason naming the pad and what to change, never approximated.
    #[test]
    fn unrepresentable_custom_pads_are_refused_by_name() {
        let pad = |options: &str, primitives: &str| {
            format!(
                "(footprint \"X\" (layer \"F.Cu\") (pad \"3\" smd custom (at 0 0) (size 0.2 0.2) \
                 (layers \"F.Cu\" \"F.Mask\") {options} (primitives {primitives})))"
            )
        };
        let ok_options = "(options (clearance outline) (anchor circle))";
        let square = "(gr_poly (pts (xy 0 0) (xy 1 0) (xy 1 1) (xy 0 1)) (width 0) (fill yes))";
        let cases = [
            (
                pad("(options (clearance convexhull) (anchor circle))", square),
                "clearance convexhull",
            ),
            (pad("", square), "no (options"),
            (pad("(options (clearance outline))", square), "no anchor"),
            (pad("(options (anchor oval))", square), "anchor 'oval'"),
            (pad("(options (anchor rect) (thermal 1))", square), "option 'thermal'"),
            (
                pad(
                    ok_options,
                    "(gr_poly (pts (xy 0 0) (arc (start 1 0) (mid 1.2 0.5) (end 1 1)) (xy 0 1)) (width 0) (fill yes))",
                ),
                "curved (arc",
            ),
            (
                pad(ok_options, "(gr_curve (pts (xy 0 0) (xy 1 0) (xy 1 1) (xy 0 1)) (width 0.1))"),
                "bezier",
            ),
            (
                pad(ok_options, "(gr_bbox (start 0 0) (end 1 1))"),
                "annotation",
            ),
            (pad(ok_options, "(gr_blob (start 0 0))"), "primitive type 'gr_blob'"),
            (
                pad(ok_options, "(gr_poly (pts (xy 0 0) (xy 1 0) (xy 1 1)) (fill yes))"),
                "no width",
            ),
            (
                pad(ok_options, "(gr_poly (pts (xy 0 0) (xy 1 0) (xy 1 1)) (width 0))"),
                "no (fill",
            ),
            (
                pad(ok_options, "(gr_poly (pts (xy 0 0) (xy 1 0) (xy NaN 1)) (width 0) (fill yes))"),
                "not finite",
            ),
            (
                pad(ok_options, "(gr_poly (pts (xy 0 0) (xy 1 0) (xy 2 0)) (width 0) (fill yes))"),
                "no area",
            ),
            (
                pad(ok_options, "(gr_poly (pts (xy 0 0) (xy 1 0)) (width 0) (fill yes))"),
                "at least 3",
            ),
            (
                pad(ok_options, "(gr_circle (center 0 0) (end 0 0) (width 0) (fill yes))"),
                "zero radius",
            ),
            (
                pad(ok_options, "(gr_line (start 0 0) (end 0 0) (width 0.1))"),
                "zero length",
            ),
            (
                pad(ok_options, "(gr_arc (start 0 0) (mid 1 0) (end 2 0) (width 0.1))"),
                "collinear",
            ),
            (
                pad(ok_options, "(gr_rect (start 0 0) (end 1 1) (radius 0.1) (width 0) (fill yes))"),
                "clause 'radius'",
            ),
            (
                pad(ok_options, "(gr_line (start 0 0) (end 1 0) (stroke (width 0.1) (type dash)))"),
                "stroke type 'dash'",
            ),
            (pad(ok_options, &format!("{square} (width 0.1)")), "list-level (width"),
            (pad(ok_options, ""), "at least one primitive"),
            (
                pad(ok_options, "(gr_poly (pts (xy 0 0) (xy 1 0) (xy 1 1)) (width -0.1) (fill yes))"),
                "non-negative",
            ),
        ];
        for (source, expected) in cases {
            let error = format!("{:#}", extract_pad_definitions(&source).unwrap_err());
            assert!(
                error.contains("custom pad '3'") && error.contains(expected),
                "expected '{expected}' in: {error}"
            );
        }
        // The control: the same pad with ordinary options is accepted.
        assert!(extract_pad_definitions(&pad(ok_options, square)).is_ok());
        // `(clearance outline)` is KiCad's default, so it may be omitted.
        assert!(extract_pad_definitions(&pad("(options (anchor rect))", square)).is_ok());
    }

    /// The live readback reports a custom pad's anchor and pad-local
    /// primitives, so Konnect can verify what KiCad holds; ordinary pads gain
    /// no new fields.
    #[tokio::test]
    async fn live_custom_pad_readback_reports_anchor_and_primitives() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("b.kicad_pcb");
        std::fs::write(&board, SAVED_BOARD_WITH_R1).unwrap();
        let pads = extract_pad_definitions(&carried(TEXAS_CUSTOM_PADS)).unwrap();
        let built = build_at(&pads, 100.0, 100.0, 90.0).unwrap();
        let children = konnect_ipc::gen::kiapi::board::types::FootprintInstance::decode(
            built.value.as_slice(),
        )
        .unwrap()
        .definition
        .unwrap()
        .items
        .into_iter()
        .filter(|child| konnect_ipc::builders::any_is(child, "kiapi.board.types.Pad"))
        .collect();
        let server = spawn_kicad_holding(&board, vec![live_footprint("R1", children)]);

        let result = handle_get_component_pads(
            &json!({ "board": board.to_string_lossy(), "reference": "R1" }),
            &ctx_talking_to(server.address().to_string()),
        )
        .await
        .unwrap();

        assert!(!result.is_error, "{:?}", result.content);
        let body = parsed(&result);
        let pads = body["pads"].as_array().unwrap();
        let pad1 = pads.iter().find(|pad| pad["number"] == "1").unwrap();
        let copper = &pad1["copper_layers"][0];
        assert_eq!(copper["shape"], "custom");
        assert_eq!(copper["custom_anchor_shape"], "circle");
        assert_eq!(
            copper["custom_primitives"],
            json!([{
                "kind": "polygon",
                "points": [[-0.4275, -0.09], [0.253358, -0.09], [0.4275, 0.084142], [0.4275, 0.09], [-0.4275, 0.09]],
                "width": 0.02,
                "filled": true
            }])
        );
        let pad2 = pads.iter().find(|pad| pad["number"] == "2").unwrap();
        let ordinary = pad2["copper_layers"][0].as_object().unwrap();
        assert!(!ordinary.contains_key("custom_primitives"));
        assert!(!ordinary.contains_key("custom_anchor_shape"));
    }

    #[tokio::test]
    async fn saved_back_side_footprint_keeps_board_space_rotation_and_back_layers() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("back.kicad_pcb");
        std::fs::write(
            &board,
            "(kicad_pcb\n\
             \t(footprint \"BackSide\"\n\
             \t\t(layer \"B.Cu\")\n\
             \t\t(at 20 30 45)\n\
             \t\t(property \"Reference\" \"U1\")\n\
             \t\t(pad \"1\" smd oval (at 2 -1 225) (size 2 1)\n\
             \t\t\t(layers \"B.Cu\" \"B.Paste\" \"B.Mask\"))\n\
             \t)\n\
             )\n",
        )
        .unwrap();

        let result = handle_get_component_pads(
            &json!({ "board": board.to_string_lossy(), "reference": "U1" }),
            &test_ctx(),
        )
        .await
        .unwrap();

        assert!(!result.is_error, "{:?}", result.content);
        let body = parsed(&result);
        assert_eq!(body["pads"][0]["rotation_deg"], 225.0);
        assert_eq!(body["pads"][0]["shape"], "oval");
        assert_eq!(
            body["pads"][0]["layers"],
            json!(["B.Cu", "B.Paste", "B.Mask"])
        );
        assert_eq!(body["pads"][0]["copper_layers"][0]["layer"], "B.Cu");
    }

    #[tokio::test]
    async fn kicad_authored_board_reports_the_saved_effective_pad_rotation() {
        let board =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/drc_ownership_j1.kicad_pcb");

        let result = handle_get_component_pads(
            &json!({ "board": board.to_string_lossy(), "reference": "J1" }),
            &test_ctx(),
        )
        .await
        .unwrap();

        assert!(!result.is_error, "{:?}", result.content);
        let body = parsed(&result);
        assert_eq!(body["source"], "file");
        assert_eq!(body["pad_count"], 4);
        assert_eq!(body["pads"][0]["shape"], "rect");
        assert_eq!(body["pads"][0]["size"], json!({"x": 1.78, "y": 1.02}));
        assert_eq!(body["pads"][0]["rotation_deg"], 90.0);
        assert_eq!(body["pads"][0]["net"], "GND");
    }

    #[tokio::test]
    async fn file_pad_reads_refuse_to_silently_drop_an_unreadable_pad() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("b.kicad_pcb");
        std::fs::write(
            &board,
            "(kicad_pcb\n\
             \t(footprint \"Malformed\"\n\
             \t\t(at 5 5 0)\n\
             \t\t(property \"Reference\" \"R1\")\n\
             \t\t(pad \"1\" smd rect (size 1 1) (layers \"F.Cu\"))\n\
             \t)\n\
             )\n",
        )
        .unwrap();

        let result = handle_get_component_pads(
            &json!({ "board": board.to_string_lossy(), "reference": "R1" }),
            &test_ctx(),
        )
        .await
        .unwrap();

        assert!(result.is_error);
        assert!(result_text(&result).contains("Pad '1' on footprint 'R1' has no readable position"));
    }

    #[tokio::test]
    async fn pad_reads_do_not_fall_back_when_reachable_kicad_rejects() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("b.kicad_pcb");
        std::fs::write(&board, SAVED_BOARD_WITH_R1).unwrap();

        let server = spawn_rejecting_kicad();
        let res = handle_get_component_pads(
            &json!({ "board": board.to_string_lossy(), "reference": "R1" }),
            &ctx_talking_to(server.address().to_string()),
        )
        .await
        .unwrap();

        assert!(
            res.is_error,
            "a live rejection must not return stale file pads"
        );
        assert!(result_text(&res).contains("mock rejects everything"));
    }

    #[tokio::test]
    async fn a_pad_position_carries_the_source_of_the_reading() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("b.kicad_pcb");
        std::fs::write(&board, SAVED_BOARD_WITH_R1).unwrap();
        let server = spawn_kicad_holding(
            &board,
            vec![live_footprint(
                "R1",
                vec![live_pad("1", 101.155, 66.11, "/VBUS")],
            )],
        );

        let res = handle_get_pad_position(
            &json!({ "board": board.to_string_lossy(), "reference": "R1", "pad_number": "1" }),
            &ctx_talking_to(server.address().to_string()),
        )
        .await
        .unwrap();

        let body = parsed(&res);
        assert_eq!(body["source"], json!("ipc"));
        assert_eq!(body["x"], json!(101.155));
    }

    // ─── board_lib_id / helpers (ported from PR #66) ──────────────────────────

    /// `board_lib_id` for a path, with the library file's declared name.
    fn id_for(path: &str, declared: &str) -> String {
        board_lib_id(path, Path::new(path), declared)
    }

    #[test]
    fn board_lib_id_never_yields_a_filesystem_path() {
        // A Library:Footprint id is already what the board wants.
        assert_eq!(
            board_lib_id("Resistor_SMD:R_0805", Path::new("/ignored"), "R_0805"),
            "Resistor_SMD:R_0805"
        );
        // A path in a .pretty library takes the nickname from its directory.
        assert_eq!(
            id_for(
                "/nonexistent/kicad/footprints/Resistor_SMD.pretty/R_0805.kicad_mod",
                "R_0805"
            ),
            "Resistor_SMD:R_0805"
        );
        // Loose file: no nickname to recover, so it keeps the name the library
        // file declares — unlinked, but a valid name rather than a path.
        assert_eq!(
            id_for("/nonexistent/scratch/R_0805.kicad_mod", "R_0805_2012Metric"),
            "R_0805_2012Metric"
        );
    }

    #[test]
    fn a_path_like_declared_name_falls_back_to_the_file_stem() {
        // A malformed library file naming itself with a path must not smuggle
        // that path into the board through the fallback branch.
        assert_eq!(
            id_for(
                "/nonexistent/scratch/R_0805.kicad_mod",
                "/tmp/other/R.kicad_mod"
            ),
            "R_0805"
        );
        assert_eq!(
            id_for("/nonexistent/scratch/R_0805.kicad_mod", r"C:\x\R.kicad_mod"),
            "R_0805"
        );
        // An empty declared name is no better than a path.
        assert_eq!(
            id_for("/nonexistent/scratch/R_0805.kicad_mod", ""),
            "R_0805"
        );
    }

    #[test]
    fn pretty_suffix_matching_ignores_case() {
        // Windows and macOS filesystems are case-insensitive, so Foo.Pretty and
        // Foo.pretty are the same directory to KiCad.
        assert_eq!(
            pretty_dir_nickname(Path::new("/libs/Resistor_SMD.Pretty")),
            Some("Resistor_SMD".into())
        );
        assert_eq!(
            pretty_dir_nickname(Path::new("/libs/Resistor_SMD.pretty")),
            Some("Resistor_SMD".into())
        );
        // A bare ".pretty" leaves no nickname behind.
        assert_eq!(pretty_dir_nickname(Path::new("/libs/.pretty")), None);
        assert_eq!(pretty_dir_nickname(Path::new("/libs/plain")), None);
    }

    #[test]
    fn a_board_edit_must_stay_one_kicad_pcb_form() {
        assert!(check_single_board_form("(kicad_pcb (version 20241229))").is_ok());
        assert!(check_single_board_form("\n  (kicad_pcb (version 1))\n\n").is_ok());

        // Truncated — the bug this guard exists for.
        assert!(check_single_board_form("(kicad_pcb (version 1)").is_err());
        // Leading garbage would otherwise be skipped by find_balanced_block.
        assert!(check_single_board_form("garbage(kicad_pcb (version 1))").is_err());
        // A second form after the root is not one board.
        assert!(check_single_board_form("(kicad_pcb (version 1))(extra)").is_err());
        // Well-formed, but not a board.
        assert!(check_single_board_form("(not_a_board (version 1))").is_err());
    }

    #[test]
    fn pad_angles_absorb_the_footprint_rotation() {
        // KiCad stores each pad's absolute orientation: a footprint placed at
        // -90 carries 270 on its pads, while pad positions stay in unrotated
        // footprint-local coordinates.
        let out = apply_rotation_to_children(&library_footprint(), -90.0);
        assert!(out.contains("(at -0.9125 0 270)"), "{out}");
        // Position is unchanged; only the angle was added.
        assert!(
            !out.contains("(at 0 -0.9125"),
            "pad position must not rotate"
        );
    }

    #[test]
    fn text_angles_are_kept_readable_in_file_fallback() {
        // A -90 footprint would put text at 270, which reads upside down, so
        // KiCad flips it by 180 to 90 — matching what pcbnew writes.
        let out = apply_rotation_to_children(&library_footprint(), -90.0);
        assert!(out.contains("(at 0 -1.65 90)"), "reference text:\n{out}");
        assert!(out.contains("(at 0 1.65 90)"), "value text:\n{out}");
    }

    #[test]
    fn zero_rotation_is_written_without_an_angle() {
        assert_eq!(format_at(1.5, -2.0, 0.0), "(at 1.5 -2)");
        assert_eq!(format_at(0.0, 0.0, 90.0), "(at 0 0 90)");
    }

    #[test]
    fn rotate_at_block_rejects_non_positional_at() {
        assert!(rotate_at_block("(at)", 90.0, false).is_none());
        assert!(rotate_at_block("(atomic 1 2)", 90.0, false).is_none());
        assert!(rotate_at_block("(at 1 2 3 4)", 90.0, false).is_none());
    }

    #[test]
    fn indent_block_reimposes_one_line_ending() {
        // A CRLF library footprint going into an LF board and the reverse:
        // whichever the destination uses is what comes out.
        assert_eq!(indent_block("a\r\nb", "\t", "\n"), "\ta\n\tb");
        assert_eq!(indent_block("a\nb", "\t", "\r\n"), "\ta\r\n\tb");
    }

    #[test]
    fn sexp_strings_are_escaped_and_quoted() {
        // Input characters:  a " b \ c
        let input = ['a', '"', 'b', '\\', 'c'].iter().collect::<String>();
        let expected = ['"', 'a', '\\', '"', 'b', '\\', '\\', 'c', '"']
            .iter()
            .collect::<String>();
        assert_eq!(quote_sexp_string(&input), expected);
        assert_eq!(quote_sexp_string("plain"), "\"plain\"");
    }

    #[test]
    fn name_span_covers_the_quoted_header_name() {
        let content = library_footprint();
        let span = footprint_name_span(&content).expect("header not found");
        assert_eq!(&content[span], "\"R_0805_2012Metric\"");
    }

    #[test]
    fn reference_substitution_targets_the_reference_property_only() {
        let mut out = library_footprint();
        replace_reference(&mut out, "R42").unwrap();
        assert!(out.contains("(property \"Reference\" \"R42\""), "{out}");
        assert!(
            out.contains("(property \"Value\" \"R_0805_2012Metric\""),
            "Value must be untouched:\n{out}"
        );
        assert!(!out.contains("REF**"));
    }

    #[tokio::test]
    async fn place_component_rejects_back_copper_and_creates_nothing() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("b.kicad_pcb");
        let board_content = "(kicad_pcb\n\t(version 20240108)\n\t(generator \"pcbnew\")\n)\n";
        std::fs::write(&board, board_content).unwrap();

        let args = json!({
            "board": board.to_string_lossy(),
            "footprint": "Resistor_SMD:R_0402",
            "reference": "R1",
            "x": 10.0, "y": 20.0,
            "layer": "B.Cu",
        });
        let res = handle_place_component(&args, &test_ctx()).await.unwrap();
        assert!(res.is_error, "B.Cu placement must be refused");
        assert_eq!(
            crate::mcp::error::extract_error_kind(&res).as_deref(),
            Some("invalid_argument"),
            "rejection should be a structured invalid_argument error"
        );
        let text = result_text(&res);
        assert!(
            text.contains("back-side placement is not yet supported"),
            "must say why: {text}"
        );
        assert!(
            text.contains("F.Cu") && text.contains("flip"),
            "must suggest the workaround: {text}"
        );
        // Rejection happens before any resolution, IPC round-trip, or file
        // write — the board is untouched and no IPC error ever surfaced.
        assert!(
            !text.contains("socket path not configured"),
            "the handler must not have reached the IPC layer: {text}"
        );
        assert_eq!(
            std::fs::read_to_string(&board).unwrap(),
            board_content,
            "board file must be left untouched"
        );
    }

    /// A saved read reports each pad's fabrication property, "none" where the
    /// saved pad has none; a live read reports it as unknown, never as none.
    #[tokio::test]
    async fn saved_pads_report_fabrication_properties_and_live_pads_say_unobservable() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("b.kicad_pcb");
        std::fs::write(
            &board,
            SAVED_BOARD_WITH_R1.replace(
                "(net \"SAVED\")",
                "(property pad_prop_heatsink) (net \"SAVED\")",
            ),
        )
        .unwrap();
        let server = spawn_kicad_holding(
            &board,
            vec![live_footprint("R1", vec![live_pad("1", 5.0, 5.9, "SAVED")])],
        );
        let ctx = ctx_talking_to(server.address().to_string());

        let saved = parsed(
            &handle_get_component_pads(
                &json!({ "board": board.to_string_lossy(), "reference": "R1", "source": "saved" }),
                &ctx,
            )
            .await
            .unwrap(),
        );
        assert_eq!(
            saved["source"], "file",
            "saved is read even while KiCad holds the board"
        );
        assert_eq!(
            saved["pads"][0]["fabrication_property"],
            "pad_prop_heatsink"
        );
        assert_eq!(saved["pad_fabrication_properties"]["observable"], true);

        let live = parsed(
            &handle_get_component_pads(
                &json!({ "board": board.to_string_lossy(), "reference": "R1" }),
                &ctx,
            )
            .await
            .unwrap(),
        );
        assert_eq!(live["source"], "ipc");
        assert_eq!(
            live["pads"][0]["fabrication_property"],
            serde_json::Value::Null
        );
        assert_eq!(
            live["pad_fabrication_properties"]["status"],
            "unobservable_via_live_typed_api"
        );

        std::fs::write(&board, SAVED_BOARD_WITH_R1).unwrap();
        let plain = parsed(
            &handle_get_component_pads(
                &json!({ "board": board.to_string_lossy(), "reference": "R1", "source": "saved" }),
                &ctx,
            )
            .await
            .unwrap(),
        );
        assert_eq!(plain["pads"][0]["fabrication_property"], "none");
    }

    #[tokio::test]
    async fn fabrication_detail_preserves_pad_numbers_properties_and_observability() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("pads.kicad_pcb");
        std::fs::write(
            &board,
            SAVED_BOARD_WITH_R1.replace(
                "(net \"SAVED\")",
                "(property pad_prop_heatsink) (net \"SAVED\")",
            ),
        )
        .unwrap();
        let server = spawn_kicad_holding(
            &board,
            vec![live_footprint("R1", vec![live_pad("1", 5.0, 5.9, "SAVED")])],
        );
        let ctx = ctx_talking_to(server.address().to_string());
        for source in ["saved", "auto"] {
            let mut args =
                json!({"board":board.to_string_lossy(),"reference":"R1","source":source});
            let full = parsed(&handle_get_component_pads(&args, &ctx).await.unwrap());
            args["detail"] = json!("fabrication");
            let result = handle_get_component_pads(&args, &ctx).await.unwrap();
            assert!(!result.is_error);
            let compact = parsed(&result);
            for key in [
                "reference",
                "pad_count",
                "source",
                "pad_fabrication_properties",
            ] {
                assert_eq!(compact[key], full[key], "{source} {key}");
            }
            let pads = compact["pads"].as_array().unwrap();
            assert_eq!(pads.len(), full["pads"].as_array().unwrap().len());
            for (pad, original) in pads.iter().zip(full["pads"].as_array().unwrap()) {
                assert_eq!(pad["number"], original["number"]);
                assert_eq!(
                    pad["fabrication_property"],
                    original["fabrication_property"]
                );
                assert_eq!(pad.as_object().unwrap().len(), 2);
            }
            assert_eq!(compact["geometry_omitted"], true);
        }
    }
}

/// The fabrication-property guard on the live whole-footprint tools: it
/// refuses before anything is sent, all or nothing, lets an ordinary
/// footprint through, and leaves the closed-board file path available.
#[cfg(test)]
mod fabrication_guard_tests {
    use super::*;
    use konnect_ipc::gen::kiapi;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::Arc;

    const BGA: &str = "(footprint \"Bga\" (version 20260206) (layer \"F.Cu\")\n\t(property \"Reference\" \"REF**\" (at 0 -2 0) (layer \"F.SilkS\"))\n\t(property \"Value\" \"Bga\" (at 0 2 0) (layer \"F.Fab\"))\n\t(attr smd)\n\t(pad \"A1\" smd circle (at 0 0) (size 0.3 0.3) (layers \"F.Cu\" \"F.Mask\") (property pad_prop_bga))\n)\n";

    fn saved_board() -> String {
        String::from(
            "(kicad_pcb (version 20260206)\n\t(footprint \"Parts:Bga\" (layer \"F.Cu\") (uuid \"aaaaaaaa-0000-4000-8000-000000000001\") (at 10 10)\n\t\t(property \"Reference\" \"U1\" (at 0 -2 0) (layer \"F.SilkS\"))\n\t\t(property \"Value\" \"Bga\" (at 0 2 0) (layer \"F.Fab\"))\n\t\t(attr smd)\n\t\t(pad \"A1\" smd circle (at 0 0) (size 0.3 0.3) (layers \"F.Cu\" \"F.Mask\") (property pad_prop_bga) (net \"GND\"))\n\t)\n\t(footprint \"Parts:Plain\" (layer \"F.Cu\") (uuid \"aaaaaaaa-0000-4000-8000-000000000002\") (at 20 10)\n\t\t(property \"Reference\" \"R1\" (at 0 -2 0) (layer \"F.SilkS\"))\n\t\t(property \"Value\" \"Plain\" (at 0 2 0) (layer \"F.Fab\"))\n\t\t(attr smd)\n\t\t(pad \"A1\" smd circle (at 0 0) (size 0.3 0.3) (layers \"F.Cu\" \"F.Mask\") (net \"GND\"))\n\t)\n)\n"
        )
    }

    fn project() -> (tempfile::TempDir, PathBuf) {
        let temp = tempfile::tempdir().unwrap();
        let pretty = temp.path().join("Parts.pretty");
        std::fs::create_dir_all(&pretty).unwrap();
        std::fs::write(pretty.join("Bga.kicad_mod"), BGA).unwrap();
        std::fs::write(
            pretty.join("Plain.kicad_mod"),
            BGA.replace(" (property pad_prop_bga)", ""),
        )
        .unwrap();
        std::fs::write(
            temp.path().join("fp-lib-table"),
            "(fp_lib_table (lib (name \"Parts\") (type \"KiCad\") (uri \"${KIPRJMOD}/Parts.pretty\") (options \"\") (descr \"\")))",
        )
        .unwrap();
        let board = temp.path().join("board.kicad_pcb");
        std::fs::write(&board, saved_board()).unwrap();
        (temp, board)
    }

    fn live(reference: &str, kiid: &str, entry: &str, x: f64) -> prost_types::Any {
        konnect_ipc::builders::pack_any(
            &kiapi::board::types::FootprintInstance {
                id: Some(kiapi::common::types::Kiid {
                    value: kiid.to_string(),
                }),
                position: Some(konnect_ipc::builders::vec2(x, 10.0)),
                reference_field: Some(kiapi::board::types::Field {
                    name: "Reference".to_string(),
                    text: Some(kiapi::board::types::BoardText {
                        text: Some(kiapi::common::types::Text {
                            text: reference.to_string(),
                            ..Default::default()
                        }),
                        ..Default::default()
                    }),
                    ..Default::default()
                }),
                definition: Some(kiapi::board::types::Footprint {
                    id: Some(kiapi::common::types::LibraryIdentifier {
                        library_nickname: "Parts".to_string(),
                        entry_name: entry.to_string(),
                    }),
                    ..Default::default()
                }),
                ..Default::default()
            },
            "kiapi.board.types.FootprintInstance",
        )
    }

    /// A KiCad holding `board` with U1 (BGA) and R1 (plain), counting every
    /// `UpdateItems` it receives.
    fn kicad(board: &Path, updates: Arc<AtomicUsize>) -> crate::test_support::MockIpcServer {
        let footprints = vec![
            live("U1", "aaaaaaaa-0000-4000-8000-000000000001", "Bga", 10.0),
            live("R1", "aaaaaaaa-0000-4000-8000-000000000002", "Plain", 20.0),
        ];
        crate::tools::pcb_board::board_mock::spawn_kicad_holding_board(board, move |command| {
            let url = command.type_url.as_str();
            if url.ends_with("GetItems") {
                return Some(konnect_ipc::builders::pack_any(
                    &kiapi::common::commands::GetItemsResponse {
                        header: None,
                        status: kiapi::common::types::ItemRequestStatus::IrsOk as i32,
                        items: footprints.clone(),
                    },
                    "kiapi.common.commands.GetItemsResponse",
                ));
            }
            if url.ends_with("BeginCommit") {
                return Some(konnect_ipc::builders::pack_any(
                    &kiapi::common::commands::BeginCommitResponse {
                        id: Some(kiapi::common::types::Kiid {
                            value: "commit".into(),
                        }),
                    },
                    "kiapi.common.commands.BeginCommitResponse",
                ));
            }
            if url.ends_with("EndCommit") {
                return Some(konnect_ipc::builders::pack_any(
                    &kiapi::common::commands::EndCommitResponse {},
                    "kiapi.common.commands.EndCommitResponse",
                ));
            }
            if url.ends_with("UpdateItems") {
                updates.fetch_add(1, Ordering::SeqCst);
                let request =
                    kiapi::common::commands::UpdateItems::decode(command.value.as_slice())
                        .expect("UpdateItems request");
                return Some(konnect_ipc::builders::pack_any(
                    &kiapi::common::commands::UpdateItemsResponse {
                        header: None,
                        status: kiapi::common::types::ItemRequestStatus::IrsOk as i32,
                        updated_items: request
                            .items
                            .into_iter()
                            .map(|item| kiapi::common::commands::ItemUpdateResult {
                                status: Some(kiapi::common::commands::ItemStatus {
                                    code: kiapi::common::commands::ItemStatusCode::IscOk as i32,
                                    error_message: String::new(),
                                }),
                                item: Some(item),
                            })
                            .collect(),
                    },
                    "kiapi.common.commands.UpdateItemsResponse",
                ));
            }
            None
        })
    }

    fn text(result: &CallToolResult) -> String {
        match &result.content[0] {
            crate::mcp::protocol::ToolContent::Text { text } => text.clone(),
            _ => String::new(),
        }
    }

    #[tokio::test]
    async fn a_property_carrying_live_footprint_refuses_the_whole_batch_before_sending() {
        let (_temp, board) = project();
        let updates = Arc::new(AtomicUsize::new(0));
        let server = kicad(&board, updates.clone());
        let ctx = crate::tools::pcb_board::board_mock::ctx_talking_to(server.address().to_string());
        let path = board.to_string_lossy().to_string();

        let batch = handle_set_component_placements(
            &json!({ "board": path, "placements": [
                {"reference": "R1", "x": 30.0, "y": 30.0, "rotation": 0.0},
                {"reference": "U1", "x": 40.0, "y": 40.0, "rotation": 90.0}
            ]}),
            &ctx,
        )
        .await
        .unwrap();
        assert!(batch.is_error);
        let refusal = text(&batch);
        assert!(
            refusal.contains("plan_blocked")
                && refusal.contains("U1")
                && refusal.contains("pad_prop_bga")
                && !refusal.contains("R1 ("),
            "{refusal}"
        );
        for result in [
            handle_move_component(
                &json!({ "board": path, "reference": "U1", "x": 1.0, "y": 2.0 }),
                &ctx,
            )
            .await
            .unwrap(),
            handle_rotate_component(
                &json!({ "board": path, "reference": "U1", "rotation": 45.0 }),
                &ctx,
            )
            .await
            .unwrap(),
            handle_edit_component(
                &json!({ "board": path, "reference": "U1", "value": "New" }),
                &ctx,
            )
            .await
            .unwrap(),
        ] {
            assert!(result.is_error, "{}", text(&result));
            assert!(text(&result).contains("plan_blocked"), "{}", text(&result));
        }
        assert_eq!(updates.load(Ordering::SeqCst), 0, "nothing reached KiCad");
        assert_eq!(std::fs::read_to_string(&board).unwrap(), saved_board());

        // An ordinary footprint is not blocked.
        let plain = handle_set_component_placements(
            &json!({ "board": path, "placements": [
                {"reference": "R1", "x": 30.0, "y": 30.0, "rotation": 0.0}
            ]}),
            &ctx,
        )
        .await
        .unwrap();
        assert!(!plain.is_error, "{}", text(&plain));
        assert_eq!(updates.load(Ordering::SeqCst), 1);
    }

    /// With the board closed the file path moves and rotates the same
    /// footprint in place and keeps its fabrication property.
    #[tokio::test]
    async fn the_closed_board_path_still_moves_property_carrying_footprints() {
        let (_temp, board) = project();
        let ctx = crate::tools::pcb_board::board_mock::ctx_talking_to(String::new());
        let result = handle_set_component_placements(
            &json!({ "board": board.to_string_lossy(), "placements": [
                {"reference": "U1", "x": 40.0, "y": 40.0, "rotation": 90.0},
                {"reference": "R1", "x": 50.0, "y": 40.0, "rotation": 0.0}
            ]}),
            &ctx,
        )
        .await
        .unwrap();
        assert!(!result.is_error, "{}", text(&result));
        let written = std::fs::read_to_string(&board).unwrap();
        assert!(written.contains("(at 40 40 90)"), "{written}");
        assert_eq!(written.matches("(property pad_prop_bga)").count(), 1);
    }
}

#[cfg(test)]
mod field_placement_tests {
    use super::*;

    #[test]
    fn field_anchors_come_from_the_library_footprint() {
        // R_0603-style: Reference above the silk at -1.43, Value below at 1.43.
        let source = "(footprint \"R_0603\"
	(property \"Reference\" \"REF**\"
		(at 0 -1.43 0)
		(layer \"F.SilkS\")
	)
	(property \"Value\" \"R_0603\"
		(at 0 1.43 0)
		(layer \"F.Fab\")
	)
)";
        let placement = extract_field_placement(source);
        assert_eq!(placement.reference_at, Some((0.0, -1.43, 0.0)));
        assert_eq!(placement.value_at, Some((0.0, 1.43, 0.0)));
    }

    #[test]
    fn missing_fields_leave_defaults() {
        let placement = extract_field_placement("(footprint \"bare\")");
        assert_eq!(placement.reference_at, None);
        assert_eq!(placement.value_at, None);
    }
}

#[cfg(test)]
mod pad_net_shape_tests {
    use super::*;
    use crate::router::ToolRouter;
    use crate::tools::ServerConfig;
    use std::sync::Arc;

    fn test_ctx() -> ToolContext {
        ToolContext::new(
            ServerConfig {
                kicad_cli: String::new(),
                kicad_binary: String::new(),
                ipc_address: String::new(),
                project_dir: None,
                jlcpcb_db_path: None,
                auto_load_toolsets: false,
                eager_toolsets: false,
            },
            Arc::new(ToolRouter::new()),
        )
    }

    async fn pads_of(board: &str) -> serde_json::Value {
        let dir = tempfile::tempdir().expect("tempdir");
        let path = dir.path().join("board.kicad_pcb");
        std::fs::write(&path, board).unwrap();
        let result = handle_get_component_pads(
            &json!({ "board": path.to_str().unwrap(), "reference": "R1" }),
            &test_ctx(),
        )
        .await
        .expect("handler should succeed");
        let body = match &result.content[0] {
            crate::mcp::protocol::ToolContent::Text { text } => text.clone(),
            _ => panic!("expected text content"),
        };
        serde_json::from_str(&body).unwrap()
    }

    fn board_with_pads(pads: &str) -> String {
        format!(
            "(kicad_pcb\n\t(version 20260206)\n\t(footprint \"TestPad\"\n\t\t(layer \"F.Cu\")\n\t\t(at 100 100)\n\
             \t\t(property \"Reference\" \"R1\" (at 0 0 0) (layer \"F.SilkS\"))\n{pads}\t)\n)\n"
        )
    }

    /// The reported bug: KiCad 10 puts the name at index 1, the old reader
    /// took index 2, and `.unwrap_or("")` turned that miss into a plausible
    /// empty string — so a fully connected pad looked exactly like an
    /// unconnected one.
    #[tokio::test]
    async fn kicad_10_pads_report_their_net_names() {
        let pads = pads_of(&board_with_pads(
            "\t\t(pad \"1\" smd rect (at -1 0) (size 1 1) (net \"GND\"))\n\
             \t\t(pad \"2\" smd rect (at 1 0) (size 1 1) (net \"VCC\"))\n",
        ))
        .await;
        assert_eq!(pads["pads"][0]["net"], json!("GND"));
        assert_eq!(pads["pads"][1]["net"], json!("VCC"));
    }

    #[tokio::test]
    async fn legacy_pads_still_report_their_net_names() {
        let pads = pads_of(&board_with_pads(
            "\t\t(pad \"1\" smd rect (at -1 0) (size 1 1) (net 1 \"GND\"))\n\
             \t\t(pad \"2\" smd rect (at 1 0) (size 1 1) (net 2 \"VCC\"))\n",
        ))
        .await;
        assert_eq!(pads["pads"][0]["net"], json!("GND"));
        assert_eq!(pads["pads"][1]["net"], json!("VCC"));
    }

    /// A pad with no net node is genuinely unconnected, and "" says so. This
    /// is the one case where the empty string is the right answer, which is
    /// why the unreadable case must not share it.
    #[tokio::test]
    async fn a_pad_with_no_net_node_reports_an_empty_name() {
        let pads = pads_of(&board_with_pads(
            "\t\t(pad \"1\" smd rect (at -1 0) (size 1 1))\n",
        ))
        .await;
        assert_eq!(pads["pads"][0]["net"], json!(""));
    }

    /// Present but unreadable is now loud. A bare `(net 1)` reference names no
    /// net, and null forces a caller to notice rather than concluding the pad
    /// is floating.
    #[tokio::test]
    async fn a_net_node_we_cannot_read_reports_null_not_empty() {
        let pads = pads_of(&board_with_pads(
            "\t\t(pad \"1\" smd rect (at -1 0) (size 1 1) (net 1))\n\
             \t\t(pad \"2\" smd rect (at 1 0) (size 1 1) (net))\n",
        ))
        .await;
        assert_eq!(pads["pads"][0]["net"], serde_json::Value::Null);
        assert_eq!(pads["pads"][1]["net"], serde_json::Value::Null);
    }

    /// The unconnected pseudo-net is named, and its name is empty — that is a
    /// real answer, not a failure, so it must not become null.
    #[tokio::test]
    async fn the_unconnected_pseudo_net_reports_an_empty_name() {
        let pads = pads_of(&board_with_pads(
            "\t\t(pad \"1\" smd rect (at -1 0) (size 1 1) (net 0 \"\"))\n",
        ))
        .await;
        assert_eq!(pads["pads"][0]["net"], json!(""));
    }
}

#[cfg(test)]
mod board_footprint_graphics_tests {
    use super::*;
    use serde_json::json;

    /// A malformed `points` is a caller mistake, so it must come back as a
    /// structured `invalid_argument` naming the field — not as an
    /// `anyhow`-flavoured handler error with the reason flattened into prose.
    /// The handler used to bubble `?` here, which is the class #194 tracks.
    #[test]
    fn a_malformed_points_argument_is_a_structured_invalid_argument() {
        let cases = [
            (json!("not an array"), "not an array"),
            (json!([{ "x": 1.0 }]), "point 0 has no numeric 'y'"),
            (json!([{ "y": 1.0 }]), "point 0 has no numeric 'x'"),
            (
                json!([{ "x": 0.0, "y": 0.0 }, { "x": 1.0, "y": "two" }]),
                "point 1 has no numeric 'y'",
            ),
        ];
        for (value, expected) in cases {
            let err = parse_points(&value).expect_err("must be refused");
            assert!(err.is_error, "{value} should be an error result");
            let text = format!("{:?}", err);
            assert!(
                text.contains("InvalidArgument") || text.contains("invalid_argument"),
                "must be structured, got: {text}"
            );
            assert!(
                text.contains(expected),
                "the message must say which point is wrong.\nwant: {expected}\ngot:  {text}"
            );
        }
    }

    #[test]
    fn well_formed_points_parse_in_order() {
        let points = parse_points(&json!([
            { "x": 0.0, "y": 0.0 },
            { "x": 2.5, "y": -1.5 },
            { "x": 2.5, "y": 2.5 }
        ]))
        .expect("valid");
        assert_eq!(points, vec![(0.0, 0.0), (2.5, -1.5), (2.5, 2.5)]);
    }

    /// An integer in JSON is a number; rejecting it would refuse
    /// `{"x": 0, "y": 0}`, which is what a caller writes for the origin.
    #[test]
    fn integer_coordinates_are_accepted() {
        assert_eq!(
            parse_points(&json!([{ "x": 0, "y": 0 }, { "x": 3, "y": 0 }, { "x": 3, "y": 3 }]))
                .expect("valid"),
            vec![(0.0, 0.0), (3.0, 0.0), (3.0, 3.0)]
        );
    }

    /// The two tools are registered, sit in `pcb_components`, and require the
    /// arguments the handlers read. A schema that drifts from its handler is
    /// the defect #217 was about, one layer up.
    #[test]
    fn both_board_graphics_tools_are_registered_with_the_arguments_they_read() {
        let tools = tools();
        for (name, required) in [
            ("list_board_footprint_graphics", vec!["board", "reference"]),
            (
                "edit_board_footprint_graphic",
                vec!["board", "reference", "uuid", "points"],
            ),
        ] {
            let def = tools
                .iter()
                .find(|t| t.name == name)
                .unwrap_or_else(|| panic!("{name} is not registered"));
            let got: Vec<&str> = def.input_schema["required"]
                .as_array()
                .unwrap_or_else(|| panic!("{name} declares no required list"))
                .iter()
                .filter_map(|v| v.as_str())
                .collect();
            assert_eq!(got, required, "{name} required arguments");
            for key in &required {
                assert!(
                    def.input_schema["properties"][key].is_object(),
                    "{name} requires '{key}' but does not declare it"
                );
            }
        }
    }
}

/// `place_component_array` declares `count_x` required and defaulted it to 1.
/// A caller who meant a 10x10 grid and mistyped the key got a single column of
/// real footprints committed to the live board over IPC, and `{"placed_count":
/// N}` back. The `count_x == 0` guard below it could never fire on that path
/// either, since the default was 1 (#218).
#[cfg(test)]
mod required_count_tests {
    use super::*;
    use serde_json::json;

    /// No IPC address configured, so any handler that reaches the IPC layer
    /// fails with the socket-path error — a different error therefore proves
    /// the handler refused before attempting anything.
    fn test_ctx_arc() -> std::sync::Arc<ToolContext> {
        std::sync::Arc::new(ToolContext::new(
            crate::tools::ServerConfig {
                kicad_cli: String::new(),
                kicad_binary: String::new(),
                ipc_address: String::new(),
                project_dir: None,
                jlcpcb_db_path: None,
                auto_load_toolsets: false,
                eager_toolsets: false,
            },
            std::sync::Arc::new(crate::router::ToolRouter::new()),
        ))
    }

    fn def(name: &str) -> ToolDef {
        tools()
            .into_iter()
            .find(|t| t.name == name)
            .unwrap_or_else(|| panic!("{name} is registered"))
    }

    fn error_of(result: &CallToolResult) -> serde_json::Value {
        let text = match result.content.first() {
            Some(crate::mcp::protocol::ToolContent::Text { text }) => text.clone(),
            other => panic!("expected text, got {other:?}"),
        };
        serde_json::from_str(&text).unwrap_or_else(|e| panic!("{e}: {text}"))
    }

    #[tokio::test]
    async fn place_component_array_refuses_a_missing_count_x() {
        let dir = tempfile::tempdir().unwrap();
        let board = dir.path().join("b.kicad_pcb");
        std::fs::write(&board, "(kicad_pcb\n  (version 20250610)\n)\n").unwrap();

        let result = (def("place_component_array").handler)(
            &json!({
                "board": board.display().to_string(),
                "footprint": "Resistor_SMD:R_0402_1005Metric",
                "start_x": 10.0,
                "start_y": 10.0,
                "count_y": 10,
                "spacing_x": 2.0,
            }),
            test_ctx_arc(),
        )
        .await
        .expect("no anyhow");

        assert!(result.is_error, "a missing count_x must be refused");
        let parsed = error_of(&result);
        assert_eq!(parsed["error"]["kind"], "invalid_argument");
        assert_eq!(parsed["error"]["field"], "count_x");
    }

    /// The zero guard still applies to an argument that is present and zero —
    /// it was only ever unreachable via the default.
    #[tokio::test]
    async fn an_explicit_zero_count_is_still_rejected() {
        let dir = tempfile::tempdir().unwrap();
        let board = dir.path().join("b.kicad_pcb");
        std::fs::write(&board, "(kicad_pcb\n  (version 20250610)\n)\n").unwrap();

        let result = (def("place_component_array").handler)(
            &json!({
                "board": board.display().to_string(),
                "footprint": "Resistor_SMD:R_0402_1005Metric",
                "start_x": 10.0, "start_y": 10.0,
                "count_x": 0, "count_y": 10, "spacing_x": 2.0,
            }),
            test_ctx_arc(),
        )
        .await
        .expect("no anyhow");
        assert!(result.is_error, "an explicit zero must still be rejected");
    }
}

#[cfg(test)]
mod placement_library_data_tests {
    use super::*;
    use konnect_ipc::gen::kiapi;
    use std::sync::{Arc, Mutex};

    /// A library footprint carrying everything `place_component` used to drop:
    /// mounting style and exclusion flags, description and keywords, and two
    /// 3D models with distinct offset, scale, rotation, opacity and visibility.
    /// Its exposed pad also carries `(zone_connect 2)`.
    const LIBRARY: &str = "(footprint \"QFN_Test\"\r\n\
        \t(version 20260206)\r\n\
        \t(generator \"pcbnew\")\r\n\
        \t(layer \"F.Cu\")\r\n\
        \t(descr \"Test QFN with an exposed pad\")\r\n\
        \t(tags \"qfn exposed-pad test\")\r\n\
        \t(property \"Reference\" \"REF**\" (at 0 -2 0) (layer \"F.SilkS\"))\r\n\
        \t(property \"Value\" \"QFN_Test\" (at 0 2 0) (layer \"F.Fab\"))\r\n\
        \t(attr smd exclude_from_pos_files exclude_from_bom allow_missing_courtyard)\r\n\
        \t(pad \"1\" smd roundrect (at -1 0) (size 0.6 0.24) (layers \"F.Cu\" \"F.Mask\" \"F.Paste\") (roundrect_rratio 0.25))\r\n\
        \t(pad \"2\" smd rect (at 0 0) (size 1 1) (layers \"F.Cu\" \"F.Mask\") (zone_connect 2))\r\n\
        \t(model \"${KICAD10_3DMODEL_DIR}/Test.3dshapes/QFN_Test.step\"\r\n\
        \t\t(offset (xyz 0.1 -0.2 0.3))\r\n\
        \t\t(scale (xyz 1 1.5 2))\r\n\
        \t\t(rotate (xyz -90 0 45))\r\n\
        \t)\r\n\
        \t(model \"extra.wrl\"\r\n\
        \t\t(hide yes)\r\n\
        \t\t(opacity 0.4)\r\n\
        \t\t(offset (xyz 0 0 1))\r\n\
        \t\t(scale (xyz 2 2 2))\r\n\
        \t\t(rotate (xyz 0 180 0))\r\n\
        \t)\r\n\
        )\r\n";

    fn project(dir: &Path) -> (PathBuf, PathBuf) {
        let pretty = dir.join("Test.pretty");
        std::fs::create_dir_all(&pretty).unwrap();
        let library = pretty.join("QFN_Test.kicad_mod");
        std::fs::write(&library, LIBRARY).unwrap();
        std::fs::write(
            dir.join("fp-lib-table"),
            format!(
                "(fp_lib_table\r\n\t(version 7)\r\n\t(lib (name \"Test\") (type \"KiCad\") (uri \"{}\") (options \"\") (descr \"\"))\r\n)\r\n",
                pretty.to_string_lossy()
            ),
        )
        .unwrap();
        let board = dir.join("b.kicad_pcb");
        std::fs::write(
            &board,
            "(kicad_pcb\n\t(version 20260206)\n\t(generator \"pcbnew\")\n)\n",
        )
        .unwrap();
        (board, library)
    }

    type Created = Arc<Mutex<Vec<prost_types::Any>>>;

    /// A KiCad holding `board` that keeps what it is asked to create, lists it
    /// back, and records every created item exactly as received.
    fn spawn_creating_kicad(board: &Path) -> (crate::test_support::MockIpcServer, Created) {
        let board = board.to_string_lossy().to_string();
        let created: Created = Arc::default();
        let held = created.clone();
        let server = crate::test_support::MockIpcServer::spawn(
            "place-library-data",
            move |request| {
                let command = request.message.expect("a command");
                let body = if command.type_url.ends_with("GetOpenDocuments") {
                    Some(konnect_ipc::builders::pack_any(
                        &kiapi::common::commands::GetOpenDocumentsResponse {
                            documents: vec![kiapi::common::types::DocumentSpecifier {
                                r#type: kiapi::common::types::DocumentType::DoctypePcb as i32,
                                project: None,
                                identifier: Some(
                                    kiapi::common::types::document_specifier::Identifier::BoardFilename(
                                        board.clone(),
                                    ),
                                ),
                            }],
                        },
                        "kiapi.common.commands.GetOpenDocumentsResponse",
                    ))
                } else if command.type_url.ends_with("GetItems") {
                    Some(konnect_ipc::builders::pack_any(
                        &kiapi::common::commands::GetItemsResponse {
                            header: None,
                            status: kiapi::common::types::ItemRequestStatus::IrsOk as i32,
                            items: held.lock().unwrap().clone(),
                        },
                        "kiapi.common.commands.GetItemsResponse",
                    ))
                } else if command.type_url.ends_with("CreateItems") {
                    let request =
                        kiapi::common::commands::CreateItems::decode(command.value.as_slice())
                            .expect("CreateItems request");
                    held.lock().unwrap().extend(request.items.iter().cloned());
                    Some(konnect_ipc::builders::pack_any(
                        &kiapi::common::commands::CreateItemsResponse {
                            header: None,
                            status: kiapi::common::types::ItemRequestStatus::IrsOk as i32,
                            created_items: request
                                .items
                                .into_iter()
                                .map(|item| kiapi::common::commands::ItemCreationResult {
                                    status: Some(kiapi::common::commands::ItemStatus {
                                        code: kiapi::common::commands::ItemStatusCode::IscOk as i32,
                                        error_message: String::new(),
                                    }),
                                    item: Some(item),
                                })
                                .collect(),
                        },
                        "kiapi.common.commands.CreateItemsResponse",
                    ))
                } else {
                    None
                };
                kiapi::common::ApiResponse {
                    status: Some(kiapi::common::ApiResponseStatus {
                        status: kiapi::common::ApiStatusCode::AsOk as i32,
                        error_message: String::new(),
                    }),
                    header: None,
                    message: body,
                }
            },
        );
        (server, created)
    }

    fn ctx_talking_to(address: String) -> ToolContext {
        ToolContext::new(
            crate::tools::ServerConfig {
                kicad_cli: String::new(),
                kicad_binary: String::new(),
                ipc_address: address,
                project_dir: None,
                jlcpcb_db_path: None,
                auto_load_toolsets: false,
                eager_toolsets: false,
            },
            std::sync::Arc::new(crate::router::ToolRouter::new()),
        )
    }

    fn vector(v: &Option<kiapi::common::types::Vector3D>) -> (f64, f64, f64) {
        let v = v.as_ref().expect("a vector");
        (v.x_nm, v.y_nm, v.z_nm)
    }

    /// The live path sends the library's mounting style, flags, description,
    /// keywords and every model with its transforms on the very message
    /// KiCad receives, and leaves the library file byte for byte as it was.
    #[tokio::test]
    async fn place_component_sends_the_library_data_on_the_created_item() {
        let tmp = tempfile::tempdir().unwrap();
        let (board, library) = project(tmp.path());
        let (server, created) = spawn_creating_kicad(&board);

        let result = handle_place_component(
            &json!({
                "board": board.to_string_lossy(),
                "footprint": "Test:QFN_Test",
                "reference": "U1",
                "x": 30.0, "y": 40.0, "rotation": 37.0,
            }),
            &ctx_talking_to(server.address().to_string()),
        )
        .await
        .unwrap();
        assert!(!result.is_error, "{:?}", result.content);

        let items = created.lock().unwrap().clone();
        let [item] = items.as_slice() else {
            panic!("exactly one created item, got {}", items.len());
        };
        assert!(konnect_ipc::builders::any_is(
            item,
            "kiapi.board.types.FootprintInstance"
        ));
        let footprint = kiapi::board::types::FootprintInstance::decode(item.value.as_slice())
            .expect("a footprint instance");

        let attributes = footprint.attributes.as_ref().expect("instance attributes");
        assert_eq!(
            attributes.mounting_style(),
            kiapi::board::types::FootprintMountingStyle::FmsSmd
        );
        assert!(attributes.exclude_from_position_files);
        assert!(attributes.exclude_from_bill_of_materials);
        assert!(attributes.exempt_from_courtyard_requirement);
        assert!(!attributes.not_in_schematic);
        assert!(!attributes.do_not_populate);
        assert!(!attributes.allow_soldermask_bridges);

        let definition = footprint.definition.as_ref().expect("a definition");
        let text = definition
            .attributes
            .as_ref()
            .expect("definition attributes");
        assert_eq!(text.description, "Test QFN with an exposed pad");
        assert_eq!(text.keywords, "qfn exposed-pad test");
        let id = definition.id.as_ref().unwrap();
        assert_eq!(
            (id.library_nickname.as_str(), id.entry_name.as_str()),
            ("Test", "QFN_Test")
        );

        let models: Vec<_> = definition
            .items
            .iter()
            .filter(|child| {
                konnect_ipc::builders::any_is(child, "kiapi.board.types.Footprint3DModel")
            })
            .map(|child| {
                kiapi::board::types::Footprint3DModel::decode(child.value.as_slice()).unwrap()
            })
            .collect();
        assert_eq!(models.len(), 2, "every library model is sent");
        assert_eq!(
            models[0].filename,
            "${KICAD10_3DMODEL_DIR}/Test.3dshapes/QFN_Test.step"
        );
        assert_eq!(vector(&models[0].offset), (0.1, -0.2, 0.3));
        assert_eq!(vector(&models[0].scale), (1.0, 1.5, 2.0));
        assert_eq!(vector(&models[0].rotation), (-90.0, 0.0, 45.0));
        assert!(models[0].visible);
        assert_eq!(models[0].opacity, 1.0);
        assert_eq!(models[1].filename, "extra.wrl");
        assert_eq!(vector(&models[1].offset), (0.0, 0.0, 1.0));
        assert_eq!(vector(&models[1].scale), (2.0, 2.0, 2.0));
        assert_eq!(vector(&models[1].rotation), (0.0, 180.0, 0.0));
        assert!(!models[1].visible);
        assert_eq!(models[1].opacity, 0.4);

        // The pads are still the library's, the exposed pad's zone connection
        // included, and the reference is the requested one.
        let pads: Vec<_> = definition
            .items
            .iter()
            .filter(|child| konnect_ipc::builders::any_is(child, "kiapi.board.types.Pad"))
            .map(|child| kiapi::board::types::Pad::decode(child.value.as_slice()).unwrap())
            .collect();
        assert_eq!(pads.len(), 2);
        let zone = |number: &str| {
            pads.iter()
                .find(|pad| pad.number == number)
                .unwrap()
                .pad_stack
                .as_ref()
                .unwrap()
                .zone_settings
                .as_ref()
                .map(|settings| settings.zone_connection)
        };
        // Pad 1 has no `(zone_connect …)`: it goes out inherited, with the
        // thermal spoke angle the library pad holds.
        assert_eq!(
            zone("1"),
            Some(kiapi::board::types::ZoneConnectionStyle::ZcsInherited as i32)
        );
        assert_eq!(
            zone("2"),
            Some(kiapi::board::types::ZoneConnectionStyle::ZcsFull as i32)
        );
        let reference = footprint
            .reference_field
            .as_ref()
            .and_then(|field| field.text.as_ref())
            .and_then(|text| text.text.as_ref())
            .map(|text| text.text.as_str());
        assert_eq!(reference, Some("U1"));

        assert_eq!(
            std::fs::read_to_string(&library).unwrap(),
            LIBRARY,
            "the library source is only read"
        );

        // The extracted placement method still refuses a duplicate reference
        // before creating anything.
        let duplicate = handle_place_component(
            &json!({
                "board": board.to_string_lossy(),
                "footprint": "Test:QFN_Test",
                "reference": "U1", "x": 0.0, "y": 0.0,
            }),
            &ctx_talking_to(server.address().to_string()),
        )
        .await
        .unwrap();
        assert!(duplicate.is_error, "{:?}", duplicate.content);
        assert_eq!(created.lock().unwrap().len(), 1, "nothing more was created");
    }

    /// A library footprint whose attributes cannot be read is refused before
    /// anything is created, rather than placed without them.
    #[tokio::test]
    async fn unreadable_library_data_refuses_the_placement_before_creating_anything() {
        let tmp = tempfile::tempdir().unwrap();
        let (board, library) = project(tmp.path());
        std::fs::write(
            &library,
            LIBRARY.replace("allow_missing_courtyard", "mystery_flag"),
        )
        .unwrap();
        let (server, created) = spawn_creating_kicad(&board);

        let result = handle_place_component(
            &json!({
                "board": board.to_string_lossy(),
                "footprint": "Test:QFN_Test",
                "reference": "U1", "x": 1.0, "y": 2.0,
            }),
            &ctx_talking_to(server.address().to_string()),
        )
        .await
        .unwrap();
        assert!(result.is_error);
        let text = format!("{:?}", result.content);
        assert!(text.contains("mystery_flag"), "{text}");
        assert!(created.lock().unwrap().is_empty());
    }

    /// The item carries the reference the placement checks and reads back;
    /// a mismatch is refused before KiCad is asked to create anything.
    #[test]
    fn a_footprint_item_must_carry_the_reference_it_is_placed_under() {
        let tmp = tempfile::tempdir().unwrap();
        let (board, _library) = project(tmp.path());
        let (server, created) = spawn_creating_kicad(&board);
        let prepared = PreparedFootprintPlacement::from_library(
            LIBRARY,
            "Test:QFN_Test",
            "U7",
            None,
            0.0,
            0.0,
            0.0,
            "F.Cu",
            board.parent(),
        )
        .unwrap();
        let item = prepared
            .build_item("Test:QFN_Test", "U7", 0.0, 0.0, 0.0, "F.Cu")
            .unwrap();
        let client = KiCadIpcClient::new(server.address());
        let error = client
            .place_footprint_item(&board, "U8", item)
            .unwrap_err()
            .to_string();
        assert!(error.contains("carries reference 'U7'"), "{error}");
        assert!(created.lock().unwrap().is_empty());
    }

    /// Inline geometry has no library, so its item is exactly the plain
    /// builder's: no attributes, description or models are invented.
    #[test]
    fn inline_placement_builds_the_plain_item() {
        let pads = vec![konnect_ipc::IpcPadDefinition {
            number: "1".into(),
            pad_type: "np_thru_hole".into(),
            shape: "circle".into(),
            x: 0.0,
            y: 0.0,
            rotation: 0.0,
            size_x: 3.2,
            size_y: 3.2,
            drill_x: Some(3.2),
            drill_y: Some(3.2),
            drill_oval: false,
            layers: vec!["*.Cu".into(), "*.Mask".into()],
            roundrect_ratio: 0.0,
            custom: None,
            zone_connection: None,
            thermal: Default::default(),
        }];
        let prepared = PreparedFootprintPlacement::from_inline(
            "MountingHole".into(),
            pads.clone(),
            Vec::new(),
            Default::default(),
            String::new(),
        );
        let built = prepared
            .build_item(
                "MountingHole:MountingHole_3.2mm_M3",
                "H1",
                5.0,
                6.0,
                0.0,
                "F.Cu",
            )
            .unwrap();
        let plain = KiCadIpcClient::build_footprint_item(
            "MountingHole:MountingHole_3.2mm_M3",
            "H1",
            "MountingHole",
            &pads,
            &[],
            &Default::default(),
            5.0,
            6.0,
            0.0,
            "F.Cu",
        )
        .unwrap();
        assert_eq!(built, plain);
        let footprint =
            kiapi::board::types::FootprintInstance::decode(built.value.as_slice()).unwrap();
        assert!(footprint.attributes.is_none());
    }

    /// The byte-exact stock footprint's exposed pad carries a fabrication
    /// property KiCad's API cannot send. Placement names it and creates
    /// nothing, rather than placing a pad that differs from the library.
    #[tokio::test]
    async fn an_uncarried_pad_property_refuses_the_placement_by_name() {
        let tmp = tempfile::tempdir().unwrap();
        let (board, library) = project(tmp.path());
        std::fs::write(
            &library,
            include_str!("../../tests/fixtures/custom_pads_texas_rje0020a_kicad10.kicad_mod"),
        )
        .unwrap();
        let (server, created) = spawn_creating_kicad(&board);

        let result = handle_place_component(
            &json!({
                "board": board.to_string_lossy(),
                "footprint": "Test:QFN_Test",
                "reference": "U1", "x": 1.0, "y": 2.0,
            }),
            &ctx_talking_to(server.address().to_string()),
        )
        .await
        .unwrap();
        assert!(result.is_error);
        let text = format!("{:?}", result.content);
        assert!(
            text.contains("pad '21'") && text.contains("pad_prop_heatsink"),
            "{text}"
        );
        assert!(created.lock().unwrap().is_empty());
    }
}

/// The compact saved readers: exact filters over the values the full answer
/// reports, duplicate pad numbers kept, empty matches told apart from unknown
/// references, and board-space pad positions for rotated and back-side parts.
#[cfg(test)]
mod compact_reader_tests {
    use super::*;
    use crate::tools::pcb_board::board_mock::ctx_talking_to;

    const BOARD: &str = r#"(kicad_pcb (version 20260206)
	(footprint "Test:Qfn" (layer "F.Cu") (uuid "11111111-1111-4111-8111-111111111111") (at 20 30 37)
		(property "Reference" "U1" (at 0 -2 37) (layer "F.SilkS"))
		(property "Value" "QFN" (at 0 2 37) (layer "F.Fab"))
		(pad "A1" smd custom (at 1 2 37) (size 0.3 0.3) (layers "F.Cu") (property pad_prop_bga) (net 1 "GND")
			(primitives (gr_poly (pts (xy -1 -1) (xy 1 -1) (xy 1 1)) (width 0) (fill yes)))
			(uuid "41111111-1111-4111-8111-111111111111"))
		(pad "EP" smd rect (at -0.5 0 37) (size 1 1) (layers "F.Cu" "F.Paste") (net 1 "GND") (uuid "42222222-2222-4222-8222-222222222222"))
		(pad "EP" smd rect (at 0.5 0 37) (size 1 1) (layers "F.Cu" "F.Paste") (net 1 "GND") (uuid "43333333-3333-4333-8333-333333333333"))
		(pad "2" smd rect (at 0 1 37) (size 0.3 0.3) (layers "F.Cu"))
		(model "keep.step" (offset (xyz 1 2 3))))
	(footprint "Resistor_SMD:R_0402" (layer "B.Cu") (uuid "12222222-2222-4222-8222-222222222222") (at 50 60 90)
		(property "Reference" "R4001" (at 0 1.5 90) (layer "B.SilkS"))
		(property "Value" "10k" (at 0 -1.5 90) (layer "B.Fab"))
		(pad "1" smd roundrect (at -0.5 0 90) (size 0.6 0.5) (layers "B.Cu" "B.Paste" "B.Mask") (net 2 "VCC") (uuid "44444444-4444-4444-8444-444444444444"))
		(pad "2" smd roundrect (at 0.5 0 180) (size 0.6 0.5) (layers "B.Cu" "B.Paste" "B.Mask") (net 1 "GND") (uuid "45555555-5555-4555-8555-555555555555")))
	(footprint "Test:Dup" (layer "F.Cu") (uuid "13333333-3333-4333-8333-333333333333") (at 1 1) (property "Reference" "DUP" (at 0 0 0)) (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu")))
	(footprint "Test:Dup" (layer "F.Cu") (uuid "14444444-4444-4444-8444-444444444444") (at 2 2) (property "Reference" "DUP" (at 0 0 0)) (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu")))
)
"#;

    fn board() -> (tempfile::TempDir, std::path::PathBuf) {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("compact.kicad_pcb");
        std::fs::write(&path, BOARD).unwrap();
        (dir, path)
    }

    async fn pads(args: serde_json::Value) -> (bool, serde_json::Value) {
        let result = handle_get_component_pads(&args, &ctx_talking_to(String::new()))
            .await
            .unwrap();
        let text = match &result.content[0] {
            crate::mcp::protocol::ToolContent::Text { text } => text.clone(),
            other => panic!("{other:?}"),
        };
        (
            result.is_error,
            serde_json::from_str(&text).unwrap_or_else(|_| json!({ "text": text })),
        )
    }

    #[tokio::test]
    async fn connectivity_is_compact_board_space_and_carries_the_footprint_pose() {
        let (_dir, path) = board();
        let (error, body) =
            pads(json!({ "board": path, "reference": "U1", "detail": "connectivity" })).await;
        assert!(!error, "{body}");
        assert_eq!(body["detail"], "connectivity");
        assert_eq!(body["pad_count"], 4);
        let a1 = &body["pads"][0];
        let mut keys: Vec<&str> = a1.as_object().unwrap().keys().map(String::as_str).collect();
        keys.sort_unstable();
        assert_eq!(
            keys,
            [
                "fabrication_property",
                "net",
                "number",
                "pad_type",
                "rotation_deg",
                "uuid",
                "x",
                "y"
            ]
        );
        // (1, 2) turned by 37° about (20, 30): x' = x·cos θ + y·sin θ, y' = −x·sin θ + y·cos θ.
        let theta = 37f64.to_radians();
        let x = 20.0 + theta.cos() + 2.0 * theta.sin();
        let y = 30.0 - theta.sin() + 2.0 * theta.cos();
        assert!((a1["x"].as_f64().unwrap() - x).abs() < 1e-9, "{a1}");
        assert!((a1["y"].as_f64().unwrap() - y).abs() < 1e-9, "{a1}");
        assert_eq!(
            a1["rotation_deg"], 37.0,
            "the saved pad angle is already board-space"
        );
        assert_eq!(a1["fabrication_property"], "pad_prop_bga");
        assert_eq!(body["pads"][3]["fabrication_property"], "none");
        assert_eq!(body["pads"][3]["net"], "", "an unconnected pad is \"\"");
        assert_eq!(body["pad_fabrication_properties"]["observable"], true);
        let fp = &body["footprint"];
        assert_eq!(fp["uuid"], "11111111-1111-4111-8111-111111111111");
        assert_eq!(fp["footprint"], "Test:Qfn");
        assert_eq!(fp["rotation"], 37.0);
        assert_eq!(fp["layer"], "F.Cu");
        assert_eq!(fp["value"], "QFN");

        // A back-side part: its saved local frame is already the flipped one,
        // so board space is the same turn about the anchor.
        let (_, r) =
            pads(json!({ "board": path, "reference": "R4001", "detail": "connectivity" })).await;
        assert_eq!(r["footprint"]["layer"], "B.Cu");
        assert!((r["pads"][0]["x"].as_f64().unwrap() - 50.0).abs() < 1e-9);
        assert!((r["pads"][0]["y"].as_f64().unwrap() - 60.5).abs() < 1e-9);
        assert!((r["pads"][1]["y"].as_f64().unwrap() - 59.5).abs() < 1e-9);
        assert_eq!(r["pads"][1]["rotation_deg"], 180.0);
        assert_eq!(r["pads"][0]["net"], "VCC");
    }

    #[tokio::test]
    async fn filters_select_actual_values_and_keep_duplicate_pad_numbers() {
        let (_dir, path) = board();
        let (_, ep) = pads(json!({ "board": path, "reference": "U1", "detail": "connectivity", "pad_numbers": ["EP"] })).await;
        assert_eq!(ep["matched_pad_count"], 2, "{ep}");
        assert_eq!(ep["total_pad_count"], 4);
        assert_ne!(ep["pads"][0]["uuid"], ep["pads"][1]["uuid"]);
        assert_eq!(ep["unmatched_pad_numbers"], json!([]));

        let (_, gnd) = pads(json!({ "board": path, "reference": "U1", "nets": ["GND"], "pad_numbers": ["A1", "2"] })).await;
        assert_eq!(gnd["matched_pad_count"], 1, "{gnd}");
        assert_eq!(gnd["pads"][0]["number"], "A1");
        assert!(
            gnd["pads"][0].get("copper_layers").is_some(),
            "full detail keeps geometry"
        );

        let (_, unconnected) =
            pads(json!({ "board": path, "reference": "U1", "nets": [""] })).await;
        assert_eq!(unconnected["pads"][0]["number"], "2");

        // A real footprint with no match is an empty answer, not an error.
        let (error, none) = pads(
            json!({ "board": path, "reference": "U1", "detail": "connectivity", "nets": ["NOPE"] }),
        )
        .await;
        assert!(!error);
        assert_eq!(none["matched_pad_count"], 0);
        assert_eq!(none["unmatched_nets"], json!(["NOPE"]));
        assert_eq!(
            none["footprint"]["uuid"],
            "11111111-1111-4111-8111-111111111111"
        );

        // An unknown reference is an error; an ambiguous one is refused.
        let (error, _) =
            pads(json!({ "board": path, "reference": "U9", "detail": "connectivity" })).await;
        assert!(error);
        let (error, dup) =
            pads(json!({ "board": path, "reference": "DUP", "detail": "connectivity" })).await;
        assert!(error);
        assert_eq!(dup["error"]["kind"], "ambiguous_target", "{dup}");
        assert_eq!(dup["error"]["candidates"].as_array().unwrap().len(), 2);

        // Without a filter or compact detail the existing answer is unchanged.
        let (_, full) = pads(json!({ "board": path, "reference": "U1" })).await;
        assert!(full.get("footprint").is_none());
        assert!(full.get("matched_pad_count").is_none());
        assert_eq!(full["pad_count"], 4);
    }

    #[tokio::test]
    async fn the_saved_component_list_filters_exact_references_and_names_its_source() {
        let (_dir, path) = board();
        let result = handle_get_component_list(
            &json!({ "board": path, "source": "saved", "references": ["R4001", "DUP", "NOPE"] }),
            &ctx_talking_to(String::new()),
        )
        .await
        .unwrap();
        let text = match &result.content[0] {
            crate::mcp::protocol::ToolContent::Text { text } => text.clone(),
            other => panic!("{other:?}"),
        };
        let body: serde_json::Value = serde_json::from_str(&text).unwrap();
        assert_eq!(body["source"], "saved");
        assert_eq!(
            body["source_evidence"]["excludes_unsaved_editor_state"],
            true
        );
        assert_eq!(body["count"], 3);
        assert_eq!(body["unknown_references"], json!(["NOPE"]));
        assert_eq!(body["duplicate_references"], json!(["DUP"]));
        let r = &body["components"][0];
        assert_eq!(r["reference"], "R4001");
        assert_eq!(r["uuid"], "12222222-2222-4222-8222-222222222222");
        assert_eq!(r["layer"], "B.Cu");
        assert_eq!(r["rotation"], 90.0);
        assert_eq!(r["value"], "10k");
        assert_eq!(r["x"], 50.0);

        // No KiCad and no source: the live default refuses rather than read the file.
        let live =
            handle_get_component_list(&json!({ "board": path }), &ctx_talking_to(String::new()))
                .await
                .unwrap();
        assert!(live.is_error);
    }
}

/// `set_component_placements` with a target side: mirror first, then place,
/// in one closed-board write or none.
#[cfg(test)]
mod side_batch_tests {
    use super::*;
    use crate::tools::pcb_board::board_mock::{ctx_talking_to, spawn_kicad_holding_board};

    const BOARD: &str = "(kicad_pcb (version 20260206)
\t(footprint \"Resistor_SMD:R_0402\" (layer \"F.Cu\") (uuid \"aaaaaaaa-1111-4111-8111-111111111111\") (at 10 20 0)
\t\t(property \"Reference\" \"R4001\" (at 0 -1.2 0) (layer \"F.SilkS\") (uuid \"aaaaaaaa-2222-4222-8222-222222222222\") (effects (font (size 1 1) (thickness 0.15))))
\t\t(fp_line (start -0.5 -0.3) (end 0.5 -0.3) (stroke (width 0.1) (type solid)) (layer \"F.SilkS\") (uuid \"aaaaaaaa-3333-4333-8333-333333333333\"))
\t\t(pad \"1\" smd roundrect (at -0.5 0.1 0) (size 0.6 0.5) (layers \"F.Cu\" \"F.Paste\" \"F.Mask\") (roundrect_rratio 0.25) (net 2 \"VCC\") (uuid \"aaaaaaaa-4444-4444-8444-444444444444\"))
\t\t(pad \"2\" smd roundrect (at 0.5 0.1 0) (size 0.6 0.5) (layers \"F.Cu\" \"F.Paste\" \"F.Mask\") (roundrect_rratio 0.25) (net 1 \"GND\") (uuid \"aaaaaaaa-5555-4555-8555-555555555555\"))
\t\t(model \"${KICAD9_3DMODEL_DIR}/Resistor_SMD.3dshapes/R_0402_1005Metric.step\" (offset (xyz 0 0 0)) (scale (xyz 1 1 1)) (rotate (xyz 0 0 0))))
\t(footprint \"Capacitor_SMD:C_0402\" (layer \"F.Cu\") (uuid \"bbbbbbbb-1111-4111-8111-111111111111\") (at 30 20 0)
\t\t(property \"Reference\" \"C1\" (at 0 -1.2 0) (layer \"F.SilkS\") (uuid \"bbbbbbbb-2222-4222-8222-222222222222\") (effects (font (size 1 1) (thickness 0.15))))
\t\t(pad \"1\" smd roundrect (at -0.5 0 0) (size 0.6 0.5) (layers \"F.Cu\" \"F.Paste\" \"F.Mask\") (roundrect_rratio 0.25) (net 1 \"GND\") (uuid \"bbbbbbbb-4444-4444-8444-444444444444\")))
\t(footprint \"Custom:Odd\" (layer \"F.Cu\") (uuid \"cccccccc-1111-4111-8111-111111111111\") (at 50 20 0)
\t\t(property \"Reference\" \"U9\" (at 0 -1.2 0) (layer \"F.SilkS\") (effects (font (size 1 1) (thickness 0.15))))
\t\t(pad \"1\" smd custom (at 0 0 0) (size 0.5 0.5) (layers \"F.Cu\") (primitives (gr_poly (pts (xy 0 0) (xy 1 0) (xy 1 1)) (width 0) (fill yes)))))
)
";

    fn result_body(result: &CallToolResult) -> serde_json::Value {
        match &result.content[0] {
            crate::mcp::protocol::ToolContent::Text { text } => {
                serde_json::from_str(text).unwrap_or_else(|_| json!({ "text": text }))
            }
            other => panic!("{other:?}"),
        }
    }

    #[tokio::test]
    async fn a_mixed_batch_mirrors_then_places_in_one_write() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("sides.kicad_pcb");
        std::fs::write(&board, BOARD).unwrap();
        let args = json!({ "board": board, "placements": [
            { "reference": "R4001", "x": 12, "y": 25, "rotation": 90, "layer": "B.Cu" },
            { "reference": "C1", "x": 31, "y": 21, "rotation": 0 }
        ] });
        let result = handle_set_component_placements(&args, &ctx_talking_to(String::new()))
            .await
            .unwrap();
        assert!(!result.is_error, "{:?}", result.content);
        let body = result_body(&result);
        assert_eq!(body["sides"][0]["flipped"], true);
        assert_eq!(body["sides"][0]["layer"], "B.Cu");
        assert_eq!(body["sides"][1]["flipped"], false);
        assert_eq!(body["sides"][1]["layer"], "F.Cu");

        // Exactly the existing single-footprint flip followed by the existing
        // closed-board set, composed.
        let (flipped, changed) =
            prepare_closed_board_footprint_side(BOARD, "R4001", "B.Cu").unwrap();
        assert!(changed);
        let placed = prepare_closed_board_footprint_update(
            &flipped,
            "R4001",
            FootprintPlacementUpdate::Set {
                x: 12.0,
                y: 25.0,
                rotation: 90.0,
            },
        )
        .unwrap();
        let expected = prepare_closed_board_footprint_update(
            &placed,
            "C1",
            FootprintPlacementUpdate::Set {
                x: 31.0,
                y: 21.0,
                rotation: 0.0,
            },
        )
        .unwrap();
        let written = std::fs::read_to_string(&board).unwrap();
        assert_eq!(written, expected);

        // The mirrored geometry: root on B.Cu at the requested pose, pads
        // mirrored in Y and moved to the back layers, text mirrored onto
        // B.SilkS, every identity, net and model kept.
        let tree = konnect_sexp::parse_sexp(&written).unwrap();
        let r = tree
            .find_all("footprint")
            .into_iter()
            .find(|fp| footprint_reference(fp).as_deref() == Some("R4001"))
            .unwrap();
        assert_eq!(r.find_str("layer"), Some("B.Cu"));
        let at = r.find("at").unwrap();
        assert_eq!(
            (at.get_f64(1), at.get_f64(2), at.get_f64(3)),
            (Some(12.0), Some(25.0), Some(90.0))
        );
        let pad = r.find_all("pad")[0];
        assert_eq!(pad.find("at").unwrap().get_f64(2), Some(-0.1));
        let layers: Vec<&str> = pad.find("layers").unwrap().children().unwrap()[1..]
            .iter()
            .filter_map(konnect_sexp::SexpNode::as_str)
            .collect();
        assert_eq!(layers, ["B.Cu", "B.Paste", "B.Mask"]);
        assert_eq!(
            konnect_sexp::net::net_name(pad.find("net").unwrap()),
            Some("VCC")
        );
        assert!(written.contains("(uuid \"aaaaaaaa-4444-4444-8444-444444444444\")"));
        assert!(written.contains("(uuid \"aaaaaaaa-2222-4222-8222-222222222222\")"));
        assert!(written.contains("R_0402_1005Metric.step\" (offset (xyz 0 0 0))"));
        let reference = r.find_all("property")[0];
        assert_eq!(reference.find_str("layer"), Some("B.SilkS"));
        assert!(reference.find("effects").unwrap().find("justify").is_some());
        assert!(
            written.contains("(layer \"B.SilkS\") (uuid \"aaaaaaaa-3333-4333-8333-333333333333\")")
        );
    }

    #[tokio::test]
    async fn one_unsupported_member_or_an_open_board_changes_nothing() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("sides.kicad_pcb");
        std::fs::write(&board, BOARD).unwrap();
        let args = json!({ "board": board, "placements": [
            { "reference": "R4001", "x": 12, "y": 25, "rotation": 90, "layer": "B.Cu" },
            { "reference": "U9", "x": 50, "y": 21, "rotation": 0, "layer": "B.Cu" }
        ] });
        let refused = handle_set_component_placements(&args, &ctx_talking_to(String::new()))
            .await
            .unwrap();
        assert!(refused.is_error);
        assert!(format!("{:?}", refused.content).contains("custom pads"));
        assert_eq!(std::fs::read_to_string(&board).unwrap(), BOARD);

        let bad = json!({ "board": board, "placements": [
            { "reference": "R4001", "x": 12, "y": 25, "rotation": 90, "layer": "F.SilkS" }
        ] });
        assert!(
            handle_set_component_placements(&bad, &ctx_talking_to(String::new()))
                .await
                .unwrap()
                .is_error
        );

        // KiCad holds the board: refused before any live command but the
        // open-document query.
        let seen = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
        let log = seen.clone();
        let server = spawn_kicad_holding_board(&board, move |command| {
            log.lock().unwrap().push(command.type_url.clone());
            None
        });
        let ok = json!({ "board": board, "placements": [
            { "reference": "R4001", "x": 12, "y": 25, "rotation": 90, "layer": "B.Cu" }
        ] });
        let refused =
            handle_set_component_placements(&ok, &ctx_talking_to(server.address().to_string()))
                .await
                .unwrap();
        assert!(refused.is_error);
        assert_eq!(
            result_body(&refused)["error"]["reasons"][0],
            "side_change_requires_closed_board"
        );
        assert!(
            seen.lock().unwrap().is_empty(),
            "{:?}",
            seen.lock().unwrap()
        );
        drop(server);

        let lock = konnect_sexp::writer::kicad_editor_lock_path(&board).unwrap();
        std::fs::write(&lock, "own-test-lock").unwrap();
        assert!(
            handle_set_component_placements(&ok, &ctx_talking_to(String::new()))
                .await
                .unwrap()
                .is_error
        );
        std::fs::remove_file(&lock).unwrap();
        assert_eq!(std::fs::read_to_string(&board).unwrap(), BOARD);
    }
}
