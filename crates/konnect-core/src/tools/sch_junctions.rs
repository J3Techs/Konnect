//! Junction dots on one sheet: what meets at each, and which ones KiCad's
//! junction rule does not call for.
//!
//! The rule itself is [`ConnectivityIndex::junction_site`], shared with every
//! delete and move that re-judges the dots it touched. This module adds the two
//! whole-sheet tools: `list_schematic_junctions` (registered in `sch_analysis`)
//! and `prune_schematic_junctions` (registered in `sch_batch`, beside the other
//! one-write sheet edits).

use crate::mcp::protocol::CallToolResult;
use crate::tool;
use crate::tools::sch_connectivity::{
    ConnectivityIndex, JunctionSite, WireMeets, COINCIDENT_TOLERANCE,
};
use crate::tools::{get_path, ToolContext, ToolDef};
use konnect_sexp::{
    geometry::points_coincident,
    parser::parse_sexp,
    schematic::{extract_all_net_labels, extract_buses, extract_wires, parse_at, LabelKind, Wire},
    writer::{
        apply_edits, find_block_starts, find_block_with_leading_whitespace, read_consistent,
        write_atomic_if_unchanged, SexpEdit,
    },
    SexpNode,
};
use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet};

/// How the rule is applied, returned with every answer so a caller can check a
/// verdict without reading this source.
const RULE: &str = "KiCad's junction rule: a dot is needed where three or more distinct exits \
     meet. A wire ending at the dot leaves one way and a wire running through it leaves two; \
     each symbol unit and each sheet pin with a connection point there is one more exit. Labels \
     and no-connect flags are not exits. So a T, a pin on a wire's span or a crossing joined on \
     purpose needs a dot; a corner, a straight join, a lone wire or a wire ending on a pin does \
     not. A second dot on one point is a duplicate. Dots touching a bus or bus entry are not \
     judged.";

pub(crate) fn list_tool() -> ToolDef {
    tool!(
        "list_schematic_junctions",
        "List every junction dot on one sheet with its UUID and what meets there: each wire \
         (UUID, and whether it ends at the dot or runs through it, with the direction it \
         leaves), each component pin, each hierarchical sheet pin, and any label or no-connect \
         flag. Each dot carries KiCad's verdict: needed where three or more distinct exits \
         meet (a T, a pin on a wire's span, a crossing joined on purpose); unneeded on a \
         corner, a straight join, a lone wire or a wire ending on a pin; duplicate when an \
         earlier dot sits on the same point. Dots touching a bus are not judged. Read-only: \
         prune_schematic_junctions removes the unneeded and duplicate dots.",
        json!({
            "type": "object",
            "properties": {
                "schematic": { "type": "string", "description": "Path to one .kicad_sch sheet" }
            },
            "required": ["schematic"]
        }),
        |args, ctx| async move { handle_list_junctions(args, ctx).await }
    )
}

pub(crate) fn prune_tool() -> ToolDef {
    tool!(
        "prune_schematic_junctions",
        "Remove the junction dots on one sheet that KiCad's junction rule does not call for, \
         in one atomic write: dots on a corner, a straight join, a lone wire or a wire end on \
         a pin, and second dots on one point. Each removed dot is reported by UUID with what \
         met there. Dots touching a bus, and dots whose verdict could depend on a symbol whose \
         library pins cannot be resolved, are kept and listed under not_judged. Set dry_run \
         to report the same list without writing. list_schematic_junctions shows every dot.",
        json!({
            "type": "object",
            "properties": {
                "schematic": { "type": "string", "description": "Path to one .kicad_sch sheet" },
                "dry_run": {
                    "type": "boolean",
                    "default": false,
                    "description": "Report what would be removed without writing"
                }
            },
            "required": ["schematic"]
        }),
        |args, ctx| async move { handle_prune_junctions(args, ctx).await }
    )
}

/// One `(junction …)` block as stored in the file.
struct Dot {
    uuid: Option<String>,
    at: (f64, f64),
    /// The block with its leading whitespace, for deletion.
    range: (usize, usize),
}

/// A dot's verdict under the rule.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Verdict {
    Needed,
    Unneeded,
    /// An earlier dot sits on the same point.
    Duplicate,
    /// Touches a bus or bus entry.
    NotJudgedBus,
    /// The rule alone would drop it, but a symbol whose pins cannot be
    /// resolved may have a pin there.
    NotJudgedUnresolvedPins,
}

impl Verdict {
    fn as_str(self) -> &'static str {
        match self {
            Verdict::Needed => "needed",
            Verdict::Unneeded => "unneeded",
            Verdict::Duplicate => "duplicate",
            Verdict::NotJudgedBus | Verdict::NotJudgedUnresolvedPins => "not_judged",
        }
    }

    fn removable(self) -> bool {
        matches!(self, Verdict::Unneeded | Verdict::Duplicate)
    }
}

/// A hierarchical sheet pin, named for reporting.
struct NamedSheetPin {
    sheet: String,
    pin: String,
    at: (f64, f64),
}

fn named_sheet_pins(tree: &SexpNode) -> Vec<NamedSheetPin> {
    let mut pins = Vec::new();
    for sheet in tree.find_all("sheet") {
        let name = sheet
            .find_all("property")
            .into_iter()
            .find(|property| property.get(1).and_then(SexpNode::as_str) == Some("Sheetname"))
            .and_then(|property| property.get(2).and_then(SexpNode::as_str))
            .unwrap_or("")
            .to_owned();
        for pin in sheet.find_all("pin") {
            let Some((x, y, _)) = parse_at(pin) else {
                continue;
            };
            pins.push(NamedSheetPin {
                sheet: name.clone(),
                pin: pin
                    .get(1)
                    .and_then(SexpNode::as_str)
                    .unwrap_or("")
                    .to_owned(),
                at: (x, y),
            });
        }
    }
    pins
}

fn junction_dots(content: &str) -> Vec<Dot> {
    let mut dots = Vec::new();
    for start in find_block_starts(content, "junction") {
        let Some((ws_start, block_end)) = find_block_with_leading_whitespace(content, start) else {
            continue;
        };
        let Ok(node) = parse_sexp(&content[start..block_end]) else {
            continue;
        };
        let Some((x, y, _)) = parse_at(&node) else {
            continue;
        };
        dots.push(Dot {
            uuid: node.find_str("uuid").map(str::to_owned),
            at: (x, y),
            range: (ws_start, block_end),
        });
    }
    dots
}

/// The way a wire leaves `from` toward `to`, in screen terms (KiCad's Y grows
/// downward).
fn toward(from: (f64, f64), to: (f64, f64)) -> &'static str {
    let dx = to.0 - from.0;
    let dy = to.1 - from.1;
    let flat = |delta: f64| delta.abs() <= COINCIDENT_TOLERANCE;
    match (flat(dx), flat(dy)) {
        (false, true) if dx > 0.0 => "right",
        (false, true) => "left",
        (true, false) if dy > 0.0 => "down",
        (true, false) => "up",
        _ if dx > 0.0 && dy > 0.0 => "down-right",
        _ if dx > 0.0 => "up-right",
        _ if dy > 0.0 => "down-left",
        _ => "up-left",
    }
}

fn wire_connection(at: (f64, f64), wire: &Wire, meets: WireMeets) -> Value {
    let (start, end) = ((wire.x1, wire.y1), (wire.x2, wire.y2));
    let (meets, leaves) = match meets {
        WireMeets::Start => ("end", json!(toward(at, end))),
        WireMeets::End => ("end", json!(toward(at, start))),
        WireMeets::Through => ("through", json!([toward(at, start), toward(at, end)])),
    };
    json!({
        "type": "wire",
        "uuid": wire.uuid,
        "meets": meets,
        "leaves": leaves,
        "x1": wire.x1, "y1": wire.y1, "x2": wire.x2, "y2": wire.y2
    })
}

/// Every judged dot on a sheet, with what meets it.
struct Survey {
    pins_resolved: bool,
    entries: Vec<(Dot, Verdict, Value)>,
}

fn survey(content: &str) -> anyhow::Result<Survey> {
    let tree = parse_sexp(content)?;
    let wires = extract_wires(&tree);
    let buses = extract_buses(&tree);
    let labels = extract_all_net_labels(&tree);
    let index = ConnectivityIndex::build(&tree, &wires, &buses, &labels, COINCIDENT_TOLERANCE);
    let pins_resolved = crate::tools::resolved_placed_pins_by_reference(&tree).is_some();
    let sheet_pins = named_sheet_pins(&tree);

    let dots = junction_dots(content);
    let mut entries: Vec<(Dot, Verdict, Value)> = Vec::with_capacity(dots.len());
    for dot in dots {
        let (x, y) = dot.at;
        let site = index.junction_site(x, y);
        // KiCad's cleanup keeps the first dot on a point and drops the rest.
        let first: Option<Option<String>> = entries
            .iter()
            .find(|(earlier, _, _)| {
                points_coincident(x, y, earlier.at.0, earlier.at.1, COINCIDENT_TOLERANCE)
            })
            .map(|(earlier, _, _)| earlier.uuid.clone());
        let verdict = match (&first, site.needed()) {
            (Some(_), _) => Verdict::Duplicate,
            (None, None) => Verdict::NotJudgedBus,
            (None, Some(true)) => Verdict::Needed,
            (None, Some(false)) if !pins_resolved => Verdict::NotJudgedUnresolvedPins,
            (None, Some(false)) => Verdict::Unneeded,
        };
        let detail = describe(&dot, verdict, &site, &index, &sheet_pins, first);
        entries.push((dot, verdict, detail));
    }
    Ok(Survey {
        pins_resolved,
        entries,
    })
}

fn describe(
    dot: &Dot,
    verdict: Verdict,
    site: &JunctionSite<'_>,
    index: &ConnectivityIndex<'_>,
    sheet_pins: &[NamedSheetPin],
    duplicate_of: Option<Option<String>>,
) -> Value {
    let (x, y) = dot.at;
    let mut connections: Vec<Value> = site
        .wires
        .iter()
        .map(|(wire, meets)| wire_connection(dot.at, wire, *meets))
        .collect();
    connections.extend(site.pins.iter().map(|pin| {
        json!({
            "type": "pin",
            "reference": pin.reference,
            "unit": pin.unit,
            "pin": pin.pin.number,
            "pin_name": pin.pin.name,
            "electrical_type": pin.pin.electrical_type
        })
    }));
    connections.extend(
        sheet_pins
            .iter()
            .filter(|pin| points_coincident(x, y, pin.at.0, pin.at.1, COINCIDENT_TOLERANCE))
            .map(|pin| json!({ "type": "sheet_pin", "sheet": pin.sheet, "pin": pin.pin })),
    );

    let mut entry = json!({
        "uuid": dot.uuid,
        "x": x,
        "y": y,
        "verdict": verdict.as_str(),
        "exits": site.exits,
        "connections": connections,
    });
    let labels: Vec<Value> = index
        .labels()
        .iter()
        .filter(|label| label.kind != LabelKind::PowerSymbol)
        .filter(|label| points_coincident(x, y, label.x, label.y, COINCIDENT_TOLERANCE))
        .map(|label| json!({ "text": label.net, "kind": label_kind(label.kind) }))
        .collect();
    if !labels.is_empty() {
        entry["labels"] = json!(labels);
    }
    if index.has_no_connect(x, y) {
        entry["no_connect"] = json!(true);
    }
    if site.bus {
        entry["bus"] = json!(true);
    }
    match verdict {
        Verdict::Duplicate => entry["duplicate_of"] = json!(duplicate_of.flatten()),
        Verdict::NotJudgedBus => entry["not_judged_reason"] = json!("touches a bus or bus entry"),
        Verdict::NotJudgedUnresolvedPins => {
            entry["not_judged_reason"] = json!(
                "fewer than three exits are visible, but a symbol on this sheet has unresolved \
                 library pins that may meet here"
            )
        }
        _ => {}
    }
    entry
}

fn label_kind(kind: LabelKind) -> &'static str {
    match kind {
        LabelKind::NetLabel => "label",
        LabelKind::GlobalLabel => "global_label",
        LabelKind::HierarchicalLabel => "hierarchical_label",
        LabelKind::PowerSymbol => "power_symbol",
    }
}

fn count(survey: &Survey, wanted: &[Verdict]) -> usize {
    survey
        .entries
        .iter()
        .filter(|(_, verdict, _)| wanted.contains(verdict))
        .count()
}

async fn handle_list_junctions(args: &Value, _ctx: &ToolContext) -> anyhow::Result<CallToolResult> {
    let path = get_path(args, "schematic")?;
    let content = read_consistent(&path)?;
    let survey = survey(&content)?;
    Ok(CallToolResult::json(&json!({
        "schematic": path.display().to_string(),
        "junction_count": survey.entries.len(),
        "needed_count": count(&survey, &[Verdict::Needed]),
        "unneeded_count": count(&survey, &[Verdict::Unneeded]),
        "duplicate_count": count(&survey, &[Verdict::Duplicate]),
        "not_judged_count": count(
            &survey,
            &[Verdict::NotJudgedBus, Verdict::NotJudgedUnresolvedPins]
        ),
        "pins_resolved": survey.pins_resolved,
        "rule": RULE,
        "junctions": survey
            .entries
            .iter()
            .map(|(_, _, detail)| detail.clone())
            .collect::<Vec<_>>(),
    })))
}

async fn handle_prune_junctions(
    args: &Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let path = get_path(args, "schematic")?;
    let dry_run = match args.get("dry_run") {
        None | Some(Value::Null) => false,
        Some(Value::Bool(flag)) => *flag,
        Some(_) => return Ok(crate::tools::invalid_arg("dry_run", "must be a boolean")),
    };
    let content = read_consistent(&path)?;
    let survey = survey(&content)?;

    let removed: Vec<&(Dot, Verdict, Value)> = survey
        .entries
        .iter()
        .filter(|(_, verdict, _)| verdict.removable())
        .collect();
    let not_judged: Vec<Value> = survey
        .entries
        .iter()
        .filter(|(_, verdict, _)| {
            matches!(
                verdict,
                Verdict::NotJudgedBus | Verdict::NotJudgedUnresolvedPins
            )
        })
        .map(|(_, _, detail)| detail.clone())
        .collect();
    let report: Vec<Value> = removed
        .iter()
        .map(|(_, _, detail)| detail.clone())
        .collect();
    let removed_uuids: Vec<String> = removed
        .iter()
        .filter_map(|(dot, _, _)| dot.uuid.clone())
        .collect();

    let written = !dry_run && !removed.is_empty();
    if written {
        let edits = removed
            .iter()
            .map(|(dot, _, _)| SexpEdit::delete(dot.range.0, dot.range.1))
            .collect();
        let pruned = apply_edits(content.clone(), edits);
        write_atomic_if_unchanged(&path, &content, &pruned)?;

        // The write is only proof of intent: read the sheet back and confirm
        // exactly the kept dots remain. They are compared as a multiset of
        // UUIDs, because a copy-pasted dot can share its original's UUID, and
        // pruning the copy leaves that UUID on the sheet.
        let committed = read_consistent(&path)?;
        let mut expected: BTreeMap<Option<String>, usize> = BTreeMap::new();
        for (dot, verdict, _) in &survey.entries {
            if !verdict.removable() {
                *expected.entry(dot.uuid.clone()).or_default() += 1;
            }
        }
        let mut found: BTreeMap<Option<String>, usize> = BTreeMap::new();
        for dot in junction_dots(&committed) {
            *found.entry(dot.uuid).or_default() += 1;
        }
        if found != expected {
            let differing: Vec<String> = expected
                .keys()
                .chain(found.keys())
                .collect::<BTreeSet<_>>()
                .into_iter()
                .filter(|uuid| expected.get(*uuid) != found.get(*uuid))
                .map(|uuid| {
                    format!(
                        "{}: expected {}, found {}",
                        uuid.as_deref().unwrap_or("<no uuid>"),
                        expected.get(uuid).copied().unwrap_or(0),
                        found.get(uuid).copied().unwrap_or(0)
                    )
                })
                .collect();
            return Ok(crate::tools::mutation_outcome_uncertain(
                &path,
                "prune_schematic_junctions",
                format!(
                    "the written sheet's junctions are not the kept ones: {}",
                    differing.join("; ")
                ),
            ));
        }
    }

    Ok(CallToolResult::json(&json!({
        "schematic": path.display().to_string(),
        "dry_run": dry_run,
        "written": written,
        "junction_count": survey.entries.len(),
        "pruned_count": removed.len(),
        "pruned_unneeded_count": count(&survey, &[Verdict::Unneeded]),
        "pruned_duplicate_count": count(&survey, &[Verdict::Duplicate]),
        "pruned_uuids": removed_uuids,
        "pruned": report,
        "kept_count": survey.entries.len() - removed.len(),
        "not_judged_count": not_judged.len(),
        "not_judged": not_judged,
        "pins_resolved": survey.pins_resolved,
        "rule": RULE,
    })))
}

#[cfg(test)]
#[path = "sch_junctions_tests.rs"]
mod tests;
