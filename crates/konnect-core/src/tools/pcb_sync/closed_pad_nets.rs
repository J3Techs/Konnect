//! `update_pcb_from_schematic` with `backend: "closed_pad_nets"`: pad net
//! reassignment on an existing, routed board KiCad does not hold.
//!
//! The live backend updates a footprint by sending the whole
//! `FootprintInstance` back, and KiCad 10 re-creates every pad from a message
//! with no field for a pad's fabrication property (`pad_prop_bga`, …), so it
//! refuses such a footprint (`fabrication_property_rebuild_refused`). This
//! backend makes the same pad net changes in the saved file instead. It plans
//! with the live planner's rules — `accept_pad_net_changes` included, with the
//! check that no track, arc or via touches an accepted pad made against the
//! saved copper — and rewrites only each changed pad's `(net …)` clause, so
//! every other byte of the board, fabrication properties included, is kept.
//!
//! Any other change the schematic implies (an added footprint, a new
//! reference, value, DNP or schematic link, a pad gaining or losing its net
//! clause) is refused: those stay with the live backend. The board must be in
//! KiCad 10's names-in-place net format.

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

use konnect_sexp::writer::{apply_edits, find_direct_child_blocks};
use konnect_sexp::{parse_sexp, SexpEdit, SexpNode};
use serde::Serialize;
use serde_json::json;

use super::closed_initial::{file_footprint, quote};
use super::pad_contacts::CopperGeometry;
use super::{
    conflict, netlist_identity, read_saved_design, BoardFootprint, BoardState, Bounds,
    ExportedDesign, PlanStatus, PlannedChange, SavedDesign, SyncDiagnostic, SyncOptions, SyncPlan,
};
use crate::mcp::protocol::{CallToolResult, ToolContent};
use crate::tools::closed_board;
use crate::tools::footprint_fabrication::{footprint_identity, footprint_pad_properties};
use crate::tools::ToolContext;

pub(super) const BACKEND: &str = "closed_pad_nets";
const OPERATION: &str = "update_pcb_from_schematic (closed_pad_nets)";
/// Part of every plan revision, so a revision from another backend can never
/// be applied here.
const REVISION_DOMAIN: &str = "konnect.update_pcb_from_schematic.closed_pad_nets.v1";

/// One pad net change, as the plan reports it.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub(super) struct PadNetEdit {
    reference: String,
    footprint_uuid: String,
    pad: String,
    from: String,
    to: String,
}

/// The `(net …)` clause of one saved pad and what replaces it.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Span {
    start: usize,
    end: usize,
    replacement: String,
}

#[derive(Debug)]
pub(super) struct ClosedPadPlan {
    sync: SyncPlan,
    edits: Vec<PadNetEdit>,
    spans: Vec<Span>,
    revision: String,
    /// Every saved footprint's pad nets, and its pad fabrication-property
    /// count, before the edit, by footprint UUID.
    before: BTreeMap<String, (BTreeMap<String, String>, usize)>,
}

impl ClosedPadPlan {
    fn status(&self) -> &'static str {
        match self.sync.status {
            PlanStatus::Ready => "ready",
            PlanStatus::Noop => "noop",
            PlanStatus::Conflict => "conflict",
        }
    }
}

/// Plan the pad net changes of `design` against the saved board text.
/// `Err` is a refusal before any plan exists (an unreadable board).
pub(super) fn plan(
    original: &str,
    netlist_source: &str,
    design: &ExportedDesign,
    accept: &BTreeSet<String>,
) -> Result<ClosedPadPlan, String> {
    let tree =
        parse_sexp(original).map_err(|e| format!("the saved board could not be parsed: {e}"))?;
    let nodes = tree.find_all("footprint");
    let footprints = nodes
        .iter()
        .enumerate()
        .map(|(index, node)| file_footprint(node, index))
        .collect::<Vec<_>>();

    // Each saved pad's UUID to its footprint's UUID and number, for the
    // copper check; and what every footprint holds before the edit.
    let mut pad_keys = BTreeMap::new();
    let mut before = BTreeMap::new();
    for (node, footprint) in nodes.iter().zip(&footprints) {
        for pad in node.find_all("pad") {
            if let Some(uuid) = pad.find_str("uuid").filter(|u| !u.is_empty()) {
                let number = pad.get(1).and_then(SexpNode::as_str).unwrap_or_default();
                pad_keys.insert(
                    uuid.to_string(),
                    (footprint.kiid.clone(), number.to_string()),
                );
            }
        }
        before.insert(
            footprint.kiid.clone(),
            (
                footprint.pad_nets.clone(),
                footprint_pad_properties(node).len(),
            ),
        );
    }

    let (route, _) = konnect_route::load_board_tree(&tree, &konnect_route::LoadOptions::default())
        .map_err(|e| format!("the saved board's copper could not be read: {e}"))?;
    let zones = tree.find_all("zone");
    let mut routed_nets = BTreeMap::<String, usize>::new();
    let net_name = |id: Option<konnect_route::api::NetId>| id.map(|n| route.net(n).name.clone());
    for net in route
        .tracks
        .iter()
        .map(|t| net_name(t.net))
        .chain(route.arcs.iter().map(|a| net_name(a.net)))
        .chain(route.vias.iter().map(|v| net_name(v.net)))
        .chain(zones.iter().map(|z| {
            z.find("net")
                .and_then(konnect_sexp::net::net_name)
                .map(str::to_string)
        }))
        .flatten()
        .filter(|name| !name.is_empty())
    {
        *routed_nets.entry(net).or_default() += 1;
    }
    let copper = CopperGeometry::from_saved(&route, &pad_keys, zones.len());
    let state = BoardState {
        footprints,
        routed_nets,
        // Only additions are staged from the bounds, and this backend
        // refuses every addition.
        bounds: Bounds {
            min_x: 0.0,
            min_y: 0.0,
            max_x: 0.0,
            max_y: 0.0,
        },
    };
    let options = SyncOptions {
        accept_pad_net_changes: accept.clone(),
        copper: Some(&copper),
    };
    let mut sync = super::plan_sync_with(netlist_source, design, &state, &options);

    let mut refusals = Vec::new();
    if !konnect_sexp::net::names_nets_in_place(&tree) {
        refusals.push(conflict(
            "closed_pad_nets_net_format",
            "The board is not in KiCad 10's names-in-place net format; closed_pad_nets \
             rewrites only name-only (net \"…\") pad clauses. Use backend 'live'."
                .to_string(),
            None,
        ));
    }
    let by_kiid = state
        .footprints
        .iter()
        .map(|f| (f.kiid.as_str(), f))
        .collect::<BTreeMap<_, _>>();
    let mut edits = Vec::new();
    for change in &sync.changes {
        match change {
            PlannedChange::Add { reference, .. } => refusals.push(conflict(
                "closed_pad_nets_add_refused",
                format!(
                    "{reference} is not on the board. closed_pad_nets only reassigns pad nets \
                     of existing footprints; add it with backend 'live'."
                ),
                Some(reference),
            )),
            PlannedChange::Update {
                kiid,
                reference,
                value,
                symbol_path,
                dnp,
                pad_nets,
                ..
            } => {
                let Some(board) = by_kiid.get(kiid.as_str()) else {
                    continue;
                };
                let mut fields = Vec::new();
                if &board.reference != reference {
                    fields.push(format!("reference '{}' -> '{reference}'", board.reference));
                }
                if &board.value != value {
                    fields.push(format!("value '{}' -> '{value}'", board.value));
                }
                if board.symbol_path.as_deref() != Some(symbol_path.as_str()) {
                    fields.push("schematic link".to_string());
                }
                if board.dnp != *dnp {
                    fields.push(format!("DNP {} -> {dnp}", board.dnp));
                }
                if !fields.is_empty() {
                    refusals.push(conflict(
                        "closed_pad_nets_field_change_refused",
                        format!(
                            "{reference} also changes {}. closed_pad_nets only reassigns pad \
                             nets; make that change with backend 'live'.",
                            fields.join(", ")
                        ),
                        Some(reference),
                    ));
                    continue;
                }
                edits.extend(pad_edits(board, pad_nets, &mut refusals));
            }
        }
    }

    let spans = if refusals.is_empty() && sync.diagnostics.is_empty() {
        locate_spans(original, &edits, &mut refusals)
    } else {
        Vec::new()
    };
    if !refusals.is_empty() {
        super::refuse_plan(&mut sync, refusals);
        edits.clear();
    }
    let revision = closed_board::revision(
        REVISION_DOMAIN,
        original,
        &json!({
            // The design sections only: kicad-cli stamps every export with
            // its date, so the whole text differs between dry run and apply.
            "netlist_identity_sha256": closed_board::sha256_hex(&netlist_identity(netlist_source)),
            "accept_pad_net_changes": accept,
            "edits": edits,
        }),
    );
    Ok(ClosedPadPlan {
        sync,
        edits: if spans.is_empty() { Vec::new() } else { edits },
        spans,
        revision,
        before,
    })
}

/// The pads of one board footprint whose net the schematic changes.
fn pad_edits(
    board: &BoardFootprint,
    pad_nets: &BTreeMap<String, String>,
    refusals: &mut Vec<SyncDiagnostic>,
) -> Vec<PadNetEdit> {
    let numbers = pad_nets
        .keys()
        .chain(board.pad_nets.keys())
        .collect::<BTreeSet<_>>();
    let mut edits = Vec::new();
    for number in numbers {
        let from = board.pad_nets.get(number).map(String::as_str).unwrap_or("");
        let to = pad_nets.get(number).map(String::as_str).unwrap_or("");
        if from == to {
            continue;
        }
        if from.is_empty() || to.is_empty() {
            refusals.push(conflict(
                "closed_pad_nets_clause_change_refused",
                format!(
                    "{} pad {number} would go from '{from}' to '{to}': closed_pad_nets only \
                     rewrites an existing pad net clause to another net, never adds or \
                     removes one. Use backend 'live'.",
                    board.reference
                ),
                Some(&board.reference),
            ));
            continue;
        }
        edits.push(PadNetEdit {
            reference: board.reference.clone(),
            footprint_uuid: board.kiid.clone(),
            pad: number.clone(),
            from: from.to_string(),
            to: to.to_string(),
        });
    }
    edits
}

/// Byte ranges of the direct child blocks of the block at `range`, which
/// starts with `(tag`.
fn children(content: &str, range: (usize, usize), tag: &str) -> Vec<(usize, usize)> {
    find_direct_child_blocks(&content[range.0..range.1], tag)
        .into_iter()
        .map(|(s, e)| (range.0 + s, range.0 + e))
        .collect()
}

fn starts_with_tag(content: &str, range: (usize, usize), tag: &str) -> bool {
    content[range.0..range.1]
        .strip_prefix('(')
        .and_then(|rest| rest.strip_prefix(tag))
        .is_some_and(|rest| rest.starts_with(|c: char| c.is_whitespace() || c == ')'))
}

/// The `(net …)` clause of every pad `edits` names, with its replacement.
/// A pad with several copies of its number (exposed-pad arrays) has every
/// copy that carries the old net rewritten. Anything unexpected is a refusal.
fn locate_spans(
    original: &str,
    edits: &[PadNetEdit],
    refusals: &mut Vec<SyncDiagnostic>,
) -> Vec<Span> {
    let mut wanted = BTreeMap::<&str, BTreeMap<&str, &PadNetEdit>>::new();
    for edit in edits {
        wanted
            .entry(edit.footprint_uuid.as_str())
            .or_default()
            .insert(edit.pad.as_str(), edit);
    }
    let mut spans = Vec::new();
    let mut found = BTreeSet::new();
    for footprint in find_direct_child_blocks(original, "kicad_pcb") {
        if !starts_with_tag(original, footprint, "footprint") {
            continue;
        }
        let Ok(node) = parse_sexp(&original[footprint.0..footprint.1]) else {
            continue;
        };
        let Some(pads) = footprint_identity(&node).and_then(|id| wanted.get(id)) else {
            continue;
        };
        for pad in children(original, footprint, "footprint") {
            if !starts_with_tag(original, pad, "pad") {
                continue;
            }
            let Ok(pad_node) = parse_sexp(&original[pad.0..pad.1]) else {
                continue;
            };
            let number = pad_node
                .get(1)
                .and_then(SexpNode::as_str)
                .unwrap_or_default();
            let Some(edit) = pads.get(number) else {
                continue;
            };
            let nets = children(original, pad, "pad")
                .into_iter()
                .filter(|&range| starts_with_tag(original, range, "net"))
                .collect::<Vec<_>>();
            let [net] = nets.as_slice() else {
                refusals.push(conflict(
                    "closed_pad_nets_clause_unreadable",
                    format!(
                        "{} pad {number} has {} (net …) clauses; closed_pad_nets rewrites \
                         exactly one.",
                        edit.reference,
                        nets.len()
                    ),
                    Some(&edit.reference),
                ));
                continue;
            };
            let clause = parse_sexp(&original[net.0..net.1]).ok();
            let name_only = clause
                .as_ref()
                .is_some_and(|c| konnect_sexp::net::net_id(c).is_none());
            let current = clause
                .as_ref()
                .and_then(konnect_sexp::net::net_name)
                .unwrap_or_default();
            if !name_only {
                refusals.push(conflict(
                    "closed_pad_nets_net_format",
                    format!(
                        "{} pad {number}'s net clause is not KiCad 10's (net \"…\") form.",
                        edit.reference
                    ),
                    Some(&edit.reference),
                ));
                continue;
            }
            if current != edit.from {
                // Another copy of the number on a different net: left alone.
                continue;
            }
            found.insert((edit.footprint_uuid.as_str(), edit.pad.as_str()));
            spans.push(Span {
                start: net.0,
                end: net.1,
                replacement: format!("(net {})", quote(&edit.to)),
            });
        }
    }
    for edit in edits {
        if !found.contains(&(edit.footprint_uuid.as_str(), edit.pad.as_str())) {
            refusals.push(conflict(
                "closed_pad_nets_clause_unreadable",
                format!(
                    "{} pad {}'s (net \"{}\") clause was not found in the saved board.",
                    edit.reference, edit.pad, edit.from
                ),
                Some(&edit.reference),
            ));
        }
    }
    spans
}

/// The planned board text.
fn build(original: &str, spans: &[Span]) -> Result<String, String> {
    let candidate = apply_edits(
        original.to_string(),
        spans
            .iter()
            .map(|s| SexpEdit::replace(s.start, s.end, s.replacement.clone()))
            .collect(),
    );
    let ranges = spans
        .iter()
        .map(|s| (s.start, s.end, s.replacement.len()))
        .collect::<Vec<_>>();
    if !closed_board::only_spans_changed(original, &candidate, &ranges) {
        return Err("the planned board changes bytes outside the pad net clauses".to_string());
    }
    Ok(candidate)
}

/// Read back the written board: every footprint's pad nets are what the plan
/// says, and every footprint keeps its pad fabrication properties.
fn verify(persisted: &str, planned: &ClosedPadPlan) -> Result<serde_json::Value, String> {
    let tree = parse_sexp(persisted).map_err(|e| format!("it does not parse: {e}"))?;
    let mut expected = planned.before.clone();
    for edit in &planned.edits {
        if let Some((nets, _)) = expected.get_mut(&edit.footprint_uuid) {
            nets.insert(edit.pad.clone(), edit.to.clone());
        }
    }
    let mut seen = 0usize;
    let mut properties = 0usize;
    for (index, node) in tree.find_all("footprint").iter().enumerate() {
        let footprint = file_footprint(node, index);
        let kept = footprint_pad_properties(node).len();
        match expected.get(&footprint.kiid) {
            Some((nets, count)) if *nets == footprint.pad_nets && *count == kept => {}
            Some(_) => {
                return Err(format!(
                    "{}'s pad nets or pad fabrication properties differ from the plan",
                    footprint.reference
                ))
            }
            None => {
                return Err(format!(
                    "{} was not on the planned board",
                    footprint.reference
                ))
            }
        }
        seen += 1;
        properties += kept;
    }
    if seen != expected.len() {
        return Err(format!(
            "{seen} footprints read back, {} planned",
            expected.len()
        ));
    }
    Ok(json!({
        "footprints_checked": seen,
        "pads_reassigned": planned.edits.len(),
        "pad_fabrication_properties_kept": properties,
    }))
}

fn response(
    planned: &ClosedPadPlan,
    status: &str,
    hierarchy_files: usize,
    closure: &serde_json::Value,
    write: Option<serde_json::Value>,
) -> CallToolResult {
    let applied = write.is_some();
    let edits = planned.edits.len();
    let updated = planned
        .edits
        .iter()
        .map(|e| e.footprint_uuid.as_str())
        .collect::<BTreeSet<_>>()
        .len();
    let value = json!({
        "status": status,
        "backend": BACKEND,
        "plan_revision": planned.revision,
        "closure": closure,
        "coverage": {
            "source": "saved_schematic_hierarchy",
            "hierarchy_files": hierarchy_files,
            "transport": "saved_board_file",
            "atomicity": "single_atomic_file_replace",
            "footprints_updated": { "planned": updated, "applied": if applied { updated } else { 0 } },
            "pads_reassigned": { "planned": edits, "applied": if applied { edits } else { 0 } },
            "board_only_preserved": planned.sync.counts.board_only_preserved,
            "skipped_by_flag": planned.sync.counts.skipped_by_flag,
            "conflicts": planned.sync.counts.conflicts,
        },
        "changes": planned.edits,
        "accepted_pad_net_changes": planned.sync.accepted_pad_net_changes,
        "unused_pad_net_acceptances": planned.sync.unused_pad_net_acceptances,
        "diagnostics": planned.sync.diagnostics,
        "unassigned_footprints": planned.sync.unassigned,
        "refill_needed": edits > 0,
        "write": write,
        "next": if applied {
            "Only the pads' (net …) clauses changed. Reopen the board in KiCad, refill every \
             zone, save, and run DRC."
        } else {
            "Apply with dry_run false, the same accept_pad_net_changes and this plan_revision, \
             with the board still closed in KiCad."
        },
    });
    CallToolResult {
        content: vec![ToolContent::Text {
            text: value.to_string(),
        }],
        is_error: status == "conflict",
    }
}

pub(super) async fn handle(
    ctx: &ToolContext,
    schematic: &Path,
    board: &Path,
    dry_run: bool,
    expected_revision: Option<String>,
    accept: BTreeSet<String>,
) -> anyhow::Result<CallToolResult> {
    let closure = match closed_board::prove_closed(ctx, board, OPERATION).await? {
        Ok(evidence) => evidence,
        Err(refusal) => return Ok(refusal),
    };
    let SavedDesign {
        hierarchy,
        netlist_source,
        design,
    } = match read_saved_design(ctx, schematic).await? {
        Ok(saved) => saved,
        Err(message) => return Ok(super::conflict_result(message)),
    };
    let original = konnect_sexp::read_consistent(board)
        .map_err(|e| anyhow::anyhow!("could not read {}: {e}", board.display()))?;
    let planned = match plan(&original, &netlist_source, &design, &accept) {
        Ok(planned) => planned,
        Err(message) => return Ok(super::conflict_result(message)),
    };
    if dry_run || planned.sync.status != PlanStatus::Ready {
        return Ok(response(
            &planned,
            planned.status(),
            hierarchy.len(),
            &closure,
            None,
        ));
    }
    if expected_revision.as_deref() != Some(planned.revision.as_str()) {
        return Ok(closed_board::stale_revision(board, OPERATION));
    }
    let candidate = match build(&original, &planned.spans) {
        Ok(candidate) => candidate,
        Err(message) => return Ok(super::conflict_result(message)),
    };
    // KiCad must still not hold the board at the moment of the write.
    let closure = match closed_board::prove_closed(ctx, board, OPERATION).await? {
        Ok(evidence) => evidence,
        Err(refusal) => return Ok(refusal),
    };
    match closed_board::write_and_read_back(board, OPERATION, &original, &candidate, |persisted| {
        verify(persisted, &planned)
    }) {
        Ok(write) => Ok(response(
            &planned,
            "applied",
            hierarchy.len(),
            &closure,
            Some(write),
        )),
        Err(refusal) => Ok(refusal),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// U1 with two BGA pads and R1, linked to the schematic; A1 on OLD with a
    /// track ending on it, A2 on SPARE with nothing touching it.
    const BOARD: &str = r#"(kicad_pcb (version 20260206) (generator "pcbnew")
  (layers (0 "F.Cu" signal) (2 "B.Cu" signal))
  (footprint "Pkg:BGA" (layer "F.Cu") (uuid "fp-u1") (at 10 10)
    (property "Reference" "U1" (at 0 0 0) (layer "F.SilkS"))
    (property "Value" "MCU" (at 0 0 0) (layer "F.Fab"))
    (path "/sheet/u1")
    (pad "A1" smd circle (at -1 0) (size 0.4 0.4) (layers "F.Cu") (property pad_prop_bga) (net "OLD") (uuid "pad-a1"))
    (pad "A2" smd circle (at 1 0) (size 0.4 0.4) (layers "F.Cu") (property pad_prop_bga) (net "SPARE") (uuid "pad-a2")))
  (footprint "Pkg:R" (layer "F.Cu") (uuid "fp-r1") (at 30 10)
    (property "Reference" "R1" (at 0 0 0) (layer "F.SilkS"))
    (property "Value" "1k" (at 0 0 0) (layer "F.Fab"))
    (path "/sheet/r1")
    (pad "1" smd rect (at -1 0) (size 0.6 0.6) (layers "F.Cu") (net "OLD") (uuid "pad-r1-1"))
    (pad "2" smd rect (at 1 0) (size 0.6 0.6) (layers "F.Cu") (net "SPARE") (uuid "pad-r1-2")))
  (segment (start 8 10) (end 9 10) (width 0.2) (layer "F.Cu") (net "OLD") (uuid "trk-1"))
  (segment (start 31 10) (end 40 10) (width 0.2) (layer "F.Cu") (net "SPARE") (uuid "trk-2"))
)
"#;

    /// The schematic: U1.A2 and R1.2 move from SPARE to `a2`; the rest as on
    /// the board, U1's value `value`.
    fn design(a2: &str, value: &str) -> (String, ExportedDesign) {
        let netlist = format!(
            r#"(export
  (components
    (comp (ref "U1") (value "{value}") (footprint "Pkg:BGA")
      (sheetpath (names "/") (tstamps "/sheet/")) (tstamps "u1"))
    (comp (ref "R1") (value "1k") (footprint "Pkg:R")
      (sheetpath (names "/") (tstamps "/sheet/")) (tstamps "r1")))
  (nets
    (net (code "1") (name "OLD")
      (node (ref "U1") (pin "A1")) (node (ref "R1") (pin "1")))
    (net (code "2") (name "{a2}")
      (node (ref "U1") (pin "A2")) (node (ref "R1") (pin "2")))))"#
        );
        let design = super::super::parse_exported_netlist(&netlist).expect("netlist parses");
        (netlist, design)
    }

    fn accept(names: &[&str]) -> BTreeSet<String> {
        names.iter().map(|n| n.to_string()).collect()
    }

    #[test]
    fn an_accepted_pad_with_copper_on_it_stays_a_conflict() {
        // SPARE is routed (trk-2), so both pads need acceptance. U1.A2 is
        // untouched; trk-2 ends on R1.2, which stays a conflict naming it.
        let (netlist, design) = design("NEW", "MCU");
        let planned = plan(BOARD, &netlist, &design, &accept(&["U1.A2", "R1.2"])).unwrap();
        assert_eq!(planned.sync.status, PlanStatus::Conflict);
        let messages = planned
            .sync
            .diagnostics
            .iter()
            .map(|d| d.message.clone())
            .collect::<Vec<_>>();
        assert!(
            messages
                .iter()
                .any(|m| m.contains("R1 pad 2") && m.contains("trk-2")),
            "{messages:?}"
        );
        assert!(
            !messages.iter().any(|m| m.contains("U1 pad A2")),
            "{messages:?}"
        );
        assert!(planned.edits.is_empty());
    }

    #[test]
    fn a_bga_pad_keeps_its_fabrication_property_through_the_edit() {
        // Only U1.A2 changes: R1.2 goes to the same new net through a board
        // without trk-2.
        let board = BOARD.replace(
            "  (segment (start 31 10) (end 40 10) (width 0.2) (layer \"F.Cu\") (net \"SPARE\") (uuid \"trk-2\"))\n",
            "",
        );
        let (netlist, design) = design("NEW", "MCU");
        let planned = plan(&board, &netlist, &design, &BTreeSet::new()).unwrap();
        assert_eq!(
            planned.sync.status,
            PlanStatus::Ready,
            "{:?}",
            planned.sync.diagnostics
        );
        assert_eq!(planned.edits.len(), 2);
        let candidate = build(&board, &planned.spans).unwrap();
        assert!(candidate.contains("(property pad_prop_bga) (net \"NEW\") (uuid \"pad-a2\")"));
        assert!(candidate.contains("(property pad_prop_bga) (net \"OLD\") (uuid \"pad-a1\")"));
        assert!(candidate.contains("(net \"NEW\") (uuid \"pad-r1-2\")"));
        assert_eq!(
            candidate.len(),
            board.len() - 2 * "SPARE".len() + 2 * "NEW".len()
        );
        let readback = verify(&candidate, &planned).unwrap();
        assert_eq!(readback["pads_reassigned"], 2);
        assert_eq!(readback["pad_fabrication_properties_kept"], 2);
        // The original text is not what the plan expects after the edit.
        assert!(verify(&board, &planned).is_err());
    }

    #[test]
    fn a_routed_net_needs_acceptance_and_an_untouched_pad() {
        // A1 moves off OLD, which has trk-1 ending on it: refused even when
        // accepted, naming the track.
        let board = BOARD.to_string();
        let netlist = r#"(export
  (components
    (comp (ref "U1") (value "MCU") (footprint "Pkg:BGA")
      (sheetpath (names "/") (tstamps "/sheet/")) (tstamps "u1"))
    (comp (ref "R1") (value "1k") (footprint "Pkg:R")
      (sheetpath (names "/") (tstamps "/sheet/")) (tstamps "r1")))
  (nets
    (net (code "1") (name "OTHER") (node (ref "U1") (pin "A1")))
    (net (code "2") (name "OLD") (node (ref "R1") (pin "1")))
    (net (code "3") (name "SPARE")
      (node (ref "U1") (pin "A2")) (node (ref "R1") (pin "2")))))"#;
        let design = super::super::parse_exported_netlist(netlist).unwrap();
        let planned = plan(&board, netlist, &design, &accept(&["U1.A1"])).unwrap();
        assert_eq!(planned.sync.status, PlanStatus::Conflict);
        assert!(planned
            .sync
            .diagnostics
            .iter()
            .any(|d| d.message.contains("U1 pad A1") && d.message.contains("trk-1")));
    }

    #[test]
    fn any_other_change_is_left_to_the_live_backend() {
        let board = BOARD.replace(
            "  (segment (start 31 10) (end 40 10) (width 0.2) (layer \"F.Cu\") (net \"SPARE\") (uuid \"trk-2\"))\n",
            "",
        );
        let (netlist, design) = design("NEW", "MCU2");
        let planned = plan(&board, &netlist, &design, &BTreeSet::new()).unwrap();
        assert_eq!(planned.sync.status, PlanStatus::Conflict);
        assert!(planned
            .sync
            .diagnostics
            .iter()
            .any(|d| d.code == "closed_pad_nets_field_change_refused"));
        assert!(planned.edits.is_empty() && planned.spans.is_empty());
    }

    #[test]
    fn the_revision_covers_the_board_and_the_acceptances() {
        let (netlist, design) = design("NEW", "MCU");
        let a = plan(BOARD, &netlist, &design, &BTreeSet::new()).unwrap();
        let b = plan(BOARD, &netlist, &design, &accept(&["U1.A2"])).unwrap();
        let c = plan(
            &BOARD.replace("(at 30 10)", "(at 31 10)"),
            &netlist,
            &design,
            &BTreeSet::new(),
        )
        .unwrap();
        assert_ne!(a.revision, b.revision);
        assert_ne!(a.revision, c.revision);
    }

    #[test]
    fn a_re_export_of_the_same_schematic_keeps_the_revision() {
        // kicad-cli dates every export; the apply re-exports the netlist.
        let (netlist, design) = design("NEW", "MCU");
        let dated = |date: &str| {
            netlist.replacen(
                "(export",
                &format!("(export (version \"E\") (design (date \"{date}\"))"),
                1,
            )
        };
        let first = plan(BOARD, &dated("01/11/2026 02:40"), &design, &BTreeSet::new()).unwrap();
        let again = plan(BOARD, &dated("01/11/2026 02:41"), &design, &BTreeSet::new()).unwrap();
        assert_eq!(first.revision, again.revision);
    }
}
