//! `update_pcb_from_schematic` with `backend: "closed_initial"`: a guarded,
//! one-time import of every schematic footprint into a board KiCad does not
//! hold.
//!
//! The live backend builds each footprint from KiCad 10's typed API, which has
//! no field for a pad's fabrication property (`pad_prop_bga`,
//! `pad_prop_heatsink`, …), so it refuses such footprints rather than place a
//! different pad than the library holds. KiCad's own string-parsing create
//! command is a stub in 10.0.4 (`handleParseAndCreateItemsFromString` returns
//! an empty response), so there is no live way to hand KiCad the library text
//! either. This backend instead copies each library footprint's complete text
//! into the saved board, changing only what makes it a board instance, and
//! writes the board once, atomically, while KiCad provably does not hold it.
//!
//! It is an *initial* import only. A board that already carries any circuit
//! footprint, a linked footprint, a netted pad, or any routed copper is
//! refused, so no existing circuit geometry is ever rewritten. Footprints the
//! board already has that are unlinked and unnetted — mounting holes, logos —
//! are kept byte for byte, as is everything else in the file.

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::path::{Path, PathBuf};

use konnect_sexp::writer::{
    find_balanced_block, find_block_starts, find_block_with_leading_whitespace,
    find_direct_child_blocks,
};
use konnect_sexp::{SexpEdit, SexpNode};
use serde::Serialize;
use serde_json::json;
use sha2::{Digest, Sha256};

use super::{
    conflict, footprint_conflict, listed, missing_pads, netlist_identity, pad_missing_conflict,
    parking, plan_sync, read_saved_design, refuse_plan, saved_hierarchy_files, BoardFootprint,
    BoardState, Bounds, PlanStatus, PlannedChange, Point, SavedDesign, SyncDiagnostic, SyncPlan,
};
use crate::mcp::protocol::{CallToolResult, ToolContent};
use crate::tools::footprint_fabrication::footprint_identity;
use crate::tools::pcb_board::{attempt_ipc_write, BoardWrite};
use crate::tools::pcb_components::{
    check_single_board_form, footprint_field_value_span, footprint_reference, indent_block,
    sexp_block_tokens as tokens, sexp_token_value as unquote,
};
use crate::tools::ToolContext;

pub(super) const BACKEND: &str = "closed_initial";
/// Part of every plan revision, so a revision from another backend or an
/// older version of this one can never be applied here.
const REVISION_DOMAIN: &str = "konnect.update_pcb_from_schematic.closed_initial.v1";

/// Library-only header clauses. KiCad's board writer never puts them inside
/// a board footprint, and a nested `(version …)` would raise the version the
/// rest of the board is parsed under, so they are the only clauses removed.
const LIBRARY_HEADER: [&str; 3] = ["version", "generator", "generator_version"];

/// Fields the schematic sets through dedicated clauses, or that KiCad 10
/// drops from footprints (`Footprint`, removed in V9).
const NOT_COPIED_FIELDS: [&str; 3] = ["Reference", "Value", "Footprint"];

/// Board items KiCad writes before its footprints.
const BOARD_HEADER: [&str; 12] = [
    "version",
    "generator",
    "generator_version",
    "general",
    "paper",
    "page",
    "title_block",
    "layers",
    "setup",
    "property",
    "net",
    "net_class",
];

const NATIVE_VALIDATION: &str = "Konnect did not run KiCad. Reopen the board in KiCad 10, \
     confirm it loads without errors, save it there, and run DRC with schematic parity and \
     library parity (lib_footprint_mismatch) before relying on this import.";

pub(super) async fn handle(
    ctx: &ToolContext,
    schematic: &Path,
    board: &Path,
    dry_run: bool,
    expected_revision: Option<String>,
) -> anyhow::Result<CallToolResult> {
    let what = if dry_run {
        "closed-board initial import dry run"
    } else {
        "closed-board initial import apply"
    };
    let closure = match prove_closed(ctx, board, what).await? {
        Ok(evidence) => evidence,
        Err(refusal) => return Ok(refusal),
    };

    // The hierarchy is hashed before and after the export, so a schematic
    // saved while KiCad exported it cannot leave a plan that read two states.
    let hierarchy_before = match saved_hierarchy_files(schematic)
        .map_err(|error| format!("saved schematic preflight failed: {error:#}"))
        .and_then(|files| hash_files(&files))
    {
        Ok(hashes) => hashes,
        Err(message) => return Ok(refusal_result(message, Some(&closure))),
    };
    let saved = match read_saved_design(ctx, schematic).await? {
        Ok(saved) => saved,
        Err(message) => return Ok(refusal_result(message, Some(&closure))),
    };
    let hierarchy = match hash_files(&saved.hierarchy) {
        Ok(hashes) if hashes == hierarchy_before => hashes,
        Ok(_) => {
            return Ok(refusal_result(
                "the saved schematic hierarchy changed while it was being exported; retry"
                    .to_string(),
                Some(&closure),
            ))
        }
        Err(message) => return Ok(refusal_result(message, Some(&closure))),
    };

    let mut planned = match plan(board, &saved, hierarchy) {
        Ok(planned) => planned,
        Err(error) => {
            return Ok(refusal_result(
                format!("closed-board planning failed: {error:#}"),
                Some(&closure),
            ))
        }
    };

    if dry_run || planned.plan.status != PlanStatus::Ready {
        let status = match planned.plan.status {
            PlanStatus::Ready => "ready",
            PlanStatus::Noop => "noop",
            PlanStatus::Conflict => "conflict",
        };
        return Ok(response(&planned, status, &closure, None));
    }
    if expected_revision.as_deref() != Some(planned.plan.plan_revision.as_str()) {
        refuse_plan(
            &mut planned.plan,
            vec![conflict(
                "stale_plan_revision",
                "The saved board, schematic hierarchy, libraries or this backend changed; \
                 rerun the closed_initial dry run and apply its new plan revision."
                    .to_string(),
                None,
            )],
        );
        return Ok(response(&planned, "conflict", &closure, None));
    }

    // Everything the plan read, read again, immediately before the write.
    if let Err(message) = planned.inputs.still_current() {
        refuse_plan(
            &mut planned.plan,
            vec![conflict("stale_plan_inputs", message, None)],
        );
        return Ok(response(&planned, "conflict", &closure, None));
    }
    let closure = match prove_closed(ctx, board, what).await? {
        Ok(evidence) => evidence,
        Err(refusal) => return Ok(refusal),
    };
    let target = planned
        .planned
        .clone()
        .expect("a ready plan carries its board text");
    Ok(write_and_read_back(board, &planned, &target, &closure))
}

/// Prove KiCad does not hold `board` before anything is planned or written.
///
/// The live write gate decides: a KiCad holding the board answers, and that is
/// a refusal here; a KiCad that answers with an error or with an uncertain
/// outcome is refused with that answer preserved. Only the two verdicts that
/// unlock a closed-board edit pass — IPC never reached, or KiCad proves the
/// board is not open — and then KiCad's own sibling lock must be absent too.
async fn prove_closed(
    ctx: &ToolContext,
    board: &Path,
    what: &str,
) -> anyhow::Result<Result<serde_json::Value, CallToolResult>> {
    let reason = match attempt_ipc_write(ctx, board, what, |_| Ok(())).await? {
        BoardWrite::Ipc(()) => {
            return Ok(Err(refusal_result(
                "KiCad holds this board open. The closed_initial backend writes the saved \
                 file and requires the board closed in KiCad, even for a dry run; save and \
                 close it in the PCB editor and retry. The board file was not modified."
                    .to_string(),
                None,
            )))
        }
        BoardWrite::Refused(result) => {
            if matches!(
                crate::mcp::error::extract_error_kind(&result).as_deref(),
                Some("ipc_outcome_unknown" | "ipc_batch_recovered" | "unsafe_file_fallback")
            ) {
                return Ok(Err(result));
            }
            let message = result
                .content
                .into_iter()
                .find_map(|content| match content {
                    ToolContent::Text { text } => Some(text),
                    _ => None,
                })
                .unwrap_or_else(|| "KiCad refused the closure check".to_string());
            return Ok(Err(refusal_result(
                format!("closure not proven: {message}"),
                None,
            )));
        }
        BoardWrite::File(reason) => reason,
    };
    match crate::tools::live_board::editor_lock(board) {
        crate::tools::live_board::EditorLock::Absent => {}
        crate::tools::live_board::EditorLock::Present(lock) => {
            return Ok(Err(refusal_result(
                format!(
                    "KiCad's sibling lock '{}' is present, so an editor may still hold this \
                     board. The board file was not modified.",
                    lock.display()
                ),
                None,
            )))
        }
        crate::tools::live_board::EditorLock::Unreadable(lock, error) => {
            return Ok(Err(refusal_result(
                format!(
                    "KiCad's sibling lock '{}' could not be inspected ({error}). The board \
                     file was not modified.",
                    lock.display()
                ),
                None,
            )))
        }
    }
    Ok(Ok(json!({
        "proven_closed": true,
        "evidence": reason.evidence(),
        "kicad_sibling_lock": "absent",
    })))
}

// ─── Inputs and revision ─────────────────────────────────────────────────────

fn sha256_hex(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

fn hash_files(files: &[PathBuf]) -> Result<Vec<(PathBuf, String)>, String> {
    files
        .iter()
        .map(|file| {
            std::fs::read(file)
                .map(|bytes| (file.clone(), sha256_hex(&bytes)))
                .map_err(|error| format!("cannot read {}: {error}", file.display()))
        })
        .collect()
}

/// Everything a plan read, by content hash.
#[derive(Debug, Clone, Serialize)]
struct InputSnapshot {
    board: PathBuf,
    board_sha256: String,
    hierarchy: Vec<(PathBuf, String)>,
    /// Library footprint id to the resolved file and its hash.
    libraries: BTreeMap<String, (PathBuf, String)>,
    /// The exported netlist's design sections, not its header: an export
    /// stamps its date and tool, which must not change the revision.
    netlist_identity_sha256: String,
    konnect_version: &'static str,
}

impl InputSnapshot {
    fn digest(&self) -> [u8; 32] {
        let mut hasher = Sha256::new();
        hasher.update(REVISION_DOMAIN.as_bytes());
        hasher.update(serde_json::to_vec(self).expect("inputs serialize"));
        hasher.finalize().into()
    }

    /// Re-read every input and refuse when one changed.
    fn still_current(&self) -> Result<(), String> {
        let board = std::fs::read(&self.board)
            .map_err(|error| format!("cannot re-read {}: {error}", self.board.display()))?;
        if sha256_hex(&board) != self.board_sha256 {
            return Err(format!(
                "{} changed after the plan was made; rerun the dry run",
                self.board.display()
            ));
        }
        for (file, hash) in &self.hierarchy {
            let bytes = std::fs::read(file)
                .map_err(|error| format!("cannot re-read {}: {error}", file.display()))?;
            if &sha256_hex(&bytes) != hash {
                return Err(format!(
                    "schematic {} changed after the plan was made; rerun the dry run",
                    file.display()
                ));
            }
        }
        for (footprint_id, (file, hash)) in &self.libraries {
            let bytes = std::fs::read(file).map_err(|error| {
                format!(
                    "cannot re-read {footprint_id} at {}: {error}",
                    file.display()
                )
            })?;
            if &sha256_hex(&bytes) != hash {
                return Err(format!(
                    "library footprint {footprint_id} ({}) changed after the plan was made; \
                     rerun the dry run",
                    file.display()
                ));
            }
        }
        Ok(())
    }

    fn json(&self) -> serde_json::Value {
        json!({
            "board": { "path": self.board.display().to_string(), "sha256": self.board_sha256 },
            "hierarchy": self.hierarchy.iter().map(|(path, sha256)| json!({
                "path": path.display().to_string(), "sha256": sha256
            })).collect::<Vec<_>>(),
            "libraries": self.libraries.iter().map(|(id, (path, sha256))| json!({
                "footprint_id": id, "path": path.display().to_string(), "sha256": sha256
            })).collect::<Vec<_>>(),
            "netlist_identity_sha256": self.netlist_identity_sha256,
            "konnect_version": self.konnect_version,
        })
    }
}

/// A complete plan: the immutable board text it would write and how it got
/// there. `planned` is `None` unless the plan is ready.
struct ClosedPlan {
    plan: SyncPlan,
    hierarchy_files: usize,
    inputs: InputSnapshot,
    original: String,
    planned: Option<String>,
    expected: Vec<ExpectedInstance>,
    templates: BTreeMap<String, Template>,
    new_nets: BTreeMap<String, i64>,
    report: serde_json::Value,
}

// ─── Planning ────────────────────────────────────────────────────────────────

fn plan(
    board_path: &Path,
    saved: &SavedDesign,
    hierarchy: Vec<(PathBuf, String)>,
) -> anyhow::Result<ClosedPlan> {
    let original = konnect_sexp::read_consistent(board_path)?;
    let mut inputs = InputSnapshot {
        board: board_path.to_path_buf(),
        board_sha256: sha256_hex(original.as_bytes()),
        hierarchy,
        libraries: BTreeMap::new(),
        netlist_identity_sha256: sha256_hex(&netlist_identity(&saved.netlist_source)),
        konnect_version: env!("CARGO_PKG_VERSION"),
    };
    let mut closed = ClosedPlan {
        plan: SyncPlan {
            status: PlanStatus::Conflict,
            plan_revision: String::new(),
            counts: Default::default(),
            changes: Vec::new(),
            diagnostics: Vec::new(),
            unassigned: Vec::new(),
            staging: None,
            parking: None,
            accepted_pad_net_changes: Vec::new(),
            unused_pad_net_acceptances: Vec::new(),
        },
        hierarchy_files: saved.hierarchy.len(),
        inputs: inputs.clone(),
        original: original.clone(),
        planned: None,
        expected: Vec::new(),
        templates: BTreeMap::new(),
        new_nets: BTreeMap::new(),
        report: json!({}),
    };

    if let Err(reason) = check_single_board_form(&original) {
        refuse_plan(
            &mut closed.plan,
            vec![conflict(
                "board_unreadable",
                format!("the saved board is not one complete (kicad_pcb …) form: {reason}"),
                None,
            )],
        );
        return Ok(finish(closed));
    }
    let inventory = match Inventory::read(&original) {
        Ok(inventory) => inventory,
        Err(reason) => {
            refuse_plan(
                &mut closed.plan,
                vec![conflict("board_unreadable", reason, None)],
            );
            return Ok(finish(closed));
        }
    };

    let mut plan = plan_sync(&saved.netlist_source, &saved.design, &inventory.state);
    let mut refusals = initial_only_refusals(&inventory, saved);

    // Library footprints, read once each, with every part that needs them.
    let mut needed: BTreeMap<String, Vec<String>> = BTreeMap::new();
    for change in &plan.changes {
        if let PlannedChange::Add {
            footprint_id,
            reference,
            ..
        } = change
        {
            needed
                .entry(footprint_id.clone())
                .or_default()
                .push(reference.clone());
        }
    }
    let board_version = inventory.version.unwrap_or(0);
    let mut templates = BTreeMap::new();
    for (footprint_id, references) in &needed {
        match read_template(footprint_id, board_path, board_version) {
            Ok(template) => {
                inputs.libraries.insert(
                    footprint_id.clone(),
                    (template.path.clone(), template.sha256.clone()),
                );
                templates.insert(footprint_id.clone(), template);
            }
            Err((code, reason)) => refusals.push(footprint_conflict(
                code,
                format!(
                    "{footprint_id} cannot be imported (needed by {}): {reason}",
                    listed(references)
                ),
                footprint_id,
                references.clone(),
            )),
        }
    }
    for change in &plan.changes {
        let PlannedChange::Add {
            reference,
            footprint_id,
            pad_nets,
            ..
        } = change
        else {
            continue;
        };
        if pad_nets.contains_key("") {
            refusals.push(conflict(
                "unnumbered_pin_connected",
                format!("the schematic connects an unnumbered pin of {reference}"),
                Some(reference),
            ));
        }
        let Some(template) = templates.get(footprint_id) else {
            continue;
        };
        let missing = missing_pads(pad_nets, &template.pad_numbers);
        if !missing.is_empty() {
            refusals.push(pad_missing_conflict(reference, footprint_id, &missing));
        }
    }
    if !refusals.is_empty() {
        refuse_plan(&mut plan, refusals);
    }
    closed.plan = plan;
    closed.inputs = inputs;
    if closed.plan.status != PlanStatus::Ready {
        closed.report = json!({
            "preserved_board_only": inventory.preserved_json(),
            "board_routing": inventory.routed,
        });
        return Ok(finish(closed));
    }

    match build(&original, &inventory, saved, &mut closed, templates) {
        Ok(()) => {}
        Err(diagnostics) => refuse_plan(&mut closed.plan, diagnostics),
    }
    Ok(finish(closed))
}

/// The revision: the inputs, what would be written, and what was concluded.
fn finish(mut closed: ClosedPlan) -> ClosedPlan {
    if closed.plan.status != PlanStatus::Ready {
        closed.planned = None;
        closed.expected.clear();
    }
    let mut hasher = Sha256::new();
    hasher.update(closed.inputs.digest());
    hasher.update(
        closed
            .planned
            .as_deref()
            .map(|text| Sha256::digest(text.as_bytes()).to_vec())
            .unwrap_or_default(),
    );
    hasher.update(serde_json::to_vec(&closed.plan.changes).expect("changes serialize"));
    hasher.update(serde_json::to_vec(&closed.plan.diagnostics).expect("diagnostics serialize"));
    closed.plan.plan_revision = format!("{:x}", hasher.finalize());
    closed
}

/// Refusals that keep this backend an initial import.
fn initial_only_refusals(inventory: &Inventory, saved: &SavedDesign) -> Vec<SyncDiagnostic> {
    let mut refusals = Vec::new();
    if inventory.version.is_none() {
        refusals.push(conflict(
            "board_format_unknown",
            "the saved board has no (version …), so the format its footprints are parsed \
             under is unknown"
                .to_string(),
            None,
        ));
    }
    let routed = inventory.routed.values().sum::<usize>();
    if routed > 0 {
        refusals.push(conflict(
            "board_has_routing",
            format!(
                "the board already has routed copper or zones ({}); closed_initial imports \
                 only into a board with none",
                inventory
                    .routed
                    .iter()
                    .filter(|(_, count)| **count > 0)
                    .map(|(kind, count)| format!("{count} {kind}"))
                    .collect::<Vec<_>>()
                    .join(", ")
            ),
            None,
        ));
    }
    let exported = saved
        .design
        .components
        .iter()
        .map(|component| component.reference.as_str())
        .chain(
            saved
                .design
                .skipped
                .iter()
                .map(|part| part.reference.as_str()),
        )
        .chain(
            saved
                .design
                .unassigned
                .iter()
                .map(|part| part.reference.as_str()),
        )
        .collect::<HashSet<_>>();
    let mut linked = Vec::new();
    let mut netted = Vec::new();
    let mut corresponding = Vec::new();
    for footprint in &inventory.state.footprints {
        if footprint.symbol_path.is_some() {
            linked.push(footprint.reference.clone());
        }
        if !footprint.pad_nets.is_empty() {
            netted.push(footprint.reference.clone());
        }
        if exported.contains(footprint.reference.as_str()) {
            corresponding.push(footprint.reference.clone());
        }
    }
    for (code, references, what) in [
        (
            "linked_circuit_footprint_present",
            linked,
            "footprints already linked to schematic symbols",
        ),
        (
            "netted_pad_present",
            netted,
            "footprints with pads that already carry nets",
        ),
        (
            "circuit_footprint_present",
            corresponding,
            "footprints whose references the schematic already exports",
        ),
    ] {
        if references.is_empty() {
            continue;
        }
        refusals.push(SyncDiagnostic {
            code: code.to_string(),
            message: format!(
                "the board already has {what}: {}. closed_initial is an initial import only \
                 and never rewrites existing circuit footprints",
                listed(&references)
            ),
            reference: match references.as_slice() {
                [only] => Some(only.clone()),
                _ => None,
            },
            references,
            footprint_id: None,
            pad_detail: None,
        });
    }
    refusals
}

// ─── The saved board ─────────────────────────────────────────────────────────

/// What the saved board holds, read from its text.
struct Inventory {
    version: Option<i64>,
    state: BoardState,
    /// Existing footprints, all unlinked and unnetted once the plan is ready.
    preserved: Vec<(String, Option<String>, String, bool)>,
    routed: BTreeMap<&'static str, usize>,
    /// `(net N "name")` declarations of a pre-10 board; empty for KiCad 10,
    /// whose pads name their nets and whose net table is built on load.
    legacy_nets: BTreeMap<String, i64>,
    last_net_end: Option<usize>,
    /// Where new footprints go: after the last footprint, else after the
    /// header items, as KiCad orders a board.
    insert_after: Option<usize>,
    ids: HashSet<String>,
    duplicate_ids: usize,
    staging_basis: &'static str,
    not_measured: BTreeMap<&'static str, usize>,
}

impl Inventory {
    fn read(content: &str) -> Result<Self, String> {
        let root = konnect_sexp::parse_sexp(content)
            .map_err(|error| format!("the saved board cannot be parsed: {error}"))?;
        let version = super::super::pcb_components::sexp_format_version(&root);
        let mut footprints = Vec::new();
        let mut preserved = Vec::new();
        let mut routed = BTreeMap::from([("tracks", 0), ("arcs", 0), ("vias", 0), ("zones", 0)]);
        let mut legacy_nets = BTreeMap::new();
        let (mut last_net_end, mut last_child_end) = (None, None);
        let (mut last_footprint_end, mut header_end, mut past_header) = (None, None, false);
        for (start, end) in find_direct_child_blocks(content, "kicad_pcb") {
            last_child_end = Some(end);
            let node = konnect_sexp::parse_sexp(&content[start..end])
                .map_err(|error| format!("a board item cannot be parsed: {error}"))?;
            if node.head() == Some("footprint") {
                last_footprint_end = Some(end);
            }
            if !past_header && BOARD_HEADER.contains(&node.head().unwrap_or_default()) {
                header_end = Some(end);
            } else {
                past_header = true;
            }
            match node.head() {
                Some("segment") => *routed.get_mut("tracks").unwrap() += 1,
                Some("arc") => *routed.get_mut("arcs").unwrap() += 1,
                Some("via") => *routed.get_mut("vias").unwrap() += 1,
                Some("zone") => *routed.get_mut("zones").unwrap() += 1,
                Some("net") => {
                    last_net_end = Some(end);
                    if let (Some(code), Some(name)) = (
                        konnect_sexp::net::net_id(&node).and_then(|id| id.parse().ok()),
                        konnect_sexp::net::net_name(&node),
                    ) {
                        legacy_nets.insert(name.to_string(), code);
                    }
                }
                Some("footprint") => {
                    let footprint = file_footprint(&node, footprints.len());
                    preserved.push((
                        footprint.reference.clone(),
                        footprint_identity(&node).map(str::to_string),
                        footprint.footprint_id.clone(),
                        footprint.not_in_schematic,
                    ));
                    footprints.push(footprint);
                }
                _ => {}
            }
        }
        let (ids, duplicate_ids) = identity_values(content);
        let measured = konnect_sexp::bounds::board_item_bounds(&root);
        let (bounds, staging_basis) = match measured.union() {
            Some((min_x, min_y, max_x, max_y)) => (
                Bounds {
                    min_x,
                    min_y,
                    max_x,
                    max_y,
                },
                if measured.not_measured.is_empty() && measured.skipped == 0 {
                    "saved_file_geometry"
                } else {
                    "partial_saved_file_geometry"
                },
            ),
            None => (
                Bounds {
                    min_x: 0.0,
                    min_y: 0.0,
                    max_x: 0.0,
                    max_y: 0.0,
                },
                "no_measured_geometry",
            ),
        };
        Ok(Self {
            version,
            state: BoardState {
                footprints,
                routed_nets: BTreeMap::new(),
                bounds,
            },
            preserved,
            routed,
            legacy_nets,
            last_net_end,
            insert_after: last_footprint_end.or(header_end).or(last_child_end),
            ids,
            duplicate_ids,
            staging_basis,
            not_measured: measured.not_measured,
        })
    }

    fn preserved_json(&self) -> Vec<serde_json::Value> {
        self.preserved
            .iter()
            .map(|(reference, uuid, footprint_id, board_only)| {
                json!({
                    "reference": reference,
                    "uuid": uuid,
                    "footprint_id": footprint_id,
                    "board_only_attribute": board_only,
                })
            })
            .collect()
    }
}

/// One saved footprint in the planner's terms.
fn file_footprint(node: &SexpNode, index: usize) -> BoardFootprint {
    let at = node.find("at");
    let attr = node
        .find("attr")
        .and_then(SexpNode::children)
        .unwrap_or_default()
        .iter()
        .filter_map(SexpNode::as_str)
        .collect::<Vec<_>>();
    let mut pad_nets = BTreeMap::new();
    let mut pad_numbers = BTreeSet::new();
    for pad in node.find_all("pad") {
        let number = pad
            .get(1)
            .and_then(SexpNode::as_str)
            .unwrap_or_default()
            .to_string();
        if let Some(net) = pad
            .find("net")
            .and_then(konnect_sexp::net::net_name)
            .filter(|name| !name.is_empty())
        {
            pad_nets.insert(number.clone(), net.to_string());
        }
        pad_numbers.insert(number);
    }
    BoardFootprint {
        kiid: footprint_identity(node)
            .map(str::to_string)
            .unwrap_or_else(|| format!("<saved footprint {index} without uuid>")),
        reference: footprint_reference(node).unwrap_or_default(),
        value: field_value(node, "Value", "value").unwrap_or_default(),
        footprint_id: node
            .get(1)
            .and_then(SexpNode::as_str)
            .unwrap_or_default()
            .to_string(),
        symbol_path: node
            .find_str("path")
            .filter(|path| !path.is_empty())
            .map(str::to_string),
        pad_nets,
        pad_numbers,
        position: Point {
            x: at.and_then(|at| at.get_f64(1)).unwrap_or(0.0),
            y: at.and_then(|at| at.get_f64(2)).unwrap_or(0.0),
        },
        rotation: at.and_then(|at| at.get_f64(3)).unwrap_or(0.0),
        layer: node.find_str("layer").unwrap_or("F.Cu").to_string(),
        locked: node.find_str("locked") == Some("yes")
            || node
                .children()
                .unwrap_or_default()
                .iter()
                .any(|child| child.as_str() == Some("locked")),
        dnp: attr.contains(&"dnp"),
        not_in_schematic: attr.contains(&"board_only"),
    }
}

/// A footprint field's text: `(property "Name" …)`, or `(fp_text kind …)` in
/// older files.
fn field_value(footprint: &SexpNode, property: &str, fp_text: &str) -> Option<String> {
    footprint
        .find_all("property")
        .into_iter()
        .find(|node| node.get(1).and_then(SexpNode::as_str) == Some(property))
        .or_else(|| {
            footprint
                .find_all("fp_text")
                .into_iter()
                .find(|node| node.get(1).and_then(SexpNode::as_str) == Some(fp_text))
        })
        .and_then(|node| node.get(2))
        .and_then(SexpNode::as_str)
        .map(str::to_string)
}

/// Every `(uuid …)`/`(tstamp …)` value in `content`, and how many repeat.
fn identity_values(content: &str) -> (HashSet<String>, usize) {
    let mut ids = HashSet::new();
    let mut duplicates = 0;
    for tag in ["uuid", "tstamp"] {
        for start in find_block_starts(content, tag) {
            let Some((block_start, block_end)) = find_balanced_block(content, start) else {
                continue;
            };
            let block = &content[block_start..block_end];
            if let Some(&(value_start, value_end)) = tokens(block).get(1) {
                if !ids.insert(unquote(&block[value_start..value_end])) {
                    duplicates += 1;
                }
            }
        }
    }
    (ids, duplicates)
}

// ─── Text helpers ────────────────────────────────────────────────────────────

/// `value` as a KiCad string literal.
fn quote(value: &str) -> String {
    let mut out = String::with_capacity(value.len() + 2);
    out.push('"');
    for character in value.chars() {
        match character {
            '\\' => out.push_str("\\\\"),
            '"' => out.push_str("\\\""),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            other => out.push(other),
        }
    }
    out.push('"');
    out
}

/// A millimetre value as KiCad writes it: at most six decimals, no trailing
/// zeros.
fn mm(value: f64) -> String {
    let text = format!("{value:.6}");
    let text = text.trim_end_matches('0').trim_end_matches('.');
    if text == "-0" {
        "0".to_string()
    } else {
        text.to_string()
    }
}

/// The whitespace that starts the line `offset` is on.
fn line_indent(text: &str, offset: usize) -> &str {
    let line_start = text[..offset].rfind('\n').map_or(0, |index| index + 1);
    let indent_end = text[line_start..]
        .find(|character: char| character != ' ' && character != '\t')
        .map_or(text.len(), |index| line_start + index);
    &text[line_start..indent_end.min(offset)]
}

// ─── Library footprints ──────────────────────────────────────────────────────

/// A library footprint read for embedding, validated once.
struct Template {
    path: PathBuf,
    sha256: String,
    source: String,
    root: SexpNode,
    /// Direct children of the root: byte range and head.
    children: Vec<(usize, usize, String)>,
    pad_numbers: BTreeSet<String>,
    /// Every `(uuid …)`/`(tstamp …)` below the root's own: the value token's
    /// byte span and its value.
    child_ids: Vec<(usize, usize, String)>,
    /// The root's own identity token, if the library carries one.
    root_id: Option<(usize, usize)>,
    /// Every group member reference: the token's byte span and its value.
    members: Vec<(usize, usize, String)>,
    eol: &'static str,
    indent: String,
}

impl Template {
    fn child(&self, head: &str) -> Option<(usize, usize)> {
        self.children
            .iter()
            .find(|(_, _, tag)| tag == head)
            .map(|(start, end, _)| (*start, *end))
    }

    /// The value span of the direct child field `name`; see
    /// [`footprint_field_value_span`].
    fn field_value_span(&self, name: &str) -> Option<(usize, usize)> {
        footprint_field_value_span(&self.source, name)
    }
}

/// Resolve, read and validate one library footprint. The three resolution
/// codes match `update_footprints_from_library`; the rest name why the text
/// cannot be embedded unchanged.
fn read_template(
    footprint_id: &str,
    board_path: &Path,
    board_version: i64,
) -> Result<Template, (&'static str, String)> {
    let path = super::super::pcb_components::resolve_footprint_file(footprint_id, board_path)
        .map_err(|error| ("footprint_library_resolution_failed", format!("{error:#}")))?;
    let bytes = std::fs::read(&path).map_err(|error| {
        (
            "footprint_library_read_failed",
            format!("failed to read {}: {error}", path.display()),
        )
    })?;
    let source = String::from_utf8(bytes.clone()).map_err(|_| {
        (
            "footprint_library_read_failed",
            format!("{} is not UTF-8", path.display()),
        )
    })?;
    let mut template = template_from_source(source, board_version)?;
    template.sha256 = sha256_hex(&bytes);
    template.path = path;
    Ok(template)
}

fn template_from_source(
    source: String,
    board_version: i64,
) -> Result<Template, (&'static str, String)> {
    let unusable = |reason: String| ("unsupported_library_footprint", reason);
    let root = konnect_sexp::parse_sexp(&source)
        .map_err(|error| unusable(format!("invalid footprint S-expression: {error}")))?;
    if root.head() != Some("footprint") {
        return Err(unusable(
            "the file is not a (footprint …) definition".to_string(),
        ));
    }
    let root_start = find_block_starts(&source, "footprint")
        .first()
        .copied()
        .ok_or_else(|| unusable("no (footprint …) block".to_string()))?;
    if source[..root_start].trim() != "" {
        return Err(unusable("content precedes the footprint".to_string()));
    }
    if multi_line_string(&source) {
        return Err(unusable(
            "a quoted string spans lines, which re-indenting into the board would change"
                .to_string(),
        ));
    }
    let children = find_direct_child_blocks(&source, "footprint")
        .into_iter()
        .map(|(start, end)| {
            let head = tokens(&source[start..end])
                .first()
                .map(|&(s, e)| source[start + s..start + e].to_string())
                .unwrap_or_default();
            (start, end, head)
        })
        .collect::<Vec<_>>();
    if root.find_str("layer") != Some("F.Cu") {
        return Err(unusable(format!(
            "the library footprint is on '{}', and an initial import places every part on F.Cu",
            root.find_str("layer").unwrap_or("no layer")
        )));
    }
    for clause in ["path", "sheetname", "sheetfile"] {
        if root.find(clause).is_some() {
            return Err(unusable(format!(
                "the library footprint already carries a board-instance ({clause} …)"
            )));
        }
    }
    let library_version = super::super::pcb_components::sexp_format_version(&root).unwrap_or(0);
    let reinterpreted = format_reinterpretations(&root, &source, library_version, board_version);
    if !reinterpreted.is_empty() {
        return Err((
            "library_format_reinterpreted",
            format!(
                "library format {library_version} parsed at the board's {board_version} would \
                 change meaning: {}. Open and save the footprint in KiCad 10's footprint editor",
                reinterpreted.join("; ")
            ),
        ));
    }

    let mut pad_numbers = BTreeSet::new();
    for pad in root.find_all("pad") {
        let number = pad.get(1).and_then(SexpNode::as_str).unwrap_or_default();
        if pad.find("net").is_some() {
            return Err(unusable(format!(
                "library pad '{number}' already carries a net"
            )));
        }
        pad_numbers.insert(number.to_string());
    }

    // Identities. The root's own is replaced by the instance UUID; every other
    // one is remapped per instance, and must be unique and well formed for
    // the group references that name them to stay meaningful.
    let root_id = children
        .iter()
        .find(|(_, _, head)| head == "uuid" || head == "tstamp")
        .and_then(|(start, end, _)| {
            tokens(&source[*start..*end])
                .get(1)
                .map(|&(s, e)| (start + s, start + e))
        });
    let mut child_ids = Vec::new();
    let mut seen = HashSet::new();
    for tag in ["uuid", "tstamp"] {
        for start in find_block_starts(&source, tag) {
            let (block_start, block_end) = find_balanced_block(&source, start)
                .ok_or_else(|| unusable(format!("an unbalanced ({tag} …)")))?;
            let Some(&(value_start, value_end)) = tokens(&source[block_start..block_end]).get(1)
            else {
                return Err((
                    "invalid_library_identity",
                    format!("a ({tag}) clause has no value"),
                ));
            };
            let span = (block_start + value_start, block_start + value_end);
            if Some(span) == root_id {
                continue;
            }
            let value = unquote(&source[span.0..span.1]);
            if !valid_identity(&value) {
                return Err((
                    "invalid_library_identity",
                    format!("'{value}' is neither a UUID nor a legacy timestamp"),
                ));
            }
            if !seen.insert(value.clone()) {
                return Err((
                    "duplicate_library_identity",
                    format!("identity '{value}' appears more than once in the footprint"),
                ));
            }
            child_ids.push((span.0, span.1, value));
        }
    }
    child_ids.sort_by_key(|(start, _, _)| *start);
    let mut members = Vec::new();
    for start in find_block_starts(&source, "members") {
        let (block_start, block_end) = find_balanced_block(&source, start)
            .ok_or_else(|| unusable("an unbalanced (members …)".to_string()))?;
        let block = &source[block_start..block_end];
        for &(token_start, token_end) in tokens(block).iter().skip(1) {
            let value = unquote(&block[token_start..token_end]);
            if !seen.contains(&value) {
                return Err((
                    "dangling_group_member",
                    format!("a group names member '{value}', which is not in the footprint"),
                ));
            }
            members.push((block_start + token_start, block_start + token_end, value));
        }
    }

    let indent = children
        .iter()
        .find(|(_, _, head)| head == "layer")
        .map(|(start, _, _)| line_indent(&source, *start).to_string())
        .filter(|indent| !indent.is_empty())
        .unwrap_or_else(|| "\t".to_string());
    let eol = if source.contains("\r\n") {
        "\r\n"
    } else {
        "\n"
    };
    Ok(Template {
        path: PathBuf::new(),
        sha256: sha256_hex(source.as_bytes()),
        root,
        children,
        pad_numbers,
        child_ids,
        root_id,
        members,
        eol,
        indent,
        source,
    })
}

fn valid_identity(value: &str) -> bool {
    uuid::Uuid::parse_str(value).is_ok()
        || (value.len() == 8 && value.chars().all(|character| character.is_ascii_hexdigit()))
}

/// Whether any quoted string in `source` contains a raw line break. KiCad
/// escapes them, so this only finds hand-edited files, whose strings
/// re-indentation would alter.
fn multi_line_string(source: &str) -> bool {
    let (mut in_string, mut escaped) = (false, false);
    for byte in source.bytes() {
        if escaped {
            escaped = false;
        } else if in_string {
            match byte {
                b'\\' => escaped = true,
                b'"' => in_string = false,
                b'\n' | b'\r' => return true,
                _ => {}
            }
        } else if byte == b'"' {
            in_string = true;
        }
    }
    false
}

/// What would read differently if this library footprint were parsed at the
/// board's format version instead of its own.
///
/// KiCad parses a board's footprints under the greater of the board's and the
/// footprint's `(version …)` (`PCB_IO_KICAD_SEXPR_PARSER::parseFOOTPRINT`), so
/// an older library footprint copied into a newer board is read under the
/// board's rules. Each check below is one version gate in KiCad 10.0.4's
/// parser, applied only when the library and the board fall on opposite sides
/// of it and the footprint uses the construct it governs. Formats before KiCad
/// 7 are refused whole: their arc and net-tie gates are not checked here.
fn format_reinterpretations(
    root: &SexpNode,
    source: &str,
    library: i64,
    board: i64,
) -> Vec<String> {
    let mut found = Vec::new();
    if library > board {
        found.push(format!(
            "the library format is newer than the board's ({library} > {board})"
        ));
        return found;
    }
    if library <= 20221018 {
        found.push(format!(
            "format {library} predates KiCad 7 (20221018), whose legacy arc, net-tie and \
             text rules this check does not reproduce"
        ));
        return found;
    }
    let below = |gate: i64| library < gate && board >= gate;
    let at_or_below = |gate: i64| library <= gate && board > gate;
    let has = |tag: &str| !find_block_starts(source, tag).is_empty();
    if below(20230620) && root.find("property").is_some() {
        found.push("footprint fields read with pre-8 visibility defaults (20230620)".into());
    }
    if below(20230825) && has("fp_text_box") {
        found.push("text boxes changed at 20230825".into());
    }
    if at_or_below(20240201) {
        for tag in [
            "solder_mask_margin",
            "solder_paste_margin",
            "solder_paste_margin_ratio",
            "clearance",
        ] {
            let zero = find_block_starts(source, tag).into_iter().any(|start| {
                find_balanced_block(source, start)
                    .and_then(|(s, e)| konnect_sexp::parse_sexp(&source[s..e]).ok())
                    .and_then(|node| node.get_f64(1))
                    == Some(0.0)
            });
            if zero {
                found.push(format!(
                    "a zero ({tag} …) meant 'inherit' before 20240201 and 'zero' after"
                ));
            }
        }
    }
    if below(20240706) && source.contains('|') {
        found.push("'|' became a token at 20240706".into());
    }
    if below(20250210) && has("zone") {
        found.push("footprint zone fill rules changed at 20250210".into());
    }
    if at_or_below(20250926) && has("die_delay") {
        found.push("pad die delay units changed at 20250926".into());
    }
    found
}

// ─── Building the board ──────────────────────────────────────────────────────

/// What the schematic export says about one part beyond the planner's view.
#[derive(Debug, Clone, Default)]
struct PartFields {
    fields: Vec<(String, String)>,
    sheetname: Option<String>,
    sheetfile: Option<String>,
}

fn netlist_fields(source: &str) -> HashMap<String, PartFields> {
    let mut parts = HashMap::new();
    let Ok(root) = konnect_sexp::parse_sexp(source) else {
        return parts;
    };
    let Some(components) = root.find("components") else {
        return parts;
    };
    for comp in components.find_all("comp") {
        let Some(reference) = comp.find_str("ref") else {
            continue;
        };
        let mut part = PartFields::default();
        if let Some(fields) = comp.find("fields") {
            for field in fields.find_all("field") {
                let Some(name) = field.find_str("name") else {
                    continue;
                };
                let value = field.get(2).and_then(SexpNode::as_str).unwrap_or_default();
                part.fields.push((name.to_string(), value.to_string()));
            }
        }
        for property in comp.find_all("property") {
            match (property.find_str("name"), property.find_str("value")) {
                (Some("Sheetname"), Some(value)) => part.sheetname = Some(value.to_string()),
                (Some("Sheetfile"), Some(value)) => part.sheetfile = Some(value.to_string()),
                _ => {}
            }
        }
        parts.insert(reference.to_string(), part);
    }
    parts
}

/// Hands out UUIDs derived from the plan's inputs: the same inputs give the
/// same IDs, so a dry run and its apply agree, and no ID repeats one the board
/// already holds or one handed out before.
struct IdAllocator {
    seed: [u8; 32],
    used: HashSet<String>,
}

impl IdAllocator {
    fn allocate(&mut self, label: &str) -> String {
        for attempt in 0u32.. {
            let mut hasher = Sha256::new();
            hasher.update(self.seed);
            hasher.update(label.as_bytes());
            hasher.update(attempt.to_le_bytes());
            let digest = hasher.finalize();
            let mut bytes = [0u8; 16];
            bytes.copy_from_slice(&digest[..16]);
            bytes[6] = (bytes[6] & 0x0f) | 0x40;
            bytes[8] = (bytes[8] & 0x3f) | 0x80;
            let id = uuid::Uuid::from_bytes(bytes).to_string();
            if self.used.insert(id.clone()) {
                return id;
            }
        }
        unreachable!("a u32 of attempts cannot all collide")
    }
}

/// What one planned instance must look like, for the readbacks.
#[derive(Debug, Clone)]
struct ExpectedInstance {
    reference: String,
    footprint_id: String,
    value: String,
    symbol_path: String,
    dnp: bool,
    pad_nets: BTreeMap<String, String>,
    root_uuid: String,
    position: Point,
    /// Library identity to instance identity.
    ids: HashMap<String, String>,
    /// Library fields whose value the schematic overrode, with the new value.
    overridden: BTreeMap<String, String>,
    /// Fields added from the schematic, hidden.
    added: BTreeMap<String, String>,
    sheetname: Option<String>,
    sheetfile: Option<String>,
}

/// One instance's text before its root `(at …)` is known.
struct Draft {
    edits: Vec<SexpEdit>,
    /// Where the root `(uuid …)` and `(at …)` go: right after the root layer.
    anchor: usize,
    /// The root UUID clause to insert there, unless the library had one.
    uuid_clause: Option<String>,
    /// Clauses inserted after the anchor with the UUID and position.
    after_anchor: String,
    /// The library's own root `(at …)` block, replaced when present.
    library_at: Option<(usize, usize)>,
}

impl Draft {
    fn text(&self, template: &Template, position: Option<Point>) -> String {
        let mut edits = self.edits.clone();
        let indent = &template.indent;
        let eol = template.eol;
        let at = position.map(|p| format!("(at {} {})", mm(p.x), mm(p.y)));
        let mut inserted = String::new();
        if let Some(uuid) = &self.uuid_clause {
            inserted.push_str(&format!("{eol}{indent}{uuid}"));
        }
        match (self.library_at, &at) {
            (Some((start, end)), Some(at)) => edits.push(SexpEdit::replace(start, end, at.clone())),
            (None, Some(at)) => inserted.push_str(&format!("{eol}{indent}{at}")),
            _ => {}
        }
        inserted.push_str(&self.after_anchor);
        if !inserted.is_empty() {
            edits.push(SexpEdit::insert(self.anchor, inserted));
        }
        konnect_sexp::apply_edits(template.source.clone(), edits)
    }
}

/// The edits that make `template` the instance `expected` describes.
fn draft(
    template: &Template,
    footprint_id: &str,
    expected: &mut ExpectedInstance,
    part: &PartFields,
    net_codes: Option<&BTreeMap<String, i64>>,
    ids: &mut IdAllocator,
) -> Result<Draft, String> {
    let source = &template.source;
    let eol = template.eol;
    let indent = &template.indent;
    let label = format!("{}\u{0}{}", expected.symbol_path, expected.reference);
    let mut edits = Vec::new();

    let (root_start, root_end) =
        find_balanced_block(source, 0).ok_or("the footprint is unbalanced")?;
    let name = tokens(&source[root_start..root_end])
        .get(1)
        .copied()
        .ok_or("the footprint has no name")?;
    edits.push(SexpEdit::replace(
        root_start + name.0,
        root_start + name.1,
        quote(footprint_id),
    ));
    for (start, end, head) in &template.children {
        if LIBRARY_HEADER.contains(&head.as_str()) {
            let (from, to) = find_block_with_leading_whitespace(source, *start)
                .filter(|(_, to)| to == end)
                .unwrap_or((*start, *end));
            edits.push(SexpEdit::delete(from, to));
        }
    }

    expected.root_uuid = ids.allocate(&format!("root\u{0}{label}"));
    let uuid_clause = match template.root_id {
        Some((start, end)) => {
            edits.push(SexpEdit::replace(start, end, quote(&expected.root_uuid)));
            None
        }
        None => Some(format!("(uuid {})", quote(&expected.root_uuid))),
    };
    for (start, end, value) in &template.child_ids {
        let new = expected
            .ids
            .entry(value.clone())
            .or_insert_with(|| ids.allocate(&format!("child\u{0}{label}\u{0}{value}")))
            .clone();
        let quoted = source[*start..*end].starts_with('"');
        edits.push(SexpEdit::replace(
            *start,
            *end,
            if quoted { quote(&new) } else { new },
        ));
    }
    for (start, end, value) in &template.members {
        let new = expected
            .ids
            .get(value)
            .ok_or_else(|| format!("group member '{value}' has no remapped identity"))?;
        edits.push(SexpEdit::replace(*start, *end, quote(new)));
    }

    let reference = template
        .field_value_span("Reference")
        .ok_or("the footprint has no Reference property or fp_text")?;
    edits.push(SexpEdit::replace(
        reference.0,
        reference.1,
        quote(&expected.reference),
    ));
    let value = template
        .field_value_span("Value")
        .ok_or("the footprint has no Value property or fp_text")?;
    edits.push(SexpEdit::replace(value.0, value.1, quote(&expected.value)));

    // Schematic fields: an existing library property keeps its presentation
    // and takes the schematic's text; an absent one is added hidden on F.Fab.
    let mut added = String::new();
    for (name, text) in &part.fields {
        if NOT_COPIED_FIELDS.contains(&name.as_str()) || text.is_empty() || text == "~" {
            continue;
        }
        if expected.overridden.contains_key(name) || expected.added.contains_key(name) {
            continue;
        }
        match template.field_value_span(name) {
            Some((start, end)) => {
                if unquote(&source[start..end]) != *text {
                    edits.push(SexpEdit::replace(start, end, quote(text)));
                    expected.overridden.insert(name.clone(), text.clone());
                }
            }
            None => {
                let uuid = ids.allocate(&format!("field\u{0}{label}\u{0}{name}"));
                added.push_str(&format!(
                    "{eol}{indent}(property {} {} (at 0 0 0) (unlocked yes) (layer \"F.Fab\") \
                     (hide yes) (uuid {}) (effects (font (size 1 1) (thickness 0.15))))",
                    quote(name),
                    quote(text),
                    quote(&uuid)
                ));
                expected.added.insert(name.clone(), text.clone());
            }
        }
    }
    added.push_str(&format!(
        "{eol}{indent}(path {})",
        quote(&expected.symbol_path)
    ));
    if let Some(sheetname) = &part.sheetname {
        added.push_str(&format!("{eol}{indent}(sheetname {})", quote(sheetname)));
    }
    if let Some(sheetfile) = &part.sheetfile {
        added.push_str(&format!("{eol}{indent}(sheetfile {})", quote(sheetfile)));
    }
    expected.sheetname = part.sheetname.clone();
    expected.sheetfile = part.sheetfile.clone();

    // DNP: the schematic overrides the library either way.
    match template.child("attr") {
        Some((start, end)) => {
            let block = &source[start..end];
            let spans = tokens(block);
            let dnp = spans
                .iter()
                .skip(1)
                .find(|(s, e)| &block[*s..*e] == "dnp")
                .copied();
            match (dnp, expected.dnp) {
                (None, true) => edits.push(SexpEdit::insert(end - 1, " dnp")),
                (Some((s, e)), false) => {
                    let from = block[..s].trim_end().len();
                    edits.push(SexpEdit::delete(start + from, start + e));
                }
                _ => {}
            }
        }
        None if expected.dnp => added.push_str(&format!("{eol}{indent}(attr dnp)")),
        None => {}
    }

    // Insert the new properties after the last library property, so they sit
    // with the others; with none, after the root identity and position.
    let last_property = template
        .children
        .iter()
        .filter(|(_, _, head)| head == "property" || head == "fp_text")
        .map(|(_, end, _)| *end)
        .max();
    let layer = template
        .child("layer")
        .ok_or("the footprint has no layer")?;
    let after_anchor = match last_property {
        Some(end) if end > layer.1 => {
            edits.push(SexpEdit::insert(end, added));
            String::new()
        }
        _ => added,
    };

    // Pad nets: every pad with a connected number, repeated numbers alike;
    // unnumbered pads never.
    let pads = template
        .children
        .iter()
        .filter(|(_, _, head)| head == "pad")
        .collect::<Vec<_>>();
    for ((start, end, _), pad) in pads.iter().zip(template.root.find_all("pad")) {
        let number = pad.get(1).and_then(SexpNode::as_str).unwrap_or_default();
        let Some(net) = expected.pad_nets.get(number).filter(|_| !number.is_empty()) else {
            continue;
        };
        let clause = match net_codes {
            Some(codes) => format!(
                "(net {} {})",
                codes.get(net).ok_or("a net has no code")?,
                quote(net)
            ),
            None => format!("(net {})", quote(net)),
        };
        let block = &source[*start..*end];
        let last = tokens(block).last().copied().ok_or("an empty pad")?;
        let multi_line = block[last.1..].contains('\n');
        let text = if multi_line {
            format!("{eol}{}{clause}", line_indent(source, start + last.0))
        } else {
            format!(" {clause}")
        };
        edits.push(SexpEdit::insert(start + last.1, text));
    }

    let mut starts = edits.iter().map(|edit| edit.start).collect::<Vec<_>>();
    starts.push(layer.1);
    starts.sort_unstable();
    if starts.windows(2).any(|pair| pair[0] == pair[1]) {
        return Err("two edits would land at one offset".to_string());
    }
    Ok(Draft {
        edits,
        anchor: layer.1,
        uuid_clause,
        after_anchor,
        library_at: template.child("at"),
    })
}

/// Build the complete board text a ready plan writes, and verify it before
/// anyone can apply it.
fn build(
    original: &str,
    inventory: &Inventory,
    saved: &SavedDesign,
    closed: &mut ClosedPlan,
    templates: BTreeMap<String, Template>,
) -> Result<(), Vec<SyncDiagnostic>> {
    let fields = netlist_fields(&saved.netlist_source);
    let legacy = !inventory.legacy_nets.is_empty();
    let mut net_names = BTreeSet::new();
    for change in &closed.plan.changes {
        if let PlannedChange::Add { pad_nets, .. } = change {
            net_names.extend(pad_nets.values().cloned());
        }
    }
    let mut net_codes = inventory.legacy_nets.clone();
    let mut new_nets = BTreeMap::new();
    if legacy {
        let mut next = net_codes.values().copied().max().unwrap_or(0) + 1;
        for name in &net_names {
            if !net_codes.contains_key(name) {
                net_codes.insert(name.clone(), next);
                new_nets.insert(name.clone(), next);
                next += 1;
            }
        }
    }

    let mut ids = IdAllocator {
        seed: closed.inputs.digest(),
        used: inventory.ids.clone(),
    };
    let mut drafts = Vec::new();
    let mut boxes = Vec::new();
    let mut failures: BTreeMap<String, (String, Vec<String>)> = BTreeMap::new();
    for change in &closed.plan.changes {
        let PlannedChange::Add {
            reference,
            value,
            footprint_id,
            symbol_path,
            dnp,
            pad_nets,
            ..
        } = change
        else {
            continue;
        };
        let template = &templates[footprint_id];
        let mut expected = ExpectedInstance {
            reference: reference.clone(),
            footprint_id: footprint_id.clone(),
            value: value.clone(),
            symbol_path: symbol_path.clone(),
            dnp: *dnp,
            pad_nets: pad_nets.clone(),
            root_uuid: String::new(),
            position: Point { x: 0.0, y: 0.0 },
            ids: HashMap::new(),
            overridden: BTreeMap::new(),
            added: BTreeMap::new(),
            sheetname: None,
            sheetfile: None,
        };
        let part = fields.get(reference).cloned().unwrap_or_default();
        let draft = draft(
            template,
            footprint_id,
            &mut expected,
            &part,
            legacy.then_some(&net_codes),
            &mut ids,
        )
        .and_then(|draft| {
            let body = draft.text(template, None);
            let node = konnect_sexp::parse_sexp(&body)
                .map_err(|error| format!("the instance does not parse: {error}"))?;
            let extent = konnect_sexp::bounds::footprint_parking_extent(&node)
                .map_err(|reason| format!("its extent cannot be measured: {reason}"))?;
            Ok((draft, parking::LocalBox::from_bbox(extent)))
        });
        match draft {
            Ok((draft, extent)) => {
                boxes.push(extent);
                drafts.push((draft, expected));
            }
            Err(reason) => {
                failures
                    .entry(footprint_id.clone())
                    .or_insert_with(|| (reason, Vec::new()))
                    .1
                    .push(reference.clone());
            }
        }
    }
    if !failures.is_empty() {
        return Err(failures
            .into_iter()
            .map(|(footprint_id, (reason, references))| {
                footprint_conflict(
                    "library_footprint_not_embeddable",
                    format!(
                        "{footprint_id} cannot be imported (needed by {}): {reason}",
                        listed(&references)
                    ),
                    &footprint_id,
                    references,
                )
            })
            .collect());
    }

    let (positions, summary) = parking::park(&boxes, inventory.state.bounds)
        .map_err(|reason| vec![conflict("parking_extent_out_of_range", reason, None)])?;
    // Whole nanometres, as KiCad stores them, so the written text and the
    // plan agree exactly.
    let positions = positions
        .into_iter()
        .map(|point| Point {
            x: (point.x * 1e6).round() / 1e6,
            y: (point.y * 1e6).round() / 1e6,
        })
        .collect::<Vec<_>>();
    let mut blocks = Vec::new();
    let mut position_iter = positions.iter();
    for change in &mut closed.plan.changes {
        if let PlannedChange::Add { position, .. } = change {
            *position = *position_iter.next().expect("one position per addition");
        }
    }
    for ((draft, expected), position) in drafts.iter_mut().zip(&positions) {
        expected.position = *position;
        blocks.push(draft.text(&templates[&expected.footprint_id], Some(*position)));
    }

    // Insert after the board's last item, so the file keeps its own closing
    // line; each block re-indented one level, in the board's line endings.
    let eol = if original.contains("\r\n") {
        "\r\n"
    } else {
        "\n"
    };
    let close = original.trim_end().len().saturating_sub(1);
    let insert_at = inventory.insert_after.unwrap_or(close);
    let footprints = blocks
        .iter()
        .map(|block| format!("{eol}{}", indent_block(block.trim_end(), "\t", eol)))
        .collect::<String>();
    let mut by_code = new_nets.iter().collect::<Vec<_>>();
    by_code.sort_by_key(|(_, code)| **code);
    let declarations = by_code
        .into_iter()
        .map(|(name, code)| format!("{eol}\t(net {code} {})", quote(name)))
        .collect::<String>();
    let net_at = inventory.last_net_end.unwrap_or(insert_at);
    let edits = if declarations.is_empty() {
        vec![SexpEdit::insert(insert_at, footprints)]
    } else if net_at == insert_at {
        vec![SexpEdit::insert(insert_at, declarations + &footprints)]
    } else {
        vec![
            SexpEdit::insert(insert_at, footprints),
            SexpEdit::insert(net_at, declarations),
        ]
    };
    let planned = konnect_sexp::apply_edits(original.to_string(), edits);

    closed.expected = drafts.into_iter().map(|(_, expected)| expected).collect();
    closed.templates = templates;
    closed.new_nets = new_nets;
    let verification = verify_board(
        original,
        &planned,
        &closed.expected,
        &closed.templates,
        &closed.new_nets,
        inventory.duplicate_ids,
    )
    .map_err(|errors| {
        vec![conflict(
            "planned_result_verification_failed",
            format!(
                "the planned board did not read back as planned, so it will not be written: {}",
                errors.join("; ")
            ),
            None,
        )]
    })?;

    let pad_properties = closed
        .expected
        .iter()
        .filter_map(|expected| {
            let facts = crate::tools::footprint_fabrication::footprint_pad_properties(
                &closed.templates[&expected.footprint_id].root,
            );
            (!facts.is_empty()).then(|| {
                json!({
                    "reference": expected.reference,
                    "footprint_id": expected.footprint_id,
                    "pad_properties": facts,
                })
            })
        })
        .collect::<Vec<_>>();
    closed.report = json!({
        "net_table": {
            "encoding": if legacy { "numbered_board_net_table" } else { "kicad10_pad_net_names" },
            "note": if legacy {
                "The board declares numbered nets; new nets were appended with the next codes and pads carry code and name."
            } else {
                "KiCad 10 boards declare no net table: each pad names its net and KiCad assigns codes on load."
            },
            "nets": net_names.len(),
            "new_numbered_nets": closed.new_nets,
        },
        "preserved_board_only": inventory.preserved_json(),
        "instances": closed.expected.iter().map(|expected| json!({
            "reference": expected.reference,
            "uuid": expected.root_uuid,
            "footprint_id": expected.footprint_id,
            "pads_netted": closed.templates[&expected.footprint_id]
                .root
                .find_all("pad")
                .iter()
                .filter(|pad| pad.get(1).and_then(SexpNode::as_str)
                    .is_some_and(|number| !number.is_empty() && expected.pad_nets.contains_key(number)))
                .count(),
            "fields_overridden": expected.overridden.keys().collect::<Vec<_>>(),
            "fields_added": expected.added.keys().collect::<Vec<_>>(),
        })).collect::<Vec<_>>(),
        "pad_fabrication_properties": pad_properties,
        "staging": {
            "basis": inventory.staging_basis,
            "board_extent": inventory.state.bounds,
            "not_measured": inventory.not_measured,
            "extent_source": "embedded_footprint_geometry",
        },
        "parking": summary,
        "planned_readback": verification,
    });
    closed.planned = Some(planned);
    Ok(())
}

// ─── Verification ────────────────────────────────────────────────────────────

/// Read `candidate` back as KiCad would see its structure and hold it to the
/// plan: every original item unchanged and in order, every new footprint the
/// library's definition with only its instance clauses changed, every pad's
/// net as planned, every identity unique. Runs on the planned text before it
/// is offered, and on the file after it is written.
fn verify_board(
    original: &str,
    candidate: &str,
    expected: &[ExpectedInstance],
    templates: &BTreeMap<String, Template>,
    new_nets: &BTreeMap<String, i64>,
    original_duplicates: usize,
) -> Result<serde_json::Value, Vec<String>> {
    let mut errors = Vec::new();
    if let Err(reason) = check_single_board_form(candidate) {
        return Err(vec![format!("not one (kicad_pcb …) form: {reason}")]);
    }
    let original_children = find_direct_child_blocks(original, "kicad_pcb");
    let candidate_children = find_direct_child_blocks(candidate, "kicad_pcb");
    if let (Some(first), Some(first_candidate)) =
        (original_children.first(), candidate_children.first())
    {
        if original[..first.0] != candidate[..first_candidate.0] {
            errors.push("the board header changed".to_string());
        }
    }
    if let (Some(last), Some(last_candidate)) =
        (original_children.last(), candidate_children.last())
    {
        if original[last.1..] != candidate[last_candidate.1..] {
            errors.push("the board's closing text changed".to_string());
        }
    }

    // Original items, unchanged and in order; everything else added by plan.
    let by_reference = expected
        .iter()
        .map(|instance| (instance.reference.as_str(), instance))
        .collect::<HashMap<_, _>>();
    let mut kept = 0usize;
    let mut found: HashMap<&str, usize> = HashMap::new();
    let mut nets_added = 0usize;
    let mut pads_netted = 0usize;
    let mut properties_preserved = 0usize;
    for &(start, end) in &candidate_children {
        let text = &candidate[start..end];
        if let Some(&(o_start, o_end)) = original_children.get(kept) {
            if &original[o_start..o_end] == text {
                kept += 1;
                continue;
            }
        }
        let Ok(node) = konnect_sexp::parse_sexp(text) else {
            errors.push("an added item does not parse".to_string());
            continue;
        };
        match node.head() {
            Some("net") => {
                let name = konnect_sexp::net::net_name(&node).unwrap_or_default();
                let code = konnect_sexp::net::net_id(&node).and_then(|id| id.parse::<i64>().ok());
                if new_nets.get(name) == code.as_ref() && code.is_some() {
                    nets_added += 1;
                } else {
                    errors.push(format!("unplanned net declaration '{name}'"));
                }
            }
            Some("footprint") => {
                let reference = footprint_reference(&node).unwrap_or_default();
                let Some(instance) = by_reference.get(reference.as_str()) else {
                    errors.push(format!("unplanned footprint '{reference}'"));
                    continue;
                };
                *found.entry(&instance.reference).or_insert(0) += 1;
                match verify_instance(
                    &node,
                    text,
                    instance,
                    &templates[&instance.footprint_id],
                    new_nets,
                ) {
                    Ok((netted, preserved)) => {
                        pads_netted += netted;
                        properties_preserved += preserved;
                    }
                    Err(problems) => errors.extend(
                        problems
                            .into_iter()
                            .map(|problem| format!("{}: {problem}", instance.reference)),
                    ),
                }
            }
            other => errors.push(format!(
                "an original {} item is missing or changed",
                other.unwrap_or("board")
            )),
        }
    }
    if kept != original_children.len() {
        errors.push(format!(
            "{} of {} original board items survived unchanged",
            kept,
            original_children.len()
        ));
    }
    for instance in expected {
        match found.get(instance.reference.as_str()) {
            Some(1) => {}
            Some(count) => errors.push(format!("{} appears {count} times", instance.reference)),
            None => errors.push(format!("{} is missing", instance.reference)),
        }
    }
    if new_nets.len() != nets_added {
        errors.push(format!(
            "{} of {} new net declarations present",
            nets_added,
            new_nets.len()
        ));
    }
    let (_, duplicates) = identity_values(candidate);
    if duplicates != original_duplicates {
        errors.push(format!(
            "{} identities repeat (the original board repeated {original_duplicates})",
            duplicates
        ));
    }
    if errors.is_empty() {
        Ok(json!({
            "verified": true,
            "original_items_unchanged": kept,
            "footprints_added": found.len(),
            "pads_netted": pads_netted,
            "pad_fabrication_properties_preserved": properties_preserved,
            "numbered_nets_added": nets_added,
            "duplicate_identities": duplicates - original_duplicates,
        }))
    } else {
        Err(errors)
    }
}

/// Hold one embedded footprint to its library definition and its plan.
/// Returns how many pads carry nets and how many fabrication-property clauses
/// survived byte for byte.
fn verify_instance(
    node: &SexpNode,
    text: &str,
    instance: &ExpectedInstance,
    template: &Template,
    new_nets: &BTreeMap<String, i64>,
) -> Result<(usize, usize), Vec<String>> {
    let mut problems = Vec::new();
    let check = |ok: bool, problem: &str, problems: &mut Vec<String>| {
        if !ok {
            problems.push(problem.to_string());
        }
    };
    check(
        node.get(1).and_then(SexpNode::as_str) == Some(instance.footprint_id.as_str()),
        "wrong library identifier",
        &mut problems,
    );
    check(
        footprint_identity(node) == Some(instance.root_uuid.as_str()),
        "wrong instance UUID",
        &mut problems,
    );
    let at = node.find("at");
    let near = |a: Option<f64>, b: f64| a.is_some_and(|a| (a - b).abs() < 1e-6);
    check(
        near(at.and_then(|at| at.get_f64(1)), instance.position.x)
            && near(at.and_then(|at| at.get_f64(2)), instance.position.y)
            && at.and_then(|at| at.get_f64(3)).unwrap_or(0.0) == 0.0,
        "wrong parked position or rotation",
        &mut problems,
    );
    check(
        node.find_str("layer") == Some("F.Cu"),
        "not on F.Cu",
        &mut problems,
    );
    check(
        node.find_str("path") == Some(instance.symbol_path.as_str()),
        "wrong symbol path",
        &mut problems,
    );
    check(
        node.find_str("sheetname") == instance.sheetname.as_deref()
            && node.find_str("sheetfile") == instance.sheetfile.as_deref(),
        "wrong sheet name or file",
        &mut problems,
    );
    check(
        field_value(node, "Value", "value").as_deref() == Some(instance.value.as_str()),
        "wrong value",
        &mut problems,
    );
    let attr = |footprint: &SexpNode| {
        footprint
            .find("attr")
            .and_then(SexpNode::children)
            .unwrap_or_default()
            .iter()
            .skip(1)
            .filter_map(SexpNode::as_str)
            .map(str::to_string)
            .collect::<BTreeSet<_>>()
    };
    let (mut library_attr, mut instance_attr) = (attr(&template.root), attr(node));
    check(
        instance_attr.contains("dnp") == instance.dnp,
        "wrong DNP",
        &mut problems,
    );
    library_attr.remove("dnp");
    instance_attr.remove("dnp");
    check(
        library_attr == instance_attr,
        "library attributes changed",
        &mut problems,
    );
    for (name, value) in &instance.added {
        let added = node
            .find_all("property")
            .into_iter()
            .find(|property| property.get(1).and_then(SexpNode::as_str) == Some(name));
        check(
            added.is_some_and(|property| {
                property.get(2).and_then(SexpNode::as_str) == Some(value.as_str())
                    && property.find_str("hide") == Some("yes")
            }),
            &format!("field '{name}' not added hidden with the schematic's text"),
            &mut problems,
        );
    }
    for (name, value) in &instance.overridden {
        check(
            field_value(node, name, "").as_deref() == Some(value.as_str()),
            &format!("field '{name}' does not carry the schematic's text"),
            &mut problems,
        );
    }

    // Groups name only this instance's own identities, never the library's.
    let mut defined = HashSet::new();
    let mut members = Vec::new();
    collect_identities(node, &mut defined, &mut members);
    for member in members {
        if !defined.contains(&member) {
            problems.push(format!(
                "group member '{member}' names no item of this footprint"
            ));
        }
    }

    // Pads: nets exactly as planned, by number, repeated numbers alike.
    let mut netted = 0;
    for pad in node.find_all("pad") {
        let number = pad.get(1).and_then(SexpNode::as_str).unwrap_or_default();
        let want = instance
            .pad_nets
            .get(number)
            .filter(|_| !number.is_empty())
            .map(String::as_str);
        let net = pad.find("net");
        let got = net.and_then(konnect_sexp::net::net_name);
        if got != want {
            problems.push(format!("pad '{number}' net is {got:?}, planned {want:?}"));
        }
        if let (Some(net), Some(name)) = (net, got) {
            netted += 1;
            if let Some(code) = new_nets.get(name) {
                if konnect_sexp::net::net_id(net) != Some(code.to_string().as_str()) {
                    problems.push(format!("pad '{number}' net code differs from the table"));
                }
            }
        }
    }

    // Everything else is the library's definition: compare trees with the
    // instance clauses taken back out and identities mapped back.
    let inverse = instance
        .ids
        .iter()
        .map(|(library, new)| (new.clone(), library.clone()))
        .collect::<HashMap<_, _>>();
    let library_children = template
        .root
        .children()
        .unwrap_or_default()
        .iter()
        .skip(2)
        .filter(|child| match child.head() {
            Some("version" | "generator" | "generator_version" | "attr" | "at") => false,
            // The root's own identity, replaced by the instance's.
            Some("uuid" | "tstamp") => template.root_id.is_none(),
            _ => true,
        })
        .map(|child| restore_library_field(child, template, instance))
        .collect::<Vec<_>>();
    let instance_children = node
        .children()
        .unwrap_or_default()
        .iter()
        .skip(2)
        .filter(|child| {
            let head = child.head().unwrap_or_default();
            if matches!(
                head,
                "uuid" | "tstamp" | "at" | "path" | "sheetname" | "sheetfile" | "attr"
            ) {
                return false;
            }
            !(head == "property"
                && child
                    .get(1)
                    .and_then(SexpNode::as_str)
                    .is_some_and(|name| instance.added.contains_key(name)))
        })
        .map(|child| restore_library_field(&map_back(child, &inverse), template, instance))
        .collect::<Vec<_>>();
    if library_children != instance_children {
        let first = library_children
            .iter()
            .zip(&instance_children)
            .position(|(a, b)| a != b)
            .unwrap_or(library_children.len().min(instance_children.len()));
        problems.push(format!(
            "the definition differs from the library at child {first} ({} library, {} instance \
             children)",
            library_children.len(),
            instance_children.len()
        ));
    }

    // Fabrication properties, byte for byte.
    let mut preserved = 0;
    let library_pads = template
        .children
        .iter()
        .filter(|(_, _, head)| head == "pad")
        .map(|(start, end, _)| &template.source[*start..*end])
        .collect::<Vec<_>>();
    let instance_pads = find_direct_child_blocks(text, "footprint")
        .into_iter()
        .map(|(start, end)| &text[start..end])
        .filter(|block| tokens(block).first().map(|&(s, e)| &block[s..e]) == Some("pad"))
        .collect::<Vec<_>>();
    if library_pads.len() != instance_pads.len() {
        problems.push("the pad count changed".to_string());
    }
    for (library_pad, instance_pad) in library_pads.iter().zip(&instance_pads) {
        for start in find_block_starts(library_pad, "property") {
            if let Some((s, e)) = find_balanced_block(library_pad, start) {
                if instance_pad.contains(&library_pad[s..e]) {
                    preserved += 1;
                } else {
                    problems.push(format!(
                        "pad fabrication clause {} was not copied byte for byte",
                        &library_pad[s..e]
                    ));
                }
            }
        }
    }
    if problems.is_empty() {
        Ok((netted, preserved))
    } else {
        Err(problems)
    }
}

/// Every identity `node` defines, and every group member it names.
fn collect_identities(node: &SexpNode, defined: &mut HashSet<String>, members: &mut Vec<String>) {
    let Some(children) = node.children() else {
        return;
    };
    match node.head() {
        Some("uuid" | "tstamp") => {
            if let Some(value) = children.get(1).and_then(SexpNode::as_str) {
                defined.insert(value.to_string());
            }
        }
        Some("members") => members.extend(
            children[1..]
                .iter()
                .filter_map(SexpNode::as_str)
                .map(str::to_string),
        ),
        _ => {}
    }
    for child in children {
        collect_identities(child, defined, members);
    }
}

/// `node` with instance identities mapped back to the library's, and pads'
/// nets taken out.
fn map_back(node: &SexpNode, inverse: &HashMap<String, String>) -> SexpNode {
    match node {
        SexpNode::List(children) => {
            let head = node.head();
            let mapped = children
                .iter()
                .enumerate()
                .filter(|(_, child)| !(head == Some("pad") && child.head() == Some("net")))
                .map(|(index, child)| match (head, child) {
                    (Some("uuid" | "tstamp" | "members"), SexpNode::Str(value)) if index > 0 => {
                        SexpNode::Str(inverse.get(value).cloned().unwrap_or(value.clone()))
                    }
                    (Some("uuid" | "tstamp"), SexpNode::Atom(value)) if index > 0 => {
                        SexpNode::Atom(inverse.get(value).cloned().unwrap_or(value.clone()))
                    }
                    _ => map_back(child, inverse),
                })
                .collect();
            SexpNode::List(mapped)
        }
        other => other.clone(),
    }
}

/// A Reference, Value or overridden field with the library's text back in
/// place, so the tree comparison sees only what the instance must not change.
fn restore_library_field(
    node: &SexpNode,
    template: &Template,
    instance: &ExpectedInstance,
) -> SexpNode {
    let SexpNode::List(children) = node else {
        return node.clone();
    };
    let key = match (node.head(), node.get(1).and_then(SexpNode::as_str)) {
        (Some("property"), Some(name))
            if name == "Reference" || name == "Value" || instance.overridden.contains_key(name) =>
        {
            name.to_string()
        }
        (Some("fp_text"), Some("reference")) => "Reference".to_string(),
        (Some("fp_text"), Some("value")) => "Value".to_string(),
        _ => return node.clone(),
    };
    let Some((start, end)) = template.field_value_span(&key) else {
        return node.clone();
    };
    let mut children = children.clone();
    if children.len() > 2 {
        children[2] = SexpNode::Str(unquote(&template.source[start..end]));
    }
    SexpNode::List(children)
}

// ─── Write and responses ─────────────────────────────────────────────────────

fn write_and_read_back(
    board: &Path,
    planned: &ClosedPlan,
    target: &str,
    closure: &serde_json::Value,
) -> CallToolResult {
    let original_duplicates = identity_values(&planned.original).1;
    let verify = |text: &str| {
        verify_board(
            &planned.original,
            text,
            &planned.expected,
            &planned.templates,
            &planned.new_nets,
            original_duplicates,
        )
    };
    let written = konnect_sexp::write_atomic_if_unchanged(board, &planned.original, target);
    let reported_error = match written {
        Ok(()) => None,
        Err(error) => {
            let now = std::fs::read_to_string(board);
            match now {
                Ok(text) if text == planned.original => {
                    let mut plan = ClosedPlan {
                        plan: planned.plan.clone(),
                        hierarchy_files: planned.hierarchy_files,
                        inputs: planned.inputs.clone(),
                        original: String::new(),
                        planned: None,
                        expected: Vec::new(),
                        templates: BTreeMap::new(),
                        new_nets: BTreeMap::new(),
                        report: planned.report.clone(),
                    };
                    refuse_plan(
                        &mut plan.plan,
                        vec![conflict(
                            "board_write_refused",
                            format!(
                                "the board was not written: {error}. It is byte-identical to \
                                 the plan's input"
                            ),
                            None,
                        )],
                    );
                    return response(&plan, "conflict", closure, None);
                }
                Ok(text) if text == target => Some(error.to_string()),
                _ => {
                    return CallToolResult::error_kind(
                        crate::mcp::error::ToolErrorKind::MutationOutcomeUncertain {
                            operation: "update_pcb_from_schematic closed_initial".to_string(),
                            path: board.display().to_string(),
                            reason: format!(
                                "the atomic write reported '{error}', and the board now matches \
                                 neither the plan's input nor its result"
                            ),
                        },
                        format!(
                            "The closed-board import may or may not have been written: {error}. \
                             Inspect {} before retrying; this may not be a no-op.",
                            board.display()
                        ),
                    )
                }
            }
        }
    };
    let persisted = match konnect_sexp::read_consistent(board) {
        Ok(text) => text,
        Err(error) => {
            return CallToolResult::error_kind(
                crate::mcp::error::ToolErrorKind::MutationOutcomeUncertain {
                    operation: "update_pcb_from_schematic closed_initial".to_string(),
                    path: board.display().to_string(),
                    reason: format!("the written board could not be read back: {error}"),
                },
                format!("The closed-board import was written but could not be read back: {error}."),
            )
        }
    };
    let readback = if persisted == target {
        verify(&persisted)
    } else {
        Err(vec![
            "the persisted board differs from the planned text".to_string()
        ])
    };
    match readback {
        Ok(verification) => {
            let mut plan = planned.plan.clone();
            plan.counts.added.applied = plan.counts.added.planned;
            plan.counts.board_only_preserved.applied = plan.counts.board_only_preserved.planned;
            plan.counts.skipped_by_flag.applied = plan.counts.skipped_by_flag.planned;
            plan.counts.unassigned_footprint.applied = plan.counts.unassigned_footprint.planned;
            let applied = ClosedPlan {
                plan,
                hierarchy_files: planned.hierarchy_files,
                inputs: planned.inputs.clone(),
                original: String::new(),
                planned: None,
                expected: Vec::new(),
                templates: BTreeMap::new(),
                new_nets: BTreeMap::new(),
                report: planned.report.clone(),
            };
            response(
                &applied,
                "applied",
                closure,
                Some(json!({
                    "file": board.display().to_string(),
                    "atomicity": "single_atomic_file_replace",
                    "written_sha256": sha256_hex(persisted.as_bytes()),
                    "write_reported_error_after_replace": reported_error,
                    "persisted_readback": verification,
                })),
            )
        }
        Err(errors) => CallToolResult::error_kind(
            crate::mcp::error::ToolErrorKind::MutationOutcomeUncertain {
                operation: "update_pcb_from_schematic closed_initial".to_string(),
                path: board.display().to_string(),
                reason: errors.join("; "),
            },
            format!(
                "The closed-board import was written, but the persisted board did not read back \
                 as planned: {}. Inspect it before relying on it.",
                errors.join("; ")
            ),
        ),
    }
}

fn response(
    closed: &ClosedPlan,
    status: &str,
    closure: &serde_json::Value,
    write: Option<serde_json::Value>,
) -> CallToolResult {
    let plan = &closed.plan;
    let value = json!({
        "status": status,
        "backend": BACKEND,
        "plan_revision": plan.plan_revision,
        "coverage": {
            "source": "saved_schematic_hierarchy",
            "hierarchy_files": closed.hierarchy_files,
            "transport": "closed_board_file",
            "atomicity": "single_atomic_file_replace",
            "footprints_added": plan.counts.added,
            "footprints_updated": plan.counts.updated,
            "pads_reassigned": plan.counts.pads_reassigned,
            "board_only_preserved": plan.counts.board_only_preserved,
            "skipped_by_flag": plan.counts.skipped_by_flag,
            "unassigned_footprint": plan.counts.unassigned_footprint,
            "conflicts": plan.counts.conflicts
        },
        "board_closure": closure,
        "changes": plan.changes,
        "diagnostics": plan.diagnostics,
        "unassigned_footprints": plan.unassigned,
        "inputs": closed.inputs.json(),
        "report": closed.report,
        "write": write,
        "native_validation": NATIVE_VALIDATION,
        "undo": if status == "applied" {
            Some("A closed-board write has no KiCad undo step: restore the board file from its \
                  previous version (inputs.board.sha256 identifies it) to reverse the import.")
        } else {
            None
        }
    });
    CallToolResult::json(&value)
}

/// A refusal before any plan exists, in the shape every conflict has.
fn refusal_result(message: String, closure: Option<&serde_json::Value>) -> CallToolResult {
    let value = json!({
        "status": "conflict",
        "backend": BACKEND,
        "coverage": {
            "transport": "closed_board_file",
            "footprints_added": super::CountPair::default(),
            "conflicts": super::CountPair { planned: 1, applied: 0 }
        },
        "board_closure": closure,
        "diagnostics": [conflict("preflight_conflict", message, None)],
        "write": null,
    });
    CallToolResult {
        content: vec![ToolContent::Text {
            text: value.to_string(),
        }],
        is_error: true,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A KiCad 10 board as the native mounting-hole tool leaves it: an
    /// outline, setup, and one unlinked, unnetted mounting hole that does not
    /// carry the board_only attribute.
    const BOARD: &str = r#"(kicad_pcb
	(version 20260206)
	(generator "pcbnew")
	(generator_version "10.0")
	(general
		(thickness 1.6)
		(legacy_teardrops no)
	)
	(paper "A4")
	(layers
		(0 "F.Cu" signal)
		(2 "B.Cu" signal)
		(25 "Edge.Cuts" user)
		(31 "F.CrtYd" user "F.Courtyard")
		(35 "F.Fab" user)
		(5 "F.SilkS" user "F.Silkscreen")
		(13 "F.Paste" user)
		(1 "F.Mask" user)
	)
	(setup
		(pad_to_mask_clearance 0.05)
	)
	(footprint "MountingHole:MountingHole_3.2mm_M3"
		(layer "F.Cu")
		(uuid "00000000-0000-4000-8000-0000000000a1")
		(at 5 5)
		(property "Reference" "MH1"
			(at 0 -4 0)
			(layer "F.SilkS")
			(uuid "00000000-0000-4000-8000-0000000000a2")
			(effects (font (size 1 1) (thickness 0.15)))
		)
		(property "Value" "MountingHole"
			(at 0 4 0)
			(layer "F.Fab")
			(uuid "00000000-0000-4000-8000-0000000000a3")
			(effects (font (size 1 1) (thickness 0.15)))
		)
		(attr exclude_from_pos_files exclude_from_bom)
		(pad "" np_thru_hole circle
			(at 0 0)
			(size 3.2 3.2)
			(drill 3.2)
			(layers "*.Cu" "*.Mask")
			(uuid "00000000-0000-4000-8000-0000000000a4")
		)
	)
	(gr_rect
		(start 0 0)
		(end 100 80)
		(stroke (width 0.05) (type default))
		(fill no)
		(layer "Edge.Cuts")
		(uuid "00000000-0000-4000-8000-0000000000b1")
	)
	(embedded_fonts no)
)
"#;

    /// A BGA-style library footprint: fabrication properties and a pad mask
    /// margin no typed pad can carry, identities on every child, a group
    /// naming two of them, a net tie, a repeated pad number, a paste-only
    /// unnumbered pad, a Datasheet property, and a model.
    const BGA: &str = r#"(footprint "Bga4"
	(version 20260206)
	(generator "pcbnew")
	(generator_version "10.0")
	(layer "F.Cu")
	(descr "Test BGA with fabrication properties")
	(tags "bga")
	(property "Reference" "REF**"
		(at 0 -3 0)
		(layer "F.SilkS")
		(uuid "5d0a6a10-0000-4000-8000-000000000001")
		(effects (font (size 1 1) (thickness 0.15)))
	)
	(property "Value" "Bga4"
		(at 0 3 0)
		(layer "F.Fab")
		(uuid "5d0a6a10-0000-4000-8000-000000000002")
		(effects (font (size 1 1) (thickness 0.15)))
	)
	(property "Datasheet" ""
		(at 0 0 0)
		(layer "F.Fab")
		(hide yes)
		(uuid "5d0a6a10-0000-4000-8000-000000000003")
		(effects (font (size 1.27 1.27) (thickness 0.15)))
	)
	(attr smd)
	(net_tie_pad_groups "A1, A2")
	(fp_line
		(start -2 -2)
		(end 2 -2)
		(stroke (width 0.12) (type solid))
		(layer "F.SilkS")
		(uuid "5d0a6a10-0000-4000-8000-000000000010")
	)
	(fp_rect
		(start -2.5 -2.5)
		(end 2.5 2.5)
		(stroke (width 0.05) (type solid))
		(fill no)
		(layer "F.CrtYd")
		(uuid "5d0a6a10-0000-4000-8000-000000000011")
	)
	(pad "A1" smd circle
		(at -0.5 -0.5)
		(size 0.3 0.3)
		(layers "F.Cu" "F.Mask" "F.Paste")
		(property pad_prop_bga)
		(solder_mask_margin 0.05)
		(uuid "5d0a6a10-0000-4000-8000-000000000020")
	)
	(pad "A2" smd circle
		(at 0.5 -0.5)
		(size 0.3 0.3)
		(layers "F.Cu" "F.Mask" "F.Paste")
		(property pad_prop_bga)
		(uuid "5d0a6a10-0000-4000-8000-000000000021")
	)
	(pad "B1" smd circle
		(at -0.5 0.5)
		(size 0.3 0.3)
		(layers "F.Cu" "F.Mask" "F.Paste")
		(property pad_prop_bga)
		(uuid "5d0a6a10-0000-4000-8000-000000000022")
	)
	(pad "B2" smd circle
		(at 0.5 0.5)
		(size 0.3 0.3)
		(layers "F.Cu" "F.Mask" "F.Paste")
		(property pad_prop_bga)
		(uuid "5d0a6a10-0000-4000-8000-000000000023")
	)
	(pad "B2" smd rect
		(at 1.2 0.5)
		(size 0.3 0.3)
		(layers "F.Cu" "F.Mask")
		(uuid "5d0a6a10-0000-4000-8000-000000000024")
	)
	(pad "" smd rect
		(at 0 1.5)
		(size 0.4 0.4)
		(layers "F.Paste")
		(uuid "5d0a6a10-0000-4000-8000-000000000025")
	)
	(group ""
		(uuid "5d0a6a10-0000-4000-8000-000000000030")
		(members "5d0a6a10-0000-4000-8000-000000000010" "5d0a6a10-0000-4000-8000-000000000011")
	)
	(embedded_fonts no)
	(model "${KIPRJMOD}/models/bga4.step"
		(offset (xyz 0 0 0))
		(scale (xyz 1 1 1))
		(rotate (xyz 0 0 0))
	)
)
"#;
    const HEATSINK: &str =
        include_str!("../../../tests/fixtures/custom_pads_vqfn20_kicad10.kicad_mod");
    const HEATSINK_ID: &str = "Package_DFN_QFN:VQFN-20-1EP_3x3mm_P0.45mm_EP1.55x1.55mm";
    const C0603: &str = include_str!("../../../tests/fixtures/c_0603_1608metric_kicad10.kicad_mod");

    fn netlist(date: &str) -> String {
        format!(
            r#"(export (version "E")
  (design (source "carrier.kicad_sch") (date "{date}") (tool "Eeschema 10.0.4"))
  (components
    (comp (ref "U1") (value "MCU") (footprint "Parts:Bga4")
      (fields (field (name "Footprint") "Parts:Bga4") (field (name "Datasheet") "https://example.com/mcu.pdf") (field (name "MPN") "MCU-123") (field (name "LCSC") "C1234") (field (name "Empty")))
      (property (name "Sheetname") (value "Root")) (property (name "Sheetfile") (value "carrier.kicad_sch"))
      (sheetpath (names "/") (tstamps "/")) (tstamps "11111111-1111-4111-8111-000000000001"))
    (comp (ref "U2") (value "MCU") (footprint "Parts:Bga4")
      (property (name "dnp"))
      (sheetpath (names "/") (tstamps "/")) (tstamps "11111111-1111-4111-8111-000000000002"))
    (comp (ref "U3") (value "BUCK") (footprint "{HEATSINK_ID}")
      (sheetpath (names "/") (tstamps "/")) (tstamps "11111111-1111-4111-8111-000000000003"))
    (comp (ref "C1") (value "100n") (footprint "Capacitor_SMD:C_0603_1608Metric")
      (sheetpath (names "/") (tstamps "/")) (tstamps "11111111-1111-4111-8111-000000000004")))
  (nets
    (net (code "1") (name "GND") (class "Default")
      (node (ref "U1") (pin "A1")) (node (ref "U1") (pin "B2")) (node (ref "U2") (pin "A1"))
      (node (ref "U3") (pin "21")) (node (ref "C1") (pin "2")))
    (net (code "2") (name "+3V3") (class "Default")
      (node (ref "U1") (pin "A2")) (node (ref "C1") (pin "1")))
    (net (code "3") (name "/SIG") (class "Default")
      (node (ref "U2") (pin "B1")))))
"#
        )
    }

    struct Project {
        _temp: tempfile::TempDir,
        dir: PathBuf,
        schematic: PathBuf,
        board: PathBuf,
        exported: PathBuf,
        cli: String,
    }

    impl Project {
        fn new(board: &str, netlist: &str) -> Self {
            use crate::tools::cli::test_support::write_script;
            let temp = tempfile::tempdir().unwrap();
            let dir = temp.path().to_path_buf();
            let board_path = dir.join("carrier.kicad_pcb");
            std::fs::write(&board_path, board).unwrap();
            for (library, name, source) in [
                ("Parts", "Bga4", BGA),
                (
                    "Package_DFN_QFN",
                    HEATSINK_ID.split_once(':').unwrap().1,
                    HEATSINK,
                ),
                ("Capacitor_SMD", "C_0603_1608Metric", C0603),
            ] {
                let pretty = dir.join(format!("{library}.pretty"));
                std::fs::create_dir_all(&pretty).unwrap();
                std::fs::write(pretty.join(format!("{name}.kicad_mod")), source).unwrap();
            }
            std::fs::write(
                dir.join("fp-lib-table"),
                "(fp_lib_table\n  (lib (name \"Parts\") (type \"KiCad\") (uri \"${KIPRJMOD}/Parts.pretty\") (options \"\") (descr \"\"))\n  (lib (name \"Package_DFN_QFN\") (type \"KiCad\") (uri \"${KIPRJMOD}/Package_DFN_QFN.pretty\") (options \"\") (descr \"\"))\n  (lib (name \"Capacitor_SMD\") (type \"KiCad\") (uri \"${KIPRJMOD}/Capacitor_SMD.pretty\") (options \"\") (descr \"\"))\n)\n",
            )
            .unwrap();
            let schematic = dir.join("carrier.kicad_sch");
            std::fs::write(
                &schematic,
                include_bytes!("../../../tests/fixtures/structural_scans_kicad10.kicad_sch"),
            )
            .unwrap();
            let exported = dir.join("carrier.net");
            std::fs::write(&exported, netlist).unwrap();
            let unix_source = exported.to_string_lossy().replace('\'', "'\\''");
            let windows_source = exported.to_string_lossy();
            let cli = write_script(
                &dir,
                "fake-kicad-cli-closed",
                &format!(
                    "#!/bin/sh\nwhile [ \"$#\" -gt 0 ]; do\n  if [ \"$1\" = \"--output\" ]; then\n    shift\n    cp '{unix_source}' \"$1\"\n    exit $?\n  fi\n  shift\ndone\nexit 2\n"
                ),
                &format!(
                    "@echo off\r\n:loop\r\nif \"%~1\"==\"\" exit /b 2\r\nif \"%~1\"==\"--output\" goto found\r\nshift\r\ngoto loop\r\n:found\r\nshift\r\ncopy /Y \"{windows_source}\" \"%~1\" >nul\r\nexit /b %ERRORLEVEL%\r\n"
                ),
            );
            Self {
                _temp: temp,
                dir,
                schematic,
                board: board_path,
                exported,
                cli: cli.to_string_lossy().to_string(),
            }
        }

        /// A context whose KiCad is `address`; "" is unreachable.
        fn ctx(&self, address: &str) -> ToolContext {
            ToolContext::new(
                crate::tools::ServerConfig {
                    kicad_cli: self.cli.clone(),
                    kicad_binary: String::new(),
                    ipc_address: address.to_string(),
                    project_dir: None,
                    jlcpcb_db_path: None,
                    auto_load_toolsets: false,
                    eager_toolsets: false,
                },
                std::sync::Arc::new(crate::router::ToolRouter::new()),
            )
        }

        async fn call(
            &self,
            ctx: &ToolContext,
            extra: serde_json::Value,
        ) -> (bool, serde_json::Value) {
            let mut args = json!({
                "schematic": self.schematic.to_string_lossy(),
                "board": self.board.to_string_lossy(),
                "backend": "closed_initial",
            });
            for (key, value) in extra.as_object().unwrap() {
                args[key] = value.clone();
            }
            let result = super::super::handle_update_pcb_from_schematic(&args, ctx)
                .await
                .unwrap();
            let text = match &result.content[0] {
                ToolContent::Text { text } => text.clone(),
                _ => panic!("text result"),
            };
            (result.is_error, serde_json::from_str(&text).unwrap())
        }

        async fn dry_run(&self, ctx: &ToolContext) -> serde_json::Value {
            self.call(ctx, json!({})).await.1
        }

        async fn apply(
            &self,
            ctx: &ToolContext,
            revision: &serde_json::Value,
        ) -> (bool, serde_json::Value) {
            self.call(
                ctx,
                json!({ "dry_run": false, "expected_plan_revision": revision }),
            )
            .await
        }

        fn board_text(&self) -> String {
            std::fs::read_to_string(&self.board).unwrap()
        }
    }

    fn footprint<'a>(root: &'a SexpNode, reference: &str) -> &'a SexpNode {
        let found = root
            .find_all("footprint")
            .into_iter()
            .filter(|node| footprint_reference(node).as_deref() == Some(reference))
            .collect::<Vec<_>>();
        assert_eq!(found.len(), 1, "{reference} appears once");
        found[0]
    }

    fn identities(node: &SexpNode, out: &mut Vec<String>) {
        if let Some(children) = node.children() {
            if matches!(node.head(), Some("uuid" | "tstamp")) {
                out.push(node.get(1).and_then(SexpNode::as_str).unwrap().to_string());
            }
            for child in children {
                identities(child, out);
            }
        }
    }

    fn pad_nets(footprint: &SexpNode) -> Vec<(String, Option<String>)> {
        footprint
            .find_all("pad")
            .into_iter()
            .map(|pad| {
                (
                    pad.get(1).and_then(SexpNode::as_str).unwrap().to_string(),
                    pad.find("net")
                        .and_then(konnect_sexp::net::net_name)
                        .map(str::to_string),
                )
            })
            .collect()
    }

    fn heatsink_path(dir: &Path) -> PathBuf {
        dir.join("Package_DFN_QFN.pretty").join(format!(
            "{}.kicad_mod",
            HEATSINK_ID.split_once(':').unwrap().1
        ))
    }

    /// The whole capability on one board: two instances of one BGA library
    /// footprint and a heatsink QFN, imported verbatim with only instance
    /// clauses changed, identities remapped per instance, groups intact,
    /// every pad netted as planned, DNP and BOM fields from the schematic,
    /// the existing mounting hole, outline and setup untouched, and the
    /// libraries unchanged.
    #[tokio::test]
    async fn closed_initial_imports_library_text_verbatim_with_instance_identity() {
        let project = Project::new(BOARD, &netlist("2026-10-15T01:02:03"));
        let ctx = project.ctx("");
        let libraries_before = [
            std::fs::read(project.dir.join("Parts.pretty/Bga4.kicad_mod")).unwrap(),
            std::fs::read(heatsink_path(&project.dir)).unwrap(),
        ];

        let plan = project.dry_run(&ctx).await;
        assert_eq!(plan["status"], "ready", "{plan:#}");
        assert_eq!(plan["backend"], "closed_initial");
        assert_eq!(plan["coverage"]["footprints_added"]["planned"], 4);
        assert_eq!(plan["coverage"]["board_only_preserved"]["planned"], 1);
        assert_eq!(plan["board_closure"]["proven_closed"], true);
        assert_eq!(plan["report"]["planned_readback"]["verified"], true);
        assert_eq!(
            plan["report"]["planned_readback"]["pad_fabrication_properties_preserved"], 9,
            "four BGA pads on each instance and the QFN's heatsink pad"
        );
        assert_eq!(project.board_text(), BOARD, "a dry run writes nothing");

        // A new export of the same design stamps a new date: same revision.
        std::fs::write(&project.exported, netlist("2027-01-01T00:00:00")).unwrap();
        let again = project.dry_run(&ctx).await;
        assert_eq!(again["plan_revision"], plan["plan_revision"]);
        assert_eq!(again["report"]["instances"], plan["report"]["instances"]);

        let (is_error, applied) = project.apply(&ctx, &plan["plan_revision"]).await;
        assert!(!is_error, "{applied:#}");
        assert_eq!(applied["status"], "applied", "{applied:#}");
        assert_eq!(applied["write"]["persisted_readback"]["verified"], true);
        assert_eq!(applied["coverage"]["footprints_added"]["applied"], 4);

        let written = project.board_text();
        // Everything that was there is still there, byte for byte.
        for (start, end) in find_direct_child_blocks(BOARD, "kicad_pcb") {
            assert!(
                written.contains(&BOARD[start..end]),
                "{}",
                &BOARD[start..end]
            );
        }
        let header = &BOARD[..BOARD.find("\t(general").unwrap()];
        assert!(written.starts_with(header));
        // New footprints follow the existing one, before the drawings.
        assert!(written.ends_with(&BOARD[BOARD.find("\t(gr_rect").unwrap()..]));
        assert!(written.find("(footprint \"Parts:Bga4\"") < written.find("(gr_rect"));
        assert_eq!(
            written.matches("(property pad_prop_bga)").count(),
            8,
            "every BGA pad clause, byte for byte"
        );
        assert_eq!(written.matches("(property pad_prop_heatsink)").count(), 1);
        assert_eq!(written.matches("(solder_mask_margin 0.05)").count(), 2);
        assert_eq!(
            written.matches("(generator_version").count(),
            1,
            "library headers removed"
        );

        let root = konnect_sexp::parse_sexp(&written).unwrap();
        let (u1, u2) = (footprint(&root, "U1"), footprint(&root, "U2"));
        let (mut u1_ids, mut u2_ids) = (Vec::new(), Vec::new());
        identities(u1, &mut u1_ids);
        identities(u2, &mut u2_ids);
        let mut library_ids = Vec::new();
        identities(&konnect_sexp::parse_sexp(BGA).unwrap(), &mut library_ids);
        assert_eq!(
            u1_ids.len(),
            library_ids.len() + 3,
            "the root, MPN and LCSC identities are added"
        );
        for id in &u1_ids {
            assert!(!library_ids.contains(id), "library identity {id} copied");
            assert!(
                !u2_ids.contains(id),
                "identity {id} shared by two instances"
            );
            assert!(uuid::Uuid::parse_str(id).is_ok());
        }
        let mut all = Vec::new();
        identities(&root, &mut all);
        assert_eq!(
            all.iter().collect::<HashSet<_>>().len(),
            all.len(),
            "every identity on the board is unique"
        );
        for instance in [u1, u2] {
            let mut own = Vec::new();
            identities(instance, &mut own);
            let members = instance.find("group").unwrap().find("members").unwrap();
            let members = members.children().unwrap()[1..]
                .iter()
                .map(|member| member.as_str().unwrap().to_string())
                .collect::<Vec<_>>();
            assert_eq!(members.len(), 2);
            assert!(
                members.iter().all(|member| own.contains(member)),
                "{members:?}"
            );
        }

        // Nets: repeated numbers alike, the paste pad never.
        assert_eq!(
            pad_nets(u1),
            [
                ("A1", Some("GND")),
                ("A2", Some("+3V3")),
                ("B1", None),
                ("B2", Some("GND")),
                ("B2", Some("GND")),
                ("", None)
            ]
            .map(|(pad, net)| (pad.to_string(), net.map(str::to_string)))
        );
        assert!(pad_nets(footprint(&root, "U3"))
            .iter()
            .all(|(pad, net)| (pad == "21") == net.is_some()));
        assert!(written.contains("(net \"GND\")"), "KiCad 10 pad net names");

        // Instance clauses.
        assert_eq!(u1.get(1).and_then(SexpNode::as_str), Some("Parts:Bga4"));
        assert_eq!(
            u1.find_str("path"),
            Some("/11111111-1111-4111-8111-000000000001")
        );
        assert_eq!(u1.find_str("sheetfile"), Some("carrier.kicad_sch"));
        assert_eq!(field_value(u1, "Value", "value").as_deref(), Some("MCU"));
        assert_eq!(
            field_value(u1, "Datasheet", "").as_deref(),
            Some("https://example.com/mcu.pdf"),
            "the library's Datasheet property takes the schematic's text"
        );
        let mpn = u1
            .find_all("property")
            .into_iter()
            .find(|property| property.get(1).and_then(SexpNode::as_str) == Some("MPN"))
            .unwrap();
        assert_eq!(mpn.get(2).and_then(SexpNode::as_str), Some("MCU-123"));
        assert_eq!(mpn.find_str("hide"), Some("yes"));
        assert!(u1
            .find_all("property")
            .iter()
            .all(|property| property.get(1).and_then(SexpNode::as_str) != Some("Empty")));
        let attr = |node: &SexpNode| {
            node.find("attr").unwrap().children().unwrap()[1..]
                .iter()
                .map(|token| token.as_str().unwrap().to_string())
                .collect::<Vec<_>>()
        };
        assert_eq!(attr(u1), ["smd"]);
        assert_eq!(attr(u2), ["smd", "dnp"]);
        assert!(u1.find("model").is_some());
        assert_eq!(u1.find_str("net_tie_pad_groups"), Some("A1, A2"));

        // Parked beside the board, unrotated, on the front.
        for reference in ["U1", "U2", "U3", "C1"] {
            let at = footprint(&root, reference).find("at").unwrap();
            assert!(
                at.get_f64(1).unwrap() > 100.0,
                "{reference} parked right of the outline"
            );
            assert_eq!(at.get(3), None);
        }

        for (path, before) in [
            project.dir.join("Parts.pretty/Bga4.kicad_mod"),
            heatsink_path(&project.dir),
        ]
        .iter()
        .zip(&libraries_before)
        {
            assert_eq!(
                &std::fs::read(path).unwrap(),
                before,
                "library sources never change"
            );
        }

        // The live typed path still refuses the same library.
        let refused = super::super::prepare_footprint_source(BGA).unwrap_err();
        assert!(refused.1.contains("pad_prop_bga"), "{}", refused.1);

        // Initial only: the populated board refuses a second import.
        let second = project.dry_run(&ctx).await;
        assert_eq!(second["status"], "conflict", "{second:#}");
        let codes = second["diagnostics"]
            .as_array()
            .unwrap()
            .iter()
            .map(|diagnostic| diagnostic["code"].as_str().unwrap())
            .collect::<Vec<_>>();
        assert!(
            codes.contains(&"linked_circuit_footprint_present"),
            "{codes:?}"
        );
        assert!(codes.contains(&"netted_pad_present"), "{codes:?}");
    }

    /// Every input change invalidates a reviewed revision, and the board is
    /// never written on a stale one.
    #[tokio::test]
    async fn a_changed_library_schematic_or_board_invalidates_the_revision() {
        for change in ["library", "schematic", "board"] {
            let project = Project::new(BOARD, &netlist("2026-10-15T01:02:03"));
            let ctx = project.ctx("");
            let plan = project.dry_run(&ctx).await;
            assert_eq!(plan["status"], "ready", "{plan:#}");
            match change {
                "library" => std::fs::write(
                    project.dir.join("Parts.pretty/Bga4.kicad_mod"),
                    BGA.replace("(descr \"Test BGA", "(descr \"Changed BGA"),
                )
                .unwrap(),
                "schematic" => {
                    let mut text = std::fs::read_to_string(&project.schematic).unwrap();
                    text.push('\n');
                    std::fs::write(&project.schematic, text).unwrap();
                }
                _ => std::fs::write(
                    &project.board,
                    BOARD.replace("(paper \"A4\")", "(paper \"A3\")"),
                )
                .unwrap(),
            }
            let before = project.board_text();
            let (_, applied) = project.apply(&ctx, &plan["plan_revision"]).await;
            assert_eq!(applied["status"], "conflict", "{change}: {applied:#}");
            assert_eq!(
                applied["diagnostics"][0]["code"], "stale_plan_revision",
                "{change}"
            );
            assert_eq!(project.board_text(), before, "{change}: no write");
        }
    }

    #[tokio::test]
    async fn summary_and_full_use_the_same_revision_and_apply_preservation_checks() {
        let project = Project::new(BOARD, &netlist("d"));
        let ctx = project.ctx("");
        let full = project.dry_run(&ctx).await;
        let (error, summary) = project
            .call(&ctx, json!({"response_detail":"summary"}))
            .await;
        assert!(!error, "{summary:#}");
        assert_eq!(summary["plan_revision"], full["plan_revision"]);
        assert_eq!(summary["coverage"], full["coverage"]);
        assert_eq!(summary["diagnostics"], full["diagnostics"]);
        assert_eq!(
            summary["report"]["pad_fabrication_properties"],
            full["report"]["pad_fabrication_properties"]
        );
        assert!(summary.get("changes").is_none());
        assert!(summary["report"].get("instances").is_none());
        let (error, applied) = project
            .call(
                &ctx,
                json!({
                    "response_detail":"summary","dry_run":false,
                    "expected_plan_revision":summary["plan_revision"]
                }),
            )
            .await;
        assert!(!error, "{applied:#}");
        assert_eq!(applied["status"], "applied");
        assert_eq!(applied["write"]["persisted_readback"]["verified"], true);
        assert_eq!(applied["coverage"]["footprints_added"]["applied"], 4);
        assert!(project.board_text().contains("(property pad_prop_bga)"));
    }

    /// One bad pad, identity or group reference refuses the whole import,
    /// naming the affected parts, and writes nothing.
    #[tokio::test]
    async fn any_source_conflict_refuses_the_whole_import_without_writing() {
        let cases: [(&str, String, String); 3] = [
            (
                "footprint_pad_missing",
                BGA.to_string(),
                netlist("d").replace("(pin \"A2\")", "(pin \"Z9\")"),
            ),
            (
                "duplicate_library_identity",
                BGA.replace(
                    "5d0a6a10-0000-4000-8000-000000000024",
                    "5d0a6a10-0000-4000-8000-000000000023",
                ),
                netlist("d"),
            ),
            (
                "dangling_group_member",
                BGA.replace(
                    "(members \"5d0a6a10-0000-4000-8000-000000000010\"",
                    "(members \"5d0a6a10-0000-4000-8000-0000000000ff\"",
                ),
                netlist("d"),
            ),
        ];
        for (code, library, exported) in cases {
            let project = Project::new(BOARD, &exported);
            std::fs::write(project.dir.join("Parts.pretty/Bga4.kicad_mod"), &library).unwrap();
            let ctx = project.ctx("");
            let plan = project.dry_run(&ctx).await;
            assert_eq!(plan["status"], "conflict", "{code}: {plan:#}");
            let diagnostic = plan["diagnostics"]
                .as_array()
                .unwrap()
                .iter()
                .find(|diagnostic| diagnostic["code"] == code)
                .unwrap_or_else(|| panic!("{code}: {plan:#}"))
                .clone();
            assert!(
                diagnostic["references"]
                    .as_array()
                    .unwrap()
                    .contains(&json!("U1")),
                "{code}: {diagnostic}"
            );
            assert_eq!(plan["changes"], json!([]));
            let (_, applied) = project.apply(&ctx, &plan["plan_revision"]).await;
            assert_ne!(applied["status"], "applied", "{code}");
            assert_eq!(project.board_text(), BOARD, "{code}: no write");
        }
    }

    /// Initial only: existing circuit footprints, nets or routing refuse.
    #[tokio::test]
    async fn an_already_populated_or_routed_board_is_refused() {
        let close = BOARD.rfind("\t(embedded_fonts no)").unwrap();
        let with = |item: &str| format!("{}{item}\n{}", &BOARD[..close], &BOARD[close..]);
        let cases = [
            (
                "board_has_routing",
                with("\t(segment (start 1 1) (end 2 2) (width 0.2) (layer \"F.Cu\") (net \"X\") (uuid \"00000000-0000-4000-8000-0000000000c1\"))"),
            ),
            (
                "linked_circuit_footprint_present",
                with("\t(footprint \"Lib:R\" (layer \"F.Cu\") (uuid \"00000000-0000-4000-8000-0000000000c2\") (at 1 1) (property \"Reference\" \"R9\") (path \"/x/y\") (pad \"1\" smd rect (at 0 0) (size 1 1) (layers \"F.Cu\")))"),
            ),
            (
                "netted_pad_present",
                with("\t(footprint \"Lib:TP\" (layer \"F.Cu\") (uuid \"00000000-0000-4000-8000-0000000000c3\") (at 1 1) (property \"Reference\" \"TP9\") (pad \"1\" smd rect (at 0 0) (size 1 1) (layers \"F.Cu\") (net \"GND\")))"),
            ),
            (
                "circuit_footprint_present",
                with("\t(footprint \"Capacitor_SMD:C_0603_1608Metric\" (layer \"F.Cu\") (uuid \"00000000-0000-4000-8000-0000000000c4\") (at 1 1) (property \"Reference\" \"C1\") (pad \"1\" smd rect (at 0 0) (size 1 1) (layers \"F.Cu\")))"),
            ),
        ];
        for (code, board) in cases {
            let project = Project::new(&board, &netlist("d"));
            let ctx = project.ctx("");
            let plan = project.dry_run(&ctx).await;
            assert_eq!(plan["status"], "conflict", "{code}: {plan:#}");
            assert!(
                plan["diagnostics"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .any(|diagnostic| diagnostic["code"] == code),
                "{code}: {plan:#}"
            );
            assert_eq!(project.board_text(), board, "{code}");
        }
    }

    /// The board must be proven closed, even for a dry run: a KiCad holding
    /// it, a KiCad that rejects the check, a sibling lock, and a board this
    /// session saw live and can no longer reach all refuse with no write.
    #[tokio::test]
    async fn an_open_locked_rejecting_or_uncertain_board_is_never_written() {
        use crate::tools::pcb_board::board_mock::spawn_kicad_holding_board;

        // Open in KiCad.
        let project = Project::new(BOARD, &netlist("d"));
        let server = spawn_kicad_holding_board(&project.board, |_| None);
        let (is_error, plan) = project
            .call(&project.ctx(server.address()), json!({}))
            .await;
        assert!(is_error);
        assert!(
            plan["diagnostics"][0]["message"]
                .as_str()
                .unwrap()
                .contains("holds this board open"),
            "{plan:#}"
        );
        let (is_error, _) = project
            .call(
                &project.ctx(server.address()),
                json!({ "dry_run": false, "expected_plan_revision": "x" }),
            )
            .await;
        assert!(is_error);
        assert_eq!(project.board_text(), BOARD);
        drop(server);

        // Reachable and rejecting.
        let rejecting = crate::test_support::MockIpcServer::spawn("closed-rejecting", |_| {
            konnect_ipc::gen::kiapi::common::ApiResponse {
                status: Some(konnect_ipc::gen::kiapi::common::ApiResponseStatus {
                    status: konnect_ipc::gen::kiapi::common::ApiStatusCode::AsBadRequest as i32,
                    error_message: "mock rejects everything".to_string(),
                }),
                header: None,
                message: None,
            }
        });
        let (is_error, plan) = project
            .call(&project.ctx(rejecting.address()), json!({}))
            .await;
        assert!(is_error, "{plan:#}");
        assert_eq!(project.board_text(), BOARD);
        drop(rejecting);

        // A KiCad lock beside the board, IPC unreachable.
        let ctx = project.ctx("");
        let plan = project.dry_run(&ctx).await;
        assert_eq!(plan["status"], "ready", "{plan:#}");
        let lock = konnect_sexp::writer::kicad_editor_lock_path(&project.board).unwrap();
        std::fs::write(&lock, "user@host").unwrap();
        let (is_error, applied) = project.apply(&ctx, &plan["plan_revision"]).await;
        assert!(is_error, "{applied:#}");
        assert_eq!(project.board_text(), BOARD);
        std::fs::remove_file(&lock).unwrap();

        // Seen live by this session, now unreachable: the saved file may be
        // stale, and the structured refusal reaches the caller unchanged.
        let ctx = project.ctx("");
        ctx.board_session.observe_live(&project.board);
        let (is_error, refused) = project.apply(&ctx, &plan["plan_revision"]).await;
        assert!(is_error);
        assert_eq!(
            refused["error"]["kind"], "unsafe_file_fallback",
            "{refused:#}"
        );
        assert_eq!(project.board_text(), BOARD);
    }

    /// A numbered pre-10 net table gets coherent codes: existing names keep
    /// theirs, new nets continue the sequence, and every pad carries both.
    #[tokio::test]
    async fn a_numbered_net_table_is_extended_coherently() {
        let board = BOARD.replace(
            "\t(footprint \"MountingHole",
            "\t(net 0 \"\")\n\t(net 1 \"GND\")\n\t(footprint \"MountingHole",
        );
        let project = Project::new(&board, &netlist("d"));
        let ctx = project.ctx("");
        let plan = project.dry_run(&ctx).await;
        assert_eq!(plan["status"], "ready", "{plan:#}");
        assert_eq!(
            plan["report"]["net_table"]["new_numbered_nets"],
            json!({ "+3V3": 2, "/SIG": 3 })
        );
        let (_, applied) = project.apply(&ctx, &plan["plan_revision"]).await;
        assert_eq!(applied["status"], "applied", "{applied:#}");
        let written = project.board_text();
        assert!(
            written.contains("\t(net 1 \"GND\")\n\t(net 2 \"+3V3\")\n\t(net 3 \"/SIG\")"),
            "{written}"
        );
        let root = konnect_sexp::parse_sexp(&written).unwrap();
        let pad = footprint(&root, "C1")
            .find_all("pad")
            .into_iter()
            .find(|pad| pad.get(1).and_then(SexpNode::as_str) == Some("1"))
            .unwrap();
        let net = pad.find("net").unwrap();
        assert_eq!(konnect_sexp::net::net_id(net), Some("2"));
        assert_eq!(konnect_sexp::net::net_name(net), Some("+3V3"));
    }

    /// The readback that guards both the plan and the persisted file rejects
    /// each way a board can differ from the plan, not only the way the
    /// builder might get it wrong.
    #[test]
    fn the_readback_verifier_rejects_every_kind_of_drift() {
        let project = Project::new(BOARD, &netlist("d"));
        let source = netlist("d");
        let saved = SavedDesign {
            hierarchy: Vec::new(),
            design: super::super::parse_exported_netlist(&source).unwrap(),
            netlist_source: source,
        };
        let closed = plan(&project.board, &saved, Vec::new()).unwrap();
        assert_eq!(closed.plan.status, PlanStatus::Ready, "{:?}", closed.plan);
        let planned = closed.planned.clone().unwrap();
        let verify = |text: &str| {
            verify_board(
                &closed.original,
                text,
                &closed.expected,
                &closed.templates,
                &closed.new_nets,
                0,
            )
        };
        assert!(verify(&planned).is_ok());

        let members = planned.find("(members \"").unwrap() + "(members \"".len();
        let mut library_member = planned.clone();
        library_member.replace_range(
            members..members + 36,
            "5d0a6a10-0000-4000-8000-000000000010",
        );
        let root_uuid = &closed.expected[0].root_uuid;
        let other_uuid = &closed.expected[1].root_uuid;
        for (what, tampered) in [
            (
                "a dropped fabrication property",
                planned.replacen("(property pad_prop_bga)", "", 1),
            ),
            ("a group naming a library identity", library_member),
            (
                "a moved net",
                planned.replacen("(net \"+3V3\")", "(net \"GND\")", 1),
            ),
            (
                "a moved mounting hole",
                planned.replacen("(at 5 5)", "(at 6 5)", 1),
            ),
            (
                "a repeated instance identity",
                planned.replacen(other_uuid.as_str(), root_uuid, 1),
            ),
            (
                "a changed library graphic",
                planned.replacen("(end 2 -2)", "(end 2.1 -2)", 1),
            ),
            (
                "DNP cleared",
                planned.replacen("(attr smd dnp)", "(attr smd)", 1),
            ),
        ] {
            assert_ne!(tampered, planned, "{what}: the tamper must change the text");
            assert!(verify(&tampered).is_err(), "{what} must not verify");
        }
    }

    /// The default live backend's schematic update rebuilds an existing
    /// footprint whole, so a property-carrying one is refused at the dry run
    /// while an ordinary one is still planned.
    #[test]
    fn a_live_update_of_a_property_carrying_footprint_is_refused_at_the_dry_run() {
        let project = Project::new(BOARD, &netlist("d"));
        let existing = |reference: &str, kiid: &str, footprint_id: &str| BoardFootprint {
            kiid: kiid.to_string(),
            reference: reference.to_string(),
            value: "v".to_string(),
            footprint_id: footprint_id.to_string(),
            symbol_path: Some(format!("/{reference}")),
            pad_nets: BTreeMap::new(),
            pad_numbers: BTreeSet::new(),
            position: Point { x: 0.0, y: 0.0 },
            rotation: 0.0,
            layer: "F.Cu".to_string(),
            locked: false,
            dnp: false,
            not_in_schematic: false,
        };
        let board = BoardState {
            footprints: vec![
                existing("U1", "k-u1", "Parts:Bga4"),
                existing("C1", "k-c1", "Capacitor_SMD:C_0603_1608Metric"),
            ],
            routed_nets: BTreeMap::new(),
            bounds: Bounds {
                min_x: 0.0,
                min_y: 0.0,
                max_x: 0.0,
                max_y: 0.0,
            },
        };
        let update = |reference: &str, kiid: &str| PlannedChange::Update {
            kiid: kiid.to_string(),
            reference: reference.to_string(),
            value: "v2".to_string(),
            symbol_path: format!("/{reference}"),
            dnp: false,
            pad_nets: BTreeMap::new(),
            preserve: super::super::PreservedBoardState {
                position: Point { x: 0.0, y: 0.0 },
                rotation: 0.0,
                layer: "F.Cu".to_string(),
                locked: false,
            },
        };
        let mut plan = plan_sync("", &saved_design_stub(), &board);
        plan.changes = vec![update("U1", "k-u1"), update("C1", "k-c1")];
        let refused =
            super::super::updates_that_would_drop_pad_properties(&project.board, &plan, &board);
        assert_eq!(refused.len(), 1, "{refused:?}");
        assert_eq!(refused[0].code, "fabrication_property_rebuild_refused");
        assert_eq!(refused[0].references, ["U1"]);
    }

    fn saved_design_stub() -> super::super::ExportedDesign {
        super::super::ExportedDesign {
            components: Vec::new(),
            skipped: Vec::new(),
            unassigned: Vec::new(),
        }
    }

    /// Formats KiCad would re-read differently at the board's version are
    /// refused, rather than embedded with a silently changed meaning.
    #[test]
    fn an_older_library_format_that_would_change_meaning_is_refused() {
        let old = BGA
            .replace("(version 20260206)", "(version 20240108)")
            .replace("(solder_mask_margin 0.05)", "(solder_mask_margin 0)");
        let Err((code, reason)) = template_from_source(old, 20260206) else {
            panic!("a zero margin meant inherit before 20240201");
        };
        assert_eq!(code, "library_format_reinterpreted");
        assert!(reason.contains("solder_mask_margin"), "{reason}");
        assert!(template_from_source(
            BGA.replace("(version 20260206)", "(version 20240108)"),
            20260206
        )
        .is_ok());
        assert!(
            template_from_source(BGA.to_string(), 20250101).is_err(),
            "newer than the board"
        );
    }

    /// Reference and Value in the older fp_text spelling resolve through the
    /// one shared reader, and never to a nested pad property.
    #[test]
    fn fp_text_fields_and_modern_properties_share_one_reader() {
        let old = "(footprint \"X\" (layer \"F.Cu\")\n\t(fp_text reference \"REF**\" (at 0 0) (layer \"F.SilkS\"))\n\t(fp_text value \"X\" (at 0 1) (layer \"F.Fab\"))\n\t(pad \"1\" smd rect (at 0 0) (size 1 1) (layers \"F.Cu\") (property pad_prop_bga)))";
        let (start, end) = footprint_field_value_span(old, "Value").unwrap();
        assert_eq!(&old[start..end], "\"X\"");
        let (start, end) = footprint_field_value_span(old, "Reference").unwrap();
        assert_eq!(&old[start..end], "\"REF**\"");
        let (start, end) = footprint_field_value_span(BGA, "Value").unwrap();
        assert_eq!(&BGA[start..end], "\"Bga4\"");
        assert_eq!(footprint_field_value_span(BGA, "pad_prop_bga"), None);
    }
}
