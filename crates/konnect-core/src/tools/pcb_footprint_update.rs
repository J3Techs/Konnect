use crate::mcp::protocol::{CallToolResult, ToolContent};
use crate::tool;
use crate::tools::{
    pcb_board::{attempt_ipc_write, BoardWrite},
    ToolContext, ToolDef,
};
use anyhow::{bail, Context, Result};
use konnect_ipc::gen::kiapi;
use prost::Message;
use serde::Serialize;
use serde_json::json;
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

#[derive(Debug, Clone)]
struct LibraryFootprint {
    library_id: String,
    definition: kiapi::board::types::Footprint,
    attributes: kiapi::board::types::FootprintAttributes,
    datasheet: Option<String>,
    description_field: Option<String>,
    properties: Vec<kiapi::board::types::Field>,
    pads: Vec<konnect_ipc::IpcPadDefinition>,
    graphics: Vec<konnect_ipc::IpcGraphicDefinition>,
    models: Vec<kiapi::board::types::Footprint3DModel>,
}

#[derive(Debug)]
struct ParsedLibraryProperties {
    datasheet: Option<String>,
    description: Option<String>,
    custom: Vec<kiapi::board::types::Field>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize)]
#[serde(rename_all = "snake_case")]
enum ChangedDomain {
    Pads,
    Graphics,
    Attributes,
    Metadata,
    Models,
}

#[derive(Debug)]
struct PreparedUpdate {
    item: prost_types::Any,
    changed_domains: BTreeSet<ChangedDomain>,
    preserved: PreservedState,
    pad_differences: Vec<PadDifference>,
    pad_differences_omitted: usize,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize)]
struct UpdateFilters {
    references: Option<BTreeSet<String>>,
    library_ids: Option<BTreeSet<String>>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct ArgumentError {
    field: String,
    reason: String,
}

impl std::fmt::Display for ArgumentError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(formatter, "{}: {}", self.field, self.reason)
    }
}

impl std::error::Error for ArgumentError {}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
enum PlanStatus {
    Ready,
    Noop,
    Conflict,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize)]
struct CountPair {
    planned: usize,
    applied: usize,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize)]
struct UpdateCoverage {
    selected: CountPair,
    changed: CountPair,
    unchanged: CountPair,
    skipped_unlinked: CountPair,
    conflicts: CountPair,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
struct PreservedState {
    position: bool,
    rotation: bool,
    layer: bool,
    locked: bool,
    kiid: bool,
    symbol_path: bool,
    pad_nets: bool,
    instance_overrides: bool,
}

impl PreservedState {
    /// Derived by comparing the rebuilt instance against the one read from
    /// the board — never asserted from policy. If a future change to
    /// `build_updated_instance` stops carrying one of these across, the
    /// response says so instead of echoing the intent.
    fn derive(
        current: &kiapi::board::types::FootprintInstance,
        updated: &kiapi::board::types::FootprintInstance,
        old_nets: &BTreeMap<String, kiapi::board::types::Net>,
    ) -> Self {
        let new_nets: BTreeMap<String, String> = updated
            .definition
            .iter()
            .flat_map(|definition| definition.items.iter())
            .filter(|item| item.type_url.ends_with("kiapi.board.types.Pad"))
            .filter_map(|item| kiapi::board::types::Pad::decode(item.value.as_slice()).ok())
            .filter_map(|pad| pad.net.map(|net| (pad.number, net.name)))
            .collect();
        Self {
            position: updated.position == current.position,
            rotation: updated.orientation == current.orientation,
            layer: updated.layer == current.layer,
            locked: updated.locked == current.locked,
            kiid: updated.id == current.id,
            symbol_path: updated.symbol_path == current.symbol_path
                && updated.symbol_sheet_name == current.symbol_sheet_name
                && updated.symbol_sheet_filename == current.symbol_sheet_filename,
            pad_nets: old_nets.iter().all(|(number, net)| {
                new_nets.get(number).map(String::as_str) == Some(net.name.as_str())
            }),
            instance_overrides: updated.overrides == current.overrides,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
struct PlannedUpdate {
    reference: String,
    library_id: String,
    changed_domains: BTreeSet<ChangedDomain>,
    preserved: PreservedState,
    /// Which pads differ and in which typed fields; see [`pad_differences`].
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pad_differences: Vec<PadDifference>,
    #[serde(skip_serializing_if = "is_zero")]
    pad_differences_omitted: usize,
}

/// Pad differences on a footprint the plan leaves unchanged, all of them
/// ignored by the plan, reported because they can still matter natively.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
struct PadParityNote {
    reference: String,
    library_id: String,
    pad_differences: Vec<PadDifference>,
    #[serde(skip_serializing_if = "is_zero")]
    pad_differences_omitted: usize,
}

fn is_zero(value: &usize) -> bool {
    *value == 0
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
struct UpdateDiagnostic {
    code: String,
    message: String,
    reference: Option<String>,
}

#[derive(Debug)]
struct UpdatePlan {
    status: PlanStatus,
    plan_revision: String,
    coverage: UpdateCoverage,
    changes: Vec<PlannedUpdate>,
    pad_parity_notes: Vec<PadParityNote>,
    diagnostics: Vec<UpdateDiagnostic>,
    prepared_items: Vec<prost_types::Any>,
}

pub(crate) fn tool() -> ToolDef {
    tool!(
        "update_footprints_from_library",
        "Plan or atomically apply KiCad's Update Footprints from Library operation to placed \
         footprints on the live board. Defaults to a non-mutating dry run; apply requires its \
         exact plan revision. Preserves placed-instance state and pad nets while refreshing \
         supported library-owned content.",
        json!({
            "type": "object",
            "properties": {
                "board": {
                    "type": "string",
                    "description": "Open .kicad_pcb path whose placed footprints will be refreshed"
                },
                "references": {
                    "type": "array",
                    "items": { "type": "string" },
                    "description": "Optional exact reference allowlist; omitted means all eligible footprints"
                },
                "library_ids": {
                    "type": "array",
                    "items": { "type": "string" },
                    "description": "Optional exact Library:Footprint allowlist"
                },
                "dry_run": {
                    "type": "boolean",
                    "default": true,
                    "description": "Plan and report changes without mutating the board"
                },
                "expected_plan_revision": {
                    "type": "string",
                    "description": "Required for apply; exact revision returned by the reviewed dry run"
                }
            },
            "required": ["board"],
            "additionalProperties": false
        }),
        |args, ctx| async move { handle_update_footprints_from_library(args, ctx).await }
    )
    .with_board_access(crate::tools::BoardAccess::LiveOnly)
}

pub(crate) async fn handle_update_footprints_from_library(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> Result<CallToolResult> {
    use kiapi::common::types::KiCadObjectType as ObjectType;

    let board_path = crate::tools::get_path(args, "board")?;
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
    if !board_path.is_file() {
        return Ok(CallToolResult::error_kind(
            crate::mcp::error::ToolErrorKind::FileNotFound {
                path: board_path.display().to_string(),
            },
            format!("Board file not found: {}", board_path.display()),
        ));
    }
    let filters = match parse_filters(args) {
        Ok(filters) => filters,
        Err(error) => {
            return Ok(CallToolResult::error_kind(
                crate::mcp::error::ToolErrorKind::InvalidArgument {
                    field: error.field.clone(),
                    reason: error.reason.clone(),
                },
                format!("Argument '{}' is invalid: {}", error.field, error.reason),
            ))
        }
    };

    let ipc_board_path = board_path.clone();
    let planning_board_path = board_path.clone();
    let operation = if dry_run {
        "footprint library update dry run"
    } else {
        "footprint library update apply"
    };
    let result = attempt_ipc_write(
        ctx,
        &board_path,
        operation,
        move |client| {
            let document = client.find_open_board(&ipc_board_path)?;
            let footprint_items =
                client.get_items_in(document.clone(), ObjectType::KotPcbFootprint)?;
            let nets = client.get_nets_in(document.clone())?;
            let net_codes = nets
                .into_iter()
                .map(|net| (net.name, net.netcode))
                .collect::<BTreeMap<_, _>>();
            let routed_nets = snapshot_routed_nets(client, document.clone())?;
            let mut plan = plan_updates(
                &planning_board_path,
                &footprint_items,
                &net_codes,
                &routed_nets,
                &filters,
            );
            refuse_updates_that_would_drop_pad_properties(&planning_board_path, &mut plan);

            if dry_run || plan.status == PlanStatus::Conflict {
                return Ok(plan_response(&plan, false));
            }
            if expected_revision.as_deref() != Some(plan.plan_revision.as_str()) {
                plan.status = PlanStatus::Conflict;
                plan.coverage.conflicts.planned += 1;
                plan.diagnostics.push(UpdateDiagnostic {
                    code: "stale_plan_revision".to_string(),
                    message: "The live board, selected libraries, or filters changed; rerun dry run and apply its new plan revision."
                        .to_string(),
                    reference: None,
                });
                plan.changes.clear();
                plan.prepared_items.clear();
                return Ok(plan_response(&plan, false));
            }
            if plan.status == PlanStatus::Noop {
                return Ok(plan_response(&plan, false));
            }

            let update_count = plan.prepared_items.len();
            client.run_commit("Update footprints from libraries", |client| {
                client.update_items_in(document, std::mem::take(&mut plan.prepared_items))
            })?;
            plan.coverage.selected.applied = plan.coverage.selected.planned;
            plan.coverage.changed.applied = update_count;
            plan.coverage.unchanged.applied = plan.coverage.unchanged.planned;
            plan.coverage.skipped_unlinked.applied = plan.coverage.skipped_unlinked.planned;
            Ok(plan_response(&plan, true))
        },
    )
    .await?;

    Ok(match result {
        BoardWrite::Ipc(response) => response,
        BoardWrite::File(reason) => preflight_conflict(format!(
            "{} update_footprints_from_library is live-IPC-only and never edits the board \
             file directly. Open the requested board in KiCad and retry.",
            reason.premise()
        )),
        BoardWrite::Refused(result) => {
            let message = result
                .content
                .into_iter()
                .find_map(|content| match content {
                    ToolContent::Text { text } => Some(text),
                    _ => None,
                })
                .unwrap_or_else(|| "KiCad refused the footprint library update".to_string());
            preflight_conflict(message)
        }
    })
}

/// Each prepared update sends a whole footprint back, and KiCad re-creates
/// its pads from a message with no field for a pad's fabrication property.
/// The library side is already refused by the typed reader; this refuses a
/// placed footprint whose saved board copy carries one, or that neither the
/// saved board nor its library can speak for.
fn refuse_updates_that_would_drop_pad_properties(board_path: &Path, plan: &mut UpdatePlan) {
    use crate::tools::footprint_fabrication::{check_live_rebuild, RebuildTarget, Remedy};

    let targets = plan
        .prepared_items
        .iter()
        .filter_map(|item| {
            kiapi::board::types::FootprintInstance::decode(item.value.as_slice()).ok()
        })
        .map(|footprint| RebuildTarget::from_instance(&footprint))
        .collect::<Vec<_>>();
    let Err(refusal) = check_live_rebuild(
        board_path,
        "update_footprints_from_library",
        &targets,
        Remedy::NoLosslessPath,
    ) else {
        return;
    };
    for finding in &refusal.findings {
        plan.diagnostics.push(UpdateDiagnostic {
            code: "fabrication_property_rebuild_refused".to_string(),
            message: format!(
                "{} A live library refresh would re-create its pads without it.",
                finding.reason()
            ),
            reference: Some(finding.reference.clone()),
        });
    }
    plan.status = PlanStatus::Conflict;
    plan.coverage.conflicts.planned += refusal.findings.len();
    plan.changes.clear();
    plan.prepared_items.clear();
}

fn snapshot_routed_nets(
    client: &konnect_ipc::KiCadIpcClient,
    document: kiapi::common::types::DocumentSpecifier,
) -> Result<BTreeSet<String>> {
    use kiapi::common::types::KiCadObjectType as ObjectType;

    let mut routed = BTreeSet::new();
    for item in client.get_items_in(document.clone(), ObjectType::KotPcbTrace)? {
        if let Ok(track) = kiapi::board::types::Track::decode(item.value.as_slice()) {
            if let Some(net) = track.net.filter(|net| !net.name.is_empty()) {
                routed.insert(net.name);
            }
        }
    }
    for item in client.get_items_in(document.clone(), ObjectType::KotPcbArc)? {
        if let Ok(arc) = kiapi::board::types::Arc::decode(item.value.as_slice()) {
            if let Some(net) = arc.net.filter(|net| !net.name.is_empty()) {
                routed.insert(net.name);
            }
        }
    }
    for item in client.get_items_in(document.clone(), ObjectType::KotPcbVia)? {
        if let Ok(via) = kiapi::board::types::Via::decode(item.value.as_slice()) {
            if let Some(net) = via.net.filter(|net| !net.name.is_empty()) {
                routed.insert(net.name);
            }
        }
    }
    if !client
        .get_items_in(document.clone(), ObjectType::KotPcbZone)?
        .is_empty()
    {
        routed.extend(
            client
                .get_nets_in(document)?
                .into_iter()
                .filter(|net| !net.name.is_empty())
                .map(|net| net.name),
        );
    }
    Ok(routed)
}

fn plan_response(plan: &UpdatePlan, applied: bool) -> CallToolResult {
    CallToolResult::json(&json!({
        "status": if applied {
            "applied"
        } else {
            match plan.status {
                PlanStatus::Ready => "ready",
                PlanStatus::Noop => "noop",
                PlanStatus::Conflict => "conflict",
            }
        },
        "plan_revision": plan.plan_revision,
        "coverage": {
            "transport": "live_kicad_ipc",
            "atomicity": "single_kicad_undo_commit",
            "selected": plan.coverage.selected,
            "changed": plan.coverage.changed,
            "unchanged": plan.coverage.unchanged,
            "skipped_unlinked": plan.coverage.skipped_unlinked,
            "conflicts": plan.coverage.conflicts
        },
        "changes": plan.changes,
        "pad_parity_notes": plan.pad_parity_notes,
        "diagnostics": plan.diagnostics,
        "undo": if applied {
            Some("Ctrl-Z reverses the whole footprint library update.")
        } else {
            None
        }
    }))
}

fn preflight_conflict(message: impl Into<String>) -> CallToolResult {
    CallToolResult::json(&json!({
        "status": "conflict",
        "coverage": {
            "transport": "live_kicad_ipc",
            "atomicity": "single_kicad_undo_commit",
            "selected": CountPair::default(),
            "changed": CountPair::default(),
            "unchanged": CountPair::default(),
            "skipped_unlinked": CountPair::default(),
            "conflicts": CountPair { planned: 1, applied: 0 }
        },
        "changes": [],
        "diagnostics": [{
            "code": "preflight_conflict",
            "message": message.into()
        }],
        "undo": null
    }))
}

fn parse_filters(args: &serde_json::Value) -> std::result::Result<UpdateFilters, ArgumentError> {
    fn parse(
        args: &serde_json::Value,
        field: &str,
    ) -> std::result::Result<Option<BTreeSet<String>>, ArgumentError> {
        let Some(value) = args.get(field) else {
            return Ok(None);
        };
        let values = value.as_array().ok_or_else(|| ArgumentError {
            field: field.to_string(),
            reason: "must be an array of strings when supplied".to_string(),
        })?;
        let parsed = values
            .iter()
            .map(|value| {
                let value = value.as_str().ok_or_else(|| ArgumentError {
                    field: field.to_string(),
                    reason: "must contain only strings".to_string(),
                })?;
                if value.is_empty() {
                    return Err(ArgumentError {
                        field: field.to_string(),
                        reason: "must not contain empty strings".to_string(),
                    });
                }
                Ok(value.to_string())
            })
            .collect::<std::result::Result<BTreeSet<_>, _>>()
            .map(Some)?;
        if field == "library_ids" {
            if let Some(invalid) = parsed.as_ref().and_then(|values| {
                values.iter().find(|value| {
                    value.split_once(':').is_none_or(|(nickname, entry)| {
                        nickname.is_empty()
                            || entry.is_empty()
                            || value.contains('/')
                            || value.contains('\\')
                    })
                })
            }) {
                return Err(ArgumentError {
                    field: field.to_string(),
                    reason: format!(
                        "'{invalid}' must use a non-empty Library:Footprint identifier"
                    ),
                });
            }
        }
        Ok(parsed)
    }

    Ok(UpdateFilters {
        references: parse(args, "references")?,
        library_ids: parse(args, "library_ids")?,
    })
}

fn plan_updates(
    board_path: &Path,
    footprint_items: &[prost_types::Any],
    net_codes: &BTreeMap<String, i32>,
    routed_nets: &BTreeSet<String>,
    filters: &UpdateFilters,
) -> UpdatePlan {
    #[derive(Debug)]
    struct Candidate {
        reference: String,
        library_id: String,
        item: prost_types::Any,
        instance: kiapi::board::types::FootprintInstance,
    }

    let mut coverage = UpdateCoverage::default();
    let mut diagnostics = Vec::new();
    let mut candidates = Vec::new();
    let mut reference_counts = BTreeMap::<String, usize>::new();

    for item in footprint_items {
        let instance = match kiapi::board::types::FootprintInstance::decode(item.value.as_slice()) {
            Ok(instance) => instance,
            Err(error) => {
                diagnostics.push(UpdateDiagnostic {
                    code: "invalid_board_footprint".to_string(),
                    message: format!("KiCad returned an invalid footprint: {error}"),
                    reference: None,
                });
                continue;
            }
        };
        let reference = field_text(&instance.reference_field);
        *reference_counts.entry(reference.clone()).or_insert(0) += 1;
        let library_id = instance
            .definition
            .as_ref()
            .and_then(|definition| definition.id.as_ref())
            .map(|id| format!("{}:{}", id.library_nickname, id.entry_name))
            .unwrap_or_default();
        candidates.push(Candidate {
            reference,
            library_id,
            item: item.clone(),
            instance,
        });
    }

    if let Some(references) = filters.references.as_ref() {
        for reference in references {
            match reference_counts.get(reference).copied().unwrap_or(0) {
                0 => diagnostics.push(UpdateDiagnostic {
                    code: "reference_not_found".to_string(),
                    message: format!("footprint reference '{reference}' was not found"),
                    reference: Some(reference.clone()),
                }),
                count if count > 1 => diagnostics.push(UpdateDiagnostic {
                    code: "duplicate_reference".to_string(),
                    message: format!("footprint reference '{reference}' appears {count} times"),
                    reference: Some(reference.clone()),
                }),
                _ => {}
            }
        }
    } else {
        for (reference, count) in &reference_counts {
            if *count > 1 {
                diagnostics.push(UpdateDiagnostic {
                    code: "duplicate_reference".to_string(),
                    message: format!("footprint reference '{reference}' appears {count} times"),
                    reference: Some(reference.clone()),
                });
            }
        }
    }

    candidates.sort_by(|left, right| {
        left.reference
            .cmp(&right.reference)
            .then_with(|| left.library_id.cmp(&right.library_id))
    });
    let selected = candidates
        .into_iter()
        .filter(|candidate| {
            filters
                .references
                .as_ref()
                .is_none_or(|references| references.contains(&candidate.reference))
        })
        .filter(|candidate| {
            filters
                .library_ids
                .as_ref()
                .is_none_or(|ids| ids.contains(&candidate.library_id))
        })
        .collect::<Vec<_>>();

    coverage.skipped_unlinked.planned = selected
        .iter()
        .filter(|candidate| candidate.library_id.is_empty())
        .count();
    coverage.selected.planned = selected
        .iter()
        .filter(|candidate| !candidate.library_id.is_empty())
        .count();
    if filters.references.is_some() {
        for candidate in selected
            .iter()
            .filter(|candidate| candidate.library_id.is_empty())
        {
            diagnostics.push(UpdateDiagnostic {
                code: "unlinked_footprint".to_string(),
                message: format!(
                    "footprint '{}' has no resolvable Library:Footprint identifier",
                    candidate.reference
                ),
                reference: Some(candidate.reference.clone()),
            });
        }
    }

    let mut hasher = Sha256::new();
    hasher.update(serde_json::to_vec(filters).expect("filters serialize"));
    let mut changes = Vec::new();
    let mut pad_parity_notes = Vec::new();
    let mut prepared_items = Vec::new();
    for candidate in selected {
        if candidate.library_id.is_empty() {
            continue;
        }
        let Some((nickname, entry)) = candidate.library_id.split_once(':') else {
            diagnostics.push(UpdateDiagnostic {
                code: "invalid_library_id".to_string(),
                message: format!(
                    "footprint '{}' has malformed library id '{}'",
                    candidate.reference, candidate.library_id
                ),
                reference: Some(candidate.reference),
            });
            continue;
        };
        if nickname.is_empty() || entry.is_empty() {
            diagnostics.push(UpdateDiagnostic {
                code: "invalid_library_id".to_string(),
                message: format!(
                    "footprint '{}' has malformed library id '{}'",
                    candidate.reference, candidate.library_id
                ),
                reference: Some(candidate.reference),
            });
            continue;
        }

        let library_path = match super::library::resolve_footprint_path(
            &candidate.library_id,
            board_path.parent(),
        ) {
            Ok(path) => path,
            Err(message) => {
                diagnostics.push(UpdateDiagnostic {
                    code: "footprint_library_resolution_failed".to_string(),
                    message,
                    reference: Some(candidate.reference),
                });
                continue;
            }
        };
        let source = match std::fs::read_to_string(&library_path) {
            Ok(source) => source,
            Err(error) => {
                diagnostics.push(UpdateDiagnostic {
                    code: "footprint_library_read_failed".to_string(),
                    message: format!("failed to read {}: {error}", library_path.display()),
                    reference: Some(candidate.reference),
                });
                continue;
            }
        };
        let library = match parse_library_footprint(&candidate.library_id, &source) {
            Ok(library) => library,
            Err(error) => {
                diagnostics.push(UpdateDiagnostic {
                    code: "unsupported_library_footprint".to_string(),
                    message: format!("{error:#}"),
                    reference: Some(candidate.reference),
                });
                continue;
            }
        };
        let prepared =
            match build_updated_instance(&candidate.instance, &library, net_codes, routed_nets) {
                Ok(prepared) => prepared,
                Err(error) => {
                    diagnostics.push(UpdateDiagnostic {
                        code: "footprint_update_conflict".to_string(),
                        message: format!("{error:#}"),
                        reference: Some(candidate.reference),
                    });
                    continue;
                }
            };

        hasher.update(candidate.item.type_url.as_bytes());
        hasher.update(&candidate.item.value);
        hasher.update(candidate.library_id.as_bytes());
        hasher.update(source.as_bytes());
        if prepared.changed_domains.is_empty() {
            coverage.unchanged.planned += 1;
            if !prepared.pad_differences.is_empty() {
                pad_parity_notes.push(PadParityNote {
                    reference: candidate.reference,
                    library_id: candidate.library_id,
                    pad_differences: prepared.pad_differences,
                    pad_differences_omitted: prepared.pad_differences_omitted,
                });
            }
        } else {
            coverage.changed.planned += 1;
            changes.push(PlannedUpdate {
                reference: candidate.reference,
                library_id: candidate.library_id,
                changed_domains: prepared.changed_domains,
                preserved: prepared.preserved,
                pad_differences: prepared.pad_differences,
                pad_differences_omitted: prepared.pad_differences_omitted,
            });
            prepared_items.push(prepared.item);
        }
    }

    if !diagnostics.is_empty() {
        coverage.conflicts.planned = diagnostics.len();
        changes.clear();
        pad_parity_notes.clear();
        prepared_items.clear();
        coverage.changed.planned = 0;
        return UpdatePlan {
            status: PlanStatus::Conflict,
            plan_revision: format!("{:x}", hasher.finalize()),
            coverage,
            changes,
            pad_parity_notes,
            diagnostics,
            prepared_items,
        };
    }

    let status = if changes.is_empty() {
        PlanStatus::Noop
    } else {
        PlanStatus::Ready
    };
    UpdatePlan {
        status,
        plan_revision: format!("{:x}", hasher.finalize()),
        coverage,
        changes,
        pad_parity_notes,
        diagnostics,
        prepared_items,
    }
}

fn parse_library_footprint(library_id: &str, source: &str) -> Result<LibraryFootprint> {
    let (library_nickname, entry_name) = library_id
        .split_once(':')
        .filter(|(nickname, entry)| !nickname.is_empty() && !entry.is_empty())
        .context("footprint identifier must use non-empty Library:Footprint syntax")?;
    let root = konnect_sexp::parse_sexp(source).context("invalid footprint S-expression")?;
    if root.head() != Some("footprint") {
        bail!("library source root must be a footprint");
    }

    validate_supported_children(&root)?;
    let properties = parse_library_properties(&root)?;
    let pads = super::pcb_components::extract_pad_definitions(source)?;
    // Custom properties travel as typed Field items below. Treating visible
    // properties as generic graphics as well would duplicate their text.
    let graphics = super::pcb_components::extract_graphic_definitions_without_properties(source)?;
    let models = super::library_footprint::models(&root)?;
    let attributes = super::library_footprint::attributes(&root)?;
    let definition = kiapi::board::types::Footprint {
        id: Some(kiapi::common::types::LibraryIdentifier {
            library_nickname: library_nickname.to_string(),
            entry_name: entry_name.to_string(),
        }),
        attributes: Some(super::library_footprint::description_and_keywords(&root)),
        ..Default::default()
    };

    Ok(LibraryFootprint {
        library_id: library_id.to_string(),
        definition,
        attributes,
        datasheet: properties.datasheet,
        description_field: properties.description,
        properties: properties.custom,
        pads,
        graphics,
        models,
    })
}

fn parse_library_properties(root: &konnect_sexp::SexpNode) -> Result<ParsedLibraryProperties> {
    let mut names = BTreeSet::new();
    let mut datasheet = None;
    let mut description = None;
    let mut custom = Vec::new();
    for property in root.find_all("property") {
        let name = property
            .get(1)
            .and_then(konnect_sexp::SexpNode::as_str)
            .context("property is missing its name")?;
        if !names.insert(name.to_string()) {
            bail!("property '{name}' appears more than once in the library footprint");
        }

        // Mandatory and custom properties share one lossless clause validator.
        // The mandatory values keep their existing first-class IPC fields; the
        // shared parser proves that none of their authored clauses would be
        // silently ignored without requiring a typed custom Field.
        let mandatory = matches!(name, "Reference" | "Value" | "Datasheet" | "Description");
        let parsed = parse_library_property(property, !mandatory)?;
        let value = property
            .get(2)
            .and_then(konnect_sexp::SexpNode::as_str)
            .with_context(|| format!("property '{name}' is missing its value"))?;
        match name {
            "Reference" | "Value" => {}
            "Datasheet" => datasheet = Some(value.to_string()),
            "Description" => description = Some(value.to_string()),
            _ => custom.push(parsed.context("custom property did not produce a typed field")?),
        }
    }

    Ok(ParsedLibraryProperties {
        datasheet,
        description,
        custom,
    })
}

/// Convert a footprint property into the typed `Field` shape carried by
/// KiCad's IPC model. Mandatory properties are validated through this same
/// path and then stored in their existing first-class fields. Unknown clauses
/// refuse here: accepting a property while dropping part of its authored
/// presentation would make the refresh lossy even when its value survived.
fn parse_library_property(
    property: &konnect_sexp::SexpNode,
    require_typed_field: bool,
) -> Result<Option<kiapi::board::types::Field>> {
    use kiapi::common::types::LockedState;

    let name = property
        .get(1)
        .and_then(konnect_sexp::SexpNode::as_str)
        .context("property is missing its name")?;
    let value = property
        .get(2)
        .and_then(konnect_sexp::SexpNode::as_str)
        .with_context(|| format!("property '{name}' is missing its value"))?;
    let mut position = None;
    let mut rotation = 0.0;
    let mut layer = None;
    let mut hidden = None;
    let mut knockout = None;
    let mut attributes = None;
    let mut identifier = None;

    for clause in property.children().unwrap_or_default().iter().skip(3) {
        let tag = clause
            .head()
            .with_context(|| format!("property '{name}' contains an unsupported atom"))?;
        match tag {
            "at" => {
                if position.is_some() {
                    bail!("property '{name}' contains duplicate 'at' clauses");
                }
                let count = clause.children().map_or(0, |children| children.len());
                if !matches!(count, 3 | 4) {
                    bail!("property '{name}' 'at' must contain x, y, and optional rotation");
                }
                let x = clause
                    .get_f64(1)
                    .with_context(|| format!("property '{name}' has an invalid X position"))?;
                let y = clause
                    .get_f64(2)
                    .with_context(|| format!("property '{name}' has an invalid Y position"))?;
                rotation = clause.get_f64(3).unwrap_or(0.0);
                if !x.is_finite() || !y.is_finite() || !rotation.is_finite() {
                    bail!("property '{name}' position and rotation must be finite");
                }
                position = Some(konnect_ipc::builders::vec2(x, y));
            }
            "layer" => {
                if layer.is_some() {
                    bail!("property '{name}' contains duplicate 'layer' clauses");
                }
                let layer_name = clause
                    .get(1)
                    .and_then(konnect_sexp::SexpNode::as_str)
                    .filter(|name| !name.is_empty())
                    .with_context(|| format!("property '{name}' has no layer name"))?;
                if clause.children().map_or(0, |children| children.len()) != 2 {
                    bail!("property '{name}' 'layer' must name exactly one layer");
                }
                layer = Some(
                    konnect_ipc::builders::try_layer_from_name(layer_name)
                        .with_context(|| format!("property '{name}' has an unsupported layer"))?
                        as i32,
                );
            }
            "hide" => {
                if hidden.is_some() {
                    bail!("property '{name}' contains duplicate 'hide' clauses");
                }
                hidden = Some(property_yes_no(clause, name, "hide")?);
            }
            "knockout" => {
                if knockout.is_some() {
                    bail!("property '{name}' contains duplicate 'knockout' clauses");
                }
                knockout = Some(property_yes_no(clause, name, "knockout")?);
            }
            "uuid" | "tstamp" => {
                if let Some(previous) = identifier {
                    bail!(
                        "property '{name}' contains multiple identifier clauses ('{previous}' and '{tag}')"
                    );
                }
                identifier = Some(tag);
                if clause.children().map_or(0, |children| children.len()) != 2
                    || clause
                        .get(1)
                        .and_then(konnect_sexp::SexpNode::as_str)
                        .is_none_or(str::is_empty)
                {
                    bail!("property '{name}' '{tag}' must contain exactly one identifier");
                }
            }
            "effects" => {
                if attributes.is_some() {
                    bail!("property '{name}' contains duplicate 'effects' clauses");
                }
                attributes = Some(parse_property_effects(clause, name)?);
            }
            unsupported => {
                bail!("property '{name}' clause '{unsupported}' is not supported losslessly")
            }
        }
    }

    if !require_typed_field {
        return Ok(None);
    }

    let position =
        position.with_context(|| format!("property '{name}' is missing its 'at' clause"))?;
    let layer =
        layer.with_context(|| format!("property '{name}' is missing its 'layer' clause"))?;
    let mut attributes =
        attributes.with_context(|| format!("property '{name}' is missing its 'effects' clause"))?;
    attributes.angle = Some(kiapi::common::types::Angle {
        value_degrees: rotation,
    });
    Ok(Some(kiapi::board::types::Field {
        id: None,
        name: name.to_string(),
        text: Some(kiapi::board::types::BoardText {
            // A library child's UUID is definition-local and cannot be reused
            // across placed instances. Let KiCad assign the board child ID.
            id: None,
            text: Some(kiapi::common::types::Text {
                position: Some(position),
                attributes: Some(attributes),
                text: value.to_string(),
                hyperlink: String::new(),
            }),
            layer,
            knockout: knockout.unwrap_or(false),
            locked: LockedState::LsUnlocked as i32,
            parent: None,
        }),
        visible: !hidden.unwrap_or(false),
    }))
}

fn property_yes_no(clause: &konnect_sexp::SexpNode, name: &str, tag: &str) -> Result<bool> {
    if clause.children().map_or(0, |children| children.len()) != 2 {
        bail!("property '{name}' '{tag}' must contain exactly one yes/no value");
    }
    match clause.get(1).and_then(konnect_sexp::SexpNode::as_str) {
        Some("yes") => Ok(true),
        Some("no") => Ok(false),
        _ => bail!("property '{name}' '{tag}' must be yes or no"),
    }
}

fn parse_property_effects(
    effects: &konnect_sexp::SexpNode,
    name: &str,
) -> Result<kiapi::common::types::TextAttributes> {
    use kiapi::common::types::{HorizontalAlignment, VerticalAlignment};

    let mut font = None;
    let mut horizontal = HorizontalAlignment::HaCenter;
    let mut vertical = VerticalAlignment::VaCenter;
    let mut mirrored = false;
    for clause in effects.children().unwrap_or_default().iter().skip(1) {
        let tag = clause
            .head()
            .with_context(|| format!("property '{name}' effects contain an unsupported atom"))?;
        match tag {
            "font" => {
                if font.replace(clause).is_some() {
                    bail!("property '{name}' contains duplicate font clauses");
                }
            }
            "justify" => {
                for value in clause.children().unwrap_or_default().iter().skip(1) {
                    match value.as_str().with_context(|| {
                        format!("property '{name}' justify contains a non-atom")
                    })? {
                        "left" if horizontal == HorizontalAlignment::HaCenter => {
                            horizontal = HorizontalAlignment::HaLeft
                        }
                        "right" if horizontal == HorizontalAlignment::HaCenter => {
                            horizontal = HorizontalAlignment::HaRight
                        }
                        "top" if vertical == VerticalAlignment::VaCenter => {
                            vertical = VerticalAlignment::VaTop
                        }
                        "bottom" if vertical == VerticalAlignment::VaCenter => {
                            vertical = VerticalAlignment::VaBottom
                        }
                        "mirror" if !mirrored => mirrored = true,
                        "left" | "right" => bail!(
                            "property '{name}' has conflicting horizontal justification"
                        ),
                        "top" | "bottom" => {
                            bail!("property '{name}' has conflicting vertical justification")
                        }
                        "mirror" => bail!("property '{name}' repeats mirrored justification"),
                        unsupported => bail!(
                            "property '{name}' justification '{unsupported}' is not supported losslessly"
                        ),
                    }
                }
            }
            unsupported => bail!(
                "property '{name}' effects clause '{unsupported}' is not supported losslessly"
            ),
        }
    }
    let font =
        font.with_context(|| format!("property '{name}' effects are missing the font clause"))?;

    let mut font_name = String::new();
    let mut size = None;
    let mut thickness = None;
    let mut bold = None;
    let mut italic = None;
    let mut line_spacing = None;
    for clause in font.children().unwrap_or_default().iter().skip(1) {
        let tag = clause
            .head()
            .with_context(|| format!("property '{name}' font contains an unsupported atom"))?;
        match tag {
            "face" => {
                if !font_name.is_empty() {
                    bail!("property '{name}' contains duplicate font face clauses");
                }
                font_name = clause
                    .get(1)
                    .and_then(konnect_sexp::SexpNode::as_str)
                    .filter(|face| !face.is_empty())
                    .with_context(|| format!("property '{name}' font face is invalid"))?
                    .to_string();
            }
            "size" => {
                if size.is_some() {
                    bail!("property '{name}' contains duplicate font size clauses");
                }
                if clause.children().map_or(0, |children| children.len()) != 3 {
                    bail!("property '{name}' font size must contain width and height");
                }
                let width = clause
                    .get_f64(1)
                    .with_context(|| format!("property '{name}' font width is invalid"))?;
                let height = clause
                    .get_f64(2)
                    .with_context(|| format!("property '{name}' font height is invalid"))?;
                if !width.is_finite() || !height.is_finite() || width <= 0.0 || height <= 0.0 {
                    bail!("property '{name}' font size must be finite and positive");
                }
                size = Some((width, height));
            }
            "thickness" => {
                if thickness.is_some() {
                    bail!("property '{name}' contains duplicate font thickness clauses");
                }
                let value = clause
                    .get_f64(1)
                    .with_context(|| format!("property '{name}' font thickness is invalid"))?;
                if clause.children().map_or(0, |children| children.len()) != 2
                    || !value.is_finite()
                    || value <= 0.0
                {
                    bail!("property '{name}' font thickness must be finite and positive");
                }
                thickness = Some(value);
            }
            "bold" => {
                if bold.is_some() {
                    bail!("property '{name}' contains duplicate font 'bold' clauses");
                }
                bold = Some(property_yes_no(clause, name, "font bold")?);
            }
            "italic" => {
                if italic.is_some() {
                    bail!("property '{name}' contains duplicate font 'italic' clauses");
                }
                italic = Some(property_yes_no(clause, name, "font italic")?);
            }
            "line_spacing" => {
                if line_spacing.is_some() {
                    bail!("property '{name}' contains duplicate font 'line_spacing' clauses");
                }
                let value = clause
                    .get_f64(1)
                    .with_context(|| format!("property '{name}' line spacing is invalid"))?;
                if clause.children().map_or(0, |children| children.len()) != 2
                    || !value.is_finite()
                    || value <= 0.0
                {
                    bail!("property '{name}' line spacing must be finite and positive");
                }
                line_spacing = Some(value);
            }
            unsupported => {
                bail!("property '{name}' font clause '{unsupported}' is not supported losslessly")
            }
        }
    }
    let (width, height) =
        size.with_context(|| format!("property '{name}' font is missing its size"))?;
    Ok(kiapi::common::types::TextAttributes {
        font_name,
        horizontal_alignment: horizontal as i32,
        vertical_alignment: vertical as i32,
        angle: None,
        line_spacing: line_spacing.unwrap_or(1.0),
        stroke_width: Some(konnect_ipc::builders::distance(
            thickness.unwrap_or(width * 0.15),
        )),
        italic: italic.unwrap_or(false),
        bold: bold.unwrap_or(false),
        underlined: false,
        visible: true,
        mirrored,
        multiline: false,
        keep_upright: false,
        size: Some(konnect_ipc::builders::vec2(width, height)),
    })
}

fn validate_supported_children(root: &konnect_sexp::SexpNode) -> Result<()> {
    for child in root.children().unwrap_or_default().iter().skip(2) {
        let Some(tag) = child.head() else {
            continue;
        };
        match tag {
            "version" | "generator" | "generator_version" | "layer" | "descr" | "tags" | "attr"
            | "property" | "fp_text" | "fp_line" | "fp_rect" | "fp_circle" | "fp_arc"
            | "fp_poly" | "pad" | "model" => {}
            // KiCad 10 writes these two flags into every footprint it saves
            // (all 15,428 official library files carry them, every one at its
            // default). At the default the typed rebuild loses nothing;
            // a non-default value carries semantics it cannot represent.
            "embedded_fonts" | "duplicate_pad_numbers_are_jumpers" => {
                let value = child.get(1).and_then(konnect_sexp::SexpNode::as_str);
                if value != Some("no") {
                    bail!(
                        "footprint '{tag} {}' is not supported by typed library refresh",
                        value.unwrap_or("")
                    );
                }
            }
            unsupported => {
                bail!("footprint child '{unsupported}' is not supported by typed library refresh")
            }
        }
        match tag {
            "pad" => validate_pad(child)?,
            "fp_line" | "fp_rect" | "fp_circle" | "fp_arc" | "fp_poly" => {
                validate_graphic(child, tag)?
            }
            "fp_text" => {
                let kind = child
                    .get(1)
                    .and_then(konnect_sexp::SexpNode::as_str)
                    .context("fp_text is missing its kind")?;
                match kind {
                    "reference" | "value" => {}
                    "user" => validate_user_text(child)?,
                    _ => {
                        bail!(
                            "fp_text kind '{kind}' is not supported losslessly by typed library refresh"
                        );
                    }
                }
            }
            _ => {}
        }
    }
    Ok(())
}

/// Accept only the `fp_text user` subset the typed IPC rebuild can reproduce.
/// Refusing an unknown clause is intentional: silently dropping even a visual
/// modifier would make a refreshed board differ from its library definition.
fn validate_user_text(text: &konnect_sexp::SexpNode) -> Result<()> {
    text.get(2)
        .and_then(konnect_sexp::SexpNode::as_str)
        .context("fp_text user is missing its text")?;

    let mut saw_at = false;
    let mut saw_layer = false;
    let mut saw_effects = false;
    let mut identifier = None;
    for clause in text.children().unwrap_or_default().iter().skip(3) {
        let tag = clause
            .head()
            .context("fp_text user contains an unsupported atom")?;
        match tag {
            "at" => {
                if saw_at {
                    bail!("fp_text user contains duplicate 'at' clauses");
                }
                saw_at = true;
                let count = clause.children().map_or(0, |children| children.len());
                if !matches!(count, 3 | 4) {
                    bail!("fp_text user 'at' must contain x, y, and optional rotation");
                }
                for index in 1..count {
                    let value = clause
                        .get_f64(index)
                        .with_context(|| format!("fp_text user 'at' value {index} is invalid"))?;
                    if !value.is_finite() {
                        bail!("fp_text user 'at' values must be finite");
                    }
                }
            }
            "layer" => {
                if saw_layer {
                    bail!("fp_text user contains duplicate 'layer' clauses");
                }
                saw_layer = true;
                if clause.children().map_or(0, |children| children.len()) != 2
                    || clause
                        .get(1)
                        .and_then(konnect_sexp::SexpNode::as_str)
                        .is_none_or(str::is_empty)
                {
                    bail!("fp_text user 'layer' must name exactly one layer");
                }
            }
            "uuid" | "tstamp" => {
                if let Some(previous) = identifier {
                    bail!(
                        "fp_text user contains multiple identifier clauses ('{previous}' and '{tag}')"
                    );
                }
                identifier = Some(tag);
                if clause.children().map_or(0, |children| children.len()) != 2
                    || clause
                        .get(1)
                        .and_then(konnect_sexp::SexpNode::as_str)
                        .is_none_or(str::is_empty)
                {
                    bail!("fp_text user '{tag}' must contain exactly one identifier");
                }
            }
            "effects" => {
                if saw_effects {
                    bail!("fp_text user contains duplicate 'effects' clauses");
                }
                saw_effects = true;
                validate_user_text_effects(clause)?;
            }
            unsupported => {
                bail!("fp_text user clause '{unsupported}' is not supported losslessly");
            }
        }
    }

    if !saw_at {
        bail!("fp_text user is missing its 'at' clause");
    }
    if !saw_layer {
        bail!("fp_text user is missing its 'layer' clause");
    }
    if !saw_effects {
        bail!("fp_text user is missing its 'effects' clause");
    }
    Ok(())
}

fn validate_user_text_effects(effects: &konnect_sexp::SexpNode) -> Result<()> {
    let mut font = None;
    for clause in effects.children().unwrap_or_default().iter().skip(1) {
        let tag = clause
            .head()
            .context("fp_text user effects contain an unsupported atom")?;
        if tag != "font" {
            bail!("fp_text user effects clause '{tag}' is not supported losslessly");
        }
        if font.replace(clause).is_some() {
            bail!("fp_text user contains duplicate font clauses");
        }
    }
    let font = font.context("fp_text user effects are missing the font clause")?;

    let mut size = None;
    let mut thickness = None;
    for clause in font.children().unwrap_or_default().iter().skip(1) {
        let tag = clause
            .head()
            .context("fp_text user font contains an unsupported atom")?;
        match tag {
            "size" => {
                if size.is_some() {
                    bail!("fp_text user contains duplicate font size clauses");
                }
                if clause.children().map_or(0, |children| children.len()) != 3 {
                    bail!("fp_text user font size must contain width and height");
                }
                let width = clause
                    .get_f64(1)
                    .context("fp_text user font width is invalid")?;
                let height = clause
                    .get_f64(2)
                    .context("fp_text user font height is invalid")?;
                if !width.is_finite() || !height.is_finite() || width <= 0.0 || height <= 0.0 {
                    bail!("fp_text user font size must be finite and positive");
                }
                if width != height {
                    bail!("fp_text user non-square font size is not supported losslessly");
                }
                size = Some(width);
            }
            "thickness" => {
                if thickness.is_some() {
                    bail!("fp_text user contains duplicate font thickness clauses");
                }
                if clause.children().map_or(0, |children| children.len()) != 2 {
                    bail!("fp_text user font thickness must contain exactly one value");
                }
                let value = clause
                    .get_f64(1)
                    .context("fp_text user font thickness is invalid")?;
                if !value.is_finite() || value <= 0.0 {
                    bail!("fp_text user font thickness must be finite and positive");
                }
                thickness = Some(value);
            }
            unsupported => {
                bail!("fp_text user font clause '{unsupported}' is not supported losslessly");
            }
        }
    }
    size.context("fp_text user font is missing its size")?;
    thickness.context("fp_text user font is missing its thickness")?;
    Ok(())
}

fn validate_graphic(graphic: &konnect_sexp::SexpNode, kind: &str) -> Result<()> {
    let allowed: &[&str] = match kind {
        "fp_line" => &["start", "end", "stroke", "layer", "uuid", "tstamp"],
        "fp_rect" => &["start", "end", "stroke", "fill", "layer", "uuid", "tstamp"],
        "fp_circle" => &["center", "end", "stroke", "fill", "layer", "uuid", "tstamp"],
        "fp_arc" => &["start", "mid", "end", "stroke", "layer", "uuid", "tstamp"],
        "fp_poly" => &["pts", "stroke", "fill", "layer", "uuid", "tstamp"],
        _ => bail!("unsupported footprint graphic '{kind}'"),
    };
    for child in graphic.children().unwrap_or_default().iter().skip(1) {
        let Some(tag) = child.head() else {
            bail!("{kind} contains an unsupported atom");
        };
        if !allowed.contains(&tag) {
            bail!("{kind} clause '{tag}' is not supported by typed library refresh");
        }
        match tag {
            "stroke" => {
                for stroke_child in child.children().unwrap_or_default().iter().skip(1) {
                    let stroke_tag = stroke_child
                        .head()
                        .context("graphic stroke contains an unsupported atom")?;
                    match stroke_tag {
                        "width" => {}
                        "type" => {
                            let stroke_type = stroke_child
                                .get(1)
                                .and_then(konnect_sexp::SexpNode::as_str)
                                .context("graphic stroke type is missing")?;
                            if !matches!(stroke_type, "solid" | "default") {
                                bail!(
                                    "graphic stroke type '{stroke_type}' is not supported losslessly"
                                );
                            }
                        }
                        unsupported => bail!(
                            "graphic stroke clause '{unsupported}' is not supported losslessly"
                        ),
                    }
                }
            }
            "fill" => {
                let fill = child
                    .get(1)
                    .and_then(konnect_sexp::SexpNode::as_str)
                    .context("graphic fill is missing")?;
                if !matches!(fill, "none" | "no" | "solid" | "yes") {
                    bail!("graphic fill '{fill}' is not supported losslessly");
                }
            }
            "pts"
                if child
                    .children()
                    .unwrap_or_default()
                    .iter()
                    .skip(1)
                    .any(|point| point.head() != Some("xy")) =>
            {
                bail!("fp_poly contains a non-xy point");
            }
            _ => {}
        }
    }
    Ok(())
}

fn validate_pad(pad: &konnect_sexp::SexpNode) -> Result<()> {
    let shape = pad
        .get(3)
        .and_then(konnect_sexp::SexpNode::as_str)
        .context("footprint pad is missing its shape")?;
    // A custom pad's anchor and primitives, and every pad's zone connection,
    // are read by the shared pad parser `parse_library_footprint` runs next,
    // which refuses what it cannot carry (a `convexhull` clearance, curved
    // polygon edges, …). Mirroring one onto the back is refused separately,
    // in `mirror_pad`.
    if !matches!(shape, "circle" | "rect" | "oval" | "roundrect" | "custom") {
        bail!("pad shape '{shape}' is not supported by typed library refresh");
    }
    super::pcb_components::pad_zone_connection(pad)
        .context("pad zone connection is not supported by typed library refresh")?;
    super::pcb_components::pad_thermal(pad, None)
        .context("pad thermal settings are not supported by typed library refresh")?;
    for child in pad.children().unwrap_or_default().iter().skip(4) {
        let Some(tag) = child.head() else {
            continue;
        };
        let custom_clause = shape == "custom" && matches!(tag, "options" | "primitives");
        if !custom_clause
            && !matches!(
                tag,
                "at" | "size"
                    | "layers"
                    | "drill"
                    | "roundrect_rratio"
                    | "zone_connect"
                    | "thermal_bridge_width"
                    | "thermal_width"
                    | "thermal_gap"
                    | "thermal_bridge_angle"
                    | "uuid"
                    | "tstamp"
                    | "remove_unused_layers"
            )
        {
            bail!("pad clause '{tag}' is not supported by typed library refresh");
        }
        if tag == "remove_unused_layers" {
            // The typed rebuild always keeps unused layers (`UlrKeep`), so a
            // pad that actually removes them cannot be rebuilt losslessly.
            let value = child.get(1).and_then(konnect_sexp::SexpNode::as_str);
            if value != Some("no") {
                bail!("pad 'remove_unused_layers yes' is not supported by typed library refresh");
            }
        }
        if tag == "drill" {
            for nested in child.children().unwrap_or_default().iter().skip(1) {
                if nested.head().is_some() {
                    bail!("nested drill clauses are not supported by typed library refresh");
                }
            }
        }
    }
    Ok(())
}

fn build_updated_instance(
    current: &kiapi::board::types::FootprintInstance,
    library: &LibraryFootprint,
    net_codes: &BTreeMap<String, i32>,
    routed_nets: &BTreeSet<String>,
) -> Result<PreparedUpdate> {
    let current_definition = current
        .definition
        .as_ref()
        .context("board footprint has no embedded definition")?;
    let current_id = current_definition
        .id
        .as_ref()
        .map(|id| format!("{}:{}", id.library_nickname, id.entry_name))
        .unwrap_or_default();
    if current_id != library.library_id {
        bail!(
            "board footprint library id '{current_id}' does not match '{}'",
            library.library_id
        );
    }

    let mut old_nets = BTreeMap::<String, kiapi::board::types::Net>::new();
    for item in &current_definition.items {
        if !item.type_url.ends_with("kiapi.board.types.Pad") {
            continue;
        }
        let pad = kiapi::board::types::Pad::decode(item.value.as_slice())
            .context("board footprint contains an invalid pad")?;
        let Some(net) = pad.net.filter(|net| !net.name.is_empty()) else {
            continue;
        };
        if let Some(existing) = old_nets.insert(pad.number.clone(), net.clone()) {
            if existing.name != net.name {
                bail!(
                    "logical pad {} carries multiple nets ('{}' and '{}')",
                    pad.number,
                    existing.name,
                    net.name
                );
            }
        }
    }
    let new_numbers = library
        .pads
        .iter()
        .map(|pad| pad.number.as_str())
        .collect::<BTreeSet<_>>();
    for (number, net) in &old_nets {
        if !new_numbers.contains(number.as_str()) {
            let routed = routed_nets.contains(&net.name);
            bail!(
                "library update removes connected pad {number} on net '{}'{}",
                net.name,
                if routed { " with routed copper" } else { "" }
            );
        }
    }

    let position = current.position.as_ref().cloned().unwrap_or_default();
    let rotation = current
        .orientation
        .as_ref()
        .map(|angle| angle.value_degrees)
        .unwrap_or(0.0);
    let is_back = current.layer == kiapi::board::types::BoardLayer::BlBCu as i32;
    let layer = if is_back { "B.Cu" } else { "F.Cu" };
    let (pads, graphics) = if is_back {
        (
            library
                .pads
                .iter()
                .map(mirror_pad)
                .collect::<Result<Vec<_>>>()?,
            library
                .graphics
                .iter()
                .map(mirror_graphic)
                .collect::<Result<Vec<_>>>()?,
        )
    } else {
        (library.pads.clone(), library.graphics.clone())
    };
    let packed = konnect_ipc::KiCadIpcClient::build_footprint_item(
        &library.library_id,
        &field_text(&current.reference_field),
        &field_text(&current.value_field),
        &pads,
        &graphics,
        &Default::default(),
        konnect_ipc::builders::nm_to_mm(position.x_nm),
        konnect_ipc::builders::nm_to_mm(position.y_nm),
        rotation,
        layer,
    )?;
    let built = kiapi::board::types::FootprintInstance::decode(packed.value.as_slice())
        .context("typed footprint builder returned an invalid item")?;

    let mut updated = current.clone();
    let mut definition = built
        .definition
        .context("typed footprint builder returned no definition")?;
    definition.attributes = library.definition.attributes.clone();
    definition.reference_field = current_definition.reference_field.clone();
    definition.value_field = current_definition.value_field.clone();
    definition.datasheet_field = current_definition.datasheet_field.clone();
    definition.description_field = current_definition.description_field.clone();
    apply_field_value(
        &mut definition.datasheet_field,
        library.datasheet.as_deref(),
    );
    apply_field_value(
        &mut definition.description_field,
        library.description_field.as_deref(),
    );
    definition.items.extend(
        library.models.iter().map(|model| {
            konnect_ipc::builders::pack_any(model, "kiapi.board.types.Footprint3DModel")
        }),
    );
    for item in &mut definition.items {
        if item.type_url.ends_with("kiapi.board.types.Pad") {
            let mut pad = kiapi::board::types::Pad::decode(item.value.as_slice())?;
            pad.net = old_nets
                .get(&pad.number)
                .map(|old| kiapi::board::types::Net {
                    code: net_codes
                        .get(&old.name)
                        .copied()
                        .or_else(|| old.code.as_ref().map(|code| code.value))
                        .map(|value| kiapi::board::types::NetCode { value }),
                    name: old.name.clone(),
                });
            *item = konnect_ipc::builders::pack_any(&pad, "kiapi.board.types.Pad");
        } else if is_back && item.type_url.ends_with("kiapi.board.types.BoardText") {
            let mut text = kiapi::board::types::BoardText::decode(item.value.as_slice())?;
            if is_side_specific_layer(text.layer) {
                if let Some(attributes) =
                    text.text.as_mut().and_then(|text| text.attributes.as_mut())
                {
                    attributes.mirrored = true;
                }
            }
            *item = konnect_ipc::builders::pack_any(&text, "kiapi.board.types.BoardText");
        }
    }
    merge_custom_properties(
        &mut definition,
        current_definition,
        &library.properties,
        &position,
        rotation,
        is_back,
    )?;
    updated.definition = Some(definition);
    apply_field_value(&mut updated.datasheet_field, library.datasheet.as_deref());
    apply_field_value(
        &mut updated.description_field,
        library.description_field.as_deref(),
    );
    let mut attributes = library.attributes.clone();
    if let Some(current_attributes) = current.attributes.as_ref() {
        attributes.not_in_schematic = current_attributes.not_in_schematic;
        attributes.do_not_populate = current_attributes.do_not_populate;
    }
    updated.attributes = Some(attributes);

    let changed_domains = changed_domains(current, &updated)?;
    let preserved = PreservedState::derive(current, &updated, &old_nets);
    let (pad_differences, pad_differences_omitted) = pad_differences(
        current_definition,
        updated
            .definition
            .as_ref()
            .context("missing updated definition")?,
    )?;
    Ok(PreparedUpdate {
        item: konnect_ipc::builders::pack_any(&updated, "kiapi.board.types.FootprintInstance"),
        changed_domains,
        preserved,
        pad_differences,
        pad_differences_omitted,
    })
}

fn apply_field_value(field: &mut Option<kiapi::board::types::Field>, value: Option<&str>) {
    let Some(value) = value else {
        return;
    };
    field
        .get_or_insert_with(Default::default)
        .text
        .get_or_insert_with(Default::default)
        .text
        .get_or_insert_with(Default::default)
        .text = value.to_string();
}

fn merge_custom_properties(
    updated: &mut kiapi::board::types::Footprint,
    current: &kiapi::board::types::Footprint,
    library_properties: &[kiapi::board::types::Field],
    footprint_position: &kiapi::common::types::Vector2,
    footprint_rotation: f64,
    is_back: bool,
) -> Result<()> {
    let library_names = library_properties
        .iter()
        .map(|field| field.name.as_str())
        .collect::<BTreeSet<_>>();
    let mut current_names = BTreeSet::new();
    for item in current
        .items
        .iter()
        .filter(|item| item.type_url.ends_with("kiapi.board.types.Field"))
    {
        let field = kiapi::board::types::Field::decode(item.value.as_slice())
            .context("board footprint contains an invalid custom property")?;
        if field.name.is_empty() {
            bail!("board footprint contains a custom property without a name");
        }
        if !current_names.insert(field.name.clone()) {
            bail!(
                "board footprint contains more than one custom property named '{}'",
                field.name
            );
        }
        if !library_names.contains(field.name.as_str()) {
            // A field that exists only on the placed instance belongs to that
            // instance. Preserve its complete typed representation verbatim.
            updated.items.push(item.clone());
        }
    }
    for property in library_properties {
        let property =
            transform_library_property(property, footprint_position, footprint_rotation, is_back)?;
        updated.items.push(konnect_ipc::builders::pack_any(
            &property,
            "kiapi.board.types.Field",
        ));
    }
    Ok(())
}

fn transform_library_property(
    property: &kiapi::board::types::Field,
    footprint_position: &kiapi::common::types::Vector2,
    footprint_rotation: f64,
    is_back: bool,
) -> Result<kiapi::board::types::Field> {
    let mut property = property.clone();
    property.id = None;
    let board_text = property
        .text
        .as_mut()
        .with_context(|| format!("property '{}' has no board text", property.name))?;
    board_text.id = None;
    let text = board_text
        .text
        .as_mut()
        .with_context(|| format!("property '{}' has no text value", property.name))?;
    let local_position = text
        .position
        .as_ref()
        .with_context(|| format!("property '{}' has no position", property.name))?;
    let local_x = konnect_ipc::builders::nm_to_mm(local_position.x_nm);
    let mut local_y = konnect_ipc::builders::nm_to_mm(local_position.y_nm);
    if is_back {
        local_y = -local_y;
    }
    let (board_x, board_y) = konnect_sexp::geometry::transform_pad(
        local_x,
        local_y,
        konnect_ipc::builders::nm_to_mm(footprint_position.x_nm),
        konnect_ipc::builders::nm_to_mm(footprint_position.y_nm),
        footprint_rotation,
    );
    text.position = Some(konnect_ipc::builders::vec2(board_x, board_y));

    let attributes = text
        .attributes
        .as_mut()
        .with_context(|| format!("property '{}' has no text attributes", property.name))?;
    let local_angle = attributes
        .angle
        .as_ref()
        .map(|angle| angle.value_degrees)
        .unwrap_or(0.0);
    let local_angle = if is_back {
        180.0 - local_angle
    } else {
        local_angle
    };
    attributes.angle = Some(kiapi::common::types::Angle {
        value_degrees: readable_property_angle(local_angle + footprint_rotation),
    });
    if is_back {
        attributes.mirrored = !attributes.mirrored;
        let layer = kiapi::board::types::BoardLayer::try_from(board_text.layer)
            .with_context(|| format!("property '{}' has an invalid layer", property.name))?;
        let layer_name = konnect_ipc::builders::layer_name(layer)
            .with_context(|| format!("property '{}' has an unnamed layer", property.name))?;
        board_text.layer =
            konnect_ipc::builders::try_layer_from_name(&flip_layer_name(layer_name)?)? as i32;
    }
    Ok(property)
}

fn readable_property_angle(degrees: f64) -> f64 {
    let mut angle = degrees.rem_euclid(360.0);
    if angle > 90.0 && angle <= 270.0 {
        angle -= 180.0;
    }
    angle
}

fn mirror_pad(pad: &konnect_ipc::IpcPadDefinition) -> Result<konnect_ipc::IpcPadDefinition> {
    // Mirroring would have to flip every pad-local primitive as well; until
    // that is implemented and verified, a back-side custom pad is refused.
    if pad.custom.is_some() {
        bail!(
            "custom pad '{}' cannot be mirrored onto the back side by typed library refresh",
            pad.number
        );
    }
    // Thermal settings stay as they are: KiCad's `PAD::Flip` negates the pad
    // orientation but leaves the spoke angle, width and gap alone.
    let mut mirrored = pad.clone();
    mirrored.y = -mirrored.y;
    mirrored.rotation = -mirrored.rotation;
    mirrored.layers = mirrored
        .layers
        .iter()
        .map(|layer| flip_layer_name(layer))
        .collect::<Result<_>>()?;
    Ok(mirrored)
}

fn mirror_graphic(
    graphic: &konnect_ipc::IpcGraphicDefinition,
) -> Result<konnect_ipc::IpcGraphicDefinition> {
    use konnect_ipc::IpcGraphicDefinition as Graphic;

    let point = |(x, y): (f64, f64)| (x, -y);
    Ok(match graphic {
        Graphic::Line {
            start,
            end,
            layer,
            width,
        } => Graphic::Line {
            start: point(*start),
            end: point(*end),
            layer: flip_layer_name(layer)?,
            width: *width,
        },
        Graphic::Rect {
            start,
            end,
            layer,
            width,
            filled,
        } => Graphic::Rect {
            start: point(*start),
            end: point(*end),
            layer: flip_layer_name(layer)?,
            width: *width,
            filled: *filled,
        },
        Graphic::Circle {
            center,
            end,
            layer,
            width,
            filled,
        } => Graphic::Circle {
            center: point(*center),
            end: point(*end),
            layer: flip_layer_name(layer)?,
            width: *width,
            filled: *filled,
        },
        Graphic::Arc {
            start,
            mid,
            end,
            layer,
            width,
        } => Graphic::Arc {
            start: point(*end),
            mid: point(*mid),
            end: point(*start),
            layer: flip_layer_name(layer)?,
            width: *width,
        },
        Graphic::Poly {
            points,
            layer,
            width,
            filled,
        } => Graphic::Poly {
            points: points.iter().copied().map(point).collect(),
            layer: flip_layer_name(layer)?,
            width: *width,
            filled: *filled,
        },
        Graphic::Text {
            text,
            position,
            rotation,
            layer,
            size,
            stroke_width_mm,
        } => Graphic::Text {
            text: text.clone(),
            position: point(*position),
            rotation: 180.0 - rotation,
            layer: flip_layer_name(layer)?,
            size: *size,
            stroke_width_mm: *stroke_width_mm,
        },
    })
}

fn flip_layer_name(layer: &str) -> Result<String> {
    let flipped = match layer {
        "F.Cu" => "B.Cu",
        "B.Cu" => "F.Cu",
        "F.Adhes" => "B.Adhes",
        "B.Adhes" => "F.Adhes",
        "F.Paste" => "B.Paste",
        "B.Paste" => "F.Paste",
        "F.SilkS" | "F.Silkscreen" => "B.SilkS",
        "B.SilkS" | "B.Silkscreen" => "F.SilkS",
        "F.Mask" => "B.Mask",
        "B.Mask" => "F.Mask",
        "F.CrtYd" | "F.Courtyard" => "B.CrtYd",
        "B.CrtYd" | "B.Courtyard" => "F.CrtYd",
        "F.Fab" => "B.Fab",
        "B.Fab" => "F.Fab",
        "*.Cu" | "*.Mask" | "*.Paste" => layer,
        other if other.starts_with("F.") || other.starts_with("B.") => {
            bail!("unsupported side-specific footprint layer '{other}'")
        }
        other => other,
    };
    Ok(flipped.to_string())
}

fn is_side_specific_layer(layer: i32) -> bool {
    use kiapi::board::types::BoardLayer;

    matches!(
        BoardLayer::try_from(layer).ok(),
        Some(
            BoardLayer::BlFCu
                | BoardLayer::BlBCu
                | BoardLayer::BlFAdhes
                | BoardLayer::BlBAdhes
                | BoardLayer::BlFPaste
                | BoardLayer::BlBPaste
                | BoardLayer::BlFSilkS
                | BoardLayer::BlBSilkS
                | BoardLayer::BlFMask
                | BoardLayer::BlBMask
                | BoardLayer::BlFCrtYd
                | BoardLayer::BlBCrtYd
                | BoardLayer::BlFFab
                | BoardLayer::BlBFab
        )
    )
}

fn changed_domains(
    current: &kiapi::board::types::FootprintInstance,
    updated: &kiapi::board::types::FootprintInstance,
) -> Result<BTreeSet<ChangedDomain>> {
    let mut changed = BTreeSet::new();
    let current_definition = current
        .definition
        .as_ref()
        .context("missing current definition")?;
    let updated_definition = updated
        .definition
        .as_ref()
        .context("missing updated definition")?;

    let current_pads = normalized_items(current_definition, "Pad")?;
    let updated_pads = normalized_items(updated_definition, "Pad")?;
    if current_pads != updated_pads {
        changed.insert(ChangedDomain::Pads);
    }
    let current_graphics = normalized_graphics(current_definition)?;
    let updated_graphics = normalized_graphics(updated_definition)?;
    if current_graphics != updated_graphics {
        changed.insert(ChangedDomain::Graphics);
    }
    if normalized_items(current_definition, "Footprint3DModel")?
        != normalized_items(updated_definition, "Footprint3DModel")?
    {
        changed.insert(ChangedDomain::Models);
    }
    if current_definition.attributes != updated_definition.attributes
        || field_text(&current_definition.datasheet_field)
            != field_text(&updated_definition.datasheet_field)
        || field_text(&current_definition.description_field)
            != field_text(&updated_definition.description_field)
        || normalized_items(current_definition, "Field")?
            != normalized_items(updated_definition, "Field")?
    {
        changed.insert(ChangedDomain::Metadata);
    }
    if library_owned_attributes(current.attributes.as_ref())
        != library_owned_attributes(updated.attributes.as_ref())
    {
        changed.insert(ChangedDomain::Attributes);
    }
    Ok(changed)
}

/// The most pad differences one footprint reports; the rest are counted.
const PAD_DIFFERENCE_LIMIT: usize = 64;

/// One typed pad field where the pad rebuilt from the library source
/// (`expected`) and the pad KiCad reports on the board (`observed`) differ.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
struct PadDifference {
    pad: String,
    field: String,
    expected: serde_json::Value,
    observed: serde_json::Value,
    /// Set when the plan does not count this difference, with why.
    #[serde(skip_serializing_if = "Option::is_none")]
    ignored_by_plan: Option<String>,
}

/// Why the plan ignores a dropped closing vertex, and why it is reported.
const CLOSING_VERTEX_NOTE: &str = "KiCad's API drops an explicit closing vertex \
     (UnpackPolyLine closes the chain), so re-applying cannot change it; KiCad 10's \
     footprint parser keeps the vertex and its library parity check compares polygon \
     vertex counts exactly, so native DRC can still report lib_footprint_mismatch";

/// Which pads, and which of their typed fields, differ between the board
/// (`current`) and the rebuild (`updated`), with both values. Bounded: a
/// polygon reports its vertex count and its first differing vertex, not
/// every coordinate, and at most [`PAD_DIFFERENCE_LIMIT`] entries are kept;
/// the second value counts the rest.
///
/// Pads pair by number; pads sharing a number pair identical ones first and
/// the rest in position order. Every difference the plan counts is listed,
/// and a pad whose normalized form differs in a field not named here is
/// still reported, as `unclassified`.
fn pad_differences(
    current: &kiapi::board::types::Footprint,
    updated: &kiapi::board::types::Footprint,
) -> Result<(Vec<PadDifference>, usize)> {
    use kiapi::board::types::Pad;
    fn pads(definition: &kiapi::board::types::Footprint) -> Result<BTreeMap<String, Vec<Pad>>> {
        let mut pads = BTreeMap::<String, Vec<Pad>>::new();
        for item in &definition.items {
            if item.type_url.ends_with("kiapi.board.types.Pad") {
                let pad = Pad::decode(item.value.as_slice())?;
                pads.entry(pad.number.clone())
                    .or_default()
                    .push(normalized_pad(pad, false));
            }
        }
        Ok(pads)
    }
    let full = |pad: &Pad| normalized_pad(pad.clone(), true).encode_to_vec();
    let position = |pad: &Pad| pad.position.map(|p| (p.x_nm, p.y_nm));

    let mut observed_pads = pads(current)?;
    let mut expected_pads = pads(updated)?;
    let numbers = observed_pads
        .keys()
        .chain(expected_pads.keys())
        .cloned()
        .collect::<BTreeSet<_>>();
    let mut out = Vec::new();
    for number in numbers {
        let mut expected = expected_pads.remove(&number).unwrap_or_default();
        let mut observed = observed_pads.remove(&number).unwrap_or_default();
        let mut pairs = Vec::new();
        expected.retain(
            |want| match observed.iter().position(|got| full(got) == full(want)) {
                Some(index) => {
                    pairs.push((want.clone(), observed.remove(index)));
                    false
                }
                None => true,
            },
        );
        expected.sort_by_key(position);
        observed.sort_by_key(position);
        let unmatched = expected.len().max(observed.len());
        let mut expected = expected.into_iter();
        let mut observed = observed.into_iter();
        for _ in 0..unmatched {
            match (expected.next(), observed.next()) {
                (Some(want), Some(got)) => pairs.push((want, got)),
                (want, got) => out.push(PadDifference {
                    pad: number.clone(),
                    field: "pad".to_string(),
                    expected: json!(if want.is_some() { "present" } else { "absent" }),
                    observed: json!(if got.is_some() { "present" } else { "absent" }),
                    ignored_by_plan: None,
                }),
            }
        }
        for (want, got) in pairs {
            let before = out.len();
            PadRecorder {
                pad: &number,
                out: &mut out,
            }
            .pad(&want, &got);
            let counted = out[before..]
                .iter()
                .any(|difference| difference.ignored_by_plan.is_none());
            if !counted && full(&want) != full(&got) {
                let text = |pad: &Pad| {
                    let mut text = format!("{pad:?}");
                    if text.len() > 2000 {
                        let end = (0..=2000).rev().find(|&i| text.is_char_boundary(i));
                        text.truncate(end.unwrap_or(0));
                        text.push('…');
                    }
                    text
                };
                out.push(PadDifference {
                    pad: number.clone(),
                    field: "unclassified".to_string(),
                    expected: json!(text(&want)),
                    observed: json!(text(&got)),
                    ignored_by_plan: None,
                });
            }
        }
    }
    let omitted = out.len().saturating_sub(PAD_DIFFERENCE_LIMIT);
    out.truncate(PAD_DIFFERENCE_LIMIT);
    Ok((out, omitted))
}

struct PadRecorder<'a> {
    pad: &'a str,
    out: &'a mut Vec<PadDifference>,
}

fn debug_value<T: std::fmt::Debug>(value: &T) -> serde_json::Value {
    json!(format!("{value:?}"))
}

/// A protocol enum by name, or its number when this build does not know it.
fn enum_value<E: TryFrom<i32> + std::fmt::Debug>(value: i32) -> serde_json::Value {
    E::try_from(value).map_or_else(|_| json!(value), |known| debug_value(&known))
}

fn point_value(node: &kiapi::common::types::PolyLineNode) -> serde_json::Value {
    use kiapi::common::types::poly_line_node::Geometry;
    match node.geometry.as_ref() {
        Some(Geometry::Point(point)) => json!({ "x_nm": point.x_nm, "y_nm": point.y_nm }),
        other => debug_value(&other),
    }
}

impl PadRecorder<'_> {
    fn note(
        &mut self,
        field: String,
        expected: serde_json::Value,
        observed: serde_json::Value,
        ignored_by_plan: Option<&str>,
    ) {
        if expected != observed {
            self.out.push(PadDifference {
                pad: self.pad.to_string(),
                field,
                expected,
                observed,
                ignored_by_plan: ignored_by_plan.map(str::to_string),
            });
        }
    }

    fn field(&mut self, field: &str, expected: serde_json::Value, observed: serde_json::Value) {
        self.note(field.to_string(), expected, observed, None);
    }

    fn pad(&mut self, want: &kiapi::board::types::Pad, got: &kiapi::board::types::Pad) {
        use kiapi::board::types::PadType;
        use kiapi::common::types::LockedState;
        self.field(
            "type",
            enum_value::<PadType>(want.r#type),
            enum_value::<PadType>(got.r#type),
        );
        self.field(
            "locked",
            enum_value::<LockedState>(want.locked),
            enum_value::<LockedState>(got.locked),
        );
        self.field(
            "position",
            debug_value(&want.position),
            debug_value(&got.position),
        );
        self.field(
            "copper_clearance_override",
            debug_value(&want.copper_clearance_override),
            debug_value(&got.copper_clearance_override),
        );
        match (want.pad_stack.as_ref(), got.pad_stack.as_ref()) {
            (Some(want), Some(got)) => self.stack(want, got),
            (want, got) => self.field("pad_stack", json!(want.is_some()), json!(got.is_some())),
        }
    }

    fn stack(&mut self, want: &kiapi::board::types::PadStack, got: &kiapi::board::types::PadStack) {
        use kiapi::board::types::{BoardLayer, PadStackType, UnconnectedLayerRemoval};
        let layers = |stack: &kiapi::board::types::PadStack| {
            json!(stack
                .layers
                .iter()
                .map(|layer| enum_value::<BoardLayer>(*layer))
                .collect::<Vec<_>>())
        };
        self.field(
            "pad_stack.type",
            enum_value::<PadStackType>(want.r#type),
            enum_value::<PadStackType>(got.r#type),
        );
        self.field("pad_stack.layers", layers(want), layers(got));
        self.field(
            "pad_stack.angle_degrees",
            json!(want.angle.as_ref().map(|angle| angle.value_degrees)),
            json!(got.angle.as_ref().map(|angle| angle.value_degrees)),
        );
        self.field(
            "pad_stack.unconnected_layer_removal",
            enum_value::<UnconnectedLayerRemoval>(want.unconnected_layer_removal),
            enum_value::<UnconnectedLayerRemoval>(got.unconnected_layer_removal),
        );
        for (field, want, got) in [
            (
                "pad_stack.drill",
                debug_value(&want.drill),
                debug_value(&got.drill),
            ),
            (
                "pad_stack.front_outer_layers",
                debug_value(&want.front_outer_layers),
                debug_value(&got.front_outer_layers),
            ),
            (
                "pad_stack.back_outer_layers",
                debug_value(&want.back_outer_layers),
                debug_value(&got.back_outer_layers),
            ),
            (
                "pad_stack.secondary_drill",
                debug_value(&want.secondary_drill),
                debug_value(&got.secondary_drill),
            ),
            (
                "pad_stack.tertiary_drill",
                debug_value(&want.tertiary_drill),
                debug_value(&got.tertiary_drill),
            ),
            (
                "pad_stack.front_post_machining",
                debug_value(&want.front_post_machining),
                debug_value(&got.front_post_machining),
            ),
            (
                "pad_stack.back_post_machining",
                debug_value(&want.back_post_machining),
                debug_value(&got.back_post_machining),
            ),
        ] {
            self.field(field, want, got);
        }
        self.zone_settings(
            "pad_stack.zone_settings",
            want.zone_settings.as_ref(),
            got.zone_settings.as_ref(),
        );
        self.field(
            "pad_stack.copper_layers.count",
            json!(want.copper_layers.len()),
            json!(got.copper_layers.len()),
        );
        for (index, (want, got)) in want
            .copper_layers
            .iter()
            .zip(&got.copper_layers)
            .enumerate()
        {
            self.copper_layer(&format!("pad_stack.copper_layers[{index}]"), want, got);
        }
    }

    fn zone_settings(
        &mut self,
        prefix: &str,
        want: Option<&kiapi::board::types::ZoneConnectionSettings>,
        got: Option<&kiapi::board::types::ZoneConnectionSettings>,
    ) {
        use kiapi::board::types::ZoneConnectionStyle;
        let connection = |settings: Option<&kiapi::board::types::ZoneConnectionSettings>| {
            settings.map_or(serde_json::Value::Null, |settings| {
                enum_value::<ZoneConnectionStyle>(settings.zone_connection)
            })
        };
        self.note(
            format!("{prefix}.zone_connection"),
            connection(want),
            connection(got),
            None,
        );
        let spokes = |settings: Option<&kiapi::board::types::ZoneConnectionSettings>| {
            settings.and_then(|settings| settings.thermal_spokes)
        };
        let (want, got) = (spokes(want), spokes(got));
        self.note(
            format!("{prefix}.thermal_spokes.width_nm"),
            json!(want
                .and_then(|spokes| spokes.width)
                .map(|width| width.value_nm)),
            json!(got
                .and_then(|spokes| spokes.width)
                .map(|width| width.value_nm)),
            None,
        );
        self.note(
            format!("{prefix}.thermal_spokes.gap_nm"),
            json!(want.and_then(|spokes| spokes.gap).map(|gap| gap.value_nm)),
            json!(got.and_then(|spokes| spokes.gap).map(|gap| gap.value_nm)),
            None,
        );
        self.note(
            format!("{prefix}.thermal_spokes.angle_degrees"),
            json!(want
                .and_then(|spokes| spokes.angle)
                .map(|angle| angle.value_degrees)),
            json!(got
                .and_then(|spokes| spokes.angle)
                .map(|angle| angle.value_degrees)),
            None,
        );
    }

    fn copper_layer(
        &mut self,
        prefix: &str,
        want: &kiapi::board::types::PadStackLayer,
        got: &kiapi::board::types::PadStackLayer,
    ) {
        use kiapi::board::types::{BoardLayer, PadStackShape};
        let mut field = |name: &str, want: serde_json::Value, got: serde_json::Value| {
            self.note(format!("{prefix}.{name}"), want, got, None);
        };
        field(
            "layer",
            enum_value::<BoardLayer>(want.layer),
            enum_value::<BoardLayer>(got.layer),
        );
        field(
            "shape",
            enum_value::<PadStackShape>(want.shape),
            enum_value::<PadStackShape>(got.shape),
        );
        field("size", debug_value(&want.size), debug_value(&got.size));
        field(
            "corner_rounding_ratio",
            json!(want.corner_rounding_ratio),
            json!(got.corner_rounding_ratio),
        );
        field(
            "chamfer_ratio",
            json!(want.chamfer_ratio),
            json!(got.chamfer_ratio),
        );
        field(
            "chamfered_corners",
            debug_value(&want.chamfered_corners),
            debug_value(&got.chamfered_corners),
        );
        field(
            "custom_anchor_shape",
            enum_value::<PadStackShape>(want.custom_anchor_shape),
            enum_value::<PadStackShape>(got.custom_anchor_shape),
        );
        field(
            "trapezoid_delta",
            debug_value(&want.trapezoid_delta),
            debug_value(&got.trapezoid_delta),
        );
        field(
            "offset",
            debug_value(&want.offset),
            debug_value(&got.offset),
        );
        field(
            "custom_shapes.count",
            json!(want.custom_shapes.len()),
            json!(got.custom_shapes.len()),
        );
        self.zone_settings(
            &format!("{prefix}.zone_settings"),
            want.zone_settings.as_ref(),
            got.zone_settings.as_ref(),
        );
        for (index, (want, got)) in want
            .custom_shapes
            .iter()
            .zip(&got.custom_shapes)
            .enumerate()
        {
            self.primitive(&format!("{prefix}.custom_shapes[{index}]"), want, got);
        }
    }

    fn primitive(
        &mut self,
        prefix: &str,
        want: &kiapi::board::types::BoardGraphicShape,
        got: &kiapi::board::types::BoardGraphicShape,
    ) {
        use kiapi::board::types::BoardLayer;
        use kiapi::common::types::{graphic_shape::Geometry, LockedState};
        self.note(
            format!("{prefix}.layer"),
            enum_value::<BoardLayer>(want.layer),
            enum_value::<BoardLayer>(got.layer),
            None,
        );
        self.note(
            format!("{prefix}.locked"),
            enum_value::<LockedState>(want.locked),
            enum_value::<LockedState>(got.locked),
            None,
        );
        let attributes = |shape: &kiapi::board::types::BoardGraphicShape| {
            shape.shape.as_ref().and_then(|shape| shape.attributes)
        };
        let (want_attributes, got_attributes) = (attributes(want), attributes(got));
        let stroke = |attributes: &Option<kiapi::common::types::GraphicAttributes>| {
            attributes.as_ref().and_then(|a| a.stroke)
        };
        let (want_stroke, got_stroke) = (stroke(&want_attributes), stroke(&got_attributes));
        self.note(
            format!("{prefix}.stroke.width_nm"),
            json!(want_stroke
                .as_ref()
                .and_then(|s| s.width)
                .map(|w| w.value_nm)),
            json!(got_stroke
                .as_ref()
                .and_then(|s| s.width)
                .map(|w| w.value_nm)),
            None,
        );
        self.note(
            format!("{prefix}.stroke.style"),
            want_stroke.as_ref().map_or(serde_json::Value::Null, |s| {
                enum_value::<kiapi::common::types::StrokeLineStyle>(s.style)
            }),
            got_stroke.as_ref().map_or(serde_json::Value::Null, |s| {
                enum_value::<kiapi::common::types::StrokeLineStyle>(s.style)
            }),
            None,
        );
        self.note(
            format!("{prefix}.stroke.color"),
            debug_value(&want_stroke.as_ref().and_then(|s| s.color)),
            debug_value(&got_stroke.as_ref().and_then(|s| s.color)),
            None,
        );
        self.note(
            format!("{prefix}.fill"),
            debug_value(&want_attributes.as_ref().and_then(|a| a.fill)),
            debug_value(&got_attributes.as_ref().and_then(|a| a.fill)),
            None,
        );
        let geometry = |shape: &kiapi::board::types::BoardGraphicShape| {
            shape
                .shape
                .as_ref()
                .and_then(|shape| shape.geometry.clone())
        };
        match (geometry(want), geometry(got)) {
            (Some(Geometry::Polygon(want)), Some(Geometry::Polygon(got))) => {
                self.note(
                    format!("{prefix}.polygon.count"),
                    json!(want.polygons.len()),
                    json!(got.polygons.len()),
                    None,
                );
                for (index, (want, got)) in want.polygons.iter().zip(&got.polygons).enumerate() {
                    let path = format!("{prefix}.polygon[{index}]");
                    let empty = kiapi::common::types::PolyLine::default();
                    let (want_outline, got_outline) = (
                        want.outline.as_ref().unwrap_or(&empty),
                        got.outline.as_ref().unwrap_or(&empty),
                    );
                    self.polyline(&format!("{path}.outline"), want_outline, got_outline);
                    self.note(
                        format!("{path}.holes.count"),
                        json!(want.holes.len()),
                        json!(got.holes.len()),
                        None,
                    );
                    for (hole, (want, got)) in want.holes.iter().zip(&got.holes).enumerate() {
                        self.polyline(&format!("{path}.holes[{hole}]"), want, got);
                    }
                }
            }
            (want, got) => self.note(
                format!("{prefix}.geometry"),
                debug_value(&want),
                debug_value(&got),
                None,
            ),
        }
    }

    /// A closed outline's difference: whether it is closed, its vertex
    /// count, and its first differing vertex with how many differ, never
    /// every coordinate. A dropped explicit closing vertex alone is reported
    /// as ignored by the plan.
    fn polyline(
        &mut self,
        prefix: &str,
        want: &kiapi::common::types::PolyLine,
        got: &kiapi::common::types::PolyLine,
    ) {
        self.note(
            format!("{prefix}.closed"),
            json!(want.closed),
            json!(got.closed),
            None,
        );
        let (want, got) = (&want.nodes, &got.nodes);
        if want == got {
            return;
        }
        let closing = |longer: &[kiapi::common::types::PolyLineNode],
                       shorter: &[kiapi::common::types::PolyLineNode]| {
            longer.len() == shorter.len() + 1
                && longer.len() > 3
                && longer.first() == longer.last()
                && longer[..shorter.len()] == *shorter
        };
        if closing(want, got) || closing(got, want) {
            self.note(
                format!("{prefix}.vertex_count"),
                json!(want.len()),
                json!(got.len()),
                Some(CLOSING_VERTEX_NOTE),
            );
            return;
        }
        self.note(
            format!("{prefix}.vertex_count"),
            json!(want.len()),
            json!(got.len()),
            None,
        );
        let differing = want
            .iter()
            .zip(got.iter())
            .enumerate()
            .filter(|(_, (want, got))| want != got)
            .map(|(index, _)| index)
            .collect::<Vec<_>>();
        if let Some(&first) = differing.first() {
            self.note(
                format!(
                    "{prefix}.vertex[{first}] (first of {} differing vertices)",
                    differing.len()
                ),
                point_value(&want[first]),
                point_value(&got[first]),
                None,
            );
        }
    }
}

fn normalized_items(
    definition: &kiapi::board::types::Footprint,
    suffix: &str,
) -> Result<Vec<Vec<u8>>> {
    let mut items = definition
        .items
        .iter()
        .filter(|item| item.type_url.ends_with(suffix))
        .map(|item| {
            if suffix == "Pad" {
                let pad = kiapi::board::types::Pad::decode(item.value.as_slice())?;
                Ok(normalized_pad(pad, true).encode_to_vec())
            } else if suffix == "Field" {
                let mut field = kiapi::board::types::Field::decode(item.value.as_slice())?;
                field.id = None;
                if let Some(text) = field.text.as_mut() {
                    text.id = None;
                }
                Ok(field.encode_to_vec())
            } else {
                Ok(item.value.clone())
            }
        })
        .collect::<Result<Vec<_>>>()?;
    items.sort();
    Ok(items)
}

/// A pad without what is not library content or what KiCad assigns itself,
/// so a pad as rebuilt and the same pad as KiCad reports it compare equal:
/// identities (the pad's and each primitive's KIID, the read-only parent),
/// nets and schematic pin data, layer order, an empty drill, an unset
/// primitive lock state (KiCad holds it unlocked) and a zero rectangle corner
/// radius. Zone settings
/// are compared as KiCad holds them, thermal spokes included (see
/// [`konnect_ipc::builders::canonical_pad_zone_settings`]); KiCad 10's own
/// library parity check compares primitives by shape, points, stroke and
/// fill, never by identity.
///
/// `drop_closing` also removes a custom polygon's explicit closing vertex,
/// which KiCad's API drops on the way in (`UnpackPolyLine` closes the
/// chain). The plan ignores that, since re-applying cannot keep it; the pad
/// diff reports it, because KiCad's file parser keeps the vertex and its
/// native parity check counts vertices.
fn normalized_pad(
    mut pad: kiapi::board::types::Pad,
    drop_closing: bool,
) -> kiapi::board::types::Pad {
    pad.id = None;
    pad.net = None;
    pad.pad_to_die_length = None;
    pad.symbol_pin = None;
    pad.pad_to_die_delay = None;
    pad.parent = None;
    if let Some(stack) = pad.pad_stack.as_mut() {
        stack.layers.sort_unstable();
        if stack.drill.as_ref().is_some_and(|drill| {
            drill.start_layer == kiapi::board::types::BoardLayer::BlUndefined as i32
                && drill.end_layer == kiapi::board::types::BoardLayer::BlUndefined as i32
                && drill
                    .diameter
                    .as_ref()
                    .is_none_or(|diameter| diameter.x_nm == 0 && diameter.y_nm == 0)
        }) {
            stack.drill = None;
        }
        stack.zone_settings = Some(konnect_ipc::builders::canonical_pad_zone_settings(stack));
        for layer in &mut stack.copper_layers {
            for shape in &mut layer.custom_shapes {
                shape.id = None;
                shape.net = None;
                shape.parent = None;
                // `PCB_SHAPE::Deserialize` locks only on `LS_LOCKED`.
                if shape.locked == kiapi::common::types::LockedState::LsUnknown as i32 {
                    shape.locked = kiapi::common::types::LockedState::LsUnlocked as i32;
                }
                if let Some(kiapi::common::types::graphic_shape::Geometry::Rectangle(rectangle)) =
                    shape.shape.as_mut().and_then(|s| s.geometry.as_mut())
                {
                    if rectangle
                        .corner_radius
                        .as_ref()
                        .is_some_and(|radius| radius.value_nm == 0)
                    {
                        rectangle.corner_radius = None;
                    }
                }
                if drop_closing {
                    drop_closing_vertices(shape);
                }
            }
        }
    }
    pad
}

/// Remove a polygon outline's last vertex where it repeats the first.
fn drop_closing_vertices(shape: &mut kiapi::board::types::BoardGraphicShape) {
    use kiapi::common::types::{graphic_shape::Geometry, poly_line_node};
    let Some(Geometry::Polygon(set)) = shape.shape.as_mut().and_then(|s| s.geometry.as_mut())
    else {
        return;
    };
    for outline in set
        .polygons
        .iter_mut()
        .filter_map(|polygon| polygon.outline.as_mut())
    {
        let point = |node: Option<&kiapi::common::types::PolyLineNode>| match node
            .and_then(|node| node.geometry.as_ref())
        {
            Some(poly_line_node::Geometry::Point(point)) => Some((point.x_nm, point.y_nm)),
            _ => None,
        };
        let nodes = &outline.nodes;
        if nodes.len() > 3
            && point(nodes.first()).is_some()
            && point(nodes.first()) == point(nodes.last())
        {
            outline.nodes.pop();
        }
    }
}

fn normalized_graphics(definition: &kiapi::board::types::Footprint) -> Result<Vec<Vec<u8>>> {
    let mut graphics = definition
        .items
        .iter()
        .filter_map(|item| {
            if item.type_url.ends_with("BoardGraphicShape") {
                Some(
                    kiapi::board::types::BoardGraphicShape::decode(item.value.as_slice()).map(
                        |mut shape| {
                            shape.id = None;
                            shape.net = None;
                            shape.locked = kiapi::common::types::LockedState::LsUnlocked as i32;
                            if let Some(graphic) = shape.shape.as_mut() {
                                if let Some(stroke) = graphic
                                    .attributes
                                    .as_mut()
                                    .and_then(|attributes| attributes.stroke.as_mut())
                                {
                                    if matches!(
                                        stroke.style(),
                                        kiapi::common::types::StrokeLineStyle::SlsUnknown
                                            | kiapi::common::types::StrokeLineStyle::SlsDefault
                                            | kiapi::common::types::StrokeLineStyle::SlsSolid
                                    ) {
                                        stroke.style =
                                            kiapi::common::types::StrokeLineStyle::SlsSolid as i32;
                                    }
                                    stroke.color = None;
                                }
                                if let Some(
                                    kiapi::common::types::graphic_shape::Geometry::Polygon(polyset),
                                ) = graphic.geometry.as_mut()
                                {
                                    normalize_polyset(polyset);
                                }
                                if let Some(
                                    kiapi::common::types::graphic_shape::Geometry::Rectangle(
                                        rectangle,
                                    ),
                                ) = graphic.geometry.as_mut()
                                {
                                    if rectangle
                                        .corner_radius
                                        .as_ref()
                                        .is_some_and(|radius| radius.value_nm == 0)
                                    {
                                        rectangle.corner_radius = None;
                                    }
                                }
                            }
                            shape.encode_to_vec()
                        },
                    ),
                )
            } else if item.type_url.ends_with("BoardText") {
                Some(
                    kiapi::board::types::BoardText::decode(item.value.as_slice()).map(
                        |mut text| {
                            text.id = None;
                            text.encode_to_vec()
                        },
                    ),
                )
            } else {
                None
            }
        })
        .collect::<std::result::Result<Vec<_>, _>>()?;
    graphics.sort();
    Ok(graphics)
}

fn normalize_polyset(polyset: &mut kiapi::common::types::PolySet) {
    for polygon in &mut polyset.polygons {
        if let Some(outline) = polygon.outline.as_mut() {
            normalize_polyline(outline);
        }
        for hole in &mut polygon.holes {
            normalize_polyline(hole);
        }
        polygon.holes.sort_by_key(prost::Message::encode_to_vec);
    }
    polyset.polygons.sort_by_key(prost::Message::encode_to_vec);
}

fn normalize_polyline(polyline: &mut kiapi::common::types::PolyLine) {
    if !polyline.closed || polyline.nodes.len() < 2 {
        return;
    }
    let candidates = [
        polyline.nodes.clone(),
        polyline.nodes.iter().cloned().rev().collect(),
    ];
    let mut best: Option<(Vec<u8>, Vec<kiapi::common::types::PolyLineNode>)> = None;
    for nodes in candidates {
        for offset in 0..nodes.len() {
            let rotated = nodes[offset..]
                .iter()
                .chain(nodes[..offset].iter())
                .cloned()
                .collect::<Vec<_>>();
            let encoded = rotated
                .iter()
                .flat_map(prost::Message::encode_to_vec)
                .collect::<Vec<_>>();
            if best
                .as_ref()
                .is_none_or(|(best_encoded, _)| encoded < *best_encoded)
            {
                best = Some((encoded, rotated));
            }
        }
    }
    if let Some((_, nodes)) = best {
        polyline.nodes = nodes;
    }
}

fn library_owned_attributes(
    attributes: Option<&kiapi::board::types::FootprintAttributes>,
) -> Option<(bool, bool, bool, bool, i32, bool)> {
    attributes.map(|attributes| {
        let mounting_style = match attributes.mounting_style() {
            kiapi::board::types::FootprintMountingStyle::FmsUnknown
            | kiapi::board::types::FootprintMountingStyle::FmsUnspecified => {
                kiapi::board::types::FootprintMountingStyle::FmsUnspecified as i32
            }
            style => style as i32,
        };
        (
            attributes.exclude_from_position_files,
            attributes.exclude_from_bill_of_materials,
            attributes.exempt_from_courtyard_requirement,
            attributes.allow_soldermask_bridges,
            mounting_style,
            attributes.not_in_schematic,
        )
    })
}

fn field_text(field: &Option<kiapi::board::types::Field>) -> String {
    field
        .as_ref()
        .and_then(|field| field.text.as_ref())
        .and_then(|text| text.text.as_ref())
        .map(|text| text.text.clone())
        .unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;
    use konnect_ipc::builders;
    use konnect_ipc::gen::kiapi;
    use prost::Message;
    use std::collections::{BTreeMap, BTreeSet};

    /// The serialization KiCad 10 actually writes — produced by running this
    /// module's hand-written fixture through `kicad-cli fp upgrade` — parses,
    /// and the two universal flags KiCad stamps into every saved footprint
    /// (`embedded_fonts no`, `duplicate_pad_numbers_are_jumpers no`, plus
    /// `remove_unused_layers no` on pads) do not refuse it. The hand-written
    /// fixture alone cannot catch this class: it shares this module's own
    /// assumptions about the format (see the fixtures-from-KiCad rule in
    /// CONTRIBUTING).
    #[test]
    fn a_footprint_kicad_saved_is_accepted_not_refused() {
        let source = KICAD_LIBRARY_FOOTPRINT;
        let library = parse_library_footprint("Konnect:Socket", source)
            .expect("KiCad's own serialization of the fixture must parse");
        assert_eq!(library.pads.len(), 4, "two '1' variants plus '2' and '3'");
        assert_eq!(
            library.graphics.len(),
            3,
            "one fp_line, one fp_poly, and KiCad's standard fab reference text"
        );
        assert!(library.graphics.iter().any(|graphic| {
            matches!(
                graphic,
                konnect_ipc::IpcGraphicDefinition::Text {
                    text,
                    position,
                    rotation,
                    layer,
                    size,
                    stroke_width_mm,
                } if text == "${REFERENCE}"
                    && *position == (0.0, 1.5)
                    && *rotation == 0.0
                    && layer == "F.Fab"
                    && *size == 1.0
                    && *stroke_width_mm == 0.15
            )
        }));
        assert_eq!(library.datasheet.as_deref(), Some("new-datasheet.pdf"));
        assert_eq!(
            library
                .properties
                .iter()
                .map(|property| (property.name.as_str(), field_text_value(property)))
                .collect::<Vec<_>>(),
            vec![
                ("KiLib_Generator", "konnect_test_generator"),
                ("AssemblyVendor", "Example Assembly"),
            ]
        );
        assert!(library.properties.iter().all(|property| !property.visible));
        assert_eq!(library.models.len(), 1);

        // The same flags at a non-default value carry semantics the typed
        // rebuild cannot represent, so they refuse rather than silently drop.
        for (from, to) in [
            ("(embedded_fonts no)", "(embedded_fonts yes)"),
            ("(remove_unused_layers no)", "(remove_unused_layers yes)"),
        ] {
            let flipped = source.replace(from, to);
            assert_ne!(flipped, source, "fixture must contain {from}");
            let error = parse_library_footprint("Konnect:Socket", &flipped)
                .expect_err("a non-default flag must refuse")
                .to_string();
            assert!(
                error.contains("not supported"),
                "refusal names the unsupported clause: {error}"
            );
        }
    }

    #[test]
    fn stock_kicad_generator_property_is_preserved_as_a_typed_field() {
        // Unmodified KiCad 10.0.5 standard-library output from
        // Capacitor_SMD.pretty/C_0603_1608Metric.kicad_mod. This is the stock
        // footprint that exposed #373; keeping its real generator-authored
        // shape prevents a synthetic fixture from agreeing with this parser.
        let source = include_str!("../../tests/fixtures/c_0603_1608metric_kicad10.kicad_mod");
        let library = parse_library_footprint("Capacitor_SMD:C_0603_1608Metric", source)
            .expect("the stock KiCad generator footprint must parse losslessly");

        assert_eq!(library.pads.len(), 2);
        let generator = library
            .properties
            .iter()
            .find(|property| property.name == "KiLib_Generator")
            .expect("KiCad's generator property must remain a typed Field");
        assert_eq!(field_text_value(generator), "SMD_2terminal_chip_molded");
        assert!(!generator.visible);
        assert_eq!(library.models.len(), 1);
    }

    #[test]
    fn visible_property_is_a_field_only_and_clause_order_does_not_change_its_angle() {
        let source = KICAD_LIBRARY_FOOTPRINT.replace(
            "\t\t(at 0.5 0.75 15)\n\t\t(layer \"F.Fab\")\n\t\t(hide yes)",
            "\t\t(layer \"F.Fab\")\n\t\t(hide no)\n\t\t(at 0.5 0.75 15)",
        );
        assert_ne!(source, KICAD_LIBRARY_FOOTPRINT);

        let library = parse_library_footprint("Konnect:Socket", &source).unwrap();
        let property = library
            .properties
            .iter()
            .find(|property| property.name == "AssemblyVendor")
            .unwrap();
        assert!(property.visible);
        assert_eq!(
            property
                .text
                .as_ref()
                .unwrap()
                .text
                .as_ref()
                .unwrap()
                .attributes
                .as_ref()
                .unwrap()
                .angle
                .as_ref()
                .unwrap()
                .value_degrees,
            15.0
        );
        assert!(!library.graphics.iter().any(|graphic| matches!(
            graphic,
            konnect_ipc::IpcGraphicDefinition::Text { text, .. }
                if text == "Example Assembly"
        )));
    }

    /// `preserved` must be a comparison of the rebuilt instance against the
    /// board's, never a policy constant: when the two instances genuinely
    /// diverge, the flags have to say so.
    #[test]
    fn preserved_flags_come_from_comparison_not_policy() {
        let pad = |number: &str, net: Option<&str>| {
            let item = kiapi::board::types::Pad {
                number: number.to_string(),
                net: net.map(|name| kiapi::board::types::Net {
                    code: Some(kiapi::board::types::NetCode { value: 3 }),
                    name: name.to_string(),
                }),
                ..Default::default()
            };
            builders::pack_any(&item, "kiapi.board.types.Pad")
        };
        let current = kiapi::board::types::FootprintInstance {
            position: Some(builders::vec2(10.0, 20.0)),
            orientation: Some(kiapi::common::types::Angle {
                value_degrees: 90.0,
            }),
            layer: kiapi::board::types::BoardLayer::BlFCu as i32,
            definition: Some(kiapi::board::types::Footprint {
                items: vec![pad("1", Some("GND"))],
                ..Default::default()
            }),
            ..Default::default()
        };
        let old_nets = BTreeMap::from([(
            "1".to_string(),
            kiapi::board::types::Net {
                code: Some(kiapi::board::types::NetCode { value: 3 }),
                name: "GND".to_string(),
            },
        )]);

        let identical = PreservedState::derive(&current, &current.clone(), &old_nets);
        assert!(
            identical.position
                && identical.rotation
                && identical.layer
                && identical.locked
                && identical.kiid
                && identical.symbol_path
                && identical.pad_nets
                && identical.instance_overrides,
            "an untouched instance preserves everything: {identical:?}"
        );

        let mut moved = current.clone();
        moved.position = Some(builders::vec2(99.0, 99.0));
        moved.definition.as_mut().unwrap().items = vec![pad("1", None)];
        let diverged = PreservedState::derive(&current, &moved, &old_nets);
        assert!(
            !diverged.position,
            "a moved instance must report position unpreserved"
        );
        assert!(
            !diverged.pad_nets,
            "a dropped pad net must report pad_nets unpreserved"
        );
        assert!(
            diverged.rotation && diverged.layer && diverged.kiid,
            "untouched fields stay preserved: {diverged:?}"
        );
    }

    const KICAD_LIBRARY_FOOTPRINT: &str =
        include_str!("../../tests/fixtures/socket_kicad10.kicad_mod");

    const LIBRARY_FOOTPRINT: &str = r#"
(footprint "Socket"
  (version 20240108)
  (generator "konnect")
  (layer "F.Cu")
  (descr "updated description")
  (tags "keyboard socket")
  (attr smd exclude_from_pos_files)
  (fp_line (start -1 -2) (end 3 4)
    (stroke (width 0.12) (type solid))
    (layer "F.SilkS"))
  (fp_poly (pts (xy -2 -1) (xy 2 -1) (xy 2 1) (xy -2 1))
    (stroke (width 0.05) (type solid))
    (fill none)
    (layer "B.CrtYd"))
  (pad "1" smd roundrect (at -2 0 15) (size 2 1)
    (layers "B.Cu" "B.Paste" "B.Mask")
    (roundrect_rratio 0.2))
  (pad "1" smd rect (at -1 0) (size 1 1) (layers "B.Cu"))
  (pad "2" thru_hole circle (at 2 0) (size 3 3)
    (layers "*.Cu" "*.Mask") (drill 1))
  (pad "3" smd rect (at 0 3) (size 1 1) (layers "F.Cu"))
  (fp_text reference "REF**" (at 0 -4 0) (layer "F.SilkS")
    (effects (font (size 1 1) (thickness 0.15))))
  (fp_text value "Socket" (at 0 4 0) (layer "F.Fab")
    (effects (font (size 1 1) (thickness 0.15))))
  (fp_text user "${REFERENCE}" (at 0 1.5 0) (layer "F.Fab")
    (uuid "f96c2efe-5925-4f74-81d2-f89a56f57e13")
    (effects (font (size 0.8 0.8) (thickness 0.11))))
  (property "Datasheet" "new-datasheet.pdf" (at 0 0) (layer "F.Fab") (hide yes))
  (property "Description" "new field description" (at 0 0) (layer "F.Fab") (hide yes))
  (model "../models/Socket.step"
    (offset (xyz 1 2 3))
    (scale (xyz -1 1 1))
    (rotate (xyz 90 0 45)))
)
"#;

    fn field(name: &str, value: &str, x: f64, y: f64, visible: bool) -> kiapi::board::types::Field {
        kiapi::board::types::Field {
            name: name.to_string(),
            visible,
            text: Some(kiapi::board::types::BoardText {
                text: Some(kiapi::common::types::Text {
                    position: Some(builders::vec2(x, y)),
                    attributes: Some(kiapi::common::types::TextAttributes {
                        size: Some(builders::vec2(1.25, 1.25)),
                        angle: Some(kiapi::common::types::Angle {
                            value_degrees: 17.0,
                        }),
                        mirrored: true,
                        ..Default::default()
                    }),
                    text: value.to_string(),
                    ..Default::default()
                }),
                layer: kiapi::board::types::BoardLayer::BlBSilkS as i32,
                ..Default::default()
            }),
            ..Default::default()
        }
    }

    fn current_pad(number: &str, net_name: &str, net_code: i32) -> prost_types::Any {
        builders::pack_any(
            &kiapi::board::types::Pad {
                id: Some(kiapi::common::types::Kiid {
                    value: format!("old-pad-{number}-{net_code}"),
                }),
                number: number.to_string(),
                net: Some(kiapi::board::types::Net {
                    code: Some(kiapi::board::types::NetCode { value: net_code }),
                    name: net_name.to_string(),
                }),
                position: Some(builders::vec2(100.0, 50.0)),
                ..Default::default()
            },
            "kiapi.board.types.Pad",
        )
    }

    fn current_instance(
        layer: kiapi::board::types::BoardLayer,
    ) -> kiapi::board::types::FootprintInstance {
        let reference = field("Reference", "SW1", 101.0, 48.0, true);
        let value = field("Value", "Socket Value", 99.0, 52.0, false);
        kiapi::board::types::FootprintInstance {
            id: Some(kiapi::common::types::Kiid {
                value: "instance-kiid".to_string(),
            }),
            position: Some(builders::vec2(100.0, 50.0)),
            orientation: Some(kiapi::common::types::Angle {
                value_degrees: 37.0,
            }),
            layer: layer as i32,
            locked: kiapi::common::types::LockedState::LsLocked as i32,
            definition: Some(kiapi::board::types::Footprint {
                id: Some(kiapi::common::types::LibraryIdentifier {
                    library_nickname: "Test".to_string(),
                    entry_name: "Socket".to_string(),
                }),
                reference_field: Some(reference.clone()),
                value_field: Some(value.clone()),
                items: vec![current_pad("1", "ROW1", 11), current_pad("2", "COL1", 12)],
                ..Default::default()
            }),
            reference_field: Some(reference),
            value_field: Some(value),
            datasheet_field: Some(field("Datasheet", "placed-datasheet", 0.0, 0.0, false)),
            description_field: Some(field("Description", "placed-description", 0.0, 0.0, false)),
            attributes: Some(kiapi::board::types::FootprintAttributes {
                not_in_schematic: false,
                exclude_from_bill_of_materials: true,
                do_not_populate: true,
                ..Default::default()
            }),
            overrides: Some(kiapi::board::types::FootprintDesignRuleOverrides {
                copper_clearance: Some(builders::distance(0.3)),
                ..Default::default()
            }),
            symbol_path: Some(kiapi::common::types::SheetPath {
                path: vec![kiapi::common::types::Kiid {
                    value: "sheet-kiid".to_string(),
                }],
                path_human_readable: "/Keyboard".to_string(),
            }),
            symbol_sheet_name: "Keyboard".to_string(),
            symbol_sheet_filename: "keyboard.kicad_sch".to_string(),
            symbol_footprint_filters: "Test:*".to_string(),
            parent: None,
        }
    }

    fn decoded_pads(
        instance: &kiapi::board::types::FootprintInstance,
    ) -> Vec<kiapi::board::types::Pad> {
        instance
            .definition
            .as_ref()
            .unwrap()
            .items
            .iter()
            .filter(|item| item.type_url.ends_with("kiapi.board.types.Pad"))
            .map(|item| kiapi::board::types::Pad::decode(item.value.as_slice()).unwrap())
            .collect()
    }

    fn decoded_custom_fields(
        instance: &kiapi::board::types::FootprintInstance,
    ) -> Vec<kiapi::board::types::Field> {
        instance
            .definition
            .as_ref()
            .unwrap()
            .items
            .iter()
            .filter(|item| item.type_url.ends_with("kiapi.board.types.Field"))
            .map(|item| {
                kiapi::board::types::Field::decode(item.value.as_slice())
                    .expect("custom field must decode")
            })
            .collect()
    }

    fn field_text_value(field: &kiapi::board::types::Field) -> &str {
        field
            .text
            .as_ref()
            .and_then(|text| text.text.as_ref())
            .map(|text| text.text.as_str())
            .unwrap_or_default()
    }

    #[test]
    fn parses_supported_library_definition_without_dropping_domains() {
        let library = parse_library_footprint("Test:Socket", LIBRARY_FOOTPRINT).unwrap();

        assert_eq!(library.library_id, "Test:Socket");
        assert_eq!(
            library.definition.attributes.as_ref().unwrap().description,
            "updated description"
        );
        assert_eq!(
            library.definition.attributes.as_ref().unwrap().keywords,
            "keyboard socket"
        );
        assert_eq!(library.pads.len(), 4);
        assert_eq!(library.graphics.len(), 3);
        assert!(library.graphics.iter().any(|graphic| matches!(
            graphic,
            konnect_ipc::IpcGraphicDefinition::Text {
                text,
                size,
                stroke_width_mm,
                ..
            } if text == "${REFERENCE}" && *size == 0.8 && *stroke_width_mm == 0.11
        )));
        assert_eq!(library.models.len(), 1);
        let model = &library.models[0];
        assert_eq!(model.filename, "../models/Socket.step");
        assert_eq!(model.offset.as_ref().unwrap().x_nm, 1.0);
        assert_eq!(model.scale.as_ref().unwrap().x_nm, -1.0);
        assert_eq!(model.rotation.as_ref().unwrap().z_nm, 45.0);
    }

    #[test]
    fn merge_preserves_instance_state_and_nets_by_logical_pad_number() {
        let current = current_instance(kiapi::board::types::BoardLayer::BlBCu);
        let library = parse_library_footprint("Test:Socket", KICAD_LIBRARY_FOOTPRINT).unwrap();
        let prepared = build_updated_instance(
            &current,
            &library,
            &BTreeMap::from([("ROW1".to_string(), 11), ("COL1".to_string(), 12)]),
            &BTreeSet::from(["ROW1".to_string(), "COL1".to_string()]),
        )
        .unwrap();
        let updated =
            kiapi::board::types::FootprintInstance::decode(prepared.item.value.as_slice()).unwrap();

        assert_eq!(updated.id, current.id);
        assert_eq!(updated.position, current.position);
        assert_eq!(updated.orientation, current.orientation);
        assert_eq!(updated.layer, current.layer);
        assert_eq!(updated.locked, current.locked);
        assert_eq!(updated.reference_field, current.reference_field);
        assert_eq!(updated.value_field, current.value_field);
        assert_eq!(field_text(&updated.datasheet_field), "new-datasheet.pdf");
        assert_eq!(
            updated
                .datasheet_field
                .as_ref()
                .and_then(|field| field.text.as_ref())
                .and_then(|text| text.text.as_ref())
                .and_then(|text| text.attributes.as_ref()),
            current
                .datasheet_field
                .as_ref()
                .and_then(|field| field.text.as_ref())
                .and_then(|text| text.text.as_ref())
                .and_then(|text| text.attributes.as_ref())
        );
        assert_eq!(
            field_text(&updated.description_field),
            "new field description"
        );
        let attributes = updated.attributes.as_ref().unwrap();
        let current_attributes = current.attributes.as_ref().unwrap();
        assert_eq!(
            attributes.not_in_schematic,
            current_attributes.not_in_schematic
        );
        assert_eq!(
            attributes.do_not_populate,
            current_attributes.do_not_populate
        );
        assert!(attributes.exclude_from_position_files);
        assert!(!attributes.exclude_from_bill_of_materials);
        assert_eq!(
            attributes.mounting_style,
            kiapi::board::types::FootprintMountingStyle::FmsSmd as i32
        );
        assert_eq!(updated.overrides, current.overrides);
        assert_eq!(updated.symbol_path, current.symbol_path);
        assert_eq!(updated.symbol_sheet_name, current.symbol_sheet_name);
        assert_eq!(updated.symbol_sheet_filename, current.symbol_sheet_filename);
        assert_eq!(
            updated.symbol_footprint_filters,
            current.symbol_footprint_filters
        );

        let properties = decoded_custom_fields(&updated);
        assert_eq!(
            properties
                .iter()
                .map(|property| (property.name.as_str(), field_text_value(property)))
                .collect::<Vec<_>>(),
            vec![
                ("KiLib_Generator", "konnect_test_generator"),
                ("AssemblyVendor", "Example Assembly"),
            ]
        );
        let assembly = properties
            .iter()
            .find(|property| property.name == "AssemblyVendor")
            .unwrap();
        let assembly_text = assembly.text.as_ref().unwrap();
        assert_eq!(
            assembly_text.layer,
            kiapi::board::types::BoardLayer::BlBFab as i32
        );
        let text = assembly_text.text.as_ref().unwrap();
        let expected = konnect_sexp::geometry::transform_pad(0.5, -0.75, 100.0, 50.0, 37.0);
        let actual = text.position.as_ref().unwrap();
        let actual = (
            builders::nm_to_mm(actual.x_nm),
            builders::nm_to_mm(actual.y_nm),
        );
        assert!((actual.0 - expected.0).abs() <= 0.000_001);
        assert!((actual.1 - expected.1).abs() <= 0.000_001);
        let attributes = text.attributes.as_ref().unwrap();
        assert!(attributes.mirrored);
        assert_eq!(
            attributes.angle.as_ref().unwrap().value_degrees,
            readable_property_angle(180.0 - 15.0 + 37.0)
        );

        let pads = decoded_pads(&updated);
        assert_eq!(pads.iter().filter(|pad| pad.number == "1").count(), 2);
        assert!(pads.iter().filter(|pad| pad.number == "1").all(|pad| pad
            .net
            .as_ref()
            .map(|net| net.name.as_str())
            == Some("ROW1")));
        assert_eq!(
            pads.iter()
                .find(|pad| pad.number == "2")
                .and_then(|pad| pad.net.as_ref())
                .map(|net| net.name.as_str()),
            Some("COL1")
        );
        assert!(pads
            .iter()
            .find(|pad| pad.number == "3")
            .unwrap()
            .net
            .is_none());
        let flipped_pad = pads
            .iter()
            .find(|pad| pad.number == "3")
            .expect("new pad 3");
        let flipped_stack = flipped_pad.pad_stack.as_ref().expect("pad stack");
        assert_eq!(
            flipped_stack.layers,
            vec![kiapi::board::types::BoardLayer::BlBCu as i32],
            "a front-copper library pad moves to back copper with a B.Cu instance"
        );
        let expected = konnect_sexp::geometry::transform_pad(0.0, -3.0, 100.0, 50.0, 37.0);
        let actual = flipped_pad.position.as_ref().expect("pad position");
        let actual = (
            builders::nm_to_mm(actual.x_nm),
            builders::nm_to_mm(actual.y_nm),
        );
        assert!((actual.0 - expected.0).abs() <= 0.000_001);
        assert!((actual.1 - expected.1).abs() <= 0.000_001);
        assert!(prepared.changed_domains.contains(&ChangedDomain::Pads));
        assert!(prepared.changed_domains.contains(&ChangedDomain::Graphics));
        assert!(prepared.changed_domains.contains(&ChangedDomain::Metadata));
        assert!(prepared.changed_domains.contains(&ChangedDomain::Models));
    }

    #[test]
    fn merge_preserves_instance_only_properties_and_refreshes_library_properties() {
        let mut current = current_instance(kiapi::board::types::BoardLayer::BlFCu);
        let instance_only = field("InstanceNote", "keep this", 105.0, 55.0, false);
        let stale_library_property = field("AssemblyVendor", "old library value", 99.0, 49.0, true);
        current.definition.as_mut().unwrap().items.extend([
            builders::pack_any(&instance_only, "kiapi.board.types.Field"),
            builders::pack_any(&stale_library_property, "kiapi.board.types.Field"),
        ]);
        let library = parse_library_footprint("Test:Socket", KICAD_LIBRARY_FOOTPRINT).unwrap();

        let prepared = build_updated_instance(
            &current,
            &library,
            &BTreeMap::from([("ROW1".to_string(), 11), ("COL1".to_string(), 12)]),
            &BTreeSet::new(),
        )
        .unwrap();
        let updated =
            kiapi::board::types::FootprintInstance::decode(prepared.item.value.as_slice()).unwrap();
        let properties = decoded_custom_fields(&updated);

        assert_eq!(
            properties
                .iter()
                .map(|property| property.name.as_str())
                .collect::<Vec<_>>(),
            vec!["InstanceNote", "KiLib_Generator", "AssemblyVendor"]
        );
        assert_eq!(
            properties
                .iter()
                .find(|property| property.name == "InstanceNote")
                .unwrap(),
            &instance_only
        );
        assert_eq!(
            field_text_value(
                properties
                    .iter()
                    .find(|property| property.name == "AssemblyVendor")
                    .unwrap()
            ),
            "Example Assembly"
        );
        assert!(prepared.changed_domains.contains(&ChangedDomain::Metadata));
    }

    #[test]
    fn merge_conflicts_when_library_removes_a_connected_logical_pad() {
        let current = current_instance(kiapi::board::types::BoardLayer::BlFCu);
        let without_pad_two = LIBRARY_FOOTPRINT.replace(
            "  (pad \"2\" thru_hole circle (at 2 0) (size 3 3)\n    (layers \"*.Cu\" \"*.Mask\") (drill 1))\n",
            "",
        );
        let library = parse_library_footprint("Test:Socket", &without_pad_two).unwrap();

        let error = build_updated_instance(
            &current,
            &library,
            &BTreeMap::from([("ROW1".to_string(), 11), ("COL1".to_string(), 12)]),
            &BTreeSet::from(["COL1".to_string()]),
        )
        .unwrap_err();

        assert!(error.to_string().contains("connected pad 2"), "{error:#}");
    }

    #[test]
    fn parser_rejects_unsupported_children_before_any_update_can_be_built() {
        let unsupported = LIBRARY_FOOTPRINT.replace(
            "  (model \"../models/Socket.step\"",
            "  (zone (net 0) (layers \"F.Cu\"))\n  (model \"../models/Socket.step\"",
        );

        let error = parse_library_footprint("Test:Socket", &unsupported).unwrap_err();

        assert!(error.to_string().contains("zone"), "{error:#}");
    }

    #[test]
    fn parser_names_unrepresentable_or_ambiguous_custom_properties() {
        let unsupported = KICAD_LIBRARY_FOOTPRINT.replace(
            "\t(property \"AssemblyVendor\" \"Example Assembly\"\n\t\t(at",
            "\t(property \"AssemblyVendor\" \"Example Assembly\"\n\t\t(unlocked yes)\n\t\t(at",
        );
        assert_ne!(unsupported, KICAD_LIBRARY_FOOTPRINT);
        let error = parse_library_footprint("Test:Socket", &unsupported).unwrap_err();
        let message = error.to_string();
        assert!(message.contains("AssemblyVendor"), "{message}");
        assert!(message.contains("unlocked"), "{message}");

        let duplicate = KICAD_LIBRARY_FOOTPRINT.replace(
            "\"KiLib_Generator\" \"konnect_test_generator\"",
            "\"AssemblyVendor\" \"konnect_test_generator\"",
        );
        assert_ne!(duplicate, KICAD_LIBRARY_FOOTPRINT);
        let error = parse_library_footprint("Test:Socket", &duplicate).unwrap_err();
        let message = error.to_string();
        assert!(message.contains("AssemblyVendor"), "{message}");
        assert!(message.contains("more than once"), "{message}");
    }

    #[test]
    fn mandatory_properties_are_validated_and_duplicate_names_refuse() {
        let mut accepted = Vec::new();
        for (name, value) in [
            ("Reference", "REF**"),
            ("Value", "Socket"),
            ("Datasheet", "new-datasheet.pdf"),
            ("Description", "new field description"),
        ] {
            let authored = format!("\t(property \"{name}\" \"{value}\"\n\t\t(at");
            let unsupported = KICAD_LIBRARY_FOOTPRINT.replace(
                &authored,
                &format!("\t(property \"{name}\" \"{value}\"\n\t\t(unlocked yes)\n\t\t(at"),
            );
            assert_ne!(unsupported, KICAD_LIBRARY_FOOTPRINT);
            match parse_library_footprint("Test:Socket", &unsupported) {
                Ok(_) => accepted.push(format!("{name} unsupported clause")),
                Err(error) => {
                    let message = error.to_string();
                    assert!(message.contains(name), "{message}");
                    assert!(message.contains("unlocked"), "{message}");
                }
            }

            let duplicate = KICAD_LIBRARY_FOOTPRINT.replace(
                "\"KiLib_Generator\" \"konnect_test_generator\"",
                &format!("\"{name}\" \"konnect_test_generator\""),
            );
            assert_ne!(duplicate, KICAD_LIBRARY_FOOTPRINT);
            match parse_library_footprint("Test:Socket", &duplicate) {
                Ok(_) => accepted.push(format!("{name} duplicate name")),
                Err(error) => {
                    let message = error.to_string();
                    assert!(message.contains(name), "{message}");
                    assert!(message.contains("more than once"), "{message}");
                }
            }
        }
        assert!(
            accepted.is_empty(),
            "parser accepted mandatory-property cases that must refuse: {}",
            accepted.join(", ")
        );
    }

    #[test]
    fn repeated_scalar_property_clauses_refuse() {
        let mut accepted = Vec::new();
        let vendor_prefix = "\t(property \"AssemblyVendor\" \"Example Assembly\"\n\t\t(at 0.5 0.75 15)\n\t\t(layer \"F.Fab\")\n\t\t(hide yes)";
        for (clause, replacement) in [
            ("hide", format!("{vendor_prefix}\n\t\t(hide no)")),
            (
                "knockout",
                format!("{vendor_prefix}\n\t\t(knockout yes)\n\t\t(knockout no)"),
            ),
        ] {
            let repeated = KICAD_LIBRARY_FOOTPRINT.replace(vendor_prefix, &replacement);
            assert_ne!(repeated, KICAD_LIBRARY_FOOTPRINT);
            match parse_library_footprint("Test:Socket", &repeated) {
                Ok(_) => accepted.push(clause),
                Err(error) => {
                    let message = error.to_string();
                    assert!(message.contains("AssemblyVendor"), "{message}");
                    assert!(message.contains(clause), "{message}");
                }
            }
        }

        let vendor_font =
            "\t\t\t\t(size 1 1)\n\t\t\t\t(thickness 0.15)\n\t\t\t)\n\t\t)\n\t)\n\t(attr";
        for (clause, repeated_clauses) in [
            ("bold", "\t\t\t\t(bold yes)\n\t\t\t\t(bold no)\n"),
            ("italic", "\t\t\t\t(italic yes)\n\t\t\t\t(italic no)\n"),
            (
                "line_spacing",
                "\t\t\t\t(line_spacing 1.1)\n\t\t\t\t(line_spacing 1.2)\n",
            ),
        ] {
            let replacement =
                vendor_font.replace("\t\t\t)\n", &format!("{repeated_clauses}\t\t\t)\n"));
            let repeated = KICAD_LIBRARY_FOOTPRINT.replace(vendor_font, &replacement);
            assert_ne!(repeated, KICAD_LIBRARY_FOOTPRINT);
            match parse_library_footprint("Test:Socket", &repeated) {
                Ok(_) => accepted.push(clause),
                Err(error) => {
                    let message = error.to_string();
                    assert!(message.contains("AssemblyVendor"), "{message}");
                    assert!(message.contains(clause), "{message}");
                }
            }
        }
        assert!(
            accepted.is_empty(),
            "parser accepted repeated scalar clauses that must refuse: {}",
            accepted.join(", ")
        );
    }

    #[test]
    fn refresh_refuses_an_unrepresentable_layer_before_building_the_update() {
        let unsupported = LIBRARY_FOOTPRINT.replace("(layer \"F.SilkS\"))", "(layer \"In99.Cu\"))");
        let library = parse_library_footprint("Test:Socket", &unsupported).unwrap();
        let current = current_instance(kiapi::board::types::BoardLayer::BlFCu);

        let error = build_updated_instance(
            &current,
            &library,
            &BTreeMap::from([("ROW1".to_string(), 11), ("COL1".to_string(), 12)]),
            &BTreeSet::new(),
        )
        .unwrap_err();

        let message = format!("{error:#}");
        assert!(message.contains("In99.Cu"), "{message}");
        assert!(message.contains("request was not sent"), "{message}");
    }

    #[test]
    fn parser_rejects_lossy_graphic_and_pad_clauses() {
        let dashed = LIBRARY_FOOTPRINT.replace(
            "(stroke (width 0.12) (type solid))",
            "(stroke (width 0.12) (type dash))",
        );
        let error = parse_library_footprint("Test:Socket", &dashed).unwrap_err();
        assert!(error.to_string().contains("stroke type"), "{error:#}");

        let solder_margin = LIBRARY_FOOTPRINT.replace(
            "(roundrect_rratio 0.2))",
            "(roundrect_rratio 0.2) (solder_mask_margin 0.05))",
        );
        let error = parse_library_footprint("Test:Socket", &solder_margin).unwrap_err();
        assert!(
            error.to_string().contains("solder_mask_margin"),
            "{error:#}"
        );
    }

    #[test]
    fn parser_rejects_lossy_user_text_variants() {
        for (from, to, expected) in [
            (
                "(at 0 1.5 0) (layer \"F.Fab\")",
                "(at 0 1.5 0) (unlocked yes) (layer \"F.Fab\")",
                "unlocked",
            ),
            (
                "(effects (font (size 0.8 0.8) (thickness 0.11)))",
                "(effects (font (size 0.8 0.7) (thickness 0.11)))",
                "non-square",
            ),
            (
                "(effects (font (size 0.8 0.8) (thickness 0.11)))",
                "(effects (font (size 0.8 0.8) (thickness 0.11)) (justify left))",
                "justify",
            ),
            (
                "(effects (font (size 0.8 0.8) (thickness 0.11)))",
                "(effects (font (size 0.8 0.8) (thickness 0.11) (italic yes)))",
                "italic",
            ),
            (
                "(effects (font (size 0.8 0.8) (thickness 0.11)))",
                "(effects (font (size 0.8 0.8)))",
                "missing its thickness",
            ),
            (
                "(uuid \"f96c2efe-5925-4f74-81d2-f89a56f57e13\")",
                "(uuid \"f96c2efe-5925-4f74-81d2-f89a56f57e13\") (uuid \"duplicate\")",
                "multiple identifier clauses",
            ),
            (
                "(uuid \"f96c2efe-5925-4f74-81d2-f89a56f57e13\")",
                "(uuid \"f96c2efe-5925-4f74-81d2-f89a56f57e13\") (tstamp \"legacy-too\")",
                "multiple identifier clauses",
            ),
        ] {
            let unsupported = LIBRARY_FOOTPRINT.replacen(from, to, 1);
            let error = parse_library_footprint("Test:Socket", &unsupported).unwrap_err();
            assert!(error.to_string().contains(expected), "{error:#}");
        }
    }

    #[test]
    fn rebuilding_an_applied_instance_is_a_noop_at_every_rotation_and_side() {
        let library = parse_library_footprint("Test:Socket", KICAD_LIBRARY_FOOTPRINT).unwrap();
        let net_codes = BTreeMap::from([("ROW1".to_string(), 11), ("COL1".to_string(), 12)]);
        let routed = BTreeSet::from(["ROW1".to_string(), "COL1".to_string()]);

        for layer in [
            kiapi::board::types::BoardLayer::BlFCu,
            kiapi::board::types::BoardLayer::BlBCu,
        ] {
            for rotation in [0.0, 90.0, 180.0, 270.0, 37.0] {
                let mut current = current_instance(layer);
                current.orientation = Some(kiapi::common::types::Angle {
                    value_degrees: rotation,
                });
                let first =
                    build_updated_instance(&current, &library, &net_codes, &routed).unwrap();
                let applied =
                    kiapi::board::types::FootprintInstance::decode(first.item.value.as_slice())
                        .unwrap();

                let second =
                    build_updated_instance(&applied, &library, &net_codes, &routed).unwrap();

                assert!(
                    second.changed_domains.is_empty(),
                    "{layer:?} at {rotation} degrees changed again: {:?}",
                    second.changed_domains
                );
            }
        }
    }

    #[test]
    fn kicad_roundtrip_defaults_do_not_create_false_pad_or_graphic_changes() {
        let library = parse_library_footprint("Test:Socket", LIBRARY_FOOTPRINT).unwrap();
        let current = current_instance(kiapi::board::types::BoardLayer::BlFCu);
        let prepared = build_updated_instance(
            &current,
            &library,
            &BTreeMap::from([("ROW1".to_string(), 11), ("COL1".to_string(), 12)]),
            &BTreeSet::new(),
        )
        .unwrap();
        let mut prepared =
            kiapi::board::types::FootprintInstance::decode(prepared.item.value.as_slice()).unwrap();
        let mut roundtripped = prepared.clone();
        roundtripped.definition.as_mut().unwrap().items.reverse();
        roundtripped.attributes.as_mut().unwrap().mounting_style =
            kiapi::board::types::FootprintMountingStyle::FmsUnspecified as i32;
        prepared.attributes.as_mut().unwrap().mounting_style =
            kiapi::board::types::FootprintMountingStyle::FmsUnknown as i32;

        for item in &mut roundtripped.definition.as_mut().unwrap().items {
            if item.type_url.ends_with("kiapi.board.types.Pad") {
                let mut pad = kiapi::board::types::Pad::decode(item.value.as_slice()).unwrap();
                pad.id = Some(kiapi::common::types::Kiid {
                    value: format!("kicad-pad-{}", pad.number),
                });
                pad.net = Some(kiapi::board::types::Net::default());
                pad.pad_to_die_length = Some(kiapi::common::types::Distance { value_nm: 0 });
                pad.symbol_pin = Some(kiapi::board::types::SymbolPinInfo::default());
                pad.pad_to_die_delay = Some(kiapi::common::types::Time { value_as: 0 });
                let stack = pad.pad_stack.as_mut().unwrap();
                stack.layers.reverse();
                if stack.drill.is_none() {
                    stack.drill = Some(kiapi::board::types::DrillProperties {
                        start_layer: kiapi::board::types::BoardLayer::BlUndefined as i32,
                        end_layer: kiapi::board::types::BoardLayer::BlUndefined as i32,
                        diameter: Some(builders::vec2(0.0, 0.0)),
                        shape: kiapi::board::types::DrillShape::DsUndefined as i32,
                        ..Default::default()
                    });
                }
                // KiCad always reports the connection and the spoke angle.
                stack.zone_settings = Some(builders::canonical_pad_zone_settings(stack));
                for shape in &mut stack
                    .copper_layers
                    .iter_mut()
                    .flat_map(|layer| layer.custom_shapes.iter_mut())
                {
                    shape.id = Some(kiapi::common::types::Kiid {
                        value: "kicad-primitive".to_string(),
                    });
                    shape.net = Some(kiapi::board::types::Net::default());
                }
                *item = builders::pack_any(&pad, "kiapi.board.types.Pad");
            } else if item
                .type_url
                .ends_with("kiapi.board.types.BoardGraphicShape")
            {
                let mut graphic =
                    kiapi::board::types::BoardGraphicShape::decode(item.value.as_slice()).unwrap();
                graphic.id = Some(kiapi::common::types::Kiid {
                    value: "kicad-graphic".to_string(),
                });
                graphic.net = Some(kiapi::board::types::Net::default());
                graphic
                    .shape
                    .as_mut()
                    .unwrap()
                    .attributes
                    .as_mut()
                    .unwrap()
                    .stroke
                    .as_mut()
                    .unwrap()
                    .style = kiapi::common::types::StrokeLineStyle::SlsSolid as i32;
                if let Some(kiapi::common::types::graphic_shape::Geometry::Polygon(polyset)) =
                    graphic
                        .shape
                        .as_mut()
                        .and_then(|shape| shape.geometry.as_mut())
                {
                    if let Some(outline) = polyset
                        .polygons
                        .first_mut()
                        .and_then(|polygon| polygon.outline.as_mut())
                    {
                        outline.nodes.rotate_left(1);
                        outline.nodes.reverse();
                    }
                }
                if let Some(kiapi::common::types::graphic_shape::Geometry::Rectangle(rectangle)) =
                    graphic
                        .shape
                        .as_mut()
                        .and_then(|shape| shape.geometry.as_mut())
                {
                    rectangle.corner_radius = Some(kiapi::common::types::Distance { value_nm: 0 });
                }
                *item = builders::pack_any(&graphic, "kiapi.board.types.BoardGraphicShape");
            }
        }

        let roundtripped_pads = normalized_items(roundtripped.definition.as_ref().unwrap(), "Pad")
            .unwrap()
            .into_iter()
            .map(|bytes| kiapi::board::types::Pad::decode(bytes.as_slice()).unwrap())
            .collect::<Vec<_>>();
        let prepared_pads = normalized_items(prepared.definition.as_ref().unwrap(), "Pad")
            .unwrap()
            .into_iter()
            .map(|bytes| kiapi::board::types::Pad::decode(bytes.as_slice()).unwrap())
            .collect::<Vec<_>>();
        assert_eq!(roundtripped_pads, prepared_pads);
        assert!(changed_domains(&roundtripped, &prepared)
            .unwrap()
            .is_empty());
    }

    fn plan_item(reference: &str, library_id: Option<&str>, kiid: &str) -> prost_types::Any {
        let mut instance = current_instance(kiapi::board::types::BoardLayer::BlFCu);
        instance.id = Some(kiapi::common::types::Kiid {
            value: kiid.to_string(),
        });
        instance.reference_field = Some(field("Reference", reference, 101.0, 48.0, true));
        if let Some(definition) = instance.definition.as_mut() {
            definition.reference_field = instance.reference_field.clone();
            definition.id = library_id.map(|library_id| {
                let (nickname, entry) = library_id.split_once(':').unwrap();
                kiapi::common::types::LibraryIdentifier {
                    library_nickname: nickname.to_string(),
                    entry_name: entry.to_string(),
                }
            });
        }
        builders::pack_any(&instance, "kiapi.board.types.FootprintInstance")
    }

    fn plan_fixture() -> (tempfile::TempDir, std::path::PathBuf, Vec<prost_types::Any>) {
        let temp = tempfile::tempdir().unwrap();
        let board = temp.path().join("board.kicad_pcb");
        std::fs::write(&board, "(kicad_pcb (version 20240108))").unwrap();
        let library_dir = temp.path().join("Test.pretty");
        std::fs::create_dir(&library_dir).unwrap();
        std::fs::write(
            library_dir.join("Socket.kicad_mod"),
            KICAD_LIBRARY_FOOTPRINT,
        )
        .unwrap();
        std::fs::write(
            temp.path().join("fp-lib-table"),
            "(fp_lib_table (lib (name \"Test\") (type \"KiCad\") (uri \"${KIPRJMOD}/Test.pretty\") (options \"\") (descr \"\")))",
        )
        .unwrap();
        let items = vec![
            plan_item("SW2", Some("Test:Socket"), "kiid-2"),
            plan_item("TP1", None, "kiid-3"),
            plan_item("SW1", Some("Test:Socket"), "kiid-1"),
        ];
        (temp, board, items)
    }

    #[test]
    fn planner_refuses_an_unrepresentable_property_before_preparing_any_update() {
        let (temp, board, items) = plan_fixture();
        let unsupported = KICAD_LIBRARY_FOOTPRINT.replace(
            "\t(property \"AssemblyVendor\" \"Example Assembly\"\n\t\t(at",
            "\t(property \"AssemblyVendor\" \"Example Assembly\"\n\t\t(unlocked yes)\n\t\t(at",
        );
        std::fs::write(
            temp.path().join("Test.pretty/Socket.kicad_mod"),
            unsupported,
        )
        .unwrap();

        let plan = plan_updates(
            &board,
            &items,
            &BTreeMap::from([("ROW1".to_string(), 11), ("COL1".to_string(), 12)]),
            &BTreeSet::new(),
            &UpdateFilters::default(),
        );

        assert_eq!(plan.status, PlanStatus::Conflict);
        assert!(plan.prepared_items.is_empty());
        assert!(plan.changes.is_empty());
        assert!(plan.diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "unsupported_library_footprint"
                && diagnostic.message.contains("AssemblyVendor")
                && diagnostic.message.contains("unlocked")
        }));
    }

    #[test]
    fn filters_are_normalized_and_supplied_empty_arrays_select_nothing() {
        let filters = parse_filters(&serde_json::json!({
            "references": ["SW2", "SW1", "SW2"],
            "library_ids": ["Test:Socket", "Test:Socket"]
        }))
        .unwrap();
        assert_eq!(
            filters.references,
            Some(BTreeSet::from(["SW1".to_string(), "SW2".to_string()]))
        );
        assert_eq!(
            filters.library_ids,
            Some(BTreeSet::from(["Test:Socket".to_string()]))
        );

        let (_, board, items) = plan_fixture();
        let empty = plan_updates(
            &board,
            &items,
            &BTreeMap::from([("ROW1".to_string(), 11), ("COL1".to_string(), 12)]),
            &BTreeSet::new(),
            &parse_filters(&serde_json::json!({ "references": [] })).unwrap(),
        );
        assert_eq!(empty.status, PlanStatus::Noop);
        assert_eq!(empty.coverage.selected.planned, 0);
        assert!(empty.changes.is_empty());

        let malformed =
            parse_filters(&serde_json::json!({ "library_ids": ["not-a-library-id"] })).unwrap_err();
        assert_eq!(malformed.field, "library_ids");
        assert!(malformed.reason.contains("Library:Footprint"));
    }

    #[test]
    fn planner_selects_deterministically_and_intersects_filters() {
        let (_temp, board, items) = plan_fixture();
        let net_codes = BTreeMap::from([("ROW1".to_string(), 11), ("COL1".to_string(), 12)]);
        let all = plan_updates(
            &board,
            &items,
            &net_codes,
            &BTreeSet::new(),
            &UpdateFilters::default(),
        );
        assert_eq!(all.status, PlanStatus::Ready);
        assert_eq!(all.coverage.selected.planned, 2);
        assert_eq!(all.coverage.changed.planned, 2);
        assert_eq!(all.coverage.skipped_unlinked.planned, 1);
        assert_eq!(
            all.changes
                .iter()
                .map(|change| change.reference.as_str())
                .collect::<Vec<_>>(),
            vec!["SW1", "SW2"]
        );

        let intersection = plan_updates(
            &board,
            &items,
            &net_codes,
            &BTreeSet::new(),
            &parse_filters(&serde_json::json!({
                "references": ["SW2"],
                "library_ids": ["Other:Socket"]
            }))
            .unwrap(),
        );
        assert_eq!(intersection.status, PlanStatus::Noop);
        assert_eq!(intersection.coverage.selected.planned, 0);
    }

    #[test]
    fn planner_conflicts_on_missing_or_duplicate_selected_references() {
        let (_temp, board, mut items) = plan_fixture();
        let net_codes = BTreeMap::from([("ROW1".to_string(), 11), ("COL1".to_string(), 12)]);
        let missing = plan_updates(
            &board,
            &items,
            &net_codes,
            &BTreeSet::new(),
            &parse_filters(&serde_json::json!({ "references": ["SW404"] })).unwrap(),
        );
        assert_eq!(missing.status, PlanStatus::Conflict);
        assert!(missing
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.code == "reference_not_found"));

        items.push(plan_item("SW1", Some("Test:Socket"), "kiid-duplicate"));
        let duplicate = plan_updates(
            &board,
            &items,
            &net_codes,
            &BTreeSet::new(),
            &parse_filters(&serde_json::json!({ "references": ["SW1"] })).unwrap(),
        );
        assert_eq!(duplicate.status, PlanStatus::Conflict);
        assert!(duplicate
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.code == "duplicate_reference"));
        assert!(duplicate.prepared_items.is_empty());
    }

    #[test]
    fn explicitly_selected_unlinked_footprint_is_a_conflict() {
        let (_temp, board, items) = plan_fixture();
        let plan = plan_updates(
            &board,
            &items,
            &BTreeMap::from([("ROW1".to_string(), 11), ("COL1".to_string(), 12)]),
            &BTreeSet::new(),
            &parse_filters(&serde_json::json!({ "references": ["TP1"] })).unwrap(),
        );

        assert_eq!(plan.status, PlanStatus::Conflict);
        assert!(plan
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.code == "unlinked_footprint"));
        assert!(plan.prepared_items.is_empty());
    }

    #[test]
    fn planner_returns_noop_after_the_prepared_update_is_applied() {
        let (_temp, board, items) = plan_fixture();
        let net_codes = BTreeMap::from([("ROW1".to_string(), 11), ("COL1".to_string(), 12)]);
        let first = plan_updates(
            &board,
            &[items[2].clone()],
            &net_codes,
            &BTreeSet::new(),
            &UpdateFilters::default(),
        );
        assert_eq!(first.status, PlanStatus::Ready);

        let second = plan_updates(
            &board,
            &first.prepared_items,
            &net_codes,
            &BTreeSet::new(),
            &UpdateFilters::default(),
        );

        assert_eq!(second.status, PlanStatus::Noop);
        assert_eq!(second.coverage.selected.planned, 1);
        assert_eq!(second.coverage.changed.planned, 0);
        assert_eq!(second.coverage.unchanged.planned, 1);
        assert!(second.prepared_items.is_empty());
    }

    #[test]
    fn datasheet_only_library_change_is_reported_as_metadata() {
        let base = parse_library_footprint("Test:Socket", LIBRARY_FOOTPRINT).unwrap();
        let current = current_instance(kiapi::board::types::BoardLayer::BlFCu);
        let net_codes = BTreeMap::from([("ROW1".to_string(), 11), ("COL1".to_string(), 12)]);
        let routed = BTreeSet::new();
        let applied = build_updated_instance(&current, &base, &net_codes, &routed).unwrap();
        let applied =
            kiapi::board::types::FootprintInstance::decode(applied.item.value.as_slice()).unwrap();
        let changed_source =
            LIBRARY_FOOTPRINT.replace("new-datasheet.pdf", "replacement-datasheet.pdf");
        let changed = parse_library_footprint("Test:Socket", &changed_source).unwrap();

        let prepared = build_updated_instance(&applied, &changed, &net_codes, &routed).unwrap();

        assert_eq!(
            prepared.changed_domains,
            BTreeSet::from([ChangedDomain::Metadata])
        );
    }

    #[test]
    fn plan_revision_tracks_selected_board_library_and_filter_inputs_only() {
        let (_temp, board, mut items) = plan_fixture();
        let net_codes = BTreeMap::from([("ROW1".to_string(), 11), ("COL1".to_string(), 12)]);
        let filters = parse_filters(&serde_json::json!({ "references": ["SW1"] })).unwrap();
        let first = plan_updates(&board, &items, &net_codes, &BTreeSet::new(), &filters);
        let identical = plan_updates(&board, &items, &net_codes, &BTreeSet::new(), &filters);
        assert_eq!(first.plan_revision, identical.plan_revision);

        items[0] = plan_item("SW2", Some("Test:Socket"), "unselected-changed");
        let unrelated = plan_updates(&board, &items, &net_codes, &BTreeSet::new(), &filters);
        assert_eq!(first.plan_revision, unrelated.plan_revision);

        items[2] = plan_item("SW1", Some("Test:Socket"), "selected-changed");
        let board_changed = plan_updates(&board, &items, &net_codes, &BTreeSet::new(), &filters);
        assert_ne!(first.plan_revision, board_changed.plan_revision);

        std::fs::write(
            board.parent().unwrap().join("Test.pretty/Socket.kicad_mod"),
            LIBRARY_FOOTPRINT.replace("updated description", "library changed"),
        )
        .unwrap();
        let library_changed = plan_updates(&board, &items, &net_codes, &BTreeSet::new(), &filters);
        assert_ne!(board_changed.plan_revision, library_changed.plan_revision);

        let different_filter = plan_updates(
            &board,
            &items,
            &net_codes,
            &BTreeSet::new(),
            &parse_filters(&serde_json::json!({ "references": ["SW2"] })).unwrap(),
        );
        assert_ne!(
            library_changed.plan_revision,
            different_filter.plan_revision
        );
    }

    fn test_context(ipc_address: &str) -> crate::tools::ToolContext {
        crate::tools::ToolContext::new(
            crate::tools::ServerConfig {
                kicad_cli: String::new(),
                kicad_binary: String::new(),
                ipc_address: ipc_address.to_string(),
                project_dir: None,
                jlcpcb_db_path: None,
                auto_load_toolsets: false,
                eager_toolsets: false,
            },
            std::sync::Arc::new(crate::router::ToolRouter::new()),
        )
    }

    fn result_json(result: &crate::mcp::protocol::CallToolResult) -> serde_json::Value {
        let text = match result.content.first() {
            Some(crate::mcp::protocol::ToolContent::Text { text }) => text,
            other => panic!("expected text result, got {other:?}"),
        };
        serde_json::from_str(text).expect("tool result must be JSON")
    }

    #[test]
    fn tool_schema_defaults_to_dry_run_and_requires_only_the_board() {
        let definition = tool();

        assert_eq!(definition.name, "update_footprints_from_library");
        assert_eq!(
            definition.input_schema["required"],
            serde_json::json!(["board"])
        );
        assert_eq!(
            definition.input_schema["properties"]["dry_run"]["default"],
            true
        );
        assert_eq!(
            definition.input_schema["properties"]["references"]["items"]["type"],
            "string"
        );
        assert_eq!(
            definition.input_schema["properties"]["library_ids"]["items"]["type"],
            "string"
        );
    }

    #[tokio::test]
    async fn apply_without_a_reviewed_revision_is_rejected_before_ipc() {
        let temp = tempfile::tempdir().unwrap();
        let board = temp.path().join("board.kicad_pcb");
        std::fs::write(&board, "(kicad_pcb (version 20240108))").unwrap();

        let result = handle_update_footprints_from_library(
            &serde_json::json!({ "board": board, "dry_run": false }),
            &test_context(""),
        )
        .await
        .unwrap();

        assert!(result.is_error);
        assert_eq!(
            crate::mcp::error::extract_error_kind(&result).as_deref(),
            Some("invalid_argument")
        );
        let text = match result.content.first().unwrap() {
            crate::mcp::protocol::ToolContent::Text { text } => text,
            _ => unreachable!(),
        };
        assert!(text.contains("expected_plan_revision"));
    }

    #[tokio::test]
    async fn unreachable_ipc_returns_a_non_mutating_conflict_response() {
        let temp = tempfile::tempdir().unwrap();
        let board = temp.path().join("board.kicad_pcb");
        let before = "(kicad_pcb (version 20240108))";
        std::fs::write(&board, before).unwrap();

        let result = handle_update_footprints_from_library(
            &serde_json::json!({ "board": board }),
            &test_context(""),
        )
        .await
        .unwrap();

        assert!(!result.is_error);
        let response = result_json(&result);
        assert_eq!(response["status"], "conflict");
        assert_eq!(response["coverage"]["transport"], "live_kicad_ipc");
        assert!(response["diagnostics"][0]["message"]
            .as_str()
            .unwrap()
            .contains("live-IPC-only"));
        assert_eq!(std::fs::read_to_string(board).unwrap(), before);
    }

    #[derive(Default)]
    struct MutationCapture {
        begin_count: usize,
        update_batches: Vec<usize>,
        commit_actions: Vec<kiapi::common::commands::CommitAction>,
    }

    struct PlannerMock {
        url: String,
        capture: std::sync::Arc<std::sync::Mutex<MutationCapture>>,
        _thread: std::thread::JoinHandle<()>,
    }

    fn api_ok() -> kiapi::common::ApiResponse {
        kiapi::common::ApiResponse {
            status: Some(kiapi::common::ApiResponseStatus {
                status: kiapi::common::ApiStatusCode::AsOk as i32,
                error_message: String::new(),
            }),
            header: None,
            message: None,
        }
    }

    fn api_reply(message: prost_types::Any) -> kiapi::common::ApiResponse {
        kiapi::common::ApiResponse {
            message: Some(message),
            ..api_ok()
        }
    }

    fn spawn_planner_mock(
        board: &Path,
        footprint: prost_types::Any,
        fail_update: bool,
    ) -> PlannerMock {
        use nng::options::Options;

        static NEXT_MOCK: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let url = format!(
            "inproc://footprint-update-{}",
            NEXT_MOCK.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
        );
        let socket = nng::Socket::new(nng::Protocol::Rep0).unwrap();
        socket
            .set_opt::<nng::options::RecvTimeout>(Some(std::time::Duration::from_secs(20)))
            .unwrap();
        socket.listen(&url).unwrap();

        let board_name = board.to_string_lossy().to_string();
        let capture = std::sync::Arc::new(std::sync::Mutex::new(MutationCapture::default()));
        let captured = capture.clone();
        let thread = std::thread::spawn(move || {
            while let Ok(message) = socket.recv() {
                let request = kiapi::common::ApiRequest::decode(message.as_slice()).unwrap();
                let message = request.message.expect("request message");
                let response = if message.type_url.ends_with("GetOpenDocuments") {
                    api_reply(builders::pack_any(
                        &kiapi::common::commands::GetOpenDocumentsResponse {
                            documents: vec![kiapi::common::types::DocumentSpecifier {
                                r#type: kiapi::common::types::DocumentType::DoctypePcb as i32,
                                identifier: Some(
                                    kiapi::common::types::document_specifier::Identifier::BoardFilename(
                                        board_name.clone(),
                                    ),
                                ),
                                project: None,
                            }],
                        },
                        "kiapi.common.commands.GetOpenDocumentsResponse",
                    ))
                } else if message.type_url.ends_with("GetItems") {
                    let command =
                        kiapi::common::commands::GetItems::decode(message.value.as_slice())
                            .unwrap();
                    let items = if command.types
                        == vec![kiapi::common::types::KiCadObjectType::KotPcbFootprint as i32]
                    {
                        vec![footprint.clone()]
                    } else {
                        Vec::new()
                    };
                    api_reply(builders::pack_any(
                        &kiapi::common::commands::GetItemsResponse {
                            header: None,
                            status: kiapi::common::types::ItemRequestStatus::IrsOk as i32,
                            items,
                        },
                        "kiapi.common.commands.GetItemsResponse",
                    ))
                } else if message.type_url.ends_with("GetNets") {
                    api_reply(builders::pack_any(
                        &kiapi::board::commands::NetsResponse {
                            nets: vec![builders::net("ROW1", 11), builders::net("COL1", 12)],
                        },
                        "kiapi.board.commands.NetsResponse",
                    ))
                } else if message.type_url.ends_with("BeginCommit") {
                    captured.lock().unwrap().begin_count += 1;
                    api_reply(builders::pack_any(
                        &kiapi::common::commands::BeginCommitResponse {
                            id: Some(kiapi::common::types::Kiid {
                                value: "commit-1".to_string(),
                            }),
                        },
                        "kiapi.common.commands.BeginCommitResponse",
                    ))
                } else if message.type_url.ends_with("UpdateItems") {
                    let update =
                        kiapi::common::commands::UpdateItems::decode(message.value.as_slice())
                            .unwrap();
                    captured
                        .lock()
                        .unwrap()
                        .update_batches
                        .push(update.items.len());
                    let updated_items = update
                        .items
                        .into_iter()
                        .map(|item| kiapi::common::commands::ItemUpdateResult {
                            status: Some(kiapi::common::commands::ItemStatus {
                                code: if fail_update {
                                    kiapi::common::commands::ItemStatusCode::IscInvalidData as i32
                                } else {
                                    kiapi::common::commands::ItemStatusCode::IscOk as i32
                                },
                                error_message: if fail_update {
                                    "mock update failure".to_string()
                                } else {
                                    String::new()
                                },
                            }),
                            item: Some(item),
                        })
                        .collect();
                    api_reply(builders::pack_any(
                        &kiapi::common::commands::UpdateItemsResponse {
                            header: None,
                            status: kiapi::common::types::ItemRequestStatus::IrsOk as i32,
                            updated_items,
                        },
                        "kiapi.common.commands.UpdateItemsResponse",
                    ))
                } else if message.type_url.ends_with("EndCommit") {
                    let end = kiapi::common::commands::EndCommit::decode(message.value.as_slice())
                        .unwrap();
                    captured.lock().unwrap().commit_actions.push(end.action());
                    api_reply(builders::pack_any(
                        &kiapi::common::commands::EndCommitResponse {},
                        "kiapi.common.commands.EndCommitResponse",
                    ))
                } else {
                    panic!("unexpected mock request {}", message.type_url);
                };
                let response = nng::Message::from(response.encode_to_vec().as_slice());
                if socket.send(response).is_err() {
                    break;
                }
            }
        });
        PlannerMock {
            url,
            capture,
            _thread: thread,
        }
    }

    #[tokio::test]
    async fn dry_run_and_stale_revision_never_open_a_commit_or_send_updates() {
        let (_temp, board, items) = plan_fixture();
        let mock = spawn_planner_mock(&board, items[2].clone(), false);
        let context = test_context(&mock.url);

        let dry_run =
            handle_update_footprints_from_library(&serde_json::json!({ "board": board }), &context)
                .await
                .unwrap();
        assert_eq!(result_json(&dry_run)["status"], "ready");

        let stale = handle_update_footprints_from_library(
            &serde_json::json!({
                "board": board,
                "dry_run": false,
                "expected_plan_revision": "stale"
            }),
            &context,
        )
        .await
        .unwrap();
        let stale = result_json(&stale);
        assert_eq!(stale["status"], "conflict");
        assert_eq!(stale["diagnostics"][0]["code"], "stale_plan_revision");

        let capture = mock.capture.lock().unwrap();
        assert_eq!(capture.begin_count, 0);
        assert!(capture.update_batches.is_empty());
        assert!(capture.commit_actions.is_empty());
    }

    #[tokio::test]
    async fn apply_sends_one_batch_in_one_commit() {
        let (_temp, board, items) = plan_fixture();
        let mock = spawn_planner_mock(&board, items[2].clone(), false);
        let context = test_context(&mock.url);
        let dry_run =
            handle_update_footprints_from_library(&serde_json::json!({ "board": board }), &context)
                .await
                .unwrap();
        let revision = result_json(&dry_run)["plan_revision"]
            .as_str()
            .unwrap()
            .to_string();

        let applied = handle_update_footprints_from_library(
            &serde_json::json!({
                "board": board,
                "dry_run": false,
                "expected_plan_revision": revision
            }),
            &context,
        )
        .await
        .unwrap();

        let applied = result_json(&applied);
        assert_eq!(applied["status"], "applied");
        assert_eq!(applied["coverage"]["changed"]["applied"], 1);
        let capture = mock.capture.lock().unwrap();
        assert_eq!(capture.begin_count, 1);
        assert_eq!(capture.update_batches, vec![1]);
        assert_eq!(
            capture.commit_actions,
            vec![kiapi::common::commands::CommitAction::CmaCommit]
        );
    }

    #[tokio::test]
    async fn failed_update_drops_the_single_commit() {
        let (_temp, board, items) = plan_fixture();
        let mock = spawn_planner_mock(&board, items[2].clone(), true);
        let context = test_context(&mock.url);
        let dry_run =
            handle_update_footprints_from_library(&serde_json::json!({ "board": board }), &context)
                .await
                .unwrap();
        let revision = result_json(&dry_run)["plan_revision"]
            .as_str()
            .unwrap()
            .to_string();

        let failed = handle_update_footprints_from_library(
            &serde_json::json!({
                "board": board,
                "dry_run": false,
                "expected_plan_revision": revision
            }),
            &context,
        )
        .await
        .unwrap();

        assert_eq!(result_json(&failed)["status"], "conflict");
        let capture = mock.capture.lock().unwrap();
        assert_eq!(capture.begin_count, 1);
        assert_eq!(capture.update_batches, vec![1]);
        assert_eq!(
            capture.commit_actions,
            vec![kiapi::common::commands::CommitAction::CmaDrop]
        );
    }

    // ─── Custom pads and zone connections through the typed refresh ─────────

    const TEXAS_CUSTOM_PADS: &str =
        include_str!("../../tests/fixtures/custom_pads_texas_rje0020a_kicad10.kicad_mod");
    use crate::test_support::without_heatsink_property as carried;

    fn pad_by_number<'a>(
        pads: &'a [kiapi::board::types::Pad],
        number: &str,
    ) -> &'a kiapi::board::types::Pad {
        pads.iter()
            .find(|pad| pad.number == number)
            .unwrap_or_else(|| panic!("pad '{number}'"))
    }

    /// The stock custom-pad footprint (less its unrepresentable heatsink
    /// property) refreshes: custom anchors and pad-local primitives as the
    /// shared parser reads them, the exposed pad's `(zone_connect 2)` in the
    /// pad stack, nets kept, and rebuilding the result changes nothing, even
    /// after KiCad spells out zone settings and drops a polygon's closing
    /// vertex.
    #[test]
    fn refresh_rebuilds_supported_custom_pads_and_zone_connect_losslessly() {
        use kiapi::board::types::{PadStackShape, ZoneConnectionStyle};
        let source = carried(TEXAS_CUSTOM_PADS);
        let library = parse_library_footprint("Test:Socket", &source).unwrap();
        let net_codes = BTreeMap::from([("ROW1".to_string(), 11), ("COL1".to_string(), 12)]);
        let current = current_instance(kiapi::board::types::BoardLayer::BlFCu);

        let first =
            build_updated_instance(&current, &library, &net_codes, &BTreeSet::new()).unwrap();
        assert!(first.changed_domains.contains(&ChangedDomain::Pads));
        let applied =
            kiapi::board::types::FootprintInstance::decode(first.item.value.as_slice()).unwrap();
        let pads = decoded_pads(&applied);
        assert_eq!(pads.len(), library.pads.len());

        for want in &library.pads {
            let pad = pad_by_number(&pads, &want.number);
            if want.number.is_empty() {
                continue;
            }
            let stack = pad.pad_stack.as_ref().unwrap();
            let copper = &stack.copper_layers[0];
            match &want.custom {
                Some(custom) => {
                    assert_eq!(copper.shape, PadStackShape::PssCustom as i32);
                    assert_eq!(copper.custom_anchor_shape, PadStackShape::PssCircle as i32);
                    let primitives = copper
                        .custom_shapes
                        .iter()
                        .map(|shape| builders::pad_primitive_from_proto(shape).unwrap())
                        .collect::<Vec<_>>();
                    assert_eq!(primitives, custom.primitives, "pad {}", want.number);
                }
                None => assert!(copper.custom_shapes.is_empty()),
            }
            assert_eq!(
                stack
                    .zone_settings
                    .as_ref()
                    .map(|settings| settings.zone_connection),
                Some(if want.number == "21" {
                    ZoneConnectionStyle::ZcsFull
                } else {
                    ZoneConnectionStyle::ZcsInherited
                } as i32),
                "pad {}",
                want.number
            );
        }
        assert_eq!(pad_by_number(&pads, "1").net.as_ref().unwrap().name, "ROW1");
        assert_eq!(pad_by_number(&pads, "2").net.as_ref().unwrap().name, "COL1");

        let second =
            build_updated_instance(&applied, &library, &net_codes, &BTreeSet::new()).unwrap();
        assert!(
            second.changed_domains.is_empty(),
            "{:?}",
            second.changed_domains
        );

        // As KiCad reports it back: every pad's zone settings and spoke angle
        // spelled out, a fresh KIID and empty net on each primitive, and each
        // custom polygon closed explicitly.
        let mut reported = applied.clone();
        for item in &mut reported.definition.as_mut().unwrap().items {
            if !item.type_url.ends_with("kiapi.board.types.Pad") {
                continue;
            }
            let mut pad = kiapi::board::types::Pad::decode(item.value.as_slice()).unwrap();
            let stack = pad.pad_stack.as_mut().unwrap();
            stack.zone_settings = Some(builders::canonical_pad_zone_settings(stack));
            for shape in &mut stack.copper_layers[0].custom_shapes {
                shape.id = Some(kiapi::common::types::Kiid {
                    value: "kicad-primitive".to_string(),
                });
                shape.net = Some(kiapi::board::types::Net::default());
                shape.locked = kiapi::common::types::LockedState::LsUnlocked as i32;
                use kiapi::common::types::graphic_shape::Geometry;
                if let Some(Geometry::Polygon(set)) =
                    shape.shape.as_mut().and_then(|s| s.geometry.as_mut())
                {
                    let outline = set.polygons[0].outline.as_mut().unwrap();
                    let first = outline.nodes[0];
                    outline.nodes.push(first);
                }
            }
            *item = builders::pack_any(&pad, "kiapi.board.types.Pad");
        }
        assert!(changed_domains(&reported, &applied).unwrap().is_empty());

        // A lost zone connection is still a pad change.
        let mut lost = applied.clone();
        for item in &mut lost.definition.as_mut().unwrap().items {
            if item.type_url.ends_with("kiapi.board.types.Pad") {
                let mut pad = kiapi::board::types::Pad::decode(item.value.as_slice()).unwrap();
                pad.pad_stack.as_mut().unwrap().zone_settings = None;
                *item = builders::pack_any(&pad, "kiapi.board.types.Pad");
            }
        }
        assert!(changed_domains(&lost, &applied)
            .unwrap()
            .contains(&ChangedDomain::Pads));

        // The source was only read.
        assert_eq!(source, carried(TEXAS_CUSTOM_PADS));
    }

    /// Mirroring a custom pad's primitives onto the back is not implemented,
    /// so a back-side instance is refused rather than rebuilt unmirrored.
    #[test]
    fn refresh_still_refuses_a_back_side_custom_pad() {
        let library = parse_library_footprint("Test:Socket", &carried(TEXAS_CUSTOM_PADS)).unwrap();
        let current = current_instance(kiapi::board::types::BoardLayer::BlBCu);
        let error = build_updated_instance(&current, &library, &BTreeMap::new(), &BTreeSet::new())
            .unwrap_err();
        assert!(
            format!("{error:#}").contains("cannot be mirrored onto the back side"),
            "{error:#}"
        );
    }

    /// The validator accepts the custom and zone-connect clauses and still
    /// names what the typed refresh cannot carry.
    #[test]
    fn refresh_validation_accepts_custom_and_zone_connect_and_names_the_rest() {
        let carried_source = carried(TEXAS_CUSTOM_PADS);
        assert!(parse_library_footprint("Test:Socket", &carried_source).is_ok());
        for (from, to, named) in [
            (
                "(clearance outline)",
                "(clearance convexhull)",
                "convexhull",
            ),
            ("(zone_connect 2)", "(zone_connect 7)", "zone connection"),
            (
                "(zone_connect 2)",
                "(zone_connect 2) (thermal_gap 0.3) (thermal_gap 0.3)",
                "thermal_gap",
            ),
            (
                "(zone_connect 2)",
                "(zone_connect 2) (thermal_bridge_width -0.1)",
                "negative",
            ),
            (
                "(xy 0.253358 -0.09)",
                "(arc (start 0.253358 -0.09) (mid 0.3 0) (end 0.4275 0.084142))",
                "arc",
            ),
        ] {
            assert!(carried_source.contains(from), "{from}");
            let unsupported = carried_source.replacen(from, to, 1);
            let error = parse_library_footprint("Test:Socket", &unsupported).unwrap_err();
            assert!(format!("{error:#}").contains(named), "{to}: {error:#}");
        }
        let error = parse_library_footprint("Test:Socket", TEXAS_CUSTOM_PADS).unwrap_err();
        assert!(format!("{error:#}").contains("property"), "{error:#}");
    }

    /// All or nothing: a refresh selecting a supported custom-pad footprint
    /// and one whose custom pad cannot be carried prepares neither.
    #[test]
    fn planner_refuses_every_update_when_one_custom_pad_footprint_is_unsupported() {
        let temp = tempfile::tempdir().unwrap();
        let board = temp.path().join("board.kicad_pcb");
        std::fs::write(&board, "(kicad_pcb (version 20240108))").unwrap();
        let library_dir = temp.path().join("Test.pretty");
        std::fs::create_dir(&library_dir).unwrap();
        let supported = carried(TEXAS_CUSTOM_PADS);
        std::fs::write(library_dir.join("Socket.kicad_mod"), &supported).unwrap();
        std::fs::write(
            library_dir.join("Hull.kicad_mod"),
            supported.replace("(clearance outline)", "(clearance convexhull)"),
        )
        .unwrap();
        std::fs::write(
            temp.path().join("fp-lib-table"),
            "(fp_lib_table (lib (name \"Test\") (type \"KiCad\") (uri \"${KIPRJMOD}/Test.pretty\") (options \"\") (descr \"\")))",
        )
        .unwrap();
        let net_codes = BTreeMap::from([("ROW1".to_string(), 11), ("COL1".to_string(), 12)]);

        let both = vec![
            plan_item("SW1", Some("Test:Socket"), "kiid-1"),
            plan_item("SW2", Some("Test:Hull"), "kiid-2"),
        ];
        let plan = plan_updates(
            &board,
            &both,
            &net_codes,
            &BTreeSet::new(),
            &UpdateFilters::default(),
        );
        assert_eq!(plan.status, PlanStatus::Conflict);
        assert!(plan.prepared_items.is_empty());
        assert!(plan.changes.is_empty());
        assert!(plan.diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "unsupported_library_footprint"
                && diagnostic.message.contains("convexhull")
        }));

        let alone = vec![plan_item("SW1", Some("Test:Socket"), "kiid-1")];
        let plan = plan_updates(
            &board,
            &alone,
            &net_codes,
            &BTreeSet::new(),
            &UpdateFilters::default(),
        );
        assert_eq!(plan.status, PlanStatus::Ready, "{:?}", plan.diagnostics);
        assert_eq!(plan.prepared_items.len(), 1);
    }

    // ─── Pad diagnostics in the dry run ─────────────────────────────────────

    /// The Texas fixture with its first custom polygon closed explicitly (the
    /// first point repeated last), as many KiCad-written footprints are.
    fn texas_with_closing_vertex() -> String {
        let source = carried(TEXAS_CUSTOM_PADS).replace("\r\n", "\n");
        let closed = source.replacen(
            "(xy -0.4275 0.09)\n\t\t\t\t)",
            "(xy -0.4275 0.09)\n\t\t\t\t\t(xy -0.4275 -0.09)\n\t\t\t\t)",
            1,
        );
        assert_ne!(closed, source, "the fixture's first polygon moved");
        closed
    }

    /// `applied` as KiCad 10 reports it back: zone settings and spoke angle
    /// spelled out, a fresh KIID, empty net and lock state on every
    /// primitive, the solid stroke style reported by the native editor, and
    /// each explicit closing vertex dropped (`UnpackPolyLine` closes the chain).
    fn as_kicad_reports(
        applied: &kiapi::board::types::FootprintInstance,
    ) -> kiapi::board::types::FootprintInstance {
        use kiapi::common::types::graphic_shape::Geometry;
        let mut reported = applied.clone();
        for item in &mut reported.definition.as_mut().unwrap().items {
            if !item.type_url.ends_with("kiapi.board.types.Pad") {
                continue;
            }
            let mut pad = kiapi::board::types::Pad::decode(item.value.as_slice()).unwrap();
            pad.id = Some(kiapi::common::types::Kiid {
                value: format!("kicad-pad-{}", pad.number),
            });
            let stack = pad.pad_stack.as_mut().unwrap();
            stack.zone_settings = Some(builders::canonical_pad_zone_settings(stack));
            for shape in &mut stack.copper_layers[0].custom_shapes {
                shape.id = Some(kiapi::common::types::Kiid {
                    value: "kicad-primitive".to_string(),
                });
                shape.net = Some(kiapi::board::types::Net::default());
                shape.locked = kiapi::common::types::LockedState::LsUnlocked as i32;
                if let Some(stroke) = shape
                    .shape
                    .as_mut()
                    .and_then(|graphic| graphic.attributes.as_mut())
                    .and_then(|attributes| attributes.stroke.as_mut())
                {
                    if stroke.style == kiapi::common::types::StrokeLineStyle::SlsUnknown as i32 {
                        stroke.style = kiapi::common::types::StrokeLineStyle::SlsSolid as i32;
                    }
                }
                if let Some(Geometry::Polygon(set)) =
                    shape.shape.as_mut().and_then(|s| s.geometry.as_mut())
                {
                    for outline in set.polygons.iter_mut().filter_map(|p| p.outline.as_mut()) {
                        if outline.nodes.len() > 3 && outline.nodes.first() == outline.nodes.last()
                        {
                            outline.nodes.pop();
                        }
                    }
                }
            }
            *item = builders::pack_any(&pad, "kiapi.board.types.Pad");
        }
        reported
    }

    /// `instance` with `edit` applied to pad `number`.
    fn edit_pad(
        instance: &mut kiapi::board::types::FootprintInstance,
        number: &str,
        edit: impl Fn(&mut kiapi::board::types::Pad),
    ) {
        for item in &mut instance.definition.as_mut().unwrap().items {
            if item.type_url.ends_with("kiapi.board.types.Pad") {
                let mut pad = kiapi::board::types::Pad::decode(item.value.as_slice()).unwrap();
                if pad.number == number {
                    edit(&mut pad);
                    *item = builders::pack_any(&pad, "kiapi.board.types.Pad");
                }
            }
        }
    }

    fn first_outline(pad: &mut kiapi::board::types::Pad) -> &mut kiapi::common::types::PolyLine {
        use kiapi::common::types::graphic_shape::Geometry;
        let shape = &mut pad.pad_stack.as_mut().unwrap().copper_layers[0].custom_shapes[0];
        match shape.shape.as_mut().and_then(|s| s.geometry.as_mut()) {
            Some(Geometry::Polygon(set)) => set.polygons[0].outline.as_mut().unwrap(),
            other => panic!("pad's first primitive is not a polygon: {other:?}"),
        }
    }

    /// The library pad whose first polygon `texas_with_closing_vertex`
    /// closed explicitly.
    fn explicitly_closed_pad(library: &LibraryFootprint) -> String {
        library
            .pads
            .iter()
            .find(|pad| {
                pad.custom
                    .as_ref()
                    .is_some_and(|custom| match &custom.primitives[0] {
                        konnect_ipc::IpcPadPrimitive::Polygon { points, .. } => {
                            points.len() > 3 && points.first() == points.last()
                        }
                        _ => false,
                    })
            })
            .expect("one explicitly closed polygon")
            .number
            .clone()
    }

    /// A fresh create through the API drops an explicit closing vertex and
    /// gives every primitive its own KIID. Neither is a planned change, so
    /// the refresh stays a noop, but the dropped vertex is still reported,
    /// with both counts and why it can matter natively. KIIDs are not.
    #[test]
    fn dry_run_keeps_a_dropped_closing_vertex_harmless_and_reports_it() {
        let library = parse_library_footprint("Test:Socket", &texas_with_closing_vertex()).unwrap();
        let closed = explicitly_closed_pad(&library);
        let net_codes = BTreeMap::from([("ROW1".to_string(), 11), ("COL1".to_string(), 12)]);
        let first = build_updated_instance(
            &current_instance(kiapi::board::types::BoardLayer::BlFCu),
            &library,
            &net_codes,
            &BTreeSet::new(),
        )
        .unwrap();
        let applied =
            kiapi::board::types::FootprintInstance::decode(first.item.value.as_slice()).unwrap();
        let reported = as_kicad_reports(&applied);

        let second =
            build_updated_instance(&reported, &library, &net_codes, &BTreeSet::new()).unwrap();
        assert!(
            second.changed_domains.is_empty(),
            "{:?}",
            second.changed_domains
        );
        assert_eq!(
            second.pad_differences,
            vec![PadDifference {
                pad: closed,
                field:
                    "pad_stack.copper_layers[0].custom_shapes[0].polygon[0].outline.vertex_count"
                        .to_string(),
                expected: json!(6),
                observed: json!(5),
                ignored_by_plan: Some(CLOSING_VERTEX_NOTE.to_string()),
            }]
        );
        assert_eq!(second.pad_differences_omitted, 0);
    }

    /// A native-like change is named by pad and typed field with the library
    /// value and the board value: a spoke angle KiCad holds as 0° (what it
    /// makes of spokes sent without an angle) and a moved polygon vertex.
    /// The harmless closing vertex beside them neither hides them nor is
    /// counted, and a polygon reports its first differing vertex, not every
    /// coordinate.
    #[test]
    fn dry_run_names_each_changed_pad_field_with_library_and_board_values() {
        let library = parse_library_footprint("Test:Socket", &texas_with_closing_vertex()).unwrap();
        let closed = explicitly_closed_pad(&library);
        let net_codes = BTreeMap::from([("ROW1".to_string(), 11), ("COL1".to_string(), 12)]);
        let first = build_updated_instance(
            &current_instance(kiapi::board::types::BoardLayer::BlFCu),
            &library,
            &net_codes,
            &BTreeSet::new(),
        )
        .unwrap();
        let applied =
            kiapi::board::types::FootprintInstance::decode(first.item.value.as_slice()).unwrap();
        let library_angle = decoded_pads(&applied)
            .iter()
            .find(|pad| pad.number == "21")
            .and_then(|pad| {
                pad.pad_stack
                    .as_ref()?
                    .zone_settings
                    .as_ref()?
                    .thermal_spokes?
                    .angle
            })
            .map(|angle| angle.value_degrees)
            .unwrap();
        let mut reported = as_kicad_reports(&applied);
        edit_pad(&mut reported, "21", |pad| {
            let settings = pad
                .pad_stack
                .as_mut()
                .unwrap()
                .zone_settings
                .as_mut()
                .unwrap();
            settings.thermal_spokes.as_mut().unwrap().angle =
                Some(kiapi::common::types::Angle { value_degrees: 0.0 });
        });
        let moved = library
            .pads
            .iter()
            .find(|pad| pad.custom.is_some() && pad.number != closed)
            .unwrap()
            .number
            .clone();
        edit_pad(&mut reported, &moved, |pad| {
            use kiapi::common::types::poly_line_node::Geometry;
            let outline = first_outline(pad);
            for node in &mut outline.nodes {
                if let Some(Geometry::Point(point)) = node.geometry.as_mut() {
                    point.x_nm += 10_000;
                }
            }
        });

        let second =
            build_updated_instance(&reported, &library, &net_codes, &BTreeSet::new()).unwrap();
        assert!(second.changed_domains.contains(&ChangedDomain::Pads));
        let find = |pad: &str, field: &str| {
            second
                .pad_differences
                .iter()
                .find(|difference| difference.pad == pad && difference.field.starts_with(field))
                .unwrap_or_else(|| panic!("{pad} {field}: {:#?}", second.pad_differences))
        };

        let angle = find("21", "pad_stack.zone_settings.thermal_spokes.angle_degrees");
        assert_eq!(angle.expected, json!(library_angle));
        assert_eq!(angle.observed, json!(0.0));
        assert_eq!(angle.ignored_by_plan, None);

        let library_vertex = decoded_pads(&applied)
            .into_iter()
            .find(|pad| pad.number == moved)
            .map(|mut pad| first_outline(&mut pad).nodes[0])
            .unwrap();
        let Some(kiapi::common::types::poly_line_node::Geometry::Point(point)) =
            library_vertex.geometry
        else {
            panic!("a point vertex");
        };
        let vertex = find(
            &moved,
            "pad_stack.copper_layers[0].custom_shapes[0].polygon[0].outline.vertex[0]",
        );
        assert!(
            vertex.field.ends_with("(first of 5 differing vertices)"),
            "{}",
            vertex.field
        );
        assert_eq!(
            vertex.expected,
            json!({ "x_nm": point.x_nm, "y_nm": point.y_nm })
        );
        assert_eq!(
            vertex.observed,
            json!({ "x_nm": point.x_nm + 10_000, "y_nm": point.y_nm })
        );
        assert_eq!(
            second
                .pad_differences
                .iter()
                .filter(|difference| difference.pad == moved)
                .count(),
            1,
            "one entry per changed polygon: {:#?}",
            second.pad_differences
        );

        let closing = find(&closed, "pad_stack.copper_layers[0].custom_shapes[0]");
        assert_eq!(
            closing.ignored_by_plan.as_deref(),
            Some(CLOSING_VERTEX_NOTE)
        );
        assert!(second
            .pad_differences
            .iter()
            .all(|difference| difference.field != "unclassified"));

        // The response carries the differences on the planned change.
        let planned = serde_json::to_value(PlannedUpdate {
            reference: "SW1".to_string(),
            library_id: "Test:Socket".to_string(),
            changed_domains: second.changed_domains.clone(),
            preserved: second.preserved.clone(),
            pad_differences: second.pad_differences.clone(),
            pad_differences_omitted: second.pad_differences_omitted,
        })
        .unwrap();
        assert_eq!(planned["pad_differences"].as_array().unwrap().len(), 3);
        assert!(planned.get("pad_differences_omitted").is_none());
    }

    /// A library pad's explicit thermal override is compared with what the
    /// board holds, an explicit zero against an unset width included.
    #[test]
    fn dry_run_compares_explicit_thermal_overrides() {
        let base = carried(TEXAS_CUSTOM_PADS);
        let library = parse_library_footprint("Test:Socket", &base).unwrap();
        let overridden = base.replacen(
            "(zone_connect 2)",
            "(zone_connect 2) (thermal_bridge_width 0) (thermal_gap 0.3) (thermal_bridge_angle 30)",
            1,
        );
        assert_ne!(overridden, base);
        let with_override = parse_library_footprint("Test:Socket", &overridden).unwrap();
        let net_codes = BTreeMap::from([("ROW1".to_string(), 11), ("COL1".to_string(), 12)]);
        let current = current_instance(kiapi::board::types::BoardLayer::BlFCu);
        let plain =
            build_updated_instance(&current, &library, &net_codes, &BTreeSet::new()).unwrap();
        let board = as_kicad_reports(
            &kiapi::board::types::FootprintInstance::decode(plain.item.value.as_slice()).unwrap(),
        );

        let refresh =
            build_updated_instance(&board, &with_override, &net_codes, &BTreeSet::new()).unwrap();
        assert!(refresh.changed_domains.contains(&ChangedDomain::Pads));
        let fields = refresh
            .pad_differences
            .iter()
            .map(|d| {
                (
                    d.pad.as_str(),
                    d.field.as_str(),
                    d.expected.clone(),
                    d.observed.clone(),
                )
            })
            .collect::<Vec<_>>();
        assert!(fields.contains(&(
            "21",
            "pad_stack.zone_settings.thermal_spokes.width_nm",
            json!(0),
            serde_json::Value::Null
        )));
        assert!(fields.contains(&(
            "21",
            "pad_stack.zone_settings.thermal_spokes.gap_nm",
            json!(300_000),
            serde_json::Value::Null
        )));
        assert!(fields.iter().any(|(pad, field, expected, _)| *pad == "21"
            && *field == "pad_stack.zone_settings.thermal_spokes.angle_degrees"
            && *expected == json!(30.0)));

        // Once the board holds the overrides, the refresh is a noop.
        let applied =
            kiapi::board::types::FootprintInstance::decode(refresh.item.value.as_slice()).unwrap();
        let again = build_updated_instance(
            &as_kicad_reports(&applied),
            &with_override,
            &net_codes,
            &BTreeSet::new(),
        )
        .unwrap();
        assert!(
            again.changed_domains.is_empty(),
            "{:?}",
            again.pad_differences
        );
    }

    /// The planner keeps its all-or-nothing response shape: a footprint whose
    /// only pad difference is the harmless closing vertex is unchanged (noop)
    /// and the difference is reported as a parity note, not a conflict.
    #[test]
    fn planner_reports_ignored_pad_differences_as_notes_on_a_noop() {
        let temp = tempfile::tempdir().unwrap();
        let board = temp.path().join("board.kicad_pcb");
        std::fs::write(&board, "(kicad_pcb (version 20240108))").unwrap();
        let library_dir = temp.path().join("Test.pretty");
        std::fs::create_dir(&library_dir).unwrap();
        let source = texas_with_closing_vertex();
        std::fs::write(library_dir.join("Socket.kicad_mod"), &source).unwrap();
        std::fs::write(
            temp.path().join("fp-lib-table"),
            "(fp_lib_table (lib (name \"Test\") (type \"KiCad\") (uri \"${KIPRJMOD}/Test.pretty\") (options \"\") (descr \"\")))",
        )
        .unwrap();
        let net_codes = BTreeMap::from([("ROW1".to_string(), 11), ("COL1".to_string(), 12)]);
        let library = parse_library_footprint("Test:Socket", &source).unwrap();
        let mut current = current_instance(kiapi::board::types::BoardLayer::BlFCu);
        current.id = Some(kiapi::common::types::Kiid {
            value: "kiid-1".to_string(),
        });
        let applied =
            build_updated_instance(&current, &library, &net_codes, &BTreeSet::new()).unwrap();
        let reported = as_kicad_reports(
            &kiapi::board::types::FootprintInstance::decode(applied.item.value.as_slice()).unwrap(),
        );
        let items = vec![builders::pack_any(
            &reported,
            "kiapi.board.types.FootprintInstance",
        )];

        let plan = plan_updates(
            &board,
            &items,
            &net_codes,
            &BTreeSet::new(),
            &UpdateFilters::default(),
        );
        assert_eq!(plan.status, PlanStatus::Noop, "{:?}", plan.diagnostics);
        assert!(plan.diagnostics.is_empty());
        assert_eq!(plan.coverage.conflicts.planned, 0);
        assert_eq!(plan.pad_parity_notes.len(), 1);
        assert_eq!(plan.pad_parity_notes[0].reference, "SW1");
        assert!(plan.pad_parity_notes[0]
            .pad_differences
            .iter()
            .all(|difference| difference.ignored_by_plan.is_some()));
        let response = plan_response(&plan, false);
        let text = serde_json::to_string(&response).unwrap();
        assert!(text.contains("pad_parity_notes") && text.contains("vertex_count"));
    }
}
