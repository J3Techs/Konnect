//! Revision-aware edits to 3D models on placed footprints (#305): live
//! through KiCad when it holds the board, otherwise in the saved file once the
//! board is proven closed (`pcb_components::saved_models`).

use crate::mcp::error::ToolErrorKind;
use crate::mcp::protocol::CallToolResult;
use crate::tool;
use crate::tools::closed_board;
use crate::tools::pcb_components::saved_models::{self, SavedModelError};
use crate::tools::{
    get_path, invalid_arg, ipc_target_error_result, require_str, with_bound_board_ipc_classified,
    BoardAccess, BoardBinding, ToolContext, ToolDef,
};
use anyhow::Error;
use konnect_ipc::client::FootprintTargetError;
use konnect_ipc::{
    IpcFootprint3DModel, IpcFootprint3DModelEdit, IpcFootprint3DModelEditOutcome,
    IpcFootprint3DModelSnapshot, IpcVector3,
};
use serde_json::{json, Value};
use sha2::{Digest, Sha256};

const OPERATION: &str = "set_placed_footprint_models";

pub(crate) fn tool() -> ToolDef {
    tool!(
        "set_placed_footprint_models",
        "Inspect, append, replace, or remove one exact 3D-model entry on a placed footprint. \
         On a board open in KiCad it uses one native undo transaction and a fresh live \
         readback (source ipc); it never rewrites an open board file. On a board proven closed \
         in KiCad it edits only that footprint's (model ...) blocks in the saved file with one \
         atomic write and a readback (source saved), keeping every pad clause; that is the \
         path for a footprint whose pads carry fabrication properties, which the live path \
         refuses. Inspect first, then pass its exact models_revision for every mutation.",
        json!({
            "type": "object",
            "properties": {
                "board": { "type": "string" },
                "reference": { "type": "string", "description": "Exact reference designator, e.g. U1" },
                "mode": { "type": "string", "enum": ["inspect", "append", "replace", "remove"] },
                "model_index": { "type": "integer", "minimum": 0, "description": "Exact zero-based entry index; required for replace/remove" },
                "expected_models_revision": { "type": "string", "description": "Exact revision from inspect; required for append/replace/remove" },
                "model": {
                    "type": "object",
                    "description": "Complete replacement model; required for append/replace",
                    "properties": {
                        "path": { "type": "string" },
                        "offset_mm": { "$ref": "#/$defs/vector3Zero" },
                        "rotation_degrees": { "$ref": "#/$defs/vector3Zero" },
                        "scale": { "$ref": "#/$defs/vector3One" },
                        "visible": { "type": "boolean", "default": true },
                        "opacity": { "type": "number", "minimum": 0.0, "maximum": 1.0, "default": 1.0 }
                    },
                    "required": ["path"]
                }
            },
            "required": ["board", "reference", "mode"],
            "$defs": {
                "vector3Zero": {
                    "type": "object",
                    "properties": {
                        "x": { "type": "number", "default": 0.0 },
                        "y": { "type": "number", "default": 0.0 },
                        "z": { "type": "number", "default": 0.0 }
                    }
                },
                "vector3One": {
                    "type": "object",
                    "properties": {
                        "x": { "type": "number", "default": 1.0 },
                        "y": { "type": "number", "default": 1.0 },
                        "z": { "type": "number", "default": 1.0 }
                    }
                }
            }
        }),
        |args, ctx| async move { handle_set_placed_footprint_models(args, ctx).await }
    )
    .with_board_access(BoardAccess::LivePreferredWithFallback)
}

enum LiveResult {
    Snapshot(Result<IpcFootprint3DModelSnapshot, Error>),
    Conflict(IpcFootprint3DModelSnapshot),
    Edited(Result<IpcFootprint3DModelEditOutcome, Error>),
    Refused(crate::tools::footprint_fabrication::RebuildRefusal),
}

async fn handle_set_placed_footprint_models(
    args: &Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let board = get_path(args, "board")?;
    let reference = match require_str(args, "reference") {
        Ok(value) => value.to_string(),
        Err(result) => return Ok(result),
    };
    if reference.trim().is_empty() {
        return Ok(invalid_arg("reference", "must not be empty"));
    }
    let mode = match require_str(args, "mode") {
        Ok(value) => value,
        Err(result) => return Ok(result),
    };
    let edit = match parse_edit(args, mode) {
        Ok(edit) => edit,
        Err(result) => return Ok(result),
    };
    let expected_revision = if edit.is_some() {
        match args.get("expected_models_revision").and_then(Value::as_str) {
            Some(value) if !value.is_empty() => Some(value.to_string()),
            _ => {
                return Ok(invalid_arg(
                    "expected_models_revision",
                    "is required for append, replace, and remove; inspect first",
                ))
            }
        }
    } else {
        None
    };
    let closed_edit = edit.clone();
    let closed_revision = expected_revision.clone();
    let live_reference = reference.clone();
    let revision_board = board.clone();
    let live = with_bound_board_ipc_classified(ctx, &board, move |client, document| {
        let snapshot = client.footprint_3d_model_snapshot_in(document.clone(), &live_reference);
        let Some(edit) = edit else {
            return Ok(LiveResult::Snapshot(snapshot));
        };
        let snapshot = match snapshot {
            Ok(snapshot) => snapshot,
            Err(error) => return Ok(LiveResult::Snapshot(Err(error))),
        };
        if expected_revision.as_deref()
            != Some(models_revision(&revision_board, &snapshot).as_str())
        {
            return Ok(LiveResult::Conflict(snapshot));
        }
        // The edit sends the whole footprint back, and KiCad re-creates its
        // pads without any pad fabrication property.
        if let Err(refusal) = crate::tools::footprint_fabrication::guard_live_rebuild_in(
            client,
            document.clone(),
            &revision_board,
            "set_placed_footprint_models",
            &[&live_reference],
            crate::tools::footprint_fabrication::Remedy::ClosedBoardPath,
        )? {
            return Ok(LiveResult::Refused(refusal));
        }
        Ok(LiveResult::Edited(client.edit_footprint_3d_models_in(
            document,
            &live_reference,
            snapshot,
            edit,
        )))
    })
    .await?;

    let live = match live {
        Ok(BoardBinding::Bound(value)) => value,
        Err(konnect_ipc::IpcFailure::Target { error, .. }) => {
            return Ok(ipc_target_error_result(&error))
        }
        // KiCad did not serve this board. The saved file is edited only once
        // the closed-board gate proves no KiCad holds it; every other answer
        // is that gate's refusal.
        Ok(BoardBinding::Unserved { .. }) | Err(_) => {
            return handle_closed(
                ctx,
                &board,
                &reference,
                closed_edit.as_ref(),
                closed_revision.as_deref(),
            )
            .await
        }
    };

    match live {
        LiveResult::Refused(refusal) => Ok(refusal.into_result()),
        LiveResult::Snapshot(Ok(snapshot)) => {
            Ok(snapshot_result("inspected", false, &board, &snapshot))
        }
        LiveResult::Snapshot(Err(error)) | LiveResult::Edited(Err(error)) => {
            Ok(footprint_error(error, &reference))
        }
        LiveResult::Conflict(snapshot) => {
            let actual = models_revision(&board, &snapshot);
            Ok(CallToolResult::error_kind(
                ToolErrorKind::StaleTarget {
                    target: format!("{} footprint {reference}", board.display()),
                    reason: format!("3D-model revision is now {actual}"),
                },
                format!(
                    "The placed footprint's 3D-model list changed after inspection. Re-inspect and retry with revision {actual}."
                ),
            ))
        }
        LiveResult::Edited(Ok(IpcFootprint3DModelEditOutcome::Applied {
            after, changed, ..
        })) => Ok(snapshot_result(
            if changed { "applied" } else { "unchanged" },
            changed,
            &board,
            &after,
        )),
        LiveResult::Edited(Ok(IpcFootprint3DModelEditOutcome::Conflict { observed, .. })) => {
            let actual = models_revision(&board, &observed);
            Ok(CallToolResult::error_kind(
                ToolErrorKind::StaleTarget {
                    target: format!("{} footprint {reference}", board.display()),
                    reason: format!("3D-model list changed immediately before mutation; revision is now {actual}"),
                },
                format!("The model list changed before KiCad could apply the edit. Re-inspect and retry with revision {actual}."),
            ))
        }
        LiveResult::Edited(Ok(IpcFootprint3DModelEditOutcome::Uncertain {
            reason, ..
        })) => Ok(CallToolResult::error_kind(
            ToolErrorKind::MutationOutcomeUncertain {
                operation: "set_placed_footprint_models".to_string(),
                path: board.display().to_string(),
                reason: reason.clone(),
            },
            format!(
                "KiCad may have changed footprint {reference}, but fresh readback did not prove the requested model list. Inspect before retrying. {reason}"
            ),
        )),
    }
}

fn parse_edit(args: &Value, mode: &str) -> Result<Option<IpcFootprint3DModelEdit>, CallToolResult> {
    let index = || {
        args.get("model_index")
            .and_then(Value::as_u64)
            .and_then(|value| usize::try_from(value).ok())
            .ok_or_else(|| invalid_arg("model_index", "is required for replace and remove"))
    };
    let model = || {
        args.get("model")
            .ok_or_else(|| invalid_arg("model", "is required for append and replace"))
            .and_then(parse_model)
    };
    match mode {
        "inspect" => Ok(None),
        "append" => Ok(Some(IpcFootprint3DModelEdit::Append { model: model()? })),
        "replace" => Ok(Some(IpcFootprint3DModelEdit::Replace {
            index: index()?,
            model: model()?,
        })),
        "remove" => Ok(Some(IpcFootprint3DModelEdit::Remove { index: index()? })),
        _ => Err(invalid_arg(
            "mode",
            "must be inspect, append, replace, or remove",
        )),
    }
}

fn parse_model(value: &Value) -> Result<IpcFootprint3DModel, CallToolResult> {
    let path = value
        .get("path")
        .and_then(Value::as_str)
        .unwrap_or_default();
    if path.trim().is_empty() || path.contains('\0') {
        return Err(invalid_arg(
            "model.path",
            "must be non-empty and contain no NUL",
        ));
    }
    let offset_mm = parse_vector(value.get("offset_mm"), 0.0, "model.offset_mm")?;
    let rotation_degrees =
        parse_vector(value.get("rotation_degrees"), 0.0, "model.rotation_degrees")?;
    let scale = parse_vector(value.get("scale"), 1.0, "model.scale")?;
    let opacity = value.get("opacity").and_then(Value::as_f64).unwrap_or(1.0);
    if !opacity.is_finite() || !(0.0..=1.0).contains(&opacity) {
        return Err(invalid_arg(
            "model.opacity",
            "must be finite and between 0 and 1",
        ));
    }
    Ok(IpcFootprint3DModel {
        filename: path.to_string(),
        offset_mm,
        rotation_degrees,
        scale,
        visible: value
            .get("visible")
            .and_then(Value::as_bool)
            .unwrap_or(true),
        opacity,
    })
}

fn parse_vector(
    value: Option<&Value>,
    default: f64,
    field: &str,
) -> Result<IpcVector3, CallToolResult> {
    let component = |name| {
        value
            .and_then(|value| value.get(name))
            .and_then(Value::as_f64)
            .unwrap_or(default)
    };
    let vector = IpcVector3 {
        x: component("x"),
        y: component("y"),
        z: component("z"),
    };
    if [vector.x, vector.y, vector.z]
        .into_iter()
        .all(f64::is_finite)
    {
        Ok(vector)
    } else {
        Err(invalid_arg(field, "components must be finite numbers"))
    }
}

fn models_revision(board: &std::path::Path, snapshot: &IpcFootprint3DModelSnapshot) -> String {
    let mut hasher = Sha256::new();
    hasher.update(board.to_string_lossy().as_bytes());
    hasher.update([0]);
    hasher.update(snapshot.reference.as_bytes());
    hasher.update([0]);
    hasher.update(snapshot.kiid.as_bytes());
    hasher.update([0]);
    hasher.update(serde_json::to_vec(&snapshot.models).expect("model snapshots serialize"));
    format!("{:x}", hasher.finalize())
}

/// The saved-file form, reached only when KiCad did not serve the board.
async fn handle_closed(
    ctx: &ToolContext,
    board: &std::path::Path,
    reference: &str,
    edit: Option<&IpcFootprint3DModelEdit>,
    expected_revision: Option<&str>,
) -> anyhow::Result<CallToolResult> {
    let closure = match closed_board::prove_closed(ctx, board, OPERATION).await? {
        Ok(closure) => closure,
        Err(refusal) => return Ok(refusal),
    };
    let original = konnect_sexp::read_consistent(board)?;
    let saved = match saved_models::read(&original, reference) {
        Ok(saved) => saved,
        Err(error) => return Ok(saved_error(error, board, reference)),
    };
    let finish = |status: &str, applied: bool, snapshot: &IpcFootprint3DModelSnapshot| {
        let mut body = snapshot_body(status, "saved", applied, board, snapshot);
        body["pad_fabrication_properties"] = json!(saved.pad_properties);
        body["board_closure"] = closure.clone();
        body["other_footprint_clauses"] =
            json!("unchanged byte-for-byte; no whole-footprint typed update");
        body["note"] = json!(
            "Read from the saved file: the last save, not unsaved editor state. Reload the board \
             in KiCad to see a change made here."
        );
        body
    };
    let Some(edit) = edit else {
        return Ok(CallToolResult::json(&finish(
            "inspected",
            false,
            &saved.snapshot,
        )));
    };
    let actual = models_revision(board, &saved.snapshot);
    if expected_revision != Some(actual.as_str()) {
        return Ok(CallToolResult::error_kind(
            ToolErrorKind::StaleTarget {
                target: format!("{} footprint {reference}", board.display()),
                reason: format!("3D-model revision is now {actual}"),
            },
            format!(
                "The placed footprint's 3D-model list in the saved board does not match the \
                 inspected revision. Re-inspect and retry with revision {actual}. The board file \
                 was not modified."
            ),
        ));
    }
    let prepared = match saved_models::prepare(&original, reference, edit) {
        Ok(prepared) => prepared,
        Err(error) => return Ok(saved_error(error, board, reference)),
    };
    let after = IpcFootprint3DModelSnapshot {
        models: prepared.expected.clone(),
        ..saved.snapshot.clone()
    };
    if !prepared.changed() {
        return Ok(CallToolResult::json(&finish("unchanged", false, &after)));
    }
    if let Err(refusal) = closed_board::prove_closed(ctx, board, OPERATION).await? {
        return Ok(refusal);
    }
    let write = match closed_board::write_and_read_back(
        board,
        OPERATION,
        &original,
        &prepared.candidate,
        |text| saved_models::verify(&original, text, &prepared, reference),
    ) {
        Ok(write) => write,
        Err(refusal) => return Ok(refusal),
    };
    let mut body = finish("applied", true, &after);
    body["write"] = write;
    body["undo"] = json!(
        "none: the saved file was replaced in one atomic write; write.original_sha256 names \
         the bytes it replaced"
    );
    Ok(CallToolResult::json(&body))
}

fn saved_error(error: SavedModelError, board: &std::path::Path, reference: &str) -> CallToolResult {
    match error {
        SavedModelError::Missing => CallToolResult::error_kind(
            ToolErrorKind::StaleTarget {
                target: reference.to_string(),
                reason: "no placed footprint has this exact reference".to_string(),
            },
            format!(
                "No footprint in the saved board {} has the reference {reference}.",
                board.display()
            ),
        ),
        SavedModelError::Ambiguous(matches) => CallToolResult::error_kind(
            ToolErrorKind::AmbiguousTarget {
                target: reference.to_string(),
                candidates: (0..matches).map(|index| format!("match {index}")).collect(),
            },
            format!("{matches} footprints in the saved board have the reference {reference}."),
        ),
        SavedModelError::IndexOutOfRange { index, count } => invalid_arg(
            "model_index",
            &format!("{index} names no entry: {reference} has {count} 3D model(s)"),
        ),
        SavedModelError::Other(reason) => CallToolResult::error_kind(
            ToolErrorKind::PlanBlocked {
                operation: OPERATION.to_string(),
                reasons: vec![reason.clone()],
            },
            format!("No 3D model changed: {reason}. The board file was not modified."),
        ),
    }
}

fn snapshot_result(
    status: &str,
    applied: bool,
    board: &std::path::Path,
    snapshot: &IpcFootprint3DModelSnapshot,
) -> CallToolResult {
    CallToolResult::json(&snapshot_body(status, "ipc", applied, board, snapshot))
}

fn snapshot_body(
    status: &str,
    source: &str,
    applied: bool,
    board: &std::path::Path,
    snapshot: &IpcFootprint3DModelSnapshot,
) -> Value {
    let models = snapshot
        .models
        .iter()
        .enumerate()
        .map(|(index, model)| json!({ "index": index, "model": model }))
        .collect::<Vec<_>>();
    json!({
        "status": status,
        "source": source,
        "applied": applied,
        "reference": snapshot.reference,
        "kiid": snapshot.kiid,
        "models_revision": models_revision(board, snapshot),
        "models": models,
        "model_count": models.len(),
        "undo": if applied { Value::String("one KiCad undo step".to_string()) } else { Value::Null }
    })
}

fn footprint_error(error: Error, reference: &str) -> CallToolResult {
    if let Some(target) = error.downcast_ref::<FootprintTargetError>() {
        return match target {
            FootprintTargetError::Missing { .. } => CallToolResult::error_kind(
                ToolErrorKind::StaleTarget {
                    target: reference.to_string(),
                    reason: "no placed footprint has this exact reference".to_string(),
                },
                target.to_string(),
            ),
            FootprintTargetError::Ambiguous { matches, .. } => CallToolResult::error_kind(
                ToolErrorKind::AmbiguousTarget {
                    target: reference.to_string(),
                    candidates: (0..*matches)
                        .map(|index| format!("match {index}"))
                        .collect(),
                },
                target.to_string(),
            ),
        };
    }
    CallToolResult::error_kind(
        ToolErrorKind::HandlerError {
            reason: format!("{error:#}"),
        },
        format!("Placed-footprint 3D-model operation failed: {error:#}"),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn snapshot(path: &str) -> IpcFootprint3DModelSnapshot {
        IpcFootprint3DModelSnapshot {
            reference: "U1".to_string(),
            kiid: "abc".to_string(),
            models: vec![IpcFootprint3DModel {
                filename: path.to_string(),
                offset_mm: IpcVector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                rotation_degrees: IpcVector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                scale: IpcVector3 {
                    x: 1.0,
                    y: 1.0,
                    z: 1.0,
                },
                visible: true,
                opacity: 1.0,
            }],
        }
    }

    #[test]
    fn revision_tracks_exact_ordered_model_state() {
        assert_eq!(
            models_revision(std::path::Path::new("board.kicad_pcb"), &snapshot("a.step")),
            models_revision(std::path::Path::new("board.kicad_pcb"), &snapshot("a.step"))
        );
        assert_ne!(
            models_revision(std::path::Path::new("board.kicad_pcb"), &snapshot("a.step")),
            models_revision(std::path::Path::new("board.kicad_pcb"), &snapshot("b.step"))
        );
    }

    #[test]
    fn mutation_modes_require_their_exact_arguments() {
        assert!(parse_edit(&json!({}), "inspect").unwrap().is_none());
        assert!(parse_edit(&json!({}), "append").is_err());
        assert!(parse_edit(&json!({ "model": { "path": "x.step" } }), "replace").is_err());
        assert!(parse_edit(&json!({}), "remove").is_err());
    }

    #[test]
    fn model_defaults_are_complete_and_invalid_values_are_refused() {
        let model = parse_model(&json!({ "path": "x.step" })).unwrap();
        assert_eq!(
            model.scale,
            IpcVector3 {
                x: 1.0,
                y: 1.0,
                z: 1.0
            }
        );
        assert_eq!(model.opacity, 1.0);
        assert!(parse_model(&json!({ "path": "x.step", "opacity": 2.0 })).is_err());
        assert!(parse_model(&json!({ "path": "" })).is_err());
    }

    /// U2's pad 9 carries `pad_prop_heatsink` and its second model is a
    /// dangling entry; the live path refuses such a footprint.
    const BOARD: &str = r#"(kicad_pcb (version 20260206)
	(footprint "Package_SO:SOIC-8-EP"
		(layer "F.Cu")
		(uuid "11111111-1111-4111-8111-111111111111")
		(at 20 30)
		(property "Reference" "U2" (at 0 -3) (layer "F.SilkS") (uuid "21111111-1111-4111-8111-111111111111"))
		(pad "9" smd rect (at 0 0) (size 2.95 4.9) (property pad_prop_heatsink) (layers "F.Cu" "F.Mask") (net 1 "GND") (uuid "51111111-1111-4111-8111-111111111111"))
		(embedded_fonts no)
		(model "${KICAD10_3DMODEL_DIR}/Package_SO.3dshapes/SOIC-8-EP.step"
			(offset
				(xyz 0 0 0)
			)
			(scale
				(xyz 1 1 1)
			)
			(rotate
				(xyz 0 0 0)
			)
		)
		(model "${KIPRJMOD}/3d/missing.step"
			(offset
				(xyz 0 0 0)
			)
			(scale
				(xyz 1 1 1)
			)
			(rotate
				(xyz 0 0 0)
			)
		)
	)
	(gr_line (start 0 0) (end 1 1) (stroke (width 0.1) (type solid)) (layer "Edge.Cuts") (uuid "61111111-1111-4111-8111-111111111111"))
)
"#;

    async fn call(args: Value) -> (bool, Value) {
        let def = tool();
        assert!(def.input_validator.is_valid(&args), "schema refused {args}");
        let ctx = crate::tools::pcb_board::board_mock::ctx_talking_to(String::new());
        let result = (def.handler)(&args, std::sync::Arc::new(ctx))
            .await
            .unwrap();
        let body = match &result.content[0] {
            crate::mcp::protocol::ToolContent::Text { text } => {
                serde_json::from_str(text).unwrap_or_else(|_| json!({ "text": text }))
            }
            other => panic!("expected text, got {other:?}"),
        };
        (result.is_error, body)
    }

    #[tokio::test]
    async fn a_closed_board_inspects_then_removes_one_entry_and_keeps_the_pad_property() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("yoshi.kicad_pcb");
        std::fs::write(&board, BOARD).unwrap();

        let (error, inspected) =
            call(json!({ "board": board, "reference": "U2", "mode": "inspect" })).await;
        assert!(!error, "{inspected}");
        assert_eq!(inspected["source"], "saved");
        assert_eq!(inspected["model_count"], 2);
        assert_eq!(
            inspected["models"][1]["model"]["filename"],
            "${KIPRJMOD}/3d/missing.step"
        );
        assert_eq!(inspected["board_closure"]["proven_closed"], true);
        assert_eq!(
            inspected["pad_fabrication_properties"][0]["property"],
            "pad_prop_heatsink"
        );

        // A mutation needs the inspected revision, and a wrong one writes nothing.
        let mut remove =
            json!({ "board": board, "reference": "U2", "mode": "remove", "model_index": 1 });
        let (error, missing) = call(remove.clone()).await;
        assert!(error);
        assert_eq!(missing["error"]["kind"], "invalid_argument", "{missing}");
        remove["expected_models_revision"] = json!("f".repeat(64));
        let (error, stale) = call(remove.clone()).await;
        assert!(error);
        assert_eq!(stale["error"]["kind"], "stale_target", "{stale}");
        assert_eq!(std::fs::read_to_string(&board).unwrap(), BOARD);

        remove["expected_models_revision"] = inspected["models_revision"].clone();
        let (error, applied) = call(remove.clone()).await;
        assert!(!error, "{applied}");
        assert_eq!(applied["status"], "applied");
        assert_eq!(applied["source"], "saved");
        assert_eq!(applied["model_count"], 1);
        assert_eq!(applied["write"]["persisted_bytes_match_plan"], true);
        assert_eq!(
            applied["write"]["persisted_readback"]["pad_fabrication_properties"][0]["property"],
            "pad_prop_heatsink"
        );
        let written = std::fs::read_to_string(&board).unwrap();
        assert!(!written.contains("missing.step"));
        // Everything but the removed block is the byte it was.
        let cut = BOARD.find("\n\t\t(model \"${KIPRJMOD}").unwrap();
        let rest = BOARD.find("\n\t)\n\t(gr_line").unwrap();
        assert_eq!(written, format!("{}{}", &BOARD[..cut], &BOARD[rest..]));

        // The revision moved with the list, so the same request is now stale.
        let (error, repeated) = call(remove).await;
        assert!(error);
        assert_eq!(repeated["error"]["kind"], "stale_target", "{repeated}");
    }

    #[tokio::test]
    async fn a_closed_board_appends_and_replaces_and_reports_bad_targets() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("yoshi.kicad_pcb");
        std::fs::write(&board, BOARD).unwrap();
        let revision = |body: &Value| body["models_revision"].clone();

        let (_, inspected) =
            call(json!({ "board": board, "reference": "U2", "mode": "inspect" })).await;
        let (error, replaced) = call(json!({
            "board": board, "reference": "U2", "mode": "replace", "model_index": 1,
            "expected_models_revision": revision(&inspected),
            "model": { "path": "${KIPRJMOD}/3d/part.step", "rotation_degrees": { "z": 90.0 }, "visible": false }
        }))
        .await;
        assert!(!error, "{replaced}");
        assert_eq!(replaced["models"][1]["model"]["visible"], false);
        assert_eq!(
            replaced["models"][1]["model"]["rotation_degrees"]["z"],
            90.0
        );

        let (error, appended) = call(json!({
            "board": board, "reference": "U2", "mode": "append",
            "expected_models_revision": revision(&replaced),
            "model": { "path": "extra.wrl", "opacity": 0.5 }
        }))
        .await;
        assert!(!error, "{appended}");
        assert_eq!(appended["model_count"], 3);

        // A fresh inspection reads back exactly what the edits reported.
        let (_, again) =
            call(json!({ "board": board, "reference": "U2", "mode": "inspect" })).await;
        assert_eq!(again["models"], appended["models"]);
        assert_eq!(revision(&again), revision(&appended));
        assert!(std::fs::read_to_string(&board)
            .unwrap()
            .contains("(property pad_prop_heatsink)"));

        let (error, out_of_range) = call(json!({
            "board": board, "reference": "U2", "mode": "remove", "model_index": 9,
            "expected_models_revision": revision(&again)
        }))
        .await;
        assert!(error);
        assert_eq!(
            out_of_range["error"]["kind"], "invalid_argument",
            "{out_of_range}"
        );
        let (error, unknown) =
            call(json!({ "board": board, "reference": "U99", "mode": "inspect" })).await;
        assert!(error);
        assert_eq!(unknown["error"]["kind"], "stale_target", "{unknown}");
    }

    /// With KiCad's sibling lock present the board is not proven closed, and
    /// neither an inspection nor an edit touches the file.
    #[tokio::test]
    async fn a_board_not_proven_closed_is_refused() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("yoshi.kicad_pcb");
        std::fs::write(&board, BOARD).unwrap();
        std::fs::write(tmp.path().join("~yoshi.kicad_pcb.lck"), "{}").unwrap();
        let (error, refused) =
            call(json!({ "board": board, "reference": "U2", "mode": "inspect" })).await;
        assert!(error, "{refused}");
        assert_eq!(std::fs::read_to_string(&board).unwrap(), BOARD);
    }
}
