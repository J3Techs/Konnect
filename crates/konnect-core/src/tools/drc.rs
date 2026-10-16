//! One ordering and source-evidence contract for both CLI DRC surfaces.

use super::{cli, ipc_target_error_result, with_board_ipc_classified, ToolContext};
use crate::mcp::{error::ToolErrorKind, protocol::CallToolResult};
use konnect_ipc::IpcFailure;
use serde_json::{json, Value};
use std::{path::Path, time::Duration};

const READY_TIMEOUT: Duration = Duration::from_secs(60);

/// The schema fragment both DRC tools share for narrowing what they return.
pub(crate) fn selection_schema(default_limit: Option<u64>) -> Value {
    let mut limit = json!({
        "type": "integer",
        "minimum": 0,
        "description": "Maximum number of violations to list after filtering; 0 lists none \
                        (the counts in by_rule still cover every match). Omit for the default."
    });
    if let Some(default) = default_limit {
        limit["default"] = json!(default);
    }
    json!({
        "limit": limit,
        "rules": {
            "type": "array",
            "items": { "type": "string" },
            "description": "Only these KiCad rule keys (e.g. 'clearance', 'courtyards_overlap', \
                            'unconnected_items')."
        },
        "exclude_rules": {
            "type": "array",
            "items": { "type": "string" },
            "description": "Leave out these KiCad rule keys (e.g. 'silk_overlap', \
                            'unconnected_items')."
        }
    })
}

/// Which findings a DRC tool returns: a minimum severity, an optional rule
/// allow-list, a rule deny-list and a cap on how many are listed.
pub(crate) struct ViolationSelection {
    min_rank: u8,
    rules: Option<Vec<String>>,
    exclude_rules: Vec<String>,
    pub(crate) limit: Option<usize>,
}

pub(crate) fn severity_rank(severity: &str) -> u8 {
    match severity {
        "error" => 2,
        "warning" => 1,
        _ => 0,
    }
}

impl ViolationSelection {
    /// Read the selection from tool arguments. A `rules` or `exclude_rules`
    /// that is not an array of strings is refused rather than ignored, so a
    /// typo cannot silently widen the result back to every rule.
    pub(crate) fn from_args(
        args: &Value,
        severity: &str,
        default_limit: Option<usize>,
    ) -> anyhow::Result<Self> {
        let list = |name: &str| -> anyhow::Result<Option<Vec<String>>> {
            match &args[name] {
                Value::Null => Ok(None),
                Value::Array(items) => items
                    .iter()
                    .map(|item| {
                        item.as_str()
                            .map(str::to_string)
                            .ok_or_else(|| anyhow::anyhow!("'{name}' must hold only strings"))
                    })
                    .collect::<anyhow::Result<Vec<_>>>()
                    .map(Some),
                _ => anyhow::bail!("'{name}' must be an array of rule keys"),
            }
        };
        let limit = match &args["limit"] {
            Value::Null => default_limit,
            value => Some(
                value
                    .as_u64()
                    .ok_or_else(|| anyhow::anyhow!("'limit' must be a non-negative integer"))?
                    as usize,
            ),
        };
        Ok(Self {
            min_rank: severity_rank(severity),
            rules: list("rules")?,
            exclude_rules: list("exclude_rules")?.unwrap_or_default(),
            limit,
        })
    }

    pub(crate) fn matches(&self, violation: &cli::DrcViolation) -> bool {
        severity_rank(&violation.severity) >= self.min_rank
            && self
                .rules
                .as_ref()
                .is_none_or(|rules| rules.contains(&violation.rule))
            && !self.exclude_rules.contains(&violation.rule)
    }

    /// Every matching finding, in report order.
    pub(crate) fn select<'a>(&self, report: &'a cli::DrcReport) -> Vec<&'a cli::DrcViolation> {
        report.all().filter(|v| self.matches(v)).collect()
    }

    /// How many of the selected findings are listed.
    pub(crate) fn shown(&self, selected: usize) -> usize {
        self.limit.map_or(selected, |limit| selected.min(limit))
    }
}

/// Selected findings counted by rule key, then by severity, so a caller can
/// see the shape of a large report without listing it.
pub(crate) fn counts_by_rule(selected: &[&cli::DrcViolation]) -> Value {
    let mut counts: std::collections::BTreeMap<&str, std::collections::BTreeMap<&str, usize>> =
        Default::default();
    for violation in selected {
        *counts
            .entry(violation.rule.as_str())
            .or_default()
            .entry(violation.severity.as_str())
            .or_default() += 1;
    }
    json!(counts)
}

/// Publish by sibling-file replacement, never by truncating a destination inode.
/// This also preserves the source board if an output path is a distinct hard link.
pub(crate) async fn write_report(output: &str, contents: &str) -> anyhow::Result<()> {
    use anyhow::Context;
    let path = std::path::PathBuf::from(output);
    let contents = contents.to_string();
    tokio::task::spawn_blocking(move || {
        if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
            std::fs::create_dir_all(parent).with_context(|| {
                format!("could not create report directory {}", parent.display())
            })?;
        }
        konnect_sexp::writer::write_atomic(&path, &contents)
            .with_context(|| format!("could not write report to {}", path.display()))
    })
    .await?
}

#[derive(serde::Serialize)]
pub(crate) struct DrcSourceEvidence {
    pub source: &'static str,
    pub live_board_synced: bool,
    pub zones_refilled: bool,
    pub zone_refill_source: Option<&'static str>,
}

/// Preserve completed work when optional report publication fails afterwards.
pub(crate) fn report_write_failure(
    evidence: &DrcSourceEvidence,
    board: &Path,
    output: &str,
    error: anyhow::Error,
) -> anyhow::Result<CallToolResult> {
    if !evidence.live_board_synced {
        return Err(error);
    }
    let body = json!({
        "error": ToolErrorKind::HandlerError { reason: format!("could not write DRC report to {output}: {error:#}") },
        "source_evidence": evidence,
        "board": board.display().to_string(),
        "message": "Board synchronization and CLI DRC completed, but optional report publication failed. Do not repeat refill/save blindly; correct the output destination and recover the report from the already-saved board.",
    });
    let mut result = CallToolResult::json(&body);
    result.is_error = true;
    Ok(result)
}

fn uncertain(board: &Path, operation: &str, reason: impl std::fmt::Display) -> CallToolResult {
    CallToolResult::error_kind(
        ToolErrorKind::MutationOutcomeUncertain {
            operation: operation.into(),
            path: board.display().to_string(),
            reason: reason.to_string(),
        },
        format!("{operation}: {reason}. The operation was not accepted as verified. Inspect the requested board in KiCad and its saved file before retrying; do not repeat possibly applied work blindly."),
    )
}

fn target_failure(failure: IpcFailure) -> CallToolResult {
    match failure {
        IpcFailure::Target { error, .. } => ipc_target_error_result(&error),
        other => CallToolResult::error_kind(
            ToolErrorKind::EditorUnavailable {
                editor: "pcb".into(),
                reason: other.to_string(),
            },
            format!("Cannot synchronize the requested board: {other}. CLI DRC was not started; open that board with IPC enabled and retry."),
        ),
    }
}

/// Complete a requested live refill without saving or switching to another board.
pub(crate) async fn refill(
    ctx: &ToolContext,
    board: &Path,
) -> anyhow::Result<Result<(), CallToolResult>> {
    let path = board.to_path_buf();
    let result = with_board_ipc_classified(ctx, board, move |client, _| {
        Ok(client
            .refill_zones()
            .and_then(|()| client.wait_for_board_ready(READY_TIMEOUT))
            .map_err(|error| uncertain(&path, "refill_zones", error)))
    })
    .await?;
    Ok(match result {
        Ok(result) => result,
        Err(error) => Err(target_failure(error)),
    })
}

pub(crate) async fn run(
    ctx: &ToolContext,
    board: &Path,
    args: &Value,
) -> anyhow::Result<Result<(cli::DrcReport, DrcSourceEvidence), CallToolResult>> {
    let option = |field: &str| -> Result<bool, CallToolResult> {
        match args.get(field) {
            None => Ok(false),
            Some(value) => value.as_bool().ok_or_else(|| {
                CallToolResult::error_kind(
                    ToolErrorKind::InvalidArgument {
                        field: field.into(),
                        reason: "must be a boolean".into(),
                    },
                    format!("{field} must be a boolean; no work was applied."),
                )
            }),
        }
    };
    let (sync, refill) = match (option("sync_live_board"), option("refill_zones")) {
        (Ok(sync), Ok(refill)) => (sync, refill),
        (Err(error), _) | (_, Err(error)) => return Ok(Err(error)),
    };
    // Refuse obvious publication errors before any live save/refill. Later
    // permission changes or filesystem races still need applied-state recovery.
    if let Some(output) = args["output"].as_str() {
        let output_path = Path::new(output);
        let same_board = match (output_path.canonicalize(), board.canonicalize()) {
            (Ok(output), Ok(board)) => output == board,
            _ => output_path == board,
        };
        let invalid = same_board
            || output_path.is_dir()
            || output_path
                .ancestors()
                .skip(1)
                .any(|parent| parent.is_file());
        if invalid {
            return Ok(Err(CallToolResult::error_kind(
                ToolErrorKind::InvalidArgument {
                    field: "output".into(),
                        reason: "destination aliases the source board, is a directory or has a non-directory ancestor".into(),
                },
                "Choose a writable report file path. No refill, save or CLI DRC was started.",
            )));
        }
    }
    let mut saved_snapshot = None;
    if sync {
        let path = board.to_path_buf();
        let synchronized = with_board_ipc_classified(ctx, board, move |client, _| {
            let operation = || -> anyhow::Result<String> {
                if refill {
                    client.refill_zones()?;
                    client.wait_for_board_ready(READY_TIMEOUT)?;
                }
                client.save_board()?;
                client.wait_for_board_ready(READY_TIMEOUT)?;
                let live = client.save_document_to_string()?;
                let saved = std::fs::read_to_string(&path)?;
                let mut live_tree = konnect_sexp::parse_sexp(&live)?;
                let mut saved_tree = konnect_sexp::parse_sexp(&saved)?;
                normalize_snapshot_metadata(&mut live_tree);
                normalize_snapshot_metadata(&mut saved_tree);
                if live_tree != saved_tree {
                    anyhow::bail!("saved PCB does not match the observed live document");
                }
                Ok(saved)
            };
            Ok(operation().map_err(|error| uncertain(&path, "synchronize_drc_source", error)))
        })
        .await?;
        match synchronized {
            Ok(Ok(snapshot)) => saved_snapshot = Some(snapshot),
            Ok(Err(error)) => return Ok(Err(error)),
            Err(error) => return Ok(Err(target_failure(error))),
        }
    }
    // CLI refill is analysis-only. In synchronized mode the persisted IPC fill
    // is authoritative, so do not refill a second time in a different process.
    let report = match cli::run_drc(&ctx.config.kicad_cli, board, refill && !sync).await {
        Ok(report) => report,
        Err(error) if sync => return Ok(Err(uncertain(board, "drc_after_save", error))),
        Err(error) => return Err(error),
    };
    if let Some(snapshot) = saved_snapshot {
        match tokio::fs::read_to_string(board).await {
            Ok(current) if current == snapshot => {}
            Ok(_) => {
                return Ok(Err(uncertain(
                    board,
                    "verify_drc_source",
                    "saved PCB changed during CLI DRC",
                )))
            }
            Err(error) => return Ok(Err(uncertain(board, "verify_drc_source", error))),
        }
    }
    Ok(Ok((
        report,
        DrcSourceEvidence {
            source: "saved_file",
            live_board_synced: sync,
            zones_refilled: refill,
            zone_refill_source: if refill {
                Some(if sync { "ipc" } else { "kicad_cli" })
            } else {
                None
            },
        },
    )))
}

// IPC serializes embedded footprints with standalone library format metadata;
// SaveDocument omits that metadata in a board. Never discard geometry, models,
// connectivity, UUIDs, or arbitrary unknown fields to make a comparison pass.
fn normalize_snapshot_metadata(node: &mut konnect_sexp::SexpNode) {
    use konnect_sexp::SexpNode;
    let footprint = node.head() == Some("footprint");
    if let SexpNode::List(children) = node {
        if footprint {
            children.retain(|child| {
                !matches!(
                    child.head(),
                    Some("version" | "generator" | "generator_version")
                )
            });
        }
        for child in children {
            normalize_snapshot_metadata(child);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{mcp::handler::McpHandler, test_support::MockIpcServer, tools::ServerConfig};
    use konnect_ipc::gen::kiapi;
    use prost::Message;
    use std::sync::{Arc, Mutex};

    fn finding(rule: &str, severity: &str) -> cli::DrcViolation {
        cli::DrcViolation {
            severity: severity.to_string(),
            description: String::new(),
            rule: rule.to_string(),
            pos: None,
            items: Vec::new(),
        }
    }

    fn report() -> cli::DrcReport {
        cli::DrcReport {
            cli_messages: Vec::new(),
            custom_rules_error: None,
            custom_rules: None,
            violations: vec![
                finding("silk_overlap", "warning"),
                finding("clearance", "error"),
                finding("silk_overlap", "warning"),
                finding("text_height", "warning"),
            ],
            unconnected_items: Some(vec![
                finding("unconnected_items", "error"),
                finding("unconnected_items", "error"),
            ]),
            schematic_parity: Some(Vec::new()),
            ownership_diagnostic: None,
            schematic_parity_diagnostic: None,
        }
    }

    /// Rule allow- and deny-lists narrow every category, the severity floor
    /// still applies, and the counts cover every match even when the listed
    /// findings are capped.
    #[test]
    fn violation_selection_filters_by_rule_and_counts_beyond_the_limit() {
        let report = report();
        let only = ViolationSelection::from_args(
            &json!({ "exclude_rules": ["silk_overlap", "unconnected_items"] }),
            "warning",
            None,
        )
        .unwrap();
        let rules: Vec<_> = only
            .select(&report)
            .iter()
            .map(|v| v.rule.as_str())
            .collect();
        assert_eq!(rules, ["clearance", "text_height"]);

        let errors = ViolationSelection::from_args(
            &json!({ "rules": ["unconnected_items", "silk_overlap"], "limit": 1 }),
            "error",
            Some(50),
        )
        .unwrap();
        let selected = errors.select(&report);
        assert_eq!(selected.len(), 2);
        assert_eq!(errors.shown(selected.len()), 1);
        assert_eq!(
            counts_by_rule(&selected),
            json!({ "unconnected_items": { "error": 2 } })
        );

        let none_listed =
            ViolationSelection::from_args(&json!({ "limit": 0 }), "warning", None).unwrap();
        let selected = none_listed.select(&report);
        assert_eq!(none_listed.shown(selected.len()), 0);
        assert_eq!(
            counts_by_rule(&selected),
            json!({
                "clearance": { "error": 1 },
                "silk_overlap": { "warning": 2 },
                "text_height": { "warning": 1 },
                "unconnected_items": { "error": 2 }
            })
        );
        let unlimited = ViolationSelection::from_args(&json!({}), "warning", None).unwrap();
        assert_eq!(unlimited.shown(6), 6);
    }

    #[test]
    fn violation_selection_refuses_malformed_filters() {
        for args in [
            json!({ "rules": "clearance" }),
            json!({ "exclude_rules": [1] }),
            json!({ "limit": -1 }),
            json!({ "limit": "ten" }),
        ] {
            assert!(
                ViolationSelection::from_args(&args, "warning", None).is_err(),
                "{args}"
            );
        }
    }

    const OLD: &str = include_str!("../../tests/fixtures/drc_ownership_j1.kicad_pcb");
    const LIVE: &str = include_str!("../../../konnect-ipc/tests/fixtures/live_ipc.kicad_pcb");

    fn response(message: Option<prost_types::Any>, busy: bool) -> kiapi::common::ApiResponse {
        kiapi::common::ApiResponse {
            status: Some(kiapi::common::ApiResponseStatus {
                status: if busy {
                    kiapi::common::ApiStatusCode::AsBusy
                } else {
                    kiapi::common::ApiStatusCode::AsOk
                } as i32,
                error_message: String::new(),
            }),
            header: None,
            message,
        }
    }

    fn packed(message: impl Message, name: &str) -> Option<prost_types::Any> {
        Some(prost_types::Any {
            type_url: format!("type.googleapis.com/{name}"),
            value: message.encode_to_vec(),
        })
    }

    async fn call(
        tool: &str,
        board: &Path,
        address: &str,
        executable: &Path,
        extra: Value,
    ) -> Value {
        let handler = McpHandler::new(ServerConfig {
            kicad_cli: executable.display().to_string(),
            kicad_binary: String::new(),
            ipc_address: address.into(),
            project_dir: None,
            jlcpcb_db_path: None,
            auto_load_toolsets: true,
            eager_toolsets: false,
        })
        .await
        .unwrap();
        let mut args = json!({"board": board.display().to_string()});
        args.as_object_mut()
            .unwrap()
            .extend(extra.as_object().unwrap().clone());
        let result = handler
            .handle_message(json!({"jsonrpc":"2.0", "id":408,
            "method":"tools/call", "params":{"name":tool,"arguments":args}}))
            .await
            .unwrap()
            .result
            .unwrap();
        let mut body: Value =
            serde_json::from_str(result["content"][0]["text"].as_str().unwrap()).unwrap();
        body["isError"] = result["isError"].clone();
        body
    }

    fn cli_fixture(dir: &Path, board: &Path) -> std::path::PathBuf {
        std::fs::write(
            board.with_extension("drc.json"),
            r#"{"violations":[],"unconnected_items":[],"schematic_parity":[]}"#,
        )
        .unwrap();
        cli::test_support::write_script(dir, "drc-probe",
            "#!/bin/sh\nfor last; do :; done\ncp \"$last\" \"$last.observed\"\nexit 0\n",
            "@echo off\r\n:loop\r\nset last=%1\r\nshift\r\nif not \"%1\"==\"\" goto loop\r\ncopy /y %last% %last%.observed >nul\r\nexit /b 0\r\n")
    }

    fn mock(board: &Path, commands: Arc<Mutex<Vec<String>>>, mode: &'static str) -> MockIpcServer {
        let path = board.to_path_buf();
        let busy = Arc::new(Mutex::new(false));
        MockIpcServer::spawn("drc-sync", move |request| {
            let command = request.message.unwrap();
            let name = command.type_url.rsplit('/').next().unwrap().to_string();
            commands.lock().unwrap().push(name.clone());
            if *busy.lock().unwrap() && name == "kiapi.common.commands.GetOpenDocuments" {
                if mode != "busy-forever" {
                    *busy.lock().unwrap() = false;
                }
                return response(None, true);
            }
            let document = |filename: &str| kiapi::common::types::DocumentSpecifier {
                r#type: kiapi::common::types::DocumentType::DoctypePcb as i32,
                identifier: Some(
                    kiapi::common::types::document_specifier::Identifier::BoardFilename(
                        filename.into(),
                    ),
                ),
                project: Some(kiapi::common::types::ProjectSpecifier {
                    name: "clock".into(),
                    path: path.parent().unwrap().display().to_string(),
                }),
            };
            match name.as_str() {
                "kiapi.common.commands.GetOpenDocuments" => response(
                    packed(
                        kiapi::common::commands::GetOpenDocumentsResponse {
                            documents: if mode == "wrong" {
                                vec![document("other.kicad_pcb")]
                            } else {
                                vec![
                                    document("other.kicad_pcb"),
                                    document(path.file_name().unwrap().to_str().unwrap()),
                                ]
                            },
                        },
                        "kiapi.common.commands.GetOpenDocumentsResponse",
                    ),
                    false,
                ),
                "kiapi.board.commands.RefillZones" => {
                    let cmd = kiapi::board::commands::RefillZones::decode(command.value.as_slice())
                        .unwrap();
                    assert_eq!(
                        cmd.board.unwrap(),
                        document(path.file_name().unwrap().to_str().unwrap())
                    );
                    if mode == "refill-fail" {
                        let mut rejected = response(None, false);
                        rejected.status.as_mut().unwrap().status =
                            kiapi::common::ApiStatusCode::AsBadRequest as i32;
                        return rejected;
                    }
                    *busy.lock().unwrap() = true;
                    response(None, false)
                }
                "kiapi.common.commands.SaveDocument" => {
                    let cmd =
                        kiapi::common::commands::SaveDocument::decode(command.value.as_slice())
                            .unwrap();
                    assert_eq!(
                        cmd.document.unwrap(),
                        document(path.file_name().unwrap().to_str().unwrap())
                    );
                    if mode == "save-fail" {
                        let mut rejected = response(None, false);
                        rejected.status.as_mut().unwrap().status =
                            kiapi::common::ApiStatusCode::AsBadRequest as i32;
                        rejected
                    } else {
                        if mode != "stale" {
                            std::fs::write(&path, LIVE).unwrap();
                        }
                        if mode == "report-fail" {
                            let report = path.with_extension("report");
                            if !report.exists() {
                                std::fs::create_dir(report).unwrap();
                            }
                        }
                        response(None, false)
                    }
                }
                "kiapi.common.commands.SaveDocumentToString" => response(
                    packed(
                        kiapi::common::commands::SavedDocumentResponse {
                            contents: LIVE.into(),
                            document: Some(document(path.file_name().unwrap().to_str().unwrap())),
                        },
                        "kiapi.common.commands.SavedDocumentResponse",
                    ),
                    false,
                ),
                _ => panic!("unexpected IPC command: {name}"),
            }
        })
    }

    #[tokio::test]
    async fn served_drc_synchronizes_exact_board_and_waits_before_save() {
        for tool in ["run_drc", "get_drc_violations"] {
            let dir = tempfile::tempdir().unwrap();
            let board = dir.path().join("clock.kicad_pcb");
            std::fs::write(&board, OLD).unwrap();
            let executable = cli_fixture(dir.path(), &board);
            let commands = Arc::new(Mutex::new(Vec::new()));
            let server = mock(&board, commands.clone(), "ok");
            let result = call(
                tool,
                &board,
                server.address(),
                &executable,
                json!({"sync_live_board":true,"refill_zones":true}),
            )
            .await;
            assert_eq!(result["isError"], false, "{result}");
            assert_eq!(result["source"], "saved_file");
            assert_eq!(result["live_board_synced"], true);
            assert_eq!(result["zone_refill_source"], "ipc");
            assert_eq!(
                std::fs::read_to_string(board.with_extension("kicad_pcb.observed")).unwrap(),
                LIVE
            );
            let seen = commands.lock().unwrap();
            let fill = seen
                .iter()
                .position(|s| s.ends_with("RefillZones"))
                .unwrap();
            let save = seen
                .iter()
                .position(|s| s.ends_with("SaveDocument"))
                .unwrap();
            assert!(
                save > fill + 2,
                "refill must be followed by busy/readiness observations: {seen:?}"
            );
            assert_eq!(
                seen.iter().filter(|s| s.ends_with("RefillZones")).count(),
                1
            );
            assert_eq!(
                seen.iter().filter(|s| s.ends_with("SaveDocument")).count(),
                1
            );
        }
    }

    #[tokio::test]
    async fn served_drc_refuses_wrong_target_save_failure_and_unproven_snapshot_before_cli() {
        for tool in ["run_drc", "get_drc_violations"] {
            for mode in ["wrong", "refill-fail", "save-fail", "stale"] {
                let dir = tempfile::tempdir().unwrap();
                let board = dir.path().join("clock.kicad_pcb");
                std::fs::write(&board, OLD).unwrap();
                let executable = cli_fixture(dir.path(), &board);
                let commands = Arc::new(Mutex::new(Vec::new()));
                let server = mock(&board, commands.clone(), mode);
                let result = call(
                    tool,
                    &board,
                    server.address(),
                    &executable,
                    json!({"sync_live_board":true,"refill_zones": mode == "refill-fail"}),
                )
                .await;
                assert_eq!(result["isError"], true, "{mode}: {result}");
                assert!(!board.with_extension("kicad_pcb.observed").exists());
                assert_eq!(std::fs::read_to_string(&board).unwrap(), OLD);
                if mode == "wrong" {
                    assert!(!commands
                        .lock()
                        .unwrap()
                        .iter()
                        .any(|s| s.ends_with("SaveDocument")));
                }
            }
        }
    }

    #[tokio::test]
    async fn served_file_only_drc_does_not_require_ipc_or_save() {
        for tool in ["run_drc", "get_drc_violations"] {
            let dir = tempfile::tempdir().unwrap();
            let board = dir.path().join("clock.kicad_pcb");
            std::fs::write(&board, OLD).unwrap();
            let executable = cli_fixture(dir.path(), &board);
            let result = call(tool, &board, "", &executable, json!({})).await;
            assert_eq!(result["isError"], false, "{result}");
            assert_eq!(result["live_board_synced"], false);
            assert_eq!(result["zones_refilled"], false);
            assert_eq!(std::fs::read_to_string(&board).unwrap(), OLD);
        }
    }

    #[tokio::test]
    async fn served_refill_waits_without_saving_and_refuses_wrong_board() {
        for mode in ["ok", "wrong"] {
            let dir = tempfile::tempdir().unwrap();
            let board = dir.path().join("clock.kicad_pcb");
            std::fs::write(&board, OLD).unwrap();
            let commands = Arc::new(Mutex::new(Vec::new()));
            let server = mock(&board, commands.clone(), mode);
            let result = call(
                "refill_zones",
                &board,
                server.address(),
                Path::new(""),
                json!({}),
            )
            .await;
            assert_eq!(result["isError"], mode == "wrong", "{result}");
            let seen = commands.lock().unwrap();
            assert!(!seen.iter().any(|s| s.ends_with("SaveDocument")));
            if mode == "ok" {
                assert_eq!(result["zones_refilled"], true);
                assert_eq!(result["saved"], false);
            }
            assert_eq!(std::fs::read_to_string(&board).unwrap(), OLD);
        }
    }

    #[test]
    fn readiness_deadline_does_not_replay_refill() {
        let dir = tempfile::tempdir().unwrap();
        let board = dir.path().join("clock.kicad_pcb");
        std::fs::write(&board, OLD).unwrap();
        let commands = Arc::new(Mutex::new(Vec::new()));
        let server = mock(&board, commands.clone(), "busy-forever");
        let client = konnect_ipc::KiCadIpcClient::new(server.address());
        client.find_open_board(&board).unwrap();
        client.refill_zones().unwrap();
        assert!(client.wait_for_board_ready(Duration::ZERO).is_err());
        assert_eq!(
            commands
                .lock()
                .unwrap()
                .iter()
                .filter(|s| s.ends_with("RefillZones"))
                .count(),
            1
        );
    }

    #[tokio::test]
    async fn served_drc_rejects_malformed_options_and_missing_ipc_without_writes() {
        for tool in ["run_drc", "get_drc_violations"] {
            for extra in [
                json!({"sync_live_board":"true"}),
                json!({"refill_zones":1}),
                json!({"sync_live_board":true}),
            ] {
                let dir = tempfile::tempdir().unwrap();
                let board = dir.path().join("clock.kicad_pcb");
                std::fs::write(&board, OLD).unwrap();
                let executable = cli_fixture(dir.path(), &board);
                let result = call(tool, &board, "", &executable, extra).await;
                assert_eq!(result["isError"], true, "{result}");
                assert_eq!(std::fs::read_to_string(&board).unwrap(), OLD);
                assert!(!board.with_extension("kicad_pcb.observed").exists());
            }
        }
    }

    #[test]
    fn snapshot_normalization_only_ignores_embedded_library_metadata() {
        let mut a = konnect_sexp::parse_sexp(OLD).unwrap();
        let mut b = a.clone();
        if let konnect_sexp::SexpNode::List(children) = &mut b {
            let footprint = children
                .iter_mut()
                .find(|node| node.head() == Some("footprint"))
                .unwrap();
            if let konnect_sexp::SexpNode::List(children) = footprint {
                children.push(konnect_sexp::parse_sexp("(version 20260206)").unwrap());
            }
        }
        normalize_snapshot_metadata(&mut a);
        normalize_snapshot_metadata(&mut b);
        assert_eq!(a, b);
        if let konnect_sexp::SexpNode::List(children) = &mut b {
            children
                .push(konnect_sexp::parse_sexp("(unknown_content must_not_be_ignored)").unwrap());
        }
        normalize_snapshot_metadata(&mut b);
        assert_ne!(a, b);
    }

    #[tokio::test]
    async fn served_drc_does_not_accept_changed_source_or_cli_failure_after_save() {
        for tool in ["run_drc", "get_drc_violations"] {
            for fail in [false, true] {
                let dir = tempfile::tempdir().unwrap();
                let board = dir.path().join("clock.kicad_pcb");
                std::fs::write(&board, OLD).unwrap();
                std::fs::write(board.with_extension("kicad_pcb.replacement"), OLD).unwrap();
                cli_fixture(dir.path(), &board);
                let executable = if fail {
                    cli::test_support::write_script(
                        dir.path(),
                        "failing-drc",
                        "#!/bin/sh\nexit 1\n",
                        "@exit /b 1\r\n",
                    )
                } else {
                    cli::test_support::write_script(dir.path(), "changing-drc",
                        "#!/bin/sh\nfor last; do :; done\ncp \"$last.replacement\" \"$last\"\nexit 0\n",
                        "@echo off\r\n:loop\r\nset last=%1\r\nshift\r\nif not \"%1\"==\"\" goto loop\r\ncopy /y %last%.replacement %last% >nul\r\nexit /b 0\r\n")
                };
                let commands = Arc::new(Mutex::new(Vec::new()));
                let server = mock(&board, commands.clone(), "ok");
                let result = call(
                    tool,
                    &board,
                    server.address(),
                    &executable,
                    json!({"sync_live_board":true}),
                )
                .await;
                assert_eq!(result["isError"], true, "{result}");
                assert_eq!(result["error"]["kind"], "mutation_outcome_uncertain");
                assert_eq!(
                    result["error"]["operation"],
                    if fail {
                        "drc_after_save"
                    } else {
                        "verify_drc_source"
                    }
                );
                assert_eq!(
                    commands
                        .lock()
                        .unwrap()
                        .iter()
                        .filter(|s| s.ends_with("SaveDocument"))
                        .count(),
                    1
                );
            }
        }
    }

    #[tokio::test]
    async fn served_report_failure_preserves_completed_sync_receipt_and_preflights_obvious_errors()
    {
        for tool in ["run_drc", "get_drc_violations"] {
            for preexisting in [true, false] {
                let dir = tempfile::tempdir().unwrap();
                let board = dir.path().join("clock.kicad_pcb");
                std::fs::write(&board, OLD).unwrap();
                let executable = cli_fixture(dir.path(), &board);
                let output = board.with_extension("report");
                if preexisting {
                    std::fs::create_dir(&output).unwrap();
                }
                let commands = Arc::new(Mutex::new(Vec::new()));
                let server = mock(&board, commands.clone(), "report-fail");
                let result = call(
                    tool,
                    &board,
                    server.address(),
                    &executable,
                    json!({"sync_live_board":true,"output":output.display().to_string()}),
                )
                .await;
                assert_eq!(result["isError"], true, "{result}");
                if preexisting {
                    assert_eq!(result["error"]["kind"], "invalid_argument");
                    assert!(commands.lock().unwrap().is_empty());
                    assert_eq!(std::fs::read_to_string(&board).unwrap(), OLD);
                } else {
                    assert_eq!(result["error"]["kind"], "handler_error");
                    assert_eq!(result["source_evidence"]["live_board_synced"], true);
                    assert_eq!(result["source_evidence"]["source"], "saved_file");
                    assert_eq!(std::fs::read_to_string(&board).unwrap(), LIVE);
                    assert_eq!(
                        commands
                            .lock()
                            .unwrap()
                            .iter()
                            .filter(|s| s.ends_with("SaveDocument"))
                            .count(),
                        1
                    );
                }
            }
        }
    }

    #[tokio::test]
    async fn report_output_cannot_replace_the_source_board() {
        for tool in ["run_drc", "get_drc_violations"] {
            let dir = tempfile::tempdir().unwrap();
            let board = dir.path().join("clock.kicad_pcb");
            std::fs::write(&board, OLD).unwrap();
            let executable = cli_fixture(dir.path(), &board);
            let commands = Arc::new(Mutex::new(Vec::new()));
            let server = mock(&board, commands.clone(), "ok");
            let result = call(
                tool,
                &board,
                server.address(),
                &executable,
                json!({"sync_live_board":true,"output":board.display().to_string()}),
            )
            .await;
            assert_eq!(result["isError"], true);
            assert_eq!(result["error"]["kind"], "invalid_argument");
            assert!(commands.lock().unwrap().is_empty());
            assert_eq!(std::fs::read_to_string(&board).unwrap(), OLD);
        }
    }

    #[tokio::test]
    async fn report_publication_does_not_truncate_a_hard_linked_board() {
        for tool in ["run_drc", "get_drc_violations"] {
            let dir = tempfile::tempdir().unwrap();
            let board = dir.path().join("clock.kicad_pcb");
            std::fs::write(&board, OLD).unwrap();
            let executable = cli_fixture(dir.path(), &board);
            let output = dir.path().join("report.json");
            std::fs::hard_link(&board, &output).unwrap();
            let result = call(
                tool,
                &board,
                "",
                &executable,
                json!({"output":output.display().to_string()}),
            )
            .await;
            assert_eq!(result["isError"], false, "{result}");
            assert_eq!(std::fs::read_to_string(&board).unwrap(), OLD);
            let report: Value =
                serde_json::from_str(&std::fs::read_to_string(&output).unwrap()).unwrap();
            assert!(report["violations"].as_array().unwrap().is_empty());
        }
    }

    /// Requires a disposable, already-open KiCad 10 board, never a working design.
    #[tokio::test]
    #[ignore = "requires disposable live KiCad board and KICAD_API_SOCKET"]
    async fn live_drc_drops_deleted_via_uuid_after_refill_and_save() {
        let board = std::path::PathBuf::from(
            std::env::var("KONNECT_LIVE_KICAD_BOARD").expect("disposable board path"),
        );
        let address = std::env::var("KICAD_API_SOCKET").expect("live IPC endpoint");
        let executable = std::path::PathBuf::from(
            std::env::var("KONNECT_TEST_KICAD_CLI").expect("CLI executable"),
        );
        let client = konnect_ipc::KiCadIpcClient::new(&address);
        client.find_open_board(&board).unwrap();
        let net = client
            .get_nets()
            .unwrap()
            .into_iter()
            .find(|net| !net.name.is_empty())
            .expect("named net");
        client.add_via(&net.name, 100.0, 120.0, 0.4, 0.8).unwrap();
        client.save_board().unwrap();
        let saved = std::fs::read_to_string(&board).unwrap();
        let tree = konnect_sexp::parse_sexp(&saved).unwrap();
        let via = tree
            .find_all("via")
            .into_iter()
            .find(|node| {
                node.find("at")
                    .is_some_and(|at| at.get_f64(1) == Some(100.0) && at.get_f64(2) == Some(120.0))
            })
            .expect("created via");
        let uuid = via
            .find("uuid")
            .unwrap()
            .get(1)
            .unwrap()
            .as_str()
            .unwrap()
            .to_string();
        client.delete_items(vec![uuid.clone()]).unwrap();
        // Add an unfilled native copper zone as a second observable outcome:
        // a refill acknowledgement alone must not pass the live regression.
        let corners: Vec<_> = tree
            .find_all("gr_line")
            .into_iter()
            .filter(|node| {
                node.find("layer")
                    .and_then(|layer| layer.get(1))
                    .and_then(konnect_sexp::SexpNode::as_str)
                    == Some("Edge.Cuts")
            })
            .flat_map(|node| [node.find("start"), node.find("end")])
            .flatten()
            .map(|point| (point.get_f64(1).unwrap(), point.get_f64(2).unwrap()))
            .collect();
        assert!(
            !corners.is_empty(),
            "native fixture must have a line-based board outline"
        );
        let left = corners.iter().map(|p| p.0).fold(f64::INFINITY, f64::min) + 0.5;
        let right = corners
            .iter()
            .map(|p| p.0)
            .fold(f64::NEG_INFINITY, f64::max)
            - 0.5;
        let top = corners.iter().map(|p| p.1).fold(f64::INFINITY, f64::min) + 0.5;
        let bottom = corners
            .iter()
            .map(|p| p.1)
            .fold(f64::NEG_INFINITY, f64::max)
            - 0.5;
        let points = [(left, top), (right, top), (right, bottom), (left, bottom)];
        let zone = konnect_ipc::builders::build_zone(
            &konnect_ipc::builders::ZoneSpec {
                layer: "F.Cu",
                net_name: &net.name,
                points: &points,
                holes: &[],
                clearance_mm: 0.3,
                min_thickness_mm: 0.25,
                name: "issue408-live-refill",
                priority: 1,
                connection: kiapi::board::types::ZoneConnectionStyle::ZcsFull,
            },
            net.netcode,
        );
        assert!(!zone.filled && zone.filled_polygons.is_empty());
        let created = client
            .create_items_returning(vec![konnect_ipc::builders::pack_any(
                &zone,
                "kiapi.board.types.Zone",
            )])
            .unwrap();
        let zone_id = created
            .iter()
            .map(|item| kiapi::board::types::Zone::decode(item.value.as_slice()).unwrap())
            .next()
            .unwrap()
            .id
            .unwrap()
            .value;
        assert!(
            std::fs::read_to_string(&board).unwrap().contains(&uuid),
            "deletion must still be unsaved for this regression"
        );
        let stale = call(
            "run_drc",
            &board,
            &address,
            &executable,
            json!({"severity":"info","limit":10000}),
        )
        .await;
        assert_eq!(stale["isError"], false, "{stale}");
        assert!(
            stale.to_string().contains(&uuid),
            "file-only control must actually report the stale UUID: {stale}"
        );
        for tool in ["run_drc", "get_drc_violations"] {
            let result = call(
                tool,
                &board,
                &address,
                &executable,
                json!({"sync_live_board":true,"refill_zones":true,"severity":"info"}),
            )
            .await;
            assert_eq!(result["isError"], false, "{result}");
            assert_eq!(result["live_board_synced"], true);
            assert_eq!(result["zones_refilled"], true);
            assert!(
                !result.to_string().contains(&uuid),
                "deleted UUID survived synchronized DRC"
            );
            assert!(!std::fs::read_to_string(&board).unwrap().contains(&uuid));
            let zones = client
                .get_items(kiapi::common::types::KiCadObjectType::KotPcbZone)
                .unwrap();
            let observed = zones
                .iter()
                .map(|item| kiapi::board::types::Zone::decode(item.value.as_slice()).unwrap())
                .find(|zone| zone.id.as_ref().is_some_and(|id| id.value == zone_id))
                .expect("created zone");
            assert!(
                observed.filled && !observed.filled_polygons.is_empty(),
                "refill must actually compute copper"
            );
        }
    }
}
