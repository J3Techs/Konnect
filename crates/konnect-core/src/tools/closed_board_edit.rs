//! `run_closed_board_edit`: run one closed-board tool against a board KiCad
//! currently holds, by taking the editor out of the way and bringing it back.
//!
//! A layout session alternates between tools that need the board open in
//! KiCad and tools that edit the saved file and need it closed. This tool
//! does the closing and reopening: save through IPC, ask the PCB editor to
//! close the way its close button does, wait for KiCad to release its lock,
//! run the named tool, start the standalone PCB editor on the board again and
//! wait until it answers.
//!
//! It adds no way around the closed-board gate. The named tool runs through
//! the same dispatch as a direct call and has to prove the board closed on
//! its own evidence; all this tool changes is the state of the world that
//! evidence describes. It never terminates a process and never deletes a
//! lock: an editor that will not close (it is showing a prompt) stops the
//! cycle with nothing edited.

mod desktop;

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

use serde_json::{json, Map, Value};

use crate::mcp::error::ToolErrorKind;
use crate::mcp::protocol::{CallToolResult, ToolContent};
use crate::tools::live_board::{self, EditorLock, LiveBoard};
use crate::tools::{
    get_path, invalid_arg, with_board_ipc_classified, with_bound_board_ipc_classified, BoardAccess,
    BoardBinding, ToolContext, ToolDef,
};
use desktop::{Desktop, EditorTarget};

const OPERATION: &str = "run_closed_board_edit";
const POLL: Duration = Duration::from_millis(250);
const DEFAULT_CLOSE_TIMEOUT_SECONDS: u64 = 30;
const DEFAULT_REOPEN_TIMEOUT_SECONDS: u64 = 90;

pub(crate) fn tool() -> ToolDef {
    crate::tool!(
        @shared "run_closed_board_edit",
        "Run one closed-board tool on a board KiCad has open, closing and reopening the PCB \
         editor around it. Steps: confirm KiCad holds exactly this board; save it through IPC \
         (unsaved editor changes are written to disk); ask the standalone PCB editor to close \
         the way its close button does; wait until it has exited and KiCad's lock is gone; run \
         'tool' with 'arguments' through the normal dispatch, where it must still prove the \
         board closed itself; start the standalone PCB editor on the board again and wait until \
         it answers over IPC. Returns each step with its timing and the nested tool's own \
         result under 'tool_result'. 'tool' must be a tool with a closed-board file path \
         (board access closed-only, live-preferred with file fallback, or apply-mode \
         dependent), with its toolset loaded. If the board is not open, the tool is simply run. \
         Never forces anything: if the editor does not close in time (for example it is showing \
         a prompt), nothing is edited and the result says what to do; if the nested tool \
         refuses or fails, the editor is still reopened and both are reported; if reopening \
         fails, the edit stands and the result says to open the editor by hand. Refuses, \
         changing nothing, when the PCB editor is a window of the KiCad project manager rather \
         than a standalone PCB editor. A dry run and its apply in two separate calls only work \
         if KiCad's save in between leaves the file byte-identical; use reopen 'never' on the \
         dry run, apply directly, then pass reopen 'always' on the last edit. Tested on Windows \
         with KiCad 10.0.7; the Linux (wmctrl) and macOS (osascript) window handling is \
         untested.",
        json!({
            "type": "object",
            "properties": {
                "board": { "type": "string", "description": "Path to the .kicad_pcb file" },
                "tool": { "type": "string", "minLength": 1, "description": "Name of the closed-board tool to run, for example set_design_rules" },
                "arguments": {
                    "type": "object",
                    "additionalProperties": true,
                    "description": "Arguments for 'tool'. 'board' is filled in from this call; if given it must name the same board"
                },
                "reopen": {
                    "type": "string",
                    "enum": ["if_was_open", "always", "never"],
                    "default": "if_was_open",
                    "description": "Whether to start the standalone PCB editor afterwards: only if this call closed it, always, or never"
                },
                "close_timeout_seconds": { "type": "integer", "minimum": 5, "maximum": 600, "default": DEFAULT_CLOSE_TIMEOUT_SECONDS, "description": "How long to wait for the editor to exit and release its lock" },
                "reopen_timeout_seconds": { "type": "integer", "minimum": 5, "maximum": 600, "default": DEFAULT_REOPEN_TIMEOUT_SECONDS, "description": "How long to wait for the reopened editor to answer over IPC" }
            },
            "required": ["board", "tool"]
        }),
        |args, ctx| async move { handle_run_closed_board_edit(args, ctx).await }
    )
}

/// Whether `access` describes a tool with a saved-file path that needs the
/// board closed: the tools this one exists to serve.
pub(crate) fn has_closed_board_path(access: BoardAccess) -> bool {
    match access {
        BoardAccess::ClosedBoardOnly
        | BoardAccess::LivePreferredWithFallback
        | BoardAccess::ApplyModeDependent => true,
        BoardAccess::LiveOnly | BoardAccess::None => false,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Reopen {
    IfWasOpen,
    Always,
    Never,
}

pub(crate) struct CycleRequest {
    pub reopen: Reopen,
    pub close_polls: u32,
    pub reopen_polls: u32,
}

/// What KiCad said about the board before anything was changed.
pub(crate) enum Held {
    /// KiCad holds exactly this board. Carries the path it reports for its
    /// own standalone PCB editor, when it answered that.
    Open { pcb_editor_binary: Option<String> },
    /// No editor at the IPC endpoint holds it: KiCad was not reached, holds
    /// other boards, or has no PCB editor there. There is nothing to close,
    /// and the nested tool's own gate decides whether that is enough.
    NotOpen {
        reason: &'static str,
        detail: String,
    },
    /// KiCad answered without settling whether it holds the board (it is
    /// busy, timed out, or lost a board this server saw open). The nested
    /// tool's gate would refuse on the same answer, so the cycle stops.
    Unknown {
        reason: &'static str,
        detail: String,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum LockState {
    Absent,
    Present(String),
    Unreadable(String),
}

pub(crate) struct Launched {
    pub pid: u32,
    pub binary: String,
}

/// Everything the cycle does to the world, so its order and its failure
/// handling can be tested without KiCad or a desktop.
pub(crate) trait Stage {
    async fn probe(&self) -> anyhow::Result<Held>;
    async fn save(&self) -> Result<(), String>;
    fn find_editor(&self) -> Result<EditorTarget, String>;
    fn request_close(&self, editor: &EditorTarget) -> Result<(), String>;
    fn editor_open(&self, editor: &EditorTarget) -> bool;
    fn blocking_dialogs(&self, editor: &EditorTarget) -> Vec<String>;
    fn lock(&self) -> LockState;
    async fn run_tool(&self) -> CallToolResult;
    fn launch(&self, pcb_editor_binary: Option<&str>) -> Result<Launched, String>;
    async fn addressable(&self) -> Result<(), String>;
    /// Whether this server has an IPC address at all. It resolves one when it
    /// starts and never again, so a server started with no KiCad running
    /// cannot reach an editor started later.
    fn ipc_configured(&self) -> bool;
    /// Another IPC endpoint the launched editor may be listening on instead
    /// of the one this server uses.
    fn other_endpoint(&self, pid: u32) -> Option<String>;
    async fn pause(&self);
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Status {
    /// The nested tool ran without error and the editor is as requested.
    Complete,
    /// The nested tool ran without error, but the editor could not be
    /// reopened or did not answer in time.
    CompleteEditorNotReopened,
    /// The nested tool returned an error. The editor was dealt with as
    /// requested; `editor_reopened` says how that went.
    ToolFailed,
    /// Whether KiCad holds the board could not be established. Nothing was
    /// changed and the nested tool was not run.
    BoardStateUnknown,
    /// KiCad holds the board, but its editor could not be closed by this
    /// tool. Nothing was changed.
    EditorNotClosable,
    /// KiCad would not save the board. Nothing was changed.
    SaveFailed,
    /// The board was saved and the editor asked to close, and it is still
    /// open or its lock is still there. Nothing was edited.
    EditorNotClosed,
}

impl Status {
    pub(crate) fn code(self) -> &'static str {
        match self {
            Self::Complete => "complete",
            Self::CompleteEditorNotReopened => "complete_editor_not_reopened",
            Self::ToolFailed => "tool_failed",
            Self::BoardStateUnknown => "board_state_unknown",
            Self::EditorNotClosable => "editor_not_closable",
            Self::SaveFailed => "save_failed",
            Self::EditorNotClosed => "editor_not_closed",
        }
    }
}

pub(crate) struct Cycle {
    pub status: Status,
    pub message: String,
    pub next_action: Option<String>,
    pub steps: Vec<Value>,
    pub editor_was_open: bool,
    pub editor: Option<Value>,
    pub board_saved: bool,
    pub tool_result: Option<CallToolResult>,
    /// `None` when no reopen was called for.
    pub editor_reopened: Option<bool>,
}

struct Steps(Vec<Value>);

impl Steps {
    fn push(&mut self, step: &str, outcome: &str, started: Instant, detail: Value) {
        let mut entry = Map::new();
        entry.insert("step".into(), json!(step));
        entry.insert("outcome".into(), json!(outcome));
        entry.insert(
            "elapsed_ms".into(),
            json!(started.elapsed().as_millis() as u64),
        );
        if let Value::Object(detail) = detail {
            entry.extend(detail);
        }
        self.0.push(Value::Object(entry));
    }
}

fn manual_route(tool: &str) -> String {
    format!(
        "Save and close the board in KiCad yourself, call {tool} directly, then open the board \
         again."
    )
}

/// Start the editor and wait for it to answer. `Ok` once it is addressable;
/// `Err` with what the user has to do otherwise.
async fn reopen<S: Stage>(
    stage: &S,
    steps: &mut Steps,
    pcb_editor_binary: Option<&str>,
    polls: u32,
) -> Result<(), String> {
    let started = Instant::now();
    let launched = match stage.launch(pcb_editor_binary) {
        Ok(launched) => launched,
        Err(error) => {
            steps.push(
                "reopen_editor",
                "failed",
                started,
                json!({ "error": error.clone() }),
            );
            return Err(format!(
                "The PCB editor could not be started ({error}). Open the board in KiCad by hand."
            ));
        }
    };
    steps.push(
        "reopen_editor",
        "launched",
        started,
        json!({ "pid": launched.pid, "binary": launched.binary }),
    );
    let started = Instant::now();
    if !stage.ipc_configured() {
        let endpoint = stage.other_endpoint(launched.pid);
        steps.push(
            "wait_addressable",
            "ipc_not_configured",
            started,
            json!({ "editor_endpoint": endpoint.clone() }),
        );
        return Err(format!(
            "The PCB editor was started (pid {}), but this Konnect server found no KiCad IPC              endpoint when it started and does not look again{}. Restart the Konnect server              (reconnect the MCP client) before calling a live-board tool.",
            launched.pid,
            endpoint
                .map(|endpoint| format!("; KiCad now listens on '{endpoint}'"))
                .unwrap_or_default()
        ));
    }
    let mut last = String::new();
    for _ in 0..polls {
        match stage.addressable().await {
            Ok(()) => {
                steps.push("wait_addressable", "addressable", started, Value::Null);
                return Ok(());
            }
            Err(error) => last = error,
        }
        stage.pause().await;
    }
    let elsewhere = stage.other_endpoint(launched.pid);
    steps.push(
        "wait_addressable",
        "timed_out",
        started,
        json!({ "last_answer": last.clone(), "editor_endpoint": elsewhere.clone() }),
    );
    Err(match elsewhere {
        Some(endpoint) => format!(
            "The PCB editor was started (pid {}) but answers on '{endpoint}', not on the IPC \
             address this Konnect server resolved when it started, most likely because another \
             KiCad instance owns that address. Close the other instance and reopen the board, \
             or restart Konnect with KICAD_API_SOCKET set to that endpoint.",
            launched.pid
        ),
        None => format!(
            "The PCB editor was started (pid {}) but this board did not become addressable over \
             IPC in time (last answer: {last}). It may still be loading, or another KiCad \
             instance may own the IPC address; check the editor before calling a live-board \
             tool.",
            launched.pid
        ),
    })
}

pub(crate) async fn run_cycle<S: Stage>(
    stage: &S,
    tool: &str,
    request: &CycleRequest,
) -> anyhow::Result<Cycle> {
    let mut steps = Steps(Vec::new());
    let mut cycle = Cycle {
        status: Status::Complete,
        message: String::new(),
        next_action: None,
        steps: Vec::new(),
        editor_was_open: false,
        editor: None,
        board_saved: false,
        tool_result: None,
        editor_reopened: None,
    };

    let started = Instant::now();
    let pcb_editor_binary = match stage.probe().await? {
        Held::NotOpen { reason, detail } => {
            steps.push(
                "confirm_open",
                "board_not_open",
                started,
                json!({ "reason": reason, "detail": detail }),
            );
            None
        }
        Held::Unknown { reason, detail } => {
            steps.push(
                "confirm_open",
                "unknown",
                started,
                json!({ "reason": reason, "detail": detail.clone() }),
            );
            cycle.status = Status::BoardStateUnknown;
            cycle.message = format!(
                "KiCad did not settle whether it holds this board ({detail}). {tool} would                  refuse on the same answer, so it was not run and nothing was changed."
            );
            cycle.next_action = Some(format!(
                "Check KiCad: finish what it is doing or answer its dialog, and make sure the                  board is either open and responsive or closed. Then call {OPERATION} again."
            ));
            cycle.steps = steps.0;
            return Ok(cycle);
        }
        Held::Open { pcb_editor_binary } => {
            steps.push("confirm_open", "board_open", started, Value::Null);
            cycle.editor_was_open = true;

            // Everything that can refuse is asked before anything is changed.
            let started = Instant::now();
            let editor = match stage.find_editor() {
                Ok(editor) => editor,
                Err(error) => {
                    steps.push(
                        "find_editor",
                        "not_found",
                        started,
                        json!({ "error": error.clone() }),
                    );
                    cycle.status = Status::EditorNotClosable;
                    cycle.message = format!(
                        "KiCad holds this board, but Konnect could not identify its PCB editor \
                         window: {error}. Nothing was changed."
                    );
                    cycle.next_action = Some(manual_route(tool));
                    cycle.steps = steps.0;
                    return Ok(cycle);
                }
            };
            let description = json!({
                "pid": editor.window.pid,
                "process": editor.window.process,
                "window_title": editor.window.title,
                "standalone": editor.standalone,
            });
            cycle.editor = Some(description.clone());
            if !editor.standalone {
                steps.push(
                    "find_editor",
                    "inside_project_manager",
                    started,
                    description,
                );
                cycle.status = Status::EditorNotClosable;
                cycle.message = format!(
                    "The PCB editor holding this board is a window of the KiCad project manager \
                     (pid {}), not a standalone PCB editor. The project manager cannot be asked \
                     over IPC to open its PCB editor again, and a standalone editor started \
                     beside the project manager is not reachable over IPC, so the board could \
                     not be reopened after the edit. Nothing was changed.",
                    editor.window.pid
                );
                cycle.next_action = Some(format!(
                    "Quit KiCad (save first), open the board in the standalone PCB editor \
                     (pcbnew <board>) and call {OPERATION} again; or save and close the PCB \
                     editor window and call {tool} directly; if it still refuses, quit the \
                     project manager too and call it again."
                ));
                cycle.steps = steps.0;
                return Ok(cycle);
            }
            steps.push("find_editor", "found", started, description);

            let started = Instant::now();
            if let Err(error) = stage.save().await {
                steps.push("save", "failed", started, json!({ "error": error.clone() }));
                cycle.status = Status::SaveFailed;
                cycle.message = format!(
                    "KiCad did not save the board ({error}), so the editor was left open and \
                     nothing was changed. KiCad answers like this while it is busy or showing a \
                     dialog."
                );
                cycle.next_action = Some(format!(
                    "Finish what KiCad is doing or answer its dialog, then call {OPERATION} \
                     again."
                ));
                cycle.steps = steps.0;
                return Ok(cycle);
            }
            steps.push("save", "saved", started, Value::Null);
            cycle.board_saved = true;

            let started = Instant::now();
            if let Err(error) = stage.request_close(&editor) {
                steps.push(
                    "close_editor",
                    "failed",
                    started,
                    json!({ "error": error.clone() }),
                );
                cycle.status = Status::EditorNotClosed;
                cycle.message = format!(
                    "The board was saved, but the PCB editor could not be asked to close \
                     ({error}). Nothing was edited."
                );
                cycle.next_action = Some(manual_route(tool));
                cycle.steps = steps.0;
                return Ok(cycle);
            }
            steps.push("close_editor", "requested", started, Value::Null);

            let started = Instant::now();
            let mut closed = false;
            for _ in 0..request.close_polls {
                if !stage.editor_open(&editor) && stage.lock() == LockState::Absent {
                    closed = true;
                    break;
                }
                stage.pause().await;
            }
            if !closed {
                let open = stage.editor_open(&editor);
                let dialogs = if open {
                    stage.blocking_dialogs(&editor)
                } else {
                    Vec::new()
                };
                let lock = stage.lock();
                steps.push(
                    "wait_closed",
                    "timed_out",
                    started,
                    json!({
                        "editor_still_open": open,
                        "dialogs": dialogs,
                        "kicad_sibling_lock": match &lock {
                            LockState::Absent => "absent",
                            LockState::Present(_) => "present",
                            LockState::Unreadable(_) => "unreadable",
                        },
                    }),
                );
                cycle.status = Status::EditorNotClosed;
                (cycle.message, cycle.next_action) = if open {
                    let showing = if dialogs.is_empty() {
                        String::new()
                    } else {
                        format!(
                            " It is showing: {}.",
                            dialogs
                                .iter()
                                .map(|title| format!("'{title}'"))
                                .collect::<Vec<_>>()
                                .join(", ")
                        )
                    };
                    (
                        format!(
                            "The board was saved and the PCB editor was asked to close, but it \
                             is still open.{showing} Konnect did not force it and edited \
                             nothing."
                        ),
                        Some(format!(
                            "Answer the dialog in KiCad or close the editor yourself. The close \
                             request cannot be withdrawn, so the editor may close as soon as \
                             the dialog is answered. Once it is closed, call {tool} directly \
                             and reopen the board."
                        )),
                    )
                } else {
                    let lock = match lock {
                        LockState::Present(path) => {
                            format!("its lock file '{path}' is still there")
                        }
                        LockState::Unreadable(detail) => {
                            format!("its lock file could not be inspected ({detail})")
                        }
                        LockState::Absent => "its lock was released too late".to_string(),
                    };
                    (
                        format!(
                            "The PCB editor exited, but {lock}. Konnect never deletes a KiCad \
                             lock, and edited nothing."
                        ),
                        Some(
                            "Open the board in KiCad, check that nothing was lost, and close \
                             it normally so KiCad removes its own lock; then retry."
                                .to_string(),
                        ),
                    )
                };
                cycle.steps = steps.0;
                return Ok(cycle);
            }
            steps.push("wait_closed", "closed", started, Value::Null);
            pcb_editor_binary
        }
    };

    let started = Instant::now();
    let result = stage.run_tool().await;
    let tool_failed = result.is_error;
    steps.push(
        "run_tool",
        if tool_failed { "error" } else { "ok" },
        started,
        json!({ "tool": tool }),
    );
    cycle.tool_result = Some(result);

    let wanted = match request.reopen {
        Reopen::Always => true,
        Reopen::IfWasOpen => cycle.editor_was_open,
        Reopen::Never => false,
    };
    let reopen_failure = if wanted {
        let outcome = reopen(
            stage,
            &mut steps,
            pcb_editor_binary.as_deref(),
            request.reopen_polls,
        )
        .await;
        cycle.editor_reopened = Some(outcome.is_ok());
        outcome.err()
    } else {
        None
    };

    let editor_note = match (cycle.editor_was_open, cycle.editor_reopened) {
        (true, Some(true)) => "The PCB editor was closed for it and is open on the board again.",
        (true, Some(false)) => "The PCB editor was closed for it.",
        (true, None) => "The PCB editor was closed for it and left closed, as requested.",
        (false, Some(true)) => {
            "KiCad did not hold the board, so no editor was closed; the PCB editor was started \
             afterwards."
        }
        (false, _) => "KiCad did not hold the board, so no editor was closed.",
    };
    cycle.status = match (tool_failed, &reopen_failure) {
        (true, _) => Status::ToolFailed,
        (false, Some(_)) => Status::CompleteEditorNotReopened,
        (false, None) => Status::Complete,
    };
    cycle.message = if tool_failed {
        format!("{tool} returned an error; see tool_result. {editor_note}")
    } else {
        format!("{tool} ran. {editor_note}")
    };
    cycle.next_action = reopen_failure;
    cycle.steps = steps.0;
    Ok(cycle)
}

// ─── The real stage ───────────────────────────────────────────────────────────

struct LiveStage {
    ctx: Arc<ToolContext>,
    board: PathBuf,
    tool: ToolDef,
    arguments: Value,
    desktop: Box<dyn Desktop>,
}

impl Stage for LiveStage {
    async fn probe(&self) -> anyhow::Result<Held> {
        let not_open = |reason, detail: String| Held::NotOpen { reason, detail };
        let unknown = |reason, detail: String| Held::Unknown { reason, detail };
        Ok(
            match live_board::observe(&self.ctx, &self.board, |client, _| {
                Ok(client.get_kicad_binary_path("pcbnew").ok())
            })
            .await?
            {
                LiveBoard::Answered(pcb_editor_binary) => Held::Open {
                    pcb_editor_binary: pcb_editor_binary.filter(|path| !path.is_empty()),
                },
                LiveBoard::NotOpen { message, .. } => not_open("board_not_open", message),
                LiveBoard::NeverReached(message) => not_open("ipc_unreachable", message),
                LiveBoard::Unserved {
                    absence, message, ..
                } => not_open(absence.reason_code(), message),
                LiveBoard::Rejected(message) => unknown("ipc_rejected", message),
                LiveBoard::Uncertain(message) => unknown("ipc_outcome_unknown", message),
                LiveBoard::Recovered(message) => unknown("ipc_recovered", message),
                LiveBoard::Unresolved(error) => unknown("board_not_resolved", error.to_string()),
                LiveBoard::LostAfterObservation { situation, .. } => {
                    unknown(live_board::PREVIOUSLY_OBSERVED_LIVE, situation.to_string())
                }
            },
        )
    }

    async fn save(&self) -> Result<(), String> {
        match with_board_ipc_classified(&self.ctx, &self.board, |client, document| {
            konnect_ipc::retry_while_busy(Duration::from_secs(10), || {
                client.save_board_in(document.clone())
            })
        })
        .await
        {
            Ok(Ok(())) => Ok(()),
            Ok(Err(failure)) => Err(failure.message().to_string()),
            Err(error) => Err(format!("{error:#}")),
        }
    }

    fn find_editor(&self) -> Result<EditorTarget, String> {
        desktop::select_pcb_editor(&self.desktop.kicad_windows()?, &self.board)
    }

    fn request_close(&self, editor: &EditorTarget) -> Result<(), String> {
        self.desktop.request_close(&editor.window)
    }

    fn editor_open(&self, editor: &EditorTarget) -> bool {
        desktop::editor_still_open(self.desktop.as_ref(), editor)
    }

    fn blocking_dialogs(&self, editor: &EditorTarget) -> Vec<String> {
        self.desktop
            .kicad_windows()
            .map(|windows| desktop::blocking_dialogs(&windows, &editor.window))
            .unwrap_or_default()
    }

    fn lock(&self) -> LockState {
        match live_board::editor_lock(&self.board) {
            EditorLock::Absent => LockState::Absent,
            EditorLock::Present(path) => LockState::Present(path.display().to_string()),
            EditorLock::Unreadable(path, error) => {
                LockState::Unreadable(format!("{}: {error}", path.display()))
            }
        }
    }

    async fn run_tool(&self) -> CallToolResult {
        crate::mcp::handler::run_loaded_tool(
            &self.tool,
            self.tool.name,
            &self.arguments,
            self.ctx.clone(),
        )
        .await
        .0
    }

    fn launch(&self, pcb_editor_binary: Option<&str>) -> Result<Launched, String> {
        let binary = pcb_editor_binary
            .map(PathBuf::from)
            .filter(|path| path.is_file())
            .or_else(|| {
                crate::kicad_install::find_pcb_editor(
                    &self.ctx.config.kicad_binary,
                    &self.ctx.config.kicad_cli,
                )
            })
            .ok_or_else(|| {
                "no standalone PCB editor (pcbnew) was found in the KiCad installation".to_string()
            })?;
        let pid = self.desktop.launch(&binary, &self.board)?;
        Ok(Launched {
            pid,
            binary: binary.display().to_string(),
        })
    }

    async fn addressable(&self) -> Result<(), String> {
        match with_bound_board_ipc_classified(&self.ctx, &self.board, |_, _| Ok(())).await {
            Ok(Ok(BoardBinding::Bound(()))) => Ok(()),
            Ok(Ok(BoardBinding::Unserved { message, .. })) => Err(message),
            Ok(Err(failure)) => Err(failure.message().to_string()),
            Err(error) => Err(format!("{error:#}")),
        }
    }

    fn ipc_configured(&self) -> bool {
        !self.ctx.config.ipc_address.is_empty()
    }

    fn other_endpoint(&self, pid: u32) -> Option<String> {
        // Looks only: nothing is dialled.
        if !self.ipc_configured() {
            return konnect_ipc::detect_ipc_address();
        }
        // KiCad names its socket `api-<pid>.sock` when `api.sock` is taken.
        konnect_ipc::candidate_socket_paths()
            .into_iter()
            .filter_map(|default| Some(default.parent()?.join(format!("api-{pid}.sock"))))
            .find(|path| path.exists())
            .map(|path| format!("ipc://{}", path.display()))
            .filter(|address| *address != self.ctx.config.ipc_address)
    }

    async fn pause(&self) {
        tokio::time::sleep(POLL).await;
    }
}

// ─── Handler ──────────────────────────────────────────────────────────────────

fn same_board(a: &Path, b: &Path) -> bool {
    let canonical = |path: &Path| path.canonicalize().unwrap_or_else(|_| path.to_path_buf());
    canonical(a) == canonical(b)
}

fn seconds(args: &Value, key: &str, default: u64) -> Result<u64, CallToolResult> {
    match args.get(key) {
        None | Some(Value::Null) => Ok(default),
        Some(value) => value
            .as_u64()
            .filter(|seconds| (5..=600).contains(seconds))
            .ok_or_else(|| invalid_arg(key, "must be an integer from 5 to 600")),
    }
}

fn polls(seconds: u64) -> u32 {
    (seconds * 1000 / POLL.as_millis() as u64) as u32
}

/// The registered definition of `name`, whether or not its toolset is loaded.
fn registered_tool(ctx: &ToolContext, name: &str) -> Option<(&'static str, ToolDef)> {
    let toolset = ctx.router.find_toolset_for_tool(name)?;
    let tool = crate::router::registry::tools_for(toolset)?
        .into_iter()
        .find(|tool| tool.name == name)?;
    Some((toolset, tool))
}

/// Resolve the nested tool and its arguments, refusing everything that can
/// be refused before KiCad is touched.
async fn nested_call(
    ctx: &ToolContext,
    board: &Path,
    args: &Value,
) -> Result<(ToolDef, Value), CallToolResult> {
    let name = crate::tools::require_str(args, "tool")?;
    let Some((toolset, registered)) = registered_tool(ctx, name) else {
        return Err(invalid_arg(
            "tool",
            &format!("'{name}' is not a Konnect tool"),
        ));
    };
    if !has_closed_board_path(registered.board_access) {
        let why = match registered.board_access {
            BoardAccess::LiveOnly => "it works only on a board open in KiCad, so call it directly",
            _ => "it declares no closed-board file path, so call it directly",
        };
        return Err(invalid_arg(
            "tool",
            &format!("'{name}' cannot be run by {OPERATION}: {why}"),
        ));
    }
    let mut tool = ctx.router.get_tool(name).await;
    if tool.is_none() && ctx.config.auto_load_toolsets {
        ctx.router.load(toolset).await;
        tool = ctx.router.get_tool(name).await;
    }
    let Some(tool) = tool else {
        return Err(CallToolResult::error_kind(
            ToolErrorKind::ToolsetNotLoaded {
                toolset: toolset.to_string(),
                tool: name.to_string(),
            },
            format!(
                "Tool '{name}' is in toolset '{toolset}' which is not currently loaded. Call \
                 load_toolset('{toolset}') first, then retry. Nothing was changed."
            ),
        ));
    };

    let mut arguments = match args.get("arguments") {
        None | Some(Value::Null) => Map::new(),
        Some(Value::Object(arguments)) => arguments.clone(),
        Some(_) => return Err(invalid_arg("arguments", "must be an object")),
    };
    if tool.input_schema["properties"].get("board").is_none() {
        return Err(invalid_arg(
            "tool",
            &format!("'{name}' takes no 'board' argument"),
        ));
    }
    match arguments.get("board") {
        None | Some(Value::Null) => {
            arguments.insert("board".into(), json!(board.to_string_lossy()));
        }
        Some(Value::String(nested)) if same_board(Path::new(nested), board) => {}
        Some(_) => {
            return Err(invalid_arg(
                "arguments.board",
                "must name the same board as 'board', or be omitted",
            ))
        }
    }
    let arguments = Value::Object(arguments);
    if let Err(error) = tool.input_validator.validate(&arguments) {
        return Err(invalid_arg(
            "arguments",
            &format!("not valid for {name}: {error}. Nothing was changed."),
        ));
    }
    Ok((tool, arguments))
}

/// The nested tool's result as JSON where it is JSON, and as text otherwise.
fn tool_result_json(result: &CallToolResult) -> Value {
    let body = result
        .content
        .iter()
        .find_map(|content| match content {
            ToolContent::Text { text } => Some(
                serde_json::from_str::<Value>(text).unwrap_or_else(|_| Value::String(text.clone())),
            ),
            _ => None,
        })
        .unwrap_or(Value::Null);
    json!({ "is_error": result.is_error, "body": body })
}

fn report(board: &Path, tool: &str, cycle: &Cycle) -> CallToolResult {
    let mut body = json!({
        "status": cycle.status.code(),
        "message": cycle.message,
        "board": board.display().to_string(),
        "tool": tool,
        "editor_was_open": cycle.editor_was_open,
        "editor": cycle.editor,
        "board_saved": cycle.board_saved,
        "tool_ran": cycle.tool_result.is_some(),
        "tool_result": cycle.tool_result.as_ref().map(tool_result_json),
        "editor_reopened": cycle.editor_reopened,
        "next_action": cycle.next_action,
        "steps": cycle.steps,
    });
    let is_error = match cycle.status {
        Status::Complete | Status::CompleteEditorNotReopened => false,
        Status::ToolFailed => {
            // The nested tool's own structured error is the error of this
            // call, so a client matching on kinds sees the same one.
            body["error"] = body["tool_result"]["body"]
                .get("error")
                .filter(|error| error.get("kind").is_some())
                .cloned()
                .unwrap_or_else(|| {
                    json!({
                        "kind": "handler_error",
                        "reason": format!("{tool} returned an error"),
                    })
                });
            true
        }
        Status::BoardStateUnknown
        | Status::EditorNotClosable
        | Status::SaveFailed
        | Status::EditorNotClosed => {
            body["error"] = json!({
                "kind": "plan_blocked",
                "operation": OPERATION,
                "reasons": [cycle.status.code()],
            });
            true
        }
    };
    CallToolResult {
        content: vec![ToolContent::Text {
            text: body.to_string(),
        }],
        is_error,
    }
}

async fn handle_run_closed_board_edit(
    args: &Value,
    ctx: Arc<ToolContext>,
) -> anyhow::Result<CallToolResult> {
    let board = get_path(args, "board")?;
    let reopen = match args.get("reopen").and_then(Value::as_str) {
        None | Some("if_was_open") => Reopen::IfWasOpen,
        Some("always") => Reopen::Always,
        Some("never") => Reopen::Never,
        Some(_) => {
            return Ok(invalid_arg(
                "reopen",
                "must be 'if_was_open', 'always' or 'never'",
            ))
        }
    };
    let (close_seconds, reopen_seconds) = match (
        seconds(args, "close_timeout_seconds", DEFAULT_CLOSE_TIMEOUT_SECONDS),
        seconds(
            args,
            "reopen_timeout_seconds",
            DEFAULT_REOPEN_TIMEOUT_SECONDS,
        ),
    ) {
        (Ok(close), Ok(reopen)) => (close, reopen),
        (Err(refusal), _) | (_, Err(refusal)) => return Ok(refusal),
    };
    let (tool, arguments) = match nested_call(&ctx, &board, args).await {
        Ok(call) => call,
        Err(refusal) => return Ok(refusal),
    };
    let name = tool.name;
    let stage = LiveStage {
        ctx,
        board: board.clone(),
        tool,
        arguments,
        desktop: desktop::system(),
    };
    let cycle = run_cycle(
        &stage,
        name,
        &CycleRequest {
            reopen,
            close_polls: polls(close_seconds),
            reopen_polls: polls(reopen_seconds),
        },
    )
    .await?;
    Ok(report(&board, name, &cycle))
}

#[cfg(test)]
mod tests;
