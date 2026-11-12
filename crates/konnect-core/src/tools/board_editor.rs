//! `open_board_editor` and `close_board_editor`: put a board into KiCad's
//! standalone PCB editor, and take it out again.
//!
//! `launch_kicad_ui` starts the project manager, where a board is opened by a
//! click Konnect cannot make. These two tools cover what an agent needs
//! instead: a live editor on one named board, and that editor gone again so
//! the tools that edit the saved file can run.
//!
//! Opening never starts a second editor on a board some reachable KiCad
//! already holds, and never starts one beside KiCad's lock for the board: a
//! lock with no reachable editor behind it belongs to an editor Konnect
//! cannot see, or to one that crashed, and KiCad would answer the launch with
//! a prompt nobody is there to read.
//!
//! Closing is only for an editor this server started. Any other editor is the
//! user's, and is left to the user. The close itself is the request the
//! editor's own close button sends (see `closed_board_edit::desktop`), so
//! KiCad runs its normal close path and removes its lock; nothing is ever
//! killed and no lock is ever deleted. An editor that does not go away, for
//! instance because it is asking about unsaved changes, is reported and left
//! as it is.

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde_json::{json, Value};

use crate::mcp::protocol::{CallToolResult, ToolContent};
use crate::tools::closed_board_edit::desktop::{self, Desktop};
use crate::tools::closed_board_edit::{polls, seconds, Launched, LockState};
use crate::tools::live_board::{self, EditorLock};
use crate::tools::{
    get_path, with_board_ipc_classified, with_bound_board_ipc_classified, BoardBinding,
    BoardEditorAbsence, ToolContext, ToolDef,
};

const OPEN: &str = "open_board_editor";
const CLOSE: &str = "close_board_editor";
const POLL: Duration = Duration::from_millis(250);
const DEFAULT_OPEN_TIMEOUT_SECONDS: u64 = 90;
const DEFAULT_CLOSE_TIMEOUT_SECONDS: u64 = 30;

pub(crate) fn open_tool() -> ToolDef {
    crate::tool!(
        "open_board_editor",
        "Open a board in KiCad's standalone PCB editor and wait until it answers over IPC, \
         so live-board tools can work on it without the user opening it by hand. If a \
         reachable KiCad already holds this board, nothing is started and the result says \
         so (status 'already_open'). Otherwise the PCB editor of the configured KiCad \
         installation is started on the board file; the result carries its process id and \
         the IPC endpoint it answers at (status 'opened'). Refuses, starting nothing, when \
         KiCad's lock file for the board is present although no reachable KiCad holds it: \
         an editor Konnect cannot reach, or one that crashed, owns the board, and the user \
         has to deal with it. An editor started here can be closed again with \
         close_board_editor. Not yet run against KiCad on any platform; the launch is the \
         one run_closed_board_edit's reopen step uses.",
        json!({
            "type": "object",
            "properties": {
                "board": { "type": "string", "description": "Path to the .kicad_pcb file" },
                "timeout_seconds": { "type": "integer", "minimum": 5, "maximum": 600, "default": DEFAULT_OPEN_TIMEOUT_SECONDS, "description": "How long to wait for the started editor to answer over IPC with the board open" }
            },
            "required": ["board"]
        }),
        |args, ctx| async move { handle_open_board_editor(args, ctx).await }
    )
}

pub(crate) fn close_tool() -> ToolDef {
    crate::tool!(
        "close_board_editor",
        "Close the standalone PCB editor that open_board_editor (or run_closed_board_edit's \
         reopen) started on a board in this server session, so closed-board tools can edit \
         the saved file. With 'save' (default true) the board is first saved through IPC. \
         The editor is then asked to close the way its close button does, and the call \
         waits until its process has exited and KiCad has removed its lock file. After \
         that (status 'closed') closed-board tools accept the board in this session. Only \
         an editor this server started is closed: for any other editor the call refuses \
         and the user has to close it. Never forces anything: an editor that stays open \
         (it is showing a prompt, for example about unsaved changes when 'save' is false) \
         is reported with status 'editor_not_closed' and left alone. On macOS the close \
         request goes to the PCB editor application, not to one process, so while another \
         standalone PCB editor is running the call refuses ('close_request_failed') \
         rather than risk closing that one. If no KiCad holds the board and no lock is \
         present, the result is status 'not_open' and nothing is done.",
        json!({
            "type": "object",
            "properties": {
                "board": { "type": "string", "description": "Path to the .kicad_pcb file" },
                "save": { "type": "boolean", "default": true, "description": "Save the board through IPC before closing. With false, KiCad asks about unsaved changes itself and the close waits on that prompt" },
                "timeout_seconds": { "type": "integer", "minimum": 5, "maximum": 600, "default": DEFAULT_CLOSE_TIMEOUT_SECONDS, "description": "How long to wait for the editor to exit and release its lock" }
            },
            "required": ["board"]
        }),
        |args, ctx| async move { handle_close_board_editor(args, ctx).await }
    )
}

// ─── What this server started ─────────────────────────────────────────────────

/// A standalone PCB editor process this server started.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct StartedEditor {
    pub pid: u32,
    pub binary: String,
}

/// The PCB editors this server process started, by board.
///
/// This is what separates an editor Konnect may close from the user's own:
/// only a process recorded here is ever asked to close.
#[derive(Clone, Default)]
pub(crate) struct StartedEditors {
    by_board: Arc<Mutex<HashMap<PathBuf, StartedEditor>>>,
}

impl StartedEditors {
    fn entries(&self) -> std::sync::MutexGuard<'_, HashMap<PathBuf, StartedEditor>> {
        self.by_board
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    pub(crate) fn record(&self, board: &Path, editor: StartedEditor) {
        self.entries()
            .insert(super::board_session::board_key(board), editor);
    }

    pub(crate) fn get(&self, board: &Path) -> Option<StartedEditor> {
        self.entries()
            .get(&super::board_session::board_key(board))
            .cloned()
    }

    pub(crate) fn forget(&self, board: &Path) {
        self.entries()
            .remove(&super::board_session::board_key(board));
    }
}

// ─── The decisions, apart from the world they act on ──────────────────────────

/// Whether some reachable KiCad holds the board.
pub(crate) enum Holder {
    /// One does. `endpoint` is the IPC address it answered at.
    Held { endpoint: Option<String> },
    /// None does: nothing was reached, no endpoint has a PCB editor, or every
    /// editor listed other boards.
    NotHeld { detail: String },
    /// A KiCad answered without settling it (busy, still starting, timed
    /// out), so it may hold the board.
    Unknown { detail: String },
}

/// Everything the two tools do to the world, so what they decide can be
/// tested without KiCad, a desktop or a process.
pub(crate) trait World {
    async fn holder(&self) -> anyhow::Result<Holder>;
    fn lock(&self) -> LockState;
    fn launch(&self) -> Result<Launched, String>;
    fn process_alive(&self, pid: u32) -> bool;
    async fn save(&self) -> Result<(), String>;
    /// Whether the PCB editor process `pid` can be asked to close, and only
    /// it. Asked before anything is changed.
    fn closable(&self, pid: u32) -> Result<(), String>;
    /// Ask the PCB editor process `pid` to close, as its close button does.
    fn request_close(&self, pid: u32) -> Result<(), String>;
    /// Let closed-board tools trust the saved file again. Called only once
    /// the editor has exited, its lock is gone and no KiCad holds the board.
    /// Returns whether the saved file is now authoritative.
    fn release_live_evidence(&self) -> bool;
    async fn pause(&self);
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Status {
    /// A reachable KiCad already held the board; nothing was started.
    AlreadyOpen,
    /// The editor was started and holds the board.
    Opened,
    /// The editor was closed: its process exited and its lock is gone.
    Closed,
    /// No KiCad holds the board and there is no lock: nothing to close.
    NotOpen,
    /// A KiCad answered without settling whether it holds the board.
    BoardStateUnknown,
    /// KiCad's lock for the board is present and no reachable KiCad holds it.
    KicadLockPresent,
    KicadLockUnreadable,
    /// The PCB editor could not be started.
    LaunchFailed,
    /// The started editor exited before it held the board.
    EditorExited,
    /// The started editor is running but the board did not become
    /// addressable over IPC.
    EditorNotAddressable,
    /// The board is held by an editor this server did not start.
    EditorNotStartedByKonnect,
    /// KiCad would not save the board; the editor was left open.
    SaveFailed,
    /// The editor could not be asked to close.
    CloseRequestFailed,
    /// The editor was asked to close and is still open, or its lock remains.
    EditorNotClosed,
}

impl Status {
    pub(crate) fn code(self) -> &'static str {
        match self {
            Self::AlreadyOpen => "already_open",
            Self::Opened => "opened",
            Self::Closed => "closed",
            Self::NotOpen => "not_open",
            Self::BoardStateUnknown => "board_state_unknown",
            Self::KicadLockPresent => "kicad_lock_present",
            Self::KicadLockUnreadable => "kicad_lock_unreadable",
            Self::LaunchFailed => "launch_failed",
            Self::EditorExited => "editor_exited",
            Self::EditorNotAddressable => "editor_not_addressable",
            Self::EditorNotStartedByKonnect => "editor_not_started_by_konnect",
            Self::SaveFailed => "save_failed",
            Self::CloseRequestFailed => "close_request_failed",
            Self::EditorNotClosed => "editor_not_closed",
        }
    }

    fn is_error(self) -> bool {
        !matches!(
            self,
            Self::AlreadyOpen | Self::Opened | Self::Closed | Self::NotOpen
        )
    }
}

/// What one call did and found.
#[derive(Debug)]
pub(crate) struct Outcome {
    pub status: Status,
    pub message: String,
    pub next_action: Option<String>,
    /// The IPC endpoint holding the board, when one does.
    pub endpoint: Option<String>,
    /// The editor process this server started for the board, when it did.
    pub editor: Option<StartedEditor>,
    /// Whether this call started that process.
    pub launched: bool,
    /// Whether the board was saved through IPC by this call.
    pub saved: bool,
    /// Whether closed-board tools may use the saved file again.
    pub file_authoritative: bool,
}

impl Outcome {
    fn new(status: Status, message: impl Into<String>) -> Self {
        Outcome {
            status,
            message: message.into(),
            next_action: None,
            endpoint: None,
            editor: None,
            launched: false,
            saved: false,
            file_authoritative: false,
        }
    }

    fn next(mut self, action: impl Into<String>) -> Self {
        self.next_action = Some(action.into());
        self
    }

    fn editor(mut self, editor: Option<StartedEditor>) -> Self {
        self.editor = editor;
        self
    }
}

const SETTLE_KICAD: &str =
    "Check KiCad: finish what it is doing or answer its dialog, and make sure the board is \
     either open and responsive or closed. Then call the tool again.";

pub(crate) async fn open_editor<W: World>(
    world: &W,
    started: &StartedEditors,
    board: &Path,
    polls: u32,
) -> anyhow::Result<Outcome> {
    // An editor recorded earlier that has since exited is no longer ours to
    // speak of, and its process id may belong to something else by now.
    let mine = started
        .get(board)
        .filter(|editor| world.process_alive(editor.pid));
    if mine.is_none() {
        started.forget(board);
    }

    match world.holder().await? {
        Holder::Held { endpoint } => {
            let mut outcome = Outcome::new(
                Status::AlreadyOpen,
                "A reachable KiCad already holds this board, so no editor was started.",
            )
            .editor(mine);
            outcome.endpoint = endpoint;
            return Ok(outcome);
        }
        // With no editor of this server's coming up, an unsettled answer may
        // be another editor holding the board.
        Holder::Unknown { detail } if mine.is_none() => {
            return Ok(Outcome::new(
                Status::BoardStateUnknown,
                format!(
                    "KiCad did not settle whether it holds this board ({detail}), so no editor \
                     was started: a second editor on a board that is already open would only \
                     meet KiCad's lock prompt."
                ),
            )
            .next(SETTLE_KICAD));
        }
        Holder::Unknown { .. } | Holder::NotHeld { .. } => {}
    }

    let (editor, launched) = match mine {
        // Started by an earlier call and still coming up: wait for that one
        // rather than starting another beside it.
        Some(editor) => (editor, false),
        None => {
            match world.lock() {
                LockState::Absent => {}
                LockState::Present(path) => {
                    return Ok(Outcome::new(
                        Status::KicadLockPresent,
                        format!(
                            "KiCad's lock file '{path}' is present, but no KiCad that Konnect \
                             can reach holds this board. An editor Konnect cannot reach has it \
                             open, or one crashed and left its lock. No editor was started, and \
                             Konnect never deletes a KiCad lock."
                        ),
                    )
                    .next(
                        "Find the KiCad window holding this board and close it, or open the \
                         board in KiCad, check that nothing was lost and close it normally so \
                         KiCad removes its own lock. Then call open_board_editor again.",
                    ));
                }
                LockState::Unreadable(detail) => {
                    return Ok(Outcome::new(
                        Status::KicadLockUnreadable,
                        format!(
                            "KiCad's lock file for this board could not be inspected \
                             ({detail}), so whether another editor holds it is unknown. No \
                             editor was started."
                        ),
                    )
                    .next(
                        "Make the board's directory readable, then call open_board_editor again.",
                    ));
                }
            }
            let launched = match world.launch() {
                Ok(launched) => launched,
                Err(error) => {
                    return Ok(Outcome::new(
                        Status::LaunchFailed,
                        format!("The PCB editor could not be started: {error}."),
                    )
                    .next(
                        "Set kicad_binary in Konnect's configuration to the KiCad executable of \
                         the installation to use, or open the board in KiCad by hand.",
                    ));
                }
            };
            let editor = StartedEditor {
                pid: launched.pid,
                binary: launched.binary,
            };
            // Recorded before the wait: an editor that is slow to answer is
            // still one this server started and may close.
            started.record(board, editor.clone());
            (editor, true)
        }
    };

    let mut last = String::new();
    for _ in 0..polls {
        match world.holder().await? {
            Holder::Held { endpoint } => {
                let mut outcome = Outcome::new(
                    Status::Opened,
                    if launched {
                        "The PCB editor was started and holds the board."
                    } else {
                        "The PCB editor started by an earlier call now holds the board."
                    },
                )
                .editor(Some(editor));
                outcome.endpoint = endpoint;
                outcome.launched = launched;
                return Ok(outcome);
            }
            Holder::NotHeld { detail } | Holder::Unknown { detail } => last = detail,
        }
        if !world.process_alive(editor.pid) {
            started.forget(board);
            let mut outcome = Outcome::new(
                Status::EditorExited,
                format!(
                    "The PCB editor (pid {}) exited before the board became addressable over \
                     IPC.",
                    editor.pid
                ),
            )
            .next("Open the board in KiCad by hand to see what it reports.");
            outcome.launched = launched;
            return Ok(outcome);
        }
        world.pause().await;
    }
    let pid = editor.pid;
    let mut outcome = Outcome::new(
        Status::EditorNotAddressable,
        format!(
            "The PCB editor is running (pid {pid}), but this board did not become addressable \
             over IPC in time (last answer: {last}). It may still be loading, it may be showing \
             a dialog, or KiCad's API server may be disabled (Preferences > Plugins > Enable \
             KiCad API)."
        ),
    )
    .editor(Some(editor))
    .next(
        "Look at the editor window. Call open_board_editor again to keep waiting for this same \
         editor, or close_board_editor with save false to close it.",
    );
    outcome.launched = launched;
    Ok(outcome)
}

pub(crate) async fn close_editor<W: World>(
    world: &W,
    started: &StartedEditors,
    board: &Path,
    save: bool,
    polls: u32,
) -> anyhow::Result<Outcome> {
    let users_editor = |what: &str| {
        Outcome::new(
            Status::EditorNotStartedByKonnect,
            format!(
                "{what} This Konnect server did not start it, so it was left alone: Konnect \
                 closes only an editor it started with open_board_editor."
            ),
        )
        .next(
            "Ask the user to save and close the board in KiCad. Closed-board tools work once \
             the editor has exited and KiCad's lock is gone.",
        )
    };

    let Some(editor) = started.get(board) else {
        return Ok(match world.holder().await? {
            Holder::Held { endpoint } => {
                let mut outcome = users_editor("A KiCad editor holds this board.");
                outcome.endpoint = endpoint;
                outcome
            }
            Holder::Unknown { detail } => Outcome::new(
                Status::BoardStateUnknown,
                format!(
                    "KiCad did not settle whether it holds this board ({detail}). Nothing was \
                     closed."
                ),
            )
            .next(SETTLE_KICAD),
            Holder::NotHeld { .. } => match world.lock() {
                LockState::Absent => Outcome::new(
                    Status::NotOpen,
                    "No reachable KiCad holds this board and KiCad's lock is absent, so there \
                     was nothing to close.",
                ),
                LockState::Present(path) => users_editor(&format!(
                    "KiCad's lock file '{path}' is present although no reachable KiCad holds \
                     this board: an editor Konnect cannot reach has it open, or one crashed."
                )),
                LockState::Unreadable(detail) => users_editor(&format!(
                    "KiCad's lock file for this board could not be inspected ({detail}), so an \
                     editor may hold it."
                )),
            },
        });
    };

    let mut saved = false;
    if world.process_alive(editor.pid) {
        let not_closable = |error: String, saved: bool| {
            let mut outcome = Outcome::new(
                Status::CloseRequestFailed,
                format!(
                    "The PCB editor (pid {}) could not be asked to close: {error}. It was left \
                     open{}.",
                    editor.pid,
                    if saved { "; the board was saved" } else { "" }
                ),
            )
            .editor(Some(editor.clone()))
            .next("Ask the user to close the PCB editor window.");
            outcome.saved = saved;
            outcome
        };
        // Everything that can refuse is asked before anything is changed.
        if let Err(error) = world.closable(editor.pid) {
            return Ok(not_closable(error, false));
        }
        if save {
            match world.holder().await? {
                Holder::Held { .. } => {}
                Holder::NotHeld { detail } | Holder::Unknown { detail } => {
                    return Ok(Outcome::new(
                        Status::EditorNotAddressable,
                        format!(
                            "The PCB editor Konnect started (pid {}) is running, but the board \
                             cannot be reached over IPC to save it ({detail}). The editor was \
                             left open.",
                            editor.pid
                        ),
                    )
                    .editor(Some(editor))
                    .next(
                        "Wait for the editor to finish loading or answer its dialog and call \
                         close_board_editor again, or pass save false to close it without \
                         saving; KiCad then asks about unsaved changes itself.",
                    ));
                }
            }
            if let Err(error) = world.save().await {
                return Ok(Outcome::new(
                    Status::SaveFailed,
                    format!(
                        "KiCad did not save the board ({error}), so the editor was left open. \
                         KiCad answers like this while it is busy or showing a dialog."
                    ),
                )
                .editor(Some(editor))
                .next(
                    "Finish what KiCad is doing or answer its dialog, then call \
                     close_board_editor again.",
                ));
            }
            saved = true;
        }

        if let Err(error) = world.request_close(editor.pid) {
            return Ok(not_closable(error, saved));
        }

        let mut closed = false;
        for _ in 0..polls {
            if !world.process_alive(editor.pid) && world.lock() == LockState::Absent {
                closed = true;
                break;
            }
            world.pause().await;
        }
        if !closed && world.process_alive(editor.pid) {
            let mut outcome = Outcome::new(
                Status::EditorNotClosed,
                format!(
                    "The PCB editor (pid {}) was asked to close and is still open; it is most \
                     likely showing a prompt. Konnect did not force it.",
                    editor.pid
                ),
            )
            .editor(Some(editor))
            .next(
                "Answer the prompt in KiCad or close the editor by hand. The close request \
                 cannot be withdrawn, so the editor may close as soon as the prompt is \
                 answered. Call close_board_editor again to confirm it has gone.",
            );
            outcome.saved = saved;
            return Ok(outcome);
        }
    }

    // The process is gone, by this call or before it. What it left behind
    // decides whether the saved file can be trusted.
    started.forget(board);
    let lock_left = match world.lock() {
        LockState::Absent => None,
        LockState::Present(path) => Some(format!("its lock file '{path}' is still there")),
        LockState::Unreadable(detail) => {
            Some(format!("its lock file could not be inspected ({detail})"))
        }
    };
    if let Some(lock) = lock_left {
        let mut outcome = Outcome::new(
            Status::EditorNotClosed,
            format!(
                "The PCB editor Konnect started (pid {}) has exited, but {lock}: it did not \
                 close through KiCad's normal path, or another editor holds the board. Konnect \
                 never deletes a KiCad lock.",
                editor.pid
            ),
        )
        .next(
            "Open the board in KiCad, check that nothing was lost, and close it normally so \
             KiCad removes its own lock.",
        );
        outcome.saved = saved;
        return Ok(outcome);
    }

    let mut outcome = match world.holder().await? {
        Holder::Held { endpoint } => {
            let mut outcome = users_editor(
                "The PCB editor Konnect started has exited, but another KiCad editor holds \
                 this board now.",
            );
            outcome.endpoint = endpoint;
            outcome
        }
        Holder::NotHeld { .. } => {
            let mut outcome = Outcome::new(
                Status::Closed,
                "The PCB editor has exited and KiCad's lock is gone.",
            )
            .editor(Some(editor));
            outcome.file_authoritative = world.release_live_evidence();
            outcome
        }
        // Closed, as far as this editor goes; what another KiCad would not
        // say is left for the closed-board tool's own check to settle.
        Holder::Unknown { detail } => Outcome::new(
            Status::Closed,
            format!(
                "The PCB editor has exited and KiCad's lock is gone. Another KiCad process did \
                 not settle whether it holds this board ({detail}), so a closed-board tool may \
                 still refuse until that KiCad answers."
            ),
        )
        .editor(Some(editor)),
    };
    outcome.saved = saved;
    Ok(outcome)
}

// ─── The real world ───────────────────────────────────────────────────────────

struct LiveWorld<'a> {
    ctx: &'a ToolContext,
    board: PathBuf,
    desktop: Box<dyn Desktop>,
}

impl World for LiveWorld<'_> {
    async fn holder(&self) -> anyhow::Result<Holder> {
        let not_held = |detail: String| Holder::NotHeld { detail };
        let unknown = |detail: String| Holder::Unknown { detail };
        Ok(
            match with_bound_board_ipc_classified(self.ctx, &self.board, |_, _| Ok(())).await? {
                Ok(BoardBinding::Bound(())) => Holder::Held {
                    endpoint: self.ctx.board_endpoints.last_holder(&self.board),
                },
                Ok(BoardBinding::Unserved {
                    absence: BoardEditorAbsence::NoPcbEditor,
                    message,
                }) => not_held(message),
                Ok(BoardBinding::Unserved { message, .. }) => unknown(message),
                Err(konnect_ipc::IpcFailure::Unreachable(message)) => not_held(message),
                Err(konnect_ipc::IpcFailure::Target { error, message })
                    if error.proves_not_open() =>
                {
                    not_held(message)
                }
                Err(failure) => unknown(failure.message().to_string()),
            },
        )
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

    fn launch(&self) -> Result<Launched, String> {
        let binary = crate::kicad_install::find_pcb_editor(
            &self.ctx.config.kicad_binary,
            &self.ctx.config.kicad_cli,
        )
        .ok_or_else(|| {
            "no standalone PCB editor (pcbnew) was found in the KiCad installation".to_string()
        })?;
        let pid = self.desktop.launch(&binary, &self.board)?;
        Ok(Launched {
            pid,
            binary: binary.display().to_string(),
        })
    }

    fn process_alive(&self, pid: u32) -> bool {
        self.desktop.process_alive(pid)
    }

    async fn save(&self) -> Result<(), String> {
        match with_board_ipc_classified(self.ctx, &self.board, |client, document| {
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

    fn closable(&self, pid: u32) -> Result<(), String> {
        desktop::closable_editor_of(&self.desktop.kicad_windows()?, pid).map(|_| ())
    }

    fn request_close(&self, pid: u32) -> Result<(), String> {
        let windows = self.desktop.kicad_windows()?;
        self.desktop
            .request_close(desktop::closable_editor_of(&windows, pid)?)
    }

    fn release_live_evidence(&self) -> bool {
        self.ctx
            .board_session
            .release_after_verified_close(&self.board)
    }

    async fn pause(&self) {
        tokio::time::sleep(POLL).await;
    }
}

// ─── Handlers ─────────────────────────────────────────────────────────────────

fn report(operation: &str, board: &Path, outcome: &Outcome, started: Instant) -> CallToolResult {
    let mut body = json!({
        "status": outcome.status.code(),
        "message": outcome.message,
        "board": board.display().to_string(),
        "endpoint": outcome.endpoint,
        "pid": outcome.editor.as_ref().map(|editor| editor.pid),
        "binary": outcome.editor.as_ref().map(|editor| editor.binary.clone()),
        "started_by_konnect": outcome.editor.is_some(),
        "next_action": outcome.next_action,
        "elapsed_ms": started.elapsed().as_millis() as u64,
    });
    if operation == OPEN {
        body["launched"] = json!(outcome.launched);
    } else {
        body["saved"] = json!(outcome.saved);
        body["closed_board_tools_ready"] = json!(outcome.file_authoritative);
    }
    let is_error = outcome.status.is_error();
    if is_error {
        body["error"] = json!({
            "kind": "plan_blocked",
            "operation": operation,
            "reasons": [outcome.status.code()],
        });
    }
    CallToolResult {
        content: vec![ToolContent::Text {
            text: body.to_string(),
        }],
        is_error,
    }
}

fn world<'a>(ctx: &'a ToolContext, board: &Path) -> LiveWorld<'a> {
    LiveWorld {
        ctx,
        board: board.to_path_buf(),
        desktop: desktop::system(),
    }
}

async fn handle_open_board_editor(
    args: &Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let board = get_path(args, "board")?;
    let timeout = match seconds(args, "timeout_seconds", DEFAULT_OPEN_TIMEOUT_SECONDS) {
        Ok(timeout) => timeout,
        Err(refusal) => return Ok(refusal),
    };
    if !board.is_file() {
        return Ok(CallToolResult::error_kind(
            crate::mcp::error::ToolErrorKind::FileNotFound {
                path: board.display().to_string(),
            },
            format!("Board file not found: {}", board.display()),
        ));
    }
    let started = Instant::now();
    let outcome = open_editor(
        &world(ctx, &board),
        &ctx.started_editors,
        &board,
        polls(timeout),
    )
    .await?;
    Ok(report(OPEN, &board, &outcome, started))
}

async fn handle_close_board_editor(
    args: &Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let board = get_path(args, "board")?;
    let save = match args.get("save") {
        None | Some(Value::Null) => true,
        Some(Value::Bool(save)) => *save,
        Some(_) => return Ok(crate::tools::invalid_arg("save", "must be true or false")),
    };
    let timeout = match seconds(args, "timeout_seconds", DEFAULT_CLOSE_TIMEOUT_SECONDS) {
        Ok(timeout) => timeout,
        Err(refusal) => return Ok(refusal),
    };
    let started = Instant::now();
    let outcome = close_editor(
        &world(ctx, &board),
        &ctx.started_editors,
        &board,
        save,
        polls(timeout),
    )
    .await?;
    Ok(report(CLOSE, &board, &outcome, started))
}

#[cfg(test)]
mod tests;
