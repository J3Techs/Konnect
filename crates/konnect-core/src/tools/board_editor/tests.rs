use super::*;
use crate::mcp::error::extract_error_kind;
use crate::tools::pcb_board::board_mock::ctx_talking_to;
use crate::tools::pcb_board::refuse_if_board_open_in_kicad;
use std::sync::Mutex;

const EDITOR_PID: u32 = 4242;
const EDITOR_ENDPOINT: &str = "ipc:///tmp/kicad/api-4242.sock";
const OTHER_ENDPOINT: &str = "ipc:///tmp/kicad/api.sock";

/// A scripted world. Time is counted in pauses, and the log records what was
/// asked of it, in order.
struct Fake {
    /// Another KiCad, not started by this server, holds the board.
    foreign_holder: bool,
    /// Some KiCad answers without settling whether it holds the board.
    unsettled: bool,
    /// A lock that is there whatever this server's editor does.
    foreign_lock: Option<LockState>,
    launch: Result<u32, String>,
    /// Pauses after its start at which the editor holds the board.
    addressable_after: Option<u32>,
    /// Pauses after its start at which the editor exits by itself.
    dies_after: Option<u32>,
    /// Whether an editor that exited by itself left its lock behind.
    death_leaves_lock: bool,
    save: Result<(), String>,
    closable: Result<(), String>,
    close: Result<(), String>,
    /// Pauses after the close request at which the editor exits.
    exits_after: Option<u32>,
    /// Pauses after the close request at which its lock is gone.
    lock_gone_after: Option<u32>,
    release: Box<dyn Fn() -> bool>,
    state: Mutex<State>,
}

#[derive(Default)]
struct State {
    pauses: u32,
    started_at: Option<u32>,
    close_requested_at: Option<u32>,
    log: Vec<String>,
}

impl Fake {
    /// No KiCad anywhere; an editor started here comes up and closes cleanly.
    fn idle() -> Self {
        Fake {
            foreign_holder: false,
            unsettled: false,
            foreign_lock: None,
            launch: Ok(EDITOR_PID),
            addressable_after: Some(3),
            dies_after: None,
            death_leaves_lock: true,
            save: Ok(()),
            closable: Ok(()),
            close: Ok(()),
            exits_after: Some(2),
            lock_gone_after: Some(3),
            release: Box::new(|| true),
            state: Mutex::new(State::default()),
        }
    }

    /// As [`Self::idle`], with this server's editor already up on the board.
    fn editor_running(started: &StartedEditors, board: &Path) -> Self {
        let fake = Self::idle();
        fake.state.lock().unwrap().started_at = Some(0);
        fake.state.lock().unwrap().pauses = 10;
        started.record(board, editor());
        fake
    }

    fn note(&self, entry: &str) {
        self.state.lock().unwrap().log.push(entry.to_string());
    }

    fn log(&self) -> Vec<String> {
        self.state.lock().unwrap().log.clone()
    }

    fn did(&self, entry: &str) -> bool {
        self.log().iter().any(|logged| logged == entry)
    }

    fn since(now: u32, at: Option<u32>, after: Option<u32>) -> bool {
        matches!((at, after), (Some(at), Some(after)) if now >= at + after)
    }

    fn editor_alive(&self) -> bool {
        let state = self.state.lock().unwrap();
        state.started_at.is_some()
            && !Self::since(state.pauses, state.started_at, self.dies_after)
            && !Self::since(state.pauses, state.close_requested_at, self.exits_after)
    }
}

fn editor() -> StartedEditor {
    StartedEditor {
        pid: EDITOR_PID,
        binary: "/kicad/bin/pcbnew".to_string(),
    }
}

impl World for Fake {
    async fn holder(&self) -> anyhow::Result<Holder> {
        self.note("holder");
        if self.unsettled {
            return Ok(Holder::Unknown {
                detail: "KiCad is busy".to_string(),
            });
        }
        if self.foreign_holder {
            return Ok(Holder::Held {
                endpoint: Some(OTHER_ENDPOINT.to_string()),
            });
        }
        let addressable = {
            let state = self.state.lock().unwrap();
            Self::since(state.pauses, state.started_at, self.addressable_after)
        };
        Ok(if addressable && self.editor_alive() {
            Holder::Held {
                endpoint: Some(EDITOR_ENDPOINT.to_string()),
            }
        } else {
            Holder::NotHeld {
                detail: "nothing listening".to_string(),
            }
        })
    }

    fn lock(&self) -> LockState {
        if let Some(lock) = &self.foreign_lock {
            return lock.clone();
        }
        let present = LockState::Present("~board.kicad_pcb.lck".to_string());
        let state = self.state.lock().unwrap();
        if state.started_at.is_none() {
            return LockState::Absent;
        }
        if state.close_requested_at.is_some() {
            return if Self::since(state.pauses, state.close_requested_at, self.lock_gone_after) {
                LockState::Absent
            } else {
                present
            };
        }
        if Self::since(state.pauses, state.started_at, self.dies_after) && !self.death_leaves_lock {
            return LockState::Absent;
        }
        present
    }

    fn launch(&self) -> Result<Launched, String> {
        self.note("launch");
        let pid = self.launch.clone()?;
        let mut state = self.state.lock().unwrap();
        state.started_at = Some(state.pauses);
        Ok(Launched {
            pid,
            binary: "/kicad/bin/pcbnew".to_string(),
        })
    }

    fn process_alive(&self, pid: u32) -> bool {
        pid == EDITOR_PID && self.editor_alive()
    }

    async fn save(&self) -> Result<(), String> {
        self.note("save");
        self.save.clone()
    }

    fn closable(&self, _pid: u32) -> Result<(), String> {
        self.closable.clone()
    }

    fn request_close(&self, pid: u32) -> Result<(), String> {
        self.note(&format!("request_close {pid}"));
        self.close.clone()?;
        let mut state = self.state.lock().unwrap();
        state.close_requested_at = Some(state.pauses);
        Ok(())
    }

    fn release_live_evidence(&self) -> bool {
        self.note("release_live_evidence");
        (self.release)()
    }

    async fn pause(&self) {
        self.state.lock().unwrap().pauses += 1;
    }
}

fn board() -> PathBuf {
    PathBuf::from("/work/board.kicad_pcb")
}

async fn open(fake: &Fake, started: &StartedEditors) -> Outcome {
    open_editor(fake, started, &board(), 20).await.unwrap()
}

async fn close(fake: &Fake, started: &StartedEditors, save: bool) -> Outcome {
    close_editor(fake, started, &board(), save, 20)
        .await
        .unwrap()
}

// ─── open_board_editor ────────────────────────────────────────────────────────

#[tokio::test]
async fn a_board_some_kicad_already_holds_is_not_given_a_second_editor() {
    let mut fake = Fake::idle();
    fake.foreign_holder = true;
    let started = StartedEditors::default();

    let outcome = open(&fake, &started).await;

    assert_eq!(outcome.status, Status::AlreadyOpen);
    assert_eq!(outcome.endpoint.as_deref(), Some(OTHER_ENDPOINT));
    assert_eq!(outcome.editor, None, "that editor is not this server's");
    assert!(!outcome.launched);
    assert_eq!(fake.log(), ["holder"], "nothing was started");
    assert_eq!(started.get(&board()), None);
}

#[tokio::test]
async fn a_board_this_servers_editor_holds_reports_that_editor() {
    let started = StartedEditors::default();
    let fake = Fake::editor_running(&started, &board());

    let outcome = open(&fake, &started).await;

    assert_eq!(outcome.status, Status::AlreadyOpen);
    assert_eq!(outcome.endpoint.as_deref(), Some(EDITOR_ENDPOINT));
    assert_eq!(outcome.editor, Some(editor()));
    assert!(!fake.did("launch"));
}

#[tokio::test]
async fn a_lock_with_no_reachable_editor_behind_it_refuses_the_launch() {
    for (lock, status) in [
        (
            LockState::Present("/work/~board.kicad_pcb.lck".to_string()),
            Status::KicadLockPresent,
        ),
        (
            LockState::Unreadable("permission denied".to_string()),
            Status::KicadLockUnreadable,
        ),
    ] {
        let mut fake = Fake::idle();
        fake.foreign_lock = Some(lock);
        let started = StartedEditors::default();

        let outcome = open(&fake, &started).await;

        assert_eq!(outcome.status, status);
        assert!(outcome.status.is_error());
        assert!(outcome.next_action.is_some());
        assert!(!fake.did("launch"), "an editor was started beside a lock");
        assert_eq!(started.get(&board()), None);
    }
}

#[tokio::test]
async fn an_unsettled_kicad_refuses_the_launch() {
    let mut fake = Fake::idle();
    fake.unsettled = true;
    let started = StartedEditors::default();

    let outcome = open(&fake, &started).await;

    assert_eq!(outcome.status, Status::BoardStateUnknown);
    assert!(!fake.did("launch"));
}

#[tokio::test]
async fn a_closed_board_is_opened_and_the_editor_is_recorded_as_this_servers() {
    let fake = Fake::idle();
    let started = StartedEditors::default();

    let outcome = open(&fake, &started).await;

    assert_eq!(outcome.status, Status::Opened);
    assert!(!outcome.status.is_error());
    assert!(outcome.launched);
    assert_eq!(outcome.endpoint.as_deref(), Some(EDITOR_ENDPOINT));
    assert_eq!(outcome.editor, Some(editor()));
    assert_eq!(started.get(&board()), Some(editor()));
    assert_eq!(
        fake.log()[..3],
        ["holder", "launch", "holder"],
        "asked who holds the board, then started one editor, then waited for it"
    );
    assert_eq!(
        fake.log().iter().filter(|entry| *entry == "launch").count(),
        1
    );
}

#[tokio::test]
async fn an_editor_that_cannot_be_started_is_reported_and_nothing_is_recorded() {
    let mut fake = Fake::idle();
    fake.launch = Err("no standalone PCB editor (pcbnew) was found".to_string());
    let started = StartedEditors::default();

    let outcome = open(&fake, &started).await;

    assert_eq!(outcome.status, Status::LaunchFailed);
    assert!(outcome.message.contains("pcbnew"), "{}", outcome.message);
    assert_eq!(started.get(&board()), None);
}

#[tokio::test]
async fn an_editor_that_exits_before_it_answers_is_forgotten() {
    let mut fake = Fake::idle();
    fake.addressable_after = None;
    fake.dies_after = Some(2);
    let started = StartedEditors::default();

    let outcome = open(&fake, &started).await;

    assert_eq!(outcome.status, Status::EditorExited);
    assert_eq!(started.get(&board()), None);
}

#[tokio::test]
async fn an_editor_slow_to_answer_stays_this_servers_and_is_not_started_twice() {
    let mut fake = Fake::idle();
    fake.addressable_after = Some(30);
    let started = StartedEditors::default();

    let outcome = open(&fake, &started).await;
    assert_eq!(outcome.status, Status::EditorNotAddressable);
    assert_eq!(outcome.editor, Some(editor()), "its pid is reported");
    assert_eq!(started.get(&board()), Some(editor()));

    // The second call waits for the same process: no second launch, and its
    // own lock is not mistaken for somebody else's.
    let outcome = open(&fake, &started).await;
    assert_eq!(outcome.status, Status::Opened);
    assert!(!outcome.launched);
    assert_eq!(outcome.editor, Some(editor()));
    assert_eq!(
        fake.log().iter().filter(|entry| *entry == "launch").count(),
        1
    );
}

#[tokio::test]
async fn an_editor_recorded_earlier_that_has_exited_does_not_stand_for_a_new_one() {
    let started = StartedEditors::default();
    started.record(&board(), editor());
    let fake = Fake::idle();

    let outcome = open(&fake, &started).await;

    assert_eq!(outcome.status, Status::Opened);
    assert!(outcome.launched);
}

// ─── close_board_editor ───────────────────────────────────────────────────────

#[tokio::test]
async fn an_editor_this_server_did_not_start_is_never_asked_to_close() {
    let mut fake = Fake::idle();
    fake.foreign_holder = true;
    let started = StartedEditors::default();

    let outcome = close(&fake, &started, true).await;

    assert_eq!(outcome.status, Status::EditorNotStartedByKonnect);
    assert!(outcome.status.is_error());
    assert_eq!(outcome.endpoint.as_deref(), Some(OTHER_ENDPOINT));
    assert!(
        outcome
            .next_action
            .as_deref()
            .is_some_and(|action| action.contains("user")),
        "{:?}",
        outcome.next_action
    );
    assert_eq!(fake.log(), ["holder"], "neither saved nor closed");
}

#[tokio::test]
async fn a_lock_nobody_reachable_owns_is_somebody_elses_editor_too() {
    let mut fake = Fake::idle();
    fake.foreign_lock = Some(LockState::Present("/work/~board.kicad_pcb.lck".to_string()));
    let started = StartedEditors::default();

    let outcome = close(&fake, &started, true).await;

    assert_eq!(outcome.status, Status::EditorNotStartedByKonnect);
    assert!(!fake.did("release_live_evidence"));
}

#[tokio::test]
async fn a_board_nothing_holds_has_nothing_to_close_and_keeps_its_evidence() {
    let fake = Fake::idle();
    let started = StartedEditors::default();

    let outcome = close(&fake, &started, true).await;

    assert_eq!(outcome.status, Status::NotOpen);
    assert!(!outcome.status.is_error());
    assert!(!outcome.file_authoritative);
    assert_eq!(fake.log(), ["holder"]);
}

#[tokio::test]
async fn an_unsettled_kicad_closes_nothing() {
    let mut fake = Fake::idle();
    fake.unsettled = true;
    let started = StartedEditors::default();

    let outcome = close(&fake, &started, true).await;

    assert_eq!(outcome.status, Status::BoardStateUnknown);
    assert_eq!(fake.log(), ["holder"]);
}

#[tokio::test]
async fn closing_saves_asks_once_waits_and_only_then_releases_the_evidence() {
    let started = StartedEditors::default();
    let fake = Fake::editor_running(&started, &board());

    let outcome = close(&fake, &started, true).await;

    assert_eq!(outcome.status, Status::Closed);
    assert!(outcome.saved);
    assert!(outcome.file_authoritative);
    assert_eq!(
        fake.log(),
        [
            "holder",
            "save",
            &format!("request_close {EDITOR_PID}"),
            "holder",
            "release_live_evidence"
        ]
    );
    assert_eq!(started.get(&board()), None, "the editor is gone");
    // Released after the process exited and the lock went, not before.
    let state = fake.state.lock().unwrap();
    assert!(state.pauses >= state.close_requested_at.unwrap() + 3);
}

#[tokio::test]
async fn closing_without_saving_sends_no_save() {
    let started = StartedEditors::default();
    let fake = Fake::editor_running(&started, &board());

    let outcome = close(&fake, &started, false).await;

    assert_eq!(outcome.status, Status::Closed);
    assert!(!outcome.saved);
    assert!(!fake.did("save"));
    assert!(fake.did("release_live_evidence"));
}

#[tokio::test]
async fn a_failed_save_leaves_the_editor_open_and_the_evidence_in_place() {
    let started = StartedEditors::default();
    let mut fake = Fake::editor_running(&started, &board());
    fake.save = Err("KiCad is busy".to_string());

    let outcome = close(&fake, &started, true).await;

    assert_eq!(outcome.status, Status::SaveFailed);
    assert_eq!(fake.log(), ["holder", "save"]);
    assert_eq!(started.get(&board()), Some(editor()));
}

#[tokio::test]
async fn an_editor_that_cannot_be_reached_to_save_is_left_open() {
    let started = StartedEditors::default();
    let mut fake = Fake::editor_running(&started, &board());
    fake.addressable_after = None;

    let outcome = close(&fake, &started, true).await;

    assert_eq!(outcome.status, Status::EditorNotAddressable);
    assert_eq!(fake.log(), ["holder"]);
    assert_eq!(started.get(&board()), Some(editor()));
}

#[tokio::test]
async fn a_close_request_that_cannot_be_sent_is_reported() {
    let started = StartedEditors::default();
    let mut fake = Fake::editor_running(&started, &board());
    fake.close = Err("no standalone PCB editor window".to_string());

    let outcome = close(&fake, &started, true).await;

    assert_eq!(outcome.status, Status::CloseRequestFailed);
    assert!(outcome.saved, "the save that preceded it stands");
    assert!(!fake.did("release_live_evidence"));
    assert_eq!(started.get(&board()), Some(editor()));
}

/// Two editors run from one application and the platform cannot address a
/// close to one of them: the other may be the user's, so nothing is done,
/// not even the save.
#[tokio::test]
async fn an_editor_that_cannot_be_told_from_another_is_not_saved_or_closed() {
    let started = StartedEditors::default();
    let mut fake = Fake::editor_running(&started, &board());
    fake.closable = Err("another PCB editor (pid 77) runs from the same application".to_string());

    let outcome = close(&fake, &started, true).await;

    assert_eq!(outcome.status, Status::CloseRequestFailed);
    assert!(!outcome.saved);
    assert!(fake.log().is_empty(), "{:?}", fake.log());
    assert_eq!(started.get(&board()), Some(editor()));
}

#[tokio::test]
async fn an_editor_that_stays_open_is_not_forced_and_releases_nothing() {
    let started = StartedEditors::default();
    let mut fake = Fake::editor_running(&started, &board());
    fake.exits_after = None;

    let outcome = close(&fake, &started, false).await;

    assert_eq!(outcome.status, Status::EditorNotClosed);
    assert!(
        outcome.message.contains("still open"),
        "{}",
        outcome.message
    );
    assert!(!fake.did("release_live_evidence"));
    assert_eq!(
        started.get(&board()),
        Some(editor()),
        "still this server's editor, for the next call"
    );
}

#[tokio::test]
async fn an_editor_that_exits_and_leaves_its_lock_releases_nothing() {
    let started = StartedEditors::default();
    let mut fake = Fake::editor_running(&started, &board());
    fake.lock_gone_after = None;

    let outcome = close(&fake, &started, true).await;

    assert_eq!(outcome.status, Status::EditorNotClosed);
    assert!(outcome.message.contains("lock"), "{}", outcome.message);
    assert!(outcome.saved);
    assert!(!fake.did("release_live_evidence"));
}

#[tokio::test]
async fn an_editor_the_user_already_closed_cleanly_counts_as_closed() {
    let started = StartedEditors::default();
    let mut fake = Fake::editor_running(&started, &board());
    fake.dies_after = Some(5);
    fake.death_leaves_lock = false;

    let outcome = close(&fake, &started, true).await;

    assert_eq!(outcome.status, Status::Closed);
    assert!(!outcome.saved, "there was no editor left to save through");
    assert_eq!(fake.log(), ["holder", "release_live_evidence"]);
    assert_eq!(started.get(&board()), None);
}

#[tokio::test]
async fn an_editor_that_died_and_left_its_lock_releases_nothing() {
    let started = StartedEditors::default();
    let mut fake = Fake::editor_running(&started, &board());
    fake.dies_after = Some(5);

    let outcome = close(&fake, &started, true).await;

    assert_eq!(outcome.status, Status::EditorNotClosed);
    assert!(fake.log().is_empty(), "nothing was asked of a dead editor");
    assert_eq!(started.get(&board()), None);
}

#[tokio::test]
async fn another_editor_holding_the_board_after_ours_exited_keeps_the_evidence() {
    let started = StartedEditors::default();
    let mut fake = Fake::editor_running(&started, &board());
    fake.dies_after = Some(5);
    fake.death_leaves_lock = false;
    fake.foreign_holder = true;

    let outcome = close(&fake, &started, true).await;

    assert_eq!(outcome.status, Status::EditorNotStartedByKonnect);
    assert!(!fake.did("release_live_evidence"));
    assert!(!fake.did(&format!("request_close {EDITOR_PID}")));
}

/// The point of the close: a closed-board tool refuses a board this server
/// saw live, and accepts it once the close has been carried out and verified.
#[tokio::test]
async fn a_verified_close_lets_closed_board_tools_use_the_board_in_the_same_session() {
    let dir = tempfile::tempdir().unwrap();
    let board = dir.path().join("board.kicad_pcb");
    std::fs::write(&board, "").unwrap();
    let ctx = ctx_talking_to(String::new());
    // Observed live with no lock on disk: nothing but a watched close can
    // restore the file's authority.
    ctx.board_session.observe_live(&board);
    assert!(refuse_if_board_open_in_kicad(&ctx, &board, "test")
        .await
        .unwrap()
        .is_some());

    let started = StartedEditors::default();
    let mut fake = Fake::editor_running(&started, &board);
    let memory = ctx.board_session.clone();
    let released = board.clone();
    fake.release = Box::new(move || memory.release_after_verified_close(&released));

    let outcome = close_editor(&fake, &started, &board, true, 20)
        .await
        .unwrap();

    assert_eq!(outcome.status, Status::Closed);
    assert!(outcome.file_authoritative);
    assert!(refuse_if_board_open_in_kicad(&ctx, &board, "test")
        .await
        .unwrap()
        .is_none());
}

/// The same close with the editor still open at the end changes nothing.
#[tokio::test]
async fn an_unfinished_close_leaves_closed_board_tools_refusing() {
    let dir = tempfile::tempdir().unwrap();
    let board = dir.path().join("board.kicad_pcb");
    std::fs::write(&board, "").unwrap();
    let ctx = ctx_talking_to(String::new());
    ctx.board_session.observe_live(&board);

    let started = StartedEditors::default();
    let mut fake = Fake::editor_running(&started, &board);
    fake.exits_after = None;
    let memory = ctx.board_session.clone();
    let released = board.clone();
    fake.release = Box::new(move || memory.release_after_verified_close(&released));

    let outcome = close_editor(&fake, &started, &board, true, 20)
        .await
        .unwrap();

    assert_eq!(outcome.status, Status::EditorNotClosed);
    assert!(ctx.board_session.was_observed_live(&board));
    assert!(refuse_if_board_open_in_kicad(&ctx, &board, "test")
        .await
        .unwrap()
        .is_some());
}

// ─── The tools as served ──────────────────────────────────────────────────────

fn body(result: &CallToolResult) -> Value {
    match &result.content[0] {
        ToolContent::Text { text } => serde_json::from_str(text).unwrap(),
        _ => panic!("expected text content"),
    }
}

/// A board on disk with KiCad's lock beside it and no KiCad anywhere: the
/// state both tools must refuse in, so neither starts or closes anything.
fn locked_board() -> (tempfile::TempDir, PathBuf) {
    let dir = tempfile::tempdir().unwrap();
    let board = dir.path().join("board.kicad_pcb");
    std::fs::write(&board, "").unwrap();
    std::fs::write(dir.path().join("~board.kicad_pcb.lck"), "").unwrap();
    (dir, board)
}

#[tokio::test]
async fn open_board_editor_refuses_beside_a_lock_with_a_structured_error() {
    let (_dir, board) = locked_board();
    let ctx = ctx_talking_to(String::new());

    let result = handle_open_board_editor(&json!({ "board": board }), &ctx)
        .await
        .unwrap();

    assert!(result.is_error);
    assert_eq!(extract_error_kind(&result).as_deref(), Some("plan_blocked"));
    let body = body(&result);
    assert_eq!(body["status"], "kicad_lock_present", "{body}");
    assert_eq!(body["error"]["operation"], "open_board_editor", "{body}");
    assert_eq!(body["error"]["reasons"], json!(["kicad_lock_present"]));
    assert_eq!(body["launched"], false, "{body}");
    assert_eq!(body["pid"], json!(null), "{body}");
    assert_eq!(ctx.started_editors.get(&board), None);
}

#[tokio::test]
async fn open_board_editor_names_a_board_file_that_is_not_there() {
    let dir = tempfile::tempdir().unwrap();
    let ctx = ctx_talking_to(String::new());

    let result = handle_open_board_editor(
        &json!({ "board": dir.path().join("absent.kicad_pcb") }),
        &ctx,
    )
    .await
    .unwrap();

    assert_eq!(
        extract_error_kind(&result).as_deref(),
        Some("file_not_found")
    );
}

#[tokio::test]
async fn close_board_editor_refuses_an_editor_it_did_not_start_with_a_structured_error() {
    let (_dir, board) = locked_board();
    let ctx = ctx_talking_to(String::new());
    ctx.board_session.observe_live(&board);

    let result = handle_close_board_editor(&json!({ "board": board }), &ctx)
        .await
        .unwrap();

    assert!(result.is_error);
    let body = body(&result);
    assert_eq!(body["status"], "editor_not_started_by_konnect", "{body}");
    assert_eq!(body["error"]["kind"], "plan_blocked", "{body}");
    assert_eq!(body["error"]["operation"], "close_board_editor", "{body}");
    assert_eq!(body["started_by_konnect"], false, "{body}");
    assert_eq!(body["closed_board_tools_ready"], false, "{body}");
    assert!(ctx.board_session.was_observed_live(&board));
}

#[tokio::test]
async fn close_board_editor_on_a_board_nothing_holds_is_not_an_error() {
    let dir = tempfile::tempdir().unwrap();
    let board = dir.path().join("board.kicad_pcb");
    std::fs::write(&board, "").unwrap();
    let ctx = ctx_talking_to(String::new());

    let result = handle_close_board_editor(&json!({ "board": board, "save": false }), &ctx)
        .await
        .unwrap();

    assert!(!result.is_error);
    assert_eq!(body(&result)["status"], "not_open");
}

#[tokio::test]
async fn both_tools_refuse_arguments_outside_their_schema() {
    let ctx = ctx_talking_to(String::new());
    let (_dir, board) = locked_board();

    let result = handle_open_board_editor(&json!({ "board": board, "timeout_seconds": 1 }), &ctx)
        .await
        .unwrap();
    assert_eq!(
        extract_error_kind(&result).as_deref(),
        Some("invalid_argument")
    );

    let result = handle_close_board_editor(&json!({ "board": board, "save": "yes" }), &ctx)
        .await
        .unwrap();
    assert_eq!(
        extract_error_kind(&result).as_deref(),
        Some("invalid_argument")
    );
}
