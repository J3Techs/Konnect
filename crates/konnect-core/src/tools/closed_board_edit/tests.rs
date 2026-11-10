use super::desktop::KiCadWindow;
use super::*;
use std::sync::Mutex;

/// A scripted world: what each step answers, and a log of what was asked of
/// it in order. Time is counted in pauses.
struct Fake {
    held: Mutex<Option<Held>>,
    editor: Result<EditorTarget, String>,
    save: Result<(), String>,
    close: Result<(), String>,
    /// Pauses after which the editor is gone; `None` means never.
    editor_gone_after: Option<u32>,
    /// Pauses after which KiCad's lock is gone; `None` means never.
    lock_gone_after: Option<u32>,
    dialogs: Vec<String>,
    tool_error: bool,
    launch: Result<u32, String>,
    /// Pauses after the launch at which the board answers; `None` means never.
    addressable_after: Option<u32>,
    other_endpoint: Option<String>,
    ipc_configured: bool,
    log: Mutex<Vec<String>>,
    pauses: Mutex<u32>,
    launched_at: Mutex<Option<u32>>,
}

fn editor(process: &str) -> EditorTarget {
    EditorTarget {
        window: KiCadWindow {
            id: "7".to_string(),
            pid: 100,
            process: process.to_string(),
            title: Some("board \u{2014} PCB Editor".to_string()),
            owner: None,
        },
        standalone: process == "pcbnew",
    }
}

impl Fake {
    /// KiCad holds the board in a standalone editor and everything works.
    fn open() -> Self {
        Fake {
            held: Mutex::new(Some(Held::Open {
                pcb_editor_binary: Some("/kicad/bin/pcbnew".to_string()),
            })),
            editor: Ok(editor("pcbnew")),
            save: Ok(()),
            close: Ok(()),
            editor_gone_after: Some(2),
            lock_gone_after: Some(3),
            dialogs: Vec::new(),
            tool_error: false,
            launch: Ok(200),
            addressable_after: Some(4),
            other_endpoint: None,
            ipc_configured: true,
            log: Mutex::new(Vec::new()),
            pauses: Mutex::new(0),
            launched_at: Mutex::new(None),
        }
    }

    fn not_open() -> Self {
        let fake = Self::open();
        *fake.held.lock().unwrap() = Some(Held::NotOpen {
            reason: "ipc_unreachable",
            detail: "nothing listening".to_string(),
        });
        fake
    }

    fn note(&self, entry: &str) {
        self.log.lock().unwrap().push(entry.to_string());
    }

    fn log(&self) -> Vec<String> {
        self.log.lock().unwrap().clone()
    }

    fn now(&self) -> u32 {
        *self.pauses.lock().unwrap()
    }
}

impl Stage for Fake {
    async fn probe(&self) -> anyhow::Result<Held> {
        self.note("probe");
        Ok(self.held.lock().unwrap().take().expect("probed once"))
    }
    async fn save(&self) -> Result<(), String> {
        self.note("save");
        self.save.clone()
    }
    fn find_editor(&self) -> Result<EditorTarget, String> {
        self.note("find_editor");
        self.editor.clone()
    }
    fn request_close(&self, _editor: &EditorTarget) -> Result<(), String> {
        self.note("request_close");
        self.close.clone()
    }
    fn editor_open(&self, _editor: &EditorTarget) -> bool {
        self.editor_gone_after.is_none_or(|at| self.now() < at)
    }
    fn blocking_dialogs(&self, _editor: &EditorTarget) -> Vec<String> {
        self.dialogs.clone()
    }
    fn lock(&self) -> LockState {
        if self.lock_gone_after.is_none_or(|at| self.now() < at) {
            LockState::Present("~board.kicad_pcb.lck".to_string())
        } else {
            LockState::Absent
        }
    }
    async fn run_tool(&self) -> CallToolResult {
        self.note("run_tool");
        if self.tool_error {
            CallToolResult::error_kind(
                ToolErrorKind::StaleTarget {
                    target: "board".to_string(),
                    reason: "stale_revision".to_string(),
                },
                "stale",
            )
        } else {
            CallToolResult::json(&json!({ "success": true }))
        }
    }
    fn launch(&self, pcb_editor_binary: Option<&str>) -> Result<Launched, String> {
        self.note(&format!(
            "launch {}",
            pcb_editor_binary.unwrap_or("<installed>")
        ));
        *self.launched_at.lock().unwrap() = Some(self.now());
        self.launch.clone().map(|pid| Launched {
            pid,
            binary: pcb_editor_binary.unwrap_or("<installed>").to_string(),
        })
    }
    async fn addressable(&self) -> Result<(), String> {
        let since = self.now() - self.launched_at.lock().unwrap().expect("launched first");
        match self.addressable_after {
            Some(at) if since >= at => Ok(()),
            _ => Err("no PCB editor at this endpoint".to_string()),
        }
    }
    fn ipc_configured(&self) -> bool {
        self.ipc_configured
    }
    fn other_endpoint(&self, _pid: u32) -> Option<String> {
        self.other_endpoint.clone()
    }
    async fn pause(&self) {
        *self.pauses.lock().unwrap() += 1;
    }
}

fn request(reopen: Reopen) -> CycleRequest {
    CycleRequest {
        reopen,
        close_polls: 10,
        reopen_polls: 10,
    }
}

async fn run(fake: &Fake, reopen: Reopen) -> Cycle {
    run_cycle(fake, "set_design_rules", &request(reopen))
        .await
        .unwrap()
}

fn step<'a>(cycle: &'a Cycle, name: &str) -> &'a Value {
    cycle
        .steps
        .iter()
        .find(|step| step["step"] == name)
        .unwrap_or_else(|| panic!("no step '{name}' in {:?}", cycle.steps))
}

#[tokio::test]
async fn an_open_board_is_saved_closed_edited_and_reopened_in_that_order() {
    let fake = Fake::open();
    let cycle = run(&fake, Reopen::IfWasOpen).await;

    assert_eq!(
        fake.log(),
        [
            "probe",
            "find_editor",
            "save",
            "request_close",
            "run_tool",
            "launch /kicad/bin/pcbnew"
        ]
    );
    assert_eq!(cycle.status, Status::Complete);
    assert!(cycle.editor_was_open && cycle.board_saved);
    assert_eq!(cycle.editor_reopened, Some(true));
    assert_eq!(step(&cycle, "wait_closed")["outcome"], "closed");
    assert_eq!(step(&cycle, "wait_addressable")["outcome"], "addressable");
    assert!(cycle.next_action.is_none());
}

#[tokio::test]
async fn the_tool_does_not_run_until_both_the_editor_and_its_lock_are_gone() {
    // The editor is gone after 2 pauses and the lock after 3: the tool must
    // wait for the later of the two.
    let fake = Fake::open();
    run(&fake, Reopen::Never).await;
    assert_eq!(fake.now(), 3);
}

#[tokio::test]
async fn a_board_kicad_does_not_hold_just_gets_the_tool_run() {
    let fake = Fake::not_open();
    let cycle = run(&fake, Reopen::IfWasOpen).await;

    assert_eq!(fake.log(), ["probe", "run_tool"]);
    assert_eq!(cycle.status, Status::Complete);
    assert!(!cycle.editor_was_open && !cycle.board_saved);
    assert_eq!(cycle.editor_reopened, None);
    assert_eq!(step(&cycle, "confirm_open")["reason"], "ipc_unreachable");
    assert!(cycle.message.contains("did not hold the board"));
}

#[tokio::test]
async fn reopen_always_starts_the_installed_editor_for_a_board_that_was_closed() {
    let fake = Fake::not_open();
    let cycle = run(&fake, Reopen::Always).await;

    assert_eq!(fake.log(), ["probe", "run_tool", "launch <installed>"]);
    assert_eq!(cycle.editor_reopened, Some(true));
}

#[tokio::test]
async fn reopen_never_leaves_the_editor_closed_after_the_edit() {
    let fake = Fake::open();
    let cycle = run(&fake, Reopen::Never).await;

    assert!(!fake.log().iter().any(|entry| entry.starts_with("launch")));
    assert_eq!(cycle.status, Status::Complete);
    assert_eq!(cycle.editor_reopened, None);
}

#[tokio::test]
async fn an_unsettled_answer_from_kicad_stops_before_anything_is_touched() {
    let fake = Fake::open();
    *fake.held.lock().unwrap() = Some(Held::Unknown {
        reason: "ipc_outcome_unknown",
        detail: "timed out".to_string(),
    });
    let cycle = run(&fake, Reopen::Always).await;

    assert_eq!(fake.log(), ["probe"]);
    assert_eq!(cycle.status, Status::BoardStateUnknown);
    assert!(cycle.tool_result.is_none());
}

#[tokio::test]
async fn an_editor_window_that_cannot_be_identified_stops_before_the_save() {
    let mut fake = Fake::open();
    fake.editor = Err("two windows".to_string());
    let cycle = run(&fake, Reopen::IfWasOpen).await;

    assert_eq!(fake.log(), ["probe", "find_editor"]);
    assert_eq!(cycle.status, Status::EditorNotClosable);
    assert!(!cycle.board_saved);
    assert!(cycle.message.contains("two windows"));
}

#[tokio::test]
async fn an_editor_inside_the_project_manager_is_refused_and_left_alone() {
    let mut fake = Fake::open();
    fake.editor = Ok(editor("kicad"));
    let cycle = run(&fake, Reopen::IfWasOpen).await;

    assert_eq!(fake.log(), ["probe", "find_editor"]);
    assert_eq!(cycle.status, Status::EditorNotClosable);
    assert!(cycle.message.contains("project manager"));
    assert_eq!(cycle.editor.as_ref().unwrap()["standalone"], false);
}

#[tokio::test]
async fn a_failed_save_leaves_the_editor_open_and_runs_nothing() {
    let mut fake = Fake::open();
    fake.save = Err("AS_BUSY".to_string());
    let cycle = run(&fake, Reopen::IfWasOpen).await;

    assert_eq!(fake.log(), ["probe", "find_editor", "save"]);
    assert_eq!(cycle.status, Status::SaveFailed);
    assert!(!cycle.board_saved);
}

#[tokio::test]
async fn an_editor_that_stays_open_behind_a_prompt_is_never_edited_behind() {
    let mut fake = Fake::open();
    fake.editor_gone_after = None;
    fake.lock_gone_after = None;
    fake.dialogs = vec!["Save Changes?".to_string()];
    let cycle = run(&fake, Reopen::IfWasOpen).await;

    assert_eq!(
        fake.log(),
        ["probe", "find_editor", "save", "request_close"],
        "no tool run and no relaunch while the editor is still there"
    );
    assert_eq!(cycle.status, Status::EditorNotClosed);
    assert!(cycle.board_saved);
    assert!(cycle.tool_result.is_none());
    assert_eq!(cycle.editor_reopened, None);
    assert!(
        cycle.message.contains("'Save Changes?'"),
        "{}",
        cycle.message
    );
    let wait = step(&cycle, "wait_closed");
    assert_eq!(wait["outcome"], "timed_out");
    assert_eq!(wait["editor_still_open"], true);
    assert_eq!(wait["kicad_sibling_lock"], "present");
}

#[tokio::test]
async fn an_editor_that_exits_leaving_its_lock_is_not_edited_behind() {
    let mut fake = Fake::open();
    fake.lock_gone_after = None;
    let cycle = run(&fake, Reopen::IfWasOpen).await;

    assert!(!fake.log().contains(&"run_tool".to_string()));
    assert_eq!(cycle.status, Status::EditorNotClosed);
    assert!(cycle.message.contains("never deletes"), "{}", cycle.message);
    assert_eq!(step(&cycle, "wait_closed")["editor_still_open"], false);
}

#[tokio::test]
async fn a_close_request_that_cannot_be_sent_runs_nothing() {
    let mut fake = Fake::open();
    fake.close = Err("window gone".to_string());
    let cycle = run(&fake, Reopen::IfWasOpen).await;

    assert_eq!(
        fake.log(),
        ["probe", "find_editor", "save", "request_close"]
    );
    assert_eq!(cycle.status, Status::EditorNotClosed);
}

#[tokio::test]
async fn a_tool_that_fails_still_gets_the_editor_reopened_and_both_are_reported() {
    let mut fake = Fake::open();
    fake.tool_error = true;
    let cycle = run(&fake, Reopen::IfWasOpen).await;

    assert_eq!(fake.log().last().unwrap(), "launch /kicad/bin/pcbnew");
    assert_eq!(cycle.status, Status::ToolFailed);
    assert_eq!(cycle.editor_reopened, Some(true));
    assert!(cycle.tool_result.as_ref().unwrap().is_error);
}

#[tokio::test]
async fn an_editor_that_cannot_be_started_leaves_the_edit_standing() {
    let mut fake = Fake::open();
    fake.launch = Err("not found".to_string());
    let cycle = run(&fake, Reopen::IfWasOpen).await;

    assert_eq!(cycle.status, Status::CompleteEditorNotReopened);
    assert!(!cycle.tool_result.as_ref().unwrap().is_error);
    assert_eq!(cycle.editor_reopened, Some(false));
    assert!(cycle.next_action.as_ref().unwrap().contains("by hand"));
}

#[tokio::test]
async fn an_editor_that_answers_on_another_endpoint_is_named_in_the_report() {
    let mut fake = Fake::open();
    fake.addressable_after = None;
    fake.other_endpoint = Some("ipc:///tmp/kicad/api-200.sock".to_string());
    let cycle = run(&fake, Reopen::IfWasOpen).await;

    assert_eq!(cycle.status, Status::CompleteEditorNotReopened);
    assert_eq!(step(&cycle, "wait_addressable")["outcome"], "timed_out");
    let next = cycle.next_action.unwrap();
    assert!(
        next.contains("api-200.sock") && next.contains("KICAD_API_SOCKET"),
        "{next}"
    );
}

#[tokio::test]
async fn a_server_that_started_without_kicad_says_it_cannot_reach_the_new_editor() {
    let mut fake = Fake::not_open();
    fake.ipc_configured = false;
    fake.other_endpoint = Some("ipc:///tmp/kicad/api.sock".to_string());
    let cycle = run(&fake, Reopen::Always).await;

    assert_eq!(cycle.status, Status::CompleteEditorNotReopened);
    assert_eq!(
        fake.now(),
        0,
        "it does not wait for an answer that cannot come"
    );
    assert_eq!(
        step(&cycle, "wait_addressable")["outcome"],
        "ipc_not_configured"
    );
    let next = cycle.next_action.unwrap();
    assert!(
        next.contains("Restart the Konnect server") && next.contains("api.sock"),
        "{next}"
    );
}

#[tokio::test]
async fn a_failed_tool_and_a_failed_reopen_are_both_in_the_report() {
    let mut fake = Fake::open();
    fake.tool_error = true;
    fake.addressable_after = None;
    let cycle = run(&fake, Reopen::IfWasOpen).await;

    assert_eq!(cycle.status, Status::ToolFailed);
    assert_eq!(cycle.editor_reopened, Some(false));
    assert!(cycle.next_action.is_some());
}

// ─── The report ───────────────────────────────────────────────────────────────

fn body(result: &CallToolResult) -> Value {
    match &result.content[0] {
        ToolContent::Text { text } => serde_json::from_str(text).unwrap(),
        _ => panic!("text content"),
    }
}

#[tokio::test]
async fn the_report_carries_the_nested_result_and_is_an_error_only_when_something_failed() {
    let board = Path::new("board.kicad_pcb");

    let fake = Fake::open();
    let result = report(
        board,
        "set_design_rules",
        &run(&fake, Reopen::IfWasOpen).await,
    );
    assert!(!result.is_error);
    let json = body(&result);
    assert_eq!(json["status"], "complete");
    assert_eq!(json["tool_result"]["body"]["success"], true);
    assert_eq!(json["tool_ran"], true);

    // A nested structured error is this call's error kind too.
    let mut fake = Fake::open();
    fake.tool_error = true;
    let result = report(
        board,
        "set_design_rules",
        &run(&fake, Reopen::IfWasOpen).await,
    );
    assert!(result.is_error);
    assert_eq!(
        crate::mcp::error::extract_error_kind(&result).as_deref(),
        Some("stale_target")
    );
    assert_eq!(body(&result)["editor_reopened"], true);

    // A cycle that stopped names its own reason.
    let mut fake = Fake::open();
    fake.editor_gone_after = None;
    let result = report(
        board,
        "set_design_rules",
        &run(&fake, Reopen::IfWasOpen).await,
    );
    assert!(result.is_error);
    let json = body(&result);
    assert_eq!(json["error"]["kind"], "plan_blocked");
    assert_eq!(json["error"]["reasons"], json!(["editor_not_closed"]));
    assert_eq!(json["tool_ran"], false);
    assert_eq!(json["tool_result"], Value::Null);

    // An edit that stands with the editor still closed is not an error.
    let mut fake = Fake::open();
    fake.launch = Err("not found".to_string());
    let result = report(
        board,
        "set_design_rules",
        &run(&fake, Reopen::IfWasOpen).await,
    );
    assert!(!result.is_error);
    assert_eq!(body(&result)["status"], "complete_editor_not_reopened");
}

// ─── Which tools may be named, and the handler around the cycle ───────────────

fn every_registered_tool() -> Vec<ToolDef> {
    crate::router::registry::ALL_TOOLSETS
        .iter()
        .flat_map(|toolset| crate::router::registry::tools_for(toolset.name).unwrap())
        .collect()
}

#[test]
fn the_tools_that_may_be_named_come_from_their_board_access_and_all_take_a_board() {
    let tools = every_registered_tool();
    let allowed: Vec<&ToolDef> = tools
        .iter()
        .filter(|tool| has_closed_board_path(tool.board_access))
        .collect();
    for tool in &allowed {
        assert!(
            tool.input_schema["properties"].get("board").is_some(),
            "{} has a closed-board path but no 'board' argument",
            tool.name
        );
    }
    let names: Vec<&str> = allowed.iter().map(|tool| tool.name).collect();
    for expected in [
        "create_netclass",
        "assign_net_to_class",
        "remove_netclass_patterns",
        "set_design_rules",
        "set_predefined_sizes",
        "set_closed_board_stackup",
        "flip_component",
        "place_component",
        "update_pcb_from_schematic",
        "set_placed_footprint_models",
        "delete_zones",
        "set_reference_field_presentations",
    ] {
        assert!(names.contains(&expected), "{expected} should be allowed");
    }
    for refused in ["route_trace", "refill_zones", "run_drc", OPERATION] {
        assert!(!names.contains(&refused), "{refused} must not be allowed");
    }
}

const PROJECT: &str = "{\n  \"meta\": {\"filename\": \"board.kicad_pro\", \"version\": 1},\n  \"board\": {\"design_settings\": {}},\n  \"schematic\": {}\n}\n";

async fn ctx_with(toolsets: &[&str]) -> Arc<ToolContext> {
    let ctx = crate::tools::pcb_board::board_mock::ctx_talking_to(String::new());
    for toolset in toolsets {
        ctx.router.load(toolset).await.unwrap();
    }
    Arc::new(ctx)
}

fn project(dir: &Path) -> PathBuf {
    let board = dir.join("board.kicad_pcb");
    std::fs::write(
        &board,
        "(kicad_pcb (version 20241229) (generator \"pcbnew\"))\n",
    )
    .unwrap();
    std::fs::write(dir.join("board.kicad_pro"), PROJECT).unwrap();
    board
}

async fn call(ctx: &Arc<ToolContext>, args: Value) -> CallToolResult {
    handle_run_closed_board_edit(&args, ctx.clone())
        .await
        .unwrap()
}

#[tokio::test]
async fn with_no_kicad_running_the_named_tool_edits_the_project_and_no_editor_is_started() {
    let dir = tempfile::tempdir().unwrap();
    let board = project(dir.path());
    let ctx = ctx_with(&["verification"]).await;

    let result = call(
        &ctx,
        json!({
            "board": board,
            "tool": "set_design_rules",
            "arguments": { "min_clearance": 0.17 }
        }),
    )
    .await;

    let json = body(&result);
    assert!(!result.is_error, "{json}");
    assert_eq!(json["status"], "complete");
    assert_eq!(json["editor_was_open"], false);
    assert_eq!(json["editor_reopened"], Value::Null);
    assert_eq!(json["steps"][0]["reason"], "ipc_unreachable");
    let stored: Value =
        serde_json::from_str(&std::fs::read_to_string(dir.path().join("board.kicad_pro")).unwrap())
            .unwrap();
    assert_eq!(
        stored["board"]["design_settings"]["rules"]["min_clearance"],
        json!(0.17)
    );
}

#[tokio::test]
async fn the_nested_tool_keeps_its_own_lock_refusal() {
    // A lock beside the board with no KiCad reachable: the cycle has nothing
    // to close, and the nested tool's gate refuses exactly as a direct call.
    let dir = tempfile::tempdir().unwrap();
    let board = project(dir.path());
    std::fs::write(dir.path().join("~board.kicad_pcb.lck"), "{}").unwrap();
    let before = std::fs::read_to_string(dir.path().join("board.kicad_pro")).unwrap();
    let ctx = ctx_with(&["verification"]).await;

    let result = call(
        &ctx,
        json!({ "board": board, "tool": "set_design_rules", "arguments": { "min_clearance": 0.17 } }),
    )
    .await;

    assert!(result.is_error);
    let json = body(&result);
    assert_eq!(json["status"], "tool_failed");
    assert_eq!(json["error"]["kind"], "unsafe_file_fallback");
    assert_eq!(json["error"]["reason"], "kicad_lock_present");
    assert_eq!(
        std::fs::read_to_string(dir.path().join("board.kicad_pro")).unwrap(),
        before
    );
    assert!(
        dir.path().join("~board.kicad_pcb.lck").exists(),
        "the lock is KiCad's and is left alone"
    );
}

#[tokio::test]
async fn requests_that_cannot_work_are_refused_before_kicad_is_asked_anything() {
    let dir = tempfile::tempdir().unwrap();
    let board = project(dir.path());
    let ctx = ctx_with(&["verification"]).await;
    let refusal = |result: CallToolResult| {
        assert!(result.is_error);
        let json = body(&result);
        (
            json["error"]["kind"].as_str().unwrap().to_string(),
            json["error"]["field"]
                .as_str()
                .unwrap_or_default()
                .to_string(),
        )
    };

    for (tool, arguments, kind, field) in [
        ("no_such_tool", json!({}), "invalid_argument", "tool"),
        // Live-only: there is no closed-board path to run.
        ("route_trace", json!({}), "invalid_argument", "tool"),
        // No board contract at all.
        ("run_drc", json!({}), "invalid_argument", "tool"),
        (OPERATION, json!({}), "invalid_argument", "tool"),
        // Its toolset (pcb_routing) is not loaded in this session.
        (
            "create_netclass",
            json!({ "name": "Power" }),
            "toolset_not_loaded",
            "",
        ),
        (
            "set_design_rules",
            json!({ "board": "elsewhere.kicad_pcb" }),
            "invalid_argument",
            "arguments.board",
        ),
        (
            "set_design_rules",
            json!({ "min_clearance": "wide" }),
            "invalid_argument",
            "arguments",
        ),
        (
            "set_design_rules",
            json!({ "no_such_rule": 1 }),
            "invalid_argument",
            "arguments",
        ),
    ] {
        let result = call(
            &ctx,
            json!({ "board": board, "tool": tool, "arguments": arguments }),
        )
        .await;
        assert_eq!(
            refusal(result),
            (kind.to_string(), field.to_string()),
            "{tool}"
        );
    }
    assert_eq!(
        std::fs::read_to_string(dir.path().join("board.kicad_pro")).unwrap(),
        PROJECT
    );
}

#[tokio::test]
async fn the_served_dispatch_reaches_the_tool_and_enforces_its_schema() {
    let handler = crate::mcp::handler::McpHandler::new(crate::tools::ServerConfig::default())
        .await
        .unwrap();
    let call = |arguments: Value| {
        let handler = handler.clone();
        async move {
            let response = handler
                .handle_message(json!({
                    "jsonrpc": "2.0", "id": 1, "method": "tools/call",
                    "params": { "name": OPERATION, "arguments": arguments }
                }))
                .await
                .unwrap();
            serde_json::to_value(response).unwrap()["result"].clone()
        }
    };
    handler
        .handle_message(json!({
            "jsonrpc": "2.0", "id": 0, "method": "tools/call",
            "params": { "name": "load_toolset", "arguments": { "name": "verification" } }
        }))
        .await;

    let dir = tempfile::tempdir().unwrap();
    let board = project(dir.path());
    let refused =
        call(json!({ "board": board, "tool": "set_design_rules", "reopen": "sometimes" })).await;
    assert_eq!(refused["isError"], true);
    let text: Value =
        serde_json::from_str(refused["content"][0]["text"].as_str().unwrap()).unwrap();
    assert_eq!(text["error"]["kind"], "invalid_argument");

    let done = call(json!({
        "board": board, "tool": "set_design_rules", "arguments": { "min_trace_width": 0.2 }
    }))
    .await;
    assert_eq!(done["isError"], false, "{done}");
    let text: Value = serde_json::from_str(done["content"][0]["text"].as_str().unwrap()).unwrap();
    assert_eq!(text["status"], "complete");
    assert_eq!(text["tool_result"]["is_error"], false);
}
