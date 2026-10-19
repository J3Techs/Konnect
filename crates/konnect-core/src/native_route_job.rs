//! Native route jobs: the konnect-route pipeline in a process of its own.
//!
//! Modelled on [`crate::freerouting_job`]. A run started with
//! `start_native_route_job` is owned by a separate `konnect
//! native-route-runner` process, started outside the client's job object, so
//! it outlives the MCP session, and several runs (one per output) proceed at
//! once, each on its own rayon pool. The runner keeps its state in a job
//! directory beside the requested plan:
//!
//! - `spec.json`: what to route and how, written once by the starter;
//! - `status.json`: the runner's state and heartbeat, rewritten as it runs;
//! - `progress.jsonl`: one JSON line per stage event, appended;
//! - `checkpoint.json`: the routed copper after each completed stage, so a
//!   killed run resumes where it stopped (`resume: true`);
//! - `report.json`: the final report, with telemetry;
//! - `stop`: created by `stop_native_route_job`; the runner stops at the
//!   next stage boundary and keeps its checkpoint;
//! - `runner.log`: the runner's own stderr.
//!
//! The plan itself (`output_plan`) is an `add_routing` `routing_file`. The
//! runner never talks to KiCad: it reads saved files and writes a plan.

use anyhow::{bail, Context, Result};
use konnect_route::api::{Generation, NetId, Stage};
use konnect_route::check::{CheckReport, ViolationKind};
use konnect_route::connectivity::ConnectivityReport;
use konnect_route::optimize::exact::{CheckerOptions, ExactChecker};
use konnect_route::optimize::{optimize_checked, OptimizeOptions, OptimizeReport};
use konnect_route::plan::{RoutedCopper, ADD_ROUTING_MAX_ITEMS};
use konnect_route::telemetry::Telemetry;
use konnect_route::{LoadOptions, ProjectRuleTable, RouteBoard, RoutePlan, RuleTable};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

/// The hidden subcommand the starter runs.
pub const RUNNER_SUBCOMMAND: &str = "native-route-runner";

const SPEC_FILE: &str = "spec.json";
const STATUS_FILE: &str = "status.json";
const PROGRESS_FILE: &str = "progress.jsonl";
const CHECKPOINT_FILE: &str = "checkpoint.json";
const REPORT_FILE: &str = "report.json";
const STOP_FILE: &str = "stop";
const RUNNER_LOG: &str = "runner.log";
/// The runner rewrites its heartbeat this often while a stage runs.
const HEARTBEAT: Duration = Duration::from_secs(2);
/// A runner silent for this long is gone.
const STALE_HEARTBEAT_SECONDS: u64 = 30;
/// How long the starter waits for the runner to report it is running.
const START_WAIT: Duration = Duration::from_secs(30);

/// Where the copper to optimize, check and plan comes from.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum CopperSource {
    /// The native S2–S5 stages route the board's open connections.
    #[default]
    Native,
    /// A `routing_file` plan (from an earlier run, say), read back.
    Plan { path: PathBuf },
    /// The board's own unlocked tracks and vias (an imported session or a
    /// previous run on the board).
    BoardUnlocked,
}

/// What to route and how.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NativeJobSpec {
    /// The saved `.kicad_pcb`.
    pub board: PathBuf,
    /// The `.kicad_pro`; defaults to the one beside the board.
    #[serde(default)]
    pub project: Option<PathBuf>,
    /// The `.kicad_dru`; defaults to the one beside the project, if any.
    #[serde(default)]
    pub rules: Option<PathBuf>,
    /// The `routing_file` to write. Never replaced if it exists.
    pub output_plan: PathBuf,
    #[serde(default)]
    pub source: CopperSource,
    /// Work only on copper inside this box, mm (x1, y1, x2, y2).
    #[serde(default)]
    pub region_mm: Option<[f64; 4]>,
    /// Work only on these nets.
    #[serde(default)]
    pub nets: Option<Vec<String>>,
    /// Worker threads; all cores when unset.
    #[serde(default)]
    pub threads: Option<usize>,
    #[serde(default = "yes")]
    pub optimize: bool,
    /// Treat other nets' zone fills as copper in the optimizer and check.
    #[serde(default = "yes")]
    pub zones_are_obstacles: bool,
    /// Stop at the next stage boundary after this long.
    #[serde(default)]
    pub time_limit_seconds: Option<u64>,
}

fn yes() -> bool {
    true
}

impl NativeJobSpec {
    fn project_path(&self) -> Result<PathBuf> {
        if let Some(p) = &self.project {
            return Ok(p.clone());
        }
        let p = self.board.with_extension("kicad_pro");
        if p.is_file() {
            return Ok(p);
        }
        let dir = self.board.parent().unwrap_or(Path::new("."));
        let mut found: Vec<PathBuf> = std::fs::read_dir(dir)
            .with_context(|| format!("read {}", dir.display()))?
            .filter_map(|e| e.ok().map(|e| e.path()))
            .filter(|p| p.extension().and_then(|e| e.to_str()) == Some("kicad_pro"))
            .collect();
        found.sort();
        match found.len() {
            1 => Ok(found.remove(0)),
            0 => bail!(
                "no .kicad_pro beside {}; pass project",
                self.board.display()
            ),
            _ => bail!(
                "several .kicad_pro files beside {}; pass project",
                self.board.display()
            ),
        }
    }

    fn rules_path(&self, project: &Path) -> Option<PathBuf> {
        self.rules
            .clone()
            .or_else(|| Some(project.with_extension("kicad_dru")).filter(|p| p.is_file()))
    }

    fn region(&self) -> Result<Option<konnect_geom::BBox>> {
        let Some([x1, y1, x2, y2]) = self.region_mm else {
            return Ok(None);
        };
        if !(x1 < x2 && y1 < y2) {
            bail!("region_mm needs x1 < x2 and y1 < y2");
        }
        let a = konnect_geom::Point::from_mm(x1, y1).context("region_mm is off the board")?;
        let b = konnect_geom::Point::from_mm(x2, y2).context("region_mm is off the board")?;
        Ok(Some(konnect_geom::BBox::new(a, b)))
    }

    /// Read the board and compile its project rules (classes assigned).
    pub(crate) fn load_inputs(
        &self,
    ) -> Result<(RouteBoard, konnect_route::LoadReport, ProjectRuleTable)> {
        let (mut board, load) =
            konnect_route::load_board_file(&self.board, &LoadOptions::default())
                .with_context(|| format!("load {}", self.board.display()))?;
        let project = self.project_path()?;
        let pro_text = std::fs::read_to_string(&project)
            .with_context(|| format!("read {}", project.display()))?;
        let dru_text = match self.rules_path(&project) {
            Some(p) => {
                Some(std::fs::read_to_string(&p).with_context(|| format!("read {}", p.display()))?)
            }
            None => None,
        };
        let rules = ProjectRuleTable::from_project(&pro_text, dru_text.as_deref(), &board)
            .context("compile the project rules")?;
        rules.assign_classes(&mut board);
        Ok((board, load, rules))
    }

    fn hash(&self) -> String {
        let text = serde_json::to_vec(self).unwrap_or_default();
        format!("{:x}", Sha256::digest(&text))
    }

    /// Everything that can be refused before a runner starts.
    pub fn validate(&self) -> Result<()> {
        if !self.board.is_file() {
            bail!("board is not a file: {}", self.board.display());
        }
        let project = self.project_path()?;
        if !project.is_file() {
            bail!("project is not a file: {}", project.display());
        }
        if let Some(r) = &self.rules {
            if !r.is_file() {
                bail!("rules is not a file: {}", r.display());
            }
        }
        if let CopperSource::Plan { path } = &self.source {
            if !path.is_file() {
                bail!("source plan is not a file: {}", path.display());
            }
        }
        let parent = self.output_plan.parent().unwrap_or(Path::new("."));
        if !parent.as_os_str().is_empty() && !parent.is_dir() {
            bail!(
                "output_plan's directory does not exist: {}",
                parent.display()
            );
        }
        self.region()?;
        if self.threads == Some(0) {
            bail!("threads must be at least 1");
        }
        Ok(())
    }
}

// ─── Pipeline ────────────────────────────────────────────────────────────

/// One stage's outcome.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StageRecord {
    pub stage: String,
    /// `done`, `resumed` (taken from the checkpoint), `skipped` or `failed`.
    pub state: String,
    pub wall_ms: u64,
    pub detail: Value,
}

/// What the internal check found.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct CheckSummary {
    /// `check_routed` (the S7 checker) or `interim_exact` (the optimizer's
    /// exact checker, used while S7 is not built).
    pub checker: String,
    pub authoritative: bool,
    pub violations: usize,
    pub by_kind: BTreeMap<String, usize>,
}

/// The pipeline's whole account of a run.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct PipelineReport {
    pub stages: Vec<StageRecord>,
    pub connections_needed: Option<usize>,
    pub work_tracks: usize,
    pub work_vias: usize,
    pub optimize: Option<OptimizeReport>,
    pub check: Option<CheckSummary>,
    pub plan_items: Option<usize>,
    pub plan_chunks: Option<usize>,
    pub plan_path: Option<PathBuf>,
    /// Set when the run stopped early (a stage failed, a stop was asked, the
    /// time limit passed).
    pub stopped: Option<String>,
    pub telemetry: Telemetry,
}

/// The copper after each completed stage, keyed to the spec it came from.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Checkpoint {
    pub spec_hash: String,
    pub completed: Vec<String>,
    pub copper: RoutedCopper,
    /// Copper outside the region or nets, kept as board copper.
    #[serde(default)]
    pub frozen: RoutedCopper,
    pub optimize: Option<OptimizeReport>,
    pub check: Option<CheckSummary>,
    pub written_unix: u64,
}

/// What the pipeline tells whoever runs it.
pub trait PipelineObserver {
    fn event(&mut self, stage: &str, event: &str, detail: &Value);
    /// Asked between stages.
    fn should_stop(&mut self) -> Option<String>;
    fn checkpoint(&mut self, checkpoint: &Checkpoint);
}

/// An observer that records nothing (in-process tool calls).
pub struct Quiet;

impl PipelineObserver for Quiet {
    fn event(&mut self, _: &str, _: &str, _: &Value) {}
    fn should_stop(&mut self) -> Option<String> {
        None
    }
    fn checkpoint(&mut self, _: &Checkpoint) {}
}

/// Run `f`, turning a panic (an unbuilt stage's `todo!`) into an error.
pub(crate) fn guarded<T>(stage: &str, f: impl FnOnce() -> T) -> Result<T> {
    std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)).map_err(|payload| {
        let msg = payload
            .downcast_ref::<String>()
            .cloned()
            .or_else(|| payload.downcast_ref::<&str>().map(|s| s.to_string()))
            .unwrap_or_else(|| "panicked".to_string());
        anyhow::anyhow!("stage {stage} is not available: {msg}")
    })
}

fn by_kind(report: &CheckReport) -> BTreeMap<String, usize> {
    let name = |k: ViolationKind| {
        serde_json::to_value(k)
            .ok()
            .and_then(|v| v.as_str().map(String::from))
            .unwrap_or_else(|| format!("{k:?}"))
    };
    let mut m = BTreeMap::new();
    for v in &report.violations {
        *m.entry(name(v.kind)).or_default() += 1;
    }
    m
}

/// The native S2–S5 route of the board's open connections.
fn route_native(board: &RouteBoard, rules: &dyn RuleTable) -> Result<RoutedCopper> {
    use konnect_route::detail::negotiate::{negotiate, NegotiationSettings};
    use konnect_route::global::{build_grid, route_global, GlobalSettings};
    let conn = ConnectivityReport::build(board, &konnect_geom::ArcPolicy::default());
    let connections = conn.connections(board);
    let settings = GlobalSettings::default();
    let grid = guarded("global", || build_grid(board, rules, &settings))?;
    let global = guarded("global", || {
        route_global(board, rules, &grid, &connections, &settings)
    })?;
    let mut backend = konnect_route::compute::CpuCompute::new();
    let detail = guarded("detail", || {
        negotiate(
            board,
            rules,
            &global,
            &NegotiationSettings::default(),
            &mut backend,
        )
    })?;
    Ok(detail.copper)
}

/// Run the pipeline for `spec`, resuming from `resume` when it matches.
/// Returns the report; a stage that cannot run ends the run with
/// `stopped` set rather than an error, so what ran is still reported.
pub fn run_pipeline(
    spec: &NativeJobSpec,
    resume: Option<Checkpoint>,
    observer: &mut dyn PipelineObserver,
) -> Result<PipelineReport> {
    spec.validate()?;
    let started = Instant::now();
    let mut report = PipelineReport::default();
    let spec_hash = spec.hash();
    let resume = resume.filter(|c| c.spec_hash == spec_hash);
    let done = |name: &str| {
        resume
            .as_ref()
            .is_some_and(|c| c.completed.iter().any(|s| s == name))
    };
    let record = |report: &mut PipelineReport,
                  observer: &mut dyn PipelineObserver,
                  stage: &str,
                  state: &str,
                  t: Instant,
                  detail: Value| {
        observer.event(stage, state, &detail);
        report.stages.push(StageRecord {
            stage: stage.to_string(),
            state: state.to_string(),
            wall_ms: t.elapsed().as_millis() as u64,
            detail,
        });
    };
    let stop_check = |observer: &mut dyn PipelineObserver| -> Option<String> {
        if let Some(why) = observer.should_stop() {
            return Some(why);
        }
        spec.time_limit_seconds
            .filter(|&s| started.elapsed() >= Duration::from_secs(s))
            .map(|s| format!("time limit of {s} s reached"))
    };

    // S0: ingest and rules.
    let t = Instant::now();
    observer.event("ingest", "start", &json!({}));
    let (mut board, load, rules) = spec.load_inputs()?;
    report
        .telemetry
        .add_stage_time(Stage::Ingest, t.elapsed().as_micros() as u64);
    record(
        &mut report,
        observer,
        "ingest",
        "done",
        t,
        json!({
            "pads": board.pads.len(), "nets": board.nets.len(),
            "tracks": board.tracks.len(), "vias": board.vias.len(),
            "zones": board.zones.len(), "skipped": load.skipped_total(),
            "inert_rules": rules.inert_rules().len(),
        }),
    );

    // S1: connectivity.
    let t = Instant::now();
    let conn = ConnectivityReport::build(&board, &konnect_geom::ArcPolicy::default());
    report.connections_needed = Some(conn.connection_count());
    report
        .telemetry
        .add_stage_time(Stage::Audit, t.elapsed().as_micros() as u64);
    record(
        &mut report,
        observer,
        "connectivity",
        "done",
        t,
        json!({ "connections_needed": conn.connection_count(),
                "unfilled_zones_skipped": conn.unfilled_zones_skipped }),
    );

    let region = spec.region()?;
    let nets: Option<BTreeSet<NetId>> = spec
        .nets
        .as_ref()
        .map(|names| names.iter().filter_map(|n| board.net_by_name(n)).collect());
    if let (Some(names), Some(found)) = (&spec.nets, &nets) {
        if found.len() != names.len() {
            let missing: Vec<&String> = names
                .iter()
                .filter(|n| board.net_by_name(n).is_none())
                .collect();
            bail!("nets not on the board: {missing:?}");
        }
    }

    // Route (or take) the copper.
    let mut checkpoint = Checkpoint {
        spec_hash: spec_hash.clone(),
        completed: Vec::new(),
        copper: RoutedCopper::default(),
        frozen: RoutedCopper::default(),
        optimize: None,
        check: None,
        written_unix: 0,
    };
    let pool = {
        let mut b = rayon::ThreadPoolBuilder::new();
        if let Some(n) = spec.threads {
            b = b.num_threads(n);
        }
        b.build().context("start the worker pool")?
    };
    report.telemetry.threads = pool.current_num_threads() as u32;

    let t = Instant::now();
    let (mut work, frozen) = if let Some(cp) = resume.as_ref().filter(|_| done("route")) {
        // The board still holds the copper the source took; take it off
        // again so it is not doubled, then freeze what the run froze.
        if spec.source == CopperSource::BoardUnlocked {
            let _ = RoutedCopper::take_unlocked(&mut board, None, Generation(0));
        }
        checkpoint.completed.push("route".to_string());
        record(
            &mut report,
            observer,
            "route",
            "resumed",
            t,
            json!({ "tracks": cp.copper.tracks.len(), "vias": cp.copper.vias.len() }),
        );
        (cp.copper.clone(), cp.frozen.clone())
    } else {
        observer.event("route", "start", &json!({ "source": spec.source }));
        let routed = match &spec.source {
            CopperSource::BoardUnlocked => {
                Ok(RoutedCopper::take_unlocked(&mut board, None, Generation(0)))
            }
            CopperSource::Plan { path } => {
                let text = std::fs::read_to_string(path)
                    .with_context(|| format!("read {}", path.display()))?;
                let plan: RoutePlan = serde_json::from_str(&text)
                    .with_context(|| format!("parse {}", path.display()))?;
                plan.to_routed(&board, Generation(0))
                    .map_err(|e| anyhow::anyhow!("source plan: {e}"))
            }
            CopperSource::Native => pool.install(|| route_native(&board, &rules)),
        };
        match routed {
            Ok(c) => {
                report
                    .telemetry
                    .add_stage_time(Stage::Detail, t.elapsed().as_micros() as u64);
                checkpoint.completed.push("route".to_string());
                record(
                    &mut report,
                    observer,
                    "route",
                    "done",
                    t,
                    json!({ "tracks": c.tracks.len(), "vias": c.vias.len() }),
                );
                // Copper outside the region or nets is frozen as board
                // copper: an obstacle and terminal, never edited or planned.
                c.partition(nets.as_ref(), region)
            }
            Err(e) => {
                record(
                    &mut report,
                    observer,
                    "route",
                    "failed",
                    t,
                    json!({ "error": format!("{e:#}") }),
                );
                report.stopped = Some(format!("{e:#}"));
                return Ok(report);
            }
        }
    };
    frozen.freeze_into(&mut board);
    report.work_tracks = work.tracks.len();
    report.work_vias = work.vias.len();
    checkpoint.copper = work.clone();
    checkpoint.frozen = frozen;
    checkpoint.written_unix = now_unix();
    observer.checkpoint(&checkpoint);
    if let Some(why) = stop_check(observer) {
        report.stopped = Some(why);
        return Ok(report);
    }

    let checker_opts = CheckerOptions {
        zones_are_obstacles: spec.zones_are_obstacles,
        ..CheckerOptions::default()
    };
    let checker = ExactChecker::new(&board, &rules, checker_opts);

    // S6: optimize.
    let t = Instant::now();
    if !spec.optimize {
        record(&mut report, observer, "optimize", "skipped", t, json!({}));
    } else if done("optimize") {
        report.optimize = resume.as_ref().and_then(|c| c.optimize);
        checkpoint.optimize = report.optimize;
        checkpoint.completed.push("optimize".to_string());
        record(&mut report, observer, "optimize", "resumed", t, json!({}));
    } else {
        observer.event("optimize", "start", &json!({}));
        let opts = OptimizeOptions {
            checker: checker_opts,
            ..OptimizeOptions::default()
        };
        let r = pool.install(|| optimize_checked(&checker, &mut work, &opts));
        report
            .telemetry
            .add_stage_time(Stage::Optimize, t.elapsed().as_micros() as u64);
        report
            .telemetry
            .count("optimize_exact_checks", r.exact_checks);
        report.optimize = Some(r);
        checkpoint.optimize = Some(r);
        checkpoint.copper = work.clone();
        checkpoint.completed.push("optimize".to_string());
        checkpoint.written_unix = now_unix();
        observer.checkpoint(&checkpoint);
        record(
            &mut report,
            observer,
            "optimize",
            "done",
            t,
            serde_json::to_value(r).unwrap_or_default(),
        );
    }
    if let Some(why) = stop_check(observer) {
        report.stopped = Some(why);
        return Ok(report);
    }

    // S7: check.
    let t = Instant::now();
    observer.event("check", "start", &json!({}));
    let summary = match guarded("check", || {
        konnect_route::check::check_routed(&board, &rules, &work)
    }) {
        Ok(r) => CheckSummary {
            checker: "check_routed".to_string(),
            authoritative: true,
            violations: r.violations.len(),
            by_kind: by_kind(&r),
        },
        Err(_) => {
            let r = pool.install(|| checker.check_copper(&work));
            CheckSummary {
                checker: "interim_exact".to_string(),
                authoritative: false,
                violations: r.violations.len(),
                by_kind: by_kind(&r),
            }
        }
    };
    report
        .telemetry
        .add_stage_time(Stage::Check, t.elapsed().as_micros() as u64);
    record(
        &mut report,
        observer,
        "check",
        "done",
        t,
        serde_json::to_value(&summary).unwrap_or_default(),
    );
    checkpoint.check = Some(summary.clone());
    report.check = Some(summary);
    checkpoint.completed.push("check".to_string());
    checkpoint.written_unix = now_unix();
    observer.checkpoint(&checkpoint);

    // S8: plan.
    let t = Instant::now();
    if work.tracks.is_empty() && work.vias.is_empty() {
        record(
            &mut report,
            observer,
            "plan",
            "skipped",
            t,
            json!({ "reason": "no copper to plan" }),
        );
        return Ok(report);
    }
    let plan = RoutePlan::from_routed(&board, &work).map_err(|e| anyhow::anyhow!("plan: {e}"))?;
    let chunks = plan.chunks_by_net(ADD_ROUTING_MAX_ITEMS).len();
    write_new_atomic(&spec.output_plan, plan.to_routing_file_json().as_bytes())?;
    report
        .telemetry
        .add_stage_time(Stage::Plan, t.elapsed().as_micros() as u64);
    report.plan_items = Some(plan.item_count());
    report.plan_chunks = Some(chunks);
    report.plan_path = Some(spec.output_plan.clone());
    record(
        &mut report,
        observer,
        "plan",
        "done",
        t,
        json!({ "items": plan.item_count(), "add_routing_calls": chunks,
                "path": spec.output_plan }),
    );
    Ok(report)
}

// ─── Job directory ───────────────────────────────────────────────────────

/// The runner's state, as `status.json` holds it.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct JobStatus {
    /// STARTING, RUNNING, COMPLETED, STOPPED_ON_REQUEST,
    /// STOPPED_AT_TIME_LIMIT, STAGE_UNAVAILABLE or FAILED.
    pub state: String,
    pub finished: bool,
    pub stage: Option<String>,
    pub runner_pid: u32,
    pub started_unix: u64,
    pub heartbeat_unix: u64,
    pub elapsed_seconds: f64,
    pub resumed: bool,
    pub output_plan: PathBuf,
    pub plan_items: Option<usize>,
    pub error: Option<String>,
}

pub fn job_dir_for(output_plan: &Path) -> PathBuf {
    let mut name = output_plan
        .file_name()
        .map(|n| n.to_os_string())
        .unwrap_or_default();
    name.push(".konnect-native-job");
    output_plan.with_file_name(name)
}

fn now_unix() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_secs())
}

fn write_atomic(path: &Path, bytes: &[u8]) -> Result<()> {
    let mut tmp = path.as_os_str().to_os_string();
    tmp.push(".tmp");
    let tmp = PathBuf::from(tmp);
    std::fs::write(&tmp, bytes).with_context(|| format!("write {}", tmp.display()))?;
    std::fs::rename(&tmp, path).with_context(|| format!("replace {}", path.display()))
}

/// Write a file that must not exist yet (never replaces one).
fn write_new_atomic(path: &Path, bytes: &[u8]) -> Result<()> {
    if path.exists() {
        bail!("{} already exists; it is never replaced", path.display());
    }
    write_atomic(path, bytes)
}

fn read_json<T: for<'de> Deserialize<'de>>(path: &Path) -> Result<T> {
    // The runner replaces files as it goes; a read can meet the rename.
    let mut last = None;
    for _ in 0..5 {
        match std::fs::read_to_string(path) {
            Ok(text) => {
                return serde_json::from_str(&text)
                    .with_context(|| format!("parse {}", path.display()))
            }
            Err(e) => last = Some(e),
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    Err(last.expect("five attempts")).with_context(|| format!("read {}", path.display()))
}

/// The runner's side of the job directory.
struct DirObserver {
    dir: PathBuf,
    status: Arc<Mutex<JobStatus>>,
    started: Instant,
}

impl DirObserver {
    fn write_status(dir: &Path, status: &Mutex<JobStatus>, started: Instant) {
        let mut s = status.lock().unwrap_or_else(|e| e.into_inner());
        s.heartbeat_unix = now_unix();
        s.elapsed_seconds = started.elapsed().as_secs_f64();
        if let Ok(bytes) = serde_json::to_vec_pretty(&*s) {
            if let Err(e) = write_atomic(&dir.join(STATUS_FILE), &bytes) {
                eprintln!("status not written: {e:#}");
            }
        }
    }
}

impl PipelineObserver for DirObserver {
    fn event(&mut self, stage: &str, event: &str, detail: &Value) {
        let line = json!({
            "unix": now_unix(),
            "elapsed_ms": self.started.elapsed().as_millis() as u64,
            "stage": stage, "event": event, "detail": detail,
        });
        let appended = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(self.dir.join(PROGRESS_FILE))
            .and_then(|mut f| writeln!(f, "{line}"));
        if let Err(e) = appended {
            eprintln!("progress not written: {e}");
        }
        {
            let mut s = self.status.lock().unwrap_or_else(|e| e.into_inner());
            s.state = "RUNNING".to_string();
            s.stage = Some(stage.to_string());
        }
        Self::write_status(&self.dir, &self.status, self.started);
    }

    fn should_stop(&mut self) -> Option<String> {
        if self.dir.join(STOP_FILE).exists() {
            return Some("stopped on request".to_string());
        }
        None
    }

    fn checkpoint(&mut self, checkpoint: &Checkpoint) {
        match serde_json::to_vec(checkpoint) {
            Ok(bytes) => {
                if let Err(e) = write_atomic(&self.dir.join(CHECKPOINT_FILE), &bytes) {
                    eprintln!("checkpoint not written: {e:#}");
                }
            }
            Err(e) => eprintln!("checkpoint not serialised: {e}"),
        }
    }
}

/// Run the job in `dir` in this process: what the runner subcommand does.
/// Exposed so tests (and a caller without a separate process) can drive a
/// job directory directly.
pub fn run_job_dir(dir: &Path) -> Result<JobStatus> {
    let spec: NativeJobSpec = read_json(&dir.join(SPEC_FILE))?;
    let resume: Option<Checkpoint> = read_json(&dir.join(CHECKPOINT_FILE)).ok();
    let started = Instant::now();
    let status = Arc::new(Mutex::new(JobStatus {
        state: "STARTING".to_string(),
        runner_pid: std::process::id(),
        started_unix: now_unix(),
        resumed: resume.is_some(),
        output_plan: spec.output_plan.clone(),
        ..JobStatus::default()
    }));
    DirObserver::write_status(dir, &status, started);
    // Heartbeat while a long stage runs.
    let beating = Arc::new(std::sync::atomic::AtomicBool::new(true));
    let heart = {
        let (dir, status, beating) = (dir.to_path_buf(), status.clone(), beating.clone());
        std::thread::spawn(move || {
            while beating.load(std::sync::atomic::Ordering::Relaxed) {
                std::thread::sleep(HEARTBEAT);
                DirObserver::write_status(&dir, &status, started);
            }
        })
    };
    let mut observer = DirObserver {
        dir: dir.to_path_buf(),
        status: status.clone(),
        started,
    };
    let result = run_pipeline(&spec, resume, &mut observer);
    beating.store(false, std::sync::atomic::Ordering::Relaxed);
    let _ = heart.join();
    {
        let mut s = status.lock().unwrap_or_else(|e| e.into_inner());
        s.finished = true;
        match &result {
            Ok(report) => {
                s.plan_items = report.plan_items;
                s.state = match &report.stopped {
                    None => "COMPLETED".to_string(),
                    Some(why) if why.starts_with("stopped on request") => {
                        "STOPPED_ON_REQUEST".to_string()
                    }
                    Some(why) if why.starts_with("time limit") => {
                        "STOPPED_AT_TIME_LIMIT".to_string()
                    }
                    Some(_) => "STAGE_UNAVAILABLE".to_string(),
                };
                s.error = report.stopped.clone();
                if let Ok(bytes) = serde_json::to_vec_pretty(report) {
                    if let Err(e) = write_atomic(&dir.join(REPORT_FILE), &bytes) {
                        eprintln!("report not written: {e:#}");
                    }
                }
            }
            Err(e) => {
                s.state = "FAILED".to_string();
                s.error = Some(format!("{e:#}"));
            }
        }
    }
    DirObserver::write_status(dir, &status, started);
    let s = status.lock().unwrap_or_else(|e| e.into_inner()).clone();
    Ok(s)
}

/// `konnect native-route-runner <job-dir>`.
pub fn run_runner(args: &[String]) -> Result<()> {
    let dir = PathBuf::from(
        args.first()
            .context("usage: konnect native-route-runner <job-directory>")?,
    );
    run_job_dir(&dir).map(|_| ())
}

/// What the tools report about a job.
fn report(dir: &Path, status: &JobStatus) -> Value {
    let age = now_unix().saturating_sub(status.heartbeat_unix);
    let lost = !status.finished && age > STALE_HEARTBEAT_SECONDS;
    let mut body = serde_json::to_value(status).unwrap_or_else(|_| json!({}));
    if lost {
        body["state"] = json!("RUNNER_LOST");
        body["note"] = json!(format!(
            "The runner has not written its status for {age} s: it ended without finishing. \
             Start the job again with resume: true to continue from its checkpoint."
        ));
    }
    body["job_dir"] = json!(dir);
    body["heartbeat_age_seconds"] = json!(age);
    body["runner_alive"] = json!(!status.finished && !lost);
    body["stop_requested"] = json!(dir.join(STOP_FILE).exists());
    body["checkpoint_exists"] = json!(dir.join(CHECKPOINT_FILE).is_file());
    body["plan_exists"] = json!(status.output_plan.is_file());
    let report_path = dir.join(REPORT_FILE);
    if let Some(r) = report_path
        .is_file()
        .then(|| read_json::<Value>(&report_path).ok())
        .flatten()
    {
        body["report"] = json!({
            "stages": r["stages"], "optimize": r["optimize"], "check": r["check"],
            "plan_items": r["plan_items"], "plan_chunks": r["plan_chunks"],
            "stopped": r["stopped"],
            "stage_wall_us": r["telemetry"]["stages"], "threads": r["telemetry"]["threads"],
        });
    }
    body
}

/// Prepare the job directory for `spec` (refusing a clash) and return it.
fn prepare(spec: &NativeJobSpec, resume: bool) -> Result<PathBuf> {
    spec.validate()?;
    let dir = job_dir_for(&spec.output_plan);
    if spec.output_plan.exists() {
        bail!(
            "{} already exists; choose a new output_plan (plans are never replaced)",
            spec.output_plan.display()
        );
    }
    if dir.exists() {
        if !resume {
            let state = read_json::<JobStatus>(&dir.join(STATUS_FILE))
                .map(|s| report(&dir, &s)["state"].clone())
                .unwrap_or_else(|_| json!("unknown"));
            bail!(
                "a job directory already exists for this output ({}, state {state}); pass \
                 resume: true to continue it, or remove the directory",
                dir.display()
            );
        }
        if let Ok(status) = read_json::<JobStatus>(&dir.join(STATUS_FILE)) {
            if report(&dir, &status)["runner_alive"] == json!(true) {
                bail!("the job in {} is still running", dir.display());
            }
        }
        let old: NativeJobSpec = read_json(&dir.join(SPEC_FILE))?;
        if old != *spec {
            bail!("resume needs the same spec the job was started with");
        }
        let _ = std::fs::remove_file(dir.join(STOP_FILE));
    } else {
        std::fs::create_dir_all(&dir).with_context(|| format!("create {}", dir.display()))?;
        write_atomic(&dir.join(SPEC_FILE), &serde_json::to_vec_pretty(spec)?)?;
    }
    Ok(dir)
}

/// Start a runner for `spec` with `executable` (the konnect binary).
pub async fn start_with(spec: &NativeJobSpec, resume: bool, executable: &Path) -> Result<Value> {
    let dir = prepare(spec, resume)?;
    let log = std::fs::File::create(dir.join(RUNNER_LOG)).context("create runner.log")?;
    let mut command = std::process::Command::new(executable);
    command
        .arg(RUNNER_SUBCOMMAND)
        .arg(&dir)
        .stdin(std::process::Stdio::null())
        .stdout(std::process::Stdio::null())
        .stderr(log);
    let (pid, left_client_job) = crate::freerouting_job::spawn_detached(&mut command)
        .with_context(|| format!("start {} {RUNNER_SUBCOMMAND}", executable.display()))?;
    let waited = Instant::now();
    let status = loop {
        tokio::time::sleep(Duration::from_millis(200)).await;
        let status = read_json::<JobStatus>(&dir.join(STATUS_FILE)).ok();
        let started = status
            .as_ref()
            .is_some_and(|s| s.runner_pid == pid && (s.finished || s.state != "STARTING"));
        if started || waited.elapsed() >= START_WAIT {
            break status;
        }
    };
    let mut body = match &status {
        Some(s) => report(&dir, s),
        None => json!({ "state": "STARTING", "job_dir": dir }),
    };
    body["runner_pid"] = json!(pid);
    body["left_client_job"] = json!(left_client_job);
    if !left_client_job {
        body["warning"] = json!(
            "The client's job object forbids breakaway: this run ends if the client's session \
             ends. Its checkpoint is kept; start it again with resume: true."
        );
    }
    body["success"] = json!(status.as_ref().is_none_or(|s| s.state != "FAILED"));
    Ok(body)
}

pub async fn start(spec: &NativeJobSpec, resume: bool) -> Result<Value> {
    let exe = std::env::current_exe().context("locate the konnect executable")?;
    start_with(spec, resume, &exe).await
}

pub fn status(output_plan: &Path) -> Result<Value> {
    let dir = job_dir_for(output_plan);
    if !dir.is_dir() {
        bail!(
            "no native route job for {} (no {})",
            output_plan.display(),
            dir.display()
        );
    }
    let status: JobStatus = read_json(&dir.join(STATUS_FILE))?;
    Ok(report(&dir, &status))
}

pub async fn stop(output_plan: &Path, wait: Duration) -> Result<Value> {
    let dir = job_dir_for(output_plan);
    if !dir.is_dir() {
        bail!(
            "no native route job for {} (no {})",
            output_plan.display(),
            dir.display()
        );
    }
    let status: JobStatus = read_json(&dir.join(STATUS_FILE))?;
    if status.finished {
        let mut body = report(&dir, &status);
        body["note"] = json!("The job had already finished; nothing to stop.");
        return Ok(body);
    }
    std::fs::write(dir.join(STOP_FILE), b"")
        .with_context(|| format!("create {}", dir.join(STOP_FILE).display()))?;
    let waited = Instant::now();
    loop {
        let status: JobStatus = read_json(&dir.join(STATUS_FILE))?;
        let body = report(&dir, &status);
        if status.finished || waited.elapsed() >= wait || body["state"] == "RUNNER_LOST" {
            return Ok(body);
        }
        tokio::time::sleep(Duration::from_millis(250)).await;
    }
}

#[cfg(test)]
mod tests;
