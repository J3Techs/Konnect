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
use konnect_route::optimize::{OptimizeOptions, OptimizeReport};
use konnect_route::pipeline::{
    AllowanceReport, ClassFallback, PipelineSettings, ReferenceReport, RoutingAllowances,
};
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
    /// After S6, take out each piece of routed copper S7 still finds a
    /// defect in ([`konnect_route::pipeline::clear_findings`]), leaving its
    /// connections open. Unset: on for native routing, off for copper taken
    /// from a plan or the board (which is checked, not edited away).
    #[serde(default)]
    pub clear_findings: Option<bool>,
    /// Stop at the next stage boundary after this long.
    #[serde(default)]
    pub time_limit_seconds: Option<u64>,
    /// Owner routing allowances and the class policy, by name (class
    /// layers, layer fallbacks, neck-down, excluded classes). Native
    /// routing only. Default: none, and wide-copper classes excluded
    /// ([`konnect_route::pipeline::default_excluded_classes`]).
    #[serde(default, skip_serializing_if = "RoutingAllowances::is_default")]
    pub allowances: RoutingAllowances,
    /// Each net's effective class as KiCad reported it over IPC when the
    /// job was started: KiCad's class name, then its constituents (highest
    /// precedence first). Overrides the project's patterns; unset, the
    /// patterns are resolved offline.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub net_classes: Option<BTreeMap<String, Vec<String>>>,
    /// Write a native run's plane fanout (S2) to this new routing_file, to
    /// apply locked, and leave it out of `output_plan`. Unset, the fanout
    /// leads `output_plan`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fanout_plan: Option<PathBuf>,
}

fn yes() -> bool {
    true
}

/// Where a run's net classes came from, and how KiCad's differed.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct NetClassSource {
    /// `kicad_ipc` or `project_patterns`.
    pub source: String,
    /// Board nets given KiCad's class.
    #[serde(default)]
    pub nets: usize,
    /// Of those, nets whose class differs from the project's patterns.
    #[serde(default)]
    pub changed: usize,
    /// Board nets KiCad did not report (they keep the patterns' class).
    #[serde(default)]
    pub missing: usize,
    /// KiCad class names the project does not define (their nets keep the
    /// patterns' class).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub unknown_classes: Vec<String>,
}

/// Put every board net KiCad named in the first of its names the project
/// defines (KiCad's effective class, else its constituents in order).
fn pin_kicad_classes(
    board: &RouteBoard,
    rules: &mut ProjectRuleTable,
    map: &BTreeMap<String, Vec<String>>,
) -> NetClassSource {
    let mut out = NetClassSource {
        source: "kicad_ipc".to_string(),
        ..NetClassSource::default()
    };
    let mut unknown: BTreeSet<String> = BTreeSet::new();
    for net in &board.nets {
        if net.name.is_empty() {
            continue;
        }
        let Some(names) = map.get(&net.name) else {
            out.missing += 1;
            continue;
        };
        let Some(class) = names
            .iter()
            .find(|n| rules.classes().iter().any(|c| &c.name == *n))
        else {
            if let Some(n) = names.first() {
                unknown.insert(n.clone());
            }
            continue;
        };
        let before = rules.class(rules.class_of(Some(net.id))).name.clone();
        if rules.pin_net_class(net.id, class) {
            out.nets += 1;
            out.changed += usize::from(&before != class);
        }
    }
    out.unknown_classes = unknown.into_iter().collect();
    out
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

    /// Read the board and compile its project rules (classes assigned:
    /// KiCad's own when [`Self::net_classes`] holds them, else the
    /// project's patterns).
    pub(crate) fn load_inputs(
        &self,
    ) -> Result<(RouteBoard, konnect_route::LoadReport, ProjectRuleTable)> {
        self.load_inputs_reported().map(|(b, l, r, _)| (b, l, r))
    }

    /// [`Self::load_inputs`], with where the classes came from.
    pub(crate) fn load_inputs_reported(
        &self,
    ) -> Result<(
        RouteBoard,
        konnect_route::LoadReport,
        ProjectRuleTable,
        NetClassSource,
    )> {
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
        let mut rules = ProjectRuleTable::from_project(&pro_text, dru_text.as_deref(), &board)
            .context("compile the project rules")?;
        let source = match &self.net_classes {
            None => NetClassSource {
                source: "project_patterns".to_string(),
                ..NetClassSource::default()
            },
            Some(map) => pin_kicad_classes(&board, &mut rules, map),
        };
        rules.assign_classes(&mut board);
        Ok((board, load, rules, source))
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
        if let Some(f) = &self.fanout_plan {
            if *f == self.output_plan {
                bail!("fanout_plan must differ from output_plan");
            }
            let parent = f.parent().unwrap_or(Path::new("."));
            if !parent.as_os_str().is_empty() && !parent.is_dir() {
                bail!(
                    "fanout_plan's directory does not exist: {}",
                    parent.display()
                );
            }
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
    /// Set when `check_routed` is built and panicked: the interim result
    /// is reported beside it, and the run stops as FAILED.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub check_routed_error: Option<String>,
}

/// The pipeline's whole account of a run.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct PipelineReport {
    pub stages: Vec<StageRecord>,
    pub connections_needed: Option<usize>,
    pub work_tracks: usize,
    pub work_vias: usize,
    /// Plane fanout copper a native run planned beside the work.
    #[serde(default)]
    pub fanout_tracks: usize,
    #[serde(default)]
    pub fanout_vias: usize,
    pub optimize: Option<OptimizeReport>,
    /// Nets S6's guard put back to their routed copper.
    #[serde(default)]
    pub optimize_reverted: usize,
    /// What the clear stage took out (`None` when it did not run).
    #[serde(default)]
    pub cleared: Option<konnect_route::pipeline::ClearReport>,
    pub check: Option<CheckSummary>,
    pub plan_items: Option<usize>,
    pub plan_chunks: Option<usize>,
    pub plan_path: Option<PathBuf>,
    /// Set when the run stopped early (a stage failed, a stop was asked, the
    /// time limit passed).
    pub stopped: Option<String>,
    /// Why, typed; set whenever `stopped` is.
    #[serde(default)]
    pub stop_kind: Option<StopKind>,
    /// Whether the run continued from a checkpoint of the same spec.
    #[serde(default)]
    pub resumed: bool,
    /// Where the net classes came from (KiCad over IPC, or the patterns).
    #[serde(default)]
    pub net_classes: Option<NetClassSource>,
    /// The allowances and class policy a native run used, by name.
    #[serde(default)]
    pub allowances: Option<AllowanceReport>,
    /// Open connections left unrouted by the class policy, per class (not
    /// failures).
    #[serde(default)]
    pub excluded_by_policy: BTreeMap<String, usize>,
    /// The reference-plane check after S6 (when a fallback names one).
    #[serde(default)]
    pub reference: Option<ReferenceReport>,
    /// The separate fanout plan, when one was asked for and written.
    #[serde(default)]
    pub fanout_plan_path: Option<PathBuf>,
    pub telemetry: Telemetry,
}

impl PipelineReport {
    fn stop(&mut self, kind: StopKind, why: String) {
        self.stop_kind = Some(kind);
        self.stopped = Some(why);
    }
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
    /// Plane fanout (S2) of a native run: board copper for the later
    /// stages (never optimized), but part of the plan.
    #[serde(default)]
    pub fanout: RoutedCopper,
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

/// A stage that panicked, told apart by why.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum StageError {
    /// The stage is a stub not built yet (its `todo!` or `unimplemented!`).
    Unavailable { stage: String, message: String },
    /// The stage is built and panicked: a bug, never hidden as unavailable.
    Panicked { stage: String, message: String },
}

impl std::fmt::Display for StageError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            StageError::Unavailable { stage, message } => {
                write!(f, "stage {stage} is not available: {message}")
            }
            StageError::Panicked { stage, message } => {
                write!(f, "stage {stage} panicked: {message}")
            }
        }
    }
}

impl std::error::Error for StageError {}

/// Run `f`, turning a panic into a [`StageError`]: an unbuilt stage's
/// `todo!` ("not yet implemented") or `unimplemented!` ("not implemented")
/// is [`StageError::Unavailable`]; any other panic is
/// [`StageError::Panicked`].
pub(crate) fn guarded<T>(stage: &str, f: impl FnOnce() -> T) -> Result<T, StageError> {
    std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)).map_err(|payload| {
        let message = payload
            .downcast_ref::<String>()
            .cloned()
            .or_else(|| payload.downcast_ref::<&str>().map(|s| s.to_string()))
            .unwrap_or_else(|| "panicked".to_string());
        let stage = stage.to_string();
        if message.starts_with("not yet implemented") || message.starts_with("not implemented") {
            StageError::Unavailable { stage, message }
        } else {
            StageError::Panicked { stage, message }
        }
    })
}

/// Why a run ended before planning.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StopKind {
    /// A stage is not built yet (job state STAGE_UNAVAILABLE).
    StageUnavailable,
    /// A built stage failed or panicked (FAILED).
    StageFailed,
    /// The input cannot be used, e.g. a source plan naming a net the board
    /// lacks (FAILED).
    InputError,
    /// A stop was requested (STOPPED_ON_REQUEST).
    StopRequested,
    /// The time limit passed (STOPPED_AT_TIME_LIMIT).
    TimeLimit,
}

impl StopKind {
    fn of(error: &anyhow::Error) -> StopKind {
        match error.downcast_ref::<StageError>() {
            Some(StageError::Unavailable { .. }) => StopKind::StageUnavailable,
            Some(StageError::Panicked { .. }) => StopKind::StageFailed,
            None => StopKind::InputError,
        }
    }

    /// The job state a run that stopped this way ends in.
    pub fn job_state(self) -> &'static str {
        match self {
            StopKind::StageUnavailable => "STAGE_UNAVAILABLE",
            StopKind::StageFailed | StopKind::InputError => "FAILED",
            StopKind::StopRequested => "STOPPED_ON_REQUEST",
            StopKind::TimeLimit => "STOPPED_AT_TIME_LIMIT",
        }
    }
}

fn has_reference(fallback: &BTreeMap<konnect_route::api::ClassId, ClassFallback>) -> bool {
    fallback.values().any(|f| f.reference.is_some())
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

/// What the native stages produced.
struct NativeRoute {
    /// S2 plane fanout.
    fanout: RoutedCopper,
    /// S5 copper, not yet optimized (the job's own S6 does that).
    routed: RoutedCopper,
    /// Open connections the class policy left out, per class.
    excluded: BTreeMap<String, usize>,
    /// Counts for the `route` stage record.
    detail: Value,
    telemetry: Telemetry,
}

impl NativeRoute {
    /// Existing copper taken as the work (no fanout, no stage telemetry).
    fn taken(routed: RoutedCopper) -> Self {
        NativeRoute {
            fanout: RoutedCopper::default(),
            routed,
            excluded: BTreeMap::new(),
            detail: json!({}),
            telemetry: Telemetry::default(),
        }
    }
}

/// The native S1–S5 stages ([`konnect_route::pipeline::route_board`]) on the
/// board's open connections, limited to `nets` and `region`: the stages only
/// ever see the selected connections. The job runs S6–S8 itself, so it can
/// checkpoint between them.
fn route_native(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    nets: Option<&BTreeSet<NetId>>,
    region: Option<konnect_geom::BBox>,
    settings: &PipelineSettings,
) -> Result<NativeRoute> {
    use konnect_route::pipeline::{route_board, PipelineScope};
    let scope = PipelineScope {
        nets: nets.cloned(),
        region,
    };
    let r = guarded("route", || route_board(board, rules, &scope, settings))?
        .map_err(|e| anyhow::anyhow!("route: {e}"))?;
    let mut failed: BTreeMap<&str, usize> = BTreeMap::new();
    for (_, d) in &r.detail.failure_details {
        *failed.entry(d.name()).or_default() += 1;
    }
    let mut excluded: BTreeMap<String, usize> = BTreeMap::new();
    for c in &r.excluded {
        *excluded
            .entry(rules.class(rules.class_of(Some(c.net))).name.clone())
            .or_default() += 1;
    }
    let detail = json!({
        "connections_before_fanout": r.connections_before_fanout,
        "fanout_sites": r.fanout.as_ref().map_or(0, |f| f.sites.len()),
        "fanout_failed": r.fanout.as_ref().map_or(0, |f| f.failed.len()),
        "tasks": r.connections.len(),
        "global_overflow": r.global.stats.total_overflow,
        "global_failed": r.global.failed.len(),
        "routed": r.detail.routed.len(),
        "failed": failed,
        "iterations": r.detail.iterations,
        "connections_after": r.connections_after,
        "excluded_by_policy": excluded,
        "fallback_routed": r.telemetry.counters.get("pipeline_fallback_routed").copied().unwrap_or(0),
    });
    Ok(NativeRoute {
        fanout: r.fanout_copper,
        routed: r.routed,
        excluded,
        detail,
        telemetry: r.telemetry,
    })
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
    report.resumed = resume.is_some();
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
    let stop_check = |observer: &mut dyn PipelineObserver| -> Option<(StopKind, String)> {
        if let Some(why) = observer.should_stop() {
            return Some((StopKind::StopRequested, why));
        }
        spec.time_limit_seconds
            .filter(|&s| started.elapsed() >= Duration::from_secs(s))
            .map(|s| (StopKind::TimeLimit, format!("time limit of {s} s reached")))
    };

    // S0: ingest and rules.
    let t = Instant::now();
    observer.event("ingest", "start", &json!({}));
    let (mut board, load, rules, classes) = spec.load_inputs_reported()?;
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
            "net_classes": classes,
        }),
    );
    report.net_classes = Some(classes);

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
        fanout: RoutedCopper::default(),
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

    // The route stage's settings: the job runs S6, the clear stage and S7
    // itself. The allowances also drive the reference check after S6.
    let mut settings = PipelineSettings {
        optimize: None,
        clear_findings: false,
        ..PipelineSettings::default()
    };
    if spec.source == CopperSource::Native {
        match spec
            .allowances
            .resolve(&board, rules.classes(), &mut settings)
        {
            Ok(a) => report.allowances = Some(a),
            Err(e) => {
                report.stop(StopKind::InputError, format!("allowances: {e}"));
                return Ok(report);
            }
        }
    }

    let t = Instant::now();
    let (mut work, frozen, fanout) = if let Some(cp) = resume.as_ref().filter(|_| done("route")) {
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
            json!({ "tracks": cp.copper.tracks.len(), "vias": cp.copper.vias.len(),
                    "fanout_tracks": cp.fanout.tracks.len(), "fanout_vias": cp.fanout.vias.len() }),
        );
        (cp.copper.clone(), cp.frozen.clone(), cp.fanout.clone())
    } else {
        observer.event("route", "start", &json!({ "source": spec.source }));
        let routed: Result<NativeRoute> = match &spec.source {
            CopperSource::BoardUnlocked => Ok(NativeRoute::taken(RoutedCopper::take_unlocked(
                &mut board,
                None,
                Generation(0),
            ))),
            CopperSource::Plan { path } => {
                let text = std::fs::read_to_string(path)
                    .with_context(|| format!("read {}", path.display()))?;
                let plan: RoutePlan = serde_json::from_str(&text)
                    .with_context(|| format!("parse {}", path.display()))?;
                plan.to_routed(&board, Generation(0))
                    .map(NativeRoute::taken)
                    .map_err(|e| anyhow::anyhow!("source plan: {e}"))
            }
            CopperSource::Native => {
                pool.install(|| route_native(&board, &rules, nets.as_ref(), region, &settings))
            }
        };
        match routed {
            Ok(r) => {
                // Routing is S1–S5 time (the pipeline's own stage timings
                // come with it); taking existing copper is ingest.
                if spec.source == CopperSource::Native {
                    report.telemetry.merge(&r.telemetry);
                } else {
                    report
                        .telemetry
                        .add_stage_time(Stage::Ingest, t.elapsed().as_micros() as u64);
                }
                checkpoint.completed.push("route".to_string());
                report.excluded_by_policy = r.excluded.clone();
                let mut detail = json!({ "tracks": r.routed.tracks.len(), "vias": r.routed.vias.len(),
                                         "fanout_tracks": r.fanout.tracks.len(),
                                         "fanout_vias": r.fanout.vias.len() });
                if let (Some(d), Some(extra)) = (detail.as_object_mut(), r.detail.as_object()) {
                    d.extend(extra.clone());
                }
                record(&mut report, observer, "route", "done", t, detail);
                // Copper outside the region or nets is frozen as board
                // copper: an obstacle and terminal, never edited or planned.
                let (work, frozen) = r.routed.partition(nets.as_ref(), region);
                (work, frozen, r.fanout)
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
                report.stop(StopKind::of(&e), format!("{e:#}"));
                return Ok(report);
            }
        }
    };
    frozen.freeze_into(&mut board);
    // The fanout is board copper for every later stage, and planned at S8.
    fanout.freeze_into(&mut board);
    report.work_tracks = work.tracks.len();
    report.work_vias = work.vias.len();
    report.fanout_tracks = fanout.tracks.len();
    report.fanout_vias = fanout.vias.len();
    checkpoint.copper = work.clone();
    checkpoint.frozen = frozen;
    checkpoint.fanout = fanout.clone();
    checkpoint.written_unix = now_unix();
    observer.checkpoint(&checkpoint);
    if let Some((kind, why)) = stop_check(observer) {
        report.stop(kind, why);
        return Ok(report);
    }

    let checker_opts = CheckerOptions {
        zones_are_obstacles: spec.zones_are_obstacles,
        ..CheckerOptions::default()
    };
    let indexed = Instant::now();
    let checker = ExactChecker::new(&board, &rules, checker_opts);
    let index_us = indexed.elapsed().as_micros() as u64;

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
            sliver_floor_nm: konnect_route::optimize::sliver_floor(&board, &rules),
            ..OptimizeOptions::default()
        };
        // S6 then the pipeline's guard: a net the optimizer left with more
        // S7 findings goes back to the copper it was routed with.
        let negotiated = has_reference(&settings.class_fallback).then(|| work.clone());
        let (r, reverted) = pool.install(|| {
            konnect_route::pipeline::optimize_guarded(
                &board,
                &rules,
                &mut work,
                &opts,
                true,
                &mut report.telemetry,
            )
        });
        report.telemetry.count("optimize_index_us", index_us);
        // Then the reference planes: a fallback class's track S6 moved off
        // its reference goes back, or its net's copper goes.
        if let Some(before) = &negotiated {
            let rr = konnect_route::pipeline::enforce_reference(
                &board,
                &rules,
                &settings,
                Some(before),
                &mut work,
            );
            rr.record(&mut report.telemetry);
            report.reference = Some(rr);
        }
        report.optimize_reverted = reverted.len();
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
    if let Some((kind, why)) = stop_check(observer) {
        report.stop(kind, why);
        return Ok(report);
    }

    // Clear: copper S7 still finds a defect in is taken out, so the plan
    // carries none KiCad would report; its connections stay open.
    if spec
        .clear_findings
        .unwrap_or(spec.source == CopperSource::Native)
    {
        let t = Instant::now();
        let cleared = pool.install(|| {
            guarded("clear", || {
                konnect_route::pipeline::clear_findings(&board, &rules, &mut work)
            })
        });
        match cleared {
            Ok(c) => {
                c.record(&mut report.telemetry);
                report
                    .telemetry
                    .add_stage_time(Stage::Check, t.elapsed().as_micros() as u64);
                let detail = serde_json::to_value(&c).unwrap_or_default();
                report.cleared = Some(c);
                record(&mut report, observer, "clear", "done", t, detail);
            }
            Err(e) => {
                record(
                    &mut report,
                    observer,
                    "clear",
                    "failed",
                    t,
                    json!({ "error": e.to_string() }),
                );
            }
        }
    }

    // S7: check.
    let t = Instant::now();
    observer.event("check", "start", &json!({}));
    let interim = |error: Option<String>| {
        let r = pool.install(|| checker.check_copper(&work));
        CheckSummary {
            checker: "interim_exact".to_string(),
            authoritative: false,
            violations: r.violations.len(),
            by_kind: by_kind(&r),
            check_routed_error: error,
        }
    };
    let (summary, failure) = match guarded("check", || {
        konnect_route::check::check_routed(&board, &rules, &work)
    }) {
        Ok(r) => (
            CheckSummary {
                checker: "check_routed".to_string(),
                authoritative: true,
                violations: r.violations.len(),
                by_kind: by_kind(&r),
                check_routed_error: None,
            },
            None,
        ),
        // Only the unbuilt stub falls back silently.
        Err(StageError::Unavailable { .. }) => (interim(None), None),
        Err(e @ StageError::Panicked { .. }) => (interim(Some(e.to_string())), Some(e)),
    };
    report
        .telemetry
        .add_stage_time(Stage::Check, t.elapsed().as_micros() as u64);
    record(
        &mut report,
        observer,
        "check",
        if failure.is_some() { "failed" } else { "done" },
        t,
        serde_json::to_value(&summary).unwrap_or_default(),
    );
    if let Some(e) = failure {
        report.check = Some(summary);
        report.stop(StopKind::StageFailed, e.to_string());
        return Ok(report);
    }
    checkpoint.check = Some(summary.clone());
    report.check = Some(summary);
    checkpoint.completed.push("check".to_string());
    checkpoint.written_unix = now_unix();
    observer.checkpoint(&checkpoint);

    // S8: plan (the fanout in its own file when asked).
    let t = Instant::now();
    let separate = spec
        .fanout_plan
        .as_ref()
        .filter(|_| spec.source == CopperSource::Native);
    if let Some(path) = separate {
        if !fanout.tracks.is_empty() || !fanout.vias.is_empty() {
            let plan = RoutePlan::from_routed(&board, &fanout)
                .map_err(|e| anyhow::anyhow!("fanout plan: {e}"))?;
            write_new_atomic(path, plan.to_routing_file_json().as_bytes())?;
            report.fanout_plan_path = Some(path.clone());
        }
    }
    let planned = if separate.is_some() {
        work.clone()
    } else {
        RoutedCopper {
            tracks: fanout.tracks.iter().chain(&work.tracks).copied().collect(),
            vias: fanout.vias.iter().chain(&work.vias).copied().collect(),
        }
    };
    if planned.tracks.is_empty() && planned.vias.is_empty() {
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
    let plan =
        RoutePlan::from_routed(&board, &planned).map_err(|e| anyhow::anyhow!("plan: {e}"))?;
    let chunks = plan.chunks_by_net(ADD_ROUTING_MAX_ITEMS).len();
    write_new_atomic(&spec.output_plan, plan.to_routing_file_json().as_bytes())?;
    report
        .telemetry
        .add_stage_time(Stage::Plan, t.elapsed().as_micros() as u64);
    report.plan_items = Some(plan.item_count());
    report.plan_chunks = Some(chunks);
    report.plan_path = Some(spec.output_plan.clone());
    let detail = json!({ "items": plan.item_count(), "add_routing_calls": chunks,
                         "path": spec.output_plan, "fanout_plan": report.fanout_plan_path });
    record(&mut report, observer, "plan", "done", t, detail);
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
pub(crate) fn write_new_atomic(path: &Path, bytes: &[u8]) -> Result<()> {
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
    fn write_status(dir: &Path, status: &Mutex<JobStatus>, started: Instant) -> bool {
        let mut s = status.lock().unwrap_or_else(|e| e.into_inner());
        s.heartbeat_unix = now_unix();
        s.elapsed_seconds = started.elapsed().as_secs_f64();
        match serde_json::to_vec_pretty(&*s) {
            Ok(bytes) => match write_atomic(&dir.join(STATUS_FILE), &bytes) {
                Ok(()) => true,
                Err(e) => {
                    eprintln!("status not written: {e:#}");
                    false
                }
            },
            Err(_) => false,
        }
    }

    /// The last status must land: on Windows the rename fails while a
    /// reader holds `status.json` open without delete sharing, and a
    /// finished job left showing RUNNING would later read as RUNNER_LOST.
    fn write_final_status(dir: &Path, status: &Mutex<JobStatus>, started: Instant) {
        for _ in 0..50 {
            if Self::write_status(dir, status, started) {
                return;
            }
            std::thread::sleep(Duration::from_millis(100));
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
        resumed: resume.as_ref().is_some_and(|c| c.spec_hash == spec.hash()),
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
                s.resumed = report.resumed;
                s.state = match (report.stop_kind, &report.stopped) {
                    (Some(kind), _) => kind.job_state().to_string(),
                    (None, None) => "COMPLETED".to_string(),
                    // Not expected: every stop carries its kind.
                    (None, Some(_)) => "FAILED".to_string(),
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
    DirObserver::write_final_status(dir, &status, started);
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
            "optimize_reverted": r["optimize_reverted"], "cleared": r["cleared"],
            "plan_items": r["plan_items"], "plan_chunks": r["plan_chunks"],
            "stopped": r["stopped"], "fanout_plan": r["fanout_plan_path"],
            "net_classes": r["net_classes"], "allowances": r["allowances"],
            "excluded_by_policy": r["excluded_by_policy"], "reference": r["reference"],
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
    if let Some(f) = spec.fanout_plan.as_ref().filter(|f| f.exists()) {
        bail!(
            "{} already exists; choose a new fanout_plan (plans are never replaced)",
            f.display()
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
