//! Freerouting jobs that outlive the MCP session that started them.
//!
//! `route_specctra_dsn` runs Freerouting inside the server process, so a run
//! lasts only as long as the client's session: a client that restarts or
//! closes ends the server, and every process it started goes with it
//! (on Windows the client's job object kills the whole tree). A large board
//! needs hours, so a run started with `start_specctra_route_job` is owned by
//! a separate `konnect freerouting-runner` process instead, started outside
//! the client's job. The runner keeps its state in a job directory beside
//! the requested SES:
//!
//! - `spec.json`: what to route and how, written once by the starter;
//! - `status.json`: the runner's state and heartbeat, rewritten every poll;
//! - `checkpoint.ses`: the board routed so far, saved at an interval;
//! - `progress.jsonl`: one JSON line per poll, checkpoint and finish,
//!   appended, so the run's progress over time survives the status rewrites;
//! - `stop`: created by `stop_specctra_route_job`; the runner then ends the
//!   job and writes the routed-so-far board to the SES output;
//! - `freerouting.log`: every byte of the router's own stderr;
//! - `freerouting-data/`: the router's `--user_data_path`, private to the
//!   job so concurrent jobs share no settings or log files;
//! - `runner.log`: the runner's own stderr.

use crate::freerouting_mcp::{
    find_value, route_local_monitored, validate_inputs, JobMonitor, LaunchOptions, PollAction,
    PollReport, RouteSettings,
};
use anyhow::{bail, Context, Result};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

/// The hidden subcommand the starter runs.
pub const RUNNER_SUBCOMMAND: &str = "freerouting-runner";

const SPEC_FILE: &str = "spec.json";
const STATUS_FILE: &str = "status.json";
const PROGRESS_FILE: &str = "progress.jsonl";
const FREEROUTING_LOG: &str = "freerouting.log";
const FREEROUTING_DATA: &str = "freerouting-data";
const STOP_FILE: &str = "stop";
const CHECKPOINT_FILE: &str = "checkpoint.ses";
const CHECKPOINT_TEMPORARY: &str = "checkpoint.tmp.ses";
const RUNNER_LOG: &str = "runner.log";
const POLL_INTERVAL: Duration = Duration::from_secs(3);
/// A runner that has not written its heartbeat for this long is gone: it
/// polls every few seconds, and one Freerouting request times out in 30 s.
const STALE_HEARTBEAT_SECONDS: u64 = 120;
/// How long the starter waits for the runner to prove the job started.
const START_WAIT: Duration = Duration::from_secs(60);
const MIN_CHECKPOINT_SECONDS: u64 = 60;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub(crate) struct JobSpec {
    pub jar: PathBuf,
    pub dsn: PathBuf,
    pub ses_output: PathBuf,
    pub max_passes: Option<u32>,
    pub optimizer_enabled: Option<bool>,
    pub fanout_enabled: Option<bool>,
    pub job_timeout_seconds: Option<u64>,
    pub checkpoint_interval_seconds: u64,
    /// Freerouting's REST API port; 0 (the default) lets the system pick.
    #[serde(default)]
    pub api_port: u16,
    #[serde(default)]
    pub max_heap_mb: Option<u32>,
}

impl JobSpec {
    fn settings(&self) -> RouteSettings {
        // The runner's only limit is the job's own: a margin above it leaves
        // time to download the routed-so-far board.
        let overall = self
            .job_timeout_seconds
            .map_or(86_400, |seconds| (seconds + 600).min(86_400));
        let dir = job_dir_for(&self.ses_output);
        RouteSettings {
            max_passes: self.max_passes,
            optimizer_enabled: self.optimizer_enabled,
            fanout_enabled: self.fanout_enabled,
            job_timeout_seconds: self.job_timeout_seconds,
            poll_interval: POLL_INTERVAL,
            overall_timeout: Duration::from_secs(overall),
            launch: LaunchOptions {
                api_port: self.api_port,
                user_data_path: Some(dir.join(FREEROUTING_DATA)),
                max_heap_mb: self.max_heap_mb,
                stderr_log: Some(dir.join(FREEROUTING_LOG)),
            },
        }
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub(crate) struct Checkpoint {
    pub path: PathBuf,
    pub bytes: u64,
    pub written_unix: u64,
    pub elapsed_seconds: f64,
    pub output_nets: Option<u64>,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub(crate) struct JobStatus {
    /// STARTING, RUNNING, COMPLETED, STOPPED_AT_TIME_LIMIT,
    /// STOPPED_ON_REQUEST or FAILED.
    pub state: String,
    pub finished: bool,
    pub partial: bool,
    pub runner_pid: u32,
    pub started_unix: u64,
    pub heartbeat_unix: u64,
    pub elapsed_seconds: f64,
    pub poll_count: u32,
    pub freerouting_session_id: Option<String>,
    pub freerouting_job_id: Option<String>,
    pub freerouting_state: Option<String>,
    pub current_pass: Option<u64>,
    /// Freerouting's statistics of the output so far.
    pub output_nets: Option<u64>,
    pub output_traces: Option<u64>,
    pub output_vias: Option<u64>,
    pub last_status: Option<String>,
    pub checkpoint: Option<Checkpoint>,
    pub checkpoint_error: Option<String>,
    pub ses_output: PathBuf,
    pub ses_bytes: Option<u64>,
    pub diagnostics_path: Option<String>,
    #[serde(default)]
    pub settings_shape: Option<String>,
    pub error: Option<String>,
}

pub(crate) fn job_dir_for(ses_output: &Path) -> PathBuf {
    let mut name = ses_output
        .file_name()
        .map(|name| name.to_os_string())
        .unwrap_or_default();
    name.push(".konnect-job");
    ses_output.with_file_name(name)
}

fn now_unix() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |elapsed| elapsed.as_secs())
}

fn write_json_atomic(path: &Path, value: &impl Serialize) -> Result<()> {
    let temporary = path.with_extension("json.tmp");
    std::fs::write(&temporary, serde_json::to_vec_pretty(value)?)
        .with_context(|| format!("write {}", temporary.display()))?;
    std::fs::rename(&temporary, path).with_context(|| format!("replace {}", path.display()))
}

/// Append one JSON line to `progress.jsonl`. The file is not capped: at the
/// default poll interval a day-long job writes on the order of 10 MB.
fn append_progress(dir: &Path, line: &Value) -> Result<()> {
    use std::io::Write;
    let path = dir.join(PROGRESS_FILE);
    let mut file = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(&path)
        .with_context(|| format!("open {}", path.display()))?;
    let mut bytes = serde_json::to_vec(line)?;
    bytes.push(b'\n');
    file.write_all(&bytes)
        .with_context(|| format!("append to {}", path.display()))
}

fn read_status_file(dir: &Path) -> Result<JobStatus> {
    let path = dir.join(STATUS_FILE);
    // The runner replaces the file every poll; a read can meet the rename.
    let mut last_error = None;
    for _ in 0..5 {
        match std::fs::read_to_string(&path) {
            Ok(text) => {
                return serde_json::from_str(&text)
                    .with_context(|| format!("parse {}", path.display()))
            }
            Err(error) => last_error = Some(error),
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    Err(last_error.unwrap()).with_context(|| format!("read {}", path.display()))
}

fn is_session(path: &Path) -> Result<u64> {
    let text = std::fs::read_to_string(path).with_context(|| format!("read {}", path.display()))?;
    let tree = konnect_sexp::parse_sexp(&text).context("parse checkpoint SES")?;
    if tree.head() != Some("session") {
        bail!("checkpoint is not a Specctra SES session");
    }
    Ok(text.len() as u64)
}

fn count_at(value: Option<&Value>, pointer: &str) -> Option<u64> {
    value
        .and_then(|value| value.pointer(pointer))
        .and_then(Value::as_u64)
}

/// The runner's side of the job directory.
struct FileMonitor {
    dir: PathBuf,
    status: JobStatus,
    interval: Duration,
    last_checkpoint: Instant,
}

impl FileMonitor {
    fn write(&mut self) {
        self.status.heartbeat_unix = now_unix();
        if let Err(error) = write_json_atomic(&self.dir.join(STATUS_FILE), &self.status) {
            eprintln!("status not written: {error:#}");
        }
    }

    /// Append one progress line: the event, the time and the router's
    /// figures as the status holds them now, plus `extra`'s fields.
    fn record(&self, event: &str, extra: Value) {
        let status = &self.status;
        let mut line = json!({
            "event": event,
            "unix": now_unix(),
            "elapsed_seconds": status.elapsed_seconds,
            "poll": status.poll_count,
            "state": status.freerouting_state.as_deref().unwrap_or(&status.state),
            "pass": status.current_pass,
            "nets": status.output_nets,
            "traces": status.output_traces,
            "vias": status.output_vias,
        });
        if let (Some(line), Value::Object(extra)) = (line.as_object_mut(), extra) {
            line.extend(extra);
        }
        if let Err(error) = append_progress(&self.dir, &line) {
            eprintln!("progress not recorded: {error:#}");
        }
    }
}

impl JobMonitor for FileMonitor {
    fn polled(&mut self, report: &PollReport<'_>) -> PollAction {
        let status = &mut self.status;
        status.state = "RUNNING".to_string();
        status.poll_count = report.poll_count;
        status.elapsed_seconds = report.elapsed.as_secs_f64();
        status.freerouting_session_id = Some(report.session_id.to_string());
        status.freerouting_job_id = Some(report.job_id.to_string());
        status.freerouting_state = Some(report.state.to_string());
        status.last_status = Some(report.last_status.to_string());
        status.current_pass = find_value(report.details, "current_pass").and_then(Value::as_u64);
        let output = find_value(report.details, "output");
        status.output_nets = count_at(output, "/statistics/nets/total_count");
        status.output_traces = count_at(output, "/statistics/traces/total_count");
        status.output_vias = count_at(output, "/statistics/vias/total_count");
        self.write();
        self.record("poll", Value::Null);

        let stop = self.dir.join(STOP_FILE).exists();
        let checkpoint = (!stop && self.last_checkpoint.elapsed() >= self.interval).then(|| {
            let temporary = self.dir.join(CHECKPOINT_TEMPORARY);
            let _ = std::fs::remove_file(&temporary);
            temporary
        });
        PollAction { stop, checkpoint }
    }

    fn checkpoint_downloaded(&mut self, path: &Path, result: &Result<()>) {
        self.last_checkpoint = Instant::now();
        let saved = result
            .as_ref()
            .map_err(|error| anyhow::anyhow!("{error:#}"))
            .and_then(|_| is_session(path))
            .and_then(|bytes| {
                let target = self.dir.join(CHECKPOINT_FILE);
                std::fs::rename(path, &target)
                    .with_context(|| format!("replace {}", target.display()))?;
                Ok((target, bytes))
            });
        match saved {
            Ok((path, bytes)) => {
                self.status.checkpoint = Some(Checkpoint {
                    path,
                    bytes,
                    written_unix: now_unix(),
                    elapsed_seconds: self.status.elapsed_seconds,
                    output_nets: self.status.output_nets,
                });
                self.status.checkpoint_error = None;
                self.record("checkpoint", json!({ "checkpoint_bytes": bytes }));
            }
            Err(error) => {
                let _ = std::fs::remove_file(path);
                let error = format!("{error:#}");
                self.record("checkpoint_failed", json!({ "error": error }));
                self.status.checkpoint_error = Some(error);
            }
        }
        self.write();
    }
}

/// `konnect freerouting-runner <job-dir>`: route the job the directory
/// describes, keeping its status file current until the job ends.
pub async fn run_runner(args: &[String]) -> Result<()> {
    let dir = PathBuf::from(
        args.first()
            .context("usage: konnect freerouting-runner <job-directory>")?,
    );
    let spec: JobSpec = serde_json::from_str(
        &std::fs::read_to_string(dir.join(SPEC_FILE))
            .with_context(|| format!("read {}", dir.join(SPEC_FILE).display()))?,
    )
    .context("parse job spec")?;
    let mut monitor = FileMonitor {
        dir: dir.clone(),
        status: JobStatus {
            state: "STARTING".to_string(),
            runner_pid: std::process::id(),
            started_unix: now_unix(),
            ses_output: spec.ses_output.clone(),
            ..Default::default()
        },
        interval: Duration::from_secs(spec.checkpoint_interval_seconds),
        last_checkpoint: Instant::now(),
    };
    monitor.write();
    let result = route_local_monitored(
        &spec.jar,
        &spec.dsn,
        &spec.ses_output,
        &spec.settings(),
        &mut monitor,
    )
    .await;
    let status = &mut monitor.status;
    status.finished = true;
    match result {
        Ok(evidence) => {
            status.state = evidence.final_state;
            status.partial = evidence.partial;
            status.elapsed_seconds = evidence.elapsed_seconds;
            status.last_status = Some(evidence.last_status);
            status.ses_bytes = Some(evidence.ses_bytes);
            status.diagnostics_path = evidence.diagnostics_path;
            status.settings_shape = evidence.settings_shape.map(str::to_string);
        }
        Err(error) => {
            status.state = "FAILED".to_string();
            status.error = Some(format!("{error:#}"));
        }
    }
    monitor.write();
    let finished = json!({
        "final_state": monitor.status.state,
        "partial": monitor.status.partial,
        "ses_bytes": monitor.status.ses_bytes,
    });
    monitor.record("finished", finished);
    Ok(())
}

/// Start the runner outside the client's job, so it survives the session.
/// Returns the child's pid and whether it left the job.
fn spawn_runner(dir: &Path) -> Result<(u32, bool)> {
    let executable = std::env::current_exe().context("locate the konnect executable")?;
    let log = std::fs::File::create(dir.join(RUNNER_LOG)).context("create runner.log")?;
    let mut command = std::process::Command::new(&executable);
    command
        .arg(RUNNER_SUBCOMMAND)
        .arg(dir)
        .stdin(std::process::Stdio::null())
        .stdout(std::process::Stdio::null())
        .stderr(log);
    spawn_detached(&mut command)
        .with_context(|| format!("start {} {RUNNER_SUBCOMMAND}", executable.display()))
}

#[cfg(windows)]
fn spawn_detached(command: &mut std::process::Command) -> std::io::Result<(u32, bool)> {
    use std::os::windows::process::CommandExt;
    const CREATE_NEW_PROCESS_GROUP: u32 = 0x0000_0200;
    const CREATE_BREAKAWAY_FROM_JOB: u32 = 0x0100_0000;
    const CREATE_NO_WINDOW: u32 = 0x0800_0000;
    const ERROR_ACCESS_DENIED: i32 = 5;
    // A hidden console of its own, which Freerouting's java inherits.
    command.creation_flags(CREATE_NEW_PROCESS_GROUP | CREATE_NO_WINDOW | CREATE_BREAKAWAY_FROM_JOB);
    match command.spawn() {
        Ok(child) => Ok((child.id(), true)),
        // The client's job forbids breakaway: the runner still runs, but
        // ends with the client.
        Err(error) if error.raw_os_error() == Some(ERROR_ACCESS_DENIED) => {
            command.creation_flags(CREATE_NEW_PROCESS_GROUP | CREATE_NO_WINDOW);
            command.spawn().map(|child| (child.id(), false))
        }
        Err(error) => Err(error),
    }
}

#[cfg(unix)]
fn spawn_detached(command: &mut std::process::Command) -> std::io::Result<(u32, bool)> {
    use std::os::unix::process::CommandExt;
    // Its own process group, so the client's group signals do not reach it.
    command.process_group(0);
    command.spawn().map(|child| (child.id(), true))
}

/// What the tools report about a job: the status file plus what only the
/// reader can tell (heartbeat age, a lost runner, files on disk).
fn report(dir: &Path, status: &JobStatus) -> Value {
    let heartbeat_age = now_unix().saturating_sub(status.heartbeat_unix);
    let lost = !status.finished && heartbeat_age > STALE_HEARTBEAT_SECONDS;
    let mut body = serde_json::to_value(status).unwrap_or_else(|_| json!({}));
    if lost {
        body["state"] = json!("RUNNER_LOST");
        body["note"] = json!(format!(
            "The runner has not written its status for {heartbeat_age} s: it ended without \
             finishing. The last checkpoint, if any, holds the board routed so far."
        ));
    }
    body["job_dir"] = json!(dir);
    body["progress_log"] = json!(dir.join(PROGRESS_FILE));
    body["freerouting_log"] = json!(dir.join(FREEROUTING_LOG));
    body["heartbeat_age_seconds"] = json!(heartbeat_age);
    body["runner_alive"] = json!(!status.finished && !lost);
    body["stop_requested"] = json!(dir.join(STOP_FILE).exists());
    body["ses_output_exists"] = json!(status.ses_output.is_file());
    body
}

pub(crate) async fn start(spec: JobSpec) -> Result<Value> {
    if spec.checkpoint_interval_seconds < MIN_CHECKPOINT_SECONDS
        || spec.checkpoint_interval_seconds > 86_400
    {
        bail!("checkpoint_interval_seconds must be between {MIN_CHECKPOINT_SECONDS} and 86400");
    }
    validate_inputs(&spec.jar, &spec.dsn, &spec.ses_output, &spec.settings())?;
    let dir = job_dir_for(&spec.ses_output);
    if dir.exists() {
        let state = read_status_file(&dir)
            .map(|status| report(&dir, &status)["state"].clone())
            .unwrap_or_else(|_| json!("unknown"));
        bail!(
            "a job directory already exists for this output ({}, state {state}); \
             choose a new ses_output_path, or remove the directory once its \
             checkpoint is no longer needed",
            dir.display()
        );
    }
    std::fs::create_dir_all(&dir).with_context(|| format!("create {}", dir.display()))?;
    write_json_atomic(&dir.join(SPEC_FILE), &spec)?;
    let (pid, left_client_job) = spawn_runner(&dir)?;

    // Wait until Freerouting has the job running (its settings proven) or
    // the runner has failed, so a bad start is reported here.
    let waited = Instant::now();
    let status = loop {
        tokio::time::sleep(Duration::from_millis(500)).await;
        let status = read_status_file(&dir).ok();
        let started = status
            .as_ref()
            .is_some_and(|status| status.finished || status.state != "STARTING");
        if started || waited.elapsed() >= START_WAIT {
            break status;
        }
    };
    let mut body = match &status {
        Some(status) => report(&dir, status),
        None => json!({ "state": "STARTING", "job_dir": dir }),
    };
    body["runner_pid"] = json!(pid);
    body["left_client_job"] = json!(left_client_job);
    if !left_client_job {
        body["warning"] = json!(
            "The client's job object forbids breakaway: this run ends if the client's \
             session ends. Checkpoints are still saved."
        );
    }
    let failed = status
        .as_ref()
        .is_some_and(|status| status.state == "FAILED");
    body["success"] = json!(!failed);
    Ok(body)
}

pub(crate) fn status(ses_output: &Path) -> Result<Value> {
    let dir = job_dir_for(ses_output);
    if !dir.is_dir() {
        bail!(
            "no route job for {} (no {})",
            ses_output.display(),
            dir.display()
        );
    }
    let status = read_status_file(&dir)?;
    Ok(report(&dir, &status))
}

pub(crate) async fn stop(ses_output: &Path, wait: Duration) -> Result<Value> {
    let dir = job_dir_for(ses_output);
    if !dir.is_dir() {
        bail!(
            "no route job for {} (no {})",
            ses_output.display(),
            dir.display()
        );
    }
    let status = read_status_file(&dir)?;
    if status.finished {
        let mut body = report(&dir, &status);
        body["note"] = json!("The job had already finished; nothing to stop.");
        return Ok(body);
    }
    std::fs::write(dir.join(STOP_FILE), b"")
        .with_context(|| format!("create {}", dir.join(STOP_FILE).display()))?;
    let waited = Instant::now();
    loop {
        let status = read_status_file(&dir)?;
        let body = report(&dir, &status);
        if status.finished || waited.elapsed() >= wait || body["state"] == "RUNNER_LOST" {
            return Ok(body);
        }
        tokio::time::sleep(Duration::from_secs(1)).await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn details(pass: u64, nets: u64) -> Value {
        json!({ "parsedTextContent": [{ "status": 200, "body": {
            "state": "RUNNING", "current_pass": pass,
            "input": { "statistics": { "nets": { "total_count": 2646 } } },
            "output": { "statistics": {
                "nets": { "total_count": nets },
                "traces": { "total_count": 310 },
                "vias": { "total_count": 132 }
            } }
        } }] })
    }

    fn monitor(dir: &Path, interval: Duration) -> FileMonitor {
        FileMonitor {
            dir: dir.to_path_buf(),
            status: JobStatus {
                state: "STARTING".to_string(),
                ses_output: dir.join("out.ses"),
                ..Default::default()
            },
            interval,
            last_checkpoint: Instant::now(),
        }
    }

    fn poll(monitor: &mut FileMonitor, details: &Value) -> PollAction {
        monitor.polled(&PollReport {
            session_id: "s",
            job_id: "j",
            state: "RUNNING",
            last_status: "RUNNING (current_pass=1)",
            details,
            poll_count: 1,
            elapsed: Duration::from_secs(5),
        })
    }

    #[test]
    fn the_job_directory_sits_beside_the_output() {
        assert_eq!(
            job_dir_for(Path::new("fr/db1-route-03.ses")),
            Path::new("fr/db1-route-03.ses.konnect-job")
        );
    }

    #[test]
    fn a_poll_records_the_router_figures_and_a_heartbeat() {
        let temp = tempfile::tempdir().unwrap();
        let mut monitor = monitor(temp.path(), Duration::from_secs(600));
        let action = poll(&mut monitor, &details(1, 148));
        assert!(!action.stop);
        assert!(
            action.checkpoint.is_none(),
            "no checkpoint before the interval"
        );
        let status = read_status_file(temp.path()).unwrap();
        assert_eq!(status.state, "RUNNING");
        assert_eq!(status.current_pass, Some(1));
        assert_eq!(
            status.output_nets,
            Some(148),
            "output, not input, statistics"
        );
        assert_eq!(status.output_vias, Some(132));
        assert_eq!(status.freerouting_job_id.as_deref(), Some("j"));
        assert!(now_unix() - status.heartbeat_unix < 5);
    }

    #[test]
    fn checkpoints_come_at_the_interval_and_keep_only_valid_sessions() {
        let temp = tempfile::tempdir().unwrap();
        let mut monitor = monitor(temp.path(), Duration::ZERO);
        let path = poll(&mut monitor, &details(1, 10)).checkpoint.unwrap();

        std::fs::write(&path, "(session board (routes))").unwrap();
        monitor.checkpoint_downloaded(&path, &Ok(()));
        let saved = read_status_file(temp.path()).unwrap().checkpoint.unwrap();
        assert_eq!(saved.path, temp.path().join(CHECKPOINT_FILE));
        assert_eq!(saved.output_nets, Some(10));
        assert!(saved.path.is_file() && !path.exists());

        // A failed or malformed download leaves the previous checkpoint.
        let path = poll(&mut monitor, &details(1, 20)).checkpoint.unwrap();
        std::fs::write(&path, "(pcb board)").unwrap();
        monitor.checkpoint_downloaded(&path, &Ok(()));
        let status = read_status_file(temp.path()).unwrap();
        assert!(status
            .checkpoint_error
            .unwrap()
            .contains("not a Specctra SES"));
        assert_eq!(status.checkpoint.unwrap().output_nets, Some(10));
        assert!(!path.exists());
    }

    #[test]
    fn a_stop_file_asks_the_loop_to_stop_without_a_checkpoint() {
        let temp = tempfile::tempdir().unwrap();
        let mut monitor = monitor(temp.path(), Duration::ZERO);
        std::fs::write(temp.path().join(STOP_FILE), b"").unwrap();
        let action = poll(&mut monitor, &details(2, 30));
        assert!(action.stop);
        assert!(action.checkpoint.is_none());
    }

    #[test]
    fn a_silent_unfinished_runner_is_reported_lost() {
        let temp = tempfile::tempdir().unwrap();
        let mut status = JobStatus {
            state: "RUNNING".to_string(),
            heartbeat_unix: now_unix() - STALE_HEARTBEAT_SECONDS - 5,
            ..Default::default()
        };
        let body = report(temp.path(), &status);
        assert_eq!(body["state"], "RUNNER_LOST");
        assert_eq!(body["runner_alive"], false);

        status.heartbeat_unix = now_unix();
        assert_eq!(report(temp.path(), &status)["state"], "RUNNING");
        status.finished = true;
        status.heartbeat_unix = 0;
        status.state = "COMPLETED".to_string();
        assert_eq!(
            report(temp.path(), &status)["state"],
            "COMPLETED",
            "a finished job is never lost"
        );
    }

    #[test]
    fn the_runner_limit_is_the_job_limit() {
        let spec = JobSpec {
            jar: PathBuf::new(),
            dsn: PathBuf::new(),
            ses_output: PathBuf::new(),
            max_passes: Some(4),
            optimizer_enabled: Some(false),
            fanout_enabled: Some(false),
            job_timeout_seconds: Some(36_000),
            checkpoint_interval_seconds: 600,
            api_port: 0,
            max_heap_mb: None,
        };
        let settings = spec.settings();
        assert_eq!(settings.overall_timeout, Duration::from_secs(36_600));
        assert_eq!(settings.job_timeout_seconds, Some(36_000));
    }

    #[test]
    fn each_job_keeps_its_router_log_and_data_in_its_own_directory() {
        let spec = |output: &str, api_port: u16| JobSpec {
            jar: PathBuf::new(),
            dsn: PathBuf::new(),
            ses_output: PathBuf::from(output),
            max_passes: None,
            optimizer_enabled: None,
            fanout_enabled: None,
            job_timeout_seconds: None,
            checkpoint_interval_seconds: 600,
            api_port,
            max_heap_mb: Some(4096),
        };
        let first = spec("fr/region-1.ses", 0).settings().launch;
        let second = spec("fr/region-2.ses", 41_002).settings().launch;
        let dir = Path::new("fr/region-1.ses.konnect-job");
        assert_eq!(
            first.stderr_log.as_deref(),
            Some(dir.join(FREEROUTING_LOG).as_path())
        );
        assert_eq!(
            first.user_data_path.as_deref(),
            Some(dir.join(FREEROUTING_DATA).as_path())
        );
        assert_ne!(first.user_data_path, second.user_data_path);
        assert_eq!((first.api_port, second.api_port), (0, 41_002));
        assert_eq!(first.max_heap_mb, Some(4096));

        // A spec written before ports and heap existed still reads.
        let old: JobSpec = serde_json::from_value(json!({
            "jar": "fr.jar", "dsn": "b.dsn", "ses_output": "b.ses", "max_passes": 4,
            "optimizer_enabled": false, "fanout_enabled": false,
            "job_timeout_seconds": 36000, "checkpoint_interval_seconds": 600
        }))
        .unwrap();
        assert_eq!((old.api_port, old.max_heap_mb), (0, None));
    }

    #[test]
    fn every_poll_and_checkpoint_appends_a_progress_line() {
        let temp = tempfile::tempdir().unwrap();
        let mut monitor = monitor(temp.path(), Duration::ZERO);
        let path = poll(&mut monitor, &details(1, 148)).checkpoint.unwrap();
        std::fs::write(&path, "(session board (routes))").unwrap();
        monitor.checkpoint_downloaded(&path, &Ok(()));
        poll(&mut monitor, &details(2, 300));

        let lines = std::fs::read_to_string(temp.path().join(PROGRESS_FILE))
            .unwrap()
            .lines()
            .map(|line| serde_json::from_str::<Value>(line).unwrap())
            .collect::<Vec<_>>();
        assert_eq!(lines.len(), 3, "{lines:?}");
        assert_eq!(lines[0]["event"], "poll");
        assert_eq!(lines[0]["pass"], 1);
        assert_eq!(lines[0]["nets"], 148);
        assert_eq!(lines[0]["traces"], 310);
        assert_eq!(lines[0]["vias"], 132);
        assert_eq!(lines[0]["state"], "RUNNING");
        assert!(lines[0]["unix"].as_u64().unwrap() > 0);
        assert_eq!(lines[1]["event"], "checkpoint");
        assert_eq!(lines[1]["checkpoint_bytes"], 24);
        assert_eq!(lines[2]["pass"], 2);
        assert_eq!(lines[2]["nets"], 300);
        // The status file still holds only the latest poll.
        assert_eq!(
            read_status_file(temp.path()).unwrap().output_nets,
            Some(300)
        );
    }
}
