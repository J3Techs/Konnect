//! Local Freerouting MCP orchestration.
//!
//! Konnect deliberately does not reimplement the router. It starts the local
//! Freerouting JAR in its documented headless stdio MCP mode, follows the
//! server's routing state machine, and returns only compact job evidence.

use anyhow::{bail, Context, Result};
use serde::Serialize;
use serde_json::{json, Map, Value};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::Stdio;
use std::time::Duration;
use tokio::io::{AsyncBufReadExt, AsyncReadExt, AsyncWriteExt, BufReader, Lines};
use tokio::process::{Child, ChildStdin, ChildStdout, Command};
use tokio::task::JoinHandle;
use tokio::time::{timeout, Instant};

#[cfg(test)]
use std::sync::atomic::{AtomicU32, Ordering};

#[cfg(test)]
static LAST_CHILD_PID: AtomicU32 = AtomicU32::new(0);

const MCP_PROTOCOL_VERSION: &str = "2024-11-05";
const STARTUP_TIMEOUT: Duration = Duration::from_secs(30);
const REQUEST_TIMEOUT: Duration = Duration::from_secs(30);
const SHUTDOWN_TIMEOUT: Duration = Duration::from_secs(3);
/// How much of the router's stderr is kept in memory for error messages;
/// the whole stream goes to the stderr log.
const MAX_STDERR_BYTES: usize = 64 * 1024;
const MAX_SES_BYTES: u64 = 512 * 1024 * 1024;
/// Bounds for [`LaunchOptions::max_heap_mb`].
const MIN_HEAP_MB: u32 = 256;
const MAX_HEAP_MB: u32 = 256 * 1024;

/// How one Freerouting process is started. Every process gets its own
/// ports and data directory, so several jobs (and `check_freerouting`) can
/// run at once.
#[derive(Debug, Clone, Default)]
pub(crate) struct LaunchOptions {
    /// Port of Freerouting's REST API on 127.0.0.1. 0 lets the system pick
    /// a free one; Freerouting then points its MCP tools at the port it
    /// bound (2.3 and 2.5 both read it back with `getLocalPort`).
    pub api_port: u16,
    /// Freerouting's `--user_data_path`: its settings file and its own log.
    /// None uses a temporary directory removed when the process ends.
    pub user_data_path: Option<PathBuf>,
    /// Java heap limit (`-Xmx`, MiB). The only JVM option Konnect passes:
    /// there is no free-text JVM argument.
    pub max_heap_mb: Option<u32>,
    /// Every byte of the router's stderr is written here. In stdio mode
    /// Freerouting sends its console output to stderr, so this is the whole
    /// log; stdout carries only the MCP protocol.
    pub stderr_log: Option<PathBuf>,
}

/// The server's MCP endpoint always takes a system-chosen port: the stdio
/// bridge inside Freerouting reads back the port it bound.
fn server_arguments(launch: &LaunchOptions, user_data_path: &Path) -> Vec<String> {
    let api_port = launch.api_port;
    let mut arguments = vec![
        "--api_server.enabled=true".to_string(),
        "--api_server.authentication.enabled=false".to_string(),
        format!("--api_server-endpoints=http://127.0.0.1:{api_port}"),
        "--mcp_server.enabled=true".to_string(),
        "--mcp_server.authentication.enabled=false".to_string(),
        "--mcp_server-endpoints=http://127.0.0.1:0".to_string(),
        "--mcp_server.stdio=true".to_string(),
        "--gui.enabled=false".to_string(),
        format!("--user_data_path={}", user_data_path.display()),
    ];
    if api_port != 0 {
        arguments.push(format!(
            "--mcp_server.target_api_base_url=http://127.0.0.1:{api_port}"
        ));
    }
    arguments
}

/// The JVM options before `-jar`: only the allowlisted heap limit.
fn jvm_arguments(launch: &LaunchOptions) -> Vec<String> {
    launch
        .max_heap_mb
        .map(|megabytes| format!("-Xmx{megabytes}m"))
        .into_iter()
        .collect()
}

const REQUIRED_TOOLS: &[&str] = &[
    "create_session",
    "enqueue_job",
    "upload_job_input_from_local_file",
    "start_job",
    "get_job_details",
    "download_job_output_to_local_file",
];

#[derive(Debug, Clone)]
pub(crate) struct RouteSettings {
    pub max_passes: Option<u32>,
    pub optimizer_enabled: Option<bool>,
    pub fanout_enabled: Option<bool>,
    pub job_timeout_seconds: Option<u64>,
    pub poll_interval: Duration,
    pub overall_timeout: Duration,
    pub launch: LaunchOptions,
}

#[derive(Debug, Clone, Serialize)]
pub(crate) struct RouteEvidence {
    pub session_id: String,
    pub job_id: String,
    pub final_state: String,
    /// Konnect stopped the job at its time limit or on request: the SES holds
    /// the routing done so far, not a finished run.
    pub partial: bool,
    /// The last job state and figures the router reported.
    pub last_status: String,
    pub poll_count: u32,
    pub elapsed_seconds: f64,
    pub ses_bytes: u64,
    pub server_protocol_version: String,
    /// The router's complete stderr, when it wrote any.
    pub diagnostics_path: Option<String>,
    /// Which settings shape Freerouting echoed back, when the shapes
    /// differ (see [`SettingsShape`]).
    pub settings_shape: Option<&'static str>,
}

#[derive(Debug, Clone, Serialize)]
pub(crate) struct BridgeProbe {
    pub native_mcp_available: bool,
    pub bridge_available: bool,
    pub server_protocol_version: Option<String>,
    pub tool_count: usize,
    pub diagnostics: Option<String>,
    pub error: Option<String>,
}

/// Probe with a system-chosen port and a temporary data directory, so a
/// probe never meets a running job's ports or files.
pub(crate) async fn probe_local(jar: &Path) -> BridgeProbe {
    let mut client = match LocalMcpClient::start(jar, &LaunchOptions::default()).await {
        Ok(client) => client,
        Err(error) => {
            return BridgeProbe {
                native_mcp_available: false,
                bridge_available: false,
                server_protocol_version: None,
                tool_count: 0,
                diagnostics: None,
                error: Some(format!("{error:#}")),
            };
        }
    };
    let server_protocol_version = Some(client.server_protocol_version.clone());
    let contract = async {
        let tools = client.list_tools().await?;
        validate_tool_contracts(&tools)?;
        let missing = REQUIRED_TOOLS
            .iter()
            .filter(|name| !tools.contains_key(**name))
            .copied()
            .collect::<Vec<_>>();
        if !missing.is_empty() {
            bail!(
                "Freerouting MCP is missing required tool(s): {}",
                missing.join(", ")
            );
        }
        Ok::<usize, anyhow::Error>(tools.len())
    }
    .await;
    let diagnostics = client.close().await;
    match contract {
        Ok(tool_count) => BridgeProbe {
            native_mcp_available: true,
            bridge_available: true,
            server_protocol_version,
            tool_count,
            diagnostics: (!diagnostics.is_empty()).then_some(diagnostics),
            error: None,
        },
        Err(error) => BridgeProbe {
            native_mcp_available: true,
            bridge_available: false,
            server_protocol_version,
            tool_count: 0,
            diagnostics: (!diagnostics.is_empty()).then_some(diagnostics),
            error: Some(format!("{error:#}")),
        },
    }
}

/// One poll of a running job, as a [`JobMonitor`] sees it.
pub(crate) struct PollReport<'a> {
    pub session_id: &'a str,
    pub job_id: &'a str,
    pub state: &'a str,
    pub last_status: &'a str,
    pub details: &'a Value,
    pub poll_count: u32,
    pub elapsed: Duration,
}

/// What a [`JobMonitor`] asks of the poll loop after a poll.
#[derive(Debug, Default)]
pub(crate) struct PollAction {
    /// End the job now and keep the board routed so far (partial).
    pub stop: bool,
    /// Download the board routed so far to this path; the job keeps running.
    pub checkpoint: Option<PathBuf>,
}

/// Watches a job from inside the poll loop: the detached runner writes its
/// status file, saves checkpoints and takes stop requests through this.
pub(crate) trait JobMonitor: Send {
    fn polled(&mut self, report: &PollReport<'_>) -> PollAction;
    /// A checkpoint download asked for by [`PollAction::checkpoint`] finished.
    fn checkpoint_downloaded(&mut self, _path: &Path, _result: &Result<()>) {}
}

struct NoMonitor;

impl JobMonitor for NoMonitor {
    fn polled(&mut self, _report: &PollReport<'_>) -> PollAction {
        PollAction::default()
    }
}

pub(crate) async fn route_local(
    jar: &Path,
    dsn: &Path,
    ses_output: &Path,
    settings: &RouteSettings,
) -> Result<RouteEvidence> {
    route_local_monitored(jar, dsn, ses_output, settings, &mut NoMonitor).await
}

pub(crate) async fn route_local_monitored(
    jar: &Path,
    dsn: &Path,
    ses_output: &Path,
    settings: &RouteSettings,
    monitor: &mut dyn JobMonitor,
) -> Result<RouteEvidence> {
    validate_inputs(jar, dsn, ses_output, settings)?;
    if let Some(parent) = ses_output
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        tokio::fs::create_dir_all(parent)
            .await
            .with_context(|| format!("create SES output directory {}", parent.display()))?;
    }
    let temporary = temporary_output_path(ses_output)?;
    if temporary.exists() {
        bail!(
            "temporary SES output already exists: {}",
            temporary.display()
        );
    }
    let mut launch = settings.launch.clone();
    let stderr_log = launch
        .stderr_log
        .get_or_insert_with(|| default_stderr_log(ses_output))
        .clone();
    if stderr_log.exists() {
        bail!(
            "Freerouting diagnostics output already exists: {}",
            stderr_log.display()
        );
    }

    let mut client = match LocalMcpClient::start(jar, &launch).await {
        Ok(client) => client,
        Err(error) => {
            finish_stderr_log(&stderr_log).await;
            return Err(error);
        }
    };
    let started = Instant::now();
    let route_result = match timeout(
        settings.overall_timeout,
        run_state_machine(&mut client, dsn, &temporary, settings, started, monitor),
    )
    .await
    {
        Ok(result) => result,
        Err(_) => Err(anyhow::anyhow!(
            "Freerouting MCP job exceeded the overall timeout of {} seconds",
            settings.overall_timeout.as_secs()
        )),
    };
    let diagnostics = client.close().await;
    let diagnostics_path = finish_stderr_log(&stderr_log).await;

    let result = match route_result {
        Ok(evidence) => Ok(evidence),
        Err(error) => {
            if diagnostics.is_empty() {
                Err(error)
            } else {
                Err(error).context(format!("Freerouting stderr: {diagnostics}"))
            }
        }
    };
    let evidence = match result {
        Ok(evidence) => evidence,
        Err(error) => {
            let _ = tokio::fs::remove_file(&temporary).await;
            return Err(error);
        }
    };

    let metadata = tokio::fs::metadata(&temporary)
        .await
        .with_context(|| format!("Freerouting did not create {}", temporary.display()))?;
    if metadata.len() == 0 || metadata.len() > MAX_SES_BYTES {
        let _ = tokio::fs::remove_file(&temporary).await;
        bail!(
            "Freerouting SES size {} is outside the supported range 1..={MAX_SES_BYTES}",
            metadata.len()
        );
    }
    let ses_source = tokio::fs::read_to_string(&temporary)
        .await
        .with_context(|| format!("read Freerouting SES {}", temporary.display()))?;
    let tree = konnect_sexp::parse_sexp(&ses_source).context("parse Freerouting SES output")?;
    if tree.head() != Some("session") {
        let _ = tokio::fs::remove_file(&temporary).await;
        bail!("Freerouting output is not a Specctra SES session");
    }
    konnect_sexp::write_new_atomic(ses_output, &ses_source)
        .with_context(|| format!("create SES output {}", ses_output.display()))?;
    tokio::fs::remove_file(&temporary)
        .await
        .with_context(|| format!("remove temporary SES {}", temporary.display()))?;
    Ok(RouteEvidence {
        ses_bytes: metadata.len(),
        diagnostics_path,
        ..evidence
    })
}

/// The stderr log beside an SES output when the caller names none.
fn default_stderr_log(output: &Path) -> PathBuf {
    output.with_extension("freerouting.log")
}

/// The stderr log's path once the process has ended, or None (and no file)
/// when the router wrote nothing.
async fn finish_stderr_log(path: &Path) -> Option<String> {
    match tokio::fs::metadata(path).await {
        Ok(metadata) if metadata.len() > 0 => Some(path.display().to_string()),
        Ok(_) => {
            let _ = tokio::fs::remove_file(path).await;
            None
        }
        Err(_) => None,
    }
}

pub(crate) fn validate_inputs(
    jar: &Path,
    dsn: &Path,
    ses_output: &Path,
    settings: &RouteSettings,
) -> Result<()> {
    if !jar.is_file() {
        bail!("Freerouting JAR does not exist: {}", jar.display());
    }
    if !dsn.is_file() || !extension_is(dsn, "dsn") {
        bail!("DSN input must be an existing .dsn file: {}", dsn.display());
    }
    if !extension_is(ses_output, "ses") {
        bail!("SES output must have the .ses extension");
    }
    if ses_output.exists() {
        bail!("SES output already exists: {}", ses_output.display());
    }
    if let Some(max_passes) = settings.max_passes {
        if !(1..=100).contains(&max_passes) {
            bail!("max_passes must be between 1 and 100");
        }
    }
    if settings.poll_interval < Duration::from_secs(2)
        || settings.poll_interval > Duration::from_secs(5)
    {
        bail!("poll interval must be between 2 and 5 seconds");
    }
    let minimum_overall_timeout = if cfg!(test) {
        Duration::from_millis(1)
    } else {
        Duration::from_secs(10)
    };
    if settings.overall_timeout < minimum_overall_timeout
        || settings.overall_timeout > Duration::from_secs(86_400)
    {
        bail!("overall timeout must be between 10 and 86400 seconds");
    }
    if settings
        .job_timeout_seconds
        .is_some_and(|seconds| seconds == 0 || seconds > 86_400)
    {
        bail!("job_timeout_seconds must be between 1 and 86400");
    }
    if settings
        .launch
        .max_heap_mb
        .is_some_and(|megabytes| !(MIN_HEAP_MB..=MAX_HEAP_MB).contains(&megabytes))
    {
        bail!("max_heap_mb must be between {MIN_HEAP_MB} and {MAX_HEAP_MB}");
    }
    Ok(())
}

async fn run_state_machine(
    client: &mut LocalMcpClient,
    dsn: &Path,
    temporary: &Path,
    settings: &RouteSettings,
    started: Instant,
    monitor: &mut dyn JobMonitor,
) -> Result<RouteEvidence> {
    let tools = client.list_tools().await?;
    validate_tool_contracts(&tools)?;
    let missing = REQUIRED_TOOLS
        .iter()
        .filter(|name| !tools.contains_key(**name))
        .copied()
        .collect::<Vec<_>>();
    if !missing.is_empty() {
        bail!(
            "Freerouting MCP is missing required tool(s): {}",
            missing.join(", ")
        );
    }

    let session = client.call_tool("create_session", json!({})).await?;
    let session_id = required_string(&session, &["sessionId", "session_id", "id"], "session id")?;
    let job = client
        .call_tool(
            "enqueue_job",
            json!({ "body": { "session_id": session_id } }),
        )
        .await?;
    let job_id = required_string(&job, &["jobId", "job_id", "id"], "job id")?;
    client
        .call_tool(
            "upload_job_input_from_local_file",
            json!({ "jobId": job_id, "filePath": absolute_utf8(dsn)? }),
        )
        .await?;

    let body = job_settings_body(settings);
    let mut settings_shape = None;
    if !body.is_empty() {
        if !tools.contains_key("update_job_settings") {
            bail!("Freerouting MCP does not expose update_job_settings");
        }
        let updated = client
            .call_tool(
                "update_job_settings",
                json!({ "path": { "jobId": job_id }, "body": body }),
            )
            .await?;
        // Freerouting ignores a key it does not know without a word, so
        // prove from its answer that every setting took, in the shape of
        // one release.
        let applied = find_value(&updated, "router_settings");
        settings_shape = applied_settings_shape(settings, applied)
            .map_err(|unapplied| {
                anyhow::anyhow!(
                    "Freerouting did not apply job setting(s) {} in the shape of any supported \
                     release (2.3 or 2.5); its router_settings: {}",
                    unapplied.join(", "),
                    applied.map(compact).unwrap_or_else(|| "absent".to_string())
                )
            })?
            .map(SettingsShape::label);
    }
    client
        .call_tool("start_job", json!({ "path": { "jobId": job_id } }))
        .await?;

    // Konnect keeps the time limit itself: Freerouting answers a job that
    // reached its own timeout with "no valid output", while a running job's
    // download is the board routed so far. Stop early enough to fetch it
    // before the overall timeout abandons the job.
    let margin = (settings.overall_timeout / 4).min(Duration::from_secs(60));
    let mut limit = settings.overall_timeout.saturating_sub(margin);
    if let Some(seconds) = settings.job_timeout_seconds {
        limit = limit.min(Duration::from_secs(seconds));
    }
    let deadline = started + limit;
    let mut stopped_early = false;
    let mut poll_count = 0u32;
    let mut last_status: String;
    let final_state = loop {
        let details = client
            .call_tool("get_job_details", json!({ "path": { "jobId": job_id } }))
            .await?;
        poll_count = poll_count.saturating_add(1);
        let state = required_string(&details, &["state", "status"], "job state")?;
        last_status = format!("{state}{}", job_progress_summary(&details));
        // Keep the client's idle timer from abandoning a long job, and say
        // what the router reports about it.
        crate::mcp::progress::report(
            f64::from(poll_count),
            None,
            &format!(
                "Freerouting job {job_id}: {last_status} after {:.0} s",
                started.elapsed().as_secs_f64(),
            ),
        )
        .await;
        let action = monitor.polled(&PollReport {
            session_id: &session_id,
            job_id: &job_id,
            state: &state,
            last_status: &last_status,
            details: &details,
            poll_count,
            elapsed: started.elapsed(),
        });
        let outcome = job_outcome(&state);
        if outcome == JobOutcome::Running && !action.stop {
            if let Some(checkpoint) = action.checkpoint {
                // A running job's download is the board routed so far.
                let result = match absolute_utf8(&checkpoint) {
                    Ok(path) => client
                        .call_tool(
                            "download_job_output_to_local_file",
                            json!({ "jobId": job_id, "filePath": path }),
                        )
                        .await
                        .map(|_| ()),
                    Err(error) => Err(error),
                };
                monitor.checkpoint_downloaded(&checkpoint, &result);
            }
        }
        match outcome {
            JobOutcome::Running if action.stop => {
                stopped_early = true;
                break "STOPPED_ON_REQUEST".to_string();
            }
            JobOutcome::Running if Instant::now() + settings.poll_interval >= deadline => {
                stopped_early = true;
                break "STOPPED_AT_TIME_LIMIT".to_string();
            }
            JobOutcome::Running => tokio::time::sleep(settings.poll_interval).await,
            JobOutcome::Finished => break state.to_ascii_uppercase(),
            JobOutcome::Failed => {
                bail!("Freerouting job {job_id} ended in state {state} ({last_status})")
            }
        }
    };

    client
        .call_tool(
            "download_job_output_to_local_file",
            json!({ "jobId": job_id, "filePath": absolute_utf8(temporary)? }),
        )
        .await?;
    if stopped_early && tools.contains_key("cancel_job") {
        // The child is closed next in any case; this only ends the job cleanly.
        let _ = client
            .call_tool("cancel_job", json!({ "path": { "jobId": job_id } }))
            .await;
    }
    Ok(RouteEvidence {
        session_id,
        job_id,
        partial: stopped_early,
        final_state,
        last_status,
        poll_count,
        elapsed_seconds: started.elapsed().as_secs_f64(),
        ses_bytes: 0,
        server_protocol_version: client.server_protocol_version.clone(),
        diagnostics_path: None,
        settings_shape,
    })
}

/// Where a Freerouting release reads its router settings. Only the pass
/// limit moved: 2.3 reads `max_passes` at the top of `router_settings`,
/// 2.5 reads it as `autorouter.max_passes` (its `AutorouterSettings`).
/// `optimizer.enabled` and `fanout.enabled` are the same in both.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SettingsShape {
    Flat,
    Nested,
}

impl SettingsShape {
    /// Newest first, so a tie reports the current release.
    const ALL: [Self; 2] = [Self::Nested, Self::Flat];

    fn label(self) -> &'static str {
        match self {
            Self::Flat => "flat (Freerouting 2.3)",
            Self::Nested => "nested (Freerouting 2.5)",
        }
    }
}

/// The settings in one release's Gson names. The time limit is not sent:
/// Konnect enforces it, so the partial result survives.
fn job_settings(settings: &RouteSettings, shape: SettingsShape) -> Map<String, Value> {
    let mut body = Map::new();
    if let Some(max_passes) = settings.max_passes {
        match shape {
            SettingsShape::Flat => {
                body.insert("max_passes".to_string(), json!(max_passes));
            }
            SettingsShape::Nested => {
                body.insert(
                    "autorouter".to_string(),
                    json!({ "max_passes": max_passes }),
                );
            }
        }
    }
    if let Some(enabled) = settings.optimizer_enabled {
        body.insert("optimizer".to_string(), json!({ "enabled": enabled }));
    }
    if let Some(enabled) = settings.fanout_enabled {
        body.insert("fanout".to_string(), json!({ "enabled": enabled }));
    }
    body
}

/// The body sent: both shapes at once. Each release's Gson model skips the
/// other's key (2.3 has no `autorouter`; 2.5 has no top-level `max_passes`
/// and, if it maps the flat name, lets the nested value win), so one
/// request serves either release and the answer shows which one ran.
fn job_settings_body(settings: &RouteSettings) -> Map<String, Value> {
    let mut body = Map::new();
    for shape in SettingsShape::ALL {
        body.extend(job_settings(settings, shape));
    }
    body
}

/// The shape in which `applied` holds every setting: Some when the shapes
/// differ, None when they are the same. Otherwise the settings missing
/// from the closest shape.
fn applied_settings_shape(
    settings: &RouteSettings,
    applied: Option<&Value>,
) -> std::result::Result<Option<SettingsShape>, Vec<String>> {
    let shapes_differ = job_settings(settings, SettingsShape::Flat)
        != job_settings(settings, SettingsShape::Nested);
    let mut closest: Option<Vec<String>> = None;
    for shape in SettingsShape::ALL {
        let missing = unapplied_settings(&job_settings(settings, shape), applied, "");
        if missing.is_empty() {
            return Ok(shapes_differ.then_some(shape));
        }
        if closest
            .as_ref()
            .is_none_or(|closest| missing.len() < closest.len())
        {
            closest = Some(missing);
        }
    }
    Err(closest.unwrap_or_default())
}

/// The first value under `key` anywhere in `value`.
pub(crate) fn find_value<'a>(value: &'a Value, key: &str) -> Option<&'a Value> {
    match value {
        Value::Object(map) => map
            .get(key)
            .or_else(|| map.values().find_map(|inner| find_value(inner, key))),
        Value::Array(items) => items.iter().find_map(|inner| find_value(inner, key)),
        _ => None,
    }
}

/// Dotted names of the `wanted` settings that `applied` does not hold.
fn unapplied_settings(
    wanted: &Map<String, Value>,
    applied: Option<&Value>,
    prefix: &str,
) -> Vec<String> {
    let mut missing = Vec::new();
    for (key, value) in wanted {
        let name = format!("{prefix}{key}");
        let got = applied.and_then(|applied| applied.get(key));
        match value {
            Value::Object(inner) => {
                missing.extend(unapplied_settings(inner, got, &format!("{name}.")));
            }
            _ if got != Some(value) => missing.push(name),
            _ => {}
        }
    }
    missing
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum JobOutcome {
    Running,
    Finished,
    Failed,
}

/// What a Freerouting 2.x job state means for the poll loop. QUEUED,
/// READY_TO_START, RUNNING, PAUSED and STOPPING are still on their way.
fn job_outcome(state: &str) -> JobOutcome {
    match state.to_ascii_uppercase().as_str() {
        "COMPLETED" => JobOutcome::Finished,
        // Freerouting's own job timeout (12 h unless set) leaves no output.
        "FAILED" | "CANCELLED" | "CANCELED" | "TERMINATED" | "INVALID" | "TIMED_OUT" => {
            JobOutcome::Failed
        }
        _ => JobOutcome::Running,
    }
}

/// The numeric and short text fields of a job-details answer, for a
/// progress message (Freerouting reports pass numbers, counts and scores).
fn job_progress_summary(details: &Value) -> String {
    let mut parts = Vec::new();
    let mut collect = |object: &serde_json::Map<String, Value>, prefix: &str| {
        for (key, value) in object {
            if key == "state" || key == "status" || parts.len() >= 8 {
                continue;
            }
            match value {
                Value::Number(n) => parts.push(format!("{prefix}{key}={n}")),
                Value::String(s) if s.len() <= 40 => parts.push(format!("{prefix}{key}={s}")),
                _ => {}
            }
        }
    };
    if let Some(object) = details.as_object() {
        collect(object, "");
        for (key, value) in object {
            if let Some(inner) = value.as_object() {
                collect(inner, &format!("{key}."));
            }
        }
    }
    if parts.is_empty() {
        String::new()
    } else {
        format!(" ({})", parts.join(", "))
    }
}

fn validate_tool_contracts(tools: &BTreeMap<String, Value>) -> Result<()> {
    for (tool, paths) in [
        ("enqueue_job", &[(&["body", "session_id"][..], false)][..]),
        (
            "upload_job_input_from_local_file",
            &[(&["jobId"][..], true), (&["filePath"][..], true)][..],
        ),
        ("start_job", &[(&["path", "jobId"][..], true)][..]),
        ("get_job_details", &[(&["path", "jobId"][..], true)][..]),
        (
            "download_job_output_to_local_file",
            &[(&["jobId"][..], true), (&["filePath"][..], true)][..],
        ),
    ] {
        let schema = tools
            .get(tool)
            .with_context(|| format!("Freerouting MCP is missing required tool '{tool}'"))?;
        for (path, leaf_must_be_required) in paths {
            require_schema_path(tool, schema, path, *leaf_must_be_required)?;
        }
    }
    Ok(())
}

fn require_schema_path(
    tool: &str,
    schema: &Value,
    path: &[&str],
    leaf_must_be_required: bool,
) -> Result<()> {
    let mut current = schema;
    for (index, key) in path.iter().enumerate() {
        if current.get("type").and_then(Value::as_str) != Some("object") {
            bail!(
                "Freerouting MCP tool '{tool}' has an incompatible input schema at '{}'",
                path.join(".")
            );
        }
        let required = current
            .get("required")
            .and_then(Value::as_array)
            .is_some_and(|values| values.iter().any(|value| value.as_str() == Some(key)));
        if !required && (index + 1 < path.len() || leaf_must_be_required) {
            bail!(
                "Freerouting MCP tool '{tool}' does not require argument '{}'",
                path.join(".")
            );
        }
        current = current
            .get("properties")
            .and_then(|properties| properties.get(*key))
            .with_context(|| {
                format!(
                    "Freerouting MCP tool '{tool}' has no argument '{}'",
                    path.join(".")
                )
            })?;
    }
    Ok(())
}

struct LocalMcpClient {
    child: Option<Child>,
    stdin: ChildStdin,
    stdout: Lines<BufReader<ChildStdout>>,
    stderr_task: Option<JoinHandle<String>>,
    next_id: u64,
    server_protocol_version: String,
    /// The temporary data directory of a process given none; removed with
    /// the client, after the process has ended.
    _user_data: Option<tempfile::TempDir>,
}

impl LocalMcpClient {
    async fn start(jar: &Path, launch: &LaunchOptions) -> Result<Self> {
        let mut client = Self::spawn(jar, launch)?;
        if let Err(error) = client.initialize().await {
            let diagnostics = client.close().await;
            return if diagnostics.is_empty() {
                Err(error)
            } else {
                Err(error).context(format!("Freerouting stderr: {diagnostics}"))
            };
        }
        Ok(client)
    }

    fn spawn(jar: &Path, launch: &LaunchOptions) -> Result<Self> {
        let (user_data_path, user_data) = match &launch.user_data_path {
            Some(path) => {
                std::fs::create_dir_all(path).with_context(|| {
                    format!("create Freerouting data directory {}", path.display())
                })?;
                (path.clone(), None)
            }
            None => {
                let directory = tempfile::Builder::new()
                    .prefix("konnect-freerouting-")
                    .tempdir()
                    .context("create a temporary Freerouting data directory")?;
                (directory.path().to_path_buf(), Some(directory))
            }
        };
        let log = match &launch.stderr_log {
            Some(path) => Some(tokio::fs::File::from_std(
                std::fs::OpenOptions::new()
                    .write(true)
                    .create_new(true)
                    .open(path)
                    .with_context(|| format!("create Freerouting stderr log {}", path.display()))?,
            )),
            None => None,
        };
        let mut command = Command::new("java");
        command
            .args(jvm_arguments(launch))
            .arg("-jar")
            .arg(jar)
            .args(server_arguments(launch, &user_data_path))
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .kill_on_drop(true);
        let mut child = command
            .spawn()
            .context("start local Freerouting MCP server")?;
        #[cfg(test)]
        if let Some(pid) = child.id() {
            LAST_CHILD_PID.store(pid, Ordering::SeqCst);
        }
        let stdin = child.stdin.take().context("Freerouting MCP has no stdin")?;
        let stdout = child
            .stdout
            .take()
            .context("Freerouting MCP has no stdout")?;
        let stderr = child
            .stderr
            .take()
            .context("Freerouting MCP has no stderr")?;
        let stderr_task = tokio::spawn(drain_stderr(stderr, log, MAX_STDERR_BYTES));
        Ok(Self {
            child: Some(child),
            stdin,
            stdout: BufReader::new(stdout).lines(),
            stderr_task: Some(stderr_task),
            next_id: 1,
            server_protocol_version: String::new(),
            _user_data: user_data,
        })
    }

    async fn initialize(&mut self) -> Result<()> {
        let initialize = timeout(
            STARTUP_TIMEOUT,
            self.request(
                "initialize",
                json!({
                    "protocolVersion": MCP_PROTOCOL_VERSION,
                    "capabilities": {},
                    "clientInfo": { "name": "Konnect", "version": env!("CARGO_PKG_VERSION") }
                }),
                STARTUP_TIMEOUT,
            ),
        )
        .await
        .map_err(|_| anyhow::anyhow!("Freerouting MCP initialization timed out"))??;
        self.server_protocol_version = find_string(&initialize, &["protocolVersion"])
            .unwrap_or_else(|| MCP_PROTOCOL_VERSION.to_string());
        self.notify("notifications/initialized", json!({})).await?;
        Ok(())
    }

    async fn list_tools(&mut self) -> Result<BTreeMap<String, Value>> {
        let result = self
            .request("tools/list", json!({}), REQUEST_TIMEOUT)
            .await?;
        let tools = result
            .get("tools")
            .and_then(Value::as_array)
            .context("Freerouting tools/list returned no tools array")?;
        tools
            .iter()
            .map(|tool| {
                let name = tool
                    .get("name")
                    .and_then(Value::as_str)
                    .map(str::to_string)
                    .context("Freerouting tools/list contains an unnamed tool")?;
                let schema = tool
                    .get("inputSchema")
                    .cloned()
                    .context("Freerouting tools/list contains a tool without inputSchema")?;
                Ok((name, schema))
            })
            .collect()
    }

    async fn call_tool(&mut self, name: &str, arguments: Value) -> Result<Value> {
        let result = self
            .request(
                "tools/call",
                json!({ "name": name, "arguments": arguments }),
                REQUEST_TIMEOUT,
            )
            .await
            .with_context(|| format!("call Freerouting MCP tool '{name}'"))?;
        if result.get("isError").and_then(Value::as_bool) == Some(true) {
            bail!(
                "Freerouting MCP tool '{name}' returned an error: {}",
                compact(&result)
            );
        }
        Ok(expand_text_content(result))
    }

    async fn request(&mut self, method: &str, params: Value, limit: Duration) -> Result<Value> {
        let id = self.next_id;
        self.next_id = self.next_id.saturating_add(1);
        self.write_json(&json!({
            "jsonrpc": "2.0",
            "id": id,
            "method": method,
            "params": params
        }))
        .await?;
        let deadline = Instant::now() + limit;
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                bail!("Freerouting MCP request '{method}' timed out");
            }
            let line = timeout(remaining, self.stdout.next_line())
                .await
                .map_err(|_| anyhow::anyhow!("Freerouting MCP request '{method}' timed out"))??
                .context("Freerouting MCP closed stdout before responding")?;
            if line.trim().is_empty() {
                continue;
            }
            let response: Value = serde_json::from_str(&line)
                .with_context(|| format!("Freerouting MCP emitted non-JSON stdout: {line}"))?;
            if response.get("id") != Some(&json!(id)) {
                if response.get("method").is_some() {
                    continue;
                }
                bail!("Freerouting MCP returned an unexpected response id");
            }
            if let Some(error) = response.get("error") {
                bail!(
                    "Freerouting MCP request '{method}' failed: {}",
                    compact(error)
                );
            }
            return response
                .get("result")
                .cloned()
                .context("Freerouting MCP response has no result");
        }
    }

    async fn notify(&mut self, method: &str, params: Value) -> Result<()> {
        self.write_json(&json!({ "jsonrpc": "2.0", "method": method, "params": params }))
            .await
    }

    async fn write_json(&mut self, value: &Value) -> Result<()> {
        let mut bytes = serde_json::to_vec(value)?;
        bytes.push(b'\n');
        self.stdin
            .write_all(&bytes)
            .await
            .context("write Freerouting MCP stdin")?;
        self.stdin
            .flush()
            .await
            .context("flush Freerouting MCP stdin")
    }

    async fn close(mut self) -> String {
        let _ = self.stdin.shutdown().await;
        if let Some(mut child) = self.child.take() {
            if timeout(SHUTDOWN_TIMEOUT, child.wait()).await.is_err() {
                let _ = child.kill().await;
                let _ = child.wait().await;
            }
        }
        match self.stderr_task.take() {
            Some(task) => task.await.unwrap_or_default(),
            None => String::new(),
        }
    }
}

impl Drop for LocalMcpClient {
    fn drop(&mut self) {
        if let Some(mut child) = self.child.take() {
            let _ = child.start_kill();
            if let Ok(runtime) = tokio::runtime::Handle::try_current() {
                runtime.spawn(async move {
                    let _ = child.wait().await;
                });
            }
        }
        if let Some(task) = self.stderr_task.take() {
            task.abort();
        }
    }
}

/// Read the router's stderr to its end: every byte goes to `log`, and the
/// last `max` bytes come back for error messages. Freerouting's startup
/// alone can fill that budget, and the routing summary comes last, so the
/// tail is the end of the stream.
async fn drain_stderr(
    stderr: impl tokio::io::AsyncRead + Unpin,
    mut log: Option<tokio::fs::File>,
    max: usize,
) -> String {
    let mut reader = BufReader::new(stderr);
    let mut tail = Vec::new();
    let mut dropped = 0usize;
    let mut log_error = None;
    let mut chunk = vec![0u8; 8192];
    while let Ok(read) = reader.read(&mut chunk).await {
        if read == 0 {
            break;
        }
        if let Some(file) = log.as_mut() {
            if let Err(error) = file.write_all(&chunk[..read]).await {
                // Keep reading: a full disk must not stall the router.
                log_error = Some(error);
                log = None;
            }
        }
        dropped += keep_tail(&mut tail, &chunk[..read], max);
    }
    if let Some(file) = log.as_mut() {
        if let Err(error) = file.flush().await {
            log_error = Some(error);
        }
    }
    let mut text = String::from_utf8_lossy(&tail).trim().to_string();
    if dropped > 0 {
        text = format!("[{dropped} earlier bytes of Freerouting stderr dropped]\n{text}");
    }
    if let Some(error) = log_error {
        text = format!("[Freerouting stderr log incomplete: {error}]\n{text}");
    }
    text
}

/// Append `chunk` to `tail`, then drop the oldest bytes beyond `max`;
/// returns how many were dropped.
fn keep_tail(tail: &mut Vec<u8>, chunk: &[u8], max: usize) -> usize {
    tail.extend_from_slice(chunk);
    let excess = tail.len().saturating_sub(max);
    tail.drain(..excess);
    excess
}

fn expand_text_content(mut result: Value) -> Value {
    let parsed = result
        .get("content")
        .and_then(Value::as_array)
        .into_iter()
        .flatten()
        .filter_map(|item| item.get("text").and_then(Value::as_str))
        .filter_map(|text| serde_json::from_str::<Value>(text).ok())
        .collect::<Vec<_>>();
    if let Some(object) = result.as_object_mut() {
        object.insert("parsedTextContent".to_string(), Value::Array(parsed));
    }
    result
}

fn required_string(value: &Value, keys: &[&str], label: &str) -> Result<String> {
    find_string(value, keys).with_context(|| {
        format!(
            "Freerouting MCP response has no {label}: {}",
            compact(value)
        )
    })
}

fn find_string(value: &Value, keys: &[&str]) -> Option<String> {
    match value {
        Value::Object(object) => {
            for (key, value) in object {
                if keys
                    .iter()
                    .any(|candidate| key.eq_ignore_ascii_case(candidate))
                {
                    if let Some(value) = value.as_str() {
                        return Some(value.to_string());
                    }
                }
            }
            object.values().find_map(|value| find_string(value, keys))
        }
        Value::Array(values) => values.iter().find_map(|value| find_string(value, keys)),
        _ => None,
    }
}

fn absolute_utf8(path: &Path) -> Result<String> {
    let absolute = if path.is_absolute() {
        path.to_path_buf()
    } else {
        std::env::current_dir()?.join(path)
    };
    absolute
        .to_str()
        .map(str::to_string)
        .with_context(|| format!("path is not UTF-8: {}", absolute.display()))
}

fn temporary_output_path(output: &Path) -> Result<PathBuf> {
    let parent = output.parent().unwrap_or_else(|| Path::new("."));
    let name = output
        .file_name()
        .and_then(|name| name.to_str())
        .context("SES output has no UTF-8 filename")?;
    Ok(parent.join(format!(".{name}.konnect-{}.tmp.ses", uuid::Uuid::new_v4())))
}

fn extension_is(path: &Path, expected: &str) -> bool {
    path.extension()
        .and_then(|extension| extension.to_str())
        .is_some_and(|extension| extension.eq_ignore_ascii_case(expected))
}

fn compact(value: &Value) -> String {
    let text = value.to_string();
    if text.chars().count() > 2_000 {
        let truncated: String = text.chars().take(2_000).collect();
        format!("{truncated}...")
    } else {
        text
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use konnect_ipc::{IpcEffectiveRoutingRules, IpcRoutingRules};

    #[cfg(windows)]
    fn process_is_running(pid: u32) -> bool {
        let output = std::process::Command::new("tasklist")
            .args(["/FI", &format!("PID eq {pid}"), "/FO", "CSV", "/NH"])
            .output()
            .expect("run tasklist");
        String::from_utf8_lossy(&output.stdout).contains(&format!("\"{pid}\""))
    }

    #[cfg(not(windows))]
    fn process_is_running(pid: u32) -> bool {
        std::process::Command::new("sh")
            .args(["-c", &format!("kill -0 {pid} 2>/dev/null")])
            .status()
            .expect("probe process")
            .success()
    }

    async fn wait_for_process_exit(pid: u32) -> bool {
        for _ in 0..50 {
            if !process_is_running(pid) {
                return true;
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
        false
    }

    fn routing_rules() -> IpcEffectiveRoutingRules {
        ["GND", "VCC"]
            .into_iter()
            .map(|net| {
                (
                    net.to_string(),
                    IpcRoutingRules {
                        class_name: "Default".to_string(),
                        constituents: vec!["Default".to_string()],
                        track_width_mm: Some(0.25),
                        clearance_mm: Some(0.2),
                        via_diameter_mm: Some(0.6),
                        via_drill_mm: Some(0.3),
                    },
                )
            })
            .collect()
    }

    #[test]
    fn nested_and_text_wrapped_identifiers_are_found() {
        let value = expand_text_content(json!({
            "content": [{ "type": "text", "text": "{\"data\":{\"sessionId\":\"abc\"}}" }]
        }));
        assert_eq!(find_string(&value, &["sessionId"]), Some("abc".to_string()));
    }

    #[test]
    fn only_completed_jobs_finish_and_timed_out_ones_fail() {
        for state in ["QUEUED", "READY_TO_START", "RUNNING", "PAUSED", "STOPPING"] {
            assert_eq!(job_outcome(state), JobOutcome::Running, "{state}");
        }
        assert_eq!(job_outcome("COMPLETED"), JobOutcome::Finished);
        for state in ["FAILED", "CANCELLED", "TERMINATED", "INVALID", "timed_out"] {
            assert_eq!(job_outcome(state), JobOutcome::Failed, "{state}");
        }
    }

    fn settings(max_passes: Option<u32>) -> RouteSettings {
        RouteSettings {
            max_passes,
            optimizer_enabled: Some(false),
            fanout_enabled: Some(false),
            job_timeout_seconds: Some(3600),
            poll_interval: Duration::from_secs(2),
            overall_timeout: Duration::from_secs(4000),
            launch: LaunchOptions::default(),
        }
    }

    #[test]
    fn job_settings_use_freeroutings_names_and_are_checked_in_its_answer() {
        let settings = settings(Some(8));
        assert_eq!(
            Value::Object(job_settings(&settings, SettingsShape::Flat)),
            json!({
                "max_passes": 8,
                "optimizer": { "enabled": false },
                "fanout": { "enabled": false }
            })
        );
        assert_eq!(
            Value::Object(job_settings(&settings, SettingsShape::Nested)),
            json!({
                "autorouter": { "max_passes": 8 },
                "optimizer": { "enabled": false },
                "fanout": { "enabled": false }
            })
        );
        // One request carries both shapes.
        assert_eq!(
            Value::Object(job_settings_body(&settings)),
            json!({
                "max_passes": 8,
                "autorouter": { "max_passes": 8 },
                "optimizer": { "enabled": false },
                "fanout": { "enabled": false }
            })
        );
        // Freerouting 2.3 echoes what it took; an unknown key is dropped.
        let answer = json!({ "status": 200, "body": { "router_settings": {
            "max_passes": 8,
            "optimizer": { "enabled": false }, "fanout": {}, "scoring": {}
        } } });
        assert_eq!(
            applied_settings_shape(&settings, find_value(&answer, "router_settings")),
            Err(vec!["fanout.enabled".to_string()])
        );
        assert_eq!(
            applied_settings_shape(&settings, None).unwrap_err().len(),
            3,
            "no echo proves nothing"
        );
    }

    #[test]
    fn both_releases_settings_echoes_are_accepted_and_named() {
        let four = settings(Some(4));
        // 2.3: the pass limit at the top; `autorouter` is not in its model.
        let flat = json!({ "router_settings": {
            "max_passes": 4,
            "optimizer": { "enabled": false }, "fanout": { "enabled": false }
        } });
        assert_eq!(
            applied_settings_shape(&four, find_value(&flat, "router_settings")),
            Ok(Some(SettingsShape::Flat))
        );
        // 2.5: the whole job comes back, its router settings nested.
        let nested = json!({ "id": "j", "state": "QUEUED", "router_settings": {
            "autorouter": { "enabled": true, "max_passes": 4 },
            "optimizer": { "enabled": false }, "fanout": { "enabled": false }
        } });
        assert_eq!(
            applied_settings_shape(&four, find_value(&nested, "router_settings")),
            Ok(Some(SettingsShape::Nested))
        );
        // 2.5 dropping the pass limit is refused, not taken for 2.3.
        let lost = json!({ "router_settings": {
            "autorouter": { "enabled": true },
            "optimizer": { "enabled": false }, "fanout": { "enabled": false }
        } });
        assert_eq!(
            applied_settings_shape(&four, find_value(&lost, "router_settings")),
            Err(vec!["autorouter.max_passes".to_string()])
        );
        // Without a pass limit the shapes agree and none is named.
        assert_eq!(
            applied_settings_shape(&settings(None), find_value(&nested, "router_settings")),
            Ok(None)
        );
    }

    #[test]
    fn stderr_capture_keeps_the_newest_bytes() {
        let mut tail = Vec::new();
        assert_eq!(keep_tail(&mut tail, b"startup ", 10), 0);
        assert_eq!(keep_tail(&mut tail, b"routed 12", 10), 7);
        assert_eq!(tail, b" routed 12");
    }

    #[test]
    fn output_path_is_unique_and_stays_beside_destination() {
        let output = Path::new("build/board.ses");
        let first = temporary_output_path(output).unwrap();
        let second = temporary_output_path(output).unwrap();
        assert_eq!(first.parent(), output.parent());
        assert_ne!(first, second);
        assert!(extension_is(&first, "ses"));
    }

    #[test]
    fn unauthenticated_servers_are_forced_to_loopback() {
        let data = Path::new("job/freerouting-data");
        let arguments = server_arguments(&LaunchOptions::default(), data);
        let has = |wanted: &str| arguments.iter().any(|argument| argument == wanted);
        assert!(has("--api_server.authentication.enabled=false"));
        assert!(has("--api_server-endpoints=http://127.0.0.1:0"));
        assert!(has("--mcp_server.authentication.enabled=false"));
        assert!(has("--mcp_server-endpoints=http://127.0.0.1:0"));
        assert!(has(&format!("--user_data_path={}", data.display())));
        assert!(!arguments
            .iter()
            .any(|argument| argument.contains("0.0.0.0") || argument.contains("target_api")));
    }

    #[test]
    fn each_process_gets_its_own_ports_and_only_an_allowlisted_heap_option() {
        let launch = LaunchOptions {
            api_port: 41_001,
            max_heap_mb: Some(6144),
            ..Default::default()
        };
        let arguments = server_arguments(&launch, Path::new("data"));
        assert!(arguments.contains(&"--api_server-endpoints=http://127.0.0.1:41001".to_string()));
        assert!(arguments
            .contains(&"--mcp_server.target_api_base_url=http://127.0.0.1:41001".to_string()));
        assert!(
            arguments.contains(&"--mcp_server-endpoints=http://127.0.0.1:0".to_string()),
            "the MCP endpoint is never a fixed port"
        );
        assert_eq!(jvm_arguments(&launch), vec!["-Xmx6144m".to_string()]);
        assert!(jvm_arguments(&LaunchOptions::default()).is_empty());

        let temp = tempfile::tempdir().unwrap();
        let jar = temp.path().join("freerouting.jar");
        let dsn = temp.path().join("board.dsn");
        std::fs::write(&jar, b"").unwrap();
        std::fs::write(&dsn, b"").unwrap();
        let mut routed = settings(Some(2));
        routed.poll_interval = Duration::from_secs(3);
        for (heap, valid) in [(Some(MIN_HEAP_MB - 1), false), (Some(MAX_HEAP_MB), true)] {
            routed.launch.max_heap_mb = heap;
            let result = validate_inputs(&jar, &dsn, &temp.path().join("out.ses"), &routed);
            assert_eq!(result.is_ok(), valid, "{heap:?}: {result:?}");
        }
    }

    #[test]
    fn required_tool_schema_drift_is_refused_before_routing() {
        let object = |properties: Value, required: Value| json!({ "type": "object", "properties": properties, "required": required });
        let mut tools = BTreeMap::new();
        tools.insert("enqueue_job".into(), object(json!({"body": object(json!({"session_id": {"type":"string"}}), json!(["session_id"]))}), json!(["body"])));
        tools.insert(
            "upload_job_input_from_local_file".into(),
            object(
                json!({"jobId":{"type":"string"},"filePath":{"type":"string"}}),
                json!(["jobId", "filePath"]),
            ),
        );
        tools.insert(
            "start_job".into(),
            object(
                json!({"path": object(json!({"jobId":{"type":"string"}}), json!(["jobId"]))}),
                json!(["path"]),
            ),
        );
        tools.insert("get_job_details".into(), tools["start_job"].clone());
        tools.insert(
            "download_job_output_to_local_file".into(),
            tools["upload_job_input_from_local_file"].clone(),
        );
        validate_tool_contracts(&tools).unwrap();

        tools.get_mut("start_job").unwrap()["properties"]["path"]["required"] = json!([]);
        let error = validate_tool_contracts(&tools).unwrap_err().to_string();
        assert!(
            error.contains("start_job") && error.contains("path.jobId"),
            "{error}"
        );
    }

    #[tokio::test]
    async fn the_whole_stderr_is_logged_and_only_its_tail_kept_in_memory() {
        let temp = tempfile::tempdir().unwrap();
        let path = default_stderr_log(&temp.path().join("board.ses"));
        assert_eq!(path, temp.path().join("board.freerouting.log"));
        let stream = (1..=5000)
            .map(|item| format!("INFO routed item {item}\n"))
            .collect::<String>();
        let log = tokio::fs::File::create(&path).await.unwrap();
        let tail = drain_stderr(stream.as_bytes(), Some(log), 64).await;

        assert_eq!(std::fs::read_to_string(&path).unwrap(), stream);
        assert!(tail.starts_with('['), "{tail}");
        assert!(tail.contains("earlier bytes of Freerouting stderr dropped"));
        assert!(tail.ends_with("INFO routed item 5000"), "{tail}");
        assert_eq!(
            finish_stderr_log(&path).await,
            Some(path.display().to_string())
        );

        // A router that wrote nothing leaves no log behind.
        let empty = temp.path().join("quiet.freerouting.log");
        let log = tokio::fs::File::create(&empty).await.unwrap();
        assert_eq!(drain_stderr(&b""[..], Some(log), 64).await, "");
        assert_eq!(finish_stderr_log(&empty).await, None);
        assert!(!empty.exists());
    }

    /// Optional local parity check for Freerouting's actual stdio MCP state
    /// machine. CI does not install Java or Freerouting.
    #[tokio::test]
    #[ignore = "requires Java and FREEROUTING_JAR"]
    async fn native_mcp_routes_the_export_fixture() {
        let jar = PathBuf::from(std::env::var_os("FREEROUTING_JAR").expect("set FREEROUTING_JAR"));
        let source = include_str!("../tests/fixtures/specctra_two_resistors.kicad_pcb");
        let temp = tempfile::tempdir().unwrap();
        let board = temp.path().join("board.kicad_pcb");
        let dsn = temp.path().join("board.dsn");
        let ses = temp.path().join("board.ses");
        std::fs::write(&board, source).unwrap();
        let export = crate::specctra::export_dsn(&board, source, &routing_rules()).unwrap();
        std::fs::write(&dsn, export.dsn).unwrap();

        let evidence = route_local(
            &jar,
            &dsn,
            &ses,
            &RouteSettings {
                max_passes: Some(2),
                optimizer_enabled: Some(false),
                fanout_enabled: None,
                job_timeout_seconds: Some(300),
                poll_interval: Duration::from_secs(2),
                overall_timeout: Duration::from_secs(300),
                launch: LaunchOptions::default(),
            },
        )
        .await
        .unwrap();
        assert_eq!(evidence.final_state, "COMPLETED");
        assert!(evidence.ses_bytes > 0);
        assert!(ses.is_file());
    }

    /// A board too big to finish in a minute: Konnect stops the job at its
    /// time limit, and the SES still arrives with the routing done so far.
    #[tokio::test]
    #[ignore = "requires Java, FREEROUTING_JAR and FREEROUTING_LARGE_DSN"]
    async fn native_mcp_time_limit_returns_the_partial_session() {
        let jar = PathBuf::from(std::env::var_os("FREEROUTING_JAR").expect("set FREEROUTING_JAR"));
        let dsn = PathBuf::from(
            std::env::var_os("FREEROUTING_LARGE_DSN").expect("set FREEROUTING_LARGE_DSN"),
        );
        let temp = tempfile::tempdir().unwrap();
        let ses = temp.path().join("partial.ses");

        let evidence = route_local(
            &jar,
            &dsn,
            &ses,
            &RouteSettings {
                max_passes: Some(10),
                optimizer_enabled: Some(false),
                fanout_enabled: Some(false),
                job_timeout_seconds: Some(75),
                poll_interval: Duration::from_secs(2),
                overall_timeout: Duration::from_secs(600),
                launch: LaunchOptions::default(),
            },
        )
        .await
        .unwrap();
        eprintln!("{evidence:#?}");
        assert_eq!(evidence.final_state, "STOPPED_AT_TIME_LIMIT");
        assert!(evidence.partial);
        assert!(evidence.ses_bytes > 0);
        assert!(ses.is_file());
    }

    #[tokio::test]
    #[ignore = "requires Java and FREEROUTING_JAR"]
    async fn overall_timeout_closes_child_and_removes_partial_output() {
        let jar = PathBuf::from(std::env::var_os("FREEROUTING_JAR").expect("set FREEROUTING_JAR"));
        let source = include_str!("../tests/fixtures/specctra_two_resistors.kicad_pcb");
        let temp = tempfile::tempdir().unwrap();
        let board = temp.path().join("board.kicad_pcb");
        let dsn = temp.path().join("board.dsn");
        let ses = temp.path().join("board.ses");
        std::fs::write(&board, source).unwrap();
        let export = crate::specctra::export_dsn(&board, source, &routing_rules()).unwrap();
        std::fs::write(&dsn, export.dsn).unwrap();

        let error = format!(
            "{:#}",
            route_local(
                &jar,
                &dsn,
                &ses,
                &RouteSettings {
                    max_passes: Some(100),
                    optimizer_enabled: Some(true),
                    fanout_enabled: None,
                    job_timeout_seconds: Some(300),
                    poll_interval: Duration::from_secs(5),
                    overall_timeout: Duration::from_millis(100),
                    launch: LaunchOptions::default(),
                },
            )
            .await
            .unwrap_err()
        );
        assert!(error.contains("overall timeout"), "{error}");
        let pid = LAST_CHILD_PID.load(Ordering::SeqCst);
        assert!(
            wait_for_process_exit(pid).await,
            "Freerouting child {pid} survived timeout"
        );
        assert!(!ses.exists());
        assert!(
            std::fs::read_dir(temp.path()).unwrap().all(|entry| !entry
                .unwrap()
                .file_name()
                .to_string_lossy()
                .contains(".tmp.ses")),
            "timeout left a temporary SES"
        );
    }

    #[tokio::test]
    #[ignore = "requires Java and FREEROUTING_JAR"]
    async fn cancelling_route_reaps_the_owned_child() {
        let jar = PathBuf::from(std::env::var_os("FREEROUTING_JAR").expect("set FREEROUTING_JAR"));
        let source = include_str!("../tests/fixtures/specctra_two_resistors.kicad_pcb");
        let temp = tempfile::tempdir().unwrap();
        let board = temp.path().join("board.kicad_pcb");
        let dsn = temp.path().join("board.dsn");
        let ses = temp.path().join("board.ses");
        std::fs::write(&board, source).unwrap();
        let export = crate::specctra::export_dsn(&board, source, &routing_rules()).unwrap();
        std::fs::write(&dsn, export.dsn).unwrap();

        LAST_CHILD_PID.store(0, Ordering::SeqCst);
        let task = tokio::spawn(async move {
            route_local(
                &jar,
                &dsn,
                &ses,
                &RouteSettings {
                    max_passes: Some(100),
                    optimizer_enabled: Some(true),
                    fanout_enabled: None,
                    job_timeout_seconds: Some(300),
                    poll_interval: Duration::from_secs(5),
                    overall_timeout: Duration::from_secs(300),
                    launch: LaunchOptions::default(),
                },
            )
            .await
        });
        let pid = loop {
            let pid = LAST_CHILD_PID.load(Ordering::SeqCst);
            if pid != 0 {
                break pid;
            }
            tokio::time::sleep(Duration::from_millis(50)).await;
        };
        task.abort();
        let _ = task.await;
        assert!(
            wait_for_process_exit(pid).await,
            "Freerouting child {pid} survived cancellation"
        );
    }
}
