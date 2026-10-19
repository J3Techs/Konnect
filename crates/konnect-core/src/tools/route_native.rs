//! The `native_routing` toolset: MCP tools for the native router.
//!
//! Every tool here reads the **saved** board file (`"source": "saved_file"`
//! in each response) and writes nothing to it. A stage the router has not
//! built yet is reported as an unsupported capability, never as an empty
//! result: a check that could not run is missing evidence, not a pass.
//!
//! | Tool | Stage |
//! |---|---|
//! | `connectivity_report` | S1 union-find over the saved copper |
//! | `audit_terminal_access` | S1/S4 terminal access audit |
//! | `plan_plane_vias` | S2 plane fanout, as an `add_routing` plan |
//! | `check_route_rules` | S7 check of a `routing_file` against the board |
//! | `route_native_region` | the pipeline on one region, in this process |
//! | `start/get/stop_native_route_job` | the pipeline in a runner process |

use crate::mcp::{error::ToolErrorKind, protocol::CallToolResult};
use crate::native_route_job::{
    self, guarded, CopperSource, NativeJobSpec, Quiet, StageError, StopKind,
};
use crate::tool;
use crate::tools::{get_path, ToolContext, ToolDef};
use konnect_route::api::Generation;
use konnect_route::check::ViolationKind;
use konnect_route::connectivity::ConnectivityReport;
use konnect_route::optimize::exact::{CheckerOptions, ExactChecker};
use konnect_route::plan::{PlanTrack, PlanVia, RoutePlan};
use konnect_route::{RouteBoard, RuleTable};
use serde_json::{json, Value};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

/// Largest region `route_native_region` routes in-process (mm per side);
/// anything larger is a job.
const REGION_MAX_MM: f64 = 60.0;
const LISTED_DEFAULT: u64 = 50;

fn board_props() -> Value {
    json!({
        "board": { "type": "string", "description": "Saved .kicad_pcb to read (the file on disk, not the live editor)" },
        "project": { "type": "string", "description": ".kicad_pro whose net classes and rules apply; defaults to the one beside the board" },
        "rules": { "type": "string", "description": ".kicad_dru; defaults to the one beside the project" }
    })
}

fn with_props(extra: Value, required: &[&str]) -> Value {
    let mut props = board_props();
    if let (Some(p), Some(e)) = (props.as_object_mut(), extra.as_object()) {
        p.extend(e.clone());
    }
    json!({ "type": "object", "properties": props, "required": required })
}

fn job_props() -> Value {
    json!({
        "output_plan": { "type": "string", "description": "New routing_file (JSON) to write; an existing file is never replaced" },
        "source": { "type": "string", "enum": ["native", "board_unlocked", "plan"], "default": "native",
                    "description": "native: route open connections with the native stages; board_unlocked: optimize the board's own unlocked tracks and vias; plan: optimize the copper in source_plan" },
        "source_plan": { "type": "string", "description": "routing_file read when source is plan" },
        "region": { "type": "object", "description": "Work only inside this box, mm; copper outside is kept as board copper",
                    "properties": { "x1": { "type": "number" }, "y1": { "type": "number" }, "x2": { "type": "number" }, "y2": { "type": "number" } },
                    "required": ["x1", "y1", "x2", "y2"] },
        "nets": { "type": "array", "items": { "type": "string", "minLength": 1 }, "minItems": 1, "description": "Work only on these nets" },
        "threads": { "type": "integer", "minimum": 1, "maximum": 256, "description": "Worker threads (all cores by default)" },
        "optimize": { "type": "boolean", "default": true, "description": "Run the exact-checked optimizer (S6)" },
        "zones_are_obstacles": { "type": "boolean", "default": true, "description": "Treat other nets' zone fills as copper" }
    })
}

pub(crate) fn tools() -> Vec<ToolDef> {
    vec![
        tool!(
            "connectivity_report",
            "Union-find connectivity of the saved board's copper: per net, the islands its pads fall into and the connections still needed to join them. Zone pieces with no fill are left out (their outline only bounds copper) and counted. Reads the saved file only.",
            with_props(json!({
                "max_listed": { "type": "integer", "minimum": 0, "maximum": 10000, "default": LISTED_DEFAULT, "description": "Nets listed, most connections needed first; the totals always cover every net" }
            }), &["board"]),
            |args, ctx| async move { handle_connectivity_report(args, ctx).await }
        ),
        tool!(
            "audit_terminal_access",
            "For each pad of the named footprints, which copper layers can leave it legally (outside its courtyard) under the board's rules. Pads with no exit are listed with the reason. Reads the saved file only.",
            with_props(json!({
                "references": { "type": "array", "items": { "type": "string", "minLength": 1 }, "minItems": 1, "description": "Footprint references to audit" }
            }), &["board", "references"]),
            |args, ctx| async move { handle_audit_terminal_access(args, ctx).await }
        ),
        tool!(
            "plan_plane_vias",
            "Plan one exact-checked via (and stub) per plane-net pad that is not yet joined to its plane, and write them as an add_routing routing_file. Nothing is written to the board; apply the file with add_routing (locked: true keeps them from later rip-up). Reads the saved file only.",
            with_props(json!({
                "output_plan": { "type": "string", "description": "New routing_file to write; an existing file is never replaced" }
            }), &["board", "output_plan"]),
            |args, ctx| async move { handle_plan_plane_vias(args, ctx).await }
        ),
        tool!(
            "check_route_rules",
            "Check the copper in a routing_file against the saved board and itself with the router's exact kernel: clearance, hole clearance, hole-to-hole, edge clearance, keepouts, track width and via size, using the project's rules. Reports whether the authoritative S7 checker ran or the optimizer's interim exact checker stood in. KiCad's DRC stays the final authority.",
            with_props(json!({
                "routing_file": { "type": "string", "description": "add_routing routing_file to check" },
                "zones_are_obstacles": { "type": "boolean", "default": true, "description": "Treat other nets' zone fills as copper (a refill would carve them)" },
                "max_listed": { "type": "integer", "minimum": 0, "maximum": 10000, "default": LISTED_DEFAULT }
            }), &["board", "routing_file"]),
            |args, ctx| async move { handle_check_route_rules(args, ctx).await }
        ),
        tool!(
            "route_native_region",
            "Run the native pipeline on one region of the saved board in this process (at most 60 mm a side) and write the result as an add_routing routing_file: route (or take existing copper), optimize with exact checks, check, plan. Use start_native_route_job for a whole board.",
            {
                let mut s = with_props(job_props(), &["board", "output_plan", "region"]);
                s["properties"]["region"]["description"] = json!("The region to work on, mm (required here)");
                s
            },
            |args, ctx| async move { handle_route_native_region(args, ctx).await }
        ),
        tool!(
            "start_native_route_job",
            "Start the native pipeline on the saved board in a separate Konnect runner process that outlives this session; several jobs (one per output_plan) run at once. Returns once the runner is running. State lives in '<output_plan>.konnect-native-job/' (status.json, progress.jsonl, checkpoint.json, report.json). Follow with get_native_route_job; end early with stop_native_route_job; resume: true continues a stopped or lost job from its checkpoint.",
            {
                let mut p = job_props();
                p["time_limit_seconds"] = json!({ "type": "integer", "minimum": 1, "maximum": 86400, "description": "Stop at the next stage boundary after this long, keeping the checkpoint" });
                p["resume"] = json!({ "type": "boolean", "default": false, "description": "Continue the job already in the job directory (same arguments)" });
                with_props(p, &["board", "output_plan"])
            },
            |args, ctx| async move { handle_start_native_route_job(args, ctx).await }
        ),
        tool!(
            "get_native_route_job",
            "Report a job started by start_native_route_job: state (RUNNING, COMPLETED, STOPPED_ON_REQUEST, STOPPED_AT_TIME_LIMIT, STAGE_UNAVAILABLE, FAILED, or RUNNER_LOST when the runner stopped writing its heartbeat), the current stage, the report when finished, and whether the plan and a checkpoint exist. Reads the job directory only.",
            json!({
                "type": "object",
                "properties": { "output_plan": { "type": "string", "description": "The output_plan the job was started with" } },
                "required": ["output_plan"]
            }),
            |args, ctx| async move { handle_get_native_route_job(args, ctx).await }
        ),
        tool!(
            "stop_native_route_job",
            "Ask a job started by start_native_route_job to stop at its next stage boundary; its checkpoint is kept for resume. Waits up to wait_seconds and reports the job.",
            json!({
                "type": "object",
                "properties": {
                    "output_plan": { "type": "string", "description": "The output_plan the job was started with" },
                    "wait_seconds": { "type": "integer", "minimum": 0, "maximum": 600, "default": 60 }
                },
                "required": ["output_plan"]
            }),
            |args, ctx| async move { handle_stop_native_route_job(args, ctx).await }
        ),
    ]
}

// ─── Argument helpers ────────────────────────────────────────────────────

type Refusal = CallToolResult;

fn invalid(field: &str, reason: &str) -> Refusal {
    crate::tools::invalid_arg(field, reason)
}

fn opt_path(args: &Value, key: &str) -> Result<Option<PathBuf>, Refusal> {
    match args.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::String(s)) if !s.is_empty() => Ok(Some(PathBuf::from(s))),
        Some(_) => Err(invalid(key, "must be a non-empty path string")),
    }
}

fn opt_bool(args: &Value, key: &str, default: bool) -> Result<bool, Refusal> {
    match args.get(key) {
        None | Some(Value::Null) => Ok(default),
        Some(Value::Bool(b)) => Ok(*b),
        Some(_) => Err(invalid(key, "must be true or false")),
    }
}

fn opt_count(args: &Value, key: &str, default: u64, max: u64) -> Result<u64, Refusal> {
    match args.get(key) {
        None | Some(Value::Null) => Ok(default),
        Some(v) => match v.as_u64() {
            Some(n) if n <= max => Ok(n),
            _ => Err(invalid(key, &format!("must be an integer from 0 to {max}"))),
        },
    }
}

fn opt_strings(args: &Value, key: &str) -> Result<Option<Vec<String>>, Refusal> {
    match args.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::Array(a)) => {
            let v: Vec<String> = a
                .iter()
                .filter_map(|x| x.as_str().filter(|s| !s.is_empty()).map(String::from))
                .collect();
            if v.is_empty() || v.len() != a.len() {
                return Err(invalid(
                    key,
                    "must be a non-empty array of non-empty strings",
                ));
            }
            Ok(Some(v))
        }
        Some(_) => Err(invalid(key, "must be an array of strings")),
    }
}

fn region(args: &Value) -> Result<Option<[f64; 4]>, Refusal> {
    let Some(r) = args.get("region").filter(|v| !v.is_null()) else {
        return Ok(None);
    };
    let get = |k: &str| r.get(k).and_then(Value::as_f64).filter(|v| v.is_finite());
    match (get("x1"), get("y1"), get("x2"), get("y2")) {
        (Some(x1), Some(y1), Some(x2), Some(y2)) if x1 < x2 && y1 < y2 => {
            Ok(Some([x1, y1, x2, y2]))
        }
        (Some(_), Some(_), Some(_), Some(_)) => Err(invalid("region", "needs x1 < x2 and y1 < y2")),
        _ => Err(invalid("region", "needs finite x1, y1, x2 and y2 (mm)")),
    }
}

fn file_must_exist(path: &Path, field: &str) -> Result<(), Refusal> {
    if path.is_file() {
        Ok(())
    } else {
        Err(CallToolResult::error_kind(
            ToolErrorKind::FileNotFound {
                path: path.display().to_string(),
            },
            format!("{field} is not a file: {}", path.display()),
        ))
    }
}

/// A spec for the board arguments every tool shares.
fn base_spec(args: &Value, output_plan: PathBuf) -> anyhow::Result<Result<NativeJobSpec, Refusal>> {
    let board = get_path(args, "board")?;
    let project = match opt_path(args, "project") {
        Ok(p) => p,
        Err(r) => return Ok(Err(r)),
    };
    let rules = match opt_path(args, "rules") {
        Ok(p) => p,
        Err(r) => return Ok(Err(r)),
    };
    if let Err(r) = file_must_exist(&board, "board") {
        return Ok(Err(r));
    }
    Ok(Ok(NativeJobSpec {
        board,
        project,
        rules,
        output_plan,
        source: CopperSource::Native,
        region_mm: None,
        nets: None,
        threads: None,
        optimize: true,
        zones_are_obstacles: true,
        time_limit_seconds: None,
    }))
}

/// The job-shaped arguments (`route_native_region`, `start_native_route_job`).
fn job_spec(args: &Value) -> anyhow::Result<Result<NativeJobSpec, Refusal>> {
    let output = get_path(args, "output_plan")?;
    let mut spec = match base_spec(args, output)? {
        Ok(s) => s,
        Err(r) => return Ok(Err(r)),
    };
    let parsed = (|| -> Result<(), Refusal> {
        spec.source = match args.get("source").and_then(Value::as_str) {
            None | Some("native") => CopperSource::Native,
            Some("board_unlocked") => CopperSource::BoardUnlocked,
            Some("plan") => {
                let Some(path) = opt_path(args, "source_plan")? else {
                    return Err(invalid("source_plan", "is required when source is plan"));
                };
                file_must_exist(&path, "source_plan")?;
                CopperSource::Plan { path }
            }
            Some(other) => {
                return Err(invalid(
                    "source",
                    &format!("'{other}' is not native, board_unlocked or plan"),
                ))
            }
        };
        spec.region_mm = region(args)?;
        spec.nets = opt_strings(args, "nets")?;
        spec.threads = match args.get("threads") {
            None | Some(Value::Null) => None,
            Some(v) => match v.as_u64() {
                Some(n) if (1..=256).contains(&n) => Some(n as usize),
                _ => return Err(invalid("threads", "must be an integer from 1 to 256")),
            },
        };
        spec.optimize = opt_bool(args, "optimize", true)?;
        spec.zones_are_obstacles = opt_bool(args, "zones_are_obstacles", true)?;
        spec.time_limit_seconds = match args.get("time_limit_seconds") {
            None | Some(Value::Null) => None,
            Some(v) => match v.as_u64() {
                Some(n) if (1..=86_400).contains(&n) => Some(n),
                _ => {
                    return Err(invalid(
                        "time_limit_seconds",
                        "must be an integer from 1 to 86400",
                    ))
                }
            },
        };
        Ok(())
    })();
    if let Err(r) = parsed {
        return Ok(Err(r));
    }
    // Make every path absolute: a runner resolves nothing itself.
    let cwd = std::env::current_dir()?;
    let abs = |p: &PathBuf| {
        if p.is_absolute() {
            p.clone()
        } else {
            cwd.join(p)
        }
    };
    spec.board = abs(&spec.board);
    spec.output_plan = abs(&spec.output_plan);
    spec.project = spec.project.as_ref().map(abs);
    spec.rules = spec.rules.as_ref().map(abs);
    if let CopperSource::Plan { path } = &spec.source {
        spec.source = CopperSource::Plan { path: abs(path) };
    }
    Ok(Ok(spec))
}

fn loaded(
    spec: &NativeJobSpec,
) -> Result<(RouteBoard, konnect_route::ProjectRuleTable), CallToolResult> {
    spec.load_inputs()
        .map(|(b, _, r)| (b, r))
        .map_err(|e| CallToolResult::error(format!("could not read the board or its rules: {e:#}")))
}

fn unavailable(capability: &str, error: &anyhow::Error) -> CallToolResult {
    CallToolResult::error_kind(
        ToolErrorKind::UnsupportedCapability {
            capability: capability.to_string(),
            kicad_version: None,
        },
        format!(
            "{capability} cannot run yet: {error:#}. No result was produced; this is missing \
             evidence, not an empty finding."
        ),
    )
}

/// A stage that could not give a result: unavailable when it is the
/// unbuilt stub, an error naming the panic when it is built and failed.
fn stage_failure(capability: &str, error: StageError) -> CallToolResult {
    match error {
        StageError::Unavailable { .. } => unavailable(capability, &anyhow::Error::new(error)),
        StageError::Panicked { .. } => CallToolResult::error(format!(
            "{capability} failed: {error}. No result was produced."
        )),
    }
}

// ─── Handlers ────────────────────────────────────────────────────────────

/// Run a handler that loads the board and checks it (half a second or
/// more) on the blocking pool, off the server's async workers.
async fn blocking(
    args: &Value,
    f: fn(&Value) -> anyhow::Result<CallToolResult>,
) -> anyhow::Result<CallToolResult> {
    let args = args.clone();
    tokio::task::spawn_blocking(move || f(&args)).await?
}

async fn handle_connectivity_report(
    args: &Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    blocking(args, connectivity_report_blocking).await
}

async fn handle_audit_terminal_access(
    args: &Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    blocking(args, audit_terminal_access_blocking).await
}

async fn handle_plan_plane_vias(
    args: &Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    blocking(args, plan_plane_vias_blocking).await
}

async fn handle_check_route_rules(
    args: &Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    blocking(args, check_route_rules_blocking).await
}

fn connectivity_report_blocking(args: &Value) -> anyhow::Result<CallToolResult> {
    let spec = match base_spec(args, PathBuf::new())? {
        Ok(s) => s,
        Err(r) => return Ok(r),
    };
    let listed = match opt_count(args, "max_listed", LISTED_DEFAULT, 10_000) {
        Ok(n) => n as usize,
        Err(r) => return Ok(r),
    };
    let (board, _) =
        konnect_route::load_board_file(&spec.board, &konnect_route::LoadOptions::default())
            .map_err(|e| anyhow::anyhow!("could not read {}: {e}", spec.board.display()))?;
    let t = std::time::Instant::now();
    let report = ConnectivityReport::build(&board, &konnect_geom::ArcPolicy::default());
    let mut nets: Vec<(usize, &str, usize)> = report
        .nets
        .iter()
        .filter(|n| n.connections_needed() > 0)
        .map(|n| {
            (
                n.connections_needed(),
                board.net(n.net).name.as_str(),
                n.pad_island_count(),
            )
        })
        .collect();
    nets.sort_by(|a, b| b.0.cmp(&a.0).then(a.1.cmp(b.1)));
    let incomplete = nets.len();
    Ok(CallToolResult::json(&json!({
        "source": "saved_file",
        "board": spec.board,
        "multi_pad_nets": report.multi_pad_net_count(),
        "connections_needed": report.connection_count(),
        "incomplete_nets": incomplete,
        "unfilled_zones_skipped": report.unfilled_zones_skipped,
        "elapsed_ms": t.elapsed().as_millis() as u64,
        "nets": nets.iter().take(listed).map(|(c, name, islands)| json!({
            "net": name, "connections_needed": c, "pad_islands": islands
        })).collect::<Vec<_>>(),
        "listed": incomplete.min(listed),
    })))
}

fn audit_terminal_access_blocking(args: &Value) -> anyhow::Result<CallToolResult> {
    let spec = match base_spec(args, PathBuf::new())? {
        Ok(s) => s,
        Err(r) => return Ok(r),
    };
    let references = match opt_strings(args, "references") {
        Ok(Some(r)) => r,
        Ok(None) => return Ok(invalid("references", "is required")),
        Err(r) => return Ok(r),
    };
    let (board, rules) = match loaded(&spec) {
        Ok(v) => v,
        Err(r) => return Ok(r),
    };
    let mut footprints = Vec::new();
    let mut missing = Vec::new();
    for r in &references {
        match board.footprint_by_reference(r) {
            Some(f) => footprints.push(f),
            None => missing.push(r.clone()),
        }
    }
    if !missing.is_empty() {
        return Ok(invalid(
            "references",
            &format!("not on the board (or not unique): {}", missing.join(", ")),
        ));
    }
    let audit = match guarded("audit", || {
        konnect_route::audit::audit_terminal_access(&board, &rules, &footprints)
    }) {
        Ok(a) => a,
        Err(e) => return Ok(stage_failure("audit_terminal_access", e)),
    };
    let layer_names = |m: konnect_route::api::LayerMask| {
        m.iter()
            .map(|l| board.layers.name(l).to_string())
            .collect::<Vec<_>>()
    };
    let unreachable: Vec<Value> = audit
        .unreachable()
        .map(|t| {
            let pad = &board.pads[t.pad.index()];
            json!({
                "reference": board.footprints[pad.footprint.index()].reference,
                "pad": pad.number, "reason": t.reason,
            })
        })
        .collect();
    let mut per_layer: BTreeMap<String, usize> = BTreeMap::new();
    for t in &audit.terminals {
        for name in layer_names(t.reachable) {
            *per_layer.entry(name).or_default() += 1;
        }
    }
    Ok(CallToolResult::json(&json!({
        "source": "saved_file",
        "terminals": audit.terminals.len(),
        "unreachable": unreachable.len(),
        "reachable_per_layer": per_layer,
        "unreachable_pads": unreachable,
    })))
}

fn plan_plane_vias_blocking(args: &Value) -> anyhow::Result<CallToolResult> {
    let output = get_path(args, "output_plan")?;
    // Absolute, as the job tools keep it, so the reply names the file.
    let output = if output.is_absolute() {
        output
    } else {
        std::env::current_dir()?.join(output)
    };
    let spec = match base_spec(args, output.clone())? {
        Ok(s) => s,
        Err(r) => return Ok(r),
    };
    if output.exists() {
        return Ok(CallToolResult::error_kind(
            ToolErrorKind::Conflict {
                paths: vec![output.display().to_string()],
            },
            format!("{} already exists; it is never replaced", output.display()),
        ));
    }
    let (board, rules) = match loaded(&spec) {
        Ok(v) => v,
        Err(r) => return Ok(r),
    };
    let conn = ConnectivityReport::build(&board, &konnect_geom::ArcPolicy::default());
    let plan = match guarded("fanout", || {
        konnect_route::fanout::plan_plane_vias(
            &board,
            &rules,
            &conn,
            &konnect_route::fanout::FanoutSettings::default(),
        )
    }) {
        Ok(p) => p,
        Err(e) => return Ok(stage_failure("plan_plane_vias", e)),
    };
    let mm = |v: i32| konnect_geom::nm_to_mm(i64::from(v));
    let mut routing = RoutePlan::default();
    for s in &plan.sites {
        let net = board.net(s.net).name.clone();
        routing.vias.push(PlanVia {
            net: net.clone(),
            x: mm(s.via.x),
            y: mm(s.via.y),
            drill: mm(s.via_drill),
            diameter: mm(s.via_diameter),
        });
        if let Some((a, b, w)) = s.stub {
            routing.tracks.push(PlanTrack {
                net,
                layer: board.layers.name(s.layer).to_string(),
                width: mm(w),
                x1: mm(a.x),
                y1: mm(a.y),
                x2: mm(b.x),
                y2: mm(b.y),
            });
        }
    }
    let failed: Vec<Value> = plan
        .failed
        .iter()
        .map(|(pad, why)| {
            let p = &board.pads[pad.index()];
            json!({ "reference": board.footprints[p.footprint.index()].reference,
                    "pad": p.number, "failure": why })
        })
        .collect();
    let written = routing.item_count() > 0;
    if written {
        if let Err(e) =
            native_route_job::write_new_atomic(&output, routing.to_routing_file_json().as_bytes())
        {
            if !output.exists() {
                return Ok(CallToolResult::error(format!("plan not written: {e:#}")));
            }
            return Ok(CallToolResult::error_kind(
                ToolErrorKind::Conflict {
                    paths: vec![output.display().to_string()],
                },
                format!("{e:#}"),
            ));
        }
    }
    Ok(CallToolResult::json(&json!({
        "source": "saved_file",
        "sites": plan.sites.len(),
        "failed": failed.len(),
        "failures": failed,
        "plan_written": written,
        "output_plan": if written { json!(output) } else { Value::Null },
        "next": "Apply with add_routing (routing_file, locked: true), then refill zones and run DRC.",
    })))
}

fn check_route_rules_blocking(args: &Value) -> anyhow::Result<CallToolResult> {
    let spec = match base_spec(args, PathBuf::new())? {
        Ok(s) => s,
        Err(r) => return Ok(r),
    };
    let file = get_path(args, "routing_file")?;
    if let Err(r) = file_must_exist(&file, "routing_file") {
        return Ok(r);
    }
    let zones = match opt_bool(args, "zones_are_obstacles", true) {
        Ok(b) => b,
        Err(r) => return Ok(r),
    };
    let listed = match opt_count(args, "max_listed", LISTED_DEFAULT, 10_000) {
        Ok(n) => n as usize,
        Err(r) => return Ok(r),
    };
    let plan: RoutePlan = match std::fs::read_to_string(&file)
        .map_err(|e| e.to_string())
        .and_then(|t| serde_json::from_str(&t).map_err(|e| e.to_string()))
    {
        Ok(p) => p,
        Err(e) => {
            return Ok(invalid(
                "routing_file",
                &format!("is not a routing_file: {e}"),
            ))
        }
    };
    let (board, rules) = match loaded(&spec) {
        Ok(v) => v,
        Err(r) => return Ok(r),
    };
    let copper = match plan.to_routed(&board, Generation(0)) {
        Ok(c) => c,
        Err(e) => return Ok(invalid("routing_file", &e.to_string())),
    };
    let t = std::time::Instant::now();
    let (report, checker, authoritative) = match guarded("check", || {
        konnect_route::check::check_routed(&board, &rules, &copper)
    }) {
        Ok(r) => (r, "check_routed", true),
        Err(e @ StageError::Panicked { .. }) => return Ok(stage_failure("check_routed", e)),
        Err(StageError::Unavailable { .. }) => {
            let ck = ExactChecker::new(
                &board,
                &rules,
                CheckerOptions {
                    zones_are_obstacles: zones,
                    ..CheckerOptions::default()
                },
            );
            (ck.check_copper(&copper), "interim_exact", false)
        }
    };
    let kind_name = |k: ViolationKind| {
        serde_json::to_value(k)
            .ok()
            .and_then(|v| v.as_str().map(String::from))
            .unwrap_or_default()
    };
    let mut by_kind: BTreeMap<String, usize> = BTreeMap::new();
    for v in &report.violations {
        *by_kind.entry(kind_name(v.kind)).or_default() += 1;
    }
    let first: Vec<Value> = report
        .violations
        .iter()
        .take(listed)
        .map(|v| {
            let (x, y) = v.at.to_mm();
            json!({
                "kind": kind_name(v.kind), "x": x, "y": y,
                "layer": v.layer.map(|l| board.layers.name(l).to_string()),
                "required_mm": konnect_geom::nm_to_mm(i64::from(v.required)),
                "a": v.a, "b": v.b,
            })
        })
        .collect();
    Ok(CallToolResult::json(&json!({
        "source": "saved_file",
        "checker": checker,
        "authoritative": authoritative,
        "note": if authoritative { Value::Null } else {
            json!("The S7 checker is not built yet; the optimizer's exact checker ran instead. It covers clearance, holes, edges, keepouts, width and via size, but not track angle or segment length. KiCad DRC remains the final authority.")
        },
        "tracks": copper.tracks.len(),
        "vias": copper.vias.len(),
        "violations": report.violations.len(),
        "by_kind": by_kind,
        "listed": first,
        "elapsed_ms": t.elapsed().as_millis() as u64,
        "inert_rules": rules.inert_rules().len(),
    })))
}

async fn handle_route_native_region(
    args: &Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let spec = match job_spec(args)? {
        Ok(s) => s,
        Err(r) => return Ok(r),
    };
    let Some([x1, y1, x2, y2]) = spec.region_mm else {
        return Ok(invalid("region", "is required"));
    };
    if x2 - x1 > REGION_MAX_MM || y2 - y1 > REGION_MAX_MM {
        return Ok(invalid(
            "region",
            &format!("is larger than {REGION_MAX_MM} mm a side; use start_native_route_job"),
        ));
    }
    if spec.output_plan.exists() {
        return Ok(CallToolResult::error_kind(
            ToolErrorKind::Conflict {
                paths: vec![spec.output_plan.display().to_string()],
            },
            format!(
                "{} already exists; it is never replaced",
                spec.output_plan.display()
            ),
        ));
    }
    let run = spec.clone();
    let report =
        tokio::task::spawn_blocking(move || native_route_job::run_pipeline(&run, None, &mut Quiet))
            .await??;
    if let Some(why) = &report.stopped {
        if report.plan_path.is_none() {
            return Ok(match report.stop_kind {
                Some(StopKind::StageUnavailable) => {
                    unavailable("route_native_region", &anyhow::anyhow!("{why}"))
                }
                _ => CallToolResult::error(format!(
                    "route_native_region failed: {why}. No plan was written."
                )),
            });
        }
    }
    Ok(CallToolResult::json(&json!({
        "source": "saved_file",
        "output_plan": report.plan_path,
        "plan_items": report.plan_items,
        "add_routing_calls": report.plan_chunks,
        "work_tracks": report.work_tracks,
        "work_vias": report.work_vias,
        "stages": report.stages,
        "optimize": report.optimize,
        "check": report.check,
        "threads": report.telemetry.threads,
        "next": "Apply the plan with add_routing (geometry_check strict), refill zones, then run DRC.",
    })))
}

async fn handle_start_native_route_job(
    args: &Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let spec = match job_spec(args)? {
        Ok(s) => s,
        Err(r) => return Ok(r),
    };
    let resume = match opt_bool(args, "resume", false) {
        Ok(b) => b,
        Err(r) => return Ok(r),
    };
    let body = match native_route_job::start(&spec, resume).await {
        Ok(b) => b,
        Err(e) => {
            return Ok(CallToolResult::error(format!(
                "Native route job not started: {e:#}"
            )))
        }
    };
    if body["success"] == json!(false) {
        return Ok(CallToolResult::error(format!(
            "Native route job failed to start: {}",
            body["error"].as_str().unwrap_or("see the job status")
        )));
    }
    Ok(CallToolResult::json(&body))
}

async fn handle_get_native_route_job(
    args: &Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let output = get_path(args, "output_plan")?;
    match native_route_job::status(&output) {
        Ok(body) => Ok(CallToolResult::json(&body)),
        Err(e) => Ok(CallToolResult::error_kind(
            ToolErrorKind::FileNotFound {
                path: output.display().to_string(),
            },
            format!("{e:#}"),
        )),
    }
}

async fn handle_stop_native_route_job(
    args: &Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let output = get_path(args, "output_plan")?;
    let wait = match opt_count(args, "wait_seconds", 60, 600) {
        Ok(n) => std::time::Duration::from_secs(n),
        Err(r) => return Ok(r),
    };
    match native_route_job::stop(&output, wait).await {
        Ok(body) => Ok(CallToolResult::json(&body)),
        Err(e) => Ok(CallToolResult::error_kind(
            ToolErrorKind::FileNotFound {
                path: output.display().to_string(),
            },
            format!("{e:#}"),
        )),
    }
}

#[cfg(test)]
mod tests;
