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
//! | `apply_route_plan` | S8 apply: a plan of any size into the open board, with a KIID ledger |
//! | `map_route_plan_drc` | KiCad's DRC report read back onto an applied plan |
//!
//! `apply_route_plan` is the one tool here that writes: it creates the
//! plan's copper in the board KiCad holds open, never the saved file.

use crate::mcp::{error::ToolErrorKind, protocol::CallToolResult};
use crate::native_route_job::{
    self, guarded, CopperSource, NativeJobSpec, Quiet, StageError, StopKind,
};
use crate::tool;
use crate::tools::{get_path, route_native_apply, ToolContext, ToolDef};
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
        "zones_are_obstacles": { "type": "boolean", "default": true, "description": "Treat other nets' zone fills as copper" },
        "clear_findings": { "type": "boolean", "description": "After the optimizer, remove each piece of routed copper the exact check still finds a defect in, leaving its connections open (default: on for native routing, off for a plan or the board's own copper)" },
        "class_layers": { "type": "object", "additionalProperties": { "type": "array", "items": { "type": "string" }, "minItems": 1 },
                          "description": "Native routing: net class name to the copper layers it routes on (e.g. [\"F.Cu\"]); unlisted classes use every routable layer" },
        "layer_fallback": { "type": "array", "description": "Native routing: a second chance on more layers for a class's connections negotiation failed, routed around all copper already routed",
                            "items": { "type": "object", "required": ["class", "layers"], "additionalProperties": false, "properties": {
                                "class": { "type": "string" },
                                "layers": { "type": "array", "items": { "type": "string" }, "minItems": 1, "description": "Every layer of the second chance, the class's own included" },
                                "nets": { "type": "array", "items": { "type": "string" }, "minItems": 1, "description": "Only these nets of the class (names or * ? patterns)" },
                                "reference": { "type": "object", "required": ["plane"], "additionalProperties": false,
                                               "description": "Needs the class in class_layers. Tracks of the fallback's nets on the 'over' layers must lie over one filled piece of this plane, margin_mm inside it and clear of the holes other nets' vias and tracks will cut at the refill (checked again after the optimizer). Use a margin of at least the zone clearance",
                                               "properties": { "plane": { "type": "string" },
                                                               "over": { "type": "array", "items": { "type": "string" }, "description": "Default: the fallback layers beyond the class's own; only those are allowed" },
                                                               "margin_mm": { "type": "number", "minimum": 0 } } } } } },
        "neck_down": { "type": "object", "additionalProperties": false,
                       "description": "Native routing: let a track leave a pad narrower than its class width, within reach_mm of the pad (never below the rules' minimum)",
                       "properties": { "width_mm": { "type": "number", "exclusiveMinimum": 0, "default": 0.1 },
                                       "reach_mm": { "type": "number", "exclusiveMinimum": 0, "default": 1.0 },
                                       "classes": { "type": "array", "items": { "type": "string" }, "description": "Only these classes (default: every class not excluded)" },
                                       "exclude_classes": { "type": "array", "items": { "type": "string" }, "description": "Classes that never neck down" } } },
        "exclude_classes": { "type": "array", "items": { "type": "string" },
                             "description": "Native routing: classes left unrouted (zones or hand copper); their connections are reported as excluded_by_policy, not as failures. Default: every class whose track width is at least 0.5 mm. [] routes every class" },
        "net_classes": { "type": "string", "enum": ["auto", "kicad", "project"], "default": "auto",
                         "description": "Where net classes come from: kicad (KiCad's effective class per net over IPC; the board must be open), project (the .kicad_pro patterns, resolved as KiCad does), auto (kicad when KiCad has the board open, else project)" },
        "fanout_plan": { "type": "string", "description": "Native routing: write the plane fanout (S2) to this new routing_file, to apply with locked: true, and leave it out of output_plan" },
        "fanout_share_within_mm": { "type": "number", "minimum": 0, "description": "Native routing: a plane pad that gets no via of its own may share a same-net via placed for a pad within this distance (default: never)" }
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
                "output_plan": { "type": "string", "description": "New routing_file to write; an existing file is never replaced" },
                "share_within_mm": { "type": "number", "minimum": 0, "description": "A pad that gets no via of its own may share a same-net via placed for a pad within this distance (default: never)" },
                "retry_failed": { "type": "boolean", "default": true, "description": "Give pads the first round left without a site every exit pass plus long far exits, placed against the sites already chosen" }
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
            "apply_route_plan",
            "Create a routing_file's tracks and vias in the board KiCad holds open, however many there are: every member, net name and (with geometry_check strict) the geometry lint is checked before anything is written, then the plan goes in as net-whole CreateItems chunks of at most 20,000 items. Writes a ledger (the KIID KiCad gave each plan item) to ledger_output for map_route_plan_drc. A failure part-way reports the chunks already created. Does not save, refill or run DRC.",
            json!({
                "type": "object",
                "properties": {
                    "board": { "type": "string", "description": "The .kicad_pcb KiCad holds open" },
                    "routing_file": { "type": "string", "description": "add_routing routing_file (a native plan) to create" },
                    "ledger_output": { "type": "string", "description": "New JSON ledger to write; an existing file is never replaced" },
                    "locked": { "type": "boolean", "default": false, "description": "Create every item locked" },
                    "geometry_check": { "type": "string", "enum": ["strict", "warn"], "default": "strict",
                                        "description": "strict refuses a plan with any geometry lint defect; warn counts them and applies" },
                    "max_items": { "type": "integer", "minimum": 1, "maximum": 20000, "default": 20000, "description": "Items per CreateItems call" }
                },
                "required": ["board", "routing_file", "ledger_output"]
            }),
            |args, ctx| async move { handle_apply_route_plan(args, ctx).await }
        )
        .with_board_access(crate::tools::BoardAccess::LiveOnly),
        tool!(
            "map_route_plan_drc",
            "Read a KiCad DRC report (run_drc's output file) back onto a plan applied with apply_route_plan: each finding's items are followed by UUID to the KIIDs in the ledger, so it says which findings touch the plan, by rule, and which plan nets an error involves (the nets to reroute). Reads files only.",
            json!({
                "type": "object",
                "properties": {
                    "routing_file": { "type": "string", "description": "The routing_file that was applied" },
                    "ledger": { "type": "string", "description": "apply_route_plan's ledger_output" },
                    "drc_report": { "type": "string", "description": "run_drc output JSON" },
                    "max_listed": { "type": "integer", "minimum": 0, "maximum": 10000, "default": LISTED_DEFAULT, "description": "Plan-touching findings listed; the counts cover all" }
                },
                "required": ["routing_file", "ledger", "drc_report"]
            }),
            |args, ctx| async move { handle_map_route_plan_drc(args, ctx).await }
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

/// A region in mm: x1, y1, x2, y2.
type RegionMm = [f64; 4];

fn region(args: &Value) -> Result<Option<RegionMm>, Refusal> {
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
        clear_findings: None,
        time_limit_seconds: None,
        allowances: Default::default(),
        net_classes: None,
        fanout_plan: None,
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
        spec.clear_findings = match args.get("clear_findings") {
            None | Some(Value::Null) => None,
            Some(Value::Bool(b)) => Some(*b),
            Some(_) => return Err(invalid("clear_findings", "must be a boolean")),
        };
        let mut allowance_args = serde_json::Map::new();
        for key in [
            "class_layers",
            "layer_fallback",
            "neck_down",
            "exclude_classes",
            "fanout_share_within_mm",
        ] {
            if let Some(v) = args.get(key).filter(|v| !v.is_null()) {
                allowance_args.insert(key.to_string(), v.clone());
            }
        }
        spec.allowances = serde_json::from_value(Value::Object(allowance_args))
            .map_err(|e| invalid("allowances", &format!("cannot be read: {e}")))?;
        if spec.source != CopperSource::Native && !spec.allowances.is_default() {
            return Err(invalid(
                "allowances",
                "class_layers, layer_fallback, neck_down, exclude_classes and                  fanout_share_within_mm apply only when source is native",
            ));
        }
        spec.fanout_plan = opt_path(args, "fanout_plan")?;
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
    spec.fanout_plan = spec.fanout_plan.as_ref().map(abs);
    if let CopperSource::Plan { path } = &spec.source {
        spec.source = CopperSource::Plan { path: abs(path) };
    }
    Ok(Ok(spec))
}

/// Where a job's net classes should come from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ClassMode {
    Auto,
    Kicad,
    Project,
}

fn class_mode(args: &Value) -> Result<ClassMode, Refusal> {
    let mode = match args.get("net_classes") {
        None | Some(Value::Null) => None,
        Some(Value::String(m)) => Some(m.as_str()),
        Some(_) => return Err(invalid("net_classes", "must be auto, kicad or project")),
    };
    match mode {
        None | Some("auto") => Ok(ClassMode::Auto),
        Some("kicad") => Ok(ClassMode::Kicad),
        Some("project") => Ok(ClassMode::Project),
        Some(other) => Err(invalid(
            "net_classes",
            &format!("'{other}' is not auto, kicad or project"),
        )),
    }
}

/// Fill `spec.net_classes` with KiCad's effective class of every net when
/// `mode` asks for it and KiCad holds the board open. Returns how the
/// classes will be resolved, for the reply; a refusal when `mode` is
/// `kicad` and KiCad cannot answer.
async fn kicad_classes(
    ctx: &ToolContext,
    spec: &mut NativeJobSpec,
    mode: ClassMode,
) -> anyhow::Result<Result<Value, CallToolResult>> {
    if mode == ClassMode::Project || spec.source != CopperSource::Native {
        return Ok(Ok(json!({ "source": "project_patterns" })));
    }
    let why_not = if ctx.config.ipc_address.is_empty() {
        "no KiCad IPC address is configured".to_string()
    } else {
        let bound = super::with_bound_board_ipc_classified(ctx, &spec.board, |client, document| {
            client.get_effective_routing_rules_in(document)
        })
        .await?;
        match bound {
            Ok(super::BoardBinding::Bound(rules)) => {
                let map: BTreeMap<String, Vec<String>> = rules
                    .into_iter()
                    .map(|(net, r)| {
                        let mut names = vec![r.class_name];
                        names.extend(r.constituents);
                        (net, names)
                    })
                    .collect();
                let nets = map.len();
                spec.net_classes = Some(map);
                return Ok(Ok(json!({ "source": "kicad_ipc", "nets": nets })));
            }
            Ok(super::BoardBinding::Unserved { message, .. }) => message,
            Err(e) => e.message().to_string(),
        }
    };
    if mode == ClassMode::Kicad {
        return Ok(Err(CallToolResult::error(format!(
            "net_classes is kicad, but KiCad's classes could not be read: {why_not}. Nothing was started."
        ))));
    }
    Ok(Ok(
        json!({ "source": "project_patterns", "kicad_unavailable": why_not }),
    ))
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

async fn handle_connectivity_report(
    args: &Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    // Loading and checking a board takes half a second or more: run it on
    // the blocking pool, off the server's async workers.
    let args = args.clone();
    tokio::task::spawn_blocking(move || connectivity_report_blocking(&args)).await?
}

async fn handle_audit_terminal_access(
    args: &Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    // Loading and checking a board takes half a second or more: run it on
    // the blocking pool, off the server's async workers.
    let args = args.clone();
    tokio::task::spawn_blocking(move || audit_terminal_access_blocking(&args)).await?
}

async fn handle_plan_plane_vias(
    args: &Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    // Loading and checking a board takes half a second or more: run it on
    // the blocking pool, off the server's async workers.
    let args = args.clone();
    tokio::task::spawn_blocking(move || plan_plane_vias_blocking(&args)).await?
}

async fn handle_check_route_rules(
    args: &Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    // Loading and checking a board takes half a second or more: run it on
    // the blocking pool, off the server's async workers.
    let args = args.clone();
    tokio::task::spawn_blocking(move || check_route_rules_blocking(&args)).await?
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
    let mut settings = konnect_route::fanout::FanoutSettings::default();
    match args.get("share_within_mm") {
        None | Some(Value::Null) => {}
        Some(v) => match v
            .as_f64()
            .filter(|x| x.is_finite() && *x >= 0.0)
            .and_then(konnect_geom::mm_to_nm)
        {
            Some(nm) => settings.share_within = nm,
            None => {
                return Ok(invalid(
                    "share_within_mm",
                    "must be a non-negative length in mm",
                ))
            }
        },
    }
    settings.retry_failed = match opt_bool(args, "retry_failed", true) {
        Ok(b) => b,
        Err(r) => return Ok(r),
    };
    let conn = ConnectivityReport::build(&board, &konnect_geom::ArcPolicy::default());
    let plan = match guarded("fanout", || {
        konnect_route::fanout::plan_plane_vias(&board, &rules, &conn, &settings)
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
    let pad_label = |pad: konnect_route::api::PadId| {
        let p = &board.pads[pad.index()];
        format!(
            "{}.{}",
            board.footprints[p.footprint.index()]
                .reference
                .as_deref()
                .unwrap_or("?"),
            p.number
        )
    };
    let failed: Vec<Value> = plan
        .failed
        .iter()
        .map(|(pad, why)| {
            let p = &board.pads[pad.index()];
            let detail = plan.details.iter().find(|d| d.pad == *pad);
            let mut top: Vec<(&String, &u32)> =
                detail.map(|d| d.rejections.iter().collect()).unwrap_or_default();
            top.sort_by(|a, b| b.1.cmp(a.1).then(a.0.cmp(b.0)));
            top.truncate(3);
            json!({ "reference": board.footprints[p.footprint.index()].reference,
                    "pad": p.number,
                    "net": p.net.map(|n| board.net(n).name.clone()),
                    "failure": why,
                    "candidates": detail.map(|d| d.candidates),
                    "legal": detail.map(|d| d.legal),
                    "top_rejections": top.into_iter().map(|(k, v)| json!({ "reason": k, "count": v })).collect::<Vec<_>>(),
                    "blocked_by": detail.map(|d| d.blocked_by.iter().map(|b| pad_label(*b)).collect::<Vec<_>>()).unwrap_or_default() })
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
        "shared_vias": plan.stats.shared_vias,
        "retried_sites": plan.stats.retried_sites,
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
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let mut spec = match job_spec(args)? {
        Ok(s) => s,
        Err(r) => return Ok(r),
    };
    let mode = match class_mode(args) {
        Ok(m) => m,
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
    let classes = match kicad_classes(ctx, &mut spec, mode).await? {
        Ok(v) => v,
        Err(r) => return Ok(r),
    };
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
        "net_classes": classes,
        "allowances": report.allowances,
        "excluded_by_policy": report.excluded_by_policy,
        "reference": report.reference,
        "fanout_plan": report.fanout_plan_path,
        "next": "Apply the fanout plan (if written) locked, then the plan, with apply_route_plan; then one run_drc with sync_live_board and refill_zones.",
    })))
}

async fn handle_start_native_route_job(
    args: &Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let mut spec = match job_spec(args)? {
        Ok(s) => s,
        Err(r) => return Ok(r),
    };
    let resume = match opt_bool(args, "resume", false) {
        Ok(b) => b,
        Err(r) => return Ok(r),
    };
    let mode = match class_mode(args) {
        Ok(m) => m,
        Err(r) => return Ok(r),
    };
    let classes = match kicad_classes(ctx, &mut spec, mode).await? {
        Ok(v) => v,
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
    let mut body = body;
    body["net_classes"] = classes;
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

// ─── Apply and DRC read-back ──────────────────────────────────────────────

fn read_json<T: serde::de::DeserializeOwned>(path: &Path, field: &str) -> Result<T, Refusal> {
    file_must_exist(path, field)?;
    std::fs::read_to_string(path)
        .map_err(|e| e.to_string())
        .and_then(|t| serde_json::from_str(&t).map_err(|e| e.to_string()))
        .map_err(|e| invalid(field, &format!("cannot be read: {e}")))
}

fn required_path(args: &Value, key: &str) -> Result<PathBuf, Refusal> {
    match opt_path(args, key)? {
        Some(p) => Ok(p),
        None => Err(invalid(key, "is required")),
    }
}

/// What `apply_route_plan` writes to `ledger_output`.
#[derive(serde::Serialize, serde::Deserialize)]
struct LedgerFile {
    routing_file: PathBuf,
    locked: bool,
    complete: bool,
    ledger: route_native_apply::ApplyLedger,
}

/// The validated arguments of `apply_route_plan`.
struct ApplyArgs {
    plan: RoutePlan,
    locked: bool,
    strict: bool,
    max_items: usize,
}

fn apply_args(args: &Value, ledger_path: &Path) -> Result<ApplyArgs, Refusal> {
    let plan: RoutePlan = read_json(&required_path(args, "routing_file")?, "routing_file")?;
    let locked = opt_bool(args, "locked", false)?;
    let strict = match args.get("geometry_check").and_then(Value::as_str) {
        None | Some("strict") => true,
        Some("warn") => false,
        Some(_) => return Err(invalid("geometry_check", "must be strict or warn")),
    };
    let max_items = match opt_count(args, "max_items", 20_000, 20_000)? {
        0 => return Err(invalid("max_items", "must be at least 1")),
        n => n as usize,
    };
    if ledger_path.exists() {
        return Err(CallToolResult::error_kind(
            ToolErrorKind::Conflict {
                paths: vec![ledger_path.display().to_string()],
            },
            format!(
                "{} already exists; it is never replaced. Nothing was created.",
                ledger_path.display()
            ),
        ));
    }
    Ok(ApplyArgs {
        plan,
        locked,
        strict,
        max_items,
    })
}

async fn handle_apply_route_plan(
    args: &Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    use route_native_apply::{ApplyOutcome, IpcSink};
    let board = get_path(args, "board")?;
    let ledger_path = get_path(args, "ledger_output")?;
    let a = match apply_args(args, &ledger_path) {
        Ok(a) => a,
        Err(r) => return Ok(r),
    };
    let routing_file = get_path(args, "routing_file")?;
    let limits = super::route_geometry::Limits::default();
    let t = std::time::Instant::now();
    let applied = {
        let plan = a.plan.clone();
        let (locked, strict, max_items) = (a.locked, a.strict, a.max_items);
        super::with_bound_board_ipc_classified(ctx, &board, move |client, document| {
            let codes: BTreeMap<String, i32> = client
                .get_nets_in(document.clone())?
                .into_iter()
                .map(|n| (n.name, n.netcode))
                .collect();
            let mut sink = IpcSink { client, document };
            Ok(route_native_apply::apply_plan_with(
                &plan, locked, &limits, strict, max_items, &codes, &mut sink,
            ))
        })
        .await?
    };
    let outcome = match applied {
        Ok(super::BoardBinding::Bound(outcome)) => outcome,
        Ok(super::BoardBinding::Unserved { message, .. }) => {
            return Ok(CallToolResult::error(format!(
                "KiCAD must be running with the board loaded (IPC error: {message}). Nothing was created."
            )))
        }
        Err(konnect_ipc::IpcFailure::Target { error, .. }) => {
            return Ok(crate::tools::ipc_target_error_result(&error))
        }
        Err(error) => {
            return Ok(CallToolResult::error(format!(
                "KiCad IPC failed: {}. Inspect the board before retrying.",
                error.message()
            )))
        }
    };
    let elapsed_ms = t.elapsed().as_millis() as u64;
    let (ledger, complete, failure) = match outcome {
        Err(refusal) => {
            return Ok(CallToolResult::error_kind(
                ToolErrorKind::InvalidArgument {
                    field: "routing_file".to_string(),
                    reason: refusal.to_string(),
                },
                format!("Nothing was created: {refusal}"),
            ))
        }
        Ok(ApplyOutcome::Complete(ledger)) => (ledger, true, Value::Null),
        Ok(ApplyOutcome::Partial {
            ledger,
            failed_chunk,
            error,
        }) => (
            ledger,
            false,
            json!({ "failed_chunk": failed_chunk, "error": error }),
        ),
    };
    let summary = route_native_apply::ledger_summary(&ledger);
    let record = LedgerFile {
        routing_file,
        locked: a.locked,
        complete,
        ledger,
    };
    let written = serde_json::to_vec(&record)
        .map_err(anyhow::Error::from)
        .and_then(|bytes| native_route_job::write_new_atomic(&ledger_path, &bytes));
    let body = json!({
        "source": "ipc",
        "complete": complete,
        "failure": failure,
        "applied": summary,
        "locked": a.locked,
        "plan_tracks": a.plan.tracks.len(),
        "plan_vias": a.plan.vias.len(),
        "ledger_output": if written.is_ok() { json!(ledger_path) } else { Value::Null },
        "ledger_error": written.err().map(|e| format!("{e:#}")),
        "elapsed_ms": elapsed_ms,
        "next": "Refill zones, save, run_drc with an output file, then map_route_plan_drc.",
    });
    Ok(if complete {
        CallToolResult::json(&body)
    } else {
        CallToolResult::error(format!(
            "The plan was applied only in part; retry the chunks not applied. {body}"
        ))
    })
}

async fn handle_map_route_plan_drc(
    args: &Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let parsed = (|| -> Result<_, Refusal> {
        let plan: RoutePlan = read_json(&required_path(args, "routing_file")?, "routing_file")?;
        let ledger: LedgerFile = read_json(&required_path(args, "ledger")?, "ledger")?;
        let report: super::cli::DrcReport =
            read_json(&required_path(args, "drc_report")?, "drc_report")?;
        let listed = opt_count(args, "max_listed", LISTED_DEFAULT, 10_000)? as usize;
        Ok((plan, ledger, report, listed))
    })();
    let (plan, ledger, report, listed) = match parsed {
        Ok(v) => v,
        Err(r) => return Ok(r),
    };
    let map = route_native_apply::map_drc(&report, &ledger.ledger, &plan);
    let mut by_rule: BTreeMap<String, BTreeMap<&str, usize>> = BTreeMap::new();
    for f in &map.findings {
        let side = if f.touches_plan() {
            "touching_plan"
        } else {
            "not_touching_plan"
        };
        *by_rule
            .entry(format!("{}/{}", f.rule, f.severity))
            .or_default()
            .entry(side)
            .or_default() += 1;
    }
    let first: Vec<&route_native_apply::MappedFinding> = map
        .findings
        .iter()
        .filter(|f| f.touches_plan() && f.category == "violations")
        .take(listed)
        .collect();
    Ok(CallToolResult::json(&json!({
        "source": "saved_file",
        "ledger_complete": ledger.complete,
        "touching_plan": map.touching_plan,
        "not_touching_plan": map.not_touching_plan,
        "items_mapped": map.items_mapped,
        "items_not_in_plan": map.items_not_in_plan,
        "by_rule": by_rule,
        "nets_to_reroute": map.nets_to_reroute.len(),
        "nets": map.nets_to_reroute.iter().take(listed).collect::<Vec<_>>(),
        "missing_categories": map.missing_categories,
        "listed": first,
    })))
}

#[cfg(test)]
mod tests;
