//! Planning tools of the `native_routing` toolset that write a routing_file
//! for named copper: `plan_tap_vias` (a via and stub per named pad, reaching
//! a target layer) and `plan_trunk` (one net's wide trunk on one layer
//! through ordered waypoints).
//!
//! Both read the saved board and the project's rules, write nothing to the
//! board, refuse to replace an existing output file, and check what they
//! wrote with the native router's exact checker (`check_routed`, the kernel
//! `check_route_rules` runs) before reporting.

use crate::mcp::{error::ToolErrorKind, protocol::CallToolResult};
use crate::native_route_job::{self, guarded};
use crate::tools::board_inspect::{self, mm, pt};
use crate::tools::route_native::{
    base_spec, invalid, loaded, opt_bool, opt_strings, stage_failure, Refusal,
};
use crate::tools::{get_path, ToolContext};
use konnect_geom::{nm_to_mm, BBox, Coord, Point};
use konnect_route::api::{Generation, LayerId, NetId};
use konnect_route::check::{CheckReport, CheckedItem, ViolationKind};
use konnect_route::plan::{RoutePlan, RoutedCopper};
use konnect_route::trunk::{TrunkSpec, TrunkTuning, ViaStatus};
use konnect_route::RouteBoard;
use serde_json::{json, Value};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

/// Copper resistivity at 20 °C (IPC-2152's annealed copper), ohm metre.
const COPPER_RESISTIVITY_OHM_M: f64 = 1.724e-8;
/// Copper assumed when the stackup does not give the layer's thickness.
const DEFAULT_COPPER_MM: f64 = 0.035;

fn absolute(p: PathBuf) -> anyhow::Result<PathBuf> {
    Ok(if p.is_absolute() {
        p
    } else {
        std::env::current_dir()?.join(p)
    })
}

fn refuse_existing(output: &Path) -> Option<CallToolResult> {
    output.exists().then(|| {
        CallToolResult::error_kind(
            ToolErrorKind::Conflict {
                paths: vec![output.display().to_string()],
            },
            format!(
                "{} already exists; it is never replaced. Nothing was planned.",
                output.display()
            ),
        )
    })
}

fn write_plan(output: &Path, plan: &RoutePlan) -> Result<(), CallToolResult> {
    native_route_job::write_new_atomic(output, plan.to_routing_file_json().as_bytes()).map_err(
        |e| {
            if output.exists() {
                CallToolResult::error_kind(
                    ToolErrorKind::Conflict {
                        paths: vec![output.display().to_string()],
                    },
                    format!("{e:#}"),
                )
            } else {
                CallToolResult::error(format!("plan not written: {e:#}"))
            }
        },
    )
}

/// The `carve_zones` argument: exact names of zones a refill will carve
/// around the planned copper. Each must name a copper zone of the board;
/// with `layer`, each must also have copper there (carving it elsewhere
/// would do nothing, so it is taken for a mistake).
pub(crate) fn carve_zones_arg(
    board: &RouteBoard,
    args: &Value,
    layer: Option<LayerId>,
) -> Result<Vec<String>, Refusal> {
    let Some(names) = opt_strings(args, "carve_zones")? else {
        return Ok(Vec::new());
    };
    let quoted = |list: Vec<&String>| {
        list.iter()
            .map(|n| format!("'{n}'"))
            .collect::<Vec<_>>()
            .join(", ")
    };
    let on = |n: &String, l: Option<LayerId>| {
        board
            .zones
            .iter()
            .any(|z| z.zone_name.as_ref() == Some(n) && l.is_none_or(|l| z.layer == l))
    };
    let missing: Vec<&String> = names.iter().filter(|n| !on(n, None)).collect();
    if !missing.is_empty() {
        return Err(invalid(
            "carve_zones",
            &format!("no copper zone of the board is named {}", quoted(missing)),
        ));
    }
    if let Some(l) = layer {
        let off: Vec<&String> = names.iter().filter(|n| !on(n, Some(l))).collect();
        if !off.is_empty() {
            return Err(invalid(
                "carve_zones",
                &format!(
                    "{} has no copper on {}, so there is nothing to carve there",
                    quoted(off),
                    board.layers.name(l)
                ),
            ));
        }
    }
    Ok(names)
}

/// The routing_file of `copper`; empty when nothing was planned (the reply
/// then says why, and no file is written).
fn routing_of(board: &RouteBoard, copper: &RoutedCopper) -> Result<RoutePlan, CallToolResult> {
    if copper.tracks.is_empty() && copper.vias.is_empty() {
        return Ok(RoutePlan::default());
    }
    RoutePlan::from_routed(board, copper)
        .map_err(|e| CallToolResult::error(format!("plan not built: {e}")))
}

fn kind_name(k: ViolationKind) -> String {
    serde_json::to_value(k)
        .ok()
        .and_then(|v| v.as_str().map(String::from))
        .unwrap_or_default()
}

/// Counts by kind and the first `listed` violations, located.
fn check_json(
    board: &RouteBoard,
    report: &CheckReport,
    against: &str,
    listed: usize,
    label: impl Fn(&CheckedItem) -> Value,
) -> Value {
    let mut by_kind: BTreeMap<String, usize> = BTreeMap::new();
    for v in &report.violations {
        *by_kind.entry(kind_name(v.kind)).or_default() += 1;
    }
    json!({
        "checker": "check_routed",
        "against": against,
        "violations": report.violations.len(),
        "by_kind": by_kind,
        "listed": report.violations.iter().take(listed).map(|v| {
            let (x, y) = v.at.to_mm();
            json!({
                "kind": kind_name(v.kind), "x": x, "y": y,
                "layer": v.layer.map(|l| board.layers.name(l).to_string()),
                "required_mm": nm_to_mm(i64::from(v.required)),
                "a": label(&v.a), "b": v.b.as_ref().map(&label),
            })
        }).collect::<Vec<_>>(),
    })
}

// ─── plan_tap_vias ───────────────────────────────────────────────────────────

pub(crate) async fn handle_plan_tap_vias(
    args: &Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let args = args.clone();
    tokio::task::spawn_blocking(move || plan_tap_vias_blocking(&args)).await?
}

fn plan_tap_vias_blocking(args: &Value) -> anyhow::Result<CallToolResult> {
    let output = absolute(get_path(args, "output_plan")?)?;
    let spec = match base_spec(args, output.clone())? {
        Ok(s) => s,
        Err(r) => return Ok(r),
    };
    let names = match opt_strings(args, "pads") {
        Ok(Some(n)) => n,
        Ok(None) => return Ok(invalid("pads", "is required")),
        Err(r) => return Ok(r),
    };
    let Some(target_name) = args.get("target_layer").and_then(Value::as_str) else {
        return Ok(invalid("target_layer", "is required (a copper layer name)"));
    };
    let retry_failed = match opt_bool(args, "retry_failed", true) {
        Ok(b) => b,
        Err(r) => return Ok(r),
    };
    if let Some(r) = refuse_existing(&output) {
        return Ok(r);
    }
    let (board, rules) = match loaded(&spec) {
        Ok(v) => v,
        Err(r) => return Ok(r),
    };
    let Some(target) = board.layers.by_name(target_name.trim()) else {
        return Ok(invalid(
            "target_layer",
            &format!("'{target_name}' is not a copper layer of the board"),
        ));
    };
    let carve = match carve_zones_arg(&board, args, Some(target)) {
        Ok(c) => c,
        Err(r) => return Ok(r),
    };
    let indices = match board_inspect::named_pads(&board, &names, "pads") {
        Ok(v) => v,
        Err(r) => return Ok(r),
    };
    let pads: Vec<konnect_route::api::PadId> = indices
        .iter()
        .map(|&i| konnect_route::api::PadId(i as u32))
        .collect();
    if let Some(p) = pads.iter().find(|p| board.pads[p.index()].net.is_none()) {
        return Ok(invalid(
            "pads",
            &format!(
                "{} has no net; a tap joins a net",
                board_inspect::pad_label(&board, &board.pads[p.index()])
            ),
        ));
    }
    let settings = konnect_route::fanout::FanoutSettings {
        retry_failed,
        ..Default::default()
    };
    let t = std::time::Instant::now();
    let plan = match guarded("tap vias", || {
        konnect_route::fanout::tap::plan_tap_vias(&board, &rules, &pads, target, &carve, &settings)
    }) {
        Ok(p) => p,
        Err(e) => return Ok(stage_failure("plan_tap_vias", e)),
    };
    let copper = plan.routed(&board);
    let routing = match routing_of(&board, &copper) {
        Ok(r) => r,
        Err(r) => return Ok(r),
    };
    let report = konnect_route::check::check_routed(&board, &rules, &copper);
    let label =
        |p: konnect_route::api::PadId| board_inspect::pad_label(&board, &board.pads[p.index()]);
    let sites: Vec<Value> = plan
        .sites
        .iter()
        .map(|s| {
            let (x, y) = s.at.to_mm();
            json!({
                "pad": label(s.pad),
                "net": board.net(s.net).name,
                "tap": if s.via.is_some() { "via" } else { "through_hole_pad" },
                "x": x, "y": y,
                "via": s.via.map(|(d, dr)| json!({ "diameter_mm": mm(d), "drill_mm": mm(dr) })),
                "stub": s.stub.map(|(a, b, w)| json!({
                    "layer": board.layers.name(s.layer),
                    "from": pt(a), "to": pt(b), "width_mm": mm(w),
                    "length_mm": nm_to_mm((b - a).length_f64().round() as i64),
                })),
            })
        })
        .collect();
    let failed: Vec<Value> = plan
        .failed
        .iter()
        .map(|(pad, why)| {
            let detail = plan.details.iter().find(|d| d.pad == *pad);
            let mut top: Vec<(&String, &u32)> = detail
                .map(|d| d.rejections.iter().collect())
                .unwrap_or_default();
            top.sort_by(|a, b| b.1.cmp(a.1).then(a.0.cmp(b.0)));
            top.truncate(4);
            json!({
                "pad": label(*pad),
                "failure": why,
                "candidates": detail.map(|d| d.candidates),
                "top_rejections": top.into_iter().map(|(k, v)| json!({ "reason": k, "count": v })).collect::<Vec<_>>(),
                "blocked_by": detail.map(|d| d.blocked_by.iter().map(|b| label(*b)).collect::<Vec<_>>()).unwrap_or_default(),
            })
        })
        .collect();
    let mut waypoints: BTreeMap<String, Vec<Value>> = BTreeMap::new();
    for s in &plan.sites {
        waypoints
            .entry(board.net(s.net).name.clone())
            .or_default()
            .push(pt(s.at));
    }
    let written = routing.item_count() > 0;
    if written {
        if let Err(r) = write_plan(&output, &routing) {
            return Ok(r);
        }
    }
    let check = check_json(&board, &report, "saved board", 50, |item| match item {
        CheckedItem::Board { item } => json!(format!("{item:?}")),
        CheckedItem::RoutedTrack { index } => json!(format!("stub {index}")),
        CheckedItem::RoutedVia { index } => json!(format!("via {index}")),
        CheckedItem::Edge => json!("board edge"),
    });
    Ok(CallToolResult::json(&json!({
        "source": "saved_file",
        "board": spec.board,
        "target_layer": board.layers.name(target),
        "carve_zones": carve,
        "requested": names.len(),
        "sites": sites,
        "failed": failed,
        "waypoints_by_net": waypoints,
        "vias": routing.vias.len(),
        "stubs": routing.tracks.len(),
        "check": check,
        "elapsed_ms": t.elapsed().as_millis() as u64,
        "plan_written": written,
        "output_plan": if written { json!(output) } else { Value::Null },
        "next": "Review with render_board_layers_png (overlay_routing_file), apply with add_routing or apply_route_plan, then refill and run DRC. The site positions are the waypoints plan_trunk takes.",
    })))
}

// ─── plan_trunk ──────────────────────────────────────────────────────────────

pub(crate) async fn handle_plan_trunk(
    args: &Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let args = args.clone();
    tokio::task::spawn_blocking(move || plan_trunk_blocking(&args)).await?
}

fn length_arg(args: &Value, key: &str, min: f64, max: f64) -> Result<Option<Coord>, Refusal> {
    match args.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(v) => match v
            .as_f64()
            .filter(|x| x.is_finite() && *x >= min && *x <= max)
            .and_then(konnect_geom::mm_to_nm)
        {
            Some(nm) => Ok(Some(nm)),
            None => Err(invalid(
                key,
                &format!("must be a length from {min} to {max} mm"),
            )),
        },
    }
}

fn point_arg(v: &Value, field: &str) -> Result<Point, Refusal> {
    let get = |k: &str| v.get(k).and_then(Value::as_f64).filter(|x| x.is_finite());
    match (get("x"), get("y")) {
        (Some(x), Some(y)) => Point::from_mm(x, y)
            .ok_or_else(|| invalid(field, "lies outside the representable board range")),
        _ => Err(invalid(field, "needs finite x and y (mm)")),
    }
}

/// The copper thickness of `layer` from the board's stackup, mm.
fn copper_thickness(board_path: &Path, layer: &str) -> Option<f64> {
    let text = std::fs::read_to_string(board_path).ok()?;
    let tree = konnect_sexp::parse_sexp(&text).ok()?;
    let stack = tree.find("setup")?.find("stackup")?;
    stack
        .find_all("layer")
        .into_iter()
        .find(|l| l.get(1).and_then(konnect_sexp::SexpNode::as_str) == Some(layer))?
        .find_f64("thickness")
        .filter(|t| t.is_finite() && *t > 0.0)
}

fn read_avoid(board: &RouteBoard, paths: &[String]) -> Result<RoutedCopper, Refusal> {
    let mut all = RoutedCopper::default();
    for (i, p) in paths.iter().enumerate() {
        let field = format!("avoid_routing_files[{i}]");
        let path = PathBuf::from(p);
        crate::tools::route_native::file_must_exist(&path, &field)?;
        let plan: RoutePlan = std::fs::read_to_string(&path)
            .map_err(|e| e.to_string())
            .and_then(|t| serde_json::from_str(&t).map_err(|e| e.to_string()))
            .map_err(|e| invalid(&field, &format!("is not a routing_file: {e}")))?;
        let copper = plan
            .to_routed(board, Generation(0))
            .map_err(|e| invalid(&field, &e.to_string()))?;
        all.tracks.extend(copper.tracks);
        all.vias.extend(copper.vias);
    }
    Ok(all)
}

struct TrunkArgs {
    spec: TrunkSpec,
    tuning: TrunkTuning,
    avoid_files: Vec<String>,
}

fn trunk_args(
    board: &RouteBoard,
    rules: &dyn konnect_route::RuleTable,
    args: &Value,
) -> Result<TrunkArgs, Refusal> {
    let Some(net_name) = args.get("net").and_then(Value::as_str) else {
        return Err(invalid("net", "is required"));
    };
    let net: NetId = board
        .net_by_name(net_name)
        .ok_or_else(|| invalid("net", &format!("'{net_name}' is not a net of the board")))?;
    let Some(layer_name) = args.get("layer").and_then(Value::as_str) else {
        return Err(invalid("layer", "is required (a copper layer name)"));
    };
    let layer = board.layers.by_name(layer_name.trim()).ok_or_else(|| {
        invalid(
            "layer",
            &format!("'{layer_name}' is not a copper layer of the board"),
        )
    })?;
    let carve_zones = carve_zones_arg(board, args, Some(layer))?;
    let Some(points) = args.get("waypoints").and_then(Value::as_array) else {
        return Err(invalid(
            "waypoints",
            "is required: an array of {x, y} in mm",
        ));
    };
    if points.len() < 2 {
        return Err(invalid("waypoints", "needs at least two points"));
    }
    let waypoints = points
        .iter()
        .enumerate()
        .map(|(i, p)| point_arg(p, &format!("waypoints[{i}]")))
        .collect::<Result<Vec<_>, _>>()?;
    let preferred = length_arg(args, "preferred_width_mm", 0.01, 20.0)?
        .ok_or_else(|| invalid("preferred_width_mm", "is required"))?;
    let floor = rules
        .track_width(
            &konnect_route::rules::RuleSubject::new(konnect_route::api::ItemKind::Track, Some(net)),
            layer,
        )
        .min;
    let min = length_arg(args, "min_width_mm", 0.01, 20.0)?.unwrap_or(floor);
    if min < floor {
        return Err(invalid(
            "min_width_mm",
            &format!(
                "is below the rules' minimum track width for {net_name} on {layer_name} ({} mm)",
                mm(floor)
            ),
        ));
    }
    if min > preferred {
        return Err(invalid("min_width_mm", "is wider than preferred_width_mm"));
    }
    let legs = waypoints.len() - 1;
    let mut leg_widths: Vec<Option<Coord>> = vec![None; legs];
    if let Some(list) = args.get("leg_widths").filter(|v| !v.is_null()) {
        let Some(list) = list.as_array() else {
            return Err(invalid("leg_widths", "must be an array of {leg, width_mm}"));
        };
        for (i, item) in list.iter().enumerate() {
            let field = format!("leg_widths[{i}]");
            let leg = item
                .get("leg")
                .and_then(Value::as_u64)
                .filter(|l| (*l as usize) < legs)
                .ok_or_else(|| {
                    invalid(
                        &field,
                        &format!("leg must be an integer from 0 to {}", legs - 1),
                    )
                })?;
            let w = item
                .get("width_mm")
                .and_then(Value::as_f64)
                .filter(|w| w.is_finite() && *w > 0.0 && *w <= 20.0)
                .and_then(konnect_geom::mm_to_nm)
                .ok_or_else(|| invalid(&field, "width_mm must be a width from 0 to 20 mm"))?;
            if w < min {
                return Err(invalid(
                    &field,
                    "width_mm is narrower than the minimum width",
                ));
            }
            leg_widths[leg as usize] = Some(w);
        }
    }
    let neck = length_arg(args, "neck_radius_mm", 0.0, 50.0)?.unwrap_or(2_000_000);
    let grid = length_arg(args, "grid_mm", 0.02, 1.0)?.unwrap_or(100_000);
    let mut keepouts: Vec<BBox> = Vec::new();
    if let Some(list) = args.get("keepouts").filter(|v| !v.is_null()) {
        let Some(list) = list.as_array() else {
            return Err(invalid("keepouts", "must be an array of boxes"));
        };
        for (i, b) in list.iter().enumerate() {
            keepouts.push(board_inspect::box_arg(b, &format!("keepouts[{i}]"))?);
        }
    }
    let avoid_files = opt_strings(args, "avoid_routing_files")?.unwrap_or_default();
    let avoid = read_avoid(board, &avoid_files)?;
    let waypoint_vias = opt_bool(args, "waypoint_vias", true)?;
    Ok(TrunkArgs {
        spec: TrunkSpec {
            net,
            layer,
            waypoints,
            preferred_width: preferred,
            min_width: min,
            leg_widths,
            neck_radius: neck,
            keepouts,
            avoid,
            grid,
            waypoint_vias,
            carve_zones,
        },
        tuning: TrunkTuning::default(),
        avoid_files,
    })
}

fn plan_trunk_blocking(args: &Value) -> anyhow::Result<CallToolResult> {
    let output = absolute(get_path(args, "output_plan")?)?;
    let spec = match base_spec(args, output.clone())? {
        Ok(s) => s,
        Err(r) => return Ok(r),
    };
    if let Some(r) = refuse_existing(&output) {
        return Ok(r);
    }
    let (board, rules) = match loaded(&spec) {
        Ok(v) => v,
        Err(r) => return Ok(r),
    };
    let a = match trunk_args(&board, &rules, args) {
        Ok(a) => a,
        Err(r) => return Ok(r),
    };
    let t = std::time::Instant::now();
    let mut plan = match guarded("trunk", || {
        konnect_route::trunk::plan_trunk(&board, &rules, &a.spec, &a.tuning)
    }) {
        Ok(p) => p,
        Err(e) => return Ok(stage_failure("plan_trunk", e)),
    };
    let (net, layer) = (a.spec.net, a.spec.layer);

    // The exact check, against the board with the avoided copper in place:
    // a piece it still flags is narrowed to the minimum once; a leg that
    // still fails is reported and left out of the file.
    let mut checked_board = board.clone();
    a.spec.avoid.freeze_into(&mut checked_board);
    let locate = |plan: &konnect_route::trunk::TrunkPlan| -> Vec<(usize, usize)> {
        plan.legs
            .iter()
            .filter(|l| l.routed())
            .flat_map(|l| (0..l.pieces.len()).map(move |p| (l.index, p)))
            .collect()
    };
    for round in 0..2 {
        let copper = plan.copper(&board, net, layer);
        let report = konnect_route::check::check_routed(&checked_board, &rules, &copper);
        if report.violations.is_empty() {
            break;
        }
        let owners = locate(&plan);
        let mut flagged: BTreeMap<(usize, usize), String> = BTreeMap::new();
        for v in &report.violations {
            for side in std::iter::once(&v.a).chain(v.b.iter()) {
                if let CheckedItem::RoutedTrack { index } = side {
                    if let Some(&(leg, piece)) = owners.get(*index as usize) {
                        flagged
                            .entry((leg, piece))
                            .or_insert_with(|| kind_name(v.kind));
                    }
                }
            }
        }
        for ((leg, piece), why) in flagged {
            let l = &mut plan.legs[leg];
            if round == 0 && l.pieces[piece].width > a.spec.min_width {
                l.pieces[piece].width = a.spec.min_width;
            } else {
                l.violations.push((piece, why));
            }
        }
    }
    let copper = plan.copper(&board, net, layer);
    let report = konnect_route::check::check_routed(&checked_board, &rules, &copper);
    let routing = match routing_of(&board, &copper) {
        Ok(r) => r,
        Err(r) => return Ok(r),
    };

    let layer_name = board.layers.name(layer).to_string();
    let (thickness, thickness_source) = match copper_thickness(&spec.board, &layer_name) {
        Some(t) => (t, "board stackup"),
        None => (
            DEFAULT_COPPER_MM,
            "assumed 35 um (1 oz): the stackup does not give this layer",
        ),
    };
    // ohm = rho [ohm m] * 1e3 [mm/m] * L [mm] / (w [mm] * t [mm])
    let resistance_mohm = |pieces: &[konnect_geom::Segment]| -> f64 {
        pieces
            .iter()
            .map(|s| {
                let l = (s.b - s.a).length_f64() / 1e6;
                COPPER_RESISTIVITY_OHM_M * 1e3 * l / (mm(s.width) * thickness) * 1e3
            })
            .sum()
    };
    let round3 = |v: f64| (v * 1000.0).round() / 1000.0;
    let mut total_len = 0.0;
    let mut total_r = 0.0;
    let legs: Vec<Value> = plan
        .legs
        .iter()
        .map(|l| {
            let status = if l.failure.is_some() {
                "failed"
            } else if !l.violations.is_empty() {
                "violations"
            } else {
                "routed"
            };
            let len = l.length() / 1e6;
            let r = resistance_mohm(&l.pieces);
            if l.routed() {
                total_len += len;
                total_r += r;
            }
            json!({
                "leg": l.index,
                "from": pt(l.from), "to": pt(l.to),
                "status": status,
                "failure": l.failure,
                "violations": l.violations.iter().map(|(p, why)| json!({ "piece": p, "kind": why })).collect::<Vec<_>>(),
                "length_mm": round3(len),
                "bends": l.bends,
                "preferred_width_mm": mm(l.preferred_width),
                "min_width_used_mm": l.pieces.iter().map(|p| mm(p.width)).fold(None, |m: Option<f64>, w| Some(m.map_or(w, |m| m.min(w)))),
                "max_width_used_mm": l.pieces.iter().map(|p| mm(p.width)).fold(None, |m: Option<f64>, w| Some(m.map_or(w, |m| m.max(w)))),
                "resistance_mohm": round3(r),
                "width_profile": l.pieces.iter().map(|p| json!({
                    "from": pt(p.a), "to": pt(p.b), "width_mm": mm(p.width),
                    "length_mm": round3((p.b - p.a).length_f64() / 1e6),
                })).collect::<Vec<_>>(),
                "grid_mm": mm(l.grid),
                "search_states": l.expansions,
            })
        })
        .collect();
    let vias: Vec<Value> = plan
        .vias
        .iter()
        .map(|v| {
            let (x, y) = v.at.to_mm();
            let mut o =
                json!({ "x": x, "y": y, "diameter_mm": mm(v.diameter), "drill_mm": mm(v.drill) });
            match &v.status {
                ViaStatus::Planned => o["status"] = json!("planned"),
                ViaStatus::Existing => o["status"] = json!("existing_on_board"),
                ViaStatus::NotRequested => o["status"] = json!("not_requested"),
                ViaStatus::Illegal { reason } => {
                    o["status"] = json!("left_out");
                    o["reason"] = json!(reason);
                }
            }
            o
        })
        .collect();
    // How much of each carved zone the routed legs cut through.
    let carved: Vec<Value> = a
        .spec
        .carve_zones
        .iter()
        .map(|z| {
            let by_leg: Vec<f64> = plan
                .legs
                .iter()
                .map(|l| {
                    if l.routed() {
                        konnect_route::trunk::length_over_zone(&board, layer, z, &l.pieces) / 1e6
                    } else {
                        0.0
                    }
                })
                .collect();
            json!({
                "zone": z,
                "length_mm": round3(by_leg.iter().sum()),
                "by_leg_mm": by_leg.into_iter().map(round3).collect::<Vec<_>>(),
            })
        })
        .collect();
    let routed = plan.legs.iter().filter(|l| l.routed()).count();
    let written = routing.item_count() > 0;
    if written {
        if let Err(r) = write_plan(&output, &routing) {
            return Ok(r);
        }
    }
    let n_tracks = copper.tracks.len();
    let owners = locate(&plan);
    let check = check_json(
        &checked_board,
        &report,
        if a.avoid_files.is_empty() {
            "saved board"
        } else {
            "saved board plus the avoided routing files"
        },
        50,
        |item| match item {
            CheckedItem::RoutedTrack { index } => match owners.get(*index as usize) {
                Some((leg, piece)) => json!(format!("leg {leg} piece {piece}")),
                None => json!(format!("track {index}")),
            },
            CheckedItem::RoutedVia { index } => json!(format!("waypoint via {}", index)),
            CheckedItem::Board { item } => json!(format!("{item:?}")),
            CheckedItem::Edge => json!("board edge"),
        },
    );
    let complete = routed == plan.legs.len() && report.violations.is_empty();
    Ok(CallToolResult::json(&json!({
        "source": "saved_file",
        "board": spec.board,
        "net": board.net(net).name,
        "layer": layer_name,
        "complete": complete,
        "legs_routed": routed,
        "legs_failed": plan.legs.len() - routed,
        "legs": legs,
        "vias": vias,
        "carve_zones": carved,
        "totals": {
            "length_mm": round3(total_len),
            "resistance_mohm": round3(total_r),
            "tracks": n_tracks,
            "vias": routing.vias.len(),
        },
        "copper": {
            "thickness_mm": thickness,
            "thickness_source": thickness_source,
            "resistivity_ohm_m": COPPER_RESISTIVITY_OHM_M,
            "note": "DC resistance at 20 C of the routed legs only (no vias, no temperature rise).",
        },
        "check": check,
        "elapsed_ms": t.elapsed().as_millis() as u64,
        "plan_written": written,
        "output_plan": if written { json!(output) } else { Value::Null },
        "next": "Review with render_board_layers_png (overlay_routing_file); check with check_route_rules; legs not routed are left out of the file.",
    })))
}
