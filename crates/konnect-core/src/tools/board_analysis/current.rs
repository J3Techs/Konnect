//! K2 `check_current_capacity`: solve each net's DC current path, then hold
//! every conductor it uses — tracks, vias, sets of parallel vias and pour
//! necks — against its IPC-2152 capacity.

use super::common::{self, marker, merged, r4, Loaded, Outputs};
use super::geom::{dist, pmm, unfracture, P};
use super::ipc::{barrel_area_mm2, ipc2152_current, ipc2152_width_mm, onderdonk_fusing_current};
use super::necks::find_necks_in;
use super::network::{pads_named, Kind, Network, Options, Solution};
use crate::mcp::protocol::CallToolResult;
use crate::tools::board_inspect::{self, pad_label};
use crate::tools::route_native::{invalid, opt_count, Refusal};
use konnect_route::api::{LayerId, NetId};
use serde_json::{json, Value};
use std::collections::BTreeMap;

pub(crate) fn net_case_schema() -> Value {
    json!({
        "type": "object",
        "properties": {
            "net": { "type": "string", "minLength": 1 },
            "current_a": { "type": "number", "exclusiveMinimum": 0, "description": "Continuous current, A" },
            "peak_a": { "type": "number", "exclusiveMinimum": 0, "description": "Peak current, A (held against the same IPC-2152 capacity, and against Onderdonk fusing when pulse_ms is given)" },
            "pulse_ms": { "type": "number", "exclusiveMinimum": 0, "description": "Peak pulse length, ms" },
            "sources": { "description": "Pads the current enters by (REF.PAD), shared equally, or \"all\" (every pad not a sink)", "anyOf": [ { "type": "array", "items": { "type": "string" }, "minItems": 1 }, { "const": "all" } ] },
            "sinks": { "description": "Pads the current leaves by (REF.PAD), held at one potential, or \"all\" (every pad not a source)", "anyOf": [ { "type": "array", "items": { "type": "string" }, "minItems": 1 }, { "const": "all" } ] },
            "sink_layers": { "type": "array", "items": { "type": "string" }, "minItems": 1, "description": "Also take every fill cell of the net on these layers as a sink (a plane the current spreads into)" },
            "area": { "type": "object", "properties": { "x1": { "type": "number" }, "y1": { "type": "number" }, "x2": { "type": "number" }, "y2": { "type": "number" } }, "required": ["x1", "y1", "x2", "y2"], "additionalProperties": false, "description": "Model only the net's copper meeting this box, mm" }
        },
        "required": ["net", "current_a", "sources"],
        "additionalProperties": false
    })
}

pub(crate) fn schema() -> Value {
    common::schema(
        json!({
            "nets": { "type": "array", "minItems": 1, "items": net_case_schema(), "description": "Nets with their currents and terminals" },
            "nets_file": { "type": "string", "description": "A JSON file of more net cases (an array, or {\"nets\": [...]})" },
            "rise_c": { "type": "number", "minimum": 1, "maximum": 100, "default": 10, "description": "Allowed temperature rise ΔT, °C" },
            "ambient_c": { "type": "number", "minimum": -55, "maximum": 150, "default": 25 },
            "plating_um": { "type": "number", "exclusiveMinimum": 0, "maximum": 100, "default": 20, "description": "Via barrel plating thickness, µm" },
            "correction_factor": { "type": "number", "exclusiveMinimum": 0, "maximum": 10, "default": 1, "description": "IPC-2152 modifier product (copper thickness, board thickness, plane); the allowed rise becomes ΔT / factor" },
            "external_factor": { "type": "number", "exclusiveMinimum": 0, "maximum": 10, "default": 1, "description": "Extra IPC-2152 modifier for outer layers" },
            "internal_factor": { "type": "number", "exclusiveMinimum": 0, "maximum": 10, "default": 1, "description": "Extra IPC-2152 modifier for inner layers" },
            "resolution_mm": { "type": "number", "minimum": 0.005, "maximum": 2, "default": 0.1, "description": "Fill mesh step" },
            "max_cells": { "type": "integer", "minimum": 1000, "maximum": 50000000, "default": 4000000 },
            "via_set_mm": { "type": "number", "exclusiveMinimum": 0, "maximum": 50, "default": 2.0, "description": "Vias carrying current within this distance of each other form one parallel set" },
            "auto_area": { "type": "boolean", "default": true, "description": "Without an area, model only the copper within auto_area_margin_mm of the terminals' box (the whole net when a terminal is cut off); false models the whole net" },
            "auto_area_margin_mm": { "type": "number", "minimum": 0, "maximum": 1000, "default": 20 },
            "neck_search_mm": { "type": "number", "exclusiveMinimum": 0, "maximum": 20, "default": 3.0, "description": "Look for pour necks up to this width (and up to the width the full current needs)" }
        }),
        &["board"],
    )
}

pub(crate) struct Terminals {
    pub sources: Vec<usize>,
    pub sinks: Vec<usize>,
    pub sink_layers: Vec<LayerId>,
}

/// `"all"`, or a list of pad names.
pub(crate) fn pad_list(
    board: &konnect_route::board::RouteBoard,
    net: NetId,
    v: Option<&Value>,
    field: &str,
) -> Result<Option<Vec<usize>>, Refusal> {
    match v {
        None => Ok(None),
        Some(Value::String(s)) if s == "all" => Ok(Some(Vec::new())),
        Some(Value::Array(a)) => {
            let names: Vec<String> = a
                .iter()
                .map(|x| x.as_str().map(str::to_string))
                .collect::<Option<_>>()
                .ok_or_else(|| invalid(field, "must hold REF.PAD strings"))?;
            let pads = pads_named(board, &names, field)?;
            for &p in &pads {
                if board.pads[p].net != Some(net) {
                    return Err(invalid(
                        field,
                        &format!(
                            "{} is on {}, not {}",
                            pad_label(board, &board.pads[p]),
                            board.pads[p]
                                .net
                                .map(|n| board.net(n).name.as_str())
                                .unwrap_or("no net"),
                            board.net(net).name
                        ),
                    ));
                }
            }
            Ok(Some(pads))
        }
        Some(_) => Err(invalid(field, "must be \"all\" or an array of REF.PAD")),
    }
}

/// The box of `pads`' centres grown by `margin`, mm.
pub(crate) fn terminal_box(
    board: &konnect_route::board::RouteBoard,
    pads: impl Iterator<Item = usize>,
    margin: f64,
) -> (P, P) {
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for p in pads {
        let c = pmm(board.pads[p].center);
        lo = [lo[0].min(c[0]), lo[1].min(c[1])];
        hi = [hi[0].max(c[0]), hi[1].max(c[1])];
    }
    (
        [lo[0] - margin, lo[1] - margin],
        [hi[0] + margin, hi[1] + margin],
    )
}

pub(crate) fn window(case: &Value, field: &str) -> Result<Option<(P, P)>, Refusal> {
    match case.get("area") {
        None | Some(Value::Null) => Ok(None),
        Some(a) => {
            let b = board_inspect::box_arg(a, field)?;
            Ok(Some((pmm(b.min), pmm(b.max))))
        }
    }
}

fn cases(args: &Value) -> Result<Vec<Value>, Refusal> {
    let mut out: Vec<Value> = args
        .get("nets")
        .and_then(Value::as_array)
        .cloned()
        .unwrap_or_default();
    if let Some(v) = common::json_file(args, "nets_file")? {
        match v {
            Value::Array(a) => out.extend(a),
            Value::Object(mut o) => match o.remove("nets") {
                Some(Value::Array(a)) => out.extend(a),
                _ => {
                    return Err(invalid(
                        "nets_file",
                        "must hold an array or {\"nets\": [...]}",
                    ))
                }
            },
            _ => return Err(invalid("nets_file", "must hold an array of net cases")),
        }
    }
    if out.is_empty() {
        return Err(invalid(
            "nets",
            "give at least one net case (nets and/or nets_file)",
        ));
    }
    Ok(out)
}

/// The allowed rise on `layer` after the IPC-2152 modifiers.
fn rise_on(rise: f64, correction: f64, ext: f64, int: f64, outer: bool) -> f64 {
    rise / (correction * if outer { ext } else { int })
}

#[derive(Debug, Clone)]
struct Finding {
    kind: &'static str,
    at: P,
    layer: Option<String>,
    current: f64,
    capacity: f64,
    width_mm: f64,
    required_width_mm: Option<f64>,
    detail: Value,
}

impl Finding {
    fn margin(&self) -> f64 {
        if self.current > 0.0 {
            self.capacity / self.current
        } else {
            f64::INFINITY
        }
    }
    fn json(&self, peak_scale: Option<f64>) -> Value {
        let mut v = json!({
            "kind": self.kind,
            "at": [r4(self.at[0]), r4(self.at[1])],
            "layer": self.layer,
            "current_a": r4(self.current),
            "capacity_a": r4(self.capacity),
            "margin": r4(self.margin()),
            "width_mm": r4(self.width_mm),
            "required_width_mm": self.required_width_mm.map(r4),
            "ok": self.margin() >= 1.0,
        });
        if let Some(s) = peak_scale {
            v["peak_current_a"] = json!(r4(self.current * s));
            v["peak_margin"] = json!(r4(self.margin() / s));
        }
        if let Value::Object(d) = &self.detail {
            for (k, x) in d {
                v[k] = x.clone();
            }
        }
        v
    }
}

pub(crate) fn check_current_capacity(
    l: &Loaded,
    args: &Value,
    out: &Outputs,
) -> Result<CallToolResult, Refusal> {
    let board = l.board();
    let list = cases(args)?;
    let rise = common::opt_f64(args, "rise_c", 10.0, 1.0, 100.0)?;
    let ambient = common::opt_f64(args, "ambient_c", 25.0, -55.0, 150.0)?;
    let plating = common::opt_f64(args, "plating_um", 20.0, 0.1, 100.0)? / 1000.0;
    let corr = common::opt_f64(args, "correction_factor", 1.0, 1e-3, 10.0)?;
    let ext_f = common::opt_f64(args, "external_factor", 1.0, 1e-3, 10.0)?;
    let int_f = common::opt_f64(args, "internal_factor", 1.0, 1e-3, 10.0)?;
    let h = common::opt_f64(args, "resolution_mm", 0.1, 0.005, 2.0)?;
    let max_cells = opt_count(args, "max_cells", 4_000_000, 50_000_000)? as usize;
    let set_mm = common::opt_f64(args, "via_set_mm", 2.0, 1e-3, 50.0)?;
    let neck_max = common::opt_f64(args, "neck_search_mm", 3.0, 1e-3, 20.0)?;
    let auto_area = crate::tools::route_native::opt_bool(args, "auto_area", true)?;
    let auto_margin = common::opt_f64(args, "auto_area_margin_mm", 20.0, 0.0, 1000.0)?;
    let max_items = opt_count(args, "max_items", 50, 100_000)? as usize;
    let t0 = std::time::Instant::now();

    let mut results = Vec::new();
    let mut summaries = Vec::new();
    let mut markers = Vec::new();
    for (ci, case) in list.iter().enumerate() {
        let f = |k: &str| format!("nets[{ci}].{k}");
        let net_name = case
            .get("net")
            .and_then(Value::as_str)
            .ok_or_else(|| invalid(&f("net"), "is required"))?;
        let net = common::net_named(board, net_name, &f("net"))?;
        let current = case
            .get("current_a")
            .and_then(Value::as_f64)
            .filter(|c| c.is_finite() && *c > 0.0)
            .ok_or_else(|| invalid(&f("current_a"), "must be a positive number"))?;
        let peak = case
            .get("peak_a")
            .and_then(Value::as_f64)
            .filter(|c| *c > 0.0);
        let pulse_ms = case
            .get("pulse_ms")
            .and_then(Value::as_f64)
            .filter(|c| *c > 0.0);
        let src = pad_list(board, net, case.get("sources"), &f("sources"))?
            .ok_or_else(|| invalid(&f("sources"), "is required"))?;
        let snk = pad_list(board, net, case.get("sinks"), &f("sinks"))?;
        let sink_layers: Vec<LayerId> =
            match crate::tools::route_native::opt_strings(case, "sink_layers")? {
                Some(names) => names
                    .iter()
                    .map(|n| {
                        board.layers.by_name(n).ok_or_else(|| {
                            invalid(&f("sink_layers"), &format!("'{n}' is not a copper layer"))
                        })
                    })
                    .collect::<Result<_, _>>()?,
                None => Vec::new(),
            };
        let net_pads: Vec<usize> = (0..board.pads.len())
            .filter(|&i| board.pads[i].net == Some(net))
            .collect();
        let (sources, sinks) = match (src.is_empty(), snk) {
            (true, Some(s)) if s.is_empty() => {
                return Err(invalid(
                    &f("sources"),
                    "sources and sinks cannot both be \"all\"",
                ))
            }
            (true, Some(s)) => (
                net_pads
                    .iter()
                    .copied()
                    .filter(|p| !s.contains(p))
                    .collect(),
                s,
            ),
            (true, None) => {
                return Err(invalid(&f("sinks"), "with sources \"all\", name the sinks"))
            }
            (false, Some(s)) if s.is_empty() => {
                let s2 = net_pads
                    .iter()
                    .copied()
                    .filter(|p| !src.contains(p))
                    .collect();
                (src, s2)
            }
            (false, Some(s)) => (src, s),
            (false, None) => (src, Vec::new()),
        };
        if sinks.is_empty() && sink_layers.is_empty() {
            return Err(invalid(&f("sinks"), "name sink pads or sink_layers"));
        }
        let terms = Terminals {
            sources,
            sinks,
            sink_layers,
        };
        let given = window(case, &f("area"))?;
        let auto = if given.is_none() && auto_area {
            Some(terminal_box(
                board,
                terms.sources.iter().chain(&terms.sinks).copied(),
                auto_margin,
            ))
        } else {
            None
        };
        let attempt = |win: Option<(P, P)>| -> Result<(Value, Value, Vec<Value>, u64), String> {
            let o = Options {
                h,
                temp_c: ambient + rise,
                plating_mm: plating,
                max_cells,
                window: win,
            };
            let tb = std::time::Instant::now();
            let net_model = Network::build(l, net, &o)?;
            let build_ms = tb.elapsed().as_millis() as u64;
            let (res, summary, mk) = analyse(
                l, &net_model, &terms, current, peak, pulse_ms, rise, ambient, corr, ext_f, int_f,
                plating, set_mm, neck_max,
            )?;
            Ok((res, summary, mk, build_ms))
        };
        let (win_used, outcome) = match auto {
            Some(w) => match attempt(Some(w)) {
                Ok(r) => (Some(w), Ok(r)),
                // A terminal the box cut off: model the whole net instead.
                Err(_) => (None, attempt(None)),
            },
            None => (given, attempt(given)),
        };
        let (mut res, mut summary, mk, build_ms) = outcome.map_err(|e| invalid(&f("net"), &e))?;
        let area_note = json!({
            "box": win_used.map(|(a, b)| json!({ "x1": r4(a[0]), "y1": r4(a[1]), "x2": r4(b[0]), "y2": r4(b[1]) })),
            "source": if given.is_some() { "area" } else if win_used.is_some() { "auto: the terminals' box grown by auto_area_margin_mm" } else { "whole net" },
        });
        for v in [&mut res, &mut summary] {
            v["solve"]["build_ms"] = json!(build_ms);
            v["modelled_area"] = area_note.clone();
        }
        markers.extend(mk);
        results.push(res);
        summaries.push(summary);
    }
    let methods = json!({
        "capacity": "IPC-2152 Figure 5-2 (generic chart) least-squares fit: A[mil²] = (117.555·ΔT^-0.913 + 1.15)·I^(0.84·ΔT^-0.108 + 1.159); conservative for internal and external conductors; modifiers enter as ΔT/(correction·layer factor)",
        "vias": "each via barrel as a conductor of its plated annulus (drill, plating) under the same IPC-2152 fit; a set of parallel vias carries the sum of its members' capacities",
        "necks": "pour necks are the exact narrowest chords of the fill polygons (up to neck_search_mm, or the width the full current needs); the current through each is the solved flux across the chord",
        "path": "DC solve of the net's tracks (ρL/wt), via barrels and fills meshed at resolution_mm (sheet resistance ρ/t), at ambient + ΔT; sources share the current, sinks are held at one potential",
        "fusing": "Onderdonk: I = A[cmil]·sqrt(log10(1 + (1083 − Ta)/(234 + Ta)) / (33·t)) for peak pulses of pulse_ms",
        "copper": "thickness per layer from the board stackup (see stackup); resistivity 1.7241e-8 Ω·m at 20 °C, +0.393 %/°C",
    });
    let full = merged(
        common::provenance(l),
        json!({
            "rise_c": rise, "ambient_c": ambient, "plating_um": plating * 1000.0,
            "correction_factor": corr, "external_factor": ext_f, "internal_factor": int_f,
            "resolution_mm": h,
            "stackup": l.stack.json(board),
            "nets": results,
            "methods": methods,
        }),
    );
    let files = out.write(&full, &markers)?;
    let trimmed: Vec<Value> = summaries
        .into_iter()
        .map(|mut s| {
            if let Some(a) = s.get_mut("below_requirement").and_then(Value::as_array_mut) {
                a.truncate(max_items);
            }
            s
        })
        .collect();
    Ok(CallToolResult::json(&merged(
        common::provenance(l),
        json!({
            "rise_c": rise, "ambient_c": ambient, "plating_um": plating * 1000.0,
            "nets": trimmed,
            "files": files,
            "methods": full["methods"],
            "elapsed_ms": t0.elapsed().as_millis() as u64,
        }),
    )))
}

#[allow(clippy::too_many_arguments)]
fn analyse(
    l: &Loaded,
    nm: &Network,
    t: &Terminals,
    current: f64,
    peak: Option<f64>,
    pulse_ms: Option<f64>,
    rise: f64,
    ambient: f64,
    corr: f64,
    ext_f: f64,
    int_f: f64,
    plating: f64,
    set_mm: f64,
    neck_max: f64,
) -> Result<(Value, Value, Vec<Value>), String> {
    let board = l.board();
    let stack = &l.stack;
    let name = board.net(nm.net).name.clone();
    let mut fixed: Vec<(u32, f64)> = Vec::new();
    for &p in &t.sinks {
        let n = nm.pad(p).ok_or_else(|| {
            format!(
                "sink {} is outside the modelled area",
                pad_label(board, &board.pads[p])
            )
        })?;
        fixed.push((n, 0.0));
    }
    for &layer in &t.sink_layers {
        let cells = nm.cells_of(layer);
        if cells.is_empty() {
            return Err(format!(
                "{name} has no fill on sink layer {}",
                board.layers.name(layer)
            ));
        }
        fixed.extend(cells.into_iter().map(|c| (c, 0.0)));
    }
    let share = current / t.sources.len().max(1) as f64;
    let mut inject = Vec::new();
    for &p in &t.sources {
        let n = nm.pad(p).ok_or_else(|| {
            format!(
                "source {} is outside the modelled area",
                pad_label(board, &board.pads[p])
            )
        })?;
        inject.push((n, share));
    }
    if inject.is_empty() {
        return Err("no source pad".to_string());
    }
    let ts = std::time::Instant::now();
    let sol: Solution = nm.solve(&fixed, &inject)?;
    let solve_ms = ts.elapsed().as_millis() as u64;
    let tp = std::time::Instant::now();
    let floor = current * 1e-3;
    let rise_l = |layer: LayerId| rise_on(rise, corr, ext_f, int_f, stack.is_outer(layer));
    let mut findings: Vec<Finding> = Vec::new();

    // Tracks and arcs: the largest current along each.
    let mut per_wire: BTreeMap<(u8, u32), (f64, usize)> = BTreeMap::new();
    for (ei, e) in nm.elems.iter().enumerate() {
        let key = match e.kind {
            Kind::Track { track } => (0u8, track),
            Kind::Arc { arc } => (1u8, arc),
            _ => continue,
        };
        let i = nm.current(&sol, e).abs();
        let ent = per_wire.entry(key).or_insert((0.0, ei));
        if i > ent.0 {
            *ent = (i, ei);
        }
    }
    for (&(k, id), &(i, ei)) in &per_wire {
        if i < floor {
            continue;
        }
        let e = &nm.elems[ei];
        let layer = if k == 0 {
            board.tracks[id as usize].layer
        } else {
            board.arcs[id as usize].layer
        };
        let th = stack.t(layer);
        let rl = rise_l(layer);
        findings.push(Finding {
            kind: if k == 0 { "track" } else { "arc" },
            at: [(e.p[0] + e.q[0]) / 2.0, (e.p[1] + e.q[1]) / 2.0],
            layer: Some(board.layers.name(layer).to_string()),
            current: i,
            capacity: ipc2152_current(e.width * th, rl),
            width_mm: e.width,
            required_width_mm: Some(ipc2152_width_mm(i, rl, th)),
            detail: json!({
                "uuid": if k == 0 { board.tracks[id as usize].uuid.clone() } else { board.arcs[id as usize].uuid.clone() },
                "copper_um": r4(th * 1000.0),
                "fusing_a": pulse_ms.map(|ms| r4(onderdonk_fusing_current(e.width * th, ambient, ms / 1000.0))),
            }),
        });
    }

    // Vias: the largest barrel current of each.
    let mut per_via: BTreeMap<u32, f64> = BTreeMap::new();
    for e in &nm.elems {
        if let Kind::Barrel { via, .. } = e.kind {
            let i = nm.current(&sol, e).abs();
            let ent = per_via.entry(via).or_insert(0.0);
            if i > *ent {
                *ent = i;
            }
        }
    }
    struct ViaRow {
        via: u32,
        at: P,
        i: f64,
        cap: f64,
        drill: f64,
        area: f64,
    }
    let mut vias: Vec<ViaRow> = Vec::new();
    for (&vi, &i) in &per_via {
        if i < floor {
            continue;
        }
        let v = &board.vias[vi as usize];
        let drill = f64::from(v.drill) / 1e6;
        let area = barrel_area_mm2(drill, plating);
        // Barrels are inner conductors for the modifiers' purposes.
        let cap = ipc2152_current(area, rise_on(rise, corr, ext_f, int_f, false));
        vias.push(ViaRow {
            via: vi,
            at: pmm(v.center),
            i,
            cap,
            drill,
            area,
        });
    }
    for v in &vias {
        findings.push(Finding {
            kind: "via",
            at: v.at,
            layer: None,
            current: v.i,
            capacity: v.cap,
            width_mm: v.drill,
            required_width_mm: None,
            detail: json!({
                "uuid": board.vias[v.via as usize].uuid,
                "diameter_mm": r4(f64::from(board.vias[v.via as usize].diameter) / 1e6),
                "barrel_area_mm2": r4(v.area),
                "fusing_a": pulse_ms.map(|ms| r4(onderdonk_fusing_current(v.area, ambient, ms / 1000.0))),
            }),
        });
    }
    // Parallel sets: single-linkage within set_mm.
    let mut set_of: Vec<usize> = (0..vias.len()).collect();
    fn root(p: &mut [usize], mut x: usize) -> usize {
        while p[x] != x {
            p[x] = p[p[x]];
            x = p[x];
        }
        x
    }
    for a in 0..vias.len() {
        for b in a + 1..vias.len() {
            if dist(vias[a].at, vias[b].at) <= set_mm {
                let (ra, rb) = (root(&mut set_of, a), root(&mut set_of, b));
                if ra != rb {
                    set_of[ra.max(rb)] = ra.min(rb);
                }
            }
        }
    }
    let mut sets: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for i in 0..vias.len() {
        let r = root(&mut set_of, i);
        sets.entry(r).or_default().push(i);
    }
    let mut via_sets = Vec::new();
    for members in sets.values() {
        let i: f64 = members.iter().map(|&m| vias[m].i).sum();
        let cap: f64 = members.iter().map(|&m| vias[m].cap).sum();
        let area: f64 = members.iter().map(|&m| vias[m].area).sum();
        let at = [
            members.iter().map(|&m| vias[m].at[0]).sum::<f64>() / members.len() as f64,
            members.iter().map(|&m| vias[m].at[1]).sum::<f64>() / members.len() as f64,
        ];
        let f = Finding {
            kind: "via_set",
            at,
            layer: None,
            current: i,
            capacity: cap,
            width_mm: 0.0,
            required_width_mm: None,
            detail: json!({
                "vias": members.len(),
                "per_via_capacity_a": members.iter().map(|&m| r4(vias[m].cap)).collect::<Vec<_>>(),
                "per_via_current_a": members.iter().map(|&m| r4(vias[m].i)).collect::<Vec<_>>(),
                "drills_mm": members.iter().map(|&m| r4(vias[m].drill)).collect::<Vec<_>>(),
                "members": members.iter().map(|&m| json!([r4(vias[m].at[0]), r4(vias[m].at[1])])).collect::<Vec<_>>(),
                "fusing_a": pulse_ms.map(|ms| r4(onderdonk_fusing_current(area, ambient, ms / 1000.0))),
            }),
        };
        via_sets.push(f.clone());
        findings.push(f);
    }

    // Pour necks on the path.
    let mut necks_found = Vec::new();
    // Only where the current flows: a plane's far side carries none.
    let active = nm.active_boxes(&sol, 0.01);
    for g in &nm.grids {
        let Some(&win) = active.get(&g.layer.0) else {
            continue;
        };
        let th = g.t;
        let rl = rise_l(g.layer);
        let threshold = ipc2152_width_mm(current, rl, th).min(neck_max);
        for z in &board.zones {
            if z.net != Some(nm.net) || z.layer != g.layer || !z.filled {
                continue;
            }
            let pts: Vec<(f64, f64)> = z.region.outline().iter().map(|&p| p.to_mm()).collect();
            let rings = unfracture(&pts);
            for n in find_necks_in(&rings, threshold, (2.0 * threshold).max(0.5), Some(win)) {
                let flux = nm.flux_across(&sol, g.layer, n.a, n.b);
                if flux < floor.max(current * 0.01) {
                    continue;
                }
                let f = Finding {
                    kind: "pour_neck",
                    at: n.at(),
                    layer: Some(board.layers.name(g.layer).to_string()),
                    current: flux,
                    capacity: ipc2152_current(n.width * th, rl),
                    width_mm: n.width,
                    required_width_mm: Some(ipc2152_width_mm(flux, rl, th)),
                    detail: json!({
                        "zone": z.zone_name,
                        "chord": [[r4(n.a[0]), r4(n.a[1])], [r4(n.b[0]), r4(n.b[1])]],
                        "copper_um": r4(th * 1000.0),
                        "fusing_a": pulse_ms.map(|ms| r4(onderdonk_fusing_current(n.width * th, ambient, ms / 1000.0))),
                    }),
                };
                necks_found.push(f.clone());
                findings.push(f);
            }
        }
    }

    // The bottleneck: the least margin among tracks, via sets and necks
    // (single vias are judged within their set).
    let peak_scale = peak.map(|p| p / current);
    let bottleneck = findings
        .iter()
        .filter(|f| f.kind != "via")
        .min_by(|a, b| a.margin().total_cmp(&b.margin()));
    let mut below: Vec<&Finding> = findings
        .iter()
        .filter(|f| f.margin() / peak_scale.unwrap_or(1.0).max(1.0) < 1.0 || f.margin() < 1.0)
        .collect();
    below.sort_by(|a, b| a.margin().total_cmp(&b.margin()));
    let mut markers = Vec::new();
    for f in &below {
        markers.push(marker(
            f.at[0],
            f.at[1],
            "current_capacity",
            format!("{name} {} {:.2}x", f.kind, f.margin()),
        ));
    }
    if let Some(b) = bottleneck {
        markers.push(marker(
            b.at[0],
            b.at[1],
            "bottleneck",
            format!("{name} bottleneck {} {:.2}x", b.kind, b.margin()),
        ));
    }
    let narrowest_neck = necks_found
        .iter()
        .min_by(|a, b| a.width_mm.total_cmp(&b.width_mm))
        .map(|f| f.json(peak_scale));
    let mut by_layer_neck: BTreeMap<String, Value> = BTreeMap::new();
    for n in &necks_found {
        let k = n.layer.clone().unwrap_or_default();
        let replace = by_layer_neck
            .get(&k)
            .is_none_or(|v| n.width_mm < v["width_mm"].as_f64().unwrap_or(f64::INFINITY));
        if replace {
            by_layer_neck.insert(k, n.json(peak_scale));
        }
    }
    let solve = json!({
        "unknowns": sol.unknowns, "iterations": sol.iterations,
        "relative_residual": sol.relative_residual, "converged": sol.converged,
        "model": nm.counts,
        "solve_ms": solve_ms,
        "post_ms": tp.elapsed().as_millis() as u64,
    });
    let terminals = json!({
        "sources": t.sources.iter().map(|&p| pad_label(board, &board.pads[p])).collect::<Vec<_>>(),
        "sinks": t.sinks.iter().map(|&p| pad_label(board, &board.pads[p])).collect::<Vec<_>>(),
        "sink_layers": t.sink_layers.iter().map(|&x| board.layers.name(x).to_string()).collect::<Vec<_>>(),
    });
    let full = json!({
        "net": name,
        "current_a": current, "peak_a": peak, "pulse_ms": pulse_ms,
        "terminals": terminals,
        "solve": solve,
        "bottleneck": bottleneck.map(|b| b.json(peak_scale)),
        "narrowest_neck": narrowest_neck,
        "narrowest_neck_per_layer": by_layer_neck,
        "via_sets": via_sets.iter().map(|f| f.json(peak_scale)).collect::<Vec<_>>(),
        "below_requirement": below.iter().map(|f| f.json(peak_scale)).collect::<Vec<_>>(),
        "elements": findings.iter().map(|f| f.json(peak_scale)).collect::<Vec<_>>(),
    });
    let summary = json!({
        "net": name,
        "current_a": current, "peak_a": peak,
        "terminals": full["terminals"],
        "solve": full["solve"],
        "bottleneck": full["bottleneck"],
        "narrowest_neck": full["narrowest_neck"],
        "narrowest_neck_per_layer": full["narrowest_neck_per_layer"],
        "via_sets": full["via_sets"],
        "elements_checked": findings.len(),
        "below_requirement_count": below.len(),
        "below_requirement": full["below_requirement"],
    });
    Ok((full, summary, markers))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A straight 1.0 mm F.Cu track between two pads: its capacity is the
    /// IPC-2152 value for 1.0 mm × 35 µm, and the whole current runs in it.
    #[test]
    fn a_single_track_reports_its_ipc_capacity() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("t.kicad_pcb");
        std::fs::write(&path, super::super::tests_support::TRACK_BOARD).unwrap();
        let l = common::load_at(&path).unwrap();
        let out = Outputs {
            output: None,
            markers: None,
        };
        let r = check_current_capacity(
            &l,
            &json!({ "board": path, "nets": [{ "net": "P", "current_a": 1.0, "sources": ["J1.1"], "sinks": ["J2.1"] }] }),
            &out,
        )
        .unwrap();
        let body: Value = match &r.content[0] {
            crate::mcp::protocol::ToolContent::Text { text } => serde_json::from_str(text).unwrap(),
            _ => panic!(),
        };
        let b = &body["nets"][0]["bottleneck"];
        assert_eq!(b["kind"], "track", "{body}");
        let cap = b["capacity_a"].as_f64().unwrap();
        assert!((cap - 1.994).abs() / 1.994 < 0.05, "{cap}");
        assert!((b["current_a"].as_f64().unwrap() - 1.0).abs() < 1e-3, "{b}");
    }
}
