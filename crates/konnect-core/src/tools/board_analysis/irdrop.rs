//! K3 `analyze_ir_drop`: DC voltage drop of a rail from its source pads to
//! its loads, through tracks, vias and meshed fills.

use super::common::{self, marker, merged, r4, r6, Loaded, Outputs};
use super::current::window;
use super::geom::{pmm, unfracture};
use super::ipc::rho_at;
use super::necks::find_necks_in;
use super::network::{pads_named, Kind, Network, Options};
use crate::mcp::protocol::CallToolResult;
use crate::tools::board_inspect::pad_label;
use crate::tools::route_native::{invalid, opt_count, Refusal};
use serde_json::{json, Value};
use std::collections::BTreeMap;

/// Sinks above which the per-sink resistance solves are skipped.
const MAX_RESISTANCE_SOLVES: usize = 24;

pub(crate) fn schema() -> Value {
    common::schema(
        json!({
            "net": { "type": "string", "minLength": 1, "description": "The rail" },
            "sources": { "type": "array", "minItems": 1, "items": { "type": "string" }, "description": "Source pads (REF.PAD), held at source_voltage" },
            "source_voltage": { "type": "number", "default": 0, "description": "Voltage at the source pads, V (drops are reported either way)" },
            "sinks": { "type": "object", "additionalProperties": { "type": "number", "minimum": 0 }, "description": "Load pad (REF.PAD) to current drawn, A" },
            "sinks_file": { "type": "string", "description": "More loads: a JSON object {\"REF.PAD\": amps} (or {\"sinks\": {...}}), or a CSV with pad,current_a columns" },
            "temperature_c": { "type": "number", "minimum": -55, "maximum": 200, "default": 25, "description": "Copper temperature for resistivity" },
            "plating_um": { "type": "number", "exclusiveMinimum": 0, "maximum": 100, "default": 20 },
            "resolution_mm": { "type": "number", "minimum": 0.005, "maximum": 2, "default": 0.1, "description": "Fill mesh step" },
            "max_cells": { "type": "integer", "minimum": 1000, "maximum": 50000000, "default": 4000000 },
            "area": { "type": "object", "properties": { "x1": { "type": "number" }, "y1": { "type": "number" }, "x2": { "type": "number" }, "y2": { "type": "number" } }, "required": ["x1", "y1", "x2", "y2"], "additionalProperties": false, "description": "Model only the rail's copper meeting this box, mm" },
            "hotspots": { "type": "integer", "minimum": 0, "maximum": 10000, "default": 20, "description": "Highest current-density places to report and mark" },
            "sink_resistance": { "type": "boolean", "default": true, "description": "Solve once more per load (at most 24 loads) for its own source-to-load resistance; off saves time on large meshes" },
            "auto_area": { "type": "boolean", "default": true, "description": "Without an area, model only the rail's copper within auto_area_margin_mm of the sources' and loads' box (the whole net when that cuts a load off); false models the whole net" },
            "auto_area_margin_mm": { "type": "number", "minimum": 0, "maximum": 1000, "default": 20 },
            "neck_report_mm": { "type": "number", "minimum": 0, "maximum": 5, "default": 0.3, "description": "Report the rail's fill necks narrower than this with the current and density through each" }
        }),
        &["board", "net", "sources"],
    )
}

fn read_sinks(args: &Value) -> Result<BTreeMap<String, f64>, Refusal> {
    let mut out = BTreeMap::new();
    let mut take = |k: &str, v: &Value, field: &str| -> Result<(), Refusal> {
        let a = v
            .as_f64()
            .filter(|a| a.is_finite() && *a >= 0.0)
            .ok_or_else(|| invalid(field, &format!("{k}: the current must be a number ≥ 0")))?;
        *out.entry(k.to_string()).or_insert(0.0) += a;
        Ok(())
    };
    if let Some(Value::Object(m)) = args.get("sinks") {
        for (k, v) in m {
            take(k, v, "sinks")?;
        }
    }
    if let Some(p) = crate::tools::route_native::opt_path(args, "sinks_file")? {
        let text = std::fs::read_to_string(&p)
            .map_err(|e| invalid("sinks_file", &format!("could not be read: {e}")))?;
        if p.extension()
            .and_then(|e| e.to_str())
            .is_some_and(|e| e.eq_ignore_ascii_case("csv"))
        {
            let mut rd = csv::Reader::from_reader(text.as_bytes());
            let headers = rd
                .headers()
                .map_err(|e| invalid("sinks_file", &e.to_string()))?
                .clone();
            let col = |names: &[&str]| {
                headers
                    .iter()
                    .position(|h| names.contains(&h.trim().to_ascii_lowercase().as_str()))
            };
            let (Some(pc), Some(ic)) = (
                col(&["pad", "pin"]),
                col(&["current_a", "current", "amps", "a"]),
            ) else {
                return Err(invalid(
                    "sinks_file",
                    "the CSV needs a pad column and a current_a column",
                ));
            };
            for row in rd.records() {
                let row = row.map_err(|e| invalid("sinks_file", &e.to_string()))?;
                let pad = row.get(pc).unwrap_or("").trim().to_string();
                let amps: f64 = row.get(ic).unwrap_or("").trim().parse().map_err(|_| {
                    invalid("sinks_file", &format!("{pad}: the current is not a number"))
                })?;
                take(&pad, &json!(amps), "sinks_file")?;
            }
        } else {
            let v: Value = serde_json::from_str(&text)
                .map_err(|e| invalid("sinks_file", &format!("is not JSON: {e}")))?;
            let m = match v {
                Value::Object(mut o) => match o.remove("sinks") {
                    Some(Value::Object(s)) => s,
                    Some(_) => return Err(invalid("sinks_file", "'sinks' must be an object")),
                    None => o,
                },
                _ => return Err(invalid("sinks_file", "must hold an object of pad to amps")),
            };
            for (k, v) in &m {
                take(k, v, "sinks_file")?;
            }
        }
    }
    if out.is_empty() {
        return Err(invalid(
            "sinks",
            "give at least one load (sinks and/or sinks_file)",
        ));
    }
    Ok(out)
}

pub(crate) fn analyze_ir_drop(
    l: &Loaded,
    args: &Value,
    out: &Outputs,
) -> Result<CallToolResult, Refusal> {
    let board = l.board();
    let net_name = args.get("net").and_then(Value::as_str).unwrap_or("");
    let net = common::net_named(board, net_name, "net")?;
    let src_names = crate::tools::route_native::opt_strings(args, "sources")?
        .ok_or_else(|| invalid("sources", "is required"))?;
    let sources = pads_named(board, &src_names, "sources")?;
    let v_src = common::opt_f64(args, "source_voltage", 0.0, -1e4, 1e4)?;
    let sinks_map = read_sinks(args)?;
    let names: Vec<String> = sinks_map.keys().cloned().collect();
    let sink_pads = pads_named(board, &names, "sinks")?;
    for &p in sources.iter().chain(&sink_pads) {
        if board.pads[p].net != Some(net) {
            return Err(invalid(
                "sinks",
                &format!("{} is not on {net_name}", pad_label(board, &board.pads[p])),
            ));
        }
    }
    let temp = common::opt_f64(args, "temperature_c", 25.0, -55.0, 200.0)?;
    let plating = common::opt_f64(args, "plating_um", 20.0, 0.1, 100.0)? / 1000.0;
    let h = common::opt_f64(args, "resolution_mm", 0.1, 0.005, 2.0)?;
    let max_cells = opt_count(args, "max_cells", 4_000_000, 50_000_000)? as usize;
    let hot_n = opt_count(args, "hotspots", 20, 10_000)? as usize;
    let neck_mm = common::opt_f64(args, "neck_report_mm", 0.3, 0.0, 5.0)?;
    let want_r = crate::tools::route_native::opt_bool(args, "sink_resistance", true)?;
    let max_items = opt_count(args, "max_items", 50, 100_000)? as usize;
    let win = window(args, "area")?;
    let t0 = std::time::Instant::now();

    let auto_area = crate::tools::route_native::opt_bool(args, "auto_area", true)?;
    let auto_margin = common::opt_f64(args, "auto_area_margin_mm", 20.0, 0.0, 1000.0)?;
    let auto = (win.is_none() && auto_area).then(|| {
        super::current::terminal_box(
            board,
            sources.iter().chain(&sink_pads).copied(),
            auto_margin,
        )
    });
    let build = |w: Option<(super::geom::P, super::geom::P)>| {
        Network::build(
            l,
            net,
            &Options {
                h,
                temp_c: temp,
                plating_mm: plating,
                max_cells,
                window: w,
            },
        )
    };
    // Every terminal must be in the model and joined to a source; else the
    // auto box gives way to the whole net.
    let joined = |nm: &Network| -> bool {
        let Some(fixed) = sources
            .iter()
            .map(|&p| nm.pad(p).map(|n| (n, v_src)))
            .collect::<Option<Vec<_>>>()
        else {
            return false;
        };
        let Some(loads) = sink_pads
            .iter()
            .map(|&p| nm.pad(p))
            .collect::<Option<Vec<_>>>()
        else {
            return false;
        };
        nm.reaches(&fixed, &loads)
    };
    let (nm, win_used) = match auto {
        Some(w) => {
            let nm = build(Some(w)).map_err(|e| invalid("resolution_mm", &e))?;
            if joined(&nm) {
                (nm, Some(w))
            } else {
                (build(None).map_err(|e| invalid("resolution_mm", &e))?, None)
            }
        }
        None => (build(win).map_err(|e| invalid("resolution_mm", &e))?, win),
    };
    let modelled_area = json!({
        "box": win_used.map(|(a, b)| json!({ "x1": r4(a[0]), "y1": r4(a[1]), "x2": r4(b[0]), "y2": r4(b[1]) })),
        "source": if win.is_some() { "area" } else if win_used.is_some() { "auto: the terminals' box grown by auto_area_margin_mm" } else { "whole net" },
    });
    let node_of = |p: usize| {
        nm.pad(p).ok_or_else(|| {
            invalid(
                "area",
                &format!(
                    "{} is outside the modelled area",
                    pad_label(board, &board.pads[p])
                ),
            )
        })
    };
    let fixed: Vec<(u32, f64)> = sources
        .iter()
        .map(|&p| node_of(p).map(|n| (n, v_src)))
        .collect::<Result<_, _>>()?;
    // Each load pad by name (a pad named twice draws the sum).
    let mut loads: Vec<(String, usize, u32, f64)> = Vec::new();
    for (name, &amps) in &sinks_map {
        let pads = pads_named(board, std::slice::from_ref(name), "sinks")?;
        let share = amps / pads.len() as f64;
        for p in pads {
            loads.push((name.clone(), p, node_of(p)?, share));
        }
    }
    let inject: Vec<(u32, f64)> = loads.iter().map(|l| (l.2, -l.3)).collect();
    let build_ms = t0.elapsed().as_millis() as u64;
    let ts = std::time::Instant::now();
    let sol = nm
        .solve(&fixed, &inject)
        .map_err(|e| invalid("sinks", &e))?;
    let solve_ms = ts.elapsed().as_millis() as u64;
    let total: f64 = loads.iter().map(|l| l.3).sum();

    // Per-sink resistance: one solve per load drawing 1 A alone.
    let resistance: Option<Vec<f64>> =
        (want_r && loads.len() <= MAX_RESISTANCE_SOLVES).then(|| {
            loads
                .iter()
                .map(|ld| {
                    nm.solve(&fixed, &[(ld.2, -1.0)])
                        .map(|s| v_src - s.v[ld.2 as usize])
                        .unwrap_or(f64::NAN)
                })
                .collect()
        });
    let mut sink_rows: Vec<Value> = loads
        .iter()
        .enumerate()
        .map(|(i, ld)| {
            let v = sol.v[ld.2 as usize];
            json!({
                "pad": pad_label(board, &board.pads[ld.1]),
                "current_a": r6(ld.3),
                "voltage_v": r6(v),
                "drop_mv": r4((v_src - v) * 1000.0),
                "resistance_mohm": resistance.as_ref().map(|r| r4(r[i] * 1000.0)),
                "at": [r4(pmm(board.pads[ld.1].center)[0]), r4(pmm(board.pads[ld.1].center)[1])],
            })
        })
        .collect();
    sink_rows.sort_by(|a, b| {
        b["drop_mv"]
            .as_f64()
            .unwrap_or(0.0)
            .total_cmp(&a["drop_mv"].as_f64().unwrap_or(0.0))
    });
    let worst = sink_rows.first().cloned();

    // Current density: fill cells and tracks.
    let mut dens: Vec<(f64, [f64; 2], String, &'static str)> = Vec::new();
    for (node, layer, j) in nm.cell_density(&sol) {
        dens.push((
            j,
            nm.nodes[node as usize].at,
            board.layers.name(layer).to_string(),
            "fill",
        ));
    }
    for e in &nm.elems {
        let layer = match e.kind {
            Kind::Track { track } => board.tracks[track as usize].layer,
            Kind::Arc { arc } => board.arcs[arc as usize].layer,
            _ => continue,
        };
        let i = nm.current(&sol, e).abs();
        let j = i / (e.width * l.stack.t(layer));
        dens.push((
            j,
            [(e.p[0] + e.q[0]) / 2.0, (e.p[1] + e.q[1]) / 2.0],
            board.layers.name(layer).to_string(),
            "track",
        ));
    }
    for e in &nm.elems {
        if let Kind::Barrel { .. } = e.kind {
            let i = nm.current(&sol, e).abs();
            let a = super::ipc::barrel_area_mm2(e.width, plating);
            dens.push((i / a, e.p, "barrel".to_string(), "via"));
        }
    }
    dens.sort_by(|a, b| b.0.total_cmp(&a.0));
    // One hotspot per 1 mm neighbourhood.
    let mut hot: Vec<&(f64, [f64; 2], String, &'static str)> = Vec::new();
    for d in &dens {
        if hot.len() >= hot_n {
            break;
        }
        if hot
            .iter()
            .all(|h| super::geom::dist(h.1, d.1) > 1.0 || h.2 != d.2)
        {
            hot.push(d);
        }
    }
    let mut markers: Vec<Value> = hot
        .iter()
        .map(|d| {
            marker(
                d.1[0],
                d.1[1],
                "current_density",
                format!("{:.1} A/mm² {}", d.0, d.2),
            )
        })
        .collect();
    if let Some(w) = &worst {
        markers.push(marker(
            w["at"][0].as_f64().unwrap_or(0.0),
            w["at"][1].as_f64().unwrap_or(0.0),
            "worst_drop",
            format!("{} -{} mV", w["pad"].as_str().unwrap_or(""), w["drop_mv"]),
        ));
    }

    // Necks of the rail's fills and the current through each.
    let mut necks = Vec::new();
    if neck_mm > 0.0 {
        let active = nm.active_boxes(&sol, 0.01);
        for g in &nm.grids {
            let Some(&win) = active.get(&g.layer.0) else {
                continue;
            };
            for z in &board.zones {
                if z.net != Some(net) || z.layer != g.layer || !z.filled {
                    continue;
                }
                let pts: Vec<(f64, f64)> = z.region.outline().iter().map(|&p| p.to_mm()).collect();
                for n in find_necks_in(
                    &unfracture(&pts),
                    neck_mm,
                    (2.0 * neck_mm).max(0.5),
                    Some(win),
                ) {
                    if n.kind() != "neck" {
                        continue;
                    }
                    let flux = nm.flux_across(&sol, g.layer, n.a, n.b);
                    let at = n.at();
                    markers.push(marker(
                        at[0],
                        at[1],
                        "fill_neck",
                        format!(
                            "{} {:.3} mm {:.3} A",
                            z.zone_name.as_deref().unwrap_or(""),
                            n.width,
                            flux
                        ),
                    ));
                    necks.push(json!({
                        "zone": z.zone_name,
                        "layer": board.layers.name(g.layer),
                        "width_mm": r4(n.width),
                        "at": [r4(at[0]), r4(at[1])],
                        "chord": [[r4(n.a[0]), r4(n.a[1])], [r4(n.b[0]), r4(n.b[1])]],
                        "current_a": r6(flux),
                        "density_a_per_mm2": r4(flux / (n.width * g.t)),
                    }));
                }
            }
        }
        necks.sort_by(|a, b| {
            a["width_mm"]
                .as_f64()
                .unwrap_or(0.0)
                .total_cmp(&b["width_mm"].as_f64().unwrap_or(0.0))
        });
    }

    let hot_rows: Vec<Value> = hot
        .iter()
        .map(|d| json!({ "density_a_per_mm2": r4(d.0), "at": [r4(d.1[0]), r4(d.1[1])], "layer": d.2, "in": d.3 }))
        .collect();
    let rho = rho_at(temp);
    let methods = json!({
        "model": "tracks ρL/(w·t); via barrels ρ·Δz/A (plated annulus); fills meshed at resolution_mm with sheet resistance ρ/t per square (coverage-weighted); pads equipotential",
        "sheet_resistance_mohm_per_sq": l.stack.thickness_mm.iter().enumerate().map(|(i, t)| (board.layers.copper[i].name.clone(), r4(rho / t * 1000.0))).collect::<BTreeMap<_, _>>(),
        "solver": "conjugate gradients, SSOR preconditioned (ω 1.6), sources held at source_voltage, loads drawing their currents",
        "resistance": "source-to-sink resistance: the same network with only that load drawing 1 A",
        "density": "fill cells: mean link current per axis over h·t·coverage; tracks I/(w·t); barrels I/A",
    });
    let full = merged(
        common::provenance(l),
        json!({
            "net": net_name,
            "source_voltage": v_src,
            "sources": src_names,
            "temperature_c": temp,
            "resolution_mm": h,
            "total_current_a": r6(total),
            "modelled_area": modelled_area,
            "solve": { "build_ms": build_ms, "solve_ms": solve_ms, "unknowns": sol.unknowns, "iterations": sol.iterations, "relative_residual": sol.relative_residual, "converged": sol.converged, "model": nm.counts },
            "stackup": l.stack.json(board),
            "worst": worst,
            "sinks": sink_rows,
            "hotspots": hot_rows,
            "necks": necks,
            "methods": methods,
        }),
    );
    let files = out.write(&full, &markers)?;
    Ok(CallToolResult::json(&merged(
        common::provenance(l),
        json!({
            "net": net_name,
            "total_current_a": r6(total),
            "modelled_area": full["modelled_area"],
            "solve": full["solve"],
            "worst": full["worst"],
            "sinks": super::hygiene::take(&full["sinks"], max_items),
            "hotspots": super::hygiene::take(&full["hotspots"], max_items),
            "necks": super::hygiene::take(&full["necks"], max_items),
            "files": files,
            "methods": full["methods"],
            "elapsed_ms": t0.elapsed().as_millis() as u64,
        }),
    )))
}

#[cfg(test)]
mod tests {
    use super::super::tests_support as ts;
    use super::*;

    fn body(r: &CallToolResult) -> Value {
        match &r.content[0] {
            crate::mcp::protocol::ToolContent::Text { text } => serde_json::from_str(text).unwrap(),
            _ => panic!(),
        }
    }

    fn run_on(board: &str, args: Value) -> Value {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("t.kicad_pcb");
        std::fs::write(&path, board).unwrap();
        let l = common::load_at(&path).unwrap();
        let out = Outputs {
            output: None,
            markers: None,
        };
        let mut a = args;
        a["board"] = json!(path);
        let r = analyze_ir_drop(&l, &a, &out).unwrap_or_else(|r| r);
        let b = body(&r);
        assert!(!r.is_error, "{b}");
        b
    }

    /// A straight track: ρL/(w·t) within 1 %.
    #[test]
    fn a_straight_track_matches_rho_l_over_wt() {
        let b = run_on(
            ts::TRACK_BOARD,
            json!({ "net": "P", "sources": ["J1.1"], "sinks": { "J2.1": 1.0 }, "temperature_c": 20 }),
        );
        let r = b["worst"]["resistance_mohm"].as_f64().unwrap() / 1000.0;
        let hand = 1.7241e-5 * 100.0 / (1.0 * 0.035);
        assert!(((r - hand) / hand).abs() < 0.01, "{r} vs {hand}");
        let drop = b["worst"]["drop_mv"].as_f64().unwrap() / 1000.0;
        assert!(((drop - hand) / hand).abs() < 0.01, "{drop}");
    }

    /// A 20 × 5 mm pour between two pads covering its short edges: the
    /// squares between the pads' inner edges times ρ/t, within 5 %.
    #[test]
    fn a_rectangular_pour_matches_its_squares() {
        let board = ts::board(&[
            ts::zone("POUR", "F.Cu", "P", 0.0, 0.0, 20.0, 5.0),
            ts::pad("J1", 0.25, 2.5, 0.5, 5.0, "P"),
            ts::pad("J2", 19.75, 2.5, 0.5, 5.0, "P"),
        ]);
        let b = run_on(
            &board,
            json!({ "net": "P", "sources": ["J1.1"], "sinks": { "J2.1": 2.0 }, "temperature_c": 20, "resolution_mm": 0.05 }),
        );
        let r = b["worst"]["resistance_mohm"].as_f64().unwrap() / 1000.0;
        let squares = 19.0 / 5.0;
        let hand = 1.7241e-5 / 0.035 * squares;
        assert!(((r - hand) / hand).abs() < 0.05, "{r} vs {hand}");
    }

    /// A neck in a pour carries the whole current and shows as a hotspot.
    #[test]
    fn a_pour_neck_is_reported_with_its_current() {
        let board = ts::board(&[
            "\t(zone (net \"P\") (layer \"F.Cu\") (name \"DUMBBELL\") (connect_pads yes (clearance 0.2)) (min_thickness 0.05) (fill yes)\n\t\t(polygon (pts (xy 0 0) (xy 4 0) (xy 4 1.9) (xy 6 1.9) (xy 6 0) (xy 10 0) (xy 10 4) (xy 6 4) (xy 6 2.1) (xy 4 2.1) (xy 4 4) (xy 0 4)))\n\t\t(filled_polygon (layer \"F.Cu\") (pts (xy 0 0) (xy 4 0) (xy 4 1.9) (xy 6 1.9) (xy 6 0) (xy 10 0) (xy 10 4) (xy 6 4) (xy 6 2.1) (xy 4 2.1) (xy 4 4) (xy 0 4)))\n\t)\n".to_string(),
            ts::pad("J1", 0.25, 2.0, 0.5, 4.0, "P"),
            ts::pad("J2", 9.75, 2.0, 0.5, 4.0, "P"),
        ]);
        let b = run_on(
            &board,
            json!({ "net": "P", "sources": ["J1.1"], "sinks": { "J2.1": 1.0 }, "resolution_mm": 0.05, "neck_report_mm": 0.3 }),
        );
        let n = &b["necks"][0];
        assert!((n["width_mm"].as_f64().unwrap() - 0.2).abs() < 1e-6, "{b}");
        assert!((n["current_a"].as_f64().unwrap() - 1.0).abs() < 0.05, "{n}");
        let hot = &b["hotspots"][0]["at"];
        assert!(
            hot[0].as_f64().unwrap() > 3.5 && hot[0].as_f64().unwrap() < 6.5,
            "{b}"
        );
    }
}
