//! K6 `check_thermal_pads`: for each exposed or thermal pad, its vias, the
//! same-net copper that spreads its heat on each layer, and its paste
//! windows, against an optional datasheet record.

use super::common::{self, marker, merged, r4, Loaded, Outputs};
use super::geom::{dist, pmm, polygon_rings, Raster};
use super::ipc::barrel_area_mm2;
use crate::mcp::protocol::CallToolResult;
use crate::tools::board_inspect::pad_label;
use crate::tools::route_native::{invalid, opt_count, opt_strings, Refusal};
use konnect_geom::{Circle, IndexedPolygon, Point, Shape};
use konnect_sexp::SexpNode;
use serde_json::{json, Value};
use std::collections::BTreeMap;

/// Copper thermal conductivity, W/(m·K).
const K_CU: f64 = 385.0;

pub(crate) fn schema() -> Value {
    common::schema(
        json!({
            "references": { "type": "array", "items": { "type": "string" }, "minItems": 1, "description": "Footprints to audit (default: every footprint with an exposed pad: an SMD pad of at least 2 mm² and three times the footprint's median pad area)" },
            "pads": { "type": "array", "items": { "type": "string" }, "minItems": 1, "description": "Exact thermal pads (REF.PAD) instead of detection" },
            "radius_mm": { "type": "number", "exclusiveMinimum": 0, "maximum": 50, "default": 5, "description": "Copper within this distance of the pad centre counts as spreading" },
            "plating_um": { "type": "number", "exclusiveMinimum": 0, "maximum": 100, "default": 20 },
            "recommended": { "type": "object", "additionalProperties": { "type": "object", "properties": {
                "vias_min": { "type": "integer", "minimum": 0 },
                "drill_mm": { "type": "number", "exclusiveMinimum": 0 },
                "drill_tolerance_mm": { "type": "number", "minimum": 0, "default": 0.05 },
                "area_mm2_min": { "type": "object", "additionalProperties": { "type": "number", "minimum": 0 }, "description": "Layer to minimum spreading copper, mm²" },
                "paste_coverage_max": { "type": "number", "minimum": 0, "maximum": 1 }
            }, "additionalProperties": false }, "description": "Footprint reference (or REF.PAD) to its datasheet recommendation" },
            "recommended_file": { "type": "string", "description": "The same map in a JSON file" }
        }),
        &["board"],
    )
}

fn pad_area(p: &konnect_route::board::Pad) -> f64 {
    p.shapes
        .iter()
        .map(|s| super::geom::rings_area(&super::geom::shape_rings(s, 48)))
        .fold(0.0, f64::max)
}

/// The footprint node with `reference`.
fn footprint_node<'a>(tree: &'a SexpNode, reference: &str) -> Option<&'a SexpNode> {
    tree.find_all("footprint").into_iter().find(|fp| {
        fp.find_all("property").into_iter().any(|p| {
            p.get(1).and_then(SexpNode::as_str) == Some("Reference")
                && p.get(2).and_then(SexpNode::as_str) == Some(reference)
        })
    })
}

fn layer_tokens(pad: &SexpNode) -> Vec<String> {
    pad.find("layers")
        .and_then(SexpNode::children)
        .unwrap_or(&[])
        .iter()
        .skip(1)
        .filter_map(SexpNode::as_str)
        .map(str::to_string)
        .collect()
}

pub(crate) fn check_thermal_pads(
    l: &Loaded,
    args: &Value,
    out: &Outputs,
) -> Result<CallToolResult, Refusal> {
    let board = l.board();
    let radius = common::opt_f64(args, "radius_mm", 5.0, 1e-3, 50.0)?;
    let plating = common::opt_f64(args, "plating_um", 20.0, 0.1, 100.0)? / 1000.0;
    let max_items = opt_count(args, "max_items", 50, 100_000)? as usize;
    let mut recommended: BTreeMap<String, Value> = BTreeMap::new();
    if let Some(Value::Object(m)) = args.get("recommended") {
        recommended.extend(m.clone());
    }
    if let Some(Value::Object(m)) = common::json_file(args, "recommended_file")? {
        recommended.extend(m);
    }
    let t0 = std::time::Instant::now();

    // The thermal pads.
    let mut targets: Vec<usize> = Vec::new();
    if let Some(names) = opt_strings(args, "pads")? {
        targets = super::network::pads_named(board, &names, "pads")?;
    } else {
        let refs: Option<Vec<String>> = opt_strings(args, "references")?;
        for fp in &board.footprints {
            let Some(r) = &fp.reference else { continue };
            if refs.as_ref().is_some_and(|list| !list.contains(r)) {
                continue;
            }
            let range: Vec<usize> = (fp.pad_range.0 as usize..fp.pad_range.1 as usize)
                .filter(|&i| !board.pads[i].layers.is_empty())
                .collect();
            if range.len() < 2 {
                continue;
            }
            let mut areas: Vec<f64> = range.iter().map(|&i| pad_area(&board.pads[i])).collect();
            let (big_i, big) = range
                .iter()
                .zip(&areas)
                .max_by(|a, b| a.1.total_cmp(b.1))
                .map(|(&i, &a)| (i, a))
                .unwrap();
            areas.sort_by(f64::total_cmp);
            let median = areas[areas.len() / 2];
            let smd = board.pads[big_i].kind == konnect_route::board::PadKind::Smd;
            if smd && big >= 2.0 && big >= 3.0 * median {
                // Every pad of that size (some packages split the exposed pad).
                for &i in &range {
                    if pad_area(&board.pads[i]) >= 0.95 * big {
                        targets.push(i);
                    }
                }
            }
        }
        if let Some(list) = &refs {
            for r in list {
                if !board
                    .footprints
                    .iter()
                    .any(|f| f.reference.as_deref() == Some(r.as_str()))
                {
                    return Err(invalid("references", &format!("{r} is not on the board")));
                }
            }
        }
    }

    let mut rows = Vec::new();
    let mut markers = Vec::new();
    for &pi in &targets {
        let pad = &board.pads[pi];
        let label = pad_label(board, pad);
        let fp = &board.footprints[pad.footprint.index()];
        let reference = fp.reference.clone().unwrap_or_default();
        let c = pmm(pad.center);
        let area = pad_area(pad);
        // Vias inside the pad: centre inside its copper.
        let mut in_pad = Vec::new();
        let mut near_pad = 0usize;
        for v in &board.vias {
            let vc = pmm(v.center);
            if dist(vc, c) > radius + 2.0 {
                continue;
            }
            let Some(p) = Point::from_mm(vc[0], vc[1]) else {
                continue;
            };
            let inside = pad.shapes.iter().any(|s| {
                konnect_geom::point_shape_distance_cmp(p, 0, s, 0) != std::cmp::Ordering::Greater
            });
            if inside {
                in_pad.push(v);
            } else if v.net == pad.net && dist(vc, c) <= radius {
                near_pad += 1;
            }
        }
        let foreign_in_pad = in_pad.iter().filter(|v| v.net != pad.net).count();
        let drills: Vec<f64> = in_pad.iter().map(|v| f64::from(v.drill) / 1e6).collect();
        let barrel: f64 = drills.iter().map(|d| barrel_area_mm2(*d, plating)).sum();
        let span = l.stack.z_mm.last().copied().unwrap_or(1.6)
            - l.stack.z_mm.first().copied().unwrap_or(0.0);
        let r_th = if barrel > 0.0 {
            Some(span / 1000.0 / (K_CU * barrel / 1e6))
        } else {
            None
        };

        // Spreading copper per layer within the radius: the net's fill
        // pieces reached from the pad (its own layer) or by a net via within
        // the radius, plus the pad itself, sampled on a 0.05 mm grid.
        let h = (radius / 200.0).clamp(0.01, 0.05);
        let lo = [c[0] - radius, c[1] - radius];
        let hi = [c[0] + radius, c[1] + radius];
        let net_vias: Vec<&konnect_route::board::Via> = board
            .vias
            .iter()
            .filter(|v| v.net == pad.net && pad.net.is_some() && dist(pmm(v.center), c) <= radius)
            .collect();
        let mut per_layer = BTreeMap::new();
        for layer in board.layers.all().iter() {
            let mut r = Raster::new(lo, hi, h);
            let mut any = false;
            if pad.layers.contains(layer) {
                for s in &pad.shapes {
                    r.fill(&super::geom::shape_rings(s, 48), 2);
                }
                any = true;
            }
            let reached: Vec<&konnect_route::board::Via> = net_vias
                .iter()
                .copied()
                .filter(|v| v.layers.contains(layer))
                .collect();
            for z in &board.zones {
                if z.net != pad.net || pad.net.is_none() || z.layer != layer || !z.filled {
                    continue;
                }
                let b = z.region.bbox();
                let (zl, zh) = (pmm(b.min), pmm(b.max));
                if zh[0] < lo[0] || zl[0] > hi[0] || zh[1] < lo[1] || zl[1] > hi[1] {
                    continue;
                }
                let ix = IndexedPolygon::new(z.region.clone());
                let touches_pad =
                    pad.layers.contains(layer) && pad.shapes.iter().any(|s| ix.touches(s));
                let touches_via = reached
                    .iter()
                    .any(|v| ix.touches(&Shape::Circle(Circle::new(v.center, v.diameter / 2))));
                if touches_pad || touches_via {
                    r.fill(&polygon_rings(&z.region), 2);
                    any = true;
                }
            }
            if !any {
                continue;
            }
            // Clip to the disc.
            let mut a = 0.0;
            for j in 0..r.ny {
                for i in 0..r.nx {
                    let p = r.center(i, j);
                    if dist(p, c) <= radius {
                        a += f64::from(r.cov[r.idx(i, j)]) * h * h;
                    }
                }
            }
            if a > 0.0 {
                per_layer.insert(board.layers.name(layer).to_string(), r4(a));
            }
        }

        // Paste: the pad's own paste layer and paste-only apertures over it.
        let mut paste = json!({ "available": false });
        if let Some(node) = footprint_node(&l.tree, &reference) {
            let mut own_paste = false;
            let mut apertures = 0usize;
            let mut aperture_area = 0.0;
            let pb = pad.bbox().map(|b| (pmm(b.min), pmm(b.max)));
            for (k, pn) in node.find_all("pad").into_iter().enumerate() {
                let tokens = layer_tokens(pn);
                let is_paste = tokens.iter().any(|t| t.ends_with(".Paste"));
                let has_cu = tokens.iter().any(|t| t.ends_with(".Cu"));
                let number = pn.get(1).and_then(SexpNode::as_str).unwrap_or("");
                let board_pad = fp.pad_range.0 as usize + k;
                if board_pad >= fp.pad_range.1 as usize {
                    break;
                }
                if board_pad == pi
                    || (number == pad.number
                        && has_cu
                        && board.pads[board_pad].center == pad.center)
                {
                    own_paste = is_paste;
                } else if is_paste && !has_cu {
                    let bp = &board.pads[board_pad];
                    let ac = pmm(bp.center);
                    if pb.is_some_and(|(a, b)| {
                        ac[0] >= a[0] && ac[0] <= b[0] && ac[1] >= a[1] && ac[1] <= b[1]
                    }) {
                        apertures += 1;
                        aperture_area += pad_area(bp);
                    }
                }
            }
            let paste_margin = node
                .find_all("pad")
                .into_iter()
                .find(|p| p.get(1).and_then(SexpNode::as_str) == Some(pad.number.as_str()))
                .and_then(|p| {
                    p.find_f64("solder_paste_margin_ratio")
                        .or_else(|| p.find_f64("solder_paste_ratio"))
                });
            paste = json!({
                "available": true,
                "pad_has_paste_layer": own_paste,
                "apertures": apertures,
                "windowed": apertures > 0 && !own_paste,
                "paste_coverage": if apertures > 0 && !own_paste { json!(r4(aperture_area / area.max(1e-9))) } else if own_paste { json!(r4((1.0 + paste_margin.unwrap_or(0.0)).powi(2).min(1.0))) } else { json!(0.0) },
                "paste_margin_ratio": paste_margin,
            });
        }

        // Against the recommendation.
        let rec = recommended
            .get(&label)
            .or_else(|| recommended.get(&reference));
        let mut checks = Vec::new();
        if let Some(rec) = rec {
            if let Some(n) = rec["vias_min"].as_u64() {
                let ok = in_pad.len() as u64 >= n;
                checks.push(
                    json!({ "check": "vias_min", "required": n, "actual": in_pad.len(), "ok": ok }),
                );
            }
            if let Some(d) = rec["drill_mm"].as_f64() {
                let tol = rec["drill_tolerance_mm"].as_f64().unwrap_or(0.05);
                let ok = !drills.is_empty() && drills.iter().all(|x| (x - d).abs() <= tol);
                checks.push(json!({ "check": "drill_mm", "required": d, "actual": drills.iter().map(|x| r4(*x)).collect::<Vec<_>>(), "ok": ok }));
            }
            if let Some(Value::Object(m)) = rec.get("area_mm2_min") {
                for (layer, min) in m {
                    let a = per_layer.get(layer).copied().unwrap_or(0.0);
                    let ok = a >= min.as_f64().unwrap_or(0.0);
                    checks.push(json!({ "check": format!("area_mm2_min {layer}"), "required": min, "actual": a, "ok": ok }));
                }
            }
            if let Some(m) = rec["paste_coverage_max"].as_f64() {
                let a = paste["paste_coverage"].as_f64().unwrap_or(1.0);
                checks.push(json!({ "check": "paste_coverage_max", "required": m, "actual": a, "ok": a <= m }));
            }
        }
        let ok = checks.iter().all(|c| c["ok"] == true);
        markers.push(marker(
            c[0],
            c[1],
            if ok {
                "thermal_pad"
            } else {
                "thermal_pad_short"
            },
            format!("{label} {} vias", in_pad.len()),
        ));
        rows.push(json!({
            "pad": label,
            "net": pad.net.map(|n| board.net(n).name.clone()),
            "at": [r4(c[0]), r4(c[1])],
            "pad_area_mm2": r4(area),
            "vias_in_pad": in_pad.len(),
            "foreign_net_vias_in_pad": foreign_in_pad,
            "drills_mm": drills.iter().map(|d| r4(*d)).collect::<Vec<_>>(),
            "via_list": in_pad.iter().map(|v| json!({ "at": [r4(pmm(v.center)[0]), r4(pmm(v.center)[1])], "drill_mm": r4(f64::from(v.drill) / 1e6), "diameter_mm": r4(f64::from(v.diameter) / 1e6), "layers": crate::tools::board_inspect::layer_names(board, v.layers), "uuid": v.uuid })).collect::<Vec<_>>(),
            "plating_um": plating * 1000.0,
            "barrel_copper_mm2": r4(barrel),
            "via_thermal_resistance_k_per_w": r_th.map(r4),
            "same_net_vias_within_radius_outside_pad": near_pad,
            "radius_mm": radius,
            "spreading_copper_mm2": per_layer,
            "paste": paste,
            "recommended": rec,
            "checks": checks,
            "ok": ok,
        }));
    }
    let methods = json!({
        "detection": "SMD pads of at least 2 mm² and three times their footprint's median pad area (or the named pads)",
        "vias": "vias whose centre lies in the pad copper; barrel copper is the plated annulus (drill, plating_um); thermal resistance of the barrels in parallel over the board's copper-to-copper depth, k = 385 W/m·K",
        "spreading": "per layer, the area within radius_mm of the pad centre of the pad itself and of the net's fill pieces touching the pad or a net via within the radius (rasterized at radius/200, 0.01–0.05 mm)",
        "paste": "from the footprint: whether the pad carries a paste layer, and paste-only aperture pads centred over it (windowed when apertures replace the pad's own paste); coverage is aperture area over pad area",
    });
    let full = merged(
        common::provenance(l),
        json!({ "thermal_pads": rows, "methods": methods }),
    );
    let files = out.write(&full, &markers)?;
    Ok(CallToolResult::json(&merged(
        common::provenance(l),
        json!({
            "thermal_pads_checked": full["thermal_pads"].as_array().map_or(0, Vec::len),
            "failing": full["thermal_pads"].as_array().map_or(0, |a| a.iter().filter(|r| r["ok"] == false).count()),
            "thermal_pads": super::hygiene::take(&full["thermal_pads"], max_items).as_array().map(|a| a.iter().map(|r| {
                let mut r = r.clone();
                if let Some(o) = r.as_object_mut() { o.remove("via_list"); }
                r
            }).collect::<Vec<_>>()),
            "files": files,
            "methods": full["methods"],
            "elapsed_ms": t0.elapsed().as_millis() as u64,
        }),
    )))
}
