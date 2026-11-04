//! K4 `check_stitching_vias`: how well a net's pours (normally GND) are
//! stitched: the largest via-free circle in each pour, the via pitch along
//! pour and board edges, the vias tying each small pour to the planes, the
//! vias around focus footprints and next to focus nets' vias, and a density
//! heat map.

use super::common::{self, marker, merged, r4, Loaded, Outputs};
use super::geom::{dist, pmm, rings_area, unfracture, Grid, Raster, P};
use crate::mcp::protocol::CallToolResult;
use crate::tools::board_inspect::{self, pad_label};
use crate::tools::route_native::{invalid, opt_count, opt_strings, Refusal};
use konnect_geom::{Circle, IndexedPolygon, Point, Shape};
use konnect_route::api::{LayerId, LayerMask};
use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet};

pub(crate) fn schema() -> Value {
    common::schema(
        json!({
            "net": { "type": "string", "default": "GND", "description": "The net whose pours are stitched" },
            "layers": { "type": "array", "items": { "type": "string" }, "minItems": 1, "description": "Layers whose pours must be stitched (default: every layer with a fill of the net)" },
            "plane_layers": { "type": "array", "items": { "type": "string" }, "minItems": 1, "description": "The planes small pours tie to (default: layers where the net's largest piece covers a quarter of the board outline or more)" },
            "focus_footprints": { "type": "array", "items": { "type": "string" }, "minItems": 1, "description": "Footprints (connectors, TVS) to count stitching vias around" },
            "focus_radius_mm": { "type": "number", "exclusiveMinimum": 0, "maximum": 50, "default": 3, "description": "A via counts for a focus footprint when its edge is within this distance of one of the footprint's pads" },
            "focus_nets": { "type": "array", "items": { "type": "string" }, "minItems": 1, "description": "Nets (names or * ? patterns; fast buses) whose every via is paired with its nearest stitching via" },
            "edge_reach_mm": { "type": "number", "exclusiveMinimum": 0, "maximum": 20, "default": 2, "description": "A via stitches an edge point within this distance" },
            "sample_mm": { "type": "number", "minimum": 0.05, "maximum": 5, "default": 0.5, "description": "Sampling step for via-free circles and edge walks" },
            "grid_mm": { "type": "number", "minimum": 0.5, "maximum": 50, "default": 5, "description": "Heat-map cell" },
            "island_area_mm2": { "type": "number", "minimum": 0, "default": 400, "description": "Pieces smaller than this are islands whose ties are listed" }
        }),
        &["board"],
    )
}

struct StitchVia {
    at: P,
    r: f64,
    layers: LayerMask,
    uuid: Option<String>,
    through_hole_pad: Option<String>,
}

/// Distance from `p` to the nearest stitch spanning `layer`.
fn nearest(
    vgrid: &Grid<u32>,
    vias: &[StitchVia],
    p: P,
    layer: LayerId,
    max: f64,
) -> (f64, Option<u32>) {
    let mut r = 1.0f64;
    loop {
        let mut best = (f64::INFINITY, None);
        vgrid.query([p[0] - r, p[1] - r], [p[0] + r, p[1] + r], |i| {
            let v = &vias[i as usize];
            if v.layers.contains(layer) {
                let d = dist(v.at, p);
                if d < best.0 {
                    best = (d, Some(i));
                }
            }
        });
        if best.0 <= r || r >= max {
            return best;
        }
        r *= 2.0;
    }
}

/// Positions along the walk of a closed ring of segments where a stitch is
/// within `reach`, and the gaps between them.
fn edge_walk(
    segs: &[(P, P)],
    step: f64,
    reach: f64,
    vgrid: &Grid<u32>,
    vias: &[StitchVia],
    layer: Option<LayerId>,
) -> Value {
    let total: f64 = segs.iter().map(|(a, b)| dist(*a, *b)).sum();
    if total <= 0.0 {
        return Value::Null;
    }
    // For each stitch near the edge, its position along the walk.
    let mut along: BTreeMap<u32, f64> = BTreeMap::new();
    let mut s0 = 0.0;
    let mut covered = 0.0;
    let mut longest = (0.0f64, [0.0, 0.0], [0.0, 0.0]);
    let mut run = 0.0f64;
    let mut run_start = [0.0, 0.0];
    for &(a, b) in segs {
        let l = dist(a, b);
        let n = (l / step).ceil().max(1.0) as usize;
        for k in 0..n {
            let t = (k as f64 + 0.5) / n as f64;
            let p = [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
            let mut hit = false;
            vgrid.query(
                [p[0] - reach, p[1] - reach],
                [p[0] + reach, p[1] + reach],
                |i| {
                    let v = &vias[i as usize];
                    if layer.is_none_or(|l| v.layers.contains(l)) && dist(v.at, p) <= reach {
                        hit = true;
                        let pos = s0 + t * l;
                        along
                            .entry(i)
                            .and_modify(|x| {
                                if dist(v.at, p) < reach * 0.5 {
                                    *x = pos
                                }
                            })
                            .or_insert(pos);
                    }
                },
            );
            let piece = l / n as f64;
            if hit {
                covered += piece;
                if run > longest.0 {
                    longest = (run, run_start, p);
                }
                run = 0.0;
            } else {
                if run == 0.0 {
                    run_start = p;
                }
                run += piece;
                if run > longest.0 {
                    longest = (run, run_start, p);
                }
            }
        }
        s0 += l;
    }
    let mut pos: Vec<f64> = along.values().copied().collect();
    pos.sort_by(f64::total_cmp);
    let mut gaps: Vec<f64> = pos.windows(2).map(|w| w[1] - w[0]).collect();
    if pos.len() >= 2 {
        gaps.push(total - pos[pos.len() - 1] + pos[0]);
    }
    let max_gap = gaps.iter().copied().fold(0.0, f64::max);
    json!({
        "length_mm": r4(total),
        "stitches_within_reach": pos.len(),
        "mean_pitch_mm": if pos.is_empty() { Value::Null } else { json!(r4(total / pos.len() as f64)) },
        "max_pitch_mm": if pos.len() >= 2 { json!(r4(max_gap)) } else { Value::Null },
        "covered_fraction": r4(covered / total),
        "longest_unstitched_mm": r4(longest.0),
        "longest_unstitched_from": [r4(longest.1[0]), r4(longest.1[1])],
        "longest_unstitched_to": [r4(longest.2[0]), r4(longest.2[1])],
    })
}

pub(crate) fn check_stitching_vias(
    l: &Loaded,
    args: &Value,
    out: &Outputs,
) -> Result<CallToolResult, Refusal> {
    let board = l.board();
    let net_name = args.get("net").and_then(Value::as_str).unwrap_or("GND");
    let net = common::net_named(board, net_name, "net")?;
    let step = common::opt_f64(args, "sample_mm", 0.5, 0.05, 5.0)?;
    let reach = common::opt_f64(args, "edge_reach_mm", 2.0, 1e-3, 20.0)?;
    let focus_r = common::opt_f64(args, "focus_radius_mm", 3.0, 1e-3, 50.0)?;
    let grid_mm = common::opt_f64(args, "grid_mm", 5.0, 0.5, 50.0)?;
    let island_area = common::opt_f64(args, "island_area_mm2", 400.0, 0.0, 1e9)?;
    let max_items = opt_count(args, "max_items", 50, 100_000)? as usize;
    let t0 = std::time::Instant::now();

    // The net's pieces by layer.
    let pieces: Vec<usize> = board
        .zones
        .iter()
        .enumerate()
        .filter(|(_, z)| z.net == Some(net) && z.filled)
        .map(|(i, _)| i)
        .collect();
    if pieces.is_empty() {
        return Err(invalid("net", &format!("{net_name} has no filled zone")));
    }
    let with_fill: LayerMask = pieces
        .iter()
        .fold(LayerMask::EMPTY, |m, &i| m.with(board.zones[i].layer));
    let layers = board_inspect::layers_arg(board, args, "layers")?.unwrap_or(with_fill);
    let outline_area: f64 = board
        .outline
        .regions
        .iter()
        .map(|p| p.area2() as f64 / 2e12)
        .sum::<f64>()
        .abs()
        .max(1.0);
    let area_of = |i: usize| rings_area(&super::geom::polygon_rings(&board.zones[i].region));
    let plane_layers = match board_inspect::layers_arg(board, args, "plane_layers")? {
        Some(m) => m,
        None => pieces
            .iter()
            .filter(|&&i| area_of(i) >= outline_area / 4.0)
            .fold(LayerMask::EMPTY, |m, &i| m.with(board.zones[i].layer)),
    };

    // Stitches: the net's vias and through-hole pads.
    let mut vias: Vec<StitchVia> = Vec::new();
    for v in &board.vias {
        if v.net == Some(net) {
            vias.push(StitchVia {
                at: pmm(v.center),
                r: f64::from(v.diameter) / 2e6,
                layers: v.layers,
                uuid: v.uuid.clone(),
                through_hole_pad: None,
            });
        }
    }
    for p in &board.pads {
        if p.net == Some(net) && p.drill.as_ref().is_some_and(|d| d.plated) && p.layers.count() > 1
        {
            let r = p
                .bbox()
                .map(|b| (b.width().min(b.height()) as f64) / 2e6)
                .unwrap_or(0.5);
            vias.push(StitchVia {
                at: pmm(p.center),
                r,
                layers: p.layers,
                uuid: None,
                through_hole_pad: Some(pad_label(board, p)),
            });
        }
    }
    let mut vgrid = Grid::new(2.0);
    for (i, v) in vias.iter().enumerate() {
        vgrid.insert(v.at, v.at, i as u32);
    }
    let mut markers = Vec::new();

    // ── Per piece: via-free circle, edge pitch, ties ──
    let mut regions = Vec::new();
    let mut islands = Vec::new();
    for &zi in &pieces {
        let z = &board.zones[zi];
        if !layers.contains(z.layer) {
            continue;
        }
        let rings = super::geom::polygon_rings(&z.region);
        let pts: Vec<(f64, f64)> = rings[0].iter().map(|p| (p[0], p[1])).collect();
        let clean = unfracture(&pts);
        let area = rings_area(&clean);
        let b = z.region.bbox();
        let (lo, hi) = (pmm(b.min), pmm(b.max));
        // Via-free circle: coarse samples, then a refinement round.
        let mut r = Raster::new(lo, hi, step);
        r.fill(&clean, 2);
        let mut best = (0.0f64, [0.0, 0.0]);
        for j in 0..r.ny {
            for i in 0..r.nx {
                if r.cov[r.idx(i, j)] < 0.5 {
                    continue;
                }
                let c = r.center(i, j);
                let (d, _) = nearest(&vgrid, &vias, c, z.layer, 1e4);
                if d > best.0 {
                    best = (d, c);
                }
            }
        }
        if best.0.is_finite() && best.0 > 0.0 {
            let fine = step / 5.0;
            let c0 = best.1;
            for dj in -5..=5 {
                for di in -5..=5 {
                    let c = [c0[0] + f64::from(di) * fine, c0[1] + f64::from(dj) * fine];
                    if !super::geom::inside_rings(c, &clean) {
                        continue;
                    }
                    let (d, _) = nearest(&vgrid, &vias, c, z.layer, 1e4);
                    if d > best.0 {
                        best = (d, c);
                    }
                }
            }
        }
        let no_via = !best.0.is_finite();
        if !no_via {
            markers.push(marker(
                best.1[0],
                best.1[1],
                "via_free_circle",
                format!("{} r {:.1} mm", board.layers.name(z.layer), best.0),
            ));
        }
        // Edge walk of the outline ring (the largest).
        let outer = clean.iter().max_by(|a, b| {
            super::geom::ring_area2(a)
                .abs()
                .total_cmp(&super::geom::ring_area2(b).abs())
        });
        let edge = outer.map(|ring| {
            let segs: Vec<(P, P)> = (0..ring.len())
                .map(|k| (ring[k], ring[(k + 1) % ring.len()]))
                .collect();
            edge_walk(&segs, step, reach, &vgrid, &vias, Some(z.layer))
        });
        if let Some(e) = &edge {
            if let (Some(a), Some(len)) = (
                e["longest_unstitched_from"].as_array(),
                e["longest_unstitched_mm"].as_f64(),
            ) {
                if len > 4.0 * reach {
                    markers.push(marker(
                        a[0].as_f64().unwrap_or(0.0),
                        a[1].as_f64().unwrap_or(0.0),
                        "unstitched_edge",
                        format!("{:.1} mm unstitched", len),
                    ));
                }
            }
        }
        // Stitches touching the piece, and those that also reach a plane.
        let ix = IndexedPolygon::new(z.region.clone());
        let mut touching = Vec::new();
        for (vi, v) in vias.iter().enumerate() {
            if !v.layers.contains(z.layer) {
                continue;
            }
            if v.at[0] < lo[0] - v.r
                || v.at[0] > hi[0] + v.r
                || v.at[1] < lo[1] - v.r
                || v.at[1] > hi[1] + v.r
            {
                continue;
            }
            let Some(c) = Point::from_mm(v.at[0], v.at[1]) else {
                continue;
            };
            let s = Shape::Circle(Circle::new(c, (v.r * 1e6) as i32));
            if ix.touches(&s) {
                touching.push(vi);
            }
        }
        let ties: Vec<usize> = touching
            .iter()
            .copied()
            .filter(|&vi| {
                vias[vi].layers.intersects(
                    plane_layers.intersection(LayerMask::single(z.layer).union(plane_layers)),
                ) && vias[vi]
                    .layers
                    .intersection(plane_layers)
                    .iter()
                    .any(|pl| pl != z.layer)
            })
            .collect();
        let row = json!({
            "layer": board.layers.name(z.layer),
            "zone": z.zone_name,
            "area_mm2": r4(area),
            "stitches": touching.len(),
            "density_per_cm2": r4(touching.len() as f64 / (area / 100.0).max(1e-9)),
            "via_free_circle": if no_via { Value::Null } else { json!({ "center": [r4(best.1[0]), r4(best.1[1])], "radius_mm": r4(best.0) }) },
            "edge": edge,
            "ties_to_planes": ties.len(),
        });
        if area < island_area || !plane_layers.contains(z.layer) {
            let tie_list: Vec<Value> = ties
                .iter()
                .map(|&vi| json!({ "at": [r4(vias[vi].at[0]), r4(vias[vi].at[1])], "uuid": vias[vi].uuid, "pad": vias[vi].through_hole_pad, "planes": board_inspect::layer_names(board, vias[vi].layers.intersection(plane_layers)) }))
                .collect();
            if ties.is_empty() {
                let c = clean
                    .first()
                    .and_then(|r| r.first())
                    .copied()
                    .unwrap_or([0.0, 0.0]);
                markers.push(marker(
                    c[0],
                    c[1],
                    "untied_island",
                    format!("{} {:.1} mm²", board.layers.name(z.layer), area),
                ));
            }
            islands.push(merged(row.clone(), json!({ "ties": tie_list })));
        }
        regions.push(row);
    }
    regions.sort_by(|a, b| {
        b["via_free_circle"]["radius_mm"]
            .as_f64()
            .unwrap_or(0.0)
            .total_cmp(&a["via_free_circle"]["radius_mm"].as_f64().unwrap_or(0.0))
    });

    // ── Board edge ──
    let mut board_edge = Vec::new();
    for layer in layers.iter() {
        let segs: Vec<(P, P)> = board
            .outline
            .edges
            .iter()
            .map(|s| (pmm(s.a), pmm(s.b)))
            .collect();
        let e = edge_walk(&segs, step, reach, &vgrid, &vias, Some(layer));
        board_edge.push(merged(json!({ "layer": board.layers.name(layer) }), e));
    }

    // ── Focus footprints ──
    let mut focus = Vec::new();
    if let Some(refs) = opt_strings(args, "focus_footprints")? {
        for r in &refs {
            let fps: Vec<_> = board
                .footprints
                .iter()
                .filter(|f| f.reference.as_deref() == Some(r.as_str()))
                .collect();
            if fps.is_empty() {
                return Err(invalid(
                    "focus_footprints",
                    &format!("{r} is not on the board"),
                ));
            }
            let mut near: BTreeMap<usize, (f64, String)> = BTreeMap::new();
            let mut pad_rows = Vec::new();
            for fp in &fps {
                for pi in fp.pad_range.0 as usize..fp.pad_range.1 as usize {
                    let p = &board.pads[pi];
                    let c = pmm(p.center);
                    let pr = p
                        .bbox()
                        .map(|b| dist(pmm(b.min), pmm(b.max)) / 2.0)
                        .unwrap_or(1.0);
                    let mut own = 0usize;
                    vgrid.query(
                        [c[0] - pr - focus_r - 2.0, c[1] - pr - focus_r - 2.0],
                        [c[0] + pr + focus_r + 2.0, c[1] + pr + focus_r + 2.0],
                        |vi| {
                            let v = &vias[vi as usize];
                            if v.through_hole_pad.as_deref() == Some(pad_label(board, p).as_str()) {
                                return;
                            }
                            let Some(vc) = Point::from_mm(v.at[0], v.at[1]) else {
                                return;
                            };
                            let s = Shape::Circle(Circle::new(vc, (v.r * 1e6) as i32));
                            let gap = p
                                .shapes
                                .iter()
                                .map(|sh| konnect_geom::gap_nm(sh, &s) / 1e6)
                                .fold(f64::INFINITY, f64::min);
                            if gap <= focus_r {
                                own += 1;
                                let label = pad_label(board, p);
                                near.entry(vi as usize)
                                    .and_modify(|e| {
                                        if gap < e.0 {
                                            *e = (gap, label.clone())
                                        }
                                    })
                                    .or_insert((gap, label));
                            }
                        },
                    );
                    pad_rows.push(json!({
                        "pad": pad_label(board, p),
                        "net": p.net.map(|n| board.net(n).name.clone()),
                        "stitches_within_radius": own,
                    }));
                }
            }
            for (&vi, (gap, pad)) in &near {
                let _ = (gap, pad);
                markers.push(marker(
                    vias[vi].at[0],
                    vias[vi].at[1],
                    "focus_stitch",
                    r.clone(),
                ));
            }
            focus.push(json!({
                "footprint": r,
                "radius_mm": focus_r,
                "stitches": near.len(),
                "vias": near.iter().filter(|(vi, _)| vias[**vi].through_hole_pad.is_none()).count(),
                "list": near.iter().map(|(&vi, (gap, pad))| json!({
                    "at": [r4(vias[vi].at[0]), r4(vias[vi].at[1])],
                    "diameter_mm": r4(vias[vi].r * 2.0),
                    "uuid": vias[vi].uuid,
                    "through_hole_pad": vias[vi].through_hole_pad,
                    "nearest_pad": pad,
                    "gap_mm": r4(*gap),
                })).collect::<Vec<_>>(),
                "pads": pad_rows,
            }));
        }
    }

    // ── Focus nets: each via's nearest stitch ──
    let mut return_paths = Vec::new();
    if let Some(p) = opt_strings(args, "focus_nets")? {
        let set = common::nets_named(board, &p, "focus_nets")?;
        let mut per_net: BTreeMap<String, Vec<f64>> = BTreeMap::new();
        let mut worst: Vec<(f64, P, String)> = Vec::new();
        for v in &board.vias {
            let Some(n) = v.net.filter(|n| set.contains(n)) else {
                continue;
            };
            let c = pmm(v.center);
            let mut best = f64::INFINITY;
            vgrid.query(
                [c[0] - 10.0, c[1] - 10.0],
                [c[0] + 10.0, c[1] + 10.0],
                |i| {
                    let s = &vias[i as usize];
                    if s.layers.intersects(v.layers) {
                        best = best.min(dist(s.at, c));
                    }
                },
            );
            let name = board.net(n).name.clone();
            per_net.entry(name.clone()).or_default().push(best);
            worst.push((best, c, name));
        }
        worst.sort_by(|a, b| b.0.total_cmp(&a.0));
        for (d, c, n) in worst.iter().take(max_items) {
            markers.push(marker(c[0], c[1], "return_via", format!("{n} {:.2} mm", d)));
        }
        for (n, ds) in per_net {
            let finite: Vec<f64> = ds.iter().copied().filter(|d| d.is_finite()).collect();
            return_paths.push(json!({
                "net": n,
                "signal_vias": ds.len(),
                "max_distance_mm": finite.iter().copied().fold(None, |m: Option<f64>, d| Some(m.map_or(d, |m| m.max(d)))).map(r4),
                "mean_distance_mm": if finite.is_empty() { Value::Null } else { json!(r4(finite.iter().sum::<f64>() / finite.len() as f64)) },
                "without_stitch_within_10mm": ds.iter().filter(|d| !d.is_finite()).count(),
            }));
        }
    }

    // ── Heat map ──
    let mut heat: BTreeMap<(i64, i64), usize> = BTreeMap::new();
    for v in &vias {
        if v.layers.intersects(layers) && v.through_hole_pad.is_none() {
            *heat
                .entry((
                    (v.at[0] / grid_mm).floor() as i64,
                    (v.at[1] / grid_mm).floor() as i64,
                ))
                .or_default() += 1;
        }
    }
    // Cells over a pour of the net with no stitch.
    let mut empty_cells = BTreeSet::new();
    if let Some(b) = board.outline.bbox {
        let (lo, hi) = (pmm(b.min), pmm(b.max));
        let mut covered = Raster::new(lo, hi, grid_mm);
        for &zi in &pieces {
            if layers.contains(board.zones[zi].layer) {
                covered.fill(&super::geom::polygon_rings(&board.zones[zi].region), 2);
            }
        }
        for j in 0..covered.ny {
            for i in 0..covered.nx {
                if covered.cov[covered.idx(i, j)] < 0.5 {
                    continue;
                }
                let c = covered.center(i, j);
                let key = (
                    (c[0] / grid_mm).floor() as i64,
                    (c[1] / grid_mm).floor() as i64,
                );
                if !heat.contains_key(&key) {
                    empty_cells.insert(key);
                    markers.push(marker(
                        c[0],
                        c[1],
                        "no_stitch_cell",
                        format!("{grid_mm} mm cell without a stitch"),
                    ));
                }
            }
        }
    }
    let heat_rows: Vec<Value> = heat
        .iter()
        .map(|(&(x, y), &n)| json!({ "x1": x as f64 * grid_mm, "y1": y as f64 * grid_mm, "x2": (x + 1) as f64 * grid_mm, "y2": (y + 1) as f64 * grid_mm, "vias": n }))
        .collect();

    let methods = json!({
        "stitches": "the net's vias plus its plated through-hole pads; a stitch serves a layer it spans",
        "via_free_circle": "the largest distance from any point of the pour to the nearest stitch serving its layer: sampled every sample_mm (coverage ≥ 0.5), refined at sample_mm/5 round the best",
        "edge": "the pour's outline (and the board edge) walked every sample_mm; a point is stitched when a stitch lies within edge_reach_mm; pitch = gaps between successive stitches' positions along the walk",
        "ties": "stitches touching a piece (exact) that also span a plane layer",
        "focus": "stitches whose edge is within focus_radius_mm of a focus footprint pad's copper (approximate gap), the footprint's own through-hole pads excluded",
        "heat_map": "vias per grid_mm cell; cells over the pours with none are marked",
    });
    let full = merged(
        common::provenance(l),
        json!({
            "net": net_name,
            "layers": board_inspect::layer_names(board, layers),
            "plane_layers": board_inspect::layer_names(board, plane_layers),
            "stitches": vias.len(),
            "regions": regions,
            "islands": islands,
            "board_edge": board_edge,
            "focus_footprints": focus,
            "focus_nets": return_paths,
            "heat_map": { "grid_mm": grid_mm, "cells": heat_rows, "pour_cells_without_stitch": empty_cells.len() },
            "methods": methods,
        }),
    );
    let files = out.write(&full, &markers)?;
    Ok(CallToolResult::json(&merged(
        common::provenance(l),
        json!({
            "net": net_name,
            "layers": full["layers"],
            "plane_layers": full["plane_layers"],
            "stitches": vias.len(),
            "largest_via_free_circles": super::hygiene::take(&full["regions"], max_items.min(20)).as_array().map(|a| a.iter().map(|r| json!({ "layer": r["layer"], "zone": r["zone"], "via_free_circle": r["via_free_circle"], "edge_longest_unstitched_mm": r["edge"]["longest_unstitched_mm"], "edge_max_pitch_mm": r["edge"]["max_pitch_mm"] })).collect::<Vec<_>>()),
            "islands": super::hygiene::take(&full["islands"], max_items),
            "untied_islands": full["islands"].as_array().map_or(0, |a| a.iter().filter(|i| i["ties_to_planes"] == 0).count()),
            "board_edge": full["board_edge"],
            "focus_footprints": full["focus_footprints"],
            "focus_nets": full["focus_nets"],
            "pour_cells_without_stitch": empty_cells.len(),
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

    #[test]
    fn a_pour_with_two_vias_reports_its_via_free_circle_and_focus_count() {
        let board = ts::board(&[
            ts::zone("G", "F.Cu", "GND", 0.0, 0.0, 20.0, 10.0),
            ts::zone("GB", "B.Cu", "GND", 0.0, 0.0, 20.0, 10.0),
            ts::via(2.0, 5.0, 0.6, 0.3, "GND"),
            ts::via(4.0, 5.0, 0.6, 0.3, "GND"),
            ts::pad("D1", 3.0, 7.0, 1.0, 1.0, "GND"),
        ]);
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("t.kicad_pcb");
        std::fs::write(&path, board).unwrap();
        let l = common::load_at(&path).unwrap();
        let out = Outputs {
            output: None,
            markers: None,
        };
        let r = check_stitching_vias(&l, &json!({ "board": path, "net": "GND", "focus_footprints": ["D1"], "focus_radius_mm": 2.0, "sample_mm": 0.25 }), &out).unwrap();
        let b: Value = match &r.content[0] {
            crate::mcp::protocol::ToolContent::Text { text } => serde_json::from_str(text).unwrap(),
            _ => panic!(),
        };
        // The farthest point of the pour from both vias is the far corner (20, 0)/(20, 10):
        // distance from (4, 5) = sqrt(16² + 5²) = 16.76.
        let c = &b["largest_via_free_circles"][0]["via_free_circle"];
        assert!(
            (c["radius_mm"].as_f64().unwrap() - 16.763).abs() < 0.2,
            "{b}"
        );
        assert_eq!(b["focus_footprints"][0]["stitches"], 2, "{b}");
    }
}
