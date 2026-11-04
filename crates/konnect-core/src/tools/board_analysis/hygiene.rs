//! K7 `check_zone_hygiene`: overlaps, wrapped foreign copper, slivers and
//! necks, and what each fill piece connects.

use super::common::{self, marker, merged, r4, Loaded, Outputs};
use super::geom::{pmm, rings_area, unfracture, Raster, P};
use super::necks::find_necks;
use crate::mcp::protocol::CallToolResult;
use crate::tools::board_inspect::{self, pad_label, zone_region};
use crate::tools::route_native::{opt_count, opt_strings, Refusal};
use konnect_geom::{ArcSide, Circle, IndexedPolygon, Point, Polygon, Segment, Shape};
use konnect_route::api::{LayerMask, NetId};
use konnect_sexp::board::ZoneRecord;
use serde_json::{json, Value};

pub(crate) fn schema() -> Value {
    common::schema(
        json!({
            "nets": { "type": "array", "items": { "type": "string", "minLength": 1 }, "minItems": 1, "description": "Only zones of these nets (names or * ? patterns)" },
            "zones": { "type": "array", "items": { "type": "string", "minLength": 1 }, "minItems": 1, "description": "Only zones with these names (or * ? patterns)" },
            "layers": { "type": "array", "items": { "type": "string", "minLength": 1 }, "minItems": 1, "description": "Only these copper layers" },
            "sliver_width_mm": { "type": "number", "exclusiveMinimum": 0, "maximum": 5, "default": 0.15, "description": "Report fill copper narrower than this: slivers and necks (the narrowest chord across the copper)" },
            "wrap_skip_zones": { "type": "array", "items": { "type": "string", "minLength": 1 }, "description": "Zones (names or nets, * ? patterns) whose enclosed foreign pads and vias are only counted, not listed: planes enclose every via by design" },
            "check_overlaps": { "type": "boolean", "default": true }
        }),
        &["board"],
    )
}

struct ZoneInfo<'a> {
    rec: &'a ZoneRecord,
    name: String,
    net: Option<NetId>,
    layers: LayerMask,
    outline: Option<Polygon>,
    indexed: Option<IndexedPolygon>,
}

fn zone_label(z: &ZoneRecord, i: usize) -> String {
    z.geometry.name.clone().unwrap_or_else(|| {
        format!(
            "zone#{i} ({})",
            z.geometry.net.as_deref().unwrap_or("no net")
        )
    })
}

fn shape_box(s: &Shape) -> (P, P) {
    let b = s.bbox();
    (pmm(b.min), pmm(b.max))
}

fn boxes_meet(a: (P, P), b: (P, P)) -> bool {
    a.1[0] >= b.0[0] && a.0[0] <= b.1[0] && a.1[1] >= b.0[1] && a.0[1] <= b.1[1]
}

/// Intersection area of two polygons, mm², by rasterizing both on a common
/// grid fine enough for the smaller.
fn overlap_area(a: &Polygon, b: &Polygon) -> (f64, Option<P>) {
    let (ba, bb) = (a.bbox(), b.bbox());
    let lo = [
        pmm(ba.min)[0].max(pmm(bb.min)[0]),
        pmm(ba.min)[1].max(pmm(bb.min)[1]),
    ];
    let hi = [
        pmm(ba.max)[0].min(pmm(bb.max)[0]),
        pmm(ba.max)[1].min(pmm(bb.max)[1]),
    ];
    if hi[0] <= lo[0] || hi[1] <= lo[1] {
        return (0.0, None);
    }
    let span = (hi[0] - lo[0]).max(hi[1] - lo[1]);
    let h = (span / 400.0).clamp(0.01, 0.5);
    let mut ra = Raster::new(lo, hi, h);
    let mut rb = Raster::new(lo, hi, h);
    ra.fill(&super::geom::polygon_rings(a), 2);
    rb.fill(&super::geom::polygon_rings(b), 2);
    let mut area = 0.0;
    let mut best: Option<(f32, P)> = None;
    for j in 0..ra.ny {
        for i in 0..ra.nx {
            let k = ra.idx(i, j);
            let c = ra.cov[k].min(rb.cov[k]);
            if c > 0.0 {
                area += f64::from(c) * h * h;
                if best.is_none_or(|(bc, _)| c > bc) {
                    best = Some((c, ra.center(i, j)));
                }
            }
        }
    }
    (area, best.map(|b| b.1))
}

pub(crate) fn check_zone_hygiene(
    l: &Loaded,
    args: &Value,
    out: &Outputs,
) -> Result<CallToolResult, Refusal> {
    let board = l.board();
    let nets = match opt_strings(args, "nets")? {
        Some(p) => Some(common::nets_named(board, &p, "nets")?),
        None => None,
    };
    let zone_patterns = opt_strings(args, "zones")?;
    let layers = board_inspect::layers_arg(board, args, "layers")?;
    let sliver = common::opt_f64(args, "sliver_width_mm", 0.15, 1e-4, 5.0)?;
    let skip = opt_strings(args, "wrap_skip_zones")?.unwrap_or_default();
    let check_overlaps = crate::tools::route_native::opt_bool(args, "check_overlaps", true)?;
    let max_items = opt_count(args, "max_items", 50, 100_000)? as usize;
    let t0 = std::time::Instant::now();

    let mut zones: Vec<ZoneInfo> = Vec::new();
    for (i, z) in l.saved.zones.iter().enumerate() {
        if z.geometry.keepout.is_some() {
            continue;
        }
        let name = zone_label(z, i);
        let net = z.geometry.net.as_deref().and_then(|n| board.net_by_name(n));
        let zl = board_inspect::zone_layers(board, z);
        if let Some(set) = &nets {
            if !net.is_some_and(|n| set.contains(&n)) {
                continue;
            }
        }
        if let Some(p) = &zone_patterns {
            if !p.iter().any(|p| common::glob(p, &name)) {
                continue;
            }
        }
        if let Some(m) = layers {
            if !m.intersects(zl) {
                continue;
            }
        }
        let outline = zone_region(z, ArcSide::Inscribed);
        let indexed = outline.clone().map(IndexedPolygon::new);
        zones.push(ZoneInfo {
            rec: z,
            name,
            net,
            layers: zl,
            outline,
            indexed,
        });
    }
    if let Some(p) = &zone_patterns {
        for pat in p {
            if !zones.iter().any(|z| common::glob(pat, &z.name)) {
                return Err(crate::tools::route_native::invalid(
                    "zones",
                    &format!("no zone of the board matches '{pat}'"),
                ));
            }
        }
    }
    let net_name = |n: Option<NetId>| n.map(|n| board.net(n).name.clone());
    let mut markers: Vec<Value> = Vec::new();

    // ── Overlaps and priority conflicts ──
    let mut overlaps = Vec::new();
    let mut conflicts = Vec::new();
    if check_overlaps {
        for a in 0..zones.len() {
            for b in a + 1..zones.len() {
                let (za, zb) = (&zones[a], &zones[b]);
                let shared = za.layers.intersection(zb.layers);
                if shared.is_empty() {
                    continue;
                }
                let (Some(pa), Some(pb)) = (&za.outline, &zb.outline) else {
                    continue;
                };
                let (ba, bb) = (pa.bbox(), pb.bbox());
                if !ba.intersects(&bb) {
                    continue;
                }
                let (area, at) = overlap_area(pa, pb);
                if area < 1e-4 {
                    continue;
                }
                let same = za.net.is_some() && za.net == zb.net;
                let equal = za.rec.geometry.priority == zb.rec.geometry.priority;
                let row = json!({
                    "zones": [za.name, zb.name],
                    "nets": [net_name(za.net), net_name(zb.net)],
                    "priorities": [za.rec.geometry.priority, zb.rec.geometry.priority],
                    "layers": board_inspect::layer_names(board, shared),
                    "same_net": same,
                    "equal_priority": equal,
                    "overlap_area_mm2": r4(area),
                    "at": at.map(|p| [r4(p[0]), r4(p[1])]),
                    "effect": if same {
                        if equal { "same-net overlap at equal priority: KiCad's DRC reports it; give one a higher priority or merge them" } else { "same-net overlap: the higher priority fills first, the other fills around it" }
                    } else if equal {
                        "different nets at equal priority: which fills the overlap depends on fill order"
                    } else {
                        "different nets: the higher priority wins the overlap"
                    },
                });
                if let Some(p) = at {
                    markers.push(marker(
                        p[0],
                        p[1],
                        "zone_overlap",
                        format!("{} / {}", za.name, zb.name),
                    ));
                }
                if equal && !same {
                    conflicts.push(row.clone());
                }
                overlaps.push(row);
            }
        }
    }

    // ── Foreign pads and vias inside each zone's outline ──
    let mut wraps = Vec::new();
    for z in &zones {
        let Some(ix) = &z.indexed else { continue };
        let bb = ix.bbox();
        let mut pads = Vec::new();
        let mut vias = Vec::new();
        let reach = |b: konnect_geom::BBox| b.intersects(&bb);
        for p in &board.pads {
            if p.net == z.net || !p.layers.intersects(z.layers) {
                continue;
            }
            if !p.bbox().is_some_and(reach) {
                continue;
            }
            let inside = ix.containment(p.center) == konnect_geom::Containment::Inside;
            // The outline overlaps the pad's copper: the fill clears round it.
            if inside || p.shapes.iter().any(|s| ix.touches(s)) {
                pads.push(json!({
                    "pad": pad_label(board, p),
                    "net": net_name(p.net),
                    "at": [r4(pmm(p.center)[0]), r4(pmm(p.center)[1])],
                    "enclosure": if inside { "inside" } else { "partial" },
                }));
            }
        }
        for v in &board.vias {
            if v.net == z.net || !v.layers.intersects(z.layers) {
                continue;
            }
            let s = Shape::Circle(Circle::new(v.center, v.diameter / 2));
            if !reach(s.bbox()) {
                continue;
            }
            let inside = ix.containment(v.center) == konnect_geom::Containment::Inside;
            if inside || ix.touches(&s) {
                vias.push(json!({
                    "net": net_name(v.net),
                    "at": [r4(pmm(v.center)[0]), r4(pmm(v.center)[1])],
                    "uuid": v.uuid,
                    "enclosure": if inside { "inside" } else { "partial" },
                }));
            }
        }
        if pads.is_empty() && vias.is_empty() {
            continue;
        }
        let skipped = skip.iter().any(|s| {
            common::glob(s, &z.name) || net_name(z.net).is_some_and(|n| common::glob(s, &n))
        });
        if !skipped {
            for p in pads.iter().chain(vias.iter()) {
                let at = &p["at"];
                markers.push(marker(
                    at[0].as_f64().unwrap_or(0.0),
                    at[1].as_f64().unwrap_or(0.0),
                    "zone_wraps_foreign",
                    format!(
                        "{} wraps {}",
                        z.name,
                        p.get("pad").and_then(Value::as_str).unwrap_or("via")
                    ),
                ));
            }
        }
        wraps.push(json!({
            "zone": z.name,
            "net": net_name(z.net),
            "layers": board_inspect::layer_names(board, z.layers),
            "foreign_pads": pads.len(),
            "foreign_vias": vias.len(),
            "listed": !skipped,
            "pads": if skipped { Value::Null } else { json!(pads) },
            "vias": if skipped { Value::Null } else { json!(vias) },
        }));
    }
    wraps.sort_by_key(|w| {
        (
            !w["listed"].as_bool().unwrap_or(false),
            std::cmp::Reverse(w["foreign_pads"].as_u64().unwrap_or(0)),
        )
    });

    // ── Fill pieces: what each connects, slivers and necks ──
    let mut pieces_out = Vec::new();
    let mut dead = Vec::new();
    let mut slivers = Vec::new();
    let spacing = (4.0 * sliver).max(1.0);
    for z in &zones {
        let mut rows = Vec::new();
        for (fi, fill) in z.rec.geometry.fills.iter().enumerate() {
            let Some(layer) = board.layers.by_name(&fill.layer) else {
                continue;
            };
            if layers.is_some_and(|m| !m.contains(layer)) {
                continue;
            }
            let rings = unfracture(&fill.points);
            let area = rings_area(&rings);
            let poly = fill
                .points
                .iter()
                .map(|&(x, y)| Point::from_mm(x, y))
                .collect::<Option<Vec<_>>>()
                .and_then(|r| Polygon::new(r, vec![]));
            let mut pads = Vec::new();
            let (mut vias, mut tracks) = (0usize, 0usize);
            if let (Some(poly), Some(net)) = (poly, z.net) {
                let pb = {
                    let b = poly.bbox();
                    (pmm(b.min), pmm(b.max))
                };
                let ix = IndexedPolygon::new(poly);
                for p in board.pads_of_net(net) {
                    if !p.layers.contains(layer) {
                        continue;
                    }
                    if p.shapes
                        .iter()
                        .any(|s| boxes_meet(shape_box(s), pb) && ix.touches(s))
                    {
                        pads.push(pad_label(board, p));
                    }
                }
                for v in &board.vias {
                    if v.net != Some(net) || !v.layers.contains(layer) {
                        continue;
                    }
                    let s = Shape::Circle(Circle::new(v.center, v.diameter / 2));
                    if boxes_meet(shape_box(&s), pb) && ix.touches(&s) {
                        vias += 1;
                    }
                }
                for t in &board.tracks {
                    if t.net != Some(net) || t.layer != layer {
                        continue;
                    }
                    let s = Shape::Segment(Segment::new(t.seg.a, t.seg.b, t.seg.width));
                    if boxes_meet(shape_box(&s), pb) && ix.touches(&s) {
                        tracks += 1;
                    }
                }
            }
            let necks = find_necks(&rings, sliver, spacing);
            for n in &necks {
                let at = n.at();
                markers.push(marker(
                    at[0],
                    at[1],
                    if n.kind() == "neck" {
                        "fill_neck"
                    } else {
                        "fill_sliver"
                    },
                    format!("{} {:.3} mm", z.name, n.width),
                ));
                slivers.push(json!({
                    "kind": n.kind(),
                    "smaller_side_perimeter_mm": n.lobe.map(r4),
                    "zone": z.name,
                    "net": net_name(z.net),
                    "layer": fill.layer,
                    "piece": fi,
                    "width_mm": r4(n.width),
                    "at": [r4(at[0]), r4(at[1])],
                    "chord": [[r4(n.a[0]), r4(n.a[1])], [r4(n.b[0]), r4(n.b[1])]],
                }));
            }
            let connects_nothing = pads.is_empty() && vias == 0 && tracks == 0;
            let first = fill.points.first().copied().unwrap_or((0.0, 0.0));
            if connects_nothing {
                markers.push(marker(
                    first.0,
                    first.1,
                    "dead_fill_piece",
                    format!("{} piece {fi}", z.name),
                ));
                dead.push(json!({ "zone": z.name, "layer": fill.layer, "piece": fi, "area_mm2": r4(area), "at": [r4(first.0), r4(first.1)] }));
            }
            rows.push(json!({
                "piece": fi,
                "layer": fill.layer,
                "area_mm2": r4(area),
                "kicad_island": fill.island,
                "pads": pads,
                "vias": vias,
                "tracks": tracks,
                "connects_nothing": connects_nothing,
                "narrowest_mm": necks.first().map(|n| r4(n.width)),
                "narrowest_neck_mm": necks.iter().find(|n| n.kind() == "neck").map(|n| r4(n.width)),
                "vertex": [r4(first.0), r4(first.1)],
            }));
        }
        pieces_out.push(json!({
            "zone": z.name,
            "net": net_name(z.net),
            "priority": z.rec.geometry.priority,
            "layers": board_inspect::layer_names(board, z.layers),
            "min_thickness_mm": z.rec.min_thickness,
            "filled": !z.rec.geometry.fills.is_empty(),
            "piece_count": rows.len(),
            "pieces": rows,
        }));
    }
    slivers.sort_by(|a, b| {
        a["width_mm"]
            .as_f64()
            .unwrap_or(0.0)
            .total_cmp(&b["width_mm"].as_f64().unwrap_or(0.0))
    });

    let methods = json!({
        "overlap": "authored outlines rasterized on a shared grid (step = larger side / 400, 0.01–0.5 mm); area is the coverage minimum summed",
        "wraps": "a foreign pad or via with copper on the zone's layers that the zone outline (less its authored holes) encloses (centre inside) or overlaps (partial): the fill clears round it",
        "slivers": "kind neck: the smaller side is at least 1 mm round (a constriction current must pass); kind sliver: a spike or flap whose smaller side is shorter. Both are closest-point chords between fill edges that cross copper and are far apart along the boundary (more than 1.6× the chord), clustered to their minimum; bridges of KiCad's fractured fills are removed first",
        "pieces": "each saved fill polygon with the same-net pads, vias and tracks whose copper touches it on its layer (exact)",
    });
    let full = merged(
        common::provenance(l),
        json!({
            "sliver_width_mm": sliver,
            "zones_checked": zones.len(),
            "overlaps": overlaps,
            "priority_conflicts": conflicts,
            "wraps": wraps,
            "slivers": slivers,
            "dead_pieces": dead,
            "zones": pieces_out,
            "methods": methods,
        }),
    );
    let files = out.write(&full, &markers)?;
    let summary = merged(
        common::provenance(l),
        json!({
            "sliver_width_mm": sliver,
            "zones_checked": zones.len(),
            "counts": {
                "overlaps": full["overlaps"].as_array().map_or(0, Vec::len),
                "same_net_overlaps": full["overlaps"].as_array().map_or(0, |a| a.iter().filter(|o| o["same_net"] == true).count()),
                "priority_conflicts": full["priority_conflicts"].as_array().map_or(0, Vec::len),
                "zones_wrapping_foreign_copper": full["wraps"].as_array().map_or(0, Vec::len),
                "slivers": full["slivers"].as_array().map_or(0, |a| a.iter().filter(|s| s["kind"] == "sliver").count()),
                "necks": full["slivers"].as_array().map_or(0, |a| a.iter().filter(|s| s["kind"] == "neck").count()),
                "dead_pieces": full["dead_pieces"].as_array().map_or(0, Vec::len),
                "fill_pieces": full["zones"].as_array().map_or(0, |a| a.iter().map(|z| z["piece_count"].as_u64().unwrap_or(0)).sum::<u64>() as usize),
            },
            "overlaps": take(&full["overlaps"], max_items),
            "priority_conflicts": take(&full["priority_conflicts"], max_items),
            "wraps": take(&full["wraps"], max_items),
            "slivers": take(&full["slivers"], max_items),
            "dead_pieces": take(&full["dead_pieces"], max_items),
            "multi_piece_zones": full["zones"].as_array().map(|a| a.iter().filter(|z| z["piece_count"].as_u64().unwrap_or(0) > 1).take(max_items).map(|z| json!({
                "zone": z["zone"], "net": z["net"], "pieces": z["piece_count"],
            })).collect::<Vec<_>>()),
            "files": files,
            "methods": full["methods"],
            "elapsed_ms": t0.elapsed().as_millis() as u64,
        }),
    );
    Ok(CallToolResult::json(&summary))
}

pub(crate) fn take(v: &Value, n: usize) -> Value {
    json!(v
        .as_array()
        .map(|a| a.iter().take(n).cloned().collect::<Vec<_>>())
        .unwrap_or_default())
}

#[cfg(test)]
mod tests {
    use super::super::tests_support as ts;
    use super::*;

    /// Two same-net zones overlapping at equal priority, one whose outline
    /// runs over a foreign pad, and a two-piece fill with a dead piece.
    #[test]
    fn overlaps_wraps_and_pieces_are_reported() {
        let board = ts::board(&[
            ts::zone("A", "F.Cu", "P", 0.0, 0.0, 10.0, 10.0),
            ts::zone("B", "F.Cu", "P", 8.0, 0.0, 18.0, 10.0),
            ts::pad("J1", 5.0, 5.0, 1.0, 1.0, "P"),
            ts::pad("D1", 9.0, 10.2, 1.0, 1.0, "GND"),
            "\t(zone (net \"P\") (layer \"B.Cu\") (name \"TWO\") (connect_pads yes (clearance 0.2)) (min_thickness 0.2) (fill yes)\n\t\t(polygon (pts (xy 0 20) (xy 20 20) (xy 20 30) (xy 0 30)))\n\t\t(filled_polygon (layer \"B.Cu\") (pts (xy 0 20) (xy 5 20) (xy 5 30) (xy 0 30)))\n\t\t(filled_polygon (layer \"B.Cu\") (pts (xy 10 20) (xy 20 20) (xy 20 30) (xy 10 30)))\n\t)\n".to_string(),
            ts::via(2.0, 25.0, 0.6, 0.3, "P"),
        ]);
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("t.kicad_pcb");
        std::fs::write(&path, board).unwrap();
        let l = common::load_at(&path).unwrap();
        let out = Outputs {
            output: None,
            markers: None,
        };
        let r = check_zone_hygiene(&l, &json!({ "board": path, "max_items": 100 }), &out).unwrap();
        let b: Value = match &r.content[0] {
            crate::mcp::protocol::ToolContent::Text { text } => serde_json::from_str(text).unwrap(),
            _ => panic!(),
        };
        let o = &b["overlaps"][0];
        assert_eq!(o["same_net"], true, "{b}");
        assert_eq!(o["equal_priority"], true);
        assert!(
            (o["overlap_area_mm2"].as_f64().unwrap() - 20.0).abs() < 0.5,
            "{o}"
        );
        let wrap = b["wraps"]
            .as_array()
            .unwrap()
            .iter()
            .find(|w| w["zone"] == "A")
            .unwrap();
        assert_eq!(wrap["pads"][0]["pad"], "D1.1");
        assert_eq!(wrap["pads"][0]["enclosure"], "partial");
        // B touches no pad, via or track; TWO's right piece neither.
        assert_eq!(b["counts"]["dead_pieces"], 2, "{b}");
        let dead: Vec<(&str, u64)> = b["dead_pieces"]
            .as_array()
            .unwrap()
            .iter()
            .map(|d| (d["zone"].as_str().unwrap(), d["piece"].as_u64().unwrap()))
            .collect();
        assert_eq!(dead, vec![("B", 0), ("TWO", 1)]);
        assert_eq!(b["multi_piece_zones"][0]["pieces"], 2);
    }
}
