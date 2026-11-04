//! K9 `measure_net_lengths`: routed length of each net (tracks, arcs and the
//! via barrels they use), optionally pad to pad, with the skew inside each
//! pair and the spread inside each group.

use super::common::{self, marker, merged, r4, Loaded, Outputs};
use super::geom::{dist, pmm};
use super::network::{Network, Options};
use crate::mcp::protocol::CallToolResult;
use crate::tools::route_native::{invalid, opt_bool, opt_count, Refusal};
use konnect_route::api::{LayerMask, NetId};
use serde_json::{json, Value};
use std::collections::BTreeMap;

pub(crate) fn schema() -> Value {
    common::schema(
        json!({
            "groups": { "type": "array", "minItems": 1, "items": { "type": "object", "properties": {
                "name": { "type": "string" },
                "nets": { "type": "array", "items": { "type": "string" }, "minItems": 1, "description": "Net names or * ? patterns" },
                "kind": { "type": "string", "enum": ["pair", "bus"], "default": "bus" },
                "target_length_mm": { "type": "number", "exclusiveMinimum": 0 },
                "tolerance_mm": { "type": "number", "minimum": 0, "description": "Allowed |length − target|" },
                "max_skew_mm": { "type": "number", "minimum": 0, "description": "Allowed spread (longest − shortest) in the group" },
                "endpoints": { "type": "object", "additionalProperties": { "type": "array", "items": { "type": "string" }, "minItems": 2, "maxItems": 2 }, "description": "Net to its two end pads (REF.PAD) for a pad-to-pad length" }
            }, "required": ["nets"], "additionalProperties": false } },
            "groups_file": { "type": "string", "description": "More groups in a JSON file (an array, or {\"groups\": [...]})" },
            "include_vias": { "type": "boolean", "default": true, "description": "Add each via's barrel between the outermost layers the net's copper uses on it" }
        }),
        &["board"],
    )
}

/// One net's routed length.
struct NetLength {
    tracks_mm: f64,
    arcs_mm: f64,
    vias_mm: f64,
    segments: usize,
    vias: usize,
    by_layer: BTreeMap<String, f64>,
}

fn net_length(l: &Loaded, net: NetId) -> NetLength {
    let board = l.board();
    let mut out = NetLength {
        tracks_mm: 0.0,
        arcs_mm: 0.0,
        vias_mm: 0.0,
        segments: 0,
        vias: 0,
        by_layer: BTreeMap::new(),
    };
    let mut ends: Vec<(super::geom::P, konnect_route::api::LayerId)> = Vec::new();
    for t in &board.tracks {
        if t.net != Some(net) {
            continue;
        }
        let len = dist(pmm(t.seg.a), pmm(t.seg.b));
        out.tracks_mm += len;
        out.segments += 1;
        *out.by_layer
            .entry(board.layers.name(t.layer).to_string())
            .or_default() += len;
        ends.push((pmm(t.seg.a), t.layer));
        ends.push((pmm(t.seg.b), t.layer));
    }
    for a in &board.arcs {
        if a.net != Some(net) {
            continue;
        }
        let net_model_len = arc_len(pmm(a.start), pmm(a.mid), pmm(a.end));
        out.arcs_mm += net_model_len;
        out.segments += 1;
        *out.by_layer
            .entry(board.layers.name(a.layer).to_string())
            .or_default() += net_model_len;
        ends.push((pmm(a.start), a.layer));
        ends.push((pmm(a.end), a.layer));
    }
    for v in &board.vias {
        if v.net != Some(net) {
            continue;
        }
        out.vias += 1;
        let c = pmm(v.center);
        let r = f64::from(v.diameter) / 2e6;
        let mut used = LayerMask::EMPTY;
        for (p, layer) in &ends {
            if v.layers.contains(*layer) && dist(*p, c) <= r + 1e-6 {
                used = used.with(*layer);
            }
        }
        for p in board.pads_of_net(net) {
            for layer in p.layers.iter() {
                if v.layers.contains(layer) && dist(pmm(p.center), c) <= r + 0.5 {
                    used = used.with(layer);
                }
            }
        }
        let zs: Vec<f64> = used.iter().map(|x| l.stack.z_mm[x.index()]).collect();
        if zs.len() >= 2 {
            let (lo, hi) = zs
                .iter()
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), z| {
                    (a.min(*z), b.max(*z))
                });
            out.vias_mm += hi - lo;
        }
    }
    out
}

fn arc_len(a: super::geom::P, m: super::geom::P, b: super::geom::P) -> f64 {
    // Same circumcircle construction as the network's arcs.
    let d = 2.0 * (a[0] * (m[1] - b[1]) + m[0] * (b[1] - a[1]) + b[0] * (a[1] - m[1]));
    if d.abs() < 1e-12 {
        return dist(a, b);
    }
    let sq = |p: super::geom::P| p[0] * p[0] + p[1] * p[1];
    let ux = (sq(a) * (m[1] - b[1]) + sq(m) * (b[1] - a[1]) + sq(b) * (a[1] - m[1])) / d;
    let uy = (sq(a) * (b[0] - m[0]) + sq(m) * (a[0] - b[0]) + sq(b) * (m[0] - a[0])) / d;
    let r = dist(a, [ux, uy]);
    let ang = |p: super::geom::P| (p[1] - uy).atan2(p[0] - ux);
    let norm = |x: f64| x.rem_euclid(std::f64::consts::TAU);
    let sweep_ab = norm(ang(b) - ang(a));
    let sweep_am = norm(ang(m) - ang(a));
    r * if sweep_am <= sweep_ab {
        sweep_ab
    } else {
        std::f64::consts::TAU - sweep_ab
    }
}

pub(crate) fn measure_net_lengths(
    l: &Loaded,
    args: &Value,
    out: &Outputs,
) -> Result<CallToolResult, Refusal> {
    let board = l.board();
    let include_vias = opt_bool(args, "include_vias", true)?;
    let max_items = opt_count(args, "max_items", 50, 100_000)? as usize;
    let mut groups: Vec<Value> = args
        .get("groups")
        .and_then(Value::as_array)
        .cloned()
        .unwrap_or_default();
    if let Some(v) = common::json_file(args, "groups_file")? {
        match v {
            Value::Array(a) => groups.extend(a),
            Value::Object(mut o) => match o.remove("groups") {
                Some(Value::Array(a)) => groups.extend(a),
                _ => {
                    return Err(invalid(
                        "groups_file",
                        "must hold an array or {\"groups\": [...]}",
                    ))
                }
            },
            _ => return Err(invalid("groups_file", "must hold an array of groups")),
        }
    }
    if groups.is_empty() {
        return Err(invalid(
            "groups",
            "give at least one group (groups and/or groups_file)",
        ));
    }
    let t0 = std::time::Instant::now();
    let mut rows = Vec::new();
    let mut markers = Vec::new();
    for (gi, g) in groups.iter().enumerate() {
        let field = format!("groups[{gi}]");
        let name = g["name"]
            .as_str()
            .map(str::to_string)
            .unwrap_or_else(|| format!("group {gi}"));
        let pats: Vec<String> = g["nets"]
            .as_array()
            .map(|a| {
                a.iter()
                    .filter_map(|x| x.as_str().map(String::from))
                    .collect()
            })
            .unwrap_or_default();
        if pats.is_empty() {
            return Err(invalid(&format!("{field}.nets"), "needs at least one net"));
        }
        let nets = common::nets_named(board, &pats, &format!("{field}.nets"))?;
        let kind = g["kind"].as_str().unwrap_or("bus");
        if kind == "pair" && nets.len() != 2 {
            return Err(invalid(
                &format!("{field}.nets"),
                &format!("a pair needs exactly two nets, these match {}", nets.len()),
            ));
        }
        let target = g["target_length_mm"].as_f64();
        let tol = g["tolerance_mm"].as_f64();
        let max_skew = g["max_skew_mm"].as_f64();
        let mut members = Vec::new();
        for &n in &nets {
            let nl = net_length(l, n);
            let routed = nl.tracks_mm + nl.arcs_mm + if include_vias { nl.vias_mm } else { 0.0 };
            let net_name = board.net(n).name.clone();
            // Pad to pad over the copper network, when asked.
            let pad_to_pad = match g
                .get("endpoints")
                .and_then(|e| e.get(&net_name))
                .and_then(Value::as_array)
            {
                Some(ends) => {
                    let names: Vec<String> = ends
                        .iter()
                        .filter_map(|x| x.as_str().map(String::from))
                        .collect();
                    let pads = super::network::pads_named(
                        board,
                        &names,
                        &format!("{field}.endpoints.{net_name}"),
                    )?;
                    let nm = Network::build(
                        l,
                        n,
                        &Options {
                            h: 0.1,
                            temp_c: 25.0,
                            plating_mm: 0.02,
                            max_cells: 4_000_000,
                            window: None,
                        },
                    )
                    .map_err(|e| invalid(&format!("{field}.endpoints"), &e))?;
                    match (nm.pad(pads[0]), nm.pad(pads[1])) {
                        (Some(a), Some(b)) => {
                            let d = nm.path_lengths(&[a]);
                            let v = d[b as usize];
                            if v.is_finite() {
                                json!(r4(v))
                            } else {
                                json!("not joined")
                            }
                        }
                        _ => json!("pad not in the net's copper"),
                    }
                }
                None => Value::Null,
            };
            members.push(json!({
                "net": net_name,
                "length_mm": r4(routed),
                "tracks_mm": r4(nl.tracks_mm),
                "arcs_mm": r4(nl.arcs_mm),
                "via_barrels_mm": r4(nl.vias_mm),
                "segments": nl.segments,
                "vias": nl.vias,
                "by_layer_mm": nl.by_layer.iter().map(|(k, v)| (k.clone(), r4(*v))).collect::<BTreeMap<_, _>>(),
                "pad_to_pad_mm": pad_to_pad,
                "target_delta_mm": target.map(|t| r4(routed - t)),
                "within_tolerance": match (target, tol) { (Some(t), Some(tl)) => json!((routed - t).abs() <= tl), _ => Value::Null },
            }));
        }
        let lens: Vec<f64> = members
            .iter()
            .map(|m| m["length_mm"].as_f64().unwrap_or(0.0))
            .collect();
        let (lo, hi) = lens
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), x| {
                (a.min(*x), b.max(*x))
            });
        let spread = hi - lo;
        let ok = max_skew.is_none_or(|m| spread <= m)
            && members
                .iter()
                .all(|m| m["within_tolerance"].as_bool() != Some(false));
        if !ok {
            // Mark the group at its first net's first pad.
            if let Some(&n) = nets.iter().next() {
                if let Some(p) = board.pads_of_net(n).next() {
                    let c = pmm(p.center);
                    markers.push(marker(
                        c[0],
                        c[1],
                        "length_mismatch",
                        format!("{name} spread {:.2} mm", spread),
                    ));
                }
            }
        }
        rows.push(json!({
            "group": name,
            "kind": kind,
            "nets": members,
            "skew_mm": if kind == "pair" { json!(r4(spread)) } else { Value::Null },
            "spread_mm": r4(spread),
            "longest_mm": r4(hi),
            "shortest_mm": r4(lo),
            "max_skew_mm": max_skew,
            "ok": ok,
        }));
    }
    let methods = json!({
        "length": "tracks (end to end) plus arcs (along the arc) of the net, plus each via's barrel between the outermost layers the net's tracks, arcs or pads use on it (stack depths between copper centres)",
        "pad_to_pad": "shortest path over the net's copper network between the two named pads",
        "skew": "pair: |L1 − L2|; group: longest − shortest",
    });
    let full = merged(
        common::provenance(l),
        json!({ "include_vias": include_vias, "groups": rows, "stackup": l.stack.json(board), "methods": methods }),
    );
    let files = out.write(&full, &markers)?;
    Ok(CallToolResult::json(&merged(
        common::provenance(l),
        json!({
            "include_vias": include_vias,
            "groups": super::hygiene::take(&full["groups"], max_items),
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
    fn a_pair_reports_lengths_vias_and_skew() {
        let board = ts::board(&[
            ts::pad("J1", 0.0, 0.0, 0.5, 0.5, "DP"),
            ts::pad("J2", 0.0, 1.0, 0.5, 0.5, "DN"),
            ts::segment(0.0, 0.0, 10.0, 0.0, 0.2, "F.Cu", "DP"),
            ts::via(10.0, 0.0, 0.4, 0.2, "DP"),
            ts::segment(10.0, 0.0, 20.0, 0.0, 0.2, "B.Cu", "DP"),
            ts::segment(0.0, 1.0, 21.0, 1.0, 0.2, "F.Cu", "DN"),
        ]);
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("t.kicad_pcb");
        std::fs::write(&path, board).unwrap();
        let l = common::load_at(&path).unwrap();
        let out = Outputs {
            output: None,
            markers: None,
        };
        let r = measure_net_lengths(&l, &json!({ "board": path, "groups": [{ "name": "USB", "kind": "pair", "nets": ["DP", "DN"], "max_skew_mm": 0.5 }] }), &out).unwrap();
        let b: Value = match &r.content[0] {
            crate::mcp::protocol::ToolContent::Text { text } => serde_json::from_str(text).unwrap(),
            _ => panic!(),
        };
        let g = &b["groups"][0];
        let dn = g["nets"]
            .as_array()
            .unwrap()
            .iter()
            .find(|m| m["net"] == "DN")
            .unwrap();
        let dp = g["nets"]
            .as_array()
            .unwrap()
            .iter()
            .find(|m| m["net"] == "DP")
            .unwrap();
        assert_eq!(dn["length_mm"], 21.0);
        // DP: 20 mm of track plus the barrel from F.Cu to B.Cu centres (0.035/2 + 1.51 + 0.035/2).
        assert!(
            (dp["via_barrels_mm"].as_f64().unwrap() - 1.545).abs() < 1e-9,
            "{dp}"
        );
        assert!((g["skew_mm"].as_f64().unwrap() - 0.545).abs() < 1e-9, "{g}");
        assert_eq!(g["ok"], false);
    }
}
