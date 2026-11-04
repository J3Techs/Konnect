//! K8 `check_voltage_spacing`: IPC-2221B Table 6-1 clearance for the
//! voltages between nets, the copper closer than that on each layer, and
//! optional `.kicad_dru` rules so KiCad's DRC enforces it afterwards.

use super::common::{self, marker, merged, r4, Loaded, Outputs};
use super::geom::{pmm, P};
use super::ipc::{ipc2221b_spacing_mm, SpacingColumn};
use crate::mcp::protocol::CallToolResult;
use crate::tools::route_native::{invalid, opt_bool, opt_count, opt_path, Refusal};
use konnect_geom::{Circle, IndexedPolygon, Segment, Shape};
use konnect_route::api::{LayerId, NetId};
use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet, HashMap};

pub(crate) fn schema() -> Value {
    common::schema(
        json!({
            "voltages": { "type": "object", "additionalProperties": { "anyOf": [
                { "type": "number" },
                { "type": "object", "properties": { "working": { "type": "number" }, "transient": { "type": "number" } }, "additionalProperties": false }
            ] }, "description": "Net name or * ? pattern to its voltage, V (DC or AC peak), or {working, transient}: the larger magnitude is used" },
            "voltages_file": { "type": "string", "description": "The same map in a JSON file" },
            "default_voltage": { "type": "number", "default": 0, "description": "Voltage of every net not in the map" },
            "external_column": { "type": "string", "enum": ["B2", "B3", "B4", "A5", "A6", "A7"], "description": "IPC-2221B Table 6-1 column for outer layers (default B2, or B4 when coated)" },
            "internal_column": { "type": "string", "enum": ["B1"], "default": "B1" },
            "coated": { "type": "boolean", "default": false, "description": "Outer layers carry a permanent polymer coating (column B4)" },
            "layers": { "type": "array", "items": { "type": "string" }, "minItems": 1 },
            "rules_output": { "type": "string", "description": "New .kicad_dru file of clearance rules for these voltages, to merge into the project's rules; an existing file is never replaced" }
        }),
        &["board"],
    )
}

#[derive(Clone)]
enum Geo {
    Shape(Shape),
    Fill(usize),
}

struct Item {
    net: NetId,
    geo: Geo,
    lo: P,
    hi: P,
    what: String,
}

fn volts_of(v: &Value) -> Option<f64> {
    match v {
        Value::Number(n) => n.as_f64(),
        Value::Object(o) => {
            let w = o.get("working").and_then(Value::as_f64);
            let t = o.get("transient").and_then(Value::as_f64);
            match (w, t) {
                (Some(a), Some(b)) => Some(if b.abs() > a.abs() { b } else { a }),
                (a, b) => a.or(b),
            }
        }
        _ => None,
    }
}

pub(crate) fn check_voltage_spacing(
    l: &Loaded,
    args: &Value,
    out: &Outputs,
) -> Result<CallToolResult, Refusal> {
    let board = l.board();
    let default_v = common::opt_f64(args, "default_voltage", 0.0, -1e5, 1e5)?;
    let coated = opt_bool(args, "coated", false)?;
    let ext = match args.get("external_column").and_then(Value::as_str) {
        Some(c) => SpacingColumn::parse(c)
            .ok_or_else(|| invalid("external_column", "is not a Table 6-1 column"))?,
        None if coated => SpacingColumn::B4,
        None => SpacingColumn::B2,
    };
    // Table 6-1 has one internal column; it is accepted by name so a
    // request states it explicitly.
    let int = match args.get("internal_column").and_then(Value::as_str) {
        None | Some("B1") => SpacingColumn::B1,
        Some(other) => {
            return Err(invalid(
                "internal_column",
                &format!("'{other}' is not an internal column of Table 6-1 (B1)"),
            ))
        }
    };
    let rules_path = opt_path(args, "rules_output")?;
    if let Some(p) = &rules_path {
        if p.exists() {
            return Err(invalid(
                "rules_output",
                &format!("{} already exists; it is never replaced", p.display()),
            ));
        }
    }
    let max_items = opt_count(args, "max_items", 50, 100_000)? as usize;
    let only = crate::tools::board_inspect::layers_arg(board, args, "layers")?;
    let mut map: BTreeMap<String, Value> = BTreeMap::new();
    if let Some(Value::Object(m)) = args.get("voltages") {
        map.extend(m.clone());
    }
    if let Some(v) = common::json_file(args, "voltages_file")? {
        match v {
            Value::Object(m) => map.extend(m),
            _ => {
                return Err(invalid(
                    "voltages_file",
                    "must hold an object of net to volts",
                ))
            }
        }
    }
    if map.is_empty() {
        return Err(invalid(
            "voltages",
            "give at least one net voltage (voltages and/or voltages_file)",
        ));
    }
    let t0 = std::time::Instant::now();
    // Resolve patterns to nets; a later, more specific entry does not
    // override an exact name.
    let mut volts: BTreeMap<NetId, f64> = BTreeMap::new();
    for (pat, v) in &map {
        let v = volts_of(v).ok_or_else(|| invalid("voltages", &format!("{pat}: not a voltage")))?;
        let nets = common::nets_named(board, std::slice::from_ref(pat), "voltages")?;
        for n in nets {
            let exact = board.net(n).name == *pat;
            if exact || !volts.contains_key(&n) {
                volts.insert(n, v);
            }
        }
    }
    let v_of = |n: NetId| volts.get(&n).copied().unwrap_or(default_v);
    let column = |layer: LayerId| if l.stack.is_outer(layer) { ext } else { int };
    let all_v: Vec<f64> = volts
        .values()
        .copied()
        .chain(std::iter::once(default_v))
        .collect();

    // Copper by layer.
    let mut fills: Vec<IndexedPolygon> = Vec::new();
    let mut by_layer: BTreeMap<u8, Vec<Item>> = BTreeMap::new();
    let push = |by_layer: &mut BTreeMap<u8, Vec<Item>>, layer: LayerId, item: Item| {
        if only.is_none_or(|m| m.contains(layer)) {
            by_layer.entry(layer.0).or_default().push(item);
        }
    };
    let bb = |s: &Shape| {
        let b = s.bbox();
        (pmm(b.min), pmm(b.max))
    };
    for p in &board.pads {
        let Some(n) = p.net else { continue };
        for layer in p.layers.iter() {
            for s in &p.shapes {
                let (lo, hi) = bb(s);
                push(
                    &mut by_layer,
                    layer,
                    Item {
                        net: n,
                        geo: Geo::Shape(s.clone()),
                        lo,
                        hi,
                        what: format!("pad {}", crate::tools::board_inspect::pad_label(board, p)),
                    },
                );
            }
        }
    }
    for t in &board.tracks {
        let Some(n) = t.net else { continue };
        let s = Shape::Segment(t.seg);
        let (lo, hi) = bb(&s);
        push(
            &mut by_layer,
            t.layer,
            Item {
                net: n,
                geo: Geo::Shape(s),
                lo,
                hi,
                what: "track".into(),
            },
        );
    }
    for a in &board.arcs {
        let Some(n) = a.net else { continue };
        for s in [
            Segment::new(a.start, a.mid, a.width),
            Segment::new(a.mid, a.end, a.width),
        ] {
            let s = Shape::Segment(s);
            let (lo, hi) = bb(&s);
            push(
                &mut by_layer,
                a.layer,
                Item {
                    net: n,
                    geo: Geo::Shape(s),
                    lo,
                    hi,
                    what: "arc".into(),
                },
            );
        }
    }
    for v in &board.vias {
        let Some(n) = v.net else { continue };
        let s = Shape::Circle(Circle::new(v.center, v.diameter / 2));
        let (lo, hi) = bb(&s);
        for layer in v.layers.iter() {
            push(
                &mut by_layer,
                layer,
                Item {
                    net: n,
                    geo: Geo::Shape(s.clone()),
                    lo,
                    hi,
                    what: "via".into(),
                },
            );
        }
    }
    for z in &board.zones {
        let (Some(n), true) = (z.net, z.filled) else {
            continue;
        };
        if only.is_some_and(|m| !m.contains(z.layer)) {
            continue;
        }
        let b = z.region.bbox();
        fills.push(IndexedPolygon::new(z.region.clone()));
        push(
            &mut by_layer,
            z.layer,
            Item {
                net: n,
                geo: Geo::Fill(fills.len() - 1),
                lo: pmm(b.min),
                hi: pmm(b.max),
                what: format!("zone {}", z.zone_name.clone().unwrap_or_default()),
            },
        );
    }

    let mut pairs: BTreeMap<(String, String), Value> = BTreeMap::new();
    let mut violations = Vec::new();
    let mut markers = Vec::new();
    let mut checked = 0usize;
    for (&lid, items) in &by_layer {
        let layer = LayerId(lid);
        let col = column(layer);
        // Bucket the layer's items on a 2 mm grid (large fills by bbox).
        let cell = 2.0;
        let mut grid: HashMap<(i32, i32), Vec<u32>> = HashMap::new();
        for (i, it) in items.iter().enumerate() {
            let (x0, y0) = (
                (it.lo[0] / cell).floor() as i32,
                (it.lo[1] / cell).floor() as i32,
            );
            let (x1, y1) = (
                (it.hi[0] / cell).floor() as i32,
                (it.hi[1] / cell).floor() as i32,
            );
            for x in x0..=x1 {
                for y in y0..=y1 {
                    grid.entry((x, y)).or_default().push(i as u32);
                }
            }
        }
        let mut seen: BTreeSet<(u32, u32)> = BTreeSet::new();
        for (ai, a) in items.iter().enumerate() {
            if !volts.contains_key(&a.net) {
                continue;
            }
            let va = v_of(a.net);
            let reach = all_v
                .iter()
                .map(|v| ipc2221b_spacing_mm((va - v).abs(), col))
                .fold(0.0, f64::max);
            let (x0, y0) = (
                ((a.lo[0] - reach) / cell).floor() as i32,
                ((a.lo[1] - reach) / cell).floor() as i32,
            );
            let (x1, y1) = (
                ((a.hi[0] + reach) / cell).floor() as i32,
                ((a.hi[1] + reach) / cell).floor() as i32,
            );
            let mut cand = BTreeSet::new();
            for x in x0..=x1 {
                for y in y0..=y1 {
                    if let Some(v) = grid.get(&(x, y)) {
                        cand.extend(v.iter().copied());
                    }
                }
            }
            for bi in cand {
                let b = &items[bi as usize];
                if b.net == a.net {
                    continue;
                }
                let key = ((ai as u32).min(bi), (ai as u32).max(bi));
                if !seen.insert(key) {
                    continue;
                }
                if b.hi[0] < a.lo[0] - reach
                    || b.lo[0] > a.hi[0] + reach
                    || b.hi[1] < a.lo[1] - reach
                    || b.lo[1] > a.hi[1] + reach
                {
                    continue;
                }
                let dv = (va - v_of(b.net)).abs();
                let req = ipc2221b_spacing_mm(dv, col);
                let req_nm = (req * 1e6).round() as i64;
                checked += 1;
                let found: Option<(f64, P)> = match (&a.geo, &b.geo) {
                    (Geo::Shape(sa), Geo::Shape(sb)) => {
                        let c = konnect_geom::closest(sa, sb);
                        (c.gap_nm < req_nm as f64).then(|| (c.gap_nm / 1e6, pmm(c.midpoint())))
                    }
                    (Geo::Fill(f), Geo::Shape(s)) | (Geo::Shape(s), Geo::Fill(f)) => fills[*f]
                        .closest_within(s, req_nm)
                        .filter(|c| c.gap_nm < req_nm as f64)
                        .map(|c| (c.gap_nm / 1e6, pmm(c.midpoint()))),
                    (Geo::Fill(fa), Geo::Fill(fb)) => (fills[*fa]
                        .gap_cmp_polygon(&fills[*fb], req_nm)
                        == std::cmp::Ordering::Less)
                        .then(|| {
                            let lo = [a.lo[0].max(b.lo[0]), a.lo[1].max(b.lo[1])];
                            let hi = [a.hi[0].min(b.hi[0]), a.hi[1].min(b.hi[1])];
                            (f64::NAN, [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0])
                        }),
                };
                let (na, nb) = (board.net(a.net).name.clone(), board.net(b.net).name.clone());
                let pk = if na <= nb {
                    (na.clone(), nb.clone())
                } else {
                    (nb.clone(), na.clone())
                };
                let row = pairs.entry(pk.clone()).or_insert_with(|| {
                    json!({
                        "nets": [pk.0, pk.1],
                        "voltage_difference_v": r4(dv),
                        "required_outer_mm": ipc2221b_spacing_mm(dv, ext),
                        "required_inner_mm": ipc2221b_spacing_mm(dv, int),
                        "violations": 0,
                        "min_gap_mm": Value::Null,
                    })
                });
                if let Some((gap, at)) = found {
                    row["violations"] = json!(row["violations"].as_u64().unwrap_or(0) + 1);
                    if gap.is_finite() && row["min_gap_mm"].as_f64().is_none_or(|g| gap < g) {
                        row["min_gap_mm"] = json!(r4(gap));
                    }
                    markers.push(marker(
                        at[0],
                        at[1],
                        "hv_spacing",
                        format!("{na}/{nb} {:.3} < {req} mm", gap),
                    ));
                    violations.push(json!({
                        "layer": board.layers.name(layer),
                        "nets": [na, nb],
                        "items": [a.what, b.what],
                        "voltage_difference_v": r4(dv),
                        "column": col.name(),
                        "required_mm": req,
                        "gap_mm": if gap.is_finite() { json!(r4(gap)) } else { Value::Null },
                        "at": [r4(at[0]), r4(at[1])],
                    }));
                }
            }
        }
    }
    violations.sort_by(|a, b| {
        let m = |v: &Value| {
            v["gap_mm"].as_f64().unwrap_or(0.0) - v["required_mm"].as_f64().unwrap_or(0.0)
        };
        m(a).total_cmp(&m(b))
    });
    let mut pair_rows: Vec<Value> = pairs.into_values().collect();
    pair_rows.sort_by(|a, b| {
        b["violations"]
            .as_u64()
            .cmp(&a["violations"].as_u64())
            .then(
                b["voltage_difference_v"]
                    .as_f64()
                    .unwrap_or(0.0)
                    .total_cmp(&a["voltage_difference_v"].as_f64().unwrap_or(0.0)),
            )
    });

    // Requirements per mapped net against the default and each other.
    let mut requirements = Vec::new();
    let mut rules =
        String::from("(version 1)\n# IPC-2221B Table 6-1 clearances from check_voltage_spacing.\n");
    let quote = |s: &str| s.replace('\'', "\\'");
    for (&n, &v) in &volts {
        let name = board.net(n).name.clone();
        let dv = (v - default_v).abs();
        let (o, i) = (ipc2221b_spacing_mm(dv, ext), ipc2221b_spacing_mm(dv, int));
        requirements.push(json!({ "net": name, "voltage_v": v, "against_default_v": default_v, "outer_mm": o, "inner_mm": i }));
        rules.push_str(&format!(
            "(rule \"IPC2221B {name} outer\"\n\t(layer outer)\n\t(constraint clearance (min {o}mm))\n\t(condition \"A.NetName == '{q}' && B.NetName != '{q}'\"))\n(rule \"IPC2221B {name} inner\"\n\t(layer inner)\n\t(constraint clearance (min {i}mm))\n\t(condition \"A.NetName == '{q}' && B.NetName != '{q}'\"))\n",
            q = quote(&name)
        ));
    }
    let ids: Vec<NetId> = volts.keys().copied().collect();
    for (k, &a) in ids.iter().enumerate() {
        for &b in &ids[k + 1..] {
            let dv = (volts[&a] - volts[&b]).abs();
            let (o, i) = (ipc2221b_spacing_mm(dv, ext), ipc2221b_spacing_mm(dv, int));
            let (na, nb) = (board.net(a).name.clone(), board.net(b).name.clone());
            rules.push_str(&format!(
                "(rule \"IPC2221B {na} to {nb} outer\"\n\t(layer outer)\n\t(constraint clearance (min {o}mm))\n\t(condition \"A.NetName == '{qa}' && B.NetName == '{qb}'\"))\n(rule \"IPC2221B {na} to {nb} inner\"\n\t(layer inner)\n\t(constraint clearance (min {i}mm))\n\t(condition \"A.NetName == '{qa}' && B.NetName == '{qb}'\"))\n",
                qa = quote(&na),
                qb = quote(&nb)
            ));
        }
    }
    let rules_written = match &rules_path {
        Some(p) => {
            crate::native_route_job::write_new_atomic(p, rules.as_bytes())
                .map_err(|e| CallToolResult::error(format!("rules_output not written: {e:#}")))?;
            Some(p.display().to_string())
        }
        None => None,
    };
    let methods = json!({
        "standard": "IPC-2221B Table 6-1 minimum conductor spacing by voltage between conductors (DC or AC peak); bands of whole volts, above 500 V per volt",
        "columns": { "outer": ext.name(), "inner": int.name() },
        "voltage": "each pair's difference of the mapped voltages (unmapped nets at default_voltage); {working, transient} uses the larger magnitude",
        "geometry": "copper edge to edge on each layer: pads, tracks, arcs, vias and zone fills; fills against fills by exact comparison (location approximate)",
        "rules": "rules_output: per mapped net against every other net, then per mapped pair (later rules take precedence), layer outer/inner",
    });
    let full = merged(
        common::provenance(l),
        json!({
            "columns": { "outer": ext.name(), "inner": int.name() },
            "default_voltage": default_v,
            "nets_mapped": volts.len(),
            "requirements": requirements,
            "pairs_near": pair_rows,
            "violations": violations,
            "item_pairs_checked": checked,
            "rules_output": rules_written,
            "rules": rules,
            "methods": methods,
        }),
    );
    let files = out.write(&full, &markers)?;
    Ok(CallToolResult::json(&merged(
        common::provenance(l),
        json!({
            "columns": full["columns"],
            "nets_mapped": volts.len(),
            "requirements": super::hygiene::take(&full["requirements"], max_items),
            "violation_count": full["violations"].as_array().map_or(0, Vec::len),
            "violations": super::hygiene::take(&full["violations"], max_items),
            "pairs_with_violations": full["pairs_near"].as_array().map(|a| a.iter().filter(|p| p["violations"].as_u64().unwrap_or(0) > 0).take(max_items).cloned().collect::<Vec<_>>()),
            "item_pairs_checked": checked,
            "rules_output": rules_written,
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

    /// Two parallel tracks 0.4 mm apart (edge to edge) on F.Cu, one at
    /// 60 V: IPC-2221B B2 asks 0.6 mm, so they violate; on B1 (inner) 0.1.
    #[test]
    fn sixty_volts_needs_the_b2_value() {
        let board = ts::board(&[
            ts::segment(0.0, 0.0, 10.0, 0.0, 0.2, "F.Cu", "HV"),
            ts::segment(0.0, 0.6, 10.0, 0.6, 0.2, "F.Cu", "LV"),
        ]);
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("t.kicad_pcb");
        std::fs::write(&path, board).unwrap();
        let rules = dir.path().join("hv.kicad_dru");
        let l = common::load_at(&path).unwrap();
        let out = Outputs {
            output: None,
            markers: None,
        };
        let r = check_voltage_spacing(
            &l,
            &json!({ "board": path, "voltages": { "HV": 60 }, "rules_output": rules }),
            &out,
        )
        .unwrap();
        let b = body(&r);
        assert_eq!(b["requirements"][0]["outer_mm"], 0.6, "{b}");
        assert_eq!(b["requirements"][0]["inner_mm"], 0.1);
        assert_eq!(b["violation_count"], 1, "{b}");
        let v = &b["violations"][0];
        assert_eq!(v["required_mm"], 0.6);
        assert!((v["gap_mm"].as_f64().unwrap() - 0.4).abs() < 1e-3, "{v}");
        let text = std::fs::read_to_string(&rules).unwrap();
        assert!(
            text.contains("(constraint clearance (min 0.6mm))"),
            "{text}"
        );
        konnect_sexp::parse_sexp(&format!("({text})")).ok();
        // Coated: B4 = 0.13 mm, no violation.
        let r = check_voltage_spacing(
            &l,
            &json!({ "board": path, "voltages": { "HV": 60 }, "coated": true }),
            &out,
        )
        .unwrap();
        assert_eq!(body(&r)["violation_count"], 0);
    }
}
