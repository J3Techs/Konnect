//! K5 `check_decoupling_placement`: for each IC power pin, the nearest
//! same-net capacitor by copper path, and how directly the capacitor's and
//! the IC's ground pins reach a ground via.

use super::common::{self, marker, merged, r4, Loaded, Outputs};
use super::geom::{dist, pmm, P};
use super::network::{Network, Options};
use crate::mcp::protocol::CallToolResult;
use crate::tools::board_inspect::pad_label;
use crate::tools::route_native::{invalid, opt_count, opt_strings, Refusal};
use konnect_geom::{Circle, Point, Shape};
use konnect_route::api::NetId;
use konnect_route::board::RouteBoard;
use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet};

pub(crate) fn schema() -> Value {
    common::schema(
        json!({
            "references": { "type": "array", "items": { "type": "string" }, "minItems": 1, "description": "ICs to check (default: every footprint whose reference starts with U)" },
            "ground_nets": { "type": "array", "items": { "type": "string" }, "minItems": 1, "description": "Ground nets (names or * ? patterns; default GND*, *GND, VSS*)" },
            "capacitor_prefix": { "type": "string", "default": "C", "description": "Reference prefix of capacitors" },
            "associations": { "type": "array", "items": { "type": "object", "properties": { "pin": { "type": "string" }, "capacitors": { "type": "array", "items": { "type": "string" }, "minItems": 1 } }, "required": ["pin", "capacitors"], "additionalProperties": false }, "description": "Pin (REF.PAD) to its intended capacitors: those are measured instead of the nearest" },
            "max_distance_mm": { "type": "number", "exclusiveMinimum": 0, "maximum": 200, "default": 25, "description": "Only capacitors within this straight-line distance of the pin are path-measured" },
            "resolution_mm": { "type": "number", "minimum": 0.02, "maximum": 2, "default": 0.1, "description": "Fill mesh step for path lengths across pours" },
            "pins": { "type": "array", "items": { "type": "string" }, "minItems": 1, "description": "Only these IC pins (REF.PAD)" },
            "power_nets": { "type": "array", "items": { "type": "string" }, "minItems": 1, "description": "Power nets (names or * ? patterns). Default: every net with at least min_capacitors capacitors to ground" },
            "min_capacitors": { "type": "integer", "minimum": 1, "maximum": 100, "default": 2, "description": "Without power_nets: a net needs this many capacitors to ground to count as a supply (a single filter capacitor marks a signal)" }
        }),
        &["board"],
    )
}

struct Gnd {
    nets: BTreeSet<NetId>,
    vias: Vec<(P, f64, Option<String>, konnect_route::api::LayerMask)>,
}

impl Gnd {
    /// The nearest ground via to a pad: (distance centre to centre, in pad, index).
    fn nearest(&self, board: &RouteBoard, pad: usize) -> Option<(f64, bool, usize)> {
        let p = &board.pads[pad];
        let c = pmm(p.center);
        let mut best: Option<(f64, bool, usize)> = None;
        for (i, v) in self.vias.iter().enumerate() {
            if !v.3.intersects(p.layers) {
                continue;
            }
            let d = dist(v.0, c);
            if d > 30.0 {
                continue;
            }
            let in_pad = Point::from_mm(v.0[0], v.0[1]).is_some_and(|vc| {
                let s = Shape::Circle(Circle::new(vc, (v.1 * 1e6) as i32));
                p.shapes.iter().any(|sh| konnect_geom::touches(sh, &s))
            });
            let key = if in_pad { 0.0 } else { d };
            if best.is_none_or(|b| key < if b.1 { 0.0 } else { b.0 }) {
                best = Some((d, in_pad, i));
            }
        }
        best
    }
}

pub(crate) fn check_decoupling_placement(
    l: &Loaded,
    args: &Value,
    out: &Outputs,
) -> Result<CallToolResult, Refusal> {
    let board = l.board();
    let gpats = opt_strings(args, "ground_nets")?
        .unwrap_or_else(|| vec!["GND*".into(), "*GND".into(), "VSS*".into()]);
    let gnets: BTreeSet<NetId> = gpats
        .iter()
        .flat_map(|p| {
            konnect_route::pipeline::coverage::select_nets(board, None, std::slice::from_ref(p))
        })
        .collect();
    if gnets.is_empty() {
        return Err(invalid("ground_nets", "no net of the board matches"));
    }
    let cap_prefix = args
        .get("capacitor_prefix")
        .and_then(Value::as_str)
        .unwrap_or("C")
        .to_string();
    let max_d = common::opt_f64(args, "max_distance_mm", 25.0, 1e-3, 200.0)?;
    let h = common::opt_f64(args, "resolution_mm", 0.1, 0.02, 2.0)?;
    let max_items = opt_count(args, "max_items", 50, 100_000)? as usize;
    let t0 = std::time::Instant::now();
    let refs: Vec<String> = match opt_strings(args, "references")? {
        Some(r) => r,
        None => board
            .footprints
            .iter()
            .filter_map(|f| f.reference.clone())
            .filter(|r| r.starts_with('U'))
            .collect(),
    };
    let only_pins: Option<BTreeSet<usize>> = match opt_strings(args, "pins")? {
        Some(names) => Some(
            super::network::pads_named(board, &names, "pins")?
                .into_iter()
                .collect(),
        ),
        None => None,
    };
    let mut assoc: BTreeMap<usize, Vec<String>> = BTreeMap::new();
    if let Some(Value::Array(a)) = args.get("associations") {
        for (i, x) in a.iter().enumerate() {
            let pin = x["pin"].as_str().unwrap_or("");
            let pads = super::network::pads_named(
                board,
                &[pin.to_string()],
                &format!("associations[{i}].pin"),
            )?;
            let caps: Vec<String> = x["capacitors"]
                .as_array()
                .map(|c| {
                    c.iter()
                        .filter_map(|v| v.as_str().map(String::from))
                        .collect()
                })
                .unwrap_or_default();
            for p in pads {
                assoc.insert(p, caps.clone());
            }
        }
    }
    let gnd = Gnd {
        nets: gnets.clone(),
        vias: board
            .vias
            .iter()
            .filter(|v| v.net.is_some_and(|n| gnets.contains(&n)))
            .map(|v| {
                (
                    pmm(v.center),
                    f64::from(v.diameter) / 2e6,
                    v.uuid.clone(),
                    v.layers,
                )
            })
            .collect(),
    };

    // Capacitors: two-pad parts whose one pad is ground. By power net: (cap ref, power pad, ground pad).
    let mut caps_by_net: BTreeMap<NetId, Vec<(String, usize, usize)>> = BTreeMap::new();
    for fp in &board.footprints {
        let Some(r) = &fp.reference else { continue };
        if !r.starts_with(&cap_prefix)
            || !r[cap_prefix.len()..].starts_with(|c: char| c.is_ascii_digit())
        {
            continue;
        }
        let pads: Vec<usize> = (fp.pad_range.0 as usize..fp.pad_range.1 as usize)
            .filter(|&i| board.pads[i].net.is_some())
            .collect();
        if pads.len() != 2 {
            continue;
        }
        let (a, b) = (pads[0], pads[1]);
        let ga = gnd.nets.contains(&board.pads[a].net.unwrap());
        let gb = gnd.nets.contains(&board.pads[b].net.unwrap());
        let (pw, gp) = match (ga, gb) {
            (false, true) => (a, b),
            (true, false) => (b, a),
            _ => continue,
        };
        caps_by_net
            .entry(board.pads[pw].net.unwrap())
            .or_default()
            .push((r.clone(), pw, gp));
    }

    let power: Option<BTreeSet<NetId>> = match opt_strings(args, "power_nets")? {
        Some(p) => Some(common::nets_named(board, &p, "power_nets")?),
        None => None,
    };
    let min_caps = opt_count(args, "min_capacitors", 2, 100)? as usize;
    caps_by_net.retain(|n, caps| match &power {
        Some(set) => set.contains(n),
        None => caps.len() >= min_caps,
    });
    let mut ics = Vec::new();
    let mut markers = Vec::new();
    let mut nearest_use: BTreeMap<usize, Vec<String>> = BTreeMap::new();
    let mut networks: BTreeMap<(NetId, String), Result<Network, String>> = BTreeMap::new();
    for r in &refs {
        let fps: Vec<_> = board
            .footprints
            .iter()
            .filter(|f| f.reference.as_deref() == Some(r.as_str()))
            .collect();
        if fps.is_empty() {
            return Err(invalid("references", &format!("{r} is not on the board")));
        }
        let fp = fps[0];
        let range = fp.pad_range.0 as usize..fp.pad_range.1 as usize;
        let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
        for i in range.clone() {
            let c = pmm(board.pads[i].center);
            lo = [lo[0].min(c[0]), lo[1].min(c[1])];
            hi = [hi[0].max(c[0]), hi[1].max(c[1])];
        }
        let win = (
            [lo[0] - max_d, lo[1] - max_d],
            [hi[0] + max_d, hi[1] + max_d],
        );
        let mut pins = Vec::new();
        let mut gnd_pins = Vec::new();
        for i in range {
            let p = &board.pads[i];
            let Some(n) = p.net else { continue };
            if gnd.nets.contains(&n) {
                let near = gnd.nearest(board, i);
                if let Some((d, inpad, vi)) = near {
                    nearest_use.entry(vi).or_default().push(pad_label(board, p));
                    gnd_pins.push(json!({ "pin": pad_label(board, p), "nearest_gnd_via_mm": r4(if inpad { 0.0 } else { d }), "via_in_pad": inpad, "via": [r4(gnd.vias[vi].0[0]), r4(gnd.vias[vi].0[1])] }));
                } else {
                    gnd_pins.push(
                        json!({ "pin": pad_label(board, p), "nearest_gnd_via_mm": Value::Null }),
                    );
                }
                continue;
            }
            if only_pins.as_ref().is_some_and(|s| !s.contains(&i)) {
                continue;
            }
            let Some(caps) = caps_by_net.get(&n) else {
                continue;
            };
            let c = pmm(p.center);
            let wanted = assoc.get(&i);
            let candidates: Vec<&(String, usize, usize)> = caps
                .iter()
                .filter(|(cr, cp, _)| match wanted {
                    Some(list) => list.contains(cr),
                    None => dist(pmm(board.pads[*cp].center), c) <= max_d,
                })
                .collect();
            if candidates.is_empty() {
                pins.push(json!({ "pin": pad_label(board, p), "net": board.net(n).name, "capacitor": Value::Null, "note": format!("no capacitor of the net within {max_d} mm") }));
                continue;
            }
            let key = (n, r.clone());
            let nm = networks.entry(key).or_insert_with(|| {
                Network::build(
                    l,
                    n,
                    &Options {
                        h,
                        temp_c: 25.0,
                        plating_mm: 0.02,
                        max_cells: 6_000_000,
                        window: Some(win),
                    },
                )
            });
            let row = match nm {
                Err(e) => {
                    json!({ "pin": pad_label(board, p), "net": board.net(n).name, "error": e })
                }
                Ok(nm) => {
                    let Some(start) = nm.pad(i) else { continue };
                    let d = nm.path_lengths(&[start]);
                    let mut best: Option<(f64, &(String, usize, usize))> = None;
                    let mut all = Vec::new();
                    for cand in &candidates {
                        let len = nm
                            .pad(cand.1)
                            .map(|n| d[n as usize])
                            .unwrap_or(f64::INFINITY);
                        all.push((len, cand));
                        if best.is_none_or(|b| len < b.0) {
                            best = Some((len, cand));
                        }
                    }
                    all.sort_by(|a, b| a.0.total_cmp(&b.0));
                    match best {
                        Some((len, (cr, cp, cg))) if len.is_finite() => {
                            let cgn = gnd.nearest(board, *cg);
                            if let Some((_, _, vi)) = cgn {
                                nearest_use
                                    .entry(vi)
                                    .or_default()
                                    .push(pad_label(board, &board.pads[*cg]));
                            }
                            let cc = pmm(board.pads[*cp].center);
                            markers.push(marker(
                                (c[0] + cc[0]) / 2.0,
                                (c[1] + cc[1]) / 2.0,
                                "decoupling_path",
                                format!("{} -> {cr} {:.2} mm", pad_label(board, p), len),
                            ));
                            json!({
                                "pin": pad_label(board, p),
                                "net": board.net(n).name,
                                "capacitor": cr,
                                "path_mm": r4(len),
                                "straight_mm": r4(dist(c, cc)),
                                "capacitor_gnd_pad": pad_label(board, &board.pads[*cg]),
                                "capacitor_gnd_via_mm": cgn.map(|(d, inpad, _)| r4(if inpad { 0.0 } else { d })),
                                "capacitor_gnd_via_in_pad": cgn.map(|x| x.1),
                                "capacitor_gnd_via": cgn.map(|(_, _, vi)| json!([r4(gnd.vias[vi].0[0]), r4(gnd.vias[vi].0[1])])),
                                "next": all.iter().skip(1).take(2).map(|(l, c)| json!({ "capacitor": c.0, "path_mm": if l.is_finite() { json!(r4(*l)) } else { Value::Null } })).collect::<Vec<_>>(),
                            })
                        }
                        _ => {
                            json!({ "pin": pad_label(board, p), "net": board.net(n).name, "capacitor": Value::Null, "note": "no candidate capacitor is joined to the pin by copper inside the window" })
                        }
                    }
                }
            };
            pins.push(row);
        }
        ics.push(json!({ "reference": r, "power_pins": pins, "ground_pins": gnd_pins }));
    }
    let shared: Vec<Value> = nearest_use
        .iter()
        .filter(|(_, users)| users.len() > 1)
        .map(|(&vi, users)| json!({ "via": [r4(gnd.vias[vi].0[0]), r4(gnd.vias[vi].0[1])], "uuid": gnd.vias[vi].2, "shared_by": users }))
        .collect();
    for s in &shared {
        markers.push(marker(
            s["via"][0].as_f64().unwrap_or(0.0),
            s["via"][1].as_f64().unwrap_or(0.0),
            "shared_gnd_via",
            format!("{} users", s["shared_by"].as_array().map_or(0, Vec::len)),
        ));
    }
    let methods = json!({
        "power_pins": "IC pads on a power net: one named in power_nets, or else one with at least min_capacitors capacitors (reference prefix capacitor_prefix, two pads) whose other pad is a ground net",
        "path": "shortest copper path from the pin to each capacitor pad of the net: tracks along their length, via barrels over the stack depth, fills on a mesh (diagonal steps), inside the IC's pads' box grown by max_distance_mm",
        "ground": "nearest ground via (centre to centre; 0 when the via touches the pad) for each capacitor ground pad and IC ground pin; shared: one via nearest to several",
    });
    let full = merged(
        common::provenance(l),
        json!({ "ics": ics, "shared_gnd_vias": shared, "methods": methods }),
    );
    let files = out.write(&full, &markers)?;
    let trimmed: Vec<Value> = full["ics"]
        .as_array()
        .unwrap()
        .iter()
        .take(max_items)
        .map(|ic| {
            let pins = ic["power_pins"].as_array().cloned().unwrap_or_default();
            let worst = pins.iter().filter(|p| p["path_mm"].is_number()).max_by(|a, b| a["path_mm"].as_f64().unwrap().total_cmp(&b["path_mm"].as_f64().unwrap())).cloned();
            json!({
                "reference": ic["reference"],
                "power_pins": pins.len(),
                "without_capacitor": pins.iter().filter(|p| p["capacitor"].is_null()).count(),
                "longest_path": worst,
                "pins": super::hygiene::take(&ic["power_pins"], max_items),
                "ground_pins_without_via_within_1mm": ic["ground_pins"].as_array().map_or(0, |g| g.iter().filter(|x| x["nearest_gnd_via_mm"].as_f64().is_none_or(|d| d > 1.0)).count()),
            })
        })
        .collect();
    Ok(CallToolResult::json(&merged(
        common::provenance(l),
        json!({
            "ics": trimmed,
            "shared_gnd_vias": super::hygiene::take(&full["shared_gnd_vias"], max_items),
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
    fn the_path_follows_the_track_not_the_straight_line() {
        // U1.1 on VCC; C1 (VCC/GND) 4 mm east, joined by an L of 3 + 4 mm.
        let board = ts::board(&[
            "\t(footprint \"IC\" (layer \"F.Cu\") (at 10 10)\n\t\t(property \"Reference\" \"U1\" (at 0 -2 0) (layer \"F.SilkS\"))\n\t\t(pad \"1\" smd rect (at 0 0) (size 0.5 0.5) (layers \"F.Cu\") (net \"VCC\"))\n\t\t(pad \"2\" smd rect (at 0 2) (size 0.5 0.5) (layers \"F.Cu\") (net \"GND\")))\n".to_string(),
            "\t(footprint \"C\" (layer \"F.Cu\") (at 14 10)\n\t\t(property \"Reference\" \"C1\" (at 0 -2 0) (layer \"F.SilkS\"))\n\t\t(pad \"1\" smd rect (at 0 0) (size 0.5 0.5) (layers \"F.Cu\") (net \"VCC\"))\n\t\t(pad \"2\" smd rect (at 1 0) (size 0.5 0.5) (layers \"F.Cu\") (net \"GND\")))\n".to_string(),
            ts::segment(10.0, 10.0, 10.0, 13.0, 0.2, "F.Cu", "VCC"),
            ts::segment(10.0, 13.0, 14.0, 13.0, 0.2, "F.Cu", "VCC"),
            ts::segment(14.0, 13.0, 14.0, 10.0, 0.2, "F.Cu", "VCC"),
            ts::via(15.0, 11.0, 0.6, 0.3, "GND"),
        ]);
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("t.kicad_pcb");
        std::fs::write(&path, board).unwrap();
        let l = common::load_at(&path).unwrap();
        let out = Outputs {
            output: None,
            markers: None,
        };
        let r = check_decoupling_placement(
            &l,
            &json!({ "board": path, "references": ["U1"], "min_capacitors": 1 }),
            &out,
        )
        .unwrap();
        let b: Value = match &r.content[0] {
            crate::mcp::protocol::ToolContent::Text { text } => serde_json::from_str(text).unwrap(),
            _ => panic!(),
        };
        let pin = &b["ics"][0]["pins"][0];
        assert_eq!(pin["capacitor"], "C1", "{b}");
        assert!(
            (pin["path_mm"].as_f64().unwrap() - 10.0).abs() < 1e-6,
            "{pin}"
        );
        assert!((pin["straight_mm"].as_f64().unwrap() - 4.0).abs() < 1e-6);
        assert!(
            (pin["capacitor_gnd_via_mm"].as_f64().unwrap() - 1.0).abs() < 1e-6,
            "{pin}"
        );
    }
}
