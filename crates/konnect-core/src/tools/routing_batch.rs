//! `add_routing` — many tracks and through vias in one KiCad `CreateItems`.
//!
//! Escapes, differential pairs and power runs are hundreds of segments and
//! vias each; `route_trace` and `add_via` write one item per call. This takes
//! the whole batch, inline or from a JSON file, validates every member before
//! KiCad sees any of it, resolves every net name in one `GetNets`, and creates
//! everything in a single request (one undo step). Items can be created
//! locked, which is how deliberate copper is kept out of a later autorouter's
//! reach.

use konnect_ipc::builders;
use konnect_ipc::gen::kiapi::board::types::BoardLayer;
use serde_json::json;
use std::collections::BTreeMap;

pub(crate) const MAX_ITEMS: usize = 20_000;
const MAX_WIDTH_MM: f64 = 20.0;
const MAX_VIA_MM: f64 = 10.0;

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Track {
    pub net: String,
    pub layer: String,
    pub width: f64,
    pub start: (f64, f64),
    pub end: (f64, f64),
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Via {
    pub net: String,
    pub at: (f64, f64),
    pub drill: f64,
    pub diameter: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Batch {
    pub tracks: Vec<Track>,
    pub vias: Vec<Via>,
    pub locked: bool,
}

type Refusal = (String, String);

fn refuse(field: impl Into<String>, reason: impl Into<String>) -> Refusal {
    (field.into(), reason.into())
}

fn obj<'a>(
    v: &'a serde_json::Value,
    at: &str,
    keys: &[&str],
) -> Result<&'a serde_json::Map<String, serde_json::Value>, Refusal> {
    let Some(o) = v.as_object() else {
        return Err(refuse(at, "must be an object"));
    };
    if let Some(k) = o.keys().find(|k| !keys.contains(&k.as_str())) {
        return Err(refuse(format!("{at}.{k}"), "is not a known member key"));
    }
    Ok(o)
}

fn num(
    o: &serde_json::Map<String, serde_json::Value>,
    at: &str,
    key: &str,
) -> Result<f64, Refusal> {
    match o.get(key).and_then(serde_json::Value::as_f64) {
        Some(v) if v.is_finite() => Ok(v),
        _ => Err(refuse(format!("{at}.{key}"), "must be a finite number")),
    }
}

fn net(o: &serde_json::Map<String, serde_json::Value>, at: &str) -> Result<String, Refusal> {
    match o.get("net").and_then(serde_json::Value::as_str) {
        Some(n) if !n.is_empty() => Ok(n.to_string()),
        _ => Err(refuse(format!("{at}.net"), "must be a non-empty net name")),
    }
}

fn copper_layer(name: &str) -> bool {
    name.ends_with(".Cu") && builders::layer_from_name(name) != BoardLayer::BlUndefined
}

fn list<'a>(source: &'a serde_json::Value, key: &str) -> Result<&'a [serde_json::Value], Refusal> {
    match source.get(key) {
        None | Some(serde_json::Value::Null) => Ok(&[]),
        Some(serde_json::Value::Array(items)) => Ok(items),
        Some(_) => Err(refuse(key, "must be an array")),
    }
}

/// Parse and validate a request. `file` is the parsed `routing_file`.
pub(crate) fn parse(
    args: &serde_json::Value,
    file: Option<&serde_json::Value>,
) -> Result<Batch, Refusal> {
    let inline = ["tracks", "vias"]
        .iter()
        .any(|k| args.get(*k).is_some_and(|v| !v.is_null()));
    let source = match (file, inline) {
        (Some(_), true) => {
            return Err(refuse(
                "routing_file",
                "give the items inline or in routing_file, not both",
            ))
        }
        (Some(f), false) => {
            let Some(o) = f.as_object() else {
                return Err(refuse(
                    "routing_file",
                    "must hold a JSON object with tracks and/or vias",
                ));
            };
            if let Some(k) = o.keys().find(|k| !["tracks", "vias"].contains(&k.as_str())) {
                return Err(refuse(
                    "routing_file",
                    format!("key '{k}' is not tracks or vias"),
                ));
            }
            f
        }
        (None, _) => args,
    };
    let locked = args
        .get("locked")
        .and_then(serde_json::Value::as_bool)
        .unwrap_or(false);

    let mut tracks = Vec::new();
    for (i, item) in list(source, "tracks")?.iter().enumerate() {
        let at = format!("tracks[{i}]");
        let o = obj(
            item,
            &at,
            &["net", "layer", "width", "x1", "y1", "x2", "y2"],
        )?;
        let layer = match o.get("layer").and_then(serde_json::Value::as_str) {
            Some(l) if copper_layer(l) => l.to_string(),
            Some(l) => {
                return Err(refuse(
                    format!("{at}.layer"),
                    format!("'{l}' is not a copper layer name"),
                ))
            }
            None => return Err(refuse(format!("{at}.layer"), "is required")),
        };
        let width = num(o, &at, "width")?;
        if !(width > 0.0 && width <= MAX_WIDTH_MM) {
            return Err(refuse(
                format!("{at}.width"),
                format!("must be above 0 and at most {MAX_WIDTH_MM} mm"),
            ));
        }
        let t = Track {
            net: net(o, &at)?,
            layer,
            width,
            start: (num(o, &at, "x1")?, num(o, &at, "y1")?),
            end: (num(o, &at, "x2")?, num(o, &at, "y2")?),
        };
        if t.start == t.end {
            return Err(refuse(at, "has zero length"));
        }
        tracks.push(t);
    }
    let mut vias = Vec::new();
    for (i, item) in list(source, "vias")?.iter().enumerate() {
        let at = format!("vias[{i}]");
        let o = obj(item, &at, &["net", "x", "y", "drill", "diameter"])?;
        let drill = num(o, &at, "drill")?;
        let diameter = num(o, &at, "diameter")?;
        if !(drill > 0.0 && diameter > drill && diameter <= MAX_VIA_MM) {
            return Err(refuse(
                at,
                format!("needs 0 < drill < diameter <= {MAX_VIA_MM} mm"),
            ));
        }
        vias.push(Via {
            net: net(o, &at)?,
            at: (num(o, &at, "x")?, num(o, &at, "y")?),
            drill,
            diameter,
        });
    }
    let total = tracks.len() + vias.len();
    if total == 0 {
        return Err(refuse("tracks", "at least one track or via is required"));
    }
    if total > MAX_ITEMS {
        return Err(refuse(
            "tracks",
            format!("{total} items exceed the {MAX_ITEMS} one call may carry"),
        ));
    }
    Ok(Batch {
        tracks,
        vias,
        locked,
    })
}

impl Batch {
    pub(crate) fn nets(&self) -> Vec<String> {
        let mut nets: Vec<String> = self
            .tracks
            .iter()
            .map(|t| t.net.clone())
            .chain(self.vias.iter().map(|v| v.net.clone()))
            .collect();
        nets.sort();
        nets.dedup();
        nets
    }

    /// Build every item, or name the nets the board does not have.
    pub(crate) fn items(
        &self,
        codes: &BTreeMap<String, i32>,
    ) -> Result<Vec<prost_types::Any>, Vec<String>> {
        let missing: Vec<String> = self
            .nets()
            .into_iter()
            .filter(|n| !codes.contains_key(n))
            .collect();
        if !missing.is_empty() {
            return Err(missing);
        }
        let lock = if self.locked {
            konnect_ipc::gen::kiapi::common::types::LockedState::LsLocked
        } else {
            konnect_ipc::gen::kiapi::common::types::LockedState::LsUnlocked
        } as i32;
        let mut out = Vec::with_capacity(self.tracks.len() + self.vias.len());
        for t in &self.tracks {
            let mut track = builders::build_track(
                &t.net,
                codes[&t.net],
                &t.layer,
                t.width,
                t.start.0,
                t.start.1,
                t.end.0,
                t.end.1,
            );
            track.locked = lock;
            out.push(builders::pack_any(&track, "kiapi.board.types.Track"));
        }
        for v in &self.vias {
            let mut via =
                builders::build_via(&v.net, codes[&v.net], v.at.0, v.at.1, v.drill, v.diameter);
            via.locked = lock;
            out.push(builders::pack_any(&via, "kiapi.board.types.Via"));
        }
        Ok(out)
    }

    /// The batch's tracks as segments for the geometry lint.
    pub(crate) fn segments(&self) -> Vec<super::route_geometry::Segment> {
        self.tracks
            .iter()
            .map(|t| super::route_geometry::Segment {
                net: t.net.clone(),
                layer: t.layer.clone(),
                width: t.width,
                a: t.start,
                b: t.end,
            })
            .collect()
    }

    pub(crate) fn summary(&self) -> serde_json::Value {
        let mut per_layer: BTreeMap<&str, usize> = BTreeMap::new();
        for t in &self.tracks {
            *per_layer.entry(t.layer.as_str()).or_default() += 1;
        }
        json!({
            "tracks": self.tracks.len(),
            "vias": self.vias.len(),
            "nets": self.nets().len(),
            "tracks_per_layer": per_layer,
            "locked": self.locked,
        })
    }
}

pub(crate) fn schema() -> serde_json::Value {
    let n = |d: &str| json!({ "type": "number", "description": d });
    let mut schema = json!({
        "type": "object",
        "properties": {
            "board": { "type": "string", "description": "Path to the .kicad_pcb KiCad holds open" },
            "tracks": {
                "type": "array",
                "items": {
                    "type": "object",
                    "additionalProperties": false,
                    "properties": {
                        "net": { "type": "string" },
                        "layer": { "type": "string", "description": "Copper layer, e.g. 'F.Cu', 'In2.Cu'" },
                        "width": n("Track width, mm"),
                        "x1": n("Start X, mm"), "y1": n("Start Y, mm"),
                        "x2": n("End X, mm"), "y2": n("End Y, mm")
                    },
                    "required": ["net", "layer", "width", "x1", "y1", "x2", "y2"]
                }
            },
            "vias": {
                "type": "array",
                "items": {
                    "type": "object",
                    "additionalProperties": false,
                    "properties": {
                        "net": { "type": "string" },
                        "x": n("X, mm"), "y": n("Y, mm"),
                        "drill": n("Drill diameter, mm"),
                        "diameter": n("Copper diameter, mm")
                    },
                    "required": ["net", "x", "y", "drill", "diameter"]
                }
            },
            "routing_file": { "type": "string", "description": "Path to a JSON object {tracks, vias} used instead of the inline items" },
            "locked": { "type": "boolean", "default": false, "description": "Create every item locked, so later bulk edits and autorouters leave it alone" }
        },
        "required": ["board"]
    });
    if let (Some(props), Some(extra)) = (
        schema["properties"].as_object_mut(),
        super::route_geometry::schema_properties().as_object(),
    ) {
        props.extend(extra.clone());
    }
    schema
}

// ─── delete_routing: selecting existing copper ────────────────────────────────

/// One routed item read back from KiCad, reduced to what selection needs.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct CopperItem {
    pub id: String,
    pub kind: &'static str,
    pub net: String,
    pub layer: String,
    pub points: Vec<(f64, f64)>,
    pub locked: bool,
}

/// Which items `delete_routing` removes. Every given filter must match.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Selection {
    pub area: Option<(f64, f64, f64, f64)>,
    pub nets: Option<Vec<String>>,
    pub kinds: Vec<String>,
    pub include_locked: bool,
}

pub(crate) fn parse_selection(args: &serde_json::Value) -> Result<Selection, Refusal> {
    let area = match args.get("area").filter(|v| !v.is_null()) {
        None => None,
        Some(a) => {
            let o = obj(a, "area", &["x1", "y1", "x2", "y2"])?;
            let (x1, y1, x2, y2) = (
                num(o, "area", "x1")?,
                num(o, "area", "y1")?,
                num(o, "area", "x2")?,
                num(o, "area", "y2")?,
            );
            if x1 >= x2 || y1 >= y2 {
                return Err(refuse("area", "needs x1 < x2 and y1 < y2"));
            }
            Some((x1, y1, x2, y2))
        }
    };
    let nets = match args.get("nets").filter(|v| !v.is_null()) {
        None => None,
        Some(v) => {
            let list: Option<Vec<String>> = v.as_array().map(|a| {
                a.iter()
                    .filter_map(|n| n.as_str().filter(|s| !s.is_empty()).map(String::from))
                    .collect()
            });
            match list {
                Some(l) if !l.is_empty() && l.len() == v.as_array().map_or(0, Vec::len) => Some(l),
                _ => return Err(refuse("nets", "must be a non-empty array of net names")),
            }
        }
    };
    if area.is_none() && nets.is_none() {
        return Err(refuse(
            "area",
            "give an area, nets, or both: deleting all routing is not offered",
        ));
    }
    let kinds = match args.get("kinds").filter(|v| !v.is_null()) {
        None => vec!["track".to_string(), "arc".to_string(), "via".to_string()],
        Some(v) => {
            let list: Vec<String> = v
                .as_array()
                .map(|a| {
                    a.iter()
                        .filter_map(|k| k.as_str().map(String::from))
                        .collect()
                })
                .unwrap_or_default();
            if list.is_empty()
                || list
                    .iter()
                    .any(|k| !["track", "arc", "via"].contains(&k.as_str()))
            {
                return Err(refuse("kinds", "must list some of track, arc and via"));
            }
            list
        }
    };
    let include_locked = args
        .get("include_locked")
        .and_then(serde_json::Value::as_bool)
        .unwrap_or(false);
    Ok(Selection {
        area,
        nets,
        kinds,
        include_locked,
    })
}

impl Selection {
    /// (selected, locked items the filters matched but that were kept)
    pub(crate) fn pick<'a>(&self, items: &'a [CopperItem]) -> (Vec<&'a CopperItem>, usize) {
        let mut picked = Vec::new();
        let mut kept_locked = 0;
        for item in items {
            if !self.kinds.iter().any(|k| k == item.kind) {
                continue;
            }
            if let Some(nets) = &self.nets {
                if !nets.contains(&item.net) {
                    continue;
                }
            }
            if let Some((x1, y1, x2, y2)) = self.area {
                if !item
                    .points
                    .iter()
                    .all(|&(x, y)| x >= x1 && x <= x2 && y >= y1 && y <= y2)
                {
                    continue;
                }
            }
            if item.locked && !self.include_locked {
                kept_locked += 1;
                continue;
            }
            picked.push(item);
        }
        (picked, kept_locked)
    }
}

pub(crate) fn summarize(items: &[&CopperItem]) -> serde_json::Value {
    let mut kinds: BTreeMap<&str, usize> = BTreeMap::new();
    let mut nets: BTreeMap<&str, usize> = BTreeMap::new();
    for i in items {
        *kinds.entry(i.kind).or_default() += 1;
        *nets.entry(i.net.as_str()).or_default() += 1;
    }
    json!({ "count": items.len(), "by_kind": kinds, "nets": nets.len(), "locked": items.iter().filter(|i| i.locked).count() })
}

/// Decode KiCad's tracks, arcs and vias into `CopperItem`s.
pub(crate) fn decode_copper(items: &[prost_types::Any]) -> anyhow::Result<Vec<CopperItem>> {
    use konnect_ipc::gen::kiapi::board::types as b;
    use prost::Message;
    let pt = |v: &Option<konnect_ipc::gen::kiapi::common::types::Vector2>| {
        v.as_ref()
            .map(|v| (builders::nm_to_mm(v.x_nm), builders::nm_to_mm(v.y_nm)))
    };
    let locked = |l: i32| l == konnect_ipc::gen::kiapi::common::types::LockedState::LsLocked as i32;
    let layer = |l: i32| {
        BoardLayer::try_from(l)
            .ok()
            .and_then(builders::layer_name)
            .unwrap_or("?")
            .to_string()
    };
    let mut out = Vec::new();
    for any in items {
        if builders::any_is(any, "kiapi.board.types.Track") {
            let m = b::Track::decode(any.value.as_slice())?;
            out.push(CopperItem {
                id: m.id.map(|k| k.value).unwrap_or_default(),
                kind: "track",
                net: m.net.map(|n| n.name).unwrap_or_default(),
                layer: layer(m.layer),
                points: [pt(&m.start), pt(&m.end)].into_iter().flatten().collect(),
                locked: locked(m.locked),
            });
        } else if builders::any_is(any, "kiapi.board.types.Arc") {
            let m = b::Arc::decode(any.value.as_slice())?;
            out.push(CopperItem {
                id: m.id.map(|k| k.value).unwrap_or_default(),
                kind: "arc",
                net: m.net.map(|n| n.name).unwrap_or_default(),
                layer: layer(m.layer),
                points: [pt(&m.start), pt(&m.mid), pt(&m.end)]
                    .into_iter()
                    .flatten()
                    .collect(),
                locked: locked(m.locked),
            });
        } else if builders::any_is(any, "kiapi.board.types.Via") {
            let m = b::Via::decode(any.value.as_slice())?;
            out.push(CopperItem {
                id: m.id.map(|k| k.value).unwrap_or_default(),
                kind: "via",
                net: m.net.map(|n| n.name).unwrap_or_default(),
                layer: "through".to_string(),
                points: pt(&m.position).into_iter().collect(),
                locked: locked(m.locked),
            });
        }
    }
    anyhow::ensure!(
        out.iter().all(|i| !i.id.is_empty()),
        "KiCad sent a routed item without a KIID"
    );
    Ok(out)
}

pub(crate) fn delete_schema() -> serde_json::Value {
    json!({
        "type": "object",
        "properties": {
            "board": { "type": "string", "description": "Path to the .kicad_pcb KiCad holds open" },
            "area": {
                "type": "object",
                "additionalProperties": false,
                "description": "Only items lying wholly inside this box (every end point / the via centre)",
                "properties": { "x1": { "type": "number" }, "y1": { "type": "number" }, "x2": { "type": "number" }, "y2": { "type": "number" } },
                "required": ["x1", "y1", "x2", "y2"]
            },
            "nets": { "type": "array", "items": { "type": "string" }, "description": "Only items on these nets" },
            "kinds": { "type": "array", "items": { "type": "string", "enum": ["track", "arc", "via"] }, "description": "Default: all three" },
            "include_locked": { "type": "boolean", "default": false, "description": "Also delete locked items; otherwise they are kept and counted" },
            "dry_run": { "type": "boolean", "default": true },
            "expected_count": { "type": "integer", "minimum": 1, "description": "Required to apply: the count a dry run of the same selection returned" }
        },
        "required": ["board"]
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn item(
        id: &str,
        kind: &'static str,
        net: &str,
        points: Vec<(f64, f64)>,
        locked: bool,
    ) -> CopperItem {
        CopperItem {
            id: id.into(),
            kind,
            net: net.into(),
            layer: "F.Cu".into(),
            points,
            locked,
        }
    }

    #[test]
    fn selection_needs_every_filter_and_keeps_locked_by_default() {
        let items = vec![
            item("a", "track", "GND", vec![(1.0, 1.0), (2.0, 1.0)], false),
            item("b", "via", "GND", vec![(1.5, 1.5)], true),
            item("c", "track", "VCC", vec![(1.0, 1.0), (2.0, 1.0)], false),
            item("d", "track", "GND", vec![(1.0, 1.0), (9.0, 1.0)], false),
        ];
        let s = parse_selection(
            &json!({ "area": { "x1": 0, "y1": 0, "x2": 3, "y2": 3 }, "nets": ["GND"] }),
        )
        .unwrap();
        let (picked, kept) = s.pick(&items);
        assert_eq!(
            picked.iter().map(|i| i.id.as_str()).collect::<Vec<_>>(),
            ["a"]
        );
        assert_eq!(kept, 1, "the locked via matched but was kept");
        let s = parse_selection(&json!({ "area": { "x1": 0, "y1": 0, "x2": 3, "y2": 3 }, "include_locked": true, "kinds": ["via"] })).unwrap();
        assert_eq!(
            s.pick(&items)
                .0
                .iter()
                .map(|i| i.id.as_str())
                .collect::<Vec<_>>(),
            ["b"]
        );
        assert_eq!(summarize(&s.pick(&items).0)["by_kind"], json!({ "via": 1 }));
    }

    #[test]
    fn a_selection_without_area_or_nets_or_with_bad_values_is_refused() {
        assert_eq!(parse_selection(&json!({})).unwrap_err().0, "area");
        assert_eq!(
            parse_selection(&json!({ "area": { "x1": 3, "y1": 0, "x2": 1, "y2": 3 } }))
                .unwrap_err()
                .0,
            "area"
        );
        assert_eq!(
            parse_selection(&json!({ "nets": [] })).unwrap_err().0,
            "nets"
        );
        assert_eq!(
            parse_selection(&json!({ "nets": ["A"], "kinds": ["zone"] }))
                .unwrap_err()
                .0,
            "kinds"
        );
    }

    #[test]
    fn decoded_copper_carries_net_points_and_lock() {
        let codes = BTreeMap::from([("GND".to_string(), 1)]);
        let mut args = sample();
        args["locked"] = json!(true);
        args["tracks"] = json!([{ "net": "GND", "layer": "F.Cu", "width": 0.2, "x1": 0, "y1": 0, "x2": 5, "y2": 0 }]);
        let mut items = parse(&args, None).unwrap().items(&codes).unwrap();
        // KiCad assigns KIIDs; give the round-trip some
        for (n, any) in items.iter_mut().enumerate() {
            use prost::Message;
            if any.type_url.ends_with("Track") {
                let mut t =
                    konnect_ipc::gen::kiapi::board::types::Track::decode(any.value.as_slice())
                        .unwrap();
                t.id = Some(konnect_ipc::gen::kiapi::common::types::Kiid {
                    value: format!("t{n}"),
                });
                *any = builders::pack_any(&t, "kiapi.board.types.Track");
            } else {
                let mut v =
                    konnect_ipc::gen::kiapi::board::types::Via::decode(any.value.as_slice())
                        .unwrap();
                v.id = Some(konnect_ipc::gen::kiapi::common::types::Kiid {
                    value: format!("v{n}"),
                });
                *any = builders::pack_any(&v, "kiapi.board.types.Via");
            }
        }
        let copper = decode_copper(&items).unwrap();
        assert_eq!(copper.len(), 2);
        assert_eq!(
            (
                copper[0].kind,
                copper[0].net.as_str(),
                copper[0].points.len(),
                copper[0].locked
            ),
            ("track", "GND", 2, true)
        );
        assert_eq!((copper[1].kind, copper[1].points[0]), ("via", (5.0, 0.0)));
    }

    fn sample() -> serde_json::Value {
        json!({
            "tracks": [
                { "net": "GND", "layer": "F.Cu", "width": 0.2, "x1": 0, "y1": 0, "x2": 5, "y2": 0 },
                { "net": "VCC", "layer": "In2.Cu", "width": 0.3, "x1": 5, "y1": 0, "x2": 5, "y2": 5 }
            ],
            "vias": [{ "net": "GND", "x": 5, "y": 0, "drill": 0.2, "diameter": 0.45 }]
        })
    }

    fn field(extra: serde_json::Value) -> String {
        parse(&extra, None).unwrap_err().0
    }

    #[test]
    fn a_valid_batch_builds_one_item_per_member() {
        let b = parse(&sample(), None).unwrap();
        assert_eq!(b.nets(), ["GND", "VCC"]);
        let codes = BTreeMap::from([("GND".to_string(), 1), ("VCC".to_string(), 2)]);
        let items = b.items(&codes).unwrap();
        assert_eq!(items.len(), 3);
        assert!(items[0].type_url.ends_with("Track") && items[2].type_url.ends_with("Via"));
        assert_eq!(
            b.summary()["tracks_per_layer"],
            json!({ "F.Cu": 1, "In2.Cu": 1 })
        );
    }

    #[test]
    fn unknown_nets_are_named_and_nothing_is_built() {
        let b = parse(&sample(), None).unwrap();
        let codes = BTreeMap::from([("GND".to_string(), 1)]);
        assert_eq!(b.items(&codes).unwrap_err(), ["VCC"]);
    }

    #[test]
    fn locked_items_are_sent_locked() {
        use prost::Message;
        let mut args = sample();
        args["locked"] = json!(true);
        let b = parse(&args, None).unwrap();
        let codes = BTreeMap::from([("GND".to_string(), 1), ("VCC".to_string(), 2)]);
        let items = b.items(&codes).unwrap();
        let track = konnect_ipc::gen::kiapi::board::types::Track::decode(items[0].value.as_slice())
            .unwrap();
        let via =
            konnect_ipc::gen::kiapi::board::types::Via::decode(items[2].value.as_slice()).unwrap();
        let locked = konnect_ipc::gen::kiapi::common::types::LockedState::LsLocked as i32;
        assert_eq!((track.locked, via.locked), (locked, locked));
    }

    #[test]
    fn malformed_members_are_refused_by_field() {
        assert_eq!(
            field(
                json!({ "tracks": [{ "net": "A", "layer": "F.SilkS", "width": 0.2, "x1": 0, "y1": 0, "x2": 1, "y2": 0 }] })
            ),
            "tracks[0].layer"
        );
        assert_eq!(
            field(
                json!({ "tracks": [{ "net": "A", "layer": "F.Cu", "width": 0, "x1": 0, "y1": 0, "x2": 1, "y2": 0 }] })
            ),
            "tracks[0].width"
        );
        assert_eq!(
            field(
                json!({ "tracks": [{ "net": "A", "layer": "F.Cu", "width": 0.2, "x1": 1, "y1": 1, "x2": 1, "y2": 1 }] })
            ),
            "tracks[0]"
        );
        assert_eq!(
            field(
                json!({ "tracks": [{ "net": "", "layer": "F.Cu", "width": 0.2, "x1": 0, "y1": 0, "x2": 1, "y2": 0 }] })
            ),
            "tracks[0].net"
        );
        assert_eq!(
            field(
                json!({ "vias": [{ "net": "A", "x": 0, "y": 0, "drill": 0.5, "diameter": 0.4 }] })
            ),
            "vias[0]"
        );
        assert_eq!(
            field(
                json!({ "vias": [{ "net": "A", "x": 0, "y": 0, "drill": 0.2, "diameter": 0.45, "type": "blind" }] })
            ),
            "vias[0].type"
        );
        assert_eq!(field(json!({})), "tracks");
    }

    #[test]
    fn file_and_inline_items_are_exclusive() {
        let file = sample();
        assert_eq!(
            parse(
                &json!({ "tracks": [] , "vias": file["vias"].clone() }),
                Some(&file)
            )
            .unwrap_err()
            .0,
            "routing_file"
        );
        assert_eq!(parse(&json!({}), Some(&file)).unwrap().tracks.len(), 2);
        assert_eq!(
            parse(&json!({}), Some(&json!({ "zones": [] })))
                .unwrap_err()
                .0,
            "routing_file"
        );
    }
}
