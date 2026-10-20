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
use std::collections::{BTreeMap, BTreeSet};

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

/// Most files one `routing_files` list may name.
pub(crate) const MAX_FILES: usize = 256;

/// One member of `routing_files`, as parsed: its path and its counts.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct FileCounts {
    pub path: String,
    pub tracks: usize,
    pub vias: usize,
}

fn read_json(path: &str) -> Result<serde_json::Value, String> {
    std::fs::read_to_string(path)
        .map_err(|e| e.to_string())
        .and_then(|t| serde_json::from_str::<serde_json::Value>(&t).map_err(|e| e.to_string()))
}

/// Parse a whole `add_routing` request: inline items, one `routing_file`,
/// or `routing_files` (at most one of the three). The files of a list are
/// read and validated in order, each on its own so a refusal names the file
/// and its index (`routing_files[1]`, with the path and the bad member in
/// the reason), then merged into one batch: tracks in file order, then vias
/// in file order. The merged batch is held to the per-call item cap.
pub(crate) fn parse_request(args: &serde_json::Value) -> Result<(Batch, Vec<FileCounts>), Refusal> {
    let present = |k: &str| args.get(k).is_some_and(|v| !v.is_null());
    let inline = present("tracks") || present("vias");
    match (present("routing_file"), present("routing_files")) {
        (true, true) => {
            return Err(refuse(
                "routing_files",
                "give routing_file or routing_files, not both",
            ))
        }
        (false, true) if inline => {
            return Err(refuse(
                "routing_files",
                "give the items inline or in routing_files, not both",
            ))
        }
        (false, true) => {}
        (true, false) => {
            let Some(path) = args.get("routing_file").and_then(serde_json::Value::as_str) else {
                return Err(refuse("routing_file", "must be a path"));
            };
            let file = read_json(path)
                .map_err(|e| refuse("routing_file", format!("cannot be read as JSON: {e}")))?;
            return parse(args, Some(&file)).map(|b| (b, Vec::new()));
        }
        (false, false) => return parse(args, None).map(|b| (b, Vec::new())),
    }
    let Some(list) = args
        .get("routing_files")
        .and_then(serde_json::Value::as_array)
    else {
        return Err(refuse("routing_files", "must be an array of paths"));
    };
    if list.is_empty() || list.len() > MAX_FILES {
        return Err(refuse(
            "routing_files",
            format!("must name from 1 to {MAX_FILES} files"),
        ));
    }
    let mut merged = Batch {
        tracks: Vec::new(),
        vias: Vec::new(),
        locked: args
            .get("locked")
            .and_then(serde_json::Value::as_bool)
            .unwrap_or(false),
    };
    let mut counts = Vec::with_capacity(list.len());
    for (i, item) in list.iter().enumerate() {
        let field = format!("routing_files[{i}]");
        let Some(path) = item.as_str().filter(|p| !p.is_empty()) else {
            return Err(refuse(field, "must be a non-empty path"));
        };
        let file = read_json(path)
            .map_err(|e| refuse(&field, format!("{path} cannot be read as JSON: {e}")))?;
        let batch = parse(&serde_json::json!({}), Some(&file)).map_err(|(member, reason)| {
            let member = if member == "routing_file" {
                String::new()
            } else {
                format!("{member} ")
            };
            refuse(&field, format!("{path}: {member}{reason}"))
        })?;
        counts.push(FileCounts {
            path: path.to_string(),
            tracks: batch.tracks.len(),
            vias: batch.vias.len(),
        });
        merged.tracks.extend(batch.tracks);
        merged.vias.extend(batch.vias);
    }
    let total = merged.tracks.len() + merged.vias.len();
    if total > MAX_ITEMS {
        return Err(refuse(
            "routing_files",
            format!("{total} items in all exceed the {MAX_ITEMS} one call may carry"),
        ));
    }
    Ok((merged, counts))
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
            "routing_files": { "type": "array", "items": { "type": "string", "minLength": 1 }, "minItems": 1, "maxItems": MAX_FILES,
                               "description": "Instead of routing_file: several routing files, merged in order (tracks, then vias, file by file) and validated and linted as one batch; a bad member is refused naming its file and index" },
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
    /// Exact KIIDs, from `uuids` or `uuids_file`. Each must name a track,
    /// arc or via of the board, or nothing is deleted.
    pub uuids: Option<BTreeSet<String>>,
    pub kinds: Vec<String>,
    pub include_locked: bool,
}

/// Most UUIDs one call may name.
pub(crate) const MAX_UUIDS: usize = MAX_ITEMS;

fn uuid_list(v: &serde_json::Value, field: &str) -> Result<BTreeSet<String>, Refusal> {
    let Some(items) = v.as_array() else {
        return Err(refuse(field, "must be an array of UUID strings"));
    };
    if items.is_empty() || items.len() > MAX_UUIDS {
        return Err(refuse(
            field,
            format!("must hold from 1 to {MAX_UUIDS} UUIDs"),
        ));
    }
    let mut out = BTreeSet::new();
    for (i, item) in items.iter().enumerate() {
        match item.as_str().map(str::trim) {
            Some(u) if !u.is_empty() => {
                out.insert(u.to_string());
            }
            _ => {
                return Err(refuse(
                    format!("{field}[{i}]"),
                    "must be a non-empty UUID string",
                ))
            }
        }
    }
    Ok(out)
}

/// `uuids` inline, or `uuids_file`: a JSON array of UUIDs, or an object
/// whose `delete_uuids` (as `plan_dogleg_fixes` writes it) or `uuids` holds
/// one.
fn uuids_arg(args: &serde_json::Value) -> Result<Option<BTreeSet<String>>, Refusal> {
    let inline = args.get("uuids").filter(|v| !v.is_null());
    let file = args.get("uuids_file").filter(|v| !v.is_null());
    match (inline, file) {
        (Some(_), Some(_)) => Err(refuse("uuids_file", "give uuids or uuids_file, not both")),
        (Some(v), None) => uuid_list(v, "uuids").map(Some),
        (None, Some(f)) => {
            let Some(path) = f.as_str().filter(|p| !p.is_empty()) else {
                return Err(refuse("uuids_file", "must be a path"));
            };
            let v: serde_json::Value = std::fs::read_to_string(path)
                .map_err(|e| e.to_string())
                .and_then(|t| serde_json::from_str(&t).map_err(|e| e.to_string()))
                .map_err(|e| refuse("uuids_file", format!("cannot be read as JSON: {e}")))?;
            let list = match &v {
                serde_json::Value::Array(_) => &v,
                serde_json::Value::Object(o) => {
                    match o.get("delete_uuids").or_else(|| o.get("uuids")) {
                        Some(list) => list,
                        None => {
                            return Err(refuse(
                                "uuids_file",
                                "holds neither an array nor a delete_uuids or uuids list",
                            ))
                        }
                    }
                }
                _ => return Err(refuse("uuids_file", "must hold an array of UUIDs")),
            };
            uuid_list(list, "uuids_file").map(Some)
        }
        (None, None) => Ok(None),
    }
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
    let uuids = uuids_arg(args)?;
    if area.is_none() && nets.is_none() && uuids.is_none() {
        return Err(refuse(
            "area",
            "give an area, nets or uuids (any combination): deleting all routing is not offered",
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
        uuids,
        kinds,
        include_locked,
    })
}

impl Selection {
    /// The named UUIDs that are not a track, arc or via of `items`, sorted.
    pub(crate) fn unknown_uuids(&self, items: &[CopperItem]) -> Vec<String> {
        let Some(uuids) = &self.uuids else {
            return Vec::new();
        };
        let present: BTreeSet<&str> = items.iter().map(|i| i.id.as_str()).collect();
        uuids
            .iter()
            .filter(|u| !present.contains(u.as_str()))
            .cloned()
            .collect()
    }

    /// (selected, locked items the filters matched but that were kept)
    pub(crate) fn pick<'a>(&self, items: &'a [CopperItem]) -> (Vec<&'a CopperItem>, usize) {
        let mut picked = Vec::new();
        let mut kept_locked = 0;
        for item in items {
            if !self.kinds.iter().any(|k| k == item.kind) {
                continue;
            }
            if let Some(uuids) = &self.uuids {
                if !uuids.contains(&item.id) {
                    continue;
                }
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
            "uuids": { "type": "array", "items": { "type": "string", "minLength": 1 }, "minItems": 1, "maxItems": MAX_UUIDS, "description": "Only the tracks, arcs and vias with these UUIDs; every one must be a track, arc or via of the board or nothing is deleted. Alone or with the other filters (every filter must match)" },
            "uuids_file": { "type": "string", "description": "Instead of uuids: a JSON file holding an array of UUIDs, or an object with a delete_uuids array (the output_fixes of plan_dogleg_fixes) or a uuids array" },
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
    fn uuids_select_alone_or_with_other_filters_and_name_the_unknown() {
        let items = vec![
            item("a", "track", "GND", vec![(1.0, 1.0), (2.0, 1.0)], false),
            item("b", "via", "GND", vec![(1.5, 1.5)], true),
            item("c", "track", "VCC", vec![(1.0, 1.0), (2.0, 1.0)], false),
        ];
        let s = parse_selection(&json!({ "uuids": ["a", "c", "b"] })).unwrap();
        let (picked, kept) = s.pick(&items);
        assert_eq!(
            picked.iter().map(|i| i.id.as_str()).collect::<Vec<_>>(),
            ["a", "c"]
        );
        assert_eq!(kept, 1, "the locked via is kept without include_locked");
        let s = parse_selection(&json!({ "uuids": ["a", "c"], "nets": ["VCC"] })).unwrap();
        assert_eq!(
            s.pick(&items)
                .0
                .iter()
                .map(|i| i.id.as_str())
                .collect::<Vec<_>>(),
            ["c"],
            "uuids and nets intersect"
        );
        let s = parse_selection(&json!({ "uuids": ["a", "zone-1", "missing"] })).unwrap();
        assert_eq!(s.unknown_uuids(&items), ["missing", "zone-1"]);

        let dir = tempfile::tempdir().unwrap();
        let file = dir.path().join("fixes.json");
        std::fs::write(&file, r#"{ "delete_uuids": ["b", "a"], "fixes": [] }"#).unwrap();
        let s = parse_selection(&json!({ "uuids_file": file, "include_locked": true })).unwrap();
        assert_eq!(s.pick(&items).0.len(), 2);
        let list = dir.path().join("list.json");
        std::fs::write(&list, r#"["c"]"#).unwrap();
        assert_eq!(
            parse_selection(&json!({ "uuids_file": list }))
                .unwrap()
                .uuids
                .unwrap()
                .len(),
            1
        );
        for (args, field) in [
            (json!({ "uuids": [] }), "uuids"),
            (json!({ "uuids": ["a", ""] }), "uuids[1]"),
            (json!({ "uuids": "a" }), "uuids"),
            (
                json!({ "uuids": ["a"], "uuids_file": "x.json" }),
                "uuids_file",
            ),
            (
                json!({ "uuids_file": dir.path().join("none.json") }),
                "uuids_file",
            ),
        ] {
            assert_eq!(parse_selection(&args).unwrap_err().0, field, "{args}");
        }
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
    fn routing_files_merge_in_order_and_a_bad_member_names_its_file() {
        let dir = tempfile::tempdir().unwrap();
        let file = |name: &str, body: serde_json::Value| {
            let p = dir.path().join(name);
            std::fs::write(&p, body.to_string()).unwrap();
            p.display().to_string()
        };
        let a = file(
            "a.json",
            json!({ "tracks": [{ "net": "A", "layer": "F.Cu", "width": 0.2, "x1": 0, "y1": 0, "x2": 1, "y2": 0 }],
                    "vias": [{ "net": "A", "x": 1, "y": 0, "drill": 0.2, "diameter": 0.45 }] }),
        );
        let b = file(
            "b.json",
            json!({ "tracks": [{ "net": "B", "layer": "In2.Cu", "width": 1.5, "x1": 0, "y1": 5, "x2": 9, "y2": 5 },
                               { "net": "B", "layer": "In2.Cu", "width": 1.5, "x1": 9, "y1": 5, "x2": 12, "y2": 8 }] }),
        );
        let c = file(
            "c.json",
            json!({ "vias": [{ "net": "C", "x": 20, "y": 20, "drill": 0.3, "diameter": 0.6 }] }),
        );
        let (batch, counts) =
            parse_request(&json!({ "routing_files": [a, b, c], "locked": true })).unwrap();
        assert_eq!(
            batch
                .tracks
                .iter()
                .map(|t| t.net.as_str())
                .collect::<Vec<_>>(),
            ["A", "B", "B"]
        );
        assert_eq!(
            batch
                .vias
                .iter()
                .map(|v| v.net.as_str())
                .collect::<Vec<_>>(),
            ["A", "C"]
        );
        assert!(batch.locked);
        assert_eq!(
            counts
                .iter()
                .map(|c| (c.tracks, c.vias))
                .collect::<Vec<_>>(),
            [(1, 1), (2, 0), (0, 1)]
        );
        // The merged batch is linted as one: B's two tracks meet at (9, 5).
        assert_eq!(batch.segments().len(), 3);

        let bad = file(
            "bad.json",
            json!({ "tracks": [{ "net": "B", "layer": "In2.Cu", "width": 0, "x1": 0, "y1": 0, "x2": 1, "y2": 0 }] }),
        );
        let (field, reason) = parse_request(&json!({ "routing_files": [a, bad] })).unwrap_err();
        assert_eq!(field, "routing_files[1]");
        assert!(
            reason.contains("bad.json") && reason.contains("tracks[0].width"),
            "{reason}"
        );
        let other = file("other.json", json!({ "zones": [] }));
        let (field, reason) = parse_request(&json!({ "routing_files": [other] })).unwrap_err();
        assert_eq!(field, "routing_files[0]");
        assert!(reason.contains("other.json: key 'zones'"), "{reason}");
        let missing = dir.path().join("missing.json").display().to_string();
        let (field, reason) =
            parse_request(&json!({ "routing_files": [a, b, missing] })).unwrap_err();
        assert_eq!(field, "routing_files[2]");
        assert!(reason.contains("missing.json cannot be read"), "{reason}");
        for args in [
            json!({ "routing_files": [a], "routing_file": b }),
            json!({ "routing_files": [a], "tracks": [] }),
            json!({ "routing_files": [] }),
            json!({ "routing_files": "a.json" }),
        ] {
            assert_eq!(
                parse_request(&args).unwrap_err().0,
                "routing_files",
                "{args}"
            );
        }
        // The single file and the inline items still work.
        assert_eq!(
            parse_request(&json!({ "routing_file": b }))
                .unwrap()
                .0
                .tracks
                .len(),
            2
        );
        assert!(parse_request(&json!({ "routing_file": b }))
            .unwrap()
            .1
            .is_empty());
        assert_eq!(parse_request(&sample()).unwrap().0.vias.len(), 1);
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
