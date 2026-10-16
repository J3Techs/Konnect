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
    json!({
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
    })
}

#[cfg(test)]
mod tests {
    use super::*;

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
