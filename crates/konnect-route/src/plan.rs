//! S8 plan: routed copper as Konnect's `add_routing` input.
//!
//! [`RoutedCopper`] is what the router produces, in board ids and nm.
//! [`RoutePlan`] is the same copper by net and layer *name* in millimetres,
//! serialised exactly as `add_routing` reads a `routing_file`:
//!
//! ```json
//! { "tracks": [ { "net": "…", "layer": "F.Cu", "width": 0.1,
//!                 "x1": 0.0, "y1": 0.0, "x2": 1.0, "y2": 0.0 } ],
//!   "vias":   [ { "net": "…", "x": 1.0, "y": 0.0, "drill": 0.3, "diameter": 0.6 } ] }
//! ```
//!
//! The file holds only `tracks` and `vias` (`add_routing` refuses any other
//! key); `locked` is the tool's own argument. [`RoutePlan::validate`] applies
//! `add_routing`'s member checks so a plan is refused here rather than by
//! KiCad, and [`RoutePlan::chunks`] splits a plan under the per-call item cap.
//! Every coordinate is an exact nm value converted to mm, which `konnect-ipc`
//! rounds straight back to the same nm.

use crate::api::{Generation, LayerId, LayerMask, NetId};
use crate::board::RouteBoard;
use konnect_geom::{nm_to_mm, Coord, Point, Segment};
use serde::{Deserialize, Serialize};

/// Most items one `add_routing` call accepts (`routing_batch::MAX_ITEMS`).
pub const ADD_ROUTING_MAX_ITEMS: usize = 20_000;
/// `add_routing`'s largest track width, mm.
pub const ADD_ROUTING_MAX_WIDTH_MM: f64 = 20.0;
/// `add_routing`'s largest via diameter, mm.
pub const ADD_ROUTING_MAX_VIA_MM: f64 = 10.0;

/// One routed straight track.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct RoutedTrack {
    pub net: NetId,
    pub layer: LayerId,
    pub seg: Segment,
    pub gen: Generation,
}

/// One routed via.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct RoutedVia {
    pub net: NetId,
    pub center: Point,
    pub diameter: Coord,
    pub drill: Coord,
    pub layers: LayerMask,
    pub gen: Generation,
}

/// Everything the router has produced, in commit order.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct RoutedCopper {
    pub tracks: Vec<RoutedTrack>,
    pub vias: Vec<RoutedVia>,
}

/// One `add_routing` track.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanTrack {
    pub net: String,
    pub layer: String,
    pub width: f64,
    pub x1: f64,
    pub y1: f64,
    pub x2: f64,
    pub y2: f64,
}

/// One `add_routing` through via.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanVia {
    pub net: String,
    pub x: f64,
    pub y: f64,
    pub drill: f64,
    pub diameter: f64,
}

/// A `routing_file` body. Field order and names are `add_routing`'s.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RoutePlan {
    #[serde(default)]
    pub tracks: Vec<PlanTrack>,
    #[serde(default)]
    pub vias: Vec<PlanVia>,
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum PlanError {
    #[error("via at ({x_nm}, {y_nm}) nm does not span every copper layer; add_routing creates through vias only")]
    NotThroughVia { x_nm: Coord, y_nm: Coord },
    #[error("{field}: {reason}")]
    Invalid { field: String, reason: String },
}

impl RoutePlan {
    /// Lower routed copper to names and millimetres.
    pub fn from_routed(board: &RouteBoard, copper: &RoutedCopper) -> Result<Self, PlanError> {
        let all = board.layers.all();
        let mm = |v: Coord| nm_to_mm(i64::from(v));
        let tracks = copper
            .tracks
            .iter()
            .map(|t| PlanTrack {
                net: board.net(t.net).name.clone(),
                layer: board.layers.name(t.layer).to_string(),
                width: mm(t.seg.width),
                x1: mm(t.seg.a.x),
                y1: mm(t.seg.a.y),
                x2: mm(t.seg.b.x),
                y2: mm(t.seg.b.y),
            })
            .collect();
        let vias = copper
            .vias
            .iter()
            .map(|v| {
                if v.layers != all {
                    return Err(PlanError::NotThroughVia {
                        x_nm: v.center.x,
                        y_nm: v.center.y,
                    });
                }
                Ok(PlanVia {
                    net: board.net(v.net).name.clone(),
                    x: mm(v.center.x),
                    y: mm(v.center.y),
                    drill: mm(v.drill),
                    diameter: mm(v.diameter),
                })
            })
            .collect::<Result<_, _>>()?;
        let plan = RoutePlan { tracks, vias };
        plan.validate()?;
        Ok(plan)
    }

    pub fn item_count(&self) -> usize {
        self.tracks.len() + self.vias.len()
    }

    /// `add_routing`'s member checks, in its order and wording.
    pub fn validate(&self) -> Result<(), PlanError> {
        let invalid = |field: String, reason: &str| PlanError::Invalid {
            field,
            reason: reason.to_string(),
        };
        for (i, t) in self.tracks.iter().enumerate() {
            let at = format!("tracks[{i}]");
            if !t.layer.ends_with(".Cu") {
                return Err(invalid(format!("{at}.layer"), "is not a copper layer name"));
            }
            if !(t.width > 0.0 && t.width <= ADD_ROUTING_MAX_WIDTH_MM) {
                return Err(invalid(
                    format!("{at}.width"),
                    "must be above 0 and at most 20 mm",
                ));
            }
            if t.net.is_empty() {
                return Err(invalid(format!("{at}.net"), "must be a non-empty net name"));
            }
            if [t.x1, t.y1, t.x2, t.y2].iter().any(|v| !v.is_finite()) {
                return Err(invalid(at, "must be a finite number"));
            }
            if (t.x1, t.y1) == (t.x2, t.y2) {
                return Err(invalid(at, "has zero length"));
            }
        }
        for (i, v) in self.vias.iter().enumerate() {
            let at = format!("vias[{i}]");
            if !(v.drill > 0.0 && v.diameter > v.drill && v.diameter <= ADD_ROUTING_MAX_VIA_MM) {
                return Err(invalid(at, "needs 0 < drill < diameter <= 10 mm"));
            }
            if v.net.is_empty() {
                return Err(invalid(format!("{at}.net"), "must be a non-empty net name"));
            }
            if !(v.x.is_finite() && v.y.is_finite()) {
                return Err(invalid(at, "must be a finite number"));
            }
        }
        if self.item_count() == 0 {
            return Err(invalid(
                "tracks".to_string(),
                "at least one track or via is required",
            ));
        }
        Ok(())
    }

    /// Split into plans of at most `max_items` items each (tracks first, in
    /// order, then vias), for chunked `add_routing` calls. Empty plans are
    /// never produced.
    pub fn chunks(&self, max_items: usize) -> Vec<RoutePlan> {
        let max_items = max_items.max(1);
        let mut out = Vec::new();
        let mut cur = RoutePlan::default();
        let mut flush = |cur: &mut RoutePlan| {
            if cur.item_count() > 0 {
                out.push(std::mem::take(cur));
            }
        };
        for t in &self.tracks {
            if cur.item_count() == max_items {
                flush(&mut cur);
            }
            cur.tracks.push(t.clone());
        }
        for v in &self.vias {
            if cur.item_count() == max_items {
                flush(&mut cur);
            }
            cur.vias.push(v.clone());
        }
        flush(&mut cur);
        out
    }

    /// The `routing_file` text.
    pub fn to_routing_file_json(&self) -> String {
        serde_json::to_string_pretty(self).expect("a RoutePlan always serializes")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synthetic::BoardBuilder;

    fn board() -> RouteBoard {
        let mut b = BoardBuilder::new(10.0, 10.0, &["F.Cu", "In1.Cu", "B.Cu"]);
        b.net("SIG");
        b.build()
    }

    fn copper(b: &RouteBoard) -> RoutedCopper {
        let net = b.net_by_name("SIG").unwrap();
        RoutedCopper {
            tracks: vec![RoutedTrack {
                net,
                layer: LayerId(0),
                seg: Segment::new(
                    Point::new(1_000_000, 2_500_000),
                    Point::new(187_300_000, 2_500_000),
                    100_000,
                ),
                gen: Generation(1),
            }],
            vias: vec![RoutedVia {
                net,
                center: Point::new(187_300_000, 2_500_000),
                diameter: 600_000,
                drill: 300_000,
                layers: b.layers.all(),
                gen: Generation(1),
            }],
        }
    }

    #[test]
    fn plan_serializes_as_an_add_routing_file() {
        let b = board();
        let plan = RoutePlan::from_routed(&b, &copper(&b)).unwrap();
        let v: serde_json::Value = serde_json::from_str(&plan.to_routing_file_json()).unwrap();
        assert_eq!(
            v,
            serde_json::json!({
                "tracks": [{ "net": "SIG", "layer": "F.Cu", "width": 0.1,
                             "x1": 1.0, "y1": 2.5, "x2": 187.3, "y2": 2.5 }],
                "vias": [{ "net": "SIG", "x": 187.3, "y": 2.5, "drill": 0.3, "diameter": 0.6 }]
            })
        );
        // Only the two keys add_routing accepts in a routing_file.
        let keys: Vec<&String> = v.as_object().unwrap().keys().collect();
        assert_eq!(keys, vec!["tracks", "vias"]);
    }

    #[test]
    fn millimetres_round_trip_to_the_same_nanometres() {
        let b = board();
        let plan = RoutePlan::from_routed(&b, &copper(&b)).unwrap();
        // konnect-ipc's builders convert with (mm * 1e6).round().
        let back = |mm: f64| (mm * 1_000_000.0).round() as i64;
        assert_eq!(back(plan.tracks[0].x2), 187_300_000);
        assert_eq!(back(plan.tracks[0].width), 100_000);
        assert_eq!(back(plan.vias[0].diameter), 600_000);
    }

    #[test]
    fn blind_vias_and_bad_members_are_refused() {
        let b = board();
        let mut c = copper(&b);
        c.vias[0].layers = LayerMask::span(LayerId(0), LayerId(1));
        assert!(matches!(
            RoutePlan::from_routed(&b, &c),
            Err(PlanError::NotThroughVia { .. })
        ));
        let mut c = copper(&b);
        c.tracks[0].seg.b = c.tracks[0].seg.a;
        let err = RoutePlan::from_routed(&b, &c).unwrap_err();
        assert_eq!(err.to_string(), "tracks[0]: has zero length");
        assert!(RoutePlan::default().validate().is_err());
    }

    #[test]
    fn chunks_respect_the_item_cap() {
        let b = board();
        let mut c = copper(&b);
        let t = c.tracks[0];
        c.tracks = (0..5)
            .map(|i| RoutedTrack {
                seg: Segment::new(Point::new(i, 0), Point::new(i + 1_000, 0), 100_000),
                ..t
            })
            .collect();
        let plan = RoutePlan::from_routed(&b, &c).unwrap();
        let chunks = plan.chunks(2);
        assert_eq!(
            chunks.iter().map(RoutePlan::item_count).collect::<Vec<_>>(),
            vec![2, 2, 2]
        );
        assert_eq!(chunks[2].vias.len(), 1);
        assert_eq!(plan.chunks(ADD_ROUTING_MAX_ITEMS).len(), 1);
    }
}
