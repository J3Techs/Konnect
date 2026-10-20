//! Which tracks, arcs and vias of the live board touch a given pad.
//!
//! A pad whose net the schematic changes is only dangerous to reassign when
//! routed copper actually reaches it: a track, arc or via ending on or
//! overlapping the pad keeps the old net's connection after the pad moves to
//! the new one. Copper of the old net elsewhere on the board, and zone fills
//! (which a refill re-forms around the pad's new net), do not. KiCad 10's
//! Zone message does not carry the zone's net, so zones are only counted.
//!
//! Pad outlines are taken conservatively, never smaller than the copper:
//! a circle is exact, every rectangle-family shape (rectangle, rounded or
//! chamfered rectangle, oval) is its full rectangle, a trapezoid the
//! rectangle grown by its delta, and a custom pad the disc around its anchor
//! that holds every primitive. A track that comes within that outline is
//! counted as touching, so "nothing touches" is never claimed wrongly.

use konnect_geom::{touches, BBox, Circle, ConvexPoly, Point, Segment, Shape};
use konnect_ipc::gen::kiapi;
use prost::Message;
use std::collections::BTreeMap;

/// One track, arc or via, with its copper.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct CopperPiece {
    pub kind: &'static str,
    pub uuid: String,
    pub net: String,
    /// The board layer, or `None` for a via (taken on every layer).
    pub layer: Option<i32>,
    pub shapes: Vec<Shape>,
    pub bbox: BBox,
}

/// A pad's copper: outlines with the layers they are on (`None`: every
/// copper layer), or why they could not be read.
#[derive(Debug, Clone, PartialEq)]
pub(super) enum PadCopper {
    Known(Vec<(Option<Vec<i32>>, Shape)>),
    Unknown(String),
}

/// The copper geometry of the live board KiCad answered with.
#[derive(Debug, Clone, Default, PartialEq)]
pub(super) struct CopperGeometry {
    pub pieces: Vec<CopperPiece>,
    /// (footprint KIID, pad number) to the pad's copper.
    pub pads: BTreeMap<(String, String), PadCopper>,
    /// Zones of the board (their nets are not reported).
    pub zones: usize,
    /// Tracks, arcs and vias per net.
    pub by_net: BTreeMap<String, usize>,
}

/// A track, arc or via found touching a pad.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct Contact {
    pub kind: &'static str,
    pub uuid: String,
    pub net: String,
    pub layer: Option<String>,
}

impl Contact {
    pub fn describe(&self) -> String {
        match &self.layer {
            Some(l) => format!("{} {} (net '{}', {l})", self.kind, self.uuid, self.net),
            None => format!("{} {} (net '{}')", self.kind, self.uuid, self.net),
        }
    }
}

fn nm_point(v: Option<&kiapi::common::types::Vector2>) -> Option<Point> {
    let v = v?;
    Some(Point::new(
        i32::try_from(v.x_nm).ok()?,
        i32::try_from(v.y_nm).ok()?,
    ))
}

fn layer_name(layer: i32) -> String {
    kiapi::board::types::BoardLayer::try_from(layer)
        .ok()
        .and_then(konnect_ipc::builders::layer_name)
        .map(str::to_string)
        .unwrap_or_else(|| format!("layer {layer}"))
}

/// KiCad's `RotatePoint`: a pad-local offset turned by the pad's angle.
fn rotate(x: f64, y: f64, deg: f64) -> (f64, f64) {
    let (s, c) = deg.to_radians().sin_cos();
    (x * c + y * s, -x * s + y * c)
}

fn at(center: Point, dx: f64, dy: f64) -> Option<Point> {
    let x = f64::from(center.x) + dx;
    let y = f64::from(center.y) + dy;
    Some(Point::new(
        i32::try_from(x.round() as i64).ok()?,
        i32::try_from(y.round() as i64).ok()?,
    ))
}

/// A `w` x `h` rectangle centred on `center`, turned by `deg`.
fn turned_rect(center: Point, w: f64, h: f64, deg: f64) -> Option<Shape> {
    let corners = [(-w, -h), (w, -h), (w, h), (-w, h)]
        .iter()
        .map(|&(x, y)| {
            let (dx, dy) = rotate(x / 2.0, y / 2.0, deg);
            at(center, dx, dy)
        })
        .collect::<Option<Vec<_>>>()?;
    ConvexPoly::new(corners, 0).map(Shape::ConvexPoly)
}

/// How far from the pad's centre a custom pad's primitives reach, nm.
fn custom_reach(layer: &kiapi::board::types::PadStackLayer) -> Result<f64, String> {
    use konnect_ipc::IpcPadPrimitive as P;
    let mut reach: f64 = 0.0;
    let mut far = |x: f64, y: f64, grow: f64| reach = reach.max(x.hypot(y) + grow);
    for (i, s) in layer.custom_shapes.iter().enumerate() {
        let p = konnect_ipc::builders::pad_primitive_from_proto(s)
            .map_err(|e| format!("custom primitive {i} is unreadable: {e:#}"))?;
        // Primitives are in pad-local mm.
        match p {
            P::Polygon { points, width, .. } => {
                for (x, y) in points {
                    far(x, y, width / 2.0);
                }
            }
            P::Rect {
                start, end, width, ..
            } => {
                for (x, y) in [start, end, (start.0, end.1), (end.0, start.1)] {
                    far(x, y, width / 2.0);
                }
            }
            P::Circle {
                center, end, width, ..
            } => {
                let r = (end.0 - center.0).hypot(end.1 - center.1);
                far(center.0, center.1, r + width / 2.0);
            }
            P::Segment { start, end, width } => {
                far(start.0, start.1, width / 2.0);
                far(end.0, end.1, width / 2.0);
            }
            P::Arc {
                start,
                mid,
                end,
                width,
            } => {
                // The arc lies within the disc through its three points'
                // farthest one plus its sagitta; its chord ends bound it well
                // enough with the mid point and half the chord added.
                let chord = (end.0 - start.0).hypot(end.1 - start.1);
                for (x, y) in [start, mid, end] {
                    far(x, y, width / 2.0 + chord / 2.0);
                }
            }
        }
    }
    Ok(reach * 1e6)
}

fn pad_copper(pad: &kiapi::board::types::Pad) -> PadCopper {
    use kiapi::board::types::PadStackShape as S;
    let Some(center) = nm_point(pad.position.as_ref()) else {
        return PadCopper::Unknown("KiCad reported no position".to_string());
    };
    let Some(stack) = pad.pad_stack.as_ref() else {
        return PadCopper::Unknown("KiCad reported no pad stack".to_string());
    };
    let deg = stack.angle.as_ref().map_or(0.0, |a| a.value_degrees);
    let drilled = stack
        .drill
        .as_ref()
        .and_then(|d| d.diameter.as_ref())
        .is_some_and(|d| d.x_nm > 0 || d.y_nm > 0);
    let layers = (!drilled && !stack.layers.is_empty()).then(|| stack.layers.clone());
    if stack.copper_layers.is_empty() {
        return PadCopper::Unknown("KiCad reported no copper shape".to_string());
    }
    let mut out = Vec::new();
    for layer in &stack.copper_layers {
        let Some(size) = layer.size.as_ref() else {
            return PadCopper::Unknown("a copper layer has no size".to_string());
        };
        let (w, h) = (size.x_nm as f64, size.y_nm as f64);
        let (ox, oy) = layer
            .offset
            .as_ref()
            .map_or((0.0, 0.0), |o| rotate(o.x_nm as f64, o.y_nm as f64, deg));
        let Some(c) = at(center, ox, oy) else {
            return PadCopper::Unknown("the pad lies outside the board range".to_string());
        };
        let shape = match S::try_from(layer.shape) {
            Ok(S::PssCircle) => Some(Shape::Circle(Circle::from_diameter(
                c,
                w.max(h).round() as i32,
            ))),
            Ok(S::PssRectangle | S::PssRoundrect | S::PssChamferedrect | S::PssOval) => {
                turned_rect(c, w, h, deg)
            }
            Ok(S::PssTrapezoid) => {
                let d = layer.trapezoid_delta.as_ref();
                let dx = d.map_or(0.0, |d| (d.x_nm as f64).abs());
                let dy = d.map_or(0.0, |d| (d.y_nm as f64).abs());
                turned_rect(c, w + dx, h + dy, deg)
            }
            Ok(S::PssCustom) => match custom_reach(layer) {
                Ok(reach) => {
                    let anchor = w.hypot(h) / 2.0;
                    let r = reach.max(anchor).ceil() as i32;
                    Some(Shape::Circle(Circle::new(c, r)))
                }
                Err(why) => return PadCopper::Unknown(why),
            },
            _ => None,
        };
        match shape {
            Some(s) => out.push((layers.clone(), s)),
            None => {
                return PadCopper::Unknown(format!(
                    "the copper shape (code {}) cannot be read",
                    layer.shape
                ))
            }
        }
    }
    PadCopper::Known(out)
}

/// The arc as two chords, widened by their sagitta so the chords cover the
/// curve.
fn arc_shapes(a: &kiapi::board::types::Arc) -> Option<Vec<Shape>> {
    let (s, m, e) = (
        nm_point(a.start.as_ref())?,
        nm_point(a.mid.as_ref())?,
        nm_point(a.end.as_ref())?,
    );
    let width = a.width.as_ref().map_or(0, |w| w.value_nm);
    let sag = |p: Point, q: Point| -> f64 {
        // Distance from the chord p-q to the arc is at most the distance
        // from the chord's midpoint to the arc's mid point.
        let mx = (f64::from(p.x) + f64::from(q.x)) / 2.0;
        let my = (f64::from(p.y) + f64::from(q.y)) / 2.0;
        (f64::from(m.x) - mx).hypot(f64::from(m.y) - my)
    };
    let grow = sag(s, e).ceil() as i64;
    let w = i32::try_from(width + 2 * grow).ok()?;
    Some(vec![
        Shape::Segment(Segment::new(s, m, w)),
        Shape::Segment(Segment::new(m, e, w)),
    ])
}

fn bbox_of(shapes: &[Shape]) -> Option<BBox> {
    shapes.iter().map(Shape::bbox).reduce(BBox::union)
}

impl CopperGeometry {
    /// Read the board's footprints' pads and its tracks, arcs and vias.
    /// An item KiCad sent that does not decode, or whose geometry is outside
    /// the board range, is kept as an unreadable piece that touches every
    /// pad, so it can never be missed.
    pub fn read(
        footprints: &[kiapi::board::types::FootprintInstance],
        tracks: &[prost_types::Any],
        arcs: &[prost_types::Any],
        vias: &[prost_types::Any],
        zones: usize,
    ) -> Self {
        let mut geometry = CopperGeometry {
            zones,
            ..CopperGeometry::default()
        };
        for footprint in footprints {
            let Some(kiid) = footprint.id.as_ref().map(|id| id.value.clone()) else {
                continue;
            };
            let Some(definition) = footprint.definition.as_ref() else {
                continue;
            };
            for child in &definition.items {
                if !konnect_ipc::builders::any_is(child, "kiapi.board.types.Pad") {
                    continue;
                }
                let Ok(pad) = kiapi::board::types::Pad::decode(child.value.as_slice()) else {
                    continue;
                };
                geometry
                    .pads
                    .insert((kiid.clone(), pad.number.clone()), pad_copper(&pad));
            }
        }
        let net_of = |n: &Option<kiapi::board::types::Net>| {
            n.as_ref().map(|n| n.name.clone()).unwrap_or_default()
        };
        let id_of = |k: &Option<kiapi::common::types::Kiid>| {
            k.as_ref().map(|k| k.value.clone()).unwrap_or_default()
        };
        for item in tracks {
            let Ok(t) = kiapi::board::types::Track::decode(item.value.as_slice()) else {
                continue;
            };
            let shapes = (|| {
                let w = i32::try_from(t.width.as_ref().map_or(0, |w| w.value_nm)).ok()?;
                Some(vec![Shape::Segment(Segment::new(
                    nm_point(t.start.as_ref())?,
                    nm_point(t.end.as_ref())?,
                    w,
                ))])
            })();
            geometry.push("track", id_of(&t.id), net_of(&t.net), Some(t.layer), shapes);
        }
        for item in arcs {
            let Ok(a) = kiapi::board::types::Arc::decode(item.value.as_slice()) else {
                continue;
            };
            let shapes = arc_shapes(&a);
            geometry.push("arc", id_of(&a.id), net_of(&a.net), Some(a.layer), shapes);
        }
        for item in vias {
            let Ok(v) = kiapi::board::types::Via::decode(item.value.as_slice()) else {
                continue;
            };
            let shapes = (|| {
                let size = v
                    .pad_stack
                    .as_ref()?
                    .copper_layers
                    .iter()
                    .filter_map(|l| l.size.as_ref())
                    .map(|s| s.x_nm.max(s.y_nm))
                    .max()?;
                Some(vec![Shape::Circle(Circle::from_diameter(
                    nm_point(v.position.as_ref())?,
                    i32::try_from(size).ok()?,
                ))])
            })();
            geometry.push("via", id_of(&v.id), net_of(&v.net), None, shapes);
        }
        geometry
    }

    fn push(
        &mut self,
        kind: &'static str,
        uuid: String,
        net: String,
        layer: Option<i32>,
        shapes: Option<Vec<Shape>>,
    ) {
        *self.by_net.entry(net.clone()).or_default() += 1;
        // Unreadable geometry is kept as an infinite box: it touches
        // everything, so a pad near it is never called untouched.
        let (shapes, bbox) = match shapes.and_then(|s| bbox_of(&s).map(|b| (s, b))) {
            Some(v) => v,
            None => (
                Vec::new(),
                BBox::new(
                    Point::new(-konnect_geom::MAX_COORD, -konnect_geom::MAX_COORD),
                    Point::new(konnect_geom::MAX_COORD, konnect_geom::MAX_COORD),
                ),
            ),
        };
        self.pieces.push(CopperPiece {
            kind,
            uuid,
            net,
            layer,
            shapes,
            bbox,
        });
    }

    /// The tracks, arcs and vias touching the pad, or why its copper could
    /// not be read.
    pub fn contacts(&self, footprint_kiid: &str, pad: &str) -> Result<Vec<Contact>, String> {
        let outlines = match self
            .pads
            .get(&(footprint_kiid.to_string(), pad.to_string()))
        {
            Some(PadCopper::Known(o)) => o,
            Some(PadCopper::Unknown(why)) => return Err(why.clone()),
            None => return Err("KiCad did not report this pad".to_string()),
        };
        let mut out = Vec::new();
        for piece in &self.pieces {
            let hit = outlines.iter().any(|(layers, outline)| {
                let shared = match (layers, piece.layer) {
                    (Some(ls), Some(l)) => ls.contains(&l),
                    _ => true,
                };
                shared
                    && piece.bbox.intersects(&outline.bbox())
                    && (piece.shapes.is_empty() || piece.shapes.iter().any(|s| touches(s, outline)))
            });
            if hit {
                out.push(Contact {
                    kind: piece.kind,
                    uuid: piece.uuid.clone(),
                    net: piece.net.clone(),
                    layer: piece.layer.map(layer_name),
                });
            }
        }
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pad(number: &str, x: f64, y: f64, w: f64, h: f64, deg: f64) -> prost_types::Any {
        use kiapi::board::types as b;
        let nm = |v: f64| (v * 1e6).round() as i64;
        let p = b::Pad {
            number: number.to_string(),
            position: Some(kiapi::common::types::Vector2 {
                x_nm: nm(x),
                y_nm: nm(y),
            }),
            pad_stack: Some(b::PadStack {
                layers: vec![b::BoardLayer::BlFCu as i32],
                angle: Some(kiapi::common::types::Angle { value_degrees: deg }),
                copper_layers: vec![b::PadStackLayer {
                    layer: b::BoardLayer::BlFCu as i32,
                    shape: b::PadStackShape::PssRectangle as i32,
                    size: Some(kiapi::common::types::Vector2 {
                        x_nm: nm(w),
                        y_nm: nm(h),
                    }),
                    ..Default::default()
                }],
                ..Default::default()
            }),
            ..Default::default()
        };
        konnect_ipc::builders::pack_any(&p, "kiapi.board.types.Pad")
    }

    fn footprint(pads: Vec<prost_types::Any>) -> kiapi::board::types::FootprintInstance {
        kiapi::board::types::FootprintInstance {
            id: Some(kiapi::common::types::Kiid {
                value: "fp".to_string(),
            }),
            definition: Some(kiapi::board::types::Footprint {
                items: pads,
                ..Default::default()
            }),
            ..Default::default()
        }
    }

    fn track(id: &str, net: &str, layer: &str, a: (f64, f64), b: (f64, f64)) -> prost_types::Any {
        let mut t = konnect_ipc::builders::build_track(net, 1, layer, 0.2, a.0, a.1, b.0, b.1);
        t.id = Some(kiapi::common::types::Kiid {
            value: id.to_string(),
        });
        konnect_ipc::builders::pack_any(&t, "kiapi.board.types.Track")
    }

    #[test]
    fn only_copper_reaching_the_pad_on_its_layer_touches_it() {
        // A 1 x 2 mm pad at (10, 10), turned 90 degrees: 2 mm wide in x.
        let fp = footprint(vec![pad("1", 10.0, 10.0, 1.0, 2.0, 90.0)]);
        let tracks = vec![
            // Ends 0.9 mm right of the centre: inside the turned pad.
            track("ends-on", "A", "F.Cu", (10.9, 10.0), (15.0, 10.0)),
            // 0.75 mm above the centre: clear of the 0.5 mm half height by
            // more than its 0.1 mm half width.
            track("passes-by", "A", "F.Cu", (5.0, 9.25), (15.0, 9.25)),
            // Through the pad on another layer.
            track("other-layer", "A", "B.Cu", (5.0, 10.0), (15.0, 10.0)),
        ];
        let mut via = konnect_ipc::builders::build_via("B", 2, 9.2, 10.0, 0.2, 0.45);
        via.id = Some(kiapi::common::types::Kiid {
            value: "via-on-edge".to_string(),
        });
        let vias = vec![konnect_ipc::builders::pack_any(
            &via,
            "kiapi.board.types.Via",
        )];
        let g = CopperGeometry::read(&[fp], &tracks, &[], &vias, 3);
        let hits: Vec<String> = g
            .contacts("fp", "1")
            .unwrap()
            .into_iter()
            .map(|c| c.uuid)
            .collect();
        assert_eq!(hits, ["ends-on", "via-on-edge"]);
        assert_eq!(g.by_net["A"], 3);
        assert_eq!(g.zones, 3);
        assert!(
            g.contacts("fp", "9").is_err(),
            "an unreported pad is unknown"
        );
    }
}
