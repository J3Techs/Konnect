//! `RouteBoard`: the router's model of one board.
//!
//! Everything is in integer nanometres ([`konnect_geom::Coord`]) and every
//! copper object carries its exact [`Shape`]s, so the check stage judges the
//! same geometry KiCad will. The model is built offline from a saved board
//! ([`crate::loader`]) or programmatically ([`crate::synthetic`]); an IPC
//! adapter will fill the same struct.
//!
//! Vectors are indexed by the matching id type in [`crate::api`]: `pads[i]`
//! has `PadId(i)`, and so on. The model is plain data — stages read it and
//! keep their own derived state (indexes, rasters, routed copper).

use crate::api::{
    ArcId, ClassId, FootprintId, ItemRef, KeepoutId, LayerId, LayerMask, NetId, Ownership, PadId,
    TrackId, ViaId, ZoneId,
};
use konnect_geom::{
    approximate_arc, ArcPolicy, ArcSide, BBox, Circle, Coord, Point, Polygon, Segment, Shape,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// One board, ready to route.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct RouteBoard {
    pub outline: BoardOutline,
    pub layers: LayerTable,
    /// Layers the router may place new copper on. Fixed copper on other
    /// layers still counts as obstacles and terminals.
    pub routable: LayerMask,
    pub nets: Vec<Net>,
    pub footprints: Vec<Footprint>,
    pub pads: Vec<Pad>,
    pub tracks: Vec<Track>,
    pub arcs: Vec<ArcTrack>,
    pub vias: Vec<Via>,
    /// Copper plane pieces, one per (fill polygon, layer).
    pub zones: Vec<ZonePiece>,
    /// Rule areas that forbid copper.
    pub keepouts: Vec<Keepout>,
}

impl RouteBoard {
    /// Look a net up by its KiCad name.
    pub fn net_by_name(&self, name: &str) -> Option<NetId> {
        self.nets
            .binary_search_by(|n| n.name.as_str().cmp(name))
            .ok()
            .map(|i| NetId(i as u32))
    }

    pub fn net(&self, id: NetId) -> &Net {
        &self.nets[id.index()]
    }

    pub fn layer_count(&self) -> usize {
        self.layers.copper.len()
    }

    /// Every pad on `net`, in pad order.
    pub fn pads_of_net(&self, net: NetId) -> impl Iterator<Item = &Pad> {
        self.pads.iter().filter(move |p| p.net == Some(net))
    }

    /// The footprint whose reference is `reference`, if exactly one has it.
    pub fn footprint_by_reference(&self, reference: &str) -> Option<FootprintId> {
        let mut found = self
            .footprints
            .iter()
            .filter(|f| f.reference.as_deref() == Some(reference))
            .map(|f| f.id);
        let first = found.next()?;
        found.next().is_none().then_some(first)
    }

    /// The net an item belongs to.
    pub fn net_of(&self, item: ItemRef) -> Option<NetId> {
        match item {
            ItemRef::Pad(id) => self.pads[id.index()].net,
            ItemRef::Track(id) => self.tracks[id.index()].net,
            ItemRef::Arc(id) => self.arcs[id.index()].net,
            ItemRef::Via(id) => self.vias[id.index()].net,
            ItemRef::Zone(id) => self.zones[id.index()].net,
        }
    }

    /// Basic structural invariants every producer of a `RouteBoard` must
    /// keep: ids equal indices, nets sorted by name, layer references within
    /// the stack. Returns the first broken invariant.
    pub fn validate(&self) -> Result<(), String> {
        let layer_ok = |l: LayerId| l.index() < self.layer_count();
        let all = self.layers.all();
        let mask_ok = |m: LayerMask| m.0 & !all.0 == 0;
        if self.layer_count() == 0 || self.layer_count() > 32 {
            return Err(format!("{} copper layers; need 1..=32", self.layer_count()));
        }
        for (i, n) in self.nets.iter().enumerate() {
            if n.id.index() != i {
                return Err(format!("net '{}' has id {} at index {i}", n.name, n.id.0));
            }
            if i > 0 && self.nets[i - 1].name >= n.name {
                return Err(format!("nets are not strictly sorted at '{}'", n.name));
            }
        }
        for (i, f) in self.footprints.iter().enumerate() {
            if f.id.index() != i {
                return Err(format!("footprint index {i} has id {}", f.id.0));
            }
            let (start, end) = (f.pad_range.0 as usize, f.pad_range.1 as usize);
            if start > end
                || end > self.pads.len()
                || self.pads[start..end].iter().any(|p| p.footprint != f.id)
            {
                return Err(format!("footprint index {i} has a bad pad_range"));
            }
        }
        for (i, p) in self.pads.iter().enumerate() {
            if p.id.index() != i || p.footprint.index() >= self.footprints.len() {
                return Err(format!("pad index {i} has a bad id or footprint"));
            }
            if !mask_ok(p.layers) {
                return Err(format!("pad index {i} names layers beyond the stack"));
            }
        }
        for (i, t) in self.tracks.iter().enumerate() {
            if t.id.index() != i || !layer_ok(t.layer) {
                return Err(format!("track index {i} has a bad id or layer"));
            }
        }
        for (i, a) in self.arcs.iter().enumerate() {
            if a.id.index() != i || !layer_ok(a.layer) {
                return Err(format!("arc index {i} has a bad id or layer"));
            }
        }
        for (i, v) in self.vias.iter().enumerate() {
            if v.id.index() != i || !mask_ok(v.layers) {
                return Err(format!("via index {i} has a bad id or layers"));
            }
        }
        for (i, z) in self.zones.iter().enumerate() {
            if z.id.index() != i || !layer_ok(z.layer) {
                return Err(format!("zone piece index {i} has a bad id or layer"));
            }
        }
        for (i, k) in self.keepouts.iter().enumerate() {
            if k.id.index() != i || !mask_ok(k.layers) {
                return Err(format!("keepout index {i} has a bad id or layers"));
            }
        }
        if !mask_ok(self.routable) {
            return Err("routable mask names layers beyond the stack".to_string());
        }
        Ok(())
    }
}

// ─── Outline ─────────────────────────────────────────────────────────────────

/// The board edge.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct BoardOutline {
    /// The board area as polygons, when the outline is provably that shape
    /// (week 1: an axis-aligned rectangle). Empty when unproven — use
    /// `edges` for edge clearance and do not assume containment.
    pub regions: Vec<Polygon>,
    /// Every Edge.Cuts boundary piece as a segment. Straight pieces have
    /// zero width; arcs and circles are inscribed chords whose width (twice
    /// the arc error, plus rounding) covers the true curve on both sides, so
    /// clearance measured to them never under-protects an outer edge or a
    /// cut-out. Copper-to-edge clearance is measured to these.
    pub edges: Vec<Segment>,
    /// Advisory bounding box of the edge graphics.
    pub bbox: Option<BBox>,
}

// ─── Layers ──────────────────────────────────────────────────────────────────

/// What KiCad's layer table says a copper layer is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CopperLayerKind {
    Signal,
    Power,
    Mixed,
    Jumper,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CopperLayer {
    pub id: LayerId,
    /// The canonical name, e.g. `F.Cu`, `In2.Cu`.
    pub name: String,
    /// KiCad's layer ordinal from the `(layers …)` table (F.Cu 0, B.Cu 2,
    /// In1.Cu 4, …). Not the stack position — that is `id`.
    pub kicad_ordinal: u32,
    pub kind: CopperLayerKind,
}

/// The copper stack, top to bottom.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct LayerTable {
    pub copper: Vec<CopperLayer>,
}

impl LayerTable {
    /// Build from names in any order: sorted into F.Cu, In1.Cu … InN.Cu, B.Cu.
    pub fn from_layers(mut layers: Vec<(String, u32, CopperLayerKind)>) -> Self {
        layers.sort_by_key(|(name, _, _)| stack_position(name));
        LayerTable {
            copper: layers
                .into_iter()
                .enumerate()
                .map(|(i, (name, kicad_ordinal, kind))| CopperLayer {
                    id: LayerId(i as u8),
                    name,
                    kicad_ordinal,
                    kind,
                })
                .collect(),
        }
    }

    pub fn by_name(&self, name: &str) -> Option<LayerId> {
        self.copper.iter().find(|l| l.name == name).map(|l| l.id)
    }

    pub fn name(&self, id: LayerId) -> &str {
        &self.copper[id.index()].name
    }

    pub fn len(&self) -> usize {
        self.copper.len()
    }

    pub fn is_empty(&self) -> bool {
        self.copper.is_empty()
    }

    pub fn all(&self) -> LayerMask {
        self.copper
            .iter()
            .fold(LayerMask::EMPTY, |m, l| m.with(l.id))
    }

    pub fn front(&self) -> Option<LayerId> {
        self.by_name("F.Cu")
    }

    pub fn back(&self) -> Option<LayerId> {
        self.by_name("B.Cu")
    }

    /// Expand one KiCad layer token into copper layers: `*.Cu` is every
    /// copper layer, `F&B.Cu` the two outer ones, anything else a single
    /// name. Non-copper and unknown names expand to nothing.
    pub fn expand(&self, token: &str) -> LayerMask {
        match token {
            "*.Cu" => self.all(),
            "F&B.Cu" => [self.front(), self.back()]
                .into_iter()
                .flatten()
                .fold(LayerMask::EMPTY, LayerMask::with),
            name => self
                .by_name(name)
                .map(LayerMask::single)
                .unwrap_or_default(),
        }
    }

    /// Union of [`Self::expand`] over several tokens.
    pub fn expand_all<'a, I: IntoIterator<Item = &'a str>>(&self, tokens: I) -> LayerMask {
        tokens
            .into_iter()
            .fold(LayerMask::EMPTY, |m, t| m.union(self.expand(t)))
    }
}

/// Physical order key: F.Cu first, inner layers by number, B.Cu last.
fn stack_position(name: &str) -> (u8, u32, String) {
    if name == "F.Cu" {
        return (0, 0, String::new());
    }
    if name == "B.Cu" {
        return (2, 0, String::new());
    }
    let inner = name
        .strip_prefix("In")
        .and_then(|rest| rest.strip_suffix(".Cu"))
        .and_then(|n| n.parse::<u32>().ok());
    match inner {
        Some(n) => (1, n, String::new()),
        // Unknown copper names keep a stable place among the inner layers.
        None => (1, u32::MAX, name.to_string()),
    }
}

// ─── Nets ────────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Net {
    pub id: NetId,
    pub name: String,
    /// Assigned by the rule table (`ClassId(0)`, `Default`, until then).
    pub class: ClassId,
    /// The other half of a differential pair, matched by KiCad's suffix
    /// convention (`+`/`-`, `P`/`N`).
    pub pair: Option<NetId>,
}

// ─── Footprints and pads ─────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Side {
    Front,
    Back,
}

/// One closed courtyard outline in board coordinates.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Courtyard {
    pub side: Side,
    /// Arcs and circles are circumscribed, so the region never under-covers
    /// what KiCad tests in `intersectsCourtyard()`.
    pub region: Polygon,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Footprint {
    pub id: FootprintId,
    pub reference: Option<String>,
    pub at: Point,
    pub rotation_deg: f64,
    pub side: Side,
    pub locked: bool,
    pub courtyards: Vec<Courtyard>,
    /// Pads of this footprint: `pads[pad_range.0 .. pad_range.1]`.
    pub pad_range: (u32, u32),
}

impl Footprint {
    pub fn courtyard_bbox(&self) -> Option<BBox> {
        self.courtyards
            .iter()
            .map(|c| c.region.bbox())
            .reduce(BBox::union)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PadKind {
    Smd,
    ThroughHole,
    NpThroughHole,
    /// Edge-connector style copper with no paste or mask opening.
    Connect,
}

/// A drilled hole: a round drill is a [`Shape::Circle`], an oblong slot a
/// [`Shape::Segment`] stadium.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Drill {
    pub shape: Shape,
    pub plated: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Pad {
    pub id: PadId,
    pub footprint: FootprintId,
    /// The pad number as written (`"1"`, `"A7"`, `"SH"`, `""`).
    pub number: String,
    pub net: Option<NetId>,
    pub kind: PadKind,
    /// The pad anchor in board coordinates: where the hole is. A drill
    /// offset moves the copper in `shapes`, not the anchor.
    pub center: Point,
    /// Board-space orientation in degrees (KiCad stores it including the
    /// footprint's rotation).
    pub angle_deg: f64,
    /// Copper layers the pad's copper occupies.
    pub layers: LayerMask,
    /// The copper outline as a union of shapes (one for ordinary pads,
    /// several for custom pads). The same on every layer in `layers`.
    pub shapes: Vec<Shape>,
    pub drill: Option<Drill>,
    /// The pad's own `(clearance …)` override, or its footprint's when the
    /// pad has none (KiCad's inheritance).
    pub local_clearance: Option<Coord>,
    /// True when the loader could not represent the pad exactly and stored a
    /// superset (chamfers, per-layer padstacks).
    pub approximated: bool,
    pub ownership: Ownership,
    pub uuid: Option<String>,
}

impl Pad {
    pub fn bbox(&self) -> Option<BBox> {
        self.shapes
            .iter()
            .map(Shape::bbox)
            .chain(self.drill.iter().map(|d| d.shape.bbox()))
            .reduce(BBox::union)
    }
}

// ─── Fixed copper ────────────────────────────────────────────────────────────

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Track {
    pub id: TrackId,
    pub net: Option<NetId>,
    pub layer: LayerId,
    pub seg: Segment,
    pub ownership: Ownership,
    pub uuid: Option<String>,
}

/// A KiCad arc track, kept exact; lower it with [`ArcTrack::segments`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ArcTrack {
    pub id: ArcId,
    pub net: Option<NetId>,
    pub layer: LayerId,
    pub start: Point,
    pub mid: Point,
    pub end: Point,
    pub width: Coord,
    pub ownership: Ownership,
    pub uuid: Option<String>,
}

impl ArcTrack {
    /// The arc as inscribed straight pieces of the same width. Clearance
    /// checked against them must add `policy.max_error_nm`.
    pub fn segments(&self, policy: &ArcPolicy) -> Vec<Segment> {
        approximate_arc(self.start, self.mid, self.end, policy, ArcSide::Inscribed)
            .windows(2)
            .map(|w| Segment::new(w[0], w[1], self.width))
            .collect()
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Via {
    pub id: ViaId,
    pub net: Option<NetId>,
    pub center: Point,
    pub diameter: Coord,
    pub drill: Coord,
    /// The layers the via spans (all copper for a through via).
    pub layers: LayerMask,
    pub ownership: Ownership,
    pub uuid: Option<String>,
}

impl Via {
    /// The annular copper, identical on every spanned layer. An odd-nm
    /// diameter rounds the radius up, so the disc never under-covers.
    pub fn shape(&self) -> Shape {
        Shape::Circle(Circle::from_diameter(self.center, self.diameter))
    }

    pub fn hole(&self) -> Shape {
        Shape::Circle(Circle::from_diameter(self.center, self.drill))
    }
}

// ─── Zones and keepouts ──────────────────────────────────────────────────────

/// One piece of poured (or, if unfilled, authored) zone copper on one layer.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ZonePiece {
    pub id: ZoneId,
    pub net: Option<NetId>,
    pub layer: LayerId,
    pub region: Polygon,
    /// True when `region` is KiCad's refilled copper; false when the zone
    /// had no fill and `region` is its authored outline (an upper bound on
    /// the copper a refill could pour).
    pub filled: bool,
    /// KiCad's `(island)` marker on the fill piece.
    pub island: bool,
    pub zone_name: Option<String>,
    pub priority: u32,
}

/// What a keepout forbids; `true` means not allowed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
pub struct KeepoutRules {
    pub tracks: bool,
    pub vias: bool,
    pub pads: bool,
    pub copperpour: bool,
    pub footprints: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Keepout {
    pub id: KeepoutId,
    pub name: Option<String>,
    pub layers: LayerMask,
    /// Arcs circumscribed: the region never under-covers the authored area.
    pub region: Polygon,
    pub rules: KeepoutRules,
}

// ─── Derived helpers ─────────────────────────────────────────────────────────

/// Pick each net's differential-pair partner by KiCad's suffix convention:
/// a trailing `+`/`-`, or a trailing `P`/`N` (which covers `_P`/`_N`). Both
/// halves must exist; a name whose partner is missing stays unpaired.
pub fn pair_partners(names: &[String]) -> Vec<Option<NetId>> {
    let index: BTreeMap<&str, u32> = names
        .iter()
        .enumerate()
        .map(|(i, n)| (n.as_str(), i as u32))
        .collect();
    names
        .iter()
        .map(|name| {
            let (split, _) = name.char_indices().last()?;
            let (stem, last) = name.split_at(split);
            let partner = match last {
                "+" => format!("{stem}-"),
                "-" => format!("{stem}+"),
                "P" => format!("{stem}N"),
                "N" => format!("{stem}P"),
                _ => return None,
            };
            index.get(partner.as_str()).map(|&i| NetId(i))
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn layers_sort_into_stack_order() {
        let t = LayerTable::from_layers(vec![
            ("B.Cu".into(), 2, CopperLayerKind::Signal),
            ("In2.Cu".into(), 6, CopperLayerKind::Signal),
            ("F.Cu".into(), 0, CopperLayerKind::Signal),
            ("In10.Cu".into(), 22, CopperLayerKind::Power),
            ("In1.Cu".into(), 4, CopperLayerKind::Power),
        ]);
        let names: Vec<&str> = t.copper.iter().map(|l| l.name.as_str()).collect();
        assert_eq!(names, vec!["F.Cu", "In1.Cu", "In2.Cu", "In10.Cu", "B.Cu"]);
        assert_eq!(t.by_name("In2.Cu"), Some(LayerId(2)));
        assert_eq!(t.expand("*.Cu").count(), 5);
        assert_eq!(
            t.expand("F&B.Cu").iter().collect::<Vec<_>>(),
            vec![LayerId(0), LayerId(4)]
        );
        assert!(t.expand("F.Mask").is_empty());
    }

    #[test]
    fn pair_partners_need_both_halves() {
        let names: Vec<String> = ["CAN_N", "CAN_P", "USB_D+", "USB_D-", "VBUSP", "X"]
            .into_iter()
            .map(String::from)
            .collect();
        let pairs = pair_partners(&names);
        assert_eq!(pairs[0], Some(NetId(1)));
        assert_eq!(pairs[1], Some(NetId(0)));
        assert_eq!(pairs[2], Some(NetId(3)));
        assert_eq!(pairs[3], Some(NetId(2)));
        assert_eq!(pairs[4], None);
        assert_eq!(pairs[5], None);
    }
}
