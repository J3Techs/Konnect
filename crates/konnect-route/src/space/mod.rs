//! Exact legality queries shared by the WS-B stages: the terminal audit
//! (S1), plane fanout (S2) and pin access (S4).
//!
//! [`Space`] indexes a board's fixed copper, holes, edges, keepouts,
//! courtyards and plane pieces once, and answers "may this new track or via
//! go here?" with the exact predicates of `konnect-geom` and the clearances
//! of a [`RuleTable`]. Plane fills are not obstacles: a refill pours around
//! new copper. They are indexed separately ([`PlanePiece`]) for the
//! inside-own-plane tests of S2.
//!
//! Rules are asked through [`SubjectKey`], a compact, hashable description
//! of an item as a rule sees it, and memoised per worker in a
//! [`ClearanceCache`], because the project table evaluates conditions on
//! every call.

pub(crate) mod poly;

use crate::api::{FootprintId, ItemKind, ItemRef, KeepoutId, LayerId, LayerMask, NetId};
use crate::board::{PadKind, RouteBoard};
use crate::rules::{RuleSubject, RuleTable, ViaRule, WidthRule};
use konnect_geom::{
    gap_cmp, touches, ArcPolicy, BBox, Circle, Coord, IndexEntry, LayerIndex, Point, Segment, Shape,
};
use poly::PolyIndex;
use std::cmp::Ordering;
use std::collections::HashMap;
use std::hash::{BuildHasherDefault, Hasher};

// ─── Hashing ─────────────────────────────────────────────────────────────────

/// A small multiplicative hasher for integer keys (FxHash's mixing step).
/// Only for caches: nothing iterates these maps into output.
#[derive(Default, Clone, Copy)]
pub(crate) struct FxHasher {
    hash: u64,
}

impl FxHasher {
    #[inline]
    fn add(&mut self, v: u64) {
        self.hash = (self.hash.rotate_left(5) ^ v).wrapping_mul(0x51_7c_c1_b7_27_22_0a_95);
    }
}

impl Hasher for FxHasher {
    fn write(&mut self, bytes: &[u8]) {
        for chunk in bytes.chunks(8) {
            let mut v = 0u64;
            for (i, b) in chunk.iter().enumerate() {
                v |= u64::from(*b) << (8 * i);
            }
            self.add(v);
        }
    }
    fn write_u8(&mut self, i: u8) {
        self.add(u64::from(i));
    }
    fn write_u16(&mut self, i: u16) {
        self.add(u64::from(i));
    }
    fn write_u32(&mut self, i: u32) {
        self.add(u64::from(i));
    }
    fn write_u64(&mut self, i: u64) {
        self.add(i);
    }
    fn write_i32(&mut self, i: i32) {
        self.add(i as u32 as u64);
    }
    fn write_usize(&mut self, i: usize) {
        self.add(i as u64);
    }
    fn finish(&self) -> u64 {
        self.hash
    }
}

pub(crate) type FxMap<K, V> = HashMap<K, V, BuildHasherDefault<FxHasher>>;

// ─── Rule subjects ───────────────────────────────────────────────────────────

/// Most courtyards one subject records. A point under more than this many
/// overlapping courtyards does not occur on real boards; extra ones are
/// dropped and counted in [`Space::truncated_subjects`].
const MAX_COURTS: usize = 6;
const MAX_AREAS: usize = 2;

/// An item as rules see it, compact and hashable.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) struct SubjectKey {
    pub kind: ItemKind,
    pub net: Option<NetId>,
    pub footprint: Option<FootprintId>,
    courts: [FootprintId; MAX_COURTS],
    n_courts: u8,
    areas: [u16; MAX_AREAS],
    n_areas: u8,
    pub local: Option<Coord>,
}

impl SubjectKey {
    pub(crate) fn new(kind: ItemKind, net: Option<NetId>) -> Self {
        SubjectKey {
            kind,
            net,
            footprint: None,
            courts: [FootprintId(0); MAX_COURTS],
            n_courts: 0,
            areas: [0; MAX_AREAS],
            n_areas: 0,
            local: None,
        }
    }

    pub(crate) fn courtyards(&self) -> &[FootprintId] {
        &self.courts[..self.n_courts as usize]
    }

    /// Returns false if the set was full.
    fn push_court(&mut self, f: FootprintId) -> bool {
        if self.courtyards().contains(&f) {
            return true;
        }
        if (self.n_courts as usize) < MAX_COURTS {
            self.courts[self.n_courts as usize] = f;
            self.n_courts += 1;
            self.courts[..self.n_courts as usize].sort();
            true
        } else {
            false
        }
    }

    fn push_area(&mut self, a: u16) -> bool {
        if self.areas[..self.n_areas as usize].contains(&a) {
            return true;
        }
        if (self.n_areas as usize) < MAX_AREAS {
            self.areas[self.n_areas as usize] = a;
            self.n_areas += 1;
            self.areas[..self.n_areas as usize].sort();
            true
        } else {
            false
        }
    }
}

/// Per-worker memo of rule answers.
#[derive(Default)]
pub(crate) struct ClearanceCache {
    clearance: FxMap<(SubjectKey, SubjectKey, u8), Coord>,
    via_rules: FxMap<SubjectKey, ViaRule>,
    track_rules: FxMap<(SubjectKey, u8), (WidthRule, Option<Coord>)>,
    pub(crate) misses: u64,
}

// ─── Indexed items ───────────────────────────────────────────────────────────

/// One copper shape of one board item.
#[derive(Debug, Clone)]
pub(crate) struct CopperItem {
    pub item: ItemRef,
    pub net: Option<NetId>,
    pub key: SubjectKey,
    pub layers: LayerMask,
    pub shape: Shape,
    /// Added to any clearance checked against this shape (inscribed arcs).
    pub extra: Coord,
}

/// A drilled hole.
#[derive(Debug, Clone)]
pub(crate) struct HoleItem {
    pub item: ItemRef,
    pub net: Option<NetId>,
    pub shape: Shape,
}

/// One filled plane piece, indexed for containment.
#[derive(Debug, Clone)]
pub(crate) struct PlanePiece {
    pub net: Option<NetId>,
    pub layer: LayerId,
    /// Not the layer's base pour: a higher-priority zone, or a piece KiCad
    /// flags as an island.
    pub island: bool,
    pub index: PolyIndex,
}

/// Why a candidate shape is illegal.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Conflict {
    /// Too close to copper of another net on `layer`.
    Copper { item: ItemRef, layer: LayerId },
    /// Hole-to-hole distance to another hole.
    HoleToHole { item: ItemRef },
    /// Hole clearance between a hole and copper of another net.
    HoleClearance { item: ItemRef },
    /// Copper to board edge.
    Edge,
    /// Inside a rule area that forbids the item.
    Keepout { id: KeepoutId },
}

/// The indexed board.
pub(crate) struct Space<'b> {
    pub board: &'b RouteBoard,
    pub items: Vec<CopperItem>,
    layer_index: Vec<LayerIndex>,
    pub holes: Vec<HoleItem>,
    hole_index: LayerIndex,
    edge_index: LayerIndex,
    keepout_index: LayerIndex,
    /// Each keepout's region as a shape, by keepout index (built once so
    /// queries need not clone polygons).
    pub keepout_shapes: Vec<Shape>,
    courtyards: Vec<(FootprintId, Shape)>,
    court_index: LayerIndex,
    /// Names of keepouts that conditions can test with `enclosedByArea`.
    area_names: Vec<&'b str>,
    area_of_keepout: Vec<Option<u16>>,
    pub planes: Vec<PlanePiece>,
    /// Largest clearance any query can need, for index inflation.
    pub reach: Coord,
    pub edge_clearance: Coord,
    pub hole_clearance: Coord,
    pub hole_to_hole: Coord,
    pub truncated_subjects: usize,
}

fn half_up(d: Coord) -> Coord {
    d / 2 + d % 2
}

impl<'b> Space<'b> {
    /// Index `board`. With `planes`, filled zone pieces are indexed for
    /// containment queries too (costly for large fills; S2 only).
    pub(crate) fn new(
        board: &'b RouteBoard,
        rules: &dyn RuleTable,
        policy: &ArcPolicy,
        planes: bool,
    ) -> Self {
        let mut courtyards = Vec::new();
        for f in &board.footprints {
            for c in &f.courtyards {
                courtyards.push((f.id, Shape::Polygon(c.region.clone())));
            }
        }
        let court_index = LayerIndex::bulk_load(
            courtyards
                .iter()
                .enumerate()
                .map(|(i, (_, s))| IndexEntry::new(i as u32, s.bbox()))
                .collect(),
        );
        let mut area_names = Vec::new();
        let mut area_of_keepout = Vec::new();
        for k in &board.keepouts {
            match k.name.as_deref() {
                Some(n) if area_names.len() < u16::MAX as usize => {
                    area_of_keepout.push(Some(area_names.len() as u16));
                    area_names.push(n);
                }
                _ => area_of_keepout.push(None),
            }
        }
        let keepout_index = LayerIndex::bulk_load(
            board
                .keepouts
                .iter()
                .map(|k| IndexEntry::new(k.id.0, k.region.bbox()))
                .collect(),
        );
        let edge_index = LayerIndex::bulk_load(
            board
                .outline
                .edges
                .iter()
                .enumerate()
                .map(|(i, e)| IndexEntry::new(i as u32, e.bbox()))
                .collect(),
        );
        let mut space = Space {
            board,
            items: Vec::new(),
            layer_index: Vec::new(),
            holes: Vec::new(),
            hole_index: LayerIndex::new(),
            edge_index,
            keepout_index,
            keepout_shapes: board
                .keepouts
                .iter()
                .map(|k| Shape::Polygon(k.region.clone()))
                .collect(),
            courtyards,
            court_index,
            area_names,
            area_of_keepout,
            planes: Vec::new(),
            reach: 0,
            edge_clearance: rules.edge_clearance(),
            hole_clearance: rules.hole_clearance(),
            hole_to_hole: rules.hole_to_hole(),
            truncated_subjects: 0,
        };

        let mut items = Vec::new();
        for pad in &board.pads {
            let mut key = SubjectKey::new(ItemKind::Pad, pad.net);
            key.footprint = Some(pad.footprint);
            key.local = pad.local_clearance;
            for s in &pad.shapes {
                space.add_context(&mut key, s);
            }
            for s in &pad.shapes {
                if !pad.layers.is_empty() {
                    items.push(CopperItem {
                        item: ItemRef::Pad(pad.id),
                        net: pad.net,
                        key,
                        layers: pad.layers,
                        shape: s.clone(),
                        extra: 0,
                    });
                }
            }
            if let Some(d) = &pad.drill {
                space.holes.push(HoleItem {
                    item: ItemRef::Pad(pad.id),
                    net: if pad.kind == PadKind::NpThroughHole {
                        None
                    } else {
                        pad.net
                    },
                    shape: d.shape.clone(),
                });
            }
        }
        for t in &board.tracks {
            let mut key = SubjectKey::new(ItemKind::Track, t.net);
            let shape = Shape::Segment(t.seg);
            space.add_context(&mut key, &shape);
            items.push(CopperItem {
                item: ItemRef::Track(t.id),
                net: t.net,
                key,
                layers: LayerMask::single(t.layer),
                shape,
                extra: 0,
            });
        }
        for a in &board.arcs {
            let mut key = SubjectKey::new(ItemKind::Arc, a.net);
            let segs = a.segments(policy);
            for s in &segs {
                space.add_context(&mut key, &Shape::Segment(*s));
            }
            for s in segs {
                items.push(CopperItem {
                    item: ItemRef::Arc(a.id),
                    net: a.net,
                    key,
                    layers: LayerMask::single(a.layer),
                    shape: Shape::Segment(s),
                    extra: policy.max_error_nm,
                });
            }
        }
        for v in &board.vias {
            let mut key = SubjectKey::new(ItemKind::Via, v.net);
            let shape = v.shape();
            space.add_context(&mut key, &shape);
            items.push(CopperItem {
                item: ItemRef::Via(v.id),
                net: v.net,
                key,
                layers: v.layers,
                shape,
                extra: 0,
            });
            space.holes.push(HoleItem {
                item: ItemRef::Via(v.id),
                net: v.net,
                shape: v.hole(),
            });
        }
        let n_layers = board.layer_count();
        let mut per_layer: Vec<Vec<IndexEntry>> = vec![Vec::new(); n_layers];
        for (i, it) in items.iter().enumerate() {
            let bb = it.shape.bbox();
            for l in it.layers.iter() {
                if let Some(list) = per_layer.get_mut(l.index()) {
                    list.push(IndexEntry::new(i as u32, bb));
                }
            }
        }
        space.layer_index = per_layer.into_iter().map(LayerIndex::bulk_load).collect();
        space.items = items;
        space.hole_index = LayerIndex::bulk_load(
            space
                .holes
                .iter()
                .enumerate()
                .map(|(i, h)| IndexEntry::new(i as u32, h.shape.bbox()))
                .collect(),
        );
        space.reach = rules
            .max_clearance()
            .max(space.edge_clearance)
            .max(space.hole_clearance)
            .max(space.hole_to_hole)
            + policy.max_error_nm;

        if planes {
            space.planes = index_planes(board);
        }
        space
    }

    /// Courtyards and rule areas of `shape`, added to `key`.
    pub(crate) fn add_context(&mut self, key: &mut SubjectKey, shape: &Shape) {
        let truncated = !self.context_into(key, shape);
        self.truncated_subjects += usize::from(truncated);
    }

    /// Like [`Self::add_context`] for a shared `Space`; `false` when a set
    /// overflowed.
    pub(crate) fn context_into(&self, key: &mut SubjectKey, shape: &Shape) -> bool {
        let bb = shape.bbox();
        let mut ok = true;
        for e in self.court_index.query_entries(&bb) {
            let (fp, court) = &self.courtyards[e.id as usize];
            if key.courtyards().contains(fp) {
                continue;
            }
            if touches(shape, court) {
                ok &= key.push_court(*fp);
            }
        }
        if !self.area_names.is_empty() {
            for e in self.keepout_index.query_entries(&bb) {
                let k = &self.board.keepouts[e.id as usize];
                let Some(area) = self.area_of_keepout[e.id as usize] else {
                    continue;
                };
                // KiCad's enclosedByArea: the item lies inside the area.
                if k.region.bbox().contains_bbox(&bb) && shape_inside_polygon(shape, &k.region) {
                    ok &= key.push_area(area);
                }
            }
        }
        ok
    }

    /// A subject key for a new item of `kind` and `net` with `shape`.
    pub(crate) fn subject_for(
        &self,
        kind: ItemKind,
        net: Option<NetId>,
        shape: &Shape,
    ) -> SubjectKey {
        let mut key = SubjectKey::new(kind, net);
        self.context_into(&mut key, shape);
        key
    }

    /// The track width rule and minimum segment length for a subject on
    /// `layer`, memoised per worker.
    pub(crate) fn track_rules(
        &self,
        cache: &mut ClearanceCache,
        rules: &dyn RuleTable,
        key: &SubjectKey,
        layer: LayerId,
    ) -> (WidthRule, Option<Coord>) {
        let k = (*key, layer.0);
        if let Some(v) = cache.track_rules.get(&k) {
            return *v;
        }
        cache.misses += 1;
        let v = self.with_subject(key, |s| {
            (
                rules.track_width(&s, layer),
                rules.min_segment_length(&s, layer),
            )
        });
        cache.track_rules.insert(k, v);
        v
    }

    /// The via rule for a subject, memoised per worker.
    pub(crate) fn via_rule(
        &self,
        cache: &mut ClearanceCache,
        rules: &dyn RuleTable,
        key: &SubjectKey,
    ) -> ViaRule {
        if let Some(v) = cache.via_rules.get(key) {
            return *v;
        }
        cache.misses += 1;
        let v = self.with_subject(key, |s| rules.via(&s));
        cache.via_rules.insert(*key, v);
        v
    }

    pub(crate) fn clearance(
        &self,
        cache: &mut ClearanceCache,
        rules: &dyn RuleTable,
        a: &SubjectKey,
        b: &SubjectKey,
        layer: LayerId,
    ) -> Coord {
        let k = (*a, *b, layer.0);
        if let Some(c) = cache.clearance.get(&k) {
            return *c;
        }
        cache.misses += 1;
        let c = self.with_subject(a, |sa| {
            self.with_subject(b, |sb| rules.clearance(&sa, &sb, layer))
        });
        cache.clearance.insert(k, c);
        c
    }

    pub(crate) fn with_subject<R>(
        &self,
        key: &SubjectKey,
        f: impl FnOnce(RuleSubject<'_>) -> R,
    ) -> R {
        let n = key.n_areas as usize;
        let mut areas: [&str; MAX_AREAS] = [""; MAX_AREAS];
        for (slot, &a) in areas.iter_mut().zip(&key.areas[..n]) {
            *slot = self.area_names[a as usize];
        }
        let mut s = RuleSubject::new(key.kind, key.net)
            .with_courtyards(key.courtyards())
            .with_areas(&areas[..n])
            .with_local_clearance(key.local);
        s.footprint = key.footprint;
        f(s)
    }

    /// Item ids (into `items`) whose shapes on `layer` meet `bbox`.
    pub(crate) fn items_near<'a>(
        &'a self,
        layer: LayerId,
        bbox: &BBox,
    ) -> impl Iterator<Item = u32> + 'a {
        let bbox = *bbox;
        self.layer_index
            .get(layer.index())
            .into_iter()
            .flat_map(move |ix| ix.query_entries(&bbox).map(|e| e.id))
    }

    pub(crate) fn holes_near<'a>(&'a self, bbox: &BBox) -> impl Iterator<Item = u32> + 'a {
        let bbox = *bbox;
        self.hole_index.query_entries(&bbox).map(|e| e.id)
    }

    /// The first copper item on `layer` of a net other than `net` that
    /// `shape` (rule subject `key`) comes too close to. `extra` is added to
    /// every clearance (a margin or an arc error).
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn copper_conflict(
        &self,
        cache: &mut ClearanceCache,
        rules: &dyn RuleTable,
        layer: LayerId,
        shape: &Shape,
        key: &SubjectKey,
        net: Option<NetId>,
        extra: Coord,
    ) -> Option<Conflict> {
        let bb = shape.bbox().inflate(self.reach + extra);
        for id in self.items_near(layer, &bb) {
            let it = &self.items[id as usize];
            if net.is_some() && it.net == net {
                continue;
            }
            let clr = self.clearance(cache, rules, key, &it.key, layer) + it.extra + extra;
            if gap_cmp(shape, &it.shape, i64::from(clr)) == Ordering::Less {
                return Some(Conflict::Copper {
                    item: it.item,
                    layer,
                });
            }
        }
        None
    }

    /// Hole clearance from `copper` (on `layers`) to holes of other nets, and
    /// from an optional new `hole` to other holes (hole-to-hole) and to
    /// other nets' copper (hole clearance).
    pub(crate) fn hole_conflict(
        &self,
        copper: &Shape,
        hole: Option<&Shape>,
        layers: LayerMask,
        net: Option<NetId>,
    ) -> Option<Conflict> {
        let reach = self.reach;
        for id in self.holes_near(&copper.bbox().inflate(reach)) {
            let h = &self.holes[id as usize];
            let other_net = h.net.is_none() || h.net != net;
            if other_net
                && gap_cmp(copper, &h.shape, i64::from(self.hole_clearance)) == Ordering::Less
            {
                return Some(Conflict::HoleClearance { item: h.item });
            }
            if let Some(hole) = hole {
                if gap_cmp(hole, &h.shape, i64::from(self.hole_to_hole)) == Ordering::Less {
                    return Some(Conflict::HoleToHole { item: h.item });
                }
            }
        }
        if let Some(hole) = hole {
            let bb = hole.bbox().inflate(reach);
            for layer in layers.iter() {
                for id in self.items_near(layer, &bb) {
                    let it = &self.items[id as usize];
                    if net.is_some() && it.net == net {
                        continue;
                    }
                    if gap_cmp(hole, &it.shape, i64::from(self.hole_clearance + it.extra))
                        == Ordering::Less
                    {
                        return Some(Conflict::HoleClearance { item: it.item });
                    }
                }
            }
        }
        None
    }

    pub(crate) fn edge_conflict(&self, shape: &Shape) -> Option<Conflict> {
        let bb = shape.bbox().inflate(self.edge_clearance);
        for e in self.edge_index.query_entries(&bb) {
            let seg = Shape::Segment(self.board.outline.edges[e.id as usize]);
            if gap_cmp(shape, &seg, i64::from(self.edge_clearance)) == Ordering::Less {
                return Some(Conflict::Edge);
            }
        }
        None
    }

    /// A keepout on any of `layers` that forbids `kind` and meets `shape`.
    pub(crate) fn keepout_conflict(
        &self,
        shape: &Shape,
        layers: LayerMask,
        kind: ItemKind,
    ) -> Option<Conflict> {
        for e in self.keepout_index.query_entries(&shape.bbox()) {
            let k = &self.board.keepouts[e.id as usize];
            let forbids = match kind {
                ItemKind::Via => k.rules.vias,
                ItemKind::Track | ItemKind::Arc => k.rules.tracks,
                ItemKind::Pad => k.rules.pads,
                ItemKind::Zone => k.rules.copperpour,
            };
            if forbids
                && k.layers.intersects(layers)
                && touches(shape, &self.keepout_shapes[e.id as usize])
            {
                return Some(Conflict::Keepout { id: k.id });
            }
        }
        None
    }

    /// Every rule a new through via of `net` at `center` must meet against
    /// existing copper: annulus clearance on every layer it spans, hole to
    /// hole, hole clearance, board edge and via keepouts.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn via_conflict(
        &self,
        cache: &mut ClearanceCache,
        rules: &dyn RuleTable,
        center: Point,
        diameter: Coord,
        drill: Coord,
        layers: LayerMask,
        net: Option<NetId>,
        key: &SubjectKey,
    ) -> Option<Conflict> {
        let disc = Shape::Circle(Circle::from_diameter(center, diameter));
        let hole = Shape::Circle(Circle::from_diameter(center, drill));
        if let Some(c) = self.edge_conflict(&disc) {
            return Some(c);
        }
        if let Some(c) = self.keepout_conflict(&disc, layers, ItemKind::Via) {
            return Some(c);
        }
        if let Some(c) = self.hole_conflict(&disc, Some(&hole), layers, net) {
            return Some(c);
        }
        for layer in layers.iter() {
            if let Some(c) = self.copper_conflict(cache, rules, layer, &disc, key, net, 0) {
                return Some(c);
            }
        }
        None
    }

    /// Every rule a new track `seg` of `net` on `layer` must meet.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn track_conflict(
        &self,
        cache: &mut ClearanceCache,
        rules: &dyn RuleTable,
        layer: LayerId,
        seg: Segment,
        net: Option<NetId>,
        key: &SubjectKey,
    ) -> Option<Conflict> {
        let shape = Shape::Segment(seg);
        if let Some(c) = self.edge_conflict(&shape) {
            return Some(c);
        }
        if let Some(c) = self.keepout_conflict(&shape, LayerMask::single(layer), ItemKind::Track) {
            return Some(c);
        }
        if let Some(c) = self.hole_conflict(&shape, None, LayerMask::single(layer), net) {
            return Some(c);
        }
        self.copper_conflict(cache, rules, layer, &shape, key, net, 0)
    }
}

/// Whether `shape` lies inside `poly` (approximated by its bounding box
/// corners for rule-area membership; areas are rectangles in practice).
fn shape_inside_polygon(shape: &Shape, poly: &konnect_geom::Polygon) -> bool {
    let bb = shape.bbox();
    [
        bb.min,
        bb.max,
        Point::new(bb.min.x, bb.max.y),
        Point::new(bb.max.x, bb.min.y),
    ]
    .iter()
    .all(|p| konnect_geom::point_in_polygon(*p, poly) != konnect_geom::Containment::Outside)
}

/// Index every filled, netted zone piece. A piece is an island when KiCad
/// flags it, or when its zone's priority is above the lowest on its layer
/// (the base pour).
fn index_planes(board: &RouteBoard) -> Vec<PlanePiece> {
    let mut base_priority: BTreeLayerMin = BTreeLayerMin::default();
    for z in board.zones.iter().filter(|z| z.filled && z.net.is_some()) {
        base_priority.note(z.layer, z.priority);
    }
    let pieces: Vec<&crate::board::ZonePiece> = board
        .zones
        .iter()
        .filter(|z| z.filled && z.net.is_some())
        .collect();
    use rayon::prelude::*;
    pieces
        .par_iter()
        .map(|z| PlanePiece {
            net: z.net,
            layer: z.layer,
            island: z.island || Some(z.priority) != base_priority.get(z.layer),
            index: PolyIndex::with_target(
                &z.region,
                // Small islands need no big grid.
                (z.region.rings().map(<[Point]>::len).sum::<usize>() * 16)
                    .clamp(1_024, poly::DEFAULT_TARGET_CELLS),
            ),
        })
        .collect()
}

#[derive(Default)]
struct BTreeLayerMin(std::collections::BTreeMap<LayerId, u32>);

impl BTreeLayerMin {
    fn note(&mut self, l: LayerId, p: u32) {
        let e = self.0.entry(l).or_insert(p);
        *e = (*e).min(p);
    }
    fn get(&self, l: LayerId) -> Option<u32> {
        self.0.get(&l).copied()
    }
}

/// Radius of a disc of diameter `d` as [`Circle::from_diameter`] rounds it.
pub(crate) fn radius_of(d: Coord) -> Coord {
    half_up(d)
}
