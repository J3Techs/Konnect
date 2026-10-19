//! The exact local checker the optimizer asks before it accepts an edit.
//!
//! [`crate::check::check_routed`] (WS-A) judges a whole [`RoutedCopper`] at
//! once. An optimizer proposes tens of thousands of small edits and needs
//! the same verdict for one new segment at a time, against the board and
//! against routed copper that is itself changing. [`ExactChecker`] indexes
//! the board once and answers that question with the same exact kernel
//! ([`konnect_geom::gap_cmp`]), the same rule table and the same
//! [`Violation`] vocabulary:
//!
//! - clearance to other-net pads, tracks, arcs (inscribed, plus the arc
//!   error), vias and (optionally) zone fills, through
//!   [`RuleTable::clearance`] with the item's courtyards and enclosing rule
//!   areas;
//! - hole clearance to every hole of another net (an NPTH has none, so it is
//!   always another net) and, for vias, hole-to-hole to every hole;
//! - edge clearance to the outline's edge segments;
//! - keepouts that forbid tracks or vias on the layer;
//! - the track-width and via-size floors.
//!
//! Same-net copper never needs clearance. Track angles and segment lengths
//! are the geometry guard's business ([`super::lint`]).

use crate::api::{FootprintId, ItemKind, ItemRef, LayerId, NetId};
use crate::board::RouteBoard;
use crate::check::Violation;
use crate::check::{CheckReport, CheckedItem, ViolationKind};
use crate::plan::{RoutedCopper, RoutedTrack, RoutedVia};
use crate::rules::{RuleSubject, RuleTable};
use konnect_geom::{
    gap_cmp, point_in_polygon, segments_intersect, touches, ArcPolicy, BBox, Circle, Containment,
    Coord, IndexEntry, LayerIndex, LayeredIndex, Point, Polygon, Segment, Shape,
};
use std::cmp::Ordering;

/// What the checker treats as an obstacle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CheckerOptions {
    /// Treat other nets' zone fills as copper. A refill would carve them
    /// around new copper, but the fill on the board today is copper, and
    /// treating it as such never under-protects.
    pub zones_are_obstacles: bool,
    pub arc_policy: ArcPolicy,
}

impl Default for CheckerOptions {
    fn default() -> Self {
        CheckerOptions {
            zones_are_obstacles: true,
            arc_policy: ArcPolicy::default(),
        }
    }
}

/// One piece of board copper on one or more layers.
#[derive(Debug, Clone)]
struct Obstacle {
    item: ItemRef,
    shape: Shape,
    net: Option<NetId>,
    kind: ItemKind,
    footprint: Option<FootprintId>,
    local_clearance: Option<Coord>,
    /// Added to every clearance (the arc approximation error).
    extra: Coord,
    courtyards: Vec<FootprintId>,
    areas: Vec<usize>,
}

#[derive(Debug, Clone)]
struct Hole {
    item: ItemRef,
    shape: Shape,
    net: Option<NetId>,
}

/// Routed copper indexed for clearance queries: a snapshot of a whole run,
/// or an overlay of edits that grows as they are committed.
#[derive(Debug, Clone, Default)]
pub struct RoutedSet {
    pub tracks: Vec<RoutedTrack>,
    pub vias: Vec<RoutedVia>,
    track_index: LayeredIndex,
    via_index: LayerIndex,
}

impl RoutedSet {
    pub fn new(layer_count: usize) -> Self {
        RoutedSet {
            track_index: LayeredIndex::new(layer_count),
            ..Default::default()
        }
    }

    pub fn from_copper(layer_count: usize, copper: &RoutedCopper) -> Self {
        let mut per_layer: Vec<Vec<IndexEntry>> = vec![Vec::new(); layer_count];
        for (i, t) in copper.tracks.iter().enumerate() {
            if let Some(l) = per_layer.get_mut(t.layer.index()) {
                l.push(IndexEntry::new(i as u32, t.seg.bbox()));
            }
        }
        let mut track_index = LayeredIndex::new(layer_count);
        for (l, entries) in per_layer.into_iter().enumerate() {
            track_index.set_layer(l, LayerIndex::bulk_load(entries));
        }
        let via_index = LayerIndex::bulk_load(
            copper
                .vias
                .iter()
                .enumerate()
                .map(|(i, v)| IndexEntry::new(i as u32, via_bbox(v)))
                .collect(),
        );
        RoutedSet {
            tracks: copper.tracks.clone(),
            vias: copper.vias.clone(),
            track_index,
            via_index,
        }
    }

    pub fn push_track(&mut self, t: RoutedTrack) {
        let id = self.tracks.len() as u32;
        if let Some(l) = self.track_index.layer_mut(t.layer.index()) {
            l.insert(IndexEntry::new(id, t.seg.bbox()));
        }
        self.tracks.push(t);
    }

    pub fn push_via(&mut self, v: RoutedVia) {
        let id = self.vias.len() as u32;
        self.via_index.insert(IndexEntry::new(id, via_bbox(&v)));
        self.vias.push(v);
    }

    pub fn is_empty(&self) -> bool {
        self.tracks.is_empty() && self.vias.is_empty()
    }
}

fn via_bbox(v: &RoutedVia) -> BBox {
    Circle::from_diameter(v.center, v.diameter.max(v.drill)).bbox()
}

fn via_shape(v: &RoutedVia) -> Shape {
    Shape::Circle(Circle::from_diameter(v.center, v.diameter))
}

fn via_hole(v: &RoutedVia) -> Shape {
    Shape::Circle(Circle::from_diameter(v.center, v.drill))
}

/// The board, indexed once for exact edit checks. Read-only after
/// construction, so worker threads share one.
pub struct ExactChecker<'a> {
    board: &'a RouteBoard,
    rules: &'a dyn RuleTable,
    opts: CheckerOptions,
    obstacles: Vec<Obstacle>,
    by_layer: LayeredIndex,
    holes: Vec<Hole>,
    hole_index: LayerIndex,
    edges: Vec<Shape>,
    edge_index: LayerIndex,
    keepout_index: LayeredIndex,
    courtyards: Vec<(FootprintId, Shape)>,
    courtyard_index: LayerIndex,
    area_names: Vec<String>,
    /// For each keepout, its index in `area_names` (named rule areas only).
    keepout_area: Vec<Option<usize>>,
    /// The largest clearance any rule (or hole/edge rule) can ask for, plus
    /// the arc error: no pair farther apart than this can violate anything.
    reach: Coord,
}

impl<'a> ExactChecker<'a> {
    pub fn new(board: &'a RouteBoard, rules: &'a dyn RuleTable, opts: CheckerOptions) -> Self {
        let layer_count = board.layer_count();
        let arc_extra = opts.arc_policy.max_error_nm.max(0);
        let reach = rules
            .max_clearance()
            .max(rules.hole_clearance())
            .max(rules.hole_to_hole())
            .max(rules.edge_clearance())
            .saturating_add(arc_extra)
            .saturating_add(1);

        let mut area_names = Vec::new();
        let keepout_area: Vec<Option<usize>> = board
            .keepouts
            .iter()
            .map(|k| {
                k.name.as_ref().map(|n| {
                    area_names.push(n.clone());
                    area_names.len() - 1
                })
            })
            .collect();
        let courtyards: Vec<(FootprintId, Shape)> = board
            .footprints
            .iter()
            .flat_map(|f| {
                f.courtyards
                    .iter()
                    .map(move |c| (f.id, Shape::Polygon(c.region.clone())))
            })
            .collect();
        let courtyard_index = LayerIndex::bulk_load(
            courtyards
                .iter()
                .enumerate()
                .map(|(i, (_, s))| IndexEntry::new(i as u32, s.bbox()))
                .collect(),
        );

        let mut checker = ExactChecker {
            board,
            rules,
            opts,
            obstacles: Vec::new(),
            by_layer: LayeredIndex::new(layer_count),
            holes: Vec::new(),
            hole_index: LayerIndex::new(),
            edges: board
                .outline
                .edges
                .iter()
                .map(|e| Shape::Segment(*e))
                .collect(),
            edge_index: LayerIndex::new(),
            keepout_index: LayeredIndex::new(layer_count),
            courtyards,
            courtyard_index,
            area_names,
            keepout_area,
            reach,
        };
        checker.edge_index = LayerIndex::bulk_load(
            checker
                .edges
                .iter()
                .enumerate()
                .map(|(i, s)| IndexEntry::new(i as u32, s.bbox()))
                .collect(),
        );
        let mut keepouts: Vec<Vec<IndexEntry>> = vec![Vec::new(); layer_count];
        for k in &board.keepouts {
            for l in k.layers.iter() {
                if let Some(v) = keepouts.get_mut(l.index()) {
                    v.push(IndexEntry::new(k.id.0, k.region.bbox()));
                }
            }
        }
        for (l, v) in keepouts.into_iter().enumerate() {
            checker.keepout_index.set_layer(l, LayerIndex::bulk_load(v));
        }

        // Board copper, one obstacle per (item, shape), on each of its layers.
        let mut per_layer: Vec<Vec<IndexEntry>> = vec![Vec::new(); layer_count];
        let add = |c: &mut ExactChecker<'a>,
                   per_layer: &mut Vec<Vec<IndexEntry>>,
                   layers: crate::api::LayerMask,
                   o: Obstacle| {
            let id = c.obstacles.len() as u32;
            let bbox = o.shape.bbox();
            for l in layers.iter() {
                if let Some(v) = per_layer.get_mut(l.index()) {
                    v.push(IndexEntry::new(id, bbox));
                }
            }
            c.obstacles.push(o);
        };
        for p in &board.pads {
            for shape in &p.shapes {
                let (courtyards, areas) = checker.context_of(shape);
                add(
                    &mut checker,
                    &mut per_layer,
                    p.layers,
                    Obstacle {
                        item: ItemRef::Pad(p.id),
                        shape: shape.clone(),
                        net: p.net,
                        kind: ItemKind::Pad,
                        footprint: Some(p.footprint),
                        local_clearance: p.local_clearance,
                        extra: 0,
                        courtyards,
                        areas,
                    },
                );
            }
            if let Some(d) = &p.drill {
                checker.holes.push(Hole {
                    item: ItemRef::Pad(p.id),
                    shape: d.shape.clone(),
                    net: p.net,
                });
            }
        }
        for t in &board.tracks {
            let shape = Shape::Segment(t.seg);
            let (courtyards, areas) = checker.context_of(&shape);
            add(
                &mut checker,
                &mut per_layer,
                crate::api::LayerMask::single(t.layer),
                Obstacle {
                    item: ItemRef::Track(t.id),
                    shape,
                    net: t.net,
                    kind: ItemKind::Track,
                    footprint: None,
                    local_clearance: None,
                    extra: 0,
                    courtyards,
                    areas,
                },
            );
        }
        for a in &board.arcs {
            for seg in a.segments(&opts.arc_policy) {
                let shape = Shape::Segment(seg);
                let (courtyards, areas) = checker.context_of(&shape);
                add(
                    &mut checker,
                    &mut per_layer,
                    crate::api::LayerMask::single(a.layer),
                    Obstacle {
                        item: ItemRef::Arc(a.id),
                        shape,
                        net: a.net,
                        kind: ItemKind::Arc,
                        footprint: None,
                        local_clearance: None,
                        extra: arc_extra,
                        courtyards,
                        areas,
                    },
                );
            }
        }
        for v in &board.vias {
            let shape = v.shape();
            let (courtyards, areas) = checker.context_of(&shape);
            add(
                &mut checker,
                &mut per_layer,
                v.layers,
                Obstacle {
                    item: ItemRef::Via(v.id),
                    shape,
                    net: v.net,
                    kind: ItemKind::Via,
                    footprint: None,
                    local_clearance: None,
                    extra: 0,
                    courtyards,
                    areas,
                },
            );
            checker.holes.push(Hole {
                item: ItemRef::Via(v.id),
                shape: v.hole(),
                net: v.net,
            });
        }
        for z in &board.zones {
            let shape = Shape::Polygon(z.region.clone());
            add(
                &mut checker,
                &mut per_layer,
                crate::api::LayerMask::single(z.layer),
                Obstacle {
                    item: ItemRef::Zone(z.id),
                    shape,
                    net: z.net,
                    kind: ItemKind::Zone,
                    footprint: None,
                    local_clearance: None,
                    extra: 0,
                    courtyards: Vec::new(),
                    areas: Vec::new(),
                },
            );
        }
        for (l, v) in per_layer.into_iter().enumerate() {
            checker.by_layer.set_layer(l, LayerIndex::bulk_load(v));
        }
        checker.hole_index = LayerIndex::bulk_load(
            checker
                .holes
                .iter()
                .enumerate()
                .map(|(i, h)| IndexEntry::new(i as u32, h.shape.bbox()))
                .collect(),
        );
        checker
    }

    pub fn board(&self) -> &'a RouteBoard {
        self.board
    }

    pub fn rules(&self) -> &'a dyn RuleTable {
        self.rules
    }

    /// Footprints whose courtyard `shape` touches, and the named rule areas
    /// that enclose it.
    fn context_of(&self, shape: &Shape) -> (Vec<FootprintId>, Vec<usize>) {
        let bbox = shape.bbox();
        let mut courtyards: Vec<FootprintId> = self
            .courtyard_index
            .query(&bbox)
            .into_iter()
            .filter(|&i| touches(shape, &self.courtyards[i as usize].1))
            .map(|i| self.courtyards[i as usize].0)
            .collect();
        courtyards.sort_unstable();
        courtyards.dedup();
        let mut areas = Vec::new();
        for k in &self.board.keepouts {
            let Some(area) = self.keepout_area[k.id.index()] else {
                continue;
            };
            if k.region.bbox().contains_bbox(&bbox) && encloses(&k.region, shape) {
                areas.push(area);
            }
        }
        (courtyards, areas)
    }

    fn subject_clearance(
        &self,
        a: (ItemKind, Option<NetId>, &[FootprintId], &[usize]),
        o: &Obstacle,
        layer: LayerId,
    ) -> Coord {
        let a_areas: Vec<&str> = a.3.iter().map(|&i| self.area_names[i].as_str()).collect();
        let o_areas: Vec<&str> = o
            .areas
            .iter()
            .map(|&i| self.area_names[i].as_str())
            .collect();
        let sa = RuleSubject::new(a.0, a.1)
            .with_courtyards(a.2)
            .with_areas(&a_areas);
        let mut sb = RuleSubject::new(o.kind, o.net)
            .with_courtyards(&o.courtyards)
            .with_areas(&o_areas)
            .with_local_clearance(o.local_clearance);
        if let Some(f) = o.footprint {
            sb = sb.with_footprint(f);
        }
        self.rules
            .clearance(&sa, &sb, layer)
            .saturating_add(o.extra)
    }

    fn routed_clearance(
        &self,
        a: (ItemKind, Option<NetId>, &[FootprintId], &[usize]),
        b: (ItemKind, NetId, &Shape),
        layer: LayerId,
    ) -> Coord {
        let (b_courtyards, b_areas) = self.context_of(b.2);
        let a_areas: Vec<&str> = a.3.iter().map(|&i| self.area_names[i].as_str()).collect();
        let b_names: Vec<&str> = b_areas
            .iter()
            .map(|&i| self.area_names[i].as_str())
            .collect();
        let sa = RuleSubject::new(a.0, a.1)
            .with_courtyards(a.2)
            .with_areas(&a_areas);
        let sb = RuleSubject::new(b.0, Some(b.1))
            .with_courtyards(&b_courtyards)
            .with_areas(&b_names);
        self.rules.clearance(&sa, &sb, layer)
    }

    /// The first violation of a candidate track, or `None` when it is legal
    /// against the board and every set in `routed` (same-net copper is
    /// skipped; so is anything `skip` names).
    pub fn track_violation(
        &self,
        net: NetId,
        layer: LayerId,
        seg: &Segment,
        routed: &[&RoutedSet],
        skip: &dyn Fn(usize, CheckedItem) -> bool,
    ) -> Option<Violation> {
        let mut first = None;
        self.track_violations_into(net, layer, seg, routed, skip, &mut |v| {
            first = Some(v);
            false
        });
        first
    }

    /// Every violation of one track; `sink` returns `false` to stop early.
    /// `skip(set, item)` names routed items (by their set's position in
    /// `routed`) to ignore, such as the track itself.
    pub fn track_violations_into(
        &self,
        net: NetId,
        layer: LayerId,
        seg: &Segment,
        routed: &[&RoutedSet],
        skip: &dyn Fn(usize, CheckedItem) -> bool,
        sink: &mut dyn FnMut(Violation) -> bool,
    ) {
        let me = CheckedItem::RoutedTrack { index: u32::MAX };
        let shape = Shape::Segment(*seg);
        let at = konnect_geom::Point::new(
            ((i64::from(seg.a.x) + i64::from(seg.b.x)) / 2) as Coord,
            ((i64::from(seg.a.y) + i64::from(seg.b.y)) / 2) as Coord,
        );
        let viol = |kind, b, required| Violation {
            kind,
            a: me,
            b,
            layer: Some(layer),
            at,
            required,
        };
        let (courtyards, areas) = self.context_of(&shape);
        let subject = (
            ItemKind::Track,
            Some(net),
            courtyards.as_slice(),
            areas.as_slice(),
        );
        {
            let sub = RuleSubject::new(ItemKind::Track, Some(net)).with_courtyards(&courtyards);
            let min = self.rules.track_width(&sub, layer).min;
            if seg.width < min && !sink(viol(ViolationKind::TrackWidth, None, min)) {
                return;
            }
        }
        if !self.common_checks(&shape, layer, true, &viol, sink) {
            return;
        }
        let max_clr = i64::from(self.reach);
        let query = shape.bbox().inflate(self.reach);
        if let Some(index) = self.by_layer.layer(layer.index()) {
            for id in index.query(&query) {
                let o = &self.obstacles[id as usize];
                if o.net == Some(net)
                    || (o.kind == ItemKind::Zone && !self.opts.zones_are_obstacles)
                {
                    continue;
                }
                if gap_cmp(&shape, &o.shape, max_clr) != Ordering::Less {
                    continue;
                }
                let clr = self.subject_clearance(subject, o, layer);
                if gap_cmp(&shape, &o.shape, i64::from(clr)) == Ordering::Less {
                    let kind = if touches(&shape, &o.shape) {
                        ViolationKind::Short
                    } else {
                        ViolationKind::Clearance
                    };
                    if !sink(viol(kind, Some(CheckedItem::Board { item: o.item }), clr)) {
                        return;
                    }
                }
            }
        }
        let hole_clr = self.rules.hole_clearance();
        for id in self
            .hole_index
            .query(&shape.bbox().inflate(hole_clr.max(0)))
        {
            let h = &self.holes[id as usize];
            if h.net == Some(net) {
                continue;
            }
            if gap_cmp(&shape, &h.shape, i64::from(hole_clr)) == Ordering::Less
                && !sink(viol(
                    ViolationKind::HoleClearance,
                    Some(CheckedItem::Board { item: h.item }),
                    hole_clr,
                ))
            {
                return;
            }
        }
        for (set_i, set) in routed.iter().enumerate() {
            if let Some(index) = set.track_index.layer(layer.index()) {
                for id in index.query(&query) {
                    let t = &set.tracks[id as usize];
                    let item = CheckedItem::RoutedTrack { index: id };
                    if t.net == net || skip(set_i, item) {
                        continue;
                    }
                    let other = Shape::Segment(t.seg);
                    if gap_cmp(&shape, &other, max_clr) != Ordering::Less {
                        continue;
                    }
                    let clr =
                        self.routed_clearance(subject, (ItemKind::Track, t.net, &other), layer);
                    if gap_cmp(&shape, &other, i64::from(clr)) == Ordering::Less {
                        let kind = if touches(&shape, &other) {
                            ViolationKind::Short
                        } else {
                            ViolationKind::Clearance
                        };
                        if !sink(viol(kind, Some(item), clr)) {
                            return;
                        }
                    }
                }
            }
            for id in set.via_index.query(&query) {
                let v = &set.vias[id as usize];
                let item = CheckedItem::RoutedVia { index: id };
                if v.net == net || skip(set_i, item) {
                    continue;
                }
                if !v.layers.contains(layer) {
                    // Its hole still passes through every layer.
                    if gap_cmp(&shape, &via_hole(v), i64::from(hole_clr)) == Ordering::Less
                        && !sink(viol(ViolationKind::HoleClearance, Some(item), hole_clr))
                    {
                        return;
                    }
                    continue;
                }
                let other = via_shape(v);
                if gap_cmp(&shape, &other, max_clr) != Ordering::Less {
                    continue;
                }
                let clr = self.routed_clearance(subject, (ItemKind::Via, v.net, &other), layer);
                if gap_cmp(&shape, &other, i64::from(clr)) == Ordering::Less {
                    let kind = if touches(&shape, &other) {
                        ViolationKind::Short
                    } else {
                        ViolationKind::Clearance
                    };
                    if !sink(viol(kind, Some(item), clr)) {
                        return;
                    }
                }
                if gap_cmp(&shape, &via_hole(v), i64::from(hole_clr)) == Ordering::Less
                    && !sink(viol(ViolationKind::HoleClearance, Some(item), hole_clr))
                {
                    return;
                }
            }
        }
    }

    /// Edge clearance and keepouts for copper `shape` on `layer`.
    fn common_checks(
        &self,
        shape: &Shape,
        layer: LayerId,
        is_track: bool,
        viol: &dyn Fn(ViolationKind, Option<CheckedItem>, Coord) -> Violation,
        sink: &mut dyn FnMut(Violation) -> bool,
    ) -> bool {
        let edge = self.rules.edge_clearance();
        for id in self.edge_index.query(&shape.bbox().inflate(edge.max(0))) {
            if gap_cmp(shape, &self.edges[id as usize], i64::from(edge)) == Ordering::Less
                && !sink(viol(
                    ViolationKind::EdgeClearance,
                    Some(CheckedItem::Edge),
                    edge,
                ))
            {
                return false;
            }
        }
        if let Some(index) = self.keepout_index.layer(layer.index()) {
            for id in index.query(&shape.bbox()) {
                let k = &self.board.keepouts[id as usize];
                let forbids = if is_track {
                    k.rules.tracks
                } else {
                    k.rules.vias
                };
                if forbids
                    && touches(shape, &Shape::Polygon(k.region.clone()))
                    && !sink(viol(ViolationKind::Keepout, None, 0))
                {
                    return false;
                }
            }
        }
        true
    }

    /// Every violation of one via (copper on each spanned layer, its hole,
    /// its size); `sink` returns `false` to stop early.
    pub fn via_violations_into(
        &self,
        via: &RoutedVia,
        routed: &[&RoutedSet],
        skip: &dyn Fn(usize, CheckedItem) -> bool,
        sink: &mut dyn FnMut(Violation) -> bool,
    ) {
        let me = CheckedItem::RoutedVia { index: u32::MAX };
        let net = via.net;
        let shape = via_shape(via);
        let hole = via_hole(via);
        let (courtyards, areas) = self.context_of(&shape);
        let subject = (
            ItemKind::Via,
            Some(net),
            courtyards.as_slice(),
            areas.as_slice(),
        );
        {
            let a_areas: Vec<&str> = areas.iter().map(|&i| self.area_names[i].as_str()).collect();
            let sub = RuleSubject::new(ItemKind::Via, Some(net))
                .with_courtyards(&courtyards)
                .with_areas(&a_areas);
            let r = self.rules.via(&sub);
            let annular = (via.diameter - via.drill) / 2;
            let bad = if via.diameter < r.min_diameter {
                Some(r.min_diameter)
            } else if via.drill < r.min_drill {
                Some(r.min_drill)
            } else if annular < r.min_annular {
                Some(r.min_annular)
            } else {
                None
            };
            if let Some(required) = bad {
                if !sink(Violation {
                    kind: ViolationKind::ViaSize,
                    a: me,
                    b: None,
                    layer: None,
                    at: via.center,
                    required,
                }) {
                    return;
                }
            }
        }
        let max_clr = i64::from(self.reach);
        let query = shape.bbox().inflate(self.reach);
        let hole_clr = self.rules.hole_clearance();
        let h2h = self.rules.hole_to_hole();
        // Hole to hole, any net.
        for id in self.hole_index.query(&hole.bbox().inflate(h2h.max(0))) {
            let h = &self.holes[id as usize];
            if gap_cmp(&hole, &h.shape, i64::from(h2h)) == Ordering::Less
                && !sink(Violation {
                    kind: ViolationKind::HoleToHole,
                    a: me,
                    b: Some(CheckedItem::Board { item: h.item }),
                    layer: None,
                    at: via.center,
                    required: h2h,
                })
            {
                return;
            }
        }
        for (set_i, set) in routed.iter().enumerate() {
            for id in set.via_index.query(&hole.bbox().inflate(h2h.max(0))) {
                let item = CheckedItem::RoutedVia { index: id };
                if skip(set_i, item) {
                    continue;
                }
                let v = &set.vias[id as usize];
                if gap_cmp(&hole, &via_hole(v), i64::from(h2h)) == Ordering::Less
                    && !sink(Violation {
                        kind: ViolationKind::HoleToHole,
                        a: me,
                        b: Some(item),
                        layer: None,
                        at: via.center,
                        required: h2h,
                    })
                {
                    return;
                }
            }
        }
        for layer in self.board.layers.all().iter() {
            let on_layer = via.layers.contains(layer);
            let viol = |kind, b, required| Violation {
                kind,
                a: me,
                b,
                layer: Some(layer),
                at: via.center,
                required,
            };
            if on_layer && !self.common_checks(&shape, layer, false, &viol, sink) {
                return;
            }
            let Some(index) = self.by_layer.layer(layer.index()) else {
                continue;
            };
            for id in index.query(&query) {
                let o = &self.obstacles[id as usize];
                if o.net == Some(net)
                    || (o.kind == ItemKind::Zone && !self.opts.zones_are_obstacles)
                {
                    continue;
                }
                if gap_cmp(&hole, &o.shape, i64::from(hole_clr)) == Ordering::Less
                    && !sink(viol(
                        ViolationKind::HoleClearance,
                        Some(CheckedItem::Board { item: o.item }),
                        hole_clr,
                    ))
                {
                    return;
                }
                if !on_layer || gap_cmp(&shape, &o.shape, max_clr) != Ordering::Less {
                    continue;
                }
                let clr = self.subject_clearance(subject, o, layer);
                if gap_cmp(&shape, &o.shape, i64::from(clr)) == Ordering::Less {
                    let kind = if touches(&shape, &o.shape) {
                        ViolationKind::Short
                    } else {
                        ViolationKind::Clearance
                    };
                    if !sink(viol(kind, Some(CheckedItem::Board { item: o.item }), clr)) {
                        return;
                    }
                }
            }
            for (set_i, set) in routed.iter().enumerate() {
                let Some(ti) = set.track_index.layer(layer.index()) else {
                    continue;
                };
                for id in ti.query(&query) {
                    let t = &set.tracks[id as usize];
                    let item = CheckedItem::RoutedTrack { index: id };
                    if t.net == net || skip(set_i, item) {
                        continue;
                    }
                    let other = Shape::Segment(t.seg);
                    if gap_cmp(&hole, &other, i64::from(hole_clr)) == Ordering::Less
                        && !sink(viol(ViolationKind::HoleClearance, Some(item), hole_clr))
                    {
                        return;
                    }
                    if !on_layer || gap_cmp(&shape, &other, max_clr) != Ordering::Less {
                        continue;
                    }
                    let clr =
                        self.routed_clearance(subject, (ItemKind::Track, t.net, &other), layer);
                    if gap_cmp(&shape, &other, i64::from(clr)) == Ordering::Less {
                        let kind = if touches(&shape, &other) {
                            ViolationKind::Short
                        } else {
                            ViolationKind::Clearance
                        };
                        if !sink(viol(kind, Some(item), clr)) {
                            return;
                        }
                    }
                }
                if !on_layer {
                    continue;
                }
                for id in set.via_index.query(&query) {
                    let v = &set.vias[id as usize];
                    let item = CheckedItem::RoutedVia { index: id };
                    if v.net == net || skip(set_i, item) || !v.layers.contains(layer) {
                        continue;
                    }
                    let other = via_shape(v);
                    if gap_cmp(&shape, &other, max_clr) != Ordering::Less {
                        continue;
                    }
                    let clr = self.routed_clearance(subject, (ItemKind::Via, v.net, &other), layer);
                    if gap_cmp(&shape, &other, i64::from(clr)) == Ordering::Less {
                        let kind = if touches(&shape, &other) {
                            ViolationKind::Short
                        } else {
                            ViolationKind::Clearance
                        };
                        if !sink(viol(kind, Some(item), clr)) {
                            return;
                        }
                    }
                }
            }
        }
    }

    /// Check every routed item against the board and each other. Each
    /// routed-routed pair is reported once. `index` fields name positions in
    /// `copper`.
    pub fn check_copper(&self, copper: &RoutedCopper) -> CheckReport {
        use rayon::prelude::*;
        let set = RoutedSet::from_copper(self.board.layer_count(), copper);
        // Pairs with routed items are reported from the lower track, and
        // from the via side for track-via pairs.
        let mut violations: Vec<Violation> = copper
            .tracks
            .par_iter()
            .enumerate()
            .flat_map_iter(|(i, t)| {
                let me = i as u32;
                let skip = |_: usize, item: CheckedItem| match item {
                    CheckedItem::RoutedTrack { index } => index <= me,
                    CheckedItem::RoutedVia { .. } => true,
                    _ => false,
                };
                let mut out = Vec::new();
                self.track_violations_into(t.net, t.layer, &t.seg, &[&set], &skip, &mut |mut v| {
                    v.a = CheckedItem::RoutedTrack { index: me };
                    out.push(v);
                    true
                });
                out
            })
            .collect();
        violations.par_extend(copper.vias.par_iter().enumerate().flat_map_iter(|(i, v)| {
            let me = i as u32;
            let skip = |_: usize, item: CheckedItem| match item {
                CheckedItem::RoutedVia { index } => index <= me,
                _ => false,
            };
            let mut out = Vec::new();
            self.via_violations_into(v, &[&set], &skip, &mut |mut x| {
                x.a = CheckedItem::RoutedVia { index: me };
                out.push(x);
                true
            });
            out
        }));
        CheckReport { violations }
    }

    /// Same-net board items on `layer` that `shape` touches, as obstacle ids.
    pub(crate) fn same_net_touching(
        &self,
        net: NetId,
        layer: LayerId,
        shape: &Shape,
        out: &mut Vec<u32>,
    ) {
        if let Some(index) = self.by_layer.layer(layer.index()) {
            for id in index.query(&shape.bbox()) {
                let o = &self.obstacles[id as usize];
                if o.net == Some(net) && touches(shape, &o.shape) {
                    out.push(id);
                }
            }
        }
    }

    pub(crate) fn obstacle_shape(&self, id: u32) -> &Shape {
        &self.obstacles[id as usize].shape
    }

    /// The board item an obstacle is (a piece of) copper of.
    pub(crate) fn obstacle_item(&self, id: u32) -> ItemRef {
        self.obstacles[id as usize].item
    }

    pub(crate) fn options(&self) -> CheckerOptions {
        self.opts
    }

    /// Pads and vias: board copper that never moves or refills.
    pub(crate) fn obstacle_is_anchor(&self, id: u32) -> bool {
        matches!(
            self.obstacles[id as usize].kind,
            ItemKind::Pad | ItemKind::Via
        )
    }
}

/// Whether `shape` lies inside `poly` (KiCad's `enclosedByArea`): its core
/// points are inside and no ring edge crosses it. Other shapes are judged by
/// their bounding box.
fn encloses(poly: &Polygon, shape: &Shape) -> bool {
    let inside = |p: Point| point_in_polygon(p, poly) == Containment::Inside;
    let crosses = |a: Point, b: Point| {
        poly.rings().any(|ring| {
            (0..ring.len()).any(|i| segments_intersect(a, b, ring[i], ring[(i + 1) % ring.len()]))
        })
    };
    match shape {
        Shape::Segment(s) => inside(s.a) && inside(s.b) && !crosses(s.a, s.b),
        other => {
            let b = other.bbox();
            let c = [
                b.min,
                Point::new(b.max.x, b.min.y),
                b.max,
                Point::new(b.min.x, b.max.y),
            ];
            c.iter().all(|&p| inside(p)) && (0..4).all(|i| !crosses(c[i], c[(i + 1) % 4]))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::api::Generation;
    use crate::synthetic::{two_pin_nets, UniformRules};

    #[test]
    fn a_track_into_another_nets_pad_is_refused_and_a_clear_one_passes() {
        let board = two_pin_nets(3, false);
        let rules = UniformRules::default();
        let checker = ExactChecker::new(&board, &rules, CheckerOptions::default());
        let n0 = board.net_by_name("N00").unwrap();
        let mm = |v: f64| konnect_geom::mm_to_nm(v).unwrap();
        // N00 pads sit at y = 2 mm; N01 at y = 4 mm (1.0 x 0.6 mm pads).
        let clear = Segment::new(
            Point::new(mm(2.0), mm(2.0)),
            Point::new(mm(18.0), mm(2.0)),
            mm(0.15),
        );
        assert!(checker
            .track_violation(n0, LayerId(0), &clear, &[], &|_, _| false)
            .is_none());
        // 0.9 mm off its own row: within 0.2 mm of N01's pad edge (y = 3.7).
        let close = Segment::new(
            Point::new(mm(2.0), mm(3.45)),
            Point::new(mm(18.0), mm(3.45)),
            mm(0.15),
        );
        let v = checker
            .track_violation(n0, LayerId(0), &close, &[], &|_, _| false)
            .unwrap();
        assert_eq!(v.kind, ViolationKind::Clearance);
        // The same track on B.Cu meets no pad.
        assert!(checker
            .track_violation(n0, LayerId(1), &close, &[], &|_, _| false)
            .is_none());
        // Edge clearance: 0.1 mm from the board edge.
        let edge = Segment::new(
            Point::new(mm(5.0), mm(0.1)),
            Point::new(mm(15.0), mm(0.1)),
            mm(0.15),
        );
        assert_eq!(
            checker
                .track_violation(n0, LayerId(1), &edge, &[], &|_, _| false)
                .unwrap()
                .kind,
            ViolationKind::EdgeClearance
        );
    }

    #[test]
    fn routed_copper_of_other_nets_is_checked_and_reported_once() {
        let board = two_pin_nets(2, false);
        let rules = UniformRules::default();
        let checker = ExactChecker::new(&board, &rules, CheckerOptions::default());
        let (n0, n1) = (
            board.net_by_name("N00").unwrap(),
            board.net_by_name("N01").unwrap(),
        );
        let mm = |v: f64| konnect_geom::mm_to_nm(v).unwrap();
        let t = |net, y: f64| RoutedTrack {
            net,
            layer: LayerId(1),
            seg: Segment::new(
                Point::new(mm(5.0), mm(y)),
                Point::new(mm(15.0), mm(y)),
                mm(0.15),
            ),
            gen: Generation(1),
        };
        let copper = RoutedCopper {
            tracks: vec![t(n0, 2.5), t(n1, 2.8)],
            vias: vec![],
        };
        let report = checker.check_copper(&copper);
        assert_eq!(report.violations.len(), 1, "{report:?}");
        assert_eq!(report.violations[0].kind, ViolationKind::Clearance);
        let far = RoutedCopper {
            tracks: vec![t(n0, 2.5), t(n1, 3.5)],
            vias: vec![],
        };
        assert!(checker.check_copper(&far).violations.is_empty());
    }
}
