//! The board's fixed copper, holes, edges and keepouts as one indexed list,
//! with the exact checks a proposal must pass before it is committed.
//!
//! Every hard obstacle is kept once, with what the rule table needs to
//! resolve its clearance (kind, net, owning footprint, the courtyards it
//! intersects, local clearance). Rasters are painted from this list, and the
//! same list re-checks every committed segment and via exactly, so a raster
//! that under-protects (a profile's representative net differs from the
//! routed one, or a via rule differs from a track rule) is caught here, not
//! by KiCad.

use crate::api::{FootprintId, ItemKind, LayerId, LayerMask, NetId};
use crate::board::{PadKind, RouteBoard};
use crate::rules::{RuleSubject, RuleTable};
use konnect_geom::{
    gap_cmp, touches, ArcPolicy, BBox, Circle, Coord, IndexEntry, LayerIndex, Segment, Shape,
};
use std::cmp::Ordering;

/// What an obstacle is, which decides the rule that spaces new copper from it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ObstacleKind {
    /// Copper of a pad, track, arc, via or filled zone piece.
    Copper(ItemKind),
    /// A non-plated hole: `hole_clearance` to copper, `hole_to_hole` to drills.
    NpthHole,
    /// A plated drill (its copper is a separate obstacle): `hole_clearance`
    /// to other nets' copper, `hole_to_hole` to drills.
    Drill,
    /// A board edge piece: `edge_clearance`.
    Edge,
    /// A rule area forbidding tracks on its layers.
    KeepoutTracks,
    /// A rule area forbidding vias on its layers.
    KeepoutVias,
}

#[derive(Debug, Clone)]
pub(crate) struct Obstacle {
    pub shape: Shape,
    pub layers: LayerMask,
    pub net: Option<NetId>,
    pub kind: ObstacleKind,
    pub footprint: Option<FootprintId>,
    /// Footprints whose courtyard this obstacle touches, sorted.
    pub courtyards: Vec<FootprintId>,
    pub local_clearance: Option<Coord>,
}

/// Courtyard polygons, for `intersectsCourtyard()` subjects.
#[derive(Debug, Default)]
pub(crate) struct Courtyards {
    index: LayerIndex,
    regions: Vec<(FootprintId, Shape)>,
}

impl Courtyards {
    fn build(board: &RouteBoard) -> Self {
        let mut regions = Vec::new();
        for f in &board.footprints {
            for c in &f.courtyards {
                regions.push((f.id, Shape::Polygon(c.region.clone())));
            }
        }
        let entries = regions
            .iter()
            .enumerate()
            .map(|(i, (_, r))| IndexEntry::new(i as u32, r.bbox()))
            .collect();
        Courtyards {
            index: LayerIndex::bulk_load(entries),
            regions,
        }
    }

    /// Footprints whose courtyard `shape` touches, sorted and unique.
    pub fn hits(&self, shape: &Shape) -> Vec<FootprintId> {
        let mut out: Vec<FootprintId> = self
            .index
            .query(&shape.bbox())
            .into_iter()
            .filter(|&i| {
                let (_, region) = &self.regions[i as usize];
                touches(region, shape)
            })
            .map(|i| self.regions[i as usize].0)
            .collect();
        out.sort_unstable();
        out.dedup();
        out
    }
}

/// Every hard obstacle of the board, indexed per copper layer.
#[derive(Debug)]
pub(crate) struct Obstacles {
    pub items: Vec<Obstacle>,
    /// Per copper layer: copper and keepouts on that layer.
    layers: Vec<LayerIndex>,
    /// Edges and non-plated holes, which apply on every layer.
    global: LayerIndex,
    /// Drills and non-plated holes, for hole-to-hole.
    holes: LayerIndex,
    pub courtyards: Courtyards,
    /// Query inflation that finds every obstacle any rule can involve.
    pub reach: Coord,
    routable: LayerMask,
    all: LayerMask,
}

impl Obstacles {
    pub fn build(board: &RouteBoard, rules: &dyn RuleTable) -> Self {
        let courtyards = Courtyards::build(board);
        let all = board.layers.all();
        let mut items: Vec<Obstacle> = Vec::new();
        let mut push = |shape: Shape,
                        layers: LayerMask,
                        net: Option<NetId>,
                        kind: ObstacleKind,
                        footprint: Option<FootprintId>,
                        local_clearance: Option<Coord>| {
            let cy = match kind {
                // A zone's courtyard set would be every footprint on it; the
                // rules compare zones by net and layer, not location.
                ObstacleKind::Copper(ItemKind::Zone) => Vec::new(),
                ObstacleKind::Copper(_) => courtyards.hits(&shape),
                _ => Vec::new(),
            };
            items.push(Obstacle {
                shape,
                layers,
                net,
                kind,
                footprint,
                courtyards: cy,
                local_clearance,
            });
        };
        for pad in &board.pads {
            for s in &pad.shapes {
                push(
                    s.clone(),
                    pad.layers,
                    pad.net,
                    ObstacleKind::Copper(ItemKind::Pad),
                    Some(pad.footprint),
                    pad.local_clearance,
                );
            }
            if let Some(d) = &pad.drill {
                let (kind, net) = if d.plated && pad.kind != PadKind::NpThroughHole {
                    (ObstacleKind::Drill, pad.net)
                } else {
                    (ObstacleKind::NpthHole, None)
                };
                push(d.shape.clone(), all, net, kind, Some(pad.footprint), None);
            }
        }
        for t in &board.tracks {
            push(
                Shape::Segment(t.seg),
                LayerMask::single(t.layer),
                t.net,
                ObstacleKind::Copper(ItemKind::Track),
                None,
                None,
            );
        }
        // Arcs are lowered to inscribed chords; widening each chord by twice
        // the chord error makes it cover the true arc.
        let policy = ArcPolicy::default();
        for a in &board.arcs {
            for mut s in a.segments(&policy) {
                s.width = s.width.saturating_add(2 * policy.max_error_nm);
                push(
                    Shape::Segment(s),
                    LayerMask::single(a.layer),
                    a.net,
                    ObstacleKind::Copper(ItemKind::Arc),
                    None,
                    None,
                );
            }
        }
        for v in &board.vias {
            push(
                v.shape(),
                v.layers,
                v.net,
                ObstacleKind::Copper(ItemKind::Via),
                None,
                None,
            );
            push(v.hole(), all, v.net, ObstacleKind::Drill, None, None);
        }
        for z in &board.zones {
            // An unfilled zone's outline only bounds what a refill could pour,
            // and a refill clears around new copper; only real fills on layers
            // the router uses block. Plane pieces elsewhere are refilled around
            // new vias, so they constrain nothing here.
            if z.filled && board.routable.contains(z.layer) {
                push(
                    Shape::Polygon(z.region.clone()),
                    LayerMask::single(z.layer),
                    z.net,
                    ObstacleKind::Copper(ItemKind::Zone),
                    None,
                    None,
                );
            }
        }
        for k in &board.keepouts {
            if k.rules.tracks {
                push(
                    Shape::Polygon(k.region.clone()),
                    k.layers,
                    None,
                    ObstacleKind::KeepoutTracks,
                    None,
                    None,
                );
            }
            if k.rules.vias {
                push(
                    Shape::Polygon(k.region.clone()),
                    k.layers,
                    None,
                    ObstacleKind::KeepoutVias,
                    None,
                    None,
                );
            }
        }
        for e in &board.outline.edges {
            push(
                Shape::Segment(*e),
                all,
                None,
                ObstacleKind::Edge,
                None,
                None,
            );
        }

        let n_layers = board.layer_count();
        let mut per_layer: Vec<Vec<IndexEntry>> = vec![Vec::new(); n_layers];
        let mut global = Vec::new();
        let mut holes = Vec::new();
        for (i, o) in items.iter().enumerate() {
            let e = IndexEntry::new(i as u32, o.shape.bbox());
            match o.kind {
                ObstacleKind::Copper(_)
                | ObstacleKind::KeepoutTracks
                | ObstacleKind::KeepoutVias => {
                    for l in o.layers.iter() {
                        if let Some(v) = per_layer.get_mut(l.index()) {
                            v.push(e);
                        }
                    }
                }
                ObstacleKind::Edge => global.push(e),
                ObstacleKind::NpthHole => {
                    global.push(e);
                    holes.push(e);
                }
                ObstacleKind::Drill => holes.push(e),
            }
        }
        let reach = rules
            .max_clearance()
            .max(rules.hole_clearance())
            .max(rules.hole_to_hole())
            .max(rules.edge_clearance())
            .saturating_add(1);
        Obstacles {
            items,
            layers: per_layer.into_iter().map(LayerIndex::bulk_load).collect(),
            global: LayerIndex::bulk_load(global),
            holes: LayerIndex::bulk_load(holes),
            courtyards,
            reach,
            routable: board.routable,
            all,
        }
    }

    /// The rule subject of a copper obstacle.
    pub fn subject<'a>(&self, o: &'a Obstacle, fp: &'a [FootprintId; 1]) -> RuleSubject<'a> {
        let kind = match o.kind {
            ObstacleKind::Copper(k) => k,
            _ => ItemKind::Pad,
        };
        let mut s = RuleSubject::new(kind, o.net)
            .with_courtyards(&o.courtyards)
            .with_local_clearance(o.local_clearance);
        if o.footprint.is_some() {
            s = s.with_footprint(fp[0]);
        }
        s
    }

    /// The clearance new track copper of `track` (with its location's
    /// courtyards) must keep from obstacle `o` on `layer`; `None` when `o`
    /// does not constrain tracks.
    pub fn track_clearance(
        &self,
        rules: &dyn RuleTable,
        o: &Obstacle,
        track: &RuleSubject<'_>,
        layer: LayerId,
    ) -> Option<Coord> {
        match o.kind {
            ObstacleKind::Copper(_) => {
                let fp = [o.footprint.unwrap_or_default()];
                Some(rules.clearance(&self.subject(o, &fp), track, layer))
            }
            // A plated hole spaces other nets' copper by `hole_clearance`
            // (KiCad's copper-to-hole rule) on every layer, independently of
            // its annular ring; the caller skips the hole's own net.
            ObstacleKind::NpthHole | ObstacleKind::Drill => Some(rules.hole_clearance()),
            ObstacleKind::Edge => Some(rules.edge_clearance()),
            // Any overlap is a violation: a 1 nm clearance makes touching count.
            ObstacleKind::KeepoutTracks => Some(1),
            ObstacleKind::KeepoutVias => None,
        }
    }

    /// Whether obstacle `o` belongs to `net` and so does not constrain it:
    /// the net's own copper and plated holes.
    fn own(o: &Obstacle, net: NetId) -> bool {
        matches!(o.kind, ObstacleKind::Copper(_) | ObstacleKind::Drill) && o.net == Some(net)
    }

    /// Obstacle ids that constrain tracks on `layer` near `bbox`, sorted.
    pub fn track_obstacles(&self, layer: LayerId, bbox: &BBox) -> Vec<u32> {
        let q = bbox.inflate(self.reach);
        let mut ids = self
            .layers
            .get(layer.index())
            .map(|ix| ix.query(&q))
            .unwrap_or_default();
        ids.retain(|&i| self.items[i as usize].kind != ObstacleKind::KeepoutVias);
        ids.extend(self.global.query(&q));
        ids.extend(
            self.holes
                .query(&q)
                .into_iter()
                .filter(|&i| self.items[i as usize].kind == ObstacleKind::Drill),
        );
        ids.sort_unstable();
        ids.dedup();
        ids
    }

    /// The first obstacle a track segment of `net` on `layer` violates.
    pub fn check_segment(
        &self,
        rules: &dyn RuleTable,
        net: NetId,
        layer: LayerId,
        seg: &Segment,
        courtyards: &[FootprintId],
    ) -> Option<u32> {
        let shape = Shape::Segment(*seg);
        let subject = RuleSubject::new(ItemKind::Track, Some(net)).with_courtyards(courtyards);
        self.track_obstacles(layer, &seg.bbox())
            .into_iter()
            .find(|&id| {
                let o = &self.items[id as usize];
                if Self::own(o, net) {
                    return false;
                }
                self.track_clearance(rules, o, &subject, layer)
                    .is_some_and(|c| gap_cmp(&o.shape, &shape, i64::from(c)) == Ordering::Less)
            })
    }

    /// The first obstacle a through via of `net` violates: copper on every
    /// layer (other nets' zones on non-routable plane layers excepted — a
    /// refill clears them), by clearance to the via's copper and by
    /// `hole_clearance` to its hole; other nets' plated holes by
    /// `hole_clearance` to its copper; hole-to-hole to every drill; edges
    /// and via keepouts.
    pub fn check_via(
        &self,
        rules: &dyn RuleTable,
        net: NetId,
        disc: &Circle,
        drill: &Circle,
        courtyards: &[FootprintId],
    ) -> Option<u32> {
        let copper = Shape::Circle(*disc);
        let hole = Shape::Circle(*drill);
        let subject = RuleSubject::new(ItemKind::Via, Some(net)).with_courtyards(courtyards);
        let q = copper.bbox().inflate(self.reach);
        let hc = i64::from(rules.hole_clearance());
        for layer in self.all.iter() {
            let Some(ix) = self.layers.get(layer.index()) else {
                continue;
            };
            for id in ix.query(&q) {
                let o = &self.items[id as usize];
                let hit = match o.kind {
                    ObstacleKind::Copper(k) => {
                        if o.net == Some(net)
                            || (k == ItemKind::Zone && !self.routable.contains(layer))
                        {
                            false
                        } else {
                            let fp = [o.footprint.unwrap_or_default()];
                            let c = rules.clearance(&self.subject(o, &fp), &subject, layer);
                            gap_cmp(&o.shape, &copper, i64::from(c)) == Ordering::Less
                                || gap_cmp(&o.shape, &hole, hc) == Ordering::Less
                        }
                    }
                    ObstacleKind::KeepoutVias => gap_cmp(&o.shape, &copper, 1) == Ordering::Less,
                    _ => false,
                };
                if hit {
                    return Some(id);
                }
            }
        }
        for id in self.global.query(&q) {
            let o = &self.items[id as usize];
            let c = match o.kind {
                ObstacleKind::Edge => rules.edge_clearance(),
                ObstacleKind::NpthHole => rules.hole_clearance(),
                _ => continue,
            };
            if gap_cmp(&o.shape, &copper, i64::from(c)) == Ordering::Less {
                return Some(id);
            }
        }
        let h2h = rules.hole_to_hole();
        for id in self.holes.query(&q) {
            let o = &self.items[id as usize];
            let plated_other = o.kind == ObstacleKind::Drill && o.net != Some(net);
            if gap_cmp(&o.shape, &hole, i64::from(h2h)) == Ordering::Less
                || (plated_other && gap_cmp(&o.shape, &copper, hc) == Ordering::Less)
            {
                return Some(id);
            }
        }
        None
    }

    /// Via keepouts on `layer` near `bbox`.
    pub fn via_keepouts(&self, layer: LayerId, bbox: &BBox) -> Vec<u32> {
        self.layers
            .get(layer.index())
            .map(|ix| ix.query(bbox))
            .unwrap_or_default()
            .into_iter()
            .filter(|&i| self.items[i as usize].kind == ObstacleKind::KeepoutVias)
            .collect()
    }
}
