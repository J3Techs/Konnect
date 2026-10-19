//! Exact conflict detection between routed connections, and the PathFinder
//! history it feeds.
//!
//! Rasters only guide the search; whether two connections are in conflict
//! is decided here, exactly, with the rule table: different nets closer
//! than their clearance, or two vias closer than hole-to-hole (any nets).
//! Every conflicting pair marks history on the lattice cells where the two
//! centre lines are too close. History never decays.

use super::copper::PieceCopper;
use crate::api::{ItemKind, LayerId, LayerMask, LATTICE_PITCH_NM};
use crate::compute::{Plane, Window};
use crate::rules::RuleTable;
use konnect_geom::{gap_cmp, BBox, Circle, Coord, IndexEntry, LayerIndex, Point, Shape};
use rayon::prelude::*;
use std::cmp::Ordering;
use std::collections::{HashMap, HashSet};
use std::sync::Arc;

/// One indexed routed item: a segment or via of a piece.
#[derive(Debug, Clone, Copy)]
struct Item {
    piece: u32,
    /// `Some(i)`: segment `i`; `None` with `via`: via `via`.
    seg: Option<u32>,
    via: u32,
}

/// Every routed piece's segments and vias in per-layer R-trees.
pub(crate) struct RoutedIndex {
    pub pieces: Vec<Arc<PieceCopper>>,
    items: Vec<Item>,
    layers: Vec<LayerIndex>,
    vias: LayerIndex,
    reach: Coord,
}

impl RoutedIndex {
    pub fn build(pieces: Vec<Arc<PieceCopper>>, layer_count: usize, reach: Coord) -> Self {
        let mut items = Vec::new();
        let mut per_layer: Vec<Vec<IndexEntry>> = vec![Vec::new(); layer_count];
        let mut vias = Vec::new();
        for (p, piece) in pieces.iter().enumerate() {
            for (s, seg) in piece.segs.iter().enumerate() {
                let id = items.len() as u32;
                items.push(Item {
                    piece: p as u32,
                    seg: Some(s as u32),
                    via: 0,
                });
                if let Some(v) = per_layer.get_mut(seg.layer.index()) {
                    v.push(IndexEntry::new(id, seg.seg.bbox()));
                }
            }
            for (v, via) in piece.vias.iter().enumerate() {
                let id = items.len() as u32;
                items.push(Item {
                    piece: p as u32,
                    seg: None,
                    via: v as u32,
                });
                let e = IndexEntry::new(id, via.disc().bbox());
                for l in per_layer.iter_mut() {
                    l.push(e);
                }
                vias.push(e);
            }
        }
        RoutedIndex {
            pieces,
            items,
            layers: per_layer.into_iter().map(LayerIndex::bulk_load).collect(),
            vias: LayerIndex::bulk_load(vias),
            reach,
        }
    }

    fn piece_of(&self, item: u32) -> &PieceCopper {
        &self.pieces[self.items[item as usize].piece as usize]
    }

    fn shape_of(&self, item: u32) -> (Shape, ItemKind, Option<LayerId>) {
        let it = self.items[item as usize];
        let p = self.piece_of(item);
        match it.seg {
            Some(s) => {
                let s = &p.segs[s as usize];
                (Shape::Segment(s.seg), ItemKind::Track, Some(s.layer))
            }
            None => (
                Shape::Circle(p.vias[it.via as usize].disc()),
                ItemKind::Via,
                None,
            ),
        }
    }

    /// The hole of a via item.
    fn hole_of(&self, item: u32) -> Option<Shape> {
        let it = self.items[item as usize];
        it.seg
            .is_none()
            .then(|| Shape::Circle(self.piece_of(item).vias[it.via as usize].hole()))
    }

    fn courtyards_of(&self, item: u32) -> &[crate::api::FootprintId] {
        let it = self.items[item as usize];
        let p = self.piece_of(item);
        match it.seg {
            Some(s) => &p.segs[s as usize].courtyards,
            None => &p.vias[it.via as usize].courtyards,
        }
    }

    /// Whether items `a` and `b` (of different connections) violate a rule.
    fn pair_conflicts(&self, rules: &dyn RuleTable, all: LayerMask, a: u32, b: u32) -> bool {
        let (pa, pb) = (self.piece_of(a), self.piece_of(b));
        let (sa, ka, la) = self.shape_of(a);
        let (sb, kb, lb) = self.shape_of(b);
        if ka == ItemKind::Via && kb == ItemKind::Via {
            let ia = self.items[a as usize];
            let ib = self.items[b as usize];
            let (va, vb) = (&pa.vias[ia.via as usize], &pb.vias[ib.via as usize]);
            // Two vias of one net at one point are one via: the result
            // keeps only one of them.
            if pa.net == pb.net && va.center == vb.center && va.drill == vb.drill {
                return false;
            }
            let ha = Shape::Circle(pa.vias[ia.via as usize].hole());
            let hb = Shape::Circle(pb.vias[ib.via as usize].hole());
            if gap_cmp(&ha, &hb, i64::from(rules.hole_to_hole())) == Ordering::Less {
                return true;
            }
        }
        if pa.net == pb.net {
            return false;
        }
        // A through hole spaces other nets' copper on every layer.
        let hc = i64::from(rules.hole_clearance());
        let hole_hit = |hole: Option<Shape>, other: &Shape| {
            hole.is_some_and(|h| gap_cmp(&h, other, hc) == Ordering::Less)
        };
        if hole_hit(self.hole_of(a), &sb) || hole_hit(self.hole_of(b), &sa) {
            return true;
        }
        let subj_a = pa.subject(ka, self.courtyards_of(a));
        let subj_b = pb.subject(kb, self.courtyards_of(b));
        let near = |l: LayerId| {
            let c = rules.clearance(&subj_a, &subj_b, l);
            gap_cmp(&sa, &sb, i64::from(c)) == Ordering::Less
        };
        match (la, lb) {
            (Some(x), Some(y)) if x == y => near(x),
            (Some(_), Some(_)) => false,
            (Some(x), None) | (None, Some(x)) => near(x),
            (None, None) => all.iter().any(near),
        }
    }

    /// Conflicting item pairs `(a, b)`, `a < b`, where `a` belongs to a piece
    /// for which `is_new` holds (pairs among old pieces were checked when
    /// they were new). Sorted and unique.
    pub fn conflicts(
        &self,
        rules: &dyn RuleTable,
        all: LayerMask,
        is_new: &(dyn Fn(&PieceCopper) -> bool + Sync),
    ) -> Vec<(u32, u32)> {
        let found: Vec<Vec<(u32, u32)>> = (0..self.items.len() as u32)
            .into_par_iter()
            .filter(|&i| is_new(self.piece_of(i)))
            .map(|i| {
                let (shape, _, layer) = self.shape_of(i);
                let q = shape.bbox().inflate(self.reach);
                let mut cand: Vec<u32> = match layer {
                    Some(l) => self.layers[l.index()].query(&q),
                    None => {
                        let mut v: Vec<u32> =
                            self.layers.iter().flat_map(|ix| ix.query(&q)).collect();
                        v.extend(self.vias.query(&q));
                        v.sort_unstable();
                        v.dedup();
                        v
                    }
                };
                // Another connection; or, for hole-to-hole between vias,
                // another piece of the same one (pieces meet at pins).
                let me = self.piece_of(i);
                let is_via = |k: u32| self.items[k as usize].seg.is_none();
                cand.retain(|&j| {
                    let pj = self.piece_of(j);
                    j != i
                        && (pj.task != me.task || (pj.piece != me.piece && is_via(i) && is_via(j)))
                });
                cand.into_iter()
                    .filter(|&j| self.pair_conflicts(rules, all, i, j))
                    .map(|j| (i.min(j), i.max(j)))
                    .collect()
            })
            .collect();
        let mut out: Vec<(u32, u32)> = found.into_iter().flatten().collect();
        out.sort_unstable();
        out.dedup();
        out
    }

    /// The connections (tasks) of a pair.
    pub fn tasks_of(&self, pair: (u32, u32)) -> (u32, u32) {
        (self.piece_of(pair.0).task, self.piece_of(pair.1).task)
    }

    /// The (task, piece) of each side of a conflicting pair.
    pub fn pieces_of(&self, pair: (u32, u32)) -> ((u32, u8), (u32, u8)) {
        let (a, b) = (self.piece_of(pair.0), self.piece_of(pair.1));
        ((a.task, a.piece), (b.task, b.piece))
    }

    /// History cells (layer, cell x, cell y) where item `a`'s centre line is
    /// within conflict distance of item `b`.
    pub fn conflict_cells(
        &self,
        rules: &dyn RuleTable,
        routable: &[LayerId],
        a: u32,
        b: u32,
        out: &mut HashSet<(u8, i32, i32)>,
    ) {
        let (pa, pb) = (self.piece_of(a), self.piece_of(b));
        let (_, ka, _) = self.shape_of(a);
        let (sb, kb, _) = self.shape_of(b);
        let subj_a = pa.subject(ka, self.courtyards_of(a));
        let subj_b = pb.subject(kb, self.courtyards_of(b));
        let it = self.items[a as usize];
        match it.seg {
            Some(s) => {
                let s = &pa.segs[s as usize];
                let c = if pa.net == pb.net {
                    0
                } else {
                    rules.clearance(&subj_a, &subj_b, s.layer)
                };
                let hw = s.seg.width / 2 + s.seg.width % 2;
                let d = s.seg.b - s.seg.a;
                let len = (d.length2() as f64).sqrt();
                let n = (len / f64::from(LATTICE_PITCH_NM)).ceil().max(1.0) as i64;
                let mut hit = false;
                for k in 0..=n {
                    let p = Point::new(
                        (i64::from(s.seg.a.x) + d.x * k / n) as Coord,
                        (i64::from(s.seg.a.y) + d.y * k / n) as Coord,
                    );
                    let dot = Shape::Circle(Circle::new(p, hw));
                    if gap_cmp(&sb, &dot, i64::from(c).max(1)) == Ordering::Less {
                        out.insert((s.layer.0, cell(p.x), cell(p.y)));
                        hit = true;
                    }
                }
                if !hit {
                    // Hole-to-hole between vias, or a sliver below sampling:
                    // mark the segment's ends.
                    out.insert((s.layer.0, cell(s.seg.a.x), cell(s.seg.a.y)));
                }
            }
            None => {
                let v = &pa.vias[it.via as usize];
                for l in routable {
                    out.insert((l.0, cell(v.center.x), cell(v.center.y)));
                }
            }
        }
    }

    /// Whether a track dot of `width` on `layer` at `p` keeps clearance from
    /// every routed item of another net (pins avoid routed copper).
    pub fn dot_is_clear(
        &self,
        rules: &dyn RuleTable,
        net: crate::api::NetId,
        layer: LayerId,
        p: Point,
        width: Coord,
        courtyards: &[crate::api::FootprintId],
    ) -> bool {
        let dot = Shape::Circle(Circle::new(p, width / 2 + width % 2));
        let q = dot.bbox().inflate(self.reach);
        let Some(ix) = self.layers.get(layer.index()) else {
            return true;
        };
        let subject =
            crate::rules::RuleSubject::new(ItemKind::Track, Some(net)).with_courtyards(courtyards);
        let hc = i64::from(rules.hole_clearance());
        ix.query(&q).into_iter().all(|j| {
            let pj = self.piece_of(j);
            if pj.net == net {
                return true;
            }
            let (sj, kj, _) = self.shape_of(j);
            let c = rules.clearance(&pj.subject(kj, self.courtyards_of(j)), &subject, layer);
            gap_cmp(&sj, &dot, i64::from(c)) != Ordering::Less
                && self
                    .hole_of(j)
                    .is_none_or(|h| gap_cmp(&h, &dot, hc) != Ordering::Less)
        })
    }
}

fn cell(v: Coord) -> i32 {
    v.div_euclid(LATTICE_PITCH_NM)
}

/// Side of one history chunk, in cells.
const CHUNK: i32 = 64;

/// PathFinder history per (layer, 50 µm cell), sparse in 64 × 64 chunks.
/// Values only grow.
#[derive(Debug, Clone, Default)]
pub(crate) struct HistoryMap {
    chunks: HashMap<(u8, i32, i32), Box<[u16]>>,
}

impl HistoryMap {
    pub fn get(&self, layer: u8, cx: i32, cy: i32) -> u16 {
        let key = (layer, cx.div_euclid(CHUNK), cy.div_euclid(CHUNK));
        self.chunks.get(&key).map_or(0, |c| {
            c[(cy.rem_euclid(CHUNK) * CHUNK + cx.rem_euclid(CHUNK)) as usize]
        })
    }

    pub fn bump(&mut self, layer: u8, cx: i32, cy: i32, by: u16) {
        let key = (layer, cx.div_euclid(CHUNK), cy.div_euclid(CHUNK));
        let c = self
            .chunks
            .entry(key)
            .or_insert_with(|| vec![0u16; (CHUNK * CHUNK) as usize].into_boxed_slice());
        let v = &mut c[(cy.rem_euclid(CHUNK) * CHUNK + cx.rem_euclid(CHUNK)) as usize];
        *v = v.saturating_add(by);
    }

    /// Copy the history under `window` into `plane` (each node takes its
    /// 50 µm cell's value; cells without history are left as they are).
    /// One chunk lookup per (row, chunk column), not per node.
    pub fn fill(&self, layer: LayerId, window: &Window, plane: &mut Plane<u16>) {
        if self.chunks.is_empty() {
            return;
        }
        let wb: BBox = window.bbox();
        let (cx0, cx1) = (cell(wb.min.x), cell(wb.max.x));
        let (cy0, cy1) = (cell(wb.min.y), cell(wb.max.y));
        let any = (cy0.div_euclid(CHUNK)..=cy1.div_euclid(CHUNK)).any(|y| {
            (cx0.div_euclid(CHUNK)..=cx1.div_euclid(CHUNK))
                .any(|x| self.chunks.contains_key(&(layer.0, x, y)))
        });
        if !any {
            return;
        }
        let cols = window.cols as usize;
        let cells: Vec<i32> = (0..window.cols)
            .map(|c| cell(window.node(c, 0).x))
            .collect();
        // Runs of columns within one chunk column: (first, end, chunk x).
        let mut runs: Vec<(usize, usize, i32)> = Vec::new();
        for (c, &cx) in cells.iter().enumerate() {
            let kx = cx.div_euclid(CHUNK);
            match runs.last_mut() {
                Some(r) if r.2 == kx => r.1 = c + 1,
                _ => runs.push((c, c + 1, kx)),
            }
        }
        let stride = plane.stride() as usize;
        let data = plane.as_mut_slice();
        for row in 0..window.rows {
            let cy = cell(window.node(0, row).y);
            let (ky, iy) = (cy.div_euclid(CHUNK), cy.rem_euclid(CHUNK));
            let base = (iy * CHUNK) as usize;
            let line = &mut data[row as usize * stride..row as usize * stride + cols];
            for &(c0, c1, kx) in &runs {
                let Some(chunk) = self.chunks.get(&(layer.0, kx, ky)) else {
                    continue;
                };
                for c in c0..c1 {
                    let h = chunk[base + cells[c].rem_euclid(CHUNK) as usize];
                    if h > 0 {
                        line[c] = h;
                    }
                }
            }
        }
    }

    /// (cells with history, total history).
    pub fn stats(&self) -> (u64, u64) {
        let mut n = 0;
        let mut sum = 0;
        for c in self.chunks.values() {
            for &v in c.iter() {
                if v > 0 {
                    n += 1;
                    sum += u64::from(v);
                }
            }
        }
        (n, sum)
    }
}
