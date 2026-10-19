//! Edge and via-site capacities from fixed obstacles.
//!
//! See the module docs of [`super`] for the model. Everything here runs
//! once per grid; the per-line kernel is in [`super::lines`].

use super::grid::Topology;
use super::lines::{inside_spans, round_out, LineSet, Obstacle};
use super::{GCellGrid, GlobalSettings};
use crate::api::{ItemKind, LayerId};
use crate::board::{PadKind, RouteBoard};
use crate::rules::{RuleSubject, RuleTable};
use konnect_geom::{ArcPolicy, Point, Shape};
use rayon::prelude::*;

/// How an obstacle is inflated.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Kind {
    /// Copper: clearance.
    Copper,
    /// A drilled hole on a layer without copper around it: hole clearance.
    Hole,
    /// The board edge: edge clearance.
    Edge,
    /// A keepout: no clearance beyond the keepout itself.
    Keepout,
}

/// One layer's rule values, nm.
#[derive(Debug, Clone, Copy)]
pub(crate) struct SlotRules {
    pub width: i64,
    pub clearance: i64,
}

impl SlotRules {
    /// Default track pitch, nm.
    pub(crate) fn pitch(&self) -> i64 {
        (self.width + self.clearance).max(1)
    }
}

pub(crate) fn slot_rules(rules: &dyn RuleTable, layer: LayerId) -> SlotRules {
    let any = RuleSubject::new(ItemKind::Track, None);
    SlotRules {
        width: i64::from(rules.track_width(&any, layer).preferred),
        clearance: i64::from(rules.clearance(&any, &any, layer)),
    }
}

/// Obstacles of one layer slot: shapes and how each is inflated.
struct SlotObstacles {
    track: Vec<(Shape, Kind)>,
    via: Vec<(Shape, Kind)>,
}

fn collect(
    board: &RouteBoard,
    layer: LayerId,
    settings: &GlobalSettings,
    policy: &ArcPolicy,
) -> SlotObstacles {
    let mut track = Vec::new();
    let mut via = Vec::new();
    for pad in &board.pads {
        let on = pad.layers.contains(layer);
        if on {
            for s in &pad.shapes {
                track.push((s.clone(), Kind::Copper));
                via.push((s.clone(), Kind::Copper));
            }
        }
        if let Some(d) = &pad.drill {
            if !on || pad.kind == PadKind::NpThroughHole || pad.shapes.is_empty() {
                track.push((d.shape.clone(), Kind::Hole));
                via.push((d.shape.clone(), Kind::Hole));
            }
        }
    }
    for t in board.tracks.iter().filter(|t| t.layer == layer) {
        track.push((Shape::Segment(t.seg), Kind::Copper));
        via.push((Shape::Segment(t.seg), Kind::Copper));
    }
    for a in board.arcs.iter().filter(|a| a.layer == layer) {
        for s in a.segments(policy) {
            track.push((Shape::Segment(s), Kind::Copper));
            via.push((Shape::Segment(s), Kind::Copper));
        }
    }
    for v in &board.vias {
        if v.layers.contains(layer) {
            track.push((v.shape(), Kind::Copper));
            via.push((v.shape(), Kind::Copper));
        } else {
            track.push((v.hole(), Kind::Hole));
            via.push((v.hole(), Kind::Hole));
        }
    }
    if settings.zones_block {
        for z in board.zones.iter().filter(|z| z.layer == layer) {
            track.push((Shape::Polygon(z.region.clone()), Kind::Copper));
            via.push((Shape::Polygon(z.region.clone()), Kind::Copper));
        }
    }
    for k in board.keepouts.iter().filter(|k| k.layers.contains(layer)) {
        if k.rules.tracks {
            track.push((Shape::Polygon(k.region.clone()), Kind::Keepout));
        }
        if k.rules.vias || k.rules.tracks {
            via.push((Shape::Polygon(k.region.clone()), Kind::Keepout));
        }
    }
    for e in &board.outline.edges {
        track.push((Shape::Segment(*e), Kind::Edge));
        via.push((Shape::Segment(*e), Kind::Edge));
    }
    SlotObstacles { track, via }
}

struct Inflation {
    copper: f64,
    hole: f64,
    edge: f64,
    keepout: f64,
}

impl Inflation {
    fn of(&self, k: Kind) -> f64 {
        match k {
            Kind::Copper => self.copper,
            Kind::Hole => self.hole,
            Kind::Edge => self.edge,
            Kind::Keepout => self.keepout,
        }
    }
}

/// Lines across the grid (vertical unless `swap`), blocked by `obs`, and
/// outside the board outline when it closes.
fn build_lines(
    board: &RouteBoard,
    grid: &GCellGrid,
    step: i64,
    swap: bool,
    obs: &[(Shape, Kind)],
    inflate: &Inflation,
) -> LineSet {
    let (origin, extent, other0, other1) = if swap {
        (
            i64::from(grid.bbox.min.y),
            i64::from(grid.bbox.max.y),
            i64::from(grid.bbox.min.x),
            i64::from(grid.bbox.max.x),
        )
    } else {
        (
            i64::from(grid.bbox.min.x),
            i64::from(grid.bbox.max.x),
            i64::from(grid.bbox.min.y),
            i64::from(grid.bbox.max.y),
        )
    };
    let count = ((extent - origin) / step + 1).max(1) as u32;
    let mut raw: Vec<Vec<(i32, i32)>> = vec![Vec::new(); count as usize];
    let mut cut = Vec::new();
    for (shape, kind) in obs {
        let o = Obstacle::from_shape(shape, inflate.of(*kind));
        let b = o.bounds();
        let (lo, hi) = if swap { (b.1, b.3) } else { (b.0, b.2) };
        let first = ((lo - origin as f64) / step as f64).ceil().max(0.0) as i64;
        let last = ((hi - origin as f64) / step as f64)
            .floor()
            .min(f64::from(count) - 1.0) as i64;
        for j in first..=last {
            cut.clear();
            let x = (origin + j * step) as f64;
            o.cut(x, swap, &mut cut);
            for &iv in &cut {
                raw[j as usize].push(round_out(iv));
            }
        }
    }
    // Everything outside a closed outline is blocked.
    if !board.outline.edges.is_empty() {
        let segs: Vec<(Point, Point)> = board.outline.edges.iter().map(|e| (e.a, e.b)).collect();
        let (lo_ext, hi_ext) = (other0 - 1, other1 + 1);
        for (j, line) in raw.iter_mut().enumerate() {
            let x = (origin + j as i64 * step) as f64;
            let Some(spans) = inside_spans(&segs, x, swap) else {
                continue;
            };
            let mut cur = lo_ext as f64;
            for (a, b) in spans {
                if a > cur {
                    line.push(round_out((cur, a)));
                }
                cur = cur.max(b);
            }
            if cur < hi_ext as f64 {
                line.push(round_out((cur, hi_ext as f64)));
            }
        }
    }
    LineSet::from_raw(origin, step, raw)
}

/// Edges whose tracks cross the same strip: horizontal edges between
/// cells whose centres sit at `s0` and `s1` (x), or vertical ones (y).
struct Strip {
    horiz: bool,
    s0: i64,
    s1: i64,
    /// `(edge, lo, hi)`: each edge's boundary span along the strip, sorted.
    edges: Spans,
}

/// `(edge, lo, hi)` spans of one strip.
type Spans = Vec<(u32, i64, i64)>;

fn strip_groups(t: &Topology) -> Vec<Strip> {
    let mut map: std::collections::BTreeMap<(bool, i64, i64), Spans> = Default::default();
    for e in 0..t.edges {
        let (a, b) = (t.edge_a[e as usize] as usize, t.edge_b[e as usize] as usize);
        let (ca, cb) = (t.center(a as u32), t.center(b as u32));
        let horiz = t.edge_horiz[e as usize];
        let (s0, s1, lo, hi) = if horiz {
            let lo = i64::from(t.y0[a].max(t.y0[b]));
            let hi = (i64::from(t.y0[a]) + i64::from(t.size[a]))
                .min(i64::from(t.y0[b]) + i64::from(t.size[b]));
            (i64::from(ca.0), i64::from(cb.0), lo, hi)
        } else {
            let lo = i64::from(t.x0[a].max(t.x0[b]));
            let hi = (i64::from(t.x0[a]) + i64::from(t.size[a]))
                .min(i64::from(t.x0[b]) + i64::from(t.size[b]));
            (i64::from(ca.1), i64::from(cb.1), lo, hi)
        };
        map.entry((horiz, s0.min(s1), s0.max(s1)))
            .or_default()
            .push((e, lo, hi));
    }
    map.into_iter()
        .map(|((horiz, s0, s1), mut edges)| {
            edges.sort_by_key(|&(e, lo, _)| (lo, e));
            Strip {
                horiz,
                s0,
                s1,
                edges,
            }
        })
        .collect()
}

/// Capacity of every edge of one strip: the blocked stretches of every
/// sample line between the two centres are merged, tracks of pitch `p` are
/// packed from the start of each free stretch along the whole strip, and
/// each edge counts the track centres in its span. Packing the whole strip
/// at once keeps a gap that straddles two cells from losing a track to
/// rounding on both sides.
fn strip_capacity(lines: &LineSet, g: &Strip, ext: (i64, i64), p: i64) -> Vec<(u32, u16)> {
    let mut blocked: Vec<(i32, i32)> = Vec::new();
    for j in lines.range(g.s0, g.s1) {
        lines.collect(j, i32::MIN, i32::MAX, &mut blocked);
    }
    blocked.sort_unstable();
    let spans: Vec<(i64, i64)> = g.edges.iter().map(|&(_, lo, hi)| (lo, hi)).collect();
    let counts = pack(&blocked, ext, p, &spans);
    g.edges
        .iter()
        .zip(counts)
        .map(|(&(e, _, _), c)| (e, c))
        .collect()
}

/// Greedy track packing. `blocked`: closed intervals sorted by start (may
/// overlap); `ext`: the half-open extent; `spans`: sorted, disjoint
/// half-open spans to count in.
fn pack(blocked: &[(i32, i32)], ext: (i64, i64), p: i64, spans: &[(i64, i64)]) -> Vec<u16> {
    let mut out = vec![0u32; spans.len()];
    let mut free: Vec<(i64, i64)> = Vec::new();
    let mut cur = ext.0;
    for &(a, b) in blocked {
        let (a, b) = (i64::from(a), i64::from(b));
        if b < cur {
            continue;
        }
        if a > cur {
            free.push((cur, (a - 1).min(ext.1 - 1)));
        }
        cur = cur.max(b + 1);
        if cur >= ext.1 {
            break;
        }
    }
    if cur < ext.1 {
        free.push((cur, ext.1 - 1));
    }
    let mut j = 0;
    for (a, b) in free {
        if b < a {
            continue;
        }
        let k_max = (b - a) / p;
        while j < spans.len() && spans[j].1 <= a {
            j += 1;
        }
        let mut k = j;
        while k < spans.len() && spans[k].0 <= b {
            let (lo, hi) = spans[k];
            let k_lo = (lo - a).max(0).div_euclid(p) + i64::from((lo - a).max(0) % p != 0);
            let k_hi = ((hi - 1 - a).div_euclid(p)).min(k_max);
            if k_hi >= k_lo {
                out[k] += (k_hi - k_lo + 1) as u32;
            }
            k += 1;
        }
    }
    out.into_iter()
        .map(|c| c.min(u32::from(u16::MAX)) as u16)
        .collect()
}

/// Capacity per resource, in the layout documented on
/// [`GCellGrid::capacity`].
pub(crate) fn compute(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    grid: &GCellGrid,
    settings: &GlobalSettings,
) -> Vec<u16> {
    let topo = Topology::new(grid);
    let layers: Vec<LayerId> = grid.layers.iter().collect();
    let policy = ArcPolicy::default();
    let via_rule = rules.via(&RuleSubject::new(ItemKind::Via, None));
    let via_r = f64::from(via_rule.diameter) / 2.0;
    let step = (topo.fsize / 4).max(1);
    let edge_cl = f64::from(rules.edge_clearance());
    let hole_cl = f64::from(rules.hole_clearance());

    struct SlotLines {
        rules: SlotRules,
        vlines: LineSet,
        hlines: LineSet,
        via: LineSet,
    }
    let slots: Vec<SlotLines> = layers
        .par_iter()
        .map(|&layer| {
            let sr = slot_rules(rules, layer);
            let obs = collect(board, layer, settings, &policy);
            let half = sr.width as f64 / 2.0;
            let c = sr.clearance as f64;
            let track = Inflation {
                copper: c + half,
                hole: hole_cl + half,
                edge: edge_cl + half,
                keepout: half,
            };
            let via = Inflation {
                copper: c + via_r,
                hole: hole_cl.max(c) + via_r,
                edge: edge_cl + via_r,
                keepout: via_r,
            };
            let (vlines, (hlines, vias)) = rayon::join(
                || build_lines(board, grid, step, false, &obs.track, &track),
                || {
                    rayon::join(
                        || build_lines(board, grid, step, true, &obs.track, &track),
                        || build_lines(board, grid, step, false, &obs.via, &via),
                    )
                },
            );
            SlotLines {
                rules: sr,
                vlines,
                hlines,
                via: vias,
            }
        })
        .collect();

    let edges = topo.edges as usize;
    let mut cap = vec![0u16; topo.resources()];
    let (planar, via_caps) = cap.split_at_mut(slots.len() * edges);
    let groups = strip_groups(&topo);
    let extent = (
        (i64::from(grid.bbox.min.y), i64::from(grid.bbox.max.y)),
        (i64::from(grid.bbox.min.x), i64::from(grid.bbox.max.x)),
    );
    planar
        .par_chunks_mut(edges.max(1))
        .zip(slots.par_iter())
        .for_each(|(out, sl)| {
            let p = sl.rules.pitch();
            let counted: Vec<Vec<(u32, u16)>> = groups
                .par_iter()
                .map(|g| {
                    let (lines, ext) = if g.horiz {
                        (&sl.vlines, extent.0)
                    } else {
                        (&sl.hlines, extent.1)
                    };
                    strip_capacity(lines, g, ext, p)
                })
                .collect();
            for (e, c) in counted.into_iter().flatten() {
                out[e as usize] = c;
            }
        });

    let via_pitch =
        f64::from(via_rule.diameter) + slots.first().map_or(0.0, |s| s.rules.clearance as f64);
    let via_lines: Vec<&LineSet> = slots.iter().map(|s| &s.via).collect();
    via_caps
        .par_chunks_mut(4096)
        .enumerate()
        .for_each(|(chunk, out)| {
            for (i, slot) in out.iter_mut().enumerate() {
                let c = (chunk * 4096 + i) as u32;
                *slot = via_capacity(&topo, &via_lines, c, via_pitch);
            }
        });
    cap
}

fn via_capacity(topo: &Topology, lines: &[&LineSet], c: u32, via_pitch: f64) -> u16 {
    if !topo.alive[c as usize] {
        return 0;
    }
    let (x0, y0, s) = (
        i64::from(topo.x0[c as usize]),
        i64::from(topo.y0[c as usize]),
        i64::from(topo.size[c as usize]),
    );
    const N: i64 = 4;
    let mut free = 0u32;
    for i in 0..N {
        let x = x0 + (2 * i + 1) * s / (2 * N);
        for j in 0..N {
            let y = (y0 + (2 * j + 1) * s / (2 * N)) as i32;
            if lines.iter().all(|l| !l.blocked(l.nearest(x), y)) {
                free += 1;
            }
        }
    }
    if free == 0 {
        return 0;
    }
    let area = (s as f64) * (s as f64) * f64::from(free) / (N * N) as f64;
    let sites = (area / (via_pitch * via_pitch)).round();
    sites.clamp(1.0, f64::from(u16::MAX)) as u16
}

/// The capacity of every planar resource whose west/north cell lies in `w`.
#[cfg(test)]
pub(crate) fn window_total(grid: &GCellGrid, w: &super::grid::Win) -> u64 {
    let topo = Topology::new(grid);
    let mut sum = 0u64;
    for slot in 0..topo.slots {
        for e in 0..topo.edges {
            let a = topo.edge_a[e as usize];
            if topo.in_window(a, w) {
                sum += u64::from(grid.capacity[topo.planar_res(slot, e) as usize]);
            }
        }
    }
    sum
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn packing_counts_each_track_once_across_spans() {
        let mm = 1_000_000i64;
        let spans = [(0, mm), (mm, 2 * mm), (2 * mm, 3 * mm)];
        // Free everywhere at 0.25 mm: four per span.
        assert_eq!(pack(&[], (0, 3 * mm), 250_000, &spans), vec![4, 4, 4]);
        // A gap of 0.3 mm straddling the first boundary holds two tracks
        // at 0.25 mm, one on each side.
        let b = [(0, 849_999), (1_150_001, 3_000_000)];
        assert_eq!(pack(&b, (0, 3 * mm), 250_000, &spans), vec![1, 1, 0]);
        // Any interior gap holds one track.
        let b = [(0, 499_999), (500_001, 3_000_000)];
        assert_eq!(pack(&b, (0, 3 * mm), 250_000, &spans), vec![1, 0, 0]);
        // Overlapping, unmerged blocks.
        let b = [(-10, 2_000_000), (100, 200), (1_999_000, 3_000_000)];
        assert_eq!(pack(&b, (0, 3 * mm), 250_000, &spans), vec![0, 0, 0]);
    }
}
