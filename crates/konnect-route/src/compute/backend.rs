//! The compute traits a CPU or GPU backend implements, and the CPU
//! reference.
//!
//! [`SearchBackend`] is the narrow seam detailed search (WS-D1) and the GPU
//! spike (WS-G) meet at; [`RouteCompute`] adds the raster maintenance
//! negotiation (WS-D2) drives. A GPU implementation may override any subset
//! of methods and delegate the rest to [`CpuCompute`]; on device loss it
//! must latch to the CPU for the rest of the run. Whatever a backend
//! returns is a *proposal*: the caller re-checks it exactly and commits in
//! deterministic order.

use crate::compute::batch::Batch;
use crate::compute::cost::{Cost, CostParams};
use crate::compute::job::{JobNode, JobPool};
use crate::compute::raster::{TileRaster, TileStack, Window};
use konnect_geom::{gap_cmp, Circle, Coord, Point, Shape};
use std::cmp::Ordering;

/// How a painted shape counts.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PaintKind {
    /// Fixed copper, keepouts, other-net pads: CORE and HALO counters.
    Hard,
    /// Routed copper of another net: the `present` counter (negotiable).
    Soft,
}

/// One obstacle as the profile sees it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PaintShape {
    pub shape: Shape,
    /// Required edge-to-edge clearance between this obstacle and the
    /// profile's copper, nm (already resolved through the rule table).
    pub clearance: Coord,
    pub kind: PaintKind,
}

/// The copper a profile draws, which every paint is relative to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ProfileGeometry {
    /// Track width of the profile, nm.
    pub track_width: Coord,
    /// Extra inflation for arc approximation error, nm.
    pub sagitta: Coord,
}

/// A lattice path a backend proposes for one job.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PathProposal {
    /// Index into `JobPool::jobs`.
    pub job: u32,
    /// Lattice nodes from a source to a target, inclusive; a layer change
    /// is two consecutive nodes with the same `(col, row)`.
    pub nodes: Vec<JobNode>,
    pub cost: Cost,
    /// A* (or sweep) expansions spent, for telemetry.
    pub expansions: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SearchOutcome {
    Found(PathProposal),
    /// The search space was exhausted: no path exists under the masks.
    Unreachable {
        job: u32,
        expansions: u64,
    },
    /// The job's `cost_bound` or an expansion budget was hit first.
    BudgetExhausted {
        job: u32,
        expansions: u64,
    },
}

/// Cost-to-reach from a source set over one window (a min-plus sweep's
/// output, or Dijkstra's distance map). Row-major per layer, unpadded.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CostField {
    pub window: Window,
    pub layer_count: u8,
    pub costs: Vec<Cost>,
}

/// Search over rasters. Implementations must be deterministic: the same
/// inputs give the same outcomes in the same order.
pub trait SearchBackend {
    /// Run every job of `batch` against the frozen `stacks` (indexed by the
    /// job's `window`). One outcome per job, in the batch's job order.
    fn search_batch(
        &mut self,
        batch: &Batch,
        pool: &JobPool,
        stacks: &[TileStack],
        params: &CostParams,
    ) -> Vec<SearchOutcome>;

    /// Cost-to-reach every node of `stack` from `sources`.
    fn cost_field(
        &mut self,
        stack: &TileStack,
        sources: &[JobNode],
        params: &CostParams,
    ) -> CostField;
}

/// Raster maintenance on top of search.
pub trait RouteCompute: SearchBackend {
    /// Count `shapes` into the raster (CORE/HALO for hard, `present` for
    /// soft), relative to the profile's copper.
    fn paint(&mut self, raster: &mut TileRaster, shapes: &[PaintShape], profile: &ProfileGeometry);

    /// Exactly undo a previous [`RouteCompute::paint`] of the same shapes.
    fn unpaint(
        &mut self,
        raster: &mut TileRaster,
        shapes: &[PaintShape],
        profile: &ProfileGeometry,
    );

    /// Recompute `heading_mask` and `via_site` from the counters.
    fn derive_masks(&mut self, raster: &mut TileRaster);

    /// One PathFinder history update: every overused node (`present > 1`)
    /// gains `history_increment`.
    fn update_costs(&mut self, raster: &mut TileRaster, params: &CostParams);

    /// An independent copy for one worker thread, when separate copies can
    /// search concurrently (a CPU backend can; a device-bound one should
    /// return `None`, the default). With copies, negotiation routes whole
    /// tiles in parallel, each on its own copy, instead of in lockstep
    /// batch rounds; a copy must return exactly what this backend would.
    fn fork(&self) -> Option<Box<dyn RouteCompute + Send>> {
        None
    }
}

/// The CPU reference backend: simple, exact, and the oracle a GPU backend
/// is compared against.
#[derive(Debug, Clone, Default)]
pub struct CpuCompute;

impl CpuCompute {
    pub fn new() -> Self {
        CpuCompute
    }

    /// Count (`up`) or uncount one shape into `raster`, relative to the
    /// profile's copper: CORE and HALO for [`PaintKind::Hard`], `present`
    /// for [`PaintKind::Soft`]. [`RouteCompute::paint`] and
    /// [`RouteCompute::unpaint`] are this, once per shape; it borrows the
    /// shape so callers that keep their own obstacle lists need not clone.
    pub fn paint_shape(
        raster: &mut TileRaster,
        shape: &Shape,
        clearance: Coord,
        kind: PaintKind,
        profile: &ProfileGeometry,
        up: bool,
    ) {
        let window = raster.window;
        let TileRaster {
            core,
            halo,
            present,
            ..
        } = raster;
        Self::for_each_affected(
            &window,
            shape,
            clearance,
            profile,
            |col, row, in_core| match kind {
                PaintKind::Hard => {
                    bump(halo.get_mut(col, row), up);
                    if in_core {
                        bump(core.get_mut(col, row), up);
                    }
                }
                PaintKind::Soft => {
                    if in_core {
                        bump16(present.get_mut(col, row), up);
                    }
                }
            },
        );
    }

    /// Visit every node within HALO of a shape, with its CORE verdict.
    ///
    /// The exact predicate decides every node; only the scan is pruned. For
    /// segments and discs each row is limited to the interval the inflated
    /// shape can reach (computed in `f64`, then widened by a node on each
    /// side), so a long diagonal track costs its length rather than its
    /// bounding square.
    fn for_each_affected(
        w: &Window,
        shape: &Shape,
        clearance: Coord,
        profile: &ProfileGeometry,
        mut visit: impl FnMut(u32, u32, bool),
    ) {
        let half_width = half_up(profile.track_width);
        // A node is within reach of its outgoing edges' far halves within
        // one half-diagonal of a cell.
        let half_diag = ((f64::from(w.pitch_nm) * std::f64::consts::FRAC_1_SQRT_2).ceil()) as Coord;
        let core_clearance = i64::from(clearance) + i64::from(profile.sagitta);
        let halo_clearance = core_clearance + i64::from(half_diag);
        let reach = clearance
            .saturating_add(profile.sagitta)
            .saturating_add(half_diag)
            .saturating_add(half_width)
            .saturating_add(w.pitch_nm);
        let region = shape.bbox().inflate(reach);
        let span = w.bbox();
        if !region.intersects(&span) {
            return;
        }
        let pitch = i64::from(w.pitch_nm);
        let first = |lo: Coord, o: Coord| ((i64::from(lo) - i64::from(o)).max(0) / pitch) as u32;
        let last = |hi: Coord, o: Coord, n: u32| {
            (((i64::from(hi) - i64::from(o)) / pitch).max(0) as u32).min(n - 1)
        };
        let (c_lo, c_hi) = (
            first(region.min.x, w.origin.x),
            last(region.max.x, w.origin.x, w.cols),
        );
        if let Shape::Polygon(poly) = shape {
            let rows = (
                first(region.min.y, w.origin.y),
                last(region.max.y, w.origin.y, w.rows),
            );
            let thresholds = PolygonScan {
                core_clearance,
                halo_clearance,
                half_width,
                reach,
            };
            // A threshold of zero or less makes a point inside the region
            // no closer than it: the full scan below decides that case.
            if core_clearance + i64::from(half_width) > 0 {
                thresholds.scan(w, poly, rows, (c_lo, c_hi), &mut visit);
                return;
            }
        }
        let ox = f64::from(w.origin.x);
        let pf = f64::from(w.pitch_nm);
        // Radius around the core (centre line or centre) that every visited
        // node lies within, given the shape's own sweep.
        let around = |sweep: Coord| f64::from(sweep) + f64::from(reach);
        for row in first(region.min.y, w.origin.y)..=last(region.max.y, w.origin.y, w.rows) {
            let y = f64::from(w.origin.y) + pf * f64::from(row);
            let (lo, hi) = match shape {
                Shape::Segment(s) => match segment_row_interval(s, around(half_up(s.width)), y) {
                    Some(iv) => iv,
                    None => continue,
                },
                Shape::Circle(c) => {
                    let r = around(c.radius);
                    let dy = y - f64::from(c.center.y);
                    if dy.abs() > r {
                        continue;
                    }
                    let dx = (r * r - dy * dy).max(0.0).sqrt();
                    (f64::from(c.center.x) - dx, f64::from(c.center.x) + dx)
                }
                _ => (f64::from(region.min.x), f64::from(region.max.x)),
            };
            let c0 = ((lo - ox) / pf).floor() - 1.0;
            let c1 = ((hi - ox) / pf).ceil() + 1.0;
            let c0 = if c0 <= f64::from(c_lo) {
                c_lo
            } else {
                c0 as u32
            };
            if c1 < f64::from(c0) {
                continue;
            }
            let c1 = if c1 >= f64::from(c_hi) {
                c_hi
            } else {
                c1 as u32
            };
            for col in c0..=c1 {
                let track = Shape::Circle(Circle::new(w.node(col, row), half_width));
                let in_core = gap_cmp(shape, &track, core_clearance) == Ordering::Less;
                let in_halo = in_core || gap_cmp(shape, &track, halo_clearance) == Ordering::Less;
                if in_halo {
                    visit(col, row, in_core);
                }
            }
        }
    }
}

fn half_up(v: Coord) -> Coord {
    v / 2 + v % 2
}

/// The scan of a polygon (a zone fill, a keepout) for
/// [`CpuCompute::for_each_affected`].
///
/// A node is in CORE (HALO) when its track disc is closer to the region
/// than the core (halo) clearance: when the node lies in the region, or
/// within `clearance + half_width` of one of its edges. Deciding that with
/// the polygon predicate walks every edge for every node of the bounding
/// box; a refilled pour has thousands of edges and covers the board, and
/// that product was nearly all of a tile's build time. This scan visits the
/// same nodes with the same verdicts, row by row:
///
/// - each edge is tested only against the nodes of the rows and columns
///   its inflated stadium can reach (the interval is computed in `f64` and
///   widened by a node on each side, as for a track), in `f64` where the
///   distance is at least a nanometre off both thresholds and with the
///   exact predicate otherwise;
/// - whether a node lies in the region is the parity of the edges that
///   cross its row to its right, from crossings computed in `f64`. That is
///   only wrong for a node within rounding error of an edge, and such a
///   node is within the (positive) threshold of that edge, so the edge test
///   has already decided it.
struct PolygonScan {
    core_clearance: i64,
    halo_clearance: i64,
    half_width: Coord,
    /// Every node within HALO of an edge is within this of it.
    reach: Coord,
}

/// The distance between a node and an edge is computed in `f64` from
/// coordinate differences, exact to far below this; within it of a
/// threshold the exact predicate decides.
const SCAN_EXACT_BAND_NM: f64 = 1.0;

/// A row's verdict per column: bit 0 HALO, bit 1 CORE.
const SCAN_HALO: u8 = 1;
const SCAN_CORE: u8 = 2;

impl PolygonScan {
    fn scan(
        &self,
        w: &Window,
        poly: &konnect_geom::Polygon,
        (r_lo, r_hi): (u32, u32),
        (c_lo, c_hi): (u32, u32),
        visit: &mut impl FnMut(u32, u32, bool),
    ) -> u64 {
        // Edge-to-node tests made, for the tests.
        let mut tests = 0u64;
        if r_lo > r_hi || c_lo > c_hi {
            return tests;
        }
        let pitch = i64::from(w.pitch_nm);
        let (ox, oy) = (i64::from(w.origin.x), i64::from(w.origin.y));
        let pf = pitch as f64;
        let reach = i64::from(self.reach);
        let edges: Vec<(Point, Point)> = poly
            .rings()
            .flat_map(|ring| {
                let n = ring.len();
                (0..n).map(move |i| (ring[i], ring[(i + 1) % n]))
            })
            .collect();
        // Per row of the scan: the edges whose stadium can reach it.
        let n_rows = (r_hi - r_lo + 1) as usize;
        let mut by_row: Vec<Vec<u32>> = vec![Vec::new(); n_rows];
        for (i, (a, b)) in edges.iter().enumerate() {
            let lo = i64::from(a.y.min(b.y)) - reach - oy;
            let hi = i64::from(a.y.max(b.y)) + reach - oy;
            if hi < 0 {
                continue;
            }
            // One row more on each side than the stadium reaches.
            let first = (lo.max(0) / pitch - 1).max(i64::from(r_lo));
            let last = (hi / pitch + 1).min(i64::from(r_hi));
            for row in first..=last {
                by_row[(row - i64::from(r_lo)) as usize].push(i as u32);
            }
        }
        let core_r = (self.core_clearance + i64::from(self.half_width)) as f64;
        let halo_r = (self.halo_clearance + i64::from(self.half_width)) as f64;
        let n_cols = (c_hi - c_lo + 1) as usize;
        let mut verdict = vec![0u8; n_cols];
        let mut crossings: Vec<f64> = Vec::new();
        for (k, row_edges) in by_row.iter().enumerate() {
            let row = r_lo + k as u32;
            let y = oy + pitch * i64::from(row);
            let yf = y as f64;
            verdict.fill(0);
            crossings.clear();
            for &e in row_edges {
                let (a, b) = edges[e as usize];
                let (ay, by) = (i64::from(a.y), i64::from(b.y));
                if (ay > y) != (by > y) {
                    let (ax, bx) = (f64::from(a.x), f64::from(b.x));
                    crossings.push(ax + (yf - ay as f64) * (bx - ax) / (by - ay) as f64);
                }
                let seg = konnect_geom::Segment::new(a, b, 0);
                let Some((lo, hi)) = segment_row_interval(&seg, self.reach as f64, yf) else {
                    continue;
                };
                let c0 = (((lo - ox as f64) / pf).floor() - 1.0).max(f64::from(c_lo));
                let c1 = (((hi - ox as f64) / pf).ceil() + 1.0).min(f64::from(c_hi));
                if c1 < c0 {
                    continue;
                }
                let edge = Shape::Segment(seg);
                for col in c0 as u32..=c1 as u32 {
                    let v = &mut verdict[(col - c_lo) as usize];
                    if *v == SCAN_HALO | SCAN_CORE {
                        continue;
                    }
                    tests += 1;
                    let x = ox + pitch * i64::from(col);
                    let d = point_edge_distance(x, y, a, b);
                    let closer = |threshold: f64, clearance: i64| {
                        if d < threshold - SCAN_EXACT_BAND_NM {
                            true
                        } else if d > threshold + SCAN_EXACT_BAND_NM {
                            false
                        } else {
                            let node = Point::new(x as Coord, y as Coord);
                            let track = Shape::Circle(Circle::new(node, self.half_width));
                            gap_cmp(&edge, &track, clearance) == Ordering::Less
                        }
                    };
                    if closer(core_r, self.core_clearance) {
                        *v = SCAN_HALO | SCAN_CORE;
                    } else if *v == 0 && closer(halo_r, self.halo_clearance) {
                        *v = SCAN_HALO;
                    }
                }
            }
            crossings.sort_unstable_by(f64::total_cmp);
            // Crossings at or left of the node; the rest are to its right.
            let mut passed = 0;
            for col in c_lo..=c_hi {
                let x = (ox + pitch * i64::from(col)) as f64;
                while passed < crossings.len() && crossings[passed] <= x {
                    passed += 1;
                }
                let inside = (crossings.len() - passed) % 2 == 1;
                let v = verdict[(col - c_lo) as usize];
                if inside || v & SCAN_HALO != 0 {
                    visit(col, row, inside || v & SCAN_CORE != 0);
                }
            }
        }
        tests
    }
}

/// The distance from the node at (`x`, `y`) to the edge `a`–`b`, in `f64`
/// from coordinate differences (exact in `f64`), so its error is far below
/// a nanometre for any board.
fn point_edge_distance(x: i64, y: i64, a: Point, b: Point) -> f64 {
    let (px, py) = ((x - i64::from(a.x)) as f64, (y - i64::from(a.y)) as f64);
    let (ux, uy) = (
        (i64::from(b.x) - i64::from(a.x)) as f64,
        (i64::from(b.y) - i64::from(a.y)) as f64,
    );
    let l2 = ux * ux + uy * uy;
    let along = if l2 > 0.0 {
        ((px * ux + py * uy) / l2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (px - along * ux).hypot(py - along * uy)
}

/// The x-interval where the horizontal line at `y` meets the stadium of
/// radius `r` around the segment's centre line, if it does. Approximate
/// (`f64`): callers widen it and decide every node exactly.
fn segment_row_interval(s: &konnect_geom::Segment, r: f64, y: f64) -> Option<(f64, f64)> {
    let (ax, ay) = (f64::from(s.a.x), f64::from(s.a.y));
    let (bx, by) = (f64::from(s.b.x), f64::from(s.b.y));
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for (px, py) in [(ax, ay), (bx, by)] {
        let dy = y - py;
        if dy.abs() <= r {
            let dx = (r * r - dy * dy).max(0.0).sqrt();
            lo = lo.min(px - dx);
            hi = hi.max(px + dx);
        }
    }
    let (ux, uy) = (bx - ax, by - ay);
    let l2 = ux * ux + uy * uy;
    if l2 > 0.0 {
        let l = l2.sqrt();
        let ry = y - ay;
        // The projection onto the centre line falls within the segment.
        let along = if ux != 0.0 {
            let x0 = ax - ry * uy / ux;
            let x1 = ax + (l2 - ry * uy) / ux;
            Some((x0.min(x1), x0.max(x1)))
        } else if (0.0..=l2).contains(&(ry * uy)) {
            Some((f64::NEG_INFINITY, f64::INFINITY))
        } else {
            None
        };
        // The perpendicular distance is within r.
        let across = if uy != 0.0 {
            let x0 = ax + (ux * ry - r * l) / uy;
            let x1 = ax + (ux * ry + r * l) / uy;
            Some((x0.min(x1), x0.max(x1)))
        } else if (ux * ry).abs() <= r * l {
            Some((f64::NEG_INFINITY, f64::INFINITY))
        } else {
            None
        };
        if let (Some(p), Some(q)) = (along, across) {
            let (a, b) = (p.0.max(q.0), p.1.min(q.1));
            if a <= b {
                lo = lo.min(a);
                hi = hi.max(b);
            }
        }
    }
    (lo <= hi).then_some((lo, hi))
}

/// Saturating, sticky counter step: 255 stays 255 in both directions.
fn bump(c: &mut u8, up: bool) {
    if *c == u8::MAX {
        return;
    }
    if up {
        *c += 1;
    } else {
        debug_assert!(*c > 0, "unpaint of a shape that was never painted");
        *c = c.saturating_sub(1);
    }
}

fn bump16(c: &mut u16, up: bool) {
    if *c == u16::MAX {
        return;
    }
    if up {
        *c += 1;
    } else {
        debug_assert!(*c > 0, "unpaint of a shape that was never painted");
        *c = c.saturating_sub(1);
    }
}

impl CpuCompute {
    fn paint_impl(
        raster: &mut TileRaster,
        shapes: &[PaintShape],
        profile: &ProfileGeometry,
        up: bool,
    ) {
        for paint in shapes {
            Self::paint_shape(
                raster,
                &paint.shape,
                paint.clearance,
                paint.kind,
                profile,
                up,
            );
        }
    }
}

impl SearchBackend for CpuCompute {
    fn search_batch(
        &mut self,
        batch: &Batch,
        pool: &JobPool,
        stacks: &[TileStack],
        params: &CostParams,
    ) -> Vec<SearchOutcome> {
        crate::detail::search::search_batch_default(batch, pool, stacks, params)
    }

    fn cost_field(
        &mut self,
        stack: &TileStack,
        sources: &[JobNode],
        params: &CostParams,
    ) -> CostField {
        crate::detail::search::cost_field_on(stack, None, sources, params)
    }
}

impl RouteCompute for CpuCompute {
    fn paint(&mut self, raster: &mut TileRaster, shapes: &[PaintShape], profile: &ProfileGeometry) {
        Self::paint_impl(raster, shapes, profile, true);
    }

    fn unpaint(
        &mut self,
        raster: &mut TileRaster,
        shapes: &[PaintShape],
        profile: &ProfileGeometry,
    ) {
        Self::paint_impl(raster, shapes, profile, false);
    }

    fn derive_masks(&mut self, raster: &mut TileRaster) {
        crate::detail::search::derive_masks_uniform(raster, None);
    }

    fn update_costs(&mut self, raster: &mut TileRaster, params: &CostParams) {
        for row in 0..raster.window.rows {
            for col in 0..raster.window.cols {
                if raster.present.get(col, row) > 1 {
                    let h = raster.history.get_mut(col, row);
                    *h = h.saturating_add(params.history_increment);
                }
            }
        }
    }

    fn fork(&self) -> Option<Box<dyn RouteCompute + Send>> {
        Some(Box::new(CpuCompute))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::api::{LayerId, ProfileId, TileId};
    use crate::compute::raster::Occupancy;
    use konnect_geom::{Polygon, Segment};

    fn raster() -> TileRaster {
        let w = Window::new(Point::new(0, 0), 50_000, 41, 41).unwrap();
        TileRaster::new(TileId(0), LayerId(0), ProfileId(0), w)
    }

    fn track_obstacle() -> PaintShape {
        // A 0.1 mm track along y = 1 mm, needing 0.1 mm clearance.
        PaintShape {
            shape: Shape::Segment(Segment::new(
                Point::new(0, 1_000_000),
                Point::new(2_000_000, 1_000_000),
                100_000,
            )),
            clearance: 100_000,
            kind: PaintKind::Hard,
        }
    }

    const PROFILE: ProfileGeometry = ProfileGeometry {
        track_width: 100_000,
        sagitta: 0,
    };

    #[test]
    fn paint_marks_core_inside_halo_and_unpaint_restores() {
        let mut r = raster();
        let mut cpu = CpuCompute::new();
        let before = r.clone();
        cpu.paint(&mut r, &[track_obstacle()], &PROFILE);
        // Centre lines closer than 0.05 + 0.1 + 0.05 = 0.2 mm are blocked:
        // rows 17..=23 (y = 0.85..1.15 mm) at 50 µm pitch; 0.2 mm exactly
        // (rows 16 and 24) is legal but within the half-diagonal halo
        // (0.2 + 0.0354 mm), and 0.25 mm (row 15) is free.
        assert_eq!(r.occupancy(20, 20), Occupancy::Blocked);
        assert_eq!(r.occupancy(20, 17), Occupancy::Blocked);
        assert_eq!(r.occupancy(20, 23), Occupancy::Blocked);
        assert_eq!(r.occupancy(20, 16), Occupancy::Near);
        assert_eq!(r.occupancy(20, 24), Occupancy::Near);
        assert_eq!(r.occupancy(20, 15), Occupancy::Free);
        assert_eq!(r.occupancy(20, 5), Occupancy::Free);
        cpu.unpaint(&mut r, &[track_obstacle()], &PROFILE);
        assert_eq!(r, before);
    }

    /// The row-interval pruning must visit exactly the nodes a full scan of
    /// the bounding box finds.
    #[test]
    fn pruned_scan_matches_a_full_scan() {
        let w = Window::new(Point::new(-1_000_000, -700_000), 50_000, 97, 83).unwrap();
        let profile = ProfileGeometry {
            track_width: 127_000,
            sagitta: 3_000,
        };
        let shapes = [
            Shape::Segment(Segment::new(
                Point::new(-200_000, -300_000),
                Point::new(2_900_000, 2_800_000),
                101_000,
            )),
            Shape::Segment(Segment::new(
                Point::new(3_100_000, 100_000),
                Point::new(-900_000, 3_300_000),
                150_000,
            )),
            Shape::Segment(Segment::new(
                Point::new(500_000, 1_000_000),
                Point::new(500_000, 1_000_000),
                90_000,
            )),
            Shape::Segment(Segment::new(
                Point::new(0, 1_525_000),
                Point::new(3_000_000, 1_525_000),
                0,
            )),
            Shape::Segment(Segment::new(
                Point::new(1_234_567, -650_000),
                Point::new(1_234_567, 2_000_000),
                75_001,
            )),
            Shape::Circle(Circle::new(Point::new(1_710_000, 830_000), 301_000)),
        ];
        for shape in &shapes {
            for clearance in [0, 100_000, 151_000] {
                let mut fast = Vec::new();
                CpuCompute::for_each_affected(&w, shape, clearance, &profile, |c, r, core| {
                    fast.push((c, r, core))
                });
                let hw = half_up(profile.track_width);
                let half_diag = 35_356;
                let core_c = i64::from(clearance) + i64::from(profile.sagitta);
                let mut full = Vec::new();
                for r in 0..w.rows {
                    for c in 0..w.cols {
                        let t = Shape::Circle(Circle::new(w.node(c, r), hw));
                        let core = gap_cmp(shape, &t, core_c) == Ordering::Less;
                        let halo = gap_cmp(shape, &t, core_c + half_diag) == Ordering::Less;
                        if core || halo {
                            full.push((c, r, core));
                        }
                    }
                }
                assert!(!full.is_empty());
                assert_eq!(fast, full, "{shape:?} at clearance {clearance}");
            }
        }
    }

    #[test]
    fn soft_paint_counts_present_and_history_follows_overuse() {
        let mut r = raster();
        let mut cpu = CpuCompute::new();
        let soft = PaintShape {
            kind: PaintKind::Soft,
            ..track_obstacle()
        };
        cpu.paint(&mut r, &[soft.clone(), soft], &PROFILE);
        assert_eq!(r.present.get(20, 20), 2);
        assert_eq!(r.occupancy(20, 20), Occupancy::Free);
        cpu.update_costs(&mut r, &CostParams::default());
        assert_eq!(r.history.get(20, 20), 1);
        assert_eq!(r.history.get(20, 5), 0);
    }

    /// Every node of the window, decided by the polygon predicate alone:
    /// what the scan of a polygon must reproduce.
    fn full_scan(
        w: &Window,
        shape: &Shape,
        clearance: Coord,
        profile: &ProfileGeometry,
    ) -> Vec<(u32, u32, bool)> {
        let hw = half_up(profile.track_width);
        let half_diag = ((f64::from(w.pitch_nm) * std::f64::consts::FRAC_1_SQRT_2).ceil()) as i64;
        let core_c = i64::from(clearance) + i64::from(profile.sagitta);
        let mut full = Vec::new();
        for r in 0..w.rows {
            for c in 0..w.cols {
                let t = Shape::Circle(Circle::new(w.node(c, r), hw));
                let core = gap_cmp(shape, &t, core_c) == Ordering::Less;
                let halo = core || gap_cmp(shape, &t, core_c + half_diag) == Ordering::Less;
                if halo {
                    full.push((c, r, core));
                }
            }
        }
        full
    }

    fn scanned(
        w: &Window,
        shape: &Shape,
        clearance: Coord,
        profile: &ProfileGeometry,
    ) -> Vec<(u32, u32, bool)> {
        let mut fast = Vec::new();
        CpuCompute::for_each_affected(w, shape, clearance, profile, |c, r, core| {
            fast.push((c, r, core))
        });
        fast
    }

    fn polygon(outline: &[(i32, i32)], holes: &[&[(i32, i32)]]) -> Shape {
        let ring = |r: &[(i32, i32)]| r.iter().map(|&(x, y)| Point::new(x, y)).collect();
        Shape::Polygon(
            Polygon::new(ring(outline), holes.iter().map(|h| ring(h)).collect()).unwrap(),
        )
    }

    /// The polygon scan must visit exactly the nodes, with the verdicts, a
    /// full scan with the polygon predicate finds: edges along node rows
    /// and columns, vertices on nodes, thresholds that land on nodes, holes,
    /// notches narrower than a cell, regions that leave the window.
    #[test]
    fn a_polygon_scan_matches_the_polygon_predicate_on_every_node() {
        let w = Window::new(Point::new(-1_000_000, -700_000), 50_000, 97, 83).unwrap();
        let shapes = [
            // Edges on node rows and columns, corners on nodes.
            polygon(
                &[
                    (0, 0),
                    (2_000_000, 0),
                    (2_000_000, 1_500_000),
                    (0, 1_500_000),
                ],
                &[],
            ),
            // Off the lattice, with a hole and a hole's notch one cell wide.
            polygon(
                &[
                    (-733_001, -412_345),
                    (2_611_117, -380_003),
                    (2_900_001, 2_444_449),
                    (-512_777, 2_700_001),
                ],
                &[&[
                    (400_000, 300_000),
                    (1_600_000, 300_000),
                    (1_600_000, 1_250_000),
                    (1_030_000, 1_250_000),
                    (1_030_000, 700_000),
                    (990_000, 700_000),
                    (990_000, 1_250_000),
                    (400_000, 1_250_000),
                ]],
            ),
            // A comb: teeth and gaps of a few cells, diagonal ends.
            polygon(
                &[
                    (-900_000, -600_000),
                    (3_500_000, -600_000),
                    (3_500_000, 3_200_000),
                    (3_100_000, 3_200_000),
                    (2_900_000, -100_000),
                    (2_500_000, -100_000),
                    (2_300_000, 3_200_000),
                    (1_500_000, 3_200_000),
                    (1_450_000, 0),
                    (1_325_000, 0),
                    (1_300_000, 3_200_000),
                    (-900_000, 3_200_000),
                ],
                &[],
            ),
            // Smaller than a cell, between nodes.
            polygon(
                &[(510_000, 510_000), (540_000, 512_000), (522_000, 541_000)],
                &[],
            ),
            // A sliver through the window, nearly along a row.
            polygon(
                &[
                    (-5_000_000, 1_000_000),
                    (9_000_000, 1_049_999),
                    (9_000_000, 1_050_001),
                    (-5_000_000, 1_000_002),
                ],
                &[],
            ),
            // The window lies inside the region, far from every edge.
            polygon(
                &[
                    (-50_000_000, -50_000_000),
                    (50_000_000, -50_000_000),
                    (50_000_000, 50_000_000),
                    (-50_000_000, 50_000_000),
                ],
                &[],
            ),
        ];
        let profiles = [
            ProfileGeometry {
                track_width: 127_000,
                sagitta: 3_000,
            },
            // 0.1 mm clearance and 0.1 mm width put the core threshold
            // (0.15 mm) exactly on nodes of the first shape.
            PROFILE,
            // No width: the fallback to the full scan at clearance 0.
            ProfileGeometry {
                track_width: 0,
                sagitta: 0,
            },
        ];
        for shape in &shapes {
            for profile in &profiles {
                for clearance in [0, 1, 100_000, 151_000, 200_000] {
                    assert_eq!(
                        scanned(&w, shape, clearance, profile),
                        full_scan(&w, shape, clearance, profile),
                        "{shape:?} at clearance {clearance}, {profile:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn a_polygon_scan_matches_the_predicate_on_random_regions_with_holes() {
        // A small generator: the same regions every run.
        let mut state = 0x2545_f491_4f6c_dd1du64;
        let mut next = move |below: u32| {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state >> 33) as u32 % below
        };
        // A star around `centre`: angles in order, radii at random.
        let mut star = |centre: (i32, i32), lo: u32, hi: u32, points: u32| -> Vec<(i32, i32)> {
            (0..points)
                .map(|i| {
                    let angle = (f64::from(i) + f64::from(next(900)) / 1000.0)
                        * std::f64::consts::TAU
                        / f64::from(points);
                    let radius = f64::from(lo + next(hi - lo));
                    (
                        centre.0 + (radius * angle.cos()) as i32,
                        centre.1 + (radius * angle.sin()) as i32,
                    )
                })
                .collect()
        };
        let w = Window::new(Point::new(-300_000, -200_000), 50_000, 61, 47).unwrap();
        let profile = ProfileGeometry {
            track_width: 151_000,
            sagitta: 2_000,
        };
        let mut nodes = 0;
        for case in 0..60 {
            let outline = star((1_200_000, 900_000), 600_000, 1_900_000, 5 + case % 23);
            let hole = star((1_200_000, 900_000), 100_000, 500_000, 3 + case % 7);
            let shape = polygon(&outline, &[&hole]);
            for clearance in [0, 77_777, 200_000] {
                let full = full_scan(&w, &shape, clearance, &profile);
                nodes += full.len();
                assert_eq!(
                    scanned(&w, &shape, clearance, &profile),
                    full,
                    "case {case} at clearance {clearance}: {shape:?}"
                );
            }
        }
        assert!(nodes > 100_000, "the regions cover the window: {nodes}");
    }

    #[test]
    fn a_polygon_scan_tests_an_edge_only_against_the_nodes_near_it() {
        // A 16 mm square in a 20 mm window at 0.1 mm: 40 401 nodes, most of
        // them inside the square and far from its four edges.
        let w = Window::new(Point::new(0, 0), 100_000, 201, 201).unwrap();
        let square = Polygon::new(
            [(2, 2), (18, 2), (18, 18), (2, 18)]
                .iter()
                .map(|&(x, y)| Point::new(x * 1_000_000, y * 1_000_000))
                .collect(),
            Vec::new(),
        )
        .unwrap();
        let scan = PolygonScan {
            core_clearance: 200_000,
            halo_clearance: 270_711,
            half_width: 75_000,
            reach: 445_711,
        };
        let mut visited = 0u64;
        let tests = scan.scan(&w, &square, (0, 200), (0, 200), &mut |_, _, _| visited += 1);
        // Everything within 0.345 mm of the square is visited.
        assert!(visited > 160 * 160, "{visited}");
        // Each edge meets about ten rows or columns of 170 nodes; walking
        // every edge for every node would be four tests a node.
        assert!(tests < 4 * 12 * 180, "{tests} edge tests");
        assert!(tests < w.node_count() as u64 / 4, "{tests} edge tests");
    }
}
