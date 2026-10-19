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
use konnect_geom::{gap_cmp, Circle, Coord, Shape};
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
        let _ = (batch, pool, stacks, params);
        todo!("WS-D1: run detail::search over each job of the batch")
    }

    fn cost_field(
        &mut self,
        stack: &TileStack,
        sources: &[JobNode],
        params: &CostParams,
    ) -> CostField {
        let _ = (stack, sources, params);
        todo!("WS-D1/WS-G: Dijkstra distance map, the oracle for the GPU sweep")
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
        let _ = raster;
        todo!("WS-D1: heading legality from neighbour occupancy; via sites from CORE")
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
    use konnect_geom::{Point, Segment};

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
}
