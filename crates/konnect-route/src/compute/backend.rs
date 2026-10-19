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
}

/// The CPU reference backend: simple, exact, and the oracle a GPU backend
/// is compared against.
#[derive(Debug, Clone, Default)]
pub struct CpuCompute;

impl CpuCompute {
    pub fn new() -> Self {
        CpuCompute
    }

    /// Visit every node a shape can affect, with its CORE and HALO verdicts.
    fn for_each_affected(
        raster: &TileRaster,
        paint: &PaintShape,
        profile: &ProfileGeometry,
        mut visit: impl FnMut(u32, u32, bool, bool),
    ) {
        let w = &raster.window;
        let half_width = profile.track_width / 2 + profile.track_width % 2;
        // A node is within reach of its outgoing edges' far halves within
        // one half-diagonal of a cell.
        let half_diag = ((f64::from(w.pitch_nm) * std::f64::consts::FRAC_1_SQRT_2).ceil()) as Coord;
        let core_clearance = i64::from(paint.clearance) + i64::from(profile.sagitta);
        let halo_clearance = core_clearance + i64::from(half_diag);
        let reach = paint
            .clearance
            .saturating_add(profile.sagitta)
            .saturating_add(half_diag)
            .saturating_add(half_width)
            .saturating_add(w.pitch_nm);
        let region = paint.shape.bbox().inflate(reach);
        let span = w.bbox();
        if !region.intersects(&span) {
            return;
        }
        let pitch = i64::from(w.pitch_nm);
        let first = |lo: Coord, o: Coord| ((i64::from(lo) - i64::from(o)).max(0) / pitch) as u32;
        let last = |hi: Coord, o: Coord, n: u32| {
            (((i64::from(hi) - i64::from(o)) / pitch).max(0) as u32).min(n - 1)
        };
        for row in first(region.min.y, w.origin.y)..=last(region.max.y, w.origin.y, w.rows) {
            for col in first(region.min.x, w.origin.x)..=last(region.max.x, w.origin.x, w.cols) {
                let track = Shape::Circle(Circle::new(w.node(col, row), half_width));
                let in_core = gap_cmp(&paint.shape, &track, core_clearance) == Ordering::Less;
                let in_halo =
                    in_core || gap_cmp(&paint.shape, &track, halo_clearance) == Ordering::Less;
                if in_halo {
                    visit(col, row, in_core, in_halo);
                }
            }
        }
    }
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
            let mut hits: Vec<(u32, u32, bool)> = Vec::new();
            Self::for_each_affected(raster, paint, profile, |col, row, core, _| {
                hits.push((col, row, core))
            });
            for (col, row, core) in hits {
                match paint.kind {
                    PaintKind::Hard => {
                        bump(raster.halo.get_mut(col, row), up);
                        if core {
                            bump(raster.core.get_mut(col, row), up);
                        }
                    }
                    PaintKind::Soft => {
                        if core {
                            bump16(raster.present.get_mut(col, row), up);
                        }
                    }
                }
            }
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
