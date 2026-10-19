//! The search backend the pipeline hands negotiation: WS-D1's lattice
//! kernel ([`crate::detail::search`]) behind the frozen
//! [`SearchBackend`]/[`RouteCompute`] seam WS-D2 drives.
//!
//! Two conventions differ between the two sides, and this adapter settles
//! both:
//!
//! - **Path nodes.** The kernel returns *corner* nodes (the source, every
//!   heading change, both ends of each via, the target); negotiation's
//!   exact check vetoes individual lattice steps, so it needs every node.
//!   Found paths are expanded back to unit steps on the stack's uniform
//!   window before they are returned.
//! - **Masks.** `derive_masks` is the kernel's own derivation
//!   ([`derive_masks_uniform`]): a heading leaves a node only toward an
//!   unblocked neighbour, and every unblocked node is a `VIA_CHECK` site.
//!   Negotiation then narrows via sites to where its profile's whole via
//!   fits (`TileRun::narrow_vias`), so a surviving site is free of HALO
//!   under the via either way. Negotiation derives the masks before it
//!   lifts a net's own copper, so [`KernelBackend::negotiation_config`]
//!   turns the heading mask off and the kernel reads the live CORE
//!   counters instead.
//!
//! Raster maintenance (`paint`, `unpaint`, `update_costs`) is the CPU
//! reference. Forks are cheap clones: the kernel keeps its scratch per
//! thread, so a fork shares nothing mutable.

use crate::api::{ItemKind, NetId};
use crate::board::RouteBoard;
use crate::compute::job::JOB_MIN_RUN;
use crate::compute::{
    Batch, CostField, CostParams, CpuCompute, JobNode, JobPool, PaintShape, PathProposal,
    ProfileGeometry, RouteCompute, SearchBackend, SearchOutcome, TileRaster, TileStack, Window,
};
use crate::detail::search::{
    cost_field_on, derive_masks_uniform, heading_of, search_batch_with, BatchContext, SearchConfig,
};
use crate::rules::{RuleSubject, RuleTable};
use konnect_geom::Coord;
use std::sync::Arc;

/// How far from a via's centre negotiation samples routed-copper
/// congestion: about a via's radius less a track's half width (the
/// reference search's default).
pub const VIA_CONGESTION_REACH_NM: konnect_geom::Coord = 250_000;

/// Expansions after which a negotiation search restarts with the relaxed
/// cost-to-go field ([`SearchConfig::field_after`]): searches that flood
/// their window around routed copper then expand little beyond their path,
/// at the same path cost.
pub const FIELD_AFTER: u64 = 2_000;

/// WS-D1's kernel as a [`RouteCompute`] backend for negotiation.
#[derive(Debug, Clone)]
pub struct KernelBackend {
    /// Kernel tuning (heuristic, bends, budgets, turn rule).
    pub config: SearchConfig,
    /// Ask for [`SearchConfig::min_run_nm`] between turns (`JOB_MIN_RUN`),
    /// so lattice jogs shorter than a segment-length rule are not proposed:
    /// on every job, or only on the nets [`Self::min_run_nets`] marks. Off
    /// by default: a minimum run can make tight pin escapes unreachable.
    pub min_run: bool,
    /// With [`Self::min_run`], the nets (by `NetId` index) whose jobs get
    /// the minimum run; `None` flags every job. See [`min_run_nets`].
    pub min_run_nets: Option<Arc<Vec<bool>>>,
}

impl KernelBackend {
    pub fn new(config: SearchConfig) -> Self {
        KernelBackend {
            config,
            min_run: false,
            min_run_nets: None,
        }
    }

    /// The kernel tuning negotiation needs: [`SearchConfig::default`]
    /// without the derived heading mask. Negotiation derives a stack's
    /// masks once, with every net's fixed copper painted, and lifts the
    /// routed net's own copper only per search; the mask would still wall
    /// the net in around its own pads. The kernel then reads the live CORE
    /// counters instead, which are exact for every net.
    pub fn negotiation_config() -> SearchConfig {
        SearchConfig {
            use_heading_mask: false,
            via_congestion_reach_nm: Some(VIA_CONGESTION_REACH_NM),
            field_after: FIELD_AFTER,
            ..SearchConfig::default()
        }
    }
}

impl Default for KernelBackend {
    fn default() -> Self {
        KernelBackend::new(KernelBackend::negotiation_config())
    }
}

impl SearchBackend for KernelBackend {
    fn search_batch(
        &mut self,
        batch: &Batch,
        pool: &JobPool,
        stacks: &[TileStack],
        params: &CostParams,
    ) -> Vec<SearchOutcome> {
        let ctx = BatchContext {
            stacks,
            lattices: None,
            config: &self.config,
            checker: None,
            lookahead: None,
            gcells: None,
        };
        let wants = |net: u32| match &self.min_run_nets {
            None => true,
            Some(nets) => nets.get(net as usize).copied().unwrap_or(false),
        };
        let flag = self.min_run && pool.jobs.iter().any(|j| wants(j.net));
        let outcomes = if flag {
            let mut flagged = pool.clone();
            for j in &mut flagged.jobs {
                if wants(j.net) {
                    j.flags |= JOB_MIN_RUN;
                }
            }
            search_batch_with(batch, &flagged, &ctx, params).outcomes
        } else {
            search_batch_with(batch, pool, &ctx, params).outcomes
        };
        outcomes
            .into_iter()
            .map(|o| match o {
                SearchOutcome::Found(p) => {
                    let window = pool
                        .jobs
                        .get(p.job as usize)
                        .and_then(|j| stacks.get(j.window as usize))
                        .map(|s| s.window);
                    SearchOutcome::Found(match window {
                        Some(w) => densify(p, &w),
                        None => p,
                    })
                }
                other => other,
            })
            .collect()
    }

    fn cost_field(
        &mut self,
        stack: &TileStack,
        sources: &[JobNode],
        params: &CostParams,
    ) -> CostField {
        cost_field_on(stack, None, sources, params)
    }
}

impl RouteCompute for KernelBackend {
    fn paint(&mut self, raster: &mut TileRaster, shapes: &[PaintShape], profile: &ProfileGeometry) {
        CpuCompute::new().paint(raster, shapes, profile);
    }

    fn unpaint(
        &mut self,
        raster: &mut TileRaster,
        shapes: &[PaintShape],
        profile: &ProfileGeometry,
    ) {
        CpuCompute::new().unpaint(raster, shapes, profile);
    }

    fn derive_masks(&mut self, raster: &mut TileRaster) {
        derive_masks_uniform(raster, None);
    }

    fn update_costs(&mut self, raster: &mut TileRaster, params: &CostParams) {
        CpuCompute::new().update_costs(raster, params);
    }

    fn fork(&self) -> Option<Box<dyn RouteCompute + Send>> {
        Some(Box::new(self.clone()))
    }
}

/// The nets with a `track_segment_length` minimum on any routable layer (by
/// `NetId` index), and the largest such minimum: what [`KernelBackend`]
/// needs to ask the kernel for minimum runs only where a rule wants them.
/// `None` when no net has one.
pub fn min_run_nets(board: &RouteBoard, rules: &dyn RuleTable) -> Option<(Vec<bool>, Coord)> {
    let mut nets = vec![false; board.nets.len()];
    let mut longest: Coord = 0;
    for (i, flag) in nets.iter_mut().enumerate() {
        let subject = RuleSubject::new(ItemKind::Track, Some(NetId(i as u32)));
        for layer in board.routable.iter() {
            if let Some(min) = rules.min_segment_length(&subject, layer) {
                if min > 0 {
                    *flag = true;
                    longest = longest.max(min);
                }
            }
        }
    }
    (longest > 0).then_some((nets, longest))
}

/// Expand a corner-node path to one node per lattice step. Consecutive
/// nodes on one layer are joined by an octilinear run; each inserted node
/// carries the run's heading. Via pairs and anything not octilinear (which
/// a uniform window never yields) are left as they are.
pub(crate) fn densify(mut p: PathProposal, window: &Window) -> PathProposal {
    let nodes = std::mem::take(&mut p.nodes);
    let mut out: Vec<JobNode> = Vec::with_capacity(nodes.len());
    for (i, n) in nodes.iter().enumerate() {
        if let Some(prev) = i.checked_sub(1).map(|k| nodes[k]) {
            if prev.layer == n.layer {
                let dc = i32::from(n.col) - i32::from(prev.col);
                let dr = i32::from(n.row) - i32::from(prev.row);
                let steps = dc.abs().max(dr.abs());
                let octilinear = dc == 0 || dr == 0 || dc.abs() == dr.abs();
                if steps > 1 && octilinear {
                    let (sc, sr) = (dc.signum(), dr.signum());
                    let mask = heading_of(sc, sr).map_or(prev.heading_mask, |h| 1u8 << h);
                    for k in 1..steps {
                        let col = i32::from(prev.col) + sc * k;
                        let row = i32::from(prev.row) + sr * k;
                        debug_assert!(
                            (col as u32) < window.cols && (row as u32) < window.rows,
                            "a run stays inside its window"
                        );
                        out.push(JobNode {
                            col: col as u16,
                            row: row as u16,
                            layer: n.layer,
                            heading_mask: mask,
                            reserved: 0,
                        });
                    }
                }
            }
        }
        out.push(*n);
    }
    p.nodes = out;
    p
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::compute::Cost;
    use konnect_geom::Point;

    fn node(col: u16, row: u16, layer: u8) -> JobNode {
        JobNode {
            col,
            row,
            layer,
            heading_mask: 0,
            reserved: 0,
        }
    }

    #[test]
    fn minimum_runs_are_asked_for_only_where_a_rule_sets_a_segment_length() {
        use crate::synthetic::{two_pin_nets, UniformRules};
        let board = two_pin_nets(3, false);
        assert!(min_run_nets(&board, &UniformRules::default()).is_none());
        let rules = UniformRules {
            min_segment_length: Some(100_000),
            ..UniformRules::default()
        };
        let (nets, longest) = min_run_nets(&board, &rules).expect("a rule applies");
        assert_eq!(longest, 100_000);
        assert_eq!(nets.len(), board.nets.len());
        assert!(nets.iter().all(|&n| n));
    }

    #[test]
    fn corner_paths_expand_to_unit_steps_and_keep_vias() {
        let w = Window::new(Point::new(0, 0), 50_000, 40, 40).unwrap();
        let p = PathProposal {
            job: 0,
            nodes: vec![
                node(2, 2, 0),
                node(6, 2, 0),
                node(9, 5, 0),
                node(9, 5, 1),
                node(9, 1, 1),
            ],
            cost: Cost(0),
            expansions: 0,
        };
        let d = densify(p, &w);
        let cells: Vec<(u16, u16, u8)> = d.nodes.iter().map(|n| (n.col, n.row, n.layer)).collect();
        let mut want = Vec::new();
        want.extend((2..=6).map(|c| (c, 2, 0)));
        want.extend((7..=9).map(|k| (k, 2 + (k - 6), 0)));
        want.push((9, 5, 1));
        want.extend((1..=4).rev().map(|r| (9, r, 1)));
        assert_eq!(cells, want);
        // Every consecutive pair is one lattice step or a via.
        for pair in d.nodes.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            let dc = (i32::from(a.col) - i32::from(b.col)).abs();
            let dr = (i32::from(a.row) - i32::from(b.row)).abs();
            assert!((a.layer != b.layer && dc == 0 && dr == 0) || dc.max(dr) == 1);
        }
    }
}
