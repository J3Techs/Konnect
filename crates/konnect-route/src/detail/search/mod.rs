//! Single-connection lattice A* (owned by WS-D1).
//!
//! Multi-source, multi-target octilinear A* over one tile's rasters, with
//! heading and run-length state, a radix heap, and an octile-plus-via
//! heuristic (or a VPR-style map lookahead). Moves: straight, ±45°, ±90°,
//! and a via where every spanned layer's `via_site` allows it. `Near` nodes
//! get a cached exact check; `Blocked` nodes are walls; other nets' routed
//! copper costs `present`/`history` per [`crate::compute::CostParams`].
//!
//! # Entry points
//!
//! - [`search`]: the frozen single-job call. Uniform window, default
//!   [`SearchConfig`], no exact checker: `Near` edges are allowed at
//!   `near_step` extra cost and the caller re-checks the proposal.
//! - [`search_with`]: the full kernel, with a [`SearchContext`] that can add
//!   a non-uniform [`Lattice`], an [`ExactCheck`] for `Near` edges and vias,
//!   a [`MapLookahead`], a GCell corridor and tuning.
//! - [`search_batch_with`]: a batch in parallel (rayon), outcomes in batch
//!   order. `CpuCompute::search_batch` is this with the defaults.
//! - [`cost_field_on`]: Dijkstra's distance map (`CpuCompute::cost_field`).
//!
//! # Conventions
//!
//! - Headings are [`HEADINGS`]: 0 = +x, counter-clockwise in 45° steps in
//!   the Y-up sense. Board rows grow with Y-down `y`, so heading 2 moves to
//!   `row − 1` ([`heading_delta`]).
//! - Costs are [`Cost`] fixed point: 1000 per base-pitch straight step,
//!   1414 per diagonal one. On a non-uniform lattice every edge costs its
//!   length at that rate, rounded up. Congestion multiplies an edge's base
//!   cost PathFinder-style, `(1 + h·hf) · (1 + p·pf)` with the factors from
//!   [`CostParams`]; bends, wrong-way steps, `Near` edges, corridor exits
//!   and vias add.
//! - A found path ([`crate::compute::PathProposal::nodes`]) lists only
//!   **corner nodes**: the source, every node where the heading changes,
//!   both ends of each via (same `(col, row)`, consecutive), and the target.
//!   Consecutive nodes on one layer are joined by one straight octilinear
//!   run. Each node's `heading_mask` holds the bit of the heading the path
//!   leaves it in (0 on the last node and before a via).
//! - The rasters must not hold the job's own net as hard copper (the driver
//!   unpaints it), and `heading_mask`/`via_site` must have been derived
//!   (`CpuCompute::derive_masks`, [`derive_masks_on_lattice`]).
//! - Terminals are trusted access points: a source or target node may be
//!   entered or left even when its own raster cell is Blocked; such edges
//!   count as `Near`.

pub mod heap;
mod kernel;
pub mod lattice;
pub mod lookahead;

pub use lattice::{
    derive_masks_in_lattice, derive_masks_on_lattice, derive_masks_uniform, paint_on_lattice,
    Lattice, LatticeSpec, ViaGeometry, VIA_CHECK, VIA_FREE, VIA_NO,
};
pub use lookahead::MapLookahead;

use crate::compute::job::JOB_ALLOW_VIA;
use crate::compute::raster::TileStack;
use crate::compute::{
    Batch, Cost, CostField, CostParams, JobDescriptor, JobNode, JobPool, SearchOutcome,
};
use crate::global::GCellGrid;
use crate::telemetry::Telemetry;
use kernel::{Mode, Problem, Region, Scratch};
use konnect_geom::{Coord, Point};
use lattice::{Grid, LatticeGrid, UniformGrid};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::cell::RefCell;

/// Per-search counters for telemetry.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct SearchStats {
    pub expansions: u64,
    pub exact_checks: u64,
    pub exact_check_cache_hits: u64,
    /// Heap pushes (relaxations that improved a state).
    pub pushes: u64,
    /// Exact via checks (cache misses).
    pub via_checks: u64,
    /// Popped states whose incoming edge or via failed the exact check.
    pub lazy_rejects: u64,
    /// Searches retried over the whole window after the bounded region
    /// came back unreachable.
    pub region_retries: u64,
    /// Lattice nodes in the last region searched.
    pub window_nodes: u64,
}

impl SearchStats {
    pub fn add(&mut self, o: &SearchStats) {
        self.expansions += o.expansions;
        self.exact_checks += o.exact_checks;
        self.exact_check_cache_hits += o.exact_check_cache_hits;
        self.pushes += o.pushes;
        self.via_checks += o.via_checks;
        self.lazy_rejects += o.lazy_rejects;
        self.region_retries += o.region_retries;
        self.window_nodes = self.window_nodes.max(o.window_nodes);
    }

    /// Fold one connection's search into run telemetry.
    pub fn record(&self, outcome: &SearchOutcome, t: &mut Telemetry) {
        t.expansions_per_connection.record(self.expansions);
        t.window_nodes.record(self.window_nodes);
        t.count("search_exact_checks", self.exact_checks);
        t.count("search_exact_check_cache_hits", self.exact_check_cache_hits);
        t.count("search_via_checks", self.via_checks);
        t.count("search_lazy_rejects", self.lazy_rejects);
        t.count("search_pushes", self.pushes);
        t.count("search_region_retries", self.region_retries);
        let name = match outcome {
            SearchOutcome::Found(_) => "search_found",
            SearchOutcome::Unreachable { .. } => "search_unreachable",
            SearchOutcome::BudgetExhausted { .. } => "search_budget_exhausted",
        };
        t.count(name, 1);
    }
}

/// The eight lattice headings, 0 = +x, counter-clockwise in 45° steps in
/// the Y-up sense: `(dx, dy)` per heading.
pub const HEADINGS: [(i32, i32); 8] = [
    (1, 0),
    (1, 1),
    (0, 1),
    (-1, 1),
    (-1, 0),
    (-1, -1),
    (0, -1),
    (1, -1),
];

/// The "no heading" state: at a terminal, or just after a via.
pub const HEADING_NONE: u8 = 8;

/// `(dcol, drow)` of heading `h` on a lattice whose rows grow with board
/// (Y-down) `y`.
#[inline]
pub fn heading_delta(h: u8) -> (i32, i32) {
    let (dx, dy) = HEADINGS[usize::from(h & 7)];
    (dx, -dy)
}

/// The heading of a lattice step `(dcol, drow)` with components in −1..=1.
pub fn heading_of(dcol: i32, drow: i32) -> Option<u8> {
    (0..8u8).find(|&h| heading_delta(h) == (dcol, drow))
}

/// Whether turning from heading `a` to heading `b` is allowed: straight or a
/// 45°/90° turn. A 135° turn (and a reversal) leaves an interior angle
/// below 90° and is forbidden.
pub fn turn_allowed(a: u8, b: u8) -> bool {
    let d = (i32::from(b) - i32::from(a)).rem_euclid(8);
    matches!(d, 0 | 1 | 2 | 6 | 7)
}

/// For each incoming heading, the outgoing headings a `track_angle`
/// minimum allows. A turn of `k · 45°` leaves an interior angle of
/// `180° − k · 45°`. `None` means 90°, so 135° turns are always banned
/// (an 89° rule keeps 90° turns; a 91° rule allows only 45° turns).
pub fn turn_set(min_angle_deg: Option<f64>) -> [u8; 8] {
    let min = min_angle_deg.unwrap_or(90.0).max(90.0);
    let max_k = (0..=2u8)
        .rev()
        .find(|&k| 180.0 - 45.0 * f64::from(k) >= min - 1e-9)
        .unwrap_or(0);
    let mut out = [0u8; 8];
    for (h, o) in out.iter_mut().enumerate() {
        for k in 0..=max_k {
            *o |= 1 << ((h as u8 + k) & 7);
            *o |= 1 << ((h as u8 + 8 - k) & 7);
        }
    }
    out
}

/// A layer's preferred routing direction (a soft cost).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
pub enum Preferred {
    #[default]
    None,
    Horizontal,
    Vertical,
}

/// Which A* heuristic to use.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
pub enum Heuristic {
    /// Octile distance plus one via when the nearest target is on another
    /// layer. Admissible and consistent.
    #[default]
    Octile,
    /// [`MapLookahead`] when the context supplies a matching table; falls
    /// back to octile-plus-via outside its range.
    Map,
}

/// Search tuning that is not part of the frozen [`CostParams`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SearchConfig {
    pub heuristic: Heuristic,
    /// Weighted A*: the heuristic is multiplied by this / 1000 (VPR's
    /// `astar_fac`). 1000 is exact A*.
    pub astar_factor: u32,
    /// Expansion budget per search (both attempts together).
    pub max_expansions: u64,
    /// The search region is the terminals' (and corridor's) box grown by
    /// this margin plus `region_margin_permille` of its larger side.
    pub region_margin_nm: Coord,
    pub region_margin_permille: u32,
    /// Retry over the whole window when the bounded region is unreachable.
    pub retry_full_window: bool,
    /// The `track_angle` minimum; `None` is 90°.
    pub min_track_angle_deg: Option<f64>,
    /// Minimum straight run between turns for `JOB_MIN_RUN` jobs, nm.
    pub min_run_nm: Coord,
    pub bend_45: Cost,
    pub bend_90: Cost,
    /// Preferred direction per stack layer (missing entries: none).
    pub layer_pref: Vec<Preferred>,
    /// Extra cost per base step outside the job's GCell corridor.
    pub corridor_penalty: Cost,
    /// Honour the raster's `heading_mask` (the derived legality filter).
    pub use_heading_mask: bool,
    /// With an exact checker, check vias even where `via_site` says free.
    pub check_free_vias: bool,
}

impl Default for SearchConfig {
    fn default() -> Self {
        SearchConfig {
            heuristic: Heuristic::Octile,
            astar_factor: 1000,
            max_expansions: 4_000_000,
            region_margin_nm: 1_000_000,
            region_margin_permille: 500,
            retry_full_window: true,
            min_track_angle_deg: None,
            min_run_nm: 100_000,
            bend_45: Cost(50),
            bend_90: Cost(200),
            layer_pref: Vec::new(),
            corridor_penalty: Cost(500),
            use_heading_mask: true,
            check_free_vias: true,
        }
    }
}

/// The exact legality oracle the search asks about `Near` edges and vias.
/// Implementations resolve the job's net and profile themselves.
pub trait ExactCheck: Sync {
    /// A track of the job's profile from `a` to `b` on stack layer `layer`
    /// is clear of every hard obstacle.
    fn segment_ok(&self, job: &JobDescriptor, layer: u8, a: Point, b: Point) -> bool;
    /// A through via of the job's profile centred at `at` is legal.
    fn via_ok(&self, job: &JobDescriptor, at: Point) -> bool;
}

/// Everything a search reads besides the job.
#[derive(Clone, Copy)]
pub struct SearchContext<'a> {
    pub stack: &'a TileStack,
    /// `None`: the uniform lattice of `stack.window`.
    pub lattice: Option<&'a Lattice>,
    pub config: &'a SearchConfig,
    pub checker: Option<&'a dyn ExactCheck>,
    pub lookahead: Option<&'a MapLookahead>,
    /// The global grid the jobs' corridor ids refer to.
    pub gcells: Option<&'a GCellGrid>,
}

impl<'a> SearchContext<'a> {
    pub fn new(stack: &'a TileStack, config: &'a SearchConfig) -> Self {
        SearchContext {
            stack,
            lattice: None,
            config,
            checker: None,
            lookahead: None,
            gcells: None,
        }
    }
}

thread_local! {
    static SCRATCH: RefCell<Scratch> = RefCell::new(Scratch::default());
}

/// Bytes of search scratch the calling thread holds (it only grows; this
/// is the per-thread memory cost of the largest search so far).
pub fn scratch_bytes() -> usize {
    SCRATCH.with(|s| s.borrow().footprint())
}

fn default_config() -> &'static SearchConfig {
    static CONFIG: std::sync::OnceLock<SearchConfig> = std::sync::OnceLock::new();
    CONFIG.get_or_init(SearchConfig::default)
}

/// Route one job inside `stack`. `job` indexes `pool`.
pub fn search(
    stack: &TileStack,
    pool: &JobPool,
    job: &JobDescriptor,
    job_index: u32,
    params: &CostParams,
    stats: &mut SearchStats,
) -> SearchOutcome {
    let ctx = SearchContext::new(stack, default_config());
    search_with(&ctx, pool, job, job_index, params, stats)
}

/// Route one job with a full context.
pub fn search_with(
    ctx: &SearchContext<'_>,
    pool: &JobPool,
    job: &JobDescriptor,
    job_index: u32,
    params: &CostParams,
    stats: &mut SearchStats,
) -> SearchOutcome {
    let bad = |e| SearchOutcome::Unreachable {
        job: job_index,
        expansions: e,
    };
    let in_pool = |first: u32, count: u32, len: usize| {
        (first as usize)
            .checked_add(count as usize)
            .is_some_and(|end| end <= len)
    };
    if !in_pool(job.sources_first, job.sources_count, pool.nodes.len())
        || !in_pool(job.targets_first, job.targets_count, pool.nodes.len())
        || !in_pool(job.corridor_first, job.corridor_count, pool.corridor.len())
    {
        return bad(0);
    }
    let lookahead = ctx.lookahead.filter(|la| {
        ctx.config.heuristic == Heuristic::Map
            && la.matches(
                params,
                ctx.config,
                ctx.lattice
                    .map_or(ctx.stack.window.pitch_nm, Lattice::base_pitch),
                ctx.stack.layers.len(),
            )
    });
    let problem = Problem {
        stack: ctx.stack,
        job,
        job_index,
        sources: pool.sources(job),
        targets: pool.targets(job),
        corridor: pool.corridor_of(job),
        params,
        config: ctx.config,
        checker: ctx.checker,
        lookahead,
        gcells: ctx.gcells,
        mode: Mode::Route,
    };
    match ctx.lattice {
        Some(l) if !l.is_uniform() => run_bounded(&LatticeGrid { lattice: l }, &problem, stats),
        Some(l) => run_bounded(&UniformGrid::new(l.window()), &problem, stats),
        None => run_bounded(&UniformGrid::new(ctx.stack.window), &problem, stats),
    }
}

fn run_bounded<G: Grid>(grid: &G, p: &Problem<'_>, stats: &mut SearchStats) -> SearchOutcome {
    SCRATCH.with(|cell| {
        let mut s = cell.borrow_mut();
        let full = Region::full(grid);
        let region = kernel::job_region(grid, p);
        stats.window_nodes = region.nodes() as u64;
        let first = kernel::run(grid, p, region, &mut s, stats);
        match first {
            SearchOutcome::Unreachable { expansions, .. }
                if region != full && p.config.retry_full_window =>
            {
                stats.region_retries += 1;
                stats.window_nodes = full.nodes() as u64;
                let config = SearchConfig {
                    max_expansions: p.config.max_expansions.saturating_sub(expansions),
                    ..p.config.clone()
                };
                let p2 = Problem {
                    config: &config,
                    ..*p
                };
                let add = |e: u64| e + expansions;
                match kernel::run(grid, &p2, full, &mut s, stats) {
                    SearchOutcome::Found(mut pp) => {
                        pp.expansions = add(pp.expansions);
                        SearchOutcome::Found(pp)
                    }
                    SearchOutcome::Unreachable { job, expansions: e } => {
                        SearchOutcome::Unreachable {
                            job,
                            expansions: add(e),
                        }
                    }
                    SearchOutcome::BudgetExhausted { job, expansions: e } => {
                        SearchOutcome::BudgetExhausted {
                            job,
                            expansions: add(e),
                        }
                    }
                }
            }
            other => other,
        }
    })
}

/// One batch's outcomes (batch order) and per-job counters.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct BatchOutcome {
    pub outcomes: Vec<SearchOutcome>,
    pub stats: Vec<SearchStats>,
}

impl BatchOutcome {
    /// Fold every job's counters into run telemetry.
    pub fn record(&self, t: &mut Telemetry) {
        for (s, o) in self.stats.iter().zip(&self.outcomes) {
            s.record(o, t);
        }
    }
}

/// Shared inputs for [`search_batch_with`].
#[derive(Clone, Copy)]
pub struct BatchContext<'a> {
    /// Indexed by `JobDescriptor::window`.
    pub stacks: &'a [TileStack],
    /// `None`, or one lattice per stack (uniform or not).
    pub lattices: Option<&'a [Lattice]>,
    pub config: &'a SearchConfig,
    pub checker: Option<&'a dyn ExactCheck>,
    pub lookahead: Option<&'a MapLookahead>,
    pub gcells: Option<&'a GCellGrid>,
}

/// Run every job of `batch` in parallel; outcomes come back in batch order
/// and do not depend on the thread count.
pub fn search_batch_with(
    batch: &Batch,
    pool: &JobPool,
    ctx: &BatchContext<'_>,
    params: &CostParams,
) -> BatchOutcome {
    let results: Vec<(SearchOutcome, SearchStats)> = batch
        .jobs
        .par_iter()
        .map(|&j| {
            let mut stats = SearchStats::default();
            let Some(job) = pool.jobs.get(j as usize) else {
                let o = SearchOutcome::Unreachable {
                    job: j,
                    expansions: 0,
                };
                return (o, stats);
            };
            let w = job.window as usize;
            let Some(stack) = ctx.stacks.get(w) else {
                let o = SearchOutcome::Unreachable {
                    job: j,
                    expansions: 0,
                };
                return (o, stats);
            };
            let sc = SearchContext {
                stack,
                lattice: ctx.lattices.and_then(|l| l.get(w)),
                config: ctx.config,
                checker: ctx.checker,
                lookahead: ctx.lookahead,
                gcells: ctx.gcells,
            };
            let o = search_with(&sc, pool, job, j, params, &mut stats);
            (o, stats)
        })
        .collect();
    let (outcomes, stats) = results.into_iter().unzip();
    BatchOutcome { outcomes, stats }
}

/// The frozen batch call with default tuning: uniform windows, no checker.
pub fn search_batch_default(
    batch: &Batch,
    pool: &JobPool,
    stacks: &[TileStack],
    params: &CostParams,
) -> Vec<SearchOutcome> {
    let ctx = BatchContext {
        stacks,
        lattices: None,
        config: default_config(),
        checker: None,
        lookahead: None,
        gcells: None,
    };
    search_batch_with(batch, pool, &ctx, params).outcomes
}

/// Cost-to-reach every node of `stack` from `sources` (Dijkstra, every
/// layer, vias allowed, default tuning, no checker). Row-major per layer.
pub fn cost_field_on(
    stack: &TileStack,
    lattice: Option<&Lattice>,
    sources: &[JobNode],
    params: &CostParams,
) -> CostField {
    let window = lattice.map_or(stack.window, Lattice::window);
    let job = JobDescriptor {
        flags: JOB_ALLOW_VIA,
        cost_bound: u32::MAX,
        ..Default::default()
    };
    let config = SearchConfig {
        max_expansions: u64::MAX,
        ..default_config().clone()
    };
    let p = Problem {
        stack,
        job: &job,
        job_index: 0,
        sources,
        targets: &[],
        corridor: &[],
        params,
        config: &config,
        checker: None,
        lookahead: None,
        gcells: None,
        mode: Mode::Field,
    };
    let layers = stack.layers.len() as u32;
    let mut costs = Vec::new();
    let dims_ok = !sources.is_empty()
        && (1..=32).contains(&stack.layers.len())
        && stack
            .layers
            .iter()
            .all(|r| r.window.cols == window.cols && r.window.rows == window.rows);
    if dims_ok {
        SCRATCH.with(|cell| {
            let mut s = cell.borrow_mut();
            let mut stats = SearchStats::default();
            match lattice {
                Some(l) if !l.is_uniform() => {
                    let g = LatticeGrid { lattice: l };
                    let region = Region::full(&g);
                    kernel::run(&g, &p, region, &mut s, &mut stats);
                    kernel::field_costs(&s, region, layers, 1, &mut costs);
                }
                _ => {
                    let g = UniformGrid::new(window);
                    let region = Region::full(&g);
                    kernel::run(&g, &p, region, &mut s, &mut stats);
                    kernel::field_costs(&s, region, layers, 1, &mut costs);
                }
            }
        });
    }
    if costs.is_empty() {
        costs = vec![Cost::INFINITE; window.node_count() * stack.layers.len()];
    }
    CostField {
        window,
        layer_count: stack.layers.len() as u8,
        costs,
    }
}

/// A found path as geometry: one `(layer, a, b)` per straight run, plus one
/// `(layer_from, layer_to, at)` per via, in path order. `lattice` must be
/// the one the search ran on (`None`: the uniform lattice of `window`).
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct PathGeometry {
    pub runs: Vec<(u8, Point, Point)>,
    pub vias: Vec<(u8, u8, Point)>,
}

pub fn path_geometry(
    nodes: &[JobNode],
    lattice: Option<&Lattice>,
    window: &crate::compute::Window,
) -> PathGeometry {
    let at = |n: &JobNode| match lattice {
        Some(l) => l.node(n.col.into(), n.row.into()),
        None => window.node(n.col.into(), n.row.into()),
    };
    let mut g = PathGeometry::default();
    for w in nodes.windows(2) {
        let (a, b) = (at(&w[0]), at(&w[1]));
        if w[0].layer != w[1].layer {
            g.vias.push((w[0].layer, w[1].layer, a));
        } else if a != b {
            g.runs.push((w[0].layer, a, b));
        }
    }
    g
}

#[cfg(test)]
mod tests;
