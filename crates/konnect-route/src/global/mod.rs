//! S3 global routing (owned by WS-C).
//!
//! PathFinder (McMurchie & Ebeling) over a GCell grid: 1 mm cells on the
//! routable layers ([`crate::api::GCELL_PITCH_NM`]), refined to 0.5 mm in
//! dense fields. Edge capacity is the free track pitches left after fixed
//! copper and fanout. Connections are seeded with pattern routes (L, Z,
//! 3-bend), then negotiated with cost `b · (1 + h) · p` for a bounded number
//! of iterations in deterministic bbox-disjoint batches. The output is one
//! corridor and layer assignment per connection, plus the overflow map.
//!
//! # Grid
//!
//! [`build_grid`] lays `cols × rows` cells of [`GlobalSettings::gcell_pitch`]
//! over the board and splits every *dense* base cell (one whose 3 × 3
//! neighbourhood holds at least [`GlobalSettings::refine_min_pads`] pad
//! centres) into `k × k` fine cells, `k = gcell_pitch / fine_pitch`. Ids:
//!
//! - a base cell keeps the frozen id [`GCellGrid::id`]; a refined base
//!   cell's id is never used;
//! - fine cells follow every base id: see [`GCellGrid::fine_id`].
//!
//! A pitch too fine for the board is coarsened until no side exceeds 65,000
//! cells and the layer slots times the cells (fine cells included) stay
//! within 2^23 nodes; refinement that would break that budget is dropped.
//! [`route_global`] refuses a grid from elsewhere beyond those limits (every
//! connection fails with `NoCorridor`).
//!
//! [`GCellGrid::cell_bbox`] and [`GCellGrid::locate`] map ids to areas and
//! back, so S5 can read a corridor without knowing the layout.
//!
//! # Capacity
//!
//! For each layer the board is cut by sample lines a quarter of a fine cell
//! apart, or closer when an obstacle inflated by the rules (twice the
//! clearance plus a track) is narrower than that, so none falls between
//! two lines. Each obstacle (pads, fixed tracks and vias, drills,
//! keepouts, the board edge and everything outside the outline) is
//! inflated by the layer's default clearance plus half the default track
//! width and intersected with every line. Edges whose tracks cross the same strip
//! (between the same two centre lines) are counted together: the blocked
//! stretches of every line in the strip are merged, tracks at the default
//! track pitch are packed from the start of each free stretch, and each
//! edge counts the track centres on its boundary, so a gap that straddles
//! two cells keeps all its tracks. Via sites are counted from a 4 × 4
//! sample of each cell where the via (inflated by clearance plus its
//! radius) is free on every routable layer, in default via pitches (the
//! default via diameter plus the largest layer clearance). Zone fills do
//! not block unless [`GlobalSettings::zones_block`] is set, because they are
//! refilled around new copper.
//!
//! # Routing
//!
//! 1. **Seed** (iteration 0): each connection takes the cheapest of its L,
//!    Z, U-detour and 3-bend patterns, with a dynamic program choosing each
//!    leg's layer and the vias between legs. The PathFinder present factor
//!    is [`PresentSchedule::first_iter_pres_fac`]; the seed adds its own
//!    demand-aware term ([`GlobalCosts::seed_congestion`], CUGR style) so
//!    early seeds steer later ones around full edges. A connection with no
//!    feasible pattern falls back to the maze search.
//! 2. **Negotiate** (iterations 1..): every connection touching an overused
//!    edge or via site is ripped up and re-routed by A* bounded to its
//!    window, with a precomputed lookahead. Cost of a resource is
//!    `b · (1 + h) · p`: `b` the base length cost (higher against the
//!    layer's preferred direction), `h` the history, `p = 1 + pres_fac ·
//!    overuse`. `pres_fac` starts at [`PresentSchedule::initial_pres_fac`]
//!    and grows by [`PresentSchedule::pres_fac_mult`] up to
//!    [`PresentSchedule::max_pres_fac`]; history grows by
//!    [`PresentSchedule::acc_fac`] times the overuse after every iteration.
//!    Windows of connections that stay overflowed grow each iteration (up
//!    to [`GlobalSettings::window_growth_cap`]). Between full re-routes
//!    ([`GlobalSettings::full_reroute_every`]) only the stretch of a path
//!    from its first to its last overused resource is re-routed, between
//!    the kept clean ends, in a window around that stretch (incremental
//!    rip-up as in VPR and nextpnr router2): far cheaper for long
//!    connections that leave a congested field. Negotiation stops when no
//!    resource is overused, after [`GlobalSettings::max_iterations`], or
//!    after [`GlobalSettings::stall_iterations`] without progress. The
//!    result is the best iteration ([`GlobalStats::best_iteration`]), not
//!    necessarily the last.
//! 3. **Schedule**: each iteration's jobs go through
//!    [`crate::compute::BatchBuilder`]. A batch's windows are disjoint, its
//!    rip-ups run first, its searches run in parallel against that frozen
//!    state and its results commit in order, so the result does not depend
//!    on the thread count. A connection whose window holds no path is routed
//!    afterwards over the whole grid, serially in order.
//!
//! Connections of one net share resources: demand counts once per net, and
//! a resource the net already holds costs [`GlobalCosts::same_net`] of its
//! base cost, which grows Steiner-like trees. A net's demand on an edge is
//! its track width plus clearance in default track pitches, rounded up; on
//! a via site, the square of its via diameter plus clearance over the
//! default via's, rounded (at least 1 each).
//!
//! A layer change is one through via: it joins any two slots the connection
//! may use for one via cost and one via-site demand, crossing the slots
//! between, so a class confined to F.Cu and B.Cu routes like any other.
//! The lookahead and the seed's dynamic program charge it the same way.
//!
//! A cell edge or via site with no capacity is a wall, except at a
//! connection's own *access* cells (its terminal cells and their
//! neighbours): a 1 mm grid cannot resolve pin access (S2 and S4 decide it).
//! An edge with either end in an access cell, and the via site of an access
//! cell, stay passable, so the exemption reaches two cells out from a
//! terminal. Such access resources cost twice their base cost and carry no
//! congestion cost (any number of nets may share one); their demand is
//! reported in [`GlobalStats::access_overuse`] but neither counts as
//! overflow nor triggers rip-up.
//!
//! The maze search is A* bounded to the job's window with a closed set, the
//! lookahead bound weighted by [`GlobalCosts::astar_fac`], epoch-stamped
//! per-thread scratch (no clearing or allocation per search) and a cached
//! heuristic per node.

mod capacity;
mod grid;
mod lines;
mod lookahead;
mod pattern;
mod router;
mod search;

use crate::api::{ClassId, Connection, ConnectionId, FailureReason, LayerId, LayerMask};
use crate::board::RouteBoard;
use crate::rules::RuleTable;
use crate::telemetry::Telemetry;
use konnect_geom::{BBox, Coord, Point};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// The PathFinder present/history schedule (VPR style). Factors are fixed
/// point with 1000 = 1.0.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct PresentSchedule {
    /// Present factor during the seed iteration.
    pub first_iter_pres_fac: u32,
    /// Present factor of negotiation iteration 1.
    pub initial_pres_fac: u32,
    /// Growth of the present factor per iteration.
    pub pres_fac_mult: u32,
    /// Ceiling on the present factor.
    pub max_pres_fac: u32,
    /// History added per unit of overuse after each iteration.
    pub acc_fac: u32,
}

impl Default for PresentSchedule {
    fn default() -> Self {
        PresentSchedule {
            first_iter_pres_fac: 0,
            initial_pres_fac: 500,
            pres_fac_mult: 1_300,
            max_pres_fac: 1_000_000,
            acc_fac: 1_000,
        }
    }
}

/// Global cost knobs, fixed point with 1000 = 1.0.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct GlobalCosts {
    /// One layer change, in millimetres of straight wire.
    pub via: u32,
    /// Extra cost factor for a step against the layer's preferred direction.
    pub wrong_way: u32,
    /// Cost factor of a resource the connection's net already holds.
    pub same_net: u32,
    /// A* heuristic weight (VPR `astar_fac`); 1000 keeps it admissible.
    pub astar_fac: u32,
    /// Seed-only penalty per unit of relative overuse.
    pub seed_congestion: u32,
}

impl Default for GlobalCosts {
    fn default() -> Self {
        GlobalCosts {
            via: 2_000,
            wrong_way: 1_000,
            same_net: 500,
            astar_fac: 1_000,
            seed_congestion: 2_000,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct GlobalSettings {
    pub gcell_pitch: Coord,
    pub fine_pitch: Coord,
    pub max_iterations: u32,
    pub schedule: PresentSchedule,
    pub costs: GlobalCosts,
    /// Refine a base cell when its 3 × 3 neighbourhood holds at least this
    /// many routable pad centres; 0 disables refinement.
    pub refine_min_pads: u32,
    /// Window margin around a connection's terminals, in base cells.
    pub window_margin: u32,
    /// Extra margin, in base cells, each time a connection is re-routed.
    pub window_growth: u32,
    /// Ceiling on the accumulated growth, in base cells.
    pub window_growth_cap: u32,
    /// Re-route a connection over its whole length every this many
    /// re-routes; in between only the stretch from its first to its last
    /// overused resource is re-routed, keeping the clean ends. 1 always
    /// re-routes in full.
    pub full_reroute_every: u32,
    /// Stop negotiating after this many iterations without cutting the
    /// best total overflow by at least 1 % (and at least 1); 0 never stops
    /// early.
    pub stall_iterations: u32,
    /// Whether zone fills on routable layers block capacity.
    pub zones_block: bool,
    /// Layers whose preferred direction is vertical; `None` alternates by
    /// routable-layer order, starting horizontal.
    pub vertical_layers: Option<LayerMask>,
    /// Worker threads; 0 uses the current rayon pool. Never changes the
    /// result.
    pub threads: u32,
    /// Jobs per batch (rayon chunk now, GPU dispatch later).
    pub batch_max_jobs: u32,
    /// Track capacity on one layer only inside a box ([`TrackRegion`]);
    /// via sites are unaffected. `None`: no such limit.
    pub track_region: Option<TrackRegion>,
}

/// New track copper on [`Self::layer`] stays inside [`Self::bbox`]; vias
/// still cross the layer anywhere. Opens a plane layer to routing in one
/// region only (the caller makes the layer routable and takes the plane's
/// fill out of the box). Global routing gives the layer no track capacity
/// outside the box and the detailed search takes no track step there.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct TrackRegion {
    pub layer: LayerId,
    pub bbox: BBox,
}

impl TrackRegion {
    /// Up to four boxes that cover `within` outside [`Self::bbox`]: full
    /// height left and right of it, then above and below it between those.
    pub fn outside(&self, within: BBox) -> Vec<BBox> {
        let (r, w) = (self.bbox, within);
        let mut out = Vec::new();
        let mut push = |x0: Coord, y0: Coord, x1: Coord, y1: Coord| {
            if x1 > x0 && y1 > y0 {
                out.push(BBox::new(Point::new(x0, y0), Point::new(x1, y1)));
            }
        };
        let (lx, hx) = (
            r.min.x.clamp(w.min.x, w.max.x),
            r.max.x.clamp(w.min.x, w.max.x),
        );
        push(w.min.x, w.min.y, lx, w.max.y);
        push(hx, w.min.y, w.max.x, w.max.y);
        push(lx, w.min.y, hx, r.min.y.min(w.max.y));
        push(lx, r.max.y.max(w.min.y), hx, w.max.y);
        out
    }
}

impl Default for GlobalSettings {
    fn default() -> Self {
        GlobalSettings {
            gcell_pitch: crate::api::GCELL_PITCH_NM,
            fine_pitch: crate::api::GCELL_FINE_PITCH_NM,
            max_iterations: 30,
            schedule: PresentSchedule::default(),
            costs: GlobalCosts::default(),
            refine_min_pads: 12,
            window_margin: 4,
            window_growth: 2,
            window_growth_cap: 16,
            full_reroute_every: 4,
            stall_iterations: 8,
            zones_block: false,
            vertical_layers: None,
            threads: 0,
            batch_max_jobs: 256,
            track_region: None,
        }
    }
}

/// Inputs beyond the board and rules: per-class layer restrictions.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct LayerPolicy {
    /// Layers each net class may route on (intersected with the grid's
    /// layers). Classes not listed may use every grid layer.
    pub class_layers: BTreeMap<ClassId, LayerMask>,
}

/// The GCell grid: `cols × rows` cells per layer over `bbox`.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GCellGrid {
    /// The area the base cells cover: `cols · pitch × rows · pitch` from
    /// `bbox.min`.
    pub bbox: BBox,
    pub pitch: Coord,
    pub cols: u32,
    pub rows: u32,
    pub layers: LayerMask,
    /// Fine cell size; `pitch` when nothing is refined.
    #[serde(default)]
    pub fine_pitch: Coord,
    /// Base cells (`row · cols + col`) split into fine cells, ascending.
    #[serde(default)]
    pub refined: Vec<u32>,
    /// Capacity per routing resource, in default track pitches (planar
    /// edges) or via sites: every planar edge of layer slot 0, then slot 1,
    /// …, then one via resource per planar cell. Edges are enumerated by
    /// cell (base cells, then fine cells), east neighbours before south
    /// ones. Built by [`build_grid`].
    #[serde(default)]
    pub capacity: Vec<u16>,
}

impl GCellGrid {
    /// Dense GCell id: `(layer_slot · rows + row) · cols + col`.
    pub fn id(&self, layer_slot: u32, col: u32, row: u32) -> u32 {
        (layer_slot * self.rows + row) * self.cols + col
    }

    /// Routable layer slots (the grid's layers in ascending id order).
    pub fn layer_slots(&self) -> u32 {
        self.layers.count()
    }

    /// The layer of slot `slot`.
    pub fn slot_layer(&self, slot: u32) -> Option<LayerId> {
        self.layers.iter().nth(slot as usize)
    }

    /// The slot of `layer`, if the grid routes it.
    pub fn layer_slot(&self, layer: LayerId) -> Option<u32> {
        self.layers
            .contains(layer)
            .then(|| LayerMask(self.layers.0 & ((1u32 << layer.0) - 1)).count())
    }

    /// Fine cells per side of a refined base cell (1 when unrefined).
    pub fn refine_factor(&self) -> u32 {
        if self.fine_pitch > 0 && self.fine_pitch < self.pitch && self.pitch % self.fine_pitch == 0
        {
            (self.pitch / self.fine_pitch) as u32
        } else {
            1
        }
    }

    fn base_cells(&self) -> u32 {
        self.cols * self.rows
    }

    fn fine_per_layer(&self) -> u32 {
        let k = self.refine_factor();
        if k == 1 {
            0
        } else {
            self.refined.len() as u32 * k * k
        }
    }

    /// Id of fine cell `sub = sy · k + sx` of `refined[index]` on `slot`.
    pub fn fine_id(&self, layer_slot: u32, index: u32, sub: u32) -> u32 {
        let k = self.refine_factor();
        self.layer_slots() * self.base_cells()
            + layer_slot * self.fine_per_layer()
            + index * k * k
            + sub
    }

    /// Ids in use span `0..cell_count()` (refined base ids are skipped).
    pub fn cell_count(&self) -> u32 {
        self.layer_slots() * (self.base_cells() + self.fine_per_layer())
    }

    /// The layer and area of cell `id`.
    pub fn cell_bbox(&self, id: u32) -> Option<(LayerId, BBox)> {
        let base = self.base_cells();
        let slots = self.layer_slots();
        let (slot, x0, y0, size) = if id < slots * base {
            let (slot, c) = (id / base, id % base);
            let (col, row) = (c % self.cols, c / self.cols);
            (
                slot,
                i64::from(col) * i64::from(self.pitch),
                i64::from(row) * i64::from(self.pitch),
                i64::from(self.pitch),
            )
        } else {
            let f = id - slots * base;
            let per = self.fine_per_layer();
            if per == 0 || f >= slots * per {
                return None;
            }
            let k = self.refine_factor();
            let (slot, rest) = (f / per, f % per);
            let (index, sub) = (rest / (k * k), rest % (k * k));
            let b = self.refined[index as usize];
            let (col, row) = (b % self.cols, b / self.cols);
            let fp = i64::from(self.fine_pitch);
            (
                slot,
                i64::from(col) * i64::from(self.pitch) + i64::from(sub % k) * fp,
                i64::from(row) * i64::from(self.pitch) + i64::from(sub / k) * fp,
                fp,
            )
        };
        let layer = self.slot_layer(slot)?;
        let at = |dx: i64, dy: i64| {
            Point::new(
                (i64::from(self.bbox.min.x) + dx) as Coord,
                (i64::from(self.bbox.min.y) + dy) as Coord,
            )
        };
        Some((layer, BBox::new(at(x0, y0), at(x0 + size, y0 + size))))
    }

    /// The finest cell holding `p` on `layer`.
    pub fn locate(&self, layer: LayerId, p: Point) -> Option<u32> {
        let slot = self.layer_slot(layer)?;
        let dx = i64::from(p.x) - i64::from(self.bbox.min.x);
        let dy = i64::from(p.y) - i64::from(self.bbox.min.y);
        let pitch = i64::from(self.pitch);
        if dx < 0 || dy < 0 || dx >= pitch * i64::from(self.cols) {
            return None;
        }
        if dy >= pitch * i64::from(self.rows) {
            return None;
        }
        let (col, row) = ((dx / pitch) as u32, (dy / pitch) as u32);
        let b = row * self.cols + col;
        match self.refined.binary_search(&b) {
            Ok(index) if self.refine_factor() > 1 => {
                let k = i64::from(self.refine_factor());
                let fp = i64::from(self.fine_pitch);
                let sx = ((dx - i64::from(col) * pitch) / fp).min(k - 1);
                let sy = ((dy - i64::from(row) * pitch) / fp).min(k - 1);
                Some(self.fine_id(slot, index as u32, (sy * k + sx) as u32))
            }
            _ => Some(self.id(slot, col, row)),
        }
    }
}

/// One connection's corridor: GCell ids (see [`GCellGrid::id`]), in path
/// order, and the layers it may use.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Corridor {
    pub connection: ConnectionId,
    /// From the `from` terminal's cell to the `to` terminal's cell. A layer
    /// change is two consecutive ids of one area on two layers of
    /// `layers`: one through via, which also crosses every layer between
    /// them (allowed or not) without running track there.
    pub cells: Vec<u32>,
    /// The layers it may use: its net class's layers on the grid (every grid
    /// layer when the class is unrestricted).
    pub layers: LayerMask,
    /// The layers the corridor's cells are on (its layer assignment), a
    /// subset of `layers`.
    #[serde(default)]
    pub assigned: LayerMask,
}

/// One negotiation iteration, for reports.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct IterationStats {
    pub iteration: u32,
    /// Present factor used, ×1000.
    pub pres_fac: u32,
    /// Connections (re)routed.
    pub routed: u32,
    /// Total overflow after the iteration.
    pub overflow: u64,
    /// Resources overused after the iteration.
    pub overused: u32,
}

/// Run totals. Deterministic: no timings (those go to telemetry).
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct GlobalStats {
    pub converged: bool,
    /// Σ (demand − capacity) over overused resources with capacity.
    pub total_overflow: u64,
    pub max_overflow: u16,
    pub overused_resources: u32,
    /// Demand on zero-capacity resources at terminal cells (pin access).
    pub access_overuse: u64,
    /// Total corridor length between cell centres, nm.
    pub wirelength_nm: u64,
    /// Via sites in use, counting a site a net's connections share once.
    pub vias: u64,
    pub pattern_seeds: u32,
    pub maze_seeds: u32,
    /// Searches that needed the whole grid after their window failed.
    pub full_grid_searches: u32,
    /// The iteration the result comes from (0 is the seed): the one with
    /// the fewest unrouted connections, then the least overflow.
    #[serde(default)]
    pub best_iteration: u32,
    pub per_iteration: Vec<IterationStats>,
}

#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct GlobalResult {
    pub corridors: Vec<Corridor>,
    /// Connections left overflowed in the returned (best) iteration.
    pub overflowed: Vec<ConnectionId>,
    /// Per-GCell overflow (demand − capacity, floored at 0), by GCell id.
    /// A cell carries the overflow of its east and south edges on its layer
    /// and, on every layer, the overflow of its via sites.
    pub overflow: Vec<u16>,
    /// Connections with no corridor at all, and why.
    #[serde(default)]
    pub failed: Vec<(ConnectionId, FailureReason)>,
    /// Negotiation iterations run after the seed.
    #[serde(default)]
    pub iterations: u32,
    #[serde(default)]
    pub stats: GlobalStats,
}

impl GlobalResult {
    /// The overflow of one layer slot at base resolution (`rows × cols`,
    /// row-major), summing a refined cell's fine cells.
    pub fn overflow_layer(&self, grid: &GCellGrid, slot: u32) -> Vec<u32> {
        let mut out = vec![0u32; (grid.cols * grid.rows) as usize];
        for (id, &v) in self.overflow.iter().enumerate() {
            if v == 0 {
                continue;
            }
            let Some((layer, bbox)) = grid.cell_bbox(id as u32) else {
                continue;
            };
            if grid.layer_slot(layer) != Some(slot) {
                continue;
            }
            let col = (i64::from(bbox.min.x) - i64::from(grid.bbox.min.x)) / i64::from(grid.pitch);
            let row = (i64::from(bbox.min.y) - i64::from(grid.bbox.min.y)) / i64::from(grid.pitch);
            out[(row as u32 * grid.cols + col as u32) as usize] += u32::from(v);
        }
        out
    }
}

/// Build the grid from the board's fixed copper.
pub fn build_grid(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    settings: &GlobalSettings,
) -> GCellGrid {
    build_grid_with(board, rules, settings, &mut Telemetry::default())
}

/// [`build_grid`], adding its wall time to [`crate::api::Stage::Global`]
/// and to the `global_grid_us` counter.
pub fn build_grid_with(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    settings: &GlobalSettings,
    telemetry: &mut Telemetry,
) -> GCellGrid {
    let start = std::time::Instant::now();
    let grid = with_threads(settings.threads, || {
        let mut grid = grid::lay_out(board, settings);
        grid.capacity = capacity::compute(board, rules, &grid, settings);
        grid
    });
    let us = start.elapsed().as_micros() as u64;
    telemetry.add_stage_time(crate::api::Stage::Global, us);
    telemetry.count("global_grid_us", us);
    telemetry.count("global_refined_cells", grid.refined.len() as u64);
    grid
}

/// Route every connection globally.
pub fn route_global(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    grid: &GCellGrid,
    connections: &[Connection],
    settings: &GlobalSettings,
) -> GlobalResult {
    route_global_with(
        board,
        rules,
        grid,
        connections,
        settings,
        &LayerPolicy::default(),
        &mut Telemetry::default(),
    )
}

/// [`route_global`] with per-class layer masks, recording timings and
/// counters in `telemetry`.
pub fn route_global_with(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    grid: &GCellGrid,
    connections: &[Connection],
    settings: &GlobalSettings,
    policy: &LayerPolicy,
    telemetry: &mut Telemetry,
) -> GlobalResult {
    let start = std::time::Instant::now();
    let result = with_threads(settings.threads, || {
        telemetry.threads = rayon::current_num_threads() as u32;
        router::run(board, rules, grid, connections, settings, policy, telemetry)
    });
    telemetry.add_stage_time(
        crate::api::Stage::Global,
        start.elapsed().as_micros() as u64,
    );
    result
}

fn with_threads<R: Send>(threads: u32, f: impl FnOnce() -> R + Send) -> R {
    if threads == 0 {
        return f();
    }
    match rayon::ThreadPoolBuilder::new()
        .num_threads(threads as usize)
        .build()
    {
        Ok(pool) => pool.install(f),
        Err(_) => f(),
    }
}

#[cfg(test)]
mod tests;
