//! Negotiated congestion over tiles (owned by WS-D2).
//!
//! The TritonRoute/Dr.CU search-and-repair loop: fixed copper is a hard
//! wall, other nets' new copper is soft with present and history costs, and
//! each iteration rips up only the connections in conflict. The board is cut
//! into tiles of [`crate::api::TILE_CORE_NM`] with [`crate::api::TILE_HALO_NM`]
//! halos, 4-coloured so same-colour tiles run in parallel (rayon), with the
//! tile grid shifted by half a tile on odd iterations. Dense BGA fields are
//! split into sub-tiles and scheduled first. After the iteration limit,
//! remaining connections fail with a [`crate::api::FailureReason`].
//!
//! # One iteration
//!
//! 1. **Rip up** the queued connections (iteration 0: all of them; later:
//!    those in conflict, plus failures worth retrying). Everything else
//!    keeps its copper — connection-based incremental rerouting (AIR).
//! 2. **Cut** each queued connection into per-tile pieces, pinned at tile
//!    crossings on the base lattice ([`pieces`]).
//! 3. **Route** pass by pass — BGA sub-tiles by colour, then the grid by
//!    colour. A pass's tiles have pairwise disjoint windows, so they are
//!    independent: their rasters are built in parallel from the copper
//!    committed so far, then each tile routes its pieces in order, one per
//!    round, all tiles' jobs going to the backend as one [`Batch`]
//!    (frozen snapshot, disjoint windows, ordered commits). Each proposal is
//!    re-checked exactly against fixed copper before it is committed; a
//!    failing step vetoes its lattice nodes and the piece is searched again.
//! 4. **Detect conflicts** exactly between new copper and all routed copper
//!    of other nets ([`conflicts`]); both sides of every conflict are queued
//!    for the next iteration, and the cells where they clash gain history.
//!    History never decays. The present factor grows by
//!    [`NegotiationSettings::present_growth`] per iteration.
//!
//! Negotiation stops when nothing is queued, or after
//! [`NegotiationSettings::max_iterations`]. Connections still in conflict
//! then keep their copper greedily in commit order (a connection keeps its
//! copper unless it conflicts with one already kept); the rest fail with
//! [`FailureReason::Congestion`]. Every connection ends routed or failed
//! with a reason ([`DetailFailure`] is the detailed form).
//!
//! # Determinism
//!
//! Nothing depends on thread count or timing: passes are independent by
//! construction, results are collected in tile order, a tile routes its
//! pieces in `order_key` order, conflict lists are sorted, and every
//! tie-break is explicit. A 1-thread and an N-thread run return equal
//! results.
//!
//! # Searching
//!
//! Search goes through the [`SearchBackend`](crate::compute::SearchBackend)
//! seam only. [`ReferenceSearch`] is a simple A* backend that makes this
//! module usable (and testable) on its own; WS-D1's search replaces it.

mod conflicts;
mod copper;
mod obstacles;
mod pieces;
pub mod reference;
mod run;
#[cfg(test)]
mod tests;
mod tiles;

pub use reference::ReferenceSearch;
pub use tiles::{detect_bga_fields, BgaField};

use crate::api::{
    Connection, ConnectionId, FailureReason, Generation, ItemKind, ItemRef, LayerId, LayerMask,
    NetId, Stage, TileId, LATTICE_FINE_PITCH_NM, LATTICE_PITCH_NM,
};
use crate::board::RouteBoard;
use crate::compute::{
    Batch, Cost, CostParams, JobPool, ProfileGeometry, RouteCompute, SearchOutcome,
};
use crate::connectivity::ConnectivityReport;
use crate::global::{GCellGrid, GlobalResult};
use crate::plan::{RoutedCopper, RoutedTrack, RoutedVia};
use crate::rules::{RuleSubject, RuleTable, ViaRule};
use crate::telemetry::Telemetry;
use conflicts::{HistoryMap, RoutedIndex};
use copper::PieceCopper;
use konnect_geom::{ArcPolicy, BBox, Coord, Point};
use obstacles::Obstacles;
use pieces::{PieceJob, PinRegistry};
use rayon::prelude::*;
use run::{PieceDone, TileRun};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::sync::Arc;
use std::time::Instant;
use tiles::Tiling;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct NegotiationSettings {
    pub max_iterations: u32,
    /// Present-cost factor growth per iteration, ×1000.
    pub present_growth: u32,
    pub history_increment: u16,
    /// Worker threads; 0 = rayon's default. Must not change the result.
    pub threads: usize,
}

impl Default for NegotiationSettings {
    fn default() -> Self {
        NegotiationSettings {
            max_iterations: 10,
            present_growth: 1_500,
            history_increment: 1,
            threads: 0,
        }
    }
}

/// The finer knobs of negotiation, with defaults that need no tuning to
/// run. Lengths in nm; factors ×1000.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct NegotiationTuning {
    /// Present-cost factor of iteration 0 (PathFinder `p`); 0 routes the
    /// first iteration blind to other nets.
    pub initial_present_factor: u32,
    /// The present factor never falls below this after iteration 0 (VPR's
    /// `initial_pres_fac`); it then grows by `present_growth`.
    pub present_factor_floor: u32,
    /// History-cost factor (PathFinder `h`), constant.
    pub history_factor: u32,
    /// Cost of one through via.
    pub via_cost: Cost,
    /// Extra cost of a node the tri-state contract calls `Near`.
    pub near_step: Cost,
    /// Lattice pitch of the regular grid tiles.
    pub base_pitch: Coord,
    /// Lattice pitch of BGA sub-tiles.
    pub fine_pitch: Coord,
    /// A footprint is a BGA field with at least this many pads …
    pub bga_min_pads: u32,
    /// … in at least this many distinct rows and columns …
    pub bga_min_rows: u32,
    /// … at a pad pitch of at most this.
    pub bga_max_pitch: Coord,
    /// Escape ring added around a field's pads.
    pub bga_margin: Coord,
    /// Largest BGA sub-tile core side. A field is split at least 2 × 2, and
    /// never into cores under two halos plus 0.5 mm.
    pub bga_sub_tile_max: Coord,
    /// A connection is one piece when its guide line plus this margin fits
    /// one tile's commit zone.
    pub fit_margin: Coord,
    /// How far along a boundary a pin may move from the guide crossing.
    pub pin_search: Coord,
    /// Exact-check rejections of one piece per iteration before it fails.
    pub max_commit_retries: u32,
    /// Lattice nodes offered per terminal and layer.
    pub max_terminal_nodes: u32,
    /// Tiles whose rasters are alive at once within a pass (memory bound;
    /// never changes the result). 0 = sixteen per thread.
    pub chunk_tiles: usize,
    /// Shift the tile grid by half a tile on odd iterations.
    pub shift_odd_iterations: bool,
    /// Always use lockstep batch rounds (one job per tile per round, all
    /// tiles in one batch), even when the backend can fork. The result is
    /// the same either way; forks are faster on a CPU.
    pub lockstep_rounds: bool,
    /// The generation stamped on routed copper.
    pub generation: Generation,
}

impl Default for NegotiationTuning {
    fn default() -> Self {
        NegotiationTuning {
            initial_present_factor: 500,
            present_factor_floor: 500,
            history_factor: 1_000,
            via_cost: Cost(8_000),
            near_step: Cost(100),
            base_pitch: LATTICE_PITCH_NM,
            fine_pitch: LATTICE_FINE_PITCH_NM,
            bga_min_pads: 48,
            bga_min_rows: 6,
            bga_max_pitch: 1_000_000,
            bga_margin: 1_000_000,
            bga_sub_tile_max: 5_000_000,
            fit_margin: 1_000_000,
            pin_search: 1_500_000,
            max_commit_retries: 8,
            max_terminal_nodes: 16,
            chunk_tiles: 0,
            shift_odd_iterations: true,
            lockstep_rounds: false,
            generation: Generation(1),
        }
    }
}

/// One tile of the schedule.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Tile {
    pub id: TileId,
    /// The core this tile owns. Pieces are cut at core boundaries; a tile
    /// commits only inside its window less a context margin (clearance
    /// plus the widest copper), so everything a commit must keep clear of
    /// is in view.
    pub core: BBox,
    /// Core plus halo: what the tile's rasters cover.
    pub window: BBox,
    /// Tiles of one colour and phase never share a halo. Grid tiles use
    /// 0..4; BGA sub-tiles prefer a 2 × 2 pattern and take a higher colour
    /// only when two fields are within a halo of each other.
    pub colour: u8,
    /// 0: BGA sub-tile (routed first); 1: regular grid tile.
    pub phase: u8,
    /// Lattice pitch of the tile's rasters, nm.
    pub pitch: Coord,
}

/// Why a connection was not routed, in more detail than
/// [`FailureReason`] (see [`DetailFailure::reason`]).
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(tag = "failure", rename_all = "snake_case")]
pub enum DetailFailure {
    /// The terminal offers no open lattice node on an allowed layer.
    NoAccess { item: ItemRef },
    /// S3 supplied corridors but none for this connection, or the
    /// connection may use no routable layer.
    NoCorridor,
    /// No legal pin where the connection crosses from one tile to another.
    NoPin { from: TileId, to: TileId },
    /// The search found no path under the hard walls, in both tilings.
    Unreachable,
    /// The search's expansion budget ran out.
    BudgetExhausted,
    /// Proposals kept failing the exact check.
    ExactCheckRejected,
    /// Still in conflict after the last iteration, with these connections.
    Congestion {
        iterations: u32,
        against: Vec<ConnectionId>,
    },
}

impl DetailFailure {
    /// The frozen reason this failure is reported as.
    pub fn reason(&self) -> FailureReason {
        match self {
            DetailFailure::NoAccess { item } => FailureReason::NoAccess { item: *item },
            DetailFailure::NoCorridor
            | DetailFailure::NoPin { .. }
            | DetailFailure::Unreachable => FailureReason::NoCorridor,
            DetailFailure::BudgetExhausted | DetailFailure::ExactCheckRejected => {
                FailureReason::BudgetExhausted
            }
            DetailFailure::Congestion { iterations, .. } => FailureReason::Congestion {
                iterations: *iterations,
            },
        }
    }

    /// Whether another iteration (another tiling) could change the outcome.
    fn retryable(&self) -> bool {
        !matches!(
            self,
            DetailFailure::NoAccess { .. } | DetailFailure::NoCorridor
        )
    }

    /// A short stable name, for counters and reports.
    pub fn name(&self) -> &'static str {
        match self {
            DetailFailure::NoAccess { .. } => "no_access",
            DetailFailure::NoCorridor => "no_corridor",
            DetailFailure::NoPin { .. } => "no_pin",
            DetailFailure::Unreachable => "unreachable",
            DetailFailure::BudgetExhausted => "budget_exhausted",
            DetailFailure::ExactCheckRejected => "exact_check_rejected",
            DetailFailure::Congestion { .. } => "congestion",
        }
    }
}

/// What one iteration did. No timings, so equal runs compare equal.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct IterationReport {
    pub iteration: u32,
    /// The tile grid was shifted by half a tile.
    pub shifted: bool,
    pub present_factor: u32,
    /// Connections ripped up and rerouted this iteration.
    pub queued: u32,
    pub pieces: u32,
    pub pins: u32,
    /// Queued connections that got copper.
    pub routed: u32,
    /// Queued connections that failed (some may be retried).
    pub failed: u32,
    /// Conflicting item pairs found against the new copper.
    pub conflict_pairs: u32,
    /// Connections in conflict after this iteration.
    pub conflicted: u32,
    /// Cells with history, and the history total, after this iteration.
    pub history_cells: u64,
    pub history_total: u64,
}

#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct DetailResult {
    pub copper: RoutedCopper,
    pub routed: Vec<ConnectionId>,
    pub failed: Vec<(ConnectionId, FailureReason)>,
    pub iterations: u32,
    pub generation: Generation,
    /// The detailed reason behind each entry of `failed`, same order.
    pub failure_details: Vec<(ConnectionId, DetailFailure)>,
    /// One report per iteration run.
    pub reports: Vec<IterationReport>,
    /// The BGA fields that were split into sub-tiles.
    pub bga_fields: Vec<BgaField>,
}

/// Cut the board into 4-coloured tiles; `shift` offsets the grid by half a
/// tile (odd iterations). BGA sub-tiles (phase 0) come first.
pub fn plan_tiles(board: &RouteBoard, shift: bool) -> Vec<Tile> {
    let tuning = NegotiationTuning::default();
    Tiling::build(board, shift, &tuning, 0)
        .map(|t| t.tiles.into_iter().map(|s| s.tile).collect())
        .unwrap_or_default()
}

/// Run negotiated detailed routing for every corridor in `global`.
///
/// The connections are the board's own task list
/// ([`ConnectivityReport::connections`]); with an empty `global` every
/// connection is routed unconstrained. See [`negotiate_connections`] for
/// the full form.
pub fn negotiate(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    global: &GlobalResult,
    settings: &NegotiationSettings,
    backend: &mut dyn RouteCompute,
) -> DetailResult {
    let connections = ConnectivityReport::build(board, &ArcPolicy::default()).connections(board);
    let mut telemetry = Telemetry::default();
    negotiate_connections(
        board,
        rules,
        &connections,
        global,
        None,
        settings,
        &NegotiationTuning::default(),
        backend,
        &mut telemetry,
    )
}

// ─── Shared context ──────────────────────────────────────────────────────────

/// One clearance profile: nets with the same class and track widths share
/// rasters, painted for a representative net (the lowest id).
#[derive(Debug, Clone)]
pub(crate) struct Profile {
    pub rep: NetId,
    /// Track width per routable layer slot.
    pub widths: Vec<Coord>,
    pub via_diameter: Coord,
}

/// Everything read-only a run shares across threads.
pub(crate) struct Ctx<'a> {
    pub board: &'a RouteBoard,
    pub rules: &'a dyn RuleTable,
    pub tuning: NegotiationTuning,
    pub obstacles: Obstacles,
    /// Routable layers in stack order: a raster's layer slot.
    pub slots: Vec<LayerId>,
    pub profiles: Vec<Profile>,
    pub net_profile: Vec<u16>,
    /// Track width per (net, slot).
    net_width: Vec<Coord>,
    pub net_via: Vec<ViaRule>,
    /// Context a commit needs inside its tile window.
    pub margin: Coord,
}

impl<'a> Ctx<'a> {
    fn new(board: &'a RouteBoard, rules: &'a dyn RuleTable, tuning: NegotiationTuning) -> Self {
        let obstacles = Obstacles::build(board, rules);
        let slots: Vec<LayerId> = board.routable.iter().collect();
        let mut profiles: Vec<Profile> = Vec::new();
        let mut keys: BTreeMap<(u16, Vec<Coord>), u16> = BTreeMap::new();
        let mut net_profile = Vec::with_capacity(board.nets.len());
        let mut net_width = Vec::with_capacity(board.nets.len() * slots.len());
        let mut net_via = Vec::with_capacity(board.nets.len());
        for net in &board.nets {
            let track = RuleSubject::new(ItemKind::Track, Some(net.id));
            let widths: Vec<Coord> = slots
                .iter()
                .map(|&l| rules.track_width(&track, l).preferred.max(1))
                .collect();
            let via = rules.via(&RuleSubject::new(ItemKind::Via, Some(net.id)));
            let class = rules.class_of(Some(net.id)).0;
            let next = profiles.len() as u16;
            let p = *keys.entry((class, widths.clone())).or_insert_with(|| {
                profiles.push(Profile {
                    rep: net.id,
                    widths: widths.clone(),
                    via_diameter: via.diameter,
                });
                next
            });
            net_profile.push(p);
            net_width.extend_from_slice(&widths);
            net_via.push(via);
        }
        let widest = profiles
            .iter()
            .flat_map(|p| p.widths.iter().copied().chain([p.via_diameter]))
            .max()
            .unwrap_or(0);
        let margin = rules
            .max_clearance()
            .saturating_add(widest)
            .saturating_add(2 * tuning.base_pitch);
        Ctx {
            board,
            rules,
            tuning,
            obstacles,
            slots,
            profiles,
            net_profile,
            net_width,
            net_via,
            margin,
        }
    }

    pub fn slot_of(&self, layer: LayerId) -> Option<usize> {
        self.slots.iter().position(|&l| l == layer)
    }

    pub fn width(&self, net: NetId, slot: usize) -> Coord {
        self.net_width[net.index() * self.slots.len() + slot]
    }

    pub fn geometry(&self, profile: u16, slot: usize) -> ProfileGeometry {
        ProfileGeometry {
            track_width: self.profiles[profile as usize].widths[slot],
            sagitta: 0,
        }
    }
}

/// One connection to route, in commit order (its index is its rank).
#[derive(Debug, Clone)]
pub(crate) struct Task {
    pub conn: Connection,
    pub rank: u32,
    pub a: Point,
    pub b: Point,
    pub layers: LayerMask,
    /// Guide line: terminal, corridor cell centres, terminal.
    pub line: Vec<Point>,
    /// Corridor layer at each guide vertex.
    pub hints: Vec<Option<LayerId>>,
    pub profile: u16,
    /// The fixed copper already joined to each terminal (pads, vias,
    /// tracks), any of which the connection may start or end on.
    pub from_island: Vec<ItemRef>,
    pub to_island: Vec<ItemRef>,
}

fn item_point(board: &RouteBoard, item: ItemRef) -> Option<Point> {
    match item {
        ItemRef::Pad(id) => board.pads.get(id.index()).map(|p| p.center),
        ItemRef::Via(id) => board.vias.get(id.index()).map(|v| v.center),
        ItemRef::Track(id) => board.tracks.get(id.index()).map(|t| t.seg.a),
        _ => None,
    }
}

/// Tasks in rank order, plus the connections that cannot even be queued.
fn build_tasks(
    ctx: &Ctx<'_>,
    connections: &[Connection],
    global: &GlobalResult,
    grid: Option<&GCellGrid>,
) -> (Vec<Task>, Vec<(Connection, DetailFailure)>) {
    let corridors: BTreeMap<ConnectionId, &crate::global::Corridor> =
        global.corridors.iter().map(|c| (c.connection, c)).collect();
    let routable = ctx.board.routable;
    // Each item's island, for terminal access through fixed escapes.
    let report = ConnectivityReport::build(ctx.board, &ArcPolicy::default());
    let mut island_of: HashMap<ItemRef, Vec<ItemRef>> = HashMap::new();
    for net in &report.nets {
        for island in &net.islands {
            let usable: Vec<ItemRef> = island
                .items
                .iter()
                .copied()
                .filter(|i| matches!(i, ItemRef::Pad(_) | ItemRef::Via(_) | ItemRef::Track(_)))
                .collect();
            for item in &island.items {
                island_of.insert(*item, usable.clone());
            }
        }
    }
    let island = |item: ItemRef| island_of.get(&item).cloned().unwrap_or_else(|| vec![item]);
    let mut tasks = Vec::new();
    let mut refused = Vec::new();
    for c in connections {
        let (Some(a), Some(b)) = (item_point(ctx.board, c.from), item_point(ctx.board, c.to))
        else {
            let item = if item_point(ctx.board, c.from).is_none() {
                c.from
            } else {
                c.to
            };
            refused.push((*c, DetailFailure::NoAccess { item }));
            continue;
        };
        let mut line = vec![a];
        let mut hints = vec![None];
        let layers = if corridors.is_empty() {
            routable
        } else {
            match corridors.get(&c.id) {
                None => {
                    refused.push((*c, DetailFailure::NoCorridor));
                    continue;
                }
                Some(cor) => {
                    if let Some(g) = grid {
                        let slots: Vec<LayerId> = g.layers.iter().collect();
                        let per_layer = g.cols * g.rows;
                        for &cell in &cor.cells {
                            if per_layer == 0 {
                                break;
                            }
                            let slot = (cell / per_layer) as usize;
                            let rem = cell % per_layer;
                            let (row, col) = (rem / g.cols, rem % g.cols);
                            let p = Point::new(
                                (i64::from(g.bbox.min.x)
                                    + i64::from(g.pitch) * i64::from(col)
                                    + i64::from(g.pitch) / 2)
                                    as Coord,
                                (i64::from(g.bbox.min.y)
                                    + i64::from(g.pitch) * i64::from(row)
                                    + i64::from(g.pitch) / 2)
                                    as Coord,
                            );
                            if line.last() != Some(&p) {
                                line.push(p);
                                hints.push(slots.get(slot).copied());
                            }
                        }
                    }
                    cor.layers.intersection(routable)
                }
            }
        };
        if layers.is_empty() {
            refused.push((*c, DetailFailure::NoCorridor));
            continue;
        }
        line.push(b);
        hints.push(None);
        if line.len() == 2 && a == b {
            line[1] = Point::new(b.x.saturating_add(1), b.y);
        }
        tasks.push(Task {
            conn: *c,
            rank: 0,
            a,
            b,
            layers,
            line,
            hints,
            profile: ctx.net_profile[c.net.index()],
            from_island: island(c.from),
            to_island: island(c.to),
        });
    }
    tasks.sort_by_key(|t| {
        let d = t.b - t.a;
        (d.x.unsigned_abs() + d.y.unsigned_abs(), t.conn.id)
    });
    for (i, t) in tasks.iter_mut().enumerate() {
        t.rank = i as u32;
    }
    (tasks, refused)
}

// ─── The driver ──────────────────────────────────────────────────────────────

/// One tile's place in an iteration's dependency graph.
struct DagNode {
    tile: usize,
    jobs: Vec<PieceJob>,
    weight: usize,
    waiting_on: usize,
    dependents: Vec<usize>,
}

/// What the DAG workers share.
struct DagState {
    nodes: Vec<DagNode>,
    /// Ready nodes, heaviest first, then canonical order.
    ready: BTreeSet<(std::cmp::Reverse<usize>, usize)>,
    finished: usize,
    /// Pieces committed so far this iteration.
    fresh: Vec<Arc<PieceCopper>>,
    results: Vec<Option<(Vec<PieceDone>, Telemetry, u64)>>,
}

/// Route one iteration's tiles as a dependency graph, one backend copy per
/// worker thread.
///
/// `order` is the canonical schedule (passes in order, tiles in id order).
/// Tile `t` depends on every earlier tile whose window meets `t`'s: only
/// those can put copper within reach of `t`'s rasters, since commits stay a
/// context margin inside their own window. Any execution that respects
/// these edges therefore computes exactly what running the passes one after
/// another computes; there are just no pass barriers, so a slow tile holds
/// up only its neighbours. Results come back in `order`.
#[allow(clippy::too_many_arguments)]
fn run_dag(
    ctx: &Ctx<'_>,
    tiling: &Tiling,
    order: &[usize],
    by_tile: &mut [Vec<PieceJob>],
    tasks: &[Task],
    kept: &[Arc<PieceCopper>],
    history: &HistoryMap,
    params: &CostParams,
    iteration: u32,
    forks: Vec<Box<dyn RouteCompute + Send>>,
    telemetry: &mut Telemetry,
) -> Vec<PieceDone> {
    let n = order.len();
    let mut nodes: Vec<DagNode> = order
        .iter()
        .map(|&t| {
            let jobs = std::mem::take(&mut by_tile[t]);
            DagNode {
                tile: t,
                weight: jobs.len(),
                jobs,
                waiting_on: 0,
                dependents: Vec::new(),
            }
        })
        .collect();
    let mut preds: Vec<Vec<usize>> = vec![Vec::new(); n];
    for k in 0..n {
        let wk = tiling.tiles[nodes[k].tile].tile.window;
        for j in 0..k {
            if tiling.tiles[nodes[j].tile].tile.window.intersects(&wk) {
                nodes[j].dependents.push(k);
                nodes[k].waiting_on += 1;
                preds[k].push(j);
            }
        }
    }
    // List scheduling by bottom level: a tile's priority is the work on
    // the longest chain it starts (pieces as the work estimate), so the
    // critical path — the dense field and its neighbours — starts first.
    let mut level = vec![0usize; n];
    for k in (0..n).rev() {
        let below = nodes[k]
            .dependents
            .iter()
            .map(|&d| level[d])
            .max()
            .unwrap_or(0);
        level[k] = nodes[k].weight + below;
    }
    for (k, node) in nodes.iter_mut().enumerate() {
        node.weight = level[k];
    }
    let ready = nodes
        .iter()
        .enumerate()
        .filter(|(_, d)| d.waiting_on == 0)
        .map(|(k, d)| (std::cmp::Reverse(d.weight), k))
        .collect();
    let state = std::sync::Mutex::new(DagState {
        nodes,
        ready,
        finished: 0,
        fresh: Vec::new(),
        results: (0..n).map(|_| None).collect(),
    });
    let wake = std::sync::Condvar::new();
    let t0 = Instant::now();
    std::thread::scope(|scope| {
        for mut backend in forks {
            let (state, wake) = (&state, &wake);
            scope.spawn(move || loop {
                let (k, tile, jobs, store) = {
                    let mut s = state.lock().expect("no worker panicked");
                    loop {
                        if s.finished == n {
                            return;
                        }
                        if let Some(first) = s.ready.pop_first() {
                            let k = first.1;
                            let jobs = std::mem::take(&mut s.nodes[k].jobs);
                            let tile = s.nodes[k].tile;
                            // Only copper within reach: no point cloning (and
                            // contending on) every piece's refcount.
                            let reach = TileRun::reach(ctx, &tiling.tiles[tile]);
                            let store: Vec<Arc<PieceCopper>> = kept
                                .iter()
                                .chain(s.fresh.iter())
                                .filter(|p| p.bbox.is_some_and(|b| b.intersects(&reach)))
                                .cloned()
                                .collect();
                            break (k, tile, jobs, store);
                        }
                        s = wake.wait(s).expect("no worker panicked");
                    }
                };
                let start = Instant::now();
                let mut run =
                    TileRun::build(ctx, &tiling.tiles[tile], jobs, tasks, &store, history);
                drop(store);
                run.derive_masks(backend.as_mut());
                run.narrow_vias(ctx);
                let built = start.elapsed().as_micros() as u64;
                run.route_all(ctx, tasks, params, iteration, backend.as_mut());
                run.telemetry.raster_rebuild_us += built;
                let busy = start.elapsed().as_micros() as u64;
                let mut s = state.lock().expect("no worker panicked");
                for d in &run.done {
                    if let Ok(p) = &d.result {
                        s.fresh.push(p.clone());
                    }
                }
                let deps = std::mem::take(&mut s.nodes[k].dependents);
                for d in deps {
                    s.nodes[d].waiting_on -= 1;
                    if s.nodes[d].waiting_on == 0 {
                        let w = s.nodes[d].weight;
                        s.ready.insert((std::cmp::Reverse(w), d));
                    }
                }
                s.results[k] = Some((std::mem::take(&mut run.done), run.telemetry, busy));
                s.finished += 1;
                wake.notify_all();
            });
        }
    });
    let wall = t0.elapsed().as_micros() as u64;
    let s = state.into_inner().expect("no worker panicked");
    let mut done = Vec::new();
    let mut busy_total = 0;
    // The measured critical path: no schedule on any number of threads can
    // beat it.
    let mut finish = vec![0u64; n];
    let mut via: Vec<Option<usize>> = vec![None; n];
    let mut busy_of = vec![0u64; n];
    let mut work = vec![0u64; n];
    for (k, r) in s.results.into_iter().enumerate() {
        let (d, t, busy) = r.expect("every tile ran");
        telemetry.merge(&t);
        busy_total += busy;
        busy_of[k] = busy;
        let e = t.counters.get("expansions").copied().unwrap_or(0);
        work[k] = e + preds[k].iter().map(|&j| work[j]).max().unwrap_or(0);
        let best = preds[k]
            .iter()
            .copied()
            .max_by_key(|&j| (finish[j], std::cmp::Reverse(j)));
        via[k] = best;
        finish[k] = busy + best.map_or(0, |j| finish[j]);
        done.extend(d);
    }
    telemetry.count("us_tiles", wall);
    telemetry.count("us_tile_busy", busy_total);
    // The same in search expansions: deterministic, unlike wall time.
    telemetry.count(
        "dag_critical_expansions",
        work.iter().copied().max().unwrap_or(0),
    );
    if let Some(end) = (0..n).max_by_key(|&k| (finish[k], std::cmp::Reverse(k))) {
        telemetry.count("us_dag_critical", finish[end]);
        let mut at = Some(end);
        while let Some(k) = at {
            let phase = if tiling.tiles[order[k]].tile.phase == 0 {
                "bga"
            } else {
                "grid"
            };
            telemetry.count(&format!("us_dag_critical_{phase}"), busy_of[k]);
            telemetry.count(&format!("dag_critical_tiles_{phase}"), 1);
            at = via[k];
        }
    }
    telemetry.count("dag_tiles", n as u64);
    done
}

/// Route one chunk of one pass in lockstep: build the tiles' rasters, then
/// run rounds (one job per tile, all tiles in one batch) until every tile's
/// queue is empty. For backends that cannot fork.
#[allow(clippy::too_many_arguments)]
fn run_chunk(
    ctx: &Ctx<'_>,
    tiling: &Tiling,
    chunk: &[usize],
    jobs: Vec<Vec<PieceJob>>,
    tasks: &[Task],
    store: &[Arc<PieceCopper>],
    history: &HistoryMap,
    params: &CostParams,
    iteration: u32,
    backend: &mut dyn RouteCompute,
    pool: &rayon::ThreadPool,
    telemetry: &mut Telemetry,
) -> Vec<PieceDone> {
    let t0 = Instant::now();
    let mut runs: Vec<TileRun> = pool.install(|| {
        chunk
            .par_iter()
            .zip(jobs.into_par_iter())
            .map(|(&t, j)| TileRun::build(ctx, &tiling.tiles[t], j, tasks, store, history))
            .collect()
    });
    for r in &mut runs {
        r.derive_masks(backend);
    }
    pool.install(|| runs.par_iter_mut().for_each(|r| r.narrow_vias(ctx)));
    telemetry.raster_rebuild_us += t0.elapsed().as_micros() as u64;

    let (mut us_prepare, mut us_search, mut us_finish) = (0u64, 0u64, 0u64);
    let mut round = 0u32;
    loop {
        let tp = Instant::now();
        let prepared: Vec<_> = pool.install(|| {
            runs.par_iter_mut()
                .map(|r| r.prepare(ctx, tasks, iteration))
                .collect()
        });
        us_prepare += tp.elapsed().as_micros() as u64;
        if prepared.iter().all(Option::is_none) {
            break;
        }
        let mut job_pool = JobPool::default();
        let mut stacks = Vec::new();
        let mut owners = Vec::new();
        for (i, p) in prepared.into_iter().enumerate() {
            if let Some(p) = p {
                let mut job = p.job;
                job.window = stacks.len() as u32;
                job_pool.push(job, &p.sources, &p.targets, &[]);
                stacks.push(runs[i].take_stack(p.stack));
                owners.push((i, p.stack));
            }
        }
        let batch = Batch {
            index: round,
            jobs: (0..stacks.len() as u32).collect(),
        };
        let ts = Instant::now();
        let outcomes = backend.search_batch(&batch, &job_pool, &stacks, params);
        us_search += ts.elapsed().as_micros() as u64;
        telemetry.count("batch_jobs", stacks.len() as u64);
        assert_eq!(
            outcomes.len(),
            stacks.len(),
            "a backend returns one outcome per job"
        );
        let mut per_run: Vec<Option<SearchOutcome>> = (0..runs.len()).map(|_| None).collect();
        for (((i, k), stack), out) in owners.into_iter().zip(stacks).zip(outcomes) {
            runs[i].put_stack(k, stack);
            per_run[i] = Some(out);
        }
        let tf = Instant::now();
        pool.install(|| {
            runs.par_iter_mut()
                .zip(per_run.into_par_iter())
                .for_each(|(r, o)| {
                    if let Some(o) = o {
                        r.finish(ctx, tasks, o);
                    }
                })
        });
        us_finish += tf.elapsed().as_micros() as u64;
        round += 1;
    }
    telemetry.count("rounds", u64::from(round));
    telemetry.count("us_prepare", us_prepare);
    telemetry.count("us_search", us_search);
    telemetry.count("us_finish", us_finish);
    let mut done = Vec::new();
    for r in runs {
        debug_assert!(!r.has_work());
        telemetry.merge(&r.telemetry);
        done.extend(r.done);
    }
    done
}

/// [`negotiate`] with an explicit task list, optional GCell grid (to read
/// corridor cells as guide lines), tuning and telemetry.
#[allow(clippy::too_many_arguments)]
pub fn negotiate_connections(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    connections: &[Connection],
    global: &GlobalResult,
    grid: Option<&GCellGrid>,
    settings: &NegotiationSettings,
    tuning: &NegotiationTuning,
    backend: &mut dyn RouteCompute,
    telemetry: &mut Telemetry,
) -> DetailResult {
    let start = Instant::now();
    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(settings.threads)
        .build()
        .expect("a rayon pool builds");
    telemetry.threads = pool.current_num_threads() as u32;
    let t_setup = Instant::now();
    let ctx = Ctx::new(board, rules, *tuning);
    let (tasks, refused) = build_tasks(&ctx, connections, global, grid);
    telemetry.count("us_setup", t_setup.elapsed().as_micros() as u64);
    let n = tasks.len();
    let all_layers = board.layers.all();
    let reach = ctx.obstacles.reach;

    let mut routed: Vec<Option<Vec<Arc<PieceCopper>>>> = vec![None; n];
    let mut failure: Vec<Option<DetailFailure>> = vec![None; n];
    let mut failed_in: Vec<[bool; 2]> = vec![[false; 2]; n];
    let mut history = HistoryMap::default();
    let mut queue: Vec<u32> = (0..n as u32).collect();
    let mut present_factor = tuning.initial_present_factor;
    let mut reports = Vec::new();
    let mut against: BTreeMap<u32, BTreeSet<u32>> = BTreeMap::new();
    let mut iterations = 0;
    let mut fields = Vec::new();
    let chunk_size = if tuning.chunk_tiles > 0 {
        tuning.chunk_tiles
    } else {
        16 * pool.current_num_threads().max(1)
    };

    for it in 0..settings.max_iterations {
        if queue.is_empty() || ctx.slots.is_empty() {
            break;
        }
        let it_start = Instant::now();
        iterations = it + 1;
        let shift = tuning.shift_odd_iterations && it % 2 == 1;
        let Some(tiling) = Tiling::build(board, shift, &ctx.tuning, ctx.margin) else {
            for &t in &queue {
                failure[t as usize] = Some(DetailFailure::NoCorridor);
            }
            break;
        };
        if it == 0 {
            fields = tiling.fields.clone();
        }
        let mut report = IterationReport {
            iteration: it,
            shifted: shift,
            present_factor,
            queued: queue.len() as u32,
            ..IterationReport::default()
        };
        // 1. Rip up.
        for &t in &queue {
            if routed[t as usize].take().is_some() {
                telemetry.count("rip_ups", 1);
            }
            failure[t as usize] = None;
        }
        // 2. Cut into pieces, pinning crossings away from routed copper.
        let t_split = Instant::now();
        let kept: Vec<Arc<PieceCopper>> = routed.iter().flatten().flatten().cloned().collect();
        let kept_index = RoutedIndex::build(kept, board.layer_count(), reach);
        let mut registry = PinRegistry::default();
        let mut by_tile: Vec<Vec<PieceJob>> = vec![Vec::new(); tiling.tiles.len()];
        let mut fresh: Vec<Vec<Option<Arc<PieceCopper>>>> = vec![Vec::new(); n];
        for &t in &queue {
            match pieces::split(
                &ctx,
                &tiling,
                &tasks[t as usize],
                t,
                &mut registry,
                &kept_index,
                &history,
            ) {
                Ok(list) => {
                    fresh[t as usize] = vec![None; list.len()];
                    report.pieces += list.len() as u32;
                    for (tile, job) in list {
                        by_tile[tile].push(job);
                    }
                }
                Err(f) => failure[t as usize] = Some(f),
            }
        }
        drop(kept_index);
        telemetry.count("us_split", t_split.elapsed().as_micros() as u64);
        report.pins = registry.placed as u32;
        telemetry.count("pins", registry.placed);
        telemetry.count("pieces", u64::from(report.pieces));

        // 3. Route, pass by pass.
        let params = CostParams {
            via: tuning.via_cost,
            present_factor,
            history_factor: tuning.history_factor,
            history_increment: settings.history_increment,
            near_step: tuning.near_step,
            ..CostParams::default()
        };
        let forks: Option<Vec<Box<dyn RouteCompute + Send>>> = if ctx.tuning.lockstep_rounds {
            None
        } else {
            (0..pool.current_num_threads().max(1))
                .map(|_| backend.fork())
                .collect()
        };
        let record = |done: Vec<PieceDone>,
                      fresh: &mut Vec<Vec<Option<Arc<PieceCopper>>>>,
                      failure: &mut Vec<Option<DetailFailure>>| {
            for d in done {
                match d.result {
                    Ok(p) => fresh[d.task as usize][d.piece as usize] = Some(p),
                    Err(f) => {
                        let slot = &mut failure[d.task as usize];
                        if slot.is_none() {
                            *slot = Some(f);
                        }
                    }
                }
            }
        };
        if let Some(forks) = forks {
            // Dependency-ordered tiles, no pass barriers.
            let order: Vec<usize> = tiling
                .passes()
                .into_iter()
                .flatten()
                .filter(|&t| !by_tile[t].is_empty())
                .collect();
            let kept: Vec<Arc<PieceCopper>> = routed.iter().flatten().flatten().cloned().collect();
            let done = run_dag(
                &ctx,
                &tiling,
                &order,
                &mut by_tile,
                &tasks,
                &kept,
                &history,
                &params,
                it,
                forks,
                telemetry,
            );
            record(done, &mut fresh, &mut failure);
        } else {
            // Pass by pass, each in lockstep batch rounds.
            for pass in tiling.passes() {
                let active: Vec<usize> = pass
                    .into_iter()
                    .filter(|&t| !by_tile[t].is_empty())
                    .collect();
                if active.is_empty() {
                    continue;
                }
                let store: Vec<Arc<PieceCopper>> = routed
                    .iter()
                    .flatten()
                    .flatten()
                    .chain(fresh.iter().flatten().flatten())
                    .cloned()
                    .collect();
                for chunk in active.chunks(chunk_size) {
                    let jobs: Vec<Vec<PieceJob>> = chunk
                        .iter()
                        .map(|&t| std::mem::take(&mut by_tile[t]))
                        .collect();
                    let done = run_chunk(
                        &ctx, &tiling, chunk, jobs, &tasks, &store, &history, &params, it, backend,
                        &pool, telemetry,
                    );
                    record(done, &mut fresh, &mut failure);
                }
            }
        }

        // Assemble the queued connections.
        let mut is_new = vec![false; n];
        for &t in &queue {
            let t = t as usize;
            if failure[t].is_some() {
                report.failed += 1;
                continue;
            }
            let pieces: Option<Vec<Arc<PieceCopper>>> = fresh[t].iter().cloned().collect();
            match pieces {
                Some(p) if !p.is_empty() => {
                    routed[t] = Some(p);
                    is_new[t] = true;
                    report.routed += 1;
                }
                _ => {
                    failure[t] = Some(DetailFailure::Unreachable);
                    report.failed += 1;
                }
            }
        }

        // 4. Conflicts and history.
        let t_conf = Instant::now();
        let all: Vec<Arc<PieceCopper>> = routed.iter().flatten().flatten().cloned().collect();
        let index = RoutedIndex::build(all, board.layer_count(), reach);
        let pairs = pool.install(|| {
            index.conflicts(rules, all_layers, &|p: &PieceCopper| {
                is_new[p.task as usize]
            })
        });
        telemetry.count("us_conflicts", t_conf.elapsed().as_micros() as u64);
        report.conflict_pairs = pairs.len() as u32;
        against.clear();
        let mut cells = HashSet::new();
        for &pair in &pairs {
            let (ta, tb) = index.tasks_of(pair);
            against.entry(ta).or_default().insert(tb);
            against.entry(tb).or_default().insert(ta);
            index.conflict_cells(rules, &ctx.slots, pair.0, pair.1, &mut cells);
            index.conflict_cells(rules, &ctx.slots, pair.1, pair.0, &mut cells);
        }
        for (l, cx, cy) in cells {
            history.bump(l, cx, cy, settings.history_increment);
        }
        let (hc, ht) = history.stats();
        report.history_cells = hc;
        report.history_total = ht;
        report.conflicted = against.len() as u32;
        telemetry.count("conflict_pairs", pairs.len() as u64);

        // Next queue: both sides of every conflict, and retryable failures.
        let mut next: BTreeSet<u32> = against.keys().copied().collect();
        for &t in &queue {
            let Some(f) = &failure[t as usize] else {
                continue;
            };
            // A failure the other tiling might avoid is retried until the
            // connection has failed in both tilings.
            let seen = &mut failed_in[t as usize];
            seen[(it % 2) as usize] = true;
            if f.retryable() && !(seen[0] && seen[1]) {
                next.insert(t);
            }
        }
        let elapsed = it_start.elapsed().as_micros() as u64;
        if !pairs.is_empty() {
            telemetry.congested_iteration_us += elapsed;
        }
        reports.push(report);
        queue = next.into_iter().collect();
        // PathFinder schedule: multiply, but never below the floor (so a
        // blind first iteration still leads to negotiation).
        present_factor =
            u32::try_from(u64::from(present_factor) * u64::from(settings.present_growth) / 1000)
                .unwrap_or(u32::MAX)
                .max(tuning.present_factor_floor);
    }

    // Unresolved conflicts: keep greedily in commit order.
    let mut kept_conflicted: BTreeSet<u32> = BTreeSet::new();
    for (&t, partners) in &against {
        if routed[t as usize].is_none() {
            continue;
        }
        if partners.contains(&t) || partners.iter().any(|p| kept_conflicted.contains(p)) {
            routed[t as usize] = None;
            failure[t as usize] = Some(DetailFailure::Congestion {
                iterations,
                against: partners
                    .iter()
                    .map(|&p| tasks[p as usize].conn.id)
                    .collect(),
            });
        } else {
            kept_conflicted.insert(t);
        }
    }

    // Assemble the result.
    let gen = tuning.generation;
    let mut result = DetailResult {
        iterations,
        generation: gen,
        reports,
        bga_fields: fields,
        ..DetailResult::default()
    };
    let mut failed: Vec<(ConnectionId, DetailFailure)> =
        refused.into_iter().map(|(c, f)| (c.id, f)).collect();
    for (t, task) in tasks.iter().enumerate() {
        match &routed[t] {
            Some(pieces) => {
                result.routed.push(task.conn.id);
                for p in pieces {
                    for s in &p.segs {
                        result.copper.tracks.push(RoutedTrack {
                            net: p.net,
                            layer: s.layer,
                            seg: s.seg,
                            gen,
                        });
                    }
                    for v in &p.vias {
                        result.copper.vias.push(RoutedVia {
                            net: p.net,
                            center: v.center,
                            diameter: v.diameter,
                            drill: v.drill,
                            layers: all_layers,
                            gen,
                        });
                    }
                }
            }
            None => failed.push((
                task.conn.id,
                failure[t].clone().unwrap_or(DetailFailure::Unreachable),
            )),
        }
    }
    result.routed.sort_unstable();
    failed.sort_by_key(|(c, _)| *c);
    for (_, f) in &failed {
        telemetry.count(&format!("failed_{}", f.name()), 1);
    }
    result.failed = failed.iter().map(|(c, f)| (*c, f.reason())).collect();
    result.failure_details = failed;
    telemetry.count("connections", connections.len() as u64);
    telemetry.count("routed", result.routed.len() as u64);
    telemetry.add_stage_time(Stage::Detail, start.elapsed().as_micros() as u64);
    result
}
