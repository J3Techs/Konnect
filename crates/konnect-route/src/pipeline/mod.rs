//! The whole native route, stage by stage (owned by INT).
//!
//! [`route_board`] runs, in order:
//!
//! | Step | Stage | Module |
//! |---|---|---|
//! | 1 | S1 connectivity: the open connections | [`crate::connectivity`] |
//! | 2 | S2 plane fanout: a via (and stub) per unjoined plane pad | [`crate::fanout`] |
//! | 3 | S1 connectivity again, over the board with the fanout, then the scope | — |
//! | 4 | S4 pin access per net class, for the selected connections' pads | [`crate::access`] |
//! | 5 | S3 global routing over the GCell grid | [`crate::global`] |
//! | 6 | S5 negotiated detailed routing on WS-D1's kernel | [`crate::detail`] |
//! | 7 | S6 optimizer, then a guard against the S7 checker | [`crate::optimize`] |
//! | 8 | S7 exact check of all new copper against the board | [`crate::check`] |
//! | 9 | S8 the `add_routing` plan | [`crate::plan`] |
//!
//! Ingest (S0) is the caller's: [`route_board`] takes a loaded
//! [`RouteBoard`] and its [`RuleTable`].
//!
//! # Hand-offs
//!
//! - **Fanout copper** is added to a working copy of the board as locked
//!   copper before anything else runs, so global capacity, negotiation and
//!   the optimizer see it as an obstacle and a terminal, and the remaining
//!   connections are recomputed against it (a plane pad joined by its via
//!   needs no route). It is part of the plan; it is never optimized.
//! - **Global to detail.** Negotiation reads the corridors through the
//!   same connection list global routing was given (ids are positions in
//!   it). A connection global routing could not route still goes to
//!   detailed routing, with an empty corridor on its class layers, so a
//!   coarse-grid failure (a terminal off the grid, a cell with no capacity)
//!   is not final.
//! - **Optimizer guard.** The optimizer's own exact gate is WS-E's interim
//!   checker. After it runs, the S7 checker compares each net's violations
//!   before and after; a net that gained any is put back to its routed
//!   copper, repeating until no net gains (putting one net back can expose
//!   another's edit). So the optimizer can only remove S7 findings.
//! - **Check.** The final check covers the fanout and the routed copper
//!   together, against the board as loaded.
//!
//! Every stage records its wall time in the returned [`Telemetry`].

pub mod backend;

pub use backend::KernelBackend;

use crate::access::{compute_pin_access_with, AccessSettings, AccessStats, PadAccess};
use crate::api::{
    ClassId, Connection, ConnectionId, FailureReason, ItemRef, LayerId, LayerMask, NetId,
    Ownership, PadId, ProfileId, Stage, TrackId, ViaId,
};
use crate::board::{Keepout, KeepoutRules};
use crate::board::{RouteBoard, Track, Via};
use crate::check::{BoardChecker, CheckOptions, CheckReport, CheckedItem};
use crate::compute::RouteCompute;
use crate::connectivity::ConnectivityReport;
use crate::detail::negotiate::{
    negotiate_connections_with, DetailResult, NeckDown, NegotiationInputs, NegotiationSettings,
    NegotiationTuning,
};
use crate::detail::search::SearchConfig;
use crate::fanout::{plan_plane_vias, FanoutSettings, PlaneViaPlan};
use crate::global::{
    build_grid_with, route_global_with, Corridor, GCellGrid, GlobalResult, GlobalSettings,
    LayerPolicy,
};
use crate::optimize::exact::ExactChecker;
use crate::optimize::{optimize_checked, record_counters, OptimizeOptions, OptimizeReport};
use crate::plan::{PlanError, RoutePlan, RoutedCopper, RoutedTrack, RoutedVia};
use crate::rules::RuleTable;
use crate::telemetry::Telemetry;
use konnect_geom::{shape_within_polygon, ArcPolicy, BBox, Coord, Polygon, Segment, Shape};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::time::Instant;

/// Which connections a run works on. Fanout sites and connections outside
/// the scope are left alone; their copper stays an obstacle.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct PipelineScope {
    /// Only these nets (all when `None`).
    pub nets: Option<BTreeSet<NetId>>,
    /// Only connections whose two terminals lie wholly inside this box, and
    /// fanout sites whose copper does (anywhere when `None`).
    pub region: Option<BBox>,
}

/// Every stage's settings. `None` skips an optional stage.
#[derive(Debug, Clone)]
pub struct PipelineSettings {
    /// S2 plane fanout; `None` routes plane pads like any other connection.
    pub fanout: Option<FanoutSettings>,
    /// S4 pin access; `None` skips it. Negotiation enters a pad whose own
    /// copper offers no open lattice node through a stub from one of its
    /// access points ([`Self::use_access`]).
    pub access: Option<AccessSettings>,
    /// Hand S4's access points to negotiation.
    pub use_access: bool,
    /// Neck-down at pads for every class not excluded (`None`: never).
    pub neck_down: Option<NeckDown>,
    /// A second chance on more layers, per class: connections of the class
    /// still failed after negotiation are routed again on
    /// [`ClassFallback::layers`], around all copper routed so far.
    pub class_fallback: BTreeMap<ClassId, ClassFallback>,
    pub global: GlobalSettings,
    pub layer_policy: LayerPolicy,
    pub negotiation: NegotiationSettings,
    pub tuning: NegotiationTuning,
    /// The detailed search kernel's tuning.
    pub search: SearchConfig,
    /// Ask the kernel for its minimum run between turns on every job.
    pub search_min_run: bool,
    /// S6; `None` keeps the routed copper as negotiation drew it.
    pub optimize: Option<OptimizeOptions>,
    /// Put back any net the optimizer left with more S7 findings.
    pub guard_optimize: bool,
    /// Worker threads for the whole run (0: the current rayon pool). Also
    /// the negotiation thread count when that is 0.
    pub threads: usize,
}

impl Default for PipelineSettings {
    fn default() -> Self {
        PipelineSettings {
            fanout: Some(FanoutSettings::default()),
            access: Some(AccessSettings::default()),
            use_access: true,
            neck_down: None,
            class_fallback: BTreeMap::new(),
            global: GlobalSettings::default(),
            layer_policy: LayerPolicy::default(),
            // Measured on a dense real board: a 2x present-factor growth
            // (capped) and a history step of 3 route more than the gentler
            // stand-alone schedule; 20 iterations add little for twice the time.
            negotiation: NegotiationSettings {
                max_iterations: 10,
                present_growth: 2_000,
                history_increment: 3,
                ..NegotiationSettings::default()
            },
            tuning: NegotiationTuning::default(),
            search: KernelBackend::negotiation_config(),
            search_min_run: false,
            optimize: Some(OptimizeOptions::default()),
            guard_optimize: true,
            threads: 0,
        }
    }
}

/// Extra layers a class may use when its own layers cannot route a
/// connection.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ClassFallback {
    /// The layers of the second chance (the class's own included).
    pub layers: LayerMask,
    /// Tracks on some of those layers must run over solid copper of a
    /// reference plane.
    #[serde(default)]
    pub reference: Option<ReferencePlane>,
}

/// Solid reference copper a class's tracks must stay over.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReferencePlane {
    /// The plane layer whose filled copper is the reference.
    pub plane: LayerId,
    /// The routing layers this applies to.
    pub over: LayerMask,
    /// How far inside one filled piece's outline a track's copper must
    /// stay (its edge, not its centre line).
    pub margin: Coord,
}

/// What a run produced, stage by stage.
#[derive(Debug, Clone, Default)]
pub struct PipelineResult {
    /// The fanout and routed copper as one `add_routing` plan.
    pub plan: RoutePlan,
    /// S2 copper (apply locked).
    pub fanout_copper: RoutedCopper,
    /// S5 copper after S6.
    pub routed: RoutedCopper,
    /// Open connections before the fanout, over the whole board.
    pub connections_before_fanout: usize,
    /// The task list global and detailed routing worked on (after the
    /// fanout, within the scope); ids are positions in it.
    pub connections: Vec<Connection>,
    pub fanout: Option<PlaneViaPlan>,
    pub access: Vec<PadAccess>,
    /// Per net class (as a profile), what pin access did.
    pub access_stats: Vec<(ProfileId, AccessStats)>,
    pub grid: Option<GCellGrid>,
    pub global: GlobalResult,
    /// Negotiation's account; its copper is moved to [`Self::routed`].
    pub detail: DetailResult,
    pub optimize: Option<OptimizeReport>,
    /// Nets the guard put back to their routed copper.
    pub optimize_reverted: Vec<NetId>,
    /// S7 over the fanout and routed copper together (indices follow
    /// [`Self::all_copper`]).
    pub check: CheckReport,
    /// Connections not routed, and why (same order as negotiation's).
    pub failed: Vec<(ConnectionId, FailureReason)>,
    /// Connections still open on the board with the plan applied.
    pub connections_after: usize,
    pub telemetry: Telemetry,
}

impl PipelineResult {
    /// The fanout copper followed by the routed copper: what the plan holds
    /// and what [`Self::check`] indexes.
    pub fn all_copper(&self) -> RoutedCopper {
        concat(&self.fanout_copper, &self.routed)
    }
}

#[derive(Debug, thiserror::Error)]
pub enum PipelineError {
    #[error("plan: {0}")]
    Plan(#[from] PlanError),
}

/// Route `board` with WS-D1's kernel as the detailed search backend.
pub fn route_board(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    scope: &PipelineScope,
    settings: &PipelineSettings,
) -> Result<PipelineResult, PipelineError> {
    let mut backend = KernelBackend {
        config: settings.search.clone(),
        min_run: settings.search_min_run,
    };
    route_board_with(board, rules, scope, settings, &mut backend)
}

/// [`route_board`] on any detailed search backend.
pub fn route_board_with(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    scope: &PipelineScope,
    settings: &PipelineSettings,
    backend: &mut (dyn RouteCompute + Send),
) -> Result<PipelineResult, PipelineError> {
    if settings.threads == 0 {
        return run(board, rules, scope, settings, backend);
    }
    match rayon::ThreadPoolBuilder::new()
        .num_threads(settings.threads)
        .build()
    {
        Ok(pool) => pool.install(|| run(board, rules, scope, settings, backend)),
        Err(_) => run(board, rules, scope, settings, backend),
    }
}

fn run(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    scope: &PipelineScope,
    settings: &PipelineSettings,
    backend: &mut (dyn RouteCompute + Send),
) -> Result<PipelineResult, PipelineError> {
    let mut out = PipelineResult::default();
    let mut telemetry = Telemetry::default();
    let tel = &mut telemetry;
    tel.threads = rayon::current_num_threads() as u32;
    let policy = ArcPolicy::default();

    // 1. Connectivity of the board as loaded.
    let t = Instant::now();
    let before = ConnectivityReport::build(board, &policy);
    out.connections_before_fanout = before.connection_count();
    tel.add_stage_time(Stage::Audit, t.elapsed().as_micros() as u64);

    // 2. Plane fanout, kept to the scope, frozen into a working board.
    let mut work = board.clone();
    if let Some(fs) = &settings.fanout {
        let t = Instant::now();
        let mut plan = plan_plane_vias(board, rules, &before, fs);
        plan.sites.retain(|s| {
            let net_ok = scope.nets.as_ref().is_none_or(|n| n.contains(&s.net));
            let r = s.via_diameter / 2 + 1;
            let mut b = BBox::point(s.via).inflate(r);
            if let Some((a, e, w)) = s.stub {
                b = b.union(BBox::from_points([a, e]).map_or(b, |x| x.inflate(w / 2 + 1)));
            }
            net_ok && scope.region.is_none_or(|reg| reg.contains_bbox(&b))
        });
        let mut copper = plan.routed();
        let all = board.layers.all();
        for v in &mut copper.vias {
            v.layers = all;
        }
        add_locked(&mut work, &copper);
        tel.add_stage_time(Stage::Fanout, t.elapsed().as_micros() as u64);
        tel.count("pipeline_fanout_sites", plan.sites.len() as u64);
        tel.count("pipeline_fanout_failed", plan.failed.len() as u64);
        out.fanout_copper = copper;
        out.fanout = Some(plan);
    }

    // 3. The task list: connections still open, within the scope.
    let t = Instant::now();
    let conn = if out.fanout_copper.tracks.is_empty() && out.fanout_copper.vias.is_empty() {
        before
    } else {
        ConnectivityReport::build(&work, &policy)
    };
    out.connections = select_connections(
        &work,
        conn.connections(&work),
        scope.nets.as_ref(),
        scope.region,
    );
    drop(conn);
    tel.add_stage_time(Stage::Audit, t.elapsed().as_micros() as u64);
    tel.count("pipeline_connections", out.connections.len() as u64);

    // 4. Pin access, one profile per net class of the selected nets.
    if let Some(access) = &settings.access {
        let t = Instant::now();
        let mut by_class: BTreeMap<ClassId, BTreeSet<PadId>> = BTreeMap::new();
        for c in &out.connections {
            let class = rules.class_of(Some(c.net));
            for end in [c.from, c.to] {
                if let ItemRef::Pad(p) = end {
                    by_class.entry(class).or_default().insert(p);
                }
            }
        }
        for (class, pads) in by_class {
            let pads: Vec<PadId> = pads.into_iter().collect();
            let profile = ProfileId(class.0);
            let (points, stats) = compute_pin_access_with(&work, rules, &pads, profile, access);
            tel.count("pipeline_access_no_access", stats.no_access as u64);
            out.access.extend(points);
            out.access_stats.push((profile, stats));
        }
        tel.add_stage_time(Stage::Access, t.elapsed().as_micros() as u64);
    }

    // 5. Global routing.
    let gs = settings.global;
    let grid = build_grid_with(&work, rules, &gs, tel);
    let mut global = route_global_with(
        &work,
        rules,
        &grid,
        &out.connections,
        &gs,
        &settings.layer_policy,
        tel,
    );
    tel.count("pipeline_global_failed", global.failed.len() as u64);

    // 6. Detailed routing. A connection with no corridor still gets a try,
    // unconstrained on its class layers.
    let detail_global = with_fallback_corridors(
        &work,
        rules,
        &settings.layer_policy,
        &out.connections,
        &global,
    );
    let mut negotiation = settings.negotiation;
    if negotiation.threads == 0 && settings.threads > 0 {
        negotiation.threads = settings.threads;
    }
    let inputs = NegotiationInputs {
        access: if settings.use_access {
            std::mem::take(&mut out.access)
        } else {
            Vec::new()
        },
        neck_down: settings.neck_down.clone(),
    };
    let mut detail = negotiate_connections_with(
        &work,
        rules,
        &out.connections,
        &detail_global,
        Some(&grid),
        &negotiation,
        &settings.tuning,
        &inputs,
        backend,
        tel,
    );
    drop(detail_global);
    let mut routed = std::mem::take(&mut detail.copper);
    let all = work.layers.all();
    for v in &mut routed.vias {
        if v.layers.is_empty() {
            v.layers = all;
        }
    }
    if !settings.class_fallback.is_empty() {
        let t = Instant::now();
        let rescued = second_chance(
            &work,
            rules,
            &out.connections,
            &mut detail,
            &mut routed,
            settings,
            &negotiation,
            &inputs,
            backend,
        );
        tel.count("pipeline_fallback_routed", rescued as u64);
        tel.add_stage_time(Stage::Detail, t.elapsed().as_micros() as u64);
    }
    if settings.use_access {
        out.access = inputs.access;
    }

    // 7. Optimize, guarded by the S7 checker.
    if let Some(opts) = &settings.optimize {
        let t = Instant::now();
        let checker = ExactChecker::new(&work, rules, opts.checker);
        let original = routed.clone();
        let report = optimize_checked(&checker, &mut routed, opts);
        record_counters(&report, tel);
        tel.add_stage_time(Stage::Optimize, t.elapsed().as_micros() as u64);
        if settings.guard_optimize {
            let t = Instant::now();
            out.optimize_reverted = guard(&work, rules, &original, &mut routed);
            tel.count(
                "pipeline_optimize_reverted",
                out.optimize_reverted.len() as u64,
            );
            tel.add_stage_time(Stage::Check, t.elapsed().as_micros() as u64);
        }
        out.optimize = Some(report);
    }

    // 8. Check everything new against the board as loaded.
    let combined = concat(&out.fanout_copper, &routed);
    let checker = BoardChecker::new(board, rules, CheckOptions::for_routed());
    out.check = checker.check_routed_timed(&combined, tel);

    // 9. Plan, and what is left open with it applied.
    let t = Instant::now();
    if !combined.tracks.is_empty() || !combined.vias.is_empty() {
        out.plan = RoutePlan::from_routed(board, &combined)?;
    }
    let mut applied = board.clone();
    add_locked(&mut applied, &combined);
    out.connections_after = ConnectivityReport::build(&applied, &policy).connection_count();
    tel.add_stage_time(Stage::Plan, t.elapsed().as_micros() as u64);

    global.corridors.shrink_to_fit();
    out.failed = detail.failed.clone();
    out.grid = Some(grid);
    out.global = global;
    out.detail = detail;
    out.routed = routed;
    out.telemetry = telemetry;
    Ok(out)
}

/// The open connections a run works on: those of `nets` (all when `None`)
/// whose two terminals both lie inside `region` (anywhere when `None`).
/// Ids are renumbered to stay equal to their index.
pub fn select_connections(
    board: &RouteBoard,
    connections: Vec<Connection>,
    nets: Option<&BTreeSet<NetId>>,
    region: Option<BBox>,
) -> Vec<Connection> {
    let inside = |item: ItemRef| {
        let Some(r) = region else { return true };
        let bbox = match item {
            ItemRef::Pad(id) => board.pads[id.index()].bbox(),
            ItemRef::Via(id) => Some(board.vias[id.index()].shape().bbox()),
            ItemRef::Track(id) => Some(board.tracks[id.index()].seg.bbox()),
            ItemRef::Arc(id) => {
                let a = &board.arcs[id.index()];
                BBox::from_points([a.start, a.mid, a.end])
            }
            ItemRef::Zone(_) => None,
        };
        bbox.is_some_and(|b| r.contains_bbox(&b))
    };
    connections
        .into_iter()
        .filter(|c| nets.is_none_or(|s| s.contains(&c.net)) && inside(c.from) && inside(c.to))
        .enumerate()
        .map(|(i, c)| Connection {
            id: ConnectionId(i as u32),
            ..c
        })
        .collect()
}

/// Route again, on their class's fallback layers, the connections of
/// classes with a [`ClassFallback`] that negotiation failed, around every
/// copper routed so far (made fixed). A rescued connection's copper is
/// appended to `routed` and it moves from failed to routed in `detail`.
/// Where the fallback names a reference plane, tracks on the layers it
/// covers are kept over one filled piece of the plane (a keepout outside
/// the pieces guides the search; a net whose tracks still leave a piece by
/// less than the margin is dropped again). Returns how many were rescued.
#[allow(clippy::too_many_arguments)]
fn second_chance(
    work: &RouteBoard,
    rules: &dyn RuleTable,
    connections: &[Connection],
    detail: &mut DetailResult,
    routed: &mut RoutedCopper,
    settings: &PipelineSettings,
    negotiation: &NegotiationSettings,
    inputs: &NegotiationInputs,
    backend: &mut (dyn RouteCompute + Send),
) -> usize {
    let mut rescued_total = 0;
    for (&class, fallback) in &settings.class_fallback {
        let ids: Vec<ConnectionId> = detail
            .failure_details
            .iter()
            .filter(|(id, _)| {
                connections
                    .get(id.index())
                    .is_some_and(|c| rules.class_of(Some(c.net)) == class)
            })
            .map(|(id, _)| *id)
            .collect();
        if ids.is_empty() {
            continue;
        }
        let mut board = work.clone();
        add_locked(&mut board, routed);
        let pieces: Vec<Polygon> = fallback
            .reference
            .map(|r| reference_pieces(work, r.plane))
            .unwrap_or_default();
        if let Some(r) = fallback.reference {
            let areas: Vec<BBox> = ids
                .iter()
                .filter_map(|id| {
                    let c = &connections[id.index()];
                    let pts = [c.from, c.to].map(|i| terminal_point(work, i));
                    BBox::from_points(pts.into_iter().flatten())
                })
                .map(|b| b.inflate(REFERENCE_AREA_NM))
                .collect();
            let keepouts = outside_keepouts(&pieces, &areas, r.over, REFERENCE_CELL_NM);
            for mut k in keepouts {
                k.id = crate::api::KeepoutId(board.keepouts.len() as u32);
                board.keepouts.push(k);
            }
        }
        let tasks: Vec<Connection> = ids
            .iter()
            .enumerate()
            .map(|(i, id)| Connection {
                id: ConnectionId(i as u32),
                ..connections[id.index()]
            })
            .collect();
        let layers = fallback.layers.intersection(board.routable);
        let global = GlobalResult {
            corridors: tasks
                .iter()
                .map(|c| Corridor {
                    connection: c.id,
                    cells: Vec::new(),
                    layers,
                    assigned: LayerMask::EMPTY,
                })
                .collect(),
            ..GlobalResult::default()
        };
        let mut tel = Telemetry::default();
        let mut second = negotiate_connections_with(
            &board,
            rules,
            &tasks,
            &global,
            None,
            negotiation,
            &settings.tuning,
            inputs,
            backend,
            &mut tel,
        );
        // Nets whose new tracks leave the reference by less than the margin.
        let mut dropped: BTreeSet<NetId> = BTreeSet::new();
        if let Some(r) = fallback.reference {
            for t in &second.copper.tracks {
                if r.over.contains(t.layer) && !over_reference(&t.seg, &pieces, r.margin) {
                    dropped.insert(t.net);
                }
            }
        }
        second.copper.tracks.retain(|t| !dropped.contains(&t.net));
        second.copper.vias.retain(|v| !dropped.contains(&v.net));
        let all = work.layers.all();
        for t in second.copper.tracks {
            routed.tracks.push(t);
        }
        for mut v in second.copper.vias {
            if v.layers.is_empty() {
                v.layers = all;
            }
            routed.vias.push(v);
        }
        let rescued: BTreeSet<ConnectionId> = second
            .routed
            .iter()
            .map(|c| ids[c.index()])
            .filter(|id| !dropped.contains(&connections[id.index()].net))
            .collect();
        rescued_total += rescued.len();
        detail.failed.retain(|(id, _)| !rescued.contains(id));
        detail
            .failure_details
            .retain(|(id, _)| !rescued.contains(id));
        detail.routed.extend(rescued.iter().copied());
        detail.routed.sort_unstable();
    }
    rescued_total
}

/// The filled copper pieces of `plane`. KiCad's fills are fractured (each
/// void joined to the outline by a zero-width slit), so an outline already
/// excludes antipads, pad clearances and voids: a track over any of them
/// counts as off the reference, which is stricter than "over the pour".
fn reference_pieces(board: &RouteBoard, plane: LayerId) -> Vec<Polygon> {
    board
        .zones
        .iter()
        .filter(|z| z.filled && z.layer == plane && z.net.is_some())
        .filter_map(|z| Polygon::new(z.region.outline().to_vec(), Vec::new()))
        .collect()
}

/// How far around a fallback connection's terminals the reference is
/// enforced as keepouts (beyond it, only the final check applies).
const REFERENCE_AREA_NM: Coord = 5_000_000;
/// Cell size of the reference keepouts.
const REFERENCE_CELL_NM: Coord = 250_000;

fn terminal_point(board: &RouteBoard, item: ItemRef) -> Option<konnect_geom::Point> {
    match item {
        ItemRef::Pad(id) => board.pads.get(id.index()).map(|p| p.center),
        ItemRef::Via(id) => board.vias.get(id.index()).map(|v| v.center),
        ItemRef::Track(id) => board.tracks.get(id.index()).map(|t| t.seg.a),
        _ => None,
    }
}

/// Track keepouts on `layers` covering every `cell`-sized square within
/// `areas` that does not lie wholly inside one piece: one rectangle per
/// run of such cells along a row, so each is cheap to paint and check.
fn outside_keepouts(
    pieces: &[Polygon],
    areas: &[BBox],
    layers: LayerMask,
    cell: Coord,
) -> Vec<Keepout> {
    let c = i64::from(cell);
    let mut cells: Vec<(i64, i64)> = Vec::new();
    for a in areas {
        let (x0, x1) = (
            i64::from(a.min.x).div_euclid(c),
            i64::from(a.max.x).div_euclid(c),
        );
        let (y0, y1) = (
            i64::from(a.min.y).div_euclid(c),
            i64::from(a.max.y).div_euclid(c),
        );
        for iy in y0..=y1 {
            for ix in x0..=x1 {
                cells.push((iy, ix));
            }
        }
    }
    cells.sort_unstable();
    cells.dedup();
    let square = |iy: i64, ix: i64| {
        let p = |x: i64, y: i64| konnect_geom::Point::new((x * c) as Coord, (y * c) as Coord);
        vec![p(ix, iy), p(ix + 1, iy), p(ix + 1, iy + 1), p(ix, iy + 1)]
    };
    let outside = |iy: i64, ix: i64| {
        let Some(sq) = konnect_geom::ConvexPoly::new(square(iy, ix), 0) else {
            return true;
        };
        let sq = Shape::ConvexPoly(sq);
        let b = sq.bbox();
        !pieces
            .iter()
            .any(|p| p.bbox().contains_bbox(&b) && shape_within_polygon(&sq, p))
    };
    let mut out = Vec::new();
    let mut run: Option<(i64, i64, i64)> = None;
    let flush = |run: Option<(i64, i64, i64)>, out: &mut Vec<Keepout>| {
        if let Some((iy, a, b)) = run {
            let p = |x: i64, y: i64| konnect_geom::Point::new((x * c) as Coord, (y * c) as Coord);
            let ring = vec![p(a, iy), p(b + 1, iy), p(b + 1, iy + 1), p(a, iy + 1)];
            if let Some(region) = Polygon::new(ring, Vec::new()) {
                out.push(Keepout {
                    id: crate::api::KeepoutId(0),
                    name: None,
                    layers,
                    region,
                    rules: KeepoutRules {
                        tracks: true,
                        ..KeepoutRules::default()
                    },
                });
            }
        }
    };
    for (iy, ix) in cells {
        if !outside(iy, ix) {
            continue;
        }
        run = match run {
            Some((ry, a, b)) if ry == iy && b + 1 == ix => Some((ry, a, ix)),
            other => {
                flush(other, &mut out);
                Some((iy, ix, ix))
            }
        };
    }
    flush(run, &mut out);
    out
}

/// Whether a track's copper, grown by `margin`, lies inside one piece.
fn over_reference(seg: &Segment, pieces: &[Polygon], margin: Coord) -> bool {
    let grown = Shape::Segment(Segment::new(
        seg.a,
        seg.b,
        seg.width.saturating_add(2 * margin),
    ));
    pieces
        .iter()
        .any(|p| p.bbox().contains_bbox(&grown.bbox()) && shape_within_polygon(&grown, p))
}

/// Append `copper` to `board` as locked copper.
fn add_locked(board: &mut RouteBoard, copper: &RoutedCopper) {
    for t in &copper.tracks {
        board.tracks.push(Track {
            id: TrackId(board.tracks.len() as u32),
            net: Some(t.net),
            layer: t.layer,
            seg: t.seg,
            ownership: Ownership::Locked,
            uuid: None,
        });
    }
    for v in &copper.vias {
        board.vias.push(Via {
            id: ViaId(board.vias.len() as u32),
            net: Some(v.net),
            center: v.center,
            diameter: v.diameter,
            drill: v.drill,
            layers: v.layers,
            ownership: Ownership::Locked,
            uuid: None,
        });
    }
}

fn concat(a: &RoutedCopper, b: &RoutedCopper) -> RoutedCopper {
    RoutedCopper {
        tracks: a.tracks.iter().chain(&b.tracks).copied().collect(),
        vias: a.vias.iter().chain(&b.vias).copied().collect(),
    }
}

/// `global`, plus an empty corridor on the class's layers (every routable
/// layer when the class is unrestricted) for each connection it failed.
fn with_fallback_corridors(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    policy: &LayerPolicy,
    connections: &[Connection],
    global: &GlobalResult,
) -> GlobalResult {
    let mut g = GlobalResult {
        corridors: global.corridors.clone(),
        ..GlobalResult::default()
    };
    if global.failed.is_empty() {
        return g;
    }
    let have: BTreeSet<ConnectionId> = g.corridors.iter().map(|c| c.connection).collect();
    for (id, _) in &global.failed {
        let Some(c) = connections.get(id.index()) else {
            continue;
        };
        if have.contains(id) {
            continue;
        }
        let class = rules.class_of(Some(c.net));
        let layers = policy
            .class_layers
            .get(&class)
            .map_or(board.routable, |m| m.intersection(board.routable));
        g.corridors.push(Corridor {
            connection: *id,
            cells: Vec::new(),
            layers,
            assigned: LayerMask::EMPTY,
        });
    }
    g.corridors.sort_by_key(|c| c.connection);
    g
}

/// Per routed net, the S7 findings that name its copper.
fn findings_per_net(checker: &BoardChecker<'_>, copper: &RoutedCopper) -> BTreeMap<NetId, usize> {
    let report = checker.check_routed(copper);
    let mut out: BTreeMap<NetId, usize> = BTreeMap::new();
    let net_of = |i: CheckedItem| match i {
        CheckedItem::RoutedTrack { index } => copper.tracks.get(index as usize).map(|t| t.net),
        CheckedItem::RoutedVia { index } => copper.vias.get(index as usize).map(|v| v.net),
        _ => None,
    };
    for v in &report.violations {
        let a = net_of(v.a);
        let b = v.b.and_then(net_of);
        if let Some(n) = a {
            *out.entry(n).or_default() += 1;
        }
        if let Some(n) = b.filter(|&n| Some(n) != a) {
            *out.entry(n).or_default() += 1;
        }
    }
    out
}

/// Put back to `original` every net the optimizer left with more S7
/// findings than it had, until none gains. Returns the nets put back.
fn guard(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    original: &RoutedCopper,
    optimized: &mut RoutedCopper,
) -> Vec<NetId> {
    let checker = BoardChecker::new(board, rules, CheckOptions::for_routed());
    let baseline = findings_per_net(&checker, original);
    // Only a net the optimizer edited can be put back: a finding between an
    // edited and an untouched net counts for both, and the edit is the cause.
    let per_net = |c: &RoutedCopper| {
        let mut m: BTreeMap<NetId, (Vec<RoutedTrack>, Vec<RoutedVia>)> = BTreeMap::new();
        for t in &c.tracks {
            m.entry(t.net).or_default().0.push(*t);
        }
        for v in &c.vias {
            m.entry(v.net).or_default().1.push(*v);
        }
        for (ts, vs) in m.values_mut() {
            ts.sort_by_key(|t| (t.layer, t.seg.a, t.seg.b, t.seg.width));
            vs.sort_by_key(|v| (v.center, v.diameter, v.drill));
        }
        m
    };
    let (before, after) = (per_net(original), per_net(optimized));
    let edited: BTreeSet<NetId> = before
        .keys()
        .chain(after.keys())
        .filter(|n| before.get(n) != after.get(n))
        .copied()
        .collect();
    let mut reverted: BTreeSet<NetId> = BTreeSet::new();
    loop {
        let now = findings_per_net(&checker, optimized);
        let worse: Vec<NetId> = now
            .iter()
            .filter(|&(n, &c)| {
                edited.contains(n)
                    && !reverted.contains(n)
                    && c > baseline.get(n).copied().unwrap_or(0)
            })
            .map(|(&n, _)| n)
            .collect();
        if worse.is_empty() {
            break;
        }
        reverted.extend(worse);
        let keep = |n: NetId| !reverted.contains(&n);
        let tracks: Vec<RoutedTrack> = optimized
            .tracks
            .iter()
            .filter(|t| keep(t.net))
            .chain(original.tracks.iter().filter(|t| !keep(t.net)))
            .copied()
            .collect();
        let vias: Vec<RoutedVia> = optimized
            .vias
            .iter()
            .filter(|v| keep(v.net))
            .chain(original.vias.iter().filter(|v| !keep(v.net)))
            .copied()
            .collect();
        *optimized = RoutedCopper { tracks, vias };
    }
    reverted.into_iter().collect()
}

#[cfg(test)]
mod tests;
