//! Rip-up and retry: open the fence around a connection nothing else could
//! route.
//!
//! Negotiation rips up only among the connections of its own run, and the
//! rescue rounds only for a connection that lost a negotiation to a few
//! others. Copper the board already held is a wall to both, locked or not,
//! and so is this run's copper once the rescue rounds are over. A track
//! along a pin row that seals the next pin in therefore leaves that pin's
//! connection open for good. This stage runs after them, on the connections
//! still open within the scope:
//!
//! 1. **Probe.** The open connections are routed on a copy of the board
//!    from which the copper that may be ripped ([`RipUp`]) is taken out
//!    within [`RipUpSettings::probe_radius`] of their pads. The exact
//!    checker then names the copper each probe path collides with on the
//!    real board: those nets block that connection. Blockers are what the
//!    path found actually crosses, never what merely lies near. When the
//!    stage ends, a connection with no probe path is probed again with
//!    every nearby track, arc and via taken out, and if that leaves no path
//!    either with every zone fill out as well, only to report what blocks
//!    it (locked copper, a plane net, a net outside the scope, a zone fill).
//! 2. **Attempt.** The blocking nets are ripped whole: every unlocked
//!    track, arc and via of the net on the board ([`RipUp::Unlocked`]) and
//!    all copper this run routed for it. The stuck connections are routed
//!    first, around everything left; then the ripped nets' connections,
//!    around those. The new copper is put through [`clear_findings`]
//!    against the rest, so what is kept passes the exact check.
//! 3. **Judge.** An attempt is kept only when the whole board then has
//!    fewer open connections than before it; otherwise nothing of it is
//!    kept. When routing the stuck connections first does not pay, the same
//!    rip set is tried once more with the stuck connections and the ripped
//!    nets negotiated as one batch. The first attempt of a round rips for all stuck connections
//!    together; when that does not pay, each is tried alone. A rip set
//!    already tried for the same stuck connections is never tried again, so
//!    two nets cannot keep evicting each other.
//!
//! The state after the stage is therefore the best seen, and never has
//! more open connections than the state before it. Locked copper, pads,
//! zones, keepouts and the plane fanout are never ripped. Nets with a zone
//! are never ripped either (their tracks tie pads to a pour the router
//! does not redraw), nor are nets outside the scope or of excluded classes.
//!
//! Connections of plane nets left to their planes
//! ([`PipelineSettings::plane_tracks`]) are no business of the stage: they
//! are never stuck connections, never routed here, and not counted among
//! the open connections an attempt is judged by. A connection routed again
//! keeps its width: a net's own is the rule table's, and a connection
//! entry ([`PipelineSettings::connection_widths`]) is matched again to the
//! task list of the copper it is routed around.
//!
//! Whole nets, not single tracks: a net ripped in part leaves stubs the
//! router would have to join again exactly, and its remaining copper keeps
//! most of the fence standing.

use super::{
    add_locked, clear_findings, concat, match_connection_widths, open_connection_count,
    route_around, select_connections, split_excluded, split_left_to_planes, PipelineScope,
    PipelineSettings,
};
use crate::access::compute_pin_access_with;
use crate::api::{
    ArcId, ClassId, Connection, ConnectionId, ItemKind, ItemRef, NetId, Ownership, PadId,
    ProfileId, TrackId, ViaId, ZoneId,
};
use crate::board::RouteBoard;
use crate::check::{BoardChecker, CheckOptions, CheckedItem};
use crate::compute::RouteCompute;
use crate::connectivity::ConnectivityReport;
use crate::detail::negotiate::{DetailResult, NegotiationInputs, NegotiationSettings};
use crate::plan::RoutedCopper;
use crate::rules::RuleTable;
use crate::telemetry::Telemetry;
use konnect_geom::{ArcPolicy, BBox, Coord};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

/// Which copper the stage may rip up.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RipUp {
    /// The stage does not run.
    None,
    /// Copper this run routed. Copper the board already held stays a wall.
    #[default]
    Own,
    /// Also the board's unlocked tracks, arcs and vias.
    Unlocked,
}

impl RipUp {
    pub fn is_default(&self) -> bool {
        *self == RipUp::default()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct RipUpSettings {
    pub mode: RipUp,
    /// Rounds at most; a round ends with its first attempt that pays.
    pub max_rounds: u32,
    /// Copper that may be ripped is taken out of the probe board within
    /// this distance of a stuck connection's pads.
    pub probe_radius: Coord,
    /// An attempt never rips more nets than this.
    pub max_nets: usize,
    /// Attempts for one stuck connection alone, per round, after the
    /// attempt for all of them together did not pay.
    pub single_attempts: u32,
}

impl Default for RipUpSettings {
    fn default() -> Self {
        RipUpSettings {
            mode: RipUp::default(),
            max_rounds: 4,
            probe_radius: 2_500_000,
            max_nets: 6,
            single_attempts: 3,
        }
    }
}

/// One board item the stage took out. The plan must delete it.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RippedItem {
    pub uuid: String,
    /// `track`, `arc` or `via`.
    pub kind: ItemKind,
    pub net: String,
    /// The copper layer (a via: its first and last, as `F.Cu-B.Cu`).
    pub layer: String,
    /// The round that ripped it.
    pub round: u32,
    /// Why: the nets whose connections its net blocked.
    pub reason: String,
}

/// One attempt: a set of nets ripped for a set of stuck connections.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct RipUpAttempt {
    pub round: u32,
    /// Nets of the stuck connections routed first.
    pub stuck_nets: Vec<String>,
    /// Nets ripped whole (empty: a plain retry).
    pub ripped_nets: Vec<String>,
    /// Open connections on the whole board before and after the attempt.
    pub open_before: usize,
    pub open_after: usize,
    /// Kept (fewer open connections) or dropped.
    pub accepted: bool,
    /// The same nets were already ripped for the same stuck connections:
    /// not tried again (`open_after` then equals `open_before`).
    #[serde(default)]
    pub repeat: bool,
    /// `stuck_first`: the stuck connections were routed first, then the
    /// ripped nets around them. `together`: tried when that did not pay,
    /// the stuck connections and the ripped nets negotiated as one batch.
    #[serde(default)]
    pub order: String,
    /// Nets within the scope with a connection open after the attempt.
    #[serde(default)]
    pub open_nets_after: Vec<String>,
}

/// What blocks a connection the stage left open.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Blocker {
    pub net: String,
    /// Why the stage did not rip it, or `ripping_did_not_help`:
    /// `locked`, `board_copper_not_requested` (the mode is [`RipUp::Own`]),
    /// `plane_net`, `out_of_scope`, `excluded_class`, `no_uuid`, or
    /// `zone_fill` (another net's filled zone: zones are never ripped; a
    /// pour refilled around the board's tracks walls in what was not
    /// routed before the refill).
    pub reason: String,
    /// The blocking board items' UUIDs (none for this run's own copper).
    pub uuids: Vec<String>,
}

/// A connection still open after the stage.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct StillOpen {
    pub net: String,
    pub from: String,
    pub to: String,
    /// What its probe path collided with. Empty: no path was found even
    /// with every nearby track, arc and via and every zone fill taken out.
    pub blocked_by: Vec<Blocker>,
}

/// The stage's account.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct RipUpReport {
    pub mode: RipUp,
    /// Rounds run (each probes once).
    pub rounds: u32,
    /// Open connections on the whole board before the stage and after it,
    /// not counting those of plane nets left to their planes.
    pub open_before: usize,
    pub open_after: usize,
    /// Board copper taken out: the plan deletes exactly these.
    pub ripped: Vec<RippedItem>,
    /// Nets whose copper (the board's or this run's) was ripped and routed
    /// again in an attempt that was kept.
    pub rerouted_nets: Vec<String>,
    pub attempts: Vec<RipUpAttempt>,
    /// Connections within the scope still open, and what blocks each.
    pub still_open: Vec<StillOpen>,
    /// Why the stage ended: `complete`, `nothing_to_rip`, `no_improvement`,
    /// `repeat` (every remaining rip set was already tried) or `max_rounds`.
    pub stopped: String,
    /// The run was put back to the state before the stage because the
    /// later stages left more connections open than it started with.
    #[serde(default)]
    pub restored: bool,
}

impl RipUpReport {
    /// Whether the stage changed the copper.
    pub fn changed(&self) -> bool {
        self.attempts.iter().any(|a| a.accepted)
    }

    /// The report of a run done again without the stage: what was tried
    /// stays listed, nothing is ripped.
    pub fn restored(&self) -> RipUpReport {
        RipUpReport {
            restored: true,
            open_after: self.open_before,
            ripped: Vec::new(),
            rerouted_nets: Vec::new(),
            ..self.clone()
        }
    }

    /// The `pipeline_rip_up_*` counters.
    pub fn record(&self, tel: &mut Telemetry) {
        tel.count("pipeline_rip_up_rounds", u64::from(self.rounds));
        tel.count("pipeline_rip_up_attempts", self.attempts.len() as u64);
        tel.count(
            "pipeline_rip_up_accepted",
            self.attempts.iter().filter(|a| a.accepted).count() as u64,
        );
        tel.count("pipeline_rip_up_ripped_items", self.ripped.len() as u64);
        tel.count(
            "pipeline_rip_up_joined",
            self.open_before.saturating_sub(self.open_after) as u64,
        );
    }
}

/// `board` without the items of `ripped` (found by UUID), ids renumbered
/// to stay equal to their index: the board as it is once the plan's
/// deletions are applied.
pub fn without_ripped(board: &RouteBoard, ripped: &[RippedItem]) -> RouteBoard {
    let gone: BTreeSet<&str> = ripped.iter().map(|r| r.uuid.as_str()).collect();
    let mut b = board.clone();
    if gone.is_empty() {
        return b;
    }
    let stays = |uuid: &Option<String>| uuid.as_deref().is_none_or(|u| !gone.contains(u));
    b.tracks.retain(|t| stays(&t.uuid));
    b.arcs.retain(|a| stays(&a.uuid));
    b.vias.retain(|v| stays(&v.uuid));
    renumber(&mut b);
    b
}

fn renumber(b: &mut RouteBoard) {
    for (n, t) in b.tracks.iter_mut().enumerate() {
        t.id = TrackId(n as u32);
    }
    for (n, a) in b.arcs.iter_mut().enumerate() {
        a.id = ArcId(n as u32);
    }
    for (n, v) in b.vias.iter_mut().enumerate() {
        v.id = ViaId(n as u32);
    }
}

/// What the stage hands back when it changed anything.
pub(super) struct Outcome {
    pub report: RipUpReport,
    /// The working board (fanout included) without the ripped copper, and
    /// the routed copper, when an attempt was kept.
    pub changed: Option<(RouteBoard, RoutedCopper)>,
}

/// The copper state the stage moves between.
#[derive(Clone)]
struct State {
    board: RouteBoard,
    routed: RoutedCopper,
    ripped: Vec<RippedItem>,
    rerouted: BTreeSet<NetId>,
    /// Open connections on the whole board, and those within the scope.
    open: usize,
    stuck: Vec<Connection>,
}

struct Ctx<'a> {
    rules: &'a dyn RuleTable,
    scope: &'a PipelineScope,
    settings: &'a PipelineSettings,
    negotiation: &'a NegotiationSettings,
    inputs: &'a NegotiationInputs,
}

fn frozen(board: &RouteBoard, routed: &RoutedCopper) -> RouteBoard {
    let mut b = board.clone();
    add_locked(&mut b, routed);
    b
}

/// Open connections of `board` plus `routed`: the whole board's count, and
/// those within the scope and not left out by class. Neither holds a
/// connection of a plane net left to its plane.
fn open_connections(
    ctx: &Ctx<'_>,
    board: &RouteBoard,
    routed: &RoutedCopper,
) -> (usize, Vec<Connection>) {
    let b = frozen(board, routed);
    let report = ConnectivityReport::build(&b, &ArcPolicy::default());
    let selected = select_connections(
        &b,
        report.connections(&b),
        ctx.scope.nets.as_ref(),
        ctx.scope.region,
    );
    let (tasks, _) = split_excluded(ctx.rules, selected, &ctx.settings.excluded_classes);
    let (tasks, _) = split_left_to_planes(&b, tasks, &ctx.settings.plane_tracks);
    (
        open_connection_count(&b, &report, &ctx.settings.plane_tracks),
        tasks,
    )
}

fn pad_of(item: ItemRef) -> Option<PadId> {
    match item {
        ItemRef::Pad(p) => Some(p),
        _ => None,
    }
}

fn pad_label(board: &RouteBoard, item: ItemRef) -> String {
    match pad_of(item) {
        Some(p) => {
            let pad = &board.pads[p.index()];
            format!(
                "{}.{}",
                board.footprints[pad.footprint.index()]
                    .reference
                    .as_deref()
                    .unwrap_or("?"),
                pad.number
            )
        }
        None => format!("{item:?}"),
    }
}

/// Route `tasks` around `board` plus `fixed`, with pin access computed on
/// that copper (a pad sealed in before has access points only once its
/// fence is gone). Via layers are filled in.
fn route(
    ctx: &Ctx<'_>,
    board: &RouteBoard,
    fixed: &RoutedCopper,
    tasks: &[Connection],
    backend: &mut (dyn RouteCompute + Send),
    tel: &mut Telemetry,
) -> DetailResult {
    if tasks.is_empty() {
        return DetailResult::default();
    }
    let tasks: Vec<Connection> = tasks
        .iter()
        .enumerate()
        .map(|(i, c)| Connection {
            id: ConnectionId(i as u32),
            ..*c
        })
        .collect();
    let ids: Vec<ConnectionId> = tasks.iter().map(|c| c.id).collect();
    let mut inputs = NegotiationInputs {
        access: Vec::new(),
        neck_down: ctx.inputs.neck_down.clone(),
        connection_widths: BTreeMap::new(),
    };
    // A connection's width entry follows it through the rip: the entries
    // are matched to these tasks by the islands of the copper they are
    // routed around, as the first task list was.
    if !ctx.settings.connection_widths.is_empty() {
        let on = frozen(board, fixed);
        let report = ConnectivityReport::build(&on, &ArcPolicy::default());
        let entries = &ctx.settings.connection_widths;
        inputs.connection_widths = match_connection_widths(&on, &report, &tasks, entries)
            .into_iter()
            .map(|(id, entry)| {
                let c = &tasks[id.index()];
                ((c.from, c.to), entries[entry].width)
            })
            .collect();
    }
    if let (Some(access), true) = (&ctx.settings.access, ctx.settings.use_access) {
        let on = frozen(board, fixed);
        let mut by_class: BTreeMap<ClassId, BTreeSet<PadId>> = BTreeMap::new();
        for c in &tasks {
            let class = ctx.rules.class_of(Some(c.net));
            for p in [c.from, c.to].into_iter().filter_map(pad_of) {
                by_class.entry(class).or_default().insert(p);
            }
        }
        for (class, pads) in by_class {
            let pads: Vec<PadId> = pads.into_iter().collect();
            let (points, _) =
                compute_pin_access_with(&on, ctx.rules, &pads, ProfileId(class.0), access);
            inputs.access.extend(points);
        }
    }
    let mut out = route_around(
        board,
        fixed,
        ctx.rules,
        &tasks,
        &ids,
        ctx.settings,
        ctx.negotiation,
        &inputs,
        backend,
        tel,
    );
    let all = board.layers.all();
    for v in &mut out.copper.vias {
        if v.layers.is_empty() {
            v.layers = all;
        }
    }
    out
}

/// Why a net's board copper may not be ripped (`None`: it may).
fn net_refusal(ctx: &Ctx<'_>, board: &RouteBoard, net: NetId) -> Option<&'static str> {
    if ctx.settings.rip_up.mode != RipUp::Unlocked {
        return Some("board_copper_not_requested");
    }
    if ctx.scope.nets.as_ref().is_some_and(|s| !s.contains(&net)) {
        return Some("out_of_scope");
    }
    if ctx
        .settings
        .excluded_classes
        .contains(&ctx.rules.class_of(Some(net)))
    {
        return Some("excluded_class");
    }
    if board.zones.iter().any(|z| z.net == Some(net)) {
        return Some("plane_net");
    }
    let movable = |o: Ownership| o == Ownership::Fixed;
    let inside = |b: BBox| ctx.scope.region.is_none_or(|r| r.contains_bbox(&b));
    let tracks = board
        .tracks
        .iter()
        .filter(|t| t.net == Some(net) && movable(t.ownership))
        .map(|t| (t.uuid.is_some(), t.seg.bbox()));
    let arcs = board
        .arcs
        .iter()
        .filter(|a| a.net == Some(net) && movable(a.ownership))
        .map(|a| (a.uuid.is_some(), arc_bbox(a)));
    let vias = board
        .vias
        .iter()
        .filter(|v| v.net == Some(net) && movable(v.ownership))
        .map(|v| (v.uuid.is_some(), v.shape().bbox()));
    for (has_uuid, bbox) in tracks.chain(arcs).chain(vias) {
        if !has_uuid {
            return Some("no_uuid");
        }
        if !inside(bbox) {
            return Some("out_of_scope");
        }
    }
    None
}

fn arc_bbox(a: &crate::board::ArcTrack) -> BBox {
    BBox::from_points([a.start, a.mid, a.end])
        .unwrap_or(BBox::point(a.start))
        .inflate(a.width / 2 + 1)
}

/// A piece of copper a probe path collided with.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
struct Hit {
    net: NetId,
    /// `None`: it may be ripped.
    refusal: Option<&'static str>,
    uuid: Option<String>,
}

/// One stuck connection's probe result.
#[derive(Debug, Clone, Default)]
struct Probe {
    /// A probe path was found.
    routed: bool,
    hits: BTreeSet<Hit>,
}

impl Probe {
    /// The nets to rip for this connection, when every blocker may be.
    fn rip_set(&self) -> Option<BTreeSet<NetId>> {
        (self.routed && self.hits.iter().all(|h| h.refusal.is_none()))
            .then(|| self.hits.iter().map(|h| h.net).collect())
    }
}

/// What a probe takes out of the board around the stuck pads.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Take {
    /// The copper that may be ripped: the probe an attempt is built on.
    Rippable,
    /// Every nearby track, arc and via, rippable or not: only to name what
    /// blocks a connection the first probe found no path for.
    Copper,
    /// That, and every zone fill, when even [`Take::Copper`] leaves no path.
    CopperAndFills,
}

/// Probe `stuck` (see the module docs).
fn probe(
    ctx: &Ctx<'_>,
    state: &State,
    stuck: &[Connection],
    take: Take,
    backend: &mut (dyn RouteCompute + Send),
    tel: &mut Telemetry,
) -> Vec<Probe> {
    let everything = take != Take::Rippable;
    let board = &state.board;
    let r = ctx.settings.rip_up.probe_radius;
    let zones: Vec<BBox> = stuck
        .iter()
        .flat_map(|c| [c.from, c.to])
        .filter_map(pad_of)
        .filter_map(|p| board.pads[p.index()].bbox())
        .map(|b| b.inflate(r))
        .collect();
    let near = |b: BBox| zones.iter().any(|z| z.intersects(&b));
    let mut refusals: BTreeMap<NetId, Option<&'static str>> = BTreeMap::new();
    let mut refusal = |net: NetId| {
        *refusals
            .entry(net)
            .or_insert_with(|| net_refusal(ctx, board, net))
    };
    // The board items a probe leaves out: unlocked copper of nets that may
    // be ripped (or, for the report, everything nearby).
    let mut out = |net: Option<NetId>, o: Ownership, bbox: BBox| {
        let Some(net) = net else { return false };
        near(bbox) && (everything || (o == Ownership::Fixed && refusal(net).is_none()))
    };
    let mut pb = board.clone();
    pb.tracks.retain(|t| !out(t.net, t.ownership, t.seg.bbox()));
    pb.arcs.retain(|a| !out(a.net, a.ownership, arc_bbox(a)));
    pb.vias
        .retain(|v| !out(v.net, v.ownership, v.shape().bbox()));
    if take == Take::CopperAndFills {
        // Zone fills too: a pour refilled around the board's tracks walls
        // in whatever was not routed before the refill.
        pb.zones.retain(|z| !(z.filled && z.net.is_some()));
        for (n, z) in pb.zones.iter_mut().enumerate() {
            z.id = ZoneId(n as u32);
        }
    }
    renumber(&mut pb);
    let mut pr = state.routed.clone();
    pr.tracks.retain(|t| !near(t.seg.bbox()));
    pr.vias
        .retain(|v| !near(BBox::point(v.center).inflate(v.diameter / 2 + 1)));
    let found = route(ctx, &pb, &pr, stuck, backend, tel);

    // What the probe paths collide with on the real copper.
    let full = frozen(board, &state.routed);
    let (nt, nv) = (board.tracks.len(), board.vias.len());
    // The routed-copper check leaves zone fills to KiCad's refill; the
    // widest probe wants them named, so it checks them as copper.
    let options = if take == Take::CopperAndFills {
        CheckOptions::default()
    } else {
        CheckOptions::for_routed()
    };
    let checker = BoardChecker::new(&full, ctx.rules, options);
    let report = checker.check_routed(&found.copper);
    let mut probes = vec![Probe::default(); stuck.len()];
    for id in &found.routed {
        probes[id.index()].routed = true;
    }
    let owners = |item: CheckedItem| -> Vec<ConnectionId> {
        match item {
            CheckedItem::RoutedTrack { index } => found
                .track_owner
                .get(index as usize)
                .copied()
                .into_iter()
                .collect(),
            CheckedItem::RoutedVia { index } => found
                .via_users
                .get(index as usize)
                .cloned()
                .unwrap_or_default(),
            _ => Vec::new(),
        }
    };
    let mut hit = |item: CheckedItem| -> Option<Hit> {
        let CheckedItem::Board { item } = item else {
            return None;
        };
        let (net, o, uuid) = match item {
            ItemRef::Track(id) if id.index() < nt => {
                let t = &full.tracks[id.index()];
                (t.net?, Some(t.ownership), t.uuid.clone())
            }
            // This run's own copper, frozen in behind the board's.
            ItemRef::Track(id) => (full.tracks[id.index()].net?, None, None),
            ItemRef::Via(id) if id.index() < nv => {
                let v = &full.vias[id.index()];
                (v.net?, Some(v.ownership), v.uuid.clone())
            }
            ItemRef::Via(id) => (full.vias[id.index()].net?, None, None),
            ItemRef::Arc(id) => {
                let a = &full.arcs[id.index()];
                (a.net?, Some(a.ownership), a.uuid.clone())
            }
            ItemRef::Zone(id) => {
                return Some(Hit {
                    net: full.zones[id.index()].net?,
                    refusal: Some("zone_fill"),
                    uuid: None,
                })
            }
            ItemRef::Pad(_) => return None,
        };
        let refusal = match o {
            None => None,
            Some(Ownership::Fixed) => refusal(net),
            Some(_) => Some("locked"),
        };
        Some(Hit { net, refusal, uuid })
    };
    for v in &report.violations {
        let Some(b) = v.b else { continue };
        for (mine, other) in [(v.a, b), (b, v.a)] {
            let who = owners(mine);
            if who.is_empty() {
                continue;
            }
            let Some(h) = hit(other) else { continue };
            for c in who {
                probes[c.index()].hits.insert(h.clone());
            }
        }
    }
    // A net's own copper never blocks it.
    for (p, c) in probes.iter_mut().zip(stuck) {
        p.hits.retain(|h| h.net != c.net);
    }
    probes
}

/// A rip set tried for a set of stuck connections.
type Tried = (BTreeSet<NetId>, BTreeSet<(NetId, ItemRef, ItemRef)>);

/// Rip `nets` whole and route `stuck` first, then the ripped nets (or,
/// `together`, all of them as one negotiated batch). Returns the state that
/// results (not judged).
#[allow(clippy::too_many_arguments)]
fn attempt(
    ctx: &Ctx<'_>,
    state: &State,
    nets: &BTreeSet<NetId>,
    stuck: &[Connection],
    together: bool,
    round: u32,
    backend: &mut (dyn RouteCompute + Send),
    tel: &mut Telemetry,
) -> State {
    let mut next = state.clone();
    let stuck_nets: BTreeSet<NetId> = stuck.iter().map(|c| c.net).collect();
    let names = |set: &BTreeSet<NetId>| -> Vec<String> {
        set.iter()
            .map(|n| state.board.net(*n).name.clone())
            .collect()
    };
    // Board copper of the ripped nets (when the board's may be ripped).
    let board_nets: BTreeSet<NetId> = nets
        .iter()
        .copied()
        .filter(|n| net_refusal(ctx, &state.board, *n).is_none())
        .collect();
    let reason = |net: NetId| {
        let blocked: BTreeSet<NetId> = stuck_nets.iter().copied().filter(|s| *s != net).collect();
        format!(
            "its net's copper blocked {}; the net is routed again",
            names(&blocked).join(", ")
        )
    };
    let layers = &state.board.layers;
    let mut take =
        |net: Option<NetId>, o: Ownership, uuid: &Option<String>, kind: ItemKind, layer: String| {
            let (Some(net), Ownership::Fixed, Some(uuid)) = (net, o, uuid) else {
                return false;
            };
            if !board_nets.contains(&net) {
                return false;
            }
            next.ripped.push(RippedItem {
                uuid: uuid.clone(),
                kind,
                net: state.board.net(net).name.clone(),
                layer,
                round,
                reason: reason(net),
            });
            true
        };
    let mut board = state.board.clone();
    board.tracks.retain(|t| {
        !take(
            t.net,
            t.ownership,
            &t.uuid,
            ItemKind::Track,
            layers.name(t.layer).to_string(),
        )
    });
    board.arcs.retain(|a| {
        !take(
            a.net,
            a.ownership,
            &a.uuid,
            ItemKind::Arc,
            layers.name(a.layer).to_string(),
        )
    });
    board.vias.retain(|v| {
        let mut span = v.layers.iter();
        let first = span.next();
        let last = span.last().or(first);
        let layer = match (first, last) {
            (Some(a), Some(b)) => format!("{}-{}", layers.name(a), layers.name(b)),
            _ => String::new(),
        };
        !take(v.net, v.ownership, &v.uuid, ItemKind::Via, layer)
    });
    renumber(&mut board);
    let mut kept = state.routed.clone();
    kept.tracks.retain(|t| !nets.contains(&t.net));
    kept.vias.retain(|v| !nets.contains(&v.net));

    // The stuck connections first, around everything left.
    let (_, open) = open_connections(ctx, &board, &kept);
    let first: Vec<Connection> = open
        .into_iter()
        .filter(|c| !together && stuck_nets.contains(&c.net) && !nets.contains(&c.net))
        .collect();
    let a = route(ctx, &board, &kept, &first, backend, tel);
    // Then the ripped nets (and any stuck one not yet joined), around those.
    let with_a = concat(&kept, &a.copper);
    let (_, open) = open_connections(ctx, &board, &with_a);
    let second: Vec<Connection> = open
        .into_iter()
        .filter(|c| stuck_nets.contains(&c.net) || nets.contains(&c.net))
        .collect();
    let b = route(ctx, &board, &with_a, &second, backend, tel);
    // Only copper that passes the exact check against the rest is kept.
    let mut fresh = concat(&a.copper, &b.copper);
    clear_findings(&frozen(&board, &kept), ctx.rules, &mut fresh);
    next.routed = concat(&kept, &fresh);
    next.board = board;
    next.rerouted.extend(nets.iter().copied());
    (next.open, next.stuck) = open_connections(ctx, &next.board, &next.routed);
    next
}

/// Run the stage on `work` (the board with the fanout as locked copper)
/// and `routed` (this run's copper so far).
#[allow(clippy::too_many_arguments)]
pub(super) fn rip_up(
    work: &RouteBoard,
    rules: &dyn RuleTable,
    scope: &PipelineScope,
    settings: &PipelineSettings,
    negotiation: &NegotiationSettings,
    inputs: &NegotiationInputs,
    routed: &RoutedCopper,
    backend: &mut (dyn RouteCompute + Send),
    tel: &mut Telemetry,
) -> Outcome {
    let ctx = Ctx {
        rules,
        scope,
        settings,
        negotiation,
        inputs,
    };
    let rs = settings.rip_up;
    let mut state = State {
        board: work.clone(),
        routed: routed.clone(),
        ripped: Vec::new(),
        rerouted: BTreeSet::new(),
        open: 0,
        stuck: Vec::new(),
    };
    let (open, mut stuck) = open_connections(&ctx, &state.board, &state.routed);
    state.open = open;
    state.stuck = stuck.clone();
    let mut report = RipUpReport {
        mode: rs.mode,
        open_before: open,
        open_after: open,
        ..RipUpReport::default()
    };
    let mut tried: BTreeSet<Tried> = BTreeSet::new();
    let name = |board: &RouteBoard, n: NetId| board.net(n).name.clone();
    loop {
        if stuck.is_empty() {
            report.stopped = "complete".to_string();
            break;
        }
        let round = report.rounds + 1;
        let mut probes = probe(&ctx, &state, &stuck, Take::Rippable, backend, tel);
        // Stuck connections whose nets are within what one attempt may rip.
        let sets: Vec<Option<BTreeSet<NetId>>> = probes
            .iter()
            .map(|p| p.rip_set().filter(|s| s.len() <= rs.max_nets))
            .collect();
        let able: Vec<usize> = (0..stuck.len()).filter(|&i| sets[i].is_some()).collect();
        // The attempts of this round: all together, then each alone.
        let mut plans: Vec<(BTreeSet<NetId>, Vec<usize>)> = Vec::new();
        let union: BTreeSet<NetId> = able
            .iter()
            .flat_map(|&i| sets[i].iter().flatten().copied())
            .collect();
        if !able.is_empty() && union.len() <= rs.max_nets {
            plans.push((union, able.clone()));
        }
        if able.len() > 1 || plans.is_empty() {
            let mut singles = able.clone();
            singles.sort_by_key(|&i| (sets[i].as_ref().map_or(0, BTreeSet::len), i));
            for i in singles.into_iter().take(rs.single_attempts as usize) {
                plans.push((sets[i].clone().unwrap_or_default(), vec![i]));
            }
        }
        // Past the last round the probe only says what blocks what is left.
        let last = round > rs.max_rounds;
        if !last {
            report.rounds = round;
        }
        let mut accepted = false;
        let mut fresh_attempts = 0;
        if !last {
            for (nets, who) in &plans {
                let conns: Vec<Connection> = who.iter().map(|&i| stuck[i]).collect();
                let key: Tried = (
                    nets.clone(),
                    conns.iter().map(|c| (c.net, c.from, c.to)).collect(),
                );
                let mut record = RipUpAttempt {
                    round,
                    stuck_nets: conns
                        .iter()
                        .map(|c| c.net)
                        .collect::<BTreeSet<_>>()
                        .into_iter()
                        .map(|n| name(&state.board, n))
                        .collect(),
                    ripped_nets: nets.iter().map(|n| name(&state.board, *n)).collect(),
                    open_before: state.open,
                    open_after: state.open,
                    ..RipUpAttempt::default()
                };
                if !tried.insert(key) {
                    record.repeat = true;
                    report.attempts.push(record);
                    continue;
                }
                fresh_attempts += 1;
                for together in [false, true] {
                    // A plain retry has no order to change.
                    if together && nets.is_empty() {
                        break;
                    }
                    let next = attempt(&ctx, &state, nets, &conns, together, round, backend, tel);
                    let mut record = record.clone();
                    record.order = if together { "together" } else { "stuck_first" }.to_string();
                    record.open_after = next.open;
                    record.accepted = next.open < state.open;
                    record.open_nets_after = next
                        .stuck
                        .iter()
                        .map(|c| c.net)
                        .collect::<BTreeSet<_>>()
                        .into_iter()
                        .map(|n| name(&next.board, n))
                        .collect();
                    report.attempts.push(record);
                    if next.open < state.open {
                        state = next;
                        accepted = true;
                        break;
                    }
                }
                if accepted {
                    break;
                }
            }
        }
        if accepted {
            stuck = state.stuck.clone();
            continue;
        }
        // The stage ends here: say what blocks what is left. Connections
        // with no probe path are probed again with every nearby track, arc
        // and via out, then with the zone fills out as well, only to name
        // their blockers.
        for take in [Take::Copper, Take::CopperAndFills] {
            let lost: Vec<usize> = (0..stuck.len()).filter(|&i| !probes[i].routed).collect();
            if lost.is_empty() {
                break;
            }
            let again: Vec<Connection> = lost.iter().map(|&i| stuck[i]).collect();
            let wide = probe(&ctx, &state, &again, take, backend, tel);
            for (i, p) in lost.into_iter().zip(wide) {
                probes[i] = p;
            }
        }
        report.stopped = if last {
            "max_rounds"
        } else if plans.is_empty() {
            "nothing_to_rip"
        } else if fresh_attempts == 0 {
            "repeat"
        } else {
            "no_improvement"
        }
        .to_string();
        for (c, p) in stuck.iter().zip(&probes) {
            let mut by: BTreeMap<(NetId, &str), Vec<String>> = BTreeMap::new();
            for h in &p.hits {
                by.entry((h.net, h.refusal.unwrap_or("ripping_did_not_help")))
                    .or_default()
                    .extend(h.uuid.clone());
            }
            report.still_open.push(StillOpen {
                net: name(&state.board, c.net),
                from: pad_label(&state.board, c.from),
                to: pad_label(&state.board, c.to),
                blocked_by: by
                    .into_iter()
                    .map(|((net, reason), uuids)| Blocker {
                        net: name(&state.board, net),
                        reason: reason.to_string(),
                        uuids,
                    })
                    .collect(),
            });
        }
        break;
    }
    report.open_after = state.open;
    report.rerouted_nets = state
        .rerouted
        .iter()
        .map(|n| name(&state.board, *n))
        .collect();
    report.ripped = state.ripped.clone();
    let changed = report.changed().then_some((state.board, state.routed));
    Outcome { report, changed }
}

#[cfg(test)]
mod tests;
