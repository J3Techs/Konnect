//! The PathFinder driver: terminals, seeding, negotiation, batching and
//! the result.

use super::capacity::slot_rules;
use super::grid::{Topology, Win};
use super::lookahead::Lookahead;
use super::pattern::{best_pattern, PatternReq};
use super::search::{astar, path_resources, Costs, Request, Scratch};
use super::{
    capacity, Corridor, GCellGrid, GlobalResult, GlobalSettings, GlobalStats, IterationStats,
    LayerPolicy,
};
use crate::api::{Connection, FailureReason, ItemKind, ItemRef, LayerMask, NetId};
use crate::board::RouteBoard;
use crate::compute::job::{JobDescriptor, JobPool, JOB_ALLOW_VIA, JOB_REROUTE};
use crate::compute::raster::Window;
use crate::compute::{BatchBuilder, BatchPolicy};
use crate::connectivity::ConnectivityReport;
use crate::rules::{RuleSubject, RuleTable};
use crate::telemetry::Telemetry;
use konnect_geom::{ArcPolicy, Point};
use rayon::prelude::*;
use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::sync::Mutex;
use std::time::Instant;

/// One connection, prepared.
struct Task {
    net: u32,
    /// Demand on a planar edge, in default track pitches.
    weight: u16,
    /// Demand on a via site, in default via sites.
    via_weight: u16,
    /// Allowed slots.
    slots: u32,
    /// Terminal cells with their slots, ascending by cell.
    src: Vec<(u32, u32)>,
    dst: Vec<(u32, u32)>,
    src_cell: u32,
    dst_cell: u32,
    src_slots: u32,
    dst_slots: u32,
    /// Cells where zero-capacity resources stay passable: the terminal
    /// cells and their neighbours (pin access the grid cannot resolve).
    access: Vec<u32>,
    /// The base-cell box of the two primary terminals.
    core: Win,
    /// Why the connection cannot be routed at all (a terminal with no
    /// cell on the grid), else `None`.
    unroutable: Option<FailureReason>,
}

/// How a job ended.
enum Outcome {
    Path { path: Vec<u32>, pattern: bool },
    NoPath,
}

struct JobResult {
    outcome: Outcome,
    expansions: u64,
    searched: bool,
}

/// A scratch borrowed from the pool for one rayon split.
struct Lease<'p> {
    pool: &'p Mutex<Vec<Scratch>>,
    sizes: (usize, usize, usize),
    s: Option<Scratch>,
}

impl<'p> Lease<'p> {
    fn new(pool: &'p Mutex<Vec<Scratch>>, sizes: (usize, usize, usize)) -> Self {
        let s = pool.lock().map(|mut v| v.pop()).unwrap_or(None);
        Lease {
            pool,
            sizes,
            s: Some(s.unwrap_or_else(|| Scratch::new(sizes.0, sizes.1, sizes.2))),
        }
    }

    fn get(&mut self) -> &mut Scratch {
        let sizes = self.sizes;
        self.s
            .get_or_insert_with(|| Scratch::new(sizes.0, sizes.1, sizes.2))
    }
}

impl Drop for Lease<'_> {
    fn drop(&mut self) {
        if let (Some(s), Ok(mut v)) = (self.s.take(), self.pool.lock()) {
            v.push(s);
        }
    }
}

/// A partial re-route: keep `prefix` and `suffix`, search between them.
struct Section {
    prefix: Vec<u32>,
    suffix: Vec<u32>,
}

impl Section {
    /// The new path through `q`, which runs from a prefix node to a suffix
    /// node.
    fn compose(&self, q: &[u32]) -> Vec<u32> {
        let x = self
            .prefix
            .iter()
            .position(|&n| n == q[0])
            .expect("a partial search starts on the kept prefix");
        let y = self
            .suffix
            .iter()
            .position(|&n| Some(&n) == q.last())
            .expect("a partial search ends on the kept suffix");
        let mut out = Vec::with_capacity(x + q.len() + self.suffix.len() - y);
        out.extend_from_slice(&self.prefix[..x]);
        out.extend_from_slice(q);
        out.extend_from_slice(&self.suffix[y + 1..]);
        out
    }
}

/// Cells (base window) spanned by `nodes`.
fn nodes_box(t: &Topology, nodes: &[u32]) -> Win {
    let mut w = Win {
        c0: u16::MAX,
        r0: u16::MAX,
        c1: 0,
        r1: 0,
    };
    for &n in nodes {
        let (_, c) = t.split(n);
        let (col, row) = (t.col[c as usize], t.row[c as usize]);
        w.c0 = w.c0.min(col);
        w.r0 = w.r0.min(row);
        w.c1 = w.c1.max(col);
        w.r1 = w.r1.max(row);
    }
    w
}

/// Steps kept around the overused stretch of a partial re-route.
const SECTION_PAD: usize = 3;

/// Most entries of the lookahead table (64 MB of `f32`).
const LOOKAHEAD_ENTRIES: u64 = 1 << 24;

struct State {
    /// Resources from here on are via sites.
    via_from: u32,
    occ: Vec<u16>,
    hist: Vec<f32>,
    /// Per net: resource → number of the net's connections using it.
    net_res: Vec<BTreeMap<u32, u16>>,
    path: Vec<Vec<u32>>,
    res: Vec<Vec<u32>>,
    routed: Vec<bool>,
    failed: Vec<bool>,
    reroutes: Vec<u32>,
}

impl State {
    fn commit(
        &mut self,
        t: &Topology,
        tasks: &[Task],
        i: usize,
        path: Vec<u32>,
        buf: &mut Vec<u32>,
    ) {
        path_resources(t, &path, buf);
        let task = &tasks[i];
        for &r in buf.iter() {
            let w = self.demand(task, r);
            let n = self.net_res[task.net as usize].entry(r).or_insert(0);
            if *n == 0 {
                self.occ[r as usize] = self.occ[r as usize].saturating_add(w);
            }
            *n += 1;
        }
        self.res[i] = buf.clone();
        self.path[i] = path;
        self.routed[i] = true;
    }

    /// Hold the kept ends of a partial re-route: their resources stay
    /// committed while the middle is searched.
    fn hold(&mut self, t: &Topology, tasks: &[Task], i: usize, sec: &Section, buf: &mut Vec<u32>) {
        let mut all = Vec::new();
        path_resources(t, &sec.prefix, buf);
        all.extend_from_slice(buf);
        path_resources(t, &sec.suffix, buf);
        all.extend_from_slice(buf);
        all.sort_unstable();
        all.dedup();
        let task = &tasks[i];
        for &r in &all {
            let w = self.demand(task, r);
            let n = self.net_res[task.net as usize].entry(r).or_insert(0);
            if *n == 0 {
                self.occ[r as usize] = self.occ[r as usize].saturating_add(w);
            }
            *n += 1;
        }
        self.res[i] = all;
        self.path[i].clear();
        self.routed[i] = true;
    }

    /// The overused stretch of connection `i`, padded, as a section; `None`
    /// when it spans the whole path.
    fn section(&self, t: &Topology, cap: &[u16], i: usize) -> Option<Section> {
        let path = &self.path[i];
        let mut first = None;
        let mut last = 0;
        for (k, w) in path.windows(2).enumerate() {
            let (sa, ca) = t.split(w[0]);
            let (sb, cb) = t.split(w[1]);
            let r = if sa == sb {
                t.planar_res(sa, t.edge_between(ca, cb)?)
            } else {
                t.via_res(ca)
            };
            if self.overused(cap, r) {
                first.get_or_insert(k);
                last = k;
            }
        }
        let first = first?;
        let lo = first.saturating_sub(SECTION_PAD);
        let hi = (last + 1 + SECTION_PAD).min(path.len() - 1);
        if lo == 0 && hi == path.len() - 1 {
            return None;
        }
        Some(Section {
            prefix: path[..=lo].to_vec(),
            suffix: path[hi..].to_vec(),
        })
    }

    fn rip_up(&mut self, tasks: &[Task], i: usize) {
        if !self.routed[i] {
            return;
        }
        let task = &tasks[i];
        let res = std::mem::take(&mut self.res[i]);
        for &r in &res {
            let w = self.demand(task, r);
            let map = &mut self.net_res[task.net as usize];
            if let Some(n) = map.get_mut(&r) {
                *n -= 1;
                if *n == 0 {
                    map.remove(&r);
                    self.occ[r as usize] = self.occ[r as usize].saturating_sub(w);
                }
            }
        }
        self.res[i] = res;
        self.res[i].clear();
        self.path[i].clear();
        self.routed[i] = false;
    }

    /// The demand `task` puts on resource `r`.
    fn demand(&self, task: &Task, r: u32) -> u16 {
        if r >= self.via_from {
            task.via_weight
        } else {
            task.weight
        }
    }

    fn overused(&self, cap: &[u16], r: u32) -> bool {
        let c = cap[r as usize];
        c > 0 && self.occ[r as usize] > c
    }
}

/// The slot bitmask of a layer mask on this grid.
fn slot_mask(grid: &GCellGrid, layers: LayerMask) -> u32 {
    let mut m = 0;
    for (slot, layer) in grid.layers.iter().enumerate() {
        if layers.contains(layer) {
            m |= 1 << slot;
        }
    }
    m
}

/// Every slot of an `n`-slot grid (`n` ≤ 32, as a [`LayerMask`] holds).
pub(super) fn all_slots(n: u32) -> u32 {
    if n == 0 {
        0
    } else {
        u32::MAX >> (32 - n.min(32))
    }
}

/// The layer mask of a slot bitmask on this grid.
fn slot_layers(grid: &GCellGrid, slots: u32) -> LayerMask {
    grid.layers
        .iter()
        .enumerate()
        .filter(|(slot, _)| slots & (1u32 << slot) != 0)
        .fold(LayerMask::EMPTY, |m, (_, l)| m.with(l))
}

/// Points and layers of one copper item, for terminals.
fn item_points(board: &RouteBoard, item: ItemRef, step: i64, out: &mut Vec<(Point, LayerMask)>) {
    match item {
        ItemRef::Pad(id) => {
            let p = &board.pads[id.index()];
            let at = p.bbox().map_or(p.center, |b| b.center());
            out.push((at, p.layers));
        }
        ItemRef::Via(id) => {
            let v = &board.vias[id.index()];
            out.push((v.center, v.layers));
        }
        ItemRef::Track(id) => {
            let tr = &board.tracks[id.index()];
            let (a, b) = (tr.seg.a, tr.seg.b);
            let len = (b - a).length_f64();
            let n = ((len / step as f64).ceil() as i64).max(1);
            for k in 0..=n {
                let f = k as f64 / n as f64;
                let p = Point::new(
                    (f64::from(a.x) + f * f64::from(b.x - a.x)).round() as i32,
                    (f64::from(a.y) + f * f64::from(b.y - a.y)).round() as i32,
                );
                out.push((p, LayerMask::single(tr.layer)));
            }
        }
        ItemRef::Arc(id) => {
            let a = &board.arcs[id.index()];
            for p in [a.start, a.mid, a.end] {
                out.push((p, LayerMask::single(a.layer)));
            }
        }
        // Planes join through fanout (S2), not as corridor terminals.
        ItemRef::Zone(_) => {}
    }
}

/// `(cell, slots)` terminal cells, the primary cell and its slots.
type Terminals = (Vec<(u32, u32)>, u32, u32);

/// Terminal cells of the island holding `item`: `(cell, slots)` ascending,
/// plus the primary item's cell and slots.
#[allow(clippy::too_many_arguments)]
fn terminals(
    board: &RouteBoard,
    grid: &GCellGrid,
    topo: &Topology,
    islands: &HashMap<ItemRef, (u32, u32)>,
    conn: &ConnectivityReport,
    item: ItemRef,
    allowed: u32,
) -> Option<Terminals> {
    let step = topo.pitch / 2;
    let mut pts = Vec::new();
    item_points(board, item, step, &mut pts);
    let (pp, pl) = *pts.first()?;
    let primary_cell = topo.locate(pp)?;
    let fix = |m: u32| {
        if m & allowed == 0 {
            allowed
        } else {
            m & allowed
        }
    };
    let primary_slots = {
        let m = slot_mask(grid, pl);
        if m == 0 {
            allowed
        } else {
            fix(m)
        }
    };
    let mut cells: BTreeMap<u32, u32> = BTreeMap::new();
    cells.insert(primary_cell, primary_slots);
    if let Some(&(net, island)) = islands.get(&item) {
        let members = &conn.nets[net as usize].islands[island as usize].items;
        for &m in members {
            pts.clear();
            item_points(board, m, step, &mut pts);
            for &(p, layers) in &pts {
                let sm = slot_mask(grid, layers);
                if sm == 0 {
                    continue;
                }
                if let Some(c) = topo.locate(p) {
                    *cells.entry(c).or_insert(0) |= fix(sm);
                }
            }
        }
    }
    Some((cells.into_iter().collect(), primary_cell, primary_slots))
}

fn prepare(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    grid: &GCellGrid,
    topo: &Topology,
    connections: &[Connection],
    policy: &LayerPolicy,
) -> Vec<Task> {
    let conn = ConnectivityReport::build(board, &ArcPolicy::default());
    let mut islands: HashMap<ItemRef, (u32, u32)> = HashMap::new();
    for nc in &conn.nets {
        for (i, isl) in nc.islands.iter().enumerate() {
            for &it in &isl.items {
                islands.insert(it, (nc.net.0, i as u32));
            }
        }
    }
    let all = all_slots(topo.slots);
    let layer0 = grid.slot_layer(0);
    let default_pitch = layer0.map_or(1, |l| slot_rules(rules, l).pitch());
    let via_pitch = |net: Option<NetId>| {
        let s = RuleSubject::new(ItemKind::Via, net);
        let v = rules.via(&s);
        let c = layer0.map_or(0, |l| i64::from(rules.clearance(&s, &s, l)));
        (i64::from(v.diameter) + c).max(1) as f64
    };
    let default_via = via_pitch(None);
    connections
        .par_iter()
        .map(|c| {
            let class = rules.class_of(Some(c.net));
            let allowed = policy
                .class_layers
                .get(&class)
                .map(|m| slot_mask(grid, *m))
                .filter(|&m| m != 0)
                .unwrap_or(all);
            let weight = layer0.map_or(1, |l| {
                let s = RuleSubject::new(ItemKind::Track, Some(c.net));
                let need = i64::from(rules.track_width(&s, l).preferred)
                    + i64::from(rules.clearance(&s, &s, l));
                ((need + default_pitch - 1) / default_pitch).clamp(1, 255) as u16
            });
            // A via site holds vias by the area they keep clear.
            let via_weight =
                ((via_pitch(Some(c.net)) / default_via).powi(2).round() as u16).clamp(1, 16);
            let from = terminals(board, grid, topo, &islands, &conn, c.from, allowed);
            let to = terminals(board, grid, topo, &islands, &conn, c.to, allowed);
            let off_grid = if from.is_none() { c.from } else { c.to };
            let (Some((src, sc, ss)), Some((dst, dc, ds))) = (from, to) else {
                return Task {
                    net: c.net.0,
                    weight,
                    via_weight,
                    slots: allowed,
                    src: Vec::new(),
                    dst: Vec::new(),
                    src_cell: 0,
                    dst_cell: 0,
                    src_slots: 0,
                    dst_slots: 0,
                    access: Vec::new(),
                    core: Win {
                        c0: 0,
                        r0: 0,
                        c1: 0,
                        r1: 0,
                    },
                    unroutable: Some(FailureReason::NoAccess { item: off_grid }),
                };
            };
            let (a, b) = (sc as usize, dc as usize);
            let core = Win {
                c0: topo.col[a].min(topo.col[b]),
                r0: topo.row[a].min(topo.row[b]),
                c1: topo.col[a].max(topo.col[b]),
                r1: topo.row[a].max(topo.row[b]),
            };
            let mut access: Vec<u32> = Vec::new();
            for &(cell, _) in src.iter().chain(dst.iter()) {
                access.push(cell);
                let (lo, hi) = (
                    topo.adj_off[cell as usize] as usize,
                    topo.adj_off[cell as usize + 1] as usize,
                );
                access.extend_from_slice(&topo.adj_cell[lo..hi]);
            }
            access.sort_unstable();
            access.dedup();
            Task {
                net: c.net.0,
                weight,
                via_weight,
                slots: allowed,
                access,
                src,
                dst,
                src_cell: sc,
                dst_cell: dc,
                src_slots: ss,
                dst_slots: ds,
                core,
                unroutable: None,
            }
        })
        .collect()
}

/// The search window around `core`: a fixed margin, plus growth with each
/// re-route (capped), plus an eighth of the span.
fn window_of(topo: &Topology, core: Win, settings: &GlobalSettings, reroutes: u32) -> Win {
    let span = u32::from((core.c1 - core.c0).max(core.r1 - core.r0));
    let grow = settings
        .window_growth
        .saturating_mul(reroutes)
        .min(settings.window_growth_cap);
    let m = settings
        .window_margin
        .saturating_add(grow)
        .saturating_add(span / 8);
    let grow_lo = |v: u16| u32::from(v).saturating_sub(m) as u16;
    let grow_hi = |v: u16, n: u32| (u32::from(v) + m).min(n - 1) as u16;
    Win {
        c0: grow_lo(core.c0),
        r0: grow_lo(core.r0),
        c1: grow_hi(core.c1, topo.cols),
        r1: grow_hi(core.r1, topo.rows),
    }
}

/// Everything one search reads.
struct Shared<'a> {
    topo: &'a Topology,
    tasks: &'a [Task],
}

#[allow(clippy::too_many_arguments)]
fn route_one(
    sh: &Shared,
    costs: &Costs,
    state: &State,
    i: usize,
    win: Win,
    seed: bool,
    section: Option<&Section>,
    s: &mut Scratch,
) -> JobResult {
    let t = sh.topo;
    let task = &sh.tasks[i];
    let owned = &state.net_res[task.net as usize];
    let term_cells: &[u32] = &task.access;
    if seed {
        let req = PatternReq {
            src_cell: task.src_cell,
            dst_cell: task.dst_cell,
            src_slots: task.src_slots,
            dst_slots: task.dst_slots,
            slots: task.slots,
            win,
            weight: task.weight,
            via_weight: task.via_weight,
            owned,
            term_cells,
        };
        if let Some((path, _)) = best_pattern(costs, &req, s) {
            return JobResult {
                outcome: Outcome::Path {
                    path,
                    pattern: true,
                },
                expansions: 0,
                searched: false,
            };
        }
    }
    let mut sources = Vec::new();
    let mut targets = Vec::new();
    let mut tbox = (i32::MAX, i32::MAX, i32::MIN, i32::MIN);
    let mut tslots = 0u32;
    let mut add_target = |n: u32, targets: &mut Vec<u32>| {
        let (slot, c) = t.split(n);
        let (x, y) = t.center(c);
        tbox = (tbox.0.min(x), tbox.1.min(y), tbox.2.max(x), tbox.3.max(y));
        tslots |= 1 << slot;
        targets.push(n);
    };
    match section {
        Some(sec) => {
            for &n in &sec.prefix {
                if t.in_window(t.split(n).1, &win) {
                    sources.push((n, 0.0f32));
                }
            }
            for &n in &sec.suffix {
                if t.in_window(t.split(n).1, &win) {
                    add_target(n, &mut targets);
                }
            }
        }
        None => {
            for &(c, m) in &task.src {
                if !t.in_window(c, &win) {
                    continue;
                }
                let mut mm = m & task.slots;
                while mm != 0 {
                    let slot = mm.trailing_zeros();
                    mm &= mm - 1;
                    sources.push((t.node(slot, c), 0.0f32));
                }
            }
            for &(c, m) in &task.dst {
                if !t.in_window(c, &win) {
                    continue;
                }
                let mut mm = m & task.slots;
                while mm != 0 {
                    let slot = mm.trailing_zeros();
                    mm &= mm - 1;
                    add_target(t.node(slot, c), &mut targets);
                }
            }
        }
    }
    if sources.is_empty() || targets.is_empty() {
        return JobResult {
            outcome: Outcome::NoPath,
            expansions: 0,
            searched: false,
        };
    }
    let req = Request {
        sources: &sources,
        targets: &targets,
        term_cells,
        win,
        slots: task.slots,
        weight: task.weight,
        via_weight: task.via_weight,
        owned,
        tbox,
        tslots,
    };
    let (found, expansions) = astar(costs, &req, s);
    JobResult {
        outcome: match found {
            Some(f) => Outcome::Path {
                path: match section {
                    Some(sec) => sec.compose(&f.path),
                    None => f.path,
                },
                pattern: false,
            },
            None => Outcome::NoPath,
        },
        expansions,
        searched: true,
    }
}

struct Ctx<'a> {
    topo: &'a Topology,
    grid: &'a GCellGrid,
    tasks: &'a [Task],
    look: &'a Lookahead,
    dir: &'a [[f32; 2]],
    settings: &'a GlobalSettings,
    pool: &'a Mutex<Vec<Scratch>>,
    sizes: (usize, usize, usize),
}

impl<'a> Ctx<'a> {
    /// The cost view over `state` for one iteration.
    fn costs<'s>(&self, state: &'s State, pres_fac: f32, seed_k: f32) -> Costs<'s>
    where
        'a: 's,
    {
        let k = |v: u32| v as f32 / 1000.0;
        Costs {
            topo: self.topo,
            cap: &self.grid.capacity,
            occ: &state.occ,
            hist: &state.hist,
            look: self.look,
            dir: self.dir,
            via: k(self.settings.costs.via),
            pres_fac,
            seed_k,
            same_net: k(self.settings.costs.same_net),
            astar: k(self.settings.costs.astar_fac),
        }
    }
}

struct IterOutcome {
    routed: u32,
    pattern: u32,
    maze: u32,
    full: u32,
}

/// Route `list` (task indices, ascending) once: batched, ripping up first.
fn iterate(
    cx: &Ctx,
    state: &mut State,
    list: &[usize],
    iteration: u32,
    pres_fac: f32,
    seed: bool,
    tel: &mut Telemetry,
) -> IterOutcome {
    let topo = cx.topo;
    let mut pool = JobPool::default();
    let mut windows = Vec::with_capacity(list.len());
    let mut wins = Vec::with_capacity(list.len());
    let every = cx.settings.full_reroute_every.max(1);
    let mut sections: Vec<Option<Section>> = Vec::with_capacity(list.len());
    for (k, &i) in list.iter().enumerate() {
        let task = &cx.tasks[i];
        let section = (!seed && state.routed[i] && !(state.reroutes[i] + 1).is_multiple_of(every))
            .then(|| state.section(topo, &cx.grid.capacity, i))
            .flatten();
        let win = match &section {
            Some(sec) => {
                let path = &state.path[i];
                let lo = sec.prefix.len() - 1;
                let hi = path.len() - sec.suffix.len();
                let core = nodes_box(topo, &path[lo..=hi]);
                window_of(topo, core, cx.settings, state.reroutes[i])
            }
            None => window_of(topo, task.core, cx.settings, state.reroutes[i]),
        };
        sections.push(section);
        let w = Window::new(
            Point::new(
                (i64::from(topo.origin.x) + i64::from(win.c0) * topo.pitch + topo.pitch / 2) as i32,
                (i64::from(topo.origin.y) + i64::from(win.r0) * topo.pitch + topo.pitch / 2) as i32,
            ),
            topo.pitch as i32,
            u32::from(win.c1 - win.c0) + 1,
            u32::from(win.r1 - win.r0) + 1,
        )
        .expect("a grid window is representable");
        let nodes = win.nodes(topo.slots);
        tel.window_nodes.record(nodes);
        let mut flags = 0;
        if task.slots.count_ones() > 1 {
            flags |= JOB_ALLOW_VIA;
        }
        if !seed {
            flags |= JOB_REROUTE;
        }
        pool.push(
            JobDescriptor {
                connection: i as u32,
                net: task.net,
                window: k as u32,
                profile: 0,
                size_class: (64 - nodes.max(1).leading_zeros()) as u8,
                flags,
                layer_mask: task.slots,
                cost_bound: u32::MAX,
                order_key: i as u32,
                iteration,
                ..Default::default()
            },
            &[],
            &[],
            &[],
        );
        windows.push(w);
        wins.push(win);
    }
    let batches = BatchBuilder::new(BatchPolicy {
        max_jobs: cx.settings.batch_max_jobs.max(1) as usize,
        split_by_size_class: false,
    })
    .build(&pool, &windows);
    tel.count("global_batches", batches.len() as u64);

    let mut out = IterOutcome {
        routed: 0,
        pattern: 0,
        maze: 0,
        full: 0,
    };
    let mut deferred: Vec<usize> = Vec::new();
    let mut buf = Vec::new();
    let seed_k = if seed {
        cx.settings.costs.seed_congestion as f32 / 1000.0
    } else {
        0.0
    };
    let sh = Shared {
        topo,
        tasks: cx.tasks,
    };
    for batch in &batches {
        for &j in &batch.jobs {
            let i = pool.jobs[j as usize].connection as usize;
            state.rip_up(cx.tasks, i);
            if let Some(sec) = &sections[j as usize] {
                state.hold(topo, cx.tasks, i, sec, &mut buf);
            }
            state.reroutes[i] += u32::from(!seed);
        }
        let results: Vec<JobResult> = {
            let st: &State = state;
            let costs = cx.costs(st, pres_fac, seed_k);
            batch
                .jobs
                .par_iter()
                .map_init(
                    || Lease::new(cx.pool, cx.sizes),
                    |lease, &j| {
                        let i = pool.jobs[j as usize].connection as usize;
                        let sec = sections[j as usize].as_ref();
                        route_one(&sh, &costs, st, i, wins[j as usize], seed, sec, lease.get())
                    },
                )
                .collect()
        };
        for (&j, res) in batch.jobs.iter().zip(results) {
            let i = pool.jobs[j as usize].connection as usize;
            if res.searched {
                tel.expansions_per_connection.record(res.expansions);
                tel.count("global_astar_expansions", res.expansions);
            }
            // Release the held ends of a partial re-route first.
            state.rip_up(cx.tasks, i);
            match res.outcome {
                Outcome::Path { path, pattern } => {
                    state.commit(topo, cx.tasks, i, path, &mut buf);
                    out.routed += 1;
                    if pattern {
                        out.pattern += 1;
                    } else {
                        out.maze += 1;
                    }
                }
                Outcome::NoPath => deferred.push(i),
            }
        }
    }
    // Windows with no path: the whole grid, serially in order.
    deferred.sort_unstable();
    let full = Win::full(topo);
    let mut lease = Lease::new(cx.pool, cx.sizes);
    for i in deferred {
        out.full += 1;
        let res = {
            let costs = cx.costs(state, pres_fac, seed_k);
            route_one(&sh, &costs, state, i, full, false, None, lease.get())
        };
        if res.searched {
            tel.expansions_per_connection.record(res.expansions);
            tel.count("global_astar_expansions", res.expansions);
        }
        match res.outcome {
            Outcome::Path { path, .. } => {
                state.commit(topo, cx.tasks, i, path, &mut buf);
                out.routed += 1;
                out.maze += 1;
            }
            Outcome::NoPath => state.failed[i] = true,
        }
    }
    out
}

/// Σ overflow, max, overused count and access overuse.
fn overuse(cap: &[u16], occ: &[u16]) -> (u64, u16, u32, u64) {
    let mut total = 0u64;
    let mut max = 0u16;
    let mut count = 0u32;
    let mut access = 0u64;
    for (&c, &o) in cap.iter().zip(occ) {
        if o > c {
            if c == 0 {
                access += u64::from(o);
            } else {
                total += u64::from(o - c);
                max = max.max(o - c);
                count += 1;
            }
        }
    }
    (total, max, count, access)
}

pub(super) fn run(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    grid: &GCellGrid,
    connections: &[Connection],
    settings: &GlobalSettings,
    policy: &LayerPolicy,
    tel: &mut Telemetry,
) -> GlobalResult {
    let t0 = Instant::now();
    if !super::grid::fits(grid) {
        // A grid from elsewhere too large for u32 ids or the memory budget
        // (build_grid never makes one): nothing can be routed on it.
        tel.count("global_grid_rejected", 1);
        return GlobalResult {
            failed: connections
                .iter()
                .map(|c| (c.id, FailureReason::NoCorridor))
                .collect(),
            ..Default::default()
        };
    }
    let topo = Topology::new(grid);
    let mut owned_grid;
    let grid = if grid.capacity.len() == topo.resources() {
        grid
    } else {
        owned_grid = grid.clone();
        owned_grid.capacity = capacity::compute(board, rules, grid, settings);
        tel.count("global_capacity_rebuilt", 1);
        &owned_grid
    };
    let slots = topo.slots as usize;
    if slots == 0 || connections.is_empty() {
        // With no routable layer, no terminal has an exit.
        return GlobalResult {
            overflow: vec![0; grid.cell_count() as usize],
            failed: connections
                .iter()
                .map(|c| (c.id, FailureReason::NoAccess { item: c.from }))
                .collect(),
            stats: GlobalStats {
                converged: connections.is_empty(),
                ..Default::default()
            },
            ..Default::default()
        };
    }
    // Preferred directions.
    let ww = 1.0 + settings.costs.wrong_way as f32 / 1000.0;
    let dir: Vec<[f32; 2]> = grid
        .layers
        .iter()
        .enumerate()
        .map(|(slot, layer)| {
            let vertical = match settings.vertical_layers {
                Some(m) => m.contains(layer),
                None => slot % 2 == 1,
            };
            if vertical {
                [ww, 1.0]
            } else {
                [1.0, ww]
            }
        })
        .collect();
    let pitch_mm = topo.pitch as f32 / 1e6;
    let hc: Vec<f32> = dir.iter().map(|d| d[0] * pitch_mm).collect();
    let vc: Vec<f32> = dir.iter().map(|d| d[1] * pitch_mm).collect();
    let via = settings.costs.via as f32 / 1000.0;
    let tl = Instant::now();
    // The table holds slots² planes; past the budget it covers a smaller
    // offset range (larger offsets read its edge, a weaker bound).
    let plane = (LOOKAHEAD_ENTRIES / (slots * slots) as u64).max(1);
    let (lc, lr) = {
        let (c, r) = (u64::from(topo.cols), u64::from(topo.rows));
        if c * r <= plane {
            (c, r)
        } else {
            let f = (plane as f64 / (c * r) as f64).sqrt();
            (
                ((c as f64 * f) as u64).clamp(1, c),
                ((r as f64 * f) as u64).clamp(1, r),
            )
        }
    };
    let look = Lookahead::build(slots, lc as usize, lr as usize, &hc, &vc, via);
    tel.count("global_lookahead_us", tl.elapsed().as_micros() as u64);

    let tp = Instant::now();
    let tasks = prepare(board, rules, grid, &topo, connections, policy);
    tel.count("global_prepare_us", tp.elapsed().as_micros() as u64);
    tel.count("global_setup_us", t0.elapsed().as_micros() as u64);

    let n = tasks.len();
    let res_count = topo.resources();
    let mut state = State {
        via_from: topo.via_res(0),
        occ: vec![0; res_count],
        hist: vec![0.0; res_count],
        net_res: vec![
            BTreeMap::new();
            board.nets.len().max(
                connections
                    .iter()
                    .map(|c| c.net.index() + 1)
                    .max()
                    .unwrap_or(0)
            )
        ],
        path: vec![Vec::new(); n],
        res: vec![Vec::new(); n],
        routed: vec![false; n],
        failed: tasks.iter().map(|t| t.unroutable.is_some()).collect(),
        reroutes: vec![0; n],
    };
    let pool = Mutex::new(Vec::new());
    let cx = Ctx {
        topo: &topo,
        grid,
        tasks: &tasks,
        look: &look,
        dir: &dir,
        settings,
        pool: &pool,
        sizes: (topo.nodes(), res_count, topo.nc as usize),
    };
    let cap = &grid.capacity;
    let acc = settings.schedule.acc_fac as f32 / 1000.0;
    let update_history = |state: &mut State| {
        for ((h, &c), &o) in state.hist.iter_mut().zip(cap.iter()).zip(state.occ.iter()) {
            if c > 0 && o > c {
                *h += acc * f32::from(o - c);
            }
        }
    };
    let mut stats = GlobalStats::default();

    // Seed.
    let ts = Instant::now();
    let list: Vec<usize> = (0..n).filter(|&i| tasks[i].unroutable.is_none()).collect();
    let first = settings.schedule.first_iter_pres_fac as f32 / 1000.0;
    let o = iterate(&cx, &mut state, &list, 0, first, true, tel);
    stats.pattern_seeds = o.pattern;
    stats.maze_seeds = o.maze;
    stats.full_grid_searches += o.full;
    let (total, _, count, _) = overuse(cap, &state.occ);
    stats.per_iteration.push(IterationStats {
        iteration: 0,
        pres_fac: settings.schedule.first_iter_pres_fac,
        routed: o.routed,
        overflow: total,
        overused: count,
    });
    update_history(&mut state);
    let mut keep = Snapshot::take(&state, cap, 0);
    tel.count("global_seed_us", ts.elapsed().as_micros() as u64);

    // Negotiation.
    let mut iterations = 0;
    let mut pres = settings.schedule.initial_pres_fac as f32 / 1000.0;
    let mult = settings.schedule.pres_fac_mult as f32 / 1000.0;
    let max_pres = settings.schedule.max_pres_fac as f32 / 1000.0;
    let mut converged = total == 0;
    let mut best = total;
    let mut stalled = 0u32;
    while !converged && iterations < settings.max_iterations {
        if settings.stall_iterations > 0 && stalled >= settings.stall_iterations {
            tel.count("global_stalled", 1);
            break;
        }
        let ti = Instant::now();
        iterations += 1;
        let list: Vec<usize> = (0..n)
            .filter(|&i| {
                !state.failed[i]
                    && (!state.routed[i] || state.res[i].iter().any(|&r| state.overused(cap, r)))
            })
            .collect();
        let o = iterate(
            &cx,
            &mut state,
            &list,
            iterations,
            pres.min(max_pres),
            false,
            tel,
        );
        stats.full_grid_searches += o.full;
        let (total, _, count, _) = overuse(cap, &state.occ);
        stats.per_iteration.push(IterationStats {
            iteration: iterations,
            pres_fac: (pres.min(max_pres) * 1000.0).round() as u32,
            routed: o.routed,
            overflow: total,
            overused: count,
        });
        tel.count("global_reroutes", u64::from(o.routed));
        update_history(&mut state);
        if Snapshot::key(&state, cap) < keep.key {
            keep = Snapshot::take(&state, cap, iterations);
        }
        pres = (pres * mult).min(max_pres);
        converged = total == 0;
        if total + (best / 100).max(1) <= best {
            best = total;
            stalled = 0;
        } else {
            stalled += 1;
        }
        tel.congested_iteration_us += ti.elapsed().as_micros() as u64;
    }

    // Result: the best iteration (fewest unrouted, then least overflow;
    // the earliest on a tie), not necessarily the last.
    stats.best_iteration = keep.iteration;
    if keep.iteration != iterations {
        keep.restore(&mut state);
    }
    let (total, max, count, access) = overuse(cap, &state.occ);
    stats.converged = total == 0 && state.routed.iter().all(|&r| r);
    stats.total_overflow = total;
    stats.max_overflow = max;
    stats.overused_resources = count;
    stats.access_overuse = access;
    let mut corridors = Vec::new();
    let mut failed = Vec::new();
    let mut overflowed = Vec::new();
    let mut net_vias: BTreeSet<(u32, u32)> = BTreeSet::new();
    for (i, c) in connections.iter().enumerate() {
        if !state.routed[i] {
            let reason = tasks[i]
                .unroutable
                .clone()
                .unwrap_or(FailureReason::NoCorridor);
            failed.push((c.id, reason));
            continue;
        }
        let path = &state.path[i];
        let mut assigned = LayerMask::EMPTY;
        let mut cells = Vec::with_capacity(path.len());
        for w in 0..path.len() {
            let (slot, cell) = topo.split(path[w]);
            assigned = assigned.with(grid.slot_layer(slot).expect("slot is on the grid"));
            cells.push(topo.gcell_id(slot, cell));
            if w + 1 < path.len() {
                let (s2, c2) = topo.split(path[w + 1]);
                if s2 == slot {
                    let (a, b) = (topo.center(cell), topo.center(c2));
                    stats.wirelength_nm += ((i64::from(a.0) - i64::from(b.0)).abs()
                        + (i64::from(a.1) - i64::from(b.1)).abs())
                        as u64;
                }
            }
        }
        // A via site a net's connections share holds one via of that net.
        for &r in &state.res[i] {
            if r >= state.via_from {
                net_vias.insert((tasks[i].net, r));
            }
        }
        if state.res[i].iter().any(|&r| state.overused(cap, r)) {
            overflowed.push(c.id);
        }
        corridors.push(Corridor {
            connection: c.id,
            cells,
            layers: slot_layers(grid, tasks[i].slots),
            assigned,
        });
    }
    stats.vias = net_vias.len() as u64;
    let mut overflow = vec![0u16; grid.cell_count() as usize];
    for slot in 0..topo.slots {
        for e in 0..topo.edges {
            let r = topo.planar_res(slot, e) as usize;
            let (c, o) = (cap[r], state.occ[r]);
            if c > 0 && o > c {
                let id = topo.gcell_id(slot, topo.edge_a[e as usize]) as usize;
                overflow[id] = overflow[id].saturating_add(o - c);
            }
        }
    }
    for cell in 0..topo.nc {
        let r = topo.via_res(cell) as usize;
        let (c, o) = (cap[r], state.occ[r]);
        if c > 0 && o > c {
            for slot in 0..topo.slots {
                let id = topo.gcell_id(slot, cell) as usize;
                overflow[id] = overflow[id].saturating_add(o - c);
            }
        }
    }
    let nets_routed: BTreeSet<NetId> = connections
        .iter()
        .enumerate()
        .filter(|(i, _)| state.routed[*i])
        .map(|(_, c)| c.net)
        .collect();
    tel.count("global_connections", n as u64);
    tel.count("global_nets_routed", nets_routed.len() as u64);
    tel.count("global_iterations", u64::from(iterations));
    tel.count("global_pattern_seeds", u64::from(stats.pattern_seeds));
    tel.count("global_maze_seeds", u64::from(stats.maze_seeds));
    tel.count(
        "global_full_grid_searches",
        u64::from(stats.full_grid_searches),
    );
    tel.count("global_failed", failed.len() as u64);
    tel.count("global_overflow_final", total);
    GlobalResult {
        corridors,
        overflowed,
        overflow,
        failed,
        iterations,
        stats,
    }
}

/// The routing state of one iteration, kept to return the best.
struct Snapshot {
    iteration: u32,
    /// Connections without a path, then total overflow.
    key: (usize, u64),
    path: Vec<Vec<u32>>,
    res: Vec<Vec<u32>>,
    routed: Vec<bool>,
    failed: Vec<bool>,
    occ: Vec<u16>,
}

impl Snapshot {
    fn key(state: &State, cap: &[u16]) -> (usize, u64) {
        let unrouted = state.routed.iter().filter(|&&r| !r).count();
        (unrouted, overuse(cap, &state.occ).0)
    }

    fn take(state: &State, cap: &[u16], iteration: u32) -> Snapshot {
        Snapshot {
            iteration,
            key: Snapshot::key(state, cap),
            path: state.path.clone(),
            res: state.res.clone(),
            routed: state.routed.clone(),
            failed: state.failed.clone(),
            occ: state.occ.clone(),
        }
    }

    /// Put the kept paths and occupancy back (history and the per-net maps
    /// are not needed after the last iteration).
    fn restore(self, state: &mut State) {
        state.path = self.path;
        state.res = self.res;
        state.routed = self.routed;
        state.failed = self.failed;
        state.occ = self.occ;
    }
}
