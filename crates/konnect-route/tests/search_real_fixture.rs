//! Real-board benchmark of the S5 search kernel. Nothing about the board is
//! committed: the directory comes from `KONNECT_DB1_FIXTURE_DIR`, files are
//! found by extension, and the test asserts only invariants (paths are
//! octilinear, never turn 135°, and pass an exact re-check).
//!
//! ```text
//! KONNECT_DB1_FIXTURE_DIR=/path/to/fixture cargo test --release \
//!     -p konnect-route --test search_real_fixture -- --nocapture
//! ```
//!
//! Optional: `KONNECT_SEARCH_TILES` (tiles to route, default 3, `all` for
//! every tile), `KONNECT_SEARCH_VARIANTS` (`all` adds the slower variants)
//! and `KONNECT_SEARCH_REFERENCE_DIR` (a directory holding a routed copy of
//! the same board, whose two-pin net lengths are compared). For timing on a
//! busy machine, `KONNECT_SEARCH_ONLY` (comma-separated name filters) and
//! `KONNECT_SEARCH_REPEAT` (runs per variant; take the minimum). For each chosen tile and net class it paints the fixed copper
//! into rasters, then routes every connection whose two islands fit in the
//! tile, one at a time against the fixed board (no negotiation): the own
//! net is unpainted around each search and repainted after. It prints
//! expansions per second, milliseconds per connection and path length
//! against the straight line, per variant.

use konnect_geom::{
    gap_cmp, touches, ArcPolicy, BBox, Circle, Coord, IndexEntry, LayerIndex, Point, Segment, Shape,
};
use konnect_route::api::{
    ClassId, FootprintId, ItemKind, ItemRef, LayerId, LayerMask, NetId, ProfileId, Stage, TileId,
    TILE_CORE_NM, TILE_HALO_NM,
};
use konnect_route::board::PadKind;
use konnect_route::compute::job::JOB_ALLOW_VIA;
use konnect_route::compute::{
    CostParams, JobDescriptor, JobNode, JobPool, PaintKind, PaintShape, ProfileGeometry,
    SearchOutcome, TileRaster, TileStack,
};
use konnect_route::connectivity::ConnectivityReport;
use konnect_route::detail::search::{
    derive_masks_in_lattice, derive_masks_on_lattice, paint_on_lattice, search_with, CheckCache,
    ExactCheck, Heuristic, Lattice, LatticeSpec, MapLookahead, Preferred, SearchConfig,
    SearchContext, SearchStats, ViaGeometry,
};
use konnect_route::rules::RuleSubject;
use konnect_route::telemetry::Telemetry;
use konnect_route::{load_board_file, LoadOptions, ProjectRuleTable, RouteBoard, RuleTable};
use rayon::prelude::*;
use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::path::{Path, PathBuf};
use std::sync::Mutex;
use std::time::Instant;

fn only_file(dir: &Path, ext: &str) -> Option<PathBuf> {
    let mut found = std::fs::read_dir(dir)
        .ok()?
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|e| e.to_str()) == Some(ext));
    let first = found.next()?;
    found.next().is_none().then_some(first)
}

/// CPU cycles this thread has run, where the platform reports them: robust
/// against preemption on a busy machine, unlike wall time.
#[cfg(windows)]
fn thread_cycles() -> u64 {
    #[link(name = "kernel32")]
    extern "system" {
        fn GetCurrentThread() -> *mut std::ffi::c_void;
        fn QueryThreadCycleTime(thread: *mut std::ffi::c_void, cycles: *mut u64) -> i32;
    }
    let mut c = 0u64;
    // SAFETY: the pseudo-handle of the current thread is always valid and
    // `c` outlives the call.
    unsafe {
        QueryThreadCycleTime(GetCurrentThread(), &mut c);
    }
    c
}

#[cfg(not(windows))]
fn thread_cycles() -> u64 {
    0
}

// ─── Obstacles and the exact checker ─────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ObKind {
    Item(ItemKind),
    /// A drilled hole (any pad drill or via drill): hole clearance, and
    /// hole-to-hole for vias.
    Hole,
    Edge,
    /// A rule area forbidding tracks (and, if `vias`, vias).
    Keepout {
        vias: bool,
    },
}

#[derive(Debug, Clone)]
struct Obstacle {
    shape: Shape,
    net: Option<NetId>,
    kind: ObKind,
    layers: LayerMask,
    footprint: Option<FootprintId>,
    local: Option<Coord>,
    courtyards: Vec<FootprintId>,
}

struct Courtyards {
    index: LayerIndex,
    shapes: Vec<(FootprintId, Shape)>,
}

impl Courtyards {
    fn new(board: &RouteBoard) -> Self {
        let mut shapes = Vec::new();
        for f in &board.footprints {
            for c in &f.courtyards {
                shapes.push((f.id, Shape::Polygon(c.region.clone())));
            }
        }
        let entries = shapes
            .iter()
            .enumerate()
            .map(|(i, (_, s))| IndexEntry::new(i as u32, s.bbox()))
            .collect();
        Courtyards {
            index: LayerIndex::bulk_load(entries),
            shapes,
        }
    }

    /// Up to 8 footprints whose courtyard touches `s`, sorted, without
    /// allocating.
    fn touching_small(&self, s: &Shape) -> ([FootprintId; 8], usize) {
        let mut out = [FootprintId(0); 8];
        let mut n = 0;
        for e in self.index.query_entries(&s.bbox()) {
            let (f, shape) = &self.shapes[e.id as usize];
            if n < out.len() && !out[..n].contains(f) && touches(shape, s) {
                out[n] = *f;
                n += 1;
            }
        }
        out[..n].sort_unstable();
        (out, n)
    }

    fn touching(&self, s: &Shape) -> Vec<FootprintId> {
        let mut out: Vec<FootprintId> = self
            .index
            .query(&s.bbox())
            .into_iter()
            .filter(|&i| touches(&self.shapes[i as usize].1, s))
            .map(|i| self.shapes[i as usize].0)
            .collect();
        out.sort_unstable();
        out.dedup();
        out
    }
}

fn obstacles(board: &RouteBoard, cy: &Courtyards) -> Vec<Obstacle> {
    let all = board.layers.all();
    let mut out = Vec::new();
    for pad in &board.pads {
        for s in &pad.shapes {
            if pad.layers.is_empty() {
                continue;
            }
            out.push(Obstacle {
                shape: s.clone(),
                net: pad.net,
                kind: ObKind::Item(ItemKind::Pad),
                layers: pad.layers,
                footprint: Some(pad.footprint),
                local: pad.local_clearance,
                courtyards: Vec::new(),
            });
        }
        if let Some(d) = &pad.drill {
            out.push(Obstacle {
                shape: d.shape.clone(),
                net: if pad.kind == PadKind::NpThroughHole {
                    None
                } else {
                    pad.net
                },
                kind: ObKind::Hole,
                layers: all,
                footprint: Some(pad.footprint),
                local: None,
                courtyards: Vec::new(),
            });
        }
    }
    for t in &board.tracks {
        out.push(Obstacle {
            shape: Shape::Segment(t.seg),
            net: t.net,
            kind: ObKind::Item(ItemKind::Track),
            layers: LayerMask::single(t.layer),
            footprint: None,
            local: None,
            courtyards: Vec::new(),
        });
    }
    for a in &board.arcs {
        for seg in a.segments(&ArcPolicy::default()) {
            out.push(Obstacle {
                shape: Shape::Segment(seg),
                net: a.net,
                kind: ObKind::Item(ItemKind::Arc),
                layers: LayerMask::single(a.layer),
                footprint: None,
                local: None,
                courtyards: Vec::new(),
            });
        }
    }
    for v in &board.vias {
        out.push(Obstacle {
            shape: v.shape(),
            net: v.net,
            kind: ObKind::Item(ItemKind::Via),
            layers: v.layers,
            footprint: None,
            local: None,
            courtyards: Vec::new(),
        });
        out.push(Obstacle {
            shape: v.hole(),
            net: v.net,
            kind: ObKind::Hole,
            layers: all,
            footprint: None,
            local: None,
            courtyards: Vec::new(),
        });
    }
    for k in &board.keepouts {
        if k.rules.tracks || k.rules.vias {
            out.push(Obstacle {
                shape: Shape::Polygon(k.region.clone()),
                net: None,
                kind: ObKind::Keepout { vias: k.rules.vias },
                layers: k.layers,
                footprint: None,
                local: None,
                courtyards: Vec::new(),
            });
        }
    }
    for e in &board.outline.edges {
        out.push(Obstacle {
            shape: Shape::Segment(*e),
            net: None,
            kind: ObKind::Edge,
            layers: all,
            footprint: None,
            local: None,
            courtyards: Vec::new(),
        });
    }
    out.par_iter_mut().for_each(|o| {
        if matches!(o.kind, ObKind::Item(_) | ObKind::Hole) {
            o.courtyards = cy.touching(&o.shape);
        }
    });
    out
}

/// The clearance a track or via of `net` (touching `track_cy`) needs from
/// `ob` on `layer`.
fn clearance_for(
    rules: &dyn RuleTable,
    kind: ItemKind,
    net: NetId,
    track_cy: &[FootprintId],
    ob: &Obstacle,
    layer: LayerId,
) -> Coord {
    match ob.kind {
        ObKind::Edge => rules.edge_clearance(),
        ObKind::Hole => rules.hole_clearance(),
        ObKind::Keepout { .. } => 0,
        ObKind::Item(k) => {
            let a = RuleSubject::new(kind, Some(net)).with_courtyards(track_cy);
            let mut b = RuleSubject::new(k, ob.net)
                .with_courtyards(&ob.courtyards)
                .with_local_clearance(ob.local);
            if let Some(f) = ob.footprint {
                b = b.with_footprint(f);
            }
            rules.clearance(&a, &b, layer)
        }
    }
}

/// Whether `ob` is the job net's own copper (ignored by the search).
fn own(ob: &Obstacle, net: NetId) -> bool {
    ob.net == Some(net) && matches!(ob.kind, ObKind::Item(_) | ObKind::Hole)
}

struct Board<'a> {
    board: &'a RouteBoard,
    rules: &'a dyn RuleTable,
    obs: Vec<Obstacle>,
    /// Per copper layer.
    index: Vec<LayerIndex>,
    cy: Courtyards,
    max_reach: Coord,
}

impl<'a> Board<'a> {
    fn new(board: &'a RouteBoard, rules: &'a dyn RuleTable) -> Self {
        let cy = Courtyards::new(board);
        let obs = obstacles(board, &cy);
        let index = (0..board.layer_count())
            .map(|l| {
                let entries = obs
                    .iter()
                    .enumerate()
                    .filter(|(_, o)| o.layers.contains(LayerId(l as u8)))
                    .map(|(i, o)| IndexEntry::new(i as u32, o.shape.bbox()))
                    .collect();
                LayerIndex::bulk_load(entries)
            })
            .collect();
        let max_reach = rules.max_clearance();
        Board {
            board,
            rules,
            obs,
            index,
            cy,
            max_reach,
        }
    }
}

/// Per-profile geometry the checker needs, indexed by `job.profile`.
#[derive(Debug, Clone, Copy)]
struct ProfileInfo {
    width: Coord,
    via_diameter: Coord,
    via_drill: Coord,
}

/// (job net, obstacle, layer, track courtyards hash, item kind).
type MemoKey = (u32, u32, u8, u64, u8);
/// Map lookahead tables by (cols, rows, pitch).
type Lookaheads = Mutex<BTreeMap<(u32, u32, Coord), std::sync::Arc<MapLookahead>>>;

struct Checker<'a> {
    b: &'a Board<'a>,
    /// Stack slot to copper layer.
    layers: Vec<LayerId>,
    profiles: Vec<ProfileInfo>,
    memo: Mutex<HashMap<MemoKey, Coord>>,
}

fn cy_key(cy: &[FootprintId]) -> u64 {
    cy.iter().fold(1469598103934665603u64, |h, f| {
        (h ^ u64::from(f.0)).wrapping_mul(1099511628211)
    })
}

impl Checker<'_> {
    fn clearance(
        &self,
        kind: ItemKind,
        net: NetId,
        cy: &[FootprintId],
        ob_id: u32,
        layer: LayerId,
    ) -> Coord {
        let key = (net.0, ob_id, layer.0, cy_key(cy), kind as u8);
        if let Some(&c) = self.memo.lock().unwrap().get(&key) {
            return c;
        }
        let c = clearance_for(
            self.b.rules,
            kind,
            net,
            cy,
            &self.b.obs[ob_id as usize],
            layer,
        );
        self.memo.lock().unwrap().insert(key, c);
        c
    }

    fn shape_ok(
        &self,
        kind: ItemKind,
        net: NetId,
        shape: &Shape,
        layer: LayerId,
        cy: &[FootprintId],
    ) -> bool {
        let q = shape.bbox().inflate(self.b.max_reach + 1);
        for e in self.b.index[layer.index()].query_entries(&q) {
            let id = e.id;
            let ob = &self.b.obs[id as usize];
            if own(ob, net) {
                continue;
            }
            if let ObKind::Keepout { vias } = ob.kind {
                if kind == ItemKind::Via && !vias {
                    continue;
                }
            }
            let c = self.clearance(kind, net, cy, id, layer);
            if gap_cmp(&ob.shape, shape, i64::from(c)) == Ordering::Less {
                return false;
            }
        }
        true
    }

    fn hole_ok(&self, net: NetId, at: Point, drill: Coord) -> bool {
        let hole = Shape::Circle(Circle::from_diameter(at, drill));
        let h2h = i64::from(self.b.rules.hole_to_hole());
        let q = hole.bbox().inflate(self.b.rules.hole_to_hole() + 1);
        // Holes span every layer, so layer 0's index holds them all.
        // Hole-to-hole applies whatever the nets.
        let _ = net;
        self.b.index[0].query_entries(&q).all(|e| {
            let ob = &self.b.obs[e.id as usize];
            ob.kind != ObKind::Hole || gap_cmp(&ob.shape, &hole, h2h) != Ordering::Less
        })
    }
}

impl ExactCheck for Checker<'_> {
    fn segment_ok(&self, job: &JobDescriptor, layer: u8, a: Point, b: Point) -> bool {
        let p = self.profiles[job.profile as usize];
        let seg = Shape::Segment(Segment::new(a, b, p.width));
        let (cy, n) = self.b.cy.touching_small(&seg);
        self.shape_ok(
            ItemKind::Track,
            NetId(job.net),
            &seg,
            self.layers[layer as usize],
            &cy[..n],
        )
    }

    fn via_ok(&self, job: &JobDescriptor, at: Point) -> bool {
        let p = self.profiles[job.profile as usize];
        let disc = Shape::Circle(Circle::from_diameter(at, p.via_diameter));
        let (cy, n) = self.b.cy.touching_small(&disc);
        let cy = &cy[..n];
        let net = NetId(job.net);
        for l in self.b.board.layers.copper.iter().map(|c| c.id) {
            if !self.shape_ok(ItemKind::Via, net, &disc, l, cy) {
                return false;
            }
        }
        self.hole_ok(net, at, p.via_drill)
    }
}

// ─── Tiles, stacks and jobs ──────────────────────────────────────────────────

struct Conn {
    net: NetId,
    class: ClassId,
    from_pad: Point,
    to_pad: Point,
    from: Vec<ItemRef>,
    to: Vec<ItemRef>,
    bbox: BBox,
}

fn item_bbox(board: &RouteBoard, it: ItemRef) -> Option<BBox> {
    match it {
        ItemRef::Pad(p) => board.pads[p.index()].bbox(),
        ItemRef::Track(t) => Some(board.tracks[t.index()].seg.bbox()),
        ItemRef::Arc(a) => {
            let a = &board.arcs[a.index()];
            BBox::from_points([a.start, a.mid, a.end]).map(|b| b.inflate(a.width))
        }
        ItemRef::Via(v) => Some(board.vias[v.index()].shape().bbox()),
        ItemRef::Zone(_) => None,
    }
}

/// Lattice nodes on an island's copper, per stack slot.
fn terminals(
    board: &RouteBoard,
    lat: &Lattice,
    slots: &[LayerId],
    items: &[ItemRef],
) -> Vec<JobNode> {
    let mut out = BTreeSet::new();
    let mut add_shape = |shape: &Shape, layers: LayerMask, force_nearest: Option<Point>| {
        let b = shape.bbox();
        let xs = lat.xs();
        let ys = lat.ys();
        let c0 = xs.partition_point(|&x| x < b.min.x);
        let c1 = xs.partition_point(|&x| x <= b.max.x);
        let r0 = ys.partition_point(|&y| y < b.min.y);
        let r1 = ys.partition_point(|&y| y <= b.max.y);
        for (slot, l) in slots.iter().enumerate() {
            if !layers.contains(*l) {
                continue;
            }
            let mut any = false;
            for (r, &y) in ys.iter().enumerate().take(r1).skip(r0) {
                for (c, &x) in xs.iter().enumerate().take(c1).skip(c0) {
                    let p = Point::new(x, y);
                    if touches(&Shape::Circle(Circle::new(p, 0)), shape) {
                        out.insert((c as u16, r as u16, slot as u8));
                        any = true;
                    }
                }
            }
            if !any {
                if let Some(p) = force_nearest {
                    if let Some((c, r)) = lat.nearest(p) {
                        out.insert((c as u16, r as u16, slot as u8));
                    }
                }
            }
        }
    };
    for &it in items {
        match it {
            ItemRef::Pad(p) => {
                let pad = &board.pads[p.index()];
                for s in &pad.shapes {
                    add_shape(s, pad.layers, Some(pad.center));
                }
            }
            ItemRef::Track(t) => {
                let t = &board.tracks[t.index()];
                add_shape(&Shape::Segment(t.seg), LayerMask::single(t.layer), None);
            }
            ItemRef::Via(v) => {
                let v = &board.vias[v.index()];
                add_shape(&v.shape(), v.layers, Some(v.center));
            }
            ItemRef::Arc(_) | ItemRef::Zone(_) => {}
        }
    }
    out.into_iter()
        .map(|(col, row, layer)| JobNode {
            col,
            row,
            layer,
            heading_mask: 0xff,
            reserved: 0,
        })
        .collect()
}

/// One painted shape with the obstacle it came from.
#[derive(Clone)]
struct Painted {
    ob: u32,
    paint: PaintShape,
}

#[derive(Clone)]
struct ClassStack {
    stack: TileStack,
    /// Per slot.
    painted: Vec<Vec<Painted>>,
    profile: ProfileGeometry,
    via: ViaGeometry,
}

fn build_stack(
    b: &Board<'_>,
    lat: &Lattice,
    slots: &[LayerId],
    nets: &[NetId],
    width: Coord,
    via_d: Coord,
    tile: TileId,
) -> ClassStack {
    let w = lat.window();
    let profile = ProfileGeometry {
        track_width: width,
        sagitta: 0,
    };
    let half = width / 2 + width % 2;
    let via = ViaGeometry {
        radius: via_d / 2 + via_d % 2,
        track_half_width: half,
    };
    let span = BBox::new(lat.node(0, 0), lat.node(lat.cols() - 1, lat.rows() - 1));
    let results: Vec<(TileRaster, Vec<Painted>)> = slots
        .par_iter()
        .map(|&layer| {
            let mut r = TileRaster::new(tile, layer, ProfileId(0), w);
            let q = span.inflate(b.max_reach + half + lat.base_pitch());
            let ids = b.index[layer.index()].query(&q);
            let painted: Vec<Painted> = ids
                .par_iter()
                .filter_map(|&id| {
                    let ob = &b.obs[id as usize];
                    if let ObKind::Keepout { .. } = ob.kind {
                        // Tracks forbidden: zero clearance.
                    }
                    let c = nets
                        .iter()
                        .map(|&n| {
                            let base = clearance_for(b.rules, ItemKind::Track, n, &[], ob, layer);
                            let in_cy = clearance_for(
                                b.rules,
                                ItemKind::Track,
                                n,
                                &ob.courtyards,
                                ob,
                                layer,
                            );
                            base.max(in_cy)
                        })
                        .max()?;
                    Some(Painted {
                        ob: id,
                        paint: PaintShape {
                            shape: ob.shape.clone(),
                            clearance: c,
                            kind: PaintKind::Hard,
                        },
                    })
                })
                .collect();
            let shapes: Vec<PaintShape> = painted.iter().map(|p| p.paint.clone()).collect();
            paint_on_lattice(&mut r, lat, &shapes, &profile, true);
            derive_masks_on_lattice(&mut r, lat, Some(via));
            (r, painted)
        })
        .collect();
    let (layers, painted) = results.into_iter().unzip();
    ClassStack {
        stack: TileStack {
            tile,
            profile: ProfileId(0),
            window: w,
            layers,
        },
        painted,
        profile,
        via,
    }
}

/// Unpaint (or repaint) the job net's own copper and re-derive the masks
/// around it.
fn toggle_own(cs: &mut ClassStack, b: &Board<'_>, lat: &Lattice, net: NetId, up: bool) {
    for (slot, list) in cs.painted.iter().enumerate() {
        let mine: Vec<PaintShape> = list
            .iter()
            .filter(|p| own(&b.obs[p.ob as usize], net))
            .map(|p| p.paint.clone())
            .collect();
        if mine.is_empty() {
            continue;
        }
        let changed = mine
            .iter()
            .map(|p| {
                p.shape
                    .bbox()
                    .inflate(p.clearance + cs.profile.track_width + lat.base_pitch())
            })
            .reduce(BBox::union)
            .unwrap_or_else(|| unreachable!());
        let r = &mut cs.stack.layers[slot];
        paint_on_lattice(r, lat, &mine, &cs.profile, up);
        derive_masks_in_lattice(r, lat, Some(cs.via), changed);
    }
}

// ─── Variants and results ────────────────────────────────────────────────────

#[derive(Clone)]
struct Variant {
    name: &'static str,
    /// Route every tile twice through one cross-search `CheckCache` and
    /// report the second pass (a negotiation iteration's reroute).
    cached: bool,
    non_uniform: bool,
    config: SearchConfig,
    params: CostParams,
    checker: bool,
}

#[derive(Default, Clone)]
struct Tally {
    jobs: u64,
    found: u64,
    unreachable: u64,
    budget: u64,
    expansions: u64,
    search_us: u64,
    times_us: Vec<u64>,
    len_ratio: Vec<f64>,
    octile_ratio: Vec<f64>,
    vias: u64,
    violations: u64,
    bad_turns: u64,
    stats: SearchStats,
    window_nodes: u64,
    lattice_nodes: u64,
    raster_ms: f64,
    reference: Vec<f64>,
    tiles: u64,
    cache_hits: u64,
    cache_misses: u64,
    cycles: u64,
    cycle_list: Vec<u64>,
    fail_cycles: u64,
    found_max_exp: u64,
}

impl Tally {
    fn merge(&mut self, o: &Tally) {
        self.jobs += o.jobs;
        self.found += o.found;
        self.unreachable += o.unreachable;
        self.budget += o.budget;
        self.expansions += o.expansions;
        self.search_us += o.search_us;
        self.times_us.extend(&o.times_us);
        self.len_ratio.extend(&o.len_ratio);
        self.octile_ratio.extend(&o.octile_ratio);
        self.vias += o.vias;
        self.violations += o.violations;
        self.bad_turns += o.bad_turns;
        self.stats.add(&o.stats);
        self.window_nodes += o.window_nodes;
        self.lattice_nodes += o.lattice_nodes;
        self.raster_ms += o.raster_ms;
        self.reference.extend(&o.reference);
        self.tiles += o.tiles;
        self.cache_hits += o.cache_hits;
        self.cycles += o.cycles;
        self.fail_cycles += o.fail_cycles;
        self.found_max_exp = self.found_max_exp.max(o.found_max_exp);
        self.cycle_list.extend(&o.cycle_list);
        self.cache_misses += o.cache_misses;
    }
}

fn pct(v: &mut [f64], q: f64) -> f64 {
    if v.is_empty() {
        return f64::NAN;
    }
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let i = ((v.len() - 1) as f64 * q).round() as usize;
    v[i]
}

fn dist(a: Point, b: Point) -> f64 {
    let dx = f64::from(b.x) - f64::from(a.x);
    let dy = f64::from(b.y) - f64::from(a.y);
    (dx * dx + dy * dy).sqrt()
}

struct TileJob {
    tile: usize,
    window: BBox,
    conns: Vec<usize>,
}

#[allow(clippy::too_many_arguments)]
fn route_tile(
    b: &Board<'_>,
    tj: &TileJob,
    conns: &[Conn],
    slots: &[LayerId],
    v: &Variant,
    fine: &[BBox],
    lookahead: &Lookaheads,
    reference_len: &BTreeMap<NetId, f64>,
    single_conn_nets: &BTreeSet<NetId>,
    telemetry: &Mutex<Telemetry>,
) -> Tally {
    let mut tally = Tally::default();
    let t0 = Instant::now();
    let base = konnect_route::api::LATTICE_PITCH_NM;
    let lat = if v.non_uniform {
        let mut spec = LatticeSpec::new(tj.window);
        spec.fine_regions = fine
            .iter()
            .filter(|f| f.intersects(&tj.window))
            .copied()
            .collect();
        // Lines through every terminal pad centre and every fine-field
        // pad centre in the window.
        for &ci in &tj.conns {
            for p in [conns[ci].from_pad, conns[ci].to_pad] {
                spec.forced_x.push(p.x);
                spec.forced_y.push(p.y);
            }
        }
        for pad in &b.board.pads {
            if fine.iter().any(|f| f.contains(pad.center)) && tj.window.contains(pad.center) {
                spec.forced_x.push(pad.center.x);
                spec.forced_y.push(pad.center.y);
            }
        }
        Lattice::build(&spec).expect("lattice builds")
    } else {
        let cols = (tj.window.width() / i64::from(base)) as u32 + 1;
        let rows = (tj.window.height() / i64::from(base)) as u32 + 1;
        Lattice::uniform(
            konnect_route::compute::Window::new(tj.window.min, base, cols, rows).unwrap(),
        )
    };
    tally.lattice_nodes = lat.node_count() as u64;
    tally.tiles = 1;
    let la = if v.config.heuristic == Heuristic::Map {
        // One table per pitch, covering any tile window's offsets.
        let key = (0, 0, lat.base_pitch());
        let mut m = lookahead.lock().unwrap();
        let e = m.entry(key).or_insert_with(|| {
            let max_dx = ((TILE_CORE_NM.0 + 2 * TILE_HALO_NM) / base) as u32 + 1;
            let max_dy = ((TILE_CORE_NM.1 + 2 * TILE_HALO_NM) / base) as u32 + 1;
            let t = Instant::now();
            let la = MapLookahead::build(slots.len(), max_dx, max_dy, base, &v.params, &v.config)
                .unwrap();
            println!(
                "[{}]   map lookahead {max_dx} x {max_dy} x {} layers built in {} ms",
                v.name,
                slots.len(),
                t.elapsed().as_millis()
            );
            std::sync::Arc::new(la)
        });
        Some(e.clone())
    } else {
        None
    };
    // Group by class.
    let mut by_class: BTreeMap<ClassId, Vec<usize>> = BTreeMap::new();
    for &ci in &tj.conns {
        by_class.entry(conns[ci].class).or_default().push(ci);
    }
    let mut profiles = Vec::new();
    let mut groups = Vec::new();
    for (class, list) in &by_class {
        let rep = conns[list[0]].net;
        let ts = RuleSubject::new(ItemKind::Track, Some(rep));
        let width = b.rules.track_width(&ts, slots[0]).preferred;
        let via = b.rules.via(&RuleSubject::new(ItemKind::Via, Some(rep)));
        profiles.push(ProfileInfo {
            width,
            via_diameter: via.diameter,
            via_drill: via.drill,
        });
        groups.push((*class, list.clone(), width, via.diameter));
    }
    let checker = Checker {
        b,
        layers: slots.to_vec(),
        profiles: profiles.clone(),
        memo: Mutex::new(HashMap::new()),
    };
    let shared = CheckCache::new(&checker);
    let check: &dyn ExactCheck = if v.cached { &shared } else { &checker };
    let passes = if v.cached { 2 } else { 1 };
    tally.raster_ms += t0.elapsed().as_secs_f64() * 1e3;
    for (pi, (_class, list, width, via_d)) in groups.into_iter().enumerate() {
        let t1 = Instant::now();
        let nets: Vec<NetId> = list
            .iter()
            .map(|&c| conns[c].net)
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect();
        let mut cs = build_stack(b, &lat, slots, &nets, width, via_d, TileId(tj.tile as u32));
        tally.raster_ms += t1.elapsed().as_secs_f64() * 1e3;
        for pass in 0..passes {
            let last = pass + 1 == passes;
            for &ci in &list {
                let c = &conns[ci];
                let src = terminals(b.board, &lat, slots, &c.from);
                let dst = terminals(b.board, &lat, slots, &c.to);
                if src.is_empty() || dst.is_empty() {
                    continue;
                }
                let mut pool = JobPool::default();
                let job = JobDescriptor {
                    connection: ci as u32,
                    net: c.net.0,
                    profile: pi as u16,
                    flags: JOB_ALLOW_VIA,
                    cost_bound: u32::MAX,
                    ..Default::default()
                };
                let ji = pool.push(job, &src, &dst, &[]);
                toggle_own(&mut cs, b, &lat, c.net, false);
                let ctx = SearchContext {
                    stack: &cs.stack,
                    lattice: Some(&lat),
                    config: &v.config,
                    checker: if v.checker { Some(check) } else { None },
                    lookahead: la.as_deref(),
                    gcells: None,
                };
                let mut stats = SearchStats::default();
                let ts = Instant::now();
                let cy0 = thread_cycles();
                let o = search_with(&ctx, &pool, &pool.jobs[0], ji, &v.params, &mut stats);
                let cycles = thread_cycles() - cy0;
                let us = ts.elapsed().as_micros() as u64;
                toggle_own(&mut cs, b, &lat, c.net, true);
                if !last {
                    continue;
                }
                stats.record(&o, &mut telemetry.lock().unwrap());
                tally.jobs += 1;
                tally.search_us += us;
                tally.times_us.push(us);
                tally.cycles += cycles;
                tally.cycle_list.push(cycles);
                tally.expansions += stats.expansions;
                tally.window_nodes += stats.window_nodes;
                tally.stats.add(&stats);
                match o {
                    SearchOutcome::Found(p) => {
                        tally.found += 1;
                        tally.found_max_exp = tally.found_max_exp.max(stats.expansions);
                        if std::env::var_os("KONNECT_SEARCH_VERBOSE").is_some()
                            && stats.expansions > 50_000
                        {
                            let a = lat.node(p.nodes[0].col.into(), p.nodes[0].row.into());
                            let z = p
                                .nodes
                                .last()
                                .map(|n| lat.node(n.col.into(), n.row.into()))
                                .unwrap_or(a);
                            let vias = p
                                .nodes
                                .windows(2)
                                .filter(|w| w[0].layer != w[1].layer)
                                .count();
                            println!(
                            "  slow: {} exp, {} us, cost {}, corners {}, vias {vias}, straight {:.2} mm, src {} dst {} nodes, window {}k, layers {}->{}",
                            stats.expansions, us, p.cost.0, p.nodes.len(), dist(a, z) / 1e6, src.len(), dst.len(),
                            stats.window_nodes / 1000, p.nodes[0].layer, p.nodes.last().map_or(0, |n| n.layer)
                        );
                        }
                        let pts: Vec<(Point, u8)> = p
                            .nodes
                            .iter()
                            .map(|n| (lat.node(n.col.into(), n.row.into()), n.layer))
                            .collect();
                        let mut len = 0.0;
                        let mut last_h: Option<u8> = None;
                        for w in pts.windows(2) {
                            if w[0].1 != w[1].1 {
                                tally.vias += 1;
                                last_h = None;
                                if !checker.via_ok(&pool.jobs[0], w[0].0) {
                                    tally.violations += 1;
                                }
                                continue;
                            }
                            len += dist(w[0].0, w[1].0);
                            let dx = i64::from(w[1].0.x) - i64::from(w[0].0.x);
                            let dy = i64::from(w[1].0.y) - i64::from(w[0].0.y);
                            assert!(
                                dx == 0 || dy == 0 || dx.abs() == dy.abs(),
                                "non-octilinear run"
                            );
                            let h = konnect_route::detail::search::heading_of(
                                dx.signum() as i32,
                                dy.signum() as i32,
                            )
                            .unwrap();
                            if let Some(lh) = last_h {
                                let d = (i32::from(h) - i32::from(lh)).rem_euclid(8);
                                if matches!(d, 3..=5) {
                                    tally.bad_turns += 1;
                                }
                            }
                            last_h = Some(h);
                            if !checker.segment_ok(&pool.jobs[0], w[0].1, w[0].0, w[1].0) {
                                tally.violations += 1;
                            }
                        }
                        let first = pts[0].0;
                        let lastp = pts.last().unwrap().0;
                        let straight = dist(first, lastp).max(1.0);
                        let (ax, ay) = (
                            (f64::from(lastp.x) - f64::from(first.x)).abs(),
                            (f64::from(lastp.y) - f64::from(first.y)).abs(),
                        );
                        let oct = ax.max(ay) + (2f64.sqrt() - 1.0) * ax.min(ay);
                        if straight > 200_000.0 {
                            tally.len_ratio.push(len / straight);
                            tally.octile_ratio.push(len / oct.max(1.0));
                        }
                        if single_conn_nets.contains(&c.net) {
                            if let Some(&rl) = reference_len.get(&c.net) {
                                if rl > 0.0 {
                                    tally.reference.push(len / rl);
                                }
                            }
                        }
                        let _ = (c.from_pad, c.to_pad);
                    }
                    ref o @ (SearchOutcome::Unreachable { .. }
                    | SearchOutcome::BudgetExhausted { .. }) => {
                        if matches!(o, SearchOutcome::Unreachable { .. }) {
                            tally.unreachable += 1;
                        } else {
                            tally.budget += 1;
                        }
                        tally.fail_cycles += cycles;
                        if std::env::var_os("KONNECT_SEARCH_VERBOSE").is_some() {
                            let blocked = |n: &JobNode| {
                                let r = &cs.stack.layers[n.layer as usize];
                                r.core.get(n.col.into(), n.row.into()) > 0
                            };
                            println!(
                                "  failed: {} after {} exp, {} us, straight {:.2} mm, src {} ({} blocked) dst {} ({} blocked), window {}k",
                                if matches!(o, SearchOutcome::Unreachable { .. }) { "unreachable" } else { "budget" },
                                stats.expansions,
                                us,
                                dist(c.from_pad, c.to_pad) / 1e6,
                                src.len(),
                                src.iter().filter(|n| blocked(n)).count(),
                                dst.len(),
                                dst.iter().filter(|n| blocked(n)).count(),
                                stats.window_nodes / 1000
                            );
                        }
                    }
                }
            }
        }
    }
    if v.cached {
        let cs = shared.stats();
        tally.cache_hits += cs.hits;
        tally.cache_misses += cs.misses;
    }
    tally
}

/// Pure kernel scaling: one tile's largest class stack, cloned per thread;
/// every thread routes the same job list (own net toggled per job).
fn kernel_scaling(b: &Board<'_>, tj: &TileJob, conns: &[Conn], slots: &[LayerId], v: &Variant) {
    let base = konnect_route::api::LATTICE_PITCH_NM;
    let cols = (tj.window.width() / i64::from(base)) as u32 + 1;
    let rows = (tj.window.height() / i64::from(base)) as u32 + 1;
    let lat = Lattice::uniform(
        konnect_route::compute::Window::new(tj.window.min, base, cols, rows).unwrap(),
    );
    let mut by_class: BTreeMap<ClassId, Vec<usize>> = BTreeMap::new();
    for &ci in &tj.conns {
        by_class.entry(conns[ci].class).or_default().push(ci);
    }
    let Some((_, list)) = by_class
        .into_iter()
        .max_by_key(|(c, l)| (l.len(), std::cmp::Reverse(*c)))
    else {
        return;
    };
    let rep = conns[list[0]].net;
    let width = b
        .rules
        .track_width(&RuleSubject::new(ItemKind::Track, Some(rep)), slots[0])
        .preferred;
    let via = b.rules.via(&RuleSubject::new(ItemKind::Via, Some(rep)));
    let nets: Vec<NetId> = list
        .iter()
        .map(|&c| conns[c].net)
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect();
    let cs = build_stack(b, &lat, slots, &nets, width, via.diameter, TileId(0));
    let jobs: Vec<(NetId, Vec<JobNode>, Vec<JobNode>)> = list
        .iter()
        .map(|&ci| {
            (
                conns[ci].net,
                terminals(b.board, &lat, slots, &conns[ci].from),
                terminals(b.board, &lat, slots, &conns[ci].to),
            )
        })
        .filter(|j| !j.1.is_empty() && !j.2.is_empty())
        .collect();
    let profiles = vec![ProfileInfo {
        width,
        via_diameter: via.diameter,
        via_drill: via.drill,
    }];
    let max_t = rayon::current_num_threads();
    let mut base_rate = 0.0;
    for t in [1usize, 2, 4, 8, 12, 16, 20]
        .into_iter()
        .filter(|&t| t <= max_t.max(1))
    {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(t)
            .build()
            .unwrap();
        let ts = Instant::now();
        let per: Vec<(u64, u64, u64)> = pool.install(|| {
            (0..t)
                .into_par_iter()
                .map(|_| {
                    let mut cs = cs.clone();
                    let checker = Checker {
                        b,
                        layers: slots.to_vec(),
                        profiles: profiles.clone(),
                        memo: Mutex::new(HashMap::new()),
                    };
                    let mut us = 0u64;
                    let mut exp = 0u64;
                    let mut cyc = 0u64;
                    // Pass 0 warms this thread's scratch; pass 1 is timed.
                    for pass in 0..2 {
                        for (net, src, dst) in &jobs {
                            let mut pool = JobPool::default();
                            let job = JobDescriptor {
                                net: net.0,
                                flags: JOB_ALLOW_VIA,
                                cost_bound: u32::MAX,
                                ..Default::default()
                            };
                            pool.push(job, src, dst, &[]);
                            toggle_own(&mut cs, b, &lat, *net, false);
                            let ctx = SearchContext {
                                stack: &cs.stack,
                                lattice: Some(&lat),
                                config: &v.config,
                                checker: if v.checker { Some(&checker) } else { None },
                                lookahead: None,
                                gcells: None,
                            };
                            let mut stats = SearchStats::default();
                            let t0 = Instant::now();
                            let c0 = thread_cycles();
                            let _ =
                                search_with(&ctx, &pool, &pool.jobs[0], 0, &v.params, &mut stats);
                            if pass == 1 {
                                cyc += thread_cycles() - c0;
                                us += t0.elapsed().as_micros() as u64;
                                exp += stats.expansions;
                            }
                            toggle_own(&mut cs, b, &lat, *net, true);
                        }
                    }
                    (us, exp, cyc)
                })
                .collect()
        });
        let wall = ts.elapsed().as_secs_f64();
        let search_s: f64 = per.iter().map(|p| p.0 as f64 / 1e6).sum();
        let exp: u64 = per.iter().map(|p| p.1).sum();
        let cyc: u64 = per.iter().map(|p| p.2).sum();
        // Searches per second of summed search time, scaled by threads.
        let rate = (t * jobs.len()) as f64 / (search_s / t as f64);
        if t == 1 {
            base_rate = rate;
        }
        println!(
            "[{} scaling] threads {t:2}: {} searches, mean {:.3} ms/search, {:.2} M exp/s per thread, {:.0} searches/s aggregate (x{:.1}), {:.0} cycles/expansion, wall {:.2} s incl. own-net toggles",
            v.name,
            t * jobs.len(),
            search_s * 1e3 / (t * jobs.len()) as f64,
            exp as f64 / search_s / 1e6,
            rate,
            rate / base_rate.max(1e-9),
            cyc as f64 / exp.max(1) as f64,
            wall
        );
    }
}

fn report(name: &str, t: &Tally, wall_ms: f64, threads: usize) {
    let mut times: Vec<f64> = t.times_us.iter().map(|&u| u as f64 / 1e3).collect();
    let mut lr = t.len_ratio.clone();
    let mut orr = t.octile_ratio.clone();
    let mut rr = t.reference.clone();
    let secs = t.search_us as f64 / 1e6;
    println!(
        "[{name}] jobs {} found {} unreachable {} budget {} | search {:.2} s, {:.3} ms/conn mean, p50 {:.3} p90 {:.3} p99 {:.3} max {:.3} ms | {:.2} M exp/s, {:.0} exp/conn | wall {:.0} ms on {threads} threads",
        t.jobs, t.found, t.unreachable, t.budget, secs,
        if t.jobs > 0 { t.search_us as f64 / 1e3 / t.jobs as f64 } else { 0.0 },
        pct(&mut times, 0.5), pct(&mut times, 0.9), pct(&mut times, 0.99), pct(&mut times, 1.0),
        t.expansions as f64 / secs.max(1e-9) / 1e6,
        t.expansions as f64 / t.jobs.max(1) as f64,
        wall_ms,
    );
    println!(
        "[{name}]   length/straight p50 {:.3} p90 {:.3} mean {:.3} | length/octile p50 {:.3} p90 {:.3} | vias {} | exact re-check violations {} | 135-degree turns {} | exact checks {} (+{} cached), via checks {}, retries {} | raster build {:.0} ms | lattice nodes/tile {:.0}k, window nodes/search {:.0}k, lazy rejects {}",
        pct(&mut lr, 0.5), pct(&mut lr, 0.9), lr.iter().sum::<f64>() / lr.len().max(1) as f64,
        pct(&mut orr, 0.5), pct(&mut orr, 0.9),
        t.vias, t.violations, t.bad_turns,
        t.stats.exact_checks, t.stats.exact_check_cache_hits, t.stats.via_checks, t.stats.region_retries,
        t.raster_ms,
        t.lattice_nodes as f64 / t.tiles.max(1) as f64 / 1e3, t.window_nodes as f64 / t.jobs.max(1) as f64 / 1e3,
        t.stats.lazy_rejects,
    );
    if t.cycles > 0 {
        let mut cyc: Vec<f64> = t.cycle_list.iter().map(|&c| c as f64 / 1e6).collect();
        println!(
            "[{name}]   CPU cycles: {:.3} Mcycles/conn mean, p50 {:.3} p90 {:.3} p99 {:.3} | {:.0} cycles/expansion | at 4.7 GHz: {:.3} ms/conn, {:.2} M exp/s | failed searches {:.0}% of cycles | most expansions in a found search {}",
            t.cycles as f64 / 1e6 / t.jobs.max(1) as f64,
            pct(&mut cyc, 0.5),
            pct(&mut cyc, 0.9),
            pct(&mut cyc, 0.99),
            t.cycles as f64 / t.expansions.max(1) as f64,
            t.cycles as f64 / 4.7e9 * 1e3 / t.jobs.max(1) as f64,
            t.expansions as f64 / (t.cycles as f64 / 4.7e9).max(1e-12) / 1e6,
            100.0 * t.fail_cycles as f64 / t.cycles.max(1) as f64,
            t.found_max_exp,
        );
    }
    if t.cache_hits + t.cache_misses > 0 {
        println!(
            "[{name}]   shared check cache over both passes: {} hits, {} misses ({:.0}% hits)",
            t.cache_hits,
            t.cache_misses,
            100.0 * t.cache_hits as f64 / (t.cache_hits + t.cache_misses) as f64
        );
    }
    if !rr.is_empty() {
        println!(
            "[{name}]   vs reference routing (two-pin nets it completed, n = {}): length ratio p10 {:.3} p50 {:.3} p90 {:.3}",
            rr.len(), pct(&mut rr, 0.1), pct(&mut rr, 0.5), pct(&mut rr, 0.9)
        );
    }
}

#[test]
fn search_kernel_on_real_tiles() {
    let Some(dir) = std::env::var_os("KONNECT_DB1_FIXTURE_DIR").map(PathBuf::from) else {
        eprintln!("skipping: KONNECT_DB1_FIXTURE_DIR is not set");
        return;
    };
    let (Some(pcb), Some(pro)) = (only_file(&dir, "kicad_pcb"), only_file(&dir, "kicad_pro"))
    else {
        eprintln!(
            "skipping: {} lacks a single .kicad_pcb/.kicad_pro",
            dir.display()
        );
        return;
    };
    let t0 = Instant::now();
    let (mut board, _) = load_board_file(&pcb, &LoadOptions::default()).expect("board loads");
    let pro_text = std::fs::read_to_string(&pro).unwrap();
    let dru = only_file(&dir, "kicad_dru").map(|p| std::fs::read_to_string(p).unwrap());
    let rules =
        ProjectRuleTable::from_project(&pro_text, dru.as_deref(), &board).expect("rules compile");
    rules.assign_classes(&mut board);
    let conn = ConnectivityReport::build(&board, &ArcPolicy::default());
    let connections = conn.connections(&board);
    println!(
        "load + rules + connectivity: {} ms, {} connections",
        t0.elapsed().as_millis(),
        connections.len()
    );

    let t1 = Instant::now();
    let b = Board::new(&board, &rules);
    println!(
        "obstacles: {} in {} ms",
        b.obs.len(),
        t1.elapsed().as_millis()
    );

    // Islands by pad.
    let mut island_of: HashMap<u32, (usize, usize)> = HashMap::new();
    for (ni, n) in conn.nets.iter().enumerate() {
        for (ii, isl) in n.islands.iter().enumerate() {
            for p in isl.pads() {
                island_of.insert(p.0, (ni, ii));
            }
        }
    }
    let plane_nets: BTreeSet<NetId> = board.zones.iter().filter_map(|z| z.net).collect();
    let mut per_net: BTreeMap<NetId, usize> = BTreeMap::new();
    for c in &connections {
        *per_net.entry(c.net).or_default() += 1;
    }
    let single_conn_nets: BTreeSet<NetId> = per_net
        .iter()
        .filter(|(_, &n)| n == 1)
        .map(|(k, _)| *k)
        .collect();
    let slots: Vec<LayerId> = board.routable.iter().collect();
    assert!(!slots.is_empty());

    let mut conns = Vec::new();
    for c in &connections {
        if plane_nets.contains(&c.net) {
            continue;
        }
        let (ItemRef::Pad(fp), ItemRef::Pad(tp)) = (c.from, c.to) else {
            continue;
        };
        let (Some(&(fn_, fi)), Some(&(tn, ti))) = (island_of.get(&fp.0), island_of.get(&tp.0))
        else {
            continue;
        };
        let from = conn.nets[fn_].islands[fi].items.clone();
        let to = conn.nets[tn].islands[ti].items.clone();
        let bbox = from
            .iter()
            .chain(&to)
            .filter_map(|&i| item_bbox(&board, i))
            .reduce(BBox::union);
        let Some(bbox) = bbox else { continue };
        // Terminals must reach a routable layer.
        let on_routable = |items: &[ItemRef]| {
            items.iter().any(|&i| match i {
                ItemRef::Pad(p) => board.pads[p.index()].layers.intersects(board.routable),
                ItemRef::Track(t) => board.routable.contains(board.tracks[t.index()].layer),
                ItemRef::Via(_) => true,
                _ => false,
            })
        };
        if !on_routable(&from) || !on_routable(&to) {
            continue;
        }
        conns.push(Conn {
            net: c.net,
            class: board.nets[c.net.index()].class,
            from_pad: board.pads[fp.index()].center,
            to_pad: board.pads[tp.index()].center,
            from,
            to,
            bbox,
        });
    }

    // Tile grid.
    let bb = board.outline.bbox.expect("outline bbox");
    let (tw, th) = TILE_CORE_NM;
    let tcols = (bb.width() / i64::from(tw) + 1) as usize;
    let trows = (bb.height() / i64::from(th) + 1) as usize;
    let mut tiles: Vec<TileJob> = Vec::new();
    for r in 0..trows {
        for c in 0..tcols {
            let min = Point::new(bb.min.x + tw * c as i32, bb.min.y + th * r as i32);
            let core = BBox::new(min, Point::new(min.x + tw, min.y + th));
            let window = BBox::new(
                Point::new(
                    (core.min.x - TILE_HALO_NM).max(bb.min.x),
                    (core.min.y - TILE_HALO_NM).max(bb.min.y),
                ),
                Point::new(
                    (core.max.x + TILE_HALO_NM).min(bb.max.x),
                    (core.max.y + TILE_HALO_NM).min(bb.max.y),
                ),
            );
            tiles.push(TileJob {
                tile: tiles.len(),
                window,
                conns: Vec::new(),
            });
        }
    }
    let mut cross = 0;
    for (ci, c) in conns.iter().enumerate() {
        let mid = c.bbox.center();
        let t = tiles.iter_mut().find(|t| {
            let core = t.window.inflate(-TILE_HALO_NM);
            core.contains(mid) && t.window.contains_bbox(&c.bbox.inflate(300_000))
        });
        match t {
            Some(t) => t.conns.push(ci),
            None => cross += 1,
        }
    }
    // Fine-pitch fields: footprints with many pads at a pitch of 1 mm or less.
    let mut fine = Vec::new();
    for f in &board.footprints {
        let pads = &board.pads[f.pad_range.0 as usize..f.pad_range.1 as usize];
        if pads.len() < 64 {
            continue;
        }
        let mut min_d = i64::MAX;
        for (i, a) in pads.iter().enumerate().take(64) {
            for bpad in pads.iter().skip(i + 1) {
                let d = (a.center - bpad.center).length2();
                if d > 0 {
                    min_d = min_d.min(d as i64);
                }
            }
        }
        if min_d <= 1_000_000i64 * 1_000_000 {
            if let Some(bx) = pads.iter().filter_map(|p| p.bbox()).reduce(BBox::union) {
                fine.push(bx.inflate(500_000));
            }
        }
    }
    let want = std::env::var("KONNECT_SEARCH_TILES").unwrap_or_else(|_| "3".into());
    let mut order: Vec<usize> = (0..tiles.len())
        .filter(|&i| !tiles[i].conns.is_empty())
        .collect();
    order.sort_by_key(|&i| (std::cmp::Reverse(tiles[i].conns.len()), i));
    let mut chosen: Vec<usize> = if want == "all" {
        order.clone()
    } else {
        order
            .iter()
            .take(want.parse().unwrap_or(3))
            .copied()
            .collect()
    };
    // Always include the busiest tile touching a fine-pitch field.
    if let Some(&ft) = order
        .iter()
        .find(|&&i| fine.iter().any(|f| f.intersects(&tiles[i].window)))
    {
        if !chosen.contains(&ft) {
            chosen.push(ft);
        }
    }
    chosen.sort_unstable();
    println!(
        "tiles {} ({} with jobs), single-tile connections {}, cross-tile {}, fine fields {}, chosen tiles {:?} with {} connections",
        tiles.len(), order.len(), conns.len() - cross, cross, fine.len(), chosen,
        chosen.iter().map(|&i| tiles[i].conns.len()).sum::<usize>()
    );

    // Lengths of a reference routing of the same board (for example an
    // earlier autorouter run imported into a copy), for two-pin nets it
    // completed, when `KONNECT_SEARCH_REFERENCE_DIR` holds one.
    let mut reference_len: BTreeMap<NetId, f64> = BTreeMap::new();
    let reference_dir = std::env::var_os("KONNECT_SEARCH_REFERENCE_DIR").map(PathBuf::from);
    if let Some(cand) = reference_dir
        .as_deref()
        .and_then(|d| only_file(d, "kicad_pcb"))
    {
        if let Ok((cb, _)) = load_board_file(&cand, &LoadOptions::default()) {
            let cc = ConnectivityReport::build(&cb, &ArcPolicy::default());
            let mut base_len: BTreeMap<String, f64> = BTreeMap::new();
            for t in &board.tracks {
                if let Some(n) = t.net {
                    *base_len
                        .entry(board.nets[n.index()].name.clone())
                        .or_default() += t.seg.length() as f64;
                }
            }
            for (ni, n) in cb.nets.iter().enumerate() {
                let complete = cc.nets.get(ni).is_some_and(|x| x.pad_island_count() == 1);
                let Some(our) = board.net_by_name(&n.name) else {
                    continue;
                };
                if !complete || !single_conn_nets.contains(&our) {
                    continue;
                }
                let total: f64 = cb
                    .tracks
                    .iter()
                    .filter(|t| t.net == Some(n.id))
                    .map(|t| t.seg.length() as f64)
                    .sum();
                let new = total - base_len.get(&n.name).copied().unwrap_or(0.0);
                if new > 0.0 {
                    reference_len.insert(our, new);
                }
            }
            println!(
                "reference routing: {} two-pin nets complete",
                reference_len.len()
            );
        }
    }

    let params = CostParams::default();
    let prefs: Vec<Preferred> = slots
        .iter()
        .enumerate()
        .map(|(i, _)| {
            if i % 2 == 0 {
                Preferred::Horizontal
            } else {
                Preferred::Vertical
            }
        })
        .collect();
    let base_cfg = SearchConfig {
        min_track_angle_deg: rules
            .min_track_angle_deg(&RuleSubject::new(ItemKind::Track, None), slots[0]),
        ..SearchConfig::default()
    };
    let pref_cfg = SearchConfig {
        layer_pref: prefs,
        ..base_cfg.clone()
    };
    let mut variants = vec![
        Variant {
            cached: false,
            name: "uniform50/octile/exact",
            non_uniform: false,
            config: base_cfg.clone(),
            params,
            checker: true,
        },
        Variant {
            cached: false,
            name: "uniform50/octile/no-check",
            non_uniform: false,
            config: base_cfg.clone(),
            params,
            checker: false,
        },
        Variant {
            cached: false,
            name: "uniform50/prefs/octile",
            non_uniform: false,
            config: pref_cfg.clone(),
            params,
            checker: true,
        },
        Variant {
            cached: false,
            name: "uniform50/prefs/map",
            non_uniform: false,
            config: SearchConfig {
                heuristic: Heuristic::Map,
                ..pref_cfg.clone()
            },
            params,
            checker: true,
        },
        Variant {
            cached: false,
            name: "padaligned/octile/exact",
            non_uniform: true,
            config: base_cfg.clone(),
            params,
            checker: true,
        },
    ];
    variants.push(Variant {
        cached: true,
        name: "uniform50/octile/exact+shared-cache, 2nd pass",
        non_uniform: false,
        config: base_cfg.clone(),
        params,
        checker: true,
    });
    if std::env::var("KONNECT_SEARCH_VARIANTS").as_deref() == Ok("weights") {
        variants.retain(|v| v.name == "uniform50/octile/exact");
        for (name, f) in [
            ("uniform50/octile/weighted1.02", 1020),
            ("uniform50/octile/weighted1.05", 1050),
            ("uniform50/octile/weighted1.1", 1100),
        ] {
            variants.push(Variant {
                cached: false,
                name,
                non_uniform: false,
                config: SearchConfig {
                    astar_factor: f,
                    ..base_cfg.clone()
                },
                params,
                checker: true,
            });
        }
    }
    if std::env::var("KONNECT_SEARCH_VARIANTS").as_deref() == Ok("all") {
        variants.push(Variant {
            cached: false,
            name: "uniform50/octile/weighted1.2",
            non_uniform: false,
            config: SearchConfig {
                astar_factor: 1200,
                ..base_cfg.clone()
            },
            params,
            checker: true,
        });
        variants.push(Variant {
            cached: false,
            name: "padaligned/prefs/map",
            non_uniform: true,
            config: SearchConfig {
                heuristic: Heuristic::Map,
                ..pref_cfg.clone()
            },
            params,
            checker: true,
        });
    }
    if let Ok(only) = std::env::var("KONNECT_SEARCH_ONLY") {
        variants.retain(|v| only.split(',').any(|o| v.name.contains(o)));
    }
    let repeat: usize = std::env::var("KONNECT_SEARCH_REPEAT")
        .ok()
        .and_then(|r| r.parse().ok())
        .unwrap_or(1);
    let variants: Vec<Variant> = variants
        .iter()
        .flat_map(|v| std::iter::repeat_n(v.clone(), repeat))
        .collect();
    let lookahead = Mutex::new(BTreeMap::new());
    let threads = rayon::current_num_threads();
    for v in &variants {
        let telemetry = Mutex::new(Telemetry {
            threads: threads as u32,
            ..Telemetry::default()
        });
        // Serial over tiles (per-search timings without contention).
        let ts = Instant::now();
        let mut total = Tally::default();
        for &ti in &chosen {
            let t = route_tile(
                &b,
                &tiles[ti],
                &conns,
                &slots,
                v,
                &fine,
                &lookahead,
                &reference_len,
                &single_conn_nets,
                &telemetry,
            );
            total.merge(&t);
        }
        let wall = ts.elapsed().as_secs_f64() * 1e3;
        let mut t = telemetry.into_inner().unwrap();
        t.add_stage_time(Stage::Detail, total.search_us);
        report(v.name, &total, wall, 1);
        println!(
            "[{}]   telemetry: expansions/conn mean {:.0}, max {}, window nodes mean {:.0}k, found {}",
            v.name,
            t.expansions_per_connection.mean(),
            t.expansions_per_connection.max,
            t.window_nodes.mean() / 1e3,
            t.counters.get("search_found").copied().unwrap_or(0)
        );
        if v.checker {
            assert_eq!(
                total.violations, 0,
                "{}: paths failed the exact re-check",
                v.name
            );
        }
        assert_eq!(total.bad_turns, 0, "{}: 135-degree turn", v.name);
    }
    if variants.len() < 2 {
        return;
    }
    if let Some(&busiest) = order.first() {
        kernel_scaling(&b, &tiles[busiest], &conns, &slots, &variants[1]);
        kernel_scaling(&b, &tiles[busiest], &conns, &slots, &variants[0]);
    }
    // Tiles in parallel, the way negotiation runs same-colour tiles, with
    // and without the exact checker.
    for v in [&variants[0], &variants[1]] {
        let telemetry = Mutex::new(Telemetry::default());
        let ts = Instant::now();
        let tallies: Vec<Tally> = chosen
            .par_iter()
            .map(|&ti| {
                route_tile(
                    &b,
                    &tiles[ti],
                    &conns,
                    &slots,
                    v,
                    &fine,
                    &lookahead,
                    &reference_len,
                    &single_conn_nets,
                    &telemetry,
                )
            })
            .collect();
        let wall = ts.elapsed().as_secs_f64() * 1e3;
        let mut total = Tally::default();
        for t in &tallies {
            total.merge(t);
        }
        report(
            &format!("{} parallel tiles", v.name),
            &total,
            wall,
            threads.min(chosen.len()),
        );
    }
}
