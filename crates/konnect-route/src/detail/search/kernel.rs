//! The A* kernel: one generic loop over any [`Grid`], monomorphized for the
//! uniform window and the non-uniform lattice.
//!
//! # State layout
//!
//! State is `(node, layer, heading, run)`, densely indexed inside the
//! search region **heading-major**: `((layer · 9 + heading) · R + run) ·
//! nodes + node`, where heading 8 is "none" (a terminal or the landing of a
//! via) and `R` is the number of run buckets (1 unless the job needs a
//! minimum run between turns). Consecutive expansions along a frontier
//! relax the same headings of neighbouring nodes, which this layout puts
//! next to each other in memory.
//!
//! Each state is one 8-byte [`StateRec`]: `g`, a 16-bit generation stamp
//! and a 16-bit parent *move* (the parent's heading, run and layer). The
//! parent node is implied: a track state's predecessor is the reverse step
//! of its heading (lattice diagonals are symmetric), and a via landing's is
//! the same node. Records are generation-stamped, so a search never clears
//! them, and every buffer lives in a thread-local [`Scratch`] that only
//! grows: after warm-up a search does not allocate except for its result.

use super::heap::RadixHeap;
use super::lattice::{Grid, DIAGONAL, STRAIGHT, VIA_CHECK, VIA_NO};
use super::lookahead::MapLookahead;
use super::{turn_set, ExactCheck, Heuristic, Preferred, SearchConfig, SearchStats, HEADING_NONE};
use crate::compute::job::{JOB_ALLOW_VIA, JOB_MIN_RUN};
use crate::compute::raster::TileStack;
use crate::compute::{Cost, CostParams, JobDescriptor, JobNode, PathProposal, SearchOutcome};
use crate::global::GCellGrid;
use konnect_geom::{Coord, Point};

/// Headings plus "none".
const HS: u32 = 9;
const SRC: u32 = 1;
const DST: u32 = 2;
const MAX_LAYERS: usize = 32;
/// Target boxes the heuristic keeps; more targets merge into these.
const MAX_TARGET_BOXES: usize = 16;
/// `info` parent-heading value of a seed.
const SEED: u16 = 15;

#[derive(Debug, Clone, Copy)]
struct TBox {
    layer: u8,
    min_x: i64,
    max_x: i64,
    min_y: i64,
    max_y: i64,
}

/// One search state.
///
/// `info`: bit 0 closed; bits 1–4 parent heading (0–8, [`SEED`] for none);
/// bits 5–9 parent run bucket (0..=[`MAX_RUN_BUCKET`]); bits 10–14 parent
/// layer.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
struct StateRec {
    g: u32,
    gen: u16,
    info: u16,
}

/// `v` as a `u32`, saturating at `u32::MAX` ([`Cost::INFINITE`]).
#[inline]
fn sat32(v: u64) -> u32 {
    v.min(u64::from(u32::MAX)) as u32
}

#[inline]
fn info(ph: u32, prun: u32, pl: u32) -> u16 {
    ((ph << 1) | (prun << 5) | (pl << 10)) as u16
}

/// The largest run bucket `info` can hold: a minimum run of up to 31 half
/// base pitches (775 µm at 50 µm) is enforced exactly; a longer one makes
/// the job unreachable rather than under-enforced.
pub(crate) const MAX_RUN_BUCKET: u32 = 31;

/// The relaxed field's per node-layer records, generation-stamped so a run
/// never clears them: `d` is valid where `tag >> 1` is the run's
/// generation, and `tag & 1` marks it closed.
#[derive(Debug, Default)]
struct FieldMap {
    recs: Vec<(u32, u32)>,
    gen: u32,
}

impl FieldMap {
    /// Start a run over `len` node-layers.
    fn begin(&mut self, len: usize) {
        if self.recs.len() < len {
            self.recs.resize(len, (u32::MAX, 0));
        }
        self.gen += 1;
        if self.gen >= 1 << 30 {
            self.recs.iter_mut().for_each(|r| r.1 = 0);
            self.gen = 1;
        }
    }

    /// The cost so far (`u32::MAX`: not reached).
    #[inline]
    fn get(&self, i: usize) -> u32 {
        let r = self.recs[i];
        if r.1 >> 1 == self.gen {
            r.0
        } else {
            u32::MAX
        }
    }

    #[inline]
    fn closed(&self, i: usize) -> bool {
        self.recs[i].1 == self.gen << 1 | 1
    }

    #[inline]
    fn set(&mut self, i: usize, d: u32) {
        self.recs[i] = (d, self.gen << 1);
    }

    #[inline]
    fn close(&mut self, i: usize) {
        self.recs[i].1 = self.gen << 1 | 1;
    }
}

/// Per node-layer terminal flags and the exact-check cache.
/// `tag` is `gen << 2 | flags`.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
struct NlRec {
    tag: u32,
    src_mask: u8,
    dst_mask: u8,
    edge_seen: u8,
    edge_ok: u8,
}

/// Reusable per-thread search memory.
#[derive(Debug, Default)]
pub(crate) struct Scratch {
    /// Search generation; states use its low 16 bits.
    gen: u32,
    st: Vec<StateRec>,
    heap: RadixHeap,
    nl: Vec<NlRec>,
    via_stamp: Vec<u32>,
    via_state: Vec<u8>,
    /// Per node: the via cost this search charges there (`u32::MAX`: no
    /// via), valid where `vcost_stamp` equals the generation. A via's cost
    /// depends only on its node, so it is computed once per node, not once
    /// per expanded state.
    vcost_stamp: Vec<u32>,
    vcost: Vec<u32>,
    corr_stamp: Vec<u32>,
    gcol: Vec<u32>,
    grow: Vec<u32>,
    chain: Vec<u32>,
    tboxes: Vec<TBox>,
    /// Reverse-flood visit stamps (per node-layer) and its queue.
    flood_stamp: Vec<u32>,
    flood_queue: Vec<u32>,
    /// The relaxed cost-to-go field per node-layer ([`relaxed_field`]) and
    /// the heap that builds it.
    field: FieldMap,
    fheap: RadixHeap,
    /// The field's pass-through bounds at the target node-layers.
    fpass: Vec<PassBound>,
    /// Per region column, then row: the gap to the sources' box, nm.
    gaps: Vec<i64>,
    /// The last [`run`] stopped at its [`Problem::cap`].
    pub capped: bool,
}

impl Scratch {
    /// Start a new search generation, growing buffers to the given sizes.
    fn begin(&mut self, states: usize, node_layers: usize, nodes: usize) {
        if self.gen >= (1 << 29) {
            self.nl.iter_mut().for_each(|r| r.tag = 0);
            self.via_stamp.iter_mut().for_each(|s| *s = 0);
            self.vcost_stamp.iter_mut().for_each(|s| *s = 0);
            self.corr_stamp.iter_mut().for_each(|s| *s = 0);
            self.st.iter_mut().for_each(|r| r.gen = 0);
            self.flood_stamp.iter_mut().for_each(|s| *s = 0);
            self.gen = 0;
        }
        self.gen += 1;
        if self.gen & 0xffff == 0 {
            // The 16-bit state stamps wrapped: forget every old stamp.
            self.st.iter_mut().for_each(|r| r.gen = 0);
            self.gen += 1;
        }
        if self.st.len() < states {
            self.st.resize(states, StateRec::default());
        }
        if self.nl.len() < node_layers {
            self.nl.resize(node_layers, NlRec::default());
            self.flood_stamp.resize(node_layers, 0);
        }
        if self.via_stamp.len() < nodes {
            self.via_stamp.resize(nodes, 0);
            self.via_state.resize(nodes, 0);
            self.vcost_stamp.resize(nodes, 0);
            self.vcost.resize(nodes, 0);
        }
        self.heap.clear();
        self.tboxes.clear();
        self.chain.clear();
    }

    #[inline]
    fn sgen(&self) -> u16 {
        (self.gen & 0xffff) as u16
    }

    /// Bytes of buffer capacity held, for allocation tests and telemetry.
    pub fn footprint(&self) -> usize {
        8 * self.st.capacity()
            + 8 * self.nl.capacity()
            + 5 * self.via_stamp.capacity()
            + 8 * self.vcost_stamp.capacity()
            + 4 * (self.flood_stamp.capacity()
                + 2 * self.field.recs.capacity()
                + self.flood_queue.capacity()
                + self.corr_stamp.capacity()
                + self.gcol.capacity()
                + self.grow.capacity()
                + self.chain.capacity())
    }

    #[inline]
    fn touch(&mut self, nl: usize) -> &mut NlRec {
        let gen = self.gen;
        let r = &mut self.nl[nl];
        if r.tag >> 2 != gen {
            *r = NlRec {
                tag: gen << 2,
                ..NlRec::default()
            };
        }
        r
    }

    #[inline]
    fn flags_of(&self, nl: usize) -> u32 {
        let r = self.nl[nl];
        if r.tag >> 2 == self.gen {
            r.tag & 3
        } else {
            0
        }
    }

    #[inline]
    fn nl_of(&self, nl: usize) -> NlRec {
        let r = self.nl[nl];
        if r.tag >> 2 == self.gen {
            r
        } else {
            NlRec::default()
        }
    }
}

/// One layer's planes as flat slices.
#[derive(Clone, Copy)]
struct View<'a> {
    core: &'a [u8],
    halo: &'a [u8],
    hmask: &'a [u8],
    via: &'a [u8],
    present: &'a [u16],
    history: &'a [u16],
    s8: usize,
    s16: usize,
}

const EMPTY_VIEW: View<'static> = View {
    core: &[],
    halo: &[],
    hmask: &[],
    via: &[],
    present: &[],
    history: &[],
    s8: 0,
    s16: 0,
};

/// The search rectangle in lattice indices, inclusive-exclusive.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Region {
    pub c0: u32,
    pub r0: u32,
    pub w: u32,
    pub h: u32,
}

impl Region {
    pub fn full<G: Grid>(grid: &G) -> Self {
        Region {
            c0: 0,
            r0: 0,
            w: grid.cols(),
            h: grid.rows(),
        }
    }

    pub fn nodes(&self) -> usize {
        self.w as usize * self.h as usize
    }
}

/// What to compute.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Mode {
    Route,
    /// Exhaust the region and leave `g` behind for [`field_costs`].
    Field,
}

/// Everything one search reads.
pub(crate) struct Problem<'a> {
    pub stack: &'a TileStack,
    pub job: &'a JobDescriptor,
    pub job_index: u32,
    pub sources: &'a [JobNode],
    pub targets: &'a [JobNode],
    pub corridor: &'a [u32],
    pub params: &'a CostParams,
    pub config: &'a SearchConfig,
    pub checker: Option<&'a dyn ExactCheck>,
    pub lookahead: Option<&'a MapLookahead>,
    pub gcells: Option<&'a GCellGrid>,
    pub mode: Mode,
    /// Expansions an earlier attempt of the same search already used; the
    /// budget is `config.max_expansions − spent`.
    pub spent: u64,
    /// Stop after this many expansions of this attempt and set
    /// [`Scratch::capped`] (`u64::MAX`: no cap).
    pub cap: u64,
    /// Add the relaxed cost-to-go field ([`relaxed_field`]) to the heuristic.
    pub use_field: bool,
}

/// Lattice indices of a GCell id's footprint, clipped to the grid.
fn gcell_span<G: Grid>(grid: &G, gc: &GCellGrid, id: u32) -> Option<(u32, u32, u32, u32)> {
    if gc.cols == 0 || gc.rows == 0 {
        return None;
    }
    let col = id % gc.cols;
    let row = (id / gc.cols) % gc.rows;
    let x0 = i64::from(gc.bbox.min.x) + i64::from(gc.pitch) * i64::from(col);
    let y0 = i64::from(gc.bbox.min.y) + i64::from(gc.pitch) * i64::from(row);
    let x1 = x0 + i64::from(gc.pitch);
    let y1 = y0 + i64::from(gc.pitch);
    let cl = |v: i64| v.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as Coord;
    Some((
        grid.col_at_or_before(cl(x0)),
        grid.col_at_or_after(cl(x1)),
        grid.row_at_or_before(cl(y0)),
        grid.row_at_or_after(cl(y1)),
    ))
}

/// The job's bounded region: terminals plus corridor, inflated by the
/// configured margin, clipped to the grid.
pub(crate) fn job_region<G: Grid>(grid: &G, p: &Problem<'_>) -> Region {
    let mut c_lo = u32::MAX;
    let mut c_hi = 0;
    let mut r_lo = u32::MAX;
    let mut r_hi = 0;
    for n in p.sources.iter().chain(p.targets) {
        c_lo = c_lo.min(u32::from(n.col));
        c_hi = c_hi.max(u32::from(n.col));
        r_lo = r_lo.min(u32::from(n.row));
        r_hi = r_hi.max(u32::from(n.row));
    }
    if let Some(gc) = p.gcells {
        for &id in p.corridor {
            if let Some((a, b, c, d)) = gcell_span(grid, gc, id) {
                c_lo = c_lo.min(a);
                c_hi = c_hi.max(b);
                r_lo = r_lo.min(c);
                r_hi = r_hi.max(d);
            }
        }
    }
    if c_lo == u32::MAX || c_lo >= grid.cols() || r_lo >= grid.rows() {
        return Region::full(grid);
    }
    let c_hi = c_hi.min(grid.cols() - 1);
    let r_hi = r_hi.min(grid.rows() - 1);
    let span = (i64::from(grid.x(c_hi)) - i64::from(grid.x(c_lo)))
        .max(i64::from(grid.y(r_hi)) - i64::from(grid.y(r_lo)));
    let margin = i64::from(p.config.region_margin_nm)
        + span * i64::from(p.config.region_margin_permille) / 1000;
    let cl = |v: i64| v.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as Coord;
    let c0 = grid.col_at_or_before(cl(i64::from(grid.x(c_lo)) - margin));
    let c1 = grid.col_at_or_after(cl(i64::from(grid.x(c_hi)) + margin));
    let r0 = grid.row_at_or_before(cl(i64::from(grid.y(r_lo)) - margin));
    let r1 = grid.row_at_or_after(cl(i64::from(grid.y(r_hi)) + margin));
    Region {
        c0,
        r0,
        w: c1 - c0 + 1,
        h: r1 - r0 + 1,
    }
}

/// Octile distance in cost units for nm offsets, rounded down.
#[inline]
fn octile(dx: i64, dy: i64, inv_pitch: u64) -> u32 {
    let (a, b) = if dx > dy { (dx, dy) } else { (dy, dx) };
    let v = (1000 * a as u64 + 414 * b as u64).saturating_mul(inv_pitch) >> 32;
    v.min(u64::from(u32::MAX >> 2)) as u32
}

#[inline]
fn axis_gap(v: i64, lo: i64, hi: i64) -> i64 {
    if v < lo {
        lo - v
    } else if v > hi {
        v - hi
    } else {
        0
    }
}

/// Whether a straight run from `(x, y)` in heading `h` (any heading for
/// [`HEADING_NONE`]) can reach box `b`.
#[inline]
fn reaches(x: i64, y: i64, h: u32, b: &TBox) -> bool {
    if h >= 8 {
        return (0..8).any(|k| reaches(x, y, k, b));
    }
    let (dc, dr) = super::heading_delta(h as u8);
    let range = |v: i64, s: i32, lo: i64, hi: i64| -> Option<(i64, i64)> {
        match s {
            0 => (lo..=hi).contains(&v).then_some((0, i64::MAX)),
            1 => Some((lo - v, hi - v)),
            _ => Some((v - hi, v - lo)),
        }
    };
    let (Some((a0, a1)), Some((b0, b1))) = (
        range(x, dc, b.min_x, b.max_x),
        range(y, dr, b.min_y, b.max_y),
    ) else {
        return false;
    };
    a0.max(b0).max(0) <= a1.min(b1)
}

/// Index arithmetic for one search. A local node index is
/// `row << wshift | col` (rows padded to a power of two), and a state is
/// `node << bshift | (layer · 9 + heading) · R + run` (each node's block
/// padded to a power of two), so decoding is shifts and masks plus a
/// division by the constant 9.
#[derive(Clone, Copy)]
struct Layout {
    layers: u32,
    runs: u32,
    region: Region,
    wshift: u32,
    bshift: u32,
}

impl Layout {
    fn new(region: Region, layers: u32, runs: u32) -> Self {
        let pow2 = |v: u32| v.max(1).next_power_of_two().trailing_zeros();
        Layout {
            layers,
            runs,
            region,
            wshift: pow2(region.w),
            bshift: pow2(layers * HS * runs),
        }
    }

    /// Node slots, including row padding.
    fn padded_nodes(&self) -> usize {
        (self.region.h as usize) << self.wshift
    }

    /// State slots, including padding.
    fn states(&self) -> usize {
        self.padded_nodes() << self.bshift
    }

    #[inline]
    fn enc(&self, node: u32, layer: u32, h: u32, run: u32) -> u32 {
        (node << self.bshift) | ((layer * HS + h) * self.runs + run)
    }

    /// `(node, layer, heading, run)`.
    #[inline]
    fn dec(&self, st: u32) -> (u32, u32, u32, u32) {
        let node = st >> self.bshift;
        let rest = st & ((1 << self.bshift) - 1);
        let (lh, run) = if self.runs == 1 {
            (rest, 0)
        } else {
            (rest / self.runs, rest % self.runs)
        };
        (node, lh / HS, lh % HS, run)
    }

    #[inline]
    fn col_row(&self, node: u32) -> (u32, u32) {
        (
            self.region.c0 + (node & ((1 << self.wshift) - 1)),
            self.region.r0 + (node >> self.wshift),
        )
    }

    #[inline]
    fn local(&self, c: u32, r: u32) -> u32 {
        ((r - self.region.r0) << self.wshift) | (c - self.region.c0)
    }
}

/// The lattice nodes inside `b` as an inclusive (col, col, row, row) box;
/// an empty box (first > last) when no node lies inside.
fn node_box<G: Grid>(grid: &G, b: konnect_geom::BBox) -> (u32, u32, u32, u32) {
    let (ca, cb) = (
        grid.col_at_or_after(b.min.x),
        grid.col_at_or_before(b.max.x),
    );
    let (ra, rb) = (
        grid.row_at_or_after(b.min.y),
        grid.row_at_or_before(b.max.y),
    );
    let inside = grid.x(ca) >= b.min.x
        && grid.x(cb) <= b.max.x
        && grid.y(ra) >= b.min.y
        && grid.y(rb) <= b.max.y;
    if inside && ca <= cb && ra <= rb {
        (ca, cb, ra, rb)
    } else {
        (1, 0, 1, 0)
    }
}

/// Track steps on stack layer `layer` may end at node `(c, r)`.
#[inline]
fn in_track_box(
    track_box: &[Option<(u32, u32, u32, u32)>; MAX_LAYERS],
    layer: usize,
    c: u32,
    r: u32,
) -> bool {
    match track_box[layer] {
        None => true,
        Some((ca, cb, ra, rb)) => c >= ca && c <= cb && r >= ra && r <= rb,
    }
}

/// The parent state of `st` from its record, or `None` for a seed.
#[inline]
fn parent_of<G: Grid>(grid: &G, lay: &Layout, st: u32, rec: StateRec) -> Option<u32> {
    let ph = u32::from((rec.info >> 1) & 15);
    if ph == u32::from(SEED) {
        return None;
    }
    let prun = u32::from((rec.info >> 5) & 31);
    let pl = u32::from((rec.info >> 10) & 31);
    let (node, layer, h, _) = lay.dec(st);
    if h == u32::from(HEADING_NONE) {
        // A via landing: same node, another layer.
        return Some(lay.enc(node, pl, ph, prun));
    }
    let (c, r) = lay.col_row(node);
    let back = grid.step(c, r, ((h + 4) & 7) as u8)?;
    Some(lay.enc(lay.local(back.col, back.row), layer, ph, prun))
}

/// Run one search. On `Mode::Field` the outcome is `Unreachable` and the
/// scratch holds the distances.
pub(crate) fn run<G: Grid>(
    grid: &G,
    p: &Problem<'_>,
    region: Region,
    s: &mut Scratch,
    stats: &mut SearchStats,
) -> SearchOutcome {
    s.capped = false;
    let job_no = p.job_index;
    let unreachable = |e| SearchOutcome::Unreachable {
        job: job_no,
        expansions: e,
    };
    let stack = p.stack;
    let nl_layers = stack.layers.len();
    if nl_layers == 0 || nl_layers > MAX_LAYERS || p.sources.is_empty() {
        return unreachable(0);
    }
    if p.mode == Mode::Route && p.targets.is_empty() {
        return unreachable(0);
    }
    for r in &stack.layers {
        if r.window.cols != grid.cols() || r.window.rows != grid.rows() {
            debug_assert!(false, "raster dimensions differ from the lattice");
            return unreachable(0);
        }
    }
    let lcount = nl_layers as u32;
    let cfg = p.config;
    let params = p.params;

    // Layer views and permissions.
    let mut views = [EMPTY_VIEW; MAX_LAYERS];
    let mut layer_ok = [false; MAX_LAYERS];
    for (i, r) in stack.layers.iter().enumerate() {
        views[i] = View {
            core: r.core.as_slice(),
            halo: r.halo.as_slice(),
            hmask: r.heading_mask.as_slice(),
            via: r.via_site.as_slice(),
            present: r.present.as_slice(),
            history: r.history.as_slice(),
            s8: r.core.stride() as usize,
            s16: r.present.stride() as usize,
        };
        let bit = r.layer.0;
        layer_ok[i] = p.job.layer_mask == 0 || (bit < 32 && p.job.layer_mask & (1 << bit) != 0);
    }
    // Per stack layer: the node box track steps must stay inside
    // ([`SearchConfig::track_region`]); `None` everywhere without one.
    let mut track_box: [Option<(u32, u32, u32, u32)>; MAX_LAYERS] = [None; MAX_LAYERS];
    if let Some(tr) = cfg.track_region {
        for (i, r) in stack.layers.iter().enumerate() {
            if r.layer == tr.layer {
                track_box[i] = Some(node_box(grid, tr.bbox));
            }
        }
    }
    let track_ok = |layer: u32, c: u32, r: u32| in_track_box(&track_box, layer as usize, c, r);
    let allow_via = p.job.has_flag(JOB_ALLOW_VIA) && lcount > 1;
    let pitch = grid.pitch().max(1);
    let run_buckets: u32 = if p.job.has_flag(JOB_MIN_RUN) && cfg.min_run_nm > 0 {
        let need = (i64::from(cfg.min_run_nm) * 2 + i64::from(pitch) - 1) / i64::from(pitch);
        if need > i64::from(MAX_RUN_BUCKET) {
            // The run field cannot count that far: refuse rather than
            // accept runs shorter than the rule.
            stats.min_run_rejects += 1;
            return unreachable(0);
        }
        need.max(0) as u32 + 1
    } else {
        1
    };
    let need_run = run_buckets - 1;
    let turns = turn_set(cfg.min_track_angle_deg);
    let mut prefs = [Preferred::None; MAX_LAYERS];
    for (i, pr) in cfg.layer_pref.iter().take(MAX_LAYERS).enumerate() {
        prefs[i] = *pr;
    }

    let lay = Layout::new(region, lcount, run_buckets);
    let nodes = lay.padded_nodes();
    let node_layers = nodes * nl_layers;
    let states = lay.states();
    if region.nodes() == 0 || states >= u32::MAX as usize {
        return unreachable(0);
    }
    s.begin(states, node_layers, nodes);
    let gen = s.gen;
    let sgen = s.sgen();
    let (c0, r0, rw, rh) = (region.c0, region.r0, region.w, region.h);
    let in_region = |c: u32, r: u32| c >= c0 && r >= r0 && c - c0 < rw && r - r0 < rh;
    let nl_index = |node: u32, layer: u32| node as usize * nl_layers + layer as usize;
    let none = u32::from(HEADING_NONE);

    // Corridor membership.
    let mut slot_of = [u32::MAX; MAX_LAYERS];
    let corridor_on = match p.gcells {
        Some(gc) if !p.corridor.is_empty() && gc.cols > 0 && gc.rows > 0 && gc.pitch > 0 => {
            let cells = gc.cols as usize * gc.rows as usize * gc.layers.count() as usize;
            if s.corr_stamp.len() < cells {
                s.corr_stamp.resize(cells, 0);
            }
            for &id in p.corridor {
                if (id as usize) < s.corr_stamp.len() {
                    s.corr_stamp[id as usize] = gen;
                }
            }
            for (i, r) in stack.layers.iter().enumerate() {
                let bit = r.layer.0;
                if bit < 32 && gc.layers.0 & (1 << bit) != 0 {
                    slot_of[i] = (gc.layers.0 & ((1u32 << bit) - 1)).count_ones();
                }
            }
            s.gcol.clear();
            s.grow.clear();
            for c in c0..c0 + rw {
                let rel = i64::from(grid.x(c)) - i64::from(gc.bbox.min.x);
                let v = rel.div_euclid(i64::from(gc.pitch));
                s.gcol.push(if (0..i64::from(gc.cols)).contains(&v) {
                    v as u32
                } else {
                    u32::MAX
                });
            }
            for r in r0..r0 + rh {
                let rel = i64::from(grid.y(r)) - i64::from(gc.bbox.min.y);
                let v = rel.div_euclid(i64::from(gc.pitch));
                s.grow.push(if (0..i64::from(gc.rows)).contains(&v) {
                    v as u32
                } else {
                    u32::MAX
                });
            }
            true
        }
        _ => false,
    };
    let (gcols, grows) = p.gcells.map_or((0, 0), |g| (g.cols, g.rows));

    // Terminals.
    let usable = |n: &JobNode| {
        let (c, r) = (u32::from(n.col), u32::from(n.row));
        u32::from(n.layer) < lcount && c < grid.cols() && r < grid.rows() && in_region(c, r)
    };
    for n in p.sources.iter().filter(|n| usable(n)) {
        let nl = nl_index(lay.local(n.col.into(), n.row.into()), n.layer.into());
        let r = s.touch(nl);
        r.tag |= SRC;
        r.src_mask |= n.heading_mask;
    }
    if p.mode == Mode::Route {
        for n in p.targets.iter().filter(|n| usable(n)) {
            let (c, r, l) = (u32::from(n.col), u32::from(n.row), u32::from(n.layer));
            let nl = nl_index(lay.local(c, r), l);
            let rec = s.touch(nl);
            rec.tag |= DST;
            rec.dst_mask |= n.heading_mask;
            // Group targets into a few boxes for the heuristic.
            let (x, y) = (i64::from(grid.x(c)), i64::from(grid.y(r)));
            let reach = 2 * i64::from(pitch);
            let hit = s.tboxes.iter().position(|b| {
                b.layer == l as u8
                    && x >= b.min_x - reach
                    && x <= b.max_x + reach
                    && y >= b.min_y - reach
                    && y <= b.max_y + reach
            });
            let idx = match hit {
                Some(i) => Some(i),
                None if s.tboxes.len() < MAX_TARGET_BOXES => {
                    s.tboxes.push(TBox {
                        layer: l as u8,
                        min_x: x,
                        max_x: x,
                        min_y: y,
                        max_y: y,
                    });
                    None
                }
                None => s
                    .tboxes
                    .iter()
                    .enumerate()
                    .min_by_key(|(_, b)| {
                        (
                            b.layer != l as u8,
                            axis_gap(x, b.min_x, b.max_x) + axis_gap(y, b.min_y, b.max_y),
                        )
                    })
                    .map(|(i, _)| i),
            };
            if let Some(i) = idx {
                let b = &mut s.tboxes[i];
                if b.layer != l as u8 {
                    // Merged across layers: the box no longer knows its
                    // layer, so it must not charge a via.
                    b.layer = u8::MAX;
                }
                b.min_x = b.min_x.min(x);
                b.max_x = b.max_x.max(x);
                b.min_y = b.min_y.min(y);
                b.max_y = b.max_y.max(y);
            }
        }
        if s.tboxes.is_empty() {
            return unreachable(0);
        }
    }

    // Via congestion sampling offsets in nodes (straight, diagonal); off
    // the uniform lattice only the via's own node is sampled.
    let via_reach: Option<(i64, i64)> = cfg.via_congestion_reach_nm.map(|r| {
        if !grid.uniform() {
            return (0, 0);
        }
        let d = (i64::from(r.max(0)) + i64::from(pitch) / 2) / i64::from(pitch);
        let e = (d as f64 * std::f64::consts::FRAC_1_SQRT_2).round() as i64;
        (d, e)
    });
    let vp = ViaPricing {
        base: params.via.0,
        reach: via_reach,
        dims: (grid.cols(), grid.rows()),
        factors: (
            u64::from(params.history_factor),
            u64::from(params.present_factor),
        ),
        check_free: cfg.check_free_vias,
        checker: p.checker.is_some(),
        near: params.near_step.0,
    };

    // The relaxed cost-to-go field, for searches that flood.
    let use_field = p.use_field && p.mode == Mode::Route;
    let inv_pitch: u64 = (1u64 << 32) / pitch as u64;
    // The sources' box in nm, for the backward search's heuristic.
    let mut sbox = (i64::MAX, i64::MIN, i64::MAX, i64::MIN);
    let mut src_layers = 0u64;
    for n in p.sources.iter().filter(|n| usable(n)) {
        src_layers |= 1 << n.layer;
        let (x, y) = (
            i64::from(grid.x(n.col.into())),
            i64::from(grid.y(n.row.into())),
        );
        sbox = (sbox.0.min(x), sbox.1.max(x), sbox.2.min(y), sbox.3.max(y));
    }
    // Per region column and row, the gap to the sources' box.
    let mut gaps = std::mem::take(&mut s.gaps);
    gaps.clear();
    if use_field && sbox.0 != i64::MAX {
        gaps.extend((c0..c0 + rw).map(|c| axis_gap(i64::from(grid.x(c)), sbox.0, sbox.1)));
        gaps.extend((r0..r0 + rh).map(|r| axis_gap(i64::from(grid.y(r)), sbox.2, sbox.3)));
    }
    let gaps = gaps;
    // A node-layer off every source's layer needs at least one via more.
    let to_sources = |c: u32, r: u32, l: u32| {
        let o = octile(
            gaps[(c - c0) as usize],
            gaps[(rw + r - r0) as usize],
            inv_pitch,
        );
        if src_layers & (1 << l) == 0 {
            o.saturating_add(params.via.0)
        } else {
            o
        }
    };
    let mut fmap = std::mem::take(&mut s.field);
    let mut fbound = FieldBound {
        stop: 0,
        complete: false,
    };
    s.fpass.clear();
    if use_field && sbox.0 != i64::MAX {
        let (settled, b) = relaxed_field(
            grid,
            &lay,
            &views,
            &layer_ok,
            &track_box,
            allow_via,
            params,
            &vp,
            p.targets,
            &to_sources,
            s,
            &mut fmap,
        );
        stats.field_settled += settled;
        fbound = b;
    }
    let fmap = fmap;
    let fpass = std::mem::take(&mut s.fpass);
    // The field's bound at a node-layer: exact where the backward search
    // closed it; elsewhere `stop - octile to the sources` (every unclosed
    // node had `d + octile >= stop`), or unreachable once it ran dry. A
    // state at a target that its mask does not end (arrived in heading `h`
    // outside it) only passes through: it takes the pass-through bound,
    // not the end state's 0, or a step into it would be inconsistent.
    let fd_at = |c: u32, r: u32, l: u32, h: u32| {
        let i = lay.local(c, r) as usize * nl_layers + l as usize;
        if fmap.closed(i) {
            let v = fmap.get(i);
            if v == 0 && h != none {
                if let Ok(k) = fpass.binary_search_by_key(&i, |p| p.nl) {
                    if fpass[k].mask & (1 << h) == 0 {
                        return fpass[k].d;
                    }
                }
            }
            v
        } else if fbound.complete {
            u32::MAX
        } else {
            fbound.stop.saturating_sub(to_sources(c, r, l))
        }
    };

    // Heuristic.
    let factor = u64::from(cfg.astar_factor.max(1000));
    let via_cost = params.via.0;
    let lookahead = if cfg.heuristic == Heuristic::Map {
        p.lookahead
    } else {
        None
    };
    // The map table is built on a uniform lattice; a non-uniform lattice's
    // shorter gaps would make it overestimate, so it is not used there.
    let lookahead = lookahead.filter(|_| grid.uniform());
    let tboxes = std::mem::take(&mut s.tboxes);
    let field = p.mode == Mode::Field;
    // Any path to a same-layer target off the state's heading ray turns at
    // least once, or uses a via, which costs more than the cheapest bend.
    let min_bend = cfg.bend_45.0.min(cfg.bend_90.0).min(via_cost);
    let heur = |c: u32, r: u32, l: u32, h: u32| -> u32 {
        if field {
            return 0;
        }
        // With the field, a state that cannot reach a target is `u32::MAX`
        // (never pushed), and every other bound is raised to the field.
        let floor = if use_field {
            let v = fd_at(c, r, l, h);
            if v == u32::MAX {
                return u32::MAX;
            }
            v
        } else {
            0
        };
        let x = i64::from(grid.x(c));
        let y = i64::from(grid.y(r));
        let scale = |best: u32| {
            let best = best.max(floor);
            if factor == 1000 {
                best
            } else {
                (u64::from(best) * factor / 1000).min(u64::from(u32::MAX >> 1)) as u32
            }
        };
        let bound = |b: &TBox| {
            let dx = axis_gap(x, b.min_x, b.max_x);
            let dy = axis_gap(y, b.min_y, b.max_y);
            let other_layer = b.layer != u8::MAX && u32::from(b.layer) != l;
            let mut base = octile(dx, dy, inv_pitch);
            let mut bend = min_bend;
            if other_layer {
                base = base.saturating_add(via_cost);
                bend = 0;
            }
            let mut m = 0;
            if let Some(la) = lookahead {
                if b.layer != u8::MAX {
                    let q = la.get(
                        l as u8,
                        b.layer,
                        dx / i64::from(pitch),
                        dy / i64::from(pitch),
                    );
                    m = q.unwrap_or(0);
                }
            }
            (base.max(m), base.saturating_add(bend).max(m))
        };
        // One box (the common case): the same bound without the tables.
        if let [b] = tboxes.as_slice() {
            let (lo, hi) = bound(b);
            let best = if lo < hi && reaches(x, y, h, b) {
                lo
            } else {
                hi
            };
            return scale(best);
        }
        // Per box, the bound is `lo` if a straight run in the state's
        // heading can reach it and `hi` otherwise: any path to a same-layer
        // target off the ray turns at least once (or vias, which costs
        // more). The minimum over boxes is `min(min hi, min lo of the
        // reachable boxes)`, and only boxes with `lo < min hi` need the ray
        // test, which is usually one box or none.
        let mut lo = [0u32; MAX_TARGET_BOXES];
        let mut hi = [0u32; MAX_TARGET_BOXES];
        let mut min_hi = u32::MAX;
        for (k, b) in tboxes.iter().enumerate() {
            (lo[k], hi[k]) = bound(b);
            min_hi = min_hi.min(hi[k]);
        }
        let mut best = min_hi;
        for (k, b) in tboxes.iter().enumerate() {
            if lo[k] < best && lo[k] < hi[k] && reaches(x, y, h, b) {
                best = lo[k];
            }
        }
        scale(best)
    };

    // Seeds.
    for n in p.sources.iter().filter(|n| usable(n)) {
        let (c, r, l) = (u32::from(n.col), u32::from(n.row), u32::from(n.layer));
        let st = lay.enc(lay.local(c, r), l, none, 0);
        let rec = &mut s.st[st as usize];
        if rec.gen == sgen {
            continue;
        }
        let hv = heur(c, r, l, none);
        if use_field && hv == u32::MAX {
            continue;
        }
        *rec = StateRec {
            g: 0,
            gen: sgen,
            info: info(u32::from(SEED), 0, 0),
        };
        s.heap.push(hv, st);
        stats.pushes += 1;
    }

    let bound = p.job.cost_bound;
    let mut expansions: u64 = 0;
    let bend45 = cfg.bend_45.0;
    let bend90 = cfg.bend_90.0;
    let near_step = params.near_step.0;
    let wrong = u64::from(params.wrong_way_step.0);
    let corr_pen = u64::from(cfg.corridor_penalty.0);
    let pf = u64::from(params.present_factor);
    let hf = u64::from(params.history_factor);
    let max_exp = cfg.max_expansions.saturating_sub(p.spent);
    let mut dead_end_checked = cfg.dead_end_check_after == 0;

    let outcome = loop {
        let Some((key, st)) = s.heap.pop() else {
            break unreachable(expansions);
        };
        let rec = s.st[st as usize];
        if rec.info & 1 == 1 {
            continue;
        }
        s.st[st as usize].info = rec.info | 1;
        let (node, layer, h, run) = lay.dec(st);
        let (col, row) = lay.col_row(node);
        let nl = nl_index(node, layer);
        let uflags = s.flags_of(nl);

        // Lazy exact checks: a state's incoming edge is unique (a track
        // state's predecessor node is fixed by its heading; a via landing
        // only comes from the same node), so checking it when the state is
        // popped rather than pushed rejects exactly the same paths and skips
        // every edge A* never settles.
        if let Some(chk) = p.checker {
            let seed = (rec.info >> 1) & 15 == SEED;
            if !seed && h == none {
                let mut check = cfg.check_free_vias;
                for vw in views.iter().take(nl_layers) {
                    if vw.via[row as usize * vw.s8 + col as usize] == VIA_CHECK {
                        check = true;
                    }
                }
                if check {
                    let n = node as usize;
                    let ok = if s.via_stamp[n] == gen {
                        stats.exact_check_cache_hits += 1;
                        s.via_state[n] == 1
                    } else {
                        stats.via_checks += 1;
                        let ok = chk.via_ok(p.job, Point::new(grid.x(col), grid.y(row)));
                        s.via_stamp[n] = gen;
                        s.via_state[n] = if ok { 1 } else { 2 };
                        ok
                    };
                    if !ok {
                        stats.lazy_rejects += 1;
                        continue;
                    }
                }
            } else if !seed {
                if let Some(back) = grid.step(col, row, ((h + 4) & 7) as u8) {
                    let vw = views[layer as usize];
                    let near = vw.halo[back.row as usize * vw.s8 + back.col as usize] > 0
                        || vw.halo[row as usize * vw.s8 + col as usize] > 0;
                    if near {
                        let pnl = nl_index(lay.local(back.col, back.row), layer);
                        let bit = 1u8 << h;
                        let cached = s.nl_of(pnl);
                        let ok = if cached.edge_seen & bit != 0 {
                            stats.exact_check_cache_hits += 1;
                            cached.edge_ok & bit != 0
                        } else {
                            stats.exact_checks += 1;
                            let a = Point::new(grid.x(back.col), grid.y(back.row));
                            let b = Point::new(grid.x(col), grid.y(row));
                            let ok = chk.segment_ok(p.job, layer as u8, a, b);
                            let rbit = 1u8 << ((h + 4) & 7);
                            let pr = s.touch(pnl);
                            pr.edge_seen |= bit;
                            if ok {
                                pr.edge_ok |= bit;
                            }
                            let ur = s.touch(nl);
                            ur.edge_seen |= rbit;
                            if ok {
                                ur.edge_ok |= rbit;
                            }
                            ok
                        };
                        if !ok {
                            stats.lazy_rejects += 1;
                            continue;
                        }
                    }
                }
            }
        }

        expansions += 1;
        let gs = rec.g;
        if expansions > p.cap && p.cap < max_exp {
            s.capped = true;
            break SearchOutcome::BudgetExhausted {
                job: job_no,
                expansions,
            };
        }
        if expansions > max_exp
            || (bound != u32::MAX && (gs > bound || (factor == 1000 && key > bound)))
        {
            break SearchOutcome::BudgetExhausted {
                job: job_no,
                expansions,
            };
        }

        if expansions == cfg.dead_end_check_after && !field && !dead_end_checked {
            dead_end_checked = true;
            stats.dead_end_checks += 1;
            let cap = (cfg.dead_end_check_after as usize).saturating_mul(4);
            if targets_enclosed(grid, &views, &lay, nl_layers, allow_via, p.targets, s, cap) {
                stats.dead_ends += 1;
                break unreachable(expansions);
            }
        }

        if uflags & DST != 0 && !field {
            let entering_ok = h == none || s.nl_of(nl).dst_mask & (1 << h) != 0;
            // A via landing (no heading) ends no track run: the run before
            // the via was already checked when the via was taken.
            if entering_ok && (run >= need_run || h == none) {
                break found(grid, p, s, &lay, st, gs, expansions);
            }
        }

        let v = views[layer as usize];
        let ui8 = row as usize * v.s8 + col as usize;
        let u_term = uflags & (SRC | DST) != 0;

        // Track moves.
        if layer_ok[layer as usize] {
            let mut allowed: u8 = if h == none {
                if uflags & SRC != 0 {
                    s.nl_of(nl).src_mask
                } else {
                    0xff
                }
            } else if run < need_run {
                1 << h
            } else {
                turns[h as usize]
            };
            // Edges the derived mask rules out stay open only into a
            // terminal (a trusted access point inside blocked cells).
            let masked: u8 = if cfg.use_heading_mask && !u_term {
                !v.hmask[ui8]
            } else {
                0
            };
            let u_halo = v.halo[ui8];
            while allowed != 0 {
                let nh = allowed.trailing_zeros();
                allowed &= allowed - 1;
                let Some(step) = grid.step(col, row, nh as u8) else {
                    continue;
                };
                if !in_region(step.col, step.row) {
                    continue;
                }
                if !track_ok(layer, step.col, step.row) {
                    continue;
                }
                let vi8 = step.row as usize * v.s8 + step.col as usize;
                let vnode = lay.local(step.col, step.row);
                let mut near = u_halo > 0 || v.halo[vi8] > 0;
                if masked & (1 << nh) != 0 || v.core[vi8] > 0 {
                    if s.flags_of(nl_index(vnode, layer)) & (SRC | DST) == 0 {
                        continue;
                    }
                    near = true;
                }
                let nrun = if run_buckets == 1 {
                    0
                } else if nh == h {
                    (run + u32::from(step.run)).min(need_run)
                } else {
                    u32::from(step.run).min(need_run)
                };
                let ns = lay.enc(vnode, layer, nh, nrun);
                let target = s.st[ns as usize];
                if target.gen == sgen && target.info & 1 == 1 {
                    continue;
                }
                // Base cost, scaled by congestion (PathFinder (b + h) · p).
                let mut base = step.cost;
                let vi16 = step.row as usize * v.s16 + step.col as usize;
                let pres = v.present[vi16];
                let hist = v.history[vi16];
                if pres | hist != 0 {
                    let b = (u64::from(base).saturating_mul(1000 + u64::from(hist) * hf) / 1000)
                        .saturating_mul(1000 + u64::from(pres) * pf)
                        / 1000;
                    base = b.min(u64::from(u32::MAX >> 2)) as u32;
                }
                // Saturating throughout: a `Cost::INFINITE` term forbids
                // the edge instead of wrapping to a cheap one.
                let mut c = base;
                if h != none && nh != h {
                    let d = (nh + 8 - h) & 7;
                    c = c.saturating_add(if d == 1 || d == 7 { bend45 } else { bend90 });
                }
                if wrong != 0 {
                    let pen = match (prefs[layer as usize], nh) {
                        (Preferred::None, _) => 0,
                        (Preferred::Horizontal, 2 | 6) | (Preferred::Vertical, 0 | 4) => {
                            wrong * u64::from(step.cost) / 1000
                        }
                        (_, n) if n & 1 == 1 => wrong * u64::from(step.cost) / 2000,
                        _ => 0,
                    };
                    c = c.saturating_add(sat32(pen));
                }
                if corridor_on {
                    let gcx = s.gcol[(step.col - c0) as usize];
                    let gcy = s.grow[(step.row - r0) as usize];
                    let slot = slot_of[layer as usize];
                    let inside = gcx != u32::MAX && gcy != u32::MAX && slot != u32::MAX && {
                        let id = ((slot * grows + gcy) * gcols + gcx) as usize;
                        id < s.corr_stamp.len() && s.corr_stamp[id] == gen
                    };
                    if !inside {
                        c = c.saturating_add(sat32(corr_pen * u64::from(step.cost) / 1000));
                    }
                }
                if near {
                    c = c.saturating_add(near_step);
                }
                let ng = gs.saturating_add(c);
                if ng == u32::MAX {
                    continue;
                }
                if target.gen != sgen || ng < target.g {
                    let hv = heur(step.col, step.row, layer, nh);
                    if use_field && hv == u32::MAX {
                        continue;
                    }
                    s.st[ns as usize] = StateRec {
                        g: ng,
                        gen: sgen,
                        info: info(h, run, layer),
                    };
                    s.heap.push(ng.saturating_add(hv), ns);
                    stats.pushes += 1;
                }
            }
        }

        // Vias: through vias, from a source or after a satisfied run.
        let via_from_here = if h == none {
            uflags & SRC != 0
        } else {
            run >= need_run
        };
        if allow_via && via_from_here {
            // The node's via cost (`u32::MAX`: no via here), once per node.
            let n = node as usize;
            let vcost = vp.cached(s, &views[..nl_layers], node, col, row);
            // An exact check may have refused this node's via.
            let ok = !(s.via_stamp[n] == gen && s.via_state[n] == 2);
            let ng = gs.saturating_add(vcost);
            // An infinite via cost forbids the via.
            if ok && ng != u32::MAX {
                for l2 in 0..lcount {
                    if l2 == layer {
                        continue;
                    }
                    if (!layer_ok[l2 as usize] || !track_ok(l2, col, row))
                        && s.flags_of(nl_index(node, l2)) & DST == 0
                    {
                        continue;
                    }
                    let ns = lay.enc(node, l2, none, 0);
                    let target = s.st[ns as usize];
                    if target.gen != sgen || (target.info & 1 == 0 && ng < target.g) {
                        let hv = heur(col, row, l2, none);
                        if use_field && hv == u32::MAX {
                            continue;
                        }
                        s.st[ns as usize] = StateRec {
                            g: ng,
                            gen: sgen,
                            info: info(h, run, layer),
                        };
                        s.heap.push(ng.saturating_add(hv), ns);
                        stats.pushes += 1;
                    }
                }
            }
        }
    };
    s.tboxes = tboxes;
    s.field = fmap;
    s.fpass = fpass;
    s.gaps = gaps;
    stats.expansions += expansions;
    outcome
}

/// How a search prices a via at a node, once per node.
#[derive(Clone, Copy)]
struct ViaPricing {
    base: u32,
    /// Congestion sampling offsets in nodes (straight, diagonal).
    reach: Option<(i64, i64)>,
    dims: (u32, u32),
    factors: (u64, u64),
    /// Add `near` where a site needs an exact check and no checker runs.
    check_free: bool,
    checker: bool,
    near: u32,
}

impl ViaPricing {
    /// The cost of a through via at `(col, row)`, `u32::MAX` where a layer
    /// refuses the site.
    #[inline]
    fn cost(&self, views: &[View<'_>], col: u32, row: u32) -> u32 {
        let mut check = self.check_free;
        for vw in views {
            let vs = vw.via[row as usize * vw.s8 + col as usize];
            if vs == VIA_NO {
                return u32::MAX;
            }
            if vs == VIA_CHECK {
                check = true;
            }
        }
        let mut vcost = self.base;
        if let Some(de) = self.reach {
            vcost = via_congestion_cost(views, col, row, de, self.dims, vcost, self.factors);
        }
        if check && !self.checker {
            vcost = vcost.saturating_add(self.near);
        }
        vcost
    }

    /// [`ViaPricing::cost`] through the search's per-node cache.
    #[inline]
    fn cached(&self, s: &mut Scratch, views: &[View<'_>], node: u32, col: u32, row: u32) -> u32 {
        let n = node as usize;
        if s.vcost_stamp[n] == s.gen {
            return s.vcost[n];
        }
        let v = self.cost(views, col, row);
        s.vcost_stamp[n] = s.gen;
        s.vcost[n] = v;
        v
    }
}

/// A via's cost scaled by the worst routed-copper congestion under it: the
/// largest `present` and `history` over every layer view, at the via's node
/// and at eight points `(d, e)` nodes out (straight, diagonal), priced
/// PathFinder-style like an edge, `b · (1 + h·hf) · (1 + p·pf)`. Saturates.
#[allow(clippy::too_many_arguments)]
fn via_congestion_cost(
    views: &[View<'_>],
    col: u32,
    row: u32,
    (d, e): (i64, i64),
    (cols, rows): (u32, u32),
    base: u32,
    (hf, pf): (u64, u64),
) -> u32 {
    let offsets = [
        (0, 0),
        (d, 0),
        (-d, 0),
        (0, d),
        (0, -d),
        (e, e),
        (e, -e),
        (-e, e),
        (-e, -e),
    ];
    let (mut pres, mut hist) = (0u64, 0u64);
    for v in views {
        for (dx, dy) in offsets {
            let (c, r) = (i64::from(col) + dx, i64::from(row) + dy);
            if c < 0 || r < 0 || c >= i64::from(cols) || r >= i64::from(rows) {
                continue;
            }
            let i = r as usize * v.s16 + c as usize;
            pres = pres.max(u64::from(v.present[i]));
            hist = hist.max(u64::from(v.history[i]));
        }
    }
    if base == u32::MAX || pres | hist == 0 {
        return base;
    }
    let b = u64::from(base).saturating_mul(1000u64.saturating_add(hist.saturating_mul(hf))) / 1000;
    let c = b.saturating_mul(1000u64.saturating_add(pres.saturating_mul(pf))) / 1000;
    c.min(u64::from(u32::MAX - 1)) as u32
}

/// The field's bound for passing through a target node-layer: arriving in
/// a heading outside its `mask` (the target's entry headings) ends nothing,
/// and the cheapest way on is a step off it (or a via) and a path from
/// there. Exact where the backward search closed that state, otherwise the
/// [`FieldBound`] value, `u32::MAX` when nothing on is reachable.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct PassBound {
    /// The target's node-layer index.
    nl: usize,
    d: u32,
    closed: bool,
    mask: u8,
}

/// Where [`relaxed_field`] stopped.
#[derive(Clone, Copy)]
struct FieldBound {
    /// The smallest open `d + octile to the sources` when it stopped.
    stop: u32,
    /// The search ran dry: every node-layer it did not close cannot reach a
    /// target.
    complete: bool,
}

/// The relaxed cost-to-go field: a backward A* from the targets toward the
/// sources over the search region, on a relaxation of the search graph
/// that drops bends, the turn rule, the heading mask, run lengths and the
/// wrong-way and corridor costs (it keeps the congestion-scaled step costs,
/// the `Near` surcharges, the walls and the real via prices). Every real
/// move is a move of the relaxation and costs at least as much, so a closed
/// node-layer's `fd` is a lower bound on the real cost to a target from any
/// state there. A target node-layer has two values: 0 for the states its
/// entry mask ends, and the cost of passing through it for the others
/// (left in `s.fpass`, [`PassBound`]). It runs until a source closes and
/// the open keys pass that source's cost by a slack; an unclosed
/// node-layer then has `d >= stop - h`, `h` the octile to the sources plus
/// a via off their layers ([`FieldBound`]). Both bounds are consistent on
/// every edge of the real search (into a target's pass-through states
/// too), so its path cost is unchanged; ties may resolve differently.
#[allow(clippy::too_many_arguments)]
fn relaxed_field<G: Grid, F: Fn(u32, u32, u32) -> u32>(
    grid: &G,
    lay: &Layout,
    views: &[View<'_>; MAX_LAYERS],
    layer_ok: &[bool; MAX_LAYERS],
    track_box: &[Option<(u32, u32, u32, u32)>; MAX_LAYERS],
    allow_via: bool,
    params: &CostParams,
    vp: &ViaPricing,
    targets: &[JobNode],
    to_sources: &F,
    s: &mut Scratch,
    fd: &mut FieldMap,
) -> (u64, FieldBound) {
    let region = lay.region;
    let nl_layers = lay.layers as usize;
    let lcount = lay.layers;
    let total = lay.padded_nodes() * nl_layers;
    fd.begin(total);
    let mut heap = std::mem::take(&mut s.fheap);
    heap.clear();
    let in_region = |c: u32, r: u32| {
        c >= region.c0 && r >= region.r0 && c - region.c0 < region.w && r - region.r0 < region.h
    };
    for t in targets {
        let (c, r, l) = (u32::from(t.col), u32::from(t.row), u32::from(t.layer));
        if l >= lcount || !in_region(c, r) {
            continue;
        }
        let nl = lay.local(c, r) as usize * nl_layers + l as usize;
        if fd.get(nl) != 0 {
            fd.set(nl, 0);
            heap.push(to_sources(c, r, l), nl as u32);
        }
    }
    let pf = u64::from(params.present_factor);
    let hf = u64::from(params.history_factor);
    let near_step = params.near_step.0;
    let uniform = grid.uniform();
    // `(dcol, drow)` per heading; with heading parity, straight or diagonal.
    const DELTAS: [(isize, isize); 8] = [
        (1, 0),
        (1, -1),
        (0, -1),
        (-1, -1),
        (-1, 0),
        (-1, 1),
        (0, 1),
        (1, 1),
    ];
    let row_step = 1isize << lay.wshift;
    let mut settled = 0u64;
    let mut limit = u32::MAX;
    let mut bound = FieldBound {
        stop: u32::MAX,
        complete: true,
    };
    // Each push carries `d + octile to the sources`; `fd` keeps `d`.
    // A target node-layer has two states: the end (`fd` 0), entered only
    // in a heading its mask allows (or by a via), and passing through it,
    // which any track step may do. The through costs live here, sorted by
    // node-layer, and their heap entries carry `THROUGH`.
    const THROUGH: u32 = 1 << 31;
    let mut through = std::mem::take(&mut s.fpass);
    through.clear();
    for t in targets {
        let (c, r, l) = (u32::from(t.col), u32::from(t.row), u32::from(t.layer));
        if l < lcount && in_region(c, r) {
            let nl = lay.local(c, r) as usize * nl_layers + l as usize;
            through.push(PassBound {
                nl,
                d: u32::MAX,
                closed: false,
                mask: s.nl_of(nl).dst_mask,
            });
        }
    }
    through.sort_unstable_by_key(|t| t.nl);
    through.dedup_by_key(|t| t.nl);
    let push = |heap: &mut RadixHeap,
                fd: &mut FieldMap,
                through: &mut [PassBound],
                pnl: usize,
                nd: u32,
                pc: u32,
                pr: u32| {
        let cur = fd.get(pnl);
        if cur == 0 {
            // A target's end state: the step passes through it instead.
            if let Ok(i) = through.binary_search_by_key(&pnl, |t| t.nl) {
                if nd < through[i].d {
                    through[i].d = nd;
                    heap.push(
                        nd.saturating_add(to_sources(pc, pr, (pnl % nl_layers) as u32)),
                        pnl as u32 | THROUGH,
                    );
                }
            }
        } else if nd < cur {
            fd.set(pnl, nd);
            heap.push(
                nd.saturating_add(to_sources(pc, pr, (pnl % nl_layers) as u32)),
                pnl as u32,
            );
        }
    };
    while let Some((key, id)) = heap.pop() {
        let is_through = id & THROUGH != 0;
        let nl = (id & !THROUGH) as usize;
        let node = (nl / nl_layers) as u32;
        let layer = nl % nl_layers;
        let (c, r) = lay.col_row(node);
        let d = if is_through {
            let Ok(i) = through.binary_search_by_key(&nl, |t| t.nl) else {
                continue;
            };
            if through[i].closed {
                continue;
            }
            through[i].d
        } else {
            if fd.closed(nl) {
                continue;
            }
            fd.get(nl)
        };
        // No freshness test: an improvement pushes a smaller key, which
        // pops (and closes the record) before any stale entry of it.
        if key > limit {
            bound = FieldBound {
                stop: key,
                complete: false,
            };
            break;
        }
        if is_through {
            if let Ok(i) = through.binary_search_by_key(&nl, |t| t.nl) {
                through[i].closed = true;
            }
        } else {
            fd.close(nl);
        }
        settled += 1;
        let v = &views[layer];
        let i8 = r as usize * v.s8 + c as usize;
        let flags = s.flags_of(nl);
        // The end state of a target is entered only in a heading its mask
        // allows; every other state may be entered from any side.
        let arrive: u8 = if flags & DST != 0 && !is_through {
            s.nl_of(nl).dst_mask
        } else {
            0xff
        };
        if flags & SRC != 0 && limit == u32::MAX {
            // A source closed: go on a small slack past its cost, so the
            // forward search finds exact bounds along near-optimal paths.
            // (Measured on real negotiation searches: a wider slack closes
            // more of the window than the forward search then saves.)
            limit = key.saturating_add(d / 256).saturating_add(1_000);
        }
        // Track steps into this node from its neighbours on this layer.
        let blocked = v.core[i8] > 0;
        let open = layer_ok[layer] && in_track_box(track_box, layer, c, r);
        if open && (!blocked || flags & (SRC | DST) != 0) {
            // A step into a blocked terminal, or touching a HALO node, pays
            // `near_step` on top (the predecessor's HALO is read below).
            let v_near = blocked || v.halo[i8] > 0;
            let i16 = r as usize * v.s16 + c as usize;
            let (pres, hist) = (v.present[i16], v.history[i16]);
            let scaled = |base: u32| {
                if pres | hist == 0 {
                    return base;
                }
                let b = (u64::from(base).saturating_mul(1000 + u64::from(hist) * hf) / 1000)
                    .saturating_mul(1000 + u64::from(pres) * pf)
                    / 1000;
                b.min(u64::from(u32::MAX >> 2)) as u32
            };
            let interior = c > region.c0
                && r > region.r0
                && c + 1 < region.c0 + region.w
                && r + 1 < region.r0 + region.h;
            if uniform && interior {
                // Every neighbour is in the region: index arithmetic only.
                let costs = [scaled(STRAIGHT), scaled(DIAGONAL)];
                for (h, &(dc, dr)) in DELTAS.iter().enumerate() {
                    // The step from that neighbour into this node heads
                    // the opposite way.
                    if arrive & (1 << ((h + 4) & 7)) == 0 {
                        continue;
                    }
                    let mut nd = d.saturating_add(costs[h & 1]);
                    let hi = (i8 as isize + dr * v.s8 as isize + dc) as usize;
                    if v_near || v.halo[hi] > 0 {
                        nd = nd.saturating_add(near_step);
                    }
                    let pnl = (nl as isize + (dr * row_step + dc) * nl_layers as isize) as usize;
                    let (pc, pr) = ((c as isize + dc) as u32, (r as isize + dr) as u32);
                    push(&mut heap, fd, &mut through, pnl, nd, pc, pr);
                }
            } else {
                for h in 0..8u8 {
                    if arrive & (1 << ((h + 4) & 7)) == 0 {
                        continue;
                    }
                    let Some(step) = grid.step(c, r, h) else {
                        continue;
                    };
                    if !in_region(step.col, step.row) {
                        continue;
                    }
                    // Lattice edges are symmetric: the step back costs the
                    // same.
                    let mut nd = d.saturating_add(scaled(step.cost));
                    if v_near || v.halo[step.row as usize * v.s8 + step.col as usize] > 0 {
                        nd = nd.saturating_add(near_step);
                    }
                    let pnl = lay.local(step.col, step.row) as usize * nl_layers + layer;
                    push(&mut heap, fd, &mut through, pnl, nd, step.col, step.row);
                }
            }
        }
        // A via landing on this node-layer, from any other layer (a via
        // landing on a target ends there, so a through state has none).
        if allow_via && !is_through && (open || flags & DST != 0) {
            let vcost = vp.cached(s, &views[..nl_layers], node, c, r);
            if vcost != u32::MAX {
                let nd = d.saturating_add(vcost);
                if nd != u32::MAX {
                    for l2 in 0..nl_layers {
                        let pnl = node as usize * nl_layers + l2;
                        if l2 != layer {
                            push(&mut heap, fd, &mut through, pnl, nd, c, r);
                        }
                    }
                }
            }
        }
    }
    s.fheap = heap;
    // An unclosed pass-through state takes the bound of every unclosed
    // node-layer.
    for t in &mut through {
        if !t.closed {
            t.d = if bound.complete {
                u32::MAX
            } else {
                let (c, r) = lay.col_row((t.nl / nl_layers) as u32);
                bound
                    .stop
                    .saturating_sub(to_sources(c, r, (t.nl % nl_layers) as u32))
            };
        }
    }
    s.fpass = through;
    (settled, bound)
}

/// Whether the targets sit in a pocket no source can reach: a breadth-first
/// flood from every target over a relaxation of the search graph (any
/// heading, no turn rule, `Near` edges and vias assumed legal, every layer
/// open) exhausts without touching a source. Every real move is a move of
/// the relaxation, so `true` proves the job unreachable. `false` means a
/// source was reached or the flood passed `cap` node-layers.
#[allow(clippy::too_many_arguments)]
fn targets_enclosed<G: Grid>(
    grid: &G,
    views: &[View<'_>; MAX_LAYERS],
    lay: &Layout,
    nl_layers: usize,
    allow_via: bool,
    targets: &[JobNode],
    s: &mut Scratch,
    cap: usize,
) -> bool {
    let gen = s.gen;
    let lcount = nl_layers as u32;
    let region = lay.region;
    s.flood_queue.clear();
    for t in targets {
        let (c, r, l) = (u32::from(t.col), u32::from(t.row), u32::from(t.layer));
        if l >= lcount
            || c < region.c0
            || r < region.r0
            || c - region.c0 >= region.w
            || r - region.r0 >= region.h
        {
            continue;
        }
        let nl = lay.local(c, r) as usize * nl_layers + l as usize;
        if s.flood_stamp[nl] != gen {
            s.flood_stamp[nl] = gen;
            s.flood_queue.push(nl as u32);
        }
    }
    let blocked = |v: &View<'_>, c: u32, r: u32| v.core[r as usize * v.s8 + c as usize] > 0;
    let mut head = 0;
    while head < s.flood_queue.len() {
        if s.flood_queue.len() > cap {
            return false;
        }
        let nl = s.flood_queue[head] as usize;
        head += 1;
        if s.flags_of(nl) & SRC != 0 {
            return false;
        }
        let node = (nl / nl_layers) as u32;
        let layer = (nl % nl_layers) as u32;
        let (c, r) = lay.col_row(node);
        let v = &views[layer as usize];
        let u_open = !blocked(v, c, r) || s.flags_of(nl) & (SRC | DST) != 0;
        if u_open {
            for h in 0..8u8 {
                let Some(st) = grid.step(c, r, h) else {
                    continue;
                };
                if st.col < region.c0
                    || st.row < region.r0
                    || st.col - region.c0 >= region.w
                    || st.row - region.r0 >= region.h
                {
                    continue;
                }
                let vnl = lay.local(st.col, st.row) as usize * nl_layers + layer as usize;
                if s.flood_stamp[vnl] == gen {
                    continue;
                }
                if blocked(v, st.col, st.row) && s.flags_of(vnl) & (SRC | DST) == 0 {
                    continue;
                }
                s.flood_stamp[vnl] = gen;
                s.flood_queue.push(vnl as u32);
            }
        }
        if allow_via {
            let site_ok = views
                .iter()
                .take(nl_layers)
                .all(|w| w.via[r as usize * w.s8 + c as usize] != VIA_NO);
            if site_ok {
                for l2 in 0..lcount {
                    let vnl = node as usize * nl_layers + l2 as usize;
                    if s.flood_stamp[vnl] != gen {
                        s.flood_stamp[vnl] = gen;
                        s.flood_queue.push(vnl as u32);
                    }
                }
            }
        }
    }
    true
}

/// Rebuild the path ending at state `goal` as corner nodes.
fn found<G: Grid>(
    grid: &G,
    p: &Problem<'_>,
    s: &mut Scratch,
    lay: &Layout,
    goal: u32,
    cost: u32,
    expansions: u64,
) -> SearchOutcome {
    s.chain.clear();
    let mut cur = Some(goal);
    while let Some(st) = cur {
        s.chain.push(st);
        cur = parent_of(grid, lay, st, s.st[st as usize]);
        if s.chain.len() > s.st.len() {
            debug_assert!(false, "parent cycle");
            break;
        }
    }
    let decode = |st: u32| -> (u16, u16, u8, u32) {
        let (node, layer, h, _) = lay.dec(st);
        let (c, r) = lay.col_row(node);
        (c as u16, r as u16, layer as u8, h)
    };
    let mk = |c: u16, r: u16, l: u8| JobNode {
        col: c,
        row: r,
        layer: l,
        heading_mask: 0,
        reserved: 0,
    };
    // Walk forward: the chain is goal-first.
    let none = u32::from(HEADING_NONE);
    let n = s.chain.len();
    let mut out: Vec<JobNode> = Vec::with_capacity(16);
    let mut prev = decode(s.chain[n - 1]);
    out.push(mk(prev.0, prev.1, prev.2));
    for i in (0..n - 1).rev() {
        let cur = decode(s.chain[i]);
        let (pc, pr, pl, ph) = prev;
        let (cc, cr, cl, ch) = cur;
        let last = out[out.len() - 1];
        let at_last = last.col == pc && last.row == pr && last.layer == pl;
        if cl != pl {
            // A via: both ends at the same (col, row).
            if !at_last {
                out.push(mk(pc, pr, pl));
            }
            out.push(mk(cc, cr, cl));
        } else if ph != none && ch != ph && !at_last {
            out.push(mk(pc, pr, pl));
        }
        prev = cur;
    }
    let (gc, gr, gl, _) = prev;
    let last = out[out.len() - 1];
    if !(last.col == gc && last.row == gr && last.layer == gl) {
        out.push(mk(gc, gr, gl));
    }
    // Record each node's leaving heading (0 on the last node and on the
    // first node of a via pair).
    for i in 0..out.len().saturating_sub(1) {
        let (a, b) = (out[i], out[i + 1]);
        if a.layer != b.layer {
            continue;
        }
        let dc = i32::from(b.col) - i32::from(a.col);
        let dr = i32::from(b.row) - i32::from(a.row);
        out[i].heading_mask = super::heading_of(dc.signum(), dr.signum()).map_or(0, |h| 1 << h);
    }
    SearchOutcome::Found(PathProposal {
        job: p.job_index,
        nodes: out,
        cost: Cost(cost),
        expansions,
    })
}

/// After a `Mode::Field` run over `region`: the minimum cost per (layer,
/// node), row-major per layer, `Cost::INFINITE` where unreached.
pub(crate) fn field_costs(
    s: &Scratch,
    region: Region,
    lcount: u32,
    run_buckets: u32,
    out: &mut Vec<Cost>,
) {
    let lay = Layout::new(region, lcount, run_buckets);
    let nodes = region.nodes();
    out.clear();
    out.resize(nodes * lcount as usize, Cost::INFINITE);
    let sgen = s.sgen();
    let per_layer = HS * run_buckets;
    for r in region.r0..region.r0 + region.h {
        for c in region.c0..region.c0 + region.w {
            let node = lay.local(c, r);
            let flat = ((r - region.r0) * region.w + (c - region.c0)) as usize;
            for l in 0..lcount {
                let base = lay.enc(node, l, 0, 0) as usize;
                for rec in &s.st[base..base + per_layer as usize] {
                    if rec.gen == sgen {
                        let o = &mut out[l as usize * nodes + flat];
                        o.0 = o.0.min(rec.g);
                    }
                }
            }
        }
    }
}
