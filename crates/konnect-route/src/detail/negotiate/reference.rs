//! A simple, exact-on-the-rasters search backend negotiation can run
//! against until WS-D1's search lands (the integrator swaps it in through
//! the same [`SearchBackend`] seam).
//!
//! Octilinear A* over `(node, layer, heading)` on one tile stack:
//!
//! - moves are straight, ±45° and ±90° ([`turn_allowed`]: 135° turns and
//!   reversals are refused), plus a through via where every layer's
//!   `via_site` allows it;
//! - `Blocked` nodes are walls; `Near` nodes cost `near_step` extra and are
//!   re-checked exactly by the caller before commit;
//! - other nets' routed copper costs PathFinder `(b + b·h·hf) · (1 + p·pf)`
//!   from the `present` and `history` planes; a via is priced by the worst
//!   congestion sampled around it on every layer, since it is wider than
//!   the track the planes are painted for;
//! - a restricted terminal (a pin's heading mask) is entered and left on its
//!   layer by one of its headings, never by a via;
//! - the heuristic is the octile distance to the targets' box plus a via
//!   when no target is on the current layer, scaled by
//!   [`ReferenceSearch::heuristic_weight`] (1.0 is admissible; the default
//!   1.2 is VPR's `astar_fac`);
//! - ties break on `(f, h, state index)`, so results never depend on timing.
//!
//! Search state lives in per-thread scratch reused across jobs (stamped,
//! never cleared), so a search allocates nothing per step. A search first
//! runs in the terminals' box plus a detour margin and falls back to the
//! whole window only when the boxed search pressed against the box edge
//! (otherwise a larger region holds nothing more to reach).

use crate::api::LayerMask;
use crate::compute::Cost;
use crate::compute::{
    Batch, CostField, CostParams, CpuCompute, JobDescriptor, JobNode, JobPool, PaintShape,
    PathProposal, ProfileGeometry, RouteCompute, SearchBackend, SearchOutcome, TileRaster,
    TileStack,
};
use crate::detail::search::{heading_delta, turn_allowed, HEADINGS};
use konnect_geom::Coord;
use rayon::prelude::*;
use std::cell::RefCell;
use std::cmp::Reverse;
use std::collections::BinaryHeap;
use std::sync::Arc;

/// Heading slot meaning "no heading yet" (a start, or just after a via).
const FREE: u32 = 8;
const HEADING_SLOTS: u32 = 9;

/// The reference [`SearchBackend`] + [`RouteCompute`] (raster maintenance
/// delegates to [`CpuCompute`]).
#[derive(Clone)]
pub struct ReferenceSearch {
    /// Expansions per job before [`SearchOutcome::BudgetExhausted`].
    pub max_expansions: u64,
    /// How far beyond the terminals' box the first attempt may wander.
    pub detour_margin_nm: Coord,
    /// How far from a via centre congestion is sampled: about the via
    /// radius less the track half width.
    pub via_reach_nm: Coord,
    /// Weighted A*: the heuristic is scaled by this, ×1000. 1000 is exact
    /// (optimal paths); above trades path cost for far fewer expansions.
    pub heuristic_weight: u32,
    pool: Option<Arc<rayon::ThreadPool>>,
}

impl std::fmt::Debug for ReferenceSearch {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ReferenceSearch")
            .field("max_expansions", &self.max_expansions)
            .field("detour_margin_nm", &self.detour_margin_nm)
            .field(
                "threads",
                &self.pool.as_ref().map(|p| p.current_num_threads()),
            )
            .finish()
    }
}

impl Default for ReferenceSearch {
    fn default() -> Self {
        ReferenceSearch::new(0)
    }
}

impl ReferenceSearch {
    /// `threads == 0` runs batches on rayon's global pool.
    pub fn new(threads: usize) -> Self {
        let pool = (threads > 0).then(|| {
            Arc::new(
                rayon::ThreadPoolBuilder::new()
                    .num_threads(threads)
                    .build()
                    .expect("a rayon pool builds"),
            )
        });
        ReferenceSearch {
            max_expansions: 2_000_000,
            detour_margin_nm: 3_000_000,
            via_reach_nm: 250_000,
            heuristic_weight: 1_200,
            pool,
        }
    }

    fn run<R: Send>(&self, f: impl FnOnce() -> R + Send) -> R {
        match &self.pool {
            Some(p) => p.install(f),
            None => f(),
        }
    }
}

/// Reusable per-thread search state.
#[derive(Default)]
struct Scratch {
    g: Vec<u32>,
    parent: Vec<u32>,
    stamp: Vec<u32>,
    cur: u32,
    heap: BinaryHeap<Reverse<(u32, u32, u32)>>,
}

impl Scratch {
    fn begin(&mut self, n: usize) {
        if self.stamp.len() < n {
            self.g.resize(n, 0);
            self.parent.resize(n, 0);
            self.stamp.resize(n, 0);
        }
        self.cur = self.cur.wrapping_add(1);
        if self.cur == 0 {
            self.stamp.iter_mut().for_each(|s| *s = 0);
            self.cur = 1;
        }
        self.heap.clear();
    }
}

thread_local! {
    static SCRATCH: RefCell<Scratch> = RefCell::new(Scratch::default());
}

/// A path: nodes, cost, expansions.
type Found = (Vec<JobNode>, u32, u64);
/// No path: (search space exhausted rather than budget, expansions,
/// whether the region edge was reached).
type Missed = (bool, u64, bool);

/// A rectangular sub-range of the window, in nodes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Region {
    c0: u32,
    r0: u32,
    w: u32,
    h: u32,
}

struct Problem<'a> {
    stack: &'a TileStack,
    params: &'a CostParams,
    allowed: Vec<bool>,
    /// (state-independent node key, heading mask), sorted by key.
    sources: Vec<(u64, u8)>,
    targets: Vec<(u64, u8)>,
    target_box: (u32, u32, u32, u32),
    target_layers: Vec<bool>,
    max_expansions: u64,
    bound: u32,
    /// Via sampling radius, in nodes.
    via_reach: u32,
    /// Heuristic weight, ×1000.
    weight: u32,
}

fn node_key(slot: u32, col: u32, row: u32) -> u64 {
    (u64::from(slot) << 40) | (u64::from(row) << 20) | u64::from(col)
}

fn octile(dx: u32, dy: u32) -> u32 {
    let (a, b) = (dx.max(dy), dx.min(dy));
    a.saturating_mul(Cost::STEP_STRAIGHT.0)
        .saturating_add(b.saturating_mul(Cost::STEP_DIAGONAL.0 - Cost::STEP_STRAIGHT.0))
}

impl Problem<'_> {
    fn blocked(&self, slot: u32, col: u32, row: u32) -> bool {
        self.stack.layers[slot as usize].core.get(col, row) > 0
    }

    fn h(&self, slot: u32, col: u32, row: u32) -> u32 {
        let (cmin, cmax, rmin, rmax) = self.target_box;
        let dx = if col < cmin {
            cmin - col
        } else {
            col.saturating_sub(cmax)
        };
        let dy = if row < rmin {
            rmin - row
        } else {
            row.saturating_sub(rmax)
        };
        let via = if self.target_layers[slot as usize] {
            0
        } else {
            self.params.via.0
        };
        let h = u64::from(octile(dx, dy).saturating_add(via));
        u32::try_from(h * u64::from(self.weight) / 1000).unwrap_or(u32::MAX)
    }

    /// Cost of entering `(slot, col, row)` by a move of base cost `base`.
    fn enter(&self, slot: u32, col: u32, row: u32, base: u32) -> u32 {
        let r: &TileRaster = &self.stack.layers[slot as usize];
        let near = if r.halo.get(col, row) > 0 {
            u64::from(self.params.near_step.0)
        } else {
            0
        };
        let present = u64::from(r.present.get(col, row));
        let hist = u64::from(r.history.get(col, row));
        let b = u64::from(base);
        let bh = b + b * hist * u64::from(self.params.history_factor) / 1000;
        let c = bh * (1000 + u64::from(self.params.present_factor) * present) / 1000 + near;
        u32::try_from(c).unwrap_or(u32::MAX)
    }

    /// Cost of a through via at `(col, row)`. A via is wider than the track
    /// the `present` plane is painted for, and it occupies every layer, so
    /// congestion is sampled on every layer at the centre and at eight
    /// points `via_reach` out, and the worst sample prices the via.
    fn via_cost(&self, col: u32, row: u32) -> u32 {
        let w = self.stack.window;
        let d = self.via_reach as i64;
        let e = (self.via_reach as f64 * std::f64::consts::FRAC_1_SQRT_2).round() as i64;
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
        let (mut present, mut hist, mut near) = (0u64, 0u64, false);
        for r in &self.stack.layers {
            for (dx, dy) in offsets {
                let (c, rr) = (i64::from(col) + dx, i64::from(row) + dy);
                if c < 0 || rr < 0 || c >= i64::from(w.cols) || rr >= i64::from(w.rows) {
                    continue;
                }
                let (c, rr) = (c as u32, rr as u32);
                present = present.max(u64::from(r.present.get(c, rr)));
                hist = hist.max(u64::from(r.history.get(c, rr)));
                near |= r.halo.get(c, rr) > 0;
            }
        }
        let b = u64::from(self.params.via.0);
        let bh = b + b * hist * u64::from(self.params.history_factor) / 1000;
        let near = if near {
            u64::from(self.params.near_step.0)
        } else {
            0
        };
        let c = bh * (1000 + u64::from(self.params.present_factor) * present) / 1000 + near;
        u32::try_from(c).unwrap_or(u32::MAX)
    }

    fn via_ok(&self, col: u32, row: u32) -> bool {
        self.stack
            .layers
            .iter()
            .all(|r| r.via_site.get(col, row) != 0)
    }

    fn lookup(list: &[(u64, u8)], key: u64) -> Option<u8> {
        list.binary_search_by_key(&key, |e| e.0)
            .ok()
            .map(|i| list[i].1)
    }

    fn run(&self, region: Region, s: &mut Scratch) -> Result<Found, Missed> {
        let layers = self.stack.layers.len() as u32;
        let (rw, rh) = (region.w, region.h);
        let n = (layers * rw * rh * HEADING_SLOTS) as usize;
        s.begin(n);
        let idx = |slot: u32, c: u32, r: u32, hd: u32| -> u32 {
            ((slot * rh + (r - region.r0)) * rw + (c - region.c0)) * HEADING_SLOTS + hd
        };
        let inside = |c: u32, r: u32| {
            c >= region.c0 && r >= region.r0 && c < region.c0 + rw && r < region.r0 + rh
        };
        let decode = |i: u32| -> (u32, u32, u32, u32) {
            let hd = i % HEADING_SLOTS;
            let rest = i / HEADING_SLOTS;
            let c = rest % rw + region.c0;
            let rest = rest / rw;
            let r = rest % rh + region.r0;
            (rest / rh, c, r, hd)
        };
        let cur = s.cur;
        let mut seeded = false;
        for &(key, _) in &self.sources {
            let (slot, row, col) = (
                (key >> 40) as u32,
                ((key >> 20) & 0xf_ffff) as u32,
                (key & 0xf_ffff) as u32,
            );
            if !inside(col, row) {
                continue;
            }
            let i = idx(slot, col, row, FREE);
            if s.stamp[i as usize] == cur {
                continue;
            }
            s.stamp[i as usize] = cur;
            s.g[i as usize] = 0;
            s.parent[i as usize] = u32::MAX;
            let h = self.h(slot, col, row);
            s.heap.push(Reverse((h, h, i)));
            seeded = true;
        }
        if !seeded {
            return Err((true, 0, true));
        }
        let mut expansions = 0u64;
        // Whether the search pressed against the region edge (a larger
        // region could then help).
        let mut touched = false;
        while let Some(Reverse((f, h, i))) = s.heap.pop() {
            let g = s.g[i as usize];
            if g.saturating_add(h) != f {
                continue; // stale entry
            }
            if f > self.bound {
                return Err((false, expansions, true));
            }
            let (slot, c, r, hd) = decode(i);
            let key = node_key(slot, c, r);
            if let Some(mask) = Self::lookup(&self.targets, key) {
                // A restricted target (a pin) must be entered on its layer by
                // one of its headings, never by a via.
                let ok = if hd == FREE {
                    g == 0 || mask == 0xff
                } else {
                    mask & (1 << hd) != 0
                };
                if ok {
                    let mut nodes = Vec::new();
                    let mut at = i;
                    loop {
                        let (sl, cc, rr, _) = decode(at);
                        let node = JobNode {
                            col: cc as u16,
                            row: rr as u16,
                            layer: sl as u8,
                            heading_mask: 0,
                            reserved: 0,
                        };
                        if nodes.last() != Some(&node) {
                            nodes.push(node);
                        }
                        let p = s.parent[at as usize];
                        if p == u32::MAX {
                            break;
                        }
                        at = p;
                    }
                    nodes.reverse();
                    return Ok((nodes, g, expansions));
                }
            }
            expansions += 1;
            if expansions > self.max_expansions {
                return Err((false, expansions, true));
            }
            // Departure headings: any from a fresh start except as the source
            // mask allows; otherwise straight, 45° or 90°.
            let source_mask = if hd == FREE && g == 0 {
                Self::lookup(&self.sources, key).unwrap_or(0xff)
            } else {
                0xff
            };
            for nh in 0..8u32 {
                if hd != FREE && !turn_allowed(hd as u8, nh as u8) {
                    continue;
                }
                if source_mask & (1 << nh) == 0 {
                    continue;
                }
                // Heading bits are Y-up (the raster contract); rows grow
                // with board y, so the step is `heading_delta`'s.
                let (dx, dy) = heading_delta(nh as u8);
                let (nc, nr) = (c as i64 + i64::from(dx), r as i64 + i64::from(dy));
                if nc < 0 || nr < 0 {
                    continue;
                }
                let (nc, nr) = (nc as u32, nr as u32);
                if !inside(nc, nr) {
                    if nc < self.stack.window.cols
                        && nr < self.stack.window.rows
                        && !self.blocked(slot, nc, nr)
                    {
                        touched = true;
                    }
                    continue;
                }
                if self.blocked(slot, nc, nr) {
                    continue;
                }
                let base = if dx != 0 && dy != 0 {
                    Cost::STEP_DIAGONAL.0
                } else {
                    Cost::STEP_STRAIGHT.0
                };
                let ng = g.saturating_add(self.enter(slot, nc, nr, base));
                let j = idx(slot, nc, nr, nh);
                if s.stamp[j as usize] != cur || ng < s.g[j as usize] {
                    s.stamp[j as usize] = cur;
                    s.g[j as usize] = ng;
                    s.parent[j as usize] = i;
                    let nh_ = self.h(slot, nc, nr);
                    s.heap.push(Reverse((ng.saturating_add(nh_), nh_, j)));
                }
            }
            // A through via, unless this state was itself reached by one.
            let after_via = hd == FREE && g > 0;
            // Nor may a restricted source (a pin) be left by a via.
            if !after_via && source_mask == 0xff && self.via_ok(c, r) {
                let via_cost = self.via_cost(c, r);
                for ns in 0..layers {
                    if ns == slot || !self.allowed[ns as usize] || self.blocked(ns, c, r) {
                        continue;
                    }
                    let ng = g.saturating_add(via_cost);
                    let j = idx(ns, c, r, FREE);
                    if s.stamp[j as usize] != cur || ng < s.g[j as usize] {
                        s.stamp[j as usize] = cur;
                        s.g[j as usize] = ng;
                        s.parent[j as usize] = i;
                        let nh_ = self.h(ns, c, r);
                        s.heap.push(Reverse((ng.saturating_add(nh_), nh_, j)));
                    }
                }
            }
        }
        Err((true, expansions, touched))
    }
}

/// Run one job: the terminals' box plus margin first, then the window.
fn search_one(
    cfg: &ReferenceSearch,
    stack: &TileStack,
    pool: &JobPool,
    job_index: u32,
    params: &CostParams,
) -> SearchOutcome {
    let job: &JobDescriptor = &pool.jobs[job_index as usize];
    let w = stack.window;
    let mask = LayerMask(job.layer_mask);
    let allowed: Vec<bool> = stack
        .layers
        .iter()
        .map(|r| mask.contains(r.layer))
        .collect();
    let pick = |nodes: &[JobNode], p: &Problem<'_>| -> Vec<(u64, u8)> {
        let mut v: Vec<(u64, u8)> = nodes
            .iter()
            .filter(|n| {
                (n.layer as usize) < allowed.len()
                    && allowed[n.layer as usize]
                    && u32::from(n.col) < w.cols
                    && u32::from(n.row) < w.rows
                    && !p.blocked(u32::from(n.layer), u32::from(n.col), u32::from(n.row))
            })
            .map(|n| {
                (
                    node_key(u32::from(n.layer), u32::from(n.col), u32::from(n.row)),
                    n.heading_mask,
                )
            })
            .collect();
        v.sort_unstable();
        // Duplicate keys: union their heading masks.
        let mut out: Vec<(u64, u8)> = Vec::with_capacity(v.len());
        for (k, m) in v {
            match out.last_mut() {
                Some(last) if last.0 == k => last.1 |= m,
                _ => out.push((k, m)),
            }
        }
        out
    };
    let mut p = Problem {
        stack,
        params,
        allowed: allowed.clone(),
        sources: Vec::new(),
        targets: Vec::new(),
        target_box: (0, 0, 0, 0),
        target_layers: vec![false; stack.layers.len()],
        max_expansions: cfg.max_expansions,
        bound: job.cost_bound,
        via_reach: (cfg.via_reach_nm.max(0) / w.pitch_nm.max(1)) as u32,
        weight: cfg.heuristic_weight.max(1000),
    };
    p.sources = pick(pool.sources(job), &p);
    p.targets = pick(pool.targets(job), &p);
    if p.sources.is_empty() || p.targets.is_empty() {
        return SearchOutcome::Unreachable {
            job: job_index,
            expansions: 0,
        };
    }
    let decode = |k: u64| {
        (
            (k >> 40) as u32,
            ((k >> 20) & 0xf_ffff) as u32,
            (k & 0xf_ffff) as u32,
        )
    };
    let (mut cmin, mut cmax, mut rmin, mut rmax) = (u32::MAX, 0, u32::MAX, 0);
    for &(k, _) in &p.targets {
        let (s, r, c) = decode(k);
        p.target_layers[s as usize] = true;
        cmin = cmin.min(c);
        cmax = cmax.max(c);
        rmin = rmin.min(r);
        rmax = rmax.max(r);
    }
    p.target_box = (cmin, cmax, rmin, rmax);
    let (mut bc0, mut bc1, mut br0, mut br1) = (cmin, cmax, rmin, rmax);
    for &(k, _) in &p.sources {
        let (_, r, c) = decode(k);
        bc0 = bc0.min(c);
        bc1 = bc1.max(c);
        br0 = br0.min(r);
        br1 = br1.max(r);
    }
    let pitch = w.pitch_nm.max(1);
    let m = (cfg.detour_margin_nm / pitch).max(4) as u32;
    let boxed = Region {
        c0: bc0.saturating_sub(m),
        r0: br0.saturating_sub(m),
        w: (bc1 + m).min(w.cols - 1) - bc0.saturating_sub(m) + 1,
        h: (br1 + m).min(w.rows - 1) - br0.saturating_sub(m) + 1,
    };
    let whole = Region {
        c0: 0,
        r0: 0,
        w: w.cols,
        h: w.rows,
    };
    let attempts: &[Region] = if boxed == whole {
        &[whole]
    } else {
        &[boxed, whole]
    };
    SCRATCH.with(|cell| {
        let mut s = cell.borrow_mut();
        let mut spent = 0u64;
        for &region in attempts {
            match p.run(region, &mut s) {
                Ok((nodes, cost, e)) => {
                    return SearchOutcome::Found(PathProposal {
                        job: job_index,
                        nodes,
                        cost: Cost(cost),
                        expansions: spent + e,
                    })
                }
                Err((exhausted, e, touched)) => {
                    spent += e;
                    if !exhausted {
                        return SearchOutcome::BudgetExhausted {
                            job: job_index,
                            expansions: spent,
                        };
                    }
                    if !touched {
                        // The search never reached the region edge: a larger
                        // region holds no more reachable nodes.
                        break;
                    }
                }
            }
        }
        SearchOutcome::Unreachable {
            job: job_index,
            expansions: spent,
        }
    })
}

impl SearchBackend for ReferenceSearch {
    fn search_batch(
        &mut self,
        batch: &Batch,
        pool: &JobPool,
        stacks: &[TileStack],
        params: &CostParams,
    ) -> Vec<SearchOutcome> {
        if batch.jobs.len() == 1 {
            // One job (a forked copy's case): no pool hop.
            let j = batch.jobs[0];
            let job = &pool.jobs[j as usize];
            return vec![search_one(
                self,
                &stacks[job.window as usize],
                pool,
                j,
                params,
            )];
        }
        let cfg = self.clone();
        self.run(|| {
            batch
                .jobs
                .par_iter()
                .map(|&j| {
                    let job = &pool.jobs[j as usize];
                    search_one(&cfg, &stacks[job.window as usize], pool, j, params)
                })
                .collect()
        })
    }

    /// Dijkstra from `sources` over the stack (no heading state): the
    /// cheapest cost to reach each node, row-major per layer.
    fn cost_field(
        &mut self,
        stack: &TileStack,
        sources: &[JobNode],
        params: &CostParams,
    ) -> CostField {
        let w = stack.window;
        let layers = stack.layers.len();
        let plane = w.node_count();
        let mut costs = vec![Cost::INFINITE; plane * layers];
        let mut heap = BinaryHeap::new();
        let at = |s: usize, c: u32, r: u32| s * plane + r as usize * w.cols as usize + c as usize;
        let p = Problem {
            stack,
            params,
            allowed: vec![true; layers],
            sources: Vec::new(),
            targets: Vec::new(),
            target_box: (0, 0, 0, 0),
            target_layers: vec![true; layers],
            max_expansions: u64::MAX,
            bound: u32::MAX,
            via_reach: (self.via_reach_nm.max(0) / w.pitch_nm.max(1)) as u32,
            weight: 1000,
        };
        for n in sources {
            let (s, c, r) = (n.layer as usize, u32::from(n.col), u32::from(n.row));
            if s < layers && c < w.cols && r < w.rows && !p.blocked(s as u32, c, r) {
                costs[at(s, c, r)] = Cost::ZERO;
                heap.push(Reverse((0u32, at(s, c, r))));
            }
        }
        while let Some(Reverse((g, i))) = heap.pop() {
            if costs[i].0 != g {
                continue;
            }
            let s = i / plane;
            let c = ((i % plane) % w.cols as usize) as u32;
            let r = ((i % plane) / w.cols as usize) as u32;
            let mut relax = |j: usize, ng: u32, heap: &mut BinaryHeap<Reverse<(u32, usize)>>| {
                if ng < costs[j].0 {
                    costs[j] = Cost(ng);
                    heap.push(Reverse((ng, j)));
                }
            };
            for (dx, dy) in HEADINGS {
                let (nc, nr) = (c as i64 + i64::from(dx), r as i64 + i64::from(dy));
                if nc < 0 || nr < 0 || nc >= i64::from(w.cols) || nr >= i64::from(w.rows) {
                    continue;
                }
                let (nc, nr) = (nc as u32, nr as u32);
                if p.blocked(s as u32, nc, nr) {
                    continue;
                }
                let base = if dx != 0 && dy != 0 {
                    Cost::STEP_DIAGONAL.0
                } else {
                    Cost::STEP_STRAIGHT.0
                };
                relax(
                    at(s, nc, nr),
                    g.saturating_add(p.enter(s as u32, nc, nr, base)),
                    &mut heap,
                );
            }
            if p.via_ok(c, r) {
                for ns in 0..layers {
                    if ns != s && !p.blocked(ns as u32, c, r) {
                        relax(
                            at(ns, c, r),
                            g.saturating_add(p.enter(ns as u32, c, r, params.via.0)),
                            &mut heap,
                        );
                    }
                }
            }
        }
        CostField {
            window: w,
            layer_count: layers as u8,
            costs,
        }
    }
}

impl RouteCompute for ReferenceSearch {
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

    /// Every heading may leave a node that is not blocked; a via may sit on
    /// any node that is not blocked. Negotiation narrows `via_site` with
    /// the via's own size afterwards.
    fn derive_masks(&mut self, raster: &mut TileRaster) {
        let w = raster.window;
        for row in 0..w.rows {
            for col in 0..w.cols {
                let open = raster.core.get(col, row) == 0;
                raster
                    .heading_mask
                    .set(col, row, if open { 0xff } else { 0 });
                raster.via_site.set(col, row, u8::from(open));
            }
        }
    }

    fn update_costs(&mut self, raster: &mut TileRaster, params: &CostParams) {
        CpuCompute::new().update_costs(raster, params);
    }

    /// A copy without its own pool: a forked copy runs one job at a time on
    /// the caller's worker thread.
    fn fork(&self) -> Option<Box<dyn RouteCompute + Send>> {
        Some(Box::new(ReferenceSearch {
            pool: None,
            ..self.clone()
        }))
    }
}
