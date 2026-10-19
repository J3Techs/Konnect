//! One tile's share of a pass: build its rasters, then route its queued
//! pieces one at a time (one per batch round), committing each exactly.
//!
//! Rasters are per (tile, profile, routable layer). Hard paint is every
//! obstacle near the window; soft paint (`present`) is every routed piece
//! near it; history comes from the run's map; nodes outside the commit zone
//! are walled off. Before a piece is searched its own net is *lifted* — its
//! fixed copper and routed pieces unpainted from the profile's stack — so a
//! net never pays for, or is blocked by, itself; the same shapes are painted
//! back afterwards (counters make that exact).

use super::cache::{HardLayer, RasterCache, RasterPool, TileStatic};
use super::conflicts::HistoryMap;
use super::copper::{PieceCopper, SegCopper, ViaCopper};
use super::obstacles::ObstacleKind;
use super::pieces::{PieceJob, Terminal};
use super::tiles::TileSpec;
use super::{Ctx, DetailFailure, Task};
use crate::api::{ItemKind, ItemRef, NetId, ProfileId};
use crate::compute::job::{JOB_ALLOW_VIA, JOB_REROUTE};
use crate::compute::{
    Batch, CostParams, CpuCompute, JobDescriptor, JobNode, JobPool, PaintKind, RouteCompute,
    SearchOutcome, TileRaster, TileStack, Window,
};
use crate::rules::RuleSubject;
use crate::telemetry::Telemetry;
use konnect_geom::{gap_cmp, touches, Circle, Coord, Point, Segment, Shape};
use std::cmp::Ordering;
use std::collections::{BTreeMap, VecDeque};
use std::sync::Arc;

/// A piece's outcome.
#[derive(Debug, Clone)]
pub(crate) struct PieceDone {
    pub task: u32,
    pub piece: u8,
    pub result: Result<Arc<PieceCopper>, DetailFailure>,
}

/// A job ready for the backend.
pub(crate) struct Prepared {
    pub stack: usize,
    pub job: JobDescriptor,
    pub sources: Vec<JobNode>,
    pub targets: Vec<JobNode>,
}

pub(crate) struct TileRun {
    id: u32,
    /// Counter suffix: `bga` for sub-tiles, `grid` otherwise.
    phase: &'static str,
    lattice: Window,
    profiles: Vec<u16>,
    stacks: Vec<Option<TileStack>>,
    /// Stacks whose hard state was just painted (masks still to derive).
    fresh: Vec<bool>,
    /// The tile's obstacle lists, and per net the (slot, obstacle id) of
    /// its painted fixed copper.
    stat: Arc<TileStatic>,
    /// Per net: routed pieces painted into this tile.
    soft: BTreeMap<NetId, Vec<Arc<PieceCopper>>>,
    queue: VecDeque<PieceJob>,
    active: Option<(PieceJob, usize)>,
    pub done: Vec<PieceDone>,
    pub telemetry: Telemetry,
    /// Wall time `route_all` took.
    pub busy_us: u64,
    /// Fill mode: other nets' routed copper is painted as a hard wall (CORE
    /// and HALO), not as soft congestion, so a fill pass can only commit
    /// copper that keeps clear of it.
    hard_routed: bool,
    /// Via sites overwritten while a net's own copper is lifted, put back
    /// when it is painted again.
    saved_sites: Vec<SavedSites>,
    /// The active job's pad stubs: terminal nodes reached through a short
    /// exact-checked track from a point in a pad.
    stubs: Vec<Stub>,
}

/// A terminal node outside a pad, joined to it by a stub track.
#[derive(Debug, Clone, Copy)]
struct Stub {
    col: u16,
    row: u16,
    slot: u8,
    /// The stub's end inside the pad.
    from: Point,
    width: Coord,
}

/// Via sites of one stack layer over a node range, saved before the net
/// being routed reopened them.
struct SavedSites {
    stack: usize,
    slot: usize,
    range: (u32, u32, u32, u32),
    values: Vec<u8>,
}

pub(crate) fn half_diag(pitch: i32) -> i32 {
    ((f64::from(pitch) * std::f64::consts::FRAC_1_SQRT_2).ceil()) as i32
}

/// S4 exit headings (0 = +x, counter-clockwise in the Y-up sense) as the
/// kernel's headings ([`crate::detail::search::HEADINGS`], lattice steps in
/// the Y-down board frame).
fn access_to_kernel(mask: u8) -> u8 {
    let mut out = 0u8;
    for h in 0..8u8 {
        if mask & (1 << h) != 0 {
            // S4 heading h points at angle h * 45 degrees with y up, i.e.
            // (cos, -sin) in board coordinates.
            let a = f64::from(h) * std::f64::consts::FRAC_PI_4;
            let (dx, dy) = (a.cos().round() as i32, (-a.sin()).round() as i32);
            if let Some(k) = crate::detail::search::HEADINGS
                .iter()
                .position(|&d| d == (dx, dy))
            {
                out |= 1 << k;
            }
        }
    }
    out
}

/// The Chebyshev radius, in nodes, a via site of `profile` on layer `slot`
/// must keep free of HALO: the via's reach past a track's half width, plus
/// a node's half-diagonal.
fn via_reach_nodes(profile: &super::Profile, slot: usize, pitch: Coord) -> usize {
    let rv = profile.via_diameter / 2 + profile.via_diameter % 2;
    let hw = profile.widths[slot] / 2 + profile.widths[slot] % 2;
    let rho = (rv - hw).max(0) + half_diag(pitch);
    ((rho + pitch - 1) / pitch) as usize
}

/// Recompute `via_site` over the node range from the live counters: a node
/// is a site (`VIA_CHECK`) when its CORE is clear and no node within
/// Chebyshev radius `k` carries HALO; nodes whose square leaves the window
/// are refused, as [`erode_into`] refuses them.
fn rederive_sites(r: &mut TileRaster, range: (u32, u32, u32, u32), k: usize) {
    let (c0, r0, c1, r1) = range;
    let w = r.window;
    let (cols, rows, k) = (i64::from(w.cols), i64::from(w.rows), k as i64);
    let x0 = (i64::from(c0) - k).max(0);
    let x1 = (i64::from(c1) + k).min(cols - 1);
    let y0 = (i64::from(r0) - k).max(0);
    let y1 = (i64::from(r1) + k).min(rows - 1);
    let pw = (x1 - x0 + 2) as usize;
    let ph = (y1 - y0 + 2) as usize;
    // pre[(y + 1) * pw + (x + 1)]: HALO nodes in [x0, x] x [y0, y].
    let mut pre = vec![0u32; pw * ph];
    for y in y0..=y1 {
        let yi = (y - y0 + 1) as usize;
        for x in x0..=x1 {
            let xi = (x - x0 + 1) as usize;
            let v = u32::from(r.halo.get(x as u32, y as u32) > 0);
            pre[yi * pw + xi] =
                v + pre[yi * pw + xi - 1] + pre[(yi - 1) * pw + xi] - pre[(yi - 1) * pw + xi - 1];
        }
    }
    let sum = |xa: i64, ya: i64, xb: i64, yb: i64| {
        let (xa, xb) = ((xa - x0) as usize, (xb - x0 + 1) as usize);
        let (ya, yb) = ((ya - y0) as usize, (yb - y0 + 1) as usize);
        pre[yb * pw + xb] + pre[ya * pw + xa] - pre[ya * pw + xb] - pre[yb * pw + xa]
    };
    for row in r0..=r1 {
        let y = i64::from(row);
        for col in c0..=c1 {
            let x = i64::from(col);
            let inside = x - k >= 0 && x + k < cols && y - k >= 0 && y + k < rows;
            let ok = inside && r.core.get(col, row) == 0 && sum(x - k, y - k, x + k, y + k) == 0;
            let site = if ok {
                crate::detail::search::VIA_CHECK
            } else {
                0
            };
            r.via_site.set(col, row, site);
        }
    }
}

impl TileRun {
    /// Routed copper whose box meets this is painted into the tile.
    pub fn reach(ctx: &Ctx<'_>, spec: &TileSpec) -> konnect_geom::BBox {
        spec.tile.window.inflate(ctx.margin + ctx.obstacles.reach)
    }

    /// Build the tile's hard state: one stack per profile its jobs need,
    /// with CORE/HALO from fixed copper and the commit-zone wall, copied
    /// from `cache` when another build already computed it. Stacks built
    /// fresh still need [`TileRun::derive_masks`] and
    /// [`TileRun::narrow_vias`]; then [`TileRun::paint_context`] adds
    /// history and routed copper. Every build computes the same planes
    /// whether or not the cache hits.
    pub fn build(
        ctx: &Ctx<'_>,
        spec: &TileSpec,
        jobs: Vec<PieceJob>,
        tasks: &[Task],
        cache: &RasterCache,
        pool: &mut RasterPool,
    ) -> TileRun {
        let t_build = std::time::Instant::now();
        let lattice = spec.lattice;
        let mut profiles: Vec<u16> = jobs
            .iter()
            .map(|j| tasks[j.task as usize].profile)
            .collect();
        profiles.sort_unstable();
        profiles.dedup();
        let mut telemetry = Telemetry::default();
        let stat = cache.tile_static(ctx, &lattice);

        let mut stacks = Vec::with_capacity(profiles.len());
        let mut fresh = Vec::with_capacity(profiles.len());
        let (mut us_hard, mut us_wall, mut us_copy) = (0u64, 0u64, 0u64);
        for &p in &profiles {
            let profile = &ctx.profiles[p as usize];
            let pid = ProfileId(p);
            let cached = cache.hard(&lattice, p);
            let mut layers = Vec::with_capacity(ctx.slots.len());
            for (slot, &layer) in ctx.slots.iter().enumerate() {
                let t = std::time::Instant::now();
                let (mut r, recycled) = pool.take(spec.tile.id, layer, pid, lattice);
                if recycled {
                    r.present.fill(0);
                }
                if let Some(c) = &cached {
                    c[slot].restore(&mut r);
                    us_copy += t.elapsed().as_micros() as u64;
                } else {
                    if recycled {
                        r.core.fill(0);
                        r.halo.fill(0);
                        r.heading_mask.fill(0);
                        r.via_site.fill(0);
                    }
                    let geom = ctx.geometry(p, slot);
                    let rep = RuleSubject::new(ItemKind::Track, Some(profile.rep));
                    for &id in &stat.per_slot[slot] {
                        let o = &ctx.obstacles.items[id as usize];
                        let track = rep.with_courtyards(&o.courtyards);
                        if let Some(c) = ctx.obstacles.track_clearance(ctx.rules, o, &track, layer)
                        {
                            CpuCompute::paint_shape(
                                &mut r,
                                &o.shape,
                                c,
                                PaintKind::Hard,
                                &geom,
                                true,
                            );
                        }
                    }
                    let t1 = std::time::Instant::now();
                    us_hard += (t1 - t).as_micros() as u64;
                    wall_off(&mut r, &spec.commit);
                    us_wall += t1.elapsed().as_micros() as u64;
                }
                telemetry.window_nodes.record(lattice.node_count() as u64);
                layers.push(r);
            }
            fresh.push(cached.is_none());
            stacks.push(Some(TileStack {
                tile: spec.tile.id,
                profile: pid,
                window: lattice,
                layers,
            }));
        }
        telemetry.count("us_build_hard", us_hard);
        telemetry.count("us_build_wall", us_wall);
        telemetry.count("us_build_copy", us_copy);
        telemetry.count("us_build", t_build.elapsed().as_micros() as u64);
        telemetry.count("tiles_built", 1);
        TileRun {
            id: spec.tile.id.0,
            phase: if spec.tile.phase == 0 { "bga" } else { "grid" },
            lattice,
            profiles,
            stacks,
            fresh,
            stat,
            soft: BTreeMap::new(),
            queue: jobs.into(),
            active: None,
            done: Vec::new(),
            telemetry,
            busy_us: 0,
            hard_routed: false,
            saved_sites: Vec::new(),
            stubs: Vec::new(),
        }
    }

    /// Let the backend derive heading and via masks from the hard counters
    /// of every freshly built stack (cached stacks carry theirs).
    pub fn derive_masks(&mut self, backend: &mut dyn RouteCompute) {
        let t = std::time::Instant::now();
        for (s, &fresh) in self.stacks.iter_mut().zip(&self.fresh) {
            if fresh {
                for r in &mut s.as_mut().expect("stack at home").layers {
                    backend.derive_masks(r);
                }
            }
        }
        self.telemetry
            .count("us_build_masks", t.elapsed().as_micros() as u64);
    }

    /// Narrow `via_site` of every freshly built stack to where the
    /// profile's whole via fits — every node within the via's reach must be
    /// clear of HALO (a square erosion, which over-covers the disc), and no
    /// via keepout may touch it — then offer the finished hard state to the
    /// cache.
    pub fn narrow_vias(&mut self, ctx: &Ctx<'_>, cache: &RasterCache) {
        let t = std::time::Instant::now();
        let w = self.lattice;
        for (k, s) in self.stacks.iter_mut().enumerate() {
            if !self.fresh[k] {
                continue;
            }
            let s = s.as_mut().expect("stacks are home between rounds");
            let profile = &ctx.profiles[self.profiles[k] as usize];
            let rv = profile.via_diameter / 2 + profile.via_diameter % 2;
            for (slot, r) in s.layers.iter_mut().enumerate() {
                let reach = via_reach_nodes(profile, slot, w.pitch_nm);
                erode_into(r, reach);
                for id in ctx.obstacles.via_keepouts(r.layer, &w.bbox().inflate(rv)) {
                    let o = &ctx.obstacles.items[id as usize];
                    // Only nodes whose disc can reach the keepout's box.
                    let b = o.shape.bbox().inflate(rv.saturating_add(w.pitch_nm));
                    let Some((c0, r0, c1, r1)) = node_range(&w, &b) else {
                        continue;
                    };
                    for row in r0..=r1 {
                        for col in c0..=c1 {
                            if r.via_site.get(col, row) != 0 {
                                let disc = Shape::Circle(Circle::new(w.node(col, row), rv));
                                if gap_cmp(&o.shape, &disc, 1) == Ordering::Less {
                                    r.via_site.set(col, row, 0);
                                }
                            }
                        }
                    }
                }
            }
            cache.offer(&w, self.profiles[k], || {
                s.layers.iter().map(HardLayer::capture).collect()
            });
            self.fresh[k] = false;
        }
        self.telemetry
            .count("us_build_narrow", t.elapsed().as_micros() as u64);
    }

    /// Add the soft state: history under the window (read once per layer,
    /// copied to every profile) and every routed piece within reach.
    pub fn paint_context(
        &mut self,
        ctx: &Ctx<'_>,
        spec: &TileSpec,
        store: &[Arc<PieceCopper>],
        history: &HistoryMap,
        fill: bool,
    ) {
        let t = std::time::Instant::now();
        self.hard_routed = fill;
        let lattice = self.lattice;
        if let Some((first, rest)) = self.stacks.split_first_mut() {
            let first = first.as_mut().expect("stack at home");
            for (slot, &layer) in ctx.slots.iter().enumerate() {
                let h = &mut first.layers[slot].history;
                h.fill(0);
                history.fill(layer, &lattice, h);
            }
            for s in rest {
                let s = s.as_mut().expect("stack at home");
                for (slot, r) in s.layers.iter_mut().enumerate() {
                    r.history
                        .as_mut_slice()
                        .copy_from_slice(first.layers[slot].history.as_slice());
                }
            }
        }
        let t_soft = std::time::Instant::now();
        self.telemetry
            .count("us_build_history", (t_soft - t).as_micros() as u64);
        let reach = Self::reach(ctx, spec);
        for piece in store {
            if piece.bbox.is_some_and(|b| b.intersects(&reach)) {
                for k in 0..self.stacks.len() {
                    self.paint_soft(ctx, k, piece, true);
                }
                self.soft.entry(piece.net).or_default().push(piece.clone());
            }
        }
        if fill {
            // Routed copper is a wall now: no via may sit within it either.
            let pitch = self.lattice.pitch_nm;
            for (k, s) in self.stacks.iter_mut().enumerate() {
                let profile = &ctx.profiles[self.profiles[k] as usize];
                let s = s.as_mut().expect("stack at home");
                for (slot, r) in s.layers.iter_mut().enumerate() {
                    erode_into(r, via_reach_nodes(profile, slot, pitch));
                }
            }
        }
        self.telemetry
            .count("us_build_soft", t_soft.elapsed().as_micros() as u64);
    }

    /// Hand the rasters back for reuse.
    pub fn recycle(&mut self, pool: &mut RasterPool) {
        for s in self.stacks.iter_mut().filter_map(Option::take) {
            for r in s.layers {
                pool.give(r);
            }
        }
    }

    pub fn take_stack(&mut self, k: usize) -> TileStack {
        self.stacks[k]
            .take()
            .expect("a stack is lent once per round")
    }

    pub fn put_stack(&mut self, k: usize, s: TileStack) {
        self.stacks[k] = Some(s);
    }

    fn paint_soft(&mut self, ctx: &Ctx<'_>, k: usize, piece: &PieceCopper, up: bool) {
        let p = self.profiles[k];
        let rep = ctx.profiles[p as usize].rep;
        let paint = if self.hard_routed {
            PaintKind::Hard
        } else {
            PaintKind::Soft
        };
        let stack = self.stacks[k].as_mut().expect("stack at home");
        for (slot, &layer) in ctx.slots.iter().enumerate() {
            let geom = ctx.geometry(p, slot);
            let r = &mut stack.layers[slot];
            for (shape, kind, cy) in piece.items_on(layer) {
                let me = RuleSubject::new(ItemKind::Track, Some(rep)).with_courtyards(cy);
                let c = ctx.rules.clearance(&piece.subject(kind, cy), &me, layer);
                CpuCompute::paint_shape(r, &shape, c, paint, &geom, up);
            }
            // A via's hole, where `hole_clearance` reaches past the copper
            // clearance of its disc.
            let hc = ctx.rules.hole_clearance();
            for v in &piece.vias {
                let me =
                    RuleSubject::new(ItemKind::Track, Some(rep)).with_courtyards(&v.courtyards);
                let c =
                    ctx.rules
                        .clearance(&piece.subject(ItemKind::Via, &v.courtyards), &me, layer);
                if v.drill / 2 + hc > v.diameter / 2 + c {
                    let hole = Shape::Circle(v.hole());
                    CpuCompute::paint_shape(r, &hole, hc, paint, &geom, up);
                }
            }
        }
    }

    /// Unpaint (`up == false`) or repaint `net`'s own copper in stack `k`.
    fn lift(&mut self, ctx: &Ctx<'_>, net: NetId, k: usize, up: bool) {
        if up {
            self.restore_sites();
        }
        self.paint_own(ctx, net, k, up);
        if !up {
            self.reopen_sites(ctx, net, k);
        }
    }

    /// Put back the via sites [`Self::reopen_sites`] overwrote.
    fn restore_sites(&mut self) {
        for saved in std::mem::take(&mut self.saved_sites) {
            let stack = self.stacks[saved.stack].as_mut().expect("stack at home");
            let r = &mut stack.layers[saved.slot];
            let (c0, r0, c1, r1) = saved.range;
            let mut it = saved.values.into_iter();
            for row in r0..=r1 {
                for col in c0..=c1 {
                    r.via_site
                        .set(col, row, it.next().expect("one value per node"));
                }
            }
        }
    }

    /// Re-derive via sites around `net`'s own copper once it is lifted from
    /// stack `k`. Sites are narrowed with every net's fixed copper painted
    /// (the cached hard state), so without this a net could never drop a
    /// via beside its own pads. A reopened site still keeps the whole via
    /// clear of every other net's HALO and of via keepouts, and never lets
    /// the via overlap one of the net's own pads (no via-in-pad).
    fn reopen_sites(&mut self, ctx: &Ctx<'_>, net: NetId, k: usize) {
        let w = self.lattice;
        let mut region: Option<konnect_geom::BBox> = None;
        let mut own_pads: Vec<u32> = Vec::new();
        if let Some(list) = self.stat.hard_own.get(&net) {
            for &(_, id) in list {
                let o = &ctx.obstacles.items[id as usize];
                let b = o.shape.bbox();
                region = Some(region.map_or(b, |r| r.union(b)));
                if o.kind == ObstacleKind::Copper(ItemKind::Pad) {
                    own_pads.push(id);
                }
            }
        }
        if self.hard_routed {
            for piece in self.soft.get(&net).into_iter().flatten() {
                if let Some(b) = piece.bbox {
                    region = Some(region.map_or(b, |r| r.union(b)));
                }
            }
        }
        let Some(region) = region else {
            return;
        };
        own_pads.sort_unstable();
        own_pads.dedup();
        let profile = &ctx.profiles[self.profiles[k] as usize];
        let rv = profile.via_diameter / 2 + profile.via_diameter % 2;
        let grow = ctx
            .obstacles
            .reach
            .saturating_add(ctx.max_half_width)
            .saturating_add(rv)
            .saturating_add(2 * w.pitch_nm);
        let Some(range) = node_range(&w, &region.inflate(grow)) else {
            return;
        };
        let (c0, r0, c1, r1) = range;
        let span = konnect_geom::BBox::new(w.node(c0, r0), w.node(c1, r1));
        let stack = self.stacks[k].as_mut().expect("stack at home");
        for (slot, r) in stack.layers.iter_mut().enumerate() {
            let mut values = Vec::with_capacity(((c1 - c0 + 1) * (r1 - r0 + 1)) as usize);
            for row in r0..=r1 {
                for col in c0..=c1 {
                    values.push(r.via_site.get(col, row));
                }
            }
            self.saved_sites.push(SavedSites {
                stack: k,
                slot,
                range,
                values,
            });
            rederive_sites(r, range, via_reach_nodes(profile, slot, w.pitch_nm));
            let mut refuse = |shape: &Shape| {
                let b = shape.bbox().inflate(rv.saturating_add(w.pitch_nm));
                let Some((a0, b0, a1, b1)) = node_range(&w, &b) else {
                    return;
                };
                for row in b0.max(r0)..=b1.min(r1) {
                    for col in a0.max(c0)..=a1.min(c1) {
                        if r.via_site.get(col, row) != 0 {
                            let disc = Shape::Circle(Circle::new(w.node(col, row), rv));
                            if gap_cmp(shape, &disc, 1) == Ordering::Less {
                                r.via_site.set(col, row, 0);
                            }
                        }
                    }
                }
            };
            for id in ctx.obstacles.via_keepouts(r.layer, &span.inflate(rv)) {
                refuse(&ctx.obstacles.items[id as usize].shape);
            }
            for &id in &own_pads {
                let o = &ctx.obstacles.items[id as usize];
                if o.layers.contains(r.layer) {
                    refuse(&o.shape);
                }
            }
        }
    }

    /// Paint (`up`) or unpaint `net`'s own fixed and routed copper in stack
    /// `k`.
    fn paint_own(&mut self, ctx: &Ctx<'_>, net: NetId, k: usize, up: bool) {
        let p = self.profiles[k];
        let rep = RuleSubject::new(ItemKind::Track, Some(ctx.profiles[p as usize].rep));
        if let Some(list) = self.stat.hard_own.get(&net) {
            let stack = self.stacks[k].as_mut().expect("stack at home");
            for &(slot, id) in list {
                let o = &ctx.obstacles.items[id as usize];
                let layer = ctx.slots[slot as usize];
                let geom = ctx.geometry(p, slot as usize);
                let track = rep.with_courtyards(&o.courtyards);
                if let Some(c) = ctx.obstacles.track_clearance(ctx.rules, o, &track, layer) {
                    CpuCompute::paint_shape(
                        &mut stack.layers[slot as usize],
                        &o.shape,
                        c,
                        PaintKind::Hard,
                        &geom,
                        up,
                    );
                }
            }
        }
        if let Some(list) = self.soft.get(&net).cloned() {
            for piece in &list {
                self.paint_soft(ctx, k, piece, up);
            }
        }
    }

    /// Lattice nodes a terminal offers in stack `k`, or empty.
    fn terminal_nodes(
        &mut self,
        ctx: &Ctx<'_>,
        task: &Task,
        t: &Terminal,
        k: usize,
        source: bool,
    ) -> Vec<JobNode> {
        let stack = self.stacks[k].as_ref().expect("stack at home");
        let w = self.lattice;
        let node = |c: u32, r: u32, slot: usize, mask: u8| JobNode {
            col: c as u16,
            row: r as u16,
            layer: slot as u8,
            heading_mask: mask,
            reserved: 0,
        };
        let open = |slot: usize, c: u32, r: u32| stack.layers[slot].core.get(c, r) == 0;
        let mut out = Vec::new();
        match t {
            Terminal::Pin(pin) => {
                if let (Some(slot), Some((c, r))) = (ctx.slot_of(pin.layer), w.nearest(pin.at)) {
                    if w.node(c, r) == pin.at && open(slot, c, r) {
                        out.push(node(c, r, slot, pin.headings));
                    }
                }
            }
            Terminal::Item(end) => {
                // Every pad, via and track end of the end's fixed island.
                let island = if *end == task.conn.from {
                    &task.from_island
                } else {
                    &task.to_island
                };
                for item in island {
                    self.item_nodes(ctx, task, *item, k, source, &mut out);
                }
            }
        }
        out
    }

    /// Open lattice nodes on one terminal item, appended to `out`.
    fn item_nodes(
        &mut self,
        ctx: &Ctx<'_>,
        task: &Task,
        item: ItemRef,
        k: usize,
        source: bool,
        out: &mut Vec<JobNode>,
    ) {
        let mut stub_slots: Vec<usize> = Vec::new();
        let stack = self.stacks[k].as_ref().expect("stack at home");
        let w = self.lattice;
        let node = |c: u32, r: u32, slot: usize| JobNode {
            col: c as u16,
            row: r as u16,
            layer: slot as u8,
            heading_mask: 0xff,
            reserved: 0,
        };
        let open = |slot: usize, c: u32, r: u32| stack.layers[slot].core.get(c, r) == 0;
        {
            {
                let (shapes, layers, points): (Vec<&Shape>, _, Vec<Point>) = match item {
                    ItemRef::Pad(id) => {
                        let pad = &ctx.board.pads[id.index()];
                        (pad.shapes.iter().collect(), pad.layers, vec![pad.center])
                    }
                    ItemRef::Via(id) => {
                        let v = &ctx.board.vias[id.index()];
                        (Vec::new(), v.layers, vec![v.center])
                    }
                    ItemRef::Track(id) => {
                        let t = &ctx.board.tracks[id.index()];
                        (
                            Vec::new(),
                            crate::api::LayerMask::single(t.layer),
                            vec![t.seg.a, t.seg.b],
                        )
                    }
                    _ => return,
                };
                let centre = points[0];
                for (slot, &layer) in ctx.slots.iter().enumerate() {
                    if !layers.contains(layer) || !task.layers.contains(layer) {
                        continue;
                    }
                    let mut cand: Vec<(i128, u32, u32)> = Vec::new();
                    if shapes.is_empty() {
                        // A via or track end: the node nearest each point, if
                        // it is on the copper (within half a pitch it is).
                        for p in &points {
                            if let Some((c, r)) = w.nearest(*p) {
                                if open(slot, c, r) {
                                    cand.push(((w.node(c, r) - *p).length2(), r, c));
                                }
                            }
                        }
                    }
                    for shape in &shapes {
                        let b = shape.bbox();
                        let span = w.bbox();
                        if !b.intersects(&span) {
                            continue;
                        }
                        let p = i64::from(w.pitch_nm);
                        let lo = |v: i32, o: i32| ((i64::from(v) - i64::from(o)).max(0) / p) as u32;
                        let (c0, r0) = (lo(b.min.x, w.origin.x), lo(b.min.y, w.origin.y));
                        let c1 = (lo(b.max.x, w.origin.x) + 1).min(w.cols - 1);
                        let r1 = (lo(b.max.y, w.origin.y) + 1).min(w.rows - 1);
                        for r in r0..=r1 {
                            for c in c0..=c1 {
                                let q = w.node(c, r);
                                if open(slot, c, r)
                                    && touches(shape, &Shape::Circle(Circle::new(q, 0)))
                                {
                                    cand.push(((q - centre).length2(), r, c));
                                }
                            }
                        }
                    }
                    cand.sort_unstable();
                    cand.dedup();
                    if cand.is_empty() && !shapes.is_empty() {
                        stub_slots.push(slot);
                    }
                    for &(_, r, c) in cand.iter().take(ctx.tuning.max_terminal_nodes as usize) {
                        out.push(node(c, r, slot));
                    }
                }
            }
        }
        if let ItemRef::Pad(pad) = item {
            for slot in stub_slots {
                self.stub_nodes(ctx, task, pad, slot, k, source, out);
            }
        }
    }

    /// Terminal nodes for a pad with no open node of its own on `slot`: from
    /// each S4 access point on the layer (else the pad's anchor), along each
    /// open exit heading, the first open node a stub track reaches that
    /// passes the exact check. The stub is the class's width when that
    /// works within [`NegotiationTuning::stub_reach`], else the neck-down
    /// width within [`super::NeckDown::reach`] when the class may neck down.
    /// The search must leave a stub's node within 45° of the stub, so the
    /// joint is never sharper than 135°.
    #[allow(clippy::too_many_arguments)]
    fn stub_nodes(
        &mut self,
        ctx: &Ctx<'_>,
        task: &Task,
        pad: crate::api::PadId,
        slot: usize,
        k: usize,
        source: bool,
        out: &mut Vec<JobNode>,
    ) {
        let w = self.lattice;
        let layer = ctx.slots[slot];
        let net = task.conn.net;
        let class = ctx.rules.class_of(Some(net));
        let stack = self.stacks[k].as_ref().expect("stack at home");
        let core = &stack.layers[slot].core;
        let full = ctx.width(net, slot);
        let mut widths = vec![(full, ctx.tuning.stub_reach)];
        if let Some(n) = ctx.neck.as_ref().filter(|n| !n.excluded.contains(&class)) {
            let min = ctx
                .rules
                .track_width(&RuleSubject::new(ItemKind::Track, Some(net)), layer)
                .min;
            let neck = n.width.max(min);
            if neck < full {
                widths.push((neck, n.reach));
            }
        }
        let mut origins: Vec<(Point, u8)> = ctx
            .access
            .get(&(pad, class))
            .into_iter()
            .flatten()
            .filter(|a| a.layer == layer && a.headings != 0)
            .map(|a| (a.point, access_to_kernel(a.headings)))
            .collect();
        let anchor = ctx.board.pads[pad.index()].center;
        if !origins.iter().any(|(p, _)| *p == anchor) {
            origins.push((anchor, 0xff));
        }
        let pitch = i64::from(w.pitch_nm);
        // A stub is a segment of its own: never shorter than the minimum.
        let min_len = ctx
            .rules
            .min_segment_length(&RuleSubject::new(ItemKind::Track, Some(net)), layer)
            .unwrap_or(0);
        let mut found: Vec<(i128, u32, u32, u8, Point, Coord)> = Vec::new();
        for &(width, reach) in &widths {
            for &(o, headings) in &origins {
                for h in 0..8u8 {
                    if headings & (1 << h) == 0 {
                        continue;
                    }
                    let (dx, dy) = crate::detail::search::HEADINGS[h as usize];
                    let diag = dx != 0 && dy != 0;
                    let mut t = (2 * pitch).max(i64::from(min_len));
                    while t <= i64::from(reach) {
                        let d = if diag {
                            (t as f64 * std::f64::consts::FRAC_1_SQRT_2).round() as i64
                        } else {
                            t
                        };
                        let e = Point::new(
                            (i64::from(o.x) + i64::from(dx) * d) as Coord,
                            (i64::from(o.y) + i64::from(dy) * d) as Coord,
                        );
                        t += pitch;
                        let Some((c, r)) = w.nearest(e) else {
                            break;
                        };
                        if core.get(c, r) != 0 {
                            continue;
                        }
                        let q = w.node(c, r);
                        let len2 = (q - o).length2();
                        if len2 > i128::from(reach) * i128::from(reach) {
                            break;
                        }
                        if len2 < i128::from(min_len) * i128::from(min_len) {
                            continue;
                        }
                        let seg = Segment::new(o, q, width);
                        let cy = ctx.obstacles.courtyards.hits(&Shape::Segment(seg));
                        // A courtyard-scoped rule may want more width here.
                        let floor = ctx
                            .rules
                            .track_width(
                                &RuleSubject::new(ItemKind::Track, Some(net)).with_courtyards(&cy),
                                layer,
                            )
                            .min;
                        if width >= floor
                            && ctx
                                .obstacles
                                .check_segment(ctx.rules, net, layer, &seg, &cy)
                                .is_none()
                        {
                            found.push((len2, c, r, h, o, width));
                        }
                        break;
                    }
                }
            }
            if !found.is_empty() {
                break;
            }
        }
        if found.is_empty() {
            return;
        }
        self.telemetry.count("stub_terminals", 1);
        if found.iter().any(|f| f.5 < full) {
            self.telemetry.count("stub_necked", 1);
        }
        found.sort_unstable_by_key(|f| (f.0, f.1, f.2, f.3));
        let mut taken = 0;
        for (_, c, r, h, o, width) in found {
            if taken >= ctx.tuning.max_terminal_nodes as usize {
                break;
            }
            if self
                .stubs
                .iter()
                .any(|s| (s.col, s.row, s.slot) == (c as u16, r as u16, slot as u8))
            {
                continue;
            }
            // Leaving (source) or arriving (target) within 45 degrees of the
            // stub's direction away from the pad.
            let along = if source { h } else { (h + 4) % 8 };
            let mask = (1u8 << along) | (1u8 << ((along + 1) % 8)) | (1u8 << ((along + 7) % 8));
            out.push(JobNode {
                col: c as u16,
                row: r as u16,
                layer: slot as u8,
                heading_mask: mask,
                reserved: 0,
            });
            self.stubs.push(Stub {
                col: c as u16,
                row: r as u16,
                slot: slot as u8,
                from: o,
                width,
            });
            taken += 1;
        }
    }

    /// Pop pieces until one is ready to search; pieces whose terminals have
    /// no open node fail on the spot.
    pub fn prepare(&mut self, ctx: &Ctx<'_>, tasks: &[Task], iteration: u32) -> Option<Prepared> {
        loop {
            let job = self.queue.pop_front()?;
            let task = &tasks[job.task as usize];
            let k = self
                .profiles
                .binary_search(&task.profile)
                .expect("every queued task's profile has a stack");
            let net = task.conn.net;
            self.lift(ctx, net, k, false);
            self.stubs.clear();
            let sources = self.terminal_nodes(ctx, task, &job.from, k, true);
            let targets = self.terminal_nodes(ctx, task, &job.to, k, false);
            if sources.is_empty() || targets.is_empty() {
                self.lift(ctx, net, k, true);
                let end = if sources.is_empty() { job.from } else { job.to };
                let failure = match end {
                    Terminal::Item(item) => DetailFailure::NoAccess { item },
                    Terminal::Pin(_) => DetailFailure::NoPin {
                        from: crate::api::TileId(self.id),
                        to: crate::api::TileId(self.id),
                    },
                };
                self.done.push(PieceDone {
                    task: job.task,
                    piece: job.piece,
                    result: Err(failure),
                });
                continue;
            }
            let size_class = (self.lattice.node_count().max(1) as f64).log2().ceil() as u8;
            let mut flags = 0;
            if task.layers.count() > 1 {
                flags |= JOB_ALLOW_VIA;
            }
            if iteration > 0 || job.retries > 0 {
                flags |= JOB_REROUTE;
            }
            let jd = JobDescriptor {
                connection: task.conn.id.0,
                net: net.0,
                window: 0,
                profile: task.profile,
                size_class,
                flags,
                layer_mask: task.layers.0,
                cost_bound: u32::MAX,
                order_key: (job.task << 8) | u32::from(job.piece),
                iteration,
                ..JobDescriptor::default()
            };
            self.active = Some((job, k));
            return Some(Prepared {
                stack: k,
                job: jd,
                sources,
                targets,
            });
        }
    }

    /// Commit or reject the active piece's search outcome.
    pub fn finish(&mut self, ctx: &Ctx<'_>, tasks: &[Task], outcome: SearchOutcome) {
        let (mut job, k) = self.active.take().expect("finish follows prepare");
        let task = &tasks[job.task as usize];
        let net = task.conn.net;
        self.telemetry.count("searches", 1);
        let (expanded, kind) = match &outcome {
            SearchOutcome::Found(p) => (p.expansions, "found"),
            SearchOutcome::Unreachable { expansions, .. } => (*expansions, "unreachable"),
            SearchOutcome::BudgetExhausted { expansions, .. } => (*expansions, "budget"),
        };
        let bga = self.phase == "bga";
        let (exp_key, kind_key) = match (kind, bga) {
            ("found", true) => ("expansions_bga", "search_found_bga"),
            ("found", false) => ("expansions_grid", "search_found_grid"),
            ("unreachable", true) => ("expansions_bga", "search_unreachable_bga"),
            ("unreachable", false) => ("expansions_grid", "search_unreachable_grid"),
            (_, true) => ("expansions_bga", "search_budget_bga"),
            (_, false) => ("expansions_grid", "search_budget_grid"),
        };
        self.telemetry.count(exp_key, expanded);
        self.telemetry.count(kind_key, 1);
        let fail = |run: &mut TileRun, job: &PieceJob, f: DetailFailure| {
            run.done.push(PieceDone {
                task: job.task,
                piece: job.piece,
                result: Err(f),
            });
        };
        match outcome {
            SearchOutcome::Found(p) if p.nodes.is_empty() => {
                // A backend contract violation; never commit it.
                self.telemetry.count("empty_paths", 1);
                self.lift(ctx, net, k, true);
                fail(self, &job, DetailFailure::Unreachable);
            }
            SearchOutcome::Found(p) => {
                self.telemetry
                    .expansions_per_connection
                    .record(p.expansions);
                self.telemetry.count("expansions", p.expansions);
                let (piece, steps) = self.geometry(ctx, task, job.task, job.piece, &p.nodes);
                let vetoes = self.exact_check(ctx, net, &piece, &steps, &p.nodes, k);
                if vetoes == 0 {
                    let piece = Arc::new(self.with_stubs(ctx, piece, &p.nodes));
                    for other in 0..self.stacks.len() {
                        if other != k {
                            self.paint_soft(ctx, other, &piece, true);
                        }
                    }
                    self.soft.entry(net).or_default().push(piece.clone());
                    self.lift(ctx, net, k, true);
                    self.done.push(PieceDone {
                        task: job.task,
                        piece: job.piece,
                        result: Ok(piece),
                    });
                } else {
                    self.telemetry.count("exact_rejects", 1);
                    self.telemetry.count("vetoes", vetoes);
                    self.lift(ctx, net, k, true);
                    job.retries += 1;
                    if job.retries > ctx.tuning.max_commit_retries {
                        fail(self, &job, DetailFailure::ExactCheckRejected);
                    } else {
                        self.queue.push_front(job);
                    }
                }
            }
            SearchOutcome::Unreachable { expansions, .. } => {
                self.telemetry.expansions_per_connection.record(expansions);
                self.telemetry.count("expansions", expansions);
                self.lift(ctx, net, k, true);
                fail(self, &job, DetailFailure::Unreachable);
            }
            SearchOutcome::BudgetExhausted { expansions, .. } => {
                self.telemetry.expansions_per_connection.record(expansions);
                self.telemetry.count("expansions", expansions);
                self.lift(ctx, net, k, true);
                fail(self, &job, DetailFailure::BudgetExhausted);
            }
        }
    }

    /// `piece` with the stub tracks of the path's end nodes, if they are
    /// stub terminals: the source's first, the target's last.
    fn with_stubs(&self, ctx: &Ctx<'_>, piece: PieceCopper, nodes: &[JobNode]) -> PieceCopper {
        if self.stubs.is_empty() {
            return piece;
        }
        let w = self.lattice;
        let stub_at = |n: &JobNode| {
            self.stubs
                .iter()
                .find(|s| (s.col, s.row, s.slot) == (n.col, n.row, n.layer))
                .copied()
        };
        let seg_of = |s: Stub| {
            let q = w.node(u32::from(s.col), u32::from(s.row));
            let seg = Segment::new(s.from, q, s.width);
            SegCopper {
                layer: ctx.slots[s.slot as usize],
                seg,
                courtyards: ctx.obstacles.courtyards.hits(&Shape::Segment(seg)),
            }
        };
        let (first, last) = match (nodes.first(), nodes.last()) {
            (Some(a), Some(b)) => (stub_at(a), stub_at(b)),
            _ => return piece,
        };
        if first.is_none() && last.is_none() {
            return piece;
        }
        let mut segs = Vec::with_capacity(piece.segs.len() + 2);
        segs.extend(first.map(seg_of));
        segs.extend(piece.segs);
        if nodes.len() > 1 || first.is_none() {
            segs.extend(last.map(seg_of));
        }
        PieceCopper::new(piece.task, piece.piece, piece.net, segs, piece.vias)
    }

    /// Lattice path to copper: straight runs merged into segments, layer
    /// changes into through vias. Also returns each segment's node range.
    fn geometry(
        &self,
        ctx: &Ctx<'_>,
        task: &Task,
        task_index: u32,
        piece: u8,
        nodes: &[JobNode],
    ) -> (PieceCopper, Vec<(usize, usize)>) {
        let w = self.lattice;
        let net = task.conn.net;
        let pt = |n: &JobNode| w.node(u32::from(n.col), u32::from(n.row));
        let mut segs = Vec::new();
        let mut steps = Vec::new();
        let mut vias = Vec::new();
        let emit =
            |s: usize, e: usize, segs: &mut Vec<SegCopper>, steps: &mut Vec<(usize, usize)>| {
                if e > s {
                    let slot = nodes[s].layer as usize;
                    let seg = Segment::new(pt(&nodes[s]), pt(&nodes[e]), ctx.width(net, slot));
                    let courtyards = ctx.obstacles.courtyards.hits(&Shape::Segment(seg));
                    segs.push(SegCopper {
                        layer: ctx.slots[slot],
                        seg,
                        courtyards,
                    });
                    steps.push((s, e));
                }
            };
        let via_rule = &ctx.net_via[net.index()];
        let mut start = 0;
        let mut dir: Option<(i32, i32)> = None;
        for i in 1..nodes.len() {
            let (a, b) = (&nodes[i - 1], &nodes[i]);
            if a.layer != b.layer {
                emit(start, i - 1, &mut segs, &mut steps);
                let center = pt(a);
                let disc = Circle::from_diameter(center, via_rule.diameter);
                vias.push(ViaCopper {
                    center,
                    diameter: via_rule.diameter,
                    drill: via_rule.drill,
                    courtyards: ctx.obstacles.courtyards.hits(&Shape::Circle(disc)),
                });
                start = i;
                dir = None;
                continue;
            }
            let d = (
                i32::from(b.col) - i32::from(a.col),
                i32::from(b.row) - i32::from(a.row),
            );
            if dir.is_some_and(|x| x != d) {
                emit(start, i - 1, &mut segs, &mut steps);
                start = i - 1;
            }
            dir = Some(d);
        }
        emit(start, nodes.len() - 1, &mut segs, &mut steps);
        (PieceCopper::new(task_index, piece, net, segs, vias), steps)
    }

    /// Exact check against fixed copper; on failure, veto the offending
    /// lattice steps and via sites in stack `k`. Returns the veto count.
    fn exact_check(
        &mut self,
        ctx: &Ctx<'_>,
        net: NetId,
        piece: &PieceCopper,
        steps: &[(usize, usize)],
        nodes: &[JobNode],
        k: usize,
    ) -> u64 {
        let w = self.lattice;
        let pt = |n: &JobNode| w.node(u32::from(n.col), u32::from(n.row));
        let mut vetoes = 0u64;
        let stack = self.stacks[k].as_mut().expect("stack at home");
        for (s, &(a, b)) in piece.segs.iter().zip(steps) {
            if ctx
                .obstacles
                .check_segment(ctx.rules, net, s.layer, &s.seg, &s.courtyards)
                .is_none()
            {
                continue;
            }
            for i in a..b {
                let step = Segment::new(pt(&nodes[i]), pt(&nodes[i + 1]), s.seg.width);
                let cy = ctx.obstacles.courtyards.hits(&Shape::Segment(step));
                if ctx
                    .obstacles
                    .check_segment(ctx.rules, net, s.layer, &step, &cy)
                    .is_none()
                {
                    continue;
                }
                let r = &mut stack.layers[nodes[i].layer as usize];
                let ends = [&nodes[i], &nodes[i + 1]];
                let near: Vec<&&JobNode> = ends
                    .iter()
                    .filter(|n| r.halo.get(u32::from(n.col), u32::from(n.row)) > 0)
                    .collect();
                let pick: Vec<&JobNode> = if near.is_empty() {
                    ends.to_vec()
                } else {
                    near.into_iter().copied().collect()
                };
                for n in pick {
                    let (c, rr) = (u32::from(n.col), u32::from(n.row));
                    let core = r.core.get_mut(c, rr);
                    *core = core.saturating_add(1);
                    let halo = r.halo.get_mut(c, rr);
                    *halo = halo.saturating_add(1);
                    vetoes += 1;
                }
            }
        }
        let h2h = i64::from(ctx.rules.hole_to_hole());
        for (i, v) in piece.vias.iter().enumerate() {
            // Two vias of one path closer than hole-to-hole: refuse the later.
            let crowded = piece.vias[..i].iter().any(|u| {
                gap_cmp(&Shape::Circle(u.hole()), &Shape::Circle(v.hole()), h2h) == Ordering::Less
            });
            if crowded
                || ctx
                    .obstacles
                    .check_via(ctx.rules, net, &v.disc(), &v.hole(), &v.courtyards)
                    .is_some()
            {
                if let Some((c, r)) = w.nearest(v.center) {
                    for layer in &mut stack.layers {
                        layer.via_site.set(c, r, 0);
                    }
                    // Sites saved while the net is lifted would put the
                    // site back on repaint: veto it there too.
                    for saved in self.saved_sites.iter_mut().filter(|s| s.stack == k) {
                        let (c0, r0, c1, r1) = saved.range;
                        if (c0..=c1).contains(&c) && (r0..=r1).contains(&r) {
                            saved.values[((r - r0) * (c1 - c0 + 1) + (c - c0)) as usize] = 0;
                        }
                    }
                    vetoes += 1;
                }
            }
        }
        vetoes
    }

    /// Route every queued piece in order on this tile's own backend copy:
    /// each search is a one-job batch.
    pub fn route_all(
        &mut self,
        ctx: &Ctx<'_>,
        tasks: &[Task],
        params: &CostParams,
        iteration: u32,
        backend: &mut dyn RouteCompute,
    ) {
        let start = std::time::Instant::now();
        let mut round = 0;
        while let Some(p) = self.prepare(ctx, tasks, iteration) {
            let mut pool = JobPool::default();
            pool.push(p.job, &p.sources, &p.targets, &[]);
            let stacks = [self.take_stack(p.stack)];
            let batch = Batch {
                index: round,
                jobs: vec![0],
            };
            let mut out = backend.search_batch(&batch, &pool, &stacks, params);
            let [stack] = stacks;
            self.put_stack(p.stack, stack);
            assert_eq!(out.len(), 1, "a backend returns one outcome per job");
            self.finish(ctx, tasks, out.pop().expect("one outcome"));
            round += 1;
        }
        self.telemetry.count("tile_rounds", u64::from(round));
        self.busy_us = start.elapsed().as_micros() as u64;
    }

    pub fn has_work(&self) -> bool {
        !self.queue.is_empty()
    }
}

/// The node range `(c0, r0, c1, r1)` (inclusive) whose positions lie in
/// `b`, or `None` when no node does.
pub(crate) fn node_range(w: &Window, b: &konnect_geom::BBox) -> Option<(u32, u32, u32, u32)> {
    let p = i64::from(w.pitch_nm);
    let first = |lo: Coord, o: Coord| (i64::from(lo) - i64::from(o) + p - 1).div_euclid(p).max(0);
    let last = |hi: Coord, o: Coord, n: u32| {
        (i64::from(hi) - i64::from(o))
            .div_euclid(p)
            .min(i64::from(n) - 1)
    };
    let (c0, c1) = (
        first(b.min.x, w.origin.x),
        last(b.max.x, w.origin.x, w.cols),
    );
    let (r0, r1) = (
        first(b.min.y, w.origin.y),
        last(b.max.y, w.origin.y, w.rows),
    );
    (c0 <= c1 && r0 <= r1).then_some((c0 as u32, r0 as u32, c1 as u32, r1 as u32))
}

/// Count one wall into CORE and HALO of every node outside `commit`: whole
/// rows above and below it, and the bands left and right of it.
fn wall_off(r: &mut TileRaster, commit: &konnect_geom::BBox) {
    let w = r.window;
    let inside = node_range(&w, commit);
    let stride = r.core.stride() as usize;
    let (cols, rows) = (w.cols as usize, w.rows as usize);
    let bump = |v: &mut u8| *v = v.saturating_add(1);
    for plane in [&mut r.core, &mut r.halo] {
        let data = plane.as_mut_slice();
        for row in 0..rows {
            let line = &mut data[row * stride..row * stride + cols];
            match inside {
                Some((c0, r0, c1, r1)) if (r0 as usize..=r1 as usize).contains(&row) => {
                    line[..c0 as usize].iter_mut().for_each(bump);
                    line[c1 as usize + 1..].iter_mut().for_each(bump);
                }
                _ => line.iter_mut().for_each(bump),
            }
        }
    }
}

/// `via_site &= (no HALO within Chebyshev radius k)`, by two separable
/// sliding-window passes (prefix sums along rows, a running column sum down
/// the rows). Nodes whose square leaves the window are refused (their
/// context is unknown).
fn erode_into(r: &mut TileRaster, k: usize) {
    let w = r.window;
    let (cols, rows) = (w.cols as usize, w.rows as usize);
    let stride = r.halo.stride() as usize;
    let halo = r.halo.as_slice();
    // horiz = some HALO within k columns (1 where the square leaves the row).
    let mut horiz = vec![1u8; cols * rows];
    let mut prefix = vec![0u32; cols + 1];
    for row in 0..rows {
        let line = &halo[row * stride..row * stride + cols];
        for (col, &h) in line.iter().enumerate() {
            prefix[col + 1] = prefix[col] + u32::from(h > 0);
        }
        let out = &mut horiz[row * cols..(row + 1) * cols];
        for col in k..cols.saturating_sub(k) {
            out[col] = u8::from(prefix[col + k + 1] - prefix[col - k] > 0);
        }
    }
    let vstride = r.via_site.stride() as usize;
    let via = r.via_site.as_mut_slice();
    let mut refuse = |row: usize| via[row * vstride..row * vstride + cols].fill(0);
    if rows < 2 * k + 1 {
        (0..rows).for_each(&mut refuse);
        return;
    }
    (0..k).chain(rows - k..rows).for_each(&mut refuse);
    // count[col] = sum of horiz over rows row-k ..= row+k.
    let mut count = vec![0u32; cols];
    for rr in 0..2 * k {
        for (c, &h) in count.iter_mut().zip(&horiz[rr * cols..(rr + 1) * cols]) {
            *c += u32::from(h);
        }
    }
    for row in k..rows - k {
        let add = row + k;
        for (c, &h) in count.iter_mut().zip(&horiz[add * cols..(add + 1) * cols]) {
            *c += u32::from(h);
        }
        let line = &mut via[row * vstride..row * vstride + cols];
        for (v, &c) in line.iter_mut().zip(&count) {
            if c > 0 {
                *v = 0;
            }
        }
        let sub = row - k;
        for (c, &h) in count.iter_mut().zip(&horiz[sub * cols..(sub + 1) * cols]) {
            *c -= u32::from(h);
        }
    }
}

#[cfg(test)]
pub(crate) fn erode_for_test(r: &mut TileRaster, k: usize) {
    erode_into(r, k)
}

#[cfg(test)]
pub(crate) fn rederive_for_test(r: &mut TileRaster, range: (u32, u32, u32, u32), k: usize) {
    rederive_sites(r, range, k)
}
