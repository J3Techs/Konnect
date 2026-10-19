//! Pattern routes for the seed iteration: L, Z, U-detour and 3-bend
//! shapes between two cells, each leg's layer chosen by dynamic programming
//! (FastRoute, FastGR and CUGR style).
//!
//! A layer change is one through via: it joins any two allowed slots at one
//! cost, and the path never visits a slot the connection may not use.

use super::grid::{Topology, Win};
use super::search::{Costs, Scratch};
use std::collections::BTreeMap;

/// One pattern request. `slots`, `src_slots` and `dst_slots` are slot
/// bitmasks.
pub(crate) struct PatternReq<'a> {
    pub src_cell: u32,
    pub dst_cell: u32,
    pub src_slots: u32,
    pub dst_slots: u32,
    pub slots: u32,
    pub win: Win,
    pub weight: u16,
    pub via_weight: u16,
    pub owned: &'a BTreeMap<u32, u16>,
    pub term_cells: &'a [u32],
}

/// Z bend positions tried per axis.
const Z_SAMPLES: usize = 16;
/// Detour offsets (base cells) tried beyond each side of the box.
const DETOURS: i64 = 3;
/// Bend positions per axis for the 3-bend staircases.
const STAIR_SAMPLES: usize = 3;

type Pt = (i64, i64);

/// Reusable buffers of [`best_pattern`], kept in the per-thread
/// [`Scratch`] so a seed allocates only its result.
#[derive(Default)]
pub(crate) struct PatternBufs {
    /// Candidate polylines, flat, with `cand_off` delimiting them.
    cands: Vec<Pt>,
    cand_off: Vec<usize>,
    cols: Vec<i64>,
    rows: Vec<i64>,
    dedup: Vec<Pt>,
    pts: Vec<Pt>,
    /// The `(cell, edge)` steps of every leg, flat, with `step_off`.
    steps: Vec<(u32, u32)>,
    step_off: Vec<usize>,
    ends: Vec<u32>,
    leg_cost: Vec<f32>,
    dp: Vec<(f32, usize)>,
    next: Vec<(f32, usize)>,
    /// `back[li · slots + slot]`: the slot of leg `li − 1` (for `li = 0`,
    /// the source slot) on the best way to `slot`.
    back: Vec<usize>,
    leg_slot: Vec<usize>,
    path: Vec<u32>,
    best: Vec<u32>,
}

/// Up to `max` evenly spaced integers strictly between `a` and `b`.
fn between(a: i64, b: i64, max: usize, out: &mut Vec<i64>) {
    out.clear();
    let (lo, hi) = (a.min(b), a.max(b));
    let n = (hi - lo - 1).max(0) as usize;
    if n <= max {
        out.extend(lo + 1..hi);
        return;
    }
    out.extend((0..max).map(|i| lo + 1 + ((i as i64 * 2 + 1) * n as i64) / (2 * max as i64)));
}

/// Walk from `start` (which holds `from`) along one axis to the cell
/// holding `to`, pushing `(cell, edge)` steps. `None` when the walk leaves
/// the window or the grid.
fn walk(
    t: &Topology,
    start: u32,
    from: Pt,
    to: Pt,
    win: &Win,
    out: &mut Vec<(u32, u32)>,
) -> Option<u32> {
    let horiz = from.1 == to.1;
    let mut c = start;
    loop {
        let i = c as usize;
        let (x0, y0, s) = (i64::from(t.x0[i]), i64::from(t.y0[i]), i64::from(t.size[i]));
        let next = if horiz {
            if to.0 >= x0 && to.0 < x0 + s {
                return Some(c);
            }
            if to.0 >= x0 + s {
                t.locate_xy(x0 + s, from.1)
            } else {
                t.locate_xy(x0 - 1, from.1)
            }
        } else {
            if to.1 >= y0 && to.1 < y0 + s {
                return Some(c);
            }
            if to.1 >= y0 + s {
                t.locate_xy(from.0, y0 + s)
            } else {
                t.locate_xy(from.0, y0 - 1)
            }
        }?;
        if !t.in_window(next, win) {
            return None;
        }
        let e = t.edge_between(c, next)?;
        out.push((next, e));
        c = next;
    }
}

/// Drop repeated points and merge collinear legs into `out`; false when a
/// leg doubles back on the previous one.
fn normalize(pts: &[Pt], tmp: &mut Vec<Pt>, out: &mut Vec<Pt>) -> bool {
    tmp.clear();
    for &p in pts {
        if tmp.last() != Some(&p) {
            tmp.push(p);
        }
    }
    out.clear();
    for &p in tmp.iter() {
        if out.len() >= 2 {
            let (a, b) = (out[out.len() - 2], out[out.len() - 1]);
            let same_axis = (a.0 == b.0 && b.0 == p.0) || (a.1 == b.1 && b.1 == p.1);
            if same_axis {
                let forward = (b.0 - a.0) * (p.0 - b.0) + (b.1 - a.1) * (p.1 - b.1);
                if forward < 0 {
                    return false;
                }
                out.pop();
            }
        }
        out.push(p);
    }
    true
}

/// The cost of one resource for the request, `None` when it is a wall.
#[inline]
fn res_cost(
    c: &Costs,
    s: &Scratch,
    ep: u32,
    r: u32,
    base: f32,
    w: u16,
    passable: bool,
) -> Option<f32> {
    let owned = s.is_owned(r, ep);
    if c.cap[r as usize] == 0 && !owned && !passable {
        return None;
    }
    Some(c.resource(r as usize, base, w, owned))
}

/// The cost of one via at `cell`, `None` for a wall.
#[inline]
fn via_at(c: &Costs, s: &Scratch, ep: u32, req: &PatternReq, cell: u32) -> Option<f32> {
    let r = c.topo.via_res(cell);
    res_cost(c, s, ep, r, c.via, req.via_weight, s.is_term(cell, ep))
}

/// Push a candidate polyline.
fn push_cand(b: &mut PatternBufs, pts: &[Pt]) {
    b.cands.extend_from_slice(pts);
    b.cand_off.push(b.cands.len());
}

/// The cheapest pattern path as nodes, with its cost.
pub(crate) fn best_pattern(
    c: &Costs,
    req: &PatternReq,
    s: &mut Scratch,
) -> Option<(Vec<u32>, f32)> {
    let ep = s.begin(req.owned, req.term_cells);
    let src_slots = req.src_slots & req.slots;
    let dst_slots = req.dst_slots & req.slots;
    if src_slots == 0 || dst_slots == 0 {
        return None;
    }
    if req.src_cell == req.dst_cell {
        return same_cell(c, req, s, ep, src_slots, dst_slots);
    }
    let mut b = std::mem::take(&mut s.pat);
    let found = search_patterns(c, req, s, ep, src_slots, dst_slots, &mut b);
    s.pat = b;
    found
}

fn search_patterns(
    c: &Costs,
    req: &PatternReq,
    s: &Scratch,
    ep: u32,
    src_slots: u32,
    dst_slots: u32,
    b: &mut PatternBufs,
) -> Option<(Vec<u32>, f32)> {
    let t = c.topo;
    let (sx, sy) = t.center(req.src_cell);
    let (tx, ty) = t.center(req.dst_cell);
    let (sp, tp) = (
        (i64::from(sx), i64::from(sy)),
        (i64::from(tx), i64::from(ty)),
    );
    let p = t.pitch;
    let ox = i64::from(t.origin.x);
    let oy = i64::from(t.origin.y);
    let col_x = |col: i64| ox + col * p + p / 2;
    let row_y = |row: i64| oy + row * p + p / 2;
    let (scol, srow) = (
        i64::from(t.col[req.src_cell as usize]),
        i64::from(t.row[req.src_cell as usize]),
    );
    let (tcol, trow) = (
        i64::from(t.col[req.dst_cell as usize]),
        i64::from(t.row[req.dst_cell as usize]),
    );

    b.cands.clear();
    b.cand_off.clear();
    b.cand_off.push(0);
    push_cand(b, &[sp, (tp.0, sp.1), tp]);
    push_cand(b, &[sp, (sp.0, tp.1), tp]);
    let mut cols = std::mem::take(&mut b.cols);
    let mut rows = std::mem::take(&mut b.rows);
    between(scol, tcol, Z_SAMPLES, &mut cols);
    for &col in &cols {
        let xm = col_x(col);
        push_cand(b, &[sp, (xm, sp.1), (xm, tp.1), tp]);
    }
    between(srow, trow, Z_SAMPLES, &mut rows);
    for &row in &rows {
        let ym = row_y(row);
        push_cand(b, &[sp, (sp.0, ym), (tp.0, ym), tp]);
    }
    let (clo, chi) = (scol.min(tcol), scol.max(tcol));
    let (rlo, rhi) = (srow.min(trow), srow.max(trow));
    for d in 1..=DETOURS {
        for col in [clo - d, chi + d] {
            if col >= i64::from(req.win.c0) && col <= i64::from(req.win.c1) {
                let xm = col_x(col);
                push_cand(b, &[sp, (xm, sp.1), (xm, tp.1), tp]);
            }
        }
        for row in [rlo - d, rhi + d] {
            if row >= i64::from(req.win.r0) && row <= i64::from(req.win.r1) {
                let ym = row_y(row);
                push_cand(b, &[sp, (sp.0, ym), (tp.0, ym), tp]);
            }
        }
    }
    between(scol, tcol, STAIR_SAMPLES, &mut cols);
    between(srow, trow, STAIR_SAMPLES, &mut rows);
    for &col in &cols {
        for &row in &rows {
            let (xm, ym) = (col_x(col), row_y(row));
            push_cand(b, &[sp, (xm, sp.1), (xm, ym), (tp.0, ym), tp]);
            push_cand(b, &[sp, (sp.0, ym), (xm, ym), (xm, tp.1), tp]);
        }
    }
    b.cols = cols;
    b.rows = rows;

    let slots = t.slots as usize;
    let src_low = src_slots.trailing_zeros() as usize;
    let dst_low = dst_slots.trailing_zeros() as usize;
    let via_src = via_at(c, s, ep, req, req.src_cell);
    let via_dst = via_at(c, s, ep, req, req.dst_cell);
    let mut best_cost = f32::INFINITY;
    b.best.clear();
    for ci in 0..b.cand_off.len() - 1 {
        let (lo, hi) = (b.cand_off[ci], b.cand_off[ci + 1]);
        let (cands, dedup, pts) = (&b.cands, &mut b.dedup, &mut b.pts);
        if !normalize(&cands[lo..hi], dedup, pts) {
            continue;
        }
        // Walk every leg.
        b.steps.clear();
        b.step_off.clear();
        b.step_off.push(0);
        b.ends.clear();
        let mut cur = req.src_cell;
        let mut ok = true;
        for w in b.pts.windows(2) {
            match walk(t, cur, w[0], w[1], &req.win, &mut b.steps) {
                Some(end) => {
                    cur = end;
                    b.ends.push(end);
                    b.step_off.push(b.steps.len());
                }
                None => {
                    ok = false;
                    break;
                }
            }
        }
        if !ok || cur != req.dst_cell {
            continue;
        }
        let legs = b.ends.len();
        // Leg costs per slot.
        b.leg_cost.clear();
        b.leg_cost.resize(legs * slots, f32::INFINITY);
        for li in 0..legs {
            let leg = &b.steps[b.step_off[li]..b.step_off[li + 1]];
            let start = if li == 0 {
                req.src_cell
            } else {
                b.ends[li - 1]
            };
            let mut m = req.slots;
            while m != 0 {
                let slot = m.trailing_zeros();
                m &= m - 1;
                let mut sum = 0.0f32;
                let mut prev = start;
                let mut feasible = true;
                for &(cell, e) in leg {
                    let r = t.planar_res(slot, e);
                    let pass = s.is_term(prev, ep) || s.is_term(cell, ep);
                    match res_cost(c, s, ep, r, c.planar_base(slot, e), req.weight, pass) {
                        Some(v) => sum += v,
                        None => {
                            feasible = false;
                            break;
                        }
                    }
                    prev = cell;
                }
                if feasible {
                    b.leg_cost[li * slots + slot as usize] = sum;
                }
            }
        }
        // DP over legs: one through via joins any two slots at a bend, so
        // each step is the better of staying and the cheapest slot plus a
        // via. dp[slot] = (cost, previous slot).
        b.back.clear();
        b.back.resize(legs * slots, usize::MAX);
        b.dp.clear();
        b.dp.resize(slots, (f32::INFINITY, usize::MAX));
        for slot in 0..slots {
            let lc = b.leg_cost[slot];
            if !lc.is_finite() {
                continue;
            }
            let entry = if src_slots & (1 << slot) != 0 {
                Some((0.0, slot))
            } else {
                via_src.map(|v| (v, src_low))
            };
            if let Some((h, a)) = entry {
                b.dp[slot] = (h + lc, a);
                b.back[slot] = a;
            }
        }
        for li in 1..legs {
            let via = via_at(c, s, ep, req, b.ends[li - 1]);
            let (mut low, mut low_at) = (f32::INFINITY, usize::MAX);
            for (a, &(pc, _)) in b.dp.iter().enumerate() {
                if pc < low {
                    (low, low_at) = (pc, a);
                }
            }
            b.next.clear();
            b.next.resize(slots, (f32::INFINITY, usize::MAX));
            for slot in 0..slots {
                let lc = b.leg_cost[li * slots + slot];
                if !lc.is_finite() {
                    continue;
                }
                let stay = b.dp[slot].0;
                let mut choice = (stay, slot);
                if let Some(v) = via {
                    if low_at != slot && low + v < stay {
                        choice = (low + v, low_at);
                    }
                }
                if choice.0.is_finite() {
                    b.next[slot] = (choice.0 + lc, choice.1);
                    b.back[li * slots + slot] = choice.1;
                }
            }
            std::mem::swap(&mut b.dp, &mut b.next);
        }
        let mut end_best = (f32::INFINITY, usize::MAX, usize::MAX);
        for (slot, &(pc, _)) in b.dp.iter().enumerate() {
            if !pc.is_finite() {
                continue;
            }
            let exit = if dst_slots & (1 << slot) != 0 {
                Some((0.0, slot))
            } else {
                via_dst.map(|v| (v, dst_low))
            };
            if let Some((h, to)) = exit {
                if pc + h < end_best.0 {
                    end_best = (pc + h, slot, to);
                }
            }
        }
        if !end_best.0.is_finite() || end_best.0 >= best_cost {
            continue;
        }
        // Recover the slot of each leg.
        b.leg_slot.clear();
        b.leg_slot.resize(legs, 0);
        b.leg_slot[legs - 1] = end_best.1;
        for li in (1..legs).rev() {
            b.leg_slot[li - 1] = b.back[li * slots + b.leg_slot[li]];
        }
        let first = b.back[b.leg_slot[0]];
        // Expand into nodes.
        b.path.clear();
        push_stack(t, &mut b.path, req.src_cell, first, b.leg_slot[0]);
        for li in 0..legs {
            let slot = b.leg_slot[li] as u32;
            for &(cell, _) in &b.steps[b.step_off[li]..b.step_off[li + 1]] {
                b.path.push(t.node(slot, cell));
            }
            let next = if li + 1 < legs {
                b.leg_slot[li + 1]
            } else {
                end_best.2
            };
            // The via ends on the node the next leg's steps start from.
            push_stack(t, &mut b.path, b.ends[li], b.leg_slot[li], next);
        }
        best_cost = end_best.0;
        std::mem::swap(&mut b.path, &mut b.best);
    }
    best_cost.is_finite().then(|| (b.best.clone(), best_cost))
}

/// Append the via at `cell` from slot `a` to slot `b` (both ends; the `a`
/// node is skipped when it is already last). One through via: the slots
/// between are crossed, not visited.
fn push_stack(t: &Topology, path: &mut Vec<u32>, cell: u32, a: usize, b: usize) {
    let first = t.node(a as u32, cell);
    if path.last() != Some(&first) {
        path.push(first);
    }
    if a != b {
        path.push(t.node(b as u32, cell));
    }
}

fn same_cell(
    c: &Costs,
    req: &PatternReq,
    s: &Scratch,
    ep: u32,
    src_slots: u32,
    dst_slots: u32,
) -> Option<(Vec<u32>, f32)> {
    let t = c.topo;
    let common = src_slots & dst_slots;
    if common != 0 {
        let slot = common.trailing_zeros();
        return Some((vec![t.node(slot, req.src_cell)], 0.0));
    }
    let r = t.via_res(req.src_cell);
    let v = res_cost(c, s, ep, r, c.via, req.via_weight, true)?;
    let (a, b) = (src_slots.trailing_zeros(), dst_slots.trailing_zeros());
    Some((vec![t.node(a, req.src_cell), t.node(b, req.src_cell)], v))
}
