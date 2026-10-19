//! Pattern routes for the seed iteration: L, Z, U-detour and 3-bend
//! shapes between two cells, each leg's layer chosen by dynamic programming
//! (FastRoute, FastGR and CUGR style).

use super::grid::{Topology, Win};
use super::search::{Costs, Scratch};

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
    pub owned: &'a [u32],
    pub term_cells: &'a [u32],
}

/// Z bend positions tried per axis.
const Z_SAMPLES: usize = 16;
/// Detour offsets (base cells) tried beyond each side of the box.
const DETOURS: i64 = 3;
/// Bend positions per axis for the 3-bend staircases.
const STAIR_SAMPLES: usize = 3;

type Pt = (i64, i64);

/// Up to `max` evenly spaced integers strictly between `a` and `b`.
fn between(a: i64, b: i64, max: usize) -> Vec<i64> {
    let (lo, hi) = (a.min(b), a.max(b));
    let n = (hi - lo - 1).max(0) as usize;
    if n <= max {
        return (lo + 1..hi).collect();
    }
    (0..max)
        .map(|i| lo + 1 + ((i as i64 * 2 + 1) * n as i64) / (2 * max as i64))
        .collect()
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

/// Drop repeated points and merge collinear legs; `None` when a leg doubles
/// back on the previous one.
fn normalize(pts: &[Pt]) -> Option<Vec<Pt>> {
    let mut v: Vec<Pt> = Vec::with_capacity(pts.len());
    for &p in pts {
        if v.last() != Some(&p) {
            v.push(p);
        }
    }
    let mut out: Vec<Pt> = Vec::with_capacity(v.len());
    for p in v {
        if out.len() >= 2 {
            let (a, b) = (out[out.len() - 2], out[out.len() - 1]);
            let same_axis = (a.0 == b.0 && b.0 == p.0) || (a.1 == b.1 && b.1 == p.1);
            if same_axis {
                let forward = (b.0 - a.0) * (p.0 - b.0) + (b.1 - a.1) * (p.1 - b.1);
                if forward < 0 {
                    return None;
                }
                out.pop();
            }
        }
        out.push(p);
    }
    Some(out)
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

/// The cheapest pattern path as nodes, with its cost.
pub(crate) fn best_pattern(
    c: &Costs,
    req: &PatternReq,
    s: &mut Scratch,
) -> Option<(Vec<u32>, f32)> {
    let t = c.topo;
    let ep = s.begin(req.owned, req.term_cells);
    let src_slots = req.src_slots & req.slots;
    let dst_slots = req.dst_slots & req.slots;
    if src_slots == 0 || dst_slots == 0 {
        return None;
    }
    if req.src_cell == req.dst_cell {
        return same_cell(c, req, s, ep, src_slots, dst_slots);
    }
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

    let mut cands: Vec<Vec<Pt>> = Vec::new();
    cands.push(vec![sp, (tp.0, sp.1), tp]);
    cands.push(vec![sp, (sp.0, tp.1), tp]);
    for col in between(scol, tcol, Z_SAMPLES) {
        let xm = col_x(col);
        cands.push(vec![sp, (xm, sp.1), (xm, tp.1), tp]);
    }
    for row in between(srow, trow, Z_SAMPLES) {
        let ym = row_y(row);
        cands.push(vec![sp, (sp.0, ym), (tp.0, ym), tp]);
    }
    let (clo, chi) = (scol.min(tcol), scol.max(tcol));
    let (rlo, rhi) = (srow.min(trow), srow.max(trow));
    for d in 1..=DETOURS {
        for col in [clo - d, chi + d] {
            if col >= i64::from(req.win.c0) && col <= i64::from(req.win.c1) {
                let xm = col_x(col);
                cands.push(vec![sp, (xm, sp.1), (xm, tp.1), tp]);
            }
        }
        for row in [rlo - d, rhi + d] {
            if row >= i64::from(req.win.r0) && row <= i64::from(req.win.r1) {
                let ym = row_y(row);
                cands.push(vec![sp, (sp.0, ym), (tp.0, ym), tp]);
            }
        }
    }
    let cols = between(scol, tcol, STAIR_SAMPLES);
    let rows = between(srow, trow, STAIR_SAMPLES);
    for &col in &cols {
        for &row in &rows {
            let (xm, ym) = (col_x(col), row_y(row));
            cands.push(vec![sp, (xm, sp.1), (xm, ym), (tp.0, ym), tp]);
            cands.push(vec![sp, (sp.0, ym), (xm, ym), (xm, tp.1), tp]);
        }
    }

    let slots = t.slots as usize;
    let mut best: Option<(f32, Vec<u32>)> = None;
    let mut steps: Vec<Vec<(u32, u32)>> = Vec::new();
    let mut leg_cost: Vec<f32> = Vec::new();
    let mut ends: Vec<u32> = Vec::new();
    for cand in &cands {
        let Some(pts) = normalize(cand) else { continue };
        // Walk every leg.
        steps.clear();
        ends.clear();
        let mut cur = req.src_cell;
        let mut ok = true;
        for w in pts.windows(2) {
            let mut v = Vec::new();
            match walk(t, cur, w[0], w[1], &req.win, &mut v) {
                Some(end) => {
                    cur = end;
                    ends.push(end);
                    steps.push(v);
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
        let legs = steps.len();
        // Leg costs per slot.
        leg_cost.clear();
        leg_cost.resize(legs * slots, f32::INFINITY);
        for (li, leg) in steps.iter().enumerate() {
            for slot in 0..slots as u32 {
                if req.slots & (1 << slot) == 0 {
                    continue;
                }
                let mut sum = 0.0f32;
                let mut prev = if li == 0 { req.src_cell } else { ends[li - 1] };
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
                    leg_cost[li * slots + slot as usize] = sum;
                }
            }
        }
        // Via cost per hop at a cell, `None` for a wall.
        let via_at = |cell: u32| -> Option<f32> {
            let r = t.via_res(cell);
            res_cost(c, s, ep, r, c.via, req.via_weight, s.is_term(cell, ep))
        };
        let hop = |cell: u32, a: usize, b: usize| -> Option<f32> {
            if a == b {
                Some(0.0)
            } else {
                via_at(cell).map(|v| v * (a as f32 - b as f32).abs())
            }
        };
        // DP over legs. dp[slot] = (cost, choice) for the current leg.
        let mut dp = vec![(f32::INFINITY, usize::MAX); slots];
        let mut back: Vec<Vec<usize>> = Vec::with_capacity(legs);
        for slot in 0..slots {
            let lc = leg_cost[slot];
            if !lc.is_finite() {
                continue;
            }
            for a in 0..slots {
                if src_slots & (1 << a) == 0 {
                    continue;
                }
                if let Some(h) = hop(req.src_cell, a, slot) {
                    let v = h + lc;
                    if v < dp[slot].0 {
                        dp[slot] = (v, a);
                    }
                }
            }
        }
        back.push(dp.iter().map(|d| d.1).collect());
        for li in 1..legs {
            let bend = ends[li - 1];
            let mut next = vec![(f32::INFINITY, usize::MAX); slots];
            for slot in 0..slots {
                let lc = leg_cost[li * slots + slot];
                if !lc.is_finite() {
                    continue;
                }
                for (a, &(pc, _)) in dp.iter().enumerate() {
                    if !pc.is_finite() {
                        continue;
                    }
                    if let Some(h) = hop(bend, a, slot) {
                        let v = pc + h + lc;
                        if v < next[slot].0 {
                            next[slot] = (v, a);
                        }
                    }
                }
            }
            back.push(next.iter().map(|d| d.1).collect());
            dp = next;
        }
        let mut end_best = (f32::INFINITY, usize::MAX, usize::MAX);
        for (slot, &(pc, _)) in dp.iter().enumerate() {
            if !pc.is_finite() {
                continue;
            }
            for b in 0..slots {
                if dst_slots & (1 << b) == 0 {
                    continue;
                }
                if let Some(h) = hop(req.dst_cell, slot, b) {
                    if pc + h < end_best.0 {
                        end_best = (pc + h, slot, b);
                    }
                }
            }
        }
        if !end_best.0.is_finite() || best.as_ref().is_some_and(|b| end_best.0 >= b.0) {
            continue;
        }
        // Recover the slot of each leg.
        let mut leg_slot = vec![0usize; legs];
        leg_slot[legs - 1] = end_best.1;
        for li in (1..legs).rev() {
            leg_slot[li - 1] = back[li][leg_slot[li]];
        }
        let first = back[0][leg_slot[0]];
        // Expand into nodes.
        let mut path = Vec::new();
        push_stack(t, &mut path, req.src_cell, first, leg_slot[0]);
        for li in 0..legs {
            let slot = leg_slot[li] as u32;
            for &(cell, _) in &steps[li] {
                path.push(t.node(slot, cell));
            }
            let next = if li + 1 < legs {
                leg_slot[li + 1]
            } else {
                end_best.2
            };
            let cell = ends[li];
            // The stack ends on the node the next leg's steps start from.
            push_stack(t, &mut path, cell, leg_slot[li], next);
        }
        best = Some((end_best.0, path));
    }
    best.map(|(cost, path)| (path, cost))
}

/// Append the via stack at `cell` from slot `a` to slot `b`, including both
/// ends (the `a` node is skipped when it is already last).
fn push_stack(t: &Topology, path: &mut Vec<u32>, cell: u32, a: usize, b: usize) {
    let first = t.node(a as u32, cell);
    if path.last() != Some(&first) {
        path.push(first);
    }
    let mut s = a;
    while s != b {
        s = if b > s { s + 1 } else { s - 1 };
        path.push(t.node(s as u32, cell));
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
    let mut best = (u32::MAX, 0usize, 0usize);
    for a in 0..t.slots as usize {
        for b in 0..t.slots as usize {
            if src_slots & (1 << a) != 0 && dst_slots & (1 << b) != 0 {
                let d = (a as i64 - b as i64).unsigned_abs() as u32;
                if d < best.0 {
                    best = (d, a, b);
                }
            }
        }
    }
    let mut path = Vec::new();
    push_stack(t, &mut path, req.src_cell, best.1, best.2);
    Some((path, v * best.0 as f32))
}
