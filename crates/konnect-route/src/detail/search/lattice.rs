//! Search lattices: the uniform GPU-shaped window and the CPU's pad-aligned
//! non-uniform octilinear lattice.
//!
//! # Non-uniform lattice
//!
//! A [`Lattice`] is the product of two sorted line sets, `xs` and `ys`
//! (Dr.CU-style; Chen et al., ICCAD 2019). It starts from the base pitch
//! (normally [`crate::api::LATTICE_PITCH_NM`]), adds lines at the fine pitch
//! across each fine region (BGA fields), and forces lines through caller
//! points: pad centres, access points, channel midlines, inflated obstacle
//! edges. A forced line closer than `snap_nm` to a base line replaces it.
//! Two neighbouring replacements can open a gap of almost
//! `base_pitch + 2 · snap_nm`, so after snapping every gap wider than
//! `base_pitch + snap_nm` is split evenly; with `snap_nm ≤ 0.4 · base_pitch`
//! no gap then exceeds `base_pitch + snap_nm` (below `√2 · base_pitch`) and
//! none is below `snap_nm`. [`Lattice::from_axes`] rejects axes with a gap
//! above `√2 · base_pitch`.
//!
//! Moves:
//! - straight moves go to the adjacent line, at a cost proportional to the
//!   gap (1000 per base pitch, rounded up);
//! - a diagonal move goes to the nearest node exactly on the 45° ray, if
//!   one lies within one base pitch on both axes; otherwise that diagonal
//!   does not exist from this node.
//!
//! Every point of an edge is then within one base half-diagonal of one of
//! its endpoints (a straight edge is at most `√2 · base_pitch` long, a
//! diagonal at most one base pitch on each axis), so the tri-state contract
//! of [`crate::compute::raster`] holds with the halo painted at the base
//! pitch. Rasters for a non-uniform
//! lattice are indexed by lattice `(col, row)`; their [`Window`] carries the
//! dimensions and base pitch but **not** the node positions, so paint them
//! with [`paint_on_lattice`], never with `CpuCompute::paint`.

use crate::compute::raster::{TileRaster, Window};
use crate::compute::{PaintKind, PaintShape, ProfileGeometry};
use konnect_geom::{gap_cmp, BBox, Circle, Coord, Point, Shape};
use std::cmp::Ordering;

use super::heading_delta;

/// The cost of one base-pitch straight step; a diagonal base step is 1414.
pub const STRAIGHT: u32 = 1000;
pub const DIAGONAL: u32 = 1414;

/// One move on a lattice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Step {
    pub col: u32,
    pub row: u32,
    /// Base cost of the edge (1000 per base pitch of travel, octile).
    pub cost: u32,
    /// Run-length increment in half base pitches, rounded down.
    pub run: u8,
}

/// What the search kernel needs from a lattice.
pub(crate) trait Grid: Sync {
    fn cols(&self) -> u32;
    fn rows(&self) -> u32;
    fn pitch(&self) -> Coord;
    fn x(&self, col: u32) -> Coord;
    fn y(&self, row: u32) -> Coord;
    /// The neighbour in heading `h` (0..8), if the lattice has that edge.
    fn step(&self, col: u32, row: u32, h: u8) -> Option<Step>;
    /// The last column with `x(col) <= x` (0 if none).
    fn col_at_or_before(&self, x: Coord) -> u32;
    /// The first column with `x(col) >= x` (`cols - 1` if none).
    fn col_at_or_after(&self, x: Coord) -> u32;
    fn row_at_or_before(&self, y: Coord) -> u32;
    fn row_at_or_after(&self, y: Coord) -> u32;
    fn uniform(&self) -> bool;
}

/// A uniform window as a [`Grid`].
#[derive(Debug, Clone, Copy)]
pub(crate) struct UniformGrid {
    pub window: Window,
}

impl UniformGrid {
    pub fn new(window: Window) -> Self {
        UniformGrid { window }
    }
}

impl Grid for UniformGrid {
    #[inline]
    fn cols(&self) -> u32 {
        self.window.cols
    }
    #[inline]
    fn rows(&self) -> u32 {
        self.window.rows
    }
    #[inline]
    fn pitch(&self) -> Coord {
        self.window.pitch_nm
    }
    #[inline]
    fn x(&self, col: u32) -> Coord {
        (i64::from(self.window.origin.x) + i64::from(self.window.pitch_nm) * i64::from(col))
            as Coord
    }
    #[inline]
    fn y(&self, row: u32) -> Coord {
        (i64::from(self.window.origin.y) + i64::from(self.window.pitch_nm) * i64::from(row))
            as Coord
    }
    #[inline]
    fn step(&self, col: u32, row: u32, h: u8) -> Option<Step> {
        let (dc, dr) = heading_delta(h);
        let c = col as i32 + dc;
        let r = row as i32 + dr;
        if c < 0 || r < 0 || c as u32 >= self.window.cols || r as u32 >= self.window.rows {
            return None;
        }
        Some(Step {
            col: c as u32,
            row: r as u32,
            cost: if h & 1 == 0 { STRAIGHT } else { DIAGONAL },
            run: 2,
        })
    }
    fn col_at_or_before(&self, x: Coord) -> u32 {
        let rel = i64::from(x) - i64::from(self.window.origin.x);
        let c = rel.div_euclid(i64::from(self.window.pitch_nm));
        c.clamp(0, i64::from(self.window.cols) - 1) as u32
    }
    fn col_at_or_after(&self, x: Coord) -> u32 {
        let rel = i64::from(x) - i64::from(self.window.origin.x);
        let p = i64::from(self.window.pitch_nm);
        let c = (rel + p - 1).div_euclid(p);
        c.clamp(0, i64::from(self.window.cols) - 1) as u32
    }
    fn row_at_or_before(&self, y: Coord) -> u32 {
        let rel = i64::from(y) - i64::from(self.window.origin.y);
        let r = rel.div_euclid(i64::from(self.window.pitch_nm));
        r.clamp(0, i64::from(self.window.rows) - 1) as u32
    }
    fn row_at_or_after(&self, y: Coord) -> u32 {
        let rel = i64::from(y) - i64::from(self.window.origin.y);
        let p = i64::from(self.window.pitch_nm);
        let r = (rel + p - 1).div_euclid(p);
        r.clamp(0, i64::from(self.window.rows) - 1) as u32
    }
    fn uniform(&self) -> bool {
        true
    }
}

/// How to build a non-uniform lattice.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LatticeSpec {
    /// The span to cover (window including halo). Lines start at `min`.
    pub bbox: BBox,
    /// Normally [`crate::api::LATTICE_PITCH_NM`]; the cost unit.
    pub base_pitch: Coord,
    /// Normally [`crate::api::LATTICE_FINE_PITCH_NM`].
    pub fine_pitch: Coord,
    /// Regions refined to `fine_pitch` (lines span the whole lattice).
    pub fine_regions: Vec<BBox>,
    /// Lines forced through these x values (pad centres, access points,
    /// obstacle edges, channel midlines).
    pub forced_x: Vec<Coord>,
    pub forced_y: Vec<Coord>,
    /// Minimum line spacing; a forced line this close to a base line
    /// replaces it. At most `0.4 · base_pitch`, so the largest gap after
    /// snapping and splitting, `base_pitch + snap_nm`, stays below
    /// `√2 · base_pitch` and the halo contract holds.
    pub snap_nm: Coord,
}

impl LatticeSpec {
    /// A spec with the frozen pitches and no refinement.
    pub fn new(bbox: BBox) -> Self {
        LatticeSpec {
            bbox,
            base_pitch: crate::api::LATTICE_PITCH_NM,
            fine_pitch: crate::api::LATTICE_FINE_PITCH_NM,
            fine_regions: Vec::new(),
            forced_x: Vec::new(),
            forced_y: Vec::new(),
            snap_nm: crate::api::LATTICE_PITCH_NM / 5,
        }
    }
}

impl LatticeSpec {
    /// Force a horizontal and a vertical line through `p`.
    pub fn force_point(&mut self, p: Point) {
        self.forced_x.push(p.x);
        self.forced_y.push(p.y);
    }

    /// Force lines along the edges of `b` grown by `inflate`: where the
    /// centre line of a track hugging an obstacle runs when `inflate` is
    /// the clearance plus half the track width.
    pub fn force_box_edges(&mut self, b: BBox, inflate: Coord) {
        let g = b.inflate(inflate);
        self.forced_x.extend([g.min.x, g.max.x]);
        self.forced_y.extend([g.min.y, g.max.y]);
    }

    /// Force lines midway between consecutive distinct values of `xs` and
    /// of `ys` that are at most `max_gap` apart: the channel centres of a
    /// ball grid whose centres are `(xs, ys)`.
    pub fn force_channel_midlines(&mut self, xs: &[Coord], ys: &[Coord], max_gap: Coord) {
        let mid = |v: &[Coord], out: &mut Vec<Coord>| {
            let mut v = v.to_vec();
            v.sort_unstable();
            v.dedup();
            for w in v.windows(2) {
                let gap = i64::from(w[1]) - i64::from(w[0]);
                if gap <= i64::from(max_gap) {
                    out.push(((i64::from(w[0]) + i64::from(w[1])) / 2) as Coord);
                }
            }
        };
        mid(xs, &mut self.forced_x);
        mid(ys, &mut self.forced_y);
    }
}

/// One diagonal neighbour: column and row offsets (magnitudes, the signs
/// come from the heading) and the edge cost. `dc == 0` means none.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct Diag {
    dc: u8,
    dr: u8,
    cost: u16,
}

/// A lattice: uniform (a [`Window`]) or pad-aligned non-uniform.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Lattice {
    xs: Vec<Coord>,
    ys: Vec<Coord>,
    base_pitch: Coord,
    uniform: bool,
    window: Window,
    /// `step_x[i]`: cost from column `i` to `i + 1`.
    step_x: Vec<u16>,
    step_y: Vec<u16>,
    run_x: Vec<u8>,
    run_y: Vec<u8>,
    /// Four per node (`node · 4 + d`, `d = heading / 2` for odd headings).
    diag: Vec<Diag>,
}

fn ceil_div(a: u64, b: u64) -> u64 {
    a.div_ceil(b)
}

/// Build one axis: base lines, fine lines inside `fine`, forced lines; snap.
fn build_axis(
    lo: Coord,
    hi: Coord,
    base: Coord,
    fine_pitch: Coord,
    fine: &[(Coord, Coord)],
    forced: &[Coord],
    snap: Coord,
) -> Vec<Coord> {
    // (value, priority): forced 2, fine 1, base 0.
    let mut cand: Vec<(Coord, u8)> = Vec::new();
    let mut v = i64::from(lo);
    while v <= i64::from(hi) {
        cand.push((v as Coord, 0));
        v += i64::from(base);
    }
    for &(a, b) in fine {
        let a = a.max(lo);
        let b = b.min(hi);
        if a > b || fine_pitch <= 0 {
            continue;
        }
        // Fine lines stay on the base grid's phase so they nest.
        let k0 = (i64::from(a) - i64::from(lo)).div_euclid(i64::from(fine_pitch));
        let mut v = i64::from(lo) + k0 * i64::from(fine_pitch);
        while v <= i64::from(b) {
            if v >= i64::from(a) {
                cand.push((v as Coord, 1));
            }
            v += i64::from(fine_pitch);
        }
    }
    for &f in forced {
        if (lo..=hi).contains(&f) {
            cand.push((f, 2));
        }
    }
    cand.sort_unstable();
    let mut kept: Vec<(Coord, u8)> = Vec::with_capacity(cand.len());
    for (v, p) in cand {
        match kept.last().copied() {
            None => kept.push((v, p)),
            Some((lv, lp)) => {
                if i64::from(v) - i64::from(lv) >= i64::from(snap) {
                    kept.push((v, p));
                } else if p > lp {
                    // Replace the weaker line if that keeps the spacing.
                    let n = kept.len();
                    let ok = n < 2 || i64::from(v) - i64::from(kept[n - 2].0) >= i64::from(snap);
                    if ok {
                        kept[n - 1] = (v, p);
                    }
                }
            }
        }
    }
    // A replacement can follow a dropped base line (base lines 0 and 50,
    // forced -9 and 59: a 68 gap), so split every gap above `base + snap`
    // into `ceil(gap / base)` equal parts. Each part is at most `base` and
    // above `base / 2 ≥ snap`, so the spacing floor still holds.
    let max_gap = i64::from(base) + i64::from(snap);
    let mut out: Vec<Coord> = Vec::with_capacity(kept.len());
    for (v, _) in kept {
        if let Some(&last) = out.last() {
            let gap = i64::from(v) - i64::from(last);
            if gap > max_gap {
                let parts = (gap + i64::from(base) - 1) / i64::from(base);
                for k in 1..parts {
                    out.push((i64::from(last) + gap * k / parts) as Coord);
                }
            }
        }
        out.push(v);
    }
    out
}

/// Whether a gap of `gap` nm keeps every point of a straight edge within
/// the base half-diagonal of an endpoint: `gap ≤ √2 · base`.
fn gap_within_contract(gap: i64, base: Coord) -> bool {
    let (g, b) = (i128::from(gap), i128::from(base));
    g * g <= 2 * b * b
}

impl Lattice {
    /// The uniform lattice of `window`.
    pub fn uniform(window: Window) -> Self {
        let xs = (0..window.cols).map(|c| window.node(c, 0).x).collect();
        let ys = (0..window.rows).map(|r| window.node(0, r).y).collect();
        Lattice {
            xs,
            ys,
            base_pitch: window.pitch_nm,
            uniform: true,
            window,
            step_x: Vec::new(),
            step_y: Vec::new(),
            run_x: Vec::new(),
            run_y: Vec::new(),
            diag: Vec::new(),
        }
    }

    /// Build a pad-aligned non-uniform lattice. `None` if the spec is
    /// degenerate or the node count leaves `u32`/`u16` lattice indices.
    pub fn build(spec: &LatticeSpec) -> Option<Self> {
        let p = spec.base_pitch;
        if p <= 0 || spec.snap_nm <= 0 || spec.snap_nm * 5 > p * 2 {
            return None;
        }
        let fx: Vec<(Coord, Coord)> = spec
            .fine_regions
            .iter()
            .map(|b| (b.min.x, b.max.x))
            .collect();
        let fy: Vec<(Coord, Coord)> = spec
            .fine_regions
            .iter()
            .map(|b| (b.min.y, b.max.y))
            .collect();
        let xs = build_axis(
            spec.bbox.min.x,
            spec.bbox.max.x,
            p,
            spec.fine_pitch,
            &fx,
            &spec.forced_x,
            spec.snap_nm,
        );
        let ys = build_axis(
            spec.bbox.min.y,
            spec.bbox.max.y,
            p,
            spec.fine_pitch,
            &fy,
            &spec.forced_y,
            spec.snap_nm,
        );
        Self::from_axes(xs, ys, p)
    }

    /// A lattice from explicit sorted, strictly increasing axes. `None` if
    /// an axis is unsorted or has a gap above `√2 · base_pitch`, which
    /// would let an edge between two Free nodes pass an obstacle.
    pub fn from_axes(xs: Vec<Coord>, ys: Vec<Coord>, base_pitch: Coord) -> Option<Self> {
        let bad_axis = |a: &[Coord]| {
            a.windows(2).any(|w| {
                w[0] >= w[1] || !gap_within_contract(i64::from(w[1]) - i64::from(w[0]), base_pitch)
            })
        };
        if xs.is_empty()
            || ys.is_empty()
            || base_pitch <= 0
            || xs.len() > usize::from(u16::MAX)
            || ys.len() > usize::from(u16::MAX)
            || bad_axis(&xs)
            || bad_axis(&ys)
        {
            return None;
        }
        // The raster window: dimensions and base pitch only.
        let window = Window {
            origin: Point::new(xs[0], ys[0]),
            pitch_nm: base_pitch,
            cols: xs.len() as u32,
            rows: ys.len() as u32,
        };
        let pitch = base_pitch as u64;
        let gap_costs = |axis: &[Coord]| -> (Vec<u16>, Vec<u8>) {
            axis.windows(2)
                .map(|w| {
                    let len = (i64::from(w[1]) - i64::from(w[0])) as u64;
                    let c = ceil_div(u64::from(STRAIGHT) * len, pitch).min(u64::from(u16::MAX));
                    let r = (2 * len / pitch).min(255);
                    (c as u16, r as u8)
                })
                .unzip()
        };
        let (step_x, run_x) = gap_costs(&xs);
        let (step_y, run_y) = gap_costs(&ys);
        let cols = xs.len();
        let rows = ys.len();
        let mut diag = vec![Diag::default(); cols * rows * 4];
        for r in 0..rows {
            for c in 0..cols {
                for d in 0..4u8 {
                    let h = 2 * d + 1;
                    let (sc, sr) = heading_delta(h);
                    if let Some((dc, dr, t)) = diag_target(&xs, &ys, c, r, sc, sr, base_pitch) {
                        let cost = ceil_div(u64::from(DIAGONAL) * t as u64, pitch)
                            .min(u64::from(u16::MAX));
                        diag[(r * cols + c) * 4 + d as usize] = Diag {
                            dc,
                            dr,
                            cost: cost as u16,
                        };
                    }
                }
            }
        }
        Some(Lattice {
            xs,
            ys,
            base_pitch,
            uniform: false,
            window,
            step_x,
            step_y,
            run_x,
            run_y,
            diag,
        })
    }

    pub fn is_uniform(&self) -> bool {
        self.uniform
    }

    /// The raster window this lattice's rasters must have.
    pub fn window(&self) -> Window {
        self.window
    }

    pub fn base_pitch(&self) -> Coord {
        self.base_pitch
    }

    pub fn cols(&self) -> u32 {
        self.xs.len() as u32
    }

    pub fn rows(&self) -> u32 {
        self.ys.len() as u32
    }

    pub fn xs(&self) -> &[Coord] {
        &self.xs
    }

    pub fn ys(&self) -> &[Coord] {
        &self.ys
    }

    pub fn node(&self, col: u32, row: u32) -> Point {
        Point::new(self.xs[col as usize], self.ys[row as usize])
    }

    pub fn node_count(&self) -> usize {
        self.xs.len() * self.ys.len()
    }

    /// The node nearest to `p` (ties to the lower index), if `p` lies within
    /// the lattice's span.
    pub fn nearest(&self, p: Point) -> Option<(u32, u32)> {
        let near = |axis: &[Coord], v: Coord| -> Option<u32> {
            if v < axis[0] || v > *axis.last()? {
                return None;
            }
            let i = axis.partition_point(|&a| a < v);
            if i == 0 {
                return Some(0);
            }
            if i == axis.len() {
                return Some(i as u32 - 1);
            }
            let (a, b) = (axis[i - 1], axis[i]);
            Some(if v - a <= b - v {
                i as u32 - 1
            } else {
                i as u32
            })
        };
        Some((near(&self.xs, p.x)?, near(&self.ys, p.y)?))
    }

    /// Whether `(col, row)` sits exactly on `p`.
    pub fn hits(&self, p: Point) -> bool {
        self.xs.binary_search(&p.x).is_ok() && self.ys.binary_search(&p.y).is_ok()
    }
}

/// The nearest node on the 45° ray from `(c, r)` with column step `sc` and
/// row step `sr`, within one base pitch on both axes: `(dc, dr, t)`.
fn diag_target(
    xs: &[Coord],
    ys: &[Coord],
    c: usize,
    r: usize,
    sc: i32,
    sr: i32,
    pitch: Coord,
) -> Option<(u8, u8, i64)> {
    let x0 = i64::from(xs[c]);
    let y0 = i64::from(ys[r]);
    let mut a: i64 = 1;
    let mut b: i64 = 1;
    loop {
        let ci = c as i64 + i64::from(sc) * a;
        let ri = r as i64 + i64::from(sr) * b;
        if ci < 0 || ri < 0 || ci >= xs.len() as i64 || ri >= ys.len() as i64 || a > 255 || b > 255
        {
            return None;
        }
        let tx = (i64::from(xs[ci as usize]) - x0).abs();
        let ty = (i64::from(ys[ri as usize]) - y0).abs();
        if tx > i64::from(pitch) || ty > i64::from(pitch) {
            return None;
        }
        match tx.cmp(&ty) {
            Ordering::Equal => return Some((a as u8, b as u8, tx)),
            Ordering::Less => a += 1,
            Ordering::Greater => b += 1,
        }
    }
}

/// A non-uniform [`Lattice`] as a [`Grid`].
#[derive(Debug, Clone, Copy)]
pub(crate) struct LatticeGrid<'a> {
    pub lattice: &'a Lattice,
}

impl Grid for LatticeGrid<'_> {
    #[inline]
    fn cols(&self) -> u32 {
        self.lattice.xs.len() as u32
    }
    #[inline]
    fn rows(&self) -> u32 {
        self.lattice.ys.len() as u32
    }
    #[inline]
    fn pitch(&self) -> Coord {
        self.lattice.base_pitch
    }
    #[inline]
    fn x(&self, col: u32) -> Coord {
        self.lattice.xs[col as usize]
    }
    #[inline]
    fn y(&self, row: u32) -> Coord {
        self.lattice.ys[row as usize]
    }
    #[inline]
    fn step(&self, col: u32, row: u32, h: u8) -> Option<Step> {
        let l = self.lattice;
        let cols = l.xs.len() as u32;
        let rows = l.ys.len() as u32;
        match h {
            0 => (col + 1 < cols).then(|| Step {
                col: col + 1,
                row,
                cost: u32::from(l.step_x[col as usize]),
                run: l.run_x[col as usize],
            }),
            4 => (col > 0).then(|| Step {
                col: col - 1,
                row,
                cost: u32::from(l.step_x[col as usize - 1]),
                run: l.run_x[col as usize - 1],
            }),
            // Heading 2 is +y in the Y-up sense: towards row 0.
            2 => (row > 0).then(|| Step {
                col,
                row: row - 1,
                cost: u32::from(l.step_y[row as usize - 1]),
                run: l.run_y[row as usize - 1],
            }),
            6 => (row + 1 < rows).then(|| Step {
                col,
                row: row + 1,
                cost: u32::from(l.step_y[row as usize]),
                run: l.run_y[row as usize],
            }),
            _ => {
                let d =
                    l.diag[(row as usize * cols as usize + col as usize) * 4 + (h as usize >> 1)];
                if d.dc == 0 {
                    return None;
                }
                let (sc, sr) = heading_delta(h);
                Some(Step {
                    col: (col as i32 + sc * i32::from(d.dc)) as u32,
                    row: (row as i32 + sr * i32::from(d.dr)) as u32,
                    cost: u32::from(d.cost),
                    // t·√2 in half base pitches.
                    run: ((u64::from(d.cost) * 2 / u64::from(STRAIGHT)).min(255)) as u8,
                })
            }
        }
    }
    fn col_at_or_before(&self, x: Coord) -> u32 {
        let i = self.lattice.xs.partition_point(|&v| v <= x);
        i.saturating_sub(1) as u32
    }
    fn col_at_or_after(&self, x: Coord) -> u32 {
        let i = self.lattice.xs.partition_point(|&v| v < x);
        i.min(self.lattice.xs.len() - 1) as u32
    }
    fn row_at_or_before(&self, y: Coord) -> u32 {
        let i = self.lattice.ys.partition_point(|&v| v <= y);
        i.saturating_sub(1) as u32
    }
    fn row_at_or_after(&self, y: Coord) -> u32 {
        let i = self.lattice.ys.partition_point(|&v| v < y);
        i.min(self.lattice.ys.len() - 1) as u32
    }
    fn uniform(&self) -> bool {
        false
    }
}

// ─── Painting and masks ──────────────────────────────────────────────────────

fn half_diagonal(pitch: Coord) -> Coord {
    (f64::from(pitch) * std::f64::consts::FRAC_1_SQRT_2).ceil() as Coord
}

fn bump(c: &mut u8, up: bool) {
    if *c == u8::MAX {
        return;
    }
    if up {
        *c += 1;
    } else {
        debug_assert!(*c > 0, "unpaint of a shape that was never painted");
        *c = c.saturating_sub(1);
    }
}

fn bump16(c: &mut u16, up: bool) {
    if *c == u16::MAX {
        return;
    }
    if up {
        *c += 1;
    } else {
        debug_assert!(*c > 0, "unpaint of a shape that was never painted");
        *c = c.saturating_sub(1);
    }
}

/// Paint (`up`) or unpaint `shapes` into a raster of `lattice`, with the same
/// CORE/HALO semantics as `CpuCompute::paint` but at the lattice's node
/// positions. The halo uses the base pitch's half-diagonal.
pub fn paint_on_lattice(
    raster: &mut TileRaster,
    lattice: &Lattice,
    shapes: &[PaintShape],
    profile: &ProfileGeometry,
    up: bool,
) {
    debug_assert_eq!(raster.window.cols, lattice.cols());
    debug_assert_eq!(raster.window.rows, lattice.rows());
    let half_width = profile.track_width / 2 + profile.track_width % 2;
    let hd = half_diagonal(lattice.base_pitch);
    let grid = LatticeGrid { lattice };
    for paint in shapes {
        let core_clearance = i64::from(paint.clearance) + i64::from(profile.sagitta);
        let halo_clearance = core_clearance + i64::from(hd);
        let reach = paint
            .clearance
            .saturating_add(profile.sagitta)
            .saturating_add(hd)
            .saturating_add(half_width)
            .saturating_add(1);
        let region = paint.shape.bbox().inflate(reach);
        let span = BBox::new(
            lattice.node(0, 0),
            lattice.node(lattice.cols() - 1, lattice.rows() - 1),
        );
        if !region.intersects(&span) {
            continue;
        }
        let c0 = grid.col_at_or_after(region.min.x);
        let c1 = grid.col_at_or_before(region.max.x);
        let r0 = grid.row_at_or_after(region.min.y);
        let r1 = grid.row_at_or_before(region.max.y);
        if c0 > c1 || r0 > r1 {
            continue;
        }
        for row in r0..=r1 {
            for col in c0..=c1 {
                let track = Shape::Circle(Circle::new(lattice.node(col, row), half_width));
                let in_core = gap_cmp(&paint.shape, &track, core_clearance) == Ordering::Less;
                let in_halo =
                    in_core || gap_cmp(&paint.shape, &track, halo_clearance) == Ordering::Less;
                if !in_halo {
                    continue;
                }
                match paint.kind {
                    PaintKind::Hard => {
                        bump(raster.halo.get_mut(col, row), up);
                        if in_core {
                            bump(raster.core.get_mut(col, row), up);
                        }
                    }
                    PaintKind::Soft => {
                        if in_core {
                            bump16(raster.present.get_mut(col, row), up);
                        }
                    }
                }
            }
        }
    }
}

/// A via as the mask derivation sees it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ViaGeometry {
    /// Radius of the via's copper, nm.
    pub radius: Coord,
    /// Half the track width the raster was painted for, nm.
    pub track_half_width: Coord,
}

/// `via_site` values written by the mask derivation. [`VIA_NO`]: rejected
/// without a check. A Blocked node lies within the via's reach; that is a
/// certain violation only when the via's clearance is at least the track
/// clearance the raster was painted with and the sagitta is 0, and merely
/// conservative otherwise (it can reject a legal via under type-specific
/// rules).
pub const VIA_NO: u8 = 0;
/// A via may fit; an exact check decides.
pub const VIA_CHECK: u8 = 1;
/// Every point of the via disc is clear of this layer's hard obstacles by
/// the raster's clearance (sampled exactly through the halo bound).
pub const VIA_FREE: u8 = 2;

/// Derive `heading_mask` and `via_site` for a raster on `grid`.
///
/// - `heading_mask` bit `h` is set when the node is not Blocked and its
///   neighbour in heading `h` exists and is not Blocked.
/// - `via_site`: without via geometry, [`VIA_CHECK`] on every node that is
///   not Blocked. With it, let `ρ = radius − track_half_width`: a node with
///   any Blocked node within `ρ` gets [`VIA_NO`]; a node whose nodes
///   within `ρ + half-diagonal` all have `halo == 0`, and whose lattice
///   cells under the disc all have a covering radius (half the cell
///   diagonal) of at most the half-diagonal, gets [`VIA_FREE`]; anything
///   else [`VIA_CHECK`]. The covering condition always holds on a uniform
///   lattice; on a non-uniform one a cell wider than the base pitch can
///   leave a disc point farther than the half-diagonal from every node,
///   where `halo == 0` proves nothing.
pub(crate) fn derive_masks_on<G: Grid>(
    raster: &mut TileRaster,
    grid: &G,
    via: Option<ViaGeometry>,
) {
    let (cols, rows) = (grid.cols(), grid.rows());
    derive_masks_range(raster, grid, via, 0, cols - 1, 0, rows - 1);
}

/// [`derive_masks_on`] restricted to columns `c0..=c1`, rows `r0..=r1`.
#[allow(clippy::too_many_arguments)]
pub(crate) fn derive_masks_range<G: Grid>(
    raster: &mut TileRaster,
    grid: &G,
    via: Option<ViaGeometry>,
    c0: u32,
    c1: u32,
    r0: u32,
    r1: u32,
) {
    debug_assert_eq!(raster.window.cols, grid.cols());
    debug_assert_eq!(raster.window.rows, grid.rows());
    let c1 = c1.min(grid.cols() - 1);
    let r1 = r1.min(grid.rows() - 1);
    for row in r0..=r1 {
        for col in c0..=c1 {
            let mut mask = 0u8;
            if raster.core.get(col, row) == 0 {
                for h in 0..8u8 {
                    if let Some(s) = grid.step(col, row, h) {
                        if raster.core.get(s.col, s.row) == 0 {
                            mask |= 1 << h;
                        }
                    }
                }
            }
            raster.heading_mask.set(col, row, mask);
        }
    }
    match via {
        None => {
            for row in r0..=r1 {
                for col in c0..=c1 {
                    let v = if raster.core.get(col, row) == 0 {
                        VIA_CHECK
                    } else {
                        VIA_NO
                    };
                    raster.via_site.set(col, row, v);
                }
            }
        }
        Some(v) => {
            let rho = i64::from((v.radius - v.track_half_width).max(0));
            let outer = rho + i64::from(half_diagonal(grid.pitch()));
            for row in r0..=r1 {
                for col in c0..=c1 {
                    let site = via_site_at(raster, grid, col, row, rho, outer);
                    raster.via_site.set(col, row, site);
                }
            }
        }
    }
}

/// Re-derive masks after painting or unpainting inside `changed` (board
/// nm): every node whose heading bits or via site can depend on a node in
/// `changed` is recomputed. `changed` should be the painted shapes' bbox
/// grown by their reach (clearance, half track width, half-diagonal).
pub fn derive_masks_in_lattice(
    raster: &mut TileRaster,
    lattice: &Lattice,
    via: Option<ViaGeometry>,
    changed: BBox,
) {
    // A neighbour step is at most one base pitch plus the snap; a via disc
    // reaches ρ + half-diagonal.
    let reach = i64::from(lattice.base_pitch) * 2
        + via.map_or(0, |v| i64::from((v.radius - v.track_half_width).max(0)))
        + i64::from(half_diagonal(lattice.base_pitch));
    let cl = |v: i64| v.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as Coord;
    let lo_x = cl(i64::from(changed.min.x) - reach);
    let hi_x = cl(i64::from(changed.max.x) + reach);
    let lo_y = cl(i64::from(changed.min.y) - reach);
    let hi_y = cl(i64::from(changed.max.y) + reach);
    if lattice.is_uniform() {
        let g = UniformGrid::new(lattice.window());
        let (c0, c1) = (g.col_at_or_before(lo_x), g.col_at_or_after(hi_x));
        let (r0, r1) = (g.row_at_or_before(lo_y), g.row_at_or_after(hi_y));
        derive_masks_range(raster, &g, via, c0, c1, r0, r1);
    } else {
        let g = LatticeGrid { lattice };
        let (c0, c1) = (g.col_at_or_before(lo_x), g.col_at_or_after(hi_x));
        let (r0, r1) = (g.row_at_or_before(lo_y), g.row_at_or_after(hi_y));
        derive_masks_range(raster, &g, via, c0, c1, r0, r1);
    }
}

fn via_site_at<G: Grid>(
    raster: &TileRaster,
    grid: &G,
    col: u32,
    row: u32,
    rho: i64,
    outer: i64,
) -> u8 {
    if raster.core.get(col, row) > 0 {
        return VIA_NO;
    }
    let cx = i64::from(grid.x(col));
    let cy = i64::from(grid.y(row));
    let clamp = |v: i64| v.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as Coord;
    let c0 = grid.col_at_or_after(clamp(cx - outer));
    let c1 = grid.col_at_or_before(clamp(cx + outer));
    let r0 = grid.row_at_or_after(clamp(cy - outer));
    let r1 = grid.row_at_or_before(clamp(cy + outer));
    let (rho2, outer2) = (rho * rho, outer * outer);
    let mut free = true;
    // A disc reaching past the lattice cannot be judged free here.
    if cx - outer < i64::from(grid.x(0))
        || cx + outer > i64::from(grid.x(grid.cols() - 1))
        || cy - outer < i64::from(grid.y(0))
        || cy + outer > i64::from(grid.y(grid.rows() - 1))
    {
        free = false;
    }
    for r in r0..=r1 {
        let dy = i64::from(grid.y(r)) - cy;
        for c in c0..=c1 {
            let dx = i64::from(grid.x(c)) - cx;
            let d2 = dx * dx + dy * dy;
            if d2 > outer2 {
                continue;
            }
            if d2 <= rho2 && raster.core.get(c, r) > 0 {
                return VIA_NO;
            }
            if raster.halo.get(c, r) > 0 {
                free = false;
            }
        }
    }
    if free && !grid.uniform() && !cells_covered(grid, cx, cy, rho, outer - rho) {
        free = false;
    }
    if free {
        VIA_FREE
    } else {
        VIA_CHECK
    }
}

/// Whether every lattice cell meeting the box of the `ρ`-disc at `(cx, cy)`
/// has a covering radius (half its diagonal) of at most `hd`, so every
/// disc point lies within `hd` of a cell corner, and that corner within
/// `ρ + hd` of the centre. Conservative: it pairs the widest column gap
/// with the widest row gap. The caller has checked that the disc lies
/// inside the lattice.
fn cells_covered<G: Grid>(grid: &G, cx: i64, cy: i64, rho: i64, hd: i64) -> bool {
    let clamp = |v: i64| v.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as Coord;
    let widest = |lo: u32, hi: u32, at: &dyn Fn(u32) -> Coord| -> i64 {
        (lo..hi)
            .map(|i| i64::from(at(i + 1)) - i64::from(at(i)))
            .max()
            .unwrap_or(0)
    };
    let gx = widest(
        grid.col_at_or_before(clamp(cx - rho)),
        grid.col_at_or_after(clamp(cx + rho)),
        &|c| grid.x(c),
    );
    let gy = widest(
        grid.row_at_or_before(clamp(cy - rho)),
        grid.row_at_or_after(clamp(cy + rho)),
        &|r| grid.y(r),
    );
    let (gx, gy, hd) = (i128::from(gx), i128::from(gy), i128::from(hd));
    gx * gx + gy * gy <= 4 * hd * hd
}

/// Derive masks for a raster of `lattice` (uniform or not).
pub fn derive_masks_on_lattice(
    raster: &mut TileRaster,
    lattice: &Lattice,
    via: Option<ViaGeometry>,
) {
    if lattice.is_uniform() {
        derive_masks_on(raster, &UniformGrid::new(lattice.window()), via);
    } else {
        derive_masks_on(raster, &LatticeGrid { lattice }, via);
    }
}

/// Derive masks for a uniform raster from its own window.
pub fn derive_masks_uniform(raster: &mut TileRaster, via: Option<ViaGeometry>) {
    let grid = UniformGrid::new(raster.window);
    derive_masks_on(raster, &grid, via);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::api::{LayerId, ProfileId, TileId};

    #[test]
    fn forced_lines_replace_close_base_lines_and_fine_regions_nest() {
        let mut spec =
            LatticeSpec::new(BBox::new(Point::new(0, 0), Point::new(1_000_000, 500_000)));
        spec.forced_x = vec![203_000, 330_000];
        spec.fine_regions = vec![BBox::new(
            Point::new(600_000, 0),
            Point::new(700_000, 100_000),
        )];
        let l = Lattice::build(&spec).unwrap();
        let xs = l.xs();
        // 203 µm is within 10 µm of 200 µm: it replaces that base line.
        assert!(xs.contains(&203_000) && !xs.contains(&200_000));
        // 330 µm is 20 µm from both neighbours: inserted.
        assert!(xs.contains(&300_000) && xs.contains(&330_000) && xs.contains(&350_000));
        // Fine lines at 25 µm inside [600, 700] µm.
        for v in [600_000, 625_000, 650_000, 675_000, 700_000] {
            assert!(xs.contains(&v), "missing {v}");
        }
        assert!(xs.windows(2).all(|w| w[1] - w[0] >= spec.snap_nm));
        assert!(xs
            .windows(2)
            .all(|w| w[1] - w[0] <= spec.base_pitch + spec.snap_nm));
        // The fine region's y lines span every column.
        assert!(l.ys().contains(&75_000));
        assert!(l.hits(Point::new(330_000, 75_000)));
    }

    fn max_gap(axis: &[Coord]) -> i64 {
        axis.windows(2)
            .map(|w| i64::from(w[1]) - i64::from(w[0]))
            .max()
            .unwrap_or(0)
    }

    #[test]
    fn neighbouring_replacements_do_not_open_a_gap_past_the_contract() {
        // Base lines at 50 and 100 µm both replaced from the outside.
        for (snap, forced) in [(10_000, [41_000, 109_000]), (20_000, [31_000, 119_000])] {
            let mut spec =
                LatticeSpec::new(BBox::new(Point::new(0, 0), Point::new(300_000, 100_000)));
            spec.snap_nm = snap;
            spec.forced_x = forced.to_vec();
            let l = Lattice::build(&spec).unwrap();
            let xs = l.xs();
            assert!(xs.contains(&forced[0]) && xs.contains(&forced[1]), "{xs:?}");
            assert!(max_gap(xs) <= i64::from(spec.base_pitch + snap), "{xs:?}");
            assert!(xs.windows(2).all(|w| w[1] - w[0] >= snap), "{xs:?}");
        }
    }

    #[test]
    fn random_forced_lines_at_the_snap_limit_keep_the_gap_bounds() {
        let mut x: u64 = 0x2545_f491_4f6c_dd1d;
        let mut next = |m: u64| {
            x = x
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (x >> 33) % m
        };
        for round in 0..300 {
            let mut spec =
                LatticeSpec::new(BBox::new(Point::new(0, 0), Point::new(2_000_000, 200_000)));
            // Up to the 0.4 · base limit `build` accepts.
            spec.snap_nm = [10_000, 15_000, 20_000][round % 3];
            let n = next(60) as usize;
            spec.forced_x = (0..n).map(|_| next(2_000_001) as Coord).collect();
            if round % 4 == 0 {
                // Lines just outside every base line, alternating sides.
                spec.forced_x = (1..40)
                    .map(|k| k * 50_000 + if k % 2 == 0 { 1 } else { -1 } * (spec.snap_nm - 1))
                    .collect();
            }
            if round % 5 == 0 {
                spec.fine_regions = vec![BBox::new(
                    Point::new(next(1_000_000) as Coord, 0),
                    Point::new(1_000_000 + next(1_000_000) as Coord, 100_000),
                )];
            }
            let l = Lattice::build(&spec).expect("a valid spec builds");
            let xs = l.xs();
            assert!(
                max_gap(xs) <= i64::from(spec.base_pitch + spec.snap_nm),
                "round {round}: {xs:?}"
            );
            assert!(
                xs.windows(2).all(|w| w[1] - w[0] >= spec.snap_nm),
                "round {round}: {xs:?}"
            );
            for f in &spec.forced_x {
                // Every forced line is kept or within `snap` of a kept line.
                let near = xs
                    .iter()
                    .any(|&v| (i64::from(v) - i64::from(*f)).abs() < i64::from(spec.snap_nm));
                assert!(xs.contains(f) || near, "round {round}: lost {f}");
            }
        }
    }

    #[test]
    fn explicit_axes_with_gaps_past_root_two_pitches_are_rejected() {
        // √2 · 50 µm ≈ 70.71 µm.
        assert!(Lattice::from_axes(vec![0, 70_000, 120_000], vec![0, 50_000], 50_000).is_some());
        assert!(Lattice::from_axes(vec![0, 70_711, 120_000], vec![0, 50_000], 50_000).is_none());
        assert!(Lattice::from_axes(vec![0, 50_000], vec![0, 80_000], 50_000).is_none());
    }

    #[test]
    fn wide_cells_never_mark_a_via_free() {
        // 70 µm cells: the cell centre is 49.5 µm from every corner, more
        // than the 35.4 µm half-diagonal the halo covers.
        let axis: Vec<Coord> = (0..11).map(|i| i * 70_000).collect();
        let wide = Lattice::from_axes(axis.clone(), axis, 50_000).unwrap();
        let narrow_axis: Vec<Coord> = (0..15).map(|i| i * 50_000).collect();
        let narrow = Lattice::from_axes(narrow_axis.clone(), narrow_axis, 50_000).unwrap();
        let profile = ProfileGeometry {
            track_width: 10_000,
            sagitta: 0,
        };
        let via = ViaGeometry {
            radius: 65_000,
            track_half_width: 5_000,
        };
        // A small obstacle at the centre of the cell next to node (5, 5):
        // no node is in its halo, yet a 65 µm via at (5, 5) overlaps it.
        let obstacle = PaintShape {
            shape: Shape::Circle(Circle::new(Point::new(385_000, 385_000), 1_000)),
            clearance: 5_000,
            kind: PaintKind::Hard,
        };
        let mut r = TileRaster::new(TileId(0), LayerId(0), ProfileId(0), wide.window());
        paint_on_lattice(
            &mut r,
            &wide,
            std::slice::from_ref(&obstacle),
            &profile,
            true,
        );
        assert!(r.halo.as_slice().iter().all(|&v| v == 0));
        derive_masks_on_lattice(&mut r, &wide, Some(via));
        assert_eq!(r.via_site.get(5, 5), VIA_CHECK);
        assert!(r.via_site.as_slice().iter().all(|&v| v != VIA_FREE));
        // On base-pitch cells an empty raster still has free interior sites.
        let mut r = TileRaster::new(TileId(0), LayerId(0), ProfileId(0), narrow.window());
        derive_masks_on_lattice(&mut r, &narrow, Some(via));
        assert_eq!(r.via_site.get(7, 7), VIA_FREE);
    }

    #[test]
    fn diagonals_land_on_the_45_degree_ray_or_do_not_exist() {
        let l = Lattice::from_axes(
            vec![0, 50_000, 70_000, 100_000],
            vec![0, 50_000, 100_000],
            50_000,
        )
        .unwrap();
        let g = LatticeGrid { lattice: &l };
        // From (0, 0) heading 7 (+x, +row): (50, 50) µm.
        let s = g.step(0, 0, 7).unwrap();
        assert_eq!((s.col, s.row, s.cost), (1, 1, 1414));
        // From x = 70 µm: x + 30 = 100 µm is a line but y + 30 is not; the
        // 50 µm ray hits x = 120, beyond the lattice: no diagonal.
        assert_eq!(g.step(2, 0, 7), None);
        // Straight steps cost by length: 50→70 µm is 400.
        let s = g.step(1, 0, 0).unwrap();
        assert_eq!((s.col, s.cost), (2, 400));
        for c in 0..l.cols() {
            for r in 0..l.rows() {
                for h in 0..8 {
                    if let Some(s) = g.step(c, r, h) {
                        let dx = (l.xs()[s.col as usize] - l.xs()[c as usize]).abs();
                        let dy = (l.ys()[s.row as usize] - l.ys()[r as usize]).abs();
                        if h % 2 == 1 {
                            assert_eq!(dx, dy);
                        } else {
                            assert_eq!(dx.min(dy), 0);
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn uniform_grid_matches_its_lattice() {
        let w = Window::new(Point::new(1_000, -2_000), 50_000, 7, 5).unwrap();
        let lat = Lattice::uniform(w);
        let g = UniformGrid::new(w);
        for c in 0..7 {
            assert_eq!(g.x(c), lat.node(c, 0).x);
        }
        assert_eq!(g.col_at_or_before(1_000 + 74_999), 1);
        assert_eq!(g.col_at_or_after(1_000 + 50_001), 2);
        assert_eq!(g.row_at_or_after(-1_000_000), 0);
        assert_eq!(g.step(0, 0, 2), None);
        assert_eq!(g.step(0, 0, 6).map(|s| (s.col, s.row)), Some((0, 1)));
    }

    #[test]
    fn via_sites_follow_the_disc_argument() {
        let w = Window::new(Point::new(0, 0), 50_000, 21, 21).unwrap();
        let mut r = TileRaster::new(TileId(0), LayerId(0), ProfileId(0), w);
        r.core.set(10, 10, 1);
        r.halo.set(10, 10, 1);
        r.halo.set(11, 10, 1);
        let via = ViaGeometry {
            radius: 150_000,
            track_half_width: 50_000,
        };
        derive_masks_uniform(&mut r, Some(via));
        assert_eq!(r.via_site.get(10, 10), VIA_NO);
        // 100 µm away: within ρ = 100 µm of the blocked node.
        assert_eq!(r.via_site.get(12, 10), VIA_NO);
        // 150 µm away: no blocked node within ρ, a halo node within ρ + hd.
        assert_eq!(r.via_site.get(13, 10), VIA_CHECK);
        assert_eq!(r.via_site.get(15, 15), VIA_FREE);
        // Next to the window edge the disc cannot be judged free.
        assert_eq!(r.via_site.get(0, 18), VIA_CHECK);
        assert_eq!(r.heading_mask.get(10, 10), 0);
        assert_eq!(r.heading_mask.get(9, 10) & 1, 0);
        assert_eq!(r.heading_mask.get(9, 10) & (1 << 4), 1 << 4);
        // Row 0 has no heading 2 (towards row -1).
        assert_eq!(r.heading_mask.get(5, 0) & (1 << 2), 0);
    }

    #[test]
    fn local_rederivation_matches_a_full_one() {
        let w = Window::new(Point::new(0, 0), 50_000, 30, 30).unwrap();
        let lat = Lattice::uniform(w);
        let via = Some(ViaGeometry {
            radius: 200_000,
            track_half_width: 50_000,
        });
        let mut r = TileRaster::new(TileId(0), LayerId(0), ProfileId(0), w);
        r.core.set(12, 12, 1);
        r.halo.set(12, 12, 1);
        derive_masks_on_lattice(&mut r, &lat, via);
        // Clear the obstacle and re-derive only around it.
        r.core.set(12, 12, 0);
        r.halo.set(12, 12, 0);
        let p = lat.node(12, 12);
        derive_masks_in_lattice(&mut r, &lat, via, BBox::new(p, p));
        let mut full = r.clone();
        derive_masks_on_lattice(&mut full, &lat, via);
        assert_eq!(r.heading_mask, full.heading_mask);
        assert_eq!(r.via_site, full.via_site);
    }

    #[test]
    fn lattice_paint_matches_uniform_paint_on_a_uniform_lattice() {
        use crate::compute::{CpuCompute, RouteCompute};
        use konnect_geom::Segment;
        let w = Window::new(Point::new(0, 0), 50_000, 41, 41).unwrap();
        let lat = Lattice::from_axes(
            (0..41).map(|i| i * 50_000).collect(),
            (0..41).map(|i| i * 50_000).collect(),
            50_000,
        )
        .unwrap();
        let shape = PaintShape {
            shape: Shape::Segment(Segment::new(
                Point::new(0, 1_000_000),
                Point::new(2_000_000, 1_300_000),
                100_000,
            )),
            clearance: 100_000,
            kind: PaintKind::Hard,
        };
        let profile = ProfileGeometry {
            track_width: 100_000,
            sagitta: 0,
        };
        let mut a = TileRaster::new(TileId(0), LayerId(0), ProfileId(0), w);
        let mut b = a.clone();
        CpuCompute::new().paint(&mut a, std::slice::from_ref(&shape), &profile);
        paint_on_lattice(&mut b, &lat, std::slice::from_ref(&shape), &profile, true);
        assert_eq!(a.core, b.core);
        assert_eq!(a.halo, b.halo);
        paint_on_lattice(&mut b, &lat, &[shape], &profile, false);
        assert!(b.core.as_slice().iter().all(|&v| v == 0));
    }
}
