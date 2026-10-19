//! Grid layout (bbox, refinement) and the planar topology every stage of
//! the global router walks.

use super::{GCellGrid, GlobalSettings};
use crate::board::RouteBoard;
use konnect_geom::{BBox, Coord, Point};

/// Base cells with no refined index.
pub(crate) const NONE: u32 = u32::MAX;

/// Most base cells along one side of the grid.
const MAX_SIDE: i64 = 65_000;

/// Most search nodes (layer slots × planar cells, fine cells included) a
/// grid may hold. Keeps GCell ids, node and resource numbers far inside
/// `u32`, and per-thread search scratch near 0.25 GB.
pub(crate) const MAX_NODES: u64 = 1 << 23;

/// Most fine cells per side of a refined base cell.
pub(crate) const MAX_REFINE: u32 = 16;

/// Whether `grid` stays inside the size limits the router's `u32` ids and
/// memory budget assume: what [`lay_out`] builds always does; a grid read
/// from elsewhere may not.
pub(crate) fn fits(grid: &GCellGrid) -> bool {
    let side = |n: u32| n >= 1 && i64::from(n) <= MAX_SIDE;
    if !side(grid.cols) || !side(grid.rows) || grid.pitch <= 0 {
        return false;
    }
    let base = u64::from(grid.cols) * u64::from(grid.rows);
    let k = u64::from(grid.refine_factor());
    if k > u64::from(MAX_REFINE) {
        return false;
    }
    let fine = if k > 1 {
        let ascending = grid.refined.windows(2).all(|w| w[0] < w[1]);
        if !ascending || grid.refined.last().is_some_and(|&b| u64::from(b) >= base) {
            return false;
        }
        grid.refined.len() as u64 * k * k
    } else {
        0
    };
    u64::from(grid.layer_slots()) * (base + fine) <= MAX_NODES
}

/// Cells needed to cover `w` nm at `pitch`.
fn span(w: i64, pitch: i64) -> u32 {
    ((w.max(1) + pitch - 1) / pitch).max(1) as u32
}

/// The board area to grid: the outline's box, else every pad's.
fn board_bbox(board: &RouteBoard) -> BBox {
    if let Some(b) = board.outline.bbox {
        return b;
    }
    let mut b = board
        .outline
        .edges
        .iter()
        .map(|e| e.bbox())
        .reduce(BBox::union);
    for pad in &board.pads {
        if let Some(pb) = pad.bbox() {
            b = Some(b.map_or(pb, |x| x.union(pb)));
        }
    }
    b.unwrap_or_else(|| BBox::point(Point::new(0, 0)))
}

/// Lay out the grid: the base cells over the board and the refined set.
pub(crate) fn lay_out(board: &RouteBoard, settings: &GlobalSettings) -> GCellGrid {
    let area = board_bbox(board);
    // A pitch too fine for the board is coarsened: cell coordinates are
    // u16, so neither side may exceed MAX_SIDE cells, and the whole grid
    // must fit the node budget on every routable layer.
    let extent = area.width().max(area.height()).max(1);
    let min_pitch = (extent + MAX_SIDE - 1) / MAX_SIDE;
    let slots = u64::from(board.routable.count().max(1));
    let mut pitch = i64::from(settings.gcell_pitch.max(1)).max(min_pitch);
    loop {
        let nodes =
            u64::from(span(area.width(), pitch)) * u64::from(span(area.height(), pitch)) * slots;
        if nodes <= MAX_NODES {
            break;
        }
        let grow = (pitch as f64 * (nodes as f64 / MAX_NODES as f64).sqrt()).ceil() as i64;
        pitch = grow.max(pitch + 1);
    }
    let pitch = pitch.min(i64::from(Coord::MAX)) as Coord;
    let cols = span(area.width(), i64::from(pitch));
    let rows = span(area.height(), i64::from(pitch));
    let max = Point::new(
        (i64::from(area.min.x) + i64::from(cols) * i64::from(pitch)).min(i64::from(i32::MAX))
            as Coord,
        (i64::from(area.min.y) + i64::from(rows) * i64::from(pitch)).min(i64::from(i32::MAX))
            as Coord,
    );
    let mut grid = GCellGrid {
        bbox: BBox::new(area.min, max),
        pitch,
        cols,
        rows,
        layers: board.routable,
        fine_pitch: pitch,
        refined: Vec::new(),
        capacity: Vec::new(),
    };
    let fine = settings.fine_pitch;
    if settings.refine_min_pads == 0
        || pitch != settings.gcell_pitch
        || fine <= 0
        || fine >= pitch
        || pitch % fine != 0
        || (pitch / fine) as u32 > MAX_REFINE
    {
        return grid;
    }
    // Pad centres per base cell, counting pads with copper on a routable
    // layer.
    let mut count = vec![0u32; (cols * rows) as usize];
    for pad in &board.pads {
        if !pad.layers.intersects(board.routable) {
            continue;
        }
        let Some(b) = pad.bbox() else { continue };
        let c = b.center();
        let dx = i64::from(c.x) - i64::from(area.min.x);
        let dy = i64::from(c.y) - i64::from(area.min.y);
        if dx < 0 || dy < 0 {
            continue;
        }
        let (col, row) = (dx / i64::from(pitch), dy / i64::from(pitch));
        if col < i64::from(cols) && row < i64::from(rows) {
            count[(row as u32 * cols + col as u32) as usize] += 1;
        }
    }
    let mut refined = Vec::new();
    for row in 0..rows {
        for col in 0..cols {
            let mut sum = 0;
            for r in row.saturating_sub(1)..=(row + 1).min(rows - 1) {
                for c in col.saturating_sub(1)..=(col + 1).min(cols - 1) {
                    sum += count[(r * cols + c) as usize];
                }
            }
            if sum >= settings.refine_min_pads {
                refined.push(row * cols + col);
            }
        }
    }
    // Refinement that would break the node budget is dropped whole (a
    // partial set would depend on scan order, not density).
    let k = u64::from((pitch / fine) as u32);
    let fine_cells = refined.len() as u64 * k * k;
    if (u64::from(cols) * u64::from(rows) + fine_cells) * slots > MAX_NODES {
        return grid;
    }
    grid.fine_pitch = fine;
    grid.refined = refined;
    grid
}

/// The planar cell graph shared by every layer, in structure-of-arrays
/// form. Cell `c < base` is base cell `c`; `c ≥ base` is fine cell
/// `c − base` (refined index · k² + sub). Search node = `slot · nc + c`.
#[derive(Debug, Clone)]
pub(crate) struct Topology {
    pub cols: u32,
    pub rows: u32,
    pub k: u32,
    pub pitch: i64,
    pub fsize: i64,
    pub origin: Point,
    pub slots: u32,
    pub base: u32,
    pub nc: u32,
    pub refine_index: Vec<u32>,
    pub alive: Vec<bool>,
    pub x0: Vec<i32>,
    pub y0: Vec<i32>,
    pub size: Vec<i32>,
    /// Base column and row of each cell (a fine cell's parent).
    pub col: Vec<u16>,
    pub row: Vec<u16>,
    pub adj_off: Vec<u32>,
    pub adj_cell: Vec<u32>,
    pub adj_edge: Vec<u32>,
    /// Undirected edges: `a` is west of (or north of) `b`.
    pub edge_a: Vec<u32>,
    pub edge_b: Vec<u32>,
    pub edge_horiz: Vec<bool>,
    /// Manhattan distance between the two cell centres, mm.
    pub edge_len: Vec<f32>,
    pub edges: u32,
}

impl Topology {
    pub(crate) fn new(grid: &GCellGrid) -> Topology {
        let k = grid.refine_factor();
        let (cols, rows) = (grid.cols, grid.rows);
        let base = cols * rows;
        let refined: &[u32] = if k > 1 { &grid.refined } else { &[] };
        let nc = base + refined.len() as u32 * k * k;
        let pitch = i64::from(grid.pitch);
        let fsize = pitch / i64::from(k);
        let origin = grid.bbox.min;
        let mut refine_index = vec![NONE; base as usize];
        for (i, &b) in refined.iter().enumerate() {
            refine_index[b as usize] = i as u32;
        }
        let n = nc as usize;
        let (mut x0, mut y0, mut size) = (vec![0i32; n], vec![0i32; n], vec![0i32; n]);
        let (mut col, mut row) = (vec![0u16; n], vec![0u16; n]);
        let mut alive = vec![true; n];
        for b in 0..base {
            let (c, r) = (b % cols, b / cols);
            x0[b as usize] = (i64::from(origin.x) + i64::from(c) * pitch) as i32;
            y0[b as usize] = (i64::from(origin.y) + i64::from(r) * pitch) as i32;
            size[b as usize] = pitch as i32;
            col[b as usize] = c as u16;
            row[b as usize] = r as u16;
            alive[b as usize] = refine_index[b as usize] == NONE;
        }
        for (i, &b) in refined.iter().enumerate() {
            for sub in 0..k * k {
                let f = (base + i as u32 * k * k + sub) as usize;
                let (sx, sy) = (i64::from(sub % k), i64::from(sub / k));
                x0[f] = (i64::from(x0[b as usize]) + sx * fsize) as i32;
                y0[f] = (i64::from(y0[b as usize]) + sy * fsize) as i32;
                size[f] = fsize as i32;
                col[f] = col[b as usize];
                row[f] = row[b as usize];
            }
        }
        let mut t = Topology {
            cols,
            rows,
            k,
            pitch,
            fsize,
            origin,
            slots: grid.layer_slots(),
            base,
            nc,
            refine_index,
            alive,
            x0,
            y0,
            size,
            col,
            row,
            adj_off: Vec::new(),
            adj_cell: Vec::new(),
            adj_edge: Vec::new(),
            edge_a: Vec::new(),
            edge_b: Vec::new(),
            edge_horiz: Vec::new(),
            edge_len: Vec::new(),
            edges: 0,
        };
        t.build_edges();
        t
    }

    fn build_edges(&mut self) {
        let (gx1, gy1) = (
            i64::from(self.origin.x) + self.pitch * i64::from(self.cols),
            i64::from(self.origin.y) + self.pitch * i64::from(self.rows),
        );
        for c in 0..self.nc {
            if !self.alive[c as usize] {
                continue;
            }
            let (x0, y0, s) = (
                i64::from(self.x0[c as usize]),
                i64::from(self.y0[c as usize]),
                i64::from(self.size[c as usize]),
            );
            let steps = (s / self.fsize).max(1);
            // East neighbours.
            if x0 + s < gx1 {
                let mut last = NONE;
                for j in 0..steps {
                    let y = y0 + j * self.fsize;
                    if let Some(n) = self.locate_xy(x0 + s, y) {
                        if n != last {
                            self.push_edge(c, n, true);
                            last = n;
                        }
                    }
                }
            }
            // South neighbours (Y down).
            if y0 + s < gy1 {
                let mut last = NONE;
                for j in 0..steps {
                    let x = x0 + j * self.fsize;
                    if let Some(n) = self.locate_xy(x, y0 + s) {
                        if n != last {
                            self.push_edge(c, n, false);
                            last = n;
                        }
                    }
                }
            }
        }
        self.edges = self.edge_a.len() as u32;
        let mut deg = vec![0u32; self.nc as usize + 1];
        for e in 0..self.edges as usize {
            deg[self.edge_a[e] as usize] += 1;
            deg[self.edge_b[e] as usize] += 1;
        }
        let mut off = vec![0u32; self.nc as usize + 1];
        for c in 0..self.nc as usize {
            off[c + 1] = off[c] + deg[c];
        }
        let total = off[self.nc as usize] as usize;
        let mut fill = off.clone();
        let mut adj_cell = vec![0u32; total];
        let mut adj_edge = vec![0u32; total];
        for e in 0..self.edges {
            let (a, b) = (self.edge_a[e as usize], self.edge_b[e as usize]);
            for (from, to) in [(a, b), (b, a)] {
                let slot = &mut fill[from as usize];
                adj_cell[*slot as usize] = to;
                adj_edge[*slot as usize] = e;
                *slot += 1;
            }
        }
        self.adj_off = off;
        self.adj_cell = adj_cell;
        self.adj_edge = adj_edge;
    }

    fn push_edge(&mut self, a: u32, b: u32, horiz: bool) {
        let (ax, ay) = self.center(a);
        let (bx, by) = self.center(b);
        let len = ((i64::from(ax) - i64::from(bx)).abs() + (i64::from(ay) - i64::from(by)).abs())
            as f64
            / 1e6;
        self.edge_a.push(a);
        self.edge_b.push(b);
        self.edge_horiz.push(horiz);
        self.edge_len.push(len as f32);
    }

    /// The cell holding `(x, y)`, half-open on every side.
    pub(crate) fn locate_xy(&self, x: i64, y: i64) -> Option<u32> {
        let dx = x - i64::from(self.origin.x);
        let dy = y - i64::from(self.origin.y);
        if dx < 0 || dy < 0 {
            return None;
        }
        let (col, row) = (dx / self.pitch, dy / self.pitch);
        if col >= i64::from(self.cols) || row >= i64::from(self.rows) {
            return None;
        }
        let b = row as u32 * self.cols + col as u32;
        let ri = self.refine_index[b as usize];
        if ri == NONE {
            return Some(b);
        }
        let k = i64::from(self.k);
        let sx = ((dx - col * self.pitch) / self.fsize).min(k - 1);
        let sy = ((dy - row * self.pitch) / self.fsize).min(k - 1);
        Some(self.base + ri * self.k * self.k + (sy * k + sx) as u32)
    }

    pub(crate) fn locate(&self, p: Point) -> Option<u32> {
        self.locate_xy(i64::from(p.x), i64::from(p.y))
    }

    /// The centre of cell `c`, rounded down.
    pub(crate) fn center(&self, c: u32) -> (i32, i32) {
        let i = c as usize;
        let h = self.size[i] / 2;
        (self.x0[i] + h, self.y0[i] + h)
    }

    pub(crate) fn node(&self, slot: u32, c: u32) -> u32 {
        slot * self.nc + c
    }

    pub(crate) fn split(&self, node: u32) -> (u32, u32) {
        (node / self.nc, node % self.nc)
    }

    pub(crate) fn nodes(&self) -> usize {
        self.slots as usize * self.nc as usize
    }

    /// The frozen GCell id of planar cell `c` on `slot`.
    pub(crate) fn gcell_id(&self, slot: u32, c: u32) -> u32 {
        if c < self.base {
            slot * self.base + c
        } else {
            let per = self.nc - self.base;
            self.slots * self.base + slot * per + (c - self.base)
        }
    }

    pub(crate) fn planar_res(&self, slot: u32, e: u32) -> u32 {
        slot * self.edges + e
    }

    pub(crate) fn via_res(&self, c: u32) -> u32 {
        self.slots * self.edges + c
    }

    pub(crate) fn resources(&self) -> usize {
        self.slots as usize * self.edges as usize + self.nc as usize
    }

    /// The edge joining cells `a` and `b`.
    pub(crate) fn edge_between(&self, a: u32, b: u32) -> Option<u32> {
        let (s, e) = (
            self.adj_off[a as usize] as usize,
            self.adj_off[a as usize + 1] as usize,
        );
        (s..e)
            .find(|&i| self.adj_cell[i] == b)
            .map(|i| self.adj_edge[i])
    }

    /// Whether cell `c` lies in the base-cell window `w = (c0, r0, c1, r1)`.
    #[inline]
    pub(crate) fn in_window(&self, c: u32, w: &Win) -> bool {
        let (col, row) = (self.col[c as usize], self.row[c as usize]);
        col >= w.c0 && col <= w.c1 && row >= w.r0 && row <= w.r1
    }
}

/// An inclusive window of base cells.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Win {
    pub c0: u16,
    pub r0: u16,
    pub c1: u16,
    pub r1: u16,
}

impl Win {
    pub(crate) fn full(t: &Topology) -> Win {
        Win {
            c0: 0,
            r0: 0,
            c1: (t.cols - 1) as u16,
            r1: (t.rows - 1) as u16,
        }
    }

    pub(crate) fn nodes(&self, slots: u32) -> u64 {
        (u64::from(self.c1 - self.c0) + 1) * (u64::from(self.r1 - self.r0) + 1) * u64::from(slots)
    }
}
