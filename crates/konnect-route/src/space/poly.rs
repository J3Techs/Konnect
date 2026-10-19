//! A grid-accelerated polygon for exact containment and disc queries.
//!
//! Zone fills are large: one plane piece can hold tens of thousands of
//! vertices, and a linear point-in-polygon test per candidate via would
//! dominate S2. [`PolyIndex`] buckets the edges into a uniform grid and
//! classifies every cell that no edge comes near as wholly inside or wholly
//! outside. A query in such a cell is answered at once; a query near the
//! boundary walks the row to the first clean cell and counts crossings with
//! the few edges in between, using exact integer orientation tests, so the
//! answer is the same as a full crossing-number test.
//!
//! Correctness rests on one invariant: every edge is registered in every
//! cell its points come within one cell of (the registration dilates by a
//! cell in x and y), so a cell marked clean has no edge touching it, and
//! any point in a clean cell has the cell's containment.

use konnect_geom::{point_segment_distance_cmp, BBox, Containment, Coord, Point, Polygon};
use std::cmp::Ordering;

const OUT: u8 = 0;
const IN: u8 = 1;
const MIXED: u8 = 2;

/// Default grid size target, in cells.
pub(crate) const DEFAULT_TARGET_CELLS: usize = 2_000_000;

#[derive(Debug, Clone)]
pub(crate) struct PolyIndex {
    edges: Vec<(Point, Point)>,
    origin: Point,
    cell: i64,
    nx: usize,
    ny: usize,
    state: Vec<u8>,
    start: Vec<u32>,
    ids: Vec<u32>,
    bbox: BBox,
}

fn orient(a: Point, b: Point, c: Point) -> i128 {
    let (abx, aby) = (
        i128::from(b.x) - i128::from(a.x),
        i128::from(b.y) - i128::from(a.y),
    );
    let (acx, acy) = (
        i128::from(c.x) - i128::from(a.x),
        i128::from(c.y) - i128::from(a.y),
    );
    abx * acy - aby * acx
}

fn within_box(a: Point, b: Point, p: Point) -> bool {
    a.x.min(b.x) <= p.x && p.x <= a.x.max(b.x) && a.y.min(b.y) <= p.y && p.y <= a.y.max(b.y)
}

/// For an edge straddling the horizontal line through `p`: whether the
/// crossing lies strictly right of `p` (same convention as the geometry
/// kernel's crossing-number test).
fn crossing_right_of(a: Point, b: Point, p: Point) -> bool {
    let o = orient(a, b, p);
    if b.y > a.y {
        o > 0
    } else {
        o < 0
    }
}

impl PolyIndex {
    /// Build with roughly `target_cells` grid cells (at least a 2 µm pitch).
    pub(crate) fn with_target(poly: &Polygon, target_cells: usize) -> Self {
        let mut edges = Vec::new();
        for ring in poly.rings() {
            let n = ring.len();
            for i in 0..n {
                edges.push((ring[i], ring[(i + 1) % n]));
            }
        }
        let bbox = poly.bbox();
        let (w, h) = (bbox.width().max(1) as f64, bbox.height().max(1) as f64);
        let target = target_cells.max(16) as f64;
        // Even cell sizes keep cell centres on integer nanometres.
        let mut cell = ((w * h / target).sqrt().ceil() as i64).max(2_000);
        cell += cell & 1;
        let margin = 2 * cell;
        let origin = Point::new(
            (i64::from(bbox.min.x) - margin) as Coord,
            (i64::from(bbox.min.y) - margin) as Coord,
        );
        let nx = (bbox.width() / cell) as usize + 6;
        let ny = (bbox.height() / cell) as usize + 6;
        let mut index = PolyIndex {
            edges,
            origin,
            cell,
            nx,
            ny,
            state: vec![OUT; nx * ny],
            start: vec![0; nx * ny + 1],
            ids: Vec::new(),
            bbox,
        };
        index.register();
        index.classify();
        index
    }

    pub(crate) fn bbox(&self) -> BBox {
        self.bbox
    }

    fn col_of(&self, x: f64) -> i64 {
        ((x - f64::from(self.origin.x)) / self.cell as f64).floor() as i64
    }

    fn row_of(&self, y: f64) -> i64 {
        ((y - f64::from(self.origin.y)) / self.cell as f64).floor() as i64
    }

    /// Calls `f(cell index)` for every cell edge `e` is registered in.
    fn for_cells_of_edge(&self, e: (Point, Point), mut f: impl FnMut(usize)) {
        let (a, b) = e;
        let (ax, ay, bx, by) = (
            f64::from(a.x),
            f64::from(a.y),
            f64::from(b.x),
            f64::from(b.y),
        );
        let cell = self.cell as f64;
        let r0 = (self.row_of(ay.min(by)) - 1).max(0);
        let r1 = (self.row_of(ay.max(by)) + 1).min(self.ny as i64 - 1);
        for r in r0..=r1 {
            // The segment's x extent within this row's slab, dilated by a
            // cell in y; then dilated by a cell in x.
            let ylo = f64::from(self.origin.y) + (r as f64 - 1.0) * cell;
            let yhi = ylo + 3.0 * cell;
            let (t0, t1) = if (by - ay).abs() < 1e-9 {
                if ay < ylo || ay > yhi {
                    continue;
                }
                (0.0, 1.0)
            } else {
                let ta = (ylo - ay) / (by - ay);
                let tb = (yhi - ay) / (by - ay);
                let (lo, hi) = (ta.min(tb).max(0.0), ta.max(tb).min(1.0));
                if lo > hi {
                    continue;
                }
                (lo, hi)
            };
            let xa = ax + (bx - ax) * t0;
            let xb = ax + (bx - ax) * t1;
            let c0 = (self.col_of(xa.min(xb)) - 1).max(0);
            let c1 = (self.col_of(xa.max(xb)) + 1).min(self.nx as i64 - 1);
            for c in c0..=c1 {
                f(r as usize * self.nx + c as usize);
            }
        }
    }

    fn register(&mut self) {
        let mut counts = vec![0u32; self.nx * self.ny];
        for i in 0..self.edges.len() {
            let e = self.edges[i];
            let mut last = usize::MAX;
            self.for_cells_of_edge(e, |c| {
                if c != last {
                    counts[c] += 1;
                    last = c;
                }
            });
        }
        let mut acc = 0u32;
        for (i, n) in counts.iter().enumerate() {
            self.start[i] = acc;
            acc += n;
        }
        self.start[self.nx * self.ny] = acc;
        let mut fill = self.start.clone();
        let mut ids = vec![0u32; acc as usize];
        for i in 0..self.edges.len() {
            let e = self.edges[i];
            let mut last = usize::MAX;
            self.for_cells_of_edge(e, |c| {
                if c != last {
                    ids[fill[c] as usize] = i as u32;
                    fill[c] += 1;
                    last = c;
                }
            });
        }
        self.ids = ids;
        for (c, s) in self.state.iter_mut().enumerate() {
            if self.start[c + 1] > self.start[c] {
                *s = MIXED;
            }
        }
    }

    /// Clean cells take the parity of the crossings of the row's centre line
    /// to their right. No edge comes within a cell of a clean cell, so the
    /// floating-point crossing positions cannot be on the wrong side.
    fn classify(&mut self) {
        let half = self.cell / 2;
        let mut crossings: Vec<f64> = Vec::new();
        let mut row_edges: Vec<u32> = Vec::new();
        for r in 0..self.ny {
            let yc = i64::from(self.origin.y) + r as i64 * self.cell + half;
            row_edges.clear();
            for c in 0..self.nx {
                let idx = r * self.nx + c;
                row_edges.extend_from_slice(
                    &self.ids[self.start[idx] as usize..self.start[idx + 1] as usize],
                );
            }
            row_edges.sort_unstable();
            row_edges.dedup();
            crossings.clear();
            for &e in &row_edges {
                let (a, b) = self.edges[e as usize];
                let (ay, by) = (i64::from(a.y), i64::from(b.y));
                if (ay > yc) != (by > yc) {
                    let t = (yc - ay) as f64 / (by - ay) as f64;
                    crossings.push(f64::from(a.x) + t * f64::from(b.x - a.x));
                }
            }
            crossings.sort_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal));
            // Walk cells left to right, counting crossings passed.
            let mut passed = 0usize;
            for c in 0..self.nx {
                let idx = r * self.nx + c;
                let xc = (i64::from(self.origin.x) + c as i64 * self.cell + half) as f64;
                while passed < crossings.len() && crossings[passed] < xc {
                    passed += 1;
                }
                if self.state[idx] != MIXED {
                    let right = crossings.len() - passed;
                    self.state[idx] = if right % 2 == 1 { IN } else { OUT };
                }
            }
        }
    }

    fn cell_xy(&self, p: Point) -> Option<(usize, usize)> {
        let c = (i64::from(p.x) - i64::from(self.origin.x)).div_euclid(self.cell);
        let r = (i64::from(p.y) - i64::from(self.origin.y)).div_euclid(self.cell);
        (c >= 0 && r >= 0 && (c as usize) < self.nx && (r as usize) < self.ny)
            .then_some((c as usize, r as usize))
    }

    fn cell_edges(&self, idx: usize) -> &[u32] {
        &self.ids[self.start[idx] as usize..self.start[idx + 1] as usize]
    }

    /// Exact containment of `p` in the polygon's region.
    pub(crate) fn contains(&self, p: Point) -> Containment {
        let Some((c, r)) = self.cell_xy(p) else {
            return Containment::Outside;
        };
        let row = r * self.nx;
        match self.state[row + c] {
            IN => return Containment::Inside,
            OUT => return Containment::Outside,
            _ => {}
        }
        // Walk right to the first clean cell; its left boundary point `q` on
        // p's row touches no edge, so its containment is the cell's.
        let mut k = c + 1;
        while k < self.nx && self.state[row + k] == MIXED {
            k += 1;
        }
        let (q, q_state) = if k < self.nx {
            (
                Point::new(
                    (i64::from(self.origin.x) + k as i64 * self.cell) as Coord,
                    p.y,
                ),
                self.state[row + k],
            )
        } else {
            // Unreachable with the grid margin; fall back to a full test.
            return self.contains_linear(p);
        };
        // Each edge once: within a row an edge is registered in a run of
        // consecutive cells, so it is new at cell `kk` unless the cell to
        // its left (inside the walk) holds it too. Cell lists are sorted.
        let mut parity = false;
        for kk in c..k {
            let left = (kk > c).then(|| self.cell_edges(row + kk - 1));
            for &e in self.cell_edges(row + kk) {
                if left.is_some_and(|l| l.binary_search(&e).is_ok()) {
                    continue;
                }
                let (a, b) = self.edges[e as usize];
                if orient(a, b, p) == 0 && within_box(a, b, p) {
                    return Containment::Boundary;
                }
                if (a.y > p.y) != (b.y > p.y)
                    && crossing_right_of(a, b, p)
                    && !crossing_right_of(a, b, q)
                {
                    parity = !parity;
                }
            }
        }
        let inside = (q_state == IN) != parity;
        if inside {
            Containment::Inside
        } else {
            Containment::Outside
        }
    }

    fn contains_linear(&self, p: Point) -> Containment {
        let mut inside = false;
        for &(a, b) in &self.edges {
            if orient(a, b, p) == 0 && within_box(a, b, p) {
                return Containment::Boundary;
            }
            if (a.y > p.y) != (b.y > p.y) && crossing_right_of(a, b, p) {
                inside = !inside;
            }
        }
        if inside {
            Containment::Inside
        } else {
            Containment::Outside
        }
    }

    /// Whether every edge is at least `r` from `p` (`Equal` counts as clear).
    fn edges_clear(&self, p: Point, r: Coord) -> bool {
        if r <= 0 {
            return true;
        }
        let lo = Point::new(p.x.saturating_sub(r), p.y.saturating_sub(r));
        let hi = Point::new(p.x.saturating_add(r), p.y.saturating_add(r));
        let (Some((c0, r0)), Some((c1, r1))) = (self.cell_xy_clamped(lo), self.cell_xy_clamped(hi))
        else {
            return true;
        };
        for row in r0..=r1 {
            for col in c0..=c1 {
                let idx = row * self.nx + col;
                if self.state[idx] != MIXED {
                    continue;
                }
                for &e in self.cell_edges(idx) {
                    let (a, b) = self.edges[e as usize];
                    if point_segment_distance_cmp(p, a, b, i64::from(r)) == Ordering::Less {
                        return false;
                    }
                }
            }
        }
        true
    }

    fn cell_xy_clamped(&self, p: Point) -> Option<(usize, usize)> {
        let c = (i64::from(p.x) - i64::from(self.origin.x)).div_euclid(self.cell);
        let r = (i64::from(p.y) - i64::from(self.origin.y)).div_euclid(self.cell);
        let c = c.clamp(0, self.nx as i64 - 1) as usize;
        let r = r.clamp(0, self.ny as i64 - 1) as usize;
        Some((c, r))
    }

    /// Whether the closed disc of radius `r` around `p` lies in the region
    /// (touching the boundary from inside counts).
    pub(crate) fn disc_inside(&self, p: Point, r: Coord) -> bool {
        match self.contains(p) {
            Containment::Inside => self.edges_clear(p, r),
            Containment::Boundary => r <= 0,
            Containment::Outside => false,
        }
    }

    /// Whether the disc of radius `r` around `p` stays out of the region's
    /// interior (touching from outside counts as out).
    pub(crate) fn disc_outside(&self, p: Point, r: Coord) -> bool {
        match self.contains(p) {
            Containment::Outside => self.edges_clear(p, r),
            Containment::Boundary => r <= 0,
            Containment::Inside => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use konnect_geom::point_in_polygon;

    fn ring(pts: &[(i32, i32)]) -> Vec<Point> {
        pts.iter().map(|&(x, y)| Point::new(x, y)).collect()
    }

    fn comb() -> Polygon {
        // A comb with narrow teeth and a hole, to exercise mixed cells.
        let outline = ring(&[
            (0, 0),
            (100_000, 0),
            (100_000, 30_000),
            (80_000, 30_000),
            (80_000, 10_000),
            (70_000, 10_000),
            (70_000, 30_000),
            (50_000, 30_000),
            (50_000, 10_000),
            (45_000, 10_000),
            (45_000, 31_000),
            (0, 60_000),
        ]);
        let hole = ring(&[
            (10_000, 5_000),
            (20_000, 5_000),
            (20_000, 15_000),
            (10_000, 15_000),
        ]);
        Polygon::new(outline, vec![hole]).unwrap()
    }

    #[test]
    fn grid_containment_matches_the_linear_test() {
        let poly = comb();
        // A coarse grid so most queries hit mixed cells.
        for target in [16, 400, 100_000] {
            let idx = PolyIndex::with_target(&poly, target);
            let mut seed = 12345u64;
            for _ in 0..20_000 {
                seed = seed
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                let x = ((seed >> 33) % 120_000) as i32 - 10_000;
                let y = ((seed >> 13) % 80_000) as i32 - 10_000;
                let p = Point::new(x, y);
                assert_eq!(
                    idx.contains(p),
                    point_in_polygon(p, &poly),
                    "{p:?} at {target}"
                );
            }
            // Vertices and edge points are on the boundary.
            for p in [
                Point::new(0, 0),
                Point::new(50_000, 20_000),
                Point::new(15_000, 5_000),
            ] {
                assert_eq!(idx.contains(p), Containment::Boundary);
            }
        }
    }

    #[test]
    fn disc_queries_respect_edges_and_holes() {
        let poly = comb();
        let idx = PolyIndex::with_target(&poly, 400);
        // Centre of the solid strip below the teeth: 5 µm from the bottom.
        assert!(idx.disc_inside(Point::new(60_000, 5_000), 5_000));
        assert!(!idx.disc_inside(Point::new(60_000, 5_000), 5_001));
        // Next to the hole.
        assert!(idx.disc_inside(Point::new(25_000, 10_000), 5_000));
        assert!(!idx.disc_inside(Point::new(25_000, 10_000), 5_001));
        // Inside the hole is outside the region.
        assert!(idx.disc_outside(Point::new(15_000, 10_000), 5_000));
        assert!(!idx.disc_outside(Point::new(15_000, 10_000), 5_001));
        assert!(!idx.disc_inside(Point::new(15_000, 10_000), 1));
        // In the notch between two teeth.
        assert!(idx.disc_outside(Point::new(75_000, 25_000), 5_000));
        assert!(!idx.disc_outside(Point::new(75_000, 25_000), 5_001));
        assert!(idx.disc_inside(Point::new(60_000, 20_000), 10_000));
    }
}
