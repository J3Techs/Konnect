//! Floating-point geometry for the analyses (mm): rings and their repair,
//! segment distances, a bucket grid and a coverage rasterizer. Exact
//! legality stays with `konnect-geom`; these measure.

use konnect_geom::{Point, Polygon, Shape};
use std::collections::HashMap;

pub(crate) type P = [f64; 2];

pub(crate) fn sub(a: P, b: P) -> P {
    [a[0] - b[0], a[1] - b[1]]
}

pub(crate) fn dist(a: P, b: P) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

pub(crate) fn mid(a: P, b: P) -> P {
    [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0]
}

pub(crate) fn pmm(p: Point) -> P {
    let (x, y) = p.to_mm();
    [x, y]
}

/// The closest point to `p` on segment `a`–`b`, and its parameter.
pub(crate) fn closest_on_segment(p: P, a: P, b: P) -> (P, f64) {
    let d = sub(b, a);
    let l2 = d[0] * d[0] + d[1] * d[1];
    if l2 == 0.0 {
        return (a, 0.0);
    }
    let t = (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / l2).clamp(0.0, 1.0);
    ([a[0] + t * d[0], a[1] + t * d[1]], t)
}

fn cross(a: P, b: P) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

fn segments_cross(a: P, b: P, c: P, d: P) -> bool {
    let d1 = cross(sub(b, a), sub(c, a));
    let d2 = cross(sub(b, a), sub(d, a));
    let d3 = cross(sub(d, c), sub(a, c));
    let d4 = cross(sub(d, c), sub(b, c));
    ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
}

/// Closest points between segments `a`–`b` and `c`–`d`: (distance, on
/// the first, on the second, parameter on the first, on the second).
pub(crate) fn segment_segment(a: P, b: P, c: P, d: P) -> (f64, P, P, f64, f64) {
    if segments_cross(a, b, c, d) {
        // Their intersection.
        let r = sub(b, a);
        let s = sub(d, c);
        let den = cross(r, s);
        let t = cross(sub(c, a), s) / den;
        let u = cross(sub(c, a), r) / den;
        let p = [a[0] + t * r[0], a[1] + t * r[1]];
        return (0.0, p, p, t, u);
    }
    let mut best = (f64::INFINITY, a, c, 0.0, 0.0);
    let (q, u) = closest_on_segment(a, c, d);
    let da = dist(a, q);
    if da < best.0 {
        best = (da, a, q, 0.0, u);
    }
    let (q, u) = closest_on_segment(b, c, d);
    let db = dist(b, q);
    if db < best.0 {
        best = (db, b, q, 1.0, u);
    }
    let (q, t) = closest_on_segment(c, a, b);
    let dc = dist(c, q);
    if dc < best.0 {
        best = (dc, q, c, t, 0.0);
    }
    let (q, t) = closest_on_segment(d, a, b);
    let dd = dist(d, q);
    if dd < best.0 {
        best = (dd, q, d, t, 1.0);
    }
    best
}

/// Even-odd containment over a set of rings (holes are just more rings).
pub(crate) fn inside_rings(p: P, rings: &[Vec<P>]) -> bool {
    let mut inside = false;
    for ring in rings {
        let n = ring.len();
        let mut j = n - 1;
        for i in 0..n {
            let (a, b) = (ring[i], ring[j]);
            if (a[1] > p[1]) != (b[1] > p[1])
                && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0]
            {
                inside = !inside;
            }
            j = i;
        }
    }
    inside
}

/// Twice the signed area of a ring.
pub(crate) fn ring_area2(ring: &[P]) -> f64 {
    let n = ring.len();
    (0..n)
        .map(|i| cross(ring[i], ring[(i + 1) % n]))
        .sum::<f64>()
}

/// The rings' enclosed area, mm² (even-odd: outline minus holes).
pub(crate) fn rings_area(rings: &[Vec<P>]) -> f64 {
    // Signs differ between outline and holes in KiCad's output, but a
    // repaired fill can come back with any orientation: the largest ring is
    // the outline and the rest are holes.
    let mut areas: Vec<f64> = rings.iter().map(|r| ring_area2(r).abs() / 2.0).collect();
    areas.sort_by(|a, b| b.total_cmp(a));
    match areas.split_first() {
        Some((outline, holes)) => outline - holes.iter().sum::<f64>(),
        None => 0.0,
    }
}

/// Split a KiCad fill polygon (one ring whose holes are joined to the
/// outline by zero-width bridges) back into its outline and hole rings: every
/// edge whose exact reverse is also an edge is a bridge and is dropped, and
/// what remains is walked into closed rings.
/// A directed edge between two vertices keyed to 0.1 µm.
type EdgeKey = ((i64, i64), (i64, i64));

pub(crate) fn unfracture(points: &[(f64, f64)]) -> Vec<Vec<P>> {
    // Vertices are keyed to 0.1 µm: KiCad's fills carry near-duplicates a
    // nanometre apart, which would otherwise read as zero-width necks.
    let key = |p: (f64, f64)| ((p.0 * 1e4).round() as i64, (p.1 * 1e4).round() as i64);
    let n = points.len();
    let mut edges: Vec<EdgeKey> = Vec::with_capacity(n);
    for i in 0..n {
        let (a, b) = (key(points[i]), key(points[(i + 1) % n]));
        if a != b {
            edges.push((a, b));
        }
    }
    // Each edge met in both directions is a bridge (or a zero-width slit):
    // drop as many of each direction as the pair has.
    let mut count: HashMap<EdgeKey, i32> = HashMap::new();
    for e in &edges {
        *count.entry(*e).or_default() += 1;
    }
    let mut remove: HashMap<EdgeKey, i32> = HashMap::new();
    for (&(a, b), &c) in &count {
        let r = count.get(&(b, a)).copied().unwrap_or(0);
        if r > 0 {
            remove.insert((a, b), c.min(r));
        }
    }
    let keep: Vec<EdgeKey> = edges
        .iter()
        .filter(|e| match remove.get_mut(e) {
            Some(n) if *n > 0 => {
                *n -= 1;
                false
            }
            _ => true,
        })
        .copied()
        .collect();
    // Walk the remaining edges into rings.
    let mut out_from: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
    for (i, e) in keep.iter().enumerate() {
        out_from.entry(e.0).or_default().push(i);
    }
    let mut used = vec![false; keep.len()];
    let mut rings = Vec::new();
    for start in 0..keep.len() {
        if used[start] {
            continue;
        }
        let mut ring = Vec::new();
        let mut cur = start;
        loop {
            used[cur] = true;
            let (a, b) = keep[cur];
            ring.push([a.0 as f64 / 1e4, a.1 as f64 / 1e4]);
            let next = out_from
                .get(&b)
                .and_then(|list| list.iter().copied().find(|&i| !used[i]));
            match next {
                Some(nx) => cur = nx,
                None => break,
            }
        }
        if ring.len() >= 3 && ring_area2(&ring).abs() > 1e-12 {
            rings.push(ring);
        }
    }
    rings
}

/// A uniform bucket grid over items with bounding boxes.
pub(crate) struct Grid<T: Copy> {
    cell: f64,
    map: HashMap<(i32, i32), Vec<T>>,
}

impl<T: Copy> Grid<T> {
    pub(crate) fn new(cell: f64) -> Self {
        Grid {
            cell: cell.max(1e-3),
            map: HashMap::new(),
        }
    }

    fn range(&self, lo: P, hi: P) -> (i32, i32, i32, i32) {
        let f = |v: f64| (v / self.cell).floor().clamp(-1e9, 1e9) as i32;
        (f(lo[0]), f(lo[1]), f(hi[0]), f(hi[1]))
    }

    pub(crate) fn insert(&mut self, lo: P, hi: P, item: T) {
        let (x0, y0, x1, y1) = self.range(lo, hi);
        // Very large items (a whole plane) would fill millions of cells:
        // callers keep those out of a grid.
        for x in x0..=x1 {
            for y in y0..=y1 {
                self.map.entry((x, y)).or_default().push(item);
            }
        }
    }

    /// Every item whose cells meet the box (an item can repeat).
    pub(crate) fn query(&self, lo: P, hi: P, mut f: impl FnMut(T)) {
        let (x0, y0, x1, y1) = self.range(lo, hi);
        for x in x0..=x1 {
            for y in y0..=y1 {
                if let Some(v) = self.map.get(&(x, y)) {
                    for &t in v {
                        f(t);
                    }
                }
            }
        }
    }
}

/// The rings of a `konnect-geom` polygon in mm.
pub(crate) fn polygon_rings(p: &Polygon) -> Vec<Vec<P>> {
    p.rings()
        .map(|r| r.iter().map(|&q| pmm(q)).collect())
        .collect()
}

/// A shape's outline as rings in mm (circles and rounded corners
/// approximated with `segments` per full turn): used to rasterize pads.
pub(crate) fn shape_rings(s: &Shape, segments: usize) -> Vec<Vec<P>> {
    let circle = |c: P, r: f64| -> Vec<P> {
        (0..segments)
            .map(|k| {
                let a = std::f64::consts::TAU * k as f64 / segments as f64;
                [c[0] + r * a.cos(), c[1] + r * a.sin()]
            })
            .collect()
    };
    match s {
        Shape::Circle(c) => vec![circle(pmm(c.center), f64::from(c.radius) / 1e6)],
        Shape::Segment(g) => {
            let (a, b, r) = (pmm(g.a), pmm(g.b), f64::from(g.width) / 2e6);
            vec![capsule(a, b, r, segments)]
        }
        Shape::ConvexPoly(cp) => {
            let pts: Vec<P> = cp.points().iter().map(|&q| pmm(q)).collect();
            let r = f64::from(cp.radius()) / 1e6;
            if r <= 0.0 {
                return vec![pts];
            }
            // Minkowski sum with a disc: each vertex becomes an arc.
            let n = pts.len();
            let ccw = ring_area2(&pts) > 0.0;
            let mut out = Vec::new();
            for i in 0..n {
                let prev = pts[(i + n - 1) % n];
                let cur = pts[i];
                let next = pts[(i + 1) % n];
                let normal = |a: P, b: P| {
                    let d = sub(b, a);
                    let l = d[0].hypot(d[1]).max(1e-12);
                    if ccw {
                        [d[1] / l, -d[0] / l]
                    } else {
                        [-d[1] / l, d[0] / l]
                    }
                };
                let n0 = normal(prev, cur);
                let n1 = normal(cur, next);
                let a0 = n0[1].atan2(n0[0]);
                let mut a1 = n1[1].atan2(n1[0]);
                if ccw {
                    while a1 > a0 {
                        a1 -= std::f64::consts::TAU;
                    }
                } else {
                    while a1 < a0 {
                        a1 += std::f64::consts::TAU;
                    }
                }
                let steps = (((a1 - a0).abs() / std::f64::consts::TAU) * segments as f64)
                    .ceil()
                    .max(1.0) as usize;
                for k in 0..=steps {
                    let a = a0 + (a1 - a0) * k as f64 / steps as f64;
                    out.push([cur[0] + r * a.cos(), cur[1] + r * a.sin()]);
                }
            }
            vec![out]
        }
        Shape::Polygon(p) => polygon_rings(p),
    }
}

/// A stadium around `a`–`b` of radius `r`.
pub(crate) fn capsule(a: P, b: P, r: f64, segments: usize) -> Vec<P> {
    let d = sub(b, a);
    let base = d[1].atan2(d[0]);
    let half = segments / 2;
    let mut out = Vec::with_capacity(segments + 2);
    for k in 0..=half {
        let t = base + std::f64::consts::FRAC_PI_2 + std::f64::consts::PI * k as f64 / half as f64;
        out.push([a[0] + r * t.cos(), a[1] + r * t.sin()]);
    }
    for k in 0..=half {
        let t = base - std::f64::consts::FRAC_PI_2 + std::f64::consts::PI * k as f64 / half as f64;
        out.push([b[0] + r * t.cos(), b[1] + r * t.sin()]);
    }
    out
}

/// A coverage raster: each cell holds the fraction of its area inside the
/// rasterized regions (0..1), the union of everything filled into it.
#[derive(Debug, Clone)]
pub(crate) struct Raster {
    pub x0: f64,
    pub y0: f64,
    pub h: f64,
    pub nx: usize,
    pub ny: usize,
    pub cov: Vec<f32>,
}

impl Raster {
    /// A raster of step `h` covering `[lo, hi]`, aligned to multiples of `h`
    /// so rasters of different layers share cell centres.
    pub(crate) fn new(lo: P, hi: P, h: f64) -> Raster {
        let x0 = (lo[0] / h).floor() * h;
        let y0 = (lo[1] / h).floor() * h;
        let nx = (((hi[0] - x0) / h).ceil() as usize).max(1);
        let ny = (((hi[1] - y0) / h).ceil() as usize).max(1);
        Raster {
            x0,
            y0,
            h,
            nx,
            ny,
            cov: vec![0.0; nx * ny],
        }
    }

    pub(crate) fn idx(&self, i: usize, j: usize) -> usize {
        j * self.nx + i
    }

    pub(crate) fn center(&self, i: usize, j: usize) -> P {
        [
            self.x0 + (i as f64 + 0.5) * self.h,
            self.y0 + (j as f64 + 0.5) * self.h,
        ]
    }

    /// Add the even-odd region of `rings`: each cell takes the larger of its
    /// coverage and the region's (`sub` sample rows per cell; exact along x).
    pub(crate) fn fill(&mut self, rings: &[Vec<P>], sub: usize) {
        let sub = sub.max(1);
        let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
        for r in rings {
            for p in r {
                lo = [lo[0].min(p[0]), lo[1].min(p[1])];
                hi = [hi[0].max(p[0]), hi[1].max(p[1])];
            }
        }
        if !lo[0].is_finite() {
            return;
        }
        let h = self.h;
        let j0 = (((lo[1] - self.y0) / h).floor().max(0.0)) as usize;
        let j1 = ((((hi[1] - self.y0) / h).ceil()) as usize).min(self.ny);
        if j0 >= j1 {
            return;
        }
        let i0 = (((lo[0] - self.x0) / h).floor().max(0.0)) as usize;
        let i1 = ((((hi[0] - self.x0) / h).ceil()) as usize).min(self.nx);
        if i0 >= i1 {
            return;
        }
        let w = i1 - i0;
        // Edge buckets by sample row.
        let rows = (j1 - j0) * sub;
        let mut buckets: Vec<Vec<(P, P)>> = vec![Vec::new(); rows];
        let row_y = |r: usize| self.y0 + (j0 as f64 + (r as f64 + 0.5) / sub as f64) * h;
        for ring in rings {
            let n = ring.len();
            for k in 0..n {
                let (a, b) = (ring[k], ring[(k + 1) % n]);
                if a[1] == b[1] {
                    continue;
                }
                let (ylo, yhi) = (a[1].min(b[1]), a[1].max(b[1]));
                let rlo = (((ylo - self.y0) / h - j0 as f64) * sub as f64 - 0.5)
                    .ceil()
                    .max(0.0) as usize;
                let rhi = ((((yhi - self.y0) / h - j0 as f64) * sub as f64 - 0.5).floor() + 1.0)
                    .max(0.0) as usize;
                for bucket in buckets.iter_mut().take(rhi.min(rows)).skip(rlo) {
                    bucket.push((a, b));
                }
            }
        }
        let mut local = vec![0f32; w];
        let mut xs: Vec<f64> = Vec::new();
        for jj in 0..(j1 - j0) {
            local.iter_mut().for_each(|c| *c = 0.0);
            for s in 0..sub {
                let r = jj * sub + s;
                let y = row_y(r);
                xs.clear();
                for &(a, b) in &buckets[r] {
                    if (a[1] > y) != (b[1] > y) {
                        xs.push(a[0] + (y - a[1]) * (b[0] - a[0]) / (b[1] - a[1]));
                    }
                }
                xs.sort_by(f64::total_cmp);
                for pair in xs.chunks_exact(2) {
                    let (xa, xb) = (pair[0], pair[1]);
                    let ca = ((xa - self.x0) / h).floor() as isize - i0 as isize;
                    let cb = ((xb - self.x0) / h).floor() as isize - i0 as isize;
                    for c in ca.max(0)..=cb.min(w as isize - 1) {
                        let cx0 = self.x0 + (i0 as f64 + c as f64) * h;
                        let overlap = (xb.min(cx0 + h) - xa.max(cx0)).max(0.0);
                        local[c as usize] += (overlap / h / sub as f64) as f32;
                    }
                }
            }
            let j = j0 + jj;
            for (c, cell) in local.iter().enumerate() {
                let k = self.idx(i0 + c, j);
                let v = cell.min(1.0);
                if v > self.cov[k] {
                    self.cov[k] = v;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unfracture_splits_a_bridged_fill_into_outline_and_hole() {
        // A 10 x 10 square with a 2 x 2 hole joined by a bridge along y = 5.
        let pts = vec![
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (0.0, 10.0),
            (0.0, 5.0),
            (4.0, 5.0),
            (4.0, 6.0),
            (6.0, 6.0),
            (6.0, 4.0),
            (4.0, 4.0),
            (4.0, 5.0),
            (0.0, 5.0),
        ];
        let rings = unfracture(&pts);
        assert_eq!(rings.len(), 2, "{rings:?}");
        assert!((rings_area(&rings) - 96.0).abs() < 1e-9);
        assert!(inside_rings([2.0, 2.0], &rings));
        assert!(!inside_rings([5.0, 5.0], &rings));
    }

    #[test]
    fn coverage_of_a_square_is_exact_on_aligned_cells_and_partial_on_edges() {
        let mut r = Raster::new([0.0, 0.0], [2.0, 2.0], 0.5);
        r.fill(&[vec![[0.25, 0.0], [1.5, 0.0], [1.5, 1.0], [0.25, 1.0]]], 4);
        assert!((r.cov[r.idx(0, 0)] - 0.5).abs() < 1e-6);
        assert!((r.cov[r.idx(1, 0)] - 1.0).abs() < 1e-6);
        assert!((r.cov[r.idx(2, 1)] - 1.0).abs() < 1e-6);
        assert_eq!(r.cov[r.idx(3, 0)], 0.0);
        assert_eq!(r.cov[r.idx(1, 2)], 0.0);
        let total: f32 = r.cov.iter().sum::<f32>() * 0.25;
        assert!((total - 1.25).abs() < 1e-5, "{total}");
    }

    #[test]
    fn segment_distances() {
        let (d, _, _, _, _) = segment_segment([0.0, 0.0], [1.0, 0.0], [0.5, 1.0], [0.5, 2.0]);
        assert!((d - 1.0).abs() < 1e-12);
        let (d, p, _, _, _) = segment_segment([0.0, 0.0], [2.0, 2.0], [0.0, 2.0], [2.0, 0.0]);
        assert_eq!(d, 0.0);
        assert!((p[0] - 1.0).abs() < 1e-12);
    }
}
