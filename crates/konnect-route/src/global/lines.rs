//! Blocked intervals along axis-parallel sample lines.
//!
//! Capacity estimation asks one question many times: along the line
//! `x = X` (or `y = Y`), which stretches are within `d` of an obstacle? Every
//! obstacle is inflated by its own `d` and intersected with each sample line
//! its box spans, giving closed intervals that are then sorted and merged
//! per line into a flat CSR layout.
//!
//! The kernel works on vertical lines; horizontal lines reuse it by swapping
//! the coordinates of every input point. Floats appear only here, in an
//! estimate that never decides legality, and every interval is rounded
//! outward to whole nanometres.

use konnect_geom::{Point, Polygon, Shape};

/// One inflated obstacle, in nm, ready for line intersection.
#[derive(Debug, Clone)]
pub(crate) enum Obstacle<'a> {
    /// A disc of radius `r`.
    Disc { c: (f64, f64), r: f64 },
    /// The points within `r` of segment `a`–`b`.
    Capsule {
        a: (f64, f64),
        b: (f64, f64),
        r: f64,
    },
    /// The points within `r` of a convex polygon.
    Convex { pts: Vec<(f64, f64)>, r: f64 },
    /// The points inside a polygon with holes, or within `r` of its rings.
    Region { poly: &'a Polygon, r: f64 },
}

fn fp(p: Point) -> (f64, f64) {
    (f64::from(p.x), f64::from(p.y))
}

impl<'a> Obstacle<'a> {
    /// The obstacle `shape` inflated by `d` nm.
    pub(crate) fn from_shape(shape: &'a Shape, d: f64) -> Obstacle<'a> {
        match shape {
            Shape::Segment(s) => Obstacle::Capsule {
                a: fp(s.a),
                b: fp(s.b),
                r: f64::from(s.width) / 2.0 + d,
            },
            Shape::Circle(c) => Obstacle::Disc {
                c: fp(c.center),
                r: f64::from(c.radius) + d,
            },
            Shape::ConvexPoly(p) => Obstacle::Convex {
                pts: p.points().iter().map(|&q| fp(q)).collect(),
                r: f64::from(p.radius()) + d,
            },
            Shape::Polygon(poly) => Obstacle::Region { poly, r: d },
        }
    }

    /// The inflated bounding box `(x0, y0, x1, y1)`.
    pub(crate) fn bounds(&self) -> (f64, f64, f64, f64) {
        let grow = |b: (f64, f64, f64, f64), r: f64| (b.0 - r, b.1 - r, b.2 + r, b.3 + r);
        match self {
            Obstacle::Disc { c, r } => (c.0 - r, c.1 - r, c.0 + r, c.1 + r),
            Obstacle::Capsule { a, b, r } => {
                grow((a.0.min(b.0), a.1.min(b.1), a.0.max(b.0), a.1.max(b.1)), *r)
            }
            Obstacle::Convex { pts, r } => grow(points_bounds(pts.iter().copied()), *r),
            Obstacle::Region { poly, r } => {
                let b = poly.bbox();
                grow(
                    (
                        f64::from(b.min.x),
                        f64::from(b.min.y),
                        f64::from(b.max.x),
                        f64::from(b.max.y),
                    ),
                    *r,
                )
            }
        }
    }

    /// Intersect with the line `x = x` (after an optional x/y swap of the
    /// obstacle), appending closed `(lo, hi)` intervals in the other axis.
    pub(crate) fn cut(&self, x: f64, swap: bool, out: &mut Vec<(f64, f64)>) {
        let s = |p: (f64, f64)| if swap { (p.1, p.0) } else { p };
        match self {
            Obstacle::Disc { c, r } => {
                if let Some(iv) = disc(s(*c), *r, x) {
                    out.push(iv);
                }
            }
            Obstacle::Capsule { a, b, r } => {
                if let Some(iv) = capsule(s(*a), s(*b), *r, x) {
                    out.push(iv);
                }
            }
            Obstacle::Convex { pts, r } => {
                let pts: Vec<(f64, f64)> = pts.iter().map(|&p| s(p)).collect();
                if let Some(iv) = rounded_convex(&pts, *r, x) {
                    out.push(iv);
                }
            }
            Obstacle::Region { poly, r } => region(poly, *r, x, swap, out),
        }
    }
}

fn points_bounds(pts: impl Iterator<Item = (f64, f64)>) -> (f64, f64, f64, f64) {
    pts.fold(
        (
            f64::INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NEG_INFINITY,
        ),
        |b, p| (b.0.min(p.0), b.1.min(p.1), b.2.max(p.0), b.3.max(p.1)),
    )
}

fn hull(a: Option<(f64, f64)>, b: Option<(f64, f64)>) -> Option<(f64, f64)> {
    match (a, b) {
        (Some(a), Some(b)) => Some((a.0.min(b.0), a.1.max(b.1))),
        (a, None) => a,
        (None, b) => b,
    }
}

fn disc(c: (f64, f64), r: f64, x: f64) -> Option<(f64, f64)> {
    let dx = x - c.0;
    if r < 0.0 || dx.abs() > r {
        return None;
    }
    let h = (r * r - dx * dx).max(0.0).sqrt();
    Some((c.1 - h, c.1 + h))
}

/// A convex polygon (any winding) against `x = x`.
fn convex(pts: &[(f64, f64)], x: f64) -> Option<(f64, f64)> {
    let n = pts.len();
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for i in 0..n {
        let (p, q) = (pts[i], pts[(i + 1) % n]);
        let (x0, x1) = (p.0.min(q.0), p.0.max(q.0));
        if x < x0 || x > x1 {
            continue;
        }
        if (q.0 - p.0).abs() < 1e-9 {
            lo = lo.min(p.1.min(q.1));
            hi = hi.max(p.1.max(q.1));
        } else {
            let t = (x - p.0) / (q.0 - p.0);
            let y = p.1 + t * (q.1 - p.1);
            lo = lo.min(y);
            hi = hi.max(y);
        }
    }
    (lo <= hi).then_some((lo, hi))
}

/// The band of half-width `r` along segment `a`–`b` (without end caps).
fn band(a: (f64, f64), b: (f64, f64), r: f64, x: f64) -> Option<(f64, f64)> {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let len = (dx * dx + dy * dy).sqrt();
    if len == 0.0 || r <= 0.0 {
        return None;
    }
    let (nx, ny) = (-dy / len * r, dx / len * r);
    convex(
        &[
            (a.0 + nx, a.1 + ny),
            (b.0 + nx, b.1 + ny),
            (b.0 - nx, b.1 - ny),
            (a.0 - nx, a.1 - ny),
        ],
        x,
    )
}

fn capsule(a: (f64, f64), b: (f64, f64), r: f64, x: f64) -> Option<(f64, f64)> {
    hull(hull(disc(a, r, x), disc(b, r, x)), band(a, b, r, x))
}

/// The Minkowski sum of a convex polygon and a disc is convex, so its cut
/// is the hull of the cuts of its pieces.
fn rounded_convex(pts: &[(f64, f64)], r: f64, x: f64) -> Option<(f64, f64)> {
    let mut acc = convex(pts, x);
    if r > 0.0 {
        let n = pts.len();
        for i in 0..n {
            let (p, q) = (pts[i], pts[(i + 1) % n]);
            acc = hull(acc, disc(p, r, x));
            acc = hull(acc, band(p, q, r, x));
        }
    }
    acc
}

/// A polygon with holes: even-odd inside spans plus a capsule around every
/// ring edge.
fn region(poly: &Polygon, r: f64, x: f64, swap: bool, out: &mut Vec<(f64, f64)>) {
    let s = |p: Point| {
        let q = fp(p);
        if swap {
            (q.1, q.0)
        } else {
            q
        }
    };
    let mut crossings: Vec<f64> = Vec::new();
    for ring in poly.rings() {
        let n = ring.len();
        for i in 0..n {
            let (p, q) = (s(ring[i]), s(ring[(i + 1) % n]));
            // Half-open in x so a vertex on the line is counted once.
            if (p.0 <= x && x < q.0) || (q.0 <= x && x < p.0) {
                let t = (x - p.0) / (q.0 - p.0);
                crossings.push(p.1 + t * (q.1 - p.1));
            }
            if r > 0.0 {
                if let Some(iv) = capsule(p, q, r, x) {
                    out.push(iv);
                }
            }
        }
    }
    crossings.sort_by(f64::total_cmp);
    for pair in crossings.chunks_exact(2) {
        out.push((pair[0], pair[1]));
    }
}

/// The even-odd inside spans of a closed set of segments along `x = x`
/// (swapped as in [`Obstacle::cut`]). `None` when the crossing count is odd,
/// which means the segments do not close.
pub(crate) fn inside_spans(segs: &[(Point, Point)], x: f64, swap: bool) -> Option<Vec<(f64, f64)>> {
    let s = |p: Point| {
        let q = fp(p);
        if swap {
            (q.1, q.0)
        } else {
            q
        }
    };
    let mut crossings: Vec<f64> = Vec::new();
    for &(a, b) in segs {
        let (p, q) = (s(a), s(b));
        if (p.0 <= x && x < q.0) || (q.0 <= x && x < p.0) {
            let t = (x - p.0) / (q.0 - p.0);
            crossings.push(p.1 + t * (q.1 - p.1));
        }
    }
    if crossings.len() % 2 == 1 {
        return None;
    }
    crossings.sort_by(f64::total_cmp);
    Some(
        crossings
            .chunks_exact(2)
            .map(|pair| (pair[0], pair[1]))
            .collect(),
    )
}

/// Sample lines at `origin + j · step`, `j < count`, each with its sorted,
/// merged, closed blocked intervals in whole nm.
#[derive(Debug, Clone, Default)]
pub(crate) struct LineSet {
    pub origin: i64,
    pub step: i64,
    pub count: u32,
    off: Vec<u32>,
    lo: Vec<i32>,
    hi: Vec<i32>,
}

impl LineSet {
    /// Build from raw per-line intervals (any order, overlapping).
    pub(crate) fn from_raw(origin: i64, step: i64, raw: Vec<Vec<(i32, i32)>>) -> LineSet {
        let mut off = Vec::with_capacity(raw.len() + 1);
        let mut lo = Vec::new();
        let mut hi = Vec::new();
        off.push(0);
        for mut line in raw.into_iter() {
            line.sort_unstable();
            let mut cur: Option<(i32, i32)> = None;
            for (a, b) in line {
                match cur {
                    Some((ca, cb)) if a <= cb.saturating_add(1) => cur = Some((ca, cb.max(b))),
                    Some(c) => {
                        lo.push(c.0);
                        hi.push(c.1);
                        cur = Some((a, b));
                    }
                    None => cur = Some((a, b)),
                }
            }
            if let Some(c) = cur {
                lo.push(c.0);
                hi.push(c.1);
            }
            off.push(lo.len() as u32);
        }
        LineSet {
            origin,
            step,
            count: (off.len() - 1) as u32,
            off,
            lo,
            hi,
        }
    }

    /// Line indices whose position lies in `[a, b]`.
    pub(crate) fn range(&self, a: i64, b: i64) -> std::ops::Range<u32> {
        let first = (a - self.origin + self.step - 1)
            .div_euclid(self.step)
            .max(0);
        let last = (b - self.origin).div_euclid(self.step);
        let last = last.min(i64::from(self.count) - 1);
        if last < first {
            return 0..0;
        }
        first as u32..last as u32 + 1
    }

    /// The nearest line to position `p`, clamped into range.
    pub(crate) fn nearest(&self, p: i64) -> u32 {
        let j = (2 * (p - self.origin) + self.step).div_euclid(2 * self.step);
        j.clamp(0, i64::from(self.count) - 1) as u32
    }

    /// Push line `j`'s intervals that meet `[a, b]` onto `out`.
    pub(crate) fn collect(&self, j: u32, a: i32, b: i32, out: &mut Vec<(i32, i32)>) {
        let (s, e) = (
            self.off[j as usize] as usize,
            self.off[j as usize + 1] as usize,
        );
        let his = &self.hi[s..e];
        let start = s + his.partition_point(|&h| h < a);
        for i in start..e {
            if self.lo[i] > b {
                break;
            }
            out.push((self.lo[i], self.hi[i]));
        }
    }

    /// Whether position `y` on line `j` is blocked.
    pub(crate) fn blocked(&self, j: u32, y: i32) -> bool {
        let (s, e) = (
            self.off[j as usize] as usize,
            self.off[j as usize + 1] as usize,
        );
        let his = &self.hi[s..e];
        let i = s + his.partition_point(|&h| h < y);
        i < e && self.lo[i] <= y
    }
}

/// Round an interval outward to whole nm, clamped into `i32`.
pub(crate) fn round_out(iv: (f64, f64)) -> (i32, i32) {
    let c = |v: f64| v.clamp(f64::from(i32::MIN), f64::from(i32::MAX)) as i32;
    (c(iv.0.floor()), c(iv.1.ceil()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use konnect_geom::{Circle, ConvexPoly, Segment};

    #[test]
    fn disc_and_capsule_cuts() {
        let c = Shape::Circle(Circle::new(Point::new(0, 0), 100));
        let mut out = Vec::new();
        Obstacle::from_shape(&c, 0.0).cut(0.0, false, &mut out);
        assert_eq!(out, vec![(-100.0, 100.0)]);
        out.clear();
        Obstacle::from_shape(&c, 0.0).cut(200.0, false, &mut out);
        assert!(out.is_empty());

        let s = Shape::Segment(Segment::new(Point::new(0, 0), Point::new(1000, 0), 200));
        Obstacle::from_shape(&s, 50.0).cut(500.0, false, &mut out);
        assert_eq!(out, vec![(-150.0, 150.0)]);
        out.clear();
        // Swapped: the horizontal segment seen along y = 0 spans its length.
        Obstacle::from_shape(&s, 50.0).cut(0.0, true, &mut out);
        let (lo, hi) = out[0];
        assert!((lo + 150.0).abs() < 1e-6 && (hi - 1150.0).abs() < 1e-6);
    }

    #[test]
    fn rounded_rect_cut_includes_corners() {
        let b = konnect_geom::BBox::new(Point::new(0, 0), Point::new(1000, 400));
        let s = Shape::ConvexPoly(ConvexPoly::axis_rect(b, 0).unwrap());
        let mut out = Vec::new();
        Obstacle::from_shape(&s, 100.0).cut(500.0, false, &mut out);
        let (lo, hi) = out[0];
        assert!((lo + 100.0).abs() < 1e-6 && (hi - 500.0).abs() < 1e-6);
        out.clear();
        // Beyond the right edge by 60: inside the corner disc.
        Obstacle::from_shape(&s, 100.0).cut(1060.0, false, &mut out);
        let (lo, hi) = out[0];
        assert!((lo + 80.0).abs() < 1e-6 && (hi - 480.0).abs() < 1e-6);
    }

    #[test]
    fn line_set_merges_and_queries() {
        let ls = LineSet::from_raw(0, 10, vec![vec![(5, 9), (0, 3), (8, 20)], vec![]]);
        let mut out = Vec::new();
        ls.collect(0, 4, 4, &mut out);
        assert!(out.is_empty());
        ls.collect(0, 0, 100, &mut out);
        assert_eq!(out, vec![(0, 3), (5, 20)]);
        assert!(ls.blocked(0, 12));
        assert!(!ls.blocked(0, 4));
        assert!(!ls.blocked(1, 4));
        assert_eq!(ls.range(-5, 10), 0..2);
        assert_eq!(ls.range(1, 9), 0..0);
        assert_eq!(ls.nearest(14), 1);
    }
}
