//! Approximate measurements for reports, costs and markers.
//!
//! Nothing here decides legality: the exact predicates do. These functions
//! say *how far* apart two shapes are and *where* they come closest, in
//! `f64`, so a checker can print "actual 0.1145 mm" next to a violation the
//! exact kernel found, and a router can rank candidates by slack.

use crate::point::Point;
use crate::predicates::{point_in_polygon, Containment};
use crate::shape::{Polygon, Shape};

/// The edge-to-edge gap between two shapes in nm (`0.0` when they touch or
/// overlap), and the two closest points on their cores.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Closest {
    pub gap_nm: f64,
    pub on_a: Point,
    pub on_b: Point,
}

impl Closest {
    /// The midpoint of the two closest points: where a marker goes.
    pub fn midpoint(&self) -> Point {
        let mid = |a: i32, b: i32| ((i64::from(a) + i64::from(b)).div_euclid(2)) as i32;
        Point::new(mid(self.on_a.x, self.on_b.x), mid(self.on_a.y, self.on_b.y))
    }
}

/// Approximate edge-to-edge gap in nm; `0.0` for touching or overlapping
/// shapes.
pub fn gap_nm(a: &Shape, b: &Shape) -> f64 {
    closest(a, b).gap_nm
}

/// Closest points between two shapes' cores and the approximate gap.
pub fn closest(a: &Shape, b: &Shape) -> Closest {
    let (d, pa, pb) = core_closest(&FCore::of(a), &FCore::of(b));
    Closest {
        gap_nm: (d - radius(a) - radius(b)).max(0.0),
        on_a: round(pa),
        on_b: round(pb),
    }
}

type F2 = (f64, f64);

enum FCore<'a> {
    Seg(F2, F2),
    Region(&'a [Point], Option<&'a Polygon>),
}

impl<'a> FCore<'a> {
    fn of(s: &'a Shape) -> Self {
        match s {
            Shape::Segment(g) => FCore::Seg(f(g.a), f(g.b)),
            Shape::Circle(c) => FCore::Seg(f(c.center), f(c.center)),
            Shape::ConvexPoly(p) => FCore::Region(p.points(), None),
            Shape::Polygon(p) => FCore::Region(p.outline(), Some(p)),
        }
    }

    fn segments(&self) -> Vec<(F2, F2)> {
        match self {
            FCore::Seg(a, b) => vec![(*a, *b)],
            FCore::Region(outline, poly) => {
                let mut out = ring_segments(outline);
                if let Some(p) = poly {
                    for h in p.holes() {
                        out.extend(ring_segments(h));
                    }
                }
                out
            }
        }
    }

    /// Whether a point is inside the region core (never for segments).
    fn contains(&self, p: F2) -> bool {
        match self {
            FCore::Seg(..) => false,
            FCore::Region(outline, poly) => {
                let q = round(p);
                match poly {
                    Some(poly) => point_in_polygon(q, poly) != Containment::Outside,
                    None => match Polygon::new(outline.to_vec(), vec![]) {
                        Some(poly) => point_in_polygon(q, &poly) != Containment::Outside,
                        None => false,
                    },
                }
            }
        }
    }

    fn probe(&self) -> F2 {
        match self {
            FCore::Seg(a, _) => *a,
            FCore::Region(outline, _) => f(outline[0]),
        }
    }
}

fn ring_segments(ring: &[Point]) -> Vec<(F2, F2)> {
    let n = ring.len();
    (0..n).map(|i| (f(ring[i]), f(ring[(i + 1) % n]))).collect()
}

fn core_closest(a: &FCore<'_>, b: &FCore<'_>) -> (f64, F2, F2) {
    if b.contains(a.probe()) {
        return (0.0, a.probe(), a.probe());
    }
    if a.contains(b.probe()) {
        return (0.0, b.probe(), b.probe());
    }
    let mut best = (f64::INFINITY, (0.0, 0.0), (0.0, 0.0));
    for (a0, a1) in a.segments() {
        for (b0, b1) in b.segments() {
            let c = seg_seg(a0, a1, b0, b1);
            if c.0 < best.0 {
                best = c;
            }
        }
    }
    best
}

fn seg_seg(a0: F2, a1: F2, b0: F2, b1: F2) -> (f64, F2, F2) {
    if let Some(x) = crossing(a0, a1, b0, b1) {
        return (0.0, x, x);
    }
    let cands = [
        (a0, project(a0, b0, b1)),
        (a1, project(a1, b0, b1)),
        (project(b0, a0, a1), b0),
        (project(b1, a0, a1), b1),
    ];
    let mut best = (f64::INFINITY, a0, b0);
    for (pa, pb) in cands {
        let d = dist(pa, pb);
        if d < best.0 {
            best = (d, pa, pb);
        }
    }
    best
}

fn crossing(a0: F2, a1: F2, b0: F2, b1: F2) -> Option<F2> {
    let r = (a1.0 - a0.0, a1.1 - a0.1);
    let s = (b1.0 - b0.0, b1.1 - b0.1);
    let den = r.0 * s.1 - r.1 * s.0;
    if den == 0.0 {
        return None;
    }
    let q = (b0.0 - a0.0, b0.1 - a0.1);
    let t = (q.0 * s.1 - q.1 * s.0) / den;
    let u = (q.0 * r.1 - q.1 * r.0) / den;
    ((0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u))
        .then_some((a0.0 + t * r.0, a0.1 + t * r.1))
}

fn project(p: F2, a: F2, b: F2) -> F2 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let l2 = dx * dx + dy * dy;
    if l2 == 0.0 {
        return a;
    }
    let t = (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / l2).clamp(0.0, 1.0);
    (a.0 + t * dx, a.1 + t * dy)
}

fn dist(a: F2, b: F2) -> f64 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

fn radius(s: &Shape) -> f64 {
    match s {
        Shape::Segment(g) => f64::from(g.width) / 2.0,
        Shape::Circle(c) => f64::from(c.radius),
        Shape::ConvexPoly(p) => f64::from(p.radius()),
        Shape::Polygon(_) => 0.0,
    }
}

fn f(p: Point) -> F2 {
    (f64::from(p.x), f64::from(p.y))
}

fn round(p: F2) -> Point {
    Point::new(p.0.round() as i32, p.1.round() as i32)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bbox::BBox;
    use crate::shape::{Circle, ConvexPoly, Segment};

    fn p(x: i32, y: i32) -> Point {
        Point::new(x, y)
    }

    #[test]
    fn parallel_tracks() {
        let a = Shape::from(Segment::new(p(0, 0), p(1000, 0), 100));
        let b = Shape::from(Segment::new(p(200, 300), p(800, 300), 100));
        let c = closest(&a, &b);
        assert!((c.gap_nm - 200.0).abs() < 1e-9);
        assert_eq!(c.on_a.y, 0);
        assert_eq!(c.on_b.y, 300);
        assert_eq!(c.midpoint().y, 150);
    }

    #[test]
    fn via_near_a_rounded_pad_and_overlaps() {
        let via = Shape::from(Circle::new(p(0, 0), 225));
        let pad =
            Shape::from(ConvexPoly::axis_rect(BBox::new(p(400, -100), p(600, 100)), 25).unwrap());
        assert!((gap_nm(&via, &pad) - 150.0).abs() < 1e-9);
        let inside = Shape::from(Circle::new(p(500, 0), 10));
        assert_eq!(gap_nm(&inside, &pad), 0.0);
        let crossing = Shape::from(Segment::new(p(0, 0), p(1000, 0), 10));
        assert_eq!(gap_nm(&crossing, &pad), 0.0);
    }
}
