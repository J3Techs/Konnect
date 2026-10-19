//! Exact distance and containment predicates.
//!
//! Every predicate answers a comparison — *is the distance less than, equal
//! to, or greater than this threshold?* — rather than returning a distance,
//! because a distance between a point and a slanted segment is irrational and
//! any rounded value can sit on the wrong side of a rule. The comparisons are
//! exact for every `i32` input:
//!
//! - coordinates are doubled into `i64` so half widths stay integers;
//! - dot and cross products are `i128`;
//! - the one comparison of products of squares (a perpendicular distance
//!   `cross² / |ab|²` against `t²`) is done in 256 bits.
//!
//! Shapes are compared edge to edge: [`gap_cmp`] measures the distance
//! between the two cores and subtracts both sweep radii (see
//! [`crate::shape`]). A gap is never negative here — overlapping shapes have
//! gap "zero or less", and every threshold the router asks about is `>= 0`.

use crate::point::Point;
use crate::shape::{Polygon, Shape};
use std::cmp::Ordering;

/// Where a point lies relative to a closed region.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Containment {
    Inside,
    /// Exactly on an edge or vertex of any ring.
    Boundary,
    Outside,
}

/// Compare the distance from `p` to the closed segment `a`–`b` with
/// `distance` (nm). `Less` means strictly closer.
pub fn point_segment_distance_cmp(p: Point, a: Point, b: Point, distance: i64) -> Ordering {
    let t = 2 * i128::from(distance);
    if t < 0 {
        return Ordering::Greater;
    }
    point_seg_cmp(dbl(p), dbl(a), dbl(b), t)
}

/// Compare the distance between closed segments `a`–`b` and `c`–`d` with
/// `distance` (nm). Intersecting segments are at distance 0.
pub fn segment_segment_distance_cmp(
    a: Point,
    b: Point,
    c: Point,
    d: Point,
    distance: i64,
) -> Ordering {
    let t = 2 * i128::from(distance);
    if t < 0 {
        return Ordering::Greater;
    }
    seg_seg_cmp(dbl(a), dbl(b), dbl(c), dbl(d), t)
}

/// Compare the distance from the closed segment `a`–`b` to the polygon's
/// closed region with `distance` (nm). A segment touching or inside the
/// region is at distance 0.
pub fn segment_polygon_distance_cmp(a: Point, b: Point, poly: &Polygon, distance: i64) -> Ordering {
    let t = 2 * i128::from(distance);
    if t < 0 {
        return Ordering::Greater;
    }
    let rings: Vec<&[Point]> = poly.rings().collect();
    core_cmp(&Core::Seg(dbl(a), dbl(b)), &Core::Region(&rings), t)
}

/// Whether two closed segments share at least one point.
pub fn segments_intersect(a: Point, b: Point, c: Point, d: Point) -> bool {
    seg_intersect(dbl(a), dbl(b), dbl(c), dbl(d))
}

/// Where `p` lies relative to the polygon's region (outline minus holes).
pub fn point_in_polygon(p: Point, poly: &Polygon) -> Containment {
    let rings: Vec<&[Point]> = poly.rings().collect();
    point_in_rings(dbl(p), &rings)
}

/// Compare the edge-to-edge gap between two shapes with `clearance` (nm).
///
/// `Less` means the shapes are closer than `clearance` — a violation of a
/// minimum-clearance rule. Overlapping or touching shapes compare as a gap of
/// zero against a positive clearance (`Less`) and `Equal` against zero.
pub fn gap_cmp(a: &Shape, b: &Shape, clearance: i64) -> Ordering {
    let t = 2 * i128::from(clearance) + i128::from(a.sweep2()) + i128::from(b.sweep2());
    if t < 0 {
        return Ordering::Greater;
    }
    with_core(a, |ca| with_core(b, |cb| core_cmp(ca, cb, t)))
}

/// `true` when the shapes are at least `clearance` apart, edge to edge.
pub fn has_clearance(a: &Shape, b: &Shape, clearance: i64) -> bool {
    gap_cmp(a, b, clearance) != Ordering::Less
}

/// `true` when the shapes touch or overlap — the connectivity test.
pub fn touches(a: &Shape, b: &Shape) -> bool {
    gap_cmp(a, b, 0) != Ordering::Greater
}

/// Compare the distance from `p` to the shape's edge with `distance` (nm).
/// `Less` means `p` is strictly within `distance` of the shape's copper (or
/// strictly inside it): the test a raster uses to decide whether a lattice
/// node lies in an obstacle inflated by `distance`.
pub fn point_shape_distance_cmp(p: Point, shape: &Shape, distance: i64) -> Ordering {
    let t = 2 * i128::from(distance) + i128::from(shape.sweep2());
    if t < 0 {
        return Ordering::Greater;
    }
    with_core(shape, |c| core_cmp(&Core::Point(dbl(p)), c, t))
}

/// Compare the distance from the zero-width segment `a`–`b` to the shape
/// (edge to edge) with `distance` (nm). With `distance` = clearance plus
/// half a track width this is the swept-track test: whether a track of that
/// width along `a`–`b` would violate the clearance to the shape. It is
/// [`gap_cmp`] without building a [`Shape`] for the segment.
pub fn segment_shape_distance_cmp(a: Point, b: Point, shape: &Shape, distance: i64) -> Ordering {
    let t = 2 * i128::from(distance) + i128::from(shape.sweep2());
    if t < 0 {
        return Ordering::Greater;
    }
    with_core(shape, |c| core_cmp(&Core::Seg(dbl(a), dbl(b)), c, t))
}

/// Whether `shape` lies entirely inside the closed region of `poly`
/// (outline minus holes): KiCad's `enclosedByArea()` test, where the item
/// minus the area must be empty. Touching the boundary from inside counts as
/// inside.
///
/// Exact for points, swept segments and swept polygons with a positive
/// sweep radius. For zero-radius cores the test is: every core vertex (and a
/// segment's midpoint) not outside, no proper crossing between core and
/// region edges, and no region vertex strictly inside a core polygon.
pub fn shape_within_polygon(shape: &Shape, poly: &Polygon) -> bool {
    let rings: Vec<&[Point]> = poly.rings().collect();
    let r2 = i128::from(shape.sweep2());
    with_core(shape, |core| {
        let probe = match core {
            Core::Point(p) => *p,
            Core::Seg(a, _) => *a,
            Core::Region(r) => dbl(r[0][0]),
        };
        if point_in_rings(probe, &rings) == Containment::Outside {
            return false;
        }
        // A swept shape pokes out wherever the boundary comes nearer than
        // its radius (doubled units: the threshold is the doubled radius).
        if r2 > 0
            && edges(&rings).any(|(c, d)| core_cmp(core, &Core::Seg(c, d), r2) == Ordering::Less)
        {
            return false;
        }
        match core {
            Core::Point(_) => true,
            Core::Seg(a, b) => {
                if r2 > 0 {
                    return true;
                }
                let mid = P2 {
                    x: (a.x + b.x) / 2,
                    y: (a.y + b.y) / 2,
                };
                point_in_rings(*b, &rings) != Containment::Outside
                    && point_in_rings(mid, &rings) != Containment::Outside
                    && !edges(&rings).any(|(c, d)| crosses_properly(*a, *b, c, d))
            }
            Core::Region(cr) => {
                if edges(cr).any(|(v, _)| point_in_rings(v, &rings) == Containment::Outside) {
                    return false;
                }
                if edges(cr).any(|(a, b)| edges(&rings).any(|(c, d)| crosses_properly(a, b, c, d)))
                {
                    return false;
                }
                !edges(&rings).any(|(v, _)| point_in_rings(v, cr) == Containment::Inside)
            }
        }
    })
}

/// Whether two segments cross at one interior point of both.
pub(crate) fn crosses_properly(a: P2, b: P2, c: P2, d: P2) -> bool {
    let d1 = orient(c, d, a).signum();
    let d2 = orient(c, d, b).signum();
    let d3 = orient(a, b, c).signum();
    let d4 = orient(a, b, d).signum();
    d1 * d2 < 0 && d3 * d4 < 0
}

/// Exact squared distance between two points.
pub fn dist2_point_point(a: Point, b: Point) -> i128 {
    (b - a).length2()
}

// ─── Doubled-coordinate kernel ───────────────────────────────────────────────

/// A point with doubled coordinates.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct P2 {
    pub(crate) x: i64,
    pub(crate) y: i64,
}

pub(crate) fn dbl(p: Point) -> P2 {
    P2 {
        x: 2 * i64::from(p.x),
        y: 2 * i64::from(p.y),
    }
}

/// A threshold no pair of `i32` points can reach in doubled units (their
/// doubled distance is below 2^34), so anything this large is simply `Less`.
const T_CAP: i128 = 1 << 40;

pub(crate) enum Core<'a> {
    Point(P2),
    Seg(P2, P2),
    /// Outline first, then holes; the closed region between them.
    Region(&'a [&'a [Point]]),
}

pub(crate) fn with_core<R>(s: &Shape, f: impl FnOnce(&Core<'_>) -> R) -> R {
    match s {
        Shape::Segment(seg) => f(&Core::Seg(dbl(seg.a), dbl(seg.b))),
        Shape::Circle(c) => f(&Core::Point(dbl(c.center))),
        Shape::ConvexPoly(p) => {
            let rings = [p.points()];
            f(&Core::Region(&rings))
        }
        Shape::Polygon(p) => {
            let rings: Vec<&[Point]> = p.rings().collect();
            f(&Core::Region(&rings))
        }
    }
}

/// Compare the distance between two cores with the doubled threshold `t >= 0`.
pub(crate) fn core_cmp(a: &Core<'_>, b: &Core<'_>, t: i128) -> Ordering {
    use Core::*;
    match (a, b) {
        (Point(p), Point(q)) => cmp_dist2(dist2(*p, *q), t),
        (Point(p), Seg(c, d)) | (Seg(c, d), Point(p)) => point_seg_cmp(*p, *c, *d, t),
        (Seg(a0, a1), Seg(b0, b1)) => seg_seg_cmp(*a0, *a1, *b0, *b1, t),
        (Point(p), Region(r)) | (Region(r), Point(p)) => {
            if point_in_rings(*p, r) != Containment::Outside {
                return zero_cmp(t);
            }
            min_cmp(edges(r).map(|(c, d)| point_seg_cmp(*p, c, d, t)))
        }
        (Seg(s0, s1), Region(r)) | (Region(r), Seg(s0, s1)) => {
            if point_in_rings(*s0, r) != Containment::Outside
                || point_in_rings(*s1, r) != Containment::Outside
                || edges(r).any(|(c, d)| seg_intersect(*s0, *s1, c, d))
            {
                return zero_cmp(t);
            }
            min_cmp(edges(r).map(|(c, d)| seg_seg_cmp(*s0, *s1, c, d, t)))
        }
        (Region(ra), Region(rb)) => {
            let crossing =
                edges(ra).any(|(a0, a1)| edges(rb).any(|(b0, b1)| seg_intersect(a0, a1, b0, b1)));
            // With no boundary crossing, the regions overlap only if one
            // outline lies inside the other's region; one vertex decides it.
            if crossing
                || point_in_rings(dbl(ra[0][0]), rb) != Containment::Outside
                || point_in_rings(dbl(rb[0][0]), ra) != Containment::Outside
            {
                return zero_cmp(t);
            }
            min_cmp(
                edges(ra).map(|(a0, a1)| {
                    min_cmp(edges(rb).map(|(b0, b1)| seg_seg_cmp(a0, a1, b0, b1, t)))
                }),
            )
        }
    }
}

/// Every edge of every ring, in doubled coordinates.
pub(crate) fn edges<'a>(rings: &'a [&'a [Point]]) -> impl Iterator<Item = (P2, P2)> + 'a {
    rings.iter().flat_map(|ring| {
        let n = ring.len();
        (0..n).map(move |i| (dbl(ring[i]), dbl(ring[(i + 1) % n])))
    })
}

/// The smallest of several comparisons against the same threshold:
/// `Less` if any is, else `Equal` if any is, else `Greater` (also for none).
pub(crate) fn min_cmp(iter: impl Iterator<Item = Ordering>) -> Ordering {
    let mut best = Ordering::Greater;
    for o in iter {
        if o == Ordering::Less {
            return o;
        }
        best = best.min(o);
    }
    best
}

pub(crate) fn zero_cmp(t: i128) -> Ordering {
    0.cmp(&t)
}

fn dist2(a: P2, b: P2) -> u128 {
    let dx = i128::from(b.x - a.x);
    let dy = i128::from(b.y - a.y);
    (dx * dx + dy * dy) as u128
}

fn cmp_dist2(d2: u128, t: i128) -> Ordering {
    if t >= T_CAP {
        return Ordering::Less;
    }
    d2.cmp(&((t * t) as u128))
}

pub(crate) fn orient(a: P2, b: P2, c: P2) -> i128 {
    let (abx, aby) = (i128::from(b.x - a.x), i128::from(b.y - a.y));
    let (acx, acy) = (i128::from(c.x - a.x), i128::from(c.y - a.y));
    abx * acy - aby * acx
}

/// `p` within the bounding box of `a`–`b` (meaningful when collinear).
fn within_box(a: P2, b: P2, p: P2) -> bool {
    a.x.min(b.x) <= p.x && p.x <= a.x.max(b.x) && a.y.min(b.y) <= p.y && p.y <= a.y.max(b.y)
}

pub(crate) fn seg_intersect(a: P2, b: P2, c: P2, d: P2) -> bool {
    let d1 = orient(c, d, a).signum();
    let d2 = orient(c, d, b).signum();
    let d3 = orient(a, b, c).signum();
    let d4 = orient(a, b, d).signum();
    if d1 * d2 < 0 && d3 * d4 < 0 {
        return true;
    }
    (d1 == 0 && within_box(c, d, a))
        || (d2 == 0 && within_box(c, d, b))
        || (d3 == 0 && within_box(a, b, c))
        || (d4 == 0 && within_box(a, b, d))
}

/// Distance from `p` to segment `a`–`b` against the doubled threshold.
pub(crate) fn point_seg_cmp(p: P2, a: P2, b: P2, t: i128) -> Ordering {
    if t >= T_CAP {
        return Ordering::Less;
    }
    let (abx, aby) = (i128::from(b.x - a.x), i128::from(b.y - a.y));
    let (apx, apy) = (i128::from(p.x - a.x), i128::from(p.y - a.y));
    let l2 = abx * abx + aby * aby;
    let dot = apx * abx + apy * aby;
    if l2 == 0 || dot <= 0 {
        return cmp_dist2(dist2(a, p), t);
    }
    if dot >= l2 {
        return cmp_dist2(dist2(b, p), t);
    }
    // Perpendicular foot inside the segment: dist² = cross² / l2, so compare
    // cross² with t² · l2 — both up to ~2^140, hence 256-bit products.
    let cross = (abx * apy - aby * apx).unsigned_abs();
    let tt = (t * t) as u128;
    cmp_products(cross, cross, tt, l2 as u128)
}

pub(crate) fn seg_seg_cmp(a: P2, b: P2, c: P2, d: P2, t: i128) -> Ordering {
    if seg_intersect(a, b, c, d) {
        return zero_cmp(t);
    }
    min_cmp(
        [
            point_seg_cmp(a, c, d, t),
            point_seg_cmp(b, c, d, t),
            point_seg_cmp(c, a, b, t),
            point_seg_cmp(d, a, b, t),
        ]
        .into_iter(),
    )
}

/// Crossing-number containment over all rings; boundary hits are exact.
pub(crate) fn point_in_rings(p: P2, rings: &[&[Point]]) -> Containment {
    let mut inside = false;
    for (a, b) in edges(rings) {
        let o = orient(a, b, p);
        if o == 0 && within_box(a, b, p) {
            return Containment::Boundary;
        }
        if (a.y > p.y) != (b.y > p.y) {
            // p is left of the crossing iff it is on the edge's left side
            // when the edge points up (+y), right side when it points down.
            let left = if b.y > a.y { o > 0 } else { o < 0 };
            if left {
                inside = !inside;
            }
        }
    }
    if inside {
        Containment::Inside
    } else {
        Containment::Outside
    }
}

/// Compare `a·b` with `c·d` exactly, all operands unsigned 128-bit.
fn cmp_products(a: u128, b: u128, c: u128, d: u128) -> Ordering {
    mul_wide(a, b).cmp(&mul_wide(c, d))
}

/// Full 256-bit product as `(high, low)` halves.
fn mul_wide(a: u128, b: u128) -> (u128, u128) {
    const MASK: u128 = u64::MAX as u128;
    let (a0, a1) = (a & MASK, a >> 64);
    let (b0, b1) = (b & MASK, b >> 64);
    let p00 = a0 * b0;
    let p01 = a0 * b1;
    let p10 = a1 * b0;
    let p11 = a1 * b1;
    let mid = (p00 >> 64) + (p01 & MASK) + (p10 & MASK);
    let lo = (p00 & MASK) | ((mid & MASK) << 64);
    let hi = p11 + (p01 >> 64) + (p10 >> 64) + (mid >> 64);
    (hi, lo)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shape::{Circle, ConvexPoly, Segment};
    use crate::BBox;
    use Ordering::*;

    fn p(x: i32, y: i32) -> Point {
        Point::new(x, y)
    }

    #[test]
    fn wide_multiply_matches_u128_where_it_fits() {
        for (a, b) in [
            (0u128, 5u128),
            (u64::MAX as u128, u64::MAX as u128),
            (12345, 678910),
        ] {
            assert_eq!(mul_wide(a, b), (0, a * b));
        }
        assert_eq!(mul_wide(1 << 127, 2), (1, 0));
        assert_eq!(mul_wide(u128::MAX, u128::MAX), (u128::MAX - 1, 1));
    }

    #[test]
    fn point_segment_distance_is_exact_at_the_threshold() {
        // Horizontal segment, point 100 nm above its middle.
        assert_eq!(
            point_segment_distance_cmp(p(50, 100), p(0, 0), p(100, 0), 100),
            Equal
        );
        assert_eq!(
            point_segment_distance_cmp(p(50, 100), p(0, 0), p(100, 0), 101),
            Less
        );
        assert_eq!(
            point_segment_distance_cmp(p(50, 100), p(0, 0), p(100, 0), 99),
            Greater
        );
        // Beyond the end: distance to the endpoint (3-4-5).
        assert_eq!(
            point_segment_distance_cmp(p(103, 4), p(0, 0), p(100, 0), 5),
            Equal
        );
    }

    #[test]
    fn slanted_distances_are_decided_without_rounding() {
        // Point (0, 2) to the line y = x: distance sqrt(2) ≈ 1.41421.
        let (a, b) = (p(-10, -10), p(10, 10));
        assert_eq!(point_segment_distance_cmp(p(0, 2), a, b, 1), Greater);
        assert_eq!(point_segment_distance_cmp(p(0, 2), a, b, 2), Less);
        // Large coordinates: same geometry 400 mm out.
        let o = 400_000_000;
        let (a, b) = (p(o - 10, o - 10), p(o + 10, o + 10));
        assert_eq!(point_segment_distance_cmp(p(o, o + 2), a, b, 1), Greater);
        assert_eq!(point_segment_distance_cmp(p(o, o + 2), a, b, 2), Less);
    }

    #[test]
    fn extreme_coordinates_do_not_overflow() {
        let (a, b) = (p(i32::MIN, i32::MIN), p(i32::MAX, i32::MAX));
        assert_eq!(
            point_segment_distance_cmp(p(i32::MIN, i32::MAX), a, b, 1),
            Greater
        );
        assert_eq!(point_segment_distance_cmp(p(0, 0), a, b, 1), Less);
    }

    #[test]
    fn crossing_and_touching_segments_are_at_zero() {
        assert!(segments_intersect(p(0, 0), p(10, 10), p(0, 10), p(10, 0)));
        assert!(segments_intersect(p(0, 0), p(10, 0), p(10, 0), p(20, 5)));
        assert!(segments_intersect(p(0, 0), p(10, 0), p(5, 0), p(20, 0)));
        assert!(!segments_intersect(p(0, 0), p(10, 0), p(11, 0), p(20, 0)));
        assert_eq!(
            segment_segment_distance_cmp(p(0, 0), p(10, 10), p(0, 10), p(10, 0), 0),
            Equal
        );
    }

    #[test]
    fn parallel_segment_distance() {
        assert_eq!(
            segment_segment_distance_cmp(p(0, 0), p(100, 0), p(20, 50), p(80, 50), 50),
            Equal
        );
        assert_eq!(
            segment_segment_distance_cmp(p(0, 0), p(100, 0), p(20, 50), p(80, 50), 51),
            Less
        );
    }

    fn square_with_hole() -> Polygon {
        Polygon::new(
            vec![p(0, 0), p(100, 0), p(100, 100), p(0, 100)],
            vec![vec![p(40, 40), p(60, 40), p(60, 60), p(40, 60)]],
        )
        .unwrap()
    }

    #[test]
    fn containment_respects_holes_and_boundaries() {
        let poly = square_with_hole();
        assert_eq!(point_in_polygon(p(10, 10), &poly), Containment::Inside);
        assert_eq!(point_in_polygon(p(50, 50), &poly), Containment::Outside);
        assert_eq!(point_in_polygon(p(0, 50), &poly), Containment::Boundary);
        assert_eq!(point_in_polygon(p(40, 50), &poly), Containment::Boundary);
        assert_eq!(point_in_polygon(p(150, 50), &poly), Containment::Outside);
        // A ray through a vertex must not double-count.
        assert_eq!(point_in_polygon(p(-10, 0), &poly), Containment::Outside);
        assert_eq!(point_in_polygon(p(50, 100), &poly), Containment::Boundary);
    }

    #[test]
    fn segment_polygon_distance() {
        let poly = square_with_hole();
        // Fully inside the hole, 5 from its edges.
        assert_eq!(
            segment_polygon_distance_cmp(p(45, 50), p(55, 50), &poly, 5),
            Equal
        );
        assert_eq!(
            segment_polygon_distance_cmp(p(45, 50), p(55, 50), &poly, 6),
            Less
        );
        // Crossing into the region.
        assert_eq!(
            segment_polygon_distance_cmp(p(50, 50), p(50, 30), &poly, 1),
            Less
        );
        // Outside, 10 from the right edge.
        assert_eq!(
            segment_polygon_distance_cmp(p(110, 0), p(110, 100), &poly, 10),
            Equal
        );
    }

    #[test]
    fn gap_subtracts_widths_and_radii() {
        // Two 100 nm tracks whose centre lines are 300 apart: gap 200.
        let a = Shape::from(Segment::new(p(0, 0), p(1000, 0), 100));
        let b = Shape::from(Segment::new(p(0, 300), p(1000, 300), 100));
        assert_eq!(gap_cmp(&a, &b, 200), Equal);
        assert!(has_clearance(&a, &b, 200));
        assert!(!has_clearance(&a, &b, 201));
        // Odd widths stay exact: 101 + 99 → half-widths sum to 100.
        let a = Shape::from(Segment::new(p(0, 0), p(1000, 0), 101));
        let b = Shape::from(Segment::new(p(0, 300), p(1000, 300), 99));
        assert_eq!(gap_cmp(&a, &b, 200), Equal);
        // A via (circle) next to a rounded pad: 400 core to core, minus the
        // 225 via radius and the 25 rounding radius.
        let via = Shape::from(Circle::new(p(0, 0), 225));
        let pad =
            Shape::from(ConvexPoly::axis_rect(BBox::new(p(400, -100), p(600, 100)), 25).unwrap());
        assert_eq!(gap_cmp(&via, &pad, 150), Equal);
    }

    #[test]
    fn overlapping_shapes_touch() {
        let a = Shape::from(Segment::new(p(0, 0), p(100, 0), 10));
        let pad = Shape::from(ConvexPoly::axis_rect(BBox::new(p(90, -20), p(130, 20)), 0).unwrap());
        assert!(touches(&a, &pad));
        let far = Shape::from(Circle::new(p(500, 500), 10));
        assert!(!touches(&a, &far));
        let plane = Shape::from(square_with_hole());
        let in_hole = Shape::from(Circle::new(p(50, 50), 5));
        assert!(!touches(&plane, &in_hole));
        assert_eq!(gap_cmp(&plane, &in_hole, 5), Equal);
    }

    #[test]
    fn point_and_swept_segment_distances_to_shapes() {
        let pad = Shape::from(ConvexPoly::axis_rect(BBox::new(p(0, 0), p(100, 100)), 10).unwrap());
        // 30 from the sharp edge, 20 from the rounded copper.
        assert_eq!(point_shape_distance_cmp(p(130, 50), &pad, 20), Equal);
        assert_eq!(point_shape_distance_cmp(p(130, 50), &pad, 21), Less);
        // Inside the swept copper is strictly closer than any distance.
        assert_eq!(point_shape_distance_cmp(p(50, 50), &pad, 0), Less);
        // A track centre line 60 above the pad: half width 25 + clearance 25.
        assert_eq!(
            segment_shape_distance_cmp(p(-50, 160), p(150, 160), &pad, 50),
            Equal
        );
        assert_eq!(
            segment_shape_distance_cmp(p(-50, 160), p(150, 160), &pad, 51),
            Less
        );
        // Same answer as gap_cmp with a real track of width 50.
        let track = Shape::from(Segment::new(p(-50, 160), p(150, 160), 50));
        assert_eq!(gap_cmp(&track, &pad, 25), Equal);
    }

    #[test]
    fn shapes_within_a_polygon_with_a_hole() {
        let area = square_with_hole();
        let inside = |s: Shape| shape_within_polygon(&s, &area);
        assert!(inside(Shape::from(Circle::new(p(20, 20), 10))));
        // Touching the outline from inside still counts.
        assert!(inside(Shape::from(Circle::new(p(10, 50), 10))));
        assert!(!inside(Shape::from(Circle::new(p(10, 50), 11))));
        // Over the hole.
        assert!(!inside(Shape::from(Circle::new(p(35, 50), 6))));
        assert!(inside(Shape::from(Segment::new(p(10, 10), p(90, 10), 10))));
        assert!(!inside(Shape::from(Segment::new(p(10, 10), p(90, 90), 2))));
        let rect = |x0, y0, x1, y1| {
            Shape::from(ConvexPoly::axis_rect(BBox::new(p(x0, y0), p(x1, y1)), 0).unwrap())
        };
        assert!(inside(rect(0, 0, 30, 30)));
        assert!(!inside(rect(30, 30, 70, 70)));
        // Surrounds the hole entirely: no vertex outside, no crossing.
        assert!(!inside(rect(35, 35, 65, 65)));
        assert!(!inside(rect(-5, 10, 20, 20)));
        assert!(!inside(Shape::from(Circle::new(p(500, 500), 1))));
    }

    #[test]
    fn region_inside_region_overlaps() {
        let outer = Shape::from(ConvexPoly::axis_rect(BBox::new(p(0, 0), p(100, 100)), 0).unwrap());
        let inner = Shape::from(ConvexPoly::axis_rect(BBox::new(p(10, 10), p(20, 20)), 0).unwrap());
        assert!(touches(&outer, &inner));
        assert!(touches(&inner, &outer));
    }
}
