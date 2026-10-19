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

use crate::bbox::BBox;
use crate::point::Point;
use crate::shape::{Polygon, Shape};
use crate::units::Coord;
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
/// minimum-clearance rule. Overlapping or touching shapes are `Less` than
/// any positive clearance. Against zero, touching shapes and overlapping
/// sharp cores (no sweep on either side) compare `Equal`, while swept shapes
/// that overlap compare `Less`; [`touches`] (`!= Greater`) covers both.
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

/// Compare the distance from a disc of diameter `width` centred on `p` to
/// the shape (edge to edge) with `clearance` (nm): whether a via or a
/// track end of that width at `p` would violate the clearance to the
/// shape. `width` 0 asks about the point itself. A raster uses it to decide
/// whether a lattice node lies in an obstacle inflated by the clearance
/// plus half a track width; passing the full width keeps odd widths exact.
///
/// `Less` means strictly closer. Overlap counts as a gap of zero or less:
/// a point on or inside a shape without sweep (a sharp pad, a zone piece)
/// compares `Equal` with `width` 0 and `clearance` 0, and `Less` only
/// against a positive threshold; a point strictly within a swept shape's
/// copper is `Less` even then. Test `!= Greater` to ask "touches or
/// overlaps".
pub fn point_shape_distance_cmp(p: Point, width: Coord, shape: &Shape, clearance: i64) -> Ordering {
    if clearance < 0 || width < 0 {
        return Ordering::Greater;
    }
    let t = 2 * i128::from(clearance) + i128::from(width) + i128::from(shape.sweep2());
    with_core(shape, |c| core_cmp(&Core::Point(dbl(p)), c, t))
}

/// Compare the gap between a track of `width` along `a`–`b` (round ends)
/// and the shape, edge to edge, with `clearance` (nm): the swept-track
/// test, whether such a track would violate the clearance to the shape.
/// It is [`gap_cmp`] of `Segment::new(a, b, width)` against `shape`
/// without building the [`Shape`], and exact for odd widths (half widths
/// are never rounded).
pub fn segment_shape_distance_cmp(
    a: Point,
    b: Point,
    width: Coord,
    shape: &Shape,
    clearance: i64,
) -> Ordering {
    if clearance < 0 || width < 0 {
        return Ordering::Greater;
    }
    let t = 2 * i128::from(clearance) + i128::from(width) + i128::from(shape.sweep2());
    with_core(shape, |c| core_cmp(&Core::Seg(dbl(a), dbl(b)), c, t))
}

/// Whether `shape` lies entirely inside the closed region of `poly`
/// (outline minus holes): KiCad's `enclosedByArea()` test, where the item
/// minus the area must be empty. Touching the boundary from inside counts as
/// inside.
///
/// Exact for every shape, given a valid `poly` (no two of its edges
/// overlap; see [`Polygon`]):
///
/// - a swept shape (positive radius) is inside when one core point is and
///   no region edge comes nearer the core than the radius;
/// - a sharp segment is inside when its ends are, it crosses no region edge
///   properly, and it does not leave the region just beside any region
///   vertex on it or any end on the boundary;
/// - a sharp region (a sharp pad, a zone piece) is inside when its vertices
///   are, no edges cross properly, no region vertex lies strictly inside
///   it, and no point just beside a vertex where the two boundaries meet is
///   inside the shape but outside the region. Any part of the shape outside
///   the region has such a corner, so nothing is missed.
///
/// "Just beside" is an infinitesimal displacement decided exactly by
/// lexicographic signs, never a rounded sample point.
pub fn shape_within_polygon(shape: &Shape, poly: &Polygon) -> bool {
    let rings: Vec<&[Point]> = poly.rings().collect();
    shape_within(shape, &RingsView::new(&rings))
}

// ─── Containment kernel ──────────────────────────────────────────────────────

/// A point displaced infinitesimally from `s`: `s + ε·d1 + ε²·d2` for an
/// infinitesimal `ε > 0` (doubled units). Containment of such a point is
/// decided exactly by comparing coefficients lexicographically.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Perturbed {
    pub(crate) s: P2,
    pub(crate) d1: (i128, i128),
    pub(crate) d2: (i128, i128),
}

/// The sign of `t[0] + t[1]·ε + t[2]·ε²`.
fn lex_sign(t: [i128; 3]) -> i128 {
    t.into_iter().find(|v| *v != 0).map_or(0, i128::signum)
}

fn cross2(a: (i128, i128), b: (i128, i128)) -> i128 {
    a.0 * b.1 - a.1 * b.0
}

fn dot2(a: (i128, i128), b: (i128, i128)) -> i128 {
    a.0 * b.0 + a.1 * b.1
}

fn vec2(from: P2, to: P2) -> (i128, i128) {
    (i128::from(to.x - from.x), i128::from(to.y - from.y))
}

/// Crossing-number containment of a perturbed point among `edges`, by the
/// same rule as [`point_in_rings`]. Every product stays below 2^70.
pub(crate) fn perturbed_in_edges(
    x: Perturbed,
    edges: impl Iterator<Item = (P2, P2)>,
) -> Containment {
    let above = |q: P2| lex_sign([i128::from(q.y - x.s.y), -x.d1.1, -x.d2.1]) > 0;
    let mut inside = false;
    for (a, b) in edges {
        if a == b {
            continue;
        }
        let e = vec2(a, b);
        let o = lex_sign([orient(a, b, x.s), cross2(e, x.d1), cross2(e, x.d2)]);
        if o == 0 {
            // On the edge's line: on the edge when between its ends.
            let t = [dot2(vec2(a, x.s), e), dot2(x.d1, e), dot2(x.d2, e)];
            if lex_sign(t) >= 0 && lex_sign([dot2(e, e) - t[0], -t[1], -t[2]]) >= 0 {
                return Containment::Boundary;
            }
            continue;
        }
        if above(a) != above(b) {
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

/// A closed polygonal region the containment kernel can query: plain rings
/// or an [`crate::IndexedPolygon`]. Points are doubled vertices (even).
pub(crate) trait RegionView {
    fn region_bbox(&self) -> BBox;
    fn contain(&self, p: P2) -> Containment;
    fn contain_perturbed(&self, x: Perturbed) -> Containment;
    /// Calls `f` on (at least) every edge whose box meets `q`, stopping and
    /// returning `true` as soon as `f` does.
    fn any_edge_near(&self, q: BBox, f: &mut dyn FnMut(P2, P2) -> bool) -> bool;
    /// Calls `f` on every edge, stopping and returning `true` as soon as `f`
    /// does.
    fn any_edge(&self, f: &mut dyn FnMut(P2, P2) -> bool) -> bool;
}

/// Plain rings, scanned edge by edge.
pub(crate) struct RingsView<'a> {
    rings: &'a [&'a [Point]],
    bbox: BBox,
}

impl<'a> RingsView<'a> {
    pub(crate) fn new(rings: &'a [&'a [Point]]) -> Self {
        let bbox = BBox::from_points(rings[0].iter().copied()).unwrap_or(BBox::point(rings[0][0]));
        RingsView { rings, bbox }
    }
}

impl RegionView for RingsView<'_> {
    fn region_bbox(&self) -> BBox {
        self.bbox
    }

    fn contain(&self, p: P2) -> Containment {
        point_in_rings(p, self.rings)
    }

    fn contain_perturbed(&self, x: Perturbed) -> Containment {
        perturbed_in_edges(x, edges(self.rings))
    }

    fn any_edge_near(&self, q: BBox, f: &mut dyn FnMut(P2, P2) -> bool) -> bool {
        edges(self.rings).any(|(a, b)| edge_bbox(a, b).intersects(&q) && f(a, b))
    }

    fn any_edge(&self, f: &mut dyn FnMut(P2, P2) -> bool) -> bool {
        edges(self.rings).any(|(a, b)| f(a, b))
    }
}

/// Back from doubled to nm coordinates (exact for doubled vertices).
pub(crate) fn undbl(p: P2) -> Point {
    Point::new((p.x / 2) as i32, (p.y / 2) as i32)
}

/// The nm box of an edge between doubled vertices.
pub(crate) fn edge_bbox(a: P2, b: P2) -> BBox {
    BBox::new(undbl(a), undbl(b))
}

/// [`shape_within_polygon`] against any region view.
pub(crate) fn shape_within(shape: &Shape, area: &dyn RegionView) -> bool {
    let bbox = shape.bbox();
    if !area.region_bbox().contains_bbox(&bbox) {
        return false;
    }
    let r2 = i128::from(shape.sweep2());
    with_core(shape, |core| {
        if r2 > 0 {
            // The core is connected: with no boundary nearer than the
            // radius it is wholly inside or wholly outside, and one point
            // decides which.
            let probe = match core {
                Core::Point(p) => *p,
                Core::Seg(a, _) => *a,
                Core::Region(r) => dbl(r[0][0]),
            };
            return area.contain(probe) != Containment::Outside
                && !area.any_edge_near(bbox, &mut |c, d| {
                    core_cmp(core, &Core::Seg(c, d), r2) == Ordering::Less
                });
        }
        match core {
            Core::Point(p) => area.contain(*p) != Containment::Outside,
            Core::Seg(a, b) => segment_within(*a, *b, area),
            Core::Region(r) => region_within(&RingsView::new(r), area),
        }
    })
}

/// Whether the sharp segment `a`–`b` lies inside `area`.
pub(crate) fn segment_within(a: P2, b: P2, area: &dyn RegionView) -> bool {
    let (ca, cb) = (area.contain(a), area.contain(b));
    if ca == Containment::Outside || cb == Containment::Outside {
        return false;
    }
    if a == b {
        return true;
    }
    let q = edge_bbox(a, b);
    if area.any_edge_near(q, &mut |c, d| crosses_properly(a, b, c, d)) {
        return false;
    }
    // Without a proper crossing the segment can leave the region only at a
    // region vertex on it, or at an end on the boundary: look just beside
    // each, along the segment.
    let d = vec2(a, b);
    let back = (-d.0, -d.1);
    let mut probes: Vec<(P2, (i128, i128))> = Vec::new();
    if ca == Containment::Boundary {
        probes.push((a, d));
    }
    if cb == Containment::Boundary {
        probes.push((b, back));
    }
    area.any_edge_near(q, &mut |v, _| {
        if v != a && v != b && orient(a, b, v) == 0 && within_box(a, b, v) {
            probes.push((v, d));
            probes.push((v, back));
        }
        false
    });
    !probes.into_iter().any(|(s, dir)| {
        area.contain_perturbed(Perturbed {
            s,
            d1: dir,
            d2: (0, 0),
        }) == Containment::Outside
    })
}

/// Whether the sharp region `inner` lies inside `area`.
///
/// `inner` minus `area` is open; when it is not empty it has a corner, and
/// with no proper edge crossing every corner is a vertex of one of the two
/// boundaries. A vertex of `inner` outside `area`, or a vertex of `area`
/// strictly inside `inner`, decides at once. Otherwise the corner is a
/// vertex on both boundaries, and a point just to either side of each
/// boundary ray leaving it lands in every sector around it.
pub(crate) fn region_within(inner: &dyn RegionView, area: &dyn RegionView) -> bool {
    let ib = inner.region_bbox();
    if !area.region_bbox().contains_bbox(&ib) {
        return false;
    }
    let mut events: Vec<P2> = Vec::new();
    if inner.any_edge(&mut |v, _| match area.contain(v) {
        Containment::Outside => true,
        Containment::Boundary => {
            events.push(v);
            false
        }
        Containment::Inside => false,
    }) {
        return false;
    }
    if inner.any_edge(&mut |a, b| {
        area.any_edge_near(edge_bbox(a, b), &mut |c, d| crosses_properly(a, b, c, d))
    }) {
        return false;
    }
    if area.any_edge_near(ib, &mut |v, _| match inner.contain(v) {
        Containment::Inside => true,
        Containment::Boundary => {
            events.push(v);
            false
        }
        Containment::Outside => false,
    }) {
        return false;
    }
    events.sort_unstable_by_key(|p| (p.x, p.y));
    events.dedup();
    let mut rays: Vec<(i128, i128)> = Vec::new();
    for s in events {
        rays.clear();
        let q = BBox::point(undbl(s));
        let mut collect = |u: P2, v: P2| {
            if u == v {
                return false;
            }
            if s == u {
                rays.push(vec2(u, v));
            } else if s == v {
                rays.push(vec2(v, u));
            } else if orient(u, v, s) == 0 && within_box(u, v, s) {
                rays.push(vec2(s, v));
                rays.push(vec2(s, u));
            }
            false
        };
        inner.any_edge_near(q, &mut collect);
        area.any_edge_near(q, &mut collect);
        for &r in &rays {
            for side in [1, -1] {
                let x = Perturbed {
                    s,
                    d1: r,
                    d2: (-r.1 * side, r.0 * side),
                };
                if inner.contain_perturbed(x) == Containment::Inside
                    && area.contain_perturbed(x) == Containment::Outside
                {
                    return false;
                }
            }
        }
    }
    true
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
pub(crate) fn within_box(a: P2, b: P2, p: P2) -> bool {
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
        assert_eq!(point_shape_distance_cmp(p(130, 50), 0, &pad, 20), Equal);
        assert_eq!(point_shape_distance_cmp(p(130, 50), 0, &pad, 21), Less);
        // A via of diameter 10 there: 15 from the copper.
        assert_eq!(point_shape_distance_cmp(p(130, 50), 10, &pad, 15), Equal);
        // Inside the swept copper is strictly closer than any distance.
        assert_eq!(point_shape_distance_cmp(p(50, 50), 0, &pad, 0), Less);
        // Inside a sharp core: distance zero, Equal at zero.
        let sharp = Shape::from(ConvexPoly::axis_rect(BBox::new(p(0, 0), p(100, 100)), 0).unwrap());
        assert_eq!(point_shape_distance_cmp(p(50, 50), 0, &sharp, 0), Equal);
        assert_eq!(point_shape_distance_cmp(p(50, 50), 0, &sharp, 1), Less);
        // A 50-wide track centre line 60 above the pad: half width 25 plus
        // clearance 25.
        assert_eq!(
            segment_shape_distance_cmp(p(-50, 160), p(150, 160), 50, &pad, 25),
            Equal
        );
        assert_eq!(
            segment_shape_distance_cmp(p(-50, 160), p(150, 160), 50, &pad, 26),
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

    #[test]
    fn swept_predicates_keep_odd_widths_exact() {
        // Sharp pad up to y = 100; a 101-wide track centred on y = 160 has
        // its edge at 109.5, a gap of 9.5. Flooring the half width would
        // move the edge to 110 and call clearance 10 legal.
        let pad = Shape::from(ConvexPoly::axis_rect(BBox::new(p(0, 0), p(100, 100)), 0).unwrap());
        let (a, b) = (p(-50, 160), p(150, 160));
        assert_eq!(segment_shape_distance_cmp(a, b, 101, &pad, 9), Greater);
        assert_eq!(segment_shape_distance_cmp(a, b, 101, &pad, 10), Less);
        let track = Shape::from(Segment::new(a, b, 101));
        for c in [9, 10] {
            assert_eq!(
                segment_shape_distance_cmp(a, b, 101, &pad, c),
                gap_cmp(&track, &pad, c)
            );
        }
        // A 101-wide disc centred 60 above the pad: gap 9.5.
        assert_eq!(point_shape_distance_cmp(p(50, 160), 101, &pad, 9), Greater);
        assert_eq!(point_shape_distance_cmp(p(50, 160), 101, &pad, 10), Less);
        // Negative inputs never report a violation.
        assert_eq!(segment_shape_distance_cmp(a, b, 101, &pad, -1), Greater);
        assert_eq!(point_shape_distance_cmp(p(50, 160), -2, &pad, 100), Greater);
    }

    fn rect_poly(x0: i32, y0: i32, x1: i32, y1: i32) -> Vec<Point> {
        vec![p(x0, y0), p(x1, y0), p(x1, y1), p(x0, y1)]
    }

    fn sharp(pts: Vec<Point>) -> Shape {
        Shape::from(ConvexPoly::new(pts, 0).unwrap())
    }

    #[test]
    fn sharp_regions_touching_holes_are_not_inside() {
        // (a) A diamond hole whose corners touch the rect's edges.
        let area = Polygon::new(
            rect_poly(0, 0, 100, 100),
            vec![vec![p(50, 40), p(60, 50), p(50, 60), p(40, 50)]],
        )
        .unwrap();
        assert!(!shape_within_polygon(
            &sharp(rect_poly(40, 40, 60, 60)),
            &area
        ));
        // (b) The rect is exactly a square hole.
        let holed =
            Polygon::new(rect_poly(0, 0, 100, 100), vec![rect_poly(40, 40, 60, 60)]).unwrap();
        assert!(!shape_within_polygon(
            &sharp(rect_poly(40, 40, 60, 60)),
            &holed
        ));
        // A hole sharing three sides with the rect: its inner edge splits
        // the rect between two boundary points.
        let area = Polygon::new(rect_poly(-10, -10, 20, 20), vec![rect_poly(0, 0, 5, 10)]).unwrap();
        assert!(!shape_within_polygon(
            &sharp(rect_poly(0, 0, 10, 10)),
            &area
        ));
        // An area edge running through two opposite corners of the rect.
        let area = Polygon::new(vec![p(-5, -5), p(25, 25), p(-5, 25)], vec![]).unwrap();
        assert!(!shape_within_polygon(
            &sharp(rect_poly(0, 0, 10, 10)),
            &area
        ));
        // Legal touching: equal to the outline, or sharing an edge or a
        // corner with a hole from outside it.
        let plain = Polygon::new(rect_poly(0, 0, 100, 100), vec![]).unwrap();
        assert!(shape_within_polygon(
            &sharp(rect_poly(0, 0, 100, 100)),
            &plain
        ));
        assert!(!shape_within_polygon(
            &sharp(rect_poly(0, 0, 100, 100)),
            &holed
        ));
        assert!(shape_within_polygon(
            &sharp(rect_poly(20, 40, 40, 60)),
            &holed
        ));
        assert!(shape_within_polygon(
            &sharp(rect_poly(20, 20, 40, 40)),
            &holed
        ));
        assert!(shape_within_polygon(
            &sharp(vec![p(60, 60), p(80, 60), p(60, 80)]),
            &holed
        ));
        // The indexed answers agree.
        let idx = crate::IndexedPolygon::new(holed.clone());
        assert!(!idx.contains(&sharp(rect_poly(40, 40, 60, 60))));
        assert!(idx.contains(&sharp(rect_poly(20, 40, 40, 60))));
        let hole_sized =
            crate::IndexedPolygon::new(Polygon::new(rect_poly(40, 40, 60, 60), vec![]).unwrap());
        assert!(!idx.contains_polygon(&hole_sized));
    }

    #[test]
    fn sharp_segments_through_hole_corners_are_not_inside() {
        // A diamond hole whose left and right corners sit on the segment,
        // away from its midpoint.
        let area = Polygon::new(
            rect_poly(-50, -50, 150, 50),
            vec![vec![p(10, 0), p(20, 10), p(30, 0), p(20, -10)]],
        )
        .unwrap();
        let seg = |a: Point, b: Point| Shape::from(Segment::new(a, b, 0));
        assert!(!shape_within_polygon(&seg(p(0, 0), p(100, 0)), &area));
        // Touching one corner only, or running along an edge, is inside.
        assert!(shape_within_polygon(&seg(p(0, 0), p(10, 0)), &area));
        assert!(shape_within_polygon(&seg(p(0, -10), p(100, -10)), &area));
        assert!(shape_within_polygon(&seg(p(10, 0), p(20, 10)), &area));
        // Along the outline, then out of it.
        assert!(shape_within_polygon(&seg(p(-50, 0), p(-50, 50)), &area));
        assert!(!shape_within_polygon(&seg(p(-50, 0), p(-50, 60)), &area));
        let idx = crate::IndexedPolygon::new(area);
        assert!(!idx.contains(&seg(p(0, 0), p(100, 0))));
        assert!(idx.contains(&seg(p(0, -10), p(100, -10))));
    }

    /// Exhaustive cross-check on a small lattice: axis rects and 45°
    /// diamonds with corners on multiples of 4 make arrangements whose every
    /// face holds a lattice point, so "a lattice point strictly inside the
    /// shape lies outside the area" decides containment exactly.
    #[test]
    fn sharp_containment_matches_lattice_sampling() {
        let mut seed: u64 = 0x5eed_1234_abcd_0001;
        let mut next = move |n: i32| {
            seed = seed
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((seed >> 33) % n as u64) as i32
        };
        fn shape_at(next: &mut dyn FnMut(i32) -> i32) -> Vec<Point> {
            let (x, y) = (4 * next(10), 4 * next(10));
            let w = 4 + 4 * next(4);
            if next(2) == 0 {
                let h = 4 + 4 * next(4);
                rect_poly(x, y, x + w, y + h)
            } else {
                vec![
                    p(x, y + w),
                    p(x + w, y),
                    p(x + 2 * w, y + w),
                    p(x + w, y + 2 * w),
                ]
            }
        }
        let mut checked = (0, 0);
        for _ in 0..400 {
            let outline = rect_poly(4 * next(3), 4 * next(3), 28 + 4 * next(4), 28 + 4 * next(4));
            let holes: Vec<Vec<Point>> = (0..next(3)).map(|_| shape_at(&mut next)).collect();
            // Holes strictly inside the outline and apart from each other.
            let out_poly = Polygon::new(outline.clone(), vec![]).unwrap();
            let valid = holes.iter().all(|h| {
                h.iter()
                    .all(|q| point_in_polygon(*q, &out_poly) == Containment::Inside)
            }) && holes.iter().enumerate().all(|(i, a)| {
                holes[i + 1..].iter().all(|b| {
                    let pa = Shape::Polygon(Polygon::new(a.clone(), vec![]).unwrap());
                    let pb = Shape::Polygon(Polygon::new(b.clone(), vec![]).unwrap());
                    gap_cmp(&pa, &pb, 0) == Greater
                })
            });
            if !valid {
                continue;
            }
            let Some(area) = Polygon::new(outline, holes) else {
                continue;
            };
            let idx = crate::IndexedPolygon::new(area.clone());
            for _ in 0..8 {
                let c = shape_at(&mut next);
                let cpoly = Polygon::new(c.clone(), vec![]).unwrap();
                let bb = cpoly.bbox();
                let mut witness = false;
                'scan: for y in bb.min.y..=bb.max.y {
                    for x in bb.min.x..=bb.max.x {
                        let q = p(x, y);
                        if point_in_polygon(q, &cpoly) == Containment::Inside
                            && point_in_polygon(q, &area) == Containment::Outside
                        {
                            witness = true;
                            break 'scan;
                        }
                    }
                }
                let got = shape_within_polygon(&sharp(c.clone()), &area);
                assert_eq!(got, !witness, "{c:?} in {area:?}");
                assert_eq!(idx.contains(&sharp(c.clone())), got, "{c:?}");
                let ic = crate::IndexedPolygon::new(cpoly);
                assert_eq!(idx.contains_polygon(&ic), got, "{c:?}");
                if got {
                    checked.0 += 1;
                } else {
                    checked.1 += 1;
                }
            }
        }
        assert!(checked.0 > 50 && checked.1 > 50, "{checked:?}");
    }
}
