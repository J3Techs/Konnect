//! Copper and keepout shapes.
//!
//! A shape is a *core* — a point, a segment, or a region — swept by a
//! non-negative radius. That one model covers everything a board carries
//! exactly: a track is a segment swept by half its width, an oval pad or
//! oblong drill is a segment swept by half its short side, a round-rect pad
//! is a convex polygon swept by its corner radius, and a plane piece is a
//! polygon with holes and no sweep. Distance predicates
//! ([`crate::predicates`]) work on the core and add the radii, so no shape is
//! ever approximated to test clearance.

use crate::bbox::BBox;
use crate::point::Point;
use crate::units::Coord;
use serde::{Deserialize, Serialize};

/// A straight track (or stadium) of `width` nanometres around the centre line
/// `a`–`b`. `a == b` is legal: a round dot of diameter `width`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Segment {
    pub a: Point,
    pub b: Point,
    /// Full width in nm, `>= 0`.
    pub width: Coord,
}

impl Segment {
    pub fn new(a: Point, b: Point, width: Coord) -> Self {
        Segment { a, b, width }
    }

    pub fn bbox(&self) -> BBox {
        BBox::new(self.a, self.b).inflate(half_up(self.width))
    }

    /// Centre-line length, rounded to the nearest nanometre.
    pub fn length(&self) -> i64 {
        (self.b - self.a).length_f64().round() as i64
    }
}

/// A disc.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Circle {
    pub center: Point,
    /// Radius in nm, `>= 0`.
    pub radius: Coord,
}

impl Circle {
    pub fn new(center: Point, radius: Coord) -> Self {
        Circle { center, radius }
    }

    /// A disc of `diameter` nm. An odd diameter rounds the radius up, so the
    /// disc never under-covers what it stands for.
    pub fn from_diameter(center: Point, diameter: Coord) -> Self {
        Circle::new(center, half_up(diameter))
    }

    pub fn bbox(&self) -> BBox {
        BBox::point(self.center).inflate(self.radius)
    }
}

/// A closed polygon path. The closing edge from the last point back to the
/// first is implicit; the first point is never repeated at the end.
pub type Ring = Vec<Point>;

/// A convex polygon, optionally rounded: every point within `radius` of the
/// polygon. `radius == 0` is the sharp polygon itself.
///
/// The constructor normalizes the winding to a positive signed area (in the
/// `Vector::cross` sense) and refuses non-convex input, so consumers may rely
/// on both. Deserializing runs the same checks.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(try_from = "ConvexPolyData")]
pub struct ConvexPoly {
    pts: Ring,
    radius: Coord,
}

/// The unchecked serialized form of a [`ConvexPoly`].
#[derive(Deserialize)]
struct ConvexPolyData {
    pts: Ring,
    radius: Coord,
}

impl TryFrom<ConvexPolyData> for ConvexPoly {
    type Error = String;

    fn try_from(d: ConvexPolyData) -> Result<Self, String> {
        ConvexPoly::new(d.pts, d.radius)
            .ok_or_else(|| "not a convex polygon with a non-negative radius".to_string())
    }
}

impl ConvexPoly {
    /// Build from vertices in either winding. Consecutive duplicates (and a
    /// closing duplicate of the first point) are dropped. `None` when fewer
    /// than three distinct vertices remain, the area is zero, or the path is
    /// not convex.
    pub fn new(pts: Vec<Point>, radius: Coord) -> Option<Self> {
        if radius < 0 {
            return None;
        }
        let mut ring = dedup_ring(pts);
        if ring.len() < 3 {
            return None;
        }
        let area = signed_area2(&ring);
        if area == 0 {
            return None;
        }
        if area < 0 {
            ring.reverse();
        }
        let n = ring.len();
        for i in 0..n {
            let (a, b, c) = (ring[i], ring[(i + 1) % n], ring[(i + 2) % n]);
            if (b - a).cross(c - b) < 0 {
                return None;
            }
        }
        Some(ConvexPoly { pts: ring, radius })
    }

    /// An axis-aligned rectangle, optionally rounded.
    pub fn axis_rect(b: BBox, radius: Coord) -> Option<Self> {
        ConvexPoly::new(
            vec![
                b.min,
                Point::new(b.max.x, b.min.y),
                b.max,
                Point::new(b.min.x, b.max.y),
            ],
            radius,
        )
    }

    pub fn points(&self) -> &[Point] {
        &self.pts
    }

    pub fn radius(&self) -> Coord {
        self.radius
    }

    pub fn bbox(&self) -> BBox {
        BBox::from_points(self.pts.iter().copied())
            .expect("a ConvexPoly has at least three points")
            .inflate(self.radius)
    }
}

/// A polygon with holes: the region inside `outline` and outside every hole.
///
/// Holes are assumed to lie inside the outline and not to overlap each other
/// (KiCad's fractured fills and zone outlines satisfy this). Windings are
/// normalized — outline positive, holes negative — but predicates do not
/// depend on them. The rings are private so every polygon passed the
/// constructor's checks; deserializing runs them too.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(try_from = "PolygonData")]
pub struct Polygon {
    outline: Ring,
    holes: Vec<Ring>,
}

/// The unchecked serialized form of a [`Polygon`].
#[derive(Deserialize)]
struct PolygonData {
    outline: Ring,
    holes: Vec<Ring>,
}

impl TryFrom<PolygonData> for Polygon {
    type Error = String;

    fn try_from(d: PolygonData) -> Result<Self, String> {
        Polygon::new(d.outline, d.holes)
            .ok_or_else(|| "polygon outline has under three points or no area".to_string())
    }
}

impl Polygon {
    /// `None` when the outline has fewer than three distinct points or zero
    /// area. Degenerate holes are dropped rather than failing the polygon:
    /// they remove no area.
    pub fn new(outline: Ring, holes: Vec<Ring>) -> Option<Self> {
        let mut outline = dedup_ring(outline);
        if outline.len() < 3 {
            return None;
        }
        let area = signed_area2(&outline);
        if area == 0 {
            return None;
        }
        if area < 0 {
            outline.reverse();
        }
        let holes = holes
            .into_iter()
            .map(dedup_ring)
            .filter_map(|mut h| {
                let a = signed_area2(&h);
                if h.len() < 3 || a == 0 {
                    return None;
                }
                if a > 0 {
                    h.reverse();
                }
                Some(h)
            })
            .collect();
        Some(Polygon { outline, holes })
    }

    pub fn outline(&self) -> &[Point] {
        &self.outline
    }

    pub fn holes(&self) -> &[Ring] {
        &self.holes
    }

    /// Every ring, outline first.
    pub fn rings(&self) -> impl Iterator<Item = &[Point]> {
        std::iter::once(self.outline.as_slice()).chain(self.holes.iter().map(Vec::as_slice))
    }

    pub fn bbox(&self) -> BBox {
        BBox::from_points(self.outline.iter().copied())
            .expect("a Polygon has at least three points")
    }

    /// Twice the enclosed area (outline minus holes), exact.
    pub fn area2(&self) -> i128 {
        self.rings().map(signed_area2).sum()
    }
}

/// Any shape the router stores, checks or rasterizes.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Shape {
    Segment(Segment),
    Circle(Circle),
    ConvexPoly(ConvexPoly),
    Polygon(Polygon),
}

impl Shape {
    pub fn bbox(&self) -> BBox {
        match self {
            Shape::Segment(s) => s.bbox(),
            Shape::Circle(c) => c.bbox(),
            Shape::ConvexPoly(p) => p.bbox(),
            Shape::Polygon(p) => p.bbox(),
        }
    }

    /// The sweep radius around the core, doubled so odd widths stay exact:
    /// a segment's width, a circle's diameter, twice a rounding radius.
    pub(crate) fn sweep2(&self) -> i64 {
        match self {
            Shape::Segment(s) => i64::from(s.width),
            Shape::Circle(c) => 2 * i64::from(c.radius),
            Shape::ConvexPoly(p) => 2 * i64::from(p.radius),
            Shape::Polygon(_) => 0,
        }
    }
}

impl From<Segment> for Shape {
    fn from(s: Segment) -> Self {
        Shape::Segment(s)
    }
}

impl From<Circle> for Shape {
    fn from(c: Circle) -> Self {
        Shape::Circle(c)
    }
}

impl From<ConvexPoly> for Shape {
    fn from(p: ConvexPoly) -> Self {
        Shape::ConvexPoly(p)
    }
}

impl From<Polygon> for Shape {
    fn from(p: Polygon) -> Self {
        Shape::Polygon(p)
    }
}

/// `ceil(width / 2)` for a non-negative width: the bbox inflation that never
/// under-covers an odd width.
fn half_up(width: Coord) -> Coord {
    width / 2 + width % 2
}

/// Drop consecutive duplicates and a closing repeat of the first point.
fn dedup_ring(mut pts: Vec<Point>) -> Ring {
    pts.dedup();
    while pts.len() > 1 && pts.first() == pts.last() {
        pts.pop();
    }
    pts
}

/// Twice the signed area of a ring (shoelace), exact.
pub(crate) fn signed_area2(ring: &[Point]) -> i128 {
    let n = ring.len();
    (0..n)
        .map(|i| {
            let (a, b) = (ring[i], ring[(i + 1) % n]);
            i128::from(a.x) * i128::from(b.y) - i128::from(b.x) * i128::from(a.y)
        })
        .sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: i32, y: i32) -> Point {
        Point::new(x, y)
    }

    #[test]
    fn convex_poly_normalizes_winding() {
        let cw = ConvexPoly::new(vec![p(0, 0), p(0, 10), p(10, 10), p(10, 0)], 0).unwrap();
        assert!(signed_area2(cw.points()) > 0);
    }

    #[test]
    fn convex_poly_refuses_concave_and_degenerate_input() {
        let concave = vec![p(0, 0), p(10, 0), p(5, 2), p(10, 10), p(0, 10)];
        assert!(ConvexPoly::new(concave, 0).is_none());
        assert!(ConvexPoly::new(vec![p(0, 0), p(5, 5), p(10, 10)], 0).is_none());
        assert!(ConvexPoly::new(vec![p(0, 0), p(1, 0), p(0, 1)], -1).is_none());
    }

    #[test]
    fn closing_duplicate_is_dropped() {
        let poly = Polygon::new(vec![p(0, 0), p(10, 0), p(10, 10), p(0, 0)], vec![]).unwrap();
        assert_eq!(poly.outline().len(), 3);
    }

    #[test]
    fn deserializing_runs_the_constructor_checks() {
        let poly = Polygon::new(vec![p(0, 0), p(10, 0), p(10, 10)], vec![]).unwrap();
        let json = serde_json::to_string(&Shape::Polygon(poly.clone())).unwrap();
        assert_eq!(
            serde_json::from_str::<Shape>(&json).unwrap(),
            Shape::Polygon(poly)
        );
        let empty = r#"{"Polygon":{"outline":[],"holes":[]}}"#;
        assert!(serde_json::from_str::<Shape>(empty).is_err());
        let concave = r#"{"pts":[{"x":0,"y":0},{"x":10,"y":0},{"x":5,"y":2},{"x":10,"y":10},{"x":0,"y":10}],"radius":0}"#;
        assert!(serde_json::from_str::<ConvexPoly>(concave).is_err());
    }

    #[test]
    fn odd_diameters_round_the_radius_up() {
        assert_eq!(Circle::from_diameter(p(0, 0), 451).radius, 226);
        assert_eq!(Circle::from_diameter(p(0, 0), 450).radius, 225);
    }

    #[test]
    fn polygon_area_subtracts_holes() {
        let poly = Polygon::new(
            vec![p(0, 0), p(10, 0), p(10, 10), p(0, 10)],
            vec![vec![p(2, 2), p(4, 2), p(4, 4), p(2, 4)]],
        )
        .unwrap();
        assert_eq!(poly.area2(), 2 * (100 - 4));
    }

    #[test]
    fn segment_bbox_covers_odd_widths() {
        let s = Segment::new(p(0, 0), p(10, 0), 5);
        assert_eq!(s.bbox(), BBox::new(p(-3, -3), p(13, 3)));
    }
}
