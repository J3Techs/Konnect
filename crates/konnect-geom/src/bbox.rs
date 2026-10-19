//! Axis-aligned bounding boxes.

use crate::point::Point;
use crate::units::Coord;
use serde::{Deserialize, Serialize};

/// A closed axis-aligned box: every point with `min.x <= x <= max.x` and
/// `min.y <= y <= max.y`. Always non-empty (`min <= max` per axis); an empty
/// set is `Option<BBox>::None`, never a box with inverted corners.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct BBox {
    pub min: Point,
    pub max: Point,
}

impl BBox {
    /// The box spanned by two corners, in either order.
    pub fn new(a: Point, b: Point) -> Self {
        BBox {
            min: Point::new(a.x.min(b.x), a.y.min(b.y)),
            max: Point::new(a.x.max(b.x), a.y.max(b.y)),
        }
    }

    /// The degenerate box holding one point.
    pub fn point(p: Point) -> Self {
        BBox { min: p, max: p }
    }

    /// The bounding box of a point set; `None` when it is empty.
    pub fn from_points<I: IntoIterator<Item = Point>>(points: I) -> Option<Self> {
        let mut iter = points.into_iter();
        let first = iter.next()?;
        Some(iter.fold(BBox::point(first), |b, p| b.union(BBox::point(p))))
    }

    pub fn union(self, o: BBox) -> Self {
        BBox {
            min: Point::new(self.min.x.min(o.min.x), self.min.y.min(o.min.y)),
            max: Point::new(self.max.x.max(o.max.x), self.max.y.max(o.max.y)),
        }
    }

    /// Grow by `d` on every side (saturating at the `i32` range). A negative
    /// `d` shrinks, collapsing to the centre line rather than inverting.
    pub fn inflate(self, d: Coord) -> Self {
        let min = Point::new(self.min.x.saturating_sub(d), self.min.y.saturating_sub(d));
        let max = Point::new(self.max.x.saturating_add(d), self.max.y.saturating_add(d));
        let c = self.center();
        BBox {
            min: Point::new(min.x.min(c.x), min.y.min(c.y)),
            max: Point::new(max.x.max(c.x), max.y.max(c.y)),
        }
    }

    /// Whether the closed boxes share at least one point (touching counts).
    pub fn intersects(&self, o: &BBox) -> bool {
        self.min.x <= o.max.x
            && o.min.x <= self.max.x
            && self.min.y <= o.max.y
            && o.min.y <= self.max.y
    }

    pub fn contains(&self, p: Point) -> bool {
        self.min.x <= p.x && p.x <= self.max.x && self.min.y <= p.y && p.y <= self.max.y
    }

    pub fn contains_bbox(&self, o: &BBox) -> bool {
        self.contains(o.min) && self.contains(o.max)
    }

    pub fn width(&self) -> i64 {
        i64::from(self.max.x) - i64::from(self.min.x)
    }

    pub fn height(&self) -> i64 {
        i64::from(self.max.y) - i64::from(self.min.y)
    }

    /// The centre, rounded toward negative infinity.
    pub fn center(&self) -> Point {
        let mid = |a: Coord, b: Coord| ((i64::from(a) + i64::from(b)).div_euclid(2)) as Coord;
        Point::new(mid(self.min.x, self.max.x), mid(self.min.y, self.max.y))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn corners_normalize() {
        let b = BBox::new(Point::new(5, -1), Point::new(-3, 7));
        assert_eq!(b.min, Point::new(-3, -1));
        assert_eq!(b.max, Point::new(5, 7));
        assert_eq!((b.width(), b.height()), (8, 8));
    }

    #[test]
    fn touching_boxes_intersect() {
        let a = BBox::new(Point::new(0, 0), Point::new(10, 10));
        let b = BBox::new(Point::new(10, 10), Point::new(20, 20));
        let c = BBox::new(Point::new(11, 0), Point::new(20, 5));
        assert!(a.intersects(&b));
        assert!(!a.intersects(&c));
    }

    #[test]
    fn inflate_saturates_and_never_inverts() {
        let b = BBox::new(Point::new(i32::MAX - 1, 0), Point::new(i32::MAX, 10));
        assert_eq!(b.inflate(5).max.x, i32::MAX);
        let shrunk = BBox::new(Point::new(0, 0), Point::new(10, 4)).inflate(-3);
        assert_eq!(shrunk.min, Point::new(3, 2));
        assert_eq!(shrunk.max, Point::new(7, 2));
    }

    #[test]
    fn from_points_of_nothing_is_none() {
        assert_eq!(BBox::from_points(std::iter::empty()), None);
        let b = BBox::from_points([Point::new(1, 2), Point::new(-1, 5)]).unwrap();
        assert_eq!(b, BBox::new(Point::new(-1, 2), Point::new(1, 5)));
    }
}
