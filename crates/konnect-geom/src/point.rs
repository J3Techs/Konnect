//! Points and difference vectors.

use crate::units::{mm_to_nm, nm_to_mm, Coord};
use serde::{Deserialize, Serialize};
use std::ops::Sub;

/// A board position in nanometres, in KiCad's Y-down board frame.
///
/// `Ord` is lexicographic (x, then y). It carries no geometric meaning; it
/// exists so collections of points sort the same way on every run.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Default, Serialize, Deserialize,
)]
pub struct Point {
    pub x: Coord,
    pub y: Coord,
}

impl Point {
    pub const fn new(x: Coord, y: Coord) -> Self {
        Point { x, y }
    }

    /// A point from millimetres; `None` if either coordinate cannot be
    /// represented (see [`mm_to_nm`]).
    pub fn from_mm(x: f64, y: f64) -> Option<Self> {
        Some(Point {
            x: mm_to_nm(x)?,
            y: mm_to_nm(y)?,
        })
    }

    /// The point in millimetres, for plans and reports.
    pub fn to_mm(self) -> (f64, f64) {
        (nm_to_mm(i64::from(self.x)), nm_to_mm(i64::from(self.y)))
    }

    /// `self + v`, or `None` when the result leaves the `i32` range.
    pub fn checked_add(self, v: Vector) -> Option<Self> {
        Some(Point {
            x: Coord::try_from(i64::from(self.x).checked_add(v.x)?).ok()?,
            y: Coord::try_from(i64::from(self.y).checked_add(v.y)?).ok()?,
        })
    }
}

/// The difference of two points. `i64` components, so the difference of any
/// two `i32` points is exact.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
pub struct Vector {
    pub x: i64,
    pub y: i64,
}

impl Vector {
    pub const fn new(x: i64, y: i64) -> Self {
        Vector { x, y }
    }

    /// Exact dot product.
    pub fn dot(self, o: Vector) -> i128 {
        i128::from(self.x) * i128::from(o.x) + i128::from(self.y) * i128::from(o.y)
    }

    /// Exact z-component of the cross product. Positive when `o` is
    /// counter-clockwise from `self` in a Y-up frame — which, because KiCad's
    /// board frame is Y-down, is clockwise as drawn on screen.
    pub fn cross(self, o: Vector) -> i128 {
        i128::from(self.x) * i128::from(o.y) - i128::from(self.y) * i128::from(o.x)
    }

    /// Exact squared length.
    pub fn length2(self) -> i128 {
        self.dot(self)
    }

    /// Approximate length, for costs and reports only.
    pub fn length_f64(self) -> f64 {
        (self.x as f64).hypot(self.y as f64)
    }
}

impl Sub for Point {
    type Output = Vector;

    fn sub(self, o: Point) -> Vector {
        Vector {
            x: i64::from(self.x) - i64::from(o.x),
            y: i64::from(self.y) - i64::from(o.y),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn differences_of_extreme_points_do_not_overflow() {
        let v = Point::new(i32::MAX, i32::MIN) - Point::new(i32::MIN, i32::MAX);
        assert_eq!(v.x, i64::from(i32::MAX) - i64::from(i32::MIN));
        assert_eq!(v.length2(), 2 * i128::from(v.x) * i128::from(v.x));
    }

    #[test]
    fn checked_add_refuses_overflow() {
        assert_eq!(Point::new(i32::MAX, 0).checked_add(Vector::new(1, 0)), None);
        assert_eq!(
            Point::new(1, 2).checked_add(Vector::new(3, -4)),
            Some(Point::new(4, -2))
        );
    }

    #[test]
    fn cross_sign_follows_the_y_up_convention() {
        let east = Vector::new(1, 0);
        let y_plus = Vector::new(0, 1);
        assert!(east.cross(y_plus) > 0);
        assert!(y_plus.cross(east) < 0);
    }
}
