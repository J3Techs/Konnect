//! KiCad's track-angle verdict for one pair of tracks of a net.
//!
//! KiCad's `track_angle` test does not only look at track ends that meet at
//! one point. For every track it takes each connected track of the net,
//! intersects the two centre lines and, when they meet, measures the angle
//! there:
//!
//! - an end of a track that lies on the meeting point points its direction
//!   away from it;
//! - a track met somewhere along its length keeps its drawn direction, and
//!   the measured angle is folded below 90° (an angle above 90° loses 90°);
//! - two collinear tracks that overlap meet at the middle of the overlap.
//!
//! So a branch landing on the middle of a run at 45° or 135°, two runs of one
//! net crossing at 45°, and a run lying along another one the same way are
//! all reported, though no two ends share a point. [`pair_angle`] gives the
//! verdict for one pair; S7 ([`super::BoardChecker`]) and the optimizer's lint
//! ([`crate::optimize::lint`]) both use it. The behaviour was read from
//! KiCad's DRC and reproduced here; no code was taken from it.

use konnect_geom::{Coord, Point};

/// Where two segments meet and the angle KiCad measures there.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PairAngle {
    /// The meeting point of the centre lines.
    pub at: Point,
    /// The measured angle, degrees in `[0, 180]`.
    pub angle_deg: f64,
    /// Whether the point is an end of both segments: an ordinary joint, which
    /// the end-pairing check already judges.
    pub shared_vertex: bool,
    /// Whether the two run along one line.
    pub collinear: bool,
}

impl PairAngle {
    /// Whether this pair is judged here when its angle is under the
    /// minimum: everywhere but at an end both share (the end-pairing check
    /// judges those). Two collinear tracks overlapping the same way meet
    /// at 0° in the middle of the overlap and are reported, whether or not
    /// they start from one point: a live KiCad 10 DRC reports both.
    pub fn reported(&self, _a: (Point, Point), _b: (Point, Point)) -> bool {
        !self.shared_vertex
    }
}

/// `num * mul / div`, rounded half away from zero.
fn rescale(num: i128, mul: i128, div: i128) -> i128 {
    let p = num * mul;
    let negative = (p < 0) != (div < 0);
    let (p, d) = (p.abs(), div.abs());
    let q = (p + d / 2) / d;
    if negative {
        -q
    } else {
        q
    }
}

fn to_point(x: i128, y: i128) -> Option<Point> {
    Some(Point::new(
        Coord::try_from(x).ok()?,
        Coord::try_from(y).ok()?,
    ))
}

/// The point where segments `a` and `b` meet, as KiCad's segment
/// intersection gives it: ends count (a touch is a meeting), parallel
/// segments do not meet unless collinear, and collinear segments that
/// overlap meet at the middle of the overlap.
pub fn intersect(a: (Point, Point), b: (Point, Point)) -> Option<Point> {
    let (a0, a1) = a;
    let (b0, b1) = b;
    // Bounding boxes first.
    let (ax0, ax1) = (a0.x.min(a1.x), a0.x.max(a1.x));
    let (ay0, ay1) = (a0.y.min(a1.y), a0.y.max(a1.y));
    let (bx0, bx1) = (b0.x.min(b1.x), b0.x.max(b1.x));
    let (by0, by1) = (b0.y.min(b1.y), b0.y.max(b1.y));
    if ax1 < bx0 || bx1 < ax0 || ay1 < by0 || by1 < ay0 {
        return None;
    }
    let w = |p: Point| (i128::from(p.x), i128::from(p.y));
    let (ax, ay) = w(a0);
    let e = (i128::from(a1.x) - ax, i128::from(a1.y) - ay);
    let f = (
        i128::from(b1.x) - i128::from(b0.x),
        i128::from(b1.y) - i128::from(b0.y),
    );
    let ac = (i128::from(b0.x) - ax, i128::from(b0.y) - ay);
    let cross = |u: (i128, i128), v: (i128, i128)| u.0 * v.1 - u.1 * v.0;
    let d = cross(f, e);
    if d == 0 {
        if cross(e, ac) != 0 || e == (0, 0) {
            return None;
        }
        // Collinear: overlap along the longer axis of `a`.
        let along_x = e.0.abs() >= e.1.abs();
        let proj = |p: Point| {
            if along_x {
                i128::from(p.x)
            } else {
                i128::from(p.y)
            }
        };
        let lo = proj(a0).min(proj(a1)).max(proj(b0).min(proj(b1)));
        let hi = proj(a0).max(proj(a1)).min(proj(b0).max(proj(b1)));
        if lo > hi {
            return None;
        }
        // Integer division truncating towards zero, as C++ does.
        let mid = (lo + hi) / 2;
        return if along_x {
            to_point(mid, ay + rescale(mid - ax, e.1, e.0))
        } else {
            to_point(ax + rescale(mid - ay, e.0, e.1), mid)
        };
    }
    let p1 = cross(e, ac);
    let p2 = cross(f, ac);
    let inside = if d > 0 {
        (0..=d).contains(&p1) && (0..=d).contains(&p2)
    } else {
        (d..=0).contains(&p1) && (d..=0).contains(&p2)
    };
    if !inside {
        return None;
    }
    let (bx, by) = w(b0);
    to_point(bx + rescale(p1, f.0, d), by + rescale(p1, f.1, d))
}

fn unit(from: Point, to: Point) -> (f64, f64) {
    let (x, y) = (
        f64::from(to.x) - f64::from(from.x),
        f64::from(to.y) - f64::from(from.y),
    );
    let l = x.hypot(y);
    (x / l, y / l)
}

/// KiCad's verdict for two tracks of one net on one layer: where their
/// centre lines meet and the angle it measures there, or `None` when they
/// do not meet (or either is a point).
pub fn pair_angle(a: (Point, Point), b: (Point, Point)) -> Option<PairAngle> {
    if a.0 == a.1 || b.0 == b.1 {
        return None;
    }
    let at = intersect(a, b)?;
    let mut below_90 = false;
    let mut leave = |s: (Point, Point)| {
        let (x, y) = unit(s.0, s.1);
        if s.1 == at {
            (-x, -y)
        } else {
            if s.0 != at {
                below_90 = true;
            }
            (x, y)
        }
    };
    let (u, v) = (leave(a), leave(b));
    let dot = (u.0 * v.0 + u.1 * v.1).clamp(-1.0, 1.0);
    let mut angle_deg = dot.acos().to_degrees();
    if below_90 && angle_deg > 90.0 {
        angle_deg -= 90.0;
    }
    let end_of = |s: (Point, Point)| s.0 == at || s.1 == at;
    let cross = |s: (Point, Point), p: Point| {
        (i128::from(s.1.x) - i128::from(s.0.x)) * (i128::from(p.y) - i128::from(s.0.y))
            - (i128::from(s.1.y) - i128::from(s.0.y)) * (i128::from(p.x) - i128::from(s.0.x))
    };
    Some(PairAngle {
        at,
        angle_deg,
        shared_vertex: end_of(a) && end_of(b),
        collinear: cross(a, b.0) == 0 && cross(a, b.1) == 0,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const MM: i32 = 1_000_000;

    fn p(x: i32, y: i32) -> Point {
        Point::new(x, y)
    }

    #[test]
    fn a_shared_end_measures_the_bend() {
        let r = pair_angle((p(0, 0), p(MM, 0)), (p(MM, 0), p(MM, MM))).unwrap();
        assert_eq!(r.at, p(MM, 0));
        assert!((r.angle_deg - 90.0).abs() < 1e-9);
        assert!(r.shared_vertex);
        let r = pair_angle((p(0, 0), p(MM, 0)), (p(MM, 0), p(0, MM))).unwrap();
        assert!((r.angle_deg - 45.0).abs() < 1e-9);
    }

    #[test]
    fn a_branch_on_the_middle_of_a_run_is_folded_below_90() {
        // A 45 degree branch, either way round, measures 45.
        for b in [(p(MM, 0), p(2 * MM, MM)), (p(MM, 0), p(0, MM))] {
            let r = pair_angle((p(0, 0), p(3 * MM, 0)), b).unwrap();
            assert_eq!(r.at, p(MM, 0));
            assert!((r.angle_deg - 45.0).abs() < 1e-9, "{r:?}");
            assert!(!r.shared_vertex);
        }
        // A perpendicular branch is a clean T.
        let r = pair_angle((p(0, 0), p(3 * MM, 0)), (p(MM, 0), p(MM, MM))).unwrap();
        assert!((r.angle_deg - 90.0).abs() < 1e-9);
    }

    #[test]
    fn crossings_and_overlaps_meet_too() {
        let r = pair_angle((p(0, 0), p(4 * MM, 0)), (p(MM, -MM), p(3 * MM, MM))).unwrap();
        assert_eq!(r.at, p(2 * MM, 0));
        assert!((r.angle_deg - 45.0).abs() < 1e-9);
        // Collinear, overlapping, the same way: 0 at the overlap's middle.
        let (a, b) = ((p(0, 0), p(2 * MM, 0)), (p(MM, 0), p(3 * MM, 0)));
        let r = pair_angle(a, b).unwrap();
        assert_eq!(r.at, p(3 * MM / 2, 0));
        assert!(r.angle_deg.abs() < 1e-9);
        assert!(r.collinear && r.reported(a, b));
        // From a shared end too: a doubled run meets itself at 0°.
        let (a, b) = ((p(0, 0), p(2 * MM, 0)), (p(0, 0), p(MM, 0)));
        let r = pair_angle(a, b).unwrap();
        assert_eq!(r.at, p(MM / 2, 0));
        assert!(r.angle_deg.abs() < 1e-9 && r.reported(a, b));
        // Opposite ways the fold lands on 90.
        let r = pair_angle((p(0, 0), p(2 * MM, 0)), (p(3 * MM, 0), p(MM, 0))).unwrap();
        assert!((r.angle_deg - 90.0).abs() < 1e-9);
        // End to end in a straight line: a straight joint.
        let r = pair_angle((p(0, 0), p(MM, 0)), (p(MM, 0), p(2 * MM, 0))).unwrap();
        assert!(r.shared_vertex);
        assert!((r.angle_deg - 180.0).abs() < 1e-9);
        // Parallel and apart, or apart along one line: no meeting.
        assert!(pair_angle((p(0, 0), p(MM, 0)), (p(0, 50_000), p(MM, 50_000))).is_none());
        assert!(pair_angle((p(0, 0), p(MM, 0)), (p(2 * MM, 0), p(3 * MM, 0))).is_none());
    }
}
