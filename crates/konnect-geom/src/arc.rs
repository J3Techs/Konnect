//! Arc approximation policy.
//!
//! Arcs are the one primitive the exact kernel does not carry: an arc track,
//! a rounded zone corner or a circular courtyard is lowered to straight
//! pieces under an explicit [`ArcPolicy`], and every consumer of the result
//! must budget for the approximation error:
//!
//! - [`ArcSide::Inscribed`] puts every vertex *on* the arc, so the chords lie
//!   up to `max_error_nm` inside it. Use it for copper the router will emit or
//!   connect to (an arc track's centre line), and add `max_error_nm` to any
//!   clearance checked against the result.
//! - [`ArcSide::Circumscribed`] puts every chord on or outside the arc: the
//!   chords are tangents, their corners pushed out and rounded away from the
//!   centre, so the result contains the true curve and lies at most about
//!   `max_error_nm` (plus 2 nm of rounding headroom) outside it. Use it for
//!   obstacles, courtyards and keepouts, which may then only over-protect.
//!
//! The vertex count is the smallest that keeps the chord sagitta at or below
//! `max_error_nm`, so a given arc and policy always produce the same points.

use crate::point::Point;
use crate::units::Coord;
use std::f64::consts::PI;

/// How finely arcs are cut into straight pieces.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ArcPolicy {
    /// Largest distance between the true curve and its approximation, nm.
    pub max_error_nm: Coord,
    /// Fewest pieces for a full circle, so tiny circles stay round.
    pub min_segments_per_circle: u32,
}

impl Default for ArcPolicy {
    /// 1 µm: a fifth of KiCad's own default arc error (5 µm), so the internal
    /// checker's curved-primitive margin stays well inside KiCad's.
    fn default() -> Self {
        ArcPolicy {
            max_error_nm: 1_000,
            min_segments_per_circle: 8,
        }
    }
}

/// Which side of the true curve the approximation lies on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ArcSide {
    Inscribed,
    Circumscribed,
}

/// Pieces needed for `sweep` radians of radius `radius` under `policy`.
pub fn segment_count(radius: f64, sweep: f64, policy: &ArcPolicy) -> u32 {
    let sweep = sweep.abs().min(2.0 * PI);
    let err = f64::from(policy.max_error_nm.max(1));
    let per_circle_floor = f64::from(policy.min_segments_per_circle.max(3));
    let step = if radius <= err {
        2.0 * PI / per_circle_floor
    } else {
        (2.0 * (1.0 - err / radius).acos()).min(2.0 * PI / per_circle_floor)
    };
    ((sweep / step).ceil() as u32).max(1)
}

/// The circle through three points: `(cx, cy, r)` in nm, or `None` when
/// they are collinear (or coincident).
fn circumcircle(a: Point, b: Point, c: Point) -> Option<(f64, f64, f64)> {
    // Work relative to `a`: board-scale absolute coordinates squared would
    // cost the precision a small fillet needs.
    let (ab, ac) = (b - a, c - a);
    let cross = ab.cross(ac);
    if cross == 0 {
        return None;
    }
    let (bx, by) = (ab.x as f64, ab.y as f64);
    let (cx, cy) = (ac.x as f64, ac.y as f64);
    let d = 2.0 * cross as f64;
    let (qb, qc) = (bx * bx + by * by, cx * cx + cy * cy);
    let ux = (cy * qb - by * qc) / d;
    let uy = (bx * qc - cx * qb) / d;
    Some((f64::from(a.x) + ux, f64::from(a.y) + uy, ux.hypot(uy)))
}

/// A KiCad three-point arc (`start`, any interior `mid`, `end`) as a polyline
/// from `start` to `end`. The first and last points are exactly `start` and
/// `end`; collinear input is the straight segment.
pub fn approximate_arc(
    start: Point,
    mid: Point,
    end: Point,
    policy: &ArcPolicy,
    side: ArcSide,
) -> Vec<Point> {
    let Some((cx, cy, r)) = circumcircle(start, mid, end) else {
        return vec![start, end];
    };
    let angle = |p: Point| (f64::from(p.y) - cy).atan2(f64::from(p.x) - cx);
    let (a_s, a_m, a_e) = (angle(start), angle(mid), angle(end));
    let tau = 2.0 * PI;
    // The sweep from start that passes through mid, signed.
    let ccw_e = (a_e - a_s).rem_euclid(tau);
    let ccw_m = (a_m - a_s).rem_euclid(tau);
    let sweep = if ccw_m <= ccw_e { ccw_e } else { ccw_e - tau };
    let n = segment_count(r, sweep, policy);
    let step = sweep / f64::from(n);
    let mut out = Vec::with_capacity(n as usize + 2);
    out.push(start);
    match side {
        ArcSide::Inscribed => {
            for i in 1..n {
                let t = a_s + step * f64::from(i);
                out.push(round_point(cx + r * t.cos(), cy + r * t.sin()));
            }
        }
        // Tangents at the endpoints and at every step between: their
        // corners sit half a step apart from the ends, so the first and last
        // chords are the tangents at `start` and `end` and none cuts inside.
        ArcSide::Circumscribed => {
            for i in 1..=n {
                let t = a_s + step * (f64::from(i) - 0.5);
                out.push(outer_vertex(cx, cy, r, step, t));
            }
        }
    }
    out.push(end);
    out.dedup();
    out
}

/// A full circle as a closed ring (first point not repeated).
pub fn approximate_circle(
    center: Point,
    radius: Coord,
    policy: &ArcPolicy,
    side: ArcSide,
) -> Vec<Point> {
    let r = f64::from(radius.max(0));
    let n = segment_count(r, 2.0 * PI, policy).max(policy.min_segments_per_circle.max(3));
    let step = 2.0 * PI / f64::from(n);
    let (cx, cy) = (f64::from(center.x), f64::from(center.y));
    let mut ring: Vec<Point> = (0..n)
        .map(|i| {
            let t = step * f64::from(i);
            match side {
                ArcSide::Inscribed => round_point(cx + r * t.cos(), cy + r * t.sin()),
                ArcSide::Circumscribed => outer_vertex(cx, cy, r, step, t),
            }
        })
        .collect();
    ring.dedup();
    ring
}

/// Round to the nearest nanometre: the half-nanometre this can move an
/// inscribed vertex is far inside any `max_error_nm` budget.
fn round_point(x: f64, y: f64) -> Point {
    Point::new(x.round() as Coord, y.round() as Coord)
}

/// The corner of two tangents `step` apart, at angle `t`: radius
/// `r / cos(step / 2)`, pushed out 2 nm and rounded away from the centre so
/// neither chord it ends can move inside the circle.
fn outer_vertex(cx: f64, cy: f64, r: f64, step: f64, t: f64) -> Point {
    let vr = r / (step / 2.0).cos() + 2.0;
    let (x, y) = (cx + vr * t.cos(), cy + vr * t.sin());
    let away = |v: f64, c: f64| if v >= c { v.ceil() } else { v.floor() };
    Point::new(away(x, cx) as Coord, away(y, cy) as Coord)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dist(p: Point, cx: f64, cy: f64) -> f64 {
        (f64::from(p.x) - cx).hypot(f64::from(p.y) - cy)
    }

    #[test]
    fn inscribed_vertices_lie_on_the_arc() {
        let policy = ArcPolicy::default();
        // Quarter circle of radius 1 mm around the origin.
        let r = 1_000_000;
        let pts = approximate_arc(
            Point::new(r, 0),
            Point::new(707_107, 707_107),
            Point::new(0, r),
            &policy,
            ArcSide::Inscribed,
        );
        assert_eq!(pts.first(), Some(&Point::new(r, 0)));
        assert_eq!(pts.last(), Some(&Point::new(0, r)));
        for p in &pts {
            assert!(
                (dist(*p, 0.0, 0.0) - f64::from(r)).abs() <= 1.0,
                "{p:?} off the arc"
            );
        }
        // Sagitta bound: chord midpoints are within max_error of the arc.
        for w in pts.windows(2) {
            let mx = (f64::from(w[0].x) + f64::from(w[1].x)) / 2.0;
            let my = (f64::from(w[0].y) + f64::from(w[1].y)) / 2.0;
            let sag = f64::from(r) - mx.hypot(my);
            assert!(sag <= f64::from(policy.max_error_nm) + 1.0, "sagitta {sag}");
        }
    }

    #[test]
    fn sweep_direction_follows_mid() {
        let policy = ArcPolicy::default();
        let r = 1_000_000;
        // Same endpoints, mid on the far side: a three-quarter arc.
        let pts = approximate_arc(
            Point::new(r, 0),
            Point::new(-707_107, -707_107),
            Point::new(0, r),
            &policy,
            ArcSide::Inscribed,
        );
        assert!(pts.iter().any(|p| p.x < -900_000), "should pass through -x");
    }

    #[test]
    fn circumscribed_ring_contains_the_circle() {
        let policy = ArcPolicy::default();
        let ring = approximate_circle(Point::new(0, 0), 225_000, &policy, ArcSide::Circumscribed);
        assert!(ring.len() >= 8);
        for w in 0..ring.len() {
            let (a, b) = (ring[w], ring[(w + 1) % ring.len()]);
            assert!(
                seg_dist(a, b, 0.0, 0.0) >= 225_000.0,
                "chord cuts inside the circle"
            );
        }
    }

    /// Closest distance from `(cx, cy)` to the segment `a`–`b`.
    fn seg_dist(a: Point, b: Point, cx: f64, cy: f64) -> f64 {
        let (ax, ay, bx, by) = (
            f64::from(a.x),
            f64::from(a.y),
            f64::from(b.x),
            f64::from(b.y),
        );
        let (dx, dy) = (bx - ax, by - ay);
        let len2 = dx * dx + dy * dy;
        let t = if len2 == 0.0 {
            0.0
        } else {
            (((cx - ax) * dx + (cy - ay) * dy) / len2).clamp(0.0, 1.0)
        };
        (ax + t * dx - cx).hypot(ay + t * dy - cy)
    }

    #[test]
    fn circumscribed_partial_arc_chords_never_cut_inside() {
        let policy = ArcPolicy::default();
        let r = 1_000_000;
        for (mid, end) in [
            // Quarter, three-quarter and a short arc.
            (Point::new(707_107, 707_107), Point::new(0, r)),
            (Point::new(-707_107, -707_107), Point::new(0, r)),
            (Point::new(996_195, 87_156), Point::new(984_808, 173_648)),
        ] {
            let start = Point::new(r, 0);
            let pts = approximate_arc(start, mid, end, &policy, ArcSide::Circumscribed);
            assert_eq!(pts.first(), Some(&start));
            assert_eq!(pts.last(), Some(&end));
            for w in pts.windows(2) {
                let d = seg_dist(w[0], w[1], 0.0, 0.0);
                assert!(d >= f64::from(r) - 0.5, "chord {w:?} dips to {d}");
            }
            for p in &pts {
                let out = dist(*p, 0.0, 0.0) - f64::from(r);
                assert!(
                    out <= 1.1 * f64::from(policy.max_error_nm) + 4.0,
                    "{p:?} {out} nm out"
                );
            }
        }
    }

    #[test]
    fn collinear_arc_is_a_segment() {
        let pts = approximate_arc(
            Point::new(0, 0),
            Point::new(5, 5),
            Point::new(10, 10),
            &ArcPolicy::default(),
            ArcSide::Inscribed,
        );
        assert_eq!(pts, vec![Point::new(0, 0), Point::new(10, 10)]);
    }
}
