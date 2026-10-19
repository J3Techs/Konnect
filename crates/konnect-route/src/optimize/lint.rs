//! A mirror of Konnect's `add_routing` geometry lint (`route_geometry::lint`
//! in `konnect-core`) for routed copper in nanometres.
//!
//! The optimizer uses it twice: as the local guard every candidate edit must
//! pass (an edit may never add a defect near it), and over the whole result,
//! so a run can say whether its plan will survive `geometry_check: strict`
//! before KiCad sees it.
//!
//! The lint itself works in millimetres with `f64`; this mirror converts
//! each coordinate exactly as [`crate::plan::RoutePlan`] does
//! ([`konnect_geom::nm_to_mm`]) and repeats the same arithmetic in the same
//! order, so the angles and lengths it compares are the very values the lint
//! will compute. Two things are stricter than the lint, never looser:
//!
//! - the lint clusters ends within 0.5 µm into one joint, greedily and in
//!   batch order. Here joints are exact points, and two *distinct* endpoints
//!   within [`NEAR_JOINT_NM`] are a defect of their own, so whenever this
//!   mirror is clean the lint's clustering equals exact grouping;
//! - a segment shorter than the rule table's `track_segment_length` minimum
//!   is a defect here (KiCad's DRC checks it; the lint does not).

use konnect_geom::{nm_to_mm, BBox, Coord, IndexEntry, LayerIndex, Point};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// Ends closer than this (mm) are one joint in the lint.
pub const JOIN_MM: f64 = 0.000_5;
/// Distinct endpoints at most this far apart (nm) are reported as
/// [`DefectKind::NearJoint`]: the lint would merge them order-dependently.
pub const NEAR_JOINT_NM: i64 = 1_000;
/// Bends at both ends of a jog or hairpin must exceed this (degrees).
const BEND_DEG: f64 = 20.0;
/// A same-direction pair of bends totalling more than this is a hairpin.
const HAIRPIN_DEG: f64 = 135.0;
/// Groups larger than this find T-junction candidates through an R-tree.
const INDEXED_GROUP: usize = 48;

/// The lint's limits in nanometres, plus the rule table's segment minimum.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct LintLimits {
    /// Segments shorter than this between two bends are jogs or hairpins
    /// (the lint's `jog_length_mm`; 0.25 mm by default).
    pub jog_length_nm: Coord,
    /// Bends sharper than this interior angle are acute; 180 is straight
    /// (the lint's `min_bend_angle_deg`).
    pub min_bend_angle_deg: f64,
    /// KiCad's `track_segment_length` minimum, when the rules set one.
    pub min_segment_length_nm: Option<Coord>,
}

impl Default for LintLimits {
    /// The lint's defaults: 0.25 mm jogs, 90 degree bends.
    fn default() -> Self {
        LintLimits {
            jog_length_nm: 250_000,
            min_bend_angle_deg: 90.0,
            min_segment_length_nm: None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DefectKind {
    ShortJog,
    Hairpin,
    AcuteBend,
    UnsplitTJunction,
    NearJoint,
    ShortSegment,
}

impl DefectKind {
    /// Whether `route_geometry::lint` itself reports this kind (the other
    /// two are this mirror's stricter additions).
    pub fn is_lint_kind(self) -> bool {
        !matches!(self, DefectKind::NearJoint | DefectKind::ShortSegment)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Defect {
    pub kind: DefectKind,
    pub at: Point,
}

/// One straight piece in a (net, layer) group.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LintSeg {
    pub a: Point,
    pub b: Point,
    pub width: Coord,
    /// Fixed board copper: it shapes the joints (KiCad checks angles against
    /// it) but is not judged for its own length.
    pub fixed: bool,
}

fn mm(p: Point) -> (f64, f64) {
    (nm_to_mm(i64::from(p.x)), nm_to_mm(i64::from(p.y)))
}

fn dist(p: (f64, f64), q: (f64, f64)) -> f64 {
    (p.0 - q.0).hypot(p.1 - q.1)
}

fn heading(from: (f64, f64), to: (f64, f64)) -> f64 {
    (to.1 - from.1).atan2(to.0 - from.0)
}

fn turn(h1: f64, h2: f64) -> f64 {
    let mut d = (h2 - h1).to_degrees();
    while d > 180.0 {
        d -= 360.0;
    }
    while d <= -180.0 {
        d += 360.0;
    }
    d
}

fn seg_point_distance(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let len2 = dx * dx + dy * dy;
    let t = if len2 == 0.0 {
        0.0
    } else {
        (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / len2).clamp(0.0, 1.0)
    };
    (dist(p, (a.0 + t * dx, a.1 + t * dy)), t)
}

/// The interior angle (degrees, 0..=180) between two directions leaving the
/// joint `p` towards `q1` and `q2`, computed as the lint does.
pub fn interior_angle_deg(p: Point, q1: Point, q2: Point) -> f64 {
    let p = mm(p);
    turn(heading(p, mm(q1)), heading(p, mm(q2))).abs()
}

/// Every defect in one (net, layer) group of segments.
pub fn lint_group(segs: &[LintSeg], limits: &LintLimits) -> Vec<Defect> {
    let mut out = Vec::new();
    if segs.is_empty() {
        return out;
    }
    // Joints: exact endpoint positions.
    let mut joints: BTreeMap<Point, Vec<(usize, bool)>> = BTreeMap::new();
    for (i, s) in segs.iter().enumerate() {
        joints.entry(s.a).or_default().push((i, true));
        joints.entry(s.b).or_default().push((i, false));
    }
    let other_end = |i: usize, at_a: bool| if at_a { segs[i].b } else { segs[i].a };

    // Distinct endpoints the lint would cluster: sweep by x.
    let points: Vec<Point> = joints.keys().copied().collect();
    for (k, p) in points.iter().enumerate() {
        for q in &points[k + 1..] {
            if i64::from(q.x) - i64::from(p.x) > NEAR_JOINT_NM {
                break;
            }
            let (dx, dy) = (
                i64::from(q.x) - i64::from(p.x),
                i64::from(q.y) - i64::from(p.y),
            );
            if dx * dx + dy * dy <= NEAR_JOINT_NM * NEAR_JOINT_NM {
                out.push(Defect {
                    kind: DefectKind::NearJoint,
                    at: *p,
                });
            }
        }
    }

    let mut turn_at: BTreeMap<(usize, bool), f64> = BTreeMap::new();
    for (p, ends) in &joints {
        let pm = mm(*p);
        for x in 0..ends.len() {
            for y in x + 1..ends.len() {
                let (i, ai) = ends[x];
                let (j, aj) = ends[y];
                let hi = heading(pm, mm(other_end(i, ai)));
                let hj = heading(pm, mm(other_end(j, aj)));
                let interior = turn(hi, hj).abs();
                if interior < limits.min_bend_angle_deg - 1e-9 {
                    out.push(Defect {
                        kind: DefectKind::AcuteBend,
                        at: *p,
                    });
                }
            }
        }
        if ends.len() == 2 {
            let (i, ai) = ends[0];
            let (j, aj) = ends[1];
            let into = heading(mm(other_end(i, ai)), pm);
            let out_h = heading(pm, mm(other_end(j, aj)));
            let t = turn(into, out_h);
            turn_at.insert((j, aj), t);
            turn_at.insert((i, ai), -t);
        }
    }

    let jog_mm = nm_to_mm(i64::from(limits.jog_length_nm));
    for (i, s) in segs.iter().enumerate() {
        let (a, b) = (mm(s.a), mm(s.b));
        let length = dist(a, b);
        if !s.fixed {
            if let Some(min) = limits.min_segment_length_nm {
                if (s.b - s.a).length2() < i128::from(min) * i128::from(min) {
                    out.push(Defect {
                        kind: DefectKind::ShortSegment,
                        at: midpoint(s.a, s.b),
                    });
                }
            }
        }
        if length >= jog_mm {
            continue;
        }
        let (Some(&ta), Some(&tb)) = (turn_at.get(&(i, true)), turn_at.get(&(i, false))) else {
            continue;
        };
        let enter = ta;
        let leave = -tb;
        if enter.abs() < BEND_DEG || leave.abs() < BEND_DEG {
            continue;
        }
        if enter.signum() != leave.signum() {
            out.push(Defect {
                kind: DefectKind::ShortJog,
                at: midpoint(s.a, s.b),
            });
        } else if enter.abs() + leave.abs() > HAIRPIN_DEG {
            out.push(Defect {
                kind: DefectKind::Hairpin,
                at: midpoint(s.a, s.b),
            });
        }
    }

    // Ends resting on the middle of another segment.
    let index = (segs.len() > INDEXED_GROUP).then(|| {
        LayerIndex::bulk_load(
            segs.iter()
                .enumerate()
                .map(|(i, s)| IndexEntry::new(i as u32, BBox::new(s.a, s.b).inflate(1_000)))
                .collect(),
        )
    });
    let mut candidates: Vec<u32> = Vec::new();
    for p in joints.keys() {
        let pm = mm(*p);
        candidates.clear();
        match &index {
            Some(index) => candidates.extend(index.query(&BBox::point(*p))),
            None => candidates.extend(0..segs.len() as u32),
        }
        for &k in &candidates {
            let s = &segs[k as usize];
            let (a, b) = (mm(s.a), mm(s.b));
            if dist(pm, a) <= JOIN_MM || dist(pm, b) <= JOIN_MM {
                continue;
            }
            let (d, t) = seg_point_distance(pm, a, b);
            if d <= JOIN_MM && t > 0.0 && t < 1.0 {
                out.push(Defect {
                    kind: DefectKind::UnsplitTJunction,
                    at: *p,
                });
            }
        }
    }
    out
}

/// The midpoint, rounded towards negative infinity.
pub fn midpoint(a: Point, b: Point) -> Point {
    Point::new(
        ((i64::from(a.x) + i64::from(b.x)).div_euclid(2)) as Coord,
        ((i64::from(a.y) + i64::from(b.y)).div_euclid(2)) as Coord,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn s(a: (i32, i32), b: (i32, i32)) -> LintSeg {
        LintSeg {
            a: Point::new(a.0, a.1),
            b: Point::new(b.0, b.1),
            width: 100_000,
            fixed: false,
        }
    }

    fn kinds(segs: &[LintSeg]) -> Vec<DefectKind> {
        let mut k: Vec<_> = lint_group(segs, &LintLimits::default())
            .into_iter()
            .map(|d| d.kind)
            .collect();
        k.sort();
        k
    }

    const MM: i32 = 1_000_000;

    #[test]
    fn the_lint_cases_are_mirrored() {
        // Along x, a 0.08 mm step, then along x again: a jog.
        let jog = [
            s((0, 0), (5 * MM, 0)),
            s((5 * MM, 0), (5 * MM, 80_000)),
            s((5 * MM, 80_000), (9 * MM, 80_000)),
        ];
        assert_eq!(kinds(&jog), [DefectKind::ShortJog]);
        // A 45 degree chamfer of a right angle is clean.
        let c = 141_421;
        let chamfer = [
            s((0, 0), (5 * MM, 0)),
            s((5 * MM, 0), (5 * MM + c, c)),
            s((5 * MM + c, c), (5 * MM + c, 5 * MM)),
        ];
        assert!(kinds(&chamfer).is_empty());
        // A short U-turn is a hairpin.
        let hairpin = [
            s((0, 0), (5 * MM, 0)),
            s((5 * MM, 0), (5 * MM, 100_000)),
            s((5 * MM, 100_000), (0, 100_000)),
        ];
        assert_eq!(kinds(&hairpin), [DefectKind::Hairpin]);
        // Acute and T junctions.
        let sharp = [s((0, 0), (5 * MM, 0)), s((5 * MM, 0), (0, MM))];
        assert_eq!(kinds(&sharp), [DefectKind::AcuteBend]);
        let t = [s((0, 0), (2 * MM, 0)), s((MM, 0), (MM, MM))];
        assert_eq!(kinds(&t), [DefectKind::UnsplitTJunction]);
        // A right angle passes at 90 and at 89 degrees.
        let right = [s((0, 0), (MM, 0)), s((MM, 0), (MM, MM))];
        assert!(kinds(&right).is_empty());
    }

    #[test]
    fn near_joints_and_short_segments_are_stricter_additions() {
        let near = [s((0, 0), (MM, 0)), s((MM + 400, 0), (MM + 400, MM))];
        assert_eq!(kinds(&near), [DefectKind::NearJoint]);
        let limits = LintLimits {
            min_segment_length_nm: Some(200_000),
            ..LintLimits::default()
        };
        let short = [s((0, 0), (100_000, 0))];
        let d = lint_group(&short, &limits);
        assert_eq!(d.len(), 1);
        assert_eq!(d[0].kind, DefectKind::ShortSegment);
        assert!(!d[0].kind.is_lint_kind());
    }

    #[test]
    fn indexed_t_search_matches_brute_force() {
        // A long comb: one spine and many teeth ending on it.
        let mut segs = vec![s((0, 0), (100 * MM, 0))];
        for i in 1..60 {
            segs.push(s((i * MM, 0), (i * MM, MM)));
        }
        let d = lint_group(&segs, &LintLimits::default());
        assert_eq!(
            d.iter()
                .filter(|d| d.kind == DefectKind::UnsplitTJunction)
                .count(),
            59
        );
    }
}
