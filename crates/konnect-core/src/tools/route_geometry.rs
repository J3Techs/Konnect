//! Geometry lint for routed copper before it reaches the board.
//!
//! KiCad's DRC judges clearances, not shape. A planner or autorouter can
//! still hand over copper that passes DRC but is poor practice: a short
//! sideways step between two bends (a jog), a short U-turn (a hairpin), an
//! acute bend, a redundant vertex on a straight run, or a track ending on the
//! middle of another without a vertex there. This module finds those in a set
//! of straight segments, joined wherever two ends of the same net and layer
//! meet, and reports where they are. `lint_board_geometry` runs the same lint
//! over a whole saved board, so joints and the T-junction search are hashed
//! rather than scanned.

use serde_json::json;
use std::collections::{BTreeMap, HashMap};

/// A joint's position and the (segment, is-start-end) ends meeting there.
pub(crate) type Joint = ((f64, f64), Vec<(usize, bool)>);

/// Ends closer than this (mm) are one joint.
pub(crate) const JOIN_MM: f64 = 0.000_5;
/// Turns smaller than this (degrees) count as straight.
pub(crate) const STRAIGHT_DEG: f64 = 1.0;
/// Bends at both ends of a jog or hairpin must exceed this (degrees).
const BEND_DEG: f64 = 20.0;
/// A same-direction pair of bends totalling more than this is a hairpin.
const HAIRPIN_DEG: f64 = 135.0;
/// Findings listed per kind; the counts always cover every finding.
const LISTED: usize = 25;
/// Grid cell (mm) the T-junction search buckets segments into.
const T_CELL_MM: f64 = 1.0;

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Segment {
    pub net: String,
    pub layer: String,
    pub width: f64,
    pub a: (f64, f64),
    pub b: (f64, f64),
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Limits {
    /// Segments shorter than this between two bends are jogs or hairpins.
    pub jog_length_mm: f64,
    /// Bends sharper than this interior angle are acute (180 is straight).
    pub min_bend_angle_deg: f64,
}

impl Default for Limits {
    fn default() -> Self {
        Self {
            jog_length_mm: 0.25,
            min_bend_angle_deg: 90.0,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Finding {
    pub kind: &'static str,
    pub net: String,
    pub layer: String,
    pub at: (f64, f64),
    pub detail: String,
    /// The segments involved, by index into the linted slice: the two
    /// meeting at a bend or vertex; the one before, the short one and the
    /// one after for a jog or hairpin; the ending one(s), then the passing
    /// one, for a T-junction.
    pub segments: Vec<usize>,
    /// Interior angles in degrees (180 is straight): one for a bend or a
    /// vertex, the two bends of a jog or hairpin, none for a T-junction.
    pub angles_deg: Vec<f64>,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub(crate) struct Report {
    pub findings: Vec<Finding>,
}

impl Report {
    fn count(&self, kind: &str) -> usize {
        self.findings.iter().filter(|f| f.kind == kind).count()
    }

    /// Findings that make a batch poor practice; collinear vertices are only
    /// untidy and are reported without counting against it.
    pub(crate) fn defects(&self) -> usize {
        self.findings
            .iter()
            .filter(|f| f.kind != "collinear_vertex")
            .count()
    }

    pub(crate) fn to_json(&self, limits: &Limits) -> serde_json::Value {
        let kinds = [
            "short_jog",
            "hairpin",
            "acute_bend",
            "unsplit_t_junction",
            "collinear_vertex",
        ];
        let mut listed = serde_json::Map::new();
        for kind in kinds {
            let items: Vec<_> = self
                .findings
                .iter()
                .filter(|f| f.kind == kind)
                .take(LISTED)
                .map(|f| {
                    json!({
                        "net": f.net, "layer": f.layer,
                        "x": round4(f.at.0), "y": round4(f.at.1),
                        "detail": f.detail
                    })
                })
                .collect();
            if !items.is_empty() {
                listed.insert(kind.to_string(), json!(items));
            }
        }
        json!({
            "clean": self.defects() == 0,
            "counts": {
                "short_jogs": self.count("short_jog"),
                "hairpins": self.count("hairpin"),
                "acute_bends": self.count("acute_bend"),
                "unsplit_t_junctions": self.count("unsplit_t_junction"),
                "collinear_vertices": self.count("collinear_vertex"),
            },
            "limits": {
                "jog_length_mm": limits.jog_length_mm,
                "min_bend_angle_deg": limits.min_bend_angle_deg,
            },
            "findings": listed,
        })
    }
}

pub(crate) fn round4(v: f64) -> f64 {
    (v * 10_000.0).round() / 10_000.0
}

pub(crate) fn dist(p: (f64, f64), q: (f64, f64)) -> f64 {
    (p.0 - q.0).hypot(p.1 - q.1)
}

/// Direction (radians) of travel from `from` to `to`.
pub(crate) fn heading(from: (f64, f64), to: (f64, f64)) -> f64 {
    (to.1 - from.1).atan2(to.0 - from.0)
}

/// Signed turn (degrees, -180..180) from heading `h1` to heading `h2`.
pub(crate) fn turn(h1: f64, h2: f64) -> f64 {
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

fn cell_of(p: (f64, f64), size: f64) -> (i64, i64) {
    ((p.0 / size).floor() as i64, (p.1 / size).floor() as i64)
}

/// The joints of one group: clusters of segment ends within [`JOIN_MM`],
/// each at the end that opened it, in order of first appearance. Joints are
/// found through a hash of [`JOIN_MM`] cells, so a group of any size is
/// clustered in linear time; the earliest joint within reach wins, as a scan
/// in order would choose.
fn joints_of(segments: &[Segment], members: &[usize]) -> Vec<Joint> {
    let mut joints: Vec<Joint> = Vec::new();
    let mut cells: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
    for &i in members {
        for (at_a, p) in [(true, segments[i].a), (false, segments[i].b)] {
            let (cx, cy) = cell_of(p, JOIN_MM);
            let mut found: Option<usize> = None;
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for &j in cells.get(&(cx + dx, cy + dy)).into_iter().flatten() {
                        if dist(joints[j].0, p) <= JOIN_MM && found.is_none_or(|f| j < f) {
                            found = Some(j);
                        }
                    }
                }
            }
            match found {
                Some(j) => joints[j].1.push((i, at_a)),
                None => {
                    cells.entry((cx, cy)).or_default().push(joints.len());
                    joints.push((p, vec![(i, at_a)]));
                }
            }
        }
    }
    joints
}

/// Lint `segments`. Joints are formed only among the given segments, so a
/// batch is judged on its own geometry.
pub(crate) fn lint(segments: &[Segment], limits: &Limits) -> Report {
    let mut report = Report::default();
    let mut groups: BTreeMap<(&str, &str), Vec<usize>> = BTreeMap::new();
    for (i, s) in segments.iter().enumerate() {
        groups
            .entry((s.net.as_str(), s.layer.as_str()))
            .or_default()
            .push(i);
    }
    for ((net, layer), members) in groups {
        let joints = joints_of(segments, &members);
        let other_end = |i: usize, at_a: bool| {
            if at_a {
                segments[i].b
            } else {
                segments[i].a
            }
        };
        let finding = |kind, at, detail: String, segs: Vec<usize>, angles: Vec<f64>| Finding {
            kind,
            net: net.to_string(),
            layer: layer.to_string(),
            at,
            detail,
            segments: segs,
            angles_deg: angles,
        };
        // At each degree-2 joint, the signed turn made on entering a segment
        // there from its neighbour, and that neighbour, keyed by
        // (segment, end).
        let mut turn_at: BTreeMap<(usize, bool), (f64, usize)> = BTreeMap::new();
        for (p, ends) in &joints {
            for x in 0..ends.len() {
                for y in x + 1..ends.len() {
                    let (i, ai) = ends[x];
                    let (j, aj) = ends[y];
                    let hi = heading(*p, other_end(i, ai));
                    let hj = heading(*p, other_end(j, aj));
                    let interior = turn(hi, hj).abs();
                    if interior < limits.min_bend_angle_deg - 1e-9 {
                        report.findings.push(finding(
                            "acute_bend",
                            *p,
                            format!("{interior:.1} deg between two segments"),
                            vec![i, j],
                            vec![interior],
                        ));
                    }
                }
            }
            if ends.len() == 2 {
                let (i, ai) = ends[0];
                let (j, aj) = ends[1];
                // Travel along i into the joint, then out along j.
                let into = heading(other_end(i, ai), *p);
                let out = heading(*p, other_end(j, aj));
                let t = turn(into, out);
                if t.abs() < STRAIGHT_DEG && segments[i].width == segments[j].width {
                    report.findings.push(finding(
                        "collinear_vertex",
                        *p,
                        "vertex on a straight run".to_string(),
                        vec![i, j],
                        vec![180.0 - t.abs()],
                    ));
                }
                // Entering j from i turns by t; entering i from j, the same
                // bend travelled the other way, by -t.
                turn_at.insert((j, aj), (t, i));
                turn_at.insert((i, ai), (-t, j));
            }
        }
        // Jogs and hairpins: a short segment with real bends at both ends.
        for &i in &members {
            let s = &segments[i];
            let length = dist(s.a, s.b);
            if length >= limits.jog_length_mm {
                continue;
            }
            let (Some(&(ta, before)), Some(&(tb, after))) =
                (turn_at.get(&(i, true)), turn_at.get(&(i, false)))
            else {
                continue;
            };
            // Travelling a -> b: the turn entering s at a, and the turn
            // leaving s at b (entering s at b, travelled the other way).
            let enter = ta;
            let leave = -tb;
            if enter.abs() < BEND_DEG || leave.abs() < BEND_DEG {
                continue;
            }
            let mid = ((s.a.0 + s.b.0) / 2.0, (s.a.1 + s.b.1) / 2.0);
            let angles = vec![180.0 - enter.abs(), 180.0 - leave.abs()];
            if enter.signum() != leave.signum() {
                report.findings.push(finding(
                    "short_jog",
                    mid,
                    format!(
                        "{length:.3} mm step between bends of {:.0} and {:.0} deg",
                        enter.abs(),
                        leave.abs()
                    ),
                    vec![before, i, after],
                    angles,
                ));
            } else if enter.abs() + leave.abs() > HAIRPIN_DEG {
                report.findings.push(finding(
                    "hairpin",
                    mid,
                    format!(
                        "{length:.3} mm segment turning {:.0} deg in total",
                        enter.abs() + leave.abs()
                    ),
                    vec![before, i, after],
                    angles,
                ));
            }
        }
        // Ends resting on the middle of another segment of the same net.
        // Segments are bucketed by the cells their (slightly grown) boxes
        // cover, so each joint is tested only against those in its cell.
        let mut cells: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
        for &k in &members {
            let s = &segments[k];
            let lo = (s.a.0.min(s.b.0) - JOIN_MM, s.a.1.min(s.b.1) - JOIN_MM);
            let hi = (s.a.0.max(s.b.0) + JOIN_MM, s.a.1.max(s.b.1) + JOIN_MM);
            let (x0, y0) = cell_of(lo, T_CELL_MM);
            let (x1, y1) = cell_of(hi, T_CELL_MM);
            for cx in x0..=x1 {
                for cy in y0..=y1 {
                    cells.entry((cx, cy)).or_default().push(k);
                }
            }
        }
        for (p, ends) in &joints {
            let mut near: Vec<usize> = cells
                .get(&cell_of(*p, T_CELL_MM))
                .cloned()
                .unwrap_or_default();
            near.sort_unstable();
            for k in near {
                let s = &segments[k];
                if dist(*p, s.a) <= JOIN_MM || dist(*p, s.b) <= JOIN_MM {
                    continue;
                }
                let (d, t) = seg_point_distance(*p, s.a, s.b);
                if d <= JOIN_MM && t > 0.0 && t < 1.0 {
                    let mut segs: Vec<usize> = ends.iter().map(|e| e.0).collect();
                    segs.push(k);
                    report.findings.push(finding(
                        "unsplit_t_junction",
                        *p,
                        "a track ends on the middle of another; add a vertex there".to_string(),
                        segs,
                        Vec::new(),
                    ));
                }
            }
        }
    }
    report
}

/// Parse `geometry_check` and its limits from tool arguments.
pub(crate) fn parse_args(
    args: &serde_json::Value,
) -> Result<(&'static str, Limits), (String, String)> {
    let mode = match args
        .get("geometry_check")
        .and_then(serde_json::Value::as_str)
    {
        None => "warn",
        Some("warn") => "warn",
        Some("strict") => "strict",
        Some("off") => "off",
        Some(other) => {
            return Err((
                "geometry_check".to_string(),
                format!("'{other}' is not warn, strict or off"),
            ))
        }
    };
    Ok((mode, parse_limits(args)?))
}

/// `jog_length_mm` and `min_bend_angle_deg`, each defaulted when absent.
pub(crate) fn parse_limits(args: &serde_json::Value) -> Result<Limits, (String, String)> {
    let mut limits = Limits::default();
    if let Some(v) = args.get("jog_length_mm").filter(|v| !v.is_null()) {
        match v.as_f64() {
            Some(v) if v > 0.0 && v <= 10.0 => limits.jog_length_mm = v,
            _ => {
                return Err((
                    "jog_length_mm".to_string(),
                    "must be above 0 and at most 10".to_string(),
                ))
            }
        }
    }
    if let Some(v) = args.get("min_bend_angle_deg").filter(|v| !v.is_null()) {
        match v.as_f64() {
            Some(v) if (0.0..=180.0).contains(&v) => limits.min_bend_angle_deg = v,
            _ => {
                return Err((
                    "min_bend_angle_deg".to_string(),
                    "must be from 0 to 180".to_string(),
                ))
            }
        }
    }
    Ok(limits)
}

pub(crate) fn schema_properties() -> serde_json::Value {
    json!({
        "geometry_check": {
            "type": "string",
            "enum": ["warn", "strict", "off"],
            "default": "warn",
            "description": "Lint the copper's shape: short jogs (a short sideways step between bends), hairpins, acute bends, tracks ending on the middle of another, and redundant vertices. 'warn' reports, 'strict' refuses copper with any defect, 'off' skips the check."
        },
        "jog_length_mm": {
            "type": "number",
            "default": 0.25,
            "description": "Segments shorter than this between two bends count as jogs or hairpins"
        },
        "min_bend_angle_deg": {
            "type": "number",
            "default": 90,
            "description": "Smallest allowed angle between two connected segments (180 is straight)"
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn seg(a: (f64, f64), b: (f64, f64)) -> Segment {
        Segment {
            net: "N".into(),
            layer: "F.Cu".into(),
            width: 0.1,
            a,
            b,
        }
    }

    fn kinds(segments: &[Segment]) -> Vec<&'static str> {
        lint(segments, &Limits::default())
            .findings
            .iter()
            .map(|f| f.kind)
            .collect()
    }

    #[test]
    fn a_sideways_step_is_a_jog() {
        // Along x, step 0.08 up, carry on along x.
        let s = [
            seg((0.0, 0.0), (5.0, 0.0)),
            seg((5.0, 0.0), (5.0, 0.08)),
            seg((5.0, 0.08), (9.0, 0.08)),
        ];
        assert_eq!(kinds(&s), ["short_jog"]);
        let jog = &lint(&s, &Limits::default()).findings[0];
        assert_eq!(jog.segments, [0, 1, 2], "before, the step, after");
        assert_eq!(jog.angles_deg, [90.0, 90.0]);
    }

    #[test]
    fn a_usb_pair_split_jog_is_caught() {
        // A USB pair's DP_ESD net as first placed: a 0.09 mm step off the
        // pair partner where the pair split towards the ESD part.
        let s = [
            seg((162.25, 9.5071), (155.3873, 10.3458)),
            seg((155.3873, 10.3458), (155.3763, 10.2559)),
            seg((155.3763, 10.2559), (152.85, 10.825)),
        ];
        // Its step back towards the pair is also sharper than 90 degrees.
        assert_eq!(kinds(&s), ["acute_bend", "short_jog"]);
    }

    #[test]
    fn a_mitred_corner_is_not_a_jog() {
        // 90 degree corner cut by a 0.2 mm 45 degree chamfer.
        let c = 0.2 / 2f64.sqrt();
        let s = [
            seg((0.0, 0.0), (5.0, 0.0)),
            seg((5.0, 0.0), (5.0 + c, c)),
            seg((5.0 + c, c), (5.0 + c, 5.0)),
        ];
        assert!(kinds(&s).is_empty(), "{:?}", kinds(&s));
    }

    #[test]
    fn a_short_u_turn_is_a_hairpin_and_its_bends_are_acute() {
        let s = [
            seg((0.0, 0.0), (5.0, 0.0)),
            seg((5.0, 0.0), (5.0, 0.1)),
            seg((5.0, 0.1), (0.0, 0.1)),
        ];
        assert_eq!(kinds(&s), ["hairpin"]);
        let sharp = [seg((0.0, 0.0), (5.0, 0.0)), seg((5.0, 0.0), (0.0, 1.0))];
        assert_eq!(kinds(&sharp), ["acute_bend"]);
        let bend = &lint(&sharp, &Limits::default()).findings[0];
        assert_eq!(bend.segments, [0, 1]);
        assert!((bend.angles_deg[0] - 11.31).abs() < 0.01, "{bend:?}");
    }

    #[test]
    fn straight_vertices_and_mid_segment_ends_are_reported() {
        let s = [seg((0.0, 0.0), (1.0, 0.0)), seg((1.0, 0.0), (2.0, 0.0))];
        assert_eq!(kinds(&s), ["collinear_vertex"]);
        let t = [seg((0.0, 0.0), (2.0, 0.0)), seg((1.0, 0.0), (1.0, 1.0))];
        assert_eq!(kinds(&t), ["unsplit_t_junction"]);
        assert_eq!(lint(&t, &Limits::default()).findings[0].segments, [1, 0]);
        let report = lint(&s, &Limits::default());
        assert_eq!(report.defects(), 0, "a collinear vertex is not a defect");
    }

    #[test]
    fn a_t_junction_is_found_on_a_long_run_far_from_its_ends() {
        // The passing track spans many grid cells; the joint lies in one of
        // the middle ones.
        let t = [seg((0.0, 0.0), (80.0, 0.0)), seg((41.3, 0.0), (41.3, 3.0))];
        assert_eq!(kinds(&t), ["unsplit_t_junction"]);
    }

    #[test]
    fn ends_within_half_a_micron_join_and_farther_ones_do_not() {
        // A right angle, taken for acute under a 100 degree minimum, only
        // when the two ends join.
        let limits = Limits {
            min_bend_angle_deg: 100.0,
            ..Limits::default()
        };
        let kinds_at = |s: &[Segment]| -> Vec<&'static str> {
            lint(s, &limits).findings.iter().map(|f| f.kind).collect()
        };
        let near = [
            seg((0.0, 0.0), (5.0, 0.0)),
            seg((5.0004, 0.0), (5.0004, 1.0)),
        ];
        assert_eq!(kinds_at(&near), ["acute_bend"]);
        let apart = [
            seg((0.0, 0.0), (5.0, 0.0)),
            seg((5.0006, 0.0), (5.0006, 1.0)),
        ];
        assert!(kinds_at(&apart).is_empty());
    }

    #[test]
    fn other_nets_and_layers_never_join() {
        let mut other = seg((5.0, 0.0), (5.0, 0.08));
        other.net = "M".into();
        let s = [
            seg((0.0, 0.0), (5.0, 0.0)),
            other,
            seg((5.0, 0.08), (9.0, 0.08)),
        ];
        assert!(kinds(&s).is_empty());
    }

    #[test]
    fn arguments_are_validated() {
        assert_eq!(parse_args(&json!({})).unwrap().0, "warn");
        assert_eq!(
            parse_args(&json!({ "geometry_check": "strict", "jog_length_mm": 0.3 }))
                .unwrap()
                .1
                .jog_length_mm,
            0.3
        );
        assert_eq!(
            parse_args(&json!({ "geometry_check": "loud" }))
                .unwrap_err()
                .0,
            "geometry_check"
        );
        assert_eq!(
            parse_args(&json!({ "min_bend_angle_deg": 200 }))
                .unwrap_err()
                .0,
            "min_bend_angle_deg"
        );
    }
}
