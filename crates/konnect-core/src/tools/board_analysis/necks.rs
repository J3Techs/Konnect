//! Narrow places in a copper region: chords across the copper shorter than
//! a threshold, clustered to their local minima.
//!
//! A chord joins the closest points of two boundary edges. It counts when
//! it crosses copper (its midpoint is inside the region) and the two points
//! are far apart along the boundary — more than 1.6 times the chord, so a
//! convex corner (at most √2) or a gently curved edge never reads as narrow,
//! while a neck between two lobes, a web between two antipads, or a sliver
//! does. Edges on different rings (outline and hole, two holes) always
//! qualify. This is the width a current must squeeze through, measured
//! exactly on the polygon, not on a raster.

use super::geom::{dist, inside_rings, mid, segment_segment, Grid, P};

/// One narrow place: the chord's width and ends.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Neck {
    pub width: f64,
    pub a: P,
    pub b: P,
    /// For a chord between two points of one ring: the shorter way round
    /// the boundary between them, mm (the perimeter of the smaller side).
    /// `None` between two rings (a web between a hole and the outline).
    pub lobe: Option<f64>,
}

impl Neck {
    pub(crate) fn at(&self) -> P {
        mid(self.a, self.b)
    }

    /// A sliver hangs off the copper (its smaller side is short: a spike or
    /// a thin flap); a neck joins two substantial parts.
    pub(crate) fn kind(&self) -> &'static str {
        match self.lobe {
            Some(w) if w < LOBE_MM => "sliver",
            _ => "neck",
        }
    }
}

/// A smaller side shorter than this round its boundary makes a sliver.
const LOBE_MM: f64 = 1.0;

/// Walk distance along the boundary over chord length below which a chord
/// is local (a corner or curve), not a neck.
const WALK_RATIO: f64 = 1.6;

/// Chords narrower than this (0.5 µm) are numerical, not copper.
const MIN_WIDTH: f64 = 5e-4;

struct Edge {
    a: P,
    b: P,
    ring: usize,
    /// Arc length at `a` along its ring.
    s: f64,
}

/// Every narrow place of the region `rings` (even-odd) under `threshold`
/// mm, one per cluster (the narrowest), clusters at least `spacing` mm apart,
/// narrowest first.
pub(crate) fn find_necks(rings: &[Vec<P>], threshold: f64, spacing: f64) -> Vec<Neck> {
    find_necks_in(rings, threshold, spacing, None)
}

/// [`find_necks`] limited to edges meeting `window` (grown by the
/// threshold): the part of a large plane a current actually crosses.
pub(crate) fn find_necks_in(
    rings: &[Vec<P>],
    threshold: f64,
    spacing: f64,
    window: Option<(P, P)>,
) -> Vec<Neck> {
    let keep = |a: P, b: P| match window {
        None => true,
        Some((lo, hi)) => {
            a[0].max(b[0]) >= lo[0] - threshold
                && a[0].min(b[0]) <= hi[0] + threshold
                && a[1].max(b[1]) >= lo[1] - threshold
                && a[1].min(b[1]) <= hi[1] + threshold
        }
    };
    let mut edges = Vec::new();
    let mut perimeter = Vec::with_capacity(rings.len());
    for (ri, ring) in rings.iter().enumerate() {
        let n = ring.len();
        let mut s = 0.0;
        for k in 0..n {
            let (a, b) = (ring[k], ring[(k + 1) % n]);
            let l = dist(a, b);
            if l > 0.0 && keep(a, b) {
                edges.push(Edge { a, b, ring: ri, s });
            }
            s += l;
        }
        perimeter.push(s);
    }
    if edges.is_empty() || threshold <= 0.0 {
        return Vec::new();
    }
    let mut grid = Grid::new(threshold.max(0.05));
    for (i, e) in edges.iter().enumerate() {
        let lo = [e.a[0].min(e.b[0]), e.a[1].min(e.b[1])];
        let hi = [e.a[0].max(e.b[0]), e.a[1].max(e.b[1])];
        grid.insert(lo, hi, i as u32);
    }
    let mut found: Vec<Neck> = Vec::new();
    let mut seen = Vec::new();
    for (i, e) in edges.iter().enumerate() {
        let lo = [
            e.a[0].min(e.b[0]) - threshold,
            e.a[1].min(e.b[1]) - threshold,
        ];
        let hi = [
            e.a[0].max(e.b[0]) + threshold,
            e.a[1].max(e.b[1]) + threshold,
        ];
        seen.clear();
        grid.query(lo, hi, |j| {
            if (j as usize) > i {
                seen.push(j);
            }
        });
        seen.sort_unstable();
        seen.dedup();
        for &j in &seen {
            let f = &edges[j as usize];
            let (d, mut p, mut q, mut t, mut u) = segment_segment(e.a, e.b, f.a, f.b);
            // Parallel edges are equally close all along their overlap: take
            // the chord at its middle, so a strip reads as one place.
            if let Some((pm, qm, tm, um)) = parallel_middle(e.a, e.b, f.a, f.b) {
                if (super::geom::dist(pm, qm) - d).abs() < 1e-9 {
                    (p, q, t, u) = (pm, qm, tm, um);
                }
            }
            if !(d >= MIN_WIDTH && d < threshold) {
                continue;
            }
            let mut lobe = None;
            if e.ring == f.ring {
                let sp = e.s + t * dist(e.a, e.b);
                let sq = f.s + u * dist(f.a, f.b);
                let along = (sp - sq).abs();
                let walk = along.min(perimeter[e.ring] - along);
                if walk <= WALK_RATIO * d {
                    continue;
                }
                lobe = Some(walk);
            }
            if !inside_rings(mid(p, q), rings) {
                continue;
            }
            found.push(Neck {
                width: d,
                a: p,
                b: q,
                lobe,
            });
        }
    }
    // Chain chords whose midpoints lie within `spacing` of each other into
    // one place, and keep each place's narrowest chord.
    let mut parent: Vec<u32> = (0..found.len() as u32).collect();
    fn root(parent: &mut [u32], mut x: u32) -> u32 {
        while parent[x as usize] != x {
            parent[x as usize] = parent[parent[x as usize] as usize];
            x = parent[x as usize];
        }
        x
    }
    let mut grid = Grid::new(spacing.max(0.01));
    for (i, n) in found.iter().enumerate() {
        let c = n.at();
        grid.insert(c, c, i as u32);
    }
    for (i, n) in found.iter().enumerate() {
        let c = n.at();
        let mut near = Vec::new();
        grid.query(
            [c[0] - spacing, c[1] - spacing],
            [c[0] + spacing, c[1] + spacing],
            |k: u32| {
                if (k as usize) != i && dist(found[k as usize].at(), c) <= spacing {
                    near.push(k);
                }
            },
        );
        for k in near {
            let (a, b) = (root(&mut parent, i as u32), root(&mut parent, k));
            if a != b {
                parent[a.max(b) as usize] = a.min(b);
            }
        }
    }
    let mut best: std::collections::BTreeMap<u32, Neck> = std::collections::BTreeMap::new();
    for (i, &n) in found.iter().enumerate() {
        let r = root(&mut parent, i as u32);
        best.entry(r)
            .and_modify(|b| {
                if n.width < b.width {
                    *b = n;
                }
            })
            .or_insert(n);
    }
    let mut kept: Vec<Neck> = best.into_values().collect();
    kept.sort_by(|x, y| x.width.total_cmp(&y.width).then(x.a[0].total_cmp(&y.a[0])));
    kept
}

/// For near-parallel segments that overlap along their direction, the
/// closest pair at the middle of the overlap: (on first, on second, t, u).
fn parallel_middle(a: P, b: P, c: P, d: P) -> Option<(P, P, f64, f64)> {
    let r = [b[0] - a[0], b[1] - a[1]];
    let s = [d[0] - c[0], d[1] - c[1]];
    let lr = r[0].hypot(r[1]);
    let ls = s[0].hypot(s[1]);
    if lr == 0.0 || ls == 0.0 {
        return None;
    }
    let cross = (r[0] * s[1] - r[1] * s[0]) / (lr * ls);
    if cross.abs() > 1e-6 {
        return None;
    }
    // Project c and d onto a–b.
    let proj = |p: P| ((p[0] - a[0]) * r[0] + (p[1] - a[1]) * r[1]) / (lr * lr);
    let (tc, td) = (proj(c), proj(d));
    let lo = tc.min(td).max(0.0);
    let hi = tc.max(td).min(1.0);
    if hi <= lo {
        return None;
    }
    let t = (lo + hi) / 2.0;
    let p = [a[0] + t * r[0], a[1] + t * r[1]];
    let (q, u) = super::geom::closest_on_segment(p, c, d);
    Some((p, q, t, u))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Vec<P> {
        vec![[x0, y0], [x1, y0], [x1, y1], [x0, y1]]
    }

    #[test]
    fn a_dumbbell_reports_its_neck_and_nothing_at_the_corners() {
        // Two 4 x 4 squares joined by a 2 mm long, 0.084 mm wide neck.
        let ring = vec![
            [0.0, 0.0],
            [4.0, 0.0],
            [4.0, 1.958],
            [6.0, 1.958],
            [6.0, 0.0],
            [10.0, 0.0],
            [10.0, 4.0],
            [6.0, 4.0],
            [6.0, 2.042],
            [4.0, 2.042],
            [4.0, 4.0],
            [0.0, 4.0],
        ];
        let necks = find_necks(&[ring], 0.2, 1.0);
        assert_eq!(necks.len(), 1, "{necks:?}");
        assert!((necks[0].width - 0.084).abs() < 1e-9);
        assert!((necks[0].at()[1] - 2.0).abs() < 1e-9);
        assert!(necks[0].at()[0] >= 4.0 && necks[0].at()[0] <= 6.0);
    }

    #[test]
    fn a_web_between_a_hole_and_the_outline_is_a_neck_and_a_square_has_none() {
        assert!(find_necks(&[rect(0.0, 0.0, 5.0, 5.0)], 0.5, 1.0).is_empty());
        let rings = vec![rect(0.0, 0.0, 5.0, 5.0), rect(1.0, 0.1, 2.0, 1.0)];
        let necks = find_necks(&rings, 0.5, 1.0);
        assert_eq!(necks.len(), 1, "{necks:?}");
        assert!((necks[0].width - 0.1).abs() < 1e-9);
    }

    #[test]
    fn a_gap_between_two_lobes_is_not_copper() {
        // A U shape: the slot between the arms is 0.1 wide but empty.
        let ring = vec![
            [0.0, 0.0],
            [3.0, 0.0],
            [3.0, 5.0],
            [1.55, 5.0],
            [1.55, 1.0],
            [1.45, 1.0],
            [1.45, 5.0],
            [0.0, 5.0],
        ];
        let necks = find_necks(&[ring], 0.2, 0.5);
        assert!(necks.is_empty(), "{necks:?}");
    }
}
