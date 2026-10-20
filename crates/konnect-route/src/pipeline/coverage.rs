//! Reference-plane coverage of a refilled board.
//!
//! The job's reference check ([`super::ClassFallback::reference`]) runs
//! against the fill saved before routing and predicts the holes the final
//! refill will cut. Once KiCad has refilled and saved the board, the fill
//! itself is the evidence: [`reference_coverage`] reads it and reports every
//! track of the given nets, on the given layers, whose copper edge grown by
//! the margin does not lie over one filled piece of the plane.
//!
//! A track has to leave its own vias, and the refill cuts an antipad
//! around each of them on the plane. The stretch of a track within its own
//! net's via antipad (the via grown by the rules' clearance to the plane's
//! zones, plus the margin) is therefore not held to the plane.

use super::reference_pieces;
use crate::api::{ClassId, ItemKind, LayerId, LayerMask, NetId};
use crate::board::RouteBoard;
use crate::rules::{RuleSubject, RuleTable};
use konnect_geom::{Circle, Containment, Coord, IndexedPolygon, Point, Segment, Shape};
use std::collections::{BTreeMap, BTreeSet};

/// Spacing of the discs a track is checked with where it is not wholly
/// inside one piece (the capsule is their union; at 50 µm the missed
/// sliver at the copper edge is under a micrometre).
const SAMPLE_STEP_NM: i64 = 50_000;

/// A track that leaves the reference.
#[derive(Debug, Clone, PartialEq)]
pub struct OffReference {
    pub net: NetId,
    pub layer: LayerId,
    pub seg: Segment,
    /// Length of the track whose grown copper is not inside a piece, nm
    /// (at the sampling step).
    pub off_nm: i64,
    /// Whether some of that length has no plane copper under the track's
    /// centre line at all (a split or the edge of the fill), rather than
    /// only coming within the margin of an edge.
    pub bare: bool,
}

/// What [`reference_coverage`] checked and found.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ReferenceCoverage {
    /// Filled pieces of the plane on the board.
    pub pieces: usize,
    /// Of those, pieces KiCad marks as islands.
    pub island_pieces: usize,
    /// Tracks of the nets on the layers.
    pub tracks_checked: usize,
    /// Nets those tracks belong to.
    pub nets_checked: BTreeSet<NetId>,
    /// Of the tracks wholly inside one piece, those whose piece is an
    /// island.
    pub over_island: usize,
    /// Tracks that leave the reference.
    pub off: Vec<OffReference>,
}

/// The board's nets of `class` (every class when `None`) whose names match
/// one of `patterns` (`*`, `?`; every name when empty).
pub fn select_nets(
    board: &RouteBoard,
    class: Option<ClassId>,
    patterns: &[String],
) -> BTreeSet<NetId> {
    let pats: Vec<Vec<char>> = patterns.iter().map(|p| p.chars().collect()).collect();
    board
        .nets
        .iter()
        .filter(|n| class.is_none_or(|c| n.class == c))
        .filter(|n| {
            pats.is_empty() || {
                let chars: Vec<char> = n.name.chars().collect();
                pats.iter()
                    .any(|p| crate::rules::wildcard_match_chars(p, &chars))
            }
        })
        .map(|n| n.id)
        .collect()
}

/// Check the tracks of `nets` on `layers` against the filled copper of
/// `plane` as saved (see the module docs). Tracks of other nets and on
/// other layers are not looked at. With no filled piece on the plane every
/// checked track is off and bare.
pub fn reference_coverage(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    plane: LayerId,
    layers: LayerMask,
    nets: &BTreeSet<NetId>,
    margin: Coord,
) -> ReferenceCoverage {
    let pieces = reference_pieces(board, plane);
    let zone_nets: BTreeSet<NetId> = pieces.iter().map(|p| p.net).collect();
    let mut memo: BTreeMap<NetId, Coord> = BTreeMap::new();
    let mut antipad = |net: NetId| -> Coord {
        *memo.entry(net).or_insert_with(|| {
            zone_nets
                .iter()
                .filter(|&&z| z != net)
                .map(|&z| {
                    rules.clearance(
                        &RuleSubject::new(ItemKind::Via, Some(net)),
                        &RuleSubject::new(ItemKind::Zone, Some(z)),
                        plane,
                    )
                })
                .max()
                .unwrap_or(0)
        })
    };
    // Own-net vias through the plane: (net, centre, antipad radius).
    let mut own_vias: BTreeMap<NetId, Vec<(Point, i64)>> = BTreeMap::new();
    for v in &board.vias {
        let Some(net) = v.net.filter(|n| nets.contains(n)) else {
            continue;
        };
        if v.layers.contains(plane) {
            let r = i64::from(v.diameter).div_euclid(2) + i64::from(antipad(net));
            own_vias.entry(net).or_default().push((v.center, r));
        }
    }

    let mut out = ReferenceCoverage {
        pieces: pieces.len(),
        island_pieces: pieces.iter().filter(|p| p.island).count(),
        ..ReferenceCoverage::default()
    };
    for t in &board.tracks {
        let Some(net) = t.net.filter(|n| nets.contains(n)) else {
            continue;
        };
        if !layers.contains(t.layer) {
            continue;
        }
        out.tracks_checked += 1;
        out.nets_checked.insert(net);
        let half = i64::from(t.seg.width).div_euclid(2) + i64::from(margin);
        let grown = Shape::Segment(Segment::new(
            t.seg.a,
            t.seg.b,
            t.seg.width.saturating_add(margin.saturating_mul(2)),
        ));
        if let Some(p) = pieces.iter().find(|p| covers(&p.poly, &grown)) {
            if p.island {
                out.over_island += 1;
            }
            continue;
        }
        let vias = own_vias.get(&net).map(Vec::as_slice).unwrap_or(&[]);
        let (off_nm, bare) = sample_off(&t.seg, half, &pieces, vias);
        if off_nm > 0 {
            out.off.push(OffReference {
                net,
                layer: t.layer,
                seg: t.seg,
                off_nm,
                bare,
            });
        }
    }
    out
}

/// Walk `seg` in discs of radius `half`: the length whose disc is inside no
/// piece (outside every own-net antipad, each grown by `half`), and whether
/// any of it has no piece under its centre.
fn sample_off(
    seg: &Segment,
    half: i64,
    pieces: &[super::ReferencePiece],
    vias: &[(Point, i64)],
) -> (i64, bool) {
    let (ax, ay) = (i64::from(seg.a.x), i64::from(seg.a.y));
    let (dx, dy) = (i64::from(seg.b.x) - ax, i64::from(seg.b.y) - ay);
    let len = ((dx as f64).hypot(dy as f64)).round() as i64;
    let steps = (len / SAMPLE_STEP_NM).max(1);
    let step_len = len / steps;
    let radius = Coord::try_from(half).unwrap_or(Coord::MAX);
    let mut off = 0i64;
    let mut bare = false;
    for k in 0..=steps {
        let x = ax + dx * k / steps;
        let y = ay + dy * k / steps;
        let (Ok(px), Ok(py)) = (Coord::try_from(x), Coord::try_from(y)) else {
            continue;
        };
        let p = Point::new(px, py);
        let exempt = vias.iter().any(|&(c, r)| {
            let (ex, ey) = ((i64::from(c.x) - x) as f64, (i64::from(c.y) - y) as f64);
            ex.hypot(ey) <= (r + half) as f64
        });
        if exempt {
            continue;
        }
        let disc = Shape::Circle(Circle::new(p, radius));
        if !pieces.iter().any(|pc| covers(&pc.poly, &disc)) {
            off += step_len.max(1);
            if !pieces
                .iter()
                .any(|pc| pc.poly.containment(p) != Containment::Outside)
            {
                bare = true;
            }
        }
    }
    (off.min(len.max(1)), bare)
}

/// Grid spacing at which [`covers`] samples a shape it cannot prove inside
/// at once.
const COVER_STEP_NM: i64 = 50_000;

/// Whether `shape` lies over the copper of the fill piece `poly`.
///
/// A plain containment test is wrong for a saved fill: KiCad saves fills
/// fractured, each void joined to the outline by a zero-width slit, and a
/// shape that straddles a slit crosses two of the polygon's edges although
/// copper lies on both sides. So when the shape is not wholly inside, the
/// points of a [`COVER_STEP_NM`] grid that fall in the shape are tested one
/// by one: a point on a slit is on the boundary, one beside it inside, one
/// in a void outside. A void narrower than the grid spacing, or one poking
/// less than that into the shape's edge, can go unseen; refill voids
/// (antipads, clearances around other copper, thermal gaps) are far wider.
pub(super) fn covers(poly: &IndexedPolygon, shape: &Shape) -> bool {
    let b = shape.bbox();
    if !poly.bbox().contains_bbox(&b) {
        return false;
    }
    if poly.contains(shape) {
        return true;
    }
    let (x0, y0) = (i64::from(b.min.x), i64::from(b.min.y));
    let (x1, y1) = (i64::from(b.max.x), i64::from(b.max.y));
    let nx = ((x1 - x0) / COVER_STEP_NM).max(1);
    let ny = ((y1 - y0) / COVER_STEP_NM).max(1);
    let mut seen = false;
    for j in 0..=ny {
        for i in 0..=nx {
            let (Ok(x), Ok(y)) = (
                Coord::try_from(x0 + (x1 - x0) * i / nx),
                Coord::try_from(y0 + (y1 - y0) * j / ny),
            ) else {
                return false;
            };
            let q = Point::new(x, y);
            if !konnect_geom::touches(shape, &Shape::Circle(Circle::new(q, 0))) {
                continue;
            }
            seen = true;
            if poly.containment(q) == Containment::Outside {
                return false;
            }
        }
    }
    seen
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synthetic::{BoardBuilder, UniformRules};
    use konnect_geom::Polygon;

    fn pt(x: f64, y: f64) -> Point {
        Point::from_mm(x, y).unwrap()
    }

    /// A GND fill on In1.Cu over (1, 1)–(19, 11), saved fractured as KiCad
    /// saves it: a 4 × 4 mm void at (8, 4)–(12, 8) joined to the left edge
    /// by a slit along y = 6, and a 1 mm antipad around (15, 6) joined to
    /// the right edge the same way.
    fn fractured_fill() -> Polygon {
        let ring = [
            (1.0, 1.0),
            (19.0, 1.0),
            (19.0, 6.0),
            (15.5, 6.0),
            (15.5, 5.5),
            (14.5, 5.5),
            (14.5, 6.5),
            (15.5, 6.5),
            (15.5, 6.0),
            (19.0, 6.0),
            (19.0, 11.0),
            (1.0, 11.0),
            (1.0, 6.0),
            (8.0, 6.0),
            (8.0, 8.0),
            (12.0, 8.0),
            (12.0, 4.0),
            (8.0, 4.0),
            (8.0, 6.0),
            (1.0, 6.0),
        ];
        Polygon::new(ring.iter().map(|&(x, y)| pt(x, y)).collect(), Vec::new()).unwrap()
    }

    fn capsule(a: (f64, f64), b: (f64, f64), width_mm: f64) -> Shape {
        Shape::Segment(Segment::new(
            pt(a.0, a.1),
            pt(b.0, b.1),
            konnect_geom::mm_to_nm(width_mm).unwrap(),
        ))
    }

    #[test]
    fn a_slit_is_copper_and_a_void_is_not() {
        let poly = IndexedPolygon::new(fractured_fill());
        let across_slit = capsule((4.0, 3.0), (4.0, 9.0), 0.75);
        // The plain containment test sees the slit's two edges cross it.
        assert!(!poly.contains(&across_slit));
        assert!(covers(&poly, &across_slit));
        assert!(!covers(&poly, &capsule((10.0, 2.0), (10.0, 10.0), 0.15)));
        // Grazing the void's edge from outside is still over copper; a
        // little way in is not.
        assert!(covers(&poly, &capsule((7.5, 4.5), (7.5, 7.5), 1.0)));
        assert!(!covers(&poly, &capsule((7.8, 4.5), (7.8, 7.5), 1.0)));
        // Past the outline.
        assert!(!covers(&poly, &capsule((2.0, 1.2), (6.0, 1.2), 0.75)));
    }

    #[test]
    fn coverage_lists_tracks_off_the_refilled_plane() {
        let mut b = BoardBuilder::new(20.0, 12.0, &["F.Cu", "In1.Cu", "B.Cu"]);
        b.zone("GND", "In1.Cu", (1.0, 1.0, 19.0, 11.0));
        b.track("A", "B.Cu", (4.0, 3.0), (4.0, 9.0), 0.15, false); // across a slit
        b.track("A", "B.Cu", (17.0, 3.0), (17.0, 9.0), 0.15, false); // across the other
        b.track("A", "B.Cu", (10.0, 2.0), (10.0, 10.0), 0.15, false); // over the void
        b.track("A", "B.Cu", (15.0, 6.0), (15.0, 9.0), 0.15, false); // from its own via
        b.track("A", "B.Cu", (2.0, 1.3), (6.0, 1.3), 0.15, false); // near the edge
        b.track("A", "F.Cu", (10.0, 2.0), (10.0, 10.0), 0.15, false); // not a checked layer
        b.track("B", "B.Cu", (11.0, 2.0), (11.0, 10.0), 0.15, false); // not a checked net
        b.via("A", (15.0, 6.0), 0.6, 0.3, false);
        let mut board = b.build();
        let in1 = board.layers.by_name("In1.Cu").unwrap();
        let bcu = board.layers.by_name("B.Cu").unwrap();
        let fill = board.zones.iter_mut().find(|z| z.layer == in1).unwrap();
        fill.region = fractured_fill();
        let a = board.net_by_name("A").unwrap();
        let nets = select_nets(&board, None, &["A".to_string()]);
        assert_eq!(nets, BTreeSet::from([a]));
        let rules = UniformRules::default();

        let cov = reference_coverage(&board, &rules, in1, LayerMask::single(bcu), &nets, 300_000);
        assert_eq!((cov.pieces, cov.island_pieces), (1, 0));
        assert_eq!(cov.tracks_checked, 5);
        let off: Vec<(Point, bool)> = cov.off.iter().map(|o| (o.seg.a, o.bare)).collect();
        assert_eq!(off, vec![(pt(10.0, 2.0), true), (pt(2.0, 1.3), false)]);
        // The void is 4 mm across; the margin adds its width on each side.
        let void = &cov.off[0];
        assert!(
            (4_500_000..=5_000_000).contains(&void.off_nm),
            "{}",
            void.off_nm
        );

        // With no margin only the track over the void is off.
        let cov = reference_coverage(&board, &rules, in1, LayerMask::single(bcu), &nets, 0);
        assert_eq!(cov.off.len(), 1);
        assert!(cov.off[0].bare);

        // A plane with no fill: every checked track is off and bare.
        let f_cu = board.layers.by_name("F.Cu").unwrap();
        let cov = reference_coverage(&board, &rules, f_cu, LayerMask::single(bcu), &nets, 0);
        assert_eq!((cov.pieces, cov.off.len()), (0, 5));
        assert!(cov.off.iter().all(|o| o.bare));
    }
}
