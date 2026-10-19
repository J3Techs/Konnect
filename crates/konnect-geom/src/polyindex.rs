//! Large polygons with an edge index.
//!
//! A refilled plane piece can carry tens of thousands of vertices, and its
//! bounding box covers most of the board, so the plain kernel
//! ([`crate::gap_cmp`]) would walk every edge for every nearby track.
//! [`IndexedPolygon`] keeps the polygon's edges in an R-tree and answers the
//! same exact questions — containment and the edge-to-edge gap compared with
//! a clearance — by visiting only the edges within reach:
//!
//! - a gap test visits the edges whose boxes meet the other shape's box
//!   inflated by the clearance; every other edge is provably farther away;
//! - containment casts the crossing-number ray through the index, so it
//!   visits only the edges whose boxes meet the ray.
//!
//! Results are identical to the unindexed kernel for every input.

use crate::bbox::BBox;
use crate::index::{IndexEntry, LayerIndex};
use crate::measure::{closest, Closest};
use crate::point::Point;
use crate::predicates::{
    core_cmp, dbl, orient, perturbed_in_edges, point_seg_cmp, region_within, seg_seg_cmp,
    shape_within, undbl, with_core, within_box, zero_cmp, Containment, Core, Perturbed, RegionView,
    P2,
};
use crate::shape::{Polygon, Segment, Shape};
use std::cmp::Ordering;

/// A polygon with holes plus an R-tree over its edges.
#[derive(Debug, Clone)]
pub struct IndexedPolygon {
    poly: Polygon,
    /// Every ring edge, outline first, as (start, end).
    edges: Vec<(Point, Point)>,
    tree: LayerIndex,
    bbox: BBox,
}

impl IndexedPolygon {
    pub fn new(poly: Polygon) -> Self {
        let mut edges = Vec::new();
        for ring in poly.rings() {
            let n = ring.len();
            edges.extend((0..n).map(|i| (ring[i], ring[(i + 1) % n])));
        }
        let entries = edges
            .iter()
            .enumerate()
            .map(|(i, &(a, b))| IndexEntry::new(i as u32, BBox::new(a, b)))
            .collect();
        let bbox = poly.bbox();
        IndexedPolygon {
            poly,
            edges,
            tree: LayerIndex::bulk_load(entries),
            bbox,
        }
    }

    pub fn polygon(&self) -> &Polygon {
        &self.poly
    }

    pub fn bbox(&self) -> BBox {
        self.bbox
    }

    pub fn edge_count(&self) -> usize {
        self.edges.len()
    }

    /// Where `p` lies relative to the region; the same answer as
    /// [`crate::point_in_polygon`].
    pub fn containment(&self, p: Point) -> Containment {
        if !self.bbox.contains(p) {
            return Containment::Outside;
        }
        let q = dbl(p);
        let ray = BBox::new(p, Point::new(self.bbox.max.x, p.y));
        let mut inside = false;
        for e in self.tree.query_entries(&ray) {
            let (a, b) = self.edges[e.id as usize];
            let (a, b) = (dbl(a), dbl(b));
            let o = orient(a, b, q);
            if o == 0 && within_box(a, b, q) {
                return Containment::Boundary;
            }
            if (a.y > q.y) != (b.y > q.y) {
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

    /// Compare the edge-to-edge gap between the region and `shape` with
    /// `clearance` (nm); the same answer as
    /// `gap_cmp(&Shape::Polygon(poly), shape, clearance)`.
    pub fn gap_cmp(&self, shape: &Shape, clearance: i64) -> Ordering {
        let t = 2 * i128::from(clearance) + i128::from(shape.sweep2());
        if t < 0 {
            return Ordering::Greater;
        }
        let reach = shape.bbox().inflate(clamp_coord(clearance));
        if !reach.intersects(&self.bbox) {
            return Ordering::Greater;
        }
        with_core(shape, |core| {
            let probe = match core {
                Core::Point(p) => undbl(*p),
                Core::Seg(a, _) => undbl(*a),
                Core::Region(r) => r[0][0],
            };
            if self.containment(probe) != Containment::Outside {
                return zero_cmp(t);
            }
            // Outside the region and no edge within reach: farther than
            // `clearance` (a region inside the shape would put its edges in
            // reach too).
            let mut best = Ordering::Greater;
            for e in self.tree.query_entries(&reach) {
                let (a, b) = self.edges[e.id as usize];
                let o = core_cmp(core, &Core::Seg(dbl(a), dbl(b)), t);
                if o == Ordering::Less {
                    return o;
                }
                best = best.min(o);
            }
            best
        })
    }

    /// The closest points between the region and `shape` when their gap is
    /// at most `reach` (nm), visiting only the edges within reach; `None`
    /// when they are farther apart. Approximate, like [`crate::measure`].
    pub fn closest_within(&self, shape: &Shape, reach: i64) -> Option<Closest> {
        let probe = match shape {
            Shape::Segment(s) => s.a,
            Shape::Circle(c) => c.center,
            Shape::ConvexPoly(p) => p.points()[0],
            Shape::Polygon(p) => p.outline()[0],
        };
        if self.containment(probe) != Containment::Outside {
            return Some(Closest {
                gap_nm: 0.0,
                on_a: probe,
                on_b: probe,
            });
        }
        let q = shape.bbox().inflate(clamp_coord(reach));
        let reach = reach.max(0) as f64;
        self.tree
            .query_entries(&q)
            .map(|e| {
                let (a, b) = self.edges[e.id as usize];
                let c = closest(&Shape::Segment(Segment::new(a, b, 0)), shape);
                (c.gap_nm, e.id, c)
            })
            .filter(|(g, _, _)| *g <= reach)
            .min_by(|x, y| x.0.total_cmp(&y.0).then(x.1.cmp(&y.1)))
            .map(|(_, _, c)| c)
    }

    /// Whether `shape` lies entirely inside the region; the same answer as
    /// [`crate::shape_within_polygon`], visiting only the edges near the
    /// shape.
    pub fn contains(&self, shape: &Shape) -> bool {
        shape_within(shape, self)
    }

    /// Whether `other`'s region lies entirely inside this one (touching the
    /// boundary from inside counts), using both edge indexes; the same
    /// answer as [`crate::shape_within_polygon`] on the two polygons.
    pub fn contains_polygon(&self, other: &IndexedPolygon) -> bool {
        region_within(other, self)
    }

    /// `true` when the region and `shape` touch or overlap; the same answer
    /// as `gap_cmp(…, 0) != Greater`, stopping at the first contact.
    pub fn touches(&self, shape: &Shape) -> bool {
        let t = i128::from(shape.sweep2());
        let reach = shape.bbox();
        if !reach.intersects(&self.bbox) {
            return false;
        }
        with_core(shape, |core| {
            let probe = match core {
                Core::Point(p) => undbl(*p),
                Core::Seg(a, _) => undbl(*a),
                Core::Region(r) => r[0][0],
            };
            self.containment(probe) != Containment::Outside
                || self.tree.query_entries(&reach).any(|e| {
                    let (a, b) = self.edges[e.id as usize];
                    core_cmp(core, &Core::Seg(dbl(a), dbl(b)), t) != Ordering::Greater
                })
        })
    }

    /// Compare the gap between two indexed regions with `clearance` (nm);
    /// the same answer as [`crate::gap_cmp`] on the two polygons.
    pub fn gap_cmp_polygon(&self, other: &IndexedPolygon, clearance: i64) -> Ordering {
        let t = 2 * i128::from(clearance);
        if t < 0 {
            return Ordering::Greater;
        }
        let c = clamp_coord(clearance);
        if !self.bbox.inflate(c).intersects(&other.bbox) {
            return Ordering::Greater;
        }
        if other.containment(self.poly.outline()[0]) != Containment::Outside
            || self.containment(other.poly.outline()[0]) != Containment::Outside
        {
            return zero_cmp(t);
        }
        // Walk the polygon with fewer edges; look the other one's up.
        let (small, large) = if self.edges.len() <= other.edges.len() {
            (self, other)
        } else {
            (other, self)
        };
        let window = large.bbox.inflate(c);
        let mut best = Ordering::Greater;
        for &(a, b) in &small.edges {
            let eb = BBox::new(a, b);
            if !eb.intersects(&window) {
                continue;
            }
            let (pa, pb) = (dbl(a), dbl(b));
            for e in large.tree.query_entries(&eb.inflate(c)) {
                let (c0, c1) = large.edges[e.id as usize];
                let o = seg_seg_cmp(pa, pb, dbl(c0), dbl(c1), t);
                if o == Ordering::Less {
                    return o;
                }
                best = best.min(o);
            }
        }
        best
    }

    /// Compare the distance from `p` to the region (zero inside) with
    /// `distance` (nm).
    pub fn point_distance_cmp(&self, p: Point, distance: i64) -> Ordering {
        let t = 2 * i128::from(distance);
        if t < 0 {
            return Ordering::Greater;
        }
        if self.containment(p) != Containment::Outside {
            return zero_cmp(t);
        }
        let q = dbl(p);
        let mut best = Ordering::Greater;
        for e in self
            .tree
            .query_entries(&BBox::point(p).inflate(clamp_coord(distance)))
        {
            let (a, b) = self.edges[e.id as usize];
            let o = point_seg_cmp(q, dbl(a), dbl(b), t);
            if o == Ordering::Less {
                return o;
            }
            best = best.min(o);
        }
        best
    }
}

impl RegionView for IndexedPolygon {
    fn region_bbox(&self) -> BBox {
        self.bbox
    }

    fn contain(&self, p: P2) -> Containment {
        self.containment(undbl(p))
    }

    fn contain_perturbed(&self, x: Perturbed) -> Containment {
        let s = undbl(x.s);
        if !self.bbox.contains(s) {
            // At least 1 nm outside the box: so is every point near it.
            return Containment::Outside;
        }
        // Every edge the ray from a point infinitesimally near `s` can meet,
        // or that can pass through `s`, has a box meeting this ray from `s`.
        let ray = BBox::new(s, Point::new(self.bbox.max.x, s.y));
        perturbed_in_edges(
            x,
            self.tree.query_entries(&ray).map(|e| {
                let (a, b) = self.edges[e.id as usize];
                (dbl(a), dbl(b))
            }),
        )
    }

    fn any_edge_near(&self, q: BBox, f: &mut dyn FnMut(P2, P2) -> bool) -> bool {
        self.tree.query_entries(&q).any(|e| {
            let (a, b) = self.edges[e.id as usize];
            f(dbl(a), dbl(b))
        })
    }

    fn any_edge(&self, f: &mut dyn FnMut(P2, P2) -> bool) -> bool {
        self.edges.iter().any(|&(a, b)| f(dbl(a), dbl(b)))
    }
}

/// Inflation amounts beyond the coordinate range saturate (`BBox::inflate`
/// saturates too), so a huge clearance simply covers everything.
fn clamp_coord(v: i64) -> i32 {
    v.clamp(0, i64::from(i32::MAX)) as i32
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::predicates::{gap_cmp, point_in_polygon};
    use crate::shape::{Circle, ConvexPoly};

    fn p(x: i32, y: i32) -> Point {
        Point::new(x, y)
    }

    /// A comb: a 1000-wide bar with teeth hanging down, plus a hole.
    fn comb() -> Polygon {
        let mut outline = vec![p(0, 0), p(1000, 0), p(1000, 100)];
        let mut x = 1000;
        while x > 0 {
            outline.extend([p(x, 300), p(x - 50, 300), p(x - 50, 100), p(x - 100, 100)]);
            x -= 100;
        }
        Polygon::new(
            outline,
            vec![vec![p(400, 20), p(600, 20), p(600, 80), p(400, 80)]],
        )
        .unwrap()
    }

    #[test]
    fn containment_matches_the_kernel() {
        let poly = comb();
        let idx = IndexedPolygon::new(poly.clone());
        for y in (-20..=320).step_by(10) {
            for x in (-20..=1020).step_by(10) {
                let q = p(x, y);
                assert_eq!(idx.containment(q), point_in_polygon(q, &poly), "{q:?}");
            }
        }
    }

    #[test]
    fn gaps_match_the_kernel() {
        let poly = comb();
        let idx = IndexedPolygon::new(poly.clone());
        let plain = Shape::Polygon(poly);
        let mut shapes = Vec::new();
        for y in (-60..=360).step_by(37) {
            for x in (-60..=1060).step_by(41) {
                shapes.push(Shape::from(Circle::new(p(x, y), 7)));
                shapes.push(Shape::from(Segment::new(p(x, y), p(x + 33, y + 19), 12)));
                shapes.push(Shape::from(
                    ConvexPoly::axis_rect(BBox::new(p(x, y), p(x + 20, y + 15)), 3).unwrap(),
                ));
            }
        }
        for s in &shapes {
            for c in [0, 5, 25, 60] {
                assert_eq!(idx.gap_cmp(s, c), gap_cmp(&plain, s, c), "{s:?} {c}");
            }
        }
    }

    #[test]
    fn polygon_pairs_match_the_kernel() {
        let a = comb();
        let ia = IndexedPolygon::new(a.clone());
        for (dx, dy) in [(0, 330), (25, 310), (0, 301), (420, 30), (2000, 0)] {
            let b = Polygon::new(
                vec![
                    p(dx, dy),
                    p(dx + 150, dy),
                    p(dx + 150, dy + 40),
                    p(dx, dy + 40),
                ],
                vec![],
            )
            .unwrap();
            let ib = IndexedPolygon::new(b.clone());
            for c in [0, 1, 10, 30, 31] {
                let want = gap_cmp(&Shape::Polygon(a.clone()), &Shape::Polygon(b.clone()), c);
                assert_eq!(ia.gap_cmp_polygon(&ib, c), want, "{dx},{dy} {c}");
                assert_eq!(ib.gap_cmp_polygon(&ia, c), want, "{dx},{dy} {c}");
            }
        }
    }

    #[test]
    fn touches_matches_the_kernel() {
        let poly = comb();
        let idx = IndexedPolygon::new(poly.clone());
        let plain = Shape::Polygon(poly);
        for y in (-40..=340).step_by(13) {
            for x in (-40..=1040).step_by(17) {
                for s in [
                    Shape::from(Circle::new(p(x, y), 6)),
                    Shape::from(Segment::new(p(x, y), p(x + 21, y - 9), 0)),
                    Shape::from(
                        ConvexPoly::axis_rect(BBox::new(p(x, y), p(x + 9, y + 9)), 0).unwrap(),
                    ),
                ] {
                    let want = gap_cmp(&plain, &s, 0) != Greater;
                    assert_eq!(idx.touches(&s), want, "{s:?}");
                }
            }
        }
    }

    #[test]
    fn closest_within_reach() {
        let idx = IndexedPolygon::new(comb());
        // 30 below a tooth's bottom edge (y = 300), between x 950 and 1000.
        let dot = Shape::from(Circle::new(p(975, 335), 5));
        let c = idx.closest_within(&dot, 40).unwrap();
        assert!((c.gap_nm - 30.0).abs() < 1e-6);
        assert_eq!(c.on_a, p(975, 300));
        assert!(idx.closest_within(&dot, 29).is_none());
        let inside = Shape::from(Circle::new(p(10, 10), 5));
        assert_eq!(idx.closest_within(&inside, 0).unwrap().gap_nm, 0.0);
    }

    #[test]
    fn contains_matches_the_kernel() {
        let poly = comb();
        let idx = IndexedPolygon::new(poly.clone());
        for y in (-40..=340).step_by(11) {
            for x in (-40..=1040).step_by(19) {
                for s in [
                    Shape::from(Circle::new(p(x, y), 6)),
                    Shape::from(Circle::new(p(x, y), 0)),
                    Shape::from(Segment::new(p(x, y), p(x + 31, y - 7), 4)),
                    Shape::from(Segment::new(p(x, y), p(x + 31, y - 7), 0)),
                    Shape::from(
                        ConvexPoly::axis_rect(BBox::new(p(x, y), p(x + 23, y + 17)), 0).unwrap(),
                    ),
                    Shape::from(
                        ConvexPoly::axis_rect(BBox::new(p(x, y), p(x + 23, y + 17)), 3).unwrap(),
                    ),
                ] {
                    let want = crate::predicates::shape_within_polygon(&s, &poly);
                    assert_eq!(idx.contains(&s), want, "{s:?}");
                }
            }
        }
    }

    #[test]
    fn polygon_containment() {
        let outer = IndexedPolygon::new(comb());
        let rect = |x0, y0, x1, y1| {
            IndexedPolygon::new(
                Polygon::new(vec![p(x0, y0), p(x1, y0), p(x1, y1), p(x0, y1)], vec![]).unwrap(),
            )
        };
        assert!(outer.contains_polygon(&rect(10, 10, 300, 90)));
        // Touching the outline from inside.
        assert!(outer.contains_polygon(&rect(0, 0, 300, 15)));
        // Over the hole, or around it.
        assert!(!outer.contains_polygon(&rect(10, 10, 500, 90)));
        assert!(!outer.contains_polygon(&rect(390, 10, 610, 90)));
        // Into a gap between teeth.
        assert!(!outer.contains_polygon(&rect(10, 10, 80, 200)));
        assert!(!outer.contains_polygon(&rect(10, 10, 2000, 90)));
        for (x, y) in [(10, 10), (300, 50)] {
            let small = rect(x, y, x + 20, y + 20);
            let want = crate::predicates::shape_within_polygon(
                &Shape::Polygon(small.polygon().clone()),
                outer.polygon(),
            );
            assert_eq!(outer.contains_polygon(&small), want);
        }
    }

    #[test]
    fn point_distance() {
        let idx = IndexedPolygon::new(comb());
        // In the hole, 30 from its long edges.
        assert_eq!(idx.point_distance_cmp(p(500, 50), 20), Greater);
        assert_eq!(idx.point_distance_cmp(p(500, 50), 31), Less);
        assert_eq!(idx.point_distance_cmp(p(500, 50), 30), Equal);
        assert_eq!(idx.point_distance_cmp(p(10, 10), 0), Equal);
    }

    use Ordering::*;
}
