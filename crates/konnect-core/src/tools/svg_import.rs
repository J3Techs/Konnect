//! SVG parsing and flattening for `import_svg_logo`.
//!
//! Splits into two layers so the geometry math is testable without any SVG
//! library involved:
//!   - `SvgSegment`/`flatten_subpaths`/`cubic_bezier_points` — pure path-flattening
//!     math (curves → straight-line polygon points). No dependency on `usvg`'s types.
//!   - `extract_polygons` — the thin `usvg` integration that walks a parsed SVG
//!     tree and adapts its path segments into `SvgSegment`s.
//!
//! KiCAD's polygon format (`PolyLine`/`gr_poly`) only supports straight points
//! and circular arcs, not cubic Bezier curves, so any curved SVG path must be
//! flattened into a point sequence before it can become filled artwork.
//!
//! A filled KiCad graphic polygon is also a single outline: KiCad 10 itself
//! fractures a polygon with a hole into one outline joined to the hole by a
//! zero-width slit before it writes `gr_poly`. `resolve_fill` therefore works
//! out which subpaths of an SVG path are outlines and which are holes under
//! the path's fill rule, and `bridge_holes` joins each hole to its outline the
//! same way.

/// A 2D point in whatever coordinate space the caller is working in (SVG user
/// units until scaled, millimeters after).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point2 {
    pub x: f64,
    pub y: f64,
}

impl Point2 {
    pub fn new(x: f64, y: f64) -> Self {
        Point2 { x, y }
    }
}

/// One command from an SVG path's `d` attribute, decoupled from any specific
/// SVG parsing library's types so the flattening logic below can be unit
/// tested with hand-built segment lists.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SvgSegment {
    MoveTo(Point2),
    LineTo(Point2),
    QuadTo(Point2, Point2),
    CubicTo(Point2, Point2, Point2),
    Close,
}

/// Number of straight-line segments used to approximate one Bezier curve.
/// Fixed (not adaptive) subdivision — plenty for silkscreen/copper artwork,
/// where visual fidelity at fabrication scale matters more than curve-fitting
/// precision.
const BEZIER_SEGMENTS: usize = 16;

/// Sample a cubic Bezier curve into `BEZIER_SEGMENTS` points (excluding the
/// start point `p0`, which the caller already has as the current position).
pub fn cubic_bezier_points(p0: Point2, p1: Point2, p2: Point2, p3: Point2) -> Vec<Point2> {
    (1..=BEZIER_SEGMENTS)
        .map(|i| {
            let t = i as f64 / BEZIER_SEGMENTS as f64;
            let mt = 1.0 - t;
            let x = mt * mt * mt * p0.x
                + 3.0 * mt * mt * t * p1.x
                + 3.0 * mt * t * t * p2.x
                + t * t * t * p3.x;
            let y = mt * mt * mt * p0.y
                + 3.0 * mt * mt * t * p1.y
                + 3.0 * mt * t * t * p2.y
                + t * t * t * p3.y;
            Point2::new(x, y)
        })
        .collect()
}

/// Elevate a quadratic Bezier (single control point) to the two control
/// points of an equivalent cubic Bezier, so quadratic curves can reuse
/// `cubic_bezier_points`. Standard exact conversion: cubic control points
/// sit 2/3 of the way from each endpoint toward the quadratic control point.
pub fn quad_to_cubic(p0: Point2, control: Point2, p2: Point2) -> (Point2, Point2) {
    let c1 = Point2::new(
        p0.x + 2.0 / 3.0 * (control.x - p0.x),
        p0.y + 2.0 / 3.0 * (control.y - p0.y),
    );
    let c2 = Point2::new(
        p2.x + 2.0 / 3.0 * (control.x - p2.x),
        p2.y + 2.0 / 3.0 * (control.y - p2.y),
    );
    (c1, c2)
}

/// Flatten a sequence of SVG path segments into one or more closed polygon
/// outlines (point lists). A `MoveTo` starts a new subpath; a `Close` (or the
/// next `MoveTo`/end of input) ends the current one. Subpaths with fewer than
/// 2 points (degenerate — e.g. a lone `MoveTo`) are dropped.
pub fn flatten_subpaths(segments: impl IntoIterator<Item = SvgSegment>) -> Vec<Vec<Point2>> {
    let mut polygons = Vec::new();
    let mut current: Vec<Point2> = Vec::new();
    let mut last = Point2::new(0.0, 0.0);

    let close_current = |current: &mut Vec<Point2>, polygons: &mut Vec<Vec<Point2>>| {
        if current.len() > 1 {
            polygons.push(std::mem::take(current));
        } else {
            current.clear();
        }
    };

    for seg in segments {
        match seg {
            SvgSegment::MoveTo(p) => {
                close_current(&mut current, &mut polygons);
                current.push(p);
                last = p;
            }
            SvgSegment::LineTo(p) => {
                current.push(p);
                last = p;
            }
            SvgSegment::QuadTo(control, p) => {
                let (c1, c2) = quad_to_cubic(last, control, p);
                current.extend(cubic_bezier_points(last, c1, c2, p));
                last = p;
            }
            SvgSegment::CubicTo(c1, c2, p) => {
                current.extend(cubic_bezier_points(last, c1, c2, p));
                last = p;
            }
            SvgSegment::Close => {
                close_current(&mut current, &mut polygons);
            }
        }
    }
    close_current(&mut current, &mut polygons);
    polygons
}

/// Scale and translate flattened SVG-unit polygons into board millimeters.
/// Scale is uniform (aspect-ratio preserving), derived from `target_width_mm`
/// against the SVG's native width; `offset_x`/`offset_y` place the top-left
/// of the scaled artwork on the board.
///
/// `mirror` flips the artwork left-to-right about the vertical centre line of
/// its own placement box (`offset_x ..= offset_x + target_width_mm`), so a
/// mirrored logo occupies exactly the box an unmirrored one would. Board
/// graphics on a back layer are stored as seen from the front, so artwork that
/// should read correctly from the back has to be stored mirrored.
///
/// Coordinates are rounded to the nanometre, KiCad's internal resolution.
pub fn scale_and_place(
    polygons: &[Vec<Point2>],
    svg_width: f64,
    target_width_mm: f64,
    offset_x: f64,
    offset_y: f64,
    mirror: bool,
) -> Vec<Vec<(f64, f64)>> {
    let scale = if svg_width > 0.0 {
        target_width_mm / svg_width
    } else {
        1.0
    };
    let box_width = svg_width.max(0.0) * scale;
    let nm = |v: f64| (v * 1e6).round() / 1e6;
    polygons
        .iter()
        .map(|poly| {
            poly.iter()
                .map(|p| {
                    let local_x = if mirror {
                        box_width - p.x * scale
                    } else {
                        p.x * scale
                    };
                    (nm(local_x + offset_x), nm(p.y * scale + offset_y))
                })
                .collect()
        })
        .collect()
}

/// How an SVG path decides which of its enclosed regions are painted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FillRule {
    NonZero,
    EvenOdd,
}

/// The filled area of one SVG path as KiCad-ready outlines.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ResolvedFill {
    /// One closed outline per filled region; a region's holes are already
    /// joined into its outline by zero-width slits.
    pub polygons: Vec<Vec<Point2>>,
    /// Holes joined into the outlines above.
    pub hole_count: usize,
    /// True when two subpaths cross each other. Nesting cannot describe such
    /// a path, so every subpath is returned as its own filled outline and no
    /// hole is cut.
    pub subpaths_cross: bool,
}

fn signed_area(ring: &[Point2]) -> f64 {
    let mut sum = 0.0;
    for (i, a) in ring.iter().enumerate() {
        let b = ring[(i + 1) % ring.len()];
        sum += a.x * b.y - b.x * a.y;
    }
    sum / 2.0
}

fn bounds(ring: &[Point2]) -> (Point2, Point2) {
    let mut lo = Point2::new(f64::INFINITY, f64::INFINITY);
    let mut hi = Point2::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
    for p in ring {
        lo = Point2::new(lo.x.min(p.x), lo.y.min(p.y));
        hi = Point2::new(hi.x.max(p.x), hi.y.max(p.y));
    }
    (lo, hi)
}

fn point_in_ring(p: Point2, ring: &[Point2]) -> bool {
    let mut inside = false;
    for (i, a) in ring.iter().enumerate() {
        let b = ring[(i + 1) % ring.len()];
        if (a.y > p.y) != (b.y > p.y) {
            let x = a.x + (p.y - a.y) / (b.y - a.y) * (b.x - a.x);
            if x > p.x {
                inside = !inside;
            }
        }
    }
    inside
}

/// Whether `inner` lies inside `outer`, for rings that do not cross. Three
/// sample points vote so one vertex resting on `outer`'s boundary cannot
/// decide it.
fn ring_inside(inner: &[Point2], outer: &[Point2]) -> bool {
    let mid = Point2::new(
        (inner[0].x + inner[1].x) / 2.0,
        (inner[0].y + inner[1].y) / 2.0,
    );
    let samples = [inner[0], mid, inner[inner.len() / 2]];
    samples.iter().filter(|p| point_in_ring(**p, outer)).count() >= 2
}

fn cross(o: Point2, a: Point2, b: Point2) -> f64 {
    (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)
}

/// Whether two rings properly cross (an edge of one passes through an edge of
/// the other). Rings that only touch do not count.
fn rings_cross(a: &[Point2], b: &[Point2]) -> bool {
    let (a_lo, a_hi) = bounds(a);
    let (b_lo, b_hi) = bounds(b);
    if a_lo.x > b_hi.x || b_lo.x > a_hi.x || a_lo.y > b_hi.y || b_lo.y > a_hi.y {
        return false;
    }
    for (i, &p1) in a.iter().enumerate() {
        let p2 = a[(i + 1) % a.len()];
        if p1.x.max(p2.x) < b_lo.x
            || p1.x.min(p2.x) > b_hi.x
            || p1.y.max(p2.y) < b_lo.y
            || p1.y.min(p2.y) > b_hi.y
        {
            continue;
        }
        for (j, &q1) in b.iter().enumerate() {
            let q2 = b[(j + 1) % b.len()];
            let d1 = cross(q1, q2, p1);
            let d2 = cross(q1, q2, p2);
            let d3 = cross(p1, p2, q1);
            let d4 = cross(p1, p2, q2);
            if d1 * d2 < 0.0 && d3 * d4 < 0.0 {
                return true;
            }
        }
    }
    false
}

/// Join `holes` into `outline` with zero-width slits, giving the single
/// outline KiCad stores for a filled polygon with holes.
///
/// Each hole is entered from its rightmost vertex along a horizontal slit to
/// the nearest edge on its right. Holes are taken rightmost first, so that
/// edge always belongs to the outline or to a hole already joined and the
/// slit crosses nothing. Returns `None` when a hole has no edge to its right,
/// which means it was not inside the outline after all.
pub fn bridge_holes(outline: &[Point2], holes: &[Vec<Point2>]) -> Option<Vec<Point2>> {
    let mut ring = outline.to_vec();
    if signed_area(&ring) < 0.0 {
        ring.reverse();
    }
    let (lo, hi) = bounds(&ring);
    let eps = (hi.x - lo.x).max(hi.y - lo.y) * 1e-12;
    let same = |a: Point2, b: Point2| (a.x - b.x).abs() <= eps && (a.y - b.y).abs() <= eps;
    let by_x_then_y = |a: Point2, b: Point2| {
        (a.x, b.y)
            .partial_cmp(&(b.x, a.y))
            .unwrap_or(std::cmp::Ordering::Equal)
    };

    // Wind every hole against the outline and start it at its rightmost
    // vertex (the upper one of a tie).
    let mut holes: Vec<Vec<Point2>> = holes
        .iter()
        .map(|hole| {
            let mut hole = hole.clone();
            if signed_area(&hole) > 0.0 {
                hole.reverse();
            }
            let start = (0..hole.len())
                .max_by(|&i, &j| by_x_then_y(hole[i], hole[j]))
                .unwrap_or(0);
            hole.rotate_left(start);
            hole
        })
        .collect();
    holes.sort_by(|a, b| by_x_then_y(b[0], a[0]));

    for hole in holes {
        let entry = hole[0];
        // Nearest crossing of the rightward ray from `entry`: (x, edge index).
        let mut nearest: Option<(f64, usize)> = None;
        for (i, &a) in ring.iter().enumerate() {
            let b = ring[(i + 1) % ring.len()];
            if (a.y <= entry.y && b.y > entry.y) || (b.y <= entry.y && a.y > entry.y) {
                let x = a.x + (entry.y - a.y) / (b.y - a.y) * (b.x - a.x);
                if x >= entry.x - eps && nearest.is_none_or(|(best, _)| x < best) {
                    nearest = Some((x, i));
                }
            }
        }
        let (x, edge) = nearest?;
        let landing = Point2::new(x, entry.y);
        let next = (edge + 1) % ring.len();
        // Land on an existing vertex when the ray hits one; otherwise split
        // the edge at the landing point.
        let (after, extra) = if same(landing, ring[edge]) {
            (edge, None)
        } else if same(landing, ring[next]) {
            (next, None)
        } else {
            (edge, Some(landing))
        };
        let back = extra.unwrap_or(ring[after]);
        let mut joined = Vec::with_capacity(ring.len() + hole.len() + 3);
        joined.extend_from_slice(&ring[..=after]);
        joined.extend(extra);
        joined.extend_from_slice(&hole);
        joined.push(entry);
        joined.push(back);
        joined.extend_from_slice(&ring[after + 1..]);
        joined.dedup_by(|b, a| same(*a, *b));
        ring = joined;
    }
    Some(ring)
}

/// Work out the filled area of one SVG path from its flattened subpaths.
///
/// Subpaths that do not cross form a nesting tree, and the fill rule decides
/// from that tree alone whether the area just inside each subpath is painted:
/// under `EvenOdd` when an odd number of subpaths enclose it, under `NonZero`
/// when the directions of the enclosing subpaths do not cancel. A subpath
/// with paint inside and none outside is an outline; one with paint outside
/// and none inside is a hole of the nearest outline around it; one with paint
/// on both sides (or neither) bounds nothing and is dropped.
pub fn resolve_fill(subpaths: Vec<Vec<Point2>>, rule: FillRule) -> ResolvedFill {
    // Drop a repeated closing point, then anything that encloses no area.
    let rings: Vec<Vec<Point2>> = subpaths
        .into_iter()
        .map(|mut ring| {
            while ring.len() > 1 && ring.first() == ring.last() {
                ring.pop();
            }
            ring
        })
        .filter(|ring| ring.len() >= 3 && signed_area(ring) != 0.0)
        .collect();
    if rings.len() < 2 {
        return ResolvedFill {
            polygons: rings,
            ..ResolvedFill::default()
        };
    }

    for (i, a) in rings.iter().enumerate() {
        if rings[i + 1..].iter().any(|b| rings_cross(a, b)) {
            return ResolvedFill {
                polygons: rings,
                hole_count: 0,
                subpaths_cross: true,
            };
        }
    }

    let areas: Vec<f64> = rings.iter().map(|ring| signed_area(ring)).collect();
    // The smallest ring that contains this one.
    let parents: Vec<Option<usize>> = (0..rings.len())
        .map(|i| {
            (0..rings.len())
                .filter(|&j| {
                    j != i && areas[j].abs() > areas[i].abs() && ring_inside(&rings[i], &rings[j])
                })
                .min_by(|&a, &b| {
                    areas[a]
                        .abs()
                        .partial_cmp(&areas[b].abs())
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
        })
        .collect();
    let painted_inside: Vec<bool> = (0..rings.len())
        .map(|i| {
            let (mut depth, mut winding) = (0i32, 0i32);
            let mut at = Some(i);
            while let Some(ring) = at {
                depth += 1;
                winding += if areas[ring] > 0.0 { 1 } else { -1 };
                at = parents[ring];
            }
            match rule {
                FillRule::EvenOdd => depth % 2 == 1,
                FillRule::NonZero => winding != 0,
            }
        })
        .collect();
    let painted_outside = |i: usize| parents[i].is_some_and(|parent| painted_inside[parent]);
    let is_outline = |i: usize| painted_inside[i] && !painted_outside(i);

    let mut holes_of: Vec<Vec<Vec<Point2>>> = vec![Vec::new(); rings.len()];
    for i in 0..rings.len() {
        if painted_inside[i] || !painted_outside(i) {
            continue;
        }
        let mut owner = parents[i];
        while let Some(ring) = owner {
            if is_outline(ring) {
                holes_of[ring].push(rings[i].clone());
                break;
            }
            owner = parents[ring];
        }
    }

    let mut resolved = ResolvedFill::default();
    for i in (0..rings.len()).filter(|&i| is_outline(i)) {
        match bridge_holes(&rings[i], &holes_of[i]) {
            Some(outline) => {
                resolved.hole_count += holes_of[i].len();
                resolved.polygons.push(outline);
            }
            // Never expected for nested rings; keep the outline whole rather
            // than lose it.
            None => resolved.polygons.push(rings[i].clone()),
        }
    }
    resolved
}

/// A parsed SVG, reduced to its native size and the outlines of its filled
/// area.
pub struct SvgLogo {
    pub width: f64,
    pub height: f64,
    /// Closed outlines in SVG user units, holes already joined in.
    pub polygons: Vec<Vec<Point2>>,
    /// Holes cut into `polygons`.
    pub hole_count: usize,
    /// Paths whose subpaths cross each other: imported as one filled outline
    /// per subpath, with no hole cut.
    pub crossing_path_count: usize,
    /// `<text>` elements with content. None of them is imported.
    pub text_element_count: usize,
}

/// Parse SVG content and reduce every path node to the outlines of its filled
/// area under its own fill rule. Every `Path` node in the tree is treated as
/// fillable artwork (no stroke-only exclusion, no color/layer separation),
/// which suits a single-color silkscreen/copper logo import.
///
/// `<text>` is not imported. usvg can only turn text into outlines with a font
/// database, and the fonts installed differ per machine, so the same SVG would
/// import as different artwork (or silently fall back to another typeface).
/// The font database is left empty, as `konnect-render` leaves it, and the
/// text elements are counted so the caller can report them.
pub fn extract_polygons(svg_content: &str) -> anyhow::Result<SvgLogo> {
    let opt = usvg::Options::default();
    let tree = usvg::Tree::from_str(svg_content, &opt)
        .map_err(|e| anyhow::anyhow!("Failed to parse SVG: {e}"))?;

    let size = tree.size();
    let mut logo = SvgLogo {
        width: size.width() as f64,
        height: size.height() as f64,
        polygons: Vec::new(),
        hole_count: 0,
        crossing_path_count: 0,
        text_element_count: count_text_elements(svg_content),
    };
    collect_path_polygons(tree.root(), &mut logo);
    Ok(logo)
}

/// `<text>` elements that would draw something: those with non-blank content.
fn count_text_elements(svg_content: &str) -> usize {
    let options = usvg::roxmltree::ParsingOptions {
        allow_dtd: true,
        ..Default::default()
    };
    let Ok(document) = usvg::roxmltree::Document::parse_with_options(svg_content, options) else {
        return 0;
    };
    document
        .descendants()
        .filter(|node| node.is_element() && node.tag_name().name() == "text")
        .filter(|node| {
            node.descendants()
                .any(|child| child.is_text() && !child.text().unwrap_or("").trim().is_empty())
        })
        .count()
}

fn collect_path_polygons(group: &usvg::Group, logo: &mut SvgLogo) {
    for node in group.children() {
        match node {
            usvg::Node::Path(path) => {
                // Path data is in the path's own coordinate system; its
                // absolute transform (viewBox scaling and every enclosing
                // group's transform) maps it to the SVG's root coordinates,
                // the ones `tree.size()` measures.
                let Some(data) = path.data().clone().transform(path.abs_transform()) else {
                    continue;
                };
                let rule = match path.fill().map(|fill| fill.rule()) {
                    Some(usvg::FillRule::EvenOdd) => FillRule::EvenOdd,
                    Some(usvg::FillRule::NonZero) | None => FillRule::NonZero,
                };
                let subpaths = flatten_subpaths(data.segments().map(adapt_segment));
                let resolved = resolve_fill(subpaths, rule);
                logo.hole_count += resolved.hole_count;
                logo.crossing_path_count += usize::from(resolved.subpaths_cross);
                logo.polygons.extend(resolved.polygons);
            }
            usvg::Node::Group(inner) => collect_path_polygons(inner, logo),
            // Images are out of scope. A text node only appears when a font
            // database resolved it, and none is loaded; see `extract_polygons`.
            usvg::Node::Image(_) | usvg::Node::Text(_) => {}
        }
    }
}

fn adapt_segment(seg: usvg::tiny_skia_path::PathSegment) -> SvgSegment {
    use usvg::tiny_skia_path::PathSegment;
    match seg {
        PathSegment::MoveTo(p) => SvgSegment::MoveTo(Point2::new(p.x as f64, p.y as f64)),
        PathSegment::LineTo(p) => SvgSegment::LineTo(Point2::new(p.x as f64, p.y as f64)),
        PathSegment::QuadTo(c, p) => SvgSegment::QuadTo(
            Point2::new(c.x as f64, c.y as f64),
            Point2::new(p.x as f64, p.y as f64),
        ),
        PathSegment::CubicTo(c1, c2, p) => SvgSegment::CubicTo(
            Point2::new(c1.x as f64, c1.y as f64),
            Point2::new(c2.x as f64, c2.y as f64),
            Point2::new(p.x as f64, p.y as f64),
        ),
        PathSegment::Close => SvgSegment::Close,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cubic_bezier_points_starts_near_control_and_ends_at_p3() {
        let p0 = Point2::new(0.0, 0.0);
        let p1 = Point2::new(0.0, 10.0);
        let p2 = Point2::new(10.0, 10.0);
        let p3 = Point2::new(10.0, 0.0);
        let pts = cubic_bezier_points(p0, p1, p2, p3);
        assert_eq!(pts.len(), BEZIER_SEGMENTS);
        let last = pts.last().unwrap();
        assert!((last.x - p3.x).abs() < 1e-9);
        assert!((last.y - p3.y).abs() < 1e-9);
    }

    #[test]
    fn quad_to_cubic_produces_control_points_between_endpoints_and_control() {
        let p0 = Point2::new(0.0, 0.0);
        let control = Point2::new(5.0, 10.0);
        let p2 = Point2::new(10.0, 0.0);
        let (c1, c2) = quad_to_cubic(p0, control, p2);
        // c1 should be 2/3 of the way from p0 to control.
        assert!((c1.x - 10.0 / 3.0).abs() < 1e-9);
        assert!((c1.y - 20.0 / 3.0).abs() < 1e-9);
        // c2 should be 2/3 of the way from p2 to control.
        assert!((c2.x - 20.0 / 3.0).abs() < 1e-9);
        assert!((c2.y - 20.0 / 3.0).abs() < 1e-9);
    }

    #[test]
    fn flatten_subpaths_single_closed_triangle() {
        let segments = vec![
            SvgSegment::MoveTo(Point2::new(0.0, 0.0)),
            SvgSegment::LineTo(Point2::new(10.0, 0.0)),
            SvgSegment::LineTo(Point2::new(5.0, 10.0)),
            SvgSegment::Close,
        ];
        let polys = flatten_subpaths(segments);
        assert_eq!(polys.len(), 1);
        assert_eq!(polys[0].len(), 3);
    }

    #[test]
    fn flatten_subpaths_two_separate_subpaths() {
        let segments = vec![
            SvgSegment::MoveTo(Point2::new(0.0, 0.0)),
            SvgSegment::LineTo(Point2::new(1.0, 0.0)),
            SvgSegment::LineTo(Point2::new(1.0, 1.0)),
            SvgSegment::Close,
            SvgSegment::MoveTo(Point2::new(5.0, 5.0)),
            SvgSegment::LineTo(Point2::new(6.0, 5.0)),
            SvgSegment::LineTo(Point2::new(6.0, 6.0)),
            SvgSegment::Close,
        ];
        let polys = flatten_subpaths(segments);
        assert_eq!(polys.len(), 2);
        assert_eq!(polys[0].len(), 3);
        assert_eq!(polys[1].len(), 3);
    }

    #[test]
    fn flatten_subpaths_drops_degenerate_lone_moveto() {
        let segments = vec![SvgSegment::MoveTo(Point2::new(0.0, 0.0)), SvgSegment::Close];
        let polys = flatten_subpaths(segments);
        assert!(polys.is_empty());
    }

    #[test]
    fn flatten_subpaths_curve_expands_into_many_points() {
        let segments = vec![
            SvgSegment::MoveTo(Point2::new(0.0, 0.0)),
            SvgSegment::CubicTo(
                Point2::new(0.0, 10.0),
                Point2::new(10.0, 10.0),
                Point2::new(10.0, 0.0),
            ),
            SvgSegment::Close,
        ];
        let polys = flatten_subpaths(segments);
        assert_eq!(polys.len(), 1);
        // 1 start point + BEZIER_SEGMENTS sampled points.
        assert_eq!(polys[0].len(), 1 + BEZIER_SEGMENTS);
    }

    #[test]
    fn scale_and_place_scales_uniformly_and_offsets() {
        let polygons = vec![vec![Point2::new(0.0, 0.0), Point2::new(100.0, 50.0)]];
        // SVG is 100 units wide; target 10mm wide => scale = 0.1.
        let placed = scale_and_place(&polygons, 100.0, 10.0, 2.0, 3.0, false);
        assert_eq!(placed.len(), 1);
        assert_eq!(placed[0][0], (2.0, 3.0));
        assert_eq!(placed[0][1], (12.0, 8.0));
    }

    #[test]
    fn scale_and_place_zero_width_falls_back_to_identity_scale() {
        let polygons = vec![vec![Point2::new(5.0, 5.0)]];
        let placed = scale_and_place(&polygons, 0.0, 10.0, 0.0, 0.0, false);
        assert_eq!(placed[0][0], (5.0, 5.0));
    }

    #[test]
    fn scale_and_place_mirrors_inside_the_placement_box() {
        // An "L" hugging the left edge of a 100-unit-wide SVG, placed 10 mm
        // wide at x = 2: the box is x 2..12 either way.
        let polygons = vec![vec![
            Point2::new(0.0, 0.0),
            Point2::new(20.0, 0.0),
            Point2::new(20.0, 50.0),
        ]];
        let plain = scale_and_place(&polygons, 100.0, 10.0, 2.0, 3.0, false);
        let mirrored = scale_and_place(&polygons, 100.0, 10.0, 2.0, 3.0, true);
        assert_eq!(plain[0], vec![(2.0, 3.0), (4.0, 3.0), (4.0, 8.0)]);
        // The left edge lands on the right edge of the same box; Y is untouched.
        assert_eq!(mirrored[0], vec![(12.0, 3.0), (10.0, 3.0), (10.0, 8.0)]);
    }

    fn square(x: f64, y: f64, side: f64, clockwise: bool) -> Vec<Point2> {
        let mut ring = vec![
            Point2::new(x, y),
            Point2::new(x + side, y),
            Point2::new(x + side, y + side),
            Point2::new(x, y + side),
        ];
        if clockwise {
            ring.reverse();
        }
        ring
    }

    fn area(ring: &[Point2]) -> f64 {
        signed_area(ring).abs()
    }

    #[test]
    fn resolve_fill_evenodd_cuts_a_nested_subpath_as_a_hole() {
        // Both squares wind the same way: only the rule makes the inner a hole.
        let subpaths = vec![square(0.0, 0.0, 10.0, false), square(3.0, 3.0, 4.0, false)];
        let fill = resolve_fill(subpaths, FillRule::EvenOdd);
        assert_eq!(fill.hole_count, 1);
        assert!(!fill.subpaths_cross);
        assert_eq!(fill.polygons.len(), 1, "one outline carries the hole");
        // A slit has no area, so the outline's area is the square less the hole.
        assert!((area(&fill.polygons[0]) - 84.0).abs() < 1e-9);
        assert!(!point_in_ring(Point2::new(5.0, 5.0), &fill.polygons[0]));
        assert!(point_in_ring(Point2::new(1.5, 5.0), &fill.polygons[0]));
        assert!(point_in_ring(Point2::new(8.5, 4.0), &fill.polygons[0]));
    }

    #[test]
    fn resolve_fill_nonzero_fills_a_nested_subpath_wound_the_same_way() {
        let subpaths = vec![square(0.0, 0.0, 10.0, false), square(3.0, 3.0, 4.0, false)];
        let fill = resolve_fill(subpaths, FillRule::NonZero);
        assert_eq!(fill.hole_count, 0);
        assert_eq!(fill.polygons.len(), 1, "the inner square bounds nothing");
        assert!((area(&fill.polygons[0]) - 100.0).abs() < 1e-9);
    }

    #[test]
    fn resolve_fill_nonzero_cuts_a_nested_subpath_wound_the_other_way() {
        let subpaths = vec![square(0.0, 0.0, 10.0, false), square(3.0, 3.0, 4.0, true)];
        let fill = resolve_fill(subpaths, FillRule::NonZero);
        assert_eq!(fill.hole_count, 1);
        assert_eq!(fill.polygons.len(), 1);
        assert!((area(&fill.polygons[0]) - 84.0).abs() < 1e-9);
        assert!(!point_in_ring(Point2::new(5.0, 5.0), &fill.polygons[0]));
    }

    #[test]
    fn resolve_fill_keeps_an_island_inside_a_hole_as_its_own_outline() {
        let subpaths = vec![
            square(0.0, 0.0, 10.0, false),
            square(2.0, 2.0, 6.0, false),
            square(4.0, 4.0, 2.0, false),
        ];
        let fill = resolve_fill(subpaths, FillRule::EvenOdd);
        assert_eq!(fill.hole_count, 1);
        assert_eq!(fill.polygons.len(), 2);
        let mut areas: Vec<f64> = fill.polygons.iter().map(|ring| area(ring)).collect();
        areas.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!((areas[0] - 4.0).abs() < 1e-9, "the island");
        assert!((areas[1] - 64.0).abs() < 1e-9, "the frame: 100 - 36");
    }

    #[test]
    fn resolve_fill_joins_two_holes_on_one_row_without_crossing_either() {
        // The left hole's slit runs right and meets the right hole first.
        let subpaths = vec![
            square(0.0, 0.0, 12.0, false),
            square(2.0, 4.0, 2.0, true),
            square(7.0, 4.0, 2.0, true),
        ];
        let fill = resolve_fill(subpaths, FillRule::NonZero);
        assert_eq!(fill.hole_count, 2);
        assert_eq!(fill.polygons.len(), 1);
        let outline = &fill.polygons[0];
        assert!((area(outline) - (144.0 - 8.0)).abs() < 1e-9);
        assert!(!point_in_ring(Point2::new(3.0, 5.5), outline));
        assert!(!point_in_ring(Point2::new(8.0, 5.5), outline));
        assert!(point_in_ring(Point2::new(5.5, 5.5), outline));
        assert!(point_in_ring(Point2::new(10.5, 5.5), outline));
    }

    #[test]
    fn resolve_fill_does_not_guess_holes_when_subpaths_cross() {
        let subpaths = vec![square(0.0, 0.0, 10.0, false), square(5.0, 5.0, 10.0, false)];
        let fill = resolve_fill(subpaths, FillRule::EvenOdd);
        assert!(fill.subpaths_cross);
        assert_eq!(fill.hole_count, 0);
        assert_eq!(fill.polygons.len(), 2, "each subpath stays its own outline");
    }

    #[test]
    fn extract_polygons_reads_the_fill_rule_of_each_path() {
        let ring = "M0 0 L100 0 L100 100 L0 100 Z M30 30 L70 30 L70 70 L30 70 Z";
        let svg = |rule: &str| {
            format!(
                r##"<svg xmlns="http://www.w3.org/2000/svg" width="100" height="100">
                <path d="{ring}" fill="black" fill-rule="{rule}"/></svg>"##
            )
        };
        let evenodd = extract_polygons(&svg("evenodd")).expect("should parse");
        assert_eq!(evenodd.hole_count, 1);
        assert_eq!(evenodd.polygons.len(), 1);
        assert!((area(&evenodd.polygons[0]) - 8400.0).abs() < 1e-6);

        // Same winding, so nonzero paints straight across the inner square.
        let nonzero = extract_polygons(&svg("nonzero")).expect("should parse");
        assert_eq!(nonzero.hole_count, 0);
        assert_eq!(nonzero.polygons.len(), 1);
        assert!((area(&nonzero.polygons[0]) - 10000.0).abs() < 1e-6);
    }

    #[test]
    fn extract_polygons_applies_view_box_and_group_transforms() {
        // A 10-unit square in a 20-unit viewBox shown 100 px wide, shifted by
        // its group: it covers 25..75 px, not 0..10.
        let svg = r##"<svg xmlns="http://www.w3.org/2000/svg" width="100" height="100" viewBox="0 0 20 20">
            <g transform="translate(5 5)"><path d="M0 0 L10 0 L10 10 L0 10 Z" fill="black"/></g>
        </svg>"##;
        let logo = extract_polygons(svg).expect("should parse");
        assert_eq!(logo.polygons.len(), 1);
        let (lo, hi) = bounds(&logo.polygons[0]);
        assert_eq!((lo.x, lo.y, hi.x, hi.y), (25.0, 25.0, 75.0, 75.0));
    }

    #[test]
    fn extract_polygons_counts_text_elements_and_imports_none_of_them() {
        let svg = r##"<svg xmlns="http://www.w3.org/2000/svg" width="100" height="100">
            <path d="M0 0 L10 0 L10 10 Z" fill="black"/>
            <text x="10" y="50" font-family="Arial">REV A</text>
            <text x="10" y="80"><tspan>J3</tspan></text>
            <text x="10" y="90">  </text>
        </svg>"##;
        let logo = extract_polygons(svg).expect("should parse");
        assert_eq!(logo.text_element_count, 2, "the blank one draws nothing");
        assert_eq!(logo.polygons.len(), 1, "only the path");
    }

    #[test]
    fn extract_polygons_parses_simple_rect_path() {
        let svg = r##"<svg xmlns="http://www.w3.org/2000/svg" width="100" height="100">
            <path d="M0 0 L100 0 L100 100 L0 100 Z" fill="black"/>
        </svg>"##;
        let logo = extract_polygons(svg).expect("should parse");
        assert_eq!(logo.width, 100.0);
        assert_eq!(logo.height, 100.0);
        assert_eq!(logo.polygons.len(), 1);
        assert_eq!(logo.polygons[0].len(), 4);
    }

    #[test]
    fn extract_polygons_handles_curved_path() {
        let svg = r##"<svg xmlns="http://www.w3.org/2000/svg" width="50" height="50">
            <path d="M0 0 C0 25 25 25 25 0 Z" fill="black"/>
        </svg>"##;
        let logo = extract_polygons(svg).expect("should parse");
        assert_eq!(logo.polygons.len(), 1);
        // More than 2 points proves the curve was flattened, not dropped.
        assert!(logo.polygons[0].len() > 2);
    }

    #[test]
    fn extract_polygons_rejects_invalid_svg() {
        let result = extract_polygons("not an svg at all");
        assert!(result.is_err());
    }

    #[test]
    fn extract_polygons_empty_svg_has_no_polygons() {
        let svg = r##"<svg xmlns="http://www.w3.org/2000/svg" width="10" height="10"></svg>"##;
        let logo = extract_polygons(svg).expect("should parse");
        assert!(logo.polygons.is_empty());
    }
}
