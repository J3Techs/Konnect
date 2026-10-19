//! S0 offline ingest: build a [`RouteBoard`] from a saved `.kicad_pcb`.
//!
//! The text is parsed with `konnect-sexp`, and every pad, track, arc, via,
//! zone fill, keepout and courtyard is lowered to exact nanometre shapes.
//! Pad frames follow KiCad's conventions as `konnect-sexp` verified them:
//! a pad's `(at x y angle)` is footprint-local for the position and
//! board-space for the angle, and rotation is screen-CCW in the Y-down frame
//! ([`konnect_sexp::geometry::rotate_about`]).
//!
//! # Loss policy
//!
//! The loader never invents geometry. An item missing a load-bearing field is
//! skipped and counted in [`LoadReport::skipped`]; a pad it can only bound
//! (chamfered corners, a per-layer padstack, an unfilled custom primitive)
//! is stored as a superset, flagged `approximated`, and counted in
//! [`LoadReport::notes`]. A caller that needs a faithful model checks both
//! before trusting the board.

use crate::api::{
    ArcId, FootprintId, KeepoutId, LayerId, LayerMask, NetId, Ownership, PadId, TrackId, ViaId,
    ZoneId,
};
use crate::board::{
    pair_partners, ArcTrack, BoardOutline, CopperLayerKind, Courtyard, Drill, Footprint, Keepout,
    KeepoutRules, LayerTable, Net, Pad, PadKind, RouteBoard, Side, Track, Via, ZonePiece,
};
use konnect_geom::{
    approximate_arc, approximate_circle, mm_to_nm, ArcPolicy, ArcSide, BBox, Circle, ConvexPoly,
    Coord, Point, Polygon, Segment, Shape,
};
use konnect_sexp::board::{self as sexp_board, CourtyardShape, OutlineShape, ZoneOutlineElement};
use konnect_sexp::geometry::{rotate_about, transform_pad};
use konnect_sexp::net::{collect_net_keys, net_id, net_name};
use konnect_sexp::{parse_sexp, SexpNode};
use serde::Serialize;
use std::collections::{BTreeMap, HashMap};
use std::path::Path;

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct LoadOptions {
    /// How arcs (arc tracks, curved zone and courtyard edges, round
    /// outlines) are lowered.
    pub arc_policy: ArcPolicy,
    /// Layers new copper may use. `None`: every copper layer KiCad's layer
    /// table marks `signal` or `mixed`.
    pub routable_layers: Option<Vec<String>>,
}

/// What the loader dropped or approximated. Empty on a faithful load.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize)]
pub struct LoadReport {
    /// Items dropped for missing or unusable load-bearing fields, by kind.
    pub skipped: BTreeMap<&'static str, usize>,
    /// Approximations and ignored features, by description.
    pub notes: BTreeMap<&'static str, usize>,
}

impl LoadReport {
    fn skip(&mut self, kind: &'static str) {
        *self.skipped.entry(kind).or_default() += 1;
    }

    fn note(&mut self, what: &'static str) {
        *self.notes.entry(what).or_default() += 1;
    }

    pub fn skipped_total(&self) -> usize {
        self.skipped.values().sum()
    }
}

#[derive(Debug, thiserror::Error)]
pub enum LoadError {
    #[error("board file {path} could not be read: {message}")]
    Io { path: String, message: String },
    #[error("board could not be parsed: {0}")]
    Parse(String),
    #[error("board declares no copper layers")]
    NoCopperLayers,
    #[error("{0}")]
    Options(String),
}

/// Read and load a `.kicad_pcb` file.
pub fn load_board_file(
    path: &Path,
    options: &LoadOptions,
) -> Result<(RouteBoard, LoadReport), LoadError> {
    let text = std::fs::read_to_string(path).map_err(|e| LoadError::Io {
        path: path.display().to_string(),
        message: e.to_string(),
    })?;
    load_board_str(&text, options)
}

/// Load a `.kicad_pcb` from its text.
pub fn load_board_str(
    text: &str,
    options: &LoadOptions,
) -> Result<(RouteBoard, LoadReport), LoadError> {
    let tree = parse_sexp(text).map_err(|e| LoadError::Parse(e.to_string()))?;
    if tree.head() != Some("kicad_pcb") {
        return Err(LoadError::Parse(
            "the root node is not (kicad_pcb …)".to_string(),
        ));
    }
    let mut report = LoadReport::default();
    let policy = options.arc_policy;

    let layers = layer_table(&tree);
    if layers.is_empty() {
        return Err(LoadError::NoCopperLayers);
    }
    let routable = match &options.routable_layers {
        None => layers
            .copper
            .iter()
            .filter(|l| matches!(l.kind, CopperLayerKind::Signal | CopperLayerKind::Mixed))
            .fold(LayerMask::EMPTY, |m, l| m.with(l.id)),
        Some(names) => {
            let mut m = LayerMask::EMPTY;
            for n in names {
                let id = layers.by_name(n).ok_or_else(|| {
                    LoadError::Options(format!(
                        "routable layer '{n}' is not a copper layer of this board"
                    ))
                })?;
                m = m.with(id);
            }
            m
        }
    };

    // Nets: every named net, sorted, so ids are stable.
    let mut names: Vec<String> = collect_net_keys(&tree).into_iter().collect();
    names.sort();
    let partners = pair_partners(&names);
    let nets: Vec<Net> = names
        .iter()
        .zip(partners)
        .enumerate()
        .map(|(i, (name, pair))| Net {
            id: NetId(i as u32),
            name: name.clone(),
            class: Default::default(),
            pair,
        })
        .collect();
    let resolver = NetResolver::new(&tree, &names);

    let mut board = RouteBoard {
        outline: outline(&tree, &policy, &mut report),
        layers,
        routable,
        nets,
        ..Default::default()
    };

    load_footprints(&tree, &resolver, &policy, &mut board, &mut report);
    load_tracks(&tree, &resolver, &mut board, &mut report);
    load_zones(&tree, &resolver, &policy, &mut board, &mut report);

    debug_assert_eq!(board.validate(), Ok(()));
    Ok((board, report))
}

// ─── Layers and nets ─────────────────────────────────────────────────────────

fn layer_table(tree: &SexpNode) -> LayerTable {
    let mut copper = Vec::new();
    for entry in tree
        .find("layers")
        .and_then(SexpNode::children)
        .unwrap_or(&[])
        .iter()
        .skip(1)
    {
        let (Some(ordinal), Some(name)) = (
            entry
                .get(0)
                .and_then(SexpNode::as_str)
                .and_then(|s| s.parse::<u32>().ok()),
            entry.get(1).and_then(SexpNode::as_str),
        ) else {
            continue;
        };
        if !name.ends_with(".Cu") {
            continue;
        }
        let kind = match entry.get(2).and_then(SexpNode::as_str) {
            Some("power") => CopperLayerKind::Power,
            Some("mixed") => CopperLayerKind::Mixed,
            Some("jumper") => CopperLayerKind::Jumper,
            _ => CopperLayerKind::Signal,
        };
        copper.push((name.to_string(), ordinal, kind));
    }
    LayerTable::from_layers(copper)
}

/// Resolves an item's `(net …)` child in either file format.
struct NetResolver {
    by_name: HashMap<String, NetId>,
    /// KiCad ≤ 9 top-level `(net N "NAME")` table.
    legacy: HashMap<String, String>,
}

impl NetResolver {
    fn new(tree: &SexpNode, names: &[String]) -> Self {
        NetResolver {
            by_name: names
                .iter()
                .enumerate()
                .map(|(i, n)| (n.clone(), NetId(i as u32)))
                .collect(),
            legacy: tree
                .find_all("net")
                .into_iter()
                .filter_map(|n| Some((net_id(n)?.to_string(), net_name(n)?.to_string())))
                .collect(),
        }
    }

    /// `None` for no net node, the unconnected pseudo-net, or a name the
    /// board's net set does not hold.
    fn resolve(&self, item: &SexpNode) -> Option<NetId> {
        let node = item.find("net")?;
        let name = match net_name(node) {
            Some(n) => n.to_string(),
            None => {
                let id = net_id(node)?;
                if id == "0" {
                    return None;
                }
                self.legacy
                    .get(id)
                    .cloned()
                    .unwrap_or_else(|| id.to_string())
            }
        };
        if name.is_empty() {
            return None;
        }
        self.by_name.get(&name).copied()
    }
}

// ─── Small readers ───────────────────────────────────────────────────────────

fn xy(node: &SexpNode, tag: &str) -> Option<(f64, f64)> {
    let n = node.find(tag)?;
    let (x, y) = (n.get_f64(1)?, n.get_f64(2)?);
    (x.is_finite() && y.is_finite()).then_some((x, y))
}

fn pt(node: &SexpNode, tag: &str) -> Option<Point> {
    let (x, y) = xy(node, tag)?;
    Point::from_mm(x, y)
}

fn positive_nm(mm: f64) -> Option<Coord> {
    (mm.is_finite() && mm > 0.0).then(|| mm_to_nm(mm)).flatten()
}

/// KiCad's lock flag on an item: `(locked yes)`, or the legacy bare token.
fn is_locked(node: &SexpNode) -> bool {
    for child in node.children().unwrap_or(&[]) {
        if let SexpNode::Atom(a) = child {
            if a == "locked" {
                return true;
            }
        }
        if child.head() == Some("locked") {
            return matches!(child.get(1).and_then(SexpNode::as_str), Some("yes"));
        }
    }
    false
}

fn ownership(node: &SexpNode) -> Ownership {
    if is_locked(node) {
        Ownership::Locked
    } else {
        Ownership::Fixed
    }
}

fn uuid(node: &SexpNode) -> Option<String> {
    node.find_str("uuid").map(str::to_string)
}

/// A closed path of lossless outline elements as a ring; arcs approximated
/// on `side`.
fn elements_to_ring(
    elements: &[ZoneOutlineElement],
    policy: &ArcPolicy,
    side: ArcSide,
) -> Option<Vec<Point>> {
    let mut ring = Vec::new();
    for e in elements {
        match *e {
            ZoneOutlineElement::Point((x, y)) => ring.push(Point::from_mm(x, y)?),
            ZoneOutlineElement::Arc { start, mid, end } => {
                let (s, m, e) = (
                    Point::from_mm(start.0, start.1)?,
                    Point::from_mm(mid.0, mid.1)?,
                    Point::from_mm(end.0, end.1)?,
                );
                let pts = approximate_arc(s, m, e, policy, side);
                // The arc's end is the next element's start; keep it once.
                ring.extend_from_slice(&pts[..pts.len() - 1]);
                ring.push(e);
            }
        }
    }
    Some(ring)
}

// ─── Outline ─────────────────────────────────────────────────────────────────

fn outline(tree: &SexpNode, policy: &ArcPolicy, report: &mut LoadReport) -> BoardOutline {
    let mut out = BoardOutline::default();
    let shape = sexp_board::board_outline_shape(tree);
    let to_bbox = |(x0, y0, x1, y1): (f64, f64, f64, f64)| {
        Some(BBox::new(Point::from_mm(x0, y0)?, Point::from_mm(x1, y1)?))
    };
    out.bbox = shape.bbox().and_then(to_bbox);
    if let OutlineShape::Rectangular { bbox } = shape {
        if let Some(b) = to_bbox(bbox) {
            out.regions.extend(
                ConvexPoly::axis_rect(b, 0).and_then(|p| Polygon::new(p.points().to_vec(), vec![])),
            );
        }
    }
    let polyline = |pts: &[Point], closed: bool, edges: &mut Vec<Segment>| {
        for w in pts.windows(2) {
            edges.push(Segment::new(w[0], w[1], 0));
        }
        if closed && pts.len() > 2 {
            edges.push(Segment::new(pts[pts.len() - 1], pts[0], 0));
        }
    };
    for child in tree.children().unwrap_or(&[]) {
        let Some(head) = child.head() else { continue };
        if child.find_str("layer") != Some("Edge.Cuts") {
            continue;
        }
        let pts: Option<(Vec<Point>, bool)> = match head {
            "gr_line" => (|| Some((vec![pt(child, "start")?, pt(child, "end")?], false)))(),
            "gr_rect" => (|| {
                let (a, b) = (pt(child, "start")?, pt(child, "end")?);
                Some((vec![a, Point::new(b.x, a.y), b, Point::new(a.x, b.y)], true))
            })(),
            "gr_arc" => (|| {
                let (s, m, e) = (pt(child, "start")?, pt(child, "mid")?, pt(child, "end")?);
                Some((approximate_arc(s, m, e, policy, ArcSide::Inscribed), false))
            })(),
            "gr_circle" => (|| {
                let (c, e) = (pt(child, "center")?, pt(child, "end")?);
                let r = (e - c).length_f64().round() as Coord;
                Some((approximate_circle(c, r, policy, ArcSide::Inscribed), true))
            })(),
            "gr_poly" | "gr_curve" => (|| {
                if head == "gr_curve" {
                    report.note("Edge.Cuts gr_curve lowered to its control polygon");
                }
                let mut v = Vec::new();
                for p in child.find("pts")?.find_all("xy") {
                    v.push(Point::from_mm(p.get_f64(1)?, p.get_f64(2)?)?);
                }
                Some((v, head == "gr_poly"))
            })(),
            _ => continue,
        };
        match pts {
            Some((pts, closed)) => polyline(&pts, closed, &mut out.edges),
            None => report.skip("edge graphic"),
        }
    }
    out
}

// ─── Footprints and pads ─────────────────────────────────────────────────────

fn load_footprints(
    tree: &SexpNode,
    nets: &NetResolver,
    policy: &ArcPolicy,
    board: &mut RouteBoard,
    report: &mut LoadReport,
) {
    for fp in sexp_board::footprints(tree) {
        let root = (|| {
            let at = fp.find("at")?;
            let (x, y) = (at.get_f64(1)?, at.get_f64(2)?);
            let rot = at.get_f64(3).unwrap_or(0.0);
            let side = match fp.find_str("layer")? {
                "F.Cu" => Side::Front,
                "B.Cu" => Side::Back,
                _ => return None,
            };
            (x.is_finite() && y.is_finite() && rot.is_finite()).then_some((
                x,
                y,
                rot,
                side,
                Point::from_mm(x, y)?,
            ))
        })();
        let Some((fx, fy, rot, side, at)) = root else {
            report.skip("footprint");
            report
                .skipped
                .entry("pad")
                .and_modify(|n| *n += fp.find_all("pad").len())
                .or_insert(fp.find_all("pad").len());
            continue;
        };
        let id = FootprintId(board.footprints.len() as u32);
        let reference = fp
            .find_all("property")
            .into_iter()
            .find(|p| p.get(1).and_then(SexpNode::as_str) == Some("Reference"))
            .and_then(|p| p.get(2).and_then(SexpNode::as_str))
            .map(str::to_string);

        let courtyards = match sexp_board::footprint_courtyard_outline(fp) {
            Some(c) => lower_courtyards(&c.outlines, policy, report),
            None => {
                report.skip("courtyard");
                Vec::new()
            }
        };
        if fp.children().unwrap_or(&[]).iter().any(|c| {
            c.head().is_some_and(|h| h.starts_with("fp_"))
                && c.find_str("layer") == Some("Edge.Cuts")
        }) {
            report.note("footprint Edge.Cuts graphics not in the outline model");
        }

        let first_pad = board.pads.len() as u32;
        for pad in fp.find_all("pad") {
            match lower_pad(pad, (fx, fy, rot), id, nets, &board.layers, policy, report) {
                Some(mut p) => {
                    p.id = PadId(board.pads.len() as u32);
                    board.pads.push(p);
                }
                None => report.skip("pad"),
            }
        }
        board.footprints.push(Footprint {
            id,
            reference,
            at,
            rotation_deg: rot,
            side,
            locked: is_locked(fp),
            courtyards,
            pad_range: (first_pad, board.pads.len() as u32),
        });
    }
}

fn lower_courtyards(
    outlines: &[sexp_board::CourtyardOutline],
    policy: &ArcPolicy,
    report: &mut LoadReport,
) -> Vec<Courtyard> {
    let mut out = Vec::new();
    for o in outlines {
        let side = match o.side {
            sexp_board::Side::Front => Side::Front,
            sexp_board::Side::Back => Side::Back,
        };
        let ring = match &o.shape {
            CourtyardShape::Path(elements) => {
                elements_to_ring(elements, policy, ArcSide::Circumscribed)
            }
            CourtyardShape::Circle { center, radius } => Point::from_mm(center.0, center.1)
                .and_then(|c| {
                    Some(approximate_circle(
                        c,
                        mm_to_nm(*radius)?,
                        policy,
                        ArcSide::Circumscribed,
                    ))
                }),
        };
        match ring.and_then(|r| Polygon::new(r, vec![])) {
            Some(region) => out.push(Courtyard { side, region }),
            None => report.skip("courtyard outline"),
        }
    }
    out
}

/// The frame a pad's own shapes are drawn in: centre and board-space angle.
#[derive(Clone, Copy)]
struct PadFrame {
    cx: f64,
    cy: f64,
    angle: f64,
}

impl PadFrame {
    fn point(&self, dx: f64, dy: f64) -> Option<Point> {
        let (x, y) = rotate_about(dx, dy, self.cx, self.cy, self.angle);
        Point::from_mm(x, y)
    }

    /// A `w × h` rectangle around the frame origin, rounded by `r` (all mm).
    /// Degenerate cores become a stadium or a circle.
    fn rounded_rect(&self, w: f64, h: f64, r: f64) -> Option<Shape> {
        const EPS: f64 = 1e-9;
        let (ix, iy) = (w / 2.0 - r, h / 2.0 - r);
        let rn = if r > EPS { mm_to_nm(r)? } else { 0 };
        if ix <= EPS && iy <= EPS {
            return Some(Shape::Circle(Circle::new(
                self.point(0.0, 0.0)?,
                mm_to_nm(w.min(h) / 2.0)?,
            )));
        }
        if ix <= EPS {
            return Some(Shape::Segment(Segment::new(
                self.point(0.0, -iy)?,
                self.point(0.0, iy)?,
                2 * rn,
            )));
        }
        if iy <= EPS {
            return Some(Shape::Segment(Segment::new(
                self.point(-ix, 0.0)?,
                self.point(ix, 0.0)?,
                2 * rn,
            )));
        }
        let corners = [(-ix, -iy), (ix, -iy), (ix, iy), (-ix, iy)];
        let pts = corners
            .iter()
            .map(|&(x, y)| self.point(x, y))
            .collect::<Option<Vec<_>>>()?;
        ConvexPoly::new(pts, rn).map(Shape::ConvexPoly)
    }

    /// A stadium (KiCad `oval`) of size `w × h`.
    fn stadium(&self, w: f64, h: f64) -> Option<Shape> {
        self.rounded_rect(w, h, w.min(h) / 2.0)
    }
}

#[allow(clippy::too_many_arguments)]
fn lower_pad(
    pad: &SexpNode,
    (fx, fy, rot): (f64, f64, f64),
    footprint: FootprintId,
    nets: &NetResolver,
    layers: &LayerTable,
    policy: &ArcPolicy,
    report: &mut LoadReport,
) -> Option<Pad> {
    let number = pad.get(1)?.as_str()?.to_string();
    let kind = match pad.get(2)?.as_str()? {
        "smd" => PadKind::Smd,
        "thru_hole" => PadKind::ThroughHole,
        "np_thru_hole" => PadKind::NpThroughHole,
        "connect" => PadKind::Connect,
        _ => return None,
    };
    let shape_name = pad.get(3)?.as_str()?;
    let at = pad.find("at")?;
    let (lx, ly) = (at.get_f64(1)?, at.get_f64(2)?);
    let angle = at.get_f64(3).unwrap_or(0.0);
    if !(lx.is_finite() && ly.is_finite() && angle.is_finite()) {
        return None;
    }
    let (cx, cy) = transform_pad(lx, ly, fx, fy, rot);
    let frame = PadFrame { cx, cy, angle };
    let size = pad.find("size")?;
    let (w, h) = (size.get_f64(1)?, size.get_f64(2)?);
    if !(w.is_finite() && h.is_finite() && w > 0.0 && h > 0.0) {
        return None;
    }

    let mut approximated = false;
    let shapes = match shape_name {
        "circle" => vec![Shape::Circle(Circle::new(
            frame.point(0.0, 0.0)?,
            mm_to_nm(w / 2.0)?,
        ))],
        "rect" => vec![frame.rounded_rect(w, h, 0.0)?],
        "roundrect" => {
            let ratio = pad
                .find_f64("roundrect_rratio")
                .unwrap_or(0.25)
                .clamp(0.0, 0.5);
            vec![frame.rounded_rect(w, h, ratio * w.min(h))?]
        }
        "oval" => vec![frame.stadium(w, h)?],
        "trapezoid" => {
            // KiCad's trapezoid corners (PAD::BuildPadPolygon), half-deltas.
            let (ddx, ddy) = pad
                .find("rect_delta")
                .and_then(|d| Some((d.get_f64(1)? / 2.0, d.get_f64(2)? / 2.0)))
                .unwrap_or((0.0, 0.0));
            let (hx, hy) = (w / 2.0, h / 2.0);
            let corners = [
                (-hx - ddy, hy + ddx),
                (hx + ddy, hy - ddx),
                (hx - ddy, -hy + ddx),
                (-hx + ddy, -hy - ddx),
            ];
            let pts = corners
                .iter()
                .map(|&(x, y)| frame.point(x, y))
                .collect::<Option<Vec<_>>>()?;
            vec![Shape::ConvexPoly(ConvexPoly::new(pts, 0)?)]
        }
        "custom" => custom_pad_shapes(pad, &frame, w, h, policy, &mut approximated, report)?,
        _ => return None,
    };
    if pad.find("chamfer").is_some() && pad.find_f64("chamfer_ratio").unwrap_or(0.0) > 0.0 {
        approximated = true;
        report.note("pad chamfers stored as the unchamfered superset");
    }
    if pad.find("padstack").is_some() {
        approximated = true;
        report.note("per-layer padstack stored as its default shape on every layer");
    }

    let drill = match pad.find("drill") {
        Some(d) => lower_drill(d, &frame, kind)?,
        None => None,
    };
    let layer_tokens: Vec<&str> = pad
        .find("layers")
        .and_then(SexpNode::children)
        .unwrap_or(&[])
        .iter()
        .skip(1)
        .filter_map(SexpNode::as_str)
        .collect();

    Some(Pad {
        id: PadId(0),
        footprint,
        number,
        net: nets.resolve(pad),
        kind,
        center: frame.point(0.0, 0.0)?,
        angle_deg: angle,
        layers: layers.expand_all(layer_tokens),
        shapes,
        drill,
        local_clearance: pad
            .find_f64("clearance")
            .filter(|c| c.is_finite() && *c >= 0.0)
            .and_then(mm_to_nm),
        approximated,
        ownership: Ownership::Fixed,
        uuid: uuid(pad),
    })
}

/// `(drill [oval] W [H] [(offset X Y)])`. `Some(None)` for a zero drill.
fn lower_drill(d: &SexpNode, frame: &PadFrame, kind: PadKind) -> Option<Option<Drill>> {
    let mut oval = false;
    let mut dims = Vec::new();
    let mut offset = (0.0, 0.0);
    for c in d.children()?.iter().skip(1) {
        match c {
            SexpNode::Atom(a) if a == "oval" => oval = true,
            SexpNode::Atom(a) => dims.push(a.parse::<f64>().ok().filter(|v| v.is_finite())?),
            SexpNode::List(_) if c.head() == Some("offset") => {
                offset = (c.get_f64(1)?, c.get_f64(2)?)
            }
            _ => return None,
        }
    }
    let dw = *dims.first()?;
    let dh = if oval {
        *dims.get(1).unwrap_or(&dw)
    } else {
        dw
    };
    if dw <= 0.0 || dh <= 0.0 {
        return Some(None);
    }
    let (hx, hy) = rotate_about(offset.0, offset.1, frame.cx, frame.cy, frame.angle);
    let hole = PadFrame {
        cx: hx,
        cy: hy,
        angle: frame.angle,
    };
    let shape = if oval && (dw - dh).abs() > 1e-9 {
        hole.stadium(dw, dh)?
    } else {
        Shape::Circle(Circle::new(hole.point(0.0, 0.0)?, mm_to_nm(dw / 2.0)?))
    };
    Some(Some(Drill {
        shape,
        plated: kind == PadKind::ThroughHole,
    }))
}

/// A custom pad: its anchor plus every primitive, as a union of shapes.
fn custom_pad_shapes(
    pad: &SexpNode,
    frame: &PadFrame,
    w: f64,
    h: f64,
    policy: &ArcPolicy,
    approximated: &mut bool,
    report: &mut LoadReport,
) -> Option<Vec<Shape>> {
    let anchor = pad
        .find("options")
        .and_then(|o| o.find_str("anchor"))
        .unwrap_or("circle");
    let mut shapes = vec![match anchor {
        "rect" => frame.rounded_rect(w, h, 0.0)?,
        _ => Shape::Circle(Circle::new(
            frame.point(0.0, 0.0)?,
            mm_to_nm(w.min(h) / 2.0)?,
        )),
    }];
    let Some(prims) = pad.find("primitives") else {
        return Some(shapes);
    };
    for prim in prims.children()?.iter().skip(1) {
        let width = prim
            .find_f64("width")
            .filter(|v| v.is_finite() && *v >= 0.0)
            .unwrap_or(0.0);
        let filled = !matches!(prim.find_str("fill"), Some("no") | Some("none"));
        let wn = mm_to_nm(width)?;
        let local = |tag: &str| -> Option<Point> {
            let (x, y) = xy(prim, tag)?;
            frame.point(x, y)
        };
        match prim.head()? {
            "gr_poly" => {
                let mut ring = Vec::new();
                for p in prim.find("pts")?.find_all("xy") {
                    ring.push(frame.point(p.get_f64(1)?, p.get_f64(2)?)?);
                }
                if filled {
                    shapes.push(Shape::Polygon(Polygon::new(ring.clone(), vec![])?));
                } else {
                    *approximated = true;
                    report.note("unfilled custom-pad polygon stored filled");
                    shapes.push(Shape::Polygon(Polygon::new(ring.clone(), vec![])?));
                }
                if wn > 0 {
                    let n = ring.len();
                    for i in 0..n {
                        shapes.push(Shape::Segment(Segment::new(ring[i], ring[(i + 1) % n], wn)));
                    }
                }
            }
            "gr_circle" => {
                let c = local("center")?;
                let e = local("end")?;
                let r = (e - c).length_f64().round() as Coord;
                if !filled {
                    *approximated = true;
                    report.note("unfilled custom-pad circle stored filled");
                }
                shapes.push(Shape::Circle(Circle::new(c, r + wn / 2)));
            }
            "gr_rect" => {
                let (x0, y0) = xy(prim, "start")?;
                let (x1, y1) = xy(prim, "end")?;
                let pts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
                    .iter()
                    .map(|&(x, y)| frame.point(x, y))
                    .collect::<Option<Vec<_>>>()?;
                if !filled {
                    *approximated = true;
                    report.note("unfilled custom-pad rectangle stored filled");
                }
                shapes.push(Shape::ConvexPoly(ConvexPoly::new(pts, wn / 2)?));
            }
            "gr_line" => shapes.push(Shape::Segment(Segment::new(
                local("start")?,
                local("end")?,
                wn,
            ))),
            "gr_arc" => {
                let pts = approximate_arc(
                    local("start")?,
                    local("mid")?,
                    local("end")?,
                    policy,
                    ArcSide::Inscribed,
                );
                for w2 in pts.windows(2) {
                    shapes.push(Shape::Segment(Segment::new(w2[0], w2[1], wn)));
                }
            }
            _ => {
                *approximated = true;
                report.note("unsupported custom-pad primitive ignored");
            }
        }
    }
    Some(shapes)
}

// ─── Tracks, arcs and vias ───────────────────────────────────────────────────

fn load_tracks(
    tree: &SexpNode,
    nets: &NetResolver,
    board: &mut RouteBoard,
    report: &mut LoadReport,
) {
    for seg in tree.find_all("segment") {
        let parsed = (|| {
            let width = positive_nm(seg.find_f64("width")?)?;
            Some(Track {
                id: TrackId(board.tracks.len() as u32),
                net: nets.resolve(seg),
                layer: board.layers.by_name(seg.find_str("layer")?)?,
                seg: Segment::new(pt(seg, "start")?, pt(seg, "end")?, width),
                ownership: ownership(seg),
                uuid: uuid(seg),
            })
        })();
        match parsed {
            Some(t) => board.tracks.push(t),
            None => report.skip("track"),
        }
    }
    for arc in tree.find_all("arc") {
        let parsed = (|| {
            Some(ArcTrack {
                id: ArcId(board.arcs.len() as u32),
                net: nets.resolve(arc),
                layer: board.layers.by_name(arc.find_str("layer")?)?,
                start: pt(arc, "start")?,
                mid: pt(arc, "mid")?,
                end: pt(arc, "end")?,
                width: positive_nm(arc.find_f64("width")?)?,
                ownership: ownership(arc),
                uuid: uuid(arc),
            })
        })();
        match parsed {
            Some(a) => board.arcs.push(a),
            None => report.skip("arc"),
        }
    }
    for via in tree.find_all("via") {
        let parsed = (|| {
            let names: Vec<&str> = via
                .find("layers")?
                .children()?
                .iter()
                .skip(1)
                .filter_map(SexpNode::as_str)
                .collect();
            let ids: Vec<LayerId> = names
                .iter()
                .filter_map(|n| board.layers.by_name(n))
                .collect();
            let (first, last) = (*ids.iter().min()?, *ids.iter().max()?);
            let diameter = positive_nm(via.find_f64("size")?)?;
            let drill = positive_nm(via.find_f64("drill")?)?;
            Some(Via {
                id: ViaId(board.vias.len() as u32),
                net: nets.resolve(via),
                center: pt(via, "at")?,
                diameter,
                drill,
                layers: LayerMask::span(first, last),
                ownership: ownership(via),
                uuid: uuid(via),
            })
        })();
        match parsed {
            Some(v) => board.vias.push(v),
            None => report.skip("via"),
        }
    }
}

// ─── Zones and keepouts ──────────────────────────────────────────────────────

fn load_zones(
    tree: &SexpNode,
    nets: &NetResolver,
    policy: &ArcPolicy,
    board: &mut RouteBoard,
    report: &mut LoadReport,
) {
    let scan = sexp_board::zone_geometry(tree);
    for _ in 0..scan.skipped {
        report.skip("zone");
    }
    for z in &scan.items {
        let net = z.net.as_deref().and_then(|n| nets.by_name.get(n).copied());
        let layers = board.layers.expand_all(z.layers.iter().map(String::as_str));
        let side = if z.keepout.is_some() {
            ArcSide::Circumscribed
        } else {
            ArcSide::Inscribed
        };
        let region = (|| {
            let outline = elements_to_ring(&z.outline, policy, side)?;
            let holes = z
                .holes
                .iter()
                .map(|h| elements_to_ring(h, policy, side))
                .collect::<Option<Vec<_>>>()?;
            Polygon::new(outline, holes)
        })();

        if let Some(k) = z.keepout {
            match region {
                Some(region) => board.keepouts.push(Keepout {
                    id: KeepoutId(board.keepouts.len() as u32),
                    name: z.name.clone(),
                    layers,
                    region,
                    rules: KeepoutRules {
                        tracks: k.tracks,
                        vias: k.vias,
                        pads: k.pads,
                        copperpour: k.copperpour,
                        footprints: k.footprints,
                    },
                }),
                None => report.skip("keepout"),
            }
            continue;
        }

        if z.fills.is_empty() {
            // Unfilled: the authored outline bounds what a refill could pour.
            let Some(region) = region else {
                report.skip("zone");
                continue;
            };
            report.note("unfilled zone modelled by its authored outline");
            for layer in layers.iter() {
                board.zones.push(ZonePiece {
                    id: ZoneId(board.zones.len() as u32),
                    net,
                    layer,
                    region: region.clone(),
                    filled: false,
                    island: false,
                    zone_name: z.name.clone(),
                    priority: z.priority,
                });
            }
            continue;
        }
        for fill in &z.fills {
            let piece = (|| {
                let layer = board.layers.by_name(&fill.layer)?;
                let ring = fill
                    .points
                    .iter()
                    .map(|&(x, y)| Point::from_mm(x, y))
                    .collect::<Option<Vec<_>>>()?;
                Some((layer, Polygon::new(ring, vec![])?))
            })();
            match piece {
                Some((layer, region)) => board.zones.push(ZonePiece {
                    id: ZoneId(board.zones.len() as u32),
                    net,
                    layer,
                    region,
                    filled: true,
                    island: fill.island,
                    zone_name: z.name.clone(),
                    priority: z.priority,
                }),
                None => report.skip("zone fill"),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synthetic::MINIMAL_KICAD10_BOARD;
    use konnect_geom::{point_in_polygon, Containment};

    fn load() -> (RouteBoard, LoadReport) {
        load_board_str(MINIMAL_KICAD10_BOARD, &LoadOptions::default()).unwrap()
    }

    #[test]
    fn layers_nets_and_routable_mask() {
        let (b, report) = load();
        assert_eq!(report.skipped_total(), 0, "{report:?}");
        let names: Vec<&str> = b.layers.copper.iter().map(|l| l.name.as_str()).collect();
        assert_eq!(names, vec!["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]);
        // In1 is a power layer: not routable by default.
        assert_eq!(
            b.routable.iter().collect::<Vec<_>>(),
            vec![LayerId(0), LayerId(2), LayerId(3)]
        );
        let net_names: Vec<&str> = b.nets.iter().map(|n| n.name.as_str()).collect();
        assert_eq!(net_names, vec!["D+", "D-", "GND", "SIG"]);
        assert_eq!(b.nets[0].pair, Some(NetId(1)));
        assert_eq!(b.validate(), Ok(()));
    }

    #[test]
    fn pads_land_in_board_space_with_exact_shapes() {
        let (b, _) = load();
        let r1 = b.footprint_by_reference("R1").unwrap();
        let fp = &b.footprints[r1.index()];
        assert_eq!(fp.pad_range, (0, 2));
        // R1 at (20, 10) rotated 90: local (-0.5, 0) lands at (20, 10.5).
        let p1 = &b.pads[0];
        assert_eq!(p1.center, Point::new(20_000_000, 10_500_000));
        assert_eq!(p1.net, b.net_by_name("SIG"));
        assert_eq!(p1.layers.iter().collect::<Vec<_>>(), vec![LayerId(0)]);
        // roundrect 0.6 x 0.5, rratio 0.25 → radius 0.125 around a
        // 0.35 x 0.25 core, rotated 90.
        let Shape::ConvexPoly(poly) = &p1.shapes[0] else {
            panic!(
                "roundrect lowers to a rounded convex polygon: {:?}",
                p1.shapes[0]
            );
        };
        assert_eq!(poly.radius(), 125_000);
        let bb = poly.bbox();
        assert_eq!((bb.width(), bb.height()), (500_000, 600_000));

        let j1 = &b.footprints[b.footprint_by_reference("J1").unwrap().index()];
        let th = &b.pads[j1.pad_range.0 as usize];
        assert_eq!(th.kind, PadKind::ThroughHole);
        assert_eq!(th.layers, b.layers.all());
        let drill = th.drill.as_ref().unwrap();
        assert!(drill.plated);
        // Oval drill 0.6 x 1.2: a vertical stadium of width 0.6.
        let Shape::Segment(slot) = drill.shape else {
            panic!("oblong drill lowers to a stadium: {:?}", drill.shape);
        };
        assert_eq!(slot.width, 600_000);
        assert_eq!((slot.b - slot.a).length2(), 600_000i128 * 600_000);
    }

    #[test]
    fn courtyards_tracks_vias_and_ownership() {
        let (b, _) = load();
        let fp = &b.footprints[b.footprint_by_reference("R1").unwrap().index()];
        assert_eq!(fp.courtyards.len(), 1);
        let c = &fp.courtyards[0].region;
        assert_eq!(
            point_in_polygon(Point::new(20_000_000, 10_000_000), c),
            Containment::Inside
        );
        assert_eq!(
            point_in_polygon(Point::new(21_500_000, 10_000_000), c),
            Containment::Outside
        );

        assert_eq!(b.tracks.len(), 2);
        assert_eq!(b.tracks[0].ownership, Ownership::Locked);
        assert_eq!(b.tracks[1].ownership, Ownership::Fixed);
        assert_eq!(b.tracks[0].seg.width, 200_000);
        assert_eq!(b.arcs.len(), 1);
        assert_eq!(b.vias.len(), 1);
        assert_eq!(b.vias[0].layers, b.layers.all());
        assert_eq!((b.vias[0].diameter, b.vias[0].drill), (600_000, 300_000));
    }

    #[test]
    fn zones_fills_and_keepouts() {
        let (b, report) = load();
        // The GND zone has one fill piece on In1; the keepout is separate.
        assert_eq!(b.zones.len(), 1);
        let z = &b.zones[0];
        assert!(z.filled);
        assert_eq!(z.net, b.net_by_name("GND"));
        assert_eq!(b.layers.name(z.layer), "In1.Cu");
        assert_eq!(b.keepouts.len(), 1);
        let k = &b.keepouts[0];
        assert_eq!(k.name.as_deref(), Some("no_vias"));
        assert!(k.rules.vias && !k.rules.tracks);
        assert_eq!(k.layers.count(), 4);
        assert_eq!(b.outline.regions.len(), 1);
        assert_eq!(b.outline.edges.len(), 4);
        assert!(report.notes.is_empty(), "{:?}", report.notes);
    }

    #[test]
    fn non_boards_are_refused() {
        assert!(matches!(
            load_board_str("(kicad_sch (version 1))", &LoadOptions::default()),
            Err(LoadError::Parse(_))
        ));
        let opts = LoadOptions {
            routable_layers: Some(vec!["In9.Cu".to_string()]),
            ..Default::default()
        };
        assert!(matches!(
            load_board_str(MINIMAL_KICAD10_BOARD, &opts),
            Err(LoadError::Options(_))
        ));
    }
}
