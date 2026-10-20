//! `lint_board_geometry` and `plan_dogleg_fixes` (`native_routing`).
//!
//! `add_routing` lints the copper it is about to create; these two tools
//! look at the copper already on a saved board. The lint reuses
//! [`route_geometry::lint`] over every track (acute bends, short jogs,
//! hairpins, collinear vertices, unsplit T-junctions) and adds two shapes
//! that only make sense on a finished board:
//!
//! - a **via dogleg**: a track chain ends on a via through a short last
//!   segment that bends off the axis of the longer segment before it, so the
//!   via sits slightly off that axis;
//! - a **pad dogleg**: the same bend, but the short segment runs into a pad.
//!
//! The planner removes via doglegs. A via is moved to the one point where
//! the axis of the longer segment meets the axes of every other track on
//! the via; the short segment goes, the longer one is extended (or
//! shortened) straight to the new point and every other track keeps its
//! direction and is re-ended there. A via whose tracks do not agree on one
//! point, whose new point would bend an octilinear track, or whose
//! replacement fails the exact check against the board without the removed
//! items, is reported and left alone. Pad doglegs are reported, never fixed.
//!
//! Both read the saved file only and never write the board; the planner
//! writes a routing_file of the new copper and a companion file listing the
//! UUIDs to delete.

use crate::mcp::{error::ToolErrorKind, protocol::CallToolResult};
use crate::native_route_job::{self, guarded};
use crate::tools::board_inspect::{self, box_arg, layers_arg, nets_arg, pad_label};
use crate::tools::route_geometry::{self, heading, turn, Limits, STRAIGHT_DEG};
use crate::tools::route_native::{
    base_spec, invalid, loaded, opt_count, opt_path, opt_strings, stage_failure, Refusal,
};
use crate::tools::{get_path, ToolContext};
use konnect_geom::{nm_to_mm, point_shape_distance_cmp, touches, BBox, Point, Shape};
use konnect_route::api::{Generation, ItemRef, LayerId, LayerMask, NetId, Ownership};
use konnect_route::check::{BoardChecker, CheckOptions, CheckedItem, ViolationKind};
use konnect_route::plan::{RoutedTrack, RoutedVia};
use konnect_route::{RouteBoard, RoutePlan, RoutedCopper};
use serde_json::{json, Value};
use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::path::{Path, PathBuf};

/// Track ends closer than this (nm) are one joint (`route_geometry`'s 0.5 µm).
const JOIN_NM: i64 = 500;
/// Bucket of the end index (nm); at least [`JOIN_NM`], so a 3 x 3 look finds
/// every end within reach.
const END_CELL_NM: i64 = 1_000;
/// Bucket of the track, via and pad indexes (nm).
const CELL_NM: i64 = 1_000_000;
/// How far (nm) a track's axis may pass from the new via position and still
/// count as meeting it.
const AXIS_TOL_NM: f64 = 2.0;
/// Default and largest `dogleg_length_mm`.
const DOGLEG_DEFAULT_MM: f64 = 1.0;
const DOGLEG_MAX_MM: f64 = 10.0;
/// Finding kinds, in the order they are counted and listed.
pub(crate) const KINDS: [&str; 7] = [
    "via_dogleg",
    "pad_dogleg",
    "acute_bend",
    "short_jog",
    "hairpin",
    "unsplit_t_junction",
    "collinear_vertex",
];
/// Side of a hotspot cell, mm.
const HOTSPOT_MM: f64 = 10.0;
const HOTSPOTS: usize = 10;
const TOP_NETS: usize = 100;

// ─── Index of the saved copper ───────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum EndItem {
    Track(u32),
    Arc(u32),
}

#[derive(Debug, Clone, Copy)]
struct End {
    item: EndItem,
    /// The item's start (`a`) end, or its end (`b`).
    at_a: bool,
    net: Option<NetId>,
    p: Point,
}

fn cell(v: i32, size: i64) -> i64 {
    i64::from(v).div_euclid(size)
}

fn near(p: Point, q: Point, reach: i64) -> bool {
    let (dx, dy) = (i128::from(p.x - q.x), i128::from(p.y - q.y));
    dx * dx + dy * dy <= i128::from(reach) * i128::from(reach)
}

fn cells_of(b: &BBox) -> impl Iterator<Item = (i64, i64)> {
    let (x0, y0) = (cell(b.min.x, CELL_NM), cell(b.min.y, CELL_NM));
    let (x1, y1) = (cell(b.max.x, CELL_NM), cell(b.max.y, CELL_NM));
    (x0..=x1).flat_map(move |x| (y0..=y1).map(move |y| (x, y)))
}

/// Hashed lookups over one board's copper: track and arc ends by layer,
/// vias, pads and tracks by 1 mm cell. Built once per call.
pub(crate) struct CopperIndex<'b> {
    board: &'b RouteBoard,
    ends: HashMap<(u8, i64, i64), Vec<End>>,
    vias: HashMap<(i64, i64), Vec<u32>>,
    pads: HashMap<(i64, i64), Vec<u32>>,
    tracks: HashMap<(u8, i64, i64), Vec<u32>>,
    arcs: HashMap<(u8, i64, i64), Vec<u32>>,
}

impl<'b> CopperIndex<'b> {
    pub(crate) fn new(board: &'b RouteBoard) -> Self {
        let mut ix = CopperIndex {
            board,
            ends: HashMap::new(),
            vias: HashMap::new(),
            pads: HashMap::new(),
            tracks: HashMap::new(),
            arcs: HashMap::new(),
        };
        for (i, t) in board.tracks.iter().enumerate() {
            for (at_a, p) in [(true, t.seg.a), (false, t.seg.b)] {
                ix.add_end(t.layer, EndItem::Track(i as u32), at_a, t.net, p);
            }
            let bb = t.seg.bbox().inflate(t.seg.width / 2 + 1);
            for (x, y) in cells_of(&bb) {
                ix.tracks
                    .entry((t.layer.0, x, y))
                    .or_default()
                    .push(i as u32);
            }
        }
        for (i, a) in board.arcs.iter().enumerate() {
            for (at_a, p) in [(true, a.start), (false, a.end)] {
                ix.add_end(a.layer, EndItem::Arc(i as u32), at_a, a.net, p);
            }
            let bb = BBox::from_points([a.start, a.mid, a.end])
                .expect("three points")
                .inflate(a.width / 2 + 1);
            for (x, y) in cells_of(&bb) {
                ix.arcs.entry((a.layer.0, x, y)).or_default().push(i as u32);
            }
        }
        for (i, v) in board.vias.iter().enumerate() {
            ix.vias
                .entry((cell(v.center.x, CELL_NM), cell(v.center.y, CELL_NM)))
                .or_default()
                .push(i as u32);
        }
        for (i, p) in board.pads.iter().enumerate() {
            if let Some(bb) = p.shapes.iter().map(Shape::bbox).reduce(BBox::union) {
                for c in cells_of(&bb) {
                    ix.pads.entry(c).or_default().push(i as u32);
                }
            }
        }
        ix
    }

    fn add_end(&mut self, layer: LayerId, item: EndItem, at_a: bool, net: Option<NetId>, p: Point) {
        self.ends
            .entry((layer.0, cell(p.x, END_CELL_NM), cell(p.y, END_CELL_NM)))
            .or_default()
            .push(End { item, at_a, net, p });
    }

    /// Track and arc ends of `net` on `layer` within [`JOIN_NM`] of `p`.
    fn ends_at(&self, p: Point, layer: LayerId, net: Option<NetId>) -> Vec<End> {
        let (cx, cy) = (cell(p.x, END_CELL_NM), cell(p.y, END_CELL_NM));
        let mut out = Vec::new();
        for dx in -1..=1 {
            for dy in -1..=1 {
                for e in self
                    .ends
                    .get(&(layer.0, cx + dx, cy + dy))
                    .into_iter()
                    .flatten()
                {
                    if e.net == net && near(e.p, p, JOIN_NM) {
                        out.push(*e);
                    }
                }
            }
        }
        out.sort_by_key(|e| (e.item, e.at_a));
        out
    }

    /// Vias of `net` spanning `layer` whose centre is within [`JOIN_NM`] of `p`.
    fn vias_at(&self, p: Point, layer: LayerId, net: Option<NetId>) -> Vec<u32> {
        let (cx, cy) = (cell(p.x, CELL_NM), cell(p.y, CELL_NM));
        let mut out = Vec::new();
        for dx in -1..=1 {
            for dy in -1..=1 {
                for &i in self.vias.get(&(cx + dx, cy + dy)).into_iter().flatten() {
                    let v = &self.board.vias[i as usize];
                    if v.net == net && v.layers.contains(layer) && near(v.center, p, JOIN_NM) {
                        out.push(i);
                    }
                }
            }
        }
        out.sort_unstable();
        out
    }

    /// Pads with copper on `layer` that contain `p`.
    fn pads_at(&self, p: Point, layer: LayerId) -> Vec<u32> {
        let mut out: Vec<u32> = self
            .pads
            .get(&(cell(p.x, CELL_NM), cell(p.y, CELL_NM)))
            .into_iter()
            .flatten()
            .copied()
            .filter(|&i| {
                let pad = &self.board.pads[i as usize];
                pad.layers.contains(layer)
                    && pad
                        .shapes
                        .iter()
                        .any(|s| point_shape_distance_cmp(p, 0, s, 0) != Ordering::Greater)
            })
            .collect();
        out.sort_unstable();
        out.dedup();
        out
    }

    /// Tracks on `layer` whose copper touches `shape`.
    fn tracks_touching(&self, shape: &Shape, layer: LayerId) -> Vec<u32> {
        let mut out: Vec<u32> = cells_of(&shape.bbox())
            .flat_map(|(x, y)| self.tracks.get(&(layer.0, x, y)).into_iter().flatten())
            .copied()
            .collect();
        out.sort_unstable();
        out.dedup();
        out.retain(|&i| touches(&Shape::Segment(self.board.tracks[i as usize].seg), shape));
        out
    }

    /// Arcs on `layer` whose copper touches `shape`.
    fn arcs_touching(&self, shape: &Shape, layer: LayerId) -> Vec<u32> {
        let policy = konnect_geom::ArcPolicy::default();
        let mut out: Vec<u32> = cells_of(&shape.bbox())
            .flat_map(|(x, y)| self.arcs.get(&(layer.0, x, y)).into_iter().flatten())
            .copied()
            .collect();
        out.sort_unstable();
        out.dedup();
        out.retain(|&i| {
            self.board.arcs[i as usize]
                .segments(&policy)
                .into_iter()
                .any(|s| touches(&Shape::Segment(s), shape))
        });
        out
    }

    /// Pads whose copper touches `shape` on a layer of `layers`.
    fn pads_touching(&self, shape: &Shape, layers: LayerMask) -> Vec<u32> {
        let mut out: Vec<u32> = cells_of(&shape.bbox())
            .flat_map(|c| self.pads.get(&c).into_iter().flatten())
            .copied()
            .collect();
        out.sort_unstable();
        out.dedup();
        out.retain(|&i| {
            let pad = &self.board.pads[i as usize];
            pad.layers.intersects(layers) && pad.shapes.iter().any(|s| touches(s, shape))
        });
        out
    }
}

// ─── Small geometry in f64 nanometres ────────────────────────────────────────

type V2 = (f64, f64);

fn v(p: Point) -> V2 {
    (f64::from(p.x), f64::from(p.y))
}

fn sub(a: V2, b: V2) -> V2 {
    (a.0 - b.0, a.1 - b.1)
}

fn dot(a: V2, b: V2) -> f64 {
    a.0 * b.0 + a.1 * b.1
}

fn cross(a: V2, b: V2) -> f64 {
    a.0 * b.1 - a.1 * b.0
}

fn norm(a: V2) -> f64 {
    a.0.hypot(a.1)
}

/// An infinite line through `p` along `d`.
#[derive(Debug, Clone, Copy)]
struct Line {
    p: V2,
    d: V2,
}

impl Line {
    fn through(from: Point, to: Point) -> Line {
        Line {
            p: v(from),
            d: sub(v(to), v(from)),
        }
    }

    fn distance(&self, q: V2) -> f64 {
        cross(sub(q, self.p), self.d).abs() / norm(self.d)
    }

    fn project(&self, q: V2) -> V2 {
        let t = dot(sub(q, self.p), self.d) / dot(self.d, self.d);
        (self.p.0 + self.d.0 * t, self.p.1 + self.d.1 * t)
    }

    fn meet(&self, o: &Line) -> Option<V2> {
        let den = cross(self.d, o.d);
        if den.abs() <= 1e-9 * norm(self.d) * norm(o.d) {
            return None;
        }
        let t = cross(sub(o.p, self.p), o.d) / den;
        Some((self.p.0 + self.d.0 * t, self.p.1 + self.d.1 * t))
    }
}

/// A direction along 0, 45 or 90 degrees, exactly.
fn octilinear(dx: i64, dy: i64) -> bool {
    dx == 0 || dy == 0 || dx.abs() == dy.abs()
}

fn mm_pt(p: Point) -> Value {
    let (x, y) = p.to_mm();
    json!([x, y])
}

fn len_mm(a: Point, b: Point) -> f64 {
    route_geometry::round4(norm(sub(v(b), v(a))) / 1e6)
}

fn round2(x: f64) -> f64 {
    (x * 100.0).round() / 100.0
}

// ─── Doglegs ─────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum DoglegEnd {
    Via(u32),
    Pad(u32),
}

/// A short last segment bending into a via or pad off the axis of the
/// longer segment before it.
#[derive(Debug, Clone)]
pub(crate) struct Dogleg {
    pub end: DoglegEnd,
    pub net: Option<NetId>,
    pub layer: LayerId,
    /// The short segment (deleted by a fix) and the longer one before it.
    pub short: u32,
    pub long: u32,
    /// The short segment's end on the via or pad.
    pub at: Point,
    /// Where the short and the long segment meet.
    pub joint: Point,
    /// The long segment's far end.
    pub far: Point,
    /// Angle at the joint between the two segments (180 is straight).
    pub interior_deg: f64,
    /// Perpendicular distance of `at` from the long segment's axis, nm.
    pub offset_nm: f64,
}

/// The dogleg whose short segment is track `s`, ending on the via or pad at
/// its `at_a` end, if the other end is a bare joint with one longer track
/// that is not collinear with it.
fn dogleg_from(
    ix: &CopperIndex,
    s: u32,
    at_a: bool,
    dogleg_nm: f64,
    end: DoglegEnd,
) -> Option<Dogleg> {
    let board = ix.board;
    let t = &board.tracks[s as usize];
    let len = t.seg.length() as f64;
    if len <= 0.0 || len >= dogleg_nm {
        return None;
    }
    let (e, j) = if at_a {
        (t.seg.a, t.seg.b)
    } else {
        (t.seg.b, t.seg.a)
    };
    let at_j = ix.ends_at(j, t.layer, t.net);
    if at_j.len() != 2 {
        return None;
    }
    let own = at_j
        .iter()
        .any(|x| x.item == EndItem::Track(s) && x.at_a != at_a);
    let other = at_j
        .iter()
        .find(|x| !(x.item == EndItem::Track(s) && x.at_a != at_a))?;
    let EndItem::Track(long) = other.item else {
        return None;
    };
    if !own || long == s {
        return None;
    }
    if !ix.vias_at(j, t.layer, t.net).is_empty() || !ix.pads_at(j, t.layer).is_empty() {
        return None;
    }
    let l = &board.tracks[long as usize];
    if (l.seg.length() as f64) <= len {
        return None;
    }
    let far = if other.at_a { l.seg.b } else { l.seg.a };
    let interior = turn(heading(v(j), v(far)), heading(v(j), v(e))).abs();
    if 180.0 - interior < STRAIGHT_DEG {
        return None;
    }
    Some(Dogleg {
        end,
        net: t.net,
        layer: t.layer,
        short: s,
        long,
        at: e,
        joint: j,
        far,
        interior_deg: interior,
        offset_nm: Line::through(far, j).distance(v(e)),
    })
}

/// Which nets and layers a call looks at.
#[derive(Debug, Clone, Default)]
pub(crate) struct Scope {
    pub nets: Option<BTreeSet<NetId>>,
    pub layers: Option<LayerMask>,
}

impl Scope {
    fn net(&self, n: Option<NetId>) -> bool {
        match &self.nets {
            None => n.is_some(),
            Some(set) => n.is_some_and(|n| set.contains(&n)),
        }
    }

    fn layer(&self, l: LayerId) -> bool {
        self.layers.is_none_or(|m| m.contains(l))
    }
}

/// Every via and pad dogleg in scope, vias first (by via, layer, short
/// segment), then pads (by short segment).
pub(crate) fn find_doglegs(ix: &CopperIndex, scope: &Scope, dogleg_nm: f64) -> Vec<Dogleg> {
    let board = ix.board;
    let mut out = Vec::new();
    for (vi, via) in board.vias.iter().enumerate() {
        if !scope.net(via.net) {
            continue;
        }
        for layer in via.layers.iter().filter(|l| scope.layer(*l)) {
            for e in ix.ends_at(via.center, layer, via.net) {
                if let EndItem::Track(s) = e.item {
                    out.extend(dogleg_from(
                        ix,
                        s,
                        e.at_a,
                        dogleg_nm,
                        DoglegEnd::Via(vi as u32),
                    ));
                }
            }
        }
    }
    for (si, t) in board.tracks.iter().enumerate() {
        if !scope.net(t.net) || !scope.layer(t.layer) || (t.seg.length() as f64) >= dogleg_nm {
            continue;
        }
        for (at_a, e) in [(true, t.seg.a), (false, t.seg.b)] {
            if !ix.vias_at(e, t.layer, t.net).is_empty() {
                continue;
            }
            let pads: Vec<u32> = ix
                .pads_at(e, t.layer)
                .into_iter()
                .filter(|&p| board.pads[p as usize].net == t.net)
                .collect();
            if let Some(&pad) = pads.first() {
                out.extend(dogleg_from(
                    ix,
                    si as u32,
                    at_a,
                    dogleg_nm,
                    DoglegEnd::Pad(pad),
                ));
            }
        }
    }
    out
}

// ─── The board lint ──────────────────────────────────────────────────────────

/// One lint finding on the saved board.
#[derive(Debug, Clone)]
pub(crate) struct BoardFinding {
    pub kind: &'static str,
    pub net: Option<NetId>,
    pub layer: LayerId,
    /// mm
    pub at: (f64, f64),
    /// Track index and its role in the finding.
    pub tracks: Vec<(u32, &'static str)>,
    pub vias: Vec<u32>,
    pub pads: Vec<u32>,
    /// Interior angles (180 is straight).
    pub angles_deg: Vec<f64>,
    pub offset_mm: Option<f64>,
    pub joint: Option<Point>,
    /// "via" or "pad" when the finding's point lies on one.
    pub on: Option<&'static str>,
    pub detail: String,
}

#[derive(Debug, Clone, Default)]
pub(crate) struct LintStats {
    pub tracks: usize,
    pub tracks_without_net: usize,
    pub zero_length_tracks: usize,
    pub arcs_not_linted: usize,
    pub vias: usize,
    pub collinear_at_via_or_pad: usize,
}

fn kind_order(kind: &str) -> usize {
    KINDS.iter().position(|k| *k == kind).unwrap_or(KINDS.len())
}

fn roles(kind: &str, n: usize) -> Vec<&'static str> {
    match kind {
        "short_jog" | "hairpin" => vec!["before", "short", "after"],
        "unsplit_t_junction" => {
            let mut r = vec!["ending"; n.saturating_sub(1)];
            r.push("passing");
            r
        }
        _ => vec!["segment"; n],
    }
}

/// Lint every track of `scope` on the board, plus its doglegs. Collinear
/// vertices on a via or pad are left out: there the vertex is the
/// connection.
pub(crate) fn lint_board(
    ix: &CopperIndex,
    scope: &Scope,
    limits: &Limits,
    dogleg_nm: f64,
) -> (Vec<BoardFinding>, LintStats) {
    let board = ix.board;
    let mut stats = LintStats {
        arcs_not_linted: board
            .arcs
            .iter()
            .filter(|a| scope.net(a.net) && scope.layer(a.layer))
            .count(),
        vias: board.vias.iter().filter(|v| scope.net(v.net)).count(),
        ..LintStats::default()
    };
    let mut segments = Vec::new();
    let mut owner: Vec<u32> = Vec::new();
    for (i, t) in board.tracks.iter().enumerate() {
        if t.net.is_none() {
            stats.tracks_without_net += 1;
            continue;
        }
        if !scope.net(t.net) || !scope.layer(t.layer) {
            continue;
        }
        if t.seg.a == t.seg.b {
            stats.zero_length_tracks += 1;
            continue;
        }
        stats.tracks += 1;
        let net = t.net.expect("checked above");
        segments.push(route_geometry::Segment {
            net: board.net(net).name.clone(),
            layer: board.layers.name(t.layer).to_string(),
            width: nm_to_mm(i64::from(t.seg.width)),
            a: t.seg.a.to_mm(),
            b: t.seg.b.to_mm(),
        });
        owner.push(i as u32);
    }
    let report = route_geometry::lint(&segments, limits);
    let mut out = Vec::new();
    for f in report.findings {
        let first = &board.tracks[owner[f.segments[0]] as usize];
        let (net, layer) = (first.net, first.layer);
        let mut vias = Vec::new();
        let mut pads = Vec::new();
        let mut on = None;
        if matches!(
            f.kind,
            "acute_bend" | "collinear_vertex" | "unsplit_t_junction"
        ) {
            if let Some(p) = Point::from_mm(f.at.0, f.at.1) {
                vias = ix.vias_at(p, layer, net);
                if vias.is_empty() {
                    pads = ix.pads_at(p, layer);
                }
                on = if !vias.is_empty() {
                    Some("via")
                } else if !pads.is_empty() {
                    Some("pad")
                } else {
                    None
                };
            }
        }
        if f.kind == "collinear_vertex" && on.is_some() {
            stats.collinear_at_via_or_pad += 1;
            continue;
        }
        let r = roles(f.kind, f.segments.len());
        out.push(BoardFinding {
            kind: f.kind,
            net,
            layer,
            at: f.at,
            tracks: f
                .segments
                .iter()
                .zip(r)
                .map(|(&k, role)| (owner[k], role))
                .collect(),
            vias,
            pads,
            angles_deg: f.angles_deg,
            offset_mm: None,
            joint: None,
            on,
            detail: f.detail,
        });
    }
    for d in find_doglegs(ix, scope, dogleg_nm) {
        out.push(dogleg_finding(board, &d));
    }
    sort_findings(board, &mut out);
    (out, stats)
}

fn dogleg_finding(board: &RouteBoard, d: &Dogleg) -> BoardFinding {
    let s = &board.tracks[d.short as usize];
    let l = &board.tracks[d.long as usize];
    let (kind, vias, pads, into) = match d.end {
        DoglegEnd::Via(v) => ("via_dogleg", vec![v], Vec::new(), "the via"),
        DoglegEnd::Pad(p) => ("pad_dogleg", Vec::new(), vec![p], "the pad"),
    };
    let offset_mm = route_geometry::round4(d.offset_nm / 1e6);
    BoardFinding {
        kind,
        net: d.net,
        layer: d.layer,
        at: d.at.to_mm(),
        tracks: vec![(d.short, "short"), (d.long, "long")],
        vias,
        pads,
        angles_deg: vec![d.interior_deg],
        offset_mm: Some(offset_mm),
        joint: Some(d.joint),
        on: None,
        detail: format!(
            "{:.3} mm segment bends {:.0} deg into {into}, which sits {offset_mm:.4} mm off the axis of the {:.3} mm track before it",
            nm_to_mm(s.seg.length()),
            180.0 - d.interior_deg,
            nm_to_mm(l.seg.length()),
        ),
    }
}

fn sort_findings(board: &RouteBoard, f: &mut [BoardFinding]) {
    let name = |n: Option<NetId>| n.map(|n| board.net(n).name.as_str()).unwrap_or("");
    f.sort_by(|a, b| {
        kind_order(a.kind)
            .cmp(&kind_order(b.kind))
            .then_with(|| name(a.net).cmp(name(b.net)))
            .then_with(|| a.layer.cmp(&b.layer))
            .then_with(|| a.at.0.total_cmp(&b.at.0))
            .then_with(|| a.at.1.total_cmp(&b.at.1))
    });
}

fn finding_json(board: &RouteBoard, f: &BoardFinding) -> Value {
    let track = |&(i, role): &(u32, &str)| {
        let t = &board.tracks[i as usize];
        json!({
            "uuid": t.uuid, "role": role,
            "from": mm_pt(t.seg.a), "to": mm_pt(t.seg.b),
            "length_mm": len_mm(t.seg.a, t.seg.b),
            "width_mm": nm_to_mm(i64::from(t.seg.width)),
            "locked": t.ownership == Ownership::Locked,
        })
    };
    let via = |&i: &u32| {
        let v = &board.vias[i as usize];
        json!({ "uuid": v.uuid, "at": mm_pt(v.center), "locked": v.ownership == Ownership::Locked })
    };
    let mut o = json!({
        "kind": f.kind,
        "net": f.net.map(|n| board.net(n).name.clone()),
        "layer": board.layers.name(f.layer),
        "x": route_geometry::round4(f.at.0),
        "y": route_geometry::round4(f.at.1),
        "tracks": f.tracks.iter().map(track).collect::<Vec<_>>(),
        "interior_angles_deg": f.angles_deg.iter().map(|a| round2(*a)).collect::<Vec<_>>(),
        "detail": f.detail,
    });
    if !f.vias.is_empty() {
        o["vias"] = json!(f.vias.iter().map(via).collect::<Vec<_>>());
    }
    if !f.pads.is_empty() {
        o["pads"] = json!(f
            .pads
            .iter()
            .map(|&p| pad_label(board, &board.pads[p as usize]))
            .collect::<Vec<_>>());
    }
    if let Some(off) = f.offset_mm {
        o["offset_mm"] = json!(off);
    }
    if let Some(j) = f.joint {
        o["joint"] = mm_pt(j);
    }
    if let Some(on) = f.on {
        o["on"] = json!(on);
    }
    o
}

/// Whether every item of a finding is locked.
fn all_locked(board: &RouteBoard, f: &BoardFinding) -> bool {
    f.tracks
        .iter()
        .all(|(i, _)| board.tracks[*i as usize].ownership == Ownership::Locked)
        && f.vias
            .iter()
            .all(|i| board.vias[*i as usize].ownership == Ownership::Locked)
}

// ─── Linting planned copper ──────────────────────────────────────────────────

/// `lint_board_geometry`'s default dogleg length, nm.
pub(crate) const DOGLEG_DEFAULT_NM: f64 = DOGLEG_DEFAULT_MM * 1e6;

/// `board` without the tracks `copper` repeats exactly (same net, layer,
/// ends in either order, and width), track ids renumbered, and how many
/// were set aside: copper a plan re-plans that is already on the board.
pub(crate) fn without_repeated(board: &RouteBoard, copper: &RoutedCopper) -> (RouteBoard, usize) {
    let same = |t: &konnect_route::board::Track, p: &RoutedTrack| {
        t.net == Some(p.net)
            && t.layer == p.layer
            && t.seg.width == p.seg.width
            && ((t.seg.a == p.seg.a && t.seg.b == p.seg.b)
                || (t.seg.a == p.seg.b && t.seg.b == p.seg.a))
    };
    let mut b = board.clone();
    let before = b.tracks.len();
    b.tracks
        .retain(|t| !copper.tracks.iter().any(|p| same(t, p)));
    let repeated = before - b.tracks.len();
    for (i, t) in b.tracks.iter_mut().enumerate() {
        t.id = konnect_route::api::TrackId(i as u32);
    }
    (b, repeated)
}

/// The lint of planned copper as it will sit on `board` (the saved board,
/// plus any other planned copper the caller froze into it): the copper is
/// added to a copy of the board, every track of its nets on its layers is
/// linted as `lint_board_geometry` does, and only findings that involve a
/// planned track or via are kept. Tracks are named by their index in
/// `copper` (`tracks[i]`), which is their index in the routing_file
/// written from it.
pub(crate) fn lint_planned(
    board: &RouteBoard,
    copper: &RoutedCopper,
    limits: &Limits,
    dogleg_nm: f64,
    listed: usize,
) -> Value {
    // The plan's own copper already applied is linted once, as planned,
    // not as a zero-degree bend against itself.
    let (mut b, duplicates) = without_repeated(board, copper);
    let (t0, v0) = (b.tracks.len() as u32, b.vias.len() as u32);
    copper.freeze_into(&mut b);
    let nets: BTreeSet<NetId> = copper
        .tracks
        .iter()
        .map(|t| t.net)
        .chain(copper.vias.iter().map(|v| v.net))
        .collect();
    let layers = copper
        .tracks
        .iter()
        .fold(LayerMask::EMPTY, |m, t| m.with(t.layer));
    let ix = CopperIndex::new(&b);
    let scope = Scope {
        nets: Some(nets),
        layers: Some(layers),
    };
    let (all, _) = lint_board(&ix, &scope, limits, dogleg_nm);
    let mine: Vec<&BoardFinding> = all
        .iter()
        .filter(|f| f.tracks.iter().any(|(i, _)| *i >= t0) || f.vias.iter().any(|v| *v >= v0))
        .collect();
    let mut by_kind: BTreeMap<&str, usize> = KINDS.iter().map(|k| (*k, 0)).collect();
    for f in &mine {
        *by_kind.entry(f.kind).or_default() += 1;
    }
    let defects = mine.iter().filter(|f| f.kind != "collinear_vertex").count();
    let member = |i: u32| {
        if i >= t0 {
            json!(format!("tracks[{}]", i - t0))
        } else {
            json!(format!(
                "board track {}",
                b.tracks[i as usize].uuid.as_deref().unwrap_or("?")
            ))
        }
    };
    json!({
        "limits": {
            "jog_length_mm": limits.jog_length_mm,
            "dogleg_length_mm": dogleg_nm / 1e6,
            "min_bend_angle_deg": limits.min_bend_angle_deg,
        },
        "defects": defects,
        "clean": defects == 0,
        "by_kind": by_kind,
        "board_tracks_repeated_by_the_plan": duplicates,
        "findings": mine.iter().take(listed).map(|f| json!({
            "kind": f.kind,
            "x": route_geometry::round4(f.at.0),
            "y": route_geometry::round4(f.at.1),
            "tracks": f.tracks.iter().map(|(i, role)| json!({ "track": member(*i), "role": role })).collect::<Vec<_>>(),
            "pads": f.pads.iter().map(|&p| pad_label(&b, &b.pads[p as usize])).collect::<Vec<_>>(),
            "interior_angles_deg": f.angles_deg.iter().map(|a| round2(*a)).collect::<Vec<_>>(),
            "detail": f.detail,
        })).collect::<Vec<_>>(),
    })
}

// ─── lint_board_geometry ─────────────────────────────────────────────────────

pub(crate) fn lint_schema() -> Value {
    json!({
        "type": "object",
        "properties": {
            "board": { "type": "string", "description": "Saved .kicad_pcb to read (the file on disk, not the live editor)" },
            "nets": { "type": "array", "items": { "type": "string", "minLength": 1 }, "minItems": 1, "description": "Only these nets (names or * ? patterns); each must match a net of the board" },
            "layers": { "type": "array", "items": { "type": "string", "minLength": 1 }, "minItems": 1, "description": "Only findings on these copper layers" },
            "area": crate::tools::route_native::box_schema("Only findings located in this box, mm (corners in any order)"),
            "locked": { "type": "boolean", "description": "true: only findings whose items are all locked; false: only findings with an unlocked item" },
            "kinds": { "type": "array", "minItems": 1, "items": { "type": "string", "enum": KINDS }, "description": "Only these kinds (default: all)" },
            "jog_length_mm": { "type": "number", "exclusiveMinimum": 0, "maximum": 10, "default": 0.25, "description": "Segments shorter than this between two bends are jogs or hairpins" },
            "dogleg_length_mm": { "type": "number", "exclusiveMinimum": 0, "maximum": DOGLEG_MAX_MM, "default": DOGLEG_DEFAULT_MM, "description": "A last segment into a via or pad shorter than this, bent off the longer segment before it, is a dogleg" },
            "min_bend_angle_deg": { "type": "number", "minimum": 0, "maximum": 180, "default": 90, "description": "Smallest allowed interior angle between connected segments (180 is straight)" },
            "max_items": { "type": "integer", "minimum": 0, "maximum": 100000, "default": 200, "description": "Findings listed; the counts cover every finding" },
            "markers_output": { "type": "string", "description": "New JSON file to write every matched finding to as render markers ({x, y, label, kind}) for render_board_layers_png or render_board_tiles; an existing file is never replaced" }
        },
        "required": ["board"]
    })
}

pub(crate) async fn handle_lint_board_geometry(
    args: &Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    // Parsing a large board takes a second or more: off the async workers.
    let args = args.clone();
    tokio::task::spawn_blocking(move || lint_board_geometry_blocking(&args)).await?
}

fn length_mm_arg(args: &Value, key: &str, default: f64, max: f64) -> Result<f64, Refusal> {
    match args.get(key) {
        None | Some(Value::Null) => Ok(default),
        Some(x) => x
            .as_f64()
            .filter(|x| x.is_finite() && *x > 0.0 && *x <= max)
            .ok_or_else(|| invalid(key, &format!("must be above 0 and at most {max} mm"))),
    }
}

fn kinds_arg(args: &Value) -> Result<BTreeSet<&'static str>, Refusal> {
    let Some(names) = opt_strings(args, "kinds")? else {
        return Ok(KINDS.into_iter().collect());
    };
    names
        .iter()
        .map(|n| {
            KINDS
                .iter()
                .find(|k| **k == n.as_str())
                .copied()
                .ok_or_else(|| {
                    invalid(
                        "kinds",
                        &format!("'{n}' is not one of {}", KINDS.join(", ")),
                    )
                })
        })
        .collect()
}

fn locked_arg(args: &Value) -> Result<Option<bool>, Refusal> {
    match args.get("locked") {
        None | Some(Value::Null) => Ok(None),
        Some(Value::Bool(b)) => Ok(Some(*b)),
        Some(_) => Err(invalid("locked", "must be true or false")),
    }
}

fn area_arg(args: &Value) -> Result<Option<BBox>, Refusal> {
    match args.get("area") {
        None | Some(Value::Null) => Ok(None),
        Some(a) => box_arg(a, "area").map(Some),
    }
}

fn in_area(area: &Option<BBox>, at: (f64, f64)) -> bool {
    area.is_none_or(|b| Point::from_mm(at.0, at.1).is_some_and(|p| b.contains(p)))
}

fn absolute(p: PathBuf) -> anyhow::Result<PathBuf> {
    Ok(if p.is_absolute() {
        p
    } else {
        std::env::current_dir()?.join(p)
    })
}

fn refuse_existing(path: &Path) -> Option<CallToolResult> {
    path.exists().then(|| {
        CallToolResult::error_kind(
            ToolErrorKind::Conflict {
                paths: vec![path.display().to_string()],
            },
            format!(
                "{} already exists; it is never replaced. Nothing was written.",
                path.display()
            ),
        )
    })
}

fn write_new(path: &Path, bytes: &[u8]) -> Result<(), CallToolResult> {
    if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
        if let Err(e) = std::fs::create_dir_all(parent) {
            return Err(CallToolResult::error(format!(
                "{} not written: {e}",
                path.display()
            )));
        }
    }
    native_route_job::write_new_atomic(path, bytes).map_err(|e| {
        if path.exists() {
            CallToolResult::error_kind(
                ToolErrorKind::Conflict {
                    paths: vec![path.display().to_string()],
                },
                format!("{e:#}"),
            )
        } else {
            CallToolResult::error(format!("{} not written: {e:#}", path.display()))
        }
    })
}

/// The markers of `findings` (label: kind and net), as render overlays take them.
pub(crate) fn markers_json(board: &RouteBoard, findings: &[&BoardFinding]) -> Value {
    json!(findings
        .iter()
        .map(|f| {
            let net = f.net.map(|n| board.net(n).name.as_str()).unwrap_or("");
            json!({
                "x": route_geometry::round4(f.at.0),
                "y": route_geometry::round4(f.at.1),
                "label": format!("{} {net}", f.kind),
                "kind": f.kind,
            })
        })
        .collect::<Vec<_>>())
}

fn lint_board_geometry_blocking(args: &Value) -> anyhow::Result<CallToolResult> {
    let saved = match board_inspect::load_saved_board(args)? {
        Ok(s) => s,
        Err(r) => return Ok(r),
    };
    let board = &saved.board;
    let parsed = (|| -> Result<_, Refusal> {
        let scope = Scope {
            nets: nets_arg(board, args)?,
            layers: layers_arg(board, args, "layers")?,
        };
        let area = area_arg(args)?;
        let locked = locked_arg(args)?;
        let kinds = kinds_arg(args)?;
        let limits = route_geometry::parse_limits(args)
            .map_err(|(field, reason)| invalid(&field, &reason))?;
        let dogleg = length_mm_arg(args, "dogleg_length_mm", DOGLEG_DEFAULT_MM, DOGLEG_MAX_MM)?;
        let max_items = opt_count(args, "max_items", 200, 100_000)? as usize;
        let markers = opt_path(args, "markers_output")?;
        Ok((
            scope, area, locked, kinds, limits, dogleg, max_items, markers,
        ))
    })();
    let (scope, area, locked, kinds, limits, dogleg, max_items, markers) = match parsed {
        Ok(v) => v,
        Err(r) => return Ok(r),
    };
    let markers = match markers.map(absolute).transpose()? {
        Some(p) => match refuse_existing(&p) {
            Some(r) => return Ok(r),
            None => Some(p),
        },
        None => None,
    };
    let t = std::time::Instant::now();
    let ix = CopperIndex::new(board);
    let (all, stats) = lint_board(&ix, &scope, &limits, dogleg * 1e6);
    let found: Vec<&BoardFinding> = all
        .iter()
        .filter(|f| kinds.contains(f.kind))
        .filter(|f| in_area(&area, f.at))
        .filter(|f| locked.is_none_or(|want| want == all_locked(board, f)))
        .collect();

    let mut by_kind: BTreeMap<&str, usize> = kinds.iter().map(|k| (*k, 0)).collect();
    let mut by_layer: BTreeMap<String, usize> = BTreeMap::new();
    let mut by_net: BTreeMap<&str, usize> = BTreeMap::new();
    let mut cells: BTreeMap<(i64, i64), usize> = BTreeMap::new();
    for f in &found {
        *by_kind.entry(f.kind).or_default() += 1;
        *by_layer
            .entry(board.layers.name(f.layer).to_string())
            .or_default() += 1;
        let net = f.net.map(|n| board.net(n).name.as_str()).unwrap_or("");
        *by_net.entry(net).or_default() += 1;
        if f.kind != "collinear_vertex" {
            let c = (
                (f.at.0 / HOTSPOT_MM).floor() as i64,
                (f.at.1 / HOTSPOT_MM).floor() as i64,
            );
            *cells.entry(c).or_default() += 1;
        }
    }
    let mut nets: Vec<(&str, usize)> = by_net.into_iter().collect();
    nets.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(b.0)));
    let nets_with_findings = nets.len();
    nets.truncate(TOP_NETS);
    let mut hot: Vec<((i64, i64), usize)> = cells.into_iter().collect();
    hot.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
    hot.truncate(HOTSPOTS);
    let defects = found
        .iter()
        .filter(|f| f.kind != "collinear_vertex")
        .count();

    let markers_written = match &markers {
        Some(p) => {
            let text = serde_json::to_string_pretty(&markers_json(board, &found))?;
            if let Err(r) = write_new(p, text.as_bytes()) {
                return Ok(r);
            }
            Some(found.len())
        }
        None => None,
    };
    let listed: Vec<Value> = found
        .iter()
        .take(max_items)
        .map(|f| finding_json(board, f))
        .collect();
    Ok(CallToolResult::json(&json!({
        "source": "saved_file",
        "board": saved.path,
        "note": board_inspect::SAVED_FILE_NOTE,
        "limits": {
            "jog_length_mm": limits.jog_length_mm,
            "dogleg_length_mm": dogleg,
            "min_bend_angle_deg": limits.min_bend_angle_deg,
        },
        "checked": {
            "tracks": stats.tracks,
            "vias": stats.vias,
            "tracks_without_net": stats.tracks_without_net,
            "zero_length_tracks": stats.zero_length_tracks,
            "arcs_not_linted": stats.arcs_not_linted,
            "collinear_vertices_on_vias_or_pads_skipped": stats.collinear_at_via_or_pad,
        },
        "matched": found.len(),
        "defects": defects,
        "clean": defects == 0,
        "by_kind": by_kind,
        "by_layer": by_layer,
        "by_net": nets.iter().map(|(n, c)| json!({ "net": n, "count": c })).collect::<Vec<_>>(),
        "nets_with_findings": nets_with_findings,
        "hotspots": hot.iter().map(|((x, y), c)| json!({
            "x1": *x as f64 * HOTSPOT_MM, "y1": *y as f64 * HOTSPOT_MM,
            "x2": (*x + 1) as f64 * HOTSPOT_MM, "y2": (*y + 1) as f64 * HOTSPOT_MM,
            "count": c,
        })).collect::<Vec<_>>(),
        "returned": listed.len(),
        "truncated": listed.len() < found.len(),
        "findings": listed,
        "markers_output": markers.as_ref().map(|p| p.display().to_string()),
        "markers_written": markers_written,
        "elapsed_ms": t.elapsed().as_millis() as u64,
        "load_report": saved.load_report_json(),
        "next": "plan_dogleg_fixes plans the via doglegs away; render_board_tiles with markers_file shows every finding.",
    })))
}

// ─── Fixing via doglegs ──────────────────────────────────────────────────────

/// A track that stays, re-ended at the via's new position.
#[derive(Debug, Clone, Copy)]
struct Kept {
    track: u32,
    /// Which end moves: the track's `a` end, or its `b` end.
    at_a: bool,
    /// The end that stays.
    far: Point,
    /// Where the moving end is now.
    old: Point,
}

impl Kept {
    fn line(&self) -> Line {
        Line::through(self.far, self.old)
    }
}

/// The geometric fix of one via: where it goes and which ends move.
#[derive(Debug, Clone)]
pub(crate) struct ViaFix {
    pub via: u32,
    pub new_center: Point,
    /// "intersection" of the axes, or "projection" onto the one axis there is.
    pub basis: &'static str,
    /// Indices into the dogleg list.
    pub doglegs: Vec<usize>,
    /// Short segments, deleted outright.
    pub shorts: Vec<u32>,
    /// (track, `a` end?, new point) for every track re-ended.
    pub moves: Vec<(u32, bool, Point)>,
}

/// Why a dogleg is left alone.
#[derive(Debug, Clone)]
pub(crate) struct NotFixable {
    pub doglegs: Vec<usize>,
    pub reason: &'static str,
    pub detail: Value,
}

fn no_fix(reason: &'static str, detail: String) -> (&'static str, String) {
    (reason, detail)
}

fn uuid_or(board: &RouteBoard, t: u32) -> String {
    board.tracks[t as usize]
        .uuid
        .clone()
        .unwrap_or_else(|| format!("track #{t}"))
}

/// Plan the fix of via `vi` given every dogleg into it.
fn fix_via(
    ix: &CopperIndex,
    vi: u32,
    doglegs: &[(usize, &Dogleg)],
) -> Result<ViaFix, (&'static str, String)> {
    let board = ix.board;
    let via = &board.vias[vi as usize];
    let c = via.center;
    if via.layers != board.layers.all() {
        return Err(no_fix(
            "not_through_via",
            "the via does not span every copper layer; add_routing makes through vias only"
                .to_string(),
        ));
    }
    if via.uuid.is_none() {
        return Err(no_fix(
            "no_uuid",
            "the via has no UUID in the file".to_string(),
        ));
    }
    let shape = via.shape();
    let shorts: Vec<u32> = doglegs.iter().map(|(_, d)| d.short).collect();
    let mut kept: Vec<Kept> = Vec::new();
    // The long segment of each dogleg first: the first is the reference axis.
    for (_, d) in doglegs {
        let l = &board.tracks[d.long as usize];
        if near(d.far, c, JOIN_NM) {
            return Err(no_fix(
                "loop",
                format!(
                    "track {} ends on the via at both ends",
                    uuid_or(board, d.long)
                ),
            ));
        }
        kept.push(Kept {
            track: d.long,
            at_a: near(l.seg.a, d.joint, JOIN_NM),
            far: d.far,
            old: d.joint,
        });
    }
    for layer in via.layers.iter() {
        let ends = ix.ends_at(c, layer, via.net);
        for e in &ends {
            match e.item {
                EndItem::Arc(a) => {
                    return Err(no_fix(
                        "arc_on_via",
                        format!(
                            "an arc ({}) ends on the via on {}",
                            board.arcs[a as usize].uuid.as_deref().unwrap_or("?"),
                            board.layers.name(layer)
                        ),
                    ))
                }
                EndItem::Track(t) if shorts.contains(&t) => {}
                EndItem::Track(t) => {
                    if kept.iter().any(|k| k.track == t) {
                        return Err(no_fix(
                            "loop",
                            format!("track {} meets the via twice", uuid_or(board, t)),
                        ));
                    }
                    let seg = board.tracks[t as usize].seg;
                    let far = if e.at_a { seg.b } else { seg.a };
                    if near(far, c, JOIN_NM) {
                        return Err(no_fix(
                            "loop",
                            format!("track {} has both ends on the via", uuid_or(board, t)),
                        ));
                    }
                    kept.push(Kept {
                        track: t,
                        at_a: e.at_a,
                        far,
                        old: e.p,
                    });
                }
            }
        }
    }
    // Copper of the via's net touching it anywhere but at its centre, other
    // than the tracks the fix already moves or deletes (a dogleg's longer
    // segment usually reaches the via's copper).
    let accounted: BTreeSet<u32> = shorts
        .iter()
        .copied()
        .chain(kept.iter().map(|k| k.track))
        .collect();
    for layer in via.layers.iter() {
        for t in ix.tracks_touching(&shape, layer) {
            let tr = &board.tracks[t as usize];
            if tr.net == via.net && !accounted.contains(&t) {
                return Err(no_fix(
                    "track_off_centre",
                    format!(
                        "track {} on {} touches the via without ending on its centre",
                        uuid_or(board, t),
                        board.layers.name(layer)
                    ),
                ));
            }
        }
        if let Some(a) = ix
            .arcs_touching(&shape, layer)
            .into_iter()
            .find(|&a| board.arcs[a as usize].net == via.net)
        {
            return Err(no_fix(
                "arc_on_via",
                format!(
                    "an arc ({}) touches the via on {}",
                    board.arcs[a as usize].uuid.as_deref().unwrap_or("?"),
                    board.layers.name(layer)
                ),
            ));
        }
    }
    if let Some(&p) = ix.pads_touching(&shape, via.layers).first() {
        return Err(no_fix(
            "via_in_pad",
            format!(
                "the via touches pad {}",
                pad_label(board, &board.pads[p as usize])
            ),
        ));
    }
    for t in shorts.iter().chain(kept.iter().map(|k| &k.track)) {
        if board.tracks[*t as usize].uuid.is_none() {
            return Err(no_fix(
                "no_uuid",
                format!("track #{t} has no UUID in the file"),
            ));
        }
    }

    // Where the axes meet.
    let lines: Vec<Line> = kept.iter().map(Kept::line).collect();
    let (p, basis) = match (1..lines.len()).find_map(|i| lines[0].meet(&lines[i])) {
        Some(p) => (p, "intersection"),
        None if lines.len() == 1 => (lines[0].project(v(c)), "projection"),
        None => {
            return Err(no_fix(
                "parallel_axes",
                format!(
                    "the axes of the tracks on the via are parallel to that of track {}, so they meet nowhere",
                    uuid_or(board, kept[0].track)
                ),
            ))
        }
    };
    for k in &kept {
        let off = k.line().distance(p);
        if off > AXIS_TOL_NM {
            return Err(no_fix(
                "inconsistent_axes",
                format!(
                    "the axis of track {} on {} passes {:.4} mm from the point ({:.4}, {:.4}) where the other axes meet",
                    uuid_or(board, k.track),
                    board.layers.name(board.tracks[k.track as usize].layer),
                    off / 1e6,
                    p.0 / 1e6,
                    p.1 / 1e6
                ),
            ));
        }
    }
    let (px, py) = (p.0.round(), p.1.round());
    let in_range = |x: f64| x.abs() <= f64::from(konnect_geom::MAX_COORD);
    if !in_range(px) || !in_range(py) {
        return Err(no_fix(
            "out_of_range",
            "the axes meet outside the representable board".to_string(),
        ));
    }
    let np = Point::new(px as i32, py as i32);
    for k in &kept {
        let (ox, oy) = (
            i64::from(k.old.x) - i64::from(k.far.x),
            i64::from(k.old.y) - i64::from(k.far.y),
        );
        let (nx, ny) = (
            i64::from(np.x) - i64::from(k.far.x),
            i64::from(np.y) - i64::from(k.far.y),
        );
        let layer = board.layers.name(board.tracks[k.track as usize].layer);
        if nx == 0 && ny == 0 || (ox as f64) * (nx as f64) + (oy as f64) * (ny as f64) <= 0.0 {
            return Err(no_fix(
                "segment_reverses",
                format!(
                    "the new via position ({:.4}, {:.4}) lies at or beyond the far end of track {} on {layer}, which would vanish or turn round",
                    px / 1e6,
                    py / 1e6,
                    uuid_or(board, k.track)
                ),
            ));
        }
        if octilinear(ox, oy) && !octilinear(nx, ny) {
            return Err(no_fix(
                "off_grid",
                format!(
                    "the axes meet at ({:.7}, {:.7}) mm, between nanometre grid points; track {} on {layer} would no longer run at a multiple of 45 degrees",
                    p.0 / 1e6,
                    p.1 / 1e6,
                    uuid_or(board, k.track)
                ),
            ));
        }
    }
    let mut ds: Vec<usize> = doglegs.iter().map(|(i, _)| *i).collect();
    ds.sort_unstable();
    Ok(ViaFix {
        via: vi,
        new_center: np,
        basis,
        doglegs: ds,
        shorts,
        moves: kept.iter().map(|k| (k.track, k.at_a, np)).collect(),
    })
}

/// Group fixes that share a track: such a group is accepted or refused
/// whole. Clusters are listed by their first fix.
fn clusters(fixes: &[ViaFix]) -> Vec<Vec<usize>> {
    let mut parent: Vec<usize> = (0..fixes.len()).collect();
    fn root(parent: &mut [usize], mut i: usize) -> usize {
        while parent[i] != i {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        i
    }
    let mut owner: BTreeMap<u32, usize> = BTreeMap::new();
    for (i, f) in fixes.iter().enumerate() {
        for t in f.shorts.iter().chain(f.moves.iter().map(|m| &m.0)) {
            match owner.get(t) {
                Some(&j) => {
                    let (a, b) = (root(&mut parent, i), root(&mut parent, j));
                    if a != b {
                        parent[a.max(b)] = a.min(b);
                    }
                }
                None => {
                    owner.insert(*t, i);
                }
            }
        }
    }
    let mut groups: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for i in 0..fixes.len() {
        let r = root(&mut parent, i);
        groups.entry(r).or_default().push(i);
    }
    groups.into_values().collect()
}

/// The copper of a set of accepted clusters: what is deleted, what is added
/// and who owns each added and each deleted item.
struct Assembly {
    deleted_tracks: BTreeSet<u32>,
    deleted_vias: BTreeSet<u32>,
    new: RoutedCopper,
    new_owner: (Vec<usize>, Vec<usize>),
    old: RoutedCopper,
    old_owner: (Vec<usize>, Vec<usize>),
    /// Per fix: the new tracks it adds (indices into `new.tracks`) and its
    /// via (index into `new.vias`).
    added: BTreeMap<usize, (Vec<usize>, usize)>,
}

/// Build the replacement copper of `accepted` clusters. A cluster whose
/// merged moves would reverse or collapse a track is returned as a conflict.
fn assemble(
    board: &RouteBoard,
    fixes: &[ViaFix],
    groups: &[Vec<usize>],
    accepted: &BTreeSet<usize>,
) -> (Assembly, Vec<(usize, String)>) {
    let mut conflicts = Vec::new();
    let mut asm = Assembly {
        deleted_tracks: BTreeSet::new(),
        deleted_vias: BTreeSet::new(),
        new: RoutedCopper::default(),
        new_owner: (Vec::new(), Vec::new()),
        old: RoutedCopper::default(),
        old_owner: (Vec::new(), Vec::new()),
        added: BTreeMap::new(),
    };
    let all = board.layers.all();
    for &g in accepted {
        // Merge the moves of the cluster per track.
        let mut ends: BTreeMap<u32, (Point, Point)> = BTreeMap::new();
        let mut first_mover: BTreeMap<u32, usize> = BTreeMap::new();
        for &fi in &groups[g] {
            for &(t, at_a, p) in &fixes[fi].moves {
                let seg = board.tracks[t as usize].seg;
                let e = ends.entry(t).or_insert((seg.a, seg.b));
                if at_a {
                    e.0 = p;
                } else {
                    e.1 = p;
                }
                first_mover.entry(t).or_insert(fi);
            }
        }
        let mut bad = None;
        for (&t, &(a, b)) in &ends {
            let seg = board.tracks[t as usize].seg;
            let o = sub(v(seg.b), v(seg.a));
            let n = sub(v(b), v(a));
            if a == b || dot(o, n) <= 0.0 {
                bad = Some(format!(
                    "the moves of this group would reverse or collapse track {}",
                    uuid_or(board, t)
                ));
            }
        }
        if let Some(why) = bad {
            conflicts.push((g, why));
            continue;
        }
        for &fi in &groups[g] {
            let f = &fixes[fi];
            let via = &board.vias[f.via as usize];
            asm.deleted_vias.insert(f.via);
            asm.old.vias.push(RoutedVia {
                net: via.net.expect("a dogleg via has a net"),
                center: via.center,
                diameter: via.diameter,
                drill: via.drill,
                layers: via.layers,
                gen: Generation(0),
            });
            asm.old_owner.1.push(g);
            let mut tracks = Vec::new();
            for t in f.shorts.iter().chain(f.moves.iter().map(|m| &m.0)) {
                if !asm.deleted_tracks.insert(*t) {
                    continue;
                }
                let tr = &board.tracks[*t as usize];
                asm.old.tracks.push(RoutedTrack {
                    net: tr.net.expect("a dogleg track has a net"),
                    layer: tr.layer,
                    seg: tr.seg,
                    gen: Generation(0),
                });
                asm.old_owner.0.push(g);
                if let Some(&(a, b)) = ends.get(t) {
                    if first_mover.get(t) == Some(&fi) {
                        tracks.push(asm.new.tracks.len());
                        asm.new.tracks.push(RoutedTrack {
                            net: tr.net.expect("a dogleg track has a net"),
                            layer: tr.layer,
                            seg: konnect_geom::Segment::new(a, b, tr.seg.width),
                            gen: Generation(0),
                        });
                        asm.new_owner.0.push(g);
                    }
                }
            }
            let vi = asm.new.vias.len();
            asm.new.vias.push(RoutedVia {
                net: via.net.expect("a dogleg via has a net"),
                center: f.new_center,
                diameter: via.diameter,
                drill: via.drill,
                layers: all,
                gen: Generation(0),
            });
            asm.new_owner.1.push(g);
            asm.added.insert(fi, (tracks, vi));
        }
    }
    (asm, conflicts)
}

/// `board` without the deleted items, ids renumbered; with the original
/// index of every track and via kept.
fn without(
    board: &RouteBoard,
    tracks: &BTreeSet<u32>,
    vias: &BTreeSet<u32>,
) -> (RouteBoard, Vec<u32>, Vec<u32>) {
    let mut b = board.clone();
    let mut track_map = Vec::new();
    let mut kept_tracks = Vec::new();
    for (i, t) in std::mem::take(&mut b.tracks).into_iter().enumerate() {
        if !tracks.contains(&(i as u32)) {
            let mut t = t;
            t.id = konnect_route::api::TrackId(kept_tracks.len() as u32);
            kept_tracks.push(t);
            track_map.push(i as u32);
        }
    }
    b.tracks = kept_tracks;
    let mut via_map = Vec::new();
    let mut kept_vias = Vec::new();
    for (i, x) in std::mem::take(&mut b.vias).into_iter().enumerate() {
        if !vias.contains(&(i as u32)) {
            let mut x = x;
            x.id = konnect_route::api::ViaId(kept_vias.len() as u32);
            kept_vias.push(x);
            via_map.push(i as u32);
        }
    }
    b.vias = kept_vias;
    (b, track_map, via_map)
}

/// One side of a violation, for comparing a fix's findings with those its
/// old copper already had.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
enum SideKey {
    Pad(u32),
    Track(u32),
    Arc(u32),
    Via(u32),
    Zone(u32),
    Edge,
    Group(usize),
}

fn side_key(item: &CheckedItem, owner: &(Vec<usize>, Vec<usize>)) -> SideKey {
    match *item {
        CheckedItem::Board { item } => match item {
            ItemRef::Pad(i) => SideKey::Pad(i.0),
            ItemRef::Track(i) => SideKey::Track(i.0),
            ItemRef::Arc(i) => SideKey::Arc(i.0),
            ItemRef::Via(i) => SideKey::Via(i.0),
            ItemRef::Zone(i) => SideKey::Zone(i.0),
        },
        CheckedItem::RoutedTrack { index } => SideKey::Group(owner.0[index as usize]),
        CheckedItem::RoutedVia { index } => SideKey::Group(owner.1[index as usize]),
        CheckedItem::Edge => SideKey::Edge,
    }
}

type ViolationKey = (ViolationKind, SideKey, Option<SideKey>);

fn violation_key(
    v: &konnect_route::check::Violation,
    owner: &(Vec<usize>, Vec<usize>),
) -> ViolationKey {
    let a = side_key(&v.a, owner);
    let b = v.b.as_ref().map(|b| side_key(b, owner));
    match b {
        Some(b) if b < a => (v.kind, b, Some(a)),
        _ => (v.kind, a, b),
    }
}

fn kind_name(k: ViolationKind) -> String {
    serde_json::to_value(k)
        .ok()
        .and_then(|v| v.as_str().map(String::from))
        .unwrap_or_default()
}

/// What a violation's other side is, in words.
fn describe(
    board: &RouteBoard,
    reduced: &RouteBoard,
    maps: &(Vec<u32>, Vec<u32>),
    side: SideKey,
    me: usize,
) -> String {
    match side {
        SideKey::Pad(i) => format!("pad {}", pad_label(reduced, &reduced.pads[i as usize])),
        SideKey::Track(i) => {
            let t = &board.tracks[maps.0[i as usize] as usize];
            format!(
                "track {} ({})",
                t.uuid.as_deref().unwrap_or("?"),
                t.net
                    .map(|n| board.net(n).name.as_str())
                    .unwrap_or("no net")
            )
        }
        SideKey::Arc(i) => format!(
            "arc {}",
            reduced.arcs[i as usize].uuid.as_deref().unwrap_or("?")
        ),
        SideKey::Via(i) => {
            let x = &board.vias[maps.1[i as usize] as usize];
            format!(
                "via {} ({})",
                x.uuid.as_deref().unwrap_or("?"),
                x.net
                    .map(|n| board.net(n).name.as_str())
                    .unwrap_or("no net")
            )
        }
        SideKey::Zone(i) => format!(
            "zone {}",
            reduced.zones[i as usize]
                .zone_name
                .as_deref()
                .unwrap_or("(unnamed)")
        ),
        SideKey::Edge => "the board edge".to_string(),
        SideKey::Group(g) if g == me => "its own new copper".to_string(),
        SideKey::Group(_) => "the new copper of another fix".to_string(),
    }
}

/// The outcome of checking the fixes as one set.
pub(crate) struct CheckedFixes {
    pub fixes: Vec<ViaFix>,
    pub groups: Vec<Vec<usize>>,
    pub accepted: BTreeSet<usize>,
    /// Refused groups with the reason and its detail.
    pub refused: BTreeMap<usize, (&'static str, Vec<Value>)>,
    /// Violations the old copper of an accepted group already had.
    pub preexisting: BTreeMap<usize, usize>,
    /// Groups that collided with another fix, set aside and retried alone
    /// against the accepted set.
    pub retried: usize,
    pub rounds: usize,
    pub final_violations: usize,
    pub assembly: Option<AssemblyOut>,
}

/// The accepted copper, ready to write.
pub(crate) struct AssemblyOut {
    pub new: RoutedCopper,
    pub added: BTreeMap<usize, (Vec<usize>, usize)>,
}

/// One check of a set of groups.
struct Round {
    asm: Assembly,
    /// Groups whose merged moves are inconsistent (never checked).
    conflicts: Vec<(usize, String)>,
    /// New violations against the board, the edge, or a group's own copper.
    hard: BTreeMap<usize, Vec<Value>>,
    /// New violations between two groups' new copper: (a, b, a's record,
    /// b's record).
    pairs: Vec<(usize, usize, Value, Value)>,
    preexisting: BTreeMap<usize, usize>,
    total: usize,
}

/// Check the new copper of `accepted` against the board without the items
/// those groups delete, with their old copper as the baseline.
fn check_round(
    board: &RouteBoard,
    rules: &dyn konnect_route::RuleTable,
    fixes: &[ViaFix],
    groups: &[Vec<usize>],
    accepted: &BTreeSet<usize>,
) -> Result<Round, CallToolResult> {
    let (asm, conflicts) = assemble(board, fixes, groups, accepted);
    let mut round = Round {
        asm,
        conflicts,
        hard: BTreeMap::new(),
        pairs: Vec::new(),
        preexisting: BTreeMap::new(),
        total: 0,
    };
    if !round.conflicts.is_empty() || accepted.is_empty() {
        return Ok(round);
    }
    let asm = &round.asm;
    let (reduced, tmap, vmap) = without(board, &asm.deleted_tracks, &asm.deleted_vias);
    let maps = (tmap, vmap);
    let checked = guarded("check", || {
        let checker = BoardChecker::new(&reduced, rules, CheckOptions::for_routed());
        (
            checker.check_routed(&asm.new),
            checker.check_routed(&asm.old),
        )
    });
    let (new_r, old_r) = checked.map_err(|e| stage_failure("check_routed", e))?;
    let before: BTreeSet<ViolationKey> = old_r
        .violations
        .iter()
        .map(|v| violation_key(v, &asm.old_owner))
        .collect();
    // Where each group's via goes, to name the other fix of a collision.
    let label = |g: usize| {
        let f = &fixes[groups[g][0]];
        let (x, y) = f.new_center.to_mm();
        format!(
            "the new copper of the fix moving via {} to ({x}, {y})",
            board.vias[f.via as usize].uuid.as_deref().unwrap_or("?")
        )
    };
    round.total = new_r.violations.len();
    let mut hard: BTreeMap<usize, Vec<Value>> = BTreeMap::new();
    let mut pairs = Vec::new();
    let mut preexisting: BTreeMap<usize, usize> = BTreeMap::new();
    for x in &new_r.violations {
        let key = violation_key(x, &asm.new_owner);
        let sides: Vec<SideKey> = [Some(key.1), key.2].into_iter().flatten().collect();
        let mine: Vec<usize> = sides
            .iter()
            .filter_map(|s| match s {
                SideKey::Group(g) => Some(*g),
                _ => None,
            })
            .collect();
        if before.contains(&key) {
            for g in mine {
                *preexisting.entry(g).or_default() += 1;
            }
            continue;
        }
        let (x_mm, y_mm) = x.at.to_mm();
        let record = |against: Option<String>| {
            json!({
                "kind": kind_name(x.kind),
                "at": [x_mm, y_mm],
                "layer": x.layer.map(|l| board.layers.name(l).to_string()),
                "required_mm": nm_to_mm(i64::from(x.required)),
                "against": against,
            })
        };
        match mine.as_slice() {
            [a, b] if a != b => {
                pairs.push((*a, *b, record(Some(label(*b))), record(Some(label(*a)))));
            }
            _ => {
                for &g in &mine {
                    let other = sides.iter().copied().find(|s| *s != SideKey::Group(g));
                    let against = match other {
                        Some(o) => describe(board, &reduced, &maps, o, g),
                        None => "its own new copper".to_string(),
                    };
                    hard.entry(g).or_default().push(record(Some(against)));
                }
            }
        }
    }
    round.hard = hard;
    round.pairs = pairs;
    round.preexisting = preexisting;
    Ok(round)
}

/// Check `fixes` with the exact kernel against `board` without the items
/// they delete, as one consistent set.
///
/// A group whose new copper has a violation its old copper did not have
/// against the board is refused; its items go back on the board and the rest
/// is checked again. Two groups whose new copper collides only with each
/// other are not both thrown away: the later is set aside, the rest settles,
/// and each group set aside is then tried alone against the accepted set,
/// accepted when it adds nothing new and refused, naming the fix it collides
/// with, otherwise.
pub(crate) fn check_fixes(
    board: &RouteBoard,
    rules: &dyn konnect_route::RuleTable,
    fixes: Vec<ViaFix>,
) -> Result<CheckedFixes, CallToolResult> {
    let groups = clusters(&fixes);
    let mut accepted: BTreeSet<usize> = (0..groups.len()).collect();
    let mut refused: BTreeMap<usize, (&'static str, Vec<Value>)> = BTreeMap::new();
    let mut deferred: BTreeSet<usize> = BTreeSet::new();
    let mut rounds = 0;
    let mut last = loop {
        rounds += 1;
        let round = check_round(board, rules, &fixes, &groups, &accepted)?;
        if !round.conflicts.is_empty() {
            for (g, why) in round.conflicts {
                accepted.remove(&g);
                refused.insert(g, ("conflict", vec![json!(why)]));
            }
            continue;
        }
        if !round.hard.is_empty() {
            for (g, v) in round.hard {
                accepted.remove(&g);
                refused.insert(g, ("check_failed", v));
            }
            continue;
        }
        if !round.pairs.is_empty() {
            for (a, b, _, _) in &round.pairs {
                let later = (*a).max(*b);
                if accepted.remove(&later) {
                    deferred.insert(later);
                }
            }
            continue;
        }
        break round;
    };
    let retried = deferred.len();
    for g in deferred {
        let mut trial = accepted.clone();
        trial.insert(g);
        rounds += 1;
        let round = check_round(board, rules, &fixes, &groups, &trial)?;
        let mut why: Vec<Value> = round.hard.get(&g).cloned().unwrap_or_default();
        for (a, b, ra, rb) in &round.pairs {
            if *a == g {
                why.push(ra.clone());
            } else if *b == g {
                why.push(rb.clone());
            }
        }
        let others_hit = round.hard.keys().any(|k| *k != g)
            || round.pairs.iter().any(|(a, b, _, _)| *a != g && *b != g);
        if !round.conflicts.is_empty() {
            refused.insert(
                g,
                (
                    "conflict",
                    round.conflicts.iter().map(|(_, w)| json!(w)).collect(),
                ),
            );
        } else if !why.is_empty() || others_hit {
            refused.insert(g, ("check_failed", why));
        } else {
            accepted = trial;
            last = round;
        }
    }
    let assembly = (!accepted.is_empty()).then_some(AssemblyOut {
        new: last.asm.new,
        added: last.asm.added,
    });
    Ok(CheckedFixes {
        fixes,
        groups,
        accepted,
        refused,
        preexisting: last.preexisting,
        retried,
        rounds,
        final_violations: last.total,
        assembly,
    })
}

// ─── plan_dogleg_fixes ───────────────────────────────────────────────────────

pub(crate) fn plan_schema() -> Value {
    json!({
        "type": "object",
        "properties": {
            "board": { "type": "string", "description": "Saved .kicad_pcb to read (the file on disk, not the live editor)" },
            "project": { "type": "string", "description": ".kicad_pro whose net classes and rules apply; defaults to the one beside the board" },
            "rules": { "type": "string", "description": ".kicad_dru; defaults to the one beside the project" },
            "nets": { "type": "array", "items": { "type": "string", "minLength": 1 }, "minItems": 1, "description": "Only doglegs of these nets (names or * ? patterns)" },
            "layers": { "type": "array", "items": { "type": "string", "minLength": 1 }, "minItems": 1, "description": "Only vias with a dogleg on one of these copper layers (the via is fixed with all of its doglegs)" },
            "area": crate::tools::route_native::box_schema("Only doglegs whose via or pad lies in this box, mm"),
            "dogleg_length_mm": { "type": "number", "exclusiveMinimum": 0, "maximum": DOGLEG_MAX_MM, "default": DOGLEG_DEFAULT_MM, "description": "A last segment shorter than this, bent off the longer segment before it, is a dogleg" },
            "output_plan": { "type": "string", "description": "New routing_file (tracks and vias to add) to write; an existing file is never replaced" },
            "output_fixes": { "type": "string", "description": "New JSON file with delete_uuids and a record per fix (default: output_plan with .fixes.json); an existing file is never replaced" },
            "max_items": { "type": "integer", "minimum": 0, "maximum": 100000, "default": 100, "description": "Fixes and refusals listed in the reply; the files hold them all" }
        },
        "required": ["board", "output_plan"]
    })
}

pub(crate) async fn handle_plan_dogleg_fixes(
    args: &Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let args = args.clone();
    tokio::task::spawn_blocking(move || plan_dogleg_fixes_blocking(&args)).await?
}

fn default_fixes_path(plan: &Path) -> PathBuf {
    let stem = plan
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_else(|| "plan".to_string());
    plan.with_file_name(format!("{stem}.fixes.json"))
}

fn dogleg_json(board: &RouteBoard, d: &Dogleg) -> Value {
    let s = &board.tracks[d.short as usize];
    let l = &board.tracks[d.long as usize];
    json!({
        "layer": board.layers.name(d.layer),
        "short_uuid": s.uuid,
        "short_length_mm": len_mm(s.seg.a, s.seg.b),
        "long_uuid": l.uuid,
        "long_length_mm": len_mm(l.seg.a, l.seg.b),
        "interior_angle_deg": round2(d.interior_deg),
        "offset_mm": route_geometry::round4(d.offset_nm / 1e6),
    })
}

fn plan_dogleg_fixes_blocking(args: &Value) -> anyhow::Result<CallToolResult> {
    let output = absolute(get_path(args, "output_plan")?)?;
    let spec = match base_spec(args, output.clone())? {
        Ok(s) => s,
        Err(r) => return Ok(r),
    };
    let fixes_path = match opt_path(args, "output_fixes") {
        Ok(Some(p)) => absolute(p)?,
        Ok(None) => default_fixes_path(&output),
        Err(r) => return Ok(r),
    };
    if fixes_path == output {
        return Ok(invalid("output_fixes", "must differ from output_plan"));
    }
    for p in [&output, &fixes_path] {
        if let Some(r) = refuse_existing(p) {
            return Ok(r);
        }
    }
    let (board, rules) = match loaded(&spec) {
        Ok(v) => v,
        Err(r) => return Ok(r),
    };
    let parsed = (|| -> Result<_, Refusal> {
        let nets = nets_arg(&board, args)?;
        let layers = layers_arg(&board, args, "layers")?;
        let area = area_arg(args)?;
        let dogleg = length_mm_arg(args, "dogleg_length_mm", DOGLEG_DEFAULT_MM, DOGLEG_MAX_MM)?;
        let max_items = opt_count(args, "max_items", 100, 100_000)? as usize;
        Ok((nets, layers, area, dogleg, max_items))
    })();
    let (nets, layers, area, dogleg, max_items) = match parsed {
        Ok(v) => v,
        Err(r) => return Ok(r),
    };
    let t = std::time::Instant::now();
    let ix = CopperIndex::new(&board);
    // Every layer is searched, so a chosen via is fixed with all its doglegs.
    let doglegs = find_doglegs(&ix, &Scope { nets, layers: None }, dogleg * 1e6);
    let chosen =
        |d: &Dogleg| layers.is_none_or(|m| m.contains(d.layer)) && in_area(&area, d.at.to_mm());
    let mut by_via: BTreeMap<u32, Vec<(usize, &Dogleg)>> = BTreeMap::new();
    let mut not_fixable: Vec<NotFixable> = Vec::new();
    for (i, d) in doglegs.iter().enumerate() {
        match d.end {
            DoglegEnd::Via(v) => by_via.entry(v).or_default().push((i, d)),
            DoglegEnd::Pad(p) if chosen(d) => not_fixable.push(NotFixable {
                doglegs: vec![i],
                reason: "ends_on_pad",
                detail: json!(format!(
                    "the short segment runs into pad {}, not a via; a pad is not moved",
                    pad_label(&board, &board.pads[p as usize])
                )),
            }),
            DoglegEnd::Pad(_) => {}
        }
    }
    by_via.retain(|_, ds| ds.iter().any(|(_, d)| chosen(d)));
    let mut fixes = Vec::new();
    for (vi, ds) in &by_via {
        match fix_via(&ix, *vi, ds) {
            Ok(f) => fixes.push(f),
            Err((reason, detail)) => not_fixable.push(NotFixable {
                doglegs: ds.iter().map(|(i, _)| *i).collect(),
                reason,
                detail: json!(detail),
            }),
        }
    }
    let geometric = fixes.len();
    let checked = match check_fixes(&board, &rules, fixes) {
        Ok(c) => c,
        Err(r) => return Ok(r),
    };
    // Refused groups become not-fixable records, one per fix.
    for (g, (reason, why)) in &checked.refused {
        for &fi in &checked.groups[*g] {
            not_fixable.push(NotFixable {
                doglegs: checked.fixes[fi].doglegs.clone(),
                reason,
                detail: json!(why),
            });
        }
    }
    not_fixable.sort_by_key(|n| n.doglegs.first().copied());

    // The accepted fixes, in via order.
    let mut accepted_fixes: Vec<usize> = checked
        .accepted
        .iter()
        .flat_map(|g| checked.groups[*g].iter().copied())
        .collect();
    accepted_fixes.sort_unstable();
    let group_of = |fi: usize| checked.groups.iter().position(|g| g.contains(&fi));
    let mut records = Vec::new();
    let mut delete_uuids: Vec<String> = Vec::new();
    let mut locked = (0usize, 0usize);
    let mut seen_tracks = BTreeSet::new();
    let routing = match &checked.assembly {
        Some(asm) => {
            for &fi in &accepted_fixes {
                let f = &checked.fixes[fi];
                let via = &board.vias[f.via as usize];
                let mut deleted = vec![json!({
                    "kind": "via", "uuid": via.uuid, "at": mm_pt(via.center),
                    "locked": via.ownership == Ownership::Locked,
                })];
                delete_uuids.push(via.uuid.clone().unwrap_or_default());
                let count_lock = |o: Ownership, l: &mut (usize, usize)| {
                    if o == Ownership::Locked {
                        l.0 += 1
                    } else {
                        l.1 += 1
                    }
                };
                count_lock(via.ownership, &mut locked);
                for t in f.shorts.iter().chain(f.moves.iter().map(|m| &m.0)) {
                    let tr = &board.tracks[*t as usize];
                    deleted.push(json!({
                        "kind": "track", "uuid": tr.uuid,
                        "layer": board.layers.name(tr.layer),
                        "from": mm_pt(tr.seg.a), "to": mm_pt(tr.seg.b),
                        "locked": tr.ownership == Ownership::Locked,
                    }));
                    if seen_tracks.insert(*t) {
                        delete_uuids.push(tr.uuid.clone().unwrap_or_default());
                        count_lock(tr.ownership, &mut locked);
                    }
                }
                let (tracks, vi) = &asm.added[&fi];
                let nv = &asm.new.vias[*vi];
                let added_tracks: Vec<Value> = tracks
                    .iter()
                    .map(|&k| {
                        let x = &asm.new.tracks[k];
                        json!({
                            "layer": board.layers.name(x.layer),
                            "from": mm_pt(x.seg.a), "to": mm_pt(x.seg.b),
                            "width_mm": nm_to_mm(i64::from(x.seg.width)),
                            "length_mm": len_mm(x.seg.a, x.seg.b),
                        })
                    })
                    .collect();
                let g = group_of(fi);
                records.push(json!({
                    "net": via.net.map(|n| board.net(n).name.clone()),
                    "via_uuid": via.uuid,
                    "old": mm_pt(via.center),
                    "new": mm_pt(f.new_center),
                    "moved_mm": len_mm(via.center, f.new_center),
                    "basis": f.basis,
                    "doglegs": f.doglegs.iter().map(|&i| dogleg_json(&board, &doglegs[i])).collect::<Vec<_>>(),
                    "deleted": deleted,
                    "added": {
                        "via": { "at": mm_pt(nv.center), "diameter_mm": nm_to_mm(i64::from(nv.diameter)), "drill_mm": nm_to_mm(i64::from(nv.drill)) },
                        "tracks": added_tracks,
                    },
                    "group_size": g.map(|g| checked.groups[g].len()),
                    "check": {
                        "result": "passed",
                        "preexisting_violations": g.and_then(|g| checked.preexisting.get(&g)).copied().unwrap_or(0),
                    },
                }));
            }
            match RoutePlan::from_routed(&board, &asm.new) {
                Ok(p) => Some(p),
                Err(e) => return Ok(CallToolResult::error(format!("plan not built: {e}"))),
            }
        }
        None => None,
    };
    let refusals: Vec<Value> = not_fixable
        .iter()
        .map(|n| {
            let d = &doglegs[n.doglegs[0]];
            let (end, at) = match d.end {
                DoglegEnd::Via(v) => {
                    let x = &board.vias[v as usize];
                    (json!({ "via_uuid": x.uuid }), x.center)
                }
                DoglegEnd::Pad(p) => (
                    json!({ "pad": pad_label(&board, &board.pads[p as usize]) }),
                    d.at,
                ),
            };
            json!({
                "net": d.net.map(|x| board.net(x).name.clone()),
                "at": mm_pt(at),
                "end": end,
                "reason": n.reason,
                "detail": n.detail,
                "doglegs": n.doglegs.iter().map(|&i| dogleg_json(&board, &doglegs[i])).collect::<Vec<_>>(),
            })
        })
        .collect();
    let mut by_reason: BTreeMap<&str, usize> = BTreeMap::new();
    for n in &not_fixable {
        *by_reason.entry(n.reason).or_default() += 1;
    }
    let in_scope = by_via.values().map(Vec::len).sum::<usize>()
        + not_fixable
            .iter()
            .filter(|n| n.reason == "ends_on_pad")
            .count();
    let written = routing.as_ref().is_some_and(|r| r.item_count() > 0);
    if let Some(plan) = routing.as_ref().filter(|_| written) {
        if let Err(r) = write_new(&output, plan.to_routing_file_json().as_bytes()) {
            return Ok(r);
        }
        let fixes_file = json!({
            "board": spec.board,
            "routing_file": output,
            "delete_uuids": delete_uuids,
            "deleted_locked": locked.0,
            "deleted_unlocked": locked.1,
            "fixes": records,
            "not_fixable": refusals,
        });
        if let Err(r) = write_new(
            &fixes_path,
            serde_json::to_string_pretty(&fixes_file)?.as_bytes(),
        ) {
            return Ok(r);
        }
    }
    let n_fixes = accepted_fixes.len();
    let doglegs_fixed: usize = accepted_fixes
        .iter()
        .map(|&fi| checked.fixes[fi].doglegs.len())
        .sum();
    Ok(CallToolResult::json(&json!({
        "source": "saved_file",
        "board": spec.board,
        "dogleg_length_mm": dogleg,
        "doglegs": in_scope,
        "vias_with_doglegs": by_via.len(),
        "fixable_vias": n_fixes,
        "doglegs_fixed": doglegs_fixed,
        "not_fixable": not_fixable.len(),
        "not_fixable_by_reason": by_reason,
        "check": {
            "checker": "check_routed",
            "against": "the saved board without the items each accepted fix deletes",
            "geometric_fixes": geometric,
            "refused_by_check": checked.refused.values().filter(|r| r.0 == "check_failed").count(),
            "rounds": checked.rounds,
            "set_aside_and_retried": checked.retried,
            "violations_already_on_the_old_copper": checked.preexisting.values().sum::<usize>(),
            "violations_in_final_check": checked.final_violations,
            "new_violations": 0,
            "note": "A fix is kept only when its new copper has no violation its old copper did not already have. Of two fixes whose new copper collides only with each other, the later is set aside and retried alone once the rest has settled.",
        },
        "delete_uuids": delete_uuids.len(),
        "deleted_locked": locked.0,
        "deleted_unlocked": locked.1,
        "add": { "tracks": routing.as_ref().map_or(0, |r| r.tracks.len()), "vias": routing.as_ref().map_or(0, |r| r.vias.len()) },
        "fixes": records.iter().take(max_items).collect::<Vec<_>>(),
        "fixes_listed": records.len().min(max_items),
        "refusals": refusals.iter().take(max_items).collect::<Vec<_>>(),
        "refusals_listed": refusals.len().min(max_items),
        "elapsed_ms": t.elapsed().as_millis() as u64,
        "plan_written": written,
        "output_plan": if written { json!(output) } else { Value::Null },
        "output_fixes": if written { json!(fixes_path) } else { Value::Null },
        "next": if written {
            json!(format!(
                "Review with render_board_layers_png (overlay_routing_file). Apply in the open board: delete_routing with uuids_file {} (include_locked true when deleted_locked > 0; dry run, then expected_count), then add_routing with this routing_file (locked true to keep the copper locked); refill and run DRC.",
                fixes_path.display()
            ))
        } else {
            json!("Nothing to apply: no dogleg passed.")
        },
    })))
}

#[cfg(test)]
#[path = "board_lint_tests.rs"]
mod tests;
