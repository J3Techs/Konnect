//! Power-trunk planning: one net's wide track on one copper layer through
//! an ordered list of waypoints (tap vias), around everything of other nets.
//!
//! # Method
//!
//! Each leg (waypoint `i` to `i + 1`) is searched on a grid anchored on its
//! start waypoint, so the leg leaves the waypoint exactly. Every cell holds
//! the largest half width a track centred there could have and still clear
//! every obstacle on the layer by its rule ([`Field`]): other nets' pads,
//! tracks, arcs and vias, every hole (hole clearance), the board edge, rule
//! areas that forbid tracks, other nets' zone fills, other planned copper
//! (`avoid`) and caller keep-out boxes. A cell is passable when the minimum
//! width fits with a margin that covers the straight run between adjacent
//! cell centres; outside the neck-down radius of the leg's waypoints, a
//! cell where the preferred width does not fit costs more.
//!
//! The search is A* over (cell, direction) with octilinear moves. A turn is
//! at most 90°: two straight tracks meeting at more than 90° of turn make an
//! acute angle, which the board's `track_angle` rule refuses, and the same
//! limit applies to the turn at a waypoint between the arriving and the
//! leaving leg. Turns cost extra, so runs are long and bends few.
//!
//! The grid path reaches the cell nearest the end waypoint; the run lengths
//! are then adjusted (two runs of independent direction absorb the residual,
//! or a short 45° jog is added) so the leg ends exactly on the waypoint.
//! Every candidate geometry is checked with the exact kernel before it is
//! accepted. Each run is then split where the room around it changes (and
//! at the neck-down circles), and every piece takes the widest width from
//! the preferred down to the minimum that the exact check accepts.

use crate::api::{Generation, ItemKind, LayerId, NetId};
use crate::board::RouteBoard;
use crate::plan::{RoutedCopper, RoutedTrack, RoutedVia};
use crate::rules::{RuleSubject, RuleTable};
use crate::space::{ClearanceCache, Conflict, Space};
use konnect_geom::{
    gap_cmp, touches, ArcPolicy, BBox, Circle, ConvexPoly, Coord, Point, Polygon, Segment, Shape,
};
use serde::{Deserialize, Serialize};
use std::cmp::{Ordering, Reverse};
use std::collections::BinaryHeap;

/// What to plan.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrunkSpec {
    pub net: NetId,
    pub layer: LayerId,
    pub waypoints: Vec<Point>,
    pub preferred_width: Coord,
    pub min_width: Coord,
    /// Per leg (`waypoints[i]` to `waypoints[i + 1]`), a preferred width in
    /// place of `preferred_width`.
    pub leg_widths: Vec<Option<Coord>>,
    /// Within this distance of a leg's waypoints the trunk may run at the
    /// minimum width without penalty.
    pub neck_radius: Coord,
    /// Boxes no trunk copper may touch.
    pub keepouts: Vec<BBox>,
    /// Other planned copper to keep clear of (its own-net copper is ignored).
    pub avoid: RoutedCopper,
    /// Search grid pitch, nm.
    pub grid: Coord,
    /// Plan a via at each waypoint (skipped where the board already has a
    /// via of the net there).
    pub waypoint_vias: bool,
}

/// Search tuning.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TrunkTuning {
    /// Extra cost of a 45° bend, mm of track.
    pub bend_45_mm: f64,
    /// Extra cost of a 90° bend, mm of track.
    pub bend_90_mm: f64,
    /// Cost multiplier added per mm where only a narrow track fits.
    pub narrow_penalty: f64,
    /// Largest number of search states one leg may expand.
    pub max_expansions: usize,
    /// Most cells a leg's search grid may hold; a larger region is searched
    /// on a coarser grid.
    pub max_cells: usize,
    /// Shortest run the endpoint fit may leave, nm.
    pub min_run: Coord,
    /// Shortest piece the width profile splits a run into, nm.
    pub min_piece: Coord,
}

impl Default for TrunkTuning {
    fn default() -> Self {
        TrunkTuning {
            bend_45_mm: 1.5,
            bend_90_mm: 2.5,
            narrow_penalty: 2.0,
            max_expansions: 40_000_000,
            max_cells: 4_000_000,
            min_run: 200_000,
            min_piece: 1_000_000,
        }
    }
}

/// One planned leg.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrunkLeg {
    pub index: usize,
    pub from: Point,
    pub to: Point,
    pub preferred_width: Coord,
    /// The leg's copper, in order from `from` to `to`; each piece carries
    /// its own width.
    pub pieces: Vec<Segment>,
    /// Direction changes along the leg (collinear width changes excluded).
    pub bends: usize,
    /// Why the leg has no copper, when it has none.
    pub failure: Option<String>,
    /// Pieces the exact check still flags, with the reason.
    pub violations: Vec<(usize, String)>,
    pub grid: Coord,
    pub expansions: usize,
}

impl TrunkLeg {
    pub fn length(&self) -> f64 {
        self.pieces.iter().map(|s| (s.b - s.a).length_f64()).sum()
    }

    pub fn routed(&self) -> bool {
        self.failure.is_none() && self.violations.is_empty() && !self.pieces.is_empty()
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "status", rename_all = "snake_case")]
pub enum ViaStatus {
    Planned,
    /// The board already has a via of the net at the waypoint.
    Existing,
    /// `waypoint_vias` was off.
    NotRequested,
    /// The via would break a rule; it is left out.
    Illegal {
        reason: String,
    },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TrunkVia {
    pub at: Point,
    pub diameter: Coord,
    pub drill: Coord,
    #[serde(flatten)]
    pub status: ViaStatus,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrunkPlan {
    pub legs: Vec<TrunkLeg>,
    pub vias: Vec<TrunkVia>,
}

impl TrunkPlan {
    /// The copper to write: every routed leg's pieces and the planned vias.
    pub fn copper(&self, board: &RouteBoard, net: NetId, layer: LayerId) -> RoutedCopper {
        let mut out = RoutedCopper::default();
        for leg in self.legs.iter().filter(|l| l.routed()) {
            for s in &leg.pieces {
                out.tracks.push(RoutedTrack {
                    net,
                    layer,
                    seg: *s,
                    gen: Generation(0),
                });
            }
        }
        let all = board.layers.all();
        for v in self.vias.iter().filter(|v| v.status == ViaStatus::Planned) {
            out.vias.push(RoutedVia {
                net,
                center: v.at,
                diameter: v.diameter,
                drill: v.drill,
                layers: all,
                gen: Generation(0),
            });
        }
        out
    }
}

// ─── Obstacles ───────────────────────────────────────────────────────────────

/// One thing the trunk must keep `clearance` from.
#[derive(Debug, Clone)]
struct Obstacle {
    shape: Shape,
    clearance: Coord,
    /// Reported in failures.
    what: &'static str,
}

/// Everything on the trunk's layer of other nets, plus the exact checks
/// the context-free raster cannot do.
struct Obstacles<'a> {
    space: Space<'a>,
    rules: &'a dyn RuleTable,
    net: NetId,
    layer: LayerId,
    /// Raster obstacles, with context-free clearances.
    list: Vec<Obstacle>,
    /// Exact-only extras (zones, planned copper, keep-out boxes): shapes,
    /// clearance and label.
    extra: Vec<Obstacle>,
}

impl<'a> Obstacles<'a> {
    fn new(board: &'a RouteBoard, rules: &'a dyn RuleTable, spec: &TrunkSpec) -> Self {
        let policy = ArcPolicy::default();
        let space = Space::new(board, rules, &policy, false);
        let (net, layer) = (spec.net, spec.layer);
        let track = RuleSubject::new(ItemKind::Track, Some(net));
        let mut list = Vec::new();
        let mut cache = ClearanceCache::default();
        let key = crate::space::SubjectKey::new(ItemKind::Track, Some(net));
        for it in space.items.iter() {
            if !it.layers.contains(layer) || it.net == Some(net) {
                continue;
            }
            let c = space.clearance(&mut cache, rules, &key, &it.key, layer) + it.extra;
            list.push(Obstacle {
                shape: it.shape.clone(),
                clearance: c,
                what: "copper",
            });
        }
        for h in &space.holes {
            if h.net.is_some() && h.net == Some(net) {
                continue;
            }
            list.push(Obstacle {
                shape: h.shape.clone(),
                clearance: space.hole_clearance,
                what: "hole",
            });
        }
        for e in &board.outline.edges {
            list.push(Obstacle {
                shape: Shape::Segment(*e),
                clearance: space.edge_clearance,
                what: "board edge",
            });
        }
        for k in &board.keepouts {
            if k.rules.tracks && k.layers.contains(layer) {
                list.push(Obstacle {
                    shape: Shape::Polygon(k.region.clone()),
                    clearance: 0,
                    what: "rule area",
                });
            }
        }
        let mut extra = Vec::new();
        for z in &board.zones {
            if z.layer != layer || z.net == Some(net) {
                continue;
            }
            let c = rules.clearance(&track, &RuleSubject::new(ItemKind::Zone, z.net), layer);
            extra.push(Obstacle {
                shape: Shape::Polygon(z.region.clone()),
                clearance: c,
                what: "zone of another net",
            });
        }
        for t in &spec.avoid.tracks {
            if t.net == net || t.layer != layer {
                continue;
            }
            let c = rules.clearance(
                &track,
                &RuleSubject::new(ItemKind::Track, Some(t.net)),
                layer,
            );
            extra.push(Obstacle {
                shape: Shape::Segment(t.seg),
                clearance: c,
                what: "planned track",
            });
        }
        for v in &spec.avoid.vias {
            if v.net == net {
                continue;
            }
            let c = rules.clearance(&track, &RuleSubject::new(ItemKind::Via, Some(v.net)), layer);
            if v.layers.contains(layer) || v.layers.is_empty() {
                extra.push(Obstacle {
                    shape: Shape::Circle(Circle::from_diameter(v.center, v.diameter)),
                    clearance: c,
                    what: "planned via",
                });
            }
            extra.push(Obstacle {
                shape: Shape::Circle(Circle::from_diameter(v.center, v.drill)),
                clearance: space.hole_clearance,
                what: "planned via hole",
            });
        }
        for b in &spec.keepouts {
            if let Some(p) = ConvexPoly::axis_rect(*b, 0) {
                extra.push(Obstacle {
                    shape: Shape::ConvexPoly(p),
                    clearance: 0,
                    what: "keep-out box",
                });
            }
        }
        Obstacles {
            space,
            rules,
            net,
            layer,
            list,
            extra,
        }
    }

    /// The exact verdict on one track piece: `None` when it is legal.
    fn piece_conflict(&self, cache: &mut ClearanceCache, seg: Segment) -> Option<String> {
        let shape = Shape::Segment(seg);
        let key = self
            .space
            .subject_for(ItemKind::Track, Some(self.net), &shape);
        if let Some(c) =
            self.space
                .track_conflict(cache, self.rules, self.layer, seg, Some(self.net), &key)
        {
            return Some(conflict_text(&self.space, &c));
        }
        let bb = seg.bbox();
        for o in &self.extra {
            if !o.shape.bbox().inflate(o.clearance + 1).intersects(&bb) {
                continue;
            }
            let bad = if o.clearance == 0 {
                touches(&shape, &o.shape)
            } else {
                gap_cmp(&shape, &o.shape, i64::from(o.clearance)) == Ordering::Less
            };
            if bad {
                return Some(o.what.to_string());
            }
        }
        None
    }
}

fn conflict_text(space: &Space<'_>, c: &Conflict) -> String {
    let item = |i: &crate::api::ItemRef| -> String {
        use crate::api::ItemRef;
        let b = space.board;
        match *i {
            ItemRef::Pad(p) => {
                let pad = &b.pads[p.index()];
                format!(
                    "pad {}.{}",
                    b.footprints[pad.footprint.index()]
                        .reference
                        .as_deref()
                        .unwrap_or("?"),
                    pad.number
                )
            }
            ItemRef::Track(t) => format!(
                "track of {}",
                b.tracks[t.index()]
                    .net
                    .map_or("no net", |n| b.net(n).name.as_str())
            ),
            ItemRef::Arc(a) => format!(
                "arc of {}",
                b.arcs[a.index()]
                    .net
                    .map_or("no net", |n| b.net(n).name.as_str())
            ),
            ItemRef::Via(v) => {
                let via = &b.vias[v.index()];
                let (x, y) = via.center.to_mm();
                format!(
                    "via of {} at ({x}, {y})",
                    via.net.map_or("no net", |n| b.net(n).name.as_str())
                )
            }
            ItemRef::Zone(_) => "zone".to_string(),
        }
    };
    match c {
        Conflict::Copper { item: i, .. } => format!("clearance to {}", item(i)),
        Conflict::HoleToHole { item: i } => format!("hole to hole with {}", item(i)),
        Conflict::HoleClearance { item: i } => format!("hole clearance to {}", item(i)),
        Conflict::Edge => "board edge clearance".to_string(),
        Conflict::Keepout { .. } => "rule area".to_string(),
    }
}

// ─── The clearance field ─────────────────────────────────────────────────────

/// Per cell, the largest half width (nm) a track centred on the cell centre
/// may have while every obstacle keeps its clearance; values at or above
/// `cap` mean "plenty".
struct Field {
    origin: Point,
    step: f64,
    /// Cell index ranges relative to `origin`, inclusive.
    i0: i64,
    j0: i64,
    nx: usize,
    ny: usize,
    hw: Vec<f32>,
}

impl Field {
    fn new(origin: Point, step: Coord, region: BBox) -> Self {
        let s = f64::from(step);
        let i0 = ((f64::from(region.min.x) - f64::from(origin.x)) / s).floor() as i64;
        let i1 = ((f64::from(region.max.x) - f64::from(origin.x)) / s).ceil() as i64;
        let j0 = ((f64::from(region.min.y) - f64::from(origin.y)) / s).floor() as i64;
        let j1 = ((f64::from(region.max.y) - f64::from(origin.y)) / s).ceil() as i64;
        let nx = (i1 - i0 + 1).max(1) as usize;
        let ny = (j1 - j0 + 1).max(1) as usize;
        Field {
            origin,
            step: s,
            i0,
            j0,
            nx,
            ny,
            hw: vec![f32::MAX; nx * ny],
        }
    }

    fn center(&self, ix: usize, iy: usize) -> (f64, f64) {
        (
            f64::from(self.origin.x) + (ix as i64 + self.i0) as f64 * self.step,
            f64::from(self.origin.y) + (iy as i64 + self.j0) as f64 * self.step,
        )
    }

    /// Cell index range covering `[lo, hi]` (nm) on one axis.
    fn span(&self, lo: f64, hi: f64, x_axis: bool) -> Option<(usize, usize)> {
        let (o, k0, n) = if x_axis {
            (f64::from(self.origin.x), self.i0, self.nx)
        } else {
            (f64::from(self.origin.y), self.j0, self.ny)
        };
        let a = ((lo - o) / self.step).floor() as i64 - k0;
        let b = ((hi - o) / self.step).ceil() as i64 - k0;
        if b < 0 || a >= n as i64 {
            return None;
        }
        Some((a.max(0) as usize, (b.min(n as i64 - 1)) as usize))
    }

    fn cell_of(&self, p: Point) -> Option<(usize, usize)> {
        let ix = ((f64::from(p.x) - f64::from(self.origin.x)) / self.step).round() as i64 - self.i0;
        let iy = ((f64::from(p.y) - f64::from(self.origin.y)) / self.step).round() as i64 - self.j0;
        (ix >= 0 && iy >= 0 && (ix as usize) < self.nx && (iy as usize) < self.ny)
            .then_some((ix as usize, iy as usize))
    }

    /// Lower every cell within reach of `shape` to its distance minus
    /// `clearance`.
    fn stamp(&mut self, shape: &Shape, clearance: Coord, cap: f64) {
        let reach = f64::from(clearance) + cap;
        let bb = shape.bbox();
        let Some((xa, xb)) = self.span(
            f64::from(bb.min.x) - reach,
            f64::from(bb.max.x) + reach,
            true,
        ) else {
            return;
        };
        let Some((ya, yb)) = self.span(
            f64::from(bb.min.y) - reach,
            f64::from(bb.max.y) + reach,
            false,
        ) else {
            return;
        };
        let big =
            matches!(shape, Shape::Polygon(p) if p.rings().map(<[Point]>::len).sum::<usize>() > 64);
        if big {
            let Shape::Polygon(p) = shape else {
                unreachable!()
            };
            self.stamp_polygon(p, f64::from(clearance), cap, (xa, xb, ya, yb));
            return;
        }
        let c = f64::from(clearance);
        for iy in ya..=yb {
            for ix in xa..=xb {
                let (x, y) = self.center(ix, iy);
                let d = point_shape_distance(x, y, shape) - c;
                let cell = &mut self.hw[iy * self.nx + ix];
                if (d as f32) < *cell {
                    *cell = d as f32;
                }
            }
        }
    }

    /// A large polygon: its edges as zero-width capsules, its inside filled.
    fn stamp_polygon(&mut self, p: &Polygon, c: f64, cap: f64, win: (usize, usize, usize, usize)) {
        for ring in p.rings() {
            let n = ring.len();
            for k in 0..n {
                let (a, b) = (ring[k], ring[(k + 1) % n]);
                let seg = Shape::Segment(Segment::new(a, b, 0));
                self.stamp(&seg, c as Coord, cap);
            }
        }
        // Inside: even-odd over every ring, by scanline.
        let (xa, xb, ya, yb) = win;
        for iy in ya..=yb {
            let (_, y) = self.center(xa, iy);
            let mut xs: Vec<f64> = Vec::new();
            for ring in p.rings() {
                let n = ring.len();
                for k in 0..n {
                    let (a, b) = (ring[k], ring[(k + 1) % n]);
                    let (ay, by) = (f64::from(a.y), f64::from(b.y));
                    if (ay <= y) != (by <= y) {
                        let t = (y - ay) / (by - ay);
                        xs.push(f64::from(a.x) + t * (f64::from(b.x) - f64::from(a.x)));
                    }
                }
            }
            xs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal));
            for pair in xs.chunks(2) {
                if pair.len() < 2 {
                    break;
                }
                if let Some((ia, ib)) = self.span(pair[0], pair[1], true) {
                    for ix in ia.max(xa)..=ib.min(xb) {
                        let (x, _) = self.center(ix, iy);
                        if x > pair[0] && x < pair[1] {
                            self.hw[iy * self.nx + ix] = f32::MIN;
                        }
                    }
                }
            }
        }
    }

    fn at(&self, ix: usize, iy: usize) -> f32 {
        self.hw[iy * self.nx + ix]
    }

    /// The field at the cell nearest `p`, or `f32::MIN` outside.
    fn sample(&self, x: f64, y: f64) -> f32 {
        let ix = ((x - f64::from(self.origin.x)) / self.step).round() as i64 - self.i0;
        let iy = ((y - f64::from(self.origin.y)) / self.step).round() as i64 - self.j0;
        if ix < 0 || iy < 0 || ix as usize >= self.nx || iy as usize >= self.ny {
            return f32::MIN;
        }
        self.at(ix as usize, iy as usize)
    }
}

fn seg_dist(px: f64, py: f64, a: Point, b: Point) -> f64 {
    let (ax, ay, bx, by) = (
        f64::from(a.x),
        f64::from(a.y),
        f64::from(b.x),
        f64::from(b.y),
    );
    let (dx, dy) = (bx - ax, by - ay);
    let l2 = dx * dx + dy * dy;
    let t = if l2 > 0.0 {
        (((px - ax) * dx + (py - ay) * dy) / l2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (px - (ax + t * dx)).hypot(py - (ay + t * dy))
}

fn ring_inside(px: f64, py: f64, ring: &[Point]) -> bool {
    let n = ring.len();
    let mut inside = false;
    for k in 0..n {
        let (a, b) = (ring[k], ring[(k + 1) % n]);
        let (ax, ay, bx, by) = (
            f64::from(a.x),
            f64::from(a.y),
            f64::from(b.x),
            f64::from(b.y),
        );
        if (ay <= py) != (by <= py) {
            let x = ax + (py - ay) / (by - ay) * (bx - ax);
            if px < x {
                inside = !inside;
            }
        }
    }
    inside
}

/// Distance from a point to a shape's copper, nm; negative inside.
fn point_shape_distance(px: f64, py: f64, s: &Shape) -> f64 {
    match s {
        Shape::Circle(c) => {
            (px - f64::from(c.center.x)).hypot(py - f64::from(c.center.y)) - f64::from(c.radius)
        }
        Shape::Segment(g) => seg_dist(px, py, g.a, g.b) - f64::from(g.width) / 2.0,
        Shape::ConvexPoly(p) => {
            let pts = p.points();
            let n = pts.len();
            let edge = (0..n)
                .map(|k| seg_dist(px, py, pts[k], pts[(k + 1) % n]))
                .fold(f64::MAX, f64::min);
            let d = if ring_inside(px, py, pts) {
                -edge
            } else {
                edge
            };
            d - f64::from(p.radius())
        }
        Shape::Polygon(p) => {
            let mut edge = f64::MAX;
            let mut inside = false;
            for ring in p.rings() {
                let n = ring.len();
                for k in 0..n {
                    edge = edge.min(seg_dist(px, py, ring[k], ring[(k + 1) % n]));
                }
                inside ^= ring_inside(px, py, ring);
            }
            if inside {
                -edge
            } else {
                edge
            }
        }
    }
}

// ─── Search ──────────────────────────────────────────────────────────────────

/// Octilinear directions, y down: E, SE, S, SW, W, NW, N, NE.
const DIRS: [(i64, i64); 8] = [
    (1, 0),
    (1, 1),
    (0, 1),
    (-1, 1),
    (-1, 0),
    (-1, -1),
    (0, -1),
    (1, -1),
];

fn turn(a: u8, b: u8) -> u8 {
    let d = (i16::from(a) - i16::from(b)).rem_euclid(8) as u8;
    d.min(8 - d)
}

/// The octilinear direction closest to `(dx, dy)`.
fn nearest_dir(dx: f64, dy: f64) -> u8 {
    let a = dy.atan2(dx);
    ((a / std::f64::consts::FRAC_PI_4).round() as i64).rem_euclid(8) as u8
}

struct LegSearch<'f> {
    field: &'f Field,
    /// Half width the minimum track needs, plus the raster margin, nm.
    need_min: f32,
    need_pref: f32,
    /// Cells inside the neck-down radius of the leg's ends.
    neck: Vec<(f64, f64)>,
    neck_r2: f64,
    tuning: TrunkTuning,
    start: (usize, usize),
    goal: (usize, usize),
    /// Allowed first directions (the turn limit at the start waypoint).
    start_dir: Option<u8>,
    /// Direction toward the next waypoint, for the end penalty.
    next_dir: Option<u8>,
}

impl LegSearch<'_> {
    fn in_neck(&self, ix: usize, iy: usize) -> bool {
        let (x, y) = self.field.center(ix, iy);
        self.neck
            .iter()
            .any(|&(nx, ny)| (x - nx).powi(2) + (y - ny).powi(2) <= self.neck_r2)
    }

    /// The cheapest path as (cells, directions), or why there is none.
    fn run(&self) -> Result<(Vec<(usize, usize)>, usize), String> {
        let f = self.field;
        let (nx, ny) = (f.nx, f.ny);
        let n_states = nx * ny * 8;
        let mut g = vec![f32::INFINITY; n_states];
        let mut prev = vec![u8::MAX; n_states];
        let mut closed = vec![false; n_states];
        let step_mm = f.step / 1e6;
        let h = |ix: usize, iy: usize| -> f32 {
            let dx = (ix as f64 - self.goal.0 as f64).abs();
            let dy = (iy as f64 - self.goal.1 as f64).abs();
            ((dx.max(dy) - dx.min(dy) + dx.min(dy) * std::f64::consts::SQRT_2) * step_mm) as f32
        };
        let passable = |ix: usize, iy: usize| -> bool {
            (ix, iy) == self.start || (ix, iy) == self.goal || f.at(ix, iy) >= self.need_min
        };
        let mut heap: BinaryHeap<Reverse<(u32, u32)>> = BinaryHeap::new();
        let idx = |ix: usize, iy: usize, d: u8| (iy * nx + ix) * 8 + d as usize;
        // Seed: one step out of the start in every allowed direction.
        for d in 0..8u8 {
            if let Some(sd) = self.start_dir {
                if turn(sd, d) > 2 {
                    continue;
                }
            }
            let (dx, dy) = DIRS[d as usize];
            let (jx, jy) = (self.start.0 as i64 + dx, self.start.1 as i64 + dy);
            if jx < 0 || jy < 0 || jx >= nx as i64 || jy >= ny as i64 {
                continue;
            }
            let (jx, jy) = (jx as usize, jy as usize);
            if !passable(jx, jy) {
                continue;
            }
            let mut cost = self.move_cost(jx, jy, d);
            if let Some(sd) = self.start_dir {
                cost += self.bend_cost(sd, d);
            }
            let s = idx(jx, jy, d);
            if cost < g[s] {
                g[s] = cost;
                prev[s] = 8;
                heap.push(Reverse(((cost + h(jx, jy)).to_bits(), s as u32)));
            }
        }
        let mut expansions = 0usize;
        let mut best_goal: Option<(f32, usize)> = None;
        while let Some(Reverse((fbits, s))) = heap.pop() {
            let s = s as usize;
            let fcost = f32::from_bits(fbits);
            if let Some((bg, _)) = best_goal {
                if fcost >= bg {
                    break;
                }
            }
            if closed[s] {
                continue;
            }
            closed[s] = true;
            expansions += 1;
            if expansions > self.tuning.max_expansions {
                return Err(format!(
                    "search gave up after {} states",
                    self.tuning.max_expansions
                ));
            }
            let d = (s % 8) as u8;
            let cell = s / 8;
            let (ix, iy) = (cell % nx, cell / nx);
            if (ix, iy) == self.goal {
                let mut total = g[s];
                if let Some(nd) = self.next_dir {
                    if turn(d, nd) > 2 {
                        total += 10.0;
                    } else {
                        total += self.bend_cost(d, nd);
                    }
                }
                if best_goal.is_none_or(|(bg, _)| total < bg) {
                    best_goal = Some((total, s));
                }
                continue;
            }
            for nd in 0..8u8 {
                if turn(d, nd) > 2 {
                    continue;
                }
                let (dx, dy) = DIRS[nd as usize];
                let (jx, jy) = (ix as i64 + dx, iy as i64 + dy);
                if jx < 0 || jy < 0 || jx >= nx as i64 || jy >= ny as i64 {
                    continue;
                }
                let (jx, jy) = (jx as usize, jy as usize);
                if !passable(jx, jy) {
                    continue;
                }
                let t = idx(jx, jy, nd);
                if closed[t] {
                    continue;
                }
                let cost = g[s] + self.move_cost(jx, jy, nd) + self.bend_cost(d, nd);
                if cost < g[t] {
                    g[t] = cost;
                    prev[t] = d;
                    heap.push(Reverse(((cost + h(jx, jy)).to_bits(), t as u32)));
                }
            }
        }
        let Some((_, mut s)) = best_goal else {
            return Err("no path at the minimum width clears every obstacle".to_string());
        };
        let mut cells = Vec::new();
        loop {
            let d = (s % 8) as u8;
            let cell = s / 8;
            let (ix, iy) = (cell % nx, cell / nx);
            cells.push((ix, iy));
            let p = prev[s];
            if p == 8 {
                break;
            }
            let (dx, dy) = DIRS[d as usize];
            let (px, py) = ((ix as i64 - dx) as usize, (iy as i64 - dy) as usize);
            s = idx(px, py, p);
        }
        cells.push(self.start);
        cells.reverse();
        Ok((cells, expansions))
    }

    fn move_cost(&self, jx: usize, jy: usize, d: u8) -> f32 {
        let len = if d % 2 == 1 {
            std::f64::consts::SQRT_2
        } else {
            1.0
        } * self.field.step
            / 1e6;
        let narrow = self.field.at(jx, jy) < self.need_pref && !self.in_neck(jx, jy);
        let m = if narrow {
            1.0 + self.tuning.narrow_penalty
        } else {
            1.0
        };
        (len * m) as f32
    }

    fn bend_cost(&self, a: u8, b: u8) -> f32 {
        match turn(a, b) {
            0 => 0.0,
            1 => self.tuning.bend_45_mm as f32,
            _ => self.tuning.bend_90_mm as f32,
        }
    }
}

/// Grid cells to runs: (direction, steps).
fn runs_of(cells: &[(usize, usize)]) -> Vec<(u8, f64)> {
    let mut runs: Vec<(u8, f64)> = Vec::new();
    for w in cells.windows(2) {
        let dx = w[1].0 as i64 - w[0].0 as i64;
        let dy = w[1].1 as i64 - w[0].1 as i64;
        let d = DIRS.iter().position(|&v| v == (dx, dy)).unwrap_or(0) as u8;
        match runs.last_mut() {
            Some((ld, n)) if *ld == d => *n += 1.0,
            _ => runs.push((d, 1.0)),
        }
    }
    runs
}

/// Vertices of runs from `a`, `step` nm per unit, the last forced to `end`.
fn vertices(a: Point, runs: &[(u8, f64)], step: f64, end: Point) -> Vec<Point> {
    let mut out = vec![a];
    let (mut x, mut y) = (f64::from(a.x), f64::from(a.y));
    for &(d, n) in runs {
        let (dx, dy) = DIRS[d as usize];
        x += dx as f64 * n * step;
        y += dy as f64 * n * step;
        out.push(Point::new(x.round() as Coord, y.round() as Coord));
    }
    if let Some(last) = out.last_mut() {
        *last = end;
    }
    out
}

/// Ways to make `runs` end exactly on `end`: one run lengthened along the
/// residual, run lengths adjusted in pairs, then a 45° jog added at the end.
/// A run whose length changes keeps at least `min_run` steps (or its own
/// length, if it was shorter); a jog only has to be longer than nothing.
fn endpoint_fits(runs: &[(u8, f64)], residual: (f64, f64), min_run: f64) -> Vec<Vec<(u8, f64)>> {
    const SLIVER: f64 = 0.05;
    let kept = |r: &[(u8, f64)]| {
        r.iter().zip(runs).all(|(new, old)| {
            (new.1 - old.1).abs() < 1e-12 || (new.1 >= min_run.min(old.1) && new.1 > SLIVER)
        })
    };
    let mut out = Vec::new();
    if residual.0.abs() < 1e-9 && residual.1.abs() < 1e-9 {
        out.push(runs.to_vec());
        return out;
    }
    let n = runs.len();
    let vec = |d: u8| {
        let (x, y) = DIRS[d as usize];
        (x as f64, y as f64)
    };
    let solve = |p: (f64, f64), q: (f64, f64)| -> Option<(f64, f64)> {
        let det = p.0 * q.1 - p.1 * q.0;
        if det.abs() < 1e-12 {
            return None;
        }
        let a = (residual.0 * q.1 - residual.1 * q.0) / det;
        let b = (p.0 * residual.1 - p.1 * residual.0) / det;
        Some((a, b))
    };
    // One run along the residual absorbs it alone.
    for q in (0..n).rev() {
        let e = vec(runs[q].0);
        let cross = e.0 * residual.1 - e.1 * residual.0;
        if cross.abs() < 1e-9 {
            let along = (residual.0 * e.0 + residual.1 * e.1) / (e.0 * e.0 + e.1 * e.1);
            let mut r = runs.to_vec();
            r[q].1 += along;
            if kept(&r) {
                out.push(r);
            }
        }
    }
    // Pairs, nearest the end first.
    for q in (0..n).rev() {
        for p in (0..q).rev() {
            let Some((a, b)) = solve(vec(runs[p].0), vec(runs[q].0)) else {
                continue;
            };
            let mut r = runs.to_vec();
            r[p].1 += a;
            r[q].1 += b;
            if kept(&r) {
                out.push(r);
            }
        }
    }
    // A jog: a short run in a direction 45° off the last, after it or
    // before it.
    if let Some(&(last, _)) = runs.last() {
        for j in [(last + 1) % 8, (last + 7) % 8] {
            if let Some((a, b)) = solve(vec(last), vec(j)) {
                let mut r = runs.to_vec();
                r[n - 1].1 += a;
                if b > SLIVER && kept(&r) {
                    let mut after = r.clone();
                    after.push((j, b));
                    out.push(after);
                    let mut before = r;
                    let tail = before.pop().expect("a run");
                    before.push((j, b));
                    before.push(tail);
                    out.push(before);
                }
            }
        }
    }
    out
}

// ─── Planning ────────────────────────────────────────────────────────────────

/// Plan the trunk. Legs that cannot be routed are reported, not guessed.
pub fn plan_trunk(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    spec: &TrunkSpec,
    tuning: &TrunkTuning,
) -> TrunkPlan {
    let obstacles = Obstacles::new(board, rules, spec);
    let mut cache = ClearanceCache::default();
    let mut legs = Vec::new();
    let mut arrival: Option<u8> = None;
    let n = spec.waypoints.len();
    for i in 0..n.saturating_sub(1) {
        let (a, b) = (spec.waypoints[i], spec.waypoints[i + 1]);
        let pref = spec
            .leg_widths
            .get(i)
            .copied()
            .flatten()
            .unwrap_or(spec.preferred_width)
            .max(spec.min_width);
        let next_dir = spec
            .waypoints
            .get(i + 2)
            .map(|c| nearest_dir(f64::from(c.x - b.x), f64::from(c.y - b.y)));
        let leg = plan_leg(
            board, &obstacles, &mut cache, spec, tuning, i, a, b, pref, arrival, next_dir,
        );
        arrival = if leg.routed() {
            leg.pieces.iter().rev().find_map(|s| {
                let (dx, dy) = (f64::from(s.b.x - s.a.x), f64::from(s.b.y - s.a.y));
                (dx != 0.0 || dy != 0.0).then(|| nearest_dir(dx, dy))
            })
        } else {
            None
        };
        legs.push(leg);
    }
    let vias = plan_vias(board, rules, &obstacles, &mut cache, spec);
    TrunkPlan { legs, vias }
}

#[allow(clippy::too_many_arguments)]
fn plan_leg(
    board: &RouteBoard,
    obstacles: &Obstacles<'_>,
    cache: &mut ClearanceCache,
    spec: &TrunkSpec,
    tuning: &TrunkTuning,
    index: usize,
    a: Point,
    b: Point,
    pref: Coord,
    arrival: Option<u8>,
    next_dir: Option<u8>,
) -> TrunkLeg {
    let mut leg = TrunkLeg {
        index,
        from: a,
        to: b,
        preferred_width: pref,
        pieces: Vec::new(),
        bends: 0,
        failure: None,
        violations: Vec::new(),
        grid: spec.grid,
        expansions: 0,
    };
    if a == b {
        leg.failure = Some("the leg's waypoints are the same point".to_string());
        return leg;
    }
    let bounds = board
        .outline
        .bbox
        .unwrap_or_else(|| BBox::new(a, b).inflate(100_000_000));
    let mut last_err = String::new();
    for margin_mm in [15.0, 45.0, 1_000.0] {
        let margin = (margin_mm * 1e6) as Coord;
        let region = clip_box(&BBox::new(a, b).inflate(margin), &bounds.inflate(1_000_000));
        let cells = (region.width() as f64 / f64::from(spec.grid))
            * (region.height() as f64 / f64::from(spec.grid));
        let step = if cells > tuning.max_cells as f64 {
            (f64::from(spec.grid) * (cells / tuning.max_cells as f64).sqrt()).ceil() as Coord
        } else {
            spec.grid
        };
        leg.grid = step;
        let margin_nm = 0.75 * f64::from(step);
        let need_min = f64::from(spec.min_width) / 2.0 + margin_nm;
        let need_pref = f64::from(pref) / 2.0 + margin_nm;
        let cap = need_pref + 1_000.0;
        let mut field = Field::new(a, step, region);
        for o in obstacles.list.iter().chain(&obstacles.extra) {
            field.stamp(&o.shape, o.clearance, cap);
        }
        let Some(start) = field.cell_of(a) else {
            last_err = "a waypoint lies outside the board".to_string();
            continue;
        };
        // The cell nearest the end waypoint first, then the other corners
        // of the grid square around it: another goal cell gives another
        // path, and another chance for the exact endpoint fit.
        for goal in goal_cells(&field, b) {
            let search = LegSearch {
                field: &field,
                need_min: need_min as f32,
                need_pref: need_pref as f32,
                neck: vec![
                    (f64::from(a.x), f64::from(a.y)),
                    (f64::from(b.x), f64::from(b.y)),
                ],
                neck_r2: f64::from(spec.neck_radius).powi(2),
                tuning: *tuning,
                start,
                goal,
                start_dir: arrival,
                next_dir,
            };
            let (cells, expansions) = match search.run() {
                Ok(v) => v,
                Err(e) => {
                    last_err = e;
                    continue;
                }
            };
            leg.expansions += expansions;
            let runs = runs_of(&cells);
            let (gx, gy) = field.center(goal.0, goal.1);
            let residual = (
                (f64::from(b.x) - gx) / f64::from(step),
                (f64::from(b.y) - gy) / f64::from(step),
            );
            let min_run = f64::from(tuning.min_run) / f64::from(step);
            let mut fitted = None;
            for fit in endpoint_fits(&runs, residual, min_run) {
                // The first run must still respect the arrival turn limit.
                if let (Some(sd), Some(first)) = (arrival, fit.first()) {
                    if turn(sd, first.0) > 2 {
                        continue;
                    }
                }
                if fit.windows(2).any(|w| turn(w[0].0, w[1].0) > 2) {
                    continue;
                }
                let verts = vertices(a, &fit, f64::from(step), b);
                let ok = verts.windows(2).all(|w| {
                    obstacles
                        .piece_conflict(cache, Segment::new(w[0], w[1], spec.min_width))
                        .is_none()
                });
                if ok {
                    fitted = Some(verts);
                    break;
                }
            }
            let Some(verts) = fitted else {
                last_err = "the path found could not be made to end exactly on the waypoint and still clear".to_string();
                continue;
            };
            leg.bends = verts.len().saturating_sub(2);
            leg.pieces = widen(obstacles, cache, &field, &verts, spec, pref, tuning, a, b);
            leg.failure = None;
            return leg;
        }
    }
    leg.failure = Some(last_err);
    leg
}

/// The grid cells to aim a leg at: the one nearest `b`, then the other
/// corners of the grid square holding `b`, nearest first.
fn goal_cells(field: &Field, b: Point) -> Vec<(usize, usize)> {
    let fx = (f64::from(b.x) - f64::from(field.origin.x)) / field.step - field.i0 as f64;
    let fy = (f64::from(b.y) - f64::from(field.origin.y)) / field.step - field.j0 as f64;
    let mut cells: Vec<(f64, (usize, usize))> = Vec::new();
    for (cx, cy) in [
        (fx.floor(), fy.floor()),
        (fx.ceil(), fy.floor()),
        (fx.floor(), fy.ceil()),
        (fx.ceil(), fy.ceil()),
    ] {
        if cx < 0.0 || cy < 0.0 || cx >= field.nx as f64 || cy >= field.ny as f64 {
            continue;
        }
        let cell = (cx as usize, cy as usize);
        if cells.iter().any(|c| c.1 == cell) {
            continue;
        }
        cells.push(((cx - fx).hypot(cy - fy), cell));
    }
    cells.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(Ordering::Equal));
    cells.into_iter().map(|c| c.1).collect()
}

fn clip_box(a: &BBox, w: &BBox) -> BBox {
    let min = Point::new(a.min.x.max(w.min.x), a.min.y.max(w.min.y));
    let max = Point::new(a.max.x.min(w.max.x), a.max.y.min(w.max.y));
    if min.x <= max.x && min.y <= max.y {
        BBox::new(min, max)
    } else {
        *a
    }
}

/// Split the polyline into pieces where the room changes and at the
/// neck-down circles, and give each the widest width the exact check
/// accepts, from `pref` down to the minimum.
#[allow(clippy::too_many_arguments)]
fn widen(
    obstacles: &Obstacles<'_>,
    cache: &mut ClearanceCache,
    field: &Field,
    verts: &[Point],
    spec: &TrunkSpec,
    pref: Coord,
    tuning: &TrunkTuning,
    a: Point,
    b: Point,
) -> Vec<Segment> {
    let margin = 0.75 * field.step;
    let wide_need = f64::from(pref) / 2.0;
    let r = f64::from(spec.neck_radius);
    let mut pieces: Vec<Segment> = Vec::new();
    for w in verts.windows(2) {
        let (p, q) = (w[0], w[1]);
        let len = (q - p).length_f64();
        if len <= 0.0 {
            continue;
        }
        // Candidate cuts: the neck circles, and where the field says the
        // preferred width starts or stops fitting.
        let mut cuts: Vec<f64> = vec![0.0, 1.0];
        for c in [a, b] {
            for t in circle_hits(p, q, c, r) {
                cuts.push(t);
            }
        }
        let samples = (len / field.step).ceil().max(1.0) as usize;
        let mut prev_wide: Option<bool> = None;
        for k in 0..=samples {
            let t = k as f64 / samples as f64;
            let (x, y) = (
                f64::from(p.x) + t * f64::from(q.x - p.x),
                f64::from(p.y) + t * f64::from(q.y - p.y),
            );
            let wide = f64::from(field.sample(x, y)) - margin >= wide_need;
            if prev_wide.is_some_and(|pw| pw != wide) {
                cuts.push(t);
            }
            prev_wide = Some(wide);
        }
        cuts.sort_by(|x, y| x.partial_cmp(y).unwrap_or(Ordering::Equal));
        // Drop cuts that would leave a piece shorter than `min_piece`.
        let min_t = f64::from(tuning.min_piece) / len;
        let mut kept: Vec<f64> = vec![0.0];
        for &t in &cuts[1..] {
            if t >= 1.0 - min_t * 0.5 {
                continue;
            }
            if t - kept.last().copied().unwrap_or(0.0) >= min_t {
                kept.push(t);
            }
        }
        kept.push(1.0);
        let at = |t: f64| {
            if t <= 0.0 {
                p
            } else if t >= 1.0 {
                q
            } else {
                Point::new(
                    (f64::from(p.x) + t * f64::from(q.x - p.x)).round() as Coord,
                    (f64::from(p.y) + t * f64::from(q.y - p.y)).round() as Coord,
                )
            }
        };
        for pair in kept.windows(2) {
            let (s, e) = (at(pair[0]), at(pair[1]));
            if s == e {
                continue;
            }
            let width = widest(obstacles, cache, s, e, spec.min_width, pref);
            pieces.push(Segment::new(s, e, width));
        }
    }
    merge_collinear(pieces)
}

/// Parameters `t` in (0, 1) where segment `p`-`q` crosses the circle.
fn circle_hits(p: Point, q: Point, c: Point, r: f64) -> Vec<f64> {
    let (dx, dy) = (f64::from(q.x - p.x), f64::from(q.y - p.y));
    let (fx, fy) = (f64::from(p.x - c.x), f64::from(p.y - c.y));
    let aa = dx * dx + dy * dy;
    let bb = 2.0 * (fx * dx + fy * dy);
    let cc = fx * fx + fy * fy - r * r;
    let disc = bb * bb - 4.0 * aa * cc;
    if aa <= 0.0 || disc < 0.0 {
        return Vec::new();
    }
    let sq = disc.sqrt();
    [(-bb - sq) / (2.0 * aa), (-bb + sq) / (2.0 * aa)]
        .into_iter()
        .filter(|t| *t > 0.0 && *t < 1.0)
        .collect()
}

/// The widest width in `[min, pref]` (10 µm resolution) at which the piece
/// clears exactly; `min` when nothing wider does.
fn widest(
    obstacles: &Obstacles<'_>,
    cache: &mut ClearanceCache,
    s: Point,
    e: Point,
    min: Coord,
    pref: Coord,
) -> Coord {
    let ok = |cache: &mut ClearanceCache, w: Coord| {
        obstacles
            .piece_conflict(cache, Segment::new(s, e, w))
            .is_none()
    };
    if ok(cache, pref) {
        return pref;
    }
    let (mut lo, mut hi) = (min, pref);
    while hi - lo > 10_000 {
        let mid = lo + ((hi - lo) / 2 / 10_000) * 10_000;
        let mid = if mid <= lo { lo + 10_000 } else { mid };
        if ok(cache, mid) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    lo
}

/// Join consecutive collinear pieces of one width.
fn merge_collinear(pieces: Vec<Segment>) -> Vec<Segment> {
    let mut out: Vec<Segment> = Vec::new();
    for s in pieces {
        if let Some(last) = out.last_mut() {
            let (u, v) = (last.b - last.a, s.b - s.a);
            if last.width == s.width && last.b == s.a && u.cross(v) == 0 && u.dot(v) > 0 {
                last.b = s.b;
                continue;
            }
        }
        out.push(s);
    }
    out
}

fn plan_vias(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    obstacles: &Obstacles<'_>,
    cache: &mut ClearanceCache,
    spec: &TrunkSpec,
) -> Vec<TrunkVia> {
    let space = &obstacles.space;
    let rule = rules.via(&RuleSubject::new(ItemKind::Via, Some(spec.net)));
    let mut out = Vec::new();
    for &w in &spec.waypoints {
        let mut v = TrunkVia {
            at: w,
            diameter: rule.diameter,
            drill: rule.drill,
            status: ViaStatus::Planned,
        };
        let existing = board
            .vias
            .iter()
            .any(|bv| bv.net == Some(spec.net) && (bv.center - w).length2() <= 1_000i128 * 1_000);
        if existing {
            v.status = ViaStatus::Existing;
        } else if !spec.waypoint_vias {
            v.status = ViaStatus::NotRequested;
        } else {
            let disc = Shape::Circle(Circle::from_diameter(w, v.diameter));
            let key = space.subject_for(ItemKind::Via, Some(spec.net), &disc);
            if let Some(c) = space.via_conflict(
                cache,
                rules,
                w,
                v.diameter,
                v.drill,
                board.layers.all(),
                Some(spec.net),
                &key,
            ) {
                v.status = ViaStatus::Illegal {
                    reason: conflict_text(space, &c),
                };
            } else {
                let hole = Shape::Circle(Circle::from_diameter(w, v.drill));
                for o in &obstacles.extra {
                    if o.what == "keep-out box" || o.what == "zone of another net" {
                        continue;
                    }
                    let probe = if o.what.ends_with("hole") {
                        &hole
                    } else {
                        &disc
                    };
                    if gap_cmp(probe, &o.shape, i64::from(o.clearance)) == Ordering::Less {
                        v.status = ViaStatus::Illegal {
                            reason: format!("clearance to a {}", o.what),
                        };
                        break;
                    }
                }
            }
        }
        out.push(v);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::check::check_routed;
    use crate::synthetic::{BoardBuilder, UniformRules};

    fn mm(x: f64, y: f64) -> Point {
        Point::from_mm(x, y).unwrap()
    }

    fn spec(board: &RouteBoard, waypoints: &[(f64, f64)]) -> TrunkSpec {
        TrunkSpec {
            net: board.net_by_name("P5V").unwrap(),
            layer: board.layers.by_name("In2.Cu").unwrap(),
            waypoints: waypoints.iter().map(|&(x, y)| mm(x, y)).collect(),
            preferred_width: 2_000_000,
            min_width: 300_000,
            leg_widths: Vec::new(),
            neck_radius: 2_000_000,
            keepouts: Vec::new(),
            avoid: RoutedCopper::default(),
            grid: 100_000,
            waypoint_vias: true,
        }
    }

    /// A row of foreign through-hole pads across the straight line, with a
    /// gap, and a GND via field near the far waypoint.
    fn board() -> RouteBoard {
        let mut b = BoardBuilder::new(60.0, 40.0, &["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]);
        b.net("P5V");
        b.net("GND");
        let j = b.footprint("J1", (30.0, 20.0));
        for k in 0..8 {
            let y = 8.0 + 2.54 * f64::from(k);
            if (16.0..24.0).contains(&y) {
                continue;
            }
            b.th_pad(j, &format!("{k}"), "GND", (30.0, y), 1.7, 1.0);
        }
        for k in 0..3 {
            b.via("GND", (48.0, 18.0 + f64::from(k)), 0.6, 0.3, false);
        }
        b.build()
    }

    #[test]
    fn a_trunk_threads_the_gap_ends_exactly_and_checks_clean() {
        let board = board();
        let rules = UniformRules::default();
        let s = spec(&board, &[(10.003, 9.987), (50.011, 19.004), (50.011, 35.0)]);
        let plan = plan_trunk(&board, &rules, &s, &TrunkTuning::default());
        for leg in &plan.legs {
            assert!(
                leg.routed(),
                "leg {} failed: {:?} {:?}",
                leg.index,
                leg.failure,
                leg.violations
            );
            assert_eq!(leg.pieces.first().unwrap().a, leg.from, "starts exactly");
            assert_eq!(leg.pieces.last().unwrap().b, leg.to, "ends exactly");
            for w in leg.pieces.windows(2) {
                assert_eq!(w[0].b, w[1].a, "pieces are joined");
            }
        }
        // The first leg must thread the gap in the header at y 16..24.
        let crosses: Vec<&Segment> = plan.legs[0]
            .pieces
            .iter()
            .filter(|s| (s.a.x <= 30_000_000) != (s.b.x <= 30_000_000))
            .collect();
        assert!(!crosses.is_empty());
        let copper = plan.copper(&board, s.net, s.layer);
        let report = check_routed(&board, &rules, &copper);
        assert!(report.violations.is_empty(), "{:?}", report.violations);
        // Somewhere it is wide, and near the via field it necks down.
        let widths: Vec<Coord> = plan
            .legs
            .iter()
            .flat_map(|l| l.pieces.iter().map(|p| p.width))
            .collect();
        assert!(widths.contains(&2_000_000), "{widths:?}");
    }

    #[test]
    fn no_turn_is_sharper_than_ninety_degrees_even_at_a_waypoint() {
        let board = board();
        let rules = UniformRules::default();
        // The third waypoint lies back the way the trunk came.
        let s = spec(&board, &[(5.0, 30.0), (20.0, 30.0), (8.0, 34.0)]);
        let plan = plan_trunk(&board, &rules, &s, &TrunkTuning::default());
        let all: Vec<Segment> = plan.legs.iter().flat_map(|l| l.pieces.clone()).collect();
        assert!(plan.legs.iter().all(TrunkLeg::routed));
        for w in all.windows(2) {
            let (u, v) = (w[0].b - w[0].a, w[1].b - w[1].a);
            assert!(u.dot(v) >= 0, "a turn of more than 90 degrees: {w:?}");
        }
        let copper = plan.copper(&board, s.net, s.layer);
        let report = check_routed(&board, &rules, &copper);
        assert!(report.violations.is_empty(), "{:?}", report.violations);
    }

    #[test]
    fn keep_out_boxes_and_planned_copper_are_obstacles() {
        let board = board();
        let rules = UniformRules::default();
        let mut s = spec(&board, &[(5.0, 5.0), (25.0, 5.0)]);
        s.keepouts.push(BBox::new(mm(12.0, 0.0), mm(14.0, 8.0)));
        let gnd = board.net_by_name("GND").unwrap();
        s.avoid.tracks.push(RoutedTrack {
            net: gnd,
            layer: s.layer,
            seg: Segment::new(mm(18.0, 0.5), mm(18.0, 9.0), 500_000),
            gen: Generation(0),
        });
        let plan = plan_trunk(&board, &rules, &s, &TrunkTuning::default());
        let leg = &plan.legs[0];
        assert!(leg.routed(), "{:?}", leg.failure);
        let box_shape = Shape::ConvexPoly(ConvexPoly::axis_rect(s.keepouts[0], 0).unwrap());
        let avoid = Shape::Segment(s.avoid.tracks[0].seg);
        for p in &leg.pieces {
            assert!(!touches(&Shape::Segment(*p), &box_shape));
            assert!(gap_cmp(&Shape::Segment(*p), &avoid, 200_000) != Ordering::Less);
        }
    }

    #[test]
    fn an_unreachable_waypoint_fails_its_leg_and_says_why() {
        let board = board();
        let rules = UniformRules::default();
        let mut s = spec(&board, &[(5.0, 5.0), (25.0, 5.0)]);
        s.keepouts.push(BBox::new(mm(20.0, 0.0), mm(22.0, 40.0)));
        let plan = plan_trunk(&board, &rules, &s, &TrunkTuning::default());
        assert!(!plan.legs[0].routed());
        assert!(plan.legs[0].failure.is_some());
        assert!(plan.copper(&board, s.net, s.layer).tracks.is_empty());
    }
}
