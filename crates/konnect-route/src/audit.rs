//! S1 terminal-access audit (owned by WS-B).
//!
//! Before anyone blames the router, prove each terminal can be reached: a
//! per-layer flood fill from the pad, through space legal for the net's
//! profile, to outside the owning footprint's courtyard. Terminals that
//! cannot be reached on any routable layer are reported with a reason and
//! excluded from router scoring — they are a placement or escape problem,
//! not a routing one. Run it first on the dense BGA fields.
//!
//! # Method
//!
//! Each footprint gets a square lattice (25 µm by default, aligned to the
//! footprint anchor so ball centres are lattice points) over its courtyard
//! plus a margin. A *goal* cell is one whose centre lies outside every
//! courtyard polygon of the footprint.
//!
//! - **Seeds** are cells whose centre lies in the terminal's own copper: its
//!   pad and everything joined to it (locked dog-bones, necks, via-in-pad),
//!   on every routable layer that copper occupies. Copper of the terminal's
//!   net is never an obstacle to it; every other net's copper is, locked
//!   escapes included.
//! - **Moves** are the 8 lattice neighbours on a routable layer, and — when
//!   [`AuditSettings::vias`] is on — a new through via at a cell where the
//!   rules allow one, to every other routable layer.
//! - **Legality** is exact. A cell is legal when a dot of the track width
//!   there keeps the resolved clearance (net class, custom rules including
//!   `intersectsCourtyard` fields, local overrides, hole clearance, board
//!   edge, keepouts) to every other net's copper; a move is legal when the
//!   unit track between two cells does. Zone fills are not obstacles: a
//!   refill pours around new copper.
//!
//! The rasters are painted once per *profile* (net class, track width, via
//! size) with a representative net, recording for each cell which net's
//! copper blocks it (none, one net, or several). A search for a terminal of
//! net `N` treats cells blocked only by `N` as free. Because a project's
//! custom rules may treat two nets of one class differently, every path the
//! fast search finds is re-checked move by move with the terminal's own
//! net, and any terminal whose path fails that check, or that the fast
//! search finds unreachable, is searched again with exact checks for its
//! own net. Reported reachability is therefore exact on the lattice: a path
//! of lattice moves exists that the rules allow. A gap narrower than the
//! lattice can resolve (under one pitch) can be missed; the pitch is in the
//! settings.
//!
//! A track's clearance is resolved for the courtyards that track touches,
//! segment by segment, so a lattice move inside a BGA courtyard carries the
//! courtyard-field rule and one outside does not.

use crate::api::{
    FootprintId, ItemKind, ItemRef, LayerId, LayerMask, NetId, PadId, LATTICE_FINE_PITCH_NM,
};
use crate::board::RouteBoard;
use crate::connectivity::ConnectivityReport;
use crate::rules::{RuleSubject, RuleTable};
use crate::space::{radius_of, ClearanceCache, Space, SubjectKey};
use konnect_geom::{
    gap_cmp, point_in_polygon, touches, ArcPolicy, BBox, Circle, Containment, Coord, Point,
    Segment, Shape,
};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::cmp::Ordering;
use std::collections::{BTreeMap, VecDeque};
use std::sync::atomic::{AtomicU32, Ordering as AtomicOrdering};
use std::time::Instant;

/// Whether one terminal can escape its footprint.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TerminalAccess {
    pub pad: PadId,
    /// Routable layers on which the pad reaches outside the courtyard
    /// (directly, or through a via the rules allow).
    pub reachable: LayerMask,
    /// Why the pad is unreachable, when `reachable` is empty.
    pub reason: Option<String>,
    /// The pad's net.
    #[serde(default)]
    pub net: Option<NetId>,
    /// The net has pads outside this terminal's island, so a router must
    /// reach it. An unreachable terminal that needs no route is harmless.
    #[serde(default)]
    pub needs_route: bool,
    /// The terminal's copper already joins one of its net's plane pieces.
    #[serde(default)]
    pub joined_to_plane: bool,
    /// Lattice states the deciding search visited.
    #[serde(default)]
    pub explored: u32,
    /// The deciding search used exact checks for this terminal's own net
    /// (the profile search was not enough on its own).
    #[serde(default)]
    pub exact_rerun: bool,
}

/// One audited footprint.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct FootprintAudit {
    pub footprint: FootprintId,
    pub terminals: usize,
    pub unreachable: usize,
    /// Unreachable terminals whose net still needs a route.
    pub unreachable_needing_route: usize,
    pub pitch: Coord,
    pub cells: usize,
    pub profiles: usize,
    pub exact_reruns: usize,
    pub wall_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct AccessAudit {
    /// One entry per audited pad, in pad order.
    pub terminals: Vec<TerminalAccess>,
    /// One entry per audited footprint, in footprint order.
    #[serde(default)]
    pub footprints: Vec<FootprintAudit>,
}

impl AccessAudit {
    pub fn unreachable(&self) -> impl Iterator<Item = &TerminalAccess> {
        self.terminals.iter().filter(|t| t.reachable.is_empty())
    }
}

/// Which of a rule's sizes the audit uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SizeChoice {
    /// What the router draws: the net class (or rule `opt`) value.
    Preferred,
    /// The rule floor: the most permissive legal size.
    Minimum,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuditSettings {
    /// Lattice pitch, nm.
    pub pitch: Coord,
    /// How far past the courtyard bounding box the lattice extends, nm.
    pub margin: Coord,
    /// Track width: the net's preferred width or its floor.
    pub width: SizeChoice,
    /// Allow new through vias inside the field.
    pub vias: bool,
    /// Via size, when `vias` is on.
    pub via_size: SizeChoice,
}

impl Default for AuditSettings {
    fn default() -> Self {
        AuditSettings {
            pitch: LATTICE_FINE_PITCH_NM,
            margin: 1_000_000,
            width: SizeChoice::Preferred,
            vias: true,
            via_size: SizeChoice::Preferred,
        }
    }
}

/// Audit every netted pad of `footprints` (all footprints when empty).
pub fn audit_terminal_access(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    footprints: &[FootprintId],
) -> AccessAudit {
    audit_terminal_access_with(board, rules, footprints, &AuditSettings::default())
}

/// [`audit_terminal_access`] with explicit settings.
pub fn audit_terminal_access_with(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    footprints: &[FootprintId],
    settings: &AuditSettings,
) -> AccessAudit {
    let policy = ArcPolicy::default();
    let conn = ConnectivityReport::build(board, &policy);
    let space = Space::new(board, rules, &policy, false);
    let mut fps: Vec<FootprintId> = if footprints.is_empty() {
        board.footprints.iter().map(|f| f.id).collect()
    } else {
        footprints.to_vec()
    };
    fps.sort();
    fps.dedup();
    let ctx = Ctx {
        space: &space,
        rules,
        conn: &conn,
        islands: conn.island_of_pads(board),
        settings: *settings,
    };
    let mut audit = AccessAudit::default();
    for f in fps {
        if f.index() >= board.footprints.len() {
            continue;
        }
        let (terminals, summary) = audit_footprint(&ctx, f);
        audit.terminals.extend(terminals);
        audit.footprints.push(summary);
    }
    audit.terminals.sort_by_key(|t| t.pad);
    audit
}

struct Ctx<'a> {
    space: &'a Space<'a>,
    rules: &'a dyn RuleTable,
    conn: &'a ConnectivityReport,
    islands: Vec<Option<u32>>,
    settings: AuditSettings,
}

// ─── Lattice ─────────────────────────────────────────────────────────────────

const FREE: u32 = u32::MAX;
const MULTI: u32 = u32::MAX - 1;
const UNKNOWN: u32 = u32::MAX - 2;

#[inline]
fn merge(cur: u32, net: Option<NetId>) -> u32 {
    let n = net.map_or(MULTI, |n| n.0);
    if cur == FREE || cur == n {
        n
    } else {
        MULTI
    }
}

#[inline]
fn passes(v: u32, net: NetId) -> bool {
    v == FREE || v == net.0
}

/// Lattice moves; `DIRS[d + 4]` is the reverse of `DIRS[d]`.
const DIRS: [(i32, i32); 8] = [
    (1, 0),
    (1, 1),
    (0, 1),
    (-1, 1),
    (-1, 0),
    (-1, -1),
    (0, -1),
    (1, -1),
];

#[derive(Debug, Clone, Copy)]
struct Grid {
    origin: Point,
    pitch: Coord,
    nx: usize,
    ny: usize,
}

impl Grid {
    /// A lattice covering `area`, with `anchor` on a lattice point.
    fn new(area: BBox, anchor: Point, pitch: Coord) -> Self {
        let p = i64::from(pitch);
        let ox = i64::from(anchor.x)
            - (i64::from(anchor.x) - i64::from(area.min.x)).div_euclid(p) * p
            - p;
        let oy = i64::from(anchor.y)
            - (i64::from(anchor.y) - i64::from(area.min.y)).div_euclid(p) * p
            - p;
        let nx = ((i64::from(area.max.x) - ox) / p + 2) as usize;
        let ny = ((i64::from(area.max.y) - oy) / p + 2) as usize;
        Grid {
            origin: Point::new(ox as Coord, oy as Coord),
            pitch,
            nx,
            ny,
        }
    }

    fn len(&self) -> usize {
        self.nx * self.ny
    }

    fn center(&self, cell: usize) -> Point {
        let (c, r) = (cell % self.nx, cell / self.nx);
        Point::new(
            self.origin.x + c as Coord * self.pitch,
            self.origin.y + r as Coord * self.pitch,
        )
    }

    fn neighbor(&self, cell: usize, d: usize) -> Option<usize> {
        let (c, r) = ((cell % self.nx) as i64, (cell / self.nx) as i64);
        let (dx, dy) = DIRS[d];
        let (c2, r2) = (c + i64::from(dx), r + i64::from(dy));
        (c2 >= 0 && r2 >= 0 && (c2 as usize) < self.nx && (r2 as usize) < self.ny)
            .then(|| r2 as usize * self.nx + c2 as usize)
    }

    /// The undirected edge id of the move `d` from `cell`.
    fn edge_id(&self, cell: usize, d: usize) -> Option<usize> {
        if d < 4 {
            Some(cell * 4 + d)
        } else {
            self.neighbor(cell, d).map(|nb| nb * 4 + (d - 4))
        }
    }

    /// Cells whose centres lie in `bbox`, as (column, row) ranges.
    fn cells_in(&self, bbox: &BBox) -> Option<(usize, usize, usize, usize)> {
        let p = i64::from(self.pitch);
        let lo_c =
            ((i64::from(bbox.min.x) - i64::from(self.origin.x)) as f64 / p as f64).ceil() as i64;
        let hi_c = (i64::from(bbox.max.x) - i64::from(self.origin.x)).div_euclid(p);
        let lo_r =
            ((i64::from(bbox.min.y) - i64::from(self.origin.y)) as f64 / p as f64).ceil() as i64;
        let hi_r = (i64::from(bbox.max.y) - i64::from(self.origin.y)).div_euclid(p);
        let (lo_c, lo_r) = (lo_c.max(0), lo_r.max(0));
        let (hi_c, hi_r) = (hi_c.min(self.nx as i64 - 1), hi_r.min(self.ny as i64 - 1));
        (lo_c <= hi_c && lo_r <= hi_r).then_some((
            lo_c as usize,
            hi_c as usize,
            lo_r as usize,
            hi_r as usize,
        ))
    }

    fn bbox(&self) -> BBox {
        BBox::new(
            self.origin,
            Point::new(
                self.origin.x + (self.nx as Coord - 1) * self.pitch,
                self.origin.y + (self.ny as Coord - 1) * self.pitch,
            ),
        )
    }
}

// ─── Profiles and rasters ────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct ProfileKey {
    class: u16,
    width: Coord,
    via_diameter: Coord,
    via_drill: Coord,
}

/// One routable layer's lattice for a profile.
struct LayerRaster {
    layer: LayerId,
    core: Vec<u32>,
    strict: Vec<u32>,
    edges: Vec<AtomicU32>,
}

struct ProfileRasters {
    key: ProfileKey,
    rep: NetId,
    layers: Vec<LayerRaster>,
    via: Vec<u32>,
}

/// Courtyard sets over the lattice for one subject kind and disc radius:
/// one key per distinct set (with no net yet), and each cell's set.
struct CourtSets {
    keys: Vec<SubjectKey>,
    set: Vec<u16>,
}

impl CourtSets {
    fn new(space: &Space<'_>, grid: &Grid, kind: ItemKind, radius: Coord) -> Self {
        let rows: Vec<Vec<SubjectKey>> = (0..grid.ny)
            .into_par_iter()
            .map(|r| {
                (0..grid.nx)
                    .map(|c| {
                        let p = grid.center(r * grid.nx + c);
                        let s = Shape::Circle(Circle::new(p, radius));
                        space.subject_for(kind, None, &s)
                    })
                    .collect()
            })
            .collect();
        let mut keys: Vec<SubjectKey> = Vec::new();
        let mut lookup: BTreeMap<Vec<FootprintId>, u16> = BTreeMap::new();
        let mut set = Vec::with_capacity(grid.len());
        for row in rows {
            for k in row {
                let id = *lookup.entry(k.courtyards().to_vec()).or_insert_with(|| {
                    keys.push(k);
                    (keys.len() - 1) as u16
                });
                set.push(id);
            }
        }
        CourtSets { keys, set }
    }

    /// The keys for subjects of `net`.
    fn keys_for(&self, net: NetId) -> Vec<SubjectKey> {
        self.keys
            .iter()
            .map(|k| {
                let mut k = *k;
                k.net = Some(net);
                k
            })
            .collect()
    }
}

/// One obstacle as a raster sees it: a shape, the net it blocks for
/// (`None` blocks every net), the clearance per courtyard set (one value:
/// the same for all), and the radius of the moving disc.
struct Paint<'a> {
    shape: std::borrow::Cow<'a, Shape>,
    net: Option<NetId>,
    clr: Vec<Coord>,
    max: Coord,
    r: Coord,
    /// The item's own clearance allowance (inscribed arcs).
    extra: Coord,
}

impl<'a> Paint<'a> {
    fn uniform(
        shape: std::borrow::Cow<'a, Shape>,
        net: Option<NetId>,
        clr: Coord,
        r: Coord,
    ) -> Self {
        Paint {
            shape,
            net,
            clr: vec![clr],
            max: clr,
            r,
            extra: 0,
        }
    }
}

/// Copper items on `layer` near the grid, with per-set clearances.
fn copper_paints<'a>(
    space: &'a Space<'a>,
    rules: &dyn RuleTable,
    grid: &Grid,
    layer: LayerId,
    keys: &[SubjectKey],
    r: Coord,
) -> Vec<Paint<'a>> {
    let area = grid.bbox().inflate(space.reach + r);
    let ids: Vec<u32> = space.items_near(layer, &area).collect();
    ids.par_iter()
        .map_init(ClearanceCache::default, |cache, &id| {
            let it = &space.items[id as usize];
            let clr: Vec<Coord> = keys
                .iter()
                .map(|k| space.clearance(cache, rules, k, &it.key, layer) + it.extra)
                .collect();
            let max = clr.iter().copied().max().unwrap_or(0);
            Paint {
                shape: std::borrow::Cow::Borrowed(&it.shape),
                net: it.net,
                clr,
                max,
                r,
                extra: it.extra,
            }
        })
        .collect()
}

/// Rows per parallel painting band.
const BAND_ROWS: usize = 16;

/// Paint `obstacles` into a core raster (and a strict one, the same test
/// with `strict_margin` more clearance), in parallel row bands.
fn paint_all(
    grid: &Grid,
    set: &[u16],
    obstacles: &[Paint<'_>],
    strict_margin: Option<Coord>,
) -> (Vec<u32>, Vec<u32>) {
    let n = grid.len();
    let nx = grid.nx;
    let mut core = vec![FREE; n];
    let mut strict = vec![FREE; n];
    let margin = strict_margin.unwrap_or(0);
    let ranges: Vec<Option<(usize, usize, usize, usize)>> = obstacles
        .iter()
        .map(|o| grid.cells_in(&o.shape.bbox().inflate(o.max + o.r + margin)))
        .collect();
    core.par_chunks_mut(BAND_ROWS * nx)
        .zip(strict.par_chunks_mut(BAND_ROWS * nx))
        .enumerate()
        .for_each(|(band, (cc, sc))| {
            let rlo = band * BAND_ROWS;
            let rhi = (rlo + BAND_ROWS).min(grid.ny) - 1;
            for (o, rg) in obstacles.iter().zip(&ranges) {
                let Some((c0, c1, r0, r1)) = *rg else {
                    continue;
                };
                let (a, b) = (r0.max(rlo), r1.min(rhi));
                if a > b {
                    continue;
                }
                for row in a..=b {
                    for c in c0..=c1 {
                        let cell = row * nx + c;
                        let local = cell - rlo * nx;
                        if cc[local] == MULTI && sc[local] == MULTI {
                            continue;
                        }
                        let clr = if o.clr.len() == 1 {
                            o.clr[0]
                        } else {
                            o.clr[set[cell] as usize]
                        };
                        let disc = Shape::Circle(Circle::new(grid.center(cell), o.r));
                        if strict_margin.is_some() {
                            if gap_cmp(&disc, &o.shape, i64::from(clr + margin)) != Ordering::Less {
                                continue;
                            }
                            sc[local] = merge(sc[local], o.net);
                        }
                        if gap_cmp(&disc, &o.shape, i64::from(clr)) == Ordering::Less {
                            cc[local] = merge(cc[local], o.net);
                        }
                    }
                }
            }
        });
    (core, strict)
}

/// Paint one routable layer of a profile.
#[allow(clippy::too_many_arguments)]
fn paint_layer(
    space: &Space<'_>,
    rules: &dyn RuleTable,
    grid: &Grid,
    layer: LayerId,
    width: Coord,
    sets: &CourtSets,
    rep: NetId,
    strict_margin: Coord,
) -> LayerRaster {
    let rw = radius_of(width);
    let area = grid.bbox().inflate(space.reach + rw + strict_margin);
    let keys = sets.keys_for(rep);
    // Copper of every net, the terminal's own included (searches skip it).
    let mut obstacles = copper_paints(space, rules, grid, layer, &keys, rw);
    // Holes: hole clearance to copper of other nets.
    for id in space.holes_near(&area) {
        let h = &space.holes[id as usize];
        obstacles.push(Paint::uniform(
            std::borrow::Cow::Borrowed(&h.shape),
            h.net,
            space.hole_clearance,
            rw,
        ));
    }
    for e in &space.board.outline.edges {
        if e.bbox().inflate(space.edge_clearance).intersects(&area) {
            obstacles.push(Paint::uniform(
                std::borrow::Cow::Owned(Shape::Segment(*e)),
                None,
                space.edge_clearance,
                rw,
            ));
        }
    }
    for k in &space.board.keepouts {
        if k.rules.tracks && k.layers.contains(layer) && k.region.bbox().intersects(&area) {
            obstacles.push(Paint::uniform(
                std::borrow::Cow::Owned(Shape::Polygon(k.region.clone())),
                None,
                0,
                rw,
            ));
        }
    }
    let (core, strict) = paint_all(grid, &sets.set, &obstacles, Some(strict_margin));
    LayerRaster {
        layer,
        edges: (0..4 * grid.len())
            .map(|_| AtomicU32::new(UNKNOWN))
            .collect(),
        core,
        strict,
    }
}

/// Paint where a through via of the profile is legal.
#[allow(clippy::too_many_arguments)]
fn paint_vias(
    space: &Space<'_>,
    rules: &dyn RuleTable,
    grid: &Grid,
    diameter: Coord,
    drill: Coord,
    sets: &CourtSets,
    rep: NetId,
) -> Vec<u32> {
    let (rv, rh) = (radius_of(diameter), radius_of(drill));
    let area = grid.bbox().inflate(space.reach + rv);
    let keys = sets.keys_for(rep);
    let mut obstacles: Vec<Paint<'_>> = Vec::new();
    for layer in space.board.layers.all().iter() {
        let copper = copper_paints(space, rules, grid, layer, &keys, rv);
        // The via's hole against the same copper (hole clearance).
        for p in &copper {
            obstacles.push(Paint::uniform(
                p.shape.clone(),
                p.net,
                space.hole_clearance + p.extra,
                rh,
            ));
        }
        obstacles.extend(copper);
    }
    for id in space.holes_near(&area) {
        let h = &space.holes[id as usize];
        // Annulus to another net's hole; hole to any hole.
        obstacles.push(Paint::uniform(
            std::borrow::Cow::Borrowed(&h.shape),
            h.net,
            space.hole_clearance,
            rv,
        ));
        obstacles.push(Paint::uniform(
            std::borrow::Cow::Borrowed(&h.shape),
            None,
            space.hole_to_hole,
            rh,
        ));
    }
    for e in &space.board.outline.edges {
        if e.bbox().inflate(space.edge_clearance).intersects(&area) {
            obstacles.push(Paint::uniform(
                std::borrow::Cow::Owned(Shape::Segment(*e)),
                None,
                space.edge_clearance,
                rv,
            ));
        }
    }
    for k in &space.board.keepouts {
        if k.rules.vias && k.region.bbox().intersects(&area) {
            obstacles.push(Paint::uniform(
                std::borrow::Cow::Owned(Shape::Polygon(k.region.clone())),
                None,
                0,
                rv,
            ));
        }
    }
    paint_all(grid, &sets.set, &obstacles, None).0
}

// ─── Exact checks for one net ────────────────────────────────────────────────

#[allow(clippy::too_many_arguments)]
fn exact_track_ok(
    space: &Space<'_>,
    rules: &dyn RuleTable,
    cache: &mut ClearanceCache,
    layer: LayerId,
    a: Point,
    b: Point,
    width: Coord,
    net: NetId,
) -> bool {
    let seg = Segment::new(a, b, width);
    let key = space.subject_for(ItemKind::Track, Some(net), &Shape::Segment(seg));
    space
        .track_conflict(cache, rules, layer, seg, Some(net), &key)
        .is_none()
}

fn exact_via_ok(
    space: &Space<'_>,
    rules: &dyn RuleTable,
    cache: &mut ClearanceCache,
    p: Point,
    diameter: Coord,
    drill: Coord,
    net: NetId,
) -> bool {
    let disc = Shape::Circle(Circle::from_diameter(p, diameter));
    let key = space.subject_for(ItemKind::Via, Some(net), &disc);
    space
        .via_conflict(
            cache,
            rules,
            p,
            diameter,
            drill,
            space.board.layers.all(),
            Some(net),
            &key,
        )
        .is_none()
}

/// The edge value of a unit move for the profile: which nets' copper it
/// violates, merged like a cell.
#[allow(clippy::too_many_arguments)]
fn edge_value(
    space: &Space<'_>,
    rules: &dyn RuleTable,
    cache: &mut ClearanceCache,
    layer: LayerId,
    a: Point,
    b: Point,
    width: Coord,
    rep: NetId,
) -> u32 {
    let seg = Segment::new(a, b, width);
    let shape = Shape::Segment(seg);
    let key = space.subject_for(ItemKind::Track, Some(rep), &shape);
    let mut v = FREE;
    let bb = shape.bbox().inflate(space.reach);
    for id in space.items_near(layer, &bb) {
        let it = &space.items[id as usize];
        let clr = space.clearance(cache, rules, &key, &it.key, layer) + it.extra;
        if gap_cmp(&shape, &it.shape, i64::from(clr)) == Ordering::Less {
            v = merge(v, it.net);
            if v == MULTI {
                return v;
            }
        }
    }
    for id in space.holes_near(&bb) {
        let h = &space.holes[id as usize];
        if gap_cmp(&shape, &h.shape, i64::from(space.hole_clearance)) == Ordering::Less {
            v = merge(v, h.net);
            if v == MULTI {
                return v;
            }
        }
    }
    if space.edge_conflict(&shape).is_some()
        || space
            .keepout_conflict(&shape, LayerMask::single(layer), ItemKind::Track)
            .is_some()
    {
        return MULTI;
    }
    v
}

// ─── Search ──────────────────────────────────────────────────────────────────

/// The legality questions a search asks.
trait Oracle {
    fn passable(&mut self, li: usize, cell: usize) -> bool;
    fn edge_ok(&mut self, li: usize, cell: usize, d: usize) -> bool;
    fn via_ok(&mut self, cell: usize) -> bool;
}

struct FastOracle<'a> {
    space: &'a Space<'a>,
    rules: &'a dyn RuleTable,
    grid: &'a Grid,
    prof: &'a ProfileRasters,
    net: NetId,
    cache: &'a mut ClearanceCache,
    vias: bool,
}

impl Oracle for FastOracle<'_> {
    fn passable(&mut self, li: usize, cell: usize) -> bool {
        passes(self.prof.layers[li].core[cell], self.net)
    }

    fn edge_ok(&mut self, li: usize, cell: usize, d: usize) -> bool {
        let lr = &self.prof.layers[li];
        let Some(nb) = self.grid.neighbor(cell, d) else {
            return false;
        };
        if passes(lr.strict[cell], self.net) && passes(lr.strict[nb], self.net) {
            return true;
        }
        let Some(eid) = self.grid.edge_id(cell, d) else {
            return false;
        };
        let mut v = lr.edges[eid].load(AtomicOrdering::Relaxed);
        if v == UNKNOWN {
            v = edge_value(
                self.space,
                self.rules,
                self.cache,
                lr.layer,
                self.grid.center(cell),
                self.grid.center(nb),
                self.prof.key.width,
                self.prof.rep,
            );
            lr.edges[eid].store(v, AtomicOrdering::Relaxed);
        }
        passes(v, self.net)
    }

    fn via_ok(&mut self, cell: usize) -> bool {
        self.vias && passes(self.prof.via[cell], self.net)
    }
}

struct ExactOracle<'a> {
    space: &'a Space<'a>,
    rules: &'a dyn RuleTable,
    grid: &'a Grid,
    layers: &'a [LayerId],
    key: ProfileKey,
    net: NetId,
    cache: &'a mut ClearanceCache,
    vias: bool,
    cell_memo: Vec<u8>,
    via_memo: Vec<u8>,
}

impl Oracle for ExactOracle<'_> {
    fn passable(&mut self, li: usize, cell: usize) -> bool {
        let i = li * self.grid.len() + cell;
        if self.cell_memo[i] == 0 {
            let p = self.grid.center(cell);
            let ok = exact_track_ok(
                self.space,
                self.rules,
                self.cache,
                self.layers[li],
                p,
                p,
                self.key.width,
                self.net,
            );
            self.cell_memo[i] = if ok { 1 } else { 2 };
        }
        self.cell_memo[i] == 1
    }

    fn edge_ok(&mut self, li: usize, cell: usize, d: usize) -> bool {
        let Some(nb) = self.grid.neighbor(cell, d) else {
            return false;
        };
        exact_track_ok(
            self.space,
            self.rules,
            self.cache,
            self.layers[li],
            self.grid.center(cell),
            self.grid.center(nb),
            self.key.width,
            self.net,
        )
    }

    fn via_ok(&mut self, cell: usize) -> bool {
        if !self.vias {
            return false;
        }
        if self.via_memo[cell] == 0 {
            let ok = exact_via_ok(
                self.space,
                self.rules,
                self.cache,
                self.grid.center(cell),
                self.key.via_diameter,
                self.key.via_drill,
                self.net,
            );
            self.via_memo[cell] = if ok { 1 } else { 2 };
        }
        self.via_memo[cell] == 1
    }
}

/// Per-thread search buffers.
#[derive(Default)]
struct Buffers {
    stamp: Vec<u32>,
    parent: Vec<u32>,
    gen: u32,
    queue: VecDeque<u32>,
}

struct SearchResult {
    mask: LayerMask,
    /// The goal state found first on each routable layer.
    goals: Vec<Option<u32>>,
    explored: u32,
    seeds: usize,
    via_sites: u32,
}

#[allow(clippy::too_many_arguments)]
fn search(
    grid: &Grid,
    goal: &[bool],
    layers: &[LayerId],
    seeds: &[(usize, usize)],
    oracle: &mut dyn Oracle,
    buf: &mut Buffers,
    vias: bool,
) -> SearchResult {
    let n = grid.len();
    let total = n * layers.len();
    if buf.stamp.len() != total {
        buf.stamp = vec![0; total];
        buf.parent = vec![0; total];
        buf.gen = 0;
    }
    buf.gen = buf.gen.wrapping_add(1);
    if buf.gen == 0 {
        buf.stamp.iter_mut().for_each(|s| *s = 0);
        buf.gen = 1;
    }
    let gen = buf.gen;
    buf.queue.clear();
    let mut res = SearchResult {
        mask: LayerMask::EMPTY,
        goals: vec![None; layers.len()],
        explored: 0,
        seeds: 0,
        via_sites: 0,
    };
    for &(li, cell) in seeds {
        let s = li * n + cell;
        if buf.stamp[s] == gen || !oracle.passable(li, cell) {
            continue;
        }
        buf.stamp[s] = gen;
        buf.parent[s] = s as u32;
        buf.queue.push_back(s as u32);
        res.seeds += 1;
    }
    let all_layers = layers.len();
    // Without vias a search can only finish on layers it starts on.
    let target = if vias {
        all_layers
    } else {
        let mut seeded = vec![false; all_layers];
        for &(li, _) in seeds {
            seeded[li] = true;
        }
        seeded.iter().filter(|s| **s).count()
    };
    let mut found = 0usize;
    while let Some(s) = buf.queue.pop_front() {
        let s = s as usize;
        res.explored += 1;
        let (li, cell) = (s / n, s % n);
        if goal[cell] {
            if res.goals[li].is_none() {
                res.goals[li] = Some(s as u32);
                res.mask = res.mask.with(layers[li]);
                found += 1;
                if found == target {
                    break;
                }
            }
            continue;
        }
        for d in 0..8 {
            let Some(nb) = grid.neighbor(cell, d) else {
                continue;
            };
            let t = li * n + nb;
            if buf.stamp[t] == gen || !oracle.passable(li, nb) || !oracle.edge_ok(li, cell, d) {
                continue;
            }
            buf.stamp[t] = gen;
            buf.parent[t] = s as u32;
            buf.queue.push_back(t as u32);
        }
        if vias && all_layers > 1 {
            // Only ask about a via when it could lead somewhere new.
            let pending = (0..all_layers).any(|lj| lj != li && buf.stamp[lj * n + cell] != gen);
            if pending && oracle.via_ok(cell) {
                res.via_sites += 1;
                for lj in 0..all_layers {
                    let t = lj * n + cell;
                    if lj == li || buf.stamp[t] == gen || !oracle.passable(lj, cell) {
                        continue;
                    }
                    buf.stamp[t] = gen;
                    buf.parent[t] = s as u32;
                    buf.queue.push_back(t as u32);
                }
            }
        }
    }
    res
}

/// Re-check the path to `goal` move by move for `net`.
#[allow(clippy::too_many_arguments)]
fn verify_path(
    space: &Space<'_>,
    rules: &dyn RuleTable,
    cache: &mut ClearanceCache,
    grid: &Grid,
    layers: &[LayerId],
    key: ProfileKey,
    net: NetId,
    buf: &Buffers,
    goal: u32,
) -> bool {
    let n = grid.len();
    let mut s = goal as usize;
    loop {
        let p = buf.parent[s] as usize;
        let (li, cell) = (s / n, s % n);
        let pt = grid.center(cell);
        if p == s {
            return exact_track_ok(space, rules, cache, layers[li], pt, pt, key.width, net);
        }
        let (lp, pcell) = (p / n, p % n);
        let ok = if lp == li {
            exact_track_ok(
                space,
                rules,
                cache,
                layers[li],
                grid.center(pcell),
                pt,
                key.width,
                net,
            )
        } else {
            exact_via_ok(
                space,
                rules,
                cache,
                pt,
                key.via_diameter,
                key.via_drill,
                net,
            ) && exact_track_ok(space, rules, cache, layers[li], pt, pt, key.width, net)
        };
        if !ok {
            return false;
        }
        s = p;
    }
}

// ─── Per footprint ───────────────────────────────────────────────────────────

struct Terminal {
    pad: PadId,
    net: NetId,
    profile: usize,
    needs_route: bool,
    joined_to_plane: bool,
    /// Own copper on routable layers: (layer index, shape).
    own: Vec<(usize, Shape)>,
}

fn audit_footprint(ctx: &Ctx<'_>, f: FootprintId) -> (Vec<TerminalAccess>, FootprintAudit) {
    let start = Instant::now();
    let space = ctx.space;
    let board = space.board;
    let rules = ctx.rules;
    let fp = &board.footprints[f.index()];
    let settings = ctx.settings;
    let pads: Vec<PadId> = (fp.pad_range.0..fp.pad_range.1)
        .map(PadId)
        .filter(|p| board.pads[p.index()].net.is_some())
        .collect();
    let mut summary = FootprintAudit {
        footprint: f,
        terminals: pads.len(),
        pitch: settings.pitch,
        ..Default::default()
    };
    let layers: Vec<LayerId> = board.routable.iter().collect();
    let Some(court_bb) = fp.courtyard_bbox().filter(|_| !layers.is_empty()) else {
        let reason = if layers.is_empty() {
            "the board has no routable layers"
        } else {
            "the footprint has no courtyard"
        };
        let out = pads
            .iter()
            .map(|&p| TerminalAccess {
                pad: p,
                reachable: LayerMask::EMPTY,
                reason: Some(reason.to_string()),
                net: board.pads[p.index()].net,
                needs_route: false,
                joined_to_plane: false,
                explored: 0,
                exact_rerun: false,
            })
            .collect::<Vec<_>>();
        summary.unreachable = out.len();
        return (out, summary);
    };
    let grid = Grid::new(court_bb.inflate(settings.margin), fp.at, settings.pitch);
    summary.cells = grid.len();
    let courts: Vec<&konnect_geom::Polygon> = fp.courtyards.iter().map(|c| &c.region).collect();
    let goal: Vec<bool> = (0..grid.len())
        .into_par_iter()
        .map(|cell| {
            let p = grid.center(cell);
            courts
                .iter()
                .all(|c| point_in_polygon(p, c) == Containment::Outside)
        })
        .collect();

    // Terminals and their profiles.
    let mut profiles: Vec<(ProfileKey, NetId)> = Vec::new();
    let mut terminals: Vec<Terminal> = Vec::new();
    let in_field = [f];
    for &pid in &pads {
        let pad = &board.pads[pid.index()];
        let net = pad.net.expect("netted pads only");
        let class = rules.class_of(Some(net));
        let track = RuleSubject::new(ItemKind::Track, Some(net)).with_courtyards(&in_field);
        let width = layers
            .iter()
            .map(|&l| {
                let w = rules.track_width(&track, l);
                match settings.width {
                    SizeChoice::Preferred => w.preferred,
                    SizeChoice::Minimum => w.min,
                }
            })
            .max()
            .unwrap_or(0)
            .max(1);
        let via = rules.via(&RuleSubject::new(ItemKind::Via, Some(net)).with_courtyards(&in_field));
        let (vd, vh) = match settings.via_size {
            SizeChoice::Preferred => (via.diameter, via.drill),
            SizeChoice::Minimum => (via.min_diameter, via.min_drill),
        };
        let key = ProfileKey {
            class: class.0,
            width,
            via_diameter: vd,
            via_drill: vh,
        };
        let profile = match profiles.iter().position(|(k, _)| *k == key) {
            Some(i) => i,
            None => {
                profiles.push((key, net));
                profiles.len() - 1
            }
        };
        let (needs_route, joined_to_plane, own) = own_copper(ctx, pid, &layers, &grid);
        terminals.push(Terminal {
            pad: pid,
            net,
            profile,
            needs_route,
            joined_to_plane,
            own,
        });
    }
    summary.profiles = profiles.len();

    // Courtyard sets per (kind, disc radius), shared by the profiles.
    let mut wanted: Vec<(u8, Coord)> = Vec::new();
    for (k, _) in &profiles {
        wanted.push((0, radius_of(k.width)));
        if settings.vias {
            wanted.push((1, radius_of(k.via_diameter)));
        }
    }
    wanted.sort();
    wanted.dedup();
    let sets: BTreeMap<(u8, Coord), CourtSets> = wanted
        .into_iter()
        .map(|(kind, r)| {
            let k = if kind == 0 {
                ItemKind::Track
            } else {
                ItemKind::Via
            };
            ((kind, r), CourtSets::new(space, &grid, k, r))
        })
        .collect();

    let mut results: Vec<TerminalAccess> = Vec::with_capacity(terminals.len());
    for (pi, (key, rep)) in profiles.iter().enumerate() {
        let members: Vec<&Terminal> = terminals.iter().filter(|t| t.profile == pi).collect();
        let prof = build_profile(space, rules, &grid, &layers, *key, *rep, settings, &sets);
        let out: Vec<TerminalAccess> = members
            .par_iter()
            .map_init(
                || (Buffers::default(), ClearanceCache::default()),
                |(buf, cache), t| audit_terminal(ctx, &grid, &goal, &layers, &prof, t, buf, cache),
            )
            .collect();
        results.extend(out);
    }
    results.sort_by_key(|t| t.pad);
    summary.unreachable = results.iter().filter(|t| t.reachable.is_empty()).count();
    summary.unreachable_needing_route = results
        .iter()
        .filter(|t| t.reachable.is_empty() && t.needs_route)
        .count();
    summary.exact_reruns = results.iter().filter(|t| t.exact_rerun).count();
    summary.wall_ms = start.elapsed().as_millis() as u64;
    (results, summary)
}

#[allow(clippy::too_many_arguments)]
fn build_profile(
    space: &Space<'_>,
    rules: &dyn RuleTable,
    grid: &Grid,
    layers: &[LayerId],
    key: ProfileKey,
    rep: NetId,
    settings: AuditSettings,
    sets: &BTreeMap<(u8, Coord), CourtSets>,
) -> ProfileRasters {
    let rw = radius_of(key.width);
    // Half a lattice diagonal, rounded up.
    let strict_margin =
        ((f64::from(settings.pitch) * std::f64::consts::SQRT_2 / 2.0).ceil() as Coord) + 1;
    let track_sets = &sets[&(0, rw)];
    let out_layers: Vec<LayerRaster> = layers
        .iter()
        .map(|&l| {
            paint_layer(
                space,
                rules,
                grid,
                l,
                key.width,
                track_sets,
                rep,
                strict_margin,
            )
        })
        .collect();
    let via = if settings.vias {
        let via_sets = &sets[&(1, radius_of(key.via_diameter))];
        paint_vias(
            space,
            rules,
            grid,
            key.via_diameter,
            key.via_drill,
            via_sets,
            rep,
        )
    } else {
        vec![MULTI; grid.len()]
    };
    ProfileRasters {
        key,
        rep,
        layers: out_layers,
        via,
    }
}

/// The terminal's own copper on routable layers (its island, without plane
/// pieces), whether its net needs a route to it, and whether it already
/// joins a plane.
fn own_copper(
    ctx: &Ctx<'_>,
    pad: PadId,
    layers: &[LayerId],
    grid: &Grid,
) -> (bool, bool, Vec<(usize, Shape)>) {
    let board = ctx.space.board;
    let p = &board.pads[pad.index()];
    let net = p.net.expect("netted pads only");
    let nc = &ctx.conn.nets[net.index()];
    let island = ctx.islands[pad.index()].map(|i| &nc.islands[i as usize]);
    let needs_route = nc.pad_island_count() > 1;
    let joined = island.is_some_and(|i| i.items.iter().any(|it| matches!(it, ItemRef::Zone(_))));
    let area = grid.bbox();
    let mut own = Vec::new();
    let mut add = |layer_mask: LayerMask, shape: Shape| {
        if !shape.bbox().intersects(&area) {
            return;
        }
        for (li, l) in layers.iter().enumerate() {
            if layer_mask.contains(*l) {
                own.push((li, shape.clone()));
            }
        }
    };
    let items: Vec<ItemRef> = match island {
        Some(i) => i.items.clone(),
        None => vec![ItemRef::Pad(pad)],
    };
    for it in items {
        match it {
            ItemRef::Pad(id) => {
                let q = &board.pads[id.index()];
                for s in &q.shapes {
                    add(q.layers, s.clone());
                }
            }
            ItemRef::Track(id) => {
                let t = &board.tracks[id.index()];
                add(LayerMask::single(t.layer), Shape::Segment(t.seg));
            }
            ItemRef::Arc(id) => {
                let a = &board.arcs[id.index()];
                for s in a.segments(&ArcPolicy::default()) {
                    add(LayerMask::single(a.layer), Shape::Segment(s));
                }
            }
            ItemRef::Via(id) => {
                let v = &board.vias[id.index()];
                add(v.layers, v.shape());
            }
            ItemRef::Zone(_) => {}
        }
    }
    (needs_route, joined, own)
}

#[allow(clippy::too_many_arguments)]
fn audit_terminal(
    ctx: &Ctx<'_>,
    grid: &Grid,
    goal: &[bool],
    layers: &[LayerId],
    prof: &ProfileRasters,
    t: &Terminal,
    buf: &mut Buffers,
    cache: &mut ClearanceCache,
) -> TerminalAccess {
    let space = ctx.space;
    let rules = ctx.rules;
    let vias = ctx.settings.vias;
    // Seeds: lattice points inside the terminal's own copper.
    let mut seeds: Vec<(usize, usize)> = Vec::new();
    for (li, shape) in &t.own {
        if let Some((c0, c1, r0, r1)) = grid.cells_in(&shape.bbox()) {
            for r in r0..=r1 {
                for c in c0..=c1 {
                    let cell = r * grid.nx + c;
                    let pt = Shape::Circle(Circle::new(grid.center(cell), 0));
                    if touches(&pt, shape) {
                        seeds.push((*li, cell));
                    }
                }
            }
        }
    }
    seeds.sort_unstable();
    seeds.dedup();

    let mut access = TerminalAccess {
        pad: t.pad,
        reachable: LayerMask::EMPTY,
        reason: None,
        net: Some(t.net),
        needs_route: t.needs_route,
        joined_to_plane: t.joined_to_plane,
        explored: 0,
        exact_rerun: false,
    };
    if seeds.is_empty() {
        access.reason =
            Some("no lattice point lies in the terminal's copper; refine the pitch".into());
        return access;
    }

    let fast = {
        let mut oracle = FastOracle {
            space,
            rules,
            grid,
            prof,
            net: t.net,
            cache,
            vias,
        };
        search(grid, goal, layers, &seeds, &mut oracle, buf, vias)
    };
    let mut verified = true;
    for g in fast.goals.iter().flatten() {
        if !verify_path(space, rules, cache, grid, layers, prof.key, t.net, buf, *g) {
            verified = false;
            break;
        }
    }
    let result = if verified && (!fast.mask.is_empty() || t.net == prof.rep) {
        fast
    } else {
        access.exact_rerun = true;
        let mut oracle = ExactOracle {
            space,
            rules,
            grid,
            layers,
            key: prof.key,
            net: t.net,
            cache,
            vias,
            cell_memo: vec![0; grid.len() * layers.len()],
            via_memo: vec![0; grid.len()],
        };
        search(grid, goal, layers, &seeds, &mut oracle, buf, vias)
    };
    access.reachable = result.mask;
    access.explored = result.explored;
    if result.mask.is_empty() {
        let names: Vec<&str> = t
            .own
            .iter()
            .map(|(li, _)| space.board.layers.name(layers[*li]))
            .collect::<std::collections::BTreeSet<_>>()
            .into_iter()
            .collect();
        access.reason = Some(if result.seeds == 0 {
            format!(
                "no legal track start in the terminal's copper (on {}) for a {} nm track",
                names.join(", "),
                prof.key.width
            )
        } else {
            format!(
                "enclosed: {} lattice states reachable from the copper on {} for a {} nm track, \
                 {} legal via sites among them, none outside the courtyard",
                result.explored,
                names.join(", "),
                prof.key.width,
                result.via_sites
            )
        });
    }
    access
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synthetic::{self, BoardBuilder, UniformRules};

    fn fp(board: &RouteBoard, r: &str) -> FootprintId {
        board.footprint_by_reference(r).unwrap()
    }

    #[test]
    fn an_open_field_is_reachable_on_every_layer() {
        // 3 x 3 balls at 1.0 mm with 0.2 mm clearance and 0.15 mm tracks: a
        // 0.6 mm gap between 0.4 mm balls leaves room for one track.
        let board = synthetic::bga_field(3, 3, 1.0, 0.4, false);
        let rules = UniformRules::default();
        let audit = audit_terminal_access(&board, &rules, &[fp(&board, "BGA1")]);
        assert_eq!(audit.terminals.len(), 9);
        assert_eq!(audit.unreachable().count(), 0);
        // Vias may be placed between the balls, so every routable layer
        // can be reached.
        for t in &audit.terminals {
            assert_eq!(t.reachable, board.routable, "{t:?}");
        }
    }

    #[test]
    fn a_ring_of_other_nets_encloses_the_centre_ball() {
        // 3 x 3 balls at 0.5 mm pitch: 0.3 mm balls leave a 0.2 mm gap,
        // less than a 0.15 mm track plus two 0.2 mm clearances. Without
        // vias, the centre ball cannot get out on F.Cu; the edge balls can.
        let board = synthetic::bga_field(3, 3, 0.5, 0.3, false);
        let rules = UniformRules::default();
        let settings = AuditSettings {
            vias: false,
            ..Default::default()
        };
        let audit = audit_terminal_access_with(&board, &rules, &[fp(&board, "BGA1")], &settings);
        let unreachable: Vec<&TerminalAccess> = audit.unreachable().collect();
        assert_eq!(unreachable.len(), 1, "{:?}", audit.terminals);
        let centre = &board.pads[unreachable[0].pad.index()];
        assert_eq!(centre.number, "1_1");
        assert!(unreachable[0]
            .reason
            .as_deref()
            .unwrap()
            .starts_with("enclosed"));
        // Corners and edges get out on F.Cu only.
        let f = board.layers.front().unwrap();
        for t in audit.terminals.iter().filter(|t| !t.reachable.is_empty()) {
            assert_eq!(t.reachable, LayerMask::single(f));
        }
    }

    #[test]
    fn own_dogbone_vias_open_the_other_layers() {
        // The dog-bone via of each ball is its own net's copper: a seed on
        // every layer. At 1.2 mm pitch the dog-bones leave no channel on
        // F.Cu for the inner balls, but B.Cu holds only the vias, 0.7 mm
        // apart: room for a 0.15 mm track and two 0.2 mm clearances.
        let board = synthetic::bga_field(4, 4, 1.2, 0.4, true);
        let rules = UniformRules::default();
        let settings = AuditSettings {
            vias: false,
            ..Default::default()
        };
        let audit = audit_terminal_access_with(&board, &rules, &[fp(&board, "BGA1")], &settings);
        assert_eq!(audit.unreachable().count(), 0, "{:?}", audit.terminals);
        let b = board.layers.back().unwrap();
        for t in &audit.terminals {
            assert!(t.reachable.contains(b), "{t:?}");
        }
    }

    #[test]
    fn locked_copper_of_another_net_is_an_obstacle() {
        // A ball fenced in by a closed square of locked track of another
        // net, inside the courtyard.
        let mut b = BoardBuilder::new(10.0, 10.0, &["F.Cu", "B.Cu"]);
        let u = b.footprint_with_courtyard("U1", (5.0, 5.0), (2.0, 2.0));
        b.ball_pad(u, "1", "A", (5.0, 5.0), 0.4, "F.Cu");
        b.track("FENCE", "F.Cu", (4.0, 4.0), (4.0, 6.0), 0.2, true)
            .track("FENCE", "F.Cu", (4.0, 6.0), (6.0, 6.0), 0.2, true)
            .track("FENCE", "F.Cu", (6.0, 6.0), (6.0, 4.0), 0.2, true)
            .track("FENCE", "F.Cu", (6.0, 4.0), (4.0, 4.0), 0.2, true);
        let board = b.build();
        let rules = UniformRules::default();
        let settings = AuditSettings {
            vias: false,
            ..Default::default()
        };
        let audit = audit_terminal_access_with(&board, &rules, &[fp(&board, "U1")], &settings);
        assert_eq!(audit.unreachable().count(), 1);
        // With vias allowed, it leaves on B.Cu through a new via inside the
        // fence, and can come back up to F.Cu through a second one outside.
        let audit = audit_terminal_access(&board, &rules, &[fp(&board, "U1")]);
        let t = &audit.terminals[0];
        assert_eq!(t.reachable, board.layers.all());
        assert!(t.reason.is_none());
    }

    #[test]
    fn results_are_deterministic_across_thread_counts() {
        let board = synthetic::bga_field(5, 5, 0.8, 0.4, true);
        let rules = UniformRules::default();
        let f = [fp(&board, "BGA1")];
        let one = rayon::ThreadPoolBuilder::new()
            .num_threads(1)
            .build()
            .unwrap()
            .install(|| audit_terminal_access(&board, &rules, &f));
        let many = rayon::ThreadPoolBuilder::new()
            .num_threads(4)
            .build()
            .unwrap()
            .install(|| audit_terminal_access(&board, &rules, &f));
        let strip = |a: &AccessAudit| {
            a.terminals
                .iter()
                .map(|t| (t.pad, t.reachable, t.reason.clone()))
                .collect::<Vec<_>>()
        };
        assert_eq!(strip(&one), strip(&many));
    }
}
