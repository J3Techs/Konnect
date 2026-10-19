//! S4 pin access (owned by WS-B).
//!
//! Exact access points and exit headings for each pad and escape end,
//! precomputed once per (pad shape class, rotation, profile) and placed on
//! the detailed lattice: a point inside the pad copper from which a track of
//! the profile can leave in each listed heading without violating any rule.
//!
//! # Method (after the "Tao of PAO" pin-access flow)
//!
//! 1. **Pad class.** A pad's class is everything its access depends on
//!    inside its own footprint: its copper relative to its anchor (so the
//!    rotation is part of it), the layer, the track width, and the other
//!    pads of the footprint within reach — their copper relative to the
//!    same anchor, whether they share the pad's net, and the clearance the
//!    rules resolve between them and the track. Identical footprints at the
//!    same rotation fall into the same classes.
//! 2. **Class access, once per class.** Candidate points are lattice points
//!    on the two lattice lines through the anchor (the lattice is
//!    pad-aligned, so they stay lattice points wherever the footprint is
//!    placed) that lie in the pad copper, nearest the anchor first. A
//!    heading is open when a track of the profile's width from the point,
//!    long enough to leave the pad by a track width plus the largest
//!    clearance, keeps the resolved clearance to every class neighbour of
//!    another net.
//! 3. **Instance check.** Each pad's class points are moved to the pad and
//!    every open heading is re-checked exactly against the whole board
//!    (other footprints, fixed tracks and vias, holes, the board edge and
//!    keepouts), with the subject the rules see there.
//! 4. **Escape ends.** When a pad's copper already continues as an escape
//!    (a locked neck or dog-bone), the escape's free ends — via centres on
//!    every routable layer, and track ends away from the pads — are access
//!    points too, listed first: a route should attach there.
//!
//! # Profiles
//!
//! There is no profile table yet. Until the integrator publishes one,
//! `ProfileId(n)` means net class `n`: its track width, never below the
//! rules' floor for the pad's net. Clearance is always resolved for the
//! pad's own net, so custom rules and courtyard fields apply. A profile
//! that names neither the default class nor the class of any net on the
//! board is refused: every pad comes back without points and
//! [`AccessStats::unknown_profile`] is set.

use crate::api::{
    ItemKind, ItemRef, LayerId, PadId, ProfileId, LATTICE_FINE_PITCH_NM, LATTICE_PITCH_NM,
};
use crate::board::{Pad, RouteBoard};
use crate::connectivity::ConnectivityReport;
use crate::rules::{RuleSubject, RuleTable};
use crate::space::{radius_of, ClearanceCache, FxMap, Space, SubjectKey};
use konnect_geom::{
    gap_cmp, touches, ArcPolicy, BBox, Circle, ConvexPoly, Coord, Point, Polygon, Segment, Shape,
};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::cmp::Ordering;

/// One way into a pad.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AccessPoint {
    pub point: Point,
    pub layer: LayerId,
    /// Bit `h`: a track may leave in heading `h` (0 = +x, 45° steps,
    /// counter-clockwise in the Y-up sense) — the same encoding as
    /// `TileRaster::heading_mask`.
    pub headings: u8,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PadAccess {
    pub pad: PadId,
    pub profile: ProfileId,
    /// Best first.
    pub points: Vec<AccessPoint>,
}

/// Tuning for [`compute_pin_access_with`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct AccessSettings {
    /// Lattice pitch for pads at least `fine_below` in both directions.
    pub pitch: Coord,
    /// Pitch for smaller pads.
    pub fine_pitch: Coord,
    pub fine_below: Coord,
    /// Most points kept per pad (escape ends not counted).
    pub max_points: usize,
    /// Include the free ends of escape copper.
    pub escape_ends: bool,
}

impl Default for AccessSettings {
    fn default() -> Self {
        AccessSettings {
            pitch: LATTICE_PITCH_NM,
            fine_pitch: LATTICE_FINE_PITCH_NM,
            fine_below: 500_000,
            max_points: 5,
            escape_ends: true,
        }
    }
}

/// Counters for one run.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct AccessStats {
    pub pads: usize,
    /// Distinct pad classes (pad, layer, width, neighbourhood).
    pub classes: usize,
    pub class_checks: usize,
    pub instance_checks: usize,
    /// Pads left with no access point at all.
    pub no_access: usize,
    pub escape_points: usize,
    pub wall_ms: u64,
    /// The profile names no net class the board uses; no pad was solved.
    #[serde(default)]
    pub unknown_profile: bool,
}

/// Whether `ProfileId(n)` names a class that is safe to look up: the
/// default class or the class of some net on the board. (The rule table
/// exposes no class count, and `class()` does not bound-check.)
fn profile_is_known(board: &RouteBoard, rules: &dyn RuleTable, profile: ProfileId) -> bool {
    let wanted = crate::api::ClassId(profile.0);
    rules.class_of(None) == wanted
        || (0..board.nets.len())
            .any(|n| rules.class_of(Some(crate::api::NetId(n as u32))) == wanted)
}

/// The eight headings as unit vectors in the board's Y-down frame
/// (heading 2 is +90° in the Y-up sense, so −y here).
const HEADINGS: [(f64, f64); 8] = {
    const D: f64 = std::f64::consts::FRAC_1_SQRT_2;
    [
        (1.0, 0.0),
        (D, -D),
        (0.0, -1.0),
        (-D, -D),
        (-1.0, 0.0),
        (-D, D),
        (0.0, 1.0),
        (D, D),
    ]
};

/// Access points for `pads` under `profile`.
pub fn compute_pin_access(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    pads: &[PadId],
    profile: ProfileId,
) -> Vec<PadAccess> {
    compute_pin_access_with(board, rules, pads, profile, &AccessSettings::default()).0
}

/// [`compute_pin_access`] with explicit settings, and its counters.
pub fn compute_pin_access_with(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    pads: &[PadId],
    profile: ProfileId,
    settings: &AccessSettings,
) -> (Vec<PadAccess>, AccessStats) {
    let start = std::time::Instant::now();
    if !profile_is_known(board, rules, profile) {
        // `RuleTable::class` has no bounds check; an unknown class would
        // panic there. Report every pad without points instead.
        let stats = AccessStats {
            pads: pads.len(),
            no_access: pads.len(),
            unknown_profile: true,
            wall_ms: start.elapsed().as_millis() as u64,
            ..Default::default()
        };
        let out = pads
            .iter()
            .map(|&p| PadAccess {
                pad: p,
                profile,
                points: Vec::new(),
            })
            .collect();
        return (out, stats);
    }
    let policy = ArcPolicy::default();
    let space = Space::new(board, rules, &policy, false);
    let conn = settings
        .escape_ends
        .then(|| ConnectivityReport::build(board, &policy));
    let islands = conn.as_ref().map(|c| c.island_of_pads(board));
    let ctx = Ctx {
        space: &space,
        rules,
        settings,
        profile,
    };

    // 1. Class keys, per pad and layer.
    let keyed: Vec<Vec<(LayerId, Option<ClassKey>, Coord)>> = pads
        .par_iter()
        .map_init(ClearanceCache::default, |cache, &p| {
            pad_layers(&ctx, p)
                .into_iter()
                .map(|(layer, width)| (layer, class_key(&ctx, cache, p, layer, width), width))
                .collect()
        })
        .collect();
    let mut keys: Vec<&ClassKey> = keyed
        .iter()
        .flatten()
        .filter_map(|(_, k, _)| k.as_ref())
        .collect();
    // Deduplicate by value: hash, then sort each bucket by first use.
    let mut index: FxMap<&ClassKey, usize> = FxMap::default();
    let mut unique: Vec<&ClassKey> = Vec::new();
    for k in keys.drain(..) {
        index.entry(k).or_insert_with(|| {
            unique.push(k);
            unique.len() - 1
        });
    }

    // 2. Class access, once per class.
    let solved: Vec<(Vec<(Point, u8)>, usize)> =
        unique.par_iter().map(|k| class_access(&ctx, k)).collect();
    let class_checks: usize = solved.iter().map(|s| s.1).sum();
    let class_points: Vec<Vec<(Point, u8)>> = solved.into_iter().map(|s| s.0).collect();

    // 3. Instances, and 4. escape ends.
    let results: Vec<(PadAccess, usize, usize)> = pads
        .par_iter()
        .zip(keyed.par_iter())
        .map_init(ClearanceCache::default, |cache, (&p, layers)| {
            let pad = &board.pads[p.index()];
            let mut points: Vec<AccessPoint> = Vec::new();
            let mut checks = 0usize;
            let mut escapes = 0usize;
            if let (Some(conn), Some(islands)) = (&conn, &islands) {
                let found = escape_ends(&ctx, cache, conn, islands, p, &mut checks);
                escapes = found.len();
                points.extend(found);
            }
            let mut own: Vec<AccessPoint> = Vec::new();
            for (layer, key, width) in layers {
                let Some(key) = key else { continue };
                let cls = &class_points[index[key]];
                for &(rel, mask) in cls {
                    let pt = Point::new(pad.center.x + rel.x, pad.center.y + rel.y);
                    let mut open = 0u8;
                    for h in 0..8 {
                        if mask & (1 << h) == 0 {
                            continue;
                        }
                        checks += 1;
                        if exit_ok(&ctx, cache, pad, *layer, pt, h, *width, key.reach) {
                            open |= 1 << h;
                        }
                    }
                    if open != 0 {
                        own.push(AccessPoint {
                            point: pt,
                            layer: *layer,
                            headings: open,
                        });
                    }
                }
            }
            rank(&mut own, pad.center);
            own.truncate(settings.max_points.max(1));
            points.extend(own);
            (
                PadAccess {
                    pad: p,
                    profile,
                    points,
                },
                checks,
                escapes,
            )
        })
        .collect();

    let mut stats = AccessStats {
        pads: pads.len(),
        classes: unique.len(),
        class_checks,
        ..Default::default()
    };
    let mut out = Vec::with_capacity(results.len());
    for (a, checks, escapes) in results {
        stats.instance_checks += checks;
        stats.escape_points += escapes;
        stats.no_access += usize::from(a.points.is_empty());
        out.push(a);
    }
    stats.wall_ms = start.elapsed().as_millis() as u64;
    (out, stats)
}

struct Ctx<'a> {
    space: &'a Space<'a>,
    rules: &'a dyn RuleTable,
    settings: &'a AccessSettings,
    profile: ProfileId,
}

/// Routable layers of a pad and the profile's width on each.
fn pad_layers(ctx: &Ctx<'_>, p: PadId) -> Vec<(LayerId, Coord)> {
    let board = ctx.space.board;
    let pad = &board.pads[p.index()];
    if pad.net.is_none() {
        return Vec::new();
    }
    let class = crate::api::ClassId(ctx.profile.0);
    let class_width = ctx.rules.class(class).track_width;
    let subject = RuleSubject::new(ItemKind::Track, pad.net);
    pad.layers
        .intersection(board.routable)
        .iter()
        .map(|l| {
            let floor = ctx.rules.track_width(&subject, l).min;
            (l, class_width.max(floor).max(1))
        })
        .collect()
}

fn translate(s: &Shape, dx: Coord, dy: Coord) -> Option<Shape> {
    let t = |p: Point| Point::new(p.x - dx, p.y - dy);
    Some(match s {
        Shape::Segment(g) => Shape::Segment(Segment::new(t(g.a), t(g.b), g.width)),
        Shape::Circle(c) => Shape::Circle(Circle::new(t(c.center), c.radius)),
        Shape::ConvexPoly(p) => Shape::ConvexPoly(ConvexPoly::new(
            p.points().iter().map(|&q| t(q)).collect(),
            p.radius(),
        )?),
        Shape::Polygon(p) => Shape::Polygon(Polygon::new(
            p.outline().iter().map(|&q| t(q)).collect(),
            p.holes()
                .iter()
                .map(|h| h.iter().map(|&q| t(q)).collect())
                .collect(),
        )?),
    })
}

/// Everything a pad's class access depends on, relative to its anchor.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
struct ClassKey {
    layer: LayerId,
    width: Coord,
    pitch: Coord,
    shapes: Vec<Shape>,
    /// Other pads of the footprint within reach: shapes, same net, the
    /// resolved clearance to the track.
    neighbours: Vec<(Vec<Shape>, bool, Coord)>,
    /// Track length beyond the pad edge, nm.
    reach: Coord,
}

fn class_key(
    ctx: &Ctx<'_>,
    cache: &mut ClearanceCache,
    p: PadId,
    layer: LayerId,
    width: Coord,
) -> Option<ClassKey> {
    let space = ctx.space;
    let board = space.board;
    let pad = &board.pads[p.index()];
    let fp = &board.footprints[pad.footprint.index()];
    let (dx, dy) = (pad.center.x, pad.center.y);
    let shapes: Vec<Shape> = pad
        .shapes
        .iter()
        .map(|s| translate(s, dx, dy))
        .collect::<Option<_>>()?;
    let bb = pad.bbox()?;
    let small = bb.width().min(bb.height()) < i64::from(ctx.settings.fine_below);
    let pitch = if small {
        ctx.settings.fine_pitch
    } else {
        ctx.settings.pitch
    };
    let dot = Shape::Circle(Circle::new(pad.center, radius_of(width)));
    let track_key = space.subject_for(ItemKind::Track, pad.net, &dot);
    let reach_box = bb.inflate(2 * (width + space.reach));
    let mut neighbours = Vec::new();
    let mut max_clr = 0;
    for q in fp.pad_range.0..fp.pad_range.1 {
        let other = &board.pads[q as usize];
        if q == p.0 || !other.layers.contains(layer) {
            continue;
        }
        let Some(ob) = other.bbox() else { continue };
        if !ob.intersects(&reach_box) {
            continue;
        }
        let same = other.net.is_some() && other.net == pad.net;
        let clr = if same {
            0
        } else {
            let mut key = SubjectKey::new(ItemKind::Pad, other.net);
            key.footprint = Some(other.footprint);
            key.local = other.local_clearance;
            for s in &other.shapes {
                space.context_into(&mut key, s);
            }
            space.clearance(cache, ctx.rules, &track_key, &key, layer)
        };
        max_clr = max_clr.max(clr);
        let rel: Vec<Shape> = other
            .shapes
            .iter()
            .map(|s| translate(s, dx, dy))
            .collect::<Option<_>>()?;
        neighbours.push((rel, same, clr));
    }
    // Deterministic order regardless of pad order in the footprint.
    // An integer key settles almost every pair; the Debug text breaks the
    // rare tie of two neighbours with the same box, kind and clearance.
    let int_key = |n: &(Vec<Shape>, bool, Coord)| {
        let bb = n.0.iter().map(Shape::bbox).reduce(BBox::union);
        (
            bb.map(|b| (b.min.x, b.min.y, b.max.x, b.max.y)),
            n.1,
            n.2,
            n.0.len(),
        )
    };
    neighbours.sort_by(|a, b| {
        int_key(a)
            .cmp(&int_key(b))
            .then_with(|| format!("{a:?}").cmp(&format!("{b:?}")))
    });
    Some(ClassKey {
        layer,
        width,
        pitch,
        shapes,
        neighbours,
        reach: width + max_clr.max(space.reach.min(1_000_000)),
    })
}

/// How far from `p` along unit `(ux, uy)` the shapes extend, nm (0 when
/// `p` is outside them).
fn exit_distance(shapes: &[Shape], p: Point, ux: f64, uy: f64) -> i64 {
    let inside = |t: f64| {
        let q = Point::new(
            (f64::from(p.x) + ux * t).round() as Coord,
            (f64::from(p.y) + uy * t).round() as Coord,
        );
        let dot = Shape::Circle(Circle::new(q, 0));
        shapes.iter().any(|s| touches(&dot, s))
    };
    if !inside(0.0) {
        return 0;
    }
    let Some(bb) = shapes.iter().map(Shape::bbox).reduce(BBox::union) else {
        return 0;
    };
    let (mut lo, mut hi) = (0.0, (bb.width() + bb.height()) as f64 + 2.0);
    while hi - lo > 1.0 {
        let mid = (lo + hi) / 2.0;
        if inside(mid) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    hi.ceil() as i64
}

fn heading_segment(shapes: &[Shape], p: Point, h: usize, width: Coord, reach: Coord) -> Segment {
    let (ux, uy) = HEADINGS[h];
    let len = (exit_distance(shapes, p, ux, uy) + i64::from(reach)) as f64;
    let end = Point::new(
        (f64::from(p.x) + ux * len).round() as Coord,
        (f64::from(p.y) + uy * len).round() as Coord,
    );
    Segment::new(p, end, width)
}

/// Solve one class in the anchor frame.
fn class_access(ctx: &Ctx<'_>, k: &ClassKey) -> (Vec<(Point, u8)>, usize) {
    let shapes = &k.shapes;
    let Some(bb) = shapes.iter().map(Shape::bbox).reduce(BBox::union) else {
        return (Vec::new(), 0);
    };
    let pitch = i64::from(k.pitch.max(1));
    let inside = |p: Point| {
        let dot = Shape::Circle(Circle::new(p, 0));
        shapes.iter().any(|s| touches(&dot, s))
    };
    // Lattice points on the two lines through the anchor, inside the pad.
    let mut pts: Vec<Point> = Vec::new();
    let (x0, x1) = (i64::from(bb.min.x) / pitch, i64::from(bb.max.x) / pitch);
    let (y0, y1) = (i64::from(bb.min.y) / pitch, i64::from(bb.max.y) / pitch);
    for i in x0..=x1 {
        pts.push(Point::new((i * pitch) as Coord, 0));
    }
    for j in y0..=y1 {
        if j != 0 {
            pts.push(Point::new(0, (j * pitch) as Coord));
        }
    }
    pts.retain(|&p| inside(p));
    if pts.is_empty() {
        // Copper offset from its anchor on both axes: use the lattice lines
        // through the lattice point nearest the copper's centre instead.
        let c = bb.center();
        let snap = |v: Coord| ((f64::from(v) / pitch as f64).round() as i64 * pitch) as Coord;
        let (cx, cy) = (snap(c.x), snap(c.y));
        for i in x0..=x1 {
            pts.push(Point::new((i * pitch) as Coord, cy));
        }
        for j in y0..=y1 {
            if (j * pitch) as Coord != cy {
                pts.push(Point::new(cx, (j * pitch) as Coord));
            }
        }
        pts.retain(|&p| inside(p));
    }
    pts.sort_by_key(|p| (i64::from(p.x).abs() + i64::from(p.y).abs(), p.x, p.y));
    // Solve more than we keep: the instance check may close some.
    pts.truncate(4 * ctx.settings.max_points.max(1));
    let mut out = Vec::new();
    let checks = 8 * pts.len();
    for p in pts {
        let mut mask = 0u8;
        for h in 0..8 {
            let seg = Shape::Segment(heading_segment(shapes, p, h, k.width, k.reach));
            let blocked = k.neighbours.iter().any(|(ns, same, clr)| {
                !same
                    && ns
                        .iter()
                        .any(|s| gap_cmp(&seg, s, i64::from(*clr)) == Ordering::Less)
            });
            if !blocked {
                mask |= 1 << h;
            }
        }
        if mask != 0 {
            out.push((p, mask));
        }
    }
    (out, checks)
}

/// The exact board check of one heading from one point.
#[allow(clippy::too_many_arguments)]
fn exit_ok(
    ctx: &Ctx<'_>,
    cache: &mut ClearanceCache,
    pad: &Pad,
    layer: LayerId,
    p: Point,
    h: usize,
    width: Coord,
    reach: Coord,
) -> bool {
    let seg = heading_segment(&pad.shapes, p, h, width, reach);
    let shape = Shape::Segment(seg);
    let key = ctx.space.subject_for(ItemKind::Track, pad.net, &shape);
    ctx.space
        .track_conflict(cache, ctx.rules, layer, seg, pad.net, &key)
        .is_none()
}

/// Best first: most open headings, then nearest the anchor, then position.
fn rank(points: &mut [AccessPoint], anchor: Point) {
    points.sort_by_key(|a| {
        (
            std::cmp::Reverse(a.headings.count_ones()),
            (a.point - anchor).length2(),
            a.layer,
            a.point,
        )
    });
}

/// The free ends of the escape copper joined to pad `p`, with their open
/// headings.
fn escape_ends(
    ctx: &Ctx<'_>,
    cache: &mut ClearanceCache,
    conn: &ConnectivityReport,
    islands: &[Option<u32>],
    p: PadId,
    checks: &mut usize,
) -> Vec<AccessPoint> {
    let space = ctx.space;
    let board = space.board;
    let pad = &board.pads[p.index()];
    let (Some(net), Some(isl)) = (pad.net, islands[p.index()]) else {
        return Vec::new();
    };
    let island = &conn.nets[net.index()].islands[isl as usize];
    if island.items.iter().any(|i| matches!(i, ItemRef::Zone(_))) {
        // Joined to a plane: nothing to route to.
        return Vec::new();
    }
    let island_pads: Vec<&Pad> = island.pads().map(|q| &board.pads[q.index()]).collect();
    let in_pad = |pt: Point, layer: LayerId| {
        let dot = Shape::Circle(Circle::new(pt, 0));
        island_pads
            .iter()
            .any(|q| q.layers.contains(layer) && q.shapes.iter().any(|s| touches(&dot, s)))
    };
    let (width, reach) = pad_layers(ctx, p)
        .first()
        .map(|&(_, w)| (w, w + space.reach.min(1_000_000)))
        .unwrap_or((0, 0));
    if width == 0 {
        return Vec::new();
    }
    // A track end is free only when no other track of the island ends
    // there and no via of the island covers it (a joint, not an end).
    let mut track_ends: Vec<(LayerId, Point)> = Vec::new();
    let mut island_vias: Vec<&crate::board::Via> = Vec::new();
    for item in &island.items {
        match *item {
            ItemRef::Track(t) => {
                let tr = &board.tracks[t.index()];
                track_ends.push((tr.layer, tr.seg.a));
                track_ends.push((tr.layer, tr.seg.b));
            }
            ItemRef::Via(v) => island_vias.push(&board.vias[v.index()]),
            _ => {}
        }
    }
    track_ends.sort_unstable();
    let joint = |layer: LayerId, e: Point| {
        let lo = track_ends.partition_point(|&k| k < (layer, e));
        let hi = track_ends.partition_point(|&k| k <= (layer, e));
        let dot = Shape::Circle(Circle::new(e, 0));
        hi - lo > 1
            || island_vias
                .iter()
                .any(|v| v.layers.contains(layer) && touches(&dot, &v.shape()))
    };
    let mut ends: Vec<(LayerId, Point, Coord)> = Vec::new();
    for item in &island.items {
        match *item {
            ItemRef::Via(v) => {
                let via = &board.vias[v.index()];
                for l in via.layers.intersection(board.routable).iter() {
                    ends.push((l, via.center, radius_of(via.diameter)));
                }
            }
            ItemRef::Track(t) => {
                let tr = &board.tracks[t.index()];
                if !board.routable.contains(tr.layer) {
                    continue;
                }
                for e in [tr.seg.a, tr.seg.b] {
                    if !in_pad(e, tr.layer) && !joint(tr.layer, e) {
                        ends.push((tr.layer, e, radius_of(tr.seg.width)));
                    }
                }
            }
            _ => {}
        }
    }
    ends.sort_by_key(|&(l, p, _)| (l, p));
    ends.dedup_by_key(|&mut (l, p, _)| (l, p));
    let mut out = Vec::new();
    for (layer, pt, r) in ends {
        let mut open = 0u8;
        for (h, &(ux, uy)) in HEADINGS.iter().enumerate() {
            *checks += 1;
            let len = f64::from(r + reach);
            let end = Point::new(
                (f64::from(pt.x) + ux * len).round() as Coord,
                (f64::from(pt.y) + uy * len).round() as Coord,
            );
            let seg = Segment::new(pt, end, width);
            let key = space.subject_for(ItemKind::Track, Some(net), &Shape::Segment(seg));
            if space
                .track_conflict(cache, ctx.rules, layer, seg, Some(net), &key)
                .is_none()
            {
                open |= 1 << h;
            }
        }
        if open != 0 {
            out.push(AccessPoint {
                point: pt,
                layer,
                headings: open,
            });
        }
    }
    rank(&mut out, pad.center);
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synthetic::{self, BoardBuilder, UniformRules};

    fn all_pads(board: &RouteBoard) -> Vec<PadId> {
        board.pads.iter().map(|p| p.id).collect()
    }

    #[test]
    fn an_isolated_pad_is_open_in_every_heading() {
        let mut b = BoardBuilder::new(10.0, 10.0, &["F.Cu", "B.Cu"]);
        let f = b.footprint("R1", (5.0, 5.0));
        b.smd_pad(f, "1", "A", (5.0, 5.0), (0.6, 0.6), "F.Cu");
        let board = b.build();
        let rules = UniformRules::default();
        let out = compute_pin_access(&board, &rules, &all_pads(&board), ProfileId(0));
        assert_eq!(out.len(), 1);
        let best = &out[0].points[0];
        assert_eq!(best.point, board.pads[0].center);
        assert_eq!(best.headings, 0xff);
        // Every point lies on the pad-aligned lattice through the anchor.
        for ap in &out[0].points {
            let d = ap.point - board.pads[0].center;
            assert!(d.x == 0 || d.y == 0);
            assert_eq!(d.x % i64::from(LATTICE_PITCH_NM), 0);
            assert_eq!(d.y % i64::from(LATTICE_PITCH_NM), 0);
        }
    }

    #[test]
    fn a_neighbour_pad_closes_the_headings_towards_it() {
        // Two pads 0.8 mm apart centre to centre (0.2 mm gap): with 0.2 mm
        // clearance and 0.15 mm tracks nothing passes between them.
        let mut b = BoardBuilder::new(10.0, 10.0, &["F.Cu", "B.Cu"]);
        let f = b.footprint("R1", (5.0, 5.0));
        b.smd_pad(f, "1", "A", (4.6, 5.0), (0.6, 0.6), "F.Cu");
        b.smd_pad(f, "2", "B", (5.4, 5.0), (0.6, 0.6), "F.Cu");
        let board = b.build();
        let rules = UniformRules::default();
        let out = compute_pin_access(&board, &rules, &[PadId(0)], ProfileId(0));
        let pts = &out[0].points;
        assert!(!pts.is_empty());
        for ap in pts {
            // +x (heading 0) points at the neighbour.
            assert_eq!(ap.headings & 1, 0, "{ap:?}");
            // -x (heading 4) is open.
            assert_ne!(ap.headings & (1 << 4), 0, "{ap:?}");
        }
    }

    #[test]
    fn classes_are_shared_between_identical_footprints() {
        let board = synthetic::two_pin_nets(6, false);
        let rules = UniformRules::default();
        let (out, stats) = compute_pin_access_with(
            &board,
            &rules,
            &all_pads(&board),
            ProfileId(0),
            &AccessSettings::default(),
        );
        assert_eq!(out.len(), 12);
        // Every pad sits alone in a big footprint, all with one shape: one
        // class.
        assert_eq!(stats.classes, 1, "{stats:?}");
        assert_eq!(stats.no_access, 0);
    }

    #[test]
    fn escape_ends_come_first() {
        // A BGA with locked dog-bones: each ball's via is an access point
        // on every routable layer, ahead of the ball itself.
        let board = synthetic::bga_field(2, 2, 1.0, 0.4, true);
        let rules = UniformRules::default();
        let out = compute_pin_access(&board, &rules, &all_pads(&board), ProfileId(0));
        for a in &out {
            let first = &a.points[0];
            let via = board
                .vias
                .iter()
                .find(|v| v.net == board.pads[a.pad.index()].net)
                .unwrap();
            assert_eq!(first.point, via.center);
        }
    }

    /// One class, looked up the way a project table does: by index, with
    /// no bounds check.
    struct OneClass(UniformRules);

    impl RuleTable for OneClass {
        fn class_of(&self, _net: Option<crate::api::NetId>) -> crate::api::ClassId {
            crate::api::ClassId(0)
        }
        fn class(&self, class: crate::api::ClassId) -> &crate::rules::NetClassRules {
            &std::slice::from_ref(&self.0.class)[class.index()]
        }
        fn clearance(&self, a: &RuleSubject<'_>, b: &RuleSubject<'_>, l: LayerId) -> Coord {
            self.0.clearance(a, b, l)
        }
        fn track_width(&self, s: &RuleSubject<'_>, l: LayerId) -> crate::rules::WidthRule {
            self.0.track_width(s, l)
        }
        fn via(&self, s: &RuleSubject<'_>) -> crate::rules::ViaRule {
            self.0.via(s)
        }
        fn min_track_angle_deg(&self, s: &RuleSubject<'_>, l: LayerId) -> Option<f64> {
            self.0.min_track_angle_deg(s, l)
        }
        fn min_segment_length(&self, s: &RuleSubject<'_>, l: LayerId) -> Option<Coord> {
            self.0.min_segment_length(s, l)
        }
        fn edge_clearance(&self) -> Coord {
            self.0.edge_clearance()
        }
        fn hole_clearance(&self) -> Coord {
            self.0.hole_clearance()
        }
        fn hole_to_hole(&self) -> Coord {
            self.0.hole_to_hole()
        }
        fn max_clearance(&self) -> Coord {
            self.0.max_clearance()
        }
        fn inert_rules(&self) -> &[crate::rules::InertRule] {
            &[]
        }
    }

    #[test]
    fn an_unknown_profile_is_refused_without_panicking() {
        let board = synthetic::two_pin_nets(2, false);
        let rules = OneClass(UniformRules::default());
        let pads = all_pads(&board);
        let (out, stats) = compute_pin_access_with(
            &board,
            &rules,
            &pads,
            ProfileId(7),
            &AccessSettings::default(),
        );
        assert!(stats.unknown_profile);
        assert_eq!(out.len(), pads.len());
        assert!(out.iter().all(|a| a.points.is_empty()));
        assert_eq!(stats.no_access, pads.len());
        // The known class still works.
        let (ok, stats) = compute_pin_access_with(
            &board,
            &rules,
            &pads,
            ProfileId(0),
            &AccessSettings::default(),
        );
        assert!(!stats.unknown_profile);
        assert!(ok.iter().all(|a| !a.points.is_empty()));
    }

    #[test]
    fn copper_offset_from_its_anchor_still_gets_points() {
        // The pad's anchor sits 0.4 mm off its copper on both axes, so
        // neither lattice line through the anchor crosses the copper.
        let mut b = BoardBuilder::new(10.0, 10.0, &["F.Cu", "B.Cu"]);
        let f = b.footprint("R1", (5.0, 5.0));
        b.smd_pad(f, "1", "A", (5.0, 5.0), (0.3, 0.3), "F.Cu");
        let mut board = b.build();
        board.pads[0].center = Point::new(5_400_000, 5_400_000);
        let rules = UniformRules::default();
        let out = compute_pin_access(&board, &rules, &[PadId(0)], ProfileId(0));
        assert!(!out[0].points.is_empty());
        for ap in &out[0].points {
            let dot = Shape::Circle(Circle::new(ap.point, 0));
            assert!(board.pads[0].shapes.iter().any(|s| touches(&dot, s)));
            let d = ap.point - board.pads[0].center;
            assert_eq!(d.x % i64::from(LATTICE_FINE_PITCH_NM), 0);
            assert_eq!(d.y % i64::from(LATTICE_FINE_PITCH_NM), 0);
        }
    }

    #[test]
    fn a_joint_between_neck_segments_is_not_an_escape_end() {
        // A pad with a two-segment neck: only the far end is free.
        let mut b = BoardBuilder::new(10.0, 10.0, &["F.Cu", "B.Cu"]);
        let f = b.footprint("R1", (3.0, 5.0));
        b.smd_pad(f, "1", "A", (3.0, 5.0), (0.6, 0.6), "F.Cu");
        b.smd_pad(f, "2", "A", (8.0, 5.0), (0.6, 0.6), "F.Cu");
        b.track("A", "F.Cu", (3.0, 5.0), (4.0, 5.0), 0.15, true)
            .track("A", "F.Cu", (4.0, 5.0), (5.0, 6.0), 0.15, true);
        let board = b.build();
        let rules = UniformRules::default();
        let out = compute_pin_access(&board, &rules, &[PadId(0)], ProfileId(0));
        let far = Point::new(5_000_000, 6_000_000);
        let joint = Point::new(4_000_000, 5_000_000);
        let pts: Vec<Point> = out[0].points.iter().map(|a| a.point).collect();
        assert_eq!(pts[0], far, "{pts:?}");
        assert!(!pts.contains(&joint), "{pts:?}");
    }

    #[test]
    fn results_do_not_depend_on_thread_count() {
        let board = synthetic::bga_field(4, 4, 0.8, 0.4, true);
        let rules = UniformRules::default();
        let pads = all_pads(&board);
        let run = |n| {
            rayon::ThreadPoolBuilder::new()
                .num_threads(n)
                .build()
                .unwrap()
                .install(|| compute_pin_access(&board, &rules, &pads, ProfileId(0)))
        };
        assert_eq!(run(1), run(4));
    }
}
