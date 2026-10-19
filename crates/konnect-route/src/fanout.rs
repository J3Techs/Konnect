//! S2 plane fanout (owned by WS-B).
//!
//! For every SMD pad on a plane net that is not yet joined to its plane,
//! choose a via site and a stub: candidates are 8 octilinear exits × a few
//! distances from the pad edge plus corner exits; each is checked exactly
//! (annulus and clearance on every layer, hole-to-hole, hole and edge
//! clearance, the via floor) and must lie inside its own net's plane piece,
//! shrunk by the margin, and outside every other net's island on each plane
//! layer the via crosses. A greedy weighted independent set per tile picks
//! non-conflicting sites. The plan is applied locked, in its own commit.
//!
//! # Units
//!
//! The unit of work is an unjoined *island*, not a pad: pads already tied
//! together by copper (duplicate-number pads, overlapping power pads) need
//! one via between them. Candidates come from every SMD pad of the island.
//! Islands that already carry a track, arc or via are escaped and skipped
//! (they belong to a deliberate escape, or to a route that still has to
//! reach the plane), as are islands on footprints the caller excludes.
//!
//! # Legality
//!
//! A candidate is a through via of the net's size (the net class via, then
//! the rule floor, per [`ViaSizePolicy`]) plus a straight stub on the pad's
//! layer from the pad's copper centre to the via centre. Against the board
//! it must pass, exactly:
//!
//! - annulus clearance to every other net's copper on every layer, with the
//!   rule table's resolved clearance (custom rules and courtyard fields
//!   included); the same for the stub on its layer;
//! - hole-to-hole to every hole, hole clearance both ways, copper-to-edge;
//! - via and track keepouts;
//! - the via must not overlap any SMD pad (no accidental via-in-pad);
//! - on every plane layer that carries the net, the via disc grown by
//!   `plane_margin` lies inside one of the net's filled pieces, and at least
//!   `min_spokes` of four spoke directions reach plane copper;
//! - on every layer, the grown disc stays outside other nets' islands
//!   (higher-priority or flagged pieces: the small pours inside a plane).
//!
//! Plane fills are not obstacles: a refill pours around the new via. Between
//! candidates of different islands the same clearances and hole-to-hole
//! decide conflicts.
//!
//! # Selection
//!
//! Islands are bucketed into tiles of [`FanoutSettings::tile`]; tiles are
//! four-coloured by parity, and tiles of one colour, which are at least a
//! tile apart and so cannot interact, are solved in parallel. Colours run
//! in order, each seeing every site committed before it. Inside a tile the
//! most constrained island goes first and takes its cheapest compatible
//! candidate; then a repair pass lets a failed island displace one chosen
//! site whose island can move to another candidate. Output is sorted by
//! pad, so 1-thread and N-thread runs give the same plan.

use crate::api::{FootprintId, Generation, ItemKind, ItemRef, LayerId, NetId, PadId, TILE_CORE_NM};
use crate::board::{PadKind, RouteBoard};
use crate::connectivity::ConnectivityReport;
use crate::plan::{PlanError, RoutePlan, RoutedCopper, RoutedTrack, RoutedVia};
use crate::rules::{RuleSubject, RuleTable};
use crate::space::{radius_of, ClearanceCache, Conflict, Space, SubjectKey};
use konnect_geom::{gap_cmp, touches, ArcPolicy, BBox, Circle, Coord, Point, Segment, Shape};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::cmp::Ordering;
use std::collections::BTreeMap;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct FanoutSettings {
    /// Shrink applied to plane pieces before the via-inside-plane test, nm.
    pub plane_margin: Coord,
    /// Spokes a thermal relief must keep (`min_resolved_spokes`).
    pub min_spokes: u8,
    /// Allow pads of one net closer than this to share a via (0: never).
    pub share_within: Coord,
    /// Which via sizes to try.
    #[serde(default)]
    pub via_size: ViaSizePolicy,
    /// Which stub widths to try.
    #[serde(default)]
    pub stub_width: StubWidthPolicy,
    /// Gaps between the pad edge and the via edge to try, nm, nearest first.
    #[serde(default = "default_gaps")]
    pub exit_gaps: [Coord; 3],
    /// Gaps for a second pass, nm, used only for islands the first pass
    /// found no legal site for (0 entries are skipped).
    #[serde(default = "default_fallback_gaps")]
    pub fallback_gaps: [Coord; 3],
    /// Evenly spaced exit directions in the second pass (0: no second pass).
    #[serde(default = "default_fallback_directions")]
    pub fallback_directions: u8,
    /// Also try exits through the corners of polygonal pads.
    #[serde(default = "yes")]
    pub corner_exits: bool,
    /// Tile size for the parallel selection, nm.
    #[serde(default = "default_tile")]
    pub tile: (Coord, Coord),
    /// Keep vias out of other nets' islands (the small pours inside a
    /// plane) on every layer. Off, a via may perforate them; the refill
    /// clears around it.
    #[serde(default = "yes")]
    pub avoid_other_islands: bool,
    /// Skip islands that already carry a track, arc or via.
    #[serde(default = "yes")]
    pub skip_escaped: bool,
    /// Round via centres to this grid, nm (1 µm keeps the plan exact).
    #[serde(default = "default_snap")]
    pub snap: Coord,
    /// Leave alone every pad of a ball-grid footprint whose escapes are
    /// drawn by hand: at least [`Self::array_min_pads`] round SMD lands,
    /// some already carrying a track or via ([`escaped_arrays`]). Its plane
    /// balls are joined through those escapes, never by a new via here.
    #[serde(default = "yes")]
    pub exclude_escaped_arrays: bool,
    #[serde(default = "default_array_min_pads")]
    pub array_min_pads: u32,
    /// A third pass for islands the first two found no site for: the via
    /// must lie inside its own net's plane piece (with spokes) on at least
    /// one plane layer of the net instead of on every one; on the others it
    /// perforates whatever is poured there, as any through via does (other
    /// nets' islands are still avoided when [`Self::avoid_other_islands`]).
    #[serde(default = "yes")]
    pub partial_plane_fallback: bool,
}

fn default_array_min_pads() -> u32 {
    64
}

fn default_gaps() -> [Coord; 3] {
    [50_000, 200_000, 450_000]
}

fn default_fallback_gaps() -> [Coord; 3] {
    [700_000, 1_000_000, 1_500_000]
}

fn default_fallback_directions() -> u8 {
    16
}

fn yes() -> bool {
    true
}

fn default_tile() -> (Coord, Coord) {
    TILE_CORE_NM
}

fn default_snap() -> Coord {
    1_000
}

impl Default for FanoutSettings {
    fn default() -> Self {
        FanoutSettings {
            plane_margin: 0,
            min_spokes: 2,
            share_within: 0,
            via_size: ViaSizePolicy::default(),
            stub_width: StubWidthPolicy::default(),
            exit_gaps: default_gaps(),
            fallback_gaps: default_fallback_gaps(),
            fallback_directions: default_fallback_directions(),
            corner_exits: true,
            avoid_other_islands: true,
            tile: default_tile(),
            skip_escaped: true,
            snap: default_snap(),
            exclude_escaped_arrays: true,
            array_min_pads: default_array_min_pads(),
            partial_plane_fallback: true,
        }
    }
}

/// Via sizes a candidate may use.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ViaSizePolicy {
    /// The net class via (or a rule's `opt`) only.
    NetClass,
    /// The rule floor only (`via_diameter`/`hole_size` minimums).
    Floor,
    /// The net class via where it fits, else the floor.
    #[default]
    NetClassThenFloor,
}

/// Stub widths a candidate may use.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StubWidthPolicy {
    Preferred,
    Minimum,
    #[default]
    PreferredThenMinimum,
}

/// One planned plane via and the stub joining it to its pad.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PlaneViaSite {
    pub pad: PadId,
    pub net: NetId,
    pub via: Point,
    pub via_diameter: Coord,
    pub via_drill: Coord,
    /// Stub from the pad's exit point to the via, on the pad's layer.
    pub stub: Option<(Point, Point, Coord)>,
    pub layer: LayerId,
    /// The via belongs to another site of the same net (via sharing,
    /// [`FanoutSettings::share_within`]); only the stub is new copper.
    #[serde(default)]
    pub shared: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "reason", rename_all = "snake_case")]
pub enum FanoutFailure {
    /// Every candidate breaks a rule against existing copper, holes or the
    /// board edge.
    NoLegalSite,
    /// Every candidate that clears existing copper falls outside the net's
    /// own plane (or into another net's island).
    OutsideOwnPlane,
    /// Legal candidates existed, but each conflicts with a chosen site.
    Conflict,
}

/// Why an island was not planned at all.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "reason", rename_all = "snake_case")]
pub enum FanoutSkip {
    /// The island already carries a track, arc or via.
    Escaped,
    /// The pad's footprint was excluded by the caller.
    ExcludedFootprint,
    /// Another pad of the same island got the via.
    JoinedThroughIsland { via_pad: PadId },
    /// No SMD pad in the island (through-hole pads join planes by fill).
    NotSmd,
}

/// One rejected candidate: where its via would have gone, and why not.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Trial {
    pub pad: PadId,
    pub via: Point,
    pub reason: String,
}

/// Rejection counts for one failed pad, by reason.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct FailureDetail {
    pub pad: PadId,
    pub net: NetId,
    pub candidates: u32,
    pub legal: u32,
    pub rejections: BTreeMap<String, u32>,
    /// Every rejected candidate of the pad's island.
    #[serde(default)]
    pub trials: Vec<Trial>,
}

/// Counters for one run.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct FanoutStats {
    pub plane_nets: usize,
    pub unjoined_islands: usize,
    pub eligible_islands: usize,
    pub eligible_pads: usize,
    pub candidates: usize,
    pub legal_candidates: usize,
    pub floor_vias: usize,
    pub minimum_stubs: usize,
    pub repairs: usize,
    /// Islands joined by a stub to another island's via.
    pub shared_vias: usize,
    pub tiles: usize,
    /// Islands with far-reaching candidates (large pads), placed serially
    /// after the tiles.
    pub serial_islands: usize,
    pub candidate_ms: u64,
    pub select_ms: u64,
    pub index_ms: u64,
    /// Footprints left alone as hand-escaped ball grids.
    #[serde(default)]
    pub escaped_arrays: usize,
    /// Sites from the partial-plane pass.
    #[serde(default)]
    pub partial_plane_sites: usize,
}

#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct PlaneViaPlan {
    pub sites: Vec<PlaneViaSite>,
    pub failed: Vec<(PadId, FanoutFailure)>,
    /// Pads not planned, with why; sorted by pad.
    #[serde(default)]
    pub skipped: Vec<(PadId, FanoutSkip)>,
    /// Per failed pad, the rejection counts; sorted by pad.
    #[serde(default)]
    pub details: Vec<FailureDetail>,
    #[serde(default)]
    pub stats: FanoutStats,
}

impl PlaneViaPlan {
    /// The planned copper, generation 0. Vias carry
    /// [`LayerMask::EMPTY`](crate::api::LayerMask::EMPTY) here, because a
    /// plan does not hold the board's layer stack; every planned via is a
    /// through via, and [`Self::route_plan`] fills in all copper layers.
    pub fn routed(&self) -> RoutedCopper {
        let mut out = RoutedCopper::default();
        for s in &self.sites {
            if let Some((a, b, w)) = s.stub {
                out.tracks.push(RoutedTrack {
                    net: s.net,
                    layer: s.layer,
                    seg: Segment::new(a, b, w),
                    gen: Generation(0),
                });
            }
            if s.shared {
                continue;
            }
            out.vias.push(RoutedVia {
                net: s.net,
                center: s.via,
                diameter: s.via_diameter,
                drill: s.via_drill,
                layers: crate::api::LayerMask::EMPTY,
                gen: Generation(0),
            });
        }
        out
    }

    /// The `add_routing` file for the sites (apply with `locked: true`).
    pub fn route_plan(&self, board: &RouteBoard) -> Result<RoutePlan, PlanError> {
        let mut copper = self.routed();
        let all = board.layers.all();
        for v in &mut copper.vias {
            v.layers = all;
        }
        RoutePlan::from_routed(board, &copper)
    }

    /// Sites per net, in net order.
    pub fn sites_per_net(&self) -> BTreeMap<NetId, usize> {
        let mut m = BTreeMap::new();
        for s in &self.sites {
            *m.entry(s.net).or_default() += 1;
        }
        m
    }
}

/// Plan plane vias for every unjoined SMD pad on a plane net.
pub fn plan_plane_vias(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    connectivity: &ConnectivityReport,
    settings: &FanoutSettings,
) -> PlaneViaPlan {
    plan_plane_vias_excluding(board, rules, connectivity, settings, &[])
}

/// [`plan_plane_vias`], leaving the pads of `exclude` alone (footprints
/// whose escapes are drawn by hand).
pub fn plan_plane_vias_excluding(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    connectivity: &ConnectivityReport,
    settings: &FanoutSettings,
    exclude: &[FootprintId],
) -> PlaneViaPlan {
    let t0 = std::time::Instant::now();
    let policy = ArcPolicy::default();
    let space = Space::new(board, rules, &policy, true);
    let mut plan = PlaneViaPlan::default();
    plan.stats.index_ms = t0.elapsed().as_millis() as u64;

    let mut exclude = exclude.to_vec();
    if settings.exclude_escaped_arrays {
        let arrays = escaped_arrays(board, connectivity, settings.array_min_pads);
        plan.stats.escaped_arrays = arrays.len();
        exclude.extend(arrays);
        exclude.sort_unstable();
        exclude.dedup();
    }
    let units = collect_units(board, connectivity, settings, &exclude, &mut plan);
    plan.stats.eligible_islands = units.len();
    plan.stats.eligible_pads = units.iter().map(|u| u.pads.len()).sum();

    let t1 = std::time::Instant::now();
    let ctx = Ctx {
        space: &space,
        rules,
        settings,
    };
    let generated: Vec<(Vec<Cand>, Rejections)> = units
        .par_iter()
        .map_init(ClearanceCache::default, |cache, u| {
            candidates(&ctx, u, cache)
        })
        .collect();
    plan.stats.candidate_ms = t1.elapsed().as_millis() as u64;
    let mut cands: Vec<Vec<Cand>> = Vec::with_capacity(units.len());
    let mut rejections: Vec<Rejections> = Vec::with_capacity(units.len());
    for (c, r) in generated {
        plan.stats.candidates += r.total as usize;
        plan.stats.legal_candidates += c.len();
        cands.push(c);
        rejections.push(r);
    }

    let t2 = std::time::Instant::now();
    let chosen = select(&ctx, &units, &cands, &mut plan.stats);
    let shares = if settings.share_within > 0 {
        share_vias(&ctx, &units, &cands, &chosen)
    } else {
        BTreeMap::new()
    };
    plan.stats.select_ms = t2.elapsed().as_millis() as u64;

    for (ui, u) in units.iter().enumerate() {
        if let Some(sh) = shares.get(&ui) {
            let host = &cands[sh.host_unit][chosen[sh.host_unit].expect("hosts are placed")];
            plan.stats.shared_vias += 1;
            plan.sites.push(PlaneViaSite {
                pad: sh.pad,
                net: u.net,
                via: host.via,
                via_diameter: host.diameter,
                via_drill: host.drill,
                stub: Some((sh.stub.a, sh.stub.b, sh.stub.width)),
                layer: sh.layer,
                shared: true,
            });
            for &p in &u.pads {
                if p != sh.pad {
                    plan.skipped
                        .push((p, FanoutSkip::JoinedThroughIsland { via_pad: sh.pad }));
                }
            }
            continue;
        }
        match chosen[ui] {
            Some(ci) => {
                let c = &cands[ui][ci];
                plan.stats.floor_vias += usize::from(c.floor_via);
                plan.stats.minimum_stubs += usize::from(c.min_stub);
                plan.stats.partial_plane_sites += usize::from(c.partial);
                plan.sites.push(PlaneViaSite {
                    pad: c.pad,
                    net: u.net,
                    via: c.via,
                    via_diameter: c.diameter,
                    via_drill: c.drill,
                    stub: Some((c.stub.a, c.stub.b, c.stub.width)),
                    layer: c.layer,
                    shared: false,
                });
                for &p in &u.pads {
                    if p != c.pad {
                        plan.skipped
                            .push((p, FanoutSkip::JoinedThroughIsland { via_pad: c.pad }));
                    }
                }
            }
            None => {
                let r = &rejections[ui];
                let failure = if !cands[ui].is_empty() {
                    FanoutFailure::Conflict
                } else if r.board == 0 && r.total > 0 {
                    FanoutFailure::OutsideOwnPlane
                } else {
                    FanoutFailure::NoLegalSite
                };
                let (reasons, trials) = (r.reasons(), r.trial_list());
                for &p in &u.pads {
                    plan.failed.push((p, failure.clone()));
                    plan.details.push(FailureDetail {
                        pad: p,
                        net: u.net,
                        candidates: r.total,
                        legal: cands[ui].len() as u32,
                        rejections: reasons.clone(),
                        trials: trials.clone(),
                    });
                }
            }
        }
    }
    plan.sites.sort_by_key(|s| s.pad);
    plan.failed.sort_by_key(|f| f.0);
    plan.skipped.sort_by_key(|s| s.0);
    plan.details.sort_by_key(|d| d.pad);
    plan
}

/// Ball-grid footprints whose escapes are already drawn: at least
/// `min_pads` pads, every one an SMD land of one circle, and some netted
/// pad on an island that carries a track, arc or via (a hand-drawn escape
/// pattern; the field's other balls are left to it as well). Sorted.
pub fn escaped_arrays(
    board: &RouteBoard,
    connectivity: &ConnectivityReport,
    min_pads: u32,
) -> Vec<FootprintId> {
    let mut escaped = vec![false; board.pads.len()];
    for net in &connectivity.nets {
        for island in &net.islands {
            let has_escape = island
                .items
                .iter()
                .any(|i| matches!(i, ItemRef::Track(_) | ItemRef::Arc(_) | ItemRef::Via(_)));
            if has_escape {
                for p in island.pads() {
                    escaped[p.index()] = true;
                }
            }
        }
    }
    board
        .footprints
        .iter()
        .filter(|f| {
            let pads = &board.pads[f.pad_range.0 as usize..f.pad_range.1 as usize];
            let balls = pads.iter().all(|p| {
                p.kind == PadKind::Smd && matches!(p.shapes.as_slice(), [Shape::Circle(_)])
            });
            balls
                && pads.len() >= min_pads as usize
                && pads
                    .iter()
                    .any(|p| p.net.is_some() && escaped[p.id.index()])
        })
        .map(|f| f.id)
        .collect()
}

struct Ctx<'a> {
    space: &'a Space<'a>,
    rules: &'a dyn RuleTable,
    settings: &'a FanoutSettings,
}

/// One unjoined island to give a via.
struct Unit {
    net: NetId,
    /// SMD pads of the island, sorted.
    pads: Vec<PadId>,
    /// Where the island sits, for tiling: its first pad's centre.
    anchor: Point,
}

fn collect_units(
    board: &RouteBoard,
    conn: &ConnectivityReport,
    settings: &FanoutSettings,
    exclude: &[FootprintId],
    plan: &mut PlaneViaPlan,
) -> Vec<Unit> {
    let joins = conn.plane_joins(board);
    plan.stats.plane_nets = joins.len();
    let mut units = Vec::new();
    for j in &joins {
        let nc = &conn.nets[j.net.index()];
        for island in &nc.islands {
            if !island.has_pad() || island.items.iter().any(|i| matches!(i, ItemRef::Zone(_))) {
                continue;
            }
            plan.stats.unjoined_islands += 1;
            let pads: Vec<PadId> = island.pads().collect();
            let escaped = island
                .items
                .iter()
                .any(|i| matches!(i, ItemRef::Track(_) | ItemRef::Arc(_) | ItemRef::Via(_)));
            if settings.skip_escaped && escaped {
                plan.skipped
                    .extend(pads.iter().map(|&p| (p, FanoutSkip::Escaped)));
                continue;
            }
            if pads
                .iter()
                .any(|p| exclude.contains(&board.pads[p.index()].footprint))
            {
                plan.skipped
                    .extend(pads.iter().map(|&p| (p, FanoutSkip::ExcludedFootprint)));
                continue;
            }
            let smd: Vec<PadId> = pads
                .iter()
                .copied()
                .filter(|p| board.pads[p.index()].kind == PadKind::Smd)
                .collect();
            if smd.is_empty() {
                plan.skipped
                    .extend(pads.iter().map(|&p| (p, FanoutSkip::NotSmd)));
                continue;
            }
            for &p in &pads {
                if !smd.contains(&p) {
                    plan.skipped.push((p, FanoutSkip::NotSmd));
                }
            }
            units.push(Unit {
                net: j.net,
                anchor: board.pads[smd[0].index()].center,
                pads: smd,
            });
        }
    }
    units
}

// ─── Candidates ──────────────────────────────────────────────────────────────

#[derive(Debug, Clone)]
struct Cand {
    pad: PadId,
    layer: LayerId,
    via: Point,
    diameter: Coord,
    drill: Coord,
    stub: Segment,
    via_key: SubjectKey,
    stub_key: SubjectKey,
    cost: i64,
    floor_via: bool,
    min_stub: bool,
    /// From the partial-plane pass.
    partial: bool,
}

#[derive(Debug, Clone, Default)]
struct Rejections {
    total: u32,
    /// Rejected against board copper, holes, edge or keepouts.
    board: u32,
    plane: u32,
    spokes: u32,
    /// Rejections by reason. Reasons are static labels; they become owned
    /// strings only in the report of a failed island.
    by_reason: BTreeMap<&'static str, u32>,
    /// Every rejected candidate: pad, via centre, reason.
    trials: Vec<(PadId, Point, &'static str)>,
}

impl Rejections {
    fn note(&mut self, why: &'static str) {
        *self.by_reason.entry(why).or_default() += 1;
    }

    fn reasons(&self) -> BTreeMap<String, u32> {
        self.by_reason
            .iter()
            .map(|(k, v)| ((*k).to_string(), *v))
            .collect()
    }

    fn trial_list(&self) -> Vec<Trial> {
        self.trials
            .iter()
            .map(|&(pad, via, why)| Trial {
                pad,
                via,
                reason: why.to_string(),
            })
            .collect()
    }
}

/// The pad copper's centre: the shapes' bounding-box centre.
fn copper_center(shapes: &[Shape]) -> Option<Point> {
    shapes
        .iter()
        .map(Shape::bbox)
        .reduce(konnect_geom::BBox::union)
        .map(|b| b.center())
}

/// How far a ray from `c` along unit `(ux, uy)` stays in the pad copper, nm.
fn exit_distance(shapes: &[Shape], c: Point, ux: f64, uy: f64) -> Option<i64> {
    let inside = |t: f64| {
        let p = Point::new(
            (f64::from(c.x) + ux * t).round() as Coord,
            (f64::from(c.y) + uy * t).round() as Coord,
        );
        let dot = Shape::Circle(Circle::new(p, 0));
        shapes.iter().any(|s| touches(&dot, s))
    };
    if !inside(0.0) {
        return None;
    }
    let bb = shapes
        .iter()
        .map(Shape::bbox)
        .reduce(konnect_geom::BBox::union)?;
    let mut hi = (bb.width().max(bb.height()) as f64) * 1.5 + 2.0;
    let mut lo = 0.0;
    if inside(hi) {
        return None;
    }
    for _ in 0..40 {
        let mid = (lo + hi) / 2.0;
        if inside(mid) {
            lo = mid;
        } else {
            hi = mid;
        }
        if hi - lo < 1.0 {
            break;
        }
    }
    Some(hi.ceil() as i64)
}

fn snap(p: Point, grid: Coord) -> Point {
    if grid <= 1 {
        return p;
    }
    let g = i64::from(grid);
    let r = |v: Coord| ((i64::from(v) as f64 / g as f64).round() as i64 * g) as Coord;
    Point::new(r(p.x), r(p.y))
}

/// Directions (unit vectors, deduplicated) for one pad: `n` evenly spaced
/// exits in the pad's frame (8 for octilinear), the board's octilinear set
/// when the pad sits at an odd angle, and, optionally, the pad's corners.
fn directions(
    pad: &crate::board::Pad,
    n: u32,
    corner_exits: bool,
    c: Point,
) -> Vec<(f64, f64, bool)> {
    let mut dirs: Vec<(f64, f64, bool)> = Vec::new();
    let mut push = |ux: f64, uy: f64, corner: bool| {
        let len = (ux * ux + uy * uy).sqrt();
        if len < 1e-12 {
            return;
        }
        let (ux, uy) = (ux / len, uy / len);
        if dirs
            .iter()
            .all(|&(x, y, _)| (x - ux).abs() > 1e-6 || (y - uy).abs() > 1e-6)
        {
            dirs.push((ux, uy, corner));
        }
    };
    let base = pad.angle_deg.to_radians();
    let step = std::f64::consts::TAU / f64::from(n.max(1));
    for k in 0..n {
        let a = base + f64::from(k) * step;
        push(a.cos(), -a.sin(), false);
    }
    if (pad.angle_deg / 45.0).fract().abs() > 1e-9 {
        for k in 0..8 {
            let a = f64::from(k) * std::f64::consts::FRAC_PI_4;
            push(a.cos(), -a.sin(), false);
        }
    }
    if corner_exits {
        for s in &pad.shapes {
            if let Shape::ConvexPoly(p) = s {
                for v in p.points() {
                    push(f64::from(v.x - c.x), f64::from(v.y - c.y), true);
                }
            }
        }
    }
    dirs
}

/// What one pad offers: its layer, copper centre and the sizes to try.
struct PadPlan<'a> {
    pid: PadId,
    pad: &'a crate::board::Pad,
    layer: LayerId,
    c: Point,
    sizes: Vec<(Coord, Coord, bool)>,
    /// Stub widths outside any courtyard (the probe for [`stub_widths`]).
    widths: Vec<(Coord, bool)>,
}

fn pad_plan<'a>(ctx: &Ctx<'a>, net: NetId, pid: PadId) -> Option<PadPlan<'a>> {
    let rules = ctx.rules;
    let s = ctx.settings;
    let pad = &ctx.space.board.pads[pid.index()];
    let layer = pad.layers.iter().next()?;
    let c = copper_center(&pad.shapes)?;
    let via_rule = rules.via(&RuleSubject::new(ItemKind::Via, Some(net)));
    let class = (via_rule.diameter, via_rule.drill, false);
    let floor = (via_rule.min_diameter, via_rule.min_drill, true);
    let sizes = match s.via_size {
        ViaSizePolicy::NetClass => vec![class],
        ViaSizePolicy::Floor => vec![floor],
        ViaSizePolicy::NetClassThenFloor if (floor.0, floor.1) == (class.0, class.1) => vec![class],
        ViaSizePolicy::NetClassThenFloor => vec![class, floor],
    };
    // Outside any courtyard; [`stub_widths`] resolves them where each stub
    // actually runs.
    let subject = RuleSubject::new(ItemKind::Track, Some(net));
    let widths = width_options(s.stub_width, rules.track_width(&subject, layer));
    Some(PadPlan {
        pid,
        pad,
        layer,
        c,
        sizes,
        widths,
    })
}

/// Stub widths to try for a stub from `pp` to `via`, and the minimum
/// segment length, resolved where the stub runs: a courtyard-scoped rule
/// can set a different width or length there than outside.
fn stub_widths(
    ctx: &Ctx<'_>,
    cache: &mut ClearanceCache,
    net: NetId,
    pp: &PadPlan<'_>,
    via: Point,
) -> (Vec<(Coord, bool)>, Coord) {
    let probe = Shape::Segment(Segment::new(pp.c, via, pp.widths[0].0));
    let key = ctx.space.subject_for(ItemKind::Track, Some(net), &probe);
    let (rule, min_len) = ctx.space.track_rules(cache, ctx.rules, &key, pp.layer);
    (
        width_options(ctx.settings.stub_width, rule),
        min_len.unwrap_or(0),
    )
}

fn width_options(policy: StubWidthPolicy, rule: crate::rules::WidthRule) -> Vec<(Coord, bool)> {
    match policy {
        StubWidthPolicy::Preferred => vec![(rule.preferred, false)],
        StubWidthPolicy::Minimum => vec![(rule.min, true)],
        StubWidthPolicy::PreferredThenMinimum if rule.min == rule.preferred => {
            vec![(rule.preferred, false)]
        }
        StubWidthPolicy::PreferredThenMinimum => {
            vec![(rule.preferred, false), (rule.min, true)]
        }
    }
}

/// One exit geometry: the first legal (size, width) combination, or the
/// reason the most permissive one failed.
#[allow(clippy::too_many_arguments)]
fn try_exit(
    ctx: &Ctx<'_>,
    cache: &mut ClearanceCache,
    net: NetId,
    own_layers: &[LayerId],
    pp: &PadPlan<'_>,
    (ux, uy, corner): (f64, f64, bool),
    t_exit: i64,
    gap: Coord,
    rank: i64,
    partial: bool,
) -> Result<Cand, (Point, &'static str, u8)> {
    let space = ctx.space;
    let mut last = (pp.c, "no candidate", 0u8);
    for &(diameter, drill, floor_via) in &pp.sizes {
        let r = i64::from(radius_of(diameter));
        let dist = t_exit + i64::from(gap) + r;
        let via = snap(
            Point::new(
                (f64::from(pp.c.x) + ux * dist as f64).round() as Coord,
                (f64::from(pp.c.y) + uy * dist as f64).round() as Coord,
            ),
            ctx.settings.snap,
        );
        let via_key = match check_via(ctx, cache, net, via, diameter, drill, own_layers, partial) {
            Ok(k) => k,
            Err((why, kind)) => {
                last = (via, why, kind);
                continue;
            }
        };
        let (widths, min_len) = stub_widths(ctx, cache, net, pp, via);
        for (w, min_stub) in widths {
            let stub = Segment::new(pp.c, via, w);
            let l2 = (stub.b - stub.a).length2();
            if l2 < i128::from(min_len) * i128::from(min_len) {
                last = (via, "stub shorter than the segment minimum", 0);
                continue;
            }
            let shape = Shape::Segment(stub);
            let stub_key = space.subject_for(ItemKind::Track, Some(net), &shape);
            if let Some(cf) =
                space.track_conflict(cache, ctx.rules, pp.layer, stub, Some(net), &stub_key)
            {
                last = (via, stub_label(&cf), 0);
                continue;
            }
            let len = (via - pp.c).length_f64() as i64;
            let cost = len
                + 150_000 * rank
                + if floor_via { 400_000 } else { 0 }
                + if min_stub { 200_000 } else { 0 }
                + if corner { 50_000 } else { 0 };
            return Ok(Cand {
                pad: pp.pid,
                layer: pp.layer,
                via,
                diameter,
                drill,
                stub,
                via_key,
                stub_key,
                cost,
                floor_via,
                min_stub,
                partial,
            });
        }
    }
    Err(last)
}

fn candidates(ctx: &Ctx<'_>, u: &Unit, cache: &mut ClearanceCache) -> (Vec<Cand>, Rejections) {
    let space = ctx.space;
    let s = ctx.settings;
    let mut out = Vec::new();
    let mut rej = Rejections::default();
    let net = u.net;

    // Plane layers carrying the net.
    let own_layers: Vec<LayerId> = {
        let mut v: Vec<LayerId> = space
            .planes
            .iter()
            .filter(|p| p.net == Some(net))
            .map(|p| p.layer)
            .collect();
        v.sort();
        v.dedup();
        v
    };
    let plans: Vec<PadPlan<'_>> = u
        .pads
        .iter()
        .filter_map(|&p| pad_plan(ctx, net, p))
        .collect();

    // Pass 1: the octilinear and corner exits at the near gaps. Pass 2, only
    // when pass 1 found nothing: more directions, farther out. Pass 3, only
    // when both found nothing: every gap and direction again, joining the
    // plane on one of its layers ([`FanoutSettings::partial_plane_fallback`]).
    let all_gaps: Vec<Coord> = s
        .exit_gaps
        .iter()
        .chain(&s.fallback_gaps)
        .copied()
        .filter(|&g| g > 0)
        .collect();
    let third_dirs: u32 = if s.partial_plane_fallback && own_layers.len() > 1 {
        u32::from(s.fallback_directions).max(8)
    } else {
        0
    };
    let passes: [(u32, bool, &[Coord], i64); 3] = [
        (8, s.corner_exits, &s.exit_gaps, 0),
        (s.fallback_directions.into(), false, &s.fallback_gaps, 3),
        (third_dirs, s.corner_exits, &all_gaps, 6),
    ];
    for (pass, &(n_dirs, corners, gaps, rank0)) in passes.iter().enumerate() {
        if pass >= 1 && (!out.is_empty() || n_dirs == 0) {
            break;
        }
        let partial = pass == 2;
        for pp in &plans {
            for dir in directions(pp.pad, n_dirs, corners, pp.c) {
                let Some(t_exit) = exit_distance(&pp.pad.shapes, pp.c, dir.0, dir.1) else {
                    continue;
                };
                for (gi, &gap) in gaps.iter().enumerate() {
                    if pass == 1 && gap <= 0 {
                        continue;
                    }
                    rej.total += 1;
                    match try_exit(
                        ctx,
                        cache,
                        net,
                        &own_layers,
                        pp,
                        dir,
                        t_exit,
                        gap,
                        rank0 + gi as i64,
                        partial,
                    ) {
                        Ok(c) => out.push(c),
                        Err((via, why, kind)) => {
                            rej.note(why);
                            rej.trials.push((pp.pid, via, why));
                            match kind {
                                1 => rej.plane += 1,
                                2 => rej.spokes += 1,
                                _ => rej.board += 1,
                            }
                        }
                    }
                }
            }
        }
    }
    // Cheapest first; ties by geometry for determinism.
    out.sort_by(|a, b| {
        a.cost
            .cmp(&b.cost)
            .then(a.pad.cmp(&b.pad))
            .then(a.via.cmp(&b.via))
    });
    (out, rej)
}

fn stub_label(c: &Conflict) -> &'static str {
    match c {
        Conflict::Copper { .. } => "stub: copper clearance",
        Conflict::HoleToHole { .. } => "stub: hole to hole",
        Conflict::HoleClearance { .. } => "stub: hole clearance",
        Conflict::Edge => "stub: board edge",
        Conflict::Keepout { .. } => "stub: keepout",
    }
}

fn via_label(c: &Conflict) -> &'static str {
    match c {
        Conflict::Copper { .. } => "via: copper clearance",
        Conflict::HoleToHole { .. } => "via: hole to hole",
        Conflict::HoleClearance { .. } => "via: hole clearance",
        Conflict::Edge => "via: board edge",
        Conflict::Keepout { .. } => "via: keepout",
    }
}

/// Exact via legality against the board and the planes. The error carries
/// a reason and its class (0 board, 1 plane, 2 spokes).
#[allow(clippy::too_many_arguments)]
fn check_via(
    ctx: &Ctx<'_>,
    cache: &mut ClearanceCache,
    net: NetId,
    via: Point,
    diameter: Coord,
    drill: Coord,
    own_layers: &[LayerId],
    partial: bool,
) -> Result<SubjectKey, (&'static str, u8)> {
    let space = ctx.space;
    let s = ctx.settings;
    let r = radius_of(diameter);
    let grown = r + s.plane_margin;
    // Plane tests first: they are cheap and reject most far-off sites. A
    // partial-plane candidate needs one own layer, the others are skipped.
    let mut joined = 0usize;
    let mut spokes_short = false;
    for &l in own_layers {
        let inside = space
            .planes
            .iter()
            .filter(|p| p.net == Some(net) && p.layer == l)
            .find(|p| p.index.disc_inside(via, grown));
        let Some(piece) = inside else {
            if partial {
                continue;
            }
            return Err(("via: outside its own plane piece", 1));
        };
        if s.min_spokes > 0 {
            let spoke_w = 100_000.max(r / 2);
            let reach = r + 2 * spoke_w;
            let mut best = 0u8;
            for rot in [0.0f64, std::f64::consts::FRAC_PI_4] {
                let mut n = 0u8;
                for k in 0..4 {
                    let a = rot + f64::from(k) * std::f64::consts::FRAC_PI_2;
                    let e = Point::new(
                        (f64::from(via.x) + a.cos() * f64::from(reach)).round() as Coord,
                        (f64::from(via.y) + a.sin() * f64::from(reach)).round() as Coord,
                    );
                    n += u8::from(piece.index.disc_inside(e, spoke_w / 2 + s.plane_margin));
                }
                best = best.max(n);
            }
            if best < s.min_spokes {
                if partial {
                    spokes_short = true;
                    continue;
                }
                return Err(("via: too few spokes reach its plane", 2));
            }
        }
        joined += 1;
    }
    if partial && joined == 0 {
        return Err(if spokes_short {
            ("via: too few spokes reach its plane", 2)
        } else {
            ("via: outside its own plane piece", 1)
        });
    }
    let islands = space
        .planes
        .iter()
        .filter(|p| s.avoid_other_islands && p.island && p.net != Some(net));
    for p in islands {
        let bb = p.index.bbox().inflate(grown);
        if bb.contains(via) && !p.index.disc_outside(via, grown) {
            return Err(("via: inside another net's island", 1));
        }
    }
    let disc = Shape::Circle(Circle::from_diameter(via, diameter));
    let key = space.subject_for(ItemKind::Via, Some(net), &disc);
    // The via floor can depend on where the via is (courtyard-scoped rules).
    let floor = space.via_rule(cache, ctx.rules, &key);
    if diameter < floor.min_diameter
        || drill < floor.min_drill
        || (diameter - drill) / 2 < floor.min_annular
    {
        return Err(("via: below the via floor here", 0));
    }
    if let Some(c) = space.via_conflict(
        cache,
        ctx.rules,
        via,
        diameter,
        drill,
        space.board.layers.all(),
        Some(net),
        &key,
    ) {
        return Err((via_label(&c), 0));
    }
    // No via-in-pad by accident: stay off the net's own SMD pads (other
    // nets' pads are already kept at their clearance).
    for l in space.board.layers.all().iter() {
        for id in space.items_near(l, &disc.bbox()) {
            let it = &space.items[id as usize];
            if let ItemRef::Pad(pid) = it.item {
                if it.net == Some(net)
                    && space.board.pads[pid.index()].kind == PadKind::Smd
                    && gap_cmp(&disc, &it.shape, 1) == Ordering::Less
                {
                    return Err(("via: on an SMD pad of its own net", 0));
                }
            }
        }
    }
    Ok(key)
}

// ─── Via sharing ─────────────────────────────────────────────────────────────

struct Share {
    pad: PadId,
    layer: LayerId,
    stub: Segment,
    host_unit: usize,
}

/// For islands that got no site: a stub from one of their pads to a placed
/// via of the same net within `share_within`, checked exactly against the
/// board and every placed site and earlier share.
fn share_vias(
    ctx: &Ctx<'_>,
    units: &[Unit],
    cands: &[Vec<Cand>],
    chosen: &[Option<usize>],
) -> BTreeMap<usize, Share> {
    let space = ctx.space;
    let within = i64::from(ctx.settings.share_within);
    let mut cache = ClearanceCache::default();
    let placed: Vec<(usize, &Cand)> = chosen
        .iter()
        .enumerate()
        .filter_map(|(u, c)| c.map(|c| (u, &cands[u][c])))
        .collect();
    let mut grid = SiteGrid::new(1_000_000);
    for (i, (u, c)) in placed.iter().enumerate() {
        grid.insert(&cand_bbox(c), Placed { unit: *u, cand: i });
    }
    let mut shares: BTreeMap<usize, Share> = BTreeMap::new();
    let mut extra: Vec<(NetId, LayerId, Segment, SubjectKey)> = Vec::new();
    for (ui, u) in units.iter().enumerate() {
        if chosen[ui].is_some() {
            continue;
        }
        'pads: for &pid in &u.pads {
            let Some(pp) = pad_plan(ctx, u.net, pid) else {
                continue;
            };
            let reach = BBox::point(pp.c).inflate(within as Coord);
            let mut hosts: Vec<(i128, usize)> = grid
                .near(&reach)
                .into_iter()
                .filter(|p| units[p.unit].net == u.net)
                .map(|p| ((placed[p.cand].1.via - pp.c).length2(), p.cand))
                .filter(|(d2, _)| *d2 <= i128::from(within) * i128::from(within))
                .collect();
            hosts.sort();
            for (_, hi) in hosts {
                let (host_unit, host) = placed[hi];
                let (widths, min_len) = stub_widths(ctx, &mut cache, u.net, &pp, host.via);
                for (w, _) in widths {
                    let stub = Segment::new(pp.c, host.via, w);
                    if (stub.b - stub.a).length2() < i128::from(min_len) * i128::from(min_len) {
                        continue;
                    }
                    let shape = Shape::Segment(stub);
                    let key = space.subject_for(ItemKind::Track, Some(u.net), &shape);
                    if space
                        .track_conflict(&mut cache, ctx.rules, pp.layer, stub, Some(u.net), &key)
                        .is_some()
                    {
                        continue;
                    }
                    let q = stub.bbox().inflate(space.reach);
                    let clash_site = grid.near(&q).into_iter().any(|p| {
                        let o = placed[p.cand].1;
                        let on = units[p.unit].net;
                        on != u.net && !stub_clears(ctx, &mut cache, &shape, &key, pp.layer, o)
                    });
                    let clash_share = extra.iter().any(|(n, l, s, k)| {
                        *n != u.net && *l == pp.layer && {
                            let c = space.clearance(&mut cache, ctx.rules, &key, k, pp.layer);
                            gap_cmp(&shape, &Shape::Segment(*s), i64::from(c)) == Ordering::Less
                        }
                    });
                    if clash_site || clash_share {
                        continue;
                    }
                    extra.push((u.net, pp.layer, stub, key));
                    shares.insert(
                        ui,
                        Share {
                            pad: pid,
                            layer: pp.layer,
                            stub,
                            host_unit,
                        },
                    );
                    break 'pads;
                }
            }
        }
    }
    shares
}

/// A new stub against a placed candidate of another net.
fn stub_clears(
    ctx: &Ctx<'_>,
    cache: &mut ClearanceCache,
    stub: &Shape,
    key: &SubjectKey,
    layer: LayerId,
    o: &Cand,
) -> bool {
    let space = ctx.space;
    let disc = Shape::Circle(Circle::from_diameter(o.via, o.diameter));
    let hole = Shape::Circle(Circle::from_diameter(o.via, o.drill));
    let c = space.clearance(cache, ctx.rules, key, &o.via_key, layer);
    if gap_cmp(stub, &disc, i64::from(c)) == Ordering::Less
        || gap_cmp(stub, &hole, i64::from(space.hole_clearance)) == Ordering::Less
    {
        return false;
    }
    if o.layer == layer {
        let c = space.clearance(cache, ctx.rules, key, &o.stub_key, layer);
        if gap_cmp(stub, &Shape::Segment(o.stub), i64::from(c)) == Ordering::Less {
            return false;
        }
    }
    true
}

// ─── Selection ───────────────────────────────────────────────────────────────

/// Whether two candidates of different islands can both be built.
fn compatible(
    ctx: &Ctx<'_>,
    cache: &mut ClearanceCache,
    a: &Cand,
    an: NetId,
    b: &Cand,
    bn: NetId,
) -> bool {
    let space = ctx.space;
    let rules = ctx.rules;
    let (ha, hb) = (
        Shape::Circle(Circle::from_diameter(a.via, a.drill)),
        Shape::Circle(Circle::from_diameter(b.via, b.drill)),
    );
    if gap_cmp(&ha, &hb, i64::from(space.hole_to_hole)) == Ordering::Less {
        return false;
    }
    if an == bn {
        return true;
    }
    let (da, db) = (
        Shape::Circle(Circle::from_diameter(a.via, a.diameter)),
        Shape::Circle(Circle::from_diameter(b.via, b.diameter)),
    );
    for l in space.board.layers.all().iter() {
        let c = space.clearance(cache, rules, &a.via_key, &b.via_key, l);
        if gap_cmp(&da, &db, i64::from(c)) == Ordering::Less {
            return false;
        }
    }
    let (sa, sb) = (Shape::Segment(a.stub), Shape::Segment(b.stub));
    let c = space.clearance(cache, rules, &a.stub_key, &b.via_key, a.layer);
    if gap_cmp(&sa, &db, i64::from(c)) == Ordering::Less
        || gap_cmp(&sa, &hb, i64::from(space.hole_clearance)) == Ordering::Less
    {
        return false;
    }
    let c = space.clearance(cache, rules, &b.stub_key, &a.via_key, b.layer);
    if gap_cmp(&sb, &da, i64::from(c)) == Ordering::Less
        || gap_cmp(&sb, &ha, i64::from(space.hole_clearance)) == Ordering::Less
    {
        return false;
    }
    if a.layer == b.layer {
        let c = space.clearance(cache, rules, &a.stub_key, &b.stub_key, a.layer);
        if gap_cmp(&sa, &sb, i64::from(c)) == Ordering::Less {
            return false;
        }
    }
    true
}

/// The candidate's copper bounding box (via annulus and stub).
fn cand_bbox(c: &Cand) -> BBox {
    Circle::from_diameter(c.via, c.diameter)
        .bbox()
        .union(c.stub.bbox())
}

/// Islands whose candidates reach farther than this from the island anchor
/// are placed in a final serial pass, so tiles stay independent.
const LOCAL_REACH: i64 = 3_000_000;

/// A placed site, for conflict lookups.
#[derive(Clone, Copy)]
struct Placed {
    unit: usize,
    cand: usize,
}

/// Placed sites bucketed by every grid cell their copper box covers.
struct SiteGrid {
    cell: i64,
    map: BTreeMap<(i64, i64), Vec<Placed>>,
}

impl SiteGrid {
    fn new(cell: i64) -> Self {
        SiteGrid {
            cell: cell.max(1),
            map: BTreeMap::new(),
        }
    }

    fn range(&self, b: &BBox) -> (i64, i64, i64, i64) {
        (
            i64::from(b.min.x).div_euclid(self.cell),
            i64::from(b.max.x).div_euclid(self.cell),
            i64::from(b.min.y).div_euclid(self.cell),
            i64::from(b.max.y).div_euclid(self.cell),
        )
    }

    fn insert(&mut self, b: &BBox, s: Placed) {
        let (x0, x1, y0, y1) = self.range(b);
        for x in x0..=x1 {
            for y in y0..=y1 {
                self.map.entry((x, y)).or_default().push(s);
            }
        }
    }

    fn remove_unit(&mut self, b: &BBox, unit: usize) {
        let (x0, x1, y0, y1) = self.range(b);
        for x in x0..=x1 {
            for y in y0..=y1 {
                if let Some(v) = self.map.get_mut(&(x, y)) {
                    v.retain(|p| p.unit != unit);
                }
            }
        }
    }

    /// Distinct placed sites whose boxes meet `b`, by unit.
    fn near(&self, b: &BBox) -> Vec<Placed> {
        let (x0, x1, y0, y1) = self.range(b);
        let mut out: Vec<Placed> = Vec::new();
        for x in x0..=x1 {
            for y in y0..=y1 {
                if let Some(v) = self.map.get(&(x, y)) {
                    out.extend(v.iter().copied());
                }
            }
        }
        out.sort_by_key(|p| p.unit);
        out.dedup_by_key(|p| p.unit);
        out
    }
}

fn select(
    ctx: &Ctx<'_>,
    units: &[Unit],
    cands: &[Vec<Cand>],
    stats: &mut FanoutStats,
) -> Vec<Option<usize>> {
    let (tw, th) = (
        i64::from(ctx.settings.tile.0.max(1_000_000)),
        i64::from(ctx.settings.tile.1.max(1_000_000)),
    );
    let reach = i64::from(ctx.space.reach) + 1;
    // Same-colour tiles are a full tile apart; local islands keep their
    // copper within `local_reach` of the anchor, so they cannot interact.
    let local_reach = LOCAL_REACH.min((tw.min(th) - reach) / 2 - 1).max(0);
    let mut tiles: BTreeMap<(i64, i64), Vec<usize>> = BTreeMap::new();
    let mut far: Vec<usize> = Vec::new();
    for (i, u) in units.iter().enumerate() {
        if cands[i].is_empty() {
            continue;
        }
        let spread = cands[i]
            .iter()
            .map(|c| {
                let b = cand_bbox(c);
                [
                    b.min,
                    b.max,
                    Point::new(b.min.x, b.max.y),
                    Point::new(b.max.x, b.min.y),
                ]
                .iter()
                .map(|p| (*p - u.anchor).length_f64().ceil() as i64)
                .max()
                .unwrap_or(0)
            })
            .max()
            .unwrap_or(0);
        if spread > local_reach {
            far.push(i);
            continue;
        }
        let k = (
            i64::from(u.anchor.x).div_euclid(tw),
            i64::from(u.anchor.y).div_euclid(th),
        );
        tiles.entry(k).or_default().push(i);
    }
    stats.tiles = tiles.len();
    stats.serial_islands = far.len();
    let cell = 1_000_000;
    let mut chosen: Vec<Option<usize>> = vec![None; units.len()];
    let mut committed = SiteGrid::new(cell);
    let mut repairs = 0usize;
    for colour in 0..4 {
        let batch: Vec<&Vec<usize>> = tiles
            .iter()
            .filter(|(k, _)| (k.0.rem_euclid(2) + 2 * k.1.rem_euclid(2)) as usize == colour)
            .map(|(_, v)| v)
            .collect();
        let solved: Vec<(Vec<(usize, usize)>, usize)> = batch
            .par_iter()
            .map_init(ClearanceCache::default, |cache, members| {
                solve_tile(ctx, cache, units, cands, members, &committed, reach, cell)
            })
            .collect();
        for (picks, r) in solved {
            repairs += r;
            for (ui, ci) in picks {
                chosen[ui] = Some(ci);
                committed.insert(&cand_bbox(&cands[ui][ci]), Placed { unit: ui, cand: ci });
            }
        }
    }
    // Islands with far-reaching candidates (large pads), serially.
    if !far.is_empty() {
        let mut cache = ClearanceCache::default();
        let (picks, r) = solve_tile(ctx, &mut cache, units, cands, &far, &committed, reach, cell);
        repairs += r;
        for (ui, ci) in picks {
            chosen[ui] = Some(ci);
        }
    }
    stats.repairs = repairs;
    chosen
}

/// Greedy independent set over `members`, then one-swap repairs. Returns
/// the (unit, candidate) picks and the number of repairs made.
#[allow(clippy::too_many_arguments)]
fn solve_tile(
    ctx: &Ctx<'_>,
    cache: &mut ClearanceCache,
    units: &[Unit],
    cands: &[Vec<Cand>],
    members: &[usize],
    committed: &SiteGrid,
    reach: i64,
    cell: i64,
) -> (Vec<(usize, usize)>, usize) {
    let mut order: Vec<usize> = members.to_vec();
    order.sort_by_key(|&u| (cands[u].len(), u));
    let mut local = SiteGrid::new(cell);
    let mut pick: BTreeMap<usize, usize> = BTreeMap::new();
    let inflate = reach.min(i64::from(Coord::MAX)) as Coord;

    // The local units a candidate conflicts with; `None` when it conflicts
    // with a committed site (unusable).
    let fits = |cache: &mut ClearanceCache,
                local: &SiteGrid,
                ui: usize,
                ci: usize,
                ignore: Option<usize>|
     -> Option<Vec<usize>> {
        let c = &cands[ui][ci];
        let q = cand_bbox(c).inflate(inflate);
        for p in committed.near(&q) {
            let o = &cands[p.unit][p.cand];
            if !compatible(ctx, cache, c, units[ui].net, o, units[p.unit].net) {
                return None;
            }
        }
        let mut hits = Vec::new();
        for p in local.near(&q) {
            if Some(p.unit) == ignore || p.unit == ui {
                continue;
            }
            let o = &cands[p.unit][p.cand];
            if !compatible(ctx, cache, c, units[ui].net, o, units[p.unit].net) {
                hits.push(p.unit);
            }
        }
        Some(hits)
    };

    let mut failed = Vec::new();
    for &ui in &order {
        let mut done = false;
        for (ci, cand) in cands[ui].iter().enumerate() {
            if let Some(h) = fits(cache, &local, ui, ci, None) {
                if h.is_empty() {
                    pick.insert(ui, ci);
                    local.insert(&cand_bbox(cand), Placed { unit: ui, cand: ci });
                    done = true;
                    break;
                }
            }
        }
        if !done {
            failed.push(ui);
        }
    }

    // Repair: a failed unit takes a candidate blocked by exactly one local
    // pick whose unit can move to another candidate compatible with both.
    let mut repairs = 0;
    for _round in 0..2 {
        let mut progress = false;
        let mut still = Vec::new();
        for &ui in &failed {
            if let Some((ci, q, alt)) =
                find_swap(ctx, cache, units, cands, &local, &pick, ui, &fits)
            {
                let qc = pick[&q];
                local.remove_unit(&cand_bbox(&cands[q][qc]), q);
                pick.insert(q, alt);
                local.insert(&cand_bbox(&cands[q][alt]), Placed { unit: q, cand: alt });
                pick.insert(ui, ci);
                local.insert(&cand_bbox(&cands[ui][ci]), Placed { unit: ui, cand: ci });
                repairs += 1;
                progress = true;
            } else {
                still.push(ui);
            }
        }
        failed = still;
        if !progress {
            break;
        }
    }
    (pick.into_iter().collect(), repairs)
}

type FitFn<'f> =
    dyn Fn(&mut ClearanceCache, &SiteGrid, usize, usize, Option<usize>) -> Option<Vec<usize>> + 'f;

/// A candidate `ci` of failed unit `ui`, blocked only by local unit `q`,
/// and an alternative `alt` for `q` compatible with everything.
#[allow(clippy::too_many_arguments)]
fn find_swap(
    ctx: &Ctx<'_>,
    cache: &mut ClearanceCache,
    units: &[Unit],
    cands: &[Vec<Cand>],
    local: &SiteGrid,
    pick: &BTreeMap<usize, usize>,
    ui: usize,
    fits: &FitFn<'_>,
) -> Option<(usize, usize, usize)> {
    for ci in 0..cands[ui].len() {
        let Some(hits) = fits(cache, local, ui, ci, None) else {
            continue;
        };
        if hits.len() != 1 {
            continue;
        }
        let q = hits[0];
        let qc = pick[&q];
        for alt in 0..cands[q].len() {
            if alt == qc {
                continue;
            }
            let Some(qh) = fits(cache, local, q, alt, Some(q)) else {
                continue;
            };
            if qh.is_empty()
                && compatible(
                    ctx,
                    cache,
                    &cands[ui][ci],
                    units[ui].net,
                    &cands[q][alt],
                    units[q].net,
                )
            {
                return Some((ci, q, alt));
            }
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synthetic::{BoardBuilder, UniformRules};
    use konnect_geom::ArcPolicy;

    fn plan(board: &RouteBoard, settings: &FanoutSettings) -> PlaneViaPlan {
        let rules = UniformRules::default();
        let conn = ConnectivityReport::build(board, &ArcPolicy::default());
        plan_plane_vias(board, &rules, &conn, settings)
    }

    /// Caps with one GND pad each over a GND plane on In1.
    fn caps(n: usize, pitch: f64) -> RouteBoard {
        let w = 10.0 + pitch * n as f64;
        let mut b = BoardBuilder::new(w, 20.0, &["F.Cu", "In1.Cu", "B.Cu"]);
        b.zone("GND", "In1.Cu", (0.5, 0.5, w - 0.5, 19.5));
        for i in 0..n {
            let x = 5.0 + pitch * i as f64;
            let f = b.footprint(&format!("C{i}"), (x, 10.0));
            b.smd_pad(f, "1", "GND", (x - 0.5, 10.0), (0.6, 0.6), "F.Cu");
            b.smd_pad(
                f,
                "2",
                &format!("S{i}"),
                (x + 0.5, 10.0),
                (0.6, 0.6),
                "F.Cu",
            );
        }
        b.build()
    }

    #[test]
    fn every_isolated_pad_gets_a_legal_via_and_stub() {
        let board = caps(4, 5.0);
        let p = plan(&board, &FanoutSettings::default());
        assert_eq!(p.stats.eligible_islands, 4);
        assert_eq!(p.sites.len(), 4, "{:?}", p.details);
        assert!(p.failed.is_empty());
        let rules = UniformRules::default();
        for s in &p.sites {
            let pad = &board.pads[s.pad.index()];
            // The via clears the pad's neighbour by the clearance.
            let other = &board.pads[s.pad.index() + 1];
            let disc = Shape::Circle(Circle::from_diameter(s.via, s.via_diameter));
            assert!(gap_cmp(&disc, &other.shapes[0], 200_000) != Ordering::Less);
            // The stub starts inside its pad.
            let (a, _, w) = s.stub.unwrap();
            assert!(touches(&Shape::Circle(Circle::new(a, 0)), &pad.shapes[0]));
            assert_eq!(w, rules.class.track_width);
        }
        let routed = p.route_plan(&board).unwrap();
        assert_eq!(routed.vias.len(), 4);
        assert_eq!(routed.tracks.len(), 4);
        // Applying the plan joins every pad: rebuild the board with it.
        let mut joined = board.clone();
        for s in &p.sites {
            let (a, b, w) = s.stub.unwrap();
            let id = crate::api::TrackId(joined.tracks.len() as u32);
            joined.tracks.push(crate::board::Track {
                id,
                net: Some(s.net),
                layer: s.layer,
                seg: Segment::new(a, b, w),
                ownership: crate::api::Ownership::Locked,
                uuid: None,
            });
            let id = crate::api::ViaId(joined.vias.len() as u32);
            joined.vias.push(crate::board::Via {
                id,
                net: Some(s.net),
                center: s.via,
                diameter: s.via_diameter,
                drill: s.via_drill,
                layers: joined.layers.all(),
                ownership: crate::api::Ownership::Locked,
                uuid: None,
            });
        }
        let conn = ConnectivityReport::build(&joined, &ArcPolicy::default());
        let j = &conn.plane_joins(&joined)[0];
        assert_eq!(j.unjoined_pads.len(), 0);
    }

    #[test]
    fn close_pads_of_different_nets_conflict_and_one_moves() {
        // Two plane nets on two layers, pads 1.6 mm apart: their nearest
        // sites would collide, so selection must keep them apart.
        let mut b = BoardBuilder::new(20.0, 20.0, &["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]);
        b.zone("GND", "In1.Cu", (0.5, 0.5, 19.5, 19.5));
        b.zone("VCC", "In2.Cu", (0.5, 0.5, 19.5, 19.5));
        let f = b.footprint("C1", (10.0, 10.0));
        b.smd_pad(f, "1", "GND", (9.2, 10.0), (0.6, 0.6), "F.Cu");
        b.smd_pad(f, "2", "VCC", (10.8, 10.0), (0.6, 0.6), "F.Cu");
        let board = b.build();
        let p = plan(&board, &FanoutSettings::default());
        assert_eq!(p.sites.len(), 2, "{:?}", p.details);
        let rules = UniformRules::default();
        let (a, c) = (&p.sites[0], &p.sites[1]);
        let d = (a.via - c.via).length_f64();
        assert!(
            d >= f64::from(a.via_diameter + rules.class.clearance),
            "{d}"
        );
    }

    #[test]
    fn a_via_must_land_in_its_own_plane() {
        // The plane covers only the left half; a pad on the right has no
        // site inside it.
        let mut b = BoardBuilder::new(20.0, 10.0, &["F.Cu", "In1.Cu", "B.Cu"]);
        b.zone("GND", "In1.Cu", (0.5, 0.5, 9.0, 9.5));
        let f = b.footprint("C1", (5.0, 5.0));
        b.smd_pad(f, "1", "GND", (5.0, 5.0), (0.6, 0.6), "F.Cu");
        let g = b.footprint("C2", (15.0, 5.0));
        b.smd_pad(g, "1", "GND", (15.0, 5.0), (0.6, 0.6), "F.Cu");
        let board = b.build();
        let p = plan(&board, &FanoutSettings::default());
        assert_eq!(p.sites.len(), 1);
        assert_eq!(p.failed.len(), 1);
        assert_eq!(p.failed[0].1, FanoutFailure::OutsideOwnPlane);
        assert!(p.details[0]
            .rejections
            .contains_key("via: outside its own plane piece"));
    }

    #[test]
    fn a_partial_plane_site_joins_one_of_the_net_planes() {
        // GND pours on In1 everywhere but on In2 only on the left: a pad on
        // the right has no site inside both, but one inside In1.
        let mut b = BoardBuilder::new(20.0, 10.0, &["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]);
        b.zone("GND", "In1.Cu", (0.5, 0.5, 19.5, 9.5));
        b.zone("GND", "In2.Cu", (0.5, 0.5, 9.0, 9.5));
        let g = b.footprint("C2", (15.0, 5.0));
        b.smd_pad(g, "1", "GND", (15.0, 5.0), (0.6, 0.6), "F.Cu");
        let board = b.build();
        let strict = plan(
            &board,
            &FanoutSettings {
                partial_plane_fallback: false,
                ..Default::default()
            },
        );
        assert_eq!(strict.failed.len(), 1);
        assert_eq!(strict.failed[0].1, FanoutFailure::OutsideOwnPlane);
        let p = plan(&board, &FanoutSettings::default());
        assert_eq!(p.sites.len(), 1, "{:?}", p.details);
        assert_eq!(p.stats.partial_plane_sites, 1);
        let s = &p.sites[0];
        // Inside the In1 pour, by the via's radius.
        let r = s.via_diameter / 2;
        assert!(s.via.x - r > 500_000 && s.via.x + r < 19_500_000);
    }

    #[test]
    fn hand_escaped_ball_grids_are_left_alone() {
        // An 8 x 8 grid of GND and signal balls; every ball has a dog-bone
        // except two GND balls, which a plain fanout would give vias.
        let mut b = BoardBuilder::new(14.0, 14.0, &["F.Cu", "In1.Cu", "B.Cu"]);
        b.zone("GND", "In1.Cu", (0.5, 0.5, 13.5, 13.5));
        let u = b.footprint("U1", (7.0, 7.0));
        for r in 0..8 {
            for c in 0..8 {
                let at = (3.5 + c as f64, 3.5 + r as f64);
                let net = if (r + c) % 2 == 0 {
                    "GND".to_string()
                } else {
                    format!("S{r}_{c}")
                };
                b.ball_pad(u, &format!("{r}_{c}"), &net, at, 0.4, "F.Cu");
                if (r, c) != (0, 0) && (r, c) != (7, 7) {
                    let via = (at.0 + 0.5, at.1 + 0.5);
                    b.track(&net, "F.Cu", at, via, 0.12, true);
                    b.via(&net, via, 0.45, 0.2, true);
                }
            }
        }
        let board = b.build();
        let conn = ConnectivityReport::build(&board, &ArcPolicy::default());
        assert_eq!(escaped_arrays(&board, &conn, 64).len(), 1);
        let p = plan(&board, &FanoutSettings::default());
        assert_eq!(p.stats.escaped_arrays, 1);
        assert!(p.sites.is_empty(), "{:?}", p.sites);
        assert!(p
            .skipped
            .iter()
            .any(|(_, why)| *why == FanoutSkip::ExcludedFootprint));
        let open = plan(
            &board,
            &FanoutSettings {
                exclude_escaped_arrays: false,
                ..Default::default()
            },
        );
        assert_eq!(open.stats.eligible_islands, 2);
    }

    #[test]
    fn excluded_and_escaped_islands_are_skipped() {
        let mut b = BoardBuilder::new(20.0, 10.0, &["F.Cu", "In1.Cu", "B.Cu"]);
        b.zone("GND", "In1.Cu", (0.5, 0.5, 19.5, 9.5));
        let f = b.footprint("C1", (5.0, 5.0));
        b.smd_pad(f, "1", "GND", (5.0, 5.0), (0.6, 0.6), "F.Cu");
        let g = b.footprint("C2", (15.0, 5.0));
        b.smd_pad(g, "1", "GND", (15.0, 5.0), (0.6, 0.6), "F.Cu");
        // C2's pad has a stub that misses the plane (no via): escaped.
        b.track("GND", "F.Cu", (15.0, 5.0), (16.0, 5.0), 0.2, true);
        let board = b.build();
        let rules = UniformRules::default();
        let conn = ConnectivityReport::build(&board, &ArcPolicy::default());
        let c1 = board.footprint_by_reference("C1").unwrap();
        let p = plan_plane_vias_excluding(&board, &rules, &conn, &FanoutSettings::default(), &[c1]);
        assert!(p.sites.is_empty());
        let reasons: Vec<&FanoutSkip> = p.skipped.iter().map(|s| &s.1).collect();
        assert_eq!(
            reasons,
            vec![&FanoutSkip::ExcludedFootprint, &FanoutSkip::Escaped]
        );
    }

    #[test]
    fn a_crowded_pad_can_share_a_neighbours_via() {
        // Two GND pads 4.2 mm apart; the plane ends 2.7 mm short of the
        // second, beyond its longest exit, so only sharing joins it.
        let mut b = BoardBuilder::new(20.0, 10.0, &["F.Cu", "In1.Cu", "B.Cu"]);
        b.zone("GND", "In1.Cu", (2.0, 0.5, 6.5, 9.5));
        let f = b.footprint("C1", (5.0, 5.0));
        b.smd_pad(f, "1", "GND", (5.0, 5.0), (0.6, 0.6), "F.Cu");
        let g = b.footprint("C2", (9.2, 5.0));
        b.smd_pad(g, "1", "GND", (9.2, 5.0), (0.6, 0.6), "F.Cu");
        let board = b.build();
        let p = plan(&board, &FanoutSettings::default());
        assert_eq!(p.sites.len(), 1);
        let shared = plan(
            &board,
            &FanoutSettings {
                share_within: 5_000_000,
                ..Default::default()
            },
        );
        assert_eq!(shared.sites.len(), 2, "{:?}", shared.details);
        assert!(shared.failed.is_empty());
        let (own, guest) = (&shared.sites[0], &shared.sites[1]);
        assert!(!own.shared && guest.shared);
        assert_eq!(own.via, guest.via);
        let routed = shared.route_plan(&board).unwrap();
        assert_eq!((routed.vias.len(), routed.tracks.len()), (1, 2));
    }

    #[test]
    fn the_plan_is_identical_across_thread_counts() {
        let board = caps(24, 2.0);
        let run = |threads| {
            rayon::ThreadPoolBuilder::new()
                .num_threads(threads)
                .build()
                .unwrap()
                .install(|| {
                    plan(
                        &board,
                        &FanoutSettings {
                            tile: (8_000_000, 8_000_000),
                            ..Default::default()
                        },
                    )
                })
        };
        let (one, many) = (run(1), run(4));
        assert_eq!(one.sites, many.sites);
        assert_eq!(one.failed, many.failed);
        assert!(
            one.stats.tiles > 1,
            "{:?} {:?}",
            one.stats,
            one.details.first()
        );
        assert!(!one.sites.is_empty());
    }
}
