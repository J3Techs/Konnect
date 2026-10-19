//! S6 optimization (owned by WS-E).
//!
//! Gridless clean-up of routed copper, in the spirit of Freerouting's
//! pull-tight and KiCad's "merge obtuse" (ideas only; no code from either):
//!
//! 1. **Junction splitting.** A routed track ending on the middle of another
//!    of its net splits that track there, so every joint is a vertex.
//! 2. **Collinear merging.** Two tracks meeting end to end in a straight line,
//!    same width, nothing else at the joint, become one.
//! 3. **Redundant-via removal.** A routed via whose copper connects to its
//!    net on one layer only, through track ends at its centre, goes.
//! 4. **Pull-tight.** Along each chain of degree-2 joints, a run of segments
//!    is replaced by the shortest octilinear connection (a straight piece,
//!    or one straight and one 45° piece) when that is shorter and legal.
//! 5. **45° corner cuts.** A right-angle bend left after pull-tight is
//!    chamfered, as large as the neighbourhood allows.
//!
//! Every candidate edit passes three gates before it is taken: the geometry
//! guard ([`lint`]: it may not add a jog, hairpin, acute bend, T-junction,
//! near joint or short segment near the edit), connectivity (every same-net
//! item the old copper touched, the new copper touches), and the exact
//! checker ([`exact::ExactChecker`]) against the board and every other net.
//!
//! # Parallelism and determinism
//!
//! Nets are optimized in parallel (rayon), each against a read-only
//! snapshot of every other net's copper. The results are then committed in
//! net order: a net's new segments are re-checked exactly against the new
//! segments of the nets committed before it, and a net that conflicts is
//! put back as it was and retried in the next round against the updated
//! copper. Nothing depends on scheduling, so 1 and N threads give the same
//! copper.

pub mod exact;
pub mod lint;

use crate::api::{LayerId, LayerMask, NetId, Stage};
use crate::board::RouteBoard;
use crate::plan::{RoutedCopper, RoutedTrack, RoutedVia};
use crate::rules::{RuleSubject, RuleTable};
use crate::telemetry::Telemetry;
use exact::{CheckerOptions, ExactChecker, RoutedSet};
use konnect_geom::{touches, BBox, Coord, Point, Segment, Shape, Vector};
use lint::{lint_group, LintLimits, LintSeg, NEAR_JOINT_NM};

/// How far from octilinear (nm, in the shorter leg) a shortcut may be and
/// still be drawn as one straight piece.
const SNAP_NM: i64 = 1_000;
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct OptimizeReport {
    pub segments_merged: u32,
    pub corners_cut: u32,
    pub vias_removed: u32,
    /// Candidate edits that improved the copper but failed a gate.
    pub edits_rejected: u32,
    /// Pull-tight replacements taken.
    pub pull_tight_edits: u32,
    /// Tracks split where another track of the net ended on them.
    pub junctions_split: u32,
    /// Track ends pulled back inside a pad to undo a sharp joint there.
    pub pad_entries_trimmed: u32,
    /// Short jogs redrawn as a slant under the lint's bend threshold.
    pub jogs_smoothed: u32,
    /// Sharp Y junctions turned into right-angle T junctions.
    pub junctions_squared: u32,
    /// Exact checks run (one per candidate segment).
    pub exact_checks: u64,
    /// Nets whose copper changed.
    pub nets_changed: u32,
    /// Times a net's edits conflicted with a net committed before it and
    /// were put back (to be retried in the next round).
    pub nets_reverted: u32,
    pub tracks_before: u32,
    pub tracks_after: u32,
    pub vias_before: u32,
    pub vias_after: u32,
    /// Total routed track centre-line length, nm.
    pub length_before_nm: i64,
    pub length_after_nm: i64,
    /// Geometry defects ([`lint`]) over the routed copper before and after.
    pub defects_before: u32,
    pub defects_after: u32,
}

/// What the optimizer may do.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OptimizeOptions {
    pub split_junctions: bool,
    pub merge_collinear: bool,
    pub remove_vias: bool,
    pub pull_tight: bool,
    pub corner_cuts: bool,
    /// End one of two tracks meeting sharply inside a pad short of the joint.
    pub trim_pad_entries: bool,
    /// Redraw a short jog between parallel runs as one gentle slant.
    pub smooth_jogs: bool,
    /// Move a branch meeting a run at a sharp angle to a right-angle T.
    pub square_junctions: bool,
    /// Keep every new segment at a multiple of 45°.
    pub octilinear: bool,
    /// The geometry gate's minimum interior bend angle. The rule table's
    /// `track_angle` minimum applies too, whichever is larger.
    pub min_bend_angle_deg: f64,
    /// The geometry gate's jog length (`route_geometry`'s default 0.25 mm).
    pub jog_length_nm: Coord,
    /// Most segments one pull-tight replacement spans.
    pub window: usize,
    /// A shortcut this close to octilinear (degrees) may be one straight
    /// piece at its own angle rather than a dogleg.
    pub max_skew_deg: f64,
    /// Passes of the whole edit sequence per net.
    pub max_passes: u32,
    /// Commit rounds (a reverted net is retried in the next).
    pub rounds: u32,
    pub checker: CheckerOptions,
}

impl Default for OptimizeOptions {
    /// Everything on; 89° bends (the plan's strict-lint setting), 0.25 mm
    /// jogs, 48-segment windows, three passes, two commit rounds.
    fn default() -> Self {
        OptimizeOptions {
            split_junctions: true,
            merge_collinear: true,
            remove_vias: true,
            pull_tight: true,
            corner_cuts: true,
            trim_pad_entries: true,
            smooth_jogs: true,
            square_junctions: true,
            octilinear: true,
            min_bend_angle_deg: 89.0,
            jog_length_nm: 250_000,
            window: 48,
            max_skew_deg: 2.0,
            max_passes: 3,
            rounds: 2,
            checker: CheckerOptions::default(),
        }
    }
}

/// Improve `copper` in place without introducing any violation.
pub fn optimize(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    copper: &mut RoutedCopper,
) -> OptimizeReport {
    optimize_with(
        board,
        rules,
        copper,
        &OptimizeOptions::default(),
        &mut Telemetry::default(),
    )
}

/// [`optimize`] with explicit options, recording into `telemetry`
/// (stage time under [`Stage::Optimize`], counters prefixed `optimize_`).
pub fn optimize_with(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    copper: &mut RoutedCopper,
    opts: &OptimizeOptions,
    telemetry: &mut Telemetry,
) -> OptimizeReport {
    let start = std::time::Instant::now();
    let checker = ExactChecker::new(board, rules, opts.checker);
    let index_us = start.elapsed().as_micros() as u64;
    let report = optimize_checked(&checker, copper, opts);
    telemetry.add_stage_time(Stage::Optimize, start.elapsed().as_micros() as u64);
    telemetry.threads = telemetry.threads.max(rayon::current_num_threads() as u32);
    telemetry.count("optimize_index_us", index_us);
    telemetry.count("optimize_exact_checks", report.exact_checks);
    telemetry.count("optimize_edits_rejected", u64::from(report.edits_rejected));
    telemetry.count("optimize_nets_changed", u64::from(report.nets_changed));
    telemetry.count("optimize_nets_reverted", u64::from(report.nets_reverted));
    report
}

/// The geometry limits for routed copper of `net` on `layer`.
pub fn lint_limits(
    rules: &dyn RuleTable,
    opts: &OptimizeOptions,
    net: NetId,
    layer: LayerId,
) -> LintLimits {
    let subject = RuleSubject::new(crate::api::ItemKind::Track, Some(net));
    LintLimits {
        jog_length_nm: opts.jog_length_nm,
        min_bend_angle_deg: opts
            .min_bend_angle_deg
            .max(rules.min_track_angle_deg(&subject, layer).unwrap_or(0.0)),
        min_segment_length_nm: rules.min_segment_length(&subject, layer),
    }
}

/// Geometry defects of routed copper, grouped by (net, layer) as
/// `route_geometry::lint` groups an `add_routing` batch.
pub fn geometry_defects(
    rules: &dyn RuleTable,
    opts: &OptimizeOptions,
    copper: &RoutedCopper,
) -> Vec<(NetId, LayerId, lint::Defect)> {
    let mut groups: BTreeMap<(NetId, LayerId), Vec<LintSeg>> = BTreeMap::new();
    for t in &copper.tracks {
        groups.entry((t.net, t.layer)).or_default().push(LintSeg {
            a: t.seg.a,
            b: t.seg.b,
            width: t.seg.width,
            fixed: false,
        });
    }
    let groups: Vec<_> = groups.into_iter().collect();
    groups
        .par_iter()
        .flat_map_iter(|((net, layer), segs)| {
            let limits = lint_limits(rules, opts, *net, *layer);
            lint_group(segs, &limits)
                .into_iter()
                .map(move |d| (*net, *layer, d))
        })
        .collect()
}

fn total_length(copper: &RoutedCopper) -> i64 {
    copper.tracks.iter().map(|t| t.seg.length()).sum()
}

/// [`optimize_with`] against a prepared checker.
pub fn optimize_checked(
    checker: &ExactChecker<'_>,
    copper: &mut RoutedCopper,
    opts: &OptimizeOptions,
) -> OptimizeReport {
    let board = checker.board();
    let rules = checker.rules();
    let mut report = OptimizeReport {
        tracks_before: copper.tracks.len() as u32,
        vias_before: copper.vias.len() as u32,
        length_before_nm: total_length(copper),
        defects_before: geometry_defects(rules, opts, copper).len() as u32,
        ..OptimizeReport::default()
    };
    let context = BoardContext::new(board);

    // Per-net copper, in net order.
    let mut per_net: BTreeMap<NetId, (Vec<RoutedTrack>, Vec<RoutedVia>)> = BTreeMap::new();
    for t in &copper.tracks {
        per_net.entry(t.net).or_default().0.push(*t);
    }
    for v in &copper.vias {
        per_net.entry(v.net).or_default().1.push(*v);
    }
    let mut current: BTreeMap<NetId, (Vec<RoutedTrack>, Vec<RoutedVia>)> = per_net.clone();
    let mut dirty: Vec<NetId> = per_net.keys().copied().collect();
    let mut changed: BTreeSet<NetId> = BTreeSet::new();

    for _round in 0..opts.rounds.max(1) {
        if dirty.is_empty() {
            break;
        }
        let snapshot_copper = RoutedCopper {
            tracks: current
                .values()
                .flat_map(|(t, _)| t.iter().copied())
                .collect(),
            vias: current
                .values()
                .flat_map(|(_, v)| v.iter().copied())
                .collect(),
        };
        let snapshot = RoutedSet::from_copper(board.layer_count(), &snapshot_copper);
        let results: Vec<(NetId, NetOutcome)> = dirty
            .par_iter()
            .map(|&net| {
                let (tracks, vias) = &current[&net];
                let mut work = NetWork::new(checker, &snapshot, &context, opts, net, tracks, vias);
                work.run();
                (net, work.finish())
            })
            .collect();
        // Commit in net order against the edits already committed.
        let mut overlay = RoutedSet::new(board.layer_count());
        let mut retry = Vec::new();
        for (net, outcome) in results {
            report.exact_checks += outcome.stats.exact_checks;
            if !outcome.changed {
                report.edits_rejected += outcome.stats.rejected;
                continue;
            }
            let before = &current[&net];
            let mut old: BTreeMap<(LayerId, Point, Point, Coord), u32> = BTreeMap::new();
            for t in &before.0 {
                *old.entry(track_key(t)).or_default() += 1;
            }
            let mut fresh = Vec::new();
            for t in &outcome.tracks {
                match old.get_mut(&track_key(t)) {
                    Some(n) if *n > 0 => *n -= 1,
                    _ => fresh.push(*t),
                }
            }
            let conflict = !overlay.is_empty()
                && fresh.iter().any(|t| {
                    report.exact_checks += 1;
                    checker
                        .track_violation(net, t.layer, &t.seg, &[&overlay], &|_, _| false)
                        .is_some()
                });
            if conflict {
                report.nets_reverted += 1;
                retry.push(net);
                continue;
            }
            for t in &fresh {
                overlay.push_track(*t);
            }
            report.segments_merged += outcome.stats.merged;
            report.corners_cut += outcome.stats.corners_cut;
            report.vias_removed += outcome.stats.vias_removed;
            report.edits_rejected += outcome.stats.rejected;
            report.pull_tight_edits += outcome.stats.pulled;
            report.junctions_split += outcome.stats.split;
            report.pad_entries_trimmed += outcome.stats.trimmed;
            report.jogs_smoothed += outcome.stats.jogs_smoothed;
            report.junctions_squared += outcome.stats.squared;
            changed.insert(net);
            current.insert(net, (outcome.tracks, outcome.vias));
        }
        dirty = retry;
    }
    report.nets_changed = changed.len() as u32;

    // Reassemble in the original order: each changed net's copper takes the
    // place of its first item.
    let mut tracks = Vec::with_capacity(copper.tracks.len());
    let mut emitted: BTreeSet<NetId> = BTreeSet::new();
    for t in &copper.tracks {
        if !changed.contains(&t.net) {
            tracks.push(*t);
        } else if emitted.insert(t.net) {
            tracks.extend(current[&t.net].0.iter().copied());
        }
    }
    let mut vias = Vec::with_capacity(copper.vias.len());
    let mut emitted: BTreeSet<NetId> = BTreeSet::new();
    for v in &copper.vias {
        if !changed.contains(&v.net) {
            vias.push(*v);
        } else if emitted.insert(v.net) {
            vias.extend(current[&v.net].1.iter().copied());
        }
    }
    copper.tracks = tracks;
    copper.vias = vias;

    report.tracks_after = copper.tracks.len() as u32;
    report.vias_after = copper.vias.len() as u32;
    report.length_after_nm = total_length(copper);
    report.defects_after = geometry_defects(rules, opts, copper).len() as u32;
    report
}

fn track_key(t: &RoutedTrack) -> (LayerId, Point, Point, Coord) {
    let (a, b) = if t.seg.a <= t.seg.b {
        (t.seg.a, t.seg.b)
    } else {
        (t.seg.b, t.seg.a)
    };
    (t.layer, a, b, t.seg.width)
}

/// Board copper the optimizer must respect while editing one net: its fixed
/// tracks (angles are judged against them) and its via centres (never
/// merged away).
struct BoardContext {
    fixed_tracks: BTreeMap<NetId, Vec<(LayerId, Segment)>>,
    via_centres: BTreeMap<NetId, Vec<Point>>,
}

impl BoardContext {
    fn new(board: &RouteBoard) -> Self {
        let mut fixed_tracks: BTreeMap<NetId, Vec<(LayerId, Segment)>> = BTreeMap::new();
        for t in &board.tracks {
            if let Some(net) = t.net {
                fixed_tracks.entry(net).or_default().push((t.layer, t.seg));
            }
        }
        let mut via_centres: BTreeMap<NetId, Vec<Point>> = BTreeMap::new();
        for v in &board.vias {
            if let Some(net) = v.net {
                via_centres.entry(net).or_default().push(v.center);
            }
        }
        BoardContext {
            fixed_tracks,
            via_centres,
        }
    }
}

#[derive(Debug, Default, Clone, Copy)]
struct NetStats {
    merged: u32,
    corners_cut: u32,
    vias_removed: u32,
    rejected: u32,
    pulled: u32,
    split: u32,
    trimmed: u32,
    jogs_smoothed: u32,
    squared: u32,
    exact_checks: u64,
}

struct NetOutcome {
    tracks: Vec<RoutedTrack>,
    vias: Vec<RoutedVia>,
    changed: bool,
    stats: NetStats,
}

/// A chain of degree-2 joints on one layer at one width.
struct Chain {
    layer: LayerId,
    verts: Vec<Point>,
    slots: Vec<usize>,
}

/// One net's copper while it is being edited.
struct NetWork<'c, 'b> {
    ck: &'c ExactChecker<'b>,
    snapshot: &'c RoutedSet,
    opts: &'c OptimizeOptions,
    net: NetId,
    tracks: Vec<Option<RoutedTrack>>,
    vias: Vec<RoutedVia>,
    fixed: &'c [(LayerId, Segment)],
    board_vias: &'c [Point],
    limits: BTreeMap<LayerId, LintLimits>,
    original: (Vec<RoutedTrack>, Vec<RoutedVia>),
    stats: NetStats,
    scratch: Vec<u32>,
    /// Accept an edit only if it removes a geometry defect (jog smoothing
    /// trades length for shape, so it must buy something).
    require_fewer_defects: bool,
}

fn is_octilinear(v: Vector) -> bool {
    v.x == 0 || v.y == 0 || v.x.abs() == v.y.abs()
}

fn add(p: Point, v: Vector) -> Point {
    Point::new(
        (i64::from(p.x) + v.x) as Coord,
        (i64::from(p.y) + v.y) as Coord,
    )
}

fn len_f(a: Point, b: Point) -> f64 {
    (b - a).length_f64()
}

/// Whether the path turns at `q` (coming from `p`, going to `r`).
fn bends(p: Point, q: Point, r: Point) -> bool {
    let (u, v) = (q - p, r - q);
    u.cross(v) != 0 || u.dot(v) < 0
}

impl<'c, 'b> NetWork<'c, 'b> {
    fn new(
        ck: &'c ExactChecker<'b>,
        snapshot: &'c RoutedSet,
        context: &'c BoardContext,
        opts: &'c OptimizeOptions,
        net: NetId,
        tracks: &[RoutedTrack],
        vias: &[RoutedVia],
    ) -> Self {
        let mut limits = BTreeMap::new();
        for t in tracks {
            limits
                .entry(t.layer)
                .or_insert_with(|| lint_limits(ck.rules(), opts, net, t.layer));
        }
        NetWork {
            ck,
            snapshot,
            opts,
            net,
            tracks: tracks.iter().copied().map(Some).collect(),
            vias: vias.to_vec(),
            fixed: context
                .fixed_tracks
                .get(&net)
                .map_or(&[][..], Vec::as_slice),
            board_vias: context.via_centres.get(&net).map_or(&[][..], Vec::as_slice),
            limits,
            original: (tracks.to_vec(), vias.to_vec()),
            stats: NetStats::default(),
            scratch: Vec::new(),
            require_fewer_defects: false,
        }
    }

    fn finish(self) -> NetOutcome {
        let tracks: Vec<RoutedTrack> = self.tracks.into_iter().flatten().collect();
        let changed = tracks != self.original.0 || self.vias != self.original.1;
        NetOutcome {
            tracks,
            vias: self.vias,
            changed,
            stats: self.stats,
        }
    }

    fn compact(&mut self) {
        self.tracks.retain(Option::is_some);
    }

    fn run(&mut self) {
        for _ in 0..self.opts.max_passes.max(1) {
            let before = self.signature();
            if self.opts.merge_collinear {
                self.merge_overlaps();
            }
            if self.opts.split_junctions {
                self.split_junctions();
            }
            if self.opts.merge_collinear {
                self.merge_collinear();
            }
            if self.opts.remove_vias {
                self.remove_redundant_vias();
                if self.opts.merge_collinear {
                    self.merge_collinear();
                }
            }
            if self.opts.pull_tight {
                self.pull_tight();
                if self.opts.merge_collinear {
                    self.merge_collinear();
                }
            }
            if self.opts.corner_cuts {
                self.corner_cuts();
                if self.opts.merge_collinear {
                    self.merge_collinear();
                }
            }
            if self.opts.smooth_jogs {
                self.smooth_jogs();
            }
            if self.opts.square_junctions {
                self.square_junctions();
            }
            if self.opts.trim_pad_entries {
                self.trim_pad_entries();
            }
            if self.signature() == before {
                break;
            }
        }
        self.compact();
    }

    fn signature(&self) -> (usize, usize, i64) {
        let live: Vec<&RoutedTrack> = self.tracks.iter().flatten().collect();
        (
            live.len(),
            self.vias.len(),
            live.iter().map(|t| t.seg.length()).sum(),
        )
    }

    fn limits(&self, layer: LayerId) -> LintLimits {
        self.limits
            .get(&layer)
            .copied()
            .unwrap_or_else(|| lint_limits(self.ck.rules(), self.opts, self.net, layer))
    }

    /// Points on `layer` that must stay joints: via centres (routed or
    /// board) and the ends of fixed tracks.
    fn pinned(&self, layer: LayerId) -> BTreeSet<Point> {
        let mut pins: BTreeSet<Point> = self.board_vias.iter().copied().collect();
        pins.extend(
            self.vias
                .iter()
                .filter(|v| v.layers.contains(layer))
                .map(|v| v.center),
        );
        for (l, s) in self.fixed {
            if *l == layer {
                pins.insert(s.a);
                pins.insert(s.b);
            }
        }
        pins
    }

    fn layers(&self) -> Vec<LayerId> {
        let set: BTreeSet<LayerId> = self.tracks.iter().flatten().map(|t| t.layer).collect();
        set.into_iter().collect()
    }

    // ─── Junction splitting ──────────────────────────────────────────────

    fn split_junctions(&mut self) {
        loop {
            let mut split = None;
            'find: for (i, t) in self.tracks.iter().enumerate() {
                let Some(t) = t else { continue };
                for (j, o) in self.tracks.iter().enumerate() {
                    let Some(o) = o else { continue };
                    if i == j || o.layer != t.layer {
                        continue;
                    }
                    for p in [o.seg.a, o.seg.b] {
                        if strictly_inside(p, t.seg.a, t.seg.b) {
                            split = Some((i, p));
                            break 'find;
                        }
                    }
                }
            }
            let Some((i, p)) = split else { break };
            let t = self.tracks[i].expect("live track");
            self.tracks[i] = Some(RoutedTrack {
                seg: Segment::new(t.seg.a, p, t.seg.width),
                ..t
            });
            self.tracks.push(Some(RoutedTrack {
                seg: Segment::new(p, t.seg.b, t.seg.width),
                ..t
            }));
            self.stats.split += 1;
        }
    }

    // ─── Overlaps ────────────────────────────────────────────────────────

    /// Collinear tracks of one width that overlap along a length (doubled
    /// copper another tool left) become their union: the same copper.
    fn merge_overlaps(&mut self) {
        let mut refused: BTreeSet<(usize, usize)> = BTreeSet::new();
        loop {
            let mut found = None;
            'find: for i in 0..self.tracks.len() {
                let Some(ti) = self.tracks[i] else { continue };
                let (a, b) = (ti.seg.a, ti.seg.b);
                let ab = b - a;
                let l2 = ab.length2();
                if l2 == 0 {
                    continue;
                }
                let bi = ti.seg.bbox();
                for j in i + 1..self.tracks.len() {
                    let Some(tj) = self.tracks[j] else { continue };
                    if refused.contains(&(i, j))
                        || tj.layer != ti.layer
                        || tj.seg.width != ti.seg.width
                        || !tj.seg.bbox().intersects(&bi)
                        || ab.cross(tj.seg.a - a) != 0
                        || ab.cross(tj.seg.b - a) != 0
                    {
                        continue;
                    }
                    let (s, t) = (ab.dot(tj.seg.a - a), ab.dot(tj.seg.b - a));
                    let (lo, hi) = (s.min(t), s.max(t));
                    // Overlap of [0, l2] and [lo, hi] with positive length.
                    if lo.max(0) >= hi.min(l2) {
                        continue;
                    }
                    let start = if lo < 0 {
                        if s < t {
                            tj.seg.a
                        } else {
                            tj.seg.b
                        }
                    } else {
                        a
                    };
                    let end = if hi > l2 {
                        if s > t {
                            tj.seg.a
                        } else {
                            tj.seg.b
                        }
                    } else {
                        b
                    };
                    found = Some((i, j, Segment::new(start, end, ti.seg.width)));
                    break 'find;
                }
            }
            let Some((i, j, union)) = found else { break };
            let layer = self.tracks[i].expect("live").layer;
            if !self.geometry_ok(layer, &[i, j], &[union]) {
                refused.insert((i, j));
                continue;
            }
            let t = self.tracks[i].expect("live");
            self.tracks[i] = Some(RoutedTrack { seg: union, ..t });
            self.tracks[j] = None;
            self.stats.merged += 1;
        }
        self.compact();
    }

    // ─── Collinear merging ───────────────────────────────────────────────

    fn joints(&self) -> BTreeMap<(LayerId, Point), Vec<(usize, bool)>> {
        let mut ends: BTreeMap<(LayerId, Point), Vec<(usize, bool)>> = BTreeMap::new();
        for (i, t) in self.tracks.iter().enumerate() {
            if let Some(t) = t {
                ends.entry((t.layer, t.seg.a)).or_default().push((i, true));
                ends.entry((t.layer, t.seg.b)).or_default().push((i, false));
            }
        }
        ends
    }

    fn other_end(&self, slot: usize, at_a: bool) -> Point {
        let t = self.tracks[slot].as_ref().expect("live track");
        if at_a {
            t.seg.b
        } else {
            t.seg.a
        }
    }

    fn merge_collinear(&mut self) {
        loop {
            let mut merged_any = false;
            let joints = self.joints();
            let mut pins: BTreeMap<LayerId, BTreeSet<Point>> = BTreeMap::new();
            // Slots merged this sweep: `joints` no longer describes them.
            let mut touched: BTreeSet<usize> = BTreeSet::new();
            for ((layer, p), ends) in &joints {
                if ends.len() != 2 {
                    continue;
                }
                let (i, ai) = ends[0];
                let (j, aj) = ends[1];
                if i == j || touched.contains(&i) || touched.contains(&j) {
                    continue;
                }
                let (ti, tj) = (self.tracks[i].unwrap(), self.tracks[j].unwrap());
                if ti.seg.width != tj.seg.width {
                    continue;
                }
                let pinned = pins.entry(*layer).or_insert_with(|| self.pinned(*layer));
                if pinned.contains(p) {
                    continue;
                }
                let (q, r) = (self.other_end(i, ai), self.other_end(j, aj));
                let (u, v) = (*p - q, r - *p);
                if u.cross(v) != 0 || u.dot(v) <= 0 {
                    continue;
                }
                let merged = RoutedTrack {
                    seg: Segment::new(q, r, ti.seg.width),
                    ..ti
                };
                // The union is unchanged; only the geometry guard decides
                // (a near joint could become a T-junction).
                if !self.geometry_ok(*layer, &[i, j], &[merged.seg]) {
                    continue;
                }
                self.tracks[i] = Some(merged);
                self.tracks[j] = None;
                touched.insert(i);
                touched.insert(j);
                self.stats.merged += 1;
                merged_any = true;
            }
            if !merged_any {
                break;
            }
        }
        self.compact();
    }

    // ─── Redundant vias ──────────────────────────────────────────────────

    fn remove_redundant_vias(&mut self) {
        let mut k = 0;
        while k < self.vias.len() {
            if self.via_is_redundant(k) {
                self.vias.remove(k);
                self.stats.vias_removed += 1;
            } else {
                k += 1;
            }
        }
    }

    fn via_is_redundant(&mut self, k: usize) -> bool {
        let via = self.vias[k];
        let shape = Shape::Circle(konnect_geom::Circle::from_diameter(
            via.center,
            via.diameter,
        ));
        let mut used = LayerMask::EMPTY;
        let mut ends_at_centre = 0;
        for layer in via.layers.iter() {
            self.scratch.clear();
            let mut scratch = std::mem::take(&mut self.scratch);
            self.ck
                .same_net_touching(self.net, layer, &shape, &mut scratch);
            let board_touch = !scratch.is_empty();
            self.scratch = scratch;
            if board_touch {
                return false;
            }
            for t in self.tracks.iter().flatten() {
                if t.layer != layer || !touches(&shape, &Shape::Segment(t.seg)) {
                    continue;
                }
                if t.seg.a != via.center && t.seg.b != via.center {
                    return false;
                }
                used = used.with(layer);
                ends_at_centre += 1;
            }
        }
        // Another routed via of the net overlapping this one ties layers.
        let overlaps_via = self.vias.iter().enumerate().any(|(i, v)| {
            i != k
                && touches(
                    &shape,
                    &Shape::Circle(konnect_geom::Circle::from_diameter(v.center, v.diameter)),
                )
        });
        if overlaps_via || self.board_vias.contains(&via.center) {
            return false;
        }
        used.count() == 1 && ends_at_centre >= 2
    }

    // ─── Chains ──────────────────────────────────────────────────────────

    fn chains(&self) -> Vec<Chain> {
        let joints = self.joints();
        let mut pins: BTreeMap<LayerId, BTreeSet<Point>> = BTreeMap::new();
        for layer in self.layers() {
            pins.insert(layer, self.pinned(layer));
        }
        let free = |layer: LayerId, p: Point| -> Option<[(usize, bool); 2]> {
            let ends = joints.get(&(layer, p))?;
            if ends.len() != 2 || pins[&layer].contains(&p) {
                return None;
            }
            let (wi, wj) = (
                self.tracks[ends[0].0]?.seg.width,
                self.tracks[ends[1].0]?.seg.width,
            );
            (wi == wj && ends[0].0 != ends[1].0).then_some([ends[0], ends[1]])
        };
        let mut seen = vec![false; self.tracks.len()];
        let mut chains = Vec::new();
        for (slot, t) in self.tracks.iter().enumerate() {
            let Some(t) = t else { continue };
            if seen[slot] {
                continue;
            }
            seen[slot] = true;
            let layer = t.layer;
            let mut verts = vec![t.seg.a, t.seg.b];
            let mut slots = vec![slot];
            let mut cyclic = false;
            // Forward from b.
            loop {
                let tail = *verts.last().unwrap();
                let last = *slots.last().unwrap();
                let Some(ends) = free(layer, tail) else { break };
                let (next, at_a) = if ends[0].0 == last { ends[1] } else { ends[0] };
                if next == slot {
                    cyclic = true;
                    break;
                }
                if seen[next] {
                    break;
                }
                seen[next] = true;
                verts.push(self.other_end(next, at_a));
                slots.push(next);
            }
            // Backward from a.
            if !cyclic {
                loop {
                    let head = verts[0];
                    let first = slots[0];
                    let Some(ends) = free(layer, head) else { break };
                    let (prev, at_a) = if ends[0].0 == first { ends[1] } else { ends[0] };
                    if seen[prev] {
                        break;
                    }
                    seen[prev] = true;
                    verts.insert(0, self.other_end(prev, at_a));
                    slots.insert(0, prev);
                }
            }
            if !cyclic && slots.len() >= 2 {
                chains.push(Chain {
                    layer,
                    verts,
                    slots,
                });
            }
        }
        chains
    }

    // ─── Gates ───────────────────────────────────────────────────────────

    /// Defects whose position falls in `region`, with `removed` slots taken
    /// out and `added` segments put in.
    fn defects_in(
        &self,
        layer: LayerId,
        region: &BBox,
        removed: &[usize],
        added: &[Segment],
        limits: &LintLimits,
    ) -> usize {
        let wide = region.inflate(limits.jog_length_nm.saturating_add(NEAR_JOINT_NM as Coord));
        let mut segs: Vec<LintSeg> = Vec::new();
        for (i, t) in self.tracks.iter().enumerate() {
            let Some(t) = t else { continue };
            if t.layer != layer || removed.contains(&i) || !t.seg.bbox().intersects(&wide) {
                continue;
            }
            segs.push(LintSeg {
                a: t.seg.a,
                b: t.seg.b,
                width: t.seg.width,
                fixed: false,
            });
        }
        for s in added {
            segs.push(LintSeg {
                a: s.a,
                b: s.b,
                width: s.width,
                fixed: false,
            });
        }
        for (l, s) in self.fixed {
            if *l == layer && s.bbox().intersects(&wide) {
                segs.push(LintSeg {
                    a: s.a,
                    b: s.b,
                    width: s.width,
                    fixed: true,
                });
            }
        }
        lint_group(&segs, limits)
            .iter()
            .filter(|d| region.contains(d.at))
            .count()
    }

    fn region(&self, removed: &[usize], added: &[Segment], limits: &LintLimits) -> BBox {
        let mut b: Option<BBox> = None;
        let mut grow = |x: BBox| b = Some(b.map_or(x, |b| b.union(x)));
        for &i in removed {
            if let Some(t) = &self.tracks[i] {
                grow(BBox::new(t.seg.a, t.seg.b));
            }
        }
        for s in added {
            grow(BBox::new(s.a, s.b));
        }
        b.unwrap_or_else(|| BBox::point(Point::default())).inflate(
            limits
                .jog_length_nm
                .saturating_add(2 * NEAR_JOINT_NM as Coord),
        )
    }

    fn geometry_ok(&self, layer: LayerId, removed: &[usize], added: &[Segment]) -> bool {
        let limits = self.limits(layer);
        let region = self.region(removed, added, &limits);
        let before = self.defects_in(layer, &region, &[], &[], &limits);
        let after = self.defects_in(layer, &region, removed, added, &limits);
        if self.require_fewer_defects {
            after < before
        } else {
            after <= before
        }
    }

    /// Every same-net item the removed copper touched, the added copper
    /// still reaches: directly, or through a same-net anchor both touch (a
    /// pad or via never moves or refills, so it is a safe bridge).
    fn connectivity_ok(&mut self, layer: LayerId, removed: &[usize], added: &[Segment]) -> bool {
        let added: Vec<Shape> = added.iter().map(|s| Shape::Segment(*s)).collect();
        let removed_shapes: Vec<Shape> = removed
            .iter()
            .filter_map(|&i| self.tracks[i].map(|t| Shape::Segment(t.seg)))
            .collect();
        let bridges = self.anchors_touching(layer, &added);
        let reaches = |s: &Shape| {
            added.iter().any(|a| touches(a, s)) || bridges.iter().any(|b| touches(b, s))
        };
        let mut scratch = std::mem::take(&mut self.scratch);
        scratch.clear();
        for s in &removed_shapes {
            self.ck.same_net_touching(self.net, layer, s, &mut scratch);
        }
        scratch.sort_unstable();
        scratch.dedup();
        let board_ok = scratch
            .iter()
            .all(|&id| reaches(self.ck.obstacle_shape(id)));
        self.scratch = scratch;
        if !board_ok {
            return false;
        }
        for v in &self.vias {
            if !v.layers.contains(layer) {
                continue;
            }
            let vs = Shape::Circle(konnect_geom::Circle::from_diameter(v.center, v.diameter));
            if removed_shapes.iter().any(|s| touches(s, &vs)) && !reaches(&vs) {
                return false;
            }
        }
        for (i, t) in self.tracks.iter().enumerate() {
            let Some(t) = t else { continue };
            if t.layer != layer || removed.contains(&i) {
                continue;
            }
            let ts = Shape::Segment(t.seg);
            if removed_shapes.iter().any(|s| touches(s, &ts)) && !reaches(&ts) {
                return false;
            }
        }
        true
    }

    /// Same-net copper on `layer` that never moves or refills (board pads
    /// and vias, and this net's routed vias) touching any of `shapes`. A
    /// track end inside one of these is connected to it, as KiCad judges.
    fn anchors_touching(&mut self, layer: LayerId, shapes: &[Shape]) -> Vec<Shape> {
        let mut scratch = std::mem::take(&mut self.scratch);
        scratch.clear();
        for s in shapes {
            self.ck.same_net_touching(self.net, layer, s, &mut scratch);
        }
        scratch.sort_unstable();
        scratch.dedup();
        let mut out: Vec<Shape> = scratch
            .iter()
            .filter(|&&id| self.ck.obstacle_is_anchor(id))
            .map(|&id| self.ck.obstacle_shape(id).clone())
            .collect();
        self.scratch = scratch;
        for v in &self.vias {
            if !v.layers.contains(layer) {
                continue;
            }
            let disc = Shape::Circle(konnect_geom::Circle::from_diameter(v.center, v.diameter));
            if shapes.iter().any(|s| touches(s, &disc)) {
                out.push(disc);
            }
        }
        out
    }

    fn exact_ok(&mut self, layer: LayerId, added: &[Segment]) -> bool {
        added.iter().all(|s| {
            self.stats.exact_checks += 1;
            self.ck
                .track_violation(self.net, layer, s, &[self.snapshot], &|_, _| false)
                .is_none()
        })
    }

    fn accept(&mut self, layer: LayerId, removed: &[usize], added: &[Segment]) -> bool {
        let ok = self.geometry_ok(layer, removed, added)
            && self.connectivity_ok(layer, removed, added)
            && self.exact_ok(layer, added);
        if !ok {
            self.stats.rejected += 1;
        }
        ok
    }

    /// Replace `chain.slots[i..j]` (vertices `i..=j`) by a path through
    /// `mids`, when the gates pass. Returns whether it did.
    fn try_replace(&mut self, chain: &mut Chain, i: usize, j: usize, mids: &[Point]) -> bool {
        let width = self.tracks[chain.slots[i]].expect("live").seg.width;
        let gen = self.tracks[chain.slots[i]].expect("live").gen;
        let mut pts = Vec::with_capacity(mids.len() + 2);
        pts.push(chain.verts[i]);
        pts.extend_from_slice(mids);
        pts.push(chain.verts[j]);
        let added: Vec<Segment> = pts
            .windows(2)
            .map(|w| Segment::new(w[0], w[1], width))
            .collect();
        if added.iter().any(|s| s.a == s.b) {
            return false;
        }
        let removed: Vec<usize> = chain.slots[i..j].to_vec();
        if !self.accept(chain.layer, &removed, &added) {
            return false;
        }
        for &s in &removed {
            self.tracks[s] = None;
        }
        let mut new_slots = Vec::with_capacity(added.len());
        for seg in added {
            new_slots.push(self.tracks.len());
            self.tracks.push(Some(RoutedTrack {
                net: self.net,
                layer: chain.layer,
                seg,
                gen,
            }));
        }
        chain.verts.splice(i + 1..j, mids.iter().copied());
        chain.slots.splice(i..j, new_slots);
        true
    }

    // ─── Jog smoothing ───────────────────────────────────────────────────

    /// A short step between two parallel runs (a jog the lint refuses) is
    /// redrawn as one gentle slant: its bends stay under the lint's 20°, so
    /// it is no longer a jog. Only taken when it removes a defect.
    fn smooth_jogs(&mut self) {
        const ANGLES_DEG: [f64; 2] = [18.0, 12.0];
        self.require_fewer_defects = true;
        for mut chain in self.chains() {
            let limits = self.limits(chain.layer);
            let mut k = 0;
            while k + 1 < chain.verts.len() {
                let n = chain.verts.len() - 1;
                let (a, b) = (chain.verts[k], chain.verts[k + 1]);
                let has_in = k >= 1;
                let has_out = k + 2 <= n;
                let jog = (b - a).length2()
                    < i128::from(limits.jog_length_nm) * i128::from(limits.jog_length_nm);
                if !jog || !(has_in || has_out) {
                    k += 1;
                    continue;
                }
                // The run direction: the incoming run, else the outgoing.
                let run = if has_in {
                    a - chain.verts[k - 1]
                } else {
                    chain.verts[k + 2] - b
                };
                let rl = run.length_f64();
                let (ux, uy) = (run.x as f64 / rl, run.y as f64 / rl);
                let step = b - a;
                let h = (ux * step.y as f64 - uy * step.x as f64).abs();
                let p0 = ux * step.x as f64 + uy * step.y as f64;
                let len_in = if has_in {
                    len_f(chain.verts[k - 1], a)
                } else {
                    0.0
                };
                let len_out = if has_out {
                    len_f(b, chain.verts[k + 2])
                } else {
                    0.0
                };
                let mut done = false;
                for deg in ANGLES_DEG {
                    let extra = h / deg.to_radians().tan() - p0;
                    if h < 1.0 || extra <= 0.0 {
                        continue;
                    }
                    let (mut s1, mut s2) = match (has_in, has_out) {
                        (true, true) => (extra / 2.0, extra / 2.0),
                        (true, false) => (extra, 0.0),
                        _ => (0.0, extra),
                    };
                    // Give what one run cannot take to the other.
                    let room_in = (len_in - 1_000.0).max(0.0);
                    let room_out = (len_out - 1_000.0).max(0.0);
                    if s1 > room_in {
                        s2 += s1 - room_in;
                        s1 = room_in;
                    }
                    if s2 > room_out {
                        s1 += s2 - room_out;
                        s2 = room_out;
                    }
                    if s1 > room_in + 0.5 || s2 > room_out + 0.5 {
                        continue;
                    }
                    let at = |p: Point, s: f64| {
                        Point::new(
                            (f64::from(p.x) + ux * s).round() as Coord,
                            (f64::from(p.y) + uy * s).round() as Coord,
                        )
                    };
                    let mut mids = Vec::new();
                    let (i, j) = (
                        if s1 > 0.5 { k - 1 } else { k },
                        if s2 > 0.5 { k + 2 } else { k + 1 },
                    );
                    if s1 > 0.5 {
                        mids.push(at(a, -s1));
                    }
                    if s2 > 0.5 {
                        mids.push(at(b, s2));
                    }
                    if self.try_replace(&mut chain, i, j, &mids) {
                        self.stats.jogs_smoothed += 1;
                        done = true;
                        break;
                    }
                }
                k += if done { 2 } else { 1 };
            }
        }
        self.require_fewer_defects = false;
        self.compact();
    }

    // ─── Junctions ───────────────────────────────────────────────────────

    /// A branch meeting a straight run at a sharp angle (a Y) is moved to
    /// the foot of its perpendicular on the run (a T): two right angles
    /// instead of one acute one, and a shorter branch.
    fn square_junctions(&mut self) {
        let joints = self.joints();
        let mut touched: BTreeSet<usize> = BTreeSet::new();
        for ((layer, p), ends) in &joints {
            if ends.len() < 3 || ends.iter().any(|(s, _)| touched.contains(s)) {
                continue;
            }
            let limits = self.limits(*layer);
            let live: Vec<(usize, Point)> = ends
                .iter()
                .filter_map(|&(s, at_a)| {
                    let t = self.tracks[s]?;
                    (t.seg.a != t.seg.b).then(|| (s, self.other_end(s, at_a)))
                })
                .collect();
            'pairs: for x in 0..live.len() {
                for y in 0..live.len() {
                    if x == y
                        || lint::interior_angle_deg(*p, live[x].1, live[y].1)
                            >= limits.min_bend_angle_deg - 1e-9
                    {
                        continue;
                    }
                    let ((bs, b), (rs, far)) = (live[x], live[y]);
                    let run = far - *p;
                    if !is_octilinear(run) {
                        continue;
                    }
                    let e = Vector::new(run.x.signum(), run.y.signum());
                    let n = run.x.abs().max(run.y.abs());
                    let l2 = run.length2();
                    let t = (b - *p).dot(run) as f64 / l2 as f64;
                    let k = (t * n as f64).round() as i64;
                    if k <= 0 || k >= n {
                        continue;
                    }
                    let foot = add(*p, Vector::new(e.x * k, e.y * k));
                    let (bt, rt) = (
                        self.tracks[bs].expect("live"),
                        self.tracks[rs].expect("live"),
                    );
                    let added = [
                        Segment::new(b, foot, bt.seg.width),
                        Segment::new(*p, foot, rt.seg.width),
                        Segment::new(foot, far, rt.seg.width),
                    ];
                    if added.iter().any(|s| s.a == s.b)
                        || len_f(b, foot) >= len_f(b, *p)
                        || !self.accept(*layer, &[bs, rs], &added)
                    {
                        continue;
                    }
                    self.tracks[bs] = Some(RoutedTrack {
                        seg: added[0],
                        ..bt
                    });
                    self.tracks[rs] = Some(RoutedTrack {
                        seg: added[1],
                        ..rt
                    });
                    self.tracks.push(Some(RoutedTrack {
                        seg: added[2],
                        ..rt
                    }));
                    touched.insert(bs);
                    touched.insert(rs);
                    self.stats.squared += 1;
                    break 'pairs;
                }
            }
        }
    }

    // ─── Pad entries ─────────────────────────────────────────────────────

    /// Tracks meeting at a sharp angle inside a same-net pad: one of them
    /// ends short, still inside the pad, so they no longer share a joint.
    /// KiCad connects a track whose end lies in a pad, so nothing is lost.
    fn trim_pad_entries(&mut self) {
        let joints = self.joints();
        for ((layer, p), ends) in &joints {
            if ends.len() < 2 {
                continue;
            }
            let limits = self.limits(*layer);
            let live: Vec<(usize, bool, Point)> = ends
                .iter()
                .filter_map(|&(s, at_a)| {
                    let t = self.tracks[s]?;
                    let end = if at_a { t.seg.a } else { t.seg.b };
                    (end == *p && t.seg.a != t.seg.b).then(|| (s, at_a, self.other_end(s, at_a)))
                })
                .collect();
            let sharp: Vec<usize> = (0..live.len())
                .filter(|&x| {
                    (0..live.len()).any(|y| {
                        y != x
                            && lint::interior_angle_deg(*p, live[x].2, live[y].2)
                                < limits.min_bend_angle_deg - 1e-9
                    })
                })
                .collect();
            if sharp.is_empty() {
                continue;
            }
            let dot = Shape::Circle(konnect_geom::Circle::new(*p, 0));
            let anchors = self.anchors_touching(*layer, std::slice::from_ref(&dot));
            if anchors.is_empty() {
                continue;
            }
            'ends: for &x in &sharp {
                let (slot, at_a, far) = live[x];
                let t = self.tracks[slot].expect("live");
                let len = len_f(*p, far);
                let w = f64::from(t.seg.width.max(1));
                for d in [w, w / 2.0, w / 4.0] {
                    if d + 1_000.0 >= len {
                        continue;
                    }
                    let f = d / len;
                    let q = Point::new(
                        (f64::from(p.x) + f64::from(far.x - p.x) * f).round() as Coord,
                        (f64::from(p.y) + f64::from(far.y - p.y) * f).round() as Coord,
                    );
                    let q_dot = Shape::Circle(konnect_geom::Circle::new(q, 0));
                    if q == *p || !anchors.iter().any(|a| touches(a, &q_dot)) {
                        continue;
                    }
                    let seg = if at_a {
                        Segment::new(q, t.seg.b, t.seg.width)
                    } else {
                        Segment::new(t.seg.a, q, t.seg.width)
                    };
                    // The shorter track is a subset of the old copper, so no
                    // clearance can get worse; only geometry and
                    // connectivity can.
                    if self.geometry_ok(*layer, &[slot], &[seg])
                        && self.connectivity_ok(*layer, &[slot], &[seg])
                    {
                        self.tracks[slot] = Some(RoutedTrack { seg, ..t });
                        self.stats.trimmed += 1;
                        break 'ends;
                    }
                    self.stats.rejected += 1;
                }
            }
        }
    }

    // ─── Pull-tight ──────────────────────────────────────────────────────

    fn pull_tight(&mut self) {
        for mut chain in self.chains() {
            let mut i = 0;
            while i + 2 < chain.verts.len() {
                let n = chain.verts.len() - 1;
                let jmax = n.min(i + self.opts.window.max(2));
                let mut done = false;
                for j in (i + 2..=jmax).rev() {
                    let old_len: f64 = chain.verts[i..=j]
                        .windows(2)
                        .map(|w| len_f(w[0], w[1]))
                        .sum();
                    let prev = (i > 0).then(|| chain.verts[i - 1]);
                    let next = chain.verts.get(j + 1).copied();
                    for mids in shortcuts(
                        chain.verts[i],
                        chain.verts[j],
                        self.opts.octilinear,
                        prev,
                        next,
                        self.opts.max_skew_deg,
                    ) {
                        let mut pts = vec![chain.verts[i]];
                        pts.extend_from_slice(&mids);
                        pts.push(chain.verts[j]);
                        let new_len: f64 = pts.windows(2).map(|w| len_f(w[0], w[1])).sum();
                        let fewer = pts.len() - 1 < j - i;
                        if !(new_len < old_len - 0.5 || (new_len <= old_len + 0.5 && fewer)) {
                            continue;
                        }
                        if self.try_replace(&mut chain, i, j, &mids) {
                            self.stats.pulled += 1;
                            done = true;
                            break;
                        }
                    }
                    if done {
                        break;
                    }
                }
                if !done {
                    i += 1;
                }
            }
        }
        self.compact();
    }

    // ─── Corner cuts ─────────────────────────────────────────────────────

    fn corner_cuts(&mut self) {
        for mut chain in self.chains() {
            let mut k = 1;
            while k + 1 < chain.verts.len() {
                let (p, v, q) = (chain.verts[k - 1], chain.verts[k], chain.verts[k + 1]);
                let (u1, u2) = (v - p, q - v);
                if !(is_octilinear(u1) && is_octilinear(u2)) || u1.dot(u2) != 0 {
                    k += 1;
                    continue;
                }
                let width = self.tracks[chain.slots[k]].expect("live").seg.width;
                let mut cut = false;
                for (q1, q2) in chamfers(p, v, q, width) {
                    if self.try_replace(&mut chain, k - 1, k + 1, &[q1, q2]) {
                        self.stats.corners_cut += 1;
                        cut = true;
                        break;
                    }
                }
                k += if cut { 2 } else { 1 };
            }
        }
        self.compact();
    }
}

/// Whether `p` lies on the open segment `a`–`b`.
fn strictly_inside(p: Point, a: Point, b: Point) -> bool {
    let (ab, ap) = (b - a, p - a);
    if ab.cross(ap) != 0 {
        return false;
    }
    let t = ab.dot(ap);
    t > 0 && t < ab.length2()
}

/// Octilinear ways from `a` to `b`, as lists of intermediate vertices,
/// best first: the one adding the fewest bends with the neighbours.
fn shortcuts(
    a: Point,
    b: Point,
    octilinear: bool,
    prev: Option<Point>,
    next: Option<Point>,
    max_skew_deg: f64,
) -> Vec<Vec<Point>> {
    let d = b - a;
    if d.x == 0 && d.y == 0 {
        return Vec::new();
    }
    if !octilinear || is_octilinear(d) {
        return vec![Vec::new()];
    }
    let m = d.x.abs().min(d.y.abs());
    let diag = Vector::new(d.x.signum() * m, d.y.signum() * m);
    let straight = Vector::new(d.x - diag.x, d.y - diag.y);
    let score = |mid: Point| {
        let mut n = 0;
        if let Some(p) = prev {
            n += usize::from(bends(p, a, mid));
        }
        if let Some(r) = next {
            n += usize::from(bends(mid, b, r));
        }
        n
    };
    let mut options = vec![vec![add(a, diag)], vec![add(a, straight)]];
    options.sort_by_key(|o| score(o[0]));
    // Within a micrometre of octilinear (rounding left by another tool), or
    // within `max_skew_deg` of it over a long run, one straight piece beats
    // a dogleg: its bends are below any lint threshold.
    let long = d.x.abs().max(d.y.abs());
    let (off, along) = if 2 * m <= long {
        (m, long)
    } else {
        (long - m, m)
    };
    if off <= SNAP_NM || (off as f64) <= (along as f64) * max_skew_deg.to_radians().tan() {
        options.insert(0, Vec::new());
    }
    options
}

/// Chamfers of the right-angle corner `p`–`v`–`q`, largest first, as the
/// two new vertices on the incoming and outgoing legs.
fn chamfers(p: Point, v: Point, q: Point, width: Coord) -> Vec<(Point, Point)> {
    let (u1, u2) = (v - p, q - v);
    let unit = |u: Vector| Vector::new(u.x.signum(), u.y.signum());
    let (e1, e2) = (unit(u1), unit(u2));
    // Steps along each leg (a diagonal step is one nm in x and in y).
    let steps = |u: Vector| u.x.abs().max(u.y.abs());
    let (n1, n2) = (steps(u1), steps(u2));
    let step_nm = |e: Vector| {
        if e.x != 0 && e.y != 0 {
            2f64.sqrt()
        } else {
            1.0
        }
    };
    let leg_max = n1.min(n2);
    let w = f64::from(width.max(1));
    let mut wanted: Vec<i64> = [
        (leg_max as f64) / 2.0,
        4.0 * w / step_nm(e1),
        2.0 * w / step_nm(e1),
        w / step_nm(e1),
        w / (2.0 * step_nm(e1)),
    ]
    .iter()
    .map(|&s| s.floor() as i64)
    .filter(|&s| s > 0 && s < leg_max)
    .collect();
    wanted.sort_unstable_by(|a, b| b.cmp(a));
    wanted.dedup();
    wanted
        .into_iter()
        .map(|k| {
            (
                add(v, Vector::new(-e1.x * k, -e1.y * k)),
                add(v, Vector::new(e2.x * k, e2.y * k)),
            )
        })
        .collect()
}

#[cfg(test)]
mod tests;
