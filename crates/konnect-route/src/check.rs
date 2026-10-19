//! S7 internal exact DRC (owned by WS-A).
//!
//! Checks copper with the exact kernel ([`konnect_geom`]) and the compiled
//! [`RuleTable`]:
//!
//! | Check | Pairs | Rule |
//! |---|---|---|
//! | copper clearance, shorts | same-layer copper of different nets | [`RuleTable::clearance`], both A/B orders, courtyard and area conditions |
//! | hole clearance | copper against another item's drilled hole (plated holes of the same net excepted; NPTH always) | [`RuleTable::hole_clearance`] |
//! | hole to hole | every pair of drilled holes | [`RuleTable::hole_to_hole`] |
//! | edge clearance | copper against `Edge.Cuts` | [`RuleTable::edge_clearance`] |
//! | keepouts | copper the keepout forbids, touching it | the keepout's own flags |
//! | track width | each track and arc | [`RuleTable::track_width`] `min` |
//! | via size | each via: diameter, drill, annular ring | [`RuleTable::via`] floors |
//! | segment length | each track and arc | [`RuleTable::min_segment_length`] |
//! | track angle | every pair of track ends of one net meeting at one point on one layer, off pads | [`RuleTable::min_track_angle_deg`] |
//!
//! # Exactness and margins
//!
//! Distances are decided by the exact integer predicates; nothing that
//! decides legality is rounded. Two deliberate biases make the checker
//! strict, never lenient, against KiCad:
//!
//! - **Curved primitives.** Arc tracks are lowered to inscribed chords
//!   ([`konnect_geom::ArcSide::Inscribed`]), whose true curve can bulge up to
//!   `arc_policy.max_error_nm` past them, so that error is added to every
//!   clearance an arc chord is checked against.
//! - **Safety margin.** [`CheckOptions::safety_margin_nm`] (1–5 µm, off by
//!   default) is added to every clearance-type requirement: copper, hole,
//!   hole-to-hole and edge. KiCad itself only flags a gap below the rule
//!   minus its DRC epsilon, so the exact check alone never passes what
//!   KiCad fails on straight geometry.
//!
//! Two options reproduce KiCad's verdicts instead:
//! [`CheckOptions::kicad_epsilon_nm`] subtracts KiCad's DRC epsilon
//! ([`CheckOptions::kicad_equivalent`]), and
//! [`CheckOptions::check_zone_fills`] off leaves plane pieces to KiCad's
//! refill, as [`check_routed`] does: routed copper is applied first and the
//! zones refill around it.
//!
//! # Agreement with KiCad
//!
//! Precedence and the less obvious semantics were settled against a saved
//! KiCad DRC of a routed board (`tests/check_oracle.rs`, run on fixtures
//! outside the repository): every error in the clearance, hole clearance,
//! track angle and segment length categories is reproduced by item
//! identity, measured values agree within KiCad's printed precision, and
//! nothing else is flagged except true violations KiCad's report leaves
//! out. Two KiCad behaviours matter when reading the reports side by side:
//!
//! - KiCad keeps only the first clearance error per track or via unless
//!   "report all track errors" is on; this checker reports every pair.
//! - KiCad reports a bend once from each of its tracks; this checker
//!   reports it once.
//!
//! # Finding pairs
//!
//! Copper is split into per-layer objects (a via or a through-hole pad on
//! each of its layers, an arc into its chords, a pad into its shapes) held
//! in one `rstar` index per layer ([`LayerIndex`]). Each object queries its
//! box inflated by the largest clearance the table can return; plane pieces
//! use an edge-indexed polygon ([`IndexedPolygon`]) so a test visits only
//! the edges in reach. Objects are checked in parallel with rayon; the
//! result is sorted and de-duplicated, so 1-thread and N-thread runs give
//! identical reports.
//!
//! # Rule subjects
//!
//! Each item's [`RuleSubject`] carries the footprints whose courtyard its
//! copper touches (KiCad's `intersectsCourtyard()`, either side) and the
//! named rule areas that wholly contain it on a shared layer
//! (`enclosedByArea()`). Subjects are interned, so the rule table is asked
//! once per distinct pair of subjects and layer, per worker.

use crate::api::{FootprintId, ItemKind, ItemRef, LayerId, NetId};
use crate::board::{Pad, PadKind, RouteBoard};
use crate::plan::RoutedCopper;
use crate::rules::{RuleSubject, RuleTable};
use crate::telemetry::Telemetry;
use konnect_geom::{
    closest, gap_cmp, ArcPolicy, BBox, Circle, Coord, IndexEntry, IndexedPolygon, LayerIndex,
    Point, Segment, Shape,
};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::cmp::Ordering;
use std::collections::{BTreeMap, HashMap};
use std::time::Instant;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ViolationKind {
    Clearance,
    Short,
    TrackWidth,
    ViaSize,
    HoleClearance,
    HoleToHole,
    EdgeClearance,
    TrackAngle,
    SegmentLength,
    Keepout,
}

impl ViolationKind {
    fn order(self) -> u8 {
        self as u8
    }
}

/// One side of a violation: a board item or a routed one.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(tag = "source", rename_all = "snake_case")]
pub enum CheckedItem {
    Board { item: ItemRef },
    RoutedTrack { index: u32 },
    RoutedVia { index: u32 },
    Edge,
}

impl CheckedItem {
    /// A total order for sorting reports.
    fn key(self) -> (u8, u8, u32) {
        match self {
            CheckedItem::Board { item } => {
                let (k, i) = match item {
                    ItemRef::Pad(id) => (0, id.0),
                    ItemRef::Track(id) => (1, id.0),
                    ItemRef::Arc(id) => (2, id.0),
                    ItemRef::Via(id) => (3, id.0),
                    ItemRef::Zone(id) => (4, id.0),
                };
                (0, k, i)
            }
            CheckedItem::RoutedTrack { index } => (1, 0, index),
            CheckedItem::RoutedVia { index } => (2, 0, index),
            CheckedItem::Edge => (3, 0, 0),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Violation {
    pub kind: ViolationKind,
    pub a: CheckedItem,
    pub b: Option<CheckedItem>,
    pub layer: Option<LayerId>,
    /// Where to look.
    pub at: Point,
    /// The rule's value, nm (or tenths of a degree for angles).
    pub required: Coord,
}

impl Violation {
    fn sort_key(&self) -> impl Ord {
        (
            self.kind.order(),
            self.a.key(),
            self.b.map(CheckedItem::key),
            self.layer,
            self.at,
            self.required,
        )
    }

    /// Violations that name the same items for the same reason are one.
    fn same_finding(&self, o: &Violation) -> bool {
        self.kind == o.kind && self.a == o.a && self.b == o.b
    }
}

/// Deterministic counts of what a check did (no timings: those go to
/// [`Telemetry`], so equal inputs give equal reports).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct CheckStats {
    /// Per-layer copper objects checked (both sides of every pair).
    pub objects: u64,
    /// Drilled holes.
    pub holes: u64,
    /// Index candidates that reached a geometric test.
    pub candidate_pairs: u64,
    /// Exact tests run against a resolved rule value.
    pub exact_tests: u64,
    /// Unfilled zone pieces left out: their outline only bounds the copper.
    pub unfilled_zones_skipped: u64,
}

#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct CheckReport {
    pub violations: Vec<Violation>,
    pub stats: CheckStats,
}

impl CheckReport {
    /// Violations per kind, for summaries.
    pub fn counts(&self) -> BTreeMap<ViolationKind, usize> {
        let mut out = BTreeMap::new();
        for v in &self.violations {
            *out.entry(v.kind).or_default() += 1;
        }
        out
    }
}

impl PartialOrd for ViolationKind {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for ViolationKind {
    fn cmp(&self, other: &Self) -> Ordering {
        self.order().cmp(&other.order())
    }
}

/// The smallest internal safety margin [`CheckOptions::with_safety_margin_nm`]
/// accepts: 1 µm.
pub const SAFETY_MARGIN_MIN_NM: Coord = 1_000;
/// The largest: 5 µm.
pub const SAFETY_MARGIN_MAX_NM: Coord = 5_000;

/// KiCad's DRC epsilon (`ADVANCED_CFG::m_DRCEpsilon`, 0.5 µm): KiCad
/// flags a clearance-type gap only when it is below the rule minus this.
pub const KICAD_DRC_EPSILON_NM: Coord = 500;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CheckOptions {
    /// Added to every clearance-type requirement (copper, hole, hole to
    /// hole, edge). 0 is the exact rule; the router's acceptance check may
    /// use 1–5 µm so near-threshold copper never reaches KiCad.
    pub safety_margin_nm: Coord,
    /// Subtracted from every clearance-type requirement to reproduce
    /// KiCad's verdicts exactly ([`KICAD_DRC_EPSILON_NM`]). 0, the default,
    /// is strict: a gap even a nanometre under the rule is flagged.
    pub kicad_epsilon_nm: Coord,
    /// Check zone fills as copper. Turn it off when the fills are stale and
    /// KiCad will refill before its own DRC (routed copper is applied, then
    /// zones refill around it), so a fill overlapping new copper is not an
    /// error.
    pub check_zone_fills: bool,
    /// How arc tracks are lowered; its error is added to their clearances.
    pub arc_policy: ArcPolicy,
}

impl Default for CheckOptions {
    fn default() -> Self {
        CheckOptions {
            safety_margin_nm: 0,
            kicad_epsilon_nm: 0,
            check_zone_fills: true,
            arc_policy: ArcPolicy::default(),
        }
    }
}

impl CheckOptions {
    /// Use a safety margin, clamped to 1–5 µm.
    pub fn with_safety_margin_nm(mut self, nm: Coord) -> Self {
        self.safety_margin_nm = nm.clamp(SAFETY_MARGIN_MIN_NM, SAFETY_MARGIN_MAX_NM);
        self
    }

    /// Reproduce KiCad's own verdicts: its DRC epsilon, no margin.
    pub fn kicad_equivalent(mut self) -> Self {
        self.safety_margin_nm = 0;
        self.kicad_epsilon_nm = KICAD_DRC_EPSILON_NM;
        self
    }

    /// The options [`check_routed`] uses: strict, and zone fills left to
    /// KiCad's refill after the copper is applied.
    pub fn for_routed() -> Self {
        CheckOptions {
            check_zone_fills: false,
            ..CheckOptions::default()
        }
    }
}

/// Check `copper` against `board` and itself, with
/// [`CheckOptions::for_routed`].
pub fn check_routed(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    copper: &RoutedCopper,
) -> CheckReport {
    BoardChecker::new(board, rules, CheckOptions::for_routed()).check_routed(copper)
}

/// Check every item of `board` against every other: a whole-board DRC of
/// the existing copper.
pub fn check_board(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    options: CheckOptions,
) -> CheckReport {
    BoardChecker::new(board, rules, options).check_board()
}

// ─── The checker ─────────────────────────────────────────────────────────────

/// Where a per-layer object came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
enum Src {
    Board(ItemRef),
    RoutedTrack(u32),
    RoutedVia(u32),
}

impl Src {
    fn checked(self) -> CheckedItem {
        match self {
            Src::Board(item) => CheckedItem::Board { item },
            Src::RoutedTrack(index) => CheckedItem::RoutedTrack { index },
            Src::RoutedVia(index) => CheckedItem::RoutedVia { index },
        }
    }

    fn kind(self) -> ItemKind {
        match self {
            Src::Board(item) => item.kind(),
            Src::RoutedTrack(_) => ItemKind::Track,
            Src::RoutedVia(_) => ItemKind::Via,
        }
    }

    fn is_routed(self) -> bool {
        !matches!(self, Src::Board(_))
    }
}

#[derive(Debug, Clone)]
enum Geom {
    Shape(Shape),
    /// A plane piece: an index into the checker's edge-indexed polygons.
    Zone(u32),
}

/// One item's copper on one layer (or one of its shapes there).
#[derive(Debug, Clone)]
struct Obj {
    src: Src,
    net: Option<NetId>,
    subject: u32,
    geom: Geom,
    bbox: BBox,
    /// Added to every clearance this object is checked against (the arc
    /// chord error).
    extra: Coord,
}

#[derive(Debug, Clone, Default)]
struct LayerObjs {
    objs: Vec<Obj>,
    index: LayerIndex,
}

impl LayerObjs {
    fn finish(objs: Vec<Obj>) -> Self {
        let entries = objs
            .iter()
            .enumerate()
            .map(|(i, o)| IndexEntry::new(i as u32, o.bbox))
            .collect();
        LayerObjs {
            index: LayerIndex::bulk_load(entries),
            objs,
        }
    }
}

#[derive(Debug, Clone)]
struct Hole {
    src: Src,
    net: Option<NetId>,
    plated: bool,
    shape: Shape,
    bbox: BBox,
    layers: crate::api::LayerMask,
}

#[derive(Debug, Clone)]
struct Holes {
    holes: Vec<Hole>,
    index: LayerIndex,
}

impl Holes {
    fn finish(holes: Vec<Hole>) -> Self {
        let entries = holes
            .iter()
            .enumerate()
            .map(|(i, h)| IndexEntry::new(i as u32, h.bbox))
            .collect();
        Holes {
            index: LayerIndex::bulk_load(entries),
            holes,
        }
    }
}

/// An interned rule subject.
#[derive(Debug, Clone)]
struct SubjectData<'b> {
    kind: ItemKind,
    net: Option<NetId>,
    footprint: Option<FootprintId>,
    courtyards: Vec<FootprintId>,
    areas: Vec<&'b str>,
    local: Option<Coord>,
}

impl SubjectData<'_> {
    fn view(&self) -> RuleSubject<'_> {
        RuleSubject {
            kind: self.kind,
            net: self.net,
            footprint: self.footprint,
            courtyards: &self.courtyards,
            areas: &self.areas,
            local_clearance: self.local,
        }
    }
}

type SubjectKey = (
    u8,
    Option<u32>,
    Option<u32>,
    Vec<u32>,
    Vec<u32>,
    Option<Coord>,
);

fn kind_code(k: ItemKind) -> u8 {
    match k {
        ItemKind::Track => 0,
        ItemKind::Arc => 1,
        ItemKind::Via => 2,
        ItemKind::Pad => 3,
        ItemKind::Zone => 4,
    }
}

#[derive(Debug, Default)]
struct Interner<'b> {
    by_key: BTreeMap<SubjectKey, u32>,
    data: Vec<SubjectData<'b>>,
}

impl<'b> Interner<'b> {
    fn intern(&mut self, d: SubjectData<'b>, area_ids: Vec<u32>) -> u32 {
        let key = (
            kind_code(d.kind),
            d.net.map(|n| n.0),
            d.footprint.map(|f| f.0),
            d.courtyards.iter().map(|f| f.0).collect(),
            area_ids,
            d.local,
        );
        if let Some(&id) = self.by_key.get(&key) {
            return id;
        }
        let id = self.data.len() as u32;
        self.data.push(d);
        self.by_key.insert(key, id);
        id
    }
}

/// A named rule area `enclosedByArea()` can test.
#[derive(Debug, Clone)]
struct Area<'b> {
    name: &'b str,
    /// Set when the area is a zone's own piece: a zone is never enclosed
    /// by itself (KiCad skips `area == item`).
    zone_name: Option<&'b str>,
    layers: crate::api::LayerMask,
    region: IndexedPolygon,
    bbox: BBox,
}

/// The board side of a check, built once: objects, holes, edges, subjects
/// and indexes. Reuse one across many [`BoardChecker::check_routed`] calls.
pub struct BoardChecker<'b> {
    board: &'b RouteBoard,
    rules: &'b dyn RuleTable,
    options: CheckOptions,
    layers: Vec<LayerObjs>,
    holes: Holes,
    edges: Vec<Shape>,
    edge_index: LayerIndex,
    zones: Vec<IndexedPolygon>,
    courtyards: Vec<(FootprintId, Shape)>,
    courtyard_index: LayerIndex,
    areas: Vec<Area<'b>>,
    subjects: Vec<SubjectData<'b>>,
    /// Each board item's interned subject, by item kind and index.
    item_subjects: ItemSubjects,
    /// Largest clearance plus margin and arc error: the query reach.
    reach: i64,
    unfilled_zones: u64,
}

#[derive(Debug, Clone, Default)]
struct ItemSubjects {
    tracks: Vec<u32>,
    arcs: Vec<u32>,
    vias: Vec<u32>,
    pads: Vec<u32>,
    zones: Vec<u32>,
}

/// The subject inputs of one item: its kind, net, footprint, local
/// clearance, copper layers and shapes.
struct ItemSpec<'s> {
    kind: ItemKind,
    net: Option<NetId>,
    footprint: Option<FootprintId>,
    local: Option<Coord>,
    layers: crate::api::LayerMask,
    shapes: Vec<&'s Shape>,
    zone: Option<u32>,
}

impl<'b> BoardChecker<'b> {
    pub fn new(board: &'b RouteBoard, rules: &'b dyn RuleTable, options: CheckOptions) -> Self {
        let mut c = BoardChecker {
            board,
            rules,
            options,
            layers: Vec::new(),
            holes: Holes::finish(Vec::new()),
            edges: board
                .outline
                .edges
                .iter()
                .map(|s| Shape::Segment(*s))
                .collect(),
            edge_index: LayerIndex::new(),
            zones: Vec::new(),
            courtyards: Vec::new(),
            courtyard_index: LayerIndex::new(),
            areas: Vec::new(),
            subjects: Vec::new(),
            item_subjects: ItemSubjects::default(),
            reach: 0,
            unfilled_zones: 0,
        };
        c.edge_index = LayerIndex::bulk_load(
            c.edges
                .iter()
                .enumerate()
                .map(|(i, s)| IndexEntry::new(i as u32, s.bbox()))
                .collect(),
        );
        for f in &board.footprints {
            for court in &f.courtyards {
                c.courtyards
                    .push((f.id, Shape::Polygon(court.region.clone())));
            }
        }
        c.courtyard_index = LayerIndex::bulk_load(
            c.courtyards
                .iter()
                .enumerate()
                .map(|(i, (_, s))| IndexEntry::new(i as u32, s.bbox()))
                .collect(),
        );
        for k in &board.keepouts {
            if let Some(name) = k.name.as_deref() {
                c.areas.push(Area {
                    name,
                    zone_name: None,
                    layers: k.layers,
                    bbox: k.region.bbox(),
                    region: IndexedPolygon::new(k.region.clone()),
                });
            }
        }
        for z in &board.zones {
            if let Some(name) = z.zone_name.as_deref() {
                c.areas.push(Area {
                    name,
                    zone_name: Some(name),
                    layers: crate::api::LayerMask::single(z.layer),
                    bbox: z.region.bbox(),
                    region: IndexedPolygon::new(z.region.clone()),
                });
            }
        }
        c.zones = board
            .zones
            .par_iter()
            .map(|z| IndexedPolygon::new(z.region.clone()))
            .collect();
        c.reach = i64::from(rules.max_clearance())
            + i64::from(options.safety_margin_nm)
            + i64::from(options.arc_policy.max_error_nm.max(0));
        c.build_board_objects();
        c
    }

    /// Added to clearance-type requirements: the safety margin less the
    /// KiCad epsilon (negative when emulating KiCad).
    fn margin(&self) -> i64 {
        i64::from(self.options.safety_margin_nm.max(0))
            - i64::from(self.options.kicad_epsilon_nm.max(0))
    }

    fn arc_extra(&self) -> Coord {
        self.options.arc_policy.max_error_nm.max(0)
    }

    // ─── Building ────────────────────────────────────────────────────────

    /// Footprints whose courtyard the item's copper touches, sorted.
    fn courtyards_of(&self, spec: &ItemSpec<'_>, bbox: BBox) -> Vec<FootprintId> {
        let mut out: Vec<FootprintId> = Vec::new();
        for e in self.courtyard_index.query_entries(&bbox) {
            let (fp, court) = &self.courtyards[e.id as usize];
            if out.contains(fp) {
                continue;
            }
            let hit = match spec.zone {
                Some(z) => self.zones[z as usize].touches(court),
                None => spec.shapes.iter().any(|s| konnect_geom::touches(s, court)),
            };
            if hit {
                out.push(*fp);
            }
        }
        out.sort_unstable();
        out
    }

    /// Named areas wholly containing the item on a shared layer.
    fn areas_of(&self, spec: &ItemSpec<'_>, bbox: BBox) -> Vec<u32> {
        let mut out = Vec::new();
        for (i, a) in self.areas.iter().enumerate() {
            if !a.layers.intersects(spec.layers) || !a.bbox.contains_bbox(&bbox) {
                continue;
            }
            let inside = match spec.zone {
                Some(z) => {
                    let own = self.board.zones[z as usize].zone_name.as_deref();
                    (a.zone_name.is_none() || a.zone_name != own)
                        && a.region.contains_polygon(&self.zones[z as usize])
                }
                None => !spec.shapes.is_empty() && spec.shapes.iter().all(|s| a.region.contains(s)),
            };
            if inside {
                out.push(i as u32);
            }
        }
        out
    }

    fn subject_of(&self, spec: &ItemSpec<'_>) -> (SubjectData<'b>, Vec<u32>) {
        let bbox = match spec.zone {
            Some(z) => Some(self.zones[z as usize].bbox()),
            None => spec.shapes.iter().map(|s| s.bbox()).reduce(BBox::union),
        };
        let (courtyards, areas) = match bbox {
            Some(b) => (self.courtyards_of(spec, b), self.areas_of(spec, b)),
            None => (Vec::new(), Vec::new()),
        };
        let names = areas.iter().map(|&i| self.areas[i as usize].name).collect();
        (
            SubjectData {
                kind: spec.kind,
                net: spec.net,
                footprint: spec.footprint,
                courtyards,
                areas: names,
                local: spec.local,
            },
            areas,
        )
    }

    fn build_board_objects(&mut self) {
        let b = self.board;
        let policy = self.options.arc_policy;
        let arc_extra = self.arc_extra();
        // Lowered geometry first, so subjects see the same shapes as checks.
        let arc_shapes: Vec<Vec<Shape>> = b
            .arcs
            .par_iter()
            .map(|a| {
                a.segments(&policy)
                    .into_iter()
                    .map(Shape::Segment)
                    .collect()
            })
            .collect();
        let track_shapes: Vec<Shape> = b.tracks.iter().map(|t| Shape::Segment(t.seg)).collect();
        let via_shapes: Vec<Shape> = b.vias.iter().map(|v| v.shape()).collect();

        let mut specs: Vec<ItemSpec<'_>> = Vec::new();
        for (t, s) in b.tracks.iter().zip(&track_shapes) {
            specs.push(ItemSpec {
                kind: ItemKind::Track,
                net: t.net,
                footprint: None,
                local: None,
                layers: crate::api::LayerMask::single(t.layer),
                shapes: vec![s],
                zone: None,
            });
        }
        for (a, ss) in b.arcs.iter().zip(&arc_shapes) {
            specs.push(ItemSpec {
                kind: ItemKind::Arc,
                net: a.net,
                footprint: None,
                local: None,
                layers: crate::api::LayerMask::single(a.layer),
                shapes: ss.iter().collect(),
                zone: None,
            });
        }
        for (v, s) in b.vias.iter().zip(&via_shapes) {
            specs.push(ItemSpec {
                kind: ItemKind::Via,
                net: v.net,
                footprint: None,
                local: None,
                layers: v.layers,
                shapes: vec![s],
                zone: None,
            });
        }
        for p in &b.pads {
            let copper = pad_has_copper(p);
            specs.push(ItemSpec {
                kind: ItemKind::Pad,
                net: p.net,
                footprint: Some(p.footprint),
                local: p.local_clearance,
                layers: if copper {
                    p.layers
                } else {
                    crate::api::LayerMask::EMPTY
                },
                shapes: if copper {
                    p.shapes.iter().collect()
                } else {
                    Vec::new()
                },
                zone: None,
            });
        }
        let zones_checked = self.options.check_zone_fills;
        for z in &b.zones {
            specs.push(ItemSpec {
                kind: ItemKind::Zone,
                net: z.net,
                footprint: None,
                local: None,
                layers: crate::api::LayerMask::single(z.layer),
                shapes: Vec::new(),
                // Unchecked fills need no courtyard or area lookups.
                zone: (zones_checked && z.filled).then_some(z.id.0),
            });
        }

        let this = &*self;
        let subjects: Vec<(SubjectData<'b>, Vec<u32>)> =
            specs.par_iter().map(|s| this.subject_of(s)).collect();
        let mut interner = Interner::default();
        let ids: Vec<u32> = subjects
            .into_iter()
            .map(|(d, areas)| interner.intern(d, areas))
            .collect();
        self.subjects = interner.data;
        let (nt, na, nv, np) = (b.tracks.len(), b.arcs.len(), b.vias.len(), b.pads.len());
        self.item_subjects = ItemSubjects {
            tracks: ids[..nt].to_vec(),
            arcs: ids[nt..nt + na].to_vec(),
            vias: ids[nt + na..nt + na + nv].to_vec(),
            pads: ids[nt + na + nv..nt + na + nv + np].to_vec(),
            zones: ids[nt + na + nv + np..].to_vec(),
        };

        let n_layers = b.layer_count();
        let mut per_layer: Vec<Vec<Obj>> = vec![Vec::new(); n_layers];
        let mut holes = Vec::new();
        let mut k = 0usize;
        for (t, s) in b.tracks.iter().zip(&track_shapes) {
            per_layer[t.layer.index()].push(Obj {
                src: Src::Board(ItemRef::Track(t.id)),
                net: t.net,
                subject: ids[k],
                bbox: s.bbox(),
                geom: Geom::Shape(s.clone()),
                extra: 0,
            });
            k += 1;
        }
        for (a, ss) in b.arcs.iter().zip(&arc_shapes) {
            for s in ss {
                per_layer[a.layer.index()].push(Obj {
                    src: Src::Board(ItemRef::Arc(a.id)),
                    net: a.net,
                    subject: ids[k],
                    bbox: s.bbox(),
                    geom: Geom::Shape(s.clone()),
                    extra: arc_extra,
                });
            }
            k += 1;
        }
        for (v, s) in b.vias.iter().zip(&via_shapes) {
            for layer in v.layers.iter() {
                if let Some(objs) = per_layer.get_mut(layer.index()) {
                    objs.push(Obj {
                        src: Src::Board(ItemRef::Via(v.id)),
                        net: v.net,
                        subject: ids[k],
                        bbox: s.bbox(),
                        geom: Geom::Shape(s.clone()),
                        extra: 0,
                    });
                }
            }
            holes.push(Hole {
                src: Src::Board(ItemRef::Via(v.id)),
                net: v.net,
                plated: true,
                shape: v.hole(),
                bbox: v.hole().bbox(),
                layers: v.layers,
            });
            k += 1;
        }
        let all_layers = b.layers.all();
        for p in &b.pads {
            if pad_has_copper(p) {
                for layer in p.layers.iter() {
                    if let Some(objs) = per_layer.get_mut(layer.index()) {
                        for s in &p.shapes {
                            objs.push(Obj {
                                src: Src::Board(ItemRef::Pad(p.id)),
                                net: p.net,
                                subject: ids[k],
                                bbox: s.bbox(),
                                geom: Geom::Shape(s.clone()),
                                extra: 0,
                            });
                        }
                    }
                }
            }
            if let Some(d) = &p.drill {
                holes.push(Hole {
                    src: Src::Board(ItemRef::Pad(p.id)),
                    net: p.net,
                    plated: d.plated,
                    shape: d.shape.clone(),
                    bbox: d.shape.bbox(),
                    layers: all_layers,
                });
            }
            k += 1;
        }
        for z in &b.zones {
            if z.filled && self.options.check_zone_fills {
                per_layer[z.layer.index()].push(Obj {
                    src: Src::Board(ItemRef::Zone(z.id)),
                    net: z.net,
                    subject: ids[k],
                    bbox: self.zones[z.id.index()].bbox(),
                    geom: Geom::Zone(z.id.0),
                    extra: 0,
                });
            } else if !z.filled {
                self.unfilled_zones += 1;
            }
            k += 1;
        }
        self.layers = per_layer.into_par_iter().map(LayerObjs::finish).collect();
        self.holes = Holes::finish(holes);
    }

    /// Routed copper as per-layer objects and holes, with subjects interned
    /// after the board's.
    fn routed_objects(
        &self,
        copper: &RoutedCopper,
    ) -> (Vec<LayerObjs>, Vec<Hole>, Vec<SubjectData<'b>>, Vec<u32>) {
        let track_shapes: Vec<Shape> = copper
            .tracks
            .iter()
            .map(|t| Shape::Segment(t.seg))
            .collect();
        let via_shapes: Vec<Shape> = copper
            .vias
            .iter()
            .map(|v| Shape::Circle(Circle::from_diameter(v.center, v.diameter)))
            .collect();
        let mut specs = Vec::new();
        for (t, s) in copper.tracks.iter().zip(&track_shapes) {
            specs.push(ItemSpec {
                kind: ItemKind::Track,
                net: Some(t.net),
                footprint: None,
                local: None,
                layers: crate::api::LayerMask::single(t.layer),
                shapes: vec![s],
                zone: None,
            });
        }
        for (v, s) in copper.vias.iter().zip(&via_shapes) {
            specs.push(ItemSpec {
                kind: ItemKind::Via,
                net: Some(v.net),
                footprint: None,
                local: None,
                layers: v.layers,
                shapes: vec![s],
                zone: None,
            });
        }
        let subjects: Vec<(SubjectData<'b>, Vec<u32>)> =
            specs.par_iter().map(|s| self.subject_of(s)).collect();
        let mut interner = Interner::default();
        let base = self.subjects.len() as u32;
        let ids: Vec<u32> = subjects
            .into_iter()
            .map(|(d, areas)| base + interner.intern(d, areas))
            .collect();

        let mut per_layer: Vec<Vec<Obj>> = vec![Vec::new(); self.board.layer_count()];
        let mut holes = Vec::new();
        for (i, (t, s)) in copper.tracks.iter().zip(&track_shapes).enumerate() {
            if let Some(objs) = per_layer.get_mut(t.layer.index()) {
                objs.push(Obj {
                    src: Src::RoutedTrack(i as u32),
                    net: Some(t.net),
                    subject: ids[i],
                    bbox: s.bbox(),
                    geom: Geom::Shape(s.clone()),
                    extra: 0,
                });
            }
        }
        let nt = copper.tracks.len();
        for (i, (v, s)) in copper.vias.iter().zip(&via_shapes).enumerate() {
            for layer in v.layers.iter() {
                if let Some(objs) = per_layer.get_mut(layer.index()) {
                    objs.push(Obj {
                        src: Src::RoutedVia(i as u32),
                        net: Some(v.net),
                        subject: ids[nt + i],
                        bbox: s.bbox(),
                        geom: Geom::Shape(s.clone()),
                        extra: 0,
                    });
                }
            }
            let hole = Shape::Circle(Circle::from_diameter(v.center, v.drill));
            holes.push(Hole {
                src: Src::RoutedVia(i as u32),
                net: Some(v.net),
                plated: true,
                bbox: hole.bbox(),
                shape: hole,
                layers: v.layers,
            });
        }
        let layers = per_layer.into_par_iter().map(LayerObjs::finish).collect();
        (layers, holes, interner.data, ids)
    }

    /// The footprints whose courtyard a board item's copper touches, as its
    /// rule subject carries them (KiCad's `intersectsCourtyard()`), and the
    /// rule areas that enclose it.
    pub fn subject_context(&self, item: ItemRef) -> (&[FootprintId], &[&'b str]) {
        let s = &self.item_subjects;
        let id = match item {
            ItemRef::Track(i) => s.tracks[i.index()],
            ItemRef::Arc(i) => s.arcs[i.index()],
            ItemRef::Via(i) => s.vias[i.index()],
            ItemRef::Pad(i) => s.pads[i.index()],
            ItemRef::Zone(i) => s.zones[i.index()],
        };
        let d = &self.subjects[id as usize];
        (&d.courtyards, &d.areas)
    }

    // ─── Running ─────────────────────────────────────────────────────────

    /// Whole-board check of the existing copper.
    pub fn check_board(&self) -> CheckReport {
        self.run(None)
    }

    /// Whole-board check, timed into `telemetry` as [`crate::api::Stage::Check`].
    pub fn check_board_timed(&self, telemetry: &mut Telemetry) -> CheckReport {
        let start = Instant::now();
        let report = self.run(None);
        record(telemetry, start, &report);
        report
    }

    /// Check routed copper against the board and itself. Board-to-board
    /// pairs are not re-checked.
    pub fn check_routed(&self, copper: &RoutedCopper) -> CheckReport {
        self.run(Some(copper))
    }

    /// [`Self::check_routed`], timed into `telemetry`.
    pub fn check_routed_timed(
        &self,
        copper: &RoutedCopper,
        telemetry: &mut Telemetry,
    ) -> CheckReport {
        let start = Instant::now();
        let report = self.run(Some(copper));
        record(telemetry, start, &report);
        report
    }

    fn run(&self, copper: Option<&RoutedCopper>) -> CheckReport {
        let empty = RoutedCopper::default();
        let routed = copper.unwrap_or(&empty);
        let (rlayers, rholes, rsubjects, rids) = self.routed_objects(routed);
        let ctx = Ctx {
            c: self,
            routed: copper.is_some(),
            rlayers: &rlayers,
            rholes: Holes::finish(rholes),
            rsubjects: &rsubjects,
            rids: &rids,
            n_routed_tracks: routed.tracks.len(),
        };
        let mut stats = CheckStats {
            unfilled_zones_skipped: self.unfilled_zones,
            ..CheckStats::default()
        };
        let mut found = Vec::new();

        let (v, s) = ctx.copper_pairs();
        found.extend(v);
        stats.candidate_pairs += s.0;
        stats.exact_tests += s.1;
        found.extend(ctx.hole_clearance());
        found.extend(ctx.hole_to_hole());
        found.extend(ctx.edge_clearance());
        found.extend(ctx.keepouts());
        found.extend(ctx.single_items(routed));
        found.extend(ctx.track_angles(routed));

        stats.objects = self
            .layers
            .iter()
            .chain(rlayers.iter())
            .map(|l| l.objs.len() as u64)
            .sum();
        stats.holes = (self.holes.holes.len() + ctx.rholes.holes.len()) as u64;

        found.sort_by_key(|v| v.sort_key());
        // One finding per (kind, a, b): sorted, so the lowest layer and
        // location come first and duplicates are adjacent.
        let mut violations = found;
        violations.dedup_by(|later, first| later.same_finding(first));
        CheckReport { violations, stats }
    }
}

fn record(telemetry: &mut Telemetry, start: Instant, report: &CheckReport) {
    telemetry.add_stage_time(crate::api::Stage::Check, start.elapsed().as_micros() as u64);
    telemetry.threads = telemetry.threads.max(rayon::current_num_threads() as u32);
    telemetry.count("check_objects", report.stats.objects);
    telemetry.count("check_candidate_pairs", report.stats.candidate_pairs);
    telemetry.count("check_exact_tests", report.stats.exact_tests);
    telemetry.count("check_violations", report.violations.len() as u64);
}

/// Whether a pad has copper on its layers. A non-plated hole whose pad is
/// no larger than the hole carries none (KiCad's `PAD::FlashLayer`): only its
/// hole is checked.
fn pad_has_copper(p: &Pad) -> bool {
    if p.kind != PadKind::NpThroughHole {
        return !p.shapes.is_empty();
    }
    let Some(drill) = &p.drill else {
        return !p.shapes.is_empty();
    };
    !p.shapes.iter().all(|s| covered_by_hole(s, &drill.shape))
}

fn covered_by_hole(s: &Shape, hole: &Shape) -> bool {
    match (s, hole) {
        (Shape::Circle(c), Shape::Circle(h)) => c.center == h.center && c.radius <= h.radius,
        (Shape::Segment(c), Shape::Segment(h)) => {
            ((c.a == h.a && c.b == h.b) || (c.a == h.b && c.b == h.a)) && c.width <= h.width
        }
        _ => false,
    }
}

/// One check run: the board side plus this call's routed copper.
struct Ctx<'c, 'b> {
    c: &'c BoardChecker<'b>,
    routed: bool,
    rlayers: &'c [LayerObjs],
    rholes: Holes,
    rsubjects: &'c [SubjectData<'b>],
    /// Routed tracks' subjects, then routed vias'.
    rids: &'c [u32],
    n_routed_tracks: usize,
}

/// Per-worker memo of resolved clearances.
type ClearanceCache = HashMap<(u32, u32, u8), Coord>;

impl<'c, 'b> Ctx<'c, 'b> {
    fn subject(&self, id: u32) -> &SubjectData<'b> {
        let base = self.c.subjects.len();
        let i = id as usize;
        if i < base {
            &self.c.subjects[i]
        } else {
            &self.rsubjects[i - base]
        }
    }

    /// The interned subject of any item.
    fn src_subject(&self, src: Src) -> &SubjectData<'b> {
        let s = &self.c.item_subjects;
        let id = match src {
            Src::Board(ItemRef::Track(i)) => s.tracks[i.index()],
            Src::Board(ItemRef::Arc(i)) => s.arcs[i.index()],
            Src::Board(ItemRef::Via(i)) => s.vias[i.index()],
            Src::Board(ItemRef::Pad(i)) => s.pads[i.index()],
            Src::Board(ItemRef::Zone(i)) => s.zones[i.index()],
            Src::RoutedTrack(i) => self.rids[i as usize],
            Src::RoutedVia(i) => self.rids[self.n_routed_tracks + i as usize],
        };
        self.subject(id)
    }

    fn clearance(&self, cache: &mut ClearanceCache, a: u32, b: u32, layer: LayerId) -> Coord {
        let key = (a.min(b), a.max(b), layer.0);
        if let Some(&v) = cache.get(&key) {
            return v;
        }
        let v = self
            .c
            .rules
            .clearance(&self.subject(a).view(), &self.subject(b).view(), layer);
        cache.insert(key, v);
        v
    }

    fn geom_gap(&self, a: &Geom, b: &Geom, clearance: i64) -> Ordering {
        match (a, b) {
            (Geom::Shape(x), Geom::Shape(y)) => gap_cmp(x, y, clearance),
            (Geom::Zone(z), Geom::Shape(s)) | (Geom::Shape(s), Geom::Zone(z)) => {
                self.c.zones[*z as usize].gap_cmp(s, clearance)
            }
            (Geom::Zone(x), Geom::Zone(y)) => {
                self.c.zones[*x as usize].gap_cmp_polygon(&self.c.zones[*y as usize], clearance)
            }
        }
    }

    /// Where two geometries come closest (only called for violations).
    fn locate(&self, a: &Geom, b: &Geom) -> Point {
        let near = |z: u32, s: &Shape| {
            let zone = &self.c.zones[z as usize];
            zone.closest_within(s, self.c.reach.max(1))
                .map_or_else(|| s.bbox().center(), |c| c.midpoint())
        };
        match (a, b) {
            (Geom::Shape(x), Geom::Shape(y)) => closest(x, y).midpoint(),
            (Geom::Zone(z), Geom::Shape(s)) | (Geom::Shape(s), Geom::Zone(z)) => near(*z, s),
            (Geom::Zone(x), Geom::Zone(y)) => {
                let (bx, by) = (
                    self.c.zones[*x as usize].bbox(),
                    self.c.zones[*y as usize].bbox(),
                );
                let overlap = BBox::new(
                    Point::new(bx.min.x.max(by.min.x), bx.min.y.max(by.min.y)),
                    Point::new(bx.max.x.min(by.max.x), bx.max.y.min(by.max.y)),
                );
                overlap.center()
            }
        }
    }

    /// Orient a pair: a routed item first, else the lower-ordered item.
    fn orient(a: Src, b: Src) -> (Src, Src) {
        match (a.is_routed(), b.is_routed()) {
            (false, true) => (b, a),
            (true, false) => (a, b),
            _ if b < a => (b, a),
            _ => (a, b),
        }
    }

    // ─── Copper to copper ───────────────────────────────────────────────

    fn test_pair(
        &self,
        cache: &mut ClearanceCache,
        layer: LayerId,
        a: &Obj,
        b: &Obj,
        counts: &mut (u64, u64),
    ) -> Option<Violation> {
        if a.src == b.src || (a.net.is_some() && a.net == b.net) {
            return None;
        }
        counts.0 += 1;
        let extra = i64::from(a.extra) + i64::from(b.extra) + self.c.margin();
        let reach = self.c.reach + i64::from(a.extra) + i64::from(b.extra);
        if self.geom_gap(&a.geom, &b.geom, reach) != Ordering::Less {
            return None;
        }
        counts.1 += 1;
        let required = self.clearance(cache, a.subject, b.subject, layer);
        if self.geom_gap(&a.geom, &b.geom, i64::from(required) + extra) != Ordering::Less {
            return None;
        }
        let short = a.net.is_some()
            && b.net.is_some()
            && self.geom_gap(&a.geom, &b.geom, 0) != Ordering::Greater;
        let (sa, sb) = Self::orient(a.src, b.src);
        Some(Violation {
            kind: if short {
                ViolationKind::Short
            } else {
                ViolationKind::Clearance
            },
            a: sa.checked(),
            b: Some(sb.checked()),
            layer: Some(layer),
            at: self.locate(&a.geom, &b.geom),
            required,
        })
    }

    /// Every same-layer copper pair that involves at least one checked
    /// object (all of them for a board check, routed ones otherwise).
    fn copper_pairs(&self) -> (Vec<Violation>, (u64, u64)) {
        let mut jobs: Vec<(usize, bool, usize)> = Vec::new();
        for l in 0..self.c.layers.len() {
            if !self.routed {
                jobs.extend((0..self.c.layers[l].objs.len()).map(|i| (l, false, i)));
            }
            if let Some(r) = self.rlayers.get(l) {
                jobs.extend((0..r.objs.len()).map(|i| (l, true, i)));
            }
        }
        let result = jobs
            .par_iter()
            .with_min_len(64)
            .fold(
                || (Vec::new(), (0u64, 0u64), ClearanceCache::new()),
                |(mut out, mut counts, mut cache), &(l, routed, i)| {
                    let layer = LayerId(l as u8);
                    let board = &self.c.layers[l];
                    let (own, obj) = if routed {
                        (&self.rlayers[l], &self.rlayers[l].objs[i])
                    } else {
                        (board, &board.objs[i])
                    };
                    let q = obj
                        .bbox
                        .inflate(clamp(self.c.reach + 2 * i64::from(self.c.arc_extra())));
                    // Pairs inside the object's own set: each once (j > i).
                    for e in own.index.query_entries(&q) {
                        let j = e.id as usize;
                        if j <= i {
                            continue;
                        }
                        if let Some(v) =
                            self.test_pair(&mut cache, layer, obj, &own.objs[j], &mut counts)
                        {
                            out.push(v);
                        }
                    }
                    // Routed objects also meet every board object.
                    if routed {
                        for e in board.index.query_entries(&q) {
                            let other = &board.objs[e.id as usize];
                            if let Some(v) =
                                self.test_pair(&mut cache, layer, obj, other, &mut counts)
                            {
                                out.push(v);
                            }
                        }
                    }
                    (out, counts, cache)
                },
            )
            .map(|(out, counts, _)| (out, counts))
            .reduce(
                || (Vec::new(), (0, 0)),
                |(mut a, ca), (b, cb)| {
                    a.extend(b);
                    (a, (ca.0 + cb.0, ca.1 + cb.1))
                },
            );
        result
    }

    // ─── Holes ──────────────────────────────────────────────────────────

    fn hole_test(&self, layer: LayerId, obj: &Obj, hole: &Hole) -> Option<Violation> {
        if obj.src == hole.src
            || !hole.layers.contains(layer)
            || (hole.plated && obj.net.is_some() && obj.net == hole.net)
        {
            return None;
        }
        let required = self.c.rules.hole_clearance();
        let need = i64::from(required) + self.c.margin() + i64::from(obj.extra);
        let hg = Geom::Shape(hole.shape.clone());
        if self.geom_gap(&obj.geom, &hg, need) != Ordering::Less {
            return None;
        }
        let (a, b) = Self::orient(obj.src, hole.src);
        // A hole finding is about the item pair; keep the layer only when
        // the copper is on one layer.
        let multi = matches!(
            obj.src,
            Src::Board(ItemRef::Via(_) | ItemRef::Pad(_)) | Src::RoutedVia(_)
        );
        Some(Violation {
            kind: ViolationKind::HoleClearance,
            a: a.checked(),
            b: Some(b.checked()),
            layer: (!multi).then_some(layer),
            at: self.locate(&obj.geom, &hg),
            required,
        })
    }

    fn hole_clearance(&self) -> Vec<Violation> {
        let reach = clamp(
            i64::from(self.c.rules.hole_clearance())
                + self.c.margin()
                + i64::from(self.c.arc_extra()),
        );
        let mut jobs: Vec<(usize, bool, usize)> = Vec::new();
        for l in 0..self.c.layers.len() {
            if !self.routed {
                jobs.extend((0..self.c.layers[l].objs.len()).map(|i| (l, false, i)));
            }
            if let Some(r) = self.rlayers.get(l) {
                jobs.extend((0..r.objs.len()).map(|i| (l, true, i)));
            }
        }
        let mut out: Vec<Violation> = jobs
            .par_iter()
            .with_min_len(64)
            .flat_map_iter(|&(l, routed, i)| {
                let layer = LayerId(l as u8);
                let obj = if routed {
                    &self.rlayers[l].objs[i]
                } else {
                    &self.c.layers[l].objs[i]
                };
                let q = obj.bbox.inflate(reach);
                let mut v = Vec::new();
                // Board copper meets board holes only in a board check;
                // routed copper meets every hole.
                if !self.routed || routed {
                    for e in self.c.holes.index.query_entries(&q) {
                        v.extend(self.hole_test(layer, obj, &self.c.holes.holes[e.id as usize]));
                    }
                }
                for e in self.rholes.index.query_entries(&q) {
                    v.extend(self.hole_test(layer, obj, &self.rholes.holes[e.id as usize]));
                }
                v
            })
            .collect();
        // Board copper against routed holes.
        if self.routed {
            out.extend(
                self.rholes
                    .holes
                    .par_iter()
                    .flat_map_iter(|h| {
                        let q = h.bbox.inflate(reach);
                        let mut v = Vec::new();
                        for layer in h.layers.iter() {
                            if let Some(lo) = self.c.layers.get(layer.index()) {
                                for e in lo.index.query_entries(&q) {
                                    v.extend(self.hole_test(layer, &lo.objs[e.id as usize], h));
                                }
                            }
                        }
                        v
                    })
                    .collect::<Vec<_>>(),
            );
        }
        out
    }

    fn hole_pair(&self, a: &Hole, b: &Hole) -> Option<Violation> {
        if a.src == b.src || !a.layers.intersects(b.layers) {
            return None;
        }
        let required = self.c.rules.hole_to_hole();
        let need = i64::from(required) + self.c.margin();
        if gap_cmp(&a.shape, &b.shape, need) != Ordering::Less {
            return None;
        }
        let (sa, sb) = Self::orient(a.src, b.src);
        Some(Violation {
            kind: ViolationKind::HoleToHole,
            a: sa.checked(),
            b: Some(sb.checked()),
            layer: None,
            at: closest(&a.shape, &b.shape).midpoint(),
            required,
        })
    }

    fn hole_to_hole(&self) -> Vec<Violation> {
        let reach = clamp(i64::from(self.c.rules.hole_to_hole()) + self.c.margin());
        let mut jobs: Vec<(bool, usize)> = Vec::new();
        if !self.routed {
            jobs.extend((0..self.c.holes.holes.len()).map(|i| (false, i)));
        }
        jobs.extend((0..self.rholes.holes.len()).map(|i| (true, i)));
        jobs.par_iter()
            .with_min_len(64)
            .flat_map_iter(|&(routed, i)| {
                let set = if routed { &self.rholes } else { &self.c.holes };
                let h = &set.holes[i];
                let q = h.bbox.inflate(reach);
                let mut v = Vec::new();
                for e in set.index.query_entries(&q) {
                    if (e.id as usize) > i {
                        v.extend(self.hole_pair(h, &set.holes[e.id as usize]));
                    }
                }
                if routed {
                    for e in self.c.holes.index.query_entries(&q) {
                        v.extend(self.hole_pair(h, &self.c.holes.holes[e.id as usize]));
                    }
                }
                v
            })
            .collect()
    }

    // ─── Edges and keepouts ─────────────────────────────────────────────

    fn edge_clearance(&self) -> Vec<Violation> {
        if self.c.edges.is_empty() {
            return Vec::new();
        }
        let required = self.c.rules.edge_clearance();
        let mut jobs: Vec<(usize, bool, usize)> = Vec::new();
        for l in 0..self.c.layers.len() {
            if !self.routed {
                jobs.extend((0..self.c.layers[l].objs.len()).map(|i| (l, false, i)));
            }
            if let Some(r) = self.rlayers.get(l) {
                jobs.extend((0..r.objs.len()).map(|i| (l, true, i)));
            }
        }
        jobs.par_iter()
            .with_min_len(64)
            .flat_map_iter(|&(l, routed, i)| {
                let obj = if routed {
                    &self.rlayers[l].objs[i]
                } else {
                    &self.c.layers[l].objs[i]
                };
                let need = i64::from(required) + self.c.margin() + i64::from(obj.extra);
                let q = obj.bbox.inflate(clamp(need));
                let mut v = Vec::new();
                for e in self.c.edge_index.query_entries(&q) {
                    let edge = Geom::Shape(self.c.edges[e.id as usize].clone());
                    if self.geom_gap(&obj.geom, &edge, need) == Ordering::Less {
                        let multi = matches!(
                            obj.src,
                            Src::Board(ItemRef::Via(_) | ItemRef::Pad(_)) | Src::RoutedVia(_)
                        );
                        v.push(Violation {
                            kind: ViolationKind::EdgeClearance,
                            a: obj.src.checked(),
                            b: Some(CheckedItem::Edge),
                            layer: (!multi).then_some(LayerId(l as u8)),
                            at: self.locate(&obj.geom, &edge),
                            required,
                        });
                        break;
                    }
                }
                v
            })
            .collect()
    }

    fn keepouts(&self) -> Vec<Violation> {
        let mut out = Vec::new();
        for k in &self.c.board.keepouts {
            let r = k.rules;
            if !(r.tracks || r.vias || r.pads || r.copperpour) {
                continue;
            }
            let region = Geom::Shape(Shape::Polygon(k.region.clone()));
            let bbox = k.region.bbox();
            for layer in k.layers.iter() {
                let mut sets: Vec<&LayerObjs> = Vec::new();
                if !self.routed {
                    sets.extend(self.c.layers.get(layer.index()));
                }
                sets.extend(self.rlayers.get(layer.index()));
                for set in sets {
                    let found: Vec<Violation> = set
                        .index
                        .query(&bbox)
                        .par_iter()
                        .filter_map(|&id| {
                            let obj = &set.objs[id as usize];
                            let forbidden = match obj.src.kind() {
                                ItemKind::Track | ItemKind::Arc => r.tracks,
                                ItemKind::Via => r.vias,
                                ItemKind::Pad => r.pads,
                                ItemKind::Zone => r.copperpour,
                            };
                            (forbidden && self.geom_gap(&obj.geom, &region, 0) != Ordering::Greater)
                                .then(|| Violation {
                                    kind: ViolationKind::Keepout,
                                    a: obj.src.checked(),
                                    b: None,
                                    layer: Some(layer),
                                    at: self.locate(&obj.geom, &region),
                                    required: 0,
                                })
                        })
                        .collect();
                    out.extend(found);
                }
            }
        }
        out
    }

    // ─── Single items ───────────────────────────────────────────────────

    /// Track width, segment length and via size.
    fn single_items(&self, routed: &RoutedCopper) -> Vec<Violation> {
        let board = self.c.board;
        let rules = self.c.rules;
        let mut items: Vec<SingleItem> = Vec::new();
        if !self.routed {
            items.extend(board.tracks.iter().map(|t| SingleItem::Track {
                src: Src::Board(ItemRef::Track(t.id)),
                layer: t.layer,
                seg: t.seg,
            }));
            items.extend(board.arcs.iter().map(|a| SingleItem::Arc {
                src: Src::Board(ItemRef::Arc(a.id)),
                layer: a.layer,
                arc: [a.start, a.mid, a.end],
                width: a.width,
            }));
            items.extend(board.vias.iter().map(|v| SingleItem::Via {
                src: Src::Board(ItemRef::Via(v.id)),
                center: v.center,
                diameter: v.diameter,
                drill: v.drill,
            }));
        }
        items.extend(
            routed
                .tracks
                .iter()
                .enumerate()
                .map(|(i, t)| SingleItem::Track {
                    src: Src::RoutedTrack(i as u32),
                    layer: t.layer,
                    seg: t.seg,
                }),
        );
        items.extend(
            routed
                .vias
                .iter()
                .enumerate()
                .map(|(i, v)| SingleItem::Via {
                    src: Src::RoutedVia(i as u32),
                    center: v.center,
                    diameter: v.diameter,
                    drill: v.drill,
                }),
        );

        items
            .par_iter()
            .with_min_len(256)
            .flat_map_iter(|item| {
                let mut v = Vec::new();
                let s = self.src_subject(item.src()).view();
                match *item {
                    SingleItem::Track {
                        src, layer, seg, ..
                    } => {
                        let mid = BBox::new(seg.a, seg.b).center();
                        let w = rules.track_width(&s, layer);
                        if seg.width < w.min {
                            v.push(single(ViolationKind::TrackWidth, src, layer, mid, w.min));
                        }
                        if let Some(min) = rules.min_segment_length(&s, layer) {
                            let l2 = (seg.b - seg.a).length2();
                            if l2 < i128::from(min) * i128::from(min) {
                                v.push(single(ViolationKind::SegmentLength, src, layer, mid, min));
                            }
                        }
                    }
                    SingleItem::Arc {
                        src,
                        layer,
                        arc,
                        width,
                        ..
                    } => {
                        let w = rules.track_width(&s, layer);
                        if width < w.min {
                            v.push(single(ViolationKind::TrackWidth, src, layer, arc[1], w.min));
                        }
                        if let Some(min) = rules.min_segment_length(&s, layer) {
                            if arc_length(arc) < f64::from(min) {
                                v.push(single(
                                    ViolationKind::SegmentLength,
                                    src,
                                    layer,
                                    arc[1],
                                    min,
                                ));
                            }
                        }
                    }
                    SingleItem::Via {
                        src,
                        center,
                        diameter,
                        drill,
                        ..
                    } => {
                        let r = rules.via(&s);
                        let annulus_short =
                            i64::from(diameter) - i64::from(drill) < 2 * i64::from(r.min_annular);
                        if diameter < r.min_diameter || drill < r.min_drill || annulus_short {
                            v.push(Violation {
                                kind: ViolationKind::ViaSize,
                                a: src.checked(),
                                b: None,
                                layer: None,
                                at: center,
                                required: if drill < r.min_drill {
                                    r.min_drill
                                } else {
                                    r.min_diameter
                                },
                            });
                        }
                    }
                }
                v
            })
            .collect()
    }

    // ─── Track angles ───────────────────────────────────────────────────

    /// Every pair of track ends of one net meeting exactly at one point on
    /// one layer, unless pad copper covers the point: the bend's inside
    /// angle against the stricter of the two tracks' rules. This is what
    /// KiCad flags on the oracle board: three-way junctions pair by pair,
    /// junctions on vias too, junctions on pads never.
    fn track_angles(&self, routed: &RoutedCopper) -> Vec<Violation> {
        let board = self.c.board;
        let mut ends: Vec<End> = Vec::new();
        let mut push_seg = |src: Src, net: Option<NetId>, layer: LayerId, seg: Segment| {
            if seg.a == seg.b {
                return;
            }
            let (a, b) = (seg.a, seg.b);
            ends.push(End::new(src, net, layer, a, dir(a, b)));
            ends.push(End::new(src, net, layer, b, dir(b, a)));
        };
        if !self.routed || !routed.tracks.is_empty() {
            for t in &board.tracks {
                push_seg(Src::Board(ItemRef::Track(t.id)), t.net, t.layer, t.seg);
            }
        }
        for (i, t) in routed.tracks.iter().enumerate() {
            push_seg(Src::RoutedTrack(i as u32), Some(t.net), t.layer, t.seg);
        }
        if !self.routed || !routed.tracks.is_empty() {
            for a in &board.arcs {
                let src = Src::Board(ItemRef::Arc(a.id));
                if let Some((ts, te)) = arc_tangents(a.start, a.mid, a.end) {
                    ends.push(End::new(src, a.net, a.layer, a.start, ts));
                    ends.push(End::new(src, a.net, a.layer, a.end, te));
                }
            }
        }
        ends.par_sort_unstable_by_key(|e| (e.layer, e.at, e.src));

        let mut groups: Vec<(usize, usize)> = Vec::new();
        let mut i = 0;
        while i < ends.len() {
            let mut j = i + 1;
            while j < ends.len() && ends[j].layer == ends[i].layer && ends[j].at == ends[i].at {
                j += 1;
            }
            if j - i >= 2 {
                groups.push((i, j));
            }
            i = j;
        }
        groups
            .par_iter()
            .flat_map_iter(|&(i, j)| {
                let group = &ends[i..j];
                let mut out = Vec::new();
                // A junction on a pad is the pad's connection, not a bend
                // (KiCad skips it); one on a via still bends.
                if self.on_pad(group[0].layer, group[0].at) {
                    return out;
                }
                for (k, e0) in group.iter().enumerate() {
                    for e1 in &group[k + 1..] {
                        if e0.net != e1.net || e0.src == e1.src {
                            continue;
                        }
                        if self.routed && !e0.src.is_routed() && !e1.src.is_routed() {
                            continue;
                        }
                        let rule = |src: Src| {
                            self.c
                                .rules
                                .min_track_angle_deg(&self.src_subject(src).view(), e0.layer)
                        };
                        // Each track's own rule applies to its bend: the
                        // stricter of the two decides.
                        let min = match (rule(e0.src), rule(e1.src)) {
                            (Some(a), Some(b)) => a.max(b),
                            (Some(a), None) | (None, Some(a)) => a,
                            (None, None) => continue,
                        };
                        // Two tracks leaving the junction the same way
                        // overlap: a redundant run, not a bend, and KiCad
                        // does not report it.
                        if overlapping(e0.dir, e1.dir) {
                            continue;
                        }
                        let angle = inside_angle_deg(e0.dir, e1.dir);
                        // A bend at exactly the minimum is legal, as in KiCad;
                        // the 1e-9° allowance only absorbs `atan2` rounding.
                        if angle < min - 1e-9 {
                            let (sa, sb) = Self::orient(e0.src, e1.src);
                            out.push(Violation {
                                kind: ViolationKind::TrackAngle,
                                a: sa.checked(),
                                b: Some(sb.checked()),
                                layer: Some(e0.layer),
                                at: e0.at,
                                required: (min * 10.0).round() as Coord,
                            });
                        }
                    }
                }
                out
            })
            .collect()
    }

    /// Whether pad copper on `layer` covers `at`.
    fn on_pad(&self, layer: LayerId, at: Point) -> bool {
        let q = BBox::point(at);
        self.c
            .layers
            .get(layer.index())
            .into_iter()
            .chain(self.rlayers.get(layer.index()))
            .any(|set| {
                set.index.query_entries(&q).any(|e| {
                    let o = &set.objs[e.id as usize];
                    o.src.kind() == ItemKind::Pad
                        && match &o.geom {
                            Geom::Shape(s) => {
                                konnect_geom::point_shape_distance_cmp(at, s, 0)
                                    != Ordering::Greater
                            }
                            Geom::Zone(_) => false,
                        }
                })
            })
    }
}

#[derive(Debug, Clone, Copy)]
enum SingleItem {
    Track {
        src: Src,
        layer: LayerId,
        seg: Segment,
    },
    Arc {
        src: Src,
        layer: LayerId,
        arc: [Point; 3],
        width: Coord,
    },
    Via {
        src: Src,
        center: Point,
        diameter: Coord,
        drill: Coord,
    },
}

impl SingleItem {
    fn src(&self) -> Src {
        match *self {
            SingleItem::Track { src, .. }
            | SingleItem::Arc { src, .. }
            | SingleItem::Via { src, .. } => src,
        }
    }
}

fn single(kind: ViolationKind, src: Src, layer: LayerId, at: Point, required: Coord) -> Violation {
    Violation {
        kind,
        a: src.checked(),
        b: None,
        layer: Some(layer),
        at,
        required,
    }
}

/// One track end: where it is and which way the track leaves it.
#[derive(Debug, Clone, Copy)]
struct End {
    src: Src,
    net: Option<NetId>,
    layer: LayerId,
    at: Point,
    dir: (f64, f64),
}

impl End {
    fn new(src: Src, net: Option<NetId>, layer: LayerId, at: Point, dir: (f64, f64)) -> Self {
        End {
            src,
            net,
            layer,
            at,
            dir,
        }
    }
}

fn dir(from: Point, to: Point) -> (f64, f64) {
    let v = to - from;
    (v.x as f64, v.y as f64)
}

/// Whether two directions leaving one point coincide. Segment directions
/// are exact integer differences (well inside `f64`'s 53 bits), so the test
/// is exact for them.
fn overlapping(u: (f64, f64), v: (f64, f64)) -> bool {
    u.0 * v.1 - u.1 * v.0 == 0.0 && u.0 * v.0 + u.1 * v.1 > 0.0
}

/// The angle between two directions leaving one point, degrees in
/// `[0, 180]`: 180 is straight on, 90 a right-angle bend. Right angles and
/// straight runs of integer directions come out exact.
fn inside_angle_deg(u: (f64, f64), v: (f64, f64)) -> f64 {
    let dot = u.0 * v.0 + u.1 * v.1;
    let cross = u.0 * v.1 - u.1 * v.0;
    if dot == 0.0 {
        return 90.0;
    }
    if cross == 0.0 {
        return if dot > 0.0 { 0.0 } else { 180.0 };
    }
    cross.abs().atan2(dot).to_degrees()
}

/// Directions leaving an arc's start and end along the arc (tangents), or
/// `None` for a degenerate arc.
fn arc_tangents(s: Point, m: Point, e: Point) -> Option<((f64, f64), (f64, f64))> {
    let (sx, sy) = (f64::from(s.x), f64::from(s.y));
    let (mx, my) = (f64::from(m.x) - sx, f64::from(m.y) - sy);
    let (ex, ey) = (f64::from(e.x) - sx, f64::from(e.y) - sy);
    let d = 2.0 * (mx * ey - my * ex);
    if d == 0.0 {
        let v = dir(s, e);
        return Some((v, (-v.0, -v.1)));
    }
    let (qm, qe) = (mx * mx + my * my, ex * ex + ey * ey);
    let cx = (ey * qm - my * qe) / d;
    let cy = (mx * qe - ex * qm) / d;
    // Tangent at a point P: perpendicular to P - C, pointed toward `toward`.
    let tangent = |px: f64, py: f64, tx: f64, ty: f64| {
        let (rx, ry) = (px - cx, py - cy);
        let t = (-ry, rx);
        if t.0 * (tx - px) + t.1 * (ty - py) >= 0.0 {
            t
        } else {
            (-t.0, -t.1)
        }
    };
    Some((tangent(0.0, 0.0, mx, my), tangent(ex, ey, mx, my)))
}

/// Length of a three-point arc, nm.
fn arc_length(p: [Point; 3]) -> f64 {
    let f = |q: Point| (f64::from(q.x), f64::from(q.y));
    let (s, m, e) = (f(p[0]), f(p[1]), f(p[2]));
    let (mx, my) = (m.0 - s.0, m.1 - s.1);
    let (ex, ey) = (e.0 - s.0, e.1 - s.1);
    let d = 2.0 * (mx * ey - my * ex);
    let chord = |a: (f64, f64), b: (f64, f64)| (a.0 - b.0).hypot(a.1 - b.1);
    if d == 0.0 {
        return chord(s, e);
    }
    let (qm, qe) = (mx * mx + my * my, ex * ex + ey * ey);
    let cx = (ey * qm - my * qe) / d;
    let cy = (mx * qe - ex * qm) / d;
    let r = cx.hypot(cy);
    let ang = |x: f64, y: f64| (y - cy).atan2(x - cx);
    let tau = std::f64::consts::TAU;
    let (a_s, a_m, a_e) = (ang(0.0, 0.0), ang(mx, my), ang(ex, ey));
    let ccw_e = (a_e - a_s).rem_euclid(tau);
    let ccw_m = (a_m - a_s).rem_euclid(tau);
    let sweep = if ccw_m <= ccw_e { ccw_e } else { tau - ccw_e };
    r * sweep
}

fn clamp(v: i64) -> Coord {
    v.clamp(0, i64::from(Coord::MAX)) as Coord
}

#[cfg(test)]
mod tests;
