//! Design rules: the [`RuleTable`] trait every stage asks, and
//! [`ProjectRuleTable`], built at runtime from a project's `.kicad_pro` and
//! `.kicad_dru` text.
//!
//! # What the week-1 table supports
//!
//! Only the rule *kinds* are fixed in code; every value comes from the
//! project files.
//!
//! | Kind | Source |
//! |---|---|
//! | net class clearance, track width, via size, diff-pair width/gap | `.kicad_pro` `net_settings.classes` + `netclass_patterns`/`netclass_assignments` |
//! | board minimums: clearance, track width, via diameter/annulus/drill | `.kicad_pro` `board.design_settings.rules` |
//! | edge clearance, hole clearance, hole-to-hole | the same `rules` block |
//! | custom clearance overrides by net class, net name, type, reference, courtyard, area | `.kicad_dru` `clearance` constraints |
//! | via size: floor, preferred, ceiling | `.kicad_dru` `via_diameter` / `hole_size` (`min`/`opt`/`max`), `annular_width` (`min`) |
//! | track width: floor, preferred, ceiling | `.kicad_dru` `track_width` (`min`/`opt`/`max`) |
//! | minimum segment length | `.kicad_dru` `track_segment_length` (`min`) |
//! | track angle minimum | `.kicad_dru` `track_angle` (`min`) |
//!
//! Anything else — another constraint type, a limit the table does not
//! apply (a `max` on a clearance, say), an unsupported condition, a
//! condition naming an area or footprint the board does not have, a length
//! too large for a [`Coord`], a rule with `(severity ignore)`, a via
//! constraint scoped to one layer — is reported in [`RulesReport::inert`]
//! and never half-applied.
//!
//! # Precedence (a hypothesis until the oracle confirms it)
//!
//! Clearance follows KiCad's `DRC_ENGINE::EvalRules`: a local override on
//! either item (the pad's `(clearance …)`, or its footprint's) beats every
//! rule, the larger of the two winning. Without one, the last matching
//! custom rule wins (KiCad processes rules bottom-up, first match wins);
//! two-item conditions are tried as (A, B) and (B, A). Otherwise the net
//! class, the larger of the two items' values. Other constraints: the last
//! matching custom rule, then the net class. Board minimums are a floor
//! under every result, and a floor beats a conflicting `max`.

mod condition;
mod netclass;

use crate::api::{ClassId, FootprintId, ItemKind, LayerId, NetId};
use crate::board::RouteBoard;
use condition::{parse_condition, Expr, NameResolver, Prop, SubjectView};
use konnect_geom::{mm_to_nm, Coord};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

pub(crate) use condition::wildcard_match_chars;

/// One item as rules see it.
#[derive(Debug, Clone, Copy)]
pub struct RuleSubject<'a> {
    pub kind: ItemKind,
    pub net: Option<NetId>,
    /// The footprint owning the item (pads).
    pub footprint: Option<FootprintId>,
    /// Footprints whose courtyard the item intersects.
    pub courtyards: &'a [FootprintId],
    /// Names of rule areas enclosing the item.
    pub areas: &'a [&'a str],
    /// A pad's own clearance override.
    pub local_clearance: Option<Coord>,
}

impl<'a> RuleSubject<'a> {
    pub fn new(kind: ItemKind, net: Option<NetId>) -> Self {
        RuleSubject {
            kind,
            net,
            footprint: None,
            courtyards: &[],
            areas: &[],
            local_clearance: None,
        }
    }

    pub fn with_footprint(mut self, footprint: FootprintId) -> Self {
        self.footprint = Some(footprint);
        self
    }

    pub fn with_courtyards(mut self, courtyards: &'a [FootprintId]) -> Self {
        self.courtyards = courtyards;
        self
    }

    pub fn with_areas(mut self, areas: &'a [&'a str]) -> Self {
        self.areas = areas;
        self
    }

    pub fn with_local_clearance(mut self, clearance: Option<Coord>) -> Self {
        self.local_clearance = clearance;
        self
    }
}

/// One net class's values, in nm.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct NetClassRules {
    pub name: String,
    pub clearance: Coord,
    pub track_width: Coord,
    pub via_diameter: Coord,
    pub via_drill: Coord,
    pub diff_pair_width: Option<Coord>,
    pub diff_pair_gap: Option<Coord>,
}

/// Board-wide minimums and hole/edge rules, in nm.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct BoardRules {
    pub min_clearance: Coord,
    pub min_track_width: Coord,
    pub min_via_diameter: Coord,
    pub min_via_annular_width: Coord,
    pub min_through_hole_diameter: Coord,
    pub edge_clearance: Coord,
    pub hole_clearance: Coord,
    pub hole_to_hole: Coord,
    /// KiCad's arc/fill approximation error (`max_error`).
    pub max_error: Coord,
}

/// The width a track must have.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct WidthRule {
    pub min: Coord,
    /// What the router should draw: the class width, never below `min`.
    pub preferred: Coord,
}

/// The via a net should use, and the floor any via must meet.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct ViaRule {
    pub diameter: Coord,
    pub drill: Coord,
    pub min_diameter: Coord,
    pub min_drill: Coord,
    pub min_annular: Coord,
}

/// A rule (or part of one) the table does not apply, and why.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct InertRule {
    pub rule: String,
    pub reason: String,
}

/// What a rule table resolved, for reports and gates.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct RulesReport {
    pub classes: Vec<String>,
    pub nets_per_class: BTreeMap<String, usize>,
    /// Custom rules applied (at least one supported constraint), file order.
    pub applied_rules: Vec<String>,
    pub inert: Vec<InertRule>,
    /// Nets whose name matches patterns of more than one class. They take
    /// the highest-precedence class (lowest `priority`, then name); KiCad
    /// composites them field by field.
    pub ambiguous_nets: Vec<String>,
    pub board: BoardRules,
}

/// Every rule question the router asks. Implementations must be pure:
/// the same question always gets the same answer.
pub trait RuleTable: Send + Sync {
    fn class_of(&self, net: Option<NetId>) -> ClassId;
    fn class(&self, class: ClassId) -> &NetClassRules;
    /// Minimum edge-to-edge clearance between two items on `layer`. It
    /// should not depend on the order of `a` and `b` (KiCad tries a rule's
    /// A/B conditions both ways); callers that memoize answers ask in one
    /// fixed order, so a table that breaks this still gives deterministic
    /// results.
    fn clearance(&self, a: &RuleSubject<'_>, b: &RuleSubject<'_>, layer: LayerId) -> Coord;
    fn track_width(&self, subject: &RuleSubject<'_>, layer: LayerId) -> WidthRule;
    fn via(&self, subject: &RuleSubject<'_>) -> ViaRule;
    /// Smallest allowed angle between consecutive track segments, degrees.
    fn min_track_angle_deg(&self, subject: &RuleSubject<'_>, layer: LayerId) -> Option<f64>;
    fn min_segment_length(&self, subject: &RuleSubject<'_>, layer: LayerId) -> Option<Coord>;
    fn edge_clearance(&self) -> Coord;
    fn hole_clearance(&self) -> Coord;
    fn hole_to_hole(&self) -> Coord;
    /// An upper bound on any clearance this table can return; index
    /// inflation uses it.
    fn max_clearance(&self) -> Coord;
    fn inert_rules(&self) -> &[InertRule];
}

#[derive(Debug, thiserror::Error)]
pub enum RuleError {
    #[error("project file is not valid JSON: {0}")]
    ProjectJson(String),
    #[error("project file has no net class named 'Default'")]
    NoDefaultClass,
    #[error("net class #{index} is malformed: {reason}")]
    BadClass { index: usize, reason: String },
    #[error("design rules file could not be parsed: {0}")]
    Dru(String),
}

// ─── The project table ───────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ConstraintKind {
    Clearance,
    TrackWidth,
    ViaDiameter,
    HoleSize,
    AnnularWidth,
    TrackSegmentLength,
    TrackAngle,
}

impl ConstraintKind {
    fn dru_name(self) -> &'static str {
        match self {
            ConstraintKind::Clearance => "clearance",
            ConstraintKind::TrackWidth => "track_width",
            ConstraintKind::ViaDiameter => "via_diameter",
            ConstraintKind::HoleSize => "hole_size",
            ConstraintKind::AnnularWidth => "annular_width",
            ConstraintKind::TrackSegmentLength => "track_segment_length",
            ConstraintKind::TrackAngle => "track_angle",
        }
    }

    fn is_via(self) -> bool {
        matches!(
            self,
            ConstraintKind::ViaDiameter | ConstraintKind::HoleSize | ConstraintKind::AnnularWidth
        )
    }

    /// Whether the table applies an `opt` and a `max` of this kind.
    fn takes_opt_and_max(self) -> bool {
        matches!(
            self,
            ConstraintKind::TrackWidth | ConstraintKind::ViaDiameter | ConstraintKind::HoleSize
        )
    }
}

/// One supported constraint; angles in degrees, lengths in nm. A length is
/// checked into [`Coord`] range when the rule is compiled, so [`to_nm`] on it
/// is exact.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Constraint {
    kind: ConstraintKind,
    min: Option<f64>,
    opt: Option<f64>,
    max: Option<f64>,
}

/// A compiled length back to nm. Exact: [`parse_length_nm`] only returns
/// whole nanometres within [`konnect_geom::MAX_COORD`].
fn to_nm(v: f64) -> Coord {
    v as Coord
}

#[derive(Debug, Clone)]
struct CompiledRule {
    name: String,
    layer: Option<LayerId>,
    condition: Expr,
    constraints: Vec<Constraint>,
}

/// A [`RuleTable`] compiled from a project's own files, at runtime.
#[derive(Debug, Clone)]
pub struct ProjectRuleTable {
    classes: Vec<NetClassRules>,
    class_by_net: Vec<ClassId>,
    board: BoardRules,
    rules: Vec<CompiledRule>,
    inert: Vec<InertRule>,
    ambiguous_nets: Vec<String>,
    net_names: Vec<String>,
    references: Vec<Option<String>>,
    max_clearance: Coord,
}

impl ProjectRuleTable {
    /// Compile the table for `board` from the `.kicad_pro` JSON text and the
    /// optional `.kicad_dru` text.
    pub fn from_project(
        pro_json: &str,
        dru_text: Option<&str>,
        board: &RouteBoard,
    ) -> Result<Self, RuleError> {
        let pro: serde_json::Value =
            serde_json::from_str(pro_json).map_err(|e| RuleError::ProjectJson(e.to_string()))?;
        let classes = parse_classes(&pro["net_settings"]["classes"])?;
        let board_rules = parse_board_rules(&pro["board"]["design_settings"]["rules"]);
        let (class_by_net, ambiguous_nets, mut inert) =
            assign_classes(&pro["net_settings"], &classes, board);

        let mut rules = Vec::new();
        if let Some(text) = dru_text {
            let (compiled, dropped) = compile_dru(text, board)?;
            rules = compiled;
            inert.extend(dropped);
        }

        let mut max_clearance = classes.iter().map(|c| c.clearance).max().unwrap_or(0);
        for r in &rules {
            for c in r
                .constraints
                .iter()
                .filter(|c| c.kind == ConstraintKind::Clearance)
            {
                if let Some(v) = c.min {
                    max_clearance = max_clearance.max(to_nm(v));
                }
            }
        }
        max_clearance = max_clearance
            .max(board_rules.min_clearance)
            .max(board_rules.edge_clearance)
            .max(board_rules.hole_clearance)
            .max(board_rules.hole_to_hole);
        for pad in &board.pads {
            if let Some(local) = pad.local_clearance {
                max_clearance = max_clearance.max(local);
            }
        }

        Ok(ProjectRuleTable {
            classes,
            class_by_net,
            board: board_rules,
            rules,
            inert,
            ambiguous_nets,
            net_names: board.nets.iter().map(|n| n.name.clone()).collect(),
            references: board
                .footprints
                .iter()
                .map(|f| f.reference.clone())
                .collect(),
            max_clearance,
        })
    }

    /// Write each net's class into the board model.
    pub fn assign_classes(&self, board: &mut RouteBoard) {
        for net in &mut board.nets {
            net.class = self.class_of(Some(net.id));
        }
    }

    /// Put `net` in the class named `class`, overriding the patterns: for a
    /// caller holding KiCad's own effective netclass (IPC, or a DSN export
    /// made from it) that disagrees with the project-file resolution.
    /// A pinned net is settled, so it leaves the report's ambiguous list.
    /// Returns `false`, changing nothing, when the class or net is unknown.
    pub fn pin_net_class(&mut self, net: NetId, class: &str) -> bool {
        let Some(id) = self.classes.iter().position(|c| c.name == class) else {
            return false;
        };
        let Some(slot) = self.class_by_net.get_mut(net.index()) else {
            return false;
        };
        *slot = ClassId(id as u16);
        if let Some(name) = self.net_names.get(net.index()) {
            self.ambiguous_nets.retain(|n| n != name);
        }
        true
    }

    pub fn board_rules(&self) -> &BoardRules {
        &self.board
    }

    pub fn classes(&self) -> &[NetClassRules] {
        &self.classes
    }

    pub fn report(&self) -> RulesReport {
        let mut nets_per_class: BTreeMap<String, usize> = BTreeMap::new();
        for c in &self.class_by_net {
            *nets_per_class
                .entry(self.classes[c.index()].name.clone())
                .or_default() += 1;
        }
        RulesReport {
            classes: self.classes.iter().map(|c| c.name.clone()).collect(),
            nets_per_class,
            applied_rules: self.rules.iter().map(|r| r.name.clone()).collect(),
            inert: self.inert.clone(),
            ambiguous_nets: self.ambiguous_nets.clone(),
            board: self.board,
        }
    }

    /// The last matching rule's constraint of `kind`: rules are tried
    /// bottom-up, two-item conditions in both orders.
    fn last_match(
        &self,
        kind: ConstraintKind,
        layer: Option<LayerId>,
        a: &RuleSubject<'_>,
        b: Option<&RuleSubject<'_>>,
    ) -> Option<Constraint> {
        let va = View { table: self, s: a };
        let vb = b.map(|s| View { table: self, s });
        for rule in self.rules.iter().rev() {
            if let (Some(rl), Some(l)) = (rule.layer, layer) {
                if rl != l {
                    continue;
                }
            } else if rule.layer.is_some() && layer.is_none() {
                continue;
            }
            let Some(c) = rule.constraints.iter().find(|c| c.kind == kind) else {
                continue;
            };
            let matched = match &vb {
                None => rule.condition.eval(&va, None),
                Some(vb) => {
                    rule.condition.eval(&va, Some(vb)) || rule.condition.eval(vb, Some(&va))
                }
            };
            if matched {
                return Some(*c);
            }
        }
        None
    }

    fn class_rules(&self, net: Option<NetId>) -> &NetClassRules {
        &self.classes[self.class_of(net).index()]
    }
}

/// A subject plus the table, so conditions can read class names and
/// references.
struct View<'t, 's> {
    table: &'t ProjectRuleTable,
    s: &'s RuleSubject<'s>,
}

impl SubjectView for View<'_, '_> {
    fn prop(&self, prop: Prop) -> &str {
        match prop {
            Prop::NetName => self
                .s
                .net
                .and_then(|n| self.table.net_names.get(n.index()))
                .map_or("", String::as_str),
            Prop::NetClass => &self.table.class_rules(self.s.net).name,
            Prop::Type => self.s.kind.kicad_type_name(),
            Prop::Reference => self
                .s
                .footprint
                .and_then(|f| self.table.references.get(f.index()))
                .and_then(|r| r.as_deref())
                .unwrap_or(""),
        }
    }

    fn intersects_courtyard(&self, footprint: FootprintId) -> bool {
        self.s.courtyards.contains(&footprint)
    }

    fn enclosed_by_area(&self, area: &str) -> bool {
        self.s.areas.contains(&area)
    }
}

impl RuleTable for ProjectRuleTable {
    fn class_of(&self, net: Option<NetId>) -> ClassId {
        net.and_then(|n| self.class_by_net.get(n.index()).copied())
            .unwrap_or(ClassId(0))
    }

    fn class(&self, class: ClassId) -> &NetClassRules {
        &self.classes[class.index()]
    }

    fn clearance(&self, a: &RuleSubject<'_>, b: &RuleSubject<'_>, layer: LayerId) -> Coord {
        let resolved = match (a.local_clearance, b.local_clearance) {
            (None, None) => {
                match self.last_match(ConstraintKind::Clearance, Some(layer), a, Some(b)) {
                    Some(Constraint { min: Some(v), .. }) => to_nm(v),
                    _ => self
                        .class_rules(a.net)
                        .clearance
                        .max(self.class_rules(b.net).clearance),
                }
            }
            (la, lb) => la.unwrap_or(0).max(lb.unwrap_or(0)),
        };
        resolved.max(self.board.min_clearance)
    }

    fn track_width(&self, subject: &RuleSubject<'_>, layer: LayerId) -> WidthRule {
        let class = self.class_rules(subject.net);
        let rule = self.last_match(ConstraintKind::TrackWidth, Some(layer), subject, None);
        let min = rule
            .and_then(|c| c.min)
            .map_or(0, to_nm)
            .max(self.board.min_track_width);
        WidthRule {
            min,
            preferred: preferred(rule, class.track_width, min),
        }
    }

    fn via(&self, subject: &RuleSubject<'_>) -> ViaRule {
        let class = self.class_rules(subject.net);
        let rule = |kind: ConstraintKind| self.last_match(kind, None, subject, None);
        let floor = |c: Option<Constraint>| c.and_then(|c| c.min).map_or(0, to_nm);
        let (diameter_rule, drill_rule) = (
            rule(ConstraintKind::ViaDiameter),
            rule(ConstraintKind::HoleSize),
        );
        let min_annular =
            floor(rule(ConstraintKind::AnnularWidth)).max(self.board.min_via_annular_width);
        let min_drill = floor(drill_rule).max(self.board.min_through_hole_diameter);
        // A via that meets the drill floor must still meet the annular one.
        let min_diameter = floor(diameter_rule)
            .max(self.board.min_via_diameter)
            .max(min_drill + 2 * min_annular);
        let drill = preferred(drill_rule, class.via_drill, min_drill);
        let diameter =
            preferred(diameter_rule, class.via_diameter, min_diameter).max(drill + 2 * min_annular);
        ViaRule {
            diameter,
            drill,
            min_diameter,
            min_drill,
            min_annular,
        }
    }

    fn min_track_angle_deg(&self, subject: &RuleSubject<'_>, layer: LayerId) -> Option<f64> {
        self.last_match(ConstraintKind::TrackAngle, Some(layer), subject, None)
            .and_then(|c| c.min)
    }

    fn min_segment_length(&self, subject: &RuleSubject<'_>, layer: LayerId) -> Option<Coord> {
        self.last_match(
            ConstraintKind::TrackSegmentLength,
            Some(layer),
            subject,
            None,
        )
        .and_then(|c| c.min)
        .map(to_nm)
    }

    fn edge_clearance(&self) -> Coord {
        self.board.edge_clearance
    }

    fn hole_clearance(&self) -> Coord {
        self.board.hole_clearance
    }

    fn hole_to_hole(&self) -> Coord {
        self.board.hole_to_hole
    }

    fn max_clearance(&self) -> Coord {
        self.max_clearance
    }

    fn inert_rules(&self) -> &[InertRule] {
        &self.inert
    }
}

/// What to draw: the rule's `opt` (else the class value), capped by the
/// rule's `max`, never below `min`.
fn preferred(rule: Option<Constraint>, class: Coord, min: Coord) -> Coord {
    let opt = rule.and_then(|c| c.opt).map_or(class, to_nm);
    let capped = match rule.and_then(|c| c.max) {
        Some(max) => opt.min(to_nm(max)),
        None => opt,
    };
    capped.max(min)
}

// ─── Project file readers ────────────────────────────────────────────────────

fn mm_field(v: &serde_json::Value, key: &str) -> Option<Coord> {
    v.get(key)?.as_f64().and_then(mm_to_nm)
}

/// Net classes, `Default` first. A key a named class omits inherits the
/// Default class's value, matching KiCad's netclass inheritance.
fn parse_classes(v: &serde_json::Value) -> Result<Vec<NetClassRules>, RuleError> {
    let list = v.as_array().map(Vec::as_slice).unwrap_or(&[]);
    let default_index = list
        .iter()
        .position(|c| c["name"].as_str() == Some("Default"))
        .ok_or(RuleError::NoDefaultClass)?;
    let read = |index: usize, c: &serde_json::Value, base: Option<&NetClassRules>| {
        let name = c["name"]
            .as_str()
            .ok_or_else(|| RuleError::BadClass {
                index,
                reason: "missing name".to_string(),
            })?
            .to_string();
        let field = |key: &str, inherited: Option<Coord>| {
            mm_field(c, key)
                .or(inherited)
                .ok_or_else(|| RuleError::BadClass {
                    index,
                    reason: format!("class '{name}' has no usable '{key}'"),
                })
        };
        Ok(NetClassRules {
            clearance: field("clearance", base.map(|b| b.clearance))?,
            track_width: field("track_width", base.map(|b| b.track_width))?,
            via_diameter: field("via_diameter", base.map(|b| b.via_diameter))?,
            via_drill: field("via_drill", base.map(|b| b.via_drill))?,
            diff_pair_width: mm_field(c, "diff_pair_width")
                .or(base.and_then(|b| b.diff_pair_width)),
            diff_pair_gap: mm_field(c, "diff_pair_gap").or(base.and_then(|b| b.diff_pair_gap)),
            name,
        })
    };
    let default = read(default_index, &list[default_index], None)?;
    let mut out = vec![default];
    for (i, c) in list.iter().enumerate() {
        if i == default_index {
            continue;
        }
        let class = read(i, c, Some(&out[0]))?;
        out.push(class);
    }
    Ok(out)
}

fn parse_board_rules(v: &serde_json::Value) -> BoardRules {
    let f = |key: &str| mm_field(v, key).unwrap_or(0);
    BoardRules {
        min_clearance: f("min_clearance"),
        min_track_width: f("min_track_width"),
        min_via_diameter: f("min_via_diameter"),
        min_via_annular_width: f("min_via_annular_width"),
        min_through_hole_diameter: f("min_through_hole_diameter"),
        edge_clearance: f("min_copper_edge_clearance"),
        hole_clearance: f("min_hole_clearance"),
        hole_to_hole: f("min_hole_to_hole"),
        max_error: f("max_error"),
    }
}

/// Each board net's class, resolved as KiCad 10 resolves it
/// ([`netclass::NetclassResolver`]): explicit `netclass_assignments`, then
/// every matching `netclass_patterns` entry (whole-name regex or wildcard,
/// bus patterns expanded), then a bus-notation name's members, then
/// `Default`. A net several classes match takes the highest-precedence one
/// and is reported ambiguous. Patterns naming an unknown class are reported
/// inert.
fn assign_classes(
    net_settings: &serde_json::Value,
    classes: &[NetClassRules],
    board: &RouteBoard,
) -> (Vec<ClassId>, Vec<String>, Vec<InertRule>) {
    let class_id = |name: &str| {
        classes
            .iter()
            .position(|c| c.name == name)
            .map(|i| ClassId(i as u16))
    };
    let mut inert = Vec::new();
    let mut patterns: Vec<(String, ClassId)> = Vec::new();
    for p in net_settings["netclass_patterns"]
        .as_array()
        .map(Vec::as_slice)
        .unwrap_or(&[])
    {
        let (Some(pattern), Some(class)) = (p["pattern"].as_str(), p["netclass"].as_str()) else {
            continue;
        };
        match class_id(class) {
            Some(id) => patterns.push((pattern.to_string(), id)),
            None => inert.push(InertRule {
                rule: format!("netclass pattern '{pattern}'"),
                reason: format!("class '{class}' is not defined"),
            }),
        }
    }
    let mut explicit: BTreeMap<String, Vec<ClassId>> = BTreeMap::new();
    if let Some(map) = net_settings["netclass_assignments"].as_object() {
        for (net, v) in map {
            let names: Vec<&str> = match v {
                serde_json::Value::String(s) => vec![s.as_str()],
                serde_json::Value::Array(a) => a.iter().filter_map(|x| x.as_str()).collect(),
                _ => Vec::new(),
            };
            let ids: Vec<ClassId> = names.into_iter().filter_map(class_id).collect();
            if !ids.is_empty() {
                explicit.insert(net.clone(), ids);
            }
        }
    }
    let listed = net_settings["classes"]
        .as_array()
        .map(Vec::as_slice)
        .unwrap_or(&[]);
    let priorities: Vec<Option<i64>> = classes
        .iter()
        .map(|c| {
            listed
                .iter()
                .find(|j| j["name"].as_str() == Some(c.name.as_str()))
                .and_then(|j| j["priority"].as_i64())
        })
        .collect();
    let resolver = netclass::NetclassResolver::new(classes, &priorities, &patterns, explicit);
    let mut ambiguous = Vec::new();
    let by_net = board
        .nets
        .iter()
        .map(|net| {
            let r = resolver.resolve(&net.name);
            if r.ambiguous {
                ambiguous.push(net.name.clone());
            }
            r.class
        })
        .collect();
    (by_net, ambiguous, inert)
}

// ─── DRU compiler ────────────────────────────────────────────────────────────

struct BoardNames<'a>(&'a RouteBoard);

impl NameResolver for BoardNames<'_> {
    fn footprint(&self, reference: &str) -> Option<FootprintId> {
        self.0.footprint_by_reference(reference)
    }

    fn has_area(&self, name: &str) -> bool {
        self.0
            .keepouts
            .iter()
            .any(|k| k.name.as_deref() == Some(name))
            || self
                .0
                .zones
                .iter()
                .any(|z| z.zone_name.as_deref() == Some(name))
    }
}

/// Compile every `(rule …)`; unsupported rules and constraints come back
/// as inert entries.
fn compile_dru(
    text: &str,
    board: &RouteBoard,
) -> Result<(Vec<CompiledRule>, Vec<InertRule>), RuleError> {
    let tree = konnect_sexp::parse_sexp(&format!("(kicad_dru\n{text}\n)"))
        .map_err(|e| RuleError::Dru(e.to_string()))?;
    let names = BoardNames(board);
    let mut rules = Vec::new();
    let mut inert = Vec::new();
    for node in tree.find_all("rule") {
        let name = node
            .get(1)
            .and_then(|n| n.as_str())
            .unwrap_or("<unnamed>")
            .to_string();
        match compile_rule(&name, node, board, &names, &mut inert) {
            Ok(rule) => rules.push(rule),
            Err(reason) => inert.push(InertRule { rule: name, reason }),
        }
    }
    Ok((rules, inert))
}

fn compile_rule(
    name: &str,
    node: &konnect_sexp::SexpNode,
    board: &RouteBoard,
    names: &dyn NameResolver,
    inert: &mut Vec<InertRule>,
) -> Result<CompiledRule, String> {
    let mut layer = None;
    let mut condition = Expr::True;
    let mut constraints = Vec::new();
    for child in node.children().unwrap_or(&[]).iter().skip(2) {
        match child.head() {
            Some("layer") => {
                let token = child.get(1).and_then(|n| n.as_str()).unwrap_or("");
                layer =
                    Some(board.layers.by_name(token).ok_or_else(|| {
                        format!("layer clause '{token}' is not a copper layer name")
                    })?);
            }
            Some("condition") => {
                let src = child
                    .get(1)
                    .and_then(|n| n.as_str())
                    .ok_or("condition has no expression")?;
                condition = parse_condition(src, names)?;
            }
            Some("constraint") => match compile_constraint(child) {
                Ok(c) => constraints.push(c),
                Err(reason) => inert.push(InertRule {
                    rule: name.to_string(),
                    reason,
                }),
            },
            Some("severity") => match child.get(1).and_then(|n| n.as_str()) {
                Some("error" | "warning") => {}
                Some("ignore") => {
                    return Err("severity 'ignore': KiCad does not enforce the rule".to_string())
                }
                other => {
                    return Err(format!(
                        "severity '{}' is not understood",
                        other.unwrap_or("")
                    ))
                }
            },
            Some(other) => return Err(format!("clause '{other}' is not supported")),
            None => return Err("malformed rule clause".to_string()),
        }
    }
    if layer.is_some() {
        // Vias span layers, and `via()` is asked without one.
        constraints.retain(|c| {
            if c.kind.is_via() {
                inert.push(InertRule {
                    rule: name.to_string(),
                    reason: format!(
                        "constraint '{}' scoped to one layer is not supported",
                        c.kind.dru_name()
                    ),
                });
            }
            !c.kind.is_via()
        });
    }
    if constraints.is_empty() {
        return Err("no supported constraint".to_string());
    }
    Ok(CompiledRule {
        name: name.to_string(),
        layer,
        condition,
        constraints,
    })
}

fn compile_constraint(node: &konnect_sexp::SexpNode) -> Result<Constraint, String> {
    let ty = node.get(1).and_then(|n| n.as_str()).unwrap_or("");
    let kind = match ty {
        "clearance" => ConstraintKind::Clearance,
        "track_width" => ConstraintKind::TrackWidth,
        "via_diameter" => ConstraintKind::ViaDiameter,
        "hole_size" => ConstraintKind::HoleSize,
        "annular_width" => ConstraintKind::AnnularWidth,
        "track_segment_length" => ConstraintKind::TrackSegmentLength,
        "track_angle" => ConstraintKind::TrackAngle,
        other => return Err(format!("constraint '{other}' is not supported")),
    };
    let mut c = Constraint {
        kind,
        min: None,
        opt: None,
        max: None,
    };
    for child in node.children().unwrap_or(&[]).iter().skip(2) {
        let head = child
            .head()
            .ok_or_else(|| format!("constraint '{ty}' has a malformed limit"))?;
        let raw = child
            .get(1)
            .and_then(|n| n.as_str())
            .ok_or_else(|| format!("constraint '{ty}' {head} has no value"))?;
        let value = if kind == ConstraintKind::TrackAngle {
            parse_angle_deg(raw)
        } else {
            parse_length_nm(raw)
        }
        .ok_or_else(|| {
            format!("constraint '{ty}' {head} value '{raw}' is not understood or out of range")
        })?;
        match head {
            "min" => c.min = Some(value),
            "opt" | "max" if !kind.takes_opt_and_max() => {
                return Err(format!("constraint '{ty}' {head} is not supported"))
            }
            "opt" => c.opt = Some(value),
            "max" => c.max = Some(value),
            other => {
                return Err(format!(
                    "constraint '{ty}' option '{other}' is not supported"
                ))
            }
        }
    }
    if let (Some(min), Some(max)) = (c.min, c.max) {
        if min > max {
            return Err(format!("constraint '{ty}' has min above max"));
        }
    }
    Ok(c)
}

/// A DRU length with its unit (`0.25mm`, `6mil`, `0.006in`, `150um`); a bare
/// number is millimetres. Returns whole nm, or `None` when the value is not
/// a number or does not fit a [`Coord`] (beyond [`konnect_geom::MAX_COORD`]).
fn parse_length_nm(raw: &str) -> Option<f64> {
    let units: [(&str, f64); 5] = [
        ("mm", 1.0),
        ("mil", 0.0254),
        ("in", 25.4),
        ("um", 0.001),
        ("µm", 0.001),
    ];
    let (number, mm_per_unit) = units
        .iter()
        .find_map(|(suffix, f)| raw.strip_suffix(suffix).map(|n| (n, *f)))
        .unwrap_or((raw, 1.0));
    let v: f64 = number.trim().parse().ok()?;
    mm_to_nm(v * mm_per_unit).map(f64::from)
}

fn parse_angle_deg(raw: &str) -> Option<f64> {
    let number = raw.strip_suffix("deg").unwrap_or(raw);
    number.trim().parse().ok().filter(|v: &f64| v.is_finite())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synthetic::BoardBuilder;

    // Invented values: every one differs from the others so a test that
    // reads the wrong field fails.
    const PRO: &str = r#"{
        "board": { "design_settings": { "rules": {
            "min_clearance": 0.12, "min_track_width": 0.09, "min_via_diameter": 0.3,
            "min_via_annular_width": 0.07, "min_through_hole_diameter": 0.18,
            "min_copper_edge_clearance": 0.4, "min_hole_clearance": 0.22, "min_hole_to_hole": 0.3,
            "max_error": 0.004
        } } },
        "net_settings": {
            "classes": [
                { "name": "Fast", "clearance": 0.12, "track_width": 0.09 },
                { "name": "Default", "clearance": 0.18, "track_width": 0.22,
                  "via_diameter": 0.55, "via_drill": 0.28 },
                { "name": "Pair", "clearance": 0.18, "track_width": 0.115,
                  "via_diameter": 0.55, "via_drill": 0.28 },
                { "name": "Power", "clearance": 0.25, "track_width": 0.35,
                  "via_diameter": 0.7, "via_drill": 0.35 }
            ],
            "netclass_patterns": [
                { "netclass": "Fast", "pattern": "IO_*" },
                { "netclass": "Pair", "pattern": "LVDS_*" },
                { "netclass": "Power", "pattern": "IO_PWR" },
                { "netclass": "Power", "pattern": "PWR_*" },
                { "netclass": "Missing", "pattern": "X" }
            ],
            "netclass_assignments": null
        }
    }"#;

    const DRU: &str = r#"(version 1)
(rule "Track floor"
  (constraint track_width (min 0.09mm)))
(rule "Area only"
  (condition "A.enclosedByArea('nowhere')")
  (constraint clearance (min 0.3mm)))
(rule "Big vias away from IC9"
  (condition "A.Type == 'Via' && !A.intersectsCourtyard('IC9')")
  (constraint via_diameter (min 0.6mm))
  (constraint hole_size (min 0.3mm)))
(rule "IC7 keep wide"
  (condition "A.intersectsCourtyard('IC7') && B.intersectsCourtyard('IC7')")
  (constraint clearance (min 0.2mm)))
(rule "Pair gap"
  (condition "A.NetClass == 'Pair' && B.NetClass == 'Pair'")
  (constraint clearance (min 0.13mm)))
(rule "Short segments"
  (layer "F.Cu")
  (condition "A.NetClass == 'Pair'")
  (constraint track_segment_length (min 0.12mm))
  (constraint length (max 50mm)))
(rule "Angles"
  (constraint track_angle (min 80)))
(rule "Power caps"
  (condition "A.NetClass == 'Power'")
  (constraint track_width (max 0.2mm))
  (constraint via_diameter (opt 0.65mm) (max 0.65mm)))
(rule "Ignored"
  (severity ignore)
  (constraint clearance (min 1mm)))
(rule "Clearance cap"
  (constraint clearance (max 0.5mm)))
(rule "Layer vias"
  (layer "B.Cu")
  (constraint hole_size (min 0.4mm))
  (constraint track_width (min 0.1mm)))
(rule "Huge"
  (constraint clearance (min 5000mm)))
"#;

    fn board() -> RouteBoard {
        let mut b = BoardBuilder::new(20.0, 20.0, &["F.Cu", "B.Cu"]);
        for net in ["IO_A", "IO_PWR", "LVDS_N", "LVDS_P", "GND", "PWR_MAIN"] {
            b.net(net);
        }
        b.footprint("IC7", (5.0, 5.0));
        b.footprint("IC9", (15.0, 15.0));
        b.build()
    }

    fn table(b: &RouteBoard) -> ProjectRuleTable {
        ProjectRuleTable::from_project(PRO, Some(DRU), b).unwrap()
    }

    #[test]
    fn classes_inherit_from_default_and_patterns_assign_in_order() {
        let b = board();
        let t = table(&b);
        assert_eq!(t.classes()[0].name, "Default");
        let fast = t.class(t.class_of(b.net_by_name("IO_A")));
        assert_eq!(fast.name, "Fast");
        // Fast omits via sizes: they come from Default.
        assert_eq!((fast.via_diameter, fast.via_drill), (550_000, 280_000));
        // IO_PWR matches both IO_* and IO_PWR: first match wins, and it is
        // reported as ambiguous.
        assert_eq!(t.class(t.class_of(b.net_by_name("IO_PWR"))).name, "Fast");
        assert_eq!(t.report().ambiguous_nets, vec!["IO_PWR".to_string()]);
        assert_eq!(t.class(t.class_of(b.net_by_name("GND"))).name, "Default");
        assert_eq!(t.class(t.class_of(None)).name, "Default");
    }

    #[test]
    fn inert_rules_are_reported_with_reasons() {
        let b = board();
        let t = table(&b);
        let inert = t.inert_rules();
        assert!(inert
            .iter()
            .any(|r| r.rule == "Area only" && r.reason.contains("nowhere")));
        assert!(inert
            .iter()
            .any(|r| r.rule == "Short segments" && r.reason.contains("'length'")));
        assert!(inert.iter().any(|r| r.reason.contains("'Missing'")));
        let has = |rule: &str, reason: &str| {
            inert
                .iter()
                .any(|r| r.rule == rule && r.reason.contains(reason))
        };
        assert!(has("Ignored", "severity 'ignore'"));
        assert!(has("Clearance cap", "max is not supported"));
        assert!(has("Layer vias", "'hole_size' scoped to one layer"));
        assert!(has("Huge", "out of range"));
        // A rule with one unsupported constraint still applies the others.
        let applied = t.report().applied_rules;
        for rule in ["Short segments", "Layer vias", "Power caps"] {
            assert!(applied.contains(&rule.to_string()), "{rule}");
        }
        for rule in ["Ignored", "Clearance cap", "Huge"] {
            assert!(!applied.contains(&rule.to_string()), "{rule}");
        }
    }

    #[test]
    fn clearance_precedence_local_then_custom_then_class_then_floor() {
        let b = board();
        let t = table(&b);
        let f = LayerId(0);
        let ic7 = b.footprint_by_reference("IC7").unwrap();
        let in_ic7 = [ic7];
        let io = RuleSubject::new(ItemKind::Track, b.net_by_name("IO_A"));
        let gnd = RuleSubject::new(ItemKind::Track, b.net_by_name("GND"));
        // Class fallback: max(0.12, 0.18).
        assert_eq!(t.clearance(&io, &gnd, f), 180_000);
        // Fast vs Fast: class 0.12, board floor 0.12.
        assert_eq!(t.clearance(&io, &io, f), 120_000);
        // Both inside IC7's courtyard: the custom rule raises it to 0.2.
        let io_ic7 = io.with_courtyards(&in_ic7);
        assert_eq!(t.clearance(&io_ic7, &io_ic7, f), 200_000);
        // Pair vs pair lowers 0.18 to 0.13 — "max of everything" would be wrong.
        let n = RuleSubject::new(ItemKind::Track, b.net_by_name("LVDS_N"));
        let p = RuleSubject::new(ItemKind::Track, b.net_by_name("LVDS_P"));
        assert_eq!(t.clearance(&n, &p, f), 130_000);
        // Local overrides beat the class...
        let pad = RuleSubject::new(ItemKind::Pad, b.net_by_name("GND"))
            .with_local_clearance(Some(300_000));
        assert_eq!(t.clearance(&pad, &gnd, f), 300_000);
        // ...and every custom rule, even when lower, with the board minimum
        // still a floor.
        let pad_ic7 = RuleSubject::new(ItemKind::Pad, b.net_by_name("IO_A"))
            .with_courtyards(&in_ic7)
            .with_local_clearance(Some(150_000));
        assert_eq!(t.clearance(&pad_ic7, &io_ic7, f), 150_000);
        let tiny = pad_ic7.with_local_clearance(Some(50_000));
        assert_eq!(t.clearance(&tiny, &io_ic7, f), 120_000);
    }

    #[test]
    fn via_floor_and_single_item_constraints() {
        let b = board();
        let t = table(&b);
        let ic9 = [b.footprint_by_reference("IC9").unwrap()];
        let via = RuleSubject::new(ItemKind::Via, b.net_by_name("GND"));
        let v = t.via(&via);
        assert_eq!((v.min_diameter, v.min_drill), (600_000, 300_000));
        // Class size is below the floor, so the floor is what gets drawn.
        assert_eq!((v.diameter, v.drill), (600_000, 300_000));
        let v = t.via(&via.with_courtyards(&ic9));
        // Drill floor 0.18 plus twice the 0.07 annulus beats the 0.3 diameter floor.
        assert_eq!((v.min_diameter, v.min_drill), (320_000, 180_000));
        assert_eq!((v.diameter, v.drill), (550_000, 280_000));

        let pair = RuleSubject::new(ItemKind::Track, b.net_by_name("LVDS_P"));
        assert_eq!(t.min_segment_length(&pair, LayerId(0)), Some(120_000));
        assert_eq!(t.min_segment_length(&pair, LayerId(1)), None);
        assert_eq!(t.min_track_angle_deg(&pair, LayerId(1)), Some(80.0));
        let w = t.track_width(&pair, LayerId(0));
        assert_eq!((w.min, w.preferred), (90_000, 115_000));
        assert_eq!(t.edge_clearance(), 400_000);
        assert_eq!(t.hole_to_hole(), 300_000);
        assert_eq!(t.max_clearance(), 400_000);
    }

    #[test]
    fn opt_and_max_shape_what_gets_drawn() {
        let b = board();
        let t = table(&b);
        let power = b.net_by_name("PWR_MAIN");
        assert_eq!(t.class(t.class_of(power)).name, "Power");
        // Class width 0.35 capped at 0.2; the 0.09 floor still holds.
        let w = t.track_width(&RuleSubject::new(ItemKind::Track, power), LayerId(0));
        assert_eq!((w.min, w.preferred), (90_000, 200_000));
        // The diameter rule's opt replaces the class 0.7; the drill keeps
        // the class 0.35 over the 0.3 floor of the earlier rule.
        let v = t.via(&RuleSubject::new(ItemKind::Via, power));
        assert_eq!((v.diameter, v.drill), (650_000, 350_000));
        assert_eq!(v.min_diameter, 440_000);
        assert!(v.diameter >= v.drill + 2 * v.min_annular);
    }

    #[test]
    fn pinned_classes_override_patterns() {
        let b = board();
        let mut t = table(&b);
        let gnd = b.net_by_name("GND").unwrap();
        assert_eq!(t.class(t.class_of(Some(gnd))).name, "Default");
        assert!(t.pin_net_class(gnd, "Power"));
        assert_eq!(t.class(t.class_of(Some(gnd))).name, "Power");
        assert!(!t.pin_net_class(gnd, "Nope"));
        assert!(!t.pin_net_class(NetId(999), "Power"));
        assert_eq!(t.class(t.class_of(Some(gnd))).name, "Power");
        // Pinning an ambiguous net settles it.
        let io_pwr = b.net_by_name("IO_PWR").unwrap();
        assert_eq!(t.report().ambiguous_nets, vec!["IO_PWR".to_string()]);
        assert!(t.pin_net_class(io_pwr, "Fast"));
        assert!(t.report().ambiguous_nets.is_empty());
    }

    #[test]
    fn units_parse() {
        assert_eq!(parse_length_nm("0.25mm"), Some(250_000.0));
        assert_eq!(parse_length_nm("6mil"), Some(152_400.0));
        assert_eq!(parse_length_nm("0.1"), Some(100_000.0));
        assert_eq!(parse_length_nm("150um"), Some(150_000.0));
        assert_eq!(parse_length_nm("fast"), None);
        assert_eq!(parse_length_nm("5000mm"), None);
        assert_eq!(parse_length_nm("1e400"), None);
        assert_eq!(parse_angle_deg("75"), Some(75.0));
        assert_eq!(parse_angle_deg("45deg"), Some(45.0));
    }

    #[test]
    fn a_project_without_default_is_refused() {
        let b = board();
        let err = ProjectRuleTable::from_project(r#"{"net_settings":{"classes":[]}}"#, None, &b)
            .unwrap_err();
        assert!(matches!(err, RuleError::NoDefaultClass));
    }
}
