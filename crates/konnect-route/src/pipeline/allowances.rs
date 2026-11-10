//! Routing allowances and the class policy, by name.
//!
//! A [`RoutingAllowances`] names classes and layers as the project does, so
//! a tool call or a job spec can carry an owner's choices; [`resolve`]
//! checks every name against the board and its rules and writes the result
//! into [`PipelineSettings`]:
//!
//! - `class_layers`: the layers a class routes on (unlisted classes use
//!   every routable layer);
//! - `layer_fallback`: a second chance on more layers for a class's
//!   connections negotiation failed, optionally with tracks on some layers
//!   kept over solid copper of a reference plane
//!   ([`super::ClassFallback`]);
//! - `neck_down`: a narrower stub out of pads, within a reach of the pad,
//!   for the classes allowed ([`NeckDown`]);
//! - `exclude_classes`: classes left out of routing altogether (kept as
//!   zones or hand-drawn copper). Their connections are reported as
//!   excluded by policy, not as failures;
//! - `track_widths`: a track width for single nets, or for single
//!   connections of a net named by two pads, drawn instead of the net
//!   class width ([`TrackWidthSpec`]; resolved by
//!   [`RoutingAllowances::resolve_track_widths`], which needs the rules).
//!
//! Every allowance is off unless named. The one default is the class
//! policy: with `exclude_classes` unset, [`default_excluded_classes`]
//! leaves out wide-copper classes; an empty list routes every class.
//!
//! [`resolve`]: RoutingAllowances::resolve

use super::{ClassFallback, ConnectionWidth, PipelineSettings, ReferencePlane};
use crate::api::{ClassId, Connection, ConnectionId, LayerMask, NetId, PadId};
use crate::board::RouteBoard;
use crate::detail::negotiate::NeckDown;
use crate::rules::{NetClassRules, ProjectRuleTable, RuleTable};
use konnect_geom::{mm_to_nm, Coord};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

/// The track width at and above which a class is left out of routing by
/// default: power copper that belongs in zones or hand-drawn pours.
pub const WIDE_CLASS_MIN_WIDTH_NM: Coord = 500_000;

/// Routing allowances and the class policy, by class and layer name.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RoutingAllowances {
    /// Class name to the layers it routes on (`F.Cu`, `B.Cu`, `*.Cu`, …).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub class_layers: BTreeMap<String, Vec<String>>,
    /// Second-chance layers per class.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub layer_fallback: Vec<LayerFallbackSpec>,
    /// Neck-down at pads.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub neck_down: Option<NeckDownSpec>,
    /// Classes not routed at all. `None`: [`default_excluded_classes`];
    /// an empty list routes every class.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub exclude_classes: Option<Vec<String>>,
    /// Plane fanout: a pad that gets no via of its own may join a via of
    /// the same net placed for another pad within this distance, mm
    /// ([`crate::fanout::FanoutSettings::share_within`]). Unset: never.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fanout_share_within_mm: Option<f64>,
    /// Track widths for single nets and connections, instead of the net
    /// class width.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub track_widths: Vec<TrackWidthSpec>,
}

/// A track width for one net, or for one connection of it.
///
/// With only `net`, every track the run draws for the net (its plane
/// fanout stubs included) is `width_mm` wide. With `from` and `to`
/// (`REFERENCE.PAD`, both pads of the net), only the connection that joins
/// those two pads' copper is ([`ConnectionWidth`]), whatever the net's own
/// width: the more specific entry wins. The width replaces the net class
/// width (or a custom rule's `opt`) and may be narrower or wider than it;
/// it must lie within the board minimum and any custom `track_width` rule's
/// `min` and `max` for the net. Clearances are not changed.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TrackWidthSpec {
    pub net: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub from: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub to: Option<String>,
    pub width_mm: f64,
}

/// A class's second chance on more layers.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LayerFallbackSpec {
    pub class: String,
    /// Every layer of the second chance, the class's own included.
    pub layers: Vec<String>,
    /// Only the class's nets matching these names or wildcard patterns
    /// (`*`, `?`) get the second chance (every net of the class when
    /// unset). A pattern that matches no net of the class is refused.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub nets: Option<Vec<String>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub reference: Option<ReferenceSpec>,
}

/// Solid reference copper the fallback's tracks must stay over.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReferenceSpec {
    /// The plane layer whose filled copper is the reference.
    pub plane: String,
    /// The routing layers it applies to: fallback layers that are not
    /// among the class's own (`class_layers`, which a reference needs).
    /// Unset: every fallback layer beyond the class's own.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub over: Option<Vec<String>>,
    /// How far inside a filled piece a track's copper edge must stay, mm.
    #[serde(default)]
    pub margin_mm: f64,
}

/// Neck-down at pads.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NeckDownSpec {
    /// The narrow width, mm (never below the rules' minimum).
    #[serde(default = "default_neck_width")]
    pub width_mm: f64,
    /// How far from the pad the narrow stub may run, mm.
    #[serde(default = "default_neck_reach")]
    pub reach_mm: f64,
    /// Only these classes neck down (`None`: every class not denied).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub classes: Option<Vec<String>>,
    /// These classes never neck down.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub exclude_classes: Vec<String>,
}

fn default_neck_width() -> f64 {
    0.10
}

fn default_neck_reach() -> f64 {
    1.0
}

impl Default for NeckDownSpec {
    fn default() -> Self {
        NeckDownSpec {
            width_mm: default_neck_width(),
            reach_mm: default_neck_reach(),
            classes: None,
            exclude_classes: Vec::new(),
        }
    }
}

/// What [`RoutingAllowances::resolve`] set, by name, for reports.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct AllowanceReport {
    pub class_layers: BTreeMap<String, Vec<String>>,
    /// Class to (fallback layers, reference plane, layers over it).
    pub layer_fallback: BTreeMap<String, FallbackReport>,
    /// Classes that may neck down (empty when neck-down is off).
    pub neck_down_classes: Vec<String>,
    pub neck_down_width_nm: Option<Coord>,
    pub neck_down_reach_nm: Option<Coord>,
    pub excluded_classes: Vec<String>,
    /// Whether the exclusions are the default policy.
    pub exclusion_is_default: bool,
    /// Fanout via sharing distance (0: off).
    #[serde(default)]
    pub fanout_share_within_nm: Coord,
    /// The track widths that override a net class, in the order given.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub track_widths: Vec<TrackWidthReport>,
}

/// One [`TrackWidthSpec`] as resolved: the width drawn, and the net class
/// whose width it replaces on purpose.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TrackWidthReport {
    pub net: String,
    /// The two pads of a connection entry (`None`: the whole net).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub from: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub to: Option<String>,
    /// The width the entry's tracks are drawn with.
    pub width_nm: Coord,
    /// The net's class, whose width this entry overrides.
    pub overrides_class: String,
    pub class_width_nm: Coord,
    /// Open connections the run routed under this entry: for a net entry,
    /// those of the net no connection entry took; for a connection entry,
    /// 0 when no open connection joins its two pads' copper. `None` until
    /// the route stage has run.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub connections: Option<usize>,
    /// Net entries: the plane fanout stubs the run drew for the net, which
    /// take the net's width (a pad's fanout via is not a connection).
    /// `None` for a connection entry and until the route stage has run.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fanout_stubs: Option<usize>,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct FallbackReport {
    pub layers: Vec<String>,
    /// The nets it is limited to (empty: the whole class).
    pub nets: Vec<String>,
    pub reference_plane: Option<String>,
    pub over: Vec<String>,
    pub margin_nm: Coord,
}

/// A name that does not resolve, or a value out of range.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum AllowanceError {
    #[error("no net class named '{0}'")]
    UnknownClass(String),
    #[error("'{0}' names no copper layer of the board")]
    UnknownLayer(String),
    #[error("{0}")]
    Invalid(String),
    /// An entry of `track_widths` that cannot be used, by its position.
    #[error("track_widths[{index}]: {reason}")]
    TrackWidth { index: usize, reason: String },
}

fn mm_text(nm: Coord) -> String {
    format!("{} mm", f64::from(nm) / 1e6)
}

/// Classes left out of routing when the caller names none: every class
/// whose track width is at least [`WIDE_CLASS_MIN_WIDTH_NM`] (power copper
/// drawn as zones or by hand). `Default` is never left out.
pub fn default_excluded_classes(classes: &[NetClassRules]) -> BTreeSet<ClassId> {
    classes
        .iter()
        .enumerate()
        .skip(1)
        .filter(|(_, c)| c.track_width >= WIDE_CLASS_MIN_WIDTH_NM)
        .map(|(i, _)| ClassId(i as u16))
        .collect()
}

impl RoutingAllowances {
    /// Whether nothing is set (the class policy at its default).
    pub fn is_default(&self) -> bool {
        *self == RoutingAllowances::default()
    }

    /// Check every name against `board` (whose nets carry their classes,
    /// [`crate::ProjectRuleTable::assign_classes`]) and `classes` (in
    /// [`ClassId`] order) and write the allowances into `settings`, replacing its
    /// class layers, fallbacks, neck-down and exclusions.
    pub fn resolve(
        &self,
        board: &RouteBoard,
        classes: &[NetClassRules],
        settings: &mut PipelineSettings,
    ) -> Result<AllowanceReport, AllowanceError> {
        let class_id = |name: &str| {
            classes
                .iter()
                .position(|c| c.name == name)
                .map(|i| ClassId(i as u16))
                .ok_or_else(|| AllowanceError::UnknownClass(name.to_string()))
        };
        let mask = |names: &[String]| -> Result<LayerMask, AllowanceError> {
            if names.is_empty() {
                return Err(AllowanceError::Invalid("an empty layer list".to_string()));
            }
            let mut m = LayerMask::EMPTY;
            for n in names {
                let l = board.layers.expand(n.trim());
                if l.is_empty() {
                    return Err(AllowanceError::UnknownLayer(n.clone()));
                }
                m = m.union(l);
            }
            Ok(m)
        };
        let names = |m: LayerMask| -> Vec<String> {
            m.iter().map(|l| board.layers.name(l).to_string()).collect()
        };
        let mm = |v: f64, what: &str| -> Result<Coord, AllowanceError> {
            if !(v.is_finite() && v >= 0.0) {
                return Err(AllowanceError::Invalid(format!(
                    "{what} must be a finite, non-negative length in mm"
                )));
            }
            mm_to_nm(v).ok_or_else(|| AllowanceError::Invalid(format!("{what} is out of range")))
        };
        let mut report = AllowanceReport::default();

        let mut class_layers = BTreeMap::new();
        for (class, layers) in &self.class_layers {
            let m = mask(layers)?;
            class_layers.insert(class_id(class)?, m);
            report.class_layers.insert(class.clone(), names(m));
        }

        let mut fallback = BTreeMap::new();
        for f in &self.layer_fallback {
            let id = class_id(&f.class)?;
            if fallback.contains_key(&id) {
                return Err(AllowanceError::Invalid(format!(
                    "class '{}' has two layer fallbacks",
                    f.class
                )));
            }
            let layers = mask(&f.layers)?;
            let own = class_layers.get(&id).copied();
            let nets = match &f.nets {
                None => None,
                Some(patterns) => {
                    let mut set = BTreeSet::new();
                    for p in patterns {
                        let pat: Vec<char> = p.chars().collect();
                        // Whether this pattern matched any net, even one an
                        // earlier pattern already took.
                        let mut matched = false;
                        for n in &board.nets {
                            let chars: Vec<char> = n.name.chars().collect();
                            if n.class == id && crate::rules::wildcard_match_chars(&pat, &chars) {
                                set.insert(n.id);
                                matched = true;
                            }
                        }
                        if !matched {
                            return Err(AllowanceError::Invalid(format!(
                                "layer_fallback for '{}': '{p}' matches no net of the class",
                                f.class
                            )));
                        }
                    }
                    Some(set)
                }
            };
            let reference = match &f.reference {
                None => None,
                Some(r) => {
                    let plane = board
                        .layers
                        .by_name(r.plane.trim())
                        .ok_or_else(|| AllowanceError::UnknownLayer(r.plane.clone()))?;
                    // The reference governs only the layers the fallback
                    // adds: the class's own copper is never held to it.
                    let Some(own) = own else {
                        return Err(AllowanceError::Invalid(format!(
                            "class '{}': a reference needs the class's own layers in class_layers",
                            f.class
                        )));
                    };
                    let over = match &r.over {
                        Some(o) => mask(o)?,
                        None => LayerMask(layers.0 & !own.0),
                    };
                    if over.is_empty() {
                        return Err(AllowanceError::Invalid(format!(
                            "class '{}': the reference applies to no layer",
                            f.class
                        )));
                    }
                    if !over.intersection(own).is_empty() {
                        return Err(AllowanceError::Invalid(format!(
                            "class '{}': the reference's layers {:?} include the class's own layers",
                            f.class,
                            names(over)
                        )));
                    }
                    if !LayerMask(over.0 & !layers.0).is_empty() {
                        return Err(AllowanceError::Invalid(format!(
                            "class '{}': the reference's layers {:?} are not all fallback layers",
                            f.class,
                            names(over)
                        )));
                    }
                    Some(ReferencePlane {
                        plane,
                        over,
                        margin: mm(r.margin_mm, "margin_mm")?,
                    })
                }
            };
            report.layer_fallback.insert(
                f.class.clone(),
                FallbackReport {
                    layers: names(layers),
                    nets: nets
                        .iter()
                        .flatten()
                        .map(|n| board.nets[n.index()].name.clone())
                        .collect(),
                    reference_plane: reference.map(|r| board.layers.name(r.plane).to_string()),
                    over: reference.map(|r| names(r.over)).unwrap_or_default(),
                    margin_nm: reference.map_or(0, |r| r.margin),
                },
            );
            fallback.insert(
                id,
                ClassFallback {
                    layers,
                    reference,
                    nets,
                },
            );
        }

        let neck = match &self.neck_down {
            None => None,
            Some(n) => {
                let all: BTreeSet<ClassId> =
                    (0..classes.len()).map(|i| ClassId(i as u16)).collect();
                let mut excluded: BTreeSet<ClassId> = match &n.classes {
                    None => BTreeSet::new(),
                    Some(allow) => {
                        let allow: BTreeSet<ClassId> = allow
                            .iter()
                            .map(|c| class_id(c))
                            .collect::<Result<_, _>>()?;
                        all.difference(&allow).copied().collect()
                    }
                };
                for c in &n.exclude_classes {
                    excluded.insert(class_id(c)?);
                }
                let width = mm(n.width_mm, "neck_down.width_mm")?;
                let reach = mm(n.reach_mm, "neck_down.reach_mm")?;
                if width == 0 || reach == 0 {
                    return Err(AllowanceError::Invalid(
                        "neck_down needs a positive width and reach".to_string(),
                    ));
                }
                report.neck_down_classes = all
                    .difference(&excluded)
                    .map(|c| classes[c.index()].name.clone())
                    .collect();
                report.neck_down_width_nm = Some(width);
                report.neck_down_reach_nm = Some(reach);
                Some(NeckDown {
                    width,
                    reach,
                    excluded,
                })
            }
        };

        let excluded: BTreeSet<ClassId> = match &self.exclude_classes {
            None => {
                report.exclusion_is_default = true;
                default_excluded_classes(classes)
            }
            Some(list) => list.iter().map(|c| class_id(c)).collect::<Result<_, _>>()?,
        };
        report.excluded_classes = excluded
            .iter()
            .map(|c| classes[c.index()].name.clone())
            .collect();

        let share = match self.fanout_share_within_mm {
            None => 0,
            Some(v) => mm(v, "fanout_share_within_mm")?,
        };
        if let Some(f) = settings.fanout.as_mut() {
            f.share_within = share;
        }
        report.fanout_share_within_nm = share;

        settings.layer_policy.class_layers = class_layers;
        settings.class_fallback = fallback;
        settings.neck_down = neck;
        settings.excluded_classes = excluded;
        Ok(report)
    }

    /// Check `track_widths` against `board` and `rules` and apply them:
    /// a net entry becomes the net's width in `rules`
    /// ([`ProjectRuleTable::set_net_track_width`]), a connection entry one
    /// of `settings.connection_widths`; `report.track_widths` lists them.
    /// Call it after [`Self::resolve`] (it reads the excluded classes).
    ///
    /// Refused, naming the entry and changing nothing: a net or pad the
    /// board does not have, a pad of another net, one pad of a pair without
    /// the other, a width below the net's minimum track width on a routable
    /// layer or above a custom rule's maximum, a net whose class the run
    /// leaves unrouted, and a second entry for the same net or pad pair.
    pub fn resolve_track_widths(
        &self,
        board: &RouteBoard,
        rules: &mut ProjectRuleTable,
        settings: &mut PipelineSettings,
        report: &mut AllowanceReport,
    ) -> Result<(), AllowanceError> {
        let mut nets: Vec<(NetId, Coord)> = Vec::new();
        let mut connections: Vec<ConnectionWidth> = Vec::new();
        let mut reports = Vec::with_capacity(self.track_widths.len());
        for (index, spec) in self.track_widths.iter().enumerate() {
            let refuse = |reason: String| AllowanceError::TrackWidth { index, reason };
            let net = board
                .net_by_name(&spec.net)
                .filter(|_| !spec.net.is_empty())
                .ok_or_else(|| refuse(format!("no net named '{}' on the board", spec.net)))?;
            let class_id = rules.class_of(Some(net));
            let class = rules.class(class_id).clone();
            if settings.excluded_classes.contains(&class_id) {
                return Err(refuse(format!(
                    "net '{}' is in class '{}', which this run leaves unrouted \
                     (exclude_classes; by default every class at least {} wide). Pass \
                     exclude_classes without '{}' ([] routes every class) to route the net \
                     at this width",
                    spec.net,
                    class.name,
                    mm_text(WIDE_CLASS_MIN_WIDTH_NM),
                    class.name
                )));
            }
            let width = Some(spec.width_mm)
                .filter(|v| v.is_finite() && *v > 0.0)
                .and_then(mm_to_nm)
                .filter(|w| *w > 0)
                .ok_or_else(|| refuse("width_mm must be a positive length in mm".to_string()))?;
            for layer in board.routable.iter() {
                let (min, max) = rules.track_width_limits(net, layer);
                let name = board.layers.name(layer);
                if width < min {
                    return Err(refuse(format!(
                        "{} is below the minimum track width of net '{}' on {name}, {} \
                         (the board minimum or a custom track_width rule)",
                        mm_text(width),
                        spec.net,
                        mm_text(min)
                    )));
                }
                if let Some(max) = max.filter(|m| width > *m && *m >= min) {
                    return Err(refuse(format!(
                        "{} is above the maximum track width a custom rule allows net '{}' \
                         on {name}, {}",
                        mm_text(width),
                        spec.net,
                        mm_text(max)
                    )));
                }
            }
            let pads = |name: &str| -> Result<Vec<PadId>, AllowanceError> {
                let (reference, number) = name
                    .split_once('.')
                    .filter(|(r, n)| !r.is_empty() && !n.is_empty())
                    .ok_or_else(|| refuse(format!("'{name}' is not a pad as REFERENCE.PAD")))?;
                let footprint = board.footprint_by_reference(reference).ok_or_else(|| {
                    refuse(format!(
                        "'{reference}' is not the reference of exactly one footprint of the board"
                    ))
                })?;
                let named: Vec<&crate::board::Pad> = board
                    .pads
                    .iter()
                    .filter(|p| p.footprint == footprint && p.number == number)
                    .collect();
                if named.is_empty() {
                    return Err(refuse(format!(
                        "footprint '{reference}' has no pad '{number}'"
                    )));
                }
                let on_net: Vec<PadId> = named
                    .iter()
                    .filter(|p| p.net == Some(net))
                    .map(|p| p.id)
                    .collect();
                if on_net.is_empty() {
                    let other = named[0]
                        .net
                        .map_or("no net", |n| board.nets[n.index()].name.as_str());
                    return Err(refuse(format!(
                        "pad {name} is on '{other}', not on net '{}'",
                        spec.net
                    )));
                }
                Ok(on_net)
            };
            match (&spec.from, &spec.to) {
                (None, None) => {
                    if nets.iter().any(|(n, _)| *n == net) {
                        return Err(refuse(format!(
                            "net '{}' already has a width in an earlier entry",
                            spec.net
                        )));
                    }
                    nets.push((net, width));
                }
                (Some(from), Some(to)) => {
                    let (a, b) = (pads(from)?, pads(to)?);
                    if a == b {
                        return Err(refuse(format!("from and to both name pad {from}")));
                    }
                    if connections
                        .iter()
                        .any(|c| (c.from == a && c.to == b) || (c.from == b && c.to == a))
                    {
                        return Err(refuse(format!(
                            "the connection {from} to {to} already has a width in an earlier entry"
                        )));
                    }
                    connections.push(ConnectionWidth {
                        net,
                        from: a,
                        to: b,
                        width,
                    });
                }
                _ => {
                    return Err(refuse(
                        "from and to name a connection together: give both pads, or neither \
                         for the whole net"
                            .to_string(),
                    ))
                }
            }
            reports.push(TrackWidthReport {
                net: spec.net.clone(),
                from: spec.from.clone(),
                to: spec.to.clone(),
                width_nm: width,
                overrides_class: class.name,
                class_width_nm: class.track_width,
                connections: None,
                fanout_stubs: None,
            });
        }
        for (net, width) in nets {
            rules.set_net_track_width(net, width);
        }
        settings.connection_widths = connections;
        report.track_widths = reports;
        Ok(())
    }
}

impl AllowanceReport {
    /// Fill each track width entry's [`TrackWidthReport::connections`] and
    /// [`TrackWidthReport::fanout_stubs`] from a run: its task list, the
    /// connection entry each task was routed under
    /// ([`super::PipelineResult::connection_widths`], whose indices count
    /// the connection entries in order) and its plane fanout copper.
    pub fn count_track_width_connections(
        &mut self,
        board: &RouteBoard,
        connections: &[Connection],
        matched: &BTreeMap<ConnectionId, usize>,
        fanout: &crate::plan::RoutedCopper,
    ) {
        let mut entry = 0;
        for r in &mut self.track_widths {
            let count = if r.from.is_some() {
                let k = entry;
                entry += 1;
                matched.values().filter(|&&e| e == k).count()
            } else {
                let net = board.net_by_name(&r.net);
                r.fanout_stubs = Some(fanout.tracks.iter().filter(|t| Some(t.net) == net).count());
                connections
                    .iter()
                    .filter(|c| Some(c.net) == net && !matched.contains_key(&c.id))
                    .count()
            };
            r.connections = Some(count);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synthetic::BoardBuilder;

    fn class(name: &str, width: Coord) -> NetClassRules {
        NetClassRules {
            name: name.to_string(),
            clearance: 150_000,
            track_width: width,
            via_diameter: 450_000,
            via_drill: 200_000,
            diff_pair_width: None,
            diff_pair_gap: None,
        }
    }

    fn classes() -> Vec<NetClassRules> {
        vec![
            class("Default", 100_000),
            class("Fast", 150_000),
            class("Wide", 600_000),
            class("Pair", 100_000),
        ]
    }

    fn board() -> RouteBoard {
        BoardBuilder::new(10.0, 10.0, &["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]).build()
    }

    #[test]
    fn nothing_named_leaves_only_the_default_class_policy() {
        let mut s = PipelineSettings::default();
        let r = RoutingAllowances::default()
            .resolve(&board(), &classes(), &mut s)
            .unwrap();
        assert!(s.class_fallback.is_empty() && s.neck_down.is_none());
        assert!(s.layer_policy.class_layers.is_empty());
        assert_eq!(s.excluded_classes, BTreeSet::from([ClassId(2)]));
        assert_eq!(r.excluded_classes, vec!["Wide".to_string()]);
        assert!(r.exclusion_is_default);
        // An empty list routes every class.
        let all = RoutingAllowances {
            exclude_classes: Some(Vec::new()),
            ..RoutingAllowances::default()
        };
        all.resolve(&board(), &classes(), &mut s).unwrap();
        assert!(s.excluded_classes.is_empty());
    }

    #[test]
    fn fallback_reference_defaults_to_the_layers_beyond_the_class() {
        let b = board();
        let a: RoutingAllowances = serde_json::from_value(serde_json::json!({
            "class_layers": { "Fast": ["F.Cu"] },
            "layer_fallback": [ { "class": "Fast", "layers": ["F.Cu", "B.Cu"],
                                  "reference": { "plane": "In2.Cu", "margin_mm": 0.3 } } ],
            "neck_down": { "exclude_classes": ["Fast", "Pair"] },
            "exclude_classes": []
        }))
        .unwrap();
        let mut s = PipelineSettings::default();
        let r = a.resolve(&b, &classes(), &mut s).unwrap();
        let f = &s.class_fallback[&ClassId(1)];
        assert_eq!(
            f.layers,
            b.layers.expand("F.Cu").union(b.layers.expand("B.Cu"))
        );
        let reference = f.reference.unwrap();
        assert_eq!(reference.over, b.layers.expand("B.Cu"));
        assert_eq!(reference.margin, 300_000);
        assert_eq!(reference.plane, b.layers.by_name("In2.Cu").unwrap());
        assert_eq!(
            s.layer_policy.class_layers[&ClassId(1)],
            b.layers.expand("F.Cu")
        );
        let neck = s.neck_down.unwrap();
        assert_eq!((neck.width, neck.reach), (100_000, 1_000_000));
        assert_eq!(neck.excluded, BTreeSet::from([ClassId(1), ClassId(3)]));
        assert_eq!(
            r.neck_down_classes,
            vec!["Default".to_string(), "Wide".to_string()]
        );
    }

    #[test]
    fn an_allow_list_necks_only_those_classes() {
        let a = RoutingAllowances {
            neck_down: Some(NeckDownSpec {
                classes: Some(vec!["Wide".to_string()]),
                ..NeckDownSpec::default()
            }),
            ..RoutingAllowances::default()
        };
        let mut s = PipelineSettings::default();
        a.resolve(&board(), &classes(), &mut s).unwrap();
        assert_eq!(
            s.neck_down.unwrap().excluded,
            BTreeSet::from([ClassId(0), ClassId(1), ClassId(3)])
        );
    }

    #[test]
    fn unknown_names_are_refused() {
        let mut s = PipelineSettings::default();
        let bad_class = RoutingAllowances {
            exclude_classes: Some(vec!["Nope".to_string()]),
            ..RoutingAllowances::default()
        };
        assert_eq!(
            bad_class.resolve(&board(), &classes(), &mut s),
            Err(AllowanceError::UnknownClass("Nope".to_string()))
        );
        let bad_layer = RoutingAllowances {
            class_layers: BTreeMap::from([("Fast".to_string(), vec!["In9.Cu".to_string()])]),
            ..RoutingAllowances::default()
        };
        assert!(matches!(
            bad_layer.resolve(&board(), &classes(), &mut s),
            Err(AllowanceError::UnknownLayer(_))
        ));
        assert!(
            serde_json::from_value::<RoutingAllowances>(serde_json::json!({ "typo": 1 })).is_err()
        );
    }

    #[test]
    fn a_fallback_can_be_limited_to_some_nets_of_its_class() {
        let mut b = BoardBuilder::new(10.0, 10.0, &["F.Cu", "B.Cu"]);
        b.net("X1").net("X2").net("Y");
        let mut board = b.build();
        for n in &mut board.nets {
            n.class = if n.name == "X2" {
                ClassId(0)
            } else {
                ClassId(1)
            };
        }
        let spec = |nets: &[&str]| RoutingAllowances {
            layer_fallback: vec![LayerFallbackSpec {
                class: "Fast".to_string(),
                layers: vec!["*.Cu".to_string()],
                nets: Some(nets.iter().map(|n| n.to_string()).collect()),
                reference: None,
            }],
            ..RoutingAllowances::default()
        };
        let mut s = PipelineSettings::default();
        let r = spec(&["X*"]).resolve(&board, &classes(), &mut s).unwrap();
        let x1 = board.net_by_name("X1").unwrap();
        assert_eq!(
            s.class_fallback[&ClassId(1)].nets,
            Some(BTreeSet::from([x1]))
        );
        assert_eq!(r.layer_fallback["Fast"].nets, vec!["X1".to_string()]);
        // A pattern whose nets an earlier one already named still matches.
        spec(&["X1", "X*"])
            .resolve(&board, &classes(), &mut s)
            .unwrap();
        assert_eq!(
            s.class_fallback[&ClassId(1)].nets,
            Some(BTreeSet::from([x1]))
        );
        // X2 is not in the class: a pattern matching only it is refused.
        assert!(spec(&["X2"]).resolve(&board, &classes(), &mut s).is_err());
    }

    const WIDTH_PRO: &str = r#"{
        "board": { "design_settings": { "rules": { "min_track_width": 0.1 } } },
        "net_settings": {
            "classes": [
                { "name": "Default", "clearance": 0.15, "track_width": 0.2,
                  "via_diameter": 0.6, "via_drill": 0.3 },
                { "name": "Power", "clearance": 0.15, "track_width": 1.0 },
                { "name": "Capped", "clearance": 0.15, "track_width": 0.3 }
            ],
            "netclass_patterns": [
                { "pattern": "+5V", "netclass": "Power" },
                { "pattern": "CAP", "netclass": "Capped" }
            ]
        }
    }"#;
    const WIDTH_DRU: &str = r#"(version 1)
(rule "Capped width"
  (condition "A.NetClass == 'Capped'")
  (constraint track_width (min 0.15mm) (max 0.4mm)))
"#;

    /// `+5V` (class Power) on U2.3, R12.1 and R13.1, `SIG` on U2.4 and
    /// R12.2, `CAP` (class Capped) on R13.2.
    fn width_board() -> (RouteBoard, ProjectRuleTable) {
        let mut b = BoardBuilder::new(20.0, 10.0, &["F.Cu", "B.Cu"]);
        let u2 = b.footprint("U2", (3.0, 5.0));
        let r12 = b.footprint("R12", (10.0, 5.0));
        let r13 = b.footprint("R13", (16.0, 5.0));
        b.smd_pad(u2, "3", "+5V", (3.0, 4.0), (0.6, 0.4), "F.Cu");
        b.smd_pad(u2, "4", "SIG", (3.0, 6.0), (0.6, 0.4), "F.Cu");
        b.smd_pad(r12, "1", "+5V", (10.0, 4.0), (0.6, 0.4), "F.Cu");
        b.smd_pad(r12, "2", "SIG", (10.0, 6.0), (0.6, 0.4), "F.Cu");
        b.smd_pad(r13, "1", "+5V", (16.0, 4.0), (0.6, 0.4), "F.Cu");
        b.smd_pad(r13, "2", "CAP", (16.0, 6.0), (0.6, 0.4), "F.Cu");
        let mut board = b.build();
        let rules = ProjectRuleTable::from_project(WIDTH_PRO, Some(WIDTH_DRU), &board).unwrap();
        rules.assign_classes(&mut board);
        (board, rules)
    }

    fn widths(entries: serde_json::Value) -> RoutingAllowances {
        serde_json::from_value(serde_json::json!({
            "exclude_classes": [], "track_widths": entries
        }))
        .unwrap()
    }

    /// Resolve `a` on the width board: the report, the settings and rules.
    fn resolve_widths(
        a: &RoutingAllowances,
    ) -> Result<(AllowanceReport, PipelineSettings, ProjectRuleTable), AllowanceError> {
        let (board, mut rules) = width_board();
        let mut s = PipelineSettings::default();
        let mut report = a.resolve(&board, rules.classes(), &mut s)?;
        a.resolve_track_widths(&board, &mut rules, &mut s, &mut report)?;
        Ok((report, s, rules))
    }

    fn refused(entries: serde_json::Value) -> (usize, String) {
        match resolve_widths(&widths(entries)) {
            Err(AllowanceError::TrackWidth { index, reason }) => (index, reason),
            other => panic!(
                "expected a track width refusal, got {:?}",
                other.map(|r| r.0)
            ),
        }
    }

    #[test]
    fn a_net_width_and_a_connection_width_are_applied_and_reported() {
        let a = widths(serde_json::json!([
            { "net": "+5V", "width_mm": 0.3 },
            { "net": "+5V", "from": "U2.3", "to": "R12.1", "width_mm": 0.2 },
            { "net": "SIG", "width_mm": 0.5 }
        ]));
        let (report, s, rules) = resolve_widths(&a).unwrap();
        let (board, _) = width_board();
        let v5 = board.net_by_name("+5V").unwrap();
        // The net's width is the rule table's answer for every stage.
        let track = crate::rules::RuleSubject::new(crate::api::ItemKind::Track, Some(v5));
        assert_eq!(
            rules.track_width(&track, crate::api::LayerId(0)).preferred,
            300_000
        );
        // A width above the class width is allowed: the class is a default.
        assert_eq!(
            rules.net_track_width(board.net_by_name("SIG").unwrap()),
            Some(500_000)
        );
        assert_eq!(s.connection_widths.len(), 1);
        let c = &s.connection_widths[0];
        assert_eq!((c.net, c.width), (v5, 200_000));
        assert_eq!((c.from.len(), c.to.len()), (1, 1));
        assert_eq!(board.pads[c.from[0].index()].number, "3");
        // The report says which class each entry overrides, on purpose.
        assert_eq!(report.track_widths.len(), 3);
        let first = &report.track_widths[0];
        assert_eq!(
            (
                first.net.as_str(),
                first.width_nm,
                first.overrides_class.as_str()
            ),
            ("+5V", 300_000, "Power")
        );
        assert_eq!((first.class_width_nm, first.connections), (1_000_000, None));
        assert_eq!(first.fanout_stubs, None);
        assert_eq!(report.track_widths[1].from.as_deref(), Some("U2.3"));
        assert_eq!(report.track_widths[2].overrides_class, "Default");
    }

    #[test]
    fn a_width_outside_the_net_s_limits_is_refused_naming_the_entry() {
        let (index, reason) = refused(serde_json::json!([
            { "net": "SIG", "width_mm": 0.3 },
            { "net": "+5V", "width_mm": 0.05 }
        ]));
        assert_eq!(index, 1);
        assert!(reason.contains("0.05 mm is below the minimum"), "{reason}");
        assert!(reason.contains("0.1 mm"), "{reason}");
        // A custom rule's floor and cap bind too.
        let (_, reason) = refused(serde_json::json!([{ "net": "CAP", "width_mm": 0.12 }]));
        assert!(
            reason.contains("below the minimum") && reason.contains("0.15 mm"),
            "{reason}"
        );
        let (_, reason) = refused(serde_json::json!([{ "net": "CAP", "width_mm": 0.5 }]));
        assert!(
            reason.contains("above the maximum") && reason.contains("0.4 mm"),
            "{reason}"
        );
        for bad in [0.0, -0.2] {
            let (_, reason) = refused(serde_json::json!([{ "net": "SIG", "width_mm": bad }]));
            assert!(reason.contains("positive length"), "{reason}");
        }
        // A refusal changes nothing.
        let (board, mut rules) = width_board();
        let mut s = PipelineSettings::default();
        let a = widths(serde_json::json!([
            { "net": "SIG", "width_mm": 0.3 },
            { "net": "+5V", "from": "U2.3", "to": "R12.1", "width_mm": 0.2 },
            { "net": "+5V", "width_mm": 0.05 }
        ]));
        let mut report = a.resolve(&board, rules.classes(), &mut s).unwrap();
        assert!(a
            .resolve_track_widths(&board, &mut rules, &mut s, &mut report)
            .is_err());
        assert_eq!(
            rules.net_track_width(board.net_by_name("SIG").unwrap()),
            None
        );
        assert!(s.connection_widths.is_empty() && report.track_widths.is_empty());
    }

    #[test]
    fn unknown_nets_and_pads_are_refused_naming_the_entry() {
        let (index, reason) = refused(serde_json::json!([{ "net": "+9V", "width_mm": 0.3 }]));
        assert_eq!(index, 0);
        assert!(reason.contains("no net named '+9V'"), "{reason}");
        let pair = |from: &str, to: &str| {
            refused(serde_json::json!([
                { "net": "+5V", "width_mm": 0.3 },
                { "net": "+5V", "from": from, "to": to, "width_mm": 0.2 }
            ]))
        };
        let (index, reason) = pair("U9.3", "R12.1");
        assert_eq!(index, 1);
        assert!(reason.contains("'U9'"), "{reason}");
        let (_, reason) = pair("U2.7", "R12.1");
        assert!(reason.contains("footprint 'U2' has no pad '7'"), "{reason}");
        let (_, reason) = pair("U2.3", "R12");
        assert!(reason.contains("REFERENCE.PAD"), "{reason}");
        // A pad of another net is not a pad of this connection.
        let (_, reason) = pair("U2.4", "R12.1");
        assert!(
            reason.contains("pad U2.4 is on 'SIG', not on net '+5V'"),
            "{reason}"
        );
        let (_, reason) = pair("U2.3", "U2.3");
        assert!(reason.contains("both name pad U2.3"), "{reason}");
        let (_, reason) = refused(serde_json::json!([
            { "net": "+5V", "from": "U2.3", "width_mm": 0.2 }
        ]));
        assert!(reason.contains("give both pads"), "{reason}");
        // One width per net and per pad pair, in either order.
        let (index, reason) = refused(serde_json::json!([
            { "net": "SIG", "width_mm": 0.3 }, { "net": "SIG", "width_mm": 0.4 }
        ]));
        assert_eq!(index, 1);
        assert!(reason.contains("already has a width"), "{reason}");
        let (_, reason) = refused(serde_json::json!([
            { "net": "+5V", "from": "U2.3", "to": "R12.1", "width_mm": 0.2 },
            { "net": "+5V", "from": "R12.1", "to": "U2.3", "width_mm": 0.3 }
        ]));
        assert!(reason.contains("already has a width"), "{reason}");
        assert!(
            serde_json::from_value::<RoutingAllowances>(serde_json::json!({
                "track_widths": [{ "net": "SIG", "width": 0.3 }]
            }))
            .is_err()
        );
    }

    #[test]
    fn a_width_for_a_net_the_class_policy_leaves_unrouted_is_refused() {
        // By default the 1 mm Power class is not routed at all.
        let a: RoutingAllowances = serde_json::from_value(serde_json::json!({
            "track_widths": [{ "net": "+5V", "width_mm": 0.3 }]
        }))
        .unwrap();
        match resolve_widths(&a) {
            Err(AllowanceError::TrackWidth { index: 0, reason }) => {
                assert!(reason.contains("class 'Power'"), "{reason}");
                assert!(reason.contains("exclude_classes"), "{reason}");
            }
            other => panic!("{:?}", other.map(|r| r.0)),
        }
    }

    #[test]
    fn without_track_widths_nothing_is_set_or_serialized() {
        let (report, s, rules) = resolve_widths(&widths(serde_json::json!([]))).unwrap();
        assert!(s.connection_widths.is_empty() && report.track_widths.is_empty());
        let (board, plain) = width_board();
        for net in &board.nets {
            assert_eq!(rules.net_track_width(net.id), None);
            let t = crate::rules::RuleSubject::new(crate::api::ItemKind::Track, Some(net.id));
            let l = crate::api::LayerId(0);
            assert_eq!(rules.track_width(&t, l), plain.track_width(&t, l));
        }
        // The spec and the report keep the shape they had.
        let spec = serde_json::to_value(RoutingAllowances::default()).unwrap();
        assert_eq!(spec, serde_json::json!({}));
        assert!(serde_json::to_value(&report)
            .unwrap()
            .get("track_widths")
            .is_none());
    }

    #[test]
    fn a_reference_must_cover_only_layers_the_fallback_adds() {
        let resolve = |v: serde_json::Value| {
            let a: RoutingAllowances = serde_json::from_value(v).unwrap();
            a.resolve(&board(), &classes(), &mut PipelineSettings::default())
        };
        let fallback = |over: serde_json::Value| {
            serde_json::json!([ { "class": "Fast", "layers": ["F.Cu", "B.Cu"],
                                  "reference": { "plane": "In2.Cu", "over": over } } ])
        };
        // No layers of its own: the class routes everywhere already.
        let no_own = resolve(serde_json::json!({
            "layer_fallback": fallback(serde_json::json!(["B.Cu"]))
        }));
        assert!(
            matches!(no_own, Err(AllowanceError::Invalid(ref m)) if m.contains("class_layers")),
            "{no_own:?}"
        );
        // Over the class's own layer.
        let own = resolve(serde_json::json!({
            "class_layers": { "Fast": ["F.Cu"] },
            "layer_fallback": fallback(serde_json::json!(["F.Cu", "B.Cu"]))
        }));
        assert!(
            matches!(own, Err(AllowanceError::Invalid(ref m)) if m.contains("own layers")),
            "{own:?}"
        );
        // Over a layer the fallback does not add.
        let outside = resolve(serde_json::json!({
            "class_layers": { "Fast": ["F.Cu"] },
            "layer_fallback": fallback(serde_json::json!(["In1.Cu"]))
        }));
        assert!(
            matches!(outside, Err(AllowanceError::Invalid(ref m)) if m.contains("not all fallback")),
            "{outside:?}"
        );
        resolve(serde_json::json!({
            "class_layers": { "Fast": ["F.Cu"] },
            "layer_fallback": fallback(serde_json::json!(["B.Cu"]))
        }))
        .unwrap();
    }
}
