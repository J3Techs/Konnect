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
//!   excluded by policy, not as failures.
//!
//! Every allowance is off unless named. The one default is the class
//! policy: with `exclude_classes` unset, [`default_excluded_classes`]
//! leaves out wide-copper classes; an empty list routes every class.
//!
//! [`resolve`]: RoutingAllowances::resolve

use super::{ClassFallback, PipelineSettings, ReferencePlane};
use crate::api::{ClassId, LayerMask};
use crate::board::RouteBoard;
use crate::detail::negotiate::NeckDown;
use crate::rules::NetClassRules;
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
    /// The routing layers it applies to. Unset: the fallback's layers that
    /// are not among the class's own (`class_layers`); every fallback layer
    /// when the class has no `class_layers` entry.
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
                        let before = set.len();
                        for n in &board.nets {
                            let chars: Vec<char> = n.name.chars().collect();
                            if n.class == id && crate::rules::wildcard_match_chars(&pat, &chars) {
                                set.insert(n.id);
                            }
                        }
                        if set.len() == before {
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
                    let over = match &r.over {
                        Some(o) => mask(o)?,
                        None => LayerMask(layers.0 & !own.unwrap_or(LayerMask::EMPTY).0),
                    };
                    if over.is_empty() {
                        return Err(AllowanceError::Invalid(format!(
                            "class '{}': the reference applies to no layer",
                            f.class
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
        // X2 is not in the class: a pattern matching only it is refused.
        assert!(spec(&["X2"]).resolve(&board, &classes(), &mut s).is_err());
    }
}
