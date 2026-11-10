//! A library footprint's child zones, as KiCad's API takes them.
//!
//! A footprint can own rule areas: the keepout under a Tag-Connect header,
//! the antenna clearance of an RF module. 229 of the footprints KiCad 10.0.7
//! ships carry 475 of them, every one a single-outline rule area. KiCad's API
//! holds such a zone as one more item of the footprint's definition, in board
//! coordinates like the pads beside it, and `CreateItems` creates it with the
//! footprint (both measured against a live 10.0.7 editor). A live placement
//! built from pads, drawings and fields alone therefore placed the footprint
//! without its keepouts, and nothing said so.
//!
//! This module reads those zones from the library source and builds the typed
//! message for a footprint at a given position and angle. Built for 24
//! footprints pcbnew 10.0.7 had placed itself (arcs, `*.Cu`, inner and
//! technical layers, at 0, 30 and -90 degrees), the 105 messages equal the
//! ones KiCad holds; `live_tests` repeats that against any open board.
//!
//! What the message cannot say is refused, never dropped or approximated:
//!
//! * a copper zone (no `(keepout …)`), which needs a net and a fill;
//! * a rule area that places components (`(placement (enabled yes) …)`);
//! * an outline of more or fewer than one `(polygon …)`;
//! * any clause this reader does not know;
//! * copper-pour settings other than the ones KiCad gives a zone created
//!   through its API.
//!
//! The last is KiCad's limit, not a choice. A rule area's typed message
//! (`RuleAreaSettings`) has no field for `connect_pads`, `min_thickness` or
//! `fill`, so a zone created through the API is saved with KiCad's own
//! `(connect_pads (clearance 0))`, `(min_thickness 0.25)`, `(thermal_gap 0.5)`
//! and `(thermal_bridge_width 0.5)`. They do nothing on a rule area, but
//! KiCad's library-parity check compares them all the same: changing any one
//! of the three numbers in a footprint pcbnew had placed made `kicad-cli pcb
//! drc` report `lib_footprint_mismatch` for it (10.0.7). 196 of the 475 stock
//! zones were drawn in imperial units (`0.254`, `0.508`); placed live they
//! would no longer match their library, so they are refused and the
//! closed-board placement, which copies the footprint's text, is named.

use std::collections::BTreeSet;
use std::fmt;

use konnect_ipc::gen::kiapi;
use konnect_sexp::SexpNode;

const ZONE_TYPE: &str = "kiapi.board.types.Zone";
const FOOTPRINT_TYPE: &str = "kiapi.board.types.FootprintInstance";

/// One vertex of a zone outline, in the footprint's frame (mm).
#[derive(Debug, Clone, Copy, PartialEq)]
enum Vertex {
    Point(f64, f64),
    Arc {
        start: (f64, f64),
        mid: (f64, f64),
        end: (f64, f64),
    },
}

/// One rule area of a library footprint.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct LibraryZone {
    name: String,
    layers: BTreeSet<i32>,
    outline: Vec<Vertex>,
    border: kiapi::board::types::ZoneBorderStyle,
    border_pitch_mm: f64,
    priority: u32,
    locked: bool,
    keepout_tracks: bool,
    keepout_vias: bool,
    keepout_pads: bool,
    keepout_copper: bool,
    keepout_footprints: bool,
    /// The copper-pour settings that differ from what KiCad gives a zone
    /// created through its API, as `min_thickness 0.254`.
    pour_differences: Vec<String>,
}

/// One zone a live placement cannot carry, and why.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct UnplaceableZone {
    /// `zone 2 'Antenna keepout'`: its place among the footprint's zones and
    /// its name, when it has one.
    pub(crate) zone: String,
    pub(crate) reason: String,
}

/// The zones of one library footprint that KiCad's API cannot be given.
/// Carried through `anyhow` so a handler can answer with a structured refusal.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct UnplaceableZones {
    pub(crate) footprint: String,
    pub(crate) zones: Vec<UnplaceableZone>,
}

impl fmt::Display for UnplaceableZones {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "footprint '{}' carries {} that a live placement cannot create as the library \
             defines it: {}. Placing it on the closed board with place_component \
             (run_closed_board_edit can run that on an open board) copies the library footprint \
             whole",
            self.footprint,
            if self.zones.len() == 1 {
                "a zone".to_string()
            } else {
                format!("{} zones", self.zones.len())
            },
            self.zones
                .iter()
                .map(|zone| format!("{} ({})", zone.zone, zone.reason))
                .collect::<Vec<_>>()
                .join("; ")
        )
    }
}

impl std::error::Error for UnplaceableZones {}

impl UnplaceableZones {
    /// The refusal for a tool that also has a closed-board placement path.
    pub(crate) fn into_result(
        self,
        operation: &str,
        remedy: Remedy,
    ) -> crate::mcp::protocol::CallToolResult {
        let mut reasons = self
            .zones
            .iter()
            .map(|zone| {
                format!(
                    "{} {}: {}. A live placement cannot create it as the library defines it.",
                    self.footprint, zone.zone, zone.reason
                )
            })
            .collect::<Vec<_>>();
        reasons.push(
            match remedy {
                Remedy::ClosedBoardPlacement => CLOSED_BOARD_REMEDY,
                Remedy::NoLosslessPath => NO_LOSSLESS_REMEDY,
            }
            .to_string(),
        );
        crate::mcp::protocol::CallToolResult::error_kind(
            crate::mcp::error::ToolErrorKind::PlanBlocked {
                operation: operation.to_string(),
                reasons: reasons.clone(),
            },
            format!(
                "Refused {operation} before any change: {} The board was not modified.",
                reasons.join(" ")
            ),
        )
    }
}

/// What a refused caller can do instead.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Remedy {
    /// `place_component` and `add_mounting_hole` copy the library footprint's
    /// text into a closed board, zones included.
    ClosedBoardPlacement,
    /// The tool only works through KiCad's API.
    NoLosslessPath,
}

const CLOSED_BOARD_REMEDY: &str = "KiCad 10's API creates a footprint's zone from a typed \
     message that describes a single-outline rule area and none of its copper-pour settings. \
     Save the board, close it in KiCad and repeat the request (run_closed_board_edit does all \
     three on an open board): with the board closed this tool copies the library footprint \
     whole, zones included.";

const NO_LOSSLESS_REMEDY: &str = "KiCad 10's API creates a footprint's zone from a typed \
     message that describes a single-outline rule area and none of its copper-pour settings, \
     and this tool has no closed-board path. Place the footprint with place_component on the \
     closed board (run_closed_board_edit can run it on an open one), which copies the library \
     footprint whole, or place it in KiCad itself.";

/// The answer for a placement that failed to prepare: the structured zone
/// refusal when that is why, the error's own text otherwise.
pub(crate) fn preparation_error_result(
    error: &anyhow::Error,
    operation: &str,
    remedy: Remedy,
) -> crate::mcp::protocol::CallToolResult {
    match error.downcast_ref::<UnplaceableZones>() {
        Some(zones) => zones.clone().into_result(operation, remedy),
        None => crate::mcp::protocol::CallToolResult::error(format!("{error:#}")),
    }
}

/// Every direct child `(zone …)` of a library footprint, each one a rule area
/// a live placement creates exactly as the library defines it.
pub(crate) fn read(root: &SexpNode) -> Result<Vec<LibraryZone>, UnplaceableZones> {
    read_with(root, true)
}

/// [`read`], optionally without holding the copper-pour settings to KiCad's
/// API defaults: the geometry, layers and rules alone, for comparing with
/// zones KiCad placed itself.
fn read_with(root: &SexpNode, exact_pour: bool) -> Result<Vec<LibraryZone>, UnplaceableZones> {
    let mut zones = Vec::new();
    let mut refused = Vec::new();
    let children = root.children().unwrap_or_default();
    for (index, node) in children
        .iter()
        .filter(|child| child.head() == Some("zone"))
        .enumerate()
    {
        let read = read_zone(node).and_then(|zone| {
            if exact_pour && !zone.pour_differences.is_empty() {
                Err(format!(
                    "its copper-pour settings ({}) are not the ones KiCad gives a zone created \
                     through its API ({API_POUR_DEFAULTS}), whose typed rule area has no field \
                     for them; KiCad would report the placed footprint as not matching its \
                     library",
                    zone.pour_differences.join(", ")
                ))
            } else {
                Ok(zone)
            }
        });
        match read {
            Ok(zone) => zones.push(zone),
            Err(reason) => {
                let name = node.find_str("name").unwrap_or_default();
                refused.push(UnplaceableZone {
                    zone: if name.is_empty() {
                        format!("zone {}", index + 1)
                    } else {
                        format!("zone {} '{name}'", index + 1)
                    },
                    reason,
                });
            }
        }
    }
    if refused.is_empty() {
        Ok(zones)
    } else {
        Err(UnplaceableZones {
            footprint: root
                .get(1)
                .and_then(SexpNode::as_str)
                .unwrap_or_default()
                .to_string(),
            zones: refused,
        })
    }
}

/// What pcbnew 10.0.7 saves for a rule area created through its API.
const API_POUR_DEFAULTS: &str =
    "connect_pads clearance 0, min_thickness 0.25, thermal_gap 0.5, thermal_bridge_width 0.5, \
     island_removal_mode 0";

/// The settings of `clause` that differ from `defaults`, each as `name value`.
/// A setting the defaults do not list is a difference too.
fn pour_differences(
    clause: &SexpNode,
    defaults: &[(&str, f64)],
    differences: &mut Vec<String>,
) -> Result<(), String> {
    for setting in clause.children().unwrap_or_default().iter().skip(1) {
        let Some(name) = setting.head() else {
            // `(connect_pads yes …)`, `(fill yes …)`: a pour's mode.
            differences.push(format!(
                "{} {}",
                clause.head().unwrap_or_default(),
                setting.as_str().unwrap_or_default()
            ));
            continue;
        };
        let value = setting
            .get_f64(1)
            .filter(|_| setting.children().map_or(0, <[_]>::len) == 2);
        match (defaults.iter().find(|(known, _)| *known == name), value) {
            (Some((_, default)), Some(value)) if (value - default).abs() < 1e-9 => {}
            (Some(_), Some(value)) => differences.push(format!("{name} {value}")),
            (Some(_), None) => return Err(format!("its ({name} …) is not one number")),
            (None, _) => differences.push(format!("a ({name} …) setting")),
        }
    }
    Ok(())
}

fn read_zone(zone: &SexpNode) -> Result<LibraryZone, String> {
    use kiapi::board::types::ZoneBorderStyle;

    let mut name = String::new();
    let mut layers = BTreeSet::new();
    let mut outline = None;
    let mut border = (ZoneBorderStyle::ZbsDiagonalEdge, 0.5);
    let mut priority = 0;
    let mut locked = false;
    let mut keepout = None;
    let mut differences = Vec::new();
    for clause in zone.children().unwrap_or_default().iter().skip(1) {
        let Some(head) = clause.head() else {
            return Err("it holds a bare atom this reader does not know".to_string());
        };
        let atom = |index: usize| clause.get(index).and_then(SexpNode::as_str);
        match head {
            // A library zone belongs to no net; KiCad writes `(net 0)`.
            "net" | "net_name" | "uuid" => {}
            // Copper-pour settings the typed rule area cannot carry (see the
            // module doc): noted, and held to KiCad's API defaults by `read`.
            "connect_pads" => pour_differences(clause, &[("clearance", 0.0)], &mut differences)?,
            "min_thickness" => match clause.get_f64(1) {
                Some(value) if (value - 0.25).abs() < 1e-9 => {}
                Some(value) => differences.push(format!("min_thickness {value}")),
                None => return Err("its (min_thickness …) is not a number".to_string()),
            },
            "fill" => pour_differences(
                clause,
                &[
                    ("thermal_gap", 0.5),
                    ("thermal_bridge_width", 0.5),
                    ("island_removal_mode", 0.0),
                ],
                &mut differences,
            )?,
            // Written by KiCad 5 and read past by KiCad 10's own parser.
            "filled_areas_thickness" => {}
            "name" => name = atom(1).unwrap_or_default().to_string(),
            "layer" | "layers" => {
                for layer in clause.children().unwrap_or_default().iter().skip(1) {
                    let layer = layer
                        .as_str()
                        .ok_or_else(|| "its layer list holds a non-atom".to_string())?;
                    layers.extend(zone_layers(layer)?);
                }
            }
            "hatch" => {
                let style = match atom(1) {
                    Some("edge") => ZoneBorderStyle::ZbsDiagonalEdge,
                    Some("full") => ZoneBorderStyle::ZbsDiagonalFull,
                    Some("none") => ZoneBorderStyle::ZbsSolid,
                    other => {
                        return Err(format!(
                            "its border hatch '{}' is not one KiCad's API names",
                            other.unwrap_or_default()
                        ))
                    }
                };
                let pitch = clause
                    .get_f64(2)
                    .filter(|pitch| pitch.is_finite() && *pitch >= 0.0)
                    .ok_or_else(|| "its border hatch has no readable pitch".to_string())?;
                border = (style, pitch);
            }
            "priority" => {
                priority = atom(1)
                    .and_then(|value| value.parse::<u32>().ok())
                    .ok_or_else(|| "its priority is not a whole number".to_string())?;
            }
            "locked" => {
                locked = match atom(1) {
                    Some("yes") => true,
                    Some("no") => false,
                    _ => return Err("its (locked …) is neither yes nor no".to_string()),
                }
            }
            "keepout" => keepout = Some(read_keepout(clause)?),
            "placement" => {
                if clause.find_str("enabled") != Some("no") {
                    return Err(
                        "it is a placement rule area, which this reader does not carry".to_string(),
                    );
                }
            }
            "polygon" => {
                if outline.is_some() {
                    return Err("its outline has more than one (polygon …)".to_string());
                }
                outline = Some(read_outline(clause)?);
            }
            other => {
                return Err(format!(
                    "it holds a ({other} …) clause KiCad's typed rule area cannot carry"
                ))
            }
        }
    }
    let Some((keepout_tracks, keepout_vias, keepout_pads, keepout_copper, keepout_footprints)) =
        keepout
    else {
        return Err(
            "it is a copper zone, not a rule area: it has no (keepout …) and would need a net \
             and a fill"
                .to_string(),
        );
    };
    let outline = outline.ok_or_else(|| "it has no (polygon …) outline".to_string())?;
    if layers.is_empty() {
        return Err("it names no layer".to_string());
    }
    Ok(LibraryZone {
        name,
        layers,
        outline,
        border: border.0,
        border_pitch_mm: border.1,
        priority,
        locked,
        keepout_tracks,
        keepout_vias,
        keepout_pads,
        keepout_copper,
        keepout_footprints,
        pour_differences: differences,
    })
}

/// The board layers one zone layer name stands for. `*.Cu` is every copper
/// layer and `F&B.Cu` the two outer ones, as KiCad's parser expands them; the
/// layers the board does not have are dropped where the board is known
/// ([`konnect_ipc::KiCadIpcClient::fit_footprint_zone_layers_in`]).
fn zone_layers(name: &str) -> Result<Vec<i32>, String> {
    use kiapi::board::types::BoardLayer;
    match name {
        "*.Cu" => Ok((BoardLayer::BlFCu as i32..=BoardLayer::BlBCu as i32).collect()),
        "F&B.Cu" => Ok(vec![BoardLayer::BlFCu as i32, BoardLayer::BlBCu as i32]),
        name if name.contains('*') || name.contains('&') => Err(format!(
            "its layer wildcard '{name}' is not one this reader expands"
        )),
        name => konnect_ipc::builders::try_layer_from_name(name)
            .map(|layer| vec![layer as i32])
            .map_err(|error| format!("{error:#}")),
    }
}

fn read_keepout(keepout: &SexpNode) -> Result<(bool, bool, bool, bool, bool), String> {
    let mut seen = [None; 5];
    for rule in keepout.children().unwrap_or_default().iter().skip(1) {
        let slot = match rule.head() {
            Some("tracks") => 0,
            Some("vias") => 1,
            Some("pads") => 2,
            Some("copperpour") => 3,
            Some("footprints") => 4,
            other => {
                return Err(format!(
                    "its keepout holds a rule '{}' this reader does not know",
                    other.unwrap_or_default()
                ))
            }
        };
        seen[slot] = Some(match rule.get(1).and_then(SexpNode::as_str) {
            Some("not_allowed") => true,
            Some("allowed") => false,
            _ => return Err("a keepout rule is neither allowed nor not_allowed".to_string()),
        });
    }
    match seen {
        [Some(tracks), Some(vias), Some(pads), Some(copper), Some(footprints)] => {
            Ok((tracks, vias, pads, copper, footprints))
        }
        _ => Err("its keepout does not state all five rules".to_string()),
    }
}

fn read_outline(polygon: &SexpNode) -> Result<Vec<Vertex>, String> {
    let point = |node: &SexpNode| -> Result<(f64, f64), String> {
        match (node.get_f64(1), node.get_f64(2)) {
            (Some(x), Some(y))
                if x.is_finite() && y.is_finite() && node.children().map_or(0, <[_]>::len) == 3 =>
            {
                Ok((x, y))
            }
            _ => Err("its outline holds an unreadable point".to_string()),
        }
    };
    let mut pts = None;
    for part in polygon.children().unwrap_or_default().iter().skip(1) {
        if part.head() != Some("pts") || pts.is_some() {
            return Err("its (polygon …) holds more than one (pts …) list".to_string());
        }
        pts = Some(part);
    }
    let pts = pts.ok_or_else(|| "its (polygon …) has no (pts …) list".to_string())?;
    let mut outline = Vec::new();
    for entry in pts.children().unwrap_or_default().iter().skip(1) {
        match entry.head() {
            Some("xy") => {
                let (x, y) = point(entry)?;
                outline.push(Vertex::Point(x, y));
            }
            Some("arc") => {
                let part = |tag: &str| {
                    entry
                        .find(tag)
                        .ok_or_else(|| format!("an outline arc has no ({tag} …)"))
                        .and_then(point)
                };
                if entry.children().map_or(0, <[_]>::len) != 4 {
                    return Err("an outline arc holds an unknown clause".to_string());
                }
                outline.push(Vertex::Arc {
                    start: part("start")?,
                    mid: part("mid")?,
                    end: part("end")?,
                });
            }
            _ => {
                return Err("its outline holds an entry that is neither a point nor an arc".into())
            }
        }
    }
    if outline.len() < 3 && !outline.iter().any(|v| matches!(v, Vertex::Arc { .. })) {
        return Err("its outline has fewer than three points".to_string());
    }
    Ok(outline)
}

impl LibraryZone {
    /// The zone of a front-side footprint at `(x, y)` turned by `rotation`
    /// degrees, in board coordinates as KiCad's API holds a footprint's
    /// children.
    fn to_proto(&self, x: f64, y: f64, rotation: f64) -> kiapi::board::types::Zone {
        use kiapi::common::types::{
            poly_line_node::Geometry, ArcStartMidEnd, PolyLine, PolyLineNode, PolySet,
            PolygonWithHoles,
        };
        let place = |(px, py): (f64, f64)| {
            let (bx, by) = konnect_sexp::geometry::transform_pad(px, py, x, y, rotation);
            konnect_ipc::builders::vec2(bx, by)
        };
        let nodes = self
            .outline
            .iter()
            .map(|vertex| PolyLineNode {
                geometry: Some(match *vertex {
                    Vertex::Point(px, py) => Geometry::Point(place((px, py))),
                    Vertex::Arc { start, mid, end } => Geometry::Arc(ArcStartMidEnd {
                        start: Some(place(start)),
                        mid: Some(place(mid)),
                        end: Some(place(end)),
                    }),
                }),
            })
            .collect();
        kiapi::board::types::Zone {
            id: None,
            r#type: kiapi::board::types::ZoneType::ZtRuleArea as i32,
            layers: self.layers.iter().copied().collect(),
            outline: Some(PolySet {
                polygons: vec![PolygonWithHoles {
                    outline: Some(PolyLine {
                        nodes,
                        closed: true,
                    }),
                    holes: Vec::new(),
                }],
            }),
            name: self.name.clone(),
            priority: self.priority,
            filled: false,
            filled_polygons: Vec::new(),
            border: Some(kiapi::board::types::ZoneBorderSettings {
                style: self.border as i32,
                pitch: Some(konnect_ipc::builders::distance(self.border_pitch_mm)),
            }),
            locked: if self.locked {
                kiapi::common::types::LockedState::LsLocked
            } else {
                kiapi::common::types::LockedState::LsUnlocked
            } as i32,
            layer_properties: Vec::new(),
            parent: None,
            settings: Some(kiapi::board::types::zone::Settings::RuleAreaSettings(
                kiapi::board::types::RuleAreaSettings {
                    keepout_copper: self.keepout_copper,
                    keepout_vias: self.keepout_vias,
                    keepout_tracks: self.keepout_tracks,
                    keepout_pads: self.keepout_pads,
                    keepout_footprints: self.keepout_footprints,
                    placement_enabled: false,
                    placement_source_type:
                        kiapi::board::types::PlacementRuleSourceType::PrstSheetName as i32,
                    placement_source: String::new(),
                },
            )),
        }
    }
}

/// Give a footprint built from pads, drawings and fields its library's zones,
/// placed where the footprint itself says it is. Only a front-side footprint:
/// putting a zone on the back means mirroring its outline and moving every
/// layer it names, which nothing here has been measured against KiCad for.
pub(crate) fn apply(
    zones: &[LibraryZone],
    footprint: &mut kiapi::board::types::FootprintInstance,
) -> anyhow::Result<()> {
    use anyhow::Context;
    if zones.is_empty() {
        return Ok(());
    }
    anyhow::ensure!(
        footprint.layer == kiapi::board::types::BoardLayer::BlFCu as i32,
        "the footprint's library carries {} child zone(s), which a typed placement can only \
         create for a footprint on the front side",
        zones.len()
    );
    let position = footprint
        .position
        .as_ref()
        .context("built footprint has no position")?;
    let (x, y) = (
        konnect_ipc::builders::nm_to_mm(position.x_nm),
        konnect_ipc::builders::nm_to_mm(position.y_nm),
    );
    let rotation = footprint
        .orientation
        .as_ref()
        .map_or(0.0, |angle| angle.value_degrees);
    let definition = footprint
        .definition
        .as_mut()
        .context("built footprint has no definition")?;
    definition.items.extend(
        zones
            .iter()
            .map(|zone| konnect_ipc::builders::pack_any(&zone.to_proto(x, y, rotation), ZONE_TYPE)),
    );
    Ok(())
}

/// [`apply`] for a footprint already packed for `CreateItems`.
pub(crate) fn apply_to_item(
    zones: &[LibraryZone],
    item: prost_types::Any,
) -> anyhow::Result<prost_types::Any> {
    use anyhow::Context;
    use prost::Message;
    if zones.is_empty() {
        return Ok(item);
    }
    let mut footprint = kiapi::board::types::FootprintInstance::decode(item.value.as_slice())
        .context("typed footprint builder returned an invalid item")?;
    apply(zones, &mut footprint)?;
    Ok(konnect_ipc::builders::pack_any(&footprint, FOOTPRINT_TYPE))
}

#[cfg(test)]
mod tests {
    use super::*;
    use prost::Message;

    const TAG_CONNECT_ZONE: &str = r#"(footprint "Connector:Tag-Connect"
	(layer "F.Cu")
	(zone_connect 1)
	(zone
		(net 0)
		(net_name "")
		(layer "F.Cu")
		(uuid "55b6b040-a746-4424-a5b4-1f45a1d15120")
		(hatch full 0.508)
		(connect_pads
			(clearance 0)
		)
		(min_thickness 0.25)
		(keepout
			(tracks allowed)
			(vias not_allowed)
			(pads allowed)
			(copperpour not_allowed)
			(footprints not_allowed)
		)
		(placement
			(enabled no)
			(sheetname "")
		)
		(fill
			(thermal_gap 0.5)
			(thermal_bridge_width 0.5)
			(island_removal_mode 0)
		)
		(polygon
			(pts
				(xy 1.27 -0.635) (xy -1.27 -0.635) (xy -1.27 0.635) (xy 1.27 0.635)
			)
		)
	)
)"#;

    fn zones(source: &str) -> Result<Vec<LibraryZone>, UnplaceableZones> {
        read(&konnect_sexp::parse_sexp(source).unwrap())
    }

    fn outline_nm(zone: &kiapi::board::types::Zone) -> Vec<(i64, i64)> {
        use kiapi::common::types::poly_line_node::Geometry;
        zone.outline.as_ref().unwrap().polygons[0]
            .outline
            .as_ref()
            .unwrap()
            .nodes
            .iter()
            .map(|node| match node.geometry.as_ref().unwrap() {
                Geometry::Point(point) => (point.x_nm, point.y_nm),
                Geometry::Arc(_) => panic!("an arc where a point was expected"),
            })
            .collect()
    }

    /// The outlines are the ones pcbnew 10.0.7 saves for this footprint
    /// placed at (50, 50) and at (80, 60) turned 90 degrees (see
    /// `pcb_components::child_zones`).
    #[test]
    fn a_rule_area_is_built_in_board_coordinates_with_its_keepout_rules() {
        let zones = zones(TAG_CONNECT_ZONE).unwrap();
        assert_eq!(zones.len(), 1, "(zone_connect …) is not a zone");

        let placed = zones[0].to_proto(50.0, 50.0, 0.0);
        assert_eq!(
            outline_nm(&placed),
            [
                (51_270_000, 49_365_000),
                (48_730_000, 49_365_000),
                (48_730_000, 50_635_000),
                (51_270_000, 50_635_000)
            ]
        );
        assert_eq!(
            placed.r#type,
            kiapi::board::types::ZoneType::ZtRuleArea as i32
        );
        assert_eq!(
            placed.layers,
            [kiapi::board::types::BoardLayer::BlFCu as i32]
        );
        let border = placed.border.as_ref().unwrap();
        assert_eq!(
            border.style,
            kiapi::board::types::ZoneBorderStyle::ZbsDiagonalFull as i32
        );
        assert_eq!(border.pitch.as_ref().unwrap().value_nm, 508_000);
        let Some(kiapi::board::types::zone::Settings::RuleAreaSettings(rules)) = placed.settings
        else {
            panic!("a rule area must carry RuleAreaSettings");
        };
        assert!(rules.keepout_vias && rules.keepout_copper && rules.keepout_footprints);
        assert!(!rules.keepout_tracks && !rules.keepout_pads && !rules.placement_enabled);

        let turned = zones[0].to_proto(80.0, 60.0, 90.0);
        assert_eq!(
            outline_nm(&turned)[..2],
            [(79_365_000, 58_730_000), (79_365_000, 61_270_000)]
        );
    }

    #[test]
    fn an_arc_in_the_outline_keeps_its_three_points_and_wildcard_layers_expand() {
        let source = TAG_CONNECT_ZONE
            .replace(
                "(layer \"F.Cu\")\n\t\t(uuid",
                "(layers \"*.Cu\" \"F.Paste\")\n\t\t(uuid",
            )
            .replace(
                "(xy -1.27 0.635) (xy 1.27 0.635)",
                "(arc (start -1.27 0.635) (mid 0 1) (end 1.27 0.635))",
            )
            .replace("(hatch full 0.508)", "(name \"Antenna\") (hatch edge 0.5)");
        let zone = &zones(&source).unwrap()[0];
        let placed = zone.to_proto(10.0, 20.0, 0.0);
        assert_eq!(placed.name, "Antenna");
        // 32 copper layers and the paste layer.
        assert_eq!(placed.layers.len(), 33);
        let nodes = &placed.outline.as_ref().unwrap().polygons[0]
            .outline
            .as_ref()
            .unwrap()
            .nodes;
        assert_eq!(nodes.len(), 3);
        let Some(kiapi::common::types::poly_line_node::Geometry::Arc(arc)) = &nodes[2].geometry
        else {
            panic!("the third node is the arc");
        };
        assert_eq!(arc.mid.as_ref().unwrap().x_nm, 10_000_000);
        assert_eq!(arc.mid.as_ref().unwrap().y_nm, 21_000_000);
    }

    #[test]
    fn a_zone_the_typed_message_cannot_describe_is_refused_by_name() {
        let copper = TAG_CONNECT_ZONE.replace(
            "(keepout\n\t\t\t(tracks allowed)\n\t\t\t(vias not_allowed)\n\t\t\t(pads allowed)\n\t\t\t(copperpour not_allowed)\n\t\t\t(footprints not_allowed)\n\t\t)",
            "(name \"Shield\")",
        );
        let refused = zones(&copper).unwrap_err();
        assert_eq!(refused.footprint, "Connector:Tag-Connect");
        assert_eq!(refused.zones[0].zone, "zone 1 'Shield'");
        assert!(refused.zones[0].reason.contains("copper zone"), "{refused}");

        for (from, to, expected) in [
            ("(enabled no)", "(enabled yes)", "placement rule area"),
            (
                "(min_thickness 0.25)",
                "(min_thickness 0.25) (attr (teardrop (type padvia)))",
                "(attr …)",
            ),
            (
                "(polygon\n",
                "(polygon (pts (xy 0 0) (xy 1 0) (xy 1 1)))\n\t\t(polygon\n",
                "more than one (polygon …)",
            ),
            (
                "(layer \"F.Cu\")\n\t\t(uuid",
                "(layers \"*.Mask\")\n\t\t(uuid",
                "'*.Mask'",
            ),
            ("(vias not_allowed)", "", "all five rules"),
        ] {
            assert!(TAG_CONNECT_ZONE.contains(from), "{from}");
            let refused = zones(&TAG_CONNECT_ZONE.replace(from, to)).unwrap_err();
            assert!(
                refused.zones[0].reason.contains(expected),
                "{expected}: {refused}"
            );
        }
    }

    /// The stock Tag-Connect keepout is drawn in imperial units. Created
    /// through the API it is saved with KiCad's metric defaults, and KiCad's
    /// DRC then reports the footprint as not matching its library.
    #[test]
    fn pour_settings_the_api_cannot_set_refuse_the_zone_and_say_which() {
        let imperial = TAG_CONNECT_ZONE
            .replace("(min_thickness 0.25)", "(min_thickness 0.254)")
            .replace("(thermal_gap 0.5)", "(thermal_gap 0.508)")
            .replace("(thermal_bridge_width 0.5)", "(thermal_bridge_width 0.508)");
        let refused = zones(&imperial).unwrap_err();
        let reason = &refused.zones[0].reason;
        assert!(
            reason.contains("(min_thickness 0.254, thermal_gap 0.508, thermal_bridge_width 0.508)"),
            "{reason}"
        );
        assert!(reason.contains("not matching its library"), "{reason}");
        // The geometry itself reads: only the uncarried settings refuse it.
        let root = konnect_sexp::parse_sexp(&imperial).unwrap();
        assert_eq!(read_with(&root, false).unwrap().len(), 1);

        for (from, to, expected) in [
            ("(clearance 0)", "(clearance 0.2)", "clearance 0.2"),
            ("(connect_pads\n", "(connect_pads yes\n", "connect_pads yes"),
            (
                "(island_removal_mode 0)",
                "(island_removal_mode 1)",
                "island_removal_mode 1",
            ),
            (
                "(island_removal_mode 0)",
                "(smoothing chamfer)",
                "a (smoothing …) setting",
            ),
        ] {
            assert!(TAG_CONNECT_ZONE.contains(from), "{from}");
            let refused = zones(&TAG_CONNECT_ZONE.replace(from, to)).unwrap_err();
            assert!(
                refused.zones[0].reason.contains(expected),
                "{expected}: {refused}"
            );
        }
        // An absent setting is KiCad's default, and KiCad 5's flag is inert.
        let sparse = TAG_CONNECT_ZONE
            .replace("(island_removal_mode 0)", "")
            .replace("(min_thickness 0.25)", "(filled_areas_thickness no)");
        assert_eq!(zones(&sparse).unwrap().len(), 1);
    }

    #[test]
    fn the_refusal_names_the_footprint_the_zone_and_the_closed_board_route() {
        let refused = zones(&TAG_CONNECT_ZONE.replace("(enabled no)", "(enabled yes)"))
            .unwrap_err()
            .into_result("place_component", Remedy::ClosedBoardPlacement);
        assert!(refused.is_error);
        let body = serde_json::to_value(&refused).unwrap().to_string();
        assert!(body.contains("plan_blocked"), "{body}");
        assert!(body.contains("Connector:Tag-Connect zone 1"), "{body}");
        assert!(body.contains("run_closed_board_edit"), "{body}");
    }

    #[test]
    fn zones_join_a_front_footprint_where_it_stands_and_are_refused_on_the_back() {
        let zones = zones(TAG_CONNECT_ZONE).unwrap();
        let item = konnect_ipc::KiCadIpcClient::build_footprint_item(
            "Connector:Tag-Connect",
            "J1",
            "v",
            &[],
            &[],
            &Default::default(),
            80.0,
            60.0,
            90.0,
            "F.Cu",
        )
        .unwrap();
        let placed = apply_to_item(&zones, item.clone()).unwrap();
        let footprint =
            kiapi::board::types::FootprintInstance::decode(placed.value.as_slice()).unwrap();
        let children = footprint.definition.unwrap().items;
        assert_eq!(children.len(), 1);
        let zone = kiapi::board::types::Zone::decode(children[0].value.as_slice()).unwrap();
        assert_eq!(outline_nm(&zone)[0], (79_365_000, 58_730_000));

        let mut back =
            kiapi::board::types::FootprintInstance::decode(item.value.as_slice()).unwrap();
        back.layer = kiapi::board::types::BoardLayer::BlBCu as i32;
        let error = apply(&zones, &mut back).unwrap_err();
        assert!(format!("{error:#}").contains("front side"), "{error:#}");
        // A footprint without zones is untouched on either side.
        assert!(apply(&[], &mut back).is_ok());
    }

    /// Every zone KiCad ships must read: a stock footprint is only ever
    /// refused for pour settings the API cannot set. Skipped where no KiCad
    /// library is installed.
    #[test]
    fn every_zone_in_the_installed_kicad_libraries_reads() {
        let Some(root) = super::super::tests::installed_footprint_dir() else {
            return;
        };
        let (mut footprints, mut read_zones, mut placeable) = (0, 0, 0);
        for library in std::fs::read_dir(root).unwrap().flatten() {
            let Ok(entries) = std::fs::read_dir(library.path()) else {
                continue;
            };
            for entry in entries.flatten() {
                let Ok(source) = std::fs::read_to_string(entry.path()) else {
                    continue;
                };
                // `(zone_connect …)` is in thousands of footprints; a zone
                // opens its own line.
                if !source.contains("(zone\n") && !source.contains("(zone\r") {
                    continue;
                }
                let root = konnect_sexp::parse_sexp(&source).unwrap();
                let found = read_with(&root, false)
                    .unwrap_or_else(|error| panic!("{}: {error}", entry.path().display()));
                if !found.is_empty() {
                    footprints += 1;
                    read_zones += found.len();
                    placeable += usize::from(read(&root).is_ok());
                }
            }
        }
        eprintln!(
            "{read_zones} zones in {footprints} installed footprints read; {placeable} of the \
             footprints can be placed live"
        );
    }
}

#[cfg(test)]
mod live_tests {
    use super::*;
    use prost::Message;

    /// A zone without what KiCad assigns or derives: its identity, its
    /// parent, the per-layer fill slots and the order of its layers. An unset
    /// lock state reads as unlocked, which is how KiCad holds it.
    fn normalized(mut zone: kiapi::board::types::Zone) -> kiapi::board::types::Zone {
        zone.id = None;
        zone.parent = None;
        zone.filled_polygons.clear();
        zone.layers.sort_unstable();
        if zone.locked == kiapi::common::types::LockedState::LsUnknown as i32 {
            zone.locked = kiapi::common::types::LockedState::LsUnlocked as i32;
        }
        zone
    }

    /// Open a disposable board in KiCad 10 that holds footprints KiCad itself
    /// placed from its libraries (pcbnew's `FootprintLoad` + `BOARD::Add`),
    /// some of them with rule areas, at any position and angle on the front
    /// side. For every one, the zones this module builds from the library
    /// source must be the zones KiCad holds.
    #[test]
    #[ignore = "requires KONNECT_ZONE_ORACLE_BOARD open in KiCad 10 with IPC enabled"]
    fn live_zones_built_from_the_library_are_the_zones_kicad_holds() {
        let board = std::path::PathBuf::from(std::env::var("KONNECT_ZONE_ORACLE_BOARD").unwrap());
        let client = konnect_ipc::KiCadIpcClient::new(konnect_ipc::detect_ipc_address().unwrap());
        let document = client.find_open_board(&board).unwrap();
        let enabled = client
            .get_enabled_layers_in(document.clone())
            .unwrap()
            .layers
            .iter()
            .map(|layer| layer.id)
            .collect::<BTreeSet<_>>();
        let (mut footprints, mut compared) = (0, 0);
        for item in client
            .get_items_in(
                document,
                kiapi::common::types::KiCadObjectType::KotPcbFootprint,
            )
            .unwrap()
        {
            let live =
                kiapi::board::types::FootprintInstance::decode(item.value.as_slice()).unwrap();
            let definition = live.definition.clone().unwrap();
            let held = definition
                .items
                .iter()
                .filter(|child| konnect_ipc::builders::any_is(child, ZONE_TYPE))
                .map(|child| {
                    normalized(kiapi::board::types::Zone::decode(child.value.as_slice()).unwrap())
                })
                .collect::<Vec<_>>();
            if held.is_empty() || live.layer != kiapi::board::types::BoardLayer::BlFCu as i32 {
                continue;
            }
            let id = definition.id.unwrap();
            let lib_id = format!("{}:{}", id.library_nickname, id.entry_name);
            let source =
                super::super::super::pcb_components::resolve_footprint_source(&lib_id, &board)
                    .unwrap();
            // Pour settings aside: they are not in the message either way.
            let zones = read_with(&konnect_sexp::parse_sexp(&source).unwrap(), false).unwrap();
            let mut built = live.clone();
            built.definition.as_mut().unwrap().items.clear();
            apply(&zones, &mut built).unwrap();
            let mut packed = konnect_ipc::builders::pack_any(&built, FOOTPRINT_TYPE);
            konnect_ipc::client::fit_footprint_zone_layers(&mut packed, &enabled).unwrap();
            let built = kiapi::board::types::FootprintInstance::decode(packed.value.as_slice())
                .unwrap()
                .definition
                .unwrap()
                .items
                .iter()
                .map(|child| {
                    normalized(kiapi::board::types::Zone::decode(child.value.as_slice()).unwrap())
                })
                .collect::<Vec<_>>();
            assert_eq!(built.len(), held.len(), "{lib_id}: zone count");
            // KiCad lists a footprint's zones in its own order; match each
            // built zone to the held one it equals.
            let mut unmatched = held;
            for zone in built {
                let Some(found) = unmatched.iter().position(|candidate| *candidate == zone) else {
                    panic!(
                        "{lib_id} at {:?} turned {:?}: no zone KiCad holds equals\n{zone:#?}\nheld:\n{unmatched:#?}",
                        live.position, live.orientation
                    );
                };
                unmatched.swap_remove(found);
                compared += 1;
            }
            footprints += 1;
        }
        assert!(
            compared > 0,
            "the board holds no front-side footprint with a zone"
        );
        eprintln!("{compared} zones of {footprints} footprints equal the ones KiCad holds");
    }
}
