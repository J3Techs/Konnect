//! Zones and plane nets in a native route job.
//!
//! Two caller decisions live here.
//!
//! **Crossable zones.** A filled pour on a routing layer is copper as
//! saved, so the router treats it as an obstacle for every other net. A
//! refill re-pours a zone around new tracks, so a caller who will refill
//! may declare zones crossable (`carve_zones` by name, `carve_zone_nets` by
//! net, or every copper zone with `zones_are_obstacles: false`). Their fill
//! then blocks nothing; connectivity and the plane fanout still read it as
//! saved. Rule areas are never crossable. The report names every declared
//! zone the new copper actually cut ([`RefillNotice`]).
//!
//! **Plane nets.** A net that owns a copper zone is left to its plane and
//! the plane fanout: its open connections are not routed as tracks unless
//! the caller names the net in `plane_nets_as_tracks`. The report lists,
//! per plane net, the pads the plane joins, the pads the fanout joined and
//! the pads that still need a via or a track ([`PlaneNetSummary`]). A net
//! whose zones have no fill in the saved file is reported as such and is
//! not routed as tracks either: nothing proves what its pour will join.

use super::NativeJobSpec;
use crate::tools::board_inspect::{layer_names, pad_label};
use konnect_route::api::{Connection, ItemRef, NetId, PadId, ZoneId};
use konnect_route::connectivity::zone_nets;
use konnect_route::pipeline::coverage::select_nets;
use konnect_route::pipeline::{crossed_zones, PlaneNetReport, PlaneOpenReason};
use konnect_route::plan::RoutedCopper;
use konnect_route::{RouteBoard, RuleTable};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

/// Pads listed per joined list of a plane net; the count is always whole.
const JOINED_PADS_LISTED: usize = 200;

/// Unrouted connections listed per plane net; the count is always whole.
pub const UNROUTED_CONNECTIONS_LISTED: usize = 200;

/// A zone option that cannot be used, by argument name.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ZoneOptionError {
    pub field: &'static str,
    pub reason: String,
}

impl std::fmt::Display for ZoneOptionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}: {}", self.field, self.reason)
    }
}

/// One copper zone on one layer, as the report names it.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct ZoneRef {
    /// The zone's name; `None` for an unnamed zone.
    pub zone: Option<String>,
    pub layer: String,
    /// The zone's net; empty for a copper zone with no net.
    pub net: String,
}

fn zone_ref(board: &RouteBoard, zone: ZoneId) -> ZoneRef {
    let z = &board.zones[zone.index()];
    ZoneRef {
        zone: z.zone_name.clone(),
        layer: board.layers.name(z.layer).to_string(),
        net: z.net.map(|n| board.net(n).name.clone()).unwrap_or_default(),
    }
}

/// What the spec's zone options resolve to on a board.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ZonePolicy {
    /// The zones declared crossable, each (name, layer, net) once, sorted.
    pub crossable: Vec<ZoneRef>,
    /// Plane nets to route as tracks.
    pub as_tracks: BTreeSet<NetId>,
}

fn quoted(names: &[&String]) -> String {
    names
        .iter()
        .map(|n| format!("'{n}'"))
        .collect::<Vec<_>>()
        .join(", ")
}

/// The plane nets matching `patterns` (names or `*` `?` patterns); a
/// pattern that matches none of them is an error naming them.
fn plane_nets_matching(
    board: &RouteBoard,
    planes: &BTreeSet<NetId>,
    patterns: &[String],
    field: &'static str,
) -> Result<BTreeSet<NetId>, ZoneOptionError> {
    let mut out = BTreeSet::new();
    let mut unmatched = Vec::new();
    for p in patterns {
        let found: Vec<NetId> = select_nets(board, None, std::slice::from_ref(p))
            .into_iter()
            .filter(|n| planes.contains(n))
            .collect();
        if found.is_empty() {
            unmatched.push(p);
        }
        out.extend(found);
    }
    if unmatched.is_empty() {
        return Ok(out);
    }
    let known: Vec<&String> = planes.iter().map(|n| &board.net(*n).name).collect();
    Err(ZoneOptionError {
        field,
        reason: format!(
            "no net with a copper zone matches {}; {}",
            quoted(&unmatched),
            if known.is_empty() {
                "the board has no copper zones".to_string()
            } else {
                format!("the nets with a copper zone are {}", quoted(&known))
            }
        ),
    })
}

impl NativeJobSpec {
    /// Resolve the zone options against `board`: mark the crossable zone
    /// pieces on it and return what was declared. Nothing is marked when
    /// an option names something the board lacks.
    pub fn apply_zone_options(
        &self,
        board: &mut RouteBoard,
    ) -> Result<ZonePolicy, ZoneOptionError> {
        let planes: BTreeSet<NetId> = zone_nets(board).into_iter().collect();
        // By name, with the refusals the planning tools' carve_zones give.
        let named = |n: &String| board.zones.iter().any(|z| z.zone_name.as_ref() == Some(n));
        let missing: Vec<&String> = self.carve_zones.iter().filter(|n| !named(n)).collect();
        if !missing.is_empty() {
            let (areas, unknown): (Vec<&String>, Vec<&String>) = missing
                .into_iter()
                .partition(|n| board.keepouts.iter().any(|k| k.name.as_ref() == Some(*n)));
            let mut why = Vec::new();
            if !unknown.is_empty() {
                why.push(format!(
                    "no copper zone of the board is named {}",
                    quoted(&unknown)
                ));
            }
            if !areas.is_empty() {
                why.push(format!(
                    "{} {} a rule area, not a copper zone: a refill pours no copper there to carve, and a rule area is never crossable",
                    quoted(&areas),
                    if areas.len() == 1 { "is" } else { "are" }
                ));
            }
            return Err(ZoneOptionError {
                field: "carve_zones",
                reason: why.join("; "),
            });
        }
        let carve_nets =
            plane_nets_matching(board, &planes, &self.carve_zone_nets, "carve_zone_nets")?;
        let as_tracks = plane_nets_matching(
            board,
            &planes,
            &self.plane_nets_as_tracks,
            "plane_nets_as_tracks",
        )?;
        let mut crossable = BTreeSet::new();
        for z in &mut board.zones {
            // A copper zone with no net is refilled like any other.
            z.crossable = !self.zones_are_obstacles
                || z.net.is_some_and(|net| carve_nets.contains(&net))
                || z.zone_name
                    .as_ref()
                    .is_some_and(|n| self.carve_zones.contains(n));
        }
        for z in board.zones.iter().filter(|z| z.crossable) {
            crossable.insert(zone_ref(board, z.id));
        }
        Ok(ZonePolicy {
            crossable: crossable.into_iter().collect(),
            as_tracks,
        })
    }
}

/// Pads by label, capped; `count` is always the whole number.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct PadList {
    pub count: usize,
    pub pads: Vec<String>,
    /// Set when `pads` holds fewer than `count`.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub truncated: bool,
}

/// A plane-net pad the plane does not join.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct OpenPad {
    pub pad: String,
    pub layers: Vec<String>,
    pub reason: PlaneOpenReason,
}

/// One plane net in a job report.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct PlaneNetSummary {
    pub net: String,
    /// `filled`, `partly_filled` (some zones of the net have no fill) or
    /// `unfilled` (none has: the pad lists are empty, nothing is known).
    pub fill: String,
    /// `left_to_plane` (the default: fanout vias only, no tracks),
    /// `routed_as_tracks` (named in `plane_nets_as_tracks`) or
    /// `class_excluded` (its class is in `exclude_classes`: no fanout, no
    /// tracks).
    pub handling: String,
    /// The net's zones with no fill in the saved file.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub unfilled_zones: Vec<ZoneRef>,
    pub pads: usize,
    /// Pads the plane joins in the saved fill.
    pub connected_by_plane: PadList,
    /// Pads this run's fanout vias join to the plane.
    pub connected_by_fanout: PadList,
    /// Pads not joined to the plane: each needs a fanout via or a track.
    /// Never truncated.
    pub needs_via_or_track: Vec<OpenPad>,
    /// Open connections of the net left unrouted because it is left to its
    /// plane.
    pub connections_left_unrouted: usize,
    /// Those connections by their two ends, at most
    /// [`UNROUTED_CONNECTIONS_LISTED`]; `connections_left_unrouted` is
    /// always the whole number.
    #[serde(default)]
    pub unrouted_connections: Vec<UnroutedConnection>,
}

/// A connection left unrouted, by the items at its two ends.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct UnroutedConnection {
    pub from: String,
    pub to: String,
}

/// One end of a connection of the route stage. Pads and zones are the
/// board's; a track or via may be fanout copper of this run, which the
/// board as loaded does not hold.
fn end_label(board: &RouteBoard, item: ItemRef) -> String {
    match item {
        ItemRef::Pad(p) => board
            .pads
            .get(p.index())
            .map_or_else(|| "pad".to_string(), |pad| pad_label(board, pad)),
        ItemRef::Zone(z) => board.zones.get(z.index()).map_or_else(
            || "zone".to_string(),
            |z| {
                format!(
                    "zone {}@{}",
                    z.zone_name.as_deref().unwrap_or("(unnamed)"),
                    board.layers.name(z.layer)
                )
            },
        ),
        ItemRef::Via(v) => board.vias.get(v.index()).map_or_else(
            || "fanout via of this run".to_string(),
            |v| {
                let (x, y) = v.center.to_mm();
                format!("via at ({x}, {y})")
            },
        ),
        ItemRef::Track(t) => board.tracks.get(t.index()).map_or_else(
            || "fanout track of this run".to_string(),
            |t| {
                let (x, y) = t.seg.a.to_mm();
                format!("track from ({x}, {y}) on {}", board.layers.name(t.layer))
            },
        ),
        ItemRef::Arc(_) => "arc".to_string(),
    }
}

fn pad_list(board: &RouteBoard, pads: &[PadId]) -> PadList {
    PadList {
        count: pads.len(),
        pads: pads
            .iter()
            .take(JOINED_PADS_LISTED)
            .map(|p| pad_label(board, &board.pads[p.index()]))
            .collect(),
        truncated: pads.len() > JOINED_PADS_LISTED,
    }
}

/// The pipeline's plane-net reports in the job's terms; `left` holds the
/// connections the run left to the planes.
pub fn summaries(
    board: &RouteBoard,
    reports: &[PlaneNetReport],
    left: &[Connection],
) -> Vec<PlaneNetSummary> {
    reports
        .iter()
        .map(|r| {
            let unfilled: BTreeSet<ZoneRef> =
                r.unfilled.iter().map(|z| zone_ref(board, *z)).collect();
            PlaneNetSummary {
                net: board.net(r.net).name.clone(),
                fill: match (r.filled, unfilled.is_empty()) {
                    (true, true) => "filled",
                    (true, false) => "partly_filled",
                    (false, _) => "unfilled",
                }
                .to_string(),
                handling: if r.class_excluded {
                    "class_excluded"
                } else if r.routed_as_tracks {
                    "routed_as_tracks"
                } else {
                    "left_to_plane"
                }
                .to_string(),
                unfilled_zones: unfilled.into_iter().collect(),
                pads: r.pads,
                connected_by_plane: pad_list(board, &r.joined),
                connected_by_fanout: pad_list(board, &r.fanout),
                needs_via_or_track: r
                    .open
                    .iter()
                    .map(|(p, why)| {
                        let pad = &board.pads[p.index()];
                        OpenPad {
                            pad: pad_label(board, pad),
                            layers: layer_names(board, pad.layers),
                            reason: *why,
                        }
                    })
                    .collect(),
                connections_left_unrouted: r.connections_left,
                unrouted_connections: left
                    .iter()
                    .filter(|c| c.net == r.net)
                    .take(UNROUTED_CONNECTIONS_LISTED)
                    .map(|c| UnroutedConnection {
                        from: end_label(board, c.from),
                        to: end_label(board, c.to),
                    })
                    .collect(),
            }
        })
        .collect()
}

/// A crossable zone the new copper cut.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CrossedZone {
    #[serde(flatten)]
    pub zone: ZoneRef,
    /// New tracks of other nets inside the fill's clearance.
    pub tracks: usize,
    /// New vias of other nets inside the fill's clearance.
    pub vias: usize,
}

/// What the crossable zones mean for the plan.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct RefillNotice {
    /// The plan's copper crosses saved fill: it is legal only after a
    /// refill.
    pub required: bool,
    /// Every zone declared crossable.
    pub crossable_zones: Vec<ZoneRef>,
    /// Of those, the zones the plan's copper cuts.
    pub crossed_zones: Vec<CrossedZone>,
    pub note: String,
}

/// The refill notice for `copper` (the plan's fanout and routed copper) on
/// `board`, whose crossable pieces `policy` marked. `None` when no zone was
/// declared crossable.
pub fn refill_notice(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    policy: &ZonePolicy,
    copper: &RoutedCopper,
) -> Option<RefillNotice> {
    if policy.crossable.is_empty() {
        return None;
    }
    let mut crossed: BTreeMap<ZoneRef, (usize, usize)> = BTreeMap::new();
    for c in crossed_zones(board, rules, copper) {
        let e = crossed.entry(zone_ref(board, c.zone)).or_default();
        e.0 += c.tracks;
        e.1 += c.vias;
    }
    let required = !crossed.is_empty();
    Some(RefillNotice {
        required,
        crossable_zones: policy.crossable.clone(),
        crossed_zones: crossed
            .into_iter()
            .map(|(zone, (tracks, vias))| CrossedZone { zone, tracks, vias })
            .collect(),
        note: if required {
            "The plan's copper runs through the saved fill of crossed_zones: until those zones are \
             refilled that is a short. A refill is required before DRC or fabrication: after applying \
             the plan, refill every zone and save, then run DRC. The refill carves each pour around the \
             new copper, and can leave a pad the pour used to join on a cut-off island (an unconnected \
             item), which only that DRC shows."
        } else {
            "No copper of the plan comes within clearance of a crossable zone's fill."
        }
        .to_string(),
    })
}

fn zone_list(list: &[ZoneRef]) -> String {
    list.iter()
        .map(|z| format!("{}@{}", z.zone.as_deref().unwrap_or("(unnamed)"), z.layer))
        .collect::<Vec<_>>()
        .join(", ")
}

/// What to say when `failed` connections were not routed on a board whose
/// saved fills are obstacles on routing layers: the caller may not know
/// that a pour it is going to refill anyway is what walls its pads in.
/// `None` when nothing failed or no such fill exists.
pub fn obstacle_hint(board: &RouteBoard, failed: usize) -> Option<String> {
    if failed == 0 {
        return None;
    }
    let blocking: BTreeSet<ZoneRef> = board
        .zones
        .iter()
        .filter(|z| z.filled && !z.crossable && z.net.is_some() && board.routable.contains(z.layer))
        .map(|z| zone_ref(board, z.id))
        .collect();
    if blocking.is_empty() {
        return None;
    }
    let blocking: Vec<ZoneRef> = blocking.into_iter().collect();
    Some(format!(
        "{failed} connection(s) were not routed, and the saved fill of {} is an obstacle to other nets on the routing layers. If these zones are refilled after the plan is applied, declare them crossable (carve_zones, carve_zone_nets) and run again.",
        zone_list(&blocking)
    ))
}

/// The lines a caller must not miss, most serious first.
pub fn warnings(plane_nets: &[PlaneNetSummary], refill: Option<&RefillNotice>) -> Vec<String> {
    let zones = zone_list;
    let mut out = Vec::new();
    for p in plane_nets {
        let unrouted = if p.handling == "routed_as_tracks" {
            "it was routed as tracks because plane_nets_as_tracks names it. To leave it to its plane instead, refill the zones in KiCad and save, then run again without naming it".to_string()
        } else if p.handling == "class_excluded" {
            "its class is excluded, so it got neither fanout vias nor tracks. Refill the zones in KiCad and save to see what the plane joins".to_string()
        } else {
            format!(
                "its {} open connection(s) were left unrouted and no fanout vias were planned. Refill the zones in KiCad and save, then run again; or name the net in plane_nets_as_tracks to route it as tracks",
                p.connections_left_unrouted
            )
        };
        match p.fill.as_str() {
            "unfilled" => out.push(format!(
                "Plane net {} has no zone fill in the saved file ({}): {unrouted}.",
                p.net,
                zones(&p.unfilled_zones)
            )),
            "partly_filled" => out.push(format!(
                "Plane net {} has zones with no fill in the saved file ({}): only its filled zones were used. Refill and save for a complete picture.",
                p.net,
                zones(&p.unfilled_zones)
            )),
            _ => {}
        }
    }
    if let Some(r) = refill.filter(|r| r.required) {
        let crossed: Vec<ZoneRef> = r.crossed_zones.iter().map(|c| c.zone.clone()).collect();
        out.push(format!(
            "Refill required before DRC or fabrication: the plan's copper crosses the saved fill of {} and shorts to it until the zones are refilled. Apply the plan, refill all zones, save and run DRC before trusting clearances or connectivity.",
            zones(&crossed)
        ));
    }
    for p in plane_nets {
        if p.handling != "routed_as_tracks" && !p.needs_via_or_track.is_empty() {
            out.push(format!(
                "Plane net {}: {} pad(s) are not joined to the plane and got no fanout via (plane_nets[].needs_via_or_track); {} connection(s) were left unrouted. Place vias (plan_plane_vias, plan_tap_vias) or name the net in plane_nets_as_tracks.",
                p.net,
                p.needs_via_or_track.len(),
                p.connections_left_unrouted
            ));
        } else if p.handling == "left_to_plane"
            && p.fill != "unfilled"
            && p.connections_left_unrouted > 0
        {
            // Every pad reaches some plane piece, yet the net is in parts.
            out.push(format!(
                "Plane net {}: every pad is joined to a piece of its plane, but the pieces are not joined to each other; {} connection(s) between them were left unrouted (plane_nets[].unrouted_connections). Join the pieces with vias or tracks, or name the net in plane_nets_as_tracks.",
                p.net, p.connections_left_unrouted
            ));
        }
    }
    out
}
