//! Plane nets in a run: what their planes join, what the fanout joined,
//! what is still open, and which crossable pours the new copper cut.
//!
//! A *plane net* here is any net that owns a copper zone, filled or not
//! ([`zone_nets`]). Its pads are meant to be joined by the pour: a
//! through-hole pad inside the fill is joined as saved, an SMD pad on a
//! layer without the pour gets a fanout via (S2). What the fanout leaves
//! open is routed as tracks only when [`PlaneTracks`] says so; otherwise
//! those connections are [`PipelineResult::left_to_planes`] and every pad
//! behind them is listed in [`PlaneNetReport::open`] with the reason, so a
//! run never leaves a plane pad unconnected without saying so.
//!
//! [`PipelineResult::left_to_planes`]: super::PipelineResult::left_to_planes

use super::{PipelineScope, PipelineSettings};
use crate::api::{Connection, ConnectionId, ItemKind, NetId, PadId, ZoneId};
use crate::board::RouteBoard;
use crate::connectivity::{zone_nets, ConnectivityReport};
use crate::fanout::{FanoutFailure, FanoutSkip, PlaneViaPlan};
use crate::plan::RoutedCopper;
use crate::rules::{RuleSubject, RuleTable};
use konnect_geom::{IndexedPolygon, Shape};
use serde::{Deserialize, Serialize};
use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};

/// What happens to the open connections of plane nets that the fanout
/// leaves.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub enum PlaneTracks {
    /// Route them as tracks like any other net's (the engine's default).
    #[default]
    All,
    /// Leave them open, listed in
    /// [`PipelineResult::left_to_planes`](super::PipelineResult::left_to_planes),
    /// except for these nets, which are routed as tracks.
    Only(BTreeSet<NetId>),
}

impl PlaneTracks {
    /// Whether open connections of plane net `net` are routed as tracks.
    pub fn routes(&self, net: NetId) -> bool {
        match self {
            PlaneTracks::All => true,
            PlaneTracks::Only(nets) => nets.contains(&net),
        }
    }
}

/// Why a plane-net pad is not joined to its plane after the fanout.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PlaneOpenReason {
    /// Every via candidate breaks a rule against copper, holes or the edge.
    FanoutNoLegalSite,
    /// Every candidate that clears the board falls outside the net's plane.
    FanoutOutsideOwnPlane,
    /// Legal candidates existed, but each conflicts with a chosen site.
    FanoutConflict,
    /// The pad's island already carries a track, arc or via that does not
    /// reach the plane; the fanout leaves it alone.
    Escaped,
    /// The pad's footprint is left to hand-drawn escapes.
    ExcludedFootprint,
    /// Not an SMD pad (a through-hole pad outside the fill): the fanout
    /// plans vias for SMD pads only.
    NotSmd,
    /// Its via site lies outside the run's region.
    OutsideScope,
    /// The net's class is excluded from routing, fanout included.
    ExcludedClass,
    /// The run had no fanout stage.
    FanoutOff,
    /// The fanout did not consider the pad.
    NotPlanned,
}

/// One plane net's pads against its planes, with the fanout applied.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct PlaneNetReport {
    pub net: NetId,
    /// The net has filled zone copper. `false`: every zone of the net is
    /// unfilled in the saved file, so nothing is known about what the plane
    /// joins, no fanout was planned and the pad lists are empty.
    pub filled: bool,
    /// The net's zone pieces with no fill (authored outline only).
    pub unfilled: Vec<ZoneId>,
    /// The net's open connections were routed as tracks ([`PlaneTracks`]).
    pub routed_as_tracks: bool,
    /// The net's class is excluded from routing.
    pub class_excluded: bool,
    /// Pads of the net.
    pub pads: usize,
    /// Pads joined to a plane piece by the copper as saved, sorted.
    pub joined: Vec<PadId>,
    /// Pads joined to a plane piece by this run's fanout vias, sorted.
    pub fanout: Vec<PadId>,
    /// Pads not joined to any plane piece, with why, sorted: each needs a
    /// fanout via or a track.
    pub open: Vec<(PadId, PlaneOpenReason)>,
    /// Open connections of the net, within the scope, left unrouted because
    /// the net is left to its plane.
    pub connections_left: usize,
}

/// `connections` split into those to route and those of plane nets that
/// `policy` leaves to their planes, each list renumbered so ids equal
/// positions.
pub fn split_left_to_planes(
    board: &RouteBoard,
    connections: Vec<Connection>,
    policy: &PlaneTracks,
) -> (Vec<Connection>, Vec<Connection>) {
    if *policy == PlaneTracks::All {
        return (connections, Vec::new());
    }
    let planes: BTreeSet<NetId> = zone_nets(board).into_iter().collect();
    let (left, keep): (Vec<Connection>, Vec<Connection>) = connections
        .into_iter()
        .partition(|c| planes.contains(&c.net) && !policy.routes(c.net));
    let renumber = |v: Vec<Connection>| -> Vec<Connection> {
        v.into_iter()
            .enumerate()
            .map(|(i, c)| Connection {
                id: ConnectionId(i as u32),
                ..c
            })
            .collect()
    };
    (renumber(keep), renumber(left))
}

/// The plane nets of the scope, read from `conn`, the connectivity of
/// `work` (the board with the fanout as locked copper). `plan` is the
/// fanout as kept; `out_of_scope` holds the pads whose sites the scope
/// dropped.
pub(super) fn report(
    work: &RouteBoard,
    rules: &dyn RuleTable,
    conn: &ConnectivityReport,
    scope: &PipelineScope,
    settings: &PipelineSettings,
    plan: Option<&PlaneViaPlan>,
    out_of_scope: &BTreeSet<PadId>,
) -> Vec<PlaneNetReport> {
    let joins = conn.plane_joins(work);
    let mut site_pads: BTreeSet<PadId> = BTreeSet::new();
    let mut failed: BTreeMap<PadId, PlaneOpenReason> = BTreeMap::new();
    let mut skipped: BTreeMap<PadId, &FanoutSkip> = BTreeMap::new();
    if let Some(plan) = plan {
        site_pads.extend(plan.sites.iter().map(|s| s.pad));
        for (pad, why) in &plan.failed {
            failed.insert(
                *pad,
                match why {
                    FanoutFailure::NoLegalSite => PlaneOpenReason::FanoutNoLegalSite,
                    FanoutFailure::OutsideOwnPlane => PlaneOpenReason::FanoutOutsideOwnPlane,
                    FanoutFailure::Conflict => PlaneOpenReason::FanoutConflict,
                },
            );
        }
        for (pad, why) in &plan.skipped {
            skipped.insert(*pad, why);
        }
    }
    let via_pad = |pad: PadId| match skipped.get(&pad) {
        Some(FanoutSkip::JoinedThroughIsland { via_pad }) => *via_pad,
        _ => pad,
    };
    let mut out = Vec::new();
    for net in zone_nets(work) {
        if scope.nets.as_ref().is_some_and(|n| !n.contains(&net)) {
            continue;
        }
        let class_excluded = settings
            .excluded_classes
            .contains(&rules.class_of(Some(net)));
        let mut r = PlaneNetReport {
            net,
            unfilled: work
                .zones
                .iter()
                .filter(|z| z.net == Some(net) && !z.filled)
                .map(|z| z.id)
                .collect(),
            routed_as_tracks: !class_excluded && settings.plane_tracks.routes(net),
            class_excluded,
            pads: work.pads.iter().filter(|p| p.net == Some(net)).count(),
            ..PlaneNetReport::default()
        };
        if let Some(join) = joins.iter().find(|j| j.net == net) {
            r.filled = true;
            let open: BTreeSet<PadId> = join.unjoined_pads.iter().copied().collect();
            for pad in work.pads.iter().filter(|p| p.net == Some(net)) {
                if open.contains(&pad.id) {
                    continue;
                }
                if site_pads.contains(&via_pad(pad.id)) {
                    r.fanout.push(pad.id);
                } else {
                    r.joined.push(pad.id);
                }
            }
            for &pad in &join.unjoined_pads {
                let why = if class_excluded {
                    PlaneOpenReason::ExcludedClass
                } else if plan.is_none() {
                    PlaneOpenReason::FanoutOff
                } else if out_of_scope.contains(&via_pad(pad)) {
                    PlaneOpenReason::OutsideScope
                } else if let Some(why) = failed.get(&pad) {
                    *why
                } else {
                    match skipped.get(&pad) {
                        Some(FanoutSkip::Escaped) => PlaneOpenReason::Escaped,
                        Some(FanoutSkip::ExcludedFootprint) => PlaneOpenReason::ExcludedFootprint,
                        Some(FanoutSkip::NotSmd) => PlaneOpenReason::NotSmd,
                        Some(FanoutSkip::JoinedThroughIsland { .. }) | None => {
                            PlaneOpenReason::NotPlanned
                        }
                    }
                };
                r.open.push((pad, why));
            }
        }
        out.push(r);
    }
    out
}

/// A crossable pour that new copper of another net comes within clearance
/// of: the refill has to carve it there.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct ZoneCrossing {
    pub zone: ZoneId,
    /// Routed tracks of other nets inside the pour's clearance.
    pub tracks: usize,
    /// Routed vias of other nets inside the pour's clearance.
    pub vias: usize,
}

/// The crossable filled pieces of `board`
/// ([`ZonePiece::crossable`](crate::board::ZonePiece::crossable)) that
/// `copper` of another net touches or comes within the rules' clearance of,
/// in piece order. Each is copper as saved and is only legal once a refill
/// has carved it around the new copper.
pub fn crossed_zones(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    copper: &RoutedCopper,
) -> Vec<ZoneCrossing> {
    let mut out = Vec::new();
    for z in board.zones.iter().filter(|z| z.crossable && z.filled) {
        let zone_net = z.net;
        let fill = IndexedPolygon::new(z.region.clone());
        let zone = RuleSubject::new(ItemKind::Zone, zone_net);
        let near = |shape: &Shape, kind: ItemKind, net: NetId| {
            let c = rules.clearance(&zone, &RuleSubject::new(kind, Some(net)), z.layer);
            fill.gap_cmp(shape, i64::from(c)) == Ordering::Less
        };
        let tracks = copper
            .tracks
            .iter()
            .filter(|t| t.layer == z.layer && Some(t.net) != zone_net)
            .filter(|t| near(&Shape::Segment(t.seg), ItemKind::Track, t.net))
            .count();
        let vias = copper
            .vias
            .iter()
            .filter(|v| {
                Some(v.net) != zone_net && (v.layers.is_empty() || v.layers.contains(z.layer))
            })
            .filter(|v| {
                let disc = Shape::Circle(konnect_geom::Circle::from_diameter(v.center, v.diameter));
                near(&disc, ItemKind::Via, v.net)
            })
            .count();
        if tracks + vias > 0 {
            out.push(ZoneCrossing {
                zone: z.id,
                tracks,
                vias,
            });
        }
    }
    out
}
