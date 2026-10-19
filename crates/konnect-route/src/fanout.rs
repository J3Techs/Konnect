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

use crate::api::{LayerId, NetId, PadId};
use crate::board::RouteBoard;
use crate::connectivity::ConnectivityReport;
use crate::rules::RuleTable;
use konnect_geom::{Coord, Point};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct FanoutSettings {
    /// Shrink applied to plane pieces before the via-inside-plane test, nm.
    pub plane_margin: Coord,
    /// Spokes a thermal relief must keep (`min_resolved_spokes`).
    pub min_spokes: u8,
    /// Allow pads of one net closer than this to share a via (0: never).
    pub share_within: Coord,
}

impl Default for FanoutSettings {
    fn default() -> Self {
        FanoutSettings {
            plane_margin: 0,
            min_spokes: 2,
            share_within: 0,
        }
    }
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
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "reason", rename_all = "snake_case")]
pub enum FanoutFailure {
    NoLegalSite,
    OutsideOwnPlane,
    Conflict,
}

#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct PlaneViaPlan {
    pub sites: Vec<PlaneViaSite>,
    pub failed: Vec<(PadId, FanoutFailure)>,
}

/// Plan plane vias for every unjoined SMD pad on a plane net.
pub fn plan_plane_vias(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    connectivity: &ConnectivityReport,
    settings: &FanoutSettings,
) -> PlaneViaPlan {
    let _ = (board, rules, connectivity, settings);
    todo!("WS-B: candidate sites, exact checks, per-tile independent set")
}
