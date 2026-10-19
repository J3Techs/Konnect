//! S7 internal exact DRC (owned by WS-A).
//!
//! Checks routed copper against the board and itself with the exact kernel:
//! clearance (per the rule table, both A/B orders), track width, via floor,
//! hole clearance, hole-to-hole, edge clearance, track angle and minimum
//! segment length, plus same-layer shorts. Strict by 1–5 µm on curved
//! primitives, so KiCad never fails what this passed. Anything flagged is
//! ripped up, never emitted.

use crate::api::{ItemRef, LayerId};
use crate::board::RouteBoard;
use crate::plan::RoutedCopper;
use crate::rules::RuleTable;
use konnect_geom::{Coord, Point};
use serde::{Deserialize, Serialize};

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

/// One side of a violation: a board item or a routed one.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(tag = "source", rename_all = "snake_case")]
pub enum CheckedItem {
    Board { item: ItemRef },
    RoutedTrack { index: u32 },
    RoutedVia { index: u32 },
    Edge,
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

#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct CheckReport {
    pub violations: Vec<Violation>,
}

/// Check `copper` against `board` and itself.
pub fn check_routed(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    copper: &RoutedCopper,
) -> CheckReport {
    let _ = (board, rules, copper);
    todo!("WS-A: exact all-pairs check through the per-layer index")
}
