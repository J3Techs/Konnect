//! S4 pin access (owned by WS-B).
//!
//! Exact access points and exit headings for each pad and escape end,
//! precomputed once per (pad shape class, rotation, profile) and placed on
//! the detailed lattice: a point inside the pad copper from which a track of
//! the profile can leave in each listed heading without violating any rule.

use crate::api::{LayerId, PadId, ProfileId};
use crate::board::RouteBoard;
use crate::rules::RuleTable;
use konnect_geom::Point;
use serde::{Deserialize, Serialize};

/// One way into a pad.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AccessPoint {
    pub point: Point,
    pub layer: LayerId,
    /// Bit `h`: a track may leave in heading `h` (0 = +x, 45° steps,
    /// counter-clockwise in the Y-up sense) — the same encoding as
    /// `TileRaster::heading_mask`.
    pub headings: u8,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PadAccess {
    pub pad: PadId,
    pub profile: ProfileId,
    /// Best first.
    pub points: Vec<AccessPoint>,
}

/// Access points for `pads` under `profile`.
pub fn compute_pin_access(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    pads: &[PadId],
    profile: ProfileId,
) -> Vec<PadAccess> {
    let _ = (board, rules, pads, profile);
    todo!("WS-B: per-shape-class access points and exit headings")
}
