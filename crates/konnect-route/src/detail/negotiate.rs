//! Negotiated congestion over tiles (owned by WS-D2).
//!
//! The TritonRoute/Dr.CU search-and-repair loop: fixed copper is a hard
//! wall, other nets' new copper is soft with present and history costs, and
//! each iteration rips up only the connections in conflict. The board is cut
//! into tiles of [`crate::api::TILE_CORE_NM`] with [`crate::api::TILE_HALO_NM`]
//! halos, 4-coloured so same-colour tiles run in parallel (rayon), with the
//! tile grid shifted by half a tile on odd iterations. Dense BGA fields are
//! split into sub-tiles and scheduled first. After the iteration limit,
//! remaining connections fail with a [`crate::api::FailureReason`].

use crate::api::{ConnectionId, FailureReason, Generation, TileId};
use crate::board::RouteBoard;
use crate::compute::RouteCompute;
use crate::global::GlobalResult;
use crate::plan::RoutedCopper;
use crate::rules::RuleTable;
use konnect_geom::BBox;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct NegotiationSettings {
    pub max_iterations: u32,
    /// Present-cost factor growth per iteration, ×1000.
    pub present_growth: u32,
    pub history_increment: u16,
    /// Worker threads; 0 = rayon's default. Must not change the result.
    pub threads: usize,
}

impl Default for NegotiationSettings {
    fn default() -> Self {
        NegotiationSettings {
            max_iterations: 10,
            present_growth: 1_500,
            history_increment: 1,
            threads: 0,
        }
    }
}

/// One tile of the schedule.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Tile {
    pub id: TileId,
    /// The core this tile owns; commits outside it are refused.
    pub core: BBox,
    /// Core plus halo: what the tile's rasters cover.
    pub window: BBox,
    /// 0..4; tiles of one colour never share a halo.
    pub colour: u8,
}

#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct DetailResult {
    pub copper: RoutedCopper,
    pub routed: Vec<ConnectionId>,
    pub failed: Vec<(ConnectionId, FailureReason)>,
    pub iterations: u32,
    pub generation: Generation,
}

/// Cut the board into 4-coloured tiles; `shift` offsets the grid by half a
/// tile (odd iterations).
pub fn plan_tiles(board: &RouteBoard, shift: bool) -> Vec<Tile> {
    let _ = (board, shift);
    todo!("WS-D2: tile grid, halos and 4-colouring")
}

/// Run negotiated detailed routing for every corridor in `global`.
pub fn negotiate(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    global: &GlobalResult,
    settings: &NegotiationSettings,
    backend: &mut dyn RouteCompute,
) -> DetailResult {
    let _ = (board, rules, global, settings, backend);
    todo!("WS-D2: present/history negotiation with rip-up over tiles")
}
