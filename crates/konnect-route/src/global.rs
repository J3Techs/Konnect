//! S3 global routing (owned by WS-C).
//!
//! PathFinder (McMurchie & Ebeling) over a GCell grid: 1 mm cells on the
//! routable layers ([`crate::api::GCELL_PITCH_NM`]), refined to 0.5 mm in
//! dense fields. Edge capacity is the free track pitches left after fixed
//! copper and fanout. Connections are seeded with pattern routes (L, Z,
//! 3-bend), then negotiated with cost `(b + h) · p` for a bounded number of
//! iterations in deterministic bbox-disjoint batches. The output is one
//! corridor and layer assignment per connection, plus the overflow map.

use crate::api::{Connection, ConnectionId, LayerMask};
use crate::board::RouteBoard;
use crate::rules::RuleTable;
use konnect_geom::{BBox, Coord};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct GlobalSettings {
    pub gcell_pitch: Coord,
    pub fine_pitch: Coord,
    pub max_iterations: u32,
}

impl Default for GlobalSettings {
    fn default() -> Self {
        GlobalSettings {
            gcell_pitch: crate::api::GCELL_PITCH_NM,
            fine_pitch: crate::api::GCELL_FINE_PITCH_NM,
            max_iterations: 30,
        }
    }
}

/// The GCell grid: `cols × rows` cells per layer over `bbox`.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GCellGrid {
    pub bbox: BBox,
    pub pitch: Coord,
    pub cols: u32,
    pub rows: u32,
    pub layers: LayerMask,
}

impl GCellGrid {
    /// Dense GCell id: `(layer_slot · rows + row) · cols + col`.
    pub fn id(&self, layer_slot: u32, col: u32, row: u32) -> u32 {
        (layer_slot * self.rows + row) * self.cols + col
    }
}

/// One connection's corridor: GCell ids (see [`GCellGrid::id`]), in path
/// order, and the layers it may use.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Corridor {
    pub connection: ConnectionId,
    pub cells: Vec<u32>,
    pub layers: LayerMask,
}

#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct GlobalResult {
    pub corridors: Vec<Corridor>,
    /// Connections left overflowed after the last iteration.
    pub overflowed: Vec<ConnectionId>,
    /// Per-GCell overflow (demand − capacity, floored at 0), by GCell id.
    pub overflow: Vec<u16>,
}

/// Build the grid from the board's fixed copper.
pub fn build_grid(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    settings: &GlobalSettings,
) -> GCellGrid {
    let _ = (board, rules, settings);
    todo!("WS-C: GCell grid and capacities from fixed copper")
}

/// Route every connection globally.
pub fn route_global(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    grid: &GCellGrid,
    connections: &[Connection],
    settings: &GlobalSettings,
) -> GlobalResult {
    let _ = (board, rules, grid, connections, settings);
    todo!("WS-C: pattern seeding plus PathFinder negotiation")
}
