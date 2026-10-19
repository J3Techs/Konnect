//! Single-connection lattice A* (owned by WS-D1).
//!
//! Multi-source, multi-target octilinear A* over one tile's rasters, with
//! heading and run-length state, a radix heap, and an octile-plus-via
//! heuristic (or a VPR-style map lookahead). Moves: straight, ±45°, ±90°,
//! and a via where every spanned layer's `via_site` allows it. `Near` nodes
//! get a cached exact check; `Blocked` nodes are walls; other nets' routed
//! copper costs `present`/`history` per [`crate::compute::CostParams`].

use crate::compute::{CostParams, JobDescriptor, JobPool, SearchOutcome, TileStack};
use serde::{Deserialize, Serialize};

/// Per-search counters for telemetry.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct SearchStats {
    pub expansions: u64,
    pub exact_checks: u64,
    pub exact_check_cache_hits: u64,
}

/// The eight lattice headings, 0 = +x, counter-clockwise in 45° steps in
/// the Y-up sense: `(dx, dy)` per heading.
pub const HEADINGS: [(i32, i32); 8] = [
    (1, 0),
    (1, 1),
    (0, 1),
    (-1, 1),
    (-1, 0),
    (-1, -1),
    (0, -1),
    (1, -1),
];

/// Whether turning from heading `a` to heading `b` is allowed: straight or a
/// 45°/90° turn. A 135° turn (and a reversal) leaves an interior angle
/// below 90° and is forbidden.
pub fn turn_allowed(a: u8, b: u8) -> bool {
    let d = (i32::from(b) - i32::from(a)).rem_euclid(8);
    matches!(d, 0 | 1 | 2 | 6 | 7)
}

/// Route one job inside `stack`. `job` indexes `pool`.
pub fn search(
    stack: &TileStack,
    pool: &JobPool,
    job: &JobDescriptor,
    job_index: u32,
    params: &CostParams,
    stats: &mut SearchStats,
) -> SearchOutcome {
    let _ = (stack, pool, job, job_index, params, stats);
    todo!("WS-D1: octilinear A* with heading/run state and radix heap")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_straight_45_and_90_degree_turns() {
        assert!(turn_allowed(0, 0));
        assert!(turn_allowed(0, 1));
        assert!(turn_allowed(0, 2));
        assert!(!turn_allowed(0, 3));
        assert!(!turn_allowed(0, 4));
        assert!(!turn_allowed(0, 5));
        assert!(turn_allowed(0, 6));
        assert!(turn_allowed(0, 7));
        assert!(turn_allowed(7, 1));
    }
}
