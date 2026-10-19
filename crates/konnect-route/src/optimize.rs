//! S6 optimization (owned by WS-E).
//!
//! Gridless clean-up of routed copper: pull-tight, 45° corner cuts, merging
//! collinear segments, via removal and junction splitting. Every candidate
//! edit is accepted only if the exact kernel ([`crate::check`]) passes it,
//! and the result must then pass the strict geometry lint (minimum bend
//! angle from the rule table).

use crate::board::RouteBoard;
use crate::plan::RoutedCopper;
use crate::rules::RuleTable;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct OptimizeReport {
    pub segments_merged: u32,
    pub corners_cut: u32,
    pub vias_removed: u32,
    pub edits_rejected: u32,
}

/// Improve `copper` in place without introducing any violation.
pub fn optimize(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    copper: &mut RoutedCopper,
) -> OptimizeReport {
    let _ = (board, rules, copper);
    todo!("WS-E: exact-checked pull-tight, merges and via removal")
}
