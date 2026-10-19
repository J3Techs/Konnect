//! S1 terminal-access audit (owned by WS-B).
//!
//! Before anyone blames the router, prove each terminal can be reached: a
//! per-layer flood fill from the pad, through space legal for the net's
//! profile, to outside the owning footprint's courtyard. Terminals that
//! cannot be reached on any routable layer are reported with a reason and
//! excluded from router scoring — they are a placement or escape problem,
//! not a routing one. Run it first on the dense BGA fields.

use crate::api::{FootprintId, LayerMask, PadId};
use crate::board::RouteBoard;
use crate::rules::RuleTable;
use serde::{Deserialize, Serialize};

/// Whether one terminal can escape its footprint.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TerminalAccess {
    pub pad: PadId,
    /// Routable layers on which the pad reaches outside the courtyard
    /// (directly, or through a via the rules allow).
    pub reachable: LayerMask,
    /// Why the pad is unreachable, when `reachable` is empty.
    pub reason: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct AccessAudit {
    /// One entry per audited pad, in pad order.
    pub terminals: Vec<TerminalAccess>,
}

impl AccessAudit {
    pub fn unreachable(&self) -> impl Iterator<Item = &TerminalAccess> {
        self.terminals.iter().filter(|t| t.reachable.is_empty())
    }
}

/// Audit every netted pad of `footprints` (all footprints when empty).
pub fn audit_terminal_access(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    footprints: &[FootprintId],
) -> AccessAudit {
    let _ = (board, rules, footprints);
    todo!("WS-B: per-layer flood fill from each pad to outside its courtyard")
}
