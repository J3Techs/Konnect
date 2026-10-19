//! `u32` fixed-point search costs.
//!
//! One unit is 1/1000 of a straight lattice step, so a straight step costs
//! 1000 and a diagonal one 1414 (√2 · 1000, rounded). All arithmetic
//! saturates: [`Cost::INFINITE`] absorbs everything, so an overflow can make
//! a path look forbidden but never cheap. Integer costs make the CPU and a
//! GPU kernel order equal-cost paths identically.

use serde::{Deserialize, Serialize};
use std::ops::Add;

#[derive(
    Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Default, Serialize, Deserialize,
)]
#[repr(transparent)]
#[serde(transparent)]
pub struct Cost(pub u32);

impl Cost {
    pub const ZERO: Cost = Cost(0);
    /// Forbidden. Saturating arithmetic keeps it absorbing.
    pub const INFINITE: Cost = Cost(u32::MAX);
    /// One straight lattice step.
    pub const STEP_STRAIGHT: Cost = Cost(1000);
    /// One diagonal lattice step.
    pub const STEP_DIAGONAL: Cost = Cost(1414);

    pub fn is_infinite(self) -> bool {
        self == Cost::INFINITE
    }

    pub fn saturating_add(self, o: Cost) -> Cost {
        Cost(self.0.saturating_add(o.0))
    }

    /// `self · num / den`, rounded down, saturating; `den == 0` is infinite.
    pub fn scale(self, num: u32, den: u32) -> Cost {
        if den == 0 || self.is_infinite() {
            return Cost::INFINITE;
        }
        let v = u64::from(self.0) * u64::from(num) / u64::from(den);
        Cost(u32::try_from(v).unwrap_or(u32::MAX))
    }
}

impl Add for Cost {
    type Output = Cost;

    fn add(self, o: Cost) -> Cost {
        self.saturating_add(o)
    }
}

/// The knobs negotiation turns. Factors are fixed point with 1000 = 1.0.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct CostParams {
    /// Cost of one layer change.
    pub via: Cost,
    /// Extra cost per step against a layer's preferred direction.
    pub wrong_way_step: Cost,
    /// Multiplier on present congestion (PathFinder `p`), ×1000.
    pub present_factor: u32,
    /// Multiplier on accumulated history (PathFinder `h`), ×1000.
    pub history_factor: u32,
    /// Added to a cell's history each iteration it stays overused.
    pub history_increment: u16,
    /// Extra cost for a cell the tri-state contract marks `Near`, which
    /// needs an exact check before it can be used.
    pub near_step: Cost,
}

impl Default for CostParams {
    fn default() -> Self {
        CostParams {
            via: Cost(8_000),
            wrong_way_step: Cost(500),
            present_factor: 1_000,
            history_factor: 1_000,
            history_increment: 1,
            near_step: Cost(100),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn infinite_absorbs() {
        assert_eq!(Cost::INFINITE + Cost::STEP_STRAIGHT, Cost::INFINITE);
        assert_eq!(Cost(u32::MAX - 1) + Cost(5), Cost::INFINITE);
        assert_eq!(Cost::INFINITE.scale(1, 2), Cost::INFINITE);
    }

    #[test]
    fn scale_is_fixed_point() {
        assert_eq!(Cost::STEP_STRAIGHT.scale(1500, 1000), Cost(1500));
        assert_eq!(Cost(u32::MAX - 1).scale(3, 1), Cost::INFINITE);
        assert_eq!(Cost(10).scale(1, 0), Cost::INFINITE);
    }
}
