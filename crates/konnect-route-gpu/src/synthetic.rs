//! Deterministic synthetic stamp workloads for parity tests, the smoke
//! kernel and the microbenchmarks. Nothing here comes from a real board.

use crate::layout::{RasterDims, Stamp, MAX_SEGMENT_SPAN_Q4, Q4_PER_CELL};

/// SplitMix64: tiny, seedable and identical on every platform, so a seed
/// names the same workload everywhere.
#[derive(Clone, Debug)]
pub struct SplitMix64(u64);

impl SplitMix64 {
    pub fn new(seed: u64) -> Self {
        Self(seed)
    }

    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in `0..bound` (`bound` > 0); the modulo bias is irrelevant
    /// for workload generation.
    pub fn below(&mut self, bound: u64) -> u64 {
        self.next_u64() % bound
    }
}

/// Octilinear unit steps, the headings a routed track can take.
const HEADINGS: [[i32; 2]; 8] = [
    [1, 0],
    [1, 1],
    [0, 1],
    [-1, 1],
    [-1, 0],
    [-1, -1],
    [0, -1],
    [1, -1],
];

/// A routing-like mix: about 80% single-layer octilinear track segments up
/// to 255 cells long and 2-6 cells in radius, 20% through vias spanning all
/// layers. Coordinates carry sub-cell offsets and may start up to a few
/// cells outside the raster, so clipping is exercised too.
pub fn synthetic_stamps(dims: RasterDims, count: usize, seed: u64) -> Vec<Stamp> {
    let mut rng = SplitMix64::new(seed);
    let q4 = Q4_PER_CELL as u64;
    let margin = 4 * q4;
    let span_x = u64::from(dims.width) * q4 + 2 * margin;
    let span_y = u64::from(dims.height) * q4 + 2 * margin;
    (0..count)
        .map(|_| {
            let x = rng.below(span_x) as i32 - margin as i32;
            let y = rng.below(span_y) as i32 - margin as i32;
            if rng.below(5) == 0 {
                let radius = 4 * Q4_PER_CELL as u32 + rng.below(4 * q4) as u32;
                Stamp::circle([x, y], radius, 0, dims.layers - 1)
            } else {
                let heading = HEADINGS[rng.below(8) as usize];
                let max_cells = (MAX_SEGMENT_SPAN_Q4 / Q4_PER_CELL) as u64 - 1;
                let length = (4 + rng.below(max_cells - 4)) as i32 * Q4_PER_CELL;
                let radius = 2 * Q4_PER_CELL as u32 + rng.below(4 * q4) as u32;
                let layer = rng.below(u64::from(dims.layers)) as u32;
                let end = [x + heading[0] * length, y + heading[1] * length];
                Stamp::segment([x, y], end, radius, layer)
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::layout::prepare_stamps;

    #[test]
    fn workloads_are_reproducible_and_within_the_layout_contract() {
        let dims = RasterDims::new(512, 384, 3).unwrap();
        let first = synthetic_stamps(dims, 5_000, 42);
        assert_eq!(first, synthetic_stamps(dims, 5_000, 42));
        assert_ne!(first, synthetic_stamps(dims, 5_000, 43));
        let prepared = prepare_stamps(dims, &first).unwrap();
        assert!(prepared.len() > 4_500, "too many stamps missed the raster");
        assert!(first.iter().any(|s| s.layer_first != s.layer_last));
    }

    #[test]
    fn splitmix_matches_its_reference_sequence() {
        // First outputs of SplitMix64 seeded with 0 (Vigna's reference).
        let mut rng = SplitMix64::new(0);
        assert_eq!(rng.next_u64(), 0xE220_A839_7B1D_CDAF);
        assert_eq!(rng.next_u64(), 0x6E78_9E6A_A1B9_65F4);
    }
}
