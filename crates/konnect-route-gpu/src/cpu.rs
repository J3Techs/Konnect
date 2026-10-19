//! The rayon reference raster backend: the oracle for every GPU kernel.
//!
//! Work is split by (layer, band of rows), so each rayon task owns a
//! disjoint slice of the raster and needs no atomics. Stamps are binned into
//! the bands their bounding boxes touch. Saturating ±1 updates of one op
//! commute, so the result does not depend on the order tasks or stamps run
//! in, which is what lets the GPU's scatter-with-atomics agree bit for bit.

use rayon::prelude::*;

use crate::layout::{prepare_stamps, CounterGrid, PaintOp, PreparedStamp, Q4_PER_CELL};
use crate::{ComputeError, PaintReport, RasterCompute};

/// Rows per rayon task. 16 rows of a 2048-cell tile are 8 KiB of counters,
/// small enough that a busy band stays in L1/L2 while its stamps run.
const BAND_ROWS: u32 = 16;

/// The CPU raster backend. It uses rayon's global pool.
#[derive(Clone, Copy, Debug, Default)]
pub struct CpuRaster;

impl CpuRaster {
    pub fn new() -> Self {
        Self
    }

    /// Paint already-validated stamps. [`RasterCompute::paint`] is this plus
    /// [`prepare_stamps`].
    pub fn paint_prepared(
        &self,
        grid: &mut CounterGrid,
        prepared: &[PreparedStamp],
        op: PaintOp,
    ) -> PaintReport {
        let dims = grid.dims();
        let row_words = dims.row_words() as usize;
        let bands_per_layer = dims.height.div_ceil(BAND_ROWS) as usize;

        let mut bins: Vec<Vec<u32>> = vec![Vec::new(); dims.layers as usize * bands_per_layer];
        for (index, stamp) in prepared.iter().enumerate() {
            for layer in stamp.layer_first..=stamp.layer_last {
                for band in stamp.y0 / BAND_ROWS..=stamp.y1 / BAND_ROWS {
                    bins[layer as usize * bands_per_layer + band as usize].push(index as u32);
                }
            }
        }

        let band_words = BAND_ROWS as usize * row_words;
        let bins = &bins;
        let clamped_events = grid
            .words_mut()
            .par_chunks_mut(dims.plane_words())
            .enumerate()
            .map(|(layer, plane)| {
                plane
                    .par_chunks_mut(band_words)
                    .enumerate()
                    .map(|(band, words)| {
                        paint_band(
                            words,
                            band as u32 * BAND_ROWS,
                            row_words,
                            &bins[layer * bands_per_layer + band],
                            prepared,
                            op,
                        )
                    })
                    .sum::<u64>()
            })
            .sum();
        PaintReport { clamped_events }
    }
}

impl RasterCompute for CpuRaster {
    fn backend_name(&self) -> &'static str {
        "cpu"
    }

    fn paint(
        &mut self,
        grid: &mut CounterGrid,
        stamps: &[crate::Stamp],
        op: PaintOp,
    ) -> Result<PaintReport, ComputeError> {
        let prepared = prepare_stamps(grid.dims(), stamps)?;
        Ok(self.paint_prepared(grid, &prepared, op))
    }
}

/// Paint the stamps of one bin into one band of rows starting at `y_start`.
fn paint_band(
    words: &mut [u32],
    y_start: u32,
    row_words: usize,
    bin: &[u32],
    prepared: &[PreparedStamp],
    op: PaintOp,
) -> u64 {
    let y_end = y_start + (words.len() / row_words) as u32 - 1;
    let mut clamped = 0u64;
    for &index in bin {
        let stamp = &prepared[index as usize];
        for y in stamp.y0.max(y_start)..=stamp.y1.min(y_end) {
            let row_base = (y - y_start) as usize * row_words;
            for word_x in stamp.x0 / 4..=stamp.x1 / 4 {
                let mask = coverage_mask(stamp, word_x, y);
                if mask != 0 {
                    let slot = &mut words[row_base + word_x as usize];
                    let (updated, word_clamped) = apply_mask(*slot, mask, op);
                    *slot = updated;
                    clamped += u64::from(word_clamped);
                }
            }
        }
    }
    clamped
}

/// One bit per byte (bit `8k` for cell `4 * word_x + k`) for the cells of
/// one raster word that `stamp` covers.
pub(crate) fn coverage_mask(stamp: &PreparedStamp, word_x: u32, y: u32) -> u32 {
    let vx = stamp.bx - stamp.ax;
    let vy = stamp.by - stamp.ay;
    let l2 = vx * vx + vy * vy;
    let r = stamp.radius_q4 as i32;
    let r2 = r * r;
    let dy = y as i32 * Q4_PER_CELL + Q4_PER_CELL / 2 - stamp.ay;
    let mut mask = 0;
    for k in 0..4 {
        let x = word_x * 4 + k;
        if x < stamp.x0 || x > stamp.x1 {
            continue;
        }
        let dx = x as i32 * Q4_PER_CELL + Q4_PER_CELL / 2 - stamp.ax;
        if covers(dx, dy, vx, vy, l2, r2) {
            mask |= 1 << (8 * k);
        }
    }
    mask
}

/// Whether the point `d` (relative to the segment start) lies within the
/// capsule of radius √`r2` around the segment `0..v`, boundary included.
/// `l2` is |v|². The layout bounds keep every i32 product below 2^31.
pub(crate) fn covers(dx: i32, dy: i32, vx: i32, vy: i32, l2: i32, r2: i32) -> bool {
    let t = dx * vx + dy * vy;
    if t <= 0 {
        return dx * dx + dy * dy <= r2;
    }
    if t >= l2 {
        let ex = dx - vx;
        let ey = dy - vy;
        return ex * ex + ey * ey <= r2;
    }
    // Perpendicular: dist² = cross² / l2, compared without division.
    let cross = u64::from((dx * vy - dy * vx).unsigned_abs());
    cross * cross <= r2 as u64 * l2 as u64
}

/// Apply `op` to the bytes of `word` selected by `mask`, saturating.
/// Returns the new word and the number of bytes that were already at the
/// limit.
pub(crate) fn apply_mask(word: u32, mask: u32, op: PaintOp) -> (u32, u32) {
    let mut updated = word;
    let mut clamped = 0;
    for k in 0..4 {
        let shift = 8 * k;
        if (mask >> shift) & 1 == 0 {
            continue;
        }
        let cell = (word >> shift) & 0xFF;
        match op {
            PaintOp::Paint if cell == 0xFF => clamped += 1,
            PaintOp::Paint => updated += 1 << shift,
            PaintOp::Unpaint if cell == 0 => clamped += 1,
            PaintOp::Unpaint => updated -= 1 << shift,
        }
    }
    (updated, clamped)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::layout::{RasterDims, Stamp};
    use crate::synthetic::{synthetic_stamps, SplitMix64};

    /// Brute force: every stamp against every cell in i128, no bounding box,
    /// no binning, no packed words.
    fn oracle_paint(grid: &mut CounterGrid, stamps: &[Stamp], op: PaintOp) -> u64 {
        let dims = grid.dims();
        let mut clamped = 0;
        for stamp in stamps {
            let (ax, ay) = (i128::from(stamp.a[0]), i128::from(stamp.a[1]));
            let (vx, vy) = (i128::from(stamp.b[0]) - ax, i128::from(stamp.b[1]) - ay);
            let l2 = vx * vx + vy * vy;
            let r2 = i128::from(stamp.radius_q4).pow(2);
            for layer in stamp.layer_first..=stamp.layer_last {
                for y in 0..dims.height {
                    for x in 0..dims.width {
                        let dx = i128::from(x) * 16 + 8 - ax;
                        let dy = i128::from(y) * 16 + 8 - ay;
                        let t = dx * vx + dy * vy;
                        let inside = if t <= 0 {
                            dx * dx + dy * dy <= r2
                        } else if t >= l2 {
                            (dx - vx).pow(2) + (dy - vy).pow(2) <= r2
                        } else {
                            (dx * vy - dy * vx).pow(2) <= r2 * l2
                        };
                        if !inside {
                            continue;
                        }
                        let cell = grid.get(x, y, layer);
                        match op {
                            PaintOp::Paint if cell == 255 => clamped += 1,
                            PaintOp::Paint => grid.set(x, y, layer, cell + 1),
                            PaintOp::Unpaint if cell == 0 => clamped += 1,
                            PaintOp::Unpaint => grid.set(x, y, layer, cell - 1),
                        }
                    }
                }
            }
        }
        clamped
    }

    fn paint(grid: &mut CounterGrid, stamps: &[Stamp], op: PaintOp) -> u64 {
        CpuRaster::new()
            .paint(grid, stamps, op)
            .unwrap()
            .clamped_events
    }

    #[test]
    fn a_one_cell_disc_at_a_centre_covers_the_plus_shape() {
        let dims = RasterDims::new(5, 5, 1).unwrap();
        let mut grid = CounterGrid::new(dims);
        // Centre of cell (2, 2); radius exactly one cell reaches the four
        // edge neighbours' centres, not the diagonal ones.
        paint(
            &mut grid,
            &[Stamp::circle([40, 40], 16, 0, 0)],
            PaintOp::Paint,
        );
        let covered: Vec<(u32, u32)> = (0..5)
            .flat_map(|y| (0..5).map(move |x| (x, y)))
            .filter(|&(x, y)| grid.get(x, y, 0) == 1)
            .collect();
        assert_eq!(covered, vec![(2, 1), (1, 2), (2, 2), (3, 2), (2, 3)]);
    }

    #[test]
    fn a_through_via_paints_every_layer_in_its_range() {
        let dims = RasterDims::new(4, 4, 4).unwrap();
        let mut grid = CounterGrid::new(dims);
        paint(
            &mut grid,
            &[Stamp::circle([24, 24], 4, 1, 2)],
            PaintOp::Paint,
        );
        assert_eq!(grid.get(1, 1, 0), 0);
        assert_eq!(grid.get(1, 1, 1), 1);
        assert_eq!(grid.get(1, 1, 2), 1);
        assert_eq!(grid.get(1, 1, 3), 0);
    }

    #[test]
    fn paint_saturates_at_255_and_counts_the_clamps() {
        let dims = RasterDims::new(3, 1, 1).unwrap();
        let mut grid = CounterGrid::new(dims);
        let stamps = vec![Stamp::circle([24, 8], 4, 0, 0); 300];
        assert_eq!(paint(&mut grid, &stamps, PaintOp::Paint), 45);
        assert_eq!(grid.get(1, 0, 0), 255);
        assert_eq!(grid.get(0, 0, 0), 0);
        assert_eq!(paint(&mut grid, &stamps, PaintOp::Unpaint), 45);
        assert_eq!(grid.get(1, 0, 0), 0);
    }

    #[test]
    fn padding_cells_past_the_width_stay_zero() {
        let dims = RasterDims::new(5, 2, 1).unwrap();
        let mut grid = CounterGrid::new(dims);
        paint(
            &mut grid,
            &[Stamp::circle([80, 16], 200, 0, 0)],
            PaintOp::Paint,
        );
        for row in grid.words().chunks(2) {
            assert_eq!(row[1] & 0xFFFF_FF00, 0, "padding bytes were painted");
        }
        assert_eq!(grid.occupied_cells(), 10);
    }

    #[test]
    fn rayon_twin_matches_the_brute_force_oracle() {
        let mut rng = SplitMix64::new(0x5EED);
        for case in 0..12 {
            let width = 8 + rng.below(70) as u32;
            let height = 8 + rng.below(70) as u32;
            let layers = 1 + rng.below(4) as u32;
            let dims = RasterDims::new(width, height, layers).unwrap();
            let stamps = synthetic_stamps(dims, 40, 1000 + case);

            let mut expected = CounterGrid::new(dims);
            let mut actual = CounterGrid::new(dims);
            let expected_clamped = oracle_paint(&mut expected, &stamps, PaintOp::Paint);
            let actual_clamped = paint(&mut actual, &stamps, PaintOp::Paint);
            assert_eq!(actual, expected, "paint, case {case}");
            assert_eq!(actual_clamped, expected_clamped);

            let half = &stamps[..stamps.len() / 2];
            oracle_paint(&mut expected, half, PaintOp::Unpaint);
            paint(&mut actual, half, PaintOp::Unpaint);
            assert_eq!(actual, expected, "unpaint, case {case}");
        }
    }

    #[test]
    fn unpaint_inverts_paint_below_saturation() {
        let dims = RasterDims::new(300, 200, 3).unwrap();
        let stamps = synthetic_stamps(dims, 2_000, 7);
        let mut grid = CounterGrid::new(dims);
        let raster = CpuRaster::new();
        let prepared = prepare_stamps(dims, &stamps).unwrap();
        raster.paint_prepared(&mut grid, &prepared, PaintOp::Paint);
        assert!(grid.occupied_cells() > 0);
        let report = raster.paint_prepared(&mut grid, &prepared, PaintOp::Unpaint);
        assert_eq!(report.clamped_events, 0);
        assert_eq!(grid, CounterGrid::new(dims));
    }

    #[test]
    fn coverage_test_survives_its_extreme_bounds() {
        use crate::layout::{MAX_RADIUS_Q4, MAX_SEGMENT_SPAN_Q4};
        let span = MAX_SEGMENT_SPAN_Q4;
        let r = MAX_RADIUS_Q4 as i32;
        let (vx, vy) = (span, -span);
        let l2 = vx * vx + vy * vy;
        // Farthest bounding-box corners from the segment's interior and ends.
        for (dx, dy) in [
            (-r, r),
            (span + r, -span - r),
            (-r, -span - r),
            (span + r, r),
        ] {
            let expected = {
                let (dx, dy, vx, vy) = (dx as f64, dy as f64, vx as f64, vy as f64);
                let t = ((dx * vx + dy * vy) / (vx * vx + vy * vy)).clamp(0.0, 1.0);
                (dx - t * vx).hypot(dy - t * vy) <= r as f64
            };
            assert_eq!(covers(dx, dy, vx, vy, l2, r * r), expected, "({dx}, {dy})");
        }
        assert!(covers(span / 2 + 1, -span / 2, vx, vy, l2, r * r));
    }

    #[test]
    fn apply_mask_touches_only_selected_bytes() {
        let word = u32::from_le_bytes([0, 255, 7, 1]);
        let (painted, clamped) = apply_mask(word, 0x0101_0101, PaintOp::Paint);
        assert_eq!(painted.to_le_bytes(), [1, 255, 8, 2]);
        assert_eq!(clamped, 1);
        let (unpainted, clamped) = apply_mask(word, 0x0000_0101, PaintOp::Unpaint);
        assert_eq!(unpainted.to_le_bytes(), [0, 254, 7, 1]);
        assert_eq!(clamped, 1);
    }
}
