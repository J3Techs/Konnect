//! The rayon raster backend: the production CPU path and the oracle for
//! every GPU kernel.
//!
//! Work is split by (layer, band of rows), so each rayon task owns a
//! disjoint slice of the raster and needs no atomics. Stamps are binned into
//! the bands their bounding boxes touch. Saturating ±1 updates of one op
//! commute, so the result does not depend on the order tasks or stamps run
//! in, which is what lets the GPU's scatter-with-atomics agree bit for bit.
//!
//! Two fills find a stamp's cells, and they agree exactly:
//!
//! - [`CpuFill::Span`] (the default) is the scanline span fill of gpu.md:
//!   per row, the covered cells are one interval (a capsule is convex), so
//!   the row costs a float estimate of that interval, a few exact
//!   `covers` tests to pin its ends, and a word-wide SWAR fill.
//! - [`CpuFill::BoundingBox`] tests every cell of the bounding box, which is
//!   what the WGSL kernel does. It is kept as a second oracle and as the
//!   per-cell reference in benchmarks.

use rayon::prelude::*;

use crate::layout::{prepare_stamps, CounterGrid, PaintOp, PreparedStamp, Q4_PER_CELL};
use crate::{ComputeError, PaintReport, RasterCompute};

/// Rows per rayon task. 16 rows of a 2048-cell tile are 8 KiB of counters,
/// small enough that a busy band stays in L1/L2 while its stamps run.
const BAND_ROWS: u32 = 16;

/// One in the low bit of every byte: the full four-cell mask.
const ALL_CELLS: u32 = 0x0101_0101;

/// Extra radius, in Q4 units, of the float span estimate. Float error on
/// these magnitudes is below 1e-9 units, so the padded estimate always
/// contains the exact interval; the exact tests then trim it.
const SPAN_SLACK_Q4: f64 = 1.0;

/// How [`CpuRaster`] finds the cells a stamp covers.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum CpuFill {
    /// Exact per-row intervals and a word-wide fill.
    #[default]
    Span,
    /// Every cell of the bounding box tested on its own, like the GPU.
    BoundingBox,
}

/// The CPU raster backend. It uses rayon's global pool.
#[derive(Clone, Copy, Debug, Default)]
pub struct CpuRaster {
    fill: CpuFill,
}

impl CpuRaster {
    /// The span-fill backend.
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_fill(fill: CpuFill) -> Self {
        Self { fill }
    }

    pub fn fill(&self) -> CpuFill {
        self.fill
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
        let fill = self.fill;
        let clamped_events = grid
            .words_mut()
            .par_chunks_mut(dims.plane_words())
            .enumerate()
            .map(|(layer, plane)| {
                plane
                    .par_chunks_mut(band_words)
                    .enumerate()
                    .map(|(band, words)| {
                        let bin = &bins[layer * bands_per_layer + band];
                        let band = Band {
                            words,
                            y_start: band as u32 * BAND_ROWS,
                            row_words,
                        };
                        match fill {
                            CpuFill::Span => paint_band_spans(band, bin, prepared, op),
                            CpuFill::BoundingBox => paint_band_cells(band, bin, prepared, op),
                        }
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

/// One rayon task's rows: `words` holds whole rows from `y_start` down.
struct Band<'a> {
    words: &'a mut [u32],
    y_start: u32,
    row_words: usize,
}

impl Band<'_> {
    fn y_end(&self) -> u32 {
        self.y_start + (self.words.len() / self.row_words) as u32 - 1
    }

    fn row(&mut self, y: u32) -> &mut [u32] {
        let base = (y - self.y_start) as usize * self.row_words;
        &mut self.words[base..base + self.row_words]
    }
}

/// Paint the stamps of one bin into one band, a row interval at a time.
fn paint_band_spans(mut band: Band, bin: &[u32], prepared: &[PreparedStamp], op: PaintOp) -> u64 {
    let y_end = band.y_end();
    let mut clamped = 0u64;
    for &index in bin {
        let stamp = &prepared[index as usize];
        for y in stamp.y0.max(band.y_start)..=stamp.y1.min(y_end) {
            if let Some((first, last)) = row_span(stamp, y) {
                clamped += fill_span(band.row(y), first, last, op);
            }
        }
    }
    clamped
}

/// Paint the stamps of one bin into one band, testing every cell of each
/// bounding box, as the WGSL kernel does.
fn paint_band_cells(mut band: Band, bin: &[u32], prepared: &[PreparedStamp], op: PaintOp) -> u64 {
    let y_end = band.y_end();
    let mut clamped = 0u64;
    for &index in bin {
        let stamp = &prepared[index as usize];
        for y in stamp.y0.max(band.y_start)..=stamp.y1.min(y_end) {
            let row = band.row(y);
            for word_x in stamp.x0 / 4..=stamp.x1 / 4 {
                let mask = coverage_mask(stamp, word_x, y);
                if mask != 0 {
                    let slot = &mut row[word_x as usize];
                    let (updated, word_clamped) = apply_mask(*slot, mask, op);
                    *slot = updated;
                    clamped += u64::from(word_clamped);
                }
            }
        }
    }
    clamped
}

/// The cells of row `y` that `stamp` covers, as an inclusive interval, or
/// `None`. Exactly the cells [`coverage_mask`] would select: the float
/// estimate is padded to a superset, and [`covers`] trims each end. A row
/// of a convex capsule has no gaps, so trimming the ends is enough.
pub(crate) fn row_span(stamp: &PreparedStamp, y: u32) -> Option<(u32, u32)> {
    let vx = stamp.bx - stamp.ax;
    let vy = stamp.by - stamp.ay;
    let l2 = vx * vx + vy * vy;
    let r = stamp.radius_q4 as i32;
    let r2 = r * r;
    let dy = y as i32 * Q4_PER_CELL + Q4_PER_CELL / 2 - stamp.ay;
    let covered = |x: u32| {
        covers(
            x as i32 * Q4_PER_CELL + Q4_PER_CELL / 2 - stamp.ax,
            dy,
            vx,
            vy,
            l2,
            r2,
        )
    };

    let (estimate_first, estimate_last) = span_estimate(stamp, dy)?;
    let mut first = estimate_first;
    while !covered(first) {
        if first == estimate_last {
            return None;
        }
        first += 1;
    }
    let mut last = estimate_last;
    while !covered(last) {
        last -= 1;
    }
    Some((first, last))
}

/// A superset of the covered cells of the row at offset `dy` (Q4, relative
/// to the stamp's start), clamped to its bounding box. The capsule's row
/// section is the hull of its two end discs' chords and its body's, where
/// the body is `0 <= d·v <= |v|²` and `|d × v| <= r|v|`.
fn span_estimate(stamp: &PreparedStamp, dy: i32) -> Option<(u32, u32)> {
    let r = f64::from(stamp.radius_q4) + SPAN_SLACK_Q4;
    let dy = f64::from(dy);
    let vx = f64::from(stamp.bx - stamp.ax);
    let vy = f64::from(stamp.by - stamp.ay);

    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    let mut include = |a: f64, b: f64| {
        if a <= b {
            lo = lo.min(a);
            hi = hi.max(b);
        }
    };
    for (cx, cy) in [(0.0, 0.0), (vx, vy)] {
        let ey = dy - cy;
        let h2 = r * r - ey * ey;
        if h2 >= 0.0 {
            let h = h2.sqrt();
            include(cx - h, cx + h);
        }
    }
    let l2 = vx * vx + vy * vy;
    if l2 > 0.0 {
        let reach = r * l2.sqrt();
        if let (Some(along), Some(across)) = (
            linear_range(vx, dy * vy, 0.0, l2),
            linear_range(vy, -dy * vx, -reach, reach),
        ) {
            include(along.0.max(across.0), along.1.min(across.1));
        }
    }
    if lo > hi {
        return None;
    }

    // Cell x has its centre at dx = 16x + 8 - ax.
    let to_cell = |dx: f64| (dx + f64::from(stamp.ax) - f64::from(Q4_PER_CELL / 2)) / 16.0;
    let first = to_cell(lo).ceil().max(f64::from(stamp.x0));
    let last = to_cell(hi).floor().min(f64::from(stamp.x1));
    (first <= last).then_some((first as u32, last as u32))
}

/// The `x` with `lo <= a·x + c <= hi`, as a closed interval, or `None`.
fn linear_range(a: f64, c: f64, lo: f64, hi: f64) -> Option<(f64, f64)> {
    if a == 0.0 {
        return (lo <= c && c <= hi).then_some((f64::NEG_INFINITY, f64::INFINITY));
    }
    let (p, q) = ((lo - c) / a, (hi - c) / a);
    Some((p.min(q), p.max(q)))
}

/// Apply `op` to cells `first..=last` of one row. Returns the clamp count.
fn fill_span(row: &mut [u32], first: u32, last: u32, op: PaintOp) -> u64 {
    let (w0, w1) = ((first / 4) as usize, (last / 4) as usize);
    let head = ALL_CELLS << (8 * (first % 4));
    let tail = ALL_CELLS >> (8 * (3 - last % 4));
    let apply = |slot: &mut u32, mask: u32| {
        let (updated, clamped) = apply_mask(*slot, mask, op);
        *slot = updated;
        u64::from(clamped)
    };
    if w0 == w1 {
        return apply(&mut row[w0], head & tail);
    }
    let mut clamped = apply(&mut row[w0], head);
    for slot in &mut row[w0 + 1..w1] {
        clamped += apply(slot, ALL_CELLS);
    }
    clamped + apply(&mut row[w1], tail)
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

/// Apply `op` to the bytes of `word` selected by `mask` (bit `8k` for byte
/// `k`), saturating. Returns the new word and the number of bytes that were
/// already at the limit. Branch-free SWAR: the step skips limit bytes, so
/// no byte carries or borrows into its neighbour.
pub(crate) fn apply_mask(word: u32, mask: u32, op: PaintOp) -> (u32, u32) {
    let at_limit = match op {
        PaintOp::Paint => zero_bytes(!word),
        PaintOp::Unpaint => zero_bytes(word),
    } & mask;
    let step = mask & !at_limit;
    let updated = match op {
        PaintOp::Paint => word + step,
        PaintOp::Unpaint => word - step,
    };
    (updated, at_limit.count_ones())
}

/// Bit `8k` set for each byte `k` of `x` that is zero, exactly (the
/// carry-free form, with no false positives above a zero byte).
fn zero_bytes(x: u32) -> u32 {
    !(((x & 0x7F7F_7F7F) + 0x7F7F_7F7F) | x | 0x7F7F_7F7F) >> 7
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
    fn both_fills_match_the_brute_force_oracle() {
        for fill in [CpuFill::Span, CpuFill::BoundingBox] {
            let mut raster = CpuRaster::with_fill(fill);
            let mut rng = SplitMix64::new(0x5EED);
            for case in 0..12 {
                let width = 8 + rng.below(70) as u32;
                let height = 8 + rng.below(70) as u32;
                let layers = 1 + rng.below(4) as u32;
                let dims = RasterDims::new(width, height, layers).unwrap();
                let mut stamps = synthetic_stamps(dims, 40, 1000 + case);
                stamps.extend(any_angle_stamps(dims, 40, 2000 + case));

                let mut expected = CounterGrid::new(dims);
                let mut actual = CounterGrid::new(dims);
                let expected_clamped = oracle_paint(&mut expected, &stamps, PaintOp::Paint);
                let report = raster.paint(&mut actual, &stamps, PaintOp::Paint).unwrap();
                assert_eq!(actual, expected, "{fill:?} paint, case {case}");
                assert_eq!(report.clamped_events, expected_clamped, "{fill:?} {case}");

                let half = &stamps[..stamps.len() / 2];
                oracle_paint(&mut expected, half, PaintOp::Unpaint);
                raster.paint(&mut actual, half, PaintOp::Unpaint).unwrap();
                assert_eq!(actual, expected, "{fill:?} unpaint, case {case}");
            }
        }
    }

    /// Segments at any angle, radii from zero to the limit and spans up to
    /// the limit, starting anywhere near the raster: the cases octilinear
    /// tracks never reach.
    fn any_angle_stamps(dims: RasterDims, count: usize, seed: u64) -> Vec<Stamp> {
        use crate::layout::{MAX_RADIUS_Q4, MAX_SEGMENT_SPAN_Q4};
        let mut rng = SplitMix64::new(seed);
        let span = 2 * MAX_SEGMENT_SPAN_Q4 as u64 + 1;
        let coord =
            |rng: &mut SplitMix64, cells: u32| rng.below(u64::from(cells) * 16 + 512) as i32 - 256;
        (0..count)
            .map(|_| {
                let a = [coord(&mut rng, dims.width), coord(&mut rng, dims.height)];
                let b = [
                    a[0] + rng.below(span) as i32 - MAX_SEGMENT_SPAN_Q4,
                    a[1] + rng.below(span) as i32 - MAX_SEGMENT_SPAN_Q4,
                ];
                let radius = match rng.below(4) {
                    0 => rng.below(17) as u32,
                    1 => MAX_RADIUS_Q4 - rng.below(3) as u32,
                    _ => rng.below(u64::from(MAX_RADIUS_Q4) / 8) as u32,
                };
                let layer = rng.below(u64::from(dims.layers)) as u32;
                if rng.below(4) == 0 {
                    Stamp::circle(a, radius, layer, layer)
                } else {
                    Stamp::segment(a, b, radius, layer)
                }
            })
            .collect()
    }

    #[test]
    fn span_fill_matches_the_cell_fill_on_arbitrary_geometry() {
        let span = CpuRaster::with_fill(CpuFill::Span);
        let cells = CpuRaster::with_fill(CpuFill::BoundingBox);
        for case in 0..12u64 {
            let dims = RasterDims::new(97 + 31 * case as u32, 211, 2).unwrap();
            let stamps = any_angle_stamps(dims, 60, 0xA11A9 + case);
            let prepared = prepare_stamps(dims, &stamps).unwrap();
            let mut by_span = CounterGrid::new(dims);
            let mut by_cell = CounterGrid::new(dims);
            let s = span.paint_prepared(&mut by_span, &prepared, PaintOp::Paint);
            let c = cells.paint_prepared(&mut by_cell, &prepared, PaintOp::Paint);
            assert_eq!(by_span, by_cell, "case {case}");
            assert_eq!(s, c, "case {case}");
        }
    }

    #[test]
    fn span_fill_handles_exact_boundary_and_single_cell_stamps() {
        let dims = RasterDims::new(9, 9, 1).unwrap();
        // Radius zero on a centre covers exactly that cell; radius 16 between
        // centres reaches the neighbours' centres exactly (boundary
        // included); a segment along a row of centres with radius zero
        // covers just that row's cells.
        let stamps = [
            Stamp::circle([8, 8], 0, 0, 0),
            Stamp::circle([72, 72], 16, 0, 0),
            Stamp::segment([24, 136], [120, 136], 0, 0),
            Stamp::segment([8, 40], [136, 104], 0, 0),
        ];
        let mut expected = CounterGrid::new(dims);
        let clamped = oracle_paint(&mut expected, &stamps, PaintOp::Paint);
        let mut actual = CounterGrid::new(dims);
        let report = CpuRaster::new()
            .paint(&mut actual, &stamps, PaintOp::Paint)
            .unwrap();
        assert_eq!(actual, expected);
        assert_eq!(report.clamped_events, clamped);
        assert_eq!(actual.get(0, 0, 0), 1);
        assert_eq!(actual.get(1, 0, 0), 0);
        assert_eq!(actual.get(4, 3, 0), 1);
        assert_eq!(actual.get(3, 3, 0), 0);
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

    #[test]
    fn swar_apply_mask_matches_a_per_byte_loop() {
        let mut rng = SplitMix64::new(0x5A4A);
        // Bias bytes towards the limits, where the carries would go wrong.
        let byte = |rng: &mut SplitMix64| match rng.below(4) {
            0 => 0,
            1 => 255,
            2 => [1, 127, 128, 254][rng.below(4) as usize],
            _ => rng.below(256) as u8,
        };
        for _ in 0..200_000 {
            let word = u32::from_le_bytes([
                byte(&mut rng),
                byte(&mut rng),
                byte(&mut rng),
                byte(&mut rng),
            ]);
            let selected = rng.below(16) as u32;
            let mask = (0..4).fold(0, |m, k| m | (((selected >> k) & 1) << (8 * k)));
            for op in [PaintOp::Paint, PaintOp::Unpaint] {
                let mut bytes = word.to_le_bytes();
                let mut clamped = 0;
                for (k, b) in bytes.iter_mut().enumerate() {
                    if (mask >> (8 * k)) & 1 == 0 {
                        continue;
                    }
                    match op {
                        PaintOp::Paint if *b == 255 => clamped += 1,
                        PaintOp::Paint => *b += 1,
                        PaintOp::Unpaint if *b == 0 => clamped += 1,
                        PaintOp::Unpaint => *b -= 1,
                    }
                }
                assert_eq!(
                    apply_mask(word, mask, op),
                    (u32::from_le_bytes(bytes), clamped),
                    "{word:#010x} {mask:#010x} {op:?}"
                );
            }
        }
    }
}
