//! The GPU-shaped data layout shared by every backend.
//!
//! All of it is integer so the CPU and GPU kernels can be compared bit for
//! bit. The bounds below are not arbitrary: they are what keeps every
//! product in the coverage test inside i32 (see [`Stamp`]).

use std::ops::Range;

use crate::ComputeError;

/// Fixed-point subdivisions per lattice cell. Cell `(x, y)` has its centre at
/// `(Q4_PER_CELL * x + Q4_PER_CELL / 2, Q4_PER_CELL * y + Q4_PER_CELL / 2)`.
/// At a 50 µm pitch one unit is 3.125 µm; the raster is a conservative
/// screen in front of the exact i64 checks, not a DRC.
pub const Q4_PER_CELL: i32 = 16;

/// Largest |bx - ax| or |by - ay| of one stamp, in Q4 units (256 cells).
/// Longer tracks are split with [`Stamp::split_segment`].
pub const MAX_SEGMENT_SPAN_Q4: i32 = 4096;

/// Largest stamp radius in Q4 units (just under 128 cells).
pub const MAX_RADIUS_Q4: u32 = 2047;

/// Largest raster width or height in cells.
pub const MAX_GRID_SIDE_CELLS: u32 = 8192;

/// Largest number of layers in one raster.
pub const MAX_LAYERS: u32 = 32;

/// Stamp coordinates must lie within this many Q4 units of the origin. It
/// keeps the bounding-box arithmetic away from i32 overflow.
const MAX_ABS_COORD_Q4: i32 = 1 << 28;

/// Upper bound on stamps in one GPU dispatch: one workgroup per stamp, and
/// 65535 is the portable `max_compute_workgroups_per_dimension`.
pub const MAX_STAMPS_PER_DISPATCH: u32 = 65_535;

/// Upper bound on raster words visited by one GPU dispatch. Each visit is at
/// most four coverage tests and one compare-exchange; 4M visits is a few
/// milliseconds on a desktop GPU and stays far inside the 50 ms dispatch
/// budget even on a slow one. The largest legal stamp visits about 2.1M
/// words on 32 layers, so a single stamp always fits.
pub const MAX_WORK_WORDS_PER_DISPATCH: u64 = 1 << 22;

/// Cost of one straight 50 µm step in the router's u32 fixed-point model.
pub const COST_STRAIGHT: u32 = 1000;

/// Cost of one diagonal 50 µm step (√2 × [`COST_STRAIGHT`], rounded).
pub const COST_DIAGONAL: u32 = 1414;

/// Add two fixed-point costs, saturating instead of wrapping. u16 would
/// overflow on a 10 mm path; u32 saturates only on unreachable values.
pub fn cost_add(a: u32, b: u32) -> u32 {
    a.saturating_add(b)
}

/// Shape of one raster: `layers` planes of `height` rows of `width` cells.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RasterDims {
    pub width: u32,
    pub height: u32,
    pub layers: u32,
}

impl RasterDims {
    pub fn new(width: u32, height: u32, layers: u32) -> Result<Self, ComputeError> {
        if width == 0 || height == 0 || layers == 0 {
            return Err(ComputeError::InvalidInput(format!(
                "raster dimensions must be non-zero, got {width} x {height} x {layers}"
            )));
        }
        if width > MAX_GRID_SIDE_CELLS || height > MAX_GRID_SIDE_CELLS {
            return Err(ComputeError::InvalidInput(format!(
                "raster side must be at most {MAX_GRID_SIDE_CELLS} cells, got {width} x {height}"
            )));
        }
        if layers > MAX_LAYERS {
            return Err(ComputeError::InvalidInput(format!(
                "raster must have at most {MAX_LAYERS} layers, got {layers}"
            )));
        }
        Ok(Self {
            width,
            height,
            layers,
        })
    }

    /// u32 words per row: four u8 cells per word, rows padded to a word.
    pub fn row_words(&self) -> u32 {
        self.width.div_ceil(4)
    }

    /// Words in one layer plane.
    pub fn plane_words(&self) -> usize {
        self.height as usize * self.row_words() as usize
    }

    /// Words in the whole raster.
    pub fn total_words(&self) -> usize {
        self.layers as usize * self.plane_words()
    }

    /// Index of the word holding cell `(x, y)` on `layer`.
    pub fn word_index(&self, x: u32, y: u32, layer: u32) -> usize {
        layer as usize * self.plane_words()
            + y as usize * self.row_words() as usize
            + (x / 4) as usize
    }
}

/// A u8-per-cell counter raster stored as packed little-endian u32 words:
/// cell `x` lives in byte `x % 4` (bits `8 * (x % 4)..`) of word `x / 4`.
/// Padding cells past `width` are always zero.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CounterGrid {
    dims: RasterDims,
    words: Vec<u32>,
}

impl CounterGrid {
    /// An all-zero raster.
    pub fn new(dims: RasterDims) -> Self {
        Self {
            dims,
            words: vec![0; dims.total_words()],
        }
    }

    pub fn dims(&self) -> RasterDims {
        self.dims
    }

    pub fn words(&self) -> &[u32] {
        &self.words
    }

    /// Crate-private: a caller writing padding bytes would break the
    /// "padding cells are always zero" invariant that [`Self::occupied_cells`]
    /// and grid equality rely on.
    pub(crate) fn words_mut(&mut self) -> &mut [u32] {
        &mut self.words
    }

    /// The counter of cell `(x, y)` on `layer`. Panics when out of range.
    pub fn get(&self, x: u32, y: u32, layer: u32) -> u8 {
        assert!(x < self.dims.width && y < self.dims.height && layer < self.dims.layers);
        let word = self.words[self.dims.word_index(x, y, layer)];
        (word >> (8 * (x % 4))) as u8
    }

    /// Overwrite the counter of cell `(x, y)` on `layer`. Panics when out of
    /// range.
    pub fn set(&mut self, x: u32, y: u32, layer: u32, value: u8) {
        assert!(x < self.dims.width && y < self.dims.height && layer < self.dims.layers);
        let index = self.dims.word_index(x, y, layer);
        let shift = 8 * (x % 4);
        self.words[index] = (self.words[index] & !(0xFF << shift)) | (u32::from(value) << shift);
    }

    /// Number of non-zero cells.
    pub fn occupied_cells(&self) -> u64 {
        self.words
            .iter()
            .map(|word| word.to_le_bytes().iter().filter(|&&b| b != 0).count() as u64)
            .sum()
    }
}

/// Paint increments covered counters (saturating at 255); unpaint
/// decrements them (saturating at 0). One batch carries one op, which is
/// what makes the result independent of update order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PaintOp {
    Paint,
    Unpaint,
}

impl PaintOp {
    /// The value the WGSL kernel reads from its `op` parameter.
    pub fn code(self) -> u32 {
        match self {
            PaintOp::Paint => 0,
            PaintOp::Unpaint => 1,
        }
    }
}

/// One inflated primitive in tile-local Q4 coordinates: the capsule of all
/// points within `radius_q4` of the segment `a..b`, on every layer from
/// `layer_first` to `layer_last`. A circle (pad, via land) has `a == b`; a
/// through via spans several layers.
///
/// A cell is covered when its centre is within the capsule, boundary
/// included. With the span and radius bounds, every intermediate product of
/// that test fits i32 except the perpendicular case, which compares two
/// values below 2^52 and is done in u64 (two u32 halves on the GPU).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Stamp {
    pub a: [i32; 2],
    pub b: [i32; 2],
    pub radius_q4: u32,
    pub layer_first: u32,
    pub layer_last: u32,
}

impl Stamp {
    /// A capsule on a single layer.
    pub fn segment(a: [i32; 2], b: [i32; 2], radius_q4: u32, layer: u32) -> Self {
        Self {
            a,
            b,
            radius_q4,
            layer_first: layer,
            layer_last: layer,
        }
    }

    /// A disc on the layers `layer_first..=layer_last`.
    pub fn circle(centre: [i32; 2], radius_q4: u32, layer_first: u32, layer_last: u32) -> Self {
        Self {
            a: centre,
            b: centre,
            radius_q4,
            layer_first,
            layer_last,
        }
    }

    /// Split a single-layer capsule of any length into stamps that respect
    /// [`MAX_SEGMENT_SPAN_Q4`]. Pieces share their integer split points, so
    /// the union differs from the long capsule only by the rounding of those
    /// points (under one Q4 unit).
    pub fn split_segment(a: [i32; 2], b: [i32; 2], radius_q4: u32, layer: u32) -> Vec<Self> {
        let dx = i64::from(b[0]) - i64::from(a[0]);
        let dy = i64::from(b[1]) - i64::from(a[1]);
        let span = dx.abs().max(dy.abs());
        let pieces = (span.div_euclid(i64::from(MAX_SEGMENT_SPAN_Q4)) + 1).max(1);
        let point = |i: i64| {
            [
                (i64::from(a[0]) + dx * i / pieces) as i32,
                (i64::from(a[1]) + dy * i / pieces) as i32,
            ]
        };
        (0..pieces)
            .map(|i| Self::segment(point(i), point(i + 1), radius_q4, layer))
            .collect()
    }
}

/// A validated stamp with its cell bounding box clamped to the raster, in
/// the exact 48-byte layout of the WGSL `Stamp` struct. Both backends paint
/// from this form; the bounding box only limits which cells are tested, the
/// coverage test itself is computed independently by each backend.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "gpu", derive(bytemuck::Pod, bytemuck::Zeroable))]
pub struct PreparedStamp {
    pub ax: i32,
    pub ay: i32,
    pub bx: i32,
    pub by: i32,
    pub radius_q4: u32,
    pub layer_first: u32,
    pub layer_last: u32,
    /// Inclusive cell bounding box, clamped to the raster.
    pub x0: u32,
    pub y0: u32,
    pub x1: u32,
    pub y1: u32,
    pub pad0: u32,
}

impl PreparedStamp {
    /// Raster words this stamp's bounding box spans over all its layers:
    /// the GPU kernel's loop count for it.
    pub fn work_words(&self) -> u64 {
        let words_per_row = u64::from(self.x1 / 4 - self.x0 / 4 + 1);
        let rows = u64::from(self.y1 - self.y0 + 1);
        let layers = u64::from(self.layer_last - self.layer_first + 1);
        words_per_row * rows * layers
    }
}

/// Lowest cell index whose centre is at or above `q4`.
fn first_cell_at_or_above(q4: i32) -> i32 {
    let half = Q4_PER_CELL / 2;
    -(-(q4 - half)).div_euclid(Q4_PER_CELL)
}

/// Highest cell index whose centre is at or below `q4`.
fn last_cell_at_or_below(q4: i32) -> i32 {
    let half = Q4_PER_CELL / 2;
    (q4 - half).div_euclid(Q4_PER_CELL)
}

/// Validate stamps against the layout bounds and clamp their bounding boxes
/// to `dims`. Stamps that cover no cell of the raster are dropped.
pub fn prepare_stamps(
    dims: RasterDims,
    stamps: &[Stamp],
) -> Result<Vec<PreparedStamp>, ComputeError> {
    let mut prepared = Vec::with_capacity(stamps.len());
    for (index, stamp) in stamps.iter().enumerate() {
        let invalid = |reason: String| {
            Err(ComputeError::InvalidInput(format!(
                "stamp {index} {reason}"
            )))
        };
        if stamp
            .a
            .iter()
            .chain(stamp.b.iter())
            .any(|c| c.unsigned_abs() > MAX_ABS_COORD_Q4 as u32)
        {
            return invalid(format!(
                "has a coordinate beyond ±{MAX_ABS_COORD_Q4} Q4 units"
            ));
        }
        let span_x = (stamp.b[0] - stamp.a[0]).abs();
        let span_y = (stamp.b[1] - stamp.a[1]).abs();
        if span_x > MAX_SEGMENT_SPAN_Q4 || span_y > MAX_SEGMENT_SPAN_Q4 {
            return invalid(format!(
                "spans {span_x} x {span_y} Q4 units; split it to at most {MAX_SEGMENT_SPAN_Q4}"
            ));
        }
        if stamp.radius_q4 > MAX_RADIUS_Q4 {
            return invalid(format!(
                "has radius {} Q4 units; the limit is {MAX_RADIUS_Q4}",
                stamp.radius_q4
            ));
        }
        if stamp.layer_first > stamp.layer_last || stamp.layer_last >= dims.layers {
            return invalid(format!(
                "has layer range {}..={} on a {}-layer raster",
                stamp.layer_first, stamp.layer_last, dims.layers
            ));
        }

        let r = stamp.radius_q4 as i32;
        let x0 = first_cell_at_or_above(stamp.a[0].min(stamp.b[0]) - r).max(0);
        let y0 = first_cell_at_or_above(stamp.a[1].min(stamp.b[1]) - r).max(0);
        let x1 = last_cell_at_or_below(stamp.a[0].max(stamp.b[0]) + r).min(dims.width as i32 - 1);
        let y1 = last_cell_at_or_below(stamp.a[1].max(stamp.b[1]) + r).min(dims.height as i32 - 1);
        if x0 > x1 || y0 > y1 {
            continue;
        }
        prepared.push(PreparedStamp {
            ax: stamp.a[0],
            ay: stamp.a[1],
            bx: stamp.b[0],
            by: stamp.b[1],
            radius_q4: stamp.radius_q4,
            layer_first: stamp.layer_first,
            layer_last: stamp.layer_last,
            x0: x0 as u32,
            y0: y0 as u32,
            x1: x1 as u32,
            y1: y1 as u32,
            pad0: 0,
        });
    }
    Ok(prepared)
}

/// Split prepared stamps into consecutive dispatch ranges, each with at most
/// `max_stamps` stamps and at most [`MAX_WORK_WORDS_PER_DISPATCH`] words of
/// work (a single larger stamp still gets a range of its own).
pub fn plan_dispatch_chunks(prepared: &[PreparedStamp], max_stamps: u32) -> Vec<Range<u32>> {
    let max_stamps = max_stamps.clamp(1, MAX_STAMPS_PER_DISPATCH);
    let mut chunks = Vec::new();
    let mut start = 0u32;
    let mut work = 0u64;
    for (index, stamp) in prepared.iter().enumerate() {
        let index = index as u32;
        let stamp_work = stamp.work_words();
        let count = index - start;
        if count > 0 && (count == max_stamps || work + stamp_work > MAX_WORK_WORDS_PER_DISPATCH) {
            chunks.push(start..index);
            start = index;
            work = 0;
        }
        work += stamp_work;
    }
    if (start as usize) < prepared.len() {
        chunks.push(start..prepared.len() as u32);
    }
    chunks
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dims(width: u32, height: u32, layers: u32) -> RasterDims {
        RasterDims::new(width, height, layers).unwrap()
    }

    #[test]
    fn raster_dims_reject_zero_and_oversized_shapes() {
        assert!(RasterDims::new(0, 4, 1).is_err());
        assert!(RasterDims::new(4, 4, 0).is_err());
        assert!(RasterDims::new(MAX_GRID_SIDE_CELLS + 1, 4, 1).is_err());
        assert!(RasterDims::new(4, 4, MAX_LAYERS + 1).is_err());
        assert!(RasterDims::new(MAX_GRID_SIDE_CELLS, MAX_GRID_SIDE_CELLS, MAX_LAYERS).is_ok());
    }

    #[test]
    fn rows_are_padded_to_whole_words() {
        let d = dims(5, 3, 2);
        assert_eq!(d.row_words(), 2);
        assert_eq!(d.plane_words(), 6);
        assert_eq!(d.total_words(), 12);
        assert_eq!(d.word_index(4, 2, 1), 6 + 2 * 2 + 1);
    }

    #[test]
    fn counter_grid_packs_cells_little_endian() {
        let mut grid = CounterGrid::new(dims(6, 2, 1));
        grid.set(1, 0, 0, 0xAB);
        grid.set(5, 1, 0, 7);
        assert_eq!(grid.words()[0], 0xAB00);
        assert_eq!(grid.words()[3], 7 << 8);
        assert_eq!(grid.get(1, 0, 0), 0xAB);
        assert_eq!(grid.get(5, 1, 0), 7);
        assert_eq!(grid.occupied_cells(), 2);
    }

    #[test]
    fn cell_bounds_round_toward_the_covered_centres() {
        // Centres are at 8, 24, 40, ... Q4.
        assert_eq!(first_cell_at_or_above(8), 0);
        assert_eq!(first_cell_at_or_above(9), 1);
        assert_eq!(first_cell_at_or_above(-100), -6);
        assert_eq!(last_cell_at_or_below(23), 0);
        assert_eq!(last_cell_at_or_below(24), 1);
        assert_eq!(last_cell_at_or_below(7), -1);
    }

    #[test]
    fn prepare_clamps_bounding_boxes_and_drops_misses() {
        let d = dims(10, 10, 2);
        let stamps = [
            Stamp::circle([8, 8], 16, 0, 1),
            Stamp::circle([-500, -500], 16, 0, 0),
            Stamp::segment([150, 8], [400, 8], 0, 1),
        ];
        let prepared = prepare_stamps(d, &stamps).unwrap();
        assert_eq!(prepared.len(), 2);
        assert_eq!(
            (
                prepared[0].x0,
                prepared[0].y0,
                prepared[0].x1,
                prepared[0].y1
            ),
            (0, 0, 1, 1)
        );
        assert_eq!((prepared[1].x0, prepared[1].x1), (9, 9));
        assert_eq!((prepared[1].y0, prepared[1].y1), (0, 0));
    }

    #[test]
    fn prepare_refuses_out_of_contract_stamps() {
        let d = dims(10, 10, 2);
        let too_long = Stamp::segment([0, 0], [MAX_SEGMENT_SPAN_Q4 + 1, 0], 4, 0);
        let too_wide = Stamp::circle([0, 0], MAX_RADIUS_Q4 + 1, 0, 0);
        let bad_layer = Stamp::circle([0, 0], 4, 1, 2);
        let reversed_layers = Stamp::circle([0, 0], 4, 1, 0);
        let far = Stamp::circle([i32::MAX, 0], 4, 0, 0);
        for stamp in [too_long, too_wide, bad_layer, reversed_layers, far] {
            let err = prepare_stamps(d, &[stamp]).unwrap_err();
            assert!(!err.is_gpu_failure(), "{err}");
        }
    }

    #[test]
    fn split_segment_respects_the_span_limit_and_shares_endpoints() {
        let pieces = Stamp::split_segment([3, -7], [20_003, 9_001], 40, 1);
        assert!(pieces.len() >= 5);
        assert_eq!(pieces.first().unwrap().a, [3, -7]);
        assert_eq!(pieces.last().unwrap().b, [20_003, 9_001]);
        for pair in pieces.windows(2) {
            assert_eq!(pair[0].b, pair[1].a);
        }
        for piece in &pieces {
            assert!((piece.b[0] - piece.a[0]).abs() <= MAX_SEGMENT_SPAN_Q4);
            assert!((piece.b[1] - piece.a[1]).abs() <= MAX_SEGMENT_SPAN_Q4);
        }
        assert_eq!(Stamp::split_segment([5, 5], [5, 5], 8, 0).len(), 1);
    }

    #[test]
    fn dispatch_chunks_respect_count_and_work_budgets() {
        let small = PreparedStamp {
            x1: 3,
            y1: 0,
            ..Default::default()
        };
        assert_eq!(small.work_words(), 1);
        let chunks = plan_dispatch_chunks(&[small; 10], 4);
        assert_eq!(chunks, vec![0..4, 4..8, 8..10]);

        // 2048 words per row x 2048 rows = 4M words: exactly one budget.
        let big = PreparedStamp {
            x1: 8191,
            y1: 2047,
            ..Default::default()
        };
        assert_eq!(big.work_words(), MAX_WORK_WORDS_PER_DISPATCH);
        let chunks = plan_dispatch_chunks(&[small, big, big, small], 100);
        assert_eq!(chunks, vec![0..1, 1..2, 2..3, 3..4]);
        assert!(plan_dispatch_chunks(&[], 100).is_empty());
    }

    #[test]
    fn costs_saturate_instead_of_wrapping() {
        assert_eq!(cost_add(COST_STRAIGHT, COST_DIAGONAL), 2414);
        assert_eq!(cost_add(u32::MAX - 1, COST_STRAIGHT), u32::MAX);
    }
}
