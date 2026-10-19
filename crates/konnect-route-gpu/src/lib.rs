//! Raster compute for Konnect's native router, with an optional GPU twin.
//!
//! The router keeps occupancy rasters: one u8 counter per lattice cell per
//! layer, incremented when a track, via or pad is stamped into the cell and
//! decremented when it is ripped up. This crate owns that one kernel in two
//! implementations that must agree **bit for bit**:
//!
//! - [`CpuRaster`], the rayon backend. It is always built, its default
//!   scanline span fill is the production CPU path, and it is the oracle
//!   every other backend is checked against.
//! - `GpuRaster` (feature `gpu`), a wgpu/WGSL compute twin on the Vulkan
//!   backend (Metal on Apple). It exists to measure whether GPU raster
//!   maintenance can pay at all on a large, dense board; nothing in the
//!   router depends on it.
//!
//! # Layout
//!
//! Everything the GPU sees is integer and tile-local, so parity is exact:
//!
//! - Rasters are flat structure-of-arrays per (tile, layer, profile), u8
//!   counters packed four to a little-endian `u32` word, each row padded to a
//!   whole word ([`RasterDims::row_words`]). WGSL has no u8 atomics; the word
//!   is the unit of atomic update.
//! - Stamp geometry is tile-local i32 in Q4 fixed point (1/16 cell), so cell
//!   `(x, y)` has its centre at `(16x + 8, 16y + 8)`. Spans and radii are
//!   bounded ([`MAX_SEGMENT_SPAN_Q4`], [`MAX_RADIUS_Q4`]) so every product
//!   fits i32, except one 64-bit comparison that the WGSL does in two u32
//!   halves.
//! - Search costs are u32 fixed point with saturating adds
//!   ([`COST_STRAIGHT`], [`COST_DIAGONAL`]); they are declared here so the
//!   later sweep kernel and the CPU A* share one numeric model.
//!
//! # Backend selection
//!
//! [`select_raster_backend`] reads `KONNECT_GPU=auto|off|wgpu`. `auto`
//! probes the adapter, runs a smoke kernel under half a second and checks it
//! against the CPU; any failure selects the CPU. A device loss or TDR during
//! a run reruns the in-flight batch on the CPU and **latches to the CPU** for
//! the rest of that backend's life ([`LatchingRaster`]); a TDR resets the
//! adapter KiCad is drawing with, so the GPU is never retried. Every dispatch
//! is planned to stay well under 50 ms, and nothing here touches the Windows
//! TDR registry keys.
//!
//! # Relationship to the router's compute trait
//!
//! The router is freezing a broader `RouteCompute` trait (paint/unpaint,
//! derive_masks, update_costs, search_batch, cost_field) in parallel.
//! [`RasterCompute`] is the minimal local stand-in for its paint/unpaint
//! seam. When that trait lands, `RasterCompute::paint` becomes its
//! paint/unpaint methods, [`CounterGrid`] becomes (or wraps) its per-tile
//! raster, and [`Stamp`] is produced from its i32-nm geometry by the
//! tile-local Q4 conversion; the kernels and parity tests carry over as-is.

pub mod cpu;
pub mod layout;
pub mod probe;
pub mod select;
pub mod shaders;
pub mod synthetic;

#[cfg(feature = "gpu")]
pub mod gpu;

pub use cpu::{CpuFill, CpuRaster};
pub use layout::{
    plan_dispatch_chunks, prepare_stamps, CounterGrid, PaintOp, PreparedStamp, RasterDims, Stamp,
    COST_DIAGONAL, COST_STRAIGHT, MAX_GRID_SIDE_CELLS, MAX_LAYERS, MAX_RADIUS_Q4,
    MAX_SEGMENT_SPAN_Q4, MAX_STAMPS_PER_DISPATCH, MAX_WORK_WORDS_PER_DISPATCH, Q4_PER_CELL,
};
pub use probe::AdapterProbe;
pub use select::{
    select_raster_backend, GpuPreference, LatchingRaster, RasterSelection, SelectionReport, GPU_ENV,
};

/// What one paint or unpaint batch did beyond changing the grid.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PaintReport {
    /// Cell updates that hit the counter's limit: a paint on a cell already
    /// at 255, or an unpaint on a cell already at 0. Within one batch of one
    /// [`PaintOp`] this count does not depend on update order, so the CPU and
    /// GPU agree on it exactly. A non-zero paint count means the u8 raster
    /// has saturated and the caller needs the u16 fallback; a non-zero
    /// unpaint count means something was ripped up that was never stamped.
    pub clamped_events: u64,
}

/// Failures of a raster backend.
#[derive(Debug, thiserror::Error)]
pub enum ComputeError {
    /// The caller's input breaks the layout contract. Never a reason to
    /// fall back: the CPU would refuse the same input.
    #[error("invalid raster input: {0}")]
    InvalidInput(String),
    /// The GPU path failed (adapter, validation, mapping, timeout or a
    /// parity mismatch). The caller may rerun the batch on the CPU.
    #[error("GPU raster backend failed: {0}")]
    Gpu(String),
    /// The batch is valid but exceeds what this device can hold (a raster
    /// or stamp list larger than its storage binding). The device is
    /// healthy: the caller runs this batch on the CPU and keeps the GPU.
    #[error("batch exceeds the GPU's limits: {0}")]
    Unsupported(String),
    /// The device was lost (TDR, driver reset or destroy). The caller must
    /// rerun the batch on the CPU and must not use the GPU again.
    #[error("GPU device lost: {0}")]
    DeviceLost(String),
}

impl ComputeError {
    /// True when the batch may be rerun on the CPU: the failure was the
    /// GPU's, not the input's.
    pub fn is_gpu_failure(&self) -> bool {
        matches!(
            self,
            ComputeError::Gpu(_) | ComputeError::DeviceLost(_) | ComputeError::Unsupported(_)
        )
    }

    /// True when the GPU must not be used again: a real failure, not a
    /// batch that was merely too big for it.
    pub fn poisons_gpu(&self) -> bool {
        matches!(self, ComputeError::Gpu(_) | ComputeError::DeviceLost(_))
    }
}

/// The paint/unpaint seam of the router's compute trait (see the crate docs
/// for how it maps onto the frozen `RouteCompute`).
pub trait RasterCompute: Send {
    /// Short backend name for run evidence: `cpu` or `wgpu`.
    fn backend_name(&self) -> &'static str;

    /// Apply `op` once for every stamp to every covered cell of `grid`.
    ///
    /// On error the grid is unchanged, so the batch can be rerun elsewhere.
    fn paint(
        &mut self,
        grid: &mut CounterGrid,
        stamps: &[Stamp],
        op: PaintOp,
    ) -> Result<PaintReport, ComputeError>;
}
