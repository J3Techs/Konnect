//! GPU-shaped compute seams.
//!
//! The detailed router runs on the CPU, but its data is laid out so a GPU
//! backend can be slotted in per method without changing callers:
//!
//! - [`raster`]: flat structure-of-arrays planes per (tile, layer, clearance
//!   profile) — `u8` CORE/HALO counters, `u16` present/history costs, `u8`
//!   heading and via-site masks — with every row padded to 4-byte words.
//! - [`cost`]: `u32` fixed-point costs with saturating arithmetic.
//! - [`job`]: one 64-byte `#[repr(C)]` [`JobDescriptor`] per connection.
//! - [`batch`]: the deterministic batch builder that feeds rayon now and GPU
//!   dispatches later.
//! - [`backend`]: the [`SearchBackend`]/[`RouteCompute`] traits and the
//!   [`CpuCompute`] reference.
//!
//! **The GPU only proposes.** Every backend returns lattice paths; the CPU
//! re-checks each proposal with the exact kernel before any geometry is
//! committed, in deterministic order.

pub mod backend;
pub mod batch;
pub mod cost;
pub mod job;
pub mod raster;

pub use backend::{
    CostField, CpuCompute, PaintKind, PaintShape, PathProposal, RouteCompute, SearchBackend,
    SearchOutcome,
};
pub use batch::{Batch, BatchBuilder, BatchPolicy};
pub use cost::{Cost, CostParams};
pub use job::{JobDescriptor, JobNode, JobPool};
pub use raster::{classify, Occupancy, Plane, TileRaster, TileStack, Window};
