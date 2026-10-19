//! The wgpu raster paint twin of [`crate::CpuRaster`].

use std::borrow::Cow;
use std::time::{Duration, Instant};

use super::{read_mapped, GpuContext};
use crate::cpu::CpuRaster;
use crate::layout::{
    plan_dispatch_chunks, prepare_stamps, CounterGrid, PaintOp, PreparedStamp, RasterDims, Stamp,
};
use crate::shaders::{PAINT_ENTRY, PAINT_PARAMS_BYTES, PAINT_WGSL};
use crate::synthetic::synthetic_stamps;
use crate::{ComputeError, PaintReport, RasterCompute};

/// The smoke kernel must finish (its first-use buffer allocation, upload,
/// dispatches and readback, plus the CPU check) inside this budget, or the
/// GPU is not selected. Device creation and pipeline compilation happen
/// before it, in [`GpuContext::new`] and [`GpuRaster::new`].
pub const SMOKE_BUDGET: Duration = Duration::from_millis(500);

const SMOKE_SEED: u64 = 0x5_3A0E;
const MIN_STAMP_CAPACITY: usize = 1024;
const MIN_CLAMP_SLOTS: usize = 16;
const STAMP_BYTES: u64 = std::mem::size_of::<PreparedStamp>() as u64;

/// A paint's report plus what it cost in dispatches.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct GpuPaintStats {
    pub report: PaintReport,
    pub dispatch_count: u32,
}

/// Device buffers for one raster shape, reused while the shape stays.
struct Resident {
    dims: RasterDims,
    grid_bytes: u64,
    grid: wgpu::Buffer,
    grid_readback: wgpu::Buffer,
}

/// One u32 clamp counter per dispatch chunk, summed in u64 on the host.
/// A chunk visits at most [`crate::MAX_WORK_WORDS_PER_DISPATCH`] words of
/// four cells, so its counter cannot wrap; one counter for a whole paint
/// could, past 2^32 events, and would then disagree with the CPU's u64.
struct ClampSlots {
    capacity: usize,
    counters: wgpu::Buffer,
    readback: wgpu::Buffer,
}

/// The GPU raster backend. The grid can be kept resident on the device
/// ([`GpuRaster::upload`], [`GpuRaster::paint_resident`],
/// [`GpuRaster::download`]) or round-tripped per batch
/// ([`RasterCompute::paint`]).
pub struct GpuRaster {
    context: GpuContext,
    pipeline: wgpu::ComputePipeline,
    bind_group_layout: wgpu::BindGroupLayout,
    resident: Option<Resident>,
    stamps: Option<(wgpu::Buffer, usize)>,
    clamp_slots: Option<ClampSlots>,
}

/// What [`GpuRaster::encode_paint`] recorded.
struct EncodedPaint {
    dispatch_count: u32,
    /// Bytes of clamp counters copied to the readback buffer.
    clamp_bytes: u64,
}

impl GpuRaster {
    /// Compile the paint kernel on `context`'s device.
    pub fn new(context: GpuContext) -> Result<Self, ComputeError> {
        context.check_health()?;
        let device = context.device();
        let scope = device.push_error_scope(wgpu::ErrorFilter::Validation);
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("raster paint"),
            source: wgpu::ShaderSource::Wgsl(Cow::Borrowed(PAINT_WGSL)),
        });
        let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("raster paint"),
            layout: None,
            module: &module,
            entry_point: Some(PAINT_ENTRY),
            compilation_options: Default::default(),
            cache: None,
        });
        if let Some(error) = pollster::block_on(scope.pop()) {
            return Err(ComputeError::Gpu(format!(
                "the paint kernel did not compile: {error}"
            )));
        }
        let bind_group_layout = pipeline.get_bind_group_layout(0);
        Ok(Self {
            context,
            pipeline,
            bind_group_layout,
            resident: None,
            stamps: None,
            clamp_slots: None,
        })
    }

    pub fn context(&self) -> &GpuContext {
        &self.context
    }

    /// Run a small fixed workload (including 300 stacked stamps, so the
    /// saturation path is exercised) and require bit-exact agreement with
    /// the CPU within [`SMOKE_BUDGET`].
    pub fn smoke(&mut self) -> Result<Duration, ComputeError> {
        let started = Instant::now();
        let dims = RasterDims::new(96, 80, 2)?;
        let mut stamps = synthetic_stamps(dims, 400, SMOKE_SEED);
        stamps.extend(std::iter::repeat_n(
            Stamp::circle([520, 600], 40, 0, 1),
            300,
        ));
        let prepared = prepare_stamps(dims, &stamps)?;

        let mut gpu_grid = CounterGrid::new(dims);
        let gpu = self.paint_round_trip(&mut gpu_grid, &prepared, PaintOp::Paint)?;
        let mut cpu_grid = CounterGrid::new(dims);
        let cpu = CpuRaster::new().paint_prepared(&mut cpu_grid, &prepared, PaintOp::Paint);

        if gpu_grid != cpu_grid || gpu.report != cpu {
            let differing = gpu_grid
                .words()
                .iter()
                .zip(cpu_grid.words())
                .filter(|(g, c)| g != c)
                .count();
            return Err(ComputeError::Gpu(format!(
                "smoke kernel disagrees with the CPU: {differing} of {} words differ, \
                 clamped {} vs {}",
                cpu_grid.words().len(),
                gpu.report.clamped_events,
                cpu.clamped_events
            )));
        }
        let elapsed = started.elapsed();
        if elapsed > SMOKE_BUDGET {
            return Err(ComputeError::Gpu(format!(
                "smoke kernel took {} ms; the budget is {} ms",
                elapsed.as_millis(),
                SMOKE_BUDGET.as_millis()
            )));
        }
        Ok(elapsed)
    }

    /// Copy `grid` to the device, replacing the resident raster.
    pub fn upload(&mut self, grid: &CounterGrid) -> Result<(), ComputeError> {
        self.context.check_health()?;
        self.ensure_resident(grid.dims())?;
        let resident = self.resident()?;
        self.context
            .queue()
            .write_buffer(&resident.grid, 0, bytemuck::cast_slice(grid.words()));
        Ok(())
    }

    /// Paint into the resident raster and wait for completion. Only the
    /// per-chunk clamp counters (4 bytes per dispatch) come back.
    pub fn paint_resident(
        &mut self,
        prepared: &[PreparedStamp],
        op: PaintOp,
    ) -> Result<GpuPaintStats, ComputeError> {
        self.context.check_health()?;
        self.resident()?;
        self.write_stamps(prepared)?;
        let mut encoder = self.encoder();
        let encoded = self.encode_paint(&mut encoder, prepared, op)?;
        let submission = self.context.queue().submit([encoder.finish()]);
        let slots = self.clamp_slots()?;
        let clamped = read_mapped(
            &self.context,
            submission,
            &[(&slots.readback, encoded.clamp_bytes)],
            |_, bytes| sum_counters(bytes),
        )?;
        Ok(GpuPaintStats {
            report: PaintReport {
                clamped_events: clamped[0],
            },
            dispatch_count: encoded.dispatch_count,
        })
    }

    /// Copy the resident raster back into `grid`, which must have its shape.
    pub fn download(&mut self, grid: &mut CounterGrid) -> Result<(), ComputeError> {
        self.context.check_health()?;
        let resident = self.resident()?;
        if resident.dims != grid.dims() {
            return Err(ComputeError::InvalidInput(format!(
                "download into a {:?} grid from a resident {:?} raster",
                grid.dims(),
                resident.dims
            )));
        }
        let mut encoder = self.encoder();
        encoder.copy_buffer_to_buffer(
            &resident.grid,
            0,
            &resident.grid_readback,
            0,
            resident.grid_bytes,
        );
        let submission = self.context.queue().submit([encoder.finish()]);
        read_mapped(
            &self.context,
            submission,
            &[(&resident.grid_readback, resident.grid_bytes)],
            |_, bytes| copy_words(grid.words_mut(), bytes),
        )?;
        Ok(())
    }

    /// Upload, paint and read back in a single submission. `grid` changes
    /// only once every readback has mapped.
    pub fn paint_round_trip(
        &mut self,
        grid: &mut CounterGrid,
        prepared: &[PreparedStamp],
        op: PaintOp,
    ) -> Result<GpuPaintStats, ComputeError> {
        self.upload(grid)?;
        self.write_stamps(prepared)?;
        let mut encoder = self.encoder();
        let encoded = self.encode_paint(&mut encoder, prepared, op)?;
        let resident = self.resident()?;
        encoder.copy_buffer_to_buffer(
            &resident.grid,
            0,
            &resident.grid_readback,
            0,
            resident.grid_bytes,
        );
        let submission = self.context.queue().submit([encoder.finish()]);
        let slots = self.clamp_slots()?;
        let mut clamped = 0;
        // The counters are read first so the grid copy is the last step.
        read_mapped(
            &self.context,
            submission,
            &[
                (&slots.readback, encoded.clamp_bytes),
                (&resident.grid_readback, resident.grid_bytes),
            ],
            |index, bytes| {
                if index == 0 {
                    clamped = sum_counters(bytes);
                } else {
                    copy_words(grid.words_mut(), bytes);
                }
            },
        )?;
        Ok(GpuPaintStats {
            report: PaintReport {
                clamped_events: clamped,
            },
            dispatch_count: encoded.dispatch_count,
        })
    }

    fn encoder(&self) -> wgpu::CommandEncoder {
        self.context
            .device()
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("raster paint"),
            })
    }

    fn resident(&self) -> Result<&Resident, ComputeError> {
        self.resident
            .as_ref()
            .ok_or_else(|| ComputeError::InvalidInput("no raster is resident on the GPU".into()))
    }

    fn clamp_slots(&self) -> Result<&ClampSlots, ComputeError> {
        self.clamp_slots
            .as_ref()
            .ok_or_else(|| ComputeError::InvalidInput("no clamp counters on the GPU".into()))
    }

    /// Grow the clamp counters to at least `count` slots.
    fn ensure_clamp_slots(&mut self, count: usize) -> Result<(), ComputeError> {
        if matches!(&self.clamp_slots, Some(slots) if slots.capacity >= count) {
            return Ok(());
        }
        let capacity = count.next_power_of_two().max(MIN_CLAMP_SLOTS);
        let bytes = capacity as u64 * 4;
        let limits = self.context.device().limits();
        if bytes > limits.max_storage_buffer_binding_size {
            return Err(ComputeError::Unsupported(format!(
                "{count} dispatch chunks exceed the device's storage binding"
            )));
        }
        let device = self.context.device();
        let buffer = |label, usage| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: bytes,
                usage,
                mapped_at_creation: false,
            })
        };
        use wgpu::BufferUsages as U;
        self.clamp_slots = Some(ClampSlots {
            capacity,
            counters: buffer(
                "raster clamp counters",
                U::STORAGE | U::COPY_DST | U::COPY_SRC,
            ),
            readback: buffer("raster clamp counters readback", U::MAP_READ | U::COPY_DST),
        });
        Ok(())
    }

    fn ensure_resident(&mut self, dims: RasterDims) -> Result<(), ComputeError> {
        if matches!(&self.resident, Some(resident) if resident.dims == dims) {
            return Ok(());
        }
        let grid_bytes = dims.total_words() as u64 * 4;
        let limits = self.context.device().limits();
        let max_bytes = limits
            .max_storage_buffer_binding_size
            .min(limits.max_buffer_size);
        if grid_bytes > max_bytes {
            return Err(ComputeError::Unsupported(format!(
                "a {grid_bytes}-byte raster exceeds the device's {max_bytes}-byte storage binding"
            )));
        }
        let device = self.context.device();
        let buffer = |label, size, usage| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size,
                usage,
                mapped_at_creation: false,
            })
        };
        use wgpu::BufferUsages as U;
        self.resident = Some(Resident {
            dims,
            grid_bytes,
            grid: buffer(
                "raster grid",
                grid_bytes,
                U::STORAGE | U::COPY_DST | U::COPY_SRC,
            ),
            grid_readback: buffer(
                "raster grid readback",
                grid_bytes,
                U::MAP_READ | U::COPY_DST,
            ),
        });
        Ok(())
    }

    fn write_stamps(&mut self, prepared: &[PreparedStamp]) -> Result<(), ComputeError> {
        let capacity = self.stamps.as_ref().map_or(0, |(_, capacity)| *capacity);
        if prepared.len() > capacity || self.stamps.is_none() {
            let capacity = prepared.len().next_power_of_two().max(MIN_STAMP_CAPACITY);
            let bytes = capacity as u64 * STAMP_BYTES;
            if bytes
                > self
                    .context
                    .device()
                    .limits()
                    .max_storage_buffer_binding_size
            {
                return Err(ComputeError::Unsupported(format!(
                    "{} stamps exceed the device's storage binding",
                    prepared.len()
                )));
            }
            let buffer = self
                .context
                .device()
                .create_buffer(&wgpu::BufferDescriptor {
                    label: Some("raster stamps"),
                    size: bytes,
                    usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                });
            self.stamps = Some((buffer, capacity));
        }
        if let Some((buffer, _)) = &self.stamps {
            if !prepared.is_empty() {
                self.context
                    .queue()
                    .write_buffer(buffer, 0, bytemuck::cast_slice(prepared));
            }
        }
        Ok(())
    }

    /// Record the clamp-counter reset, the paint dispatches and the counter
    /// readback copy into the resident raster.
    fn encode_paint(
        &mut self,
        encoder: &mut wgpu::CommandEncoder,
        prepared: &[PreparedStamp],
        op: PaintOp,
    ) -> Result<EncodedPaint, ComputeError> {
        let max_groups = self
            .context
            .device()
            .limits()
            .max_compute_workgroups_per_dimension;
        let chunks = plan_dispatch_chunks(prepared, max_groups);
        // At least one slot, so an empty paint still reads back a zero.
        let slot_count = chunks.len().max(1);
        self.ensure_clamp_slots(slot_count)?;
        let clamp_bytes = slot_count as u64 * 4;

        let resident = self.resident()?;
        let slots = self.clamp_slots()?;
        let Some((stamps, _)) = &self.stamps else {
            return Err(ComputeError::InvalidInput("no stamp buffer".into()));
        };
        let device = self.context.device();
        let queue = self.context.queue();
        let dims = resident.dims;
        let bind_groups: Vec<wgpu::BindGroup> = chunks
            .iter()
            .enumerate()
            .map(|(chunk, range)| {
                let params: [u32; 8] = [
                    dims.width,
                    dims.height,
                    dims.layers,
                    dims.row_words(),
                    range.start,
                    range.end - range.start,
                    op.code(),
                    chunk as u32,
                ];
                // Not `create_buffer_init`: it maps at creation and panics
                // on a lost device instead of reporting it.
                let buffer = device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("raster paint params"),
                    size: PAINT_PARAMS_BYTES,
                    usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                });
                queue.write_buffer(&buffer, 0, bytemuck::cast_slice(&params));
                let params = buffer;
                device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some("raster paint"),
                    layout: &self.bind_group_layout,
                    entries: &[
                        wgpu::BindGroupEntry {
                            binding: 0,
                            resource: params.as_entire_binding(),
                        },
                        wgpu::BindGroupEntry {
                            binding: 1,
                            resource: stamps.as_entire_binding(),
                        },
                        wgpu::BindGroupEntry {
                            binding: 2,
                            resource: resident.grid.as_entire_binding(),
                        },
                        wgpu::BindGroupEntry {
                            binding: 3,
                            resource: slots.counters.as_entire_binding(),
                        },
                    ],
                })
            })
            .collect();

        encoder.clear_buffer(&slots.counters, 0, Some(clamp_bytes));
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("raster paint"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.pipeline);
            for (range, bind_group) in chunks.iter().zip(&bind_groups) {
                pass.set_bind_group(0, Some(bind_group), &[]);
                pass.dispatch_workgroups(range.end - range.start, 1, 1);
            }
        }
        encoder.copy_buffer_to_buffer(&slots.counters, 0, &slots.readback, 0, clamp_bytes);
        Ok(EncodedPaint {
            dispatch_count: chunks.len() as u32,
            clamp_bytes,
        })
    }
}

impl RasterCompute for GpuRaster {
    fn backend_name(&self) -> &'static str {
        "wgpu"
    }

    fn paint(
        &mut self,
        grid: &mut CounterGrid,
        stamps: &[Stamp],
        op: PaintOp,
    ) -> Result<PaintReport, ComputeError> {
        let prepared = prepare_stamps(grid.dims(), stamps)?;
        Ok(self.paint_round_trip(grid, &prepared, op)?.report)
    }
}

fn le_u32(bytes: &[u8]) -> u32 {
    let mut word = [0; 4];
    word.copy_from_slice(&bytes[..4]);
    u32::from_le_bytes(word)
}

/// Sum little-endian u32 per-chunk counters without wrapping.
fn sum_counters(bytes: &[u8]) -> u64 {
    bytes
        .chunks_exact(4)
        .map(|chunk| u64::from(le_u32(chunk)))
        .sum()
}

/// Copy little-endian device words into host words. Mapped ranges are
/// normally word-aligned, so the cast is the fast path.
fn copy_words(dst: &mut [u32], bytes: &[u8]) {
    match bytemuck::try_cast_slice::<u8, u32>(bytes) {
        Ok(words) if cfg!(target_endian = "little") => dst.copy_from_slice(words),
        _ => {
            for (word, chunk) in dst.iter_mut().zip(bytes.chunks_exact(4)) {
                *word = le_u32(chunk);
            }
        }
    }
}
