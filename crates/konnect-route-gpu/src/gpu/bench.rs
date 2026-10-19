//! Latency probes for the G1 toolchain gate: what a submit, a readback and
//! an empty dispatch cost on this driver, before any kernel work.

use std::borrow::Cow;
use std::time::{Duration, Instant};

use super::{read_mapped, GpuContext};
use crate::shaders::{NOOP_ENTRY, NOOP_WGSL};
use crate::ComputeError;

/// Host-side timings of `iterations` round trips: write 256 bytes, copy them
/// on the GPU into a mappable buffer, submit, `map_async`, wait, read and
/// unmap. This is the floor under any CPU-consumes-GPU-result design.
pub fn round_trip_latency(
    context: &GpuContext,
    iterations: usize,
) -> Result<Vec<Duration>, ComputeError> {
    const BYTES: u64 = 256;
    let device = context.device();
    let source = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("round trip source"),
        size: BYTES,
        usage: wgpu::BufferUsages::COPY_SRC | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let readback = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("round trip readback"),
        size: BYTES,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let mut samples = Vec::with_capacity(iterations);
    for iteration in 0..iterations {
        let payload = [iteration as u8; BYTES as usize];
        let started = Instant::now();
        context.queue().write_buffer(&source, 0, &payload);
        let mut encoder = device.create_command_encoder(&Default::default());
        encoder.copy_buffer_to_buffer(&source, 0, &readback, 0, BYTES);
        let submission = context.queue().submit([encoder.finish()]);
        let first = read_mapped(context, submission, &[(&readback, BYTES)], |_, bytes| {
            bytes[0]
        })?;
        samples.push(started.elapsed());
        if first[0] != iteration as u8 {
            return Err(ComputeError::Gpu(format!(
                "round trip {iteration} read back {} instead of {}",
                first[0], iteration as u8
            )));
        }
    }
    Ok(samples)
}

/// An empty compute pipeline for launch-overhead measurements.
pub fn noop_pipeline(context: &GpuContext) -> Result<wgpu::ComputePipeline, ComputeError> {
    let device = context.device();
    let scope = device.push_error_scope(wgpu::ErrorFilter::Validation);
    let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("noop"),
        source: wgpu::ShaderSource::Wgsl(Cow::Borrowed(NOOP_WGSL)),
    });
    let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("noop"),
        layout: None,
        module: &module,
        entry_point: Some(NOOP_ENTRY),
        compilation_options: Default::default(),
        cache: None,
    });
    if let Some(error) = pollster::block_on(scope.pop()) {
        return Err(ComputeError::Gpu(format!(
            "the noop kernel did not compile: {error}"
        )));
    }
    Ok(pipeline)
}

/// Submit `dispatches` empty single-workgroup dispatches in one compute pass
/// and wait. Returns the host wall time of encode + submit + wait.
pub fn empty_dispatches(
    context: &GpuContext,
    pipeline: &wgpu::ComputePipeline,
    dispatches: u32,
) -> Result<Duration, ComputeError> {
    let started = Instant::now();
    let mut encoder = context.device().create_command_encoder(&Default::default());
    {
        let mut pass = encoder.begin_compute_pass(&Default::default());
        pass.set_pipeline(pipeline);
        for _ in 0..dispatches {
            pass.dispatch_workgroups(1, 1, 1);
        }
    }
    let submission = context.queue().submit([encoder.finish()]);
    context.wait(submission)?;
    Ok(started.elapsed())
}
