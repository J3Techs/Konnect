//! The wgpu backend: adapter probe, device health and the raster twin.
//!
//! The GPU drives the desktop and KiCad's canvas, so the device is treated
//! as fragile: every wait has a timeout far below the 2 s TDR watchdog,
//! every dispatch is planned under 50 ms, and a lost device or uncaptured
//! validation error poisons the context for good (see
//! [`crate::LatchingRaster`]).

pub mod bench;
mod raster;

pub use raster::{GpuPaintStats, GpuRaster};

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::probe::AdapterProbe;
use crate::ComputeError;

/// Longest a host wait on the GPU may take before the context is declared
/// failed. Windows resets the adapter after 2 s.
pub const WAIT_TIMEOUT: Duration = Duration::from_secs(1);

/// Optional features requested when the adapter has them. None is needed by
/// the raster kernel; they are what the sweep prototype will want, and
/// requesting them now proves the device can be created with them.
fn wanted_features() -> wgpu::Features {
    wgpu::Features::SHADER_INT64
        | wgpu::Features::SHADER_INT64_ATOMIC_MIN_MAX
        | wgpu::Features::SUBGROUP
        | wgpu::Features::TIMESTAMP_QUERY
}

/// The backend set this build pins: Vulkan everywhere but Apple, Metal
/// there. DX12 and GLES are not compiled in.
pub fn pinned_backends() -> wgpu::Backends {
    if cfg!(target_vendor = "apple") {
        wgpu::Backends::METAL
    } else {
        wgpu::Backends::VULKAN
    }
}

#[derive(Default)]
struct Health {
    lost: AtomicBool,
    failure: Mutex<Option<String>>,
}

impl Health {
    fn record(&self, message: String) {
        let mut failure = self.failure.lock().unwrap_or_else(|e| e.into_inner());
        failure.get_or_insert(message);
    }
}

/// An adapter, its device and queue, and the health latch fed by the
/// device-lost and uncaptured-error callbacks.
pub struct GpuContext {
    device: wgpu::Device,
    queue: wgpu::Queue,
    probe: AdapterProbe,
    health: Arc<Health>,
}

impl GpuContext {
    /// Probe the pinned backend's high-performance adapter and open a device
    /// on it with the adapter's own limits (the WebGPU defaults would cap a
    /// storage binding at 128 MiB).
    pub fn new() -> Result<Self, ComputeError> {
        pollster::block_on(Self::new_async())
    }

    async fn new_async() -> Result<Self, ComputeError> {
        let mut descriptor = wgpu::InstanceDescriptor::new_without_display_handle();
        descriptor.backends = pinned_backends();
        let instance = wgpu::Instance::new(descriptor);
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                ..Default::default()
            })
            .await
            .map_err(|e| {
                ComputeError::Gpu(format!("no adapter on {:?}: {e}", pinned_backends()))
            })?;
        let probe = probe_adapter(&adapter);

        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor {
                label: Some("konnect-route-gpu"),
                required_features: adapter.features().intersection(wanted_features()),
                required_limits: adapter.limits(),
                ..Default::default()
            })
            .await
            .map_err(|e| {
                ComputeError::Gpu(format!("device request on '{}' failed: {e}", probe.name))
            })?;

        let health = Arc::new(Health::default());
        let lost = Arc::clone(&health);
        device.set_device_lost_callback(move |reason, message| {
            tracing::warn!(?reason, %message, "GPU device lost");
            lost.lost.store(true, Ordering::SeqCst);
            lost.record(format!("{reason:?}: {message}"));
        });
        let errors = Arc::clone(&health);
        let handler: Arc<dyn wgpu::UncapturedErrorHandler> = Arc::new(move |error: wgpu::Error| {
            tracing::warn!(%error, "uncaptured GPU error");
            errors.record(error.to_string());
        });
        device.on_uncaptured_error(handler);

        Ok(Self {
            device,
            queue,
            probe,
            health,
        })
    }

    pub fn probe(&self) -> &AdapterProbe {
        &self.probe
    }

    pub fn device(&self) -> &wgpu::Device {
        &self.device
    }

    pub fn queue(&self) -> &wgpu::Queue {
        &self.queue
    }

    /// Fail if the device was lost or any uncaptured error was reported.
    pub fn check_health(&self) -> Result<(), ComputeError> {
        let failure = self
            .health
            .failure
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone();
        if self.health.lost.load(Ordering::SeqCst) {
            return Err(ComputeError::DeviceLost(
                failure.unwrap_or_else(|| "device lost".to_string()),
            ));
        }
        match failure {
            Some(message) => Err(ComputeError::Gpu(message)),
            None => Ok(()),
        }
    }

    /// Block until `submission` and its callbacks have completed, within
    /// [`WAIT_TIMEOUT`].
    pub fn wait(&self, submission: wgpu::SubmissionIndex) -> Result<(), ComputeError> {
        self.device
            .poll(wgpu::PollType::Wait {
                submission_index: Some(submission),
                timeout: Some(WAIT_TIMEOUT),
            })
            .map_err(|e| ComputeError::Gpu(format!("waiting for the GPU failed: {e}")))?;
        self.check_health()
    }

    /// Destroy the device, as a driver reset would. For fault-injection
    /// tests of the latch-to-CPU path; the context is useless afterwards.
    pub fn simulate_device_loss(&self) {
        self.device.destroy();
    }
}

fn probe_adapter(adapter: &wgpu::Adapter) -> AdapterProbe {
    let info = adapter.get_info();
    let features = adapter.features();
    let limits = adapter.limits();
    AdapterProbe {
        name: info.name,
        backend: format!("{:?}", info.backend),
        device_type: format!("{:?}", info.device_type),
        vendor_id: info.vendor,
        device_id: info.device,
        driver: info.driver,
        driver_info: info.driver_info,
        subgroup_min_size: info.subgroup_min_size,
        subgroup_max_size: info.subgroup_max_size,
        shader_int64: features.contains(wgpu::Features::SHADER_INT64),
        shader_int64_atomic_min_max: features.contains(wgpu::Features::SHADER_INT64_ATOMIC_MIN_MAX),
        shader_int64_atomic_all_ops: features.contains(wgpu::Features::SHADER_INT64_ATOMIC_ALL_OPS),
        subgroup: features.contains(wgpu::Features::SUBGROUP),
        timestamp_query: features.contains(wgpu::Features::TIMESTAMP_QUERY),
        max_storage_buffer_binding_size_bytes: limits.max_storage_buffer_binding_size,
        max_buffer_size_bytes: limits.max_buffer_size,
        max_compute_workgroup_storage_size_bytes: limits.max_compute_workgroup_storage_size,
        max_compute_invocations_per_workgroup: limits.max_compute_invocations_per_workgroup,
        max_compute_workgroups_per_dimension: limits.max_compute_workgroups_per_dimension,
    }
}

/// Map `buffer`'s first `size` bytes for reading after `submission`, wait,
/// and hand the bytes to `read`. The buffer is unmapped afterwards.
pub(crate) fn read_mapped<T>(
    context: &GpuContext,
    submission: wgpu::SubmissionIndex,
    buffers: &[(&wgpu::Buffer, u64)],
    mut read: impl FnMut(usize, &[u8]) -> T,
) -> Result<Vec<T>, ComputeError> {
    let (sender, receiver) = std::sync::mpsc::channel();
    for (index, (buffer, size)) in buffers.iter().enumerate() {
        let sender = sender.clone();
        buffer
            .slice(..*size)
            .map_async(wgpu::MapMode::Read, move |result| {
                let _ = sender.send((index, result));
            });
    }
    drop(sender);
    context.wait(submission)?;

    let mut mapped = 0;
    while let Ok((index, result)) = receiver.try_recv() {
        result.map_err(|e| ComputeError::Gpu(format!("readback {index} did not map: {e}")))?;
        mapped += 1;
    }
    if mapped != buffers.len() {
        return Err(ComputeError::Gpu(format!(
            "{} of {} readbacks mapped before the wait returned",
            mapped,
            buffers.len()
        )));
    }

    let mut out = Vec::with_capacity(buffers.len());
    for (index, (buffer, size)) in buffers.iter().enumerate() {
        let slice = buffer.slice(..*size);
        let value = {
            let view = slice
                .get_mapped_range()
                .map_err(|e| ComputeError::Gpu(format!("readback {index} range: {e}")))?;
            read(index, &view)
        };
        buffer.unmap();
        out.push(value);
    }
    Ok(out)
}
