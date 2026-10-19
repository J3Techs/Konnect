//! Backend selection and the latch-to-CPU wrapper.
//!
//! Order: the `KONNECT_GPU` preference, then (for `auto` and `wgpu`) an
//! adapter probe for features and limits, then a smoke kernel under half a
//! second checked against the CPU. Under `auto` any failure selects the CPU
//! and records why; under `wgpu` it is an error, because the caller asked
//! for the GPU explicitly.

use serde::Serialize;

use crate::cpu::CpuRaster;
use crate::layout::{CounterGrid, PaintOp, Stamp};
use crate::probe::AdapterProbe;
use crate::{ComputeError, PaintReport, RasterCompute};

/// Environment variable naming the backend preference.
pub const GPU_ENV: &str = "KONNECT_GPU";

/// Which raster backend the caller wants.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum GpuPreference {
    /// The GPU when it is built, present and passes the smoke test;
    /// otherwise the CPU.
    #[default]
    Auto,
    /// Always the CPU.
    Off,
    /// The GPU or an error.
    Wgpu,
}

impl GpuPreference {
    pub fn parse(value: &str) -> Result<Self, ComputeError> {
        match value.trim().to_ascii_lowercase().as_str() {
            "" | "auto" => Ok(Self::Auto),
            "off" | "cpu" | "0" => Ok(Self::Off),
            "wgpu" | "gpu" | "1" => Ok(Self::Wgpu),
            other => Err(ComputeError::InvalidInput(format!(
                "{GPU_ENV} must be auto, off or wgpu, got '{other}'"
            ))),
        }
    }

    /// Read [`GPU_ENV`]; unset means `auto`.
    pub fn from_env() -> Result<Self, ComputeError> {
        match std::env::var(GPU_ENV) {
            Ok(value) => Self::parse(&value),
            Err(std::env::VarError::NotPresent) => Ok(Self::Auto),
            Err(std::env::VarError::NotUnicode(_)) => Err(ComputeError::InvalidInput(format!(
                "{GPU_ENV} is not valid Unicode"
            ))),
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Auto => "auto",
            Self::Off => "off",
            Self::Wgpu => "wgpu",
        }
    }
}

/// Why a backend was chosen, for run evidence.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct SelectionReport {
    pub requested: &'static str,
    pub selected: &'static str,
    pub reason: String,
    pub adapter: Option<AdapterProbe>,
    pub smoke_ms: Option<f64>,
}

/// The chosen backend and the evidence for the choice.
pub struct RasterSelection {
    pub backend: LatchingRaster,
    pub report: SelectionReport,
}

/// Choose a raster backend for `preference`.
pub fn select_raster_backend(preference: GpuPreference) -> Result<RasterSelection, ComputeError> {
    let requested = preference.as_str();
    if preference == GpuPreference::Off {
        return Ok(cpu_selection(requested, format!("{GPU_ENV}=off"), None));
    }
    select_gpu(preference, requested)
}

fn cpu_selection(
    requested: &'static str,
    reason: String,
    adapter: Option<AdapterProbe>,
) -> RasterSelection {
    RasterSelection {
        backend: LatchingRaster::cpu(),
        report: SelectionReport {
            requested,
            selected: "cpu",
            reason,
            adapter,
            smoke_ms: None,
        },
    }
}

#[cfg(not(feature = "gpu"))]
fn select_gpu(
    preference: GpuPreference,
    requested: &'static str,
) -> Result<RasterSelection, ComputeError> {
    let reason = "built without the `gpu` feature".to_string();
    if preference == GpuPreference::Wgpu {
        return Err(ComputeError::Gpu(reason));
    }
    Ok(cpu_selection(requested, reason, None))
}

#[cfg(feature = "gpu")]
fn select_gpu(
    preference: GpuPreference,
    requested: &'static str,
) -> Result<RasterSelection, ComputeError> {
    let mut adapter = None;
    let attempt = (|| {
        let context = crate::gpu::GpuContext::new()?;
        adapter = Some(context.probe().clone());
        // A software rasterizer (lavapipe, llvmpipe, WARP) is slower than
        // rayon on the same cores. Only an explicit `wgpu` request, as the
        // lavapipe parity job makes, may select one.
        if preference == GpuPreference::Auto && context.probe().is_software() {
            return Err(ComputeError::Gpu(format!(
                "adapter '{}' is a software rasterizer",
                context.probe().name
            )));
        }
        let mut raster = crate::gpu::GpuRaster::new(context)?;
        let smoke = raster.smoke()?;
        Ok::<_, ComputeError>((raster, smoke))
    })();
    match attempt {
        Ok((raster, smoke)) => Ok(RasterSelection {
            backend: LatchingRaster::gpu(raster),
            report: SelectionReport {
                requested,
                selected: "wgpu",
                reason: "adapter probe and smoke kernel passed".to_string(),
                adapter,
                smoke_ms: Some(smoke.as_secs_f64() * 1e3),
            },
        }),
        Err(e) if preference == GpuPreference::Wgpu => Err(e),
        Err(e) => {
            tracing::info!(error = %e, "GPU raster backend unavailable; using the CPU");
            Ok(cpu_selection(requested, e.to_string(), adapter))
        }
    }
}

/// A raster backend that starts on the GPU (when given one) and falls back
/// to the CPU for good on the first GPU failure. The failing batch is rerun
/// on the CPU from the unchanged grid, so the caller sees one successful
/// paint either way.
pub struct LatchingRaster {
    #[cfg(feature = "gpu")]
    gpu: Option<crate::gpu::GpuRaster>,
    cpu: CpuRaster,
    latch_reason: Option<String>,
}

impl LatchingRaster {
    /// A backend that only ever uses the CPU.
    pub fn cpu() -> Self {
        Self {
            #[cfg(feature = "gpu")]
            gpu: None,
            cpu: CpuRaster::new(),
            latch_reason: None,
        }
    }

    #[cfg(feature = "gpu")]
    pub fn gpu(raster: crate::gpu::GpuRaster) -> Self {
        Self {
            gpu: Some(raster),
            cpu: CpuRaster::new(),
            latch_reason: None,
        }
    }

    /// Why the backend latched to the CPU, if it did.
    pub fn latch_reason(&self) -> Option<&str> {
        self.latch_reason.as_deref()
    }

    /// The GPU backend while it is still active.
    #[cfg(feature = "gpu")]
    pub fn gpu_raster(&mut self) -> Option<&mut crate::gpu::GpuRaster> {
        self.gpu.as_mut()
    }
}

impl RasterCompute for LatchingRaster {
    fn backend_name(&self) -> &'static str {
        #[cfg(feature = "gpu")]
        if self.gpu.is_some() {
            return "wgpu";
        }
        "cpu"
    }

    fn paint(
        &mut self,
        grid: &mut CounterGrid,
        stamps: &[Stamp],
        op: PaintOp,
    ) -> Result<PaintReport, ComputeError> {
        #[cfg(feature = "gpu")]
        if let Some(gpu) = self.gpu.as_mut() {
            match gpu.paint(grid, stamps, op) {
                Err(e) if e.poisons_gpu() => {
                    tracing::warn!(error = %e, "GPU raster failed; rerunning on the CPU and latching to it");
                    if let Some(gpu) = self.gpu.take() {
                        retire_off_thread(gpu);
                    }
                    self.latch_reason = Some(e.to_string());
                }
                Err(e) if e.is_gpu_failure() => {
                    tracing::debug!(error = %e, "batch too big for the GPU; running it on the CPU");
                }
                result => return result,
            }
        }
        self.cpu.paint(grid, stamps, op)
    }
}

/// Drop a failed backend on a detached thread.
///
/// Dropping a `GpuRaster` drops its wgpu `Queue`, and wgpu-core's
/// `Queue::drop` waits for the device to go idle with no timeout (and
/// panics on any error but a lost device). After a wait timeout that
/// would stall the routing thread until the GPU finishes or the TDR
/// watchdog fires, so the latch hands it to a thread that may block or
/// panic on its own. If no thread can be spawned the backend is leaked
/// instead: a stuck device is never waited on by the caller.
#[cfg(any(feature = "gpu", test))]
fn retire_off_thread<T: Send + 'static>(retired: T) {
    let slot = std::sync::Arc::new(std::sync::Mutex::new(Some(retired)));
    let for_thread = std::sync::Arc::clone(&slot);
    let spawned = std::thread::Builder::new()
        .name("konnect-gpu-retire".to_string())
        .spawn(move || {
            let retired = for_thread.lock().unwrap_or_else(|e| e.into_inner()).take();
            drop(retired);
        });
    if spawned.is_err() {
        let retired = slot.lock().unwrap_or_else(|e| e.into_inner()).take();
        std::mem::forget(retired);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A value whose drop blocks until released, like a queue waiting on a
    /// hung device.
    struct SlowDrop {
        release: std::sync::mpsc::Receiver<()>,
        dropped: std::sync::mpsc::Sender<std::thread::ThreadId>,
    }

    impl Drop for SlowDrop {
        fn drop(&mut self) {
            let _ = self.release.recv();
            let _ = self.dropped.send(std::thread::current().id());
        }
    }

    #[test]
    fn retiring_a_backend_never_blocks_the_caller() {
        let (release, release_rx) = std::sync::mpsc::channel();
        let (dropped_tx, dropped) = std::sync::mpsc::channel();
        retire_off_thread(SlowDrop {
            release: release_rx,
            dropped: dropped_tx,
        });
        // Reaching this line at all means the caller did not wait on the
        // drop; it is still blocked until released.
        assert!(dropped.try_recv().is_err());
        release.send(()).unwrap();
        let dropped_on = dropped
            .recv_timeout(std::time::Duration::from_secs(5))
            .unwrap();
        assert_ne!(dropped_on, std::thread::current().id());
    }

    #[test]
    fn preference_parses_the_documented_values() {
        assert_eq!(GpuPreference::parse("auto").unwrap(), GpuPreference::Auto);
        assert_eq!(GpuPreference::parse("").unwrap(), GpuPreference::Auto);
        assert_eq!(GpuPreference::parse(" OFF ").unwrap(), GpuPreference::Off);
        assert_eq!(GpuPreference::parse("wgpu").unwrap(), GpuPreference::Wgpu);
        assert!(GpuPreference::parse("cuda").is_err());
    }

    #[test]
    fn off_selects_the_cpu_without_touching_a_gpu() {
        let selection = select_raster_backend(GpuPreference::Off).unwrap();
        assert_eq!(selection.report.selected, "cpu");
        assert_eq!(selection.backend.backend_name(), "cpu");
        assert!(selection.report.adapter.is_none());
    }

    #[cfg(not(feature = "gpu"))]
    #[test]
    fn without_the_feature_auto_falls_back_and_wgpu_refuses() {
        let selection = select_raster_backend(GpuPreference::Auto).unwrap();
        assert_eq!(selection.report.selected, "cpu");
        assert!(selection.report.reason.contains("gpu"));
        assert!(select_raster_backend(GpuPreference::Wgpu).is_err());
    }
}
