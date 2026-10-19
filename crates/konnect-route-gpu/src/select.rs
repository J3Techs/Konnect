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
                Err(e) if e.is_gpu_failure() => {
                    tracing::warn!(error = %e, "GPU raster failed; rerunning on the CPU and latching to it");
                    self.gpu = None;
                    self.latch_reason = Some(e.to_string());
                }
                result => return result,
            }
        }
        self.cpu.paint(grid, stamps, op)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
