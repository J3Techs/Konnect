//! GPU-vs-CPU raster parity on real hardware (feature `gpu`).
//!
//! Without an adapter these tests skip with a message, so GPU-less CI
//! stays green. Set `KONNECT_REQUIRE_GPU=1` (the lavapipe CI job does) to
//! turn a missing adapter into a failure instead.

#![cfg(feature = "gpu")]

use konnect_route_gpu::gpu::{GpuContext, GpuRaster};
use konnect_route_gpu::synthetic::synthetic_stamps;
use konnect_route_gpu::{
    prepare_stamps, CounterGrid, CpuRaster, LatchingRaster, PaintOp, RasterCompute, RasterDims,
    Stamp,
};

fn gpu_or_skip(test: &str) -> Option<GpuRaster> {
    let required = std::env::var("KONNECT_REQUIRE_GPU").is_ok_and(|v| v == "1");
    match GpuContext::new().and_then(GpuRaster::new) {
        Ok(raster) => Some(raster),
        Err(e) if required => panic!("{test}: KONNECT_REQUIRE_GPU=1 but no usable GPU: {e}"),
        Err(e) => {
            eprintln!("{test}: skipped, no usable GPU: {e}");
            None
        }
    }
}

#[test]
fn smoke_kernel_passes_within_budget() {
    let Some(mut gpu) = gpu_or_skip("smoke_kernel_passes_within_budget") else {
        return;
    };
    let elapsed = gpu.smoke().unwrap();
    eprintln!(
        "smoke kernel: {elapsed:?} on {}",
        gpu.context().probe().name
    );
}

#[test]
fn paint_and_unpaint_are_bit_exact_with_the_cpu_twin() {
    let Some(mut gpu) = gpu_or_skip("paint_and_unpaint_are_bit_exact_with_the_cpu_twin") else {
        return;
    };
    let shapes = [(37, 29, 1), (128, 96, 2), (517, 263, 3), (1024, 1024, 4)];
    for (case, &(width, height, layers)) in shapes.iter().enumerate() {
        let dims = RasterDims::new(width, height, layers).unwrap();
        let count = (width * height / 64) as usize;
        let mut stamps = synthetic_stamps(dims, count, 77 + case as u64);
        // Saturate one cell on every layer.
        stamps.extend(std::iter::repeat_n(
            Stamp::circle([8, 8], 0, 0, layers - 1),
            260,
        ));

        let mut cpu_grid = CounterGrid::new(dims);
        let mut gpu_grid = CounterGrid::new(dims);
        let cpu = CpuRaster::new()
            .paint(&mut cpu_grid, &stamps, PaintOp::Paint)
            .unwrap();
        let gpu_report = gpu.paint(&mut gpu_grid, &stamps, PaintOp::Paint).unwrap();
        assert_eq!(gpu_grid, cpu_grid, "paint, case {case}");
        assert_eq!(gpu_report, cpu, "paint clamps, case {case}");
        assert!(cpu.clamped_events >= 5 * u64::from(layers));

        let half = &stamps[..stamps.len() / 2];
        let cpu = CpuRaster::new()
            .paint(&mut cpu_grid, half, PaintOp::Unpaint)
            .unwrap();
        let gpu_report = gpu.paint(&mut gpu_grid, half, PaintOp::Unpaint).unwrap();
        assert_eq!(gpu_grid, cpu_grid, "unpaint, case {case}");
        assert_eq!(gpu_report, cpu, "unpaint clamps, case {case}");
    }
}

#[test]
fn resident_paint_unpaint_round_trips_to_the_uploaded_grid() {
    let Some(mut gpu) = gpu_or_skip("resident_paint_unpaint_round_trips_to_the_uploaded_grid")
    else {
        return;
    };
    let dims = RasterDims::new(700, 500, 3).unwrap();
    let stamps = synthetic_stamps(dims, 6_000, 11);
    let prepared = prepare_stamps(dims, &stamps).unwrap();
    let mut base = CounterGrid::new(dims);
    CpuRaster::new().paint_prepared(&mut base, &prepared[..100], PaintOp::Paint);

    gpu.upload(&base).unwrap();
    let painted = gpu.paint_resident(&prepared, PaintOp::Paint).unwrap();
    assert!(painted.dispatch_count >= 1);
    let mut expected = base.clone();
    CpuRaster::new().paint_prepared(&mut expected, &prepared, PaintOp::Paint);
    let mut downloaded = CounterGrid::new(dims);
    gpu.download(&mut downloaded).unwrap();
    assert_eq!(downloaded, expected);

    gpu.paint_resident(&prepared, PaintOp::Unpaint).unwrap();
    gpu.download(&mut downloaded).unwrap();
    assert_eq!(downloaded, base);
}

#[test]
fn device_loss_reruns_on_the_cpu_and_latches() {
    let Some(gpu) = gpu_or_skip("device_loss_reruns_on_the_cpu_and_latches") else {
        return;
    };
    gpu.context().simulate_device_loss();
    let mut backend = LatchingRaster::gpu(gpu);
    assert_eq!(backend.backend_name(), "wgpu");

    let dims = RasterDims::new(64, 64, 2).unwrap();
    let stamps = synthetic_stamps(dims, 100, 3);
    let mut grid = CounterGrid::new(dims);
    let report = backend.paint(&mut grid, &stamps, PaintOp::Paint).unwrap();

    let mut expected = CounterGrid::new(dims);
    let cpu = CpuRaster::new()
        .paint(&mut expected, &stamps, PaintOp::Paint)
        .unwrap();
    assert_eq!(grid, expected);
    assert_eq!(report, cpu);
    assert_eq!(backend.backend_name(), "cpu");
    assert!(backend.latch_reason().is_some());
}

#[test]
fn auto_selection_picks_a_hardware_gpu_and_records_its_evidence() {
    let Some(gpu) = gpu_or_skip("auto_selection_picks_a_hardware_gpu_and_records_its_evidence")
    else {
        return;
    };
    let software = gpu.context().probe().is_software();
    drop(gpu);
    let selection =
        konnect_route_gpu::select_raster_backend(konnect_route_gpu::GpuPreference::Auto).unwrap();
    let adapter = selection.report.adapter.clone().unwrap();
    assert!(!adapter.name.is_empty());
    if software {
        // lavapipe and friends are slower than rayon: auto must refuse them.
        assert_eq!(selection.report.selected, "cpu");
        assert!(selection.report.reason.contains("software"));
        return;
    }
    assert_eq!(selection.report.selected, "wgpu");
    assert_eq!(selection.backend.backend_name(), "wgpu");
    assert!(selection.report.smoke_ms.is_some());
}

#[test]
fn clamp_counts_sum_across_dispatch_chunks() {
    let Some(mut gpu) = gpu_or_skip("clamp_counts_sum_across_dispatch_chunks") else {
        return;
    };
    // More stamps than one dispatch takes, all on one cell: every chunk
    // saturates it and counts clamps into its own slot.
    let dims = RasterDims::new(8, 4, 1).unwrap();
    let stamps = vec![Stamp::circle([8, 8], 0, 0, 0); 140_000];
    let prepared = prepare_stamps(dims, &stamps).unwrap();
    let mut gpu_grid = CounterGrid::new(dims);
    let stats = gpu
        .paint_round_trip(&mut gpu_grid, &prepared, PaintOp::Paint)
        .unwrap();
    assert!(
        stats.dispatch_count >= 3,
        "{} dispatches",
        stats.dispatch_count
    );
    let mut cpu_grid = CounterGrid::new(dims);
    let cpu = CpuRaster::new().paint_prepared(&mut cpu_grid, &prepared, PaintOp::Paint);
    assert_eq!(cpu.clamped_events, 140_000 - 255);
    assert_eq!(stats.report, cpu);
    assert_eq!(gpu_grid, cpu_grid);
}
