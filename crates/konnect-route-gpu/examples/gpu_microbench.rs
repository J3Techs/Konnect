//! GPU microbenchmarks for the G1 toolchain gate.
//!
//! Measures, on the machine it runs on:
//! - submit + `map_async` round-trip latency,
//! - empty-dispatch overhead (one per submit, and many per submit),
//! - the raster paint kernel against its rayon twin, end to end (upload,
//!   kernel, readback) and resident (kernel and a 4-byte readback only),
//!   with a bit-exact parity check of every result.
//!
//! Workloads are synthetic; nothing comes from a real board.
//!
//! ```text
//! cargo run --release -p konnect-route-gpu --features gpu --example gpu_microbench -- \
//!     [--out <path.json>] [--quick]
//! ```
//!
//! Without `--out` the JSON report is printed to stdout.

use std::path::PathBuf;
use std::process::ExitCode;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use konnect_route_gpu::gpu::{bench, GpuContext, GpuRaster};
use konnect_route_gpu::synthetic::synthetic_stamps;
use konnect_route_gpu::{
    prepare_stamps, CounterGrid, CpuRaster, PaintOp, RasterCompute, RasterDims,
};
use serde_json::{json, Value};

/// G1 pass conditions (gpu.md section 5).
const G1_MAX_ROUND_TRIP_US: f64 = 1000.0;
const G1_MIN_RASTER_SPEEDUP: f64 = 2.0;
/// The per-dispatch budget the planner must respect on this shared GPU.
const DISPATCH_BUDGET_MS: f64 = 50.0;

struct Workload {
    name: &'static str,
    width: u32,
    height: u32,
    layers: u32,
    stamps: usize,
}

const WORKLOADS: &[Workload] = &[
    // One routing window's commit batch.
    Workload {
        name: "window_512x512x2_2k",
        width: 512,
        height: 512,
        layers: 2,
        stamps: 2_000,
    },
    // A ~100 mm board tile at 50 um, three signal layers.
    Workload {
        name: "tile_2048x2048x3_20k",
        width: 2048,
        height: 2048,
        layers: 3,
        stamps: 20_000,
    },
    // A whole-region rebuild.
    Workload {
        name: "region_4096x4096x4_100k",
        width: 4096,
        height: 4096,
        layers: 4,
        stamps: 100_000,
    },
];

/// The workload G1's raster condition is judged on.
const G1_WORKLOAD: &str = "tile_2048x2048x3_20k";

fn main() -> ExitCode {
    let mut out: Option<PathBuf> = None;
    let mut quick = false;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--out" => match args.next() {
                Some(path) => out = Some(PathBuf::from(path)),
                None => return usage("--out needs a path"),
            },
            "--quick" => quick = true,
            other => return usage(&format!("unknown argument '{other}'")),
        }
    }

    let report = match run(quick) {
        Ok(report) => report,
        Err(e) => {
            eprintln!("gpu_microbench: {e}");
            return ExitCode::FAILURE;
        }
    };
    let text = serde_json::to_string_pretty(&report).expect("report serializes");
    match out {
        Some(path) => {
            if let Some(parent) = path.parent() {
                if let Err(e) = std::fs::create_dir_all(parent) {
                    eprintln!("gpu_microbench: cannot create {}: {e}", parent.display());
                    return ExitCode::FAILURE;
                }
            }
            if let Err(e) = std::fs::write(&path, text + "\n") {
                eprintln!("gpu_microbench: cannot write {}: {e}", path.display());
                return ExitCode::FAILURE;
            }
            eprintln!("gpu_microbench: wrote {}", path.display());
        }
        None => println!("{text}"),
    }
    if report["parity"]["all_bit_exact"] == Value::Bool(true) {
        ExitCode::SUCCESS
    } else {
        eprintln!("gpu_microbench: parity FAILED");
        ExitCode::FAILURE
    }
}

fn usage(problem: &str) -> ExitCode {
    eprintln!("gpu_microbench: {problem}\nusage: gpu_microbench [--out <path.json>] [--quick]");
    ExitCode::FAILURE
}

fn run(quick: bool) -> Result<Value, Box<dyn std::error::Error>> {
    let started = Instant::now();
    let context = GpuContext::new()?;
    let probe = context.probe().clone();
    eprintln!(
        "adapter: {} ({}, {})",
        probe.name, probe.backend, probe.driver_info
    );

    let latency_iterations = if quick { 50 } else { 300 };
    let round_trip = bench::round_trip_latency(&context, latency_iterations)?;

    let noop = bench::noop_pipeline(&context)?;
    let mut single = Vec::with_capacity(latency_iterations);
    for _ in 0..latency_iterations {
        single.push(bench::empty_dispatches(&context, &noop, 1)?);
    }
    const BATCH: u32 = 1000;
    let mut batched = Vec::new();
    for _ in 0..if quick { 3 } else { 10 } {
        batched.push(bench::empty_dispatches(&context, &noop, BATCH)? / BATCH);
    }

    let mut gpu = GpuRaster::new(context)?;
    let smoke = gpu.smoke()?;

    let iterations = if quick { 3 } else { 7 };
    let workloads: Vec<&Workload> = if quick {
        WORKLOADS.iter().take(2).collect()
    } else {
        WORKLOADS.iter().collect()
    };
    let mut raster = Vec::new();
    let mut all_bit_exact = true;
    let mut g1_speedup = None;
    let mut worst_dispatch_ms: f64 = 0.0;
    for workload in workloads {
        eprintln!("raster workload {}", workload.name);
        let result = bench_raster(&mut gpu, workload, iterations)?;
        all_bit_exact &= result["parity"]["bit_exact"] == Value::Bool(true);
        if workload.name == G1_WORKLOAD {
            g1_speedup = result["speedup"]["end_to_end"].as_f64();
        }
        worst_dispatch_ms =
            worst_dispatch_ms.max(result["gpu_mean_ms_per_dispatch"].as_f64().unwrap_or(0.0));
        raster.push(result);
    }

    let round_trip_median_us = median(&round_trip).as_secs_f64() * 1e6;
    let g1_round_trip = round_trip_median_us <= G1_MAX_ROUND_TRIP_US;
    let g1_raster = g1_speedup.map(|s| s >= G1_MIN_RASTER_SPEEDUP);

    Ok(json!({
        "tool": "konnect-route-gpu gpu_microbench",
        "unix_time_s": SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0),
        "quick": quick,
        "adapter": probe,
        "cpu_threads": rayon::current_num_threads(),
        "smoke_ms": ms(smoke),
        "round_trip_256_bytes": summary(&round_trip),
        "empty_dispatch_submit_and_wait": summary(&single),
        "empty_dispatch_batched_per_dispatch": {
            "dispatches_per_submit": BATCH,
            "timing": summary(&batched),
        },
        "raster_paint": raster,
        "parity": { "all_bit_exact": all_bit_exact },
        "dispatch_budget": {
            "budget_ms": DISPATCH_BUDGET_MS,
            "worst_mean_ms_per_dispatch": worst_dispatch_ms,
            "within_budget": worst_dispatch_ms <= DISPATCH_BUDGET_MS,
            "note": "mean of resident paint wall time over its dispatches; not a per-dispatch timestamp",
        },
        "g1": {
            "round_trip_median_us": round_trip_median_us,
            "round_trip_pass": g1_round_trip,
            "raster_workload": G1_WORKLOAD,
            "raster_end_to_end_speedup": g1_speedup,
            "raster_pass": g1_raster,
            "pass": g1_round_trip && g1_raster == Some(true),
        },
        "total_s": started.elapsed().as_secs_f64(),
    }))
}

fn bench_raster(
    gpu: &mut GpuRaster,
    workload: &Workload,
    iterations: usize,
) -> Result<Value, Box<dyn std::error::Error>> {
    let dims = RasterDims::new(workload.width, workload.height, workload.layers)?;
    let stamps = synthetic_stamps(dims, workload.stamps, 0xB0A2D ^ workload.stamps as u64);
    let prepared = prepare_stamps(dims, &stamps)?;
    let base = CounterGrid::new(dims);
    let mut cpu = CpuRaster::new();

    // CPU twin, including stamp preparation, as a caller would run it.
    let mut cpu_times = Vec::with_capacity(iterations);
    let mut cpu_grid = base.clone();
    let mut cpu_report = Default::default();
    for _ in 0..=iterations {
        let mut grid = base.clone();
        let t = Instant::now();
        cpu_report = cpu.paint(&mut grid, &stamps, PaintOp::Paint)?;
        cpu_times.push(t.elapsed());
        cpu_grid = grid;
    }
    cpu_times.remove(0); // warm-up

    // GPU end to end: prepare, upload, kernel, readback, one submission.
    let mut gpu_times = Vec::with_capacity(iterations);
    let mut gpu_grid = base.clone();
    let mut gpu_report = Default::default();
    for _ in 0..=iterations {
        let mut grid = base.clone();
        let t = Instant::now();
        gpu_report = gpu.paint(&mut grid, &stamps, PaintOp::Paint)?;
        gpu_times.push(t.elapsed());
        gpu_grid = grid;
    }
    gpu_times.remove(0);

    let paint_exact = gpu_grid == cpu_grid && gpu_report == cpu_report;
    let differing_words = gpu_grid
        .words()
        .iter()
        .zip(cpu_grid.words())
        .filter(|(g, c)| g != c)
        .count();

    let occupied_cells = cpu_grid.occupied_cells();
    let cpu_unpaint = cpu.paint(&mut cpu_grid, &stamps, PaintOp::Unpaint)?;
    let gpu_unpaint = gpu.paint(&mut gpu_grid, &stamps, PaintOp::Unpaint)?;
    let unpaint_exact = gpu_grid == cpu_grid && gpu_unpaint == cpu_unpaint;

    // Resident: the grid stays on the device; stamps go up, 4 bytes come
    // back. Paint and unpaint alternate so the raster returns to `base`.
    let t = Instant::now();
    gpu.upload(&base)?;
    gpu.paint_resident(&[], PaintOp::Paint)?;
    let upload = t.elapsed();
    let mut resident_paint = Vec::with_capacity(iterations);
    let mut resident_unpaint = Vec::with_capacity(iterations);
    let mut dispatch_count = 0;
    for _ in 0..=iterations {
        let t = Instant::now();
        dispatch_count = gpu
            .paint_resident(&prepared, PaintOp::Paint)?
            .dispatch_count;
        resident_paint.push(t.elapsed());
        let t = Instant::now();
        gpu.paint_resident(&prepared, PaintOp::Unpaint)?;
        resident_unpaint.push(t.elapsed());
    }
    resident_paint.remove(0);
    resident_unpaint.remove(0);
    let t = Instant::now();
    let mut downloaded = CounterGrid::new(dims);
    gpu.download(&mut downloaded)?;
    let download = t.elapsed();
    let resident_exact = downloaded == base;

    let cpu_median = median(&cpu_times).as_secs_f64();
    let gpu_median = median(&gpu_times).as_secs_f64();
    let resident_median = median(&resident_paint).as_secs_f64();
    Ok(json!({
        "name": workload.name,
        "dims": { "width": dims.width, "height": dims.height, "layers": dims.layers },
        "grid_bytes": dims.total_words() * 4,
        "stamps": stamps.len(),
        "stamps_on_grid": prepared.len(),
        "work_words": prepared.iter().map(|s| s.work_words()).sum::<u64>(),
        "occupied_cells": occupied_cells,
        "clamped_events": cpu_report.clamped_events,
        "cpu_rayon_paint": summary(&cpu_times),
        "gpu_end_to_end_paint": summary(&gpu_times),
        "gpu_resident_paint": summary(&resident_paint),
        "gpu_resident_unpaint": summary(&resident_unpaint),
        "gpu_upload_plus_round_trip_ms": ms(upload),
        "gpu_download_ms": ms(download),
        "gpu_dispatches_per_paint": dispatch_count,
        "gpu_mean_ms_per_dispatch": resident_median * 1e3 / f64::from(dispatch_count.max(1)),
        "speedup": {
            "end_to_end": cpu_median / gpu_median,
            "resident": cpu_median / resident_median,
        },
        "parity": {
            "bit_exact": paint_exact && unpaint_exact && resident_exact,
            "paint": paint_exact,
            "paint_differing_words": differing_words,
            "unpaint": unpaint_exact,
            "resident_paint_unpaint_returns_to_base": resident_exact,
        },
    }))
}

fn median(samples: &[Duration]) -> Duration {
    let mut sorted = samples.to_vec();
    sorted.sort();
    sorted.get(sorted.len() / 2).copied().unwrap_or_default()
}

fn ms(d: Duration) -> f64 {
    d.as_secs_f64() * 1e3
}

fn summary(samples: &[Duration]) -> Value {
    let mut sorted = samples.to_vec();
    sorted.sort();
    let us = |d: Duration| d.as_secs_f64() * 1e6;
    let at = |q: f64| {
        sorted
            .get(((sorted.len() as f64 - 1.0) * q).round() as usize)
            .copied()
            .unwrap_or_default()
    };
    json!({
        "samples": sorted.len(),
        "min_us": us(at(0.0)),
        "median_us": us(at(0.5)),
        "p90_us": us(at(0.9)),
        "max_us": us(at(1.0)),
    })
}
