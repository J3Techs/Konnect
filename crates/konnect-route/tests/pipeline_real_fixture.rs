//! Runs the whole native pipeline ([`konnect_route::pipeline::route_board`])
//! on a real board from outside the repository and prints what each stage
//! did and how long it took. Nothing about the board is committed: the
//! directory comes from `KONNECT_DB1_FIXTURE_DIR`, files are found by
//! extension, and only board-independent invariants are asserted (every
//! task is routed or failed with a reason, the plan validates, the plan
//! never leaves more connections open than the board had).
//!
//! ```text
//! KONNECT_DB1_FIXTURE_DIR=/path/to/fixture cargo test -p konnect-route --release \
//!     --test pipeline_real_fixture -- --nocapture
//! ```
//!
//! Optional knobs, all environment variables:
//!
//! - `KONNECT_PIPELINE_THREADS` (default 0: the whole machine);
//! - `KONNECT_PIPELINE_NEGOTIATE_THREADS` (default: the pipeline's);
//! - `KONNECT_PIPELINE_ITERATIONS` (negotiation, default 10);
//! - `KONNECT_PIPELINE_REGION=x0,y0,x1,y1` in mm;
//! - `KONNECT_PIPELINE_NO_FANOUT=1`, `KONNECT_PIPELINE_NO_ACCESS=1`,
//!   `KONNECT_PIPELINE_NO_OPTIMIZE=1`;
//! - `KONNECT_PIPELINE_REFERENCE=1`: negotiation's reference search instead
//!   of the kernel, for comparison;
//! - `KONNECT_PIPELINE_MIN_RUN=1`: ask the kernel for minimum runs;
//! - `KONNECT_PIPELINE_MAX_EXPANSIONS`: the kernel's budget per search;
//! - `KONNECT_PIPELINE_ASTAR`: the kernel's heuristic weight, ×1000;
//! - `KONNECT_PIPELINE_GRID_CORE_MM=w,h`: negotiation's grid tile core size;
//! - `KONNECT_PIPELINE_OUT=dir`: write `summary.json` and `plan.json` there
//!   (keep it in an ignored build location).

use konnect_geom::{BBox, Point};
use konnect_route::api::Stage;
use konnect_route::detail::negotiate::ReferenceSearch;
use konnect_route::pipeline::{
    route_board, route_board_with, PipelineResult, PipelineScope, PipelineSettings,
};
use konnect_route::{load_board_file, LoadOptions, ProjectRuleTable};
use serde_json::json;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::time::Instant;

fn only_file(dir: &Path, ext: &str) -> Option<PathBuf> {
    let mut found = std::fs::read_dir(dir)
        .ok()?
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|e| e.to_str()) == Some(ext));
    let first = found.next()?;
    found.next().is_none().then_some(first)
}

fn env_num(name: &str, default: u64) -> u64 {
    std::env::var(name)
        .ok()
        .and_then(|v| v.trim().parse().ok())
        .unwrap_or(default)
}

fn env_on(name: &str) -> bool {
    std::env::var(name).as_deref() == Ok("1")
}

#[test]
fn pipeline_real_board_fixture() {
    let Some(dir) = std::env::var_os("KONNECT_DB1_FIXTURE_DIR").map(PathBuf::from) else {
        eprintln!("skipping: KONNECT_DB1_FIXTURE_DIR is not set");
        return;
    };
    let (Some(pcb), Some(pro)) = (only_file(&dir, "kicad_pcb"), only_file(&dir, "kicad_pro"))
    else {
        eprintln!(
            "skipping: {} lacks a single .kicad_pcb and .kicad_pro",
            dir.display()
        );
        return;
    };
    let t0 = Instant::now();
    let (mut board, _) = load_board_file(&pcb, &LoadOptions::default()).expect("board loads");
    let pro_text = std::fs::read_to_string(&pro).expect("project reads");
    let dru_text =
        only_file(&dir, "kicad_dru").map(|p| std::fs::read_to_string(p).expect("rules read"));
    let rules = ProjectRuleTable::from_project(&pro_text, dru_text.as_deref(), &board)
        .expect("rules compile");
    rules.assign_classes(&mut board);
    let load_ms = t0.elapsed().as_millis();
    println!("load + rules: {load_ms} ms");

    let region = std::env::var("KONNECT_PIPELINE_REGION").ok().map(|s| {
        let v: Vec<f64> = s
            .split(',')
            .map(|x| x.trim().parse().expect("mm"))
            .collect();
        assert_eq!(v.len(), 4, "x0,y0,x1,y1");
        BBox::new(
            Point::from_mm(v[0], v[1]).unwrap(),
            Point::from_mm(v[2], v[3]).unwrap(),
        )
    });
    let scope = PipelineScope { nets: None, region };
    let mut settings = PipelineSettings {
        threads: env_num("KONNECT_PIPELINE_THREADS", 0) as usize,
        ..PipelineSettings::default()
    };
    settings.negotiation.threads = env_num("KONNECT_PIPELINE_NEGOTIATE_THREADS", 0) as usize;
    settings.negotiation.max_iterations = env_num("KONNECT_PIPELINE_ITERATIONS", 10) as u32;
    settings.search.max_expansions = env_num(
        "KONNECT_PIPELINE_MAX_EXPANSIONS",
        settings.search.max_expansions,
    );
    settings.search.astar_factor = env_num(
        "KONNECT_PIPELINE_ASTAR",
        u64::from(settings.search.astar_factor),
    ) as u32;
    settings.search_min_run = env_on("KONNECT_PIPELINE_MIN_RUN");
    if let Ok(v) = std::env::var("KONNECT_PIPELINE_GRID_CORE_MM") {
        let mm: Vec<f64> = v
            .split(',')
            .map(|x| x.trim().parse().expect("mm"))
            .collect();
        assert_eq!(mm.len(), 2, "width,height");
        settings.tuning.grid_core = (
            konnect_geom::mm_to_nm(mm[0]).expect("width"),
            konnect_geom::mm_to_nm(mm[1]).expect("height"),
        );
    }
    if env_on("KONNECT_PIPELINE_NO_FANOUT") {
        settings.fanout = None;
    }
    if env_on("KONNECT_PIPELINE_NO_ACCESS") {
        settings.access = None;
    }
    if env_on("KONNECT_PIPELINE_NO_OPTIMIZE") {
        settings.optimize = None;
    }
    let reference = env_on("KONNECT_PIPELINE_REFERENCE");

    let t = Instant::now();
    let r: PipelineResult = if reference {
        let mut backend = ReferenceSearch::new(settings.negotiation.threads);
        route_board_with(&board, &rules, &scope, &settings, &mut backend)
    } else {
        route_board(&board, &rules, &scope, &settings)
    }
    .expect("the pipeline plans");
    let wall_ms = t.elapsed().as_millis();

    let backend = if reference { "reference" } else { "kernel" };
    println!(
        "pipeline ({backend}): {wall_ms} ms wall on {} threads",
        r.telemetry.threads
    );
    let mut stages = BTreeMap::new();
    for s in [
        Stage::Audit,
        Stage::Fanout,
        Stage::Access,
        Stage::Global,
        Stage::Detail,
        Stage::Optimize,
        Stage::Check,
        Stage::Plan,
    ] {
        let ms = r.telemetry.stage_us(s) as f64 / 1000.0;
        println!("  {s:?}: {ms:.1} ms");
        stages.insert(format!("{s:?}"), ms);
    }
    let fanout_sites = r.fanout.as_ref().map_or(0, |f| f.sites.len());
    let fanout_failed = r.fanout.as_ref().map_or(0, |f| f.failed.len());
    println!(
        "connections: {} on the board; fanout {} sites ({} pads failed); {} tasks after it",
        r.connections_before_fanout,
        fanout_sites,
        fanout_failed,
        r.connections.len()
    );
    let no_access: usize = r.access_stats.iter().map(|(_, s)| s.no_access).sum();
    println!(
        "access: {} pads over {} classes, {} with no point",
        r.access.len(),
        r.access_stats.len(),
        no_access
    );
    println!(
        "global: overflow {} (max {}), {} failed, {} iterations, best {}",
        r.global.stats.total_overflow,
        r.global.stats.max_overflow,
        r.global.failed.len(),
        r.global.iterations,
        r.global.stats.best_iteration
    );
    let mut reasons: BTreeMap<&str, usize> = BTreeMap::new();
    for (_, d) in &r.detail.failure_details {
        *reasons.entry(d.name()).or_default() += 1;
    }
    println!(
        "detail: routed {} of {} ({:.1} %), failed {:?}, {} iterations, {} short segments",
        r.detail.routed.len(),
        r.connections.len(),
        100.0 * r.detail.routed.len() as f64 / r.connections.len().max(1) as f64,
        reasons,
        r.detail.iterations,
        r.detail.short_segments.len()
    );
    for it in &r.detail.reports {
        println!(
            "  it {:2} routed {:5} failed {:5} conflicted {:5} pieces {:5}",
            it.iteration, it.routed, it.failed, it.conflicted, it.pieces
        );
    }
    if let Some(o) = &r.optimize {
        println!(
            "optimize: tracks {} -> {}, vias {} -> {}, length {:.1} -> {:.1} mm, defects {} -> {}, {} nets put back by the guard",
            o.tracks_before,
            o.tracks_after,
            o.vias_before,
            o.vias_after,
            o.length_before_nm as f64 / 1e6,
            o.length_after_nm as f64 / 1e6,
            o.defects_before,
            o.defects_after,
            r.optimize_reverted.len()
        );
    }
    let by_kind = r.check.counts();
    println!("check: {} findings {:?}", r.check.violations.len(), by_kind);
    println!(
        "plan: {} tracks, {} vias; connections open {} -> {}",
        r.plan.tracks.len(),
        r.plan.vias.len(),
        r.connections_before_fanout,
        r.connections_after
    );
    let counters: BTreeMap<_, _> = r
        .telemetry
        .counters
        .iter()
        .filter(|(k, _)| {
            k.starts_with("pipeline_") || k.starts_with("search_") || k.starts_with("optimize_")
        })
        .collect();
    println!("counters: {counters:?}");

    // Board-independent invariants.
    assert_eq!(
        r.detail.routed.len() + r.detail.failed.len(),
        r.connections.len(),
        "every task is routed or failed"
    );
    assert_eq!(r.detail.failed.len(), r.detail.failure_details.len());
    if r.plan.item_count() > 0 {
        r.plan.validate().expect("the plan validates");
    }
    assert!(r.connections_after <= r.connections_before_fanout);

    if let Some(out) = std::env::var_os("KONNECT_PIPELINE_OUT").map(PathBuf::from) {
        std::fs::create_dir_all(&out).expect("output directory");
        let summary = json!({
            "backend": backend,
            "wall_ms": wall_ms,
            "load_ms": load_ms,
            "threads": r.telemetry.threads,
            "stages_ms": stages,
            "connections_before": r.connections_before_fanout,
            "fanout_sites": fanout_sites,
            "tasks": r.connections.len(),
            "global_overflow": r.global.stats.total_overflow,
            "routed": r.detail.routed.len(),
            "failed": reasons,
            "check": by_kind.iter().map(|(k, v)| (format!("{k:?}"), *v)).collect::<BTreeMap<_, _>>(),
            "connections_after": r.connections_after,
            "plan_tracks": r.plan.tracks.len(),
            "plan_vias": r.plan.vias.len(),
            "optimize": r.optimize,
            "telemetry": r.telemetry,
        });
        std::fs::write(
            out.join("summary.json"),
            serde_json::to_string_pretty(&summary).unwrap(),
        )
        .expect("summary written");
        std::fs::write(out.join("plan.json"), r.plan.to_routing_file_json()).expect("plan written");
        println!("wrote {}", out.display());
    }
}
