//! Global routing (S3) on a real board kept outside the repository, as a
//! benchmark and a determinism check. Nothing about that board is
//! committed: the directory comes from `KONNECT_DB1_FIXTURE_DIR`, the files
//! are found by extension, and the test asserts only invariants. Counts and
//! timings are printed; the overflow map and a summary are written to the
//! build's temporary directory (ignored, never committed).
//!
//! ```text
//! KONNECT_DB1_FIXTURE_DIR=/path/to/fixture cargo test --release \
//!     -p konnect-route --test global_real_fixture -- --nocapture
//! ```
//!
//! Optional knobs:
//! - `KONNECT_GLOBAL_ALL_NETS=1` also routes nets that own a plane (they
//!   normally join through plane fanout, S2);
//! - `KONNECT_GLOBAL_CLASS_LAYERS="ClassA=F.Cu;ClassB=F.Cu+B.Cu"` restricts
//!   net classes to layers;
//! - `KONNECT_GLOBAL_LAYERS="F.Cu,In2.Cu,B.Cu"` sets the routable layers
//!   (default: the layers KiCad marks signal or mixed);
//! - `KONNECT_GLOBAL_THREADS=n` sets the parallel run's thread count
//!   (default: every core);
//! - `KONNECT_GLOBAL_ITERATIONS=n` caps negotiation iterations;
//! - `KONNECT_GLOBAL_REFINE=n` sets the refinement pad threshold;
//! - `KONNECT_GLOBAL_SETTINGS='{"costs":{"astar_fac":1200}}'` overrides any
//!   [`GlobalSettings`] field (missing fields keep their defaults);
//! - `KONNECT_GLOBAL_QUICK=1` skips the 1-thread comparison run;
//! - `KONNECT_GLOBAL_DEBUG=1` prints where the overflow sits.

use konnect_geom::ArcPolicy;
use konnect_route::api::{ClassId, LayerMask, NetId, Stage};
use konnect_route::connectivity::ConnectivityReport;
use konnect_route::global::{
    build_grid_with, route_global_with, GCellGrid, GlobalResult, GlobalSettings, LayerPolicy,
};
use konnect_route::telemetry::Telemetry;
use konnect_route::{load_board_file, LoadOptions, ProjectRuleTable};
use std::collections::BTreeSet;
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

fn env_u32(name: &str) -> Option<u32> {
    std::env::var(name).ok()?.parse().ok()
}

/// A small FNV-1a hash, to print a fingerprint of a result.
fn fnv(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf29ce484222325u64, |h, &b| {
        (h ^ u64::from(b)).wrapping_mul(0x100000001b3)
    })
}

fn write_pgm(path: &Path, cols: u32, rows: u32, data: &[u32]) {
    let max = data.iter().copied().max().unwrap_or(0).max(1);
    let mut out = format!("P2\n{cols} {rows}\n255\n").into_bytes();
    for r in 0..rows {
        let line: Vec<String> = (0..cols)
            .map(|c| {
                let v = data[(r * cols + c) as usize];
                (if v == 0 { 0 } else { 40 + v * 215 / max }).to_string()
            })
            .collect();
        out.extend_from_slice(line.join(" ").as_bytes());
        out.push(b'\n');
    }
    let _ = std::fs::write(path, out);
}

fn summarize(label: &str, grid: &GCellGrid, r: &GlobalResult, tel: &Telemetry, wall_ms: u128) {
    println!(
        "[{label}] threads {}: {wall_ms} ms wall (stage {} ms); iterations {}, converged {}",
        tel.threads,
        tel.stage_us(Stage::Global) / 1000,
        r.iterations,
        r.stats.converged
    );
    println!(
        "[{label}] corridors {}, failed {}, overflowed connections {}",
        r.corridors.len(),
        r.failed.len(),
        r.overflowed.len()
    );
    println!(
        "[{label}] overflow total {}, max {}, overused resources {}, access overuse {}",
        r.stats.total_overflow,
        r.stats.max_overflow,
        r.stats.overused_resources,
        r.stats.access_overuse
    );
    println!(
        "[{label}] wirelength {:.1} m, vias {}, pattern seeds {}, maze seeds {}, full-grid searches {}",
        r.stats.wirelength_nm as f64 / 1e9,
        r.stats.vias,
        r.stats.pattern_seeds,
        r.stats.maze_seeds,
        r.stats.full_grid_searches
    );
    for it in &r.stats.per_iteration {
        println!(
            "[{label}]   iter {:>2}: pres {:>8.3}, routed {:>5}, overflow {:>6}, overused {:>5}",
            it.iteration,
            f64::from(it.pres_fac) / 1000.0,
            it.routed,
            it.overflow,
            it.overused
        );
    }
    let per_layer: Vec<u64> = (0..grid.layer_slots())
        .map(|s| {
            r.overflow_layer(grid, s)
                .iter()
                .map(|&v| u64::from(v))
                .sum()
        })
        .collect();
    println!("[{label}] overflow map sum per layer slot: {per_layer:?}");
    println!(
        "[{label}] A* expansions: {} searches, mean {:.0}, max {}; window nodes mean {:.0}, max {}",
        tel.expansions_per_connection.total,
        tel.expansions_per_connection.mean(),
        tel.expansions_per_connection.max,
        tel.window_nodes.mean(),
        tel.window_nodes.max
    );
    println!(
        "[{label}] congested-iteration time {} ms",
        tel.congested_iteration_us / 1000
    );
    println!("[{label}] counters: {:?}", tel.counters);
}

#[test]
fn global_routing_on_the_real_fixture() {
    let Some(dir) = std::env::var_os("KONNECT_DB1_FIXTURE_DIR").map(PathBuf::from) else {
        eprintln!("skipping: KONNECT_DB1_FIXTURE_DIR is not set");
        return;
    };
    let Some(pcb) = only_file(&dir, "kicad_pcb") else {
        eprintln!("skipping: {} holds no single .kicad_pcb", dir.display());
        return;
    };
    let Some(pro) = only_file(&dir, "kicad_pro") else {
        eprintln!("skipping: {} holds no single .kicad_pro", dir.display());
        return;
    };

    let t = Instant::now();
    let options = LoadOptions {
        routable_layers: std::env::var("KONNECT_GLOBAL_LAYERS")
            .ok()
            .map(|v| v.split(',').map(str::to_string).collect()),
        ..Default::default()
    };
    let (mut board, _) = load_board_file(&pcb, &options).expect("board loads");
    let pro_text = std::fs::read_to_string(&pro).expect("project reads");
    let dru_text =
        only_file(&dir, "kicad_dru").map(|p| std::fs::read_to_string(p).expect("rules read"));
    let rules = ProjectRuleTable::from_project(&pro_text, dru_text.as_deref(), &board)
        .expect("rules compile");
    rules.assign_classes(&mut board);
    let conn = ConnectivityReport::build(&board, &ArcPolicy::default());
    let all = conn.connections(&board);
    println!(
        "load + rules + connectivity: {} ms",
        t.elapsed().as_millis()
    );

    let plane_nets: BTreeSet<NetId> = board.zones.iter().filter_map(|z| z.net).collect();
    let include_planes = std::env::var("KONNECT_GLOBAL_ALL_NETS").is_ok_and(|v| v == "1");
    let connections: Vec<_> = all
        .iter()
        .filter(|c| include_planes || !plane_nets.contains(&c.net))
        .copied()
        .collect();
    println!(
        "connections: {} total, {} on {} plane nets, routing {}",
        all.len(),
        all.len() - all.iter().filter(|c| !plane_nets.contains(&c.net)).count(),
        plane_nets.len(),
        connections.len()
    );

    let mut policy = LayerPolicy::default();
    if let Ok(spec) = std::env::var("KONNECT_GLOBAL_CLASS_LAYERS") {
        for part in spec.split(';').filter(|s| !s.is_empty()) {
            let (class, layers) = part.split_once('=').expect("Class=Layer+Layer");
            let idx = rules
                .classes()
                .iter()
                .position(|c| c.name == class)
                .unwrap_or_else(|| panic!("no net class '{class}'"));
            let mask = layers
                .split('+')
                .map(|l| board.layers.expand(l))
                .fold(LayerMask::EMPTY, LayerMask::union);
            policy.class_layers.insert(ClassId(idx as u16), mask);
        }
        println!(
            "class layer masks: {} classes restricted",
            policy.class_layers.len()
        );
    }

    let threads = env_u32("KONNECT_GLOBAL_THREADS")
        .unwrap_or_else(|| std::thread::available_parallelism().map_or(4, |n| n.get() as u32));
    let defaults: GlobalSettings = std::env::var("KONNECT_GLOBAL_SETTINGS")
        .ok()
        .map(|j| serde_json::from_str(&j).expect("KONNECT_GLOBAL_SETTINGS is GlobalSettings JSON"))
        .unwrap_or_default();
    let base = GlobalSettings {
        max_iterations: env_u32("KONNECT_GLOBAL_ITERATIONS").unwrap_or(defaults.max_iterations),
        refine_min_pads: env_u32("KONNECT_GLOBAL_REFINE").unwrap_or(defaults.refine_min_pads),
        ..defaults
    };

    let run = |threads: u32| {
        let settings = GlobalSettings { threads, ..base };
        let mut tel = Telemetry::default();
        let start = Instant::now();
        let grid = build_grid_with(&board, &rules, &settings, &mut tel);
        let r = route_global_with(
            &board,
            &rules,
            &grid,
            &connections,
            &settings,
            &policy,
            &mut tel,
        );
        let wall = start.elapsed().as_millis();
        (grid, r, tel, wall)
    };

    let (grid_n, r_n, tel_n, wall_n) = run(threads);
    let zero_cap = grid_n.capacity.iter().filter(|&&c| c == 0).count();
    println!(
        "grid: {} x {} base cells x {} layers, {} refined (pitch {} nm, fine {} nm), {} ids, {} resources ({} at zero capacity)",
        grid_n.cols,
        grid_n.rows,
        grid_n.layer_slots(),
        grid_n.refined.len(),
        grid_n.pitch,
        grid_n.fine_pitch,
        grid_n.cell_count(),
        grid_n.capacity.len(),
        zero_cap
    );
    summarize("N", &grid_n, &r_n, &tel_n, wall_n);
    if std::env::var("KONNECT_GLOBAL_DEBUG").is_ok_and(|v| v == "1") {
        // Where the overflow sits, by the footprint courtyard it falls in
        // (printed only; never written).
        let mut hot: Vec<(u16, u32)> = r_n
            .overflow
            .iter()
            .enumerate()
            .filter(|(_, &v)| v > 0)
            .map(|(id, &v)| (v, id as u32))
            .collect();
        hot.sort_by(|a, b| b.cmp(a));
        let mut by_fp: std::collections::BTreeMap<String, (u32, u64)> = Default::default();
        for &(v, id) in &hot {
            let (_, b) = grid_n.cell_bbox(id).unwrap();
            let c = b.center();
            let fp = board
                .footprints
                .iter()
                .find(|f| f.courtyard_bbox().is_some_and(|cb| cb.contains(c)))
                .and_then(|f| f.reference.clone())
                .unwrap_or_else(|| "-".to_string());
            let e = by_fp.entry(fp).or_default();
            e.0 += 1;
            e.1 += u64::from(v);
        }
        let mut v: Vec<_> = by_fp.into_iter().collect();
        v.sort_by_key(|e| std::cmp::Reverse(e.1 .1));
        println!(
            "overflow by courtyard (cells, sum): {:?}",
            &v[..v.len().min(25)]
        );
        for &(v, id) in hot.iter().take(15) {
            let (layer, b) = grid_n.cell_bbox(id).unwrap();
            let (x, y) = b.center().to_mm();
            println!(
                "  hot cell {id}: {v} on {} at ({x:.2}, {y:.2}) size {} nm",
                board.layers.name(layer),
                b.width()
            );
        }
        let pad_name = |item: konnect_route::api::ItemRef| match item {
            konnect_route::api::ItemRef::Pad(p) => {
                let pad = &board.pads[p.index()];
                let fp = &board.footprints[pad.footprint.index()];
                format!(
                    "{}.{} on {:?}",
                    fp.reference.as_deref().unwrap_or("?"),
                    pad.number,
                    pad.layers
                        .iter()
                        .map(|l| board.layers.name(l))
                        .collect::<Vec<_>>()
                )
            }
            other => format!("{other:?}"),
        };
        for (id, reason) in &r_n.failed {
            let c = connections.iter().find(|c| c.id == *id).unwrap();
            println!(
                "  failed {}: {} -> {} ({reason:?})",
                board.net(c.net).name,
                pad_name(c.from),
                pad_name(c.to)
            );
        }
    }

    if std::env::var("KONNECT_GLOBAL_QUICK").is_ok_and(|v| v == "1") {
        println!("quick mode: skipping the 1-thread comparison");
        return;
    }
    let (grid_1, r_1, tel_1, wall_1) = run(1);
    summarize("1", &grid_1, &r_1, &tel_1, wall_1);

    let json_n = serde_json::to_vec(&r_n).expect("result serializes");
    let json_1 = serde_json::to_vec(&r_1).expect("result serializes");
    println!(
        "fingerprints: 1 thread {:016x}, {} threads {:016x}; speedup {:.2}x",
        fnv(&json_1),
        tel_n.threads,
        fnv(&json_n),
        wall_1 as f64 / wall_n.max(1) as f64
    );
    assert_eq!(grid_1, grid_n, "the grid does not depend on threads");
    assert_eq!(r_1, r_n, "the result does not depend on threads");
    assert_eq!(json_1, json_n);

    // Invariants that hold for any board.
    assert_eq!(r_n.corridors.len() + r_n.failed.len(), connections.len());
    assert_eq!(r_n.overflow.len(), grid_n.cell_count() as usize);
    for c in &r_n.corridors {
        assert!(!c.cells.is_empty());
        assert!(c.cells.iter().all(|&id| id < grid_n.cell_count()));
        assert!(c.layers.0 & !grid_n.layers.0 == 0);
    }

    // Generated evidence goes to the build's temp dir, never the repo.
    let out = Path::new(env!("CARGO_TARGET_TMPDIR")).join("global-real-fixture");
    if std::fs::create_dir_all(&out).is_ok() {
        for slot in 0..grid_n.layer_slots() {
            let layer = r_n.overflow_layer(&grid_n, slot);
            write_pgm(
                &out.join(format!("overflow-slot{slot}.pgm")),
                grid_n.cols,
                grid_n.rows,
                &layer,
            );
        }
        let summary = serde_json::json!({
            "stats": r_n.stats,
            "iterations": r_n.iterations,
            "failed": r_n.failed.len(),
            "overflowed": r_n.overflowed.len(),
            "telemetry": tel_n,
        });
        let _ = std::fs::write(
            out.join("summary.json"),
            serde_json::to_vec_pretty(&summary).unwrap_or_default(),
        );
        println!("evidence written to {}", out.display());
    }
}
