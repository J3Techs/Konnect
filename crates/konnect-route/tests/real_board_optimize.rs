//! Runs the S6 optimizer over a real routed board from outside the
//! repository and prints what it did. Nothing about that board is committed.
//!
//! The routed board is the single `.kicad_pcb` found in a subdirectory of
//! `KONNECT_DB1_FIXTURE_DIR` (the project's `.kicad_pro` and `.kicad_dru`
//! sit in the directory itself). Its unlocked tracks and vias become routed
//! copper; everything else stays board. The test asserts only what must hold
//! for any board: the optimizer adds no violation of any kind, loses no
//! connection, and gives the same copper on one thread and on many.
//!
//! ```text
//! KONNECT_DB1_FIXTURE_DIR=/path/to/fixture cargo test --release -p konnect-route \
//!     --test real_board_optimize -- --nocapture
//! ```

mod common;

use konnect_geom::ArcPolicy;
use konnect_route::api::Generation;
use konnect_route::connectivity::ConnectivityReport;
use konnect_route::optimize::exact::{CheckerOptions, ExactChecker};
use konnect_route::optimize::{geometry_defects, optimize_checked, OptimizeOptions};
use konnect_route::plan::RoutedCopper;
use konnect_route::{load_board_file, LoadOptions, ProjectRuleTable, RouteBoard, RoutePlan};
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

fn routed_board(dir: &Path) -> Option<PathBuf> {
    let mut subdirs: Vec<PathBuf> = std::fs::read_dir(dir)
        .ok()?
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.is_dir())
        .collect();
    subdirs.sort();
    subdirs.iter().find_map(|d| only_file(d, "kicad_pcb"))
}

/// Move the board's unlocked tracks and vias into routed copper.
fn split(board: &mut RouteBoard) -> RoutedCopper {
    RoutedCopper::take_unlocked(board, None, Generation(0))
}

/// The board with `copper` put back as board copper, for the union-find.
fn joined(board: &RouteBoard, copper: &RoutedCopper) -> RouteBoard {
    let mut b = board.clone();
    copper.freeze_into(&mut b);
    b
}

fn by_kind(v: &[konnect_route::check::Violation]) -> BTreeMap<String, usize> {
    let mut m = BTreeMap::new();
    for x in v {
        *m.entry(format!("{:?}", x.kind)).or_default() += 1;
    }
    m
}

#[test]
fn optimizer_on_a_real_routed_board() {
    let Some(dir) = std::env::var_os("KONNECT_DB1_FIXTURE_DIR").map(PathBuf::from) else {
        eprintln!("skipping: KONNECT_DB1_FIXTURE_DIR is not set");
        return;
    };
    let Some(pcb) = routed_board(&dir) else {
        eprintln!(
            "skipping: no subdirectory of {} holds one .kicad_pcb",
            dir.display()
        );
        return;
    };
    let (Some(pro), dru) = (only_file(&dir, "kicad_pro"), only_file(&dir, "kicad_dru")) else {
        eprintln!("skipping: no .kicad_pro beside the fixture");
        return;
    };

    let t0 = Instant::now();
    let (mut board, _) = load_board_file(&pcb, &LoadOptions::default()).expect("board loads");
    let rules = ProjectRuleTable::from_project(
        &std::fs::read_to_string(&pro).unwrap(),
        dru.map(|p| std::fs::read_to_string(p).unwrap()).as_deref(),
        &board,
    )
    .expect("rules compile");
    rules.assign_classes(&mut board);
    let copper = split(&mut board);
    println!("load + rules: {} ms", t0.elapsed().as_millis());
    println!(
        "routed copper: {} tracks, {} vias; board keeps {} tracks, {} vias",
        copper.tracks.len(),
        copper.vias.len(),
        board.tracks.len(),
        board.vias.len()
    );

    let t = Instant::now();
    let checker = ExactChecker::new(&board, &rules, CheckerOptions::default());
    println!("checker index: {} ms", t.elapsed().as_millis());
    let t = Instant::now();
    let before = checker.check_copper(&copper);
    println!(
        "exact check before: {} ms, {:?}",
        t.elapsed().as_millis(),
        by_kind(&before.violations)
    );
    let conn_before = ConnectivityReport::build(&joined(&board, &copper), &ArcPolicy::default())
        .connection_count();

    let opts = OptimizeOptions::default();
    let run = |threads: usize| {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap();
        let mut c = copper.clone();
        let t = Instant::now();
        let report = pool.install(|| optimize_checked(&checker, &mut c, &opts));
        (c, report, t.elapsed())
    };
    let threads = std::thread::available_parallelism().map_or(4, |n| n.get());
    let (many, report, many_time) = run(threads);
    println!(
        "optimize on {threads} threads: {} ms",
        many_time.as_millis()
    );
    println!("{report:#?}");
    let (one, report1, one_time) = run(1);
    println!(
        "optimize on 1 thread: {} ms (speed-up {:.1}x)",
        one_time.as_millis(),
        one_time.as_secs_f64() / many_time.as_secs_f64().max(1e-9)
    );
    assert_eq!(one, many, "1 and {threads} threads give the same copper");
    assert_eq!(report1, report);

    let t = Instant::now();
    let after = checker.check_copper(&many);
    println!(
        "exact check after: {} ms, {:?}",
        t.elapsed().as_millis(),
        by_kind(&after.violations)
    );
    let (b, a) = (by_kind(&before.violations), by_kind(&after.violations));
    for (kind, n) in &a {
        assert!(
            *n <= b.get(kind).copied().unwrap_or(0),
            "the optimizer added {kind:?} violations: {n} after, {:?} before",
            b.get(kind)
        );
    }
    let conn_after =
        ConnectivityReport::build(&joined(&board, &many), &ArcPolicy::default()).connection_count();
    println!("connections still needed: {conn_before} before, {conn_after} after");
    assert!(conn_after <= conn_before, "no connection was lost");
    // The same, judged by anchors as KiCad joins copper (overlap alone can
    // hide a track end that left a pad).
    let split_nets = common::nets_split(
        &common::anchor_islands(&board, &copper),
        &common::anchor_islands(&board, &many),
    );
    println!("nets split by anchors: {}", split_nets.len());
    assert!(split_nets.is_empty(), "anchor joins lost: {split_nets:?}");

    let mut kinds: BTreeMap<String, usize> = BTreeMap::new();
    let defects = geometry_defects(&rules, &opts, &many);
    for (_, _, d) in &defects {
        *kinds.entry(format!("{:?}", d.kind)).or_default() += 1;
    }
    println!("geometry defects after: {kinds:?}");
    // Local diagnosis only (never committed): KONNECT_OPT_DUMP=N prints the
    // first N remaining defects with the copper around them.
    let dump: usize = std::env::var("KONNECT_OPT_DUMP")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0);
    for (net, layer, d) in defects.iter().take(dump) {
        println!(
            "{:?} net {} layer {} at {:?}",
            d.kind,
            net.0,
            layer.0,
            d.at.to_mm()
        );
        let near = |p: konnect_geom::Point| {
            (i64::from(p.x) - i64::from(d.at.x)).abs() < 600_000
                && (i64::from(p.y) - i64::from(d.at.y)).abs() < 600_000
        };
        for t in many
            .tracks
            .iter()
            .filter(|t| t.net == *net && t.layer == *layer)
        {
            if near(t.seg.a) || near(t.seg.b) {
                println!(
                    "   track {:?} -> {:?} w {}",
                    t.seg.a.to_mm(),
                    t.seg.b.to_mm(),
                    t.seg.width
                );
            }
        }
        for v in many.vias.iter().filter(|v| v.net == *net && near(v.center)) {
            println!("   via {:?}", v.center.to_mm());
        }
        for p in board
            .pads
            .iter()
            .filter(|p| p.net == Some(*net) && near(p.center))
        {
            println!("   pad {:?} layers {:?}", p.center.to_mm(), p.layers);
        }
        for t in board
            .tracks
            .iter()
            .filter(|t| t.net == Some(*net) && (near(t.seg.a) || near(t.seg.b)))
        {
            println!(
                "   fixed track {:?} -> {:?}",
                t.seg.a.to_mm(),
                t.seg.b.to_mm()
            );
        }
    }

    // The same copper judged with zone fills ignored: what a refill leaves.
    let unfilled = ExactChecker::new(
        &board,
        &rules,
        CheckerOptions {
            zones_are_obstacles: false,
            ..CheckerOptions::default()
        },
    );
    println!(
        "exact check ignoring zone fills: before {:?}, after {:?}",
        by_kind(&unfilled.check_copper(&copper).violations),
        by_kind(&unfilled.check_copper(&many).violations)
    );
    let zone_free = OptimizeOptions {
        checker: CheckerOptions {
            zones_are_obstacles: false,
            ..CheckerOptions::default()
        },
        ..OptimizeOptions::default()
    };
    let mut c = copper.clone();
    let t = Instant::now();
    let r = optimize_checked(&unfilled, &mut c, &zone_free);
    println!(
        "optimize ignoring zone fills: {} ms, {} pull-tight, {} corners, {} merges, defects {} -> {}, length {} -> {} nm",
        t.elapsed().as_millis(),
        r.pull_tight_edits,
        r.corners_cut,
        r.segments_merged,
        r.defects_before,
        r.defects_after,
        r.length_before_nm,
        r.length_after_nm
    );
    let after_free = by_kind(&unfilled.check_copper(&c).violations);
    let before_free = by_kind(&unfilled.check_copper(&copper).violations);
    for (kind, n) in &after_free {
        assert!(
            *n <= before_free.get(kind).copied().unwrap_or(0),
            "{kind}: {n}"
        );
    }
    let plan = RoutePlan::from_routed(&board, &many).expect("plan builds");
    println!(
        "plan: {} items in {} add_routing calls",
        plan.item_count(),
        plan.chunks(konnect_route::plan::ADD_ROUTING_MAX_ITEMS)
            .len()
    );
}
