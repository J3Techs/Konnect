//! Runs S5 negotiation (with the reference search backend) on a real board
//! from outside the repository and prints what happened. Nothing about the
//! board is committed: the directory comes from `KONNECT_DB1_FIXTURE_DIR`,
//! files are found by extension, and only board-independent invariants are
//! asserted (every connection is routed or failed with a reason; the plan
//! validates; optionally, 1 and N threads agree).
//!
//! ```text
//! KONNECT_DB1_FIXTURE_DIR=/path/to/fixture cargo test -p konnect-route --release \
//!     --test negotiate_real_fixture -- --nocapture
//! ```
//!
//! Optional knobs, all environment variables:
//!
//! - `KONNECT_NEGOTIATE_THREADS` (default 0 = all cores);
//! - `KONNECT_NEGOTIATE_ITERATIONS` (default 10);
//! - `KONNECT_NEGOTIATE_REGION=x0,y0,x1,y1` in mm: only connections with
//!   both terminals inside;
//! - `KONNECT_NEGOTIATE_LIMIT=n`: only the first `n` connections (by id);
//! - `KONNECT_NEGOTIATE_DETERMINISM=1`: rerun on one thread and compare;
//! - `KONNECT_NEGOTIATE_LOCKSTEP=1`: batch rounds instead of per-tile backend
//!   copies (same result, for comparison);
//! - `KONNECT_NEGOTIATE_ASTAR_WEIGHT` (×1000, default 1200): the reference
//!   search's heuristic weight.
//!
//! Nets that own a zone piece (planes) are left out: S2 fanout joins them.

use konnect_geom::{ArcPolicy, Point};
use konnect_route::api::Stage;
use konnect_route::connectivity::ConnectivityReport;
use konnect_route::detail::negotiate::{
    negotiate_connections, DetailResult, NegotiationSettings, NegotiationTuning, ReferenceSearch,
};
use konnect_route::global::GlobalResult;
use konnect_route::telemetry::Telemetry;
use konnect_route::{load_board_file, LoadOptions, ProjectRuleTable, RoutePlan};
use std::collections::{BTreeMap, BTreeSet};
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

fn env_num(name: &str, default: usize) -> usize {
    std::env::var(name)
        .ok()
        .and_then(|v| v.trim().parse().ok())
        .unwrap_or(default)
}

#[test]
fn negotiate_real_board_fixture() {
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
    let all = ConnectivityReport::build(&board, &ArcPolicy::default()).connections(&board);
    println!(
        "load + rules + connectivity: {} ms",
        t0.elapsed().as_millis()
    );

    let plane_nets: BTreeSet<_> = board.zones.iter().filter_map(|z| z.net).collect();
    let region = std::env::var("KONNECT_NEGOTIATE_REGION").ok().map(|s| {
        let v: Vec<f64> = s
            .split(',')
            .map(|x| x.trim().parse().expect("mm"))
            .collect();
        assert_eq!(v.len(), 4, "x0,y0,x1,y1");
        (
            Point::from_mm(v[0], v[1]).unwrap(),
            Point::from_mm(v[2], v[3]).unwrap(),
        )
    });
    let inside =
        |p: Point| region.is_none_or(|(a, b)| p.x >= a.x && p.x <= b.x && p.y >= a.y && p.y <= b.y);
    let centre = |item| match item {
        konnect_route::api::ItemRef::Pad(id) => Some(board.pads[id.index()].center),
        _ => None,
    };
    let mut connections: Vec<_> = all
        .iter()
        .filter(|c| !plane_nets.contains(&c.net))
        .filter(|c| centre(c.from).is_some_and(inside) && centre(c.to).is_some_and(inside))
        .copied()
        .collect();
    let limit = env_num("KONNECT_NEGOTIATE_LIMIT", usize::MAX);
    connections.truncate(limit);
    println!(
        "connections: {} on the board, {} on plane nets (left to fanout), {} routed here",
        all.len(),
        all.iter().filter(|c| plane_nets.contains(&c.net)).count(),
        connections.len()
    );

    let threads = env_num("KONNECT_NEGOTIATE_THREADS", 0);
    let settings = NegotiationSettings {
        threads,
        max_iterations: env_num("KONNECT_NEGOTIATE_ITERATIONS", 10) as u32,
        ..NegotiationSettings::default()
    };
    let tuning = NegotiationTuning {
        lockstep_rounds: std::env::var("KONNECT_NEGOTIATE_LOCKSTEP").as_deref() == Ok("1"),
        bga_sub_tile_max: env_num("KONNECT_NEGOTIATE_BGA_SUB_UM", 5_000) as i32 * 1_000,
        ..NegotiationTuning::default()
    };
    let run = |settings: &NegotiationSettings| -> (DetailResult, Telemetry, u128) {
        let mut backend = ReferenceSearch::new(settings.threads);
        backend.heuristic_weight = env_num("KONNECT_NEGOTIATE_ASTAR_WEIGHT", 1200) as u32;
        let mut telemetry = Telemetry::default();
        let t = Instant::now();
        let r = negotiate_connections(
            &board,
            &rules,
            &connections,
            &GlobalResult::default(),
            None,
            settings,
            &tuning,
            &mut backend,
            &mut telemetry,
        );
        (r, telemetry, t.elapsed().as_millis())
    };
    let (result, telemetry, ms) = run(&settings);
    println!(
        "negotiate: {ms} ms wall ({} us S5 stage) on {} threads; raster builds {} us; congested iterations {} us",
        telemetry.stage_us(Stage::Detail),
        telemetry.threads,
        telemetry.raster_rebuild_us,
        telemetry.congested_iteration_us
    );
    println!(
        "bga fields: {} (pads per field {:?})",
        result.bga_fields.len(),
        result.bga_fields.iter().map(|f| f.pads).collect::<Vec<_>>()
    );
    for r in &result.reports {
        println!(
            "  it {:2}{} pf {:6} queued {:5} pieces {:5} pins {:5} routed {:5} failed {:5} conflict pairs {:5} conflicted {:5} history cells {:7}",
            r.iteration,
            if r.shifted { "s" } else { " " },
            r.present_factor,
            r.queued,
            r.pieces,
            r.pins,
            r.routed,
            r.failed,
            r.conflict_pairs,
            r.conflicted,
            r.history_cells
        );
    }
    let mut reasons: BTreeMap<&str, usize> = BTreeMap::new();
    for (_, f) in &result.failure_details {
        *reasons.entry(f.name()).or_default() += 1;
    }
    println!(
        "routed {} / {} connections ({:.1}%), failed {} {:?}",
        result.routed.len(),
        connections.len(),
        100.0 * result.routed.len() as f64 / connections.len().max(1) as f64,
        result.failed.len(),
        reasons
    );
    println!(
        "copper: {} tracks, {} vias; expansions mean {:.0} max {} over {} searches",
        result.copper.tracks.len(),
        result.copper.vias.len(),
        telemetry.expansions_per_connection.mean(),
        telemetry.expansions_per_connection.max,
        telemetry.expansions_per_connection.total
    );
    if std::env::var("KONNECT_NEGOTIATE_VERBOSE").as_deref() == Ok("1") {
        // Local diagnosis only; never pasted into the repository.
        let describe = |item: konnect_route::api::ItemRef| match item {
            konnect_route::api::ItemRef::Pad(id) => {
                let p = &board.pads[id.index()];
                let f = &board.footprints[p.footprint.index()];
                format!(
                    "{}.{} at {:?} mm layers {:?} kind {:?}",
                    f.reference.as_deref().unwrap_or("?"),
                    p.number,
                    p.center.to_mm(),
                    board
                        .layers
                        .copper
                        .iter()
                        .filter(|l| p.layers.contains(l.id))
                        .map(|l| l.name.as_str())
                        .collect::<Vec<_>>(),
                    p.kind
                )
            }
            other => format!("{other:?}"),
        };
        for (id, f) in &result.failure_details {
            let c = connections
                .iter()
                .find(|c| c.id == *id)
                .expect("known connection");
            println!(
                "  FAIL {} {:?}: {} -> {}",
                board.net(c.net).name,
                f,
                describe(c.from),
                describe(c.to)
            );
        }
    }
    let h = &telemetry.expansions_per_connection;
    let buckets: Vec<String> = h
        .counts
        .iter()
        .enumerate()
        .filter(|(_, c)| **c > 0)
        .map(|(i, c)| match h.bounds.get(i) {
            Some(b) => format!("<={b}: {c}"),
            None => format!(">: {c}"),
        })
        .collect();
    println!("expansions per search: {}", buckets.join(", "));
    println!("counters: {:?}", telemetry.counters);

    assert_eq!(
        result.routed.len() + result.failed.len(),
        connections.len(),
        "every connection is routed or failed with a reason"
    );
    if !result.copper.tracks.is_empty() || !result.copper.vias.is_empty() {
        let plan = RoutePlan::from_routed(&board, &result.copper).expect("plan validates");
        println!(
            "plan: {} items in {} add_routing chunks",
            plan.item_count(),
            plan.chunks(konnect_route::plan::ADD_ROUTING_MAX_ITEMS)
                .len()
        );
    }
    if std::env::var("KONNECT_NEGOTIATE_DETERMINISM").as_deref() == Ok("1") {
        let one = NegotiationSettings {
            threads: 1,
            ..settings
        };
        let (single, t1, ms1) = run(&one);
        println!(
            "1 thread: {ms1} ms ({} us S5); speedup {:.2}x",
            t1.stage_us(Stage::Detail),
            ms1 as f64 / ms.max(1) as f64
        );
        assert_eq!(single, result, "1-thread and N-thread results differ");
        let a = RoutePlan::from_routed(&board, &single.copper).map(|p| p.to_routing_file_json());
        let b = RoutePlan::from_routed(&board, &result.copper).map(|p| p.to_routing_file_json());
        assert_eq!(a.ok(), b.ok(), "plans differ");
        println!(
            "determinism: 1-thread and {}-thread plans are identical",
            telemetry.threads
        );
    }
}
