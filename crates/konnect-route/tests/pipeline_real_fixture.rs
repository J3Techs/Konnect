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
//! - `KONNECT_PIPELINE_ITERATIONS` (negotiation, default: the pipeline's);
//! - `KONNECT_PIPELINE_REGION=x0,y0,x1,y1` in mm;
//! - `KONNECT_PIPELINE_NETS_FILE=path`: route only the nets named there,
//!   one per line (keep the file outside the repository);
//! - `KONNECT_PIPELINE_NO_FANOUT=1`, `KONNECT_PIPELINE_NO_ACCESS=1`,
//!   `KONNECT_PIPELINE_NO_OPTIMIZE=1`;
//! - `KONNECT_PIPELINE_FANOUT_RETRY=0|1`: the fanout's retry round for
//!   pads the first selection left without a site (default on);
//! - `KONNECT_PIPELINE_FANOUT_FAR_GAPS=mm,mm,mm`: that round's far exit
//!   gaps (default 2.0, 2.5, 3.0; `0`: no far exits);
//! - `KONNECT_PIPELINE_REFERENCE=1`: negotiation's reference search instead
//!   of the kernel, for comparison;
//! - `KONNECT_PIPELINE_CLEAR=0|1`: take out routed copper S7 still finds a
//!   defect in (default on);
//! - `KONNECT_PIPELINE_REPAIR=0|1`: route again what the clear stage
//!   opened, with the connections still failed (default on);
//! - `KONNECT_PIPELINE_RESCUE`, `KONNECT_PIPELINE_RESCUE_ITERATIONS`,
//!   `KONNECT_PIPELINE_RESCUE_NEIGHBOURS`: the rescue rounds after
//!   negotiation, their negotiation iterations, and how many winners a
//!   loser may rip up; `KONNECT_PIPELINE_RESCUE_RETRY=0|1`: later rounds
//!   retry what a round found no way for (default on);
//! - `KONNECT_PIPELINE_CAPACITY_SCALE`: the share of global capacity
//!   corridors are planned on, ×1000;
//! - `KONNECT_PIPELINE_MIN_RUN=0|1`: minimum runs on rule nets (default on);
//! - `KONNECT_PIPELINE_MAX_EXPANSIONS`: the kernel's budget per search;
//! - `KONNECT_PIPELINE_ASTAR`: the kernel's heuristic weight, ×1000;
//! - `KONNECT_PIPELINE_GRID_CORE_MM=w,h`: negotiation's grid tile core size;
//! - `KONNECT_PIPELINE_OUT=dir`: write `summary.json`, `plan.json` and the
//!   copper (`fanout.json`, `routed.json`) there (keep it in an ignored
//!   build location);
//! - `KONNECT_PIPELINE_FILL` (fill passes, default 2),
//!   `KONNECT_PIPELINE_STALL` (stop after this many iterations without
//!   fewer conflicts, default 0: off);
//! - `KONNECT_PIPELINE_PRESENT_GROWTH` (×1000) and
//!   `KONNECT_PIPELINE_HISTORY_INC`: the PathFinder schedule;
//! - `KONNECT_PIPELINE_BGA_SUBTILE_MM`, `KONNECT_PIPELINE_PIN_SEARCH_MM`:
//!   the largest BGA sub-tile core and the pin search reach;
//! - `KONNECT_PIPELINE_NO_STUBS=1`: do not hand S4 access to negotiation;
//! - `KONNECT_PIPELINE_CLASS_LAYERS=Class=F.Cu+B.Cu;…`: class layer masks;
//! - `KONNECT_PIPELINE_CLASS_FALLBACK=Class=F.Cu+B.Cu@Plane.Cu:margin_mm;…`:
//!   second-chance layers, optionally over a reference plane's fill;
//! - `KONNECT_PIPELINE_NECK=ClassA,ClassB`: neck down at pads (0.10 mm
//!   within 1 mm) for every class except these;
//! - `KONNECT_PIPELINE_ALLOWANCES=<JSON or path to a JSON file>`: a
//!   [`RoutingAllowances`] (class layers, layer fallbacks, neck-down and
//!   excluded classes, by name), resolved as the job tools resolve it; it
//!   replaces the three knobs above. Unset, no class is excluded;
//! - `KONNECT_PIPELINE_DIAG=1`: list every failed connection.

use konnect_geom::{BBox, Point};
use konnect_route::api::Stage;
use konnect_route::api::{ClassId, LayerMask};
use konnect_route::detail::negotiate::NeckDown;
use konnect_route::detail::negotiate::ReferenceSearch;
use konnect_route::pipeline::{
    route_board, route_board_with, ClassFallback, PipelineResult, PipelineScope, PipelineSettings,
    ReferencePlane, RoutingAllowances,
};
use konnect_route::rules::RuleTable;
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
    // A file of net names, one per line: route only their connections.
    let nets = std::env::var("KONNECT_PIPELINE_NETS_FILE").ok().map(|f| {
        let text = std::fs::read_to_string(f).expect("nets file reads");
        let wanted: std::collections::BTreeSet<&str> = text
            .lines()
            .map(str::trim)
            .filter(|l| !l.is_empty())
            .collect();
        board
            .nets
            .iter()
            .enumerate()
            .filter(|(_, n)| wanted.contains(n.name.as_str()))
            .map(|(i, _)| konnect_route::api::NetId(i as u32))
            .collect()
    });
    let scope = PipelineScope { nets, region };
    let mut settings = PipelineSettings {
        threads: env_num("KONNECT_PIPELINE_THREADS", 0) as usize,
        ..PipelineSettings::default()
    };
    settings.negotiation.threads = env_num("KONNECT_PIPELINE_NEGOTIATE_THREADS", 0) as usize;
    settings.negotiation.max_iterations = env_num(
        "KONNECT_PIPELINE_ITERATIONS",
        u64::from(settings.negotiation.max_iterations),
    ) as u32;
    settings.negotiation.present_growth = env_num(
        "KONNECT_PIPELINE_PRESENT_GROWTH",
        u64::from(settings.negotiation.present_growth),
    ) as u32;
    settings.negotiation.history_increment = env_num(
        "KONNECT_PIPELINE_HISTORY_INC",
        u64::from(settings.negotiation.history_increment),
    ) as u16;
    settings.search.max_expansions = env_num(
        "KONNECT_PIPELINE_MAX_EXPANSIONS",
        settings.search.max_expansions,
    );
    settings.search.astar_factor = env_num(
        "KONNECT_PIPELINE_ASTAR",
        u64::from(settings.search.astar_factor),
    ) as u32;
    if let Ok(v) = std::env::var("KONNECT_PIPELINE_CLEAR") {
        settings.clear_findings = v == "1";
    }
    if let Ok(v) = std::env::var("KONNECT_PIPELINE_RESCUE_RETRY") {
        settings.rescue_retry_unreachable = v == "1";
    }
    if let Ok(v) = std::env::var("KONNECT_PIPELINE_REPAIR") {
        settings.clear_repair = v == "1";
    }
    if let Ok(v) = std::env::var("KONNECT_PIPELINE_MIN_RUN") {
        settings.search_min_run = v.trim() == "1";
    }
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
    if let (Some(f), Ok(v)) = (
        settings.fanout.as_mut(),
        std::env::var("KONNECT_PIPELINE_FANOUT_RETRY"),
    ) {
        f.retry_failed = v.trim() != "0";
    }
    if let (Some(f), Ok(v)) = (
        settings.fanout.as_mut(),
        std::env::var("KONNECT_PIPELINE_FANOUT_FAR_GAPS"),
    ) {
        // Up to three gaps in mm, comma separated; 0 or empty: none.
        let mut gaps = [0; 3];
        for (g, mm) in gaps
            .iter_mut()
            .zip(v.split(',').filter(|x| !x.trim().is_empty()))
        {
            let mm: f64 = mm.trim().parse().expect("mm");
            *g = (mm * 1e6).round() as i32;
        }
        f.far_gaps = gaps;
    }
    if env_on("KONNECT_PIPELINE_NO_ACCESS") {
        settings.access = None;
    }
    if env_on("KONNECT_PIPELINE_NO_OPTIMIZE") {
        settings.optimize = None;
    }
    settings.tuning.fill_passes = env_num(
        "KONNECT_PIPELINE_FILL",
        u64::from(settings.tuning.fill_passes),
    ) as u32;
    settings.tuning.stall_iterations = env_num("KONNECT_PIPELINE_STALL", 0) as u32;
    settings.global.capacity_scale = env_num(
        "KONNECT_PIPELINE_CAPACITY_SCALE",
        u64::from(settings.global.capacity_scale),
    ) as u32;
    settings.rescue_rounds =
        env_num("KONNECT_PIPELINE_RESCUE", u64::from(settings.rescue_rounds)) as u32;
    settings.rescue_neighbours = env_num(
        "KONNECT_PIPELINE_RESCUE_NEIGHBOURS",
        u64::from(settings.rescue_neighbours),
    ) as u32;
    settings.rescue_iterations = env_num(
        "KONNECT_PIPELINE_RESCUE_ITERATIONS",
        u64::from(settings.rescue_iterations),
    ) as u32;
    if let Ok(v) = std::env::var("KONNECT_PIPELINE_BGA_SUBTILE_MM") {
        let mm: f64 = v.trim().parse().expect("mm");
        settings.tuning.bga_sub_tile_max = (mm * 1e6).round() as i32;
    }
    if let Ok(v) = std::env::var("KONNECT_PIPELINE_PIN_SEARCH_MM") {
        let mm: f64 = v.trim().parse().expect("mm");
        settings.tuning.pin_search = (mm * 1e6).round() as i32;
    }
    if env_on("KONNECT_PIPELINE_NO_STUBS") {
        settings.use_access = false;
    }
    let class_id = |name: &str| {
        let idx = rules
            .classes()
            .iter()
            .position(|c| c.name == name)
            .unwrap_or_else(|| panic!("no net class '{name}'"));
        ClassId(idx as u16)
    };
    let mask = |spec: &str| {
        spec.split('+')
            .map(|l| board.layers.expand(l.trim()))
            .fold(LayerMask::EMPTY, LayerMask::union)
    };
    if let Ok(spec) = std::env::var("KONNECT_PIPELINE_CLASS_LAYERS") {
        for part in spec.split(';').filter(|s| !s.trim().is_empty()) {
            let (class, layers) = part.split_once('=').expect("Class=Layer+Layer");
            settings
                .layer_policy
                .class_layers
                .insert(class_id(class.trim()), mask(layers));
        }
    }
    if let Ok(spec) = std::env::var("KONNECT_PIPELINE_CLASS_FALLBACK") {
        for part in spec.split(';').filter(|s| !s.trim().is_empty()) {
            let (class, rest) = part.split_once('=').expect("Class=Layers[@Plane:mm]");
            let (layers, reference) = match rest.split_once('@') {
                Some((l, r)) => (l, Some(r)),
                None => (rest, None),
            };
            let layers = mask(layers);
            let reference = reference.map(|r| {
                let (plane, margin) = r.split_once(':').unwrap_or((r, "0"));
                let plane = board.layers.by_name(plane.trim()).expect("plane layer");
                let margin: f64 = margin.trim().parse().expect("margin in mm");
                // The fallback's layers beyond the class's own need it.
                let own = settings
                    .layer_policy
                    .class_layers
                    .get(&class_id(class.trim()))
                    .copied()
                    .unwrap_or(LayerMask::EMPTY);
                ReferencePlane {
                    plane,
                    over: LayerMask(layers.0 & !own.0),
                    margin: (margin * 1e6).round() as i32,
                }
            });
            settings.class_fallback.insert(
                class_id(class.trim()),
                ClassFallback {
                    layers,
                    reference,
                    nets: None,
                },
            );
        }
    }
    if let Ok(spec) = std::env::var("KONNECT_PIPELINE_NECK") {
        let excluded = spec
            .split(',')
            .filter(|s| !s.trim().is_empty())
            .map(|s| class_id(s.trim()))
            .collect();
        settings.neck_down = Some(NeckDown {
            excluded,
            ..NeckDown::default()
        });
    }
    if let Ok(v) = std::env::var("KONNECT_PIPELINE_ALLOWANCES") {
        let text = if v.trim_start().starts_with('{') {
            v
        } else {
            std::fs::read_to_string(v.trim()).expect("allowances file reads")
        };
        let a: RoutingAllowances = serde_json::from_str(&text).expect("allowances parse");
        let report = a
            .resolve(&board, rules.classes(), &mut settings)
            .expect("allowances resolve");
        println!("allowances: {}", serde_json::to_string(&report).unwrap());
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
    // Failures per net class and reason (printed only; nothing committed).
    let mut by_class: BTreeMap<(String, &str), usize> = BTreeMap::new();
    for (id, d) in &r.detail.failure_details {
        let c = &r.connections[id.index()];
        let class = rules.class(rules.class_of(Some(c.net))).name.clone();
        *by_class.entry((class, d.name())).or_default() += 1;
    }
    println!("failures by class: {by_class:?}");
    let mut tasks_by_class: BTreeMap<String, usize> = BTreeMap::new();
    for c in &r.connections {
        let class = rules.class(rules.class_of(Some(c.net))).name.clone();
        *tasks_by_class.entry(class).or_default() += 1;
    }
    println!("tasks by class: {tasks_by_class:?}");
    let mut excluded_by_class: BTreeMap<String, usize> = BTreeMap::new();
    for c in &r.excluded {
        let class = rules.class(rules.class_of(Some(c.net))).name.clone();
        *excluded_by_class.entry(class).or_default() += 1;
    }
    println!(
        "excluded by policy: {} {excluded_by_class:?}",
        r.excluded.len()
    );
    let mut lost_by_class: BTreeMap<String, usize> = BTreeMap::new();
    for id in &r.detail.congestion_lost {
        let c = &r.connections[id.index()];
        let class = rules.class(rules.class_of(Some(c.net))).name.clone();
        *lost_by_class.entry(class).or_default() += 1;
    }
    println!(
        "lost to congestion before the fill passes: {} {lost_by_class:?}",
        r.detail.congestion_lost.len()
    );
    if env_on("KONNECT_PIPELINE_DIAG") {
        let item = |i: konnect_route::api::ItemRef| match i {
            konnect_route::api::ItemRef::Pad(p) => {
                let pad = &board.pads[p.index()];
                format!(
                    "{}.{} @({:.3},{:.3})",
                    board.footprints[pad.footprint.index()]
                        .reference
                        .as_deref()
                        .unwrap_or("?"),
                    pad.number,
                    pad.center.x as f64 / 1e6,
                    pad.center.y as f64 / 1e6
                )
            }
            other => format!("{other:?}"),
        };
        for (id, d) in &r.detail.failure_details {
            let c = &r.connections[id.index()];
            println!(
                "  fail {:5} {:24} {:16} {} -> {}",
                id.0,
                board.nets[c.net.index()].name,
                d.name(),
                item(c.from),
                item(c.to)
            );
        }
    }
    for it in &r.detail.reports {
        println!(
            "  it {:2}{} queued {:5} routed {:5} failed {:5} conflicted {:5} pieces {:5}",
            it.iteration,
            if it.fill { " fill" } else { "     " },
            it.queued,
            it.routed,
            it.failed,
            it.conflicted,
            it.pieces
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
    println!(
        "clear: {} spare tracks, {} cut ({} joins opened), {} dangling behind them, {} spurs; {} pieces, {} tracks, {} vias; {} nets; findings {:?}",
        r.cleared.single_tracks,
        r.cleared.cut_tracks,
        r.cleared.opened,
        r.cleared.dangling_tracks,
        r.cleared.spurs,
        r.cleared.pieces,
        r.cleared.tracks,
        r.cleared.vias,
        r.cleared.nets.len(),
        r.cleared.findings
    );
    let by_kind = r.check.counts();
    println!("check: {} findings {:?}", r.check.violations.len(), by_kind);
    if env_on("KONNECT_PIPELINE_DIAG") {
        let all = r.all_copper();
        let nf = r.fanout_copper.tracks.len() as u32;
        let nfv = r.fanout_copper.vias.len() as u32;
        let name = |i: konnect_route::check::CheckedItem| match i {
            konnect_route::check::CheckedItem::RoutedTrack { index } => format!(
                "{} track {} on {}",
                if index < nf { "fanout" } else { "routed" },
                board.nets[all.tracks[index as usize].net.index()].name,
                board.layers.name(all.tracks[index as usize].layer)
            ),
            konnect_route::check::CheckedItem::RoutedVia { index } => format!(
                "{} via {}",
                if index < nfv { "fanout" } else { "routed" },
                board.nets[all.vias[index as usize].net.index()].name
            ),
            other => format!("{other:?}"),
        };
        for v in &r.check.violations {
            println!(
                "  finding {:?} at ({:.3},{:.3}): {} / {}",
                v.kind,
                v.at.x as f64 / 1e6,
                v.at.y as f64 / 1e6,
                name(v.a),
                v.b.map(name).unwrap_or_default()
            );
        }
    }
    // The reference planes hold for the plan: checking it again changes
    // nothing.
    if settings
        .class_fallback
        .values()
        .any(|f| f.reference.is_some())
    {
        // Against the board with the fanout on it, as the run saw it: the
        // fanout's vias cut holes in the plane too.
        let mut work = board.clone();
        konnect_route::pipeline::add_locked(&mut work, &r.fanout_copper);
        let mut copy = r.routed.clone();
        let again =
            konnect_route::pipeline::enforce_reference(&work, &rules, &settings, None, &mut copy);
        println!(
            "reference: {} tracks checked again, {} nets off it",
            again.tracks_checked,
            again.dropped_nets.len()
        );
        assert!(again.dropped_nets.is_empty(), "tracks off the reference");
    }
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
            "excluded": r.excluded.len(),
            "check": by_kind.iter().map(|(k, v)| (format!("{k:?}"), *v)).collect::<BTreeMap<_, _>>(),
            "connections_after": r.connections_after,
            "plan_tracks": r.plan.tracks.len(),
            "plan_vias": r.plan.vias.len(),
            "optimize": r.optimize,
            "cleared": r.cleared,
            "telemetry": r.telemetry,
        });
        std::fs::write(
            out.join("summary.json"),
            serde_json::to_string_pretty(&summary).unwrap(),
        )
        .expect("summary written");
        std::fs::write(out.join("plan.json"), r.plan.to_routing_file_json()).expect("plan written");
        // The copper itself, for `pipeline_tail_fixture` to re-optimize
        // offline (with `KONNECT_PIPELINE_NO_OPTIMIZE=1` it is negotiation's).
        std::fs::write(
            out.join("fanout.json"),
            serde_json::to_string(&r.fanout_copper).unwrap(),
        )
        .expect("fanout copper written");
        std::fs::write(
            out.join("routed.json"),
            serde_json::to_string(&r.routed).unwrap(),
        )
        .expect("routed copper written");
        println!("wrote {}", out.display());
    }
}
