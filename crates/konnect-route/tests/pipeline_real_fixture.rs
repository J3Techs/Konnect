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
//! - `KONNECT_PIPELINE_NO_FANOUT=1`, `KONNECT_PIPELINE_NO_ACCESS=1`,
//!   `KONNECT_PIPELINE_NO_OPTIMIZE=1`;
//! - `KONNECT_PIPELINE_REFERENCE=1`: negotiation's reference search instead
//!   of the kernel, for comparison;
//! - `KONNECT_PIPELINE_CLEAR=0|1`: take out routed copper S7 still finds a
//!   defect in (default on);
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
//! - `KONNECT_PIPELINE_DIAG=1`: list every failed connection;
//! - `KONNECT_PIPELINE_VERTICAL=Layer+Layer`: global routing's vertical
//!   layers (default: alternate by routable-layer order);
//! - `KONNECT_PIPELINE_OPEN_LAYER=Layer@REF:margin_mm` (or
//!   `Layer@x0,y0,x1,y1` in mm): capacity experiment, in memory only. The
//!   plane layer becomes routable inside the box (the footprint's courtyard
//!   grown by the margin), its fill there is taken out, the rest of its
//!   fill stays, and tracks on it stay inside the box (vias still cross it
//!   anywhere). Prints the fill area removed and whether any remaining
//!   piece of that fill is left without a through item of its net;
//! - `KONNECT_PIPELINE_VIA_IN_PAD=REF[:Class+Class][:planes]`: capacity
//!   experiment, in memory only. A through via (the rule floor size where
//!   it sits) at the centre of each netted pad of footprint `REF` whose net
//!   is in one of the classes (every netted pad when no class is given) or,
//!   with `planes`, is a plane net. Fanout then skips those pads.

use konnect_geom::{BBox, Coord, Point, Polygon};
use konnect_route::api::Stage;
use konnect_route::api::{ClassId, LayerMask};
use konnect_route::api::{FootprintId, ItemKind, LayerId, NetId, Ownership, ViaId, ZoneId};
use konnect_route::board::{Via, ZonePiece};
use konnect_route::connectivity::ConnectivityReport;
use konnect_route::detail::negotiate::NeckDown;
use konnect_route::detail::negotiate::ReferenceSearch;
use konnect_route::global::TrackRegion;
use konnect_route::pipeline::add_locked;
use konnect_route::pipeline::{
    route_board, route_board_with, ClassFallback, PipelineResult, PipelineScope, PipelineSettings,
    ReferencePlane,
};
use konnect_route::rules::RuleSubject;
use konnect_route::rules::RuleTable;
use konnect_route::{load_board_file, LoadOptions, ProjectRuleTable};
use konnect_route::{RouteBoard, RoutedCopper};
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

    // Capacity experiments: in-memory changes to the loaded board only.
    let vip = std::env::var("KONNECT_PIPELINE_VIA_IN_PAD")
        .ok()
        .map(|spec| vias_in_pads(&mut board, &rules, &spec));
    let open = std::env::var("KONNECT_PIPELINE_OPEN_LAYER")
        .ok()
        .map(|spec| open_layer(&mut board, &spec));

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
    if let Some(o) = &open {
        settings.global.track_region = Some(o.region);
        settings.search.track_region = Some(o.region);
    }
    if let Ok(spec) = std::env::var("KONNECT_PIPELINE_VERTICAL") {
        let m = spec
            .split('+')
            .map(|l| board.layers.expand(l.trim()))
            .fold(LayerMask::EMPTY, LayerMask::union);
        settings.global.vertical_layers = Some(m);
    }
    if let Ok(v) = std::env::var("KONNECT_PIPELINE_CLEAR") {
        settings.clear_findings = v == "1";
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
            settings
                .class_fallback
                .insert(class_id(class.trim()), ClassFallback { layers, reference });
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
    println!(
        "plan: {} tracks, {} vias; connections open {} -> {}",
        r.plan.tracks.len(),
        r.plan.vias.len(),
        r.connections_before_fanout,
        r.connections_after
    );
    // What is left open, by net class, with the plan applied.
    let mut applied = board.clone();
    add_locked(&mut applied, &r.fanout_copper);
    add_locked(&mut applied, &r.routed);
    let open_after = ConnectivityReport::build(&applied, &Default::default()).connections(&applied);
    let mut open_by_class: BTreeMap<String, usize> = BTreeMap::new();
    for c in &open_after {
        let class = rules.class(rules.class_of(Some(c.net))).name.clone();
        *open_by_class.entry(class).or_default() += 1;
    }
    println!("open after, by class: {open_by_class:?}");
    let new_vias = r.fanout_copper.vias.len() + r.routed.vias.len();
    let added = vip.as_ref().map_or(0, |v| v.vias);
    println!(
        "vias: fanout {}, routed {}, via-in-pad {added}; total new {}",
        r.fanout_copper.vias.len(),
        r.routed.vias.len(),
        new_vias + added
    );
    if let Some(v) = &vip {
        println!(
            "via-in-pad: {} vias, by class {:?}; dog-bones taken out: {} necks, {} vias",
            v.vias, v.by_class, v.necks_removed, v.dogbone_vias_removed
        );
    }
    let fp_sites: BTreeMap<String, usize> = r.fanout.as_ref().map_or_else(BTreeMap::new, |f| {
        let mut m = BTreeMap::new();
        for s in &f.sites {
            let fp = board.pads[s.pad.index()].footprint;
            let name = board.footprints[fp.index()]
                .reference
                .clone()
                .unwrap_or_default();
            *m.entry(name).or_default() += 1;
        }
        m
    });
    if let Ok(reference) = std::env::var("KONNECT_PIPELINE_REPORT_FP") {
        // The footprint's plane-net pads: what the fanout did with each.
        let mut status: BTreeMap<String, usize> = BTreeMap::new();
        if let (Some(fp), Some(f)) = (board.footprint_by_reference(&reference), &r.fanout) {
            let planes = plane_nets(&board);
            let (p0, p1) = board.footprints[fp.index()].pad_range;
            for i in p0..p1 {
                let pad = &board.pads[i as usize];
                if !pad.net.is_some_and(|n| planes.contains(&n)) {
                    continue;
                }
                let id = pad.id;
                let s = if let Some(site) = f.sites.iter().find(|s| s.pad == id) {
                    if site.stub.is_some() {
                        "site with stub".to_string()
                    } else {
                        "site without stub".to_string()
                    }
                } else if let Some((_, why)) = f.failed.iter().find(|(p, _)| *p == id) {
                    format!("failed {why:?}")
                } else if let Some((_, why)) = f.skipped.iter().find(|(p, _)| *p == id) {
                    match why {
                        konnect_route::fanout::FanoutSkip::JoinedThroughIsland { .. } => {
                            "skipped: joined through island".to_string()
                        }
                        other => format!("skipped {other:?}"),
                    }
                } else {
                    "not a fanout unit (joined already)".to_string()
                };
                *status.entry(s).or_default() += 1;
            }
        }
        println!(
            "fanout sites on {reference}: {}; its plane-net pads: {status:?}",
            fp_sites.get(&reference).copied().unwrap_or(0)
        );
    }
    let open_report = open.as_ref().map(|o| {
        let outside: Vec<_> = r
            .routed
            .tracks
            .iter()
            .chain(r.fanout_copper.tracks.iter())
            .filter(|t| {
                t.layer == o.region.layer
                    && !(o.region.bbox.contains(t.seg.a) && o.region.bbox.contains(t.seg.b))
            })
            .collect();
        let on_layer = r
            .routed
            .tracks
            .iter()
            .filter(|t| t.layer == o.region.layer)
            .count();
        let net_items = through_items(&board, &r.fanout_copper, o.net);
        let islands = plane_islands(&o.original, &o.region, &net_items);
        println!(
            "open layer {}: box {:.2},{:.2} .. {:.2},{:.2} mm; fill removed {:.1} mm2 of {:.1} mm2; {} tracks on it, {} outside the box",
            board.layers.name(o.region.layer),
            o.region.bbox.min.x as f64 / 1e6,
            o.region.bbox.min.y as f64 / 1e6,
            o.region.bbox.max.x as f64 / 1e6,
            o.region.bbox.max.y as f64 / 1e6,
            o.removed_mm2,
            o.total_mm2,
            on_layer,
            outside.len()
        );
        println!("open layer islands: {islands:?}");
        json!({
            "layer": board.layers.name(o.region.layer),
            "box_mm": [o.region.bbox.min.x as f64 / 1e6, o.region.bbox.min.y as f64 / 1e6,
                       o.region.bbox.max.x as f64 / 1e6, o.region.bbox.max.y as f64 / 1e6],
            "fill_removed_mm2": o.removed_mm2,
            "fill_total_mm2": o.total_mm2,
            "tracks_on_layer": on_layer,
            "tracks_outside_box": outside.len(),
            "islands": islands,
        })
    });
    let report_class = std::env::var("KONNECT_PIPELINE_REPORT_CLASS").unwrap_or_default();
    println!(
        "EXPERIMENT routed {} of {} | open {} | {report_class} open {} | new vias {} | vias on the board {} | wall {:.1} s",
        r.detail.routed.len(),
        r.connections.len(),
        r.connections_after,
        open_by_class.get(&report_class).copied().unwrap_or(0),
        new_vias + added,
        board.vias.len() + new_vias,
        wall_ms as f64 / 1000.0
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
            "cleared": r.cleared,
            "open_by_class": open_by_class,
            "new_vias": new_vias + added,
            "board_vias_total": board.vias.len() + new_vias,
            "via_in_pad": vip.as_ref().map(|v| json!({"vias": v.vias, "by_class": v.by_class, "necks_removed": v.necks_removed, "dogbone_vias_removed": v.dogbone_vias_removed})),
            "open_layer": open_report,
            "fanout_sites_by_footprint": fp_sites,
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

// ─── Capacity experiments (in memory only) ──────────────────────────────────

/// What [`vias_in_pads`] did.
struct ViaInPad {
    /// Vias added at pad centres.
    vias: usize,
    by_class: BTreeMap<String, usize>,
    /// Existing dog-bones taken out: their necks and their vias.
    necks_removed: usize,
    dogbone_vias_removed: usize,
}

/// Nets with a filled zone piece.
fn plane_nets(board: &RouteBoard) -> std::collections::BTreeSet<NetId> {
    board
        .zones
        .iter()
        .filter(|z| z.filled)
        .filter_map(|z| z.net)
        .collect()
}

/// The longest neck taken for a dog-bone: a track from the pad to a via of
/// its net no longer than this.
const NECK_MAX_NM: f64 = 1_000_000.0;

/// `REF[:Class+Class][:planes]`: every selected netted pad of footprint
/// `REF` loses its dog-bone (a short track from the pad to a via of its
/// net, and that via when nothing else lands on it) and gets a through via
/// of the rule floor size at its centre instead.
fn vias_in_pads(board: &mut RouteBoard, rules: &ProjectRuleTable, spec: &str) -> ViaInPad {
    let mut parts = spec.split(':');
    let reference = parts.next().expect("footprint reference").trim();
    let mut classes: Option<Vec<String>> = None;
    let mut planes = false;
    for p in parts {
        let p = p.trim();
        if p == "planes" {
            planes = true;
        } else if !p.is_empty() {
            classes = Some(p.split('+').map(|c| c.trim().to_string()).collect());
        }
    }
    let fp: FootprintId = board
        .footprint_by_reference(reference)
        .unwrap_or_else(|| panic!("no footprint '{reference}'"));
    let plane = plane_nets(board);
    let all = board.layers.all();
    let (p0, p1) = board.footprints[fp.index()].pad_range;
    let mut out = ViaInPad {
        vias: 0,
        by_class: BTreeMap::new(),
        necks_removed: 0,
        dogbone_vias_removed: 0,
    };
    let picked: Vec<usize> = (p0 as usize..p1 as usize)
        .filter(|&i| {
            let Some(net) = board.pads[i].net else {
                return false;
            };
            let class = &rules.class(rules.class_of(Some(net))).name;
            match (&classes, planes) {
                (None, false) => true,
                (None, true) => plane.contains(&net),
                (Some(c), pl) => c.contains(class) || (pl && plane.contains(&net)),
            }
        })
        .collect();

    // The dog-bones: necks from a picked pad to a via of its net.
    let near = |a: Point, b: Point, d: f64| f64::from(a.x - b.x).hypot(f64::from(a.y - b.y)) <= d;
    let mut drop_track = vec![false; board.tracks.len()];
    let mut via_ends: Vec<(NetId, Point)> = Vec::new();
    for &i in &picked {
        let pad = &board.pads[i];
        let reach = pad.bbox().map_or(0.0, |b| {
            f64::from((b.max.x - b.min.x).max(b.max.y - b.min.y)) / 2.0
        }) + 1_000.0;
        for (k, t) in board.tracks.iter().enumerate() {
            if t.net != pad.net || !pad.layers.contains(t.layer) {
                continue;
            }
            let len = f64::from(t.seg.b.x - t.seg.a.x).hypot(f64::from(t.seg.b.y - t.seg.a.y));
            if len > NECK_MAX_NM {
                continue;
            }
            let far = if near(t.seg.a, pad.center, reach) {
                t.seg.b
            } else if near(t.seg.b, pad.center, reach) {
                t.seg.a
            } else {
                continue;
            };
            let lands = board
                .vias
                .iter()
                .any(|v| v.net == t.net && near(v.center, far, 1_000.0));
            if lands {
                drop_track[k] = true;
                via_ends.push((t.net.expect("netted"), far));
            }
        }
    }
    let mut drop_via = vec![false; board.vias.len()];
    for (k, v) in board.vias.iter().enumerate() {
        let Some(net) = v.net else { continue };
        if !via_ends
            .iter()
            .any(|&(n, at)| n == net && near(v.center, at, 1_000.0))
        {
            continue;
        }
        // Keep a via other copper still lands on.
        let r = f64::from(v.diameter) / 2.0;
        let used = board.tracks.iter().enumerate().any(|(j, t)| {
            !drop_track[j]
                && t.net == v.net
                && (near(t.seg.a, v.center, r) || near(t.seg.b, v.center, r))
        });
        drop_via[k] = !used;
    }
    out.necks_removed = drop_track.iter().filter(|&&d| d).count();
    out.dogbone_vias_removed = drop_via.iter().filter(|&&d| d).count();
    let mut k = 0;
    board.tracks.retain(|_| {
        k += 1;
        !drop_track[k - 1]
    });
    let mut k = 0;
    board.vias.retain(|_| {
        k += 1;
        !drop_via[k - 1]
    });
    for (i, t) in board.tracks.iter_mut().enumerate() {
        t.id = konnect_route::api::TrackId(i as u32);
    }
    for (i, v) in board.vias.iter_mut().enumerate() {
        v.id = ViaId(i as u32);
    }

    let courts = [fp];
    for &i in &picked {
        let net = board.pads[i].net.expect("picked pads are netted");
        let class = rules.class(rules.class_of(Some(net))).name.clone();
        let rule = rules.via(&RuleSubject::new(ItemKind::Via, Some(net)).with_courtyards(&courts));
        let (diameter, drill) = if rule.min_diameter > 0 && rule.min_drill > 0 {
            (rule.min_diameter, rule.min_drill)
        } else {
            (rule.diameter, rule.drill)
        };
        let center = board.pads[i].center;
        let id = ViaId(board.vias.len() as u32);
        board.vias.push(Via {
            id,
            net: Some(net),
            center,
            diameter,
            drill,
            layers: all,
            ownership: Ownership::Locked,
            uuid: None,
        });
        out.vias += 1;
        *out.by_class.entry(class).or_default() += 1;
    }
    out
}

/// What [`open_layer`] did.
struct OpenLayer {
    region: TrackRegion,
    /// The (first) net of the fill taken out.
    net: Option<NetId>,
    /// The layer's filled pieces as loaded.
    original: Vec<Polygon>,
    removed_mm2: f64,
    total_mm2: f64,
}

#[derive(Clone, Copy)]
enum Keep {
    XLe(Coord),
    XGe(Coord),
    YLe(Coord),
    YGe(Coord),
}

/// Sutherland–Hodgman: `ring` cut to one closed half-plane. A concave
/// ring may come back with zero-width bridges along the cut line; they
/// enclose no area and change no containment test off the line.
fn clip(ring: &[Point], keep: Keep) -> Vec<Point> {
    let inside = |p: Point| match keep {
        Keep::XLe(v) => p.x <= v,
        Keep::XGe(v) => p.x >= v,
        Keep::YLe(v) => p.y <= v,
        Keep::YGe(v) => p.y >= v,
    };
    let lerp = |a0: Coord, a1: Coord, b0: Coord, b1: Coord, v: Coord| {
        let t = f64::from(v - a0) / f64::from(a1 - a0);
        (f64::from(b0) + t * f64::from(b1 - b0)).round() as Coord
    };
    let cross = |a: Point, b: Point| -> Point {
        match keep {
            Keep::XLe(v) | Keep::XGe(v) => Point::new(v, lerp(a.x, b.x, a.y, b.y, v)),
            Keep::YLe(v) | Keep::YGe(v) => Point::new(lerp(a.y, b.y, a.x, b.x, v), v),
        }
    };
    let n = ring.len();
    let mut out = Vec::with_capacity(n + 4);
    for i in 0..n {
        let (prev, cur) = (ring[(i + n - 1) % n], ring[i]);
        match (inside(prev), inside(cur)) {
            (true, true) => out.push(cur),
            (true, false) => out.push(cross(prev, cur)),
            (false, true) => {
                out.push(cross(prev, cur));
                out.push(cur);
            }
            (false, false) => {}
        }
    }
    out
}

fn clip_all(ring: &[Point], keeps: &[Keep]) -> Vec<Point> {
    keeps.iter().fold(
        ring.to_vec(),
        |r, &k| {
            if r.is_empty() {
                r
            } else {
                clip(&r, k)
            }
        },
    )
}

fn ring_area2(ring: &[Point]) -> i128 {
    let n = ring.len();
    (0..n)
        .map(|i| {
            let (a, b) = (ring[i], ring[(i + 1) % n]);
            i128::from(a.x) * i128::from(b.y) - i128::from(b.x) * i128::from(a.y)
        })
        .sum()
}

/// Enclosed area of `poly` inside the box, mm².
fn area_inside(poly: &Polygon, b: &BBox) -> f64 {
    let keeps = [
        Keep::XGe(b.min.x),
        Keep::XLe(b.max.x),
        Keep::YGe(b.min.y),
        Keep::YLe(b.max.y),
    ];
    let mut a2: i128 = ring_area2(&clip_all(poly.outline(), &keeps)).abs();
    for h in poly.holes() {
        a2 -= ring_area2(&clip_all(h, &keeps)).abs();
    }
    a2 as f64 / 2.0 / 1e12
}

fn area_mm2(poly: &Polygon) -> f64 {
    poly.area2().unsigned_abs() as f64 / 2.0 / 1e12
}

/// `Layer@REF:margin_mm` or `Layer@x0,y0,x1,y1`.
fn open_layer(board: &mut RouteBoard, spec: &str) -> OpenLayer {
    let (layer, at) = spec
        .split_once('@')
        .expect("Layer@REF:margin or Layer@x0,y0,x1,y1");
    let layer: LayerId = board.layers.by_name(layer.trim()).expect("copper layer");
    let bbox = if let Some((reference, margin)) = at.split_once(':') {
        let fp = board
            .footprint_by_reference(reference.trim())
            .unwrap_or_else(|| panic!("no footprint '{reference}'"));
        let margin: f64 = margin.trim().parse().expect("margin in mm");
        board.footprints[fp.index()]
            .courtyard_bbox()
            .expect("the footprint has a courtyard")
            .inflate((margin * 1e6).round() as Coord)
    } else {
        let v: Vec<f64> = at
            .split(',')
            .map(|x| x.trim().parse().expect("mm"))
            .collect();
        assert_eq!(v.len(), 4, "x0,y0,x1,y1");
        BBox::new(
            Point::from_mm(v[0], v[1]).unwrap(),
            Point::from_mm(v[2], v[3]).unwrap(),
        )
    };
    board.routable = board.routable.with(layer);
    let keeps = [
        Keep::XLe(bbox.min.x),
        Keep::XGe(bbox.max.x),
        Keep::YLe(bbox.min.y),
        Keep::YGe(bbox.max.y),
    ];
    let mut zones: Vec<ZonePiece> = Vec::with_capacity(board.zones.len() + 8);
    let (mut removed, mut total) = (0.0, 0.0);
    let mut original = Vec::new();
    let mut net = None;
    for z in std::mem::take(&mut board.zones) {
        if z.layer != layer || !z.filled {
            zones.push(z);
            continue;
        }
        net = net.or(z.net);
        total += area_mm2(&z.region);
        removed += area_inside(&z.region, &bbox);
        // The rest of the piece as four overlapping half-plane pieces
        // (left of, right of, above and below the box); same-net overlaps
        // join, so connectivity sees one piece wherever the original was
        // one outside the box.
        for &k in &keeps {
            let outline = clip(z.region.outline(), k);
            let holes = z.region.holes().iter().map(|h| clip(h, k)).collect();
            if let Some(region) = Polygon::new(outline, holes) {
                zones.push(ZonePiece {
                    region,
                    ..z.clone()
                });
            }
        }
        original.push(z.region);
    }
    for (i, z) in zones.iter_mut().enumerate() {
        z.id = ZoneId(i as u32);
    }
    board.zones = zones;
    OpenLayer {
        region: TrackRegion { layer, bbox },
        net,
        original,
        removed_mm2: removed,
        total_mm2: total,
    }
}

/// Centres of the net's through items: board vias (via-in-pad ones
/// included), plated through pads and the fanout's vias.
fn through_items(board: &RouteBoard, fanout: &RoutedCopper, net: Option<NetId>) -> Vec<Point> {
    let Some(net) = net else {
        return Vec::new();
    };
    let mut out: Vec<Point> = board
        .vias
        .iter()
        .filter(|v| v.net == Some(net))
        .map(|v| v.center)
        .collect();
    out.extend(
        board
            .pads
            .iter()
            .filter(|p| p.net == Some(net) && p.drill.as_ref().is_some_and(|d| d.plated))
            .map(|p| p.center),
    );
    out.extend(
        fanout
            .vias
            .iter()
            .filter(|v| v.net == net)
            .map(|v| v.center),
    );
    out
}

/// 4-connected components of `cells` on a `w`-wide raster: a label per
/// cell (`u32::MAX` outside) and how many components there are.
fn components(cells: &[bool], w: usize) -> (Vec<u32>, usize) {
    let h = cells.len() / w;
    let mut lab = vec![u32::MAX; cells.len()];
    let mut n = 0usize;
    let mut stack = Vec::new();
    for start in 0..cells.len() {
        if !cells[start] || lab[start] != u32::MAX {
            continue;
        }
        lab[start] = n as u32;
        stack.push(start);
        while let Some(i) = stack.pop() {
            let (r, c) = (i / w, i % w);
            let mut next = [None; 4];
            if c > 0 {
                next[0] = Some(i - 1);
            }
            if c + 1 < w {
                next[1] = Some(i + 1);
            }
            if r > 0 {
                next[2] = Some(i - w);
            }
            if r + 1 < h {
                next[3] = Some(i + w);
            }
            for j in next.into_iter().flatten() {
                if cells[j] && lab[j] == u32::MAX {
                    lab[j] = n as u32;
                    stack.push(j);
                }
            }
        }
        n += 1;
    }
    (lab, n)
}

/// Connected pieces of the layer's original fill, before and after the
/// box is taken out, on a 0.1 mm raster (4-connected): how many there
/// are, how many of those left hold none of the net's through items (cut
/// off from the net), and the area they hold.
fn plane_islands(original: &[Polygon], region: &TrackRegion, items: &[Point]) -> serde_json::Value {
    const PX: i64 = 100_000;
    let Some(bb) = original.iter().map(Polygon::bbox).reduce(BBox::union) else {
        return json!(null);
    };
    let (x0, y0) = (i64::from(bb.min.x), i64::from(bb.min.y));
    let w = ((i64::from(bb.max.x) - x0) / PX + 1) as usize;
    let h = ((i64::from(bb.max.y) - y0) / PX + 1) as usize;
    let mut fill = vec![false; w * h];
    let mut xs: Vec<f64> = Vec::new();
    for poly in original {
        for row in 0..h {
            let y = (y0 + row as i64 * PX + PX / 2) as f64;
            xs.clear();
            for ring in poly.rings() {
                let n = ring.len();
                for i in 0..n {
                    let (a, b) = (ring[i], ring[(i + 1) % n]);
                    let (ay, by) = (f64::from(a.y), f64::from(b.y));
                    if (ay <= y) != (by <= y) {
                        let t = (y - ay) / (by - ay);
                        xs.push(f64::from(a.x) + t * (f64::from(b.x) - f64::from(a.x)));
                    }
                }
            }
            xs.sort_by(f64::total_cmp);
            for pair in xs.chunks_exact(2) {
                let c0 = ((pair[0] - x0 as f64) / PX as f64 - 0.5).ceil().max(0.0) as usize;
                let c1 = ((pair[1] - x0 as f64) / PX as f64 - 0.5).floor();
                if c1 < 0.0 {
                    continue;
                }
                for c in c0..=(c1 as usize).min(w - 1) {
                    fill[row * w + c] = true;
                }
            }
        }
    }
    let (_, before) = components(&fill, w);
    let mut after = fill;
    let rb = region.bbox;
    for row in 0..h {
        let y = y0 + row as i64 * PX + PX / 2;
        if y < i64::from(rb.min.y) || y > i64::from(rb.max.y) {
            continue;
        }
        for c in 0..w {
            let x = x0 + c as i64 * PX + PX / 2;
            if x >= i64::from(rb.min.x) && x <= i64::from(rb.max.x) {
                after[row * w + c] = false;
            }
        }
    }
    let (lab, n_after) = components(&after, w);
    let mut area = vec![0usize; n_after];
    for &l in &lab {
        if l != u32::MAX {
            area[l as usize] += 1;
        }
    }
    let mut stitched = vec![false; n_after];
    for p in items {
        let c = (i64::from(p.x) - x0) / PX;
        let r = (i64::from(p.y) - y0) / PX;
        if c < 0 || r < 0 || c as usize >= w || r as usize >= h {
            continue;
        }
        let l = lab[r as usize * w + c as usize];
        if l != u32::MAX {
            stitched[l as usize] = true;
        }
    }
    let px_mm2 = (PX as f64 / 1e6).powi(2);
    let cut: Vec<f64> = (0..n_after)
        .filter(|&i| !stitched[i])
        .map(|i| area[i] as f64 * px_mm2)
        .collect();
    json!({
        "raster_mm": PX as f64 / 1e6,
        "pieces_before": before,
        "pieces_after": n_after,
        "pieces_after_without_through_item": cut.len(),
        "area_without_through_item_mm2": cut.iter().sum::<f64>(),
        "largest_piece_after_mm2": area.iter().max().map_or(0.0, |&a| a as f64 * px_mm2),
    })
}
