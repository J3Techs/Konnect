//! M1-offline: the whole native pipeline on one region of a real board,
//! with every planned plane via applied first, compared against an older
//! routed candidate of the same board on the same nets.
//!
//! Nothing about the board is committed. `KONNECT_DB1_FIXTURE_DIR` names the
//! project directory (one `.kicad_pcb`, `.kicad_pro` and `.kicad_dru`, found
//! by extension); the routed candidate is the only `.kicad_pcb` in the first
//! subdirectory that holds one. The region comes from the environment too:
//!
//! ```text
//! KONNECT_DB1_FIXTURE_DIR=/path/to/fixture KONNECT_REGION_MM=x0,y0,x1,y1 \
//!     cargo test -p konnect-route --release --test region_prototype -- --nocapture
//! ```
//!
//! What it does:
//!
//! 1. plans S2 plane vias over the whole board, checks them with the S7
//!    checker and adds them to a working board as locked copper;
//! 2. takes the region's nets: nets of two or more pads whose every pad
//!    centre lies inside the region;
//! 3. runs [`route_board`] on the working board with the scope set to those
//!    nets (fanout off: it is already applied), once per thread count in
//!    `KONNECT_REGION_THREADS` (comma-separated, default `1,0`; 0 is every
//!    core), and compares the plans for determinism;
//! 4. counts the complete region nets after the run and in the candidate.
//!
//! Optional: `KONNECT_REGION_ITERATIONS` (negotiation, default 10) and
//! `KONNECT_REGION_OUT=dir` (writes `summary.json` there; keep it in an
//! ignored build location). Without the variables the test prints why and
//! passes. Only board-independent invariants are asserted.

use konnect_geom::{ArcPolicy, BBox, Point};
use konnect_route::api::{NetId, Ownership, Stage, TrackId, ViaId};
use konnect_route::board::{Track, Via};
use konnect_route::check::{BoardChecker, CheckOptions};
use konnect_route::connectivity::ConnectivityReport;
use konnect_route::fanout::{plan_plane_vias, FanoutSettings};
use konnect_route::pipeline::{route_board, PipelineResult, PipelineScope, PipelineSettings};
use konnect_route::{load_board_file, LoadOptions, ProjectRuleTable, RouteBoard, RoutedCopper};
use serde_json::json;
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

fn candidate(dir: &Path) -> Option<PathBuf> {
    let mut subs: Vec<PathBuf> = std::fs::read_dir(dir)
        .ok()?
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.is_dir())
        .collect();
    subs.sort();
    subs.iter().find_map(|d| only_file(d, "kicad_pcb"))
}

fn with_locked(board: &RouteBoard, copper: &RoutedCopper, locked: bool) -> RouteBoard {
    let mut b = board.clone();
    for t in &copper.tracks {
        b.tracks.push(Track {
            id: TrackId(b.tracks.len() as u32),
            net: Some(t.net),
            layer: t.layer,
            seg: t.seg,
            ownership: if locked {
                Ownership::Locked
            } else {
                Ownership::Routed {
                    net: t.net,
                    gen: t.gen,
                }
            },
            uuid: None,
        });
    }
    for v in &copper.vias {
        b.vias.push(Via {
            id: ViaId(b.vias.len() as u32),
            net: Some(v.net),
            center: v.center,
            diameter: v.diameter,
            drill: v.drill,
            layers: v.layers,
            ownership: if locked {
                Ownership::Locked
            } else {
                Ownership::Routed {
                    net: v.net,
                    gen: v.gen,
                }
            },
            uuid: None,
        });
    }
    b
}

/// Per net of `nets`, the number of pad islands (1: complete).
fn pad_islands(board: &RouteBoard, nets: &BTreeSet<NetId>) -> BTreeMap<NetId, usize> {
    let conn = ConnectivityReport::build(board, &ArcPolicy::default());
    conn.nets
        .iter()
        .filter(|n| nets.contains(&n.net))
        .map(|n| (n.net, n.pad_island_count()))
        .collect()
}

fn region_nets(board: &RouteBoard, region: BBox) -> BTreeSet<NetId> {
    let mut pads: BTreeMap<NetId, (usize, bool)> = BTreeMap::new();
    for p in &board.pads {
        let Some(net) = p.net else { continue };
        let e = pads.entry(net).or_insert((0, true));
        e.0 += 1;
        e.1 &= region.contains(p.center);
    }
    pads.into_iter()
        .filter(|&(_, (n, inside))| n >= 2 && inside)
        .map(|(net, _)| net)
        .collect()
}

#[test]
fn region_prototype_on_the_real_board() {
    let Some(dir) = std::env::var_os("KONNECT_DB1_FIXTURE_DIR").map(PathBuf::from) else {
        eprintln!("skipping: KONNECT_DB1_FIXTURE_DIR is not set");
        return;
    };
    let Some(region) = std::env::var("KONNECT_REGION_MM").ok() else {
        eprintln!("skipping: KONNECT_REGION_MM is not set");
        return;
    };
    let (Some(pcb), Some(pro)) = (only_file(&dir, "kicad_pcb"), only_file(&dir, "kicad_pro"))
    else {
        eprintln!(
            "skipping: {} lacks a .kicad_pcb and .kicad_pro",
            dir.display()
        );
        return;
    };
    let v: Vec<f64> = region
        .split(',')
        .map(|x| x.trim().parse().expect("mm"))
        .collect();
    assert_eq!(v.len(), 4, "x0,y0,x1,y1");
    let region = BBox::new(
        Point::from_mm(v[0], v[1]).unwrap(),
        Point::from_mm(v[2], v[3]).unwrap(),
    );

    let t0 = Instant::now();
    let (mut board, _) = load_board_file(&pcb, &LoadOptions::default()).expect("board loads");
    let pro_text = std::fs::read_to_string(&pro).expect("project reads");
    let dru_text =
        only_file(&dir, "kicad_dru").map(|p| std::fs::read_to_string(p).expect("rules read"));
    let rules = ProjectRuleTable::from_project(&pro_text, dru_text.as_deref(), &board)
        .expect("rules compile");
    rules.assign_classes(&mut board);
    println!("load + rules: {} ms", t0.elapsed().as_millis());

    // 1. Plane vias over the whole board, checked, then applied.
    let t = Instant::now();
    let conn = ConnectivityReport::build(&board, &ArcPolicy::default());
    let conn_ms = t.elapsed().as_secs_f64() * 1e3;
    let t = Instant::now();
    let plan = plan_plane_vias(&board, &rules, &conn, &FanoutSettings::default());
    let fanout_ms = t.elapsed().as_secs_f64() * 1e3;
    let mut fanout = plan.routed();
    let all = board.layers.all();
    for v in &mut fanout.vias {
        v.layers = all;
    }
    let t = Instant::now();
    let checker = BoardChecker::new(&board, &rules, CheckOptions::for_routed());
    let fanout_check = checker.check_routed(&fanout);
    let fanout_check_ms = t.elapsed().as_secs_f64() * 1e3;
    let mut sites_per_net: BTreeMap<String, (usize, usize)> = BTreeMap::new();
    for s in &plan.sites {
        sites_per_net
            .entry(board.net(s.net).name.clone())
            .or_default()
            .0 += 1;
    }
    for (p, _) in &plan.failed {
        if let Some(n) = board.pads[p.index()].net {
            sites_per_net
                .entry(board.net(n).name.clone())
                .or_default()
                .1 += 1;
        }
    }
    println!(
        "G-Fanout: {} sites, {} pads failed, {} skipped; {} tracks, {} vias; plan {fanout_ms:.1} ms \
         (connectivity {conn_ms:.1} ms), S7 check {} findings {:?} in {fanout_check_ms:.1} ms",
        plan.sites.len(),
        plan.failed.len(),
        plan.skipped.len(),
        fanout.tracks.len(),
        fanout.vias.len(),
        fanout_check.violations.len(),
        fanout_check.counts()
    );
    for (n, (ok, bad)) in &sites_per_net {
        println!("  fanout net {n}: {ok} sites, {bad} failed");
    }
    drop(checker);
    let work = with_locked(&board, &fanout, true);

    // 2. The region's nets.
    let nets = region_nets(&work, region);
    let open_before = pad_islands(&work, &nets);
    let open_nets = open_before.values().filter(|&&k| k > 1).count();
    let open_conns: usize = open_before.values().map(|k| k.saturating_sub(1)).sum();
    println!(
        "region: {} nets with every pad inside, {open_nets} not complete, {open_conns} connections open",
        nets.len()
    );

    // 3. The pipeline, per thread count.
    let threads: Vec<usize> = std::env::var("KONNECT_REGION_THREADS")
        .unwrap_or_else(|_| "1,0".into())
        .split(',')
        .map(|x| x.trim().parse().expect("thread count"))
        .collect();
    let iterations = std::env::var("KONNECT_REGION_ITERATIONS")
        .ok()
        .and_then(|v| v.trim().parse().ok())
        .unwrap_or(10);
    let scope = PipelineScope {
        nets: Some(nets.clone()),
        region: None,
    };
    let mut runs = Vec::new();
    let mut first_plan: Option<String> = None;
    for &n in &threads {
        let mut settings = PipelineSettings {
            fanout: None,
            threads: n,
            ..PipelineSettings::default()
        };
        settings.negotiation.max_iterations = iterations;
        let t = Instant::now();
        let r: PipelineResult = route_board(&work, &rules, &scope, &settings).expect("plans");
        let wall_s = t.elapsed().as_secs_f64();
        assert_eq!(
            r.detail.routed.len() + r.detail.failed.len(),
            r.connections.len()
        );
        if r.plan.item_count() > 0 {
            r.plan.validate().expect("the plan validates");
        }
        let after = pad_islands(&with_locked(&work, &r.routed, false), &nets);
        let complete = after.values().filter(|&&k| k == 1).count();
        let open_after: usize = after.values().map(|k| k.saturating_sub(1)).sum();
        let mut reasons: BTreeMap<&str, usize> = BTreeMap::new();
        for (_, d) in &r.detail.failure_details {
            *reasons.entry(d.name()).or_default() += 1;
        }
        let target_s = nets.len() as f64 / 6500.0 * 3600.0;
        println!(
            "run threads={} (pool {}): {wall_s:.2} s wall (target {target_s:.1} s); connections \
             attempted {}, routed {}, failed {reasons:?}; region nets complete {complete} of {} \
             ({open_after} connections open); S7 {} findings {:?}; routed {} tracks, {} vias; \
             plan {} tracks, {} vias; {} short segments",
            n,
            r.telemetry.threads,
            r.connections.len(),
            r.detail.routed.len(),
            nets.len(),
            r.check.violations.len(),
            r.check.counts(),
            r.routed.tracks.len(),
            r.routed.vias.len(),
            r.plan.tracks.len(),
            r.plan.vias.len(),
            r.detail.short_segments.len()
        );
        let mut stages = BTreeMap::new();
        for s in [
            Stage::Audit,
            Stage::Access,
            Stage::Global,
            Stage::Detail,
            Stage::Optimize,
            Stage::Check,
            Stage::Plan,
        ] {
            let ms = r.telemetry.stage_us(s) as f64 / 1000.0;
            print!(" {s:?} {ms:.1} ms;");
            stages.insert(format!("{s:?}"), ms);
        }
        println!();
        for it in &r.detail.reports {
            println!(
                "  it {:2} routed {:4} failed {:4} conflicted {:4}",
                it.iteration, it.routed, it.failed, it.conflicted
            );
        }
        let plan_json = r.plan.to_routing_file_json();
        let same = first_plan.as_ref().map(|p| *p == plan_json);
        if let Some(s) = same {
            println!("  plan identical to the first run: {s}");
        } else {
            first_plan = Some(plan_json);
        }
        runs.push(json!({
            "threads": n,
            "pool": r.telemetry.threads,
            "wall_s": wall_s,
            "target_s": target_s,
            "stages_ms": stages,
            "attempted": r.connections.len(),
            "routed": r.detail.routed.len(),
            "failed": reasons,
            "complete_nets": complete,
            "open_connections": open_after,
            "check": r.check.counts().iter().map(|(k, v)| (format!("{k:?}"), *v)).collect::<BTreeMap<_, _>>(),
            "routed_tracks": r.routed.tracks.len(),
            "routed_vias": r.routed.vias.len(),
            "short_segments": r.detail.short_segments.len(),
            "identical_to_first": same,
        }));
    }

    // 4. The candidate on the same nets (matched by name).
    let mut cand_summary = json!(null);
    if let Some(cpath) = candidate(&dir) {
        let t = Instant::now();
        let (cboard, _) = load_board_file(&cpath, &LoadOptions::default()).expect("candidate");
        let cnets: BTreeSet<NetId> = nets
            .iter()
            .filter_map(|&n| cboard.net_by_name(&work.net(n).name))
            .collect();
        let islands = pad_islands(&cboard, &cnets);
        let complete = islands.values().filter(|&&k| k == 1).count();
        let open: usize = islands.values().map(|k| k.saturating_sub(1)).sum();
        let tracks = cboard
            .tracks
            .iter()
            .filter(|t| t.net.is_some_and(|n| cnets.contains(&n)))
            .count();
        let vias = cboard
            .vias
            .iter()
            .filter(|v| v.net.is_some_and(|n| cnets.contains(&n)))
            .count();
        println!(
            "candidate: {} of {} region nets matched, {complete} complete, {open} connections open, \
             {tracks} tracks, {vias} vias on them ({} ms)",
            cnets.len(),
            nets.len(),
            t.elapsed().as_millis()
        );
        cand_summary = json!({
            "matched": cnets.len(),
            "complete_nets": complete,
            "open_connections": open,
            "tracks": tracks,
            "vias": vias,
        });
    }

    if let Some(out) = std::env::var_os("KONNECT_REGION_OUT").map(PathBuf::from) {
        std::fs::create_dir_all(&out).expect("output directory");
        let summary = json!({
            "fanout": {
                "sites": plan.sites.len(),
                "failed": plan.failed.len(),
                "skipped": plan.skipped.len(),
                "plan_ms": fanout_ms,
                "check_findings": fanout_check.violations.len(),
                "per_net": sites_per_net,
            },
            "region_nets": nets.len(),
            "open_nets_before": open_nets,
            "open_connections_before": open_conns,
            "runs": runs,
            "candidate": cand_summary,
        });
        std::fs::write(
            out.join("summary.json"),
            serde_json::to_string_pretty(&summary).unwrap(),
        )
        .expect("summary written");
        println!("wrote {}", out.display());
    }
}
