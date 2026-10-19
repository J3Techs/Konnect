//! Re-runs the end of the native pipeline (S6 with its S7 guard, then the
//! S7 check) offline, on copper a `pipeline_real_fixture` run wrote, so the
//! optimizer can be measured on a real board's routed copper without
//! routing it again. Nothing about the board is committed.
//!
//! ```text
//! KONNECT_PIPELINE_NO_OPTIMIZE=1 KONNECT_PIPELINE_OUT=dir \
//!     cargo test -p konnect-route --release --test pipeline_real_fixture
//! KONNECT_DB1_FIXTURE_DIR=/path/to/fixture KONNECT_PIPELINE_REPLAY=dir \
//!     cargo test -p konnect-route --release --test pipeline_tail_fixture -- --nocapture
//! ```
//!
//! `dir` holds `fanout.json` and `routed.json`. With
//! `KONNECT_PIPELINE_REPLAY_VERBOSE=1` every remaining finding and geometry
//! defect is printed; `KONNECT_PIPELINE_REPLAY_OUT=dir` writes the optimized
//! copper there as `optimized.json`. The test asserts only board-independent invariants:
//! the guard leaves no edited net with more findings, the optimizer opens no
//! connection, and the result is the same on one thread and on many.

mod common;

use konnect_geom::ArcPolicy;
use konnect_route::check::{BoardChecker, CheckOptions, CheckedItem};
use konnect_route::connectivity::ConnectivityReport;
use konnect_route::optimize::{geometry_defects, OptimizeOptions};
use konnect_route::pipeline::{add_locked, optimize_guarded};
use konnect_route::plan::RoutedCopper;
use konnect_route::telemetry::Telemetry;
use konnect_route::{load_board_file, LoadOptions, ProjectRuleTable, RuleTable};
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

fn concat(a: &RoutedCopper, b: &RoutedCopper) -> RoutedCopper {
    RoutedCopper {
        tracks: a.tracks.iter().chain(&b.tracks).copied().collect(),
        vias: a.vias.iter().chain(&b.vias).copied().collect(),
    }
}

fn short_segments(rules: &dyn RuleTable, copper: &RoutedCopper) -> usize {
    copper
        .tracks
        .iter()
        .filter(|t| {
            let s = konnect_route::rules::RuleSubject::new(
                konnect_route::api::ItemKind::Track,
                Some(t.net),
            );
            rules
                .min_segment_length(&s, t.layer)
                .is_some_and(|m| (t.seg.b - t.seg.a).length2() < i128::from(m) * i128::from(m))
        })
        .count()
}

#[test]
fn pipeline_tail_on_routed_copper() {
    let Some(dir) = std::env::var_os("KONNECT_DB1_FIXTURE_DIR").map(PathBuf::from) else {
        eprintln!("skipping: KONNECT_DB1_FIXTURE_DIR is not set");
        return;
    };
    let Some(replay) = std::env::var_os("KONNECT_PIPELINE_REPLAY").map(PathBuf::from) else {
        eprintln!("skipping: KONNECT_PIPELINE_REPLAY is not set");
        return;
    };
    let (Some(pcb), Some(pro)) = (only_file(&dir, "kicad_pcb"), only_file(&dir, "kicad_pro"))
    else {
        eprintln!("skipping: {} lacks a board and project", dir.display());
        return;
    };
    let read = |name: &str| -> Option<RoutedCopper> {
        let text = std::fs::read_to_string(replay.join(name)).ok()?;
        Some(serde_json::from_str(&text).expect("copper parses"))
    };
    let (Some(fanout), Some(raw)) = (read("fanout.json"), read("routed.json")) else {
        eprintln!(
            "skipping: {} lacks fanout.json and routed.json",
            replay.display()
        );
        return;
    };
    let verbose = std::env::var("KONNECT_PIPELINE_REPLAY_VERBOSE").as_deref() == Ok("1");

    let (mut board, _) = load_board_file(&pcb, &LoadOptions::default()).expect("board loads");
    let pro_text = std::fs::read_to_string(&pro).expect("project reads");
    let dru_text =
        only_file(&dir, "kicad_dru").map(|p| std::fs::read_to_string(p).expect("rules read"));
    let rules = ProjectRuleTable::from_project(&pro_text, dru_text.as_deref(), &board)
        .expect("rules compile");
    rules.assign_classes(&mut board);
    let mut work = board.clone();
    add_locked(&mut work, &fanout);

    // `KONNECT_PIPELINE_REPLAY_OFF=a,b`: optimizer passes to turn off.
    let mut opts = OptimizeOptions::default();
    for pass in std::env::var("KONNECT_PIPELINE_REPLAY_OFF")
        .unwrap_or_default()
        .split(',')
    {
        match pass.trim() {
            "" => {}
            "split" => opts.split_junctions = false,
            "merge" => opts.merge_collinear = false,
            "vias" => opts.remove_vias = false,
            "pull" => opts.pull_tight = false,
            "corners" => opts.corner_cuts = false,
            "trim" => opts.trim_pad_entries = false,
            "jogs" => opts.smooth_jogs = false,
            "square" => opts.square_junctions = false,
            "prune" => opts.prune_stubs = false,
            "skew" => opts.max_skew_deg = 0.0,
            other => panic!("unknown pass {other}"),
        }
    }
    let checker = BoardChecker::new(&board, &rules, CheckOptions::for_routed());
    let raw_check = checker.check_routed(&concat(&fanout, &raw));
    let raw_defects = geometry_defects(&rules, &opts, &raw);
    let kinds = |d: &[(
        konnect_route::api::NetId,
        konnect_route::api::LayerId,
        konnect_route::optimize::lint::Defect,
    )]| {
        let mut m: BTreeMap<String, usize> = BTreeMap::new();
        for (_, _, x) in d {
            *m.entry(format!("{:?}", x.kind)).or_default() += 1;
        }
        m
    };
    println!(
        "raw: {} tracks, {} vias, {} short segments; S7 {:?}; lint {:?}",
        raw.tracks.len(),
        raw.vias.len(),
        short_segments(&rules, &raw),
        raw_check.counts(),
        kinds(&raw_defects)
    );

    let mut routed = raw.clone();
    let mut tel = Telemetry::default();
    let t = Instant::now();
    let (report, reverted) = optimize_guarded(&work, &rules, &mut routed, &opts, true, &mut tel);
    let ms = t.elapsed().as_millis();
    let combined = concat(&fanout, &routed);
    let check = checker.check_routed(&combined);
    let defects = geometry_defects(&rules, &opts, &routed);
    println!(
        "optimized in {ms} ms: tracks {} -> {}, length {:.1} -> {:.1} mm, {} nets put back",
        report.tracks_before,
        report.tracks_after,
        report.length_before_nm as f64 / 1e6,
        report.length_after_nm as f64 / 1e6,
        reverted.len()
    );
    println!("report: {report:?}");
    println!(
        "after: {} short segments; S7 {} {:?}; lint {} {:?}",
        short_segments(&rules, &routed),
        check.violations.len(),
        check.counts(),
        defects.len(),
        kinds(&defects)
    );
    let strict: usize = defects.iter().filter(|d| d.2.kind.is_lint_kind()).count();
    println!("strict lint kinds: {strict}");
    if verbose {
        let class = |n: konnect_route::api::NetId| {
            let c = rules.class_of(Some(n));
            rules.class(c).name.clone()
        };
        // What the guard undid: the optimizer's own copper for those nets.
        let mut unguarded = raw.clone();
        optimize_guarded(
            &work,
            &rules,
            &mut unguarded,
            &opts,
            false,
            &mut Telemetry::default(),
        );
        let ucombined = concat(&fanout, &unguarded);
        let ucheck = checker.check_routed(&ucombined);
        for v in &ucheck.violations {
            let net = |i: CheckedItem| match i {
                CheckedItem::RoutedTrack { index } => {
                    ucombined.tracks.get(index as usize).map(|t| (t.net, t.seg))
                }
                CheckedItem::RoutedVia { index } => ucombined.vias.get(index as usize).map(|v| {
                    (
                        v.net,
                        konnect_geom::Segment::new(v.center, v.center, v.diameter),
                    )
                }),
                _ => None,
            };
            let a = net(v.a);
            let b = v.b.and_then(net);
            if a.is_some_and(|(n, _)| reverted.contains(&n))
                || b.is_some_and(|(n, _)| reverted.contains(&n))
            {
                println!(
                    "  unguarded {:?} at ({:.4}, {:.4}) layer {:?} a={:?} b={:?} other={:?}",
                    v.kind,
                    v.at.x as f64 / 1e6,
                    v.at.y as f64 / 1e6,
                    v.layer,
                    a,
                    b,
                    v.b
                );
            }
        }
        for v in &check.violations {
            let net = |i: CheckedItem| match i {
                CheckedItem::RoutedTrack { index } => {
                    combined.tracks.get(index as usize).map(|t| (t.net, t.seg))
                }
                _ => None,
            };
            let a = net(v.a);
            let b = v.b.and_then(net);
            println!(
                "  finding {:?} at ({:.4}, {:.4}) layer {:?} a={:?} b={:?} class={:?}",
                v.kind,
                v.at.x as f64 / 1e6,
                v.at.y as f64 / 1e6,
                v.layer,
                a,
                b,
                a.map(|(n, _)| class(n))
            );
        }
        for (n, l, d) in &defects {
            println!(
                "  defect {:?} net {:?} {} layer {:?} at ({:.4}, {:.4})",
                d.kind,
                n,
                class(*n),
                l,
                d.at.x as f64 / 1e6,
                d.at.y as f64 / 1e6
            );
        }
    }

    if let Some(out) = std::env::var_os("KONNECT_PIPELINE_REPLAY_OUT").map(PathBuf::from) {
        std::fs::create_dir_all(&out).expect("output directory");
        std::fs::write(
            out.join("optimized.json"),
            serde_json::to_string(&routed).unwrap(),
        )
        .expect("optimized copper written");
    }

    // Board-independent invariants.
    let policy = ArcPolicy::default();
    let open = |copper: &RoutedCopper| {
        let mut b = board.clone();
        add_locked(&mut b, &concat(&fanout, copper));
        ConnectivityReport::build(&b, &policy).connection_count()
    };
    let (open_raw, open_opt) = (open(&raw), open(&routed));
    println!("connections open: raw {open_raw}, optimized {open_opt}");
    assert!(open_opt <= open_raw, "the optimizer opened a connection");
    let split_nets = common::nets_split(
        &common::anchor_islands(&work, &raw),
        &common::anchor_islands(&work, &routed),
    );
    println!("nets split by anchors: {}", split_nets.len());
    assert!(split_nets.is_empty(), "anchor joins lost: {split_nets:?}");
    if std::env::var("KONNECT_PIPELINE_REPLAY_DETERMINISM").as_deref() == Ok("1") {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(1)
            .build()
            .unwrap();
        let mut one = raw.clone();
        pool.install(|| {
            optimize_guarded(
                &work,
                &rules,
                &mut one,
                &opts,
                true,
                &mut Telemetry::default(),
            )
        });
        assert_eq!(one, routed, "1 and N threads give the same copper");
        println!("determinism: 1 thread matches");
    }
}
