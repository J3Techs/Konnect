//! Plane fanout (S2) on a real board from outside the repository, as the
//! pipeline runs it, with each failed pad's reason printed. Nothing about
//! the board is committed: the directory comes from
//! `KONNECT_DB1_FIXTURE_DIR`, files are found by extension, and only
//! board-independent invariants are asserted (determinism across thread
//! counts, failures carry a reason and their rejection counts).
//!
//! ```text
//! KONNECT_DB1_FIXTURE_DIR=/path/to/fixture cargo test -p konnect-route --release \
//!     --test fanout_real_fixture -- --nocapture
//! ```
//!
//! `KONNECT_FANOUT_SHARE_MM` sets via sharing within that distance (default
//! off); `KONNECT_FANOUT_RETRY=0` turns the retry round off.

use konnect_geom::ArcPolicy;
use konnect_route::connectivity::ConnectivityReport;
use konnect_route::fanout::{plan_plane_vias, FanoutFailure, FanoutSettings, PlaneViaPlan};
use konnect_route::{load_board_file, LoadOptions, ProjectRuleTable, RouteBoard};
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

fn pad_name(board: &RouteBoard, p: konnect_route::api::PadId) -> String {
    let pad = &board.pads[p.index()];
    format!(
        "{}.{}",
        board.footprints[pad.footprint.index()]
            .reference
            .as_deref()
            .unwrap_or("?"),
        pad.number
    )
}

fn print_plan(board: &RouteBoard, label: &str, p: &PlaneViaPlan, ms: u128) {
    let mut why: BTreeMap<String, usize> = BTreeMap::new();
    for (_, f) in &p.failed {
        *why.entry(format!("{f:?}")).or_default() += 1;
    }
    println!(
        "== {label}: {} sites ({} shared, {} from the retry round), {} pads failed {why:?}, {ms} ms",
        p.sites.len(),
        p.stats.shared_vias,
        p.stats.retried_sites,
        p.failed.len()
    );
    for d in &p.details {
        let f = p
            .failed
            .iter()
            .find(|(pad, _)| *pad == d.pad)
            .map(|(_, f)| f.clone())
            .unwrap_or(FanoutFailure::NoLegalSite);
        let mut top: Vec<(&String, &u32)> = d.rejections.iter().collect();
        top.sort_by(|a, b| b.1.cmp(a.1).then(a.0.cmp(b.0)));
        let blocked: Vec<String> = d.blocked_by.iter().map(|b| pad_name(board, *b)).collect();
        println!(
            "  {:12} {:16} {:?} candidates {} legal {} top {:?} blocked by {:?}",
            pad_name(board, d.pad),
            board.net(d.net).name,
            f,
            d.candidates,
            d.legal,
            &top[..top.len().min(3)],
            blocked
        );
    }
}

#[test]
fn fanout_on_the_real_board() {
    let Some(dir) = std::env::var_os("KONNECT_DB1_FIXTURE_DIR").map(PathBuf::from) else {
        eprintln!("skipping: KONNECT_DB1_FIXTURE_DIR is not set");
        return;
    };
    let (Some(pcb), Some(pro)) = (only_file(&dir, "kicad_pcb"), only_file(&dir, "kicad_pro"))
    else {
        eprintln!("skipping: no board or project in {}", dir.display());
        return;
    };
    let (mut board, _) = load_board_file(&pcb, &LoadOptions::default()).expect("board loads");
    let pro_text = std::fs::read_to_string(&pro).unwrap();
    let dru = only_file(&dir, "kicad_dru").map(|p| std::fs::read_to_string(p).unwrap());
    let rules = ProjectRuleTable::from_project(&pro_text, dru.as_deref(), &board).unwrap();
    rules.assign_classes(&mut board);
    let conn = ConnectivityReport::build(&board, &ArcPolicy::default());

    let mut settings = FanoutSettings::default();
    if let Ok(v) = std::env::var("KONNECT_FANOUT_SHARE_MM") {
        let mm: f64 = v.trim().parse().expect("mm");
        settings.share_within = (mm * 1e6).round() as i32;
    }
    if std::env::var("KONNECT_FANOUT_RETRY").as_deref() == Ok("0") {
        settings.retry_failed = false;
    }
    let t = Instant::now();
    let plan = plan_plane_vias(&board, &rules, &conn, &settings);
    print_plan(&board, "fanout", &plan, t.elapsed().as_millis());
    let single = rayon::ThreadPoolBuilder::new()
        .num_threads(1)
        .build()
        .unwrap()
        .install(|| plan_plane_vias(&board, &rules, &conn, &settings));
    assert_eq!(
        single.sites, plan.sites,
        "1-thread and N-thread plans differ"
    );
    assert_eq!(single.failed, plan.failed);
    assert_eq!(plan.failed.len(), plan.details.len());
    // The fanout alone is S7-clean.
    let mut copper = plan.routed();
    for v in &mut copper.vias {
        v.layers = board.layers.all();
    }
    let report = konnect_route::check::check_routed(&board, &rules, &copper);
    for v in &report.violations {
        let name = |i: konnect_route::check::CheckedItem| match i {
            konnect_route::check::CheckedItem::RoutedTrack { index } => {
                let t = &copper.tracks[index as usize];
                format!("stub {} {:?}", board.net(t.net).name, t.seg)
            }
            konnect_route::check::CheckedItem::RoutedVia { index } => {
                format!("via {:?}", copper.vias[index as usize].center)
            }
            other => format!("{other:?}"),
        };
        println!(
            "  finding {:?} at ({:.3},{:.3}): {} / {}",
            v.kind,
            v.at.x as f64 / 1e6,
            v.at.y as f64 / 1e6,
            name(v.a),
            v.b.map(name).unwrap_or_default()
        );
    }
    assert!(report.violations.is_empty(), "the fanout has S7 findings");
    for d in &plan.details {
        assert!(d.candidates > 0 || d.rejections.is_empty());
    }
}
