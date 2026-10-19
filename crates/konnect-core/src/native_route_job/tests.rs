use super::*;
use konnect_route::plan::PlanTrack;

const PRO: &str = r#"{
    "board": { "design_settings": { "rules": {
        "min_clearance": 0.1, "min_track_width": 0.1, "min_via_diameter": 0.4,
        "min_via_annular_width": 0.1, "min_through_hole_diameter": 0.2,
        "min_copper_edge_clearance": 0.3, "min_hole_clearance": 0.2, "min_hole_to_hole": 0.25
    } } },
    "net_settings": {
        "classes": [
            { "name": "Default", "clearance": 0.2, "track_width": 0.2,
              "via_diameter": 0.6, "via_drill": 0.3 }
        ],
        "netclass_patterns": []
    }
}"#;

struct Fixture {
    _dir: tempfile::TempDir,
    root: PathBuf,
}

fn fixture() -> Fixture {
    let dir = tempfile::tempdir().unwrap();
    let root = dir.path().to_path_buf();
    std::fs::write(
        root.join("demo.kicad_pcb"),
        konnect_route::synthetic::MINIMAL_KICAD10_BOARD,
    )
    .unwrap();
    std::fs::write(root.join("demo.kicad_pro"), PRO).unwrap();
    Fixture { _dir: dir, root }
}

fn spec(root: &Path, out: &str, source: CopperSource) -> NativeJobSpec {
    NativeJobSpec {
        board: root.join("demo.kicad_pcb"),
        project: None,
        rules: None,
        output_plan: root.join(out),
        source,
        region_mm: None,
        nets: None,
        threads: Some(2),
        optimize: true,
        zones_are_obstacles: true,
        time_limit_seconds: None,
    }
}

/// A 50 µm staircase on B.Cu in open board, written as a routing_file.
fn staircase_plan(root: &Path) -> PathBuf {
    let mut tracks = Vec::new();
    let (mut x, mut y) = (5.0f64, 5.0f64);
    while x < 8.0 - 1e-9 {
        let t = |x1: f64, y1: f64, x2: f64, y2: f64| PlanTrack {
            net: "GND".into(),
            layer: "B.Cu".into(),
            width: 0.2,
            x1,
            y1,
            x2,
            y2,
        };
        tracks.push(t(x, y, x + 0.05, y));
        tracks.push(t(x + 0.05, y, x + 0.05, y + 0.05));
        x += 0.05;
        y += 0.05;
    }
    // Round to the nm so the plan is exact.
    for t in &mut tracks {
        for v in [&mut t.x1, &mut t.y1, &mut t.x2, &mut t.y2] {
            *v = (*v * 1e6).round() / 1e6;
        }
    }
    let plan = RoutePlan {
        tracks,
        vias: vec![],
    };
    let path = root.join("seed.json");
    std::fs::write(&path, plan.to_routing_file_json()).unwrap();
    path
}

fn progress(dir: &Path) -> Vec<Value> {
    std::fs::read_to_string(dir.join(PROGRESS_FILE))
        .unwrap()
        .lines()
        .map(|l| serde_json::from_str(l).unwrap())
        .collect()
}

#[test]
fn a_job_directory_runs_to_a_plan_with_progress_and_a_report() {
    let f = fixture();
    let seed = staircase_plan(&f.root);
    let s = spec(&f.root, "out.json", CopperSource::Plan { path: seed });
    let dir = prepare(&s, false).unwrap();
    let status = run_job_dir(&dir).unwrap();
    assert_eq!(status.state, "COMPLETED", "{status:?}");
    assert!(status.finished);
    let plan: RoutePlan =
        serde_json::from_str(&std::fs::read_to_string(&s.output_plan).unwrap()).unwrap();
    assert!(plan.item_count() >= 1 && plan.item_count() <= 3, "{plan:?}");
    assert_eq!(status.plan_items, Some(plan.item_count()));

    let events = progress(&dir);
    let done: Vec<&str> = events
        .iter()
        .filter(|e| e["event"] == "done")
        .map(|e| e["stage"].as_str().unwrap())
        .collect();
    assert_eq!(
        done,
        [
            "ingest",
            "connectivity",
            "route",
            "optimize",
            "check",
            "plan"
        ]
    );
    let report: PipelineReport = read_json(&dir.join(REPORT_FILE)).unwrap();
    let opt = report.optimize.unwrap();
    assert!(opt.tracks_before > 100 && opt.tracks_after <= 3, "{opt:?}");
    let check = report.check.unwrap();
    assert_eq!(
        (check.checker.as_str(), check.violations),
        ("interim_exact", 0)
    );
    assert!(report.telemetry.stage_us(Stage::Optimize) > 0);
    let cp: Checkpoint = read_json(&dir.join(CHECKPOINT_FILE)).unwrap();
    assert_eq!(cp.completed, ["route", "optimize", "check"]);
    // The tools' view of the finished job.
    let body = status_of(&s.output_plan);
    assert_eq!(body["state"], "COMPLETED");
    assert_eq!(body["runner_alive"], false);
    assert_eq!(body["plan_exists"], true);
    assert_eq!(body["report"]["plan_items"], json!(plan.item_count()));
}

fn status_of(output: &Path) -> Value {
    status(output).unwrap()
}

#[test]
fn board_copper_can_be_reoptimized_and_a_region_freezes_the_rest() {
    let f = fixture();
    let mut s = spec(&f.root, "board.json", CopperSource::BoardUnlocked);
    let dir = prepare(&s, false).unwrap();
    assert_eq!(run_job_dir(&dir).unwrap().state, "COMPLETED");
    let report: PipelineReport = read_json(&dir.join(REPORT_FILE)).unwrap();
    // The board's one unlocked track and via.
    assert_eq!((report.work_tracks, report.work_vias), (1, 1));

    // A region away from that copper leaves nothing to plan.
    s.output_plan = f.root.join("empty.json");
    s.region_mm = Some([1.0, 1.0, 5.0, 5.0]);
    let dir = prepare(&s, false).unwrap();
    assert_eq!(run_job_dir(&dir).unwrap().state, "COMPLETED");
    let report: PipelineReport = read_json(&dir.join(REPORT_FILE)).unwrap();
    assert_eq!((report.work_tracks, report.work_vias), (0, 0));
    assert_eq!(report.stages.last().unwrap().state, "skipped");
    assert!(!s.output_plan.exists());
}

#[test]
fn an_unbuilt_native_stage_ends_the_run_as_unavailable_not_failed() {
    let f = fixture();
    let s = spec(&f.root, "native.json", CopperSource::Native);
    let dir = prepare(&s, false).unwrap();
    let status = run_job_dir(&dir).unwrap();
    assert_eq!(status.state, "STAGE_UNAVAILABLE", "{status:?}");
    assert!(status.error.unwrap().contains("not available"));
    assert!(!s.output_plan.exists());
    let events = progress(&dir);
    assert!(events
        .iter()
        .any(|e| e["stage"] == "route" && e["event"] == "failed"));
}

#[test]
fn a_stopped_job_resumes_from_its_checkpoint() {
    let f = fixture();
    let seed = staircase_plan(&f.root);
    let s = spec(&f.root, "stop.json", CopperSource::Plan { path: seed });
    let dir = prepare(&s, false).unwrap();
    std::fs::write(dir.join(STOP_FILE), b"").unwrap();
    let status = run_job_dir(&dir).unwrap();
    assert_eq!(status.state, "STOPPED_ON_REQUEST");
    assert!(!s.output_plan.exists());
    let cp: Checkpoint = read_json(&dir.join(CHECKPOINT_FILE)).unwrap();
    assert_eq!(cp.completed, ["route"]);

    // Starting it again needs resume, and the same spec.
    assert!(prepare(&s, false)
        .unwrap_err()
        .to_string()
        .contains("resume: true"));
    let mut other = s.clone();
    other.threads = Some(1);
    assert!(prepare(&other, true)
        .unwrap_err()
        .to_string()
        .contains("same spec"));
    let dir = prepare(&s, true).unwrap();
    assert!(
        !dir.join(STOP_FILE).exists(),
        "resume clears the stop request"
    );
    let status = run_job_dir(&dir).unwrap();
    assert_eq!(status.state, "COMPLETED");
    assert!(status.resumed);
    let report: PipelineReport = read_json(&dir.join(REPORT_FILE)).unwrap();
    let route = report.stages.iter().find(|r| r.stage == "route").unwrap();
    assert_eq!(route.state, "resumed");
    assert!(s.output_plan.is_file());
}

#[test]
fn bad_specs_are_refused_before_any_runner_starts() {
    let f = fixture();
    let mut s = spec(&f.root, "x.json", CopperSource::BoardUnlocked);
    s.region_mm = Some([5.0, 5.0, 1.0, 1.0]);
    assert!(prepare(&s, false).is_err());
    s.region_mm = None;
    s.board = f.root.join("missing.kicad_pcb");
    assert!(prepare(&s, false)
        .unwrap_err()
        .to_string()
        .contains("not a file"));
    let s = spec(&f.root, "demo.kicad_pro", CopperSource::BoardUnlocked);
    assert!(prepare(&s, false)
        .unwrap_err()
        .to_string()
        .contains("never replaced"));
    assert!(!job_dir_for(&s.output_plan).exists());
}

#[test]
fn several_jobs_run_at_once_and_agree() {
    let f = fixture();
    let seed = staircase_plan(&f.root);
    let specs: Vec<NativeJobSpec> = (0..3)
        .map(|i| {
            spec(
                &f.root,
                &format!("par{i}.json"),
                CopperSource::Plan { path: seed.clone() },
            )
        })
        .collect();
    let dirs: Vec<PathBuf> = specs.iter().map(|s| prepare(s, false).unwrap()).collect();
    let handles: Vec<_> = dirs
        .iter()
        .cloned()
        .map(|d| std::thread::spawn(move || run_job_dir(&d).unwrap()))
        .collect();
    for h in handles {
        assert_eq!(h.join().unwrap().state, "COMPLETED");
    }
    let plans: Vec<String> = specs
        .iter()
        .map(|s| std::fs::read_to_string(&s.output_plan).unwrap())
        .collect();
    assert!(plans.windows(2).all(|w| w[0] == w[1]));
}

#[test]
fn a_silent_runner_is_reported_lost() {
    let f = fixture();
    let dir = f.root.join("lost.konnect-native-job");
    std::fs::create_dir_all(&dir).unwrap();
    let status = JobStatus {
        state: "RUNNING".into(),
        heartbeat_unix: now_unix() - STALE_HEARTBEAT_SECONDS - 5,
        ..JobStatus::default()
    };
    let body = report(&dir, &status);
    assert_eq!(body["state"], "RUNNER_LOST");
    assert_eq!(body["runner_alive"], false);
}

#[test]
fn a_source_plan_naming_a_missing_net_fails_as_an_input_error() {
    let f = fixture();
    let plan = RoutePlan {
        tracks: vec![PlanTrack {
            net: "NO_SUCH_NET".into(),
            layer: "B.Cu".into(),
            width: 0.2,
            x1: 5.0,
            y1: 5.0,
            x2: 6.0,
            y2: 5.0,
        }],
        vias: vec![],
    };
    let seed = f.root.join("bad-seed.json");
    std::fs::write(&seed, plan.to_routing_file_json()).unwrap();
    let s = spec(&f.root, "bad.json", CopperSource::Plan { path: seed });
    let dir = prepare(&s, false).unwrap();
    let status = run_job_dir(&dir).unwrap();
    assert_eq!(status.state, "FAILED", "{status:?}");
    assert!(status.error.unwrap().contains("source plan"));
    let report: PipelineReport = read_json(&dir.join(REPORT_FILE)).unwrap();
    assert_eq!(report.stop_kind, Some(StopKind::InputError));
    assert!(!s.output_plan.exists());
}

#[test]
fn only_an_unbuilt_stub_reads_as_unavailable() {
    let todo = guarded("x", || -> u32 { todo!("WS-X: later") }).unwrap_err();
    assert!(matches!(todo, StageError::Unavailable { .. }), "{todo}");
    let unimpl = guarded("x", || -> u32 { unimplemented!() }).unwrap_err();
    assert!(matches!(unimpl, StageError::Unavailable { .. }), "{unimpl}");
    let bug = guarded("x", || -> u32 { panic!("index out of bounds") }).unwrap_err();
    assert!(matches!(bug, StageError::Panicked { .. }), "{bug}");
    assert!(bug.to_string().contains("index out of bounds"));
    let as_anyhow = anyhow::Error::new(bug);
    assert_eq!(StopKind::of(&as_anyhow), StopKind::StageFailed);
    assert_eq!(StopKind::of(&as_anyhow).job_state(), "FAILED");
    assert_eq!(
        StopKind::of(&anyhow::Error::new(todo)).job_state(),
        "STAGE_UNAVAILABLE"
    );
    assert_eq!(
        StopKind::of(&anyhow::anyhow!("source plan: bad")).job_state(),
        "FAILED"
    );
}

#[test]
fn native_routing_sees_only_the_selected_nets_and_region() {
    use konnect_route::synthetic::BoardBuilder;
    let mut b = BoardBuilder::new(40.0, 20.0, &["F.Cu", "B.Cu"]);
    let u = b.footprint("U1", (5.0, 5.0));
    b.smd_pad(u, "1", "ALPHA", (5.0, 5.0), (0.5, 0.5), "F.Cu");
    b.smd_pad(u, "2", "BETA", (5.0, 8.0), (0.5, 0.5), "F.Cu");
    let v = b.footprint("U2", (10.0, 5.0));
    b.smd_pad(v, "1", "ALPHA", (10.0, 5.0), (0.5, 0.5), "F.Cu");
    b.smd_pad(v, "2", "BETA", (10.0, 8.0), (0.5, 0.5), "F.Cu");
    let w = b.footprint("U3", (35.0, 5.0));
    b.smd_pad(w, "1", "ALPHA", (35.0, 5.0), (0.5, 0.5), "F.Cu");
    let board = b.build();
    let all = || {
        ConnectivityReport::build(&board, &konnect_geom::ArcPolicy::default()).connections(&board)
    };
    assert_eq!(all().len(), 3);
    assert_eq!(select_connections(&board, all(), None, None).len(), 3);
    let alpha: BTreeSet<NetId> = [board.net_by_name("ALPHA").unwrap()].into();
    let picked = select_connections(&board, all(), Some(&alpha), None);
    assert_eq!(picked.len(), 2);
    assert!(picked.iter().all(|c| alpha.contains(&c.net)));
    // A box round the two left footprints drops the long ALPHA link.
    let region = konnect_geom::BBox::new(
        konnect_geom::Point::from_mm(2.0, 2.0).unwrap(),
        konnect_geom::Point::from_mm(15.0, 12.0).unwrap(),
    );
    let picked = select_connections(&board, all(), None, Some(region));
    assert_eq!(picked.len(), 2);
    let picked = select_connections(&board, all(), Some(&alpha), Some(region));
    assert_eq!(picked.len(), 1);
    assert_eq!(picked[0].id.0, 0, "ids are renumbered");
}

#[test]
fn a_native_run_with_nothing_selected_routes_nothing_and_completes() {
    let f = fixture();
    let mut s = spec(&f.root, "nothing.json", CopperSource::Native);
    // A corner of the board no connection lies inside.
    s.region_mm = Some([0.0, 0.0, 0.5, 0.5]);
    let dir = prepare(&s, false).unwrap();
    let status = run_job_dir(&dir).unwrap();
    assert_eq!(status.state, "COMPLETED", "{status:?}");
    let report: PipelineReport = read_json(&dir.join(REPORT_FILE)).unwrap();
    assert_eq!((report.work_tracks, report.work_vias), (0, 0));
    assert!(!s.output_plan.exists());
}
