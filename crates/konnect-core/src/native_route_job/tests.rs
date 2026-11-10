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
        carve_zones: Vec::new(),
        carve_zone_nets: Vec::new(),
        plane_nets_as_tracks: Vec::new(),
        clear_findings: None,
        time_limit_seconds: None,
        allowances: Default::default(),
        net_classes: None,
        fanout_plan: None,
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
        (
            check.checker.as_str(),
            check.violations,
            check.authoritative
        ),
        ("check_routed", 0, true)
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
    // The report file holds every connection the run leaves open, by pad;
    // the tools' view lists them and says where the file is.
    let open = report.unrouted.expect("the open connections");
    assert_eq!(open.total, open.connections.len());
    assert!(open.total >= 1, "{open:?}");
    for c in &open.connections {
        assert_eq!(c.reason, "not_routed", "{c:?}");
        assert!(c.from.reference.is_some() && c.from.pad.is_some(), "{c:?}");
        assert!(c.to.reference.is_some() && c.to.pad.is_some(), "{c:?}");
    }
    let body = status_of(&s.output_plan);
    let listed = &body["report"]["unrouted"];
    assert_eq!(listed["total"], json!(open.total));
    assert_eq!(listed["by_reason"]["not_routed"], json!(open.total));
    assert_eq!(
        listed["full_list"],
        json!(dir.join(REPORT_FILE).display().to_string())
    );
    let capped = status_listing(&s.output_plan, 0).unwrap();
    assert_eq!(capped["report"]["unrouted"]["listed"], 0);
    assert!(capped["report"]["unrouted"]["note"]
        .as_str()
        .unwrap()
        .contains("report.json"));

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
fn a_native_run_routes_through_every_stage_and_reports_them() {
    let f = fixture();
    let s = spec(&f.root, "native.json", CopperSource::Native);
    let dir = prepare(&s, false).unwrap();
    let status = run_job_dir(&dir).unwrap();
    assert_eq!(status.state, "COMPLETED", "{status:?}");
    let report: PipelineReport = read_json(&dir.join(REPORT_FILE)).unwrap();
    let route = report
        .stages
        .iter()
        .find(|r| r.stage == "route")
        .expect("a route stage record");
    assert_eq!(route.state, "done");
    for key in ["tasks", "routed", "failed", "connections_after"] {
        assert!(
            route.detail.get(key).is_some(),
            "{key} in {:?}",
            route.detail
        );
    }
    // The pipeline's own stage timings reach the job's telemetry.
    assert!(report
        .telemetry
        .stages
        .iter()
        .any(|t| t.stage == Stage::Global));
    let planned = report.work_tracks + report.work_vias + report.fanout_tracks + report.fanout_vias;
    assert_eq!(s.output_plan.exists(), planned > 0, "{report:?}");
    let events = progress(&dir);
    assert!(events
        .iter()
        .any(|e| e["stage"] == "route" && e["event"] == "done"));
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
    assert_eq!(
        konnect_route::pipeline::select_connections(&board, all(), None, None).len(),
        3
    );
    let alpha: BTreeSet<NetId> = [board.net_by_name("ALPHA").unwrap()].into();
    let picked = konnect_route::pipeline::select_connections(&board, all(), Some(&alpha), None);
    assert_eq!(picked.len(), 2);
    assert!(picked.iter().all(|c| alpha.contains(&c.net)));
    // A box round the two left footprints drops the long ALPHA link.
    let region = konnect_geom::BBox::new(
        konnect_geom::Point::from_mm(2.0, 2.0).unwrap(),
        konnect_geom::Point::from_mm(15.0, 12.0).unwrap(),
    );
    let picked = konnect_route::pipeline::select_connections(&board, all(), None, Some(region));
    assert_eq!(picked.len(), 2);
    let picked =
        konnect_route::pipeline::select_connections(&board, all(), Some(&alpha), Some(region));
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

#[test]
fn excluded_classes_are_reported_by_policy_and_kicad_classes_pin_nets() {
    let f = fixture();
    let mut s = spec(&f.root, "policy.json", CopperSource::Native);
    s.allowances.exclude_classes = Some(vec!["Default".to_string()]);
    // KiCad's classes: one net named in a class the project lacks.
    s.net_classes = Some(BTreeMap::from([(
        "GND".to_string(),
        vec!["Elsewhere".to_string()],
    )]));
    let report = run_pipeline(&s, None, &mut Quiet).unwrap();
    assert!(report.stopped.is_none(), "{report:?}");
    let classes = report.net_classes.as_ref().unwrap();
    assert_eq!(classes.source, "kicad_ipc");
    assert_eq!(classes.unknown_classes, vec!["Elsewhere".to_string()]);
    let allowances = report.allowances.as_ref().unwrap();
    assert_eq!(allowances.excluded_classes, vec!["Default".to_string()]);
    assert!(!allowances.exclusion_is_default);
    let route = report.stages.iter().find(|r| r.stage == "route").unwrap();
    // Nothing of an excluded class is routed or failed (the pipeline's own
    // tests cover the split; this board's one connection is fanout's).
    assert_eq!(route.detail["tasks"], 0, "{:?}", route.detail);
    assert_eq!(route.detail["failed"], json!({}));
    assert!(route.detail.get("excluded_by_policy").is_some());
}

#[test]
fn unknown_allowance_names_stop_the_run_as_an_input_error() {
    let f = fixture();
    let mut s = spec(&f.root, "bad.json", CopperSource::Native);
    s.allowances.class_layers = BTreeMap::from([("Nope".to_string(), vec!["F.Cu".to_string()])]);
    let report = run_pipeline(&s, None, &mut Quiet).unwrap();
    assert_eq!(report.stop_kind, Some(StopKind::InputError));
    assert!(report.stopped.as_deref().unwrap().contains("Nope"));
    assert!(!s.output_plan.exists());
}

#[test]
fn a_fanout_plan_must_be_a_new_file_of_its_own() {
    let f = fixture();
    let mut s = spec(&f.root, "main.json", CopperSource::Native);
    s.fanout_plan = Some(s.output_plan.clone());
    assert!(prepare(&s, false)
        .unwrap_err()
        .to_string()
        .contains("differ"));
    s.fanout_plan = Some(f.root.join("demo.kicad_pro"));
    assert!(prepare(&s, false)
        .unwrap_err()
        .to_string()
        .contains("never replaced"));
    // A default spec keeps its old serialised form (and so its hash).
    let plain = serde_json::to_value(spec(&f.root, "main.json", CopperSource::Native)).unwrap();
    for key in ["allowances", "net_classes", "fanout_plan"] {
        assert!(plain.get(key).is_none(), "{key} in {plain}");
    }
}

/// What `apply_route_plan` says about a written plan under strict.
fn strict_apply_defects(path: &Path) -> usize {
    use crate::tools::{route_geometry, route_native_apply};
    let plan: RoutePlan = serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
    match route_native_apply::preflight(&plan, &route_geometry::Limits::default(), true, 20_000, 0)
    {
        Ok(_) => 0,
        Err(route_native_apply::ApplyRefusal::Geometry { defects, .. }) => defects,
        Err(other) => panic!("{other}"),
    }
}

#[test]
fn a_native_run_lints_its_own_plan_as_apply_route_plan_will() {
    let f = fixture();
    let s = spec(&f.root, "native.json", CopperSource::Native);
    let dir = prepare(&s, false).unwrap();
    assert_eq!(run_job_dir(&dir).unwrap().state, "COMPLETED");
    let report: PipelineReport = read_json(&dir.join(REPORT_FILE)).unwrap();
    assert!(s.output_plan.exists(), "{report:?}");
    let lint = report.plan_lint.expect("a written plan is linted");
    assert_eq!(lint.path, s.output_plan);
    // The report and the apply tool agree, whatever the router drew.
    assert_eq!(lint.total, strict_apply_defects(&s.output_plan));
    assert_eq!(lint.passes_strict, lint.total == 0);
    assert_eq!(lint.defects.len(), lint.total);
    // This board's routes are clean, and the tools say so.
    assert!(lint.passes_strict, "{lint:?}");
    let body = status_of(&s.output_plan);
    assert_eq!(body["report"]["plan_lint"]["passes_strict"], true);
    assert_eq!(body["report"]["plan_lint"]["total"], 0);
    assert!(body["report"]["plan_lint"].get("remedy").is_none());
}

#[test]
fn a_plan_the_strict_lint_will_refuse_is_reported_with_its_tracks_before_it_is_applied() {
    let f = fixture();
    let seed = staircase_plan(&f.root);
    // The optimizer off: the staircase reaches the plan as it is.
    let mut s = spec(&f.root, "stairs.json", CopperSource::Plan { path: seed });
    s.optimize = false;
    let dir = prepare(&s, false).unwrap();
    assert_eq!(run_job_dir(&dir).unwrap().state, "COMPLETED");
    let report: PipelineReport = read_json(&dir.join(REPORT_FILE)).unwrap();
    let lint = report.plan_lint.expect("a written plan is linted");
    assert!(!lint.passes_strict);
    assert!(lint.total > PLAN_LINT_LISTED, "{}", lint.total);
    assert_eq!(lint.total, strict_apply_defects(&s.output_plan));
    assert_eq!(lint.by_kind.get("short_jog"), Some(&lint.total), "{lint:?}");
    // The report file lists every defect, each with its plan tracks.
    assert_eq!(lint.defects.len(), lint.total);
    let plan: RoutePlan =
        serde_json::from_str(&std::fs::read_to_string(&s.output_plan).unwrap()).unwrap();
    let first = &lint.defects[0];
    assert_eq!(
        (&first["net"], &first["layer"]),
        (&json!("GND"), &json!("B.Cu"))
    );
    let tracks = first["tracks"].as_array().unwrap();
    assert_eq!(tracks.len(), 3, "{first}");
    let short = &plan.tracks[tracks[1]["index"].as_u64().unwrap() as usize];
    assert_eq!(tracks[1]["from"], json!([short.x1, short.y1]));
    assert_eq!(tracks[1]["to"], json!([short.x2, short.y2]));
    // The plan stage record counts them; the tool result lists a bounded
    // number, the total, the remedy and where the whole list is.
    let stage = report.stages.last().unwrap();
    assert_eq!(stage.detail["strict_lint_defects"], json!(lint.total));
    let body = status_of(&s.output_plan);
    let shown = &body["report"]["plan_lint"];
    assert_eq!(shown["passes_strict"], false);
    assert_eq!(shown["total"], json!(lint.total));
    assert_eq!(shown["listed"], json!(PLAN_LINT_LISTED));
    assert_eq!(shown["defects"].as_array().unwrap().len(), PLAN_LINT_LISTED);
    assert!(shown["remedy"].as_str().unwrap().contains("index"));
    assert!(
        shown["note"].as_str().unwrap().contains(REPORT_FILE),
        "{shown}"
    );
}

/// The demo board with its GND plane's fill taken out of the saved file.
fn without_fill(root: &Path) {
    let text = konnect_route::synthetic::MINIMAL_KICAD10_BOARD;
    let start = text.find("\t\t(filled_polygon").unwrap();
    let end = start + text[start..].find("\t\t)\n").unwrap() + "\t\t)\n".len();
    let unfilled = format!("{}{}", &text[..start], &text[end..]);
    assert!(!unfilled.contains("filled_polygon"));
    std::fs::write(root.join("demo.kicad_pcb"), unfilled).unwrap();
}

/// The demo board with a filled GND pour named `GND wall` right across
/// every routing layer, between J1.2 (D+) and a new D+ pad R2.1 below it.
fn with_wall(root: &Path) {
    let text = konnect_route::synthetic::MINIMAL_KICAD10_BOARD;
    let end = text.rfind(')').unwrap();
    let band = "(pts (xy 0.1 28) (xy 49.9 28) (xy 49.9 29) (xy 0.1 29))";
    let extra = format!(
        r#"	(footprint "R_0402"
		(layer "F.Cu")
		(uuid "00000000-0000-0000-0000-000000000030")
		(at 30 33)
		(property "Reference" "R2"
			(at 0 -1 0)
			(layer "F.SilkS")
		)
		(pad "1" smd roundrect
			(at 0 0)
			(size 0.6 0.5)
			(layers "F.Cu" "F.Mask" "F.Paste")
			(roundrect_rratio 0.25)
			(net "D+")
			(uuid "00000000-0000-0000-0000-000000000031")
		)
	)
	(zone
		(net "GND")
		(layers "F.Cu" "In2.Cu" "B.Cu")
		(uuid "00000000-0000-0000-0000-000000000032")
		(name "GND wall")
		(polygon {band})
		(filled_polygon (layer "F.Cu") {band})
		(filled_polygon (layer "In2.Cu") {band})
		(filled_polygon (layer "B.Cu") {band})
	)
"#
    );
    let walled = format!("{}{}{}", &text[..end], extra, &text[end..]);
    std::fs::write(root.join("demo.kicad_pcb"), walled).unwrap();
}

fn plan_nets(path: &Path) -> BTreeSet<String> {
    let plan: RoutePlan = serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
    plan.tracks
        .iter()
        .map(|t| t.net.clone())
        .chain(plan.vias.iter().map(|v| v.net.clone()))
        .collect()
}

#[test]
fn a_plane_net_is_left_to_its_plane_and_its_pads_are_accounted_for() {
    let f = fixture();
    let s = spec(&f.root, "plane.json", CopperSource::Native);
    let report = run_pipeline(&s, None, &mut Quiet).unwrap();
    assert!(report.stopped.is_none(), "{report:?}");
    assert_eq!(report.plane_nets.len(), 1, "{:?}", report.plane_nets);
    let gnd = &report.plane_nets[0];
    assert_eq!(
        (gnd.net.as_str(), gnd.fill.as_str(), gnd.handling.as_str()),
        ("GND", "filled", "left_to_plane")
    );
    // J1.1 is a through-hole pad in the plane; R1.2 got the fanout's via.
    assert_eq!(gnd.pads, 2);
    assert_eq!(gnd.connected_by_plane.pads, vec!["J1.1".to_string()]);
    assert_eq!(gnd.connected_by_fanout.pads, vec!["R1.2".to_string()]);
    assert!(gnd.needs_via_or_track.is_empty());
    assert_eq!(gnd.connections_left_unrouted, 0);
    assert!(report.warnings.is_empty(), "{:?}", report.warnings);
    assert!(report.refill.is_none(), "no zone was declared crossable");
    // The report a finished job hands back carries the same account.
    let dir = prepare(&spec(&f.root, "plane2.json", CopperSource::Native), false).unwrap();
    run_job_dir(&dir).unwrap();
    let status = status_of(&f.root.join("plane2.json"));
    assert_eq!(status["report"]["plane_nets"][0]["net"], "GND", "{status}");
    assert_eq!(status["report"]["warnings"], json!([]));
}

#[test]
fn a_plane_net_with_no_fill_is_warned_about_and_routed_as_tracks_only_when_named() {
    let f = fixture();
    without_fill(&f.root);
    let s = spec(&f.root, "unfilled.json", CopperSource::Native);
    let report = run_pipeline(&s, None, &mut Quiet).unwrap();
    assert!(report.stopped.is_none(), "{report:?}");
    let gnd = &report.plane_nets[0];
    assert_eq!(
        (gnd.net.as_str(), gnd.fill.as_str(), gnd.handling.as_str()),
        ("GND", "unfilled", "left_to_plane")
    );
    assert_eq!(gnd.unfilled_zones.len(), 1);
    assert_eq!(gnd.unfilled_zones[0].zone.as_deref(), Some("GND plane"));
    assert_eq!(gnd.unfilled_zones[0].layer, "In1.Cu");
    assert_eq!(gnd.connections_left_unrouted, 1);
    // The connection left open is named by its two ends.
    assert_eq!(gnd.unrouted_connections.len(), 1, "{gnd:?}");
    let c = &gnd.unrouted_connections[0];
    assert!(
        c.from.contains('.') && c.to.contains('.') && c.from != c.to,
        "{c:?}"
    );
    // Nothing of GND was routed: with no other open connection, no plan.
    let route = report.stages.iter().find(|r| r.stage == "route").unwrap();
    assert_eq!(route.detail["tasks"], 0, "{:?}", route.detail);
    assert_eq!(route.detail["left_to_planes"], 1);
    assert_eq!(route.detail["failed"], json!({}));
    assert!(!s.output_plan.exists());
    // The list of open connections holds it, as left to its plane: it is
    // no failure of the run, and nothing is unexplained.
    let open = report.unrouted.as_ref().expect("the run reached the check");
    assert_eq!(open.total, 1, "{open:?}");
    assert_eq!(open.connections[0].net, "GND");
    assert_eq!(open.connections[0].reason, "left_to_plane");
    assert_eq!(
        open.by_reason,
        BTreeMap::from([("left_to_plane".to_string(), 1)])
    );
    assert!(open.route_failures.is_empty());
    let listed = unrouted::listing(&serde_json::to_value(open).unwrap(), 50, None);
    assert!(
        listed["reasons"]["left_to_plane"]
            .as_str()
            .unwrap()
            .contains("plane_nets"),
        "{listed}"
    );
    assert_eq!(report.warnings.len(), 1, "{:?}", report.warnings);
    let w = &report.warnings[0];
    assert!(
        w.contains("GND")
            && w.contains("no zone fill")
            && w.contains("GND plane@In1.Cu")
            && w.contains("left unrouted")
            && w.contains("plane_nets_as_tracks"),
        "{w}"
    );

    // Asked for, the net is routed as tracks; the missing fill is still said.
    let mut s = spec(&f.root, "as_tracks.json", CopperSource::Native);
    s.plane_nets_as_tracks = vec!["GN?".to_string()];
    let report = run_pipeline(&s, None, &mut Quiet).unwrap();
    assert!(report.stopped.is_none(), "{report:?}");
    assert_eq!(report.plane_nets[0].handling, "routed_as_tracks");
    assert_eq!(report.plane_nets[0].connections_left_unrouted, 0);
    let route = report.stages.iter().find(|r| r.stage == "route").unwrap();
    assert_eq!(route.detail["tasks"], 1, "{:?}", route.detail);
    assert_eq!(route.detail["connections_after"], 0, "{:?}", route.detail);
    assert_eq!(
        plan_nets(&s.output_plan),
        BTreeSet::from(["GND".to_string()])
    );
    assert!(
        report.warnings[0].contains("no zone fill")
            && report.warnings[0].contains("routed as tracks"),
        "{:?}",
        report.warnings
    );
}

#[test]
fn a_filled_pour_is_crossed_only_when_declared_and_the_report_asks_for_a_refill() {
    let f = fixture();
    with_wall(&f.root);
    let only_d_plus = |out: &str| {
        let mut s = spec(&f.root, out, CopperSource::Native);
        s.nets = Some(vec!["D+".to_string()]);
        s
    };
    // As saved the wall is copper: the connection fails, nothing is planned.
    let s = only_d_plus("blocked.json");
    let report = run_pipeline(&s, None, &mut Quiet).unwrap();
    assert!(report.stopped.is_none(), "{report:?}");
    let route = report.stages.iter().find(|r| r.stage == "route").unwrap();
    assert_eq!(route.detail["tasks"], 1, "{:?}", route.detail);
    assert_eq!(route.detail["routed"], 0, "{:?}", route.detail);
    assert!(!s.output_plan.exists());
    assert!(report.refill.is_none());
    // The failure names the fills in the way and the option that lifts them.
    assert_eq!(report.warnings.len(), 1, "{:?}", report.warnings);
    let w = &report.warnings[0];
    assert!(
        w.starts_with("1 connection(s) were not routed")
            && w.contains("GND wall@F.Cu")
            && w.contains("GND wall@B.Cu")
            && !w.contains("In1.Cu")
            && w.contains("carve_zones"),
        "{w}"
    );

    // By name, by net, and with the old switch: each lets the route through.
    for (out, by) in [("named.json", 0), ("by_net.json", 1), ("all.json", 2)] {
        let mut s = only_d_plus(out);
        match by {
            0 => s.carve_zones = vec!["GND wall".to_string()],
            1 => s.carve_zone_nets = vec!["GND".to_string()],
            _ => s.zones_are_obstacles = false,
        }
        let report = run_pipeline(&s, None, &mut Quiet).unwrap();
        assert!(report.stopped.is_none(), "{out}: {report:?}");
        let route = report.stages.iter().find(|r| r.stage == "route").unwrap();
        assert_eq!(route.detail["routed"], 1, "{out}: {:?}", route.detail);
        assert_eq!(report.check.as_ref().unwrap().violations, 0, "{out}");
        assert_eq!(
            plan_nets(&s.output_plan),
            BTreeSet::from(["D+".to_string()])
        );
        let refill = report
            .refill
            .as_ref()
            .expect("zones were declared crossable");
        assert!(refill.required, "{out}: {refill:?}");
        assert!(!refill.crossed_zones.is_empty(), "{out}: {refill:?}");
        assert!(refill
            .crossed_zones
            .iter()
            .all(|c| c.zone.net == "GND" && c.tracks + c.vias > 0));
        let wall = |c: &planes::CrossedZone| c.zone.zone.as_deref() == Some("GND wall");
        assert!(refill.crossed_zones.iter().any(|c| wall(c) && c.tracks > 0));
        // By name only the wall is declared. By net or with the switch the
        // In1 plane is too: no track runs there, but a via through it needs
        // its antipad from the same refill.
        assert_eq!(
            refill.crossable_zones.len(),
            if by == 0 { 3 } else { 4 },
            "{out}"
        );
        assert!(refill
            .crossed_zones
            .iter()
            .all(|c| wall(c) || (by != 0 && c.zone.layer == "In1.Cu" && c.tracks == 0)));
        assert!(
            report
                .warnings
                .iter()
                .any(|w| w.starts_with("Refill required") && w.contains("GND wall@")),
            "{out}: {:?}",
            report.warnings
        );
    }
}

#[test]
fn zone_options_naming_nothing_on_the_board_stop_the_run_as_an_input_error() {
    let f = fixture();
    let stopped = |edit: &dyn Fn(&mut NativeJobSpec)| {
        let mut s = spec(&f.root, "bad_zone.json", CopperSource::Native);
        edit(&mut s);
        let report = run_pipeline(&s, None, &mut Quiet).unwrap();
        assert_eq!(report.stop_kind, Some(StopKind::InputError), "{report:?}");
        assert!(!s.output_plan.exists());
        report.stopped.unwrap()
    };
    // A rule area is never crossable.
    let why = stopped(&|s| s.carve_zones = vec!["no_vias".to_string()]);
    assert!(
        why.contains("carve_zones") && why.contains("rule area"),
        "{why}"
    );
    let why = stopped(&|s| s.carve_zones = vec!["GND plane".to_string(), "Nope".to_string()]);
    assert!(
        why.contains("no copper zone of the board is named 'Nope'"),
        "{why}"
    );
    // SIG is a net, but owns no zone.
    let why = stopped(&|s| s.carve_zone_nets = vec!["SIG".to_string()]);
    assert!(
        why.contains("carve_zone_nets") && why.contains("'GND'"),
        "{why}"
    );
    let why = stopped(&|s| s.plane_nets_as_tracks = vec!["SIG".to_string()]);
    assert!(
        why.contains("plane_nets_as_tracks") && why.contains("'SIG'"),
        "{why}"
    );
    // Taking existing copper routes nothing, so there is nothing to name.
    let mut s = spec(&f.root, "taken.json", CopperSource::BoardUnlocked);
    s.plane_nets_as_tracks = vec!["GND".to_string()];
    let report = run_pipeline(&s, None, &mut Quiet).unwrap();
    assert_eq!(report.stop_kind, Some(StopKind::InputError));
    assert!(report.stopped.unwrap().contains("native routing"));
}

#[test]
fn old_specs_without_the_zone_options_still_read() {
    let f = fixture();
    let s = spec(&f.root, "old.json", CopperSource::Native);
    let mut v = serde_json::to_value(&s).unwrap();
    // The options are not written when unset, so an old job's spec is the
    // same bytes and resumes.
    for key in ["carve_zones", "carve_zone_nets", "plane_nets_as_tracks"] {
        assert!(v.get(key).is_none(), "{key} in {v}");
    }
    v.as_object_mut().unwrap().remove("zones_are_obstacles");
    let back: NativeJobSpec = serde_json::from_value(v).unwrap();
    assert_eq!(back, s);
}

#[test]
fn a_filled_plane_in_separate_pieces_is_warned_about_even_when_every_pad_is_joined() {
    use super::planes::{warnings, PlaneNetSummary, UnroutedConnection};
    let split = PlaneNetSummary {
        net: "GND".to_string(),
        fill: "filled".to_string(),
        handling: "left_to_plane".to_string(),
        pads: 4,
        connections_left_unrouted: 1,
        unrouted_connections: vec![UnroutedConnection {
            from: "C1.2".to_string(),
            to: "C2.2".to_string(),
        }],
        ..PlaneNetSummary::default()
    };
    let w = warnings(std::slice::from_ref(&split), None);
    assert_eq!(w.len(), 1, "{w:?}");
    assert!(
        w[0].contains("GND")
            && w[0].contains("1 connection(s)")
            && w[0].contains("unrouted_connections")
            && w[0].contains("plane_nets_as_tracks"),
        "{}",
        w[0]
    );
    // Whole, the same net is nothing to warn about.
    let whole = PlaneNetSummary {
        connections_left_unrouted: 0,
        unrouted_connections: Vec::new(),
        ..split
    };
    assert!(warnings(&[whole], None).is_empty());
}
