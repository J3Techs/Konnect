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
        rip_up: Default::default(),
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
    for key in ["allowances", "net_classes", "fanout_plan", "rip_up"] {
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

// --- Track width overrides ---------------------------------------------------

/// A 2-layer board with an unrouted `+5V` (class Power, 0.8 mm) on J1.1,
/// R1.1 and R2.1, 8 mm apart in a row, and `SIG` on R1.2 and R2.2.
const FEED_BOARD: &str = r#"(kicad_pcb
	(version 20260206)
	(generator "pcbnew")
	(general
		(thickness 1.6)
	)
	(layers
		(0 "F.Cu" signal)
		(2 "B.Cu" signal)
		(25 "Edge.Cuts" user)
	)
	(footprint "TestPoint"
		(layer "F.Cu")
		(uuid "00000000-0000-0000-0000-000000000101")
		(at 5 5)
		(property "Reference" "J1" (at 0 -2 0) (layer "F.SilkS"))
		(pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu" "F.Mask") (net "+5V"))
	)
	(footprint "R_0603"
		(layer "F.Cu")
		(uuid "00000000-0000-0000-0000-000000000102")
		(at 13 5)
		(property "Reference" "R1" (at 0 -2 0) (layer "F.SilkS"))
		(pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu" "F.Mask") (net "+5V"))
		(pad "2" smd rect (at 0 4) (size 1 1) (layers "F.Cu" "F.Mask") (net "SIG"))
	)
	(footprint "R_0603"
		(layer "F.Cu")
		(uuid "00000000-0000-0000-0000-000000000103")
		(at 21 5)
		(property "Reference" "R2" (at 0 -2 0) (layer "F.SilkS"))
		(pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu" "F.Mask") (net "+5V"))
		(pad "2" smd rect (at 0 4) (size 1 1) (layers "F.Cu" "F.Mask") (net "SIG"))
	)
	(gr_rect
		(start 0 0)
		(end 26 14)
		(stroke (width 0.05) (type default))
		(fill no)
		(layer "Edge.Cuts")
	)
)
"#;

const FEED_PRO: &str = r#"{
    "board": { "design_settings": { "rules": {
        "min_clearance": 0.1, "min_track_width": 0.1, "min_via_diameter": 0.4,
        "min_via_annular_width": 0.1, "min_through_hole_diameter": 0.2,
        "min_copper_edge_clearance": 0.3, "min_hole_clearance": 0.2, "min_hole_to_hole": 0.25
    } } },
    "net_settings": {
        "classes": [
            { "name": "Default", "clearance": 0.2, "track_width": 0.2,
              "via_diameter": 0.6, "via_drill": 0.3 },
            { "name": "Power", "clearance": 0.2, "track_width": 0.8 }
        ],
        "netclass_patterns": [ { "pattern": "+5V", "netclass": "Power" } ]
    }
}"#;

fn feed_fixture() -> Fixture {
    let dir = tempfile::tempdir().unwrap();
    let root = dir.path().to_path_buf();
    std::fs::write(root.join("demo.kicad_pcb"), FEED_BOARD).unwrap();
    std::fs::write(root.join("demo.kicad_pro"), FEED_PRO).unwrap();
    Fixture { _dir: dir, root }
}

/// A native spec on the feed board that routes every class, with
/// `track_widths`.
fn feed_spec(root: &Path, out: &str, track_widths: Value) -> NativeJobSpec {
    let mut s = spec(root, out, CopperSource::Native);
    s.allowances = serde_json::from_value(json!({
        "exclude_classes": [], "track_widths": track_widths
    }))
    .unwrap();
    s
}

/// The distinct widths (mm) of the plan's tracks of `net` lying wholly
/// within `x` (mm), sorted.
fn plan_widths(plan: &Path, net: &str, x: (f64, f64)) -> Vec<f64> {
    let plan: RoutePlan = read_json(plan).unwrap();
    let mut w: Vec<f64> = plan
        .tracks
        .iter()
        .filter(|t| t.net == net)
        .filter(|t| [t.x1, t.x2].iter().all(|v| (x.0..=x.1).contains(v)))
        .map(|t| t.width)
        .collect();
    w.sort_by(f64::total_cmp);
    w.dedup();
    w
}

#[test]
fn a_net_s_track_width_reaches_the_plan_and_the_report_says_it_overrides_the_class() {
    let f = feed_fixture();
    // Without the option: the class width, and nothing about widths reported.
    let plain = feed_spec(&f.root, "plain.json", json!([]));
    let report = run_pipeline(&plain, None, &mut Quiet).unwrap();
    assert!(report.stopped.is_none(), "{report:?}");
    assert_eq!(
        plan_widths(&plain.output_plan, "+5V", (0.0, 26.0)),
        vec![0.8]
    );
    assert_eq!(
        plan_widths(&plain.output_plan, "SIG", (0.0, 26.0)),
        vec![0.2]
    );
    let allowances = serde_json::to_value(report.allowances.as_ref().unwrap()).unwrap();
    assert!(allowances.get("track_widths").is_none(), "{allowances}");
    let spec_json = serde_json::to_value(&plain).unwrap();
    assert!(
        spec_json["allowances"].get("track_widths").is_none(),
        "{spec_json}"
    );

    // The whole net narrower than its class, one connection narrower still.
    let hinted = feed_spec(
        &f.root,
        "hinted.json",
        json!([
            { "net": "+5V", "width_mm": 0.3 },
            { "net": "+5V", "from": "J1.1", "to": "R1.1", "width_mm": 0.15 }
        ]),
    );
    let report = run_pipeline(&hinted, None, &mut Quiet).unwrap();
    assert!(report.stopped.is_none(), "{report:?}");
    // S6 and the clear stage keep the widths; S7 accepts them (only the
    // minimum width is a rule).
    let check = report.check.as_ref().unwrap();
    assert_eq!(check.violations, 0, "{check:?}");
    assert_eq!(
        plan_widths(&hinted.output_plan, "+5V", (4.0, 13.6)),
        vec![0.15]
    );
    assert_eq!(
        plan_widths(&hinted.output_plan, "+5V", (12.4, 22.0)),
        vec![0.3]
    );
    assert_eq!(
        plan_widths(&hinted.output_plan, "SIG", (0.0, 26.0)),
        vec![0.2]
    );
    let widths = &report.allowances.as_ref().unwrap().track_widths;
    assert_eq!(
        serde_json::to_value(widths).unwrap(),
        json!([
            { "net": "+5V", "width_nm": 300_000, "overrides_class": "Power",
              "class_width_nm": 800_000, "connections": 1, "fanout_stubs": 0 },
            { "net": "+5V", "from": "J1.1", "to": "R1.1", "width_nm": 150_000,
              "overrides_class": "Power", "class_width_nm": 800_000, "connections": 1 }
        ])
    );
}

#[test]
fn a_track_width_the_board_cannot_take_stops_the_run_naming_the_entry() {
    let f = feed_fixture();
    for (entries, needle) in [
        (
            json!([{ "net": "SIG", "width_mm": 0.25 }, { "net": "+5V", "width_mm": 0.05 }]),
            "track_widths[1]: 0.05 mm is below the minimum track width",
        ),
        (
            json!([{ "net": "+12V", "width_mm": 0.3 }]),
            "track_widths[0]: no net named '+12V'",
        ),
        (
            json!([{ "net": "+5V", "from": "J1.1", "to": "R9.1", "width_mm": 0.3 }]),
            "track_widths[0]: 'R9'",
        ),
    ] {
        let s = feed_spec(&f.root, "bad.json", entries);
        let report = run_pipeline(&s, None, &mut Quiet).unwrap();
        assert_eq!(report.stop_kind, Some(StopKind::InputError));
        let why = report.stopped.as_deref().unwrap();
        assert!(why.contains(needle), "{why}");
        assert!(!s.output_plan.exists());
        assert!(matches!(
            s.check_allowances().unwrap(),
            Err(AllowanceError::TrackWidth { .. })
        ));
    }
    // Under the default class policy the Power class is not routed at all,
    // so a width for one of its nets would be silently unused.
    let mut s = feed_spec(
        &f.root,
        "policy.json",
        json!([{ "net": "+5V", "width_mm": 0.3 }]),
    );
    s.allowances.exclude_classes = None;
    let report = run_pipeline(&s, None, &mut Quiet).unwrap();
    let why = report.stopped.as_deref().unwrap();
    assert!(
        why.contains("track_widths[0]") && why.contains("exclude_classes"),
        "{why}"
    );
}

#[test]
fn a_net_s_track_width_reaches_the_fanout_stub_of_a_net_left_to_its_plane() {
    let f = fixture();
    // GND owns the In1 plane, so the run leaves it to the plane: no task,
    // only the fanout's via for R1.2, on a stub drawn at the class width.
    let plain = spec(&f.root, "plain.json", CopperSource::Native);
    let report = run_pipeline(&plain, None, &mut Quiet).unwrap();
    assert!(report.stopped.is_none(), "{report:?}");
    assert_eq!(report.plane_nets[0].handling, "left_to_plane");
    assert_eq!(
        plan_widths(&plain.output_plan, "GND", (0.0, 100.0)),
        vec![0.2]
    );

    // A net entry is the rule table's width, so the stub follows it even
    // though no connection of the net is routed; the report counts the stub.
    let mut s = spec(&f.root, "hinted.json", CopperSource::Native);
    s.allowances =
        serde_json::from_value(json!({ "track_widths": [{ "net": "GND", "width_mm": 0.3 }] }))
            .unwrap();
    let report = run_pipeline(&s, None, &mut Quiet).unwrap();
    assert!(report.stopped.is_none(), "{report:?}");
    let gnd = &report.plane_nets[0];
    assert_eq!(gnd.handling, "left_to_plane");
    assert_eq!(gnd.connected_by_fanout.pads, vec!["R1.2".to_string()]);
    assert_eq!(plan_widths(&s.output_plan, "GND", (0.0, 100.0)), vec![0.3]);
    let widths = serde_json::to_value(&report.allowances.as_ref().unwrap().track_widths).unwrap();
    assert_eq!(widths[0]["net"], "GND", "{widths}");
    assert_eq!(widths[0]["width_nm"], 300_000, "{widths}");
    assert_eq!(widths[0]["connections"], 0, "{widths}");
    assert_eq!(widths[0]["fanout_stubs"], 1, "{widths}");
    // Left to its plane and joined by the stub: nothing of GND is open.
    let open = report.unrouted.as_ref().unwrap();
    assert!(open.connections.iter().all(|c| c.net != "GND"), "{open:?}");
}

/// Net A's pad TP1 inside a ring of net B drawn on both copper layers
/// (so no via gets out), its other pad TP2 outside. B's two pads sit on
/// opposite corners of the ring.
fn sealed_pin_board(locked: bool) -> String {
    let pad = |reference: &str, n: u32, x: f64, y: f64, net: &str| {
        format!(
            r#"	(footprint "TP"
		(layer "F.Cu")
		(uuid "00000000-0000-0000-0001-{n:012}")
		(at {x} {y})
		(property "Reference" "{reference}" (at 0 -1 0) (layer "F.SilkS"))
		(pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu") (net "{net}")
			(uuid "00000000-0000-0000-0002-{n:012}"))
	)
"#
        )
    };
    let lock = if locked { "(locked yes) " } else { "" };
    let mut segments = String::new();
    let corners = [(3.0, 4.0), (7.0, 4.0), (7.0, 8.0), (3.0, 8.0)];
    for (l, layer) in ["F.Cu", "B.Cu"].iter().enumerate() {
        for i in 0..4 {
            let (a, b) = (corners[i], corners[(i + 1) % 4]);
            segments.push_str(&format!(
                "\t(segment (start {} {}) (end {} {}) (width 0.2) {lock}(layer \"{layer}\") \
                 (net \"B\") (uuid \"00000000-0000-0000-0003-{:012}\"))\n",
                a.0,
                a.1,
                b.0,
                b.1,
                l * 4 + i
            ));
        }
    }
    format!(
        r#"(kicad_pcb
	(version 20260206)
	(generator "pcbnew")
	(general (thickness 1.6))
	(layers
		(0 "F.Cu" signal)
		(2 "B.Cu" signal)
		(25 "Edge.Cuts" user)
	)
{}{}{}{}	(gr_rect (start 0 0) (end 20 12) (stroke (width 0.05) (type default)) (fill no) (layer "Edge.Cuts"))
{segments})
"#,
        pad("TP1", 1, 5.0, 6.0, "A"),
        pad("TP2", 2, 15.0, 6.0, "A"),
        pad("TP3", 3, 3.0, 4.0, "B"),
        pad("TP4", 4, 7.0, 8.0, "B"),
    )
}

fn sealed_fixture(locked: bool) -> Fixture {
    let f = fixture();
    std::fs::write(f.root.join("demo.kicad_pcb"), sealed_pin_board(locked)).unwrap();
    f
}

/// Open connections of the saved board once `plan` is applied to it, the
/// UUIDs in `delete` deleted first.
fn open_after_apply(s: &NativeJobSpec, plan: Option<&Path>, delete: &[String]) -> usize {
    let (mut board, _, _) = s.load_inputs().unwrap();
    board.tracks.retain(|t| {
        t.uuid
            .as_ref()
            .is_none_or(|u| !delete.iter().any(|d| d == u))
    });
    for (i, t) in board.tracks.iter_mut().enumerate() {
        t.id = konnect_route::api::TrackId(i as u32);
    }
    if let Some(path) = plan {
        let plan: RoutePlan =
            serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
        plan.to_routed(&board, Generation(0))
            .unwrap()
            .freeze_into(&mut board);
    }
    ConnectivityReport::build(&board, &konnect_geom::ArcPolicy::default()).connection_count()
}

#[test]
fn a_rip_up_unlocked_job_routes_the_sealed_pin_and_lists_the_board_copper_its_plan_replaces() {
    let f = sealed_fixture(false);
    let mut s = spec(&f.root, "ripped.json", CopperSource::Native);
    s.rip_up = RipUp::Unlocked;
    assert_eq!(open_after_apply(&s, None, &[]), 1, "A is open on the board");
    let dir = prepare(&s, false).unwrap();
    let status = run_job_dir(&dir).unwrap();
    assert_eq!(status.state, "COMPLETED", "{status:?}");
    let report: PipelineReport = read_json(&dir.join(REPORT_FILE)).unwrap();
    let rip = report.rip_up.clone().expect("the rip-up stage ran");
    assert_eq!((rip.open_before, rip.open_after, rip.rounds), (1, 0, 1));
    assert_eq!(rip.rerouted_nets, ["B"]);
    assert!(rip.still_open.is_empty() && !rip.restored);
    // Every unlocked track of B is listed with its UUID, net and layer.
    assert_eq!(rip.ripped.len(), 8, "{:?}", rip.ripped);
    assert!(rip
        .ripped
        .iter()
        .all(|i| i.net == "B" && !i.reason.is_empty()));
    for layer in ["F.Cu", "B.Cu"] {
        assert_eq!(rip.ripped.iter().filter(|i| i.layer == layer).count(), 4);
    }
    // The deletions file names exactly those, in the shape delete_routing,
    // the replaces_file arguments and apply_route_plan read.
    let replaces = replaces_path_for(&s.output_plan);
    assert_eq!(report.replaces_path.as_deref(), Some(replaces.as_path()));
    let body: Value = read_json(&replaces).unwrap();
    let mut listed: Vec<String> = rip.ripped.iter().map(|i| i.uuid.clone()).collect();
    listed.sort();
    assert_eq!(body["delete_uuids"], json!(listed));
    assert_eq!(body["ripped"].as_array().unwrap().len(), 8);
    // The plan routes both nets and is clean against the board without the
    // ring; applied with the deletions, nothing is left open.
    let plan: RoutePlan =
        serde_json::from_str(&std::fs::read_to_string(&s.output_plan).unwrap()).unwrap();
    for net in ["A", "B"] {
        assert!(
            plan.tracks.iter().any(|t| t.net == net),
            "no {net} in the plan"
        );
    }
    assert_eq!(report.check.as_ref().unwrap().violations, 0);
    assert_eq!(open_after_apply(&s, Some(&s.output_plan), &listed), 0);
    // The guard after the clear stage recorded what was left open.
    let guard = report.stages.iter().find(|r| r.stage == "rip_up").unwrap();
    assert_eq!(guard.state, "done");
    assert_eq!(guard.detail["open_after_optimize_and_clear"], 0);
    // The tools' view carries the report and the deletions file.
    let view = status_of(&s.output_plan);
    assert_eq!(
        view["report"]["rip_up"]["ripped"].as_array().unwrap().len(),
        8
    );
    assert_eq!(view["report"]["replaces_file"], json!(replaces));
    // A second run for the same plan name would write the same deletions
    // file: refused, like the plan itself.
    std::fs::remove_file(&s.output_plan).unwrap();
    std::fs::remove_dir_all(&dir).unwrap();
    let again = prepare(&s, false).unwrap_err().to_string();
    assert!(again.contains("replaces.json"), "{again}");
}

#[test]
fn by_default_a_job_leaves_board_copper_alone_and_says_what_blocks_the_pin() {
    let f = sealed_fixture(false);
    let s = spec(&f.root, "own.json", CopperSource::Native);
    let dir = prepare(&s, false).unwrap();
    assert_eq!(run_job_dir(&dir).unwrap().state, "COMPLETED");
    let report: PipelineReport = read_json(&dir.join(REPORT_FILE)).unwrap();
    let rip = report.rip_up.expect("the rip-up stage ran");
    assert!(rip.ripped.is_empty() && rip.attempts.is_empty());
    assert_eq!((rip.open_before, rip.open_after), (1, 1));
    assert_eq!(rip.still_open.len(), 1);
    let open = &rip.still_open[0];
    assert_eq!((open.net.as_str(), open.from.as_str()), ("A", "TP1.1"));
    assert_eq!(open.blocked_by[0].net, "B");
    assert_eq!(open.blocked_by[0].reason, "board_copper_not_requested");
    assert!(report.replaces_path.is_none());
    assert!(!replaces_path_for(&s.output_plan).exists());
    assert!(!s.output_plan.exists(), "nothing was routed, so no plan");
}

#[test]
fn a_job_never_rips_locked_copper_and_reports_it_as_the_blocker() {
    let f = sealed_fixture(true);
    let mut s = spec(&f.root, "locked.json", CopperSource::Native);
    s.rip_up = RipUp::Unlocked;
    let dir = prepare(&s, false).unwrap();
    assert_eq!(run_job_dir(&dir).unwrap().state, "COMPLETED");
    let report: PipelineReport = read_json(&dir.join(REPORT_FILE)).unwrap();
    let rip = report.rip_up.expect("the rip-up stage ran");
    assert!(rip.ripped.is_empty() && rip.attempts.is_empty());
    assert_eq!(rip.still_open.len(), 1);
    assert_eq!(rip.still_open[0].blocked_by[0].reason, "locked");
    assert!(!rip.still_open[0].blocked_by[0].uuids.is_empty());
    assert!(!replaces_path_for(&s.output_plan).exists());
}

#[test]
fn rip_up_is_refused_for_copper_that_is_not_routed_natively() {
    let f = fixture();
    let mut s = spec(&f.root, "board.json", CopperSource::BoardUnlocked);
    s.rip_up = RipUp::Unlocked;
    let e = prepare(&s, false).unwrap_err().to_string();
    assert!(e.contains("rip_up"), "{e}");
}

/// [`sealed_pin_board`] with the ring unlocked, and a second sealed pin to
/// its right: net C's pad TP5 inside a locked ring of net D, its other pad
/// TP6 outside. Nothing may open that one.
fn two_sealed_pins_board() -> String {
    let pad = |reference: &str, n: u32, x: f64, y: f64, net: &str| {
        format!(
            r#"	(footprint "TP"
		(layer "F.Cu")
		(uuid "00000000-0000-0000-0001-{n:012}")
		(at {x} {y})
		(property "Reference" "{reference}" (at 0 -1 0) (layer "F.SilkS"))
		(pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu") (net "{net}")
			(uuid "00000000-0000-0000-0002-{n:012}"))
	)
"#
        )
    };
    let mut extra = String::new();
    for (reference, n, x, y, net) in [
        ("TP5", 5, 23.0, 6.0, "C"),
        ("TP6", 6, 28.0, 6.0, "C"),
        ("TP7", 7, 21.0, 4.0, "D"),
        ("TP8", 8, 25.0, 8.0, "D"),
    ] {
        extra.push_str(&pad(reference, n, x, y, net));
    }
    let corners = [(21.0, 4.0), (25.0, 4.0), (25.0, 8.0), (21.0, 8.0)];
    for (l, layer) in ["F.Cu", "B.Cu"].iter().enumerate() {
        for i in 0..4 {
            let (a, b) = (corners[i], corners[(i + 1) % 4]);
            extra.push_str(&format!(
                "\t(segment (start {} {}) (end {} {}) (width 0.2) (locked yes) (layer \"{layer}\") \
                 (net \"D\") (uuid \"00000000-0000-0000-0004-{:012}\"))\n",
                a.0,
                a.1,
                b.0,
                b.1,
                l * 4 + i
            ));
        }
    }
    let board = sealed_pin_board(false).replace("(end 20 12)", "(end 30 12)");
    let end = board.rfind(')').unwrap();
    format!("{}{extra}{}", &board[..end], &board[end..])
}

#[test]
fn after_a_kept_rip_the_report_lists_exactly_what_is_open_with_its_blockers_and_lints_the_plan() {
    let f = fixture();
    std::fs::write(f.root.join("demo.kicad_pcb"), two_sealed_pins_board()).unwrap();
    let mut s = spec(&f.root, "two.json", CopperSource::Native);
    s.rip_up = RipUp::Unlocked;
    // B is ripped and routed again: at the width asked for it.
    s.allowances =
        serde_json::from_value(json!({ "track_widths": [{ "net": "B", "width_mm": 0.3 }] }))
            .unwrap();
    assert_eq!(open_after_apply(&s, None, &[]), 2, "A and C are open");
    let dir = prepare(&s, false).unwrap();
    let status = run_job_dir(&dir).unwrap();
    assert_eq!(status.state, "COMPLETED", "{status:?}");
    let report: PipelineReport = read_json(&dir.join(REPORT_FILE)).unwrap();

    // The stage opened A's ring and could not open C's.
    let rip = report.rip_up.clone().expect("the rip-up stage ran");
    assert_eq!((rip.open_before, rip.open_after), (2, 1), "{rip:?}");
    assert_eq!(rip.rerouted_nets, ["B"]);
    assert_eq!(rip.ripped.len(), 8, "{:?}", rip.ripped);
    assert!(!rip.restored);
    assert_eq!(rip.still_open.len(), 1, "{:?}", rip.still_open);
    let stuck = &rip.still_open[0];
    assert_eq!(
        (stuck.net.as_str(), stuck.from.as_str(), stuck.to.as_str()),
        ("C", "TP5.1", "TP6.1")
    );
    assert_eq!(stuck.blocked_by[0].net, "D");
    assert_eq!(stuck.blocked_by[0].reason, "locked");

    // `unrouted` lists that one connection and no other: A and B are
    // joined by the plan, on the board without the copper it replaces.
    let deleted: Vec<String> = rip.ripped.iter().map(|i| i.uuid.clone()).collect();
    let open = report.unrouted.as_ref().expect("the run reached the check");
    assert_eq!(open.total, 1, "{open:?}");
    assert_eq!(
        open.total,
        open_after_apply(&s, Some(&s.output_plan), &deleted)
    );
    let c = &open.connections[0];
    assert_eq!(c.net, "C");
    assert_eq!(
        (c.from.label(), c.to.label()),
        ("TP5.1".to_string(), "TP6.1".to_string())
    );
    // It is the route stage's own failure of that pad pair, and it carries
    // what the rip-up stage found in its way: the two accounts agree.
    assert_eq!(open.route_failures.len(), 1, "{:?}", open.route_failures);
    assert_eq!(open.route_failures[0].net, "C");
    assert_eq!(c.reason, open.route_failures[0].reason);
    assert_eq!(open.by_reason, BTreeMap::from([(c.reason.clone(), 1)]));
    assert_eq!(c.blocked_by.as_ref(), Some(&stuck.blocked_by));
    let view = status_of(&s.output_plan);
    assert_eq!(
        view["report"]["unrouted"]["connections"][0]["blocked_by"][0]["reason"], "locked",
        "{view}"
    );

    // The plan a rip-up run writes is linted like any other, and the
    // apply tool agrees with the report.
    let lint = report.plan_lint.as_ref().expect("a written plan is linted");
    assert_eq!(lint.path, s.output_plan);
    assert_eq!(lint.total, strict_apply_defects(&s.output_plan));
    assert!(lint.passes_strict, "{lint:?}");
    assert_eq!(view["report"]["plan_lint"]["passes_strict"], true);
    assert_eq!(report.check.as_ref().unwrap().violations, 0);

    // The net routed again kept its width; A, with no entry, its class's.
    assert_eq!(plan_widths(&s.output_plan, "B", (0.0, 100.0)), vec![0.3]);
    assert_eq!(plan_widths(&s.output_plan, "A", (0.0, 100.0)), vec![0.2]);
    let widths = serde_json::to_value(&report.allowances.as_ref().unwrap().track_widths).unwrap();
    assert_eq!(widths[0]["net"], "B", "{widths}");
    assert_eq!(widths[0]["connections"], 1, "{widths}");
}

#[test]
fn a_run_without_a_kept_rip_carries_the_blocker_on_the_connection_it_leaves_open() {
    // Board copper is not asked for: nothing is ripped, and the open
    // connection says in one place why and what is in its way.
    let f = sealed_fixture(false);
    let s = spec(&f.root, "own2.json", CopperSource::Native);
    let report = run_pipeline(&s, None, &mut Quiet).unwrap();
    let rip = report.rip_up.as_ref().expect("the rip-up stage ran");
    let open = report.unrouted.as_ref().unwrap();
    assert_eq!(open.total, 1, "{open:?}");
    let c = &open.connections[0];
    assert_eq!(c.net, "A");
    let by = c.blocked_by.as_ref().expect("the stage's blockers");
    assert_eq!(by, &rip.still_open[0].blocked_by);
    assert_eq!(
        (by[0].net.as_str(), by[0].reason.as_str()),
        ("B", "board_copper_not_requested")
    );
}
