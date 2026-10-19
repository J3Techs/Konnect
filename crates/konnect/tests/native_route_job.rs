//! Native route jobs run in runner processes of their own, several at once.
//!
//! Starts real `konnect native-route-runner` processes (this package's
//! binary) on a small synthetic board and waits for them. No KiCad is
//! involved: the runner reads saved files and writes a plan.

use konnect_core::native_route_job::{self, CopperSource, NativeJobSpec};
use konnect_route::plan::{PlanTrack, RoutePlan};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

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

/// A 50 µm staircase on B.Cu in open board.
fn seed(root: &Path) -> PathBuf {
    let r = |v: f64| (v * 1e6).round() / 1e6;
    let t = |a: (f64, f64), b: (f64, f64)| PlanTrack {
        net: "GND".into(),
        layer: "B.Cu".into(),
        width: 0.2,
        x1: r(a.0),
        y1: r(a.1),
        x2: r(b.0),
        y2: r(b.1),
    };
    let mut tracks = Vec::new();
    let (mut x, mut y) = (5.0f64, 5.0f64);
    while x < 8.0 - 1e-9 {
        tracks.push(t((x, y), (x + 0.05, y)));
        tracks.push(t((x + 0.05, y), (x + 0.05, y + 0.05)));
        x += 0.05;
        y += 0.05;
    }
    let path = root.join("seed.json");
    std::fs::write(
        &path,
        RoutePlan {
            tracks,
            vias: vec![],
        }
        .to_routing_file_json(),
    )
    .unwrap();
    path
}

#[tokio::test(flavor = "multi_thread")]
async fn several_runner_processes_route_at_once() {
    let dir = tempfile::tempdir().unwrap();
    let root = dir.path();
    std::fs::write(
        root.join("demo.kicad_pcb"),
        konnect_route::synthetic::MINIMAL_KICAD10_BOARD,
    )
    .unwrap();
    std::fs::write(root.join("demo.kicad_pro"), PRO).unwrap();
    let seed = seed(root);
    let exe = Path::new(env!("CARGO_BIN_EXE_konnect"));
    let specs: Vec<NativeJobSpec> = (0..3)
        .map(|i| NativeJobSpec {
            board: root.join("demo.kicad_pcb"),
            project: None,
            rules: None,
            output_plan: root.join(format!("plan{i}.json")),
            source: CopperSource::Plan { path: seed.clone() },
            region_mm: None,
            nets: None,
            threads: Some(2),
            optimize: true,
            zones_are_obstacles: true,
            clear_findings: None,
            time_limit_seconds: None,
        })
        .collect();
    let mut pids = Vec::new();
    for spec in &specs {
        let body = native_route_job::start_with(spec, false, exe)
            .await
            .expect("runner starts");
        assert_eq!(body["success"], true, "{body}");
        pids.push(body["runner_pid"].as_u64().unwrap());
    }
    pids.sort_unstable();
    pids.dedup();
    assert_eq!(pids.len(), 3, "one process per job");
    assert!(!pids.contains(&u64::from(std::process::id())));

    let waited = Instant::now();
    for spec in &specs {
        loop {
            let body = native_route_job::status(&spec.output_plan).unwrap();
            if body["finished"] == true {
                assert_eq!(body["state"], "COMPLETED", "{body}");
                assert_eq!(body["plan_exists"], true);
                break;
            }
            assert!(
                waited.elapsed() < Duration::from_secs(120),
                "runner did not finish: {body}"
            );
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    }
    let plans: Vec<String> = specs
        .iter()
        .map(|s| std::fs::read_to_string(&s.output_plan).unwrap())
        .collect();
    assert!(
        plans.windows(2).all(|w| w[0] == w[1]),
        "same input, same plan"
    );
    let plan: RoutePlan = serde_json::from_str(&plans[0]).unwrap();
    assert!(plan.item_count() <= 3, "the staircase was pulled tight");

    // A second start on a finished job is refused without resume.
    let again = native_route_job::start_with(&specs[0], false, exe).await;
    assert!(again.is_err());
}
