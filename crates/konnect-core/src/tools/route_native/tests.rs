use super::*;
use crate::mcp::protocol::ToolContent;
use crate::router::ToolRouter;
use crate::tools::ServerConfig;
use std::sync::Arc;

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

fn ctx() -> ToolContext {
    ToolContext::new(
        ServerConfig {
            kicad_cli: String::new(),
            kicad_binary: String::new(),
            ipc_address: String::new(),
            project_dir: None,
            jlcpcb_db_path: None,
            auto_load_toolsets: false,
            eager_toolsets: false,
        },
        Arc::new(ToolRouter::new()),
    )
}

fn body(r: &CallToolResult) -> Value {
    match &r.content[0] {
        ToolContent::Text { text } => serde_json::from_str(text).unwrap_or(json!({ "text": text })),
        _ => panic!("expected text"),
    }
}

fn text(r: &CallToolResult) -> String {
    match &r.content[0] {
        ToolContent::Text { text } => text.clone(),
        _ => String::new(),
    }
}

struct Files {
    _dir: tempfile::TempDir,
    root: PathBuf,
}

impl Files {
    fn board(&self) -> String {
        self.root.join("demo.kicad_pcb").display().to_string()
    }
    fn path(&self, name: &str) -> String {
        self.root.join(name).display().to_string()
    }
}

fn files() -> Files {
    let dir = tempfile::tempdir().unwrap();
    let root = dir.path().to_path_buf();
    std::fs::write(
        root.join("demo.kicad_pcb"),
        konnect_route::synthetic::MINIMAL_KICAD10_BOARD,
    )
    .unwrap();
    std::fs::write(root.join("demo.kicad_pro"), PRO).unwrap();
    Files { _dir: dir, root }
}

fn write_plan(f: &Files, name: &str, tracks: Vec<PlanTrack>) -> String {
    let plan = RoutePlan {
        tracks,
        vias: vec![],
    };
    std::fs::write(f.root.join(name), plan.to_routing_file_json()).unwrap();
    f.path(name)
}

fn t(net: &str, layer: &str, a: (f64, f64), b: (f64, f64)) -> PlanTrack {
    PlanTrack {
        net: net.into(),
        layer: layer.into(),
        width: 0.2,
        x1: a.0,
        y1: a.1,
        x2: b.0,
        y2: b.1,
    }
}

#[test]
fn every_draft_tool_has_a_valid_schema_and_a_unique_name() {
    let defs = tools();
    let mut names: Vec<&str> = defs.iter().map(|d| d.name).collect();
    assert_eq!(names.len(), 8);
    names.sort();
    names.dedup();
    assert_eq!(names.len(), 8);
}

#[tokio::test]
async fn connectivity_report_counts_the_saved_copper() {
    let f = files();
    let r = handle_connectivity_report(&json!({ "board": f.board() }), &ctx())
        .await
        .unwrap();
    assert!(!r.is_error, "{}", text(&r));
    let b = body(&r);
    assert_eq!(b["source"], "saved_file");
    assert!(b["connections_needed"].as_u64().is_some());
    assert!(b["nets"].as_array().unwrap().len() <= 50);

    let r = handle_connectivity_report(&json!({ "board": f.path("nope.kicad_pcb") }), &ctx())
        .await
        .unwrap();
    assert!(r.is_error);
    let r = handle_connectivity_report(&json!({ "board": f.board(), "max_listed": -1 }), &ctx())
        .await
        .unwrap();
    assert!(r.is_error && text(&r).contains("max_listed"));
}

#[tokio::test]
async fn check_route_rules_finds_a_track_through_another_nets_pad() {
    let f = files();
    // R1 pad 2 (GND) sits at (20, 9.5) on F.Cu; run SIG straight across it.
    let bad = write_plan(
        &f,
        "bad.json",
        vec![t("SIG", "F.Cu", (18.0, 9.5), (22.0, 9.5))],
    );
    let r = handle_check_route_rules(&json!({ "board": f.board(), "routing_file": bad }), &ctx())
        .await
        .unwrap();
    assert!(!r.is_error, "{}", text(&r));
    let b = body(&r);
    assert_eq!(b["checker"], "interim_exact");
    assert_eq!(b["authoritative"], false);
    assert!(b["violations"].as_u64().unwrap() >= 1, "{b}");
    assert!(b["by_kind"]["short"].as_u64().unwrap_or(0) >= 1, "{b}");

    let good = write_plan(
        &f,
        "good.json",
        vec![t("GND", "B.Cu", (5.0, 5.0), (8.0, 5.0))],
    );
    let r = handle_check_route_rules(&json!({ "board": f.board(), "routing_file": good }), &ctx())
        .await
        .unwrap();
    assert_eq!(body(&r)["violations"], 0, "{}", text(&r));

    let unknown = write_plan(
        &f,
        "unknown.json",
        vec![t("NOPE", "B.Cu", (5.0, 5.0), (8.0, 5.0))],
    );
    let r = handle_check_route_rules(
        &json!({ "board": f.board(), "routing_file": unknown }),
        &ctx(),
    )
    .await
    .unwrap();
    assert!(
        r.is_error && text(&r).contains("does not have"),
        "{}",
        text(&r)
    );
}

#[tokio::test]
async fn unbuilt_stages_are_reported_as_unavailable_not_as_empty_results() {
    let f = files();
    let r =
        handle_audit_terminal_access(&json!({ "board": f.board(), "references": ["R1"] }), &ctx())
            .await
            .unwrap();
    assert!(r.is_error);
    assert!(text(&r).contains("missing evidence"), "{}", text(&r));
    let r = handle_audit_terminal_access(
        &json!({ "board": f.board(), "references": ["U99"] }),
        &ctx(),
    )
    .await
    .unwrap();
    assert!(r.is_error && text(&r).contains("U99"));

    let out = f.path("vias.json");
    let r = handle_plan_plane_vias(&json!({ "board": f.board(), "output_plan": out }), &ctx())
        .await
        .unwrap();
    assert!(
        r.is_error && text(&r).contains("cannot run yet"),
        "{}",
        text(&r)
    );
    assert!(!Path::new(&out).exists());
}

#[tokio::test]
async fn route_native_region_optimizes_a_plan_inside_its_region() {
    let f = files();
    let mut tracks = Vec::new();
    let (mut x, mut y) = (5.0f64, 5.0f64);
    while x < 7.0 - 1e-9 {
        let r = |v: f64| (v * 1e6).round() / 1e6;
        tracks.push(t("GND", "B.Cu", (r(x), r(y)), (r(x + 0.05), r(y))));
        tracks.push(t(
            "GND",
            "B.Cu",
            (r(x + 0.05), r(y)),
            (r(x + 0.05), r(y + 0.05)),
        ));
        x += 0.05;
        y += 0.05;
    }
    let seed = write_plan(&f, "seed.json", tracks);
    let out = f.path("region.json");
    let args = json!({
        "board": f.board(), "output_plan": out, "source": "plan", "source_plan": seed,
        "region": { "x1": 2.0, "y1": 2.0, "x2": 12.0, "y2": 12.0 }, "threads": 2
    });
    let r = handle_route_native_region(&args, &ctx()).await.unwrap();
    assert!(!r.is_error, "{}", text(&r));
    let b = body(&r);
    assert_eq!(b["work_tracks"], 80);
    assert!(b["plan_items"].as_u64().unwrap() <= 3, "{b}");
    assert_eq!(b["check"]["violations"], 0);
    assert!(Path::new(&out).is_file());

    // The same output again is a conflict; a huge region is a job.
    let r = handle_route_native_region(&args, &ctx()).await.unwrap();
    assert!(r.is_error && text(&r).contains("never replaced"));
    let mut big = args.clone();
    big["output_plan"] = json!(f.path("big.json"));
    big["region"] = json!({ "x1": 0.0, "y1": 0.0, "x2": 100.0, "y2": 10.0 });
    let r = handle_route_native_region(&big, &ctx()).await.unwrap();
    assert!(r.is_error && text(&r).contains("start_native_route_job"));
    // A missing region is refused, and native routing reports its stages.
    let mut none = args.clone();
    none["output_plan"] = json!(f.path("none.json"));
    none.as_object_mut().unwrap().remove("region");
    let r = handle_route_native_region(&none, &ctx()).await.unwrap();
    assert!(r.is_error && text(&r).contains("region"));
    // Native routing sees only the region's connections: none in this box,
    // so nothing is routed (and no unbuilt stage is reached) ...
    let mut native = args.clone();
    native["output_plan"] = json!(f.path("native.json"));
    native["source"] = json!("native");
    let r = handle_route_native_region(&native, &ctx()).await.unwrap();
    assert!(!r.is_error, "{}", text(&r));
    assert_eq!(body(&r)["work_tracks"], 0);
    assert!(body(&r)["output_plan"].is_null());
    // ... while a box holding the open connection reaches the stub.
    native["output_plan"] = json!(f.path("native2.json"));
    native["region"] = json!({ "x1": 15.0, "y1": 5.0, "x2": 40.0, "y2": 30.0 });
    let r = handle_route_native_region(&native, &ctx()).await.unwrap();
    assert!(
        r.is_error && text(&r).contains("not available"),
        "{}",
        text(&r)
    );
}

#[tokio::test]
async fn job_tools_refuse_bad_arguments_and_unknown_jobs() {
    let f = files();
    let r = handle_get_native_route_job(&json!({ "output_plan": f.path("none.json") }), &ctx())
        .await
        .unwrap();
    assert!(r.is_error && text(&r).contains("no native route job"));
    let r = handle_stop_native_route_job(&json!({ "output_plan": f.path("none.json") }), &ctx())
        .await
        .unwrap();
    assert!(r.is_error);
    let r = handle_start_native_route_job(
        &json!({ "board": f.board(), "output_plan": f.path("p.json"), "source": "plan" }),
        &ctx(),
    )
    .await
    .unwrap();
    assert!(r.is_error && text(&r).contains("source_plan"));
    let r = handle_start_native_route_job(
        &json!({ "board": f.board(), "output_plan": f.path("p.json"), "threads": 0 }),
        &ctx(),
    )
    .await
    .unwrap();
    assert!(r.is_error && text(&r).contains("threads"));
}
