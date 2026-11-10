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
    assert_eq!(names.len(), 20);
    names.sort();
    names.dedup();
    assert_eq!(names.len(), 20);
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
async fn check_reference_plane_reads_the_saved_fill_and_refuses_bad_names() {
    let f = files();
    let call = |extra: Value| {
        let mut a = json!({ "board": f.board(), "plane": "F.Cu", "layers": ["B.Cu"] });
        a.as_object_mut()
            .unwrap()
            .extend(extra.as_object().unwrap().clone());
        a
    };
    // F.Cu has no fill: every checked B.Cu track is off and bare.
    let r = handle_check_reference_plane(&call(json!({ "class": "Default" })), &ctx())
        .await
        .unwrap();
    assert!(!r.is_error, "{}", text(&r));
    let b = body(&r);
    assert_eq!(b["source"], "saved_file");
    assert_eq!(b["plane_pieces"], 0);
    assert_eq!(b["tracks_off"], b["tracks_checked"]);
    assert_eq!(b["tracks_off_bare"], b["tracks_checked"]);
    assert_eq!(b["all_over_plane"], b["tracks_checked"] == 0);

    for (extra, field) in [
        (json!({}), "class"),
        (json!({ "class": "Nope" }), "class"),
        (json!({ "class": "Default", "plane": "In9.Cu" }), "plane"),
        (json!({ "class": "Default", "layers": ["X.Cu"] }), "layers"),
        (json!({ "nets": ["NO_SUCH_*"] }), "nets"),
        (json!({ "class": "Default", "margin_mm": -1 }), "margin_mm"),
    ] {
        let r = handle_check_reference_plane(&call(extra.clone()), &ctx())
            .await
            .unwrap();
        assert!(
            r.is_error && text(&r).contains(field),
            "{extra}: {}",
            text(&r)
        );
    }
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
    assert_eq!(b["checker"], "check_routed");
    assert_eq!(b["authoritative"], true);
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
async fn the_audit_and_plane_vias_run_on_the_saved_board() {
    let f = files();
    let r =
        handle_audit_terminal_access(&json!({ "board": f.board(), "references": ["R1"] }), &ctx())
            .await
            .unwrap();
    assert!(!r.is_error, "{}", text(&r));
    let b = body(&r);
    assert_eq!(b["source"], "saved_file");
    assert!(b["terminals"].as_u64().unwrap() >= 1, "{b}");
    assert!(b["unreachable"].as_u64().unwrap() <= b["terminals"].as_u64().unwrap());
    assert!(b["reachable_per_layer"].is_object(), "{b}");
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
    assert!(!r.is_error, "{}", text(&r));
    let b = body(&r);
    // Whatever was planned is on disk, and only then.
    let written = b["plan_written"].as_bool().unwrap();
    assert_eq!(Path::new(&out).exists(), written, "{b}");
    assert_eq!(written, b["sites"].as_u64().unwrap() > 0, "{b}");
    // The same output again is refused, never replaced.
    if written {
        let r = handle_plan_plane_vias(&json!({ "board": f.board(), "output_plan": out }), &ctx())
            .await
            .unwrap();
        assert!(r.is_error && text(&r).contains("never replaced"));
    }
    // carve_zones: any copper zone of the board (plane vias are through
    // vias); an unknown name or a rule area is refused before planning.
    let carved = f.path("carved.json");
    let r = handle_plan_plane_vias(
        &json!({ "board": f.board(), "output_plan": carved, "carve_zones": ["GND plane"] }),
        &ctx(),
    )
    .await
    .unwrap();
    assert!(!r.is_error, "{}", text(&r));
    let b = body(&r);
    assert_eq!(b["carve_zones"], json!(["GND plane"]));
    assert!(b["carving_sites"].is_array(), "{b}");
    // Every planned net names the plane its vias reach, and why.
    for p in b["planes"].as_array().unwrap() {
        assert!(p["net"].is_string() && p["layer"].is_string(), "{p}");
        assert!(p["chosen"].is_string() && p["why"].is_string(), "{p}");
    }
    // plane_layer / plane_zone must name filled copper of the board.
    for (extra, field) in [
        (json!({ "plane_layer": "Nope.Cu" }), "plane_layer"),
        (json!({ "plane_zone": "Nope" }), "plane_zone"),
        (json!({ "plane_zone": 3 }), "plane_zone"),
    ] {
        let mut a = json!({ "board": f.board(), "output_plan": f.path("y.json") });
        a.as_object_mut()
            .unwrap()
            .extend(extra.as_object().unwrap().clone());
        let r = handle_plan_plane_vias(&a, &ctx()).await.unwrap();
        assert!(r.is_error, "{a} accepted");
        assert!(text(&r).contains(field), "{}", text(&r));
        assert!(!Path::new(&f.path("y.json")).exists());
    }
    for names in [json!(["Nope"]), json!(["no_vias"])] {
        let r = handle_plan_plane_vias(
            &json!({ "board": f.board(), "output_plan": f.path("x.json"), "carve_zones": names }),
            &ctx(),
        )
        .await
        .unwrap();
        assert!(r.is_error, "{names} accepted");
        assert!(text(&r).contains("carve_zones"), "{}", text(&r));
        assert!(!Path::new(&f.path("x.json")).exists());
    }
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
    // so nothing is routed ...
    let mut native = args.clone();
    native["output_plan"] = json!(f.path("native.json"));
    native["source"] = json!("native");
    let r = handle_route_native_region(&native, &ctx()).await.unwrap();
    assert!(!r.is_error, "{}", text(&r));
    assert_eq!(body(&r)["work_tracks"], 0);
    assert!(body(&r)["output_plan"].is_null());
    // ... and the board's open connection is listed by pad as one the run
    // was not asked to route; max_listed bounds the list, not the totals.
    let open = body(&r)["unrouted"].clone();
    assert_eq!(open["total"], 1, "{open}");
    assert_eq!(open["by_reason"]["outside_scope"], 1, "{open}");
    let listed = &open["connections"][0];
    assert_eq!(listed["reason"], "outside_scope");
    for end in ["from", "to"] {
        assert_eq!(listed[end]["kind"], "pad", "{listed}");
        assert!(listed[end]["reference"].is_string() && listed[end]["pad"].is_string());
        assert!(listed[end]["position_mm"].is_array() && listed[end]["layers"].is_array());
    }
    assert!(open["reasons"]["outside_scope"].is_string());
    native["output_plan"] = json!(f.path("native1.json"));
    native["max_listed"] = json!(0);
    let r = handle_route_native_region(&native, &ctx()).await.unwrap();
    let open = body(&r)["unrouted"].clone();
    assert_eq!((&open["total"], &open["listed"]), (&json!(1), &json!(0)));
    assert!(open["note"].as_str().unwrap().contains("0 of 1"), "{open}");
    native.as_object_mut().unwrap().remove("max_listed");
    // ... while a box holding the open connection runs every stage and
    // plans the copper that joins it.
    native["output_plan"] = json!(f.path("native2.json"));
    native["region"] = json!({ "x1": 15.0, "y1": 5.0, "x2": 40.0, "y2": 30.0 });
    let r = handle_route_native_region(&native, &ctx()).await.unwrap();
    assert!(!r.is_error, "{}", text(&r));
    let b = body(&r);
    let route = b["stages"]
        .as_array()
        .unwrap()
        .iter()
        .find(|s| s["stage"] == "route")
        .unwrap()
        .clone();
    assert_eq!(route["state"], "done", "{b}");
    assert_eq!(route["detail"]["connections_after"], 0, "{b}");
    assert_eq!(b["unrouted"]["total"], 0, "{b}");
    assert_eq!(b["unrouted"]["connections"], json!([]), "{b}");
    assert_eq!(b["check"]["violations"], 0, "{b}");
    assert!(b["plan_items"].as_u64().unwrap() >= 1, "{b}");
    assert!(Path::new(&f.path("native2.json")).is_file());
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
    // rip_up is none, own or unlocked, and only for native routing.
    for extra in [
        json!({ "rip_up": "everything" }),
        json!({ "rip_up": true }),
        json!({ "rip_up": "unlocked", "source": "board_unlocked" }),
    ] {
        let mut args = json!({ "board": f.board(), "output_plan": f.path("p.json") });
        args.as_object_mut()
            .unwrap()
            .extend(extra.as_object().unwrap().clone());
        let r = handle_start_native_route_job(&args, &ctx()).await.unwrap();
        assert!(r.is_error && text(&r).contains("rip_up"), "{}", text(&r));
    }
    assert!(!Path::new(&f.path("p.json.konnect-native-job")).exists());
}

#[tokio::test]
async fn a_strict_refusal_names_each_defect_and_the_plan_tracks_behind_it() {
    let f = files();
    // A clean track of another net first, so the indices are the file's:
    // then two short sideways steps on SIG, each between two long runs.
    let plan = write_plan(
        &f,
        "jogs.json",
        vec![
            t("OTHER", "F.Cu", (0.0, 9.0), (5.0, 9.0)),
            t("SIG", "F.Cu", (0.0, 0.0), (5.0, 0.0)),
            t("SIG", "F.Cu", (5.0, 0.0), (5.0, 0.08)),
            t("SIG", "F.Cu", (5.0, 0.08), (9.0, 0.08)),
            t("SIG", "F.Cu", (9.0, 0.08), (9.0, 0.16)),
            t("SIG", "F.Cu", (9.0, 0.16), (14.0, 0.16)),
        ],
    );
    let ledger = f.path("ledger.json");
    let mut args = json!({ "board": f.board(), "routing_file": plan, "ledger_output": ledger });
    let r = handle_apply_route_plan(&args, &ctx()).await.unwrap();
    let said = text(&r);
    assert!(r.is_error, "{said}");
    let body: Value = serde_json::from_str(&said).unwrap();
    assert!(
        body["message"].as_str().unwrap().starts_with(
            "Nothing was created: the plan has 2 geometry defect(s) under geometry_check strict (2 short_jog)"
        ),
        "{said}"
    );
    assert_eq!(body["error"]["kind"], "invalid_argument");
    assert_eq!(body["error"]["field"], "routing_file");
    let detail = &body["geometry"];
    assert_eq!(detail["total"], 2, "{detail}");
    assert_eq!(detail["by_kind"], json!({ "short_jog": 2 }), "{detail}");
    assert_eq!(detail["listed"], 2);
    let first = &detail["defects"][0];
    assert_eq!(first["kind"], "short_jog");
    assert_eq!(first["net"], "SIG");
    assert_eq!(first["layer"], "F.Cu");
    assert_eq!((&first["x"], &first["y"]), (&json!(5.0), &json!(0.04)));
    let indices = |d: &Value| -> Vec<u64> {
        d["tracks"]
            .as_array()
            .unwrap()
            .iter()
            .map(|t| t["index"].as_u64().unwrap())
            .collect()
    };
    assert_eq!(indices(first), [1, 2, 3]);
    assert_eq!(indices(&detail["defects"][1]), [3, 4, 5]);
    assert_eq!(first["tracks"][1]["from"], json!([5.0, 0.0]));
    assert_eq!(first["tracks"][1]["to"], json!([5.0, 0.08]));
    assert!(detail["remedy"].as_str().unwrap().contains("warn"));
    assert!(detail.get("note").is_none());
    assert!(!Path::new(&ledger).exists());

    // The list is bounded; the totals are not.
    args["max_listed"] = json!(1);
    let r = handle_apply_route_plan(&args, &ctx()).await.unwrap();
    let said = text(&r);
    let body: Value = serde_json::from_str(&said).unwrap();
    let detail = &body["geometry"];
    assert_eq!(
        (&detail["total"], &detail["listed"]),
        (&json!(2), &json!(1))
    );
    assert_eq!(detail["defects"].as_array().unwrap().len(), 1);
    assert!(
        detail["note"].as_str().unwrap().contains("1 of 2"),
        "{detail}"
    );
}

#[tokio::test]
async fn a_strict_refusal_judges_the_plan_among_the_saved_boards_tracks_and_says_what_it_left() {
    let f = files();
    // The saved board holds a SIG track on F.Cu from (20, 10.5) to
    // (25, 10.5). This plan comes up to that track's end, steps 0.08 mm
    // sideways and carries on: a jog on its own, but on the board the
    // first bend is a three-way junction with the board track.
    let plan = write_plan(
        &f,
        "step.json",
        vec![
            t("SIG", "F.Cu", (25.0, 14.0), (25.0, 10.5)),
            t("SIG", "F.Cu", (25.0, 10.5), (25.08, 10.5)),
            t("SIG", "F.Cu", (25.08, 10.5), (25.08, 7.0)),
        ],
    );
    let ledger = f.path("ledger.json");
    let mut args = json!({ "board": f.board(), "routing_file": plan, "ledger_output": ledger });
    // No KiCad here. The lint passes among the saved tracks, so what is
    // missing is the open board, not a geometry defect.
    let r = handle_apply_route_plan(&args, &ctx()).await.unwrap();
    let said = text(&r);
    assert!(r.is_error && !said.contains("geometry defect"), "{said}");

    // A plan that replaces that board track is judged without it: the step
    // is a jog, and the refusal says the copper to replace was left too,
    // and that the verdict is the saved file's.
    let met = f.path("met.replaces.json");
    std::fs::write(&met, r#"["00000000-0000-0000-0000-000000000010"]"#).unwrap();
    args["replaces_file"] = json!(met);
    let r = handle_apply_route_plan(&args, &ctx()).await.unwrap();
    let said = text(&r);
    assert!(r.is_error, "{said}");
    let body: Value = serde_json::from_str(&said).unwrap();
    let message = body["message"].as_str().unwrap();
    assert!(
        message.starts_with(
            "Nothing was deleted or created: the plan has 1 geometry defect(s) under geometry_check strict (1 short_jog)"
        ),
        "{message}"
    );
    assert!(
        message.contains("judged among the tracks saved in") && message.contains("demo.kicad_pcb"),
        "{message}"
    );
    let defect = &body["geometry"]["defects"][0];
    assert_eq!(defect["kind"], "short_jog", "{said}");
    assert_eq!(defect["tracks"][1]["index"], 1, "{said}");

    // Replacing some other copper leaves the junction, and the plan passes.
    let other = f.path("other.replaces.json");
    std::fs::write(&other, r#"["00000000-0000-0000-0000-000000000011"]"#).unwrap();
    args["replaces_file"] = json!(other);
    let r = handle_apply_route_plan(&args, &ctx()).await.unwrap();
    let said = text(&r);
    assert!(r.is_error && !said.contains("geometry defect"), "{said}");
    assert!(!Path::new(&ledger).exists());
}

#[tokio::test]
async fn apply_route_plan_refuses_before_any_ipc() {
    let f = files();
    let plan = write_plan(&f, "p.json", vec![t("SIG", "F.Cu", (1.0, 1.0), (2.0, 1.0))]);
    let ledger = f.path("ledger.json");
    let call = |extra: Value| {
        let mut args = json!({ "board": f.board(), "routing_file": plan, "ledger_output": ledger });
        args.as_object_mut()
            .unwrap()
            .extend(extra.as_object().unwrap().clone());
        args
    };
    for (extra, field) in [
        (json!({ "geometry_check": "off" }), "geometry_check"),
        (json!({ "max_items": 0 }), "max_items"),
        (json!({ "max_items": 20_001 }), "max_items"),
        (json!({ "locked": "yes" }), "locked"),
        (
            json!({ "routing_file": f.path("missing.json") }),
            "routing_file",
        ),
        (
            json!({ "replaces_file": f.path("missing.json") }),
            "replaces_file",
        ),
        // A file that holds no UUID list.
        (json!({ "replaces_file": plan }), "replaces_file"),
    ] {
        let r = handle_apply_route_plan(&call(extra), &ctx()).await.unwrap();
        assert!(
            r.is_error && text(&r).contains(field),
            "{field}: {}",
            text(&r)
        );
    }
    std::fs::write(&ledger, "{}").unwrap();
    let r = handle_apply_route_plan(&call(json!({})), &ctx())
        .await
        .unwrap();
    assert!(
        r.is_error && text(&r).contains("never replaced"),
        "{}",
        text(&r)
    );
    assert_eq!(std::fs::read_to_string(&ledger).unwrap(), "{}");
}

#[tokio::test]
async fn map_route_plan_drc_follows_drc_items_to_plan_items() {
    let f = files();
    let plan = write_plan(
        &f,
        "p.json",
        vec![
            t("SIG", "F.Cu", (1.0, 1.0), (2.0, 1.0)),
            t("SIG", "F.Cu", (2.0, 1.0), (3.0, 2.0)),
        ],
    );
    let ledger = f.path("ledger.json");
    std::fs::write(
        &ledger,
        json!({
            "routing_file": plan, "locked": false, "complete": true,
            "ledger": { "chunks_total": 1, "chunks_applied": 1, "items_created": 2,
                        "items_unmapped": 0,
                        "items": [
                            { "kiid": "AAAA", "chunk": 0, "item": { "kind": "track", "index": 0 }, "net": "SIG" },
                            { "kiid": "bbbb", "chunk": 0, "item": { "kind": "track", "index": 1 }, "net": "SIG" }
                        ] }
        })
        .to_string(),
    )
    .unwrap();
    let item =
        |uuid: &str| json!({ "description": "x", "pos": { "x": 1.0, "y": 1.0 }, "uuid": uuid });
    let report = f.path("drc.json");
    std::fs::write(
        &report,
        json!({
            "violations": [
                { "severity": "error", "description": "Clearance", "rule": "clearance",
                  "pos": { "x": 1.0, "y": 1.0 }, "items": [item("aaaa"), item("pad-uuid")] },
                { "severity": "warning", "description": "Silk", "rule": "silk_overlap",
                  "pos": { "x": 1.0, "y": 1.0 }, "items": [item("other")] }
            ],
            "unconnected_items": [],
            "schematic_parity": []
        })
        .to_string(),
    )
    .unwrap();
    let r = handle_map_route_plan_drc(
        &json!({ "routing_file": plan, "ledger": ledger, "drc_report": report }),
        &ctx(),
    )
    .await
    .unwrap();
    assert!(!r.is_error, "{}", text(&r));
    let b = body(&r);
    assert_eq!(b["touching_plan"], 1);
    assert_eq!(b["not_touching_plan"], 1);
    assert_eq!(b["by_rule"]["clearance/error"]["touching_plan"], 1);
    assert_eq!(b["nets"], json!(["SIG"]));
    assert_eq!(b["listed"][0]["items"][0]["plan_item"]["index"], 0);

    let r = handle_map_route_plan_drc(&json!({ "routing_file": plan, "ledger": ledger }), &ctx())
        .await
        .unwrap();
    assert!(r.is_error && text(&r).contains("drc_report"));
}

#[test]
fn allowance_arguments_reach_the_spec_and_typos_are_refused() {
    let f = files();
    let args = json!({
        "board": f.board(), "output_plan": f.path("p.json"), "fanout_plan": f.path("fan.json"),
        "class_layers": { "Default": ["F.Cu"] },
        "layer_fallback": [ { "class": "Default", "layers": ["F.Cu", "B.Cu"], "nets": ["G*"],
                              "reference": { "plane": "B.Cu", "margin_mm": 0.3 } } ],
        "neck_down": { "exclude_classes": ["Default"] },
        "exclude_classes": []
    });
    let spec = job_spec(&args).unwrap().unwrap();
    let a = &spec.allowances;
    assert_eq!(a.class_layers["Default"], vec!["F.Cu".to_string()]);
    assert_eq!(
        a.layer_fallback[0].reference.as_ref().unwrap().margin_mm,
        0.3
    );
    assert_eq!(a.neck_down.as_ref().unwrap().width_mm, 0.1);
    assert_eq!(a.exclude_classes, Some(Vec::new()));
    assert!(spec.fanout_plan.as_ref().unwrap().is_absolute());
    // Nothing named: the default policy, serialised as nothing.
    let plain = job_spec(&json!({ "board": f.board(), "output_plan": f.path("q.json") }))
        .unwrap()
        .unwrap();
    assert!(plain.allowances.is_default());
    let typo = json!({ "board": f.board(), "output_plan": f.path("r.json"),
                       "neck_down": { "widht_mm": 0.1 } });
    assert!(job_spec(&typo).unwrap().is_err());
    assert!(class_mode(&json!({ "net_classes": "maybe" })).is_err());
    assert_eq!(class_mode(&json!({})).unwrap(), ClassMode::Auto);
    assert!(class_mode(&json!({ "net_classes": true })).is_err());
    assert_eq!(
        class_mode(&json!({ "net_classes": null })).unwrap(),
        ClassMode::Auto
    );
    // Allowances steer native routing only: with copper taken from the
    // board they would be silently ignored, so they are refused.
    let taken = json!({ "board": f.board(), "output_plan": f.path("s.json"),
                        "source": "board_unlocked", "exclude_classes": [] });
    assert!(job_spec(&taken).unwrap().is_err());
    let taken_plain = json!({ "board": f.board(), "output_plan": f.path("t.json"),
                              "source": "board_unlocked" });
    assert!(job_spec(&taken_plain).unwrap().is_ok());
}

#[tokio::test]
async fn project_classes_need_no_kicad() {
    let f = files();
    let args = json!({ "board": f.board(), "output_plan": f.path("p.json") });
    let mut spec = job_spec(&args).unwrap().unwrap();
    // No IPC address configured: auto falls back to the patterns, kicad
    // refuses.
    let auto = kicad_classes(&ctx(), &mut spec, ClassMode::Auto)
        .await
        .unwrap()
        .unwrap();
    assert_eq!(auto["source"], "project_patterns");
    assert!(spec.net_classes.is_none());
    assert!(kicad_classes(&ctx(), &mut spec, ClassMode::Kicad)
        .await
        .unwrap()
        .is_err());
}

#[tokio::test]
async fn zone_options_reach_the_spec_and_are_refused_before_anything_runs() {
    let f = files();
    let args = json!({
        "board": f.board(), "output_plan": f.path("z.json"),
        "carve_zones": ["GND plane"], "carve_zone_nets": ["G*"], "plane_nets_as_tracks": ["GND"]
    });
    let spec = job_spec(&args).unwrap().unwrap();
    assert_eq!(spec.carve_zones, vec!["GND plane".to_string()]);
    assert_eq!(spec.carve_zone_nets, vec!["G*".to_string()]);
    assert_eq!(spec.plane_nets_as_tracks, vec!["GND".to_string()]);
    assert!(check_zone_options(&spec).await.unwrap().is_ok());
    let plain = job_spec(&json!({ "board": f.board(), "output_plan": f.path("y.json") }))
        .unwrap()
        .unwrap();
    assert!(plain.carve_zones.is_empty() && plain.plane_nets_as_tracks.is_empty());
    // Routing nothing, there is no plane net to route as tracks.
    let taken = json!({ "board": f.board(), "output_plan": f.path("x.json"),
                        "source": "board_unlocked", "plane_nets_as_tracks": ["GND"] });
    assert!(job_spec(&taken).unwrap().is_err());
    assert!(job_spec(
        &json!({ "board": f.board(), "output_plan": f.path("w.json"),
                              "carve_zones": "GND plane" })
    )
    .unwrap()
    .is_err());

    // A rule area, an unknown zone and a net without a zone are each
    // refused by both tools, naming the argument; nothing is written.
    let region = json!({ "x1": 15.0, "y1": 5.0, "x2": 40.0, "y2": 30.0 });
    for (key, value, says) in [
        ("carve_zones", json!(["no_vias"]), "rule area"),
        (
            "carve_zones",
            json!(["Nope"]),
            "no copper zone of the board is named 'Nope'",
        ),
        (
            "carve_zone_nets",
            json!(["SIG"]),
            "no net with a copper zone matches 'SIG'",
        ),
        (
            "plane_nets_as_tracks",
            json!(["D?"]),
            "no net with a copper zone matches 'D?'",
        ),
    ] {
        let mut bad =
            json!({ "board": f.board(), "output_plan": f.path("bad.json"), "region": region });
        bad[key] = value;
        for started in [false, true] {
            let r = if started {
                handle_start_native_route_job(&bad, &ctx()).await.unwrap()
            } else {
                handle_route_native_region(&bad, &ctx()).await.unwrap()
            };
            let said = text(&r);
            assert!(r.is_error, "{key}: {said}");
            assert!(said.contains(key) && said.contains(says), "{key}: {said}");
            assert!(said.contains("Nothing was started"), "{said}");
            assert!(!Path::new(&f.path("bad.json")).exists());
            assert!(!native_route_job::job_dir_for(Path::new(&f.path("bad.json"))).exists());
        }
    }
}

#[tokio::test]
async fn route_native_region_accounts_for_plane_nets_and_names_the_refill() {
    let f = files();
    let args = json!({
        "board": f.board(), "output_plan": f.path("planes.json"), "threads": 2,
        "region": { "x1": 15.0, "y1": 5.0, "x2": 40.0, "y2": 30.0 },
        "carve_zone_nets": ["*"]
    });
    let r = handle_route_native_region(&args, &ctx()).await.unwrap();
    assert!(!r.is_error, "{}", text(&r));
    let b = body(&r);
    let gnd = &b["plane_nets"][0];
    assert_eq!(gnd["net"], "GND", "{b}");
    assert_eq!(gnd["fill"], "filled");
    assert_eq!(gnd["handling"], "left_to_plane");
    assert_eq!(
        gnd["connected_by_plane"],
        json!({ "count": 1, "pads": ["J1.1"] })
    );
    assert_eq!(
        gnd["connected_by_fanout"],
        json!({ "count": 1, "pads": ["R1.2"] })
    );
    assert_eq!(gnd["needs_via_or_track"], json!([]));
    assert_eq!(b["warnings"], json!([]));
    // The In1 plane was declared crossable, but no new copper is on In1.
    assert_eq!(b["refill"]["required"], false, "{b}");
    assert_eq!(
        b["refill"]["crossable_zones"],
        json!([{ "zone": "GND plane", "layer": "In1.Cu", "net": "GND" }])
    );
    assert_eq!(b["refill"]["crossed_zones"], json!([]));
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

fn feed_files() -> Files {
    let dir = tempfile::tempdir().unwrap();
    let root = dir.path().to_path_buf();
    std::fs::write(root.join("demo.kicad_pcb"), FEED_BOARD).unwrap();
    std::fs::write(root.join("demo.kicad_pro"), FEED_PRO).unwrap();
    Files { _dir: dir, root }
}

#[test]
fn track_widths_reach_the_spec_and_typos_are_refused() {
    let f = feed_files();
    let args = json!({
        "board": f.board(), "output_plan": f.path("p.json"), "exclude_classes": [],
        "track_widths": [
            { "net": "+5V", "width_mm": 0.3 },
            { "net": "+5V", "from": "J1.1", "to": "R1.1", "width_mm": 0.2 }
        ]
    });
    let spec = job_spec(&args).unwrap().unwrap();
    let w = &spec.allowances.track_widths;
    assert_eq!(
        (w[0].net.as_str(), w[0].width_mm, w[0].from.as_deref()),
        ("+5V", 0.3, None)
    );
    assert_eq!(
        (w[1].from.as_deref(), w[1].to.as_deref()),
        (Some("J1.1"), Some("R1.1"))
    );
    assert!(spec.check_allowances().unwrap().is_ok());
    let typo = json!({ "board": f.board(), "output_plan": f.path("q.json"),
                       "track_widths": [{ "net": "+5V", "width": 0.3 }] });
    assert!(job_spec(&typo).unwrap().is_err());
    // A width steers native routing only.
    let taken = json!({ "board": f.board(), "output_plan": f.path("r.json"),
                        "source": "board_unlocked",
                        "track_widths": [{ "net": "+5V", "width_mm": 0.3 }] });
    assert!(job_spec(&taken).unwrap().is_err());
}

#[tokio::test]
async fn a_track_width_the_board_cannot_take_is_refused_before_a_job_starts() {
    let f = feed_files();
    let refusal = |entries: Value| {
        let args = json!({ "board": f.board(), "output_plan": f.path("job.json"),
                           "exclude_classes": [], "net_classes": "project",
                           "track_widths": entries });
        async move {
            let r = handle_start_native_route_job(&args, &ctx()).await.unwrap();
            assert!(r.is_error, "{}", text(&r));
            let region = {
                let mut a = args.clone();
                a["region"] = json!({ "x1": 0, "y1": 0, "x2": 26, "y2": 14 });
                handle_route_native_region(&a, &ctx()).await.unwrap()
            };
            assert!(region.is_error);
            assert_eq!(body(&region)["error"], body(&r)["error"]);
            r
        }
    };
    let field = |r: &CallToolResult| {
        let b = body(r);
        assert_eq!(b["error"]["kind"], "invalid_argument", "{b}");
        b["error"]["field"].as_str().unwrap().to_string()
    };
    // Below the board minimum (0.1 mm): refused naming the entry.
    let r = refusal(json!([
        { "net": "SIG", "width_mm": 0.25 }, { "net": "+5V", "width_mm": 0.05 }
    ]))
    .await;
    assert_eq!(field(&r), "track_widths[1]");
    assert!(
        text(&r).contains("below the minimum track width"),
        "{}",
        text(&r)
    );
    // An unknown net, and an unknown pad.
    let r = refusal(json!([{ "net": "+12V", "width_mm": 0.3 }])).await;
    assert_eq!(field(&r), "track_widths[0]");
    assert!(text(&r).contains("no net named '+12V'"), "{}", text(&r));
    let r = refusal(json!([
        { "net": "+5V", "from": "J1.1", "to": "R1.7", "width_mm": 0.3 }
    ]))
    .await;
    assert_eq!(field(&r), "track_widths[0]");
    assert!(
        text(&r).contains("footprint 'R1' has no pad '7'"),
        "{}",
        text(&r)
    );
    // Nothing was started or written: no job directory, no plan.
    assert!(!f.root.join("job.json").exists());
    assert!(!native_route_job::job_dir_for(&f.root.join("job.json")).exists());
}

#[tokio::test]
async fn route_native_region_routes_a_net_at_its_track_width_and_reports_the_override() {
    let f = feed_files();
    let args = |out: &str, widths: Value| {
        let mut a = json!({ "board": f.board(), "output_plan": f.path(out),
                            "region": { "x1": 0, "y1": 0, "x2": 26, "y2": 14 },
                            "exclude_classes": [], "net_classes": "project", "threads": 2 });
        if !widths.is_null() {
            a["track_widths"] = widths;
        }
        a
    };
    let widths = |out: &str, net: &str| {
        let plan: RoutePlan =
            serde_json::from_str(&std::fs::read_to_string(f.root.join(out)).unwrap()).unwrap();
        let mut w: Vec<f64> = plan
            .tracks
            .iter()
            .filter(|t| t.net == net)
            .map(|t| t.width)
            .collect();
        w.sort_by(f64::total_cmp);
        w.dedup();
        w
    };
    let r = handle_route_native_region(&args("plain.json", Value::Null), &ctx())
        .await
        .unwrap();
    assert!(!r.is_error, "{}", text(&r));
    assert_eq!(widths("plain.json", "+5V"), vec![0.8]);
    assert!(body(&r)["allowances"].get("track_widths").is_none());

    let r = handle_route_native_region(
        &args("hinted.json", json!([{ "net": "+5V", "width_mm": 0.3 }])),
        &ctx(),
    )
    .await
    .unwrap();
    assert!(!r.is_error, "{}", text(&r));
    let b = body(&r);
    assert_eq!(widths("hinted.json", "+5V"), vec![0.3]);
    assert_eq!(widths("hinted.json", "SIG"), widths("plain.json", "SIG"));
    assert_eq!(b["check"]["violations"], 0, "{b}");
    assert_eq!(
        b["allowances"]["track_widths"],
        json!([{ "net": "+5V", "width_nm": 300_000, "overrides_class": "Power",
                 "class_width_nm": 800_000, "connections": 2, "fanout_stubs": 0 }])
    );
}
