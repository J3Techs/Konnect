//! Tool-level tests of `build_route`, `plan_trunk`'s regularization and the
//! routing-file lists of `check_route_rules`, `render_board_layers_png` and
//! `render_board_tiles`, on a hand-written KiCad 10 board. Expected
//! geometry is worked out by hand from the board text.

use crate::mcp::protocol::{CallToolResult, ToolContent};
use crate::router::ToolRouter;
use crate::tools::{board_render, route_native, route_plan_tools, ServerConfig, ToolContext};
use serde_json::{json, Value};
use std::path::{Path, PathBuf};
use std::sync::Arc;

const PRO: &str = r#"{
    "board": { "design_settings": { "rules": {
        "min_clearance": 0.1, "min_track_width": 0.1, "min_via_diameter": 0.4,
        "min_via_annular_width": 0.1, "min_through_hole_diameter": 0.2,
        "min_copper_edge_clearance": 0.3, "min_hole_clearance": 0.2, "min_hole_to_hole": 0.25
    } } },
    "net_settings": {
        "classes": [
            { "name": "Default", "clearance": 0.3, "track_width": 0.2,
              "via_diameter": 0.8, "via_drill": 0.4 }
        ],
        "netclass_patterns": []
    }
}"#;

/// Two through-hole pads off any 0.1 mm grid (J1.1 of PWR_A at
/// (40.0313, 30.0157), J1.2 of PWR_B 4.4 mm to its right), a via of each
/// net 2.5 mm apart on x = 10.1234, and an SMD pad of SIG near the top.
const BOARD: &str = r#"(kicad_pcb
	(version 20260206)
	(generator "pcbnew")
	(general (thickness 1.6))
	(layers
		(0 "F.Cu" signal)
		(4 "In1.Cu" signal)
		(6 "In2.Cu" signal)
		(2 "B.Cu" signal)
		(25 "Edge.Cuts" user)
	)
	(footprint "Conn"
		(layer "F.Cu")
		(uuid "fp-j1")
		(at 40.0313 30.0157)
		(property "Reference" "J1" (at 0 -3 0) (layer "F.SilkS"))
		(pad "1" thru_hole circle (at 0 0) (size 2 2) (drill 1.1) (layers "*.Cu" "*.Mask") (net "PWR_A") (uuid "pad-j1-1"))
		(pad "2" thru_hole circle (at 4.4 0) (size 2 2) (drill 1.1) (layers "*.Cu" "*.Mask") (net "PWR_B") (uuid "pad-j1-2"))
	)
	(footprint "R"
		(layer "F.Cu")
		(uuid "fp-r1")
		(at 25 3)
		(property "Reference" "R1" (at 0 -1 0) (layer "F.SilkS"))
		(pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu" "F.Mask") (net "SIG") (uuid "pad-r1-1"))
	)
	(via (at 10.1234 9.8765) (size 0.8) (drill 0.4) (layers "F.Cu" "B.Cu") (net "PWR_A") (uuid "via-a"))
	(via (at 10.1234 7.3765) (size 0.8) (drill 0.4) (layers "F.Cu" "B.Cu") (net "PWR_B") (uuid "via-b"))
	(gr_rect (start 0 0) (end 60 45) (stroke (width 0.05) (type default)) (fill no) (layer "Edge.Cuts"))
)
"#;

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

fn text(r: &CallToolResult) -> String {
    match &r.content[0] {
        ToolContent::Text { text } => text.clone(),
        _ => String::new(),
    }
}

fn body(r: &CallToolResult) -> Value {
    serde_json::from_str(&text(r)).unwrap_or(json!({ "text": text(r) }))
}

fn field_of(r: &CallToolResult) -> String {
    body(r)["error"]["field"].as_str().unwrap_or("").to_string()
}

struct Files {
    _dir: tempfile::TempDir,
    root: PathBuf,
}

impl Files {
    fn board(&self) -> String {
        self.path("demo.kicad_pcb")
    }
    fn path(&self, name: &str) -> String {
        self.root.join(name).display().to_string()
    }
}

fn files() -> Files {
    let dir = tempfile::tempdir().unwrap();
    let root = dir.path().to_path_buf();
    std::fs::write(root.join("demo.kicad_pcb"), BOARD).unwrap();
    std::fs::write(root.join("demo.kicad_pro"), PRO).unwrap();
    Files { _dir: dir, root }
}

fn with(base: Value, extra: Value) -> Value {
    let mut a = base;
    a.as_object_mut()
        .unwrap()
        .extend(extra.as_object().unwrap().clone());
    a
}

fn xy(x: f64, y: f64) -> Value {
    json!({ "x": x, "y": y })
}

/// Via A to J1.1: east 5, down-right 15, east 4.7687, down-right 5.1392.
fn route_a() -> Value {
    json!([
        xy(10.1234, 9.8765),
        xy(15.1234, 9.8765),
        xy(30.1234, 24.8765),
        xy(34.8921, 24.8765),
        xy(40.0313, 30.0157)
    ])
}

async fn build(f: &Files, out: &str, extra: Value) -> CallToolResult {
    let args = with(
        json!({ "board": f.board(), "net": "PWR_A", "layer": "In2.Cu", "output_plan": f.path(out),
                "width_mm": 1.5 }),
        extra,
    );
    crate::tools::route_build::handle_build_route(&args, &ctx())
        .await
        .unwrap()
}

fn plan_tracks(path: &str) -> Vec<Value> {
    let v: Value = serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
    v["tracks"].as_array().unwrap().clone()
}

// ─── build_route ─────────────────────────────────────────────────────────────

#[tokio::test]
async fn build_route_writes_a_clean_five_vertex_route_onto_its_anchors() {
    let f = files();
    let r = build(
        &f,
        "a.json",
        json!({ "vertices": route_a(), "start": { "via_uuid": "via-a" }, "end": { "pad": "J1.1" } }),
    )
    .await;
    assert!(!r.is_error, "{}", text(&r));
    let b = body(&r);
    assert_eq!(b["plan_written"], true, "{b}");
    assert_eq!(b["check"]["violations"], 0, "{b}");
    assert_eq!(b["lint"]["defects"], 0, "{b}");
    assert_eq!(b["anchors"]["start"]["offset_mm"], 0.0);
    assert_eq!(b["anchors"]["end"]["at"], json!([40.0313, 30.0157]));
    let dirs: Vec<&str> = b["segments"]
        .as_array()
        .unwrap()
        .iter()
        .map(|s| s["direction"].as_str().unwrap())
        .collect();
    assert_eq!(dirs, ["E", "SE", "E", "SE"]);
    assert_eq!(b["smallest_interior_angle_deg"], 135.0);
    assert_eq!(b["shortest_segment_mm"], 4.7687);
    // The file holds exactly the given vertices, in order.
    let t = plan_tracks(&f.path("a.json"));
    assert_eq!(t.len(), 4);
    assert_eq!(
        (t[0]["x1"].as_f64(), t[0]["y1"].as_f64()),
        (Some(10.1234), Some(9.8765))
    );
    assert_eq!(
        (t[3]["x2"].as_f64(), t[3]["y2"].as_f64()),
        (Some(40.0313), Some(30.0157))
    );
    assert_eq!(t[1]["width"], 1.5);
    // The same output again is never replaced.
    let r = build(&f, "a.json", json!({ "vertices": route_a() })).await;
    assert!(
        r.is_error && text(&r).contains("never replaced"),
        "{}",
        text(&r)
    );
}

#[tokio::test]
async fn build_route_refuses_bad_geometry_and_anchors_and_writes_nothing() {
    let f = files();
    let mut skew = route_a();
    skew[2] = xy(30.1234, 24.9);
    let mut short = route_a();
    short[1] = xy(10.2234, 9.8765);
    short[2] = xy(25.2234, 24.8765);
    short[3] = xy(34.8921, 24.8765);
    let acute = json!([xy(10.1234, 9.8765), xy(20.0, 9.8765), xy(15.0, 14.8765)]);
    let mut off_by_2um = route_a();
    off_by_2um[4] = xy(40.0313, 30.0177);
    off_by_2um[3] = xy(34.8941, 24.8765);
    for (extra, field, words) in [
        (json!({ "vertices": skew }), "vertices", "off 0, 45 or 90"),
        (
            json!({ "vertices": short, "min_segment_mm": 0.25 }),
            "vertices",
            "shorter than min_segment_mm",
        ),
        (
            json!({ "vertices": acute }),
            "vertices",
            "interior angle of 45.00",
        ),
        (
            json!({ "vertices": off_by_2um, "end": { "pad": "J1.1" } }),
            "end",
            "0.0020 mm from pad J1.1",
        ),
        (
            json!({ "vertices": route_a(), "end": { "pad": "J1.2" } }),
            "end.pad",
            "is on PWR_B, not PWR_A",
        ),
        (
            json!({ "vertices": route_a(), "start": { "via_uuid": "via-b" } }),
            "start.via_uuid",
            "is on PWR_B",
        ),
        (
            json!({ "vertices": route_a(), "start": { "via_at": xy(10.1234, 9.88) } }),
            "start.via_at",
            "within 1 micron",
        ),
        (
            json!({ "vertices": route_a(), "end": { "pad": "R1.1" } }),
            "end.pad",
            "no copper on In2.Cu",
        ),
        (
            json!({ "vertices": route_a(), "start": { "pad": "J1.1", "via_uuid": "via-a" } }),
            "start",
            "exactly one",
        ),
        (
            json!({ "vertices": route_a(), "extend_to": { "end": 45 } }),
            "extend_to.end",
            "needs a end anchor",
        ),
        (
            json!({ "vertices": route_a(), "width_mm": null, "segment_widths": [1.0, 1.0] }),
            "segment_widths",
            "4 segments",
        ),
        (
            json!({ "vertices": route_a(), "net": "NOPE" }),
            "net",
            "not a net",
        ),
    ] {
        let r = build(&f, "bad.json", extra.clone()).await;
        assert!(r.is_error, "{extra} accepted");
        assert_eq!(field_of(&r), field, "{extra}: {}", text(&r));
        assert!(text(&r).contains(words), "{extra}: {}", text(&r));
        assert!(!Path::new(&f.path("bad.json")).exists());
    }
    // Any angle may be allowed; the skewed route then checks and writes.
    let mut skew = route_a();
    skew[2] = xy(30.1234, 24.9);
    let r = build(
        &f,
        "skew.json",
        json!({ "vertices": skew, "allow_any_angle": true }),
    )
    .await;
    assert!(!r.is_error, "{}", text(&r));
    assert_eq!(body(&r)["plan_written"], true);
    // A route through another net's via is checked, reported and not
    // written.
    let through = json!([xy(5.0, 7.3765), xy(15.0, 7.3765)]);
    let r = build(&f, "through.json", json!({ "vertices": through })).await;
    assert!(!r.is_error, "{}", text(&r));
    let b = body(&r);
    assert_eq!(b["plan_written"], false);
    assert!(b["check"]["violations"].as_u64().unwrap() >= 1, "{b}");
    assert!(!Path::new(&f.path("through.json")).exists());
}

#[tokio::test]
async fn build_route_extends_into_an_anchor_and_runs_parallel_at_a_pitch() {
    let f = files();
    let r = build(&f, "a.json", json!({ "vertices": route_a() })).await;
    assert_eq!(body(&r)["plan_written"], true, "{}", text(&r));

    // Route B: route A offset 2.5 mm to its left starts exactly on via B
    // (2.5 mm above via A); its last run turns 45 degrees into J1.2.
    let args = json!({
        "board": f.board(), "net": "PWR_B", "layer": "In2.Cu", "output_plan": f.path("b.json"),
        "width_mm": 1.5,
        "parallel_to": { "routing_file": f.path("a.json"), "pitch_mm": 2.5, "side": "left" },
        "start": { "via_at": xy(10.1234, 7.3765) }, "end": { "pad": "J1.2" },
        "extend_to": { "end": 45 }
    });
    let r = crate::tools::route_build::handle_build_route(&args, &ctx())
        .await
        .unwrap();
    assert!(!r.is_error, "{}", text(&r));
    let b = body(&r);
    assert_eq!(b["built_from"], "parallel_to");
    assert_eq!(b["check"]["violations"], 0, "{b}");
    assert_eq!(b["lint"]["defects"], 0, "{b}");
    assert_eq!(b["plan_written"], true);
    let v = b["vertices"].as_array().unwrap();
    assert_eq!(v[0], json!([10.1234, 7.3765]), "starts on via B");
    assert_eq!(
        v.last().unwrap(),
        &json!([44.4313, 30.0157]),
        "ends on J1.2"
    );
    // The runs before the turn are route A's at exactly 2.5 mm: the first
    // two are y = 7.3765 and the 45 degree line x - y = 20.2469 - 2.5*sqrt2.
    assert_eq!(v[1][1], json!(7.3765));
    let (x2, y2) = (v[2][0].as_f64().unwrap(), v[2][1].as_f64().unwrap());
    let c = 30.1234 - 24.8765 + 2.5 * std::f64::consts::SQRT_2;
    assert!(((x2 - y2) - c).abs() < 1.5e-6, "{x2} {y2}");
    // Every segment is exactly octilinear and the last one is east or
    // south into the pad.
    let dirs: Vec<&str> = b["segments"]
        .as_array()
        .unwrap()
        .iter()
        .map(|s| s["direction"].as_str().unwrap_or("skew"))
        .collect();
    assert!(dirs.iter().all(|d| *d != "skew"), "{dirs:?}");
    assert!(["E", "S"].contains(dirs.last().unwrap()), "{dirs:?}");
    assert!(b["extend_to"]["end"]["turn_vertex"].is_array());
    assert!(b["avoid_routing_files"]
        .as_array()
        .unwrap()
        .contains(&json!(f.path("a.json"))));
    // parallel_to without a side is refused.
    let mut bad = args.clone();
    bad["output_plan"] = json!(f.path("b2.json"));
    bad["parallel_to"] = json!({ "routing_file": f.path("a.json"), "pitch_mm": 2.5 });
    let r = crate::tools::route_build::handle_build_route(&bad, &ctx())
        .await
        .unwrap();
    assert!(r.is_error);
    assert_eq!(field_of(&r), "parallel_to.side", "{}", text(&r));
}

#[tokio::test]
async fn reproducing_a_route_already_on_the_board_checks_and_lints_clean() {
    let f = files();
    let segments: String = [
        ((10.1234, 9.8765), (15.1234, 9.8765)),
        ((15.1234, 9.8765), (30.1234, 24.8765)),
        ((30.1234, 24.8765), (34.8921, 24.8765)),
        ((34.8921, 24.8765), (40.0313, 30.0157)),
    ]
    .iter()
    .enumerate()
    .map(|(i, (a, b))| {
        format!(
            "\t(segment (start {} {}) (end {} {}) (width 1.5) (layer \"In2.Cu\") (net \"PWR_A\") (uuid \"seg-{i}\"))\n",
            a.0, a.1, b.0, b.1
        )
    })
    .collect();
    let board = BOARD.replace("\t(gr_rect", &format!("{segments}\t(gr_rect"));
    std::fs::write(f.path("demo.kicad_pcb"), board).unwrap();
    let r = build(
        &f,
        "again.json",
        json!({ "vertices": route_a(), "start": { "via_at": xy(10.1234, 9.8765) }, "end": { "pad": "J1.1" } }),
    )
    .await;
    assert!(!r.is_error, "{}", text(&r));
    let b = body(&r);
    assert_eq!(b["check"]["violations"], 0, "{b}");
    assert_eq!(b["lint"]["board_tracks_repeated_by_the_plan"], 4, "{b}");
    assert_eq!(b["check"]["board_tracks_repeated_set_aside"], 4, "{b}");
    assert_eq!(b["lint"]["defects"], 0, "{b}");
}

// ─── plan_trunk regularization ───────────────────────────────────────────────

#[tokio::test]
async fn plan_trunk_lands_on_an_off_grid_pad_with_no_run_under_a_millimetre() {
    let f = files();
    let args = json!({
        "board": f.board(), "net": "PWR_A", "layer": "In2.Cu", "output_plan": f.path("t.json"),
        "waypoints": [xy(10.1234, 9.8765), xy(40.0313, 30.0157)],
        "preferred_width_mm": 1.5, "min_width_mm": 0.5
    });
    let r = route_plan_tools::handle_plan_trunk(&args, &ctx())
        .await
        .unwrap();
    assert!(!r.is_error, "{}", text(&r));
    let b = body(&r);
    assert_eq!(b["complete"], true, "{b}");
    assert_eq!(b["regularize"]["on"], true);
    assert_eq!(b["regularize"]["min_segment_mm"], 1.0);
    let leg = &b["legs"][0];
    assert_ne!(leg["regularization"], "forced", "{leg}");
    assert!(leg["shortest_segment_mm"].as_f64().unwrap() >= 1.0, "{leg}");
    assert_eq!(leg["segments_under_minimum"], 0);
    let segs = leg["segments"].as_array().unwrap();
    assert_eq!(segs[0]["from"], json!([10.1234, 9.8765]));
    assert_eq!(segs.last().unwrap()["to"], json!([40.0313, 30.0157]));
    for s in segs {
        let (a, z) = (&s["from"], &s["to"]);
        let dx = ((z[0].as_f64().unwrap() - a[0].as_f64().unwrap()) * 1e6)
            .round()
            .abs();
        let dy = ((z[1].as_f64().unwrap() - a[1].as_f64().unwrap()) * 1e6)
            .round()
            .abs();
        assert!(dx == 0.0 || dy == 0.0 || dx == dy, "{s}");
    }
    assert_eq!(b["lint"]["defects"], 0, "{b}");
    assert_eq!(b["lint"]["by_kind"]["short_jog"], 0);
    assert_eq!(b["lint"]["by_kind"]["via_dogleg"], 0);
    assert_eq!(b["lint"]["by_kind"]["pad_dogleg"], 0);
    assert_eq!(b["check"]["violations"], 0);

    // Off: the reply says so; a bad minimum is refused.
    let off = with(
        args.clone(),
        json!({ "output_plan": f.path("off.json"), "regularize": false }),
    );
    let r = route_plan_tools::handle_plan_trunk(&off, &ctx())
        .await
        .unwrap();
    assert_eq!(body(&r)["legs"][0]["regularization"], "off", "{}", text(&r));
    let bad = with(
        args,
        json!({ "output_plan": f.path("bad.json"), "min_segment_mm": 0 }),
    );
    let r = route_plan_tools::handle_plan_trunk(&bad, &ctx())
        .await
        .unwrap();
    assert!(r.is_error);
    assert_eq!(field_of(&r), "min_segment_mm");
}

// ─── Routing-file lists ──────────────────────────────────────────────────────

fn write(f: &Files, name: &str, body: Value) -> String {
    std::fs::write(f.path(name), body.to_string()).unwrap();
    f.path(name)
}

fn track(net: &str, a: (f64, f64), b: (f64, f64)) -> Value {
    json!({ "net": net, "layer": "In2.Cu", "width": 0.5, "x1": a.0, "y1": a.1, "x2": b.0, "y2": b.1 })
}

#[tokio::test]
async fn check_route_rules_merges_a_list_and_names_the_file_of_each_side() {
    let f = files();
    let one = write(
        &f,
        "one.json",
        json!({ "tracks": [track("PWR_A", (5.0, 20.0), (15.0, 20.0))] }),
    );
    let two = write(
        &f,
        "two.json",
        json!({ "tracks": [track("PWR_B", (5.0, 22.0), (15.0, 22.0))] }),
    );
    // 0.3 mm from file two's track: copper 0.3 - 0.5 apart, a violation
    // only the merged batch sees.
    let three = write(
        &f,
        "three.json",
        json!({ "tracks": [track("SIG", (5.0, 22.3), (15.0, 22.3))], "vias": [] }),
    );
    let check = |extra: Value| {
        let a = with(json!({ "board": f.board() }), extra);
        async move {
            route_native::handle_check_route_rules(&a, &ctx())
                .await
                .unwrap()
        }
    };
    let r = check(json!({ "routing_files": [one, two] })).await;
    assert!(!r.is_error, "{}", text(&r));
    let b = body(&r);
    assert_eq!(b["violations"], 0);
    assert_eq!(b["tracks"], 2);
    assert_eq!(b["routing_files"][1]["tracks"], 1);

    let r = check(json!({ "routing_files": [one, two, three] })).await;
    let b = body(&r);
    assert!(b["violations"].as_u64().unwrap() >= 1, "{b}");
    let v = &b["listed"][0];
    let files: Vec<u64> = [&v["a"], &v["b"]]
        .iter()
        .filter_map(|s| s["file"].as_u64())
        .collect();
    assert_eq!(files.len(), 2, "{v}");
    assert!(files.contains(&1) && files.contains(&2), "{v}");
    assert_eq!(v["a"]["member"], "tracks[0]");

    // A bad member names its file's index and path; a missing file is not
    // found; both forms at once and neither are refused.
    let bad = write(
        &f,
        "bad.json",
        json!({ "tracks": [track("NOPE", (1.0, 1.0), (2.0, 1.0))] }),
    );
    let r = check(json!({ "routing_files": [one, two, bad] })).await;
    assert!(r.is_error);
    assert_eq!(field_of(&r), "routing_files[2]", "{}", text(&r));
    assert!(
        text(&r).contains("bad.json") && text(&r).contains("tracks[0].net"),
        "{}",
        text(&r)
    );
    let r = check(json!({ "routing_files": [one, f.path("missing.json")] })).await;
    assert!(
        r.is_error && text(&r).contains("routing_files[1] is not a file"),
        "{}",
        text(&r)
    );
    let r = check(json!({ "routing_files": [one], "routing_file": two })).await;
    assert_eq!(field_of(&r), "routing_files", "{}", text(&r));
    let r = check(json!({})).await;
    assert_eq!(field_of(&r), "routing_file", "{}", text(&r));
    // The single form still works.
    let r = check(json!({ "routing_file": one })).await;
    assert_eq!(body(&r)["violations"], 0, "{}", text(&r));
    assert!(body(&r)["routing_files"].is_null());
}

#[tokio::test]
async fn renders_lay_a_list_of_routing_files_over_the_board() {
    let f = files();
    let one = write(
        &f,
        "one.json",
        json!({ "tracks": [track("PWR_A", (5.0, 20.0), (15.0, 20.0))] }),
    );
    let two = write(
        &f,
        "two.json",
        json!({ "tracks": [track("PWR_B", (5.0, 22.0), (15.0, 22.0)), track("PWR_B", (15.0, 22.0), (17.0, 24.0))] }),
    );
    let three = write(
        &f,
        "three.json",
        json!({ "vias": [{ "net": "SIG", "x": 20.0, "y": 20.0, "drill": 0.3, "diameter": 0.6 }] }),
    );
    let r = board_render::handle_render_board_layers_png(
        &json!({ "board": f.board(), "layers": ["In2.Cu"], "px_per_mm": 4,
                 "overlay_routing_files": [one, two, three], "output": f.path("o.png") }),
        &ctx(),
    )
    .await
    .unwrap();
    assert!(!r.is_error, "{}", text(&r));
    let d = &body(&r)["drawn"];
    assert_eq!(
        (d["overlay_tracks"].as_u64(), d["overlay_vias"].as_u64()),
        (Some(3), Some(1))
    );

    let bad = write(&f, "bad.json", json!({ "segments": [] }));
    let r = board_render::handle_render_board_layers_png(
        &json!({ "board": f.board(), "overlay_routing_files": [one, bad], "output": f.path("x.png") }),
        &ctx(),
    )
    .await
    .unwrap();
    assert!(r.is_error);
    assert_eq!(field_of(&r), "overlay_routing_files[1]", "{}", text(&r));
    assert!(text(&r).contains("bad.json"), "{}", text(&r));
    assert!(!Path::new(&f.path("x.png")).exists());

    let r = board_render::handle_render_board_tiles(
        &json!({ "board": f.board(), "layers": ["In2.Cu"], "px_per_mm": 2, "tile_mm": 30,
                 "overlay_routing_files": [one, two, three], "output_dir": f.path("tiles") }),
        &ctx(),
    )
    .await
    .unwrap();
    assert!(!r.is_error, "{}", text(&r));
    assert!(body(&r)["tiles_written"].as_u64().unwrap() >= 1);
}
