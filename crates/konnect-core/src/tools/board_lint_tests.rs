//! Tests of `lint_board_geometry` and `plan_dogleg_fixes` on hand-written
//! KiCad 10 boards. The two dogleg shapes are those of a current-sense net
//! running into its vias, one on F.Cu from a pad and one on B.Cu along a 45
//! degree run; the expected geometry is worked out by hand from the board
//! text, not taken from the code.

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
              "via_diameter": 0.45, "via_drill": 0.2 }
        ],
        "netclass_patterns": []
    }
}"#;

const CS: &str = "Net-(U1-CS+)";

fn board_with(body: &str) -> String {
    format!(
        r#"(kicad_pcb
	(version 20260206)
	(generator "pcbnew")
	(general (thickness 1.6))
	(layers
		(0 "F.Cu" signal)
		(4 "In1.Cu" power)
		(6 "In2.Cu" signal)
		(2 "B.Cu" signal)
		(25 "Edge.Cuts" user)
	)
	(footprint "SOIC"
		(layer "F.Cu")
		(uuid "fp-u1")
		(at 35.05 17.2)
		(property "Reference" "U1" (at 0 -2 0) (layer "F.SilkS"))
		(pad "18" smd rect (at 0 0) (size 0.3 0.8) (layers "F.Cu" "F.Mask") (net "{CS}") (uuid "pad-18"))
	)
	(gr_rect (start 20 10) (end 50 50) (stroke (width 0.05) (type default)) (fill no) (layer "Edge.Cuts"))
{body}
)
"#
    )
}

fn seg(uuid: &str, layer: &str, net: &str, a: (f64, f64), b: (f64, f64)) -> String {
    format!(
        "\t(segment (start {} {}) (end {} {}) (width 0.2) (locked yes) (layer \"{layer}\") (net \"{net}\") (uuid \"{uuid}\"))\n",
        a.0, a.1, b.0, b.1
    )
}

fn via(uuid: &str, net: &str, at: (f64, f64)) -> String {
    format!(
        "\t(via (at {} {}) (size 0.45) (drill 0.2) (layers \"F.Cu\" \"B.Cu\") (locked yes) (net \"{net}\") (uuid \"{uuid}\"))\n",
        at.0, at.1
    )
}

/// The CS+ net's two doglegs: on F.Cu from pad U1.18 into a via whose
/// In2.Cu track leaves along y = 20.2, and on B.Cu along a 45 degree run
/// into a via whose In2.Cu track leaves down.
fn cs_plus() -> String {
    let mut s = String::new();
    s += &seg("f-long", "F.Cu", CS, (35.05, 17.2), (35.05, 19.85));
    s += &seg("f-short", "F.Cu", CS, (35.05, 19.85), (35.4, 20.2));
    s += &via("via-1", CS, (35.4, 20.2));
    s += &seg("in2-a", "In2.Cu", CS, (30.7, 20.2), (35.4, 20.2));
    s += &seg("b-long", "B.Cu", CS, (38.15, 42.2), (37.35, 43.0));
    s += &seg("b-short", "B.Cu", CS, (37.35, 43.0), (37.1, 43.0));
    s += &via("via-2", CS, (37.1, 43.0));
    s += &seg("in2-b", "In2.Cu", CS, (37.1, 43.0), (37.1, 44.6));
    s
}

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

fn files_with(body: &str) -> Files {
    let dir = tempfile::tempdir().unwrap();
    let root = dir.path().to_path_buf();
    std::fs::write(root.join("demo.kicad_pcb"), board_with(body)).unwrap();
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

async fn lint(f: &Files, extra: Value) -> Value {
    let r = handle_lint_board_geometry(&with(json!({ "board": f.board() }), extra), &ctx())
        .await
        .unwrap();
    assert!(!r.is_error, "{}", text(&r));
    body(&r)
}

async fn plan(f: &Files, extra: Value) -> CallToolResult {
    handle_plan_dogleg_fixes(
        &with(
            json!({ "board": f.board(), "output_plan": f.path("fix.json") }),
            extra,
        ),
        &ctx(),
    )
    .await
    .unwrap()
}

fn near_eq(a: &Value, b: f64) -> bool {
    a.as_f64().is_some_and(|a| (a - b).abs() < 1e-4)
}

fn pt_eq(a: &Value, x: f64, y: f64) -> bool {
    near_eq(&a[0], x) && near_eq(&a[1], y)
}

// ─── lint_board_geometry ─────────────────────────────────────────────────────

#[tokio::test]
async fn both_cs_plus_doglegs_are_found_with_their_numbers() {
    let f = files_with(&cs_plus());
    let b = lint(&f, json!({})).await;
    assert_eq!(b["by_kind"]["via_dogleg"], 2, "{b}");
    assert_eq!(b["defects"], 2, "nothing else is wrong: {b}");
    assert_eq!(b["by_layer"], json!({ "F.Cu": 1, "B.Cu": 1 }));
    assert_eq!(b["by_net"], json!([{ "net": CS, "count": 2 }]));
    let found = b["findings"].as_array().unwrap();
    let front = found.iter().find(|x| x["layer"] == "F.Cu").unwrap();
    assert!(near_eq(&front["x"], 35.4) && near_eq(&front["y"], 20.2));
    assert_eq!(front["vias"][0]["uuid"], "via-1");
    assert_eq!(front["tracks"][0]["uuid"], "f-short");
    assert_eq!(front["tracks"][0]["role"], "short");
    assert!(near_eq(&front["tracks"][0]["length_mm"], 0.495));
    assert_eq!(front["tracks"][1]["uuid"], "f-long");
    assert!(near_eq(&front["tracks"][1]["length_mm"], 2.65));
    assert!(near_eq(&front["interior_angles_deg"][0], 135.0), "{front}");
    assert!(near_eq(&front["offset_mm"], 0.35), "{front}");
    assert!(pt_eq(&front["joint"], 35.05, 19.85));

    let back = found.iter().find(|x| x["layer"] == "B.Cu").unwrap();
    assert_eq!(back["vias"][0]["uuid"], "via-2");
    assert!(near_eq(&back["tracks"][0]["length_mm"], 0.25));
    assert!(near_eq(&back["interior_angles_deg"][0], 135.0), "{back}");
    // 0.25 mm sideways from a 45 degree axis: 0.25 / sqrt(2).
    assert!(near_eq(&back["offset_mm"], 0.1768), "{back}");
}

#[tokio::test]
async fn filters_narrow_the_findings_and_markers_are_written() {
    let f = files_with(&cs_plus());
    let only_back = lint(&f, json!({ "layers": ["B.Cu"] })).await;
    assert_eq!(only_back["matched"], 1);
    let boxed = lint(
        &f,
        json!({ "area": { "x1": 30, "y1": 15, "x2": 40, "y2": 25 } }),
    )
    .await;
    assert_eq!(boxed["matched"], 1);
    assert_eq!(boxed["findings"][0]["layer"], "F.Cu");
    let unlocked = lint(&f, json!({ "locked": false })).await;
    assert_eq!(unlocked["matched"], 0, "every item is locked");
    let none = lint(&f, json!({ "kinds": ["acute_bend"] })).await;
    assert_eq!(none["matched"], 0);

    let markers = f.path("m.json");
    let b = lint(&f, json!({ "markers_output": markers })).await;
    assert_eq!(b["markers_written"], 2);
    let written: Value = serde_json::from_str(&std::fs::read_to_string(&markers).unwrap()).unwrap();
    assert_eq!(written[0]["kind"], "via_dogleg");
    assert_eq!(written[0]["label"], format!("via_dogleg {CS}"));
    let again = handle_lint_board_geometry(
        &json!({ "board": f.board(), "markers_output": markers }),
        &ctx(),
    )
    .await
    .unwrap();
    assert!(again.is_error && text(&again).contains("never replaced"));

    for (extra, field) in [
        (json!({ "nets": ["NOPE"] }), "nets"),
        (json!({ "layers": ["In9.Cu"] }), "layers"),
        (json!({ "kinds": ["wiggle"] }), "kinds"),
        (json!({ "dogleg_length_mm": 0 }), "dogleg_length_mm"),
        (json!({ "min_bend_angle_deg": 270 }), "min_bend_angle_deg"),
    ] {
        let r =
            handle_lint_board_geometry(&with(json!({ "board": f.board() }), extra.clone()), &ctx())
                .await
                .unwrap();
        assert!(r.is_error, "{extra}");
        assert_eq!(body(&r)["error"]["field"], field, "{}", text(&r));
    }
}

#[tokio::test]
async fn a_straight_track_into_a_via_on_its_axis_is_clean() {
    let mut s = String::new();
    s += &seg("t1", "F.Cu", "SIG", (25.0, 15.0), (30.0, 15.0));
    s += &via("v1", "SIG", (30.0, 15.0));
    s += &seg("t2", "In2.Cu", "SIG", (30.0, 15.0), (30.0, 19.0));
    // A straight run split at a via is the connection, not a stray vertex.
    s += &seg("t3", "In2.Cu", "SIG", (30.0, 19.0), (30.0, 25.0));
    s += &via("v2", "SIG", (30.0, 19.0));
    let f = files_with(&s);
    let b = lint(&f, json!({})).await;
    assert_eq!(b["matched"], 0, "{b}");
    assert_eq!(b["clean"], true);
    assert_eq!(
        b["checked"]["collinear_vertices_on_vias_or_pads_skipped"],
        1
    );
    assert_eq!(b["checked"]["tracks"], 3);
    let r = plan(&f, json!({})).await;
    assert!(!r.is_error, "{}", text(&r));
    let p = body(&r);
    assert_eq!(
        (p["doglegs"].as_u64(), p["plan_written"].as_bool()),
        (Some(0), Some(false))
    );
    assert!(!Path::new(&f.path("fix.json")).exists());
}

#[tokio::test]
async fn an_acute_bend_is_reported_with_its_angle_and_tracks() {
    let mut s = String::new();
    s += &seg("a", "F.Cu", "SIG", (25.0, 15.0), (30.0, 15.0));
    s += &seg("b", "F.Cu", "SIG", (30.0, 15.0), (26.0, 17.0));
    let f = files_with(&s);
    let b = lint(&f, json!({})).await;
    assert_eq!(b["by_kind"]["acute_bend"], 1, "{b}");
    let bend = &b["findings"][0];
    assert!(near_eq(&bend["x"], 30.0) && near_eq(&bend["y"], 15.0));
    let uuids: Vec<&str> = bend["tracks"]
        .as_array()
        .unwrap()
        .iter()
        .map(|t| t["uuid"].as_str().unwrap())
        .collect();
    assert_eq!(uuids, ["a", "b"]);
    // atan(2 / 4) = 26.57 degrees between the two.
    assert!(near_eq(&bend["interior_angles_deg"][0], 26.57), "{bend}");
}

// ─── plan_dogleg_fixes ───────────────────────────────────────────────────────

fn track_eq(t: &Value, layer: &str, a: (f64, f64), b: (f64, f64)) -> bool {
    t["layer"] == layer
        && near_eq(&t["x1"], a.0)
        && near_eq(&t["y1"], a.1)
        && near_eq(&t["x2"], b.0)
        && near_eq(&t["y2"], b.1)
}

#[tokio::test]
async fn the_cs_plus_doglegs_are_fixed_to_the_exact_ideal_geometry() {
    let f = files_with(&cs_plus());
    let r = plan(&f, json!({})).await;
    assert!(!r.is_error, "{}", text(&r));
    let b = body(&r);
    assert_eq!(b["fixable_vias"], 2, "{b}");
    assert_eq!(b["not_fixable"], 0, "{b}");
    assert_eq!(b["check"]["refused_by_check"], 0);
    assert_eq!(
        b["deleted_locked"], 8,
        "two vias, two shorts, four re-ended tracks"
    );

    let routing: Value =
        serde_json::from_str(&std::fs::read_to_string(f.path("fix.json")).unwrap()).unwrap();
    let tracks = routing["tracks"].as_array().unwrap();
    let vias = routing["vias"].as_array().unwrap();
    assert_eq!((tracks.len(), vias.len()), (4, 2), "{routing}");
    // F.Cu: the via goes onto the pad's column, where In2.Cu's run already lies.
    assert!(
        near_eq(&vias[0]["x"], 35.05) && near_eq(&vias[0]["y"], 20.2),
        "{routing}"
    );
    assert!(
        track_eq(&tracks[0], "F.Cu", (35.05, 17.2), (35.05, 20.2)),
        "{routing}"
    );
    assert!(
        track_eq(&tracks[1], "In2.Cu", (30.7, 20.2), (35.05, 20.2)),
        "{routing}"
    );
    // B.Cu: the diagonal's axis meets the In2.Cu run's axis at (37.1, 43.25).
    assert!(
        near_eq(&vias[1]["x"], 37.1) && near_eq(&vias[1]["y"], 43.25),
        "{routing}"
    );
    assert!(
        track_eq(&tracks[2], "B.Cu", (38.15, 42.2), (37.1, 43.25)),
        "{routing}"
    );
    assert!(
        track_eq(&tracks[3], "In2.Cu", (37.1, 43.25), (37.1, 44.6)),
        "{routing}"
    );
    assert!(near_eq(&vias[0]["diameter"], 0.45) && near_eq(&vias[0]["drill"], 0.2));
    assert_eq!(tracks[0]["width"], 0.2);

    let fixes: Value =
        serde_json::from_str(&std::fs::read_to_string(f.path("fix.fixes.json")).unwrap()).unwrap();
    let mut deleted: Vec<&str> = fixes["delete_uuids"]
        .as_array()
        .unwrap()
        .iter()
        .map(|u| u.as_str().unwrap())
        .collect();
    deleted.sort_unstable();
    assert_eq!(
        deleted,
        ["b-long", "b-short", "f-long", "f-short", "in2-a", "in2-b", "via-1", "via-2"]
    );
    let first = &fixes["fixes"][0];
    assert_eq!(first["via_uuid"], "via-1");
    assert!(pt_eq(&first["old"], 35.4, 20.2) && pt_eq(&first["new"], 35.05, 20.2));
    assert_eq!(first["basis"], "intersection");
    assert_eq!(first["check"]["result"], "passed");

    // The written routing_file passes the independent rule check too.
    let c = crate::tools::route_native::handle_check_route_rules(
        &json!({ "board": f.board(), "routing_file": f.path("fix.json") }),
        &ctx(),
    )
    .await
    .unwrap();
    assert!(!c.is_error, "{}", text(&c));

    // Never replaced.
    let again = plan(&f, json!({})).await;
    assert!(again.is_error && text(&again).contains("never replaced"));
}

#[tokio::test]
async fn a_via_whose_other_tracks_disagree_on_one_point_is_not_fixable() {
    let mut s = String::new();
    // F.Cu dogleg: along y = 25, then 0.28 mm diagonally up to the via.
    s += &seg("long", "F.Cu", "SIG", (25.0, 25.0), (29.8, 25.0));
    s += &seg("short", "F.Cu", "SIG", (29.8, 25.0), (30.0, 25.2));
    s += &via("v", "SIG", (30.0, 25.2));
    // In2.Cu leaves along x = 30 (meeting y = 25 at (30, 25)) but B.Cu
    // leaves along y = 25.2, which never meets y = 25.
    s += &seg("in2", "In2.Cu", "SIG", (30.0, 25.2), (30.0, 30.0));
    s += &seg("back", "B.Cu", "SIG", (30.0, 25.2), (35.0, 25.2));
    let f = files_with(&s);
    let lint_body = lint(&f, json!({})).await;
    assert_eq!(lint_body["by_kind"]["via_dogleg"], 1, "{lint_body}");
    let r = plan(&f, json!({})).await;
    assert!(!r.is_error, "{}", text(&r));
    let b = body(&r);
    assert_eq!(b["fixable_vias"], 0, "{b}");
    assert_eq!(
        b["not_fixable_by_reason"],
        json!({ "inconsistent_axes": 1 })
    );
    let refusal = &b["refusals"][0];
    assert_eq!(refusal["end"]["via_uuid"], "v");
    assert!(
        refusal["detail"].as_str().unwrap().contains("back"),
        "the refusal names the track that disagrees: {refusal}"
    );
    assert_eq!(b["plan_written"], false);
    assert!(!Path::new(&f.path("fix.json")).exists());
}

#[tokio::test]
async fn a_fix_whose_new_via_crowds_another_net_is_refused_by_the_check() {
    let mut s = cs_plus();
    // Another net's F.Cu track 0.45 mm left of the new F.Cu via position:
    // clear of the old via and tracks, too close to the new via.
    s += &seg("other", "F.Cu", "OTHER", (34.6, 19.9), (34.6, 20.5));
    let f = files_with(&s);
    let r = plan(&f, json!({})).await;
    assert!(!r.is_error, "{}", text(&r));
    let b = body(&r);
    assert_eq!(b["fixable_vias"], 1, "{b}");
    assert_eq!(b["check"]["refused_by_check"], 1, "{b}");
    let refusal = &b["refusals"][0];
    assert_eq!(refusal["reason"], "check_failed");
    assert_eq!(refusal["end"]["via_uuid"], "via-1");
    assert_eq!(refusal["detail"][0]["kind"], "clearance", "{refusal}");
    assert!(
        refusal["detail"][0]["against"]
            .as_str()
            .unwrap()
            .contains("other"),
        "{refusal}"
    );
    // The B.Cu fix stands on its own.
    let routing: Value =
        serde_json::from_str(&std::fs::read_to_string(f.path("fix.json")).unwrap()).unwrap();
    assert_eq!(routing["vias"].as_array().unwrap().len(), 1);
    assert!(near_eq(&routing["vias"][0]["y"], 43.25));
}

#[tokio::test]
async fn of_two_fixes_that_collide_only_with_each_other_the_first_is_kept() {
    // A CS- neighbour 0.5 mm left of CS+: its dogleg runs into a via whose
    // In2.Cu track leaves along y = 19.3. Fixed, its via would sit on
    // x = 34.55, 0.175 mm from CS+'s F.Cu run on x = 35.05 (the rules want
    // 0.2): the two fixes cannot both be made.
    let mut s = cs_plus();
    s += &seg("m-long", "F.Cu", "CS-", (34.55, 17.2), (34.55, 18.95));
    s += &seg("m-short", "F.Cu", "CS-", (34.55, 18.95), (34.2, 19.3));
    s += &via("via-3", "CS-", (34.2, 19.3));
    s += &seg("m-in2", "In2.Cu", "CS-", (30.0, 19.3), (34.2, 19.3));
    let f = files_with(&s);
    let r = plan(&f, json!({})).await;
    assert!(!r.is_error, "{}", text(&r));
    let b = body(&r);
    assert_eq!(b["fixable_vias"], 2, "{b}");
    assert_eq!(b["check"]["set_aside_and_retried"], 1, "{b}");
    assert_eq!(
        b["not_fixable_by_reason"],
        json!({ "check_failed": 1 }),
        "{b}"
    );
    let refusal = &b["refusals"][0];
    assert_eq!(refusal["end"]["via_uuid"], "via-3");
    let against = refusal["detail"][0]["against"].as_str().unwrap();
    assert!(
        against.contains("via-1") && against.contains("35.05"),
        "the refusal names the fix it collides with: {refusal}"
    );
    // Both CS+ fixes keep their ideal geometry.
    let routing: Value =
        serde_json::from_str(&std::fs::read_to_string(f.path("fix.json")).unwrap()).unwrap();
    let vias = routing["vias"].as_array().unwrap();
    assert_eq!(vias.len(), 2);
    assert!(near_eq(&vias[0]["x"], 35.05) && near_eq(&vias[0]["y"], 20.2));
    assert!(near_eq(&vias[1]["x"], 37.1) && near_eq(&vias[1]["y"], 43.25));
}

#[tokio::test]
async fn a_short_segment_into_a_pad_is_reported_and_never_fixed() {
    let mut s = String::new();
    // Along x = 34.85 from above, then 0.28 mm diagonally into pad 18.
    s += &seg("run", "F.Cu", CS, (34.85, 12.0), (34.85, 16.6));
    s += &seg("into", "F.Cu", CS, (34.85, 16.6), (35.05, 16.8));
    let f = files_with(&s);
    let b = lint(&f, json!({ "kinds": ["pad_dogleg"] })).await;
    assert_eq!(b["matched"], 1, "{b}");
    assert_eq!(b["findings"][0]["pads"], json!(["U1.18"]));
    let r = plan(&f, json!({})).await;
    let p = body(&r);
    assert_eq!(
        p["not_fixable_by_reason"],
        json!({ "ends_on_pad": 1 }),
        "{p}"
    );
    assert_eq!(p["refusals"][0]["end"]["pad"], "U1.18");
    assert_eq!(p["plan_written"], false);
}

#[test]
fn fixes_sharing_a_track_are_grouped() {
    let fix = |via, tracks: &[u32]| ViaFix {
        via,
        new_center: Point::new(0, 0),
        basis: "intersection",
        doglegs: vec![],
        shorts: vec![],
        moves: tracks
            .iter()
            .map(|&t| (t, true, Point::new(0, 0)))
            .collect(),
    };
    let groups = clusters(&[fix(0, &[1, 2]), fix(1, &[3]), fix(2, &[2, 4])]);
    assert_eq!(groups, vec![vec![0, 2], vec![1]]);
}
