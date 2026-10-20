//! Tests of the saved-board inspection and planning tools: `query_copper`,
//! `query_pads`, `connectivity_report`'s island listing,
//! `render_board_layers_png`, `plan_tap_vias` and `plan_trunk`, on
//! hand-written KiCad 10 boards. Expected values come from the board text,
//! not from the code under test.

use crate::mcp::handler::McpHandler;
use crate::mcp::protocol::{CallToolResult, ToolContent};
use crate::router::ToolRouter;
use crate::tools::{
    board_inspect, board_render, route_native, route_plan_tools, ServerConfig, ToolContext,
};
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
            { "name": "Default", "clearance": 0.2, "track_width": 0.2,
              "via_diameter": 0.6, "via_drill": 0.3 }
        ],
        "netclass_patterns": []
    }
}"#;

/// A footprint rotated 90 degrees with a custom pad whose copper is a
/// 0.3 x 2.4 mm polygon bar around a 0.16 mm anchor, and a stackup naming
/// In2.Cu's copper thickness.
const CUSTOM_PAD_BOARD: &str = r#"(kicad_pcb
	(version 20260206)
	(generator "pcbnew")
	(layers
		(0 "F.Cu" signal)
		(4 "In1.Cu" signal)
		(6 "In2.Cu" signal)
		(2 "B.Cu" signal)
		(25 "Edge.Cuts" user)
	)
	(setup
		(stackup
			(layer "F.Cu" (type "copper") (thickness 0.035))
			(layer "In2.Cu" (type "copper") (thickness 0.0152))
		)
	)
	(footprint "Bar"
		(layer "F.Cu")
		(at 100 50 90)
		(property "Reference" "U1" (at 0 -3 90) (layer "F.SilkS"))
		(pad "5" smd custom
			(at 0 0 90)
			(size 0.16 0.16)
			(layers "F.Cu" "F.Mask")
			(net "VOUT")
			(options (clearance outline) (anchor circle))
			(primitives
				(gr_poly (pts (xy -0.15 -1.2) (xy 0.15 -1.2) (xy 0.15 1.2) (xy -0.15 1.2)) (width 0) (fill yes))
			)
		)
	)
	(gr_rect (start 80 30) (end 120 70) (stroke (width 0.05) (type default)) (fill no) (layer "Edge.Cuts"))
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

fn files_with(board: &str) -> Files {
    let dir = tempfile::tempdir().unwrap();
    let root = dir.path().to_path_buf();
    std::fs::write(root.join("demo.kicad_pcb"), board).unwrap();
    std::fs::write(root.join("demo.kicad_pro"), PRO).unwrap();
    Files { _dir: dir, root }
}

fn files() -> Files {
    files_with(konnect_route::synthetic::MINIMAL_KICAD10_BOARD)
}

fn with(base: Value, extra: Value) -> Value {
    let mut a = base;
    a.as_object_mut()
        .unwrap()
        .extend(extra.as_object().unwrap().clone());
    a
}

fn field_of(r: &CallToolResult) -> String {
    body(r)["error"]["field"].as_str().unwrap_or("").to_string()
}

// ─── query_copper ────────────────────────────────────────────────────────────

async fn copper(f: &Files, extra: Value) -> CallToolResult {
    board_inspect::handle_query_copper(&with(json!({ "board": f.board() }), extra), &ctx())
        .await
        .unwrap()
}

#[tokio::test]
async fn query_copper_lists_every_kind_with_its_geometry() {
    let f = files();
    let r = copper(&f, json!({})).await;
    assert!(!r.is_error, "{}", text(&r));
    let b = body(&r);
    assert_eq!(b["source"], "saved_file");
    // Two segments, one arc, one via, the GND plane and the no_vias area.
    assert_eq!(b["matched"], 6);
    assert_eq!(
        b["by_kind"],
        json!({ "track": 2, "arc": 1, "via": 1, "zone": 1, "rule_area": 1 })
    );
    let items = b["items"].as_array().unwrap();
    let via = items.iter().find(|i| i["kind"] == "via").unwrap();
    assert_eq!(via["uuid"], "00000000-0000-0000-0000-000000000013");
    assert_eq!(
        (via["x"].as_f64(), via["y"].as_f64()),
        (Some(30.0), Some(25.0))
    );
    assert_eq!(
        (via["diameter_mm"].as_f64(), via["drill_mm"].as_f64()),
        (Some(0.6), Some(0.3))
    );
    assert_eq!(via["layers"].as_array().unwrap().len(), 4);
    assert_eq!(via["through"], true);
    let locked = items
        .iter()
        .find(|i| i["uuid"] == "00000000-0000-0000-0000-000000000010")
        .unwrap();
    assert_eq!(locked["locked"], true);
    assert_eq!(locked["width_mm"].as_f64(), Some(0.2));
    let zone = items.iter().find(|i| i["kind"] == "zone").unwrap();
    assert_eq!(zone["name"], "GND plane");
    assert_eq!(zone["net"], "GND");
    assert_eq!(zone["outline"].as_array().unwrap().len(), 4);
    assert_eq!(zone["holes"].as_array().unwrap().len(), 1);
    assert_eq!(zone["fill"]["state"], "filled");
    assert_eq!(zone["fill"]["pieces"], 1);
    let area = items.iter().find(|i| i["kind"] == "rule_area").unwrap();
    assert_eq!(area["name"], "no_vias");
    assert_eq!(area["forbids"]["vias"], true);
    assert_eq!(area["forbids"]["tracks"], false);
}

#[tokio::test]
async fn query_copper_filters_by_area_net_layer_kind_and_lock() {
    let f = files();
    let count = |r: &CallToolResult| body(r)["matched"].as_u64().unwrap();
    let area = json!({ "x1": 19, "y1": 9, "x2": 27, "y2": 13 });
    // The SIG track and arc, and the GND plane whose outline covers the box.
    assert_eq!(count(&copper(&f, json!({ "area": area })).await), 3);
    // Wholly inside: only the SIG copper.
    let r = copper(&f, json!({ "area": area, "area_mode": "inside" })).await;
    assert_eq!(count(&r), 2, "{}", text(&r));
    assert_eq!(count(&copper(&f, json!({ "nets": ["SIG"] })).await), 2);
    // A pattern: GND's track, via and plane (rule areas carry no net).
    assert_eq!(count(&copper(&f, json!({ "nets": ["G*"] })).await), 3);
    // B.Cu: the GND track, the through via, the all-layer rule area.
    assert_eq!(count(&copper(&f, json!({ "layers": ["B.Cu"] })).await), 3);
    assert_eq!(count(&copper(&f, json!({ "locked": true })).await), 1);
    let r = copper(&f, json!({ "kinds": ["via"], "counts_only": true })).await;
    let b = body(&r);
    assert_eq!(b["by_kind"], json!({ "via": 1 }));
    assert_eq!(b["by_net"], json!({ "GND": 1 }));
    assert!(b.get("items").is_none());
    let r = copper(&f, json!({ "max_items": 1 })).await;
    let b = body(&r);
    assert_eq!(
        (b["returned"].as_u64(), b["truncated"].as_bool()),
        (Some(1), Some(true))
    );
}

#[tokio::test]
async fn query_copper_refuses_names_that_match_nothing() {
    let f = files();
    for (extra, field) in [
        (json!({ "nets": ["NOPE"] }), "nets"),
        (json!({ "layers": ["F.SilkS"] }), "layers"),
        (json!({ "kinds": ["pad"] }), "kinds"),
        (json!({ "area": { "x1": 0, "y1": 0, "x2": 1 } }), "area"),
        (json!({ "area_mode": "near" }), "area_mode"),
        (json!({ "locked": "yes" }), "locked"),
    ] {
        let r = copper(&f, extra.clone()).await;
        assert!(r.is_error, "{extra} accepted");
        assert_eq!(field_of(&r), field, "{}", text(&r));
    }
    let r =
        board_inspect::handle_query_copper(&json!({ "board": f.path("none.kicad_pcb") }), &ctx())
            .await
            .unwrap();
    assert!(r.is_error);
    assert_eq!(body(&r)["error"]["kind"], "file_not_found");
}

/// Through the served dispatch: the schema gate refuses a misspelt
/// argument before the handler runs, and a valid call answers.
#[tokio::test]
async fn served_query_copper_answers_and_refuses_a_typo() {
    let f = files();
    let handler = McpHandler::new(ServerConfig {
        kicad_cli: String::new(),
        kicad_binary: String::new(),
        ipc_address: String::new(),
        project_dir: None,
        jlcpcb_db_path: None,
        auto_load_toolsets: true,
        eager_toolsets: false,
    })
    .await
    .unwrap();
    let call = |args: Value| {
        json!({ "jsonrpc": "2.0", "id": 1, "method": "tools/call",
                "params": { "name": "query_copper", "arguments": args } })
    };
    let ok = handler
        .handle_message(call(json!({ "board": f.board(), "nets": ["SIG"] })))
        .await
        .unwrap()
        .result
        .unwrap();
    assert_eq!(ok["isError"], false, "{ok}");
    let b: Value = serde_json::from_str(ok["content"][0]["text"].as_str().unwrap()).unwrap();
    assert_eq!(b["matched"], 2);
    let typo = handler
        .handle_message(call(json!({ "board": f.board(), "net": ["SIG"] })))
        .await
        .unwrap()
        .result
        .unwrap();
    assert_eq!(typo["isError"], true, "{typo}");
}

// ─── query_pads ──────────────────────────────────────────────────────────────

async fn pads(f: &Files, extra: Value) -> CallToolResult {
    board_inspect::handle_query_pads(&with(json!({ "board": f.board() }), extra), &ctx())
        .await
        .unwrap()
}

#[tokio::test]
async fn query_pads_gives_board_space_outlines_and_drills() {
    let f = files();
    let r = pads(&f, json!({ "references": ["R1"] })).await;
    assert!(!r.is_error, "{}", text(&r));
    let b = body(&r);
    assert_eq!(b["matched"], 2);
    let p1 = &b["pads"][0];
    // R1 at (20, 10) rotated 90: pad 1's local (-0.5, 0) lands at (20, 10.5),
    // and its 0.6 x 0.5 copper turns to 0.5 x 0.6.
    assert_eq!(
        (p1["x"].as_f64(), p1["y"].as_f64()),
        (Some(20.0), Some(10.5))
    );
    assert_eq!(p1["net"], "SIG");
    assert_eq!(p1["kind"], "smd");
    assert_eq!(p1["layers"], json!(["F.Cu"]));
    assert_eq!(
        p1["bbox"],
        json!({ "x1": 19.75, "y1": 10.2, "x2": 20.25, "y2": 10.8 })
    );
    assert_eq!(p1["shapes"][0]["corner_radius_mm"].as_f64(), Some(0.125));

    let r = pads(&f, json!({ "pads": ["J1.1"] })).await;
    let j = &body(&r)["pads"][0];
    assert_eq!(j["kind"], "thru_hole");
    assert_eq!(j["layers"].as_array().unwrap().len(), 4);
    assert_eq!(j["drill"]["shape"], "oblong");
    assert_eq!(j["drill"]["width_mm"].as_f64(), Some(0.6));
    assert_eq!(j["drill"]["length_mm"].as_f64(), Some(1.2));
    assert_eq!(j["drill"]["plated"], true);

    // A box around J1.3 only; a net filter; compact output.
    let r = pads(
        &f,
        json!({ "area": { "x1": 29, "y1": 24, "x2": 31, "y2": 26 } }),
    )
    .await;
    let b = body(&r);
    assert_eq!(b["matched"], 1);
    assert_eq!(b["pads"][0]["number"], "3");
    let r = pads(&f, json!({ "nets": ["D+"], "compact": true })).await;
    let b = body(&r);
    assert_eq!(b["matched"], 1);
    assert!(b["pads"][0].get("shapes").is_none());
    assert!(b["pads"][0]["bbox"].is_object());
}

#[tokio::test]
async fn query_pads_transforms_custom_primitives_into_board_space() {
    let f = files_with(CUSTOM_PAD_BOARD);
    let r = pads(&f, json!({ "pads": ["U1.5"] })).await;
    assert!(!r.is_error, "{}", text(&r));
    let p = &body(&r)["pads"][0];
    // The vertical 0.3 x 2.4 bar turned 90 degrees about (100, 50).
    let bb = &p["bbox"];
    let near = |v: &Value, want: f64| (v.as_f64().unwrap() - want).abs() < 1e-6;
    assert!(near(&bb["x1"], 98.8) && near(&bb["x2"], 101.2), "{bb}");
    assert!(near(&bb["y1"], 49.85) && near(&bb["y2"], 50.15), "{bb}");
    let kinds: Vec<&str> = p["shapes"]
        .as_array()
        .unwrap()
        .iter()
        .map(|s| s["type"].as_str().unwrap())
        .collect();
    assert_eq!(kinds, vec!["circle", "polygon"]);
}

#[tokio::test]
async fn query_pads_refuses_pads_and_references_that_are_not_there() {
    let f = files();
    for (extra, field) in [
        (json!({ "pads": ["R1"] }), "pads"),
        (json!({ "pads": ["R9.1"] }), "pads"),
        (json!({ "pads": ["R1.7"] }), "pads"),
        (json!({ "references": ["X9"] }), "references"),
        (json!({ "pad_kinds": ["round"] }), "pad_kinds"),
    ] {
        let r = pads(&f, extra.clone()).await;
        assert!(r.is_error, "{extra} accepted");
        assert_eq!(field_of(&r), field, "{}", text(&r));
    }
}

// ─── connectivity_report islands ─────────────────────────────────────────────

#[tokio::test]
async fn connectivity_report_names_each_islands_pads_and_copper() {
    let f = files();
    let r = route_native::handle_connectivity_report(
        &json!({ "board": f.board(), "nets": ["GND", "SIG"] }),
        &ctx(),
    )
    .await
    .unwrap();
    assert!(!r.is_error, "{}", text(&r));
    let b = body(&r);
    // The existing totals are still there.
    assert!(b["connections_needed"].as_u64().is_some());
    let gnd = &b["net_islands"][0];
    assert_eq!(gnd["net"], "GND");
    // R1.2 is alone on F.Cu; J1.1, the B.Cu track, the via and the In1
    // plane are joined.
    assert_eq!(gnd["pad_islands"], 2);
    let islands = gnd["islands"].as_array().unwrap();
    let alone = islands
        .iter()
        .find(|i| i["pads"] == json!(["R1.2"]))
        .unwrap();
    assert_eq!(alone["items"], 1);
    let joined = islands
        .iter()
        .find(|i| i["pads"] == json!(["J1.1"]))
        .unwrap();
    assert_eq!(
        (
            joined["tracks"].as_u64(),
            joined["vias"].as_u64(),
            joined["zone_pieces"].as_u64()
        ),
        (Some(1), Some(1), Some(1))
    );
    assert_eq!(joined["zones"], json!(["GND plane"]));
    let sig = &b["net_islands"][1];
    assert_eq!(sig["islands"][0]["pads"], json!(["R1.1"]));
    assert_eq!(
        (
            sig["islands"][0]["tracks"].as_u64(),
            sig["islands"][0]["arcs"].as_u64()
        ),
        (Some(1), Some(1))
    );

    // Without nets, the listing is absent and nothing else changes.
    let r = route_native::handle_connectivity_report(&json!({ "board": f.board() }), &ctx())
        .await
        .unwrap();
    assert!(body(&r)["net_islands"].is_null());
    let r = route_native::handle_connectivity_report(
        &json!({ "board": f.board(), "nets": ["NOPE"] }),
        &ctx(),
    )
    .await
    .unwrap();
    assert_eq!(field_of(&r), "nets");
}

// ─── render_board_layers_png ─────────────────────────────────────────────────

async fn render(f: &Files, extra: Value) -> CallToolResult {
    board_render::handle_render_board_layers_png(
        &with(json!({ "board": f.board() }), extra),
        &ctx(),
    )
    .await
    .unwrap()
}

#[tokio::test]
async fn render_draws_the_window_and_reports_what_it_drew() {
    let f = files();
    let out = f.path("front.png");
    let r = render(
        &f,
        json!({ "layers": ["F.Cu"], "px_per_mm": 10, "output": out, "inline": true }),
    )
    .await;
    assert!(!r.is_error, "{}", text(&r));
    let b = body(&r);
    // The outline is 50 x 40 mm; the default window adds 0.5 mm all round.
    assert_eq!(
        (b["width_px"].as_u64(), b["height_px"].as_u64()),
        (Some(510), Some(410))
    );
    assert_eq!(b["drawn"]["tracks"], 1);
    assert_eq!(b["drawn"]["arcs"], 1);
    assert_eq!(b["drawn"]["vias"], 1);
    // R1's two pads and J1's three through-hole pads have F.Cu copper.
    assert_eq!(b["drawn"]["pads"], 5);
    let png = std::fs::read(&out).unwrap();
    let (w, h, rgba) = konnect_render::png_rgba(&png).unwrap();
    assert_eq!((w, h), (510, 410));
    // R1 pad 1's centre (20, 10.5) is copper, not the dark ground.
    let (x, y) = (
        ((20.0 + 0.5) * 10.0) as usize,
        ((10.5 + 0.5) * 10.0) as usize,
    );
    let at = (y * w as usize + x) * 4;
    let px = &rgba[at..at + 4];
    assert!(px[0] > 60 || px[1] > 60 || px[2] > 60, "{px:?}");
    assert!(r
        .content
        .iter()
        .any(|c| matches!(c, ToolContent::Image { mime_type, .. } if mime_type == "image/png")));
}

#[tokio::test]
async fn render_overlays_a_plan_and_refuses_bad_windows_and_files() {
    let f = files();
    let plan = f.path("plan.json");
    std::fs::write(
        &plan,
        r#"{"tracks":[{"net":"SIG","layer":"F.Cu","width":0.3,"x1":5,"y1":5,"x2":15,"y2":5}],
            "vias":[{"net":"SIG","x":15,"y":5,"drill":0.3,"diameter":0.6}]}"#,
    )
    .unwrap();
    let zones = f.path("zones.json");
    std::fs::write(
        &zones,
        r#"[{"net_name":"GND","layer":"F.Cu","name":"pour","points":[{"x":2,"y":20},{"x":8,"y":20},{"x":8,"y":26}]}]"#,
    )
    .unwrap();
    let r = render(
        &f,
        json!({ "area": { "x1": 0, "y1": 0, "x2": 25, "y2": 30 }, "px_per_mm": 8,
                "overlay_routing_file": plan, "overlay_zones_file": zones,
                "labels": ["references", "zones", "pad_numbers", "nets"], "output": f.path("o.png") }),
    )
    .await;
    assert!(!r.is_error, "{}", text(&r));
    let d = &body(&r)["drawn"];
    assert_eq!(
        (
            d["overlay_tracks"].as_u64(),
            d["overlay_vias"].as_u64(),
            d["overlay_zones"].as_u64()
        ),
        (Some(1), Some(1), Some(1))
    );
    assert!(d["labels"].as_u64().unwrap() >= 2);

    let bad_plan = f.path("bad.json");
    std::fs::write(&bad_plan, r#"{"segments":[]}"#).unwrap();
    for (extra, field) in [
        (json!({ "px_per_mm": 300 }), "px_per_mm"),
        (json!({ "layers": ["Nope.Cu"] }), "layers"),
        (json!({ "grid_mm": 0.01 }), "grid_mm"),
        (json!({ "labels": ["everything"] }), "labels"),
        (
            json!({ "overlay_routing_file": bad_plan }),
            "overlay_routing_file",
        ),
        (json!({ "overlay_zones_file": f.path("missing.json") }), ""),
    ] {
        let r = render(
            &f,
            with(json!({ "output": f.path("x.png") }), extra.clone()),
        )
        .await;
        assert!(r.is_error, "{extra} accepted");
        if !field.is_empty() {
            assert_eq!(field_of(&r), field, "{}", text(&r));
        }
    }
    assert!(
        !Path::new(&f.path("x.png")).exists(),
        "a refusal writes nothing"
    );
}

// ─── plan_tap_vias ───────────────────────────────────────────────────────────

#[tokio::test]
async fn plan_tap_vias_writes_an_anchored_via_per_pad_that_checks_clean() {
    let f = files();
    let out = f.path("taps.json");
    let args = json!({ "board": f.board(), "pads": ["R1.1", "J1.2"], "target_layer": "In2.Cu", "output_plan": out });
    let r = route_plan_tools::handle_plan_tap_vias(&args, &ctx())
        .await
        .unwrap();
    assert!(!r.is_error, "{}", text(&r));
    let b = body(&r);
    assert_eq!(b["failed"], json!([]));
    let sites = b["sites"].as_array().unwrap();
    assert_eq!(sites[0]["pad"], "R1.1");
    assert_eq!(sites[0]["tap"], "via");
    assert_eq!(sites[0]["stub"]["layer"], "F.Cu");
    assert_eq!(
        sites[0]["stub"]["from"],
        json!([20.0, 10.5]),
        "from the pad's copper centre"
    );
    assert_eq!(
        sites[0]["stub"]["to"],
        json!([sites[0]["x"], sites[0]["y"]])
    );
    assert_eq!(sites[1]["pad"], "J1.2");
    assert_eq!(sites[1]["tap"], "through_hole_pad");
    assert_eq!(b["check"]["violations"], 0);
    assert_eq!(b["plan_written"], true);
    // The written file passes the independent rule check tool too.
    let r = route_native::handle_check_route_rules(
        &json!({ "board": f.board(), "routing_file": out }),
        &ctx(),
    )
    .await
    .unwrap();
    let c = body(&r);
    assert_eq!(
        (
            c["violations"].as_u64(),
            c["vias"].as_u64(),
            c["tracks"].as_u64()
        ),
        (Some(0), Some(1), Some(1)),
        "{c}"
    );

    // Never replaced; unknown names refused.
    let r = route_plan_tools::handle_plan_tap_vias(&args, &ctx())
        .await
        .unwrap();
    assert!(r.is_error && text(&r).contains("never replaced"));
    for (extra, field) in [
        (json!({ "pads": ["R1.9"] }), "pads"),
        (json!({ "target_layer": "In9.Cu" }), "target_layer"),
    ] {
        let a = with(
            args.clone(),
            with(
                json!({ "output_plan": f.path("other.json") }),
                extra.clone(),
            ),
        );
        let r = route_plan_tools::handle_plan_tap_vias(&a, &ctx())
            .await
            .unwrap();
        assert!(r.is_error, "{extra}");
        assert_eq!(field_of(&r), field, "{}", text(&r));
        assert!(!Path::new(&f.path("other.json")).exists());
    }
}

// ─── plan_trunk ──────────────────────────────────────────────────────────────

#[tokio::test]
async fn plan_trunk_routes_exact_legs_around_foreign_pads_and_checks_clean() {
    let f = files();
    let out = f.path("trunk.json");
    // From the left edge to the right of J1, whose GND and D- pads and the
    // GND via sit across the straight line.
    let args = json!({
        "board": f.board(), "net": "D+", "layer": "In2.Cu", "output_plan": out,
        "waypoints": [{ "x": 5.003, "y": 23.517 }, { "x": 44.0, "y": 22.0 }, { "x": 44.0, "y": 8.0 }],
        "preferred_width_mm": 1.0, "min_width_mm": 0.2,
        "leg_widths": [{ "leg": 1, "width_mm": 0.8 }],
        "keepouts": [{ "x1": 10, "y1": 18, "x2": 12, "y2": 40 }]
    });
    let r = route_plan_tools::handle_plan_trunk(&args, &ctx())
        .await
        .unwrap();
    assert!(!r.is_error, "{}", text(&r));
    let b = body(&r);
    assert_eq!(b["complete"], true, "{b}");
    assert_eq!(b["check"]["violations"], 0);
    let legs = b["legs"].as_array().unwrap();
    assert_eq!(legs.len(), 2);
    let first = &legs[0]["width_profile"];
    assert_eq!(
        first[0]["from"],
        json!([5.003, 23.517]),
        "starts exactly on the waypoint"
    );
    let last = legs[1]["width_profile"]
        .as_array()
        .unwrap()
        .last()
        .unwrap()
        .clone();
    assert_eq!(
        last["to"],
        json!([44.0, 8.0]),
        "ends exactly on the waypoint"
    );
    assert!(legs[1]["max_width_used_mm"].as_f64().unwrap() <= 0.8);
    assert!(legs[0]["resistance_mohm"].as_f64().unwrap() > 0.0);
    assert!(b["copper"]["thickness_source"]
        .as_str()
        .unwrap()
        .starts_with("assumed"));
    assert_eq!(b["vias"].as_array().unwrap().len(), 3);
    // Independent check of the written file.
    let r = route_native::handle_check_route_rules(
        &json!({ "board": f.board(), "routing_file": out }),
        &ctx(),
    )
    .await
    .unwrap();
    assert_eq!(body(&r)["violations"], 0, "{}", text(&r));
    // The keep-out box is never entered.
    let plan: Value = serde_json::from_str(&std::fs::read_to_string(&out).unwrap()).unwrap();
    for t in plan["tracks"].as_array().unwrap() {
        let (x1, x2) = (t["x1"].as_f64().unwrap(), t["x2"].as_f64().unwrap());
        let (y1, y2) = (t["y1"].as_f64().unwrap(), t["y2"].as_f64().unwrap());
        let w = t["width"].as_f64().unwrap() / 2.0;
        let inside_x = x1.min(x2) - w < 12.0 && x1.max(x2) + w > 10.0;
        let inside_y = y1.min(y2) - w < 40.0 && y1.max(y2) + w > 18.0;
        assert!(!(inside_x && inside_y && (x1 - x2).abs() < 1e-9), "{t}");
    }
}

#[tokio::test]
async fn plan_trunk_reads_the_stackup_and_avoids_planned_copper() {
    // A GND segment puts the net on the board; the planned VOUT track
    // across the middle is another net's copper the GND trunk must avoid.
    let board = CUSTOM_PAD_BOARD.replace(
        "\t(gr_rect",
        "\t(segment (start 85 32) (end 86 32) (width 0.2) (layer \"B.Cu\") (net \"GND\"))\n\t(gr_rect",
    );
    let f = files_with(&board);
    std::fs::write(
        f.path("avoid.json"),
        r#"{"tracks":[{"net":"VOUT","layer":"In2.Cu","width":0.5,"x1":95,"y1":35,"x2":95,"y2":65}],"vias":[]}"#,
    )
    .unwrap();
    let out = f.path("gnd.json");
    let args = json!({
        "board": f.board(), "net": "GND", "layer": "In2.Cu", "output_plan": out,
        "waypoints": [{ "x": 90.0, "y": 50.0 }, { "x": 110.0, "y": 50.0 }],
        "preferred_width_mm": 1.0, "min_width_mm": 0.3,
        "avoid_routing_files": [f.path("avoid.json")]
    });
    let r = route_plan_tools::handle_plan_trunk(&args, &ctx())
        .await
        .unwrap();
    assert!(!r.is_error, "{}", text(&r));
    let b = body(&r);
    assert_eq!(b["complete"], true, "{b}");
    assert_eq!(b["copper"]["thickness_mm"].as_f64(), Some(0.0152));
    assert_eq!(b["copper"]["thickness_source"], "board stackup");
    assert_eq!(
        b["check"]["against"],
        "saved board plus the avoided routing files"
    );
    // It cannot cross x = 95 between y 35 and 65: it goes round an end.
    let profile = b["legs"][0]["width_profile"].as_array().unwrap();
    assert!(
        profile.iter().any(|p| {
            let (a, z) = (&p["from"], &p["to"]);
            a[1].as_f64().unwrap() < 35.0
                || a[1].as_f64().unwrap() > 65.0
                || z[1].as_f64().unwrap() < 35.0
                || z[1].as_f64().unwrap() > 65.0
        }),
        "{profile:?}"
    );
}

#[tokio::test]
async fn plan_trunk_refuses_bad_arguments_before_writing() {
    let f = files();
    let base = json!({
        "board": f.board(), "net": "D+", "layer": "In2.Cu", "output_plan": f.path("t.json"),
        "waypoints": [{ "x": 5, "y": 5 }, { "x": 40, "y": 5 }],
        "preferred_width_mm": 1.0
    });
    for (extra, field) in [
        (json!({ "net": "NOPE" }), "net"),
        (json!({ "layer": "In9.Cu" }), "layer"),
        (json!({ "waypoints": [{ "x": 5, "y": 5 }] }), "waypoints"),
        (json!({ "min_width_mm": 0.05 }), "min_width_mm"),
        (json!({ "min_width_mm": 2.0 }), "min_width_mm"),
        (
            json!({ "leg_widths": [{ "leg": 3, "width_mm": 1.0 }] }),
            "leg_widths[0]",
        ),
        (json!({ "keepouts": [{ "x1": 1 }] }), "keepouts[0]"),
        (
            json!({ "avoid_routing_files": [f.path("missing.json")] }),
            "",
        ),
    ] {
        let r = route_plan_tools::handle_plan_trunk(&with(base.clone(), extra.clone()), &ctx())
            .await
            .unwrap();
        assert!(r.is_error, "{extra} accepted");
        if !field.is_empty() {
            assert_eq!(field_of(&r), field, "{}", text(&r));
        }
        assert!(!Path::new(&f.path("t.json")).exists());
    }
}
