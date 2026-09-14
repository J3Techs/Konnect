use super::*;
use crate::mcp::handler::McpHandler;
use crate::mcp::protocol::ToolContent;
use crate::tools::ServerConfig;
use std::sync::Arc;

const SOCKET: &str = include_str!("../../tests/fixtures/socket_kicad10.kicad_mod");
// Retained verbatim from KiCad 10.0.6's stock
// Connector_USB:USB_C_Receptacle_CNCTech_C-ARA1-AK51X.
const OFFSET_PAD: &str = "\t(pad \"SH\" thru_hole oval\n\t\t(at -4.575 2)\n\t\t(size 0.95 2.28)\n\t\t(drill oval 0.55 1.6\n\t\t\t(offset 0 -0.14)\n\t\t)\n\t\t(property pad_prop_mechanical)\n\t\t(layers \"*.Cu\" \"*.Mask\" \"F.Paste\")\n\t\t(remove_unused_layers no)\n\t\t(uuid \"e95309d6-2525-4715-b36a-9f7be26999e7\")\n\t)";

fn config() -> ServerConfig {
    ServerConfig {
        kicad_cli: String::new(),
        kicad_binary: String::new(),
        ipc_address: String::new(),
        project_dir: None,
        jlcpcb_db_path: None,
        auto_load_toolsets: true,
        eager_toolsets: true,
    }
}

fn ctx() -> ToolContext {
    ToolContext::new(config(), Arc::new(crate::router::ToolRouter::new()))
}

fn body(result: &CallToolResult) -> serde_json::Value {
    let ToolContent::Text { text } = &result.content[0] else {
        panic!("expected JSON text")
    };
    serde_json::from_str(text).unwrap()
}

async fn inspect(path: &Path) -> serde_json::Value {
    let result = handle_get_footprint_info(
        &json!({"footprint_path": path, "include_pads": true}),
        &ctx(),
    )
    .await
    .unwrap();
    assert!(!result.is_error, "{:?}", result.content);
    body(&result)
}

fn pad(drill: serde_json::Value) -> serde_json::Value {
    json!({"number": "1", "type": "thru_hole", "shape": "rect", "x": 0, "y": 0, "width": 2, "height": 2.5, "drill": drill})
}

#[tokio::test]
async fn creates_plated_and_unplated_slots_and_reads_local_axes_and_rotation() {
    let temp = tempfile::tempdir().unwrap();
    let path = temp.path().join("Slots.kicad_mod");
    let mut vertical = pad(json!({"shape": "oval", "width": 1.0, "height": 1.3}));
    vertical["rotation"] = json!(90);
    let mut horizontal = pad(json!({"shape": "oval", "width": 1.3, "height": 1.0}));
    horizontal["number"] = json!("");
    horizontal["type"] = json!("np_thru_hole");
    horizontal["x"] = json!(4);
    let mut round = pad(json!(1));
    round["x"] = json!(8);
    let result = handle_create_footprint(
        &json!({"output": path, "name": "Slots", "pads": [vertical, horizontal, round]}),
        &ctx(),
    )
    .await
    .unwrap();
    assert!(!result.is_error, "{:?}", result.content);
    let content = std::fs::read_to_string(&path).unwrap();
    assert_eq!(parse_sexp(&content).unwrap().head(), Some("footprint"));
    assert!(content.contains("(drill oval 1 1.3)"));
    assert!(content.contains("(drill oval 1.3 1)"));
    assert!(content.contains("(drill 1)"));
    let info = inspect(&path).await;
    assert_eq!(info["source"], "file");
    assert_eq!(info["pad_count"], 3);
    let pads = info["pads"].as_array().unwrap();
    assert_eq!(pads.len(), 3);
    assert_eq!(pads[0]["rotation"], 90.0);
    assert_eq!(pads[0]["width"], 2.0);
    assert_eq!(pads[0]["height"], 2.5);
    assert_eq!(
        pads[0]["drill"],
        json!({"shape": "oval", "width": 1.0, "height": 1.3, "offset": {"x": 0.0, "y": 0.0}})
    );
    assert_eq!(pads[1]["type"], "np_thru_hole");
    assert_eq!(pads[1]["drill"]["width"], 1.3);
    assert_eq!(
        pads[2]["number"], "1",
        "duplicate physical pads remain visible"
    );
    assert_eq!(pads[2]["drill"]["shape"], "circle");
}

#[tokio::test]
async fn invalid_drills_refuse_before_creating_or_replacing_files() {
    let temp = tempfile::tempdir().unwrap();
    let path = temp.path().join("Socket.kicad_mod");
    let invalid = [
        json!(0),
        json!(-1),
        json!(true),
        json!(null),
        json!("1"),
        json!({"shape": "oval", "width": 1}),
        json!({"shape": "circle", "width": 1, "height": 1}),
        json!({"shape": "oval", "width": 0, "height": 1}),
        json!({"shape": "oval", "width": 1, "height": -1}),
        json!({"shape": "oval", "width": "1", "height": 1}),
        json!({"shape": "oval", "width": 1, "height": 1, "offset": {"x": 0, "y": 0}}),
    ];
    for drill in invalid {
        std::fs::write(&path, SOCKET).unwrap();
        let result = handle_create_footprint(
            &json!({"output": path, "name": "Slots", "pads": [pad(drill.clone())]}),
            &ctx(),
        )
        .await
        .unwrap();
        assert!(result.is_error, "accepted {drill}");
        assert_eq!(body(&result)["error"]["field"], "pads[0].drill");
        assert_eq!(std::fs::read_to_string(&path).unwrap(), SOCKET);
        let result = handle_edit_footprint_pad(
            &json!({"footprint_path": path, "pad_number": "2", "drill": drill}),
            &ctx(),
        )
        .await
        .unwrap();
        assert!(result.is_error);
        assert_eq!(body(&result)["error"]["field"], "drill");
        assert_eq!(std::fs::read_to_string(&path).unwrap(), SOCKET);
    }
    let new_path = temp.path().join("absent/subdir/Slots.kicad_mod");
    let mut smd = pad(json!({"shape": "oval", "width": 1, "height": 1.3}));
    smd["type"] = json!("smd");
    let result = handle_create_footprint(
        &json!({"output": new_path, "name": "Slots", "pads": [smd]}),
        &ctx(),
    )
    .await
    .unwrap();
    assert!(result.is_error);
    assert!(!new_path.parent().unwrap().exists());
}

#[tokio::test]
async fn editing_drill_shape_preserves_every_unrelated_fixture_byte() {
    let temp = tempfile::tempdir().unwrap();
    let path = temp.path().join("Socket.kicad_mod");
    std::fs::write(&path, SOCKET).unwrap();
    for (request, expected) in [
        (
            json!({"shape": "oval", "width": 1, "height": 1.3}),
            "(drill oval 1 1.3)",
        ),
        (json!(0.8), "(drill 0.8)"),
    ] {
        let result = handle_edit_footprint_pad(
            &json!({"footprint_path": path, "pad_number": "2", "drill": request}),
            &ctx(),
        )
        .await
        .unwrap();
        assert!(!result.is_error, "{:?}", result.content);
        assert_eq!(body(&result)["updated_count"], 1);
        assert_eq!(
            std::fs::read_to_string(&path).unwrap(),
            SOCKET.replace("(drill 1)", expected)
        );
        let info = inspect(&path).await;
        assert_eq!(info["pads"][0]["drill"], serde_json::Value::Null);
        assert_eq!(info["pads"][0]["roundrect_rratio"], 0.2);
        assert_eq!(
            info["pads"][2]["drill"]["shape"],
            if request.is_number() {
                "circle"
            } else {
                "oval"
            }
        );
    }
}

#[tokio::test]
async fn nested_drill_offset_survives_round_and_oval_edits() {
    let temp = tempfile::tempdir().unwrap();
    let path = temp.path().join("Offset.kicad_mod");
    let original = format!("(footprint \"USB_C_Receptacle_CNCTech_C-ARA1-AK51X\"\n\t(version 20260206)\n\t(layer \"F.Cu\")\n{OFFSET_PAD}\n)\n");
    std::fs::write(&path, &original).unwrap();
    for request in [
        json!(0.6),
        json!({"shape": "oval", "width": 0.6, "height": 1.3}),
    ] {
        let result = handle_edit_footprint_pad(
            &json!({"footprint_path": path, "pad_number": "SH", "drill": request}),
            &ctx(),
        )
        .await
        .unwrap();
        assert!(!result.is_error, "{:?}", result.content);
        let content = std::fs::read_to_string(&path).unwrap();
        assert_eq!(parse_sexp(&content).unwrap().head(), Some("footprint"));
        let observed = inspect(&path).await;
        assert_eq!(
            observed["pads"][0]["drill"]["offset"],
            json!({"x": 0.0, "y": -0.14})
        );
        assert_eq!(observed["pads"][0]["x"], -4.575);
        assert!(content.contains("(property pad_prop_mechanical)"));
        assert!(content.contains("(uuid \"e95309d6-2525-4715-b36a-9f7be26999e7\")"));
        let strip_drill = |source: &str| {
            let (start, end) = find_direct_child_blocks(source, "footprint")
                .into_iter()
                .find(|(s, e)| parse_sexp(&source[*s..*e]).unwrap().head() == Some("pad"))
                .unwrap();
            let pad = &source[start..end];
            let (ds, de) = find_direct_child_blocks(pad, "pad")
                .into_iter()
                .find(|(s, e)| parse_sexp(&pad[*s..*e]).unwrap().head() == Some("drill"))
                .unwrap();
            format!("{}{}", &source[..start + ds], &source[start + de..])
        };
        assert_eq!(strip_drill(&content), strip_drill(&original));
    }
}

#[tokio::test]
async fn batch_drill_edit_refuses_atomically_if_a_later_match_is_smd() {
    let temp = tempfile::tempdir().unwrap();
    let path = temp.path().join("Socket.kicad_mod");
    std::fs::write(&path, SOCKET).unwrap();
    let renamed = handle_edit_footprint_pad(
        &json!({"footprint_path": path, "pad_number": "3", "new_number": "2"}),
        &ctx(),
    )
    .await
    .unwrap();
    assert!(!renamed.is_error);
    let before = std::fs::read_to_string(&path).unwrap();
    let mut args = json!({"footprint_path": path, "pad_number": "2", "match_all": true, "drill": {"shape": "oval", "width": 1, "height": 1.3}});
    let result = handle_edit_footprint_pad(&args, &ctx()).await.unwrap();
    assert!(result.is_error);
    assert_eq!(body(&result)["error"]["field"], "drill");
    assert_eq!(std::fs::read_to_string(&path).unwrap(), before);
    args["match_all"] = json!(false);
    let result = handle_edit_footprint_pad(&args, &ctx()).await.unwrap();
    assert!(!result.is_error);
    assert_eq!(body(&result)["updated_count"], 1);
    assert_eq!(
        std::fs::read_to_string(&path).unwrap(),
        before.replace("(drill 1)", "(drill oval 1 1.3)")
    );
}

#[tokio::test]
async fn omitted_drill_and_pad_inspection_preserve_legacy_behavior() {
    let temp = tempfile::tempdir().unwrap();
    let path = temp.path().join("Socket.kicad_mod");
    std::fs::write(&path, SOCKET).unwrap();
    let result = handle_edit_footprint_pad(
        &json!({"footprint_path": path, "pad_number": "2", "new_number": "4"}),
        &ctx(),
    )
    .await
    .unwrap();
    assert!(!result.is_error);
    assert_eq!(
        std::fs::read_to_string(&path).unwrap(),
        SOCKET.replace("(pad \"2\"", "(pad \"4\"")
    );
    let result = handle_get_footprint_info(&json!({"footprint_path": path}), &ctx())
        .await
        .unwrap();
    assert!(!result.is_error);
    assert!(body(&result).get("pads").is_none());
    assert_eq!(body(&result)["pad_count"], 4);
    let result = handle_get_footprint_info(
        &json!({"footprint_path": path, "include_pads": "yes"}),
        &ctx(),
    )
    .await
    .unwrap();
    assert!(result.is_error);
    assert_eq!(body(&result)["error"]["field"], "include_pads");
}

#[tokio::test]
async fn ambiguous_or_malformed_drill_readback_is_an_error_and_edit_refuses() {
    let temp = tempfile::tempdir().unwrap();
    let path = temp.path().join("Socket.kicad_mod");
    for malformed in [
        "(drill 1) (drill 0.8)",
        "(drill oval 1)",
        "(drill 1 (offset 0))",
        "(drill 1 (unknown 2))",
        "(drill 1 (offset NaN 0))",
    ] {
        let input = SOCKET.replace("(drill 1)", malformed);
        std::fs::write(&path, &input).unwrap();
        let result = handle_get_footprint_info(
            &json!({"footprint_path": path, "include_pads": true}),
            &ctx(),
        )
        .await
        .unwrap();
        assert!(result.is_error, "accepted {malformed}");
        assert_eq!(body(&result)["error"]["field"], "footprint_path");
        let result = handle_edit_footprint_pad(
            &json!({"footprint_path": path, "pad_number": "2", "drill": 0.9}),
            &ctx(),
        )
        .await
        .unwrap();
        assert!(result.is_error, "edited {malformed}");
        assert_eq!(std::fs::read_to_string(&path).unwrap(), input);
    }
}

#[tokio::test]
async fn served_mcp_accepts_slots_and_rejects_malformed_nested_drills() {
    let handler = McpHandler::new(config()).await.unwrap();
    let temp = tempfile::tempdir().unwrap();
    let path = temp.path().join("Slot.kicad_mod");
    let call = |name: &str, arguments: serde_json::Value| json!({"jsonrpc": "2.0", "id": 1, "method": "tools/call", "params": {"name": name, "arguments": arguments}});
    let response = handler.handle_message(call("create_footprint", json!({"output": path, "name": "Slot", "pads": [pad(json!({"shape": "oval", "width": 1, "height": 1.3}))]}))).await.unwrap();
    let result: CallToolResult = serde_json::from_value(response.result.unwrap()).unwrap();
    assert!(!result.is_error, "{:?}", result.content);
    let before = std::fs::read_to_string(&path).unwrap();
    let response = handler.handle_message(call("edit_footprint_pad", json!({"footprint_path": path, "pad_number": "1", "drill": {"shape": "oval", "width": 1}}))).await.unwrap();
    let result: CallToolResult = serde_json::from_value(response.result.unwrap()).unwrap();
    assert!(result.is_error);
    assert_eq!(body(&result)["error"]["kind"], "invalid_argument");
    assert!(body(&result)["error"]["field"]
        .as_str()
        .unwrap()
        .starts_with("drill"));
    assert_eq!(std::fs::read_to_string(&path).unwrap(), before);
    let response = handler
        .handle_message(call(
            "get_footprint_info",
            json!({"footprint_path": path, "include_pads": true}),
        ))
        .await
        .unwrap();
    let result: CallToolResult = serde_json::from_value(response.result.unwrap()).unwrap();
    assert!(!result.is_error);
    assert_eq!(body(&result)["pads"][0]["drill"]["height"], 1.3);
}
