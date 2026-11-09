//! `edit_symbol` on KiCad's own `Device:R` (the `reference_prefix_kicad10`
//! fixture, stock KiCad 10.0.6 bytes): pins 1 and 2 at (0, ±3.81) facing the
//! body, 1.27 long, passive; the body rectangle in the common unit 0.

use super::*;
use crate::tools::ServerConfig;
use std::sync::Arc;

const DEVICE: &str = include_str!("../../tests/fixtures/reference_prefix_kicad10/Device.kicad_sym");

fn library() -> (tempfile::TempDir, std::path::PathBuf) {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("Device.kicad_sym");
    std::fs::write(&path, DEVICE).unwrap();
    (dir, path)
}

async fn edit(path: &Path, mut args: Value) -> CallToolResult {
    args["library_path"] = json!(path.display().to_string());
    if args.get("symbol_name").is_none() {
        args["symbol_name"] = json!("R");
    }
    let ctx = ToolContext::new(
        ServerConfig::default(),
        Arc::new(crate::router::ToolRouter::new()),
    );
    (tool().handler)(&args, Arc::new(ctx)).await.unwrap()
}

fn body(result: &CallToolResult) -> Value {
    let crate::mcp::protocol::ToolContent::Text { text } = &result.content[0] else {
        panic!("expected text");
    };
    serde_json::from_str(text).unwrap()
}

/// The edit changes the named tokens and nothing else: the written file is
/// the original with exactly those substrings replaced.
#[tokio::test]
async fn pins_and_the_body_change_in_place_and_nothing_else_does() {
    let (_d, path) = library();
    let result = edit(
        &path,
        json!({
            "pins": [
                { "number": "1", "dy": 1.27, "length": 2.54, "new_name": "A",
                  "electrical_type": "input" },
                { "number": "2", "new_number": "3" }
            ],
            "rectangles": [ { "unit": 0, "move_top": 1.27 } ]
        }),
    )
    .await;
    assert!(!result.is_error, "{result:?}");
    let response = body(&result);
    assert_eq!(response["written"], true);
    assert_eq!(response["pin_changes"][0]["after"]["y"], 5.08);
    assert_eq!(response["rectangle_changes"][0]["after"]["end"]["y"], 3.81);

    let written = std::fs::read_to_string(&path).unwrap();
    let pin1_before = &DEVICE[DEVICE
        .find("(pin passive line\n\t\t\t\t(at 0 3.81 270)")
        .unwrap()..];
    let pin1_before = &pin1_before[..pin1_before.find("\n\t\t\t)").unwrap()];
    let pin1_after = pin1_before
        .replacen("(pin passive line", "(pin input line", 1)
        .replacen("(at 0 3.81 270)", "(at 0 5.08 270)", 1)
        .replacen("(length 1.27)", "(length 2.54)", 1)
        .replacen("(name \"\"", "(name \"A\"", 1);
    let expected = DEVICE
        .replacen(pin1_before, &pin1_after, 1)
        .replacen("(number \"2\"", "(number \"3\"", 1)
        .replacen("(end 1.016 2.54)", "(end 1.016 3.81)", 1);
    assert_eq!(written, expected);
}

/// A dry run reports the same changes and writes nothing.
#[tokio::test]
async fn a_dry_run_reports_without_writing() {
    let (_d, path) = library();
    let result = edit(
        &path,
        json!({ "dry_run": true, "pins": [ { "number": "1", "x": 2.54 } ] }),
    )
    .await;
    assert!(!result.is_error, "{result:?}");
    let response = body(&result);
    assert_eq!(response["written"], false);
    assert_eq!(response["pin_changes"][0]["after"]["x"], 2.54);
    assert_eq!(std::fs::read_to_string(&path).unwrap(), DEVICE);
}

/// Every refusal names its field and leaves the library as it was.
#[tokio::test]
async fn a_wrong_request_is_refused_before_anything_is_written() {
    let cases = [
        (json!({ "pins": [ { "number": "9" } ] }), "pins[0].number"),
        (
            json!({ "pins": [ { "number": "1", "unit": 2 } ] }),
            "pins[0].unit",
        ),
        (
            json!({ "pins": [ { "number": "2", "new_number": "1" } ] }),
            "pins",
        ),
        (
            json!({ "pins": [ { "number": "1", "x": 1.0, "dx": 1.0 } ] }),
            "pins[0].dx",
        ),
        (
            json!({ "pins": [ { "number": "1", "dy": 1.0 }, { "number": "1", "dy": 2.0 } ] }),
            "pins[1].number",
        ),
        (
            json!({ "rectangles": [ { "unit": 0, "index": 1 } ] }),
            "rectangles[0].index",
        ),
        (
            json!({ "rectangles": [ { "unit": 0, "move_top": -5.08 } ] }),
            "rectangles[0].index",
        ),
        (
            json!({ "symbol_name": "C", "pins": [ { "number": "1" } ] }),
            "symbol_name",
        ),
    ];
    for (args, field) in cases {
        let (_d, path) = library();
        let result = edit(&path, args.clone()).await;
        assert!(result.is_error, "{args}");
        assert_eq!(body(&result)["error"]["field"], field, "{args}");
        assert_eq!(std::fs::read_to_string(&path).unwrap(), DEVICE, "{args}");
    }
}

/// Asking for what the pin already is changes nothing and writes nothing.
#[tokio::test]
async fn restating_a_pin_writes_nothing() {
    let (_d, path) = library();
    let result = edit(
        &path,
        json!({ "pins": [ { "number": "1", "x": 0.0, "y": 3.81, "length": 1.27 } ] }),
    )
    .await;
    assert!(!result.is_error, "{result:?}");
    assert_eq!(body(&result)["written"], false);
    assert_eq!(std::fs::read_to_string(&path).unwrap(), DEVICE);
}
