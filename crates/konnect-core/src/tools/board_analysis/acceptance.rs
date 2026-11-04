//! Acceptance runs of the board analyses against a real board, ignored by
//! default. The cases live outside the repository, in a JSON file:
//!
//! ```json
//! { "board": "path/to/board.kicad_pcb",
//!   "cases": [
//!     { "case": "hygiene", "tool": "check_zone_hygiene", "args": { "sliver_width_mm": 0.15 } },
//!     { "case": "fields", "tool": "set_footprint_field_values", "board": "scratch/copy.kicad_pcb",
//!       "args": { "edits": [] }, "drc_parity": true }
//!   ] }
//! ```
//!
//! ```text
//! KONNECT_ACCEPT_CASES=cases.json KONNECT_ACCEPT_OUT=<dir> [KICAD_CLI_PATH=<kicad-cli>] \
//!   [KONNECT_ACCEPT_ONLY=case1,case2] \
//!   cargo test -p konnect-core --release --lib board_analysis::acceptance -- --ignored --nocapture
//! ```
//!
//! An analysis case writes `<case>.json` (the full output),
//! `<case>.summary.json` (the inline reply) and `<case>.markers.json`, and
//! only reads its board. A `set_footprint_field_values` case dry-runs, then
//! applies the edits to its own `board` (a scratch copy, which KiCad must
//! not hold) and, with `drc_parity`, records DRC schematic parity before and
//! after in `<case>.json`.

use crate::mcp::protocol::{CallToolResult, ToolContent};
use serde_json::{json, Value};
use std::path::PathBuf;

type Analysis = fn(
    &super::common::Loaded,
    &Value,
    &super::common::Outputs,
) -> Result<CallToolResult, CallToolResult>;

fn analysis(tool: &str) -> Option<Analysis> {
    Some(match tool {
        "check_zone_hygiene" => super::hygiene::check_zone_hygiene,
        "check_current_capacity" => super::current::check_current_capacity,
        "analyze_ir_drop" => super::irdrop::analyze_ir_drop,
        "check_stitching_vias" => super::stitching::check_stitching_vias,
        "check_decoupling_placement" => super::decoupling::check_decoupling_placement,
        "check_thermal_pads" => super::thermal::check_thermal_pads,
        "check_voltage_spacing" => super::spacing::check_voltage_spacing,
        "measure_net_lengths" => super::lengths::measure_net_lengths,
        _ => return None,
    })
}

fn body(r: &CallToolResult) -> Value {
    match r.content.first() {
        Some(ToolContent::Text { text }) => {
            serde_json::from_str(text).unwrap_or(json!({ "raw": text }))
        }
        _ => Value::Null,
    }
}

fn context(cli: String) -> std::sync::Arc<crate::tools::ToolContext> {
    std::sync::Arc::new(crate::tools::ToolContext::new(
        crate::tools::ServerConfig {
            kicad_cli: cli,
            kicad_binary: String::new(),
            // Nothing listens here: KiCad provably holds no board.
            ipc_address: "ipc://konnect-acceptance-no-kicad.sock".into(),
            project_dir: None,
            jlcpcb_db_path: None,
            auto_load_toolsets: false,
            eager_toolsets: false,
        },
        std::sync::Arc::new(crate::router::ToolRouter::new()),
    ))
}

async fn parity(ctx: &crate::tools::ToolContext, board: &str) -> Value {
    let path = PathBuf::from(board);
    match crate::tools::drc::run(ctx, &path, &json!({})).await {
        Ok(Ok((report, _))) => {
            let list = report.schematic_parity.clone().unwrap_or_default();
            json!({
                "count": list.len(),
                "items": list.iter().map(|v| json!({ "rule": v.rule, "description": v.description })).collect::<Vec<_>>(),
            })
        }
        Ok(Err(r)) => json!({ "error": body(&r) }),
        Err(e) => json!({ "error": e.to_string() }),
    }
}

#[tokio::test]
#[ignore]
async fn acceptance_cases() {
    let (Ok(cases), Ok(out)) = (
        std::env::var("KONNECT_ACCEPT_CASES"),
        std::env::var("KONNECT_ACCEPT_OUT"),
    ) else {
        eprintln!("set KONNECT_ACCEPT_CASES and KONNECT_ACCEPT_OUT to run");
        return;
    };
    let spec: Value = serde_json::from_str(&std::fs::read_to_string(&cases).unwrap()).unwrap();
    let out = PathBuf::from(out);
    std::fs::create_dir_all(&out).unwrap();
    let only = std::env::var("KONNECT_ACCEPT_ONLY").ok();
    let mut failed = Vec::new();
    for case in spec["cases"].as_array().expect("cases") {
        let name = case["case"].as_str().expect("case name");
        if only
            .as_deref()
            .is_some_and(|o| !o.split(',').any(|x| x == name))
        {
            continue;
        }
        let tool = case["tool"].as_str().expect("tool");
        let board = case["board"]
            .as_str()
            .or(spec["board"].as_str())
            .expect("board")
            .to_string();
        let mut args = case["args"].clone();
        args["board"] = json!(board);
        let t = std::time::Instant::now();
        let full = out.join(format!("{name}.json"));
        let (ok, summary) = if let Some(f) = analysis(tool) {
            let markers = out.join(format!("{name}.markers.json"));
            let _ = std::fs::remove_file(&full);
            let _ = std::fs::remove_file(&markers);
            args["output"] = json!(full.display().to_string());
            args["markers_output"] = json!(markers.display().to_string());
            let r = super::blocking(&args, f).await.unwrap();
            (!r.is_error, body(&r))
        } else if tool == "set_footprint_field_values" {
            let ctx = context(std::env::var("KICAD_CLI_PATH").unwrap_or_default());
            let want_parity = case["drc_parity"].as_bool().unwrap_or(false);
            let before = if want_parity {
                parity(&ctx, &board).await
            } else {
                Value::Null
            };
            let def = crate::tools::pcb_fields::tools()
                .into_iter()
                .find(|t| t.name == tool)
                .unwrap();
            let dry = (def.handler)(&args, ctx.clone()).await.unwrap();
            let dry_body = body(&dry);
            let mut apply_args = args.clone();
            apply_args["dry_run"] = json!(false);
            apply_args["expected_revision"] = dry_body["plan_revision"].clone();
            let apply = (def.handler)(&apply_args, ctx.clone()).await.unwrap();
            let after = if want_parity {
                parity(&ctx, &board).await
            } else {
                Value::Null
            };
            let report = json!({
                "board": board, "parity_before": before, "dry_run": dry_body,
                "apply": body(&apply), "parity_after": after,
            });
            std::fs::write(&full, serde_json::to_string_pretty(&report).unwrap()).unwrap();
            (!dry.is_error && !apply.is_error, report)
        } else {
            panic!("{name}: no acceptance runner for tool {tool}");
        };
        std::fs::write(
            out.join(format!("{name}.summary.json")),
            serde_json::to_string_pretty(&summary).unwrap(),
        )
        .unwrap();
        eprintln!("{name}: ok={ok} in {:.1}s", t.elapsed().as_secs_f64());
        if !ok {
            failed.push(name.to_string());
        }
    }
    assert!(failed.is_empty(), "cases failed: {failed:?}");
}
