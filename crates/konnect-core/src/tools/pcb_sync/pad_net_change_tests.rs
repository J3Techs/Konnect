//! `update_pcb_from_schematic` on pads whose net the schematic changes while
//! the old net has routed copper, through the registered tool against the
//! mock KiCad: the diagnostic says whether a track, arc or via touches the
//! pad, and `accept_pad_net_changes` plans the change only where none does.

use super::*;
use crate::router::ToolRouter;
use crate::tools::cli::test_support::write_script;
use crate::tools::{ServerConfig, ToolContext};
use konnect_ipc::gen::kiapi;
use std::sync::{Arc, Mutex};

const R1_CAPTURE: &[u8] = include_bytes!("../../../tests/fixtures/issue_474_r1.ipc.bin");

/// The netlist: R1 pad 1 on NEW (the board has it on OLD), pad 2 on
/// `pad2` (the board has GND).
fn netlist(pad2: &str) -> String {
    format!(
        r#"
(export
  (components
    (comp
      (ref "R1")
      (value "1k")
      (footprint "Resistor_SMD:R_0402")
      (sheetpath (names "/Power/") (tstamps "/sheet-uuid/"))
      (tstamps "symbol-uuid")
      (units (unit (name "A") (pins (pin (num "1")) (pin (num "2")))))))
  (nets
    (net (code "1") (name "NEW") (class "Default")
      (node (ref "R1") (pin "1") (pintype "passive")))
    (net (code "2") (name "{pad2}") (class "Default")
      (node (ref "R1") (pin "2") (pintype "passive")))))
"#
    )
}

/// The captured R1, linked to the schematic, its pad 1 on OLD and pad 2 on
/// GND; and pad 1's board position in nm.
fn r1_on_board() -> (prost_types::Any, (i64, i64)) {
    let mut fp = kiapi::board::types::FootprintInstance::decode(R1_CAPTURE)
        .expect("the checked-in KiCad IPC capture must decode");
    fp.symbol_path = Some(kiapi::common::types::SheetPath {
        path: vec![
            kiapi::common::types::Kiid {
                value: "sheet-uuid".to_string(),
            },
            kiapi::common::types::Kiid {
                value: "symbol-uuid".to_string(),
            },
        ],
        path_human_readable: "/Power/".to_string(),
    });
    set_field_text(&mut fp.value_field, "Value", "1k");
    let mut pad1 = None;
    let definition = fp.definition.as_mut().expect("definition");
    for item in &mut definition.items {
        if !konnect_ipc::builders::any_is(item, "kiapi.board.types.Pad") {
            continue;
        }
        let mut pad = kiapi::board::types::Pad::decode(item.value.as_slice()).unwrap();
        let net = match pad.number.as_str() {
            "1" => "OLD",
            _ => "GND",
        };
        pad.net = Some(kiapi::board::types::Net {
            code: None,
            name: net.to_string(),
        });
        if pad.number == "1" {
            let p = pad.position.as_ref().expect("pad position");
            pad1 = Some((p.x_nm, p.y_nm));
        }
        *item = konnect_ipc::builders::pack_any(&pad, "kiapi.board.types.Pad");
    }
    (
        konnect_ipc::builders::pack_any(&fp, "kiapi.board.types.FootprintInstance"),
        pad1.expect("R1 has pad 1"),
    )
}

fn track(id: &str, net: &str, a: (f64, f64), b: (f64, f64)) -> prost_types::Any {
    let mut t = konnect_ipc::builders::build_track(net, 1, "F.Cu", 0.2, a.0, a.1, b.0, b.1);
    t.id = Some(kiapi::common::types::Kiid {
        value: id.to_string(),
    });
    konnect_ipc::builders::pack_any(&t, "kiapi.board.types.Track")
}

fn copper_zone() -> prost_types::Any {
    prost_types::Any {
        type_url: "type.googleapis.com/kiapi.board.types.Zone".to_string(),
        value: include_bytes!("../../../tests/fixtures/issue_474_copper_zone_0.ipc.bin").to_vec(),
    }
}

struct Rig {
    _dir: tempfile::TempDir,
    _server: crate::test_support::MockIpcServer,
    schematic: PathBuf,
    board: PathBuf,
    tool: crate::tools::ToolDef,
    context: Arc<ToolContext>,
    footprints: Arc<Mutex<Vec<prost_types::Any>>>,
}

impl Rig {
    async fn call(&self, extra: serde_json::Value) -> (bool, serde_json::Value) {
        let mut args = serde_json::json!({
            "schematic": self.schematic.to_string_lossy(),
            "board": self.board.to_string_lossy(),
        });
        args.as_object_mut()
            .unwrap()
            .extend(extra.as_object().unwrap().clone());
        let r = (self.tool.handler)(&args, self.context.clone())
            .await
            .unwrap();
        let text = match &r.content[0] {
            ToolContent::Text { text } => text.clone(),
            _ => panic!("sync response was not text"),
        };
        (
            r.is_error,
            serde_json::from_str(&text).unwrap_or(serde_json::json!({ "text": text })),
        )
    }
}

/// A live board holding R1 (pad 1 on OLD), `tracks` and one copper zone,
/// with the schematic's netlist moving pad 1 to NEW and pad 2 to `pad2`.
async fn rig(tracks: Vec<prost_types::Any>, pad2: &str) -> Rig {
    let dir = tempfile::tempdir().unwrap();
    let schematic = dir.path().join("pads.kicad_sch");
    let board = dir.path().join("pads.kicad_pcb");
    let exported = dir.path().join("pads.net");
    std::fs::write(
        &schematic,
        include_bytes!("../../../tests/fixtures/structural_scans_kicad10.kicad_sch"),
    )
    .unwrap();
    std::fs::write(
        &board,
        include_bytes!("../../../tests/fixtures/specctra_two_resistors.kicad_pcb"),
    )
    .unwrap();
    std::fs::write(&exported, netlist(pad2)).unwrap();
    let unix_source = exported.to_string_lossy().replace('\'', "'\\''");
    let windows_source = exported.to_string_lossy();
    let cli = write_script(
        dir.path(),
        "fake-kicad-cli-pads",
        &format!(
            "#!/bin/sh\nwhile [ \"$#\" -gt 0 ]; do\n  if [ \"$1\" = \"--output\" ]; then\n    shift\n    cp '{unix_source}' \"$1\"\n    exit $?\n  fi\n  shift\ndone\nexit 2\n"
        ),
        &format!(
            "@echo off\r\n:loop\r\nif \"%~1\"==\"\" exit /b 2\r\nif \"%~1\"==\"--output\" goto found\r\nshift\r\ngoto loop\r\n:found\r\nshift\r\ncopy /Y \"{windows_source}\" \"%~1\" >nul\r\nexit /b %ERRORLEVEL%\r\n"
        ),
    );
    let (r1, _) = r1_on_board();
    let footprints = Arc::new(Mutex::new(vec![r1]));
    let held = footprints.clone();
    let server =
        crate::tools::pcb_board::board_mock::spawn_kicad_holding_board(&board, move |command| {
            use kiapi::common::types::KiCadObjectType as T;
            if command.type_url.ends_with("SaveDocumentToString") {
                let request =
                    kiapi::common::commands::SaveDocumentToString::decode(command.value.as_slice())
                        .expect("snapshot request");
                return Some(konnect_ipc::builders::pack_any(
                    &kiapi::common::commands::SavedDocumentResponse {
                        document: request.document,
                        contents: include_str!(
                            "../../../tests/fixtures/specctra_two_resistors.kicad_pcb"
                        )
                        .into(),
                    },
                    "kiapi.common.commands.SavedDocumentResponse",
                ));
            }
            if command.type_url.ends_with("GetItems") {
                let request = kiapi::common::commands::GetItems::decode(command.value.as_slice())
                    .expect("GetItems request");
                let items = match request.types.first().copied().map(T::try_from) {
                    Some(Ok(T::KotPcbFootprint)) => held.lock().unwrap().clone(),
                    Some(Ok(T::KotPcbTrace)) => tracks.clone(),
                    Some(Ok(T::KotPcbZone)) => vec![copper_zone()],
                    _ => Vec::new(),
                };
                return Some(konnect_ipc::builders::pack_any(
                    &kiapi::common::commands::GetItemsResponse {
                        header: None,
                        status: kiapi::common::types::ItemRequestStatus::IrsOk as i32,
                        items,
                    },
                    "kiapi.common.commands.GetItemsResponse",
                ));
            }
            if command.type_url.ends_with("GetNets") {
                let net = |code: i32, name: &str| kiapi::board::types::Net {
                    code: Some(kiapi::board::types::NetCode { value: code }),
                    name: name.to_string(),
                };
                return Some(konnect_ipc::builders::pack_any(
                    &kiapi::board::commands::NetsResponse {
                        nets: vec![net(1, "OLD"), net(2, "NEW"), net(3, "GND"), net(4, "NEW2")],
                    },
                    "kiapi.board.commands.NetsResponse",
                ));
            }
            if command.type_url.ends_with("GetBoundingBox") {
                return Some(crate::tools::pcb_board::board_mock::kicad_bounding_boxes(
                    command,
                    |_| (0.0, 0.0, 50.0, 40.0),
                ));
            }
            if command.type_url.ends_with("BeginCommit") {
                return Some(konnect_ipc::builders::pack_any(
                    &kiapi::common::commands::BeginCommitResponse {
                        id: Some(kiapi::common::types::Kiid {
                            value: "commit-pads".to_string(),
                        }),
                    },
                    "kiapi.common.commands.BeginCommitResponse",
                ));
            }
            if command.type_url.ends_with("UpdateItems") {
                let request =
                    kiapi::common::commands::UpdateItems::decode(command.value.as_slice())
                        .expect("UpdateItems request");
                let mut board_footprints = held.lock().unwrap();
                let mut updated_items = Vec::new();
                for updated in request.items {
                    board_footprints[0] = updated.clone();
                    updated_items.push(kiapi::common::commands::ItemUpdateResult {
                        status: Some(kiapi::common::commands::ItemStatus {
                            code: kiapi::common::commands::ItemStatusCode::IscOk as i32,
                            error_message: String::new(),
                        }),
                        item: Some(updated),
                    });
                }
                return Some(konnect_ipc::builders::pack_any(
                    &kiapi::common::commands::UpdateItemsResponse {
                        header: None,
                        status: kiapi::common::types::ItemRequestStatus::IrsOk as i32,
                        updated_items,
                    },
                    "kiapi.common.commands.UpdateItemsResponse",
                ));
            }
            if command.type_url.ends_with("EndCommit") {
                return Some(konnect_ipc::builders::pack_any(
                    &kiapi::common::commands::EndCommitResponse {},
                    "kiapi.common.commands.EndCommitResponse",
                ));
            }
            None
        });
    let router = Arc::new(ToolRouter::new());
    router.load("sch_export").await.expect("registered toolset");
    let tool = router
        .get_tool("update_pcb_from_schematic")
        .await
        .expect("registered sync tool");
    let context = Arc::new(ToolContext::new(
        ServerConfig {
            kicad_cli: cli.to_string_lossy().to_string(),
            kicad_binary: String::new(),
            ipc_address: server.address().to_string(),
            project_dir: None,
            jlcpcb_db_path: None,
            auto_load_toolsets: false,
            eager_toolsets: false,
        },
        router,
    ));
    Rig {
        _dir: dir,
        _server: server,
        schematic,
        board,
        tool,
        context,
        footprints,
    }
}

fn pad_diagnostics<'a>(body: &'a serde_json::Value, pad: &str) -> Vec<&'a serde_json::Value> {
    body["diagnostics"]
        .as_array()
        .unwrap()
        .iter()
        .filter(|d| d["code"] == "routed_pad_net_change" && d["pad"] == pad)
        .collect()
}

/// OLD has a track, but far from R1: the conflict says only copper
/// elsewhere on the net is involved, and accepting R1.1 plans the change.
#[tokio::test]
async fn a_pad_only_its_net_has_copper_elsewhere_is_planned_when_accepted() {
    let rig = rig(
        vec![track("far-track", "OLD", (500.0, 500.0), (505.0, 500.0))],
        "GND",
    )
    .await;
    let (error, plain) = rig.call(serde_json::json!({})).await;
    assert!(!error, "{plain}");
    assert_eq!(plain["status"], "conflict", "{plain:#}");
    let d = pad_diagnostics(&plain, "R1.1");
    assert_eq!(d.len(), 1, "{plain:#}");
    let message = d[0]["message"].as_str().unwrap();
    assert!(
        message.contains("no track, arc or via touches the pad")
            && message.contains("1 track(s), arc(s) or via(s) on 'OLD'")
            && message.contains("zone(s)")
            && message.contains("accept_pad_net_changes"),
        "{message}"
    );
    assert!(d[0].get("touching_copper").is_none(), "{plain:#}");

    let (error, accepted) = rig
        .call(serde_json::json!({ "accept_pad_net_changes": ["R1.1"] }))
        .await;
    assert!(!error, "{accepted}");
    assert_eq!(accepted["status"], "ready", "{accepted:#}");
    assert_eq!(
        accepted["accepted_pad_net_changes"],
        serde_json::json!([{ "pad": "R1.1", "from": "OLD", "to": "NEW" }])
    );
    assert_eq!(accepted["refill_needed"], true);
    assert_eq!(accepted["coverage"]["pads_reassigned"]["planned"], 1);
    assert_ne!(
        accepted["plan_revision"], plain["plan_revision"],
        "the acceptance is part of the plan's identity"
    );

    // The apply must name the same acceptance: without it the plan is the
    // unaccepted conflict.
    let (_, unaccepted_apply) = rig
        .call(serde_json::json!({ "dry_run": false, "expected_plan_revision": accepted["plan_revision"] }))
        .await;
    assert_eq!(unaccepted_apply["status"], "conflict");
    let (error, applied) = rig
        .call(serde_json::json!({
            "dry_run": false,
            "expected_plan_revision": accepted["plan_revision"],
            "accept_pad_net_changes": ["R1.1"],
        }))
        .await;
    assert!(!error, "{applied}");
    assert_eq!(applied["status"], "applied", "{applied:#}");
    assert_eq!(applied["refill_needed"], true);
    // R1's pad 1 is on NEW in what KiCad now holds.
    let fp = kiapi::board::types::FootprintInstance::decode(
        rig.footprints.lock().unwrap()[0].value.as_slice(),
    )
    .unwrap();
    let pad1 = fp
        .definition
        .unwrap()
        .items
        .iter()
        .filter(|i| konnect_ipc::builders::any_is(i, "kiapi.board.types.Pad"))
        .filter_map(|i| kiapi::board::types::Pad::decode(i.value.as_slice()).ok())
        .find(|p| p.number == "1")
        .unwrap();
    assert_eq!(pad1.net.unwrap().name, "NEW");
}

/// A track of OLD ending on R1 pad 1 keeps the change a conflict even when
/// accepted, and the diagnostic names the track.
#[tokio::test]
async fn an_accepted_pad_a_track_touches_stays_a_conflict_naming_the_track() {
    let (_, (x, y)) = r1_on_board();
    let (x, y) = (x as f64 / 1e6, y as f64 / 1e6);
    let rig = rig(
        vec![
            track("touching-track", "OLD", (x, y), (x + 5.0, y)),
            track("far-track", "OLD", (500.0, 500.0), (505.0, 500.0)),
        ],
        "GND",
    )
    .await;
    let (error, body) = rig
        .call(serde_json::json!({ "accept_pad_net_changes": ["R1.1"] }))
        .await;
    assert!(!error, "{body}");
    assert_eq!(body["status"], "conflict", "{body:#}");
    let d = pad_diagnostics(&body, "R1.1");
    assert_eq!(d.len(), 1, "{body:#}");
    let touching = d[0]["touching_copper"].as_array().unwrap();
    assert_eq!(touching.len(), 1, "{body:#}");
    assert!(
        touching[0]
            .as_str()
            .unwrap()
            .starts_with("track touching-track (net 'OLD', F.Cu)"),
        "{body:#}"
    );
    let message = d[0]["message"].as_str().unwrap();
    assert!(
        message.contains("touching-track") && message.contains("delete or reroute"),
        "{message}"
    );
    assert!(body
        .get("accepted_pad_net_changes")
        .is_none_or(|a| a == &serde_json::json!([])));
}

/// Accepting R1.1 says nothing about R1.2: its change keeps today's
/// conflict, and an entry naming a pad that does not change is reported
/// unused and still moves the revision.
#[tokio::test]
async fn an_unlisted_pad_still_conflicts_and_the_revision_follows_the_list() {
    let rig = rig(
        vec![track("far-track", "OLD", (500.0, 500.0), (505.0, 500.0))],
        "NEW2",
    )
    .await;
    let (_, one) = rig
        .call(serde_json::json!({ "accept_pad_net_changes": ["R1.1"] }))
        .await;
    assert_eq!(one["status"], "conflict", "{one:#}");
    assert!(
        pad_diagnostics(&one, "R1.1").is_empty(),
        "R1.1 is accepted: {one:#}"
    );
    let pad2 = pad_diagnostics(&one, "R1.2");
    assert_eq!(pad2.len(), 1, "{one:#}");
    assert!(pad2[0]["message"]
        .as_str()
        .unwrap()
        .contains("from 'GND' to 'NEW2'"));

    let (_, both) = rig
        .call(serde_json::json!({ "accept_pad_net_changes": ["R1.1", "R1.2"] }))
        .await;
    assert_eq!(both["status"], "ready", "{both:#}");
    assert_eq!(both["coverage"]["pads_reassigned"]["planned"], 2);
    let (_, extra) = rig
        .call(serde_json::json!({ "accept_pad_net_changes": ["R1.1", "R1.2", "R9.4"] }))
        .await;
    assert_eq!(extra["status"], "ready");
    assert_eq!(
        extra["unused_pad_net_acceptances"],
        serde_json::json!(["R9.4"])
    );
    let revisions: BTreeSet<String> = [&one, &both, &extra]
        .iter()
        .map(|b| b["plan_revision"].as_str().unwrap().to_string())
        .collect();
    assert_eq!(revisions.len(), 3, "each list is its own plan");

    let (error, refused) = rig
        .call(serde_json::json!({ "accept_pad_net_changes": ["J1"] }))
        .await;
    assert!(error, "{refused}");
    assert_eq!(refused["error"]["field"], "accept_pad_net_changes");
}
