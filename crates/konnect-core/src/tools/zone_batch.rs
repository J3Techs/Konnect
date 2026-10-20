//! `add_zones`: many copper zones in one call.
//!
//! Every zone is validated with `add_zone`'s rules before anything is
//! written. With KiCad live on the board the batch is one `CreateItems` in
//! one commit (one undo step) followed by one refill. With no live KiCad the
//! saved file takes every zone in one compare-and-swap write, all or
//! nothing, and the written zones are read back by UUID.

use crate::mcp::protocol::CallToolResult;
use crate::tool;
use crate::tools::pcb_board::{
    attempt_ipc_write, format_zone_block, zone_schema, BoardWrite, PadConnection,
    DEFAULT_ZONE_CLEARANCE_MM, DEFAULT_ZONE_MIN_WIDTH_MM,
};
use crate::tools::{get_path, invalid_arg, ToolContext, ToolDef};
use serde_json::{json, Value};
use std::collections::BTreeSet;

/// Most zones one call takes.
const MAX_ZONES: usize = 1_000;

pub(super) fn tool() -> ToolDef {
    let mut item = zone_schema();
    if let Some(props) = item["properties"].as_object_mut() {
        props.remove("board");
    }
    item["required"] = json!(["net_name", "layer", "points"]);
    item["description"] = json!("One zone, with add_zone's fields");
    tool!(
        "add_zones",
        "Add many copper zones in one call, each with add_zone's fields (net_name, layer, points, \
         holes, clearance, min_width, name, priority, pad_connection). Every zone is checked \
         first and nothing is written if any is refused. With KiCad live on this board the zones \
         are created in one undo step and refilled once. With no live KiCad the saved file takes \
         the whole batch in one atomic write (holes included, as further polygons of the zone), \
         every net must already be on the board, and the written zones are read back by UUID; \
         that path carries add_zone's file-edit warning.",
        json!({
            "type": "object",
            "properties": {
                "board": { "type": "string", "description": "Path to the .kicad_pcb file" },
                "zones": { "type": "array", "minItems": 1, "maxItems": MAX_ZONES, "items": item,
                           "description": "The zones to add, in order" }
            },
            "required": ["board", "zones"]
        }),
        |args, ctx| async move { handle_add_zones(args, ctx).await }
    )
    .with_board_access(crate::tools::BoardAccess::LivePreferredWithFallback)
}

/// One validated zone.
#[derive(Debug, Clone)]
pub(crate) struct ZoneRequest {
    pub net_name: String,
    pub layer: String,
    pub points: Vec<(f64, f64)>,
    pub holes: Vec<Vec<(f64, f64)>>,
    pub clearance: f64,
    pub min_width: f64,
    pub name: String,
    pub priority: u32,
    pub connection: PadConnection,
}

impl ZoneRequest {
    fn spec(&self) -> konnect_ipc::builders::ZoneSpec<'_> {
        konnect_ipc::builders::ZoneSpec {
            layer: &self.layer,
            net_name: &self.net_name,
            points: &self.points,
            holes: &self.holes,
            clearance_mm: self.clearance,
            min_thickness_mm: self.min_width,
            name: &self.name,
            priority: self.priority,
            connection: self.connection.as_ipc(),
        }
    }

    fn describe(&self) -> Value {
        json!({
            "net": self.net_name,
            "layer": self.layer,
            "point_count": self.points.len(),
            "hole_count": self.holes.len(),
            "name": self.name,
            "priority": self.priority,
            "pad_connection": self.connection.as_str(),
            "clearance_mm": self.clearance,
            "min_width_mm": self.min_width,
        })
    }
}

/// `{x, y}` vertices, every one finite.
fn vertices(value: &Value) -> Option<Vec<(f64, f64)>> {
    value
        .as_array()?
        .iter()
        .map(|p| {
            let (x, y) = (p.get("x")?.as_f64()?, p.get("y")?.as_f64()?);
            (x.is_finite() && y.is_finite()).then_some((x, y))
        })
        .collect()
}

/// `holes` as `add_zone` reads it: an array of rings, each at least three
/// finite `{x, y}` vertices. `field` names the argument in refusals.
pub(crate) fn parse_zone_holes(
    value: Option<&Value>,
    field: &str,
) -> Result<Vec<Vec<(f64, f64)>>, CallToolResult> {
    let Some(items) = value.filter(|v| !v.is_null()) else {
        return Ok(Vec::new());
    };
    let Some(items) = items.as_array() else {
        return Err(invalid_arg(field, "must be an array of vertex arrays"));
    };
    let mut holes = Vec::new();
    for (index, hole) in items.iter().enumerate() {
        match vertices(hole) {
            Some(v) if v.len() >= 3 => holes.push(v),
            _ => {
                return Err(invalid_arg(
                    &format!("{field}[{index}]"),
                    "must hold at least 3 {x, y} vertices with finite numbers",
                ))
            }
        }
    }
    Ok(holes)
}

fn finite_positive(z: &Value, key: &str, field: &str, default: f64) -> Result<f64, CallToolResult> {
    match z.get(key) {
        None | Some(Value::Null) => Ok(default),
        Some(v) => match v.as_f64().filter(|x| x.is_finite() && *x >= 0.0) {
            Some(x) => Ok(x),
            None => Err(invalid_arg(
                &format!("{field}.{key}"),
                "must be a non-negative length in mm",
            )),
        },
    }
}

/// One entry of `zones`, with `add_zone`'s defaults and rules. `field` is
/// the entry's path (`zones[3]`) for refusals.
pub(crate) fn parse_zone_request(z: &Value, field: &str) -> Result<ZoneRequest, CallToolResult> {
    let text = |key: &str| -> Result<String, CallToolResult> {
        match z.get(key).and_then(Value::as_str) {
            Some(s) if !s.trim().is_empty() => Ok(s.to_string()),
            _ => Err(invalid_arg(
                &format!("{field}.{key}"),
                "is required and must be a non-empty string",
            )),
        }
    };
    let net_name = text("net_name")?;
    let layer = text("layer")?;
    if !layer.ends_with(".Cu") {
        return Err(invalid_arg(
            &format!("{field}.layer"),
            &format!("'{layer}' is not a copper layer"),
        ));
    }
    let points = match z.get("points").and_then(vertices) {
        Some(p) if p.len() >= 3 => p,
        _ => {
            return Err(invalid_arg(
                &format!("{field}.points"),
                "needs at least 3 {x, y} vertices with finite numbers",
            ))
        }
    };
    let holes = parse_zone_holes(z.get("holes"), &format!("{field}.holes"))?;
    let clearance = finite_positive(z, "clearance", field, DEFAULT_ZONE_CLEARANCE_MM)?;
    let min_width = finite_positive(z, "min_width", field, DEFAULT_ZONE_MIN_WIDTH_MM)?;
    let name = match z.get("name") {
        None | Some(Value::Null) => String::new(),
        Some(Value::String(s)) => s.clone(),
        Some(_) => return Err(invalid_arg(&format!("{field}.name"), "must be a string")),
    };
    let priority = match z.get("priority") {
        None | Some(Value::Null) => 0,
        Some(v) => match v.as_u64().and_then(|p| u32::try_from(p).ok()) {
            Some(p) => p,
            None => {
                return Err(invalid_arg(
                    &format!("{field}.priority"),
                    "must be a non-negative integer",
                ))
            }
        },
    };
    let connection = PadConnection::parse(z.get("pad_connection").and_then(Value::as_str))
        .map_err(|message| invalid_arg(&format!("{field}.pad_connection"), &message))?;
    if z.get("pad_connection")
        .is_some_and(|v| !v.is_null() && !v.is_string())
    {
        return Err(invalid_arg(
            &format!("{field}.pad_connection"),
            "must be solid, thermal or none",
        ));
    }
    Ok(ZoneRequest {
        net_name,
        layer,
        points,
        holes,
        clearance,
        min_width,
        name,
        priority,
        connection,
    })
}

async fn handle_add_zones(args: &Value, ctx: &ToolContext) -> anyhow::Result<CallToolResult> {
    let board_path = get_path(args, "board")?;
    let Some(entries) = args.get("zones").and_then(Value::as_array) else {
        return Ok(invalid_arg("zones", "must be an array of zones"));
    };
    if entries.is_empty() || entries.len() > MAX_ZONES {
        return Ok(invalid_arg(
            "zones",
            &format!("must hold 1 to {MAX_ZONES} zones"),
        ));
    }
    let mut zones = Vec::with_capacity(entries.len());
    for (i, z) in entries.iter().enumerate() {
        match parse_zone_request(z, &format!("zones[{i}]")) {
            Ok(r) => zones.push(r),
            Err(refusal) => return Ok(refusal),
        }
    }

    let live = zones.clone();
    let attempt = attempt_ipc_write(ctx, &board_path, "zones", move |c| {
        let specs: Vec<_> = live.iter().map(ZoneRequest::spec).collect();
        c.add_zones(&specs)
    })
    .await?;
    let reason = match attempt {
        BoardWrite::Refused(refusal) => return Ok(refusal),
        BoardWrite::Ipc(ids) => {
            let listed: Vec<Value> = zones
                .iter()
                .zip(ids)
                .map(|(z, id)| {
                    let mut v = z.describe();
                    v["zone_id"] = json!(id);
                    v
                })
                .collect();
            return Ok(CallToolResult::json(&json!({
                "source": "ipc",
                "created": listed.len(),
                "undo_steps": 1,
                "refilled": true,
                "zones": listed,
                "next": "Save the board before reading it with a saved-file tool.",
            })));
        }
        BoardWrite::File(reason) => reason,
    };
    Ok(write_zones_to_file(&board_path, &zones)
        .map(|mut body| {
            body["fallback_reason"] = reason.evidence();
            body["warning"] = json!(reason.warning());
            CallToolResult::json(&body)
        })
        .unwrap_or_else(|refusal| refusal))
}

/// The saved-file path: every net and layer checked against the board, one
/// atomic compare-and-swap write, then each zone read back by UUID.
pub(crate) fn write_zones_to_file(
    board_path: &std::path::Path,
    zones: &[ZoneRequest],
) -> Result<Value, CallToolResult> {
    let original = std::fs::read_to_string(board_path).map_err(|e| {
        CallToolResult::error_kind(
            crate::mcp::error::ToolErrorKind::FileNotFound {
                path: board_path.display().to_string(),
            },
            format!("board could not be read: {}: {e}", board_path.display()),
        )
    })?;
    let tree = konnect_sexp::parse_sexp(&original).map_err(|e| {
        CallToolResult::error(format!(
            "{} is not a readable board: {e}. Nothing was written.",
            board_path.display()
        ))
    })?;
    let nets = konnect_sexp::net::collect_net_keys(&tree);
    let copper: BTreeSet<String> = konnect_sexp::layers::layers(&tree)
        .into_iter()
        .map(|l| l.name)
        .filter(|n| n.ends_with(".Cu"))
        .collect();
    let mut refs = Vec::with_capacity(zones.len());
    for (i, z) in zones.iter().enumerate() {
        if !nets.contains(&z.net_name) {
            return Err(invalid_arg(
                &format!("zones[{i}].net_name"),
                &format!(
                    "net '{}' is not on {}; nothing was written",
                    z.net_name,
                    board_path.display()
                ),
            ));
        }
        if !copper.contains(&z.layer) {
            return Err(invalid_arg(
                &format!("zones[{i}].layer"),
                &format!(
                    "'{}' is not a copper layer of this board; nothing was written",
                    z.layer
                ),
            ));
        }
        match konnect_sexp::net::net_ref_for_write(&tree, &z.net_name) {
            Some(r) => refs.push(r),
            None => {
                return Err(invalid_arg(
                    &format!("zones[{i}].net_name"),
                    &format!(
                        "net '{}' is not declared in this legacy board's net table; nothing was written",
                        z.net_name
                    ),
                ))
            }
        }
    }
    if zones.iter().any(|z| !z.holes.is_empty())
        && refs
            .iter()
            .any(|r| matches!(r, konnect_sexp::net::NetRef::ById { .. }))
    {
        return Err(CallToolResult::error(
            "Holes are written only to KiCad 10 boards on the file path; this legacy board \
             needs KiCad open. Nothing was written.",
        ));
    }
    let uuids: Vec<String> = zones
        .iter()
        .map(|_| uuid::Uuid::new_v4().to_string())
        .collect();
    let mut block = String::new();
    for ((z, net), uuid) in zones.iter().zip(&refs).zip(&uuids) {
        block.push_str(&format_zone_block(
            net,
            &z.layer,
            z.clearance,
            z.min_width,
            &z.points,
            &z.holes,
            &z.name,
            z.priority,
            z.connection,
            uuid,
        ));
    }
    let close = original.rfind(')').ok_or_else(|| {
        CallToolResult::error("The board has no closing parenthesis. Nothing was written.")
    })?;
    let candidate = konnect_sexp::apply_edits(
        original.clone(),
        vec![konnect_sexp::SexpEdit::insert(close, block)],
    );
    let written = crate::tools::closed_board::write_and_read_back(
        board_path,
        "add_zones",
        &original,
        &candidate,
        |persisted| verify_zones(persisted, zones, &uuids),
    )?;
    let listed: Vec<Value> = zones
        .iter()
        .zip(&uuids)
        .map(|(z, uuid)| {
            let mut v = z.describe();
            v["uuid"] = json!(uuid);
            v
        })
        .collect();
    Ok(json!({
        "source": "file",
        "created": listed.len(),
        "zones": listed,
        "write": written,
        "refilled": false,
        "next": "The zones are unfilled until KiCad refills them (open the board and press B, or run_drc with refill_zones).",
    }))
}

/// Read back every zone by UUID: net, layer, name, priority, vertex and
/// hole counts must be what was asked for.
fn verify_zones(persisted: &str, zones: &[ZoneRequest], uuids: &[String]) -> Result<Value, String> {
    let tree = konnect_sexp::parse_sexp(persisted).map_err(|e| e.to_string())?;
    let scan = konnect_sexp::board::zone_records(&tree);
    let mut checked = 0usize;
    for (z, uuid) in zones.iter().zip(uuids) {
        let Some(r) = scan
            .items
            .iter()
            .find(|r| r.uuid.as_deref() == Some(uuid.as_str()))
        else {
            return Err(format!("zone {uuid} is not in the written board"));
        };
        let g = &r.geometry;
        let ok = g.net.as_deref() == Some(z.net_name.as_str())
            && g.layers == vec![z.layer.clone()]
            && g.priority == z.priority
            && g.name.as_deref().unwrap_or("") == z.name
            && g.outline.len() == z.points.len()
            && g.holes.len() == z.holes.len()
            && r.pad_connection.map(|c| c.as_str()) == Some(z.connection.as_str());
        if !ok {
            return Err(format!(
                "zone {uuid} reads back as net {:?}, layers {:?}, priority {}, name {:?}",
                g.net, g.layers, g.priority, g.name
            ));
        }
        checked += 1;
    }
    Ok(json!({ "zones_verified": checked, "zones_on_board": scan.items.len() }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mcp::protocol::ToolContent;
    use crate::tools::pcb_board::board_mock::{ctx_talking_to, spawn_kicad_holding_board};
    use konnect_ipc::gen::kiapi;
    use std::sync::{Arc, Mutex};

    /// A KiCad 10 board whose copper names GND and VCC.
    const BOARD: &str = "(kicad_pcb\n\t(version 20260206)\n\t(generator \"pcbnew\")\n\t(layers\n\t\t(0 \"F.Cu\" signal)\n\t\t(4 \"In1.Cu\" power)\n\t\t(2 \"B.Cu\" signal)\n\t\t(25 \"Edge.Cuts\" user)\n\t)\n\t(segment\n\t\t(start 10 10)\n\t\t(end 20 10)\n\t\t(width 0.2)\n\t\t(layer \"F.Cu\")\n\t\t(net \"GND\")\n\t)\n\t(segment\n\t\t(start 10 12)\n\t\t(end 20 12)\n\t\t(width 0.2)\n\t\t(layer \"F.Cu\")\n\t\t(net \"VCC\")\n\t)\n)\n";

    fn text(r: &CallToolResult) -> String {
        match r.content.first() {
            Some(ToolContent::Text { text }) => text.clone(),
            other => panic!("expected text, got {other:?}"),
        }
    }

    fn body(r: &CallToolResult) -> Value {
        serde_json::from_str(&text(r)).expect("JSON")
    }

    fn square(x: f64, y: f64, s: f64) -> Value {
        json!([{ "x": x, "y": y }, { "x": x + s, "y": y }, { "x": x + s, "y": y + s }, { "x": x, "y": y + s }])
    }

    fn board_file() -> (tempfile::TempDir, std::path::PathBuf) {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("board.kicad_pcb");
        std::fs::write(&path, BOARD).unwrap();
        (dir, path)
    }

    #[tokio::test]
    async fn the_file_path_writes_every_zone_in_one_write_and_reads_them_back() {
        let (_dir, path) = board_file();
        let args = json!({ "board": path, "zones": [
            { "net_name": "GND", "layer": "B.Cu", "points": square(0.0, 0.0, 5.0), "name": "a", "priority": 2 },
            { "net_name": "VCC", "layer": "F.Cu", "points": square(10.0, 0.0, 5.0), "pad_connection": "solid",
              "clearance": 0.3, "min_width": 0.25 },
            { "net_name": "GND", "layer": "In1.Cu", "points": square(20.0, 0.0, 6.0),
              "holes": [[{ "x": 21.0, "y": 1.0 }, { "x": 22.0, "y": 1.0 }, { "x": 22.0, "y": 2.0 }]] },
        ]});
        let r = handle_add_zones(&args, &ctx_talking_to(String::new()))
            .await
            .unwrap();
        assert!(!r.is_error, "{}", text(&r));
        let b = body(&r);
        assert_eq!(b["source"], "file");
        assert_eq!(b["created"], 3);
        assert_eq!(b["write"]["persisted_readback"]["zones_verified"], 3);
        assert!(b["warning"].as_str().is_some(), "the file path warns");

        // Independent readback: the zones as the board scanner reads them.
        let after = std::fs::read_to_string(&path).unwrap();
        let tree = konnect_sexp::parse_sexp(&after).unwrap();
        let zones = konnect_sexp::board::zone_records(&tree);
        assert_eq!(zones.skipped, 0);
        assert_eq!(zones.items.len(), 3);
        let by_uuid = |i: usize| {
            let uuid = b["zones"][i]["uuid"].as_str().unwrap();
            zones
                .items
                .iter()
                .find(|z| z.uuid.as_deref() == Some(uuid))
                .unwrap()
        };
        assert_eq!(by_uuid(0).geometry.priority, 2);
        assert_eq!(by_uuid(0).geometry.name.as_deref(), Some("a"));
        assert_eq!(
            by_uuid(1).pad_connection,
            Some(konnect_sexp::board::ZonePadConnection::Solid)
        );
        assert_eq!(by_uuid(1).clearance, Some(0.3));
        assert_eq!(
            by_uuid(2).geometry.holes.len(),
            1,
            "the hole is a second polygon"
        );
        assert_eq!(by_uuid(2).geometry.layers, vec!["In1.Cu".to_string()]);
        assert!(
            after.starts_with(&BOARD[..BOARD.len() - 2]),
            "everything else is kept"
        );
    }

    #[tokio::test]
    async fn one_bad_zone_refuses_the_batch_and_the_file_is_untouched() {
        for (bad, field) in [
            (
                json!({ "net_name": "NOPE", "layer": "B.Cu", "points": square(0.0, 0.0, 1.0) }),
                "zones[1].net_name",
            ),
            (
                json!({ "net_name": "GND", "layer": "F.SilkS", "points": square(0.0, 0.0, 1.0) }),
                "zones[1].layer",
            ),
            (
                json!({ "net_name": "GND", "layer": "In7.Cu", "points": square(0.0, 0.0, 1.0) }),
                "zones[1].layer",
            ),
            (
                json!({ "net_name": "GND", "layer": "B.Cu", "points": [{ "x": 0, "y": 0 }, { "x": 1, "y": 0 }] }),
                "zones[1].points",
            ),
            (
                json!({ "net_name": "GND", "layer": "B.Cu", "points": square(0.0, 0.0, 1.0), "pad_connection": "spokes" }),
                "zones[1].pad_connection",
            ),
            (
                json!({ "net_name": "GND", "layer": "B.Cu", "points": square(0.0, 0.0, 1.0), "holes": [[{ "x": 0, "y": 0 }]] }),
                "zones[1].holes[0]",
            ),
            (
                json!({ "net_name": "GND", "layer": "B.Cu", "points": square(0.0, 0.0, 1.0), "clearance": -1 }),
                "zones[1].clearance",
            ),
        ] {
            let (_dir, path) = board_file();
            let args = json!({ "board": path, "zones": [
                { "net_name": "GND", "layer": "B.Cu", "points": square(0.0, 0.0, 5.0) },
                bad,
            ]});
            let r = handle_add_zones(&args, &ctx_talking_to(String::new()))
                .await
                .unwrap();
            assert!(r.is_error, "{field} was accepted");
            assert_eq!(body(&r)["error"]["field"], field, "{}", text(&r));
            assert_eq!(std::fs::read_to_string(&path).unwrap(), BOARD, "{field}");
        }
    }

    #[tokio::test]
    async fn a_rejecting_kicad_never_touches_the_file() {
        let (_dir, path) = board_file();
        let server = crate::tools::pcb_board::mounting_hole_tests::spawn_rejecting_kicad();
        let ctx = crate::tools::pcb_board::mounting_hole_tests::ctx_with_ipc(
            server.address().to_string(),
        );
        let args = json!({ "board": path, "zones": [
            { "net_name": "GND", "layer": "B.Cu", "points": square(0.0, 0.0, 5.0) },
        ]});
        let r = handle_add_zones(&args, &ctx).await.unwrap();
        assert!(r.is_error, "{}", text(&r));
        assert_eq!(std::fs::read_to_string(&path).unwrap(), BOARD);
    }

    #[derive(Default)]
    struct Seen {
        create_calls: usize,
        zones_created: usize,
        refills: usize,
        commits: usize,
    }

    fn live_kicad(
        path: &std::path::Path,
        seen: Arc<Mutex<Seen>>,
    ) -> crate::test_support::MockIpcServer {
        use prost::Message;
        spawn_kicad_holding_board(path, move |command| {
            let url = command.type_url.as_str();
            if url.ends_with("GetNets") {
                return Some(konnect_ipc::builders::pack_any(
                    &kiapi::board::commands::NetsResponse {
                        nets: vec![
                            konnect_ipc::builders::net("GND", 1),
                            konnect_ipc::builders::net("VCC", 2),
                        ],
                    },
                    "kiapi.board.commands.NetsResponse",
                ));
            }
            if url.ends_with("BeginCommit") {
                return Some(konnect_ipc::builders::pack_any(
                    &kiapi::common::commands::BeginCommitResponse {
                        id: Some(kiapi::common::types::Kiid {
                            value: "zones".to_string(),
                        }),
                    },
                    "kiapi.common.commands.BeginCommitResponse",
                ));
            }
            if url.ends_with("EndCommit") {
                seen.lock().unwrap().commits += 1;
                return Some(konnect_ipc::builders::pack_any(
                    &kiapi::common::commands::EndCommitResponse {},
                    "kiapi.common.commands.EndCommitResponse",
                ));
            }
            if url.ends_with("RefillZones") {
                seen.lock().unwrap().refills += 1;
                return None;
            }
            if url.ends_with("CreateItems") {
                let request =
                    kiapi::common::commands::CreateItems::decode(command.value.as_slice()).unwrap();
                let mut s = seen.lock().unwrap();
                s.create_calls += 1;
                s.zones_created += request.items.len();
                let created = request
                    .items
                    .into_iter()
                    .enumerate()
                    .map(|(i, item)| {
                        let mut zone =
                            kiapi::board::types::Zone::decode(item.value.as_slice()).unwrap();
                        zone.id = Some(kiapi::common::types::Kiid {
                            value: format!("z{i}"),
                        });
                        kiapi::common::commands::ItemCreationResult {
                            status: Some(kiapi::common::commands::ItemStatus {
                                code: kiapi::common::commands::ItemStatusCode::IscOk as i32,
                                error_message: String::new(),
                            }),
                            item: Some(konnect_ipc::builders::pack_any(
                                &zone,
                                "kiapi.board.types.Zone",
                            )),
                        }
                    })
                    .collect();
                return Some(konnect_ipc::builders::pack_any(
                    &kiapi::common::commands::CreateItemsResponse {
                        header: None,
                        status: kiapi::common::types::ItemRequestStatus::IrsOk as i32,
                        created_items: created,
                    },
                    "kiapi.common.commands.CreateItemsResponse",
                ));
            }
            None
        })
    }

    /// Against a mock KiCad holding the board (not a live editor): one
    /// CreateItems for the whole batch inside one commit, then one refill.
    #[tokio::test]
    async fn a_live_board_takes_the_batch_in_one_commit_and_one_refill() {
        let (_dir, path) = board_file();
        let seen = Arc::new(Mutex::new(Seen::default()));
        let server = live_kicad(&path, seen.clone());
        let ctx = ctx_talking_to(server.address().to_string());
        let args = json!({ "board": path, "zones": [
            { "net_name": "GND", "layer": "B.Cu", "points": square(0.0, 0.0, 5.0) },
            { "net_name": "VCC", "layer": "F.Cu", "points": square(10.0, 0.0, 5.0), "priority": 1 },
            { "net_name": "GND", "layer": "In1.Cu", "points": square(20.0, 0.0, 5.0) },
        ]});
        let r = handle_add_zones(&args, &ctx).await.unwrap();
        assert!(!r.is_error, "{}", text(&r));
        let b = body(&r);
        assert_eq!(b["source"], "ipc");
        assert_eq!(b["created"], 3);
        assert_eq!(b["zones"][2]["zone_id"], "z2");
        let s = seen.lock().unwrap();
        assert_eq!(
            (s.create_calls, s.zones_created, s.commits, s.refills),
            (1, 3, 1, 1)
        );
        assert_eq!(
            std::fs::read_to_string(&path).unwrap(),
            BOARD,
            "the file is not edited"
        );
    }

    #[tokio::test]
    async fn a_live_board_refuses_an_unknown_net_before_creating_anything() {
        let (_dir, path) = board_file();
        let seen = Arc::new(Mutex::new(Seen::default()));
        let server = live_kicad(&path, seen.clone());
        let ctx = ctx_talking_to(server.address().to_string());
        let args = json!({ "board": path, "zones": [
            { "net_name": "GND", "layer": "B.Cu", "points": square(0.0, 0.0, 5.0) },
            { "net_name": "MISSING", "layer": "B.Cu", "points": square(10.0, 0.0, 5.0) },
        ]});
        let r = handle_add_zones(&args, &ctx).await.unwrap();
        assert!(r.is_error, "{}", text(&r));
        let s = seen.lock().unwrap();
        assert_eq!((s.create_calls, s.refills), (0, 0));
    }
}
