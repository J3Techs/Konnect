//! The closed-board path of `delete_zones`.
//!
//! With no live KiCad on the board, the zones the call selects are cut from
//! the saved file: each is one direct child `(zone …)` block of the board
//! form, removed whole together with the whitespace that set it on its own
//! line. A zone that is a child of a footprint is never read as a candidate,
//! so it cannot be selected. KiCad must provably not hold the board when the
//! plan is made and again before the write; the write is one atomic replace,
//! and the persisted board is read back: every byte outside the removed
//! blocks is the original's, the board zones left are exactly the ones not
//! selected, and the footprint child zones are the ones it had.

use std::path::Path;

use serde_json::{json, Value};

use crate::mcp::error::ToolErrorKind;
use crate::mcp::protocol::CallToolResult;
use crate::tools::closed_board::{only_spans_changed, prove_closed, write_and_read_back};
use crate::tools::pcb_board::NoLiveBoard;
use crate::tools::pcb_components::check_single_board_form;
use crate::tools::pcb_routing::{ZoneInfo, ZoneSelection};
use crate::tools::ToolContext;

pub(crate) const OPERATION: &str = "delete_zones";

/// The zones of a saved board: its own (direct children of the board form,
/// in file order, with the byte span of each block) and the UUIDs of those
/// that belong to footprints.
struct SavedZones {
    zones: Vec<ZoneInfo>,
    spans: Vec<(usize, usize)>,
    footprint_zone_uuids: Vec<String>,
}

fn head_of(block: &str) -> &str {
    block[1..]
        .trim_start()
        .split(|c: char| c.is_whitespace() || c == '(' || c == ')')
        .next()
        .unwrap_or("")
}

/// A zone's layers as the listing and the `layer` filter see them: `F&B.Cu`
/// is both outer copper layers, `*.Cu` every copper layer of the board, and
/// `*.X` otherwise the front and back layer of that name.
fn expand_layers(written: &[String], copper: &[String]) -> Vec<String> {
    let mut out: Vec<String> = Vec::new();
    for layer in written {
        let expanded = match layer.as_str() {
            "F&B.Cu" => vec!["F.Cu".to_string(), "B.Cu".to_string()],
            "*.Cu" => copper.to_vec(),
            other => match other.strip_prefix("*.") {
                Some(suffix) => vec![format!("F.{suffix}"), format!("B.{suffix}")],
                None => vec![other.to_string()],
            },
        };
        for name in expanded {
            if !out.contains(&name) {
                out.push(name);
            }
        }
    }
    out
}

fn read_saved(source: &str) -> Result<SavedZones, String> {
    check_single_board_form(source)?;
    let tree = konnect_sexp::parse_sexp(source).map_err(|error| error.to_string())?;
    let spans: Vec<(usize, usize)> =
        konnect_sexp::writer::find_direct_child_blocks(source, "kicad_pcb")
            .into_iter()
            .filter(|&(start, end)| head_of(&source[start..end]) == "zone")
            .collect();
    let nodes = tree.find_all("zone");
    let scan = konnect_sexp::board::zone_records(&tree);
    if scan.skipped > 0 {
        return Err(format!(
            "{} zone(s) of the saved board could not be read, so the selection cannot be judged",
            scan.skipped
        ));
    }
    if nodes.len() != spans.len() || scan.items.len() != spans.len() {
        return Err("the board's zone blocks could not be located unambiguously".to_string());
    }
    let copper: Vec<String> = konnect_sexp::layers::layers(&tree)
        .into_iter()
        .map(|layer| layer.name)
        .filter(|name| name.ends_with(".Cu"))
        .collect();
    let mut zones: Vec<ZoneInfo> = Vec::with_capacity(spans.len());
    for ((node, record), &(start, end)) in nodes.iter().zip(&scan.items).zip(&spans) {
        let Some(uuid) = record.uuid.clone().filter(|uuid| !uuid.is_empty()) else {
            return Err(
                "a zone of the saved board has no (uuid …), so it cannot be identified; open \
                 and save the board in KiCad 10 first"
                    .to_string(),
            );
        };
        if !source[start..end].contains(uuid.as_str()) {
            return Err(format!("zone '{uuid}' could not be located in the file"));
        }
        if zones.iter().any(|zone| zone.id.eq_ignore_ascii_case(&uuid)) {
            return Err(format!("zone UUID '{uuid}' appears more than once"));
        }
        let geometry = &record.geometry;
        let layers = expand_layers(&geometry.layers, &copper);
        let teardrop = node
            .find("attr")
            .is_some_and(|attr| attr.find("teardrop").is_some());
        let kind = if geometry.keepout.is_some() {
            "rule_area"
        } else if teardrop {
            "teardrop"
        } else if layers.iter().any(|layer| layer.ends_with(".Cu")) {
            "copper"
        } else {
            "graphical"
        };
        zones.push(ZoneInfo {
            id: uuid,
            name: geometry.name.clone().unwrap_or_default(),
            kind,
            net: geometry.net.clone().filter(|net| !net.is_empty()),
            layers,
            priority: geometry.priority,
            locked: record.locked,
            outline_points: geometry.outline.len(),
            holes: geometry.holes.len(),
        });
    }
    let footprint_zone_uuids = tree
        .find_all("footprint")
        .into_iter()
        .flat_map(|footprint| footprint.find_all("zone"))
        .map(|zone| zone.find_str("uuid").unwrap_or_default().to_string())
        .collect();
    Ok(SavedZones {
        zones,
        spans,
        footprint_zone_uuids,
    })
}

/// What one call would do to the saved board.
#[derive(Debug)]
struct Plan {
    listing: Vec<Value>,
    deleted: Vec<String>,
    /// UUIDs of the board zones that stay, in file order.
    remaining: Vec<String>,
    footprint_zone_uuids: Vec<String>,
    kept_locked: usize,
    candidate: String,
    /// `(start, end, replacement_len)` of each removed block.
    edits: Vec<(usize, usize, usize)>,
}

fn blocked(reason: String) -> CallToolResult {
    CallToolResult::error_kind(
        ToolErrorKind::PlanBlocked {
            operation: OPERATION.to_string(),
            reasons: vec![reason.clone()],
        },
        format!("No zone was deleted: {reason}. The board file was not modified."),
    )
}

fn plan(source: &str, selection: &ZoneSelection) -> Result<Plan, CallToolResult> {
    let saved = read_saved(source).map_err(blocked)?;
    let (picked, kept_locked) = selection
        .pick(&saved.zones, &saved.footprint_zone_uuids)
        .map_err(|(field, reason)| crate::tools::invalid_arg(&field, &reason))?;
    let mut changes = Vec::new();
    for zone in &picked {
        let index = saved
            .zones
            .iter()
            .position(|candidate| candidate.id == zone.id)
            .expect("a picked zone is one of the board's");
        let (start, end) = saved.spans[index];
        let block = &source[start..end];
        let elsewhere =
            source.matches(zone.id.as_str()).count() - block.matches(zone.id.as_str()).count();
        if elsewhere > 0 {
            return Err(blocked(format!(
                "zone '{}' is named {elsewhere} more time(s) elsewhere in the board, such as a \
                 group's member list; deleting it from the file would leave that reference \
                 dangling. Ungroup it in KiCad, or delete it with the board open",
                zone.id
            )));
        }
        // The block and the whitespace that sets it on its own line.
        let cut_from = source[..start]
            .trim_end_matches([' ', '\t', '\r', '\n'])
            .len();
        changes.push(konnect_sexp::SexpEdit::delete(cut_from, end));
    }
    let edits: Vec<_> = changes
        .iter()
        .map(|edit| (edit.start, edit.end, edit.replacement.len()))
        .collect();
    let deleted: Vec<String> = picked.iter().map(|zone| zone.id.clone()).collect();
    let plan = Plan {
        listing: picked.iter().map(|zone| zone.to_json()).collect(),
        remaining: saved
            .zones
            .iter()
            .map(|zone| zone.id.clone())
            .filter(|id| !deleted.contains(id))
            .collect(),
        deleted,
        footprint_zone_uuids: saved.footprint_zone_uuids,
        kept_locked,
        candidate: konnect_sexp::apply_edits(source.to_string(), changes),
        edits,
    };
    verify(source, &plan.candidate, &plan).map_err(blocked)?;
    Ok(plan)
}

/// The candidate (or persisted) board differs from the original only by the
/// removed blocks, still is one board form, keeps exactly the board zones not
/// selected, and keeps every footprint child zone.
fn verify(original: &str, candidate: &str, plan: &Plan) -> Result<Value, String> {
    if !only_spans_changed(original, candidate, &plan.edits) {
        return Err("bytes outside the removed zone blocks changed".to_string());
    }
    let after = read_saved(candidate)?;
    let remaining: Vec<&str> = after.zones.iter().map(|zone| zone.id.as_str()).collect();
    if remaining != plan.remaining {
        return Err(
            "the board zones left are not exactly the ones that were not selected".to_string(),
        );
    }
    if after.footprint_zone_uuids != plan.footprint_zone_uuids {
        return Err("the zones that belong to footprints changed".to_string());
    }
    Ok(json!({
        "zones_deleted": plan.deleted.len(),
        "zones_on_board": after.zones.len(),
        "footprint_zones_kept": after.footprint_zone_uuids.len(),
        "bytes_outside_removed_zone_blocks_unchanged": true,
    }))
}

/// `delete_zones` on a board the live write gate said no KiCad holds.
pub(crate) async fn handle_closed(
    ctx: &ToolContext,
    board: &Path,
    selection: &ZoneSelection,
    dry_run: bool,
    expected_count: Option<u64>,
    reason: &NoLiveBoard,
) -> anyhow::Result<CallToolResult> {
    let closure = match prove_closed(ctx, board, OPERATION).await? {
        Ok(closure) => closure,
        Err(refusal) => return Ok(refusal),
    };
    let original = match konnect_sexp::read_consistent(board) {
        Ok(text) => text,
        Err(error) => {
            return Ok(CallToolResult::error_kind(
                ToolErrorKind::FileNotFound {
                    path: board.display().to_string(),
                },
                format!("board could not be read: {}: {error}", board.display()),
            ))
        }
    };
    let plan = match plan(&original, selection) {
        Ok(plan) => plan,
        Err(refusal) => return Ok(refusal),
    };
    let mut body = json!({
        "count": plan.listing.len(),
        "zones": plan.listing,
        "dry_run": dry_run,
        "deleted": false,
        "locked_kept": plan.kept_locked,
        "footprint_zones_kept": plan.footprint_zone_uuids.len(),
        "source": "file",
        "board_closure": closure,
        "fallback_reason": reason.evidence(),
    });
    if dry_run {
        body["next"] = json!("Apply with dry_run false and expected_count set to this count.");
        return Ok(CallToolResult::json(&body));
    }
    if Some(plan.deleted.len() as u64) != expected_count {
        return Ok(CallToolResult::error(format!(
            "The selection now matches {} zone(s), not the expected {}; nothing was deleted. Run a fresh dry run.",
            plan.deleted.len(),
            expected_count.unwrap_or(0)
        )));
    }
    if let Err(refusal) = prove_closed(ctx, board, OPERATION).await? {
        return Ok(refusal);
    }
    match write_and_read_back(board, OPERATION, &original, &plan.candidate, |text| {
        verify(&original, text, &plan)
    }) {
        Ok(write) => body["write"] = write,
        Err(refusal) => return Ok(refusal),
    }
    body["deleted"] = json!(true);
    body["refilled"] = json!(false);
    body["warning"] = json!(reason.warning());
    body["next"] = json!(
        "The zones left keep their saved fills until KiCad refills them (open the board and press B, or run_drc with refill_zones)."
    );
    Ok(CallToolResult::json(&body))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mcp::protocol::ToolContent;
    use crate::tools::pcb_board::board_mock::ctx_talking_to;

    const GND_F: &str = "11111111-1111-4111-8111-111111111111";
    const GND_B: &str = "22222222-2222-4222-8222-222222222222";
    const VCC_IN1: &str = "33333333-3333-4333-8333-333333333333";
    const KEEPOUT: &str = "44444444-4444-4444-8444-444444444444";
    const LOCKED: &str = "55555555-5555-4555-8555-555555555555";
    const FP_ZONE: &str = "66666666-6666-4666-8666-666666666666";
    const TEARDROP: &str = "77777777-7777-4777-8777-777777777777";

    fn copper_zone(uuid: &str, net: &str, layer: &str, name: &str, extra: &str) -> String {
        format!(
            "\t(zone\n\t\t(net \"{net}\")\n\t\t(layer \"{layer}\")\n\t\t(uuid \"{uuid}\")\n\t\t(name \"{name}\")\n{extra}\t\t(hatch edge 0.508)\n\t\t(connect_pads\n\t\t\t(clearance 0.25)\n\t\t)\n\t\t(min_thickness 0.2)\n\t\t(fill yes\n\t\t\t(thermal_gap 0.5)\n\t\t\t(thermal_bridge_width 0.5)\n\t\t)\n\t\t(polygon\n\t\t\t(pts\n\t\t\t\t(xy 0 0) (xy 10 0) (xy 10 10) (xy 0 10)\n\t\t\t)\n\t\t)\n\t\t(filled_polygon\n\t\t\t(layer \"{layer}\")\n\t\t\t(pts\n\t\t\t\t(xy 0 0) (xy 10 0) (xy 10 10) (xy 0 10)\n\t\t\t)\n\t\t)\n\t)\n"
        )
    }

    fn keepout_zone(uuid: &str, indent: &str) -> String {
        format!(
            "{indent}(zone\n{indent}\t(layers \"F&B.Cu\")\n{indent}\t(uuid \"{uuid}\")\n{indent}\t(hatch full 0.508)\n{indent}\t(connect_pads\n{indent}\t\t(clearance 0)\n{indent}\t)\n{indent}\t(min_thickness 0.25)\n{indent}\t(keepout\n{indent}\t\t(tracks allowed)\n{indent}\t\t(vias not_allowed)\n{indent}\t\t(pads allowed)\n{indent}\t\t(copperpour not_allowed)\n{indent}\t\t(footprints not_allowed)\n{indent}\t)\n{indent}\t(polygon\n{indent}\t\t(pts\n{indent}\t\t\t(xy 1 1) (xy 2 1) (xy 2 2) (xy 1 2)\n{indent}\t\t)\n{indent}\t)\n{indent})\n"
        )
    }

    /// Two outer GND pours, an inner VCC pour, a board rule area, a locked
    /// pour, a teardrop, and a footprint that owns a rule area of its own.
    fn board_text(group: &str) -> String {
        let mut text = String::from(
            "(kicad_pcb\n\t(version 20260206)\n\t(generator \"pcbnew\")\n\t(layers\n\t\t(0 \"F.Cu\" signal)\n\t\t(4 \"In1.Cu\" power)\n\t\t(2 \"B.Cu\" signal)\n\t\t(25 \"Edge.Cuts\" user)\n\t)\n\t(footprint \"Lib:Conn\"\n\t\t(layer \"F.Cu\")\n\t\t(uuid \"99999999-9999-4999-8999-999999999999\")\n\t\t(at 5 5)\n\t\t(property \"Reference\" \"J302\"\n\t\t\t(at 0 0)\n\t\t\t(layer \"F.SilkS\")\n\t\t\t(uuid \"99999999-9999-4999-8999-99999999999a\")\n\t\t)\n",
        );
        text.push_str(&keepout_zone(FP_ZONE, "\t\t"));
        text.push_str("\t)\n\t(segment\n\t\t(start 10 10)\n\t\t(end 20 10)\n\t\t(width 0.2)\n\t\t(layer \"F.Cu\")\n\t\t(net \"GND\")\n\t)\n");
        text.push_str(&copper_zone(GND_F, "GND", "F.Cu", "GND_F", ""));
        text.push_str(&copper_zone(VCC_IN1, "VCC", "In1.Cu", "VCC_IN1", ""));
        text.push_str(&keepout_zone(KEEPOUT, "\t"));
        text.push_str(&copper_zone(GND_B, "GND", "B.Cu", "GND_B", ""));
        text.push_str(&copper_zone(
            LOCKED,
            "GND",
            "In1.Cu",
            "locked",
            "\t\t(locked yes)\n",
        ));
        text.push_str(&copper_zone(
            TEARDROP,
            "GND",
            "F.Cu",
            "$teardrop_padvia$",
            "\t\t(attr\n\t\t\t(teardrop\n\t\t\t\t(type padvia)\n\t\t\t)\n\t\t)\n",
        ));
        text.push_str(group);
        text.push_str("\t(embedded_fonts no)\n)\n");
        text
    }

    fn board_file(text: &str) -> (tempfile::TempDir, std::path::PathBuf) {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("board.kicad_pcb");
        std::fs::write(&path, text).unwrap();
        (dir, path)
    }

    fn text(result: &CallToolResult) -> String {
        match result.content.first() {
            Some(ToolContent::Text { text }) => text.clone(),
            other => panic!("expected text, got {other:?}"),
        }
    }

    fn body(result: &CallToolResult) -> Value {
        serde_json::from_str(&text(result)).expect("JSON")
    }

    async fn call(args: Value) -> CallToolResult {
        let def = crate::tools::pcb_routing::tools()
            .into_iter()
            .find(|tool| tool.name == "delete_zones")
            .unwrap();
        assert!(def.input_validator.is_valid(&args), "schema refuses {args}");
        (def.handler)(&args, std::sync::Arc::new(ctx_talking_to(String::new())))
            .await
            .unwrap()
    }

    fn ids(body: &Value) -> Vec<String> {
        let mut ids: Vec<String> = body["zones"]
            .as_array()
            .unwrap()
            .iter()
            .map(|zone| zone["uuid"].as_str().unwrap().to_string())
            .collect();
        ids.sort();
        ids
    }

    fn zone_uuids_left(path: &Path) -> Vec<String> {
        let saved = read_saved(&std::fs::read_to_string(path).unwrap()).unwrap();
        let mut left: Vec<String> = saved.zones.into_iter().map(|zone| zone.id).collect();
        left.sort();
        left
    }

    #[tokio::test]
    async fn a_closed_board_dry_run_lists_the_selection_and_writes_nothing() {
        let original = board_text("");
        let (_dir, path) = board_file(&original);
        let result = call(json!({ "board": path, "nets": ["GND"], "layer": "F.Cu" })).await;
        assert!(!result.is_error, "{}", text(&result));
        let b = body(&result);
        assert_eq!(b["source"], "file");
        assert_eq!(b["dry_run"], true);
        assert_eq!(b["deleted"], false);
        assert_eq!(b["count"], 2, "{b}");
        assert_eq!(ids(&b), [GND_F, TEARDROP]);
        assert_eq!(b["board_closure"]["proven_closed"], true);
        assert_eq!(b["footprint_zones_kept"], 1);
        assert!(b.get("write").is_none());
        assert_eq!(std::fs::read_to_string(&path).unwrap(), original);
    }

    #[tokio::test]
    async fn a_closed_board_apply_cuts_exactly_the_selected_zone_blocks() {
        let original = board_text("");
        let (_dir, path) = board_file(&original);
        let args = json!({
            "board": path, "nets": ["GND"], "kinds": ["copper"],
            "dry_run": false, "expected_count": 2
        });
        let result = call(args).await;
        assert!(!result.is_error, "{}", text(&result));
        let b = body(&result);
        assert_eq!(b["deleted"], true);
        assert_eq!(b["locked_kept"], 1);
        assert_eq!(ids(&b), [GND_F, GND_B]);
        assert_eq!(b["write"]["persisted_readback"]["zones_deleted"], 2);
        assert_eq!(b["write"]["persisted_readback"]["zones_on_board"], 4);
        assert_eq!(b["write"]["persisted_readback"]["footprint_zones_kept"], 1);
        assert!(b["warning"].as_str().is_some(), "the file path warns");

        // Independent of the planner: the original minus the two blocks.
        let mut expected = original.clone();
        for uuid in [GND_F, GND_B] {
            let layer = if uuid == GND_F { "F.Cu" } else { "B.Cu" };
            let name = if uuid == GND_F { "GND_F" } else { "GND_B" };
            let block = copper_zone(uuid, "GND", layer, name, "");
            assert_eq!(expected.matches(&block).count(), 1);
            expected = expected.replace(&block, "");
        }
        assert_eq!(std::fs::read_to_string(&path).unwrap(), expected);
        assert_eq!(zone_uuids_left(&path), [VCC_IN1, KEEPOUT, LOCKED, TEARDROP]);
    }

    #[tokio::test]
    async fn all_selects_copper_zones_only_unless_kinds_names_more() {
        let (_dir, path) = board_file(&board_text(""));
        let copper = body(&call(json!({ "board": path, "all": true })).await);
        assert_eq!(ids(&copper), [GND_F, GND_B, VCC_IN1], "{copper}");
        assert_eq!(copper["locked_kept"], 1);

        let with_rules = body(
            &call(json!({ "board": path, "all": true, "kinds": ["copper", "rule_area"], "include_locked": true }))
                .await,
        );
        assert_eq!(ids(&with_rules), [GND_F, GND_B, VCC_IN1, KEEPOUT, LOCKED]);
        assert!(
            !ids(&with_rules).contains(&FP_ZONE.to_string()),
            "a footprint's zone is never selected"
        );

        let rules =
            body(&call(json!({ "board": path, "all": true, "kinds": ["rule_area"] })).await);
        assert_eq!(ids(&rules), [KEEPOUT]);
        assert_eq!(rules["zones"][0]["layers"], json!(["F.Cu", "B.Cu"]));

        // A layer filter alone still reaches a rule area on that layer.
        let on_back = body(&call(json!({ "board": path, "layer": "B.Cu" })).await);
        assert_eq!(ids(&on_back), [GND_B, KEEPOUT]);
        let pours_on_back =
            body(&call(json!({ "board": path, "layer": "B.Cu", "kinds": ["copper"] })).await);
        assert_eq!(ids(&pours_on_back), [GND_B]);

        // The outer pours of one net, in one call.
        let outer = body(
            &call(json!({
                "board": path, "nets": ["GND"], "layers": ["F.Cu", "B.Cu"], "kinds": ["copper"]
            }))
            .await,
        );
        assert_eq!(ids(&outer), [GND_F, GND_B]);
    }

    #[tokio::test]
    async fn uuids_select_exactly_and_an_unknown_or_footprint_zone_refuses_the_call() {
        let original = board_text("");
        let (_dir, path) = board_file(&original);
        let picked =
            body(&call(json!({ "board": path, "uuids": [KEEPOUT, GND_B.to_uppercase()] })).await);
        assert_eq!(ids(&picked), [GND_B, KEEPOUT]);

        for (uuid, says) in [
            (FP_ZONE, "belongs to a footprint"),
            (
                "00000000-0000-4000-8000-000000000000",
                "not a zone of the board",
            ),
        ] {
            let refused = call(json!({
                "board": path, "uuids": [GND_F, uuid], "dry_run": false, "expected_count": 2
            }))
            .await;
            assert!(refused.is_error, "{uuid}");
            let b = body(&refused);
            assert_eq!(b["error"]["field"], "uuids", "{b}");
            assert!(text(&refused).contains(says), "{}", text(&refused));
            assert!(text(&refused).contains(uuid));
            assert_eq!(std::fs::read_to_string(&path).unwrap(), original);
        }
    }

    #[tokio::test]
    async fn a_wrong_expected_count_or_an_ambiguous_selector_deletes_nothing() {
        let original = board_text("");
        let (_dir, path) = board_file(&original);
        for args in [
            json!({ "board": path, "all": true, "dry_run": false, "expected_count": 2 }),
            json!({ "board": path, "all": true, "dry_run": false }),
            json!({ "board": path }),
            json!({ "board": path, "all": false }),
            json!({ "board": path, "all": true, "nets": ["GND"] }),
            json!({ "board": path, "layer": "F.Cu", "layers": ["B.Cu"] }),
            json!({ "board": path, "names": ["absent"], "dry_run": false, "expected_count": 1 }),
        ] {
            let result = call(args.clone()).await;
            assert!(result.is_error, "{args}");
            assert_eq!(std::fs::read_to_string(&path).unwrap(), original, "{args}");
        }
    }

    #[tokio::test]
    async fn a_zone_a_group_lists_is_refused_rather_than_left_dangling() {
        let group = format!(
            "\t(group \"pours\"\n\t\t(uuid \"88888888-8888-4888-8888-888888888888\")\n\t\t(members \"{GND_F}\" \"{GND_B}\")\n\t)\n"
        );
        let original = board_text(&group);
        let (_dir, path) = board_file(&original);
        let result = call(json!({
            "board": path, "uuids": [GND_F], "dry_run": false, "expected_count": 1
        }))
        .await;
        assert!(result.is_error);
        let b = body(&result);
        assert_eq!(b["error"]["kind"], "plan_blocked", "{b}");
        assert!(text(&result).contains("group"), "{}", text(&result));
        assert_eq!(std::fs::read_to_string(&path).unwrap(), original);

        // A zone the group does not list still goes.
        let other = call(json!({
            "board": path, "uuids": [VCC_IN1], "dry_run": false, "expected_count": 1
        }))
        .await;
        assert!(!other.is_error, "{}", text(&other));
    }

    #[tokio::test]
    async fn a_present_kicad_lock_refuses_the_closed_board_delete() {
        let original = board_text("");
        let (dir, path) = board_file(&original);
        std::fs::write(dir.path().join("~board.kicad_pcb.lck"), "{}").unwrap();
        let result = call(json!({ "board": path, "all": true })).await;
        assert!(result.is_error, "{}", text(&result));
        assert_eq!(
            body(&result)["error"]["reason"],
            "kicad_lock_present",
            "{}",
            text(&result)
        );
        assert_eq!(std::fs::read_to_string(&path).unwrap(), original);
    }

    #[test]
    fn verification_catches_a_changed_byte_and_a_lost_footprint_zone() {
        let original = board_text("");
        let selection = ZoneSelection::parse(&json!({ "uuids": [GND_F] })).unwrap();
        let plan = plan(&original, &selection).unwrap();
        assert!(verify(&original, &plan.candidate, &plan).is_ok());

        let touched = plan.candidate.replace("(width 0.2)", "(width 0.3)");
        assert!(verify(&original, &touched, &plan)
            .unwrap_err()
            .contains("outside the removed zone blocks"));

        let fp_zone = keepout_zone(FP_ZONE, "\t\t");
        let without = original.replace(&fp_zone, "");
        let stale = Plan {
            candidate: without.clone(),
            edits: vec![{
                let start = original.find(&fp_zone).unwrap();
                (start, start + fp_zone.len(), 0)
            }],
            deleted: Vec::new(),
            remaining: read_saved(&original)
                .unwrap()
                .zones
                .into_iter()
                .map(|zone| zone.id)
                .collect(),
            listing: Vec::new(),
            footprint_zone_uuids: plan.footprint_zone_uuids.clone(),
            kept_locked: 0,
        };
        assert!(verify(&original, &without, &stale)
            .unwrap_err()
            .contains("belong to footprints"));
    }
}
