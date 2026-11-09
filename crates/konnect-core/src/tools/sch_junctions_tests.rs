//! KiCad's junction rule, through the tools that apply it: the listing, the
//! prune pass, and the deletes and moves that re-judge the dots they touch.
//!
//! `junction_rule.kicad_sch` is KiCad-saved; its README tabulates the twelve
//! dots. Junction N is `dot(N)`, wire N is `wire(N)`.

use crate::mcp::protocol::{CallToolResult, ToolContent};
use crate::tools::{ServerConfig, ToolContext};
use serde_json::{json, Value};
use std::collections::BTreeSet;
use std::path::Path;
use std::sync::Arc;

const FIXTURE: &str = include_str!("../../tests/fixtures/junction_rule.kicad_sch");
const SHEET_PIN_FIXTURE: &str = include_str!("../../tests/fixtures/junction_sheet_pin.kicad_sch");

fn dot(n: u32) -> String {
    format!("d07e0000-0000-4000-8000-000001{n:06}")
}

fn wire(n: u32) -> String {
    format!("d07e0000-0000-4000-8000-000002{n:06}")
}

/// A scratch copy of `source` as a sheet on disk.
fn sheet(source: &str) -> tempfile::NamedTempFile {
    let file = tempfile::Builder::new()
        .suffix(".kicad_sch")
        .tempfile()
        .unwrap();
    std::fs::write(file.path(), source).unwrap();
    file
}

async fn call_result(tool: &str, path: &Path, mut args: Value) -> CallToolResult {
    args["schematic"] = json!(path.display().to_string());
    let definition = crate::tools::sch_analysis::tools()
        .into_iter()
        .chain(crate::tools::sch_batch::tools())
        .chain(crate::tools::sch_wiring::tools())
        .chain(crate::tools::sch_components::tools())
        .chain(crate::tools::sch_hierarchy::tools())
        .find(|tool_def| tool_def.name == tool)
        .unwrap_or_else(|| panic!("no tool named {tool}"));
    let context = ToolContext::new(
        ServerConfig::default(),
        Arc::new(crate::router::ToolRouter::new()),
    );
    (definition.handler)(&args, Arc::new(context))
        .await
        .unwrap()
}

/// The tool's answer: parsed JSON, or the plain text a few tools return.
async fn call(tool: &str, path: &Path, args: Value) -> Value {
    let result = call_result(tool, path, args).await;
    let ToolContent::Text { text } = &result.content[0] else {
        panic!("expected text from {tool}");
    };
    assert!(!result.is_error, "{tool} failed: {text}");
    serde_json::from_str(text).unwrap_or_else(|_| json!(text))
}

fn junction_uuids(path: &Path) -> BTreeSet<String> {
    let source = std::fs::read_to_string(path).unwrap();
    let tree = konnect_sexp::parse_sexp(&source).unwrap();
    tree.find_all("junction")
        .into_iter()
        .filter_map(|node| node.find_str("uuid").map(str::to_owned))
        .collect()
}

fn all_dots() -> BTreeSet<String> {
    (1..=12).map(dot).collect()
}

fn without(dots: &[u32]) -> BTreeSet<String> {
    let gone: BTreeSet<String> = dots.iter().copied().map(dot).collect();
    all_dots().difference(&gone).cloned().collect()
}

fn entry<'a>(listing: &'a Value, uuid: &str) -> &'a Value {
    listing["junctions"]
        .as_array()
        .unwrap()
        .iter()
        .find(|junction| junction["uuid"] == uuid)
        .unwrap_or_else(|| panic!("{uuid} not listed: {listing}"))
}

fn strings(values: &Value) -> Vec<String> {
    values
        .as_array()
        .unwrap()
        .iter()
        .map(|value| value.as_str().unwrap().to_owned())
        .collect()
}

#[tokio::test]
async fn listing_gives_kicads_verdict_for_every_case() {
    let file = sheet(FIXTURE);
    let listing = call("list_schematic_junctions", file.path(), json!({})).await;

    let expected = [
        (1, "unneeded", 2),
        (2, "unneeded", 2),
        (3, "needed", 3),
        (4, "needed", 3),
        (5, "needed", 4),
        (6, "needed", 3),
        (7, "unneeded", 2),
        (8, "needed", 3),
        (9, "needed", 3),
        (10, "duplicate", 3),
        (11, "unneeded", 2),
        (12, "unneeded", 2),
    ];
    for (n, verdict, exits) in expected {
        let junction = entry(&listing, &dot(n));
        assert_eq!(junction["verdict"], verdict, "junction {n}: {junction}");
        assert_eq!(junction["exits"], exits, "junction {n}: {junction}");
    }
    assert_eq!(listing["junction_count"], 12);
    assert_eq!(listing["needed_count"], 6);
    assert_eq!(listing["unneeded_count"], 5);
    assert_eq!(listing["duplicate_count"], 1);
    assert_eq!(listing["not_judged_count"], 0);
    assert_eq!(listing["pins_resolved"], true);
    assert_eq!(entry(&listing, &dot(10))["duplicate_of"], dot(9));
}

#[tokio::test]
async fn listing_names_what_meets_each_dot() {
    let file = sheet(FIXTURE);
    let listing = call("list_schematic_junctions", file.path(), json!({})).await;

    let connection = |junction: u32, uuid: &str| -> Value {
        entry(&listing, &dot(junction))["connections"]
            .as_array()
            .unwrap()
            .iter()
            .find(|connection| connection["uuid"] == uuid)
            .cloned()
            .unwrap_or_else(|| panic!("{uuid} does not meet junction {junction}"))
    };

    // A wire ending on another wire's span: one runs through, one ends.
    let t = &entry(&listing, &dot(4))["connections"];
    assert_eq!(t.as_array().unwrap().len(), 2, "{t}");
    let through = connection(4, &wire(8));
    assert_eq!(through["meets"], "through");
    assert_eq!(through["leaves"], json!(["left", "right"]));
    let stem = connection(4, &wire(9));
    assert_eq!(stem["meets"], "end");
    assert_eq!(stem["leaves"], "down");

    // A corner on a pin: two wire ends and the pin, the three connections.
    let corner_on_pin = entry(&listing, &dot(8))["connections"]
        .as_array()
        .unwrap()
        .clone();
    assert_eq!(corner_on_pin.len(), 3, "{corner_on_pin:?}");
    assert_eq!(connection(8, &wire(14))["leaves"], "left");
    assert_eq!(connection(8, &wire(15))["leaves"], "up");
    let pin = corner_on_pin
        .iter()
        .find(|connection| connection["type"] == "pin")
        .unwrap();
    assert_eq!(pin["reference"], "R3");
    assert_eq!(pin["pin"], "1");

    // A label is reported but is not an exit, so the corner under it is
    // still a corner.
    let labelled = entry(&listing, &dot(11));
    assert_eq!(labelled["labels"][0]["text"], "CORNER_NET");
    assert_eq!(labelled["labels"][0]["kind"], "label");
    assert_eq!(labelled["connections"].as_array().unwrap().len(), 2);
}

#[tokio::test]
async fn prune_dry_run_reports_without_writing() {
    let file = sheet(FIXTURE);
    let report = call(
        "prune_schematic_junctions",
        file.path(),
        json!({ "dry_run": true }),
    )
    .await;

    assert_eq!(report["dry_run"], true);
    assert_eq!(report["written"], false);
    let pruned: BTreeSet<String> = strings(&report["pruned_uuids"]).into_iter().collect();
    let expected: BTreeSet<String> = [1, 2, 7, 10, 11, 12].into_iter().map(dot).collect();
    assert_eq!(pruned, expected);
    assert_eq!(report["pruned_unneeded_count"], 5);
    assert_eq!(report["pruned_duplicate_count"], 1);
    assert_eq!(report["kept_count"], 6);
    assert_eq!(std::fs::read_to_string(file.path()).unwrap(), FIXTURE);
}

#[tokio::test]
async fn prune_removes_only_the_dots_the_rule_does_not_call_for() {
    let file = sheet(FIXTURE);
    let report = call("prune_schematic_junctions", file.path(), json!({})).await;
    assert_eq!(report["written"], true);
    assert_eq!(report["pruned_count"], 6);
    assert_eq!(junction_uuids(file.path()), without(&[1, 2, 7, 10, 11, 12]));

    // Everything else is untouched: the sheet still parses and carries every
    // wire, and a second pass finds nothing left to do.
    let source = std::fs::read_to_string(file.path()).unwrap();
    let tree = konnect_sexp::parse_sexp(&source).unwrap();
    assert_eq!(konnect_sexp::schematic::extract_wires(&tree).len(), 21);
    let again = call("prune_schematic_junctions", file.path(), json!({})).await;
    assert_eq!(again["pruned_count"], 0);
    assert_eq!(again["written"], false);
}

/// A copy-pasted dot can carry its original's UUID: DB1's
/// `82_Valve_Current_Monitor.kicad_sch` has two such pairs. Pruning the copy
/// leaves that UUID on the sheet, so the read-back has to count dots rather
/// than look for the UUID.
#[tokio::test]
async fn prune_removes_a_copy_that_shares_its_originals_uuid() {
    let (start, end) = konnect_sexp::writer::find_direct_child_blocks(FIXTURE, "kicad_sch")
        .into_iter()
        .find(|&(start, end)| {
            FIXTURE[start..end].starts_with("(junction") && FIXTURE[start..end].contains(&dot(3))
        })
        .expect("the fixture holds junction 3");
    let close = FIXTURE.rfind(')').unwrap();
    let copied = format!(
        "{}\t{}\r\n{}",
        &FIXTURE[..close],
        &FIXTURE[start..end],
        &FIXTURE[close..]
    );
    let file = sheet(&copied);
    let report = call("prune_schematic_junctions", file.path(), json!({})).await;
    assert_eq!(report["pruned_duplicate_count"], 2, "{report}");
    assert_eq!(report["pruned_count"], 7, "{report}");
    let source = std::fs::read_to_string(file.path()).unwrap();
    assert_eq!(source.matches(dot(3).as_str()).count(), 1);
    assert_eq!(junction_uuids(file.path()), without(&[1, 2, 7, 10, 11, 12]));
}

#[tokio::test]
async fn prune_refuses_a_non_boolean_dry_run() {
    let file = sheet(FIXTURE);
    let result = call_result(
        "prune_schematic_junctions",
        file.path(),
        json!({ "dry_run": "yes" }),
    )
    .await;
    assert!(result.is_error);
    assert_eq!(std::fs::read_to_string(file.path()).unwrap(), FIXTURE);
}

/// Points the rule does not judge are kept and said to be kept.
#[tokio::test]
async fn bus_points_and_unresolved_pins_are_not_judged() {
    // A bus ending on the corner at junction 1.
    let bus = "\t(bus\n\t\t(pts\n\t\t\t(xy 30.48 30.48) (xy 30.48 20.32)\n\t\t)\n\t\t(stroke\n\t\t\t(width 0)\n\t\t\t(type default)\n\t\t)\n\t\t(uuid \"d07e0000-0000-4000-8000-000009000001\")\n\t)\n";
    let close = FIXTURE.rfind(')').unwrap();
    let with_bus = format!("{}{bus}{}", &FIXTURE[..close], &FIXTURE[close..]);
    let file = sheet(&with_bus);
    let listing = call("list_schematic_junctions", file.path(), json!({})).await;
    let on_bus = entry(&listing, &dot(1));
    assert_eq!(on_bus["verdict"], "not_judged");
    assert_eq!(on_bus["bus"], true);
    let report = call("prune_schematic_junctions", file.path(), json!({})).await;
    assert!(junction_uuids(file.path()).contains(&dot(1)), "{report}");
    assert_eq!(report["not_judged_count"], 1);

    let file = sheet(&with_orphan());
    let report = call("prune_schematic_junctions", file.path(), json!({})).await;
    assert_eq!(report["pins_resolved"], false);
    // Only the duplicate goes: no pin could make a second dot on a point
    // needed. Every dot the visible exits alone call unneeded is kept.
    assert_eq!(strings(&report["pruned_uuids"]), vec![dot(10)]);
    assert_eq!(report["not_judged_count"], 5);
    assert_eq!(junction_uuids(file.path()), without(&[10]));
}

/// The fixture plus a placed symbol whose library definition is missing, so
/// its pins cannot be placed.
fn with_orphan() -> String {
    let orphan = "\t(symbol\n\t\t(lib_id \"Missing:Part\")\n\t\t(at 150 150 0)\n\t\t(unit 1)\n\t\t(uuid \"d07e0000-0000-4000-8000-000009000002\")\n\t\t(property \"Reference\" \"U99\"\n\t\t\t(at 150 150 0)\n\t\t)\n\t)\n";
    let close = FIXTURE.rfind(')').unwrap();
    format!("{}{orphan}{}", &FIXTURE[..close], &FIXTURE[close..])
}

/// A delete re-judges with the same caution: one symbol of four failing to
/// resolve may be the third exit at a corner it cannot see, so the corner a
/// deleted wire leaves keeps its dot. Only a dot left with no wire at all
/// goes.
#[tokio::test]
async fn a_delete_keeps_a_corner_dot_while_a_symbol_is_unresolved() {
    let file = sheet(&with_orphan());
    let reply = call(
        "batch_delete_schematic_wire",
        file.path(),
        json!({ "uuids": [wire(5), wire(21)] }),
    )
    .await;
    assert_eq!(reply["deleted"], 2);
    // Junction 3 is left on a corner: kept. Junction 12 is left on no wire.
    assert_eq!(reply["junctions_pruned_count"], 1, "{reply}");
    assert_eq!(junction_uuids(file.path()), without(&[12]));
}

/// Eeschema counts a symbol once however many of its pins stack on a point,
/// and two symbols there as two exits.
#[tokio::test]
async fn stacked_pins_of_one_symbol_are_one_exit() {
    let lib = "\t\t(symbol \"Test:Stack\"\n\t\t\t(exclude_from_sim no)\n\t\t\t(in_bom yes)\n\t\t\t(on_board yes)\n\t\t\t(symbol \"Stack_1_1\"\n\t\t\t\t(pin passive line\n\t\t\t\t\t(at 0 0 0)\n\t\t\t\t\t(length 2.54)\n\t\t\t\t\t(name \"A\"\n\t\t\t\t\t\t(effects\n\t\t\t\t\t\t\t(font\n\t\t\t\t\t\t\t\t(size 1.27 1.27)\n\t\t\t\t\t\t\t)\n\t\t\t\t\t\t)\n\t\t\t\t\t)\n\t\t\t\t\t(number \"1\"\n\t\t\t\t\t\t(effects\n\t\t\t\t\t\t\t(font\n\t\t\t\t\t\t\t\t(size 1.27 1.27)\n\t\t\t\t\t\t\t)\n\t\t\t\t\t\t)\n\t\t\t\t\t)\n\t\t\t\t)\n\t\t\t\t(pin passive line\n\t\t\t\t\t(at 0 0 0)\n\t\t\t\t\t(length 2.54)\n\t\t\t\t\t(name \"B\"\n\t\t\t\t\t\t(effects\n\t\t\t\t\t\t\t(font\n\t\t\t\t\t\t\t\t(size 1.27 1.27)\n\t\t\t\t\t\t\t)\n\t\t\t\t\t\t)\n\t\t\t\t\t)\n\t\t\t\t\t(number \"2\"\n\t\t\t\t\t\t(effects\n\t\t\t\t\t\t\t(font\n\t\t\t\t\t\t\t\t(size 1.27 1.27)\n\t\t\t\t\t\t\t)\n\t\t\t\t\t\t)\n\t\t\t\t\t)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n";
    let placed = |reference: &str, n: u32| {
        format!(
            "\t(symbol\n\t\t(lib_id \"Test:Stack\")\n\t\t(at 76.2 91.44 0)\n\t\t(unit 1)\n\t\t(uuid \"d07e0000-0000-4000-8000-00000900001{n}\")\n\t\t(property \"Reference\" \"{reference}\"\n\t\t\t(at 76.2 91.44 0)\n\t\t)\n\t)\n"
        )
    };
    // A dot on wire 21's open end, where the stacked symbols are placed.
    let end_dot = "\t(junction\n\t\t(at 76.2 91.44)\n\t\t(diameter 0)\n\t\t(color 0 0 0 0)\n\t\t(uuid \"d07e0000-0000-4000-8000-000009000020\")\n\t)\n";
    let lib_at = FIXTURE.find("\t(lib_symbols").unwrap();
    let lib_body = lib_at + FIXTURE[lib_at..].find('\n').unwrap() + 1;
    let close = FIXTURE.rfind(')').unwrap();
    let build = |symbols: &str| {
        format!(
            "{}{lib}{}{end_dot}{symbols}{}",
            &FIXTURE[..lib_body],
            &FIXTURE[lib_body..close],
            &FIXTURE[close..]
        )
    };

    let file = sheet(&build(&placed("U1", 1)));
    let listing = call("list_schematic_junctions", file.path(), json!({})).await;
    let one = entry(&listing, "d07e0000-0000-4000-8000-000009000020");
    assert_eq!(one["exits"], 2, "{one}");
    assert_eq!(one["verdict"], "unneeded");
    assert_eq!(one["connections"].as_array().unwrap().len(), 3, "{one}");

    let file = sheet(&build(&format!("{}{}", placed("U1", 1), placed("U2", 2))));
    let listing = call("list_schematic_junctions", file.path(), json!({})).await;
    let two = entry(&listing, "d07e0000-0000-4000-8000-000009000020");
    assert_eq!(two["exits"], 3, "{two}");
    assert_eq!(two["verdict"], "needed");
}

/// The acceptance shape: deleting one wire of a three-way join leaves a corner,
/// and the corner's dot goes with the wire.
#[tokio::test]
async fn deleting_a_wire_prunes_a_dot_left_on_a_corner() {
    let file = sheet(FIXTURE);
    let reply = call(
        "delete_schematic_wire",
        file.path(),
        json!({ "uuid": wire(5) }),
    )
    .await;
    assert_eq!(reply, "Wire deleted. Removed 1 orphaned junction(s).");
    assert_eq!(junction_uuids(file.path()), without(&[3]));
}

#[tokio::test]
async fn batch_wire_delete_prunes_by_the_same_rule() {
    let file = sheet(FIXTURE);
    // Wire 8 carries junction 4's T; wire 16 is one arm of the duplicated T,
    // whose two dots both go once it is a corner.
    let reply = call(
        "batch_delete_schematic_wire",
        file.path(),
        json!({ "uuids": [wire(8), wire(16)] }),
    )
    .await;
    assert_eq!(reply["deleted"], 2);
    assert_eq!(reply["junctions_pruned_count"], 3);
    assert_eq!(junction_uuids(file.path()), without(&[4, 9, 10]));
}

#[tokio::test]
async fn batch_delete_prunes_the_dots_its_wires_and_components_leave() {
    let file = sheet(FIXTURE);
    let reply = call(
        "batch_delete",
        file.path(),
        json!({ "references": ["R3"], "uuids": [wire(5)] }),
    )
    .await;
    assert_eq!(reply["deleted_count"], 2, "{reply}");
    // Junction 3 is left on a corner by the wire, junction 8 by the pin.
    assert_eq!(
        strings(&reply["junctions_pruned_uuids"]),
        vec![dot(3), dot(8)]
    );
    assert_eq!(junction_uuids(file.path()), without(&[3, 8]));
}

#[tokio::test]
async fn a_junction_named_in_batch_delete_is_deleted_not_pruned() {
    let file = sheet(FIXTURE);
    let reply = call("batch_delete", file.path(), json!({ "uuids": [dot(12)] })).await;
    assert_eq!(strings(&reply["deleted_item_uuids"]), vec![dot(12)]);
    assert_eq!(reply["junctions_pruned_count"], 0, "{reply}");
    assert_eq!(junction_uuids(file.path()), without(&[12]));
}

#[tokio::test]
async fn moving_a_pin_off_a_corner_prunes_its_dot() {
    let file = sheet(FIXTURE);
    let reply = call(
        "move_schematic_component",
        file.path(),
        json!({ "reference": "R3", "x": 139.7, "y": 64.77 }),
    )
    .await;
    assert_eq!(reply["junctions_pruned_count"], 1, "{reply}");
    assert_eq!(reply["junctions_added_count"], 0, "{reply}");
    assert_eq!(junction_uuids(file.path()), without(&[8]));
}

/// The fixture with one placed symbol's block removed.
fn without_symbol(source: &str, reference: &str) -> String {
    let needle = format!("(property \"Reference\" \"{reference}\"");
    let (start, end) = konnect_sexp::writer::find_direct_child_blocks(source, "kicad_sch")
        .into_iter()
        .find(|&(start, end)| {
            source[start..end].starts_with("(symbol") && source[start..end].contains(&needle)
        })
        .expect("the fixture places that reference");
    format!("{}{}", &source[..start], &source[end..])
}

/// In `junction_sheet_pin.kicad_sch` the dot sits on a wire's span where R1's
/// pin and the sheet's SHPIN meet. Without R1, the sheet pin is the dot's
/// third exit, so deleting the sheet leaves a lone wire.
#[tokio::test]
async fn deleting_a_sheet_prunes_a_dot_its_pin_leaves() {
    let file = sheet(&without_symbol(SHEET_PIN_FIXTURE, "R1"));
    let before = junction_uuids(file.path());
    assert_eq!(before.len(), 1);
    let reply = call("delete_sheet", file.path(), json!({ "sheet_name": "test" })).await;
    assert_eq!(reply["junctions_pruned_count"], 1, "{reply}");
    assert_eq!(
        strings(&reply["junctions_pruned_uuids"]),
        before.into_iter().collect::<Vec<_>>()
    );
    assert!(junction_uuids(file.path()).is_empty());
}

/// With R1 still on the dot, the sheet's pin was never its only third exit.
#[tokio::test]
async fn deleting_a_sheet_keeps_a_dot_still_needed() {
    let file = sheet(SHEET_PIN_FIXTURE);
    let reply = call("delete_sheet", file.path(), json!({ "sheet_name": "test" })).await;
    assert_eq!(reply["junctions_pruned_count"], 0, "{reply}");
    assert_eq!(junction_uuids(file.path()).len(), 1);
}
