//! The design-wide net tools on the `design_nets` fixture.
//!
//! Every expected member below is KiCad's: the fixture's two netlists are
//! `kicad-cli` exports of it (see `tests/fixtures/design_nets.README.md`), and
//! a fake `kicad-cli` serves them — the post-removal one whenever the root it
//! is handed no longer holds the child's sheet block, which is exactly the
//! copy `plan_sheet_removal` makes. `committed_netlists_are_kicads_answer`
//! re-exports the fixture with a real KiCad to keep them honest.

use super::*;
use crate::mcp::protocol::ToolContent;
use crate::tools::ServerConfig;
use std::sync::Arc;

const ROOT: &str = include_str!("../../tests/fixtures/design_nets.kicad_sch");
const CHILD: &str = include_str!("../../tests/fixtures/design_nets_child.kicad_sch");
const NETLIST: &str = include_str!("../../tests/fixtures/design_nets.net");
const CHILD_BLOCK_UUID: &str = "2a3b4c5d-6e7f-4a8b-9c0d-1e2f3a4b5c6d";

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures")
        .join(name)
}

/// The fixture pair in a temporary project, with a fake `kicad-cli`.
struct Project {
    dir: tempfile::TempDir,
    cli: PathBuf,
}

impl Project {
    fn new(root: &str) -> Self {
        let dir = tempfile::tempdir().unwrap();
        std::fs::write(dir.path().join("design_nets.kicad_sch"), root).unwrap();
        std::fs::write(dir.path().join("design_nets_child.kicad_sch"), CHILD).unwrap();
        std::fs::write(dir.path().join("design_nets.kicad_pro"), "{}\n").unwrap();
        let full = fixture("design_nets.net");
        let removed = fixture("design_nets_child_removed.net");
        // `sch export netlist --output <5> --format kicadsexpr <8>`
        let cli = crate::tools::cli::test_support::write_script(
            dir.path(),
            "fake-kicad-cli",
            &format!(
                "#!/bin/sh\nif grep -q '{CHILD_BLOCK_UUID}' \"$8\"; then cp '{}' \"$5\"; else cp '{}' \"$5\"; fi\n",
                full.display(),
                removed.display()
            ),
            &format!(
                "@echo off\r\nfindstr /C:\"{CHILD_BLOCK_UUID}\" \"%~8\" >nul\r\nif errorlevel 1 (copy /Y \"{}\" \"%~5\" >nul) else (copy /Y \"{}\" \"%~5\" >nul)\r\nexit /b 0\r\n",
                removed.display(),
                full.display()
            ),
        );
        Project { dir, cli }
    }

    fn root(&self) -> PathBuf {
        self.dir.path().join("design_nets.kicad_sch")
    }

    async fn call_result(&self, tool: &str, mut args: Value) -> CallToolResult {
        if args.get("schematic").is_none() {
            args["schematic"] = json!(self.root().display().to_string());
        }
        let ctx = ToolContext::new(
            ServerConfig {
                kicad_cli: self.cli.display().to_string(),
                ..Default::default()
            },
            Arc::new(crate::router::ToolRouter::new()),
        );
        let def = tools().into_iter().find(|def| def.name == tool).unwrap();
        (def.handler)(&args, Arc::new(ctx)).await.unwrap()
    }

    async fn call(&self, tool: &str, args: Value) -> Value {
        let result = self.call_result(tool, args).await;
        let body = text_of(&result);
        assert!(!result.is_error, "{tool} failed: {body}");
        serde_json::from_str(&body).unwrap()
    }
}

fn text_of(result: &CallToolResult) -> String {
    match &result.content[0] {
        ToolContent::Text { text } => text.clone(),
        other => panic!("expected text, got {other:?}"),
    }
}

fn error_of(result: &CallToolResult) -> Value {
    assert!(result.is_error, "expected a refusal: {}", text_of(result));
    serde_json::from_str::<Value>(&text_of(result)).unwrap()["error"].clone()
}

fn net<'a>(response: &'a Value, name: &str) -> &'a Value {
    response["nets"]
        .as_array()
        .unwrap()
        .iter()
        .find(|net| net["net"] == name)
        .unwrap_or_else(|| panic!("{name} missing: {response}"))
}

fn pins(net: &Value) -> Vec<String> {
    net["pins"]
        .as_array()
        .unwrap()
        .iter()
        .map(|pin| {
            format!(
                "{}.{}",
                pin["reference"].as_str().unwrap(),
                pin["pin"].as_str().unwrap()
            )
        })
        .collect()
}

fn root_without_child() -> String {
    remove_sheet_block(ROOT, CHILD_BLOCK_UUID).unwrap()
}

// ─── KiCad's netlist ─────────────────────────────────────────────────────────

#[test]
fn the_netlist_is_read_as_kicad_wrote_it() {
    let netlist = Netlist::parse(NETLIST).unwrap();

    assert_eq!(netlist.tool.as_deref(), Some("Eeschema 10.0.7"));
    // Codes 1–17, less 8: the ORPHAN net reaches no pin and is not exported.
    assert_eq!(netlist.nets.len(), 16);
    assert!(netlist.net_named("ORPHAN").is_none());

    let pgood = &netlist.nets[netlist.net_named("PGOOD").unwrap()];
    let members: Vec<(&str, &str, &str)> = pgood
        .nodes
        .iter()
        .map(|node| {
            (
                node.reference.as_str(),
                node.pin.as_str(),
                node.electrical_type.as_str(),
            )
        })
        .collect();
    assert_eq!(
        members,
        [
            ("R1", "2", "passive"),
            ("U1", "2", "input"),
            ("U11", "4", "open_collector")
        ]
    );
}

/// KiCad 10.0.7 writes `pinfunction` as `VOUT_5`; the bare name is the
/// library part's.
#[test]
fn pin_names_come_from_the_library_part_not_pinfunction() {
    let netlist = Netlist::parse(NETLIST).unwrap();
    let rail = &netlist.nets[netlist.net_named("RAIL_A").unwrap()];
    let vout = rail
        .nodes
        .iter()
        .find(|node| node.reference == "U10")
        .unwrap();

    assert_eq!(vout.pin_function.as_deref(), Some("VOUT_5"));
    assert_eq!(netlist.pin_name(vout), "VOUT");
    let input = rail
        .nodes
        .iter()
        .find(|node| node.reference == "U3")
        .unwrap();
    assert_eq!(netlist.pin_name(input), "", "an unnamed pin stays unnamed");
}

#[test]
fn a_no_connect_suffix_is_split_from_the_pin_type() {
    let netlist = Netlist::parse(NETLIST).unwrap();
    let nc = &netlist.nets[netlist.net_named("unconnected-(U4-NC-Pad1)").unwrap()];

    assert_eq!(nc.nodes[0].electrical_type, "free");
    assert!(nc.nodes[0].no_connect, "free+no_connect");
}

#[test]
fn what_is_not_a_netlist_is_an_error_not_an_empty_design() {
    assert!(Netlist::parse("(kicad_sch (version 20260306))").is_err());
    assert!(Netlist::parse("(export (version \"E\"))").is_err());
    assert!(Netlist::parse("not an s-expression (").is_err());
}

// ─── Small pieces ─────────────────────────────────────────────────────────────

#[test]
fn glob_matches_star_and_question_mark_only() {
    assert!(glob_match("RAIL_*", "RAIL_A"));
    assert!(glob_match("RAIL_?", "RAIL_A"));
    assert!(glob_match("*_OK", "/RAIL_A_OK"));
    assert!(glob_match("*", ""));
    assert!(!glob_match("RAIL", "RAIL_A"));
    assert!(
        !glob_match("rail_*", "RAIL_A"),
        "net names are case-sensitive"
    );
}

#[test]
fn designators_sort_the_way_they_read() {
    let mut refs = vec!["R10", "U5211", "R2", "U702", "C1"];
    refs.sort_by(|a, b| natural_cmp(a, b));
    assert_eq!(refs, ["C1", "R2", "R10", "U702", "U5211"]);
}

#[test]
fn removing_a_sheet_block_takes_that_block_and_nothing_else() {
    let edited = root_without_child();

    assert!(!edited.contains(CHILD_BLOCK_UUID));
    assert!(
        edited.contains("(sheet_instances"),
        "sheet_instances is not a sheet"
    );
    let before = parse_sexp(ROOT).unwrap();
    let after = parse_sexp(&edited).unwrap();
    assert_eq!(before.find_all("sheet").len(), 1);
    assert_eq!(after.find_all("sheet").len(), 0);
    assert_eq!(
        before.find_all("symbol").len(),
        after.find_all("symbol").len()
    );
    assert!(remove_sheet_block(ROOT, "00000000-0000-4000-8000-000000000000").is_err());
}

// ─── get_net_members ──────────────────────────────────────────────────────────

/// A global rail sourced on the child and used on the root: every pin with its
/// sheet, every label with its position and shape, and what drives it.
#[tokio::test]
async fn a_global_rail_lists_its_members_on_every_sheet() {
    let project = Project::new(ROOT);
    let response = project
        .call("get_net_members", json!({ "nets": ["RAIL_A"] }))
        .await;
    let rail = net(&response, "RAIL_A");

    assert_eq!(pins(rail), ["C10.1", "R2.1", "R10.1", "U3.2", "U10.5"]);
    let vout = &rail["pins"][4];
    assert_eq!(vout["pin_name"], "VOUT");
    assert_eq!(vout["electrical_type"], "power_out");
    assert_eq!(vout["sheet_file"], "design_nets_child.kicad_sch");
    assert_eq!(rail["pins"][3]["sheet_file"], "design_nets.kicad_sch");

    let labels = rail["labels"].as_array().unwrap();
    assert_eq!(labels.len(), 5, "two on the root, three on the child");
    assert!(labels.iter().all(|label| label["kind"] == "global_label"));
    let output = labels
        .iter()
        .find(|label| label["shape"] == "output")
        .unwrap();
    assert_eq!(output["sheet_file"], "design_nets_child.kicad_sch");
    assert_eq!(
        (output["x"].as_f64(), output["y"].as_f64()),
        (Some(73.66), Some(60.96))
    );
    assert_eq!(output["rotation"], 0.0);

    assert_eq!(rail["drive"]["status"], "driven");
    assert_eq!(rail["drive"]["has_source"], true);
    assert_eq!(rail["drive"]["sources"][0]["reference"], "U10");
    assert_eq!(
        rail["sheets"].as_array().unwrap().len(),
        2,
        "sheets: {}",
        rail["sheets"]
    );
    assert_eq!(
        response["source"]["complete"], true,
        "{}",
        response["source"]
    );
}

/// An open collector counts as a source; the pull-up stays a passive pin.
#[tokio::test]
async fn an_open_drain_net_is_driven_by_its_open_collector() {
    let project = Project::new(ROOT);
    let response = project
        .call("get_net_members", json!({ "nets": ["PGOOD"] }))
        .await;
    let pgood = net(&response, "PGOOD");

    assert_eq!(pins(pgood), ["R1.2", "U1.2", "U11.4"]);
    assert_eq!(pgood["drive"]["status"], "driven");
    assert_eq!(
        pgood["drive"]["sources"][0]["electrical_type"],
        "open_collector"
    );
    assert_eq!(pgood["drive"]["open_drain_only"], true);
}

/// A local net is asked for by its label text, and a hierarchical one is
/// found through the child's label as well as the root's: the sheet pin and
/// the hierarchical label land on KiCad's `/REG_EN`.
#[tokio::test]
async fn a_net_is_found_by_any_label_on_it() {
    let project = Project::new(ROOT);
    let response = project
        .call("get_net_members", json!({ "nets": ["EN", "CHILD_LOCAL"] }))
        .await;

    let en = net(&response, "/REG_EN");
    assert_eq!(en["matched_by"], json!(["label"]));
    assert_eq!(pins(en), ["U1.4", "U10.3"]);
    let kinds: BTreeSet<&str> = en["labels"]
        .as_array()
        .unwrap()
        .iter()
        .map(|label| label["kind"].as_str().unwrap())
        .collect();
    assert_eq!(kinds, BTreeSet::from(["hierarchical_label", "label"]));
    assert_eq!(en["sheet_pins"][0]["name"], "EN");
    assert_eq!(
        en["sheet_pins"][0]["child_sheet_path"],
        "/Rail A regulator/"
    );

    let local = net(&response, "/Rail A regulator/CHILD_LOCAL");
    assert_eq!(pins(local), ["R10.2", "U11.2"]);
    assert_eq!(
        local["drive"]["status"], "biased",
        "R10 pulls it up to RAIL_A"
    );
}

/// KiCad does not export a net that reaches no pin. The sheet walk still
/// finds it, and says it is not KiCad's.
#[tokio::test]
async fn a_net_with_no_pin_is_found_through_its_label() {
    let project = Project::new(ROOT);
    let response = project
        .call("get_net_members", json!({ "nets": ["ORPHAN"] }))
        .await;
    let orphan = net(&response, "ORPHAN");

    assert_eq!(orphan["kicad_net"], false);
    assert_eq!(orphan["counts"]["pins"], 0);
    assert_eq!(orphan["counts"]["labels"], 1);
    assert_eq!(orphan["drive"]["status"], "label_only");
    assert_eq!(orphan["drive"]["label_only"], true);
}

/// Power symbols on both sheets, and the flag that drives the rail.
#[tokio::test]
async fn a_power_rail_lists_its_symbols_and_its_flag() {
    let project = Project::new(ROOT);
    let response = project
        .call("get_net_members", json!({ "nets": ["+3V3"] }))
        .await;
    let rail = net(&response, "+3V3");

    let symbols = rail["power_symbols"].as_array().unwrap();
    let files: BTreeSet<&str> = symbols
        .iter()
        .map(|symbol| symbol["sheet_file"].as_str().unwrap())
        .collect();
    assert_eq!(
        files,
        BTreeSet::from(["design_nets.kicad_sch", "design_nets_child.kicad_sch"])
    );
    assert!(symbols.iter().all(|symbol| symbol["value"] == "+3V3"));
    assert_eq!(rail["power_flags"].as_array().unwrap().len(), 1);
    assert_eq!(rail["drive"]["power_flags"], 1);
    assert_eq!(rail["drive"]["status"], "driven");
    assert!(rail["drive"]["sources"].as_array().unwrap().is_empty());
}

#[tokio::test]
async fn a_glob_matches_net_names_and_labels() {
    let project = Project::new(ROOT);
    let response = project
        .call("get_net_members", json!({ "net_glob": "RAIL_*" }))
        .await;
    let names: Vec<&str> = response["nets"]
        .as_array()
        .unwrap()
        .iter()
        .map(|net| net["net"].as_str().unwrap())
        .collect();

    assert_eq!(names, ["/RAIL_A_OK", "RAIL_A"]);
    assert_eq!(
        net(&response, "/RAIL_A_OK")["matched_by"],
        json!(["net_glob_label"])
    );
}

#[tokio::test]
async fn an_unknown_name_is_reported_not_refused() {
    let project = Project::new(ROOT);
    let response = project
        .call(
            "get_net_members",
            json!({ "nets": ["NO_SUCH_NET", "PGOOD"] }),
        )
        .await;

    assert_eq!(response["not_found"], json!(["NO_SUCH_NET"]));
    assert_eq!(response["net_count"], 1);
}

#[tokio::test]
async fn a_net_or_a_glob_is_required() {
    let project = Project::new(ROOT);
    let refusal = project.call_result("get_net_members", json!({})).await;

    assert_eq!(error_of(&refusal)["kind"], "invalid_argument");
    assert_eq!(error_of(&refusal)["field"], "nets");
}

/// kicad-cli treats the file it is given as the root, so a sub-sheet would be
/// netlisted alone. The refusal names the root to run on instead.
#[tokio::test]
async fn a_sub_sheet_is_refused_with_its_root_named() {
    let project = Project::new(ROOT);
    let child = project.dir.path().join("design_nets_child.kicad_sch");
    let refusal = project
        .call_result(
            "get_net_members",
            json!({ "schematic": child.display().to_string(), "nets": ["RAIL_A"] }),
        )
        .await;
    let error = error_of(&refusal);

    assert_eq!(error["kind"], "invalid_argument");
    assert_eq!(error["field"], "schematic");
    assert!(error["reason"]
        .as_str()
        .unwrap()
        .contains("design_nets.kicad_sch"));
}

#[tokio::test]
async fn a_missing_schematic_is_file_not_found() {
    let project = Project::new(ROOT);
    let missing = project.dir.path().join("absent.kicad_sch");
    let refusal = project
        .call_result(
            "get_net_members",
            json!({ "schematic": missing.display().to_string(), "nets": ["X"] }),
        )
        .await;

    assert_eq!(error_of(&refusal)["kind"], "file_not_found");
}

/// The inline answer is a summary; the file carries identity and sheet path.
#[tokio::test]
async fn the_output_file_carries_the_full_result() {
    let project = Project::new(ROOT);
    let output = project.dir.path().join("reports").join("rail_a.json");
    let response = project
        .call(
            "get_net_members",
            json!({ "nets": ["RAIL_A"], "output": output.display().to_string() }),
        )
        .await;

    assert!(net(&response, "RAIL_A")["labels"][0].get("uuid").is_none());
    assert_eq!(
        response["output"]["bytes"].as_u64().unwrap(),
        std::fs::metadata(&output).unwrap().len()
    );
    let full: Value = serde_json::from_str(&std::fs::read_to_string(&output).unwrap()).unwrap();
    let rail = net(&full, "RAIL_A");
    assert!(rail["labels"][0]["uuid"].is_string());
    assert_eq!(rail["pins"][4]["sheet_path"], "/Rail A regulator/");
    assert_eq!(rail["pins"][4]["value"], "AP2204K-1.5");
}

// ─── find_undriven_nets ───────────────────────────────────────────────────────

/// The fixture as built: STRANDED has a resistor that reaches only an open
/// drain and a capacitor that reaches only ground, and ERC says nothing.
#[tokio::test]
async fn the_passive_only_net_erc_misses_is_reported() {
    let project = Project::new(ROOT);
    let response = project.call("find_undriven_nets", json!({})).await;

    let undriven: Vec<&str> = response["undriven_nets"]
        .as_array()
        .unwrap()
        .iter()
        .map(|net| net["net"].as_str().unwrap())
        .collect();
    assert_eq!(undriven, ["STRANDED"]);
    let stranded = &response["undriven_nets"][0];
    assert_eq!(stranded["kind"], "passive_only");
    assert_eq!(
        stranded["passives"],
        json!(["C4.1 passive", "R4.1 passive"])
    );

    assert_eq!(response["label_only_nets"][0]["net"], "ORPHAN");
    let singles: Vec<&str> = response["single_global_labels"]
        .as_array()
        .unwrap()
        .iter()
        .map(|label| label["text"].as_str().unwrap())
        .collect();
    assert_eq!(singles, ["ORPHAN", "STRANDED"]);
    let biased: Vec<&str> = response["biased_nets"]
        .as_array()
        .unwrap()
        .iter()
        .map(|net| net.as_object().unwrap()["net"].as_str().unwrap())
        .collect();
    assert_eq!(biased, ["/Rail A regulator/CHILD_LOCAL"]);
}

/// With the regulator's sheet deleted, RAIL_A keeps a pull-up and an input
/// and nothing to drive them, while PGOOD keeps its pull-up to +3V3: one is a
/// dead rail, the other merely biased.
#[tokio::test]
async fn a_rail_whose_source_sheet_is_gone_is_reported_with_its_pull_ups() {
    let project = Project::new(&root_without_child());
    let response = project.call("find_undriven_nets", json!({})).await;

    let rail = response["undriven_nets"]
        .as_array()
        .unwrap()
        .iter()
        .find(|net| net["net"] == "RAIL_A")
        .unwrap_or_else(|| panic!("RAIL_A not reported: {response}"));
    assert_eq!(rail["kind"], "inputs");
    assert_eq!(rail["inputs"], json!(["U3.2 input"]));
    assert_eq!(rail["passives"], json!(["R2.1 passive"]));
    assert!(response["biased_nets"]
        .as_array()
        .unwrap()
        .iter()
        .any(|net| net["net"] == "PGOOD"));
}

// ─── plan_sheet_removal ───────────────────────────────────────────────────────

/// Removing the regulator sheet: RAIL_A loses its only source and leaves an
/// undriven input and an orphaned pull-up, PGOOD loses its open drain but
/// keeps its pull-up, /REG_EN is left with its driver alone, and FAULT_N's
/// pull-up now runs to a dead rail.
#[tokio::test]
async fn removing_the_regulator_sheet_reports_what_it_strands() {
    let project = Project::new(ROOT);
    let before = std::fs::read(project.root()).unwrap();
    let response = project
        .call(
            "plan_sheet_removal",
            json!({ "sheet_names": ["Rail A regulator"] }),
        )
        .await;

    assert_eq!(response["dry_run"], true);
    assert_eq!(
        std::fs::read(project.root()).unwrap(),
        before,
        "nothing is written"
    );
    assert_eq!(
        response["removed_sheets"][0]["sheet_path"],
        "/Rail A regulator/"
    );
    assert_eq!(
        response["removed_components"],
        json!(["C10", "R10", "U10", "U11"])
    );
    assert_eq!(response["verification"]["agrees"], true);

    let crossing = |name: &str| -> Value {
        response["crossing_nets"]
            .as_array()
            .unwrap()
            .iter()
            .find(|net| net["net"] == name)
            .cloned()
            .unwrap_or_else(|| panic!("{name} not crossing: {response}"))
    };
    let rail = crossing("RAIL_A");
    assert_eq!(rail["verdict"], "loses_drive");
    assert_eq!(rail["lost_sources"], json!(["U10.5 VOUT power_out"]));
    assert_eq!(rail["remainders"][0]["drive_after"], "undriven");

    let pgood = crossing("PGOOD");
    assert_eq!(pgood["verdict"], "remains_defined");
    assert_eq!(pgood["remainders"][0]["drive_after"], "biased");

    assert_eq!(crossing("+3V3")["verdict"], "keeps_drive");
    assert_eq!(crossing("GND")["verdict"], "keeps_drive");

    let findings = &response["findings"];
    let stranded: Vec<String> = findings["inputs_losing_driver"]
        .as_array()
        .unwrap()
        .iter()
        .map(|pin| {
            format!(
                "{} {}.{}",
                pin["net"].as_str().unwrap(),
                pin["reference"].as_str().unwrap(),
                pin["pin"].as_str().unwrap()
            )
        })
        .collect();
    assert_eq!(stranded, ["RAIL_A U3.2"], "U1.2 on PGOOD stays defined");
    assert_eq!(
        findings["dead_remainders"][0]["passives"],
        json!(["R2.1 passive"])
    );
    assert_eq!(findings["single_pin_remainders"][0]["net"], "/REG_EN");
    assert_eq!(findings["single_pin_remainders"][0]["pin"], "U1.4 output");
    let open_drain = &findings["open_drain_nets_losing_pull_up"][0];
    assert_eq!(open_drain["net"], "FAULT_N");
    assert_eq!(open_drain["pull_up"]["reference"], "R2");

    let internal: Vec<&str> = response["internal_nets"]
        .as_array()
        .unwrap()
        .iter()
        .map(|name| name.as_str().unwrap())
        .collect();
    assert!(
        internal.contains(&"/Rail A regulator/CHILD_LOCAL"),
        "{internal:?}"
    );
}

/// A sheet is named by its name, its path or its file, with one answer.
#[tokio::test]
async fn a_sheet_is_found_by_name_path_or_file() {
    let project = Project::new(ROOT);
    for request in [
        "Rail A regulator",
        "/Rail A regulator/",
        "design_nets_child.kicad_sch",
    ] {
        let response = project
            .call("plan_sheet_removal", json!({ "sheet_names": [request] }))
            .await;
        assert_eq!(
            response["removed_sheets"][0]["sheet_path"], "/Rail A regulator/",
            "{request}"
        );
    }
}

#[tokio::test]
async fn an_unknown_sheet_is_refused_with_the_sheets_that_exist() {
    let project = Project::new(ROOT);
    let refusal = project
        .call_result(
            "plan_sheet_removal",
            json!({ "sheet_names": ["No such sheet"] }),
        )
        .await;
    let error = error_of(&refusal);

    assert_eq!(error["kind"], "invalid_argument");
    assert_eq!(error["field"], "sheet_names");
    assert!(error["reason"]
        .as_str()
        .unwrap()
        .contains("/Rail A regulator/"));
}

// ─── Resolution ───────────────────────────────────────────────────────────────

/// When KiCad keeps apart two pins the sheet walk joins, the walk's labels
/// are listed under each net and the disagreement is reported — never
/// silently assigned to one of them.
#[test]
fn a_disagreement_with_kicad_is_reported_not_guessed() {
    // Move U3.2 out of RAIL_A into a net of its own, as if KiCad had not
    // joined the stub its RAIL_A label sits on.
    let mut netlist = Netlist::parse(NETLIST).unwrap();
    let rail = netlist.net_named("RAIL_A").unwrap();
    let position = netlist.nets[rail]
        .nodes
        .iter()
        .position(|node| node.reference == "U3")
        .unwrap();
    let node = netlist.nets[rail].nodes.remove(position);
    netlist.pin_net.insert(
        (node.reference.clone(), node.pin.clone()),
        netlist.nets.len(),
    );
    netlist.nets.push(NetlistNet {
        code: "99".into(),
        name: "Net-(U3-Pad2)".into(),
        class: None,
        nodes: vec![node],
    });

    let dir = tempfile::tempdir().unwrap();
    std::fs::write(dir.path().join("design_nets.kicad_sch"), ROOT).unwrap();
    std::fs::write(dir.path().join("design_nets_child.kicad_sch"), CHILD).unwrap();
    let hierarchy = Hierarchy::load(&dir.path().join("design_nets.kicad_sch")).unwrap();
    let resolved = resolve(&netlist, &hierarchy, &HashSet::new());

    assert_eq!(
        resolved.disagreements.len(),
        1,
        "{:?}",
        resolved.disagreements
    );
    let nets = &resolved.disagreements[0]["kicad_nets"];
    assert_eq!(nets, &json!(["Net-(U3-Pad2)", "RAIL_A"]));
}

/// The walk and KiCad agree on the fixture: every walked pin is one of
/// KiCad's, and every label lands on exactly one net.
#[test]
fn the_walk_and_kicad_agree_on_the_fixture() {
    let dir = tempfile::tempdir().unwrap();
    std::fs::write(dir.path().join("design_nets.kicad_sch"), ROOT).unwrap();
    std::fs::write(dir.path().join("design_nets_child.kicad_sch"), CHILD).unwrap();
    let hierarchy = Hierarchy::load(&dir.path().join("design_nets.kicad_sch")).unwrap();
    let netlist = Netlist::parse(NETLIST).unwrap();
    let resolved = resolve(&netlist, &hierarchy, &HashSet::new());

    assert!(hierarchy.problems.is_empty(), "{:?}", hierarchy.problems);
    assert_eq!(hierarchy.instances.len(), 2);
    assert_eq!(hierarchy.instances[1].names_path, "/Rail A regulator/");
    assert_eq!(hierarchy.instances[1].page.as_deref(), Some("2"));
    assert!(
        resolved.disagreements.is_empty(),
        "{:?}",
        resolved.disagreements
    );
    assert!(
        resolved.unmatched_pins.is_empty(),
        "{:?}",
        resolved.unmatched_pins
    );
    assert!(
        resolved.attached_by_name.is_empty(),
        "{:?}",
        resolved.attached_by_name
    );
    for (item, nets) in &resolved.item_nets {
        assert_eq!(nets.len(), 1, "{item:?} on {nets:?}");
    }
}

// ─── Against a real KiCad ─────────────────────────────────────────────────────

/// The committed netlists are KiCad's answer for the fixture. Needs
/// `kicad-cli` on PATH or in a standard KiCad install:
/// `cargo test -p konnect-core committed_netlists_are_kicads_answer -- --ignored`.
#[tokio::test]
#[ignore = "needs a real kicad-cli"]
async fn committed_netlists_are_kicads_answer() {
    /// Each net's name and its `(reference, pin, electrical type)` nodes.
    type Nets = Vec<(String, Vec<(String, String, String)>)>;

    async fn nets_of_export(root: &str) -> Nets {
        let dir = tempfile::tempdir().unwrap();
        std::fs::write(dir.path().join("design_nets.kicad_sch"), root).unwrap();
        std::fs::write(dir.path().join("design_nets_child.kicad_sch"), CHILD).unwrap();
        let text = cli::export_netlist_text("kicad-cli", &dir.path().join("design_nets.kicad_sch"))
            .await
            .unwrap();
        nets_of(&Netlist::parse(&text).unwrap())
    }
    fn nets_of(netlist: &Netlist) -> Nets {
        netlist
            .nets
            .iter()
            .map(|net| {
                (
                    net.name.clone(),
                    net.nodes
                        .iter()
                        .map(|node| {
                            (
                                node.reference.clone(),
                                node.pin.clone(),
                                node.electrical_type.clone(),
                            )
                        })
                        .collect(),
                )
            })
            .collect()
    }

    let committed = std::fs::read_to_string(fixture("design_nets_child_removed.net")).unwrap();
    assert_eq!(
        nets_of_export(ROOT).await,
        nets_of(&Netlist::parse(NETLIST).unwrap())
    );
    assert_eq!(
        nets_of_export(&root_without_child()).await,
        nets_of(&Netlist::parse(&committed).unwrap())
    );
}

// ─── The drive model, on small netlists ───────────────────────────────────────
//
// Synthetic on purpose: each is one circuit shape the drive rules have to tell
// apart, written as KiCad's netlist grammar with only the nets section.

/// `(net …)` text: `nodes` are `(reference, pin, pintype)`.
fn net_text(code: u32, name: &str, nodes: &[(&str, &str, &str)]) -> String {
    let nodes: String = nodes
        .iter()
        .map(|(reference, pin, pintype)| {
            format!("(node (ref \"{reference}\") (pin \"{pin}\") (pintype \"{pintype}\"))")
        })
        .collect();
    format!("(net (code \"{code}\") (name \"{name}\") (class \"Default\") {nodes})")
}

/// Drive of `name` on a netlist of `nets`, with a `PWR_FLAG` on each of
/// `flagged`.
fn drive_on(nets: &[String], flagged: &[&str], name: &str) -> Drive {
    let netlist = Netlist::parse(&format!(
        "(export (version \"E\") (components) (nets {}))",
        nets.concat()
    ))
    .unwrap();
    let resolved = Resolved {
        nets: netlist
            .nets
            .iter()
            .enumerate()
            .map(|(index, net)| DesignNet {
                name: net.name.clone(),
                netlist: Some(index),
                items: if flagged.contains(&net.name.as_str()) {
                    vec![Item::Flag(0, 0)]
                } else {
                    Vec::new()
                },
            })
            .collect(),
        item_nets: HashMap::new(),
        pin_instance: HashMap::new(),
        disagreements: Vec::new(),
        attached_by_name: Vec::new(),
        unmatched_pins: Vec::new(),
    };
    DriveModel::new(&netlist, &resolved).drive(netlist.net_named(name))
}

/// A 10 kΩ pull-down holds an input low; it does not drive it.
#[test]
fn a_pull_down_biases_an_input_without_driving_it() {
    let nets = [
        net_text(1, "IN", &[("U1", "1", "input"), ("R1", "1", "passive")]),
        net_text(2, "GND", &[("R1", "2", "passive"), ("U1", "2", "power_in")]),
    ];
    let drive = drive_on(&nets, &["GND"], "IN");

    assert_eq!(drive.status, DriveStatus::Biased);
    assert_eq!(drive.via[0].reference, "R1");
}

/// Every pull-up meets the others at the rail. Walking on through it would
/// make this input "driven" by the connector another pull-up reaches.
#[test]
fn a_rail_ends_a_series_path_instead_of_carrying_it() {
    let nets = [
        net_text(1, "IN", &[("U1", "1", "input"), ("R1", "1", "passive")]),
        net_text(
            2,
            "+3V3",
            &[
                ("R1", "2", "passive"),
                ("R2", "1", "passive"),
                ("U9", "1", "power_out"),
            ],
        ),
        net_text(3, "EXT", &[("R2", "2", "passive"), ("J1", "1", "passive")]),
    ];

    assert_eq!(drive_on(&nets, &[], "IN").status, DriveStatus::Biased);
}

/// A rail with a logic pin tied to it is still a rail: a resistor to it is a
/// pull-up, not a path from that pin.
#[test]
fn a_logic_pin_tied_to_a_rail_does_not_make_it_a_driver() {
    let nets = [
        net_text(1, "IN", &[("U1", "1", "input"), ("R1", "1", "passive")]),
        net_text(
            2,
            "GND",
            &[("R1", "2", "passive"), ("U7", "4", "bidirectional")],
        ),
    ];

    assert_eq!(drive_on(&nets, &["GND"], "IN").status, DriveStatus::Biased);
}

/// A series resistor from a push-pull output drives the far side.
#[test]
fn a_series_resistor_from_an_output_drives() {
    let nets = [
        net_text(1, "GATE", &[("U2", "3", "output"), ("R1", "1", "passive")]),
        net_text(
            2,
            "LINE",
            &[
                ("R1", "2", "passive"),
                ("R2", "1", "passive"),
                ("U3", "2", "input"),
            ],
        ),
        net_text(3, "GND", &[("R2", "2", "passive")]),
    ];
    let drive = drive_on(&nets, &["GND"], "LINE");

    assert_eq!(drive.status, DriveStatus::DrivenThroughSeries);
    assert!(drive.reaches.unwrap().starts_with("U2.3 (output)"));
}

/// A ferrite from a supply feeds a sub-rail; a resistor from it only pulls.
#[test]
fn a_ferrite_feeds_a_supply_and_a_resistor_only_pulls() {
    let fed = [
        net_text(
            1,
            "AVDD",
            &[("FB1", "2", "passive"), ("U4", "7", "power_in")],
        ),
        net_text(
            2,
            "+3V3",
            &[("FB1", "1", "passive"), ("U9", "1", "power_out")],
        ),
    ];
    assert_eq!(
        drive_on(&fed, &[], "AVDD").status,
        DriveStatus::DrivenThroughSeries
    );

    let pulled = [
        net_text(
            1,
            "AVDD",
            &[("R5", "2", "passive"), ("U4", "7", "power_in")],
        ),
        net_text(
            2,
            "+3V3",
            &[("R5", "1", "passive"), ("U9", "1", "power_out")],
        ),
    ];
    assert_eq!(drive_on(&pulled, &[], "AVDD").status, DriveStatus::Biased);
}

/// An open drain drives its own net and nothing beyond its pull-up; a
/// capacitor drives nothing.
#[test]
fn an_open_drain_and_a_capacitor_do_not_drive_the_far_side() {
    let nets = [
        net_text(
            1,
            "RAIL",
            &[
                ("R1", "1", "passive"),
                ("C1", "1", "passive"),
                ("U5", "5", "input"),
            ],
        ),
        net_text(
            2,
            "FLT_N",
            &[("R1", "2", "passive"), ("U6", "6", "open_collector")],
        ),
        net_text(3, "GND", &[("C1", "2", "passive")]),
    ];

    assert_eq!(
        drive_on(&nets, &["GND"], "RAIL").status,
        DriveStatus::Undriven
    );
    let open_drain = drive_on(&nets, &["GND"], "FLT_N");
    assert_eq!(open_drain.status, DriveStatus::Driven);
    assert!(open_drain.open_drain_only);
}

/// An analog switch's passive pin may feed the net; a bleed resistor beside it
/// does not make the net merely biased.
#[test]
fn a_parts_passive_pin_outranks_a_bias_resistor() {
    let nets = [
        net_text(
            1,
            "ADC_IN",
            &[
                ("U1", "T5", "input"),
                ("R1", "1", "passive"),
                ("U2", "3", "passive"),
            ],
        ),
        net_text(2, "GND", &[("R1", "2", "passive"), ("U2", "8", "power_in")]),
        net_text(3, "SENSE", &[("U2", "2", "passive")]),
    ];

    assert_eq!(
        drive_on(&nets, &["GND"], "ADC_IN").status,
        DriveStatus::Indeterminate
    );
}

#[test]
fn a_connector_pin_drives_from_off_the_board() {
    let nets = [net_text(
        1,
        "CAN_H",
        &[("J4", "2", "passive"), ("R9", "1", "passive")],
    )];

    assert_eq!(drive_on(&nets, &[], "CAN_H").status, DriveStatus::External);
}

/// The removal verdicts, from the drive before and after.
#[test]
fn removal_verdicts_separate_drive_from_bias() {
    let driven = Drive {
        status: DriveStatus::Driven,
        sources: vec![0],
        power_flags: 0,
        via: Vec::new(),
        reaches: None,
        has_inputs: true,
        passive_only: false,
        open_drain_only: false,
    };
    let open_drain = Drive {
        open_drain_only: true,
        ..driven.clone()
    };
    let with = |status| Drive {
        status,
        sources: Vec::new(),
        open_drain_only: false,
        ..driven.clone()
    };

    assert_eq!(
        remainder_verdict(&driven, &with(DriveStatus::Biased)),
        "loses_drive"
    );
    assert_eq!(
        remainder_verdict(&driven, &with(DriveStatus::Undriven)),
        "loses_drive"
    );
    assert_eq!(
        remainder_verdict(&open_drain, &with(DriveStatus::Biased)),
        "remains_defined"
    );
    assert_eq!(
        remainder_verdict(&open_drain, &with(DriveStatus::Undriven)),
        "loses_drive"
    );
    assert_eq!(
        remainder_verdict(&driven, &with(DriveStatus::DrivenThroughSeries)),
        "keeps_drive"
    );
    assert_eq!(
        remainder_verdict(&driven, &with(DriveStatus::Indeterminate)),
        "indeterminate"
    );
    assert_eq!(
        remainder_verdict(&with(DriveStatus::Biased), &with(DriveStatus::Undriven)),
        "loses_bias"
    );
    assert_eq!(
        remainder_verdict(&with(DriveStatus::Undriven), &with(DriveStatus::Undriven)),
        "had_no_drive"
    );
}
