//! Closed-board footprint attribute edits. No typed footprint reconstruction:
//! pads, opaque fabrication data, identities, nets and all artwork stay intact.
use super::*;
use konnect_sexp::SexpNode;
use serde_json::Value;

const FLAGS: [&str; 3] = ["board_only", "exclude_from_bom", "exclude_from_pos_files"];

fn attributes(node: &SexpNode) -> Vec<String> {
    node.find("attr")
        .and_then(SexpNode::children)
        .unwrap_or(&[])
        .iter()
        .skip(1)
        .filter_map(SexpNode::as_str)
        .map(str::to_string)
        .collect()
}

fn prepare(
    source: &str,
    reference: &str,
    requested: &BTreeMap<String, bool>,
) -> anyhow::Result<(String, Value)> {
    check_single_board_form(source).map_err(anyhow::Error::msg)?;
    let mut targets = Vec::new();
    for (start, end) in find_direct_child_blocks(source, "kicad_pcb") {
        let node = konnect_sexp::parse_sexp(&source[start..end])?;
        if node.head() == Some("footprint")
            && footprint_reference(&node).as_deref() == Some(reference)
        {
            targets.push((start, end, node));
        }
    }
    anyhow::ensure!(
        targets.len() == 1,
        "Expected one footprint named {reference}, found {}",
        targets.len()
    );
    let (start, end, node) = &targets[0];
    let block = &source[*start..*end];
    let before = attributes(node);
    let attr_blocks = find_direct_child_blocks(block, "footprint")
        .into_iter()
        .filter(|(a, b)| {
            konnect_sexp::parse_sexp(&block[*a..*b]).is_ok_and(|n| n.head() == Some("attr"))
        })
        .collect::<Vec<_>>();
    anyhow::ensure!(attr_blocks.len() <= 1, "Duplicate footprint attr clauses");
    let mut edits = Vec::new();
    let mut after = before.clone();
    for (flag, wanted) in requested {
        if *wanted {
            if !after.contains(flag) {
                after.push(flag.clone());
            }
        } else {
            after.retain(|present| present != flag);
        }
    }
    if after != before {
        if let Some((a, b)) = attr_blocks.first() {
            let attr_source = &block[*a..*b];
            let mut tokens = sexp_block_tokens(attr_source)
                .into_iter()
                .skip(1)
                .map(|(s, e)| attr_source[s..e].to_string())
                .collect::<Vec<_>>();
            for (flag, wanted) in requested {
                if *wanted {
                    if !tokens.iter().any(|token| sexp_token_value(token) == *flag) {
                        tokens.push(flag.clone());
                    }
                } else {
                    tokens.retain(|token| sexp_token_value(token) != *flag);
                }
            }
            edits.push(SexpEdit::replace(
                start + a,
                start + b,
                format!("(attr {})", tokens.join(" ")),
            ));
        } else {
            let (layer_end, _) = find_direct_child_blocks(block, "footprint")
                .into_iter()
                .find_map(|(a, b)| {
                    konnect_sexp::parse_sexp(&block[a..b])
                        .ok()
                        .filter(|n| n.head() == Some("layer"))
                        .map(|_| (b, a))
                })
                .context("Footprint has no layer clause")?;
            let eol = if source.contains("\r\n") {
                "\r\n"
            } else {
                "\n"
            };
            edits.push(SexpEdit::insert(
                start + layer_end,
                format!("{eol}\t\t(attr {})", after.join(" ")),
            ));
        }
    }
    let candidate = apply_edits(source.to_string(), edits);
    let parsed = konnect_sexp::parse_sexp(&candidate)?;
    let observed = parsed
        .find_all("footprint")
        .into_iter()
        .filter(|fp| footprint_reference(fp).as_deref() == Some(reference))
        .collect::<Vec<_>>();
    anyhow::ensure!(
        observed.len() == 1,
        "Attribute edit changed reference coverage"
    );
    let actual = attributes(observed[0]);
    anyhow::ensure!(
        actual == after,
        "Attribute readback differs from requested flags"
    );
    let flags = FLAGS
        .into_iter()
        .map(|flag| {
            (
                flag.to_string(),
                json!(actual.iter().any(|item| item == flag)),
            )
        })
        .collect::<serde_json::Map<_, _>>();
    Ok((
        candidate,
        json!({"reference":reference,"footprint":node.get(1).and_then(SexpNode::as_str),"uuid":node.find_str("uuid").or_else(||node.find_str("tstamp")),"attributes":actual,"flags":flags}),
    ))
}

pub(super) async fn handle(args: &Value, ctx: &ToolContext) -> anyhow::Result<CallToolResult> {
    let board = get_path(args, "board")?;
    let reference = match require_str(args, "reference") {
        Ok(reference) => reference,
        Err(result) => return Ok(result),
    };
    let Some(object) = args["attributes"].as_object() else {
        return Ok(CallToolResult::error(
            "attributes must be an object".to_string(),
        ));
    };
    let mut requested = BTreeMap::new();
    for (name, value) in object {
        if !FLAGS.contains(&name.as_str()) || !value.is_boolean() {
            return Ok(CallToolResult::error(format!(
                "Unsupported or non-boolean footprint attribute '{name}'"
            )));
        }
        requested.insert(name.clone(), value.as_bool().unwrap());
    }
    match attempt_ipc_write(ctx,&board,"closed-board footprint attributes",|_|Ok(())).await? {
        BoardWrite::Ipc(())=>return Ok(CallToolResult::error("Save and close this exact board before editing saved footprint attributes. No change was made.".to_string())),
        BoardWrite::Refused(result)=>return Ok(result),
        BoardWrite::File(_)=>{}
    }
    if !matches!(
        crate::tools::live_board::editor_lock(&board),
        crate::tools::live_board::EditorLock::Absent
    ) {
        return Ok(CallToolResult::error("The exact board's KiCad lock is present or unreadable; no attribute write was attempted.".to_string()));
    }
    let original = read_consistent(&board)?;
    let (candidate, mut observation) = match prepare(&original, reference, &requested) {
        Ok(result) => result,
        Err(error) => {
            return Ok(CallToolResult::error(format!(
                "No attributes changed: {error:#}"
            )))
        }
    };
    let changed = candidate != original;
    if changed {
        if let Err(error) = write_atomic_if_unchanged(&board, &original, &candidate) {
            return Ok(match read_consistent(&board) {
                Ok(now) if now == original => CallToolResult::error(format!(
                    "Attribute write refused; saved board is unchanged: {error}"
                )),
                _ => crate::tools::mutation_outcome_uncertain(
                    &board,
                    "edit_component attributes",
                    format!(
                        "Atomic writer reported {error}; inspect the saved board before retrying"
                    ),
                ),
            });
        }
        match read_consistent(&board) {
            Ok(now) if now == candidate => {}
            _ => {
                return Ok(crate::tools::mutation_outcome_uncertain(
                    &board,
                    "edit_component attributes",
                    "Could not verify exact persisted attribute edit".to_string(),
                ))
            }
        }
    }
    observation["status"] = json!(if changed { "applied" } else { "noop" });
    observation["source"] = json!("saved");
    observation["readback_verified"] = json!(true);
    observation["other_footprint_clauses"] =
        json!("unchanged byte-for-byte; no whole-footprint typed update");
    Ok(CallToolResult::json(&observation))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tools::pcb_board::board_mock::{ctx_talking_to, spawn_kicad_holding_board};

    const BOARD: &str = r#"(kicad_pcb (version 20260206)
      (footprint "Test:Bga" (layer "F.Cu") (at 20 30 37) (uuid "11111111-1111-4111-8111-111111111111")
        (property "Reference" "U1" (at 0 0)) (attr smd preserve_me)
        (path "/keep/identity")
        (pad "A1" smd custom (at 1 2 37) (size 0.3 0.3) (layers "F.Cu")
          (property pad_prop_bga) (net "GND") (options (anchor circle) (clearance outline))
          (primitives (gr_poly (pts (xy -1 -1) (xy 1 -1) (xy 1 1)) (width 0) (fill yes))))
        (model "keep.step" (offset (xyz 1 2 3)) (rotate (xyz 0 0 37))))
      (footprint "Test:Other" (layer "B.Cu") (property "Reference" "R1" (at 0 0)) (at 80 90))
      (segment (start 1 2) (end 3 4) (width 0.2) (layer "F.Cu") (net "GND")))"#;

    #[test]
    fn attributes_preserve_all_opaque_geometry_nets_models_and_other_items() {
        let requested = FLAGS
            .into_iter()
            .map(|name| (name.to_string(), true))
            .collect();
        let (candidate, observation) = prepare(BOARD, "U1", &requested).unwrap();
        assert_eq!(
            candidate,
            BOARD.replace(
                "(attr smd preserve_me)",
                "(attr smd preserve_me board_only exclude_from_bom exclude_from_pos_files)"
            )
        );
        assert_eq!(observation["flags"]["board_only"], true);
        assert_eq!(observation["uuid"], "11111111-1111-4111-8111-111111111111");
        assert_eq!(prepare(&candidate, "U1", &requested).unwrap().0, candidate);
        let removed = prepare(
            &candidate,
            "U1",
            &BTreeMap::from([("board_only".to_string(), false)]),
        )
        .unwrap()
        .0;
        assert!(removed.contains("(attr smd preserve_me exclude_from_bom exclude_from_pos_files)"));
        assert!(prepare(BOARD, "missing", &requested).is_err());
        let duplicate = BOARD.replace("\"Reference\" \"R1\"", "\"Reference\" \"U1\"");
        assert!(prepare(&duplicate, "U1", &requested).is_err());
    }

    #[test]
    fn absent_attributes_are_inserted_without_rebuilding_the_footprint() {
        let source = BOARD.replace("(attr smd preserve_me)", "");
        let (candidate, _) = prepare(
            &source,
            "U1",
            &BTreeMap::from([("board_only".to_string(), true)]),
        )
        .unwrap();
        assert!(candidate.contains("(attr board_only)"));
        assert!(candidate.contains("(property pad_prop_bga) (net \"GND\")"));
        assert_eq!(prepare(&source, "U1", &BTreeMap::new()).unwrap().0, source);
        assert!(prepare(
            &BOARD.replace("(attr smd preserve_me)", "(attr smd) (attr board_only)"),
            "U1",
            &BTreeMap::new()
        )
        .is_err());
    }

    #[tokio::test]
    async fn open_board_and_exact_sibling_lock_refuse_before_writing() {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("b.kicad_pcb");
        std::fs::write(&board, BOARD).unwrap();
        let args = json!({"board":board,"reference":"U1","attributes":{"board_only":true}});
        let server = spawn_kicad_holding_board(&board, |_| None);
        assert!(
            handle(&args, &ctx_talking_to(server.address().to_string()))
                .await
                .unwrap()
                .is_error
        );
        assert_eq!(std::fs::read_to_string(&board).unwrap(), BOARD);
        drop(server);
        let lock = konnect_sexp::writer::kicad_editor_lock_path(&board).unwrap();
        std::fs::write(&lock, "own-test-lock").unwrap();
        assert!(
            handle(&args, &ctx_talking_to(String::new()))
                .await
                .unwrap()
                .is_error
        );
        assert_eq!(std::fs::read_to_string(&board).unwrap(), BOARD);
        std::fs::remove_file(lock).unwrap();
        let applied = handle(&args, &ctx_talking_to(String::new())).await.unwrap();
        assert!(!applied.is_error, "{applied:?}");
        assert!(std::fs::read_to_string(&board)
            .unwrap()
            .contains("(attr smd preserve_me board_only)"));
        let after = std::fs::read_to_string(&board).unwrap();
        let bad = handle(
            &json!({"board":board,"reference":"U1","attributes":{"board_only":"yes"}}),
            &ctx_talking_to(String::new()),
        )
        .await
        .unwrap();
        assert!(bad.is_error);
        assert_eq!(std::fs::read_to_string(&board).unwrap(), after);
    }
}
