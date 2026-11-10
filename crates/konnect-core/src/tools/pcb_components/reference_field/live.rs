//! The same batch on the board KiCad holds open, over IPC.
//!
//! KiCad 10 replaces a footprint through `UpdateItems` as a whole: it has no
//! proven way to send one field of a placed footprint on its own, and
//! `edit_footprint_fields` goes the same way. So each changed footprint is
//! sent back complete with only its Reference field altered, all of them in
//! one commit, which is one undo step, and every one is read back whole.
//!
//! A whole-footprint update re-creates pads without their fabrication
//! property (`footprint_fabrication`), so that guard runs over every footprint
//! the plan would change, on a dry run too, and refuses the whole batch before
//! anything is sent. The closed-board path keeps those pads byte for byte.
//!
//! A live field's position is its board position and its angle is the board
//! angle; the footprint-frame offset reported and accepted here is that
//! position turned back by the footprint angle about its anchor, the same
//! relation the saved `(at …)` has.

use super::*;
use crate::tools::footprint_fabrication::{check_live_rebuild, RebuildRefusal, RebuildTarget};
use konnect_ipc::{builders, gen::kiapi};

type Footprint = kiapi::board::types::FootprintInstance;
const FP_TYPE: &str = "kiapi.board.types.FootprintInstance";
const DOMAIN: &str = "konnect.set_reference_field_presentations.live.v1";

/// How a live batch ended.
pub(crate) enum Outcome {
    /// A dry run, a no-op or an applied batch.
    Report(Value),
    /// A member cannot be planned; nothing was sent.
    Blocked(Vec<String>),
    /// A footprint the plan would change may not be rebuilt live.
    Fabrication(RebuildRefusal),
    /// The live footprints or the request are not the dry run's.
    Stale,
    /// The commit was sent and did not read back as planned.
    Uncertain(String),
}

fn observe(footprint: &Footprint) -> Result<Observed, String> {
    let position = footprint.position.as_ref().ok_or("has no live position")?;
    let field = footprint
        .reference_field
        .as_ref()
        .ok_or("has no live Reference field")?;
    let board_text = field
        .text
        .as_ref()
        .ok_or("has a Reference field with no text item")?;
    let text = board_text
        .text
        .as_ref()
        .ok_or("has a Reference field with no text")?;
    let at = text
        .position
        .as_ref()
        .ok_or("has a Reference field with no position")?;
    let attributes = text
        .attributes
        .as_ref()
        .ok_or("has a Reference field with no text attributes")?;
    let layer_name = |layer: i32| {
        kiapi::board::types::BoardLayer::try_from(layer)
            .ok()
            .and_then(builders::layer_name)
            .map(str::to_string)
    };
    let footprint_xy = (
        builders::nm_to_mm(position.x_nm),
        builders::nm_to_mm(position.y_nm),
    );
    let footprint_angle = footprint
        .orientation
        .as_ref()
        .map_or(0.0, |angle| angle.value_degrees);
    let (lx, ly) = konnect_sexp::geometry::rotate_about(
        builders::nm_to_mm(at.x_nm) - footprint_xy.0,
        builders::nm_to_mm(at.y_nm) - footprint_xy.1,
        0.0,
        0.0,
        -footprint_angle,
    );
    Ok(Observed {
        footprint: footprint
            .definition
            .as_ref()
            .and_then(|definition| definition.id.as_ref())
            .map(|id| format!("{}:{}", id.library_nickname, id.entry_name)),
        footprint_uuid: footprint.id.as_ref().map(|id| id.value.clone()),
        footprint_xy,
        footprint_angle,
        footprint_layer: layer_name(footprint.layer).ok_or("is on no named layer")?,
        kind: "property",
        field_uuid: board_text.id.as_ref().map(|id| id.value.clone()),
        layer: layer_name(board_text.layer),
        mirrored: attributes.mirrored,
        hidden: !field.visible,
        local: (round_nm(lx), round_nm(ly)),
        angle_deg: normalize(
            attributes
                .angle
                .as_ref()
                .map_or(0.0, |angle| angle.value_degrees),
        ),
        width_mm: attributes
            .size
            .as_ref()
            .map(|size| builders::nm_to_mm(size.x_nm)),
        height_mm: attributes
            .size
            .as_ref()
            .map(|size| builders::nm_to_mm(size.y_nm)),
        stroke_mm: attributes
            .stroke_width
            .as_ref()
            .map(|stroke| builders::nm_to_mm(stroke.value_nm)),
    })
}

/// `footprint` with its Reference field carrying what `after` says and
/// `before` does not; a value the request left alone is not rewritten.
fn updated(footprint: &Footprint, before: &Observed, after: &Observed) -> Footprint {
    let mut out = footprint.clone();
    let field = out.reference_field.as_mut().expect("observed");
    field.visible = !after.hidden;
    let text = field
        .text
        .as_mut()
        .and_then(|board_text| board_text.text.as_mut())
        .expect("observed");
    if before.local != after.local {
        let (x, y) = after.board();
        text.position = Some(builders::vec2(x, y));
    }
    let attributes = text.attributes.as_mut().expect("observed");
    if before.angle_deg != after.angle_deg {
        attributes.angle = Some(kiapi::common::types::Angle {
            value_degrees: after.angle_deg,
        });
    }
    if before.width_mm != after.width_mm || before.height_mm != after.height_mm {
        if let (Some(width), Some(height)) = (after.width_mm, after.height_mm) {
            attributes.size = Some(builders::vec2(width, height));
        }
    }
    if before.stroke_mm != after.stroke_mm {
        if let Some(stroke) = after.stroke_mm {
            attributes.stroke_width = Some(kiapi::common::types::Distance {
                value_nm: builders::mm_to_nm(stroke),
            });
        }
    }
    out
}

/// The live field shows what the plan asked for. KiCad answers in whole
/// nanometres and may express the same angle another way.
fn presented_as(have: &Observed, want: &Observed) -> bool {
    let close = |a: Option<f64>, b: Option<f64>| match (a, b) {
        (Some(a), Some(b)) => (a - b).abs() <= 2e-6,
        (None, None) => true,
        _ => false,
    };
    close(Some(have.local.0), Some(want.local.0))
        && close(Some(have.local.1), Some(want.local.1))
        && normalize(have.angle_deg - want.angle_deg).abs() <= 1e-6
        && close(have.width_mm, want.width_mm)
        && close(have.height_mm, want.height_mm)
        && close(have.stroke_mm, want.stroke_mm)
        && have.hidden == want.hidden
        && have.mirrored == want.mirrored
        && have.layer == want.layer
}

fn reference_of(footprint: &Footprint) -> &str {
    footprint
        .reference_field
        .as_ref()
        .and_then(|field| field.text.as_ref())
        .and_then(|text| text.text.as_ref())
        .map_or("", |text| text.text.as_str())
}

fn by_reference(items: Vec<prost_types::Any>) -> anyhow::Result<BTreeMap<String, Vec<Footprint>>> {
    let mut out: BTreeMap<String, Vec<Footprint>> = BTreeMap::new();
    for item in items {
        if !builders::any_is(&item, FP_TYPE) {
            continue;
        }
        let footprint = Footprint::decode(item.value.as_slice())?;
        out.entry(reference_of(&footprint).to_string())
            .or_default()
            .push(footprint);
    }
    Ok(out)
}

/// Plan, and unless `dry_run` apply, `requests` on the open board at `board`.
pub(crate) fn run(
    client: &KiCadIpcClient,
    board: &Path,
    requests: &[FieldRequest],
    dry_run: bool,
    expected_revision: Option<&str>,
    report_all: bool,
) -> anyhow::Result<Outcome> {
    let document = client.find_open_board(board)?;
    let kind = kiapi::common::types::KiCadObjectType::KotPcbFootprint;
    let live = by_reference(client.get_items_in(document.clone(), kind)?)?;

    let mut reasons = Vec::new();
    let mut seen = HashSet::new();
    let mut members: Vec<(&str, &Footprint, Observed, Observed)> = Vec::new();
    for want in requests {
        let reference = want.reference.as_str();
        if !seen.insert(reference) {
            reasons.push(format!("'{reference}' is requested more than once"));
            continue;
        }
        let footprint = match live.get(reference).map(Vec::as_slice) {
            None | Some([]) => {
                reasons.push(format!("no footprint on the open board is '{reference}'"));
                continue;
            }
            Some([footprint]) => footprint,
            Some(found) => {
                reasons.push(format!(
                    "{} footprints on the open board are '{reference}'",
                    found.len()
                ));
                continue;
            }
        };
        let before = match observe(footprint) {
            Ok(before) => before,
            Err(reason) => {
                reasons.push(format!("'{reference}' {reason}"));
                continue;
            }
        };
        let after = requested(&before, want);
        members.push((reference, footprint, before, after));
    }
    if !reasons.is_empty() {
        return Ok(Outcome::Blocked(reasons));
    }

    let changed: Vec<usize> = (0..members.len())
        .filter(|&i| members[i].2 != members[i].3)
        .collect();
    let targets: Vec<RebuildTarget> = changed
        .iter()
        .map(|&i| RebuildTarget::from_instance(members[i].1))
        .collect();
    if let Err(refusal) = check_live_rebuild(
        board,
        "set_reference_field_presentations",
        &targets,
        crate::tools::footprint_fabrication::Remedy::ClosedBoardPath,
    ) {
        return Ok(Outcome::Fabrication(refusal));
    }

    let normalized = Value::Array(requests.iter().map(FieldRequest::json).collect());
    let mut hasher = Sha256::new();
    for part in [
        DOMAIN.as_bytes(),
        board.to_string_lossy().as_bytes(),
        normalized.to_string().as_bytes(),
    ] {
        hasher.update((part.len() as u64).to_le_bytes());
        hasher.update(part);
    }
    for (_, footprint, _, _) in &members {
        let bytes = footprint.encode_to_vec();
        hasher.update((bytes.len() as u64).to_le_bytes());
        hasher.update(bytes);
    }
    let revision = format!("{:x}", hasher.finalize());

    let listed: Vec<Value> = members
        .iter()
        .filter(|(_, _, before, after)| report_all || before != after)
        .map(|(reference, _, before, after)| {
            json!({
                "reference": reference,
                "changed": before != after,
                "before": before.json(reference),
                "after": after.json(reference),
            })
        })
        .collect();
    let mut body = json!({
        "board": board.display().to_string(),
        "status": if changed.is_empty() { "noop" } else { "ready" },
        "dry_run": dry_run,
        "revision": revision,
        "requested_count": requests.len(),
        "changed_count": changed.len(),
        "unchanged_count": requests.len() - changed.len(),
        "report": if report_all { "all" } else { "changed" },
        "fields": listed,
        "convention": CONVENTION,
        "source": "ipc",
        "saved": false,
    });
    if dry_run {
        return Ok(Outcome::Report(body));
    }
    if expected_revision != Some(revision.as_str()) {
        return Ok(Outcome::Stale);
    }
    if changed.is_empty() {
        return Ok(Outcome::Report(body));
    }

    let planned: Vec<(&str, Footprint, &Observed)> = changed
        .iter()
        .map(|&i| {
            let (reference, footprint, before, after) = &members[i];
            (*reference, updated(footprint, before, after), after)
        })
        .collect();
    let packed = planned
        .iter()
        .map(|(_, footprint, _)| builders::pack_any(footprint, FP_TYPE))
        .collect();
    // KiCad stages each footprint replacement until the commit is pushed, so
    // the readback follows it.
    let outcome = client
        .run_commit("Set reference field presentations", |c| {
            c.update_items_in(document.clone(), packed)
        })
        .and_then(|_| {
            let mut now = by_reference(client.get_items_in(document.clone(), kind)?)?;
            for (reference, want, after) in &planned {
                let [mut have] = now
                    .remove(*reference)
                    .unwrap_or_default()
                    .try_into()
                    .map_err(|found: Vec<Footprint>| {
                        anyhow::anyhow!("'{reference}' reads back as {} footprints", found.len())
                    })?;
                let seen = observe(&have).map_err(|e| anyhow::anyhow!("'{reference}' {e}"))?;
                if !presented_as(&seen, after) {
                    anyhow::bail!(
                        "'{reference}' Reference field reads back as {}, not the planned {}",
                        seen.json(reference),
                        after.json(reference)
                    );
                }
                // Everything but the Reference field is the footprint as sent.
                have.reference_field = want.reference_field.clone();
                if crate::tools::pcb_fields::comparable(&have)?
                    != crate::tools::pcb_fields::comparable(want)?
                {
                    anyhow::bail!("'{reference}' changed outside its Reference field");
                }
            }
            Ok(())
        });
    Ok(match outcome {
        Ok(()) => {
            body["status"] = json!("applied");
            body["undo"] = json!("one KiCad undo step for the whole batch");
            body["readback"] = json!({ "footprints_match_plan": planned.len() });
            body["next"] = json!("save_project (or save in KiCad) to persist the change");
            Outcome::Report(body)
        }
        Err(error) => Outcome::Uncertain(format!("{error:#}")),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tools::pcb_board::board_mock::{ctx_talking_to, spawn_kicad_holding_board};
    use kiapi::common::{commands as cmd, types as ct};
    use std::sync::{Arc, Mutex};

    /// What the mock KiCad holds: its footprints, the ones an open commit has
    /// staged, and how many commits and `UpdateItems` items it has seen.
    #[derive(Default)]
    struct Editor {
        footprints: Vec<Footprint>,
        staged: Vec<Footprint>,
        commits: usize,
        updated_items: usize,
    }

    fn footprint(reference: &str, kiid: &str, x: f64, y: f64, angle: f64) -> Footprint {
        Footprint {
            id: Some(ct::Kiid { value: kiid.into() }),
            position: Some(builders::vec2(x, y)),
            orientation: Some(ct::Angle {
                value_degrees: angle,
            }),
            layer: builders::layer_from_name("F.Cu") as i32,
            reference_field: Some(kiapi::board::types::Field {
                name: "Reference".into(),
                visible: true,
                text: Some(kiapi::board::types::BoardText {
                    id: Some(ct::Kiid {
                        value: format!("f{}", &kiid[1..]),
                    }),
                    text: Some(ct::Text {
                        text: reference.into(),
                        // One millimetre above the anchor on the board.
                        position: Some(builders::vec2(x, y - 1.0)),
                        attributes: Some(ct::TextAttributes {
                            size: Some(builders::vec2(1.0, 1.0)),
                            stroke_width: Some(ct::Distance { value_nm: 150_000 }),
                            angle: Some(ct::Angle {
                                value_degrees: angle,
                            }),
                            ..Default::default()
                        }),
                        ..Default::default()
                    }),
                    layer: builders::layer_from_name("F.SilkS") as i32,
                    ..Default::default()
                }),
                ..Default::default()
            }),
            definition: Some(kiapi::board::types::Footprint {
                id: Some(ct::LibraryIdentifier {
                    library_nickname: "Parts".into(),
                    entry_name: "Plain".into(),
                }),
                items: vec![builders::pack_any(
                    &kiapi::board::types::Pad {
                        number: "1".into(),
                        ..Default::default()
                    },
                    "kiapi.board.types.Pad",
                )],
                ..Default::default()
            }),
            ..Default::default()
        }
    }

    const R1: &str = "a0000000-0000-4000-8000-000000000001";
    const R2: &str = "a0000000-0000-4000-8000-000000000002";
    const U3: &str = "a0000000-0000-4000-8000-000000000003";

    /// The saved copy the fabrication guard reads: R1 and R2 are plain, U3
    /// has a heatsink pad the typed API cannot carry.
    fn saved_board() -> String {
        let footprint = |reference: &str, kiid: &str, pad: &str| {
            format!(
                "\t(footprint \"Parts:Plain\" (layer \"F.Cu\") (uuid \"{kiid}\") (at 0 0)\n\
                 \t\t(property \"Reference\" \"{reference}\" (at 0 -1 0) (layer \"F.SilkS\") \
                 (effects (font (size 1 1) (thickness 0.15))))\n\
                 \t\t(pad \"1\" smd rect (at 0 0) (size 1 1) (layers \"F.Cu\"){pad}))\n"
            )
        };
        format!(
            "(kicad_pcb (version 20260206)\n{}{}{})\n",
            footprint("R1", R1, ""),
            footprint("R2", R2, ""),
            footprint("U3", U3, " (property pad_prop_heatsink)")
        )
    }

    struct Bench {
        _tmp: tempfile::TempDir,
        board: std::path::PathBuf,
        editor: Arc<Mutex<Editor>>,
        server: crate::test_support::MockIpcServer,
    }

    /// A KiCad holding the board open that stages footprint replacements
    /// until `EndCommit`, as KiCad 10 does. `after_commit` then alters what
    /// it holds, standing in for an editor that did something else.
    fn bench(after_commit: impl Fn(&mut Vec<Footprint>) + Send + 'static) -> Bench {
        let tmp = tempfile::tempdir().unwrap();
        let board = tmp.path().join("live.kicad_pcb");
        std::fs::write(&board, saved_board()).unwrap();
        let editor = Arc::new(Mutex::new(Editor {
            footprints: vec![
                footprint("R1", R1, 10.0, 10.0, 0.0),
                footprint("R2", R2, 20.0, 10.0, 90.0),
                footprint("U3", U3, 30.0, 10.0, 0.0),
            ],
            ..Default::default()
        }));
        let held = editor.clone();
        let server = spawn_kicad_holding_board(&board, move |command| {
            let mut editor = held.lock().unwrap();
            let url = command.type_url.as_str();
            Some(if url.ends_with("GetItems") {
                builders::pack_any(
                    &cmd::GetItemsResponse {
                        header: None,
                        status: ct::ItemRequestStatus::IrsOk as i32,
                        items: editor
                            .footprints
                            .iter()
                            .map(|footprint| builders::pack_any(footprint, FP_TYPE))
                            .collect(),
                    },
                    "kiapi.common.commands.GetItemsResponse",
                )
            } else if url.ends_with("BeginCommit") {
                editor.commits += 1;
                builders::pack_any(
                    &cmd::BeginCommitResponse {
                        id: Some(ct::Kiid {
                            value: "commit".into(),
                        }),
                    },
                    "kiapi.common.commands.BeginCommitResponse",
                )
            } else if url.ends_with("UpdateItems") {
                let update = cmd::UpdateItems::decode(command.value.as_slice()).unwrap();
                editor.updated_items += update.items.len();
                for item in &update.items {
                    editor
                        .staged
                        .push(Footprint::decode(item.value.as_slice()).unwrap());
                }
                builders::pack_any(
                    &cmd::UpdateItemsResponse {
                        header: None,
                        status: ct::ItemRequestStatus::IrsOk as i32,
                        updated_items: update
                            .items
                            .into_iter()
                            .map(|item| cmd::ItemUpdateResult {
                                status: Some(cmd::ItemStatus {
                                    code: cmd::ItemStatusCode::IscOk as i32,
                                    error_message: String::new(),
                                }),
                                item: Some(item),
                            })
                            .collect(),
                    },
                    "kiapi.common.commands.UpdateItemsResponse",
                )
            } else if url.ends_with("EndCommit") {
                let staged = std::mem::take(&mut editor.staged);
                for replacement in staged {
                    let held = editor
                        .footprints
                        .iter_mut()
                        .find(|footprint| footprint.id == replacement.id)
                        .expect("an update names a footprint KiCad holds");
                    *held = replacement;
                }
                after_commit(&mut editor.footprints);
                builders::pack_any(
                    &cmd::EndCommitResponse {},
                    "kiapi.common.commands.EndCommitResponse",
                )
            } else {
                return None;
            })
        });
        Bench {
            _tmp: tmp,
            board,
            editor,
            server,
        }
    }

    async fn call(bench: &Bench, mut args: Value) -> (bool, Value) {
        args["board"] = json!(bench.board);
        let def = crate::tools::placement::tools()
            .into_iter()
            .find(|tool| tool.name == "set_reference_field_presentations")
            .unwrap();
        assert!(def.input_validator.is_valid(&args), "schema refused {args}");
        let ctx = ctx_talking_to(bench.server.address().to_string());
        let result = (def.handler)(&args, Arc::new(ctx)).await.unwrap();
        let body = match &result.content[0] {
            crate::mcp::protocol::ToolContent::Text { text } => {
                serde_json::from_str(text).unwrap_or_else(|_| json!({ "text": text }))
            }
            other => panic!("expected text, got {other:?}"),
        };
        (result.is_error, body)
    }

    fn reference_field(editor: &Editor, index: usize) -> kiapi::board::types::Field {
        editor.footprints[index].reference_field.clone().unwrap()
    }

    #[tokio::test]
    async fn an_open_board_takes_the_whole_batch_in_one_commit_and_reads_it_back() {
        let bench = bench(|_| {});
        let before = bench.editor.lock().unwrap().footprints.clone();
        let saved = std::fs::read_to_string(&bench.board).unwrap();
        // R2 sits at (20, 10) turned 90°: board (22, 10) is local (0, 2).
        let fields = json!([
            { "reference": "R1", "visible": false },
            { "reference": "R2", "board_x": 22, "board_y": 10, "relative_angle_deg": 0,
              "width_mm": 0.8, "height_mm": 0.9, "stroke_mm": 0.12 },
            { "reference": "U3" }
        ]);
        let (error, plan) = call(&bench, json!({ "fields": fields, "report": "all" })).await;
        assert!(!error, "{plan}");
        assert_eq!(plan["source"], "ipc");
        assert_eq!(plan["status"], "ready");
        assert_eq!(plan["changed_count"], 2);
        assert_eq!(plan["unchanged_count"], 1, "U3 is only read");
        assert_eq!(plan["fields"][0]["after"]["visible"], false);
        assert_eq!(plan["fields"][1]["before"]["local_x"], 1.0);
        assert_eq!(plan["fields"][1]["after"]["local_x"], 0.0);
        assert_eq!(plan["fields"][1]["after"]["local_y"], 2.0);
        assert_eq!(plan["fields"][1]["after"]["angle_deg"], 90.0);
        assert_eq!(
            bench.editor.lock().unwrap().commits,
            0,
            "a dry run sends nothing"
        );

        let mut apply =
            json!({ "fields": fields, "dry_run": false, "expected_revision": "0".repeat(64) });
        let (error, stale) = call(&bench, apply.clone()).await;
        assert!(error);
        assert_eq!(stale["error"]["reason"], "stale_revision", "{stale}");
        assert_eq!(bench.editor.lock().unwrap().commits, 0);

        apply["expected_revision"] = plan["revision"].clone();
        let (error, applied) = call(&bench, apply.clone()).await;
        assert!(!error, "{applied}");
        assert_eq!(applied["status"], "applied");
        assert_eq!(applied["saved"], false);
        assert_eq!(applied["readback"]["footprints_match_plan"], 2);
        {
            let editor = bench.editor.lock().unwrap();
            assert_eq!(editor.commits, 1, "one commit is one undo step");
            assert_eq!(
                editor.updated_items, 2,
                "the unchanged footprint is not sent"
            );
            assert!(!reference_field(&editor, 0).visible);
            let moved = reference_field(&editor, 1).text.unwrap().text.unwrap();
            assert_eq!(moved.position, Some(builders::vec2(22.0, 10.0)));
            let attributes = moved.attributes.unwrap();
            assert_eq!(attributes.size, Some(builders::vec2(0.8, 0.9)));
            assert_eq!(attributes.stroke_width.unwrap().value_nm, 120_000);
            assert_eq!(attributes.angle.unwrap().value_degrees, 90.0);
            // Nothing but the two Reference fields differs from what KiCad held.
            let mut restored = editor.footprints.clone();
            restored[0].reference_field = before[0].reference_field.clone();
            restored[1].reference_field = before[1].reference_field.clone();
            assert_eq!(restored, before);
        }
        assert_eq!(
            std::fs::read_to_string(&bench.board).unwrap(),
            saved,
            "the live path never writes the file"
        );

        // The applied revision is spent, and the same request is now a no-op.
        let (error, spent) = call(&bench, apply).await;
        assert!(error, "{spent}");
        let (error, again) = call(&bench, json!({ "fields": fields })).await;
        assert!(!error);
        assert_eq!(again["status"], "noop");
        assert_eq!(bench.editor.lock().unwrap().commits, 1);
    }

    #[tokio::test]
    async fn a_footprint_with_a_pad_fabrication_property_refuses_the_whole_live_batch() {
        let bench = bench(|_| {});
        let fields = json!([
            { "reference": "R1", "visible": false },
            { "reference": "U3", "visible": false }
        ]);
        let (error, refused) = call(&bench, json!({ "fields": fields })).await;
        assert!(error, "the dry run already refuses: {refused}");
        assert_eq!(refused["error"]["kind"], "plan_blocked", "{refused}");
        let text = refused.to_string();
        assert!(text.contains("pad_prop_heatsink"), "{text}");
        assert!(text.contains("close it in KiCad"), "{text}");

        // A request cannot reach the commit either, whatever revision it brings.
        let (error, _) = call(
            &bench,
            json!({ "fields": fields, "dry_run": false, "expected_revision": "0".repeat(64) }),
        )
        .await;
        assert!(error);
        let editor = bench.editor.lock().unwrap();
        assert_eq!((editor.commits, editor.updated_items), (0, 0));
        assert!(
            reference_field(&editor, 0).visible,
            "R1 was not hidden alone"
        );
    }

    #[tokio::test]
    async fn an_unknown_or_repeated_reference_blocks_the_live_batch_before_anything_is_sent() {
        let bench = bench(|_| {});
        for fields in [
            json!([{ "reference": "R1", "visible": false }, { "reference": "R99", "visible": false }]),
            json!([{ "reference": "R1", "visible": false }, { "reference": "R1", "local_x": 1 }]),
        ] {
            let (error, refused) = call(&bench, json!({ "fields": fields })).await;
            assert!(error);
            assert_eq!(refused["error"]["kind"], "plan_blocked", "{refused}");
        }
        assert_eq!(bench.editor.lock().unwrap().commits, 0);
    }

    #[tokio::test]
    async fn a_commit_that_does_not_read_back_as_planned_is_reported_uncertain() {
        // The editor drops the pad of the footprint it replaced.
        let bench = bench(|footprints| {
            footprints[0].definition.as_mut().unwrap().items.clear();
        });
        let fields = json!([{ "reference": "R1", "visible": false }]);
        let (_, plan) = call(&bench, json!({ "fields": fields })).await;
        let (error, outcome) = call(
            &bench,
            json!({ "fields": fields, "dry_run": false, "expected_revision": plan["revision"] }),
        )
        .await;
        assert!(error, "{outcome}");
        assert_eq!(
            outcome["error"]["kind"], "mutation_outcome_uncertain",
            "{outcome}"
        );
        assert!(outcome.to_string().contains("outside its Reference field"));
    }
}
