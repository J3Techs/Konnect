//! Guarded live footprint assembly attributes and pad paste overrides.
use super::{
    pcb_board::{attempt_ipc_write, BoardWrite},
    ToolContext, ToolDef,
};
use crate::{mcp::protocol::CallToolResult, tool};
use anyhow::{bail, Context, Result};
use konnect_ipc::{builders, gen::kiapi};
use prost::Message;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
type Footprint = kiapi::board::types::FootprintInstance;
type Pad = kiapi::board::types::Pad;
const FP: &str = "kiapi.board.types.FootprintInstance";
const PAD: &str = "kiapi.board.types.Pad";
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
enum MountingStyle {
    Smd,
    ThroughHole,
    Unspecified,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct PadEdit {
    number: String,
    expected_pad_count: usize,
    solder_paste_margin_mm: f64,
    solder_paste_margin_ratio: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Args {
    board: String,
    reference: String,
    mounting_style: Option<MountingStyle>,
    pads: Option<Vec<PadEdit>>,
    #[serde(default = "yes")]
    dry_run: bool,
    expected_plan_revision: Option<String>,
}
fn yes() -> bool {
    true
}
fn validate(a: &Args) -> Result<()> {
    if a.board.trim().is_empty() || a.reference.trim().is_empty() {
        bail!("board and reference must be nonempty")
    }
    if !a.dry_run
        && a.expected_plan_revision
            .as_ref()
            .is_none_or(|v| v.trim().is_empty())
    {
        bail!("expected_plan_revision required for apply")
    }
    if let Some(pads) = &a.pads {
        if pads.is_empty() {
            bail!("pads must be nonempty when present")
        }
        let mut numbers = BTreeSet::new();
        for p in pads {
            if p.number.trim().is_empty() || !numbers.insert(&p.number) {
                bail!("pad numbers must be nonempty and unique")
            }
            if p.expected_pad_count == 0 {
                bail!("expected_pad_count must be positive")
            }
            if !p.solder_paste_margin_mm.is_finite() || p.solder_paste_margin_mm.abs() > 1.0 {
                bail!("solder_paste_margin_mm outside +/-1 mm")
            }
            if !p.solder_paste_margin_ratio.is_finite()
                || !(-0.49..=1.0).contains(&p.solder_paste_margin_ratio)
            {
                bail!("solder_paste_margin_ratio outside -0.49 to 1")
            }
        }
    }
    Ok(())
}
fn prepare(fp: &Footprint, a: &Args) -> Result<Footprint> {
    let mut out = fp.clone();
    if let Some(style) = &a.mounting_style {
        out.attributes.get_or_insert_default().mounting_style = match style {
            MountingStyle::Smd => 2,
            MountingStyle::ThroughHole => 1,
            MountingStyle::Unspecified => 3,
        };
    }
    if let Some(edits) = &a.pads {
        let items = &mut out
            .definition
            .as_mut()
            .context("missing footprint definition")?
            .items;
        for edit in edits {
            let mut count = 0;
            for item in items.iter_mut() {
                if !builders::any_is(item, PAD) {
                    continue;
                }
                let mut pad = Pad::decode(item.value.as_slice())?;
                if pad.number != edit.number {
                    continue;
                }
                if pad.encode_to_vec() != item.value {
                    bail!("pad contains unsupported wire data; refusing lossy edit")
                }
                count += 1;
                let stack = pad.pad_stack.as_mut().context("missing pad stack")?;
                if stack.copper_layers.is_empty() {
                    bail!("pad has no copper geometry")
                }
                for layer in &stack.copper_layers {
                    let size = layer.size.as_ref().context("pad size missing")?;
                    for dimension in [size.x_nm, size.y_nm] {
                        let aperture = dimension as f64
                            * (1.0 + 2.0 * edit.solder_paste_margin_ratio)
                            + 2.0 * edit.solder_paste_margin_mm * 1e6;
                        if aperture <= 0.0 {
                            bail!("paste overrides produce nonpositive aperture")
                        }
                    }
                }
                // KiCad currently serializes symmetric front/back paste overrides.
                for outer in [&mut stack.front_outer_layers, &mut stack.back_outer_layers] {
                    let settings = outer
                        .get_or_insert_default()
                        .solder_paste_settings
                        .get_or_insert_default();
                    settings.solder_paste_margin = Some(kiapi::common::types::Distance {
                        value_nm: builders::mm_to_nm(edit.solder_paste_margin_mm),
                    });
                    settings.solder_paste_margin_ratio = Some(kiapi::common::types::Ratio {
                        value: edit.solder_paste_margin_ratio,
                    });
                }
                *item = builders::pack_any(&pad, PAD);
            }
            if count != edit.expected_pad_count {
                bail!(
                    "pad '{}' expected {} physical pads, found {}",
                    edit.number,
                    edit.expected_pad_count,
                    count
                )
            }
        }
    }
    Ok(out)
}
fn name(fp: &Footprint) -> &str {
    fp.reference_field
        .as_ref()
        .and_then(|f| f.text.as_ref())
        .and_then(|t| t.text.as_ref())
        .map(|t| t.text.as_str())
        .unwrap_or("")
}
fn select(items: Vec<prost_types::Any>, reference: &str) -> Result<Footprint> {
    let mut found = None;
    for i in items {
        if !builders::any_is(&i, FP) {
            bail!("unexpected footprint response type")
        }
        let fp = Footprint::decode(i.value.as_slice())?;
        if fp.encode_to_vec() != i.value {
            bail!("footprint contains unsupported wire data")
        }
        if name(&fp) == reference {
            if found.is_some() {
                bail!("ambiguous reference")
            };
            found = Some(fp);
        }
    }
    found.context("footprint not found")
}
fn comparable(fp: &Footprint) -> Footprint {
    let mut out = fp.clone();
    if let Some(d) = &mut out.definition {
        d.items
            .sort_by(|a, b| (&a.type_url, &a.value).cmp(&(&b.type_url, &b.value)));
    }
    out
}
fn revision(path: &str, fp: &Footprint, a: &Args) -> Result<String> {
    let mut h = Sha256::new();
    h.update(path.as_bytes());
    h.update(fp.encode_to_vec());
    h.update(serde_json::to_vec(&(&a.mounting_style, &a.pads))?);
    Ok(format!("{:x}", h.finalize()))
}
fn view(fp: &Footprint) -> Result<Value> {
    let mut pads = vec![];
    for item in &fp.definition.as_ref().context("missing definition")?.items {
        if builders::any_is(item, PAD) {
            let p = Pad::decode(item.value.as_slice())?;
            let s = p.pad_stack.as_ref().context("missing pad stack")?;
            let paste = |o: &Option<kiapi::board::types::PadStackOuterLayer>| {
                let p = o.as_ref().and_then(|o| o.solder_paste_settings.as_ref());
                json!({"margin_mm":p.and_then(|p|p.solder_paste_margin.as_ref()).map(|v|v.value_nm as f64/1e6),"margin_ratio":p.and_then(|p|p.solder_paste_margin_ratio.as_ref()).map(|v|v.value)})
            };
            pads.push(json!({"number":p.number,"uuid":p.id.as_ref().map(|v|&v.value),"front_paste":paste(&s.front_outer_layers),"back_paste":paste(&s.back_outer_layers)}));
        }
    }
    Ok(json!({"mounting_style":fp.attributes.as_ref().map(|a|a.mounting_style),"pads":pads}))
}
pub(crate) fn tool() -> ToolDef {
    tool!("edit_footprint_fabrication","Inspect, plan or apply a live footprint's mounting style and symmetric pad paste overrides. Omit mounting_style and pads to inspect. Paste edits require both margin values and exact physical pad count for each logical pad number; zero is explicit, ratios are fractions (negative 0.2 reduces each dimension by 40 percent). Defaults to dry run; apply requires the current plan revision. Preserves other attributes, geometry, nets and routing. One undo commit and complete footprint readback; no file fallback. Does not validate stencil suitability or save the document.",json!({"type":"object","properties":{"board":{"type":"string"},"reference":{"type":"string"},"mounting_style":{"type":"string","enum":["smd","through_hole","unspecified"]},"pads":{"type":"array","minItems":1,"items":{"type":"object","properties":{"number":{"type":"string","minLength":1},"expected_pad_count":{"type":"integer","minimum":1},"solder_paste_margin_mm":{"type":"number","minimum":-1,"maximum":1},"solder_paste_margin_ratio":{"type":"number","minimum":-0.49,"maximum":1}},"required":["number","expected_pad_count","solder_paste_margin_mm","solder_paste_margin_ratio"],"additionalProperties":false}},"dry_run":{"type":"boolean","default":true},"expected_plan_revision":{"type":"string","minLength":1}},"required":["board","reference"],"additionalProperties":false}),|args,ctx|async move{handle_edit_footprint_fabrication(args,ctx).await}).with_board_access(super::BoardAccess::LiveOnly)
}
async fn handle_edit_footprint_fabrication(
    args: &Value,
    ctx: &ToolContext,
) -> Result<CallToolResult> {
    fn null(v: &Value) -> bool {
        match v {
            Value::Null => true,
            Value::Array(a) => a.iter().any(null),
            Value::Object(m) => m.values().any(null),
            _ => false,
        }
    }
    for key in ["dry_run", "expected_plan_revision"] {
        if args.get(key).is_some_and(Value::is_null) {
            return Ok(super::invalid_arg(
                key,
                "null is unsupported; omit unchanged options",
            ));
        }
    }
    if null(args) {
        return Ok(super::invalid_arg(
            "arguments",
            "null is unsupported; omit unchanged options",
        ));
    }
    let mut normalized = args.clone();
    if let Some(pads) = normalized.get_mut("pads").and_then(Value::as_array_mut) {
        for pad in pads {
            match super::opt_u32(pad, "expected_pad_count") {
                Ok(Some(count)) => pad["expected_pad_count"] = json!(count),
                Ok(None) => return Ok(super::invalid_arg("expected_pad_count", "required")),
                Err(error) => return Ok(error),
            }
        }
    }
    let a: Args = match serde_json::from_value(normalized) {
        Ok(a) => a,
        Err(e) => return Ok(super::invalid_arg("arguments", &e.to_string())),
    };
    if let Err(e) = validate(&a) {
        return Ok(super::invalid_arg("arguments", &e.to_string()));
    }
    let board = super::get_path(args, "board")?;
    let path = board.clone();
    let result=attempt_ipc_write(ctx,&board,"footprint fabrication",move |client|{
  let doc=client.find_open_board(&path)?;let kind=kiapi::common::types::KiCadObjectType::KotPcbFootprint;
  let fp=select(client.get_items_in(doc.clone(),kind)?,&a.reference)?;
  if a.mounting_style.is_none()&&a.pads.is_none(){return Ok(CallToolResult::json(&json!({"status":"inspected","source":"ipc","board":path,"reference":a.reference,"fabrication":view(&fp)?})));}
  let updated=match prepare(&fp,&a){Ok(u)=>u,Err(e)=>return Ok(CallToolResult::json(&json!({"status":"conflict","applied":false,"source":"ipc","diagnostic":e.to_string()})))};
  let rev=revision(&path.to_string_lossy(),&fp,&a)?;
  if !a.dry_run&&a.expected_plan_revision.as_deref()!=Some(&rev){return Ok(CallToolResult::json(&json!({"status":"conflict","applied":false,"source":"ipc","diagnostic":"stale_plan_revision"})));}
  let changed=comparable(&fp)!=comparable(&updated);
  if a.dry_run||!changed{return Ok(CallToolResult::json(&json!({"status":if changed{"ready"}else{"noop"},"applied":false,"source":"ipc","plan_revision":rev,"before":view(&fp)?,"after":view(&updated)?})));}
  let outcome=client.run_commit("Edit footprint fabrication",|c|c.update_items_in(doc.clone(),vec![builders::pack_any(&updated,FP)])).and_then(|()|{let actual=select(client.get_items_in(doc.clone(),kind)?,&a.reference)?;if comparable(&actual)!=comparable(&updated){bail!("complete footprint readback differs; inspect before retrying")};Ok(actual)});
  match outcome {
   Ok(actual)=>Ok(CallToolResult::json(&json!({"status":"complete","applied":true,"source":"ipc","saved":false,"plan_revision":rev,"fabrication":view(&actual)?}))),
   Err(e)=>{let unchanged=client.get_items_in(doc,kind).and_then(|i|select(i,&a.reference)).map(|f|comparable(&f)==comparable(&fp)).unwrap_or(false);Ok(CallToolResult::json(&json!({"status":if unchanged{"unchanged_after_failure"}else{"uncertain"},"applied":if unchanged{json!(false)}else{Value::Null},"source":"ipc","diagnostic":format!("{e:#}"),"recovery":"Inspect live document and compare with pre-edit state before retrying; do not assume rollback"})))}
  }
 }).await?;
    Ok(match result {
        BoardWrite::Ipc(r) | BoardWrite::Refused(r) => r,
        BoardWrite::File(reason) => CallToolResult::error(format!(
            "{} Presentation edits require live IPC; no file fallback.",
            reason.premise()
        )),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    fn fixture() -> Footprint {
        let mut fp = Footprint::decode(
            include_bytes!("../../tests/fixtures/board_only_shared_reference.ipc.bin").as_slice(),
        )
        .unwrap();
        // The captured board-only footprint has an unnumbered pad. Give it a
        // logical number for selection tests without changing its geometry.
        let item = fp
            .definition
            .as_mut()
            .unwrap()
            .items
            .iter_mut()
            .find(|i| builders::any_is(i, PAD))
            .unwrap();
        let mut pad = Pad::decode(item.value.as_slice()).unwrap();
        pad.number = "9".into();
        *item = builders::pack_any(&pad, PAD);
        fp
    }
    #[test]
    fn captured_parent_fields_round_trip_without_loss() {
        let bytes = include_bytes!("../../tests/fixtures/board_only_shared_reference.ipc.bin");
        let fp = Footprint::decode(bytes.as_slice()).unwrap();
        assert_eq!(fp.encode_to_vec(), bytes.as_slice());
        for item in &fp.definition.as_ref().unwrap().items {
            if builders::any_is(item, PAD) {
                let pad = Pad::decode(item.value.as_slice()).unwrap();
                assert_eq!(pad.encode_to_vec(), item.value);
                assert!(pad.parent.is_some());
            }
        }
    }
    fn request() -> Value {
        json!({"board":"/field-test.kicad_pcb","reference":name(&fixture()),"mounting_style":"smd"})
    }
    fn paste_request(fp: &Footprint) -> Args {
        let pad = fp
            .definition
            .as_ref()
            .unwrap()
            .items
            .iter()
            .find(|i| {
                builders::any_is(i, PAD)
                    && !Pad::decode(i.value.as_slice()).unwrap().number.is_empty()
            })
            .unwrap();
        let pad = Pad::decode(pad.value.as_slice()).unwrap();
        let count = fp
            .definition
            .as_ref()
            .unwrap()
            .items
            .iter()
            .filter(|i| {
                builders::any_is(i, PAD)
                    && Pad::decode(i.value.as_slice()).unwrap().number == pad.number
            })
            .count();
        serde_json::from_value(json!({"board":"a.kicad_pcb","reference":name(fp),"pads":[{"number":pad.number,"expected_pad_count":count,"solder_paste_margin_mm":0,"solder_paste_margin_ratio":0}]})).unwrap()
    }
    #[test]
    fn mounting_change_preserves_every_other_attribute_and_child() {
        let fp = fixture();
        let a = serde_json::from_value(request()).unwrap();
        let mut out = prepare(&fp, &a).unwrap();
        assert_eq!(out.attributes.as_ref().unwrap().mounting_style, 2);
        out.attributes = fp.attributes.clone();
        assert_eq!(out, fp);
    }
    #[test]
    fn explicit_zero_paste_preserves_pad_geometry_and_nets() {
        let fp = fixture();
        let a = paste_request(&fp);
        let out = prepare(&fp, &a).unwrap();
        for (before, after) in fp
            .definition
            .as_ref()
            .unwrap()
            .items
            .iter()
            .zip(&out.definition.as_ref().unwrap().items)
        {
            if !builders::any_is(before, PAD) {
                assert_eq!(before, after);
                continue;
            }
            let p = Pad::decode(before.value.as_slice()).unwrap();
            let mut q = Pad::decode(after.value.as_slice()).unwrap();
            if p.number == a.pads.as_ref().unwrap()[0].number {
                let old = p.pad_stack.as_ref().unwrap();
                let stack = q.pad_stack.as_mut().unwrap();
                for outer in [&stack.front_outer_layers, &stack.back_outer_layers] {
                    let settings = outer
                        .as_ref()
                        .unwrap()
                        .solder_paste_settings
                        .as_ref()
                        .unwrap();
                    assert_eq!(settings.solder_paste_margin.as_ref().unwrap().value_nm, 0);
                    assert_eq!(
                        settings.solder_paste_margin_ratio.as_ref().unwrap().value,
                        0.0
                    );
                }
                stack.front_outer_layers = old.front_outer_layers;
                stack.back_outer_layers = old.back_outer_layers;
            }
            assert_eq!(p, q);
        }
    }
    #[test]
    fn wrong_physical_count_and_nonpositive_aperture_refuse() {
        let fp = fixture();
        let mut a = paste_request(&fp);
        a.pads.as_mut().unwrap()[0].expected_pad_count += 1;
        assert!(prepare(&fp, &a).is_err());
        let mut a = paste_request(&fp);
        a.pads.as_mut().unwrap()[0].solder_paste_margin_mm = -1.0;
        a.pads.as_mut().unwrap()[0].solder_paste_margin_ratio = -0.49;
        assert!(prepare(&fp, &a).is_err());
    }
    #[test]
    fn invalid_requests_and_unknown_wire_data_refuse() {
        let fp = fixture();
        let mut a = paste_request(&fp);
        let edit = a.pads.as_ref().unwrap()[0].clone();
        a.pads.as_mut().unwrap().push(edit);
        assert!(validate(&a).is_err());
        for value in [
            json!({"pads":[]}),
            json!({"dry_run":false}),
            json!({"reference":" "}),
        ] {
            let mut v = request();
            for (k, x) in value.as_object().unwrap() {
                v[k] = x.clone()
            }
            assert!(validate(&serde_json::from_value(v).unwrap()).is_err());
        }
        let mut item = builders::pack_any(&fp, FP);
        item.value.extend([0x98, 0x06, 1]);
        assert!(select(vec![item], name(&fp)).is_err());
        let mut changed = fp.clone();
        let a = paste_request(&fp);
        let item = changed
            .definition
            .as_mut()
            .unwrap()
            .items
            .iter_mut()
            .find(|i| {
                builders::any_is(i, PAD)
                    && Pad::decode(i.value.as_slice()).unwrap().number
                        == a.pads.as_ref().unwrap()[0].number
            })
            .unwrap();
        item.value.extend([0x98, 0x06, 1]);
        assert!(prepare(&changed, &a).is_err());
        assert!(select(
            vec![builders::pack_any(&fp, FP), builders::pack_any(&fp, FP)],
            name(&fp)
        )
        .is_err());
    }
    #[test]
    fn revision_binds_request_geometry_and_path() {
        let fp = fixture();
        let a: Args = serde_json::from_value(request()).unwrap();
        let rev = revision("a", &fp, &a).unwrap();
        assert_ne!(rev, revision("b", &fp, &a).unwrap());
        let mut other = fp.clone();
        other.position = None;
        assert_ne!(rev, revision("a", &other, &a).unwrap());
        let mut b = a.clone();
        b.mounting_style = Some(MountingStyle::ThroughHole);
        assert_ne!(rev, revision("a", &fp, &b).unwrap());
    }
    fn body(result: CallToolResult) -> Value {
        match &result.content[0] {
            crate::mcp::protocol::ToolContent::Text { text } => serde_json::from_str(text).unwrap(),
            other => panic!("{other:?}"),
        }
    }
    // The double deliberately exposes the OLD footprint until EndCommit,
    // reproducing KiCad's staged Remove/Add behavior observed in 10.0.6.
    #[tokio::test]
    async fn commit_lifecycle_verifies_published_state_and_reports_uncertain_changes() {
        use kiapi::common::{commands as cmd, types as ct};
        use std::sync::{Arc, Mutex};
        for corrupt_readback in [false, true] {
            let captured = Footprint::decode(
                include_bytes!("../../tests/fixtures/board_only_shared_reference.ipc.bin")
                    .as_slice(),
            )
            .unwrap();
            let name = name(&captured).to_string();
            let initial = captured.clone();
            let state = Arc::new(Mutex::new((captured, None::<Footprint>, 0usize)));
            let observed = state.clone();
            let server = crate::test_support::MockIpcServer::spawn("field-commit", move |req| {
                let msg = req.message.unwrap();
                let mut s = observed.lock().unwrap();
                let response = if msg.type_url.ends_with("GetOpenDocuments") {
                    builders::pack_any(
                        &cmd::GetOpenDocumentsResponse {
                            documents: vec![ct::DocumentSpecifier {
                                r#type: ct::DocumentType::DoctypePcb as i32,
                                identifier: Some(
                                    ct::document_specifier::Identifier::BoardFilename(
                                        "/field-test.kicad_pcb".into(),
                                    ),
                                ),
                                project: None,
                            }],
                        },
                        "kiapi.common.commands.GetOpenDocumentsResponse",
                    )
                } else if msg.type_url.ends_with("GetItems") {
                    builders::pack_any(
                        &cmd::GetItemsResponse {
                            header: None,
                            status: ct::ItemRequestStatus::IrsOk as i32,
                            items: vec![builders::pack_any(&s.0, FP)],
                        },
                        "kiapi.common.commands.GetItemsResponse",
                    )
                } else if msg.type_url.ends_with("BeginCommit") {
                    s.2 += 1;
                    builders::pack_any(
                        &cmd::BeginCommitResponse {
                            id: Some(ct::Kiid {
                                value: "field-commit".into(),
                            }),
                        },
                        "kiapi.common.commands.BeginCommitResponse",
                    )
                } else if msg.type_url.ends_with("UpdateItems") {
                    let update = cmd::UpdateItems::decode(msg.value.as_slice()).unwrap();
                    assert_eq!(update.items.len(), 1);
                    s.1 = Some(Footprint::decode(update.items[0].value.as_slice()).unwrap());
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
                } else if msg.type_url.ends_with("EndCommit") {
                    s.0 = s.1.take().unwrap();
                    if corrupt_readback {
                        s.0.definition.as_mut().unwrap().items.clear();
                    }
                    builders::pack_any(
                        &cmd::EndCommitResponse {},
                        "kiapi.common.commands.EndCommitResponse",
                    )
                } else {
                    panic!("unexpected request {}", msg.type_url)
                };
                kiapi::common::ApiResponse {
                    status: Some(kiapi::common::ApiResponseStatus {
                        status: kiapi::common::ApiStatusCode::AsOk as i32,
                        error_message: String::new(),
                    }),
                    header: None,
                    message: Some(response),
                }
            });
            let ctx = ToolContext::new(
                super::super::ServerConfig {
                    kicad_cli: String::new(),
                    kicad_binary: String::new(),
                    ipc_address: server.address().into(),
                    project_dir: None,
                    jlcpcb_db_path: None,
                    auto_load_toolsets: false,
                    eager_toolsets: false,
                },
                Arc::new(crate::router::ToolRouter::new()),
            );
            let mut request = request();
            request["reference"] = json!(name);
            let planned = body(
                handle_edit_footprint_fabrication(&request, &ctx)
                    .await
                    .unwrap(),
            );
            assert_eq!(planned["status"], "ready");
            assert_eq!(state.lock().unwrap().0, initial);
            assert_eq!(state.lock().unwrap().2, 0);
            request["dry_run"] = json!(false);
            request["expected_plan_revision"] = json!("stale");
            let stale = body(
                handle_edit_footprint_fabrication(&request, &ctx)
                    .await
                    .unwrap(),
            );
            assert_eq!(stale["status"], "conflict");
            assert_eq!(state.lock().unwrap().0, initial);
            assert_eq!(state.lock().unwrap().2, 0);
            request["expected_plan_revision"] = planned["plan_revision"].clone();
            let applied = body(
                handle_edit_footprint_fabrication(&request, &ctx)
                    .await
                    .unwrap(),
            );
            assert_eq!(state.lock().unwrap().2, 1);
            if corrupt_readback {
                assert_eq!(applied["status"], "uncertain");
                assert!(applied["applied"].is_null());
                assert_ne!(state.lock().unwrap().0, initial);
            } else {
                assert_eq!(applied["status"], "complete");
                assert_eq!(applied["applied"], true);
                let actual = state.lock().unwrap();
                assert_eq!(actual.0.attributes.as_ref().unwrap().mounting_style, 2);
                assert_eq!(actual.0.value_field, initial.value_field);
            }
        }
    }
}
