//! Live footprint-only presentation edits; no library refresh or file fallback.
use super::{
    pcb_board::{attempt_ipc_write, BoardWrite},
    ToolContext, ToolDef,
};
use crate::{mcp::protocol::CallToolResult, tool};
use anyhow::{bail, Context, Result};
use konnect_ipc::{
    builders,
    gen::kiapi,
    transform::{transform_footprint_children, Xform},
};
use prost::Message;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
type Footprint = kiapi::board::types::FootprintInstance;
type Model = kiapi::board::types::Footprint3DModel;
type Graphic = kiapi::board::types::BoardGraphicShape;
const FP: &str = "kiapi.board.types.FootprintInstance";
const MODEL: &str = "kiapi.board.types.Footprint3DModel";
const GRAPHIC: &str = "kiapi.board.types.BoardGraphicShape";
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct ModelSpec {
    path: String,
    offset: [f64; 3],
    scale: [f64; 3],
    rotate: [f64; 3],
    visible: bool,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct GraphicEdit {
    uuid: String,
    layer: Option<String>,
    dx: Option<f64>,
    dy: Option<f64>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Args {
    board: String,
    reference: String,
    models: Option<Vec<ModelSpec>>,
    graphics: Option<Vec<GraphicEdit>>,
    #[serde(default = "yes")]
    dry_run: bool,
    expected_plan_revision: Option<String>,
}
fn yes() -> bool {
    true
}
fn documentation(layer: &str) -> bool {
    [
        "F.SilkS",
        "B.SilkS",
        "F.Fab",
        "B.Fab",
        "Dwgs.User",
        "Cmts.User",
    ]
    .contains(&layer)
}
fn validate(a: &Args) -> Result<()> {
    if a.board.trim().is_empty() || a.reference.trim().is_empty() {
        bail!("board and reference must be nonempty")
    }
    if !a.dry_run && a.expected_plan_revision.is_none() {
        bail!("expected_plan_revision required for apply")
    }
    if let Some(models) = &a.models {
        for m in models {
            if m.path.trim().is_empty() || m.path.contains(['\n', '\r', '\0']) {
                bail!("models.path must be nonempty and single-line")
            }
            for v in m.offset.iter().chain(&m.rotate) {
                if !v.is_finite() || v.abs() > 10000.0 {
                    bail!("models offset or rotation outside range")
                }
            }
            if m.scale
                .iter()
                .any(|v| !v.is_finite() || *v <= 0.0 || *v > 1000.0)
            {
                bail!("models.scale must be positive and <=1000")
            }
        }
    }
    let mut ids = BTreeSet::new();
    if let Some(graphics) = &a.graphics {
        for g in graphics {
            if g.uuid.is_empty() || !ids.insert(&g.uuid) {
                bail!("graphics.uuid must be nonempty and unique")
            }
            if g.layer.is_none() && g.dx.is_none() {
                bail!("empty graphic edit")
            }
            if g.dx.is_some() != g.dy.is_some() {
                bail!("graphics.dx and dy must occur together")
            }
            if g.layer.as_ref().is_some_and(|l| !documentation(l)) {
                bail!("graphics.layer must be a documentation layer")
            }
            if [g.dx, g.dy]
                .iter()
                .flatten()
                .any(|v| !v.is_finite() || v.abs() > 1000.0)
            {
                bail!("graphics delta outside +/-1000 mm")
            }
        }
    }
    Ok(())
}
fn vec3(v: [f64; 3]) -> kiapi::common::types::Vector3D {
    kiapi::common::types::Vector3D {
        x_nm: v[0],
        y_nm: v[1],
        z_nm: v[2],
    }
}
fn arr(v: &Option<kiapi::common::types::Vector3D>) -> Option<[f64; 3]> {
    v.as_ref().map(|v| [v.x_nm, v.y_nm, v.z_nm])
}
fn prepare(fp: &Footprint, a: &Args) -> Result<Footprint> {
    let mut out = fp.clone();
    let items = &mut out.definition.as_mut().context("missing definition")?.items;
    if let Some(models) = &a.models {
        items.retain(|i| !builders::any_is(i, MODEL));
        for m in models {
            items.push(builders::pack_any(
                &Model {
                    filename: m.path.clone(),
                    offset: Some(vec3(m.offset)),
                    scale: Some(vec3(m.scale)),
                    rotation: Some(vec3(m.rotate)),
                    visible: m.visible,
                    opacity: 1.0,
                },
                MODEL,
            ));
        }
    }
    if let Some(edits) = &a.graphics {
        for edit in edits {
            let mut matched = Vec::new();
            for (i, item) in items.iter().enumerate() {
                if builders::any_is(item, GRAPHIC) {
                    let g = Graphic::decode(item.value.as_slice())?;
                    if g.encode_to_vec() != item.value {
                        bail!("graphic contains unsupported wire data; refusing lossy edit")
                    }
                    if g.id.as_ref().is_some_and(|id| id.value == edit.uuid) {
                        matched.push((i, g));
                    }
                }
            }
            if matched.len() != 1 {
                bail!("graphic '{}' missing or ambiguous", edit.uuid)
            }
            let (idx, mut g) = matched.remove(0);
            let old = kiapi::board::types::BoardLayer::try_from(g.layer)
                .ok()
                .and_then(builders::layer_name)
                .context("unknown graphic layer")?;
            if !documentation(old) {
                bail!("only documentation graphics may be edited")
            }
            if let Some(l) = &edit.layer {
                g.layer = builders::layer_from_name(l) as i32;
            }
            let mut isolated = Footprint {
                definition: Some(kiapi::board::types::Footprint {
                    items: vec![builders::pack_any(&g, GRAPHIC)],
                    ..Default::default()
                }),
                ..Default::default()
            };
            if let (Some(dx), Some(dy)) = (edit.dx, edit.dy) {
                transform_footprint_children(
                    &mut isolated,
                    &Xform::Translate {
                        dx_nm: builders::mm_to_nm(dx),
                        dy_nm: builders::mm_to_nm(dy),
                    },
                )?;
            }
            items[idx] = isolated.definition.unwrap().items.remove(0);
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
        let fp = Footprint::decode(i.value.as_slice())?;
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
    h.update(serde_json::to_vec(&(&a.models, &a.graphics))?);
    Ok(format!("{:x}", h.finalize()))
}
fn view(fp: &Footprint) -> Result<Value> {
    let mut models = vec![];
    let mut graphics = vec![];
    for i in &fp.definition.as_ref().context("missing definition")?.items {
        if builders::any_is(i, MODEL) {
            let m = Model::decode(i.value.as_slice())?;
            models.push(json!({"path":m.filename,"offset":arr(&m.offset),"scale":arr(&m.scale),"rotate":arr(&m.rotation),"visible":m.visible,"opacity":m.opacity}));
        }
        if builders::any_is(i, GRAPHIC) {
            let g = Graphic::decode(i.value.as_slice())?;
            graphics.push(json!({"uuid":g.id.map(|id|id.value),"layer":kiapi::board::types::BoardLayer::try_from(g.layer).ok().and_then(builders::layer_name),"geometry_debug":format!("{:?}",g.shape)}));
        }
    }
    Ok(json!({"models":models,"graphics":graphics}))
}
pub(crate) fn tool() -> ToolDef {
    let v = json!({"type":"array","items":{"type":"number"},"minItems":3,"maxItems":3});
    tool!("edit_footprint_presentation","Inspect, plan or apply an exact live footprint's 3D model list and documentation graphic translations/layers. Omit models and graphics to inspect. Models replace the list, including explicit [] to clear; transforms use footprint-local mm and degrees as stored by KiCad, scale dimensionless. Graphic dx/dy use absolute board mm. Defaults to dry run; apply requires plan revision. Preserves pads, fields and routing; one undo commit with complete footprint readback. No file fallback. Model files are not downloaded or validated for mechanical accuracy.",json!({"type":"object","properties":{"board":{"type":"string"},"reference":{"type":"string"},"models":{"type":"array","items":{"type":"object","properties":{"path":{"type":"string"},"offset":v,"scale":v,"rotate":v,"visible":{"type":"boolean"}},"required":["path","offset","scale","rotate","visible"],"additionalProperties":false}},"graphics":{"type":"array","items":{"type":"object","properties":{"uuid":{"type":"string"},"layer":{"type":"string"},"dx":{"type":"number"},"dy":{"type":"number"}},"required":["uuid"],"additionalProperties":false}},"dry_run":{"type":"boolean","default":true},"expected_plan_revision":{"type":"string"}},"required":["board","reference"],"additionalProperties":false}),|args,ctx|async move{handle_edit_footprint_presentation(args,ctx).await}).with_board_access(super::BoardAccess::LiveOnly)
}
async fn handle_edit_footprint_presentation(
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
    let a: Args = match serde_json::from_value(args.clone()) {
        Ok(a) => a,
        Err(e) => return Ok(super::invalid_arg("arguments", &e.to_string())),
    };
    if let Err(e) = validate(&a) {
        return Ok(super::invalid_arg("arguments", &e.to_string()));
    }
    let board = super::get_path(args, "board")?;
    let path = board.clone();
    let result=attempt_ipc_write(ctx,&board,"footprint presentation",move |client|{
  let doc=client.find_open_board(&path)?;let kind=kiapi::common::types::KiCadObjectType::KotPcbFootprint;
  let fp=select(client.get_items_in(doc.clone(),kind)?,&a.reference)?;
  if a.models.is_none()&&a.graphics.is_none(){return Ok(CallToolResult::json(&json!({"status":"inspected","source":"ipc","board":path,"reference":a.reference,"presentation":view(&fp)?})));}
  let updated=match prepare(&fp,&a){Ok(u)=>u,Err(e)=>return Ok(CallToolResult::json(&json!({"status":"conflict","applied":false,"source":"ipc","diagnostic":e.to_string()})))};
  let rev=revision(&path.to_string_lossy(),&fp,&a)?;
  if !a.dry_run&&a.expected_plan_revision.as_deref()!=Some(&rev){return Ok(CallToolResult::json(&json!({"status":"conflict","applied":false,"source":"ipc","diagnostic":"stale_plan_revision"})));}
  let changed=comparable(&fp)!=comparable(&updated);
  if a.dry_run||!changed{return Ok(CallToolResult::json(&json!({"status":if changed{"ready"}else{"noop"},"applied":false,"source":"ipc","plan_revision":rev,"before":view(&fp)?,"after":view(&updated)?})));}
  let outcome=client.run_commit("Edit footprint presentation",|c|c.update_items_in(doc.clone(),vec![builders::pack_any(&updated,FP)])).and_then(|()|{let actual=select(client.get_items_in(doc.clone(),kind)?,&a.reference)?;if comparable(&actual)!=comparable(&updated){bail!("complete footprint readback differs; inspect before retrying")};Ok(actual)});
  match outcome {
   Ok(actual)=>Ok(CallToolResult::json(&json!({"status":"complete","applied":true,"source":"ipc","saved":false,"plan_revision":rev,"presentation":view(&actual)?}))),
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
        Footprint::decode(
            include_bytes!("../../tests/fixtures/board_only_shared_reference.ipc.bin").as_slice(),
        )
        .unwrap()
    }
    fn request() -> Value {
        json!({"board":"/field-test.kicad_pcb","reference":name(&fixture()),"models":[{"path":"test.step","offset":[1,2,3],"scale":[1,1,1],"rotate":[10,20,30],"visible":true}]})
    }
    #[test]
    fn preserves_every_nonmodel_byte() {
        let fp = fixture();
        let a: Args = serde_json::from_value(request()).unwrap();
        validate(&a).unwrap();
        let out = prepare(&fp, &a).unwrap();
        let mut a = out.clone();
        a.definition
            .as_mut()
            .unwrap()
            .items
            .retain(|i| !builders::any_is(i, MODEL));
        let mut b = fp.clone();
        b.definition
            .as_mut()
            .unwrap()
            .items
            .retain(|i| !builders::any_is(i, MODEL));
        assert_eq!(a, b);
        assert_eq!(
            view(&out).unwrap()["models"][0]["offset"],
            json!([1.0, 2.0, 3.0])
        );
    }
    #[test]
    fn rejects_duplicate_graphics_copper_layers_and_bad_models() {
        for patch in [
            json!({"models":[{"path":"x","offset":[0,0,0],"scale":[0,1,1],"rotate":[0,0,0],"visible":true}]}),
            json!({"graphics":[{"uuid":"a","layer":"F.Cu"}]}),
            json!({"graphics":[{"uuid":"a","dx":1}]}),
            json!({"graphics":[{"uuid":"a","layer":"F.Fab"},{"uuid":"a","layer":"F.Fab"}]}),
        ] {
            let mut v = request();
            for (k, val) in patch.as_object().unwrap() {
                v[k] = val.clone();
            }
            assert!(validate(&serde_json::from_value(v).unwrap()).is_err());
        }
    }
    #[test]
    fn missing_graphic_refuses_before_change() {
        let fp = fixture();
        let mut v = request();
        v["graphics"] = json!([{"uuid":"missing","layer":"F.Fab"}]);
        assert!(prepare(&fp, &serde_json::from_value(v).unwrap()).is_err());
    }
    #[test]
    fn readback_detects_pad_and_field_changes() {
        let fp = fixture();
        let mut f = fp.clone();
        f.definition.as_mut().unwrap().items.clear();
        assert_ne!(comparable(&fp), comparable(&f));
        f = fp.clone();
        f.value_field = None;
        assert_ne!(comparable(&fp), comparable(&f));
    }
    #[test]
    fn revision_binds_path_request_and_geometry() {
        let fp = fixture();
        let a: Args = serde_json::from_value(request()).unwrap();
        let r = revision("a", &fp, &a).unwrap();
        assert_ne!(r, revision("b", &fp, &a).unwrap());
        let mut moved = fp.clone();
        moved.position = None;
        assert_ne!(r, revision("a", &moved, &a).unwrap());
        let mut b = a.clone();
        b.models = Some(vec![]);
        assert_ne!(r, revision("a", &fp, &b).unwrap());
    }

    #[test]
    fn translates_only_selected_documentation_graphic() {
        let mut fp = fixture();
        let mut g = builders::board_segment("B.SilkS", 0.12, 10.0, 20.0, 11.0, 20.0);
        g.id = Some(kiapi::common::types::Kiid {
            value: "marker".into(),
        });
        g.parent = fp.id.clone();
        fp.definition
            .as_mut()
            .unwrap()
            .items
            .push(builders::pack_any(&g, GRAPHIC));
        let mut v = request();
        v.as_object_mut().unwrap().remove("models");
        v["graphics"] = json!([{"uuid":"marker","dx":0.5,"dy":-1.0,"layer":"B.Fab"}]);
        let out = prepare(&fp, &serde_json::from_value(v.clone()).unwrap()).unwrap();
        let mut unchanged = out.clone();
        unchanged.definition.as_mut().unwrap().items.pop();
        let mut original = fp.clone();
        original.definition.as_mut().unwrap().items.pop();
        assert_eq!(unchanged, original);
        let g = Graphic::decode(
            out.definition
                .unwrap()
                .items
                .last()
                .unwrap()
                .value
                .as_slice(),
        )
        .unwrap();
        assert_eq!(g.layer, builders::layer_from_name("B.Fab") as i32);
        assert_eq!(g.parent, fp.id);
        match g.shape.unwrap().geometry.unwrap() {
            kiapi::common::types::graphic_shape::Geometry::Segment(s) => {
                assert_eq!(s.start.unwrap(), builders::vec2(10.5, 19.0));
                assert_eq!(s.end.unwrap(), builders::vec2(11.5, 19.0));
            }
            _ => panic!("expected segment"),
        }
        let items = &mut fp.definition.as_mut().unwrap().items;
        let last = items.last_mut().unwrap();
        let mut g = Graphic::decode(last.value.as_slice()).unwrap();
        g.layer = builders::layer_from_name("Edge.Cuts") as i32;
        *last = builders::pack_any(&g, GRAPHIC);
        assert!(prepare(&fp, &serde_json::from_value(v).unwrap()).is_err());
    }

    #[test]
    fn unknown_graphic_wire_data_refuses_lossy_edit() {
        let mut fp = fixture();
        let mut g = builders::board_segment("F.SilkS", 0.1, 1.0, 2.0, 3.0, 2.0);
        g.id = Some(kiapi::common::types::Kiid {
            value: "future".into(),
        });
        let mut item = builders::pack_any(&g, GRAPHIC);
        item.value.extend([0x98, 0x06, 0x01]);
        fp.definition.as_mut().unwrap().items.push(item);
        let a:Args=serde_json::from_value(json!({"board":"a.kicad_pcb","reference":name(&fp),"graphics":[{"uuid":"future","layer":"F.Fab"}]})).unwrap();
        assert!(prepare(&fp, &a)
            .unwrap_err()
            .to_string()
            .contains("unsupported wire data"));
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
                handle_edit_footprint_presentation(&request, &ctx)
                    .await
                    .unwrap(),
            );
            assert_eq!(planned["status"], "ready");
            assert_eq!(state.lock().unwrap().0, initial);
            assert_eq!(state.lock().unwrap().2, 0);
            request["dry_run"] = json!(false);
            request["expected_plan_revision"] = json!("stale");
            let stale = body(
                handle_edit_footprint_presentation(&request, &ctx)
                    .await
                    .unwrap(),
            );
            assert_eq!(stale["status"], "conflict");
            assert_eq!(state.lock().unwrap().0, initial);
            assert_eq!(state.lock().unwrap().2, 0);
            request["expected_plan_revision"] = planned["plan_revision"].clone();
            let applied = body(
                handle_edit_footprint_presentation(&request, &ctx)
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
                assert_eq!(view(&actual.0).unwrap()["models"][0]["path"], "test.step");
                assert_eq!(actual.0.value_field, initial.value_field);
            }
        }
    }
}
