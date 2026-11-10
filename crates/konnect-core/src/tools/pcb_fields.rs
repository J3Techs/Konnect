//! Footprint field edits, live and revision-bound, plus the closed-board pad
//! fabrication-property setter (`pcb_components::pad_fabrication`).
use super::{
    footprint_fabrication::{check_live_rebuild, RebuildTarget, Remedy},
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
use std::collections::{BTreeMap, BTreeSet};
type Footprint = kiapi::board::types::FootprintInstance;
type Field = kiapi::board::types::Field;
const FIELD_TYPE: &str = "kiapi.board.types.Field";
const FP_TYPE: &str = "kiapi.board.types.FootprintInstance";
mod values;
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Change {
    name: String,
    value: Option<String>,
    visible: Option<bool>,
    x: Option<f64>,
    y: Option<f64>,
    layer: Option<String>,
    size: Option<f64>,
    stroke: Option<f64>,
    font_name: Option<String>,
    centered: Option<bool>,
    rotation: Option<f64>,
    #[serde(default)]
    create: bool,
}
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ReadArgs {
    board: String,
    reference: String,
}
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Args {
    board: String,
    reference: String,
    fields: Vec<Change>,
    #[serde(default = "yes")]
    dry_run: bool,
    expected_plan_revision: Option<String>,
}
fn yes() -> bool {
    true
}
fn fields(fp: &Footprint) -> Result<BTreeMap<String, Field>> {
    let mut out = BTreeMap::new();
    for (name, field) in [
        ("Reference", &fp.reference_field),
        ("Value", &fp.value_field),
        ("Datasheet", &fp.datasheet_field),
        ("Description", &fp.description_field),
    ] {
        if let Some(f) = field {
            let mut f = f.clone();
            f.name = name.into();
            out.insert(name.into(), f);
        }
    }
    for item in &fp
        .definition
        .as_ref()
        .context("footprint has no definition")?
        .items
    {
        if builders::any_is(item, FIELD_TYPE) {
            let f = Field::decode(item.value.as_slice())?;
            if out.insert(f.name.clone(), f).is_some() {
                bail!("duplicate footprint field name");
            }
        }
    }
    Ok(out)
}
fn reference(fp: &Footprint) -> &str {
    fp.reference_field
        .as_ref()
        .and_then(|f| f.text.as_ref())
        .and_then(|t| t.text.as_ref())
        .map(|t| t.text.as_str())
        .unwrap_or("")
}
fn replace(fp: &mut Footprint, f: Field) -> Result<()> {
    match f.name.as_str() {
        "Reference" => fp.reference_field = Some(f),
        "Value" => fp.value_field = Some(f),
        "Datasheet" => fp.datasheet_field = Some(f),
        "Description" => fp.description_field = Some(f),
        _ => {
            let items = &mut fp.definition.as_mut().context("missing definition")?.items;
            for item in items.iter_mut() {
                if builders::any_is(item, FIELD_TYPE)
                    && Field::decode(item.value.as_slice())?.name == f.name
                {
                    *item = builders::pack_any(&f, FIELD_TYPE);
                    return Ok(());
                }
            }
            items.push(builders::pack_any(&f, FIELD_TYPE));
        }
    }
    Ok(())
}
fn validate(a: &Args) -> Result<()> {
    if a.board.is_empty() || a.reference.is_empty() || a.fields.is_empty() {
        bail!("board, reference and fields must be nonempty");
    }
    if !a.dry_run && a.expected_plan_revision.is_none() {
        bail!("expected_plan_revision required for apply");
    }
    let mut names = BTreeSet::new();
    for c in &a.fields {
        if c.font_name
            .as_ref()
            .is_some_and(|f| f.contains(['\0', '\n', '\r']))
        {
            bail!("font_name must be single-line");
        }
        if c.name.is_empty() || !names.insert(&c.name) {
            bail!("fields.name must be nonempty and unique");
        }
        if ["Reference", "Value", "Datasheet", "Description"]
            .iter()
            .any(|name| c.name.eq_ignore_ascii_case(name) && c.name != *name)
        {
            bail!("reserved field names require their exact capitalization");
        }
        if c.name == "Reference" && (c.value.is_some() || c.create) {
            bail!("Reference content cannot be changed; use schematic synchronization");
        }
        if c.x.is_some() != c.y.is_some() {
            bail!("fields.x and fields.y must occur together");
        }
        for v in [c.x, c.y].into_iter().flatten() {
            if !v.is_finite() || v.abs() > 10000.0 {
                bail!("field coordinate outside +/-10000 mm");
            }
        }
        for (name, v, min, max) in [
            ("size", c.size, 0.1, 100.0),
            ("stroke", c.stroke, 0.01, 10.0),
            ("rotation", c.rotation, -360.0, 360.0),
        ] {
            if v.is_some_and(|v| !v.is_finite() || v < min || v > max) {
                bail!("fields.{name} outside supported range");
            }
        }
        if let Some(l) = &c.layer {
            if ![
                "F.SilkS",
                "B.SilkS",
                "F.Fab",
                "B.Fab",
                "Dwgs.User",
                "Cmts.User",
            ]
            .contains(&l.as_str())
            {
                bail!("fields.layer must be a non-copper documentation layer");
            }
        }
        if c.value.is_none()
            && c.visible.is_none()
            && c.x.is_none()
            && c.layer.is_none()
            && c.size.is_none()
            && c.stroke.is_none()
            && c.rotation.is_none()
            && c.font_name.is_none()
            && c.centered != Some(true)
        {
            bail!("field change is empty");
        }
    }
    Ok(())
}
fn prepare(fp: &Footprint, changes: &[Change]) -> Result<Footprint> {
    let mut result = fp.clone();
    let existing = fields(fp)?;
    let mut next_id = existing
        .values()
        .filter_map(|f| f.id.as_ref().map(|id| id.id))
        .max()
        .unwrap_or(3)
        .max(3)
        + 1;
    for c in changes {
        let mut f = if let Some(f) = existing.get(&c.name) {
            if c.create {
                bail!("field '{}' already exists", c.name);
            }
            f.clone()
        } else {
            if !c.create || c.value.is_none() {
                bail!(
                    "missing field '{}'; creation requires create=true and value",
                    c.name
                );
            }
            let mut f = fp
                .value_field
                .clone()
                .context("cannot derive new field style")?;
            f.name = c.name.clone();
            f.id = Some(kiapi::board::types::FieldId { id: next_id });
            next_id += 1;
            f.text.as_mut().context("missing field BoardText")?.id = None;
            f.visible = false;
            f
        };
        let bt = f.text.as_mut().context("field has no BoardText")?;
        let text = bt.text.as_mut().context("field has no Text")?;
        if let Some(v) = &c.value {
            text.text = v.clone();
        }
        if let Some(v) = c.visible {
            f.visible = v;
        }
        if let (Some(x), Some(y)) = (c.x, c.y) {
            text.position = Some(kiapi::common::types::Vector2 {
                x_nm: builders::mm_to_nm(x),
                y_nm: builders::mm_to_nm(y),
            });
        }
        if let Some(l) = &c.layer {
            bt.layer = builders::layer_from_name(l) as i32;
        }
        let attr = text
            .attributes
            .as_mut()
            .context("field has no text attributes")?;
        if c.centered == Some(true) {
            attr.horizontal_alignment = 2;
            attr.vertical_alignment = 2;
        }
        if let Some(font) = &c.font_name {
            attr.font_name = font.clone();
        }
        if let Some(s) = c.size {
            attr.size = Some(kiapi::common::types::Vector2 {
                x_nm: builders::mm_to_nm(s),
                y_nm: builders::mm_to_nm(s),
            });
        }
        if let Some(s) = c.stroke {
            attr.stroke_width = Some(kiapi::common::types::Distance {
                value_nm: builders::mm_to_nm(s),
            });
        }
        if let Some(r) = c.rotation {
            attr.angle = Some(kiapi::common::types::Angle { value_degrees: r });
        }
        replace(&mut result, f)?;
    }
    Ok(result)
}
fn view(fp: &Footprint) -> Result<Value> {
    let mut rows = Vec::new();
    for (name, f) in fields(fp)? {
        let bt = f.text.as_ref().context("field missing BoardText")?;
        let t = bt.text.as_ref().context("field missing Text")?;
        let p = t.position.as_ref().context("field missing position")?;
        let a = t.attributes.as_ref().context("field missing attributes")?;
        rows.push(json!({"name":name,"value":t.text,"visible":f.visible,"x":builders::nm_to_mm(p.x_nm),"y":builders::nm_to_mm(p.y_nm),"layer":kiapi::board::types::BoardLayer::try_from(bt.layer).ok().and_then(builders::layer_name),"size":a.size.as_ref().map(|s|[builders::nm_to_mm(s.x_nm),builders::nm_to_mm(s.y_nm)]),"font_name":a.font_name,"rotation":a.angle.as_ref().map(|a|a.value_degrees),"stroke":a.stroke_width.as_ref().map(|s|builders::nm_to_mm(s.value_nm))}));
    }
    Ok(json!(rows))
}
fn revision(board: &str, fp: &Footprint, changes: &[Change]) -> Result<String> {
    let mut h = Sha256::new();
    h.update(board.as_bytes());
    h.update(fp.encode_to_vec());
    h.update(serde_json::to_vec(changes)?);
    Ok(format!("{:x}", h.finalize()))
}
pub(crate) fn comparable(fp: &Footprint) -> Result<Footprint> {
    let mut out = fp.clone();
    // KiCad serializes custom fields ahead of pads, independent of insertion order.
    out.definition
        .as_mut()
        .context("missing definition")?
        .items
        .retain(|item| !builders::any_is(item, FIELD_TYPE));
    // KiCad assigns identity to newly created fields. Ignore only field identity;
    // every field property and every unrelated footprint item still must match.
    for (_, mut f) in fields(fp)? {
        f.id = None;
        if let Some(t) = &mut f.text {
            t.id = None;
        }
        replace(&mut out, f)?;
    }
    Ok(out)
}
fn select(items: Vec<prost_types::Any>, name: &str) -> Result<Footprint> {
    let mut found = None;
    for item in items {
        let fp = Footprint::decode(item.value.as_slice())?;
        if reference(&fp) == name {
            if found.is_some() {
                bail!("ambiguous footprint reference '{name}'");
            }
            found = Some(fp);
        }
    }
    found.with_context(|| format!("footprint '{name}' not found"))
}
pub(crate) fn tools() -> [ToolDef; 4] {
    let schema = json!({"type":"object","properties":{"board":{"type":"string"},"reference":{"type":"string"},"fields":{"type":"array","minItems":1,"items":{"type":"object","properties":{"name":{"type":"string"},"value":{"type":"string"},"visible":{"type":"boolean"},"x":{"type":"number"},"y":{"type":"number"},"layer":{"type":"string"},"size":{"type":"number"},"stroke":{"type":"number"},"centered":{"type":"boolean","description":"True centers both axes on the field anchor; false preserves existing alignment"},"font_name":{"type":"string","description":"Font family; empty string uses KiCad stroke font"},"rotation":{"type":"number"},"create":{"type":"boolean","default":false}},"required":["name"],"additionalProperties":false}},"dry_run":{"type":"boolean","default":true},"expected_plan_revision":{"type":"string"}},"required":["board","reference","fields"],"additionalProperties":false});
    [tool!("list_footprint_fields","Read mandatory and custom fields from an exact footprint on the requested live board.",json!({"type":"object","properties":{"board":{"type":"string"},"reference":{"type":"string"}},"required":["board","reference"],"additionalProperties":false}),|args,ctx|async move {handle_list_footprint_fields(args,ctx).await}).with_board_access(super::BoardAccess::LiveOnly),
    tool!("edit_footprint_fields","Plan or apply footprint text visibility, style, position and custom properties through live IPC. Defaults to dry run. Apply requires exact revision and verifies the complete footprint after publishing one undo entry. Never renumbers references. No file fallback. A footprint whose pads carry a fabrication property (pad_prop_heatsink, pad_prop_bga, …) is refused, plan and apply, because KiCad 10's typed update re-creates pads without it; set_footprint_field_values (values) and edit_component reference_field (Reference presentation) edit such a footprint on a closed board.",schema,|args,ctx|async move {handle_edit_footprint_fields(args,ctx).await}).with_board_access(super::BoardAccess::LiveOnly),
    values::tool(),
    super::pcb_components::pad_fabrication::tool()]
}
async fn handle_list_footprint_fields(args: &Value, ctx: &ToolContext) -> Result<CallToolResult> {
    handle(args, ctx, false).await
}
async fn handle_edit_footprint_fields(args: &Value, ctx: &ToolContext) -> Result<CallToolResult> {
    handle(args, ctx, true).await
}
async fn handle(args: &Value, ctx: &ToolContext, editing: bool) -> Result<CallToolResult> {
    let board = super::get_path(args, "board")?;
    let name = match super::require_str(args, "reference") {
        Ok(v) => v.to_owned(),
        Err(e) => return Ok(e),
    };
    let parsed = if editing {
        // Name the malformed option instead of treating a present null as omitted.
        for key in [
            "board",
            "reference",
            "fields",
            "dry_run",
            "expected_plan_revision",
        ] {
            if args.get(key).is_some_and(Value::is_null) {
                return Ok(super::invalid_arg(
                    key,
                    "null is not supported; omit unchanged options",
                ));
            }
        }
        if args["fields"].as_array().is_some_and(|a| {
            a.iter().any(|v| {
                v.as_object()
                    .is_some_and(|m| m.values().any(Value::is_null))
            })
        }) {
            return Ok(super::invalid_arg(
                "fields",
                "null is not supported; omit unchanged options",
            ));
        }
        let a: Args = match serde_json::from_value(args.clone()) {
            Ok(a) => a,
            Err(e) => return Ok(super::invalid_arg("fields", &e.to_string())),
        };
        if let Err(e) = validate(&a) {
            return Ok(super::invalid_arg("fields", &e.to_string()));
        }
        Some(a)
    } else {
        let read: ReadArgs = match serde_json::from_value(args.clone()) {
            Ok(v) => v,
            Err(e) => return Ok(super::invalid_arg("arguments", &e.to_string())),
        };
        if read.board.trim().is_empty() || read.reference.trim().is_empty() {
            return Ok(super::invalid_arg(
                "reference",
                "board and reference must be nonempty",
            ));
        }
        None
    };
    let path = board.clone();
    let result=attempt_ipc_write(ctx,&board,"footprint fields",move |client|{
        let doc=client.find_open_board(&path)?;let kind=kiapi::common::types::KiCadObjectType::KotPcbFootprint;
        let fp=select(client.get_items_in(doc.clone(),kind)?,&name)?;
        let Some(a)=parsed else {return Ok(CallToolResult::json(&json!({"source":"ipc","board":path,"reference":name,"fields":view(&fp)?})));};
        let updated=match prepare(&fp,&a.fields){Ok(v)=>v,Err(e)=>return Ok(CallToolResult::json(&json!({"status":"conflict","applied":false,"source":"ipc","diagnostic":e.to_string()})))};
        let changed=fp!=updated;
        // A changed footprint is sent back whole, and KiCad re-creates its
        // pads without any pad fabrication property. Refuse the plan too.
        if changed {
            if let Err(refusal)=check_live_rebuild(&path,"edit_footprint_fields",&[RebuildTarget::from_instance(&fp)],Remedy::NoLosslessPath){return Ok(refusal.into_result());}
        }
        let rev=revision(&path.to_string_lossy(),&fp,&a.fields)?;
        if !a.dry_run && a.expected_plan_revision.as_deref()!=Some(&rev){return Ok(CallToolResult::json(&json!({"status":"conflict","applied":false,"source":"ipc","diagnostic":"stale_plan_revision"})));}
        if a.dry_run || !changed{return Ok(CallToolResult::json(&json!({"status":if changed{"ready"}else{"noop"},"applied":false,"source":"ipc","plan_revision":rev,"before":view(&fp)?,"after":view(&updated)?})));}
        // KiCad stages footprint replacement as Remove/Add until PushCommit.
        // GetItems inside that commit still returns the old footprint.
        let outcome=client.run_commit("Edit footprint fields",|c|{
            c.update_items_in(doc.clone(),vec![builders::pack_any(&updated,FP_TYPE)])
        }).and_then(|()| {
            let actual=select(client.get_items_in(doc.clone(),kind)?,&name)?;
            if comparable(&actual)?!=comparable(&updated)? {bail!("readback differs; expected fields {}; observed fields {}", view(&updated)?, view(&actual)?);}
            Ok(actual)
        });
        match outcome {
            Ok(actual)=>Ok(CallToolResult::json(&json!({"status":"complete","applied":true,"source":"ipc","saved":false,"plan_revision":rev,"fields":view(&actual)?}))),
            Err(e)=>{
                let restored=client.get_items_in(doc,kind).and_then(|items|select(items,&name)).and_then(|actual|Ok(comparable(&actual)?==comparable(&fp)?));
                Ok(CallToolResult::json(&json!({"status":if matches!(restored,Ok(true)){"unchanged_after_failure"}else{"uncertain"},"applied":if matches!(restored,Ok(true)){json!(false)}else{Value::Null},"source":"ipc","diagnostic":format!("{e:#}"),"recovery":if matches!(restored,Ok(true)){"Readback matches pre-edit footprint"}else{"Inspect live document before retrying; rollback not verified"}})))
            }
        }
    }).await?;
    Ok(match result {
        BoardWrite::Ipc(r) | BoardWrite::Refused(r) => r,
        BoardWrite::File(reason) => CallToolResult::error(format!(
            "{} Footprint field tools require live IPC and never edit the board file.",
            reason.premise()
        )),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    fn fixture() -> Footprint {
        let field = Field {
            name: "Value".into(),
            visible: true,
            text: Some(kiapi::board::types::BoardText {
                text: Some(kiapi::common::types::Text {
                    text: "100nF".into(),
                    position: Some(kiapi::common::types::Vector2 {
                        x_nm: 1000000,
                        y_nm: 2000000,
                    }),
                    attributes: Some(Default::default()),
                    ..Default::default()
                }),
                layer: builders::layer_from_name("F.SilkS") as i32,
                ..Default::default()
            }),
            ..Default::default()
        };
        let mut reference = field.clone();
        reference.name = "Reference".into();
        reference.text.as_mut().unwrap().text.as_mut().unwrap().text = "C1".into();
        Footprint {
            reference_field: Some(reference),
            value_field: Some(field),
            definition: Some(kiapi::board::types::Footprint {
                items: vec![builders::pack_any(
                    &kiapi::board::types::Pad {
                        number: "1".into(),
                        net: Some(kiapi::board::types::Net {
                            name: "GND".into(),
                            ..Default::default()
                        }),
                        ..Default::default()
                    },
                    "kiapi.board.types.Pad",
                )],
                ..Default::default()
            }),
            ..Default::default()
        }
    }
    fn args(fields: Value) -> Args {
        serde_json::from_value(json!({"board":"test.kicad_pcb","reference":"C1","fields":fields}))
            .unwrap()
    }
    #[test]
    fn font_and_center_changes_preserve_other_field_and_footprint_data() {
        let fp = fixture();
        let changes: Vec<Change> =
            serde_json::from_value(json!([{"name":"Reference","font_name":"","centered":true}]))
                .unwrap();
        let updated = prepare(&fp, &changes).unwrap();
        let attrs = updated
            .reference_field
            .as_ref()
            .unwrap()
            .text
            .as_ref()
            .unwrap()
            .text
            .as_ref()
            .unwrap()
            .attributes
            .as_ref()
            .unwrap();
        assert_eq!(attrs.font_name, "");
        assert_eq!(attrs.horizontal_alignment, 2);
        assert_eq!(attrs.vertical_alignment, 2);
        let mut restored = updated;
        restored.reference_field = fp.reference_field.clone();
        assert_eq!(restored, fp);
    }

    #[test]
    fn changes_only_selected_field() {
        let fp = fixture();
        let a = args(json!([{"name":"Value","visible":false,"x":3,"y":4}]));
        validate(&a).unwrap();
        let updated = prepare(&fp, &a.fields).unwrap();
        assert_eq!(updated.definition, fp.definition);
        assert_eq!(updated.reference_field, fp.reference_field);
        assert!(!updated.value_field.as_ref().unwrap().visible);
        assert!(fp.value_field.as_ref().unwrap().visible);
        assert_eq!(
            updated
                .value_field
                .unwrap()
                .text
                .unwrap()
                .text
                .unwrap()
                .position
                .unwrap()
                .x_nm,
            3000000
        );
    }
    #[test]
    fn custom_creation_is_explicit_and_preserves_pad_bytes() {
        let fp = fixture();
        let bad = args(json!([{"name":"MPN","value":"part"}]));
        assert!(prepare(&fp, &bad.fields).is_err());
        let a = args(json!([{"name":"MPN","value":"part","create":true}]));
        let updated = prepare(&fp, &a.fields).unwrap();
        assert_eq!(
            updated.definition.as_ref().unwrap().items[0],
            fp.definition.as_ref().unwrap().items[0]
        );
        let fields = fields(&updated).unwrap();
        assert_eq!(
            fields["MPN"]
                .text
                .as_ref()
                .unwrap()
                .text
                .as_ref()
                .unwrap()
                .text,
            "part"
        );
        assert!(!fields["MPN"].visible);
        assert!(prepare(&updated, &a.fields).is_err());
    }
    #[test]
    fn rejects_reference_rename_duplicate_empty_and_half_coordinate() {
        for f in [
            json!([{"name":"Reference","value":"C2"}]),
            json!([{"name":"Value","visible":false},{"name":"Value","value":"x"}]),
            json!([{"name":"Value"}]),
            json!([{"name":"Value","x":2}]),
            json!([{"name":"Value","size":0.01}]),
            json!([{"name":"Value","layer":"F.Cu"}]),
        ] {
            assert!(validate(&args(f)).is_err());
        }
    }
    #[test]
    fn revision_binds_geometry_and_request() {
        let fp = fixture();
        let a = args(json!([{"name":"Value","visible":false}]));
        let rev = revision("board", &fp, &a.fields).unwrap();
        let mut moved = fp.clone();
        moved.position = Some(kiapi::common::types::Vector2 { x_nm: 1, y_nm: 0 });
        assert_ne!(rev, revision("board", &moved, &a.fields).unwrap());
        assert_ne!(rev, revision("other", &fp, &a.fields).unwrap());
        let b = args(json!([{"name":"Value","visible":true}]));
        assert_ne!(rev, revision("board", &fp, &b.fields).unwrap());
    }
    #[test]
    fn readback_check_detects_unrelated_pad_change() {
        let fp = fixture();
        let mut changed = fp.clone();
        changed.definition.as_mut().unwrap().items.clear();
        assert_ne!(comparable(&fp).unwrap(), comparable(&changed).unwrap());
    }
    #[test]
    fn schemas_are_registered_and_closed() {
        let tools = super::tools();
        assert_eq!(tools.len(), 4);
        let a = args(json!([{"name":"Value","visible":false}]));
        assert!(a.dry_run);
        assert!(!a.fields[0].create);
        assert!(serde_json::from_value::<Args>(
            json!({"board":"x","reference":"C1","fields":[{"name":"Value","visible":"false"}]})
        )
        .is_err());
        assert!(serde_json::from_value::<Args>(
            json!({"board":"x","reference":"C1","fields":[{"name":"Value","bogus":1}]})
        )
        .is_err());
    }
    #[test]
    fn normalization_allows_field_order_and_assigned_ids_but_detects_other_changes() {
        let fp = Footprint::decode(
            include_bytes!("../../tests/fixtures/board_only_shared_reference.ipc.bin").as_slice(),
        )
        .unwrap();
        let a = args(json!([{"name":"MPN","value":"example","create":true}]));
        let prepared = prepare(&fp, &a.fields).unwrap();
        let mut actual = prepared.clone();
        let items = &mut actual.definition.as_mut().unwrap().items;
        let last = items.pop().unwrap();
        items.insert(0, last);
        let mut added = fields(&actual).unwrap()["MPN"].clone();
        added.id = Some(kiapi::board::types::FieldId { id: 99 });
        replace(&mut actual, added).unwrap();
        assert_eq!(comparable(&prepared).unwrap(), comparable(&actual).unwrap());
        actual.position = Some(kiapi::common::types::Vector2 {
            x_nm: 123,
            y_nm: 456,
        });
        assert_ne!(comparable(&prepared).unwrap(), comparable(&actual).unwrap());
    }

    #[test]
    fn field_view_reports_requested_stroke_for_review() {
        let a = args(json!([{"name":"Value", "stroke":0.12}]));
        let updated = prepare(&fixture(), &a.fields).unwrap();
        let rows = view(&updated).unwrap();
        let value = rows
            .as_array()
            .unwrap()
            .iter()
            .find(|v| v["name"] == "Value")
            .unwrap();
        assert_eq!(value["stroke"], 0.12);
    }

    fn body(result: CallToolResult) -> Value {
        match &result.content[0] {
            crate::mcp::protocol::ToolContent::Text { text } => serde_json::from_str(text).unwrap(),
            other => panic!("unexpected result {other:?}"),
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
            let name = reference(&captured).to_string();
            let initial = captured.clone();
            let state = Arc::new(Mutex::new((captured, None::<Footprint>, 0usize)));
            let observed = state.clone();
            // An absolute path on every platform: KiCad names an open board
            // absolutely. The saved copy shows the footprint's pads carry no
            // fabrication property, which the live edit needs proven.
            let temp = tempfile::tempdir().unwrap();
            let board_file = temp.path().join("field-test.kicad_pcb");
            std::fs::write(
                &board_file,
                format!(
                    "(kicad_pcb (version 20260206)\n\t(footprint \"Saved\" (layer \"F.Cu\")\n\t\t(property \"Reference\" \"{name}\" (at 0 0 0) (layer \"F.SilkS\"))\n\t\t(pad \"1\" smd rect (at 0 0) (size 1 1) (layers \"F.Cu\"))\n\t)\n)\n"
                ),
            )
            .unwrap();
            let board_path = board_file.to_string_lossy().into_owned();
            let served_path = board_path.clone();
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
                                        served_path.clone(),
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
                            items: vec![builders::pack_any(&s.0, FP_TYPE)],
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
            let mut request = json!({"board":board_path,"reference":name,"fields":[{"name":"Value","visible":false,"layer":"F.Fab"}]});
            let planned = body(handle(&request, &ctx, true).await.unwrap());
            assert_eq!(planned["status"], "ready");
            assert_eq!(state.lock().unwrap().0, initial);
            assert_eq!(state.lock().unwrap().2, 0);
            request["dry_run"] = json!(false);
            request["expected_plan_revision"] = json!("stale");
            let stale = body(handle(&request, &ctx, true).await.unwrap());
            assert_eq!(stale["status"], "conflict");
            assert_eq!(state.lock().unwrap().0, initial);
            assert_eq!(state.lock().unwrap().2, 0);
            request["expected_plan_revision"] = planned["plan_revision"].clone();
            let applied = body(handle(&request, &ctx, true).await.unwrap());
            assert_eq!(state.lock().unwrap().2, 1);
            if corrupt_readback {
                assert_eq!(applied["status"], "uncertain");
                assert!(applied["applied"].is_null());
                assert_ne!(state.lock().unwrap().0, initial);
            } else {
                assert_eq!(applied["status"], "complete");
                assert_eq!(applied["applied"], true);
                let actual = state.lock().unwrap();
                assert!(!actual.0.value_field.as_ref().unwrap().visible);
                assert_eq!(actual.0.definition, initial.definition);
            }
        }
    }

    /// U2's pad 9 carries `pad_prop_heatsink` in the saved board; R1's pads
    /// carry nothing. [`guard_board`] writes it with each property the guard
    /// tests cover.
    const GUARD_BOARD: &str = "(kicad_pcb (version 20260206)\n\t(footprint \"Parts:Reg\" (layer \"F.Cu\") (uuid \"aaaaaaaa-0000-4000-8000-000000000001\") (at 10 10)\n\t\t(property \"Reference\" \"U2\" (at 0 -2 0) (layer \"F.SilkS\") (uuid \"aaaaaaaa-0000-4000-8000-000000000011\")\n\t\t\t(effects (font (size 1 1) (thickness 0.15))))\n\t\t(property \"Value\" \"100nF\" (at 0 2 0) (layer \"F.Fab\") (uuid \"aaaaaaaa-0000-4000-8000-000000000012\")\n\t\t\t(effects (font (size 1 1) (thickness 0.15))))\n\t\t(pad \"9\" smd rect (at 0 0) (size 2.95 4.9) (property pad_prop_heatsink) (layers \"F.Cu\" \"F.Mask\") (net 1 \"GND\") (uuid \"aaaaaaaa-0000-4000-8000-000000000013\"))\n\t)\n\t(footprint \"Parts:Plain\" (layer \"F.Cu\") (uuid \"aaaaaaaa-0000-4000-8000-000000000002\") (at 20 10)\n\t\t(property \"Reference\" \"R1\" (at 0 -2 0) (layer \"F.SilkS\") (uuid \"aaaaaaaa-0000-4000-8000-000000000021\")\n\t\t\t(effects (font (size 1 1) (thickness 0.15))))\n\t\t(property \"Value\" \"100nF\" (at 0 2 0) (layer \"F.Fab\") (uuid \"aaaaaaaa-0000-4000-8000-000000000022\")\n\t\t\t(effects (font (size 1 1) (thickness 0.15))))\n\t\t(pad \"1\" smd rect (at 0 0) (size 1 1) (layers \"F.Cu\" \"F.Mask\") (net 1 \"GND\") (uuid \"aaaaaaaa-0000-4000-8000-000000000023\"))\n\t)\n)\n";

    fn named(reference: &str) -> Footprint {
        let mut footprint = fixture();
        let field = footprint.reference_field.as_mut().unwrap();
        field.text.as_mut().unwrap().text.as_mut().unwrap().text = reference.into();
        footprint
    }

    /// A KiCad holding `board` with U2 and R1, publishing each `UpdateItems`
    /// it receives and counting them.
    fn guard_kicad(
        board: &std::path::Path,
        updates: std::sync::Arc<std::sync::atomic::AtomicUsize>,
    ) -> crate::test_support::MockIpcServer {
        use kiapi::common::{commands as cmd, types as ct};
        let live = std::sync::Mutex::new(vec![named("U2"), named("R1")]);
        crate::tools::pcb_board::board_mock::spawn_kicad_holding_board(board, move |command| {
            let url = command.type_url.as_str();
            let mut live = live.lock().unwrap();
            if url.ends_with("GetItems") {
                return Some(builders::pack_any(
                    &cmd::GetItemsResponse {
                        header: None,
                        status: ct::ItemRequestStatus::IrsOk as i32,
                        items: live
                            .iter()
                            .map(|footprint| builders::pack_any(footprint, FP_TYPE))
                            .collect(),
                    },
                    "kiapi.common.commands.GetItemsResponse",
                ));
            }
            if url.ends_with("BeginCommit") {
                return Some(builders::pack_any(
                    &cmd::BeginCommitResponse {
                        id: Some(ct::Kiid {
                            value: "commit".into(),
                        }),
                    },
                    "kiapi.common.commands.BeginCommitResponse",
                ));
            }
            if url.ends_with("EndCommit") {
                return Some(builders::pack_any(
                    &cmd::EndCommitResponse {},
                    "kiapi.common.commands.EndCommitResponse",
                ));
            }
            if url.ends_with("UpdateItems") {
                updates.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
                let request = cmd::UpdateItems::decode(command.value.as_slice()).unwrap();
                for item in &request.items {
                    let sent = Footprint::decode(item.value.as_slice()).unwrap();
                    let slot = live
                        .iter_mut()
                        .find(|footprint| reference(footprint) == reference(&sent))
                        .unwrap();
                    *slot = sent;
                }
                return Some(builders::pack_any(
                    &cmd::UpdateItemsResponse {
                        header: None,
                        status: ct::ItemRequestStatus::IrsOk as i32,
                        updated_items: request
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
                ));
            }
            None
        })
    }

    fn text(result: &CallToolResult) -> String {
        match &result.content[0] {
            crate::mcp::protocol::ToolContent::Text { text } => text.clone(),
            other => panic!("unexpected result {other:?}"),
        }
    }

    /// The two fabrication properties a live `edit_footprint_fields` dropped
    /// from a real board.
    const GUARDED_PROPERTIES: [&str; 2] = ["pad_prop_heatsink", "pad_prop_mechanical"];

    fn guard_board(property: &str) -> String {
        GUARD_BOARD.replace("pad_prop_heatsink", property)
    }

    fn guard_project(property: &str) -> (tempfile::TempDir, std::path::PathBuf) {
        let temp = tempfile::tempdir().unwrap();
        let board = temp.path().join("board.kicad_pcb");
        std::fs::write(&board, guard_board(property)).unwrap();
        (temp, board)
    }

    #[tokio::test]
    async fn editing_a_field_of_a_footprint_with_a_pad_fabrication_property_is_refused_before_sending(
    ) {
        for property in GUARDED_PROPERTIES {
            use std::sync::atomic::{AtomicUsize, Ordering};
            let (_temp, board) = guard_project(property);
            let updates = std::sync::Arc::new(AtomicUsize::new(0));
            let server = guard_kicad(&board, updates.clone());
            let ctx =
                crate::tools::pcb_board::board_mock::ctx_talking_to(server.address().to_string());
            let path = board.to_string_lossy().to_string();

            let mut request =
                json!({"board":path,"reference":"U2","fields":[{"name":"Value","visible":false}]});
            for apply in [false, true] {
                if apply {
                    request["dry_run"] = json!(false);
                    request["expected_plan_revision"] = json!("any");
                }
                let refused = handle(&request, &ctx, true).await.unwrap();
                let refusal = text(&refused);
                assert!(refused.is_error, "{refusal}");
                assert!(
                    refusal.contains("plan_blocked")
                        && refusal.contains("edit_footprint_fields")
                        && refusal.contains(&format!("pad '9' {property}"))
                        && refusal.contains("make this change in KiCad itself"),
                    "{refusal}"
                );
            }
            assert_eq!(updates.load(Ordering::SeqCst), 0, "nothing reached KiCad");
            assert_eq!(
                std::fs::read_to_string(&board).unwrap(),
                guard_board(property)
            );

            // A request that changes nothing sends nothing, so it is not refused.
            let noop = body(
            handle(
                &json!({"board":path,"reference":"U2","fields":[{"name":"Value","visible":true}]}),
                &ctx,
                true,
            )
            .await
            .unwrap(),
        );
            assert_eq!(noop["status"], "noop");

            // An ordinary footprint is planned and edited as before.
            let mut plain =
                json!({"board":path,"reference":"R1","fields":[{"name":"Value","visible":false}]});
            let planned = body(handle(&plain, &ctx, true).await.unwrap());
            assert_eq!(planned["status"], "ready");
            plain["dry_run"] = json!(false);
            plain["expected_plan_revision"] = planned["plan_revision"].clone();
            let applied = body(handle(&plain, &ctx, true).await.unwrap());
            assert_eq!(applied["status"], "complete");
            assert_eq!(updates.load(Ordering::SeqCst), 1);
        }
    }

    #[tokio::test]
    async fn setting_values_live_refuses_the_whole_batch_when_one_footprint_has_a_pad_fabrication_property(
    ) {
        for property in GUARDED_PROPERTIES {
            use std::sync::atomic::{AtomicUsize, Ordering};
            let (_temp, board) = guard_project(property);
            let updates = std::sync::Arc::new(AtomicUsize::new(0));
            let server = guard_kicad(&board, updates.clone());
            let ctx =
                crate::tools::pcb_board::board_mock::ctx_talking_to(server.address().to_string());
            let path = board.to_string_lossy().to_string();

            let mut batch = json!({"board":path,"edits":[
                {"reference":"R1","fields":{"Value":"1uF"}},
                {"reference":"U2","fields":{"Value":"1uF","MPN":"X"}}
            ]});
            for apply in [false, true] {
                if apply {
                    batch["dry_run"] = json!(false);
                    batch["expected_revision"] = json!("any");
                }
                let refused = values::handle_set_footprint_field_values(&batch, &ctx)
                    .await
                    .unwrap();
                let refusal = text(&refused);
                assert!(refused.is_error, "{refusal}");
                assert!(
                    refusal.contains("plan_blocked")
                        && refusal.contains("set_footprint_field_values")
                        && refusal.contains("U2 (")
                        && refusal.contains(&format!("pad '9' {property}"))
                        && refusal.contains("close it in KiCad")
                        && !refusal.contains("R1 ("),
                    "{refusal}"
                );
            }
            assert_eq!(updates.load(Ordering::SeqCst), 0, "nothing reached KiCad");
            assert_eq!(
                std::fs::read_to_string(&board).unwrap(),
                guard_board(property)
            );

            // Naming the footprint without changing it sends nothing of it.
            let mut plain = json!({"board":path,"edits":[
                {"reference":"R1","fields":{"Value":"1uF"}},
                {"reference":"U2","fields":{"Value":"100nF"}}
            ]});
            let planned = body(
                values::handle_set_footprint_field_values(&plain, &ctx)
                    .await
                    .unwrap(),
            );
            assert_eq!(planned["status"], "ready");
            assert_eq!(planned["changed"], 1);
            plain["dry_run"] = json!(false);
            plain["expected_revision"] = planned["plan_revision"].clone();
            let applied = body(
                values::handle_set_footprint_field_values(&plain, &ctx)
                    .await
                    .unwrap(),
            );
            assert_eq!(applied["status"], "complete");
            assert_eq!(updates.load(Ordering::SeqCst), 1);
        }
    }

    /// The remedy the live refusal names: with the board closed the same
    /// values are set in the saved file and the pad clause is untouched.
    #[tokio::test]
    async fn the_closed_board_path_sets_values_and_keeps_the_pad_fabrication_property() {
        for property in GUARDED_PROPERTIES {
            let (_temp, board) = guard_project(property);
            let ctx = crate::tools::pcb_board::board_mock::ctx_talking_to(String::new());
            let mut request = json!({"board":board.to_string_lossy(),"edits":[
                {"reference":"U2","fields":{"Value":"1uF","MPN":"X"}}
            ]});
            let planned = body(
                values::handle_set_footprint_field_values(&request, &ctx)
                    .await
                    .unwrap(),
            );
            assert_eq!(planned["status"], "ready", "{planned}");
            request["dry_run"] = json!(false);
            request["expected_revision"] = planned["plan_revision"].clone();
            let applied = values::handle_set_footprint_field_values(&request, &ctx)
                .await
                .unwrap();
            assert!(!applied.is_error, "{}", text(&applied));
            let written = std::fs::read_to_string(&board).unwrap();
            assert!(written.contains("(property \"Value\" \"1uF\""), "{written}");
            assert!(written.contains("(property \"MPN\" \"X\""), "{written}");
            let pad = "(pad \"9\" smd rect (at 0 0) (size 2.95 4.9) (property pad_prop_heatsink) (layers \"F.Cu\" \"F.Mask\") (net 1 \"GND\") (uuid \"aaaaaaaa-0000-4000-8000-000000000013\"))".replace("pad_prop_heatsink", property);
            assert!(written.contains(&pad), "{property}: {written}");
        }
    }
}
