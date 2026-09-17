//! Exact live pad translations; never reconstruct a footprint or write a board file.
use super::{
    pcb_board::{attempt_ipc_write, BoardWrite},
    ToolDef,
};
use crate::mcp::protocol::CallToolResult;
use anyhow::{ensure, Context, Result};
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
#[derive(Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Shift {
    uuid: String,
    dx_mm: f64,
    dy_mm: f64,
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Args {
    board: String,
    reference: String,
    shifts: Vec<Shift>,
    #[serde(default = "yes")]
    dry_run: bool,
    expected_plan_revision: Option<String>,
}
fn yes() -> bool {
    true
}
fn validate(shifts: &[Shift]) -> Result<()> {
    ensure!(!shifts.is_empty(), "shifts must be nonempty");
    let mut ids = BTreeSet::new();
    for s in shifts {
        ensure!(
            !s.uuid.is_empty() && ids.insert(&s.uuid),
            "empty or duplicate pad UUID"
        );
        ensure!(
            [s.dx_mm, s.dy_mm]
                .iter()
                .all(|v| v.is_finite() && v.abs() <= 1.0),
            "local displacement must be finite and within +/-1 mm"
        );
    }
    Ok(())
}
fn prepare(old: &Footprint, shifts: &[Shift]) -> Result<(Footprint, Value)> {
    ensure!(
        old.layer == builders::layer_from_name("F.Cu") as i32
            && old
                .orientation
                .as_ref()
                .is_none_or(|a| a.value_degrees.abs() < 1e-9),
        "pad translation currently supports unrotated front footprints only"
    );
    validate(shifts)?;
    let mut new = old.clone();
    let mut report = Vec::new();
    let items = &mut new
        .definition
        .as_mut()
        .context("missing footprint definition")?
        .items;
    for s in shifts {
        let mut count = 0;
        for item in items.iter_mut() {
            if !builders::any_is(item, PAD) {
                continue;
            }
            let mut pad = Pad::decode(item.value.as_slice())?;
            if pad.id.as_ref().is_none_or(|id| id.value != s.uuid) {
                continue;
            }
            count += 1;
            let p = pad.position.as_mut().context("pad position absent")?;
            let before = *p;
            p.x_nm = p
                .x_nm
                .checked_add(builders::mm_to_nm(s.dx_mm))
                .context("X overflow")?;
            p.y_nm = p
                .y_nm
                .checked_add(builders::mm_to_nm(s.dy_mm))
                .context("Y overflow")?;
            report.push(json!({"uuid":s.uuid,"number":pad.number,"before_ipc_mm":[before.x_nm as f64/1e6,before.y_nm as f64/1e6],"after_ipc_mm":[p.x_nm as f64/1e6,p.y_nm as f64/1e6]}));
            *item = builders::pack_any(&pad, PAD);
        }
        ensure!(count == 1, "pad UUID missing or ambiguous: {}", s.uuid);
    }
    Ok((new, json!(report)))
}
fn select(items: Vec<prost_types::Any>, reference: &str) -> Result<Footprint> {
    let mut matches = Vec::new();
    for item in items {
        ensure!(
            builders::any_is(&item, FP),
            "unexpected footprint item type"
        );
        let fp = Footprint::decode(item.value.as_slice())?;
        if fp
            .reference_field
            .as_ref()
            .and_then(|f| f.text.as_ref())
            .and_then(|t| t.text.as_ref())
            .is_some_and(|t| t.text == reference)
        {
            matches.push(fp)
        }
    }
    ensure!(
        matches.len() == 1,
        "footprint reference missing or ambiguous"
    );
    Ok(matches.remove(0))
}
fn revision(path: &str, old: &Footprint, new: &Footprint) -> String {
    let mut hash = Sha256::new();
    for b in [
        path.as_bytes().to_vec(),
        old.encode_to_vec(),
        new.encode_to_vec(),
    ] {
        hash.update((b.len() as u64).to_le_bytes());
        hash.update(b);
    }
    format!("{:x}", hash.finalize())
}
pub(super) fn tool() -> ToolDef {
    tool!("translate_footprint_pads","Plan or translate exact pads by UUID in IPC-reported axes on unrotated front footprints through live IPC. Each displacement is limited to +/-1 mm. Preserves every non-position pad/footprint field. Review the reported before/after coordinates. Dry run and exact revision required; applies in one undo commit and verifies the whole footprint. No board-file fallback; does not update the library or attached routing. Refill/save/run DRC afterward.",
 json!({"type":"object","additionalProperties":false,"properties":{"board":{"type":"string"},"reference":{"type":"string"},"shifts":{"type":"array","minItems":1,"items":{"type":"object","additionalProperties":false,"properties":{"uuid":{"type":"string"},"dx_mm":{"type":"number","minimum":-1,"maximum":1},"dy_mm":{"type":"number","minimum":-1,"maximum":1}},"required":["uuid","dx_mm","dy_mm"]}},"dry_run":{"type":"boolean","default":true},"expected_plan_revision":{"type":"string"}},"required":["board","reference","shifts"]}),
 |args,ctx|async move {handle(args,ctx).await}
 ).with_board_access(super::BoardAccess::LiveOnly)
}
async fn handle(args: &Value, ctx: &super::ToolContext) -> Result<CallToolResult> {
    let a: Args = serde_json::from_value(args.clone())?;
    ensure!(
        !a.board.is_empty() && !a.reference.is_empty(),
        "board and reference required"
    );
    validate(&a.shifts)?;
    ensure!(
        a.dry_run || a.expected_plan_revision.is_some(),
        "apply requires revision"
    );
    let board = super::get_path(args, "board")?;
    let path = board.clone();
    let result=attempt_ipc_write(ctx,&board,"pad position edit",move|client|{
   let doc=client.find_open_board(&path)?;let kind=kiapi::common::types::KiCadObjectType::KotPcbFootprint;
   let old=select(client.get_items_in(doc.clone(),kind)?,&a.reference)?;
   let (new,changes)=prepare(&old,&a.shifts)?;let rev=revision(&path.to_string_lossy(),&old,&new);
   if !a.dry_run {ensure!(a.expected_plan_revision.as_deref()==Some(&rev),"stale plan revision");}
   if a.dry_run || old==new {return Ok(CallToolResult::json(&json!({"status":if old==new{"noop"}else{"ready"},"plan_revision":rev,"changes":changes,"applied":false})));}
   client.run_commit("Translate selected footprint pads",|c|c.update_items_in(doc.clone(),vec![builders::pack_any(&new,FP)]))?;
   let actual=select(client.get_items_in(doc.clone(),kind)?,&a.reference)?;
   if comparable(&actual)? != comparable(&new)? {
    client.run_commit("Restore footprint after pad readback mismatch",|c|c.update_items_in(doc.clone(),vec![builders::pack_any(&old,FP)]))?;
    ensure!(select(client.get_items_in(doc,kind)?,&a.reference)?==old,"pad readback differed and restoration failed; inspect live board");
    anyhow::bail!("pad readback differed; original footprint restored. Differences: {}", describe_difference(&new, &actual));
   }
   Ok(CallToolResult::json(&json!({"status":"complete","plan_revision":rev,"changes":changes,"applied":true,"saved":false,"whole_footprint_readback_verified":true})))
  }).await?;
    Ok(match result {
        BoardWrite::Ipc(r) | BoardWrite::Refused(r) => r,
        BoardWrite::File(reason) => CallToolResult::error(format!(
            "{} Pad translations require live IPC.",
            reason.premise()
        )),
    })
}

fn comparable(fp: &Footprint) -> Result<Footprint> {
    let mut result = fp.clone();
    for item in &mut result
        .definition
        .as_mut()
        .context("missing definition")?
        .items
    {
        if builders::any_is(item, PAD) {
            // Any contains protobuf wire bytes. KiCad can emit equivalent fields in a
            // different order; compare every decoded Pad field, not wire ordering.
            let pad = Pad::decode(item.value.as_slice())?;
            *item = builders::pack_any(&pad, PAD);
        }
    }
    Ok(result)
}

fn describe_difference(expected: &Footprint, actual: &Footprint) -> Value {
    let mut differences = Vec::new();
    if let (Some(e), Some(a)) = (&expected.definition, &actual.definition) {
        for (i, (x, y)) in e.items.iter().zip(&a.items).enumerate() {
            if x != y {
                if builders::any_is(x, PAD) && builders::any_is(y, PAD) {
                    if let (Ok(p), Ok(q)) = (
                        Pad::decode(x.value.as_slice()),
                        Pad::decode(y.value.as_slice()),
                    ) {
                        differences.push(json!({"item":i,"expected_pad":format!("{p:?}"),"actual_pad":format!("{q:?}")}));
                    }
                } else {
                    differences.push(
                        json!({"item":i,"expected_type":x.type_url,"actual_type":y.type_url}),
                    );
                }
            }
        }
    }
    let mut e = expected.clone();
    let mut a = actual.clone();
    e.definition = None;
    a.definition = None;
    json!({"non_definition_equal":e==a,"items":differences})
}

#[cfg(test)]
mod tests {
    use super::*;
    fn fixture() -> Footprint {
        let pad = Pad {
            id: Some(kiapi::common::types::Kiid {
                value: "pad1".into(),
            }),
            number: "A1".into(),
            position: Some(kiapi::common::types::Vector2 {
                x_nm: 3200000,
                y_nm: -1075000,
            }),
            net: Some(kiapi::board::types::Net {
                code: Some(kiapi::board::types::NetCode { value: 7 }),
                name: "GND".into(),
            }),
            ..Default::default()
        };
        Footprint {
            layer: builders::layer_from_name("F.Cu") as i32,
            definition: Some(kiapi::board::types::Footprint {
                items: vec![
                    builders::pack_any(&pad, PAD),
                    prost_types::Any {
                        type_url: "untouched".into(),
                        value: vec![1, 2, 3],
                    },
                ],
                ..Default::default()
            }),
            ..Default::default()
        }
    }
    fn shift() -> Shift {
        Shift {
            uuid: "pad1".into(),
            dx_mm: 0.0,
            dy_mm: -0.03,
        }
    }
    #[test]
    fn pad_translate_preserves_everything_except_position() {
        let old = fixture();
        let (mut new, _) = prepare(&old, &[shift()]).unwrap();
        let i = &mut new.definition.as_mut().unwrap().items[0];
        let mut p = Pad::decode(i.value.as_slice()).unwrap();
        assert_eq!(p.position.unwrap().y_nm, -1105000);
        p.position.as_mut().unwrap().y_nm += 30000;
        *i = builders::pack_any(&p, PAD);
        assert_eq!(new, old);
    }
    #[test]
    fn pad_translate_missing_duplicate_and_large_shifts_refuse() {
        let old = fixture();
        assert!(prepare(&old, &[]).is_err());
        assert!(prepare(&old, &[shift(), shift()]).is_err());
        let mut s = shift();
        s.uuid = "missing".into();
        assert!(prepare(&old, &[s]).is_err());
        let mut s = shift();
        s.dx_mm = 1.1;
        assert!(prepare(&old, &[s]).is_err());
    }
    #[test]
    fn pad_translate_revision_binds_whole_footprint_and_board() {
        let old = fixture();
        let (new, _) = prepare(&old, &[shift()]).unwrap();
        assert_ne!(revision("a", &old, &new), revision("b", &old, &new));
        assert_ne!(revision("a", &old, &new), revision("a", &new, &new));
    }
    #[test]
    fn pad_translate_readback_ignores_only_protobuf_field_order() {
        let original = fixture();
        let mut reordered = original.clone();
        let item = &mut reordered.definition.as_mut().unwrap().items[0];
        assert_eq!(item.value[0], 10); // first field: length-delimited pad ID
        let boundary = 2 + item.value[1] as usize;
        item.value.rotate_left(boundary);
        assert_ne!(original, reordered);
        assert_eq!(
            comparable(&original).unwrap(),
            comparable(&reordered).unwrap()
        );
        let changed = prepare(&reordered, &[shift()]).unwrap().0;
        assert_ne!(
            comparable(&original).unwrap(),
            comparable(&changed).unwrap()
        );
    }
    #[test]
    fn pad_translate_refuses_unverified_coordinate_frames() {
        let mut old = fixture();
        old.layer = builders::layer_from_name("B.Cu") as i32;
        assert!(prepare(&old, &[shift()]).is_err());
        old.layer = builders::layer_from_name("F.Cu") as i32;
        old.orientation = Some(kiapi::common::types::Angle {
            value_degrees: 180.0,
        });
        assert!(prepare(&old, &[shift()]).is_err());
    }
}
