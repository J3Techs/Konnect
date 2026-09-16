//! Explicit, layer-scoped custom paste apertures. No copper or mask edits.
use crate::tools::{get_path, with_board_ipc_classified, BoardAccess, ToolDef};
use crate::{mcp::protocol::CallToolResult, tool};
use anyhow::{ensure, Context};
use konnect_ipc::{builders, gen::kiapi};
use konnect_sexp::writer::{
    apply_edits, find_direct_child_blocks, read_consistent, write_atomic_if_unchanged, SexpEdit,
};
use prost::Message;
use serde::Deserialize;
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;

#[derive(Clone, Debug, Deserialize)]
struct Point {
    x: f64,
    y: f64,
}
#[derive(Clone, Debug, Deserialize)]
struct Aperture {
    pad_number: String,
    points: Vec<Point>,
}
fn apertures(args: &Value) -> anyhow::Result<Vec<Aperture>> {
    let a: Vec<Aperture> = serde_json::from_value(args["apertures"].clone())?;
    ensure!(!a.is_empty(), "apertures must not be empty");
    for p in &a {
        ensure!(
            !p.pad_number.is_empty() && p.points.len() >= 3,
            "each aperture needs a pad number and at least three points"
        );
        ensure!(
            p.points.iter().all(|v| v.x.is_finite() && v.y.is_finite()),
            "nonfinite point"
        );
        let area: f64 = p
            .points
            .iter()
            .zip(p.points.iter().cycle().skip(1))
            .take(p.points.len())
            .map(|(a, b)| a.x * b.y - b.x * a.y)
            .sum();
        ensure!(area.abs() > 1e-10, "degenerate aperture");
    }
    Ok(a)
}
fn schema(board: bool) -> Value {
    let mut s = json!({"type":"object","additionalProperties":false,"properties":{
        "apertures":{"type":"array","minItems":1,"items":{"type":"object","additionalProperties":false,"properties":{
            "pad_number":{"type":"string","minLength":1},"points":{"type":"array","minItems":3,"items":{"type":"object","additionalProperties":false,"properties":{"x":{"type":"number"},"y":{"type":"number"}},"required":["x","y"]}}
        },"required":["pad_number","points"]}},
        "dry_run":{"type":"boolean","default":true},"expected_plan_revision":{"type":"string"}
    },"required":["apertures"]});
    for key in if board {
        vec!["board", "reference"]
    } else {
        vec!["footprint_path"]
    } {
        s["properties"][key] = json!({"type":"string"});
        s["required"].as_array_mut().unwrap().push(json!(key));
    }
    s
}
fn revision(bytes: &[u8], args: &Value) -> String {
    let mut h = Sha256::new();
    h.update(bytes);
    h.update(args["apertures"].to_string());
    format!("{:x}", h.finalize())
}
fn apply_requested(args: &Value, rev: &str) -> anyhow::Result<bool> {
    let apply = !args["dry_run"].as_bool().unwrap_or(true);
    if apply {
        ensure!(
            args["expected_plan_revision"].as_str() == Some(rev),
            "stale or missing expected_plan_revision; rerun dry run"
        );
    }
    Ok(apply)
}
pub(super) fn library_tool() -> ToolDef {
    tool!("set_library_footprint_paste_apertures",
        "Replace all front paste zones/graphics in a footprint library with explicit filled polygons in footprint-local mm. Disable automatic paste on the explicitly covered SMD pads. Preserves copper, mask, pad geometry and all other layers. Requires coverage of every pad and refuses groups; dry run and exact revision required.",schema(false),
        |args,_ctx|async move {
            let p=get_path(args,"footprint_path")?;ensure!(p.extension().and_then(|v|v.to_str())==Some("kicad_mod"),"expected .kicad_mod");
            let old=read_consistent(&p)?;let a=apertures(args)?;let new=library_replacement(&old,&a)?;let rev=revision(old.as_bytes(),args);
            let apply=apply_requested(args,&rev)?;if apply {write_atomic_if_unchanged(&p,&old,&new)?;ensure!(read_consistent(&p)?==new,"library readback differs");}
            Ok(CallToolResult::json(&json!({"dry_run":!apply,"plan_revision":rev,"apertures":a.len(),"footprint_path":p,"scope":"F.Paste only"})))
        })
}
pub(super) fn board_tool() -> ToolDef {
    tool!("set_board_footprint_paste_apertures",
        "Replace all front paste zones/graphics in one live front footprint with explicit filled polygons in footprint-local mm. Disable automatic paste on the explicitly covered SMD pads. Preserves copper, mask, geometry and other layers. Requires every pad covered and no groups. Dry run and exact revision required; apply uses one undo commit and readback. Does not edit the library.",schema(true),
        |args,ctx|async move {
            let path=get_path(args,"board")?;let args=args.clone();let a=apertures(&args)?;
            let r=with_board_ipc_classified(ctx,&path,move|c| {
                let reference=args["reference"].as_str().context("missing reference")?;
                let old=find_footprint(c,reference)?;let new=board_replacement(&old,&a)?;
                let rev=revision(&old.encode_to_vec(),&args);let apply=apply_requested(&args,&rev)?;
                if apply {
                    // KiCad defers footprint remove/add until the undo commit is pushed.
                    // Reading inside the commit sees the old instance, not the update.
                    c.run_commit("Replace footprint paste apertures",|c|c.update_items(vec![builders::pack_any(&new,"kiapi.board.types.FootprintInstance")]))?;
                    let checked=find_footprint(c,reference).and_then(|actual|verify_readback(&old,&new,&actual));
                    if let Err(error)=checked {
                        c.run_commit("Restore footprint after failed paste verification",|c|c.update_items(vec![builders::pack_any(&old,"kiapi.board.types.FootprintInstance")]))?;
                        ensure!(find_footprint(c,reference)?==old,"paste verification failed ({error}); restoration readback also failed");
                        anyhow::bail!("paste verification failed; original footprint restored: {error}");
                    }
                }
                Ok(json!({"reference":reference,"dry_run":!apply,"plan_revision":rev,"apertures":a.len(),"scope":"F.Paste only"}))
            }).await?;
            match r {Ok(v)=>Ok(CallToolResult::json(&v)),Err(e)=>Ok(CallToolResult::error(format!("live paste update refused: {e}")))}
        }).with_board_access(BoardAccess::LiveOnly)
}
fn find_footprint(
    c: &konnect_ipc::client::KiCadIpcClient,
    r: &str,
) -> anyhow::Result<kiapi::board::types::FootprintInstance> {
    let mut matches = Vec::new();
    for item in c.get_items(kiapi::common::types::KiCadObjectType::KotPcbFootprint)? {
        ensure!(
            builders::any_is(&item, "kiapi.board.types.FootprintInstance"),
            "unexpected item type"
        );
        let fp = kiapi::board::types::FootprintInstance::decode(item.value.as_slice())?;
        if fp
            .reference_field
            .as_ref()
            .and_then(|f| f.text.as_ref())
            .and_then(|t| t.text.as_ref())
            .map(|t| t.text.as_str())
            == Some(r)
        {
            matches.push(fp);
        }
    }
    ensure!(matches.len() == 1, "expected one footprint {r}");
    Ok(matches.remove(0))
}
fn is_paste(item: &prost_types::Any) -> anyhow::Result<bool> {
    let layer = kiapi::board::types::BoardLayer::BlFPaste as i32;
    if builders::any_is(item, "kiapi.board.types.BoardGraphicShape") {
        return Ok(
            kiapi::board::types::BoardGraphicShape::decode(item.value.as_slice())?.layer == layer,
        );
    }
    if builders::any_is(item, "kiapi.board.types.Zone") {
        let z = kiapi::board::types::Zone::decode(item.value.as_slice())?;
        if z.layers.contains(&layer) {
            ensure!(
                z.layers == vec![layer],
                "multi-layer paste zone unsupported"
            );
            return Ok(true);
        }
    }
    Ok(false)
}
fn board_replacement(
    old: &kiapi::board::types::FootprintInstance,
    a: &[Aperture],
) -> anyhow::Result<kiapi::board::types::FootprintInstance> {
    use kiapi::board::types::*;
    ensure!(
        old.layer == BoardLayer::BlFCu as i32,
        "only front footprints supported"
    );
    let mut fp = old.clone();
    let def = fp.definition.as_mut().context("no footprint definition")?;
    let mut numbers = BTreeSet::new();
    let wanted: BTreeSet<_> = a.iter().map(|a| a.pad_number.clone()).collect();
    let mut items = Vec::new();
    for item in &def.items {
        ensure!(
            !item.type_url.ends_with(".Group"),
            "grouped footprints unsupported"
        );
        if is_paste(item)? {
            continue;
        }
        if builders::any_is(item, "kiapi.board.types.Pad") {
            let mut pad = Pad::decode(item.value.as_slice())?;
            ensure!(
                pad.r#type == PadType::PtSmd as i32,
                "only SMD pads supported"
            );
            ensure!(numbers.insert(pad.number.clone()), "duplicate pad number");
            let stack = pad.pad_stack.as_mut().context("missing padstack")?;
            ensure!(
                stack.layers.contains(&(BoardLayer::BlFCu as i32))
                    && !stack.layers.contains(&(BoardLayer::BlBCu as i32)),
                "front SMD pad required"
            );
            stack.layers.retain(|l| *l != BoardLayer::BlFPaste as i32);
            if let Some(front) = stack.front_outer_layers.as_mut() {
                front.solder_paste_settings = None;
                front.solder_paste_mode = SolderPasteMode::SpmNoPaste as i32;
            }
            items.push(builders::pack_any(&pad, "kiapi.board.types.Pad"));
        } else {
            items.push(item.clone());
        }
    }
    ensure!(
        numbers == wanted,
        "apertures must cover every pad exactly by number"
    );
    let origin = old.position.as_ref().context("missing position")?;
    let angle = old
        .orientation
        .as_ref()
        .map(|a| a.value_degrees)
        .unwrap_or(0.0)
        .to_radians();
    for aperture in a {
        let pts = aperture
            .points
            .iter()
            .map(|p| {
                (
                    builders::nm_to_mm(origin.x_nm) + p.x * angle.cos() + p.y * angle.sin(),
                    builders::nm_to_mm(origin.y_nm) - p.x * angle.sin() + p.y * angle.cos(),
                )
            })
            .collect::<Vec<_>>();
        items.push(builders::pack_any(
            &builders::board_polygon("F.Paste", 0.0, true, &[pts]),
            "kiapi.board.types.BoardGraphicShape",
        ));
    }
    def.items = items;
    Ok(fp)
}
fn verify_readback(
    old: &kiapi::board::types::FootprintInstance,
    expected: &kiapi::board::types::FootprintInstance,
    actual: &kiapi::board::types::FootprintInstance,
) -> anyhow::Result<()> {
    ensure!(
        actual.position == old.position
            && actual.orientation == old.orientation
            && actual.id == old.id
            && actual.layer == old.layer,
        "placement/identity changed"
    );
    let metadata = |f: &kiapi::board::types::FootprintInstance| {
        let mut copy = f.clone();
        if let Some(d) = copy.definition.as_mut() {
            d.items.clear();
        }
        copy
    };
    ensure!(
        metadata(old) == metadata(actual),
        "protected footprint metadata changed"
    );
    let items=|f:&kiapi::board::types::FootprintInstance|->anyhow::Result<(Vec<Vec<u8>>,Vec<Vec<u8>>)> {
        let mut rest=Vec::new();let mut paste=Vec::new();
        for i in &f.definition.as_ref().context("missing definition")?.items {
            if is_paste(i)? {
                ensure!(builders::any_is(i,"kiapi.board.types.BoardGraphicShape"),"old paste zone survived replacement");
                let mut shape=kiapi::board::types::BoardGraphicShape::decode(i.value.as_slice())?;
                shape.id=None;
                ensure!(shape.net.as_ref().is_none_or(|n|n.name.is_empty()),"paste polygon unexpectedly has a net");
                shape.net=None;
                if let Some(stroke)=shape.shape.as_mut().and_then(|s|s.attributes.as_mut()).and_then(|a|a.stroke.as_mut()) {
                    use kiapi::common::types::StrokeLineStyle::*;
                    if matches!(stroke.style(),SlsUnknown|SlsDefault|SlsSolid){stroke.style=SlsSolid as i32;}
                }
                paste.push(shape.encode_to_vec());
            } else {rest.push(i.encode_to_vec());}
        }
        rest.sort();paste.sort();Ok((rest,paste))
    };
    ensure!(
        items(expected)? == items(actual)?,
        "paste or protected item readback mismatch"
    );
    Ok(())
}
fn library_replacement(source: &str, a: &[Aperture]) -> anyhow::Result<String> {
    let tree = konnect_sexp::parse_sexp(source)?;
    ensure!(tree.head() == Some("footprint"), "expected footprint root");
    let wanted: BTreeSet<_> = a.iter().map(|a| a.pad_number.clone()).collect();
    let mut numbers = BTreeSet::new();
    let mut edits = Vec::new();
    for (start, end) in find_direct_child_blocks(source, "footprint") {
        let block = &source[start..end];
        let node = konnect_sexp::parse_sexp(block)?;
        let tag = node.head().unwrap_or("");
        ensure!(tag != "group", "grouped footprints unsupported");
        if node.find_str("layer") == Some("F.Paste") {
            ensure!(
                tag == "zone" || tag.starts_with("fp_"),
                "unsupported paste item"
            );
            edits.push(SexpEdit {
                start,
                end,
                replacement: String::new(),
            });
            continue;
        }
        if tag != "pad" {
            continue;
        }
        let children = node.children().context("pad children")?;
        let number = children
            .get(1)
            .and_then(|x| x.as_str())
            .context("pad number")?;
        ensure!(
            children.get(2).and_then(|x| x.as_str()) == Some("smd"),
            "only SMD pads supported"
        );
        ensure!(numbers.insert(number.to_string()), "duplicate pad number");
        let layers = node
            .find("layers")
            .and_then(|x| x.children())
            .context("pad layers")?;
        let names: Vec<_> = layers.iter().skip(1).filter_map(|x| x.as_str()).collect();
        ensure!(
            names.contains(&"F.Cu") && !names.contains(&"B.Cu"),
            "front SMD pad required"
        );
        let mut pad_edits = Vec::new();
        for (s, e) in find_direct_child_blocks(block, "pad") {
            let n = konnect_sexp::parse_sexp(&block[s..e])?;
            match n.head() {
                Some("layers") => pad_edits.push(SexpEdit {
                    start: s,
                    end: e,
                    replacement: format!(
                        "(layers {})",
                        names
                            .iter()
                            .filter(|n| **n != "F.Paste")
                            .map(|n| format!("\"{n}\""))
                            .collect::<Vec<_>>()
                            .join(" ")
                    ),
                }),
                Some("solder_paste_margin" | "solder_paste_margin_ratio") => {
                    pad_edits.push(SexpEdit {
                        start: s,
                        end: e,
                        replacement: String::new(),
                    })
                }
                _ => {}
            }
        }
        edits.push(SexpEdit {
            start,
            end,
            replacement: apply_edits(block.to_string(), pad_edits),
        });
    }
    ensure!(
        numbers == wanted,
        "apertures must cover every pad by number"
    );
    let mut polygons = String::new();
    for p in a {
        polygons.push_str(&format!("\n(fp_poly (pts {}) (stroke (width 0) (type solid)) (fill solid) (layer \"F.Paste\") (uuid \"{}\"))\n",p.points.iter().map(|p|format!("(xy {:.8} {:.8})",p.x,p.y)).collect::<Vec<_>>().join(" "),konnect_sexp::writer::new_uuid()));
    }
    let end = source.rfind(')').context("missing closing parenthesis")?;
    edits.push(SexpEdit {
        start: end,
        end,
        replacement: polygons,
    });
    let result = apply_edits(source.to_string(), edits);
    konnect_sexp::parse_sexp(&result)?;
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn input() -> Vec<Aperture> {
        apertures(&json!({"apertures":[{"pad_number":"1","points":[{"x":-0.4,"y":-0.3},{"x":0.4,"y":-0.3},{"x":0.4,"y":0.3},{"x":-0.4,"y":0.3}]}]})).unwrap()
    }
    const LIB: &str = r#"(footprint "test" (version 20241229) (layer "F.Cu")
        (fp_line (start 0 0) (end 1 0) (stroke (width 0.1) (type solid)) (layer "F.SilkS") (uuid "silk"))
        (pad "1" smd rect (at 0 0) (size 0.8 0.6) (layers "F.Cu" "F.Mask" "F.Paste") (solder_mask_margin 0.05) (solder_paste_margin -100) (uuid "pad"))
        (zone (layer "F.Paste") (uuid "old-paste") (polygon (pts (xy 1 2) (xy 2 2) (xy 2 3)))))"#;
    #[test]
    fn library_preserves_copper_mask_and_other_artwork() {
        let out = library_replacement(LIB, &input()).unwrap();
        let tree = konnect_sexp::parse_sexp(&out).unwrap();
        let pad = tree.find("pad").unwrap();
        assert_eq!(pad.find_str("uuid"), Some("pad"));
        assert_eq!(pad.find_f64("solder_mask_margin"), Some(0.05));
        assert_eq!(
            pad.find("size"),
            konnect_sexp::parse_sexp(LIB)
                .unwrap()
                .find("pad")
                .unwrap()
                .find("size")
        );
        assert!(pad.find("solder_paste_margin").is_none());
        assert_eq!(tree.find_all("fp_poly").len(), 1);
        assert!(tree.find("zone").is_none());
        assert!(out.contains("(layers \"F.Cu\" \"F.Mask\")"));
        assert!(out.contains("(uuid \"silk\")"));
    }
    #[test]
    fn library_rejects_incomplete_duplicate_through_hole_or_grouped() {
        let mut a = input();
        a[0].pad_number = "2".into();
        assert!(library_replacement(LIB, &a).is_err());
        assert!(library_replacement(&LIB.replace("smd rect", "thru_hole rect"), &input()).is_err());
        assert!(library_replacement(
            &LIB.replace("(version 20241229)", "(group \"g\" (members \"pad\"))"),
            &input()
        )
        .is_err());
        assert!(library_replacement(
            &LIB.replace(
                "(version 20241229)",
                "(pad \"1\" smd rect (layers \"F.Cu\"))"
            ),
            &input()
        )
        .is_err());
    }
    #[test]
    fn plan_revision_covers_content_and_apertures() {
        let args = json!({"apertures":[],"dry_run":false});
        let rev = revision(b"old", &args);
        assert!(apply_requested(&args, &rev).is_err());
        let mut args = args;
        args["expected_plan_revision"] = json!(rev);
        assert!(apply_requested(&args, &rev).unwrap());
        assert_ne!(rev, revision(b"changed", &args));
        args["apertures"] = json!([1]);
        assert_ne!(rev, revision(b"old", &args));
    }
    fn live() -> kiapi::board::types::FootprintInstance {
        use kiapi::board::types::*;
        let pad = Pad {
            number: "1".into(),
            r#type: PadType::PtSmd as i32,
            position: Some(builders::vec2(10.0, 20.0)),
            pad_stack: Some(PadStack {
                layers: vec![
                    BoardLayer::BlFCu as i32,
                    BoardLayer::BlFPaste as i32,
                    BoardLayer::BlFMask as i32,
                ],
                front_outer_layers: Some(PadStackOuterLayer {
                    solder_mask_settings: Some(SolderMaskOverrides {
                        solder_mask_margin: Some(kiapi::common::types::Distance {
                            value_nm: 50000,
                        }),
                    }),
                    ..Default::default()
                }),
                ..Default::default()
            }),
            ..Default::default()
        };
        FootprintInstance {
            layer: BoardLayer::BlFCu as i32,
            position: Some(builders::vec2(10.0, 20.0)),
            orientation: Some(kiapi::common::types::Angle {
                value_degrees: 90.0,
            }),
            definition: Some(Footprint {
                items: vec![
                    builders::pack_any(&pad, "kiapi.board.types.Pad"),
                    builders::pack_any(
                        &builders::board_polygon(
                            "F.SilkS",
                            0.1,
                            false,
                            &[vec![(1.0, 2.0), (2.0, 2.0), (2.0, 3.0)]],
                        ),
                        "kiapi.board.types.BoardGraphicShape",
                    ),
                ],
                ..Default::default()
            }),
            ..Default::default()
        }
    }
    #[test]
    fn board_preserves_pad_and_rotates_local_paste() {
        use kiapi::board::types::*;
        let old = live();
        let new = board_replacement(&old, &input()).unwrap();
        let items = &new.definition.as_ref().unwrap().items;
        let pad = Pad::decode(items[0].value.as_slice()).unwrap();
        assert_eq!(pad.position, Some(builders::vec2(10.0, 20.0)));
        assert!(!pad
            .pad_stack
            .as_ref()
            .unwrap()
            .layers
            .contains(&(BoardLayer::BlFPaste as i32)));
        assert_eq!(items[1], old.definition.as_ref().unwrap().items[1]);
        let shape = BoardGraphicShape::decode(items[2].value.as_slice()).unwrap();
        let expected = builders::board_polygon(
            "F.Paste",
            0.0,
            true,
            &[vec![(9.7, 20.4), (9.7, 19.6), (10.3, 19.6), (10.3, 20.4)]],
        );
        assert_eq!(shape.shape, expected.shape);
        verify_readback(&old, &new, &new).unwrap();
        let mut broken = new.clone();
        broken.position = Some(builders::vec2(11.0, 20.0));
        assert!(verify_readback(&old, &new, &broken).is_err());
    }
    #[test]
    fn board_refuses_backside_and_missing_pad() {
        let mut old = live();
        old.layer = kiapi::board::types::BoardLayer::BlBCu as i32;
        assert!(board_replacement(&old, &input()).is_err());
        let old = live();
        let mut a = input();
        a[0].pad_number = "2".into();
        assert!(board_replacement(&old, &a).is_err());
    }
    #[test]
    fn tools_have_valid_schemas() {
        let _ = board_tool();
        let _ = library_tool();
    }
    #[test]
    fn readback_accepts_only_equivalent_kicad_graphic_defaults() {
        let old = live();
        let expected = board_replacement(&old, &input()).unwrap();
        let mut actual = expected.clone();
        let item = actual
            .definition
            .as_mut()
            .unwrap()
            .items
            .last_mut()
            .unwrap();
        let mut shape =
            kiapi::board::types::BoardGraphicShape::decode(item.value.as_slice()).unwrap();
        shape.net = Some(Default::default());
        shape
            .shape
            .as_mut()
            .unwrap()
            .attributes
            .as_mut()
            .unwrap()
            .stroke
            .as_mut()
            .unwrap()
            .style = kiapi::common::types::StrokeLineStyle::SlsSolid as i32;
        *item = builders::pack_any(&shape, "kiapi.board.types.BoardGraphicShape");
        verify_readback(&old, &expected, &actual).unwrap();
        let item = actual
            .definition
            .as_mut()
            .unwrap()
            .items
            .last_mut()
            .unwrap();
        shape.layer = kiapi::board::types::BoardLayer::BlFCu as i32;
        *item = builders::pack_any(&shape, "kiapi.board.types.BoardGraphicShape");
        assert!(verify_readback(&old, &expected, &actual).is_err());
    }
}
