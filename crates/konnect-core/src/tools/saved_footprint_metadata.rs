//! Revision-bound restoration of missing library metadata on closed boards.
use super::{get_path, BoardAccess, ToolContext, ToolDef};
use crate::{mcp::protocol::CallToolResult, tool};
use anyhow::{bail, ensure, Context, Result};
use konnect_sexp::writer::{
    apply_edits, find_direct_child_blocks, read_consistent, write_atomic_if_unchanged,
};
use konnect_sexp::{parse_sexp, SexpEdit, SexpNode};
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

pub(super) fn definition() -> ToolDef {
    tool!("restore_saved_footprint_metadata",
 "Restore missing SMD mounting, heatsink-pad and solid zone-connection metadata from resolved libraries on an explicitly closed saved board. Front-side footprints only; refuses conflicting existing metadata or differing physical pads. Optional target_library_ids relinks reviewed artwork variants without rewriting footprint geometry; caller must verify their artwork first. Preserves geometry, nets, artwork, placement and other attributes. Dry run and exact revision required; no live-board fallback.",
 json!({"type":"object","additionalProperties":false,"properties":{
 "board":{"type":"string"},"references":{"type":"array","minItems":1,"items":{"type":"string","minLength":1}},
 "target_library_ids":{"type":"object","additionalProperties":{"type":"string"},"description":"Optional reference to Library:Footprint map for reviewed variants with identical physical lands"},
 "dry_run":{"type":"boolean","default":true},"expected_plan_revision":{"type":"string"}},"required":["board","references"]}),
 |args,ctx| async move {handle(args,ctx).await}).with_board_access(BoardAccess::ClosedBoardOnly)
}
fn unique<'a>(node: &'a SexpNode, tag: &str) -> Result<Option<&'a SexpNode>> {
    let found = node.find_all(tag);
    ensure!(found.len() <= 1, "duplicate {tag} clause");
    Ok(found.first().copied())
}
fn library_token_span(block: &str) -> Result<(usize, usize)> {
    let tail = block
        .strip_prefix("(footprint")
        .context("unsupported footprint header")?
        .trim_start();
    ensure!(tail.starts_with('"'), "quoted library identifier required");
    let start = block.len() - tail.len();
    let mut escaped = false;
    for (index, byte) in tail.bytes().enumerate().skip(1) {
        if escaped {
            escaped = false;
        } else if byte == b'\\' {
            escaped = true;
        } else if byte == b'"' {
            return Ok((start, start + index + 1));
        }
    }
    bail!("unterminated library identifier")
}
fn angle(node: &SexpNode) -> Result<f64> {
    let value = match node.get(3) {
        None => 0.0,
        Some(_) => node.get_f64(3).context("invalid rotation")?,
    };
    ensure!(value.is_finite(), "non-finite rotation");
    Ok(value)
}
fn physical_match(a: &SexpNode, b: &SexpNode, rotation: f64) -> Result<bool> {
    if a.get(2) != b.get(2) || a.get(3) != b.get(3) {
        return Ok(false);
    }
    for tag in [
        "size",
        "layers",
        "drill",
        "roundrect_rratio",
        "chamfer_ratio",
        "chamfer",
        "primitives",
    ] {
        if unique(a, tag)? != unique(b, tag)? {
            return Ok(false);
        }
    }
    let aa = unique(a, "at")?.context("pad lacks position")?;
    let bb = unique(b, "at")?.context("library pad lacks position")?;
    let delta = (angle(aa)? - angle(bb)? - rotation).rem_euclid(360.0);
    Ok(aa.get_f64(1) == bb.get_f64(1)
        && aa.get_f64(2) == bb.get_f64(2)
        && delta.min(360.0 - delta) < 1e-8)
}
fn repair_footprint(block: &str, library: &str) -> Result<(String, Vec<Value>)> {
    let fp = parse_sexp(block)?;
    let lib = parse_sexp(library)?;
    ensure!(
        fp.head() == Some("footprint") && lib.head() == Some("footprint"),
        "expected footprints"
    );
    ensure!(
        fp.find_str("layer") == Some("F.Cu"),
        "only front footprints supported"
    );
    let attr = unique(&fp, "attr")?;
    let la = unique(&lib, "attr")?.context("library mounting style missing")?;
    ensure!(
        la.children()
            .context("invalid attributes")?
            .iter()
            .any(|n| n.as_str() == Some("smd")),
        "library is not SMD"
    );
    let existing = attr.and_then(|n| n.children()).unwrap_or(&[]);
    ensure!(
        !existing.iter().any(|n| n.as_str() == Some("through_hole")),
        "conflicting mounting style"
    );
    let mut edits = Vec::new();
    let mut changes = Vec::new();
    if !existing.iter().any(|n| n.as_str() == Some("smd")) {
        if attr.is_some() {
            let (_, end) = find_direct_child_blocks(block, "footprint")
                .into_iter()
                .find(|(s, e)| {
                    parse_sexp(&block[*s..*e])
                        .map(|n| n.head() == Some("attr"))
                        .unwrap_or(false)
                })
                .context("attr span missing")?;
            edits.push(SexpEdit::insert(end - 1, " smd"));
        } else {
            edits.push(SexpEdit::insert(block.len() - 1, "\n\t(attr smd)\n"));
        }
        changes.push(json!({"field":"mounting_style","to":"smd"}));
    }
    let pads = fp.find_all("pad");
    let library_pads = lib.find_all("pad");
    let rotation = unique(&fp, "at")?.map(angle).transpose()?.unwrap_or(0.0);
    ensure!(
        pads.len() == library_pads.len(),
        "pad count differs from library"
    );
    // Pad angles in saved boards are absolute; positions and sizes are local.
    // Require a one-to-one physical-land mapping, including paste-only apertures.
    let mut used = BTreeSet::new();
    for pad in &pads {
        let mut matching = Vec::new();
        for (i, p) in library_pads.iter().enumerate() {
            if p.get(1) == pad.get(1) && physical_match(pad, p, rotation)? {
                matching.push(i)
            }
        }
        ensure!(
            matching.len() == 1 && used.insert(matching[0]),
            "ambiguous or differing physical land"
        );
    }
    for (start, end) in find_direct_child_blocks(block, "footprint") {
        let raw = &block[start..end];
        let pad = parse_sexp(raw)?;
        if pad.head() != Some("pad") {
            continue;
        }
        let number = pad
            .get(1)
            .and_then(SexpNode::as_str)
            .context("pad number missing")?;
        if number.is_empty() {
            continue;
        }
        let targets: Vec<_> = library_pads
            .iter()
            .filter(|p| p.get(1) == pad.get(1))
            .collect();
        ensure!(
            targets.len() == 1 && pads.iter().filter(|p| p.get(1) == pad.get(1)).count() == 1,
            "ambiguous numbered pad {number}"
        );
        let mut text = String::new();
        for (tag, allowed) in [("property", "pad_prop_heatsink"), ("zone_connect", "2")] {
            let desired = unique(targets[0], tag)?;
            let present = unique(&pad, tag)?;
            if let Some(desired) = desired {
                ensure!(
                    desired.children().map(|c| c.len()) == Some(2)
                        && desired.get(1).and_then(SexpNode::as_str) == Some(allowed),
                    "unsupported library {tag}"
                );
                if let Some(present) = present {
                    ensure!(
                        present == desired,
                        "conflicting existing pad {number} {tag}"
                    );
                } else {
                    text.push_str(&format!("\n\t\t({tag} {allowed})"));
                    changes.push(json!({"pad":number,"field":tag,"to":allowed}));
                }
            }
        }
        if !text.is_empty() {
            edits.push(SexpEdit::insert(end - 1, text));
        }
    }
    let result = apply_edits(block.to_owned(), edits);
    parse_sexp(&result)?;
    Ok((result, changes))
}
fn plan(
    content: &str,
    refs: &[String],
    targets: &BTreeMap<String, String>,
    mut resolve: impl FnMut(&str) -> Result<String>,
) -> Result<(String, Vec<Value>, Vec<(String, String)>)> {
    ensure!(
        parse_sexp(content)?.head() == Some("kicad_pcb"),
        "expected board"
    );
    ensure!(!refs.is_empty(), "references must not be empty");
    ensure!(
        targets.keys().all(|r| refs.contains(r)),
        "target reference not selected"
    );
    let mut seen = BTreeSet::new();
    let mut edits = Vec::new();
    let mut changes = Vec::new();
    let mut libraries = Vec::new();
    for reference in refs {
        ensure!(
            !reference.is_empty() && seen.insert(reference),
            "empty or duplicate reference"
        );
        let mut found = Vec::new();
        for (start, end) in find_direct_child_blocks(content, "kicad_pcb") {
            let f = parse_sexp(&content[start..end])?;
            if f.head() == Some("footprint")
                && f.find_all("property").iter().any(|p| {
                    p.get(1).and_then(SexpNode::as_str) == Some("Reference")
                        && p.get(2).and_then(SexpNode::as_str) == Some(reference)
                })
            {
                found.push((start, end, f));
            }
        }
        ensure!(
            found.len() == 1,
            "reference {reference} missing or ambiguous"
        );
        let (start, end, f) = &found[0];
        let id = f
            .get(1)
            .and_then(SexpNode::as_str)
            .context("library ID missing")?;
        let target = targets.get(reference).map(String::as_str).unwrap_or(id);
        let lib = resolve(target)?;
        let (mut new, mut detail) = repair_footprint(&content[*start..*end], &lib)?;
        if target != id {
            ensure!(
                target.split(':').count() == 2
                    && target.split(':').all(|s| !s.is_empty())
                    && target
                        .chars()
                        .all(|c| c.is_ascii_alphanumeric() || "_:.-".contains(c)),
                "unsupported target library identifier"
            );
            let (a, b) = library_token_span(&new)?;
            new = apply_edits(new, vec![SexpEdit::replace(a, b, format!("\"{target}\""))]);
            parse_sexp(&new)?;
            detail.push(json!({"field":"library_id","from":id,"to":target}));
        }
        libraries.push((target.to_owned(), lib));
        if !detail.is_empty() {
            edits.push(SexpEdit::replace(*start, *end, new));
            changes.push(json!({"reference":reference,"library_id":target,"changes":detail}));
        }
    }
    Ok((apply_edits(content.to_owned(), edits), changes, libraries))
}
async fn handle(args: &Value, ctx: &ToolContext) -> Result<CallToolResult> {
    let path = get_path(args, "board")?;
    if let Some(r) =
        super::pcb_board::refuse_if_board_open_in_kicad(ctx, &path, "footprint metadata repair")
            .await?
    {
        return Ok(r);
    }
    let refs = args["references"]
        .as_array()
        .context("references must be array")?
        .iter()
        .map(|v| {
            v.as_str()
                .map(str::to_owned)
                .context("references must be strings")
        })
        .collect::<Result<Vec<_>>>()?;
    let dry = match args.get("dry_run") {
        None => true,
        Some(v) => v.as_bool().context("dry_run must be boolean")?,
    };
    let content = read_consistent(&path)?;
    let resolve = |id: &str| -> Result<String> {
        let p = super::library::resolve_footprint_path(id, path.parent())
            .map_err(anyhow::Error::msg)?;
        read_consistent(&p).map_err(Into::into)
    };
    let targets: BTreeMap<String, String> = args
        .get("target_library_ids")
        .map(|v| serde_json::from_value(v.clone()))
        .transpose()?
        .unwrap_or_default();
    let (updated, changes, libraries) = plan(&content, &refs, &targets, resolve)?;
    let revision = format!(
        "{:x}",
        Sha256::digest(serde_json::to_vec(&(
            path.to_string_lossy(),
            &content,
            &refs,
            &libraries
        ))?)
    );
    if !dry && args["expected_plan_revision"].as_str() != Some(&revision) {
        bail!("stale or missing expected_plan_revision")
    }
    let status = if updated == content {
        "noop"
    } else if dry {
        "ready"
    } else {
        if let Some(r) =
            super::pcb_board::refuse_if_board_open_in_kicad(ctx, &path, "footprint metadata repair")
                .await?
        {
            return Ok(r);
        }
        for (id, source) in &libraries {
            ensure!(resolve(id)? == *source, "library changed during plan")
        }
        write_atomic_if_unchanged(&path, &content, &updated)?;
        ensure!(
            read_consistent(&path)? == updated,
            "readback mismatch; inspect before retrying"
        );
        "applied"
    };
    Ok(CallToolResult::json(
        &json!({"status":status,"plan_revision":revision,"changes":changes,"geometry_preserved":true,"board_access":"closed_board_only"}),
    ))
}
#[cfg(test)]
mod tests {
    use super::*;
    const FP: &str = r#"(footprint "L:X" (layer "F.Cu") (at 12 13 180) (property "Reference" "U1") (attr exclude_from_bom)
 (fp_poly (pts (xy 1 2) (xy 3 4) (xy 5 6)))
 (pad "9" smd rect (at 0 0 180) (size 2 3) (layers "F.Cu" "F.Mask") (net 1 "GND")))"#;
    const LIB: &str = r#"(footprint "X" (layer "F.Cu") (attr smd)
 (pad "9" smd rect (at 0 0) (size 2 3) (layers "F.Cu" "F.Mask") (property pad_prop_heatsink) (zone_connect 2)))"#;
    #[test]
    fn preserves_geometry_and_only_adds_missing_metadata() {
        let (out, changes) = repair_footprint(FP, LIB).unwrap();
        assert_eq!(changes.len(), 3);
        assert_eq!(
            parse_sexp(&out).unwrap().find("fp_poly"),
            parse_sexp(FP).unwrap().find("fp_poly")
        );
        assert!(out.contains("(net 1 \"GND\")"));
        assert!(out.contains("(attr exclude_from_bom smd)"));
        assert_eq!(repair_footprint(&out, LIB).unwrap().0, out);
    }
    #[test]
    fn refuses_conflicts_and_physical_mismatch() {
        for bad in [
            FP.replace("(size 2 3)", "(size 2 4)"),
            FP.replace("(at 0 0 180)", "(at 0 0 90)"),
            FP.replace("(net 1", "(zone_connect 1) (net 1"),
            FP.replace("(layer \"F.Cu\")", "(layer \"B.Cu\")"),
            FP.replace("(attr exclude_from_bom)", "(attr through_hole)"),
        ] {
            assert!(repair_footprint(&bad, LIB).is_err());
        }
    }
    #[test]
    fn plans_all_or_nothing_and_rejects_duplicate_targets() {
        let board = format!("(kicad_pcb {FP} (segment (width 0.2)))");
        let resolve = |_: &str| Ok(LIB.to_owned());
        assert!(plan(
            &board,
            &["U1".into(), "missing".into()],
            &BTreeMap::new(),
            resolve
        )
        .is_err());
        assert!(plan(
            &board,
            &["U1".into(), "U1".into()],
            &BTreeMap::new(),
            resolve
        )
        .is_err());
        let (out, changes, _) = plan(&board, &["U1".into()], &BTreeMap::new(), resolve).unwrap();
        assert_eq!(changes.len(), 1);
        assert!(out.contains("(segment (width 0.2))"));
    }
    #[test]
    fn relinks_reviewed_variant_without_rewriting_other_content() {
        let complete = repair_footprint(FP, LIB).unwrap().0;
        let board = format!("(kicad_pcb {complete})");
        let targets = BTreeMap::from([("U1".into(), "Project:X_MovedMarker".into())]);
        let (out, changes, _) =
            plan(&board, &["U1".into()], &targets, |_| Ok(LIB.to_owned())).unwrap();
        assert_eq!(
            out,
            board.replacen("\"L:X\"", "\"Project:X_MovedMarker\"", 1)
        );
        assert_eq!(changes[0]["changes"].as_array().unwrap().len(), 1);
        assert!(plan(&board, &["U2".into()], &targets, |_| Ok(LIB.to_owned())).is_err());
        assert!(plan(&board, &["U1".into()], &targets, |_| Ok(
            LIB.replace("(size 2 3)", "(size 4 5)")
        ))
        .is_err());
    }
}
