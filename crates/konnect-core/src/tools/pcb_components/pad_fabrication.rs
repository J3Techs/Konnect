//! `set_pad_fabrication_properties`: set placed pads' fabrication property
//! (`(property pad_prop_…)`) in the saved board, explicitly or copied from
//! each footprint's library.
//!
//! KiCad 10's typed API has no field for a pad's fabrication property, so
//! neither a live edit nor `update_footprints_from_library` can carry one (see
//! `footprint_fabrication`). This tool edits the saved file of a board KiCad
//! does not hold, and changes nothing but each targeted pad's one
//! `(property …)` clause: it is replaced, inserted before the pad's
//! `(layers …)` (where KiCad writes it), or cut. Every other byte — the pad's
//! geometry, net and UUID, the footprint's fields, artwork and placement —
//! stays as it was, which the plan and the readback both check.
//!
//! With `references`, each pad takes the property its library pad of the same
//! number carries (none included), which is what KiCad's Update Footprint from
//! Library would give it. This is the lossless closed-board form of that
//! update for the one library-owned clause the typed refresh cannot carry.

use super::reference_field::{footprints_by_reference, head_of};
use super::*;
use crate::tools::closed_board::only_spans_changed;
use crate::tools::footprint_fabrication::pad_fabrication_property;
use serde::Serialize;
use serde_json::Value;
use std::collections::BTreeSet;

pub(crate) const OPERATION: &str = "set_pad_fabrication_properties";

/// A library footprint's pads: number and fabrication property, in order.
type LibraryPads = Vec<(String, Option<String>)>;

/// The fabrication properties KiCad 10 reads for a pad, plus `none`.
pub(crate) const PROPERTIES: &[&str] = &[
    "pad_prop_bga",
    "pad_prop_fiducial_glob",
    "pad_prop_fiducial_loc",
    "pad_prop_testpoint",
    "pad_prop_heatsink",
    "pad_prop_castellated",
    "pad_prop_mechanical",
    "pad_prop_pressfit",
];

pub(crate) fn tool() -> ToolDef {
    let mut values: Vec<&str> = PROPERTIES.to_vec();
    values.push("none");
    tool!(
        "set_pad_fabrication_properties",
        "Set the fabrication property of placed pads (pad_prop_fiducial_glob, pad_prop_heatsink, \
         pad_prop_bga, …, or none) in the saved board, which KiCad 10's typed API and \
         update_footprints_from_library cannot do. 'references' copies each pad's property from \
         its footprint's library pad of the same number — the closed-board form of Update \
         Footprint from Library for that clause; 'pads' sets named pads explicitly. Requires \
         the exact board closed in KiCad (no sibling lock), even for a dry run. Dry run by \
         default: it reports each pad's before and after and a revision; apply repeats that \
         revision and makes one atomic write that changes only each targeted pad's \
         (property …) clause, checked byte for byte and read back. A second run settles to \
         noop.",
        json!({
            "type": "object",
            "properties": {
                "board": { "type": "string", "description": "Saved .kicad_pcb, closed in KiCad" },
                "references": {
                    "type": "array",
                    "items": { "type": "string" },
                    "description": "Footprints whose pads take their library pads' fabrication properties, matched by pad number."
                },
                "pads": {
                    "type": "array",
                    "description": "Explicit settings. Every pad of the footprint with that number is set.",
                    "items": {
                        "type": "object",
                        "properties": {
                            "reference": { "type": "string" },
                            "pad": { "type": "string", "description": "Pad number, \"\" for an unnumbered pad" },
                            "property": { "type": "string", "enum": values }
                        },
                        "required": ["reference", "pad", "property"],
                        "additionalProperties": false
                    }
                },
                "dry_run": { "type": "boolean", "default": true },
                "expected_revision": {
                    "type": "string",
                    "description": "Required for apply: the revision the reviewed dry run returned."
                }
            },
            "required": ["board"]
        }),
        |args, ctx| async move { handle_set_pad_fabrication_properties(args, ctx).await }
    )
    .with_board_access(crate::tools::BoardAccess::ClosedBoardOnly)
}

/// One request, normalized.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize)]
pub(crate) struct Request {
    /// Footprints that copy from their library.
    pub from_library: BTreeSet<String>,
    /// `(reference, pad number) -> property`, `None` to clear.
    pub explicit: BTreeMap<(String, String), Option<String>>,
}

fn parse_request(args: &Value) -> Result<Request, (String, String)> {
    let mut request = Request::default();
    if let Some(references) = args.get("references").filter(|v| !v.is_null()) {
        let references = references
            .as_array()
            .ok_or_else(|| ("references".to_string(), "must be an array".to_string()))?;
        for (index, reference) in references.iter().enumerate() {
            let reference = reference
                .as_str()
                .filter(|r| !r.is_empty())
                .ok_or_else(|| {
                    (
                        format!("references[{index}]"),
                        "must be a non-empty string".to_string(),
                    )
                })?;
            if !request.from_library.insert(reference.to_string()) {
                return Err((
                    format!("references[{index}]"),
                    format!("'{reference}' is repeated"),
                ));
            }
        }
    }
    if let Some(pads) = args.get("pads").filter(|v| !v.is_null()) {
        let pads = pads
            .as_array()
            .ok_or_else(|| ("pads".to_string(), "must be an array".to_string()))?;
        for (index, entry) in pads.iter().enumerate() {
            let field = |name: &str| format!("pads[{index}].{name}");
            let text = |name: &str| {
                entry
                    .get(name)
                    .and_then(Value::as_str)
                    .ok_or_else(|| (field(name), "must be a string".to_string()))
            };
            let reference = text("reference")?;
            if reference.is_empty() {
                return Err((field("reference"), "must not be empty".to_string()));
            }
            let pad = text("pad")?;
            let property = text("property")?;
            let property = match property {
                "none" => None,
                known if PROPERTIES.contains(&known) => Some(known.to_string()),
                other => {
                    return Err((
                        field("property"),
                        format!("'{other}' is not a KiCad pad fabrication property"),
                    ))
                }
            };
            if request.from_library.contains(reference) {
                return Err((
                    field("reference"),
                    format!("'{reference}' is also in references; choose one source per footprint"),
                ));
            }
            let key = (reference.to_string(), pad.to_string());
            if request.explicit.insert(key, property).is_some() {
                return Err((
                    field("pad"),
                    format!("pad '{pad}' of '{reference}' is repeated"),
                ));
            }
        }
    }
    if request.from_library.is_empty() && request.explicit.is_empty() {
        return Err((
            "references".to_string(),
            "give references, pads, or both".to_string(),
        ));
    }
    Ok(request)
}

/// One pad of a saved footprint.
#[derive(Debug, Clone)]
struct SavedPad {
    number: String,
    uuid: String,
    span: (usize, usize),
    property: Option<String>,
    property_span: Option<(usize, usize)>,
}

fn saved_pads(source: &str, footprint: (usize, usize)) -> Result<Vec<SavedPad>, String> {
    let block = &source[footprint.0..footprint.1];
    let mut pads = Vec::new();
    for (s, e) in find_direct_child_blocks(block, "footprint") {
        let text = &block[s..e];
        if head_of(text) != "pad" {
            continue;
        }
        let tokens = sexp_block_tokens(text);
        let number = tokens
            .get(1)
            .map(|&(a, b)| sexp_token_value(&text[a..b]))
            .ok_or("a pad has no number")?;
        let node = konnect_sexp::parse_sexp(text).map_err(|error| error.to_string())?;
        let property_spans: Vec<_> = find_direct_child_blocks(text, "pad")
            .into_iter()
            .filter(|&(a, b)| head_of(&text[a..b]) == "property")
            .collect();
        if property_spans.len() > 1 {
            return Err(format!(
                "pad '{number}' has more than one (property …) clause"
            ));
        }
        pads.push(SavedPad {
            number,
            uuid: node.find_str("uuid").unwrap_or_default().to_string(),
            span: (footprint.0 + s, footprint.0 + e),
            property: pad_fabrication_property(&node).map(str::to_string),
            property_span: property_spans
                .first()
                .map(|&(a, b)| (footprint.0 + s + a, footprint.0 + s + b)),
        });
    }
    Ok(pads)
}

fn footprint_id(source: &str, footprint: (usize, usize)) -> String {
    let block = &source[footprint.0..footprint.1];
    sexp_block_tokens(block)
        .get(1)
        .map(|&(a, b)| sexp_token_value(&block[a..b]))
        .unwrap_or_default()
}

/// A library footprint read for its pads' properties.
#[derive(Debug, Clone, Serialize)]
pub(crate) struct LibrarySource {
    pub footprint_id: String,
    pub path: String,
    pub sha256: String,
}

/// One pad the plan changes.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub(crate) struct PadChange {
    pub reference: String,
    pub footprint_id: String,
    pub pad: String,
    pub pad_uuid: String,
    pub before: String,
    pub after: String,
    pub origin: &'static str,
}

#[derive(Debug, Clone)]
pub(crate) struct Prepared {
    pub candidate: String,
    pub edits: Vec<(usize, usize, usize)>,
    pub changes: Vec<PadChange>,
    pub unchanged: usize,
    pub libraries: Vec<LibrarySource>,
    pub notes: Vec<String>,
    /// Per touched footprint, the property every pad must read back with.
    expected: Vec<(String, Vec<Option<String>>)>,
}

fn shown(property: &Option<String>) -> String {
    property.clone().unwrap_or_else(|| "none".to_string())
}

/// Each board pad's wanted property from its library footprint: the library
/// pads of the same number, all agreeing or matched in order.
fn library_targets(
    reference: &str,
    pads: &[SavedPad],
    library: &[(String, Option<String>)],
    notes: &mut Vec<String>,
) -> Result<Vec<Option<Option<String>>>, String> {
    let mut wanted = vec![None; pads.len()];
    let numbers: BTreeSet<&str> = pads.iter().map(|pad| pad.number.as_str()).collect();
    for number in &numbers {
        let board: Vec<usize> = (0..pads.len())
            .filter(|&i| pads[i].number == *number)
            .collect();
        let lib: Vec<&Option<String>> = library
            .iter()
            .filter(|(n, _)| n == number)
            .map(|(_, property)| property)
            .collect();
        if lib.is_empty() {
            notes.push(format!(
                "{reference} pad '{number}' has no library pad of that number; left as it is"
            ));
            continue;
        }
        if lib.iter().all(|property| *property == lib[0]) {
            for &i in &board {
                wanted[i] = Some(lib[0].clone());
            }
        } else if lib.len() == board.len() {
            for (&i, property) in board.iter().zip(&lib) {
                wanted[i] = Some((*property).clone());
            }
        } else {
            return Err(format!(
                "{reference}: the library's {} pads numbered '{number}' carry different \
                 properties and the board has {} such pads, so they cannot be matched",
                lib.len(),
                board.len()
            ));
        }
    }
    for (number, _) in library {
        if !numbers.contains(number.as_str()) {
            notes.push(format!(
                "{reference}: library pad '{number}' has no placed pad of that number"
            ));
        }
    }
    notes.dedup();
    Ok(wanted)
}

fn read_library(footprint_id: &str, board: &Path) -> Result<(LibraryPads, LibrarySource), String> {
    let path = super::resolve_footprint_file(footprint_id, board)
        .map_err(|error| format!("library footprint '{footprint_id}' did not resolve: {error}"))?;
    let text = std::fs::read_to_string(&path)
        .map_err(|error| format!("failed to read {}: {error}", path.display()))?;
    let root = konnect_sexp::parse_sexp(&text)
        .map_err(|error| format!("{} is not a footprint: {error}", path.display()))?;
    if root.head() != Some("footprint") && root.head() != Some("module") {
        return Err(format!("{} is not a footprint", path.display()));
    }
    let pads = root
        .find_all("pad")
        .into_iter()
        .map(|pad| {
            (
                pad.get(1)
                    .and_then(konnect_sexp::SexpNode::as_str)
                    .unwrap_or_default()
                    .to_string(),
                pad_fabrication_property(pad).map(str::to_string),
            )
        })
        .collect();
    Ok((
        pads,
        LibrarySource {
            footprint_id: footprint_id.to_string(),
            path: path.display().to_string(),
            sha256: crate::tools::closed_board::sha256_hex(text.as_bytes()),
        },
    ))
}

/// Plan the request against the saved `source` of `board`.
pub(crate) fn prepare(
    source: &str,
    board: &Path,
    request: &Request,
) -> Result<Prepared, Vec<String>> {
    check_single_board_form(source).map_err(|e| vec![e])?;
    let index = footprints_by_reference(source).map_err(|e| vec![e])?;
    let references: BTreeSet<&String> = request
        .from_library
        .iter()
        .chain(request.explicit.keys().map(|(reference, _)| reference))
        .collect();
    let mut reasons = Vec::new();
    let mut notes = Vec::new();
    let mut libraries: BTreeMap<String, (LibraryPads, LibrarySource)> = BTreeMap::new();
    let mut changes = Vec::new();
    let mut sexp_edits = Vec::new();
    let mut expected = Vec::new();
    let mut unchanged = 0;
    for reference in references {
        let footprint = match index.get(reference).map(Vec::as_slice).unwrap_or(&[]) {
            [span] => *span,
            [] => {
                reasons.push(format!(
                    "footprint '{reference}' was not found in the saved board"
                ));
                continue;
            }
            many => {
                reasons.push(format!(
                    "footprint reference '{reference}' appears {} times",
                    many.len()
                ));
                continue;
            }
        };
        let id = footprint_id(source, footprint);
        let pads = match saved_pads(source, footprint) {
            Ok(pads) => pads,
            Err(reason) => {
                reasons.push(format!("{reference}: {reason}"));
                continue;
            }
        };
        let mut wanted: Vec<Option<Option<String>>> = vec![None; pads.len()];
        let mut origin = vec![""; pads.len()];
        if request.from_library.contains(reference) {
            if !libraries.contains_key(&id) {
                match read_library(&id, board) {
                    Ok(read) => {
                        libraries.insert(id.clone(), read);
                    }
                    Err(reason) => {
                        reasons.push(format!("{reference}: {reason}"));
                        continue;
                    }
                }
            }
            match library_targets(reference, &pads, &libraries[&id].0, &mut notes) {
                Ok(targets) => {
                    for (i, target) in targets.into_iter().enumerate() {
                        if target.is_some() {
                            wanted[i] = target;
                            origin[i] = "library";
                        }
                    }
                }
                Err(reason) => {
                    reasons.push(reason);
                    continue;
                }
            }
        }
        for ((r, number), property) in &request.explicit {
            if r != reference {
                continue;
            }
            let matching: Vec<usize> = (0..pads.len())
                .filter(|&i| pads[i].number == *number)
                .collect();
            if matching.is_empty() {
                reasons.push(format!("{reference} has no pad numbered '{number}'"));
            }
            for i in matching {
                wanted[i] = Some(property.clone());
                origin[i] = "request";
            }
        }
        let mut readback = Vec::with_capacity(pads.len());
        for (i, pad) in pads.iter().enumerate() {
            let Some(want) = wanted[i].clone() else {
                readback.push(pad.property.clone());
                continue;
            };
            readback.push(want.clone());
            if want == pad.property {
                unchanged += 1;
                continue;
            }
            match (&want, pad.property_span) {
                (Some(property), Some((s, e))) => {
                    sexp_edits.push(SexpEdit::replace(s, e, format!("(property {property})")))
                }
                (None, Some((s, e))) => {
                    let cut_from = source[..s].trim_end_matches([' ', '\t', '\r', '\n']).len();
                    sexp_edits.push(SexpEdit::delete(cut_from, e));
                }
                (Some(property), None) => {
                    let text = &source[pad.span.0..pad.span.1];
                    let layers = find_direct_child_blocks(text, "pad")
                        .into_iter()
                        .find(|&(a, b)| head_of(&text[a..b]) == "layers");
                    match layers {
                        Some((a, _)) => {
                            let gap_from =
                                text[..a].trim_end_matches([' ', '\t', '\r', '\n']).len();
                            let gap = &text[gap_from..a];
                            let gap = if gap.is_empty() { " " } else { gap };
                            sexp_edits.push(SexpEdit::insert(
                                pad.span.0 + a,
                                format!("(property {property}){gap}"),
                            ));
                        }
                        None => sexp_edits.push(SexpEdit::insert(
                            pad.span.1 - 1,
                            format!(" (property {property})"),
                        )),
                    }
                }
                (None, None) => unreachable!("an unchanged pad was skipped"),
            }
            changes.push(PadChange {
                reference: reference.clone(),
                footprint_id: id.clone(),
                pad: pad.number.clone(),
                pad_uuid: pad.uuid.clone(),
                before: shown(&pad.property),
                after: shown(&want),
                origin: origin[i],
            });
        }
        expected.push((reference.clone(), readback));
    }
    if !reasons.is_empty() {
        return Err(reasons);
    }
    let edits: Vec<_> = sexp_edits
        .iter()
        .map(|edit| (edit.start, edit.end, edit.replacement.len()))
        .collect();
    let candidate = apply_edits(source.to_string(), sexp_edits);
    let prepared = Prepared {
        candidate,
        edits,
        changes,
        unchanged,
        libraries: libraries.into_values().map(|(_, source)| source).collect(),
        notes,
        expected,
    };
    verify(source, &prepared.candidate, &prepared).map_err(|e| vec![e])?;
    Ok(prepared)
}

/// The candidate (or persisted) board differs only inside the planned spans,
/// and every pad of each touched footprint reads back with its planned
/// property.
pub(crate) fn verify(
    original: &str,
    candidate: &str,
    prepared: &Prepared,
) -> Result<Value, String> {
    if !only_spans_changed(original, candidate, &prepared.edits) {
        return Err("bytes outside the planned pad property spans changed".to_string());
    }
    check_single_board_form(candidate)?;
    let index = footprints_by_reference(candidate)?;
    let mut pads_checked = 0;
    for (reference, want) in &prepared.expected {
        let [span] = index.get(reference).map(Vec::as_slice).unwrap_or(&[]) else {
            return Err(format!("'{reference}' is no longer one footprint"));
        };
        let have: Vec<Option<String>> = saved_pads(candidate, *span)?
            .into_iter()
            .map(|pad| pad.property)
            .collect();
        if &have != want {
            return Err(format!(
                "the pads of '{reference}' do not read back as planned"
            ));
        }
        pads_checked += have.len();
    }
    konnect_sexp::parse_sexp(candidate).map_err(|error| error.to_string())?;
    Ok(json!({
        "footprints_checked": prepared.expected.len(),
        "pads_match_plan": pads_checked,
        "bytes_outside_planned_spans_unchanged": true,
    }))
}

async fn handle_set_pad_fabrication_properties(
    args: &Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let board = get_path(args, "board")?;
    let request = match parse_request(args) {
        Ok(request) => request,
        Err((field, reason)) => return Ok(crate::tools::invalid_arg(&field, &reason)),
    };
    let dry_run = args["dry_run"].as_bool().unwrap_or(true);
    let expected_revision = args["expected_revision"].as_str();
    if !dry_run && expected_revision.is_none() {
        return Ok(crate::tools::invalid_arg(
            "expected_revision",
            "required when dry_run is false; apply the revision a dry run returned",
        ));
    }
    let closure = match crate::tools::closed_board::prove_closed(ctx, &board, OPERATION).await? {
        Ok(closure) => closure,
        Err(refusal) => return Ok(refusal),
    };
    let original = read_consistent(&board)?;
    let prepared = match prepare(&original, &board, &request) {
        Ok(prepared) => prepared,
        Err(reasons) => {
            return Ok(CallToolResult::error_kind(
                crate::mcp::error::ToolErrorKind::PlanBlocked {
                    operation: OPERATION.to_string(),
                    reasons: reasons.clone(),
                },
                format!(
                    "No pad fabrication property changed: {}. The board file was not modified.",
                    reasons.join("; ")
                ),
            ))
        }
    };
    let revision = crate::tools::closed_board::revision(
        OPERATION,
        &original,
        &json!({
            "request": request,
            "libraries": prepared.libraries.iter().map(|l| json!([l.footprint_id, l.sha256])).collect::<Vec<_>>(),
        }),
    );
    let changed = !prepared.edits.is_empty();
    let mut body = json!({
        "status": match (changed, dry_run) {
            (false, _) => "noop",
            (true, true) => "ready",
            (true, false) => "applied",
        },
        "source": "saved",
        "revision": revision,
        "changed_count": prepared.changes.len(),
        "unchanged_count": prepared.unchanged,
        "changes": prepared.changes,
        "libraries": prepared.libraries,
        "notes": prepared.notes,
        "board_closure": closure,
        "other_clauses": "unchanged byte-for-byte; only each changed pad's (property …) clause is written",
    });
    if dry_run {
        return Ok(CallToolResult::json(&body));
    }
    if expected_revision != Some(revision.as_str()) {
        return Ok(crate::tools::closed_board::stale_revision(
            &board, OPERATION,
        ));
    }
    if changed {
        if let Err(refusal) =
            crate::tools::closed_board::prove_closed(ctx, &board, OPERATION).await?
        {
            return Ok(refusal);
        }
        match crate::tools::closed_board::write_and_read_back(
            &board,
            OPERATION,
            &original,
            &prepared.candidate,
            |text| verify(&original, text, &prepared),
        ) {
            Ok(write) => body["write"] = write,
            Err(refusal) => return Ok(refusal),
        }
    }
    Ok(CallToolResult::json(&body))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tools::pcb_board::board_mock::{ctx_talking_to, spawn_kicad_holding_board};

    const FIDUCIAL_LIB: &str = r#"(footprint "Fiducial_1mm_Mask2mm"
	(version 20260206)
	(layer "F.Cu")
	(attr smd exclude_from_bom)
	(pad "" smd circle
		(at 0 0)
		(size 1 1)
		(property pad_prop_fiducial_glob)
		(layers "F.Cu" "F.Mask")
		(solder_mask_margin 0.5)
		(clearance 0.6)
	)
	(embedded_fonts no)
)
"#;

    /// Two placed fiducials (one hidden-Reference, one on the back), a QFN
    /// with a heatsink pad and duplicate numbers, and a part with no library.
    const BOARD: &str = r#"(kicad_pcb (version 20260206)
	(footprint "Fid:Fiducial_1mm_Mask2mm"
		(layer "F.Cu")
		(uuid "11111111-1111-4111-8111-111111111111")
		(at 371 296.5)
		(property "Reference" "FID1"
			(at 0 -1.11 0)
			(layer "F.SilkS")
			(hide yes)
			(uuid "21111111-1111-4111-8111-111111111111")
			(effects (font (size 0.8 0.8) (thickness 0.12)))
		)
		(attr smd board_only exclude_from_bom)
		(pad "" smd circle
			(at 0 0)
			(size 1 1)
			(layers "F.Cu" "F.Mask")
			(solder_mask_margin 0.5)
			(clearance 0.6)
			(uuid "31111111-1111-4111-8111-111111111111")
		)
		(embedded_fonts no)
	)
	(footprint "Fid:Fiducial_1mm_Mask2mm" (layer "B.Cu") (uuid "12222222-2222-4222-8222-222222222222") (at 10 20 90)
		(property "Reference" "FID2" (at 0 -1.11 90) (layer "B.SilkS") (uuid "22222222-2222-4222-8222-222222222222") (effects (font (size 0.8 0.8) (thickness 0.12)) (justify mirror)))
		(pad "" smd circle (at 0 0 90) (size 1 1)(layers "B.Cu" "B.Mask") (uuid "32222222-2222-4222-8222-222222222222")))
	(footprint "Lib:QFN" (layer "F.Cu") (uuid "13333333-3333-4333-8333-333333333333") (at 5 5)
		(property "Reference" "U5901" (at 0 0 0) (layer "F.SilkS") (uuid "23333333-3333-4333-8333-333333333333") (effects (font (size 1 1) (thickness 0.15))))
		(pad "1" smd rect (at -1 0) (size 0.3 0.8) (layers "F.Cu" "F.Mask") (net 1 "A") (uuid "33333333-3333-4333-8333-333333333331"))
		(pad "25" smd rect (at 0 0) (size 2.5 2.5) (property pad_prop_heatsink) (layers "F.Cu" "F.Mask") (net 2 "GND") (uuid "33333333-3333-4333-8333-333333333332"))
		(pad "25" thru_hole circle (at 0.5 0.5) (size 0.6 0.6) (drill 0.3) (property pad_prop_heatsink) (layers "*.Cu") (net 2 "GND") (uuid "33333333-3333-4333-8333-333333333333")))
	(footprint "Missing:Part" (layer "F.Cu") (uuid "14444444-4444-4444-8444-444444444444") (at 1 1)
		(property "Reference" "X1" (at 0 0 0) (layer "F.SilkS") (uuid "24444444-4444-4444-8444-444444444444") (effects (font (size 1 1) (thickness 0.15))))
		(pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (uuid "34444444-4444-4444-8444-444444444444")))
)
"#;

    fn project() -> (tempfile::TempDir, std::path::PathBuf) {
        let tmp = tempfile::tempdir().unwrap();
        let lib = tmp.path().join("Fid.pretty");
        std::fs::create_dir(&lib).unwrap();
        std::fs::write(lib.join("Fiducial_1mm_Mask2mm.kicad_mod"), FIDUCIAL_LIB).unwrap();
        std::fs::write(
            tmp.path().join("fp-lib-table"),
            "(fp_lib_table\n  (version 7)\n  (lib (name \"Fid\")(type \"KiCad\")(uri \"${KIPRJMOD}/Fid.pretty\")(options \"\")(descr \"\"))\n)\n",
        )
        .unwrap();
        let board = tmp.path().join("fid.kicad_pcb");
        std::fs::write(&board, BOARD).unwrap();
        (tmp, board)
    }

    fn library(references: &[&str]) -> Request {
        Request {
            from_library: references.iter().map(|r| r.to_string()).collect(),
            ..Request::default()
        }
    }

    #[test]
    fn library_copy_inserts_the_property_where_kicad_writes_it_and_nothing_else() {
        let (_tmp, board) = project();
        let prepared = prepare(BOARD, &board, &library(&["FID1", "FID2"])).unwrap();
        assert_eq!(prepared.changes.len(), 2);
        assert_eq!(prepared.changes[0].before, "none");
        assert_eq!(prepared.changes[0].after, "pad_prop_fiducial_glob");
        assert_eq!(prepared.changes[0].origin, "library");
        assert_eq!(
            prepared.changes[0].pad_uuid,
            "31111111-1111-4111-8111-111111111111"
        );
        assert!(prepared.candidate.contains(
            "(size 1 1)\n\t\t\t(property pad_prop_fiducial_glob)\n\t\t\t(layers \"F.Cu\" \"F.Mask\")"
        ));
        assert!(prepared
            .candidate
            .contains("(size 1 1)(property pad_prop_fiducial_glob) (layers \"B.Cu\""));
        // The hidden Reference, attributes and pad UUIDs are the original bytes.
        assert!(prepared
            .candidate
            .contains("(layer \"F.SilkS\")\n\t\t\t(hide yes)"));
        assert!(prepared
            .candidate
            .contains("(attr smd board_only exclude_from_bom)"));
        assert_eq!(prepared.libraries.len(), 1);
        assert!(prepared.notes.is_empty(), "{:?}", prepared.notes);

        let again = prepare(&prepared.candidate, &board, &library(&["FID1", "FID2"])).unwrap();
        assert!(again.edits.is_empty());
        assert_eq!(again.unchanged, 2);
    }

    #[test]
    fn explicit_settings_replace_clear_and_cover_every_pad_of_a_number() {
        let (_tmp, board) = project();
        let mut request = Request::default();
        request
            .explicit
            .insert(("U5901".to_string(), "25".to_string()), None);
        request.explicit.insert(
            ("U5901".to_string(), "1".to_string()),
            Some("pad_prop_testpoint".to_string()),
        );
        let prepared = prepare(BOARD, &board, &request).unwrap();
        assert_eq!(prepared.changes.len(), 3);
        assert!(!prepared.candidate.contains("pad_prop_heatsink"));
        assert!(prepared
            .candidate
            .contains("(size 0.3 0.8) (property pad_prop_testpoint) (layers"));
        assert!(prepared.candidate.contains("(size 2.5 2.5) (layers"));
        assert!(prepared.candidate.contains("(drill 0.3) (layers \"*.Cu\")"));

        let mut swap = Request::default();
        swap.explicit.insert(
            ("U5901".to_string(), "25".to_string()),
            Some("pad_prop_bga".to_string()),
        );
        let swapped = prepare(BOARD, &board, &swap).unwrap();
        assert_eq!(
            swapped.candidate.matches("(property pad_prop_bga)").count(),
            2
        );
    }

    #[test]
    fn unresolvable_and_ambiguous_requests_refuse_the_whole_plan() {
        let (_tmp, board) = project();
        let reasons = prepare(BOARD, &board, &library(&["X1", "FID1", "NOPE"])).unwrap_err();
        assert_eq!(reasons.len(), 2, "{reasons:?}");
        assert!(reasons
            .iter()
            .any(|r| r.contains("X1") && r.contains("did not resolve")));
        assert!(reasons.iter().any(|r| r.contains("'NOPE' was not found")));

        let mut request = Request::default();
        request
            .explicit
            .insert(("U5901".to_string(), "99".to_string()), None);
        let reasons = prepare(BOARD, &board, &request).unwrap_err();
        assert!(reasons[0].contains("no pad numbered '99'"));
    }

    #[test]
    fn library_pads_are_matched_by_number_then_by_order() {
        let pad = |number: &str| SavedPad {
            number: number.to_string(),
            uuid: String::new(),
            span: (0, 0),
            property: None,
            property_span: None,
        };
        let board = [pad("1"), pad("2"), pad("2"), pad("9")];
        let library = [
            ("1".to_string(), None),
            ("2".to_string(), Some("pad_prop_heatsink".to_string())),
            ("2".to_string(), None),
            ("3".to_string(), None),
        ];
        let mut notes = Vec::new();
        let wanted = library_targets("U1", &board, &library, &mut notes).unwrap();
        assert_eq!(
            wanted,
            [
                Some(None),
                Some(Some("pad_prop_heatsink".to_string())),
                Some(None),
                None
            ]
        );
        assert_eq!(notes.len(), 2, "{notes:?}");
        let short = [pad("2")];
        assert!(library_targets("U1", &short, &library, &mut notes).is_err());
    }

    fn body(result: &CallToolResult) -> Value {
        match &result.content[0] {
            crate::mcp::protocol::ToolContent::Text { text } => {
                serde_json::from_str(text).unwrap_or_else(|_| json!({ "text": text }))
            }
            other => panic!("expected text, got {other:?}"),
        }
    }

    async fn call(ctx: ToolContext, args: Value) -> (bool, Value) {
        let def = tool();
        assert!(def.input_validator.is_valid(&args), "schema refused {args}");
        let result = (def.handler)(&args, std::sync::Arc::new(ctx))
            .await
            .unwrap();
        (result.is_error, body(&result))
    }

    #[tokio::test]
    async fn dry_run_apply_and_rerun_settle_to_noop() {
        let (_tmp, board) = project();
        let args = json!({ "board": board, "references": ["FID1", "FID2"] });
        let (error, plan) = call(ctx_talking_to(String::new()), args.clone()).await;
        assert!(!error, "{plan}");
        assert_eq!(plan["status"], "ready");
        assert_eq!(plan["changed_count"], 2);
        assert_eq!(plan["changes"][0]["after"], "pad_prop_fiducial_glob");
        assert_eq!(std::fs::read_to_string(&board).unwrap(), BOARD);

        let mut apply = args.clone();
        apply["dry_run"] = json!(false);
        let (error, missing) = call(ctx_talking_to(String::new()), apply.clone()).await;
        assert!(error);
        assert_eq!(missing["error"]["kind"], "invalid_argument");

        apply["expected_revision"] = json!("0".repeat(64));
        let (error, stale) = call(ctx_talking_to(String::new()), apply.clone()).await;
        assert!(error);
        assert_eq!(stale["error"]["reason"], "stale_revision");
        assert_eq!(std::fs::read_to_string(&board).unwrap(), BOARD);

        apply["expected_revision"] = plan["revision"].clone();
        let (error, applied) = call(ctx_talking_to(String::new()), apply).await;
        assert!(!error, "{applied}");
        assert_eq!(applied["status"], "applied");
        assert_eq!(applied["write"]["persisted_readback"]["pads_match_plan"], 2);

        let (error, again) = call(ctx_talking_to(String::new()), args).await;
        assert!(!error);
        assert_eq!(again["status"], "noop");
        assert_eq!(again["unchanged_count"], 2);
    }

    #[tokio::test]
    async fn a_board_open_in_kicad_is_refused_even_for_a_dry_run() {
        let (_tmp, board) = project();
        let kicad = spawn_kicad_holding_board(&board, |_| None);
        let (error, refused) = call(
            ctx_talking_to(kicad.address().to_string()),
            json!({ "board": board, "references": ["FID1"] }),
        )
        .await;
        assert!(error, "{refused}");
        assert_eq!(std::fs::read_to_string(&board).unwrap(), BOARD);
    }
}
