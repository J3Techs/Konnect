//! `set_footprint_field_values`: set, create or remove footprint field
//! values (MPN, LCSC, Value, …) on many placed footprints in one call.
//!
//! The board KiCad holds open is edited over IPC in one commit, each footprint
//! cloned and changed only in the named fields, then read back. A board KiCad
//! provably does not hold (the live gate's verdict, then KiCad's sibling lock)
//! is edited in the saved file: only the value token of an existing
//! `(property …)` changes, a created field is a copy of the footprint's Value
//! property with its own name, value and UUID, hidden, and a removed field's
//! block is cut. Every other byte stays. Either way a dry run (the default)
//! returns a revision and an apply must repeat it.

use super::*;
use crate::mcp::error::ToolErrorKind;
use crate::tools::closed_board;
use crate::tools::pcb_components::{
    check_single_board_form, footprint_reference, sexp_block_tokens, sexp_token_value,
};
use konnect_sexp::writer::{find_direct_child_blocks, new_uuid, read_consistent};

const OPERATION: &str = "set_footprint_field_values";
const MANDATORY: [&str; 4] = ["Reference", "Value", "Datasheet", "Description"];
const MAX_EDITS: usize = 20_000;

/// One footprint's requested values: `Some` sets (creating a missing field),
/// `None` removes the field.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct FootprintEdit {
    pub reference: String,
    pub fields: BTreeMap<String, Option<String>>,
}

fn edits_json(edits: &[FootprintEdit]) -> Value {
    json!(edits
        .iter()
        .map(|e| json!({ "reference": e.reference, "fields": e.fields }))
        .collect::<Vec<_>>())
}

/// Read `edits` (inline) and/or `edits_file`, refusing anything malformed.
pub(crate) fn parse_edits(args: &Value) -> Result<Vec<FootprintEdit>, CallToolResult> {
    let mut raw: Vec<Value> = Vec::new();
    match args.get("edits") {
        None => {}
        Some(Value::Array(a)) => raw.extend(a.iter().cloned()),
        Some(_) => return Err(super::super::invalid_arg("edits", "must be an array")),
    }
    if let Some(p) = args.get("edits_file") {
        let Some(p) = p.as_str().filter(|s| !s.trim().is_empty()) else {
            return Err(super::super::invalid_arg(
                "edits_file",
                "must be a non-empty path",
            ));
        };
        let text = std::fs::read_to_string(p).map_err(|e| {
            super::super::invalid_arg("edits_file", &format!("could not be read: {e}"))
        })?;
        let v: Value = serde_json::from_str(&text)
            .map_err(|e| super::super::invalid_arg("edits_file", &format!("is not JSON: {e}")))?;
        let list = match v {
            Value::Array(a) => a,
            Value::Object(mut o) => match o.remove("edits") {
                Some(Value::Array(a)) => a,
                _ => {
                    return Err(super::super::invalid_arg(
                        "edits_file",
                        "must hold an array, or an object whose 'edits' is one",
                    ))
                }
            },
            _ => {
                return Err(super::super::invalid_arg(
                    "edits_file",
                    "must hold an array of edits",
                ))
            }
        };
        raw.extend(list);
    }
    if raw.is_empty() {
        return Err(super::super::invalid_arg(
            "edits",
            "give at least one edit (edits and/or edits_file)",
        ));
    }
    if raw.len() > MAX_EDITS {
        return Err(super::super::invalid_arg(
            "edits",
            &format!("at most {MAX_EDITS} footprints per call"),
        ));
    }
    let mut out: Vec<FootprintEdit> = Vec::new();
    let mut seen = BTreeSet::new();
    for (i, item) in raw.iter().enumerate() {
        let at = format!("edits[{i}]");
        let Some(obj) = item.as_object() else {
            return Err(super::super::invalid_arg(&at, "must be an object"));
        };
        if let Some(k) = obj.keys().find(|k| *k != "reference" && *k != "fields") {
            return Err(super::super::invalid_arg(
                &format!("{at}.{k}"),
                "is not a field of an edit (reference, fields)",
            ));
        }
        let Some(reference) = obj
            .get("reference")
            .and_then(Value::as_str)
            .filter(|s| !s.is_empty())
        else {
            return Err(super::super::invalid_arg(
                &format!("{at}.reference"),
                "must be a non-empty string",
            ));
        };
        if !seen.insert(reference.to_string()) {
            return Err(super::super::invalid_arg(
                &format!("{at}.reference"),
                &format!("'{reference}' appears twice; merge its fields into one edit"),
            ));
        }
        let Some(fields) = obj.get("fields").and_then(Value::as_object) else {
            return Err(super::super::invalid_arg(
                &format!("{at}.fields"),
                "must be an object of field name to value (string) or null (remove)",
            ));
        };
        if fields.is_empty() {
            return Err(super::super::invalid_arg(
                &format!("{at}.fields"),
                "names no field",
            ));
        }
        let mut map = BTreeMap::new();
        for (name, value) in fields {
            let field = format!("{at}.fields.{name}");
            if name.trim().is_empty() || name.contains(['\0', '\n', '\r', '"']) {
                return Err(super::super::invalid_arg(
                    &field,
                    "a field name must be non-empty, single-line and without quotes",
                ));
            }
            if MANDATORY
                .iter()
                .any(|m| name.eq_ignore_ascii_case(m) && name != m)
            {
                return Err(super::super::invalid_arg(
                    &field,
                    "mandatory field names need their exact capitalization",
                ));
            }
            if name == "Reference" {
                return Err(super::super::invalid_arg(
                    &field,
                    "the Reference is never renamed here; use schematic synchronization",
                ));
            }
            let v = match value {
                Value::String(s) => {
                    if s.contains('\0') {
                        return Err(super::super::invalid_arg(&field, "contains a NUL"));
                    }
                    Some(s.clone())
                }
                Value::Null if MANDATORY.contains(&name.as_str()) => {
                    return Err(super::super::invalid_arg(
                        &field,
                        "a mandatory field cannot be removed; set \"\" to clear it",
                    ))
                }
                Value::Null => None,
                _ => {
                    return Err(super::super::invalid_arg(
                        &field,
                        "must be a string (set, creating it if missing) or null (remove)",
                    ))
                }
            };
            map.insert(name.clone(), v);
        }
        out.push(FootprintEdit {
            reference: reference.to_string(),
            fields: map,
        });
    }
    Ok(out)
}

// ─── Saved file ──────────────────────────────────────────────────────────────

/// One field's state in a footprint block.
#[derive(Debug, Clone, PartialEq)]
struct SavedField {
    value: String,
    /// The whole `(property …)` block, relative to the board.
    block: (usize, usize),
    /// The value token, quotes included, relative to the board.
    value_span: (usize, usize),
}

fn head(block: &str) -> &str {
    sexp_block_tokens(block)
        .first()
        .map_or("", |&(s, e)| &block[s..e])
}

/// The root `(property …)` fields of the footprint at `fp` (board offsets).
fn saved_fields(source: &str, fp: (usize, usize)) -> Result<BTreeMap<String, SavedField>, String> {
    let block = &source[fp.0..fp.1];
    let mut out = BTreeMap::new();
    for (s, e) in find_direct_child_blocks(block, "footprint") {
        let text = &block[s..e];
        if head(text) != "property" {
            continue;
        }
        let tokens = sexp_block_tokens(text);
        let (Some(&(ks, ke)), Some(&(vs, ve))) = (tokens.get(1), tokens.get(2)) else {
            return Err("has a property without a name and value".to_string());
        };
        let name = sexp_token_value(&text[ks..ke]);
        let field = SavedField {
            value: sexp_token_value(&text[vs..ve]),
            block: (fp.0 + s, fp.0 + e),
            value_span: (fp.0 + s + vs, fp.0 + s + ve),
        };
        if out.insert(name.clone(), field).is_some() {
            return Err(format!("has two '{name}' properties"));
        }
    }
    Ok(out)
}

/// Footprint spans by Reference, in board order.
fn saved_footprints(source: &str) -> Result<BTreeMap<String, Vec<(usize, usize)>>, String> {
    check_single_board_form(source)?;
    let mut out: BTreeMap<String, Vec<(usize, usize)>> = BTreeMap::new();
    for (start, end) in find_direct_child_blocks(source, "kicad_pcb") {
        if head(&source[start..end]) != "footprint" {
            continue;
        }
        let node = konnect_sexp::parse_sexp(&source[start..end]).map_err(|e| e.to_string())?;
        if let Some(reference) = footprint_reference(&node) {
            out.entry(reference).or_default().push((start, end));
        }
    }
    Ok(out)
}

fn quote(value: &str) -> String {
    let mut s = String::with_capacity(value.len() + 2);
    s.push('"');
    for c in value.chars() {
        match c {
            '\\' => s.push_str("\\\\"),
            '"' => s.push_str("\\\""),
            '\n' => s.push_str("\\n"),
            '\r' => s.push_str("\\r"),
            '\t' => s.push_str("\\t"),
            c => s.push(c),
        }
    }
    s.push('"');
    s
}

/// A new hidden `(property …)` copied from the Value property's block: its
/// position, layer and text style, with `name`, `value` and a fresh UUID.
fn created_block(value_block: &str, name: &str, value: &str) -> Result<String, String> {
    let tokens = sexp_block_tokens(value_block);
    let (Some(&(ks, ke)), Some(&(vs, ve))) = (tokens.get(1), tokens.get(2)) else {
        return Err("its Value property is malformed".to_string());
    };
    let mut out = String::new();
    out.push_str(&value_block[..ks]);
    out.push_str(&quote(name));
    out.push_str(&value_block[ke..vs]);
    out.push_str(&quote(value));
    let mut rest = String::new();
    let mut last = ve;
    let mut has_hide = false;
    for &(s, e) in &tokens[3..] {
        let t = &value_block[s..e];
        match head(t) {
            "uuid" | "tstamp" => {
                rest.push_str(&value_block[last..s]);
                rest.push_str(&format!("(uuid {})", quote(&new_uuid())));
            }
            "hide" => {
                has_hide = true;
                rest.push_str(&value_block[last..s]);
                rest.push_str("(hide yes)");
            }
            _ => rest.push_str(&value_block[last..e]),
        }
        last = e;
    }
    rest.push_str(&value_block[last..]);
    if !has_hide {
        // After (layer …) when present, as KiCad writes it; else after the value.
        match rest.find("(layer ") {
            Some(at) => {
                let (ls, le) = konnect_sexp::writer::find_balanced_block(&rest, at)
                    .ok_or("its Value property has an unbalanced layer clause")?;
                debug_assert_eq!(ls, at);
                rest.insert_str(le, " (hide yes)");
            }
            None => rest.insert_str(0, " (hide yes)"),
        }
    }
    out.push_str(&rest);
    Ok(out)
}

/// The planned file edits and per-footprint before/after values.
#[derive(Debug)]
pub(crate) struct SavedPlan {
    pub candidate: String,
    edits: Vec<(usize, usize, usize)>,
    pub changes: Vec<Value>,
    pub expected: Vec<(String, BTreeMap<String, Option<String>>)>,
}

/// Plan every edit against the saved `source`; any failure refuses the batch.
pub(crate) fn plan_saved(source: &str, edits: &[FootprintEdit]) -> Result<SavedPlan, Vec<String>> {
    let footprints = saved_footprints(source).map_err(|e| vec![format!("board: {e}")])?;
    let mut reasons = Vec::new();
    let mut spans: Vec<(usize, usize, String)> = Vec::new();
    let mut changes = Vec::new();
    let mut expected = Vec::new();
    for edit in edits {
        let fp = match footprints.get(&edit.reference).map(Vec::as_slice) {
            Some([one]) => *one,
            Some(_) => {
                reasons.push(format!("{}: more than one footprint", edit.reference));
                continue;
            }
            None => {
                reasons.push(format!("{}: no such footprint", edit.reference));
                continue;
            }
        };
        let fields = match saved_fields(source, fp) {
            Ok(f) => f,
            Err(e) => {
                reasons.push(format!("{}: {e}", edit.reference));
                continue;
            }
        };
        let mut before = BTreeMap::new();
        let mut inserts: Vec<String> = Vec::new();
        for (name, want) in &edit.fields {
            let have = fields.get(name);
            before.insert(name.clone(), have.map(|f| f.value.clone()));
            match (have, want) {
                (Some(f), Some(v)) if f.value == *v => {}
                (Some(f), Some(v)) => spans.push((f.value_span.0, f.value_span.1, quote(v))),
                (None, Some(v)) => {
                    let Some(value) = fields.get("Value") else {
                        reasons.push(format!(
                            "{}: has no Value property to copy a new field's style from",
                            edit.reference
                        ));
                        continue;
                    };
                    match created_block(&source[value.block.0..value.block.1], name, v) {
                        Ok(b) => inserts.push(b),
                        Err(e) => reasons.push(format!("{}: {e}", edit.reference)),
                    }
                }
                (Some(f), None) => {
                    // Cut the block and the whitespace before it on its line.
                    let mut start = f.block.0;
                    while start > fp.0 && matches!(source.as_bytes()[start - 1], b' ' | b'\t') {
                        start -= 1;
                    }
                    if start > fp.0 && source.as_bytes()[start - 1] == b'\n' {
                        start -= 1;
                        if start > fp.0 && source.as_bytes()[start - 1] == b'\r' {
                            start -= 1;
                        }
                    }
                    spans.push((start, f.block.1, String::new()));
                }
                (None, None) => {}
            }
        }
        if !inserts.is_empty() {
            // After the last root property, on new lines with its indentation.
            let last = fields.values().map(|f| f.block).max_by_key(|b| b.1);
            let Some((ls, le)) = last else {
                reasons.push(format!(
                    "{}: has no property to insert after",
                    edit.reference
                ));
                continue;
            };
            let line_start = source[..ls].rfind('\n').map_or(0, |i| i + 1);
            let indent: String = source[line_start..ls]
                .chars()
                .take_while(|c| *c == ' ' || *c == '\t')
                .collect();
            let nl = if source[..ls].ends_with('\r') || source.contains("\r\n") {
                "\r\n"
            } else {
                "\n"
            };
            let text: String = inserts.iter().map(|b| format!("{nl}{indent}{b}")).collect();
            spans.push((le, le, text));
        }
        let after: BTreeMap<String, Option<String>> = edit.fields.clone();
        let changed = edit
            .fields
            .iter()
            .any(|(n, v)| before.get(n).cloned().flatten() != *v);
        changes.push(json!({
            "reference": edit.reference,
            "changed": changed,
            "before": before,
            "after": after,
        }));
        expected.push((edit.reference.clone(), after));
    }
    if !reasons.is_empty() {
        return Err(reasons);
    }
    spans.sort_by_key(|s| (s.0, s.1));
    if spans.windows(2).any(|w| w[1].0 < w[0].1) {
        return Err(vec!["planned edits overlap".to_string()]);
    }
    let mut candidate = String::with_capacity(source.len() + 1024);
    let mut last = 0;
    let mut file_edits = Vec::new();
    for (s, e, text) in &spans {
        candidate.push_str(&source[last..*s]);
        candidate.push_str(text);
        file_edits.push((*s, *e, text.len()));
        last = *e;
    }
    candidate.push_str(&source[last..]);
    Ok(SavedPlan {
        candidate,
        edits: file_edits,
        changes,
        expected,
    })
}

/// Check a persisted board: only the planned spans changed, and every
/// edited footprint now holds exactly the requested values.
fn verify_saved(
    original: &str,
    persisted: &str,
    plan_edits: &[(usize, usize, usize)],
    expected: &[(String, BTreeMap<String, Option<String>>)],
) -> Result<Value, String> {
    if !closed_board::only_spans_changed(original, persisted, plan_edits) {
        return Err("bytes outside the planned spans changed".to_string());
    }
    let footprints = saved_footprints(persisted)?;
    for (reference, want) in expected {
        let Some([fp]) = footprints.get(reference).map(Vec::as_slice) else {
            return Err(format!("{reference} is not one footprint after the write"));
        };
        let fields = saved_fields(persisted, *fp)?;
        for (name, v) in want {
            let got = fields.get(name).map(|f| f.value.clone());
            if got != *v {
                return Err(format!(
                    "{reference}.{name} reads back as {got:?}, not {v:?}"
                ));
            }
        }
    }
    Ok(json!({ "footprints_verified": expected.len() }))
}

// ─── Live board ──────────────────────────────────────────────────────────────

/// The live footprint with `edit` applied to its fields.
fn live_prepare(fp: &Footprint, edit: &FootprintEdit) -> Result<Footprint> {
    let existing = fields(fp)?;
    let mut sets: Vec<Change> = Vec::new();
    for (name, value) in &edit.fields {
        if let Some(v) = value {
            if existing
                .get(name)
                .and_then(|f| f.text.as_ref())
                .and_then(|t| t.text.as_ref())
                .is_some_and(|t| t.text == *v)
            {
                continue;
            }
            sets.push(serde_json::from_value(
                json!({ "name": name, "value": v, "create": !existing.contains_key(name) }),
            )?);
        }
    }
    let mut out = if sets.is_empty() {
        fp.clone()
    } else {
        prepare(fp, &sets)?
    };
    for (name, value) in &edit.fields {
        if value.is_none() && existing.contains_key(name) {
            let items = &mut out.definition.as_mut().context("missing definition")?.items;
            let before = items.len();
            let mut kept = Vec::with_capacity(before);
            for item in items.drain(..) {
                if builders::any_is(&item, FIELD_TYPE)
                    && Field::decode(item.value.as_slice())?.name == *name
                {
                    continue;
                }
                kept.push(item);
            }
            *items = kept;
            if items.len() + 1 != before {
                bail!("field '{name}' could not be removed exactly once");
            }
        }
    }
    Ok(out)
}

fn live_values(fp: &Footprint, names: &BTreeMap<String, Option<String>>) -> Result<Value> {
    let all = fields(fp)?;
    let mut out = serde_json::Map::new();
    for name in names.keys() {
        let v = all
            .get(name)
            .and_then(|f| f.text.as_ref())
            .and_then(|t| t.text.as_ref())
            .map(|t| t.text.clone());
        out.insert(name.clone(), json!(v));
    }
    Ok(Value::Object(out))
}

// ─── Tool ────────────────────────────────────────────────────────────────────

pub(crate) fn tool() -> ToolDef {
    tool!(
        "set_footprint_field_values",
        "Set, create or remove footprint field values (MPN, LCSC, Value, Datasheet, any custom \
         property) on many placed footprints in one call. Each edit names a footprint by \
         reference and maps field names to a string (set; a missing field is created hidden, \
         styled like the footprint's Value) or null (remove a custom field; set \"\" to clear a \
         mandatory one). The board KiCad holds open is edited over live IPC in one commit and \
         read back; a board KiCad provably does not hold (and has no sibling lock) is edited in \
         the saved file with one revision-checked atomic write that changes only the named \
         values. Dry run by default: it returns plan_revision, which an apply must repeat in \
         expected_revision. One bad edit refuses the whole batch.",
        json!({
            "type": "object",
            "properties": {
                "board": { "type": "string", "description": "The .kicad_pcb (open in KiCad, or the saved file when closed)" },
                "edits": {
                    "type": "array", "maxItems": MAX_EDITS,
                    "items": {
                        "type": "object",
                        "properties": {
                            "reference": { "type": "string", "minLength": 1 },
                            "fields": { "type": "object", "minProperties": 1,
                                        "additionalProperties": { "type": ["string", "null"] },
                                        "description": "Field name to value; null removes a custom field" }
                        },
                        "required": ["reference", "fields"],
                        "additionalProperties": false
                    }
                },
                "edits_file": { "type": "string", "description": "A JSON file of more edits (an array, or {\"edits\": [...]}), with or instead of edits" },
                "dry_run": { "type": "boolean", "default": true },
                "expected_revision": { "type": "string", "description": "The plan_revision of the dry run; required to apply" }
            },
            "required": ["board"]
        }),
        |args, ctx| async move { handle_set_footprint_field_values(args, ctx).await }
    )
    .with_board_access(crate::tools::BoardAccess::LivePreferredWithFallback)
}

async fn handle_set_footprint_field_values(
    args: &Value,
    ctx: &ToolContext,
) -> Result<CallToolResult> {
    let board = super::super::get_path(args, "board")?;
    let edits = match parse_edits(args) {
        Ok(e) => e,
        Err(r) => return Ok(r),
    };
    let dry_run = match args.get("dry_run") {
        None => true,
        Some(Value::Bool(b)) => *b,
        Some(_) => return Ok(super::super::invalid_arg("dry_run", "must be a boolean")),
    };
    let expected = match args.get("expected_revision") {
        None => None,
        Some(Value::String(s)) if !s.is_empty() => Some(s.clone()),
        Some(_) => {
            return Ok(super::super::invalid_arg(
                "expected_revision",
                "must be a non-empty string",
            ))
        }
    };
    if !dry_run && expected.is_none() {
        return Ok(super::super::invalid_arg(
            "expected_revision",
            "an apply needs the plan_revision of a dry run",
        ));
    }
    let request = edits_json(&edits);

    // Live first: the board KiCad holds is the authoritative copy.
    let path = board.clone();
    let live_edits = edits.clone();
    let live_request = request.clone();
    let live_expected = expected.clone();
    let attempt = attempt_ipc_write(ctx, &board, OPERATION, move |client| {
        let doc = client.find_open_board(&path)?;
        let kind = kiapi::common::types::KiCadObjectType::KotPcbFootprint;
        let items = client.get_items_in(doc.clone(), kind)?;
        let mut before = Vec::new();
        for e in &live_edits {
            before.push(select(items.clone(), &e.reference)?);
        }
        let mut hasher = Sha256::new();
        hasher.update(path.to_string_lossy().as_bytes());
        for fp in &before {
            hasher.update(fp.encode_to_vec());
        }
        hasher.update(live_request.to_string().as_bytes());
        let rev = format!("{:x}", hasher.finalize());
        let mut updated = Vec::new();
        let mut changes = Vec::new();
        for (fp, e) in before.iter().zip(&live_edits) {
            let u = live_prepare(fp, e)?;
            changes.push(json!({
                "reference": e.reference,
                "changed": u != *fp,
                "before": live_values(fp, &e.fields)?,
                "after": live_values(&u, &e.fields)?,
            }));
            updated.push(u);
        }
        let changed: Vec<usize> = (0..updated.len())
            .filter(|&i| updated[i] != before[i])
            .collect();
        if dry_run || changed.is_empty() {
            return Ok(json!({
                "status": if changed.is_empty() { "noop" } else { "ready" },
                "applied": false, "source": "ipc", "plan_revision": rev,
                "footprints": live_edits.len(), "changed": changed.len(), "changes": changes,
            }));
        }
        if live_expected.as_deref() != Some(rev.as_str()) {
            return Ok(json!({ "status": "stale_revision", "applied": false, "source": "ipc",
                              "diagnostic": "expected_revision does not match the live footprints and request; rerun the dry run" }));
        }
        let packed: Vec<_> = changed
            .iter()
            .map(|&i| builders::pack_any(&updated[i], FP_TYPE))
            .collect();
        let outcome = client
            .run_commit("Set footprint field values", |c| {
                c.update_items_in(doc.clone(), packed)
            })
            .and_then(|()| {
                let items = client.get_items_in(doc.clone(), kind)?;
                for &i in &changed {
                    let actual = select(items.clone(), &live_edits[i].reference)?;
                    if comparable(&actual)? != comparable(&updated[i])? {
                        bail!(
                            "{} reads back differently: {}",
                            live_edits[i].reference,
                            live_values(&actual, &live_edits[i].fields)?
                        );
                    }
                }
                Ok(())
            });
        Ok(match outcome {
            Ok(()) => json!({
                "status": "complete", "applied": true, "source": "ipc", "saved": false,
                "plan_revision": rev, "footprints": live_edits.len(), "changed": changed.len(),
                "changes": changes,
                "next": "save_project (or save in KiCad) to persist the change",
            }),
            Err(e) => json!({
                "status": "uncertain", "applied": Value::Null, "source": "ipc",
                "diagnostic": format!("{e:#}"),
                "recovery": "Inspect the live footprints before retrying; no rollback was attempted",
            }),
        })
    })
    .await?;
    match attempt {
        BoardWrite::Ipc(body) => return Ok(CallToolResult::json(&body)),
        BoardWrite::Refused(r) => return Ok(r),
        BoardWrite::File(_) => {}
    }

    // Closed: prove it, then plan against the exact saved bytes.
    let closure = match closed_board::prove_closed(ctx, &board, OPERATION).await? {
        Ok(c) => c,
        Err(r) => return Ok(r),
    };
    let original = read_consistent(&board)?;
    let plan = match plan_saved(&original, &edits) {
        Ok(p) => p,
        Err(reasons) => {
            return Ok(CallToolResult::error_kind(
                ToolErrorKind::PlanBlocked {
                    operation: OPERATION.to_string(),
                    reasons: reasons.clone(),
                },
                format!(
                    "No field changed: {}. The board file was not modified.",
                    reasons.join("; ")
                ),
            ))
        }
    };
    let rev = closed_board::revision(OPERATION, &original, &request);
    let changed = plan.changes.iter().filter(|c| c["changed"] == true).count();
    let mut body = json!({
        "status": if changed == 0 { "noop" } else if dry_run { "ready" } else { "applied" },
        "applied": false,
        "source": "saved_file",
        "plan_revision": rev,
        "footprints": edits.len(),
        "changed": changed,
        "changes": plan.changes,
        "board_closure": closure,
    });
    if dry_run || changed == 0 {
        return Ok(CallToolResult::json(&body));
    }
    if expected.as_deref() != Some(rev.as_str()) {
        return Ok(closed_board::stale_revision(&board, OPERATION));
    }
    if let Err(r) = closed_board::prove_closed(ctx, &board, OPERATION).await? {
        return Ok(r);
    }
    let file_edits = plan.edits.clone();
    let expected_values = plan.expected.clone();
    match closed_board::write_and_read_back(&board, OPERATION, &original, &plan.candidate, |t| {
        verify_saved(&original, t, &file_edits, &expected_values)
    }) {
        Ok(write) => {
            body["applied"] = json!(true);
            body["write"] = write;
            body["next"] = json!("Reload the board in KiCad before editing it there.");
        }
        Err(r) => return Ok(r),
    }
    Ok(CallToolResult::json(&body))
}

#[cfg(test)]
mod tests {
    use super::*;

    const BOARD: &str = "(kicad_pcb (version 20260206)\n\t(footprint \"R_0603\" (layer \"F.Cu\") (uuid \"11111111-1111-4111-8111-111111111111\") (at 10 20 90)\n\t\t(property \"Reference\" \"R10203\" (at 0 -1.4 90) (layer \"F.SilkS\") (uuid \"21111111-1111-4111-8111-111111111111\")\n\t\t\t(effects (font (size 1 1) (thickness 0.15))))\n\t\t(property \"Value\" \"10k\" (at 0 1.4 90) (layer \"F.Fab\") (uuid \"31111111-1111-4111-8111-111111111111\")\n\t\t\t(effects (font (size 1 1) (thickness 0.15))))\n\t\t(property \"MPN\" \"0603WAF1002T5E\" (at 0 0 90) (layer \"F.Fab\") (hide yes) (uuid \"41111111-1111-4111-8111-111111111111\")\n\t\t\t(effects (font (size 1 1) (thickness 0.15))))\n\t\t(property \"Old\" \"x\" (at 0 0 90) (layer \"F.Fab\") (hide yes) (uuid \"51111111-1111-4111-8111-111111111111\")\n\t\t\t(effects (font (size 1 1) (thickness 0.15))))\n\t\t(pad \"1\" smd roundrect (at -0.8 0 90) (size 0.8 0.9) (layers \"F.Cu\") (net 1 \"A\") (property \"Inner\" \"keep\") (uuid \"61111111-1111-4111-8111-111111111111\"))\n\t)\n\t(footprint \"R_0603\" (layer \"F.Cu\") (at 30 20)\n\t\t(property \"Reference\" \"R10204\" (at 0 -1.4 0) (layer \"F.SilkS\") (effects (font (size 1 1) (thickness 0.15))))\n\t\t(property \"Value\" \"10k\" (at 0 1.4 0) (layer \"F.Fab\") (effects (font (size 1 1) (thickness 0.15))))\n\t)\n)\n";

    fn edits(v: Value) -> Vec<FootprintEdit> {
        parse_edits(&json!({ "edits": v })).unwrap()
    }

    #[test]
    fn sets_creates_and_removes_only_the_named_values() {
        let e = edits(json!([
            { "reference": "R10203", "fields": { "Value": "0R", "MPN": "0603WAF0000T5E", "LCSC": "C21189", "Old": null } },
            { "reference": "R10204", "fields": { "Value": "0R", "MPN": "0603WAF0000T5E", "LCSC": "C21189" } }
        ]));
        let plan = plan_saved(BOARD, &e).unwrap();
        let fps = saved_footprints(&plan.candidate).unwrap();
        let a = saved_fields(&plan.candidate, fps["R10203"][0]).unwrap();
        assert_eq!(a["Value"].value, "0R");
        assert_eq!(a["MPN"].value, "0603WAF0000T5E");
        assert_eq!(a["LCSC"].value, "C21189");
        assert!(!a.contains_key("Old"));
        let b = saved_fields(&plan.candidate, fps["R10204"][0]).unwrap();
        assert_eq!(b["LCSC"].value, "C21189");
        // A created field is hidden, keeps the Value's layer and gets its own UUID.
        let lcsc = &plan.candidate[a["LCSC"].block.0..a["LCSC"].block.1];
        assert!(lcsc.contains("(hide yes)") && lcsc.contains("(layer \"F.Fab\")"));
        assert!(!lcsc.contains("31111111-1111-4111-8111-111111111111"));
        // The pad's own property and every other byte are untouched.
        assert!(plan.candidate.contains("(property \"Inner\" \"keep\")"));
        verify_saved(BOARD, &plan.candidate, &plan.edits, &plan.expected).unwrap();
        konnect_sexp::parse_sexp(&plan.candidate).unwrap();
    }

    #[test]
    fn an_unchanged_request_plans_nothing_and_bad_members_refuse_the_batch() {
        let e = edits(json!([{ "reference": "R10203", "fields": { "Value": "10k" } }]));
        let plan = plan_saved(BOARD, &e).unwrap();
        assert_eq!(plan.candidate, BOARD);
        assert_eq!(plan.changes[0]["changed"], false);
        let bad = edits(json!([
            { "reference": "R10203", "fields": { "LCSC": "C1" } },
            { "reference": "R9", "fields": { "LCSC": "C1" } }
        ]));
        let reasons = plan_saved(BOARD, &bad).unwrap_err();
        assert!(reasons[0].contains("R9"), "{reasons:?}");
    }

    #[test]
    fn malformed_requests_are_refused_before_planning() {
        for args in [
            json!({ "edits": [] }),
            json!({ "edits": [{ "reference": "R1", "fields": { "Reference": "R2" } }] }),
            json!({ "edits": [{ "reference": "R1", "fields": { "Value": null } }] }),
            json!({ "edits": [{ "reference": "R1", "fields": { "value": "x" } }] }),
            json!({ "edits": [{ "reference": "R1", "fields": { "MPN": 3 } }] }),
            json!({ "edits": [{ "reference": "R1", "fields": { "A": "1" } }, { "reference": "R1", "fields": { "B": "2" } }] }),
        ] {
            assert!(parse_edits(&args).is_err(), "{args}");
        }
    }

    #[test]
    fn values_are_escaped_as_kicad_reads_them() {
        let e = edits(json!([{ "reference": "R10204", "fields": { "Note": "a \"b\" \\ c" } }]));
        let plan = plan_saved(BOARD, &e).unwrap();
        let fps = saved_footprints(&plan.candidate).unwrap();
        let f = saved_fields(&plan.candidate, fps["R10204"][0]).unwrap();
        assert_eq!(f["Note"].value, "a \"b\" \\ c");
    }
}
