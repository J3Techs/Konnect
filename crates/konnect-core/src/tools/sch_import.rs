//! Promote embedded imported symbols to a project library without changing geometry.
use crate::{
    mcp::protocol::CallToolResult,
    tools::{get_path, ToolContext, ToolDef},
};
use konnect_sexp::{
    parse_sexp,
    writer::{
        apply_edits, find_direct_child_blocks, read_consistent, write_atomic_if_unchanged,
        write_new_atomic, SexpEdit,
    },
    DocumentRevision, SexpNode,
};
use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet};

pub(super) fn tools() -> Vec<ToolDef> {
    vec![tool!(
 "promote_embedded_symbols", "Copy selected embedded symbols to a new project-local library and qualify their instance library IDs, preserving graphics, pin positions, properties and UUIDs. Optional pin_types changes only electrical types. New library must not exist. Dry-run first and apply the exact revision; register the returned library using register_symbol_library afterward.",
 json!({"type":"object","additionalProperties":false,"properties":{
 "schematic":{"type":"string"},"library_path":{"type":"string"},"nickname":{"type":"string"},
 "symbols":{"type":"array","minItems":1,"items":{"type":"object","additionalProperties":false,"properties":{"reference":{"type":"string"},"name":{"type":"string"},"pin_types":{"type":"object","additionalProperties":{"type":"string"}}},"required":["reference","name"]}},
 "dry_run":{"type":"boolean","default":true},"expected_revision":{"type":"string"}},"required":["schematic","library_path","nickname","symbols"]}),
 |args,ctx| async move {handle(args,ctx).await})]
}

fn quoted(value: &str) -> String {
    format!(
        "\"{}\"",
        value
            .replace('\\', "\\\\")
            .replace('"', "\\\"")
            .replace('\n', "\\n")
            .replace('\r', "\\r")
            .replace('\t', "\\t")
    )
}
fn rename_head(block: &str, name: &str) -> anyhow::Result<String> {
    let start = block
        .find('"')
        .ok_or_else(|| anyhow::anyhow!("missing quoted symbol name"))?;
    let mut escape = false;
    let mut end = None;
    for (i, c) in block[start + 1..].char_indices() {
        if escape {
            escape = false;
        } else if c == '\\' {
            escape = true;
        } else if c == '"' {
            end = Some(start + i + 2);
            break;
        }
    }
    let end = end.ok_or_else(|| anyhow::anyhow!("unterminated symbol name"))?;
    Ok(format!(
        "{}{}{}",
        &block[..start],
        quoted(name),
        &block[end..]
    ))
}
fn rewrite_definition(
    block: &str,
    old: &str,
    name: &str,
    types: &BTreeMap<String, String>,
) -> anyhow::Result<String> {
    let mut edits = Vec::new();
    let mut seen = BTreeSet::new();
    for (a, b) in find_direct_child_blocks(block, "symbol") {
        let child = &block[a..b];
        let node = parse_sexp(child)?;
        if node.head() != Some("symbol") {
            continue;
        }
        let childname = node
            .get(1)
            .and_then(SexpNode::as_str)
            .ok_or_else(|| anyhow::anyhow!("missing unit name"))?;
        let suffix = childname
            .strip_prefix(old)
            .ok_or_else(|| anyhow::anyhow!("unit does not share symbol prefix"))?;
        let mut pe = Vec::new();
        for (x, y) in find_direct_child_blocks(child, "symbol") {
            let pin = &child[x..y];
            let pn = parse_sexp(pin)?;
            if pn.head() != Some("pin") {
                continue;
            }
            let number = pn
                .find_str("number")
                .ok_or_else(|| anyhow::anyhow!("pin missing number"))?;
            if let Some(kind) = types.get(number) {
                seen.insert(number.to_owned());
                let oldkind = pn
                    .get(1)
                    .and_then(SexpNode::as_str)
                    .ok_or_else(|| anyhow::anyhow!("pin missing type"))?;
                let start = pin
                    .find(oldkind)
                    .ok_or_else(|| anyhow::anyhow!("type token missing"))?;
                pe.push(SexpEdit::replace(
                    x + start,
                    x + start + oldkind.len(),
                    kind,
                ));
            }
        }
        let changed = apply_edits(child.to_owned(), pe);
        edits.push(SexpEdit::replace(
            a,
            b,
            rename_head(&changed, &format!("{name}{suffix}"))?,
        ));
    }
    anyhow::ensure!(seen.len() == types.len(), "requested pin number missing");
    rename_head(&apply_edits(block.to_owned(), edits), name)
}
fn plan(content: &str, args: &Value) -> anyhow::Result<(String, String, Vec<Value>)> {
    let tree = parse_sexp(content)?;
    anyhow::ensure!(tree.head() == Some("kicad_sch"), "expected schematic");
    let nickname = args["nickname"]
        .as_str()
        .ok_or_else(|| anyhow::anyhow!("nickname required"))?;
    anyhow::ensure!(
        !nickname.is_empty()
            && nickname
                .chars()
                .all(|c| c.is_ascii_alphanumeric() || c == '_'),
        "nickname must be alphanumeric or underscore"
    );
    let selected = args["symbols"]
        .as_array()
        .ok_or_else(|| anyhow::anyhow!("symbols required"))?;
    anyhow::ensure!(!selected.is_empty(), "symbols empty");
    let blocks = find_direct_child_blocks(content, "kicad_sch");
    let (ls, le) = blocks
        .iter()
        .copied()
        .find(|(a, b)| parse_sexp(&content[*a..*b]).is_ok_and(|n| n.head() == Some("lib_symbols")))
        .ok_or_else(|| anyhow::anyhow!("missing embedded library"))?;
    let lib = &content[ls..le];
    let mut edits = Vec::new();
    let mut embedded = String::new();
    let mut exported = String::from("(kicad_symbol_lib (version 20241209) (generator konnect)\n");
    let mut changes = Vec::new();
    let mut names = BTreeSet::new();
    let mut refs = BTreeSet::new();
    for spec in selected {
        let reference = spec["reference"]
            .as_str()
            .ok_or_else(|| anyhow::anyhow!("reference required"))?;
        let name = spec["name"]
            .as_str()
            .ok_or_else(|| anyhow::anyhow!("name required"))?;
        anyhow::ensure!(
            !name.is_empty() && name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_'),
            "name must be alphanumeric or underscore"
        );
        anyhow::ensure!(
            names.insert(name) && refs.insert(reference),
            "duplicate name or reference"
        );
        let types: BTreeMap<String, String> =
            serde_json::from_value(spec.get("pin_types").cloned().unwrap_or(json!({})))?;
        for kind in types.values() {
            anyhow::ensure!(
                [
                    "input",
                    "output",
                    "bidirectional",
                    "tri_state",
                    "passive",
                    "free",
                    "unspecified",
                    "power_in",
                    "power_out",
                    "open_collector",
                    "open_emitter",
                    "no_connect"
                ]
                .contains(&kind.as_str()),
                "invalid electrical type"
            );
        }
        let instances: Vec<_> = blocks
            .iter()
            .copied()
            .filter(|(a, b)| {
                parse_sexp(&content[*a..*b]).is_ok_and(|n| {
                    n.head() == Some("symbol")
                        && n.find_all("property").iter().any(|p| {
                            p.get(1).and_then(SexpNode::as_str) == Some("Reference")
                                && p.get(2).and_then(SexpNode::as_str) == Some(reference)
                        })
                })
            })
            .collect();
        anyhow::ensure!(!instances.is_empty(), "reference missing: {reference}");
        let first = parse_sexp(&content[instances[0].0..instances[0].1])?;
        let old = first
            .find_str("lib_id")
            .ok_or_else(|| anyhow::anyhow!("lib_id missing"))?;
        let source = find_direct_child_blocks(lib, "lib_symbols")
            .into_iter()
            .find_map(|(a, b)| {
                parse_sexp(&lib[a..b])
                    .ok()
                    .filter(|n| {
                        n.head() == Some("symbol")
                            && n.get(1).and_then(SexpNode::as_str) == Some(old)
                    })
                    .map(|_| &lib[a..b])
            })
            .ok_or_else(|| anyhow::anyhow!("embedded definition missing"))?;
        let oldbase = old.rsplit(':').next().unwrap();
        let definition = rewrite_definition(source, oldbase, name, &types)?;
        let new_id = format!("{nickname}:{name}");
        anyhow::ensure!(
            !tree
                .find("lib_symbols")
                .unwrap()
                .find_all("symbol")
                .iter()
                .any(|n| n.get(1).and_then(SexpNode::as_str) == Some(new_id.as_str())),
            "target embedded ID exists"
        );
        exported.push_str(&definition);
        exported.push('\n');
        embedded.push_str(&rename_head(&definition, &new_id)?);
        embedded.push('\n');
        for (a, b) in instances {
            let instance = &content[a..b];
            let node = parse_sexp(instance)?;
            anyhow::ensure!(
                node.find_str("lib_id") == Some(old),
                "mixed library IDs across units"
            );
            let (x, y) = find_direct_child_blocks(instance, "symbol")
                .into_iter()
                .find(|(x, y)| {
                    parse_sexp(&instance[*x..*y]).is_ok_and(|n| n.head() == Some("lib_id"))
                })
                .ok_or_else(|| anyhow::anyhow!("instance lib_id missing"))?;
            edits.push(SexpEdit::replace(
                a + x,
                a + y,
                format!("(lib_id {})", quoted(&new_id)),
            ));
        }
        changes.push(
            json!({"reference":reference,"old_lib_id":old,"new_lib_id":new_id,"pin_types":types}),
        );
    }
    edits.push(SexpEdit::insert(le - 1, embedded));
    exported.push_str(")\n");
    let candidate = apply_edits(content.to_owned(), edits);
    parse_sexp(&candidate)?;
    parse_sexp(&exported)?;
    Ok((candidate, exported, changes))
}
async fn handle(args: &Value, _ctx: &ToolContext) -> anyhow::Result<CallToolResult> {
    let path = get_path(args, "schematic")?;
    let library = get_path(args, "library_path")?;
    anyhow::ensure!(
        library.extension().and_then(|s| s.to_str()) == Some("kicad_sym"),
        "library must have kicad_sym extension"
    );
    anyhow::ensure!(
        library.parent() == path.parent(),
        "library must be in schematic directory"
    );
    anyhow::ensure!(!library.exists(), "new library already exists");
    let content = read_consistent(&path)?;
    let revision = DocumentRevision::of(&content).to_string();
    let (candidate, exported, changes) = plan(&content, args)?;
    let dry = args["dry_run"].as_bool().unwrap_or(true);
    if !dry {
        anyhow::ensure!(
            args["expected_revision"].as_str() == Some(revision.as_str()),
            "stale or missing revision; dry-run again"
        );
        // Library first: a failed schematic commit leaves an unused new library, never a broken source link.
        write_new_atomic(&library, &exported)?;
        if let Err(error) = write_atomic_if_unchanged(&path, &content, &candidate) {
            anyhow::bail!("Schematic commit failed: {error}. New library {} was created and may remain unused; inspect both paths before retrying.", library.display());
        }
        anyhow::ensure!(
            read_consistent(&path)? == candidate && read_consistent(&library)? == exported,
            "committed readback mismatch; inspect before retrying"
        );
    }
    Ok(CallToolResult::json(
        &json!({"dry_run":dry,"revision":revision,"library_path":library,"nickname":args["nickname"],"changes":changes,"readback_verified":!dry}),
    ))
}
#[cfg(test)]
mod tests {
    use super::*;
    const SCH: &str = r#"(kicad_sch (lib_symbols (symbol "Old" (property "Value" "Old") (symbol "Old_1_1" (rectangle (start 0 0)(end 1 1)) (pin unspecified line (at 2 3 180) (length 2.54) (name "A") (number "1"))))) (symbol (lib_id "Old") (at 10 20 0) (uuid "abc") (property "Reference" "D1")))"#;
    fn args() -> Value {
        json!({"nickname":"Local","symbols":[{"reference":"D1","name":"Diode","pin_types":{"1":"passive"}}]})
    }
    #[test]
    fn promotes_without_geometry_change() {
        let (sch, lib, c) = plan(SCH, &args()).unwrap();
        assert!(sch.contains("(lib_id \"Local:Diode\")"));
        assert!(lib.contains("(pin passive line (at 2 3 180)"));
        assert!(lib.contains("(symbol \"Diode_1_1\""));
        assert!(sch.contains("(uuid \"abc\")"));
        assert_eq!(c.len(), 1);
    }
    #[test]
    fn rejects_missing_pin() {
        let mut a = args();
        a["symbols"][0]["pin_types"] = json!({"9":"passive"});
        assert!(plan(SCH, &a).is_err());
    }
    #[test]
    fn rejects_missing_reference() {
        let mut a = args();
        a["symbols"][0]["reference"] = json!("D2");
        assert!(plan(SCH, &a).is_err());
    }
    #[test]
    fn rejects_invalid_type() {
        let mut a = args();
        a["symbols"][0]["pin_types"] = json!({"1":"magic"});
        assert!(plan(SCH, &a).is_err());
    }
    #[test]
    fn rejects_duplicate_name() {
        let mut a = args();
        let duplicate = a["symbols"][0].clone();
        a["symbols"].as_array_mut().unwrap().push(duplicate);
        assert!(plan(SCH, &a).is_err());
    }
}
