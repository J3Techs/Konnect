//! Native schematic population attributes, with revision-bound atomic updates.
use crate::mcp::protocol::CallToolResult;
use crate::tools::{find_all_symbol_instance_blocks, get_path};
use anyhow::{bail, Context};
use konnect_sexp::{
    parse_sexp,
    writer::{
        apply_edits, find_direct_child_blocks, read_consistent, write_atomic_if_unchanged, SexpEdit,
    },
};
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;

fn plan(content: &str, references: &[String], dnp: bool) -> anyhow::Result<(String, Value)> {
    let tree = parse_sexp(content)?;
    if tree.head() != Some("kicad_sch") {
        bail!("expected a schematic");
    }
    let mut seen = BTreeSet::new();
    let mut edits = Vec::new();
    let mut units = 0;
    let mut changed = 0;
    for reference in references {
        if !seen.insert(reference) {
            bail!("duplicate reference '{reference}'");
        }
        let blocks = find_all_symbol_instance_blocks(content, reference);
        if blocks.is_empty() {
            bail!("reference '{reference}' not found");
        }
        for (start, end) in blocks {
            units += 1;
            let block = &content[start..end];
            let children = find_direct_child_blocks(block, "symbol");
            let mut flags = Vec::new();
            for (a, b) in children {
                let node = parse_sexp(&block[a..b])?;
                if node.head() == Some("dnp") {
                    if node.children().map(|c| c.len()) != Some(2)
                        || !matches!(node.get(1).and_then(|n| n.as_str()), Some("yes" | "no"))
                    {
                        bail!("invalid dnp attribute on '{reference}'");
                    }
                    flags.push((a, b, node.get(1).unwrap().as_str().unwrap() == "yes"));
                }
            }
            if flags.len() > 1 {
                bail!("duplicate dnp attributes on '{reference}'");
            }
            let replacement = format!("(dnp {})", if dnp { "yes" } else { "no" });
            if let Some((a, b, old)) = flags.first() {
                if *old != dnp {
                    edits.push(SexpEdit::replace(start + a, start + b, replacement));
                    changed += 1;
                }
            } else if dnp {
                // Absent DNP means populated. Insert only the native direct child.
                edits.push(SexpEdit::insert(
                    end - 1,
                    format!("\n\t\t{replacement}\n\t"),
                ));
                changed += 1;
            }
        }
    }
    let out = apply_edits(content.to_owned(), edits);
    parse_sexp(&out)?;
    Ok((
        out,
        json!({"references":references,"dnp":dnp,"units":units,"changed_units":changed}),
    ))
}

pub(super) async fn handle(
    args: &Value,
    _ctx: &crate::tools::ToolContext,
) -> anyhow::Result<CallToolResult> {
    let path = get_path(args, "schematic")?;
    let refs = args["references"]
        .as_array()
        .context("references must be an array")?
        .iter()
        .map(|v| {
            v.as_str()
                .map(String::from)
                .context("references must be strings")
        })
        .collect::<anyhow::Result<Vec<_>>>()?;
    if refs.is_empty() {
        bail!("references must not be empty");
    }
    let dnp = args["dnp"].as_bool().context("dnp must be boolean")?;
    let dry = match args.get("dry_run") {
        None => true,
        Some(v) => v.as_bool().context("dry_run must be boolean")?,
    };
    let content = read_consistent(&path)?;
    let (updated, mut result) = plan(&content, &refs, dnp)?;
    let revision = format!(
        "{:x}",
        Sha256::digest(serde_json::to_vec(&(
            path.to_string_lossy(),
            &content,
            &refs,
            dnp
        ))?)
    );
    result["plan_revision"] = json!(revision);
    if !dry && args["expected_plan_revision"].as_str() != Some(revision.as_str()) {
        return Ok(CallToolResult::error(
            "stale or missing expected_plan_revision; run a fresh dry run",
        ));
    }
    if content == updated {
        result["status"] = json!("noop");
    } else if dry {
        result["status"] = json!("ready");
    } else {
        write_atomic_if_unchanged(&path, &content, &updated)?;
        let readback = read_consistent(&path)?;
        if readback != updated {
            bail!("committed population change readback differs; inspect before retrying");
        }
        let (verified, _) = plan(&readback, &refs, dnp)?;
        if verified != readback {
            bail!("native DNP readback mismatch");
        }
        result["status"] = json!("applied");
    }
    Ok(CallToolResult::json(&result))
}

#[cfg(test)]
mod tests {
    use super::*;
    const SCH: &str = r#"(kicad_sch (lib_symbols (symbol "L" (dnp no)))
      (symbol (lib_id "L") (unit 1) (dnp no) (property "Reference" "U1") (property "Note" "(dnp no)"))
      (symbol (lib_id "L") (unit 2) (property "Reference" "U1"))
      (symbol (lib_id "L") (dnp no) (property "Reference" "R1")))"#;
    #[test]
    fn changes_every_unit_but_not_neighbor_library_or_custom_property() {
        let (out, result) = plan(SCH, &["U1".into()], true).unwrap();
        assert_eq!(result["units"], 2);
        assert_eq!(result["changed_units"], 2);
        assert!(out.contains("(lib_symbols (symbol \"L\" (dnp no)))"));
        assert!(out.contains("(property \"Note\" \"(dnp no)\")"));
        assert!(out.contains("(dnp no) (property \"Reference\" \"R1\")"));
        let (again, result) = plan(&out, &["U1".into()], true).unwrap();
        assert_eq!(again, out);
        assert_eq!(result["changed_units"], 0);
        let (populated, result) = plan(&out, &["U1".into()], false).unwrap();
        assert_eq!(result["changed_units"], 2);
        assert!(!populated.contains("(dnp yes)"));
    }
    #[test]
    fn invalid_targets_and_ambiguous_attributes_refuse_the_whole_plan() {
        assert!(plan(SCH, &["U1".into(), "missing".into()], true).is_err());
        assert!(plan(SCH, &["U1".into(), "U1".into()], true).is_err());
        assert!(plan(
            &SCH.replacen("(unit 1) (dnp no)", "(unit 1) (dnp no) (dnp yes)", 1),
            &["U1".into()],
            true
        )
        .is_err());
        assert!(plan(
            &SCH.replacen("(unit 1) (dnp no)", "(unit 1) (dnp maybe)", 1),
            &["U1".into()],
            true
        )
        .is_err());
    }
    #[tokio::test]
    async fn dry_run_is_non_mutating_and_apply_requires_exact_revision() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("test.kicad_sch");
        std::fs::write(&path, SCH).unwrap();
        let ctx = crate::tools::ToolContext::new(
            crate::tools::ServerConfig::default(),
            std::sync::Arc::new(crate::router::ToolRouter::new()),
        );
        let mut args = json!({"schematic":path,"references":["U1"],"dnp":true});
        let r = handle(&args, &ctx).await.unwrap();
        let crate::mcp::protocol::ToolContent::Text { text } = &r.content[0] else {
            panic!()
        };
        let result: Value = serde_json::from_str(text).unwrap();
        assert_eq!(result["status"], "ready");
        assert_eq!(std::fs::read_to_string(&path).unwrap(), SCH);
        args["dry_run"] = json!(false);
        assert!(handle(&args, &ctx).await.unwrap().is_error);
        args["expected_plan_revision"] = result["plan_revision"].clone();
        assert!(!handle(&args, &ctx).await.unwrap().is_error);
        let saved = std::fs::read_to_string(&path).unwrap();
        assert_eq!(plan(&saved, &["U1".into()], true).unwrap().0, saved);
        assert!(handle(&args, &ctx).await.unwrap().is_error);
    }
}
