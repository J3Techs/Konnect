//! Revision-bound pad clearance rules with explicit footprint scopes.
use super::{get_path, ToolDef};
use crate::{mcp::protocol::CallToolResult, tool};
use anyhow::{ensure, Context, Result};
use konnect_sexp::writer::{read_consistent, write_atomic_if_unchanged};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::{collections::BTreeSet, path::Path};
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
enum Constraint {
    EdgeClearance,
    HoleClearance,
}
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Rule {
    name: String,
    references: Vec<String>,
    constraint: Constraint,
    minimum_mm: f64,
    rationale: String,
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Args {
    board: String,
    rules: Vec<Rule>,
    #[serde(default = "yes")]
    dry_run: bool,
    expected_plan_revision: Option<String>,
}
fn yes() -> bool {
    true
}
fn token(s: &str) -> bool {
    !s.is_empty()
        && s.len() <= 100
        && s.chars()
            .all(|c| c.is_ascii_alphanumeric() || "_-".contains(c))
}
fn render(r: &Rule) -> Result<(String, String)> {
    ensure!(token(&r.name), "rule name must be an ASCII identifier");
    ensure!(
        !r.references.is_empty(),
        "a nonempty footprint allowlist is required"
    );
    ensure!(
        r.minimum_mm.is_finite() && r.minimum_mm >= 0.05 && r.minimum_mm <= 10.0,
        "minimum_mm outside 0.05..10"
    );
    ensure!(
        !r.rationale.trim().is_empty() && !r.rationale.contains(['\n', '\r', '\0']),
        "single-line rationale required"
    );
    let mut refs = BTreeSet::new();
    for reference in &r.references {
        ensure!(
            token(reference) && refs.insert(reference),
            "invalid or duplicate footprint reference"
        );
    }
    let mut terms = Vec::new();
    for reference in refs {
        terms.push(match r.constraint {
            Constraint::EdgeClearance => format!("A.memberOfFootprint('{reference}')"),
            Constraint::HoleClearance => format!(
                "(A.memberOfFootprint('{reference}') && B.memberOfFootprint('{reference}'))"
            ),
        });
    }
    let (kind, typ) = match r.constraint {
        Constraint::EdgeClearance => ("edge_clearance", "A.Type == 'Pad'"),
        Constraint::HoleClearance => ("hole_clearance", "A.Type == 'Pad' && B.Type == 'Pad'"),
    };
    let name = format!("konnect:scoped:{}", r.name);
    let rule = format!(
        "(rule \"{name}\"\n  # {}\n  (condition \"{typ} && ({})\")\n  (constraint {kind} (min {}mm)))",
        r.rationale,
        terms.join(" || "),
        r.minimum_mm
    );
    Ok((name, rule))
}
fn candidate(old: &str, rules: &[Rule]) -> Result<String> {
    ensure!(!rules.is_empty(), "rules must not be empty");
    let mut names = BTreeSet::new();
    let mut new = old.to_string();
    for r in rules {
        let (n, text) = render(r)?;
        ensure!(names.insert(n.clone()), "duplicate rule name");
        new = super::verification::upsert_named_rule(&new, &n, &text);
    }
    // Parse a wrapper because .kicad_dru contains several top-level expressions.
    konnect_sexp::parser::parse_sexp(&format!("(rules {new})"))?;
    Ok(new)
}
fn revision(board: &Path, board_content: &str, old: &str, new: &str) -> String {
    let mut h = Sha256::new();
    for bytes in [
        board.as_os_str().as_encoded_bytes(),
        board_content.as_bytes(),
        old.as_bytes(),
        new.as_bytes(),
    ] {
        h.update((bytes.len() as u64).to_le_bytes());
        h.update(bytes);
    }
    format!("{:x}", h.finalize())
}
pub(super) fn tool() -> ToolDef {
    tool!("set_scoped_design_rules","Plan or apply pad-only edge/hole clearance rules scoped to exact footprint reference allowlists. Hole rules apply only within each named footprint. Preserves unrelated rules and board geometry. Dry run defaults true; apply requires exact board/rules revision. Requires a rationale. Writes native .kicad_dru; run native DRC afterward. Does not waive violations or change global minima.",
 json!({"type":"object","additionalProperties":false,"properties":{"board":{"type":"string"},"rules":{"type":"array","minItems":1,"items":{"type":"object","additionalProperties":false,"properties":{"name":{"type":"string"},"references":{"type":"array","minItems":1,"items":{"type":"string"}},"constraint":{"type":"string","enum":["edge_clearance","hole_clearance"]},"minimum_mm":{"type":"number","minimum":0.05,"maximum":10},"rationale":{"type":"string"}},"required":["name","references","constraint","minimum_mm","rationale"]}},"dry_run":{"type":"boolean","default":true},"expected_plan_revision":{"type":"string"}},"required":["board","rules"]}),
 |args,_ctx| async move {handle(args).await})
}
async fn handle(args: &Value) -> Result<CallToolResult> {
    let a: Args = serde_json::from_value(args.clone())?;
    ensure!(!a.board.is_empty(), "board required");
    if !a.dry_run {
        ensure!(
            a.expected_plan_revision.is_some(),
            "apply requires expected_plan_revision"
        );
    }
    let board = get_path(args, "board")?;
    ensure!(
        board.extension().is_some_and(|e| e == "kicad_pcb"),
        "board extension must be .kicad_pcb"
    );
    let board_content = read_consistent(&board)?;
    let tree = konnect_sexp::parser::parse_sexp(&board_content)?;
    let references: BTreeSet<String> = tree
        .find_all("footprint")
        .iter()
        .flat_map(|fp| fp.find_all("property"))
        .filter(|p| p.get(1).and_then(|x| x.as_str()) == Some("Reference"))
        .filter_map(|p| p.get(2).and_then(|x| x.as_str()).map(str::to_owned))
        .collect();
    for r in &a.rules {
        for reference in &r.references {
            ensure!(
                references.contains(reference),
                "footprint '{reference}' absent from saved board"
            );
        }
    }
    let path = board.with_extension("kicad_dru");
    let existed = path.exists();
    let old = if existed {
        read_consistent(&path)?
    } else {
        "(version 1)\n".into()
    };
    let new = candidate(&old, &a.rules)?;
    let rev = revision(&board, &board_content, &old, &new);
    if !a.dry_run {
        ensure!(
            a.expected_plan_revision.as_deref() == Some(&rev),
            "stale plan revision"
        );
        ensure!(
            read_consistent(&board)? == board_content,
            "saved board changed during planning"
        );
        if existed {
            write_atomic_if_unchanged(&path, &old, &new)?;
        } else {
            use std::io::Write;
            let mut f = tempfile::NamedTempFile::new_in(path.parent().context("no parent")?)?;
            f.write_all(new.as_bytes())?;
            f.as_file().sync_all()?;
            f.persist_noclobber(&path)?;
        }
        ensure!(read_consistent(&path)? == new, "rule file readback differs");
    }
    Ok(CallToolResult::json(
        &json!({"status":if a.dry_run{"ready"}else{"complete"},"plan_revision":rev,"rules_file":path,"rules":a.rules,"before":old,"after":new,"geometry_unchanged":true,"native_drc_required":true}),
    ))
}
#[cfg(test)]
mod tests {
    use super::*;
    fn rule() -> Rule {
        Rule {
            name: "connector_edge".into(),
            references: vec!["H1".into(), "H2".into()],
            constraint: Constraint::EdgeClearance,
            minimum_mm: 0.25,
            rationale: "Routed fabrication contract".into(),
        }
    }
    #[test]
    fn scoped_rules_reject_unbounded_and_injected_conditions() {
        for refs in [
            vec![],
            vec!["*".into()],
            vec!["H1') || 1".into()],
            vec!["H1".into(), "H1".into()],
        ] {
            let mut r = rule();
            r.references = refs;
            assert!(render(&r).is_err());
        }
        let mut r = rule();
        r.minimum_mm = 0.;
        assert!(render(&r).is_err());
    }
    #[test]
    fn scoped_rules_preserve_other_rules_and_are_idempotent() {
        let old = "(version 1)\n(rule \"user\" (constraint track_width (min 0.2mm)))\n";
        let r = rule();
        let first = candidate(old, std::slice::from_ref(&r)).unwrap();
        assert!(first.contains("(rule \"user\" (constraint track_width (min 0.2mm)))"));
        assert_eq!(first, candidate(&first, &[r]).unwrap());
        assert!(first.contains("A.Type == 'Pad'"));
    }
    #[test]
    fn scoped_holes_match_only_same_footprint() {
        let mut r = rule();
        r.constraint = Constraint::HoleClearance;
        let (_, s) = render(&r).unwrap();
        assert!(s.contains("(A.memberOfFootprint('H1') && B.memberOfFootprint('H1'))"));
        assert!(s.contains("B.Type == 'Pad'"));
    }
    #[test]
    fn scoped_revision_binds_board_rule_preimage_and_candidate() {
        let p = Path::new("a.kicad_pcb");
        let base = revision(p, "b", "old", "new");
        for (b, o, n) in [("x", "old", "new"), ("b", "x", "new"), ("b", "old", "x")] {
            assert_ne!(base, revision(p, b, o, n));
        }
    }
    #[tokio::test]
    async fn scoped_apply_without_revision_refuses_before_access() {
        assert!(
            handle(&json!({"board":"absent.kicad_pcb","rules":[rule()],"dry_run":false}))
                .await
                .is_err()
        );
    }
}
