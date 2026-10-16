//! `set_custom_rule` / `get_custom_rules` — named, conditional rules in the
//! board's sibling `.kicad_dru`.
//!
//! KiCad's board constraints are absolute floors: no netclass or rule can ask
//! for less. An exception — smaller vias under one BGA, say — therefore needs
//! the floor lowered *and* a conditional rule holding every other item to the
//! stronger value. `set_layer_constraints` writes only per-layer minima, so
//! this writes any named rule with an optional condition and layer, replacing
//! a rule of the same name in place, and reads the file back for inspection.
//! KiCad parses the file at every DRC; run `run_drc` after a change, since only
//! KiCad can evaluate a condition expression.

use crate::mcp::protocol::CallToolResult;
use crate::tool;
use crate::tools::verification::{named_rule_range, upsert_named_rule};
use crate::tools::{get_path, invalid_arg, ToolContext, ToolDef};
use konnect_ipc::builders;
use konnect_ipc::gen::kiapi::board::types::BoardLayer;
use konnect_sexp::writer::write_atomic;
use serde_json::json;

/// Numeric KiCad 10 rule constraints, all taking lengths in mm.
const CONSTRAINTS: &[&str] = &[
    "annular_width",
    "clearance",
    "connection_width",
    "courtyard_clearance",
    "diff_pair_gap",
    "diff_pair_uncoupled",
    "edge_clearance",
    "hole_clearance",
    "hole_size",
    "hole_to_hole",
    "length",
    "physical_clearance",
    "physical_hole_clearance",
    "silk_clearance",
    "skew",
    "text_height",
    "text_thickness",
    "thermal_relief_gap",
    "thermal_spoke_width",
    "track_width",
    "via_diameter",
];

#[derive(Debug, Clone, PartialEq)]
struct Constraint {
    kind: String,
    min: Option<f64>,
    opt: Option<f64>,
    max: Option<f64>,
}

#[derive(Debug, Clone, PartialEq)]
struct Rule {
    name: String,
    layer: Option<String>,
    condition: Option<String>,
    constraints: Vec<Constraint>,
}

fn mm(value: f64) -> String {
    let text = format!("{value:.6}");
    let text = text.trim_end_matches('0').trim_end_matches('.');
    format!("{text}mm")
}

impl Rule {
    fn text(&self) -> String {
        let mut out = format!("(rule \"{}\"", self.name);
        if let Some(layer) = &self.layer {
            if layer == "outer" || layer == "inner" {
                out.push_str(&format!("\n  (layer {layer})"));
            } else {
                out.push_str(&format!("\n  (layer \"{layer}\")"));
            }
        }
        if let Some(condition) = &self.condition {
            out.push_str(&format!("\n  (condition \"{condition}\")"));
        }
        for c in &self.constraints {
            out.push_str(&format!("\n  (constraint {}", c.kind));
            for (key, value) in [("min", c.min), ("opt", c.opt), ("max", c.max)] {
                if let Some(v) = value {
                    out.push_str(&format!(" ({key} {})", mm(v)));
                }
            }
            out.push(')');
        }
        out.push(')');
        out
    }
}

type Refusal = (String, String);

fn refuse(field: impl Into<String>, reason: impl Into<String>) -> Refusal {
    (field.into(), reason.into())
}

fn rule_name(args: &serde_json::Value) -> Result<String, Refusal> {
    let Some(name) = args.get("name").and_then(serde_json::Value::as_str) else {
        return Err(refuse("name", "is required"));
    };
    let ok = !name.is_empty()
        && name.len() <= 100
        && name
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || " _-.:/+".contains(c));
    if !ok {
        return Err(refuse(
            "name",
            "must be 1-100 characters of letters, digits, space and _-.:/+",
        ));
    }
    Ok(name.to_string())
}

fn parse(args: &serde_json::Value) -> Result<Rule, Refusal> {
    let name = rule_name(args)?;
    let layer = match args.get("layer").filter(|v| !v.is_null()) {
        None => None,
        Some(value) => {
            let Some(layer) = value.as_str() else {
                return Err(refuse("layer", "must be a string"));
            };
            if layer != "outer"
                && layer != "inner"
                && builders::layer_from_name(layer) == BoardLayer::BlUndefined
            {
                return Err(refuse(
                    "layer",
                    format!("'{layer}' is not 'outer', 'inner' or a KiCad board layer name"),
                ));
            }
            Some(layer.to_string())
        }
    };
    let condition = match args.get("condition").filter(|v| !v.is_null()) {
        None => None,
        Some(value) => {
            let Some(condition) = value.as_str().map(str::trim) else {
                return Err(refuse("condition", "must be a string"));
            };
            if condition.is_empty() {
                None
            } else if condition.contains('"')
                || condition.contains('\\')
                || condition.contains('\n')
            {
                return Err(refuse(
                    "condition",
                    "must use single quotes for strings and contain no double quote, backslash or newline",
                ));
            } else if condition.matches('(').count() != condition.matches(')').count() {
                return Err(refuse("condition", "has unbalanced parentheses"));
            } else {
                Some(condition.to_string())
            }
        }
    };
    let Some(items) = args
        .get("constraints")
        .and_then(serde_json::Value::as_array)
    else {
        return Err(refuse("constraints", "must be an array"));
    };
    if items.is_empty() || items.len() > 32 {
        return Err(refuse("constraints", "must hold 1 to 32 constraints"));
    }
    let mut constraints = Vec::new();
    for (index, item) in items.iter().enumerate() {
        let at = format!("constraints[{index}]");
        let Some(object) = item.as_object() else {
            return Err(refuse(at, "must be an object"));
        };
        if let Some(key) = object
            .keys()
            .find(|k| !["type", "min", "opt", "max"].contains(&k.as_str()))
        {
            return Err(refuse(
                format!("{at}.{key}"),
                "is not type, min, opt or max",
            ));
        }
        let kind = match object.get("type").and_then(serde_json::Value::as_str) {
            Some(kind) if CONSTRAINTS.contains(&kind) => kind.to_string(),
            _ => {
                return Err(refuse(
                    format!("{at}.type"),
                    format!("must be one of {}", CONSTRAINTS.join(", ")),
                ))
            }
        };
        let bound = |key: &str| -> Result<Option<f64>, Refusal> {
            match object.get(key).filter(|v| !v.is_null()) {
                None => Ok(None),
                Some(value) => match value.as_f64() {
                    Some(v) if (0.0..=1000.0).contains(&v) => Ok(Some(v)),
                    _ => Err(refuse(
                        format!("{at}.{key}"),
                        "must be a number of mm from 0 to 1000",
                    )),
                },
            }
        };
        let (min, opt, max) = (bound("min")?, bound("opt")?, bound("max")?);
        if min.is_none() && opt.is_none() && max.is_none() {
            return Err(refuse(at, "needs at least one of min, opt and max"));
        }
        let ordered = |a: Option<f64>, b: Option<f64>| match (a, b) {
            (Some(a), Some(b)) => a <= b,
            _ => true,
        };
        if !(ordered(min, opt) && ordered(opt, max) && ordered(min, max)) {
            return Err(refuse(at, "must keep min <= opt <= max"));
        }
        constraints.push(Constraint {
            kind,
            min,
            opt,
            max,
        });
    }
    Ok(Rule {
        name,
        layer,
        condition,
        constraints,
    })
}

/// Every top-level `(rule "…" …)` in the file, by name, in order.
fn rules_in(content: &str) -> Vec<(String, String)> {
    let mut out = Vec::new();
    let mut from = 0;
    while let Some(found) = content[from..].find("(rule \"") {
        let start = from + found;
        let name_start = start + "(rule \"".len();
        let Some(name_len) = content[name_start..].find('"') else {
            break;
        };
        let name = content[name_start..name_start + name_len].to_string();
        match named_rule_range(&content[start..], &name) {
            Some((s, e)) => {
                out.push((name, content[start + s..start + e].to_string()));
                from = start + e;
            }
            None => break,
        }
    }
    out
}

fn remove_rule(content: &str, name: &str) -> Option<String> {
    let (start, end) = named_rule_range(content, name)?;
    let head = content[..start].trim_end();
    let tail = content[end..].trim_start();
    let mut out = head.to_string();
    if !tail.is_empty() {
        out.push_str("\n\n");
        out.push_str(tail);
    }
    if !out.ends_with('\n') {
        out.push('\n');
    }
    Some(out)
}

pub(crate) fn set_tool() -> ToolDef {
    let bound = json!({ "type": "number", "minimum": 0, "maximum": 1000 });
    tool!(
        "set_custom_rule",
        "Create, replace or remove one named rule in the board's sibling .kicad_dru custom-rules \
         file: an optional KiCad condition expression (single-quoted strings, e.g. \
         \"A.Type == 'Via' && !A.intersectsCourtyard('U5801')\"), an optional layer ('outer', \
         'inner' or a layer name) and numeric constraints in mm (min/opt/max). A rule of the same \
         name is replaced in place; 'remove' deletes it. Board constraints remain absolute floors \
         that no rule can go below, so an exception needs the floor lowered (set_design_rules) and \
         a rule holding everything else to the stronger value. Konnect checks the rule's shape, \
         not the condition's meaning: run run_drc afterwards, which makes KiCad parse the file.",
        json!({
            "type": "object",
            "properties": {
                "board": { "type": "string", "description": "Path to .kicad_pcb file; the sibling .kicad_dru is edited" },
                "name": { "type": "string", "description": "Rule name, unique in the file" },
                "condition": { "type": "string", "description": "KiCad DRC condition expression; omit for an unconditional rule" },
                "layer": { "type": "string", "description": "'outer', 'inner' or a board layer name; omit for every layer" },
                "constraints": {
                    "type": "array",
                    "minItems": 1,
                    "maxItems": 32,
                    "items": {
                        "type": "object",
                        "additionalProperties": false,
                        "properties": {
                            "type": { "type": "string", "enum": CONSTRAINTS },
                            "min": bound.clone(), "opt": bound.clone(), "max": bound
                        },
                        "required": ["type"]
                    }
                },
                "remove": { "type": "boolean", "default": false, "description": "Delete the named rule instead" }
            },
            "required": ["board", "name"]
        }),
        |args, ctx| async move { handle_set_custom_rule(args, ctx).await }
    )
}

pub(crate) fn get_tool() -> ToolDef {
    tool!(
        "get_custom_rules",
        "Read the board's sibling .kicad_dru custom-rules file: whether it exists, its version \
         header, and every named rule with its full text, in file order. Read-only.",
        json!({
            "type": "object",
            "properties": {
                "board": { "type": "string", "description": "Path to .kicad_pcb file" }
            },
            "required": ["board"]
        }),
        |args, ctx| async move { handle_get_custom_rules(args, ctx).await }
    )
}

fn rules_path(board: &std::path::Path) -> std::path::PathBuf {
    board.with_extension("kicad_dru")
}

async fn read_rules(path: &std::path::Path) -> anyhow::Result<Option<String>> {
    match tokio::fs::read_to_string(path).await {
        Ok(content) => Ok(Some(content)),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(None),
        Err(error) => Err(error.into()),
    }
}

async fn handle_set_custom_rule(
    args: &serde_json::Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let board = get_path(args, "board")?;
    let path = rules_path(&board);
    let existing = read_rules(&path).await?;
    if args.get("remove").and_then(serde_json::Value::as_bool) == Some(true) {
        let name = match rule_name(args) {
            Ok(name) => name,
            Err((field, reason)) => return Ok(invalid_arg(&field, &reason)),
        };
        let removed = existing
            .as_deref()
            .and_then(|content| remove_rule(content, &name));
        if let Some(content) = &removed {
            write_atomic(&path, content)?;
        }
        return Ok(CallToolResult::json(&json!({
            "rules_file": path.display().to_string(),
            "name": name,
            "removed": removed.is_some(),
        })));
    }
    let rule = match parse(args) {
        Ok(rule) => rule,
        Err((field, reason)) => return Ok(invalid_arg(&field, &reason)),
    };
    let content = existing
        .clone()
        .unwrap_or_else(|| "(version 1)\n".to_string());
    let replaced = named_rule_range(&content, &rule.name).is_some();
    let text = rule.text();
    let updated = upsert_named_rule(&content, &rule.name, &text);
    let changed = existing.as_deref() != Some(updated.as_str());
    if changed {
        write_atomic(&path, &updated)?;
    }
    Ok(CallToolResult::json(&json!({
        "rules_file": path.display().to_string(),
        "name": rule.name,
        "status": if !changed { "unchanged" } else if replaced { "replaced" } else { "added" },
        "rule": text,
        "next": "Run run_drc: KiCad parses the rules file there and reports a syntax or expression error.",
    })))
}

async fn handle_get_custom_rules(
    args: &serde_json::Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let board = get_path(args, "board")?;
    let path = rules_path(&board);
    let Some(content) = read_rules(&path).await? else {
        return Ok(CallToolResult::json(&json!({
            "rules_file": path.display().to_string(),
            "exists": false,
            "rules": [],
        })));
    };
    let version = content.find("(version ").and_then(|at| {
        let rest = &content[at + "(version ".len()..];
        rest.find(')').map(|end| rest[..end].trim().to_string())
    });
    let rules: Vec<serde_json::Value> = rules_in(&content)
        .into_iter()
        .map(|(name, text)| json!({ "name": name, "text": text }))
        .collect();
    Ok(CallToolResult::json(&json!({
        "rules_file": path.display().to_string(),
        "exists": true,
        "version": version,
        "count": rules.len(),
        "rules": rules,
    })))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rule(extra: serde_json::Value) -> serde_json::Value {
        let mut base = json!({
            "board": "b.kicad_pcb",
            "name": "ordinary_via",
            "condition": "A.Type == 'Via' && !A.intersectsCourtyard('U5801')",
            "constraints": [
                { "type": "via_diameter", "min": 0.45 },
                { "type": "hole_size", "min": 0.2 }
            ]
        });
        for (k, v) in extra.as_object().unwrap() {
            base[k] = v.clone();
        }
        base
    }

    #[test]
    fn rule_text_is_kicad_dru_syntax() {
        let r = parse(&rule(json!({ "layer": "outer" }))).unwrap();
        assert_eq!(
            r.text(),
            "(rule \"ordinary_via\"\n  (layer outer)\n  (condition \"A.Type == 'Via' && \
             !A.intersectsCourtyard('U5801')\")\n  (constraint via_diameter (min 0.45mm))\n  \
             (constraint hole_size (min 0.2mm)))"
        );
        let named = parse(&rule(json!({ "layer": "In2.Cu", "constraints": [
            { "type": "track_width", "min": 0.09, "opt": 0.1064, "max": 0.2 }
        ] })))
        .unwrap();
        assert!(named.text().contains("(layer \"In2.Cu\")"));
        assert!(named
            .text()
            .contains("(constraint track_width (min 0.09mm) (opt 0.1064mm) (max 0.2mm))"));
        konnect_sexp::parser::parse_sexp(&named.text()).expect("balanced S-expression");
    }

    #[test]
    fn malformed_rules_are_refused_by_field() {
        let field = |extra| parse(&rule(extra)).unwrap_err().0;
        assert_eq!(field(json!({ "name": "bad\"name" })), "name");
        assert_eq!(field(json!({ "layer": "F.Copper" })), "layer");
        assert_eq!(
            field(json!({ "condition": "A.NetName == \"GND\"" })),
            "condition"
        );
        assert_eq!(
            field(json!({ "condition": "A.Type == 'Via' && (B" })),
            "condition"
        );
        assert_eq!(field(json!({ "constraints": [] })), "constraints");
        assert_eq!(
            field(json!({ "constraints": [{ "type": "via_count", "min": 1 }] })),
            "constraints[0].type"
        );
        assert_eq!(
            field(json!({ "constraints": [{ "type": "clearance" }] })),
            "constraints[0]"
        );
        assert_eq!(
            field(json!({ "constraints": [{ "type": "clearance", "min": 0.3, "max": 0.2 }] })),
            "constraints[0]"
        );
        assert_eq!(
            field(json!({ "constraints": [{ "type": "clearance", "min": -1 }] })),
            "constraints[0].min"
        );
        assert_eq!(
            field(json!({ "constraints": [{ "type": "clearance", "min": 1, "extra": 2 }] })),
            "constraints[0].extra"
        );
    }

    #[test]
    fn rules_are_listed_and_removed_by_name() {
        let content = "(version 1)\n\n(rule \"a\"\n  (constraint clearance (min 0.2mm)))\n\n\
                       (rule \"b (x)\"\n  (condition \"A.NetName == 'GND'\")\n  (constraint clearance (min 0.3mm)))\n";
        let names: Vec<String> = rules_in(content).into_iter().map(|(n, _)| n).collect();
        assert_eq!(names, ["a", "b (x)"]);
        let without_a = remove_rule(content, "a").unwrap();
        assert!(!without_a.contains("(rule \"a\""));
        assert!(
            without_a.starts_with("(version 1)\n\n(rule \"b (x)\""),
            "{without_a}"
        );
        assert!(remove_rule(content, "c").is_none());
    }

    fn ctx() -> ToolContext {
        crate::tools::pcb_board::board_mock::ctx_talking_to(String::new())
    }

    fn body(r: &CallToolResult) -> serde_json::Value {
        match r.content.first() {
            Some(crate::mcp::protocol::ToolContent::Text { text }) => {
                serde_json::from_str(text).unwrap()
            }
            other => panic!("expected text, got {other:?}"),
        }
    }

    #[tokio::test]
    async fn set_get_replace_and_remove_round_trip() {
        let dir = tempfile::tempdir().unwrap();
        let board = dir.path().join("b.kicad_pcb");
        std::fs::write(&board, "(kicad_pcb)").unwrap();
        let dru = dir.path().join("b.kicad_dru");
        std::fs::write(
            &dru,
            "(version 1)\n(rule \"keep\"\n  (constraint clearance (min 0.1mm)))\n",
        )
        .unwrap();
        let mut args = rule(json!({}));
        args["board"] = json!(board);

        let res = handle_set_custom_rule(&args, &ctx()).await.unwrap();
        assert!(!res.is_error, "{:?}", res.content);
        assert_eq!(body(&res)["status"], "added");
        let res = handle_set_custom_rule(&args, &ctx()).await.unwrap();
        assert_eq!(body(&res)["status"], "unchanged");
        args["constraints"] = json!([{ "type": "via_diameter", "min": 0.5 }]);
        let res = handle_set_custom_rule(&args, &ctx()).await.unwrap();
        assert_eq!(body(&res)["status"], "replaced");

        let listed = body(
            &handle_get_custom_rules(&json!({ "board": board }), &ctx())
                .await
                .unwrap(),
        );
        assert_eq!(listed["version"], "1");
        assert_eq!(listed["count"], 2);
        assert_eq!(listed["rules"][0]["name"], "keep");
        assert!(listed["rules"][1]["text"]
            .as_str()
            .unwrap()
            .contains("(min 0.5mm)"));
        assert_eq!(
            std::fs::read_to_string(&dru)
                .unwrap()
                .matches("(rule \"ordinary_via\"")
                .count(),
            1
        );

        let removed = body(
            &handle_set_custom_rule(
                &json!({ "board": board, "name": "ordinary_via", "remove": true }),
                &ctx(),
            )
            .await
            .unwrap(),
        );
        assert_eq!(removed["removed"], true);
        let text = std::fs::read_to_string(&dru).unwrap();
        assert!(
            text.contains("(rule \"keep\"") && !text.contains("ordinary_via"),
            "{text}"
        );
        konnect_sexp::parser::parse_sexp(&format!("({text})")).unwrap();
    }

    #[tokio::test]
    async fn a_missing_file_reads_empty_and_is_created_on_set() {
        let dir = tempfile::tempdir().unwrap();
        let board = dir.path().join("b.kicad_pcb");
        let listed = body(
            &handle_get_custom_rules(&json!({ "board": board }), &ctx())
                .await
                .unwrap(),
        );
        assert_eq!(listed["exists"], false);
        let mut args = rule(json!({}));
        args["board"] = json!(board);
        let res = handle_set_custom_rule(&args, &ctx()).await.unwrap();
        assert!(!res.is_error);
        let text = std::fs::read_to_string(dir.path().join("b.kicad_dru")).unwrap();
        assert!(
            text.starts_with("(version 1)\n\n(rule \"ordinary_via\""),
            "{text}"
        );
    }
}
