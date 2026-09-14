//! UUID-targeted editing of plain schematic annotations; labels and fields are excluded.

use crate::mcp::{error::ToolErrorKind, protocol::CallToolResult};
use crate::tool;
use crate::tools::{get_path, require_str, ToolContext, ToolDef};
use konnect_sexp::{
    commit_command, parse_sexp, prepare_command,
    writer::{find_direct_child_blocks, read_consistent},
    DocumentRevision, ItemId, SchematicCommand, SexpError, SexpNode,
};
use serde_json::{json, Value};
use std::path::Path;

pub(super) fn tools() -> Vec<ToolDef> {
    vec![
        tool!(
            "list_schematic_texts",
            "List plain root-level text annotations from the saved schematic, with UUIDs, \
             positions in millimetres and an opaque document revision. Does not include \
             net labels, symbol fields, text boxes or nested library text. The source is \
             the saved file even when KiCad has an unsaved editor open.",
            json!({
                "type": "object", "additionalProperties": false,
                "properties": { "schematic": { "type": "string" } },
                "required": ["schematic"]
            }),
            |args, ctx| async move { handle_list_schematic_texts(args, ctx).await }
        ),
        tool!(
            "edit_schematic_text",
            "Replace one plain annotation's text by saved UUID. Requires the exact revision \
             from list_schematic_texts; refuses stale or editor-owned files. Preserves \
             position, style, UUID, unknown fields and unrelated objects. Escapes multiline \
             contents for KiCad and verifies committed readback. Other annotation changes \
             can use delete_schematic_text followed by add_schematic_text.",
            json!({
                "type": "object", "additionalProperties": false,
                "properties": {
                    "schematic": { "type": "string" },
                    "uuid": { "type": "string", "minLength": 1 },
                    "expected_revision": { "type": "string", "minLength": 1 },
                    "text": { "type": "string" }
                },
                "required": ["schematic", "uuid", "expected_revision", "text"]
            }),
            |args, ctx| async move { handle_edit_schematic_text(args, ctx).await }
        ),
        tool!(
            "delete_schematic_text",
            "Delete one plain annotation by saved UUID and exact list_schematic_texts \
             revision. Refuses stale or editor-owned files and annotations referenced by \
             a group. Preserves unrelated objects and verifies the UUID is absent afterward.",
            json!({
                "type": "object", "additionalProperties": false,
                "properties": {
                    "schematic": { "type": "string" },
                    "uuid": { "type": "string", "minLength": 1 },
                    "expected_revision": { "type": "string", "minLength": 1 }
                },
                "required": ["schematic", "uuid", "expected_revision"]
            }),
            |args, ctx| async move { handle_delete_schematic_text(args, ctx).await }
        ),
    ]
}

fn stale(path: &Path, reason: impl Into<String>) -> CallToolResult {
    let reason = reason.into();
    CallToolResult::error_kind(
        ToolErrorKind::StaleTarget {
            target: path.display().to_string(),
            reason: reason.clone(),
        },
        reason,
    )
}

fn annotation(node: &SexpNode) -> anyhow::Result<Value> {
    let text = match node.get(1) {
        Some(SexpNode::Str(text)) => text,
        _ => anyhow::bail!("annotation contents are not a quoted string"),
    };
    let uuid = node
        .find_str("uuid")
        .filter(|uuid| !uuid.trim().is_empty())
        .ok_or_else(|| anyhow::anyhow!("annotation has no UUID"))?;
    let at = node
        .find("at")
        .ok_or_else(|| anyhow::anyhow!("annotation has no position"))?;
    let coordinate = |index| {
        at.get_f64(index)
            .filter(|number| number.is_finite())
            .ok_or_else(|| anyhow::anyhow!("annotation has an invalid position/rotation"))
    };
    Ok(json!({"uuid": uuid, "text": text, "x": coordinate(1)?,
              "y": coordinate(2)?, "rotation": coordinate(3)?}))
}

fn annotations(path: &Path, content: &str) -> Result<Vec<Value>, CallToolResult> {
    let tree = parse_sexp(content).map_err(|error| stale(path, error.to_string()))?;
    if tree.head() != Some("kicad_sch") {
        return Err(stale(path, "expected exactly one kicad_sch document"));
    }
    // Reuse the full top-level UUID index, so a text/wire duplicate is also ambiguous.
    super::sch_components::indexed_uuid_items(path, content)
        .map_err(|error| error.into_result())?;
    tree.find_all("text")
        .into_iter()
        .map(|node| annotation(node).map_err(|error| stale(path, error.to_string())))
        .collect()
}

async fn handle_list_schematic_texts(
    args: &Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let path = get_path(args, "schematic")?;
    let content = read_consistent(&path)?;
    let texts = match annotations(&path, &content) {
        Ok(texts) => texts,
        Err(refusal) => return Ok(refusal),
    };
    Ok(CallToolResult::json(&json!({
        "schematic": path, "source": "saved_file", "revision": DocumentRevision::of(&content).to_string(),
        "count": texts.len(), "texts": texts
    })))
}

fn replace_contents(block: &str, text: &str) -> anyhow::Result<String> {
    let start = block
        .find('"')
        .ok_or_else(|| anyhow::anyhow!("annotation has no quoted contents"))?;
    let mut escaped = false;
    let end = block[start + 1..]
        .char_indices()
        .find_map(|(offset, ch)| {
            if escaped {
                escaped = false;
            } else if ch == '\\' {
                escaped = true;
            } else if ch == '"' {
                return Some(start + 1 + offset);
            }
            None
        })
        .ok_or_else(|| anyhow::anyhow!("annotation contents are unterminated"))?;
    let quoted = text
        .replace('\\', "\\\\")
        .replace('"', "\\\"")
        .replace('\n', "\\n")
        .replace('\r', "\\r")
        .replace('\t', "\\t");
    Ok(format!(
        "{}{}{}",
        &block[..start + 1],
        quoted,
        &block[end..]
    ))
}

struct TextPlan {
    command: SchematicCommand,
    candidate: String,
    uuid: String,
    operation: &'static str,
}

fn plan_text(
    path: &Path,
    content: &str,
    uuid: &str,
    revision: &str,
    text: Option<&str>,
) -> Result<TextPlan, CallToolResult> {
    if revision != DocumentRevision::of(content).to_string() {
        return Err(stale(
            path,
            "the saved schematic revision changed; list annotations again",
        ));
    }
    let before = annotations(path, content)?;
    if !before
        .iter()
        .any(|item| item["uuid"].as_str() == Some(uuid))
    {
        return Err(stale(path, format!("no plain annotation has UUID {uuid}")));
    }
    let tree = parse_sexp(content).map_err(|error| stale(path, error.to_string()))?;
    if text.is_none()
        && tree.find_all("group").iter().any(|group| {
            group
                .find("members")
                .and_then(SexpNode::children)
                .unwrap_or(&[])
                .iter()
                .skip(1)
                .any(|member| member.as_str() == Some(uuid))
        })
    {
        return Err(stale(
            path,
            "annotation belongs to a group; remove group membership before deleting it",
        ));
    }
    let id = ItemId::new(uuid).map_err(|error| stale(path, error.to_string()))?;
    let operation = if text.is_some() {
        "edit_schematic_text"
    } else {
        "delete_schematic_text"
    };
    let command = if let Some(text) = text {
        let block = find_direct_child_blocks(content, "kicad_sch")
            .into_iter()
            .find_map(|(start, end)| {
                let node = parse_sexp(&content[start..end]).ok()?;
                (node.head() == Some("text") && node.find_str("uuid") == Some(uuid))
                    .then_some(&content[start..end])
            })
            .ok_or_else(|| stale(path, "annotation block could not be resolved"))?;
        let replacement =
            replace_contents(block, text).map_err(|error| stale(path, error.to_string()))?;
        SchematicCommand::replace_item(content, id, replacement, operation)
    } else {
        SchematicCommand::delete_items(content, [id], operation)
    }
    .map_err(|error| stale(path, error.to_string()))?
    .requiring_unchanged_document();
    let candidate = prepare_command(path, content, &command)
        .map_err(|error| stale(path, error.to_string()))?
        .0;
    let resulting = annotations(path, &candidate)?;
    let actual = resulting
        .iter()
        .find(|item| item["uuid"].as_str() == Some(uuid));
    if actual.and_then(|item| item["text"].as_str()) != text {
        return Err(stale(
            path,
            "prospective annotation contents did not match the requested result",
        ));
    }
    Ok(TextPlan {
        command,
        candidate,
        uuid: uuid.to_owned(),
        operation,
    })
}

fn uncertain(path: &Path, operation: &str, reason: impl Into<String>) -> CallToolResult {
    CallToolResult::error_kind(
        ToolErrorKind::MutationOutcomeUncertain {
            operation: operation.to_owned(), path: path.display().to_string(), reason: reason.into(),
        },
        "Annotation mutation may have applied; list the saved annotations and inspect before retrying.",
    )
}

fn commit_text(path: &Path, plan: TextPlan) -> CallToolResult {
    commit_text_with_readback(path, plan, || read_consistent(path))
}

fn commit_text_with_readback(
    path: &Path,
    plan: TextPlan,
    readback: impl FnOnce() -> Result<String, SexpError>,
) -> CallToolResult {
    if let Err(error) = commit_command(path, &plan.command) {
        return match error {
            SexpError::KiCadEditorLocked { .. } | SexpError::ItemConflict { .. } => {
                stale(path, error.to_string())
            }
            // A generic conflict may also follow persistence. Never promise rollback.
            _ => uncertain(path, plan.operation, error.to_string()),
        };
    }
    let committed = match readback() {
        Ok(committed) => committed,
        Err(error) => return uncertain(path, plan.operation, error.to_string()),
    };
    if committed != plan.candidate {
        return uncertain(
            path,
            plan.operation,
            "committed file does not match the prepared document",
        );
    }
    let texts = match annotations(path, &committed) {
        Ok(texts) => texts,
        Err(_) => {
            return uncertain(
                path,
                plan.operation,
                "committed annotations could not be decoded",
            )
        }
    };
    let annotation = texts
        .iter()
        .find(|item| item["uuid"].as_str() == Some(&plan.uuid));
    CallToolResult::json(&json!({
        "schematic": path, "source": "saved_file_readback", "operation": plan.operation,
        "revision": DocumentRevision::of(&committed).to_string(), "uuid": plan.uuid,
        "annotation": annotation, "deleted": annotation.is_none(), "count": texts.len()
    }))
}

async fn handle_edit_schematic_text(
    args: &Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let text = match require_str(args, "text") {
        Ok(value) => value,
        Err(error) => return Ok(error),
    };
    if text
        .chars()
        .any(|ch| ch.is_control() && !['\n', '\r', '\t'].contains(&ch))
    {
        return Ok(CallToolResult::error_kind(
            ToolErrorKind::InvalidArgument {
                field: "text".into(),
                reason: "unsupported control character".into(),
            },
            "Only newline, carriage return and tab control characters are supported.",
        ));
    }
    handle_mutate_text(args, Some(text)).await
}

async fn handle_delete_schematic_text(
    args: &Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    handle_mutate_text(args, None).await
}

async fn handle_mutate_text(args: &Value, text: Option<&str>) -> anyhow::Result<CallToolResult> {
    let path = get_path(args, "schematic")?;
    let uuid = match require_str(args, "uuid") {
        Ok(value) => value,
        Err(error) => return Ok(error),
    };
    let revision = match require_str(args, "expected_revision") {
        Ok(value) => value,
        Err(error) => return Ok(error),
    };
    for (field, value) in [("uuid", uuid), ("expected_revision", revision)] {
        if value.trim().is_empty() {
            return Ok(CallToolResult::error_kind(
                ToolErrorKind::InvalidArgument {
                    field: field.into(),
                    reason: "must not be empty".into(),
                },
                format!("{field} must not be empty"),
            ));
        }
    }
    let content = read_consistent(&path)?;
    let plan = match plan_text(&path, &content, uuid, revision, text) {
        Ok(plan) => plan,
        Err(refusal) => return Ok(refusal),
    };
    Ok(commit_text(&path, plan))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mcp::{error::extract_error_kind, handler::McpHandler, protocol::ToolContent};
    use crate::tools::ServerConfig;
    use std::sync::Arc;

    // Existing KiCad 10 demo fixture, including its CRLF formatting and styled note.
    const SAVED: &str = include_str!("../../tests/fixtures/project_ownership/ampli_ht.kicad_sch");
    const TEXT_UUID: &str = "4fee597b-5b3d-4d5f-9246-cb5aaa802827";
    const WIRE_UUID: &str = "0dfdfa9f-1e3f-4e14-b64b-12bde76a80c7";

    fn config() -> ServerConfig {
        ServerConfig {
            kicad_cli: String::new(),
            kicad_binary: String::new(),
            ipc_address: String::new(),
            project_dir: None,
            jlcpcb_db_path: None,
            auto_load_toolsets: true,
            eager_toolsets: false,
        }
    }

    fn context() -> ToolContext {
        ToolContext::new(config(), Arc::new(crate::router::ToolRouter::new()))
    }

    fn fixture(content: &str) -> (tempfile::TempDir, std::path::PathBuf) {
        let directory = tempfile::tempdir().unwrap();
        let path = directory.path().join("annotation.kicad_sch");
        std::fs::write(&path, content).unwrap();
        (directory, path)
    }

    fn body(result: &CallToolResult) -> Value {
        match &result.content[0] {
            ToolContent::Text { text } => serde_json::from_str(text).unwrap(),
            _ => panic!("expected JSON text"),
        }
    }

    fn args(path: &Path) -> Value {
        json!({"schematic": path, "uuid": TEXT_UUID,
               "expected_revision": DocumentRevision::of(SAVED).to_string()})
    }

    #[tokio::test]
    async fn list_reads_native_plain_annotations_without_changing_saved_bytes() {
        let (_directory, path) = fixture(SAVED);
        let result = handle_list_schematic_texts(&json!({"schematic": path}), &context())
            .await
            .unwrap();
        let actual = body(&result);
        assert_eq!(actual["source"], "saved_file");
        assert_eq!(actual["count"], 1);
        assert_eq!(
            actual["texts"][0],
            json!({"uuid": TEXT_UUID, "text": "Filter:\nFc =1000Hz",
            "x": 66.04, "y": 74.93, "rotation": 0.0})
        );
        assert_eq!(std::fs::read_to_string(path).unwrap(), SAVED);
    }

    #[tokio::test]
    async fn edit_round_trips_multiline_content_and_preserves_every_other_byte() {
        let (_directory, path) = fixture(SAVED);
        let text = "VPW\n\"CAN\"\tC:\\new\\test\rΩ (text nested)";
        let mut request = args(&path);
        request["text"] = json!(text);
        let actual = body(
            &handle_edit_schematic_text(&request, &context())
                .await
                .unwrap(),
        );
        assert_eq!(actual["source"], "saved_file_readback");
        assert_eq!(actual["annotation"]["text"], text);
        assert_eq!(actual["deleted"], false);
        let expected = SAVED.replacen(
            "Filter:\\nFc =1000Hz",
            "VPW\\n\\\"CAN\\\"\\tC:\\\\new\\\\test\\rΩ (text nested)",
            1,
        );
        assert_eq!(std::fs::read_to_string(path).unwrap(), expected);
        assert_ne!(actual["revision"], request["expected_revision"]);
    }

    #[tokio::test]
    async fn delete_removes_only_the_selected_annotation() {
        let (_directory, path) = fixture(SAVED);
        let actual = body(
            &handle_delete_schematic_text(&args(&path), &context())
                .await
                .unwrap(),
        );
        assert_eq!(actual["deleted"], true);
        assert_eq!(actual["count"], 0);
        let mut expected = parse_sexp(SAVED).unwrap();
        if let SexpNode::List(children) = &mut expected {
            children.retain(|child| child.head() != Some("text"));
        }
        assert_eq!(
            parse_sexp(&std::fs::read_to_string(path).unwrap()).unwrap(),
            expected
        );
    }

    #[tokio::test]
    async fn stale_revision_and_non_text_uuid_refuse_without_mutation() {
        let (_directory, path) = fixture(SAVED);
        for (field, value) in [
            ("expected_revision", "stale"),
            ("uuid", WIRE_UUID),
            ("uuid", "missing"),
        ] {
            let mut request = args(&path);
            request[field] = json!(value);
            let result = handle_delete_schematic_text(&request, &context())
                .await
                .unwrap();
            assert_eq!(extract_error_kind(&result).as_deref(), Some("stale_target"));
            assert_eq!(std::fs::read_to_string(&path).unwrap(), SAVED);
        }
    }

    #[tokio::test]
    async fn duplicate_uuid_is_ambiguous_even_across_item_types() {
        let original = SAVED.replace(WIRE_UUID, TEXT_UUID);
        let (_directory, path) = fixture(&original);
        let mut request = args(&path);
        request["expected_revision"] = json!(DocumentRevision::of(&original).to_string());
        let result = handle_delete_schematic_text(&request, &context())
            .await
            .unwrap();
        assert_eq!(
            extract_error_kind(&result).as_deref(),
            Some("ambiguous_target")
        );
        assert_eq!(std::fs::read_to_string(path).unwrap(), original);
    }

    #[tokio::test]
    async fn grouped_annotation_cannot_be_deleted_but_its_text_can_be_edited() {
        let group = format!("(group \"notes\" (uuid \"0f4e109f-98e4-4994-84aa-9992bdf207db\") (members \"{TEXT_UUID}\"))\n\t(text ");
        let original = SAVED.replacen("(text ", &group, 1);
        let (_directory, path) = fixture(&original);
        let mut request = args(&path);
        request["expected_revision"] = json!(DocumentRevision::of(&original).to_string());
        let result = handle_delete_schematic_text(&request, &context())
            .await
            .unwrap();
        assert_eq!(extract_error_kind(&result).as_deref(), Some("stale_target"));
        assert_eq!(std::fs::read_to_string(&path).unwrap(), original);
        request["text"] = json!("updated group member");
        let edited = handle_edit_schematic_text(&request, &context())
            .await
            .unwrap();
        assert_eq!(body(&edited)["annotation"]["text"], "updated group member");
        assert!(std::fs::read_to_string(path)
            .unwrap()
            .contains(&format!("(members \"{TEXT_UUID}\")")));
    }

    #[tokio::test]
    async fn kicad_editor_lock_refuses_edit_and_delete_without_writing() {
        let (_directory, path) = fixture(SAVED);
        std::fs::write(
            konnect_sexp::writer::kicad_editor_lock_path(&path).unwrap(),
            "owned",
        )
        .unwrap();
        let mut request = args(&path);
        request["text"] = json!("new note");
        let edit = handle_edit_schematic_text(&request, &context())
            .await
            .unwrap();
        let delete = handle_delete_schematic_text(&request, &context())
            .await
            .unwrap();
        assert_eq!(extract_error_kind(&edit).as_deref(), Some("stale_target"));
        assert_eq!(extract_error_kind(&delete).as_deref(), Some("stale_target"));
        assert_eq!(std::fs::read_to_string(path).unwrap(), SAVED);
    }

    #[test]
    fn concurrent_document_change_cannot_be_overwritten_by_prepared_command() {
        let (_directory, path) = fixture(SAVED);
        let plan = plan_text(
            &path,
            SAVED,
            TEXT_UUID,
            &DocumentRevision::of(SAVED).to_string(),
            Some("planned"),
        )
        .unwrap();
        let newer = SAVED.replace("(paper \"A4\")", "(paper \"A3\")");
        assert_ne!(newer, SAVED);
        std::fs::write(&path, &newer).unwrap();
        let result = commit_text(&path, plan);
        assert!(result.is_error);
        assert_eq!(std::fs::read_to_string(path).unwrap(), newer);
    }

    #[test]
    fn post_write_readback_failure_is_uncertain_and_does_not_claim_rollback() {
        let (_directory, path) = fixture(SAVED);
        let plan = plan_text(
            &path,
            SAVED,
            TEXT_UUID,
            &DocumentRevision::of(SAVED).to_string(),
            Some("committed"),
        )
        .unwrap();
        let result = commit_text_with_readback(&path, plan, || {
            Err(SexpError::InvalidValue("unavailable readback".into()))
        });
        assert_eq!(
            extract_error_kind(&result).as_deref(),
            Some("mutation_outcome_uncertain")
        );
        let actual = annotations(&path, &std::fs::read_to_string(&path).unwrap()).unwrap();
        assert_eq!(actual[0]["text"], "committed");
    }

    #[test]
    fn post_write_readback_mismatch_is_uncertain() {
        let (_directory, path) = fixture(SAVED);
        let plan = plan_text(
            &path,
            SAVED,
            TEXT_UUID,
            &DocumentRevision::of(SAVED).to_string(),
            Some("committed"),
        )
        .unwrap();
        let result = commit_text_with_readback(&path, plan, || Ok(SAVED.to_owned()));
        assert_eq!(
            extract_error_kind(&result).as_deref(),
            Some("mutation_outcome_uncertain")
        );
        let actual = annotations(&path, &std::fs::read_to_string(&path).unwrap()).unwrap();
        assert_eq!(actual[0]["text"], "committed");
    }

    #[tokio::test]
    async fn unsupported_control_character_and_wrong_types_refuse_before_write() {
        let (_directory, path) = fixture(SAVED);
        for value in [json!("bad\u{0000}note"), json!(42), Value::Null] {
            let mut request = args(&path);
            request["text"] = value;
            let result = handle_edit_schematic_text(&request, &context())
                .await
                .unwrap();
            assert_eq!(
                extract_error_kind(&result).as_deref(),
                Some("invalid_argument")
            );
            assert_eq!(std::fs::read_to_string(&path).unwrap(), SAVED);
        }
    }

    #[tokio::test]
    async fn served_dispatch_exposes_tools_and_enforces_their_schemas() {
        let (_directory, path) = fixture(SAVED);
        let handler = McpHandler::new(config()).await.unwrap();
        let invoke = |name: &str, arguments: Value| {
            json!({"jsonrpc": "2.0", "id": 1,
            "method": "tools/call", "params": {"name": name, "arguments": arguments}})
        };
        let response = handler
            .handle_message(invoke("list_schematic_texts", json!({"schematic": path})))
            .await
            .unwrap()
            .result
            .unwrap();
        let listed: Value =
            serde_json::from_str(response["content"][0]["text"].as_str().unwrap()).unwrap();
        assert_eq!(listed["count"], 1);
        let mut request = args(&path);
        request["text"] = json!("served update");
        request["unexpected"] = json!(true);
        let bad = handler
            .handle_message(invoke("edit_schematic_text", request.clone()))
            .await
            .unwrap()
            .result
            .unwrap();
        assert_eq!(bad["isError"], true);
        assert_eq!(std::fs::read_to_string(&path).unwrap(), SAVED);
        request.as_object_mut().unwrap().remove("unexpected");
        let good = handler
            .handle_message(invoke("edit_schematic_text", request))
            .await
            .unwrap()
            .result
            .unwrap();
        let edited: Value =
            serde_json::from_str(good["content"][0]["text"].as_str().unwrap()).unwrap();
        assert_eq!(edited["annotation"]["text"], "served update");
        assert_eq!(edited["source"], "saved_file_readback");
    }
}
