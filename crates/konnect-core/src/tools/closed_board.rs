//! Guards shared by the saved-file board edits that carry a dry-run revision:
//! the physical stackup and placed Reference-field presentation.
//!
//! Each edit plans against the exact saved bytes, hands back a revision that
//! covers those bytes and the normalized request, and applies only that
//! revision. KiCad must provably not hold the board — the live write gate's
//! verdict, then KiCad's own sibling lock — both when the plan is made and
//! again immediately before the write. The write is one atomic replace that
//! fails if the file changed, and what was persisted is read back and checked.

use std::path::Path;

use serde_json::{json, Value};
use sha2::{Digest, Sha256};

use crate::mcp::error::ToolErrorKind;
use crate::mcp::protocol::{CallToolResult, ToolContent};
use crate::tools::live_board::{self, EditorLock};
use crate::tools::pcb_board::{attempt_ipc_write, BoardWrite};
use crate::tools::ToolContext;

pub(crate) fn sha256_hex(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

/// The revision a dry run reports and an apply must repeat: the operation's
/// domain, the exact saved bytes, and the request as normalized for planning.
pub(crate) fn revision(domain: &str, original: &str, normalized_request: &Value) -> String {
    let mut hasher = Sha256::new();
    for part in [
        domain.as_bytes(),
        original.as_bytes(),
        normalized_request.to_string().as_bytes(),
    ] {
        hasher.update((part.len() as u64).to_le_bytes());
        hasher.update(part);
    }
    format!("{:x}", hasher.finalize())
}

fn refusal(kind: ToolErrorKind, message: String) -> CallToolResult {
    CallToolResult::error_kind(kind, message)
}

/// Prove KiCad does not hold `board`, as `closed_initial` does: only the two
/// verdicts that unlock a file edit pass — IPC never reached, or KiCad proved
/// the board is not open — and then the exact board's sibling lock must be
/// absent. Every other answer, including an uncertain or ambiguous one, is a
/// refusal carrying that answer.
pub(crate) async fn prove_closed(
    ctx: &ToolContext,
    board: &Path,
    operation: &str,
) -> anyhow::Result<Result<Value, CallToolResult>> {
    let reason = match attempt_ipc_write(ctx, board, operation, |_| Ok(())).await? {
        BoardWrite::Ipc(()) => {
            return Ok(Err(refusal(
                ToolErrorKind::PlanBlocked {
                    operation: operation.to_string(),
                    reasons: vec!["board_open_in_kicad".to_string()],
                },
                format!(
                    "KiCad holds this exact board open. {operation} edits the saved file and \
                     requires the board closed in KiCad, even for a dry run; save and close it \
                     in the PCB editor and retry. The board file was not modified."
                ),
            )))
        }
        BoardWrite::Refused(result) => {
            // A structured refusal (uncertain IPC, ambiguous document list,
            // lost live board, lock) is returned as KiCad's gate worded it.
            if crate::mcp::error::extract_error_kind(&result)
                .is_some_and(|kind| kind != "handler_error")
            {
                return Ok(Err(result));
            }
            let message = result
                .content
                .into_iter()
                .find_map(|content| match content {
                    ToolContent::Text { text } => Some(text),
                    _ => None,
                })
                .unwrap_or_else(|| "KiCad refused the closure check".to_string());
            return Ok(Err(refusal(
                ToolErrorKind::PlanBlocked {
                    operation: operation.to_string(),
                    reasons: vec!["closure_not_proven".to_string()],
                },
                format!("Closure not proven: {message}. The board file was not modified."),
            )));
        }
        BoardWrite::File(reason) => reason,
    };
    match live_board::editor_lock(board) {
        EditorLock::Absent => {}
        EditorLock::Present(lock) => {
            return Ok(Err(refusal(
                ToolErrorKind::UnsafeFileFallback {
                    path: board.display().to_string(),
                    reason: "kicad_lock_present".to_string(),
                },
                format!(
                    "KiCad's sibling lock '{}' is present, so an editor may still hold this \
                     board. The board file was not modified.",
                    lock.display()
                ),
            )))
        }
        EditorLock::Unreadable(lock, error) => {
            return Ok(Err(refusal(
                ToolErrorKind::UnsafeFileFallback {
                    path: board.display().to_string(),
                    reason: "kicad_lock_unreadable".to_string(),
                },
                format!(
                    "KiCad's sibling lock '{}' could not be inspected ({error}). The board \
                     file was not modified.",
                    lock.display()
                ),
            )))
        }
    }
    Ok(Ok(json!({
        "proven_closed": true,
        "evidence": reason.evidence(),
        "kicad_sibling_lock": "absent",
    })))
}

/// A dry-run revision that no longer matches the saved board or request.
pub(crate) fn stale_revision(board: &Path, operation: &str) -> CallToolResult {
    let mut result = refusal(
        ToolErrorKind::StaleTarget {
            target: board.display().to_string(),
            reason: "stale_revision".to_string(),
        },
        format!(
            "{operation}: expected_revision does not match the current saved board and \
             request. Rerun the dry run and apply its new revision. The board file was not \
             modified."
        ),
    );
    if let Some(ToolContent::Text { text }) = result.content.first_mut() {
        if let Ok(mut body) = serde_json::from_str::<Value>(text) {
            body["current_revision_available_from_dry_run"] = json!(true);
            body["status"] = json!("stale_revision");
            *text = body.to_string();
        }
    }
    result
}

/// Every byte of `candidate` outside the `(start, end)` spans of `original`
/// that `edits` replaced is the original's, in order. `edits` are
/// `(start, end, replacement_len)` against `original`, non-overlapping.
pub(crate) fn only_spans_changed(
    original: &str,
    candidate: &str,
    edits: &[(usize, usize, usize)],
) -> bool {
    let mut edits = edits.to_vec();
    edits.sort_by_key(|edit| edit.0);
    let (mut from, mut to) = (0usize, 0usize);
    for &(start, end, replacement) in &edits {
        if start < from || end < start || end > original.len() {
            return false;
        }
        let kept = &original[from..start];
        if candidate.get(to..to + kept.len()) != Some(kept) {
            return false;
        }
        to += kept.len() + replacement;
        from = end;
    }
    candidate.get(to..) == Some(&original[from..])
}

/// Write `candidate` over `original` once, atomically, then read back the
/// persisted bytes. `verify` checks the persisted text semantically; the
/// exact bytes must equal `candidate`. Any write whose outcome cannot be
/// proven either way is reported as uncertain, never as a refusal.
pub(crate) fn write_and_read_back(
    board: &Path,
    operation: &str,
    original: &str,
    candidate: &str,
    verify: impl Fn(&str) -> Result<Value, String>,
) -> Result<Value, CallToolResult> {
    let uncertain = |reason: String| {
        refusal(
            ToolErrorKind::MutationOutcomeUncertain {
                operation: operation.to_string(),
                path: board.display().to_string(),
                reason: reason.clone(),
            },
            format!(
                "{operation} may or may not have been written: {reason}. Inspect {} before \
                 retrying; this may not be a no-op.",
                board.display()
            ),
        )
    };
    let reported_error = match konnect_sexp::write_atomic_if_unchanged(board, original, candidate) {
        Ok(()) => None,
        Err(error) => match std::fs::read_to_string(board) {
            Ok(now) if now == original => {
                return Err(refusal(
                    ToolErrorKind::StaleTarget {
                        target: board.display().to_string(),
                        reason: "board_write_refused".to_string(),
                    },
                    format!(
                        "{operation} was not written: {error}. The saved board is \
                         byte-identical to the planned input."
                    ),
                ))
            }
            Ok(now) if now == candidate => Some(error.to_string()),
            _ => {
                return Err(uncertain(format!(
                    "the atomic writer reported '{error}', and the board now matches neither \
                     the planned input nor its result"
                )))
            }
        },
    };
    let persisted = konnect_sexp::read_consistent(board)
        .map_err(|error| uncertain(format!("the written board could not be read back: {error}")))?;
    if persisted != candidate {
        return Err(uncertain(
            "the persisted board differs from the planned bytes".to_string(),
        ));
    }
    let semantic = verify(&persisted)
        .map_err(|error| uncertain(format!("the persisted board did not read back: {error}")))?;
    Ok(json!({
        "file": board.display().to_string(),
        "atomicity": "single_atomic_file_replace",
        "original_sha256": sha256_hex(original.as_bytes()),
        "written_sha256": sha256_hex(persisted.as_bytes()),
        "persisted_bytes_match_plan": true,
        "write_reported_error_after_replace": reported_error,
        "persisted_readback": semantic,
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_spans_changed_accepts_exactly_the_planned_replacements() {
        let original = "(a (b 1) (c 2) (d 3))";
        let candidate = "(a (b 10) (c 2) (d 30))";
        let b = original.find("(b 1)").unwrap();
        let d = original.find("(d 3)").unwrap();
        assert!(only_spans_changed(
            original,
            candidate,
            &[(b, b + 5, 6), (d, d + 5, 6)]
        ));
        // A byte changed outside the spans is caught.
        assert!(!only_spans_changed(
            original,
            "(a (b 10) (c 9) (d 30))",
            &[(b, b + 5, 6), (d, d + 5, 6)]
        ));
        assert!(only_spans_changed(original, original, &[]));
        assert!(!only_spans_changed(original, "(a)", &[]));
    }

    #[test]
    fn the_revision_covers_domain_bytes_and_request() {
        let a = revision("d", "board", &json!({"x": 1}));
        assert_eq!(a, revision("d", "board", &json!({"x": 1})));
        assert_ne!(a, revision("e", "board", &json!({"x": 1})));
        assert_ne!(a, revision("d", "board ", &json!({"x": 1})));
        assert_ne!(a, revision("d", "board", &json!({"x": 2})));
    }
}
