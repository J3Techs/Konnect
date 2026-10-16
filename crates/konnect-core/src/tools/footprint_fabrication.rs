//! Pad fabrication properties, and the live KiCad 10 paths that drop them.
//!
//! A pad's fabrication property — `(property pad_prop_bga)`,
//! `(property pad_prop_heatsink)` and the rest — has no field in KiCad 10's
//! `Pad` protobuf. `FOOTPRINT::Deserialize` (pcbnew/footprint.cpp, 10.0.4)
//! clears a footprint's pads and re-creates every one from that message, so a
//! live tool that sends a whole `FootprintInstance` back through `UpdateItems`
//! — to move it, rotate it, rename its value, edit its models — re-creates its
//! pads without the property. A typed readback afterwards cannot show the
//! loss: it cannot show the property either.
//!
//! This module reads the property from the two sources that can show it, the
//! saved board file and the footprint's library, and refuses a live
//! whole-footprint rebuild of a footprint either source says carries one, or
//! one neither source can be read for. The closed-board file paths edit the
//! footprint's text in place and keep every pad clause, so they stay the way
//! to move such a footprint.

use std::collections::{BTreeMap, HashMap};
use std::path::Path;

use konnect_ipc::gen::kiapi;
use konnect_sexp::SexpNode;
use prost::Message;
use serde::Serialize;

use crate::mcp::protocol::CallToolResult;

/// How a live typed read reports a pad's fabrication property: it cannot.
/// Distinct from a saved read that found none.
pub(crate) const LIVE_TYPED_UNOBSERVABLE: &str = "unobservable_via_live_typed_api";

/// The `(property …)` a saved or library pad carries.
pub(crate) fn pad_fabrication_property(pad: &SexpNode) -> Option<&str> {
    pad.find("property")
        .and_then(|property| property.get(1))
        .and_then(SexpNode::as_str)
}

/// One pad's fabrication property, as a file states it.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub(crate) struct PadProperty {
    pub(crate) pad: String,
    pub(crate) property: String,
}

/// Every pad of `footprint` that carries a fabrication property, in order.
pub(crate) fn footprint_pad_properties(footprint: &SexpNode) -> Vec<PadProperty> {
    footprint
        .find_all("pad")
        .into_iter()
        .filter_map(|pad| {
            Some(PadProperty {
                pad: pad
                    .get(1)
                    .and_then(SexpNode::as_str)
                    .unwrap_or_default()
                    .to_string(),
                property: pad_fabrication_property(pad)?.to_string(),
            })
        })
        .collect()
}

/// One live footprint a tool is about to rebuild through `UpdateItems`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct RebuildTarget {
    pub(crate) reference: String,
    pub(crate) kiid: String,
    pub(crate) footprint_id: String,
}

impl RebuildTarget {
    pub(crate) fn from_instance(footprint: &kiapi::board::types::FootprintInstance) -> Self {
        Self {
            reference: footprint
                .reference_field
                .as_ref()
                .and_then(|field| field.text.as_ref())
                .and_then(|text| text.text.as_ref())
                .map(|text| text.text.clone())
                .unwrap_or_default(),
            kiid: footprint
                .id
                .as_ref()
                .map(|id| id.value.clone())
                .unwrap_or_default(),
            footprint_id: footprint
                .definition
                .as_ref()
                .and_then(|definition| definition.id.as_ref())
                .map(|id| format!("{}:{}", id.library_nickname, id.entry_name))
                .unwrap_or_default(),
        }
    }
}

/// The live footprints named by `references`, read from the board the client
/// is bound to. A reference KiCad does not hold is left out: the mutation
/// itself reports it, unchanged.
pub(crate) fn live_rebuild_targets(
    client: &konnect_ipc::KiCadIpcClient,
    document: Option<kiapi::common::types::DocumentSpecifier>,
    references: &[&str],
) -> anyhow::Result<Vec<RebuildTarget>> {
    let mut targets = Vec::new();
    let kind = kiapi::common::types::KiCadObjectType::KotPcbFootprint;
    let items = match document {
        Some(document) => client.get_items_in(document, kind)?,
        None => client.get_items(kind)?,
    };
    for item in items {
        if !konnect_ipc::builders::any_is(&item, "kiapi.board.types.FootprintInstance") {
            continue;
        }
        let footprint = kiapi::board::types::FootprintInstance::decode(item.value.as_slice())?;
        let target = RebuildTarget::from_instance(&footprint);
        if references.contains(&target.reference.as_str()) {
            targets.push(target);
        }
    }
    Ok(targets)
}

/// What one source said about one footprint.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
#[serde(tag = "status", rename_all = "snake_case")]
pub(crate) enum SourceFacts {
    Read {
        path: String,
        pad_properties: Vec<PadProperty>,
    },
    Unavailable {
        reason: String,
    },
}

impl SourceFacts {
    fn carries_property(&self) -> bool {
        matches!(self, Self::Read { pad_properties, .. } if !pad_properties.is_empty())
    }

    fn is_read(&self) -> bool {
        matches!(self, Self::Read { .. })
    }
}

/// Why one footprint may not be rebuilt live.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub(crate) struct RebuildFinding {
    pub(crate) reference: String,
    pub(crate) kiid: String,
    pub(crate) footprint_id: String,
    /// `carries_fabrication_property` or `unverifiable`.
    pub(crate) verdict: &'static str,
    pub(crate) saved_board: SourceFacts,
    pub(crate) library: SourceFacts,
}

impl RebuildFinding {
    pub(crate) fn reason(&self) -> String {
        let properties = |facts: &SourceFacts| match facts {
            SourceFacts::Read { pad_properties, .. } => pad_properties
                .iter()
                .map(|fact| format!("pad '{}' {}", fact.pad, fact.property))
                .collect::<Vec<_>>()
                .join(", "),
            SourceFacts::Unavailable { reason } => format!("unavailable ({reason})"),
        };
        match self.verdict {
            "carries_fabrication_property" => format!(
                "{} ({}) carries pad fabrication properties — saved board: {}; library: {}. \
                 A live whole-footprint update re-creates its pads without them.",
                self.reference,
                self.footprint_id,
                properties(&self.saved_board),
                properties(&self.library)
            ),
            _ => format!(
                "{} ({}): neither the saved board nor the footprint library could be read to \
                 prove its pads carry no fabrication property — saved board: {}; library: {}.",
                self.reference,
                self.footprint_id,
                properties(&self.saved_board),
                properties(&self.library)
            ),
        }
    }
}

/// What a refused caller can do instead.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Remedy {
    /// The tool has a closed-board file path that keeps every pad clause.
    ClosedBoardPath,
    /// The tool has no lossless path for such a footprint.
    NoLosslessPath,
}

/// A live whole-footprint rebuild refused before anything was sent.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct RebuildRefusal {
    pub(crate) operation: String,
    pub(crate) findings: Vec<RebuildFinding>,
    pub(crate) remedy: Remedy,
}

impl RebuildRefusal {
    pub(crate) fn into_result(self) -> CallToolResult {
        let mut reasons = self
            .findings
            .iter()
            .map(RebuildFinding::reason)
            .collect::<Vec<_>>();
        reasons.push(
            match self.remedy {
                Remedy::ClosedBoardPath => CLOSED_BOARD_REMEDY,
                Remedy::NoLosslessPath => NO_LOSSLESS_REMEDY,
            }
            .to_string(),
        );
        CallToolResult::error_kind(
            crate::mcp::error::ToolErrorKind::PlanBlocked {
                operation: self.operation.clone(),
                reasons: reasons.clone(),
            },
            format!(
                "Refused {} before any change: {} The board was not modified.",
                self.operation,
                reasons.join(" ")
            ),
        )
    }
}

const CLOSED_BOARD_REMEDY: &str = "KiCad 10's typed footprint API has no field for a pad's \
     fabrication property, and Konnect cannot observe the loss afterwards. Save the board, \
     close it in KiCad and repeat the request: with the board closed this tool edits the saved \
     footprint in place and keeps every pad clause. Then reopen the board.";

const NO_LOSSLESS_REMEDY: &str = "KiCad 10's typed footprint API has no field for a pad's \
     fabrication property, and Konnect cannot observe the loss afterwards. This tool has no \
     closed-board path that keeps it; make this change in KiCad itself.";

/// Refuse a live whole-footprint rebuild of any target whose saved board
/// footprint or library footprint carries a pad fabrication property, or for
/// which neither can be read. Each source is read once, whatever the number of
/// targets.
pub(crate) fn check_live_rebuild(
    board_path: &Path,
    operation: &str,
    targets: &[RebuildTarget],
    remedy: Remedy,
) -> Result<(), RebuildRefusal> {
    if targets.is_empty() {
        return Ok(());
    }
    let saved = SavedBoardIndex::read(board_path);
    let mut libraries: HashMap<String, SourceFacts> = HashMap::new();
    let mut findings = Vec::new();
    for target in targets {
        let saved_board = saved.facts(target);
        let library = libraries
            .entry(target.footprint_id.clone())
            .or_insert_with(|| library_facts(&target.footprint_id, board_path))
            .clone();
        let verdict = if saved_board.carries_property() || library.carries_property() {
            "carries_fabrication_property"
        } else if !saved_board.is_read() && !library.is_read() {
            "unverifiable"
        } else {
            continue;
        };
        findings.push(RebuildFinding {
            reference: target.reference.clone(),
            kiid: target.kiid.clone(),
            footprint_id: target.footprint_id.clone(),
            verdict,
            saved_board,
            library,
        });
    }
    if findings.is_empty() {
        Ok(())
    } else {
        Err(RebuildRefusal {
            operation: operation.to_string(),
            findings,
            remedy,
        })
    }
}

/// [`check_live_rebuild`] for the live footprints `references` names, read
/// from the board the client is bound to: the one guard every live
/// whole-footprint tool runs before it sends anything.
pub(crate) fn guard_live_rebuild(
    client: &konnect_ipc::KiCadIpcClient,
    board_path: &Path,
    operation: &str,
    references: &[&str],
    remedy: Remedy,
) -> anyhow::Result<Result<(), RebuildRefusal>> {
    let targets = live_rebuild_targets(client, None, references)?;
    Ok(check_live_rebuild(board_path, operation, &targets, remedy))
}

/// [`guard_live_rebuild`] for a tool bound to an exact document.
pub(crate) fn guard_live_rebuild_in(
    client: &konnect_ipc::KiCadIpcClient,
    document: kiapi::common::types::DocumentSpecifier,
    board_path: &Path,
    operation: &str,
    references: &[&str],
    remedy: Remedy,
) -> anyhow::Result<Result<(), RebuildRefusal>> {
    let targets = live_rebuild_targets(client, Some(document), references)?;
    Ok(check_live_rebuild(board_path, operation, &targets, remedy))
}

fn library_facts(footprint_id: &str, board_path: &Path) -> SourceFacts {
    if footprint_id.is_empty() || footprint_id == ":" {
        return SourceFacts::Unavailable {
            reason: "the live footprint names no library footprint".to_string(),
        };
    }
    let path = match super::pcb_components::resolve_footprint_file(footprint_id, board_path) {
        Ok(path) => path,
        Err(error) => {
            return SourceFacts::Unavailable {
                reason: format!("{error:#}"),
            }
        }
    };
    let facts = std::fs::read_to_string(&path)
        .map_err(|error| error.to_string())
        .and_then(|source| konnect_sexp::parse_sexp(&source).map_err(|error| error.to_string()))
        .and_then(|root| {
            if root.head() == Some("footprint") {
                Ok(footprint_pad_properties(&root))
            } else {
                Err("not a footprint file".to_string())
            }
        });
    match facts {
        Ok(pad_properties) => SourceFacts::Read {
            path: path.display().to_string(),
            pad_properties,
        },
        Err(reason) => SourceFacts::Unavailable {
            reason: format!("{}: {reason}", path.display()),
        },
    }
}

/// The saved board's footprints by KIID and by reference, read once.
struct SavedBoardIndex {
    path: String,
    /// `Err` holds why the board could not be read.
    footprints: Result<Vec<SavedFootprint>, String>,
}

struct SavedFootprint {
    kiid: Option<String>,
    reference: Option<String>,
    pad_properties: Vec<PadProperty>,
}

impl SavedBoardIndex {
    fn read(board_path: &Path) -> Self {
        let footprints = std::fs::read_to_string(board_path)
            .map_err(|error| error.to_string())
            .and_then(|source| konnect_sexp::parse_sexp(&source).map_err(|error| error.to_string()))
            .and_then(|root| {
                if root.head() != Some("kicad_pcb") {
                    return Err("not a kicad_pcb file".to_string());
                }
                Ok(root
                    .find_all("footprint")
                    .into_iter()
                    .map(|footprint| SavedFootprint {
                        kiid: footprint_identity(footprint).map(str::to_string),
                        reference: super::pcb_components::footprint_reference(footprint),
                        pad_properties: footprint_pad_properties(footprint),
                    })
                    .collect())
            });
        Self {
            path: board_path.display().to_string(),
            footprints,
        }
    }

    fn facts(&self, target: &RebuildTarget) -> SourceFacts {
        let footprints = match &self.footprints {
            Ok(footprints) => footprints,
            Err(reason) => {
                return SourceFacts::Unavailable {
                    reason: format!("{}: {reason}", self.path),
                }
            }
        };
        let read = |found: &SavedFootprint| SourceFacts::Read {
            path: self.path.clone(),
            pad_properties: found.pad_properties.clone(),
        };
        if let Some(found) = footprints
            .iter()
            .find(|footprint| footprint.kiid.as_deref() == Some(target.kiid.as_str()))
        {
            return read(found);
        }
        let by_reference = footprints
            .iter()
            .filter(|footprint| footprint.reference.as_deref() == Some(target.reference.as_str()))
            .collect::<Vec<_>>();
        match by_reference.as_slice() {
            [only] => read(only),
            [] => SourceFacts::Unavailable {
                reason: "the saved board has no footprint with this KIID or reference; save \
                         the board first"
                    .to_string(),
            },
            several => {
                // Ambiguous, so it proves nothing absent; but any of them
                // carrying a property is reason enough to refuse.
                let mut merged = BTreeMap::new();
                for footprint in several {
                    for fact in &footprint.pad_properties {
                        merged.insert((fact.pad.clone(), fact.property.clone()), ());
                    }
                }
                if merged.is_empty() {
                    SourceFacts::Unavailable {
                        reason: format!(
                            "the saved board has {} footprints named {} and none with this KIID",
                            several.len(),
                            target.reference
                        ),
                    }
                } else {
                    SourceFacts::Read {
                        path: self.path.clone(),
                        pad_properties: merged
                            .into_keys()
                            .map(|(pad, property)| PadProperty { pad, property })
                            .collect(),
                    }
                }
            }
        }
    }
}

/// A footprint's own `(uuid …)`, or the `(tstamp …)` older files use.
pub(crate) fn footprint_identity(footprint: &SexpNode) -> Option<&str> {
    footprint
        .find_str("uuid")
        .or_else(|| footprint.find_str("tstamp"))
        .filter(|value| !value.is_empty())
}

#[cfg(test)]
mod tests {
    use super::*;

    const LIBRARY: &str = r#"(footprint "Bga"
	(layer "F.Cu")
	(pad "A1" smd circle (at 0 0) (size 0.3 0.3) (layers "F.Cu" "F.Mask") (property pad_prop_bga))
	(pad "A2" smd circle (at 0.5 0) (size 0.3 0.3) (layers "F.Cu" "F.Mask") (property pad_prop_bga))
)"#;

    fn project(board: &str) -> (tempfile::TempDir, std::path::PathBuf) {
        let temp = tempfile::tempdir().unwrap();
        let pretty = temp.path().join("Parts.pretty");
        std::fs::create_dir_all(&pretty).unwrap();
        std::fs::write(pretty.join("Bga.kicad_mod"), LIBRARY).unwrap();
        std::fs::write(
            pretty.join("Plain.kicad_mod"),
            LIBRARY.replace(" (property pad_prop_bga)", ""),
        )
        .unwrap();
        std::fs::write(
            temp.path().join("fp-lib-table"),
            "(fp_lib_table (lib (name \"Parts\") (type \"KiCad\") (uri \"${KIPRJMOD}/Parts.pretty\") (options \"\") (descr \"\")))",
        )
        .unwrap();
        let path = temp.path().join("board.kicad_pcb");
        std::fs::write(&path, board).unwrap();
        (temp, path)
    }

    fn target(reference: &str, kiid: &str, footprint_id: &str) -> RebuildTarget {
        RebuildTarget {
            reference: reference.to_string(),
            kiid: kiid.to_string(),
            footprint_id: footprint_id.to_string(),
        }
    }

    #[test]
    fn a_library_or_saved_property_refuses_and_a_plain_footprint_passes() {
        let (_temp, board) = project(
            r#"(kicad_pcb (version 20260206)
  (footprint "Parts:Plain" (layer "F.Cu") (uuid "aaaaaaaa-0000-4000-8000-000000000001")
    (property "Reference" "U2")
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (property pad_prop_heatsink)))
  (footprint "Parts:Plain" (layer "F.Cu") (uuid "aaaaaaaa-0000-4000-8000-000000000003")
    (property "Reference" "U3")
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu"))))"#,
        );
        let refusal = check_live_rebuild(
            &board,
            "move_component",
            &[
                target("U1", "new-unsaved", "Parts:Bga"),
                target("U2", "aaaaaaaa-0000-4000-8000-000000000001", "Parts:Plain"),
                target("U3", "aaaaaaaa-0000-4000-8000-000000000003", "Parts:Plain"),
            ],
            Remedy::ClosedBoardPath,
        )
        .unwrap_err();
        let verdicts = refusal
            .findings
            .iter()
            .map(|finding| (finding.reference.as_str(), finding.verdict))
            .collect::<Vec<_>>();
        assert_eq!(
            verdicts,
            [
                ("U1", "carries_fabrication_property"),
                ("U2", "carries_fabrication_property")
            ],
            "the library names U1's BGA pads, the saved board U2's heatsink pad; U3 is plain"
        );
        let text = format!("{:?}", refusal.clone().into_result().content);
        assert!(
            text.contains("plan_blocked") && text.contains("pad_prop_heatsink"),
            "{text}"
        );

        assert!(check_live_rebuild(
            &board,
            "move_component",
            &[target(
                "U3",
                "aaaaaaaa-0000-4000-8000-000000000003",
                "Parts:Plain"
            )],
            Remedy::ClosedBoardPath
        )
        .is_ok());
    }

    /// Absence must be read, never assumed: a footprint with no readable
    /// library and no saved copy is refused, not passed.
    #[test]
    fn a_footprint_neither_source_can_read_is_unverifiable() {
        let (_temp, board) = project("(kicad_pcb (version 20260206))");
        let refusal = check_live_rebuild(
            &board,
            "rotate_component",
            &[target("X1", "unsaved", "Missing:Thing")],
            Remedy::NoLosslessPath,
        )
        .unwrap_err();
        assert_eq!(refusal.findings[0].verdict, "unverifiable");
    }
}
