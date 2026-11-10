//! Closed-board reads and edits of the 3D-model entries of one placed footprint.
//!
//! The live path of `set_placed_footprint_models` sends the whole
//! `FootprintInstance` back, and `FOOTPRINT::Deserialize` re-creates every pad
//! from a message with no field for a pad's fabrication property (see
//! `footprint_fabrication`), so it refuses a footprint that carries one. On a
//! board KiCad does not hold, this module edits the saved file instead: it
//! replaces, cuts or inserts only `(model …)` blocks that are direct children
//! of the one footprint the reference names, and every other byte of the board
//! stays as it was. Pad clauses, fabrication properties, nets, fields and
//! UUIDs are untouched by construction, and the write is checked byte for byte
//! outside the planned spans.
//!
//! Values are the saved file's own: offset in millimetres, rotation in
//! degrees, unitless scale, `(hide yes)` and `(opacity …)`.

use super::reference_field::{footprints_by_reference, head_of, mm};
use super::*;
use crate::tools::closed_board::only_spans_changed;
use crate::tools::footprint_fabrication::{
    footprint_identity, footprint_pad_properties, PadProperty,
};
use konnect_ipc::{
    IpcFootprint3DModel, IpcFootprint3DModelEdit, IpcFootprint3DModelSnapshot, IpcVector3,
};
use konnect_sexp::SexpNode;
use serde_json::Value;

const TOLERANCE: f64 = 5e-7;

/// Why the saved footprint could not be read or the edit planned.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum SavedModelError {
    /// No footprint in the saved board has this exact reference.
    Missing,
    /// More than one does.
    Ambiguous(usize),
    /// `model_index` names no entry.
    IndexOutOfRange { index: usize, count: usize },
    /// Anything else, in words.
    Other(String),
}

impl From<String> for SavedModelError {
    fn from(reason: String) -> Self {
        Self::Other(reason)
    }
}

/// The 3D-model entries of one saved footprint, in file order.
#[derive(Debug, Clone)]
pub(crate) struct SavedModels {
    pub snapshot: IpcFootprint3DModelSnapshot,
    pub pad_properties: Vec<PadProperty>,
    footprint: (usize, usize),
    spans: Vec<(usize, usize)>,
}

fn footprint_span(source: &str, reference: &str) -> Result<(usize, usize), SavedModelError> {
    let index = footprints_by_reference(source)?;
    match index.get(reference).map(Vec::as_slice).unwrap_or(&[]) {
        [] => Err(SavedModelError::Missing),
        [span] => Ok(*span),
        many => Err(SavedModelError::Ambiguous(many.len())),
    }
}

fn xyz(node: &SexpNode, tag: &str, index: usize) -> Result<IpcVector3, String> {
    let vector = node
        .find(tag)
        .and_then(|clause| clause.find("xyz"))
        .and_then(|xyz| {
            Some(IpcVector3 {
                x: xyz.get_f64(1)?,
                y: xyz.get_f64(2)?,
                z: xyz.get_f64(3)?,
            })
        });
    vector.ok_or_else(|| format!("3D model {index} has no readable ({tag} (xyz …)) clause"))
}

/// One `(model …)` block as KiCad 10 writes it. A clause this reader does not
/// know is refused rather than dropped by a later replace.
fn read_model(text: &str, index: usize) -> Result<IpcFootprint3DModel, String> {
    let node = konnect_sexp::parse_sexp(text).map_err(|error| error.to_string())?;
    let children = node.children().unwrap_or(&[]);
    let filename = children
        .get(1)
        .and_then(SexpNode::as_str)
        .ok_or_else(|| format!("3D model {index} names no file"))?
        .to_string();
    let mut visible = true;
    for child in &children[2.min(children.len())..] {
        match (child.head(), child.as_str()) {
            (Some("hide"), _) => {
                // `(hide yes)`; a bare `(hide)` hides too, `(hide no)` does not.
                visible = child.get(1).and_then(SexpNode::as_str) == Some("no");
            }
            (None, Some("hide")) => visible = false,
            (Some("opacity" | "offset" | "scale" | "rotate"), _) => {}
            (Some("at"), _) => {
                return Err(format!(
                    "3D model {index} uses the legacy (at …) offset; open and save the board in \
                     KiCad once so it is written as (offset …)"
                ))
            }
            (head, atom) => {
                return Err(format!(
                    "3D model {index} carries a clause this tool does not know: '{}'",
                    head.or(atom).unwrap_or("?")
                ))
            }
        }
    }
    Ok(IpcFootprint3DModel {
        filename,
        offset_mm: xyz(&node, "offset", index)?,
        rotation_degrees: xyz(&node, "rotate", index)?,
        scale: xyz(&node, "scale", index)?,
        visible,
        opacity: match node.find("opacity") {
            Some(_) => node
                .find_f64("opacity")
                .ok_or_else(|| format!("3D model {index} has an unreadable (opacity …)"))?,
            None => 1.0,
        },
    })
}

/// Read the 3D-model entries of the footprint `reference` names.
pub(crate) fn read(source: &str, reference: &str) -> Result<SavedModels, SavedModelError> {
    check_single_board_form(source)?;
    let footprint = footprint_span(source, reference)?;
    let block = &source[footprint.0..footprint.1];
    let node = konnect_sexp::parse_sexp(block).map_err(|error| error.to_string())?;
    let mut models = Vec::new();
    let mut spans = Vec::new();
    for (start, end) in find_direct_child_blocks(block, "footprint") {
        if head_of(&block[start..end]) != "model" {
            continue;
        }
        models.push(read_model(&block[start..end], models.len())?);
        spans.push((footprint.0 + start, footprint.0 + end));
    }
    Ok(SavedModels {
        snapshot: IpcFootprint3DModelSnapshot {
            reference: reference.to_string(),
            kiid: footprint_identity(&node).unwrap_or_default().to_string(),
            models,
        },
        pad_properties: footprint_pad_properties(&node),
        footprint,
        spans,
    })
}

fn same_vector(a: IpcVector3, b: IpcVector3) -> bool {
    (a.x - b.x).abs() <= TOLERANCE
        && (a.y - b.y).abs() <= TOLERANCE
        && (a.z - b.z).abs() <= TOLERANCE
}

fn same_model(a: &IpcFootprint3DModel, b: &IpcFootprint3DModel) -> bool {
    a.filename == b.filename
        && a.visible == b.visible
        && (a.opacity - b.opacity).abs() <= TOLERANCE
        && same_vector(a.offset_mm, b.offset_mm)
        && same_vector(a.rotation_degrees, b.rotation_degrees)
        && same_vector(a.scale, b.scale)
}

fn same_models(a: &[IpcFootprint3DModel], b: &[IpcFootprint3DModel]) -> bool {
    a.len() == b.len() && a.iter().zip(b).all(|(a, b)| same_model(a, b))
}

/// The whitespace a block at `start` is indented by, when it sits on its own
/// line.
fn indent_before(source: &str, start: usize) -> Option<&str> {
    let line_start = source[..start].rfind('\n').map_or(0, |i| i + 1);
    let indent = &source[line_start..start];
    indent
        .chars()
        .all(|c| c == ' ' || c == '\t')
        .then_some(indent)
}

/// One model block. With `indent` (the block sits on its own line) it is laid
/// out as KiCad 10 saves it; otherwise on one line.
fn model_text(model: &IpcFootprint3DModel, indent: Option<&str>, eol: &str) -> String {
    let path = crate::tools::footprint_models::quote_string(&model.filename);
    let vector = |v: IpcVector3| format!("(xyz {} {} {})", mm(v.x), mm(v.y), mm(v.z));
    let hide = (!model.visible).then(|| "(hide yes)".to_string());
    let opacity = ((model.opacity - 1.0).abs() > TOLERANCE)
        .then(|| format!("(opacity {})", mm(model.opacity)));
    let Some(indent) = indent else {
        let mut parts = vec![format!("(model {path}")];
        parts.extend(hide);
        parts.extend(opacity);
        parts.push(format!("(offset {})", vector(model.offset_mm)));
        parts.push(format!("(scale {})", vector(model.scale)));
        parts.push(format!("(rotate {}))", vector(model.rotation_degrees)));
        return parts.join(" ");
    };
    let step = if indent.contains(' ') && !indent.contains('\t') {
        "  "
    } else {
        "\t"
    };
    let inner = format!("{indent}{step}");
    let mut lines = vec![format!("(model {path}")];
    lines.extend(hide.map(|hide| format!("{inner}{hide}")));
    lines.extend(opacity.map(|opacity| format!("{inner}{opacity}")));
    for (tag, value) in [
        ("offset", model.offset_mm),
        ("scale", model.scale),
        ("rotate", model.rotation_degrees),
    ] {
        lines.push(format!("{inner}({tag}"));
        lines.push(format!("{inner}{step}{}", vector(value)));
        lines.push(format!("{inner})"));
    }
    lines.push(format!("{indent})"));
    lines.join(eol)
}

/// The planned write.
#[derive(Debug, Clone)]
pub(crate) struct Prepared {
    pub candidate: String,
    /// `(start, end, replacement_len)` against the original.
    pub edits: Vec<(usize, usize, usize)>,
    pub before: SavedModels,
    /// The model list the footprint must read back with.
    pub expected: Vec<IpcFootprint3DModel>,
}

impl Prepared {
    pub(crate) fn changed(&self) -> bool {
        !self.edits.is_empty()
    }
}

/// Plan `edit` on the footprint `reference` names, and check the candidate
/// exactly as an apply checks the written file.
pub(crate) fn prepare(
    source: &str,
    reference: &str,
    edit: &IpcFootprint3DModelEdit,
) -> Result<Prepared, SavedModelError> {
    let before = read(source, reference)?;
    let count = before.spans.len();
    let in_range = |index: usize| {
        if index < count {
            Ok(index)
        } else {
            Err(SavedModelError::IndexOutOfRange { index, count })
        }
    };
    let eol = if source.contains("\r\n") {
        "\r\n"
    } else {
        "\n"
    };
    let mut expected = before.snapshot.models.clone();
    let mut changes = Vec::new();
    match edit {
        IpcFootprint3DModelEdit::Remove { index } => {
            let (start, end) = before.spans[in_range(*index)?];
            // The block and the whitespace that sets it on its own line.
            let cut_from = source[..start]
                .trim_end_matches([' ', '\t', '\r', '\n'])
                .len();
            changes.push(SexpEdit::delete(cut_from, end));
            expected.remove(*index);
        }
        IpcFootprint3DModelEdit::Replace { index, model } => {
            let (start, end) = before.spans[in_range(*index)?];
            if !same_model(&expected[*index], model) {
                changes.push(SexpEdit::replace(
                    start,
                    end,
                    model_text(model, indent_before(source, start), eol),
                ));
                expected[*index] = model.clone();
            }
        }
        IpcFootprint3DModelEdit::Append { model } => {
            // KiCad saves a footprint's models last, so a new one goes after
            // the last model, or after the footprint's last child.
            let (at, indent) = match before.spans.last() {
                Some(&(start, end)) => (end, indent_before(source, start)),
                None => {
                    let block = &source[before.footprint.0..before.footprint.1];
                    let indent = find_direct_child_blocks(block, "footprint")
                        .last()
                        .and_then(|&(start, _)| indent_before(source, before.footprint.0 + start));
                    let at = source[..before.footprint.1 - 1]
                        .trim_end_matches([' ', '\t', '\r', '\n'])
                        .len();
                    (at, indent)
                }
            };
            let text = model_text(model, indent, eol);
            changes.push(SexpEdit::insert(
                at,
                match indent {
                    Some(indent) => format!("{eol}{indent}{text}"),
                    None => format!(" {text}"),
                },
            ));
            expected.push(model.clone());
        }
    }
    let edits: Vec<_> = changes
        .iter()
        .map(|edit| (edit.start, edit.end, edit.replacement.len()))
        .collect();
    let prepared = Prepared {
        candidate: apply_edits(source.to_string(), changes),
        edits,
        before,
        expected,
    };
    verify(source, &prepared.candidate, &prepared, reference)?;
    Ok(prepared)
}

/// The footprint's direct children that are not 3D models, as text.
fn other_children(source: &str, footprint: (usize, usize)) -> Vec<&str> {
    let block = &source[footprint.0..footprint.1];
    find_direct_child_blocks(block, "footprint")
        .into_iter()
        .map(|(start, end)| &block[start..end])
        .filter(|child| head_of(child) != "model")
        .collect()
}

/// The candidate (or persisted) board differs from the original only inside
/// the planned spans, the footprint's models read back as planned, and every
/// other clause of the footprint — its pads and their fabrication properties
/// among them — is the text it was.
pub(crate) fn verify(
    original: &str,
    candidate: &str,
    prepared: &Prepared,
    reference: &str,
) -> Result<Value, String> {
    if !only_spans_changed(original, candidate, &prepared.edits) {
        return Err("bytes outside the planned 3D-model spans changed".to_string());
    }
    konnect_sexp::parse_sexp(candidate).map_err(|error| error.to_string())?;
    let after = read(candidate, reference).map_err(|error| match error {
        SavedModelError::Other(reason) => reason,
        other => format!("footprint '{reference}' no longer reads back: {other:?}"),
    })?;
    if !same_models(&after.snapshot.models, &prepared.expected) {
        return Err(format!(
            "the 3D models of '{reference}' do not read back as planned"
        ));
    }
    if after.snapshot.kiid != prepared.before.snapshot.kiid {
        return Err(format!("the UUID of '{reference}' changed"));
    }
    if after.pad_properties != prepared.before.pad_properties {
        return Err(format!(
            "the pad fabrication properties of '{reference}' changed"
        ));
    }
    if other_children(candidate, after.footprint)
        != other_children(original, prepared.before.footprint)
    {
        return Err(format!(
            "a clause of '{reference}' other than a 3D model changed"
        ));
    }
    Ok(json!({
        "models": after.snapshot.models,
        "pad_fabrication_properties": after.pad_properties,
        "bytes_outside_planned_spans_unchanged": true,
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A thermal-via SOIC as KiCad 10 saves it, whose pad 9 carries
    /// `pad_prop_heatsink` and whose second model file does not exist; a
    /// connector with no model; and a one-line footprint with a hidden,
    /// translucent one.
    const BOARD: &str = "(kicad_pcb (version 20260206)
\t(footprint \"Package_SO:SOIC-8-EP\"
\t\t(layer \"F.Cu\")
\t\t(uuid \"11111111-1111-4111-8111-111111111111\")
\t\t(at 20 30 -90)
\t\t(property \"Reference\" \"U2\" (at 0 -3 90) (layer \"F.SilkS\") (uuid \"21111111-1111-4111-8111-111111111111\")
\t\t\t(effects (font (size 0.8 0.8) (thickness 0.12))))
\t\t(pad \"9\" smd rect (at 0 0 -90) (size 2.95 4.9) (property pad_prop_heatsink) (layers \"F.Cu\" \"F.Mask\") (net 1 \"GND\") (uuid \"51111111-1111-4111-8111-111111111111\"))
\t\t(embedded_fonts no)
\t\t(model \"${KICAD10_3DMODEL_DIR}/Package_SO.3dshapes/SOIC-8-EP.step\"
\t\t\t(offset
\t\t\t\t(xyz 0 0 0)
\t\t\t)
\t\t\t(scale
\t\t\t\t(xyz 1 1 1)
\t\t\t)
\t\t\t(rotate
\t\t\t\t(xyz 0 0 0)
\t\t\t)
\t\t)
\t\t(model \"${KIPRJMOD}/3d/missing.step\"
\t\t\t(offset
\t\t\t\t(xyz 0 0.5 0)
\t\t\t)
\t\t\t(scale
\t\t\t\t(xyz 1 1 1)
\t\t\t)
\t\t\t(rotate
\t\t\t\t(xyz 0 0 90)
\t\t\t)
\t\t)
\t)
\t(footprint \"Connector_USB:USB_C\"
\t\t(layer \"F.Cu\")
\t\t(uuid \"12222222-2222-4222-8222-222222222222\")
\t\t(at 5 5)
\t\t(property \"Reference\" \"J301\" (at 0 0) (layer \"F.SilkS\") (uuid \"22222222-2222-4222-8222-222222222222\"))
\t\t(pad \"SH\" thru_hole oval (at 1 1) (size 1 2) (drill 0.6) (property pad_prop_mechanical) (layers \"*.Cu\" \"*.Mask\") (uuid \"52222222-2222-4222-8222-222222222222\"))
\t\t(embedded_fonts no)
\t)
\t(footprint \"Lib:Flat\" (layer \"B.Cu\") (uuid \"13333333-3333-4333-8333-333333333333\") (at 9 9) (property \"Reference\" \"D1\" (at 0 0) (layer \"B.SilkS\") (uuid \"23333333-3333-4333-8333-333333333333\")) (model \"a b.wrl\" (hide yes) (opacity 0.5) (offset (xyz 1 2 3)) (scale (xyz 1 1 1)) (rotate (xyz 0 0 0))))
)
";

    fn model(path: &str) -> IpcFootprint3DModel {
        let v = |x, y, z| IpcVector3 { x, y, z };
        IpcFootprint3DModel {
            filename: path.to_string(),
            offset_mm: v(0.0, 0.0, 0.0),
            rotation_degrees: v(0.0, 0.0, 0.0),
            scale: v(1.0, 1.0, 1.0),
            visible: true,
            opacity: 1.0,
        }
    }

    #[test]
    fn reads_each_entry_with_its_transform_visibility_and_opacity() {
        let u2 = read(BOARD, "U2").unwrap();
        assert_eq!(u2.snapshot.kiid, "11111111-1111-4111-8111-111111111111");
        assert_eq!(u2.snapshot.models.len(), 2);
        assert_eq!(
            u2.snapshot.models[1].filename,
            "${KIPRJMOD}/3d/missing.step"
        );
        assert_eq!(u2.snapshot.models[1].offset_mm.y, 0.5);
        assert_eq!(u2.snapshot.models[1].rotation_degrees.z, 90.0);
        assert_eq!(u2.pad_properties[0].property, "pad_prop_heatsink");

        assert!(read(BOARD, "J301").unwrap().snapshot.models.is_empty());
        let d1 = &read(BOARD, "D1").unwrap().snapshot.models[0];
        assert!(!d1.visible);
        assert_eq!(d1.opacity, 0.5);
        assert_eq!(d1.offset_mm.z, 3.0);

        assert_eq!(read(BOARD, "NOPE").unwrap_err(), SavedModelError::Missing);
    }

    #[test]
    fn remove_cuts_only_the_named_block_and_keeps_the_heatsink_pad() {
        let prepared = prepare(BOARD, "U2", &IpcFootprint3DModelEdit::Remove { index: 1 }).unwrap();
        assert_eq!(prepared.edits.len(), 1);
        assert!(!prepared.candidate.contains("missing.step"));
        assert!(prepared.candidate.contains(
            "\t\t\t(rotate\n\t\t\t\t(xyz 0 0 0)\n\t\t\t)\n\t\t)\n\t)\n\t(footprint \"Connector_USB:USB_C\""
        ));
        assert!(prepared.candidate.contains("(property pad_prop_heatsink)"));
        assert_eq!(
            read(&prepared.candidate, "U2").unwrap().snapshot.models,
            [model(
                "${KICAD10_3DMODEL_DIR}/Package_SO.3dshapes/SOIC-8-EP.step"
            )]
        );
        // The other footprints are the bytes they were.
        let tail = BOARD.find("\t(footprint \"Connector_USB").unwrap();
        assert!(prepared.candidate.ends_with(&BOARD[tail..]));

        assert_eq!(
            prepare(BOARD, "U2", &IpcFootprint3DModelEdit::Remove { index: 2 }).unwrap_err(),
            SavedModelError::IndexOutOfRange { index: 2, count: 2 }
        );
    }

    #[test]
    fn replace_rewrites_one_block_in_kicad_layout_and_an_equal_model_is_a_noop() {
        let mut new = model("${KIPRJMOD}/3d/part.step");
        new.rotation_degrees.z = -90.0;
        new.visible = false;
        new.opacity = 0.25;
        let edit = IpcFootprint3DModelEdit::Replace {
            index: 1,
            model: new.clone(),
        };
        let prepared = prepare(BOARD, "U2", &edit).unwrap();
        assert!(prepared.candidate.contains(
            "\t\t(model \"${KIPRJMOD}/3d/part.step\"\n\t\t\t(hide yes)\n\t\t\t(opacity 0.25)\n\t\t\t(offset\n\t\t\t\t(xyz 0 0 0)\n\t\t\t)\n\t\t\t(scale\n\t\t\t\t(xyz 1 1 1)\n\t\t\t)\n\t\t\t(rotate\n\t\t\t\t(xyz 0 0 -90)\n\t\t\t)\n\t\t)\n\t)"
        ));
        assert!(prepared.candidate.contains("SOIC-8-EP.step"));
        assert!(!prepare(&prepared.candidate, "U2", &edit).unwrap().changed());
    }

    #[test]
    fn append_goes_after_the_last_model_or_the_last_child() {
        let edit = IpcFootprint3DModelEdit::Append {
            model: model("${KIPRJMOD}/3d/usb.step"),
        };
        let first = prepare(BOARD, "J301", &edit).unwrap();
        assert!(first.candidate.contains(
            "\t\t(embedded_fonts no)\n\t\t(model \"${KIPRJMOD}/3d/usb.step\"\n\t\t\t(offset\n"
        ));
        assert!(first.candidate.contains("(property pad_prop_mechanical)"));
        assert_eq!(first.expected.len(), 1);

        let second = prepare(BOARD, "U2", &edit).unwrap();
        assert_eq!(second.expected.len(), 3);
        assert_eq!(second.expected[2].filename, "${KIPRJMOD}/3d/usb.step");

        // A one-line footprint keeps its one line.
        let flat = prepare(BOARD, "D1", &edit).unwrap();
        assert!(flat.candidate.contains(
            "(rotate (xyz 0 0 0))) (model \"${KIPRJMOD}/3d/usb.step\" (offset (xyz 0 0 0)) (scale (xyz 1 1 1)) (rotate (xyz 0 0 0))))\n)"
        ));
    }

    #[test]
    fn a_model_this_reader_cannot_account_for_is_refused() {
        let legacy = BOARD.replace("(offset (xyz 1 2 3))", "(at (xyz 1 2 3))");
        let error = read(&legacy, "D1").unwrap_err();
        assert!(matches!(error, SavedModelError::Other(reason) if reason.contains("legacy")));
        let unknown = BOARD.replace("(opacity 0.5)", "(tint 0.5)");
        let error = read(&unknown, "D1").unwrap_err();
        assert!(matches!(error, SavedModelError::Other(reason) if reason.contains("'tint'")));
    }

    #[test]
    fn verify_refuses_a_candidate_that_touched_anything_else() {
        let prepared = prepare(BOARD, "U2", &IpcFootprint3DModelEdit::Remove { index: 1 }).unwrap();
        let tampered = prepared
            .candidate
            .replace("(property pad_prop_heatsink) ", "");
        assert!(verify(BOARD, &tampered, &prepared, "U2").is_err());
    }
}
