//! The saved board's own `(setup (stackup …))` block and `(general (thickness …))`.
//!
//! Read with the crate's S-expression reader and located with its byte-span
//! finders; nothing else in the board is interpreted. The grammar is KiCad
//! 10's: `PCB_IO_KICAD_SEXPR_PARSER::parseBoardStackup` reads it and
//! `BOARD_STACKUP::FormatBoardStackup` writes it. KiCad skips a clause it does
//! not know and drops it on its next save; this reader instead reports it, so
//! the setter can refuse to rewrite a block it cannot carry losslessly.

use konnect_ipc::{IpcBoardStackup, IpcStackupDielectric, IpcStackupLayer};
use konnect_sexp::writer::find_direct_child_blocks;
use konnect_sexp::SexpNode;
use serde_json::{json, Value};

use crate::tools::pcb_components::{check_single_board_form, sexp_block_tokens};

/// Where an entry sits in the physical stack.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Kind {
    Silkscreen,
    Paste,
    Mask,
    Copper,
    Dielectric,
}

impl Kind {
    pub(crate) fn as_str(self) -> &'static str {
        match self {
            Kind::Silkscreen => "silkscreen",
            Kind::Paste => "solderpaste",
            Kind::Mask => "soldermask",
            Kind::Copper => "copper",
            Kind::Dielectric => "dielectric",
        }
    }

    /// The kind KiCad's parser gives an entry name: a canonical copper name is
    /// copper, the six technical layers are what they name, any name that is
    /// not a board layer is a dielectric, and any other board layer is refused
    /// (`Expecting( "layer_name" )`).
    pub(crate) fn of_name(name: &str) -> Result<Self, String> {
        Ok(match name {
            "F.SilkS" | "B.SilkS" => Kind::Silkscreen,
            "F.Paste" | "B.Paste" => Kind::Paste,
            "F.Mask" | "B.Mask" => Kind::Mask,
            _ if konnect_sexp::layers::is_canonical_name(name) => {
                if konnect_sexp::layers::is_copper_name(name) {
                    Kind::Copper
                } else {
                    return Err(format!(
                        "stackup entry '{name}' names a board layer KiCad does not allow in a \
                         stackup"
                    ));
                }
            }
            _ => Kind::Dielectric,
        })
    }

    /// `IsThicknessEditable`: the entries KiCad writes and sums a thickness for.
    pub(crate) fn has_thickness(self) -> bool {
        matches!(self, Kind::Copper | Kind::Dielectric | Kind::Mask)
    }

    /// `IsMaterialEditable`, which is also `IsColorEditable`.
    pub(crate) fn has_material(self) -> bool {
        matches!(self, Kind::Dielectric | Kind::Mask | Kind::Silkscreen)
    }

    /// `HasEpsilonRValue` and `HasLossTangentValue`.
    pub(crate) fn has_dielectric_properties(self) -> bool {
        matches!(self, Kind::Dielectric | Kind::Mask)
    }
}

/// The type name KiCad's default stackup gives a new entry
/// (`BOARD_STACKUP::BuildDefaultStackupList`).
pub(crate) fn default_type_name(name: &str) -> Option<&'static str> {
    Some(match name {
        "F.SilkS" => "Top Silk Screen",
        "F.Paste" => "Top Solder Paste",
        "F.Mask" => "Top Solder Mask",
        "B.Mask" => "Bottom Solder Mask",
        "B.Paste" => "Bottom Solder Paste",
        "B.SilkS" => "Bottom Silk Screen",
        _ if konnect_sexp::layers::is_copper_name(name) => "copper",
        _ => return None,
    })
}

/// The type name `BOARD_STACKUP_ITEM`'s constructor gives an entry, which is
/// what KiCad holds for one read with no `(type …)`.
pub(crate) fn constructor_type_name(kind: Kind) -> &'static str {
    match kind {
        Kind::Copper => "copper",
        Kind::Dielectric => "core",
        Kind::Paste => "solderpaste",
        Kind::Mask => "soldermask",
        Kind::Silkscreen => "silkscreen",
    }
}

/// One physical ply. Every field is `None` when the file has no clause for it.
#[derive(Debug, Clone, PartialEq, Default)]
pub(crate) struct Ply {
    pub thickness_nm: Option<i64>,
    pub locked: bool,
    pub material: Option<String>,
    pub epsilon_r: Option<f64>,
    pub loss_tangent: Option<f64>,
    pub color: Option<String>,
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Entry {
    /// As written; a dielectric's is "dielectric N".
    pub name: String,
    pub kind: Kind,
    pub type_name: Option<String>,
    /// At least one. Only a dielectric has more.
    pub plies: Vec<Ply>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum EdgeConnector {
    None,
    Plain,
    Bevelled,
}

impl EdgeConnector {
    /// The live answer's vocabulary, which this tool reports in.
    pub(crate) fn as_str(self) -> &'static str {
        match self {
            EdgeConnector::None => "none",
            EdgeConnector::Plain => "plain",
            EdgeConnector::Bevelled => "beveled",
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Stackup {
    pub entries: Vec<Entry>,
    /// `None` when the file names no finish: KiCad writes the clause only for
    /// a specified finish.
    pub finish: Option<String>,
    pub dielectric_constraints: bool,
    pub edge_connector: EdgeConnector,
    pub edge_plating: bool,
}

/// What the saved board says, with the byte spans a setter may touch.
#[derive(Debug)]
pub(crate) struct SavedBoard {
    pub setup: (usize, usize),
    pub stackup_span: Option<(usize, usize)>,
    pub stackup: Option<Stackup>,
    /// Clauses KiCad would skip and drop on save, `entry: clause`.
    pub unrecognized: Vec<String>,
    pub general: Option<(usize, usize)>,
    pub nominal_span: Option<(usize, usize)>,
    pub nominal_thickness_mm: Option<f64>,
    pub layer_table: Vec<konnect_sexp::layers::Layer>,
}

/// The head atom of a `(head …)` block.
fn head(block: &str) -> &str {
    sexp_block_tokens(block)
        .first()
        .map_or("", |&(start, end)| &block[start..end])
}

/// The direct children of the first `(parent …)` in `source` whose head is `tag`.
fn children_tagged(source: &str, parent: &str, tag: &str) -> Vec<(usize, usize)> {
    find_direct_child_blocks(source, parent)
        .into_iter()
        .filter(|&(start, end)| head(&source[start..end]) == tag)
        .collect()
}

fn at_most_one(spans: Vec<(usize, usize)>, what: &str) -> Result<Option<(usize, usize)>, String> {
    match spans.len() {
        0 => Ok(None),
        1 => Ok(Some(spans[0])),
        n => Err(format!(
            "the board has {n} {what} clauses; one is required to know which KiCad reads"
        )),
    }
}

fn number(node: &SexpNode, index: usize, what: &str) -> Result<f64, String> {
    node.get_f64(index)
        .filter(|value| value.is_finite())
        .ok_or_else(|| format!("{what} is not a finite number"))
}

pub(crate) fn mm_to_nm(mm: f64) -> i64 {
    (mm * 1_000_000.0).round() as i64
}

pub(crate) fn nm_to_mm(nm: i64) -> f64 {
    nm as f64 / 1_000_000.0
}

/// Locate and read the saved stackup. Malformed or duplicated clauses are an
/// error; a clause KiCad would skip is reported in `unrecognized`.
pub(crate) fn read(source: &str) -> Result<SavedBoard, String> {
    check_single_board_form(source)?;
    let root = konnect_sexp::parse_sexp(source).map_err(|error| error.to_string())?;
    let setup = at_most_one(children_tagged(source, "kicad_pcb", "setup"), "(setup …)")?
        .ok_or("the board has no (setup …) clause")?;
    let setup_text = &source[setup.0..setup.1];
    let stackup_span = at_most_one(
        children_tagged(setup_text, "setup", "stackup"),
        "(setup (stackup …))",
    )?
    .map(|(start, end)| (setup.0 + start, setup.0 + end));
    let general = at_most_one(
        children_tagged(source, "kicad_pcb", "general"),
        "(general …)",
    )?;
    let (nominal_span, nominal_thickness_mm) = match general {
        None => (None, None),
        Some((start, end)) => {
            let text = &source[start..end];
            match at_most_one(
                children_tagged(text, "general", "thickness"),
                "(general (thickness …))",
            )? {
                None => (None, None),
                Some((a, b)) => {
                    let node = konnect_sexp::parse_sexp(&text[a..b]).map_err(|e| e.to_string())?;
                    if node.children().map_or(0, <[SexpNode]>::len) != 2 {
                        return Err("(general (thickness …)) must hold one number".to_string());
                    }
                    let mm = number(&node, 1, "(general (thickness …))")?;
                    (Some((start + a, start + b)), Some(mm))
                }
            }
        }
    };
    let mut unrecognized = Vec::new();
    let stackup = match stackup_span {
        None => None,
        Some((start, end)) => {
            let node = konnect_sexp::parse_sexp(&source[start..end]).map_err(|e| e.to_string())?;
            Some(parse_stackup(&node, &mut unrecognized)?)
        }
    };
    Ok(SavedBoard {
        setup,
        stackup_span,
        stackup,
        unrecognized,
        general,
        nominal_span,
        nominal_thickness_mm,
        layer_table: konnect_sexp::layers::layers(&root),
    })
}

fn render(node: &SexpNode) -> String {
    match node {
        SexpNode::Atom(atom) => atom.clone(),
        SexpNode::Str(text) => quote(text),
        SexpNode::List(items) => format!(
            "({})",
            items.iter().map(render).collect::<Vec<_>>().join(" ")
        ),
    }
}

fn single_string(node: &SexpNode, what: &str) -> Result<String, String> {
    match node.children() {
        Some([_, value]) => value
            .as_str()
            .map(str::to_string)
            .ok_or_else(|| format!("{what} must hold one string")),
        _ => Err(format!("{what} must hold one string")),
    }
}

fn yes_no(node: &SexpNode, what: &str) -> Result<bool, String> {
    match node.children() {
        Some([_, SexpNode::Atom(value)]) if value == "yes" => Ok(true),
        Some([_, SexpNode::Atom(value)]) if value == "no" => Ok(false),
        _ => Err(format!("{what} must be yes or no")),
    }
}

fn parse_stackup(node: &SexpNode, unrecognized: &mut Vec<String>) -> Result<Stackup, String> {
    let mut stackup = Stackup {
        entries: Vec::new(),
        finish: None,
        dielectric_constraints: false,
        edge_connector: EdgeConnector::None,
        edge_plating: false,
    };
    let mut seen = std::collections::BTreeSet::new();
    let mut once = |tag: &str| -> Result<(), String> {
        if seen.insert(tag.to_string()) {
            Ok(())
        } else {
            Err(format!("the stackup has more than one ({tag} …) clause"))
        }
    };
    for child in node.children().unwrap_or(&[]).iter().skip(1) {
        let Some(tag) = child.head() else {
            return Err(format!(
                "the stackup holds '{}' outside any clause",
                render(child)
            ));
        };
        match tag {
            "layer" => stackup.entries.push(parse_entry(child, unrecognized)?),
            "copper_finish" => {
                once(tag)?;
                stackup.finish = Some(single_string(child, "(copper_finish …)")?);
            }
            "dielectric_constraints" => {
                once(tag)?;
                stackup.dielectric_constraints = yes_no(child, "(dielectric_constraints …)")?;
            }
            "edge_plating" => {
                once(tag)?;
                stackup.edge_plating = yes_no(child, "(edge_plating …)")?;
            }
            "edge_connector" => {
                once(tag)?;
                stackup.edge_connector = match child.children() {
                    Some([_, SexpNode::Atom(value)]) if value == "yes" => EdgeConnector::Plain,
                    Some([_, SexpNode::Atom(value)]) if value == "bevelled" => {
                        EdgeConnector::Bevelled
                    }
                    _ => return Err("(edge_connector …) must be yes or bevelled".to_string()),
                };
            }
            _ => unrecognized.push(format!("stackup: {}", render(child))),
        }
    }
    let mut names = std::collections::BTreeSet::new();
    for entry in &stackup.entries {
        if !names.insert(entry.name.clone()) {
            return Err(format!(
                "the stackup names '{}' more than once; KiCad would drop every entry after it",
                entry.name
            ));
        }
    }
    Ok(stackup)
}

fn parse_entry(node: &SexpNode, unrecognized: &mut Vec<String>) -> Result<Entry, String> {
    let items = node.children().unwrap_or(&[]);
    let name = match items.get(1) {
        Some(SexpNode::Str(name)) | Some(SexpNode::Atom(name)) => name.clone(),
        _ => return Err("a stackup (layer …) has no name".to_string()),
    };
    let kind = Kind::of_name(&name)?;
    let mut entry = Entry {
        name: name.clone(),
        kind,
        type_name: None,
        plies: vec![Ply::default()],
    };
    let mut seen = std::collections::BTreeSet::new();
    for item in items.iter().skip(2) {
        if let SexpNode::Atom(atom) = item {
            if atom == "addsublayer" {
                if kind != Kind::Dielectric {
                    return Err(format!("'{name}' is not a dielectric but adds a sub-layer"));
                }
                entry.plies.push(Ply::default());
                seen.clear();
                continue;
            }
            return Err(format!("'{name}' holds the stray token '{atom}'"));
        }
        let Some(tag) = item.head() else {
            return Err(format!("'{name}' holds a stray string"));
        };
        if tag != "type" && !seen.insert(tag.to_string()) {
            return Err(format!(
                "'{name}' has more than one ({tag} …) for one sub-layer"
            ));
        }
        let ply = entry.plies.last_mut().expect("at least one ply");
        let what = format!("'{name}' ({tag} …)");
        match tag {
            "type" => {
                if entry.type_name.is_some() {
                    return Err(format!("'{name}' has more than one (type …)"));
                }
                entry.type_name = Some(single_string(item, &what)?);
            }
            "thickness" => {
                let mm = number(item, 1, &what)?;
                ply.thickness_nm = Some(mm_to_nm(mm));
                match item.children().map(|c| &c[2..]) {
                    Some([]) => {}
                    Some([SexpNode::Atom(flag)]) if flag == "locked" => {
                        // KiCad keeps a lock only on a dielectric.
                        if kind == Kind::Dielectric {
                            ply.locked = true;
                        } else {
                            unrecognized.push(format!("{name}: {}", render(item)));
                        }
                    }
                    _ => return Err(format!("{what} is malformed")),
                }
            }
            "material" => ply.material = Some(single_string(item, &what)?),
            "color" => ply.color = Some(single_string(item, &what)?),
            "epsilon_r" | "loss_tangent" => {
                if item.children().map_or(0, <[SexpNode]>::len) != 2 {
                    return Err(format!("{what} must hold one number"));
                }
                let value = number(item, 1, &what)?;
                if tag == "epsilon_r" {
                    ply.epsilon_r = Some(value);
                } else {
                    ply.loss_tangent = Some(value);
                }
            }
            _ => unrecognized.push(format!("{name}: {}", render(item))),
        }
    }
    Ok(entry)
}

// ─── Writing ─────────────────────────────────────────────────────────────────

pub(crate) fn quote(text: &str) -> String {
    format!("\"{}\"", text.replace('\\', "\\\\").replace('"', "\\\""))
}

/// KiCad's `FormatInternalUnits` for millimetres: nanometre precision, no
/// trailing zeros.
pub(crate) fn format_mm(nm: i64) -> String {
    let text = format!("{:.6}", nm_to_mm(nm));
    let text = text.trim_end_matches('0').trim_end_matches('.');
    if text == "-0" {
        "0".to_string()
    } else {
        text.to_string()
    }
}

/// A double as `FormatDouble2Str` writes one: shortest, never exponential.
pub(crate) fn format_double(value: f64) -> String {
    format!("{value}")
}

fn specified(value: &Option<String>) -> Option<&str> {
    value.as_deref().filter(|text| !super::unspecified(text))
}

/// The block exactly as `BOARD_STACKUP::FormatBoardStackup` orders it, laid out
/// as KiCad 10 lays it out: one clause per line, `indent` before the block's
/// own closing paren and one `unit` deeper for each level inside it.
pub(crate) fn format(stackup: &Stackup, indent: &str, unit: &str, eol: &str) -> String {
    let inner = format!("{indent}{unit}");
    let deeper = format!("{inner}{unit}");
    let mut out = format!("(stackup{eol}");
    for entry in &stackup.entries {
        out.push_str(&format!("{inner}(layer {}{eol}", quote(&entry.name)));
        // An entry read without a type has the one KiCad's constructor gives it.
        let type_name = entry
            .type_name
            .clone()
            .unwrap_or_else(|| constructor_type_name(entry.kind).to_string());
        out.push_str(&format!("{deeper}(type {}){eol}", quote(&type_name)));
        for (index, ply) in entry.plies.iter().enumerate() {
            if index > 0 {
                out.push_str(&format!("{deeper}addsublayer{eol}"));
            }
            if entry.kind.has_material() {
                if let Some(color) = specified(&ply.color) {
                    out.push_str(&format!("{deeper}(color {}){eol}", quote(color)));
                }
            }
            if entry.kind.has_thickness() {
                let locked = if entry.kind == Kind::Dielectric && ply.locked {
                    " locked"
                } else {
                    ""
                };
                out.push_str(&format!(
                    "{deeper}(thickness {}{locked}){eol}",
                    format_mm(ply.thickness_nm.unwrap_or(0))
                ));
            }
            let material = entry
                .kind
                .has_material()
                .then(|| specified(&ply.material))
                .flatten();
            if let Some(material) = material {
                out.push_str(&format!("{deeper}(material {}){eol}", quote(material)));
                if entry.kind.has_dielectric_properties() {
                    if let Some(er) = ply.epsilon_r {
                        out.push_str(&format!("{deeper}(epsilon_r {}){eol}", format_double(er)));
                    }
                    if let Some(tan) = ply.loss_tangent {
                        out.push_str(&format!(
                            "{deeper}(loss_tangent {}){eol}",
                            format_double(tan)
                        ));
                    }
                }
            }
        }
        out.push_str(&format!("{inner}){eol}"));
    }
    if let Some(finish) = specified(&stackup.finish) {
        out.push_str(&format!("{inner}(copper_finish {}){eol}", quote(finish)));
    }
    out.push_str(&format!(
        "{inner}(dielectric_constraints {}){eol}",
        if stackup.dielectric_constraints {
            "yes"
        } else {
            "no"
        }
    ));
    match stackup.edge_connector {
        EdgeConnector::None => {}
        EdgeConnector::Plain => out.push_str(&format!("{inner}(edge_connector yes){eol}")),
        EdgeConnector::Bevelled => out.push_str(&format!("{inner}(edge_connector bevelled){eol}")),
    }
    if stackup.edge_plating {
        out.push_str(&format!("{inner}(edge_plating yes){eol}"));
    }
    out.push_str(&format!("{indent})"));
    out
}

/// The whitespace before the line holding `offset`, up to the offset.
pub(crate) fn line_indent(source: &str, offset: usize) -> &str {
    let line_start = source[..offset].rfind('\n').map_or(0, |at| at + 1);
    let prefix = &source[line_start..offset];
    if prefix.chars().all(|ch| ch == ' ' || ch == '\t') {
        prefix
    } else {
        ""
    }
}

pub(crate) fn indent_unit(indent: &str) -> &'static str {
    if indent.contains(' ') && !indent.contains('\t') {
        "  "
    } else {
        "\t"
    }
}

pub(crate) fn eol(source: &str) -> &'static str {
    if source.contains("\r\n") {
        "\r\n"
    } else {
        "\n"
    }
}

// ─── Views ───────────────────────────────────────────────────────────────────

/// The stack sum KiCad derives (`BuildBoardThicknessFromStackup`): every
/// copper, dielectric and mask ply. Distinct from the nominal thickness.
pub(crate) fn stack_thickness_nm(stackup: &Stackup) -> i64 {
    stackup
        .entries
        .iter()
        .filter(|entry| entry.kind.has_thickness())
        .flat_map(|entry| &entry.plies)
        .map(|ply| ply.thickness_nm.unwrap_or(0))
        .sum()
}

/// As the live answer's crate type, for the shared findings and comparisons.
/// A value the file does not carry is NaN or empty, never a default, and a
/// saved entry is always built (`enabled`).
pub(crate) fn to_ipc(stackup: &Stackup) -> IpcBoardStackup {
    let layers = stackup
        .entries
        .iter()
        .map(|entry| {
            let first = &entry.plies[0];
            let dielectric = entry.kind == Kind::Dielectric;
            IpcStackupLayer {
                layer: (!dielectric).then(|| entry.name.clone()),
                user_name: None,
                kind: entry.kind.as_str().to_string(),
                enabled: Some(true),
                thickness_nm: first.thickness_nm.unwrap_or(0),
                material: if dielectric {
                    String::new()
                } else {
                    first.material.clone().unwrap_or_default()
                },
                color: None,
                dielectric_type: dielectric.then(|| entry.type_name.clone()).flatten(),
                dielectric: if dielectric {
                    entry
                        .plies
                        .iter()
                        .map(|ply| IpcStackupDielectric {
                            thickness_nm: ply.thickness_nm.unwrap_or(0),
                            material: ply.material.clone().unwrap_or_default(),
                            epsilon_r: ply.epsilon_r.unwrap_or(f64::NAN),
                            loss_tangent: ply.loss_tangent.unwrap_or(f64::NAN),
                            thickness_locked: ply.locked,
                        })
                        .collect()
                } else {
                    Vec::new()
                },
                epsilon_r: (entry.kind == Kind::Mask)
                    .then_some(first.epsilon_r)
                    .flatten(),
                loss_tangent: (entry.kind == Kind::Mask)
                    .then_some(first.loss_tangent)
                    .flatten(),
            }
        })
        .collect();
    IpcBoardStackup {
        finish: stackup.finish.clone(),
        impedance_controlled: Some(stackup.dielectric_constraints),
        edge_connector: Some(stackup.edge_connector.as_str().to_string()),
        has_edge_plating: Some(stackup.edge_plating),
        layers,
    }
}

fn ply_json(kind: Kind, ply: &Ply) -> Value {
    let mut out = json!({});
    if kind.has_thickness() {
        out["thickness_mm"] = json!(ply.thickness_nm.map(nm_to_mm));
    }
    if kind == Kind::Dielectric {
        out["thickness_locked"] = json!(ply.locked);
    }
    if kind.has_material() {
        out["material"] = json!(ply.material);
        out["color"] = json!(ply.color);
    }
    if kind.has_dielectric_properties() {
        out["epsilon_r"] = json!(ply.epsilon_r);
        out["loss_tangent"] = json!(ply.loss_tangent);
    }
    out
}

/// One saved entry, every value as the file states it (`null` when absent).
pub(crate) fn entry_json(entry: &Entry) -> Value {
    let mut out = json!({
        "name": entry.name,
        "type": entry.kind.as_str(),
        "type_name": entry.type_name,
        "enabled": true,
    });
    if entry.kind == Kind::Dielectric {
        out["dielectric_type"] = json!(entry.type_name);
        out["thickness_mm"] = json!(entry
            .plies
            .iter()
            .map(|ply| ply.thickness_nm)
            .sum::<Option<i64>>()
            .map(nm_to_mm));
        out["sublayers"] = json!(entry
            .plies
            .iter()
            .map(|ply| ply_json(entry.kind, ply))
            .collect::<Vec<_>>());
    } else if let Some(fields) = ply_json(entry.kind, &entry.plies[0]).as_object() {
        for (key, value) in fields {
            out[key] = value.clone();
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const FOUR_LAYER: &str =
        include_str!("../../../../konnect-ipc/tests/fixtures/four_layer_stackup_kicad10.kicad_pcb");

    #[test]
    fn kicads_own_four_layer_file_reads_and_rewrites_to_its_own_bytes() {
        let saved = read(FOUR_LAYER).unwrap();
        let stackup = saved.stackup.as_ref().unwrap();
        assert!(saved.unrecognized.is_empty(), "{:?}", saved.unrecognized);
        assert_eq!(stackup.entries.len(), 13);
        assert_eq!(stackup.finish.as_deref(), Some("ENIG"));
        assert!(stackup.dielectric_constraints);
        assert_eq!(stackup.edge_connector, EdgeConnector::Bevelled);
        assert!(stackup.edge_plating);
        assert_eq!(saved.nominal_thickness_mm, Some(0.7912));
        assert_eq!(stack_thickness_nm(stackup), 791_200);

        // KiCad's writer and this one agree byte for byte on KiCad's own file.
        let (start, end) = saved.stackup_span.unwrap();
        let indent = line_indent(FOUR_LAYER, start);
        let written = format(stackup, indent, indent_unit(indent), eol(FOUR_LAYER));
        assert_eq!(written, &FOUR_LAYER[start..end]);
    }

    #[test]
    fn duplicates_and_malformed_clauses_are_refused_and_unknown_ones_reported() {
        let dup_setup = FOUR_LAYER.replacen("\t(setup\n", "\t(setup\n\t)\n\t(setup\n", 1);
        assert!(read(&dup_setup).unwrap_err().contains("(setup …)"));
        let dup_finish = FOUR_LAYER.replace(
            "(copper_finish \"ENIG\")",
            "(copper_finish \"ENIG\") (copper_finish \"OSP\")",
        );
        assert!(read(&dup_finish).unwrap_err().contains("copper_finish"));
        let dup_thickness =
            FOUR_LAYER.replace("(thickness 0.25)", "(thickness 0.25) (thickness 0.3)");
        assert!(read(&dup_thickness).unwrap_err().contains("more than one"));
        let bad_number = FOUR_LAYER.replace("(thickness 0.25)", "(thickness abc)");
        assert!(read(&bad_number).unwrap_err().contains("finite"));
        let dup_name = FOUR_LAYER.replace("\"dielectric 3\"", "\"dielectric 1\"");
        assert!(read(&dup_name).unwrap_err().contains("more than once"));
        let edge_layer = FOUR_LAYER.replace("(layer \"F.SilkS\"", "(layer \"Edge.Cuts\"");
        assert!(read(&edge_layer).is_err());
        let dup_general = FOUR_LAYER.replace(
            "\t\t(thickness 0.7912)\n",
            "\t\t(thickness 0.7912)\n\t\t(thickness 1.6)\n",
        );
        assert!(read(&dup_general).unwrap_err().contains("general"));

        let unknown = FOUR_LAYER.replace(
            "(epsilon_r 4.23)",
            "(epsilon_r 4.23)\n\t\t\t\t(future_clause 7)",
        );
        let saved = read(&unknown).unwrap();
        assert_eq!(saved.unrecognized, ["dielectric 2: (future_clause 7)"]);
    }
}
