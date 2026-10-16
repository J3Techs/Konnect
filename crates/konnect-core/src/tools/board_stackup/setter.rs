//! Planning a closed-board stackup edit: the requested physical stack, checked
//! against the board's enabled layer table and merged over the saved stackup,
//! written back as one `(setup (stackup …))` span and, only when asked, the
//! `(general (thickness …))` span. Pure: no I/O.

use serde_json::{json, Value};

use super::saved::{
    self, default_type_name, entry_json, format, format_mm, mm_to_nm, nm_to_mm, stack_thickness_nm,
    EdgeConnector, Entry, Kind, Ply, SavedBoard, Stackup,
};
use crate::tools::closed_board::only_spans_changed;

#[derive(Debug, Clone, Default, PartialEq)]
pub(crate) struct PlyRequest {
    pub thickness_nm: Option<i64>,
    pub material: Option<String>,
    pub epsilon_r: Option<f64>,
    pub loss_tangent: Option<f64>,
    pub thickness_locked: Option<bool>,
    pub color: Option<String>,
}

impl PlyRequest {
    fn is_empty(&self) -> bool {
        *self == PlyRequest::default()
    }

    fn json(&self) -> Value {
        json!({
            "thickness_nm": self.thickness_nm,
            "material": self.material,
            "epsilon_r": self.epsilon_r,
            "loss_tangent": self.loss_tangent,
            "thickness_locked": self.thickness_locked,
            "color": self.color,
        })
    }
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct EntryRequest {
    pub name: String,
    pub dielectric_type: Option<String>,
    pub ply: PlyRequest,
    pub sublayers: Option<Vec<PlyRequest>>,
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Request {
    pub layers: Vec<EntryRequest>,
    pub finish: Option<String>,
    pub impedance_controlled: Option<bool>,
    pub edge_connector: Option<EdgeConnector>,
    pub edge_plating: Option<bool>,
    pub nominal_board_thickness_nm: Option<i64>,
}

impl Request {
    /// The request as planned, for the revision.
    pub(crate) fn normalized(&self) -> Value {
        json!({
            "layers": self.layers.iter().map(|layer| json!({
                "name": layer.name,
                "dielectric_type": layer.dielectric_type,
                "ply": layer.ply.json(),
                "sublayers": layer.sublayers.as_ref().map(|plies| {
                    plies.iter().map(PlyRequest::json).collect::<Vec<_>>()
                }),
            })).collect::<Vec<_>>(),
            "finish": self.finish,
            "impedance_controlled": self.impedance_controlled,
            "edge_connector": self.edge_connector.map(EdgeConnector::as_str),
            "edge_plating": self.edge_plating,
            "nominal_board_thickness_nm": self.nominal_board_thickness_nm,
        })
    }
}

// ─── Planning ────────────────────────────────────────────────────────────────

pub(crate) struct Plan {
    pub candidate: String,
    pub before: Option<Stackup>,
    pub target: Stackup,
    pub nominal_before_mm: Option<f64>,
    pub nominal_after_mm: Option<f64>,
    pub edits: Vec<(usize, usize, usize)>,
    pub differences: Vec<Value>,
    pub warnings: Vec<String>,
}

impl Plan {
    pub(crate) fn changed(&self) -> bool {
        !self.edits.is_empty()
    }
}

/// Physical position of a copper layer: F.Cu, then In1.Cu … In30.Cu, then
/// B.Cu. KiCad's layer IDs are not this order (F.Cu 0, B.Cu 2, In1.Cu 4 …).
fn copper_rank(name: &str) -> Option<u32> {
    match name {
        "F.Cu" => Some(0),
        "B.Cu" => Some(u32::MAX),
        _ => name
            .strip_prefix("In")
            .and_then(|rest| rest.strip_suffix(".Cu"))
            .and_then(|n| n.parse().ok())
            .filter(|n| (1..=30).contains(n)),
    }
}

/// The request's names, checked as a physical stack against the board's
/// enabled layer table.
fn check_structure(request: &Request, saved: &SavedBoard) -> Result<Vec<String>, Vec<String>> {
    let mut reasons = Vec::new();
    let names: Vec<&str> = request.layers.iter().map(|l| l.name.as_str()).collect();
    let mut kinds = Vec::new();
    for name in &names {
        match Kind::of_name(name) {
            Ok(kind) => kinds.push(kind),
            Err(reason) => {
                reasons.push(reason);
                kinds.push(Kind::Dielectric);
            }
        }
    }
    let mut seen = std::collections::BTreeSet::new();
    for name in &names {
        if !seen.insert(*name) {
            reasons.push(format!("'{name}' appears more than once"));
        }
    }
    let table: std::collections::BTreeSet<&str> =
        saved.layer_table.iter().map(|l| l.name.as_str()).collect();
    let mut table_copper: Vec<&str> = saved
        .layer_table
        .iter()
        .filter(|l| l.is_copper())
        .map(|l| l.name.as_str())
        .collect();
    if let Some(odd) = table_copper.iter().find(|name| copper_rank(name).is_none()) {
        reasons.push(format!(
            "the board's layer table has an unplaceable copper layer '{odd}'"
        ));
    }
    table_copper.sort_by_key(|name| copper_rank(name));
    let requested_copper: Vec<&str> = names
        .iter()
        .zip(&kinds)
        .filter(|(_, kind)| **kind == Kind::Copper)
        .map(|(name, _)| *name)
        .collect();
    if requested_copper != table_copper {
        reasons.push(format!(
            "the stackup's copper must be the board's enabled copper layers in physical order \
             ({}); the request has ({})",
            table_copper.join(", "),
            requested_copper.join(", ")
        ));
    }
    if table_copper.len() < 2 {
        reasons.push("a KiCad board needs at least two copper layers".to_string());
    }
    for (name, kind) in names.iter().zip(&kinds) {
        if matches!(kind, Kind::Mask | Kind::Silkscreen | Kind::Paste) && !table.contains(name) {
            reasons.push(format!("'{name}' is not an enabled layer of this board"));
        }
    }

    // [F.SilkS] [F.Paste] [F.Mask] F.Cu (dielectric copper)+ [B.Mask] [B.Paste] [B.SilkS]
    let first_cu = names.iter().position(|n| *n == "F.Cu");
    let last_cu = names.iter().position(|n| *n == "B.Cu");
    match (first_cu, last_cu) {
        (Some(first), Some(last)) if first < last => {
            let top = ["F.SilkS", "F.Paste", "F.Mask"];
            let bottom = ["B.Mask", "B.Paste", "B.SilkS"];
            let ordered_subset = |part: &[&str], order: &[&str]| {
                let mut at = 0;
                part.iter()
                    .all(|name| match order[at..].iter().position(|o| o == name) {
                        Some(offset) => {
                            at += offset + 1;
                            true
                        }
                        None => false,
                    })
            };
            if !ordered_subset(&names[..first], &top) {
                reasons.push(
                    "above F.Cu only F.SilkS, F.Paste and F.Mask may appear, in that order"
                        .to_string(),
                );
            }
            if !ordered_subset(&names[last + 1..], &bottom) {
                reasons.push(
                    "below B.Cu only B.Mask, B.Paste and B.SilkS may appear, in that order"
                        .to_string(),
                );
            }
            for (offset, kind) in kinds[first..=last].iter().enumerate() {
                let want = if offset % 2 == 0 {
                    Kind::Copper
                } else {
                    Kind::Dielectric
                };
                if *kind != want {
                    reasons.push(format!(
                        "from F.Cu to B.Cu the stack must alternate copper and one dielectric; \
                         '{}' is out of place",
                        names[first + offset]
                    ));
                    break;
                }
            }
        }
        _ => reasons.push("the stack must run from F.Cu above to B.Cu below".to_string()),
    }
    let mut ordinal = 0;
    for (name, kind) in names.iter().zip(&kinds) {
        if *kind == Kind::Dielectric {
            ordinal += 1;
            if *name != format!("dielectric {ordinal}") {
                reasons.push(format!(
                    "dielectrics are named in stack order as KiCad writes them; '{name}' must \
                     be 'dielectric {ordinal}'"
                ));
            }
        }
    }
    if !reasons.is_empty() {
        return Err(reasons);
    }
    let warnings = saved
        .layer_table
        .iter()
        .filter(|l| {
            matches!(
                l.name.as_str(),
                "F.Mask" | "B.Mask" | "F.SilkS" | "B.SilkS" | "F.Paste" | "B.Paste"
            )
        })
        .filter(|l| !names.contains(&l.name.as_str()))
        .map(|l| format!("enabled layer '{}' has no stackup entry", l.name))
        .collect();
    Ok(warnings)
}

/// The saved dielectric between the same two copper layers, which is the
/// one a requested dielectric inherits unrequested values from.
fn saved_dielectric<'a>(
    before: Option<&'a Stackup>,
    above: &str,
    below: &str,
) -> Option<&'a Entry> {
    let entries = &before?.entries;
    entries.windows(3).find_map(|w| {
        (w[0].name == above && w[1].kind == Kind::Dielectric && w[2].name == below).then_some(&w[1])
    })
}

fn merge_ply(
    base: Option<&Ply>,
    want: &PlyRequest,
    kind: Kind,
    at: &str,
    reasons: &mut Vec<String>,
) -> Ply {
    let mut ply = base.cloned().unwrap_or_default();
    if let Some(nm) = want.thickness_nm {
        ply.thickness_nm = Some(nm);
    }
    if let Some(material) = &want.material {
        ply.material = Some(material.clone());
    }
    if let Some(er) = want.epsilon_r {
        ply.epsilon_r = Some(er);
    }
    if let Some(tan) = want.loss_tangent {
        ply.loss_tangent = Some(tan);
    }
    if let Some(locked) = want.thickness_locked {
        ply.locked = locked;
    }
    if let Some(color) = &want.color {
        ply.color = Some(color.clone());
    }
    if base.is_none() {
        let mut need = |missing: bool, field: &str| {
            if missing {
                reasons.push(format!("{at} is new and needs {field}"));
            }
        };
        if kind.has_thickness() {
            need(ply.thickness_nm.is_none(), "thickness_mm");
        }
        if kind == Kind::Dielectric {
            need(ply.material.is_none(), "material");
            need(ply.epsilon_r.is_none(), "epsilon_r");
            need(ply.loss_tangent.is_none(), "loss_tangent");
        }
    }
    if kind.has_dielectric_properties()
        && (ply.epsilon_r.is_some() || ply.loss_tangent.is_some())
        && ply.material.as_deref().is_none_or(super::unspecified)
    {
        reasons.push(format!(
            "{at}: KiCad writes εr and loss tangent only beside a specified material"
        ));
    }
    ply
}

fn refuse_fields(want: &EntryRequest, allowed: &[&str], kind: Kind, reasons: &mut Vec<String>) {
    let given = [
        ("thickness_mm", want.ply.thickness_nm.is_some()),
        ("material", want.ply.material.is_some()),
        ("epsilon_r", want.ply.epsilon_r.is_some()),
        ("loss_tangent", want.ply.loss_tangent.is_some()),
        ("thickness_locked", want.ply.thickness_locked.is_some()),
        ("color", want.ply.color.is_some()),
        ("type", want.dielectric_type.is_some()),
        ("sublayers", want.sublayers.is_some()),
    ];
    for (field, present) in given {
        if present && !allowed.contains(&field) {
            reasons.push(format!(
                "'{}' is a {} entry, which KiCad gives no {field}",
                want.name,
                kind.as_str()
            ));
        }
    }
}

fn target_entry(
    want: &EntryRequest,
    kind: Kind,
    base: Option<&Entry>,
    reasons: &mut Vec<String>,
) -> Entry {
    let at = format!("'{}'", want.name);
    let allowed: &[&str] = match kind {
        Kind::Copper => &["thickness_mm"],
        Kind::Mask => &[
            "thickness_mm",
            "material",
            "epsilon_r",
            "loss_tangent",
            "color",
        ],
        Kind::Silkscreen => &["material", "color"],
        Kind::Paste => &[],
        Kind::Dielectric => &PLY_FIELDS_AND_SHAPE,
    };
    refuse_fields(want, allowed, kind, reasons);
    let type_name = match (kind, &want.dielectric_type, base) {
        (Kind::Dielectric, Some(kind), _) => Some(kind.clone()),
        (_, _, Some(base)) => Some(
            base.type_name
                .clone()
                .unwrap_or_else(|| saved::constructor_type_name(kind).to_string()),
        ),
        (Kind::Dielectric, None, None) => {
            reasons.push(format!("{at} is new and needs type 'core' or 'prepreg'"));
            None
        }
        (_, _, None) => default_type_name(&want.name).map(str::to_string),
    };
    let plies = if kind != Kind::Dielectric {
        vec![merge_ply(
            base.map(|b| &b.plies[0]),
            &want.ply,
            kind,
            &at,
            reasons,
        )]
    } else {
        match (&want.sublayers, want.ply.is_empty(), base) {
            (Some(_), false, _) => {
                reasons.push(format!(
                    "{at}: give its values either per sub-layer or for the slot, not both"
                ));
                Vec::new()
            }
            (Some(plies), true, _) => plies
                .iter()
                .enumerate()
                .map(|(index, ply)| {
                    merge_ply(
                        base.and_then(|b| b.plies.get(index)),
                        ply,
                        kind,
                        &format!("{at} sub-layer {}", index + 1),
                        reasons,
                    )
                })
                .collect(),
            // Named and nothing else: every saved ply stands.
            (None, true, Some(base)) => base.plies.clone(),
            (None, false, Some(base)) if base.plies.len() > 1 => {
                reasons.push(format!(
                    "{at} has {} saved sub-layers; give 'sublayers' to say which ply each value \
                     belongs to",
                    base.plies.len()
                ));
                Vec::new()
            }
            (None, _, base) => vec![merge_ply(
                base.map(|b| &b.plies[0]),
                &want.ply,
                kind,
                &at,
                reasons,
            )],
        }
    };
    Entry {
        name: want.name.clone(),
        kind,
        type_name,
        plies,
    }
}

const PLY_FIELDS_AND_SHAPE: [&str; 8] = [
    "thickness_mm",
    "material",
    "epsilon_r",
    "loss_tangent",
    "thickness_locked",
    "color",
    "type",
    "sublayers",
];

fn differences(before: Option<&Stackup>, target: &Stackup) -> Vec<Value> {
    let mut out = Vec::new();
    let old: Vec<&Entry> = before
        .map(|s| s.entries.iter().collect())
        .unwrap_or_default();
    for entry in &target.entries {
        match old
            .iter()
            .find(|o| o.name == entry.name && o.kind == entry.kind)
        {
            None => out
                .push(json!({ "name": entry.name, "change": "added", "after": entry_json(entry) })),
            Some(o) => {
                let (a, b) = (entry_json(o), entry_json(entry));
                for (key, value) in b.as_object().into_iter().flatten() {
                    if a.get(key) != Some(value) {
                        out.push(json!({
                            "name": entry.name, "field": key,
                            "before": a.get(key), "after": value,
                        }));
                    }
                }
            }
        }
    }
    for o in &old {
        if !target
            .entries
            .iter()
            .any(|e| e.name == o.name && e.kind == o.kind)
        {
            out.push(json!({ "name": o.name, "change": "removed" }));
        }
    }
    let board = |s: &Stackup| {
        json!({
            "finish": s.finish,
            "impedance_controlled": s.dielectric_constraints,
            "edge_connector": s.edge_connector.as_str(),
            "edge_plating": s.edge_plating,
        })
    };
    let a = before.map(board).unwrap_or(json!({}));
    let b = board(target);
    for (key, value) in b.as_object().into_iter().flatten() {
        if before.is_none() || a.get(key) != Some(value) {
            out.push(
                json!({ "name": "board", "field": key, "before": a.get(key), "after": value }),
            );
        }
    }
    out
}

/// Plan the edit. `Err` carries every reason it was refused.
pub(crate) fn plan(original: &str, request: &Request) -> Result<Plan, Vec<String>> {
    let saved = saved::read(original).map_err(|reason| vec![reason])?;
    if !saved.unrecognized.is_empty() {
        return Err(vec![format!(
            "the saved stackup holds clauses KiCad 10 does not know and would drop on save, so \
             Konnect will not rewrite it: {}",
            saved.unrecognized.join("; ")
        )]);
    }
    let warnings = check_structure(request, &saved)?;
    let before = saved.stackup.as_ref();
    let mut reasons = Vec::new();
    let mut entries = Vec::with_capacity(request.layers.len());
    for (index, want) in request.layers.iter().enumerate() {
        let kind = Kind::of_name(&want.name).expect("checked");
        let base = if kind == Kind::Dielectric {
            saved_dielectric(
                before,
                &request.layers[index - 1].name,
                &request.layers[index + 1].name,
            )
        } else {
            before.and_then(|s| {
                s.entries
                    .iter()
                    .find(|e| e.name == want.name && e.kind == kind)
            })
        };
        entries.push(target_entry(want, kind, base, &mut reasons));
    }
    let target = Stackup {
        entries,
        finish: match &request.finish {
            Some(finish) if super::unspecified(finish) => None,
            Some(finish) => Some(finish.clone()),
            None => before.and_then(|s| s.finish.clone()),
        },
        dielectric_constraints: request
            .impedance_controlled
            .or(before.map(|s| s.dielectric_constraints))
            .unwrap_or(false),
        edge_connector: request
            .edge_connector
            .or(before.map(|s| s.edge_connector))
            .unwrap_or(EdgeConnector::None),
        edge_plating: request
            .edge_plating
            .or(before.map(|s| s.edge_plating))
            .unwrap_or(false),
    };
    if !reasons.is_empty() {
        return Err(reasons);
    }

    let eol = saved::eol(original);
    let (indent, edit_span) = match saved.stackup_span {
        Some((start, end)) => (
            saved::line_indent(original, start).to_string(),
            (start, end),
        ),
        None => {
            let setup_indent = saved::line_indent(original, saved.setup.0);
            let unit = saved::indent_unit(setup_indent);
            let after_head = saved.setup.0 + "(setup".len();
            (format!("{setup_indent}{unit}"), (after_head, after_head))
        }
    };
    let unit = saved::indent_unit(&indent);
    let block = format(&target, &indent, unit, eol);
    // What cannot be written cannot be planned: the written block must read
    // back as exactly the target.
    let reread = saved::read(&format!("(kicad_pcb (setup {block}))"))
        .ok()
        .filter(|s| s.unrecognized.is_empty())
        .and_then(|s| s.stackup);
    if reread.as_ref() != Some(&target) {
        return Err(vec![
            "the requested stackup cannot be written in KiCad's grammar without changing a value"
                .to_string(),
        ]);
    }

    let mut edits = Vec::new();
    let mut changes = Vec::new();
    if before != Some(&target) {
        let replacement = if saved.stackup_span.is_some() {
            block
        } else {
            format!("{eol}{indent}{block}")
        };
        edits.push((edit_span.0, edit_span.1, replacement.len()));
        changes.push(konnect_sexp::SexpEdit::replace(
            edit_span.0,
            edit_span.1,
            replacement,
        ));
    }
    let nominal_before_mm = saved.nominal_thickness_mm;
    let nominal_after_mm = match request.nominal_board_thickness_nm {
        None => nominal_before_mm,
        Some(nm) => {
            let wanted = nm_to_mm(nm);
            let same = nominal_before_mm.is_some_and(|mm| mm_to_nm(mm) == nm);
            if !same {
                let clause = format!("(thickness {})", format_mm(nm));
                match (saved.nominal_span, saved.general) {
                    (Some((start, end)), _) => {
                        edits.push((start, end, clause.len()));
                        changes.push(konnect_sexp::SexpEdit::replace(start, end, clause));
                    }
                    (None, Some((start, _))) => {
                        let general_indent = saved::line_indent(original, start);
                        let text = format!(
                            "{eol}{general_indent}{}{clause}",
                            saved::indent_unit(general_indent)
                        );
                        let at = start + "(general".len();
                        edits.push((at, at, text.len()));
                        changes.push(konnect_sexp::SexpEdit::insert(at, text));
                    }
                    (None, None) => {
                        return Err(vec![
                            "the board has no (general …) clause to hold a nominal thickness"
                                .to_string(),
                        ])
                    }
                }
            }
            Some(wanted)
        }
    };
    let candidate = konnect_sexp::apply_edits(original.to_string(), changes);
    if let Err(reason) = verify(original, &candidate, &edits, &target, nominal_after_mm) {
        return Err(vec![reason]);
    }
    Ok(Plan {
        candidate,
        differences: differences(before, &target),
        before: saved.stackup,
        target,
        nominal_before_mm,
        nominal_after_mm,
        edits,
        warnings,
    })
}

/// The planned (or persisted) board holds exactly the target stackup and
/// nominal thickness, and differs from the original only inside the edits.
pub(crate) fn verify(
    original: &str,
    candidate: &str,
    edits: &[(usize, usize, usize)],
    target: &Stackup,
    nominal_mm: Option<f64>,
) -> Result<Value, String> {
    if !only_spans_changed(original, candidate, edits) {
        return Err("bytes outside the stackup and nominal-thickness spans changed".to_string());
    }
    let saved = saved::read(candidate)?;
    if saved.stackup.as_ref() != Some(target) {
        return Err("the stackup does not read back as planned".to_string());
    }
    if !saved.unrecognized.is_empty() {
        return Err("the written stackup holds unrecognized clauses".to_string());
    }
    let same_nominal = match (saved.nominal_thickness_mm, nominal_mm) {
        (Some(a), Some(b)) => mm_to_nm(a) == mm_to_nm(b),
        (None, None) => true,
        _ => false,
    };
    if !same_nominal {
        return Err("the nominal board thickness does not read back as planned".to_string());
    }
    konnect_sexp::parse_sexp(candidate).map_err(|error| error.to_string())?;
    Ok(json!({
        "stackup_matches_plan": true,
        "nominal_thickness_matches_plan": true,
        "bytes_outside_planned_spans_unchanged": true,
        "entry_count": target.entries.len(),
        "stack_thickness_mm": nm_to_mm(stack_thickness_nm(target)),
    }))
}

/// A compact physical summary: counts, thicknesses and every entry.
pub(crate) fn summary(stackup: Option<&Stackup>, nominal_mm: Option<f64>) -> Value {
    let Some(stackup) = stackup else {
        return json!({ "stackup_defined": false, "nominal_board_thickness_mm": nominal_mm });
    };
    let count = |kind: Kind| stackup.entries.iter().filter(|e| e.kind == kind).count();
    let stack_mm = nm_to_mm(stack_thickness_nm(stackup));
    json!({
        "stackup_defined": true,
        "copper_layer_count": count(Kind::Copper),
        "dielectric_count": count(Kind::Dielectric),
        "entry_count": stackup.entries.len(),
        "stack_thickness_mm": stack_mm,
        "nominal_board_thickness_mm": nominal_mm,
        "nominal_minus_stack_mm": nominal_mm.map(|n| ((n - stack_mm) * 1e6).round() / 1e6),
        "finish": stackup.finish,
        "impedance_controlled": stackup.dielectric_constraints,
        "edge_connector": stackup.edge_connector.as_str(),
        "edge_plating": stackup.edge_plating,
        "layers": stackup.entries.iter().map(entry_json).collect::<Vec<_>>(),
    })
}
