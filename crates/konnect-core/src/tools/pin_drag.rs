//! Carrying a sheet's connections along with pins a library update moved.
//!
//! `update_symbols_from_library` with `allow_pin_moves` re-embeds a symbol
//! whose pins moved in its library, and the wires, labels and no-connect
//! flags that sat on the old pin ends stay where they were. This module finds
//! every pin end the update moved, on every placed instance, and either
//! reports what was left behind or drags it, keeping wires orthogonal:
//!
//! - A label or no-connect flag on the old end moves to the new one.
//! - A wire ending there whose far end holds nothing but labels — a label
//!   stub — moves whole, labels and all, when its new place touches nothing.
//! - Otherwise a wire lying along the move has its end slid to the new point,
//!   and any other gets a jog from the old end to the new one, as does an old
//!   end that something else still holds: another pin, a sheet pin, a wire
//!   running through.
//!
//! Junction dots at every point touched are judged afterwards by KiCad's rule.

use konnect_schematic_editor as cse;
use konnect_sexp::geometry::{point_on_segment, points_coincident};
use konnect_sexp::schematic::{extract_sheet_pins, pin_endpoint};
use konnect_sexp::{parse_sexp, SexpNode};
use serde_json::{json, Value};
use std::path::Path;

const TOL: f64 = crate::tools::sch_connectivity::COINCIDENT_TOLERANCE;

/// One pin end an update moved on one placed instance.
#[derive(Debug, Clone)]
pub(crate) struct MovedPinEnd {
    pub(crate) reference: String,
    pub(crate) unit: u32,
    pub(crate) number: String,
    pub(crate) from: (f64, f64),
    /// `None` when the new definition has no such pin.
    pub(crate) to: Option<(f64, f64)>,
}

impl MovedPinEnd {
    fn json(&self) -> Value {
        json!({
            "reference": self.reference,
            "unit": self.unit,
            "pin": self.number,
            "from": { "x": self.from.0, "y": self.from.1 },
            "to": self.to.map(|(x, y)| json!({ "x": x, "y": y })),
        })
    }
}

fn same(a: (f64, f64), b: (f64, f64)) -> bool {
    points_coincident(a.0, a.1, b.0, b.1, TOL)
}

fn round((x, y): (f64, f64)) -> (f64, f64) {
    (
        konnect_sexp::geometry::round6(x),
        konnect_sexp::geometry::round6(y),
    )
}

/// Every pin end of the instances of `lib_ids` that is somewhere else in
/// `after` than in `before`, matched by instance UUID, unit and pin number.
pub(crate) fn moved_pin_ends(
    before: &SexpNode,
    after: &SexpNode,
    lib_ids: &[String],
) -> Vec<MovedPinEnd> {
    let after_pins = crate::tools::placed_pins_by_reference(after);
    let mut moved = Vec::new();
    for (instance, pins) in crate::tools::placed_pins_by_reference(before) {
        if !lib_ids.contains(&instance.lib_id) {
            continue;
        }
        let now = after_pins.iter().find(|(other, _)| {
            other.uuid == instance.uuid
                && other.unit == instance.unit
                && other.reference == instance.reference
        });
        for (pin, transform) in &pins {
            let from = round(pin_endpoint(pin, *transform));
            let to = now.and_then(|(_, now_pins)| {
                now_pins
                    .iter()
                    .find(|(other, _)| other.number == pin.number)
                    .map(|(other, t)| round(pin_endpoint(other, *t)))
            });
            if to.is_some_and(|to| same(to, from)) {
                continue;
            }
            moved.push(MovedPinEnd {
                reference: instance.reference.clone(),
                unit: instance.unit,
                number: pin.number.clone(),
                from,
                to,
            });
        }
    }
    moved
}

/// What sits on one point: wire ends (by UUID), labels (by text) and
/// no-connect flags.
fn items_at(sch: &cse::Schematic, at: (f64, f64)) -> Value {
    let wires: Vec<&str> = sch
        .wires
        .iter()
        .filter(|wire| same(wire.start, at) || same(wire.end, at))
        .map(|wire| wire.uuid.as_str())
        .collect();
    let mut labels: Vec<String> = Vec::new();
    labels.extend(
        sch.labels
            .iter()
            .filter(|label| same((label.at.x, label.at.y), at))
            .map(|label| label.text.clone()),
    );
    labels.extend(
        sch.global_labels
            .iter()
            .filter(|label| same((label.at.x, label.at.y), at))
            .map(|label| label.text.clone()),
    );
    labels.extend(
        sch.hierarchical_labels
            .iter()
            .filter(|label| same((label.at.x, label.at.y), at))
            .map(|label| label.text.clone()),
    );
    let no_connects = sch
        .no_connects
        .iter()
        .filter(|flag| same((flag.x, flag.y), at))
        .count();
    json!({ "wires": wires, "labels": labels, "no_connects": no_connects })
}

fn is_empty(items: &Value) -> bool {
    items["wires"].as_array().is_some_and(Vec::is_empty)
        && items["labels"].as_array().is_some_and(Vec::is_empty)
        && items["no_connects"] == 0
}

/// Every pin end in `content`, as the sheet now places them.
fn pin_ends(tree: &SexpNode) -> Vec<(f64, f64)> {
    crate::tools::all_pin_endpoints(tree)
}

/// What each moved pin left on its old end that no pin holds any more: the
/// list `update_symbols_from_library` reports when it does not drag.
pub(crate) fn left_dangling(
    path: &Path,
    content: &str,
    moved: &[MovedPinEnd],
) -> anyhow::Result<Vec<Value>> {
    let tree = parse_sexp(content)?;
    let pins = pin_ends(&tree);
    let sch = cse::Schematic::from_source(path, content.to_owned())?;
    let mut out = Vec::new();
    for pin in moved {
        if pins.iter().any(|&end| same(end, pin.from)) {
            continue;
        }
        let items = items_at(&sch, pin.from);
        if is_empty(&items) {
            continue;
        }
        let mut entry = pin.json();
        entry["left_at_old_end"] = items;
        out.push(entry);
    }
    Ok(out)
}

/// The result of dragging.
pub(crate) struct Dragged {
    pub(crate) content: String,
    /// Per moved pin: what was done.
    pub(crate) dragged: Vec<Value>,
    /// Per moved pin that could not be dragged (its pin is gone): what stays.
    pub(crate) left_dangling: Vec<Value>,
    pub(crate) junctions_added: usize,
    pub(crate) junctions_pruned: usize,
}

/// Drag what sits on each moved pin's old end to its new one; see the module
/// documentation for the rules.
pub(crate) fn drag_connections(
    path: &Path,
    content: &str,
    moved: &[MovedPinEnd],
) -> anyhow::Result<Dragged> {
    let tree = parse_sexp(content)?;
    let pins = pin_ends(&tree);
    let sheet_pins = extract_sheet_pins(&tree);
    let mut sch = cse::Schematic::from_source(path, content.to_owned())?;
    let mut dragged = Vec::new();
    let mut left = Vec::new();
    let mut jogs: Vec<crate::tools::sch_wiring::WireSpan> = Vec::new();
    let mut touched: Vec<(f64, f64)> = Vec::new();

    for pin in moved {
        let from = pin.from;
        let Some(to) = pin.to else {
            let items = items_at(&sch, from);
            if !is_empty(&items) {
                let mut entry = pin.json();
                entry["left_at_old_end"] = items;
                left.push(entry);
            }
            continue;
        };
        let items = items_at(&sch, from);
        if is_empty(&items) {
            continue;
        }
        touched.extend([from, to]);
        let delta = (to.0 - from.0, to.1 - from.1);
        let mut actions: Vec<Value> = Vec::new();

        // Something besides this pin still holds the old end: keep it all
        // there, and bring the new end to it with a jog.
        let held = pins.iter().any(|&end| same(end, from))
            || sheet_pins.iter().any(|&end| same(end, from))
            || sch.wires.iter().any(|wire| {
                point_on_segment(
                    from.0,
                    from.1,
                    wire.start.0,
                    wire.start.1,
                    wire.end.0,
                    wire.end.1,
                    TOL,
                ) && !same(wire.start, from)
                    && !same(wire.end, from)
            });
        if held {
            jogs.extend(add_jog(&mut sch, from, to));
            actions.push(json!({ "action": "jog", "reason": "the old end is still held" }));
            let mut entry = pin.json();
            entry["actions"] = json!(actions);
            dragged.push(entry);
            continue;
        }

        // Labels and flags on the pin end go with it.
        let mut moved_labels = Vec::new();
        for label in sch.labels.iter_mut() {
            if same((label.at.x, label.at.y), from) {
                label.translate(delta.0, delta.1);
                moved_labels.push(label.text.clone());
            }
        }
        for label in sch.global_labels.iter_mut() {
            if same((label.at.x, label.at.y), from) {
                label.translate(delta.0, delta.1);
                moved_labels.push(label.text.clone());
            }
        }
        for label in sch.hierarchical_labels.iter_mut() {
            if same((label.at.x, label.at.y), from) {
                label.translate(delta.0, delta.1);
                moved_labels.push(label.text.clone());
            }
        }
        if !moved_labels.is_empty() {
            actions.push(json!({ "action": "moved_labels", "labels": moved_labels }));
        }
        for flag in sch.no_connects.iter_mut() {
            if same((flag.x, flag.y), from) {
                flag.x = konnect_sexp::geometry::round6(flag.x + delta.0);
                flag.y = konnect_sexp::geometry::round6(flag.y + delta.1);
                actions.push(json!({ "action": "moved_no_connect" }));
            }
        }

        let attached: Vec<String> = sch
            .wires
            .iter()
            .filter(|wire| same(wire.start, from) || same(wire.end, from))
            .map(|wire| wire.uuid.clone())
            .collect();
        let mut jogged = false;
        for uuid in attached {
            let Some(index) = sch.wires.iter().position(|wire| wire.uuid == uuid) else {
                continue;
            };
            let wire = sch.wires.get(index).expect("found").clone();
            let far = if same(wire.start, from) {
                wire.end
            } else {
                wire.start
            };
            touched.push(far);
            if is_label_stub(&sch, &pins, &sheet_pins, &wire.uuid, far)
                && translation_is_clear(&sch, &pins, &sheet_pins, &wire.uuid, from, far, delta)
            {
                let new_far = round((far.0 + delta.0, far.1 + delta.1));
                let mut labels = Vec::new();
                for label in sch.labels.iter_mut() {
                    if same((label.at.x, label.at.y), far) {
                        label.translate(delta.0, delta.1);
                        labels.push(label.text.clone());
                    }
                }
                for label in sch.global_labels.iter_mut() {
                    if same((label.at.x, label.at.y), far) {
                        label.translate(delta.0, delta.1);
                        labels.push(label.text.clone());
                    }
                }
                for label in sch.hierarchical_labels.iter_mut() {
                    if same((label.at.x, label.at.y), far) {
                        label.translate(delta.0, delta.1);
                        labels.push(label.text.clone());
                    }
                }
                let wire = sch.wires.get_mut(index).expect("found");
                wire.start = round((wire.start.0 + delta.0, wire.start.1 + delta.1));
                wire.end = round((wire.end.0 + delta.0, wire.end.1 + delta.1));
                touched.push(new_far);
                actions.push(json!({
                    "action": "moved_stub",
                    "wire": uuid,
                    "labels": labels,
                }));
                continue;
            }
            let along = (delta.0 == 0.0 && (wire.start.0 - wire.end.0).abs() < TOL)
                || (delta.1 == 0.0 && (wire.start.1 - wire.end.1).abs() < TOL);
            if along {
                if same(to, far) {
                    sch.wires.remove_by_uuid(&uuid);
                    actions.push(json!({ "action": "removed_wire", "wire": uuid,
                        "reason": "the pin end moved onto its far end" }));
                } else {
                    let wire = sch.wires.get_mut(index).expect("found");
                    if same(wire.start, from) {
                        wire.start = to;
                    } else {
                        wire.end = to;
                    }
                    actions.push(json!({ "action": "slid_wire_end", "wire": uuid }));
                }
                continue;
            }
            if !jogged {
                jogs.extend(add_jog(&mut sch, from, to));
                jogged = true;
            }
            actions.push(json!({ "action": "jog", "wire": uuid }));
        }
        let mut entry = pin.json();
        entry["actions"] = json!(actions);
        dragged.push(entry);
    }

    // New jogs get the dots KiCad would draw for them; every point touched
    // is then re-judged by the same rule.
    let jog_dots = crate::tools::sch_wiring::junctions_needed_by_new_wires(&sch.to_source(), &jogs);
    for &point in &jog_dots {
        sch.add_junction(point.0, point.1);
        touched.push(point);
    }
    let (content, added, pruned) =
        crate::tools::sch_wiring::reconcile_junctions_at(sch.to_source(), &touched);
    Ok(Dragged {
        content,
        dragged,
        left_dangling: left,
        junctions_added: jog_dots.len() + added,
        junctions_pruned: pruned,
    })
}

/// A jog from `from` to `to`: one segment when they share an axis, an L
/// (along X, then Y) when they do not. Returns the segments added.
fn add_jog(
    sch: &mut cse::Schematic,
    from: (f64, f64),
    to: (f64, f64),
) -> Vec<crate::tools::sch_wiring::WireSpan> {
    let mut segments = Vec::new();
    if (from.0 - to.0).abs() < TOL || (from.1 - to.1).abs() < TOL {
        segments.push((from, to));
    } else {
        let corner = (to.0, from.1);
        segments.push((from, corner));
        segments.push((corner, to));
    }
    for &(a, b) in &segments {
        sch.add_wire(a.0, a.1, b.0, b.1);
    }
    segments
}

/// A wire whose far end holds nothing but labels: no other wire meets or
/// runs through it, and no pin, sheet pin, junction or flag sits on it.
fn is_label_stub(
    sch: &cse::Schematic,
    pins: &[(f64, f64)],
    sheet_pins: &[(f64, f64)],
    uuid: &str,
    far: (f64, f64),
) -> bool {
    let other_wire = sch.wires.iter().any(|wire| {
        wire.uuid != uuid
            && point_on_segment(
                far.0,
                far.1,
                wire.start.0,
                wire.start.1,
                wire.end.0,
                wire.end.1,
                TOL,
            )
    });
    !other_wire
        && !pins.iter().any(|&end| same(end, far))
        && !sheet_pins.iter().any(|&end| same(end, far))
        && !sch.junctions.iter().any(|dot| same((dot.x, dot.y), far))
        && !sch
            .no_connects
            .iter()
            .any(|flag| same((flag.x, flag.y), far))
}

/// Whether the stub `uuid`, moved by `delta`, lands clear of everything else:
/// its new far end on no wire, pin, sheet pin or other label, and no pin or
/// other wire's end on its new span except the moved pin's own end.
fn translation_is_clear(
    sch: &cse::Schematic,
    pins: &[(f64, f64)],
    sheet_pins: &[(f64, f64)],
    uuid: &str,
    from: (f64, f64),
    far: (f64, f64),
    delta: (f64, f64),
) -> bool {
    let to = round((from.0 + delta.0, from.1 + delta.1));
    let new_far = round((far.0 + delta.0, far.1 + delta.1));
    let on_span = |point: (f64, f64)| {
        point_on_segment(point.0, point.1, to.0, to.1, new_far.0, new_far.1, TOL)
            && !same(point, to)
    };
    let wires_clear = sch
        .wires
        .iter()
        .filter(|wire| wire.uuid != uuid)
        .all(|wire| {
            !point_on_segment(
                new_far.0,
                new_far.1,
                wire.start.0,
                wire.start.1,
                wire.end.0,
                wire.end.1,
                TOL,
            ) && !on_span(wire.start)
                && !on_span(wire.end)
        });
    let labels_clear = sch
        .labels
        .iter()
        .map(|label| (label.at.x, label.at.y))
        .chain(
            sch.global_labels
                .iter()
                .map(|label| (label.at.x, label.at.y)),
        )
        .chain(
            sch.hierarchical_labels
                .iter()
                .map(|label| (label.at.x, label.at.y)),
        )
        .filter(|&at| !same(at, far) && !same(at, from))
        .all(|at| !on_span(at) && !same(at, new_far));
    wires_clear
        && labels_clear
        && !pins.iter().any(|&end| on_span(end))
        && !sheet_pins
            .iter()
            .any(|&end| on_span(end) || same(end, new_far))
}
