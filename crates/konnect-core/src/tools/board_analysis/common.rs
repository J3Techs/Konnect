//! What every board analysis shares: the saved board with its stackup,
//! argument readers, the `output` / `markers_output` files and the method
//! citations each report carries.

use crate::mcp::{error::ToolErrorKind, protocol::CallToolResult};
use crate::native_route_job;
use crate::tools::board_inspect::{self, SavedBoard};
use crate::tools::get_path;
use crate::tools::route_native::{invalid, opt_path, Refusal};
use konnect_route::api::{LayerId, NetId};
use konnect_route::board::RouteBoard;
use konnect_sexp::SexpNode;
use serde_json::{json, Value};
use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

/// Annealed copper resistivity at 20 °C, Ω·mm (IACS: 1.7241e-8 Ω·m).
pub(crate) const RHO_CU_20C_OHM_MM: f64 = 1.7241e-5;
/// Its temperature coefficient, 1/°C (IACS, 20 °C reference).
pub(crate) const ALPHA_CU: f64 = 0.00393;
/// Copper assumed for an outer / inner layer the stackup does not give, mm.
const DEFAULT_OUTER_MM: f64 = 0.035;
const DEFAULT_INNER_MM: f64 = 0.0175;
/// Board thickness assumed without a stackup, mm.
const DEFAULT_BOARD_MM: f64 = 1.6;

/// The copper of each layer (by router layer id) and where its centre sits
/// in the stack, measured down from the top of F.Cu.
#[derive(Debug, Clone)]
pub(crate) struct Stackup {
    pub thickness_mm: Vec<f64>,
    pub z_mm: Vec<f64>,
    /// Per layer: the thickness came from `(setup (stackup …))`.
    pub from_file: Vec<bool>,
}

impl Stackup {
    /// Read the copper plies and dielectrics of `(setup (stackup …))` in stack
    /// order; layers it does not give fall back to 35 µm outer / 17.5 µm
    /// inner, spread evenly through a 1.6 mm board.
    pub(crate) fn of(tree: &SexpNode, board: &RouteBoard) -> Stackup {
        let n = board.layers.len();
        let mut thickness = vec![None; n];
        let mut z = vec![None; n];
        if let Some(stack) = tree.find("setup").and_then(|s| s.find("stackup")) {
            let mut depth = 0.0;
            for layer in stack.find_all("layer") {
                let name = layer.get(1).and_then(SexpNode::as_str).unwrap_or("");
                let kind = layer.find_str("type").unwrap_or("");
                let plies: f64 = layer
                    .find_all("thickness")
                    .iter()
                    .filter_map(|t| t.get_f64(1))
                    .filter(|t| t.is_finite() && *t > 0.0)
                    .sum();
                if let Some(id) = board.layers.by_name(name) {
                    let t = layer
                        .find_f64("thickness")
                        .filter(|t| t.is_finite() && *t > 0.0);
                    if let Some(t) = t {
                        thickness[id.index()] = Some(t);
                        z[id.index()] = Some(depth + t / 2.0);
                        depth += t;
                    }
                } else if kind == "core" || kind == "prepreg" || name.starts_with("dielectric") {
                    depth += plies;
                }
            }
        }
        let from_file: Vec<bool> = thickness.iter().map(Option::is_some).collect();
        let last = n.saturating_sub(1);
        let thickness_mm: Vec<f64> = (0..n)
            .map(|i| {
                thickness[i].unwrap_or(if i == 0 || i == last {
                    DEFAULT_OUTER_MM
                } else {
                    DEFAULT_INNER_MM
                })
            })
            .collect();
        let z_mm = if z.iter().all(Option::is_some) {
            z.into_iter().map(Option::unwrap).collect()
        } else {
            (0..n)
                .map(|i| DEFAULT_BOARD_MM * i as f64 / last.max(1) as f64)
                .collect()
        };
        Stackup {
            thickness_mm,
            z_mm,
            from_file,
        }
    }

    pub(crate) fn t(&self, layer: LayerId) -> f64 {
        self.thickness_mm[layer.index()]
    }

    /// Whether `layer` is an outer (external) copper layer.
    pub(crate) fn is_outer(&self, layer: LayerId) -> bool {
        layer.index() == 0 || layer.index() + 1 == self.thickness_mm.len()
    }

    pub(crate) fn json(&self, board: &RouteBoard) -> Value {
        json!((0..self.thickness_mm.len())
            .map(|i| json!({
                "layer": board.layers.copper[i].name,
                "copper_um": r4(self.thickness_mm[i] * 1000.0),
                "depth_mm": r4(self.z_mm[i]),
                "source": if self.from_file[i] { "board stackup" } else { "assumed (stackup does not give it)" },
            }))
            .collect::<Vec<_>>())
    }
}

/// The saved board, its parsed tree and its stackup.
pub(crate) struct Loaded {
    pub saved: SavedBoard,
    pub tree: SexpNode,
    pub stack: Stackup,
}

impl Loaded {
    pub(crate) fn board(&self) -> &RouteBoard {
        &self.saved.board
    }
}

pub(crate) fn load(args: &Value) -> anyhow::Result<Result<Loaded, Refusal>> {
    let path = get_path(args, "board")?;
    Ok(load_at(&path))
}

pub(crate) fn load_at(path: &Path) -> Result<Loaded, Refusal> {
    let (saved, tree) = board_inspect::load_board_and_tree(path)?;
    let stack = Stackup::of(&tree, &saved.board);
    Ok(Loaded { saved, tree, stack })
}

/// Round to 4 decimals for reports.
pub(crate) fn r4(v: f64) -> f64 {
    if v.is_finite() {
        (v * 1e4).round() / 1e4
    } else {
        v
    }
}

/// Round to 6 significant-ish decimals (resistances, voltages).
pub(crate) fn r6(v: f64) -> f64 {
    if v.is_finite() {
        (v * 1e6).round() / 1e6
    } else {
        v
    }
}

pub(crate) fn opt_f64(
    args: &Value,
    key: &str,
    default: f64,
    min: f64,
    max: f64,
) -> Result<f64, Refusal> {
    match args.get(key) {
        None | Some(Value::Null) => Ok(default),
        Some(v) => match v.as_f64() {
            Some(x) if x.is_finite() && x >= min && x <= max => Ok(x),
            _ => Err(invalid(
                key,
                &format!("must be a number from {min} to {max}"),
            )),
        },
    }
}

/// Net names or `*`/`?` patterns, each of which must match a net.
pub(crate) fn nets_named(
    board: &RouteBoard,
    patterns: &[String],
    field: &str,
) -> Result<BTreeSet<NetId>, Refusal> {
    let mut all = BTreeSet::new();
    let mut unknown = Vec::new();
    for p in patterns {
        let found =
            konnect_route::pipeline::coverage::select_nets(board, None, std::slice::from_ref(p));
        if found.is_empty() {
            unknown.push(p.clone());
        }
        all.extend(found);
    }
    if !unknown.is_empty() {
        return Err(invalid(
            field,
            &format!("no net of the board matches: {}", unknown.join(", ")),
        ));
    }
    Ok(all)
}

/// One exact net name.
pub(crate) fn net_named(board: &RouteBoard, name: &str, field: &str) -> Result<NetId, Refusal> {
    board
        .net_by_name(name)
        .ok_or_else(|| invalid(field, &format!("no net named '{name}' on the board")))
}

/// Whether `name` matches a `*`/`?` pattern.
pub(crate) fn glob(pattern: &str, name: &str) -> bool {
    fn rec(p: &[char], s: &[char]) -> bool {
        match (p.first(), s.first()) {
            (None, None) => true,
            (Some('*'), _) => rec(&p[1..], s) || (!s.is_empty() && rec(p, &s[1..])),
            (Some('?'), Some(_)) => rec(&p[1..], &s[1..]),
            (Some(a), Some(b)) if a == b => rec(&p[1..], &s[1..]),
            _ => false,
        }
    }
    let p: Vec<char> = pattern.chars().collect();
    let s: Vec<char> = name.chars().collect();
    rec(&p, &s)
}

/// Read a JSON file named by `key`, if given.
pub(crate) fn json_file(args: &Value, key: &str) -> Result<Option<Value>, Refusal> {
    let Some(path) = opt_path(args, key)? else {
        return Ok(None);
    };
    let text = std::fs::read_to_string(&path)
        .map_err(|e| invalid(key, &format!("{} could not be read: {e}", path.display())))?;
    serde_json::from_str(&text)
        .map(Some)
        .map_err(|e| invalid(key, &format!("{} is not JSON: {e}", path.display())))
}

/// The report files a call asked for: the full result and render markers.
pub(crate) struct Outputs {
    pub output: Option<PathBuf>,
    pub markers: Option<PathBuf>,
}

fn absolute(p: PathBuf) -> PathBuf {
    if p.is_absolute() {
        p
    } else {
        std::env::current_dir().map(|d| d.join(&p)).unwrap_or(p)
    }
}

fn refuse_existing(path: &Path) -> Result<(), Refusal> {
    if path.exists() {
        return Err(CallToolResult::error_kind(
            ToolErrorKind::Conflict {
                paths: vec![path.display().to_string()],
            },
            format!(
                "{} already exists; it is never replaced. Nothing was written.",
                path.display()
            ),
        ));
    }
    Ok(())
}

fn write_new(path: &Path, bytes: &[u8]) -> Result<(), CallToolResult> {
    if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
        if let Err(e) = std::fs::create_dir_all(parent) {
            return Err(CallToolResult::error(format!(
                "{} not written: {e}",
                path.display()
            )));
        }
    }
    native_route_job::write_new_atomic(path, bytes)
        .map_err(|e| CallToolResult::error(format!("{} not written: {e:#}", path.display())))
}

impl Outputs {
    /// `output` and `markers_output`, refused up front when either exists.
    pub(crate) fn of(args: &Value) -> Result<Outputs, Refusal> {
        let output = opt_path(args, "output")?.map(absolute);
        let markers = opt_path(args, "markers_output")?.map(absolute);
        for p in output.iter().chain(markers.iter()) {
            refuse_existing(p)?;
        }
        if output.is_some() && output == markers {
            return Err(invalid("markers_output", "must differ from output"));
        }
        Ok(Outputs { output, markers })
    }

    /// Write the full result and the markers; returns what was written, for
    /// the inline summary.
    pub(crate) fn write(&self, full: &Value, markers: &[Value]) -> Result<Value, CallToolResult> {
        if let Some(p) = &self.output {
            let text = serde_json::to_string_pretty(full)
                .map_err(|e| CallToolResult::error(format!("output not serialized: {e}")))?;
            write_new(p, text.as_bytes())?;
        }
        if let Some(p) = &self.markers {
            let text = serde_json::to_string_pretty(&json!(markers))
                .map_err(|e| CallToolResult::error(format!("markers not serialized: {e}")))?;
            write_new(p, text.as_bytes())?;
        }
        Ok(json!({
            "output": self.output.as_ref().map(|p| p.display().to_string()),
            "markers_output": self.markers.as_ref().map(|p| p.display().to_string()),
            "markers_written": self.markers.as_ref().map(|_| markers.len()),
        }))
    }
}

/// A render marker as `render_board_layers_png` reads them.
pub(crate) fn marker(x: f64, y: f64, kind: &str, label: String) -> Value {
    json!({ "x": r4(x), "y": r4(y), "kind": kind, "label": label })
}

/// The schema properties every analysis shares.
pub(crate) fn common_props() -> Value {
    json!({
        "board": { "type": "string", "description": "Saved .kicad_pcb to read (the file on disk, not the live editor)" },
        "output": { "type": "string", "description": "New JSON file for the full result (the reply carries a summary); an existing file is never replaced" },
        "markers_output": { "type": "string", "description": "New JSON file of render markers ({x, y, label, kind}) for render_board_layers_png / render_board_tiles markers_file; an existing file is never replaced" },
        "max_items": { "type": "integer", "minimum": 0, "maximum": 100000, "default": 50, "description": "Findings listed inline; output holds them all" }
    })
}

/// `common_props()` plus `extra`, as a closed object schema.
pub(crate) fn schema(extra: Value, required: &[&str]) -> Value {
    let mut props = common_props();
    if let (Some(p), Some(e)) = (props.as_object_mut(), extra.as_object()) {
        p.extend(e.clone());
    }
    json!({ "type": "object", "properties": props, "required": required, "additionalProperties": false })
}

/// The `source` block every report starts with.
pub(crate) fn provenance(l: &Loaded) -> Value {
    json!({
        "source": "saved_file",
        "board": l.saved.path,
        "note": board_inspect::SAVED_FILE_NOTE,
        "load_report": l.saved.load_report_json(),
    })
}

/// Merge `extra` into `base` (objects only).
pub(crate) fn merged(mut base: Value, extra: Value) -> Value {
    if let (Some(b), Value::Object(e)) = (base.as_object_mut(), extra) {
        b.extend(e);
    }
    base
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn globs_match_like_the_net_filters() {
        assert!(glob("GND*", "GND_A"));
        assert!(glob("*GND", "AGND"));
        assert!(glob("V?", "V5"));
        assert!(!glob("V?", "V55"));
        assert!(glob("USB_D*", "USB_D+"));
    }

    #[test]
    fn the_stackup_gives_copper_and_depth_in_stack_order() {
        let tree = konnect_sexp::parse_sexp(
            r#"(kicad_pcb (layers (0 "F.Cu" signal) (4 "In1.Cu" signal) (2 "B.Cu" signal))
                (setup (stackup
                  (layer "F.Mask" (type "Top Solder Mask") (thickness 0.01))
                  (layer "F.Cu" (type "copper") (thickness 0.035))
                  (layer "dielectric 1" (type "prepreg") (thickness 0.1))
                  (layer "In1.Cu" (type "copper") (thickness 0.0152))
                  (layer "dielectric 2" (type "core") (thickness 0.5) addsublayer (thickness 0.3))
                  (layer "B.Cu" (type "copper") (thickness 0.035)))))"#,
        )
        .unwrap();
        let (board, _) =
            konnect_route::load_board_tree(&tree, &konnect_route::LoadOptions::default()).unwrap();
        let s = Stackup::of(&tree, &board);
        assert_eq!(s.thickness_mm, vec![0.035, 0.0152, 0.035]);
        assert!((s.z_mm[0] - 0.0175).abs() < 1e-12);
        assert!((s.z_mm[1] - (0.035 + 0.1 + 0.0076)).abs() < 1e-12);
        assert!((s.z_mm[2] - (0.035 + 0.1 + 0.0152 + 0.8 + 0.0175)).abs() < 1e-12);
        assert!(s.from_file.iter().all(|f| *f));
    }
}
