//! Saved-board inspection for the `native_routing` toolset: `query_copper`
//! and `query_pads` list copper with exact geometry, and
//! `render_board_layers_png` draws it.
//!
//! Every answer comes from the saved `.kicad_pcb` (`"source": "saved_file"`):
//! the file is parsed once, lowered with the native router's loader (exact
//! pad shapes, custom primitives in board space, true via spans) and its
//! zones read with their pour settings. Edits KiCad holds unsaved are not
//! seen; save first when the live editor matters. Nothing is written.

use crate::mcp::{error::ToolErrorKind, protocol::CallToolResult};
use crate::tools::get_path;
use crate::tools::route_native::{invalid, opt_bool, opt_count, opt_strings, Refusal};
use konnect_geom::{
    nm_to_mm, touches, ArcPolicy, ArcSide, BBox, Circle, ConvexPoly, Coord, Point, Polygon,
    Segment, Shape,
};
use konnect_route::api::{LayerMask, NetId, Ownership};
use konnect_route::board::{Pad, PadKind, RouteBoard, Side};
use konnect_route::{LoadOptions, LoadReport};
use konnect_sexp::board::{ZoneOutlineElement, ZoneRecord};
use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};

/// Default and largest `max_items`.
pub(crate) const ITEMS_DEFAULT: u64 = 500;
pub(crate) const ITEMS_MAX: u64 = 100_000;

pub(crate) const SAVED_FILE_NOTE: &str =
    "Read from the saved board file: changes KiCad holds unsaved are not included.";

// ─── The saved board ─────────────────────────────────────────────────────────

/// One saved board, parsed once: the router's model plus every zone with its
/// settings.
pub(crate) struct SavedBoard {
    pub path: PathBuf,
    pub board: RouteBoard,
    pub report: LoadReport,
    pub zones: Vec<ZoneRecord>,
    pub zones_skipped: usize,
}

impl SavedBoard {
    /// What the loader dropped or approximated, or `null` when nothing was.
    pub(crate) fn load_report_json(&self) -> Value {
        if self.report.skipped.is_empty() && self.report.notes.is_empty() && self.zones_skipped == 0
        {
            return Value::Null;
        }
        json!({
            "skipped": self.report.skipped,
            "notes": self.report.notes,
            "zones_unreadable": self.zones_skipped,
        })
    }
}

/// Read and load the board named by `args.board`.
pub(crate) fn load_saved_board(args: &Value) -> anyhow::Result<Result<SavedBoard, Refusal>> {
    let path = get_path(args, "board")?;
    Ok(load_board_at(&path))
}

pub(crate) fn load_board_at(path: &Path) -> Result<SavedBoard, Refusal> {
    let text = match std::fs::read_to_string(path) {
        Ok(t) => t,
        Err(e) => {
            return Err(CallToolResult::error_kind(
                ToolErrorKind::FileNotFound {
                    path: path.display().to_string(),
                },
                format!("board could not be read: {}: {e}", path.display()),
            ))
        }
    };
    let tree = konnect_sexp::parse_sexp(&text).map_err(|e| {
        CallToolResult::error(format!(
            "{} is not a readable board: {e}. Nothing was read.",
            path.display()
        ))
    })?;
    let (board, report) =
        konnect_route::load_board_tree(&tree, &LoadOptions::default()).map_err(|e| {
            CallToolResult::error(format!(
                "{} could not be loaded: {e}. Nothing was read.",
                path.display()
            ))
        })?;
    let zones = konnect_sexp::board::zone_records(&tree);
    Ok(SavedBoard {
        path: path.to_path_buf(),
        board,
        report,
        zones: zones.items,
        zones_skipped: zones.skipped,
    })
}

// ─── Shared filters ──────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum AreaMode {
    Intersects,
    Inside,
}

/// A box in board millimetres and how items must relate to it.
#[derive(Debug, Clone)]
pub(crate) struct Area {
    pub bbox: BBox,
    pub mode: AreaMode,
    shape: Shape,
}

impl Area {
    pub(crate) fn new(bbox: BBox, mode: AreaMode) -> Self {
        let shape = if bbox.width() == 0 && bbox.height() == 0 {
            Shape::Circle(Circle::new(bbox.min, 0))
        } else if bbox.width() == 0 || bbox.height() == 0 {
            Shape::Segment(Segment::new(bbox.min, bbox.max, 0))
        } else {
            Shape::ConvexPoly(ConvexPoly::axis_rect(bbox, 0).expect("a box with area"))
        };
        Area { bbox, mode, shape }
    }

    /// Whether an item made of `shapes` passes: any of them touches the box
    /// (`intersects`), or all of them lie within it (`inside`).
    pub(crate) fn accepts<'a>(&self, shapes: impl IntoIterator<Item = &'a Shape>) -> bool {
        let mut any = false;
        for s in shapes {
            any = true;
            let bb = s.bbox();
            match self.mode {
                AreaMode::Inside => {
                    if !self.bbox.contains_bbox(&bb) {
                        return false;
                    }
                }
                AreaMode::Intersects => {
                    if bb.intersects(&self.bbox) && touches(s, &self.shape) {
                        return true;
                    }
                }
            }
        }
        any && self.mode == AreaMode::Inside
    }
}

/// Parse the four corners of a box object (any corner order), in mm.
pub(crate) fn box_arg(value: &Value, field: &str) -> Result<BBox, Refusal> {
    let get = |k: &str| {
        value
            .get(k)
            .and_then(Value::as_f64)
            .filter(|v| v.is_finite())
    };
    let (Some(x1), Some(y1), Some(x2), Some(y2)) = (get("x1"), get("y1"), get("x2"), get("y2"))
    else {
        return Err(invalid(field, "needs finite x1, y1, x2 and y2 (mm)"));
    };
    match (Point::from_mm(x1, y1), Point::from_mm(x2, y2)) {
        (Some(a), Some(b)) => Ok(BBox::new(a, b)),
        _ => Err(invalid(field, "lies outside the representable board range")),
    }
}

/// `area` and `area_mode`.
pub(crate) fn area_arg(args: &Value) -> Result<Option<Area>, Refusal> {
    let mode = match args.get("area_mode") {
        None | Some(Value::Null) => AreaMode::Intersects,
        Some(Value::String(m)) if m == "intersects" => AreaMode::Intersects,
        Some(Value::String(m)) if m == "inside" => AreaMode::Inside,
        Some(_) => return Err(invalid("area_mode", "must be intersects or inside")),
    };
    match args.get("area") {
        None | Some(Value::Null) => Ok(None),
        Some(v) => Ok(Some(Area::new(box_arg(v, "area")?, mode))),
    }
}

/// `nets`: names or `*`/`?` patterns. Every entry must match a net of the
/// board, so a typo is refused instead of answering with nothing.
pub(crate) fn nets_arg(
    board: &RouteBoard,
    args: &Value,
) -> Result<Option<BTreeSet<NetId>>, Refusal> {
    let Some(patterns) = opt_strings(args, "nets")? else {
        return Ok(None);
    };
    let mut all = BTreeSet::new();
    let mut unknown = Vec::new();
    for p in &patterns {
        let found =
            konnect_route::pipeline::coverage::select_nets(board, None, std::slice::from_ref(p));
        if found.is_empty() {
            unknown.push(p.clone());
        }
        all.extend(found);
    }
    if !unknown.is_empty() {
        return Err(invalid(
            "nets",
            &format!("no net of the board matches: {}", unknown.join(", ")),
        ));
    }
    Ok(Some(all))
}

/// Copper layer names (or `*.Cu`, `F&B.Cu`) under `key`.
pub(crate) fn layers_arg(
    board: &RouteBoard,
    args: &Value,
    key: &str,
) -> Result<Option<LayerMask>, Refusal> {
    let Some(names) = opt_strings(args, key)? else {
        return Ok(None);
    };
    let mut mask = LayerMask::EMPTY;
    for n in &names {
        let m = board.layers.expand(n.trim());
        if m.is_empty() {
            return Err(invalid(
                key,
                &format!(
                    "'{n}' is not a copper layer of the board ({})",
                    layer_names(board, board.layers.all()).join(", ")
                ),
            ));
        }
        mask = mask.union(m);
    }
    Ok(Some(mask))
}

pub(crate) fn layer_names(board: &RouteBoard, mask: LayerMask) -> Vec<String> {
    mask.iter()
        .map(|l| board.layers.name(l).to_string())
        .collect()
}

// ─── JSON helpers ────────────────────────────────────────────────────────────

pub(crate) fn mm(nm: Coord) -> f64 {
    nm_to_mm(i64::from(nm))
}

pub(crate) fn pt(p: Point) -> Value {
    let (x, y) = p.to_mm();
    json!([x, y])
}

pub(crate) fn bbox_json(b: &BBox) -> Value {
    let (x1, y1) = b.min.to_mm();
    let (x2, y2) = b.max.to_mm();
    json!({ "x1": x1, "y1": y1, "x2": x2, "y2": y2 })
}

/// An exact copper shape: a disc, a stadium (a centre line swept by half its
/// width), a convex polygon grown by `corner_radius_mm`, or a polygon.
pub(crate) fn shape_json(s: &Shape) -> Value {
    match s {
        Shape::Circle(c) => json!({
            "type": "circle", "center": pt(c.center), "radius_mm": mm(c.radius)
        }),
        Shape::Segment(g) => json!({
            "type": "stadium", "a": pt(g.a), "b": pt(g.b), "width_mm": mm(g.width)
        }),
        Shape::ConvexPoly(p) => json!({
            "type": "polygon",
            "points": p.points().iter().map(|q| pt(*q)).collect::<Vec<_>>(),
            "corner_radius_mm": mm(p.radius()),
        }),
        Shape::Polygon(p) => json!({
            "type": "polygon",
            "points": p.outline().iter().map(|q| pt(*q)).collect::<Vec<_>>(),
            "holes": p.holes().iter().map(|h| h.iter().map(|q| pt(*q)).collect::<Vec<_>>()).collect::<Vec<_>>(),
        }),
    }
}

fn net_name(board: &RouteBoard, net: Option<NetId>) -> Value {
    match net {
        Some(n) => json!(board.net(n).name),
        None => Value::Null,
    }
}

fn outline_json(elements: &[ZoneOutlineElement]) -> Vec<Value> {
    elements
        .iter()
        .map(|e| match *e {
            ZoneOutlineElement::Point((x, y)) => json!([x, y]),
            ZoneOutlineElement::Arc { start, mid, end } => json!({
                "arc": { "start": [start.0, start.1], "mid": [mid.0, mid.1], "end": [end.0, end.1] }
            }),
        })
        .collect()
}

/// A zone's authored region (outline minus holes) as an exact polygon, arcs
/// lowered on `side`.
pub(crate) fn zone_region(z: &ZoneRecord, side: ArcSide) -> Option<Polygon> {
    let policy = ArcPolicy::default();
    let outline = konnect_route::loader::elements_to_ring(&z.geometry.outline, &policy, side)?;
    let holes = z
        .geometry
        .holes
        .iter()
        .map(|h| konnect_route::loader::elements_to_ring(h, &policy, side))
        .collect::<Option<Vec<_>>>()?;
    Polygon::new(outline, holes)
}

pub(crate) fn zone_layers(b: &RouteBoard, z: &ZoneRecord) -> LayerMask {
    b.layers
        .expand_all(z.geometry.layers.iter().map(String::as_str))
}

// ─── query_copper ────────────────────────────────────────────────────────────

const COPPER_KINDS: [&str; 5] = ["track", "arc", "via", "zone", "rule_area"];

pub(crate) async fn handle_query_copper(
    args: &Value,
    _ctx: &crate::tools::ToolContext,
) -> anyhow::Result<CallToolResult> {
    // Parsing a large board takes a second or more: off the async workers.
    let args = args.clone();
    tokio::task::spawn_blocking(move || query_copper_blocking(&args)).await?
}

struct CopperFilters {
    area: Option<Area>,
    nets: Option<BTreeSet<NetId>>,
    layers: Option<LayerMask>,
    kinds: BTreeSet<&'static str>,
    locked: Option<bool>,
}

fn kinds_arg(args: &Value) -> Result<BTreeSet<&'static str>, Refusal> {
    let Some(names) = opt_strings(args, "kinds")? else {
        return Ok(COPPER_KINDS.into_iter().collect());
    };
    let mut kinds = BTreeSet::new();
    for n in &names {
        match COPPER_KINDS.iter().find(|k| **k == n.as_str()) {
            Some(k) => {
                kinds.insert(*k);
            }
            None => {
                return Err(invalid(
                    "kinds",
                    &format!("'{n}' is not one of {}", COPPER_KINDS.join(", ")),
                ))
            }
        }
    }
    Ok(kinds)
}

fn locked_arg(args: &Value) -> Result<Option<bool>, Refusal> {
    match args.get("locked") {
        None | Some(Value::Null) => Ok(None),
        Some(Value::Bool(b)) => Ok(Some(*b)),
        Some(_) => Err(invalid("locked", "must be true or false")),
    }
}

fn query_copper_blocking(args: &Value) -> anyhow::Result<CallToolResult> {
    let saved = match load_saved_board(args)? {
        Ok(s) => s,
        Err(r) => return Ok(r),
    };
    let parsed = (|| -> Result<_, Refusal> {
        let filters = CopperFilters {
            area: area_arg(args)?,
            nets: nets_arg(&saved.board, args)?,
            layers: layers_arg(&saved.board, args, "layers")?,
            kinds: kinds_arg(args)?,
            locked: locked_arg(args)?,
        };
        let counts_only = opt_bool(args, "counts_only", false)?;
        let max_items = opt_count(args, "max_items", ITEMS_DEFAULT, ITEMS_MAX)? as usize;
        let include_fills = opt_bool(args, "include_fills", false)?;
        Ok((filters, counts_only, max_items, include_fills))
    })();
    let (filters, counts_only, max_items, include_fills) = match parsed {
        Ok(v) => v,
        Err(r) => return Ok(r),
    };
    let t = std::time::Instant::now();
    let found = select_copper(&saved, &filters);

    let mut by_kind: BTreeMap<&str, usize> = COPPER_KINDS
        .iter()
        .filter(|k| filters.kinds.contains(*k))
        .map(|k| (*k, 0))
        .collect();
    let mut by_layer: BTreeMap<String, usize> = BTreeMap::new();
    let mut by_net: BTreeMap<String, usize> = BTreeMap::new();
    for item in &found {
        *by_kind.entry(item.kind()).or_default() += 1;
        for l in item.layers(&saved).iter() {
            *by_layer
                .entry(saved.board.layers.name(l).to_string())
                .or_default() += 1;
        }
        let net = item.net_name(&saved).unwrap_or("(no net)").to_string();
        *by_net.entry(net).or_default() += 1;
    }
    let items: Vec<Value> = if counts_only {
        Vec::new()
    } else {
        found
            .iter()
            .take(max_items)
            .map(|i| i.to_json(&saved, include_fills))
            .collect()
    };
    let returned = items.len();
    let mut body = json!({
        "source": "saved_file",
        "board": saved.path,
        "note": SAVED_FILE_NOTE,
        "matched": found.len(),
        "by_kind": by_kind,
        "by_layer": by_layer,
        "elapsed_ms": t.elapsed().as_millis() as u64,
        "load_report": saved.load_report_json(),
    });
    if counts_only {
        body["by_net"] = json!(by_net);
    } else {
        body["returned"] = json!(returned);
        body["truncated"] = json!(returned < found.len());
        body["items"] = json!(items);
    }
    Ok(CallToolResult::json(&body))
}

/// One matched copper item, by index into its source list.
#[derive(Debug, Clone, Copy)]
enum Found {
    Track(usize),
    Arc(usize),
    Via(usize),
    Zone(usize),
    RuleArea(usize),
}

impl Found {
    fn kind(self) -> &'static str {
        match self {
            Found::Track(_) => "track",
            Found::Arc(_) => "arc",
            Found::Via(_) => "via",
            Found::Zone(_) => "zone",
            Found::RuleArea(_) => "rule_area",
        }
    }

    fn layers(self, s: &SavedBoard) -> LayerMask {
        let b = &s.board;
        match self {
            Found::Track(i) => LayerMask::single(b.tracks[i].layer),
            Found::Arc(i) => LayerMask::single(b.arcs[i].layer),
            Found::Via(i) => b.vias[i].layers,
            Found::Zone(i) | Found::RuleArea(i) => zone_layers(b, &s.zones[i]),
        }
    }

    fn net_name(self, s: &SavedBoard) -> Option<&str> {
        let b = &s.board;
        let by_id = |n: Option<NetId>| n.map(|n| b.net(n).name.as_str());
        match self {
            Found::Track(i) => by_id(b.tracks[i].net),
            Found::Arc(i) => by_id(b.arcs[i].net),
            Found::Via(i) => by_id(b.vias[i].net),
            Found::Zone(i) | Found::RuleArea(i) => s.zones[i].geometry.net.as_deref(),
        }
    }

    fn to_json(self, s: &SavedBoard, include_fills: bool) -> Value {
        let b = &s.board;
        let locked = |o: Ownership| o == Ownership::Locked;
        match self {
            Found::Track(i) => {
                let t = &b.tracks[i];
                let (x1, y1) = t.seg.a.to_mm();
                let (x2, y2) = t.seg.b.to_mm();
                json!({
                    "kind": "track", "uuid": t.uuid, "net": net_name(b, t.net),
                    "layer": b.layers.name(t.layer),
                    "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                    "width_mm": mm(t.seg.width),
                    "length_mm": nm_to_mm(t.seg.length()),
                    "locked": locked(t.ownership),
                })
            }
            Found::Arc(i) => {
                let a = &b.arcs[i];
                json!({
                    "kind": "arc", "uuid": a.uuid, "net": net_name(b, a.net),
                    "layer": b.layers.name(a.layer),
                    "start": pt(a.start), "mid": pt(a.mid), "end": pt(a.end),
                    "width_mm": mm(a.width),
                    "locked": locked(a.ownership),
                })
            }
            Found::Via(i) => {
                let v = &b.vias[i];
                let (x, y) = v.center.to_mm();
                json!({
                    "kind": "via", "uuid": v.uuid, "net": net_name(b, v.net),
                    "x": x, "y": y,
                    "diameter_mm": mm(v.diameter), "drill_mm": mm(v.drill),
                    "layers": layer_names(b, v.layers),
                    "through": v.layers == b.layers.all(),
                    "locked": locked(v.ownership),
                })
            }
            Found::Zone(i) | Found::RuleArea(i) => zone_json(&s.zones[i], include_fills),
        }
    }
}

fn zone_json(z: &ZoneRecord, include_fills: bool) -> Value {
    let g = &z.geometry;
    let mut v = json!({
        "kind": if g.keepout.is_some() { "rule_area" } else { "zone" },
        "uuid": z.uuid,
        "name": g.name,
        "net": g.net,
        "layers": g.layers,
        "priority": g.priority,
        "locked": z.locked,
        "outline": outline_json(&g.outline),
        "holes": g.holes.iter().map(|h| outline_json(h)).collect::<Vec<_>>(),
    });
    if let Some(k) = g.keepout {
        v["forbids"] = json!({
            "tracks": k.tracks, "vias": k.vias, "pads": k.pads,
            "copperpour": k.copperpour, "footprints": k.footprints,
        });
        return v;
    }
    let mut by_layer: BTreeMap<&str, usize> = BTreeMap::new();
    for f in &g.fills {
        *by_layer.entry(f.layer.as_str()).or_default() += 1;
    }
    v["pad_connection"] = json!(z.pad_connection.map(|c| c.as_str()));
    v["clearance_mm"] = json!(z.clearance);
    v["min_width_mm"] = json!(z.min_thickness);
    v["thermal_gap_mm"] = json!(z.thermal_gap);
    v["thermal_bridge_width_mm"] = json!(z.thermal_bridge_width);
    v["fill"] = json!({
        "enabled": z.fill_enabled,
        "hatched": z.hatched,
        "pieces": g.fills.len(),
        "island_pieces": g.fills.iter().filter(|f| f.island).count(),
        "pieces_by_layer": by_layer,
        "state": if g.fills.is_empty() { "unfilled" } else { "filled" },
    });
    if include_fills {
        v["fill_polygons"] = json!(g
            .fills
            .iter()
            .map(|f| json!({
                "layer": f.layer, "island": f.island,
                "points": f.points.iter().map(|(x, y)| json!([x, y])).collect::<Vec<_>>(),
            }))
            .collect::<Vec<_>>());
    }
    v
}

fn select_copper(s: &SavedBoard, f: &CopperFilters) -> Vec<Found> {
    let b = &s.board;
    let policy = ArcPolicy::default();
    let net_ok = |n: Option<NetId>| match &f.nets {
        None => true,
        Some(set) => n.is_some_and(|n| set.contains(&n)),
    };
    let layer_ok = |m: LayerMask| f.layers.is_none_or(|want| want.intersects(m));
    let lock_ok = |o: Ownership| f.locked.is_none_or(|want| want == (o == Ownership::Locked));
    let mut out = Vec::new();
    if f.kinds.contains("track") {
        for (i, t) in b.tracks.iter().enumerate() {
            if net_ok(t.net)
                && layer_ok(LayerMask::single(t.layer))
                && lock_ok(t.ownership)
                && f.area
                    .as_ref()
                    .is_none_or(|a| a.accepts([&Shape::Segment(t.seg)]))
            {
                out.push(Found::Track(i));
            }
        }
    }
    if f.kinds.contains("arc") {
        for (i, a) in b.arcs.iter().enumerate() {
            if net_ok(a.net) && layer_ok(LayerMask::single(a.layer)) && lock_ok(a.ownership) {
                let pieces: Vec<Shape> = a
                    .segments(&policy)
                    .into_iter()
                    .map(Shape::Segment)
                    .collect();
                if f.area.as_ref().is_none_or(|ar| ar.accepts(&pieces)) {
                    out.push(Found::Arc(i));
                }
            }
        }
    }
    if f.kinds.contains("via") {
        for (i, v) in b.vias.iter().enumerate() {
            if net_ok(v.net)
                && layer_ok(v.layers)
                && lock_ok(v.ownership)
                && f.area.as_ref().is_none_or(|a| a.accepts([&v.shape()]))
            {
                out.push(Found::Via(i));
            }
        }
    }
    let want_zone = f.kinds.contains("zone");
    let want_area = f.kinds.contains("rule_area");
    if want_zone || want_area {
        for (i, z) in s.zones.iter().enumerate() {
            let is_area = z.geometry.keepout.is_some();
            if (is_area && !want_area) || (!is_area && !want_zone) {
                continue;
            }
            let net = z.geometry.net.as_deref().and_then(|n| b.net_by_name(n));
            if f.nets.is_some() && (is_area || !net_ok(net)) {
                continue;
            }
            if !layer_ok(zone_layers(b, z)) {
                continue;
            }
            if f.locked.is_some_and(|want| want != z.locked) {
                continue;
            }
            if let Some(a) = &f.area {
                let side = if is_area {
                    ArcSide::Circumscribed
                } else {
                    ArcSide::Inscribed
                };
                match zone_region(z, side) {
                    Some(poly) => {
                        if !a.accepts([&Shape::Polygon(poly)]) {
                            continue;
                        }
                    }
                    None => continue,
                }
            }
            out.push(if is_area {
                Found::RuleArea(i)
            } else {
                Found::Zone(i)
            });
        }
    }
    out
}

// ─── query_pads ──────────────────────────────────────────────────────────────

pub(crate) async fn handle_query_pads(
    args: &Value,
    _ctx: &crate::tools::ToolContext,
) -> anyhow::Result<CallToolResult> {
    let args = args.clone();
    tokio::task::spawn_blocking(move || query_pads_blocking(&args)).await?
}

/// `REF.NUMBER` split at the first dot.
pub(crate) fn split_pad_name(s: &str) -> Option<(&str, &str)> {
    let (r, n) = s.split_once('.')?;
    (!r.is_empty() && !n.is_empty()).then_some((r, n))
}

/// The pads named `REF.NUMBER` in `names`, by index, refusing (under
/// `field`) any name that matches no pad.
pub(crate) fn named_pads(
    board: &RouteBoard,
    names: &[String],
    field: &str,
) -> Result<Vec<usize>, Refusal> {
    let mut chosen = Vec::new();
    let mut missing = Vec::new();
    for name in names {
        let Some((r, n)) = split_pad_name(name) else {
            return Err(invalid(
                field,
                &format!("'{name}' is not REFERENCE.NUMBER (e.g. C12.1)"),
            ));
        };
        let mut any = false;
        for fp in board
            .footprints
            .iter()
            .filter(|f| f.reference.as_deref() == Some(r))
        {
            for i in fp.pad_range.0 as usize..fp.pad_range.1 as usize {
                if board.pads[i].number == n {
                    any = true;
                    chosen.push(i);
                }
            }
        }
        if !any {
            missing.push(name.clone());
        }
    }
    if !missing.is_empty() {
        return Err(invalid(
            field,
            &format!("no such pad on the board: {}", missing.join(", ")),
        ));
    }
    Ok(chosen)
}

/// Which pads `references` and `pads` name, refusing any name that matches
/// nothing. `None` when neither is given.
fn pad_identity_arg(board: &RouteBoard, args: &Value) -> Result<Option<BTreeSet<usize>>, Refusal> {
    let refs = opt_strings(args, "references")?;
    let pads = opt_strings(args, "pads")?;
    if refs.is_none() && pads.is_none() {
        return Ok(None);
    }
    let mut chosen = BTreeSet::new();
    let mut missing = Vec::new();
    for r in refs.iter().flatten() {
        let mut any = false;
        for fp in board
            .footprints
            .iter()
            .filter(|f| f.reference.as_deref() == Some(r))
        {
            any = true;
            chosen.extend(fp.pad_range.0 as usize..fp.pad_range.1 as usize);
        }
        if !any {
            missing.push(r.clone());
        }
    }
    if !missing.is_empty() {
        return Err(invalid(
            "references",
            &format!("not on the board: {}", missing.join(", ")),
        ));
    }
    if let Some(names) = &pads {
        chosen.extend(named_pads(board, names, "pads")?);
    }
    Ok(Some(chosen))
}

pub(crate) fn pad_kind_name(k: PadKind) -> &'static str {
    match k {
        PadKind::Smd => "smd",
        PadKind::ThroughHole => "thru_hole",
        PadKind::NpThroughHole => "np_thru_hole",
        PadKind::Connect => "connect",
    }
}

pub(crate) fn pad_label(board: &RouteBoard, pad: &Pad) -> String {
    format!(
        "{}.{}",
        board.footprints[pad.footprint.index()]
            .reference
            .as_deref()
            .unwrap_or("?"),
        pad.number
    )
}

fn drill_json(pad: &Pad) -> Value {
    let Some(d) = &pad.drill else {
        return Value::Null;
    };
    match &d.shape {
        Shape::Circle(c) => json!({
            "shape": "circle", "center": pt(c.center), "diameter_mm": mm(2 * c.radius), "plated": d.plated
        }),
        Shape::Segment(g) => json!({
            "shape": "oblong", "a": pt(g.a), "b": pt(g.b), "width_mm": mm(g.width),
            "length_mm": nm_to_mm(g.length() + i64::from(g.width)), "plated": d.plated
        }),
        other => json!({ "shape": "other", "outline": shape_json(other), "plated": d.plated }),
    }
}

pub(crate) fn pad_json(board: &RouteBoard, pad: &Pad, compact: bool) -> Value {
    let fp = &board.footprints[pad.footprint.index()];
    let (x, y) = pad.center.to_mm();
    let copper = pad.shapes.iter().map(Shape::bbox).reduce(BBox::union);
    let mut v = json!({
        "reference": fp.reference,
        "number": pad.number,
        "net": net_name(board, pad.net),
        "kind": pad_kind_name(pad.kind),
        "x": x, "y": y,
        "angle_deg": pad.angle_deg,
        "footprint_rotation_deg": fp.rotation_deg,
        "side": match fp.side { Side::Front => "front", Side::Back => "back" },
        "layers": layer_names(board, pad.layers),
        "drill": drill_json(pad),
        "bbox": copper.map(|b| bbox_json(&b)),
        "approximated": pad.approximated,
        "local_clearance_mm": pad.local_clearance.map(mm),
        "uuid": pad.uuid,
    });
    if !compact {
        v["shapes"] = json!(pad.shapes.iter().map(shape_json).collect::<Vec<_>>());
    }
    v
}

fn pad_kinds_arg(args: &Value) -> Result<Option<BTreeSet<&'static str>>, Refusal> {
    let Some(names) = opt_strings(args, "pad_kinds")? else {
        return Ok(None);
    };
    let all = ["smd", "thru_hole", "np_thru_hole", "connect"];
    let mut out = BTreeSet::new();
    for n in &names {
        match all.iter().find(|k| **k == n.as_str()) {
            Some(k) => {
                out.insert(*k);
            }
            None => {
                return Err(invalid(
                    "pad_kinds",
                    &format!("'{n}' is not one of {}", all.join(", ")),
                ))
            }
        }
    }
    Ok(Some(out))
}

fn query_pads_blocking(args: &Value) -> anyhow::Result<CallToolResult> {
    let saved = match load_saved_board(args)? {
        Ok(s) => s,
        Err(r) => return Ok(r),
    };
    let b = &saved.board;
    let parsed = (|| -> Result<_, Refusal> {
        let area = area_arg(args)?;
        let nets = nets_arg(b, args)?;
        let layers = layers_arg(b, args, "layers")?;
        let identity = pad_identity_arg(b, args)?;
        let kinds = pad_kinds_arg(args)?;
        let compact = opt_bool(args, "compact", false)?;
        let max_items = opt_count(args, "max_items", ITEMS_DEFAULT, ITEMS_MAX)? as usize;
        Ok((area, nets, layers, identity, kinds, compact, max_items))
    })();
    let (area, nets, layers, identity, kinds, compact, max_items) = match parsed {
        Ok(v) => v,
        Err(r) => return Ok(r),
    };
    let t = std::time::Instant::now();
    let found: Vec<&Pad> = b
        .pads
        .iter()
        .enumerate()
        .filter(|(i, p)| {
            identity.as_ref().is_none_or(|set| set.contains(i))
                && nets
                    .as_ref()
                    .is_none_or(|set| p.net.is_some_and(|n| set.contains(&n)))
                && layers.is_none_or(|m| m.intersects(p.layers))
                && kinds
                    .as_ref()
                    .is_none_or(|k| k.contains(pad_kind_name(p.kind)))
                && area.as_ref().is_none_or(|a| a.accepts(&p.shapes))
        })
        .map(|(_, p)| p)
        .collect();
    let pads: Vec<Value> = found
        .iter()
        .take(max_items)
        .map(|p| pad_json(b, p, compact))
        .collect();
    Ok(CallToolResult::json(&json!({
        "source": "saved_file",
        "board": saved.path,
        "note": SAVED_FILE_NOTE,
        "outline_note": "shapes are the exact copper in board coordinates (mm), the same on every layer in 'layers': a circle, a stadium (centre line a-b swept by half width_mm), or a polygon grown by corner_radius_mm. 'approximated' marks a pad the loader could only bound (chamfers, per-layer padstacks).",
        "matched": found.len(),
        "returned": pads.len(),
        "truncated": pads.len() < found.len(),
        "elapsed_ms": t.elapsed().as_millis() as u64,
        "load_report": saved.load_report_json(),
        "pads": pads,
    })))
}
