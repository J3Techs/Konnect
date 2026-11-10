//! `build_route` (`native_routing`): one net's route on one copper layer
//! from a polyline the caller specifies, checked and written as an
//! `add_routing` routing_file. It is the hand-specification path beside
//! `plan_trunk`: nothing is searched and nothing is snapped.
//!
//! The polyline is the caller's `vertices`, or `parallel_to`: another
//! route's chain of tracks offset sideways by a pitch. `extend_to` may then
//! turn the first or last run into an end anchor by a 0, 45 or 90 degree run,
//! computing the vertex that lands it exactly. Each end may be anchored on a
//! pad (`REF.PAD`), a via by UUID or a via by position; the end vertex must
//! lie within 1 micron of it, and the anchor must be of the route's net with
//! copper on its layer. An end may instead place a `new_via`: a through via
//! of the route's net exactly on the end vertex, so the route changes layers
//! there and a later call lands on it with `via_at` (this plan in its
//! `avoid_routing_files`). A new via meets the tap-via site rules
//! (`fanout::tap::check_via_site`: the via floor, clearances, no SMD pad of
//! its own net unless `via_in_pad` names the pad and the via lies wholly
//! inside its copper, outside other nets' islands on every layer unless
//! `carve_zones` names them) and goes into the plan with the tracks.
//!
//! `replaces` / `replaces_file` name board tracks, arcs and vias the plan
//! replaces: the check and the lint run on the board without them, an anchor
//! may not be one of them, and their UUIDs are written beside the plan
//! (`<stem>.replaces.json`, `{"delete_uuids": [...]}`) for `delete_routing`.
//!
//! The geometry is refused, naming the segment or vertex, when a segment is
//! not 0, 45 or 90 degrees within 1 nm (unless `allow_any_angle`), is shorter
//! than `min_segment_mm`, bends at an interior angle under
//! `min_bend_angle_deg`, or crosses another segment of the route. The route
//! is then checked with `check_route_rules`' exact kernel against the saved
//! board plus `avoid_routing_files`, linted as `lint_board_geometry` would,
//! and written only when the check finds nothing.

use crate::mcp::protocol::CallToolResult;
use crate::native_route_job::guarded;
use crate::tools::board_inspect::{self, mm, pt};
use crate::tools::route_native::{
    base_spec, invalid, loaded, opt_bool, opt_strings, stage_failure, Refusal,
};
use crate::tools::route_plan_tools::{
    absolute, board_item_label, carve_labels, carve_zones_arg_hint, check_json, length_arg,
    point_arg, read_avoid, refuse_existing, replaces_arg, via_in_pad_arg, write_plan, Replaced,
};
use crate::tools::{board_lint, get_path, route_geometry, ToolContext};
use konnect_geom::{Coord, Point, Segment};
use konnect_route::api::{Generation, LayerId, NetId};
use konnect_route::check::CheckedItem;
use konnect_route::plan::{RoutePlan, RoutedCopper, RoutedTrack, RoutedVia};
use konnect_route::RouteBoard;
use serde_json::{json, Value};
use std::path::{Path, PathBuf};

/// How far an end vertex may lie from its anchor, nm (1 micron).
const ANCHOR_TOL_NM: f64 = 1_000.0;
/// How far a segment may be off 0, 45 or 90 degrees, nm (1e-6 mm).
const OCTILINEAR_TOL_NM: f64 = 1.0;
const MAX_VERTICES: usize = 10_000;
const MAX_WIDTH_MM: f64 = 20.0;
/// Track ends closer than this (nm) continue one chain (`route_geometry`'s
/// 0.5 micron).
const JOIN_NM: f64 = 500.0;

/// Compass names of the octilinear directions, y down (S is down the board).
const COMPASS: [(i64, i64, &str); 8] = [
    (1, 0, "E"),
    (1, 1, "SE"),
    (0, 1, "S"),
    (-1, 1, "SW"),
    (-1, 0, "W"),
    (-1, -1, "NW"),
    (0, -1, "N"),
    (1, -1, "NE"),
];

pub(crate) fn schema() -> Value {
    let anchor = |end: &str| {
        json!({
            "type": "object",
            "description": format!("What the {end} vertex lands on: exactly one of pad, via_uuid, via_at or new_via. A pad or via must be of the route's net with copper on its layer, and the vertex must lie within 1 micron of it (a pad's anchor, the hole centre of a through-hole pad; a via's centre). A via listed in replaces cannot be an anchor"),
            "properties": {
                "pad": { "type": "string", "minLength": 3, "description": "REFERENCE.NUMBER, e.g. J1.4" },
                "via_uuid": { "type": "string", "minLength": 1, "description": "UUID of a via on the saved board" },
                "via_at": { "type": "object", "description": "Position of a via on the saved board or in avoid_routing_files, mm (within 1 micron)",
                            "properties": { "x": { "type": "number" }, "y": { "type": "number" } }, "required": ["x", "y"] },
                "new_via": { "type": "object", "additionalProperties": false,
                             "description": format!("Place a new through via of the route's net exactly at the {end} vertex, so the route can change layers there (a later build_route anchors on it with via_at and this plan in avoid_routing_files). It goes into the routing_file and is checked with the tracks; it must also meet the tap-via site rules (the via floor, clearances, no SMD pad of its own net under it unless via_in_pad names that pad, and outside other nets' islands on every layer unless carve_zones names them). Refused when a via of the net already sits there (use via_at), or together with extend_to on the same end"),
                             "properties": {
                                 "diameter_mm": { "type": "number", "exclusiveMinimum": 0, "maximum": 20, "description": "Default: the net class via diameter" },
                                 "drill_mm": { "type": "number", "exclusiveMinimum": 0, "maximum": 20, "description": "Default: the net class via drill" }
                             } }
            }
        })
    };
    let turn = |end: &str| {
        json!({ "type": "integer", "enum": [0, 45, 90],
                "description": format!("Land on the {end} anchor by a run turning this many degrees off the {end} run as given (0: the {end} run itself, extended or shortened along its line to the anchor). The vertex where the turn is made is computed; the given {end} vertex only sets the line of that run, and the anchor itself is not one of the vertices") })
    };
    crate::tools::route_native::with_props(
        json!({
            "net": { "type": "string", "description": "The route's net" },
            "layer": { "type": "string", "description": "Copper layer, e.g. In2.Cu" },
            "vertices": { "type": "array", "minItems": 2, "maxItems": MAX_VERTICES, "description": "The polyline, mm, in order (not with parallel_to)",
                          "items": { "type": "object", "properties": { "x": { "type": "number" }, "y": { "type": "number" } }, "required": ["x", "y"] } },
            "width_mm": { "type": "number", "exclusiveMinimum": 0, "maximum": MAX_WIDTH_MM, "description": "Width of every segment" },
            "segment_widths": { "type": "array", "minItems": 1, "items": { "type": "number", "exclusiveMinimum": 0, "maximum": MAX_WIDTH_MM },
                                "description": "Instead of width_mm: one width per segment of the polyline as given (before extend_to; a run extend_to adds takes the width of the run it turns off)" },
            "start": anchor("first"),
            "end": anchor("last"),
            "extend_to": { "type": "object", "description": "Compute the turn into an end anchor instead of giving it",
                           "properties": { "start": turn("first"), "end": turn("last") } },
            "allow_any_angle": { "type": "boolean", "default": false, "description": "Accept segments at any angle; otherwise every segment must be 0, 45 or 90 degrees within 1 nm" },
            "min_bend_angle_deg": { "type": "number", "minimum": 0, "maximum": 180, "default": 90, "description": "Smallest interior angle allowed at a bend (180 is straight)" },
            "min_segment_mm": { "type": "number", "minimum": 0.001, "maximum": 50, "default": 0.25, "description": "Shortest segment allowed" },
            "avoid_routing_files": { "type": "array", "items": { "type": "string", "minLength": 1 }, "minItems": 1,
                                     "description": "Planned copper (routing_files) the route is checked against as if it were on the board; their vias also serve via_at anchors" },
            "carve_zones": { "type": "array", "items": { "type": "string", "minLength": 1 }, "minItems": 1,
                             "description": "Zones (exact names) a refill will carve around the new copper. On the route's layer their fill is no obstacle (as for plan_trunk; the exact check leaves fills to the refill anyway) and the reply gives the route length over each; a new_via may sit inside their islands on any copper layer and lists the pours it carves. Each must be a copper zone of the board; without a new_via it must have copper on the route's layer" },
            "via_in_pad": { "type": "array", "items": { "type": "string", "minLength": 3 }, "minItems": 1,
                            "description": "SMD pads of the route's net (REFERENCE.NUMBER) a new_via may sit inside, for a large pad with no legal via site beside it. The via's copper must lie wholly inside the pad's copper (a via across the pad's edge is refused); every other check stays, and the new via reports in_pad. Unless the fabricator fills and caps the via, an open via in a pad wicks solder away from the joint. Needs a start or end new_via. Default: none, and no new via may touch an SMD pad of its net" },
            "replaces": crate::tools::route_native::replaces_prop(),
            "replaces_file": crate::tools::route_native::replaces_file_prop(),
            "parallel_to": { "type": "object", "description": "Instead of vertices: the polyline offset sideways from another route, one unbranched chain of tracks of one net on this layer in a routing_file (another net's plan, or a build_route output), run in the direction of its first listed track. Collinear tracks are one run. That file is also checked against",
                             "properties": {
                                 "routing_file": { "type": "string", "minLength": 1 },
                                 "net": { "type": "string", "description": "The chain's net (default: the only net with tracks on this layer in the file)" },
                                 "pitch_mm": { "type": "number", "exclusiveMinimum": 0, "maximum": 50, "description": "Centre-to-centre distance from the chain" },
                                 "side": { "type": "string", "enum": ["left", "right"], "description": "Which side of the chain, looking along it as drawn on the board (y down)" }
                             },
                             "required": ["routing_file", "pitch_mm", "side"] },
            "output_plan": { "type": "string", "description": "New routing_file to write; an existing file is never replaced. With replaces, the UUIDs are also written beside it as <output_plan stem>.replaces.json ({\"delete_uuids\": [...]}) for delete_routing's uuids_file; that file is never replaced either" }
        }),
        &["board", "net", "layer", "output_plan"],
    )
}

pub(crate) async fn handle_build_route(
    args: &Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let args = args.clone();
    tokio::task::spawn_blocking(move || build_route_blocking(&args)).await?
}

// ─── Small geometry ──────────────────────────────────────────────────────────

fn mm_pair(p: Point) -> (f64, f64) {
    p.to_mm()
}

fn fmt_pt(p: Point) -> String {
    let (x, y) = mm_pair(p);
    format!("({x}, {y})")
}

fn round4(v: f64) -> f64 {
    (v * 10_000.0).round() / 10_000.0
}

/// How far (nm) the segment `a b` is from the nearest 0, 45 or 90 degree
/// direction, measured at its far end.
fn octilinear_error(a: Point, b: Point) -> f64 {
    let v = b - a;
    let (dx, dy) = (v.x.abs() as f64, v.y.abs() as f64);
    let straight = dx.min(dy);
    let diagonal = (dx - dy).abs() / std::f64::consts::SQRT_2;
    straight.min(diagonal)
}

/// The compass name of an exactly octilinear direction.
fn compass(a: Point, b: Point) -> Option<&'static str> {
    let v = b - a;
    let (dx, dy) = (v.x, v.y);
    if dx == 0 && dy == 0 || !(dx == 0 || dy == 0 || dx.abs() == dy.abs()) {
        return None;
    }
    COMPASS
        .iter()
        .find(|(cx, cy, _)| *cx == dx.signum() && *cy == dy.signum())
        .map(|c| c.2)
}

/// The octilinear direction index (into [`COMPASS`]) of `a b`, when it is
/// exactly octilinear.
fn dir_index(a: Point, b: Point) -> Option<usize> {
    let name = compass(a, b)?;
    COMPASS.iter().position(|c| c.2 == name)
}

/// Interior angle at `q` between `p q` and `q r` (180 is straight).
fn interior_deg(p: Point, q: Point, r: Point) -> f64 {
    let h1 = route_geometry::heading(mm_pair(q), mm_pair(p));
    let h2 = route_geometry::heading(mm_pair(q), mm_pair(r));
    route_geometry::turn(h1, h2).abs()
}

fn orient(p: Point, q: Point, r: Point) -> i128 {
    (q - p).cross(r - p).signum()
}

fn within(p: Point, q: Point, r: Point) -> bool {
    r.x >= p.x.min(q.x) && r.x <= p.x.max(q.x) && r.y >= p.y.min(q.y) && r.y <= p.y.max(q.y)
}

fn segments_meet(p1: Point, p2: Point, q1: Point, q2: Point) -> bool {
    let (d1, d2) = (orient(q1, q2, p1), orient(q1, q2, p2));
    let (d3, d4) = (orient(p1, p2, q1), orient(p1, p2, q2));
    if d1 * d2 < 0 && d3 * d4 < 0 {
        return true;
    }
    (d1 == 0 && within(q1, q2, p1))
        || (d2 == 0 && within(q1, q2, p2))
        || (d3 == 0 && within(p1, p2, q1))
        || (d4 == 0 && within(p1, p2, q2))
}

fn to_point(x: f64, y: f64) -> Option<Point> {
    let fits = |v: f64| v.is_finite() && v.abs() < f64::from(konnect_geom::MAX_COORD);
    (fits(x) && fits(y)).then(|| Point::new(x.round() as Coord, y.round() as Coord))
}

// ─── Anchors ─────────────────────────────────────────────────────────────────

/// An end anchor, resolved on the board.
#[derive(Debug, Clone)]
pub(crate) struct Anchor {
    kind: &'static str,
    name: String,
    at: Point,
}

impl Anchor {
    fn json(&self, vertex: Point) -> Value {
        json!({
            "kind": self.kind, "name": self.name, "at": pt(self.at),
            "offset_mm": round4((vertex - self.at).length_f64() / 1e6),
        })
    }
}

/// A new through via the route places at an end vertex.
#[derive(Debug, Clone)]
pub(crate) struct NewVia {
    diameter: Coord,
    drill: Coord,
    /// `net_class` or `arguments`: where the size came from.
    sized_from: &'static str,
}

/// What an end lands on.
#[derive(Debug, Clone)]
pub(crate) enum End {
    Anchor(Anchor),
    NewVia(NewVia),
}

impl End {
    fn anchor(&self) -> Option<&Anchor> {
        match self {
            End::Anchor(a) => Some(a),
            End::NewVia(_) => None,
        }
    }
}

/// `{key}.new_via`: the via size, the net class via's where not given.
fn new_via_arg(
    v: &Value,
    field: &str,
    rules: &konnect_route::ProjectRuleTable,
    net: NetId,
) -> Result<NewVia, Refusal> {
    let Some(o) = v.as_object() else {
        return Err(invalid(
            field,
            "must be an object (diameter_mm and drill_mm are optional)",
        ));
    };
    if let Some(k) = o
        .keys()
        .find(|k| !["diameter_mm", "drill_mm"].contains(&k.as_str()))
    {
        return Err(invalid(
            &format!("{field}.{k}"),
            "is not diameter_mm or drill_mm",
        ));
    }
    let class = konnect_route::RuleTable::via(
        rules,
        &konnect_route::rules::RuleSubject::new(konnect_route::api::ItemKind::Via, Some(net)),
    );
    let size = |k: &str| -> Result<Option<Coord>, Refusal> {
        match o.get(k).filter(|v| !v.is_null()) {
            None => Ok(None),
            Some(v) => v
                .as_f64()
                .filter(|x| x.is_finite() && *x > 0.0 && *x <= MAX_WIDTH_MM)
                .and_then(konnect_geom::mm_to_nm)
                .filter(|n| *n > 0)
                .map(Some)
                .ok_or_else(|| {
                    invalid(
                        &format!("{field}.{k}"),
                        &format!("must be a size above 0 and at most {MAX_WIDTH_MM} mm"),
                    )
                }),
        }
    };
    let (d, dr) = (size("diameter_mm")?, size("drill_mm")?);
    let sized_from = if d.is_none() && dr.is_none() {
        "net_class"
    } else {
        "arguments"
    };
    let (diameter, drill) = (d.unwrap_or(class.diameter), dr.unwrap_or(class.drill));
    if drill >= diameter {
        return Err(invalid(
            field,
            &format!(
                "the drill ({} mm) must be smaller than the diameter ({} mm)",
                mm(drill),
                mm(diameter)
            ),
        ));
    }
    Ok(NewVia {
        diameter,
        drill,
        sized_from,
    })
}

#[allow(clippy::too_many_arguments)]
fn anchor_arg(
    board: &RouteBoard,
    rules: &konnect_route::ProjectRuleTable,
    avoid: &RoutedCopper,
    replaced: &Replaced,
    args: &Value,
    key: &str,
    net: NetId,
    layer: LayerId,
) -> Result<Option<End>, Refusal> {
    let Some(v) = args.get(key).filter(|v| !v.is_null()) else {
        return Ok(None);
    };
    let Some(o) = v.as_object() else {
        return Err(invalid(
            key,
            "must be an object with one of pad, via_uuid, via_at or new_via",
        ));
    };
    let given: Vec<&str> = ["pad", "via_uuid", "via_at", "new_via"]
        .into_iter()
        .filter(|k| o.get(*k).is_some_and(|v| !v.is_null()))
        .collect();
    let [which] = given[..] else {
        return Err(invalid(
            key,
            "give exactly one of pad (REFERENCE.NUMBER), via_uuid, via_at or new_via",
        ));
    };
    if which == "new_via" {
        let field = format!("{key}.new_via");
        return new_via_arg(&o["new_via"], &field, rules, net).map(|n| Some(End::NewVia(n)));
    }
    anchor_existing(board, avoid, replaced, o, which, key, net, layer).map(|a| a.map(End::Anchor))
}

/// A `pad`, `via_uuid` or `via_at` anchor, resolved.
#[allow(clippy::too_many_arguments)]
fn anchor_existing(
    board: &RouteBoard,
    avoid: &RoutedCopper,
    replaced: &Replaced,
    o: &serde_json::Map<String, Value>,
    which: &str,
    key: &str,
    net: NetId,
    layer: LayerId,
) -> Result<Option<Anchor>, Refusal> {
    let net_name = |n: Option<NetId>| n.map_or("no net".to_string(), |n| board.net(n).name.clone());
    let layer_name = board.layers.name(layer);
    let route_net = &board.net(net).name;
    match which {
        "pad" => {
            let field = format!("{key}.pad");
            let Some(name) = o["pad"].as_str() else {
                return Err(invalid(&field, "must be REFERENCE.NUMBER"));
            };
            let found = board_inspect::named_pads(board, &[name.to_string()], &field)?;
            let [index] = found[..] else {
                return Err(invalid(
                    &field,
                    &format!("'{name}' names {} pads (several footprints share the reference or the number)", found.len()),
                ));
            };
            let pad = &board.pads[index];
            if !pad.layers.contains(layer) {
                return Err(invalid(
                    &field,
                    &format!("{name} has no copper on {layer_name}"),
                ));
            }
            if pad.net != Some(net) {
                return Err(invalid(
                    &field,
                    &format!("{name} is on {}, not {route_net}", net_name(pad.net)),
                ));
            }
            Ok(Some(Anchor {
                kind: "pad",
                name: name.to_string(),
                at: pad.center,
            }))
        }
        "via_uuid" => {
            let field = format!("{key}.via_uuid");
            let Some(uuid) = o["via_uuid"].as_str() else {
                return Err(invalid(&field, "must be a UUID string"));
            };
            let Some(via) = board
                .vias
                .iter()
                .find(|v| v.uuid.as_deref() == Some(uuid.trim()))
            else {
                return Err(invalid(
                    &field,
                    &format!("no via of the saved board has UUID {uuid}"),
                ));
            };
            if !via.layers.contains(layer) {
                return Err(invalid(
                    &field,
                    &format!("the via {uuid} does not reach {layer_name}"),
                ));
            }
            if via.net != Some(net) {
                return Err(invalid(
                    &field,
                    &format!(
                        "the via {uuid} is on {}, not {route_net}",
                        net_name(via.net)
                    ),
                ));
            }
            if replaced.has_via(board, uuid.trim()) {
                return Err(invalid(
                    &field,
                    &format!("the via {uuid} is listed in replaces: it will be deleted, so the route cannot land on it (place a new_via there, or leave the via out of replaces)"),
                ));
            }
            Ok(Some(Anchor {
                kind: "via",
                name: uuid.to_string(),
                at: via.center,
            }))
        }
        _ => {
            let field = format!("{key}.via_at");
            let at = point_arg(&o["via_at"], &field)?;
            let near = |c: Point| (c - at).length_f64() <= ANCHOR_TOL_NM;
            // (centre, net, source, replaced)
            let mut found: Vec<(Point, Option<NetId>, String, bool)> = board
                .vias
                .iter()
                .enumerate()
                .filter(|(_, v)| near(v.center) && v.layers.contains(layer))
                .map(|(i, v)| {
                    (
                        v.center,
                        v.net,
                        "saved board".to_string(),
                        replaced.vias.contains(&i),
                    )
                })
                .collect();
            found.extend(avoid.vias.iter().filter(|v| near(v.center)).map(|v| {
                (
                    v.center,
                    Some(v.net),
                    "avoid_routing_files".to_string(),
                    false,
                )
            }));
            if found.is_empty() {
                return Err(invalid(
                    &field,
                    &format!(
                        "no via of the saved board or of avoid_routing_files lies within 1 micron of {} on {layer_name}",
                        fmt_pt(at)
                    ),
                ));
            }
            if !found.iter().any(|f| f.1 == Some(net) && !f.3) {
                if let Some((center, ..)) = found.iter().find(|f| f.1 == Some(net) && f.3) {
                    return Err(invalid(
                        &field,
                        &format!("the via of {route_net} at {} is listed in replaces: it will be deleted, so the route cannot land on it (place a new_via there, or leave the via out of replaces)", fmt_pt(*center)),
                    ));
                }
            }
            let Some((center, _, source, _)) = found.iter().find(|f| f.1 == Some(net) && !f.3)
            else {
                return Err(invalid(
                    &field,
                    &format!(
                        "the via at {} is on {}, not {route_net}",
                        fmt_pt(found[0].0),
                        net_name(found[0].1)
                    ),
                ));
            };
            Ok(Some(Anchor {
                kind: "via",
                name: format!("via at {} ({source})", fmt_pt(*center)),
                at: *center,
            }))
        }
    }
}

// ─── extend_to ───────────────────────────────────────────────────────────────

/// `v` with its last run turned by `turn_deg` into `p`. With 0 the last run
/// is extended or shortened along its line to `p`, which must lie on that
/// line ahead of the run's start. With 45 or 90 the last vertex moves along
/// the last run's line to where a run turning that much (to whichever side
/// reaches `p`) leads exactly to `p`, which is appended. Octilinear runs are
/// solved in whole nanometres.
fn extend_end(v: &[Point], p: Point, turn_deg: u32, line_tol: f64) -> Result<Vec<Point>, String> {
    let n = v.len();
    let (s, e) = (v[n - 2], v[n - 1]);
    let u = e - s;
    let w = p - s;
    let ulen = u.length_f64();
    let (ux, uy) = (u.x as f64 / ulen, u.y as f64 / ulen);
    let (wx, wy) = (w.x as f64, w.y as f64);
    let along = ux * wx + uy * wy;
    let off = ux * wy - uy * wx;
    let mut out = v[..n - 1].to_vec();
    if turn_deg == 0 {
        if off.abs() > line_tol {
            return Err(format!(
                "lies {:.6} mm off the line of the run it would extend; a 0 degree landing needs it on that line within {} mm (give vertices or a pitch that put it there, or turn 45 or 90)",
                off.abs() / 1e6,
                line_tol / 1e6
            ));
        }
        if along <= 0.0 {
            return Err("lies behind the start of the run it would extend".to_string());
        }
        out.push(p);
        return Ok(out);
    }
    let steps = (turn_deg / 45) as usize;
    // Exactly octilinear: integer directions, the remainder solved exactly.
    if let Some(d0) = dir_index(s, e) {
        for side in [steps, 8 - steps] {
            let (dx, dy) = (COMPASS[d0].0, COMPASS[d0].1);
            let (tx, ty) = (COMPASS[(d0 + side) % 8].0, COMPASS[(d0 + side) % 8].1);
            let det = dx * ty - dy * tx;
            let (a_num, t_num) = (w.x * ty - w.y * tx, dx * w.y - dy * w.x);
            if a_num * det <= 0 || t_num * det <= 0 {
                continue;
            }
            // s + a d = q, q + t t = p; a half nanometre (two diagonals)
            // rounds up.
            let a2 = 2 * a_num / det;
            let (qx, qy) = (2 * i64::from(s.x) + dx * a2, 2 * i64::from(s.y) + dy * a2);
            let q = Point::new(
                Coord::try_from(qx.div_euclid(2) + qx.rem_euclid(2))
                    .map_err(|_| "leaves the coordinate range".to_string())?,
                Coord::try_from(qy.div_euclid(2) + qy.rem_euclid(2))
                    .map_err(|_| "leaves the coordinate range".to_string())?,
            );
            out.push(q);
            out.push(p);
            return Ok(out);
        }
    } else {
        for sign in [1.0f64, -1.0] {
            let th = sign * f64::from(turn_deg).to_radians();
            let (dx, dy) = (ux * th.cos() - uy * th.sin(), ux * th.sin() + uy * th.cos());
            let det = ux * dy - uy * dx;
            let a = (wx * dy - wy * dx) / det;
            let t = (ux * wy - uy * wx) / det;
            if a > 0.0 && t > 0.0 {
                let q = to_point(f64::from(s.x) + a * ux, f64::from(s.y) + a * uy)
                    .ok_or_else(|| "leaves the coordinate range".to_string())?;
                out.push(q);
                out.push(p);
                return Ok(out);
            }
        }
    }
    Err(format!(
        "cannot be reached by a {turn_deg} degree turn off the run: it lies on or behind its line"
    ))
}

fn extend_start(v: &[Point], p: Point, turn_deg: u32, line_tol: f64) -> Result<Vec<Point>, String> {
    let rev: Vec<Point> = v.iter().rev().copied().collect();
    let mut out = extend_end(&rev, p, turn_deg, line_tol)?;
    out.reverse();
    Ok(out)
}

fn extend_arg(args: &Value, end: &str) -> Result<Option<u32>, Refusal> {
    let Some(e) = args.get("extend_to").filter(|v| !v.is_null()) else {
        return Ok(None);
    };
    let Some(o) = e.as_object() else {
        return Err(invalid(
            "extend_to",
            "must be an object with start and/or end",
        ));
    };
    if let Some(k) = o.keys().find(|k| !["start", "end"].contains(&k.as_str())) {
        return Err(invalid(&format!("extend_to.{k}"), "is not start or end"));
    }
    match o.get(end).filter(|v| !v.is_null()) {
        None => Ok(None),
        Some(v) => match v.as_u64() {
            Some(d @ (0 | 45 | 90)) => Ok(Some(d as u32)),
            _ => Err(invalid(&format!("extend_to.{end}"), "must be 0, 45 or 90")),
        },
    }
}

// ─── parallel_to ─────────────────────────────────────────────────────────────

/// One line of an offset polyline: exactly octilinear lines keep an integer
/// form (`y = c`, `x = c`, `x - y = c`, `x + y = c`) so their intersections
/// are whole (or half) nanometres.
#[derive(Debug, Clone, Copy)]
struct OffsetLine {
    /// A point on the line and its unit direction, nm.
    p: (f64, f64),
    d: (f64, f64),
    /// Family (0: y = c, 1: x = c, 2: x - y = c, 3: x + y = c) and c.
    exact: Option<(u8, i64)>,
}

impl OffsetLine {
    fn new(a: Point, b: Point, shift: (f64, f64)) -> OffsetLine {
        let v = b - a;
        let len = v.length_f64();
        let d = (v.x as f64 / len, v.y as f64 / len);
        let p = (f64::from(a.x) + shift.0, f64::from(a.y) + shift.1);
        let exact = dir_index(a, b).map(|i| {
            let (dx, dy) = (COMPASS[i].0, COMPASS[i].1);
            if dy == 0 {
                (0, p.1.round() as i64)
            } else if dx == 0 {
                (1, p.0.round() as i64)
            } else if dx == dy {
                (2, (p.0 - p.1).round() as i64)
            } else {
                (3, (p.0 + p.1).round() as i64)
            }
        });
        OffsetLine { p, d, exact }
    }

    /// The point of the line nearest `q`, on the line exactly when it is
    /// octilinear.
    fn foot(&self, q: (f64, f64)) -> Option<Point> {
        let t = (q.0 - self.p.0) * self.d.0 + (q.1 - self.p.1) * self.d.1;
        let (x, y) = (self.p.0 + t * self.d.0, self.p.1 + t * self.d.1);
        match self.exact {
            Some((0, c)) => to_point(x, c as f64),
            Some((1, c)) => to_point(c as f64, y),
            Some((2, c)) => to_point(x.round(), x.round() - c as f64),
            Some((_, c)) => to_point(x.round(), c as f64 - x.round()),
            None => to_point(x, y),
        }
    }

    fn meet(&self, o: &OffsetLine) -> Option<Point> {
        if let (Some((f1, c1)), Some((f2, c2))) = (self.exact, o.exact) {
            let (f1, c1, f2, c2) = if f1 <= f2 {
                (f1, c1, f2, c2)
            } else {
                (f2, c2, f1, c1)
            };
            let half = |v: i64| v.div_euclid(2) + v.rem_euclid(2);
            let (x, y) = match (f1, f2) {
                (0, 1) => (c2, c1),
                (0, 2) => (c2 + c1, c1),
                (0, 3) => (c2 - c1, c1),
                (1, 2) => (c1, c1 - c2),
                (1, 3) => (c1, c2 - c1),
                (2, 3) => (half(c1 + c2), half(c2 - c1)),
                _ => return None,
            };
            return Some(Point::new(
                Coord::try_from(x).ok()?,
                Coord::try_from(y).ok()?,
            ));
        }
        let det = self.d.0 * o.d.1 - self.d.1 * o.d.0;
        if det.abs() < 1e-12 {
            return None;
        }
        let (wx, wy) = (o.p.0 - self.p.0, o.p.1 - self.p.1);
        let a = (wx * o.d.1 - wy * o.d.0) / det;
        to_point(self.p.0 + a * self.d.0, self.p.1 + a * self.d.1)
    }
}

/// The parsed `parallel_to` argument and the reference chain it names.
struct Parallel {
    path: PathBuf,
    net: String,
    pitch: Coord,
    side: &'static str,
    reference: Vec<Point>,
}

/// The tracks of `net` on `layer` in `plan` as one unbranched chain, run in
/// the direction of the first of them, collinear runs joined.
fn chain_of(plan: &RoutePlan, net: &str, layer: &str) -> Result<Vec<Point>, String> {
    let mut segs: Vec<(Point, Point)> = Vec::new();
    for t in plan
        .tracks
        .iter()
        .filter(|t| t.net == net && t.layer == layer)
    {
        let (Some(a), Some(b)) = (Point::from_mm(t.x1, t.y1), Point::from_mm(t.x2, t.y2)) else {
            return Err("holds a track outside the coordinate range".to_string());
        };
        segs.push((a, b));
    }
    let Some(&(a, b)) = segs.first() else {
        return Err(format!("has no track of {net} on {layer}"));
    };
    let near = |p: Point, q: Point| (p - q).length_f64() <= JOIN_NM;
    let mut used = vec![false; segs.len()];
    used[0] = true;
    let mut chain = std::collections::VecDeque::from([a, b]);
    loop {
        let mut grew = false;
        for at_tail in [true, false] {
            let end = if at_tail {
                *chain.back().expect("two points")
            } else {
                *chain.front().expect("two points")
            };
            let next: Vec<usize> = (0..segs.len())
                .filter(|&i| !used[i] && (near(segs[i].0, end) || near(segs[i].1, end)))
                .collect();
            match next[..] {
                [] => {}
                [i] => {
                    used[i] = true;
                    let other = if near(segs[i].0, end) {
                        segs[i].1
                    } else {
                        segs[i].0
                    };
                    if at_tail {
                        chain.push_back(other);
                    } else {
                        chain.push_front(other);
                    }
                    grew = true;
                }
                _ => {
                    return Err(format!(
                        "branches at {}: the reference must be one unbranched chain",
                        fmt_pt(end)
                    ))
                }
            }
        }
        if !grew {
            break;
        }
    }
    let loose = used.iter().filter(|u| !**u).count();
    if loose > 0 {
        return Err(format!(
            "has {loose} track(s) of {net} on {layer} not joined to the chain of its first track: the reference must be one unbranched chain"
        ));
    }
    let mut out: Vec<Point> = Vec::new();
    for p in chain {
        if out.last().is_some_and(|l| near(*l, p)) {
            continue;
        }
        if out.len() >= 2 {
            let (q, r) = (out[out.len() - 2], out[out.len() - 1]);
            if (r - q).cross(p - r) == 0 && (r - q).dot(p - r) > 0 {
                out.pop();
            }
        }
        out.push(p);
    }
    if out.len() < 2 {
        return Err("chain has no length".to_string());
    }
    Ok(out)
}

/// `reference` offset by `pitch` nm to `side`. Refuses an offset that would
/// fold a run over (too short a run on the inner side of a bend).
fn offset(reference: &[Point], pitch: Coord, side: &str) -> Result<Vec<Point>, String> {
    let sign = if side == "left" { 1.0 } else { -1.0 };
    let lines: Vec<OffsetLine> = reference
        .windows(2)
        .map(|w| {
            let v = w[1] - w[0];
            let len = v.length_f64();
            // Left of the direction of travel, y down: (dy, -dx).
            let n = (sign * v.y as f64 / len, -sign * v.x as f64 / len);
            let p = f64::from(pitch);
            OffsetLine::new(w[0], w[1], (n.0 * p, n.1 * p))
        })
        .collect();
    let first = lines[0]
        .foot((f64::from(reference[0].x), f64::from(reference[0].y)))
        .ok_or("leaves the coordinate range")?;
    let mut out = vec![first];
    for k in 1..lines.len() {
        let q = lines[k - 1]
            .meet(&lines[k])
            .ok_or_else(|| format!("runs {} and {k} are parallel", k - 1))?;
        out.push(q);
    }
    let last_ref = reference[reference.len() - 1];
    let last = lines[lines.len() - 1]
        .foot((f64::from(last_ref.x), f64::from(last_ref.y)))
        .ok_or("leaves the coordinate range")?;
    out.push(last);
    for (i, (o, r)) in out.windows(2).zip(reference.windows(2)).enumerate() {
        let (ov, rv) = (o[1] - o[0], r[1] - r[0]);
        if ov.length2() == 0 || ov.dot(rv) <= 0 {
            return Err(format!(
                "at this pitch run {i} folds over: the reference run is too short on the inner side of its bends"
            ));
        }
    }
    Ok(out)
}

fn parallel_arg(
    args: &Value,
    board: &RouteBoard,
    layer: LayerId,
) -> Result<Option<Parallel>, Refusal> {
    let Some(v) = args.get("parallel_to").filter(|v| !v.is_null()) else {
        return Ok(None);
    };
    let Some(o) = v.as_object() else {
        return Err(invalid("parallel_to", "must be an object"));
    };
    if let Some(k) = o
        .keys()
        .find(|k| !["routing_file", "net", "pitch_mm", "side"].contains(&k.as_str()))
    {
        return Err(invalid(&format!("parallel_to.{k}"), "is not a known key"));
    }
    let Some(path) = o
        .get("routing_file")
        .and_then(Value::as_str)
        .filter(|p| !p.is_empty())
    else {
        return Err(invalid("parallel_to.routing_file", "is required"));
    };
    let path = PathBuf::from(path);
    crate::tools::route_native::file_must_exist(&path, "parallel_to.routing_file")?;
    let plan: RoutePlan = std::fs::read_to_string(&path)
        .map_err(|e| e.to_string())
        .and_then(|t| serde_json::from_str(&t).map_err(|e| e.to_string()))
        .map_err(|e| {
            invalid(
                "parallel_to.routing_file",
                &format!("is not a routing_file: {e}"),
            )
        })?;
    let pitch = length_arg(v, "pitch_mm", 0.001, 50.0)
        .map_err(|_| {
            invalid(
                "parallel_to.pitch_mm",
                "must be a length from 0.001 to 50 mm",
            )
        })?
        .ok_or_else(|| invalid("parallel_to.pitch_mm", "is required"))?;
    let side = match o.get("side").and_then(Value::as_str) {
        Some("left") => "left",
        Some("right") => "right",
        _ => return Err(invalid("parallel_to.side", "must be left or right")),
    };
    let layer_name = board.layers.name(layer);
    let net = match o.get("net").and_then(Value::as_str) {
        Some(n) => n.to_string(),
        None => {
            let mut nets: Vec<&str> = plan
                .tracks
                .iter()
                .filter(|t| t.layer == layer_name)
                .map(|t| t.net.as_str())
                .collect();
            nets.sort_unstable();
            nets.dedup();
            match nets[..] {
                [one] => one.to_string(),
                [] => {
                    return Err(invalid(
                        "parallel_to.routing_file",
                        &format!("has no track on {layer_name}"),
                    ))
                }
                _ => {
                    return Err(invalid(
                        "parallel_to.net",
                        &format!(
                            "is required: the file has tracks of {} on {layer_name}",
                            nets.join(", ")
                        ),
                    ))
                }
            }
        }
    };
    let reference =
        chain_of(&plan, &net, layer_name).map_err(|e| invalid("parallel_to.routing_file", &e))?;
    Ok(Some(Parallel {
        path,
        net,
        pitch,
        side,
        reference,
    }))
}

// ─── The tool ────────────────────────────────────────────────────────────────

fn vertices_arg(args: &Value) -> Result<Option<Vec<Point>>, Refusal> {
    let Some(v) = args.get("vertices").filter(|v| !v.is_null()) else {
        return Ok(None);
    };
    let Some(list) = v.as_array() else {
        return Err(invalid("vertices", "must be an array of {x, y} in mm"));
    };
    if list.len() < 2 || list.len() > MAX_VERTICES {
        return Err(invalid(
            "vertices",
            &format!("needs from 2 to {MAX_VERTICES} points"),
        ));
    }
    list.iter()
        .enumerate()
        .map(|(i, p)| point_arg(p, &format!("vertices[{i}]")))
        .collect::<Result<Vec<_>, _>>()
        .map(Some)
}

fn widths_arg(args: &Value, segments: usize) -> Result<Vec<Coord>, Refusal> {
    let one = args.get("width_mm").filter(|v| !v.is_null());
    let many = args.get("segment_widths").filter(|v| !v.is_null());
    let width = |v: &Value, field: &str| {
        v.as_f64()
            .filter(|w| w.is_finite() && *w > 0.0 && *w <= MAX_WIDTH_MM)
            .and_then(konnect_geom::mm_to_nm)
            .filter(|w| *w > 0)
            .ok_or_else(|| {
                invalid(
                    field,
                    &format!("must be a width above 0 and at most {MAX_WIDTH_MM} mm"),
                )
            })
    };
    match (one, many) {
        (Some(_), Some(_)) => Err(invalid(
            "segment_widths",
            "give width_mm or segment_widths, not both",
        )),
        (None, None) => Err(invalid("width_mm", "is required (or segment_widths)")),
        (Some(w), None) => Ok(vec![width(w, "width_mm")?; segments]),
        (None, Some(list)) => {
            let Some(list) = list.as_array() else {
                return Err(invalid(
                    "segment_widths",
                    "must be an array of widths in mm",
                ));
            };
            if list.len() != segments {
                return Err(invalid(
                    "segment_widths",
                    &format!(
                        "has {} widths; the polyline as given has {segments} segments",
                        list.len()
                    ),
                ));
            }
            list.iter()
                .enumerate()
                .map(|(i, w)| width(w, &format!("segment_widths[{i}]")))
                .collect()
        }
    }
}

/// The final polyline's refusals, in the order they are looked for.
fn check_polyline(
    v: &[Point],
    field: &str,
    any_angle: bool,
    min_segment: Coord,
    min_bend: f64,
) -> Result<(), Refusal> {
    let seg = |i: usize| format!("segment {i} from {} to {}", fmt_pt(v[i]), fmt_pt(v[i + 1]));
    for i in 0..v.len() - 1 {
        let (a, b) = (v[i], v[i + 1]);
        if a == b {
            return Err(invalid(
                field,
                &format!(
                    "vertices {i} and {} are the same point {}",
                    i + 1,
                    fmt_pt(a)
                ),
            ));
        }
        if !any_angle {
            let err = octilinear_error(a, b);
            if err > OCTILINEAR_TOL_NM {
                let v = b - a;
                let deg = (v.y.abs() as f64).atan2(v.x.abs() as f64).to_degrees();
                return Err(invalid(
                    field,
                    &format!(
                        "{} runs at {deg:.4} degrees, {:.6} mm off 0, 45 or 90 degrees (allow_any_angle permits it)",
                        seg(i),
                        err / 1e6
                    ),
                ));
            }
        }
        let len = (b - a).length_f64();
        if len < f64::from(min_segment) - 0.5 {
            return Err(invalid(
                field,
                &format!(
                    "{} is {:.4} mm long, shorter than min_segment_mm {}",
                    seg(i),
                    len / 1e6,
                    f64::from(min_segment) / 1e6
                ),
            ));
        }
    }
    for i in 1..v.len() - 1 {
        let interior = interior_deg(v[i - 1], v[i], v[i + 1]);
        if interior < min_bend - 1e-6 {
            return Err(invalid(
                field,
                &format!(
                    "the bend at vertex {i} {} has an interior angle of {interior:.2} degrees, under min_bend_angle_deg {min_bend}",
                    fmt_pt(v[i])
                ),
            ));
        }
    }
    let n = v.len() - 1;
    for i in 0..n {
        for j in i + 2..n {
            if segments_meet(v[i], v[i + 1], v[j], v[j + 1]) {
                return Err(invalid(
                    field,
                    &format!("segments {i} and {j} of the route cross or touch"),
                ));
            }
        }
    }
    Ok(())
}

fn build_route_blocking(args: &Value) -> anyhow::Result<CallToolResult> {
    let output = absolute(get_path(args, "output_plan")?)?;
    let spec = match base_spec(args, output.clone())? {
        Ok(s) => s,
        Err(r) => return Ok(r),
    };
    if let Some(r) = refuse_existing(&output) {
        return Ok(r);
    }
    let (board, rules) = match loaded(&spec) {
        Ok(v) => v,
        Err(r) => return Ok(r),
    };
    match build(&board, &rules, args, &output) {
        Ok(r) | Err(r) => Ok(r),
    }
}

fn build(
    board: &RouteBoard,
    rules: &konnect_route::ProjectRuleTable,
    args: &Value,
    output: &std::path::Path,
) -> Result<CallToolResult, Refusal> {
    let Some(net_name) = args.get("net").and_then(Value::as_str) else {
        return Err(invalid("net", "is required"));
    };
    let net = board
        .net_by_name(net_name)
        .ok_or_else(|| invalid("net", &format!("'{net_name}' is not a net of the board")))?;
    let Some(layer_name) = args.get("layer").and_then(Value::as_str) else {
        return Err(invalid("layer", "is required (a copper layer name)"));
    };
    let layer = board.layers.by_name(layer_name.trim()).ok_or_else(|| {
        invalid(
            "layer",
            &format!("'{layer_name}' is not a copper layer of the board"),
        )
    })?;
    let any_angle = opt_bool(args, "allow_any_angle", false)?;
    let min_bend = match args.get("min_bend_angle_deg").filter(|v| !v.is_null()) {
        None => 90.0,
        Some(v) => v
            .as_f64()
            .filter(|a| (0.0..=180.0).contains(a))
            .ok_or_else(|| invalid("min_bend_angle_deg", "must be from 0 to 180"))?,
    };
    let min_segment = length_arg(args, "min_segment_mm", 0.001, 50.0)?.unwrap_or(250_000);
    let mut avoid_files = opt_strings(args, "avoid_routing_files")?.unwrap_or_default();
    let parallel = parallel_arg(args, board, layer)?;
    if let Some(p) = &parallel {
        let path = p.path.display().to_string();
        if !avoid_files.contains(&path) {
            avoid_files.push(path);
        }
    }
    let avoid = read_avoid(board, &avoid_files)?;
    // Board copper the plan replaces: checked and linted as deleted, and
    // written beside the plan for delete_routing.
    let replaced = replaces_arg(board, args)?;
    let replaces_out = (!replaced.is_empty()).then(|| replaces_path(output));
    if let Some(r) = replaces_out.as_deref().and_then(refuse_existing) {
        return Err(r);
    }
    let start = anchor_arg(board, rules, &avoid, &replaced, args, "start", net, layer)?;
    let end = anchor_arg(board, rules, &avoid, &replaced, args, "end", net, layer)?;
    let any_new_via = [&start, &end]
        .iter()
        .any(|e| matches!(e, Some(End::NewVia(_))));
    // A new via is a through via: a zone on any copper layer may be carved.
    // Without one the route has copper on its own layer only.
    let carve = carve_zones_arg_hint(
        board,
        args,
        (!any_new_via).then_some(layer),
        ", and without a start or end new_via the route has no copper on other layers",
    )?;

    // Pads a new via may sit inside: of the route's net, and only with a
    // new via to put there.
    let via_in_pad = via_in_pad_arg(board, args)?;
    if let Some(p) = via_in_pad
        .iter()
        .map(|p| &board.pads[p.index()])
        .find(|p| p.net != Some(net))
    {
        return Err(invalid(
            "via_in_pad",
            &format!(
                "{} is on {}, not the route's net {}: only a pad of the via's own net can hold it",
                board_inspect::pad_label(board, p),
                p.net
                    .map_or("no net".to_string(), |n| board.net(n).name.clone()),
                board.net(net).name
            ),
        ));
    }
    if !via_in_pad.is_empty() && !any_new_via {
        return Err(invalid(
            "via_in_pad",
            "needs a start or end new_via: it only says where a new via may sit",
        ));
    }

    // The polyline as given.
    let given = vertices_arg(args)?;
    let (base, field) = match (&given, &parallel) {
        (Some(_), Some(_)) => {
            return Err(invalid(
                "parallel_to",
                "give vertices or parallel_to, not both",
            ))
        }
        (None, None) => return Err(invalid("vertices", "is required (or parallel_to)")),
        (Some(v), None) => (v.clone(), "vertices"),
        (None, Some(p)) => (
            offset(&p.reference, p.pitch, p.side).map_err(|e| invalid("parallel_to", &e))?,
            "parallel_to",
        ),
    };
    if let Some(i) = (0..base.len() - 1).find(|&i| base[i] == base[i + 1]) {
        return Err(invalid(
            field,
            &format!(
                "vertices {i} and {} are the same point {}",
                i + 1,
                fmt_pt(base[i])
            ),
        ));
    }
    let base_widths = widths_arg(args, base.len() - 1)?;

    // extend_to: the turn into an anchor, computed.
    let mut verts = base.clone();
    let mut widths = base_widths.clone();
    let mut extended = serde_json::Map::new();
    let line_tol = if any_angle {
        ANCHOR_TOL_NM
    } else {
        OCTILINEAR_TOL_NM
    };
    for (end_name, anchor) in [("start", &start), ("end", &end)] {
        let Some(turn) = extend_arg(args, end_name)? else {
            continue;
        };
        let field = format!("extend_to.{end_name}");
        let anchor = match anchor {
            None => {
                return Err(invalid(
                    &field,
                    &format!("needs a {end_name} anchor to land on"),
                ))
            }
            Some(End::NewVia(_)) => {
                return Err(invalid(
                    &field,
                    &format!(
                        "cannot be used with {end_name}.new_via: the new via is placed exactly at the {} vertex as given, so there is no anchor to turn into. Drop extend_to.{end_name}, or land on a pad or an existing via",
                        if end_name == "start" { "first" } else { "last" }
                    ),
                ))
            }
            Some(End::Anchor(a)) => a,
        };
        let before = verts.len();
        verts = if end_name == "start" {
            extend_start(&verts, anchor.at, turn, line_tol)
        } else {
            extend_end(&verts, anchor.at, turn, line_tol)
        }
        .map_err(|e| invalid(&field, &format!("{} {} {e}", anchor.kind, anchor.name)))?;
        let added = verts.len() > before;
        if added {
            if end_name == "start" {
                widths.insert(0, widths[0]);
            } else {
                widths.push(*widths.last().expect("a width"));
            }
        }
        let turned_at = if end_name == "start" {
            verts[1]
        } else {
            verts[verts.len() - 2]
        };
        extended.insert(
            end_name.to_string(),
            json!({ "turn_deg": turn, "anchor": anchor.name,
                    "turn_vertex": if added { pt(turned_at) } else { Value::Null } }),
        );
    }

    // Anchors: the end vertices must coincide with them.
    let mut anchors = serde_json::Map::new();
    for (end_name, anchor, vertex, index) in [
        ("start", &start, verts[0], 0),
        ("end", &end, verts[verts.len() - 1], verts.len() - 1),
    ] {
        let Some(anchor) = anchor.as_ref().and_then(End::anchor) else {
            if matches!(anchor, Some(End::NewVia(_))) {
                anchors.insert(
                    end_name.to_string(),
                    json!({ "kind": "new_via", "name": format!("new via at {}", fmt_pt(vertex)),
                            "at": pt(vertex), "offset_mm": 0.0 }),
                );
            }
            continue;
        };
        let off = (vertex - anchor.at).length_f64();
        if off > ANCHOR_TOL_NM {
            return Err(invalid(
                end_name,
                &format!(
                    "the {} vertex {index} {} is {:.4} mm from {} {} at {}; it must lie within 1 micron (extend_to can compute the turn into it)",
                    if end_name == "start" { "first" } else { "last" },
                    fmt_pt(vertex),
                    off / 1e6,
                    anchor.kind,
                    anchor.name,
                    fmt_pt(anchor.at)
                ),
            ));
        }
        anchors.insert(end_name.to_string(), anchor.json(vertex));
    }
    check_polyline(&verts, field, any_angle, min_segment, min_bend)?;

    // The copper, checked against the board (without the copper it
    // replaces) and the avoided copper.
    let mut copper = RoutedCopper {
        tracks: verts
            .windows(2)
            .zip(&widths)
            .map(|(w, &width)| RoutedTrack {
                net,
                layer,
                seg: Segment::new(w[0], w[1], width),
                gen: Generation(0),
            })
            .collect(),
        vias: Vec::new(),
    };
    let mut base_board = replaced.apply(board);
    avoid.freeze_into(&mut base_board);
    // Board tracks the route repeats exactly are the route already applied
    // (re-planning a hand route): checked as the route, not against it.
    let (checked, repeated) = board_lint::without_repeated(&base_board, &copper);

    // New vias at the ends: placed exactly on the end vertex, judged by the
    // tap-via site rules before the exact check.
    let mut new_vias: Vec<(&'static str, Point, NewVia)> = Vec::new();
    for (end_name, e, vertex) in [
        ("start", &start, verts[0]),
        ("end", &end, verts[verts.len() - 1]),
    ] {
        if let Some(End::NewVia(nv)) = e {
            new_vias.push((end_name, vertex, nv.clone()));
        }
    }
    let route_net = board.net(net).name.clone();
    // Per new via: the pad named in via_in_pad it sits wholly inside.
    let mut in_pads: Vec<Option<konnect_route::api::PadId>> = Vec::new();
    for (i, (end_name, at, nv)) in new_vias.iter().enumerate() {
        let field = format!("{end_name}.new_via");
        let near = |c: Point| (c - *at).length_f64() <= ANCHOR_TOL_NM;
        let on_board = board
            .vias
            .iter()
            .enumerate()
            .any(|(k, v)| v.net == Some(net) && near(v.center) && !replaced.vias.contains(&k));
        let in_avoid = avoid.vias.iter().any(|v| v.net == net && near(v.center));
        if on_board || in_avoid {
            return Err(invalid(
                &field,
                &format!(
                    "a via of {route_net} already sits at {} ({}): land on it with {end_name}.via_at instead",
                    fmt_pt(*at),
                    if on_board { "saved board" } else { "avoid_routing_files" }
                ),
            ));
        }
        if i == 1 && near(new_vias[0].1) {
            return Err(invalid(
                &field,
                "lies on the start's new via: the route would place two vias at one point",
            ));
        }
        let verdict = match guarded("via site", || {
            konnect_route::fanout::tap::check_via_site(
                &checked,
                rules,
                net,
                *at,
                nv.diameter,
                nv.drill,
                &carve,
                &via_in_pad,
            )
        }) {
            Ok(v) => v,
            Err(e) => return Err(stage_failure("check_via_site", e)),
        };
        match verdict {
            Ok(pad) => in_pads.push(pad),
            Err(why) => {
                return Err(invalid(
                    &field,
                    &site_refusal(&checked, &why, &route_net, *at, nv),
                ))
            }
        }
    }
    let all_layers = board.layers.all();
    copper.vias = new_vias
        .iter()
        .map(|(_, at, nv)| RoutedVia {
            net,
            center: *at,
            diameter: nv.diameter,
            drill: nv.drill,
            layers: all_layers,
            gen: Generation(0),
        })
        .collect();

    let report = match guarded("check", || {
        konnect_route::check::check_routed(&checked, rules, &copper)
    }) {
        Ok(r) => r,
        Err(e) => return Err(stage_failure("check_routed", e)),
    };
    let mut check = check_json(
        &checked,
        &report,
        match (avoid_files.is_empty(), replaced.is_empty()) {
            (true, true) => "saved board",
            (false, true) => "saved board plus the avoided routing files",
            (true, false) => "saved board without the replaced copper",
            (false, false) => {
                "saved board without the replaced copper, plus the avoided routing files"
            }
        },
        50,
        |item| match item {
            CheckedItem::RoutedTrack { index } => json!(format!("segment {index}")),
            CheckedItem::RoutedVia { index } => match new_vias.get(*index as usize) {
                Some((end_name, ..)) => json!(format!("new via at the {end_name}")),
                None => json!(format!("via {index}")),
            },
            CheckedItem::Board { item } => json!(board_item_label(&checked, *item)),
            CheckedItem::Edge => json!("board edge"),
        },
    );
    check["board_tracks_repeated_set_aside"] = json!(repeated);
    let limits = route_geometry::Limits {
        min_bend_angle_deg: min_bend,
        ..route_geometry::Limits::default()
    };
    let lint = board_lint::lint_planned(
        &base_board,
        &copper,
        &limits,
        board_lint::DOGLEG_DEFAULT_NM,
        50,
    );
    let clean = report.violations.is_empty();
    if clean {
        let routing = RoutePlan::from_routed(board, &copper)
            .map_err(|e| CallToolResult::error(format!("plan not built: {e}")))?;
        write_plan(output, &routing)?;
        if let Some(path) = &replaces_out {
            let body = json!({ "delete_uuids": replaced.uuids, "routing_file": output });
            let text = serde_json::to_string_pretty(&body).unwrap_or_default();
            crate::native_route_job::write_new_atomic(path, text.as_bytes()).map_err(|e| {
                CallToolResult::error(format!(
                    "the plan was written to {}, but the replaced UUIDs could not be written to {}: {e:#}",
                    output.display(),
                    path.display()
                ))
            })?;
        }
    }
    // The pours a refill carves around the new copper: each new via's, and
    // the route's length over each named zone on its layer.
    let via_carves = konnect_route::fanout::via_carves(
        &checked,
        rules,
        &new_vias
            .iter()
            .map(|(_, at, nv)| (net, *at, nv.diameter))
            .collect::<Vec<_>>(),
        &carve,
    );
    let segs: Vec<Segment> = copper.tracks.iter().map(|t| t.seg).collect();
    let carved: Vec<Value> = carve
        .iter()
        .map(|z| {
            let over = konnect_route::trunk::length_over_zone(board, layer, z, &segs);
            json!({ "zone": z, "route_length_mm": round4(over / 1e6) })
        })
        .collect();
    let new_vias_json: Vec<Value> = new_vias
        .iter()
        .zip(&via_carves)
        .zip(&in_pads)
        .map(|(((end_name, at, nv), zones), in_pad)| {
            json!({
                "end": end_name, "at": pt(*at),
                "diameter_mm": mm(nv.diameter), "drill_mm": mm(nv.drill),
                "sized_from": nv.sized_from,
                "in_pad": in_pad.map(|p| board_inspect::pad_label(board, &board.pads[p.index()])),
                "carves": carve_labels(&checked, zones),
            })
        })
        .collect();
    let segments: Vec<Value> = copper
        .tracks
        .iter()
        .map(|t| {
            let (a, b) = (t.seg.a, t.seg.b);
            json!({
                "from": pt(a), "to": pt(b),
                "length_mm": round4((b - a).length_f64() / 1e6),
                "width_mm": mm(t.seg.width),
                "direction": compass(a, b),
                "angle_deg": round4(((a.y as f64) - (b.y as f64)).atan2((b.x as f64) - (a.x as f64)).to_degrees()),
            })
        })
        .collect();
    let shortest = copper
        .tracks
        .iter()
        .map(|t| (t.seg.b - t.seg.a).length_f64())
        .fold(f64::MAX, f64::min);
    let smallest_angle = (1..verts.len() - 1)
        .map(|i| interior_deg(verts[i - 1], verts[i], verts[i + 1]))
        .reduce(f64::min);
    Ok(CallToolResult::json(&json!({
        "source": "saved_file",
        "net": board.net(net).name,
        "layer": board.layers.name(layer),
        "built_from": field,
        "parallel_to": parallel.as_ref().map(|p| json!({
            "routing_file": p.path, "net": p.net, "pitch_mm": mm(p.pitch), "side": p.side,
            "reference": p.reference.iter().map(|q| pt(*q)).collect::<Vec<_>>(),
        })),
        "extend_to": if extended.is_empty() { Value::Null } else { Value::Object(extended) },
        "anchors": anchors,
        "vertices": verts.iter().map(|q| pt(*q)).collect::<Vec<_>>(),
        "segments": segments,
        "shortest_segment_mm": round4(shortest / 1e6),
        "smallest_interior_angle_deg": smallest_angle.map(|a| (a * 100.0).round() / 100.0),
        "limits": {
            "allow_any_angle": any_angle,
            "min_segment_mm": f64::from(min_segment) / 1e6,
            "min_bend_angle_deg": min_bend,
        },
        "avoid_routing_files": avoid_files,
        "new_vias": new_vias_json,
        "carve_zones": carved,
        "replaces": replaced.json(),
        "replaces_output": if clean { json!(replaces_out) } else { Value::Null },
        "check": check,
        "lint": lint,
        "plan_written": clean,
        "output_plan": if clean { json!(output) } else { Value::Null },
        "next": match (clean, replaces_out.is_some()) {
            (true, false) => "Review with render_board_layers_png (overlay_routing_files with the other plans); apply with add_routing (routing_files to add several plans as one batch), then refill and run DRC. A new via here is a via_at anchor for the next build_route with this plan in avoid_routing_files.",
            (true, true) => "Review with render_board_layers_png (overlay_routing_files with the other plans). Apply in the open board: delete_routing with uuids_file set to replaces_output (dry run, then expected_count; include_locked if any is locked), then add_routing with this routing_file; refill and run DRC.",
            (false, _) => "Nothing was written: the exact check found violations (listed). Move the vertices, narrow the widths or change the avoided plans, and build again.",
        },
    })))
}

/// `<output_plan stem>.replaces.json` beside the plan.
fn replaces_path(output: &Path) -> PathBuf {
    let stem = output
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_else(|| "plan".to_string());
    output.with_file_name(format!("{stem}.replaces.json"))
}

/// Why a new via's site is refused, naming what refuses it.
fn site_refusal(
    board: &RouteBoard,
    why: &konnect_route::fanout::tap::ViaSiteRefusal,
    net: &str,
    at: Point,
    nv: &NewVia,
) -> String {
    let mut s = format!(
        "a new via of {net} (diameter {} mm, drill {} mm) at {} breaks the via-site rules: {}",
        mm(nv.diameter),
        mm(nv.drill),
        fmt_pt(at),
        why.reason
    );
    if let Some(item) = why.item {
        s.push_str(&format!(" ({})", board_item_label(board, item)));
    }
    if let Some(z) = why.zone {
        let p = &board.zones[z.index()];
        s.push_str(&format!(
            " (zone {} of {} on {}; name it in carve_zones if the refill may cut an antipad around the via)",
            p.zone_name.as_deref().unwrap_or("(unnamed)"),
            p.net.map_or("no net".to_string(), |n| board.net(n).name.clone()),
            board.layers.name(p.layer)
        ));
    }
    if let Some(k) = why.keepout {
        let area = &board.keepouts[k.index()];
        s.push_str(&format!(
            " (rule area {})",
            area.name.as_deref().unwrap_or("(unnamed)")
        ));
    }
    if let Some(f) = why.floor {
        s.push_str(&format!(
            " (the floor here: diameter {} mm, drill {} mm, annular ring {} mm)",
            mm(f.min_diameter),
            mm(f.min_drill),
            mm(f.min_annular)
        ));
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: f64, y: f64) -> Point {
        Point::from_mm(x, y).unwrap()
    }

    #[test]
    fn extend_end_lands_a_45_degree_run_exactly_on_the_anchor() {
        // East, then down-right into (20, 3): the turn is at (17, 0).
        let v = extend_end(
            &[p(0.0, 0.0), p(5.0, 0.0)],
            p(20.0, 3.0),
            45,
            OCTILINEAR_TOL_NM,
        )
        .unwrap();
        assert_eq!(v, vec![p(0.0, 0.0), p(17.0, 0.0), p(20.0, 3.0)]);
        // Above the line: up-right.
        let v = extend_end(
            &[p(0.0, 0.0), p(5.0, 0.0)],
            p(20.0, -3.0),
            45,
            OCTILINEAR_TOL_NM,
        )
        .unwrap();
        assert_eq!(v[1], p(17.0, 0.0));
        // 90 degrees, and 0 (straight on).
        let v = extend_end(
            &[p(0.0, 0.0), p(5.0, 0.0)],
            p(20.0, 3.0),
            90,
            OCTILINEAR_TOL_NM,
        )
        .unwrap();
        assert_eq!(v[1], p(20.0, 0.0));
        let v = extend_end(
            &[p(0.0, 0.0), p(5.0, 0.0)],
            p(20.0, 0.0),
            0,
            OCTILINEAR_TOL_NM,
        )
        .unwrap();
        assert_eq!(v, vec![p(0.0, 0.0), p(20.0, 0.0)]);
        assert!(extend_end(
            &[p(0.0, 0.0), p(5.0, 0.0)],
            p(20.0, 0.1),
            0,
            OCTILINEAR_TOL_NM
        )
        .is_err());
        // 0 degrees: on the line to 1 nm, not 1 micron (unless any angle is allowed).
        let near = Point::new(20_000_000, 2);
        assert!(extend_end(&[p(0.0, 0.0), p(5.0, 0.0)], near, 0, OCTILINEAR_TOL_NM).is_err());
        assert!(extend_end(&[p(0.0, 0.0), p(5.0, 0.0)], near, 0, ANCHOR_TOL_NM).is_ok());
        // Behind the run.
        assert!(extend_end(
            &[p(0.0, 0.0), p(5.0, 0.0)],
            p(-3.0, 3.0),
            45,
            OCTILINEAR_TOL_NM
        )
        .is_err());
        // The start, mirrored: a diagonal run turning onto a vertical.
        let v = extend_start(
            &[p(10.0, 10.0), p(15.0, 15.0)],
            p(10.0, 2.0),
            45,
            OCTILINEAR_TOL_NM,
        )
        .unwrap();
        assert_eq!(v, vec![p(10.0, 2.0), p(10.0, 10.0), p(15.0, 15.0)]);
    }

    #[test]
    fn an_offset_chain_keeps_the_pitch_and_exact_octilinear_runs() {
        // East 10, down-right 5, south 10.
        let r = vec![p(0.0, 0.0), p(10.0, 0.0), p(15.0, 5.0), p(15.0, 15.0)];
        let o = offset(&r, 2_000_000, "right").unwrap();
        assert_eq!(o.len(), 4);
        assert_eq!(o[0], p(0.0, 2.0), "right of eastward is down the board");
        assert_eq!(o.last().unwrap().y, p(0.0, 15.0).y);
        for w in o.windows(2) {
            assert!(compass(w[0], w[1]).is_some(), "{w:?} is exactly octilinear");
        }
        // Each run is exactly the pitch from its reference run.
        for (ow, rw) in o.windows(2).zip(r.windows(2)) {
            let (u, q) = (rw[1] - rw[0], ow[0] - rw[0]);
            let d = (u.cross(q) as f64).abs() / u.length_f64();
            assert!((d - 2_000_000.0).abs() <= 1.0, "{d}");
        }
        // This chain turns right: on its inner (right) side a 1.4 mm diagonal
        // folds over at a 3 mm pitch; on the outer side it grows.
        let short = vec![p(0.0, 0.0), p(10.0, 0.0), p(11.0, 1.0), p(11.0, 10.0)];
        assert!(offset(&short, 3_000_000, "left").is_ok());
        assert!(offset(&short, 3_000_000, "right").is_err());
    }

    #[test]
    fn a_chain_is_followed_in_the_first_tracks_direction_and_branches_are_refused() {
        let t = |a: (f64, f64), b: (f64, f64)| konnect_route::plan::PlanTrack {
            net: "N".into(),
            layer: "In2.Cu".into(),
            width: 1.0,
            x1: a.0,
            y1: a.1,
            x2: b.0,
            y2: b.1,
        };
        let plan = RoutePlan {
            tracks: vec![
                t((5.0, 0.0), (10.0, 0.0)),
                t((15.0, 5.0), (10.0, 0.0)),
                t((0.0, 0.0), (5.0, 0.0)),
            ],
            vias: vec![],
        };
        let c = chain_of(&plan, "N", "In2.Cu").unwrap();
        assert_eq!(
            c,
            vec![p(0.0, 0.0), p(10.0, 0.0), p(15.0, 5.0)],
            "collinear runs join"
        );
        let mut branched = plan.clone();
        branched.tracks.push(t((10.0, 0.0), (10.0, 5.0)));
        assert!(chain_of(&branched, "N", "In2.Cu")
            .unwrap_err()
            .contains("branches"));
        assert!(chain_of(&plan, "N", "F.Cu").is_err());
    }

    #[test]
    fn the_polyline_checks_name_what_is_wrong() {
        let ok = vec![p(0.0, 0.0), p(5.0, 0.0), p(8.0, 3.0), p(8.0, 9.0)];
        assert!(check_polyline(&ok, "vertices", false, 250_000, 90.0).is_ok());
        let reason = |v: &[Point], any: bool, min_bend: f64| {
            let r = check_polyline(v, "vertices", any, 250_000, min_bend).unwrap_err();
            serde_json::to_string(&r).unwrap()
        };
        let skew = vec![p(0.0, 0.0), p(5.0, 0.0), p(8.0, 3.1)];
        assert!(reason(&skew, false, 90.0).contains("off 0, 45 or 90"));
        assert!(check_polyline(&skew, "vertices", true, 250_000, 90.0).is_ok());
        let short = vec![p(0.0, 0.0), p(5.0, 0.0), p(5.1, 0.1), p(9.0, 0.1)];
        assert!(reason(&short, false, 0.0).contains("shorter than min_segment_mm"));
        let acute = vec![p(0.0, 0.0), p(5.0, 0.0), p(2.0, 3.0)];
        assert!(reason(&acute, false, 90.0).contains("interior angle of 45.00"));
        let crossing = vec![
            p(0.0, 0.0),
            p(10.0, 0.0),
            p(10.0, 5.0),
            p(5.0, 5.0),
            p(5.0, -5.0),
        ];
        assert!(reason(&crossing, false, 90.0).contains("cross"));
        // 1 nm off 45 degrees is within the tolerance; 3 nm is not.
        let one = vec![Point::new(0, 0), Point::new(1_000_000, 1_000_001)];
        assert!(check_polyline(&one, "vertices", false, 250_000, 90.0).is_ok());
        let three = vec![Point::new(0, 0), Point::new(1_000_000, 1_000_003)];
        assert!(check_polyline(&three, "vertices", false, 250_000, 90.0).is_err());
    }
}

#[cfg(test)]
#[path = "route_build_tests.rs"]
mod tool_tests;
