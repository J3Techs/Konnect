//! `render_board_layers_png`: a PNG of a window of the saved board.
//!
//! Copper is drawn from the native router's exact model (pad outlines with
//! custom primitives in board space, tracks, arcs, vias with their drills,
//! zone fills and authored zone outlines), coloured per net with a stable
//! colour from the net name: GND nets slate grey, power nets in warm hues,
//! everything else spread over the remaining hues. A routing_file and a
//! zones file can be laid over the board so a plan is seen before it is
//! applied. Labels use the renderer's built-in stroke font.

use crate::mcp::protocol::{CallToolResult, ToolContent};
use crate::tools::board_inspect::{
    box_arg, layer_names, layers_arg, load_saved_board, zone_layers, zone_region, SavedBoard,
    SAVED_FILE_NOTE,
};
use crate::tools::route_native::{invalid, opt_bool, opt_path, opt_strings, Refusal};
use konnect_geom::{ArcPolicy, ArcSide, BBox, Point, Shape};
use konnect_render::{Anchor, Canvas, Rgba};
use konnect_route::api::{LayerId, LayerMask};
use konnect_route::board::PadKind;
use konnect_route::RoutePlan;
use serde_json::{json, Value};
use std::collections::BTreeMap;
use std::path::PathBuf;

const BACKGROUND: Rgba = Rgba(16, 20, 24, 255);
const NO_NET: Rgba = Rgba(165, 165, 165, 255);
const EDGE: Rgba = Rgba(230, 210, 60, 255);
const KEEPOUT: Rgba = Rgba(235, 70, 70, 255);
const TEXT: Rgba = Rgba(245, 245, 245, 255);
const HALO: Rgba = Rgba(0, 0, 0, 200);
const PLAN_MARK: Rgba = Rgba(255, 255, 255, 230);
const PLAN_VIA_RING: Rgba = Rgba(255, 60, 220, 255);
const DEFAULT_PX_PER_MM: f64 = 20.0;

pub(crate) async fn handle_render_board_layers_png(
    args: &Value,
    _ctx: &crate::tools::ToolContext,
) -> anyhow::Result<CallToolResult> {
    let args = args.clone();
    tokio::task::spawn_blocking(move || render_blocking(&args)).await?
}

// ─── Colours ─────────────────────────────────────────────────────────────────

fn fnv1a(s: &str) -> u32 {
    let mut h: u32 = 0x811c_9dc5;
    for b in s.bytes() {
        h ^= u32::from(b);
        h = h.wrapping_mul(0x0100_0193);
    }
    h
}

fn hsl(h: f32, s: f32, l: f32) -> Rgba {
    let h = h.rem_euclid(360.0) / 60.0;
    let c = (1.0 - (2.0 * l - 1.0).abs()) * s;
    let x = c * (1.0 - (h % 2.0 - 1.0).abs());
    let (r, g, b) = match h as u32 {
        0 => (c, x, 0.0),
        1 => (x, c, 0.0),
        2 => (0.0, c, x),
        3 => (0.0, x, c),
        4 => (x, 0.0, c),
        _ => (c, 0.0, x),
    };
    let m = l - c / 2.0;
    let to = |v: f32| ((v + m) * 255.0).round().clamp(0.0, 255.0) as u8;
    Rgba(to(r), to(g), to(b), 255)
}

/// A supply rail by name: a leading `+`, a voltage such as `5V`, `3V3`,
/// `12V`, or a rail word (VCC, VDD, VBAT, VBUS, VIN, VM, PWR).
pub(crate) fn is_power_net(name: &str) -> bool {
    let up = name.to_ascii_uppercase();
    if up.starts_with('+') {
        return true;
    }
    if [
        "VCC", "VDD", "VBAT", "VBUS", "VIN", "VSYS", "PWR", "POWER", "SUPPLY", "RAIL",
    ]
    .iter()
    .any(|w| up.contains(w))
    {
        return true;
    }
    let b = up.as_bytes();
    b.windows(2).enumerate().any(|(i, w)| {
        w[0].is_ascii_digit()
            && w[1] == b'V'
            && (i == 0 || !b[i - 1].is_ascii_alphabetic() || b[i - 1].is_ascii_digit())
    }) || up.split(['_', '-']).any(|t| t == "VM")
}

pub(crate) fn is_ground_net(name: &str) -> bool {
    name.to_ascii_uppercase().contains("GND")
}

/// The stable colour of a net: slate for grounds, warm hues for supplies,
/// the rest of the wheel for signals.
pub(crate) fn net_color(name: Option<&str>) -> Rgba {
    let Some(name) = name else { return NO_NET };
    let h = fnv1a(name);
    if is_ground_net(name) {
        let l = 0.50 + (h % 7) as f32 * 0.01;
        return hsl(210.0, 0.16, l);
    }
    if is_power_net(name) {
        let hue = (h % 50) as f32 - 10.0;
        return hsl(hue, 0.85, 0.55 + (h >> 8) as f32 % 10.0 * 0.01);
    }
    let hue = 55.0 + (h % 290) as f32;
    hsl(hue, 0.70, 0.58)
}

fn lighter(c: Rgba, f: f32) -> Rgba {
    let up = |v: u8| (f32::from(v) + (255.0 - f32::from(v)) * f).round() as u8;
    Rgba(up(c.0), up(c.1), up(c.2), c.3)
}

// ─── Arguments ───────────────────────────────────────────────────────────────

#[derive(Debug, Default, Clone, Copy)]
struct Labels {
    references: bool,
    pad_numbers: bool,
    nets: bool,
    zones: bool,
}

fn labels_arg(args: &Value) -> Result<Labels, Refusal> {
    let Some(names) = opt_strings(args, "labels")? else {
        return Ok(Labels {
            references: true,
            ..Labels::default()
        });
    };
    let mut l = Labels::default();
    for n in &names {
        match n.as_str() {
            "references" => l.references = true,
            "pad_numbers" => l.pad_numbers = true,
            "nets" => l.nets = true,
            "zones" => l.zones = true,
            "none" => {}
            other => {
                return Err(invalid(
                    "labels",
                    &format!("'{other}' is not references, pad_numbers, nets, zones or none"),
                ))
            }
        }
    }
    Ok(l)
}

fn positive_arg(args: &Value, key: &str, default: f64, max: f64) -> Result<f64, Refusal> {
    match args.get(key) {
        None | Some(Value::Null) => Ok(default),
        Some(v) => match v
            .as_f64()
            .filter(|x| x.is_finite() && *x > 0.0 && *x <= max)
        {
            Some(x) => Ok(x),
            None => Err(invalid(
                key,
                &format!("must be a number above 0 and at most {max}"),
            )),
        },
    }
}

/// `grid_mm`: 0 draws none; absent picks a step at least 40 px apart.
fn grid_arg(args: &Value, px_per_mm: f64) -> Result<f64, Refusal> {
    match args.get("grid_mm") {
        None | Some(Value::Null) => {
            let steps = [0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0];
            Ok(steps
                .iter()
                .copied()
                .find(|s| s * px_per_mm >= 40.0)
                .unwrap_or(100.0))
        }
        Some(v) => match v.as_f64().filter(|x| x.is_finite() && *x >= 0.0) {
            Some(x) if x == 0.0 || x * px_per_mm >= 4.0 => Ok(x),
            Some(_) => Err(invalid(
                "grid_mm",
                "is finer than 4 px at this scale; pick a larger step or 0 for none",
            )),
            None => Err(invalid("grid_mm", "must be a non-negative number of mm")),
        },
    }
}

/// The zones of a `zones` overlay file: a JSON array of `add_zones` entries,
/// or an object with a `zones` array.
fn read_zone_overlay(
    path: &std::path::Path,
) -> Result<Vec<super::zone_batch::ZoneRequest>, Refusal> {
    super::route_native::file_must_exist(path, "overlay_zones_file")?;
    let text = std::fs::read_to_string(path)
        .map_err(|e| invalid("overlay_zones_file", &format!("cannot be read: {e}")))?;
    let v: Value = serde_json::from_str(&text)
        .map_err(|e| invalid("overlay_zones_file", &format!("is not JSON: {e}")))?;
    let entries = match &v {
        Value::Array(a) => a.clone(),
        Value::Object(o) => match o.get("zones").and_then(Value::as_array) {
            Some(a) => a.clone(),
            None => return Err(invalid("overlay_zones_file", "has no zones array")),
        },
        _ => return Err(invalid("overlay_zones_file", "must be an array of zones")),
    };
    entries
        .iter()
        .enumerate()
        .map(|(i, z)| super::zone_batch::parse_zone_request(z, &format!("overlay_zones_file[{i}]")))
        .collect()
}

fn read_plan_overlay(path: &std::path::Path) -> Result<RoutePlan, Refusal> {
    super::route_native::file_must_exist(path, "overlay_routing_file")?;
    std::fs::read_to_string(path)
        .map_err(|e| e.to_string())
        .and_then(|t| serde_json::from_str(&t).map_err(|e| e.to_string()))
        .map_err(|e| {
            invalid(
                "overlay_routing_file",
                &format!("is not a routing_file: {e}"),
            )
        })
}

// ─── Drawing ─────────────────────────────────────────────────────────────────

fn mm2(p: Point) -> (f64, f64) {
    p.to_mm()
}

/// Draw one exact shape grown by `grow_mm` (a border pass when positive).
fn draw_shape(c: &mut Canvas, s: &Shape, color: Rgba, grow_mm: f64) {
    match s {
        Shape::Circle(k) => {
            let (x, y) = mm2(k.center);
            c.fill_circle(x, y, k.radius as f64 / 1e6 + grow_mm, color);
        }
        Shape::Segment(g) => c.stroke_segment(
            mm2(g.a),
            mm2(g.b),
            g.width as f64 / 1e6 + 2.0 * grow_mm,
            color,
        ),
        Shape::ConvexPoly(p) => {
            let ring: Vec<(f64, f64)> = p.points().iter().map(|q| mm2(*q)).collect();
            c.fill_rounded_polygon(&ring, p.radius() as f64 / 1e6 + grow_mm, color);
        }
        Shape::Polygon(p) => {
            let rings: Vec<Vec<(f64, f64)>> = p
                .rings()
                .map(|r| r.iter().map(|q| mm2(*q)).collect())
                .collect();
            if grow_mm > 0.0 {
                let w = (2.0 * grow_mm * c.px_per_mm()) as f32;
                for r in &rings {
                    c.stroke_px(r, true, w, color, None);
                }
            }
            c.fill_polygon(&rings, color);
        }
    }
}

#[derive(Default)]
struct Drawn {
    pads: usize,
    tracks: usize,
    arcs: usize,
    vias: usize,
    zones: usize,
    fill_pieces: usize,
    rule_areas: usize,
    plan_tracks: usize,
    plan_vias: usize,
    plan_zones: usize,
    labels: usize,
}

struct Scene<'a> {
    saved: &'a SavedBoard,
    window: BBox,
    layers: Vec<LayerId>,
    fill_zones: bool,
    labels: Labels,
}

impl Scene<'_> {
    fn visible(&self, b: &BBox) -> bool {
        b.intersects(&self.window)
    }

    /// Opacity of a layer: the first listed is drawn on top at full
    /// strength, the others under it, faded.
    fn layer_fade(&self, l: LayerId) -> f32 {
        if self.layers.first() == Some(&l) {
            1.0
        } else {
            0.4
        }
    }

    fn draw_board(&self, c: &mut Canvas, d: &mut Drawn) {
        let b = &self.saved.board;
        let policy = ArcPolicy::default();
        let net_name = |n: Option<konnect_route::api::NetId>| n.map(|n| b.net(n).name.as_str());
        let border_mm = 1.0 / c.px_per_mm();
        // Bottom layer of the list first, so the first listed ends on top.
        for &layer in self.layers.iter().rev() {
            let fade = self.layer_fade(layer);
            let mask = LayerMask::single(layer);
            // Zone fills, then their authored outlines.
            for z in &self.saved.zones {
                if z.geometry.keepout.is_some() || !zone_layers(b, z).intersects(mask) {
                    continue;
                }
                let color = net_color(z.geometry.net.as_deref());
                let lname = b.layers.name(layer);
                let mut any = false;
                if self.fill_zones {
                    for f in z.geometry.fills.iter().filter(|f| f.layer == lname) {
                        let ring: Vec<(f64, f64)> = f.points.clone();
                        let bb = ring_bbox(&ring);
                        if bb.is_some_and(|bb| self.visible(&bb)) {
                            c.fill_polygon(&[ring], color.faded(0.38 * fade));
                            d.fill_pieces += 1;
                            any = true;
                        }
                    }
                }
                if let Some(poly) = zone_region(z, ArcSide::Inscribed) {
                    if self.visible(&poly.bbox()) {
                        for r in poly.rings() {
                            let ring: Vec<(f64, f64)> = r.iter().map(|q| mm2(*q)).collect();
                            c.stroke_px(
                                &ring,
                                true,
                                1.5,
                                color.faded(0.9 * fade),
                                Some((6.0, 4.0)),
                            );
                        }
                        any = true;
                    }
                }
                d.zones += usize::from(any);
            }
            for t in b.tracks.iter().filter(|t| t.layer == layer) {
                if self.visible(&t.seg.bbox()) {
                    let color = net_color(net_name(t.net)).faded(fade);
                    c.stroke_segment(mm2(t.seg.a), mm2(t.seg.b), t.seg.width as f64 / 1e6, color);
                    d.tracks += 1;
                }
            }
            for a in b.arcs.iter().filter(|a| a.layer == layer) {
                let segs = a.segments(&policy);
                if segs.iter().any(|s| self.visible(&s.bbox())) {
                    let color = net_color(net_name(a.net)).faded(fade);
                    for s in segs {
                        c.stroke_segment(mm2(s.a), mm2(s.b), s.width as f64 / 1e6, color);
                    }
                    d.arcs += 1;
                }
            }
            for p in b.pads.iter().filter(|p| p.layers.contains(layer)) {
                let Some(bb) = p.bbox() else { continue };
                if !self.visible(&bb) {
                    continue;
                }
                let color = lighter(net_color(net_name(p.net)), 0.25).faded(fade);
                for s in &p.shapes {
                    draw_shape(c, s, Rgba(0, 0, 0, 170).faded(fade), border_mm);
                }
                for s in &p.shapes {
                    draw_shape(c, s, color, 0.0);
                }
                if layer == self.layers[0] {
                    d.pads += 1;
                }
            }
            for v in b.vias.iter().filter(|v| v.layers.contains(layer)) {
                if self.visible(&v.shape().bbox()) {
                    let (x, y) = mm2(v.center);
                    let color = net_color(net_name(v.net)).faded(fade);
                    c.fill_circle(x, y, v.diameter as f64 / 2e6, color);
                    if layer == self.layers[0] {
                        d.vias += 1;
                    }
                }
            }
        }
        // Holes go through every layer: drawn once, over all copper.
        for p in &b.pads {
            if let Some(dr) = &p.drill {
                if self.visible(&dr.shape.bbox()) {
                    draw_shape(c, &dr.shape, BACKGROUND, 0.0);
                    if p.kind == PadKind::NpThroughHole {
                        draw_shape_outline(c, &dr.shape, Rgba(220, 220, 220, 255));
                    }
                }
            }
        }
        for v in &b.vias {
            if self.visible(&v.shape().bbox()) && self.layers.iter().any(|l| v.layers.contains(*l))
            {
                let (x, y) = mm2(v.center);
                c.fill_circle(x, y, v.drill as f64 / 2e6, BACKGROUND);
            }
        }
        // Rule areas on the drawn layers.
        let drawn_mask = self.layers.iter().fold(LayerMask::EMPTY, |m, l| m.with(*l));
        for z in self
            .saved
            .zones
            .iter()
            .filter(|z| z.geometry.keepout.is_some())
        {
            if !zone_layers(b, z).intersects(drawn_mask) {
                continue;
            }
            if let Some(poly) = zone_region(z, ArcSide::Circumscribed) {
                if self.visible(&poly.bbox()) {
                    for r in poly.rings() {
                        let ring: Vec<(f64, f64)> = r.iter().map(|q| mm2(*q)).collect();
                        c.stroke_px(&ring, true, 2.0, KEEPOUT, Some((3.0, 3.0)));
                    }
                    d.rule_areas += 1;
                }
            }
        }
        for e in &b.outline.edges {
            if self.visible(&e.bbox()) {
                c.stroke_px(&[mm2(e.a), mm2(e.b)], false, 2.0, EDGE, None);
            }
        }
    }

    fn draw_plan(&self, c: &mut Canvas, plan: &RoutePlan, d: &mut Drawn) {
        let b = &self.saved.board;
        for t in &plan.tracks {
            let Some(layer) = b.layers.by_name(&t.layer) else {
                continue;
            };
            if !self.layers.contains(&layer) {
                continue;
            }
            let (a, z) = ((t.x1, t.y1), (t.x2, t.y2));
            let bb = ring_bbox(&[a, z]).map(|bb| bb.inflate((t.width * 5e5) as i32));
            if !bb.is_some_and(|bb| self.visible(&bb)) {
                continue;
            }
            let color = net_color(Some(&t.net));
            c.stroke_segment(a, z, t.width, color.faded(0.85));
            c.stroke_px(&[a, z], false, 1.2, PLAN_MARK, Some((5.0, 4.0)));
            d.plan_tracks += 1;
        }
        for v in &plan.vias {
            let bb = ring_bbox(&[(v.x, v.y)]).map(|bb| bb.inflate((v.diameter * 5e5) as i32));
            if !bb.is_some_and(|bb| self.visible(&bb)) {
                continue;
            }
            let color = net_color(Some(&v.net));
            c.fill_circle(v.x, v.y, v.diameter / 2.0, color);
            c.fill_circle(v.x, v.y, v.drill / 2.0, BACKGROUND);
            c.stroke_circle(
                v.x,
                v.y,
                v.diameter / 2.0 + 1.5 / c.px_per_mm(),
                2.0,
                PLAN_VIA_RING,
            );
            d.plan_vias += 1;
        }
    }

    fn draw_zone_overlay(
        &self,
        c: &mut Canvas,
        zones: &[super::zone_batch::ZoneRequest],
        d: &mut Drawn,
    ) {
        let b = &self.saved.board;
        for z in zones {
            let Some(layer) = b.layers.by_name(&z.layer) else {
                continue;
            };
            if !self.layers.contains(&layer) {
                continue;
            }
            let Some(bb) = ring_bbox(&z.points) else {
                continue;
            };
            if !self.visible(&bb) {
                continue;
            }
            let color = net_color(Some(&z.net_name));
            let mut rings = vec![z.points.clone()];
            rings.extend(z.holes.iter().cloned());
            c.fill_polygon(&rings, color.faded(0.3));
            for r in &rings {
                c.stroke_px(r, true, 2.5, color, None);
                c.stroke_px(r, true, 1.0, PLAN_MARK, Some((5.0, 4.0)));
            }
            if self.labels.zones {
                let (x, y) = bbox_center(&bb);
                let label = if z.name.is_empty() {
                    &z.net_name
                } else {
                    &z.name
                };
                c.text_mm((x, y), Anchor::Center, 12.0, label, TEXT, Some(HALO));
                d.labels += 1;
            }
            d.plan_zones += 1;
        }
    }

    fn draw_labels(&self, c: &mut Canvas, d: &mut Drawn) {
        let b = &self.saved.board;
        let drawn_mask = self.layers.iter().fold(LayerMask::EMPTY, |m, l| m.with(*l));
        let scale = c.px_per_mm() as f32;
        if self.labels.zones {
            for z in self
                .saved
                .zones
                .iter()
                .filter(|z| z.geometry.keepout.is_none())
            {
                if !zone_layers(b, z).intersects(drawn_mask) {
                    continue;
                }
                let Some(poly) = zone_region(z, ArcSide::Inscribed) else {
                    continue;
                };
                let bb = poly.bbox();
                if !self.visible(&bb) {
                    continue;
                }
                let label = z
                    .geometry
                    .name
                    .clone()
                    .or_else(|| z.geometry.net.clone())
                    .unwrap_or_default();
                let clipped = clip(&bb, &self.window);
                let (x, y) = bbox_center(&clipped);
                c.text_mm(
                    (x, y),
                    Anchor::Center,
                    11.0,
                    &label,
                    net_color(z.geometry.net.as_deref()),
                    Some(HALO),
                );
                d.labels += 1;
            }
        }
        if self.labels.pad_numbers || self.labels.nets {
            for p in b.pads.iter().filter(|p| p.layers.intersects(drawn_mask)) {
                let Some(bb) = p.bbox() else { continue };
                if !self.window.contains(bb.center()) {
                    continue;
                }
                let side = (bb.width().min(bb.height()) as f32 / 1e6) * scale;
                let (x, y) = mm2(bb.center());
                let mut lines: Vec<(String, f32)> = Vec::new();
                if self.labels.pad_numbers && side >= 7.0 {
                    lines.push((p.number.clone(), (side * 0.45).clamp(7.0, 14.0)));
                }
                if self.labels.nets && side >= 10.0 {
                    if let Some(n) = p.net {
                        lines.push((b.net(n).name.clone(), 8.0));
                    }
                }
                let total: f32 = lines.iter().map(|(_, h)| h + 2.0).sum();
                let (px, py) = c.to_px(x, y);
                let mut top = py - total / 2.0;
                for (text, h) in lines {
                    c.text_px(
                        (px, top + h / 2.0),
                        Anchor::Center,
                        h,
                        &text,
                        TEXT,
                        Some(HALO),
                    );
                    top += h + 2.0;
                    d.labels += 1;
                }
            }
        }
        if self.labels.references {
            for f in &b.footprints {
                let pads = &b.pads[f.pad_range.0 as usize..f.pad_range.1 as usize];
                if !pads.iter().any(|p| p.layers.intersects(drawn_mask)) {
                    continue;
                }
                let Some(reference) = &f.reference else {
                    continue;
                };
                let at = f.courtyard_bbox().map(|bb| bb.center()).unwrap_or(f.at);
                if !self.window.contains(at) {
                    continue;
                }
                let (x, y) = mm2(at);
                c.text_mm(
                    (x, y),
                    Anchor::Center,
                    13.0,
                    reference,
                    Rgba(255, 245, 160, 255),
                    Some(HALO),
                );
                d.labels += 1;
            }
        }
    }
}

fn draw_shape_outline(c: &mut Canvas, s: &Shape, color: Rgba) {
    if let Shape::Circle(k) = s {
        let (x, y) = mm2(k.center);
        c.stroke_circle(x, y, k.radius as f64 / 1e6, 1.0, color);
    } else if let Shape::Segment(g) = s {
        c.stroke_px(&[mm2(g.a), mm2(g.b)], false, 1.0, color, None);
    }
}

fn ring_bbox(ring: &[(f64, f64)]) -> Option<BBox> {
    BBox::from_points(ring.iter().filter_map(|&(x, y)| Point::from_mm(x, y)))
}

fn bbox_center(b: &BBox) -> (f64, f64) {
    b.center().to_mm()
}

fn clip(a: &BBox, w: &BBox) -> BBox {
    let min = Point::new(a.min.x.max(w.min.x), a.min.y.max(w.min.y));
    let max = Point::new(a.max.x.min(w.max.x), a.max.y.min(w.max.y));
    if min.x <= max.x && min.y <= max.y {
        BBox::new(min, max)
    } else {
        *a
    }
}

/// A grid of `step` mm with coordinate labels along the top and left edges.
fn draw_grid(c: &mut Canvas, window: &BBox, step: f64) {
    if step <= 0.0 {
        return;
    }
    let (x0, y0) = window.min.to_mm();
    let (x1, y1) = window.max.to_mm();
    let line = Rgba(255, 255, 255, 34);
    let px = step * c.px_per_mm();
    let every = ((60.0 / px).ceil() as i64).max(1);
    let decimals = if step >= 1.0 {
        0
    } else if step >= 0.1 {
        1
    } else {
        2
    };
    let first_x = (x0 / step).ceil() as i64;
    let last_x = (x1 / step).floor() as i64;
    for k in first_x..=last_x {
        let x = k as f64 * step;
        let (sx, _) = c.to_px(x, y0);
        c.line_px((sx, 0.0), (sx, c.height_px() as f32), 1.0, line);
        if k % every == 0 {
            c.text_px(
                (sx + 2.0, 2.0),
                Anchor::TopLeft,
                9.0,
                &format!("{x:.decimals$}"),
                Rgba(200, 200, 200, 255),
                Some(HALO),
            );
        }
    }
    let first_y = (y0 / step).ceil() as i64;
    let last_y = (y1 / step).floor() as i64;
    for k in first_y..=last_y {
        let y = k as f64 * step;
        let (_, sy) = c.to_px(x0, y);
        c.line_px((0.0, sy), (c.width_px() as f32, sy), 1.0, line);
        if k % every == 0 {
            c.text_px(
                (2.0, sy + 2.0),
                Anchor::TopLeft,
                9.0,
                &format!("{y:.decimals$}"),
                Rgba(200, 200, 200, 255),
                Some(HALO),
            );
        }
    }
}

fn default_output(saved: &SavedBoard, layers: &[String], w: &BBox) -> PathBuf {
    let stem = saved
        .path
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_else(|| "board".to_string());
    let (x0, y0) = w.min.to_mm();
    let (x1, y1) = w.max.to_mm();
    let layers = layers.join("+").replace('.', "");
    std::env::temp_dir().join("konnect-renders").join(format!(
        "{stem}-{layers}-{x0:.2}_{y0:.2}_{x1:.2}_{y1:.2}.png"
    ))
}

fn render_blocking(args: &Value) -> anyhow::Result<CallToolResult> {
    let saved = match load_saved_board(args)? {
        Ok(s) => s,
        Err(r) => return Ok(r),
    };
    let b = &saved.board;
    let parsed = (|| -> Result<_, Refusal> {
        let window = match args.get("area") {
            None | Some(Value::Null) => match b.outline.bbox {
                Some(bb) => bb.inflate(500_000),
                None => {
                    return Err(invalid(
                        "area",
                        "is required: the board has no Edge.Cuts outline to frame",
                    ))
                }
            },
            Some(v) => box_arg(v, "area")?,
        };
        let mask = layers_arg(b, args, "layers")?.unwrap_or_else(|| {
            b.layers
                .front()
                .map(LayerMask::single)
                .unwrap_or(LayerMask::EMPTY)
        });
        // Keep the caller's order: the first listed layer is drawn on top.
        let mut layers: Vec<LayerId> = Vec::new();
        for n in opt_strings(args, "layers")?.unwrap_or_else(|| vec!["F.Cu".to_string()]) {
            for l in b.layers.expand(n.trim()).iter() {
                if !layers.contains(&l) {
                    layers.push(l);
                }
            }
        }
        if layers.is_empty() {
            layers.extend(mask.iter());
        }
        if layers.is_empty() {
            return Err(invalid(
                "layers",
                "is required: the board has no F.Cu to draw by default",
            ));
        }
        let px_per_mm = positive_arg(args, "px_per_mm", DEFAULT_PX_PER_MM, 400.0)?;
        let grid = grid_arg(args, px_per_mm)?;
        let fill_zones = opt_bool(args, "fill_zones", true)?;
        let labels = labels_arg(args)?;
        let plan = match opt_path(args, "overlay_routing_file")? {
            Some(p) => Some(read_plan_overlay(&p)?),
            None => None,
        };
        let zones = match opt_path(args, "overlay_zones_file")? {
            Some(p) => Some(read_zone_overlay(&p)?),
            None => None,
        };
        let output = opt_path(args, "output")?;
        let inline = opt_bool(args, "inline", false)?;
        Ok((
            window, layers, px_per_mm, grid, fill_zones, labels, plan, zones, output, inline,
        ))
    })();
    let (window, layers, px_per_mm, grid, fill_zones, labels, plan, zones, output, inline) =
        match parsed {
            Ok(v) => v,
            Err(r) => return Ok(r),
        };
    let (x0, y0) = window.min.to_mm();
    let (x1, y1) = window.max.to_mm();
    let mut canvas = match Canvas::new(x0, y0, x1, y1, px_per_mm, BACKGROUND) {
        Ok(c) => c,
        Err(e) => return Ok(invalid("px_per_mm", &format!("{e:#}"))),
    };
    let t = std::time::Instant::now();
    let scene = Scene {
        saved: &saved,
        window,
        layers: layers.clone(),
        fill_zones,
        labels,
    };
    let mut drawn = Drawn::default();
    draw_grid(&mut canvas, &window, grid);
    scene.draw_board(&mut canvas, &mut drawn);
    if let Some(z) = &zones {
        scene.draw_zone_overlay(&mut canvas, z, &mut drawn);
    }
    if let Some(p) = &plan {
        scene.draw_plan(&mut canvas, p, &mut drawn);
    }
    scene.draw_labels(&mut canvas, &mut drawn);
    let layer_list = layer_names(b, layers.iter().fold(LayerMask::EMPTY, |m, l| m.with(*l)));
    let ordered: Vec<String> = layers
        .iter()
        .map(|l| b.layers.name(*l).to_string())
        .collect();
    let caption = format!(
        "{}  {}  {:.3},{:.3} - {:.3},{:.3} mm  {} px/mm",
        saved
            .path
            .file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_default(),
        ordered.join(" over "),
        x0,
        y0,
        x1,
        y1,
        px_per_mm
    );
    let h = canvas.height_px() as f32;
    canvas.text_px(
        (4.0, h - 14.0),
        Anchor::TopLeft,
        10.0,
        &caption,
        TEXT,
        Some(HALO),
    );
    let png = canvas.encode_png()?;
    let path = output.unwrap_or_else(|| default_output(&saved, &ordered, &window));
    if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(&path, &png)?;
    let mut legend: BTreeMap<&str, Value> = BTreeMap::new();
    legend.insert("ground_nets", json!("slate grey"));
    legend.insert("power_nets", json!("warm hues (red to orange)"));
    legend.insert("other_nets", json!("a stable hue per net name"));
    legend.insert(
        "zone_outline",
        json!("dashed, in the zone's net colour; fills translucent"),
    );
    legend.insert("rule_area", json!("red dashed outline"));
    legend.insert("board_edge", json!("yellow"));
    legend.insert(
        "overlay",
        json!("planned copper carries a white dashed centre line; planned vias a magenta ring"),
    );
    let body = json!({
        "source": "saved_file",
        "board": saved.path,
        "note": SAVED_FILE_NOTE,
        "rendered": path.display().to_string(),
        "width_px": canvas.width_px(),
        "height_px": canvas.height_px(),
        "png_bytes": png.len(),
        "window": { "x1": x0, "y1": y0, "x2": x1, "y2": y1 },
        "px_per_mm": px_per_mm,
        "grid_mm": grid,
        "layers": ordered,
        "layers_drawn": layer_list,
        "drawn": {
            "pads": drawn.pads, "tracks": drawn.tracks, "arcs": drawn.arcs, "vias": drawn.vias,
            "zones": drawn.zones, "fill_pieces": drawn.fill_pieces, "rule_areas": drawn.rule_areas,
            "overlay_tracks": drawn.plan_tracks, "overlay_vias": drawn.plan_vias,
            "overlay_zones": drawn.plan_zones, "labels": drawn.labels,
        },
        "legend": legend,
        "elapsed_ms": t.elapsed().as_millis() as u64,
        "inline": inline,
    });
    let mut result = CallToolResult::json(&body);
    if inline {
        use base64::Engine as _;
        result.content.push(ToolContent::Image {
            data: base64::engine::general_purpose::STANDARD.encode(&png),
            mime_type: "image/png".to_string(),
        });
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn net_colours_are_stable_and_ground_and_power_stand_apart() {
        assert_eq!(net_color(Some("SIG_A")), net_color(Some("SIG_A")));
        let gnd = net_color(Some("GND"));
        assert!(gnd.0.abs_diff(gnd.2) < 40, "grounds are near-grey: {gnd:?}");
        for p in ["AUX_5V", "+3V3", "CORE_3V3", "BUS_12V", "VBAT", "MOTOR_VM"] {
            assert!(is_power_net(p), "{p}");
            let c = net_color(Some(p));
            assert!(c.0 > c.2, "{p} is warm: {c:?}");
        }
        for s in ["CAN_H", "SPI_MOSI", "OUT_1_HIGH", "SD_D3"] {
            assert!(!is_power_net(s), "{s}");
        }
        assert_eq!(net_color(None), NO_NET);
    }
}
