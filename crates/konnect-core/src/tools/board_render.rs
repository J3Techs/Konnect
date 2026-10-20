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

/// `overlay_routing_file`, or `overlay_routing_files` merged in order.
fn read_plan_overlay(args: &Value) -> Result<Option<RoutePlan>, Refusal> {
    let files = super::route_native::read_plan_files(
        args,
        "overlay_routing_file",
        "overlay_routing_files",
    )?;
    Ok((!files.is_empty()).then(|| super::route_native::merged_plan(&files)))
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
    markers: usize,
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

// ─── Markers ─────────────────────────────────────────────────────────────────

/// Radius of a marker ring, px: the same at every scale.
pub(crate) const MARKER_RING_PX: f64 = 12.0;
const MARKER_STROKE_PX: f32 = 3.0;
/// Most markers one render takes.
const MARKERS_MAX: usize = 100_000;

/// A point to ring on a render, with an optional short label; its colour
/// comes from its kind.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Marker {
    pub x: f64,
    pub y: f64,
    pub label: Option<String>,
    pub kind: Option<String>,
}

/// The ring colour of a marker kind: the lint's kinds have fixed bright
/// colours, any other kind a stable hue, no kind white.
pub(crate) fn marker_color(kind: Option<&str>) -> Rgba {
    match kind {
        None => Rgba(255, 255, 255, 255),
        Some("via_dogleg") => Rgba(0, 255, 255, 255),
        Some("pad_dogleg") => Rgba(90, 150, 255, 255),
        Some("acute_bend") => Rgba(255, 255, 0, 255),
        Some("short_jog") => Rgba(255, 140, 0, 255),
        Some("hairpin") => Rgba(255, 60, 170, 255),
        Some("unsplit_t_junction") => Rgba(140, 255, 60, 255),
        Some("collinear_vertex") => Rgba(205, 205, 205, 255),
        Some(other) => hsl((fnv1a(other) % 360) as f32, 1.0, 0.6),
    }
}

fn parse_marker(v: &Value, field: &str) -> Result<Marker, Refusal> {
    let Some(o) = v.as_object() else {
        return Err(invalid(field, "must be an object {x, y, label?, kind?}"));
    };
    if let Some(k) = o
        .keys()
        .find(|k| !["x", "y", "label", "kind"].contains(&k.as_str()))
    {
        return Err(invalid(
            &format!("{field}.{k}"),
            "is not a marker key (x, y, label, kind)",
        ));
    }
    let num = |k: &str| o.get(k).and_then(Value::as_f64).filter(|x| x.is_finite());
    let text = |k: &str| -> Result<Option<String>, Refusal> {
        match o.get(k) {
            None | Some(Value::Null) => Ok(None),
            Some(Value::String(s)) => Ok(Some(s.clone())),
            Some(_) => Err(invalid(&format!("{field}.{k}"), "must be a string")),
        }
    };
    match (num("x"), num("y")) {
        (Some(x), Some(y)) => Ok(Marker {
            x,
            y,
            label: text("label")?,
            kind: text("kind")?,
        }),
        _ => Err(invalid(field, "needs finite x and y (mm)")),
    }
}

/// `markers` inline and/or `markers_file` (a JSON array of markers, or an
/// object with a `markers` array), in that order.
fn markers_arg(args: &Value) -> Result<Vec<Marker>, Refusal> {
    let mut out = Vec::new();
    match args.get("markers") {
        None | Some(Value::Null) => {}
        Some(Value::Array(items)) => {
            for (i, m) in items.iter().enumerate() {
                out.push(parse_marker(m, &format!("markers[{i}]"))?);
            }
        }
        Some(_) => return Err(invalid("markers", "must be an array of markers")),
    }
    if let Some(path) = opt_path(args, "markers_file")? {
        super::route_native::file_must_exist(&path, "markers_file")?;
        let v: Value = std::fs::read_to_string(&path)
            .map_err(|e| e.to_string())
            .and_then(|t| serde_json::from_str(&t).map_err(|e| e.to_string()))
            .map_err(|e| invalid("markers_file", &format!("cannot be read as JSON: {e}")))?;
        let items = match &v {
            Value::Array(a) => a,
            Value::Object(o) => match o.get("markers").and_then(Value::as_array) {
                Some(a) => a,
                None => return Err(invalid("markers_file", "has no markers array")),
            },
            _ => return Err(invalid("markers_file", "must hold an array of markers")),
        };
        for (i, m) in items.iter().enumerate() {
            out.push(parse_marker(m, &format!("markers_file[{i}]"))?);
        }
    }
    if out.len() > MARKERS_MAX {
        return Err(invalid(
            "markers",
            &format!(
                "{} markers exceed the {MARKERS_MAX} one render takes",
                out.len()
            ),
        ));
    }
    Ok(out)
}

fn in_window(window: &BBox, x: f64, y: f64) -> bool {
    Point::from_mm(x, y).is_some_and(|p| window.contains(p))
}

/// Ring each marker in the window, label it, and count them.
fn draw_markers(c: &mut Canvas, window: &BBox, markers: &[Marker]) -> usize {
    let r_mm = MARKER_RING_PX / c.px_per_mm();
    let mut n = 0;
    for m in markers.iter().filter(|m| in_window(window, m.x, m.y)) {
        let color = marker_color(m.kind.as_deref());
        c.stroke_circle(m.x, m.y, r_mm, MARKER_STROKE_PX + 3.0, HALO);
        c.stroke_circle(m.x, m.y, r_mm, MARKER_STROKE_PX, color);
        if let Some(label) = m.label.as_deref().filter(|l| !l.is_empty()) {
            let (px, py) = c.to_px(m.x, m.y);
            let r = MARKER_RING_PX as f32;
            c.text_px(
                (px + r + 3.0, py - r - 2.0),
                Anchor::TopLeft,
                11.0,
                label,
                color,
                Some(HALO),
            );
        }
        n += 1;
    }
    n
}

// ─── One window, drawn ───────────────────────────────────────────────────────

/// What a render draws besides its window, parsed once: shared by
/// `render_board_layers_png` and every tile of `render_board_tiles`.
struct RenderSpec {
    layers: Vec<LayerId>,
    px_per_mm: f64,
    grid: f64,
    fill_zones: bool,
    labels: Labels,
    plan: Option<RoutePlan>,
    zones: Option<Vec<super::zone_batch::ZoneRequest>>,
    markers: Vec<Marker>,
}

impl RenderSpec {
    fn layer_names(&self, b: &konnect_route::RouteBoard) -> Vec<String> {
        self.layers
            .iter()
            .map(|l| b.layers.name(*l).to_string())
            .collect()
    }
}

/// `area`, or the board outline grown by 0.5 mm.
fn window_arg(b: &konnect_route::RouteBoard, args: &Value) -> Result<BBox, Refusal> {
    match args.get("area") {
        None | Some(Value::Null) => match b.outline.bbox {
            Some(bb) => Ok(bb.inflate(500_000)),
            None => Err(invalid(
                "area",
                "is required: the board has no Edge.Cuts outline to frame",
            )),
        },
        Some(v) => box_arg(v, "area"),
    }
}

fn render_spec(b: &konnect_route::RouteBoard, args: &Value) -> Result<RenderSpec, Refusal> {
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
    let plan = read_plan_overlay(args)?;
    let zones = match opt_path(args, "overlay_zones_file")? {
        Some(p) => Some(read_zone_overlay(&p)?),
        None => None,
    };
    let markers = markers_arg(args)?;
    Ok(RenderSpec {
        layers,
        px_per_mm,
        grid,
        fill_zones,
        labels,
        plan,
        zones,
        markers,
    })
}

/// Draw `window` of the board as `spec` says, caption included.
fn draw_window(
    saved: &SavedBoard,
    spec: &RenderSpec,
    window: BBox,
) -> Result<(Canvas, Drawn), Refusal> {
    let (x0, y0) = window.min.to_mm();
    let (x1, y1) = window.max.to_mm();
    let mut canvas = Canvas::new(x0, y0, x1, y1, spec.px_per_mm, BACKGROUND)
        .map_err(|e| invalid("px_per_mm", &format!("{e:#}")))?;
    let scene = Scene {
        saved,
        window,
        layers: spec.layers.clone(),
        fill_zones: spec.fill_zones,
        labels: spec.labels,
    };
    let mut drawn = Drawn::default();
    draw_grid(&mut canvas, &window, spec.grid);
    scene.draw_board(&mut canvas, &mut drawn);
    if let Some(z) = &spec.zones {
        scene.draw_zone_overlay(&mut canvas, z, &mut drawn);
    }
    if let Some(p) = &spec.plan {
        scene.draw_plan(&mut canvas, p, &mut drawn);
    }
    scene.draw_labels(&mut canvas, &mut drawn);
    drawn.markers = draw_markers(&mut canvas, &window, &spec.markers);
    let caption = format!(
        "{}  {}  {:.3},{:.3} - {:.3},{:.3} mm  {} px/mm",
        saved
            .path
            .file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_default(),
        spec.layer_names(&saved.board).join(" over "),
        x0,
        y0,
        x1,
        y1,
        spec.px_per_mm
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
    Ok((canvas, drawn))
}

fn legend() -> BTreeMap<&'static str, Value> {
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
    legend.insert(
        "markers",
        json!("a ring of fixed pixel size with its label: via_dogleg cyan, pad_dogleg blue, acute_bend yellow, short_jog orange, hairpin pink, unsplit_t_junction lime, collinear_vertex grey, other kinds a stable hue, no kind white"),
    );
    legend
}

fn drawn_json(d: &Drawn) -> Value {
    json!({
        "pads": d.pads, "tracks": d.tracks, "arcs": d.arcs, "vias": d.vias,
        "zones": d.zones, "fill_pieces": d.fill_pieces, "rule_areas": d.rule_areas,
        "overlay_tracks": d.plan_tracks, "overlay_vias": d.plan_vias,
        "overlay_zones": d.plan_zones, "labels": d.labels, "markers": d.markers,
    })
}

fn render_blocking(args: &Value) -> anyhow::Result<CallToolResult> {
    let saved = match load_saved_board(args)? {
        Ok(s) => s,
        Err(r) => return Ok(r),
    };
    let b = &saved.board;
    let parsed = (|| -> Result<_, Refusal> {
        let window = window_arg(b, args)?;
        let spec = render_spec(b, args)?;
        let output = opt_path(args, "output")?;
        let inline = opt_bool(args, "inline", false)?;
        Ok((window, spec, output, inline))
    })();
    let (window, spec, output, inline) = match parsed {
        Ok(v) => v,
        Err(r) => return Ok(r),
    };
    let t = std::time::Instant::now();
    let (canvas, drawn) = match draw_window(&saved, &spec, window) {
        Ok(v) => v,
        Err(r) => return Ok(r),
    };
    let (x0, y0) = window.min.to_mm();
    let (x1, y1) = window.max.to_mm();
    let layer_list = layer_names(
        b,
        spec.layers.iter().fold(LayerMask::EMPTY, |m, l| m.with(*l)),
    );
    let ordered = spec.layer_names(b);
    let png = canvas.encode_png()?;
    let path = output.unwrap_or_else(|| default_output(&saved, &ordered, &window));
    if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(&path, &png)?;
    let body = json!({
        "source": "saved_file",
        "board": saved.path,
        "note": SAVED_FILE_NOTE,
        "rendered": path.display().to_string(),
        "width_px": canvas.width_px(),
        "height_px": canvas.height_px(),
        "png_bytes": png.len(),
        "window": { "x1": x0, "y1": y0, "x2": x1, "y2": y1 },
        "px_per_mm": spec.px_per_mm,
        "grid_mm": spec.grid,
        "layers": ordered,
        "layers_drawn": layer_list,
        "drawn": drawn_json(&drawn),
        "markers_given": spec.markers.len(),
        "legend": legend(),
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

// ─── render_board_tiles ──────────────────────────────────────────────────────

const TILE_DEFAULT_MM: f64 = 40.0;
const TILE_OVERLAP_DEFAULT_MM: f64 = 2.0;
const TILES_DEFAULT: u64 = 100;
const TILES_MAX: u64 = 2_000;

/// One tile of a grid: its place and its window, mm.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Tile {
    pub row: usize,
    pub col: usize,
    pub x1: f64,
    pub y1: f64,
    pub x2: f64,
    pub y2: f64,
}

/// The starts and ends of tiles `size` long, overlapping by `overlap`,
/// covering `lo..hi`. The last tile is clipped to `hi`.
fn spans(lo: f64, hi: f64, size: f64, overlap: f64) -> Vec<(f64, f64)> {
    let step = size - overlap;
    let width = hi - lo;
    let n = if width <= size {
        1
    } else {
        (((width - size) / step) - 1e-9).ceil() as usize + 1
    };
    (0..n)
        .map(|i| {
            let a = lo + i as f64 * step;
            (a, (a + size).min(hi))
        })
        .collect()
}

/// Square tiles of `size` mm overlapping by `overlap` mm over the window,
/// row by row from the top left.
pub(crate) fn tile_grid(window: (f64, f64, f64, f64), size: f64, overlap: f64) -> Vec<Tile> {
    let (x0, y0, x1, y1) = window;
    let cols = spans(x0, x1, size, overlap);
    let rows = spans(y0, y1, size, overlap);
    let mut out = Vec::with_capacity(cols.len() * rows.len());
    for (row, &(ya, yb)) in rows.iter().enumerate() {
        for (col, &(xa, xb)) in cols.iter().enumerate() {
            out.push(Tile {
                row,
                col,
                x1: xa,
                y1: ya,
                x2: xb,
                y2: yb,
            });
        }
    }
    out
}

pub(crate) fn tiles_schema() -> Value {
    let mut s = super::route_native::render_schema();
    let p = s["properties"]
        .as_object_mut()
        .expect("render schema has properties");
    p.remove("output");
    p.remove("inline");
    p.insert(
        "area".into(),
        super::route_native::box_schema(
            "The window to split into tiles, mm (default: the board outline)",
        ),
    );
    p.insert("tile_mm".into(), json!({ "type": "number", "minimum": 1, "maximum": 1000, "default": TILE_DEFAULT_MM, "description": "Side of a square tile, mm" }));
    p.insert("overlap_mm".into(), json!({ "type": "number", "minimum": 0, "default": TILE_OVERLAP_DEFAULT_MM, "description": "How far neighbouring tiles overlap, mm (less than tile_mm)" }));
    p.insert("only_tiles_with_markers".into(), json!({ "type": "boolean", "default": false, "description": "Write only the tiles that hold at least one marker" }));
    p.insert("max_tiles".into(), json!({ "type": "integer", "minimum": 1, "maximum": TILES_MAX, "default": TILES_DEFAULT, "description": "Most tiles written, row by row; the reply says when more were left out" }));
    p.insert("output_dir".into(), json!({ "type": "string", "description": "Directory for the tiles and index.json; it must be new or empty (default: a new directory under konnect-renders in the system temp directory)" }));
    s
}

pub(crate) async fn handle_render_board_tiles(
    args: &Value,
    _ctx: &crate::tools::ToolContext,
) -> anyhow::Result<CallToolResult> {
    let args = args.clone();
    tokio::task::spawn_blocking(move || render_tiles_blocking(&args)).await?
}

fn render_tiles_blocking(args: &Value) -> anyhow::Result<CallToolResult> {
    let saved = match load_saved_board(args)? {
        Ok(s) => s,
        Err(r) => return Ok(r),
    };
    let b = &saved.board;
    let parsed = (|| -> Result<_, Refusal> {
        let window = window_arg(b, args)?;
        let spec = render_spec(b, args)?;
        let tile = positive_arg(args, "tile_mm", TILE_DEFAULT_MM, 1000.0)?;
        if tile < 1.0 {
            return Err(invalid("tile_mm", "must be at least 1 mm"));
        }
        let overlap = match args.get("overlap_mm") {
            None | Some(Value::Null) => TILE_OVERLAP_DEFAULT_MM.min(tile / 2.0),
            Some(v) => match v
                .as_f64()
                .filter(|x| x.is_finite() && *x >= 0.0 && *x < tile)
            {
                Some(x) => x,
                None => {
                    return Err(invalid(
                        "overlap_mm",
                        "must be at least 0 and less than tile_mm",
                    ))
                }
            },
        };
        Canvas::size_for(0.0, 0.0, tile, tile, spec.px_per_mm)
            .map_err(|e| invalid("px_per_mm", &format!("a {tile} mm tile: {e:#}")))?;
        let only = opt_bool(args, "only_tiles_with_markers", false)?;
        let max_tiles = match args.get("max_tiles") {
            None | Some(Value::Null) => TILES_DEFAULT,
            Some(v) => match v.as_u64().filter(|n| (1..=TILES_MAX).contains(n)) {
                Some(n) => n,
                None => {
                    return Err(invalid(
                        "max_tiles",
                        &format!("must be an integer from 1 to {TILES_MAX}"),
                    ))
                }
            },
        } as usize;
        let dir = opt_path(args, "output_dir")?;
        Ok((window, spec, tile, overlap, only, max_tiles, dir))
    })();
    let (window, spec, tile, overlap, only, max_tiles, dir) = match parsed {
        Ok(v) => v,
        Err(r) => return Ok(r),
    };
    let ordered = spec.layer_names(b);
    let dir = match dir {
        Some(d) => d,
        None => {
            let stem = saved
                .path
                .file_stem()
                .map(|s| s.to_string_lossy().into_owned())
                .unwrap_or_else(|| "board".to_string());
            let stamp = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_millis())
                .unwrap_or(0);
            std::env::temp_dir().join("konnect-renders").join(format!(
                "{stem}-tiles-{}-{stamp}",
                ordered.join("+").replace('.', "")
            ))
        }
    };
    if dir.exists() {
        let empty = std::fs::read_dir(&dir).map(|mut d| d.next().is_none());
        match empty {
            Ok(true) => {}
            Ok(false) => {
                return Ok(CallToolResult::error_kind(
                    crate::mcp::error::ToolErrorKind::Conflict {
                        paths: vec![dir.display().to_string()],
                    },
                    format!(
                        "{} is not empty; tiles are written only to a new or empty directory. Nothing was written.",
                        dir.display()
                    ),
                ))
            }
            Err(e) => {
                return Ok(invalid(
                    "output_dir",
                    &format!("is not a readable directory: {e}"),
                ))
            }
        }
    }
    let t = std::time::Instant::now();
    let (x0, y0) = window.min.to_mm();
    let (x1, y1) = window.max.to_mm();
    let grid = tile_grid((x0, y0, x1, y1), tile, overlap);
    let rows = grid.iter().map(|g| g.row + 1).max().unwrap_or(0);
    let cols = grid.iter().map(|g| g.col + 1).max().unwrap_or(0);
    let count_in = |g: &Tile| {
        spec.markers
            .iter()
            .filter(|m| m.x >= g.x1 && m.x <= g.x2 && m.y >= g.y1 && m.y <= g.y2)
            .count()
    };
    let chosen: Vec<(Tile, usize)> = grid
        .iter()
        .map(|g| (*g, count_in(g)))
        .filter(|(_, n)| !only || *n > 0)
        .collect();
    let skipped_without_markers = grid.len() - chosen.len();
    let truncated = chosen.len() > max_tiles;
    std::fs::create_dir_all(&dir)?;
    let mut entries = Vec::new();
    for (g, n) in chosen.iter().take(max_tiles) {
        let (Some(a), Some(z)) = (Point::from_mm(g.x1, g.y1), Point::from_mm(g.x2, g.y2)) else {
            continue;
        };
        let (canvas, drawn) = match draw_window(&saved, &spec, BBox::new(a, z)) {
            Ok(v) => v,
            Err(r) => return Ok(r),
        };
        let file = format!("tile_r{:02}_c{:02}.png", g.row, g.col);
        let png = canvas.encode_png()?;
        std::fs::write(dir.join(&file), &png)?;
        entries.push(json!({
            "file": file,
            "path": dir.join(&file).display().to_string(),
            "row": g.row, "col": g.col,
            "window": { "x1": g.x1, "y1": g.y1, "x2": g.x2, "y2": g.y2 },
            "layers": ordered,
            "markers": n,
            "markers_drawn": drawn.markers,
            "width_px": canvas.width_px(), "height_px": canvas.height_px(),
        }));
    }
    let index = json!({
        "board": saved.path,
        "source": "saved_file",
        "window": { "x1": x0, "y1": y0, "x2": x1, "y2": y1 },
        "tile_mm": tile,
        "overlap_mm": overlap,
        "px_per_mm": spec.px_per_mm,
        "layers": ordered,
        "rows": rows,
        "cols": cols,
        "markers": spec.markers.len(),
        "only_tiles_with_markers": only,
        "tiles": entries,
    });
    let index_path = dir.join("index.json");
    std::fs::write(&index_path, serde_json::to_string_pretty(&index)?)?;
    Ok(CallToolResult::json(&json!({
        "source": "saved_file",
        "board": saved.path,
        "note": SAVED_FILE_NOTE,
        "output_dir": dir.display().to_string(),
        "index": index_path.display().to_string(),
        "rows": rows,
        "cols": cols,
        "tiles_in_grid": grid.len(),
        "tiles_written": entries.len(),
        "skipped_without_markers": skipped_without_markers,
        "truncated": truncated,
        "tile_mm": tile,
        "overlap_mm": overlap,
        "px_per_mm": spec.px_per_mm,
        "layers": ordered,
        "markers": spec.markers.len(),
        "tiles": entries,
        "legend": legend(),
        "elapsed_ms": t.elapsed().as_millis() as u64,
    })))
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
