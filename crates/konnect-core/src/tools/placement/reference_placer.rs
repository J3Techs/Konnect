//! `plan_reference_designator_placement` — where each footprint's reference
//! designator can sit without touching anything, planned from the saved board.
//!
//! For every footprint in scope the planner tries a short, ordered list of
//! candidates: the designator beside the footprint's courtyard on each
//! requested side, one clearance away, at each requested text angle, centred
//! on that side and then flush with its start and its end. A
//! candidate is rejected when its text box, grown by the clearance, touches
//!
//! - a pad whose copper is on the designator's face of the board (a
//!   through-hole pad is on both),
//! - another footprint's courtyard on that face,
//! - another visible silkscreen text on that face, a designator placed
//!   earlier in the same plan included,
//! - a footprint's silkscreen artwork on that face, its own included (KiCad's
//!   `silk_overlap` reports a designator across its own pin-1 marker), or
//! - the board edge, or lies outside the outline.
//!
//! The first candidate that survives is the placement. A designator with no
//! surviving candidate is hidden (or left alone with `allow_hide: false`).
//! A visible designator that touches none of these where it is, and keeps the
//! edge clearance, is kept: the clearance spaces the positions the plan
//! chooses and is not held against one already drawn clear.
//! Nothing is written: the result carries a `fields` list in the form
//! `set_reference_field_presentations` takes, which applies it in one call.
//!
//! # Determinism
//!
//! Footprints are planned in natural reference order (`C2` before `C10`),
//! never in file or hash order; every obstacle list is built in that order
//! too, and the candidate order is the caller's. The same saved board and
//! options always give the same plan.
//!
//! # Text extents
//!
//! The box is the ink of KiCad's stroke font, not KiCad's looser selection
//! box. Its width is the glyph advances of the text (measured from KiCad
//! 10.0.7 for printable ASCII, in 1/21 of the character width) times the
//! character width, less the outer side bearings, plus one stroke width; its
//! height is the character height plus one stroke width, sitting slightly
//! above the anchor as KiCad draws capitals. Justification moves the box off
//! the field's anchor, mirrored text (the back side) reverses the horizontal
//! justification, and the text angle turns the box about the anchor. The
//! constants were fitted to `PCB_TEXT::GetEffectiveTextShape` in KiCad 10.0.7
//! over front and mirrored text, four angles and all nine justifications; the
//! box held the ink to within 0.03 mm at sizes up to 1 mm. It is an estimate
//! all the same: keep a clearance above that. An outline font is not measured:
//! its footprint is skipped by the setter's own render-cache rule.

use super::*;
use crate::tools::pcb_components::reference_field::{self, Observed};
use std::collections::BTreeMap;

pub(super) const TOOL: &str = "plan_reference_designator_placement";

/// `(min_x, min_y, max_x, max_y)` in board millimetres.
type Rect = (f64, f64, f64, f64);
type Segment = ((f64, f64), (f64, f64));

/// Boxes closer than this are touching; it absorbs the 0.1 µm rounding of a
/// planned anchor.
const EPSILON: f64 = 2e-4;

pub(super) fn tool() -> ToolDef {
    tool!(
        "plan_reference_designator_placement",
        "Plan where reference designators can sit clear of everything, from the saved \
         .kicad_pcb; writes nothing to the board. For each footprint it tries the designator \
         beside the footprint's courtyard on each side in 'positions' order, at each text \
         angle in 'angles_deg' order, centred on that side and then flush with either end of \
         it, one clearance away, and takes the first candidate \
         whose text box touches no pad on that face, no other footprint's courtyard, no other \
         visible silkscreen text (designators placed earlier in the plan included), no \
         footprint silkscreen artwork and not the board edge. A designator with no clear candidate is hidden ('hidden: no clear \
         position'), or left where it is with allow_hide: false. A visible designator that touches none of these where it is and keeps the edge clearance \
         is kept, however near it sits (keep_clear_existing). Deterministic: footprints are \
         planned in natural reference order. The result lists per footprint the action \
         (placed, kept, hidden, unplaced, skipped), the chosen position and board \
         coordinates, and why each earlier candidate was rejected, with totals, and carries \
         'fields' ready for set_reference_field_presentations, which applies the whole plan \
         with one dry run and one apply; 'plan_file' writes that list to disk for its \
         'fields_file'. Text extents follow the field's own size, stroke, justification, \
         mirroring and the planned angle. Geometry is the saved file's: save the board in \
         KiCad first. Check the applied result with KiCad DRC (silk_overlap, \
         silk_over_copper, silk_edge_clearance).",
        json!({
            "type": "object",
            "properties": {
                "board": { "type": "string", "description": "Path to .kicad_pcb file" },
                "references": {
                    "type": "array", "items": { "type": "string", "minLength": 1 }, "minItems": 1,
                    "description": "Plan only these footprints; every other visible designator stays where it is and is an obstacle. Default: all."
                },
                "side": {
                    "type": "string", "enum": ["front", "back", "both"], "default": "both",
                    "description": "Plan only designators on this side's silkscreen"
                },
                "positions": {
                    "type": "array", "minItems": 1, "uniqueItems": true,
                    "items": { "type": "string", "enum": ["top", "bottom", "left", "right"] },
                    "default": ["top", "bottom", "left", "right"],
                    "description": "Sides of the courtyard to try, in order of preference, in board coordinates as seen from the front (top is smaller Y)"
                },
                "angles_deg": {
                    "type": "array", "minItems": 1, "uniqueItems": true,
                    "items": { "type": "number", "enum": [0, 90] },
                    "default": [0, 90],
                    "description": "Board text angles to try at each position, in order"
                },
                "clearance_mm": {
                    "type": "number", "minimum": 0, "maximum": 10, "default": 0.15,
                    "description": "Gap kept between a newly placed designator's text box and its own courtyard, pads, other courtyards, silkscreen artwork and other silkscreen text"
                },
                "edge_clearance_mm": {
                    "type": "number", "minimum": 0, "maximum": 10, "default": 0.3,
                    "description": "Gap kept between a designator's text box and the board edge"
                },
                "text_size_mm": {
                    "type": "number", "exclusiveMinimum": 0, "maximum": 100,
                    "description": "Set the character width and height of every planned designator; default keeps each field's size"
                },
                "text_stroke_mm": {
                    "type": "number", "exclusiveMinimum": 0, "maximum": 25,
                    "description": "Set the stroke width of every planned designator; default keeps each field's stroke"
                },
                "allow_hide": {
                    "type": "boolean", "default": true,
                    "description": "Hide a designator that has no clear position; false leaves it where it is and reports it unplaced"
                },
                "keep_clear_existing": {
                    "type": "boolean", "default": true,
                    "description": "Keep a visible designator that touches nothing where it is and keeps the edge clearance; clearance_mm is not held against it"
                },
                "include_hidden": {
                    "type": "boolean", "default": false,
                    "description": "Also plan designators that are hidden now, showing those that have a clear position"
                },
                "plan_file": {
                    "type": "string",
                    "description": "Write the 'fields' list to this path as JSON, for set_reference_field_presentations' fields_file, and leave it out of the inline result"
                }
            },
            "required": ["board"]
        }),
        |args, ctx| async move { handle_plan_reference_designator_placement(args, ctx).await }
    )
}

// ─── Options ─────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Position {
    Top,
    Bottom,
    Left,
    Right,
}

impl Position {
    fn name(self) -> &'static str {
        match self {
            Self::Top => "top",
            Self::Bottom => "bottom",
            Self::Left => "left",
            Self::Right => "right",
        }
    }

    fn parse(name: &str) -> Option<Self> {
        [Self::Top, Self::Bottom, Self::Left, Self::Right]
            .into_iter()
            .find(|position| position.name() == name)
    }
}

#[derive(Debug, Clone)]
pub(super) struct Options {
    pub references: Option<BTreeSet<String>>,
    pub side: Option<Side>,
    pub positions: Vec<Position>,
    pub angles: Vec<f64>,
    pub clearance: f64,
    pub edge_clearance: f64,
    pub text_size: Option<f64>,
    pub text_stroke: Option<f64>,
    pub allow_hide: bool,
    pub keep_clear_existing: bool,
    pub include_hidden: bool,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            references: None,
            side: None,
            positions: vec![
                Position::Top,
                Position::Bottom,
                Position::Left,
                Position::Right,
            ],
            angles: vec![0.0, 90.0],
            clearance: 0.15,
            edge_clearance: 0.3,
            text_size: None,
            text_stroke: None,
            allow_hide: true,
            keep_clear_existing: true,
            include_hidden: false,
        }
    }
}

fn options(args: &serde_json::Value) -> Result<Options, CallToolResult> {
    use crate::tools::invalid_arg;
    let mut options = Options::default();
    let strings = |key: &str| -> Result<Option<Vec<String>>, CallToolResult> {
        match args.get(key) {
            None | Some(serde_json::Value::Null) => Ok(None),
            Some(value) => value
                .as_array()
                .filter(|items| !items.is_empty())
                .and_then(|items| {
                    items
                        .iter()
                        .map(|item| item.as_str().map(str::to_string))
                        .collect::<Option<Vec<_>>>()
                })
                .map(Some)
                .ok_or_else(|| invalid_arg(key, "must be a non-empty array of strings")),
        }
    };
    let number = |key: &str, positive: bool, max: f64| -> Result<Option<f64>, CallToolResult> {
        match args.get(key) {
            None | Some(serde_json::Value::Null) => Ok(None),
            Some(value) => match value.as_f64() {
                Some(n) if n.is_finite() && n <= max && (n > 0.0 || (!positive && n == 0.0)) => {
                    Ok(Some(n))
                }
                _ => Err(invalid_arg(
                    key,
                    &format!(
                        "must be a number {} 0 and at most {max}",
                        if positive {
                            "greater than"
                        } else {
                            "of at least"
                        }
                    ),
                )),
            },
        }
    };
    let flag = |key: &str, default: bool| -> Result<bool, CallToolResult> {
        match args.get(key) {
            None | Some(serde_json::Value::Null) => Ok(default),
            Some(serde_json::Value::Bool(value)) => Ok(*value),
            Some(_) => Err(invalid_arg(key, "must be true or false")),
        }
    };

    if let Some(references) = strings("references")? {
        if references.iter().any(String::is_empty) {
            return Err(invalid_arg(
                "references",
                "must not hold an empty reference",
            ));
        }
        options.references = Some(references.into_iter().collect());
    }
    options.side = match args.get("side").and_then(serde_json::Value::as_str) {
        None | Some("both") => None,
        Some("front") => Some(Side::Front),
        Some("back") => Some(Side::Back),
        Some(other) => {
            return Err(invalid_arg(
                "side",
                &format!("must be 'front', 'back' or 'both', got '{other}'"),
            ))
        }
    };
    if args
        .get("side")
        .is_some_and(|side| !side.is_null() && !side.is_string())
    {
        return Err(invalid_arg("side", "must be 'front', 'back' or 'both'"));
    }
    if let Some(names) = strings("positions")? {
        let mut positions = Vec::new();
        for name in &names {
            let Some(position) = Position::parse(name) else {
                return Err(invalid_arg(
                    "positions",
                    &format!("'{name}' is not one of top, bottom, left, right"),
                ));
            };
            if positions.contains(&position) {
                return Err(invalid_arg(
                    "positions",
                    &format!("'{name}' is given more than once"),
                ));
            }
            positions.push(position);
        }
        options.positions = positions;
    }
    match args.get("angles_deg") {
        None | Some(serde_json::Value::Null) => {}
        Some(value) => {
            let angles = value
                .as_array()
                .filter(|items| !items.is_empty())
                .and_then(|items| {
                    items
                        .iter()
                        .map(serde_json::Value::as_f64)
                        .collect::<Option<Vec<_>>>()
                })
                .filter(|angles| angles.iter().all(|angle| *angle == 0.0 || *angle == 90.0))
                .filter(|angles| angles.len() < 2 || angles[0] != angles[1])
                .filter(|angles| angles.len() <= 2);
            match angles {
                Some(angles) => options.angles = angles,
                None => {
                    return Err(invalid_arg(
                        "angles_deg",
                        "must be [0], [90], [0, 90] or [90, 0]",
                    ))
                }
            }
        }
    }
    if let Some(clearance) = number("clearance_mm", false, 10.0)? {
        options.clearance = clearance;
    }
    if let Some(clearance) = number("edge_clearance_mm", false, 10.0)? {
        options.edge_clearance = clearance;
    }
    options.text_size = number("text_size_mm", true, 100.0)?;
    options.text_stroke = number("text_stroke_mm", true, 25.0)?;
    options.allow_hide = flag("allow_hide", true)?;
    options.keep_clear_existing = flag("keep_clear_existing", true)?;
    options.include_hidden = flag("include_hidden", false)?;
    Ok(options)
}

// ─── Text extents ────────────────────────────────────────────────────────────

/// Glyph advances of KiCad's stroke font for U+0020..U+007E, in 1/21 of the
/// character width, measured from KiCad 10.0.7 (`pcbnew.PCB_TEXT` bounding
/// boxes of repeated glyphs). A character outside the table, or one KiCad's
/// markup consumes, takes the widest advance.
const ADVANCES: [u8; 95] = [
    16, 10, 16, 21, 28, 24, 26, 10, 14, 14, 16, 26, 10, 26, 10, 22, // space .. /
    20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 10, 10, 26, 26, 26, 18, // 0 .. ?
    27, 18, 21, 21, 21, 19, 18, 21, 22, 10, 16, 21, 17, 24, 22, 22, // @ .. O
    21, 22, 21, 20, 16, 22, 18, 24, 20, 18, 20, 14, 14, 14, 28, 28, // P .. _
    8, 19, 19, 18, 19, 18, 12, 19, 19, 10, 10, 17, 11, 28, 19, 19, // ` .. o
    19, 19, 13, 17, 12, 19, 16, 22, 17, 16, 17, 28, 20, 28, 28, // p .. ~
];
const WIDEST_ADVANCE: u8 = 28;

/// What the first and last glyphs' side bearings take off a centred text's
/// advance, in character widths; the least measured, so the box never
/// undercuts the ink.
const SIDE_BEARINGS: f64 = 0.25;
/// How far capitals stand above a vertically centred anchor, in heights.
const CAP_ABOVE: f64 = 0.545;
/// How far a top- or bottom-justified text moves from the centred one, in
/// heights.
const VERTICAL_JUSTIFY_SHIFT: f64 = 0.585;
/// The extra reach, in heights, allowed above and below text that holds
/// anything but capitals other than `Q`, digits and `-+.:=`.
const TALL_GLYPH_REACH: f64 = 0.12;

fn is_plain_glyph(c: char) -> bool {
    (c.is_ascii_uppercase() && c != 'Q') || c.is_ascii_digit() || "-+.:=".contains(c)
}

/// The advance of `text` in character widths.
fn advance_in_widths(text: &str) -> f64 {
    text.chars()
        .map(|c| {
            let advance = (c as usize)
                .checked_sub(0x20)
                .and_then(|index| ADVANCES.get(index))
                .copied()
                .unwrap_or(WIDEST_ADVANCE);
            f64::from(advance) / 21.0
        })
        .sum()
}

/// How a text hangs off its anchor.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
struct Justify {
    /// −1 left, 0 centre, 1 right.
    horizontal: i8,
    /// −1 top, 0 centre, 1 bottom.
    vertical: i8,
    mirrored: bool,
}

impl Justify {
    /// From a text's `(effects … (justify …))`.
    fn read(text: &SexpNode) -> Self {
        let mut justify = Self::default();
        for node in text
            .find("effects")
            .map(|effects| effects.find_all("justify"))
            .unwrap_or_default()
        {
            for token in node.children().unwrap_or(&[]).iter().skip(1) {
                match token.as_str() {
                    Some("left") => justify.horizontal = -1,
                    Some("right") => justify.horizontal = 1,
                    Some("top") => justify.vertical = -1,
                    Some("bottom") => justify.vertical = 1,
                    Some("mirror") => justify.mirrored = true,
                    _ => {}
                }
            }
        }
        justify
    }
}

/// One text's ink, independent of where its anchor is.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Extent {
    /// The box centre minus the anchor, in board axes.
    offset: (f64, f64),
    /// Half the box's width and height, in board axes.
    half: (f64, f64),
}

impl Extent {
    fn of(
        text: &str,
        width: f64,
        height: f64,
        stroke: f64,
        angle_deg: f64,
        justify: Justify,
    ) -> Self {
        // In the text's own frame: X along the reading direction, Y down,
        // the anchor at the origin.
        let advance = advance_in_widths(text) * width;
        let (mut left, mut right) = match justify.horizontal {
            // The first and last glyphs' side bearings fall outside the ink.
            0 => {
                let ink = (advance - SIDE_BEARINGS * width).max(0.2 * width) + stroke;
                (-ink / 2.0, ink / 2.0)
            }
            // Off-centre, the whole advance is taken: the ink starts one
            // bearing in from the anchor.
            h if h < 0 => (-stroke / 2.0, advance + stroke / 2.0),
            _ => (-advance - stroke / 2.0, stroke / 2.0),
        };
        if justify.mirrored {
            (left, right) = (-right, -left);
        }
        // Capitals and digits stand CAP_ABOVE of the height above the centre
        // line; anything else may reach further either way.
        let reach = if text.chars().all(is_plain_glyph) {
            0.0
        } else {
            TALL_GLYPH_REACH * height
        };
        let shift = -f64::from(justify.vertical) * VERTICAL_JUSTIFY_SHIFT * height;
        let top = -(CAP_ABOVE * height + stroke / 2.0) - reach + shift;
        let bottom = (1.0 - CAP_ABOVE) * height + stroke / 2.0 + reach + shift;

        let (cx, cy) = ((left + right) / 2.0, (top + bottom) / 2.0);
        let (half_width, half_height) = ((right - left) / 2.0, (bottom - top) / 2.0);
        // KiCad's Y-down, counter-clockwise-positive turn, as a footprint's.
        let (sin, cos) = angle_deg.to_radians().sin_cos();
        Self {
            offset: (cx * cos + cy * sin, -cx * sin + cy * cos),
            half: (
                (half_width * cos).abs() + (half_height * sin).abs(),
                (half_width * sin).abs() + (half_height * cos).abs(),
            ),
        }
    }

    fn box_at(&self, anchor: (f64, f64)) -> Rect {
        let (cx, cy) = (anchor.0 + self.offset.0, anchor.1 + self.offset.1);
        (
            cx - self.half.0,
            cy - self.half.1,
            cx + self.half.0,
            cy + self.half.1,
        )
    }
}

// ─── Geometry ────────────────────────────────────────────────────────────────

fn grown(rect: Rect, by: f64) -> Rect {
    (rect.0 - by, rect.1 - by, rect.2 + by, rect.3 + by)
}

/// The rectangles share area (a shared edge is not area).
fn overlap(a: Rect, b: Rect) -> bool {
    a.0 < b.2 && b.0 < a.2 && a.1 < b.3 && b.1 < a.3
}

fn point_segment_distance(p: (f64, f64), (a, b): Segment) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let length2 = dx * dx + dy * dy;
    let t = if length2 == 0.0 {
        0.0
    } else {
        (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / length2).clamp(0.0, 1.0)
    };
    (p.0 - (a.0 + t * dx)).hypot(p.1 - (a.1 + t * dy))
}

fn point_rect_distance(p: (f64, f64), rect: Rect) -> f64 {
    let dx = (rect.0 - p.0).max(p.0 - rect.2).max(0.0);
    let dy = (rect.1 - p.1).max(p.1 - rect.3).max(0.0);
    dx.hypot(dy)
}

/// Liang–Barsky: the segment passes through the rectangle.
fn segment_crosses_rect((a, b): Segment, rect: Rect) -> bool {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let (mut t0, mut t1) = (0.0f64, 1.0f64);
    for (p, q) in [
        (-dx, a.0 - rect.0),
        (dx, rect.2 - a.0),
        (-dy, a.1 - rect.1),
        (dy, rect.3 - a.1),
    ] {
        if p == 0.0 {
            if q < 0.0 {
                return false;
            }
        } else {
            let r = q / p;
            if p < 0.0 {
                t0 = t0.max(r);
            } else {
                t1 = t1.min(r);
            }
            if t0 > t1 {
                return false;
            }
        }
    }
    true
}

fn segment_rect_distance(segment: Segment, rect: Rect) -> f64 {
    if segment_crosses_rect(segment, rect) {
        return 0.0;
    }
    let corners = [
        (rect.0, rect.1),
        (rect.2, rect.1),
        (rect.2, rect.3),
        (rect.0, rect.3),
    ];
    corners
        .iter()
        .map(|corner| point_segment_distance(*corner, segment))
        .chain([
            point_rect_distance(segment.0, rect),
            point_rect_distance(segment.1, rect),
        ])
        .fold(f64::INFINITY, f64::min)
}

/// The board outline as straight pieces; arcs, circles and curves are
/// approximated by chords no longer than 5° of arc.
#[derive(Debug, Default)]
struct Outline {
    segments: Vec<Segment>,
}

impl Outline {
    /// Every top-level Edge.Cuts graphic. `Err` names the first one that
    /// cannot be read, so the edge test is skipped aloud rather than run on
    /// part of an outline.
    fn read(tree: &SexpNode) -> Result<Self, String> {
        let mut segments = Vec::new();
        for child in tree.children().unwrap_or(&[]) {
            let Some(head) = child.head() else { continue };
            if child.find_str("layer") != Some("Edge.Cuts") || !head.starts_with("gr_") {
                continue;
            }
            let point = |tag: &str| -> Option<(f64, f64)> {
                let node = child.find(tag)?;
                let (x, y) = (node.get_f64(1)?, node.get_f64(2)?);
                (x.is_finite() && y.is_finite()).then_some((x, y))
            };
            let points = || -> Option<Vec<(f64, f64)>> {
                child
                    .find("pts")?
                    .find_all("xy")
                    .into_iter()
                    .map(|xy| {
                        let (x, y) = (xy.get_f64(1)?, xy.get_f64(2)?);
                        (x.is_finite() && y.is_finite()).then_some((x, y))
                    })
                    .collect()
            };
            let mut chain = |vertices: &[(f64, f64)], closed: bool| {
                for pair in vertices.windows(2) {
                    segments.push((pair[0], pair[1]));
                }
                if closed && vertices.len() > 2 {
                    segments.push((vertices[vertices.len() - 1], vertices[0]));
                }
            };
            let read = match head {
                "gr_line" => point("start")
                    .zip(point("end"))
                    .map(|(a, b)| chain(&[a, b], false)),
                "gr_rect" => point("start").zip(point("end")).map(|(a, b)| {
                    chain(&[a, (b.0, a.1), b, (a.0, b.1)], true);
                }),
                "gr_poly" => points().map(|vertices| chain(&vertices, true)),
                "gr_arc" => point("start")
                    .zip(point("mid"))
                    .zip(point("end"))
                    .map(|((start, mid), end)| chain(&arc_chords(start, mid, end), false)),
                "gr_circle" => point("center").zip(point("end")).map(|(center, end)| {
                    let radius = (end.0 - center.0).hypot(end.1 - center.1);
                    let vertices: Vec<_> = (0..72)
                        .map(|i| {
                            let angle = f64::from(i) * std::f64::consts::TAU / 72.0;
                            (
                                center.0 + radius * angle.cos(),
                                center.1 + radius * angle.sin(),
                            )
                        })
                        .collect();
                    chain(&vertices, true);
                }),
                "gr_curve" => points()
                    .filter(|control| control.len() == 4)
                    .map(|control| {
                        let vertices: Vec<_> = (0..=16)
                            .map(|i| {
                                let t = f64::from(i) / 16.0;
                                let u = 1.0 - t;
                                let weight =
                                    [u * u * u, 3.0 * u * u * t, 3.0 * u * t * t, t * t * t];
                                (
                                    (0..4).map(|k| weight[k] * control[k].0).sum(),
                                    (0..4).map(|k| weight[k] * control[k].1).sum(),
                                )
                            })
                            .collect();
                        chain(&vertices, false);
                    }),
                // Text or a dimension on Edge.Cuts is not outline geometry.
                _ => Some(()),
            };
            if read.is_none() {
                return Err(format!("an Edge.Cuts {head} cannot be read"));
            }
        }
        if segments.is_empty() {
            return Err("the board has no Edge.Cuts outline".to_string());
        }
        Ok(Self { segments })
    }

    /// Even-odd containment, which also treats a cut-out as outside.
    fn contains(&self, (px, py): (f64, f64)) -> bool {
        let mut inside = false;
        for &((x1, y1), (x2, y2)) in &self.segments {
            if (y1 > py) != (y2 > py) && px < x1 + (py - y1) / (y2 - y1) * (x2 - x1) {
                inside = !inside;
            }
        }
        inside
    }

    fn distance_to(&self, rect: Rect) -> f64 {
        self.segments
            .iter()
            .map(|segment| segment_rect_distance(*segment, rect))
            .fold(f64::INFINITY, f64::min)
    }
}

/// The arc from `start` through `mid` to `end` as chords of at most 5°.
fn arc_chords(start: (f64, f64), mid: (f64, f64), end: (f64, f64)) -> Vec<(f64, f64)> {
    let (ax, ay) = start;
    let (bx, by) = mid;
    let (cx, cy) = end;
    let d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
    if d.abs() < 1e-12 {
        return vec![start, mid, end];
    }
    let (a2, b2, c2) = (ax * ax + ay * ay, bx * bx + by * by, cx * cx + cy * cy);
    let ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d;
    let uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d;
    let radius = (ax - ux).hypot(ay - uy);
    let angle = |p: (f64, f64)| (p.1 - uy).atan2(p.0 - ux);
    let tau = std::f64::consts::TAU;
    let from = angle(start);
    let to_mid = (angle(mid) - from).rem_euclid(tau);
    let mut sweep = (angle(end) - from).rem_euclid(tau);
    // The mid point lies on the swept side; otherwise the arc runs the other way.
    if to_mid > sweep {
        sweep -= tau;
    }
    let steps = ((sweep.abs() / 5f64.to_radians()).ceil() as usize).max(1);
    let mut vertices: Vec<_> = (0..steps)
        .map(|i| {
            let a = from + sweep * i as f64 / steps as f64;
            (ux + radius * a.cos(), uy + radius * a.sin())
        })
        .collect();
    vertices[0] = start;
    vertices.push(end);
    vertices
}

// ─── The board as the planner sees it ────────────────────────────────────────

fn side_index(side: Side) -> usize {
    match side {
        Side::Front => 0,
        Side::Back => 1,
    }
}

fn silk_side(layer: Option<&str>) -> Option<Side> {
    match layer? {
        "F.SilkS" | "F.Silkscreen" => Some(Side::Front),
        "B.SilkS" | "B.Silkscreen" => Some(Side::Back),
        _ => None,
    }
}

/// `C2` before `C10`: the letters, then the number, then the whole string.
fn natural_key(reference: &str) -> (String, u64, String) {
    let digits_at = reference
        .find(|c: char| c.is_ascii_digit())
        .unwrap_or(reference.len());
    let number: String = reference[digits_at..]
        .chars()
        .take_while(char::is_ascii_digit)
        .collect();
    (
        reference[..digits_at].to_string(),
        number.parse().unwrap_or(u64::MAX),
        reference.to_string(),
    )
}

/// A named rectangle something must stay clear of.
#[derive(Debug, Clone)]
struct Obstacle {
    rect: Rect,
    name: String,
    /// The footprint it belongs to, which its own designator ignores for a
    /// courtyard.
    owner: Option<String>,
}

/// Everything fixed on one face of the board.
#[derive(Debug, Default)]
struct Face {
    pads: Vec<Obstacle>,
    courtyards: Vec<Obstacle>,
    texts: Vec<Obstacle>,
    /// Footprint silkscreen artwork, one box per stroke.
    graphics: Vec<Obstacle>,
}

/// One designator the plan may move.
#[derive(Debug, Clone)]
struct Label {
    reference: String,
    side: Side,
    footprint_side: Side,
    courtyard: Rect,
    hidden: bool,
    anchor: (f64, f64),
    angle: f64,
    width: f64,
    height: f64,
    stroke: f64,
    justify: Justify,
}

impl Label {
    fn extent(&self, angle: f64) -> Extent {
        Extent::of(
            &self.reference,
            self.width,
            self.height,
            self.stroke,
            angle,
            self.justify,
        )
    }
}

/// Which faces a pad's layer stack puts copper or a mask opening on.
fn pad_faces(pad: &SexpNode) -> [bool; 2] {
    let mut faces = [false; 2];
    for layer in pad
        .find("layers")
        .and_then(SexpNode::children)
        .unwrap_or(&[])
        .iter()
        .skip(1)
        .filter_map(SexpNode::as_str)
    {
        match layer.split_once('.').map_or(layer, |(face, _)| face) {
            "F" => faces[0] = true,
            "B" => faces[1] = true,
            "*" | "F&B" => faces = [true; 2],
            _ => {}
        }
    }
    faces
}

/// A text node's box on the board: a board `gr_text`, or a footprint's
/// `fp_text`/`property` given that footprint's `(x, y, angle)`. `None` for a
/// hidden text, one off the silkscreen, or one without readable geometry.
fn silk_text_box(
    node: &SexpNode,
    footprint: Option<(f64, f64, f64)>,
    text: &str,
) -> Option<(Side, Rect)> {
    let side = silk_side(node.find_str("layer"))?;
    let hidden = |node: &SexpNode| {
        node.find("hide")
            .is_some_and(|hide| hide.get(1).and_then(SexpNode::as_str) != Some("no"))
            || node
                .children()
                .unwrap_or(&[])
                .iter()
                .any(|child| matches!(child, SexpNode::Atom(atom) if atom == "hide"))
    };
    let effects = node.find("effects")?;
    if hidden(node) || hidden(effects) {
        return None;
    }
    let at = node.find("at")?;
    let (x, y) = (at.get_f64(1)?, at.get_f64(2)?);
    let angle = at.get_f64(3).unwrap_or(0.0);
    let anchor = match footprint {
        Some((fx, fy, rotation)) => konnect_sexp::geometry::rotate_about(x, y, fx, fy, rotation),
        None => (x, y),
    };
    let font = effects.find("font")?;
    let size = font.find("size")?;
    let (height, width) = (size.get_f64(1)?, size.get_f64(2)?);
    let stroke = font.find_f64("thickness").unwrap_or(height * 0.15);
    let all_finite = [anchor.0, anchor.1, angle, height, width, stroke]
        .iter()
        .all(|value| value.is_finite());
    all_finite.then(|| {
        (
            side,
            Extent::of(text, width, height, stroke, angle, Justify::read(node)).box_at(anchor),
        )
    })
}

/// The boxes of a footprint's silkscreen artwork on the board, one per stroke:
/// each line, each side of an unfilled rectangle or polygon, the whole of a
/// filled shape, an arc or a circle. Each is grown by half its stroke width.
/// An unreadable shape is left out; it is artwork, not a rule.
fn silk_graphic_boxes(
    footprint: &SexpNode,
    (fx, fy, rotation): (f64, f64, f64),
) -> Vec<(Side, Rect)> {
    let mut boxes = Vec::new();
    for child in footprint.children().unwrap_or(&[]) {
        let Some(head) = child.head().filter(|head| head.starts_with("fp_")) else {
            continue;
        };
        let Some(side) = silk_side(child.find_str("layer")) else {
            continue;
        };
        let point = |tag: &str| -> Option<(f64, f64)> {
            let node = child.find(tag)?;
            let (x, y) = (node.get_f64(1)?, node.get_f64(2)?);
            (x.is_finite() && y.is_finite()).then_some((x, y))
        };
        let half_stroke = child
            .find("stroke")
            .and_then(|stroke| stroke.find_f64("width"))
            .or_else(|| child.find_f64("width"))
            .filter(|width| width.is_finite() && *width > 0.0)
            .unwrap_or(0.0)
            / 2.0;
        let filled = child
            .find("fill")
            .and_then(|fill| fill.get(1))
            .and_then(SexpNode::as_str)
            .is_some_and(|fill| fill != "no" && fill != "none");
        // Local points whose hull, taken on the board, is one box.
        let mut hulls: Vec<Vec<(f64, f64)>> = Vec::new();
        let mut outline = |vertices: Vec<(f64, f64)>| {
            if filled {
                hulls.push(vertices);
            } else {
                for index in 0..vertices.len() {
                    hulls.push(vec![
                        vertices[index],
                        vertices[(index + 1) % vertices.len()],
                    ]);
                }
            }
        };
        match head {
            "fp_line" => {
                if let Some((start, end)) = point("start").zip(point("end")) {
                    hulls.push(vec![start, end]);
                }
            }
            "fp_rect" => {
                if let Some((a, b)) = point("start").zip(point("end")) {
                    outline(vec![a, (b.0, a.1), b, (a.0, b.1)]);
                }
            }
            "fp_poly" => {
                let vertices: Option<Vec<(f64, f64)>> = child.find("pts").and_then(|pts| {
                    pts.find_all("xy")
                        .into_iter()
                        .map(|xy| Some((xy.get_f64(1)?, xy.get_f64(2)?)))
                        .collect()
                });
                if let Some(vertices) = vertices.filter(|vertices| !vertices.is_empty()) {
                    outline(vertices);
                }
            }
            "fp_arc" => {
                if let Some(((start, mid), end)) =
                    point("start").zip(point("mid")).zip(point("end"))
                {
                    hulls.push(arc_chords(start, mid, end));
                }
            }
            "fp_circle" => {
                if let Some((centre, end)) = point("center").zip(point("end")) {
                    let radius = (end.0 - centre.0).hypot(end.1 - centre.1);
                    // A circle's box does not turn with the footprint.
                    let (cx, cy) =
                        konnect_sexp::geometry::transform_pad(centre.0, centre.1, fx, fy, rotation);
                    boxes.push((
                        side,
                        grown(
                            (cx - radius, cy - radius, cx + radius, cy + radius),
                            half_stroke,
                        ),
                    ));
                }
            }
            _ => {}
        }
        for hull in hulls {
            let mut rect = (
                f64::INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::NEG_INFINITY,
            );
            for (x, y) in hull {
                let (bx, by) = konnect_sexp::geometry::transform_pad(x, y, fx, fy, rotation);
                rect = (
                    rect.0.min(bx),
                    rect.1.min(by),
                    rect.2.max(bx),
                    rect.3.max(by),
                );
            }
            if rect.0.is_finite() && rect.3.is_finite() {
                boxes.push((side, grown(rect, half_stroke)));
            }
        }
    }
    boxes
}

#[derive(Debug)]
struct Scene {
    faces: [Face; 2],
    outline: Result<Outline, String>,
    /// In natural reference order.
    labels: Vec<Label>,
    /// `(reference, reason)` for footprints the plan leaves alone.
    skipped: Vec<(String, String)>,
    footprint_count: usize,
}

fn scene(source: &str, options: &Options) -> Result<Scene, String> {
    let tree = konnect_sexp::parse_sexp(source).map_err(|error| error.to_string())?;
    if tree.head() != Some("kicad_pcb") {
        return Err("the file is not a (kicad_pcb …) board".to_string());
    }
    let observed = reference_field::observe_all(source)?;

    let mut courtyards: BTreeMap<String, Vec<FootprintCourtyard>> = BTreeMap::new();
    let mut faces = [Face::default(), Face::default()];
    for courtyard in footprint_courtyards(&tree).items {
        let name = courtyard.reference.clone().unwrap_or_default();
        faces[side_index(courtyard.layer_side)]
            .courtyards
            .push(Obstacle {
                rect: courtyard.bbox,
                name: format!(
                    "courtyard of {}",
                    if name.is_empty() {
                        "an unnamed footprint"
                    } else {
                        &name
                    }
                ),
                owner: courtyard.reference.clone(),
            });
        courtyards.entry(name).or_default().push(courtyard);
    }

    let mut nodes: BTreeMap<String, Vec<&SexpNode>> = BTreeMap::new();
    let all_footprints = footprints(&tree);
    for footprint in &all_footprints {
        let reference =
            crate::tools::pcb_components::footprint_reference(footprint).unwrap_or_default();
        nodes.entry(reference.clone()).or_default().push(footprint);

        let Some((fx, fy, rotation)) = footprint
            .find("at")
            .and_then(|at| Some((at.get_f64(1)?, at.get_f64(2)?, at.get_f64(3).unwrap_or(0.0))))
        else {
            continue;
        };
        for pad in footprint.find_all("pad") {
            let Some((at, size)) = pad.find("at").zip(pad.find("size")) else {
                continue;
            };
            let (Some(lx), Some(ly), Some(w), Some(h)) = (
                at.get_f64(1),
                at.get_f64(2),
                size.get_f64(1),
                size.get_f64(2),
            ) else {
                continue;
            };
            // The stored pad angle is already the board angle.
            let (sin, cos) = at.get_f64(3).unwrap_or(0.0).to_radians().sin_cos();
            let (cx, cy) = konnect_sexp::geometry::transform_pad(lx, ly, fx, fy, rotation);
            let half = (
                (w / 2.0 * cos).abs() + (h / 2.0 * sin).abs(),
                (w / 2.0 * sin).abs() + (h / 2.0 * cos).abs(),
            );
            let number = pad.get(1).and_then(SexpNode::as_str).unwrap_or_default();
            let obstacle = Obstacle {
                rect: (cx - half.0, cy - half.1, cx + half.0, cy + half.1),
                name: if number.is_empty() {
                    format!("unnumbered pad of {reference}")
                } else {
                    format!("pad {reference}.{number}")
                },
                owner: Some(reference.clone()),
            };
            for (index, on_face) in pad_faces(pad).into_iter().enumerate() {
                if on_face {
                    faces[index].pads.push(obstacle.clone());
                }
            }
        }
        for (side, rect) in silk_graphic_boxes(footprint, (fx, fy, rotation)) {
            faces[side_index(side)].graphics.push(Obstacle {
                rect,
                name: format!("silkscreen of {reference}"),
                owner: Some(reference.clone()),
            });
        }
        // Silkscreen text the plan never moves: other fields and user text.
        for child in footprint.children().unwrap_or(&[]) {
            let (text, what) = match (child.head(), child.get(1).and_then(SexpNode::as_str)) {
                (Some("property"), Some(name)) if name != "Reference" => (
                    child.get(2).and_then(SexpNode::as_str).unwrap_or_default(),
                    format!("{name} of {reference}"),
                ),
                (Some("fp_text"), Some(kind)) if kind != "reference" => (
                    child.get(2).and_then(SexpNode::as_str).unwrap_or_default(),
                    format!("text of {reference}"),
                ),
                _ => continue,
            };
            let text = text.replace("${REFERENCE}", &reference);
            if let Some((side, rect)) = silk_text_box(child, Some((fx, fy, rotation)), &text) {
                faces[side_index(side)].texts.push(Obstacle {
                    rect,
                    name: what,
                    owner: Some(reference.clone()),
                });
            }
        }
    }
    for child in tree.find_all("gr_text") {
        let text = child.get(1).and_then(SexpNode::as_str).unwrap_or_default();
        if let Some((side, rect)) = silk_text_box(child, None, text) {
            faces[side_index(side)].texts.push(Obstacle {
                rect,
                name: format!("board text '{text}'"),
                owner: None,
            });
        }
    }

    let mut labels = Vec::new();
    let mut skipped = Vec::new();
    let mut seen_requested = BTreeSet::new();
    let mut observed: Vec<_> = observed;
    observed.sort_by_key(|(reference, _)| natural_key(reference));
    for (reference, field) in observed {
        let requested = options
            .references
            .as_ref()
            .is_none_or(|wanted| wanted.contains(&reference));
        if requested {
            seen_requested.insert(reference.clone());
        }
        let field: Observed = match field {
            Ok(field) => field,
            Err(reason) => {
                if requested {
                    skipped.push((
                        reference,
                        format!("its Reference field cannot be edited: {reason}"),
                    ));
                }
                continue;
            }
        };
        let Some(side) = silk_side(field.layer.as_deref()) else {
            if requested {
                skipped.push((
                    reference,
                    format!(
                        "its Reference field is on {}, not a silkscreen layer",
                        field.layer.as_deref().unwrap_or("no layer")
                    ),
                ));
            }
            continue;
        };
        let node = nodes.get(&reference).and_then(|found| found.first());
        let justify = node
            .and_then(|footprint| {
                footprint.children().unwrap_or(&[]).iter().find(|child| {
                    matches!(
                        (child.head(), child.get(1).and_then(SexpNode::as_str)),
                        (Some("property"), Some("Reference"))
                            | (Some("fp_text"), Some("reference"))
                    )
                })
            })
            .map(Justify::read)
            .unwrap_or_default();
        let (Some(width), Some(height)) = (field.width_mm, field.height_mm) else {
            if requested {
                skipped.push((
                    reference,
                    "its Reference field has no saved (size …)".to_string(),
                ));
            }
            continue;
        };
        let anchor = konnect_sexp::geometry::rotate_about(
            field.local.0,
            field.local.1,
            field.footprint_xy.0,
            field.footprint_xy.1,
            field.footprint_angle,
        );
        let mut label = Label {
            reference: reference.clone(),
            side,
            footprint_side: if field.footprint_layer == "B.Cu" {
                Side::Back
            } else {
                Side::Front
            },
            courtyard: (0.0, 0.0, 0.0, 0.0),
            hidden: field.hidden,
            anchor,
            angle: field.angle_deg,
            width,
            height,
            stroke: field.stroke_mm.unwrap_or(height * 0.15),
            justify,
        };
        let in_scope = requested
            && options.side.is_none_or(|wanted| wanted == side)
            && (!field.hidden || options.include_hidden);
        let courtyard = match courtyards.get(&reference).map(Vec::as_slice) {
            Some([courtyard]) => Some(courtyard.bbox),
            _ => None,
        };
        match (in_scope, courtyard) {
            (true, Some(courtyard)) => {
                label.courtyard = courtyard;
                if let Some(size) = options.text_size {
                    label.width = size;
                    label.height = size;
                }
                if let Some(stroke) = options.text_stroke {
                    label.stroke = stroke;
                }
                labels.push(label);
            }
            (in_scope, _) => {
                // Out of scope, or no usable footprint geometry: it stays, and
                // while visible it is in the way of the others.
                if !label.hidden {
                    faces[side_index(side)].texts.push(Obstacle {
                        rect: label.extent(label.angle).box_at(label.anchor),
                        name: format!("designator {reference}"),
                        owner: Some(reference.clone()),
                    });
                }
                if in_scope {
                    skipped.push((
                        reference,
                        "its footprint has no readable courtyard or pad geometry".to_string(),
                    ));
                } else if requested && options.references.is_some() && field.hidden {
                    skipped.push((
                        reference,
                        "it is hidden; include_hidden plans it".to_string(),
                    ));
                }
            }
        }
    }
    if let Some(wanted) = &options.references {
        let missing: Vec<&str> = wanted
            .iter()
            .filter(|reference| !seen_requested.contains(*reference))
            .map(String::as_str)
            .collect();
        if !missing.is_empty() {
            return Err(format!(
                "no footprint on the saved board is {}",
                missing
                    .iter()
                    .map(|reference| format!("'{reference}'"))
                    .collect::<Vec<_>>()
                    .join(", ")
            ));
        }
    }
    Ok(Scene {
        faces,
        outline: Outline::read(&tree),
        labels,
        skipped,
        footprint_count: all_footprints.len(),
    })
}

// ─── Planning ────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, PartialEq)]
pub(super) enum Action {
    /// Moved to a clear candidate.
    Placed {
        position: Position,
        /// `centre`, or flush with the `start` or `end` of that side.
        alignment: &'static str,
    },
    /// Already clear where it is.
    Kept,
    /// No clear candidate; the plan hides it.
    Hidden,
    /// No clear candidate and hiding is off, or it is hidden already.
    Unplaced,
}

#[derive(Debug, Clone)]
pub(super) struct Placement {
    pub reference: String,
    pub side: Side,
    pub footprint_side: Side,
    pub action: Action,
    pub anchor: (f64, f64),
    pub angle: f64,
    pub rect: Rect,
    pub was_hidden: bool,
    /// What the position it has now touches, for a visible designator that
    /// was not kept.
    pub current_blocked_by: Option<String>,
    /// `candidate: what it touches`, in the order tried.
    pub rejected: Vec<String>,
    /// The request for `set_reference_field_presentations`, when anything
    /// changes.
    pub field: Option<serde_json::Value>,
}

#[derive(Debug)]
pub(super) struct Plan {
    pub placements: Vec<Placement>,
    pub skipped: Vec<(String, String)>,
    pub footprint_count: usize,
    pub edge_check: Result<(), String>,
}

fn round4(value: f64) -> f64 {
    let rounded = (value * 1e4).round() / 1e4;
    if rounded == 0.0 {
        0.0
    } else {
        rounded
    }
}

/// What `rect` touches on its face, if anything.
fn blocker(
    rect: Rect,
    label: &Label,
    face: &Face,
    placed: &[Obstacle],
    outline: Option<&Outline>,
    options: &Options,
) -> Option<String> {
    let reach = grown(rect, (options.clearance - EPSILON).max(0.0));
    let own = Some(label.reference.as_str());
    let hit = |obstacles: &[Obstacle], skip_own: bool| {
        obstacles
            .iter()
            .find(|obstacle| {
                !(skip_own && obstacle.owner.as_deref() == own) && overlap(reach, obstacle.rect)
            })
            .map(|obstacle| obstacle.name.clone())
    };
    if let Some(name) = hit(&face.pads, false)
        .or_else(|| hit(&face.courtyards, true))
        .or_else(|| hit(&face.texts, false))
        .or_else(|| hit(&face.graphics, false))
        .or_else(|| hit(placed, true))
    {
        return Some(name);
    }
    let outline = outline?;
    let centre = ((rect.0 + rect.2) / 2.0, (rect.1 + rect.3) / 2.0);
    if !outline.contains(centre) {
        return Some("outside the board outline".to_string());
    }
    (outline.distance_to(rect) < options.edge_clearance - EPSILON).then(|| "board edge".to_string())
}

pub(super) fn plan(source: &str, options: &Options) -> Result<Plan, String> {
    let scene = scene(source, options)?;
    let outline = scene.outline.as_ref().ok();
    let resized = options.text_size.is_some() || options.text_stroke.is_some();
    let size_values = |label: &Label, field: &mut serde_json::Value| {
        if options.text_size.is_some() {
            field["width_mm"] = json!(label.width);
            field["height_mm"] = json!(label.height);
        }
        if options.text_stroke.is_some() {
            field["stroke_mm"] = json!(label.stroke);
        }
    };

    // Designators settled so far, per face, in the order they were settled.
    let mut settled: [Vec<Obstacle>; 2] = [Vec::new(), Vec::new()];
    let mut outcomes: BTreeMap<usize, Placement> = BTreeMap::new();
    let settle = |settled: &mut [Vec<Obstacle>; 2], label: &Label, rect: Rect| {
        settled[side_index(label.side)].push(Obstacle {
            rect,
            name: format!("designator {}", label.reference),
            owner: Some(label.reference.clone()),
        });
    };

    // A designator stays where it touches nothing: the clearance is for the
    // positions the plan chooses, not a reason to move or hide one that is
    // already drawn clear.
    let touching = Options {
        clearance: 0.0,
        ..options.clone()
    };

    // First the designators that can stay, so nothing is planned on top of them.
    if options.keep_clear_existing {
        for (index, label) in scene.labels.iter().enumerate() {
            if label.hidden {
                continue;
            }
            let rect = label.extent(label.angle).box_at(label.anchor);
            let face = &scene.faces[side_index(label.side)];
            if blocker(
                rect,
                label,
                face,
                &settled[side_index(label.side)],
                outline,
                &touching,
            )
            .is_some()
            {
                continue;
            }
            settle(&mut settled, label, rect);
            let mut field = json!({ "reference": label.reference });
            size_values(label, &mut field);
            outcomes.insert(
                index,
                Placement {
                    reference: label.reference.clone(),
                    side: label.side,
                    footprint_side: label.footprint_side,
                    action: Action::Kept,
                    anchor: label.anchor,
                    angle: label.angle,
                    rect,
                    was_hidden: false,
                    current_blocked_by: None,
                    rejected: Vec::new(),
                    field: resized.then_some(field),
                },
            );
        }
    }

    for (index, label) in scene.labels.iter().enumerate() {
        if outcomes.contains_key(&index) {
            continue;
        }
        let face = &scene.faces[side_index(label.side)];
        let gap = options.clearance;
        let (x0, y0, x1, y1) = label.courtyard;
        let mut rejected = Vec::new();
        let mut chosen = None;
        'candidates: for &position in &options.positions {
            for &angle in &options.angles {
                let extent = label.extent(angle);
                // Centred on the side first, then flush with either end of it.
                let along = |low: f64, high: f64, half: f64| {
                    let mut stops = vec![("centre", (low + high) / 2.0)];
                    for stop in [("start", low + half), ("end", high - half)] {
                        if stops.iter().all(|(_, at)| (at - stop.1).abs() > 1e-6) {
                            stops.push(stop);
                        }
                    }
                    stops
                };
                let stops: Vec<(&str, (f64, f64))> = match position {
                    Position::Top | Position::Bottom => {
                        let y = if position == Position::Top {
                            y0 - gap - extent.half.1
                        } else {
                            y1 + gap + extent.half.1
                        };
                        along(x0, x1, extent.half.0)
                            .into_iter()
                            .map(|(alignment, x)| (alignment, (x, y)))
                            .collect()
                    }
                    Position::Left | Position::Right => {
                        let x = if position == Position::Left {
                            x0 - gap - extent.half.0
                        } else {
                            x1 + gap + extent.half.0
                        };
                        along(y0, y1, extent.half.1)
                            .into_iter()
                            .map(|(alignment, y)| (alignment, (x, y)))
                            .collect()
                    }
                };
                for (alignment, box_centre) in stops {
                    let anchor = (
                        round4(box_centre.0 - extent.offset.0),
                        round4(box_centre.1 - extent.offset.1),
                    );
                    let rect = extent.box_at(anchor);
                    match blocker(
                        rect,
                        label,
                        face,
                        &settled[side_index(label.side)],
                        outline,
                        options,
                    ) {
                        None => {
                            chosen = Some((position, alignment, angle, anchor, rect));
                            break 'candidates;
                        }
                        Some(name) => rejected.push(format!(
                            "{} {alignment} at {angle}°: {name}",
                            position.name()
                        )),
                    }
                }
            }
        }
        let current = label.extent(label.angle).box_at(label.anchor);
        let current_blocked_by = (!label.hidden)
            .then(|| {
                blocker(
                    current,
                    label,
                    face,
                    &settled[side_index(label.side)],
                    outline,
                    &touching,
                )
            })
            .flatten();
        let placement = match chosen {
            Some((position, alignment, angle, anchor, rect)) => {
                settle(&mut settled, label, rect);
                let mut field = json!({
                    "reference": label.reference,
                    "board_x": anchor.0,
                    "board_y": anchor.1,
                    "angle_deg": angle,
                    "visible": true,
                });
                size_values(label, &mut field);
                Placement {
                    action: Action::Placed {
                        position,
                        alignment,
                    },
                    anchor,
                    angle,
                    rect,
                    field: Some(field),
                    ..Placement::unchanged(label, current, rejected)
                }
            }
            None if options.allow_hide && !label.hidden => Placement {
                action: Action::Hidden,
                field: Some(json!({ "reference": label.reference, "visible": false })),
                ..Placement::unchanged(label, current, rejected)
            },
            None => {
                // It stays where it is; while visible it is in the way.
                if !label.hidden {
                    settle(&mut settled, label, current);
                }
                Placement::unchanged(label, current, rejected)
            }
        };
        outcomes.insert(
            index,
            Placement {
                current_blocked_by,
                ..placement
            },
        );
    }

    Ok(Plan {
        placements: outcomes.into_values().collect(),
        skipped: scene.skipped,
        footprint_count: scene.footprint_count,
        edge_check: scene.outline.map(|_| ()),
    })
}

impl Placement {
    fn unchanged(label: &Label, rect: Rect, rejected: Vec<String>) -> Self {
        Self {
            reference: label.reference.clone(),
            side: label.side,
            footprint_side: label.footprint_side,
            action: Action::Unplaced,
            anchor: label.anchor,
            angle: label.angle,
            rect,
            was_hidden: label.hidden,
            current_blocked_by: None,
            rejected,
            field: None,
        }
    }

    fn json(&self) -> serde_json::Value {
        let side = |side: Side| if side == Side::Back { "back" } else { "front" };
        let round = |value: f64| (value * 1e4).round() / 1e4;
        let mut row = json!({
            "reference": self.reference,
            "silkscreen_side": side(self.side),
            "footprint_side": side(self.footprint_side),
            "changes": self.field.is_some(),
        });
        match &self.action {
            Action::Placed {
                position,
                alignment,
            } => {
                row["action"] = json!("placed");
                row["position"] = json!(position.name());
                row["alignment"] = json!(alignment);
                row["board_x"] = json!(self.anchor.0);
                row["board_y"] = json!(self.anchor.1);
                row["angle_deg"] = json!(self.angle);
                row["text_box"] = json!([
                    round(self.rect.0),
                    round(self.rect.1),
                    round(self.rect.2),
                    round(self.rect.3)
                ]);
                if self.was_hidden {
                    row["was_hidden"] = json!(true);
                }
            }
            Action::Kept => {
                row["action"] = json!("kept");
                row["result"] = json!("kept: the current position is clear");
                row["board_x"] = json!(round(self.anchor.0));
                row["board_y"] = json!(round(self.anchor.1));
                row["angle_deg"] = json!(self.angle);
                row["text_box"] = json!([
                    round(self.rect.0),
                    round(self.rect.1),
                    round(self.rect.2),
                    round(self.rect.3)
                ]);
            }
            Action::Hidden => {
                row["action"] = json!("hidden");
                row["result"] = json!("hidden: no clear position");
            }
            Action::Unplaced => {
                row["action"] = json!("unplaced");
                row["result"] = json!(if self.was_hidden {
                    "unplaced: no clear position; it stays hidden"
                } else {
                    "unplaced: no clear position; it stays where it is"
                });
            }
        }
        if let Some(name) = &self.current_blocked_by {
            row["current_position_blocked_by"] = json!(name);
        }
        if !self.rejected.is_empty() {
            row["rejected"] = json!(self.rejected);
        }
        row
    }
}

// ─── Handler ─────────────────────────────────────────────────────────────────

async fn handle_plan_reference_designator_placement(
    args: &serde_json::Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let board = get_path(args, "board")?;
    let options = match options(args) {
        Ok(options) => options,
        Err(refusal) => return Ok(refusal),
    };
    let plan_file = match args.get("plan_file") {
        None | Some(serde_json::Value::Null) => None,
        Some(serde_json::Value::String(path)) if !path.is_empty() => {
            Some(std::path::PathBuf::from(path))
        }
        Some(_) => {
            return Ok(crate::tools::invalid_arg(
                "plan_file",
                "must be a file path",
            ))
        }
    };
    let source = match read_saved_board(&board)? {
        Ok(source) => source,
        Err(refusal) => return Ok(refusal),
    };
    let plan = match plan(&source, &options) {
        Ok(plan) => plan,
        Err(reason) => {
            return Ok(CallToolResult::error_kind(
                ToolErrorKind::PlanBlocked {
                    operation: TOOL.to_string(),
                    reasons: vec![reason.clone()],
                },
                format!("No designator placement was planned: {reason}. Nothing was written."),
            ))
        }
    };

    let count = |wanted: fn(&Action) -> bool| {
        plan.placements
            .iter()
            .filter(|placement| wanted(&placement.action))
            .count()
    };
    let fields: Vec<serde_json::Value> = plan
        .placements
        .iter()
        .filter_map(|placement| placement.field.clone())
        .collect();
    let mut body = json!({
        "board": board.display().to_string(),
        "source": "saved",
        "status": if fields.is_empty() { "noop" } else { "planned" },
        "totals": {
            "footprints": plan.footprint_count,
            "planned": plan.placements.len(),
            "placed": count(|action| matches!(action, Action::Placed { .. })),
            "kept": count(|action| matches!(action, Action::Kept)),
            "hidden": count(|action| matches!(action, Action::Hidden)),
            "unplaced": count(|action| matches!(action, Action::Unplaced)),
            "skipped": plan.skipped.len(),
            "fields_to_change": fields.len(),
        },
        "options": {
            "side": match options.side { None => "both", Some(Side::Front) => "front", Some(Side::Back) => "back" },
            "positions": options.positions.iter().map(|position| position.name()).collect::<Vec<_>>(),
            "angles_deg": options.angles,
            "clearance_mm": options.clearance,
            "edge_clearance_mm": options.edge_clearance,
            "text_size_mm": options.text_size,
            "text_stroke_mm": options.text_stroke,
            "allow_hide": options.allow_hide,
            "keep_clear_existing": options.keep_clear_existing,
            "include_hidden": options.include_hidden,
        },
        "edge_check": match &plan.edge_check {
            Ok(()) => json!("checked"),
            Err(reason) => json!(format!("skipped: {reason}")),
        },
        "placements": plan.placements.iter().map(Placement::json).collect::<Vec<_>>(),
        "skipped": plan.skipped.iter().map(|(reference, reason)| json!({ "reference": reference, "reason": reason })).collect::<Vec<_>>(),
        "convention": "Positions are board millimetres as seen from the front (top is smaller Y); board_x/board_y is the field's anchor and angle_deg its board text angle, as set_reference_field_presentations takes them. text_box is [min_x, min_y, max_x, max_y] of the estimated stroke-font ink.",
        "next": "Pass 'fields' (or plan_file as 'fields_file') to set_reference_field_presentations: dry run, then apply with its expected_revision. Then run KiCad DRC.",
    });
    match plan_file {
        None => body["fields"] = json!(fields),
        Some(path) => {
            if let Err(error) = std::fs::write(&path, serde_json::to_string_pretty(&fields)?) {
                return Ok(crate::tools::invalid_arg(
                    "plan_file",
                    &format!("cannot be written: {error}"),
                ));
            }
            body["plan_file"] = json!(path.display().to_string());
        }
    }
    Ok(CallToolResult::json(&body))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tools::pcb_board::board_mock::ctx_talking_to;

    /// A two-pad chip part in KiCad 10's saved layout, one clause a line:
    /// courtyard ±(1.5, 0.75) about the anchor, pads 0.8 × 0.95 at x = ±0.8.
    /// `face` is `F` or `B`; `field_at` is the Reference field's saved
    /// `(at …)`; `extra` is further children.
    fn part(reference: &str, at: (f64, f64), face: char, field_at: &str, extra: &str) -> String {
        let id = reference
            .bytes()
            .fold(0u32, |sum, byte| sum.wrapping_mul(31) + u32::from(byte));
        let justify = if face == 'B' {
            "\n\t\t\t\t(justify mirror)"
        } else {
            ""
        };
        let pad = |number: u8, x: f64| {
            format!(
                "\t\t(pad \"{number}\" smd roundrect\n\
                 \t\t\t(at {x} 0)\n\
                 \t\t\t(size 0.8 0.95)\n\
                 \t\t\t(layers \"{face}.Cu\" \"{face}.Mask\" \"{face}.Paste\")\n\
                 \t\t\t(roundrect_rratio 0.25)\n\
                 \t\t\t(uuid \"{id:08x}-0000-4000-8000-00000000001{number}\")\n\
                 \t\t)\n"
            )
        };
        format!(
            "\t(footprint \"Resistor_SMD:R_0603_1608Metric\"\n\
             \t\t(layer \"{face}.Cu\")\n\
             \t\t(uuid \"{id:08x}-0000-4000-8000-000000000001\")\n\
             \t\t(at {} {})\n\
             \t\t(property \"Reference\" \"{reference}\"\n\
             \t\t\t(at {field_at})\n\
             \t\t\t(layer \"{face}.SilkS\")\n\
             \t\t\t(uuid \"{id:08x}-0000-4000-8000-000000000002\")\n\
             \t\t\t(effects\n\
             \t\t\t\t(font\n\
             \t\t\t\t\t(size 1 1)\n\
             \t\t\t\t\t(thickness 0.15)\n\
             \t\t\t\t){justify}\n\
             \t\t\t)\n\
             \t\t)\n\
             \t\t(fp_rect\n\
             \t\t\t(start -1.5 -0.75)\n\
             \t\t\t(end 1.5 0.75)\n\
             \t\t\t(stroke\n\
             \t\t\t\t(width 0.05)\n\
             \t\t\t\t(type solid)\n\
             \t\t\t)\n\
             \t\t\t(fill no)\n\
             \t\t\t(layer \"{face}.CrtYd\")\n\
             \t\t\t(uuid \"{id:08x}-0000-4000-8000-000000000003\")\n\
             \t\t)\n\
             {extra}{}{}\
             \t)\n",
            at.0,
            at.1,
            pad(1, -0.8),
            pad(2, 0.8),
        )
    }

    /// A 40 × 30 mm board, its corner at the origin, holding `parts`.
    fn board(parts: &[String]) -> String {
        format!(
            "(kicad_pcb\n\
             \t(version 20260206)\n\
             \t(generator \"pcbnew\")\n\
             \t(gr_rect\n\
             \t\t(start 0 0)\n\
             \t\t(end 40 30)\n\
             \t\t(stroke\n\
             \t\t\t(width 0.1)\n\
             \t\t\t(type default)\n\
             \t\t)\n\
             \t\t(fill no)\n\
             \t\t(layer \"Edge.Cuts\")\n\
             \t\t(uuid \"e0000000-0000-4000-8000-000000000000\")\n\
             \t)\n\
             {})\n",
            parts.concat()
        )
    }

    fn only(references: &[&str]) -> Options {
        Options {
            references: Some(references.iter().map(|name| name.to_string()).collect()),
            ..Options::default()
        }
    }

    fn placement<'a>(plan: &'a Plan, reference: &str) -> &'a Placement {
        plan.placements
            .iter()
            .find(|placement| placement.reference == reference)
            .unwrap_or_else(|| panic!("{reference} is not in the plan"))
    }

    fn placed(placement: &Placement) -> (Position, &'static str, f64) {
        match placement.action {
            Action::Placed {
                position,
                alignment,
            } => (position, alignment, placement.angle),
            ref other => panic!(
                "{} is {other:?}, rejected {:?}",
                placement.reference, placement.rejected
            ),
        }
    }

    fn near(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-6
    }

    #[test]
    fn a_lone_part_gets_its_designator_centred_above_its_courtyard_one_clearance_away() {
        let source = board(&[part("R1", (10.0, 10.0), 'F', "0 0 0", "")]);
        let plan = plan(&source, &Options::default()).unwrap();
        assert_eq!(plan.placements.len(), 1);
        let r1 = placement(&plan, "R1");
        assert_eq!(placed(r1), (Position::Top, "centre", 0.0));
        assert!(r1.rejected.is_empty(), "{:?}", r1.rejected);
        assert_eq!(
            r1.current_blocked_by.as_deref(),
            Some("pad R1.1"),
            "where it was, it lay across its own pads"
        );
        // The courtyard's top is y = 9.25; the box ends 0.15 above it and is
        // centred on x = 10.
        assert!(near(r1.rect.3, 9.1), "{:?}", r1.rect);
        assert!(near((r1.rect.0 + r1.rect.2) / 2.0, 10.0), "{:?}", r1.rect);
        // "R1" at 1 mm with a 0.15 mm stroke: capitals stand 0.62 above the
        // anchor and 0.53 below, so the anchor is 0.53 above the box's foot.
        assert_eq!(r1.anchor, (10.0, 8.57));
        assert_eq!(
            r1.field,
            Some(json!({
                "reference": "R1", "board_x": 10.0, "board_y": 8.57,
                "angle_deg": 0.0, "visible": true,
            }))
        );
        assert!(plan.edge_check.is_ok());
    }

    #[test]
    fn a_pad_a_courtyard_and_silkscreen_artwork_each_reject_a_candidate_by_name() {
        // R2 sits just above R1: its courtyard covers every candidate above.
        let source = board(&[
            part("R1", (10.0, 10.0), 'F', "0 0 0", ""),
            part("R2", (10.0, 8.3), 'F', "0 0 0", ""),
        ]);
        let plan_ = plan(&source, &only(&["R1"])).unwrap();
        let r1 = placement(&plan_, "R1");
        assert_eq!(placed(r1), (Position::Bottom, "centre", 0.0));
        assert_eq!(r1.rejected.len(), 6, "{:?}", r1.rejected);
        assert_eq!(r1.rejected[0], "top centre at 0°: pad R2.1");
        assert!(r1.rejected.iter().all(|reason| reason.starts_with("top ")));

        // Its own pin-1 bar above the courtyard does the same.
        let bar = "\t\t(fp_line\n\
                   \t\t\t(start -0.5 -1)\n\
                   \t\t\t(end 0.5 -1)\n\
                   \t\t\t(stroke\n\
                   \t\t\t\t(width 0.12)\n\
                   \t\t\t\t(type solid)\n\
                   \t\t\t)\n\
                   \t\t\t(layer \"F.SilkS\")\n\
                   \t\t\t(uuid \"b0000000-0000-4000-8000-000000000000\")\n\
                   \t\t)\n";
        let source = board(&[part("R1", (10.0, 10.0), 'F', "0 0 0", bar)]);
        let plan_ = plan(&source, &Options::default()).unwrap();
        let r1 = placement(&plan_, "R1");
        assert_eq!(placed(r1), (Position::Bottom, "centre", 0.0));
        assert_eq!(r1.rejected[0], "top centre at 0°: silkscreen of R1");

        // A courtyard with no pad under the candidate is named as a courtyard.
        let source = board(&[
            part("R1", (10.0, 10.0), 'F', "0 0 0", ""),
            part("R2", (10.0, 8.3), 'F', "0 0 0", "").replace("(pad ", "(skipped_pad "),
        ]);
        let plan_ = plan(&source, &only(&["R1"])).unwrap();
        assert_eq!(
            placement(&plan_, "R1").rejected[0],
            "top centre at 0°: courtyard of R2"
        );
    }

    #[test]
    fn a_designator_placed_earlier_in_the_plan_blocks_a_later_one_and_so_does_the_board_edge() {
        // R1 is too near the top edge for a designator above it, so it takes
        // the gap below, which is the gap above R2.
        let source = board(&[
            part("R1", (10.0, 1.5), 'F', "0 0 0", ""),
            part("R2", (10.0, 4.95), 'F', "0 0 0", ""),
        ]);
        let plan_ = plan(&source, &Options::default()).unwrap();
        let r1 = placement(&plan_, "R1");
        assert_eq!(placed(r1), (Position::Bottom, "centre", 0.0));
        assert_eq!(
            r1.rejected,
            [
                "top centre at 0°: board edge",
                "top start at 0°: board edge",
                "top end at 0°: board edge",
                "top centre at 90°: outside the board outline",
                "top start at 90°: outside the board outline",
                "top end at 90°: outside the board outline",
            ]
        );
        let r2 = placement(&plan_, "R2");
        assert_eq!(placed(r2), (Position::Bottom, "centre", 0.0));
        assert_eq!(
            &r2.rejected[..3],
            [
                "top centre at 0°: designator R1",
                "top start at 0°: designator R1",
                "top end at 0°: designator R1",
            ]
        );
        assert!(!overlap(grown(r1.rect, 0.15), r2.rect));
    }

    #[test]
    fn a_designator_with_no_clear_candidate_is_hidden_or_left_unplaced_when_hiding_is_off() {
        let source = board(&[
            part("R1", (10.0, 10.0), 'F', "0 0 0", ""),
            part("R2", (10.0, 8.3), 'F', "0 0 0", ""),
            part("R3", (10.0, 11.7), 'F', "0 0 0", ""),
            part("R4", (6.8, 10.0), 'F', "0 0 0", ""),
            part("R5", (13.2, 10.0), 'F', "0 0 0", ""),
        ]);
        let hidden = plan(&source, &only(&["R1"])).unwrap();
        let r1 = placement(&hidden, "R1");
        assert_eq!(r1.action, Action::Hidden);
        assert_eq!(
            r1.field,
            Some(json!({ "reference": "R1", "visible": false }))
        );
        assert_eq!(
            r1.rejected.len(),
            24,
            "four sides, two angles, three stops: {:?}",
            r1.rejected
        );
        assert_eq!(r1.json()["result"], "hidden: no clear position");

        let kept_visible = plan(
            &source,
            &Options {
                allow_hide: false,
                ..only(&["R1"])
            },
        )
        .unwrap();
        let r1 = placement(&kept_visible, "R1");
        assert_eq!(r1.action, Action::Unplaced);
        assert_eq!(r1.field, None);
    }

    #[test]
    fn a_clear_designator_is_kept_and_a_hidden_one_is_planned_only_on_request() {
        let clear = board(&[part("R1", (10.0, 10.0), 'F', "0.3 -1.6 0", "")]);
        let plan_ = plan(&clear, &Options::default()).unwrap();
        let r1 = placement(&plan_, "R1");
        assert_eq!(r1.action, Action::Kept);
        assert_eq!(r1.field, None, "a kept designator is not rewritten");
        let moved = plan(
            &clear,
            &Options {
                keep_clear_existing: false,
                ..Options::default()
            },
        )
        .unwrap();
        assert_eq!(placement(&moved, "R1").anchor, (10.0, 8.57));

        // A new text size is a change even for a designator that stays.
        let resized = plan(
            &clear,
            &Options {
                text_size: Some(0.8),
                text_stroke: Some(0.12),
                ..Options::default()
            },
        )
        .unwrap();
        assert_eq!(
            placement(&resized, "R1").field,
            Some(json!({
                "reference": "R1", "width_mm": 0.8, "height_mm": 0.8, "stroke_mm": 0.12,
            }))
        );

        let hidden = clear.replace(
            "\t\t\t(layer \"F.SilkS\")\n",
            "\t\t\t(layer \"F.SilkS\")\n\t\t\t(hide yes)\n",
        );
        assert!(plan(&hidden, &Options::default())
            .unwrap()
            .placements
            .is_empty());
        let shown = plan(
            &hidden,
            &Options {
                include_hidden: true,
                ..Options::default()
            },
        )
        .unwrap();
        let r1 = placement(&shown, "R1");
        assert_eq!(placed(r1), (Position::Top, "centre", 0.0));
        assert!(r1.was_hidden);
        assert_eq!(r1.field.as_ref().unwrap()["visible"], true);
    }

    #[test]
    fn a_designator_nearer_than_the_clearance_but_touching_nothing_is_kept() {
        // R1's designator stands 0.03 mm below R2's courtyard: nearer than the
        // 0.15 mm clearance, and touching nothing.
        let source = board(&[
            part("R1", (10.0, 10.0), 'F', "0 -1.4 0", ""),
            part("R2", (10.0, 7.2), 'F', "0 0 0", ""),
        ]);
        let plan_ = plan(&source, &only(&["R1"])).unwrap();
        let r1 = placement(&plan_, "R1");
        assert!(near(r1.rect.1, 7.98), "{:?}", r1.rect);
        assert_eq!(r1.action, Action::Kept);
        assert_eq!(r1.field, None);

        // Planned afresh, the same gap is too small: it goes below instead.
        let moved = plan(
            &source,
            &Options {
                keep_clear_existing: false,
                ..only(&["R1"])
            },
        )
        .unwrap();
        let r1 = placement(&moved, "R1");
        assert_eq!(placed(r1), (Position::Bottom, "centre", 0.0));
        assert_eq!(r1.current_blocked_by, None, "where it was touched nothing");
    }

    #[test]
    fn a_back_side_designator_ignores_the_front_face_but_not_a_through_hole_pad() {
        let front_neighbour = part("R2", (10.0, 8.3), 'F', "0 0 0", "");
        let source = board(&[
            part("R1", (10.0, 10.0), 'B', "0 0 0", ""),
            front_neighbour.clone(),
        ]);
        let plan_ = plan(&source, &only(&["R1"])).unwrap();
        let r1 = placement(&plan_, "R1");
        assert_eq!(r1.side, Side::Back);
        assert_eq!(placed(r1), (Position::Top, "centre", 0.0));
        assert_eq!(r1.anchor, (10.0, 8.57));

        let through_hole = front_neighbour.replace(
            "(layers \"F.Cu\" \"F.Mask\" \"F.Paste\")",
            "(layers \"*.Cu\" \"*.Mask\")",
        );
        let source = board(&[part("R1", (10.0, 10.0), 'B', "0 0 0", ""), through_hole]);
        let plan_ = plan(&source, &only(&["R1"])).unwrap();
        let r1 = placement(&plan_, "R1");
        assert_eq!(placed(r1), (Position::Bottom, "centre", 0.0));
        assert_eq!(r1.rejected[0], "top centre at 0°: pad R2.1");

        // side: front leaves the back designator out of the plan.
        let front_only = plan(
            &source,
            &Options {
                side: Some(Side::Front),
                ..Options::default()
            },
        )
        .unwrap();
        assert!(front_only
            .placements
            .iter()
            .all(|placement| placement.reference == "R2"));
    }

    #[test]
    fn the_plan_follows_natural_reference_order_whatever_order_the_file_holds() {
        // Whichever of the two is planned first takes the gap between them, so
        // the order shows: C2 goes before C10, though "C10" sorts first as text.
        let near_edge = part("C10", (10.0, 1.5), 'F', "0 0 0", "");
        let below = part("C2", (10.0, 4.95), 'F', "0 0 0", "");
        let other = part("R1", (25.0, 10.0), 'F', "0 0 0", "");
        let one = plan(
            &board(&[near_edge.clone(), below.clone(), other.clone()]),
            &Options::default(),
        )
        .unwrap();
        let two = plan(&board(&[other, below, near_edge]), &Options::default()).unwrap();

        let order = |plan: &Plan| -> Vec<String> {
            plan.placements
                .iter()
                .map(|placement| placement.reference.clone())
                .collect()
        };
        assert_eq!(order(&one), ["C2", "C10", "R1"]);
        assert_eq!(
            placed(placement(&one, "C2")),
            (Position::Top, "centre", 0.0)
        );
        assert_eq!(
            placed(placement(&one, "C10")),
            (Position::Left, "centre", 0.0)
        );
        let fields = |plan: &Plan| -> Vec<serde_json::Value> {
            plan.placements
                .iter()
                .map(|placement| placement.json())
                .chain(
                    plan.placements
                        .iter()
                        .filter_map(|placement| placement.field.clone()),
                )
                .collect()
        };
        assert_eq!(fields(&one), fields(&two));
        assert_eq!(
            natural_key("C2").cmp(&natural_key("C10")),
            std::cmp::Ordering::Less
        );
    }

    #[test]
    fn the_text_box_holds_the_ink_kicad_draws() {
        // (text, width, height, stroke, angle, horizontal, vertical, mirrored,
        // ink box about the anchor) from PCB_TEXT::GetEffectiveTextShape in
        // KiCad 10.0.7.
        type Case = (&'static str, f64, f64, f64, f64, i8, i8, bool, Rect);
        let cases: [Case; 8] = [
            (
                "R301",
                1.0,
                1.0,
                0.15,
                0.0,
                0,
                0,
                false,
                (-1.7655, -0.6202, 1.8131, 0.5298),
            ),
            (
                "C201",
                0.8,
                0.8,
                0.15,
                90.0,
                0,
                0,
                false,
                (-0.5127, -1.4655, 0.4373, 1.4655),
            ),
            (
                "U2",
                1.0,
                1.0,
                0.15,
                0.0,
                -1,
                0,
                false,
                (0.2618, -0.6202, 1.9832, 0.5298),
            ),
            (
                "J1",
                0.6,
                0.8,
                0.08,
                90.0,
                1,
                1,
                false,
                (-0.9421, 0.1269, -0.0621, 1.0355),
            ),
            (
                "Q8",
                1.0,
                1.0,
                0.15,
                0.0,
                0,
                0,
                false,
                (-0.8845, -0.6202, 0.8845, 0.6251),
            ),
            (
                "TP10",
                0.8,
                0.8,
                0.15,
                0.0,
                0,
                0,
                true,
                (-1.3893, -0.5127, 1.4655, 0.4373),
            ),
            (
                "FB3",
                1.0,
                1.0,
                0.15,
                90.0,
                -1,
                -1,
                true,
                (-0.0352, 0.2618, 1.1148, 2.7927),
            ),
            (
                "D15",
                0.8,
                0.8,
                0.15,
                0.0,
                0,
                0,
                false,
                (-1.0464, -0.5127, 1.0845, 0.4373),
            ),
        ];
        for (text, width, height, stroke, angle, horizontal, vertical, mirrored, ink) in cases {
            let justify = Justify {
                horizontal,
                vertical,
                mirrored,
            };
            let model = Extent::of(text, width, height, stroke, angle, justify).box_at((0.0, 0.0));
            let outside = (model.0 - ink.0)
                .max(model.1 - ink.1)
                .max(ink.2 - model.2)
                .max(ink.3 - model.3);
            let slack = (ink.0 - model.0)
                .max(ink.1 - model.1)
                .max(model.2 - ink.2)
                .max(model.3 - ink.3);
            assert!(
                outside <= 0.03,
                "{text}: ink {ink:?} leaves {model:?} by {outside}"
            );
            assert!(
                slack <= 0.35,
                "{text}: {model:?} is {slack} looser than {ink:?}"
            );
        }
    }

    #[test]
    fn options_outside_the_schema_and_unknown_references_are_refused() {
        for args in [
            json!({ "positions": ["top", "top"] }),
            json!({ "positions": ["above"] }),
            json!({ "positions": [] }),
            json!({ "angles_deg": [45] }),
            json!({ "angles_deg": [0, 0] }),
            json!({ "clearance_mm": -0.1 }),
            json!({ "text_size_mm": 0 }),
            json!({ "side": "top" }),
            json!({ "allow_hide": "no" }),
            json!({ "references": [""] }),
        ] {
            assert!(options(&args).is_err(), "{args} must be refused");
        }
        let parsed = options(&json!({
            "positions": ["right", "top"], "angles_deg": [90], "side": "back",
            "clearance_mm": 0, "references": ["R1"],
        }))
        .unwrap();
        assert_eq!(parsed.positions, [Position::Right, Position::Top]);
        assert_eq!(parsed.angles, [90.0]);
        assert_eq!(parsed.side, Some(Side::Back));
        assert_eq!(parsed.clearance, 0.0);

        let source = board(&[part("R1", (10.0, 10.0), 'F', "0 0 0", "")]);
        let reason = plan(&source, &only(&["R1", "R99"])).unwrap_err();
        assert!(reason.contains("'R99'"), "{reason}");
    }

    async fn call(tool: &str, args: serde_json::Value) -> (bool, serde_json::Value) {
        let def = crate::tools::placement::tools()
            .into_iter()
            .find(|def| def.name == tool)
            .unwrap();
        assert!(def.input_validator.is_valid(&args), "schema refused {args}");
        let result = (def.handler)(&args, std::sync::Arc::new(ctx_talking_to(String::new())))
            .await
            .unwrap();
        let crate::mcp::protocol::ToolContent::Text { text } = &result.content[0] else {
            panic!("expected text content");
        };
        (
            result.is_error,
            serde_json::from_str(text).unwrap_or_else(|_| json!({ "text": text })),
        )
    }

    #[tokio::test]
    async fn the_plan_applies_through_the_setter_in_one_batch_and_then_plans_nothing() {
        let tmp = tempfile::tempdir().unwrap();
        let board_path = tmp.path().join("labels.kicad_pcb");
        let source = board(&[
            part("R1", (10.0, 10.0), 'F', "0 0 0", ""),
            part("R2", (20.0, 10.0), 'B', "0 0 0", ""),
            // Boxed in on the front: its designator is hidden.
            part("R3", (30.0, 20.0), 'F', "0 0 0", ""),
            part("R4", (30.0, 18.3), 'F', "0.3 -1.6 0", ""),
            part("R5", (30.0, 21.7), 'F', "0.3 1.6 0", ""),
            part("R6", (26.8, 20.0), 'F', "-0.3 -1.6 0", ""),
            part("R7", (33.2, 20.0), 'F', "0.3 -1.6 0", ""),
        ]);
        std::fs::write(&board_path, &source).unwrap();

        let (error, planned) = call(TOOL, json!({ "board": board_path })).await;
        assert!(!error, "{planned}");
        assert_eq!(planned["status"], "planned");
        assert_eq!(planned["source"], "saved");
        assert_eq!(planned["edge_check"], "checked");
        assert_eq!(
            planned["totals"],
            json!({
                "footprints": 7, "planned": 7, "placed": 2, "kept": 4, "hidden": 1,
                "unplaced": 0, "skipped": 0, "fields_to_change": 3,
            })
        );
        assert_eq!(
            std::fs::read_to_string(&board_path).unwrap(),
            source,
            "planning writes nothing"
        );
        let (_, again) = call(TOOL, json!({ "board": board_path })).await;
        assert_eq!(again, planned, "the same board gives the same plan");

        // plan_file carries the same list, for the setter's fields_file.
        let plan_file = tmp.path().join("plan.json");
        let (error, filed) =
            call(TOOL, json!({ "board": board_path, "plan_file": plan_file })).await;
        assert!(!error, "{filed}");
        assert!(filed.get("fields").is_none());
        let listed: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(&plan_file).unwrap()).unwrap();
        assert_eq!(listed, planned["fields"]);

        const SETTER: &str = "set_reference_field_presentations";
        let (error, dry) = call(
            SETTER,
            json!({ "board": board_path, "fields_file": plan_file }),
        )
        .await;
        assert!(!error, "{dry}");
        assert_eq!(dry["changed_count"], 3);
        let (error, applied) = call(
            SETTER,
            json!({
                "board": board_path, "fields_file": plan_file, "dry_run": false,
                "expected_revision": dry["revision"],
            }),
        )
        .await;
        assert!(!error, "{applied}");
        assert_eq!(applied["status"], "applied");

        let (error, after) = call(TOOL, json!({ "board": board_path })).await;
        assert!(!error, "{after}");
        assert_eq!(after["status"], "noop");
        assert_eq!(after["totals"]["kept"], 6);
        assert_eq!(after["totals"]["planned"], 6, "the hidden one is left out");
        assert_eq!(after["fields"], json!([]));
    }

    #[tokio::test]
    async fn a_board_without_an_outline_is_planned_with_the_edge_check_reported_skipped() {
        let tmp = tempfile::tempdir().unwrap();
        let board_path = tmp.path().join("no_outline.kicad_pcb");
        let source = board(&[part("R1", (10.0, 1.5), 'F', "0 0 0", "")])
            .replace("(layer \"Edge.Cuts\")", "(layer \"Dwgs.User\")");
        std::fs::write(&board_path, source).unwrap();
        let (error, planned) = call(TOOL, json!({ "board": board_path })).await;
        assert!(!error, "{planned}");
        assert_eq!(
            planned["edge_check"],
            "skipped: the board has no Edge.Cuts outline"
        );
        assert_eq!(planned["placements"][0]["position"], "top");

        let (error, missing) = call(
            TOOL,
            json!({ "board": tmp.path().join("absent.kicad_pcb") }),
        )
        .await;
        assert!(error, "{missing}");
    }
}
