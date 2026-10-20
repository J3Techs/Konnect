//! A millimetre-addressed raster canvas for board renders.
//!
//! Callers draw in board millimetres (x right, y down, as KiCad stores
//! them); the canvas maps a window of the board onto pixels at a fixed
//! scale. Copper shapes are drawn exactly as the router models them: discs,
//! stadiums (round-capped strokes), convex polygons grown by a radius
//! (fill plus a round-joined stroke) and polygons with holes (even-odd
//! fill). Text uses the built-in stroke font, never a system font, so the
//! same scene renders to the same pixels on every machine.

use crate::font;
use anyhow::{bail, Context, Result};
use resvg::tiny_skia::{
    Color, FillRule, LineCap, LineJoin, Paint, PathBuilder, Pixmap, Stroke, StrokeDash, Transform,
};

/// Largest side of a canvas, px.
pub const MAX_SIDE_PX: u32 = 8192;
/// Largest canvas area, px.
pub const MAX_PIXELS: u64 = 40_000_000;

/// An sRGB colour with alpha.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Rgba(pub u8, pub u8, pub u8, pub u8);

impl Rgba {
    pub fn with_alpha(self, a: u8) -> Rgba {
        Rgba(self.0, self.1, self.2, a)
    }

    /// Scale the alpha by `f` in `[0, 1]`.
    pub fn faded(self, f: f32) -> Rgba {
        Rgba(
            self.0,
            self.1,
            self.2,
            (f32::from(self.3) * f.clamp(0.0, 1.0)).round() as u8,
        )
    }

    fn paint(self) -> Paint<'static> {
        let mut p = Paint::default();
        p.set_color_rgba8(self.0, self.1, self.2, self.3);
        p.anti_alias = true;
        p
    }
}

/// Where a text label sits relative to its anchor point.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Anchor {
    Center,
    /// Left edge, vertically centred.
    Left,
    /// Top-left corner.
    TopLeft,
}

/// A board window being drawn.
pub struct Canvas {
    pixmap: Pixmap,
    x0: f64,
    y0: f64,
    scale: f64,
}

impl Canvas {
    /// A canvas showing `x0..x1` × `y0..y1` mm at `px_per_mm`, filled with
    /// `background`.
    pub fn new(
        x0: f64,
        y0: f64,
        x1: f64,
        y1: f64,
        px_per_mm: f64,
        background: Rgba,
    ) -> Result<Self> {
        let (w, h) = Self::size_for(x0, y0, x1, y1, px_per_mm)?;
        let mut pixmap = Pixmap::new(w, h).context("could not allocate the canvas")?;
        pixmap.fill(Color::from_rgba8(
            background.0,
            background.1,
            background.2,
            background.3,
        ));
        Ok(Canvas {
            pixmap,
            x0: x0.min(x1),
            y0: y0.min(y1),
            scale: px_per_mm,
        })
    }

    /// The pixel size a window would have, refusing one too large to draw.
    pub fn size_for(x0: f64, y0: f64, x1: f64, y1: f64, px_per_mm: f64) -> Result<(u32, u32)> {
        if !(px_per_mm.is_finite() && px_per_mm > 0.0) {
            bail!("px_per_mm must be a positive number");
        }
        let w = ((x1 - x0).abs() * px_per_mm).round();
        let h = ((y1 - y0).abs() * px_per_mm).round();
        if !(w.is_finite() && h.is_finite()) || w < 1.0 || h < 1.0 {
            bail!("the window is empty at this scale");
        }
        if w > f64::from(MAX_SIDE_PX) || h > f64::from(MAX_SIDE_PX) || w * h > MAX_PIXELS as f64 {
            bail!(
                "the window would be {w} x {h} px; the limit is {MAX_SIDE_PX} px a side and \
                 {MAX_PIXELS} px in all: lower px_per_mm or draw a smaller window"
            );
        }
        Ok((w as u32, h as u32))
    }

    pub fn width_px(&self) -> u32 {
        self.pixmap.width()
    }

    pub fn height_px(&self) -> u32 {
        self.pixmap.height()
    }

    pub fn px_per_mm(&self) -> f64 {
        self.scale
    }

    /// Board millimetres to canvas pixels.
    pub fn to_px(&self, x: f64, y: f64) -> (f32, f32) {
        (
            ((x - self.x0) * self.scale) as f32,
            ((y - self.y0) * self.scale) as f32,
        )
    }

    fn mm_to_px(&self, d: f64) -> f32 {
        (d * self.scale) as f32
    }

    fn ring_path(&self, pb: &mut PathBuilder, ring: &[(f64, f64)]) {
        let mut it = ring.iter();
        let Some(&(x, y)) = it.next() else { return };
        let (px, py) = self.to_px(x, y);
        pb.move_to(px, py);
        for &(x, y) in it {
            let (px, py) = self.to_px(x, y);
            pb.line_to(px, py);
        }
        pb.close();
    }

    /// Fill a polygon with holes (even-odd: every ring after the first cuts).
    pub fn fill_polygon(&mut self, rings: &[Vec<(f64, f64)>], color: Rgba) {
        let mut pb = PathBuilder::new();
        for r in rings {
            if r.len() >= 3 {
                self.ring_path(&mut pb, r);
            }
        }
        if let Some(path) = pb.finish() {
            self.pixmap.fill_path(
                &path,
                &color.paint(),
                FillRule::EvenOdd,
                Transform::identity(),
                None,
            );
        }
    }

    /// A convex (or any simple) polygon grown by `radius_mm`: the fill plus a
    /// round-joined outline twice the radius wide.
    pub fn fill_rounded_polygon(&mut self, ring: &[(f64, f64)], radius_mm: f64, color: Rgba) {
        let mut pb = PathBuilder::new();
        self.ring_path(&mut pb, ring);
        let Some(path) = pb.finish() else { return };
        let paint = color.paint();
        self.pixmap.fill_path(
            &path,
            &paint,
            FillRule::Winding,
            Transform::identity(),
            None,
        );
        if radius_mm > 0.0 {
            let stroke = Stroke {
                width: self.mm_to_px(2.0 * radius_mm).max(0.5),
                line_join: LineJoin::Round,
                line_cap: LineCap::Round,
                ..Stroke::default()
            };
            self.pixmap
                .stroke_path(&path, &paint, &stroke, Transform::identity(), None);
        }
    }

    pub fn fill_circle(&mut self, x: f64, y: f64, radius_mm: f64, color: Rgba) {
        let (px, py) = self.to_px(x, y);
        let r = self.mm_to_px(radius_mm).max(0.5);
        if let Some(path) = PathBuilder::from_circle(px, py, r) {
            self.pixmap.fill_path(
                &path,
                &color.paint(),
                FillRule::Winding,
                Transform::identity(),
                None,
            );
        }
    }

    /// A track or stadium: the centre line `a`–`b` with round caps.
    pub fn stroke_segment(&mut self, a: (f64, f64), b: (f64, f64), width_mm: f64, color: Rgba) {
        if a == b {
            self.fill_circle(a.0, a.1, width_mm / 2.0, color);
            return;
        }
        let w = self.mm_to_px(width_mm).max(0.75);
        self.stroke_px(&[a, b], false, w, color, None);
    }

    /// A polyline whose width is in pixels (outlines, grid, overlays),
    /// optionally dashed (`dash` in pixels: on, off).
    pub fn stroke_px(
        &mut self,
        pts: &[(f64, f64)],
        closed: bool,
        width_px: f32,
        color: Rgba,
        dash: Option<(f32, f32)>,
    ) {
        if pts.len() < 2 {
            return;
        }
        let mut pb = PathBuilder::new();
        let (x, y) = self.to_px(pts[0].0, pts[0].1);
        pb.move_to(x, y);
        for &(x, y) in &pts[1..] {
            let (px, py) = self.to_px(x, y);
            pb.line_to(px, py);
        }
        if closed {
            pb.close();
        }
        let Some(path) = pb.finish() else { return };
        let stroke = Stroke {
            width: width_px,
            line_cap: LineCap::Round,
            line_join: LineJoin::Round,
            dash: dash.and_then(|(on, off)| StrokeDash::new(vec![on, off], 0.0)),
            ..Stroke::default()
        };
        self.pixmap
            .stroke_path(&path, &color.paint(), &stroke, Transform::identity(), None);
    }

    /// A circle outline `width_px` wide.
    pub fn stroke_circle(&mut self, x: f64, y: f64, radius_mm: f64, width_px: f32, color: Rgba) {
        let (px, py) = self.to_px(x, y);
        let r = self.mm_to_px(radius_mm).max(0.5);
        if let Some(path) = PathBuilder::from_circle(px, py, r) {
            let stroke = Stroke {
                width: width_px,
                ..Stroke::default()
            };
            self.pixmap
                .stroke_path(&path, &color.paint(), &stroke, Transform::identity(), None);
        }
    }

    /// A line in pixel coordinates (grid and frame furniture).
    pub fn line_px(&mut self, a: (f32, f32), b: (f32, f32), width_px: f32, color: Rgba) {
        let mut pb = PathBuilder::new();
        pb.move_to(a.0, a.1);
        pb.line_to(b.0, b.1);
        if let Some(path) = pb.finish() {
            let stroke = Stroke {
                width: width_px,
                ..Stroke::default()
            };
            self.pixmap
                .stroke_path(&path, &color.paint(), &stroke, Transform::identity(), None);
        }
    }

    /// Text `height_px` tall at pixel position `at`, in `color` over a
    /// `halo` outline for legibility on copper.
    pub fn text_px(
        &mut self,
        at: (f32, f32),
        anchor: Anchor,
        height_px: f32,
        text: &str,
        color: Rgba,
        halo: Option<Rgba>,
    ) {
        if text.is_empty() || height_px <= 0.0 {
            return;
        }
        let unit = height_px / font::HEIGHT;
        let width = font::text_width(text) * unit;
        let (ox, oy) = match anchor {
            Anchor::Center => (at.0 - width / 2.0, at.1 - height_px / 2.0),
            Anchor::Left => (at.0, at.1 - height_px / 2.0),
            Anchor::TopLeft => (at.0, at.1),
        };
        let mut pb = PathBuilder::new();
        for (i, c) in text.chars().enumerate() {
            let Some(strokes) = font::glyph(c) else {
                continue;
            };
            let cx = ox + i as f32 * font::ADVANCE * unit;
            for s in strokes {
                let mut first = true;
                for &(gx, gy) in s.iter() {
                    let (x, y) = (cx + gx * unit, oy + gy * unit);
                    if first {
                        pb.move_to(x, y);
                        first = false;
                    } else {
                        pb.line_to(x, y);
                    }
                }
            }
        }
        let Some(path) = pb.finish() else { return };
        let w = (height_px / 7.0).max(1.0);
        if let Some(h) = halo {
            let stroke = Stroke {
                width: w + 2.5,
                line_cap: LineCap::Round,
                line_join: LineJoin::Round,
                ..Stroke::default()
            };
            self.pixmap
                .stroke_path(&path, &h.paint(), &stroke, Transform::identity(), None);
        }
        let stroke = Stroke {
            width: w,
            line_cap: LineCap::Round,
            line_join: LineJoin::Round,
            ..Stroke::default()
        };
        self.pixmap
            .stroke_path(&path, &color.paint(), &stroke, Transform::identity(), None);
    }

    /// Text at a board position `(x, y)` in mm.
    pub fn text_mm(
        &mut self,
        (x, y): (f64, f64),
        anchor: Anchor,
        height_px: f32,
        text: &str,
        color: Rgba,
        halo: Option<Rgba>,
    ) {
        let at = self.to_px(x, y);
        self.text_px(at, anchor, height_px, text, color, halo);
    }

    /// The width a label of `height_px` would take, px.
    pub fn text_width_px(text: &str, height_px: f32) -> f32 {
        font::text_width(text) * height_px / font::HEIGHT
    }

    pub fn encode_png(&self) -> Result<Vec<u8>> {
        self.pixmap.encode_png().context("PNG encoding failed")
    }

    /// The RGBA bytes of one pixel (tests and probes).
    pub fn pixel(&self, x: u32, y: u32) -> Option<[u8; 4]> {
        let p = self.pixmap.pixel(x, y)?;
        let c = p.demultiply();
        Some([c.red(), c.green(), c.blue(), c.alpha()])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const BG: Rgba = Rgba(0, 0, 0, 255);
    const RED: Rgba = Rgba(255, 0, 0, 255);

    #[test]
    fn millimetres_map_onto_the_window() {
        let mut c = Canvas::new(10.0, 20.0, 12.0, 21.0, 50.0, BG).unwrap();
        assert_eq!((c.width_px(), c.height_px()), (100, 50));
        c.fill_circle(11.0, 20.5, 0.2, RED);
        assert_eq!(c.pixel(50, 25), Some([255, 0, 0, 255]));
        assert_eq!(c.pixel(5, 5), Some([0, 0, 0, 255]));
    }

    #[test]
    fn holes_cut_their_polygon() {
        let mut c = Canvas::new(0.0, 0.0, 10.0, 10.0, 10.0, BG).unwrap();
        let outer = vec![(1.0, 1.0), (9.0, 1.0), (9.0, 9.0), (1.0, 9.0)];
        let hole = vec![(4.0, 4.0), (6.0, 4.0), (6.0, 6.0), (4.0, 6.0)];
        c.fill_polygon(&[outer, hole], RED);
        assert_eq!(c.pixel(20, 20), Some([255, 0, 0, 255]));
        assert_eq!(c.pixel(50, 50), Some([0, 0, 0, 255]), "inside the hole");
    }

    #[test]
    fn a_stadium_has_round_caps() {
        let mut c = Canvas::new(0.0, 0.0, 10.0, 4.0, 10.0, BG).unwrap();
        c.stroke_segment((3.0, 2.0), (7.0, 2.0), 2.0, RED);
        // Past the end by less than the half width, on the centre line.
        assert_eq!(c.pixel(75, 20), Some([255, 0, 0, 255]));
        // Diagonally off the end beyond the cap's radius.
        assert_eq!(c.pixel(79, 29), Some([0, 0, 0, 255]));
    }

    #[test]
    fn text_draws_strokes_and_oversized_windows_are_refused() {
        let mut c = Canvas::new(0.0, 0.0, 20.0, 10.0, 10.0, BG).unwrap();
        c.text_px((100.0, 50.0), Anchor::Center, 20.0, "U17", RED, None);
        let lit = (0..200)
            .flat_map(|x| (0..100).map(move |y| (x, y)))
            .filter(|&(x, y)| c.pixel(x, y).is_some_and(|p| p[0] > 128))
            .count();
        assert!(lit > 50, "{lit}");
        assert!(Canvas::size_for(0.0, 0.0, 400.0, 300.0, 100.0).is_err());
        assert!(Canvas::new(0.0, 0.0, 1.0, 1.0, 0.0, BG).is_err());
    }
}
