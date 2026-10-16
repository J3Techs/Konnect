//! Bounded parking for the footprints a sync adds.
//!
//! This is not placement. Added footprints need *some* position before a
//! person or a later tool places them, and the old staging put every one in a
//! single column below the board's top edge: a 2,466-part import is metres
//! tall, past the ±2147.48 mm KiCad can hold in its signed 32-bit nanometre
//! coordinates. Parking packs the footprints' conservative boxes into shelves
//! of a bounded region beside the board, keeps them apart by a margin, and
//! refuses before anything is written when even that region would leave the
//! coordinate range. It never chooses a functional location, never rotates,
//! and never routes.

use serde::Serialize;

use super::{Bounds, Point};

/// KiCad stores board coordinates as signed 32-bit nanometres, ±2147.48 mm.
/// Parking stays inside a smaller range so later moves and KiCad's own box
/// arithmetic keep headroom.
pub(super) const KICAD_COORDINATE_LIMIT_MM: f64 = 2000.0;
/// Gap between the board's measured extent and the parking region.
pub(super) const BOARD_GAP_MM: f64 = 5.0;
/// Clearance between neighbouring parked boxes.
pub(super) const PARKING_MARGIN_MM: f64 = 1.0;

/// A footprint's box in its own frame, before it is placed.
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub(super) struct LocalBox {
    pub(super) min_x: f64,
    pub(super) min_y: f64,
    pub(super) max_x: f64,
    pub(super) max_y: f64,
}

impl LocalBox {
    pub(super) fn from_bbox((min_x, min_y, max_x, max_y): konnect_sexp::bounds::Bbox) -> Self {
        Self {
            min_x,
            min_y,
            max_x,
            max_y,
        }
    }

    pub(super) fn width(&self) -> f64 {
        self.max_x - self.min_x
    }

    pub(super) fn height(&self) -> f64 {
        self.max_y - self.min_y
    }

    #[cfg(test)]
    /// The board-space box this footprint occupies with its anchor at
    /// `position`, unrotated.
    pub(super) fn at(&self, position: Point) -> Bounds {
        Bounds {
            min_x: position.x + self.min_x,
            min_y: position.y + self.min_y,
            max_x: position.x + self.max_x,
            max_y: position.y + self.max_y,
        }
    }
}

/// What the parked positions were computed against, for the response.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub(super) struct ParkingSummary {
    pub(super) parked: usize,
    pub(super) rows: usize,
    /// The region every parked box lies inside; `None` when nothing was parked.
    pub(super) region: Option<Bounds>,
    pub(super) board_gap_mm: f64,
    pub(super) margin_mm: f64,
    pub(super) coordinate_limit_mm: f64,
}

/// Anchor positions for `boxes`, in their order, packed into shelves to the
/// right of `board`.
///
/// Shelf order is by height, then width, then input order, so the same boxes
/// always park the same way. The region's width is about the square root of
/// the boxes' total area, never narrower than the widest box. An `Err` names
/// the extent that would have been needed; nothing is parked then.
pub(super) fn park(
    boxes: &[LocalBox],
    board: Bounds,
) -> Result<(Vec<Point>, ParkingSummary), String> {
    let mut summary = ParkingSummary {
        parked: boxes.len(),
        rows: 0,
        region: None,
        board_gap_mm: BOARD_GAP_MM,
        margin_mm: PARKING_MARGIN_MM,
        coordinate_limit_mm: KICAD_COORDINATE_LIMIT_MM,
    };
    if boxes.is_empty() {
        return Ok((Vec::new(), summary));
    }
    for (index, item) in boxes.iter().enumerate() {
        let values = [item.min_x, item.min_y, item.max_x, item.max_y];
        if values.iter().any(|value| !value.is_finite())
            || item.width() < 0.0
            || item.height() < 0.0
        {
            return Err(format!("footprint box {index} is not a finite box"));
        }
    }
    let board_values = [board.min_x, board.min_y, board.max_x, board.max_y];
    if board_values.iter().any(|value| !value.is_finite()) {
        return Err("the board extent is not finite".to_string());
    }

    let margin = PARKING_MARGIN_MM;
    let area: f64 = boxes
        .iter()
        .map(|item| (item.width() + margin) * (item.height() + margin))
        .sum();
    let widest = boxes.iter().map(LocalBox::width).fold(0.0, f64::max);
    let row_width = widest.max(area.sqrt() * 1.15);

    let mut order = (0..boxes.len()).collect::<Vec<_>>();
    order.sort_by(|&a, &b| {
        let (a_box, b_box) = (&boxes[a], &boxes[b]);
        b_box
            .height()
            .total_cmp(&a_box.height())
            .then(b_box.width().total_cmp(&a_box.width()))
            .then(a.cmp(&b))
    });

    let origin_x = board.max_x + BOARD_GAP_MM;
    let origin_y = board.min_y;
    let mut positions = vec![Point { x: 0.0, y: 0.0 }; boxes.len()];
    let (mut cursor_x, mut cursor_y, mut shelf_height) = (0.0f64, 0.0f64, 0.0f64);
    let mut used_width = 0.0f64;
    let mut rows = 1usize;
    for index in order {
        let item = &boxes[index];
        if cursor_x > 0.0 && cursor_x + item.width() > row_width {
            cursor_y += shelf_height + margin;
            cursor_x = 0.0;
            shelf_height = 0.0;
            rows += 1;
        }
        positions[index] = Point {
            x: origin_x + cursor_x - item.min_x,
            y: origin_y + cursor_y - item.min_y,
        };
        used_width = used_width.max(cursor_x + item.width());
        cursor_x += item.width() + margin;
        shelf_height = shelf_height.max(item.height());
    }
    let region = Bounds {
        min_x: origin_x,
        min_y: origin_y,
        max_x: origin_x + used_width,
        max_y: origin_y + cursor_y + shelf_height,
    };
    let limit = KICAD_COORDINATE_LIMIT_MM;
    let outside = [region.min_x, region.min_y, region.max_x, region.max_y]
        .iter()
        .any(|value| value.abs() > limit);
    if outside {
        return Err(format!(
            "parking {} footprints beside the board needs the region x {:.3}..{:.3} mm, \
             y {:.3}..{:.3} mm, which leaves the ±{limit} mm range Konnect keeps inside \
             KiCad's signed 32-bit nanometre coordinates",
            boxes.len(),
            region.min_x,
            region.max_x,
            region.min_y,
            region.max_y
        ));
    }
    summary.rows = rows;
    summary.region = Some(region);
    Ok((positions, summary))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn overlaps(a: &Bounds, b: &Bounds) -> bool {
        a.min_x < b.max_x && b.min_x < a.max_x && a.min_y < b.max_y && b.min_y < a.max_y
    }

    fn board() -> Bounds {
        Bounds {
            min_x: 0.0,
            min_y: 0.0,
            max_x: 400.0,
            max_y: 300.0,
        }
    }

    /// A DB1-sized import: 2,466 parts of mixed sizes, off-centre origins
    /// included, with a few large connectors. The old single column needed
    /// metres; parking must stay bounded, inside KiCad's range, beside the
    /// board and with no two boxes touching.
    #[test]
    fn a_2466_part_import_parks_bounded_and_without_overlap() {
        let mut boxes = Vec::new();
        for index in 0..2466usize {
            // DB1-like: five 80 mm connectors, a few dozen BGAs and QFNs, a
            // few hundred mid-size parts, the rest passives.
            let (w, h) = match index {
                0..=4 => (81.6, 53.5),
                _ if index % 60 == 0 => (17.5, 17.5),
                _ if index % 25 == 0 => (5.2, 4.1),
                _ if index % 7 == 0 => (3.0, 1.5),
                _ => (2.0, 1.2),
            };
            // An anchor that is not the box centre, as many footprints have.
            let (ox, oy) = ((index % 7) as f64 * 0.3, (index % 5) as f64 * -0.2);
            boxes.push(LocalBox {
                min_x: -w / 2.0 + ox,
                min_y: -h / 2.0 + oy,
                max_x: w / 2.0 + ox,
                max_y: h / 2.0 + oy,
            });
        }
        let (positions, summary) = park(&boxes, board()).expect("parks");
        let region = summary.region.unwrap();
        assert!(region.min_x >= board().max_x + BOARD_GAP_MM - 1e-9);
        let placed = boxes
            .iter()
            .zip(&positions)
            .map(|(item, position)| item.at(*position))
            .collect::<Vec<_>>();
        for occupied in &placed {
            for value in [
                occupied.min_x,
                occupied.min_y,
                occupied.max_x,
                occupied.max_y,
            ] {
                assert!(value.abs() < KICAD_COORDINATE_LIMIT_MM, "{occupied:?}");
            }
            assert!(occupied.min_x >= region.min_x - 1e-9 && occupied.max_x <= region.max_x + 1e-9);
            assert!(occupied.min_y >= region.min_y - 1e-9 && occupied.max_y <= region.max_y + 1e-9);
            assert!(!overlaps(occupied, &board()), "parked on the board");
        }
        // Sweep for overlaps instead of comparing all pairs.
        let mut by_x = (0..placed.len()).collect::<Vec<_>>();
        by_x.sort_by(|&a, &b| placed[a].min_x.total_cmp(&placed[b].min_x));
        for (rank, &a) in by_x.iter().enumerate() {
            for &b in &by_x[rank + 1..] {
                if placed[b].min_x >= placed[a].max_x {
                    break;
                }
                assert!(!overlaps(&placed[a], &placed[b]), "{a} and {b} overlap");
            }
        }
        // Compact: far below a metre on each side, nowhere near the limit.
        assert!(region.max_x - region.min_x < 1000.0, "{region:?}");
        assert!(region.max_y - region.min_y < 1000.0, "{region:?}");
        assert!(summary.rows > 1);

        // Deterministic: the same boxes park the same way.
        assert_eq!(park(&boxes, board()).unwrap().0, positions);
    }

    #[test]
    fn an_extent_beyond_kicads_range_is_refused_before_anything_is_parked() {
        let far = Bounds {
            min_x: 0.0,
            min_y: 0.0,
            max_x: 1990.0,
            max_y: 10.0,
        };
        let boxes = vec![
            LocalBox {
                min_x: -10.0,
                min_y: -10.0,
                max_x: 10.0,
                max_y: 10.0,
            };
            4
        ];
        let error = park(&boxes, far).unwrap_err();
        assert!(error.contains("32-bit"), "{error}");
    }

    #[test]
    fn nothing_to_park_is_not_an_error() {
        let (positions, summary) = park(&[], board()).unwrap();
        assert!(positions.is_empty());
        assert_eq!(summary.region, None);
    }
}
