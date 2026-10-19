//! The tile schedule: a regular grid of cores
//! ([`NegotiationTuning::grid_core`], by default [`crate::api::TILE_CORE_NM`]) with
//! [`TILE_HALO_NM`] halos, 4-coloured so same-colour tiles never share a
//! halo, shifted by half a tile on odd iterations; and fine-pitch BGA fields
//! carved out as sub-tiles on the fine lattice, scheduled first.
//!
//! Every tile's lattice is aligned to one board-wide grid of
//! [`LATTICE_PITCH_NM`], so a node on a shared core boundary exists in both
//! neighbours' lattices (the fine pitch divides the base pitch). That is
//! what lets a connection that crosses tiles be pinned at a boundary node
//! both pieces reach exactly.

use super::{NegotiationTuning, Tile};
use crate::api::{FootprintId, TileId, LATTICE_PITCH_NM, TILE_HALO_NM};
use crate::board::RouteBoard;
use crate::compute::Window;
use konnect_geom::{BBox, Coord, Point};
use serde::{Deserialize, Serialize};

/// A fine-pitch ball field routed on sub-tiles before the regular grid.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BgaField {
    pub footprint: FootprintId,
    /// The field plus its escape ring, snapped out to the base lattice.
    pub region: BBox,
    /// The smallest centre-to-centre distance between the field's pads.
    pub pitch: Coord,
    pub pads: u32,
}

/// One scheduled tile with the derived data the run needs.
#[derive(Debug, Clone)]
pub(crate) struct TileSpec {
    pub tile: Tile,
    /// Where this tile may commit copper: the window less a context margin,
    /// so everything within clearance of committed copper is in view.
    pub commit: BBox,
    /// The uniform lattice over the window.
    pub lattice: Window,
}

/// One iteration's tiles: BGA sub-tiles first, then the regular grid.
#[derive(Debug, Clone)]
pub(crate) struct Tiling {
    pub tiles: Vec<TileSpec>,
    pub fields: Vec<BgaField>,
    bga_tiles: usize,
    origin: Point,
    /// Grid core width and height.
    tw: Coord,
    th: Coord,
    nx: u32,
    ny: u32,
}

pub(crate) fn snap_down(v: Coord, step: Coord) -> Coord {
    (i64::from(v).div_euclid(i64::from(step)) * i64::from(step)) as Coord
}

pub(crate) fn snap_up(v: Coord, step: Coord) -> Coord {
    let s = i64::from(step);
    ((i64::from(v) + s - 1).div_euclid(s) * s) as Coord
}

/// The board's extent: the outline box, else the pads' union.
pub(crate) fn board_bbox(board: &RouteBoard) -> Option<BBox> {
    board.outline.bbox.or_else(|| {
        board
            .pads
            .iter()
            .filter_map(|p| p.bbox())
            .reduce(BBox::union)
    })
}

/// The grid core size: the tuning's, snapped to the base lattice and kept
/// above two halos plus 0.5 mm so same-colour windows stay disjoint.
fn grid_core(tuning: &NegotiationTuning) -> (Coord, Coord) {
    let min = 2 * TILE_HALO_NM + 500_000;
    let side = |v: Coord| snap_up(v.max(min), LATTICE_PITCH_NM);
    (side(tuning.grid_core.0), side(tuning.grid_core.1))
}

/// Half-open containment, so a point on a shared boundary has one owner.
fn owns(core: &BBox, p: Point) -> bool {
    p.x >= core.min.x && p.x < core.max.x && p.y >= core.min.y && p.y < core.max.y
}

/// Footprints whose pads form a fine-pitch 2-D array.
pub fn detect_bga_fields(board: &RouteBoard, tuning: &NegotiationTuning) -> Vec<BgaField> {
    let mut out = Vec::new();
    for f in &board.footprints {
        let pads = &board.pads[f.pad_range.0 as usize..f.pad_range.1 as usize];
        if pads.len() < tuning.bga_min_pads as usize {
            continue;
        }
        let mut xs: Vec<Coord> = pads.iter().map(|p| p.center.x).collect();
        let mut ys: Vec<Coord> = pads.iter().map(|p| p.center.y).collect();
        xs.sort_unstable();
        xs.dedup();
        ys.sort_unstable();
        ys.dedup();
        if xs.len() < tuning.bga_min_rows as usize || ys.len() < tuning.bga_min_rows as usize {
            continue;
        }
        // An array fills its grid; a perimeter package only its edges.
        if (pads.len() as u64) * 1000
            < u64::from(tuning.bga_min_fill) * (xs.len() * ys.len()) as u64
        {
            continue;
        }
        // Smallest neighbour distance, from sorted pad centres (a sweep in
        // x with an early break keeps it near-linear).
        let mut centres: Vec<Point> = pads.iter().map(|p| p.center).collect();
        centres.sort_unstable();
        let mut best = i128::MAX;
        for i in 0..centres.len() {
            for j in i + 1..centres.len() {
                let dx = i128::from(centres[j].x) - i128::from(centres[i].x);
                if dx * dx >= best {
                    break;
                }
                let d = (centres[j] - centres[i]).length2();
                if d > 0 && d < best {
                    best = d;
                }
            }
        }
        if best == i128::MAX {
            continue;
        }
        let pitch = (best as f64).sqrt().round() as Coord;
        if pitch > tuning.bga_max_pitch {
            continue;
        }
        let Some(extent) = pads.iter().filter_map(|p| p.bbox()).reduce(BBox::union) else {
            continue;
        };
        let grown = extent.inflate(tuning.bga_margin);
        let region = BBox::new(
            Point::new(
                snap_down(grown.min.x, LATTICE_PITCH_NM),
                snap_down(grown.min.y, LATTICE_PITCH_NM),
            ),
            Point::new(
                snap_up(grown.max.x, LATTICE_PITCH_NM),
                snap_up(grown.max.y, LATTICE_PITCH_NM),
            ),
        );
        out.push(BgaField {
            footprint: f.id,
            region,
            pitch,
            pads: pads.len() as u32,
        });
    }
    out
}

impl Tiling {
    /// The tiles for one iteration. `margin` is the context a commit needs
    /// inside the window (clearance plus the widest copper).
    pub fn build(
        board: &RouteBoard,
        shift: bool,
        tuning: &NegotiationTuning,
        margin: Coord,
    ) -> Option<Tiling> {
        let bb = board_bbox(board)?;
        let fields = detect_bga_fields(board, tuning);
        let mut tiles: Vec<TileSpec> = Vec::new();
        let spec = |id: usize, core: BBox, colour: u8, phase: u8, pitch: Coord| {
            let window = core.inflate(TILE_HALO_NM);
            let cols = (window.width() / i64::from(pitch)) as u32 + 1;
            let rows = (window.height() / i64::from(pitch)) as u32 + 1;
            let lattice = Window::new(window.min, pitch, cols, rows)?;
            Some(TileSpec {
                tile: Tile {
                    id: TileId(id as u32),
                    core,
                    window,
                    colour,
                    phase,
                    pitch,
                },
                commit: window.inflate(-margin),
                lattice,
            })
        };

        // BGA sub-tiles, coloured greedily (2×2 pattern preferred) so that no
        // two of one colour share a window, even across nearby fields.
        let sub_max = tuning.bga_sub_tile_max.max(LATTICE_PITCH_NM);
        for field in &fields {
            let r = field.region;
            // At most `sub_max` per side, but never below two halos (plus a
            // margin), so sub-tiles two apart have disjoint windows and the
            // field's sub-tiles run four colours deep, not one chain.
            let min_core = 2 * i64::from(TILE_HALO_NM) + 500_000;
            let split = |len: i64| {
                let want = (len + i64::from(sub_max) - 1) / i64::from(sub_max);
                want.min(len / min_core).max(2)
            };
            let (nsx, nsy) = (split(r.width()), split(r.height()));
            let cut = |lo: Coord, len: i64, n: i64, k: i64| -> Coord {
                if k == n {
                    return (i64::from(lo) + len) as Coord;
                }
                snap_down((i64::from(lo) + len * k / n) as Coord, LATTICE_PITCH_NM)
            };
            for sy in 0..nsy {
                for sx in 0..nsx {
                    let core = BBox::new(
                        Point::new(
                            cut(r.min.x, r.width(), nsx, sx),
                            cut(r.min.y, r.height(), nsy, sy),
                        ),
                        Point::new(
                            cut(r.min.x, r.width(), nsx, sx + 1),
                            cut(r.min.y, r.height(), nsy, sy + 1),
                        ),
                    );
                    let window = core.inflate(TILE_HALO_NM);
                    let preferred = ((sx & 1) + 2 * (sy & 1)) as u8;
                    let taken: Vec<u8> = tiles
                        .iter()
                        .filter(|t| t.tile.window.intersects(&window))
                        .map(|t| t.tile.colour)
                        .collect();
                    let colour = if taken.contains(&preferred) {
                        (0u8..=u8::MAX)
                            .find(|c| !taken.contains(c))
                            .expect("fewer than 256 neighbours")
                    } else {
                        preferred
                    };
                    let id = tiles.len();
                    tiles.push(spec(id, core, colour, 0, tuning.fine_pitch)?);
                }
            }
        }
        let bga_tiles = tiles.len();

        // The regular grid, aligned to the base lattice, shifted by half a
        // tile on odd iterations.
        let (tw, th) = grid_core(tuning);
        let mut origin = Point::new(
            snap_down(bb.min.x, LATTICE_PITCH_NM),
            snap_down(bb.min.y, LATTICE_PITCH_NM),
        );
        if shift {
            // Half a tile, kept on the base lattice.
            origin = Point::new(
                origin.x - snap_down(tw / 2, LATTICE_PITCH_NM),
                origin.y - snap_down(th / 2, LATTICE_PITCH_NM),
            );
        }
        let nx = ((i64::from(bb.max.x) - i64::from(origin.x)) / i64::from(tw) + 1) as u32;
        let ny = ((i64::from(bb.max.y) - i64::from(origin.y)) / i64::from(th) + 1) as u32;
        for iy in 0..ny {
            for ix in 0..nx {
                let min = Point::new(origin.x + (ix as Coord) * tw, origin.y + (iy as Coord) * th);
                let core = BBox::new(min, Point::new(min.x + tw, min.y + th));
                let colour = ((ix & 1) + 2 * (iy & 1)) as u8;
                let id = tiles.len();
                tiles.push(spec(id, core, colour, 1, tuning.base_pitch)?);
            }
        }
        Some(Tiling {
            tiles,
            fields,
            bga_tiles,
            origin,
            tw,
            th,
            nx,
            ny,
        })
    }

    /// The tile that owns `p`: a BGA sub-tile if one's core holds it, else
    /// the grid tile (clamped to the grid).
    pub fn owner(&self, p: Point) -> usize {
        if let Some(i) = self.tiles[..self.bga_tiles]
            .iter()
            .position(|t| owns(&t.tile.core, p))
        {
            return i;
        }
        let (tw, th) = (self.tw, self.th);
        let ix = ((i64::from(p.x) - i64::from(self.origin.x)).div_euclid(i64::from(tw)))
            .clamp(0, i64::from(self.nx) - 1);
        let iy = ((i64::from(p.y) - i64::from(self.origin.y)).div_euclid(i64::from(th)))
            .clamp(0, i64::from(self.ny) - 1);
        self.bga_tiles + (iy as usize) * self.nx as usize + ix as usize
    }

    /// Tile indices grouped into passes: BGA sub-tiles by colour, then the
    /// grid by colour. Tiles of one pass have pairwise disjoint windows.
    pub fn passes(&self) -> Vec<Vec<usize>> {
        let mut keys: Vec<(u8, u8)> = self
            .tiles
            .iter()
            .map(|t| (t.tile.phase, t.tile.colour))
            .collect();
        keys.sort_unstable();
        keys.dedup();
        keys.into_iter()
            .map(|(phase, colour)| {
                (0..self.tiles.len())
                    .filter(|&i| {
                        self.tiles[i].tile.phase == phase && self.tiles[i].tile.colour == colour
                    })
                    .collect()
            })
            .collect()
    }
}
