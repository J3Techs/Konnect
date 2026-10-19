//! Structure-of-arrays rasters per (tile, layer, clearance profile).
//!
//! # Tri-state contract
//!
//! Every lattice node is classified from two counters ([`classify`]):
//!
//! - **Blocked** — `core > 0`: some obstacle forbids a track centre line on
//!   this node, decided exactly when the obstacle was painted.
//! - **Free** — `halo == 0`: no obstacle comes within its clearance plus
//!   half a cell diagonal. Every point of a lattice edge is within half a
//!   diagonal of one of its two nodes, so an edge between two Free nodes is
//!   legal with no further check.
//! - **Near** — otherwise: an edge touching this node may or may not be
//!   legal; the search must run (and may cache) an exact CPU check.
//!
//! CORE is painted with the obstacle inflated by clearance plus half the
//! profile's track width (plus the arc sagitta bound); HALO additionally by
//! the half-diagonal of a lattice cell, so CORE ⊆ HALO. Counters count
//! obstacles, so rip-up unpaints one obstacle without a rebuild. A counter
//! that reaches 255 sticks there (still blocked) until the raster is rebuilt.
//!
//! # Layout
//!
//! Each [`Plane`] is one flat `Vec` in row-major order whose row stride is
//! padded so every row starts on a 4-byte boundary. A GPU backend uploads a
//! plane as-is and addresses it as `u32` words.

use crate::api::{LayerId, ProfileId, TileId, GPU_WINDOW_MAX_NM};
use konnect_geom::{BBox, Coord, Point};

/// A uniform lattice window: node `(col, row)` sits at
/// `origin + (col, row) · pitch`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Window {
    pub origin: Point,
    pub pitch_nm: Coord,
    pub cols: u32,
    pub rows: u32,
}

impl Window {
    /// `None` for a non-positive pitch, an empty window, or one whose far
    /// corner leaves the `i32` range.
    pub fn new(origin: Point, pitch_nm: Coord, cols: u32, rows: u32) -> Option<Self> {
        if pitch_nm <= 0 || cols == 0 || rows == 0 {
            return None;
        }
        let w = Window {
            origin,
            pitch_nm,
            cols,
            rows,
        };
        let far = |o: Coord, n: u32| i64::from(o) + i64::from(pitch_nm) * (i64::from(n) - 1);
        let ok = Coord::try_from(far(origin.x, cols)).is_ok()
            && Coord::try_from(far(origin.y, rows)).is_ok();
        ok.then_some(w)
    }

    /// The node position. Callers pass `col < cols`, `row < rows`.
    pub fn node(&self, col: u32, row: u32) -> Point {
        let p = i64::from(self.pitch_nm);
        Point::new(
            (i64::from(self.origin.x) + p * i64::from(col)) as Coord,
            (i64::from(self.origin.y) + p * i64::from(row)) as Coord,
        )
    }

    /// The nearest node to `p`, if `p` lies within half a pitch of the
    /// window.
    pub fn nearest(&self, p: Point) -> Option<(u32, u32)> {
        let idx = |v: Coord, o: Coord, n: u32| {
            let rel = i64::from(v) - i64::from(o);
            let pitch = i64::from(self.pitch_nm);
            let i = (2 * rel + pitch).div_euclid(2 * pitch);
            (0..i64::from(n)).contains(&i).then_some(i as u32)
        };
        Some((
            idx(p.x, self.origin.x, self.cols)?,
            idx(p.y, self.origin.y, self.rows)?,
        ))
    }

    /// The span of node positions.
    pub fn bbox(&self) -> BBox {
        BBox::new(self.origin, self.node(self.cols - 1, self.rows - 1))
    }

    /// Whether the window satisfies the GPU's exact-`f32` extent limit.
    pub fn fits_gpu(&self) -> bool {
        let b = self.bbox();
        b.width() <= i64::from(GPU_WINDOW_MAX_NM) && b.height() <= i64::from(GPU_WINDOW_MAX_NM)
    }

    pub fn node_count(&self) -> usize {
        self.cols as usize * self.rows as usize
    }
}

/// One row-major plane of `T` with rows padded to a 4-byte multiple. `T`
/// must be 1, 2 or 4 bytes wide (`u8`, `u16`, `u32`), so whole elements fill
/// a padded row exactly; [`Plane::new`] refuses anything else at compile time.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Plane<T> {
    cols: u32,
    rows: u32,
    stride: u32,
    data: Vec<T>,
}

impl<T: Copy + Default> Plane<T> {
    pub fn new(cols: u32, rows: u32) -> Self {
        const { assert!(matches!(std::mem::size_of::<T>(), 1 | 2 | 4)) };
        let size = std::mem::size_of::<T>();
        let row_bytes = cols as usize * size;
        let padded = row_bytes.div_ceil(4) * 4;
        let stride = (padded / size) as u32;
        Plane {
            cols,
            rows,
            stride,
            data: vec![T::default(); stride as usize * rows as usize],
        }
    }

    pub fn cols(&self) -> u32 {
        self.cols
    }

    pub fn rows(&self) -> u32 {
        self.rows
    }

    /// Elements per row including padding.
    pub fn stride(&self) -> u32 {
        self.stride
    }

    fn offset(&self, col: u32, row: u32) -> usize {
        debug_assert!(col < self.cols && row < self.rows);
        row as usize * self.stride as usize + col as usize
    }

    pub fn get(&self, col: u32, row: u32) -> T {
        self.data[self.offset(col, row)]
    }

    pub fn set(&mut self, col: u32, row: u32, v: T) {
        let o = self.offset(col, row);
        self.data[o] = v;
    }

    pub fn get_mut(&mut self, col: u32, row: u32) -> &mut T {
        let o = self.offset(col, row);
        &mut self.data[o]
    }

    /// The whole padded buffer, for upload.
    pub fn as_slice(&self) -> &[T] {
        &self.data
    }

    /// The whole padded buffer, mutable: row `r` starts at `r * stride()`.
    /// For bulk writes (copying a cached plane, painting row bands).
    pub fn as_mut_slice(&mut self) -> &mut [T] {
        &mut self.data
    }

    pub fn fill(&mut self, v: T) {
        self.data.iter_mut().for_each(|x| *x = v);
    }
}

/// Node classification under the tri-state contract.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Occupancy {
    Free,
    Near,
    Blocked,
}

pub fn classify(core: u8, halo: u8) -> Occupancy {
    if core > 0 {
        Occupancy::Blocked
    } else if halo == 0 {
        Occupancy::Free
    } else {
        Occupancy::Near
    }
}

/// Every plane for one (tile, layer, profile).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TileRaster {
    pub tile: TileId,
    pub layer: LayerId,
    pub profile: ProfileId,
    pub window: Window,
    /// Hard obstacles forbidding a centre line on the node.
    pub core: Plane<u8>,
    /// Hard obstacles within reach of the node's outgoing edges.
    pub halo: Plane<u8>,
    /// Routed copper of other nets on the node (soft, PathFinder present).
    pub present: Plane<u16>,
    /// Accumulated overuse (PathFinder history).
    pub history: Plane<u16>,
    /// Bit `h` set: the edge in heading `h` (0 = +x, counter-clockwise in
    /// 45° steps, Y-up sense) may be legal.
    pub heading_mask: Plane<u8>,
    /// Non-zero: a via of this profile may be centred here on this layer.
    pub via_site: Plane<u8>,
}

impl TileRaster {
    pub fn new(tile: TileId, layer: LayerId, profile: ProfileId, window: Window) -> Self {
        let (c, r) = (window.cols, window.rows);
        TileRaster {
            tile,
            layer,
            profile,
            window,
            core: Plane::new(c, r),
            halo: Plane::new(c, r),
            present: Plane::new(c, r),
            history: Plane::new(c, r),
            heading_mask: Plane::new(c, r),
            via_site: Plane::new(c, r),
        }
    }

    pub fn occupancy(&self, col: u32, row: u32) -> Occupancy {
        classify(self.core.get(col, row), self.halo.get(col, row))
    }
}

/// One tile's rasters across the routable layers, sharing a window.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TileStack {
    pub tile: TileId,
    pub profile: ProfileId,
    pub window: Window,
    /// One raster per routable layer, in layer order.
    pub layers: Vec<TileRaster>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rows_are_padded_to_words() {
        let p: Plane<u8> = Plane::new(5, 3);
        assert_eq!(p.stride(), 8);
        assert_eq!(p.as_slice().len(), 24);
        let q: Plane<u16> = Plane::new(5, 3);
        assert_eq!(q.stride(), 6);
        assert_eq!(q.as_slice().len() * 2 % 4, 0);
        let r: Plane<u8> = Plane::new(8, 1);
        assert_eq!(r.stride(), 8);
    }

    #[test]
    fn tri_state_classification() {
        assert_eq!(classify(0, 0), Occupancy::Free);
        assert_eq!(classify(0, 3), Occupancy::Near);
        assert_eq!(classify(1, 3), Occupancy::Blocked);
    }

    #[test]
    fn window_nodes_and_nearest() {
        let w = Window::new(Point::new(1_000, 2_000), 50_000, 10, 4).unwrap();
        assert_eq!(w.node(2, 1), Point::new(101_000, 52_000));
        assert_eq!(w.nearest(Point::new(126_000, 52_000)), Some((3, 1)));
        assert_eq!(w.nearest(Point::new(124_000, 52_000)), Some((2, 1)));
        assert_eq!(w.nearest(Point::new(-30_000, 2_000)), None);
        assert!(w.fits_gpu());
        let big = Window::new(Point::new(0, 0), 50_000, 400, 10).unwrap();
        assert!(!big.fits_gpu());
        assert!(Window::new(Point::new(i32::MAX - 10, 0), 50_000, 2, 1).is_none());
    }
}
