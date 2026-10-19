//! A VPR-style map lookahead: the exact minimum cost from a node to an
//! offset `(dx, dy)` and a target layer on an obstacle-free lattice, under
//! the search's own cost model (octile steps, bends, preferred-direction
//! penalties, vias, the turn policy).
//!
//! VPR precomputes such a table per wire type by sampling its routing graph
//! (`router_lookahead_map`); a PCB lattice is translation-invariant, so one
//! Dijkstra per source layer from a single origin gives the table exactly.
//! Obstacles, congestion, Near costs and corridor penalties only add cost,
//! so on a uniform lattice the table is an admissible, consistent-in-
//! practice heuristic that is never weaker than octile-plus-via: it sees
//! that a long vertical run on a horizontal-preferred layer is cheaper
//! through a via to a vertical layer. A suffix minimum makes it monotone in
//! `|dx|` and `|dy|`, so a lookup at the distance to a target box bounds
//! every target in it.

use super::kernel::{self, Mode, Problem, Region, Scratch};
use super::lattice::{derive_masks_uniform, UniformGrid, VIA_FREE};
use super::{Heuristic, Preferred, SearchConfig, SearchStats};
use crate::api::{LayerId, ProfileId, TileId};
use crate::compute::job::JOB_ALLOW_VIA;
use crate::compute::raster::{TileRaster, TileStack, Window};
use crate::compute::{Cost, CostParams, JobDescriptor, JobNode};
use konnect_geom::{Coord, Point};
use rayon::prelude::*;

/// Steps of margin around the origin, so optimal paths that wander
/// slightly behind the source stay inside the build lattice.
const MARGIN: u32 = 4;

/// What a table was built for; a search only uses a matching table.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Key {
    params: CostParams,
    pitch: Coord,
    layers: u8,
    prefs: Vec<Preferred>,
    bend_45: Cost,
    bend_90: Cost,
    min_angle_millideg: Option<i64>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MapLookahead {
    key: Key,
    /// Offsets `0..=max_dx` and `0..=max_dy`, in base steps.
    w: u32,
    h: u32,
    /// `((from · layers + to) · h + dy) · w + dx`.
    table: Vec<u32>,
}

fn key_of(params: &CostParams, config: &SearchConfig, pitch: Coord, layers: u8) -> Key {
    let mut prefs = config.layer_pref.clone();
    prefs.truncate(usize::from(layers));
    while prefs.last() == Some(&Preferred::None) {
        prefs.pop();
    }
    Key {
        params: *params,
        pitch,
        layers,
        prefs,
        bend_45: config.bend_45,
        bend_90: config.bend_90,
        min_angle_millideg: config
            .min_track_angle_deg
            .map(|a| (a * 1000.0).round() as i64),
    }
}

impl MapLookahead {
    /// Build the table for `layers` stack layers and offsets up to
    /// `max_dx × max_dy` base steps at `pitch`.
    pub fn build(
        layers: usize,
        max_dx: u32,
        max_dy: u32,
        pitch: Coord,
        params: &CostParams,
        config: &SearchConfig,
    ) -> Option<Self> {
        if layers == 0 || layers > 32 || pitch <= 0 {
            return None;
        }
        let w = max_dx + 1;
        let h = max_dy + 1;
        let cols = w + 2 * MARGIN;
        let rows = h + 2 * MARGIN;
        let window = Window::new(Point::new(0, 0), pitch, cols, rows)?;
        let mut rasters = Vec::with_capacity(layers);
        for l in 0..layers {
            let mut r = TileRaster::new(TileId(0), LayerId(l as u8), ProfileId(0), window);
            derive_masks_uniform(&mut r, None);
            r.via_site.fill(VIA_FREE);
            rasters.push(r);
        }
        let stack = TileStack {
            tile: TileId(0),
            profile: ProfileId(0),
            window,
            layers: rasters,
        };
        let mut cfg = config.clone();
        cfg.heuristic = Heuristic::Octile;
        cfg.astar_factor = 1000;
        cfg.max_expansions = u64::MAX;
        cfg.check_free_vias = false;
        cfg.use_heading_mask = true;
        let job = JobDescriptor {
            flags: JOB_ALLOW_VIA,
            cost_bound: u32::MAX,
            ..Default::default()
        };
        let grid = UniformGrid::new(window);
        let region = Region::full(&grid);
        let per_layer: Vec<Vec<Cost>> = (0..layers)
            .into_par_iter()
            .map(|lf| {
                let src = [JobNode {
                    col: MARGIN as u16,
                    row: MARGIN as u16,
                    layer: lf as u8,
                    heading_mask: 0xff,
                    reserved: 0,
                }];
                let p = Problem {
                    stack: &stack,
                    job: &job,
                    job_index: 0,
                    sources: &src,
                    targets: &[],
                    corridor: &[],
                    params,
                    config: &cfg,
                    checker: None,
                    lookahead: None,
                    gcells: None,
                    mode: Mode::Field,
                };
                let mut scratch = Scratch::default();
                let mut stats = SearchStats::default();
                kernel::run(&grid, &p, region, &mut scratch, &mut stats);
                let mut out = Vec::new();
                kernel::field_costs(&scratch, region, layers as u32, 1, &mut out);
                out
            })
            .collect();
        let n = layers;
        let mut table = vec![u32::MAX; n * n * (w * h) as usize];
        let nodes = (cols * rows) as usize;
        for (lf, field) in per_layer.iter().enumerate() {
            for lt in 0..n {
                let base = (lf * n + lt) * (w * h) as usize;
                for dy in 0..h {
                    for dx in 0..w {
                        // Row increases with dy; the cost model is mirror
                        // symmetric, so the quadrant stands for all four.
                        let node = ((MARGIN + dy) * cols + MARGIN + dx) as usize;
                        table[base + (dy * w + dx) as usize] = field[lt * nodes + node].0;
                    }
                }
                // Suffix minimum: T(dx, dy) = min over dx' ≥ dx, dy' ≥ dy.
                for dy in (0..h).rev() {
                    for dx in (0..w).rev() {
                        let i = base + (dy * w + dx) as usize;
                        let mut v = table[i];
                        if dx + 1 < w {
                            v = v.min(table[i + 1]);
                        }
                        if dy + 1 < h {
                            v = v.min(table[i + w as usize]);
                        }
                        table[i] = v;
                    }
                }
            }
        }
        Some(MapLookahead {
            key: key_of(params, config, pitch, layers as u8),
            w,
            h,
            table,
        })
    }

    /// A table covering every offset inside `window`.
    pub fn for_window(
        window: &Window,
        layers: usize,
        params: &CostParams,
        config: &SearchConfig,
    ) -> Option<Self> {
        Self::build(
            layers,
            window.cols.saturating_sub(1),
            window.rows.saturating_sub(1),
            window.pitch_nm,
            params,
            config,
        )
    }

    /// Whether this table was built for the given cost model.
    pub fn matches(
        &self,
        params: &CostParams,
        config: &SearchConfig,
        pitch: Coord,
        layers: usize,
    ) -> bool {
        layers <= usize::from(u8::MAX) && self.key == key_of(params, config, pitch, layers as u8)
    }

    pub fn max_offset(&self) -> (u32, u32) {
        (self.w - 1, self.h - 1)
    }

    /// The bound from layer `from` to an offset of `dx × dy` base steps on
    /// layer `to`; `None` outside the table.
    #[inline]
    pub fn get(&self, from: u8, to: u8, dx: i64, dy: i64) -> Option<u32> {
        let n = u32::from(self.key.layers);
        if dx < 0 || dy < 0 || dx >= i64::from(self.w) || dy >= i64::from(self.h) {
            return None;
        }
        if u32::from(from) >= n || u32::from(to) >= n {
            return None;
        }
        let i = ((u32::from(from) * n + u32::from(to)) * self.h + dy as u32) * self.w + dx as u32;
        let v = self.table[i as usize];
        (v != u32::MAX).then_some(v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_lattice_table_is_octile_plus_via_without_penalties() {
        let config = SearchConfig {
            bend_45: Cost(0),
            bend_90: Cost(0),
            ..SearchConfig::default()
        };
        let params = CostParams::default();
        let la = MapLookahead::build(2, 20, 12, 50_000, &params, &config).unwrap();
        for (dx, dy) in [(0, 0), (5, 0), (0, 7), (6, 3), (20, 12)] {
            let oct = 1000 * dx.max(dy) as u32 + 414 * dx.min(dy) as u32;
            assert_eq!(la.get(0, 0, dx, dy), Some(oct), "({dx}, {dy})");
            assert_eq!(
                la.get(0, 1, dx, dy),
                Some(oct + params.via.0),
                "({dx}, {dy})"
            );
        }
        assert_eq!(la.get(0, 0, 21, 0), None);
        assert!(la.matches(&params, &config, 50_000, 2));
        assert!(!la.matches(&params, &SearchConfig::default(), 50_000, 2));
    }

    #[test]
    fn preferred_directions_make_the_table_see_through_vias() {
        let config = SearchConfig {
            layer_pref: vec![Preferred::Horizontal, Preferred::Vertical],
            ..SearchConfig::default()
        };
        let params = CostParams {
            wrong_way_step: Cost(1_000),
            ..CostParams::default()
        };
        let la = MapLookahead::build(2, 4, 40, 50_000, &params, &config).unwrap();
        // 40 steps vertically on the horizontal layer: 80k wrong-way, or
        // two vias (16k) plus 40k on the vertical layer.
        let v = la.get(0, 0, 0, 40).unwrap();
        assert_eq!(v, 40_000 + 2 * params.via.0);
        // Monotone after the suffix minimum.
        for dy in 0..40 {
            assert!(la.get(0, 0, 0, dy).unwrap() <= la.get(0, 0, 0, dy + 1).unwrap());
        }
    }
}
