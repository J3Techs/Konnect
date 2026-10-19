//! A VPR-style map lookahead: the cheapest base cost from any layer slot to
//! any other over an offset of `(dx, dy)` base cells, on an empty grid.
//!
//! Base costs are translation invariant (each layer has a fixed cost per mm
//! along and across its preferred direction, and a through via costs the
//! same everywhere, whichever slots it joins), so one table per (target
//! slot, source slot) covers the whole board. Monotone paths are optimal on
//! an empty grid, so a single dynamic-programming sweep per target slot
//! fills it. Present and history costs only add, so the table is a lower
//! bound on every real cost except a net's discounted reuse of its own
//! resources. The table may be smaller than the grid (a size budget):
//! larger offsets read its edge, which only weakens the bound.

use rayon::prelude::*;

#[derive(Debug, Clone)]
pub(crate) struct Lookahead {
    slots: usize,
    cols: usize,
    rows: usize,
    /// `[(t · slots + s) · rows + dy] · cols + dx`.
    table: Vec<f32>,
}

impl Lookahead {
    /// `h[s]`, `v[s]`: cost per base cell of a horizontal or vertical step on
    /// slot `s`; `via`: cost of one (through) via, whichever slots it joins.
    pub(crate) fn build(
        slots: usize,
        cols: usize,
        rows: usize,
        h: &[f32],
        v: &[f32],
        via: f32,
    ) -> Self {
        let plane = rows * cols;
        let per_target: Vec<Vec<f32>> = (0..slots)
            .into_par_iter()
            .map(|t| {
                let mut out = vec![0f32; slots * plane];
                let mut cur = vec![0f32; slots];
                for dy in 0..rows {
                    for dx in 0..cols {
                        for (s, c) in cur.iter_mut().enumerate() {
                            *c = if dx == 0 && dy == 0 {
                                if s == t {
                                    0.0
                                } else {
                                    via
                                }
                            } else {
                                let mut best = f32::INFINITY;
                                if dx > 0 {
                                    best = best.min(out[s * plane + dy * cols + dx - 1] + h[s]);
                                }
                                if dy > 0 {
                                    best = best.min(out[s * plane + (dy - 1) * cols + dx] + v[s]);
                                }
                                best
                            };
                        }
                        // A through via reaches any slot from any other.
                        let low = cur.iter().copied().fold(f32::INFINITY, f32::min);
                        for c in cur.iter_mut() {
                            *c = c.min(low + via);
                        }
                        for (s, c) in cur.iter().enumerate() {
                            out[s * plane + dy * cols + dx] = *c;
                        }
                    }
                }
                out
            })
            .collect();
        let mut table = Vec::with_capacity(slots * slots * plane);
        for t in per_target {
            table.extend(t);
        }
        Lookahead {
            slots,
            cols,
            rows,
            table,
        }
    }

    /// Lower bound from slot `s` to slot `t` over `(dx, dy)` base cells
    /// (clamped into the table).
    #[inline]
    pub(crate) fn get(&self, t: usize, s: usize, dx: usize, dy: usize) -> f32 {
        let dx = dx.min(self.cols - 1);
        let dy = dy.min(self.rows - 1);
        self.table[((t * self.slots + s) * self.rows + dy) * self.cols + dx]
    }

    /// The best bound to any slot of `targets` (a slot bitmask).
    #[inline]
    pub(crate) fn to_any(&self, targets: u32, s: usize, dx: usize, dy: usize) -> f32 {
        let mut best = f32::INFINITY;
        let mut m = targets;
        while m != 0 {
            let t = m.trailing_zeros() as usize;
            m &= m - 1;
            best = best.min(self.get(t, s, dx, dy));
        }
        best
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn table_prefers_preferred_directions_and_vias() {
        // Two slots: 0 horizontal (h 1, v 2), 1 vertical (h 2, v 1); via 1.5.
        let l = Lookahead::build(2, 20, 20, &[1.0, 2.0], &[2.0, 1.0], 1.5);
        assert_eq!(l.get(0, 0, 0, 0), 0.0);
        assert_eq!(l.get(0, 1, 0, 0), 1.5);
        assert_eq!(l.get(0, 0, 5, 0), 5.0);
        // 5 right then 5 down: stay on 0 (5 + 10 = 15) or switch twice
        // (5 + 1.5 + 5 + 1.5 = 13).
        assert_eq!(l.get(0, 0, 5, 5), 13.0);
        assert_eq!(l.to_any(0b11, 1, 0, 5), 5.0);
        // One through via joins any two slots.
        let l = Lookahead::build(4, 4, 4, &[1.0; 4], &[1.0; 4], 1.5);
        assert_eq!(l.get(3, 0, 0, 0), 1.5);
        assert_eq!(l.get(0, 3, 2, 0), 3.5);
    }
}
