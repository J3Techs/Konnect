//! Cutting connections into per-tile pieces, pinned where they cross tiles.
//!
//! A connection whose terminals (and corridor) fit inside one tile's commit
//! zone is one piece. Any other is walked along its guide line — the global
//! corridor's GCell centres when S3 supplied one, else the straight line
//! between its terminals — and cut wherever the owning tile changes. Each
//! cut gets a *pin*: a node of the board-wide base lattice on the shared
//! core boundary, on one routable layer, with an exact-legal track dot that
//! keeps clear of other nets' pins and (preferably) of routed copper. Both
//! neighbouring pieces must cross the boundary through the pin in the same
//! direction (within ±45°), so the joint never makes an acute angle.

use super::conflicts::{HistoryMap, RoutedIndex};
use super::tiles::Tiling;
use super::{Ctx, DetailFailure, Task};
use crate::api::{ItemKind, ItemRef, LayerId, NetId, LATTICE_PITCH_NM};
use crate::detail::search::HEADINGS;
use crate::rules::RuleSubject;
use konnect_geom::{BBox, Coord, Point, Segment, Shape};
use std::collections::HashMap;

/// One end of a piece.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Terminal {
    /// The connection's own terminal item.
    Item(ItemRef),
    /// A pin at a tile crossing.
    Pin(Pin),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Pin {
    pub at: Point,
    pub layer: LayerId,
    /// Headings a piece may enter (as target) or leave (as source) by.
    pub headings: u8,
}

/// One piece of one connection, queued in one tile.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct PieceJob {
    pub task: u32,
    pub piece: u8,
    pub from: Terminal,
    pub to: Terminal,
    /// Exact-check rejections so far this iteration.
    pub retries: u32,
}

/// A placed pin: where, whose, and its track width.
type PlacedPin = (Point, NetId, Coord);

/// Pins placed this iteration, bucketed by 1 mm cell per layer.
#[derive(Debug, Default)]
pub(crate) struct PinRegistry {
    cells: HashMap<(u8, i32, i32), Vec<PlacedPin>>,
    pub placed: u64,
}

const PIN_CELL: Coord = 1_000_000;

impl PinRegistry {
    fn key(layer: LayerId, p: Point) -> (u8, i32, i32) {
        (layer.0, p.x.div_euclid(PIN_CELL), p.y.div_euclid(PIN_CELL))
    }

    fn crowded(&self, ctx: &Ctx<'_>, net: NetId, layer: LayerId, p: Point, width: Coord) -> bool {
        let (l, cx, cy) = Self::key(layer, p);
        let me = RuleSubject::new(ItemKind::Track, Some(net));
        for dy in -1..=1 {
            for dx in -1..=1 {
                let Some(list) = self.cells.get(&(l, cx + dx, cy + dy)) else {
                    continue;
                };
                for &(q, other, ow) in list {
                    if other == net {
                        continue;
                    }
                    let c = ctx.rules.clearance(
                        &me,
                        &RuleSubject::new(ItemKind::Track, Some(other)),
                        layer,
                    );
                    let s = i128::from(width / 2 + width % 2)
                        + i128::from(ow / 2 + ow % 2)
                        + i128::from(c);
                    if (q - p).length2() < s * s {
                        return true;
                    }
                }
            }
        }
        false
    }

    fn add(&mut self, net: NetId, layer: LayerId, p: Point, width: Coord) {
        self.cells
            .entry(Self::key(layer, p))
            .or_default()
            .push((p, net, width));
        self.placed += 1;
    }
}

/// A change of owning tile along a guide line.
#[derive(Debug, Clone, Copy)]
struct Crossing {
    at: Point,
    from: usize,
    to: usize,
    /// Direction of travel along the guide segment.
    dir: (i64, i64),
    hint: Option<LayerId>,
}

fn lerp(p: Point, q: Point, t: f64) -> Point {
    Point::new(
        (f64::from(p.x) + (f64::from(q.x) - f64::from(p.x)) * t).round() as Coord,
        (f64::from(p.y) + (f64::from(q.y) - f64::from(p.y)) * t).round() as Coord,
    )
}

fn walk(tiling: &Tiling, line: &[Point], hints: &[Option<LayerId>]) -> Vec<Crossing> {
    let mut out = Vec::new();
    let mut owner = tiling.owner(line[0]);
    for i in 0..line.len().saturating_sub(1) {
        let (p, q) = (line[i], line[i + 1]);
        let hint = hints.get(i + 1).copied().flatten().or(hints[i]);
        let d = q - p;
        let len = (d.length2() as f64).sqrt();
        if len == 0.0 {
            continue;
        }
        let steps = (len / 200_000.0).ceil().max(1.0) as u32;
        let mut t_prev = 0.0;
        for k in 1..=steps {
            let t = f64::from(k) / f64::from(steps);
            let at_end = tiling.owner(lerp(p, q, t));
            let mut lo = t_prev;
            let mut guard = 0;
            while at_end != owner && guard < 64 {
                guard += 1;
                let (mut a, mut b) = (lo, t);
                while (b - a) * len > 500.0 {
                    let m = 0.5 * (a + b);
                    if tiling.owner(lerp(p, q, m)) == owner {
                        a = m;
                    } else {
                        b = m;
                    }
                }
                let at = lerp(p, q, b);
                let next = tiling.owner(at);
                if next == owner {
                    break;
                }
                out.push(Crossing {
                    at,
                    from: owner,
                    to: next,
                    dir: (d.x, d.y),
                    hint,
                });
                owner = next;
                lo = b;
            }
            t_prev = t;
        }
    }
    out
}

/// The heading bits whose move has `sign` along the axis (`x` or `y`).
fn crossing_headings(vertical_boundary: bool, sign: i32) -> u8 {
    HEADINGS
        .iter()
        .enumerate()
        .filter(|(_, (dx, dy))| {
            if vertical_boundary {
                *dx == sign
            } else {
                *dy == sign
            }
        })
        .fold(0u8, |m, (h, _)| m | (1 << h))
}

#[allow(clippy::too_many_arguments)]
fn choose_pin(
    ctx: &Ctx<'_>,
    tiling: &Tiling,
    task: &Task,
    index: usize,
    cr: &Crossing,
    registry: &PinRegistry,
    routed: &RoutedIndex,
    history: &HistoryMap,
) -> Option<Pin> {
    let a = &tiling.tiles[cr.from];
    let b = &tiling.tiles[cr.to];
    // The shared boundary: the core edge nearest the crossing point.
    let mut best_line: Option<(i64, bool, Coord)> = None;
    for core in [a.tile.core, b.tile.core] {
        for (vertical, v) in [
            (true, core.min.x),
            (true, core.max.x),
            (false, core.min.y),
            (false, core.max.y),
        ] {
            let d = if vertical {
                (i64::from(v) - i64::from(cr.at.x)).abs()
            } else {
                (i64::from(v) - i64::from(cr.at.y)).abs()
            };
            if best_line.is_none_or(|(bd, _, _)| d < bd) {
                best_line = Some((d, vertical, v));
            }
        }
    }
    let (_, vertical, v) = best_line?;
    let axis_dir = if vertical { cr.dir.0 } else { cr.dir.1 };
    let fallback_dir = if vertical {
        i64::from(b.tile.core.center().x) - i64::from(a.tile.core.center().x)
    } else {
        i64::from(b.tile.core.center().y) - i64::from(a.tile.core.center().y)
    };
    let sign = match (axis_dir.signum(), fallback_dir.signum()) {
        (0, f) => f as i32,
        (s, _) => s as i32,
    };
    if sign == 0 {
        return None;
    }
    let headings = crossing_headings(vertical, sign);
    let pitch = LATTICE_PITCH_NM;
    let along = if vertical { cr.at.y } else { cr.at.x };
    let base =
        (i64::from(along) + i64::from(pitch) / 2).div_euclid(i64::from(pitch)) * i64::from(pitch);
    let point = |k: i64| -> Option<Point> {
        let u = Coord::try_from(base + k * i64::from(pitch)).ok()?;
        Some(if vertical {
            Point::new(v, u)
        } else {
            Point::new(u, v)
        })
    };
    let inner = |zone: BBox| zone.inflate(-2 * pitch);
    let (za, zb) = (inner(a.commit), inner(b.commit));

    let mut layers: Vec<LayerId> = task
        .layers
        .iter()
        .filter(|l| ctx.slot_of(*l).is_some())
        .collect();
    if layers.is_empty() {
        return None;
    }
    // Layers both terminals sit on come first (no via pair across the pin),
    // rotated by rank so crossings spread; then the rest, rotated likewise.
    let on = |item: ItemRef| match item {
        ItemRef::Pad(id) => ctx.board.pads[id.index()].layers,
        ItemRef::Track(id) => crate::api::LayerMask::single(ctx.board.tracks[id.index()].layer),
        _ => ctx.board.layers.all(),
    };
    let shared = on(task.conn.from).intersection(on(task.conn.to));
    let (mut first, mut rest): (Vec<LayerId>, Vec<LayerId>) =
        layers.iter().partition(|l| shared.contains(**l));
    let rot = task.rank as usize + index;
    if !first.is_empty() {
        let n = first.len();
        first.rotate_left(rot % n);
    }
    if !rest.is_empty() {
        let n = rest.len();
        rest.rotate_left(rot % n);
    }
    layers = first.into_iter().chain(rest).collect();
    if let Some(h) = cr.hint.filter(|h| layers.contains(h)) {
        layers.retain(|l| *l != h);
        layers.insert(0, h);
    }
    let max_steps = i64::from(ctx.tuning.pin_search / pitch);
    let layer_penalty = 5 * i64::from(pitch);
    let mut strict: Option<(i64, Point, LayerId)> = None;
    let mut loose: Option<(i64, Point, LayerId)> = None;
    for (li, &layer) in layers.iter().enumerate() {
        let slot = ctx.slot_of(layer)?;
        let width = ctx.width(task.conn.net, slot);
        'steps: for step in 0..=max_steps {
            let offsets = [step, -step];
            for &k in &offsets[..if step == 0 { 1 } else { 2 }] {
                // Distance and layer rank are a lower bound on the score;
                // history (never negative) is added once the point is known.
                let bound = k.abs() * i64::from(pitch) + li as i64 * layer_penalty;
                if strict.is_some_and(|(s, _, _)| s <= bound) {
                    break 'steps;
                }
                let Some(q) = point(k) else { continue };
                let hist = history.get(layer.0, q.x.div_euclid(pitch), q.y.div_euclid(pitch));
                let score = bound + i64::from(hist) * layer_penalty;
                if strict.is_some_and(|(s, _, _)| s <= score) {
                    continue;
                }
                if !(za.contains(q) && zb.contains(q)) {
                    continue;
                }
                if registry.crowded(ctx, task.conn.net, layer, q, width) {
                    continue;
                }
                let dot = Segment::new(q, q, width);
                let cy = ctx.obstacles.courtyards.hits(&Shape::Segment(dot));
                if ctx
                    .obstacles
                    .check_segment(ctx.rules, task.conn.net, layer, &dot, &cy)
                    .is_some()
                {
                    continue;
                }
                if loose.is_none_or(|(s, _, _)| score < s) {
                    loose = Some((score, q, layer));
                }
                if strict.is_none_or(|(s, _, _)| score < s)
                    && routed.dot_is_clear(ctx.rules, task.conn.net, layer, q, width, &cy)
                {
                    strict = Some((score, q, layer));
                }
            }
        }
    }
    let (_, at, layer) = strict.or(loose)?;
    Some(Pin {
        at,
        layer,
        headings,
    })
}

/// Cut `task` into pieces for this tiling: `(tile index, piece)` in piece
/// order. Pins are registered as they are placed.
pub(crate) fn split(
    ctx: &Ctx<'_>,
    tiling: &Tiling,
    task: &Task,
    task_index: u32,
    registry: &mut PinRegistry,
    routed: &RoutedIndex,
    history: &HistoryMap,
) -> Result<Vec<(usize, PieceJob)>, DetailFailure> {
    let job = |piece: u8, from: Terminal, to: Terminal| PieceJob {
        task: task_index,
        piece,
        from,
        to,
        retries: 0,
    };
    let ends = (Terminal::Item(task.conn.from), Terminal::Item(task.conn.to));
    let ta = tiling.owner(task.a);
    let tb = tiling.owner(task.b);
    let span = BBox::from_points(task.line.iter().copied())
        .expect("a guide line has two points")
        .inflate(ctx.tuning.fit_margin);
    // A terminal in a BGA field is routed by its fine sub-tile, never
    // swallowed whole by a coarse grid tile.
    let fine = |t: usize| tiling.tiles[t].tile.phase == 0;
    let any_fine = fine(ta) || fine(tb);
    for t in [ta, tb] {
        if (fine(t) || !any_fine) && tiling.tiles[t].commit.contains_bbox(&span) {
            return Ok(vec![(t, job(0, ends.0, ends.1))]);
        }
    }
    let crossings = walk(tiling, &task.line, &task.hints);
    if crossings.is_empty() {
        return Ok(vec![(ta, job(0, ends.0, ends.1))]);
    }
    if crossings.len() >= u8::MAX as usize {
        return Err(DetailFailure::NoPin {
            from: tiling.tiles[ta].tile.id,
            to: tiling.tiles[tb].tile.id,
        });
    }
    let mut pins = Vec::with_capacity(crossings.len());
    for (k, cr) in crossings.iter().enumerate() {
        let Some(pin) = choose_pin(ctx, tiling, task, k, cr, registry, routed, history) else {
            return Err(DetailFailure::NoPin {
                from: tiling.tiles[cr.from].tile.id,
                to: tiling.tiles[cr.to].tile.id,
            });
        };
        let slot = ctx.slot_of(pin.layer).expect("pins use routable layers");
        registry.add(
            task.conn.net,
            pin.layer,
            pin.at,
            ctx.width(task.conn.net, slot),
        );
        pins.push(pin);
    }
    let mut out = Vec::with_capacity(pins.len() + 1);
    let mut from = ends.0;
    for (k, cr) in crossings.iter().enumerate() {
        let to = Terminal::Pin(pins[k]);
        out.push((cr.from, job(k as u8, from, to)));
        from = to;
    }
    let last = crossings.last().expect("non-empty").to;
    out.push((last, job(crossings.len() as u8, from, ends.1)));
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crossing_heading_sets_are_three_adjacent_headings() {
        // +x: (1,-1), (1,0), (1,1) are headings 7, 0, 1.
        assert_eq!(crossing_headings(true, 1), 0b1000_0011);
        assert_eq!(crossing_headings(true, -1), 0b0011_1000);
        assert_eq!(crossing_headings(false, 1), 0b0000_1110);
        assert_eq!(crossing_headings(false, -1), 0b1110_0000);
    }
}
