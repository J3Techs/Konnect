//! One net's copper as a resistive network, for current (K2), IR drop (K3)
//! and copper path lengths (K5, K9).
//!
//! - **Tracks** and **arcs** are resistors ρL/(w·t) between joints: their
//!   ends, plus every point where another track's end or a via lands on them
//!   (a T-junction splits the track there).
//! - **Vias** are a node per copper layer they span, joined by barrel
//!   resistors ρ·Δz/A over the stack depth between layer centres, with A the
//!   plated annulus (drill, plating thickness).
//! - **Pads** are one node each (pad copper is taken as equipotential).
//! - **Zone fills** are meshed on a square grid of step h: each cell holds
//!   its copper coverage c (0..1, exact along x, 4 samples along y); two
//!   neighbours are joined by the sheet conductance t/ρ·min(c₁, c₂) (one
//!   square), so a 0.5 oz layer at 25 °C reads ≈ 1.15 mΩ/sq and 1 oz ≈ 0.50.
//! - Ends, via landings and pads that touch are merged into one node; a
//!   joint, via or pad over a fill is tied to the cells under it by a half
//!   square each.
//!
//! The solver is conjugate gradients with an SSOR preconditioner on the
//! reduced Laplacian (fixed-voltage nodes eliminated), restricted to the part
//! of the network the fixed nodes reach.

use super::common::Loaded;
use super::geom::{closest_on_segment, dist, pmm, polygon_rings, Raster, P};
use super::ipc::{barrel_area_mm2, rho_at};
use konnect_geom::{point_shape_distance_cmp, Point, Shape};
use konnect_route::api::{LayerId, NetId};
use konnect_route::board::RouteBoard;
use std::cmp::Ordering;
use std::collections::{BTreeMap, BinaryHeap, HashMap};

pub(crate) const NONE: u32 = u32::MAX;

/// Shortest wire resistor, mm; joints closer are merged.
const MIN_WIRE_MM: f64 = 1e-3;

/// Bucket size of the sheet-link index, mm.
const SHEET_CELL: f64 = 1.0;

/// Largest raster (copper or not) one build allocates, cells.
const RASTER_LIMIT: usize = 30_000_000;

/// How a network is built.
#[derive(Debug, Clone)]
pub(crate) struct Options {
    /// Mesh step for zone fills, mm.
    pub h: f64,
    /// Copper temperature for resistivity, °C.
    pub temp_c: f64,
    /// Via barrel plating, mm.
    pub plating_mm: f64,
    /// Refuse a mesh larger than this many cells.
    pub max_cells: usize,
    /// Only copper meeting this box, mm.
    pub window: Option<(P, P)>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum Kind {
    Track {
        track: u32,
    },
    Arc {
        arc: u32,
    },
    Barrel {
        via: u32,
        from: LayerId,
        to: LayerId,
    },
    Sheet {
        layer: LayerId,
    },
    Contact,
}

#[derive(Debug, Clone)]
pub(crate) struct Elem {
    pub a: u32,
    pub b: u32,
    /// Conductance, S.
    pub g: f64,
    /// Path length, mm.
    pub len: f64,
    pub kind: Kind,
    pub p: P,
    pub q: P,
    /// Conductor width, mm (tracks, arcs; sheet: the cell step).
    pub width: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum What {
    Pad(u32),
    Via(u32, LayerId),
    Joint(LayerId),
    Cell(LayerId),
}

#[derive(Debug, Clone)]
pub(crate) struct NodeInfo {
    pub at: P,
    pub what: What,
}

pub(crate) struct LayerGrid {
    pub layer: LayerId,
    pub raster: Raster,
    pub node: Vec<u32>,
    pub t: f64,
}

pub(crate) struct Network {
    pub net: NetId,
    pub nodes: Vec<NodeInfo>,
    pub elems: Vec<Elem>,
    pub grids: Vec<LayerGrid>,
    pub pad_node: BTreeMap<u32, u32>,
    pub h: f64,
    pub counts: BTreeMap<&'static str, usize>,
    /// Sheet links bucketed by midpoint, built on first use.
    sheet_index: std::sync::OnceLock<HashMap<(u8, i32, i32), Vec<u32>>>,
}

struct Dsu(Vec<u32>);

impl Dsu {
    fn add(&mut self) -> u32 {
        self.0.push(self.0.len() as u32);
        (self.0.len() - 1) as u32
    }
    fn find(&mut self, mut x: u32) -> u32 {
        while self.0[x as usize] != x {
            let p = self.0[x as usize];
            self.0[x as usize] = self.0[p as usize];
            x = p;
        }
        x
    }
    fn union(&mut self, a: u32, b: u32) {
        let (a, b) = (self.find(a), self.find(b));
        if a != b {
            self.0[a.max(b) as usize] = a.min(b);
        }
    }
}

/// A track or arc as a straight wire for joint finding.
struct Wire {
    layer: LayerId,
    a: P,
    b: P,
    width: f64,
    len: f64,
    kind: Kind,
    /// Joint parameters along a→b, with their proto node.
    joints: Vec<(f64, u32)>,
}

fn in_window(w: &Option<(P, P)>, lo: P, hi: P) -> bool {
    match w {
        None => true,
        Some((a, b)) => hi[0] >= a[0] && lo[0] <= b[0] && hi[1] >= a[1] && lo[1] <= b[1],
    }
}

fn point_in_shape(p: P, s: &Shape, tol_mm: f64) -> bool {
    match Point::from_mm(p[0], p[1]) {
        Some(q) => {
            point_shape_distance_cmp(q, 0, s, (tol_mm * 1e6).round() as i64) != Ordering::Greater
        }
        None => false,
    }
}

fn arc_length(a: P, m: P, b: P) -> f64 {
    // Circumcircle through the three points.
    let d = 2.0 * (a[0] * (m[1] - b[1]) + m[0] * (b[1] - a[1]) + b[0] * (a[1] - m[1]));
    if d.abs() < 1e-12 {
        return dist(a, b);
    }
    let sq = |p: P| p[0] * p[0] + p[1] * p[1];
    let ux = (sq(a) * (m[1] - b[1]) + sq(m) * (b[1] - a[1]) + sq(b) * (a[1] - m[1])) / d;
    let uy = (sq(a) * (b[0] - m[0]) + sq(m) * (a[0] - b[0]) + sq(b) * (m[0] - a[0])) / d;
    let r = dist(a, [ux, uy]);
    let ang = |p: P| (p[1] - uy).atan2(p[0] - ux);
    let (t0, t1, t2) = (ang(a), ang(m), ang(b));
    let norm = |x: f64| x.rem_euclid(std::f64::consts::TAU);
    let sweep_ab = norm(t2 - t0);
    let sweep_am = norm(t1 - t0);
    let sweep = if sweep_am <= sweep_ab {
        sweep_ab
    } else {
        std::f64::consts::TAU - sweep_ab
    };
    r * sweep
}

impl Network {
    /// Build `net`'s network from the saved board.
    pub(crate) fn build(l: &Loaded, net: NetId, o: &Options) -> Result<Network, String> {
        let board = l.board();
        let stack = &l.stack;
        let rho = rho_at(o.temp_c);
        let mut dsu = Dsu(Vec::new());
        let mut what: Vec<NodeInfo> = Vec::new();
        let mut counts: BTreeMap<&'static str, usize> = BTreeMap::new();
        let add = |dsu: &mut Dsu, what: &mut Vec<NodeInfo>, at: P, w: What| {
            what.push(NodeInfo { at, what: w });
            dsu.add()
        };

        // Pads.
        let mut pad_proto: BTreeMap<u32, u32> = BTreeMap::new();
        for (i, p) in board.pads.iter().enumerate() {
            if p.net != Some(net) {
                continue;
            }
            let bb = p.bbox();
            let inside = bb
                .map(|b| in_window(&o.window, pmm(b.min), pmm(b.max)))
                .unwrap_or(false);
            if !inside {
                continue;
            }
            let id = add(&mut dsu, &mut what, pmm(p.center), What::Pad(i as u32));
            pad_proto.insert(i as u32, id);
        }
        *counts.entry("pads").or_default() += pad_proto.len();

        // Vias: one node per spanned layer.
        let mut via_proto: BTreeMap<u32, Vec<(LayerId, u32)>> = BTreeMap::new();
        for (i, v) in board.vias.iter().enumerate() {
            if v.net != Some(net) {
                continue;
            }
            let c = pmm(v.center);
            let r = f64::from(v.diameter) / 2e6;
            if !in_window(&o.window, [c[0] - r, c[1] - r], [c[0] + r, c[1] + r]) {
                continue;
            }
            let mut list = Vec::new();
            for layer in v.layers.iter() {
                let id = add(&mut dsu, &mut what, c, What::Via(i as u32, layer));
                list.push((layer, id));
            }
            via_proto.insert(i as u32, list);
        }
        *counts.entry("vias").or_default() += via_proto.len();

        // Tracks and arcs as wires with end joints.
        let mut wires: Vec<Wire> = Vec::new();
        for (i, t) in board.tracks.iter().enumerate() {
            if t.net != Some(net) {
                continue;
            }
            let (a, b) = (pmm(t.seg.a), pmm(t.seg.b));
            let w = f64::from(t.seg.width) / 1e6;
            let lo = [a[0].min(b[0]) - w, a[1].min(b[1]) - w];
            let hi = [a[0].max(b[0]) + w, a[1].max(b[1]) + w];
            if !in_window(&o.window, lo, hi) {
                continue;
            }
            wires.push(Wire {
                layer: t.layer,
                a,
                b,
                width: w,
                len: dist(a, b),
                kind: Kind::Track { track: i as u32 },
                joints: Vec::new(),
            });
        }
        for (i, arc) in board.arcs.iter().enumerate() {
            if arc.net != Some(net) {
                continue;
            }
            let (a, m, b) = (pmm(arc.start), pmm(arc.mid), pmm(arc.end));
            let w = f64::from(arc.width) / 1e6;
            let lo = [a[0].min(b[0]).min(m[0]) - w, a[1].min(b[1]).min(m[1]) - w];
            let hi = [a[0].max(b[0]).max(m[0]) + w, a[1].max(b[1]).max(m[1]) + w];
            if !in_window(&o.window, lo, hi) {
                continue;
            }
            wires.push(Wire {
                layer: arc.layer,
                a,
                b,
                width: w,
                len: arc_length(a, m, b),
                kind: Kind::Arc { arc: i as u32 },
                joints: Vec::new(),
            });
        }
        *counts.entry("tracks_and_arcs").or_default() += wires.len();
        for w in &mut wires {
            let ja = add(&mut dsu, &mut what, w.a, What::Joint(w.layer));
            let jb = add(&mut dsu, &mut what, w.b, What::Joint(w.layer));
            w.joints.push((0.0, ja));
            w.joints.push((1.0, jb));
        }

        // Spatial index of wire ends per layer.
        let cell = 0.5;
        let key = |p: P| ((p[0] / cell).floor() as i64, (p[1] / cell).floor() as i64);
        let mut ends: HashMap<(u8, i64, i64), Vec<(usize, usize)>> = HashMap::new();
        for (wi, w) in wires.iter().enumerate() {
            for (k, p) in [(0usize, w.a), (1usize, w.b)] {
                let (x, y) = key(p);
                ends.entry((w.layer.0, x, y)).or_default().push((wi, k));
            }
        }
        let near_ends = |layer: LayerId, p: P, r: f64, f: &mut dyn FnMut(usize, usize)| {
            let (x0, y0) = key([p[0] - r, p[1] - r]);
            let (x1, y1) = key([p[0] + r, p[1] + r]);
            for x in x0..=x1 {
                for y in y0..=y1 {
                    if let Some(v) = ends.get(&(layer.0, x, y)) {
                        for &(wi, k) in v {
                            f(wi, k);
                        }
                    }
                }
            }
        };

        // End to end, and ends landing on another wire's length (T-joints).
        let mut tees: Vec<(usize, f64, u32)> = Vec::new();
        for wi in 0..wires.len() {
            let w = &wires[wi];
            let reach = w.width / 2.0;
            let bb_lo = [w.a[0].min(w.b[0]) - reach, w.a[1].min(w.b[1]) - reach];
            let bb_hi = [w.a[0].max(w.b[0]) + reach, w.a[1].max(w.b[1]) + reach];
            let centre = [(bb_lo[0] + bb_hi[0]) / 2.0, (bb_lo[1] + bb_hi[1]) / 2.0];
            let rad = dist(bb_lo, bb_hi) / 2.0;
            let mut hits = Vec::new();
            near_ends(w.layer, centre, rad, &mut |oj, k| {
                if oj != wi {
                    hits.push((oj, k));
                }
            });
            for (oj, k) in hits {
                let o_end = if k == 0 { wires[oj].a } else { wires[oj].b };
                let o_joint = wires[oj].joints[k].1;
                let (q, t) = closest_on_segment(o_end, w.a, w.b);
                if dist(q, o_end) > reach + 1e-6 {
                    continue;
                }
                if t <= 1e-9 || dist(o_end, w.a) <= reach {
                    dsu.union(o_joint, wires[wi].joints[0].1);
                } else if t >= 1.0 - 1e-9 || dist(o_end, w.b) <= reach {
                    dsu.union(o_joint, wires[wi].joints[1].1);
                } else if matches!(w.kind, Kind::Track { .. }) {
                    tees.push((wi, t, o_joint));
                }
            }
        }
        for (wi, t, j) in tees {
            wires[wi].joints.push((t, j));
        }

        // Vias: ends inside the via, wires running over it.
        for (&vi, list) in &via_proto {
            let v = &board.vias[vi as usize];
            let c = pmm(v.center);
            let r = f64::from(v.diameter) / 2e6;
            for &(layer, node) in list {
                let mut hits = Vec::new();
                near_ends(layer, c, r + 2.0, &mut |wi, k| hits.push((wi, k)));
                for (wi, k) in hits {
                    let p = if k == 0 { wires[wi].a } else { wires[wi].b };
                    if dist(p, c) <= r + 1e-6 {
                        dsu.union(wires[wi].joints[k].1, node);
                    }
                }
            }
        }
        // Wires passing over a via centre (no end on it).
        for wire in &mut wires {
            if !matches!(wire.kind, Kind::Track { .. }) {
                continue;
            }
            let (a, b, layer, reach) = (wire.a, wire.b, wire.layer, wire.width / 2.0);
            for (&vi, list) in &via_proto {
                let Some(&(_, node)) = list.iter().find(|(l, _)| *l == layer) else {
                    continue;
                };
                let c = pmm(board.vias[vi as usize].center);
                let (q, t) = closest_on_segment(c, a, b);
                if dist(q, c) <= reach
                    && t > 1e-9
                    && t < 1.0 - 1e-9
                    && dist(c, a) > reach
                    && dist(c, b) > reach
                {
                    wire.joints.push((t, node));
                }
            }
        }

        // Pads: wire ends inside a pad, vias touching a pad.
        for (&pi, &node) in &pad_proto {
            let pad = &board.pads[pi as usize];
            let c = pmm(pad.center);
            let reach = pad
                .bbox()
                .map(|b| dist(pmm(b.min), pmm(b.max)) / 2.0)
                .unwrap_or(1.0);
            for layer in pad.layers.iter() {
                let mut hits = Vec::new();
                near_ends(layer, c, reach + 0.5, &mut |wi, k| hits.push((wi, k)));
                for (wi, k) in hits {
                    let p = if k == 0 { wires[wi].a } else { wires[wi].b };
                    if pad.shapes.iter().any(|s| point_in_shape(p, s, 1e-4)) {
                        dsu.union(wires[wi].joints[k].1, node);
                    }
                }
                for (&vi, list) in &via_proto {
                    let v = &board.vias[vi as usize];
                    let Some(&(_, vnode)) = list.iter().find(|(l, _)| *l == layer) else {
                        continue;
                    };
                    let vc = pmm(v.center);
                    if dist(vc, c) > reach + 2.0 {
                        continue;
                    }
                    let circle = Shape::Circle(konnect_geom::Circle::new(v.center, v.diameter / 2));
                    if pad.shapes.iter().any(|s| konnect_geom::touches(s, &circle)) {
                        dsu.union(vnode, node);
                    }
                }
            }
            // A through-hole pad is one node on every layer it spans; vias
            // and wires join it per layer above.
        }

        // Joints closer than a micron along a wire are one node: a resistor
        // that short would be a million times stiffer than its neighbours
        // and stall the solver.
        for w in &mut wires {
            w.joints.sort_by(|x, y| x.0.total_cmp(&y.0));
            for pair in w.joints.windows(2) {
                if (pair[1].0 - pair[0].0) * w.len < MIN_WIRE_MM {
                    dsu.union(pair[0].1, pair[1].1);
                }
            }
        }

        // Compress proto nodes.
        let n_proto = dsu.0.len();
        let mut root_index: HashMap<u32, u32> = HashMap::new();
        let mut nodes: Vec<NodeInfo> = Vec::new();
        let mut proto_to: Vec<u32> = vec![NONE; n_proto];
        // Prefer a pad, then a via, as the node's identity.
        let rank = |w: &What| match w {
            What::Pad(_) => 0,
            What::Via(..) => 1,
            _ => 2,
        };
        for p in 0..n_proto as u32 {
            let r = dsu.find(p);
            let idx = *root_index.entry(r).or_insert_with(|| {
                nodes.push(what[p as usize].clone());
                (nodes.len() - 1) as u32
            });
            proto_to[p as usize] = idx;
            if rank(&what[p as usize].what) < rank(&nodes[idx as usize].what) {
                nodes[idx as usize] = what[p as usize].clone();
            }
        }
        let mut elems: Vec<Elem> = Vec::new();

        // Wire resistors between consecutive joints.
        for w in &mut wires {
            w.joints.sort_by(|x, y| x.0.total_cmp(&y.0));
            let t = stack.t(w.layer);
            for pair in w.joints.windows(2) {
                let (t0, n0) = pair[0];
                let (t1, n1) = pair[1];
                let (a, b) = (proto_to[n0 as usize], proto_to[n1 as usize]);
                if a == b {
                    continue;
                }
                let len = ((t1 - t0) * w.len).max(MIN_WIRE_MM);
                let p = [
                    w.a[0] + t0 * (w.b[0] - w.a[0]),
                    w.a[1] + t0 * (w.b[1] - w.a[1]),
                ];
                let q = [
                    w.a[0] + t1 * (w.b[0] - w.a[0]),
                    w.a[1] + t1 * (w.b[1] - w.a[1]),
                ];
                elems.push(Elem {
                    a,
                    b,
                    g: w.width * t / (rho * len),
                    len,
                    kind: w.kind,
                    p,
                    q,
                    width: w.width,
                });
            }
        }
        // Via barrels between consecutive spanned layers.
        for (&vi, list) in &via_proto {
            let v = &board.vias[vi as usize];
            let area = barrel_area_mm2(f64::from(v.drill) / 1e6, o.plating_mm);
            let c = pmm(v.center);
            for pair in list.windows(2) {
                let ((la, pa), (lb, pb)) = (pair[0], pair[1]);
                let (a, b) = (proto_to[pa as usize], proto_to[pb as usize]);
                if a == b {
                    continue;
                }
                let len = (stack.z_mm[lb.index()] - stack.z_mm[la.index()])
                    .abs()
                    .max(1e-4);
                elems.push(Elem {
                    a,
                    b,
                    g: area / (rho * len),
                    len,
                    kind: Kind::Barrel {
                        via: vi,
                        from: la,
                        to: lb,
                    },
                    p: c,
                    q: c,
                    width: f64::from(v.drill) / 1e6,
                });
            }
        }

        // Zone fills, meshed per layer.
        let mut grids: Vec<LayerGrid> = Vec::new();
        let mut by_layer: BTreeMap<u8, Vec<usize>> = BTreeMap::new();
        for (zi, z) in board.zones.iter().enumerate() {
            if z.net == Some(net) && z.filled {
                let b = z.region.bbox();
                if in_window(&o.window, pmm(b.min), pmm(b.max)) {
                    by_layer.entry(z.layer.0).or_default().push(zi);
                }
            }
        }
        let mut total_cells = 0usize;
        for (layer, pieces) in &by_layer {
            let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
            for &zi in pieces {
                let b = board.zones[zi].region.bbox();
                let (a, c) = (pmm(b.min), pmm(b.max));
                lo = [lo[0].min(a[0]), lo[1].min(a[1])];
                hi = [hi[0].max(c[0]), hi[1].max(c[1])];
            }
            if let Some((wa, wb)) = o.window {
                lo = [lo[0].max(wa[0]), lo[1].max(wa[1])];
                hi = [hi[0].min(wb[0]), hi[1].min(wb[1])];
            }
            let cells = (((hi[0] - lo[0]) / o.h).ceil().max(1.0)
                * ((hi[1] - lo[1]) / o.h).ceil().max(1.0)) as usize;
            total_cells += cells;
            if total_cells > RASTER_LIMIT {
                return Err(format!(
                    "the fills' extent needs a {total_cells}-cell raster at {} mm (limit {RASTER_LIMIT}); use a coarser resolution_mm or a smaller area",
                    o.h
                ));
            }
            let mut raster = Raster::new(lo, hi, o.h);
            for &zi in pieces {
                raster.fill(&polygon_rings(&board.zones[zi].region), 4);
            }
            grids.push(LayerGrid {
                layer: LayerId(*layer),
                node: vec![NONE; raster.cov.len()],
                raster,
                t: stack.t(LayerId(*layer)),
            });
        }
        const MIN_COV: f32 = 0.02;
        for g in &mut grids {
            let r = &g.raster;
            for j in 0..r.ny {
                for i in 0..r.nx {
                    let k = r.idx(i, j);
                    if r.cov[k] >= MIN_COV {
                        nodes.push(NodeInfo {
                            at: r.center(i, j),
                            what: What::Cell(g.layer),
                        });
                        g.node[k] = (nodes.len() - 1) as u32;
                    }
                }
            }
            let sheet = g.t / rho;
            for j in 0..r.ny {
                for i in 0..r.nx {
                    let k = r.idx(i, j);
                    if g.node[k] == NONE {
                        continue;
                    }
                    for (di, dj) in [(1usize, 0usize), (0, 1)] {
                        let (ii, jj) = (i + di, j + dj);
                        if ii >= r.nx || jj >= r.ny {
                            continue;
                        }
                        let kk = r.idx(ii, jj);
                        if g.node[kk] == NONE {
                            continue;
                        }
                        let c = f64::from(r.cov[k].min(r.cov[kk]));
                        elems.push(Elem {
                            a: g.node[k],
                            b: g.node[kk],
                            g: sheet * c,
                            len: r.h,
                            kind: Kind::Sheet { layer: g.layer },
                            p: r.center(i, j),
                            q: r.center(ii, jj),
                            width: r.h * c,
                        });
                    }
                }
            }
        }
        let copper_cells: usize = grids
            .iter()
            .map(|g| g.node.iter().filter(|n| **n != NONE).count())
            .sum();
        if copper_cells > o.max_cells {
            return Err(format!(
                "the zone mesh has {copper_cells} copper cells at {} mm (max_cells {}); use a coarser resolution_mm or a smaller area",
                o.h, o.max_cells
            ));
        }
        counts.insert("cells", copper_cells);

        // Ties to the mesh: a node over fill joins the cells under it.
        let tie = |elems: &mut Vec<Elem>,
                   g: &LayerGrid,
                   node: u32,
                   at: P,
                   test: &dyn Fn(P) -> bool,
                   reach: f64| {
            let r = &g.raster;
            let i0 = ((at[0] - reach - r.x0) / r.h).floor().max(0.0) as usize;
            let j0 = ((at[1] - reach - r.y0) / r.h).floor().max(0.0) as usize;
            let i1 = (((at[0] + reach - r.x0) / r.h).ceil().max(0.0) as usize).min(r.nx);
            let j1 = (((at[1] + reach - r.y0) / r.h).ceil().max(0.0) as usize).min(r.ny);
            let mut any = false;
            for j in j0..j1 {
                for i in i0..i1 {
                    let k = r.idx(i, j);
                    if g.node[k] == NONE || g.node[k] == node {
                        continue;
                    }
                    if test(r.center(i, j)) {
                        any = true;
                        elems.push(Elem {
                            a: node,
                            b: g.node[k],
                            g: 2.0 * g.t / rho * f64::from(r.cov[k]),
                            len: 0.0,
                            kind: Kind::Contact,
                            p: at,
                            q: r.center(i, j),
                            width: r.h,
                        });
                    }
                }
            }
            any
        };
        let grid_of = |layer: LayerId| grids.iter().find(|g| g.layer == layer);
        for (&pi, &proto) in &pad_proto {
            let pad = &board.pads[pi as usize];
            let node = proto_to[proto as usize];
            let reach = pad
                .bbox()
                .map(|b| dist(pmm(b.min), pmm(b.max)) / 2.0)
                .unwrap_or(1.0)
                + o.h;
            for layer in pad.layers.iter() {
                if let Some(g) = grid_of(layer) {
                    let half = g.raster.h / 2.0;
                    tie(
                        &mut elems,
                        g,
                        node,
                        pmm(pad.center),
                        &|c| pad.shapes.iter().any(|s| point_in_shape(c, s, half)),
                        reach,
                    );
                }
            }
        }
        for (&vi, list) in &via_proto {
            let v = &board.vias[vi as usize];
            let c = pmm(v.center);
            let r = f64::from(v.diameter) / 2e6;
            for &(layer, proto) in list {
                if let Some(g) = grid_of(layer) {
                    let half = g.raster.h / 2.0;
                    tie(
                        &mut elems,
                        g,
                        proto_to[proto as usize],
                        c,
                        &|q| dist(q, c) <= r + half,
                        r + o.h,
                    );
                }
            }
        }
        for w in &wires {
            if let Some(g) = grid_of(w.layer) {
                let half = w.width / 2.0;
                for &(t, proto) in &w.joints {
                    let at = [
                        w.a[0] + t * (w.b[0] - w.a[0]),
                        w.a[1] + t * (w.b[1] - w.a[1]),
                    ];
                    let gh = g.raster.h / 2.0;
                    tie(
                        &mut elems,
                        g,
                        proto_to[proto as usize],
                        at,
                        &|q| dist(q, at) <= half + gh,
                        half + o.h,
                    );
                }
            }
        }

        let pad_node = pad_proto
            .iter()
            .map(|(&p, &proto)| (p, proto_to[proto as usize]))
            .collect();
        Ok(Network {
            net,
            nodes,
            elems,
            grids,
            pad_node,
            h: o.h,
            counts,
            sheet_index: std::sync::OnceLock::new(),
        })
    }

    /// Every cell node of `layer`.
    pub(crate) fn cells_of(&self, layer: LayerId) -> Vec<u32> {
        self.grids
            .iter()
            .filter(|g| g.layer == layer)
            .flat_map(|g| g.node.iter().copied().filter(|n| *n != NONE))
            .collect()
    }

    fn adjacency(&self) -> (Vec<usize>, Vec<(u32, u32)>) {
        let n = self.nodes.len();
        let mut deg = vec![0usize; n + 1];
        for e in &self.elems {
            deg[e.a as usize + 1] += 1;
            deg[e.b as usize + 1] += 1;
        }
        for i in 0..n {
            deg[i + 1] += deg[i];
        }
        let mut fill = deg.clone();
        let mut adj = vec![(0u32, 0u32); deg[n]];
        for (ei, e) in self.elems.iter().enumerate() {
            adj[fill[e.a as usize]] = (e.b, ei as u32);
            fill[e.a as usize] += 1;
            adj[fill[e.b as usize]] = (e.a, ei as u32);
            fill[e.b as usize] += 1;
        }
        (deg, adj)
    }

    /// Whether every node of `targets` is joined by copper to a node of
    /// `fixed`.
    pub(crate) fn reaches(&self, fixed: &[(u32, f64)], targets: &[u32]) -> bool {
        let (start, adj) = self.adjacency();
        let mut seen = vec![false; self.nodes.len()];
        let mut stack: Vec<u32> = fixed.iter().map(|f| f.0).collect();
        for &s in &stack {
            seen[s as usize] = true;
        }
        while let Some(x) = stack.pop() {
            for &(y, e) in &adj[start[x as usize]..start[x as usize + 1]] {
                if self.elems[e as usize].g > 0.0 && !seen[y as usize] {
                    seen[y as usize] = true;
                    stack.push(y);
                }
            }
        }
        targets.iter().all(|&t| seen[t as usize])
    }

    /// Solve for node voltages with `fixed` nodes held at their voltages and
    /// `inject` currents (A, positive into the node) at others. Nodes the
    /// fixed ones do not reach are NaN.
    pub(crate) fn solve(
        &self,
        fixed: &[(u32, f64)],
        inject: &[(u32, f64)],
    ) -> Result<Solution, String> {
        let n = self.nodes.len();
        if fixed.is_empty() {
            return Err("no node is held at a voltage".to_string());
        }
        let (start, adj) = self.adjacency();
        // Reach.
        let mut reached = vec![false; n];
        let mut stack: Vec<u32> = fixed.iter().map(|f| f.0).collect();
        for &s in &stack {
            reached[s as usize] = true;
        }
        while let Some(x) = stack.pop() {
            for &(y, e) in &adj[start[x as usize]..start[x as usize + 1]] {
                if self.elems[e as usize].g > 0.0 && !reached[y as usize] {
                    reached[y as usize] = true;
                    stack.push(y);
                }
            }
        }
        for &(node, i) in inject {
            if !reached[node as usize] && i != 0.0 {
                return Err(format!(
                    "a current is drawn at {:?} ({:.3}, {:.3}), which no copper joins to the source",
                    self.nodes[node as usize].what, self.nodes[node as usize].at[0], self.nodes[node as usize].at[1]
                ));
            }
        }
        let mut v = vec![f64::NAN; n];
        let mut is_fixed = vec![false; n];
        for &(node, volts) in fixed {
            v[node as usize] = volts;
            is_fixed[node as usize] = true;
        }
        // Free unknowns.
        let mut index = vec![NONE; n];
        let mut free: Vec<u32> = Vec::new();
        for i in 0..n {
            if reached[i] && !is_fixed[i] {
                index[i] = free.len() as u32;
                free.push(i as u32);
            }
        }
        let m = free.len();
        let mut b = vec![0.0; m];
        for &(node, i) in inject {
            if index[node as usize] != NONE {
                b[index[node as usize] as usize] += i;
            }
        }
        let mut diag = vec![0.0; m];
        let mut rows: Vec<Vec<(u32, f64)>> = vec![Vec::new(); m];
        for e in &self.elems {
            if e.g <= 0.0 {
                continue;
            }
            let (a, c) = (e.a as usize, e.b as usize);
            if !reached[a] {
                continue;
            }
            match (index[a], index[c]) {
                (NONE, NONE) => {}
                (ia, NONE) => {
                    diag[ia as usize] += e.g;
                    b[ia as usize] += e.g * v[c];
                }
                (NONE, ic) => {
                    diag[ic as usize] += e.g;
                    b[ic as usize] += e.g * v[a];
                }
                (ia, ic) => {
                    diag[ia as usize] += e.g;
                    diag[ic as usize] += e.g;
                    rows[ia as usize].push((ic, e.g));
                    rows[ic as usize].push((ia, e.g));
                }
            }
        }
        // CSR.
        let mut rs = vec![0usize; m + 1];
        for i in 0..m {
            rs[i + 1] = rs[i] + rows[i].len();
        }
        let mut cols = Vec::with_capacity(rs[m]);
        let mut vals = Vec::with_capacity(rs[m]);
        for r in rows {
            for (c, g) in r {
                cols.push(c);
                vals.push(g);
            }
        }
        let mul = |x: &[f64], y: &mut [f64]| {
            for i in 0..m {
                let mut s = diag[i] * x[i];
                for k in rs[i]..rs[i + 1] {
                    s -= vals[k] * x[cols[k] as usize];
                }
                y[i] = s;
            }
        };
        // Initial guess: the mean fixed voltage.
        let v0 = fixed.iter().map(|f| f.1).sum::<f64>() / fixed.len() as f64;
        let mut x = vec![v0; m];
        let mut r = vec![0.0; m];
        mul(&x, &mut r);
        for i in 0..m {
            r[i] = b[i] - r[i];
        }
        let bnorm = b.iter().map(|q| q * q).sum::<f64>().sqrt().max(1e-30);
        let inv: Vec<f64> = diag
            .iter()
            .map(|d| if *d > 0.0 { 1.0 / d } else { 0.0 })
            .collect();
        // SSOR preconditioner (ω = 1.6): a forward then a backward
        // relaxation sweep over the rows, in mesh order.
        const OMEGA: f64 = 1.6;
        let precondition = |r: &[f64], z: &mut [f64]| {
            for i in 0..m {
                let mut acc = r[i];
                for k in rs[i]..rs[i + 1] {
                    let j = cols[k] as usize;
                    if j < i {
                        acc += vals[k] * z[j];
                    }
                }
                z[i] = OMEGA * acc * inv[i];
            }
            for i in (0..m).rev() {
                let mut acc = 0.0;
                for k in rs[i]..rs[i + 1] {
                    let j = cols[k] as usize;
                    if j > i {
                        acc += vals[k] * z[j];
                    }
                }
                z[i] = (2.0 - OMEGA) * z[i] + OMEGA * acc * inv[i];
            }
        };
        let mut z = vec![0.0; m];
        precondition(&r, &mut z);
        let mut p = z.clone();
        let mut rz: f64 = r.iter().zip(&z).map(|(a, b)| a * b).sum();
        let mut ap = vec![0.0; m];
        let mut it = 0usize;
        let max_it = 20 * (m as f64).sqrt() as usize + 10_000;
        let tol = 1e-11;
        let mut rnorm = r.iter().map(|q| q * q).sum::<f64>().sqrt();
        // Also stop once the terminals' voltages settle: their change over
        // the last CHECK iterations below 1e-7 of their spread from the fixed
        // voltage, twice running. Far, idle plane copper converges slowly and
        // does not move them.
        const CHECK: usize = 100;
        let watch: Vec<usize> = inject
            .iter()
            .filter(|&&(n, _)| index[n as usize] != NONE)
            .map(|&(n, _)| index[n as usize] as usize)
            .collect();
        let mut last: Vec<f64> = watch.iter().map(|&k| x[k]).collect();
        let mut calm = 0usize;
        let mut settled = false;
        while rnorm > tol * bnorm && it < max_it && m > 0 && !settled {
            mul(&p, &mut ap);
            let pap: f64 = p.iter().zip(&ap).map(|(a, b)| a * b).sum();
            if pap <= 0.0 {
                break;
            }
            let alpha = rz / pap;
            for i in 0..m {
                x[i] += alpha * p[i];
                r[i] -= alpha * ap[i];
            }
            precondition(&r, &mut z);
            let rz_new: f64 = r.iter().zip(&z).map(|(a, b)| a * b).sum();
            let beta = rz_new / rz;
            rz = rz_new;
            for i in 0..m {
                p[i] = z[i] + beta * p[i];
            }
            rnorm = r.iter().map(|q| q * q).sum::<f64>().sqrt();
            it += 1;
            if !watch.is_empty() && it.is_multiple_of(CHECK) {
                let now: Vec<f64> = watch.iter().map(|&k| x[k]).collect();
                let change = now
                    .iter()
                    .zip(&last)
                    .map(|(a, b)| (a - b).abs())
                    .fold(0.0, f64::max);
                let spread = now
                    .iter()
                    .map(|a| (a - v0).abs())
                    .fold(0.0, f64::max)
                    .max(1e-15);
                calm = if change <= 1e-7 * spread { calm + 1 } else { 0 };
                settled = calm >= 2;
                last = now;
            }
            #[cfg(test)]
            if it.is_multiple_of(500) && std::env::var_os("KONNECT_SOLVER_TRACE").is_some() {
                eprintln!(
                    "cg {m} unknowns: iteration {it}, relative residual {:.3e}",
                    rnorm / bnorm
                );
            }
        }
        let converged = rnorm <= tol * bnorm || m == 0 || settled;
        for (k, &node) in free.iter().enumerate() {
            v[node as usize] = x[k];
        }
        Ok(Solution {
            v,
            iterations: it,
            relative_residual: rnorm / bnorm,
            converged,
            unknowns: m,
        })
    }

    /// Current through element `e`, A, from `a` to `b`.
    pub(crate) fn current(&self, s: &Solution, e: &Elem) -> f64 {
        let (va, vb) = (s.v[e.a as usize], s.v[e.b as usize]);
        if va.is_nan() || vb.is_nan() {
            0.0
        } else {
            e.g * (va - vb)
        }
    }

    /// Shortest copper path length, mm, from any of `from` to every node
    /// (infinite where unreached). Zone cells also join their diagonal
    /// neighbours (h·√2) so a path across a plane is within 8 % of straight.
    pub(crate) fn path_lengths(&self, from: &[u32]) -> Vec<f64> {
        let n = self.nodes.len();
        let (start, adj) = self.adjacency();
        let mut d = vec![f64::INFINITY; n];
        #[derive(PartialEq)]
        struct Item(f64, u32);
        impl Eq for Item {}
        impl PartialOrd for Item {
            fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
                Some(self.cmp(o))
            }
        }
        impl Ord for Item {
            fn cmp(&self, o: &Self) -> Ordering {
                o.0.total_cmp(&self.0).then(o.1.cmp(&self.1))
            }
        }
        let mut heap = BinaryHeap::new();
        for &f in from {
            d[f as usize] = 0.0;
            heap.push(Item(0.0, f));
        }
        // Cell lookups for diagonal steps.
        let mut cell_pos: HashMap<u32, (usize, usize, usize)> = HashMap::new();
        for (gi, g) in self.grids.iter().enumerate() {
            for j in 0..g.raster.ny {
                for i in 0..g.raster.nx {
                    let k = g.raster.idx(i, j);
                    if g.node[k] != NONE {
                        cell_pos.insert(g.node[k], (gi, i, j));
                    }
                }
            }
        }
        while let Some(Item(dx, x)) = heap.pop() {
            if dx > d[x as usize] {
                continue;
            }
            let mut relax = |y: u32, len: f64, heap: &mut BinaryHeap<Item>| {
                let nd = dx + len;
                if nd < d[y as usize] {
                    d[y as usize] = nd;
                    heap.push(Item(nd, y));
                }
            };
            for &(y, e) in &adj[start[x as usize]..start[x as usize + 1]] {
                let el = &self.elems[e as usize];
                let len = match el.kind {
                    Kind::Contact => dist(el.p, el.q),
                    _ => el.len,
                };
                relax(y, len, &mut heap);
            }
            if let Some(&(gi, i, j)) = cell_pos.get(&x) {
                let g = &self.grids[gi];
                for (di, dj) in [(-1i64, -1i64), (-1, 1), (1, -1), (1, 1)] {
                    let (ii, jj) = (i as i64 + di, j as i64 + dj);
                    if ii < 0 || jj < 0 || ii >= g.raster.nx as i64 || jj >= g.raster.ny as i64 {
                        continue;
                    }
                    let y = g.node[g.raster.idx(ii as usize, jj as usize)];
                    if y != NONE {
                        relax(y, g.raster.h * std::f64::consts::SQRT_2, &mut heap);
                    }
                }
            }
        }
        d
    }

    /// The node of pad `pad` (board pad index), if in the network.
    pub(crate) fn pad(&self, pad: usize) -> Option<u32> {
        self.pad_node.get(&(pad as u32)).copied()
    }
}

/// A solved network.
pub(crate) struct Solution {
    pub v: Vec<f64>,
    pub iterations: usize,
    pub relative_residual: f64,
    pub converged: bool,
    pub unknowns: usize,
}

impl Network {
    /// Net current, A, crossing the chord `a`–`b` on `layer` through the
    /// mesh: every sheet link between two cells whose centres lie on
    /// opposite sides of the chord line, within the chord extended by one
    /// cell at each end, counted with its direction.
    pub(crate) fn flux_across(&self, s: &Solution, layer: LayerId, a: P, b: P) -> f64 {
        let d = [b[0] - a[0], b[1] - a[1]];
        let l = d[0].hypot(d[1]);
        if l == 0.0 {
            return 0.0;
        }
        let u = [d[0] / l, d[1] / l];
        let n = [-u[1], u[0]];
        let ext = self.h;
        let mut total = 0.0;
        let index = self.sheet_index.get_or_init(|| {
            let mut m: HashMap<(u8, i32, i32), Vec<u32>> = HashMap::new();
            for (i, e) in self.elems.iter().enumerate() {
                if let Kind::Sheet { layer } = e.kind {
                    let c = super::geom::mid(e.p, e.q);
                    m.entry((
                        layer.0,
                        (c[0] / SHEET_CELL).floor() as i32,
                        (c[1] / SHEET_CELL).floor() as i32,
                    ))
                    .or_default()
                    .push(i as u32);
                }
            }
            m
        });
        let pad = ext + self.h;
        let (x0, x1) = (a[0].min(b[0]) - pad, a[0].max(b[0]) + pad);
        let (y0, y1) = (a[1].min(b[1]) - pad, a[1].max(b[1]) + pad);
        let mut near: Vec<u32> = Vec::new();
        for cx in (x0 / SHEET_CELL).floor() as i32..=(x1 / SHEET_CELL).floor() as i32 {
            for cy in (y0 / SHEET_CELL).floor() as i32..=(y1 / SHEET_CELL).floor() as i32 {
                if let Some(v) = index.get(&(layer.0, cx, cy)) {
                    near.extend_from_slice(v);
                }
            }
        }
        for &ei in &near {
            let e = &self.elems[ei as usize];
            let side = |p: P| (p[0] - a[0]) * n[0] + (p[1] - a[1]) * n[1];
            let (sp, sq) = (side(e.p), side(e.q));
            if (sp > 0.0) == (sq > 0.0) || sp == sq {
                continue;
            }
            // Where the link crosses the chord line, along the chord.
            let f = sp / (sp - sq);
            let x = [
                e.p[0] + f * (e.q[0] - e.p[0]),
                e.p[1] + f * (e.q[1] - e.p[1]),
            ];
            let along = (x[0] - a[0]) * u[0] + (x[1] - a[1]) * u[1];
            if along < -ext || along > l + ext {
                continue;
            }
            let i = self.current(s, e);
            total += if sp > 0.0 { i } else { -i };
        }
        total.abs()
    }

    /// Current density per mesh cell, A/mm²: the cell's x and y currents
    /// (mean of its two links each way) over its copper cross-section.
    pub(crate) fn cell_density(&self, s: &Solution) -> Vec<(u32, LayerId, f64)> {
        let mut jx: HashMap<u32, (f64, u8)> = HashMap::new();
        let mut jy: HashMap<u32, (f64, u8)> = HashMap::new();
        for e in &self.elems {
            let Kind::Sheet { .. } = e.kind else { continue };
            let i = self.current(s, e);
            let horizontal = (e.p[1] - e.q[1]).abs() < 1e-9;
            let map = if horizontal { &mut jx } else { &mut jy };
            for node in [e.a, e.b] {
                let ent = map.entry(node).or_insert((0.0, 0));
                ent.0 += i;
                ent.1 += 1;
            }
        }
        let mut out = Vec::new();
        for g in &self.grids {
            for (k, &node) in g.node.iter().enumerate() {
                if node == NONE {
                    continue;
                }
                let (x, nx) = jx.get(&node).copied().unwrap_or((0.0, 1));
                let (y, ny) = jy.get(&node).copied().unwrap_or((0.0, 1));
                let ix = x / f64::from(nx.max(1));
                let iy = y / f64::from(ny.max(1));
                let cov = f64::from(g.raster.cov[k]).max(0.1);
                let j = ix.hypot(iy) / (g.raster.h * g.t * cov);
                out.push((node, g.layer, j));
            }
        }
        out
    }
}

impl Network {
    /// Per layer, the box of the mesh cells carrying at least `frac` of the
    /// layer's peak current density: where the current actually flows.
    pub(crate) fn active_boxes(&self, s: &Solution, frac: f64) -> BTreeMap<u8, (P, P)> {
        let dens = self.cell_density(s);
        let mut peak: BTreeMap<u8, f64> = BTreeMap::new();
        for &(_, layer, j) in &dens {
            let e = peak.entry(layer.0).or_insert(0.0);
            if j.is_finite() && j > *e {
                *e = j;
            }
        }
        let mut out: BTreeMap<u8, (P, P)> = BTreeMap::new();
        for &(node, layer, j) in &dens {
            let p = peak.get(&layer.0).copied().unwrap_or(0.0);
            if p <= 0.0 || j.is_nan() || j < frac * p {
                continue;
            }
            let at = self.nodes[node as usize].at;
            let h = self.h;
            out.entry(layer.0)
                .and_modify(|(lo, hi)| {
                    *lo = [lo[0].min(at[0] - h), lo[1].min(at[1] - h)];
                    *hi = [hi[0].max(at[0] + h), hi[1].max(at[1] + h)];
                })
                .or_insert(([at[0] - h, at[1] - h], [at[0] + h, at[1] + h]));
        }
        out
    }
}

/// Parse `REF.PAD` names into board pad indices under `field`.
pub(crate) fn pads_named(
    board: &RouteBoard,
    names: &[String],
    field: &str,
) -> Result<Vec<usize>, crate::tools::route_native::Refusal> {
    crate::tools::board_inspect::named_pads(board, names, field)
}
