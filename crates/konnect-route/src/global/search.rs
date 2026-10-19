//! The global cost model and the bounded A* maze search.

use super::grid::{Topology, Win, NONE};
use super::lookahead::Lookahead;
use std::cmp::Reverse;
use std::collections::BinaryHeap;

/// Cost factor of a zero-capacity resource at a terminal cell.
pub(crate) const ACCESS_FACTOR: f32 = 2.0;

/// A frozen view of the congestion state plus the cost knobs of one
/// iteration. Shared read-only by every search of a batch.
pub(crate) struct Costs<'a> {
    pub topo: &'a Topology,
    pub cap: &'a [u16],
    pub occ: &'a [u16],
    pub hist: &'a [f32],
    pub look: &'a Lookahead,
    /// Per slot: cost factor of a horizontal and of a vertical step.
    pub dir: &'a [[f32; 2]],
    pub via: f32,
    pub pres_fac: f32,
    pub seed_k: f32,
    pub same_net: f32,
    pub astar: f32,
}

impl Costs<'_> {
    /// Cost of using resource `r` with base cost `base` for a net of demand
    /// `w`. A resource the net already holds is discounted and adds no
    /// demand.
    #[inline]
    pub(crate) fn resource(&self, r: usize, base: f32, w: u16, owned: bool) -> f32 {
        if owned {
            return base * self.same_net;
        }
        let cap = i32::from(self.cap[r]);
        if cap == 0 {
            // Only reachable at the connection's own terminal cells: pin
            // access the grid cannot resolve, so it carries no congestion.
            return base * ACCESS_FACTOR;
        }
        let over = (i32::from(self.occ[r]) + i32::from(w) - cap).max(0) as f32;
        let p = 1.0 + self.pres_fac * over + self.seed_k * over / cap.max(1) as f32;
        base * (1.0 + self.hist[r]) * p
    }

    #[inline]
    pub(crate) fn planar_base(&self, slot: u32, e: u32) -> f32 {
        let d = &self.dir[slot as usize];
        self.topo.edge_len[e as usize]
            * if self.topo.edge_horiz[e as usize] {
                d[0]
            } else {
                d[1]
            }
    }

    /// The lookahead bound from `node` to the target box and slots.
    #[inline]
    pub(crate) fn heuristic(&self, node: u32, tbox: &(i32, i32, i32, i32), tslots: u32) -> f32 {
        let (slot, c) = self.topo.split(node);
        let (x, y) = self.topo.center(c);
        let dx = (i64::from(tbox.0) - i64::from(x))
            .max(i64::from(x) - i64::from(tbox.2))
            .max(0);
        let dy = (i64::from(tbox.1) - i64::from(y))
            .max(i64::from(y) - i64::from(tbox.3))
            .max(0);
        let p = self.topo.pitch;
        self.look
            .to_any(tslots, slot as usize, (dx / p) as usize, (dy / p) as usize)
            * self.astar
    }
}

/// One search: where from, where to, and what it may use.
pub(crate) struct Request<'a> {
    /// Source nodes with their starting cost.
    pub sources: &'a [(u32, f32)],
    pub targets: &'a [u32],
    /// Planar cells where zero-capacity resources stay passable (the
    /// connection's own terminal cells).
    pub term_cells: &'a [u32],
    pub win: Win,
    /// Allowed slots, bit per slot.
    pub slots: u32,
    pub weight: u16,
    pub via_weight: u16,
    /// Resources the net already holds through its other connections.
    pub owned: &'a [u32],
    /// Bounding box of the target cell centres and their slots, for the
    /// heuristic.
    pub tbox: (i32, i32, i32, i32),
    pub tslots: u32,
}

/// Per-thread search state, sized for the whole grid and reused through
/// epoch stamps so no search clears or allocates.
pub(crate) struct Scratch {
    g: Vec<f32>,
    parent: Vec<u32>,
    seen: Vec<u32>,
    closed: Vec<u32>,
    target: Vec<u32>,
    pub(crate) owned: Vec<u32>,
    pub(crate) term: Vec<u32>,
    /// Heuristic cache, valid where `hseen` holds the epoch.
    h: Vec<f32>,
    hseen: Vec<u32>,
    epoch: u32,
    heap: BinaryHeap<Reverse<(u32, u32, u32)>>,
}

impl Scratch {
    pub(crate) fn new(nodes: usize, resources: usize, cells: usize) -> Scratch {
        Scratch {
            g: vec![0.0; nodes],
            parent: vec![NONE; nodes],
            seen: vec![0; nodes],
            closed: vec![0; nodes],
            target: vec![0; nodes],
            owned: vec![0; resources],
            term: vec![0; cells],
            h: vec![0.0; nodes],
            hseen: vec![0; nodes],
            epoch: 0,
            heap: BinaryHeap::new(),
        }
    }

    /// Start a new search and stamp its owned resources and terminal cells.
    pub(crate) fn begin(&mut self, owned: &[u32], term_cells: &[u32]) -> u32 {
        if self.epoch == u32::MAX {
            self.seen.fill(0);
            self.closed.fill(0);
            self.target.fill(0);
            self.owned.fill(0);
            self.term.fill(0);
            self.hseen.fill(0);
            self.epoch = 0;
        }
        self.epoch += 1;
        let ep = self.epoch;
        for &r in owned {
            self.owned[r as usize] = ep;
        }
        for &c in term_cells {
            self.term[c as usize] = ep;
        }
        ep
    }

    #[inline]
    pub(crate) fn is_owned(&self, r: u32, ep: u32) -> bool {
        self.owned[r as usize] == ep
    }

    #[inline]
    pub(crate) fn is_term(&self, c: u32, ep: u32) -> bool {
        self.term[c as usize] == ep
    }
}

/// Outcome of one maze search.
pub(crate) struct Found {
    pub path: Vec<u32>,
}

/// Bounded multi-source, multi-target A*. `None` when no target is
/// reachable inside the window; the expansion count is returned either way.
pub(crate) fn astar(c: &Costs, req: &Request, s: &mut Scratch) -> (Option<Found>, u64) {
    let topo = c.topo;
    let ep = s.begin(req.owned, req.term_cells);
    for &t in req.targets {
        s.target[t as usize] = ep;
    }
    s.heap.clear();
    for &(n, g0) in req.sources {
        let ni = n as usize;
        if s.seen[ni] == ep && s.g[ni] <= g0 {
            continue;
        }
        s.seen[ni] = ep;
        s.g[ni] = g0;
        s.parent[ni] = NONE;
        let f = g0 + cached_h(c, req, s, ep, n);
        s.heap.push(Reverse((f.to_bits(), n, g0.to_bits())));
    }
    let w = req.weight;
    let via_base = c.via;
    let mut expansions = 0u64;
    while let Some(Reverse((_, n, gb))) = s.heap.pop() {
        let ni = n as usize;
        if s.g[ni].to_bits() != gb || s.closed[ni] == ep {
            continue;
        }
        // Expanded nodes stay closed: with a weighted or slightly
        // inconsistent heuristic, re-opening costs far more than the
        // occasional longer path saves.
        s.closed[ni] = ep;
        if s.target[ni] == ep {
            let mut path = vec![n];
            let mut cur = s.parent[ni];
            while cur != NONE {
                path.push(cur);
                cur = s.parent[cur as usize];
            }
            path.reverse();
            return (Some(Found { path }), expansions);
        }
        expansions += 1;
        let g = s.g[ni];
        let (slot, cell) = topo.split(n);
        let cell_term = s.is_term(cell, ep);
        let (a, b) = (
            topo.adj_off[cell as usize] as usize,
            topo.adj_off[cell as usize + 1] as usize,
        );
        for i in a..b {
            let m_cell = topo.adj_cell[i];
            if !topo.in_window(m_cell, &req.win) {
                continue;
            }
            let e = topo.adj_edge[i];
            let r = topo.planar_res(slot, e);
            let owned = s.is_owned(r, ep);
            if c.cap[r as usize] == 0 && !owned && !cell_term && !s.is_term(m_cell, ep) {
                continue;
            }
            let ng = g + c.resource(r as usize, c.planar_base(slot, e), w, owned);
            relax(c, req, s, ep, n, topo.node(slot, m_cell), ng);
        }
        let r = topo.via_res(cell);
        let owned = s.is_owned(r, ep);
        if c.cap[r as usize] == 0 && !owned && !cell_term {
            continue;
        }
        let vc = c.resource(r as usize, via_base, req.via_weight, owned);
        for ns in [slot.wrapping_sub(1), slot + 1] {
            if ns < topo.slots && req.slots & (1 << ns) != 0 {
                relax(c, req, s, ep, n, topo.node(ns, cell), g + vc);
            }
        }
    }
    (None, expansions)
}

#[inline]
fn relax(c: &Costs, req: &Request, s: &mut Scratch, ep: u32, from: u32, m: u32, ng: f32) {
    let mi = m as usize;
    if s.closed[mi] == ep || (s.seen[mi] == ep && s.g[mi] <= ng) {
        return;
    }
    s.seen[mi] = ep;
    s.g[mi] = ng;
    s.parent[mi] = from;
    let f = ng + cached_h(c, req, s, ep, m);
    s.heap.push(Reverse((f.to_bits(), m, ng.to_bits())));
}

#[inline]
fn cached_h(c: &Costs, req: &Request, s: &mut Scratch, ep: u32, n: u32) -> f32 {
    let i = n as usize;
    if s.hseen[i] != ep {
        s.hseen[i] = ep;
        s.h[i] = c.heuristic(n, &req.tbox, req.tslots);
    }
    s.h[i]
}

/// The resources a node path uses, sorted and de-duplicated: planar edges
/// for steps within a slot, the via site for a layer change.
pub(crate) fn path_resources(topo: &Topology, path: &[u32], out: &mut Vec<u32>) {
    out.clear();
    for w in path.windows(2) {
        let (sa, ca) = topo.split(w[0]);
        let (sb, cb) = topo.split(w[1]);
        if sa == sb {
            let e = topo
                .edge_between(ca, cb)
                .expect("consecutive path cells are adjacent");
            out.push(topo.planar_res(sa, e));
        } else {
            out.push(topo.via_res(ca));
        }
    }
    out.sort_unstable();
    out.dedup();
}
