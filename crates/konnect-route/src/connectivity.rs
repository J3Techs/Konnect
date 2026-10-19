//! S1 connectivity: which copper is already joined, and what is left to
//! route.
//!
//! Two items are joined when they share a net and a copper layer and their
//! exact shapes touch ([`konnect_geom::touches`]). Union-find over every
//! joined pair gives each net's *islands*; a net with `k` islands that hold
//! pads needs `k - 1` connections. This is the router's own count — it has
//! no 499-item cap and does not depend on KiCad's ratsnest.
//!
//! Items without a net never join anything: KiCad's unconnected pseudo-net
//! is not a net. Nor does an unfilled zone, whose outline is only an upper
//! bound on its copper; such pieces are counted in
//! [`ConnectivityReport::unfilled_zones_skipped`].

use crate::api::{Connection, ConnectionId, ItemRef, LayerId, NetId, PadId};
use crate::board::{PadKind, RouteBoard};
use konnect_geom::{touches, ArcPolicy, IndexEntry, LayerIndex, Shape};
use serde::{Deserialize, Serialize};

/// Disjoint sets with union by size and path halving. Deterministic: the
/// representative of a set depends only on the sequence of unions.
#[derive(Debug, Clone)]
pub struct UnionFind {
    parent: Vec<u32>,
    size: Vec<u32>,
}

impl UnionFind {
    pub fn new(n: usize) -> Self {
        UnionFind {
            parent: (0..n as u32).collect(),
            size: vec![1; n],
        }
    }

    pub fn len(&self) -> usize {
        self.parent.len()
    }

    pub fn is_empty(&self) -> bool {
        self.parent.is_empty()
    }

    pub fn find(&mut self, mut x: u32) -> u32 {
        while self.parent[x as usize] != x {
            let grandparent = self.parent[self.parent[x as usize] as usize];
            self.parent[x as usize] = grandparent;
            x = grandparent;
        }
        x
    }

    /// Join the sets holding `a` and `b`; `false` if they were already one.
    pub fn union(&mut self, a: u32, b: u32) -> bool {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra == rb {
            return false;
        }
        // Larger set wins; ties go to the smaller index, for determinism.
        let (big, small) = match self.size[ra as usize].cmp(&self.size[rb as usize]) {
            std::cmp::Ordering::Greater => (ra, rb),
            std::cmp::Ordering::Less => (rb, ra),
            std::cmp::Ordering::Equal => (ra.min(rb), ra.max(rb)),
        };
        self.parent[small as usize] = big;
        self.size[big as usize] += self.size[small as usize];
        true
    }

    pub fn same(&mut self, a: u32, b: u32) -> bool {
        self.find(a) == self.find(b)
    }
}

/// A maximal set of joined copper on one net.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Island {
    /// Members, sorted (pads first, then tracks, arcs, vias, zone pieces).
    pub items: Vec<ItemRef>,
}

impl Island {
    pub fn pads(&self) -> impl Iterator<Item = PadId> + '_ {
        self.items.iter().filter_map(|i| match i {
            ItemRef::Pad(p) => Some(*p),
            _ => None,
        })
    }

    pub fn has_pad(&self) -> bool {
        matches!(self.items.first(), Some(ItemRef::Pad(_)))
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct NetConnectivity {
    pub net: NetId,
    /// Islands ordered by their first member.
    pub islands: Vec<Island>,
    pub pad_count: usize,
}

impl NetConnectivity {
    /// Islands holding at least one pad — the ones routing must join.
    pub fn pad_island_count(&self) -> usize {
        self.islands.iter().filter(|i| i.has_pad()).count()
    }

    pub fn connections_needed(&self) -> usize {
        self.pad_island_count().saturating_sub(1)
    }
}

/// Every net's islands.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct ConnectivityReport {
    /// One entry per board net, by net id (nets with no copper have no
    /// islands).
    pub nets: Vec<NetConnectivity>,
    /// Netted zone pieces left out because the zone had no fill: their
    /// authored outline only bounds the copper a refill could pour, so it
    /// cannot prove that anything is joined.
    pub unfilled_zones_skipped: usize,
}

impl ConnectivityReport {
    /// Compute connectivity from the board's copper. Arc tracks are lowered
    /// under `policy`.
    pub fn build(board: &RouteBoard, policy: &ArcPolicy) -> Self {
        let mut nodes: Vec<(ItemRef, Option<NetId>)> = Vec::new();
        // Per layer: (node, shape).
        let mut per_layer: Vec<Vec<(u32, Shape)>> = vec![Vec::new(); board.layer_count()];
        let mut place = |layer: LayerId, node: u32, shape: Shape| {
            if let Some(list) = per_layer.get_mut(layer.index()) {
                list.push((node, shape));
            }
        };

        for pad in &board.pads {
            let node = nodes.len() as u32;
            nodes.push((ItemRef::Pad(pad.id), pad.net));
            if pad.net.is_some() {
                for layer in pad.layers.iter() {
                    for s in &pad.shapes {
                        place(layer, node, s.clone());
                    }
                }
            }
        }
        for t in &board.tracks {
            let node = nodes.len() as u32;
            nodes.push((ItemRef::Track(t.id), t.net));
            if t.net.is_some() {
                place(t.layer, node, Shape::Segment(t.seg));
            }
        }
        for a in &board.arcs {
            let node = nodes.len() as u32;
            nodes.push((ItemRef::Arc(a.id), a.net));
            if a.net.is_some() {
                for s in a.segments(policy) {
                    place(a.layer, node, Shape::Segment(s));
                }
            }
        }
        for v in &board.vias {
            let node = nodes.len() as u32;
            nodes.push((ItemRef::Via(v.id), v.net));
            if v.net.is_some() {
                for layer in v.layers.iter() {
                    place(layer, node, v.shape());
                }
            }
        }
        let mut unfilled_zones_skipped = 0;
        for z in &board.zones {
            if !z.filled {
                unfilled_zones_skipped += usize::from(z.net.is_some());
                continue;
            }
            let node = nodes.len() as u32;
            nodes.push((ItemRef::Zone(z.id), z.net));
            if z.net.is_some() {
                place(z.layer, node, Shape::Polygon(z.region.clone()));
            }
        }

        let mut uf = UnionFind::new(nodes.len());
        for shapes in &per_layer {
            let index = LayerIndex::bulk_load(
                shapes
                    .iter()
                    .enumerate()
                    .map(|(i, (_, s))| IndexEntry::new(i as u32, s.bbox()))
                    .collect(),
            );
            for (i, (node_i, shape_i)) in shapes.iter().enumerate() {
                let net_i = nodes[*node_i as usize].1;
                for j in index.query(&shape_i.bbox()) {
                    let (node_j, shape_j) = &shapes[j as usize];
                    if (j as usize) <= i
                        || nodes[*node_j as usize].1 != net_i
                        || uf.same(*node_i, *node_j)
                    {
                        continue;
                    }
                    if touches(shape_i, shape_j) {
                        uf.union(*node_i, *node_j);
                    }
                }
            }
        }

        // Group members by root, per net.
        let mut by_net: Vec<std::collections::BTreeMap<u32, Vec<ItemRef>>> =
            vec![Default::default(); board.nets.len()];
        let mut pad_counts = vec![0usize; board.nets.len()];
        for (i, (item, net)) in nodes.iter().enumerate() {
            let Some(net) = net else { continue };
            let Some(groups) = by_net.get_mut(net.index()) else {
                continue;
            };
            if matches!(item, ItemRef::Pad(_)) {
                pad_counts[net.index()] += 1;
            }
            groups.entry(uf.find(i as u32)).or_default().push(*item);
        }
        let nets = by_net
            .into_iter()
            .enumerate()
            .map(|(n, groups)| {
                let mut islands: Vec<Island> = groups
                    .into_values()
                    .map(|mut items| {
                        items.sort();
                        Island { items }
                    })
                    .collect();
                islands.sort_by(|a, b| a.items.cmp(&b.items));
                NetConnectivity {
                    net: NetId(n as u32),
                    islands,
                    pad_count: pad_counts[n],
                }
            })
            .collect();
        ConnectivityReport {
            nets,
            unfilled_zones_skipped,
        }
    }

    /// Nets with at least two pads.
    pub fn multi_pad_net_count(&self) -> usize {
        self.nets.iter().filter(|n| n.pad_count >= 2).count()
    }

    /// Total connections still needed across the board.
    pub fn connection_count(&self) -> usize {
        self.nets
            .iter()
            .map(NetConnectivity::connections_needed)
            .sum()
    }

    /// Connections still needed on plane nets (see [`plane_nets`]) and on
    /// the rest, in that order. They add up to [`Self::connection_count`].
    pub fn connection_split(&self, board: &RouteBoard) -> (usize, usize) {
        let planes = plane_net_mask(board);
        let mut plane = 0;
        let mut signal = 0;
        for n in &self.nets {
            if planes.get(n.net.index()).copied().unwrap_or(false) {
                plane += n.connections_needed();
            } else {
                signal += n.connections_needed();
            }
        }
        (plane, signal)
    }

    /// The state of a net with at least two pads; `None` for other nets.
    pub fn net_status(&self, board: &RouteBoard, net: NetId) -> Option<NetStatus> {
        let n = self.nets.get(net.index())?;
        if n.pad_count < 2 {
            return None;
        }
        Some(if n.pad_island_count() <= 1 {
            NetStatus::Complete
        } else if net_has_routing_copper(board)[net.index()] {
            NetStatus::Partial
        } else {
            NetStatus::NoCopper
        })
    }

    /// Complete, partial and copper-less nets among those with two or more
    /// pads, plus the connection totals.
    pub fn completeness(&self, board: &RouteBoard) -> Completeness {
        let copper = net_has_routing_copper(board);
        let mut out = Completeness::default();
        for n in self.nets.iter().filter(|n| n.pad_count >= 2) {
            out.multi_pad_nets += 1;
            if n.pad_island_count() <= 1 {
                out.complete += 1;
            } else if copper[n.net.index()] {
                out.partial += 1;
            } else {
                out.no_copper += 1;
            }
        }
        out.connections = self.connection_count();
        (out.plane_connections, out.signal_connections) = self.connection_split(board);
        out
    }

    /// For each pad, the index of its island in its net's
    /// [`NetConnectivity::islands`]; `None` for pads with no net.
    pub fn island_of_pads(&self, board: &RouteBoard) -> Vec<Option<u32>> {
        let mut out = vec![None; board.pads.len()];
        for n in &self.nets {
            for (i, island) in n.islands.iter().enumerate() {
                for p in island.pads() {
                    out[p.index()] = Some(i as u32);
                }
            }
        }
        out
    }

    /// Per plane net, which pads are joined to one of its filled plane
    /// pieces through copper, in net order.
    pub fn plane_joins(&self, board: &RouteBoard) -> Vec<PlaneJoin> {
        let mut out = Vec::new();
        for net in plane_nets(board) {
            let Some(n) = self.nets.get(net.index()) else {
                continue;
            };
            let mut join = PlaneJoin {
                net,
                pieces: board
                    .zones
                    .iter()
                    .filter(|z| z.filled && z.net == Some(net))
                    .map(|z| z.id)
                    .collect(),
                pads: n.pad_count,
                ..Default::default()
            };
            for island in &n.islands {
                let on_plane = island.items.iter().any(|i| matches!(i, ItemRef::Zone(_)));
                if on_plane {
                    join.plane_islands += 1;
                    join.joined_pads += island.pads().count();
                    continue;
                }
                if !island.has_pad() {
                    continue;
                }
                join.unjoined_islands += 1;
                let escaped = island
                    .items
                    .iter()
                    .any(|i| matches!(i, ItemRef::Track(_) | ItemRef::Arc(_) | ItemRef::Via(_)));
                join.unjoined_islands_with_copper += usize::from(escaped);
                for p in island.pads() {
                    join.unjoined_pads.push(p);
                    if board.pads[p.index()].kind == PadKind::Smd {
                        join.unjoined_smd_pads += 1;
                    }
                }
            }
            join.unjoined_pads.sort();
            out.push(join);
        }
        out
    }
}

/// Nets that own at least one filled zone piece: the planes.
pub fn plane_nets(board: &RouteBoard) -> Vec<NetId> {
    let mask = plane_net_mask(board);
    (0..board.nets.len())
        .filter(|&i| mask[i])
        .map(|i| NetId(i as u32))
        .collect()
}

fn plane_net_mask(board: &RouteBoard) -> Vec<bool> {
    let mut mask = vec![false; board.nets.len()];
    for z in board.zones.iter().filter(|z| z.filled) {
        if let Some(n) = z.net {
            if let Some(m) = mask.get_mut(n.index()) {
                *m = true;
            }
        }
    }
    mask
}

/// Nets with any track, arc or via: the copper a router makes. Zone fills
/// do not count, matching the HANDOFF definition of "no copper".
fn net_has_routing_copper(board: &RouteBoard) -> Vec<bool> {
    let mut has = vec![false; board.nets.len()];
    let nets = board
        .tracks
        .iter()
        .map(|t| t.net)
        .chain(board.arcs.iter().map(|a| a.net))
        .chain(board.vias.iter().map(|v| v.net));
    for n in nets.flatten() {
        if let Some(h) = has.get_mut(n.index()) {
            *h = true;
        }
    }
    has
}

/// How far a net with two or more pads is from routed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NetStatus {
    /// All pads in one island.
    Complete,
    /// Several pad islands, and the net has some track, arc or via.
    Partial,
    /// Several pad islands and no track, arc or via at all.
    NoCopper,
}

/// Board-wide completeness, over nets with at least two pads.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct Completeness {
    pub multi_pad_nets: usize,
    pub complete: usize,
    pub partial: usize,
    pub no_copper: usize,
    /// Connections still needed (pad islands minus one, summed).
    pub connections: usize,
    /// The part of `connections` on plane nets.
    pub plane_connections: usize,
    pub signal_connections: usize,
}

/// One plane net's pads against its plane pieces.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct PlaneJoin {
    pub net: NetId,
    /// The net's filled zone pieces.
    pub pieces: Vec<crate::api::ZoneId>,
    pub pads: usize,
    /// Islands holding a plane piece (more than one when the pieces are not
    /// joined to each other).
    pub plane_islands: usize,
    pub joined_pads: usize,
    /// Pads in islands that hold no plane piece, sorted.
    pub unjoined_pads: Vec<PadId>,
    pub unjoined_smd_pads: usize,
    /// Pad islands that hold no plane piece; a router needs one connection
    /// (a via, usually) for each.
    pub unjoined_islands: usize,
    /// Of those, islands that already carry a track, arc or via (escaped
    /// pads whose copper misses the plane).
    pub unjoined_islands_with_copper: usize,
}

impl ConnectivityReport {
    /// The routing task list: per net, a minimum spanning tree over its pad
    /// islands, each edge between the closest pad pair of two islands
    /// (Prim's algorithm; ties broken by island order). Deterministic.
    pub fn connections(&self, board: &RouteBoard) -> Vec<Connection> {
        let mut out = Vec::new();
        for net in &self.nets {
            let islands: Vec<Vec<PadId>> = net
                .islands
                .iter()
                .filter(|i| i.has_pad())
                .map(|i| i.pads().collect())
                .collect();
            let k = islands.len();
            if k < 2 {
                continue;
            }
            let closest = |a: &[PadId], b: &[PadId]| {
                let mut best: Option<(i128, PadId, PadId)> = None;
                for &pa in a {
                    for &pb in b {
                        let d = (board.pads[pa.index()].center - board.pads[pb.index()].center)
                            .length2();
                        if best.is_none_or(|(bd, _, _)| d < bd) {
                            best = Some((d, pa, pb));
                        }
                    }
                }
                best.expect("pad islands are non-empty")
            };
            let mut in_tree = vec![false; k];
            in_tree[0] = true;
            let mut best: Vec<Option<(i128, PadId, PadId)>> = (0..k)
                .map(|j| (j != 0).then(|| closest(&islands[0], &islands[j])))
                .collect();
            for _ in 1..k {
                let next = (0..k)
                    .filter(|&j| !in_tree[j])
                    .min_by_key(|&j| (best[j].map_or(i128::MAX, |b| b.0), j))
                    .expect("an island remains outside the tree");
                let (_, from, to) = best[next].expect("every outside island has a candidate edge");
                out.push(Connection {
                    id: ConnectionId(out.len() as u32),
                    net: net.net,
                    from: ItemRef::Pad(from),
                    to: ItemRef::Pad(to),
                });
                in_tree[next] = true;
                for j in 0..k {
                    if !in_tree[j] {
                        let cand = closest(&islands[next], &islands[j]);
                        if best[j].is_none_or(|b| cand.0 < b.0) {
                            best[j] = Some(cand);
                        }
                    }
                }
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synthetic;

    #[test]
    fn union_find_joins_and_is_deterministic() {
        let mut uf = UnionFind::new(5);
        assert!(uf.union(3, 4));
        assert!(uf.union(1, 3));
        assert!(!uf.union(4, 1));
        assert!(uf.same(1, 4));
        assert!(!uf.same(0, 1));
        let mut again = UnionFind::new(5);
        again.union(3, 4);
        again.union(1, 3);
        assert_eq!(uf.find(4), again.find(4));
    }

    #[test]
    fn a_track_joins_the_pads_it_touches() {
        let board = synthetic::two_pin_nets(3, true);
        let report = ConnectivityReport::build(&board, &ArcPolicy::default());
        // Net 0's pads are joined by a track; nets 1 and 2 are open.
        let n0 = &report.nets[0];
        assert_eq!(n0.pad_count, 2);
        assert_eq!(n0.pad_island_count(), 1);
        assert_eq!(report.nets[1].connections_needed(), 1);
        assert_eq!(report.connection_count(), 2);
        assert_eq!(report.multi_pad_net_count(), 3);
        let conns = report.connections(&board);
        assert_eq!(conns.len(), 2);
        assert_eq!(conns[0].id, ConnectionId(0));
        assert_eq!(conns[0].net, NetId(1));
    }

    #[test]
    fn a_plane_joins_through_hole_pads_but_not_smd_pads_on_other_layers() {
        let board = synthetic::plane_with_pads();
        let report = ConnectivityReport::build(&board, &ArcPolicy::default());
        let gnd = board.net_by_name("GND").unwrap();
        let net = &report.nets[gnd.index()];
        // Two TH pads join through the plane; the two SMD pads stay alone.
        assert_eq!(net.pad_count, 4);
        assert_eq!(net.pad_island_count(), 3);
        assert_eq!(net.connections_needed(), 2);
        assert_eq!(report.unfilled_zones_skipped, 0);
    }

    #[test]
    fn an_unfilled_zone_joins_nothing() {
        let mut board = synthetic::plane_with_pads();
        for z in &mut board.zones {
            z.filled = false;
        }
        let report = ConnectivityReport::build(&board, &ArcPolicy::default());
        let gnd = board.net_by_name("GND").unwrap();
        // The outline would join the TH pads; without a fill, all four are
        // apart.
        assert_eq!(report.nets[gnd.index()].pad_island_count(), 4);
        assert_eq!(report.unfilled_zones_skipped, board.zones.len());
    }
}
