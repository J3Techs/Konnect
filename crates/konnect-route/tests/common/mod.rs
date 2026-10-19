//! Helpers shared by the real-board fixture tests.

use konnect_geom::{touches, ArcPolicy, BBox, Circle, IndexEntry, LayerIndex, Point, Shape};
use konnect_route::api::{LayerMask, NetId};
use konnect_route::plan::RoutedCopper;
use konnect_route::RouteBoard;
use std::collections::{BTreeMap, BTreeSet};

/// One piece of copper as KiCad's connectivity sees it.
struct Item {
    net: NetId,
    layers: LayerMask,
    shapes: Vec<Shape>,
    anchors: Vec<Point>,
    pad: bool,
}

fn find(parent: &mut [usize], mut x: usize) -> usize {
    while parent[x] != x {
        parent[x] = parent[parent[x]];
        x = parent[x];
    }
    x
}

/// Per net, how many separate islands its pads fall into when items join
/// only as KiCad joins them: an *anchor* of one (a track end, a via centre,
/// a one-shape pad's copper centre) inside the other's copper on a shared
/// layer. Zones are left out, so plane nets count their pads as joined by
/// tracks and vias alone. Stricter than the overlap union-find of
/// `ConnectivityReport`: a track that only grazes a pad does not join it.
pub fn anchor_islands(board: &RouteBoard, copper: &RoutedCopper) -> BTreeMap<NetId, usize> {
    let (items, mut parent) = joined_items(board, copper);
    let mut roots: BTreeMap<NetId, BTreeSet<usize>> = BTreeMap::new();
    for (i, it) in items.iter().enumerate() {
        if it.pad {
            let r = find(&mut parent, i);
            roots.entry(it.net).or_default().insert(r);
        }
    }
    roots.into_iter().map(|(n, r)| (n, r.len())).collect()
}

fn joined_items(board: &RouteBoard, copper: &RoutedCopper) -> (Vec<Item>, Vec<usize>) {
    let policy = ArcPolicy::default();
    let mut items: Vec<Item> = Vec::new();
    for p in &board.pads {
        let Some(net) = p.net else { continue };
        let anchors = if p.shapes.len() == 1 {
            vec![p.shapes[0].bbox().center()]
        } else {
            Vec::new()
        };
        items.push(Item {
            net,
            layers: p.layers,
            shapes: p.shapes.clone(),
            anchors,
            pad: true,
        });
    }
    for v in &board.vias {
        let Some(net) = v.net else { continue };
        items.push(Item {
            net,
            layers: v.layers,
            shapes: vec![v.shape()],
            anchors: vec![v.center],
            pad: false,
        });
    }
    for t in &board.tracks {
        let Some(net) = t.net else { continue };
        items.push(Item {
            net,
            layers: LayerMask::EMPTY.with(t.layer),
            shapes: vec![Shape::Segment(t.seg)],
            anchors: vec![t.seg.a, t.seg.b],
            pad: false,
        });
    }
    for a in &board.arcs {
        let Some(net) = a.net else { continue };
        items.push(Item {
            net,
            layers: LayerMask::EMPTY.with(a.layer),
            shapes: a
                .segments(&policy)
                .into_iter()
                .map(Shape::Segment)
                .collect(),
            anchors: vec![a.start, a.end],
            pad: false,
        });
    }
    for t in &copper.tracks {
        items.push(Item {
            net: t.net,
            layers: LayerMask::EMPTY.with(t.layer),
            shapes: vec![Shape::Segment(t.seg)],
            anchors: vec![t.seg.a, t.seg.b],
            pad: false,
        });
    }
    for v in &copper.vias {
        items.push(Item {
            net: v.net,
            layers: v.layers,
            shapes: vec![Shape::Circle(Circle::from_diameter(v.center, v.diameter))],
            anchors: vec![v.center],
            pad: false,
        });
    }

    let layer_count = board
        .layers
        .all()
        .iter()
        .map(|l| l.index() + 1)
        .max()
        .unwrap_or(0);
    let mut per_layer: Vec<Vec<IndexEntry>> = vec![Vec::new(); layer_count];
    for (i, it) in items.iter().enumerate() {
        let Some(bbox) = it
            .shapes
            .iter()
            .map(Shape::bbox)
            .reduce(|a: BBox, b| a.union(b))
        else {
            continue;
        };
        for l in it.layers.iter() {
            if let Some(v) = per_layer.get_mut(l.index()) {
                v.push(IndexEntry::new(i as u32, bbox.inflate(1)));
            }
        }
    }
    let index: Vec<LayerIndex> = per_layer.into_iter().map(LayerIndex::bulk_load).collect();

    let mut parent: Vec<usize> = (0..items.len()).collect();
    for (i, it) in items.iter().enumerate() {
        for l in it.layers.iter() {
            let Some(idx) = index.get(l.index()) else {
                continue;
            };
            for &p in &it.anchors {
                let dot = Shape::Circle(Circle::new(p, 0));
                for j in idx.query(&BBox::point(p)) {
                    let j = j as usize;
                    let other = &items[j];
                    if j == i || other.net != it.net || !other.layers.contains(l) {
                        continue;
                    }
                    if other.shapes.iter().any(|s| touches(&dot, s)) {
                        let (a, b) = (find(&mut parent, i), find(&mut parent, j));
                        parent[a] = b;
                    }
                }
            }
        }
    }
    (items, parent)
}

/// Nets with more anchor islands in `after` than in `before`.
pub fn nets_split(
    before: &BTreeMap<NetId, usize>,
    after: &BTreeMap<NetId, usize>,
) -> Vec<(NetId, usize, usize)> {
    after
        .iter()
        .filter_map(|(n, &a)| {
            let b = before.get(n).copied().unwrap_or(0);
            (a > b).then_some((*n, b, a))
        })
        .collect()
}
