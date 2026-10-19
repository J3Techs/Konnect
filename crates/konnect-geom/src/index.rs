//! Per-layer spatial index over inflated bounding boxes.
//!
//! The index answers "which objects might be within reach of this box?" and
//! nothing more; the exact predicates decide. Entries are stored inflated by
//! the caller's chosen reach (typically the largest clearance in the rule
//! table), so one box query finds every object a rule could involve.
//!
//! Results come back as sorted ids. R-tree traversal order depends on the
//! tree's shape, which depends on insertion history; sorting makes every
//! consumer deterministic regardless of how the tree was built.

use crate::bbox::BBox;
use rstar::{RTree, RTreeObject, AABB};

/// One indexed object: a caller-owned id and its (already inflated) box.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct IndexEntry {
    pub id: u32,
    pub bbox: BBox,
}

impl IndexEntry {
    pub fn new(id: u32, bbox: BBox) -> Self {
        IndexEntry { id, bbox }
    }
}

impl RTreeObject for IndexEntry {
    // i64 envelopes: rstar computes areas and margins of boxes internally,
    // and a board-sized box in nm overflows i32 arithmetic.
    type Envelope = AABB<[i64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        AABB::from_corners(
            [i64::from(self.bbox.min.x), i64::from(self.bbox.min.y)],
            [i64::from(self.bbox.max.x), i64::from(self.bbox.max.y)],
        )
    }
}

fn envelope(b: &BBox) -> AABB<[i64; 2]> {
    AABB::from_corners(
        [i64::from(b.min.x), i64::from(b.min.y)],
        [i64::from(b.max.x), i64::from(b.max.y)],
    )
}

/// The R-tree for one copper layer (or for holes, which span layers).
#[derive(Debug, Clone, Default)]
pub struct LayerIndex {
    tree: RTree<IndexEntry>,
}

impl LayerIndex {
    pub fn new() -> Self {
        LayerIndex { tree: RTree::new() }
    }

    /// Build in one pass; much faster than repeated inserts for a whole
    /// board's fixed copper.
    pub fn bulk_load(entries: Vec<IndexEntry>) -> Self {
        LayerIndex {
            tree: RTree::bulk_load(entries),
        }
    }

    pub fn insert(&mut self, entry: IndexEntry) {
        self.tree.insert(entry);
    }

    /// Remove an entry previously inserted with exactly this id and box.
    /// Returns whether it was present — rip-up of routed copper must find
    /// what it stamped.
    pub fn remove(&mut self, entry: &IndexEntry) -> bool {
        self.tree.remove(entry).is_some()
    }

    pub fn len(&self) -> usize {
        self.tree.size()
    }

    pub fn is_empty(&self) -> bool {
        self.tree.size() == 0
    }

    /// Ids of every entry whose box intersects `bbox` (touching counts),
    /// sorted ascending and de-duplicated.
    pub fn query(&self, bbox: &BBox) -> Vec<u32> {
        let mut ids: Vec<u32> = self.query_entries(bbox).map(|e| e.id).collect();
        ids.sort_unstable();
        ids.dedup();
        ids
    }

    /// The matching entries themselves, in tree order (not deterministic
    /// across differently built trees — sort before acting on the order).
    pub fn query_entries<'a>(&'a self, bbox: &BBox) -> impl Iterator<Item = &'a IndexEntry> + 'a {
        self.tree.locate_in_envelope_intersecting(&envelope(bbox))
    }
}

/// One [`LayerIndex`] per copper layer, indexed by the router's layer id.
#[derive(Debug, Clone, Default)]
pub struct LayeredIndex {
    layers: Vec<LayerIndex>,
}

impl LayeredIndex {
    pub fn new(layer_count: usize) -> Self {
        LayeredIndex {
            layers: (0..layer_count).map(|_| LayerIndex::new()).collect(),
        }
    }

    pub fn layer_count(&self) -> usize {
        self.layers.len()
    }

    /// `None` for a layer id beyond the stack.
    pub fn layer(&self, layer: usize) -> Option<&LayerIndex> {
        self.layers.get(layer)
    }

    pub fn layer_mut(&mut self, layer: usize) -> Option<&mut LayerIndex> {
        self.layers.get_mut(layer)
    }

    /// Replace one layer's tree, e.g. after a bulk load.
    pub fn set_layer(&mut self, layer: usize, index: LayerIndex) {
        if let Some(slot) = self.layers.get_mut(layer) {
            *slot = index;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::point::Point;

    fn b(x0: i32, y0: i32, x1: i32, y1: i32) -> BBox {
        BBox::new(Point::new(x0, y0), Point::new(x1, y1))
    }

    #[test]
    fn query_returns_sorted_intersecting_ids() {
        let idx = LayerIndex::bulk_load(vec![
            IndexEntry::new(7, b(0, 0, 10, 10)),
            IndexEntry::new(3, b(5, 5, 20, 20)),
            IndexEntry::new(9, b(100, 100, 110, 110)),
        ]);
        assert_eq!(idx.query(&b(8, 8, 9, 9)), vec![3, 7]);
        assert_eq!(idx.query(&b(10, 10, 10, 10)), vec![3, 7]);
        assert!(idx.query(&b(50, 50, 60, 60)).is_empty());
    }

    #[test]
    fn board_sized_boxes_do_not_overflow() {
        let mut idx = LayerIndex::new();
        idx.insert(IndexEntry::new(1, b(0, 0, 440_000_000, 300_000_000)));
        idx.insert(IndexEntry::new(
            2,
            b(-1_000_000_000, -1_000_000_000, 1_000_000_000, 1_000_000_000),
        ));
        assert_eq!(idx.query(&b(1, 1, 2, 2)), vec![1, 2]);
    }

    #[test]
    fn remove_finds_what_was_inserted() {
        let mut idx = LayerIndex::new();
        let e = IndexEntry::new(4, b(0, 0, 1, 1));
        idx.insert(e);
        assert_eq!(idx.len(), 1);
        assert!(idx.remove(&e));
        assert!(!idx.remove(&e));
        assert!(idx.is_empty());
    }

    #[test]
    fn layered_index_is_per_layer() {
        let mut idx = LayeredIndex::new(3);
        idx.layer_mut(2)
            .unwrap()
            .insert(IndexEntry::new(1, b(0, 0, 5, 5)));
        assert!(idx.layer(0).unwrap().query(&b(0, 0, 5, 5)).is_empty());
        assert_eq!(idx.layer(2).unwrap().query(&b(0, 0, 5, 5)), vec![1]);
        assert!(idx.layer(3).is_none());
    }
}
