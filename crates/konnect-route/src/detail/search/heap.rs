//! A monotone radix heap over `u32` keys (Ahuja, Mehlhorn, Orlin and Tarjan,
//! 1990), the priority queue integer-cost A* needs.
//!
//! Keys must never drop below the last popped key. A* with a consistent
//! heuristic guarantees that; a weighted or map lookahead may not, so
//! [`RadixHeap::push`] clamps such a key up to the last popped one, which
//! only reorders that entry among ties. Buckets keep their capacity across
//! [`RadixHeap::clear`], so a warmed-up heap never allocates.
//!
//! Within one key, entries pop last-in first-out. That favours the deepest
//! state among equal `f`, which is what A* wants, and it is deterministic.

/// One bucket per possible highest differing bit, plus bucket 0 for keys
/// equal to the last popped key.
const BUCKETS: usize = 33;

#[derive(Debug, Clone)]
pub struct RadixHeap {
    last: u32,
    len: usize,
    buckets: [Vec<(u32, u32)>; BUCKETS],
}

impl Default for RadixHeap {
    fn default() -> Self {
        RadixHeap {
            last: 0,
            len: 0,
            buckets: std::array::from_fn(|_| Vec::new()),
        }
    }
}

#[inline]
fn bucket_of(key: u32, last: u32) -> usize {
    (32 - (key ^ last).leading_zeros()) as usize
}

impl RadixHeap {
    pub fn new() -> Self {
        Self::default()
    }

    /// Empty the heap and reset its floor to 0, keeping bucket capacity.
    pub fn clear(&mut self) {
        for b in &mut self.buckets {
            b.clear();
        }
        self.last = 0;
        self.len = 0;
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// The key of the last pop (the heap's floor).
    pub fn floor(&self) -> u32 {
        self.last
    }

    /// Push `value` with `key`, clamped up to the floor.
    #[inline]
    pub fn push(&mut self, key: u32, value: u32) {
        let key = key.max(self.last);
        self.buckets[bucket_of(key, self.last)].push((key, value));
        self.len += 1;
    }

    /// Pop an entry with the smallest key.
    #[inline]
    pub fn pop(&mut self) -> Option<(u32, u32)> {
        if self.buckets[0].is_empty() {
            if self.len == 0 {
                return None;
            }
            let i = (1..BUCKETS).find(|&i| !self.buckets[i].is_empty())?;
            let min = self.buckets[i].iter().map(|e| e.0).min()?;
            self.last = min;
            // Every entry of bucket i now lands in a strictly lower bucket.
            let mut moved = std::mem::take(&mut self.buckets[i]);
            for &(k, v) in &moved {
                self.buckets[bucket_of(k, min)].push((k, v));
            }
            moved.clear();
            self.buckets[i] = moved;
        }
        let e = self.buckets[0].pop()?;
        self.len -= 1;
        Some(e)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pops_in_key_order_and_lifo_within_a_key() {
        let mut h = RadixHeap::new();
        for (k, v) in [(5, 0), (3, 1), (9, 2), (3, 3), (1_000_000, 4), (4, 5)] {
            h.push(k, v);
        }
        let mut out = Vec::new();
        while let Some(e) = h.pop() {
            out.push(e);
        }
        assert_eq!(
            out,
            vec![(3, 3), (3, 1), (4, 5), (5, 0), (9, 2), (1_000_000, 4)]
        );
        assert!(h.is_empty());
    }

    #[test]
    fn keys_below_the_floor_are_clamped() {
        let mut h = RadixHeap::new();
        h.push(10, 0);
        assert_eq!(h.pop(), Some((10, 0)));
        h.push(7, 1);
        h.push(12, 2);
        assert_eq!(h.pop(), Some((10, 1)));
        assert_eq!(h.pop(), Some((12, 2)));
        assert_eq!(h.pop(), None);
    }

    #[test]
    fn interleaved_pushes_match_a_sorted_reference() {
        // A monotone workload: every push is >= the last pop.
        let mut h = RadixHeap::new();
        let mut reference: Vec<(u32, u32)> = Vec::new();
        let mut x: u64 = 12345;
        let mut floor = 0u32;
        for i in 0..5_000u32 {
            x = x
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            let k = floor + (x >> 40) as u32 % 5_000;
            h.push(k, i);
            reference.push((k, i));
            if i % 3 == 0 {
                let (pk, _) = h.pop().unwrap();
                let min = reference.iter().map(|e| e.0).min().unwrap();
                assert_eq!(pk, min);
                let pos = reference.iter().position(|e| e.0 == pk).unwrap();
                reference.swap_remove(pos);
                floor = pk;
            }
        }
        while let Some((k, _)) = h.pop() {
            let min = reference.iter().map(|e| e.0).min().unwrap();
            assert_eq!(k, min);
            let pos = reference.iter().position(|e| e.0 == k).unwrap();
            reference.swap_remove(pos);
        }
        assert!(reference.is_empty());
    }

    #[test]
    fn clear_keeps_capacity() {
        let mut h = RadixHeap::new();
        for i in 0..100 {
            h.push(i * 7, i);
        }
        while h.pop().is_some() {}
        let cap: usize = h.buckets.iter().map(Vec::capacity).sum();
        h.clear();
        assert_eq!(h.buckets.iter().map(Vec::capacity).sum::<usize>(), cap);
        assert_eq!(h.floor(), 0);
    }
}
