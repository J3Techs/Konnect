//! A cross-search cache in front of an [`ExactCheck`].
//!
//! The search already caches exact verdicts within one search. Across
//! searches, the same edges come back: every connection of a multi-pin net
//! leaves the same pads, and negotiation reroutes the same connections
//! iteration after iteration while the hard copper they are checked
//! against stays put. [`CheckCache`] remembers each verdict per
//! `(net, profile)`, sharded so parallel tiles rarely contend.
//!
//! A verdict is only valid while the hard obstacles the inner checker sees
//! are unchanged: drop or [`CheckCache::clear`] the cache when fixed copper
//! changes (a new fanout commit), not on rip-up of routed copper, which
//! the inner checker does not treat as hard.

use super::ExactCheck;
use crate::compute::JobDescriptor;
use konnect_geom::Point;
use std::collections::HashMap;
use std::hash::BuildHasher;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Mutex;

const SHARDS: usize = 64;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum Key {
    /// Endpoints in a canonical order (an edge is checked either way).
    Segment {
        net: u32,
        profile: u16,
        layer: u8,
        a: (i32, i32),
        b: (i32, i32),
    },
    Via {
        net: u32,
        profile: u16,
        at: (i32, i32),
    },
}

/// Hit and miss counts.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct CacheStats {
    pub hits: u64,
    pub misses: u64,
}

pub struct CheckCache<'a> {
    inner: &'a dyn ExactCheck,
    shards: Vec<Mutex<HashMap<Key, bool>>>,
    hasher: std::collections::hash_map::RandomState,
    hits: AtomicU64,
    misses: AtomicU64,
}

impl<'a> CheckCache<'a> {
    pub fn new(inner: &'a dyn ExactCheck) -> Self {
        CheckCache {
            inner,
            shards: (0..SHARDS).map(|_| Mutex::new(HashMap::new())).collect(),
            hasher: std::collections::hash_map::RandomState::new(),
            hits: AtomicU64::new(0),
            misses: AtomicU64::new(0),
        }
    }

    /// Forget every verdict (after the hard copper changed).
    pub fn clear(&self) {
        for s in &self.shards {
            s.lock().unwrap_or_else(|e| e.into_inner()).clear();
        }
    }

    pub fn stats(&self) -> CacheStats {
        CacheStats {
            hits: self.hits.load(Ordering::Relaxed),
            misses: self.misses.load(Ordering::Relaxed),
        }
    }

    pub fn len(&self) -> usize {
        self.shards
            .iter()
            .map(|s| s.lock().unwrap_or_else(|e| e.into_inner()).len())
            .sum()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    fn lookup(&self, key: Key, compute: impl FnOnce() -> bool) -> bool {
        let shard = &self.shards[(self.hasher.hash_one(key) as usize) % SHARDS];
        if let Some(&v) = shard.lock().unwrap_or_else(|e| e.into_inner()).get(&key) {
            self.hits.fetch_add(1, Ordering::Relaxed);
            return v;
        }
        // Compute outside the lock; a racing duplicate computes the same
        // verdict, so the double insert is harmless.
        let v = compute();
        self.misses.fetch_add(1, Ordering::Relaxed);
        shard
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .insert(key, v);
        v
    }
}

impl ExactCheck for CheckCache<'_> {
    fn segment_ok(&self, job: &JobDescriptor, layer: u8, a: Point, b: Point) -> bool {
        let (pa, pb) = ((a.x, a.y), (b.x, b.y));
        let (a2, b2) = if pa <= pb { (pa, pb) } else { (pb, pa) };
        let key = Key::Segment {
            net: job.net,
            profile: job.profile,
            layer,
            a: a2,
            b: b2,
        };
        self.lookup(key, || self.inner.segment_ok(job, layer, a, b))
    }

    fn via_ok(&self, job: &JobDescriptor, at: Point) -> bool {
        let key = Key::Via {
            net: job.net,
            profile: job.profile,
            at: (at.x, at.y),
        };
        self.lookup(key, || self.inner.via_ok(job, at))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicU32;

    struct Counting {
        calls: AtomicU32,
    }

    impl ExactCheck for Counting {
        fn segment_ok(&self, _j: &JobDescriptor, _l: u8, a: Point, _b: Point) -> bool {
            self.calls.fetch_add(1, Ordering::Relaxed);
            a.x >= 0
        }
        fn via_ok(&self, _j: &JobDescriptor, at: Point) -> bool {
            self.calls.fetch_add(1, Ordering::Relaxed);
            at.y >= 0
        }
    }

    #[test]
    fn verdicts_are_cached_per_net_in_either_direction() {
        let inner = Counting {
            calls: AtomicU32::new(0),
        };
        let cache = CheckCache::new(&inner);
        let j = JobDescriptor {
            net: 3,
            ..Default::default()
        };
        let (a, b) = (Point::new(-5, 0), Point::new(10, 0));
        assert!(!cache.segment_ok(&j, 0, a, b));
        assert!(cache.segment_ok(&j, 0, b, a) == cache.segment_ok(&j, 0, a, b));
        assert_eq!(inner.calls.load(Ordering::Relaxed), 1);
        // Another net or layer is another question.
        let k = JobDescriptor { net: 4, ..j };
        cache.segment_ok(&k, 0, a, b);
        cache.segment_ok(&j, 1, a, b);
        assert_eq!(inner.calls.load(Ordering::Relaxed), 3);
        assert!(cache.via_ok(&j, Point::new(0, 1)));
        assert!(cache.via_ok(&j, Point::new(0, 1)));
        assert_eq!(inner.calls.load(Ordering::Relaxed), 4);
        assert_eq!(cache.stats(), CacheStats { hits: 3, misses: 4 });
        assert_eq!(cache.len(), 4);
        cache.clear();
        assert!(cache.is_empty());
    }
}
