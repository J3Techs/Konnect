//! Deterministic batch builder (owned by WS-C).
//!
//! A batch is a set of jobs that may run concurrently against one frozen
//! snapshot of the rasters: their windows are pairwise disjoint (bbox-
//! disjoint, halos included), so no job can see another's uncommitted
//! copper. Batches run in index order; inside a batch, results commit in
//! `order_key` order. That schedule — frozen snapshot, disjoint windows,
//! ordered commits (Gort & Anderson, FPT 2010) — is what makes a 1-thread
//! and a 20-thread run produce byte-identical plans, and it is the same
//! schedule a GPU dispatch uses.

use crate::compute::job::JobPool;
use crate::compute::raster::Window;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BatchPolicy {
    /// Upper bound on jobs per batch (rayon chunking or GPU dispatch size).
    pub max_jobs: usize,
    /// Only jobs of one `size_class` share a batch, so GPU work groups are
    /// uniform.
    pub split_by_size_class: bool,
}

impl Default for BatchPolicy {
    fn default() -> Self {
        BatchPolicy {
            max_jobs: 256,
            split_by_size_class: false,
        }
    }
}

/// Jobs that run together; indices into `JobPool::jobs`, ascending by
/// `order_key`.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Batch {
    pub index: u32,
    pub jobs: Vec<u32>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct BatchBuilder {
    pub policy: BatchPolicy,
}

impl BatchBuilder {
    pub fn new(policy: BatchPolicy) -> Self {
        BatchBuilder { policy }
    }

    /// Partition every job in `pool` into batches.
    ///
    /// Contract (frozen):
    /// - the output depends only on `pool`, `windows` and the policy — never
    ///   on thread count, hash order or timing;
    /// - every job appears in exactly one batch;
    /// - within a batch, the jobs' windows (`windows[job.window]`) are
    ///   pairwise bbox-disjoint;
    /// - jobs are greedily placed in ascending `order_key`, each into the
    ///   first batch it fits, so cheap early jobs are never starved;
    /// - batch indices are dense from 0.
    pub fn build(&self, pool: &JobPool, windows: &[Window]) -> Vec<Batch> {
        let _ = (pool, windows);
        todo!("WS-C: deterministic greedy first-fit batching over disjoint windows")
    }
}
