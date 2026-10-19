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
use konnect_geom::BBox;

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
    ///
    /// Windows are closed boxes: two that touch overlap. Equal `order_key`s
    /// are ordered by job index. A job whose `window` indexes no window
    /// cannot be shown disjoint from anything, so it runs alone.
    /// `max_jobs == 0` is treated as 1.
    pub fn build(&self, pool: &JobPool, windows: &[Window]) -> Vec<Batch> {
        let max_jobs = self.policy.max_jobs.max(1);
        let mut order: Vec<u32> = (0..pool.jobs.len() as u32).collect();
        order.sort_by_key(|&j| (pool.jobs[j as usize].order_key, j));

        let mut open: Vec<Open> = Vec::new();
        // Batches that may still take jobs, in creation order, so full ones
        // are never scanned again.
        let mut live: Vec<usize> = Vec::new();
        for j in order {
            let job = &pool.jobs[j as usize];
            let Some(b) = windows.get(job.window as usize).map(Window::bbox) else {
                open.push(Open::sealed(j));
                continue;
            };
            let class = job.size_class;
            let split = self.policy.split_by_size_class;
            let hit = live.iter().position(|&k| {
                let o = &open[k];
                (!split || o.class == class) && o.fits(&b)
            });
            match hit {
                Some(pos) => {
                    let k = live[pos];
                    open[k].add(j, &b);
                    if open[k].jobs.len() >= max_jobs {
                        live.remove(pos);
                    }
                }
                None => {
                    let mut o = Open::new(class);
                    o.add(j, &b);
                    open.push(o);
                    if max_jobs > 1 {
                        live.push(open.len() - 1);
                    }
                }
            }
        }
        open.into_iter()
            .enumerate()
            .map(|(i, o)| Batch {
                index: i as u32,
                jobs: o.jobs,
            })
            .collect()
    }
}

/// A batch under construction: its jobs and their windows as
/// structure-of-arrays boxes, plus their union for a quick accept.
struct Open {
    jobs: Vec<u32>,
    class: u8,
    x0: Vec<i32>,
    y0: Vec<i32>,
    x1: Vec<i32>,
    y1: Vec<i32>,
    union: Option<BBox>,
}

impl Open {
    fn new(class: u8) -> Self {
        Open {
            jobs: Vec::new(),
            class,
            x0: Vec::new(),
            y0: Vec::new(),
            x1: Vec::new(),
            y1: Vec::new(),
            union: None,
        }
    }

    /// A batch holding one job that nothing may join (it is never live).
    fn sealed(job: u32) -> Self {
        let mut o = Open::new(u8::MAX);
        o.jobs.push(job);
        o
    }

    fn add(&mut self, job: u32, b: &BBox) {
        self.jobs.push(job);
        self.x0.push(b.min.x);
        self.y0.push(b.min.y);
        self.x1.push(b.max.x);
        self.y1.push(b.max.y);
        self.union = Some(self.union.map_or(*b, |u| u.union(*b)));
    }

    /// Whether `b` is disjoint from every window in the batch.
    fn fits(&self, b: &BBox) -> bool {
        match &self.union {
            None => return true,
            Some(u) if !u.intersects(b) => return true,
            _ => {}
        }
        let (bx0, by0, bx1, by1) = (b.min.x, b.min.y, b.max.x, b.max.y);
        !(0..self.jobs.len()).any(|i| {
            self.x0[i] <= bx1 && bx0 <= self.x1[i] && self.y0[i] <= by1 && by0 <= self.y1[i]
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::compute::job::JobDescriptor;
    use konnect_geom::Point;

    fn window(x: i32, y: i32, w: u32, h: u32) -> Window {
        Window::new(Point::new(x, y), 10, w, h).unwrap()
    }

    /// `(order_key, window, size_class)` per job.
    fn pool_of(keys: &[(u32, u32, u8)]) -> JobPool {
        let mut pool = JobPool::default();
        for &(order_key, window, size_class) in keys {
            pool.push(
                JobDescriptor {
                    connection: order_key,
                    window,
                    order_key,
                    size_class,
                    ..Default::default()
                },
                &[],
                &[],
                &[],
            );
        }
        pool
    }

    fn check_contract(pool: &JobPool, windows: &[Window], batches: &[Batch]) {
        let mut seen = vec![0u32; pool.jobs.len()];
        for (i, b) in batches.iter().enumerate() {
            assert_eq!(b.index as usize, i);
            assert!(!b.jobs.is_empty());
            for w in b.jobs.windows(2) {
                assert!(pool.jobs[w[0] as usize].order_key < pool.jobs[w[1] as usize].order_key);
            }
            for (x, &a) in b.jobs.iter().enumerate() {
                seen[a as usize] += 1;
                for &c in &b.jobs[x + 1..] {
                    let wa = windows[pool.jobs[a as usize].window as usize].bbox();
                    let wc = windows[pool.jobs[c as usize].window as usize].bbox();
                    assert!(!wa.intersects(&wc), "jobs {a} and {c} overlap in batch {i}");
                }
            }
        }
        assert!(seen.iter().all(|&n| n == 1));
    }

    #[test]
    fn disjoint_jobs_share_a_batch_and_overlaps_split() {
        let windows = vec![
            window(0, 0, 5, 5),
            window(100, 0, 5, 5),
            window(20, 20, 5, 5),
            window(110, 10, 5, 5),
        ];
        let pool = pool_of(&[(0, 0, 0), (1, 1, 0), (2, 2, 0), (3, 3, 0)]);
        let batches = BatchBuilder::default().build(&pool, &windows);
        check_contract(&pool, &windows, &batches);
        // Windows 0 and 2 overlap (0..40 vs 20..60); 1 and 3 do too.
        assert_eq!(batches.len(), 2);
        assert_eq!(batches[0].jobs, vec![0, 1]);
        assert_eq!(batches[1].jobs, vec![2, 3]);
    }

    #[test]
    fn touching_windows_conflict() {
        // 0..40 and 40..80 share the column of nodes at x = 40.
        let windows = vec![window(0, 0, 5, 5), window(40, 0, 5, 5), window(50, 0, 5, 5)];
        let pool = pool_of(&[(0, 0, 0), (1, 1, 0), (2, 2, 0)]);
        let batches = BatchBuilder::default().build(&pool, &windows);
        check_contract(&pool, &windows, &batches);
        assert_eq!(batches[0].jobs, vec![0, 2]);
        assert_eq!(batches[1].jobs, vec![1]);
    }

    #[test]
    fn first_fit_follows_order_key_not_insertion_order() {
        let windows = vec![
            window(0, 0, 5, 5),
            window(0, 0, 5, 5),
            window(1000, 0, 5, 5),
        ];
        // Job 0 has the highest key: jobs 1 and 2 come first.
        let pool = pool_of(&[(9, 0, 0), (1, 1, 0), (2, 2, 0)]);
        let batches = BatchBuilder::default().build(&pool, &windows);
        check_contract(&pool, &windows, &batches);
        assert_eq!(batches[0].jobs, vec![1, 2]);
        assert_eq!(batches[1].jobs, vec![0]);
    }

    #[test]
    fn max_jobs_and_size_classes_bound_batches() {
        let windows: Vec<Window> = (0..10).map(|i| window(i * 100, 0, 3, 3)).collect();
        let keys: Vec<(u32, u32, u8)> = (0..10).map(|i| (i, i, (i % 2) as u8)).collect();
        let pool = pool_of(&keys);
        let b = BatchBuilder::new(BatchPolicy {
            max_jobs: 3,
            split_by_size_class: false,
        })
        .build(&pool, &windows);
        check_contract(&pool, &windows, &b);
        assert_eq!(
            b.iter().map(|x| x.jobs.len()).collect::<Vec<_>>(),
            vec![3, 3, 3, 1]
        );
        let b = BatchBuilder::new(BatchPolicy {
            max_jobs: 256,
            split_by_size_class: true,
        })
        .build(&pool, &windows);
        check_contract(&pool, &windows, &b);
        assert_eq!(b.len(), 2);
        assert_eq!(b[0].jobs, vec![0, 2, 4, 6, 8]);
        assert_eq!(b[1].jobs, vec![1, 3, 5, 7, 9]);
    }

    #[test]
    fn a_job_without_a_window_runs_alone() {
        let windows = vec![window(0, 0, 5, 5), window(100, 0, 5, 5)];
        let pool = pool_of(&[(0, 0, 0), (1, 7, 0), (2, 1, 0)]);
        let b = BatchBuilder::default().build(&pool, &windows);
        assert_eq!(b.len(), 2);
        assert_eq!(b[0].jobs, vec![0, 2]);
        assert_eq!(b[1].jobs, vec![1]);
        let one = BatchBuilder::new(BatchPolicy {
            max_jobs: 0,
            split_by_size_class: false,
        })
        .build(&pool, &windows);
        assert!(one.iter().all(|x| x.jobs.len() == 1));
        assert_eq!(one.len(), 3);
    }

    #[test]
    fn output_is_a_pure_function_of_its_inputs_and_first_fit() {
        let mut windows = Vec::new();
        let mut keys = Vec::new();
        let mut s = 12345u64;
        for i in 0..500u32 {
            s = s
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            let x = ((s >> 33) % 2000) as i32;
            let y = ((s >> 13) % 2000) as i32;
            let w = 1 + ((s >> 50) % 20) as u32;
            windows.push(window(x, y, w, w));
            keys.push((1000 - i, i, 0u8));
        }
        let pool = pool_of(&keys);
        // No size bound, so only overlaps decide placement.
        let builder = BatchBuilder::new(BatchPolicy {
            max_jobs: usize::MAX,
            split_by_size_class: false,
        });
        let a = builder.build(&pool, &windows);
        let b = builder.build(&pool, &windows);
        assert_eq!(a, b);
        check_contract(&pool, &windows, &a);
        // First fit: every job conflicts with some earlier-keyed job of
        // every earlier batch.
        for (bi, batch) in a.iter().enumerate() {
            for &j in &batch.jobs {
                let wj = windows[pool.jobs[j as usize].window as usize].bbox();
                let key = pool.jobs[j as usize].order_key;
                for earlier in &a[..bi] {
                    let blocked = earlier.jobs.iter().any(|&o| {
                        pool.jobs[o as usize].order_key < key
                            && windows[pool.jobs[o as usize].window as usize]
                                .bbox()
                                .intersects(&wj)
                    });
                    assert!(blocked, "job {j} fit an earlier batch");
                }
            }
        }
    }
}
