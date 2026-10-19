//! Run telemetry: per-stage wall time, A* expansions per connection, window
//! sizes, congested-iteration share and raster rebuild time. Every go/no-go
//! gate (GPU G2, G-M1, G-Full) reads these numbers, so they are collected
//! from the first run, not added later.

use crate::api::Stage;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::time::Instant;

/// A fixed-bucket histogram: `counts[i]` holds samples `<= bounds[i]` (and
/// above the previous bound); the last count holds everything above the
/// last bound.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Histogram {
    pub bounds: Vec<u64>,
    pub counts: Vec<u64>,
    pub total: u64,
    pub sum: u64,
    pub max: u64,
}

impl Histogram {
    /// Bounds must be ascending.
    pub fn new(bounds: Vec<u64>) -> Self {
        debug_assert!(bounds.windows(2).all(|w| w[0] < w[1]));
        let n = bounds.len() + 1;
        Histogram {
            bounds,
            counts: vec![0; n],
            total: 0,
            sum: 0,
            max: 0,
        }
    }

    /// Powers of two from 1 to 2^`top`.
    pub fn powers_of_two(top: u32) -> Self {
        Histogram::new((0..=top).map(|i| 1u64 << i).collect())
    }

    pub fn record(&mut self, v: u64) {
        let i = self.bounds.partition_point(|&b| b < v);
        self.counts[i] += 1;
        self.total += 1;
        self.sum = self.sum.saturating_add(v);
        self.max = self.max.max(v);
    }

    pub fn mean(&self) -> f64 {
        if self.total == 0 {
            0.0
        } else {
            self.sum as f64 / self.total as f64
        }
    }
}

/// One stage's accumulated wall time.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct StageTiming {
    pub stage: Stage,
    pub wall_us: u64,
    pub runs: u32,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Telemetry {
    /// In first-recorded order.
    pub stages: Vec<StageTiming>,
    /// A* expansions per routed or failed connection.
    pub expansions_per_connection: Histogram,
    /// Lattice nodes per search window.
    pub window_nodes: Histogram,
    /// Wall time spent in negotiation iterations that still had conflicts.
    pub congested_iteration_us: u64,
    pub raster_rebuild_us: u64,
    /// Thread count the run used (recorded with every timing).
    pub threads: u32,
    /// Free-form named counters (`exact_checks`, `rip_ups`, …).
    pub counters: BTreeMap<String, u64>,
}

impl Default for Telemetry {
    fn default() -> Self {
        Telemetry {
            stages: Vec::new(),
            expansions_per_connection: Histogram::powers_of_two(24),
            window_nodes: Histogram::powers_of_two(24),
            congested_iteration_us: 0,
            raster_rebuild_us: 0,
            threads: 1,
            counters: BTreeMap::new(),
        }
    }
}

impl Telemetry {
    /// Run `f`, adding its wall time to `stage`.
    pub fn time<R>(&mut self, stage: Stage, f: impl FnOnce() -> R) -> R {
        let start = Instant::now();
        let out = f();
        self.add_stage_time(stage, start.elapsed().as_micros() as u64);
        out
    }

    pub fn add_stage_time(&mut self, stage: Stage, wall_us: u64) {
        match self.stages.iter_mut().find(|s| s.stage == stage) {
            Some(s) => {
                s.wall_us += wall_us;
                s.runs += 1;
            }
            None => self.stages.push(StageTiming {
                stage,
                wall_us,
                runs: 1,
            }),
        }
    }

    pub fn stage_us(&self, stage: Stage) -> u64 {
        self.stages
            .iter()
            .find(|s| s.stage == stage)
            .map_or(0, |s| s.wall_us)
    }

    pub fn total_us(&self) -> u64 {
        self.stages.iter().map(|s| s.wall_us).sum()
    }

    pub fn count(&mut self, name: &str, by: u64) {
        *self.counters.entry(name.to_string()).or_default() += by;
    }

    /// Fold another worker's telemetry into this one (stage times add, so a
    /// parallel stage should be timed once by its driver, not per worker).
    pub fn merge(&mut self, o: &Telemetry) {
        for s in &o.stages {
            match self.stages.iter_mut().find(|x| x.stage == s.stage) {
                Some(x) => {
                    x.wall_us += s.wall_us;
                    x.runs += s.runs;
                }
                None => self.stages.push(*s),
            }
        }
        for (mine, theirs) in [
            (
                &mut self.expansions_per_connection,
                &o.expansions_per_connection,
            ),
            (&mut self.window_nodes, &o.window_nodes),
        ] {
            if mine.bounds == theirs.bounds {
                for (a, b) in mine.counts.iter_mut().zip(&theirs.counts) {
                    *a += b;
                }
                mine.total += theirs.total;
                mine.sum = mine.sum.saturating_add(theirs.sum);
                mine.max = mine.max.max(theirs.max);
            }
        }
        self.congested_iteration_us += o.congested_iteration_us;
        self.raster_rebuild_us += o.raster_rebuild_us;
        for (k, v) in &o.counters {
            *self.counters.entry(k.clone()).or_default() += v;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn histogram_buckets_are_upper_inclusive() {
        let mut h = Histogram::new(vec![1, 10, 100]);
        for v in [0, 1, 2, 10, 11, 100, 1_000] {
            h.record(v);
        }
        assert_eq!(h.counts, vec![2, 2, 2, 1]);
        assert_eq!(h.total, 7);
        assert_eq!(h.max, 1_000);
    }

    #[test]
    fn stage_times_accumulate_and_merge() {
        let mut t = Telemetry::default();
        t.add_stage_time(Stage::Detail, 5);
        t.add_stage_time(Stage::Detail, 7);
        t.add_stage_time(Stage::Ingest, 1);
        assert_eq!(t.stage_us(Stage::Detail), 12);
        let value = t.time(Stage::Check, || 42);
        assert_eq!(value, 42);
        let mut other = Telemetry::default();
        other.add_stage_time(Stage::Detail, 3);
        other.expansions_per_connection.record(5);
        other.count("rip_ups", 2);
        t.merge(&other);
        assert_eq!(t.stage_us(Stage::Detail), 15);
        assert_eq!(t.expansions_per_connection.total, 1);
        assert_eq!(t.counters["rip_ups"], 2);
        assert_eq!(t.stages[0].stage, Stage::Detail);
    }
}
