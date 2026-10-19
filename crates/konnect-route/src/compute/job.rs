//! 64-byte job descriptors and their shared endpoint pool.
//!
//! A [`JobDescriptor`] is one connection's search request, fixed-size and
//! `#[repr(C)]` so a batch is a flat array a GPU can read directly. Variable
//! data (source and target nodes, corridor cells) lives in a [`JobPool`] and
//! is referenced by `(first, count)` ranges.

use crate::api::{ConnectionId, NetId, ProfileId, TileId};
use crate::compute::cost::Cost;

/// The job may change layers.
pub const JOB_ALLOW_VIA: u8 = 1 << 0;
/// The net is half of a differential pair.
pub const JOB_DIFF_PAIR: u8 = 1 << 1;
/// A minimum straight run is required between turns (controlled-impedance
/// classes with a segment-length rule).
pub const JOB_MIN_RUN: u8 = 1 << 2;
/// The job is being rerouted after rip-up.
pub const JOB_REROUTE: u8 = 1 << 3;

/// One connection's search request. Exactly 64 bytes.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct JobDescriptor {
    pub connection: u32,
    pub net: u32,
    /// Window (tile) index the search runs in.
    pub window: u32,
    pub profile: u16,
    /// Coarse size class (log2 of the window node count), for batching.
    pub size_class: u8,
    /// `JOB_*` bits.
    pub flags: u8,
    /// Layers the search may use (`LayerMask` bits).
    pub layer_mask: u32,
    pub sources_first: u32,
    pub sources_count: u32,
    pub targets_first: u32,
    pub targets_count: u32,
    /// Corridor GCell ids from the global stage; `count == 0` means
    /// unconstrained within the window.
    pub corridor_first: u32,
    pub corridor_count: u32,
    /// Give up above this cost (`u32::MAX`: no bound).
    pub cost_bound: u32,
    /// Total order for commits: lower first. Ties are impossible by
    /// construction (it embeds the connection id).
    pub order_key: u32,
    /// Negotiation iteration that issued the job.
    pub iteration: u32,
    pub reserved: [u32; 2],
}

const _: () = assert!(std::mem::size_of::<JobDescriptor>() == 64);
const _: () = assert!(std::mem::align_of::<JobDescriptor>() == 4);

impl JobDescriptor {
    pub fn connection_id(&self) -> ConnectionId {
        ConnectionId(self.connection)
    }

    pub fn net_id(&self) -> NetId {
        NetId(self.net)
    }

    pub fn tile(&self) -> TileId {
        TileId(self.window)
    }

    pub fn profile_id(&self) -> ProfileId {
        ProfileId(self.profile)
    }

    pub fn bound(&self) -> Cost {
        Cost(self.cost_bound)
    }

    pub fn has_flag(&self, flag: u8) -> bool {
        self.flags & flag != 0
    }
}

/// A search endpoint on a window's lattice. Exactly 8 bytes.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, PartialOrd, Ord)]
pub struct JobNode {
    pub col: u16,
    pub row: u16,
    /// Index into the tile stack's layers.
    pub layer: u8,
    /// Headings allowed to leave (sources) or enter (targets) this node;
    /// `0xff` for any.
    pub heading_mask: u8,
    pub reserved: u16,
}

const _: () = assert!(std::mem::size_of::<JobNode>() == 8);

/// Every job of a run plus the shared variable-length data.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct JobPool {
    pub jobs: Vec<JobDescriptor>,
    pub nodes: Vec<JobNode>,
    pub corridor: Vec<u32>,
}

impl JobPool {
    pub fn sources(&self, job: &JobDescriptor) -> &[JobNode] {
        let a = job.sources_first as usize;
        &self.nodes[a..a + job.sources_count as usize]
    }

    pub fn targets(&self, job: &JobDescriptor) -> &[JobNode] {
        let a = job.targets_first as usize;
        &self.nodes[a..a + job.targets_count as usize]
    }

    pub fn corridor_of(&self, job: &JobDescriptor) -> &[u32] {
        let a = job.corridor_first as usize;
        &self.corridor[a..a + job.corridor_count as usize]
    }

    /// Append a job with its endpoints and corridor, filling in the ranges.
    /// Returns the job's index.
    pub fn push(
        &mut self,
        mut job: JobDescriptor,
        sources: &[JobNode],
        targets: &[JobNode],
        corridor: &[u32],
    ) -> u32 {
        job.sources_first = self.nodes.len() as u32;
        job.sources_count = sources.len() as u32;
        self.nodes.extend_from_slice(sources);
        job.targets_first = self.nodes.len() as u32;
        job.targets_count = targets.len() as u32;
        self.nodes.extend_from_slice(targets);
        job.corridor_first = self.corridor.len() as u32;
        job.corridor_count = corridor.len() as u32;
        self.corridor.extend_from_slice(corridor);
        self.jobs.push(job);
        (self.jobs.len() - 1) as u32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pool_ranges_round_trip() {
        let mut pool = JobPool::default();
        let a = JobNode {
            col: 1,
            row: 2,
            layer: 0,
            heading_mask: 0xff,
            reserved: 0,
        };
        let b = JobNode { col: 9, ..a };
        let i = pool.push(
            JobDescriptor {
                connection: 5,
                ..Default::default()
            },
            &[a],
            &[b, a],
            &[10, 11, 12],
        );
        let job = pool.jobs[i as usize];
        assert_eq!(pool.sources(&job), &[a]);
        assert_eq!(pool.targets(&job), &[b, a]);
        assert_eq!(pool.corridor_of(&job), &[10, 11, 12]);
        assert_eq!(job.connection_id(), ConnectionId(5));
    }
}
