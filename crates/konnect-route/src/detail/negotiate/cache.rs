//! Reuse across tile builds.
//!
//! A tile's hard state — CORE/HALO from fixed copper and the commit-zone
//! wall, and the heading and via masks derived from them — depends only on
//! the tile's lattice and the clearance profile, never on routed copper or
//! history. The tiling of one parity is the same every iteration, so the
//! hard state of a (lattice, profile) is computed once and copied into
//! later builds ([`RasterCache`]); only soft paint and history are redone.
//! The cache is bounded in bytes and only ever changes speed: a hit copies
//! exactly the planes a fresh build would compute.
//!
//! Raster buffers themselves are recycled per worker ([`RasterPool`]), so a
//! build overwrites warm memory instead of allocating (and faulting in)
//! fresh planes.

use super::obstacles::ObstacleKind;
use super::Ctx;
use crate::api::{LayerId, NetId, ProfileId, TileId};
use crate::compute::{Plane, TileRaster, Window};
use std::collections::{BTreeMap, HashMap};
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, PoisonError};

/// Identifies a tile lattice independently of its tile id.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) struct LatticeKey {
    x: i32,
    y: i32,
    pitch: i32,
    cols: u32,
    rows: u32,
}

impl LatticeKey {
    pub fn of(w: &Window) -> Self {
        LatticeKey {
            x: w.origin.x,
            y: w.origin.y,
            pitch: w.pitch_nm,
            cols: w.cols,
            rows: w.rows,
        }
    }
}

/// The fixed obstacles that can reach a lattice, per routable layer slot,
/// and which of them belong to each net (for lifting a net's own copper).
#[derive(Debug)]
pub(crate) struct TileStatic {
    pub per_slot: Vec<Vec<u32>>,
    pub hard_own: BTreeMap<NetId, Vec<(u8, u32)>>,
}

impl TileStatic {
    fn build(ctx: &Ctx<'_>, lattice: &Window) -> Self {
        // Inflated for the widest profile of the run, so the list does not
        // depend on which profiles a build happens to need; obstacles out
        // of a narrower profile's reach paint nothing.
        let query = lattice.bbox().inflate(
            ctx.max_half_width + super::run::half_diag(lattice.pitch_nm) + lattice.pitch_nm,
        );
        let per_slot: Vec<Vec<u32>> = ctx
            .slots
            .iter()
            .map(|&l| ctx.obstacles.track_obstacles(l, &query))
            .collect();
        let mut hard_own: BTreeMap<NetId, Vec<(u8, u32)>> = BTreeMap::new();
        for (slot, ids) in per_slot.iter().enumerate() {
            for &id in ids {
                let o = &ctx.obstacles.items[id as usize];
                let own = matches!(o.kind, ObstacleKind::Copper(_) | ObstacleKind::Drill);
                if let (true, Some(n)) = (own, o.net) {
                    if o.layers.contains(ctx.slots[slot]) {
                        hard_own.entry(n).or_default().push((slot as u8, id));
                    }
                }
            }
        }
        TileStatic { per_slot, hard_own }
    }
}

/// The cached hard planes of one layer slot.
#[derive(Debug)]
pub(crate) struct HardLayer {
    pub core: Plane<u8>,
    pub halo: Plane<u8>,
    pub heading_mask: Plane<u8>,
    pub via_site: Plane<u8>,
}

impl HardLayer {
    pub fn capture(r: &TileRaster) -> Self {
        HardLayer {
            core: r.core.clone(),
            halo: r.halo.clone(),
            heading_mask: r.heading_mask.clone(),
            via_site: r.via_site.clone(),
        }
    }

    pub fn restore(&self, r: &mut TileRaster) {
        r.core.as_mut_slice().copy_from_slice(self.core.as_slice());
        r.halo.as_mut_slice().copy_from_slice(self.halo.as_slice());
        r.heading_mask
            .as_mut_slice()
            .copy_from_slice(self.heading_mask.as_slice());
        r.via_site
            .as_mut_slice()
            .copy_from_slice(self.via_site.as_slice());
    }

    fn bytes(&self) -> usize {
        self.core.as_slice().len()
            + self.halo.as_slice().len()
            + self.heading_mask.as_slice().len()
            + self.via_site.as_slice().len()
    }
}

type HardKey = (LatticeKey, u16);

/// Hard state and obstacle lists shared by every build of a run.
#[derive(Debug)]
pub(crate) struct RasterCache {
    statics: Mutex<HashMap<LatticeKey, Arc<TileStatic>>>,
    hard: Mutex<HashMap<HardKey, Arc<Vec<HardLayer>>>>,
    bytes: AtomicUsize,
    cap: usize,
    pub hits: AtomicU64,
    pub misses: AtomicU64,
}

impl RasterCache {
    /// A cache holding at most `cap` bytes of hard planes (0 disables it).
    pub fn new(cap: usize) -> Self {
        RasterCache {
            statics: Mutex::new(HashMap::new()),
            hard: Mutex::new(HashMap::new()),
            bytes: AtomicUsize::new(0),
            cap,
            hits: AtomicU64::new(0),
            misses: AtomicU64::new(0),
        }
    }

    pub fn tile_static(&self, ctx: &Ctx<'_>, lattice: &Window) -> Arc<TileStatic> {
        let key = LatticeKey::of(lattice);
        if let Some(s) = self
            .statics
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .get(&key)
        {
            return s.clone();
        }
        // Built outside the lock; a racing duplicate is equal and dropped.
        let built = Arc::new(TileStatic::build(ctx, lattice));
        self.statics
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .entry(key)
            .or_insert(built)
            .clone()
    }

    pub fn hard(&self, lattice: &Window, profile: u16) -> Option<Arc<Vec<HardLayer>>> {
        let hit = self
            .hard
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .get(&(LatticeKey::of(lattice), profile))
            .cloned();
        let counter = if hit.is_some() {
            &self.hits
        } else {
            &self.misses
        };
        counter.fetch_add(1, Ordering::Relaxed);
        hit
    }

    /// Keep `layers` for later builds if there is room.
    pub fn offer(&self, lattice: &Window, profile: u16, layers: impl FnOnce() -> Vec<HardLayer>) {
        if self.cap == 0 || self.bytes.load(Ordering::Relaxed) >= self.cap {
            return;
        }
        let layers = layers();
        let size: usize = layers.iter().map(HardLayer::bytes).sum();
        if self.bytes.fetch_add(size, Ordering::Relaxed) + size > self.cap {
            self.bytes.fetch_sub(size, Ordering::Relaxed);
            return;
        }
        let mut map = self.hard.lock().unwrap_or_else(PoisonError::into_inner);
        if map
            .insert((LatticeKey::of(lattice), profile), Arc::new(layers))
            .is_some()
        {
            // Another worker cached the same (equal) planes first.
            self.bytes.fetch_sub(size, Ordering::Relaxed);
        }
    }

    pub fn bytes(&self) -> usize {
        self.bytes.load(Ordering::Relaxed)
    }
}

/// Raster buffers a worker has finished with, for reuse.
#[derive(Debug, Default)]
pub(crate) struct RasterPool {
    free: Vec<TileRaster>,
}

/// Rasters a pool keeps at most (a few tiles' worth).
const POOL_MAX: usize = 48;

impl RasterPool {
    /// A raster for `window`; `true` when it is recycled, so its planes
    /// hold stale values the caller must overwrite.
    pub fn take(
        &mut self,
        tile: TileId,
        layer: LayerId,
        profile: ProfileId,
        window: Window,
    ) -> (TileRaster, bool) {
        let fits = |r: &TileRaster| r.window.cols == window.cols && r.window.rows == window.rows;
        match self.free.iter().rposition(fits) {
            Some(i) => {
                let mut r = self.free.swap_remove(i);
                r.tile = tile;
                r.layer = layer;
                r.profile = profile;
                r.window = window;
                (r, true)
            }
            None => (TileRaster::new(tile, layer, profile, window), false),
        }
    }

    pub fn give(&mut self, r: TileRaster) {
        if self.free.len() < POOL_MAX {
            self.free.push(r);
        }
    }
}
