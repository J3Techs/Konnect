//! Frozen identifiers, enums and constants shared by every stage.
//!
//! **Interface freeze.** Everything in this module is the contract the
//! parallel workstreams build against. Changes go through the integrator
//! (see `docs/native-router/INTERFACES.md`): add, never silently repurpose.

use konnect_geom::Coord;
use serde::{Deserialize, Serialize};

macro_rules! id_type {
    ($(#[$doc:meta])* $name:ident($inner:ty)) => {
        $(#[$doc])*
        #[derive(
            Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Default, Serialize, Deserialize,
        )]
        #[serde(transparent)]
        pub struct $name(pub $inner);

        impl $name {
            /// The id as a `Vec` index.
            pub fn index(self) -> usize {
                self.0 as usize
            }
        }
    };
}

id_type!(
    /// Index into `RouteBoard::nets`. Assigned in sorted net-name order, so
    /// the same board always gets the same ids.
    NetId(u32)
);
id_type!(
    /// Index into `RouteBoard::footprints`, in file order.
    FootprintId(u32)
);
id_type!(
    /// Index into `RouteBoard::pads`, in file order.
    PadId(u32)
);
id_type!(
    /// Index into `RouteBoard::tracks`.
    TrackId(u32)
);
id_type!(
    /// Index into `RouteBoard::arcs`.
    ArcId(u32)
);
id_type!(
    /// Index into `RouteBoard::vias`.
    ViaId(u32)
);
id_type!(
    /// Index into `RouteBoard::zones` (one entry per plane piece).
    ZoneId(u32)
);
id_type!(
    /// Index into `RouteBoard::keepouts`.
    KeepoutId(u32)
);
id_type!(
    /// Index into `RouteBoard::layers.copper`, in physical stack order:
    /// `F.Cu` is 0 and `B.Cu` is the last.
    LayerId(u8)
);
id_type!(
    /// Index into the rule table's net classes; 0 is always `Default`.
    ClassId(u16)
);
id_type!(
    /// One routing task: a pair of islands of the same net to join.
    ConnectionId(u32)
);
id_type!(
    /// A detailed-routing tile (CPU) or window (GPU).
    TileId(u32)
);
id_type!(
    /// A clearance profile: one (net class, rule region) combination the
    /// rasters are built for.
    ProfileId(u16)
);
id_type!(
    /// A negotiation generation; routed copper remembers which one made it.
    Generation(u32)
);

/// A set of copper layers, bit `i` for [`LayerId`]`(i)`. KiCad has at most
/// 32 copper layers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(transparent)]
pub struct LayerMask(pub u32);

impl LayerMask {
    pub const EMPTY: LayerMask = LayerMask(0);

    /// The mask of one layer; empty for an id beyond 31.
    pub fn single(layer: LayerId) -> Self {
        LayerMask(1u32.checked_shl(u32::from(layer.0)).unwrap_or(0))
    }

    /// Every layer from `a` to `b` inclusive, in either order.
    pub fn span(a: LayerId, b: LayerId) -> Self {
        let (lo, hi) = (a.0.min(b.0), a.0.max(b.0));
        (lo..=hi).fold(LayerMask::EMPTY, |m, l| m.with(LayerId(l)))
    }

    pub fn contains(self, layer: LayerId) -> bool {
        self.intersects(LayerMask::single(layer))
    }

    pub fn with(self, layer: LayerId) -> Self {
        self.union(LayerMask::single(layer))
    }

    pub fn union(self, o: LayerMask) -> Self {
        LayerMask(self.0 | o.0)
    }

    pub fn intersection(self, o: LayerMask) -> Self {
        LayerMask(self.0 & o.0)
    }

    pub fn intersects(self, o: LayerMask) -> bool {
        self.0 & o.0 != 0
    }

    pub fn is_empty(self) -> bool {
        self.0 == 0
    }

    pub fn count(self) -> u32 {
        self.0.count_ones()
    }

    /// Layers in ascending id order.
    pub fn iter(self) -> impl Iterator<Item = LayerId> {
        (0u8..32)
            .filter(move |l| self.0 & (1u32 << l) != 0)
            .map(LayerId)
    }
}

/// Who owns a piece of copper, and therefore who may move it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Ownership {
    /// Existing copper the router treats as a hard obstacle (and terminal)
    /// but that KiCad does not lock.
    Fixed,
    /// Copper KiCad marks `(locked yes)`: never moved, never ripped up.
    Locked,
    /// Copper this router produced. Only the generation that owns it may rip
    /// it up; later generations see it as soft (present/history cost).
    Routed { net: NetId, gen: Generation },
}

impl Ownership {
    /// Whether the router may rip this copper up.
    pub fn is_routed(self) -> bool {
        matches!(self, Ownership::Routed { .. })
    }
}

/// The item kinds KiCad's DRC distinguishes (`A.Type == '…'`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ItemKind {
    Track,
    Arc,
    Via,
    Pad,
    Zone,
}

impl ItemKind {
    /// The string KiCad's DRC `Type` property yields for this kind.
    pub fn kicad_type_name(self) -> &'static str {
        match self {
            ItemKind::Track => "Track",
            ItemKind::Arc => "Arc",
            ItemKind::Via => "Via",
            ItemKind::Pad => "Pad",
            ItemKind::Zone => "Zone",
        }
    }
}

/// A reference to any copper item on the board.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(tag = "kind", content = "id", rename_all = "snake_case")]
pub enum ItemRef {
    Pad(PadId),
    Track(TrackId),
    Arc(ArcId),
    Via(ViaId),
    Zone(ZoneId),
}

impl ItemRef {
    pub fn kind(self) -> ItemKind {
        match self {
            ItemRef::Pad(_) => ItemKind::Pad,
            ItemRef::Track(_) => ItemKind::Track,
            ItemRef::Arc(_) => ItemKind::Arc,
            ItemRef::Via(_) => ItemKind::Via,
            ItemRef::Zone(_) => ItemKind::Zone,
        }
    }
}

/// One routing task: join the island holding `from` to the island holding
/// `to`, both on `net`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Connection {
    pub id: ConnectionId,
    pub net: NetId,
    pub from: ItemRef,
    pub to: ItemRef,
}

/// Why a connection was not routed. Reported, never silently dropped.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(tag = "reason", rename_all = "snake_case")]
pub enum FailureReason {
    /// A terminal has no legal exit on any routable layer (S1/S4).
    NoAccess { item: ItemRef },
    /// The global stage found no corridor within capacity (S3).
    NoCorridor,
    /// Negotiation gave up with the connection still in conflict (S5).
    Congestion { iterations: u32 },
    /// A rule the router cannot evaluate governs this connection.
    UnsupportedRule { rule: String },
    /// The search budget ran out before a path was found.
    BudgetExhausted,
}

/// The pipeline stages, for telemetry and reports.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Stage {
    /// S0: build the board model.
    Ingest,
    /// S1: connectivity and terminal-access audit.
    Audit,
    /// S2: plane fanout.
    Fanout,
    /// S3: global routing.
    Global,
    /// S4: pin access.
    Access,
    /// S5: detailed routing.
    Detail,
    /// S6: optimization.
    Optimize,
    /// S7: internal exact DRC.
    Check,
    /// S8: plan, apply and KiCad DRC.
    Plan,
}

// ─── Frozen lattice and window constants ─────────────────────────────────────

/// Detailed-routing lattice base pitch (S5): 50 µm.
pub const LATTICE_PITCH_NM: Coord = 50_000;
/// Refined lattice pitch in fine-pitch BGA fields: 25 µm.
pub const LATTICE_FINE_PITCH_NM: Coord = 25_000;
/// Global-routing GCell pitch (S3): 1 mm.
pub const GCELL_PITCH_NM: Coord = 1_000_000;
/// Refined GCell pitch in dense fields: 0.5 mm.
pub const GCELL_FINE_PITCH_NM: Coord = 500_000;
/// CPU detailed tile core size, width × height: 20 × 15 mm.
pub const TILE_CORE_NM: (Coord, Coord) = (20_000_000, 15_000_000);
/// Halo around each CPU tile core: 2 mm.
pub const TILE_HALO_NM: Coord = 2_000_000;
/// Largest GPU window extent including halo: 2^24 nm (about 16.78 mm), so
/// tile-local `f32` coordinates are exact.
pub const GPU_WINDOW_MAX_NM: Coord = 1 << 24;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn layer_mask_span_and_iteration() {
        let m = LayerMask::span(LayerId(3), LayerId(1));
        assert_eq!(
            m.iter().collect::<Vec<_>>(),
            vec![LayerId(1), LayerId(2), LayerId(3)]
        );
        assert!(m.contains(LayerId(2)));
        assert!(!m.contains(LayerId(0)));
        assert_eq!(m.count(), 3);
        assert!(!m.contains(LayerId(200)));
    }

    #[test]
    fn ownership_serializes_with_a_kind_tag() {
        let routed = Ownership::Routed {
            net: NetId(4),
            gen: Generation(2),
        };
        assert_eq!(
            serde_json::to_value(routed).unwrap(),
            serde_json::json!({ "kind": "routed", "net": 4, "gen": 2 })
        );
        assert_eq!(
            serde_json::to_value(Ownership::Locked).unwrap(),
            serde_json::json!({ "kind": "locked" })
        );
    }

    #[test]
    fn gpu_window_keeps_f32_exact() {
        // Every integer up to 2^24 is exactly representable in f32.
        let max = GPU_WINDOW_MAX_NM as f32;
        assert_eq!(max as i32, GPU_WINDOW_MAX_NM);
        assert_eq!((GPU_WINDOW_MAX_NM - 1) as f32 as i32, GPU_WINDOW_MAX_NM - 1);
    }
}
