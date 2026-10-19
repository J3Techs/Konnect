//! Exact integer board geometry for the Konnect native router.
//!
//! # Numeric model
//!
//! Board coordinates are [`Coord`] = `i32` nanometres, which is KiCad's own
//! internal unit, so a value read from a board and written back through IPC
//! is exact. Arithmetic on coordinates widens: differences are `i64`, dot and
//! cross products `i128`, and the few products of squares a distance test
//! needs are compared in 256 bits (see [`predicates`]). Nothing that decides
//! legality uses floating point; `f64` appears only at the edges — converting
//! millimetres, rotating KiCad's pad frames, and approximating arcs under an
//! explicit [`ArcPolicy`].
//!
//! # Shapes
//!
//! Every copper or keepout object is a [`Shape`]: a [`Segment`] with a width,
//! a [`Circle`], a [`ConvexPoly`] (optionally rounded), or a [`Polygon`] with
//! holes. Clearance is always measured edge to edge ([`gap_cmp`]), so a
//! track's half width and a pad's rounding radius are part of the shape, not
//! of the rule.
//!
//! # Indexing
//!
//! [`LayerIndex`] is an `rstar` R-tree over inflated bounding boxes, one per
//! copper layer ([`LayeredIndex`]). Query results are sorted ids, so callers
//! iterate in the same order on every run and every thread count.

pub mod arc;
pub mod bbox;
pub mod index;
pub mod point;
pub mod predicates;
pub mod shape;
pub mod units;

pub use arc::{approximate_arc, approximate_circle, ArcPolicy, ArcSide};
pub use bbox::BBox;
pub use index::{IndexEntry, LayerIndex, LayeredIndex};
pub use point::{Point, Vector};
pub use predicates::{
    gap_cmp, has_clearance, point_in_polygon, point_segment_distance_cmp,
    segment_polygon_distance_cmp, segment_segment_distance_cmp, segments_intersect, touches,
    Containment,
};
pub use shape::{Circle, ConvexPoly, Polygon, Ring, Segment, Shape};
pub use units::{mm_to_nm, nm_to_mm, Coord, MAX_COORD, NM_PER_MM};
