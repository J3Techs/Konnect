//! S5 detailed routing: lattice search ([`search`], WS-D1) under negotiated
//! congestion ([`negotiate`], WS-D2).
//!
//! The lattice is octilinear with a base pitch of
//! [`crate::api::LATTICE_PITCH_NM`], refined to
//! [`crate::api::LATTICE_FINE_PITCH_NM`] in BGA fields, with lines forced
//! through pad centres, access points and inflated obstacle edges. Search
//! state is `(x, y, layer, heading, run_since_turn)`; 135° turns are
//! forbidden (they leave an acute interior angle, which a `track_angle` rule
//! may refuse), and branches join existing copper only at an endpoint or
//! at 90°.

pub mod negotiate;
pub mod search;
