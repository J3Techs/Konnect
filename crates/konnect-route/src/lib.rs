//! Konnect's native autorouter: a CPU-first global plus tiled detailed
//! router that plans copper offline and hands it to Konnect's existing
//! write path.
//!
//! No IPC and no async runtime live here: the crate reads a board model,
//! computes, and returns a [`plan::RoutePlan`] in `add_routing` form. Konnect
//! tools own the KiCad side.
//!
//! # Stages
//!
//! | Stage | Module | Owner |
//! |---|---|---|
//! | S0 ingest | [`loader`], [`board`] | WS-B |
//! | S1 connectivity, terminal audit | [`connectivity`], [`audit`] | WS-B |
//! | S2 plane fanout | [`fanout`] | WS-B |
//! | S3 global | [`global`], [`compute::batch`] | WS-C |
//! | S4 pin access | [`access`] | WS-B |
//! | S5 detailed search | [`detail::search`] | WS-D1 |
//! | S5 negotiation | [`detail::negotiate`] | WS-D2 |
//! | S6 optimize | [`optimize`] | WS-E |
//! | S7 internal DRC | [`check`] | WS-A |
//! | S8 plan | [`plan`] | WS-E |
//!
//! Shared vocabulary is frozen in [`api`], rules in [`rules`], GPU-shaped
//! seams in [`compute`], run metrics in [`telemetry`], and test boards in
//! [`synthetic`]. See `docs/native-router/INTERFACES.md` for the ownership
//! and change-control rules.

pub mod access;
pub mod api;
pub mod audit;
pub mod board;
pub mod check;
pub mod compute;
pub mod connectivity;
pub mod detail;
pub mod fanout;
pub mod global;
pub mod loader;
pub mod optimize;
pub mod pipeline;
pub mod plan;
pub mod rules;
mod space;
pub mod synthetic;
pub mod telemetry;

pub use board::RouteBoard;
pub use loader::{load_board_file, load_board_str, LoadOptions, LoadReport};
pub use plan::{RoutePlan, RoutedCopper};
pub use rules::{ProjectRuleTable, RuleTable};
