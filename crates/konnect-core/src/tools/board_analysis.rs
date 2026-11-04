//! The `board_analysis` toolset: read-only electrical and manufacturing
//! analyses of the saved board for a layout adequacy review.
//!
//! | Tool | Analysis |
//! |---|---|
//! | `check_zone_hygiene` | overlaps, wrapped foreign copper, slivers/necks, fill pieces |
//! | `check_current_capacity` | IPC-2152 capacity of tracks, vias and pour necks on a solved current path |
//! | `analyze_ir_drop` | DC voltage drop through tracks, vias and meshed fills |
//! | `check_stitching_vias` | via-free circles, edge stitch pitch, island ties, focus regions |
//! | `check_decoupling_placement` | copper path from IC power pins to caps, GND via distances |
//! | `check_thermal_pads` | thermal-pad vias, same-net copper area per layer, paste windows |
//! | `check_voltage_spacing` | IPC-2221B clearance for net voltages, violations, DRC rules |
//! | `measure_net_lengths` | routed length, pair skew and group spread |
//!
//! Each reads the saved `.kicad_pcb` once (`"source": "saved_file"`), takes
//! copper thickness and layer depth from its stackup, answers with a summary
//! inline, writes the full result to `output` and render markers to
//! `markers_output` when asked, and cites its method. Nothing is written to
//! the board.

#[cfg(test)]
mod acceptance;
mod common;
mod current;
mod decoupling;
mod geom;
mod hygiene;
mod ipc;
mod irdrop;
mod lengths;
mod necks;
mod network;
mod spacing;
mod stitching;
#[cfg(test)]
mod tests_support;
mod thermal;

use crate::mcp::protocol::CallToolResult;
use crate::tool;
use crate::tools::{ToolContext, ToolDef};
use common::Outputs;
use serde_json::Value;

/// Run `f` on the loaded board off the async runtime, with the output files
/// checked first.
async fn blocking(
    args: &Value,
    f: fn(&common::Loaded, &Value, &Outputs) -> Result<CallToolResult, CallToolResult>,
) -> anyhow::Result<CallToolResult> {
    let args = args.clone();
    tokio::task::spawn_blocking(move || -> anyhow::Result<CallToolResult> {
        let out = match Outputs::of(&args) {
            Ok(o) => o,
            Err(r) => return Ok(r),
        };
        let loaded = match common::load(&args)? {
            Ok(l) => l,
            Err(r) => return Ok(r),
        };
        Ok(match f(&loaded, &args, &out) {
            Ok(r) | Err(r) => r,
        })
    })
    .await?
}

// The explicit call (not the bare function) lets the schema-usage test
// follow the handler into its analysis.
#[allow(clippy::redundant_closure)]
pub(crate) async fn handle_check_zone_hygiene(
    args: &Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    blocking(args, |l, a, o| hygiene::check_zone_hygiene(l, a, o)).await
}

// The explicit call (not the bare function) lets the schema-usage test
// follow the handler into its analysis.
#[allow(clippy::redundant_closure)]
pub(crate) async fn handle_check_current_capacity(
    args: &Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    blocking(args, |l, a, o| current::check_current_capacity(l, a, o)).await
}

// The explicit call (not the bare function) lets the schema-usage test
// follow the handler into its analysis.
#[allow(clippy::redundant_closure)]
pub(crate) async fn handle_analyze_ir_drop(
    args: &Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    blocking(args, |l, a, o| irdrop::analyze_ir_drop(l, a, o)).await
}

// The explicit call (not the bare function) lets the schema-usage test
// follow the handler into its analysis.
#[allow(clippy::redundant_closure)]
pub(crate) async fn handle_check_stitching_vias(
    args: &Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    blocking(args, |l, a, o| stitching::check_stitching_vias(l, a, o)).await
}

// The explicit call (not the bare function) lets the schema-usage test
// follow the handler into its analysis.
#[allow(clippy::redundant_closure)]
pub(crate) async fn handle_check_decoupling_placement(
    args: &Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    blocking(args, |l, a, o| {
        decoupling::check_decoupling_placement(l, a, o)
    })
    .await
}

// The explicit call (not the bare function) lets the schema-usage test
// follow the handler into its analysis.
#[allow(clippy::redundant_closure)]
pub(crate) async fn handle_check_thermal_pads(
    args: &Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    blocking(args, |l, a, o| thermal::check_thermal_pads(l, a, o)).await
}

// The explicit call (not the bare function) lets the schema-usage test
// follow the handler into its analysis.
#[allow(clippy::redundant_closure)]
pub(crate) async fn handle_check_voltage_spacing(
    args: &Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    blocking(args, |l, a, o| spacing::check_voltage_spacing(l, a, o)).await
}

// The explicit call (not the bare function) lets the schema-usage test
// follow the handler into its analysis.
#[allow(clippy::redundant_closure)]
pub(crate) async fn handle_measure_net_lengths(
    args: &Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    blocking(args, |l, a, o| lengths::measure_net_lengths(l, a, o)).await
}

pub(crate) fn tools() -> Vec<ToolDef> {
    vec![tool!(
        "check_zone_hygiene",
        "Audit the saved board's zones: same-net and cross-net outline overlaps (true polygon \
         intersection, with priorities; equal priorities flagged), zones whose outline encloses \
         foreign-net pads or vias (the fill wraps them), fill slivers and necks narrower than \
         sliver_width_mm (the narrowest chord across the fill copper, located), and every fill \
         piece with the same-net pads, vias and tracks it touches (pieces that connect nothing \
         are dead). Filters: nets, zones, layers. Summary inline; output has everything; \
         markers_output rings each finding for render_board_layers_png. Reads the saved file \
         only.",
        hygiene::schema(),
        |args, ctx| async move { handle_check_zone_hygiene(args, ctx).await }
    ),
    tool!(
        "check_current_capacity",
        "Check that each net's copper carries its current, by IPC-2152. For every net case          (current_a, optional peak_a and pulse_ms; source pads; sink pads, \"all\", and/or          sink_layers whose fill takes the current), the net's tracks, via barrels and zone fills          are solved as a DC network, and each conductor on the path is held against its          IPC-2152 capacity for the layer's stackup copper: tracks, single vias, sets of parallel          vias (count × per-via capacity) and pour necks (the exact narrowest chords of the fill,          with the solved current through each). Reports the worst bottleneck with location,          kind, capacity, required width and margin, the narrowest neck per layer, every via set,          and every element below requirement. rise_c, ambient_c, plating_um and IPC-2152          modifier factors are parameters; area limits the model to a box. Reads the saved file          only.",
        current::schema(),
        |args, ctx| async move { handle_check_current_capacity(args, ctx).await }
    ),
    tool!(
        "analyze_ir_drop",
        "DC IR drop of one rail: source pads held at source_voltage, loads drawing their          currents (sinks, or sinks_file as JSON or a pad,current_a CSV). Tracks are ρL/(w·t),          vias plated barrels over the stack depth, fills a square mesh (resolution_mm) with          sheet resistance from the stackup copper; solved with preconditioned conjugate          gradients. Reports the voltage and drop at each load, the worst drop, the resistance          from the source to each load, current-density hotspots (A/mm²) as markers, and the          rail's fill necks with the current through each. Reads the saved file only.",
        irdrop::schema(),
        |args, ctx| async move { handle_analyze_ir_drop(args, ctx).await }
    ),
    tool!(
        "check_stitching_vias",
        "Stitching of a net's pours (default GND): for each fill piece on the given layers, the          largest via-free circle (centre and radius), the stitch pitch and longest unstitched          run along its edge, and the board edge per layer; for each island pour, the vias that          tie it to the planes; for focus_footprints (connectors, TVS), every stitching via          within focus_radius_mm of their pads, counted; for focus_nets (fast buses), each          signal via's distance to its nearest stitch; and a via-density heat map with empty          pour cells as markers. Stitches are the net's vias and plated through-hole pads.          Reads the saved file only.",
        stitching::schema(),
        |args, ctx| async move { handle_check_stitching_vias(args, ctx).await }
    ),
    tool!(
        "check_decoupling_placement",
        "For each IC power pin (references, or every U*), the nearest same-net capacitor by          copper path length (tracks, via barrels and pours, not straight line), with the          straight distance and the next candidates; the distance from that capacitor's ground          pad to its nearest ground via; the distance from each IC ground pin to its nearest          ground via; and ground vias several of them share. associations pins a pin to its          intended capacitors. Reads the saved file only.",
        decoupling::schema(),
        |args, ctx| async move { handle_check_decoupling_placement(args, ctx).await }
    ),
    tool!(
        "check_thermal_pads",
        "Audit exposed and thermal pads (detected, or by references or REF.PAD): via count,          drills and plating inside the pad with the barrels' copper and thermal resistance,          same-net copper area per layer within radius_mm reached from the pad or its vias, and          whether the paste is windowed (paste-only apertures) with its coverage. Compared with          an optional per-part datasheet recommendation (vias_min, drill_mm, area_mm2_min per          layer, paste_coverage_max). Reads the saved file only.",
        thermal::schema(),
        |args, ctx| async move { handle_check_thermal_pads(args, ctx).await }
    ),
    tool!(
        "check_voltage_spacing",
        "Voltage-based clearance by IPC-2221B Table 6-1: from a net to voltage map (working or          transient), the required spacing for every pair with a mapped net (outer layers B2, B3,          B4 coated, or A5–A7; inner B1), and every place on each layer where pads, tracks, vias          or fills of the two nets come closer, located. rules_output writes matching KiCad          clearance rules (.kicad_dru) to merge so DRC enforces them afterwards. Reads the saved          file only.",
        spacing::schema(),
        |args, ctx| async move { handle_check_voltage_spacing(args, ctx).await }
    ),
    tool!(
        "measure_net_lengths",
        "Routed length of each net in groups (pairs and buses): tracks plus arcs plus the via          barrels they use over the stackup, per layer, optionally pad to pad through the net's          copper; the skew inside each pair and the spread inside each group, against          target_length_mm, tolerance_mm and max_skew_mm. Reads the saved file only.",
        lengths::schema(),
        |args, ctx| async move { handle_measure_net_lengths(args, ctx).await }
    )]
}
