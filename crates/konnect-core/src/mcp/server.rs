//! MCP server session context.

use super::protocol::*;
use crate::router::ToolRouter;
use std::sync::Arc;

/// What every client is told at `initialize`, whether or not the bundled
/// skills were installed where that client reads them. The skills carry the
/// full workflow; this is the part that must not depend on them, because a
/// sheet built against it still nets up and passes ERC.
pub const STANDING_INSTRUCTIONS: &str = "\
Konnect schematics are read by people. Draw each functional block as a circuit: place its parts \
in signal-flow order and join them with wires (connect_pins, add_wire, batch_add_wire, \
add_schematic_connection). Use a net label only where a net leaves the sheet, runs to a distant \
block, or fans out from a large IC or connector, and a power symbol for a rail. Do not build a \
sheet as parts with a label on every pin and no wires between them: that is a netlist laid out \
on a page, and batch_connect_to_net is not a way to wire a block. Redraw a block that was built \
that way. Look at the rendered sheet (render_schematic_png) before calling a schematic done.";

pub struct McpServerState {
    pub router: Arc<ToolRouter>,
}

impl McpServerState {
    pub fn new(router: Arc<ToolRouter>) -> Self {
        McpServerState { router }
    }

    /// `notice` is this start's one-off report (installed guidance out of
    /// sync); it follows the standing instructions on lines of its own.
    pub fn build_initialize_result(notice: Option<String>) -> InitializeResult {
        let instructions = match notice {
            Some(notice) => Some(format!("{STANDING_INSTRUCTIONS}\n\n{notice}")),
            None => Some(STANDING_INSTRUCTIONS.to_owned()),
        };
        InitializeResult {
            protocol_version: "2025-06-18".to_string(),
            capabilities: ServerCapabilities {
                tools: Some(ToolsCapability {
                    list_changed: Some(true),
                }),
                ..Default::default()
            },
            server_info: ServerInfo {
                name: "konnect".to_string(),
                version: env!("CARGO_PKG_VERSION").to_string(),
            },
            instructions,
        }
    }
}
