use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// KiCad design editor addressed by the typed IPC API.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IpcEditorKind {
    Schematic,
    Pcb,
}

impl IpcEditorKind {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Schematic => "schematic",
            Self::Pcb => "pcb",
        }
    }
}

/// Runtime availability of one semantic editor capability.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IpcCapabilityAvailability {
    Available,
    Unsupported,
    Unknown,
}

/// One capability statement and the evidence used to make it.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct IpcCapability {
    pub availability: IpcCapabilityAvailability,
    pub evidence_source: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
}

/// Capabilities relevant to the Priority 1 semantic navigation surface.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct IpcEditorCapabilities {
    pub observe_documents: IpcCapability,
    pub observe_active_context: IpcCapability,
    pub read_selection: IpcCapability,
    pub mutate_selection: IpcCapability,
    pub activate_document: IpcCapability,
    pub activate_sheet: IpcCapability,
    pub reveal_object: IpcCapability,
    pub center_object: IpcCapability,
    pub fit_view: IpcCapability,
    pub cross_probe: IpcCapability,
}

/// Running KiCad version observed through `GetVersion`.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct IpcKiCadVersion {
    pub major: u32,
    pub minor: u32,
    pub patch: u32,
    pub full_version: String,
}

/// Project identity carried by a live KiCad `DocumentSpecifier`.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct IpcProjectIdentity {
    pub name: String,
    pub path: String,
}

/// Canonical schematic instance identity carried by KiCad IPC.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct IpcSheetInstancePath {
    pub kiids: Vec<String>,
    pub human_readable: String,
}

/// One exact live document identity observed through KiCad IPC.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct IpcEditorDocument {
    pub editor: IpcEditorKind,
    pub project: Option<IpcProjectIdentity>,
    /// Exact board path when KiCad provides one. KiCad 10 schematic document
    /// specifiers carry a sheet path rather than a schematic filename, so this
    /// is deliberately null for schematics instead of being inferred from disk.
    pub document_path: Option<String>,
    pub sheet_instance_path: Option<IpcSheetInstancePath>,
}

/// Observation for one editor kind on the configured IPC endpoint.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct IpcEditorObservation {
    pub editor: IpcEditorKind,
    pub addressable: bool,
    pub documents: Vec<IpcEditorDocument>,
    pub capabilities: IpcEditorCapabilities,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unavailable_reason: Option<String>,
}

/// Result of observing the configured KiCad IPC endpoint.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct IpcEditorStateObservation {
    pub kicad_version: IpcKiCadVersion,
    pub evidence_source: String,
    pub editors: Vec<IpcEditorObservation>,
    /// KiCad 10 has no stable typed foreground-frame or active-document query.
    /// These fields remain null rather than treating open-document order as
    /// active state.
    pub active_editor: Option<IpcEditorKind>,
    pub active_document: Option<IpcEditorDocument>,
    pub active_sheet_instance: Option<IpcSheetInstancePath>,
    pub limitations: Vec<String>,
}

/// One selected object observed from the exact requested live document.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct IpcSelectedObject {
    /// Stable KiCad object identifier (KIID/UUID), never a display reference.
    pub kiid: String,
    /// Semantic object kind derived from the exact protobuf type URL.
    pub object_type: String,
    /// Full protobuf type carried by KiCad, retained as decoding evidence.
    pub protocol_type: String,
}

/// Selection readback bound to one exact live editor/document/sheet context.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct IpcSelectionObservation {
    pub project: Option<IpcProjectIdentity>,
    pub document: IpcEditorDocument,
    pub editor: IpcEditorKind,
    pub sheet_instance_path: Option<IpcSheetInstancePath>,
    pub selected_objects: Vec<IpcSelectedObject>,
    pub evidence_source: String,
}

/// Semantic selection change requested from one exact editor context.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IpcSelectionMutation {
    Clear,
    Add,
    Remove,
}

impl IpcSelectionMutation {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Clear => "clear",
            Self::Add => "add",
            Self::Remove => "remove",
        }
    }
}

/// Verified selection mutation. Success means the post-operation observation
/// exactly matched the requested set transition, not merely that IPC replied.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct IpcSelectionMutationResult {
    pub operation: IpcSelectionMutation,
    pub requested_kiids: Vec<String>,
    pub before: IpcSelectionObservation,
    pub after: IpcSelectionObservation,
    pub evidence_source: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IpcSelectionMutationErrorKind {
    InvalidRequest,
    ReadbackMismatch,
}

/// A semantic selection mutation was invalid or its observed result did not
/// prove the requested change.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IpcSelectionMutationError {
    pub kind: IpcSelectionMutationErrorKind,
    pub operation: IpcSelectionMutation,
    pub requested_kiids: Vec<String>,
    pub before_kiids: Vec<String>,
    pub after_kiids: Vec<String>,
    pub reason: String,
}

impl IpcSelectionMutationError {
    pub fn from_error(error: &anyhow::Error) -> Option<&Self> {
        error.chain().find_map(|cause| cause.downcast_ref::<Self>())
    }
}

impl std::fmt::Display for IpcSelectionMutationError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            formatter,
            "cannot verify {} selection mutation: {}",
            self.operation.as_str(),
            self.reason
        )
    }
}

impl std::error::Error for IpcSelectionMutationError {}

/// Stable classification for a fail-closed selection observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IpcSelectionObservationErrorKind {
    WrongProject,
    WrongDocument,
    WrongSheetInstance,
    AmbiguousDocument,
    StaleEditorState,
    UnsupportedObjectType,
    MalformedSelectedObject,
}

/// A selection could not be attributed to the exact requested live context.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IpcSelectionObservationError {
    pub kind: IpcSelectionObservationErrorKind,
    pub editor: IpcEditorKind,
    pub requested: String,
    pub candidates: Vec<String>,
    pub reason: String,
}

impl IpcSelectionObservationError {
    pub fn from_error(error: &anyhow::Error) -> Option<&Self> {
        error.chain().find_map(|cause| cause.downcast_ref::<Self>())
    }
}

impl std::fmt::Display for IpcSelectionObservationError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            formatter,
            "cannot observe {} selection for {}: {}",
            self.editor.as_str(),
            self.requested,
            self.reason
        )
    }
}

impl std::error::Error for IpcSelectionObservationError {}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct IpcVector2 {
    pub x: f64,
    pub y: f64,
}

/// One copper-layer shape from a KiCad pad stack.
///
/// `layer` is the native KiCad layer name.  A normal stack may expose one
/// representative copper entry while `IpcPad::layers` still names every layer
/// on which the pad exists. Front/inner/back stacks retain every distinct
/// entry here; custom shapes are identified as `custom` rather than reduced to
/// a guessed ordinary outline.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct IpcPadLayerGeometry {
    pub layer: String,
    pub shape: Option<String>,
    pub size: Option<IpcVector2>,
    pub offset: Option<IpcVector2>,
    pub corner_rounding_ratio: Option<f64>,
    pub chamfer_ratio: Option<f64>,
    /// For a `custom` entry, the anchor shape KiCad reports; absent otherwise.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub custom_anchor_shape: Option<String>,
    /// For a `custom` entry, its primitives in pad-local mm, exactly as KiCad
    /// reports them; absent for ordinary shapes.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub custom_primitives: Option<Vec<IpcPadPrimitive>>,
    /// Why `custom_primitives` is absent from a `custom` entry: KiCad sent a
    /// primitive this build cannot describe. Never left silently empty.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub custom_primitives_unreadable: Option<String>,
}

/// Native drill information for a pad, when it has a drilled hole.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct IpcPadDrill {
    pub shape: Option<String>,
    pub size: Option<IpcVector2>,
    pub start_layer: Option<String>,
    pub end_layer: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IpcFootprint {
    pub reference: String,
    pub value: String,
    pub footprint: String,
    pub position: IpcVector2,
    pub rotation: f64,
    pub layer: String,
    /// The footprint instance KIID KiCad reported, `None` when it sent none.
    #[serde(default)]
    pub uuid: Option<String>,
}

/// A pad of a footprint placed on the board, read back from KiCad.
///
/// Coordinates are absolute board millimetres: KiCad serializes a
/// `FootprintInstance`'s children in board space (see the `transform` module),
/// so no anchor or rotation transform is applied on the way out.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IpcPad {
    /// Stable KiCad item identifier, or `None` when the source did not carry
    /// one.  Never synthesize an identifier for a pad that KiCad did not name.
    pub uuid: Option<String>,
    pub number: String,
    pub x: f64,
    pub y: f64,
    /// Net name, empty when the pad carries no net.
    pub net: String,
    /// KiCad layer names from the live pad stack.
    pub layers: Vec<String>,
    /// Native KiCad pad type (`smd`, `thru_hole`, `np_thru_hole`, or
    /// `edge_connector`), or `None` for an unknown future type.
    pub pad_type: Option<String>,
    /// Effective pad-stack rotation in board space, in degrees.
    pub rotation_deg: Option<f64>,
    /// Compact geometry for ordinary pads.  These mirror the first copper
    /// entry; callers needing custom/front-inner-back detail use
    /// `copper_layers` instead.
    pub shape: Option<String>,
    pub size: Option<IpcVector2>,
    pub drill: Option<IpcPadDrill>,
    /// Every copper geometry entry KiCad supplied.  This prevents custom and
    /// front/inner/back pad stacks from being flattened into a guessed shape.
    pub copper_layers: Vec<IpcPadLayerGeometry>,
    /// The pad's corner-rounding ratio, read from the pad itself rather than
    /// a copper entry, so a pad with no copper (a paste-only aperture) still
    /// reports it. Mirrors the first copper entry on the live board.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub corner_rounding_ratio: Option<f64>,
    /// How copper zones connect to the pad. A footprint file without a
    /// `(zone_connect …)` reads as `inherited`; a live pad whose stack reports
    /// no zone settings leaves this absent.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub zone_connection: Option<IpcPadZoneConnection>,
    /// Thermal-relief settings as KiCad holds them: what the live pad stack
    /// reports, or for a footprint file the explicit clauses with the angle
    /// KiCad's parser gives the pad. Absent when the live stack reports none.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub thermal: Option<IpcPadThermal>,
    /// Whether a footprint file wrote `(thermal_bridge_angle …)` itself
    /// (`false`: `thermal.spoke_angle` is the parser's shape default). Always
    /// `null` for a live pad, whose stack cannot say.
    #[serde(default)]
    pub thermal_spoke_angle_explicit: Option<bool>,
}

/// A pad's own zone-connection override: KiCad's `ZONE_CONNECTION`, written
/// in a footprint file as `(zone_connect N)` and carried by the API as
/// `PadStack.zone_settings.zone_connection`.
///
/// `PadStackLayer.zone_settings` also exists, but the protocol reserves it
/// for future per-layer use; the pad-stack field is the one KiCad applies.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IpcPadZoneConnection {
    /// `-1`: follow the footprint, zone and board rules. KiCad writes it by
    /// leaving the token out.
    Inherited,
    /// `0`
    None,
    /// `1`: thermal reliefs.
    Thermal,
    /// `2`: solid copper.
    Solid,
    /// `3`: thermal reliefs for plated through holes only, solid otherwise.
    PthThermal,
}

impl IpcPadZoneConnection {
    /// The connection a footprint file's `(zone_connect N)` names, or `None`
    /// for a number KiCad does not define.
    pub fn from_file_value(value: i64) -> Option<Self> {
        Some(match value {
            -1 => Self::Inherited,
            0 => Self::None,
            1 => Self::Thermal,
            2 => Self::Solid,
            3 => Self::PthThermal,
            _ => return None,
        })
    }

    /// The number a footprint file writes for this connection.
    pub fn file_value(self) -> i64 {
        match self {
            Self::Inherited => -1,
            Self::None => 0,
            Self::Thermal => 1,
            Self::Solid => 2,
            Self::PthThermal => 3,
        }
    }

    pub fn to_proto(self) -> crate::gen::kiapi::board::types::ZoneConnectionStyle {
        use crate::gen::kiapi::board::types::ZoneConnectionStyle as Style;
        match self {
            Self::Inherited => Style::ZcsInherited,
            Self::None => Style::ZcsNone,
            Self::Thermal => Style::ZcsThermal,
            Self::Solid => Style::ZcsFull,
            Self::PthThermal => Style::ZcsPthThermal,
        }
    }

    /// The connection a protocol value names, or `None` for `ZCS_UNKNOWN`
    /// and values this build does not know.
    pub fn from_proto(value: i32) -> Option<Self> {
        use crate::gen::kiapi::board::types::ZoneConnectionStyle as Style;
        Some(match Style::try_from(value).ok()? {
            Style::ZcsInherited => Self::Inherited,
            Style::ZcsNone => Self::None,
            Style::ZcsThermal => Self::Thermal,
            Style::ZcsFull => Self::Solid,
            Style::ZcsPthThermal => Self::PthThermal,
            Style::ZcsUnknown => return None,
        })
    }
}

/// A pad's thermal-relief settings: `(thermal_bridge_width …)`,
/// `(thermal_gap …)` and `(thermal_bridge_angle …)` in a footprint file, and
/// `PadStack.zone_settings.thermal_spokes` in the API.
///
/// `None` means "not set": the width and gap then inherit from the zone, and
/// an absent angle means the shape default. An explicit zero is `Some(0.0)`,
/// never folded into `None`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct IpcPadThermal {
    /// Spoke width override, in mm.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub spoke_width: Option<f64>,
    /// Relief gap override, in mm.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gap: Option<f64>,
    /// Spoke angle, in degrees.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub spoke_angle: Option<f64>,
}

impl IpcPadThermal {
    pub fn is_empty(&self) -> bool {
        self.spoke_width.is_none() && self.gap.is_none() && self.spoke_angle.is_none()
    }

    /// The `ThermalSpokeSettings` to send, or `None` when nothing is set.
    ///
    /// KiCad 10's `PADSTACK::Deserialize` reads the angle unconditionally once
    /// `thermal_spokes` is present (an absent angle arrives as 0°), so a width
    /// or gap without an angle is refused rather than sent.
    pub fn to_proto(
        &self,
    ) -> anyhow::Result<Option<crate::gen::kiapi::board::types::ThermalSpokeSettings>> {
        if self.is_empty() {
            return Ok(None);
        }
        self.validate()?;
        let angle = self.spoke_angle.ok_or_else(|| {
            anyhow::anyhow!(
                "thermal spoke settings need an angle: KiCad reads a missing one as 0 degrees"
            )
        })?;
        Ok(Some(
            crate::gen::kiapi::board::types::ThermalSpokeSettings {
                width: self.spoke_width.map(crate::builders::distance),
                angle: Some(crate::gen::kiapi::common::types::Angle {
                    value_degrees: angle,
                }),
                gap: self.gap.map(crate::builders::distance),
            },
        ))
    }

    /// Exactly the settings a protocol message carries; absent fields stay
    /// absent.
    pub fn from_proto(settings: &crate::gen::kiapi::board::types::ThermalSpokeSettings) -> Self {
        Self {
            spoke_width: settings
                .width
                .map(|width| crate::builders::nm_to_mm(width.value_nm)),
            gap: settings
                .gap
                .map(|gap| crate::builders::nm_to_mm(gap.value_nm)),
            spoke_angle: settings.angle.as_ref().map(|angle| angle.value_degrees),
        }
    }

    /// Refuse what KiCad could not hold as given: non-finite numbers, and a
    /// negative or out-of-range (KiCad clamps board units to 32 bits) width
    /// or gap.
    pub fn validate(&self) -> anyhow::Result<()> {
        for (label, value) in [
            ("thermal spoke width", self.spoke_width),
            ("thermal gap", self.gap),
        ] {
            if let Some(value) = value {
                anyhow::ensure!(value.is_finite(), "{label} {value} is not finite");
                anyhow::ensure!(value >= 0.0, "{label} {value} mm is negative");
                anyhow::ensure!(
                    crate::builders::mm_to_nm(value) <= i64::from(i32::MAX),
                    "{label} {value} mm exceeds KiCad's board-unit range"
                );
            }
        }
        if let Some(angle) = self.spoke_angle {
            anyhow::ensure!(
                angle.is_finite(),
                "thermal spoke angle {angle} is not finite"
            );
        }
        Ok(())
    }
}

/// The spoke angle KiCad 10's footprint-file parser gives a pad without a
/// `(thermal_bridge_angle …)`: 45° for a circle, and for a custom pad on a
/// circle anchor (90° in files of format 20211014 or older), 90° otherwise.
/// `format_version` is the file's `(version …)`; KiCad reads a missing one as
/// 0, the oldest format.
pub fn file_default_thermal_spoke_angle(
    shape: &str,
    custom_anchor: Option<IpcCustomPadAnchor>,
    format_version: Option<i64>,
) -> f64 {
    match (shape, custom_anchor) {
        ("circle", _) => 45.0,
        ("custom", Some(IpcCustomPadAnchor::Circle)) if format_version.unwrap_or(0) > 20211014 => {
            45.0
        }
        _ => 90.0,
    }
}

/// The document's title block, which the board file also carries.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct IpcTitleBlock {
    pub title: String,
    pub date: String,
    pub revision: String,
    pub company: String,
}

/// Complete target placement for one existing footprint.
///
/// Keeping the four values together lets the IPC client transform all selected
/// footprints from one board snapshot and publish them in one undoable update,
/// instead of issuing a move and a rotation as separate round trips.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct IpcFootprintPlacement {
    pub reference: String,
    pub x: f64,
    pub y: f64,
    pub rotation: f64,
}

#[derive(Debug, Clone)]
pub struct IpcPadDefinition {
    pub number: String,
    pub pad_type: String,
    pub shape: String,
    pub x: f64,
    pub y: f64,
    pub rotation: f64,
    pub size_x: f64,
    pub size_y: f64,
    pub drill_x: Option<f64>,
    pub drill_y: Option<f64>,
    pub drill_oval: bool,
    pub layers: Vec<String>,
    pub roundrect_ratio: f64,
    /// The anchor and primitives of a `custom` pad; `None` for every other
    /// shape. `build_footprint_item` refuses a `custom` pad without it, and
    /// any other shape with it, so the two can never disagree silently.
    pub custom: Option<IpcCustomPad>,
    /// The pad's `(zone_connect …)`, or `None` when the library leaves it out
    /// and the pad inherits. An explicit `-1` is kept as `Inherited`.
    pub zone_connection: Option<IpcPadZoneConnection>,
    /// The pad's thermal-relief settings. A footprint-file reader fills the
    /// angle in even when the file leaves it out (see
    /// [`file_default_thermal_spoke_angle`]), because KiCad's parser always
    /// sets one; empty sends no thermal settings at all.
    pub thermal: IpcPadThermal,
}

/// The anchor of a custom pad: the ordinary pad shape, sized by the pad's
/// `(size …)`, that the primitives are unioned with. KiCad supports only these
/// two (`PadStackLayer::custom_anchor_shape`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IpcCustomPadAnchor {
    Circle,
    Rect,
}

impl IpcCustomPadAnchor {
    /// The token a footprint file uses in `(options (anchor …))`.
    pub fn token(self) -> &'static str {
        match self {
            Self::Circle => "circle",
            Self::Rect => "rect",
        }
    }
}

/// The custom-shape part of a `custom` pad.
///
/// Primitive coordinates are PAD-LOCAL: relative to the pad position, before
/// the pad's rotation. KiCad keeps them that way (`PADSTACK` primitives) and
/// rotates them with the pad, so they must never be passed through the
/// footprint placement transform.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct IpcCustomPad {
    pub anchor: IpcCustomPadAnchor,
    pub primitives: Vec<IpcPadPrimitive>,
}

impl IpcCustomPad {
    /// Distance from the pad origin to the farthest copper any primitive
    /// draws, stroke included, in mm. A rotation-invariant extent.
    pub fn primitive_reach(&self) -> f64 {
        self.primitives
            .iter()
            .map(IpcPadPrimitive::reach)
            .fold(0.0, f64::max)
    }

    /// Refuse anything KiCad would not draw exactly as described: non-finite
    /// numbers, an empty primitive list and degenerate primitives.
    pub fn validate(&self) -> anyhow::Result<()> {
        anyhow::ensure!(
            !self.primitives.is_empty(),
            "a custom pad needs at least one primitive"
        );
        for (index, primitive) in self.primitives.iter().enumerate() {
            primitive.validate().map_err(|error| {
                anyhow::anyhow!("primitive {index} ({}): {error}", primitive.kind())
            })?;
        }
        Ok(())
    }
}

/// One custom-pad primitive in pad-local mm (see [`IpcCustomPad`]).
///
/// `width` is the stroke width, which KiCad adds to the copper outline of a
/// filled shape as well as an outlined one.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum IpcPadPrimitive {
    /// `gr_poly` — one closed outline of straight edges.
    Polygon {
        points: Vec<(f64, f64)>,
        width: f64,
        filled: bool,
    },
    /// `gr_rect` — axis-aligned (in the pad frame) between two corners.
    Rect {
        start: (f64, f64),
        end: (f64, f64),
        width: f64,
        filled: bool,
    },
    /// `gr_circle` — centre plus a point on the circumference.
    Circle {
        center: (f64, f64),
        end: (f64, f64),
        width: f64,
        filled: bool,
    },
    /// `gr_line` — a stroked segment.
    Segment {
        start: (f64, f64),
        end: (f64, f64),
        width: f64,
    },
    /// `gr_arc` — a stroked arc through start, mid and end.
    Arc {
        start: (f64, f64),
        mid: (f64, f64),
        end: (f64, f64),
        width: f64,
    },
}

fn finite_point(label: &str, (x, y): (f64, f64)) -> anyhow::Result<()> {
    anyhow::ensure!(
        x.is_finite() && y.is_finite(),
        "{label} ({x}, {y}) is not a finite coordinate"
    );
    Ok(())
}

impl IpcPadPrimitive {
    /// The footprint-file token this primitive is written with.
    pub fn kind(&self) -> &'static str {
        match self {
            Self::Polygon { .. } => "gr_poly",
            Self::Rect { .. } => "gr_rect",
            Self::Circle { .. } => "gr_circle",
            Self::Segment { .. } => "gr_line",
            Self::Arc { .. } => "gr_arc",
        }
    }

    pub fn width(&self) -> f64 {
        match self {
            Self::Polygon { width, .. }
            | Self::Rect { width, .. }
            | Self::Circle { width, .. }
            | Self::Segment { width, .. }
            | Self::Arc { width, .. } => *width,
        }
    }

    pub fn filled(&self) -> bool {
        match self {
            Self::Polygon { filled, .. }
            | Self::Rect { filled, .. }
            | Self::Circle { filled, .. } => *filled,
            Self::Segment { .. } | Self::Arc { .. } => false,
        }
    }

    /// See [`IpcCustomPad::primitive_reach`].
    pub fn reach(&self) -> f64 {
        let distance = |(x, y): (f64, f64)| x.hypot(y);
        let farthest = match self {
            Self::Polygon { points, .. } => {
                points.iter().copied().map(distance).fold(0.0, f64::max)
            }
            Self::Rect { start, end, .. } => [*start, *end, (start.0, end.1), (end.0, start.1)]
                .into_iter()
                .map(distance)
                .fold(0.0, f64::max),
            Self::Circle { center, end, .. } => {
                distance(*center) + (end.0 - center.0).hypot(end.1 - center.1)
            }
            Self::Segment { start, end, .. } => distance(*start).max(distance(*end)),
            // The chord points bound the arc only loosely; the circle through
            // them bounds it for certain.
            Self::Arc {
                start, mid, end, ..
            } => match circle_through(*start, *mid, *end) {
                Some((center, radius)) => distance(center) + radius,
                None => distance(*start).max(distance(*mid)).max(distance(*end)),
            },
        };
        farthest + self.width() / 2.0
    }

    pub fn validate(&self) -> anyhow::Result<()> {
        let width = self.width();
        anyhow::ensure!(
            width.is_finite() && width >= 0.0,
            "stroke width {width} is not a finite, non-negative length"
        );
        match self {
            Self::Polygon { points, filled, .. } => {
                for (index, point) in points.iter().enumerate() {
                    finite_point(&format!("point {index}"), *point)?;
                }
                let mut distinct = points.clone();
                if distinct.len() > 1 && distinct.first() == distinct.last() {
                    distinct.pop();
                }
                anyhow::ensure!(
                    distinct.len() >= 3,
                    "a polygon needs at least 3 distinct points, got {}",
                    distinct.len()
                );
                let twice_area: f64 = distinct
                    .iter()
                    .zip(distinct.iter().cycle().skip(1))
                    .map(|(a, b)| a.0 * b.1 - b.0 * a.1)
                    .sum();
                anyhow::ensure!(
                    twice_area.abs() > 1e-12,
                    "the polygon encloses no area (its points are collinear)"
                );
                anyhow::ensure!(
                    *filled || width > 0.0,
                    "an unfilled polygon with zero stroke width draws no copper"
                );
            }
            Self::Rect {
                start, end, filled, ..
            } => {
                finite_point("start", *start)?;
                finite_point("end", *end)?;
                anyhow::ensure!(
                    start.0 != end.0 && start.1 != end.1,
                    "the rectangle from {start:?} to {end:?} has no area"
                );
                anyhow::ensure!(
                    *filled || width > 0.0,
                    "an unfilled rectangle with zero stroke width draws no copper"
                );
            }
            Self::Circle {
                center,
                end,
                filled,
                ..
            } => {
                finite_point("center", *center)?;
                finite_point("end", *end)?;
                anyhow::ensure!(center != end, "the circle has zero radius");
                anyhow::ensure!(
                    *filled || width > 0.0,
                    "an unfilled circle with zero stroke width draws no copper"
                );
            }
            Self::Segment { start, end, .. } => {
                finite_point("start", *start)?;
                finite_point("end", *end)?;
                anyhow::ensure!(start != end, "the line has zero length");
                anyhow::ensure!(width > 0.0, "a line with zero stroke width draws no copper");
            }
            Self::Arc {
                start, mid, end, ..
            } => {
                finite_point("start", *start)?;
                finite_point("mid", *mid)?;
                finite_point("end", *end)?;
                anyhow::ensure!(
                    circle_through(*start, *mid, *end).is_some(),
                    "the arc's start, mid and end points are collinear or coincide"
                );
                anyhow::ensure!(width > 0.0, "an arc with zero stroke width draws no copper");
            }
        }
        Ok(())
    }
}

/// Centre and radius of the circle through three points, or `None` when they
/// are collinear (or two coincide).
fn circle_through(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> Option<((f64, f64), f64)> {
    let d = 2.0 * (a.0 * (b.1 - c.1) + b.0 * (c.1 - a.1) + c.0 * (a.1 - b.1));
    if d.abs() < 1e-12 {
        return None;
    }
    let sq = |p: (f64, f64)| p.0 * p.0 + p.1 * p.1;
    let ux = (sq(a) * (b.1 - c.1) + sq(b) * (c.1 - a.1) + sq(c) * (a.1 - b.1)) / d;
    let uy = (sq(a) * (c.0 - b.0) + sq(b) * (a.0 - c.0) + sq(c) * (b.0 - a.0)) / d;
    Some(((ux, uy), (a.0 - ux).hypot(a.1 - uy)))
}

/// A footprint graphic item in footprint-local coordinates (mm), parsed from
/// the library `.kicad_mod` source.
///
/// Points are pre-transform: `build_footprint_item` rotates and translates
/// them into absolute board coordinates before emission, because KiCAD
/// serializes `FootprintInstance` children in absolute board space (see the
/// `transform` module docs / issue #23).
#[derive(Debug, Clone, PartialEq)]
pub enum IpcGraphicDefinition {
    /// `fp_line` — straight segment.
    Line {
        start: (f64, f64),
        end: (f64, f64),
        layer: String,
        width: f64,
    },
    /// `fp_rect` — axis-aligned rectangle between two opposite corners.
    Rect {
        start: (f64, f64),
        end: (f64, f64),
        layer: String,
        width: f64,
        filled: bool,
    },
    /// `fp_circle` — center plus a point on the circumference.
    Circle {
        center: (f64, f64),
        end: (f64, f64),
        layer: String,
        width: f64,
        filled: bool,
    },
    /// `fp_arc` — start / mid / end points.
    Arc {
        start: (f64, f64),
        mid: (f64, f64),
        end: (f64, f64),
        layer: String,
        width: f64,
    },
    /// `fp_poly` — closed outline.
    Poly {
        points: Vec<(f64, f64)>,
        layer: String,
        width: f64,
        filled: bool,
    },
    /// Visible `fp_text` / `property` text.
    Text {
        text: String,
        position: (f64, f64),
        /// Text angle in degrees, footprint-local.
        rotation: f64,
        layer: String,
        /// Glyph size (width and height) in mm.
        size: f64,
        /// Font stroke width in mm.
        stroke_width_mm: f64,
    },
}

impl IpcGraphicDefinition {
    /// The KiCAD layer name this item draws on.
    pub fn layer(&self) -> &str {
        match self {
            Self::Line { layer, .. }
            | Self::Rect { layer, .. }
            | Self::Circle { layer, .. }
            | Self::Arc { layer, .. }
            | Self::Poly { layer, .. }
            | Self::Text { layer, .. } => layer,
        }
    }

    /// What this item is, for an error that has to name it.
    pub fn kind(&self) -> &'static str {
        match self {
            Self::Line { .. } => "fp_line",
            Self::Rect { .. } => "fp_rect",
            Self::Circle { .. } => "fp_circle",
            Self::Arc { .. } => "fp_arc",
            Self::Poly { .. } => "fp_poly",
            Self::Text { .. } => "fp_text",
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IpcTrack {
    /// KIID of the track, needed to delete it via delete_track. Empty only if
    /// KiCAD returned a track without an id.
    pub uuid: String,
    pub net_name: String,
    pub layer: String,
    pub width: f64,
    pub start: IpcVector2,
    pub end: IpcVector2,
}

/// A graphic item inside a placed footprint — silkscreen, fabrication, or
/// courtyard artwork, not a pad.
///
/// `points` are footprint-local millimetres, matching what the `.kicad_mod`
/// shows, even though KiCad carries them in absolute board coordinates.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IpcFootprintGraphic {
    pub uuid: String,
    pub kind: String,
    pub layer: String,
    pub points: Vec<IpcVector2>,
    /// How many outlines a polygon's `PolySet` carries, and how many holes
    /// across them; `0` for every other kind. `points` reports the first
    /// outline only, so anything above `1` outline or above `0` holes means
    /// this listing is not the whole shape — hence stating it rather than
    /// letting the caller infer a simple triangle from three points.
    pub outlines: usize,
    pub holes: usize,
    /// Whether `edit_board_footprint_graphic` can address this item: a
    /// single-outline polygon with no holes, carrying a UUID.
    pub editable: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IpcNet {
    pub name: String,
    pub netcode: i32,
}

/// A board graphic — a shape, text, textbox, or dimension — read back from
/// KiCad.
///
/// `kind` is normalized (`line`, `rect`, `arc`, `circle`, `poly`, `curve`,
/// `text`, `textbox`, `dimension`) so the live and the file reader answer in
/// one vocabulary rather than protobuf names on one side and `gr_*` file tags
/// on the other.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IpcGraphic {
    pub uuid: String,
    pub kind: String,
    pub layer: String,
    /// First defining point in mm: a segment's start, a rectangle's top-left,
    /// an arc's start, a circle's centre, a polygon's first vertex, a text's
    /// position. `None` when KiCad sent no geometry.
    pub origin: Option<IpcVector2>,
}

/// Effective PCB routing rules returned by KiCad for one net.
///
/// Values are optional because KiCad's protobuf permits an incomplete class.
/// A routing exporter must refuse an incomplete rule set rather than inventing
/// manufacturing constraints.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct IpcRoutingRules {
    pub class_name: String,
    pub constituents: Vec<String>,
    pub track_width_mm: Option<f64>,
    pub clearance_mm: Option<f64>,
    pub via_diameter_mm: Option<f64>,
    pub via_drill_mm: Option<f64>,
}

/// Net name to its effective (merged) KiCad routing rules.
pub type IpcEffectiveRoutingRules = BTreeMap<String, IpcRoutingRules>;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IpcLayer {
    pub name: String,
    pub id: i32,
    pub kind: String,
    /// The name KiCad shows for the layer, which the user may have renamed.
    /// `None` when the enabled set was read without asking for it — an empty
    /// string is a name KiCad could have answered with, so absence needs its
    /// own spelling.
    pub display_name: Option<String>,
}

/// The enabled layer set as KiCad reports it.
///
/// `copper_layer_count` is the response's own field, not a count of `layers`
/// whose name ends in `.Cu` — the two agree on an ordinary stackup, and that
/// agreement is exactly what stops holding on an unusual one.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IpcEnabledLayers {
    pub copper_layer_count: u32,
    pub layers: Vec<IpcLayer>,
}

/// A board's physical stackup, as KiCad's `GetBoardStackup` serves it.
///
/// KiCad answers with the board's own stackup, or with its default one when
/// the board defines none (`BOARD::GetStackupOrDefault`), and the message does
/// not say which. Values KiCad marks "Not specified" are kept as it sends them.
///
/// `None` is a value KiCad did not serialize, never a default standing in for
/// one. KiCad 10.0.4's `BOARD_STACKUP::Serialize` writes no finish, impedance
/// or edge settings, no layer `enabled` flag, and attaches each dielectric's
/// sub-layers to a detached message, so none of those reach the wire; 10.0.6
/// fills them all.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct IpcBoardStackup {
    /// The copper finish KiCad names, e.g. "ENIG"; empty or "Not specified"
    /// when the board names none. `None` when KiCad sent no finish message.
    pub finish: Option<String>,
    /// `None` when KiCad sent no impedance message.
    pub impedance_controlled: Option<bool>,
    /// "none", "plain", "beveled" or "unknown"; `None` when KiCad sent no
    /// edge-connector message.
    pub edge_connector: Option<String>,
    /// `None` when KiCad sent no edge-plating message.
    pub has_edge_plating: Option<bool>,
    /// Top to bottom, in KiCad's order.
    pub layers: Vec<IpcStackupLayer>,
}

/// One entry of a physical stackup.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct IpcStackupLayer {
    /// KiCad's canonical layer name, e.g. "F.Cu" or "F.Mask". `None` for a
    /// dielectric, which is not a board layer of its own.
    pub layer: Option<String>,
    /// The name the board gives the layer, when KiCad reports one.
    pub user_name: Option<String>,
    /// "copper", "dielectric", "soldermask", "silkscreen", "solderpaste" or
    /// "undefined".
    pub kind: String,
    /// `None` when the answer carries no evidence that KiCad set the flag: a
    /// proto3 bool has no presence, and KiCad 10.0.4 never sets it, so a
    /// `false` is only believed from an answer that also carries 10.0.6's
    /// board settings or some layer marked enabled.
    pub enabled: Option<bool>,
    /// The entry's thickness in nanometres. A dielectric's own value is its
    /// first sub-layer's; `dielectric` carries every sub-layer.
    pub thickness_nm: i64,
    /// Empty when KiCad reports none. A dielectric's materials are per
    /// sub-layer, in `dielectric`.
    pub material: String,
    /// `#RRGGBBAA`, only when the board specifies a color.
    pub color: Option<String>,
    /// "core", "prepreg", "none", or "unknown" for a value this crate does not
    /// know, for a dielectric. `None` when KiCad left the type unset.
    pub dielectric_type: Option<String>,
    /// A dielectric's physical sub-layers, top to bottom. Empty for a
    /// dielectric whose sub-layers KiCad did not serialize.
    pub dielectric: Vec<IpcStackupDielectric>,
    /// A solder mask's relative permittivity.
    pub epsilon_r: Option<f64>,
    /// A solder mask's loss tangent.
    pub loss_tangent: Option<f64>,
}

/// One physical sub-layer of a dielectric slot. A slot between two copper
/// layers can be built from several.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct IpcStackupDielectric {
    pub thickness_nm: i64,
    pub material: String,
    pub epsilon_r: f64,
    pub loss_tangent: f64,
    pub thickness_locked: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IpcBoardExtents {
    pub min: IpcVector2,
    pub max: IpcVector2,
}

/// What [`crate::client::KiCadIpcClient::get_board_bounds_in`] measured on a
/// live board.
#[derive(Debug, Clone)]
pub struct IpcBoardBounds {
    /// The union of every item's box; `None` only for a board with no items.
    pub extents: Option<IpcBoardExtents>,
    /// How many items of each class KiCad listed and measured, by the name a
    /// response reports the class under. Classes with no items are absent.
    pub measured: std::collections::BTreeMap<String, usize>,
    /// Items KiCad listed under a KIID it had already listed. Each KIID is
    /// measured once, so these have no box of their own.
    pub shared_kiid_count: usize,
    /// Board item classes whose items are not in `extents`, because KiCad
    /// would not list them or listed them in a form this protocol cannot read.
    pub unavailable: Vec<IpcUnavailableItemClass>,
}

/// A class of board items a live bounds read could not measure, and why. A
/// class is named here rather than left out, so an answer never reads as
/// complete when it is not (#688).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IpcUnavailableItemClass {
    /// The class, by the name a response reports it under (`tables`).
    pub class: String,
    pub reason: UnavailableReason,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum UnavailableReason {
    /// KiCad declined to list the class. KiCad 10.0.5 answers `GetItems` for
    /// tables and generators with `AS_BAD_REQUEST`.
    Refused { status: String, message: String },
    /// KiCad listed items of a class the bundled protocol has no message for,
    /// so their KIIDs cannot be read to measure them.
    Undecodable { listed_count: usize },
}

/// Footprint-local placement of the Reference and Value text fields, read
/// from the library footprint so placed parts keep the library's text
/// positions. A hardcoded offset put the Reference on top of the part's own
/// silkscreen (silk_overlap DRC warnings in live verification).
#[derive(Debug, Clone, Copy, Default)]
pub struct IpcFieldPlacement {
    /// (x, y, rotation) of the Reference text, footprint-local mm/degrees.
    pub reference_at: Option<(f64, f64, f64)>,
    /// (x, y, rotation) of the Value text, footprint-local mm/degrees.
    pub value_at: Option<(f64, f64, f64)>,
}

/// A three-component value as KiCad's `Vector3D` message carries it.
///
/// The enclosing field names the quantity and unit. KiCad reuses this wire
/// shape for unitless scale, rotation in degrees, and offset in millimetres,
/// so a bare vector must never be exposed without that context.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct IpcVector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// A footprint's associated 3D model, as read back from a placed instance.
/// `flip_footprint` reports these so a caller can independently confirm the
/// model transform native `FlipItems` applied, without Konnect reimplementing
/// KiCad's own 3D-model flip math.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct IpcFootprint3DModel {
    pub filename: String,
    /// Footprint-local model offset in millimetres.
    pub offset_mm: IpcVector3,
    /// Model rotation around each axis in degrees.
    pub rotation_degrees: IpcVector3,
    /// Unitless model scale along each axis.
    pub scale: IpcVector3,
    pub visible: bool,
    pub opacity: f64,
}

/// One exact live footprint and the ordered 3D-model entries KiCad returned.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct IpcFootprint3DModelSnapshot {
    pub reference: String,
    pub kiid: String,
    pub models: Vec<IpcFootprint3DModel>,
}

/// A bounded edit to the ordered 3D-model entries on one placed footprint.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum IpcFootprint3DModelEdit {
    Append {
        model: IpcFootprint3DModel,
    },
    Replace {
        index: usize,
        model: IpcFootprint3DModel,
    },
    Remove {
        index: usize,
    },
}

/// Readback-derived result of editing a placed footprint's 3D-model entries.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum IpcFootprint3DModelEditOutcome {
    Conflict {
        expected: IpcFootprint3DModelSnapshot,
        observed: IpcFootprint3DModelSnapshot,
    },
    Applied {
        before: IpcFootprint3DModelSnapshot,
        after: IpcFootprint3DModelSnapshot,
        changed: bool,
    },
    Uncertain {
        reference: String,
        kiid: String,
        expected_models: Vec<IpcFootprint3DModel>,
        observed_models: Option<Vec<IpcFootprint3DModel>>,
        reason: String,
    },
}

/// The result of a `flip_footprint` call against a live KiCad IPC session.
///
/// Three outcomes, not two (accepted contract for issue #604): a definite
/// success (`Flipped`), authoritative evidence that this KiCad build has no
/// native `FlipItems` handler (`Unsupported`), and a distinct "reconcile
/// before retrying" state for when the mutation looked like it went through
/// but a fresh readback could not confirm it (`Uncertain`). A hard failure
/// (footprint not found, wrong KIID/type, a non-`ISC_OK` per-item status, or
/// any other rejected request) is not represented here — it is returned as
/// `Err` and classified by the caller's normal IPC-failure handling.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum IpcFlipOutcome {
    /// KiCad flipped the footprint (or it already sat on the requested
    /// layer, in which case no `FlipItems` call was made at all) and a fresh
    /// readback — never the mutation response — independently confirmed the
    /// requested layer and the footprint's identity.
    Flipped {
        reference: String,
        kiid: String,
        previous_layer: String,
        layer: String,
        already_on_layer: bool,
        models: Vec<IpcFootprint3DModel>,
    },
    /// KiCad answered `AS_UNHANDLED`/`AS_UNIMPLEMENTED` for `FlipItems`,
    /// which the 10.0.6 protocol treats as authoritative capability
    /// evidence. Never inferred from a version string.
    Unsupported {
        kicad_version: Option<IpcKiCadVersion>,
    },
    /// The mutation appeared to succeed, but a fresh post-flip readback
    /// disagreed with it or could not be resolved. The board may or may not
    /// have changed; the caller must reconcile before retrying rather than
    /// reporting either success or failure.
    Uncertain {
        reference: String,
        kiid: String,
        requested_layer: String,
        reason: String,
    },
}
