//! Fail-closed KiCad PCB snapshot to Specctra DSN lowering.
//!
//! The input is the exact string returned by KiCad's IPC
//! `SaveDocumentToString`; this module never reads or edits the live board
//! file. Every feature is either represented exactly or conservatively (the
//! router sees at least as much copper as the board has), and anything else is
//! refused.
//!
//! Images are board-oriented: each component is placed front side up with
//! rotation 0, and every pin carries its board offset and absolute pad angle,
//! with padstacks on the copper layers the pad really occupies. Back-side and
//! rotated footprints therefore need no Specctra mirroring convention.

use anyhow::{bail, Context, Result};
use konnect_ipc::IpcEffectiveRoutingRules;
use konnect_sexp::{parse_sexp, SexpNode};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use specctra::read::{ListTokenizer, ReadDsn};
use specctra::structure as dsn;
use specctra::write::{ListWriter, WriteSes};
use std::collections::{BTreeMap, BTreeSet};
use std::io::{BufReader, Cursor};
use std::path::Path;

const UM_PER_MM: f64 = 1_000.0;
const DSN_RESOLUTION: f32 = 10.0;
const MANIFEST_SCHEMA: u32 = 2;
/// Straight edges per quarter circle when a rounded corner is lowered to a
/// circumscribed polygon (worst outward error 1.9 % of the corner radius).
const ARC_SEGMENTS_PER_QUADRANT: usize = 4;

#[derive(Debug, Clone)]
pub(crate) struct ExportBundle {
    pub dsn: String,
    pub manifest: String,
    pub source_sha256: String,
    pub component_count: usize,
    pub pad_count: usize,
    pub net_count: usize,
    pub class_count: usize,
    pub copper_layer_count: usize,
    pub back_component_count: usize,
    pub skipped_pad_count: usize,
    pub keepout_count: usize,
    pub plane_count: usize,
    pub plane_piece_count: usize,
    /// The courtyard clearance rules the DSN carries, and what they became.
    pub courtyard_clearances: Vec<CourtyardClearanceReport>,
    /// `class_class` scopes written for them.
    pub class_pair_rule_count: usize,
}

/// Caller-supplied acknowledgements that widen the exported profile.
#[derive(Debug, Clone, Default)]
pub(crate) struct ExportOptions {
    /// SHA-256 of the board's `.kicad_dru` when the caller has acknowledged
    /// that the router sees only net-class rules and KiCad DRC stays the
    /// authority for the custom ones.
    pub custom_rules_sha256: Option<String>,
    /// Custom rules raising the clearance between items in one footprint's
    /// courtyard, to lower into the DSN (see [`courtyard_clearance_rules`]).
    pub courtyard_clearances: Vec<CourtyardClearance>,
}

/// A custom rule `A.intersectsCourtyard('REF') && B.intersectsCourtyard('REF')`
/// with `(constraint clearance (min X))`: copper inside footprint REF's
/// courtyard keeps at least X from other copper there.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub(crate) struct CourtyardClearance {
    pub rule_name: String,
    pub reference: String,
    pub clearance_um: i64,
}

/// One represented courtyard rule in the export answer.
#[derive(Debug, Clone, Serialize)]
pub(crate) struct CourtyardClearanceReport {
    #[serde(flatten)]
    pub rule: CourtyardClearance,
    /// Nets with a pad on the footprint whose class clearance is below the
    /// rule's: the nets the DSN gives the field clearance.
    pub raised_net_count: usize,
    /// False when the board has no footprint with that reference.
    pub footprint_found: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
enum PadShape {
    Circle,
    Rect,
    Roundrect,
    Oval,
    Custom,
}

impl PadShape {
    fn label(self) -> &'static str {
        match self {
            PadShape::Circle => "circle",
            PadShape::Rect => "rect",
            PadShape::Roundrect => "roundrect",
            PadShape::Oval => "oval",
            PadShape::Custom => "custom",
        }
    }
}

/// Copper outline of one pad in its own frame (DSN axes, micrometres,
/// before the pin rotation).
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
enum PadGeometry {
    Circle { diameter: i64, x: i64, y: i64 },
    Rect { x1: i64, y1: i64, x2: i64, y2: i64 },
    Polygon(Vec<(i64, i64)>),
    Path { width: i64, coords: Vec<(i64, i64)> },
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
struct PadstackKey {
    shape: PadShape,
    layers: Vec<String>,
    geometry: PadGeometry,
    size_x_um: i64,
    size_y_um: i64,
    drill_um: Option<i64>,
}

#[derive(Debug, Clone)]
struct CopperLayer {
    name: String,
    dsn_type: String,
}

#[derive(Debug, Clone)]
struct PadModel {
    number: String,
    pin_id: String,
    x_um: i64,
    y_um: i64,
    rotation_degrees: f64,
    net: Option<String>,
    padstack: PadstackKey,
    /// Local pad clearance larger than the board default, with the diameter
    /// of the circle that encloses the pad copper.
    clearance_override: Option<(i64, i64)>,
}

/// A keepout circle in image coordinates.
#[derive(Debug, Clone)]
struct KeepoutModel {
    layers: Vec<String>,
    diameter_um: i64,
    x_um: i64,
    y_um: i64,
}

#[derive(Debug, Clone)]
struct FootprintModel {
    reference: String,
    kiid: String,
    image_name: String,
    x_um: i64,
    y_um: i64,
    rotation_degrees: f64,
    side: String,
    pads: Vec<PadModel>,
    keepouts: Vec<KeepoutModel>,
    skipped_pads: usize,
}

#[derive(Debug, Clone)]
struct PlaneModel {
    net: String,
    layer: String,
    pieces: Vec<Vec<(i64, i64)>>,
}

#[derive(Debug, Clone)]
struct LockedTrackModel {
    start_x_um: i64,
    start_y_um: i64,
    end_x_um: i64,
    end_y_um: i64,
    width_um: i64,
    layer: String,
    net: String,
}

#[derive(Debug, Clone)]
struct LockedViaModel {
    x_um: i64,
    y_um: i64,
    diameter_um: i64,
    drill_um: i64,
    net: String,
}

#[derive(Debug, Clone, Default)]
struct LockedRouting {
    tracks: Vec<LockedTrackModel>,
    vias: Vec<LockedViaModel>,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
struct RuleKey {
    track_width_um: i64,
    /// The net-class clearance KiCad reports.
    clearance_um: i64,
    via_diameter_um: i64,
    via_drill_um: i64,
    /// Courtyard fields (reference, clearance) that raise this class's
    /// clearance against the other classes in the same field. Empty for a
    /// board without courtyard rules, so its classes are unchanged.
    fields: Vec<(String, i64)>,
}

impl RuleKey {
    /// The clearance the DSN class carries: against itself, its own nets
    /// share every field.
    fn dsn_clearance_um(&self) -> i64 {
        self.fields
            .iter()
            .map(|(_, clearance)| *clearance)
            .fold(self.clearance_um, i64::max)
    }

    /// The key of the via padstack, which fields do not change.
    fn via_key(&self) -> RuleKey {
        RuleKey {
            fields: Vec::new(),
            ..self.clone()
        }
    }

    /// The clearance two classes need between them: the larger class
    /// clearance, raised by every field both are in.
    fn pair_clearance_um(&self, other: &RuleKey) -> i64 {
        self.fields
            .iter()
            .filter(|field| other.fields.contains(field))
            .map(|(_, clearance)| *clearance)
            .fold(self.clearance_um.max(other.clearance_um), i64::max)
    }
}

type RuleGroups = BTreeMap<RuleKey, Vec<String>>;
type NetClassNames = BTreeMap<String, String>;
/// Each net's courtyard fields, as (reference, clearance) pairs.
type NetFields = BTreeMap<String, Vec<(String, i64)>>;

#[derive(Debug, Deserialize, Serialize)]
struct Manifest {
    schema_version: u32,
    board_path: String,
    source_sha256: String,
    coordinate_unit: String,
    resolution: u32,
    supported_profile: SupportedProfile,
    layers: Vec<ManifestLayer>,
    components: Vec<ManifestComponent>,
    nets: Vec<ManifestNet>,
    padstacks: Vec<ManifestPadstack>,
}

#[derive(Debug, Deserialize, Serialize)]
struct SupportedProfile {
    copper_layers: u32,
    component_side: String,
    pad_shapes: Vec<String>,
    existing_routing: bool,
    #[serde(default)]
    locked_track_count: usize,
    #[serde(default)]
    locked_via_count: usize,
    copper_zones: bool,
    custom_rules: bool,
    outline: String,
    #[serde(default)]
    image_frame: String,
    #[serde(default)]
    plane_count: usize,
    #[serde(default)]
    custom_rules_sha256: Option<String>,
}

#[derive(Debug, Deserialize, Serialize)]
struct ManifestLayer {
    kicad_name: String,
    dsn_name: String,
    index: usize,
    #[serde(default)]
    dsn_type: String,
}

/// `x_um`, `y_um`, `rotation_degrees` and `side` are the DSN placement the
/// session must echo; `kicad_*` record the footprint as KiCad has it.
#[derive(Debug, Deserialize, Serialize)]
struct ManifestComponent {
    reference: String,
    kiid: String,
    image_name: String,
    x_um: i64,
    y_um: i64,
    rotation_degrees: f64,
    side: String,
    #[serde(default)]
    kicad_side: String,
    #[serde(default)]
    kicad_rotation_degrees: f64,
    pads: Vec<ManifestPin>,
}

#[derive(Debug, Deserialize, Serialize)]
struct ManifestPin {
    pad_number: String,
    dsn_pin: String,
    net: Option<String>,
    padstack_name: String,
}

#[derive(Debug, Deserialize, Serialize)]
struct ManifestNet {
    name: String,
    pins: Vec<String>,
    class_name: String,
}

#[derive(Debug, Deserialize, Serialize)]
struct ManifestPadstack {
    name: String,
    purpose: String,
    shape: PadShape,
    layers: Vec<String>,
    size_x_um: i64,
    size_y_um: i64,
    drill_um: Option<i64>,
}

#[cfg(test)]
pub(crate) fn export_dsn(
    board_path: &Path,
    board_source: &str,
    effective_rules: &IpcEffectiveRoutingRules,
) -> Result<ExportBundle> {
    export_dsn_with(
        board_path,
        board_source,
        effective_rules,
        &ExportOptions::default(),
    )
}

pub(crate) fn export_dsn_with(
    board_path: &Path,
    board_source: &str,
    effective_rules: &IpcEffectiveRoutingRules,
    options: &ExportOptions,
) -> Result<ExportBundle> {
    let tree = parse_sexp(board_source).context("parse KiCad IPC board snapshot")?;
    if tree.head() != Some("kicad_pcb") {
        bail!("KiCad IPC snapshot root is not 'kicad_pcb'");
    }

    let layers = copper_layers(&tree)?;
    let copper_layers = layers
        .iter()
        .map(|layer| layer.name.clone())
        .collect::<Vec<_>>();
    let outline = simple_closed_outline(&tree)?;
    let net_table = top_level_net_table(&tree);
    let locked_routing = locked_routing(&tree, &net_table, &copper_layers)?;
    let planes = planes(&tree, &net_table, &layers)?;
    let footprints = footprints(&tree, &net_table, &copper_layers)?;
    if footprints.is_empty() {
        bail!("supported routing profile requires at least one footprint");
    }

    let mut net_pins: BTreeMap<String, Vec<String>> = BTreeMap::new();
    for footprint in &footprints {
        for pad in &footprint.pads {
            if let Some(net) = &pad.net {
                net_pins
                    .entry(net.clone())
                    .or_default()
                    .push(format!("{}-{}", footprint.reference, pad.pin_id));
            }
        }
    }
    for plane in &planes {
        if !net_pins.contains_key(&plane.net) {
            bail!(
                "copper zone on {} belongs to net '{}', which no exported pad uses",
                plane.layer,
                plane.net
            );
        }
    }
    if net_pins.is_empty() {
        bail!("supported routing profile requires at least one connected pad");
    }
    for pins in net_pins.values_mut() {
        pins.sort();
        pins.dedup();
    }

    let (net_fields, mut courtyard_clearances) =
        courtyard_fields(&footprints, &options.courtyard_clearances);
    let (class_nets, net_classes) = normalize_rules(&net_pins, effective_rules, &net_fields)?;
    for report in &mut courtyard_clearances {
        let field = (report.rule.reference.clone(), report.rule.clearance_um);
        report.raised_net_count = class_nets
            .iter()
            .filter(|(rule, _)| rule.fields.contains(&field))
            .map(|(_, nets)| nets.len())
            .sum();
    }

    let mut padstack_keys = BTreeSet::new();
    for footprint in &footprints {
        for pad in &footprint.pads {
            padstack_keys.insert(pad.padstack.clone());
        }
    }
    let padstack_names: BTreeMap<PadstackKey, String> = padstack_keys
        .into_iter()
        .enumerate()
        .map(|(index, key)| (key, format!("konnect_pad_{:04}", index + 1)))
        .collect();

    let via_keys: BTreeSet<RuleKey> = class_nets.keys().map(RuleKey::via_key).collect();
    let via_names: BTreeMap<RuleKey, String> = via_keys
        .into_iter()
        .enumerate()
        .map(|(index, key)| (key, format!("konnect_via_{:04}", index + 1)))
        .collect();
    let fixed_via_names = fixed_via_padstacks(&locked_routing, &class_nets)?;

    let mut footprints = footprints;
    let min_clearance_um = class_nets
        .keys()
        .map(|rule| rule.clearance_um)
        .min()
        .context("routing model has no rule class")?;
    add_clearance_keepouts(&mut footprints, min_clearance_um);

    let pcb = build_pcb(
        board_path,
        &layers,
        &outline,
        &footprints,
        &net_pins,
        &class_nets,
        &net_classes,
        &padstack_names,
        &via_names,
        &fixed_via_names,
        &locked_routing,
        &planes,
    )?;
    let class_pairs = class_pair_rules(&class_nets);
    let dsn = add_class_pair_rules(serialize_and_validate(pcb)?, &class_pairs)?;
    let source_sha256 = sha256_hex(board_source.as_bytes());
    let manifest = build_manifest(
        board_path,
        &source_sha256,
        &layers,
        &footprints,
        &net_pins,
        &net_classes,
        &padstack_names,
        &via_names,
        &fixed_via_names,
        &locked_routing,
        &planes,
        options,
    )?;

    Ok(ExportBundle {
        dsn,
        manifest,
        source_sha256,
        component_count: footprints.len(),
        pad_count: footprints
            .iter()
            .map(|footprint| footprint.pads.len())
            .sum(),
        net_count: net_pins.len(),
        class_count: class_nets.len(),
        copper_layer_count: layers.len(),
        back_component_count: footprints
            .iter()
            .filter(|footprint| footprint.side == "back")
            .count(),
        skipped_pad_count: footprints
            .iter()
            .map(|footprint| footprint.skipped_pads)
            .sum(),
        keepout_count: footprints
            .iter()
            .map(|footprint| footprint.keepouts.len())
            .sum(),
        plane_count: planes.len(),
        plane_piece_count: planes.iter().map(|plane| plane.pieces.len()).sum(),
        courtyard_clearances,
        class_pair_rule_count: class_pairs.len(),
    })
}

/// Fixed vias whose size differs from their net's rule via get a padstack
/// of their own, keyed by diameter and drill.
fn fixed_via_padstacks(
    locked_routing: &LockedRouting,
    class_nets: &RuleGroups,
) -> Result<BTreeMap<(i64, i64), String>> {
    let net_rules = class_nets
        .iter()
        .flat_map(|(rule, nets)| nets.iter().map(move |net| (net.as_str(), rule)))
        .collect::<BTreeMap<_, _>>();
    let mut sizes = BTreeSet::new();
    for via in &locked_routing.vias {
        let rule = net_rules
            .get(via.net.as_str())
            .with_context(|| format!("locked via uses unknown routing net '{}'", via.net))?;
        if via.diameter_um != rule.via_diameter_um || via.drill_um != rule.via_drill_um {
            sizes.insert((via.diameter_um, via.drill_um));
        }
    }
    Ok(sizes
        .into_iter()
        .enumerate()
        .map(|(index, size)| (size, format!("konnect_fixed_via_{:04}", index + 1)))
        .collect())
}

/// A pad whose local clearance exceeds every class clearance is surrounded
/// by a keepout sized so that the router, adding its own class clearance,
/// keeps at least the local clearance from the pad copper.
fn add_clearance_keepouts(footprints: &mut [FootprintModel], min_clearance_um: i64) {
    for footprint in footprints {
        for pad in &footprint.pads {
            let Some((clearance_um, enclosing_um)) = pad.clearance_override else {
                continue;
            };
            if clearance_um <= min_clearance_um {
                continue;
            }
            footprint.keepouts.push(KeepoutModel {
                layers: pad.padstack.layers.clone(),
                diameter_um: enclosing_um + 2 * (clearance_um - min_clearance_um),
                x_um: pad.x_um,
                y_um: pad.y_um,
            });
        }
    }
}

/// Replace Konnect's deterministic DSN with a KiCad-native DSN while keeping
/// the same fail-closed board profile and rewriting the reverse manifest to
/// the exact identifiers KiCad emitted. This lets the strict SES importer
/// remain authoritative instead of trusting a second, looser return path.
pub(crate) fn adopt_native_dsn(
    mut baseline: ExportBundle,
    native_dsn: String,
) -> Result<ExportBundle> {
    validate_dsn_syntax(&native_dsn, "KiCad native DSN")?;
    if baseline.class_pair_rule_count > 0 {
        bail!(
            "KiCad's native DSN cannot carry the courtyard clearance rules in Konnect's DSN \
             ({} class_class rules)",
            baseline.class_pair_rule_count
        );
    }
    let baseline_tree = parse_dsn_sexp(&baseline.dsn).context("parse Konnect baseline DSN")?;
    let native_tree = parse_dsn_sexp(&native_dsn).context("parse KiCad native DSN")?;
    let baseline_identity = dsn_identity(&baseline_tree)?;
    let native_identity = dsn_identity(&native_tree)?;
    let mut manifest: Manifest =
        serde_json::from_str(&baseline.manifest).context("parse baseline routing manifest")?;

    let layer_names = correlate_layers(&baseline_identity, &native_identity)?;
    let (class_names, via_names) = correlate_classes(&baseline_identity, &native_identity)?;
    let padstack_names = correlate_components(&baseline_identity, &native_identity)?;
    validate_native_nets(&baseline_identity, &native_identity)?;

    for layer in &mut manifest.layers {
        layer.dsn_name = layer_names
            .get(&layer.dsn_name)
            .with_context(|| format!("native DSN omitted layer '{}'", layer.dsn_name))?
            .clone();
    }
    for component in &mut manifest.components {
        let native = native_identity
            .components
            .get(&component.reference)
            .with_context(|| format!("native DSN omitted component '{}'", component.reference))?;
        component.image_name = native.image_name.clone();
        for pin in &mut component.pads {
            pin.padstack_name = padstack_names
                .get(&pin.padstack_name)
                .with_context(|| {
                    format!(
                        "native DSN omitted padstack mapping for {}-{}",
                        component.reference, pin.pad_number
                    )
                })?
                .clone();
        }
    }
    for net in &mut manifest.nets {
        net.class_name = class_names
            .get(&net.class_name)
            .with_context(|| format!("native DSN omitted class mapping for '{}'", net.name))?
            .clone();
    }
    for padstack in &mut manifest.padstacks {
        let mapping = if padstack.purpose == "via" {
            &via_names
        } else {
            &padstack_names
        };
        padstack.name = mapping
            .get(&padstack.name)
            .with_context(|| format!("native DSN omitted padstack '{}'", padstack.name))?
            .clone();
    }
    manifest
        .padstacks
        .sort_by(|left, right| left.name.cmp(&right.name));
    baseline.manifest =
        serde_json::to_string_pretty(&manifest).context("serialize native routing manifest")?;
    baseline.dsn = native_dsn;
    Ok(baseline)
}

#[derive(Debug, Clone)]
struct DsnPlacementIdentity {
    image_name: String,
    x: i64,
    y: i64,
    side: String,
    rotation: OrderedF64,
    pins: BTreeMap<String, DsnPinIdentity>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct DsnPinIdentity {
    padstack: String,
    x: i64,
    y: i64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct OrderedF64(u64);

impl OrderedF64 {
    fn new(value: f64) -> Result<Self> {
        if !value.is_finite() {
            bail!("DSN contains a non-finite number");
        }
        Ok(Self(value.to_bits()))
    }
}

#[derive(Debug, Clone)]
struct DsnClassIdentity {
    nets: BTreeSet<String>,
    via_name: String,
    width: i64,
    clearance: i64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum DsnShapeIdentity {
    Circle {
        layer: String,
        diameter: i64,
    },
    Rect {
        layer: String,
        x1: i64,
        y1: i64,
        x2: i64,
        y2: i64,
    },
    /// Polygon, path, or offset circle, compared by its exact S-expression.
    Other(String),
}

#[derive(Debug, Clone)]
struct DsnIdentity {
    layers: BTreeMap<usize, String>,
    components: BTreeMap<String, DsnPlacementIdentity>,
    nets: BTreeMap<String, BTreeSet<String>>,
    classes: BTreeMap<String, DsnClassIdentity>,
    padstacks: BTreeMap<String, Vec<DsnShapeIdentity>>,
}

fn validate_dsn_syntax(source: &str, label: &str) -> Result<()> {
    let cursor = Cursor::new(source.as_bytes());
    let mut tokenizer = ListTokenizer::new(BufReader::new(cursor));
    dsn::DsnFile::read_dsn(&mut tokenizer)
        .map_err(|error| anyhow::anyhow!("{label} failed Specctra parser validation: {error}"))?;
    Ok(())
}

fn parse_dsn_sexp(source: &str) -> Result<SexpNode> {
    // Specctra represents the quote delimiter as the bare token
    // `(string_quote ")`. KiCad's S-expression parser correctly treats the
    // opening quote as a string delimiter, so give only this metadata token an
    // escaped representation for structural inspection. The original DSN is
    // retained byte-for-byte and independently validated by Topola's parser.
    let compatible = source.replace("(string_quote \")", "(string_quote \"\\\"\")");
    parse_sexp(&compatible).context("parse normalized Specctra S-expression")
}

fn dsn_identity(root: &SexpNode) -> Result<DsnIdentity> {
    if root.head() != Some("pcb") {
        bail!("Specctra export root is not 'pcb'");
    }
    let structure = root.find("structure").context("DSN has no structure")?;
    let mut layers = BTreeMap::new();
    for layer in structure.find_all("layer") {
        let name = dsn_atom(layer, 1, "layer name")?.to_string();
        let index = layer
            .find("property")
            .and_then(|property| property.find("index"))
            .and_then(|index| index.get(1))
            .and_then(SexpNode::as_str)
            .context("DSN layer has no index")?
            .parse::<usize>()
            .context("DSN layer index is not an integer")?;
        if layers.insert(index, name).is_some() {
            bail!("DSN repeats a layer index");
        }
    }

    let library = root.find("library").context("DSN has no library")?;
    let mut image_pins = BTreeMap::<String, BTreeMap<String, DsnPinIdentity>>::new();
    for image in library.find_all("image") {
        let name = dsn_atom(image, 1, "image name")?.to_string();
        let mut pins = BTreeMap::new();
        for pin in image.find_all("pin") {
            let padstack = dsn_atom(pin, 1, "pin padstack")?.to_string();
            let number = dsn_atom(pin, 2, "pin number")?.to_string();
            let identity = DsnPinIdentity {
                padstack,
                x: dsn_integer(pin, 3, "pin x")?,
                y: dsn_integer(pin, 4, "pin y")?,
            };
            if pins.insert(number, identity).is_some() {
                bail!("DSN image '{name}' repeats a pin number");
            }
        }
        if image_pins.insert(name.clone(), pins).is_some() {
            bail!("DSN repeats image '{name}'");
        }
    }
    let mut padstacks = BTreeMap::new();
    for padstack in library.find_all("padstack") {
        let name = dsn_atom(padstack, 1, "padstack name")?.to_string();
        let shapes = padstack
            .find_all("shape")
            .into_iter()
            .map(|shape| {
                let geometry = shape.get(1).context("DSN shape has no geometry")?;
                match geometry.head() {
                    Some("circle") if geometry.children().is_some_and(|c| c.len() > 3) => {
                        Ok(DsnShapeIdentity::Other(format!("{geometry:?}")))
                    }
                    Some("polygon" | "path") => {
                        Ok(DsnShapeIdentity::Other(format!("{geometry:?}")))
                    }
                    Some("circle") => Ok(DsnShapeIdentity::Circle {
                        layer: dsn_atom(geometry, 1, "circle layer")?.to_string(),
                        diameter: dsn_integer(geometry, 2, "circle diameter")?,
                    }),
                    Some("rect") => Ok(DsnShapeIdentity::Rect {
                        layer: dsn_atom(geometry, 1, "rect layer")?.to_string(),
                        x1: dsn_integer(geometry, 2, "rect x1")?,
                        y1: dsn_integer(geometry, 3, "rect y1")?,
                        x2: dsn_integer(geometry, 4, "rect x2")?,
                        y2: dsn_integer(geometry, 5, "rect y2")?,
                    }),
                    other => bail!("DSN padstack has unsupported shape {other:?}"),
                }
            })
            .collect::<Result<Vec<_>>>()?;
        if shapes.is_empty() || padstacks.insert(name.clone(), shapes).is_some() {
            bail!("DSN contains an empty or duplicate padstack '{name}'");
        }
    }

    let placement = root.find("placement").context("DSN has no placement")?;
    let mut components = BTreeMap::new();
    for component in placement.find_all("component") {
        let image_name = dsn_atom(component, 1, "component image")?.to_string();
        let pins = image_pins
            .get(&image_name)
            .with_context(|| format!("DSN placement uses unknown image '{image_name}'"))?
            .clone();
        for place in component.find_all("place") {
            let reference = dsn_atom(place, 1, "place reference")?.to_string();
            let x = dsn_integer(place, 2, "place x")?;
            let y = dsn_integer(place, 3, "place y")?;
            let side = dsn_atom(place, 4, "place side")?.to_string();
            let rotation = OrderedF64::new(dsn_number(place, 5, "place rotation")?)?;
            let identity = DsnPlacementIdentity {
                image_name: image_name.clone(),
                x,
                y,
                side,
                rotation,
                pins: pins.clone(),
            };
            if components.insert(reference.clone(), identity).is_some() {
                bail!("DSN repeats component '{reference}'");
            }
        }
    }

    let network = root.find("network").context("DSN has no network")?;
    let mut nets = BTreeMap::new();
    for net in network.find_all("net") {
        let name = dsn_atom(net, 1, "net name")?.to_string();
        let pins = net
            .find("pins")
            .context("DSN net has no pins")?
            .children()
            .context("DSN pins is not a list")?
            .iter()
            .skip(1)
            .map(|pin| {
                pin.as_str()
                    .map(str::to_string)
                    .context("DSN pins contains a list")
            })
            .collect::<Result<BTreeSet<_>>>()?;
        if pins.is_empty() || nets.insert(name.clone(), pins).is_some() {
            bail!("DSN contains an empty or duplicate net '{name}'");
        }
    }
    let mut classes = BTreeMap::new();
    for class in network.find_all("class") {
        let name = dsn_atom(class, 1, "class name")?.to_string();
        let children = class.children().context("DSN class is not a list")?;
        let nets = children
            .iter()
            .skip(2)
            .take_while(|child| child.as_str().is_some())
            .map(|child| child.as_str().unwrap().to_string())
            .collect::<BTreeSet<_>>();
        let via_name = class
            .find("circuit")
            .and_then(|circuit| circuit.find("use_via"))
            .and_then(|via| via.get(1))
            .and_then(SexpNode::as_str)
            .context("DSN class has no use_via")?
            .to_string();
        let rule = class.find("rule").context("DSN class has no rule")?;
        let width = rule.find("width").context("DSN class has no width")?;
        let clearance = rule
            .find("clearance")
            .context("DSN class has no clearance")?;
        let width = dsn_integer(width, 1, "class width")?;
        let clearance = dsn_integer(clearance, 1, "class clearance")?;
        if nets.is_empty()
            || !nets.iter().all(|net| {
                network
                    .find_all("net")
                    .iter()
                    .any(|node| node.get(1).and_then(SexpNode::as_str) == Some(net))
            })
            || classes
                .insert(
                    name.clone(),
                    DsnClassIdentity {
                        nets,
                        via_name,
                        width,
                        clearance,
                    },
                )
                .is_some()
        {
            bail!("DSN contains an invalid or duplicate class '{name}'");
        }
    }
    Ok(DsnIdentity {
        layers,
        components,
        nets,
        classes,
        padstacks,
    })
}

fn correlate_layers(
    baseline: &DsnIdentity,
    native: &DsnIdentity,
) -> Result<BTreeMap<String, String>> {
    if baseline.layers.len() != native.layers.len() {
        bail!("native DSN changed the copper-layer count");
    }
    let mappings = baseline
        .layers
        .iter()
        .map(|(index, baseline_name)| {
            let native_name = native
                .layers
                .get(index)
                .with_context(|| format!("native DSN omitted layer index {index}"))?;
            Ok((baseline_name.clone(), native_name.clone()))
        })
        .collect::<Result<BTreeMap<_, _>>>()?;
    if mappings.iter().any(|(baseline, native)| baseline != native) {
        bail!("native DSN renamed a KiCad copper layer");
    }
    Ok(mappings)
}

fn correlate_components(
    baseline: &DsnIdentity,
    native: &DsnIdentity,
) -> Result<BTreeMap<String, String>> {
    if baseline.components.len() != native.components.len() {
        bail!("native DSN changed the component count");
    }
    let mut padstacks = BTreeMap::new();
    for (reference, baseline_component) in &baseline.components {
        let native_component = native
            .components
            .get(reference)
            .with_context(|| format!("native DSN omitted component '{reference}'"))?;
        if baseline_component.x != native_component.x
            || baseline_component.y != native_component.y
            || baseline_component.side != native_component.side
            || baseline_component.rotation != native_component.rotation
            || baseline_component.pins.keys().collect::<Vec<_>>()
                != native_component.pins.keys().collect::<Vec<_>>()
        {
            bail!("native DSN changed placement or pins for component '{reference}'");
        }
        for (pin, baseline_pin) in &baseline_component.pins {
            let native_pin = native_component.pins.get(pin).unwrap();
            if baseline_pin.x != native_pin.x || baseline_pin.y != native_pin.y {
                bail!("native DSN changed pin position for component '{reference}' pin '{pin}'");
            }
            let baseline_shape = baseline
                .padstacks
                .get(&baseline_pin.padstack)
                .with_context(|| {
                    format!("baseline DSN omitted padstack '{}'", baseline_pin.padstack)
                })?;
            let native_shape = native
                .padstacks
                .get(&native_pin.padstack)
                .with_context(|| {
                    format!("native DSN omitted padstack '{}'", native_pin.padstack)
                })?;
            if baseline_shape != native_shape {
                bail!("native DSN changed pad geometry for component '{reference}' pin '{pin}'");
            }
            if padstacks
                .insert(baseline_pin.padstack.clone(), native_pin.padstack.clone())
                .is_some_and(|previous| previous != native_pin.padstack)
            {
                bail!("native DSN maps one pad geometry to inconsistent padstacks");
            }
        }
    }
    Ok(padstacks)
}

fn validate_native_nets(baseline: &DsnIdentity, native: &DsnIdentity) -> Result<()> {
    if baseline.nets != native.nets {
        bail!("native DSN changed net or pin membership");
    }
    Ok(())
}

fn correlate_classes(
    baseline: &DsnIdentity,
    native: &DsnIdentity,
) -> Result<(BTreeMap<String, String>, BTreeMap<String, String>)> {
    if baseline.classes.len() != native.classes.len() {
        bail!("native DSN changed the routing-class count");
    }
    let mut class_names = BTreeMap::new();
    let mut via_names = BTreeMap::new();
    for (baseline_name, baseline_class) in &baseline.classes {
        let matches = native
            .classes
            .iter()
            .filter(|(_, native_class)| native_class.nets == baseline_class.nets)
            .collect::<Vec<_>>();
        if matches.len() != 1 {
            bail!("native DSN has no unique class for '{baseline_name}'");
        }
        let (native_name, native_class) = matches[0];
        if baseline_class.width != native_class.width
            || baseline_class.clearance != native_class.clearance
        {
            bail!("native DSN changed width or clearance for class '{baseline_name}'");
        }
        let baseline_via = baseline
            .padstacks
            .get(&baseline_class.via_name)
            .with_context(|| format!("baseline DSN omitted via '{}'", baseline_class.via_name))?;
        let native_via = native
            .padstacks
            .get(&native_class.via_name)
            .with_context(|| format!("native DSN omitted via '{}'", native_class.via_name))?;
        if baseline_via != native_via {
            bail!("native DSN changed via geometry for class '{baseline_name}'");
        }
        class_names.insert(baseline_name.clone(), native_name.clone());
        if via_names
            .insert(
                baseline_class.via_name.clone(),
                native_class.via_name.clone(),
            )
            .is_some_and(|previous| previous != native_class.via_name)
        {
            bail!("native DSN maps one via rule to inconsistent padstacks");
        }
    }
    Ok((class_names, via_names))
}

fn dsn_atom<'a>(node: &'a SexpNode, index: usize, label: &str) -> Result<&'a str> {
    node.get(index)
        .and_then(SexpNode::as_str)
        .with_context(|| format!("DSN is missing {label}"))
}

fn dsn_number(node: &SexpNode, index: usize, label: &str) -> Result<f64> {
    let value = dsn_atom(node, index, label)?
        .parse::<f64>()
        .with_context(|| format!("DSN {label} is not numeric"))?;
    if !value.is_finite() {
        bail!("DSN {label} is not finite");
    }
    Ok(value)
}

fn dsn_integer(node: &SexpNode, index: usize, label: &str) -> Result<i64> {
    let value = dsn_number(node, index, label)?;
    if value.fract() != 0.0 || value < i64::MIN as f64 || value > i64::MAX as f64 {
        bail!("DSN {label} is not a supported integer");
    }
    Ok(value as i64)
}

fn locked_routing(
    tree: &SexpNode,
    net_table: &BTreeMap<String, String>,
    copper_layers: &[String],
) -> Result<LockedRouting> {
    let through_span = [
        copper_layers
            .first()
            .context("board has no copper layers")?
            .clone(),
        copper_layers
            .last()
            .context("board has no copper layers")?
            .clone(),
    ];
    let arc_count = tree.find_all("arc").len();
    if arc_count > 0 {
        let locked = tree
            .find_all("arc")
            .iter()
            .filter(|arc| arc.find_str("locked") == Some("yes"))
            .count();
        if locked > 0 {
            bail!(
                "unsupported routing profile: board contains {locked} locked routed arc(s), which cannot be represented without approximation"
            );
        }
        bail!("unsupported routing profile: board contains {arc_count} existing routed arc(s)");
    }

    let mut routing = LockedRouting::default();
    for segment in tree.find_all("segment") {
        if segment.find_str("locked") != Some("yes") {
            bail!("unsupported routing profile: board contains an existing track segment that is not locked");
        }
        let layer = segment
            .find_str("layer")
            .context("locked track segment has no layer")?
            .to_string();
        if !copper_layers.contains(&layer) {
            bail!("locked track segment uses unsupported layer '{layer}'");
        }
        let net = segment
            .find("net")
            .and_then(|node| resolve_net(node, net_table))
            .context("locked track segment has no connected net")?;
        let (start_x_um, start_y_um) = point_um(segment, "start")?;
        let (end_x_um, end_y_um) = point_um(segment, "end")?;
        if start_x_um == end_x_um && start_y_um == end_y_um {
            bail!("locked track segment has zero length");
        }
        routing.tracks.push(LockedTrackModel {
            start_x_um,
            start_y_um,
            end_x_um,
            end_y_um,
            width_um: positive_um(
                segment.find("width").and_then(|width| width.get_f64(1)),
                "locked track width",
            )?,
            layer,
            net,
        });
    }
    for via in tree.find_all("via") {
        if via.find_str("locked") != Some("yes") {
            bail!("unsupported routing profile: board contains an existing via that is not locked");
        }
        if let Some(kind) = via.find_str("type") {
            bail!("unsupported locked via type '{kind}'; only through vias are supported");
        }
        let layers = via
            .find("layers")
            .and_then(SexpNode::children)
            .unwrap_or(&[])
            .iter()
            .skip(1)
            .filter_map(SexpNode::as_str)
            .map(str::to_string)
            .collect::<Vec<_>>();
        if layers != through_span {
            bail!(
                "unsupported locked via layer span [{}]; expected a through via [{}]",
                layers.join(", "),
                through_span.join(", ")
            );
        }
        let net = via
            .find("net")
            .and_then(|node| resolve_net(node, net_table))
            .context("locked via has no connected net")?;
        let (x_um, y_um) = point_um(via, "at")?;
        let diameter_um = positive_um(
            via.find("size").and_then(|size| size.get_f64(1)),
            "locked via diameter",
        )?;
        let drill = via.find("drill").context("locked via has no drill")?;
        if drill.get(1).and_then(SexpNode::as_str) == Some("oval") {
            bail!("unsupported locked via with oval drill");
        }
        let drill_um = positive_um(drill.get_f64(1), "locked via drill")?;
        if drill_um >= diameter_um {
            bail!("locked via drill is not smaller than its diameter");
        }
        routing.vias.push(LockedViaModel {
            x_um,
            y_um,
            diameter_um,
            drill_um,
            net,
        });
    }
    routing.tracks.sort_by(|left, right| {
        (
            &left.net,
            &left.layer,
            left.start_x_um,
            left.start_y_um,
            left.end_x_um,
            left.end_y_um,
        )
            .cmp(&(
                &right.net,
                &right.layer,
                right.start_x_um,
                right.start_y_um,
                right.end_x_um,
                right.end_y_um,
            ))
    });
    routing.vias.sort_by(|left, right| {
        (&left.net, left.x_um, left.y_um).cmp(&(&right.net, right.x_um, right.y_um))
    });
    Ok(routing)
}

/// Copper layers in physical order (F.Cu, In1.Cu.., B.Cu) with the Specctra
/// layer type taken from KiCad's: `power` layers are planes the router does
/// not draw tracks on.
fn copper_layers(tree: &SexpNode) -> Result<Vec<CopperLayer>> {
    let layers = tree.find("layers").context("board has no layers table")?;
    let mut copper = Vec::new();
    for row in layers.children().unwrap_or(&[]).iter().skip(1) {
        // Layer table rows are `(numeric-id "name" type ["user name"])`, so
        // the layer name is the first data item.
        let Some(name) = row.get(1).and_then(SexpNode::as_str) else {
            continue;
        };
        if !name.ends_with(".Cu") {
            continue;
        }
        let order = match name {
            "F.Cu" => 0,
            "B.Cu" => usize::MAX,
            inner => inner
                .strip_prefix("In")
                .and_then(|rest| rest.strip_suffix(".Cu"))
                .and_then(|number| number.parse::<usize>().ok())
                .with_context(|| format!("unrecognised copper layer name '{inner}'"))?,
        };
        let dsn_type = match row.get(2).and_then(SexpNode::as_str) {
            Some("signal") | None => "signal",
            Some("power") => "power",
            Some("mixed") => "mixed",
            Some(other) => bail!("copper layer {name} has unsupported type '{other}'"),
        };
        copper.push((
            order,
            CopperLayer {
                name: name.to_string(),
                dsn_type: dsn_type.to_string(),
            },
        ));
    }
    copper.sort_by_key(|(order, _)| *order);
    copper.dedup_by_key(|(order, _)| *order);
    let copper = copper
        .into_iter()
        .map(|(_, layer)| layer)
        .collect::<Vec<_>>();
    let names = copper
        .iter()
        .map(|layer| layer.name.as_str())
        .collect::<Vec<_>>();
    if names.len() < 2 || names[0] != "F.Cu" || names[names.len() - 1] != "B.Cu" {
        bail!(
            "unsupported copper stack: expected F.Cu .. B.Cu, got [{}]",
            names.join(", ")
        );
    }
    if !copper.iter().any(|layer| layer.dsn_type != "power") {
        bail!("every copper layer is a power plane; nothing can be routed");
    }
    Ok(copper)
}

/// Copper zones on power layers become Specctra planes. A zone outline with
/// holes is cut into hole-free trapezoids, which together cover exactly the
/// outline minus its holes. Zones on routing layers and rule areas are
/// refused: their filled copper depends on what is routed around them.
fn planes(
    tree: &SexpNode,
    net_table: &BTreeMap<String, String>,
    layers: &[CopperLayer],
) -> Result<Vec<PlaneModel>> {
    for footprint in tree.find_all("footprint") {
        if !footprint.find_all("zone").is_empty() {
            let reference = property_value(footprint, "Reference").unwrap_or("?");
            bail!("footprint '{reference}' contains a zone or rule area, which is not supported");
        }
    }
    let mut planes = Vec::new();
    let mut placed: Vec<PlacedPlane> = Vec::new();
    for zone in tree.find_all("zone") {
        let name = zone.find_str("name").unwrap_or("").to_string();
        let label = if name.is_empty() {
            "copper zone".to_string()
        } else {
            format!("zone '{name}'")
        };
        if zone.find("keepout").is_some() {
            bail!("{label} is a rule area; rule areas are not supported");
        }
        let mut zone_layers = Vec::new();
        if let Some(layer) = zone.find_str("layer") {
            zone_layers.push(layer.to_string());
        }
        if let Some(list) = zone.find("layers") {
            zone_layers.extend(
                list.children()
                    .unwrap_or(&[])
                    .iter()
                    .skip(1)
                    .filter_map(SexpNode::as_str)
                    .map(str::to_string),
            );
        }
        if zone_layers.is_empty() {
            bail!("{label} has no layer");
        }
        let net = zone
            .find("net")
            .and_then(|node| resolve_net(node, net_table))
            .with_context(|| format!("{label} has no net"))?;
        let mut rings = Vec::new();
        for polygon in zone.find_all("polygon") {
            let pts = polygon.find("pts").context("zone polygon has no points")?;
            let mut ring = Vec::new();
            for point in pts.children().unwrap_or(&[]).iter().skip(1) {
                match point.head() {
                    Some("xy") => ring.push((
                        finite_um(point.get_f64(1), "zone x")?,
                        -finite_um(point.get_f64(2), "zone y")?,
                    )),
                    other => bail!(
                        "{label} outline contains {other:?}; only straight edges are supported"
                    ),
                }
            }
            if ring.len() < 3 {
                bail!("{label} has a polygon with fewer than three points");
            }
            rings.push(ring);
        }
        if rings.is_empty() {
            bail!("{label} has no outline");
        }
        let pieces = trapezoids(&rings);
        if pieces.is_empty() {
            bail!("{label} has an empty outline");
        }
        for layer in zone_layers {
            let copper = layers
                .iter()
                .find(|candidate| candidate.name == layer)
                .with_context(|| format!("{label} is on non-copper layer '{layer}'"))?;
            if copper.dsn_type != "power" {
                bail!(
                    "{label} is on routing layer {layer}; pour zones on routing layers after routing, or make {layer} a power layer"
                );
            }
            // Split planes are fine; overlapping ones would make the router
            // believe two nets' copper shares an area.
            if let Some((other, _)) = placed
                .iter()
                .filter(|(_, (other_layer, _))| *other_layer == layer)
                .find(|(_, (_, other_pieces))| pieces_overlap(&pieces, other_pieces))
            {
                bail!("{label} overlaps {other} on {layer}; overlapping planes are not supported");
            }
            placed.push((label.clone(), (layer.clone(), pieces.clone())));
            planes.push(PlaneModel {
                net: net.clone(),
                layer,
                pieces: pieces.clone(),
            });
        }
    }
    planes.sort_by(|left, right| left.layer.cmp(&right.layer));
    Ok(planes)
}

/// A zone already lowered to plane pieces: its label, layer and pieces.
type PlacedPlane = (String, (String, Vec<Vec<(i64, i64)>>));

/// The vertical extent of a convex piece at `x`, if the line x = `x` meets it.
fn piece_span_at(piece: &[(i64, i64)], x: f64) -> Option<(f64, f64)> {
    let mut low = f64::INFINITY;
    let mut high = f64::NEG_INFINITY;
    for edge in piece.windows(2) {
        let ((x1, y1), (x2, y2)) = (edge[0], edge[1]);
        let (x1, y1, x2, y2) = (x1 as f64, y1 as f64, x2 as f64, y2 as f64);
        if (x < x1.min(x2)) || (x > x1.max(x2)) {
            continue;
        }
        let ys = if x1 == x2 {
            vec![y1, y2]
        } else {
            vec![y1 + (y2 - y1) * (x - x1) / (x2 - x1)]
        };
        for y in ys {
            low = low.min(y);
            high = high.max(y);
        }
    }
    (low <= high).then_some((low, high))
}

/// Whether two sets of trapezoid pieces share any area (touching is not
/// overlapping). Each piece has vertical sides and straight top and bottom
/// edges, so within a shared x-span the overlap of two pieces changes
/// linearly and is found at the span's ends.
fn pieces_overlap(a: &[Vec<(i64, i64)>], b: &[Vec<(i64, i64)>]) -> bool {
    let x_range = |piece: &[(i64, i64)]| {
        let xs = piece.iter().map(|p| p.0);
        (xs.clone().min().unwrap_or(0), xs.max().unwrap_or(0))
    };
    for pa in a {
        let (a1, a2) = x_range(pa);
        for pb in b {
            let (b1, b2) = x_range(pb);
            let (from, to) = (a1.max(b1) as f64, a2.min(b2) as f64);
            if to - from <= 1.0 {
                continue;
            }
            let inset = ((to - from) / 1000.0).min(1.0);
            for x in [from + inset, to - inset] {
                if let (Some((la, ha)), Some((lb, hb))) =
                    (piece_span_at(pa, x), piece_span_at(pb, x))
                {
                    if ha.min(hb) - la.max(lb) > 1.0 {
                        return true;
                    }
                }
            }
        }
    }
    false
}

/// Even-odd decomposition of polygon rings (an outline and its holes) into
/// trapezoids with vertical sides. Slabs are cut at every vertex x, so no
/// edge starts or ends inside a slab; inside each slab the crossing edges
/// pair up bottom to top. Consecutive slabs bounded by the same two edges
/// merge, so a rectilinear outline yields rectangles.
fn trapezoids(rings: &[Vec<(i64, i64)>]) -> Vec<Vec<(i64, i64)>> {
    let mut edges = Vec::new();
    for ring in rings {
        for index in 0..ring.len() {
            let a = ring[index];
            let b = ring[(index + 1) % ring.len()];
            if a.0 != b.0 {
                edges.push(if a.0 < b.0 { (a, b) } else { (b, a) });
            }
        }
    }
    let y_at = |edge: &((i64, i64), (i64, i64)), x: f64| -> f64 {
        let ((x1, y1), (x2, y2)) = *edge;
        y1 as f64 + (y2 - y1) as f64 * (x - x1 as f64) / (x2 - x1) as f64
    };
    let mut xs = rings
        .iter()
        .flatten()
        .map(|point| point.0)
        .collect::<Vec<_>>();
    xs.sort_unstable();
    xs.dedup();

    // (lower edge, upper edge) -> (x where the piece starts)
    let mut open: BTreeMap<(usize, usize), i64> = BTreeMap::new();
    let mut pieces = Vec::new();
    let close = |pair: (usize, usize), from: i64, to: i64, pieces: &mut Vec<Vec<(i64, i64)>>| {
        let (lower, upper) = (&edges[pair.0], &edges[pair.1]);
        let mut ring = vec![
            (from, y_at(lower, from as f64).round() as i64),
            (to, y_at(lower, to as f64).round() as i64),
            (to, y_at(upper, to as f64).round() as i64),
            (from, y_at(upper, from as f64).round() as i64),
        ];
        ring.dedup();
        // A hole edge lying on the outline edge makes a zero-height strip:
        // no copper, so no piece (Freerouting rejects unbounded shapes).
        let doubled_area = ring
            .iter()
            .zip(ring.iter().cycle().skip(1))
            .map(|(a, b)| i128::from(a.0) * i128::from(b.1) - i128::from(b.0) * i128::from(a.1))
            .sum::<i128>();
        if ring.len() >= 3 && doubled_area != 0 {
            let first = ring[0];
            ring.push(first);
            pieces.push(ring);
        }
    };
    for window in xs.windows(2) {
        let (from, to) = (window[0], window[1]);
        let middle = (from as f64 + to as f64) / 2.0;
        let mut crossing = edges
            .iter()
            .enumerate()
            .filter(|(_, edge)| edge.0 .0 <= from && edge.1 .0 >= to)
            .map(|(index, edge)| (y_at(edge, middle), index))
            .collect::<Vec<_>>();
        crossing.sort_by(|left, right| left.0.total_cmp(&right.0));
        let pairs = crossing
            .chunks_exact(2)
            .map(|pair| (pair[0].1, pair[1].1))
            .collect::<BTreeSet<_>>();
        let ended = open
            .keys()
            .filter(|pair| !pairs.contains(pair))
            .copied()
            .collect::<Vec<_>>();
        for pair in ended {
            let start = open.remove(&pair).unwrap();
            close(pair, start, from, &mut pieces);
        }
        for pair in pairs {
            open.entry(pair).or_insert(from);
        }
    }
    if let Some(&last) = xs.last() {
        for (pair, start) in std::mem::take(&mut open) {
            close(pair, start, last, &mut pieces);
        }
    }
    pieces.sort();
    pieces
}

fn simple_closed_outline(tree: &SexpNode) -> Result<Vec<(i64, i64)>> {
    let mut edges = Vec::new();
    for node in tree.children().unwrap_or(&[]) {
        if node.find_str("layer") != Some("Edge.Cuts") {
            continue;
        }
        match node.head() {
            Some("gr_line") => {
                let start = point_um(node, "start").context("Edge.Cuts line has no start")?;
                let end = point_um(node, "end").context("Edge.Cuts line has no end")?;
                if start == end {
                    bail!("unsupported outline: zero-length Edge.Cuts line");
                }
                edges.push((start, end));
            }
            Some(tag) if tag.starts_with("gr_") => {
                bail!("unsupported routing profile: Edge.Cuts '{tag}' is not a straight line")
            }
            _ => {}
        }
    }
    if edges.len() < 3 {
        bail!("unsupported outline: need at least three Edge.Cuts lines");
    }

    let mut adjacency: BTreeMap<(i64, i64), Vec<(i64, i64)>> = BTreeMap::new();
    for (start, end) in &edges {
        adjacency.entry(*start).or_default().push(*end);
        adjacency.entry(*end).or_default().push(*start);
    }
    for (point, neighbours) in &mut adjacency {
        neighbours.sort();
        neighbours.dedup();
        if neighbours.len() != 2 {
            bail!(
                "unsupported outline: vertex ({}, {}) has degree {}, expected 2",
                point.0,
                point.1,
                neighbours.len()
            );
        }
    }

    let start = *adjacency.keys().next().context("outline has no vertices")?;
    let mut ordered = vec![start];
    let mut previous = None;
    let mut current = start;
    loop {
        let neighbours = &adjacency[&current];
        let next = match previous {
            None => neighbours[0],
            Some(previous) if neighbours[0] == previous => neighbours[1],
            Some(_) => neighbours[0],
        };
        if next == start {
            ordered.push(start);
            break;
        }
        if ordered.contains(&next) {
            bail!("unsupported outline: Edge.Cuts contains more than one loop");
        }
        ordered.push(next);
        previous = Some(current);
        current = next;
    }
    if ordered.len() != edges.len() + 1 {
        bail!("unsupported outline: Edge.Cuts is not one closed loop");
    }
    Ok(ordered)
}

fn footprints(
    tree: &SexpNode,
    net_table: &BTreeMap<String, String>,
    copper_layers: &[String],
) -> Result<Vec<FootprintModel>> {
    let mut output = Vec::new();
    let mut references = BTreeSet::new();
    for footprint in tree.find_all("footprint") {
        let layer = footprint
            .find_str("layer")
            .context("footprint has no layer")?;
        let side = match layer {
            "F.Cu" => "front",
            "B.Cu" => "back",
            other => bail!("footprint on unsupported layer '{other}'"),
        };
        if footprint.find("clearance").is_some() {
            bail!("unsupported routing profile: footprint has a local clearance override");
        }
        let reference = property_value(footprint, "Reference")
            .filter(|value| !value.is_empty())
            .context("footprint has no Reference property")?
            .to_string();
        if !references.insert(reference.clone()) {
            bail!("duplicate footprint reference '{reference}'");
        }
        let kiid = footprint
            .find_str("uuid")
            .context("footprint has no UUID")?
            .to_string();
        let at = footprint.find("at").context("footprint has no position")?;
        let x_um = finite_um(at.get_f64(1), "footprint x")?;
        let y_um = -finite_um(at.get_f64(2), "footprint y")?;
        let rotation_degrees = finite_number(at.get_f64(3).or(Some(0.0)), "footprint rotation")?;
        let (sin, cos) = rotation_degrees.to_radians().sin_cos();
        let image_name = format!("konnect_image_{reference}");

        let mut pads = Vec::new();
        let mut keepouts = Vec::new();
        let mut skipped_pads = 0;
        let mut occurrences: BTreeMap<String, usize> = BTreeMap::new();
        let mut pin_ids = BTreeSet::new();
        for pad in footprint.find_all("pad") {
            let number = pad
                .get(1)
                .and_then(SexpNode::as_str)
                .unwrap_or("")
                .to_string();
            let label = format!("{reference}-{number}");
            let pad_type = pad
                .get(2)
                .and_then(SexpNode::as_str)
                .context("pad has no type")?;
            // KiCad stores the pad position relative to the footprint and
            // unrotated (already mirrored for back-side parts), and the pad
            // angle as absolute. The image is placed unrotated, so the pin
            // carries the footprint rotation in its offset.
            let at = pad.find("at").context("pad has no position")?;
            let local = (
                finite_um(at.get_f64(1), "pad x")? as f64,
                -finite_um(at.get_f64(2), "pad y")? as f64,
            );
            let pin_x_um = (local.0 * cos - local.1 * sin).round() as i64;
            let pin_y_um = (local.0 * sin + local.1 * cos).round() as i64;
            let pad_rotation = finite_number(at.get_f64(3).or(Some(0.0)), "pad rotation")?;
            let copper = pad
                .find("layers")
                .and_then(SexpNode::children)
                .unwrap_or(&[])
                .iter()
                .skip(1)
                .filter_map(SexpNode::as_str)
                .filter(|layer| layer.ends_with(".Cu"))
                .map(str::to_string)
                .collect::<Vec<_>>();
            let drill = pad.find("drill");
            let oval_drill = drill
                .and_then(|drill| drill.get(1))
                .and_then(SexpNode::as_str)
                == Some("oval");
            let drill_size = |drill: &SexpNode| -> Result<(i64, i64)> {
                if oval_drill {
                    Ok((
                        positive_um(drill.get_f64(2), "pad drill x")?,
                        positive_um(drill.get_f64(3), "pad drill y")?,
                    ))
                } else {
                    let diameter = positive_um(drill.get_f64(1), "pad drill")?;
                    Ok((diameter, diameter))
                }
            };

            if pad_type == "np_thru_hole" {
                let (drill_x, drill_y) =
                    drill_size(drill.with_context(|| format!("NPTH {label} has no drill"))?)?;
                keepouts.push(KeepoutModel {
                    layers: copper_layers.to_vec(),
                    diameter_um: drill_x.max(drill_y),
                    x_um: pin_x_um,
                    y_um: pin_y_um,
                });
                continue;
            }
            if !matches!(pad_type, "smd" | "connect" | "thru_hole") {
                bail!("unsupported routing profile: pad {label} has type '{pad_type}'");
            }
            let size = pad.find("size").context("pad has no size")?;
            let size_x_um = positive_um(size.get_f64(1), "pad width")?;
            let size_y_um = positive_um(size.get_f64(2), "pad height")?;
            let (layers, drill_um) = if pad_type == "thru_hole" {
                let all = copper.iter().any(|layer| layer == "*.Cu")
                    || (copper.iter().any(|layer| layer == "F.Cu")
                        && copper.iter().any(|layer| layer == "B.Cu"));
                if !all {
                    bail!(
                        "unsupported through-hole pad {label}: copper layers are [{}]",
                        copper.join(", ")
                    );
                }
                let (drill_x, drill_y) = drill_size(
                    drill.with_context(|| format!("through-hole pad {label} has no drill"))?,
                )?;
                let fits = if oval_drill {
                    drill_x < size_x_um && drill_y < size_y_um
                } else {
                    drill_x < size_x_um.min(size_y_um)
                };
                if !fits {
                    bail!("pad {label} drill is not smaller than its copper");
                }
                (copper_layers.to_vec(), Some(drill_x.min(drill_y)))
            } else {
                match copper.as_slice() {
                    [] => {
                        // Paste or mask aperture without copper.
                        skipped_pads += 1;
                        continue;
                    }
                    [one] if one == "F.Cu" || one == "B.Cu" => (vec![one.clone()], None),
                    _ => bail!(
                        "unsupported SMD pad {label}: copper layers are [{}]",
                        copper.join(", ")
                    ),
                }
            };
            let offset = match pad.find("offset") {
                Some(offset) => (
                    finite_um(offset.get_f64(1), "pad offset x")? as f64,
                    -finite_um(offset.get_f64(2), "pad offset y")? as f64,
                ),
                None => (0.0, 0.0),
            };
            let (shape, geometry) = pad_geometry(pad, &label, size_x_um, size_y_um, offset)?;
            let clearance_override = match pad.find("clearance") {
                Some(clearance) => Some((
                    non_negative_um(clearance.get_f64(1), "pad clearance")?,
                    enclosing_diameter_um(&geometry),
                )),
                None => None,
            };

            let seen = occurrences.entry(number.clone()).or_insert(0);
            let pin_id = match (number.is_empty(), *seen) {
                (true, index) => format!("@{index}"),
                (false, 0) => number.clone(),
                (false, index) => format!("{number}@{index}"),
            };
            *seen += 1;
            if !pin_ids.insert(pin_id.clone()) {
                bail!("footprint '{reference}' has pads that collide on pin id '{pin_id}'");
            }
            let net = pad
                .find("net")
                .and_then(|node| resolve_net(node, net_table));
            pads.push(PadModel {
                number,
                pin_id,
                x_um: pin_x_um,
                y_um: pin_y_um,
                rotation_degrees: pad_rotation,
                net,
                padstack: PadstackKey {
                    shape,
                    layers,
                    geometry,
                    size_x_um,
                    size_y_um,
                    drill_um,
                },
                clearance_override,
            });
        }
        if pads.is_empty() && keepouts.is_empty() {
            continue;
        }
        pads.sort_by(|left, right| left.pin_id.cmp(&right.pin_id));
        output.push(FootprintModel {
            reference,
            kiid,
            image_name,
            x_um,
            y_um,
            rotation_degrees,
            side: side.to_string(),
            pads,
            keepouts,
            skipped_pads,
        });
    }
    output.sort_by(|left, right| left.reference.cmp(&right.reference));
    Ok(output)
}

/// Geometry is quantised to the DSN resolution (0.1 um).
fn tenths(value_um: f64) -> i64 {
    (value_um * DSN_RESOLUTION as f64).round() as i64
}

fn pad_geometry(
    pad: &SexpNode,
    label: &str,
    size_x_um: i64,
    size_y_um: i64,
    offset: (f64, f64),
) -> Result<(PadShape, PadGeometry)> {
    let (sx, sy) = (size_x_um as f64, size_y_um as f64);
    let (ox, oy) = offset;
    let rect = || PadGeometry::Rect {
        x1: tenths(ox - sx / 2.0),
        y1: tenths(oy - sy / 2.0),
        x2: tenths(ox + sx / 2.0),
        y2: tenths(oy + sy / 2.0),
    };
    let circle = |diameter: f64| PadGeometry::Circle {
        diameter: tenths(diameter),
        x: tenths(ox),
        y: tenths(oy),
    };
    Ok(match pad.get(3).and_then(SexpNode::as_str) {
        Some("circle") => {
            if size_x_um != size_y_um {
                bail!("circle pad {label} does not have equal X/Y size");
            }
            (PadShape::Circle, circle(sx))
        }
        Some("rect") => (PadShape::Rect, rect()),
        Some("roundrect") => {
            if pad.find_f64("chamfer_ratio").unwrap_or(0.0) > 0.0 && pad.find("chamfer").is_some() {
                bail!("unsupported routing profile: pad {label} has chamfered corners");
            }
            let ratio = pad.find_f64("roundrect_rratio").unwrap_or(0.0);
            if !(0.0..=0.5).contains(&ratio) {
                bail!("pad {label} has an invalid corner ratio {ratio}");
            }
            let radius = ratio * sx.min(sy);
            if radius < 1.0 {
                (PadShape::Roundrect, rect())
            } else {
                (
                    PadShape::Roundrect,
                    PadGeometry::Polygon(rounded_rect(sx, sy, radius, offset)),
                )
            }
        }
        Some("oval") => {
            let geometry = if size_x_um == size_y_um {
                circle(sx)
            } else {
                let width = sx.min(sy);
                let half = (sx.max(sy) - width) / 2.0;
                let (dx, dy) = if sx > sy { (half, 0.0) } else { (0.0, half) };
                PadGeometry::Path {
                    width: tenths(width),
                    coords: vec![
                        (tenths(ox - dx), tenths(oy - dy)),
                        (tenths(ox + dx), tenths(oy + dy)),
                    ],
                }
            };
            (PadShape::Oval, geometry)
        }
        Some("custom") => {
            if ox != 0.0 || oy != 0.0 {
                bail!("unsupported routing profile: custom pad {label} has a shape offset");
            }
            (PadShape::Custom, custom_pad_hull(pad, label, sx, sy)?)
        }
        Some(other) => bail!("unsupported routing profile: pad {label} has shape '{other}'"),
        None => bail!("pad {label} has no shape"),
    })
}

/// Circumscribed polygon of a rounded rectangle: every corner arc is
/// replaced by tangent segments, so the polygon contains the pad.
fn rounded_rect(sx: f64, sy: f64, radius: f64, offset: (f64, f64)) -> Vec<(i64, i64)> {
    let (hx, hy) = (sx / 2.0 - radius, sy / 2.0 - radius);
    let step = std::f64::consts::FRAC_PI_2 / ARC_SEGMENTS_PER_QUADRANT as f64;
    let reach = radius / (step / 2.0).cos();
    let mut ring = Vec::new();
    for (quadrant, (cx, cy)) in [(hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy)]
        .into_iter()
        .enumerate()
    {
        for segment in 0..ARC_SEGMENTS_PER_QUADRANT {
            let angle =
                quadrant as f64 * std::f64::consts::FRAC_PI_2 + (segment as f64 + 0.5) * step;
            ring.push((
                tenths(offset.0 + cx + reach * angle.cos()),
                tenths(offset.1 + cy + reach * angle.sin()),
            ));
        }
    }
    let first = ring[0];
    ring.push(first);
    ring
}

/// Eight points whose octagon contains the circle.
fn circle_points(points: &mut Vec<(f64, f64)>, center: (f64, f64), radius: f64) {
    let step = std::f64::consts::FRAC_PI_4;
    let reach = radius / (step / 2.0).cos();
    for index in 0..8 {
        let angle = (index as f64 + 0.5) * step;
        points.push((
            center.0 + reach * angle.cos(),
            center.1 + reach * angle.sin(),
        ));
    }
}

/// Convex hull of a custom pad's anchor and primitives (pad frame, DSN axes).
/// Freerouting treats pad shapes as convex, and the hull contains the copper.
fn custom_pad_hull(pad: &SexpNode, label: &str, sx: f64, sy: f64) -> Result<PadGeometry> {
    let mut points = Vec::new();
    match pad
        .find("options")
        .and_then(|options| options.find_str("anchor"))
        .unwrap_or("circle")
    {
        "circle" => circle_points(&mut points, (0.0, 0.0), sx / 2.0),
        "rect" => points.extend([
            (-sx / 2.0, -sy / 2.0),
            (sx / 2.0, -sy / 2.0),
            (sx / 2.0, sy / 2.0),
            (-sx / 2.0, sy / 2.0),
        ]),
        other => bail!("custom pad {label} has unsupported anchor '{other}'"),
    }
    let primitives = pad
        .find("primitives")
        .with_context(|| format!("custom pad {label} has no primitives"))?;
    let xy = |node: &SexpNode, tag: &str| -> Result<(f64, f64)> {
        let point = node
            .find(tag)
            .with_context(|| format!("custom pad {label} primitive has no {tag}"))?;
        Ok((
            finite_um(point.get_f64(1), "primitive x")? as f64,
            -finite_um(point.get_f64(2), "primitive y")? as f64,
        ))
    };
    for primitive in primitives.children().unwrap_or(&[]).iter().skip(1) {
        let width = match primitive.find("width") {
            Some(width) => non_negative_um(width.get_f64(1), "primitive width")? as f64,
            None => 0.0,
        };
        let mut outline = Vec::new();
        match primitive.head() {
            Some("gr_poly") => {
                let pts = primitive
                    .find("pts")
                    .with_context(|| format!("custom pad {label} polygon has no points"))?;
                for point in pts.children().unwrap_or(&[]).iter().skip(1) {
                    if point.head() != Some("xy") {
                        bail!("custom pad {label} polygon has a curved edge");
                    }
                    outline.push((
                        finite_um(point.get_f64(1), "primitive x")? as f64,
                        -finite_um(point.get_f64(2), "primitive y")? as f64,
                    ));
                }
            }
            Some("gr_rect") => {
                let (start, end) = (xy(primitive, "start")?, xy(primitive, "end")?);
                outline.extend([start, (end.0, start.1), end, (start.0, end.1)]);
            }
            Some("gr_line") => outline.extend([xy(primitive, "start")?, xy(primitive, "end")?]),
            Some("gr_circle") => {
                let (center, end) = (xy(primitive, "center")?, xy(primitive, "end")?);
                let radius = (end.0 - center.0).hypot(end.1 - center.1);
                circle_points(&mut outline, center, radius);
            }
            other => bail!("custom pad {label} has unsupported primitive {other:?}"),
        }
        if width > 0.0 {
            for point in outline {
                circle_points(&mut points, point, width / 2.0);
            }
        } else {
            points.extend(outline);
        }
    }
    let mut hull = convex_hull(
        points
            .into_iter()
            .map(|(x, y)| (tenths(x), tenths(y)))
            .collect(),
    );
    if hull.len() < 3 {
        bail!("custom pad {label} has no area");
    }
    let first = hull[0];
    hull.push(first);
    Ok(PadGeometry::Polygon(hull))
}

/// Andrew's monotone chain; counter-clockwise, without the closing point.
fn convex_hull(mut points: Vec<(i64, i64)>) -> Vec<(i64, i64)> {
    points.sort_unstable();
    points.dedup();
    if points.len() < 3 {
        return points;
    }
    let cross = |o: (i64, i64), a: (i64, i64), b: (i64, i64)| -> i128 {
        (a.0 - o.0) as i128 * (b.1 - o.1) as i128 - (a.1 - o.1) as i128 * (b.0 - o.0) as i128
    };
    let mut hull: Vec<(i64, i64)> = Vec::with_capacity(points.len() * 2);
    for pass in 0..2 {
        let start = hull.len();
        let iter: Box<dyn Iterator<Item = &(i64, i64)>> = if pass == 0 {
            Box::new(points.iter())
        } else {
            Box::new(points.iter().rev())
        };
        for &point in iter {
            while hull.len() >= start + 2
                && cross(hull[hull.len() - 2], hull[hull.len() - 1], point) <= 0
            {
                hull.pop();
            }
            hull.push(point);
        }
        hull.pop();
    }
    hull
}

/// Diameter (um, rounded up) of the circle about the pad origin that holds
/// the pad copper.
fn enclosing_diameter_um(geometry: &PadGeometry) -> i64 {
    let reach = |x: i64, y: i64| (x as f64).hypot(y as f64);
    let tenths_radius = match geometry {
        PadGeometry::Circle { diameter, x, y } => reach(*x, *y) + *diameter as f64 / 2.0,
        PadGeometry::Rect { x1, y1, x2, y2 } => [(*x1, *y1), (*x1, *y2), (*x2, *y1), (*x2, *y2)]
            .into_iter()
            .map(|(x, y)| reach(x, y))
            .fold(0.0, f64::max),
        PadGeometry::Polygon(ring) => ring.iter().map(|&(x, y)| reach(x, y)).fold(0.0, f64::max),
        PadGeometry::Path { width, coords } => {
            coords.iter().map(|&(x, y)| reach(x, y)).fold(0.0, f64::max) + *width as f64 / 2.0
        }
    };
    (2.0 * tenths_radius / DSN_RESOLUTION as f64).ceil() as i64
}

/// The courtyard fields each net is in: a net with a pad on the rule's
/// footprint is in that footprint's field. Items of other nets inside the
/// courtyard are not seen; that is the approximation a net-based DSN rule
/// allows, and KiCad DRC remains the authority for it.
fn courtyard_fields(
    footprints: &[FootprintModel],
    rules: &[CourtyardClearance],
) -> (NetFields, Vec<CourtyardClearanceReport>) {
    let mut net_fields = NetFields::new();
    let mut reports = Vec::new();
    for rule in rules {
        let footprint = footprints
            .iter()
            .find(|footprint| footprint.reference == rule.reference);
        for net in footprint
            .into_iter()
            .flat_map(|footprint| &footprint.pads)
            .filter_map(|pad| pad.net.as_ref())
        {
            let fields = net_fields.entry(net.clone()).or_default();
            let field = (rule.reference.clone(), rule.clearance_um);
            if !fields.contains(&field) {
                fields.push(field);
            }
        }
        reports.push(CourtyardClearanceReport {
            rule: rule.clone(),
            raised_net_count: 0,
            footprint_found: footprint.is_some(),
        });
    }
    for fields in net_fields.values_mut() {
        fields.sort();
    }
    (net_fields, reports)
}

fn normalize_rules(
    net_pins: &BTreeMap<String, Vec<String>>,
    effective_rules: &IpcEffectiveRoutingRules,
    net_fields: &NetFields,
) -> Result<(RuleGroups, NetClassNames)> {
    let mut class_nets: BTreeMap<RuleKey, Vec<String>> = BTreeMap::new();
    for net in net_pins.keys() {
        let rules = effective_rules.get(net).with_context(|| {
            format!("KiCad returned no effective routing rules for net '{net}'")
        })?;
        let clearance_um = non_negative_um(rules.clearance_mm, "clearance")?;
        let key = RuleKey {
            track_width_um: positive_um(rules.track_width_mm, "track width")?,
            clearance_um,
            via_diameter_um: positive_um(rules.via_diameter_mm, "via diameter")?,
            via_drill_um: positive_um(rules.via_drill_mm, "via drill")?,
            // A field at or below the net's own clearance changes nothing.
            fields: net_fields
                .get(net)
                .into_iter()
                .flatten()
                .filter(|(_, field_um)| *field_um > clearance_um)
                .cloned()
                .collect(),
        };
        if key.via_drill_um >= key.via_diameter_um {
            bail!(
                "net '{net}' has via drill {} um not smaller than diameter {} um",
                key.via_drill_um,
                key.via_diameter_um
            );
        }
        class_nets.entry(key).or_default().push(net.clone());
    }
    for nets in class_nets.values_mut() {
        nets.sort();
    }
    let mut net_classes = BTreeMap::new();
    for (index, nets) in class_nets.values().enumerate() {
        let name = format!("konnect_class_{:04}", index + 1);
        for net in nets {
            net_classes.insert(net.clone(), name.clone());
        }
    }
    Ok((class_nets, net_classes))
}

/// Freerouting's built-in clearance class, for no-net items and keepouts.
const FREEROUTING_DEFAULT_CLASS: &str = "default";

/// `class_class` clearances for every pair of classes where one is in a
/// courtyard field: Specctra's targeted class-to-class rule. A field class
/// carries the raised clearance itself (so it also holds within the class),
/// and Freerouting widens a class's clearance against every other class
/// too; each pair is therefore set to exactly what it needs, which lowers a
/// field class back to the ordinary clearance against classes outside its
/// field and keeps the raise inside it. Names follow the classes' order.
///
/// That widening also reaches Freerouting's built-in `default` class, which
/// no-net pads and keepouts use, so each field class is set back to its own
/// net-class clearance against `default` too. Freerouting resolves that
/// name to its default class (`KiCadNetClassNames.resolveNetClass`); a
/// release that did not would only warn and keep the wider value.
fn class_pair_rules(class_nets: &RuleGroups) -> Vec<(String, String, i64)> {
    let classes = class_nets.keys().enumerate().collect::<Vec<_>>();
    let mut pairs = Vec::new();
    for (position, (first_index, first)) in classes.iter().enumerate() {
        if !first.fields.is_empty() {
            pairs.push((
                format!("konnect_class_{:04}", first_index + 1),
                FREEROUTING_DEFAULT_CLASS.to_string(),
                first.clearance_um,
            ));
        }
        for (second_index, second) in &classes[position + 1..] {
            if first.fields.is_empty() && second.fields.is_empty() {
                continue;
            }
            pairs.push((
                format!("konnect_class_{:04}", first_index + 1),
                format!("konnect_class_{:04}", second_index + 1),
                first.pair_clearance_um(second),
            ));
        }
    }
    pairs
}

/// Insert the `class_class` scopes at the end of the DSN's network scope.
/// Topola's model has no class_class, so they are added to the text it
/// wrote and validated, and the result is checked structurally.
fn add_class_pair_rules(dsn: String, pairs: &[(String, String, i64)]) -> Result<String> {
    if pairs.is_empty() {
        return Ok(dsn);
    }
    let start = dsn
        .find("(network")
        .context("generated DSN has no network scope")?;
    let end = matching_close(&dsn, start).context("generated DSN network scope is not closed")?;
    let mut scopes = String::new();
    for (first, second, clearance_um) in pairs {
        scopes.push_str(&format!(
            "    (class_class (classes {first} {second}) (rule (clearance {clearance_um})))\n"
        ));
    }
    let before = dsn[..end].trim_end_matches([' ', '\t']);
    let separator = if before.ends_with('\n') { "" } else { "\n" };
    let text = format!("{before}{separator}{scopes}  {}", &dsn[end..]);

    let tree = parse_dsn_sexp(&text).context("parse DSN with class_class rules")?;
    let written = tree
        .find("network")
        .map_or(0, |network| network.find_all("class_class").len());
    if written != pairs.len() {
        bail!(
            "DSN holds {written} class_class rules after insertion, expected {}",
            pairs.len()
        );
    }
    Ok(text)
}

/// The byte index of the `)` closing the list that opens at `open`.
/// Specctra quotes with `"` and has no escapes inside a quoted token.
fn matching_close(text: &str, open: usize) -> Option<usize> {
    let mut depth = 0usize;
    let mut quoted = false;
    for (index, byte) in text.bytes().enumerate().skip(open) {
        match byte {
            b'"' => quoted = !quoted,
            b'(' if !quoted => depth += 1,
            b')' if !quoted => {
                depth = depth.checked_sub(1)?;
                if depth == 0 {
                    return Some(index);
                }
            }
            _ => {}
        }
    }
    None
}

/// The custom rules in a `.kicad_dru` that a DSN can carry as courtyard
/// fields: no layer clause, the condition exactly
/// `A.intersectsCourtyard('REF') && B.intersectsCourtyard('REF')` (either
/// order, either quote, one literal reference), a minimum clearance and no
/// `(severity ignore)`. Every other rule is left to KiCad DRC, as before.
/// Precedence between rules is not evaluated: a later rule that lowers the
/// clearance for some of the same items is not seen, which only makes the
/// router stricter than KiCad.
pub(crate) fn courtyard_clearance_rules(dru: &str) -> Result<Vec<CourtyardClearance>> {
    // KiCad allows `#` comment lines, which the S-expression parser does not.
    let text = dru
        .lines()
        .filter(|line| !line.trim_start().starts_with('#'))
        .collect::<Vec<_>>()
        .join("\n");
    let tree = parse_sexp(&format!("(kicad_dru\n{text}\n)")).context("parse .kicad_dru rules")?;
    let mut rules = Vec::new();
    for rule in tree.find_all("rule") {
        // A rule KiCad DRC ignores asks nothing of the router either.
        let ignored = rule
            .find("severity")
            .and_then(|severity| severity.get(1))
            .and_then(SexpNode::as_str)
            == Some("ignore");
        if ignored || rule.find("layer").is_some() {
            continue;
        }
        let Some(reference) = rule
            .find("condition")
            .and_then(|condition| condition.get(1))
            .and_then(SexpNode::as_str)
            .and_then(courtyard_pair_reference)
        else {
            continue;
        };
        let clearance = rule
            .find_all("constraint")
            .into_iter()
            .filter(|constraint| constraint.get(1).and_then(SexpNode::as_str) == Some("clearance"))
            .filter_map(|constraint| constraint.find("min"))
            .filter_map(|min| min.get(1).and_then(SexpNode::as_str))
            .filter_map(dru_length_um)
            .max();
        let Some(clearance_um) = clearance else {
            continue;
        };
        rules.push(CourtyardClearance {
            rule_name: rule
                .get(1)
                .and_then(SexpNode::as_str)
                .unwrap_or_default()
                .to_string(),
            reference,
            clearance_um,
        });
    }
    Ok(rules)
}

/// REF when `condition` is `A.intersectsCourtyard('REF') && B.intersectsCourtyard('REF')`.
fn courtyard_pair_reference(condition: &str) -> Option<String> {
    let mut compact = condition
        .chars()
        .filter(|c| !c.is_whitespace())
        .collect::<String>();
    while compact.starts_with('(')
        && compact.ends_with(')')
        && matching_close(&compact, 0) == Some(compact.len() - 1)
    {
        compact = compact[1..compact.len() - 1].to_string();
    }
    let parts = compact.split("&&").collect::<Vec<_>>();
    let [first, second] = parts.as_slice() else {
        return None;
    };
    let side = |part: &str| -> Option<(char, String)> {
        let part = part.trim_start_matches('(').trim_end_matches(')');
        let (item, call) = part.split_once(".intersectsCourtyard(")?;
        let item = match item {
            "A" => 'A',
            "B" => 'B',
            _ => return None,
        };
        let quote = call.chars().next().filter(|c| *c == '\'' || *c == '"')?;
        let reference = call.strip_prefix(quote)?.strip_suffix(quote)?;
        // Wildcards name several footprints; a literal names one.
        if reference.is_empty() || reference.contains(['*', '?', quote]) {
            return None;
        }
        Some((item, reference.to_string()))
    };
    let (first_item, first_reference) = side(first)?;
    let (second_item, second_reference) = side(second)?;
    (first_item != second_item && first_reference == second_reference).then_some(first_reference)
}

/// A DRU length such as `0.15mm`, `6mil` or `0.006in`, in micrometres.
fn dru_length_um(value: &str) -> Option<i64> {
    let (number, um_per_unit) = [
        ("mm", 1_000.0),
        ("mil", 25.4),
        ("in", 25_400.0),
        ("um", 1.0),
    ]
    .into_iter()
    .find_map(|(unit, scale)| value.strip_suffix(unit).map(|number| (number, scale)))?;
    let um = number.trim().parse::<f64>().ok()? * um_per_unit;
    (um.is_finite() && um >= 0.0).then(|| um.round() as i64)
}

#[allow(clippy::too_many_arguments)]
fn build_pcb(
    board_path: &Path,
    copper_layers: &[CopperLayer],
    outline: &[(i64, i64)],
    footprints: &[FootprintModel],
    net_pins: &BTreeMap<String, Vec<String>>,
    class_nets: &BTreeMap<RuleKey, Vec<String>>,
    net_classes: &BTreeMap<String, String>,
    padstack_names: &BTreeMap<PadstackKey, String>,
    via_names: &BTreeMap<RuleKey, String>,
    fixed_via_names: &BTreeMap<(i64, i64), String>,
    locked_routing: &LockedRouting,
    planes: &[PlaneModel],
) -> Result<dsn::Pcb> {
    let default_rule = class_nets
        .keys()
        .next()
        .context("routing model has no rule class")?;
    let layer_names = copper_layers
        .iter()
        .map(|layer| layer.name.clone())
        .collect::<Vec<_>>();
    let layers = copper_layers
        .iter()
        .enumerate()
        .map(|(index, layer)| dsn::Layer {
            name: layer.name.clone(),
            r#type: layer.dsn_type.clone(),
            property: Some(dsn::Property { index }),
        })
        .collect();
    let point = |(x, y): (i64, i64)| dsn::Point {
        x: x as f64,
        y: y as f64,
    };
    let fine = |value: i64| value as f64 / DSN_RESOLUTION as f64;
    let fine_point = |(x, y): (i64, i64)| dsn::Point {
        x: fine(x),
        y: fine(y),
    };
    let boundary = dsn::Boundary::Path(dsn::Path {
        layer: "pcb".to_string(),
        width: 0.0,
        coords: outline.iter().copied().map(point).collect(),
    });

    let components = footprints
        .iter()
        .map(|footprint| dsn::Component {
            name: footprint.image_name.clone(),
            places: vec![dsn::Place {
                name: footprint.reference.clone(),
                x: footprint.x_um as f64,
                y: footprint.y_um as f64,
                side: "front".to_string(),
                rotation: 0.0,
                PN: None,
            }],
        })
        .collect();
    let mut keepout_index = 0;
    let images = footprints
        .iter()
        .map(|footprint| dsn::Image {
            name: footprint.image_name.clone(),
            outlines: Vec::new(),
            pins: footprint
                .pads
                .iter()
                .map(|pad| dsn::Pin {
                    name: padstack_names[&pad.padstack].clone(),
                    rotate: (pad.rotation_degrees != 0.0).then_some(pad.rotation_degrees),
                    id: pad.pin_id.clone(),
                    x: pad.x_um as f64,
                    y: pad.y_um as f64,
                })
                .collect(),
            keepouts: dsn::Keepouts(
                footprint
                    .keepouts
                    .iter()
                    .flat_map(|keepout| keepout.layers.iter().map(move |layer| (keepout, layer)))
                    .map(|(keepout, layer)| {
                        keepout_index += 1;
                        dsn::Keepout {
                            kind: dsn::KeepoutKind::Normal,
                            id_: format!("konnect_keepout_{keepout_index:05}"),
                            sequence_number: None,
                            shape: dsn::Shape::Circle(dsn::Circle {
                                layer: layer.clone(),
                                diameter: keepout.diameter_um as f64,
                                offset: Some(point((keepout.x_um, keepout.y_um))),
                            }),
                            rule: None,
                        }
                    })
                    .collect(),
            ),
        })
        .collect();

    let via_padstack = |name: &str, diameter_um: i64| dsn::Padstack {
        name: name.to_string(),
        shapes: layer_names
            .iter()
            .map(|layer| {
                dsn::Shape::Circle(dsn::Circle {
                    layer: layer.clone(),
                    diameter: diameter_um as f64,
                    offset: None,
                })
            })
            .collect(),
        attach: Some(false),
    };
    let mut padstacks = padstack_names
        .iter()
        .map(|(key, name)| padstack(name, key, fine_point))
        .collect::<Vec<_>>();
    padstacks.extend(
        via_names
            .iter()
            .map(|(rule, name)| via_padstack(name, rule.via_diameter_um)),
    );
    padstacks.extend(
        fixed_via_names
            .iter()
            .map(|((diameter, _), name)| via_padstack(name, *diameter)),
    );
    padstacks.sort_by(|left, right| left.name.cmp(&right.name));

    let nets = net_pins
        .iter()
        .map(|(name, pins)| dsn::NetPinAssignments {
            name: name.clone(),
            pins: Some(dsn::Pins {
                names: pins.clone(),
            }),
        })
        .collect();
    let classes = class_nets
        .iter()
        .enumerate()
        .map(|(index, (rule, nets))| dsn::Class {
            name: format!("konnect_class_{:04}", index + 1),
            nets: nets.clone(),
            circuit: dsn::Circuit {
                use_via: via_names[&rule.via_key()].clone(),
            },
            rule: dsn::Rule {
                width: rule.track_width_um as f32,
                clearances: vec![dsn::Clearance {
                    value: rule.dsn_clearance_um() as f32,
                    r#type: None,
                }],
            },
        })
        .collect();
    debug_assert_eq!(net_classes.len(), net_pins.len());

    let net_rules = class_nets
        .iter()
        .flat_map(|(rule, nets)| nets.iter().map(move |net| (net, rule)))
        .collect::<BTreeMap<_, _>>();
    let wires = locked_routing
        .tracks
        .iter()
        .map(|track| {
            if !net_rules.contains_key(&track.net) {
                bail!("locked track uses unknown routing net '{}'", track.net);
            }
            Ok(dsn::Wire {
                path: dsn::Path {
                    layer: track.layer.clone(),
                    width: track.width_um as f64,
                    coords: vec![
                        point((track.start_x_um, track.start_y_um)),
                        point((track.end_x_um, track.end_y_um)),
                    ],
                },
                net: track.net.clone(),
                r#type: "fix".to_string(),
            })
        })
        .collect::<Result<Vec<_>>>()?;
    let fixed_vias = locked_routing
        .vias
        .iter()
        .map(|via| {
            let rule = net_rules
                .get(&via.net)
                .with_context(|| format!("locked via uses unknown routing net '{}'", via.net))?;
            let name =
                if via.diameter_um == rule.via_diameter_um && via.drill_um == rule.via_drill_um {
                    via_names[&rule.via_key()].clone()
                } else {
                    fixed_via_names[&(via.diameter_um, via.drill_um)].clone()
                };
            Ok(dsn::Via {
                name,
                x: via.x_um as f64,
                y: via.y_um as f64,
                net: via.net.clone(),
                r#type: Some("fix".to_string()),
            })
        })
        .collect::<Result<Vec<_>>>()?;
    let planes = planes
        .iter()
        .flat_map(|plane| {
            plane.pieces.iter().map(move |piece| dsn::Plane {
                net: plane.net.clone(),
                polygon: dsn::Polygon {
                    layer: plane.layer.clone(),
                    width: 0.0,
                    coords: piece.iter().copied().map(point).collect(),
                },
            })
        })
        .collect();

    Ok(dsn::Pcb {
        name: board_path
            .file_name()
            .and_then(|name| name.to_str())
            .unwrap_or("board.kicad_pcb")
            .to_string(),
        parser: Some(dsn::Parser {
            string_quote: Some('"'),
            space_in_quoted_tokens: Some(true),
            host_cad: Some("Konnect".to_string()),
            host_version: Some(env!("CARGO_PKG_VERSION").to_string()),
        }),
        resolution: dsn::Resolution {
            unit: "um".to_string(),
            value: DSN_RESOLUTION,
        },
        unit: Some("um".to_string()),
        structure: dsn::Structure {
            layers,
            boundary,
            place_boundary: None,
            planes,
            keepouts: dsn::Keepouts(Vec::new()),
            via: dsn::ViaNames {
                names: via_names.values().cloned().collect(),
            },
            grids: Vec::new(),
            rules: vec![dsn::StructureRule {
                width: Some(default_rule.track_width_um as f32),
                clearances: vec![dsn::Clearance {
                    value: default_rule.clearance_um as f32,
                    r#type: None,
                }],
            }],
        },
        placement: dsn::Placement { components },
        library: dsn::Library { images, padstacks },
        network: dsn::Network { nets, classes },
        wiring: dsn::Wiring {
            wires,
            vias: fixed_vias,
        },
    })
}

fn padstack(
    name: &str,
    key: &PadstackKey,
    fine_point: impl Fn((i64, i64)) -> dsn::Point,
) -> dsn::Padstack {
    let fine = |value: i64| value as f64 / DSN_RESOLUTION as f64;
    let shapes = key
        .layers
        .iter()
        .map(|layer| match &key.geometry {
            PadGeometry::Circle { diameter, x, y } => dsn::Shape::Circle(dsn::Circle {
                layer: layer.clone(),
                diameter: fine(*diameter),
                offset: (*x != 0 || *y != 0).then(|| fine_point((*x, *y))),
            }),
            PadGeometry::Rect { x1, y1, x2, y2 } => dsn::Shape::Rect(dsn::Rect {
                layer: layer.clone(),
                x1: fine(*x1),
                y1: fine(*y1),
                x2: fine(*x2),
                y2: fine(*y2),
            }),
            PadGeometry::Polygon(ring) => dsn::Shape::Polygon(dsn::Polygon {
                layer: layer.clone(),
                width: 0.0,
                coords: ring.iter().copied().map(&fine_point).collect(),
            }),
            PadGeometry::Path { width, coords } => dsn::Shape::Path(dsn::Path {
                layer: layer.clone(),
                width: fine(*width),
                coords: coords.iter().copied().map(&fine_point).collect(),
            }),
        })
        .collect();
    dsn::Padstack {
        name: name.to_string(),
        shapes,
        attach: Some(false),
    }
}

fn serialize_and_validate(pcb: dsn::Pcb) -> Result<String> {
    let file = dsn::DsnFile { pcb };
    let mut bytes = Vec::new();
    {
        let mut writer = ListWriter::new(&mut bytes);
        file.write_dsn(&mut writer)
            .context("serialize Specctra DSN")?;
    }
    let mut text = String::from_utf8(bytes).context("Specctra writer emitted non-UTF-8")?;
    if text.starts_with('\n') {
        text.remove(0);
    }
    text.push('\n');

    let cursor = Cursor::new(text.as_bytes());
    let mut tokenizer = ListTokenizer::new(BufReader::new(cursor));
    dsn::DsnFile::read_dsn(&mut tokenizer)
        .map_err(|error| anyhow::anyhow!("generated DSN failed parser round-trip: {error}"))?;
    Ok(text)
}

#[allow(clippy::too_many_arguments)]
fn build_manifest(
    board_path: &Path,
    source_sha256: &str,
    copper_layers: &[CopperLayer],
    footprints: &[FootprintModel],
    net_pins: &BTreeMap<String, Vec<String>>,
    net_classes: &BTreeMap<String, String>,
    padstack_names: &BTreeMap<PadstackKey, String>,
    via_names: &BTreeMap<RuleKey, String>,
    fixed_via_names: &BTreeMap<(i64, i64), String>,
    locked_routing: &LockedRouting,
    planes: &[PlaneModel],
    options: &ExportOptions,
) -> Result<String> {
    let layer_names = copper_layers
        .iter()
        .map(|layer| layer.name.clone())
        .collect::<Vec<_>>();
    let components = footprints
        .iter()
        .map(|footprint| ManifestComponent {
            reference: footprint.reference.clone(),
            kiid: footprint.kiid.clone(),
            image_name: footprint.image_name.clone(),
            x_um: footprint.x_um,
            y_um: footprint.y_um,
            rotation_degrees: 0.0,
            side: "front".to_string(),
            kicad_side: footprint.side.clone(),
            kicad_rotation_degrees: footprint.rotation_degrees,
            pads: footprint
                .pads
                .iter()
                .map(|pad| ManifestPin {
                    pad_number: pad.number.clone(),
                    dsn_pin: format!("{}-{}", footprint.reference, pad.pin_id),
                    net: pad.net.clone(),
                    padstack_name: padstack_names[&pad.padstack].clone(),
                })
                .collect(),
        })
        .collect();
    let nets = net_pins
        .iter()
        .map(|(name, pins)| ManifestNet {
            name: name.clone(),
            pins: pins.clone(),
            class_name: net_classes[name].clone(),
        })
        .collect();
    let mut padstacks = padstack_names
        .iter()
        .map(|(key, name)| ManifestPadstack {
            name: name.clone(),
            purpose: "pad".to_string(),
            shape: key.shape,
            layers: key.layers.clone(),
            size_x_um: key.size_x_um,
            size_y_um: key.size_y_um,
            drill_um: key.drill_um,
        })
        .collect::<Vec<_>>();
    let via = |name: &String, diameter_um: i64, drill_um: i64| ManifestPadstack {
        name: name.clone(),
        purpose: "via".to_string(),
        shape: PadShape::Circle,
        layers: layer_names.clone(),
        size_x_um: diameter_um,
        size_y_um: diameter_um,
        drill_um: Some(drill_um),
    };
    padstacks.extend(
        via_names
            .iter()
            .map(|(rule, name)| via(name, rule.via_diameter_um, rule.via_drill_um)),
    );
    padstacks.extend(
        fixed_via_names
            .iter()
            .map(|((diameter, drill), name)| via(name, *diameter, *drill)),
    );
    padstacks.sort_by(|left, right| left.name.cmp(&right.name));

    let sides = footprints
        .iter()
        .map(|footprint| footprint.side.as_str())
        .collect::<BTreeSet<_>>();
    let component_side = match (sides.contains("front"), sides.contains("back")) {
        (true, true) => "both",
        (false, true) => "back",
        _ => "front",
    };
    let pad_shapes = padstack_names
        .keys()
        .map(|key| key.shape.label().to_string())
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect();

    serde_json::to_string_pretty(&Manifest {
        schema_version: MANIFEST_SCHEMA,
        board_path: board_path.display().to_string(),
        source_sha256: source_sha256.to_string(),
        coordinate_unit: "um".to_string(),
        resolution: DSN_RESOLUTION as u32,
        supported_profile: SupportedProfile {
            copper_layers: copper_layers.len() as u32,
            component_side: component_side.to_string(),
            pad_shapes,
            existing_routing: !locked_routing.tracks.is_empty() || !locked_routing.vias.is_empty(),
            locked_track_count: locked_routing.tracks.len(),
            locked_via_count: locked_routing.vias.len(),
            copper_zones: !planes.is_empty(),
            custom_rules: options.custom_rules_sha256.is_some(),
            outline: "one closed loop of straight Edge.Cuts lines".to_string(),
            image_frame: "board_oriented".to_string(),
            plane_count: planes.len(),
            custom_rules_sha256: options.custom_rules_sha256.clone(),
        },
        layers: copper_layers
            .iter()
            .enumerate()
            .map(|(index, layer)| ManifestLayer {
                kicad_name: layer.name.clone(),
                dsn_name: layer.name.clone(),
                index,
                dsn_type: layer.dsn_type.clone(),
            })
            .collect(),
        components,
        nets,
        padstacks,
    })
    .context("serialize routing manifest")
}

fn top_level_net_table(tree: &SexpNode) -> BTreeMap<String, String> {
    tree.find_all("net")
        .into_iter()
        .filter_map(|net| {
            let id = net.get(1)?.as_str()?;
            let name = net.get(2)?.as_str()?;
            Some((id.to_string(), name.to_string()))
        })
        .collect()
}

fn resolve_net(net: &SexpNode, table: &BTreeMap<String, String>) -> Option<String> {
    if let Some(name) = net.get(2).and_then(SexpNode::as_str) {
        return (!name.is_empty()).then(|| name.to_string());
    }
    let value = net.get(1)?.as_str()?;
    if let Some(name) = table.get(value).filter(|name| !name.is_empty()) {
        return Some(name.clone());
    }
    // KiCad 10 stores the net name directly as `(net "NAME")`. Older board
    // files used `(net id "NAME")`, handled by the branch above.
    (!value.is_empty()).then(|| value.to_string())
}

fn property_value<'a>(node: &'a SexpNode, property_name: &str) -> Option<&'a str> {
    node.find_all("property")
        .into_iter()
        .find(|property| property.get(1).and_then(SexpNode::as_str) == Some(property_name))?
        .get(2)?
        .as_str()
}

fn point_um(node: &SexpNode, tag: &str) -> Result<(i64, i64)> {
    let point = node.find(tag).with_context(|| format!("missing '{tag}'"))?;
    Ok((
        finite_um(point.get_f64(1), tag)?,
        -finite_um(point.get_f64(2), tag)?,
    ))
}

fn finite_number(value: Option<f64>, label: &str) -> Result<f64> {
    let value = value.with_context(|| format!("missing {label}"))?;
    if !value.is_finite() {
        bail!("{label} is not finite");
    }
    Ok(value)
}

fn finite_um(value: Option<f64>, label: &str) -> Result<i64> {
    let value = finite_number(value, label)? * UM_PER_MM;
    if value < i64::MIN as f64 || value > i64::MAX as f64 {
        bail!("{label} is outside the supported coordinate range");
    }
    Ok(value.round() as i64)
}

fn positive_um(value: Option<f64>, label: &str) -> Result<i64> {
    let value = finite_um(value, label)?;
    if value <= 0 {
        bail!("{label} must be greater than zero");
    }
    Ok(value)
}

fn non_negative_um(value: Option<f64>, label: &str) -> Result<i64> {
    let value = finite_um(value, label)?;
    if value < 0 {
        bail!("{label} must not be negative");
    }
    Ok(value)
}

fn sha256_hex(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

#[cfg(test)]
mod tests {
    use super::*;
    use konnect_ipc::IpcRoutingRules;

    fn rules() -> IpcEffectiveRoutingRules {
        ["GND", "VCC"]
            .into_iter()
            .map(|net| {
                (
                    net.to_string(),
                    IpcRoutingRules {
                        class_name: "Default".to_string(),
                        constituents: vec!["Default".to_string()],
                        track_width_mm: Some(0.25),
                        clearance_mm: Some(0.2),
                        via_diameter_mm: Some(0.6),
                        via_drill_mm: Some(0.3),
                    },
                )
            })
            .collect()
    }

    fn native_fixture_rules() -> IpcEffectiveRoutingRules {
        let mut rules = rules();
        for rule in rules.values_mut() {
            rule.track_width_mm = Some(0.2);
        }
        rules
    }

    #[test]
    fn deterministic_export_round_trips_through_specctra_parser() {
        let source = include_str!("../tests/fixtures/specctra_two_resistors.kicad_pcb");
        let first = export_dsn(Path::new("board.kicad_pcb"), source, &rules()).unwrap();
        let second = export_dsn(Path::new("board.kicad_pcb"), source, &rules()).unwrap();

        assert_eq!(first.dsn, second.dsn);
        assert_eq!(first.manifest, second.manifest);
        assert_eq!(first.component_count, 2);
        assert_eq!(first.pad_count, 4);
        assert_eq!(first.net_count, 2);
        assert!(first.dsn.contains("(pcb board.kicad_pcb"));
        assert!(first.dsn.contains("(boundary"));
        assert!(first.dsn.contains("(net GND"));
        assert!(first.dsn.contains("R1-1"));
        let manifest: serde_json::Value = serde_json::from_str(&first.manifest).unwrap();
        assert_eq!(manifest["components"][0]["reference"], "R1");
        assert_eq!(manifest["components"][0]["x_um"], 100_000);
        assert_eq!(manifest["components"][0]["y_um"], -50_000);
        assert_eq!(manifest["components"][0]["rotation_degrees"], 0.0);
        assert_eq!(manifest["components"][0]["side"], "front");
    }

    #[test]
    fn freerouting_owned_dsn_corpus_parses() {
        let source = include_str!("../tests/fixtures/freerouting_issue269_minimal_v2_3_0.dsn");
        let mut tokenizer = ListTokenizer::new(BufReader::new(Cursor::new(source.as_bytes())));
        dsn::DsnFile::read_dsn(&mut tokenizer)
            .expect("Freerouting v2.3.0 corpus fixture must remain parseable");
    }

    #[test]
    fn native_kicad_dsn_rewrites_manifest_identifiers_without_changing_semantics() {
        let source = include_str!("../tests/fixtures/specctra_two_resistors.kicad_pcb");
        let native = include_str!("../tests/fixtures/specctra_two_resistors.native-kicad-10.dsn");
        let baseline = export_dsn(
            Path::new("board.kicad_pcb"),
            source,
            &native_fixture_rules(),
        )
        .unwrap();
        let adopted = adopt_native_dsn(baseline, native.to_string()).unwrap();
        let manifest: serde_json::Value = serde_json::from_str(&adopted.manifest).unwrap();

        assert_eq!(adopted.dsn, native);
        assert_eq!(
            manifest["components"][0]["image_name"],
            "Resistor_SMD:R_0402"
        );
        assert_eq!(
            manifest["components"][0]["pads"][0]["padstack_name"],
            "Rect[T]Pad_600.000000x500.000000_um"
        );
        assert_eq!(manifest["nets"][0]["class_name"], "kicad_default");
        assert!(manifest["padstacks"]
            .as_array()
            .unwrap()
            .iter()
            .any(|padstack| padstack["name"] == "Via[0-1]_600:300_um"));
    }

    /// Optional local parity check against the Freerouting engine. CI does not
    /// install Java or Freerouting; maintainers can opt in with
    /// `FREEROUTING_JAR=/path/to/freerouting.jar cargo test -p konnect-core
    /// freerouting_accepts_exported_fixture -- --ignored`.
    #[test]
    #[ignore = "requires Java and FREEROUTING_JAR"]
    fn freerouting_accepts_exported_fixture() {
        let jar = std::env::var_os("FREEROUTING_JAR").expect("set FREEROUTING_JAR");
        let source = include_str!("../tests/fixtures/specctra_two_resistors.kicad_pcb");
        let temp = tempfile::tempdir().expect("tempdir");
        let board_path = temp.path().join("board.kicad_pcb");
        let dsn_path = temp.path().join("board.dsn");
        let ses_path = temp.path().join("board.ses");
        std::fs::write(&board_path, source).expect("write board fixture");
        let export = export_dsn(&board_path, source, &rules()).unwrap();
        std::fs::write(&dsn_path, export.dsn).expect("write DSN");

        let output = std::process::Command::new("java")
            .arg("-jar")
            .arg(jar)
            .arg("-de")
            .arg(&dsn_path)
            .arg("-do")
            .arg(&ses_path)
            .arg("-mp")
            .arg("2")
            .output()
            .expect("launch Freerouting");

        assert!(
            output.status.success(),
            "Freerouting refused generated DSN:\nstdout:\n{}\nstderr:\n{}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
        assert!(
            std::fs::metadata(&ses_path)
                .map(|metadata| metadata.len() > 0)
                .unwrap_or(false),
            "Freerouting did not produce a non-empty SES"
        );
        let ses = std::fs::read_to_string(&ses_path).expect("read Freerouting SES");
        crate::specctra_ses::parse_import_plan(&board_path, source, &export.manifest, &ses)
            .expect("Freerouting SES must pass Konnect's strict import planner");
    }

    /// Real-engine parity for KiCad 10's native exporter identifiers plus the
    /// rewritten revision manifest consumed by the strict SES planner.
    #[test]
    #[ignore = "requires Java and FREEROUTING_JAR"]
    fn freerouting_round_trips_native_kicad_export() {
        let jar = std::env::var_os("FREEROUTING_JAR").expect("set FREEROUTING_JAR");
        let source = include_str!("../tests/fixtures/specctra_two_resistors.kicad_pcb");
        let native = include_str!("../tests/fixtures/specctra_two_resistors.native-kicad-10.dsn");
        let temp = tempfile::tempdir().expect("tempdir");
        let board_path = temp.path().join("board.kicad_pcb");
        let dsn_path = temp.path().join("board.dsn");
        let ses_path = temp.path().join("board.ses");
        std::fs::write(&board_path, source).expect("write board fixture");
        let baseline = export_dsn(&board_path, source, &native_fixture_rules()).unwrap();
        let export = adopt_native_dsn(baseline, native.to_string()).unwrap();
        std::fs::write(&dsn_path, &export.dsn).expect("write native DSN");

        let output = std::process::Command::new("java")
            .arg("-jar")
            .arg(jar)
            .arg("-de")
            .arg(&dsn_path)
            .arg("-do")
            .arg(&ses_path)
            .arg("-mp")
            .arg("2")
            .output()
            .expect("run Freerouting");
        assert!(
            output.status.success(),
            "Freerouting failed:\nstdout:\n{}\nstderr:\n{}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
        let ses = std::fs::read_to_string(&ses_path).expect("read SES");
        let plan =
            crate::specctra_ses::parse_import_plan(&board_path, source, &export.manifest, &ses)
                .expect("strict SES planner accepts native KiCad identifiers");
        assert!(!plan.tracks.is_empty() || !plan.vias.is_empty());
    }

    #[test]
    fn incomplete_effective_rules_are_refused() {
        let source = include_str!("../tests/fixtures/specctra_two_resistors.kicad_pcb");
        let mut rules = rules();
        rules.get_mut("GND").unwrap().via_drill_mm = None;
        let error = export_dsn(Path::new("board.kicad_pcb"), source, &rules)
            .unwrap_err()
            .to_string();
        assert!(error.contains("via drill"), "{error}");
    }

    #[test]
    fn unlocked_routing_is_refused_before_export() {
        let source = include_str!("../tests/fixtures/specctra_two_resistors.kicad_pcb").replace(
            "\n)",
            "\n  (segment (start 1 1) (end 2 2) (width 0.2) (layer \"F.Cu\") (net 1))\n)",
        );
        let error = export_dsn(Path::new("board.kicad_pcb"), &source, &rules())
            .unwrap_err()
            .to_string();
        assert!(error.contains("existing track segment"), "{error}");
    }

    #[test]
    fn locked_tracks_and_vias_are_exported_as_fixed_wiring() {
        let source = include_str!("../tests/fixtures/specctra_two_resistors_locked.kicad_pcb");
        let export = export_dsn(Path::new("board.kicad_pcb"), source, &rules()).unwrap();
        assert!(export.dsn.contains("(type fix)"));
        assert!(export.dsn.contains("(path F.Cu 250"));
        assert!(export.dsn.contains("(via konnect_via_"));
        let manifest: serde_json::Value = serde_json::from_str(&export.manifest).unwrap();
        assert_eq!(manifest["supported_profile"]["existing_routing"], true);
        assert_eq!(manifest["supported_profile"]["locked_track_count"], 1);
        assert_eq!(manifest["supported_profile"]["locked_via_count"], 1);
    }

    #[test]
    fn freerouting_session_preserves_locked_routing_outside_the_import_plan() {
        let source = include_str!("../tests/fixtures/specctra_two_resistors_locked.kicad_pcb");
        let native =
            include_str!("../tests/fixtures/specctra_two_resistors_locked.native-kicad-10.dsn");
        let ses =
            include_str!("../tests/fixtures/specctra_two_resistors_locked.freerouting-2.3.0.ses");
        let temp = tempfile::tempdir().unwrap();
        let board_path = temp.path().join("specctra_two_resistors_locked.kicad_pcb");
        std::fs::write(&board_path, source).unwrap();
        let baseline = export_dsn(&board_path, source, &native_fixture_rules()).unwrap();
        let adopted = adopt_native_dsn(baseline, native.to_string()).unwrap();
        let plan =
            crate::specctra_ses::parse_import_plan(&board_path, source, &adopted.manifest, ses)
                .unwrap();
        assert_eq!(plan.locked_track_count, 1);
        assert_eq!(plan.locked_via_count, 1);
        assert!(!plan.tracks.is_empty() || !plan.vias.is_empty());
    }

    #[test]
    fn locked_arcs_are_refused_instead_of_approximated() {
        let source = include_str!("../tests/fixtures/specctra_two_resistors_locked_arc.kicad_pcb");
        let error = export_dsn(Path::new("board.kicad_pcb"), source, &rules())
            .unwrap_err()
            .to_string();
        assert!(error.contains("locked routed arc"), "{error}");
        assert!(error.contains("without approximation"), "{error}");
    }

    #[test]
    fn branched_outline_is_refused() {
        let source = include_str!("../tests/fixtures/specctra_two_resistors.kicad_pcb")
            .replace(
                "\n)",
                "\n  (gr_line (start 80 30) (end 90 40) (stroke (width 0.05) (type default)) (layer \"Edge.Cuts\") (uuid \"branch\"))\n)",
            );
        let error = export_dsn(Path::new("board.kicad_pcb"), &source, &rules())
            .unwrap_err()
            .to_string();
        assert!(error.contains("degree"), "{error}");
    }

    #[test]
    fn pad_shape_offset_moves_the_padstack_copper() {
        let source = include_str!("../tests/fixtures/specctra_two_resistors.kicad_pcb").replacen(
            "(size 0.6 0.5)",
            "(size 0.6 0.5)\n\t\t\t(offset 0.1 0)",
            1,
        );
        let export = export_dsn(Path::new("board.kicad_pcb"), &source, &rules()).unwrap();
        assert!(
            export.dsn.contains("(rect F.Cu -200 -250 400 250)"),
            "{}",
            export.dsn
        );
    }

    const MULTILAYER: &str = include_str!("../tests/fixtures/specctra_multilayer.kicad_pcb");

    fn multilayer_rules() -> IpcEffectiveRoutingRules {
        let mut rules = rules();
        let gnd = rules["GND"].clone();
        rules.remove("VCC");
        rules.insert("SIG".to_string(), gnd);
        rules
    }

    fn parsed(dsn: &str) -> dsn::Pcb {
        let mut tokenizer = ListTokenizer::new(BufReader::new(Cursor::new(dsn.as_bytes())));
        dsn::DsnFile::read_dsn(&mut tokenizer).unwrap().pcb
    }

    fn image<'a>(pcb: &'a dsn::Pcb, reference: &str) -> &'a dsn::Image {
        let name = format!("konnect_image_{reference}");
        pcb.library
            .images
            .iter()
            .find(|image| image.name == name)
            .unwrap()
    }

    fn pin<'a>(image: &'a dsn::Image, id: &str) -> &'a dsn::Pin {
        image.pins.iter().find(|pin| pin.id == id).unwrap()
    }

    fn padstack_of<'a>(pcb: &'a dsn::Pcb, name: &str) -> &'a dsn::Padstack {
        pcb.library
            .padstacks
            .iter()
            .find(|padstack| padstack.name == name)
            .unwrap()
    }

    #[test]
    fn multilayer_stack_keeps_physical_order_and_plane_types() {
        let export = export_dsn(
            Path::new("board.kicad_pcb"),
            MULTILAYER,
            &multilayer_rules(),
        )
        .unwrap();
        let pcb = parsed(&export.dsn);
        let layers = pcb
            .structure
            .layers
            .iter()
            .map(|layer| (layer.name.as_str(), layer.r#type.as_str()))
            .collect::<Vec<_>>();
        assert_eq!(
            layers,
            [
                ("F.Cu", "signal"),
                ("In1.Cu", "power"),
                ("In2.Cu", "signal"),
                ("B.Cu", "signal")
            ]
        );
        assert_eq!(export.copper_layer_count, 4);
    }

    #[test]
    fn rotated_and_back_side_pins_carry_their_board_offsets() {
        let export = export_dsn(
            Path::new("board.kicad_pcb"),
            MULTILAYER,
            &multilayer_rules(),
        )
        .unwrap();
        let pcb = parsed(&export.dsn);
        for component in &pcb.placement.components {
            let place = &component.places[0];
            assert_eq!(
                (place.side.as_str(), place.rotation),
                ("front", 0.0),
                "{}",
                place.name
            );
        }
        // U1 at (10, 10) rotated 90: pad 1 at local (-1, 0) lands at (10, 11).
        let u1 = pin(image(&pcb, "U1"), "1");
        assert_eq!((u1.x, u1.y, u1.rotate), (0.0, -1000.0, Some(90.0)));
        // C1 on the back at (20, 10) rotated -90: pad 1 at local (-0.5, 0)
        // lands at (20, 9.5), on B.Cu only.
        let c1 = pin(image(&pcb, "C1"), "1");
        assert_eq!((c1.x, c1.y, c1.rotate), (0.0, 500.0, Some(270.0)));
        let layers = padstack_of(&pcb, &c1.name)
            .shapes
            .iter()
            .map(|shape| match shape {
                dsn::Shape::Rect(rect) => rect.layer.clone(),
                other => panic!("{other:?}"),
            })
            .collect::<Vec<_>>();
        assert_eq!(layers, ["B.Cu"]);
        assert_eq!(export.back_component_count, 1);
        let manifest: serde_json::Value = serde_json::from_str(&export.manifest).unwrap();
        let c1 = manifest["components"]
            .as_array()
            .unwrap()
            .iter()
            .find(|component| component["reference"] == "C1")
            .unwrap();
        assert_eq!(c1["kicad_side"], "back");
        assert_eq!(c1["kicad_rotation_degrees"], -90.0);
        assert_eq!(c1["rotation_degrees"], 0.0);
    }

    #[test]
    fn rounded_oval_and_custom_pads_lower_to_enclosing_shapes() {
        let export = export_dsn(
            Path::new("board.kicad_pcb"),
            MULTILAYER,
            &multilayer_rules(),
        )
        .unwrap();
        let pcb = parsed(&export.dsn);
        let roundrect = padstack_of(&pcb, &pin(image(&pcb, "U1"), "1").name);
        let dsn::Shape::Polygon(polygon) = &roundrect.shapes[0] else {
            panic!("{roundrect:?}")
        };
        assert_eq!(polygon.coords.len(), 4 * ARC_SEGMENTS_PER_QUADRANT + 1);
        let max_x = polygon.coords.iter().map(|p| p.x).fold(f64::MIN, f64::max);
        let max_y = polygon.coords.iter().map(|p| p.y).fold(f64::MIN, f64::max);
        assert_eq!(
            (max_x, max_y),
            (500.0, 250.0),
            "flat sides lie on the pad edges"
        );
        // Corner radius 125 um: the corner vertex sits just outside the arc.
        for p in polygon.coords.iter().filter(|p| p.x > 375.0 && p.y > 125.0) {
            let reach = (p.x - 375.0).hypot(p.y - 125.0);
            assert!((125.0..=128.0).contains(&reach), "{reach}");
        }

        let custom = padstack_of(&pcb, &pin(image(&pcb, "U1"), "2").name);
        let dsn::Shape::Polygon(hull) = &custom.shapes[0] else {
            panic!("{custom:?}")
        };
        assert_eq!(hull.coords.len(), 5, "the square contains the round anchor");

        let slot = padstack_of(&pcb, &pin(image(&pcb, "J1"), "1").name);
        assert_eq!(
            slot.shapes.len(),
            4,
            "through pads cover every copper layer"
        );
        let dsn::Shape::Path(path) = &slot.shapes[0] else {
            panic!("{slot:?}")
        };
        assert_eq!(path.width, 1000.0);
        assert_eq!((path.coords[0].x, path.coords[1].x), (-500.0, 500.0));
    }

    #[test]
    fn repeated_and_unnumbered_pads_get_distinct_pin_ids() {
        let export = export_dsn(
            Path::new("board.kicad_pcb"),
            MULTILAYER,
            &multilayer_rules(),
        )
        .unwrap();
        let pcb = parsed(&export.dsn);
        let j1 = image(&pcb, "J1");
        assert!(j1.pins.iter().any(|pin| pin.id == "1"));
        assert!(j1.pins.iter().any(|pin| pin.id == "1@1"));
        assert!(image(&pcb, "FID1").pins.iter().any(|pin| pin.id == "@0"));
        let gnd = pcb
            .network
            .nets
            .iter()
            .find(|net| net.name == "GND")
            .unwrap();
        let pins = &gnd.pins.as_ref().unwrap().names;
        assert!(pins.contains(&"J1-1".to_string()) && pins.contains(&"J1-1@1".to_string()));
        assert_eq!(
            export.skipped_pad_count, 1,
            "the paste-only aperture has no copper"
        );
    }

    #[test]
    fn holes_and_enlarged_clearances_become_keepouts() {
        let export = export_dsn(
            Path::new("board.kicad_pcb"),
            MULTILAYER,
            &multilayer_rules(),
        )
        .unwrap();
        let pcb = parsed(&export.dsn);
        let circles = |reference: &str| {
            image(&pcb, reference)
                .keepouts
                .0
                .iter()
                .map(|keepout| match &keepout.shape {
                    dsn::Shape::Circle(circle) => {
                        let at = circle.offset.as_ref().unwrap();
                        (circle.layer.clone(), circle.diameter, at.x, at.y)
                    }
                    other => panic!("{other:?}"),
                })
                .collect::<Vec<_>>()
        };
        let hole = circles("J1");
        assert_eq!(hole.len(), 4);
        assert!(hole
            .iter()
            .all(|(_, d, x, y)| (*d, *x, *y) == (1500.0, 0.0, -3000.0)));
        // 1 mm pad, local clearance 0.6 against the 0.2 class clearance.
        assert_eq!(circles("FID1"), [("F.Cu".to_string(), 1800.0, 0.0, 0.0)]);
    }

    #[test]
    fn power_layer_zone_with_a_hole_becomes_exact_plane_pieces() {
        let export = export_dsn(
            Path::new("board.kicad_pcb"),
            MULTILAYER,
            &multilayer_rules(),
        )
        .unwrap();
        let pcb = parsed(&export.dsn);
        assert_eq!(pcb.structure.planes.len(), 4);
        let mut area = 0.0;
        for plane in &pcb.structure.planes {
            assert_eq!(
                (plane.net.as_str(), plane.polygon.layer.as_str()),
                ("GND", "In1.Cu")
            );
            let c = &plane.polygon.coords;
            for pair in c.windows(2) {
                area += pair[0].x * pair[1].y - pair[1].x * pair[0].y;
            }
            // The hole (15..25, 5..15 mm) is outside every piece.
            let inside = |x: f64, y: f64| {
                let (xs, ys): (Vec<f64>, Vec<f64>) = c.iter().map(|p| (p.x, p.y)).unzip();
                let fold = |v: &[f64], f: fn(f64, f64) -> f64, s: f64| v.iter().copied().fold(s, f);
                x > fold(&xs, f64::min, f64::MAX)
                    && x < fold(&xs, f64::max, f64::MIN)
                    && y > fold(&ys, f64::min, f64::MAX)
                    && y < fold(&ys, f64::max, f64::MIN)
            };
            assert!(!inside(20_000.0, -10_000.0));
        }
        assert_eq!(
            (area / 2.0).abs(),
            1_100_000_000.0,
            "40x30 mm minus the 10x10 mm hole"
        );
        let manifest: serde_json::Value = serde_json::from_str(&export.manifest).unwrap();
        assert_eq!(manifest["supported_profile"]["copper_zones"], true);
        assert_eq!(manifest["supported_profile"]["plane_count"], 1);
        assert_eq!(manifest["supported_profile"]["component_side"], "both");
    }

    #[test]
    fn a_hole_on_the_outline_edge_leaves_no_zero_area_piece() {
        // A 100x100 square with a 40x30 notch-like hole sitting on its bottom edge.
        let outline = vec![(0, 0), (100, 0), (100, 100), (0, 100)];
        let hole = vec![(30, 0), (30, 30), (70, 30), (70, 0)];
        let pieces = trapezoids(&[outline, hole]);
        let mut total = 0i128;
        for piece in &pieces {
            let doubled = piece
                .windows(2)
                .map(|w| {
                    i128::from(w[0].0) * i128::from(w[1].1)
                        - i128::from(w[1].0) * i128::from(w[0].1)
                })
                .sum::<i128>()
                .abs();
            assert!(doubled > 0, "zero-area piece {piece:?}");
            total += doubled;
        }
        assert_eq!(total, 2 * (100 * 100 - 40 * 30));
    }

    #[test]
    fn trapezoids_cut_a_sloped_outline_exactly() {
        let triangle = vec![(0, 0), (100, 0), (0, 100)];
        let notch = vec![(10, 10), (30, 10), (10, 30)];
        let pieces = trapezoids(&[triangle, notch]);
        let area: i64 = pieces
            .iter()
            .map(|ring| {
                ring.windows(2)
                    .map(|pair| pair[0].0 * pair[1].1 - pair[1].0 * pair[0].1)
                    .sum::<i64>()
            })
            .sum::<i64>()
            .abs();
        assert_eq!(area, 2 * (5_000 - 200));
    }

    #[test]
    fn off_rule_locked_vias_get_their_own_fixed_padstack() {
        let export = export_dsn(
            Path::new("board.kicad_pcb"),
            MULTILAYER,
            &multilayer_rules(),
        )
        .unwrap();
        let pcb = parsed(&export.dsn);
        let via = &pcb.wiring.vias[0];
        assert_eq!(via.name, "konnect_fixed_via_0001");
        let padstack = padstack_of(&pcb, &via.name);
        assert_eq!(padstack.shapes.len(), 4);
        assert!(
            !pcb.structure.via.names.contains(&via.name),
            "the router may not place it"
        );
        assert_eq!(pcb.wiring.wires[0].path.layer, "In2.Cu");
    }

    #[test]
    fn zone_on_a_routing_layer_is_refused() {
        let source = MULTILAYER.replace(
            "(layer \"In1.Cu\")\n\t\t(uuid \"f0000000",
            "(layer \"In2.Cu\")\n\t\t(uuid \"f0000000",
        );
        let error = export_dsn(Path::new("board.kicad_pcb"), &source, &multilayer_rules())
            .unwrap_err()
            .to_string();
        assert!(error.contains("routing layer In2.Cu"), "{error}");
    }

    fn with_in1_zone(net: &str, x1: f64, y1: f64, x2: f64, y2: f64) -> String {
        let zone = format!(
            "\t(zone\n\t\t(net \"{net}\")\n\t\t(layer \"In1.Cu\")\n\t\t(uuid \"f0000000-0000-4000-8000-000000000009\")\n\t\t(name \"island\")\n\t\t(polygon\n\t\t\t(pts\n\t\t\t\t(xy {x1} {y1}) (xy {x2} {y1}) (xy {x2} {y2}) (xy {x1} {y2})\n\t\t\t)\n\t\t)\n\t)\n\t(embedded_fonts no)"
        );
        MULTILAYER.replacen("\t(embedded_fonts no)", &zone, 1)
    }

    #[test]
    fn an_island_in_a_plane_hole_is_a_split_plane() {
        // The GND plane on In1 has a hole at 15..25 x 5..15 mm.
        let source = with_in1_zone("SIG", 16.0, 6.0, 24.0, 14.0);
        let export =
            export_dsn(Path::new("board.kicad_pcb"), &source, &multilayer_rules()).unwrap();
        let pcb = parsed(&export.dsn);
        let nets: BTreeSet<_> = pcb
            .structure
            .planes
            .iter()
            .map(|p| p.net.as_str())
            .collect();
        assert_eq!(nets, BTreeSet::from(["GND", "SIG"]));
        assert_eq!(export.plane_count, 2);
    }

    #[test]
    fn overlapping_planes_on_one_layer_are_refused() {
        let source = with_in1_zone("SIG", 10.0, 10.0, 20.0, 20.0);
        let error = export_dsn(Path::new("board.kicad_pcb"), &source, &multilayer_rules())
            .unwrap_err()
            .to_string();
        assert!(error.contains("overlaps"), "{error}");
    }

    #[test]
    fn touching_pieces_do_not_overlap() {
        let left = vec![vec![(0, 0), (10, 0), (10, 10), (0, 10), (0, 0)]];
        let right = vec![vec![(10, 0), (20, 0), (20, 10), (10, 10), (10, 0)]];
        let above = vec![vec![(0, 10), (10, 10), (10, 20), (0, 20), (0, 10)]];
        assert!(!pieces_overlap(&left, &right));
        assert!(!pieces_overlap(&left, &above));
        let inside = vec![vec![(2, 2), (8, 2), (8, 8), (2, 8), (2, 2)]];
        assert!(pieces_overlap(&left, &inside));
    }

    #[test]
    fn blind_locked_via_is_refused() {
        let source = MULTILAYER.replace(
            "(layers \"F.Cu\" \"B.Cu\")\n\t\t(locked yes)",
            "(layers \"F.Cu\" \"In2.Cu\")\n\t\t(locked yes)",
        );
        let error = export_dsn(Path::new("board.kicad_pcb"), &source, &multilayer_rules())
            .unwrap_err()
            .to_string();
        assert!(error.contains("through via"), "{error}");
    }

    #[test]
    fn multilayer_session_imports_inner_layer_wires_and_vias() {
        let temp = tempfile::tempdir().unwrap();
        let board = temp.path().join("board.kicad_pcb");
        std::fs::write(&board, MULTILAYER).unwrap();
        let export = export_dsn(&board, MULTILAYER, &multilayer_rules()).unwrap();
        let pcb = parsed(&export.dsn);
        let placement = pcb
            .placement
            .components
            .iter()
            .map(|component| {
                let place = &component.places[0];
                format!(
                    "(component {} (place {} {} {} front 0))",
                    component.name,
                    place.name,
                    place.x * 10.0,
                    place.y * 10.0
                )
            })
            .collect::<String>();
        let ses = format!(
            "(session board (base_design board) (placement (resolution um 10) {placement}) (was_is) \
             (routes (resolution um 10) (library_out (padstack konnect_via_0001 \
             (shape (circle F.Cu 6000 0 0)) (shape (circle In1.Cu 6000 0 0)) \
             (shape (circle In2.Cu 6000 0 0)) (shape (circle B.Cu 6000 0 0)) (attach off))) \
             (network_out (net SIG (wire (path In2.Cu 2500 100000 -110000 200000 -110000)) \
             (via konnect_via_0001 200000 -110000)))))"
        );
        let plan =
            crate::specctra_ses::parse_import_plan(&board, MULTILAYER, &export.manifest, &ses)
                .unwrap();
        assert_eq!(plan.tracks[0].layer, "In2.Cu");
        assert_eq!((plan.vias[0].x_mm, plan.vias[0].y_mm), (20.0, 11.0));
        assert_eq!((plan.locked_track_count, plan.locked_via_count), (1, 1));
        assert_eq!(plan.skipped_point_wires, 0);

        // Freerouting sometimes writes a wire of one point; it has no copper.
        let point_wire = ses.replace(
            "(via konnect_via_0001",
            "(wire (path In2.Cu 2500 100000 -110000)) (via konnect_via_0001",
        );
        let plan = crate::specctra_ses::parse_import_plan(
            &board,
            MULTILAYER,
            &export.manifest,
            &point_wire,
        )
        .unwrap();
        assert_eq!((plan.tracks.len(), plan.skipped_point_wires), (1, 1));
        let unknown_layer = point_wire.replace(
            "(path In2.Cu 2500 100000 -110000)",
            "(path In9.Cu 2500 1 1)",
        );
        let error = crate::specctra_ses::parse_import_plan(
            &board,
            MULTILAYER,
            &export.manifest,
            &unknown_layer,
        )
        .unwrap_err()
        .to_string();
        assert!(error.contains("unknown layer"), "{error}");

        let three_layer_via = ses.replace(" (shape (circle In1.Cu 6000 0 0))", "");
        let error = crate::specctra_ses::parse_import_plan(
            &board,
            MULTILAYER,
            &export.manifest,
            &three_layer_via,
        )
        .unwrap_err()
        .to_string();
        assert!(error.contains("every expected copper layer"), "{error}");
    }

    /// Real-engine check of the multi-layer profile: Freerouting must accept
    /// the planes, inner layers, board-oriented images and fixed vias, and its
    /// session must pass the strict planner.
    #[test]
    #[ignore = "requires Java and FREEROUTING_JAR"]
    fn freerouting_routes_the_multilayer_fixture() {
        let jar = std::env::var_os("FREEROUTING_JAR").expect("set FREEROUTING_JAR");
        let temp = tempfile::tempdir().expect("tempdir");
        let board_path = temp.path().join("board.kicad_pcb");
        let dsn_path = temp.path().join("board.dsn");
        let ses_path = temp.path().join("board.ses");
        std::fs::write(&board_path, MULTILAYER).expect("write board fixture");
        let export = export_dsn(&board_path, MULTILAYER, &multilayer_rules()).unwrap();
        std::fs::write(&dsn_path, &export.dsn).expect("write DSN");
        let output = std::process::Command::new("java")
            .arg("-jar")
            .arg(jar)
            .arg("-de")
            .arg(&dsn_path)
            .arg("-do")
            .arg(&ses_path)
            .arg("-mp")
            .arg("4")
            .output()
            .expect("run Freerouting");
        assert!(
            output.status.success(),
            "Freerouting failed:\nstdout:\n{}\nstderr:\n{}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
        let ses = std::fs::read_to_string(&ses_path).expect("read SES");
        let plan =
            crate::specctra_ses::parse_import_plan(&board_path, MULTILAYER, &export.manifest, &ses)
                .unwrap_or_else(|error| panic!("{error:#}\n{ses}"));
        assert!(!plan.tracks.is_empty(), "{ses}");
        assert!(
            plan.tracks.iter().all(|track| track.layer != "In1.Cu"),
            "no tracks on the power plane"
        );
        if let Ok(keep) = std::env::var("KONNECT_KEEP_FREEROUTING_OUTPUT") {
            std::fs::copy(&dsn_path, Path::new(&keep).join("multilayer.dsn")).unwrap();
            std::fs::copy(&ses_path, Path::new(&keep).join("multilayer.ses")).unwrap();
        }
    }

    #[test]
    fn acknowledged_custom_rules_are_recorded_and_imported() {
        let temp = tempfile::tempdir().unwrap();
        let board = temp.path().join("board.kicad_pcb");
        std::fs::write(&board, MULTILAYER).unwrap();
        let options = ExportOptions {
            custom_rules_sha256: Some("ab".repeat(32)),
            ..Default::default()
        };
        let export = export_dsn_with(&board, MULTILAYER, &multilayer_rules(), &options).unwrap();
        let manifest: serde_json::Value = serde_json::from_str(&export.manifest).unwrap();
        assert_eq!(manifest["supported_profile"]["custom_rules"], true);
        assert_eq!(
            manifest["supported_profile"]["custom_rules_sha256"],
            "ab".repeat(32)
        );

        let mut forged: serde_json::Value = manifest.clone();
        forged["supported_profile"]["custom_rules_sha256"] = serde_json::Value::Null;
        let error = crate::specctra_ses::parse_import_plan(
            &board,
            MULTILAYER,
            &forged.to_string(),
            "(session board)",
        )
        .unwrap_err()
        .to_string();
        assert!(error.contains("supported routing profile"), "{error}");
    }

    #[test]
    fn only_literal_courtyard_pair_clearance_rules_are_read_from_the_dru() {
        let dru = r#"(version 1)
# A field around one part.
(rule "Field"
  (condition "A.intersectsCourtyard('U7') && B.intersectsCourtyard('U7')")
  (constraint clearance (min 0.15mm)))
(rule "Field, reversed and bracketed"
  (condition "( B.intersectsCourtyard('Q2') )&&(A.intersectsCourtyard('Q2'))")
  (constraint clearance (min 6mil)))
(rule "Layer scoped" (layer "F.Cu")
  (condition "A.intersectsCourtyard('U7') && B.intersectsCourtyard('U7')")
  (constraint clearance (min 0.2mm)))
(rule "Wildcard"
  (condition "A.intersectsCourtyard('U*') && B.intersectsCourtyard('U*')")
  (constraint clearance (min 0.2mm)))
(rule "One side"
  (condition "A.intersectsCourtyard('U7')")
  (constraint clearance (min 0.2mm)))
(rule "Two parts"
  (condition "A.intersectsCourtyard('U7') && B.intersectsCourtyard('U8')")
  (constraint clearance (min 0.2mm)))
(rule "Negated"
  (condition "A.intersectsCourtyard('U7') && !B.intersectsCourtyard('U7')")
  (constraint clearance (min 0.2mm)))
(rule "Width only"
  (condition "A.intersectsCourtyard('U7') && B.intersectsCourtyard('U7')")
  (constraint track_width (min 0.2mm)))
(rule "No unit"
  (condition "A.intersectsCourtyard('U7') && B.intersectsCourtyard('U7')")
  (constraint clearance (min 0.2)))
(rule "Ignored"
  (severity ignore)
  (condition "A.intersectsCourtyard('U7') && B.intersectsCourtyard('U7')")
  (constraint clearance (min 0.3mm)))
"#;
        assert_eq!(
            courtyard_clearance_rules(dru).unwrap(),
            vec![
                CourtyardClearance {
                    rule_name: "Field".to_string(),
                    reference: "U7".to_string(),
                    clearance_um: 150,
                },
                CourtyardClearance {
                    rule_name: "Field, reversed and bracketed".to_string(),
                    reference: "Q2".to_string(),
                    clearance_um: 152,
                },
            ]
        );
        assert_eq!(
            courtyard_pair_reference(
                r#"A.intersectsCourtyard("U1") && B.intersectsCourtyard("U1")"#
            ),
            Some("U1".to_string())
        );
        assert_eq!(
            courtyard_pair_reference("A.intersectsCourtyard('U1') || B.intersectsCourtyard('U1')"),
            None
        );
        assert_eq!(dru_length_um("0.006in"), Some(152));
        assert_eq!(dru_length_um("-0.1mm"), None);
    }

    /// The two-resistor board with R2 moved onto two nets of its own, one of
    /// them in a wider class.
    fn field_board() -> (String, IpcEffectiveRoutingRules) {
        let source = include_str!("../tests/fixtures/specctra_two_resistors.kicad_pcb");
        let split = source.find("\"Reference\" \"R2\"").unwrap();
        let (r1, r2) = source.split_at(split);
        let board = format!(
            "{r1}{}",
            r2.replace("(net \"GND\")", "(net \"SIG_A\")")
                .replace("(net \"VCC\")", "(net \"SIG_B\")")
        );
        let mut rules = rules();
        rules.insert("SIG_A".to_string(), rules["GND"].clone());
        let mut wide = rules["GND"].clone();
        wide.track_width_mm = Some(0.3);
        rules.insert("SIG_B".to_string(), wide);
        (board, rules)
    }

    fn field_options(rules: &[(&str, i64)]) -> ExportOptions {
        ExportOptions {
            courtyard_clearances: rules
                .iter()
                .map(|(reference, clearance_um)| CourtyardClearance {
                    rule_name: format!("{reference} field"),
                    reference: reference.to_string(),
                    clearance_um: *clearance_um,
                })
                .collect(),
            ..Default::default()
        }
    }

    /// Every `(class_class (classes A B) (rule (clearance X)))`, by pair.
    fn class_pairs(dsn: &str) -> BTreeMap<(String, String), i64> {
        let tree = parse_dsn_sexp(dsn).unwrap();
        tree.find("network")
            .unwrap()
            .find_all("class_class")
            .into_iter()
            .map(|rule| {
                let classes = rule.find("classes").unwrap();
                let clearance = rule.find("rule").unwrap().find("clearance").unwrap();
                (
                    (
                        dsn_atom(classes, 1, "class").unwrap().to_string(),
                        dsn_atom(classes, 2, "class").unwrap().to_string(),
                    ),
                    dsn_integer(clearance, 1, "clearance").unwrap(),
                )
            })
            .collect()
    }

    fn class_clearances(dsn: &str) -> BTreeMap<String, (BTreeSet<String>, i64)> {
        let tree = parse_dsn_sexp(dsn).unwrap();
        dsn_identity(&tree)
            .unwrap()
            .classes
            .into_iter()
            .map(|(name, class)| (name, (class.nets, class.clearance)))
            .collect()
    }

    #[test]
    fn a_courtyard_rule_raises_clearance_only_between_the_footprints_nets() {
        let (board, rules) = field_board();
        let path = Path::new("board.kicad_pcb");
        let export = export_dsn_with(path, &board, &rules, &field_options(&[("R2", 300)])).unwrap();

        // R2's nets get the field clearance in classes of their own; the
        // board default and R1's nets keep 0.2 mm.
        let nets = |names: &[&str]| names.iter().map(|n| n.to_string()).collect::<BTreeSet<_>>();
        assert_eq!(
            class_clearances(&export.dsn),
            BTreeMap::from([
                (
                    "konnect_class_0001".to_string(),
                    (nets(&["GND", "VCC"]), 200)
                ),
                ("konnect_class_0002".to_string(), (nets(&["SIG_A"]), 300)),
                ("konnect_class_0003".to_string(), (nets(&["SIG_B"]), 300)),
            ])
        );
        let pcb = parse_dsn_sexp(&export.dsn).unwrap();
        let structure_rule = pcb.find("structure").unwrap().find("rule").unwrap();
        assert_eq!(
            dsn_integer(structure_rule.find("clearance").unwrap(), 1, "clearance").unwrap(),
            200
        );
        // Each pair is set to what it needs: the field inside, 0.2 across,
        // and 0.2 against Freerouting's default class.
        let pair = |a: &str, b: &str| (format!("konnect_class_{a}"), format!("konnect_class_{b}"));
        let default = |a: &str| (format!("konnect_class_{a}"), "default".to_string());
        assert_eq!(
            class_pairs(&export.dsn),
            BTreeMap::from([
                (pair("0001", "0002"), 200),
                (pair("0001", "0003"), 200),
                (pair("0002", "0003"), 300),
                (default("0002"), 200),
                (default("0003"), 200),
            ])
        );
        assert_eq!(export.class_pair_rule_count, 5);
        assert_eq!(export.courtyard_clearances.len(), 1);
        assert_eq!(export.courtyard_clearances[0].raised_net_count, 2);
        assert!(export.courtyard_clearances[0].footprint_found);
        // The field classes still use the class via; the manifest names
        // each net's class.
        assert_eq!(
            pcb.find("structure")
                .unwrap()
                .find("via")
                .unwrap()
                .children()
                .unwrap()
                .len(),
            3
        );
        let manifest: serde_json::Value = serde_json::from_str(&export.manifest).unwrap();
        let class_of = |net: &str| {
            manifest["nets"]
                .as_array()
                .unwrap()
                .iter()
                .find(|entry| entry["name"] == net)
                .unwrap()["class_name"]
                .clone()
        };
        assert_eq!(class_of("SIG_A"), "konnect_class_0002");
        assert_eq!(class_of("GND"), "konnect_class_0001");
    }

    #[test]
    fn courtyard_rules_that_change_nothing_leave_the_dsn_unchanged() {
        let (board, rules) = field_board();
        let path = Path::new("board.kicad_pcb");
        let plain = export_dsn(path, &board, &rules).unwrap();
        assert!(!plain.dsn.contains("class_class"));
        assert!(plain.courtyard_clearances.is_empty());

        // A missing footprint, and a field no tighter than the net classes.
        let inert = export_dsn_with(
            path,
            &board,
            &rules,
            &field_options(&[("U99", 300), ("R2", 200)]),
        )
        .unwrap();
        assert_eq!(inert.dsn, plain.dsn);
        assert_eq!(inert.manifest, plain.manifest);
        assert_eq!(inert.class_pair_rule_count, 0);
        assert!(!inert.courtyard_clearances[0].footprint_found);
        assert_eq!(inert.courtyard_clearances[1].raised_net_count, 0);
    }

    /// The courtyard rules of a real board, read from a fixture directory
    /// outside the repository (`KONNECT_DB1_FIXTURE_DIR`, holding one
    /// `.kicad_pcb` and its `.kicad_dru`). The saved board gives no
    /// effective rules, so every net gets one uniform class here: the test
    /// proves the lowering on a full-size board, not its clearances.
    #[test]
    fn a_real_boards_courtyard_rules_lower_into_its_dsn() {
        let Some(dir) = std::env::var_os("KONNECT_DB1_FIXTURE_DIR").map(std::path::PathBuf::from)
        else {
            eprintln!("skipped: KONNECT_DB1_FIXTURE_DIR is not set");
            return;
        };
        let find = |extension: &str| {
            std::fs::read_dir(&dir).ok()?.find_map(|entry| {
                let path = entry.ok()?.path();
                (path.extension()? == extension).then_some(path)
            })
        };
        let (Some(board), Some(dru)) = (find("kicad_pcb"), find("kicad_dru")) else {
            eprintln!("skipped: no .kicad_pcb and .kicad_dru in {}", dir.display());
            return;
        };
        let source = std::fs::read_to_string(&board).unwrap();
        let rules = courtyard_clearance_rules(&std::fs::read_to_string(&dru).unwrap()).unwrap();
        assert!(!rules.is_empty(), "the fixture board has a courtyard rule");

        let tree = parse_sexp(&source).unwrap();
        let net_table = top_level_net_table(&tree);
        let uniform = IpcRoutingRules {
            class_name: "Default".to_string(),
            constituents: vec!["Default".to_string()],
            track_width_mm: Some(0.1),
            clearance_mm: Some(0.1),
            via_diameter_mm: Some(0.45),
            via_drill_mm: Some(0.2),
        };
        let mut effective = IpcEffectiveRoutingRules::new();
        for footprint in tree.find_all("footprint") {
            for pad in footprint.find_all("pad") {
                if let Some(net) = pad.find("net").and_then(|net| resolve_net(net, &net_table)) {
                    effective.insert(net, uniform.clone());
                }
            }
        }
        let options = ExportOptions {
            custom_rules_sha256: Some("00".repeat(32)),
            courtyard_clearances: rules,
        };
        // The fixture is present, so a failed export is a regression.
        let export = export_dsn_with(&board, &source, &effective, &options)
            .unwrap_or_else(|error| panic!("the fixture board no longer exports: {error:#}"));
        for report in &export.courtyard_clearances {
            eprintln!(
                "{}: {} um, footprint found {}, {} nets raised",
                report.rule.reference,
                report.rule.clearance_um,
                report.footprint_found,
                report.raised_net_count
            );
        }
        assert!(export
            .courtyard_clearances
            .iter()
            .any(|report| report.footprint_found && report.raised_net_count > 0));
        assert!(export.class_pair_rule_count > 0);
        // A field class against the rest keeps the uniform clearance.
        let pairs = class_pairs(&export.dsn);
        assert_eq!(pairs.len(), export.class_pair_rule_count);
        assert!(pairs.values().any(|clearance| *clearance == 100));
    }

    #[test]
    fn a_dsn_with_courtyard_rules_is_not_replaced_by_the_native_one() {
        let (board, rules) = field_board();
        let export = export_dsn_with(
            Path::new("board.kicad_pcb"),
            &board,
            &rules,
            &field_options(&[("R2", 300)]),
        )
        .unwrap();
        let native = include_str!("../tests/fixtures/specctra_two_resistors.native-kicad-10.dsn");
        let error = adopt_native_dsn(export, native.to_string())
            .unwrap_err()
            .to_string();
        assert!(error.contains("courtyard clearance"), "{error}");
    }
}
