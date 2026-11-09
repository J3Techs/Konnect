//! Design-wide nets: every member of a net across the whole sheet hierarchy.
//!
//! The per-file net tools (`get_net_components`, `get_net_connections`, …) read
//! one `.kicad_sch`. A net that leaves its sheet through a global label, a
//! power symbol or a sheet pin is answered there only in part, and on a root
//! sheet that holds no components it is not answered at all. These tools
//! answer design-wide, from two sources:
//!
//! - **KiCad's own netlist** (`kicad-cli sch export netlist --format
//!   kicadsexpr`) for connectivity: which pins are on which net, each pin's
//!   electrical type, and each component's sheet. KiCad resolves the
//!   hierarchy, so none of that is re-derived here.
//! - **The sheet tree**, walked from the root, for what the netlist leaves
//!   out: labels with their kind, position, rotation and shape; power symbols
//!   and `PWR_FLAG`s, which KiCad keeps out of the netlist as components; and
//!   nets that reach no pin at all, which it does not export.
//!
//! A walked item reaches KiCad's net through the pins beside it. Each sheet's
//! wire graph (`sch_connectivity`) gives the segment an item sits on, and
//! segments are joined the way KiCad 10 joins them, checked against
//! `kicad-cli` on the `design_nets` fixture: by name within one sheet
//! instance, whatever the label kind; by global label or power symbol name
//! across sheets; and through a sheet pin to the hierarchical labels of the
//! same name in that instance of the child. A class of joined segments takes
//! the net of the component pins in it. A class with no pin is a net KiCad does
//! not export; a class reaching two of KiCad's nets is reported as a
//! disagreement instead of being guessed.
//!
//! `plan_sheet_removal` asks KiCad the same question about a design that does
//! not exist yet: it copies the hierarchy to a private temporary directory,
//! deletes the sheet blocks there, and netlists the copy. The project is never
//! written.

use crate::mcp::{error::ToolErrorKind, protocol::CallToolResult};
use crate::tool;
use crate::tools::sch_connectivity::{net_graph_for, NetRoot};
use crate::tools::{cli, get_path, invalid_arg, opt_str_list, ToolContext, ToolDef};
use anyhow::{bail, Context, Result};
use konnect_sexp::schematic::{
    extract_all_net_labels, extract_lib_pins_for_unit, extract_symbol_instances, extract_wires,
    find_lib_symbol, parse_at, pin_endpoint, read_schematic,
};
use konnect_sexp::{parse_sexp, SexpNode};
use serde_json::{json, Value};
use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet, VecDeque};
use std::path::{Path, PathBuf};

// ─── Tool definitions ─────────────────────────────────────────────────────────

const ROOT_DESC: &str =
    "Root .kicad_sch of the project. A sub-sheet is refused, with the root named \
     in the error: kicad-cli treats the file it is given as the root of the hierarchy.";

const OUTPUT_DESC: &str = "Optional path for the full result as JSON (every member with sheet, \
     position and identity). The inline response is a summary either way.";

pub(crate) fn tools() -> Vec<ToolDef> {
    vec![
        tool!(
            "get_net_members",
            "Everything on a net across the whole sheet hierarchy: every component pin \
             (reference, pin number and name, electrical type, sheet file and path), every \
             label (kind, sheet, position, rotation, shape), power symbols, PWR_FLAGs and sheet \
             pins, plus a summary of what drives the net. Connectivity is KiCad's own (kicad-cli \
             netlist of the saved files); labels and power symbols come from walking the sheet \
             tree. A name matches a net's KiCad name or any label or power symbol on it, so a \
             local net can be asked for by its label text. Nets that reach no pin, which KiCad \
             does not export, are found through their labels.",
            json!({
                "type": "object",
                "properties": {
                    "schematic": { "type": "string", "description": ROOT_DESC },
                    "nets": {
                        "type": "array",
                        "items": { "type": "string", "minLength": 1 },
                        "description": "Net names to report, e.g. ['PERIPH_3V3']"
                    },
                    "net_glob": {
                        "type": "string",
                        "minLength": 1,
                        "description": "Wildcard over net and label names: '*' any run, '?' one character, e.g. 'PERIPH_3V3*'"
                    },
                    "output": { "type": "string", "description": OUTPUT_DESC }
                },
                "required": ["schematic"]
            }),
            |args, ctx| async move { handle_get_net_members(args, ctx).await }
        ),
        tool!(
            "plan_sheet_removal",
            "Read-only dry run of deleting sheets: KiCad netlists a temporary copy of the \
             hierarchy with the sheet blocks removed, and the result is compared with the design \
             as saved. Reports every net that crosses the removed sheets' boundary, what remains \
             on it, whether it keeps a source, input and power-input pins left with no driver \
             (a pull resistor alone does not drive; an open drain's surviving pull-up keeps its \
             input defined), passive-only remainders with no source (dead rails, orphaned \
             pull-ups), nets reduced to a single pin or label, open-drain nets whose only pull-up \
             rail dies, nets that lose their drive one series part away, and the nets entirely \
             inside the removed sheets. Nothing in the project is written.",
            json!({
                "type": "object",
                "properties": {
                    "schematic": { "type": "string", "description": ROOT_DESC },
                    "sheet_names": {
                        "type": "array",
                        "minItems": 1,
                        "items": { "type": "string", "minLength": 1 },
                        "description": "Sheets to remove, each by Sheetname ('PERIPH 3V3 buck'), sheet path ('/PERIPH 3V3 buck/') or Sheetfile. A name that matches sheet blocks in different places is refused with the candidates listed."
                    },
                    "output": { "type": "string", "description": OUTPUT_DESC }
                },
                "required": ["schematic", "sheet_names"]
            }),
            |args, ctx| async move { handle_plan_sheet_removal(args, ctx).await }
        ),
        tool!(
            "find_undriven_nets",
            "Design-wide check for nets with no source, which ERC does not report: no output, \
             power-out, tri-state, bidirectional, open-collector or open-emitter pin, no PWR_FLAG \
             and no connector pin, and nothing feeding them through a series part (a resistor or \
             crystal from a push-pull output or connector, or an inductor, ferrite, fuse or jumper \
             from a supply). Lists each with its pins and labels, the nets that reach no pin at \
             all, and global labels that occur only once. Nets held only by a pull resistor \
             ('biased') or by a pin that may be driven inside its part ('indeterminate') are \
             counted and named; unlabelled single-pin nets are counted only, since ERC reports \
             those.",
            json!({
                "type": "object",
                "properties": {
                    "schematic": { "type": "string", "description": ROOT_DESC },
                    "output": { "type": "string", "description": OUTPUT_DESC }
                },
                "required": ["schematic"]
            }),
            |args, ctx| async move { handle_find_undriven_nets(args, ctx).await }
        ),
    ]
}

// ─── KiCad's netlist ──────────────────────────────────────────────────────────

/// One pin on one net, as KiCad's netlist exporter resolved it.
#[derive(Debug, Clone)]
pub(crate) struct NetlistNode {
    pub(crate) reference: String,
    pub(crate) pin: String,
    /// KiCad 10.0.7 writes `<name>_<number>` here (`OUT_6`), not the bare pin
    /// name, and leaves it out for an unnamed pin. [`Netlist::pin_name`] reads
    /// the name from the library part instead.
    pub(crate) pin_function: Option<String>,
    /// The electrical type with KiCad's `+no_connect` suffix removed.
    pub(crate) electrical_type: String,
    /// KiCad appended `+no_connect`: a no-connect flag sits on this pin's net.
    pub(crate) no_connect: bool,
}

#[derive(Debug, Clone)]
pub(crate) struct NetlistNet {
    pub(crate) code: String,
    pub(crate) name: String,
    pub(crate) class: Option<String>,
    pub(crate) nodes: Vec<NetlistNode>,
}

#[derive(Debug, Clone)]
pub(crate) struct NetlistComponent {
    pub(crate) reference: String,
    pub(crate) value: String,
    pub(crate) lib: String,
    pub(crate) part: String,
    /// KiCad's sheet path by name, `/A/B/`. For a multi-unit symbol this is
    /// the first unit's sheet only, which is why pins are located through the
    /// sheet walk first.
    pub(crate) sheet_names: String,
    pub(crate) sheet_file: Option<String>,
}

/// `kicad-cli sch export netlist --format kicadsexpr`, parsed.
pub(crate) struct Netlist {
    pub(crate) tool: Option<String>,
    pub(crate) components: Vec<NetlistComponent>,
    pub(crate) nets: Vec<NetlistNet>,
    component_index: HashMap<String, usize>,
    pin_net: HashMap<(String, String), usize>,
    net_by_name: HashMap<String, usize>,
    symbol_owners: HashMap<String, Vec<usize>>,
    lib_pin_names: HashMap<(String, String), HashMap<String, String>>,
}

impl Netlist {
    /// Parse KiCad's S-expression netlist. Anything that is not one is an
    /// error, never an empty design: no evidence is not "no nets".
    pub(crate) fn parse(source: &str) -> Result<Self> {
        let root = parse_sexp(source).context("KiCad's netlist is not a valid S-expression")?;
        if root.head() != Some("export") {
            bail!("KiCad's netlist export does not start with (export …)");
        }
        let tool = root
            .find("design")
            .and_then(|design| design.find_str("tool"))
            .map(str::to_owned);
        let components_node = root
            .find("components")
            .context("KiCad's netlist has no (components …) section")?;
        let nets_node = root
            .find("nets")
            .context("KiCad's netlist has no (nets …) section")?;

        let mut components = Vec::new();
        let mut component_index = HashMap::new();
        let mut symbol_owners: HashMap<String, Vec<usize>> = HashMap::new();
        for comp in components_node.find_all("comp") {
            let reference = comp
                .find_str("ref")
                .context("a component in KiCad's netlist has no (ref …)")?
                .to_owned();
            let property = |name: &str| {
                comp.find_all("property")
                    .into_iter()
                    .find(|property| property.find_str("name") == Some(name))
                    .and_then(|property| property.find_str("value"))
                    .map(str::to_owned)
            };
            let libsource = comp.find("libsource");
            let symbol_uuids: Vec<String> = comp
                .find("tstamps")
                .and_then(SexpNode::children)
                .map(|children| {
                    children
                        .iter()
                        .skip(1)
                        .filter_map(SexpNode::as_str)
                        .map(str::to_owned)
                        .collect()
                })
                .unwrap_or_default();
            let index = components.len();
            for uuid in &symbol_uuids {
                symbol_owners.entry(uuid.clone()).or_default().push(index);
            }
            component_index.entry(reference.clone()).or_insert(index);
            components.push(NetlistComponent {
                reference,
                value: comp.find_str("value").unwrap_or("").to_owned(),
                lib: libsource
                    .and_then(|source| source.find_str("lib"))
                    .unwrap_or("")
                    .to_owned(),
                part: libsource
                    .and_then(|source| source.find_str("part"))
                    .unwrap_or("")
                    .to_owned(),
                sheet_names: comp
                    .find("sheetpath")
                    .and_then(|path| path.find_str("names"))
                    .unwrap_or("/")
                    .to_owned(),
                sheet_file: property("Sheetfile"),
            });
        }

        let mut lib_pin_names = HashMap::new();
        if let Some(libparts) = root.find("libparts") {
            for libpart in libparts.find_all("libpart") {
                let key = (
                    libpart.find_str("lib").unwrap_or("").to_owned(),
                    libpart.find_str("part").unwrap_or("").to_owned(),
                );
                let names = libpart
                    .find("pins")
                    .map(|pins| {
                        pins.find_all("pin")
                            .into_iter()
                            .filter_map(|pin| {
                                let number = pin.find_str("num")?;
                                let name = pin.find_str("name").unwrap_or("");
                                let name = if name == "~" { "" } else { name };
                                Some((number.to_owned(), name.to_owned()))
                            })
                            .collect()
                    })
                    .unwrap_or_default();
                lib_pin_names.insert(key, names);
            }
        }

        let mut nets = Vec::new();
        let mut pin_net = HashMap::new();
        let mut net_by_name = HashMap::new();
        for net in nets_node.find_all("net") {
            let name = net
                .find_str("name")
                .context("a net in KiCad's netlist has no (name …)")?
                .to_owned();
            let index = nets.len();
            let mut nodes = Vec::new();
            for node in net.find_all("node") {
                let reference = node
                    .find_str("ref")
                    .with_context(|| format!("a node of net {name} has no (ref …)"))?
                    .to_owned();
                let pin = node
                    .find_str("pin")
                    .with_context(|| format!("a node of net {name} has no (pin …)"))?
                    .to_owned();
                let pintype = node.find_str("pintype").unwrap_or("");
                let (base, flags) = pintype.split_once('+').unwrap_or((pintype, ""));
                pin_net
                    .entry((reference.clone(), pin.clone()))
                    .or_insert(index);
                nodes.push(NetlistNode {
                    reference,
                    pin,
                    pin_function: node.find_str("pinfunction").map(str::to_owned),
                    electrical_type: base.to_owned(),
                    no_connect: flags.split('+').any(|flag| flag == "no_connect"),
                });
            }
            net_by_name.entry(name.clone()).or_insert(index);
            nets.push(NetlistNet {
                code: net.find_str("code").unwrap_or("").to_owned(),
                name,
                class: net.find_str("class").map(str::to_owned),
                nodes,
            });
        }

        Ok(Netlist {
            tool,
            components,
            nets,
            component_index,
            pin_net,
            net_by_name,
            symbol_owners,
            lib_pin_names,
        })
    }

    pub(crate) fn component(&self, reference: &str) -> Option<&NetlistComponent> {
        self.component_index
            .get(reference)
            .map(|&index| &self.components[index])
    }

    pub(crate) fn net_of(&self, reference: &str, pin: &str) -> Option<usize> {
        self.pin_net
            .get(&(reference.to_owned(), pin.to_owned()))
            .copied()
    }

    pub(crate) fn net_named(&self, name: &str) -> Option<usize> {
        self.net_by_name.get(name).copied()
    }

    /// The bare pin name, from the library part KiCad exported with the
    /// component; `pinfunction` minus its `_<number>` suffix when the part
    /// does not list the pin.
    pub(crate) fn pin_name(&self, node: &NetlistNode) -> String {
        let listed = self
            .component(&node.reference)
            .and_then(|component| {
                self.lib_pin_names
                    .get(&(component.lib.clone(), component.part.clone()))
            })
            .and_then(|pins| pins.get(&node.pin));
        if let Some(name) = listed {
            return name.clone();
        }
        match &node.pin_function {
            Some(function) => function
                .strip_suffix(&format!("_{}", node.pin))
                .unwrap_or(function)
                .to_owned(),
            None => String::new(),
        }
    }

    fn sole_owner_of_symbol(&self, uuid: &str) -> Option<&NetlistComponent> {
        match self.symbol_owners.get(uuid).map(Vec::as_slice) {
            Some([only]) => Some(&self.components[*only]),
            _ => None,
        }
    }
}

// ─── The sheet tree ───────────────────────────────────────────────────────────

/// The three net-label kinds, by how far each one reaches.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub(crate) enum LabelScope {
    Local,
    Global,
    Hierarchical,
}

impl LabelScope {
    pub(crate) fn tag(self) -> &'static str {
        match self {
            LabelScope::Local => "label",
            LabelScope::Global => "global_label",
            LabelScope::Hierarchical => "hierarchical_label",
        }
    }
}

#[derive(Debug, Clone)]
pub(crate) struct LabelItem {
    pub(crate) scope: LabelScope,
    pub(crate) text: String,
    pub(crate) x: f64,
    pub(crate) y: f64,
    pub(crate) rotation: f64,
    pub(crate) shape: Option<String>,
    pub(crate) justify: Option<String>,
    pub(crate) uuid: Option<String>,
    /// A bus label (`D[0..7]`, `{SPI}`) names a bundle, not one net, and is
    /// left out of net membership.
    pub(crate) bus: bool,
    root: NetRoot,
}

/// A power symbol (its `power_in` pin names a net) or a power flag (its
/// `power_out` pin declares the net it touches driven).
#[derive(Debug, Clone)]
pub(crate) struct PowerItem {
    symbol_uuid: Option<String>,
    reference_property: String,
    pub(crate) lib_id: String,
    pub(crate) value: String,
    /// `(power local)` names a net on its own sheet only (KiCad 9 and later).
    pub(crate) global: bool,
    pub(crate) x: f64,
    pub(crate) y: f64,
    root: NetRoot,
}

#[derive(Debug, Clone)]
struct PinItem {
    symbol_uuid: Option<String>,
    reference_property: String,
    pin: String,
    root: NetRoot,
}

#[derive(Debug, Clone)]
pub(crate) struct BlockPin {
    pub(crate) name: String,
    pub(crate) shape: String,
    pub(crate) x: f64,
    pub(crate) y: f64,
    root: NetRoot,
}

/// A `(sheet …)` block: one child sheet as its parent places it.
#[derive(Debug, Clone)]
pub(crate) struct BlockItem {
    pub(crate) uuid: String,
    pub(crate) name: String,
    pub(crate) file: String,
    pub(crate) pins: Vec<BlockPin>,
    /// `(instances … (path P (page N)))`: the page this block has under each
    /// instance of its parent.
    pages: Vec<(String, String)>,
}

/// What one `.kicad_sch` contributes, independent of where it is instanced.
pub(crate) struct SheetFile {
    pub(crate) path: PathBuf,
    pub(crate) uuid: Option<String>,
    pub(crate) labels: Vec<LabelItem>,
    pub(crate) power_symbols: Vec<PowerItem>,
    pub(crate) power_flags: Vec<PowerItem>,
    pins: Vec<PinItem>,
    pub(crate) blocks: Vec<BlockItem>,
    /// Symbol UUID → every `(path, reference)` its `(instances …)` records.
    symbol_refs: HashMap<String, Vec<(String, String)>>,
    pub(crate) unresolved_symbols: usize,
}

/// One place a sheet file appears in the hierarchy.
pub(crate) struct SheetInstance {
    pub(crate) parent: Option<usize>,
    pub(crate) file: usize,
    /// The block's index in its parent file's `blocks`.
    pub(crate) block: Option<usize>,
    pub(crate) sheet_name: String,
    /// `Sheetfile` as the parent wrote it; the root's own file name.
    pub(crate) sheet_file: String,
    /// `/A/B/`, the form KiCad's netlist uses.
    pub(crate) names_path: String,
    /// `/<root uuid>/<block uuid>…`, the form `(instances (path …))` uses.
    pub(crate) uuid_path: String,
    pub(crate) page: Option<String>,
}

pub(crate) struct Hierarchy {
    pub(crate) root: PathBuf,
    pub(crate) files: Vec<SheetFile>,
    pub(crate) instances: Vec<SheetInstance>,
    children: HashMap<(usize, usize), usize>,
    /// Sheets the walk could not follow, said rather than skipped silently.
    pub(crate) problems: Vec<String>,
}

fn canonical(path: &Path) -> PathBuf {
    path.canonicalize().unwrap_or_else(|_| path.to_path_buf())
}

/// KiCad's bus syntax: a vector `NAME[0..7]` or a group `{A B}` / `NAME{A B}`.
fn is_bus_name(text: &str) -> bool {
    (text.contains('[') && text.contains("..") && text.ends_with(']'))
        || (text.contains('{') && text.ends_with('}'))
}

impl Hierarchy {
    /// Walk the tree from `root` with the walk behind `get_sheet_hierarchy`
    /// (`sch_hierarchy::build_hierarchy_node`), so a missing child, a cycle or
    /// an over-deep tree is handled — and reported — exactly as there, and
    /// read each sheet file it reaches once.
    pub(crate) fn load(root: &Path) -> Result<Self> {
        let project_name = crate::tools::project_name_for(root);
        let mut visited = HashSet::new();
        let tree = crate::tools::sch_hierarchy::build_hierarchy_node(
            root,
            &project_name,
            0,
            &mut visited,
        )?;
        if let Some(error) = tree.get("error").and_then(Value::as_str) {
            bail!("cannot read root schematic {}: {error}", root.display());
        }
        let root_file = read_sheet_file(root)
            .with_context(|| format!("cannot read root schematic {}", root.display()))?;
        let mut problems = Vec::new();
        let root_uuid = match &root_file.uuid {
            Some(uuid) => uuid.clone(),
            None => {
                problems.push(
                    "the root schematic has no (uuid …), so symbol instance paths cannot be                      matched; references were taken from KiCad's netlist instead"
                        .to_owned(),
                );
                String::new()
            }
        };
        let root_name = root
            .file_name()
            .map(|name| name.to_string_lossy().into_owned())
            .unwrap_or_default();
        let mut hierarchy = Hierarchy {
            root: root.to_path_buf(),
            files: vec![root_file],
            instances: vec![SheetInstance {
                parent: None,
                file: 0,
                block: None,
                sheet_name: String::new(),
                sheet_file: root_name,
                names_path: "/".to_owned(),
                uuid_path: format!("/{root_uuid}"),
                page: Some("1".to_owned()),
            }],
            children: HashMap::new(),
            problems,
        };
        let mut by_path = HashMap::from([(canonical(root), 0usize)]);
        hierarchy.descend(0, &tree, &mut by_path);
        Ok(hierarchy)
    }

    fn descend(&mut self, instance: usize, node: &Value, by_path: &mut HashMap<PathBuf, usize>) {
        let file_index = self.instances[instance].file;
        let directory = self.files[file_index]
            .path
            .parent()
            .map(Path::to_path_buf)
            .unwrap_or_else(|| PathBuf::from("."));
        for child in node["children"]
            .as_array()
            .map(Vec::as_slice)
            .unwrap_or(&[])
        {
            let text = |key: &str| child[key].as_str().unwrap_or("").to_owned();
            let (name, file, uuid) = (text("name"), text("file"), text("uuid"));
            let (parent_names, parent_uuids) = {
                let parent = &self.instances[instance];
                (parent.names_path.clone(), parent.uuid_path.clone())
            };
            let names_path = format!("{parent_names}{name}/");
            if let Some(error) = child.get("error").and_then(Value::as_str) {
                self.problems
                    .push(format!("sheet {names_path} ({file}) was not read: {error}"));
                continue;
            }
            let Some(block) = self.files[file_index]
                .blocks
                .iter()
                .position(|block| block.uuid == uuid)
            else {
                self.problems.push(format!(
                    "sheet {names_path} has no sheet block with UUID {uuid} in {}",
                    self.files[file_index].path.display()
                ));
                continue;
            };
            let file = self.files[file_index].blocks[block].file.clone();
            let child_path = directory.join(&file);
            let canon = canonical(&child_path);
            let child_file = match by_path.get(&canon) {
                Some(&index) => index,
                None => match read_sheet_file(&child_path) {
                    Ok(sheet) => {
                        self.files.push(sheet);
                        by_path.insert(canon, self.files.len() - 1);
                        self.files.len() - 1
                    }
                    Err(error) => {
                        self.problems.push(format!(
                            "sheet {names_path} ({}) could not be read: {error:#}",
                            child_path.display()
                        ));
                        continue;
                    }
                },
            };
            // The page this block has under this instance of its parent; the
            // hierarchy walk's own answer is the project's first one.
            let page = self.files[file_index].blocks[block]
                .pages
                .iter()
                .find(|(path, _)| *path == parent_uuids)
                .map(|(_, page)| page.clone())
                .or_else(|| child["page"].as_str().map(str::to_owned));
            let index = self.instances.len();
            self.instances.push(SheetInstance {
                parent: Some(instance),
                file: child_file,
                block: Some(block),
                sheet_name: name,
                sheet_file: file,
                names_path,
                uuid_path: format!("{parent_uuids}/{uuid}"),
                page,
            });
            self.children.insert((instance, block), index);
            self.descend(index, child, by_path);
        }
    }

    fn child_of(&self, parent: usize, block: usize) -> Option<usize> {
        self.children.get(&(parent, block)).copied()
    }

    /// Whether `instance` is `ancestor` or below it.
    fn is_within(&self, instance: usize, ancestor: usize) -> bool {
        let mut current = Some(instance);
        while let Some(index) = current {
            if index == ancestor {
                return true;
            }
            current = self.instances[index].parent;
        }
        false
    }

    fn instance_named(&self, names_path: &str) -> Option<usize> {
        self.instances
            .iter()
            .position(|instance| instance.names_path == names_path)
    }
}

/// Read one sheet: its labels, power symbols, flags, component pins and child
/// blocks, each with the wire-graph segment it sits on.
fn read_sheet_file(path: &Path) -> Result<SheetFile> {
    let (_, tree) =
        read_schematic(path).with_context(|| format!("cannot read {}", path.display()))?;
    let wires = extract_wires(&tree);
    let net_labels = extract_all_net_labels(&tree);
    let mut graph = net_graph_for(&tree, &wires, &net_labels);

    let mut labels = Vec::new();
    for node in tree.children().unwrap_or(&[]) {
        let scope = match node.head() {
            Some("label") => LabelScope::Local,
            Some("global_label") => LabelScope::Global,
            Some("hierarchical_label") => LabelScope::Hierarchical,
            _ => continue,
        };
        let Some(text) = node.get(1).and_then(SexpNode::as_str) else {
            continue;
        };
        let Some((x, y, rotation)) = parse_at(node) else {
            continue;
        };
        let justify = node
            .find("effects")
            .and_then(|effects| effects.find("justify"))
            .and_then(SexpNode::children)
            .map(|children| {
                children
                    .iter()
                    .skip(1)
                    .filter_map(SexpNode::as_str)
                    .collect::<Vec<_>>()
                    .join(" ")
            });
        labels.push(LabelItem {
            scope,
            text: text.to_owned(),
            x,
            y,
            rotation,
            shape: node.find_str("shape").map(str::to_owned),
            justify,
            uuid: node.find_str("uuid").map(str::to_owned),
            bus: is_bus_name(text),
            root: graph.root_at(x, y),
        });
    }

    let mut symbol_refs = HashMap::new();
    for node in tree.find_all("symbol") {
        if node.find("lib_id").is_none() {
            continue;
        }
        let Some(uuid) = node.find_str("uuid") else {
            continue;
        };
        let mut refs = Vec::new();
        if let Some(instances) = node.find("instances") {
            for project in instances.find_all("project") {
                for entry in project.find_all("path") {
                    let path = entry.get(1).and_then(SexpNode::as_str);
                    if let (Some(path), Some(reference)) = (path, entry.find_str("reference")) {
                        refs.push((path.to_owned(), reference.to_owned()));
                    }
                }
            }
        }
        symbol_refs.insert(uuid.to_owned(), refs);
    }

    let lib_symbols = tree
        .find("lib_symbols")
        .map(|node| node.find_all("symbol"))
        .unwrap_or_default();
    let mut power_symbols = Vec::new();
    let mut power_flags = Vec::new();
    let mut pins = Vec::new();
    let mut unresolved_symbols = 0;
    for instance in extract_symbol_instances(&tree) {
        let Some(symbol) = find_lib_symbol(&lib_symbols, &instance) else {
            unresolved_symbols += 1;
            continue;
        };
        let transform = instance.pin_transform();
        let unit_pins = extract_lib_pins_for_unit(symbol, instance.unit);
        if let Some(power) = symbol.find("power") {
            // `(power)` before KiCad 9, `(power global)` / `(power local)` after.
            let global = power.get(1).and_then(SexpNode::as_str) != Some("local");
            for pin in &unit_pins {
                let (x, y) = pin_endpoint(pin, transform);
                let item = PowerItem {
                    symbol_uuid: instance.uuid.clone(),
                    reference_property: instance.reference.clone(),
                    lib_id: instance.lib_id.clone(),
                    value: instance.value.clone(),
                    global,
                    x,
                    y,
                    root: graph.root_at(x, y),
                };
                match pin.electrical_type.as_str() {
                    "power_in" if !instance.value.is_empty() => power_symbols.push(item),
                    "power_out" => power_flags.push(item),
                    _ => {}
                }
            }
            continue;
        }
        for pin in unit_pins {
            let (x, y) = pin_endpoint(&pin, transform);
            pins.push(PinItem {
                symbol_uuid: instance.uuid.clone(),
                reference_property: instance.reference.clone(),
                pin: pin.number.clone(),
                root: graph.root_at(x, y),
            });
        }
    }

    let mut blocks = Vec::new();
    for node in tree.find_all("sheet") {
        let property = |name: &str| {
            node.find_all("property")
                .into_iter()
                .find(|property| property.get(1).and_then(SexpNode::as_str) == Some(name))
                .and_then(|property| property.get(2))
                .and_then(SexpNode::as_str)
                .unwrap_or("")
                .to_owned()
        };
        let mut block_pins = Vec::new();
        for pin in node.find_all("pin") {
            let Some(name) = pin.get(1).and_then(SexpNode::as_str) else {
                continue;
            };
            let Some((x, y, _)) = parse_at(pin) else {
                continue;
            };
            block_pins.push(BlockPin {
                name: name.to_owned(),
                shape: pin
                    .get(2)
                    .and_then(SexpNode::as_str)
                    .unwrap_or("passive")
                    .to_owned(),
                x,
                y,
                root: graph.root_at(x, y),
            });
        }
        let pages = node
            .find("instances")
            .map(|instances| {
                instances
                    .find_all("project")
                    .into_iter()
                    .flat_map(|project| project.find_all("path"))
                    .filter_map(|entry| {
                        Some((
                            entry.get(1)?.as_str()?.to_owned(),
                            entry.find_str("page")?.to_owned(),
                        ))
                    })
                    .collect()
            })
            .unwrap_or_default();
        blocks.push(BlockItem {
            uuid: node.find_str("uuid").unwrap_or("").to_owned(),
            name: property("Sheetname"),
            file: property("Sheetfile"),
            pins: block_pins,
            pages,
        });
    }

    Ok(SheetFile {
        path: path.to_path_buf(),
        uuid: tree.find_str("uuid").map(str::to_owned),
        labels,
        power_symbols,
        power_flags,
        pins,
        blocks,
        symbol_refs,
        unresolved_symbols,
    })
}

/// The reference a placed symbol carries in one instance of its sheet: the
/// `(instances …)` entry for that path, which is what KiCad reads; else the
/// one netlist component that lists the symbol's UUID; else the `Reference`
/// property a pre-instance file carries.
fn instance_reference(
    file: &SheetFile,
    instance: &SheetInstance,
    symbol_uuid: Option<&str>,
    reference_property: &str,
    netlist: &Netlist,
) -> Option<String> {
    if let Some(uuid) = symbol_uuid {
        if let Some((_, reference)) = file
            .symbol_refs
            .get(uuid)
            .and_then(|refs| refs.iter().find(|(path, _)| *path == instance.uuid_path))
        {
            return Some(reference.clone());
        }
        if let Some(component) = netlist.sole_owner_of_symbol(uuid) {
            return Some(component.reference.clone());
        }
    }
    (!reference_property.is_empty()).then(|| reference_property.to_owned())
}

// ─── Resolution ───────────────────────────────────────────────────────────────

/// An item found by the walk: `(instance, index)` into the instance's file.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub(crate) enum Item {
    Label(usize, usize),
    Power(usize, usize),
    Flag(usize, usize),
    /// `(parent instance, block index, pin index)`.
    SheetPin(usize, usize, usize),
}

impl Item {
    fn instance(self) -> usize {
        match self {
            Item::Label(instance, _)
            | Item::Power(instance, _)
            | Item::Flag(instance, _)
            | Item::SheetPin(instance, _, _) => instance,
        }
    }
}

pub(crate) struct DesignNet {
    pub(crate) name: String,
    /// The net's index in KiCad's netlist; `None` for a net that reaches no
    /// pin, which KiCad does not export.
    pub(crate) netlist: Option<usize>,
    pub(crate) items: Vec<Item>,
}

pub(crate) struct Resolved {
    /// One entry per netlist net, in netlist order, then the label-only nets.
    pub(crate) nets: Vec<DesignNet>,
    item_nets: HashMap<Item, Vec<usize>>,
    /// Which sheet instance each `(reference, pin)` sits on.
    pin_instance: HashMap<(String, String), usize>,
    /// Classes of joined segments that reached more than one of KiCad's nets.
    pub(crate) disagreements: Vec<Value>,
    /// Items attached through a net name because no pin joined them.
    pub(crate) attached_by_name: Vec<Value>,
    /// Walked component pins KiCad's netlist does not list.
    pub(crate) unmatched_pins: Vec<String>,
}

struct UnionFind {
    parent: Vec<usize>,
}

impl UnionFind {
    fn new(size: usize) -> Self {
        UnionFind {
            parent: (0..size).collect(),
        }
    }

    fn add(&mut self) -> usize {
        self.parent.push(self.parent.len());
        self.parent.len() - 1
    }

    fn find(&mut self, node: usize) -> usize {
        let mut root = node;
        while self.parent[root] != root {
            root = self.parent[root];
        }
        let mut current = node;
        while self.parent[current] != root {
            let next = self.parent[current];
            self.parent[current] = root;
            current = next;
        }
        root
    }

    fn union(&mut self, a: usize, b: usize) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra != rb {
            self.parent[rb] = ra;
        }
    }
}

fn segment(
    uf: &mut UnionFind,
    segments: &mut HashMap<(usize, NetRoot), usize>,
    instance: usize,
    root: NetRoot,
) -> usize {
    *segments.entry((instance, root)).or_insert_with(|| uf.add())
}

impl Resolved {
    pub(crate) fn nets_of(&self, item: Item) -> &[usize] {
        self.item_nets.get(&item).map(Vec::as_slice).unwrap_or(&[])
    }
}

/// Attach every walked item of the instances not in `removed` to the nets in
/// `netlist`.
pub(crate) fn resolve(
    netlist: &Netlist,
    hierarchy: &Hierarchy,
    removed: &HashSet<usize>,
) -> Resolved {
    let net_count = netlist.nets.len();
    let mut uf = UnionFind::new(net_count);
    let mut segments: HashMap<(usize, NetRoot), usize> = HashMap::new();
    let mut globals: HashMap<String, usize> = HashMap::new();
    let mut item_segments: Vec<(Item, usize)> = Vec::new();
    let mut segment_nets: HashMap<usize, BTreeSet<usize>> = HashMap::new();
    let mut pin_instance = HashMap::new();
    let mut unmatched_pins = Vec::new();

    for (index, instance) in hierarchy.instances.iter().enumerate() {
        if removed.contains(&index) {
            continue;
        }
        let file = &hierarchy.files[instance.file];

        for pin in &file.pins {
            let Some(reference) = instance_reference(
                file,
                instance,
                pin.symbol_uuid.as_deref(),
                &pin.reference_property,
                netlist,
            ) else {
                continue;
            };
            match netlist.net_of(&reference, &pin.pin) {
                Some(net) => {
                    let seg = segment(&mut uf, &mut segments, index, pin.root);
                    uf.union(seg, net);
                    segment_nets.entry(seg).or_default().insert(net);
                    pin_instance
                        .entry((reference, pin.pin.clone()))
                        .or_insert(index);
                }
                // `#`-prefixed symbols are not components to KiCad's netlist.
                None if reference.starts_with('#') => {}
                None => unmatched_pins.push(format!(
                    "{reference}.{} on {}",
                    pin.pin, instance.names_path
                )),
            }
        }

        for (position, label) in file.labels.iter().enumerate() {
            if label.bus {
                continue;
            }
            let seg = segment(&mut uf, &mut segments, index, label.root);
            item_segments.push((Item::Label(index, position), seg));
            if label.scope == LabelScope::Global {
                let global = *globals
                    .entry(label.text.clone())
                    .or_insert_with(|| uf.add());
                uf.union(seg, global);
            }
        }
        for (position, power) in file.power_symbols.iter().enumerate() {
            let seg = segment(&mut uf, &mut segments, index, power.root);
            item_segments.push((Item::Power(index, position), seg));
            if power.global {
                let global = *globals
                    .entry(power.value.clone())
                    .or_insert_with(|| uf.add());
                uf.union(seg, global);
            }
        }
        for (position, flag) in file.power_flags.iter().enumerate() {
            let seg = segment(&mut uf, &mut segments, index, flag.root);
            item_segments.push((Item::Flag(index, position), seg));
        }
        for (block_index, block) in file.blocks.iter().enumerate() {
            let child = hierarchy.child_of(index, block_index);
            if child.is_some_and(|child| removed.contains(&child)) {
                // The block itself is what is being deleted, pins and all.
                continue;
            }
            for (pin_index, pin) in block.pins.iter().enumerate() {
                let seg = segment(&mut uf, &mut segments, index, pin.root);
                item_segments.push((Item::SheetPin(index, block_index, pin_index), seg));
                let Some(child) = child else {
                    continue;
                };
                let child_file = &hierarchy.files[hierarchy.instances[child].file];
                for label in child_file.labels.iter().filter(|label| {
                    label.scope == LabelScope::Hierarchical && label.text == pin.name
                }) {
                    let inner = segment(&mut uf, &mut segments, child, label.root);
                    uf.union(seg, inner);
                }
            }
        }
    }

    let mut class_nets: HashMap<usize, BTreeSet<usize>> = HashMap::new();
    for net in 0..net_count {
        class_nets.entry(uf.find(net)).or_default().insert(net);
    }

    let mut nets: Vec<DesignNet> = netlist
        .nets
        .iter()
        .enumerate()
        .map(|(index, net)| DesignNet {
            name: net.name.clone(),
            netlist: Some(index),
            items: Vec::new(),
        })
        .collect();
    let mut item_nets: HashMap<Item, Vec<usize>> = HashMap::new();
    let mut unanchored: BTreeMap<usize, Vec<Item>> = BTreeMap::new();
    // A class of joined segments that reaches two of KiCad's nets is the walk
    // disagreeing with KiCad, whatever its items attach to. It is reported
    // with every item in it; each item still lands on the net its own segment
    // reaches when that is one net, and on each candidate when it is not.
    let mut disagreeing: BTreeMap<usize, Vec<Item>> = BTreeMap::new();

    for &(item, seg) in &item_segments {
        let class = uf.find(seg);
        let in_class = class_nets.get(&class);
        if in_class.is_some_and(|nets| nets.len() > 1) {
            disagreeing.entry(class).or_default().push(item);
        }
        let direct = segment_nets.get(&seg).filter(|direct| direct.len() == 1);
        let targets: Vec<usize> = match (direct, in_class) {
            (Some(direct), _) => direct.iter().copied().collect(),
            (None, Some(in_class)) => in_class.iter().copied().collect(),
            (None, None) => {
                unanchored.entry(class).or_default().push(item);
                continue;
            }
        };
        for &net in &targets {
            nets[net].items.push(item);
        }
        item_nets.insert(item, targets);
    }

    let disagreements = disagreeing
        .iter()
        .map(|(class, items)| {
            let mut kicad_nets: Vec<String> = class_nets[class]
                .iter()
                .map(|&net| netlist.nets[net].name.clone())
                .collect();
            kicad_nets.sort_by(|a, b| natural_cmp(a, b));
            json!({
                "kicad_nets": kicad_nets,
                "items": items.iter().map(|&item| item_summary(item, hierarchy)).collect::<Vec<_>>(),
                "reason": "the sheet walk joins these items into one net and KiCad's netlist keeps them on several; each item is listed under the net its own wire reaches, or under every candidate when that is not one net"
            })
        })
        .collect();

    let mut attached_by_name = Vec::new();
    let mut used_names: HashSet<String> = netlist.nets.iter().map(|net| net.name.clone()).collect();
    for items in unanchored.into_values() {
        let named: BTreeSet<usize> = items
            .iter()
            .filter_map(|&item| kicad_name_of(item, hierarchy))
            .filter_map(|name| netlist.net_named(&name))
            .collect();
        let targets = if named.len() == 1 {
            let net = *named.iter().next().expect("one");
            attached_by_name.push(json!({
                "net": netlist.nets[net].name,
                "items": items.iter().map(|&item| item_summary(item, hierarchy)).collect::<Vec<_>>()
            }));
            net
        } else {
            let mut name = label_only_name(&items, hierarchy);
            if !used_names.insert(name.clone()) {
                let base = name.clone();
                let mut suffix = 2;
                while !used_names.insert(format!("{base} ({suffix})")) {
                    suffix += 1;
                }
                name = format!("{base} ({suffix})");
            }
            nets.push(DesignNet {
                name,
                netlist: None,
                items: Vec::new(),
            });
            nets.len() - 1
        };
        for item in items {
            nets[targets].items.push(item);
            item_nets.insert(item, vec![targets]);
        }
    }

    Resolved {
        nets,
        item_nets,
        pin_instance,
        disagreements,
        attached_by_name,
        unmatched_pins,
    }
}

/// The name KiCad would give the net an item drives, if that item named it.
fn kicad_name_of(item: Item, hierarchy: &Hierarchy) -> Option<String> {
    let instance = &hierarchy.instances[item.instance()];
    let file = &hierarchy.files[instance.file];
    match item {
        Item::Label(_, index) => {
            let label = &file.labels[index];
            Some(match label.scope {
                LabelScope::Global => label.text.clone(),
                _ => format!("{}{}", instance.names_path, label.text),
            })
        }
        Item::Power(_, index) => {
            let power = &file.power_symbols[index];
            Some(if power.global {
                power.value.clone()
            } else {
                format!("{}{}", instance.names_path, power.value)
            })
        }
        Item::Flag(..) | Item::SheetPin(..) => None,
    }
}

/// Name a net KiCad did not export after its strongest driver, in KiCad's
/// `CONNECTION_SUBGRAPH` priority: global label, power symbol, local label,
/// hierarchical label, sheet pin; ties broken by name.
fn label_only_name(items: &[Item], hierarchy: &Hierarchy) -> String {
    let mut best: Option<(u8, String)> = None;
    for &item in items {
        let instance = &hierarchy.instances[item.instance()];
        let file = &hierarchy.files[instance.file];
        let (rank, name) = match item {
            Item::Label(_, index) => {
                let rank = match file.labels[index].scope {
                    LabelScope::Global => 5,
                    LabelScope::Local => 3,
                    LabelScope::Hierarchical => 2,
                };
                (rank, kicad_name_of(item, hierarchy))
            }
            Item::Power(..) => (4, kicad_name_of(item, hierarchy)),
            Item::SheetPin(_, block, pin) => (
                1,
                Some(format!(
                    "{}{}",
                    instance.names_path, file.blocks[block].pins[pin].name
                )),
            ),
            Item::Flag(..) => (0, None),
        };
        let Some(name) = name else {
            continue;
        };
        let wins = match &best {
            None => true,
            Some((held_rank, held)) => rank > *held_rank || (rank == *held_rank && name < *held),
        };
        if wins {
            best = Some((rank, name));
        }
    }
    best.map(|(_, name)| name).unwrap_or_else(|| {
        let flag = items.iter().find_map(|&item| match item {
            Item::Flag(instance, index) => {
                let file = &hierarchy.files[hierarchy.instances[instance].file];
                Some(file.power_flags[index].reference_property.clone())
            }
            _ => None,
        });
        format!("unconnected-({})", flag.unwrap_or_default())
    })
}

// ─── Drive ────────────────────────────────────────────────────────────────────

/// Pin types that drive a net on their own, as the specification lists them,
/// with the open emitter beside the open collector.
const SOURCE_TYPES: [&str; 6] = [
    "output",
    "power_out",
    "tri_state",
    "bidirectional",
    "open_collector",
    "open_emitter",
];

/// Logic outputs that drive both levels, so they drive the far side of a
/// series resistor too. An open collector or open emitter drives one level
/// only: it drives its own net, never the far side of its pull-up.
const PUSH_PULL_TYPES: [&str; 3] = ["output", "tri_state", "bidirectional"];

/// Two-terminal parts a net is fed through, by reference prefix. A resistor
/// or a crystal feeds a signal; only an inductor, ferrite, fuse, jumper or net
/// tie feeds a supply, because a resistor to a rail is a pull-up, not a feed.
/// A capacitor or diode never counts: a dead rail keeps its decoupling and
/// its TVS.
const SERIES_PREFIXES: [&str; 7] = ["R", "Y", "L", "FB", "F", "JP", "NT"];
const RESISTIVE_PREFIXES: [&str; 2] = ["R", "Y"];

/// Connectors bring a net in from off the board.
const CONNECTOR_PREFIXES: [&str; 4] = ["J", "P", "CN", "X"];

/// Simple parts whose passive pins are not a hidden driver.
const SIMPLE_PREFIXES: [&str; 7] = ["C", "D", "TP", "MH", "H", "LED", "SW"];

fn reference_prefix(reference: &str) -> String {
    reference
        .chars()
        .take_while(char::is_ascii_alphabetic)
        .collect::<String>()
        .to_ascii_uppercase()
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum DriveStatus {
    /// A source pin or a `PWR_FLAG` is on the net.
    Driven,
    /// No source pin, but a connector pin: driven from off the board.
    External,
    /// No source of its own; a series part reaches a push-pull output or a
    /// connector, or a supply through an inductor, ferrite, fuse or jumper.
    DrivenThroughSeries,
    /// No drive; a resistor path to a rail sets a static level — a pull-up or
    /// pull-down.
    Biased,
    /// No drive found, but a pin that may be driven from inside its part: a
    /// passive pin of an IC, analog switch or transistor, or a `free` or
    /// `unspecified` pin — on the net itself, which outranks a bias, or
    /// reached through a series part when nothing else is.
    Indeterminate,
    /// Pins, and nothing that drives or biases them.
    Undriven,
    /// No pin at all: labels or power symbols only.
    LabelOnly,
}

impl DriveStatus {
    pub(crate) fn as_str(self) -> &'static str {
        match self {
            DriveStatus::Driven => "driven",
            DriveStatus::External => "external",
            DriveStatus::DrivenThroughSeries => "driven_through_series",
            DriveStatus::Biased => "biased",
            DriveStatus::Indeterminate => "indeterminate",
            DriveStatus::Undriven => "undriven",
            DriveStatus::LabelOnly => "label_only",
        }
    }

    /// Something actively drives the net.
    pub(crate) fn is_driven(self) -> bool {
        matches!(
            self,
            DriveStatus::Driven | DriveStatus::External | DriveStatus::DrivenThroughSeries
        )
    }
}

/// What a series path ended on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AnchorKind {
    /// A push-pull logic output.
    Signal,
    /// A connector pin.
    External,
    /// A `power_out` pin or a `PWR_FLAG`.
    Rail,
    /// A pin that may be driven inside its part.
    Opaque,
}

#[derive(Debug, Clone)]
pub(crate) struct SeriesHop {
    pub(crate) reference: String,
    pub(crate) from_pin: String,
    pub(crate) to_pin: String,
    pub(crate) to_net: usize,
}

#[derive(Debug, Clone)]
pub(crate) struct Drive {
    pub(crate) status: DriveStatus,
    /// Node indices of the source pins in the net.
    pub(crate) sources: Vec<usize>,
    pub(crate) power_flags: usize,
    /// The series path behind `DrivenThroughSeries`, `Biased` or an
    /// `Indeterminate` reached through a part.
    pub(crate) via: Vec<SeriesHop>,
    /// What that path ended on.
    pub(crate) reaches: Option<String>,
    pub(crate) has_inputs: bool,
    pub(crate) passive_only: bool,
    /// Every source on the net is an open collector or open emitter and
    /// nothing else drives it: its other level comes from a pull resistor.
    pub(crate) open_drain_only: bool,
}

/// Drive analysis over one netlist and the walked items resolved onto it.
pub(crate) struct DriveModel<'a> {
    netlist: &'a Netlist,
    flags: Vec<usize>,
    component_nodes: HashMap<&'a str, Vec<(usize, usize)>>,
}

impl<'a> DriveModel<'a> {
    pub(crate) fn new(netlist: &'a Netlist, resolved: &Resolved) -> Self {
        let mut flags = vec![0; netlist.nets.len()];
        for (index, net) in resolved.nets.iter().enumerate().take(netlist.nets.len()) {
            flags[index] = net
                .items
                .iter()
                .filter(|item| matches!(item, Item::Flag(..)))
                .count();
        }
        let mut component_nodes: HashMap<&str, Vec<(usize, usize)>> = HashMap::new();
        for (net_index, net) in netlist.nets.iter().enumerate() {
            for (node_index, node) in net.nodes.iter().enumerate() {
                component_nodes
                    .entry(node.reference.as_str())
                    .or_default()
                    .push((net_index, node_index));
            }
        }
        DriveModel {
            netlist,
            flags,
            component_nodes,
        }
    }

    fn node(&self, (net, node): (usize, usize)) -> &NetlistNode {
        &self.netlist.nets[net].nodes[node]
    }

    /// A two-pin part with both pins passive and a series prefix.
    fn is_series(&self, reference: &str) -> bool {
        let prefix = reference_prefix(reference);
        if !SERIES_PREFIXES.contains(&prefix.as_str()) {
            return false;
        }
        self.component_nodes.get(reference).is_some_and(|nodes| {
            nodes.len() == 2
                && nodes
                    .iter()
                    .all(|&at| self.node(at).electrical_type == "passive")
        })
    }

    fn is_connector(reference: &str) -> bool {
        CONNECTOR_PREFIXES.contains(&reference_prefix(reference).as_str())
    }

    /// A pin whose drive cannot be judged from its type: `free` or
    /// `unspecified`, or a passive pin of a part that is neither a simple
    /// two-terminal part nor a connector (an IC's dV/dt or charge-pump pin, an
    /// analog switch, a transistor's collector).
    fn is_opaque(&self, node: &NetlistNode) -> bool {
        match node.electrical_type.as_str() {
            "free" | "unspecified" => true,
            "passive" => {
                let prefix = reference_prefix(&node.reference);
                !SERIES_PREFIXES.contains(&prefix.as_str())
                    && !CONNECTOR_PREFIXES.contains(&prefix.as_str())
                    && !SIMPLE_PREFIXES.contains(&prefix.as_str())
                    && self
                        .component_nodes
                        .get(node.reference.as_str())
                        .is_some_and(|nodes| nodes.len() >= 3)
            }
            _ => false,
        }
    }

    /// What a series path stopping at `net` would have reached, with a
    /// description of the pin or flag.
    ///
    /// A supply decides first. GND and a 3.3 V rail carry logic pins tied to
    /// them, and a 10 kΩ resistor to such a rail is a pull-down or pull-up,
    /// not a path from the output that happens to share the net.
    fn anchors(&self, net: usize) -> Vec<(AnchorKind, String)> {
        let name = &self.netlist.nets[net].name;
        let nodes = &self.netlist.nets[net].nodes;
        if let Some(node) = nodes
            .iter()
            .find(|node| node.electrical_type == "power_out")
        {
            return vec![(
                AnchorKind::Rail,
                format!("{}.{} (power_out) on {name}", node.reference, node.pin),
            )];
        }
        if self.flags[net] > 0 {
            return vec![(AnchorKind::Rail, format!("PWR_FLAG on {name}"))];
        }
        let mut found = Vec::new();
        if let Some(node) = nodes
            .iter()
            .find(|node| PUSH_PULL_TYPES.contains(&node.electrical_type.as_str()))
        {
            found.push((
                AnchorKind::Signal,
                format!(
                    "{}.{} ({}) on {name}",
                    node.reference, node.pin, node.electrical_type
                ),
            ));
        }
        if let Some(node) = nodes
            .iter()
            .find(|node| Self::is_connector(&node.reference))
        {
            found.push((
                AnchorKind::External,
                format!("{}.{} (connector) on {name}", node.reference, node.pin),
            ));
        }
        if let Some(node) = nodes.iter().find(|node| self.is_opaque(node)) {
            found.push((
                AnchorKind::Opaque,
                format!(
                    "{}.{} ({}, may be driven inside its part) on {name}",
                    node.reference, node.pin, node.electrical_type
                ),
            ));
        }
        found
    }

    /// Shortest series path from `start` to a net whose anchor `accept`s,
    /// given whether the path so far crosses a resistor or crystal.
    fn search(
        &self,
        start: usize,
        accept: impl Fn(AnchorKind, bool) -> bool,
    ) -> Option<(Vec<SeriesHop>, String)> {
        let mut seen = HashSet::from([(start, false)]);
        let mut queue = VecDeque::from([(start, false, Vec::<SeriesHop>::new())]);
        while let Some((net, resistive, path)) = queue.pop_front() {
            if net != start {
                let anchors = self.anchors(net);
                if let Some((_, description)) =
                    anchors.iter().find(|(kind, _)| accept(*kind, resistive))
                {
                    return Some((path, description.clone()));
                }
                // A supply is where a path ends, never what it runs through:
                // every pull-up on the board meets at the 3.3 V rail, and
                // walking on from there would call each of them driven by
                // whatever another pull-up happens to reach.
                if anchors.iter().any(|(kind, _)| *kind == AnchorKind::Rail) {
                    continue;
                }
            }
            for node in &self.netlist.nets[net].nodes {
                if !self.is_series(&node.reference) {
                    continue;
                }
                let crossing_resistive = resistive
                    || RESISTIVE_PREFIXES.contains(&reference_prefix(&node.reference).as_str());
                for &(other_net, other_node) in &self.component_nodes[node.reference.as_str()] {
                    let other = &self.netlist.nets[other_net].nodes[other_node];
                    if other.pin == node.pin || !seen.insert((other_net, crossing_resistive)) {
                        continue;
                    }
                    let mut next = path.clone();
                    next.push(SeriesHop {
                        reference: node.reference.clone(),
                        from_pin: node.pin.clone(),
                        to_pin: other.pin.clone(),
                        to_net: other_net,
                    });
                    queue.push_back((other_net, crossing_resistive, next));
                }
            }
        }
        None
    }

    /// A path that drives: to a push-pull output or a connector through any
    /// series parts, or to a supply through non-resistive ones.
    fn drive_path(&self, net: usize) -> Option<(Vec<SeriesHop>, String)> {
        self.search(net, |kind, resistive| match kind {
            AnchorKind::Signal | AnchorKind::External => true,
            AnchorKind::Rail => !resistive,
            AnchorKind::Opaque => false,
        })
    }

    /// A path that only biases: through a resistor to a supply.
    fn bias_path(&self, net: usize) -> Option<(Vec<SeriesHop>, String)> {
        self.search(net, |kind, _| kind == AnchorKind::Rail)
    }

    pub(crate) fn drive(&self, net: Option<usize>) -> Drive {
        let mut drive = Drive {
            status: DriveStatus::LabelOnly,
            sources: Vec::new(),
            power_flags: 0,
            via: Vec::new(),
            reaches: None,
            has_inputs: false,
            passive_only: false,
            open_drain_only: false,
        };
        let Some(net) = net else {
            return drive;
        };
        let nodes = &self.netlist.nets[net].nodes;
        drive.sources = nodes
            .iter()
            .enumerate()
            .filter(|(_, node)| SOURCE_TYPES.contains(&node.electrical_type.as_str()))
            .map(|(index, _)| index)
            .collect();
        drive.power_flags = self.flags[net];
        drive.has_inputs = nodes
            .iter()
            .any(|node| matches!(node.electrical_type.as_str(), "input" | "power_in"));
        drive.passive_only =
            !nodes.is_empty() && nodes.iter().all(|node| node.electrical_type == "passive");
        let connector = nodes.iter().any(|node| Self::is_connector(&node.reference));
        if nodes.is_empty() {
            return drive;
        }
        if !drive.sources.is_empty() || drive.power_flags > 0 {
            drive.status = DriveStatus::Driven;
            drive.open_drain_only = drive.power_flags == 0
                && !connector
                && drive.sources.iter().all(|&source| {
                    matches!(
                        nodes[source].electrical_type.as_str(),
                        "open_collector" | "open_emitter"
                    )
                })
                && self.drive_path(net).is_none();
            return drive;
        }
        if connector {
            drive.status = DriveStatus::External;
            return drive;
        }
        // A part's own passive pin outranks a bias resistor: an analog
        // switch feeding an ADC input beside a 402 kΩ bleed to ground is fed
        // through the switch, not merely pulled down.
        let (status, path) = if let Some(path) = self.drive_path(net) {
            (DriveStatus::DrivenThroughSeries, Some(path))
        } else if let Some(node) = nodes.iter().find(|node| self.is_opaque(node)) {
            let reaches = format!(
                "{}.{} ({}, may be driven inside its part) on {}",
                node.reference, node.pin, node.electrical_type, self.netlist.nets[net].name
            );
            (DriveStatus::Indeterminate, Some((Vec::new(), reaches)))
        } else if let Some(path) = self.bias_path(net) {
            (DriveStatus::Biased, Some(path))
        } else if let Some(path) = self.search(net, |kind, _| kind == AnchorKind::Opaque) {
            (DriveStatus::Indeterminate, Some(path))
        } else {
            (DriveStatus::Undriven, None)
        };
        drive.status = status;
        if let Some((via, reaches)) = path {
            drive.via = via;
            drive.reaches = Some(reaches);
        }
        drive
    }

    /// Open-drain nets whose pull-up ran to `dead`: with that rail gone and
    /// no other pull left, an input on them floats whenever the open drain
    /// releases.
    fn open_drain_losing_pull_up(&self, dead: usize) -> Vec<(usize, SeriesHop)> {
        let mut found = Vec::new();
        for node in &self.netlist.nets[dead].nodes {
            if !self.is_series(&node.reference) {
                continue;
            }
            for &(other_net, other_node) in &self.component_nodes[node.reference.as_str()] {
                let other = &self.netlist.nets[other_net].nodes[other_node];
                if other.pin == node.pin || other_net == dead {
                    continue;
                }
                if !self.drive(Some(other_net)).open_drain_only {
                    continue;
                }
                if self.bias_path(other_net).is_none() {
                    found.push((
                        other_net,
                        SeriesHop {
                            reference: node.reference.clone(),
                            from_pin: other.pin.clone(),
                            to_pin: node.pin.clone(),
                            to_net: dead,
                        },
                    ));
                }
            }
        }
        found
    }
}

// ─── Rendering ────────────────────────────────────────────────────────────────

/// Compare designators the way a person reads them: `R2` before `R10`.
fn natural_cmp(a: &str, b: &str) -> Ordering {
    let (mut left, mut right) = (a.chars().peekable(), b.chars().peekable());
    loop {
        match (left.peek().copied(), right.peek().copied()) {
            (None, None) => return a.cmp(b),
            (None, Some(_)) => return Ordering::Less,
            (Some(_), None) => return Ordering::Greater,
            (Some(x), Some(y)) if x.is_ascii_digit() && y.is_ascii_digit() => {
                let take = |chars: &mut std::iter::Peekable<std::str::Chars>| {
                    let mut digits = String::new();
                    while let Some(&c) = chars.peek() {
                        if !c.is_ascii_digit() {
                            break;
                        }
                        digits.push(c);
                        chars.next();
                    }
                    digits
                };
                let (dx, dy) = (take(&mut left), take(&mut right));
                let (tx, ty) = (dx.trim_start_matches('0'), dy.trim_start_matches('0'));
                let order = tx.len().cmp(&ty.len()).then_with(|| tx.cmp(ty));
                if order != Ordering::Equal {
                    return order;
                }
            }
            (Some(x), Some(y)) => {
                if x != y {
                    return x.cmp(&y);
                }
                left.next();
                right.next();
            }
        }
    }
}

/// `*` matches any run of characters, `?` exactly one. Case-sensitive, like
/// KiCad net names.
fn glob_match(pattern: &str, text: &str) -> bool {
    let pattern: Vec<char> = pattern.chars().collect();
    let text: Vec<char> = text.chars().collect();
    let (mut p, mut t) = (0, 0);
    let (mut star, mut mark) = (None, 0);
    while t < text.len() {
        if p < pattern.len() && (pattern[p] == '?' || pattern[p] == text[t]) {
            p += 1;
            t += 1;
        } else if p < pattern.len() && pattern[p] == '*' {
            star = Some(p);
            mark = t;
            p += 1;
        } else if let Some(position) = star {
            p = position + 1;
            mark += 1;
            t = mark;
        } else {
            return false;
        }
    }
    while p < pattern.len() && pattern[p] == '*' {
        p += 1;
    }
    p == pattern.len()
}

/// Everything needed to describe nets resolved against one netlist.
struct View<'a> {
    netlist: &'a Netlist,
    hierarchy: &'a Hierarchy,
    resolved: &'a Resolved,
    drive: DriveModel<'a>,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Detail {
    Compact,
    Full,
}

impl<'a> View<'a> {
    fn new(netlist: &'a Netlist, hierarchy: &'a Hierarchy, resolved: &'a Resolved) -> Self {
        View {
            netlist,
            hierarchy,
            resolved,
            drive: DriveModel::new(netlist, resolved),
        }
    }

    fn net(&self, index: usize) -> &DesignNet {
        &self.resolved.nets[index]
    }

    fn nodes(&self, index: usize) -> &[NetlistNode] {
        self.net(index)
            .netlist
            .map(|net| self.netlist.nets[net].nodes.as_slice())
            .unwrap_or(&[])
    }

    fn drive_of(&self, index: usize) -> Drive {
        self.drive.drive(self.net(index).netlist)
    }

    /// Where a pin sits: the walk's answer, else KiCad's component sheet.
    fn pin_sheet(&self, node: &NetlistNode) -> (String, String, Option<usize>) {
        if let Some(&instance) = self
            .resolved
            .pin_instance
            .get(&(node.reference.clone(), node.pin.clone()))
        {
            let instance_ref = &self.hierarchy.instances[instance];
            return (
                instance_ref.names_path.clone(),
                instance_ref.sheet_file.clone(),
                Some(instance),
            );
        }
        match self.netlist.component(&node.reference) {
            Some(component) => {
                let instance = self.hierarchy.instance_named(&component.sheet_names);
                (
                    component.sheet_names.clone(),
                    component
                        .sheet_file
                        .clone()
                        .or_else(|| {
                            instance.map(|i| self.hierarchy.instances[i].sheet_file.clone())
                        })
                        .unwrap_or_default(),
                    instance,
                )
            }
            None => (String::new(), String::new(), None),
        }
    }

    fn pin_json(&self, node: &NetlistNode, detail: Detail) -> Value {
        let (sheet_path, sheet_file, _) = self.pin_sheet(node);
        let mut value = json!({
            "reference": node.reference,
            "pin": node.pin,
            "pin_name": self.netlist.pin_name(node),
            "electrical_type": node.electrical_type,
            "sheet_file": sheet_file,
        });
        if detail == Detail::Full {
            value["sheet_path"] = json!(sheet_path);
            value["value"] = json!(self
                .netlist
                .component(&node.reference)
                .map(|component| component.value.as_str())
                .unwrap_or(""));
        }
        if node.no_connect {
            value["no_connect_flag"] = json!(true);
        }
        value
    }

    fn pin_tag(&self, node: &NetlistNode) -> String {
        let name = self.netlist.pin_name(node);
        if name.is_empty() {
            format!("{}.{} {}", node.reference, node.pin, node.electrical_type)
        } else {
            format!(
                "{}.{} {} {}",
                node.reference, node.pin, name, node.electrical_type
            )
        }
    }

    fn instance_sheet(&self, instance: usize) -> (&str, &str, Option<&str>) {
        let instance = &self.hierarchy.instances[instance];
        (
            instance.names_path.as_str(),
            instance.sheet_file.as_str(),
            instance.page.as_deref(),
        )
    }

    fn item_reference(&self, instance: usize, power: &PowerItem) -> String {
        let instance_ref = &self.hierarchy.instances[instance];
        let file = &self.hierarchy.files[instance_ref.file];
        instance_reference(
            file,
            instance_ref,
            power.symbol_uuid.as_deref(),
            &power.reference_property,
            self.netlist,
        )
        .unwrap_or_default()
    }

    fn item_json(&self, item: Item, detail: Detail) -> Value {
        let (sheet_path, sheet_file, page) = self.instance_sheet(item.instance());
        let file = &self.hierarchy.files[self.hierarchy.instances[item.instance()].file];
        let mut value = match item {
            Item::Label(_, index) => {
                let label = &file.labels[index];
                let mut value = json!({
                    "kind": label.scope.tag(),
                    "text": label.text,
                    "sheet_file": sheet_file,
                    "x": label.x,
                    "y": label.y,
                    "rotation": label.rotation,
                });
                if let Some(shape) = &label.shape {
                    value["shape"] = json!(shape);
                }
                if detail == Detail::Full {
                    value["justify"] = json!(label.justify);
                    value["uuid"] = json!(label.uuid);
                }
                value
            }
            Item::Power(instance, index) | Item::Flag(instance, index) => {
                let power = match item {
                    Item::Power(..) => &file.power_symbols[index],
                    _ => &file.power_flags[index],
                };
                let mut value = json!({
                    "reference": self.item_reference(instance, power),
                    "value": power.value,
                    "sheet_file": sheet_file,
                    "x": power.x,
                    "y": power.y,
                });
                if detail == Detail::Full {
                    value["lib_id"] = json!(power.lib_id);
                    if matches!(item, Item::Power(..)) {
                        value["scope"] = json!(if power.global { "global" } else { "local" });
                    }
                }
                value
            }
            Item::SheetPin(instance, block, pin) => {
                let block_item = &file.blocks[block];
                let pin_item = &block_item.pins[pin];
                let child = self
                    .hierarchy
                    .child_of(instance, block)
                    .map(|child| self.hierarchy.instances[child].names_path.clone());
                json!({
                    "name": pin_item.name,
                    "shape": pin_item.shape,
                    "sheet_file": sheet_file,
                    "child_sheet_path": child.unwrap_or_else(|| format!("{sheet_path}{}/", block_item.name)),
                    "x": pin_item.x,
                    "y": pin_item.y,
                })
            }
        };
        if detail == Detail::Full {
            value["sheet_path"] = json!(sheet_path);
            if let Some(page) = page {
                value["page"] = json!(page);
            }
        }
        value
    }

    fn items_of(&self, index: usize, keep: impl Fn(&Item) -> bool) -> Vec<Item> {
        let mut items: Vec<Item> = self
            .net(index)
            .items
            .iter()
            .copied()
            .filter(|item| keep(item))
            .collect();
        items.sort_by(|a, b| {
            let (pa, _, _) = self.instance_sheet(a.instance());
            let (pb, _, _) = self.instance_sheet(b.instance());
            pa.cmp(pb).then_with(|| a.cmp(b))
        });
        items.dedup();
        items
    }

    fn sorted_nodes(&self, index: usize) -> Vec<&NetlistNode> {
        let mut nodes: Vec<&NetlistNode> = self.nodes(index).iter().collect();
        nodes.sort_by(|a, b| {
            natural_cmp(&a.reference, &b.reference).then_with(|| natural_cmp(&a.pin, &b.pin))
        });
        nodes
    }

    fn drive_json(&self, index: usize, drive: &Drive, detail: Detail) -> Value {
        let nodes = self.nodes(index);
        let sources: Vec<Value> = drive
            .sources
            .iter()
            .map(|&source| self.pin_json(&nodes[source], Detail::Compact))
            .collect();
        let mut value = json!({
            "status": drive.status.as_str(),
            "has_source": !drive.sources.is_empty() || drive.power_flags > 0,
            "sources": sources,
            "power_flags": drive.power_flags,
            "passive_only": drive.passive_only,
            "label_only": drive.status == DriveStatus::LabelOnly,
        });
        if drive.reaches.is_some() {
            value["via"] = json!(drive
                .via
                .iter()
                .map(|hop| self.hop_json(hop))
                .collect::<Vec<_>>());
            value["reaches"] = json!(drive.reaches);
        }
        if drive.open_drain_only {
            value["open_drain_only"] = json!(true);
        }
        if detail == Detail::Full {
            value["has_inputs"] = json!(drive.has_inputs);
        }
        value
    }

    /// One line saying what holds a net's level once its drive is gone.
    fn level_json(&self, drive: &Drive) -> Value {
        match drive.status {
            DriveStatus::Biased => json!(format!(
                "biased: {} to {}",
                drive
                    .via
                    .iter()
                    .map(|hop| hop.reference.as_str())
                    .collect::<Vec<_>>()
                    .join(", "),
                drive.reaches.as_deref().unwrap_or("a rail")
            )),
            DriveStatus::Indeterminate => json!(format!(
                "indeterminate: {}",
                drive
                    .reaches
                    .as_deref()
                    .unwrap_or("a pin on the net may be driven inside its part")
            )),
            DriveStatus::LabelOnly => json!("no pins"),
            _ => json!("floating"),
        }
    }

    fn hop_json(&self, hop: &SeriesHop) -> Value {
        json!({
            "reference": hop.reference,
            "from_pin": hop.from_pin,
            "to_pin": hop.to_pin,
            "to_net": self.netlist.nets[hop.to_net].name,
        })
    }

    fn sheets_json(&self, index: usize) -> Vec<Value> {
        let mut sheets: BTreeMap<String, (String, Option<String>)> = BTreeMap::new();
        for node in self.nodes(index) {
            let (path, file, instance) = self.pin_sheet(node);
            let page = instance.and_then(|i| self.hierarchy.instances[i].page.clone());
            sheets.entry(path).or_insert((file, page));
        }
        for item in &self.net(index).items {
            let instance = &self.hierarchy.instances[item.instance()];
            sheets
                .entry(instance.names_path.clone())
                .or_insert((instance.sheet_file.clone(), instance.page.clone()));
        }
        sheets
            .into_iter()
            .map(|(path, (file, page))| json!({ "sheet_path": path, "sheet_file": file, "page": page }))
            .collect()
    }

    fn aliases(&self, index: usize) -> BTreeSet<String> {
        let mut names = BTreeSet::new();
        for &item in &self.net(index).items {
            let file = &self.hierarchy.files[self.hierarchy.instances[item.instance()].file];
            match item {
                Item::Label(_, label) => {
                    names.insert(file.labels[label].text.clone());
                }
                Item::Power(_, power) => {
                    names.insert(file.power_symbols[power].value.clone());
                }
                Item::Flag(..) | Item::SheetPin(..) => {}
            }
        }
        names
    }

    fn net_json(&self, index: usize, detail: Detail, with_members: bool) -> Value {
        let net = self.net(index);
        let drive = self.drive_of(index);
        let labels = self.items_of(index, |item| matches!(item, Item::Label(..)));
        let powers = self.items_of(index, |item| matches!(item, Item::Power(..)));
        let flags = self.items_of(index, |item| matches!(item, Item::Flag(..)));
        let sheet_pins = self.items_of(index, |item| matches!(item, Item::SheetPin(..)));
        let nodes = self.sorted_nodes(index);
        let mut value = json!({
            "net": net.name,
            "kicad_net": net.netlist.is_some(),
            "names": self.aliases(index),
            "drive": self.drive_json(index, &drive, detail),
            "counts": {
                "pins": nodes.len(),
                "labels": labels.len(),
                "power_symbols": powers.len(),
                "power_flags": flags.len(),
                "sheet_pins": sheet_pins.len(),
            },
        });
        if let Some(netlist_index) = net.netlist {
            let kicad = &self.netlist.nets[netlist_index];
            value["code"] = json!(kicad.code);
            value["class"] = json!(kicad.class);
        }
        if with_members {
            value["sheets"] = json!(self.sheets_json(index));
            value["pins"] = json!(nodes
                .iter()
                .map(|node| self.pin_json(node, detail))
                .collect::<Vec<_>>());
            value["labels"] = json!(labels
                .iter()
                .map(|&item| self.item_json(item, detail))
                .collect::<Vec<_>>());
            value["power_symbols"] = json!(powers
                .iter()
                .map(|&item| self.item_json(item, detail))
                .collect::<Vec<_>>());
            value["power_flags"] = json!(flags
                .iter()
                .map(|&item| self.item_json(item, detail))
                .collect::<Vec<_>>());
            value["sheet_pins"] = json!(sheet_pins
                .iter()
                .map(|&item| self.item_json(item, detail))
                .collect::<Vec<_>>());
        }
        value
    }

    fn member_count(&self, index: usize) -> usize {
        self.nodes(index).len() + self.net(index).items.len()
    }
}

/// A short identity for an item, for diagnostics.
fn item_summary(item: Item, hierarchy: &Hierarchy) -> String {
    let instance = &hierarchy.instances[item.instance()];
    let file = &hierarchy.files[instance.file];
    match item {
        Item::Label(_, index) => {
            let label = &file.labels[index];
            format!(
                "{} '{}' on {} at ({}, {})",
                label.scope.tag(),
                label.text,
                instance.names_path,
                label.x,
                label.y
            )
        }
        Item::Power(_, index) => {
            let power = &file.power_symbols[index];
            format!(
                "power symbol '{}' on {} at ({}, {})",
                power.value, instance.names_path, power.x, power.y
            )
        }
        Item::Flag(_, index) => {
            let flag = &file.power_flags[index];
            format!(
                "power flag {} on {} at ({}, {})",
                flag.reference_property, instance.names_path, flag.x, flag.y
            )
        }
        Item::SheetPin(_, block, pin) => {
            let pin = &file.blocks[block].pins[pin];
            format!(
                "sheet pin '{}' of '{}' on {}",
                pin.name, file.blocks[block].name, instance.names_path
            )
        }
    }
}

// ─── Handlers ─────────────────────────────────────────────────────────────────

/// The design every tool here reads: KiCad's netlist of the saved hierarchy
/// and the walked sheet tree.
struct Design {
    netlist: Netlist,
    hierarchy: Hierarchy,
}

/// Validate the root, netlist it with kicad-cli and walk it. `Ok(Err)` is a
/// refusal to return as-is.
async fn load_design(
    args: &Value,
    ctx: &ToolContext,
) -> Result<std::result::Result<(PathBuf, Design), CallToolResult>> {
    let root = get_path(args, "schematic")?;
    if !root.is_file() {
        return Ok(Err(CallToolResult::error_kind(
            ToolErrorKind::FileNotFound {
                path: root.display().to_string(),
            },
            format!("Schematic '{}' not found", root.display()),
        )));
    }
    match crate::tools::sch_export::owning_project_root(&root) {
        Err(error) => return Ok(Err(error.into_tool_result())),
        Ok(Some(owner)) => {
            let reason = format!(
                "{} is a sheet inside the project rooted at {}. kicad-cli treats the file it is \
                 given as the root of the hierarchy, so a sub-sheet would be netlisted without \
                 its parents and without the project's libraries. Run on {}.",
                root.display(),
                owner.display(),
                owner.display()
            );
            return Ok(Err(CallToolResult::error_kind(
                ToolErrorKind::InvalidArgument {
                    field: "schematic".to_owned(),
                    reason: reason.clone(),
                },
                reason,
            )));
        }
        Ok(None) => {}
    }
    let design = load_design_from(&ctx.config.kicad_cli, &root).await?;
    Ok(Ok((root, design)))
}

async fn load_design_from(kicad_cli: &str, root: &Path) -> Result<Design> {
    let text = cli::export_netlist_text(kicad_cli, root)
        .await
        .context("KiCad's netlist export failed, so the design's connectivity is unknown")?;
    let netlist = Netlist::parse(&text)?;
    let hierarchy = Hierarchy::load(root)?;
    Ok(Design { netlist, hierarchy })
}

/// Where the answer came from: saved files, KiCad's netlist for connectivity,
/// the sheet walk for labels and power symbols.
fn source_json(root: &Path, design: &Design, resolved: &Resolved) -> Value {
    let unresolved: usize = design
        .hierarchy
        .files
        .iter()
        .map(|file| file.unresolved_symbols)
        .sum();
    let mut problems = design.hierarchy.problems.clone();
    if unresolved > 0 {
        problems.push(format!(
            "{unresolved} placed symbol(s) have no embedded library definition, so their power \
             pins and flags were not read from the sheets"
        ));
    }
    json!({
        "root": root.display().to_string(),
        "documents": "saved_files",
        "connectivity": "kicad_netlist",
        "kicad_tool": design.netlist.tool,
        "labels": "sheet_walk",
        "sheet_instances": design.hierarchy.instances.len(),
        "sheet_files": design.hierarchy.files.len(),
        "complete": problems.is_empty() && resolved.disagreements.is_empty() && resolved.unmatched_pins.is_empty(),
        "problems": problems,
        "disagreements": resolved.disagreements,
        "attached_by_name": resolved.attached_by_name,
        "unmatched_pins": resolved.unmatched_pins,
    })
}

fn output_path(args: &Value) -> std::result::Result<Option<PathBuf>, CallToolResult> {
    match args.get("output") {
        None | Some(Value::Null) => Ok(None),
        Some(Value::String(path)) if !path.trim().is_empty() => Ok(Some(PathBuf::from(path))),
        Some(_) => Err(invalid_arg("output", "must be a non-empty path string")),
    }
}

/// Write the full result and say where, by reading back what was published.
async fn write_output(path: &Path, full: &Value) -> Result<Value> {
    let bytes = serde_json::to_vec_pretty(full)?;
    cli::publish_verified_bytes(path, &bytes, "Report").await?;
    let written = tokio::fs::metadata(path).await?.len();
    Ok(json!({ "path": path.display().to_string(), "bytes": written }))
}

/// Above this many members (pins, labels, symbols, flags, sheet pins) across
/// the matched nets, `get_net_members` reports counts inline and leaves the
/// member lists to `output`.
const INLINE_MEMBER_LIMIT: usize = 400;

async fn handle_get_net_members(args: &Value, ctx: &ToolContext) -> Result<CallToolResult> {
    let names = match opt_str_list(args, "nets") {
        Ok(names) => names.unwrap_or_default(),
        Err(error) => return Ok(error),
    };
    if names.iter().any(|name| name.is_empty()) {
        return Ok(invalid_arg("nets", "net names must not be empty"));
    }
    let glob = match args.get("net_glob") {
        None | Some(Value::Null) => None,
        Some(Value::String(pattern)) if !pattern.is_empty() => Some(pattern.clone()),
        Some(_) => return Ok(invalid_arg("net_glob", "must be a non-empty string")),
    };
    if names.is_empty() && glob.is_none() {
        return Ok(invalid_arg(
            "nets",
            "name at least one net in 'nets', or give 'net_glob'",
        ));
    }
    let output = match output_path(args) {
        Ok(output) => output,
        Err(error) => return Ok(error),
    };
    let (root, design) = match load_design(args, ctx).await? {
        Ok(loaded) => loaded,
        Err(refusal) => return Ok(refusal),
    };
    let resolved = resolve(&design.netlist, &design.hierarchy, &HashSet::new());
    let view = View::new(&design.netlist, &design.hierarchy, &resolved);

    let mut matched: BTreeMap<usize, BTreeSet<&'static str>> = BTreeMap::new();
    let mut not_found = Vec::new();
    let aliases: Vec<BTreeSet<String>> =
        (0..resolved.nets.len()).map(|i| view.aliases(i)).collect();
    for name in &names {
        let mut any = false;
        for (index, net) in resolved.nets.iter().enumerate() {
            if net.name == *name {
                matched.entry(index).or_default().insert("net_name");
                any = true;
            } else if aliases[index].contains(name) {
                matched.entry(index).or_default().insert("label");
                any = true;
            }
        }
        if !any {
            not_found.push(name.clone());
        }
    }
    if let Some(pattern) = &glob {
        for (index, net) in resolved.nets.iter().enumerate() {
            if glob_match(pattern, &net.name) {
                matched.entry(index).or_default().insert("net_glob");
            } else if aliases[index]
                .iter()
                .any(|alias| glob_match(pattern, alias))
            {
                matched.entry(index).or_default().insert("net_glob_label");
            }
        }
    }

    let mut order: Vec<usize> = matched.keys().copied().collect();
    order.sort_by(|&a, &b| natural_cmp(&resolved.nets[a].name, &resolved.nets[b].name));
    let members: usize = order.iter().map(|&index| view.member_count(index)).sum();
    let inline_members = members <= INLINE_MEMBER_LIMIT;

    let render = |detail: Detail, with_members: bool| -> Vec<Value> {
        order
            .iter()
            .map(|&index| {
                let mut value = view.net_json(index, detail, with_members);
                value["matched_by"] = json!(matched[&index]);
                value
            })
            .collect()
    };
    let source = source_json(&root, &design, &resolved);
    let mut response = json!({
        "net_count": order.len(),
        "nets": render(Detail::Compact, inline_members),
        "not_found": not_found,
        "members_inline": inline_members,
        "source": source,
    });
    if !inline_members {
        response["members_omitted"] = json!(format!(
            "{members} members across the matched nets exceed the inline limit of \
             {INLINE_MEMBER_LIMIT}; give 'output' for every member, or ask for fewer nets"
        ));
    }
    if let Some(path) = output {
        let full = json!({
            "net_count": order.len(),
            "nets": render(Detail::Full, true),
            "not_found": response["not_found"],
            "source": response["source"],
        });
        response["output"] = write_output(&path, &full).await?;
    }
    Ok(CallToolResult::json(&response))
}

/// The undriven-net report for one resolved design.
struct UndrivenReport {
    undriven: Vec<usize>,
    biased: Vec<usize>,
    label_only: Vec<usize>,
    indeterminate: Vec<usize>,
    single_globals: Vec<(String, Item)>,
    skipped_unconnected: usize,
    skipped_no_connect: usize,
}

fn undriven_report(view: &View) -> UndrivenReport {
    let mut report = UndrivenReport {
        undriven: Vec::new(),
        biased: Vec::new(),
        label_only: Vec::new(),
        indeterminate: Vec::new(),
        single_globals: Vec::new(),
        skipped_unconnected: 0,
        skipped_no_connect: 0,
    };
    for index in 0..view.resolved.nets.len() {
        let nodes = view.nodes(index);
        let named = view
            .net(index)
            .items
            .iter()
            .any(|item| matches!(item, Item::Label(..) | Item::Power(..)));
        if !nodes.is_empty()
            && nodes
                .iter()
                .all(|node| node.no_connect || node.electrical_type == "no_connect")
        {
            report.skipped_no_connect += 1;
            continue;
        }
        let drive = view.drive_of(index);
        match drive.status {
            DriveStatus::LabelOnly => report.label_only.push(index),
            DriveStatus::Undriven if nodes.len() == 1 && !named => report.skipped_unconnected += 1,
            DriveStatus::Undriven => report.undriven.push(index),
            DriveStatus::Biased => report.biased.push(index),
            DriveStatus::Indeterminate => report.indeterminate.push(index),
            _ => {}
        }
    }
    let mut globals: BTreeMap<String, Vec<Item>> = BTreeMap::new();
    for (instance_index, instance) in view.hierarchy.instances.iter().enumerate() {
        let file = &view.hierarchy.files[instance.file];
        for (index, label) in file.labels.iter().enumerate() {
            if label.scope == LabelScope::Global && !label.bus {
                globals
                    .entry(label.text.clone())
                    .or_default()
                    .push(Item::Label(instance_index, index));
            }
        }
    }
    report.single_globals = globals
        .into_iter()
        .filter_map(|(name, items)| match items.as_slice() {
            [only] => Some((name, *only)),
            _ => None,
        })
        .collect();
    let by_name = |a: &usize, b: &usize| natural_cmp(&view.net(*a).name, &view.net(*b).name);
    report.undriven.sort_by(by_name);
    report.biased.sort_by(by_name);
    report.label_only.sort_by(by_name);
    report.indeterminate.sort_by(by_name);
    report
}

fn undriven_net_json(view: &View, index: usize, detail: Detail) -> Value {
    let nodes = view.sorted_nodes(index);
    let tags = |keep: &dyn Fn(&NetlistNode) -> bool| -> Vec<String> {
        nodes
            .iter()
            .filter(|node| keep(node))
            .map(|node| view.pin_tag(node))
            .collect()
    };
    let inputs = tags(&|node| matches!(node.electrical_type.as_str(), "input" | "power_in"));
    let passives = tags(&|node| node.electrical_type == "passive");
    let others = tags(&|node| {
        !matches!(
            node.electrical_type.as_str(),
            "input" | "power_in" | "passive"
        )
    });
    let labels = view.items_of(index, |item| {
        matches!(item, Item::Label(..) | Item::Power(..))
    });
    let mut value = json!({
        "net": view.net(index).name,
        "kind": if inputs.is_empty() { "passive_only" } else { "inputs" },
        "inputs": inputs,
        "passives": passives,
        "labels": labels.iter().map(|&item| view.item_json(item, detail)).collect::<Vec<_>>(),
    });
    if !others.is_empty() {
        value["other_pins"] = json!(others);
    }
    if detail == Detail::Full {
        value["pins"] = json!(nodes
            .iter()
            .map(|node| view.pin_json(node, Detail::Full))
            .collect::<Vec<_>>());
        value["sheets"] = json!(view.sheets_json(index));
    }
    value
}

/// Entries of a secondary list shown inline before the rest are counted.
const SUMMARY_LIST_LIMIT: usize = 50;

fn undriven_json(view: &View, report: &UndrivenReport, detail: Detail) -> Value {
    let full = detail == Detail::Full;
    let label_only: Vec<Value> = report
        .label_only
        .iter()
        .map(|&index| {
            let items = view.items_of(index, |_| true);
            json!({
                "net": view.net(index).name,
                "labels": items.iter().map(|&item| view.item_json(item, detail)).collect::<Vec<_>>(),
            })
        })
        .collect();

    // The secondary lists are long on a real design — a generated schematic
    // uses a global label where a local one would do, and every divider and
    // strap is biased — so inline they are named and counted, and the output
    // file carries each in full.
    let single_entry = |(name, item): &(String, Item)| -> Value {
        let nets = view.resolved.nets_of(*item);
        let net = nets.first().map(|&net| view.net(net).name.clone());
        let pins = nets.first().map(|&net| view.nodes(net).len()).unwrap_or(0);
        if full {
            let mut value = view.item_json(*item, detail);
            value["net"] = json!(net);
            value["net_pin_count"] = json!(pins);
            value
        } else {
            let (_, sheet_file, _) = view.instance_sheet(item.instance());
            json!({ "text": name, "sheet_file": sheet_file, "net_pin_count": pins })
        }
    };
    let biased_entry = |&index: &usize| -> Value {
        let drive = view.drive_of(index);
        let mut value = json!({
            "net": view.net(index).name,
            "level": view.level_json(&drive),
        });
        if full {
            value["inputs"] = json!(view
                .sorted_nodes(index)
                .iter()
                .filter(|node| matches!(node.electrical_type.as_str(), "input" | "power_in"))
                .map(|node| view.pin_tag(node))
                .collect::<Vec<_>>());
            value["pins"] = json!(view
                .sorted_nodes(index)
                .iter()
                .map(|node| view.pin_json(node, Detail::Full))
                .collect::<Vec<_>>());
            value["drive"] = view.drive_json(index, &drive, Detail::Full);
        }
        value
    };
    let indeterminate_entry = |&index: &usize| -> Value {
        if full {
            json!({
                "net": view.net(index).name,
                "pins": view.sorted_nodes(index).iter().map(|node| view.pin_tag(node)).collect::<Vec<_>>(),
                "drive": view.drive_json(index, &view.drive_of(index), Detail::Full),
            })
        } else {
            json!(view.net(index).name)
        }
    };
    let limit = if full { usize::MAX } else { SUMMARY_LIST_LIMIT };
    let (singles, singles_more) = capped_to(report.single_globals.iter().map(single_entry), limit);
    let (biased, biased_more) = capped_to(report.biased.iter().map(biased_entry), limit);
    let (indeterminate, indeterminate_more) =
        capped_to(report.indeterminate.iter().map(indeterminate_entry), limit);

    let mut value = json!({
        "undriven_net_count": report.undriven.len(),
        "undriven_nets": report.undriven.iter().map(|&index| undriven_net_json(view, index, detail)).collect::<Vec<_>>(),
        "label_only_net_count": label_only.len(),
        "label_only_nets": label_only,
        "single_global_label_count": report.single_globals.len(),
        "single_global_labels": singles,
        "biased_net_count": report.biased.len(),
        "biased_nets": biased,
        "indeterminate_net_count": report.indeterminate.len(),
        "indeterminate_nets": indeterminate,
        "skipped": {
            "unlabelled_single_pin_nets": report.skipped_unconnected,
            "no_connect_nets": report.skipped_no_connect,
        },
    });
    let not_listed: BTreeMap<&str, usize> = [
        ("single_global_labels", singles_more),
        ("biased_nets", biased_more),
        ("indeterminate_nets", indeterminate_more),
    ]
    .into_iter()
    .filter(|(_, more)| *more > 0)
    .collect();
    if !not_listed.is_empty() {
        value["not_listed_inline"] = json!(not_listed);
    }
    value
}

const UNDRIVEN_RULE: &str = "A net is driven by an output, power-out, tri-state, bidirectional, \
     open-collector or open-emitter pin or a PWR_FLAG on it; by a connector pin (J, P, CN, X); \
     or through two-pin series parts (R, Y, L, FB, F, JP, NT) from a push-pull output or a \
     connector, or from a supply through L, FB, F, JP or NT alone. A resistor path to a supply \
     only biases a net: 'biased_nets' have a pull-up or pull-down holding a level and nothing \
     driving them. A pin that may be driven inside its part (an IC's passive pin, a free or \
     unspecified pin) makes a net 'indeterminate'. 'undriven_nets' are the rest that have pins: \
     dead rails and floating inputs. A path ends at a supply rather than running through it.";

async fn handle_find_undriven_nets(args: &Value, ctx: &ToolContext) -> Result<CallToolResult> {
    let output = match output_path(args) {
        Ok(output) => output,
        Err(error) => return Ok(error),
    };
    let (root, design) = match load_design(args, ctx).await? {
        Ok(loaded) => loaded,
        Err(refusal) => return Ok(refusal),
    };
    let none = HashSet::new();
    let resolved = resolve(&design.netlist, &design.hierarchy, &none);
    let view = View::new(&design.netlist, &design.hierarchy, &resolved);
    let report = undriven_report(&view);
    let mut response = undriven_json(&view, &report, Detail::Compact);
    response["rule"] = json!(UNDRIVEN_RULE);
    response["source"] = source_json(&root, &design, &resolved);
    if let Some(path) = output {
        let mut full = undriven_json(&view, &report, Detail::Full);
        full["rule"] = json!(UNDRIVEN_RULE);
        full["source"] = response["source"].clone();
        response["output"] = write_output(&path, &full).await?;
    }
    Ok(CallToolResult::json(&response))
}

// ─── Sheet removal ────────────────────────────────────────────────────────────

/// A `(sheet …)` block to delete: the file that holds it and its UUID.
#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord)]
struct BlockRef {
    parent_file: usize,
    block_uuid: String,
}

struct Removal {
    blocks: Vec<BlockRef>,
    /// Every instance that disappears: the named ones, every other instance of
    /// the same block (a block in a reused file is in all its instances), and
    /// everything below them.
    instances: HashSet<usize>,
    named: Vec<(usize, String)>,
}

fn block_of(hierarchy: &Hierarchy, instance: usize) -> Option<BlockRef> {
    let sheet = &hierarchy.instances[instance];
    let parent = sheet.parent?;
    let block = sheet.block?;
    let parent_file = hierarchy.instances[parent].file;
    Some(BlockRef {
        parent_file,
        block_uuid: hierarchy.files[parent_file].blocks[block].uuid.clone(),
    })
}

/// Resolve the requested sheets to blocks, refusing a request that is not one
/// unambiguous block.
fn plan_removal(
    hierarchy: &Hierarchy,
    requested: &[String],
) -> std::result::Result<Removal, CallToolResult> {
    let mut blocks: BTreeSet<BlockRef> = BTreeSet::new();
    let mut named = Vec::new();
    for request in requested {
        let trimmed = request.trim();
        let as_path = if trimmed.ends_with('/') {
            trimmed.to_owned()
        } else {
            format!("{trimmed}/")
        };
        // By path first, then by name, then by file: the first rule that
        // matches anything decides.
        let matchers: [fn(&SheetInstance, &str, &str) -> bool; 3] = [
            |sheet, request, as_path| sheet.names_path == request || sheet.names_path == as_path,
            |sheet, request, _| sheet.sheet_name == request,
            |sheet, request, _| {
                sheet.sheet_file == request
                    || Path::new(&sheet.sheet_file)
                        .file_name()
                        .is_some_and(|name| name.to_string_lossy() == request)
            },
        ];
        let candidates: Vec<usize> = matchers
            .iter()
            .map(|matches| {
                (1..hierarchy.instances.len())
                    .filter(|&index| matches(&hierarchy.instances[index], trimmed, &as_path))
                    .collect::<Vec<_>>()
            })
            .find(|found| !found.is_empty())
            .unwrap_or_default();
        if candidates.is_empty() {
            let mut available: Vec<String> = hierarchy.instances[1..]
                .iter()
                .map(|sheet| sheet.names_path.clone())
                .collect();
            available.sort();
            available.truncate(40);
            return Err(invalid_arg(
                "sheet_names",
                &format!(
                    "no sheet is named, located at or filed as '{request}'. Sheets include: {}",
                    available.join(", ")
                ),
            ));
        }
        let found: BTreeSet<BlockRef> = candidates
            .iter()
            .filter_map(|&index| block_of(hierarchy, index))
            .collect();
        if found.len() > 1 {
            return Err(CallToolResult::error_kind(
                ToolErrorKind::AmbiguousTarget {
                    target: request.clone(),
                    candidates: candidates
                        .iter()
                        .map(|&index| hierarchy.instances[index].names_path.clone())
                        .collect(),
                },
                format!(
                    "'{request}' matches sheet blocks in more than one place: {}. Name one by its \
                     sheet path.",
                    candidates
                        .iter()
                        .map(|&index| hierarchy.instances[index].names_path.as_str())
                        .collect::<Vec<_>>()
                        .join(", ")
                ),
            ));
        }
        for &index in &candidates {
            named.push((index, request.clone()));
        }
        blocks.extend(found);
    }

    let direct: Vec<usize> = (1..hierarchy.instances.len())
        .filter(|&index| block_of(hierarchy, index).is_some_and(|block| blocks.contains(&block)))
        .collect();
    let instances = (0..hierarchy.instances.len())
        .filter(|&index| direct.iter().any(|&top| hierarchy.is_within(index, top)))
        .collect();
    Ok(Removal {
        blocks: blocks.into_iter().collect(),
        instances,
        named,
    })
}

/// Copy the hierarchy into `staging`, keeping every file's place relative to
/// the others, with the removed blocks deleted from their parents' copies.
/// Returns the copy of the root.
fn stage_without_blocks(
    hierarchy: &Hierarchy,
    removal: &Removal,
    staging: &Path,
) -> Result<PathBuf> {
    let root = canonical(&hierarchy.root);
    let sources: Vec<PathBuf> = hierarchy
        .files
        .iter()
        .map(|file| canonical(&file.path))
        .collect();
    let project = hierarchy.root.with_extension("kicad_pro");
    let project = project.is_file().then(|| canonical(&project));
    let mut common = root
        .parent()
        .map(Path::to_path_buf)
        .context("the root schematic has no parent directory")?;
    for source in sources.iter().chain(project.iter()) {
        while !source.starts_with(&common) {
            common = common
                .parent()
                .map(Path::to_path_buf)
                .context("the hierarchy's files share no common directory")?;
        }
    }
    for instance in &hierarchy.instances[1..] {
        if Path::new(&instance.sheet_file).is_absolute() {
            bail!(
                "sheet {} names its file by an absolute path ({}); the dry run copies the \
                 hierarchy and cannot keep such a reference inside the copy",
                instance.names_path,
                instance.sheet_file
            );
        }
    }
    let destination = |source: &Path| -> Result<PathBuf> {
        let relative = source
            .strip_prefix(&common)
            .context("a hierarchy file lies outside the common directory")?;
        Ok(staging.join(relative))
    };

    for (index, source) in sources.iter().enumerate() {
        let mut content = konnect_sexp::read_consistent(source)
            .with_context(|| format!("cannot read {}", source.display()))?;
        for block in removal
            .blocks
            .iter()
            .filter(|block| block.parent_file == index)
        {
            content = remove_sheet_block(&content, &block.block_uuid).with_context(|| {
                format!(
                    "cannot remove sheet block {} from the copy of {}",
                    block.block_uuid,
                    source.display()
                )
            })?;
        }
        let target = destination(source)?;
        if let Some(parent) = target.parent() {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::write(&target, content)?;
    }
    if let Some(project) = project {
        let target = destination(&project)?;
        std::fs::copy(&project, target)?;
    }
    destination(&root)
}

/// Delete the top-level `(sheet …)` block whose `(uuid …)` is `uuid`, with
/// the whitespace before it.
fn remove_sheet_block(content: &str, uuid: &str) -> Result<String> {
    use konnect_sexp::writer::{find_block_with_leading_whitespace, find_direct_child_blocks};
    let mut hit = None;
    for (start, end) in find_direct_child_blocks(content, "kicad_sch") {
        let block = &content[start..end];
        if !block.starts_with("(sheet")
            || !block[6..].starts_with(|c: char| c.is_whitespace() || c == '(')
        {
            continue;
        }
        let node = parse_sexp(block).context("a sheet block is not a valid S-expression")?;
        if node.find_str("uuid") == Some(uuid) {
            if hit.is_some() {
                bail!("two sheet blocks carry the UUID {uuid}");
            }
            hit = Some(start);
        }
    }
    let start = hit.with_context(|| format!("no sheet block carries the UUID {uuid}"))?;
    let (from, to) = find_block_with_leading_whitespace(content, start)
        .context("the sheet block is not balanced")?;
    let mut edited = String::with_capacity(content.len());
    edited.push_str(&content[..from]);
    edited.push_str(&content[to..]);
    Ok(edited)
}

/// Every member of a net, split by whether the removal takes it.
struct Split {
    removed_nodes: Vec<usize>,
    kept_nodes: Vec<usize>,
    removed_items: Vec<Item>,
    kept_items: Vec<Item>,
}

fn split_members(before: &View, index: usize, after_netlist: &Netlist, removal: &Removal) -> Split {
    let mut split = Split {
        removed_nodes: Vec::new(),
        kept_nodes: Vec::new(),
        removed_items: Vec::new(),
        kept_items: Vec::new(),
    };
    for (position, node) in before.nodes(index).iter().enumerate() {
        if after_netlist.net_of(&node.reference, &node.pin).is_some() {
            split.kept_nodes.push(position);
        } else {
            split.removed_nodes.push(position);
        }
    }
    for &item in &before.net(index).items {
        let gone = removal.instances.contains(&item.instance())
            || matches!(item, Item::SheetPin(parent, block, _)
                if before.hierarchy.child_of(parent, block).is_some_and(|child| removal.instances.contains(&child)));
        if gone {
            split.removed_items.push(item);
        } else {
            split.kept_items.push(item);
        }
    }
    split.removed_items.sort();
    split.removed_items.dedup();
    split.kept_items.sort();
    split.kept_items.dedup();
    split
}

async fn handle_plan_sheet_removal(args: &Value, ctx: &ToolContext) -> Result<CallToolResult> {
    let requested = match opt_str_list(args, "sheet_names") {
        Ok(Some(names))
            if !names.is_empty() && names.iter().all(|name| !name.trim().is_empty()) =>
        {
            names
        }
        Ok(_) => {
            return Ok(invalid_arg(
                "sheet_names",
                "name at least one sheet, each a non-empty string",
            ))
        }
        Err(error) => return Ok(error),
    };
    let output = match output_path(args) {
        Ok(output) => output,
        Err(error) => return Ok(error),
    };
    let (root, design) = match load_design(args, ctx).await? {
        Ok(loaded) => loaded,
        Err(refusal) => return Ok(refusal),
    };
    let removal = match plan_removal(&design.hierarchy, &requested) {
        Ok(removal) => removal,
        Err(refusal) => return Ok(refusal),
    };

    let staging = tempfile::Builder::new()
        .prefix("konnect-sheet-removal-")
        .tempdir()
        .context("failed to create the dry-run directory")?;
    let staged_root = stage_without_blocks(&design.hierarchy, &removal, staging.path())?;
    let after_text = cli::export_netlist_text(&ctx.config.kicad_cli, &staged_root)
        .await
        .context("KiCad could not netlist the copy with the sheets removed")?;
    staging
        .close()
        .context("failed to clean up the dry-run directory")?;
    let after_netlist = Netlist::parse(&after_text)?;

    let none = HashSet::new();
    let before_resolved = resolve(&design.netlist, &design.hierarchy, &none);
    let after_resolved = resolve(&after_netlist, &design.hierarchy, &removal.instances);
    let before = View::new(&design.netlist, &design.hierarchy, &before_resolved);
    let after = View::new(&after_netlist, &design.hierarchy, &after_resolved);

    let report = removal_report(&before, &after, &after_netlist, &removal, &root, &design);
    let mut response = report.summary;
    if let Some(path) = output {
        response["output"] = write_output(&path, &report.full).await?;
    }
    Ok(CallToolResult::json(&response))
}

struct RemovalReport {
    summary: Value,
    full: Value,
}

/// Pins listed by name in an inline summary before the rest are counted.
const SUMMARY_PIN_LIMIT: usize = 25;

/// A healthy remainder larger than this keeps only its counts in the full
/// result: its members are the design's, not the removal's.
const FULL_MEMBER_LIMIT: usize = 200;

/// The first [`SUMMARY_PIN_LIMIT`] entries, and how many were left out.
fn capped(entries: impl Iterator<Item = String>) -> (Vec<String>, usize) {
    capped_to(entries, SUMMARY_PIN_LIMIT)
}

/// The first `limit` entries, and how many were left out.
fn capped_to<T>(entries: impl Iterator<Item = T>, limit: usize) -> (Vec<T>, usize) {
    let all: Vec<T> = entries.collect();
    let more = all.len().saturating_sub(limit);
    (all.into_iter().take(limit).collect(), more)
}

/// What the removal does to one remainder of a net.
///
/// Drive and bias are kept apart on purpose. A 10 kΩ pull-down left on an
/// input whose push-pull driver was removed holds a level but drives nothing,
/// so the input has lost its driver. The exception is an open drain: its
/// high level always came from the pull-up, so losing the open drain while
/// the pull-up stays leaves the input at the level it already read whenever
/// the open drain was released — `remains_defined`.
fn remainder_verdict(before: &Drive, after: &Drive) -> &'static str {
    if after.status.is_driven() {
        "keeps_drive"
    } else if before.status.is_driven() {
        if after.status == DriveStatus::Biased && before.open_drain_only {
            "remains_defined"
        } else if after.status == DriveStatus::Indeterminate {
            "indeterminate"
        } else {
            "loses_drive"
        }
    } else if before.status == DriveStatus::Biased
        && matches!(
            after.status,
            DriveStatus::Undriven | DriveStatus::Indeterminate | DriveStatus::LabelOnly
        )
    {
        "loses_bias"
    } else {
        "had_no_drive"
    }
}

/// Worst first: a net reports the worst of its remainders, and the report
/// lists the worst nets first.
fn verdict_severity(verdict: &str) -> u8 {
    match verdict {
        "loses_drive" => 0,
        "loses_bias" => 1,
        "indeterminate" => 2,
        "remains_defined" => 3,
        "had_no_drive" => 4,
        _ => 5,
    }
}

/// The findings one removal produces, gathered across every affected net.
#[derive(Default)]
struct Findings {
    inputs_losing_driver: Vec<Value>,
    dead_remainders: Vec<Value>,
    single_pin: Vec<Value>,
    single_label: Vec<Value>,
    open_drain: Vec<Value>,
    open_drain_seen: HashSet<usize>,
}

/// One net touched by the removal, before and after.
struct AffectedNet {
    summary: Value,
    full: Value,
    verdict: &'static str,
}

/// Describe how one before-net's surviving members fare after the removal,
/// recording what it contributes to `findings`.
#[allow(clippy::too_many_arguments)]
fn affected_net(
    before: &View,
    after: &View,
    after_netlist: &Netlist,
    index: usize,
    split: &Split,
    crossing: bool,
    findings: &mut Findings,
) -> AffectedNet {
    let net_name = before.net(index).name.clone();
    let nodes = before.nodes(index);
    let before_drive = before.drive_of(index);

    let mut remainders: BTreeSet<usize> = BTreeSet::new();
    for &position in &split.kept_nodes {
        let node = &nodes[position];
        if let Some(net) = after_netlist.net_of(&node.reference, &node.pin) {
            remainders.insert(net);
        }
    }
    for &item in &split.kept_items {
        remainders.extend(after.resolved.nets_of(item).iter().copied());
    }

    let mut worst = "keeps_drive";
    let mut summaries = Vec::new();
    let mut fulls = Vec::new();
    for remainder in remainders {
        let drive = after.drive_of(remainder);
        let verdict = remainder_verdict(&before_drive, &drive);
        if verdict_severity(verdict) < verdict_severity(worst) {
            worst = verdict;
        }
        let after_nodes = after.sorted_nodes(remainder);
        let named = after.items_of(remainder, |item| {
            matches!(item, Item::Label(..) | Item::Power(..))
        });
        let net_after = after.net(remainder).name.clone();
        let reduced_to = match (after_nodes.len(), named.len()) {
            (1, _) => Some("single_pin"),
            (0, 1) => Some("single_label"),
            (0, _) => Some("labels_only"),
            _ => None,
        };
        let lost = match verdict {
            // An indeterminate remainder had a driver and now has only a pin
            // that may or may not drive it: its pins are reported, with the
            // level saying why it is not a certainty.
            "loses_drive" | "indeterminate" => Some("drive"),
            "loses_bias" => Some("bias"),
            _ => None,
        };

        // Inputs, and the passive pins of ICs, which on a net that has lost
        // its drive are inputs too: an analog switch, an ADC channel.
        let stranded: Vec<&NetlistNode> = if lost.is_some() {
            after_nodes
                .iter()
                .copied()
                .filter(|node| {
                    matches!(node.electrical_type.as_str(), "input" | "power_in")
                        || after.drive.is_opaque(node)
                })
                .collect()
        } else {
            Vec::new()
        };
        let orphaned: Vec<&NetlistNode> = if lost.is_some() && drive.status == DriveStatus::Undriven
        {
            after_nodes
                .iter()
                .copied()
                .filter(|node| node.electrical_type == "passive")
                .collect()
        } else {
            Vec::new()
        };
        let level = after.level_json(&drive);
        for node in &stranded {
            let mut pin = after.pin_json(node, Detail::Compact);
            pin["net"] = json!(net_name);
            pin["net_after"] = json!(net_after);
            pin["lost"] = json!(lost);
            pin["level_after"] = level.clone();
            if after.drive.is_opaque(node) {
                pin["ic_passive_pin"] = json!(true);
            }
            findings.inputs_losing_driver.push(pin);
        }
        if !orphaned.is_empty() {
            findings.dead_remainders.push(json!({
                "net": net_name,
                "net_after": net_after,
                "inputs": stranded.iter().map(|node| after.pin_tag(node)).collect::<Vec<_>>(),
                "passives": orphaned.iter().map(|node| after.pin_tag(node)).collect::<Vec<_>>(),
            }));
            if let Some(after_index) = after.net(remainder).netlist {
                for (open_net, hop) in after.drive.open_drain_losing_pull_up(after_index) {
                    if !findings.open_drain_seen.insert(open_net) {
                        continue;
                    }
                    let open_nodes = &after_netlist.nets[open_net].nodes;
                    findings.open_drain.push(json!({
                        "net": after_netlist.nets[open_net].name,
                        "pull_up": after.hop_json(&hop),
                        "dead_rail": net_after,
                        "open_drain_pins": open_nodes.iter().filter(|node| matches!(node.electrical_type.as_str(), "open_collector" | "open_emitter")).map(|node| after.pin_tag(node)).collect::<Vec<_>>(),
                        "inputs": open_nodes.iter().filter(|node| matches!(node.electrical_type.as_str(), "input" | "power_in")).map(|node| after.pin_tag(node)).collect::<Vec<_>>(),
                    }));
                }
            }
        }
        match reduced_to {
            Some("single_pin") => findings.single_pin.push(json!({
                "net": net_name,
                "net_after": net_after,
                "pin": after_nodes.first().map(|node| after.pin_tag(node)),
                "drive_after": drive.status.as_str(),
            })),
            Some("single_label") => findings.single_label.push(json!({
                "net": net_name,
                "net_after": net_after,
                "label": named.first().map(|&item| after.item_json(item, Detail::Compact)),
            })),
            _ => {}
        }

        // A rail that keeps its drive is listed by count: GND alone remains
        // on well over a thousand pins in a real design.
        let healthy = verdict == "keeps_drive";
        let mut summary = json!({
            "net_after": net_after,
            "verdict": verdict,
            "drive_after": drive.status.as_str(),
            "pin_count": after_nodes.len(),
            "labels": named.len(),
            "reduced_to": reduced_to,
        });
        if after.net(remainder).netlist.is_none() {
            summary["kicad_net"] = json!(false);
        }
        if drive.status == DriveStatus::DrivenThroughSeries {
            summary["driven_by"] = json!(drive.reaches);
        }
        if !healthy {
            let (pins, more) = capped(after_nodes.iter().map(|node| after.pin_tag(node)));
            summary["pins"] = json!(pins);
            if more > 0 {
                summary["pins_not_listed"] = json!(more);
            }
            summary["level_after"] = level;
        }
        summaries.push(summary);
        let members = !healthy || after_nodes.len() <= FULL_MEMBER_LIMIT;
        let mut full = after.net_json(remainder, Detail::Full, members);
        if !members {
            full["members_omitted"] = json!(format!(
                "keeps its drive; {} pins remain",
                after_nodes.len()
            ));
        }
        full["verdict"] = json!(verdict);
        full["reduced_to"] = json!(reduced_to);
        full["stranded_inputs"] = json!(stranded
            .iter()
            .map(|node| after.pin_json(node, Detail::Full))
            .collect::<Vec<_>>());
        full["orphaned_passives"] = json!(orphaned
            .iter()
            .map(|node| after.pin_json(node, Detail::Full))
            .collect::<Vec<_>>());
        fulls.push(full);
    }

    let lost_sources: Vec<usize> = before_drive
        .sources
        .iter()
        .copied()
        .filter(|source| split.removed_nodes.contains(source))
        .collect();
    let (removed_pins, removed_not_listed) = capped(
        split
            .removed_nodes
            .iter()
            .map(|&position| before.pin_tag(&nodes[position])),
    );
    let mut summary = json!({
        "net": net_name,
        "verdict": worst,
        "drive_before": before_drive.status.as_str(),
        "remainders": summaries,
    });
    let mut full = json!({
        "net": net_name,
        "verdict": worst,
        "drive_before": before.drive_json(index, &before_drive, Detail::Full),
        "remainders": fulls,
    });
    if crossing {
        summary["lost_sources"] = json!(lost_sources
            .iter()
            .map(|&position| before.pin_tag(&nodes[position]))
            .collect::<Vec<_>>());
        summary["lost_power_flags"] = json!(split
            .removed_items
            .iter()
            .filter(|item| matches!(item, Item::Flag(..)))
            .count());
        summary["removed_pin_count"] = json!(split.removed_nodes.len());
        summary["removed_pins"] = json!(removed_pins);
        summary["removed_pins_not_listed"] = json!(removed_not_listed);
        summary["removed_labels"] = json!(split
            .removed_items
            .iter()
            .filter(|item| matches!(item, Item::Label(..) | Item::Power(..)))
            .count());
        full["lost_sources"] = json!(lost_sources
            .iter()
            .map(|&position| before.pin_json(&nodes[position], Detail::Full))
            .collect::<Vec<_>>());
        full["removed_pins"] = json!(split
            .removed_nodes
            .iter()
            .map(|&position| before.pin_json(&nodes[position], Detail::Full))
            .collect::<Vec<_>>());
        full["removed_items"] = json!(split
            .removed_items
            .iter()
            .map(|&item| before.item_json(item, Detail::Full))
            .collect::<Vec<_>>());
    } else {
        // Nothing of this net was on the removed sheets; it was fed through
        // a series part from a net that was.
        summary["fed_before_by"] = json!(before_drive.reaches);
        summary["fed_before_via"] = json!(before_drive
            .via
            .iter()
            .map(|hop| before.hop_json(hop))
            .collect::<Vec<_>>());
    }
    AffectedNet {
        summary,
        full,
        verdict: worst,
    }
}

fn removal_report(
    before: &View,
    after: &View,
    after_netlist: &Netlist,
    removal: &Removal,
    root: &Path,
    design: &Design,
) -> RemovalReport {
    let hierarchy = before.hierarchy;

    // Components KiCad dropped, against those the removed sheets held.
    let after_refs: HashSet<&str> = after_netlist
        .components
        .iter()
        .map(|component| component.reference.as_str())
        .collect();
    let mut dropped: Vec<&str> = before
        .netlist
        .components
        .iter()
        .map(|component| component.reference.as_str())
        .filter(|reference| !after_refs.contains(reference))
        .collect();
    dropped.sort_by(|a, b| natural_cmp(a, b));
    let expected: BTreeSet<&str> = before
        .resolved
        .pin_instance
        .iter()
        .filter(|(_, instance)| removal.instances.contains(instance))
        .map(|((reference, _), _)| reference.as_str())
        .collect();
    let unexpected: Vec<&str> = dropped
        .iter()
        .copied()
        .filter(|reference| !expected.contains(reference))
        .collect();
    let kept_anyway: Vec<&str> = expected
        .iter()
        .copied()
        .filter(|reference| after_refs.contains(reference))
        .collect();

    let mut findings = Findings::default();
    let mut crossing: Vec<AffectedNet> = Vec::new();
    let mut indirect: Vec<AffectedNet> = Vec::new();
    let mut internal = Vec::new();

    let mut order: Vec<usize> = (0..before.resolved.nets.len()).collect();
    order.sort_by(|&a, &b| natural_cmp(&before.net(a).name, &before.net(b).name));
    for index in order {
        let split = split_members(before, index, after_netlist, removal);
        let removed_any = !split.removed_nodes.is_empty() || !split.removed_items.is_empty();
        let kept_any = !split.kept_nodes.is_empty() || !split.kept_items.is_empty();
        if removed_any && !kept_any {
            internal.push(json!({
                "net": before.net(index).name,
                "pins": before.nodes(index).len(),
                "kicad_net": before.net(index).netlist.is_some(),
            }));
            continue;
        }
        if removed_any {
            crossing.push(affected_net(
                before,
                after,
                after_netlist,
                index,
                &split,
                true,
                &mut findings,
            ));
            continue;
        }
        // Untouched itself, but fed or biased through a series part from a
        // net that was: a filter or divider behind a removed connector.
        let before_status = before.drive_of(index).status;
        if !matches!(
            before_status,
            DriveStatus::DrivenThroughSeries | DriveStatus::Biased
        ) {
            continue;
        }
        let entry = affected_net(
            before,
            after,
            after_netlist,
            index,
            &split,
            false,
            &mut findings,
        );
        if !matches!(entry.verdict, "keeps_drive" | "had_no_drive") {
            indirect.push(entry);
        }
    }

    let sort = |entries: &mut Vec<AffectedNet>| {
        entries.sort_by(|a, b| {
            verdict_severity(a.verdict)
                .cmp(&verdict_severity(b.verdict))
                .then_with(|| {
                    natural_cmp(
                        a.summary["net"].as_str().unwrap_or(""),
                        b.summary["net"].as_str().unwrap_or(""),
                    )
                })
        });
    };
    sort(&mut crossing);
    sort(&mut indirect);

    let removed_sheets: Vec<Value> = removal
        .instances
        .iter()
        .copied()
        .collect::<BTreeSet<_>>()
        .into_iter()
        .map(|index| {
            let sheet = &hierarchy.instances[index];
            let requested_as = removal
                .named
                .iter()
                .find(|(named, _)| *named == index)
                .map(|(_, request)| request.clone());
            let reason = if requested_as.is_some() {
                "named"
            } else if removal
                .named
                .iter()
                .any(|(named, _)| hierarchy.is_within(index, *named))
            {
                "below_a_named_sheet"
            } else {
                "same_block_in_a_reused_parent"
            };
            json!({
                "sheet_path": sheet.names_path,
                "sheet_name": sheet.sheet_name,
                "sheet_file": sheet.sheet_file,
                "page": sheet.page,
                "parent_sheet_path": sheet.parent.map(|parent| hierarchy.instances[parent].names_path.clone()),
                "requested_as": requested_as,
                "reason": reason,
            })
        })
        .collect();

    let findings_json = json!({
        "inputs_losing_driver": findings.inputs_losing_driver,
        "dead_remainders": findings.dead_remainders,
        "single_pin_remainders": findings.single_pin,
        "single_label_remainders": findings.single_label,
        "open_drain_nets_losing_pull_up": findings.open_drain,
    });
    let verification = json!({
        "components_removed_by_kicad": dropped.len(),
        "components_on_removed_sheets": expected.len(),
        "agrees": unexpected.is_empty() && kept_anyway.is_empty(),
        "removed_but_not_on_removed_sheets": unexpected,
        "on_removed_sheets_but_kept": kept_anyway,
    });
    let mut source = source_json(root, design, before.resolved);
    source["after"] = json!({
        "kicad_tool": after_netlist.tool,
        "disagreements": after.resolved.disagreements,
        "unmatched_pins": after.resolved.unmatched_pins,
    });
    let header = |crossing_nets: Value, indirect_nets: Value, internal_nets: Value| {
        json!({
            "dry_run": true,
            "writes": "none: KiCad netlisted a temporary copy of the hierarchy with the sheet blocks removed",
            "removed_sheets": removed_sheets,
            "removed_component_count": dropped.len(),
            "removed_components": dropped,
            "verification": verification,
            "crossing_net_count": crossing.len(),
            "nets_losing_drive_indirectly_count": indirect.len(),
            "internal_net_count": internal.len(),
            "findings": findings_json,
            "crossing_nets": crossing_nets,
            "nets_losing_drive_indirectly": indirect_nets,
            "internal_nets": internal_nets,
            "verdicts": VERDICTS,
            "source": source,
        })
    };
    let summary = header(
        json!(crossing.iter().map(|net| &net.summary).collect::<Vec<_>>()),
        json!(indirect.iter().map(|net| &net.summary).collect::<Vec<_>>()),
        json!(internal
            .iter()
            .map(|net| net["net"].clone())
            .collect::<Vec<_>>()),
    );
    let full = header(
        json!(crossing.iter().map(|net| &net.full).collect::<Vec<_>>()),
        json!(indirect.iter().map(|net| &net.full).collect::<Vec<_>>()),
        json!(internal),
    );
    RemovalReport { summary, full }
}

/// The verdict vocabulary, stated in every removal report.
const VERDICTS: &str = "keeps_drive: something still drives the remainder. remains_defined: an \
     open-drain source was removed and its pull-up stays, so the input reads the released level. \
     loses_drive: the remainder had a driver and now has only a pull resistor, or nothing. \
     loses_bias: the remainder had only a pull resistor, and that is gone too. indeterminate: \
     only a pin that may be driven inside its part is left. had_no_drive: nothing drove it \
     before either.";

#[cfg(test)]
#[path = "design_nets_tests.rs"]
mod tests;
