//! `audit_manufacturing`'s check that a placed footprint's child zones are
//! where the footprint is.
//!
//! A footprint can carry zones of its own: the keepout under a Tag-Connect
//! header or an antenna, the copper of a net tie. KiCad saves them in board
//! coordinates (see `pcb_components::child_zones`), so an edit of the saved
//! file that moves the footprint's root `(at …)` and nothing else leaves the
//! zone behind. Such a board loads, and KiCad's DRC says nothing: the keepout
//! simply keeps nothing out under the part it belongs to.
//!
//! Two tests, in this order, and each finding says which one made it:
//!
//! - `library_outline`: the library footprint resolves and holds a zone of
//!   the same shape. The zone must lie where that outline lands at the
//!   instance's position, rotation and side. A zone the library does not
//!   describe (edited on the board, or a library that has changed since) is
//!   not judged by this test: a different shape is not a misplaced one.
//! - `footprint_extent`: otherwise, the zone must come within reach of the
//!   footprint. It is reported only when its bounding box lies further from
//!   the footprint's position than everything else the footprint holds, by a
//!   margin of at least [`EXTENT_MARGIN_MM`]. No footprint of KiCad's stock
//!   library comes near that at any rotation or on either side.

use crate::tools::pcb_components::child_zones::{
    distance, outlines, placed_point, same_place, same_shape, xy, Outlines, Point,
};
use crate::tools::pcb_components::resolve_footprint_source;
use konnect_sexp::SexpNode;
use serde_json::{json, Value};
use std::collections::HashMap;
use std::path::Path;

/// The `kind` of every finding of this check.
pub(super) const KIND: &str = "misplaced_footprint_zone";
/// The least margin (mm) of the `footprint_extent` test.
const EXTENT_MARGIN_MM: f64 = 10.0;
/// An expected outline longer than this is counted, not listed.
const LISTED_POINTS: usize = 32;

/// What the check covered, and what it found.
#[derive(Default)]
pub(super) struct ZoneCheck {
    /// Child zones judged, by either test.
    pub checked: usize,
    pub by_library_outline: usize,
    pub by_footprint_extent: usize,
    /// Child zones with no readable outline, or on a footprint with no
    /// readable position.
    pub unreadable: usize,
    pub findings: Vec<Value>,
}

impl ZoneCheck {
    pub(super) fn coverage(&self) -> Value {
        json!({
            "checked": self.checked,
            "by_library_outline": self.by_library_outline,
            "by_footprint_extent": self.by_footprint_extent,
            "unreadable": self.unreadable,
            "misplaced": self.findings.len(),
        })
    }
}

#[derive(Clone, Copy)]
struct BBox {
    x1: f64,
    y1: f64,
    x2: f64,
    y2: f64,
}

impl BBox {
    fn of(outlines: &Outlines) -> BBox {
        let mut b = BBox {
            x1: f64::INFINITY,
            y1: f64::INFINITY,
            x2: f64::NEG_INFINITY,
            y2: f64::NEG_INFINITY,
        };
        for (x, y) in outlines.iter().flatten() {
            b.x1 = b.x1.min(*x);
            b.y1 = b.y1.min(*y);
            b.x2 = b.x2.max(*x);
            b.y2 = b.y2.max(*y);
        }
        b
    }

    fn center(&self) -> Point {
        ((self.x1 + self.x2) / 2.0, (self.y1 + self.y2) / 2.0)
    }

    /// Distance from `p` to the box; zero inside it.
    fn distance_to(&self, p: Point) -> f64 {
        let dx = (self.x1 - p.0).max(p.0 - self.x2).max(0.0);
        let dy = (self.y1 - p.1).max(p.1 - self.y2).max(0.0);
        dx.hypot(dy)
    }

    fn json(&self) -> Value {
        json!({ "x1": round(self.x1), "y1": round(self.y1), "x2": round(self.x2), "y2": round(self.y2) })
    }
}

fn round(value: f64) -> f64 {
    let rounded = (value * 1e6).round() / 1e6;
    if rounded == 0.0 {
        0.0
    } else {
        rounded
    }
}

fn point_json(p: Point) -> Value {
    json!({ "x": round(p.0), "y": round(p.1) })
}

/// How far (mm) anything a footprint holds, other than its zones, reaches
/// from the footprint's position. Everything but a zone is saved relative to
/// that position, so the reach holds at any rotation and on either side.
fn reach(node: &SexpNode, base: f64) -> f64 {
    let placed = matches!(node.head(), Some("footprint" | "pad"));
    let mut far = base;
    for child in node.children().unwrap_or_default() {
        let Some(head) = child.head() else { continue };
        match head {
            "zone" => {}
            // The footprint's own position, on the board; a pad's is in `base`.
            "at" if placed => {}
            // A pad's own shapes are saved relative to the pad.
            "pad" => {
                let at = child.find("at").and_then(xy).unwrap_or((0.0, 0.0));
                let size = child.find("size").and_then(xy).unwrap_or((0.0, 0.0));
                let centre = base + at.0.hypot(at.1);
                far = far.max(reach(child, centre) + size.0.hypot(size.1) / 2.0);
            }
            "fp_circle" | "gr_circle" => {
                if let (Some(c), Some(e)) = (
                    child.find("center").and_then(xy),
                    child.find("end").and_then(xy),
                ) {
                    far = far.max(base + c.0.hypot(c.1) + distance(c, e));
                }
            }
            "at" | "start" | "mid" | "end" | "center" | "xy" => {
                if let Some(p) = xy(child) {
                    far = far.max(base + p.0.hypot(p.1));
                }
            }
            _ => far = far.max(reach(child, base)),
        }
    }
    far
}

fn reference(footprint: &SexpNode) -> String {
    footprint
        .find_all("property")
        .into_iter()
        .find(|p| p.get(1).and_then(SexpNode::as_str) == Some("Reference"))
        .and_then(|p| p.get(2))
        .or_else(|| {
            footprint
                .find_all("fp_text")
                .into_iter()
                .find(|t| t.get(1).and_then(SexpNode::as_str) == Some("reference"))
                .and_then(|t| t.get(2))
        })
        .and_then(SexpNode::as_str)
        .unwrap_or_default()
        .to_string()
}

fn zone_layers(zone: &SexpNode) -> Vec<String> {
    zone.find("layers")
        .or_else(|| zone.find("layer"))
        .and_then(SexpNode::children)
        .unwrap_or_default()
        .iter()
        .skip(1)
        .filter_map(SexpNode::as_str)
        .map(str::to_string)
        .collect()
}

/// The zones of the library footprint `lib_id`, in the footprint's own frame.
fn library_zones(lib_id: &str, board: &Path) -> Result<Vec<Outlines>, String> {
    let source = resolve_footprint_source(lib_id, board).map_err(|error| format!("{error:#}"))?;
    let root = konnect_sexp::parse_sexp(&source).map_err(|error| error.to_string())?;
    Ok(root
        .find_all("zone")
        .into_iter()
        .filter_map(outlines)
        .collect())
}

/// Check every child zone of every footprint of the board `tree`, read from
/// the file `board` (whose directory holds the project's library table).
pub(super) fn check(tree: &SexpNode, board: &Path) -> ZoneCheck {
    let mut out = ZoneCheck::default();
    let mut libraries: HashMap<String, Result<Vec<Outlines>, String>> = HashMap::new();
    for footprint in tree.find_all("footprint") {
        let zones = footprint.find_all("zone");
        if zones.is_empty() {
            continue;
        }
        let Some(at) = footprint.find("at") else {
            out.unreadable += zones.len();
            continue;
        };
        let Some(position) = xy(at) else {
            out.unreadable += zones.len();
            continue;
        };
        let rotation = at.get_f64(3).unwrap_or(0.0);
        let back = footprint
            .find_str("layer")
            .is_some_and(|layer| layer.starts_with("B."));
        let lib_id = footprint
            .get(1)
            .and_then(SexpNode::as_str)
            .unwrap_or_default();
        let library = libraries
            .entry(lib_id.to_string())
            .or_insert_with(|| library_zones(lib_id, board));
        let expected: Result<Vec<(Outlines, Outlines)>, String> = match library {
            Ok(zones) => Ok(zones
                .iter()
                .map(|local| {
                    let placed = local
                        .iter()
                        .map(|outline| {
                            outline
                                .iter()
                                .map(|p| placed_point(*p, position, rotation, back))
                                .collect()
                        })
                        .collect();
                    (local.clone(), placed)
                })
                .collect()),
            Err(reason) => Err(format!(
                "the library footprint could not be read ({reason})"
            )),
        };
        let extent = reach(footprint, 0.0);
        let within = extent + extent.max(EXTENT_MARGIN_MM);

        for (index, zone) in zones.into_iter().enumerate() {
            let Some(outline) = outlines(zone) else {
                out.unreadable += 1;
                continue;
            };
            out.checked += 1;
            let bbox = BBox::of(&outline);
            let away = bbox.distance_to(position);
            let beyond_extent = away > within;

            // By the library: in place, or the library's shape somewhere else.
            let by_library = expected.as_ref().ok().and_then(|expected| {
                if expected
                    .iter()
                    .any(|(_, placed)| same_place(&outline, placed))
                {
                    return Some(None);
                }
                let moved = |(local, _): &&(Outlines, Outlines)| same_shape(&outline, local);
                expected
                    .get(index)
                    .filter(moved)
                    .or_else(|| expected.iter().find(moved))
                    .map(|(_, placed)| Some(placed))
            });
            let (test, placed, why_not_library) = match by_library {
                Some(None) => {
                    out.by_library_outline += 1;
                    continue;
                }
                Some(Some(placed)) => {
                    out.by_library_outline += 1;
                    ("library_outline", Some(placed), None)
                }
                None => {
                    out.by_footprint_extent += 1;
                    if !beyond_extent {
                        continue;
                    }
                    let why = match &expected {
                        Err(reason) => reason.clone(),
                        Ok(_) => "the library footprint holds no zone of this shape".to_string(),
                    };
                    ("footprint_extent", None, Some(why))
                }
            };

            let name = reference(footprint);
            let layers = zone_layers(zone);
            let kind = if zone.find("keepout").is_some() {
                "rule_area"
            } else {
                "copper"
            };
            let uuid = zone
                .find_str("uuid")
                .or_else(|| zone.find_str("tstamp"))
                .unwrap_or_default();
            let center = bbox.center();
            let side = if back { "back" } else { "front" };
            let described = format!(
                "{name} ({lib_id}) is at ({}, {}) on the {side}, but its {} zone {uuid} on {} lies around ({}, {})",
                round(position.0),
                round(position.1),
                if kind == "rule_area" { "rule area (keepout)" } else { "copper" },
                layers.join(", "),
                round(center.0),
                round(center.1),
            );
            let (issue, expected_json, distance_mm) = match placed {
                Some(placed) => {
                    let want = BBox::of(placed);
                    let off = distance(center, want.center());
                    let points: usize = placed.iter().map(Vec::len).sum();
                    let mut expected_json = json!({
                        "center": point_json(want.center()),
                        "bbox": want.json(),
                        "points": points,
                    });
                    if points <= LISTED_POINTS {
                        expected_json["outline"] = json!(placed
                            .iter()
                            .map(|o| o.iter().map(|p| point_json(*p)).collect::<Vec<_>>())
                            .collect::<Vec<_>>());
                    }
                    (
                        format!(
                            "{described}, {} mm from where the library footprint puts it at this position, rotation and side, around ({}, {}).",
                            round_mm(off),
                            round(want.center().0),
                            round(want.center().1),
                        ),
                        expected_json,
                        off,
                    )
                }
                None => (
                    format!(
                        "{described}, {} mm from the footprint, whose own pads, graphics and texts reach {} mm from its position. Judged from the footprint's extent: {}.",
                        round_mm(away),
                        round_mm(extent),
                        why_not_library.unwrap_or_default(),
                    ),
                    json!({ "near": point_json(position), "within_mm": round_mm(within) }),
                    away,
                ),
            };
            out.findings.push(json!({
                // Off the footprint altogether, the zone does nothing it was
                // drawn for. Near it but not where the library puts it, it
                // may still be a deliberate edit.
                "severity": if beyond_extent { "error" } else { "warning" },
                "category": "dfm",
                "kind": KIND,
                "component": name,
                "issue": issue,
                "recommendation": if placed.is_some() {
                    format!(
                        "With the board closed in KiCad, edit_component with reference '{name}' and library_zones {{}} \
                         plans the move back to the footprint, and library_zones {{\"restore\": true}} writes it \
                         (run_closed_board_edit runs it on a board KiCad has open). In KiCad itself, \
                         Tools > Update Footprints from Library on {name} redraws the zone at the footprint. \
                         KiCad's DRC does not report a footprint zone left away from its footprint."
                    )
                } else {
                    format!(
                        "Open the board in KiCad and update {name} from its library (Tools > Update Footprints from Library), \
                         which redraws the zone at the footprint, then save. edit_component with library_zones can move \
                         the zone back only when the footprint's library resolves and holds a zone of this shape. \
                         KiCad's DRC does not report a footprint zone left away from its footprint."
                    )
                },
                "test": test,
                "footprint": {
                    "reference": name,
                    "lib_id": lib_id,
                    "uuid": footprint.find_str("uuid").or_else(|| footprint.find_str("tstamp")),
                    "x": round(position.0),
                    "y": round(position.1),
                    "rotation": rotation,
                    "side": side,
                },
                "zone": {
                    "uuid": uuid,
                    "kind": kind,
                    "layers": layers,
                    "center": point_json(center),
                    "bbox": bbox.json(),
                },
                "expected": expected_json,
                "distance_mm": round_mm(distance_mm),
            }));
        }
    }
    out
}

fn round_mm(value: f64) -> f64 {
    (value * 1e3).round() / 1e3
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::TempDir;

    /// The library footprint: one pad and an L-shaped keepout, so that no
    /// rotation or mirror of the outline is the outline again.
    const LIBRARY: &str = r#"(footprint "Keepout_L"
	(version 20241229)
	(generator "pcbnew")
	(layer "F.Cu")
	(property "Reference" "REF**" (at 0 -2 0) (layer "F.SilkS"))
	(property "Value" "Keepout_L" (at 0 3 0) (layer "F.Fab"))
	(attr smd)
	(pad "1" smd rect (at -1 0.5) (size 1 1) (layers "F.Cu" "F.Mask" "F.Paste"))
	(zone
		(net 0)
		(net_name "")
		(layer "F.Cu")
		(hatch full 0.508)
		(connect_pads (clearance 0))
		(min_thickness 0.25)
		(keepout (tracks allowed) (vias not_allowed) (pads allowed) (copperpour not_allowed) (footprints allowed))
		(fill (thermal_gap 0.5) (thermal_bridge_width 0.5))
		(polygon
			(pts (xy 0 0) (xy 3 0) (xy 3 1) (xy 1 1) (xy 1 2) (xy 0 2))
		)
	)
)
"#;

    // The keepout as pcbnew 10.0.7 saves it for an instance of `LIBRARY`:
    /// `(at 50 50)` on the front.
    const AT_50_50: &str = "(xy 50 50) (xy 53 50) (xy 53 51) (xy 51 51) (xy 51 52) (xy 50 52)";
    /// `(at 80 60 30)` on the front.
    const AT_80_60_30: &str = "(xy 80 60) (xy 82.598076 58.5) (xy 83.098076 59.366025) \
                               (xy 81.366025 60.366025) (xy 81.866025 61.232051) (xy 81 61.732051)";
    /// Set to -90 on the front, then flipped top to bottom: `(at 110 70 90)`.
    const BACK_AT_110_70_90: &str =
        "(xy 110 70) (xy 110 67) (xy 109 67) (xy 109 69) (xy 108 69) (xy 108 70)";
    /// Set to 200 on the front, then flipped left to right: `(at 140 80 -20)`.
    const BACK_AT_140_80_M20: &str = "(xy 140 80) (xy 142.819078 81.02606) (xy 143.161098 80.086368) \
                                      (xy 141.281713 79.402328) (xy 141.623733 78.462635) (xy 140.68404 78.120615)";

    fn instance(lib_id: &str, reference: &str, at: &str, layer: &str, pts: &str) -> String {
        format!(
            r#"	(footprint "{lib_id}"
		(layer "{layer}")
		(uuid "f0000000-0000-4000-8000-0000000000{reference}")
		(at {at})
		(property "Reference" "{reference}" (at 0 -2 0) (layer "F.SilkS"))
		(property "Value" "Keepout_L" (at 0 3 0) (layer "F.Fab"))
		(attr smd)
		(pad "1" smd rect (at -1 0.5) (size 1 1) (layers "{layer}"))
		(zone
			(net 0)
			(net_name "")
			(layer "{layer}")
			(uuid "a0000000-0000-4000-8000-0000000000{reference}")
			(hatch full 0.508)
			(connect_pads (clearance 0))
			(min_thickness 0.25)
			(keepout (tracks allowed) (vias not_allowed) (pads allowed) (copperpour not_allowed) (footprints allowed))
			(fill (thermal_gap 0.5) (thermal_bridge_width 0.5))
			(polygon
				(pts {pts})
			)
		)
	)
"#
        )
    }

    /// The audit of a board holding `footprints`, in a project directory
    /// whose own library table resolves `Probe:Keepout_L` and nothing else.
    async fn audit(footprints: &[String]) -> Value {
        let dir = TempDir::new().unwrap();
        let pretty = dir.path().join("Probe.pretty");
        std::fs::create_dir_all(&pretty).unwrap();
        std::fs::write(pretty.join("Keepout_L.kicad_mod"), LIBRARY).unwrap();
        std::fs::write(
            dir.path().join("fp-lib-table"),
            format!(
                "(fp_lib_table\n\t(version 7)\n\t(lib (name \"Probe\") (type \"KiCad\") (uri \"{}\") (options \"\") (descr \"\"))\n)\n",
                pretty.to_string_lossy()
            ),
        )
        .unwrap();
        let board = dir.path().join("board.kicad_pcb");
        std::fs::write(
            &board,
            format!(
                "(kicad_pcb\n\t(version 20260206)\n\t(generator \"pcbnew\")\n\t(layers\n\t\t(0 \"F.Cu\" signal)\n\t\t(2 \"B.Cu\" signal)\n\t\t(25 \"Edge.Cuts\" user)\n\t)\n{})\n",
                footprints.concat()
            ),
        )
        .unwrap();
        let result = super::super::handle_audit_manufacturing(
            &json!({ "board": board.to_str().unwrap() }),
            &super::super::test_ctx(),
        )
        .await
        .unwrap();
        assert!(!result.is_error);
        let Some(crate::mcp::protocol::ToolContent::Text { text }) = result.content.first() else {
            panic!("audit_manufacturing returned no text");
        };
        serde_json::from_str(text).unwrap()
    }

    fn zone_findings(body: &Value) -> Vec<&Value> {
        body["findings"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|finding| finding["kind"] == KIND)
            .collect()
    }

    #[tokio::test]
    async fn zones_where_kicad_saves_them_at_any_rotation_and_on_either_side_are_not_reported() {
        let body = audit(&[
            instance("Probe:Keepout_L", "01", "50 50", "F.Cu", AT_50_50),
            instance("Probe:Keepout_L", "02", "80 60 30", "F.Cu", AT_80_60_30),
            instance(
                "Probe:Keepout_L",
                "03",
                "110 70 90",
                "B.Cu",
                BACK_AT_110_70_90,
            ),
            instance(
                "Probe:Keepout_L",
                "04",
                "140 80 -20",
                "B.Cu",
                BACK_AT_140_80_M20,
            ),
        ])
        .await;
        assert!(zone_findings(&body).is_empty(), "{body:#}");
        assert_eq!(
            body["footprint_zones"],
            json!({
                "checked": 4, "by_library_outline": 4, "by_footprint_extent": 0,
                "unreadable": 0, "misplaced": 0,
            })
        );
    }

    #[tokio::test]
    async fn a_zone_left_at_the_board_origin_is_an_error_naming_where_it_is_and_where_it_belongs() {
        let library_outline = "(xy 0 0) (xy 3 0) (xy 3 1) (xy 1 1) (xy 1 2) (xy 0 2)";
        let body = audit(&[instance(
            "Probe:Keepout_L",
            "01",
            "50 50",
            "F.Cu",
            library_outline,
        )])
        .await;
        let findings = zone_findings(&body);
        assert_eq!(findings.len(), 1, "{body:#}");
        let finding = findings[0];
        assert_eq!(finding["severity"], "error");
        assert_eq!(finding["category"], "dfm");
        assert_eq!(finding["test"], "library_outline");
        assert_eq!(finding["component"], "01");
        assert_eq!(
            finding["footprint"],
            json!({
                "reference": "01", "lib_id": "Probe:Keepout_L",
                "uuid": "f0000000-0000-4000-8000-000000000001",
                "x": 50.0, "y": 50.0, "rotation": 0.0, "side": "front",
            })
        );
        assert_eq!(
            finding["zone"],
            json!({
                "uuid": "a0000000-0000-4000-8000-000000000001",
                "kind": "rule_area", "layers": ["F.Cu"],
                "center": { "x": 1.5, "y": 1.0 },
                "bbox": { "x1": 0.0, "y1": 0.0, "x2": 3.0, "y2": 2.0 },
            })
        );
        assert_eq!(
            finding["expected"]["center"],
            json!({ "x": 51.5, "y": 51.0 })
        );
        assert_eq!(
            finding["expected"]["outline"][0][1],
            json!({ "x": 53.0, "y": 50.0 })
        );
        assert_eq!(finding["distance_mm"], 70.711);
        let issue = finding["issue"].as_str().unwrap();
        assert!(
            issue.contains("01 (Probe:Keepout_L) is at (50, 50)"),
            "{issue}"
        );
        assert!(issue.contains("lies around (1.5, 1)"), "{issue}");
        assert!(finding["recommendation"]
            .as_str()
            .unwrap()
            .contains("Update Footprints from Library"));
        assert!(finding["recommendation"]
            .as_str()
            .unwrap()
            .contains("edit_component with reference '01' and library_zones {}"));
        assert_eq!(body["footprint_zones"]["misplaced"], 1);
        assert_eq!(body["footprint_zones"]["by_library_outline"], 1);
    }

    #[tokio::test]
    async fn a_zone_that_did_not_turn_with_its_footprint_is_a_warning_by_the_library_outline() {
        // The footprint is at 30 degrees; the zone is where it was at 0.
        let unturned = "(xy 80 60) (xy 83 60) (xy 83 61) (xy 81 61) (xy 81 62) (xy 80 62)";
        let body = audit(&[instance(
            "Probe:Keepout_L",
            "02",
            "80 60 30",
            "F.Cu",
            unturned,
        )])
        .await;
        let findings = zone_findings(&body);
        assert_eq!(findings.len(), 1, "{body:#}");
        assert_eq!(findings[0]["severity"], "warning");
        assert_eq!(findings[0]["test"], "library_outline");
        assert_eq!(
            findings[0]["expected"]["outline"][0][1],
            json!({ "x": 82.598076, "y": 58.5 })
        );
    }

    #[tokio::test]
    async fn a_zone_that_did_not_flip_with_its_footprint_is_reported_by_the_library_outline() {
        // The footprint is on the back at 90 degrees; the zone is where a
        // front footprint at 90 degrees has it.
        let unflipped = "(xy 110 70) (xy 110 67) (xy 111 67) (xy 111 69) (xy 112 69) (xy 112 70)";
        let body = audit(&[instance(
            "Probe:Keepout_L",
            "03",
            "110 70 90",
            "B.Cu",
            unflipped,
        )])
        .await;
        let findings = zone_findings(&body);
        assert_eq!(findings.len(), 1, "{body:#}");
        assert_eq!(findings[0]["test"], "library_outline");
        assert_eq!(findings[0]["footprint"]["side"], "back");
        assert_eq!(
            findings[0]["expected"]["bbox"],
            json!({ "x1": 108.0, "y1": 67.0, "x2": 110.0, "y2": 70.0 })
        );
    }

    #[tokio::test]
    async fn without_the_library_a_zone_at_its_footprint_passes_and_one_far_from_it_is_reported() {
        let body = audit(&[
            instance(
                "Gone:Keepout_L",
                "03",
                "110 70 90",
                "B.Cu",
                BACK_AT_110_70_90,
            ),
            instance("Gone:Keepout_L", "04", "140 80 -20", "B.Cu", AT_50_50),
        ])
        .await;
        let findings = zone_findings(&body);
        assert_eq!(findings.len(), 1, "{body:#}");
        let finding = findings[0];
        assert_eq!(finding["component"], "04");
        assert_eq!(finding["severity"], "error");
        assert_eq!(finding["test"], "footprint_extent");
        assert_eq!(
            finding["expected"]["near"],
            json!({ "x": 140.0, "y": 80.0 })
        );
        // The Value field at (0, 3) is the furthest thing from the anchor:
        // 3 mm of reach and the 10 mm margin.
        assert_eq!(finding["expected"]["within_mm"], 13.0);
        let issue = finding["issue"].as_str().unwrap();
        assert!(
            issue.contains("the library footprint could not be read"),
            "{issue}"
        );
        assert_eq!(
            body["footprint_zones"],
            json!({
                "checked": 2, "by_library_outline": 0, "by_footprint_extent": 2,
                "unreadable": 0, "misplaced": 1,
            })
        );
    }

    #[tokio::test]
    async fn a_zone_reshaped_on_the_board_is_judged_by_the_footprint_extent_not_the_library() {
        // Not the library's outline anywhere: a plain rectangle.
        let near = "(xy 50 50) (xy 53 50) (xy 53 52) (xy 50 52)";
        let far = "(xy 0 0) (xy 3 0) (xy 3 2) (xy 0 2)";
        let body = audit(&[
            instance("Probe:Keepout_L", "01", "50 50", "F.Cu", near),
            instance("Probe:Keepout_L", "02", "50 50", "F.Cu", far),
        ])
        .await;
        let findings = zone_findings(&body);
        assert_eq!(findings.len(), 1, "{body:#}");
        assert_eq!(findings[0]["component"], "02");
        assert_eq!(findings[0]["test"], "footprint_extent");
        assert!(findings[0]["issue"]
            .as_str()
            .unwrap()
            .contains("the library footprint holds no zone of this shape"));
        assert_eq!(body["footprint_zones"]["by_footprint_extent"], 2);
    }

    #[test]
    fn the_library_shape_is_recognised_turned_and_mirrored_and_a_different_shape_is_not() {
        let library: Outlines = vec![vec![
            (0.0, 0.0),
            (3.0, 0.0),
            (3.0, 1.0),
            (1.0, 1.0),
            (1.0, 2.0),
            (0.0, 2.0),
        ]];
        let moved = |back: bool, rotation: f64| -> Outlines {
            vec![library[0]
                .iter()
                .map(|p| placed_point(*p, (12.0, -7.0), rotation, back))
                .collect()]
        };
        assert!(same_shape(&moved(false, 0.0), &library));
        assert!(same_shape(&moved(false, 33.5), &library));
        assert!(same_shape(&moved(true, -120.0), &library));
        let mut bent = moved(false, 33.5);
        bent[0][3].0 += 0.05;
        assert!(!same_shape(&bent, &library));
        assert!(!same_shape(&vec![library[0][..5].to_vec()], &library));
    }
}
