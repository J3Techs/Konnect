//! Child zones of a footprint in a saved board.
//!
//! Everything else inside a placed `(footprint …)` — pads, graphics, fields —
//! is saved in the footprint's own frame, relative to its root `(at …)`. A
//! child `(zone …)` is not: KiCad's board writer saves its outline and fills
//! in board coordinates. Placed through pcbnew 10.0.7, the Tag-Connect
//! `TC2030-IDC-NL` keepout, whose library outline is
//! `(xy 1.27 -0.635) (xy -1.27 -0.635) (xy -1.27 0.635) (xy 1.27 0.635)`, is
//! saved as `(xy 51.27 49.365) …` on a footprint at `(at 50 50)` and as
//! `(xy 79.365 58.73) (xy 79.365 61.27) …` on one at `(at 80 60 90)`.
//!
//! So a saved-file edit that places a library footprint, or moves or rotates a
//! placed one, has to carry each zone point through the same motion itself;
//! rewriting the root `(at …)` alone leaves the zone where it was. This module
//! is that one step: it rewrites every point of every child zone through a
//! caller-supplied map and touches nothing else.

use super::{direct_children_with_tags, format_mm};
use anyhow::Context;
use konnect_sexp::writer::apply_edits;
use konnect_sexp::{SexpEdit, SexpNode};

/// Where KiCad saves the library zone point `(local_x, local_y)` of a
/// footprint whose root is `(at x y rotation)`, on the front or the back.
///
/// On the front the point is turned by the footprint's rotation about its
/// position. On the back it is first mirrored in the footprint's own Y: KiCad
/// flips about the footprint's position, top to bottom, and a left-to-right
/// flip is the same mirror followed by a half turn that the saved rotation
/// already carries, so side and saved rotation fix the point whichever way the
/// footprint was flipped.
pub(crate) fn placed_point(
    (local_x, local_y): (f64, f64),
    (x, y): (f64, f64),
    rotation: f64,
    back: bool,
) -> (f64, f64) {
    let local_y = if back { -local_y } else { local_y };
    konnect_sexp::geometry::rotate_about(local_x, local_y, x, y, rotation)
}

/// Clause heads that carry a coordinate wherever they appear.
const POINT_TAGS: [&str; 6] = ["xy", "pts", "start", "mid", "end", "center"];

fn carries_points(node: &SexpNode) -> bool {
    node.children().is_some_and(|children| {
        children.iter().any(|child| {
            child.head().is_some_and(|head| POINT_TAGS.contains(&head)) || carries_points(child)
        })
    })
}

fn point(block: &str, tag: &str, map: &dyn Fn(f64, f64) -> (f64, f64)) -> anyhow::Result<String> {
    let node = konnect_sexp::parse_sexp(block)?;
    let coordinate = |index: usize| {
        node.get_f64(index)
            .with_context(|| format!("a ({tag} …) point has an unreadable coordinate"))
    };
    if node.children().map_or(0, <[SexpNode]>::len) != 3 {
        anyhow::bail!("a ({tag} …) point does not hold exactly two coordinates");
    }
    let (x, y) = map(coordinate(1)?, coordinate(2)?);
    Ok(format!("({tag} {} {})", format_mm(x), format_mm(y)))
}

/// One `(pts …)` list: `(xy …)` vertices and `(arc (start …) (mid …) (end …))`
/// entries. Also the outline of a footprint polygon or a custom pad's
/// primitive, which are written the same way.
pub(crate) fn map_pts(pts: &str, map: &dyn Fn(f64, f64) -> (f64, f64)) -> anyhow::Result<String> {
    let mut edits = Vec::new();
    for (start, end, tag) in direct_children_with_tags(pts, "pts")? {
        let entry = &pts[start..end];
        match tag.as_str() {
            "xy" => edits.push(SexpEdit::replace(start, end, point(entry, "xy", map)?)),
            "arc" => {
                let mut arc_edits = Vec::new();
                for (s, e, part) in direct_children_with_tags(entry, "arc")? {
                    if !matches!(part.as_str(), "start" | "mid" | "end") {
                        anyhow::bail!("an outline arc holds an unsupported ({part} …) clause");
                    }
                    arc_edits.push(SexpEdit::replace(s, e, point(&entry[s..e], &part, map)?));
                }
                edits.push(SexpEdit::replace(
                    start,
                    end,
                    apply_edits(entry.to_string(), arc_edits),
                ));
            }
            other => anyhow::bail!("an outline holds an unsupported ({other} …) entry"),
        }
    }
    Ok(apply_edits(pts.to_string(), edits))
}

/// One `(zone …)` block with every outline and fill point mapped.
pub(crate) fn map_zone(zone: &str, map: &dyn Fn(f64, f64) -> (f64, f64)) -> anyhow::Result<String> {
    let mut edits = Vec::new();
    for (start, end, tag) in direct_children_with_tags(zone, "zone")? {
        let block = &zone[start..end];
        if !matches!(tag.as_str(), "polygon" | "filled_polygon") {
            // Anything else with a coordinate is a clause this code does not
            // know how to carry; moving the rest and leaving it is the bug
            // this module exists to prevent.
            if carries_points(&konnect_sexp::parse_sexp(block)?) {
                anyhow::bail!(
                    "a child zone carries coordinates in a ({tag} …) clause that cannot be \
                     carried with the footprint"
                );
            }
            continue;
        }
        let mut inner = Vec::new();
        let mut outlines = 0;
        for (s, e, part) in direct_children_with_tags(block, &tag)? {
            match part.as_str() {
                "pts" => {
                    outlines += 1;
                    inner.push(SexpEdit::replace(s, e, map_pts(&block[s..e], map)?));
                }
                // A fill names the layer it is on (and, in older files, an
                // island flag); neither has a coordinate.
                "layer" | "island" => {}
                other => anyhow::bail!("a zone ({tag} …) holds an unsupported ({other} …) clause"),
            }
        }
        if outlines == 0 {
            anyhow::bail!("a zone ({tag} …) has no (pts …) outline");
        }
        edits.push(SexpEdit::replace(
            start,
            end,
            apply_edits(block.to_string(), inner),
        ));
    }
    Ok(apply_edits(zone.to_string(), edits))
}

/// `footprint` with every point of every direct child `(zone …)` mapped
/// through `map`, and every other byte kept. A footprint without a child zone
/// comes back unchanged. A zone whose coordinates cannot all be accounted for
/// is an error, never a partial move.
pub(crate) fn map_child_zones(
    footprint: &str,
    map: &dyn Fn(f64, f64) -> (f64, f64),
) -> anyhow::Result<String> {
    let mut edits = Vec::new();
    for (start, end, tag) in direct_children_with_tags(footprint, "footprint")? {
        if tag == "zone" {
            edits.push(SexpEdit::replace(
                start,
                end,
                map_zone(&footprint[start..end], map)?,
            ));
        }
    }
    Ok(apply_edits(footprint.to_string(), edits))
}

#[cfg(test)]
mod tests {
    use super::*;

    const FOOTPRINT: &str = r#"(footprint "Connector:Tag-Connect"
	(layer "F.Cu")
	(at 50 50)
	(pad "1" smd circle (at -1.27 0.635) (size 0.7874 0.7874) (layers "F.Cu" "F.Mask"))
	(zone
		(net 0)
		(net_name "")
		(layer "F.Cu")
		(hatch full 0.508)
		(connect_pads (clearance 0))
		(min_thickness 0.254)
		(keepout (tracks allowed) (vias not_allowed) (pads allowed) (copperpour not_allowed) (footprints not_allowed))
		(fill (thermal_gap 0.508) (thermal_bridge_width 0.508))
		(polygon
			(pts
				(xy 1.27 -0.635) (xy -1.27 -0.635) (arc (start -1.27 0.635) (mid 0 1) (end 1.27 0.635))
			)
		)
		(filled_polygon
			(layer "F.Cu")
			(pts (xy 1 -0.5) (xy -1 -0.5) (xy -1 0.5))
		)
	)
)"#;

    #[test]
    fn every_outline_and_fill_point_moves_and_nothing_else_does() {
        let moved = map_child_zones(FOOTPRINT, &|x, y| (x + 50.0, y + 50.0)).unwrap();
        assert!(moved.contains(
            "(xy 51.27 49.365) (xy 48.73 49.365) (arc (start 48.73 50.635) (mid 50 51) (end 51.27 50.635))"
        ));
        assert!(moved.contains("(pts (xy 51 49.5) (xy 49 49.5) (xy 49 50.5))"));
        // The pad, in the footprint's own frame, and every zone setting stay.
        assert!(moved.contains("(pad \"1\" smd circle (at -1.27 0.635)"));
        assert!(moved.contains("(hatch full 0.508)"));
        let (before, after) = (
            FOOTPRINT.find("(polygon").unwrap(),
            moved.find("(polygon").unwrap(),
        );
        assert_eq!(&FOOTPRINT[..before], &moved[..after]);
    }

    #[test]
    fn a_footprint_without_a_zone_is_returned_byte_for_byte() {
        let plain = "(footprint \"R\" (layer \"F.Cu\") (at 1 2) (zone_connect 1) (pad \"1\" smd rect (at 0 0) (size 1 1) (layers \"F.Cu\")))";
        assert_eq!(map_child_zones(plain, &|_, _| (9.0, 9.0)).unwrap(), plain);
    }

    #[test]
    fn a_zone_with_coordinates_this_code_cannot_account_for_is_refused() {
        let unknown = FOOTPRINT.replace(
            "(min_thickness 0.254)",
            "(min_thickness 0.254) (anchor (xy 1 1))",
        );
        let error = map_child_zones(&unknown, &|x, y| (x, y)).unwrap_err();
        assert!(format!("{error:#}").contains("(anchor …)"), "{error:#}");

        let curve = FOOTPRINT.replace("(xy 1.27 -0.635)", "(bezier 1 2 3 4)");
        let error = map_child_zones(&curve, &|x, y| (x, y)).unwrap_err();
        assert!(format!("{error:#}").contains("(bezier …)"), "{error:#}");
    }
}
