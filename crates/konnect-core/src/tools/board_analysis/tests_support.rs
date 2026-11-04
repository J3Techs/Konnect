//! Synthetic boards for the reference-math tests, in KiCad 10's saved form.

/// Header with an explicit two-layer stackup: 35 µm copper both sides.
const HEAD: &str = r#"(kicad_pcb
	(version 20260206)
	(generator "pcbnew")
	(general (thickness 1.6))
	(layers
		(0 "F.Cu" signal)
		(2 "B.Cu" signal)
		(25 "Edge.Cuts" user)
	)
	(setup
		(stackup
			(layer "F.Cu" (type "copper") (thickness 0.035))
			(layer "dielectric 1" (type "core") (thickness 1.51))
			(layer "B.Cu" (type "copper") (thickness 0.035))
		)
	)
"#;

/// One SMD pad `w` × `h` mm on F.Cu at (x, y) in footprint `reference`.
pub(crate) fn pad(reference: &str, x: f64, y: f64, w: f64, h: f64, net: &str) -> String {
    format!(
        r#"	(footprint "Pad"
		(layer "F.Cu")
		(at {x} {y})
		(property "Reference" "{reference}" (at 0 -2 0) (layer "F.SilkS"))
		(pad "1" smd rect (at 0 0) (size {w} {h}) (layers "F.Cu" "F.Mask") (net "{net}"))
	)
"#
    )
}

pub(crate) fn segment(
    x1: f64,
    y1: f64,
    x2: f64,
    y2: f64,
    w: f64,
    layer: &str,
    net: &str,
) -> String {
    format!(
        "\t(segment (start {x1} {y1}) (end {x2} {y2}) (width {w}) (layer \"{layer}\") (net \"{net}\"))\n"
    )
}

pub(crate) fn via(x: f64, y: f64, size: f64, drill: f64, net: &str) -> String {
    format!("\t(via (at {x} {y}) (size {size}) (drill {drill}) (layers \"F.Cu\" \"B.Cu\") (net \"{net}\"))\n")
}

/// A filled rectangular zone (fill = outline).
pub(crate) fn zone(
    name: &str,
    layer: &str,
    net: &str,
    x0: f64,
    y0: f64,
    x1: f64,
    y1: f64,
) -> String {
    format!(
        r#"	(zone (net "{net}") (layer "{layer}") (name "{name}") (connect_pads yes (clearance 0.2)) (min_thickness 0.2) (fill yes)
		(polygon (pts (xy {x0} {y0}) (xy {x1} {y0}) (xy {x1} {y1}) (xy {x0} {y1})))
		(filled_polygon (layer "{layer}") (pts (xy {x0} {y0}) (xy {x1} {y0}) (xy {x1} {y1}) (xy {x0} {y1})))
	)
"#
    )
}

pub(crate) fn board(items: &[String]) -> String {
    let mut s = HEAD.to_string();
    for i in items {
        s.push_str(i);
    }
    s.push_str("\t(gr_rect (start -10 -10) (end 200 100) (stroke (width 0.05) (type default)) (fill no) (layer \"Edge.Cuts\"))\n)\n");
    s
}

/// Two pads 100 mm apart joined by a 1.0 mm F.Cu track.
pub(crate) const TRACK_BOARD: &str = r#"(kicad_pcb
	(version 20260206)
	(generator "pcbnew")
	(general (thickness 1.6))
	(layers
		(0 "F.Cu" signal)
		(2 "B.Cu" signal)
		(25 "Edge.Cuts" user)
	)
	(setup
		(stackup
			(layer "F.Cu" (type "copper") (thickness 0.035))
			(layer "dielectric 1" (type "core") (thickness 1.51))
			(layer "B.Cu" (type "copper") (thickness 0.035))
		)
	)
	(footprint "Pad" (layer "F.Cu") (at 10 10)
		(property "Reference" "J1" (at 0 -2 0) (layer "F.SilkS"))
		(pad "1" smd rect (at 0 0) (size 1.5 1.5) (layers "F.Cu" "F.Mask") (net "P")))
	(footprint "Pad" (layer "F.Cu") (at 110 10)
		(property "Reference" "J2" (at 0 -2 0) (layer "F.SilkS"))
		(pad "1" smd rect (at 0 0) (size 1.5 1.5) (layers "F.Cu" "F.Mask") (net "P")))
	(segment (start 10 10) (end 110 10) (width 1.0) (layer "F.Cu") (net "P"))
	(gr_rect (start 0 0) (end 120 20) (stroke (width 0.05) (type default)) (fill no) (layer "Edge.Cuts"))
)
"#;
