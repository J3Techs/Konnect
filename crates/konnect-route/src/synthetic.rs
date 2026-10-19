//! Synthetic boards and rules for unit tests across every workstream.
//!
//! Nothing here comes from a real project: boards are generated in code (or
//! hand-written, for the loader), small enough to reason about by hand, and
//! deterministic. Use [`BoardBuilder`] for a custom case and the generators
//! for the common ones; pair them with [`UniformRules`] when a test needs a
//! [`RuleTable`] without a project file.

use crate::api::{
    ClassId, FootprintId, ItemKind, KeepoutId, LayerId, NetId, Ownership, PadId, TrackId, ViaId,
    ZoneId,
};
use crate::board::{
    pair_partners, BoardOutline, CopperLayerKind, Courtyard, Drill, Footprint, Keepout,
    KeepoutRules, LayerTable, Net, Pad, PadKind, RouteBoard, Side, Track, Via, ZonePiece,
};
use crate::rules::{InertRule, NetClassRules, RuleSubject, RuleTable, ViaRule, WidthRule};
use konnect_geom::{BBox, Circle, ConvexPoly, Coord, Point, Polygon, Segment, Shape};
use std::collections::BTreeSet;

fn p(mm: (f64, f64)) -> Point {
    Point::from_mm(mm.0, mm.1).expect("synthetic coordinates are representable")
}

fn nm(mm: f64) -> Coord {
    konnect_geom::mm_to_nm(mm).expect("synthetic lengths are representable")
}

fn rect_mm(x0: f64, y0: f64, x1: f64, y1: f64) -> Polygon {
    let (a, b) = (p((x0, y0)), p((x1, y1)));
    Polygon::new(
        vec![a, Point::new(b.x, a.y), b, Point::new(a.x, b.y)],
        vec![],
    )
    .expect("non-degenerate rectangle")
}

struct PadSpec {
    footprint: usize,
    number: String,
    net: Option<String>,
    kind: PadKind,
    center: Point,
    layers: Vec<String>,
    shape: Shape,
    drill: Option<Shape>,
}

/// Builds a [`RouteBoard`] programmatically. All inputs are millimetres;
/// nets are referred to by name and get sorted ids at [`BoardBuilder::build`].
pub struct BoardBuilder {
    width: f64,
    height: f64,
    layers: Vec<String>,
    nets: BTreeSet<String>,
    footprints: Vec<(String, Point, Option<Polygon>)>,
    pads: Vec<PadSpec>,
    tracks: Vec<(String, String, Segment, bool)>,
    vias: Vec<(String, Point, Coord, Coord, bool)>,
    zones: Vec<(String, String, Polygon)>,
    keepouts: Vec<(String, Vec<String>, Polygon, KeepoutRules)>,
}

impl BoardBuilder {
    /// A `width × height` mm rectangular board with origin at (0, 0) and
    /// these copper layers (all `signal`), top to bottom.
    pub fn new(width: f64, height: f64, layers: &[&str]) -> Self {
        BoardBuilder {
            width,
            height,
            layers: layers.iter().map(|s| s.to_string()).collect(),
            nets: BTreeSet::new(),
            footprints: Vec::new(),
            pads: Vec::new(),
            tracks: Vec::new(),
            vias: Vec::new(),
            zones: Vec::new(),
            keepouts: Vec::new(),
        }
    }

    pub fn net(&mut self, name: &str) -> &mut Self {
        self.nets.insert(name.to_string());
        self
    }

    /// A footprint with no courtyard; returns its index for pads.
    pub fn footprint(&mut self, reference: &str, at: (f64, f64)) -> usize {
        self.footprints.push((reference.to_string(), p(at), None));
        self.footprints.len() - 1
    }

    /// A footprint with a rectangular courtyard of half-size `half` around
    /// `at`.
    pub fn footprint_with_courtyard(
        &mut self,
        reference: &str,
        at: (f64, f64),
        half: (f64, f64),
    ) -> usize {
        let court = rect_mm(at.0 - half.0, at.1 - half.1, at.0 + half.0, at.1 + half.1);
        self.footprints
            .push((reference.to_string(), p(at), Some(court)));
        self.footprints.len() - 1
    }

    /// An axis-aligned rectangular SMD pad of `size` centred on `center`.
    pub fn smd_pad(
        &mut self,
        footprint: usize,
        number: &str,
        net: &str,
        center: (f64, f64),
        size: (f64, f64),
        layer: &str,
    ) -> &mut Self {
        let b = BBox::new(
            p((center.0 - size.0 / 2.0, center.1 - size.1 / 2.0)),
            p((center.0 + size.0 / 2.0, center.1 + size.1 / 2.0)),
        );
        let shape = Shape::ConvexPoly(ConvexPoly::axis_rect(b, 0).expect("positive pad size"));
        self.push_pad(
            footprint,
            number,
            net,
            PadKind::Smd,
            center,
            vec![layer.to_string()],
            shape,
            None,
        )
    }

    /// A round SMD pad (a BGA ball land).
    pub fn ball_pad(
        &mut self,
        footprint: usize,
        number: &str,
        net: &str,
        center: (f64, f64),
        diameter: f64,
        layer: &str,
    ) -> &mut Self {
        let shape = Shape::Circle(Circle::new(p(center), nm(diameter / 2.0)));
        self.push_pad(
            footprint,
            number,
            net,
            PadKind::Smd,
            center,
            vec![layer.to_string()],
            shape,
            None,
        )
    }

    /// A round plated through-hole pad on every copper layer.
    pub fn th_pad(
        &mut self,
        footprint: usize,
        number: &str,
        net: &str,
        center: (f64, f64),
        diameter: f64,
        drill: f64,
    ) -> &mut Self {
        let shape = Shape::Circle(Circle::new(p(center), nm(diameter / 2.0)));
        let hole = Shape::Circle(Circle::new(p(center), nm(drill / 2.0)));
        self.push_pad(
            footprint,
            number,
            net,
            PadKind::ThroughHole,
            center,
            vec!["*.Cu".to_string()],
            shape,
            Some(hole),
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn push_pad(
        &mut self,
        footprint: usize,
        number: &str,
        net: &str,
        kind: PadKind,
        center: (f64, f64),
        layers: Vec<String>,
        shape: Shape,
        drill: Option<Shape>,
    ) -> &mut Self {
        let net = (!net.is_empty()).then(|| {
            self.nets.insert(net.to_string());
            net.to_string()
        });
        self.pads.push(PadSpec {
            footprint,
            number: number.to_string(),
            net,
            kind,
            center: p(center),
            layers,
            shape,
            drill,
        });
        self
    }

    pub fn track(
        &mut self,
        net: &str,
        layer: &str,
        from: (f64, f64),
        to: (f64, f64),
        width: f64,
        locked: bool,
    ) -> &mut Self {
        self.nets.insert(net.to_string());
        self.tracks.push((
            net.to_string(),
            layer.to_string(),
            Segment::new(p(from), p(to), nm(width)),
            locked,
        ));
        self
    }

    /// A through via.
    pub fn via(
        &mut self,
        net: &str,
        at: (f64, f64),
        diameter: f64,
        drill: f64,
        locked: bool,
    ) -> &mut Self {
        self.nets.insert(net.to_string());
        self.vias
            .push((net.to_string(), p(at), nm(diameter), nm(drill), locked));
        self
    }

    /// A filled rectangular plane piece.
    pub fn zone(&mut self, net: &str, layer: &str, rect: (f64, f64, f64, f64)) -> &mut Self {
        self.nets.insert(net.to_string());
        self.zones.push((
            net.to_string(),
            layer.to_string(),
            rect_mm(rect.0, rect.1, rect.2, rect.3),
        ));
        self
    }

    pub fn keepout(
        &mut self,
        name: &str,
        layers: &[&str],
        rect: (f64, f64, f64, f64),
        rules: KeepoutRules,
    ) -> &mut Self {
        self.keepouts.push((
            name.to_string(),
            layers.iter().map(|s| s.to_string()).collect(),
            rect_mm(rect.0, rect.1, rect.2, rect.3),
            rules,
        ));
        self
    }

    pub fn build(&self) -> RouteBoard {
        let layers = LayerTable::from_layers(
            self.layers
                .iter()
                .enumerate()
                .map(|(i, n)| (n.clone(), 2 * i as u32, CopperLayerKind::Signal))
                .collect(),
        );
        let names: Vec<String> = self.nets.iter().cloned().collect();
        let pairs = pair_partners(&names);
        let nets: Vec<Net> = names
            .iter()
            .zip(pairs)
            .enumerate()
            .map(|(i, (name, pair))| Net {
                id: NetId(i as u32),
                name: name.clone(),
                class: ClassId(0),
                pair,
            })
            .collect();
        let net_id = |name: &str| {
            names
                .binary_search_by(|n| n.as_str().cmp(name))
                .ok()
                .map(|i| NetId(i as u32))
        };
        let layer = |name: &str| {
            layers
                .by_name(name)
                .expect("synthetic layer is in the stack")
        };

        // Pads grouped by footprint (stable), so each footprint owns a range.
        let mut order: Vec<usize> = (0..self.pads.len()).collect();
        order.sort_by_key(|&i| self.pads[i].footprint);
        let mut pads = Vec::new();
        let mut ranges = vec![(0u32, 0u32); self.footprints.len()];
        for &i in &order {
            let s = &self.pads[i];
            let id = PadId(pads.len() as u32);
            let range = &mut ranges[s.footprint];
            if range.0 == range.1 {
                *range = (id.0, id.0);
            }
            range.1 = id.0 + 1;
            pads.push(Pad {
                id,
                footprint: FootprintId(s.footprint as u32),
                number: s.number.clone(),
                net: s.net.as_deref().and_then(net_id),
                kind: s.kind,
                center: s.center,
                angle_deg: 0.0,
                layers: layers.expand_all(s.layers.iter().map(String::as_str)),
                shapes: vec![s.shape.clone()],
                drill: s.drill.clone().map(|shape| Drill {
                    shape,
                    plated: true,
                }),
                local_clearance: None,
                approximated: false,
                ownership: Ownership::Fixed,
                uuid: None,
            });
        }
        let footprints = self
            .footprints
            .iter()
            .enumerate()
            .map(|(i, (reference, at, court))| Footprint {
                id: FootprintId(i as u32),
                reference: Some(reference.clone()),
                at: *at,
                rotation_deg: 0.0,
                side: Side::Front,
                locked: false,
                courtyards: court
                    .iter()
                    .map(|region| Courtyard {
                        side: Side::Front,
                        region: region.clone(),
                    })
                    .collect(),
                pad_range: ranges[i],
            })
            .collect();
        let own = |locked: bool| {
            if locked {
                Ownership::Locked
            } else {
                Ownership::Fixed
            }
        };
        let tracks = self
            .tracks
            .iter()
            .enumerate()
            .map(|(i, (net, l, seg, locked))| Track {
                id: TrackId(i as u32),
                net: net_id(net),
                layer: layer(l),
                seg: *seg,
                ownership: own(*locked),
                uuid: None,
            })
            .collect();
        let vias = self
            .vias
            .iter()
            .enumerate()
            .map(|(i, (net, at, d, drill, locked))| Via {
                id: ViaId(i as u32),
                net: net_id(net),
                center: *at,
                diameter: *d,
                drill: *drill,
                layers: layers.all(),
                ownership: own(*locked),
                uuid: None,
            })
            .collect();
        let zones = self
            .zones
            .iter()
            .enumerate()
            .map(|(i, (net, l, region))| ZonePiece {
                id: ZoneId(i as u32),
                net: net_id(net),
                layer: layer(l),
                region: region.clone(),
                filled: true,
                island: false,
                zone_name: None,
                priority: 0,
            })
            .collect();
        let keepouts = self
            .keepouts
            .iter()
            .enumerate()
            .map(|(i, (name, ls, region, rules))| Keepout {
                id: KeepoutId(i as u32),
                name: Some(name.clone()),
                layers: layers.expand_all(ls.iter().map(String::as_str)),
                region: region.clone(),
                rules: *rules,
            })
            .collect();
        let outline_poly = rect_mm(0.0, 0.0, self.width, self.height);
        let ring = outline_poly.outline.clone();
        let edges = (0..ring.len())
            .map(|i| Segment::new(ring[i], ring[(i + 1) % ring.len()], 0))
            .collect();
        let routable = layers.all();
        let board = RouteBoard {
            outline: BoardOutline {
                bbox: Some(outline_poly.bbox()),
                regions: vec![outline_poly],
                edges,
            },
            layers,
            routable,
            nets,
            footprints,
            pads,
            tracks,
            arcs: Vec::new(),
            vias,
            zones,
            keepouts,
        };
        debug_assert_eq!(board.validate(), Ok(()));
        board
    }
}

// ─── Generators ──────────────────────────────────────────────────────────────

/// `n` two-pin nets `N00`, `N01`, … across a 2-layer board: pad 1 of each
/// on footprint `U1` at x = 2 mm, pad 2 on `U2` at x = 18 mm, 2 mm apart in
/// y. With `join_first`, net `N00` is already routed by a straight F.Cu
/// track.
pub fn two_pin_nets(n: usize, join_first: bool) -> RouteBoard {
    let mut b = BoardBuilder::new(20.0, 2.0 * n as f64 + 2.0, &["F.Cu", "B.Cu"]);
    let left = b.footprint("U1", (2.0, 1.0));
    let right = b.footprint("U2", (18.0, 1.0));
    for i in 0..n {
        let net = format!("N{i:02}");
        let y = 2.0 + 2.0 * i as f64;
        b.smd_pad(
            left,
            &(i + 1).to_string(),
            &net,
            (2.0, y),
            (1.0, 0.6),
            "F.Cu",
        );
        b.smd_pad(
            right,
            &(i + 1).to_string(),
            &net,
            (18.0, y),
            (1.0, 0.6),
            "F.Cu",
        );
    }
    if join_first && n > 0 {
        b.track("N00", "F.Cu", (2.0, 2.0), (18.0, 2.0), 0.2, false);
    }
    b.build()
}

/// A 4-layer board with a GND plane on `In1.Cu`, two GND through-hole pads
/// (joined through the plane), two GND SMD pads on F.Cu (not joined), and a
/// two-pin `SIG` net.
pub fn plane_with_pads() -> RouteBoard {
    let mut b = BoardBuilder::new(30.0, 20.0, &["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]);
    b.zone("GND", "In1.Cu", (0.5, 0.5, 29.5, 19.5));
    let j = b.footprint("J1", (5.0, 10.0));
    b.th_pad(j, "1", "GND", (5.0, 5.0), 1.6, 1.0);
    b.th_pad(j, "2", "GND", (5.0, 15.0), 1.6, 1.0);
    let c = b.footprint("C1", (20.0, 10.0));
    b.smd_pad(c, "1", "GND", (19.5, 10.0), (0.6, 0.5), "F.Cu");
    b.smd_pad(c, "2", "SIG", (20.5, 10.0), (0.6, 0.5), "F.Cu");
    let r = b.footprint("R1", (25.0, 10.0));
    b.smd_pad(r, "1", "GND", (24.5, 10.0), (0.6, 0.5), "F.Cu");
    b.smd_pad(r, "2", "SIG", (25.5, 10.0), (0.6, 0.5), "F.Cu");
    b.build()
}

/// A `cols × rows` BGA field of round lands at `pitch` on F.Cu, one net per
/// ball (`B<row>_<col>`), under footprint `BGA1` with a courtyard 0.5 mm
/// outside the outer balls. With `dogbones`, each ball gets a 0.5/0.25 via
/// offset half a pitch diagonally and a 0.12 mm stub, all locked — the escape
/// pattern an access audit has to see through.
pub fn bga_field(cols: usize, rows: usize, pitch: f64, ball: f64, dogbones: bool) -> RouteBoard {
    let margin = 3.0;
    let w = (cols as f64 - 1.0) * pitch + 2.0 * margin;
    let h = (rows as f64 - 1.0) * pitch + 2.0 * margin;
    let mut b = BoardBuilder::new(w, h, &["F.Cu", "In1.Cu", "B.Cu"]);
    let center = (w / 2.0, h / 2.0);
    let half = (
        (cols as f64 - 1.0) * pitch / 2.0 + ball / 2.0 + 0.5,
        (rows as f64 - 1.0) * pitch / 2.0 + ball / 2.0 + 0.5,
    );
    let u = b.footprint_with_courtyard("BGA1", center, half);
    for r in 0..rows {
        for c in 0..cols {
            let net = format!("B{r:02}_{c:02}");
            let at = (margin + c as f64 * pitch, margin + r as f64 * pitch);
            b.ball_pad(u, &format!("{r}_{c}"), &net, at, ball, "F.Cu");
            if dogbones {
                let via = (at.0 + pitch / 2.0, at.1 + pitch / 2.0);
                b.track(&net, "F.Cu", at, via, 0.12, true);
                b.via(&net, via, 0.5, 0.25, true);
            }
        }
    }
    b.build()
}

// ─── Uniform rules ───────────────────────────────────────────────────────────

/// A [`RuleTable`] with one class and constant values, for tests that need
/// rules but not a project. All values in nm.
#[derive(Debug, Clone)]
pub struct UniformRules {
    pub class: NetClassRules,
    pub edge_clearance: Coord,
    pub hole_clearance: Coord,
    pub hole_to_hole: Coord,
    pub min_track_angle_deg: Option<f64>,
    pub min_segment_length: Option<Coord>,
}

impl Default for UniformRules {
    /// 0.2 mm clearance, 0.15 mm tracks, 0.6/0.3 vias, 0.3 mm edge,
    /// 0.25 mm hole clearance, 0.35 mm hole-to-hole, 90° minimum angle.
    fn default() -> Self {
        UniformRules {
            class: NetClassRules {
                name: "Default".to_string(),
                clearance: 200_000,
                track_width: 150_000,
                via_diameter: 600_000,
                via_drill: 300_000,
                diff_pair_width: None,
                diff_pair_gap: None,
            },
            edge_clearance: 300_000,
            hole_clearance: 250_000,
            hole_to_hole: 350_000,
            min_track_angle_deg: Some(90.0),
            min_segment_length: None,
        }
    }
}

impl RuleTable for UniformRules {
    fn class_of(&self, _net: Option<NetId>) -> ClassId {
        ClassId(0)
    }

    fn class(&self, _class: ClassId) -> &NetClassRules {
        &self.class
    }

    fn clearance(&self, a: &RuleSubject<'_>, b: &RuleSubject<'_>, _layer: LayerId) -> Coord {
        match (a.local_clearance, b.local_clearance) {
            (None, None) => self.class.clearance,
            (la, lb) => la.unwrap_or(0).max(lb.unwrap_or(0)),
        }
    }

    fn track_width(&self, _subject: &RuleSubject<'_>, _layer: LayerId) -> WidthRule {
        WidthRule {
            min: self.class.track_width,
            preferred: self.class.track_width,
        }
    }

    fn via(&self, _subject: &RuleSubject<'_>) -> ViaRule {
        ViaRule {
            diameter: self.class.via_diameter,
            drill: self.class.via_drill,
            min_diameter: self.class.via_diameter,
            min_drill: self.class.via_drill,
            min_annular: (self.class.via_diameter - self.class.via_drill) / 2,
        }
    }

    fn min_track_angle_deg(&self, _subject: &RuleSubject<'_>, _layer: LayerId) -> Option<f64> {
        self.min_track_angle_deg
    }

    fn min_segment_length(&self, subject: &RuleSubject<'_>, _layer: LayerId) -> Option<Coord> {
        (subject.kind == ItemKind::Track)
            .then_some(self.min_segment_length)
            .flatten()
    }

    fn edge_clearance(&self) -> Coord {
        self.edge_clearance
    }

    fn hole_clearance(&self) -> Coord {
        self.hole_clearance
    }

    fn hole_to_hole(&self) -> Coord {
        self.hole_to_hole
    }

    fn max_clearance(&self) -> Coord {
        self.class
            .clearance
            .max(self.edge_clearance)
            .max(self.hole_clearance)
            .max(self.hole_to_hole)
    }

    fn inert_rules(&self) -> &[InertRule] {
        &[]
    }
}

// ─── Hand-written board text ─────────────────────────────────────────────────

/// A small KiCad 10 board exercising every construct the loader reads: a
/// rectangular outline, a power inner layer, a rotated SMD footprint with a
/// line courtyard, a through-hole connector with an oval drill, locked and
/// unlocked tracks, an arc, a through via, a filled plane with a hole, and a
/// keepout rule area. Hand-written; no real project.
pub const MINIMAL_KICAD10_BOARD: &str = r#"(kicad_pcb
	(version 20260206)
	(generator "pcbnew")
	(general
		(thickness 1.6)
	)
	(layers
		(0 "F.Cu" signal)
		(4 "In1.Cu" power)
		(6 "In2.Cu" signal)
		(2 "B.Cu" signal)
		(25 "Edge.Cuts" user)
		(31 "F.CrtYd" user "F.Courtyard")
	)
	(footprint "R_0402"
		(layer "F.Cu")
		(uuid "00000000-0000-0000-0000-000000000001")
		(at 20 10 90)
		(property "Reference" "R1"
			(at 0 -1 90)
			(layer "F.SilkS")
		)
		(fp_line (start -1 -0.6) (end 1 -0.6) (stroke (width 0.05) (type default)) (layer "F.CrtYd"))
		(fp_line (start 1 -0.6) (end 1 0.6) (stroke (width 0.05) (type default)) (layer "F.CrtYd"))
		(fp_line (start 1 0.6) (end -1 0.6) (stroke (width 0.05) (type default)) (layer "F.CrtYd"))
		(fp_line (start -1 0.6) (end -1 -0.6) (stroke (width 0.05) (type default)) (layer "F.CrtYd"))
		(pad "1" smd roundrect
			(at -0.5 0 90)
			(size 0.6 0.5)
			(layers "F.Cu" "F.Mask" "F.Paste")
			(roundrect_rratio 0.25)
			(net "SIG")
			(uuid "00000000-0000-0000-0000-000000000002")
		)
		(pad "2" smd roundrect
			(at 0.5 0 90)
			(size 0.6 0.5)
			(layers "F.Cu" "F.Mask" "F.Paste")
			(roundrect_rratio 0.25)
			(net "GND")
			(uuid "00000000-0000-0000-0000-000000000003")
		)
	)
	(footprint "Conn_1x03"
		(layer "F.Cu")
		(uuid "00000000-0000-0000-0000-000000000004")
		(at 30 20)
		(property "Reference" "J1"
			(at 0 -2 0)
			(layer "F.SilkS")
		)
		(pad "1" thru_hole oval
			(at 0 0)
			(size 1 1.6)
			(drill oval 0.6 1.2)
			(layers "*.Cu" "*.Mask")
			(net "GND")
		)
		(pad "2" thru_hole circle
			(at 0 2.54)
			(size 1.6 1.6)
			(drill 1)
			(layers "*.Cu" "*.Mask")
			(net "D+")
		)
		(pad "3" thru_hole circle
			(at 0 5.08)
			(size 1.6 1.6)
			(drill 1)
			(layers "*.Cu" "*.Mask")
			(net "D-")
		)
	)
	(gr_rect
		(start 0 0)
		(end 50 40)
		(stroke (width 0.05) (type default))
		(fill no)
		(layer "Edge.Cuts")
	)
	(segment
		(start 20 10.5)
		(end 25 10.5)
		(width 0.2)
		(locked yes)
		(layer "F.Cu")
		(net "SIG")
		(uuid "00000000-0000-0000-0000-000000000010")
	)
	(segment
		(start 30 20)
		(end 30 25)
		(width 0.3)
		(layer "B.Cu")
		(net "GND")
		(uuid "00000000-0000-0000-0000-000000000011")
	)
	(arc
		(start 25 10.5)
		(mid 26 11)
		(end 26.5 12)
		(width 0.2)
		(layer "F.Cu")
		(net "SIG")
		(uuid "00000000-0000-0000-0000-000000000012")
	)
	(via
		(at 30 25)
		(size 0.6)
		(drill 0.3)
		(layers "F.Cu" "B.Cu")
		(net "GND")
		(uuid "00000000-0000-0000-0000-000000000013")
	)
	(zone
		(net "GND")
		(layer "In1.Cu")
		(uuid "00000000-0000-0000-0000-000000000020")
		(name "GND plane")
		(polygon (pts (xy 1 1) (xy 49 1) (xy 49 39) (xy 1 39)))
		(polygon (pts (xy 10 10) (xy 15 10) (xy 15 15) (xy 10 15)))
		(filled_polygon
			(layer "In1.Cu")
			(pts (xy 1.5 1.5) (xy 48.5 1.5) (xy 48.5 38.5) (xy 1.5 38.5))
		)
	)
	(zone
		(net "")
		(layers "*.Cu")
		(uuid "00000000-0000-0000-0000-000000000021")
		(name "no_vias")
		(keepout (tracks allowed) (vias not_allowed) (pads allowed) (copperpour allowed) (footprints allowed))
		(polygon (pts (xy 40 30) (xy 45 30) (xy 45 35) (xy 40 35)))
	)
)
"#;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::api::LayerMask;

    #[test]
    fn generators_build_valid_boards() {
        for board in [
            two_pin_nets(4, true),
            plane_with_pads(),
            bga_field(4, 3, 0.8, 0.4, true),
            bga_field(2, 2, 1.0, 0.5, false),
        ] {
            assert_eq!(board.validate(), Ok(()));
        }
    }

    #[test]
    fn builder_groups_pads_by_footprint_and_sorts_nets() {
        let mut b = BoardBuilder::new(10.0, 10.0, &["F.Cu", "B.Cu"]);
        let a = b.footprint("A1", (1.0, 1.0));
        let c = b.footprint("C1", (5.0, 5.0));
        b.smd_pad(c, "1", "ZED", (5.0, 5.0), (1.0, 1.0), "F.Cu");
        b.smd_pad(a, "1", "ALPHA", (1.0, 1.0), (1.0, 1.0), "B.Cu");
        b.smd_pad(c, "2", "", (6.0, 5.0), (1.0, 1.0), "F.Cu");
        let board = b.build();
        assert_eq!(board.footprints[0].pad_range, (0, 1));
        assert_eq!(board.footprints[1].pad_range, (1, 3));
        assert_eq!(board.nets[0].name, "ALPHA");
        assert_eq!(board.pads[0].net, Some(NetId(0)));
        assert_eq!(board.pads[2].net, None);
    }

    #[test]
    fn bga_field_has_locked_escapes_inside_the_courtyard() {
        let board = bga_field(3, 3, 0.8, 0.4, true);
        assert_eq!(board.pads.len(), 9);
        assert_eq!(board.vias.len(), 9);
        assert!(board.vias.iter().all(|v| v.ownership == Ownership::Locked));
        assert_eq!(board.footprints[0].courtyards.len(), 1);
        assert_eq!(board.nets.len(), 9);
    }

    #[test]
    fn uniform_rules_answer_constants() {
        let r = UniformRules::default();
        let s = RuleSubject::new(ItemKind::Track, Some(NetId(0)));
        assert_eq!(r.clearance(&s, &s, LayerId(0)), 200_000);
        assert_eq!(r.via(&s).min_annular, 150_000);
        assert_eq!(r.max_clearance(), 350_000);
        let lane = LayerMask::single(LayerId(0));
        assert!(lane.contains(LayerId(0)));
    }
}
