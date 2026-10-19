//! Synthetic tests for the internal checker. Every board is generated here
//! with invented names and values (see [`crate::synthetic`]).

use super::*;
use crate::api::{ArcId, Generation, LayerMask, Ownership};
use crate::board::{ArcTrack, Drill, KeepoutRules};
use crate::plan::{RoutedTrack, RoutedVia};
use crate::synthetic::{bga_field, BoardBuilder, UniformRules};
use crate::ProjectRuleTable;
use konnect_geom::Polygon;

fn mm(v: f64) -> Coord {
    konnect_geom::mm_to_nm(v).unwrap()
}

fn pt(x: f64, y: f64) -> Point {
    Point::from_mm(x, y).unwrap()
}

fn rules() -> UniformRules {
    UniformRules::default()
}

fn kinds(r: &CheckReport) -> Vec<ViolationKind> {
    r.violations.iter().map(|v| v.kind).collect()
}

fn board_check(b: &RouteBoard, rules: &dyn RuleTable) -> CheckReport {
    check_board(b, rules, CheckOptions::default())
}

/// Two parallel 0.15 mm tracks whose edges are `gap_nm` apart.
fn parallel_tracks(gap_nm: Coord) -> RouteBoard {
    let mut b = BoardBuilder::new(20.0, 20.0, &["F.Cu", "B.Cu"]);
    b.track("A", "F.Cu", (2.0, 5.0), (12.0, 5.0), 0.15, false);
    b.track("B", "F.Cu", (2.0, 6.0), (12.0, 6.0), 0.15, false);
    let mut board = b.build();
    // Edge-to-edge gap: centre spacing minus one width.
    let y = mm(5.0) + mm(0.15) + gap_nm;
    board.tracks[1].seg.a.y = y;
    board.tracks[1].seg.b.y = y;
    board
}

#[test]
fn a_clean_board_has_no_violations() {
    let b = parallel_tracks(mm(0.5));
    let r = board_check(&b, &rules());
    assert!(r.violations.is_empty(), "{:?}", r.violations);
    let (_, stats) =
        BoardChecker::new(&b, &rules(), CheckOptions::default()).check_board_with_stats();
    assert!(stats.objects >= 2);
}

#[test]
fn clearance_is_exact_to_the_nanometre() {
    let at = board_check(&parallel_tracks(mm(0.2)), &rules());
    assert!(at.violations.is_empty(), "a gap at the rule is legal");
    let under = board_check(&parallel_tracks(mm(0.2) - 1), &rules());
    assert_eq!(kinds(&under), vec![ViolationKind::Clearance]);
    let v = &under.violations[0];
    assert_eq!(v.required, mm(0.2));
    assert_eq!(v.layer, Some(LayerId(0)));
    assert_eq!(
        (v.a, v.b),
        (
            CheckedItem::Board {
                item: ItemRef::Track(crate::api::TrackId(0))
            },
            Some(CheckedItem::Board {
                item: ItemRef::Track(crate::api::TrackId(1))
            })
        )
    );
    // The marker sits between the two tracks.
    assert!(v.at.y > mm(5.0) && v.at.y < mm(5.0) + mm(0.35));
}

#[test]
fn margin_and_kicad_epsilon_move_the_threshold() {
    let b = parallel_tracks(mm(0.2));
    let strict = CheckOptions::default();
    assert!(check_board(&b, &rules(), strict).violations.is_empty());
    let margin = strict.with_safety_margin_nm(1_000);
    assert_eq!(margin.safety_margin_nm, SAFETY_MARGIN_MIN_NM);
    assert_eq!(
        kinds(&check_board(&b, &rules(), margin)),
        vec![ViolationKind::Clearance]
    );
    assert_eq!(
        strict.with_safety_margin_nm(50_000).safety_margin_nm,
        SAFETY_MARGIN_MAX_NM
    );
    // 400 nm under the rule: strict flags it, KiCad's epsilon does not.
    let b = parallel_tracks(mm(0.2) - 400);
    assert_eq!(
        kinds(&check_board(&b, &rules(), strict)),
        vec![ViolationKind::Clearance]
    );
    assert!(check_board(&b, &rules(), strict.kicad_equivalent())
        .violations
        .is_empty());
    let b = parallel_tracks(mm(0.2) - 600);
    assert_eq!(
        kinds(&check_board(&b, &rules(), strict.kicad_equivalent())),
        vec![ViolationKind::Clearance]
    );
}

#[test]
fn overlapping_copper_of_two_nets_is_a_short_and_one_net_is_fine() {
    let mut b = BoardBuilder::new(20.0, 20.0, &["F.Cu", "B.Cu"]);
    b.track("A", "F.Cu", (2.0, 5.0), (12.0, 5.0), 0.15, false);
    b.track("B", "F.Cu", (7.0, 2.0), (7.0, 8.0), 0.15, false);
    b.track("A", "F.Cu", (2.0, 5.0), (2.0, 9.0), 0.15, false);
    let r = board_check(&b.build(), &rules());
    assert_eq!(kinds(&r), vec![ViolationKind::Short]);
}

#[test]
fn copper_on_different_layers_does_not_interact() {
    let mut b = BoardBuilder::new(20.0, 20.0, &["F.Cu", "B.Cu"]);
    b.track("A", "F.Cu", (2.0, 5.0), (12.0, 5.0), 0.15, false);
    b.track("B", "B.Cu", (2.0, 5.0), (12.0, 5.0), 0.15, false);
    assert!(board_check(&b.build(), &rules()).violations.is_empty());
}

#[test]
fn track_width_floor() {
    let mut b = BoardBuilder::new(20.0, 20.0, &["F.Cu", "B.Cu"]);
    b.track("A", "F.Cu", (2.0, 5.0), (12.0, 5.0), 0.12, false);
    let r = board_check(&b.build(), &rules());
    assert_eq!(kinds(&r), vec![ViolationKind::TrackWidth]);
    assert_eq!(r.violations[0].required, mm(0.15));
}

#[test]
fn edge_clearance() {
    let mut b = BoardBuilder::new(20.0, 20.0, &["F.Cu", "B.Cu"]);
    // Edge of the copper 0.29 mm from the board edge at y = 0.
    b.track("A", "F.Cu", (2.0, 0.365), (12.0, 0.365), 0.15, false);
    b.track("B", "F.Cu", (2.0, 10.0), (12.0, 10.0), 0.15, false);
    let r = board_check(&b.build(), &rules());
    assert_eq!(kinds(&r), vec![ViolationKind::EdgeClearance]);
    assert_eq!(r.violations[0].b, Some(CheckedItem::Edge));
    assert_eq!(r.violations[0].required, mm(0.3));
}

#[test]
fn via_size_and_hole_to_hole() {
    let mut b = BoardBuilder::new(20.0, 20.0, &["F.Cu", "B.Cu"]);
    b.via("A", (5.0, 5.0), 0.5, 0.3, false);
    // Same net, holes 0.34 mm apart edge to edge: hole-to-hole only.
    b.via("B", (10.0, 10.0), 0.6, 0.3, false);
    b.via("B", (10.64, 10.0), 0.6, 0.3, false);
    let r = board_check(&b.build(), &rules());
    assert_eq!(
        kinds(&r),
        vec![ViolationKind::ViaSize, ViolationKind::HoleToHole]
    );
    assert_eq!(r.violations[1].required, mm(0.35));
    assert_eq!(r.violations[1].layer, None);
}

#[test]
fn non_plated_holes_are_checked_against_any_copper() {
    let mut b = BoardBuilder::new(20.0, 20.0, &["F.Cu", "B.Cu"]);
    let f = b.footprint("H1", (10.0, 10.0));
    b.th_pad(f, "", "", (10.0, 10.0), 1.0, 1.0);
    // Track edge 0.2 mm from the 1.0 mm hole: under the 0.25 hole rule.
    b.track("A", "F.Cu", (5.0, 10.775), (15.0, 10.775), 0.15, false);
    let mut board = b.build();
    let pad = &mut board.pads[0];
    pad.kind = PadKind::NpThroughHole;
    pad.drill = Some(Drill {
        shape: pad.drill.as_ref().unwrap().shape.clone(),
        plated: false,
    });
    let r = board_check(&board, &rules());
    // The pad is no larger than its hole: no copper, only the hole counts.
    assert_eq!(kinds(&r), vec![ViolationKind::HoleClearance]);
    assert_eq!(r.violations[0].required, mm(0.25));
    assert_eq!(r.violations[0].layer, Some(LayerId(0)));
}

#[test]
fn plated_holes_of_the_same_net_are_exempt() {
    let mut b = BoardBuilder::new(20.0, 20.0, &["F.Cu", "B.Cu"]);
    b.via("A", (10.0, 10.0), 0.6, 0.3, false);
    b.track("A", "F.Cu", (10.0, 10.0), (15.0, 10.0), 0.15, false);
    assert!(board_check(&b.build(), &rules()).violations.is_empty());
}

#[test]
fn segment_length_floor() {
    let mut b = BoardBuilder::new(20.0, 20.0, &["F.Cu", "B.Cu"]);
    b.track("A", "F.Cu", (2.0, 5.0), (2.05, 5.0), 0.15, false);
    b.track("A", "F.Cu", (2.05, 5.0), (8.0, 5.0), 0.15, false);
    let rules = UniformRules {
        min_segment_length: Some(mm(0.1)),
        ..UniformRules::default()
    };
    let r = board_check(&b.build(), &rules);
    assert_eq!(kinds(&r), vec![ViolationKind::SegmentLength]);
    assert_eq!(
        r.violations[0].a,
        CheckedItem::Board {
            item: ItemRef::Track(crate::api::TrackId(0))
        }
    );
}

#[test]
fn track_angles_follow_kicad() {
    let rules = UniformRules {
        min_track_angle_deg: Some(89.0),
        ..UniformRules::default()
    };
    // Acute 45° bend, a right angle, and a straight run.
    let mut b = BoardBuilder::new(30.0, 30.0, &["F.Cu", "B.Cu"]);
    b.track("A", "F.Cu", (2.0, 2.0), (6.0, 2.0), 0.15, false);
    b.track("A", "F.Cu", (6.0, 2.0), (3.0, 5.0), 0.15, false);
    b.track("B", "F.Cu", (2.0, 10.0), (6.0, 10.0), 0.15, false);
    b.track("B", "F.Cu", (6.0, 10.0), (6.0, 14.0), 0.15, false);
    b.track("C", "F.Cu", (2.0, 20.0), (6.0, 20.0), 0.15, false);
    b.track("C", "F.Cu", (6.0, 20.0), (9.0, 20.0), 0.15, false);
    let r = board_check(&b.build(), &rules);
    assert_eq!(kinds(&r), vec![ViolationKind::TrackAngle]);
    assert_eq!(r.violations[0].at, pt(6.0, 2.0));
    assert_eq!(r.violations[0].required, 890);
}

#[test]
fn track_angles_pair_up_three_way_junctions_and_skip_pads() {
    let rules = UniformRules {
        min_track_angle_deg: Some(89.0),
        ..UniformRules::default()
    };
    let mut b = BoardBuilder::new(30.0, 30.0, &["F.Cu", "B.Cu"]);
    // Three ends at (6, 6): left, right and up-right at 45° to the right one.
    b.track("A", "F.Cu", (2.0, 6.0), (6.0, 6.0), 0.15, false);
    b.track("A", "F.Cu", (6.0, 6.0), (10.0, 6.0), 0.15, false);
    b.track("A", "F.Cu", (6.0, 6.0), (9.0, 3.0), 0.15, false);
    // The same acute bend on a pad: a connection, not a bend.
    let f = b.footprint("P1", (6.0, 16.0));
    b.smd_pad(f, "1", "B", (6.0, 16.0), (0.6, 0.6), "F.Cu");
    b.track("B", "F.Cu", (2.0, 16.0), (6.0, 16.0), 0.15, false);
    b.track("B", "F.Cu", (6.0, 16.0), (3.0, 19.0), 0.15, false);
    // ... and on a via, where it still counts.
    b.via("C", (20.0, 6.0), 0.6, 0.3, false);
    b.track("C", "F.Cu", (16.0, 6.0), (20.0, 6.0), 0.15, false);
    b.track("C", "F.Cu", (20.0, 6.0), (17.0, 9.0), 0.15, false);
    // An overlapping same-direction run is not a bend.
    b.track("D", "F.Cu", (2.0, 25.0), (6.0, 25.0), 0.15, false);
    b.track("D", "F.Cu", (2.0, 25.0), (4.0, 25.0), 0.15, false);
    let r = board_check(&b.build(), &rules);
    let at: Vec<Point> = r
        .violations
        .iter()
        .filter(|v| v.kind == ViolationKind::TrackAngle)
        .map(|v| v.at)
        .collect();
    assert_eq!(at, vec![pt(6.0, 6.0), pt(20.0, 6.0)], "{:?}", r.violations);
}

#[test]
fn keepouts_forbid_only_what_they_name() {
    let mut b = BoardBuilder::new(20.0, 20.0, &["F.Cu", "B.Cu"]);
    b.keepout(
        "no tracks",
        &["F.Cu"],
        (8.0, 2.0, 12.0, 8.0),
        KeepoutRules {
            tracks: true,
            ..KeepoutRules::default()
        },
    );
    b.track("A", "F.Cu", (2.0, 5.0), (18.0, 5.0), 0.15, false);
    b.track("B", "B.Cu", (2.0, 5.0), (18.0, 5.0), 0.15, false);
    b.via("C", (10.0, 3.0), 0.6, 0.3, false);
    let r = board_check(&b.build(), &rules());
    assert_eq!(kinds(&r), vec![ViolationKind::Keepout]);
    assert_eq!(
        r.violations[0].a,
        CheckedItem::Board {
            item: ItemRef::Track(crate::api::TrackId(0))
        }
    );
}

#[test]
fn zone_fills_are_copper_unless_left_to_a_refill() {
    let mut b = BoardBuilder::new(20.0, 20.0, &["F.Cu", "B.Cu"]);
    b.zone("GND", "B.Cu", (1.0, 1.0, 19.0, 19.0));
    b.track("A", "B.Cu", (2.0, 5.0), (12.0, 5.0), 0.15, false);
    // Same net as the plane: fine.
    b.track("GND", "B.Cu", (2.0, 8.0), (12.0, 8.0), 0.15, false);
    let board = b.build();
    let r = board_check(&board, &rules());
    assert_eq!(kinds(&r), vec![ViolationKind::Short]);
    let refill = CheckOptions {
        check_zone_fills: false,
        ..CheckOptions::default()
    };
    assert!(check_board(&board, &rules(), refill).violations.is_empty());

    // A plane with a cut-out around the track: clearance to the cut-out.
    let mut board = board;
    let ring =
        |x0: f64, y0: f64, x1: f64, y1: f64| vec![pt(x0, y0), pt(x1, y0), pt(x1, y1), pt(x0, y1)];
    // Track edge at 4.925 .. 5.075; cut-out 0.15 above and below.
    board.zones[0].region = Polygon::new(
        ring(1.0, 1.0, 19.0, 19.0),
        vec![ring(1.5, 4.775, 12.5, 5.225), ring(1.5, 7.0, 12.5, 9.0)],
    )
    .unwrap();
    let r = board_check(&board, &rules());
    assert_eq!(kinds(&r), vec![ViolationKind::Clearance]);
    assert_eq!(r.violations[0].required, mm(0.2));
}

#[test]
fn routed_copper_is_checked_against_the_board_and_itself_only() {
    let mut b = BoardBuilder::new(20.0, 20.0, &["F.Cu", "B.Cu"]);
    // A board-board violation the routed check must not report.
    b.track("A", "F.Cu", (2.0, 2.0), (12.0, 2.0), 0.15, false);
    b.track("B", "F.Cu", (2.0, 2.3), (5.0, 2.3), 0.15, false);
    b.net("C");
    let board = b.build();
    let net_c = board.net_by_name("C").unwrap();
    let net_a = board.net_by_name("A").unwrap();
    let gen = Generation(1);
    let copper = RoutedCopper {
        tracks: vec![
            // 0.1 mm from board track B's edge: a violation.
            RoutedTrack {
                net: net_c,
                layer: LayerId(0),
                seg: Segment::new(pt(2.0, 2.55), pt(5.0, 2.55), mm(0.15)),
                gen,
            },
            // Far away, but its 45° bend with the next one is acute.
            RoutedTrack {
                net: net_c,
                layer: LayerId(1),
                seg: Segment::new(pt(2.0, 10.0), pt(6.0, 10.0), mm(0.15)),
                gen,
            },
            RoutedTrack {
                net: net_c,
                layer: LayerId(1),
                seg: Segment::new(pt(6.0, 10.0), pt(3.0, 13.0), mm(0.15)),
                gen,
            },
        ],
        vias: vec![
            // On board track A: a short, and its hole is in A's copper.
            RoutedVia {
                net: net_c,
                center: pt(9.0, 2.0),
                diameter: mm(0.6),
                drill: mm(0.3),
                layers: board.layers.all(),
                gen,
            },
            // Undersized and on net A: a via-size finding only.
            RoutedVia {
                net: net_a,
                center: pt(15.0, 15.0),
                diameter: mm(0.5),
                drill: mm(0.3),
                layers: board.layers.all(),
                gen,
            },
        ],
    };
    let r = check_routed(&board, &rules(), &copper);
    let got: Vec<(ViolationKind, CheckedItem)> =
        r.violations.iter().map(|v| (v.kind, v.a)).collect();
    assert_eq!(
        got,
        vec![
            (
                ViolationKind::Clearance,
                CheckedItem::RoutedTrack { index: 0 }
            ),
            (ViolationKind::Short, CheckedItem::RoutedVia { index: 0 }),
            (ViolationKind::ViaSize, CheckedItem::RoutedVia { index: 1 }),
            (
                ViolationKind::HoleClearance,
                CheckedItem::RoutedVia { index: 0 }
            ),
            (
                ViolationKind::TrackAngle,
                CheckedItem::RoutedTrack { index: 1 }
            ),
        ],
        "{:?}",
        r.violations
    );
    // The board check sees the board pair instead.
    assert_eq!(
        kinds(&board_check(&board, &rules())),
        vec![ViolationKind::Clearance]
    );
}

#[test]
fn arcs_are_checked_with_their_chord_error() {
    let mut b = BoardBuilder::new(20.0, 20.0, &["F.Cu", "B.Cu"]);
    let f = b.footprint("P1", (10.0, 10.0));
    b.ball_pad(f, "1", "B", (10.0, 10.0), 0.5, "F.Cu");
    b.net("A");
    let mut board = b.build();
    // A quarter arc of radius 1 mm around the pad centre, 0.1 mm wide: its
    // copper is 1 - 0.05 - 0.25 = 0.7 mm from the pad. Raise the rule to
    // just under that so only the chord-error allowance trips it.
    let r = mm(1.0);
    let c = pt(10.0, 10.0);
    board.arcs.push(ArcTrack {
        id: ArcId(0),
        net: board.net_by_name("A"),
        layer: LayerId(0),
        start: Point::new(c.x + r, c.y),
        mid: Point::new(c.x + 707_107, c.y + 707_107),
        end: Point::new(c.x, c.y + r),
        width: mm(0.1),
        ownership: Ownership::Fixed,
        uuid: None,
    });
    let mut rules = rules();
    rules.class.clearance = mm(0.7) - 500;
    rules.class.track_width = mm(0.1);
    let report = board_check(&board, &rules);
    // True gap 0.7 mm > 0.6995 rule, but the chords sit up to 1 µm inside
    // the curve and the checker may not trust them that far.
    assert_eq!(kinds(&report), vec![ViolationKind::Clearance]);
    rules.class.clearance = mm(0.7) - 2_000;
    assert!(board_check(&board, &rules).violations.is_empty());
}

#[test]
fn reports_are_identical_on_one_thread_and_many() {
    let mut board = bga_field(14, 14, 0.8, 0.4, true);
    // Squeeze half the dog-bone tracks sideways so there is plenty to find.
    for (i, t) in board.tracks.iter_mut().enumerate() {
        if i % 2 == 0 {
            t.seg.b.x += mm(0.31);
        }
    }
    let run = |threads: usize| {
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap()
            .install(|| board_check(&board, &rules()))
    };
    let one = run(1);
    assert!(!one.violations.is_empty());
    assert_eq!(one, run(4));
    assert_eq!(one, run(7));
    let mut telemetry = Telemetry::default();
    let timed = BoardChecker::new(&board, &rules(), CheckOptions::default())
        .check_board_timed(&mut telemetry);
    assert_eq!(timed, one);
    assert_eq!(
        telemetry.counters["check_violations"],
        one.violations.len() as u64
    );
    assert!(telemetry
        .stages
        .iter()
        .any(|s| s.stage == crate::api::Stage::Check));
}

// ─── Project rules: courtyard and area conditions reach the checker ──────────

const PRO: &str = r#"{
    "board": { "design_settings": { "rules": {
        "min_clearance": 0.05, "min_track_width": 0.05, "min_via_diameter": 0.2,
        "min_via_annular_width": 0.05, "min_through_hole_diameter": 0.1,
        "min_copper_edge_clearance": 0.25, "min_hole_clearance": 0.15, "min_hole_to_hole": 0.2,
        "max_error": 0.005
    } } },
    "net_settings": {
        "classes": [
            { "name": "Default", "clearance": 0.12, "track_width": 0.1,
              "via_diameter": 0.5, "via_drill": 0.25 },
            { "name": "Tight", "clearance": 0.08 }
        ],
        "netclass_patterns": [ { "netclass": "Tight", "pattern": "T_*" } ]
    }
}"#;

const DRU: &str = r#"(version 1)
(rule "Wide inside IC4"
  (condition "A.intersectsCourtyard('IC4') && B.intersectsCourtyard('IC4')")
  (constraint clearance (min 0.3mm)))
(rule "Narrow escape area"
  (condition "A.enclosedByArea('escape') && B.enclosedByArea('escape')")
  (constraint clearance (min 0.06mm)))
"#;

#[test]
fn courtyard_and_area_conditions_decide_the_clearance() {
    let mut b = BoardBuilder::new(40.0, 40.0, &["F.Cu", "B.Cu"]);
    b.footprint_with_courtyard("IC4", (10.0, 10.0), (4.0, 4.0));
    b.keepout(
        "escape",
        &["F.Cu"],
        (20.0, 20.0, 30.0, 30.0),
        KeepoutRules::default(),
    );
    // Inside IC4's courtyard, 0.25 mm apart: under the 0.3 courtyard rule.
    b.track("T_A", "F.Cu", (7.0, 10.0), (13.0, 10.0), 0.1, false);
    b.track("T_B", "F.Cu", (7.0, 10.35), (13.0, 10.35), 0.1, false);
    // The same spacing outside: the class (0.08) allows it.
    b.track("T_C", "F.Cu", (7.0, 30.0), (13.0, 30.0), 0.1, false);
    b.track("T_D", "F.Cu", (7.0, 30.35), (13.0, 30.35), 0.1, false);
    // Default nets 0.07 apart inside the area: the 0.06 area rule allows
    // it although the class says 0.12.
    b.track("N_E", "F.Cu", (22.0, 25.0), (28.0, 25.0), 0.1, false);
    b.track("N_F", "F.Cu", (22.0, 25.17), (28.0, 25.17), 0.1, false);
    // ... and one of them leaving the area loses the exemption.
    b.track("N_G", "F.Cu", (22.0, 22.0), (31.0, 22.0), 0.1, false);
    b.track("N_H", "F.Cu", (22.0, 22.17), (28.0, 22.17), 0.1, false);
    let mut board = b.build();
    let table = ProjectRuleTable::from_project(PRO, Some(DRU), &board).unwrap();
    table.assign_classes(&mut board);
    assert!(table.inert_rules().is_empty(), "{:?}", table.inert_rules());
    let checker = BoardChecker::new(&board, &table, CheckOptions::default());
    let r = checker.check_board();
    let net = |c: CheckedItem| match c {
        CheckedItem::Board { item } => board.net(board.net_of(item).unwrap()).name.clone(),
        _ => String::new(),
    };
    let found: Vec<(String, String, Coord)> = r
        .violations
        .iter()
        .map(|v| (net(v.a), net(v.b.unwrap()), v.required))
        .collect();
    assert_eq!(
        found,
        vec![
            ("T_A".to_string(), "T_B".to_string(), mm(0.3)),
            ("N_G".to_string(), "N_H".to_string(), mm(0.12)),
        ]
    );
    let ic4 = board.footprint_by_reference("IC4").unwrap();
    let (courts, areas) = checker.subject_context(ItemRef::Track(crate::api::TrackId(0)));
    assert_eq!(courts, &[ic4]);
    assert!(areas.is_empty());
    let (_, areas) = checker.subject_context(ItemRef::Track(crate::api::TrackId(4)));
    assert_eq!(areas, &["escape"]);
}

#[test]
fn via_floor_follows_courtyard_conditions() {
    const DRU_VIA: &str = r#"(version 1)
(rule "Big vias away from IC4"
  (condition "A.Type == 'Via' && !A.intersectsCourtyard('IC4')")
  (constraint via_diameter (min 0.45mm)))
"#;
    let mut b = BoardBuilder::new(40.0, 40.0, &["F.Cu", "B.Cu"]);
    b.footprint_with_courtyard("IC4", (10.0, 10.0), (4.0, 4.0));
    b.via("A", (10.0, 10.0), 0.35, 0.2, false);
    b.via("B", (30.0, 30.0), 0.35, 0.2, false);
    let board = b.build();
    let table = ProjectRuleTable::from_project(PRO, Some(DRU_VIA), &board).unwrap();
    let r = check_board(&board, &table, CheckOptions::default());
    assert_eq!(kinds(&r), vec![ViolationKind::ViaSize]);
    assert_eq!(
        r.violations[0].a,
        CheckedItem::Board {
            item: ItemRef::Via(crate::api::ViaId(1))
        }
    );
    let _ = LayerMask::EMPTY;
}

// ─── Review fixes: shorts at zero clearance, rule order, exact sizes ─────────

fn zero_clearance() -> UniformRules {
    let mut r = UniformRules::default();
    r.class.clearance = 0;
    r
}

#[test]
fn shorts_survive_a_zero_or_sub_epsilon_clearance() {
    // Two overlapping fills of different nets: sharp cores, so the overlap
    // compares Equal at a zero threshold.
    let mut b = BoardBuilder::new(20.0, 20.0, &["F.Cu", "B.Cu"]);
    b.zone("A", "B.Cu", (1.0, 1.0, 10.0, 10.0));
    b.zone("B", "B.Cu", (8.0, 8.0, 19.0, 19.0));
    let zones = b.build();
    let strict = CheckOptions::default();
    for (rules, options) in [
        (rules(), strict),
        (zero_clearance(), strict),
        (zero_clearance(), strict.kicad_equivalent()),
    ] {
        let r = check_board(&zones, &rules, options);
        assert_eq!(kinds(&r), vec![ViolationKind::Short], "{options:?}");
    }
    // A 300 nm rule under KiCad's 500 nm epsilon still finds the short.
    let mut tiny = UniformRules::default();
    tiny.class.clearance = 300;
    assert_eq!(
        kinds(&check_board(&zones, &tiny, strict.kicad_equivalent())),
        vec![ViolationKind::Short]
    );

    // Sharp pads of two nets: overlapping, then sharing an edge (touching
    // is a short too), then 1 nm apart (legal at clearance 0).
    for (dx, want) in [
        (-1000, vec![ViolationKind::Short]),
        (0, vec![ViolationKind::Short]),
        (1, vec![]),
    ] {
        let mut b = BoardBuilder::new(20.0, 20.0, &["F.Cu", "B.Cu"]);
        let f = b.footprint("IC2", (5.0, 5.0));
        b.smd_pad(f, "1", "A", (5.0, 5.0), (1.0, 1.0), "F.Cu");
        b.smd_pad(f, "2", "B", (6.0, 5.0), (1.0, 1.0), "F.Cu");
        let mut board = b.build();
        board.pads[1].shapes[0] = Shape::ConvexPoly(
            konnect_geom::ConvexPoly::axis_rect(
                BBox::new(
                    Point::new(pt(5.5, 4.5).x + dx, pt(5.5, 4.5).y),
                    pt(6.5, 5.5),
                ),
                0,
            )
            .unwrap(),
        );
        let r = check_board(&board, &zero_clearance(), strict);
        assert_eq!(kinds(&r), want, "dx {dx}");
    }

    // Same net overlapping: nothing.
    let mut b = BoardBuilder::new(20.0, 20.0, &["F.Cu", "B.Cu"]);
    b.zone("A", "B.Cu", (1.0, 1.0, 10.0, 10.0));
    b.zone("A", "B.Cu", (8.0, 8.0, 19.0, 19.0));
    assert!(check_board(&b.build(), &zero_clearance(), strict)
        .violations
        .is_empty());
}

/// A table whose clearance depends on argument order (only a misbehaving
/// table does this): the checker must still be deterministic.
struct Lopsided(UniformRules);

impl RuleTable for Lopsided {
    fn class_of(&self, net: Option<NetId>) -> crate::api::ClassId {
        self.0.class_of(net)
    }
    fn class(&self, class: crate::api::ClassId) -> &crate::rules::NetClassRules {
        self.0.class(class)
    }
    fn clearance(&self, a: &RuleSubject<'_>, b: &RuleSubject<'_>, _layer: LayerId) -> Coord {
        if a.net < b.net {
            mm(0.3)
        } else {
            mm(0.1)
        }
    }
    fn track_width(&self, s: &RuleSubject<'_>, layer: LayerId) -> crate::rules::WidthRule {
        self.0.track_width(s, layer)
    }
    fn via(&self, s: &RuleSubject<'_>) -> crate::rules::ViaRule {
        self.0.via(s)
    }
    fn min_track_angle_deg(&self, s: &RuleSubject<'_>, layer: LayerId) -> Option<f64> {
        self.0.min_track_angle_deg(s, layer)
    }
    fn min_segment_length(&self, s: &RuleSubject<'_>, layer: LayerId) -> Option<Coord> {
        self.0.min_segment_length(s, layer)
    }
    fn edge_clearance(&self) -> Coord {
        self.0.edge_clearance()
    }
    fn hole_clearance(&self) -> Coord {
        self.0.hole_clearance()
    }
    fn hole_to_hole(&self) -> Coord {
        self.0.hole_to_hole()
    }
    fn max_clearance(&self) -> Coord {
        mm(0.3)
    }
    fn inert_rules(&self) -> &[crate::rules::InertRule] {
        self.0.inert_rules()
    }
}

#[test]
fn an_order_dependent_table_still_gives_one_answer_on_any_thread_count() {
    // Rows of tracks, 0.2 mm apart edge to edge, nets in mixed order so the
    // same subject pair is met both ways round.
    let mut b = BoardBuilder::new(60.0, 60.0, &["F.Cu", "B.Cu"]);
    for col in 0..6 {
        let x = 1.0 + 9.5 * f64::from(col);
        for row in 0..150 {
            // A, B, B, A, A, B, B, A, ...: neighbours meet both ways round.
            let net = if ((row + 1) / 2) % 2 == 0 { "A" } else { "B" };
            let y = 1.0 + 0.35 * f64::from(row);
            b.track(net, "F.Cu", (x, y), (x + 8.0, y), 0.15, false);
        }
    }
    let board = b.build();
    let rules = Lopsided(UniformRules::default());
    let run = |threads: usize| {
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap()
            .install(|| check_board(&board, &rules, CheckOptions::default()))
    };
    let one = run(1);
    for threads in [2, 4, 7, 16] {
        assert_eq!(one, run(threads), "{threads} threads");
    }
    // Every finding names the same rule value: the table is asked one way.
    let values: std::collections::BTreeSet<Coord> =
        one.violations.iter().map(|v| v.required).collect();
    assert!(values.len() <= 1, "{values:?}");
}

#[test]
fn via_size_reports_the_floor_that_failed() {
    // 0.7 / 0.45: diameter and drill pass, the 0.125 ring is under 0.15.
    let mut b = BoardBuilder::new(20.0, 20.0, &["F.Cu", "B.Cu"]);
    b.via("A", (5.0, 5.0), 0.7, 0.45, false);
    b.via("B", (10.0, 5.0), 0.5, 0.3, false);
    b.via("C", (15.0, 5.0), 0.6, 0.2, false);
    let r = board_check(&b.build(), &rules());
    let sizes: Vec<Coord> = r
        .violations
        .iter()
        .filter(|v| v.kind == ViolationKind::ViaSize)
        .map(|v| v.required)
        .collect();
    assert_eq!(sizes, vec![mm(0.15), mm(0.6), mm(0.3)]);
}

#[test]
fn odd_widths_and_diameters_stay_exact() {
    // Two 150_001 nm tracks: half widths sum to 150_001, so centre lines
    // 350_001 apart leave exactly the 0.2 mm rule.
    for (spacing, want) in [(350_001, vec![]), (350_000, vec![ViolationKind::Clearance])] {
        let mut board = parallel_tracks(mm(0.5));
        for t in &mut board.tracks {
            t.seg.width = 150_001;
        }
        let y = board.tracks[0].seg.a.y + spacing;
        board.tracks[1].seg.a.y = y;
        board.tracks[1].seg.b.y = y;
        assert_eq!(kinds(&board_check(&board, &rules())), want, "{spacing}");
    }
    // A 600_001 nm via (radius rounded up to 300_001) by a 0.15 mm track:
    // the true gap is 200_000.5 at centre distance 575_001 (legal) and
    // 199_999.5 at 575_000 (a violation); rounding up never makes a legal
    // verdict of a real violation.
    for (dy, want) in [(575_001, vec![]), (575_000, vec![ViolationKind::Clearance])] {
        let mut b = BoardBuilder::new(20.0, 20.0, &["F.Cu", "B.Cu"]);
        b.via("A", (10.0, 5.0), 0.6, 0.3, false);
        b.track("B", "F.Cu", (2.0, 8.0), (18.0, 8.0), 0.15, false);
        let mut board = b.build();
        board.vias[0].diameter = 600_001;
        let y = board.vias[0].center.y + dy;
        board.tracks[0].seg.a.y = y;
        board.tracks[0].seg.b.y = y;
        let r = board_check(&board, &rules());
        let copper: Vec<ViolationKind> = kinds(&r)
            .into_iter()
            .filter(|k| *k == ViolationKind::Clearance)
            .collect();
        assert_eq!(copper, want, "{dy}");
    }
}

#[test]
fn a_sharp_pad_in_a_plane_cut_out() {
    // A 1 mm sharp pad inside a 1.4 mm cut-out: 0.2 mm all round, legal.
    let ring =
        |x0: f64, y0: f64, x1: f64, y1: f64| vec![pt(x0, y0), pt(x1, y0), pt(x1, y1), pt(x0, y1)];
    for (cut, want) in [
        (0.7, vec![]),
        (0.699_999, vec![ViolationKind::Clearance]),
        (0.4, vec![ViolationKind::Short]),
    ] {
        let mut b = BoardBuilder::new(20.0, 20.0, &["F.Cu", "B.Cu"]);
        b.zone("GND", "F.Cu", (1.0, 1.0, 19.0, 19.0));
        let f = b.footprint("IC2", (10.0, 10.0));
        b.smd_pad(f, "1", "A", (10.0, 10.0), (1.0, 1.0), "F.Cu");
        let mut board = b.build();
        board.zones[0].region = Polygon::new(
            ring(1.0, 1.0, 19.0, 19.0),
            vec![ring(10.0 - cut, 10.0 - cut, 10.0 + cut, 10.0 + cut)],
        )
        .unwrap();
        let r = board_check(&board, &rules());
        assert_eq!(kinds(&r), want, "cut-out half width {cut}");
    }
}
