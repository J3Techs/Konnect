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
    assert!(r.stats.objects >= 2);
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
