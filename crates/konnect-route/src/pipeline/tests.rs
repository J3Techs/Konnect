//! Synthetic end-to-end runs of the pipeline. Every board is generated
//! here; nothing is real.

use super::*;
use crate::check::ViolationKind;
use crate::detail::negotiate::negotiate_connections;
use crate::synthetic::{plane_with_pads, two_pin_nets, BoardBuilder, UniformRules};
use konnect_geom::Point;

fn settings(threads: usize) -> PipelineSettings {
    PipelineSettings {
        threads,
        ..PipelineSettings::default()
    }
}

/// `n` nets from a left column to a right column in reverse order on two
/// layers: every net crosses every other, so a clean result needs vias and
/// negotiation.
fn crossing_bundle(n: usize) -> RouteBoard {
    let h = 2.0 * n as f64 + 2.0;
    let mut b = BoardBuilder::new(16.0, h, &["F.Cu", "B.Cu"]);
    let l = b.footprint("J1", (2.0, h / 2.0));
    let r = b.footprint("J2", (14.0, h / 2.0));
    for i in 0..n {
        let net = format!("X{i:02}");
        let yl = 2.0 + 2.0 * i as f64;
        let yr = 2.0 + 2.0 * (n - 1 - i) as f64;
        b.smd_pad(l, &(i + 1).to_string(), &net, (2.0, yl), (0.8, 0.5), "F.Cu");
        b.smd_pad(
            r,
            &(i + 1).to_string(),
            &net,
            (14.0, yr),
            (0.8, 0.5),
            "F.Cu",
        );
    }
    b.build()
}

fn assert_complete(r: &PipelineResult) {
    assert!(r.failed.is_empty(), "failed: {:?}", r.failed);
    assert!(
        r.check.violations.is_empty(),
        "S7 findings: {:?}",
        r.check.violations
    );
    assert_eq!(r.connections_after, 0, "every connection is joined");
    assert_eq!(
        r.detail.routed.len(),
        r.connections.len(),
        "negotiation routed every task"
    );
    assert_eq!(
        r.plan.item_count(),
        r.fanout_copper.tracks.len()
            + r.fanout_copper.vias.len()
            + r.routed.tracks.len()
            + r.routed.vias.len()
    );
}

#[test]
fn parallel_two_pin_nets_route_clean_and_complete() {
    let board = two_pin_nets(6, false);
    let rules = UniformRules::default();
    let r = route_board(&board, &rules, &PipelineScope::default(), &settings(2)).unwrap();
    assert_eq!(r.connections_before_fanout, 6);
    assert_complete(&r);
    // Stage timings were recorded for every stage that ran.
    for stage in [
        Stage::Audit,
        Stage::Global,
        Stage::Detail,
        Stage::Optimize,
        Stage::Check,
        Stage::Plan,
    ] {
        assert!(
            r.telemetry.stages.iter().any(|s| s.stage == stage),
            "no timing for {stage:?}"
        );
    }
}

#[test]
fn a_crossing_bundle_routes_clean_with_vias_on_the_kernel() {
    let board = crossing_bundle(5);
    let rules = UniformRules::default();
    let r = route_board(&board, &rules, &PipelineScope::default(), &settings(2)).unwrap();
    assert_complete(&r);
    assert!(!r.routed.vias.is_empty(), "crossing nets need vias");
    assert!(r.global.corridors.len() == r.connections.len());
}

#[test]
fn plans_do_not_depend_on_the_thread_count() {
    let board = crossing_bundle(4);
    let rules = UniformRules::default();
    let one = route_board(&board, &rules, &PipelineScope::default(), &settings(1)).unwrap();
    let three = route_board(&board, &rules, &PipelineScope::default(), &settings(3)).unwrap();
    assert_eq!(
        one.plan.to_routing_file_json(),
        three.plan.to_routing_file_json()
    );
    assert_eq!(one.check, three.check);
}

#[test]
fn fanout_copper_joins_plane_pads_and_is_planned_but_not_rerouted() {
    let board = plane_with_pads();
    let rules = UniformRules::default();
    let r = route_board(&board, &rules, &PipelineScope::default(), &settings(2)).unwrap();
    let fanout = r.fanout.as_ref().expect("fanout ran");
    assert!(!fanout.sites.is_empty(), "plane pads get vias: {fanout:?}");
    assert!(!r.fanout_copper.vias.is_empty());
    // The plane connections the fanout joined are no longer tasks.
    assert!(r.connections.len() < r.connections_before_fanout);
    assert!(r.check.violations.is_empty(), "{:?}", r.check.violations);
    assert!(r.plan.vias.len() >= r.fanout_copper.vias.len());
    // Without the fanout the same pads are routed as ordinary connections.
    let bare = PipelineSettings {
        fanout: None,
        ..settings(2)
    };
    let b = route_board(&board, &rules, &PipelineScope::default(), &bare).unwrap();
    assert!(b.fanout.is_none() && b.fanout_copper.vias.is_empty());
    assert_eq!(b.connections.len(), b.connections_before_fanout);
}

#[test]
fn the_scope_limits_the_work_to_its_nets() {
    let board = two_pin_nets(4, false);
    let rules = UniformRules::default();
    let net = board.net_by_name("N02").unwrap();
    let scope = PipelineScope {
        nets: Some([net].into_iter().collect()),
        region: None,
    };
    let r = route_board(&board, &rules, &scope, &settings(1)).unwrap();
    assert_eq!(r.connections.len(), 1);
    assert_eq!(r.connections[0].net, net);
    assert!(r.routed.tracks.iter().all(|t| t.net == net));
    assert!(!r.routed.tracks.is_empty());
    assert_eq!(r.connections_after, 3, "the other nets stay open");
    // A region holding no whole connection selects nothing.
    let nowhere = PipelineScope {
        nets: None,
        region: Some(BBox::new(
            Point::new(0, 0),
            Point::new(1_000_000, 1_000_000),
        )),
    };
    let r = route_board(&board, &rules, &nowhere, &settings(1)).unwrap();
    assert!(r.connections.is_empty() && r.plan.item_count() == 0);
}

#[test]
fn the_guard_puts_back_a_net_the_optimizer_made_worse() {
    let board = two_pin_nets(2, false);
    let rules = UniformRules::default();
    let r = route_board(
        &board,
        &rules,
        &PipelineScope::default(),
        &PipelineSettings {
            optimize: None,
            ..settings(1)
        },
    )
    .unwrap();
    assert_complete(&r);
    let original = r.routed.clone();
    // An "optimized" copy that runs N00 straight through N01's pads.
    let n00 = board.net_by_name("N00").unwrap();
    let mut worse = original.clone();
    let t = worse
        .tracks
        .iter_mut()
        .find(|t| t.net == n00)
        .expect("N00 is routed");
    t.seg = Segment::new(
        Point::new(2_000_000, 4_000_000),
        Point::new(18_000_000, 4_000_000),
        t.seg.width,
    );
    let before = BoardChecker::new(&board, &rules, CheckOptions::for_routed())
        .check_routed(&worse)
        .violations;
    assert!(before
        .iter()
        .any(|v| matches!(v.kind, ViolationKind::Clearance | ViolationKind::Short)));
    let reverted = guard(&board, &rules, &original, &mut worse);
    assert_eq!(reverted, vec![n00]);
    let mut a = worse.tracks.clone();
    let mut b = original.tracks.clone();
    a.sort_by_key(|t| (t.net, t.layer, t.seg.a, t.seg.b));
    b.sort_by_key(|t| (t.net, t.layer, t.seg.a, t.seg.b));
    assert_eq!(a, b, "the worse net is back to its routed copper");
}

#[test]
fn a_connection_global_routing_failed_still_gets_a_detail_corridor() {
    let board = two_pin_nets(2, false);
    let rules = UniformRules::default();
    let connections = ConnectivityReport::build(&board, &ArcPolicy::default()).connections(&board);
    let global = GlobalResult {
        corridors: vec![Corridor {
            connection: connections[0].id,
            cells: vec![3],
            layers: board.routable,
            assigned: board.routable,
        }],
        failed: vec![(connections[1].id, FailureReason::NoCorridor)],
        ..GlobalResult::default()
    };
    let g = with_fallback_corridors(
        &board,
        &rules,
        &LayerPolicy::default(),
        &connections,
        &global,
    );
    assert_eq!(g.corridors.len(), 2);
    assert_eq!(g.corridors[1].connection, connections[1].id);
    assert!(g.corridors[1].cells.is_empty());
    assert_eq!(g.corridors[1].layers, board.routable);
    assert_eq!(g.corridors[0], global.corridors[0]);
}

#[test]
fn negotiation_on_the_kernel_resolves_a_crossing_bundle() {
    let board = crossing_bundle(6);
    let rules = UniformRules::default();
    let connections = ConnectivityReport::build(&board, &ArcPolicy::default()).connections(&board);
    // Iteration 0 blind to other nets forces conflicts to negotiate away.
    let tuning = NegotiationTuning {
        initial_present_factor: 0,
        ..NegotiationTuning::default()
    };
    let mut backend = KernelBackend::default();
    let mut telemetry = Telemetry::default();
    let r = negotiate_connections(
        &board,
        &rules,
        &connections,
        &GlobalResult::default(),
        None,
        &NegotiationSettings {
            threads: 2,
            max_iterations: 20,
            ..NegotiationSettings::default()
        },
        &tuning,
        &mut backend,
        &mut telemetry,
    );
    assert!(r.reports[0].conflict_pairs > 0, "{:?}", r.reports);
    assert!(r.failed.is_empty(), "{:?}", r.failure_details);
    assert_eq!(r.reports.last().unwrap().conflicted, 0);
    let check =
        BoardChecker::new(&board, &rules, CheckOptions::for_routed()).check_routed(&r.copper);
    assert!(check.violations.is_empty(), "{:?}", check.violations);
}

/// A net on F.Cu cut by a locked wall of another net on F.Cu: it can only
/// cross on B.Cu, through vias. A GND pour on In1 covers a band of the
/// board, the reference a fallback may demand.
fn walled_net() -> RouteBoard {
    let mut b = BoardBuilder::new(20.0, 10.0, &["F.Cu", "In1.Cu", "B.Cu"]);
    b.zone("GND", "In1.Cu", (0.5, 2.0, 19.5, 8.0));
    let l = b.footprint("J1", (3.0, 5.0));
    b.smd_pad(l, "1", "S", (3.0, 5.0), (0.6, 0.6), "F.Cu");
    let r = b.footprint("J2", (17.0, 5.0));
    b.smd_pad(r, "1", "S", (17.0, 5.0), (0.6, 0.6), "F.Cu");
    b.track("W", "F.Cu", (10.0, 0.0), (10.0, 10.0), 0.3, true);
    let mut board = b.build();
    board.routable = board
        .layers
        .expand("F.Cu")
        .with(board.layers.expand("B.Cu").iter().next().unwrap());
    board
}

#[test]
fn a_class_fallback_routes_on_more_layers_over_its_reference() {
    let board = walled_net();
    let rules = UniformRules::default();
    let f = board.layers.expand("F.Cu");
    let fb = f.union(board.layers.expand("B.Cu"));
    let in1 = board.layers.by_name("In1.Cu").unwrap();
    let mut s = settings(2);
    s.fanout = None;
    s.layer_policy.class_layers.insert(ClassId(0), f);
    let only_f = route_board(&board, &rules, &PipelineScope::default(), &s).unwrap();
    assert_eq!(only_f.failed.len(), 1, "the wall cuts F.Cu");

    s.class_fallback.insert(
        ClassId(0),
        ClassFallback {
            nets: None,
            layers: fb,
            reference: Some(ReferencePlane {
                plane: in1,
                over: board.layers.expand("B.Cu"),
                margin: 200_000,
            }),
        },
    );
    let r = route_board(&board, &rules, &PipelineScope::default(), &s).unwrap();
    assert!(r.failed.is_empty(), "{:?}", r.detail.failure_details);
    assert!(r.check.violations.is_empty(), "{:?}", r.check.violations);
    assert_eq!(r.connections_after, 0);
    assert_eq!(r.telemetry.counters["pipeline_fallback_routed"], 1);
    // Every B.Cu track stays over the pour, its edge 0.2 mm inside.
    let b_cu = board.layers.by_name("B.Cu").unwrap();
    let on_b: Vec<_> = r.routed.tracks.iter().filter(|t| t.layer == b_cu).collect();
    assert!(!on_b.is_empty());
    for t in on_b {
        for p in [t.seg.a, t.seg.b] {
            let edge = 200_000 + t.seg.width / 2;
            assert!(p.y - edge >= 2_000_000 && p.y + edge <= 8_000_000, "{t:?}");
        }
    }
}

// ─── Clearing what S7 still finds ───────────────────────────────────────────

/// Net `A` with pads at (2, 5) and (18, 5), and a third pad at (14, 9) when
/// `branch_pad`; the board carries no copper of its own.
fn clear_board(branch_pad: bool) -> RouteBoard {
    let mut b = BoardBuilder::new(20.0, 12.0, &["F.Cu", "B.Cu"]);
    let f = b.footprint("U1", (10.0, 6.0));
    b.smd_pad(f, "1", "A", (2.0, 5.0), (0.8, 0.5), "F.Cu");
    b.smd_pad(f, "2", "A", (18.0, 5.0), (0.8, 0.5), "F.Cu");
    if branch_pad {
        b.smd_pad(f, "3", "A", (14.0, 9.0), (0.5, 0.5), "F.Cu");
    }
    b.build()
}

/// A segment's two ends, mm.
type Ends = ((f64, f64), (f64, f64));

fn routed_tracks(board: &RouteBoard, segs: &[Ends]) -> RoutedCopper {
    let net = board.net_by_name("A").expect("net A");
    let mm = |v: f64| (v * 1e6).round() as konnect_geom::Coord;
    RoutedCopper {
        tracks: segs
            .iter()
            .map(|&(a, b)| RoutedTrack {
                net,
                layer: LayerId(0),
                seg: Segment::new(
                    Point::new(mm(a.0), mm(a.1)),
                    Point::new(mm(b.0), mm(b.1)),
                    150_000,
                ),
                gen: crate::api::Generation(0),
            })
            .collect(),
        vias: Vec::new(),
    }
}

fn angle_rules() -> UniformRules {
    UniformRules {
        min_track_angle_deg: Some(89.0),
        ..UniformRules::default()
    }
}

#[test]
fn clear_findings_drops_a_spare_track_that_crosses_its_net() {
    let board = clear_board(false);
    let rules = angle_rules();
    // A run joining the pads, crossed at 45° by a piece that joins nothing.
    let mut routed = routed_tracks(
        &board,
        &[((2.0, 5.0), (18.0, 5.0)), ((9.0, 4.0), (11.0, 6.0))],
    );
    let checker = BoardChecker::new(&board, &rules, CheckOptions::for_routed());
    assert!(checker
        .check_routed(&routed)
        .violations
        .iter()
        .any(|v| v.kind == ViolationKind::TrackAngle));
    let r = clear_findings(&board, &rules, &mut routed);
    assert_eq!(
        (r.single_tracks, r.cut_tracks, r.pieces),
        (1, 0, 0),
        "{r:?}"
    );
    assert_eq!(routed.tracks.len(), 1);
    assert!(checker.check_routed(&routed).violations.is_empty());
}

#[test]
fn clear_findings_cuts_the_track_that_opens_fewest_joins() {
    let board = clear_board(true);
    let rules = angle_rules();
    // The third pad's branch lands on the run's middle at 45°: neither is
    // spare, and cutting the branch opens one join where the run opens two.
    let mut routed = routed_tracks(
        &board,
        &[((2.0, 5.0), (18.0, 5.0)), ((14.0, 9.0), (10.0, 5.0))],
    );
    let r = clear_findings(&board, &rules, &mut routed);
    assert_eq!(
        (r.single_tracks, r.cut_tracks, r.opened),
        (0, 1, 1),
        "{r:?}"
    );
    assert_eq!(routed.tracks.len(), 1);
    assert_eq!(routed.tracks[0].seg.length(), 16_000_000);
    let checker = BoardChecker::new(&board, &rules, CheckOptions::for_routed());
    assert!(checker.check_routed(&routed).violations.is_empty());
}

#[test]
fn clear_findings_removes_spurs_but_not_tracks_that_carry_a_join() {
    let board = clear_board(true);
    let rules = angle_rules();
    // Right-angle spur off the run with a free end: it joins nothing. The
    // third pad's run (free at neither end) stays, as does the main run,
    // whose ends are on pads.
    let mut routed = routed_tracks(
        &board,
        &[
            ((2.0, 5.0), (14.0, 5.0)),
            ((14.0, 5.0), (18.0, 5.0)),
            ((14.0, 5.0), (14.0, 9.0)),
            ((6.0, 5.0), (6.0, 8.0)),
        ],
    );
    let r = clear_findings(&board, &rules, &mut routed);
    assert_eq!(r.spurs, 1, "{r:?}");
    assert_eq!(routed.tracks.len(), 3);
    assert!(routed.tracks.iter().all(|t| t.seg.a.x != 6_000_000));
}

#[test]
fn clear_findings_removes_a_via_joined_on_one_layer_and_its_spur() {
    let board = clear_board(false);
    let rules = angle_rules();
    let mut routed = routed_tracks(
        &board,
        &[((2.0, 5.0), (18.0, 5.0)), ((6.0, 5.0), (6.0, 8.0))],
    );
    let net = routed.tracks[0].net;
    routed.vias.push(RoutedVia {
        net,
        center: Point::new(6_000_000, 8_000_000),
        diameter: 600_000,
        drill: 300_000,
        layers: LayerMask::single(LayerId(0)).with(LayerId(1)),
        gen: crate::api::Generation(0),
    });
    let r = clear_findings(&board, &rules, &mut routed);
    assert_eq!((r.lone_vias, r.spurs), (1, 1), "{r:?}");
    assert_eq!(routed.tracks.len(), 1);
    assert!(routed.vias.is_empty());
}

#[test]
fn excluded_classes_are_left_open_and_never_failed() {
    let board = two_pin_nets(4, false);
    let rules = UniformRules::default();
    let s = PipelineSettings {
        excluded_classes: BTreeSet::from([ClassId(0)]),
        ..settings(1)
    };
    let r = route_board(&board, &rules, &PipelineScope::default(), &s).unwrap();
    assert!(r.connections.is_empty(), "nothing of the class is routed");
    assert_eq!(r.excluded.len(), 4);
    assert!(r
        .excluded
        .iter()
        .enumerate()
        .all(|(i, c)| c.id.index() == i));
    assert!(r.failed.is_empty(), "exclusion is not a failure");
    assert_eq!(r.connections_after, 4);
    assert_eq!(r.plan.item_count(), 0);
    assert_eq!(
        r.telemetry.counters.get("pipeline_excluded_by_policy"),
        Some(&4)
    );
}

#[test]
fn tracks_off_the_reference_go_back_or_go() {
    let mut b = BoardBuilder::new(20.0, 10.0, &["F.Cu", "In1.Cu", "B.Cu"]);
    b.net("GND").net("A").net("B");
    // Solid reference copper on In1 over the left half only.
    b.zone("GND", "In1.Cu", (0.5, 0.5, 10.0, 9.5));
    let board = b.build();
    let rules = UniformRules::default();
    let (bcu, fcu) = (
        board.layers.by_name("B.Cu").unwrap(),
        board.layers.by_name("F.Cu").unwrap(),
    );
    let net = |n: &str| board.net_by_name(n).unwrap();
    let track = |n: &str, layer, x0: f64, x1: f64| RoutedTrack {
        net: net(n),
        layer,
        seg: konnect_geom::Segment::new(
            Point::from_mm(x0, 5.0).unwrap(),
            Point::from_mm(x1, 5.0).unwrap(),
            150_000,
        ),
        gen: crate::api::Generation(0),
    };
    let fallback = BTreeMap::from([(
        ClassId(0),
        ClassFallback {
            nets: None,
            layers: board.layers.all(),
            reference: Some(ReferencePlane {
                plane: board.layers.by_name("In1.Cu").unwrap(),
                over: LayerMask::single(bcu),
                margin: 300_000,
            }),
        },
    )]);
    // A: over the plane before S6, moved off it after; B: off it in both.
    // F.Cu is not covered by the reference.
    let before = RoutedCopper {
        tracks: vec![
            track("A", bcu, 2.0, 6.0),
            track("B", bcu, 8.0, 14.0),
            track("B", fcu, 12.0, 18.0),
        ],
        vias: vec![],
    };
    let mut after = RoutedCopper {
        tracks: vec![
            track("A", bcu, 2.0, 12.0),
            track("B", bcu, 8.0, 14.0),
            track("B", fcu, 12.0, 18.0),
        ],
        vias: vec![],
    };
    let mut s = PipelineSettings {
        class_fallback: fallback,
        ..PipelineSettings::default()
    };
    s.layer_policy
        .class_layers
        .insert(ClassId(0), LayerMask::single(fcu));
    let r = enforce_reference(&board, &rules, &s, Some(&before), &mut after);
    assert_eq!(r.tracks_checked, 2);
    assert_eq!(r.reverted_nets, vec![net("A")]);
    assert_eq!(r.dropped_nets, vec![net("B")]);
    assert_eq!((r.dropped_tracks, r.dropped_vias), (1, 0));
    // B's F.Cu track is a piece of its own: only the offending piece goes.
    assert_eq!(
        after.tracks,
        vec![track("B", fcu, 12.0, 18.0), track("A", bcu, 2.0, 6.0)]
    );
}

#[test]
fn the_reference_holds_only_the_fallback_s_nets_on_its_added_layers() {
    let mut b = BoardBuilder::new(20.0, 10.0, &["F.Cu", "In1.Cu", "B.Cu"]);
    b.net("GND").net("A").net("B").net("C");
    b.zone("GND", "In1.Cu", (0.5, 0.5, 19.5, 9.5));
    let board = b.build();
    let rules = UniformRules::default();
    let layer = |n: &str| board.layers.by_name(n).unwrap();
    let (fcu, bcu) = (layer("F.Cu"), layer("B.Cu"));
    let net = |n: &str| board.net_by_name(n).unwrap();
    let track = |n: &str, layer, (x0, x1): (f64, f64), y: f64| RoutedTrack {
        net: net(n),
        layer,
        seg: Segment::new(
            Point::from_mm(x0, y).unwrap(),
            Point::from_mm(x1, y).unwrap(),
            150_000,
        ),
        gen: crate::api::Generation(0),
    };
    let mut s = PipelineSettings::default();
    s.class_fallback.insert(
        ClassId(0),
        ClassFallback {
            nets: Some(BTreeSet::from([net("A"), net("B")])),
            layers: board.layers.all(),
            reference: Some(ReferencePlane {
                plane: layer("In1.Cu"),
                over: board.layers.all(),
                margin: 100_000,
            }),
        },
    );
    // A: over the plane on B.Cu, off it on F.Cu (its own layer, never
    // held to the reference). B: over the plane's saved fill, but a routed
    // via of C beside it will cut a hole there. C: off the plane on B.Cu,
    // but outside the fallback's nets.
    let routed = RoutedCopper {
        tracks: vec![
            track("A", bcu, (2.0, 8.0), 5.0),
            track("A", fcu, (2.0, 8.0), 9.8),
            track("B", bcu, (12.0, 18.0), 5.0),
            track("C", bcu, (12.0, 18.0), 9.8),
        ],
        vias: vec![RoutedVia {
            net: net("C"),
            center: Point::from_mm(15.0, 5.6).unwrap(),
            diameter: 600_000,
            drill: 300_000,
            layers: board.layers.all(),
            gen: crate::api::Generation(0),
        }],
    };
    // A class with no layers of its own routes on every layer: nothing of
    // it is held to the reference.
    let mut copy = routed.clone();
    let r = enforce_reference(&board, &rules, &s, None, &mut copy);
    assert_eq!(r, ReferenceReport::default());
    assert_eq!(copy, routed);

    s.layer_policy
        .class_layers
        .insert(ClassId(0), LayerMask::single(fcu));
    let mut copy = routed.clone();
    let r = enforce_reference(&board, &rules, &s, None, &mut copy);
    assert_eq!(r.tracks_checked, 2, "A and B on B.Cu");
    assert_eq!(r.dropped_nets, vec![net("B")]);
    assert_eq!((r.dropped_tracks, r.dropped_vias), (1, 0));
    assert_eq!(copy.tracks.len(), 3);
    assert!(copy.tracks.iter().all(|t| t.net != net("B")));
    assert_eq!(copy.vias, routed.vias);

    // Without C's via, B's track is over the plane.
    let mut no_via = routed.clone();
    no_via.vias.clear();
    let r = enforce_reference(&board, &rules, &s, None, &mut no_via);
    assert!(r.dropped_nets.is_empty(), "{r:?}");
}

#[test]
fn connections_whose_copper_the_reference_took_are_failed() {
    let board = clear_board(false);
    let connections = ConnectivityReport::build(&board, &ArcPolicy::default()).connections(&board);
    assert_eq!(connections.len(), 1);
    let id = connections[0].id;
    let net = connections[0].net;
    let joined = routed_tracks(&board, &[((2.0, 5.0), (18.0, 5.0))]);
    let mut detail = DetailResult {
        routed: vec![id],
        ..DetailResult::default()
    };
    assert_eq!(
        fail_unjoined(&board, &connections, &[net], &joined, &mut detail),
        0
    );
    assert_eq!(detail.routed, vec![id]);
    let opened = fail_unjoined(
        &board,
        &connections,
        &[net],
        &RoutedCopper::default(),
        &mut detail,
    );
    assert_eq!(opened, 1);
    assert!(detail.routed.is_empty());
    assert_eq!(detail.failed, vec![(id, FailureReason::NoCorridor)]);
    assert_eq!(
        detail.failure_details,
        vec![(id, DetailFailure::OffReference)]
    );
}

#[test]
fn excluded_classes_get_no_fanout_copper() {
    let board = plane_with_pads();
    let rules = UniformRules::default();
    let s = PipelineSettings {
        excluded_classes: BTreeSet::from([ClassId(0)]),
        ..settings(1)
    };
    let r = route_board(&board, &rules, &PipelineScope::default(), &s).unwrap();
    let fanout = r.fanout.as_ref().expect("fanout ran");
    assert!(fanout.sites.is_empty() && fanout.failed.is_empty());
    assert!(r.fanout_copper.vias.is_empty() && r.fanout_copper.tracks.is_empty());
    assert!(
        r.telemetry.counters["pipeline_fanout_excluded_by_policy"] > 0,
        "the plan had sites to drop"
    );
    assert!(r.connections.is_empty() && r.failed.is_empty());
    assert_eq!(r.plan.item_count(), 0);
}

#[test]
fn ripping_up_a_connection_keeps_a_via_another_one_shares() {
    let board = clear_board(false);
    let mut routed = routed_tracks(
        &board,
        &[((2.0, 5.0), (6.0, 5.0)), ((6.0, 5.0), (18.0, 5.0))],
    );
    let net = routed.tracks[0].net;
    let via = |x: f64| RoutedVia {
        net,
        center: Point::from_mm(x, 5.0).unwrap(),
        diameter: 600_000,
        drill: 300_000,
        layers: LayerMask::single(LayerId(0)).with(LayerId(1)),
        gen: crate::api::Generation(0),
    };
    routed.vias.push(via(6.0));
    routed.vias.push(via(18.0));
    let (a, b) = (ConnectionId(0), ConnectionId(1));
    let owners = Owners {
        tracks: vec![a, b],
        vias: vec![vec![a, b], vec![b]],
    };
    let (copper, left) = owners.without(&routed, &BTreeSet::from([b]));
    assert_eq!(copper.tracks.len(), 1);
    assert_eq!(copper.tracks[0], routed.tracks[0]);
    // The shared via stays with its other user; b's own via goes.
    assert_eq!(copper.vias, vec![routed.vias[0]]);
    assert_eq!(left.tracks, vec![a]);
    assert_eq!(left.vias, vec![vec![a]]);
}

/// A two-layer board whose SIG pads sit either side of a GND pour that
/// spans the board on both layers: no way round it.
fn walled_by_pours() -> RouteBoard {
    let mut b = BoardBuilder::new(30.0, 12.0, &["F.Cu", "B.Cu"]);
    b.zone("GND", "F.Cu", (12.0, 0.1, 18.0, 11.9));
    b.zone("GND", "B.Cu", (12.0, 0.1, 18.0, 11.9));
    let l = b.footprint("R1", (5.0, 6.0));
    b.smd_pad(l, "1", "SIG", (5.0, 6.0), (0.8, 0.5), "F.Cu");
    let r = b.footprint("R2", (25.0, 6.0));
    b.smd_pad(r, "1", "SIG", (25.0, 6.0), (0.8, 0.5), "F.Cu");
    b.build()
}

#[test]
fn a_filled_pour_of_another_net_blocks_a_route_until_it_is_declared_crossable() {
    let mut board = walled_by_pours();
    let rules = UniformRules::default();
    let sig = board.net_by_name("SIG").unwrap();
    let blocked = route_board(&board, &rules, &PipelineScope::default(), &settings(2)).unwrap();
    assert_eq!(blocked.connections.len(), 1);
    assert_eq!(blocked.failed.len(), 1, "the pours wall the pads apart");
    assert!(blocked.routed.tracks.is_empty());
    assert!(crossed_zones(&board, &rules, &blocked.routed).is_empty());

    // One pour declared crossable is enough: the route goes through it.
    let front = board
        .zones
        .iter()
        .position(|z| z.layer == LayerId(0))
        .unwrap();
    board.zones[front].crossable = true;
    let r = route_board(&board, &rules, &PipelineScope::default(), &settings(2)).unwrap();
    assert!(r.failed.is_empty(), "failed: {:?}", r.failed);
    assert_eq!(r.connections_after, 0);
    assert!(r.routed.tracks.iter().all(|t| t.net == sig));
    assert!(
        r.routed.tracks.iter().all(|t| t.layer == LayerId(0)) && r.routed.vias.is_empty(),
        "the pour that was not declared still blocks its layer: {:?}",
        r.routed
    );
    // The optimizer kept the crossing (its checker leaves the pour alone
    // too), and S7 leaves every fill to the refill.
    assert!(r.check.violations.is_empty(), "{:?}", r.check.violations);
    let crossed = crossed_zones(&board, &rules, &r.routed);
    assert_eq!(crossed.len(), 1, "{crossed:?}");
    assert_eq!(crossed[0].zone, board.zones[front].id);
    assert!(crossed[0].tracks > 0 && crossed[0].vias == 0);
}

#[test]
fn copper_of_the_pours_own_net_or_clear_of_it_is_not_a_crossing() {
    let mut board = walled_by_pours();
    for z in &mut board.zones {
        z.crossable = true;
    }
    let rules = UniformRules::default();
    let (gnd, sig) = (
        board.net_by_name("GND").unwrap(),
        board.net_by_name("SIG").unwrap(),
    );
    let mm = |v: f64| konnect_geom::mm_to_nm(v).unwrap();
    let track = |net, x1: f64, x2: f64| RoutedTrack {
        net,
        layer: LayerId(0),
        seg: Segment::new(
            Point::new(mm(x1), mm(6.0)),
            Point::new(mm(x2), mm(6.0)),
            mm(0.15),
        ),
        gen: crate::api::Generation(0),
    };
    let copper = |tracks| RoutedCopper {
        tracks,
        vias: vec![],
    };
    // GND's own track through its pour, and a SIG track 1 mm short of it.
    let clear = copper(vec![track(gnd, 10.0, 20.0), track(sig, 5.0, 11.0)]);
    assert!(crossed_zones(&board, &rules, &clear).is_empty());
    // 0.1 mm from the pour's edge is inside the 0.2 mm clearance.
    let near = copper(vec![track(sig, 5.0, 11.825)]);
    let crossed = crossed_zones(&board, &rules, &near);
    assert_eq!(crossed.len(), 1);
    assert_eq!((crossed[0].tracks, crossed[0].vias), (1, 0));
    // A pour that was not declared is never reported.
    for z in &mut board.zones {
        z.crossable = false;
    }
    assert!(crossed_zones(&board, &rules, &near).is_empty());
}

#[test]
fn plane_nets_are_left_to_their_planes_unless_named_for_tracks() {
    let board = plane_with_pads();
    let rules = UniformRules::default();
    let gnd = board.net_by_name("GND").unwrap();
    let pad = |reference: &str, number: &str| {
        board
            .pads
            .iter()
            .find(|p| {
                p.number == number
                    && board.footprints[p.footprint.index()].reference.as_deref() == Some(reference)
            })
            .unwrap()
            .id
    };
    // No fanout: the SMD pads stay open, and say why.
    let left = PipelineSettings {
        fanout: None,
        plane_tracks: PlaneTracks::Only(BTreeSet::new()),
        ..settings(2)
    };
    let r = route_board(&board, &rules, &PipelineScope::default(), &left).unwrap();
    assert!(r.connections.iter().all(|c| c.net != gnd));
    assert_eq!(r.left_to_planes.len(), 2);
    assert!(r
        .left_to_planes
        .iter()
        .enumerate()
        .all(|(i, c)| c.net == gnd && c.id.index() == i));
    assert!(r.failed.is_empty(), "left to the plane is not a failure");
    assert!(r.routed.tracks.iter().all(|t| t.net != gnd));
    assert!(!r.routed.tracks.is_empty(), "SIG is still routed");
    assert_eq!(
        r.telemetry.counters.get("pipeline_left_to_planes"),
        Some(&2)
    );
    assert_eq!(r.plane_nets.len(), 1);
    let p = &r.plane_nets[0];
    assert_eq!((p.net, p.filled, p.routed_as_tracks), (gnd, true, false));
    assert_eq!(p.pads, 4);
    assert_eq!(p.joined, vec![pad("J1", "1"), pad("J1", "2")]);
    assert!(p.fanout.is_empty() && p.unfilled.is_empty());
    assert_eq!(
        p.open,
        vec![
            (pad("C1", "1"), PlaneOpenReason::FanoutOff),
            (pad("R1", "1"), PlaneOpenReason::FanoutOff)
        ]
    );
    assert_eq!(p.connections_left, 2);

    // With the fanout, its vias join them and nothing is left.
    let fanned = PipelineSettings {
        plane_tracks: PlaneTracks::Only(BTreeSet::new()),
        ..settings(2)
    };
    let r = route_board(&board, &rules, &PipelineScope::default(), &fanned).unwrap();
    let p = &r.plane_nets[0];
    assert_eq!(p.fanout, vec![pad("C1", "1"), pad("R1", "1")]);
    assert!(p.open.is_empty() && r.left_to_planes.is_empty());
    assert_eq!(p.connections_left, 0);

    // Named, the net is routed as tracks like any other.
    let named = PipelineSettings {
        fanout: None,
        plane_tracks: PlaneTracks::Only(BTreeSet::from([gnd])),
        ..settings(2)
    };
    let r = route_board(&board, &rules, &PipelineScope::default(), &named).unwrap();
    assert!(r.left_to_planes.is_empty());
    assert_eq!(r.connections.iter().filter(|c| c.net == gnd).count(), 2);
    assert!(r.plane_nets[0].routed_as_tracks);
    assert_eq!(r.plane_nets[0].connections_left, 0);
}

#[test]
fn a_plane_net_without_a_fill_is_reported_and_not_routed_as_tracks() {
    let mut board = plane_with_pads();
    for z in &mut board.zones {
        z.filled = false;
    }
    let rules = UniformRules::default();
    let gnd = board.net_by_name("GND").unwrap();
    let left = PipelineSettings {
        plane_tracks: PlaneTracks::Only(BTreeSet::new()),
        ..settings(2)
    };
    let r = route_board(&board, &rules, &PipelineScope::default(), &left).unwrap();
    // No fill, so no plane to fan out to, and no tracks in its place.
    assert!(r.fanout_copper.vias.is_empty());
    assert!(r.routed.tracks.iter().all(|t| t.net != gnd));
    assert!(r.routed.vias.iter().all(|v| v.net != gnd));
    assert_eq!(r.left_to_planes.len(), 3, "four pads, none joined");
    assert!(r.failed.is_empty());
    let p = &r.plane_nets[0];
    assert_eq!((p.net, p.filled), (gnd, false));
    assert_eq!(p.unfilled, vec![board.zones[0].id]);
    assert!(p.joined.is_empty() && p.fanout.is_empty() && p.open.is_empty());
    assert_eq!((p.pads, p.connections_left), (4, 3));
    // The engine's own default still routes such a net when asked to.
    let all = route_board(&board, &rules, &PipelineScope::default(), &settings(2)).unwrap();
    assert!(all.left_to_planes.is_empty());
    assert_eq!(all.connections.iter().filter(|c| c.net == gnd).count(), 3);
}
