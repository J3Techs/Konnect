//! Synthetic end-to-end runs of the pipeline. Every board is generated
//! here; nothing is real.

use super::*;
use crate::check::ViolationKind;
use crate::synthetic::{plane_with_pads, two_pin_nets, BoardBuilder, UniformRules};
use konnect_geom::{Point, Segment};

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
