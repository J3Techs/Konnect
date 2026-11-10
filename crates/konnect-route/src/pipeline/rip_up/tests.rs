//! Synthetic single-layer boards for the rip-up stage: with one copper
//! layer a fence cannot be hopped over with a via, so what is sealed stays
//! sealed until the fence goes.

use super::*;
use crate::pipeline::{route_board, PipelineResult};
use crate::synthetic::{BoardBuilder, UniformRules};

fn settings(mode: RipUp) -> PipelineSettings {
    PipelineSettings {
        threads: 2,
        rip_up: RipUpSettings {
            mode,
            ..RipUpSettings::default()
        },
        ..PipelineSettings::default()
    }
}

fn run(board: &RouteBoard, settings: &PipelineSettings) -> PipelineResult {
    route_board(
        board,
        &UniformRules::default(),
        &PipelineScope::default(),
        settings,
    )
    .unwrap()
}

/// Give every track and via a UUID, as a loaded board has.
fn with_uuids(mut board: RouteBoard) -> RouteBoard {
    for (i, t) in board.tracks.iter_mut().enumerate() {
        t.uuid = Some(format!("track-{i}"));
    }
    for (i, v) in board.vias.iter_mut().enumerate() {
        v.uuid = Some(format!("via-{i}"));
    }
    board
}

/// A ring of net `fence` around the pad at `at`, half-size 2 mm, joining
/// the fence net's two pads on opposite corners.
fn ring(b: &mut BoardBuilder, footprint: usize, fence: &str, at: (f64, f64), locked: bool) {
    let (x0, y0, x1, y1) = (at.0 - 2.0, at.1 - 2.0, at.0 + 2.0, at.1 + 2.0);
    b.smd_pad(
        footprint,
        &format!("{fence}1"),
        fence,
        (x0, y0),
        (0.6, 0.6),
        "F.Cu",
    );
    b.smd_pad(
        footprint,
        &format!("{fence}2"),
        fence,
        (x1, y1),
        (0.6, 0.6),
        "F.Cu",
    );
    for (from, to) in [
        ((x0, y0), (x1, y0)),
        ((x1, y0), (x1, y1)),
        ((x1, y1), (x0, y1)),
        ((x0, y1), (x0, y0)),
    ] {
        b.track(fence, "F.Cu", from, to, 0.15, locked);
    }
}

/// Net `A` from a pad sealed inside a ring of net `B` to a pad outside it.
fn sealed_pin(locked: bool) -> RouteBoard {
    let mut b = BoardBuilder::new(20.0, 12.0, &["F.Cu"]);
    let f = b.footprint("U1", (10.0, 6.0));
    b.smd_pad(f, "1", "A", (5.0, 6.0), (0.6, 0.6), "F.Cu");
    b.smd_pad(f, "2", "A", (15.0, 6.0), (0.6, 0.6), "F.Cu");
    ring(&mut b, f, "B", (5.0, 6.0), locked);
    with_uuids(b.build())
}

/// A pocket walled in by locked tracks whose only way out is a gap one
/// track wide. Nets `A` and `B` each have a pad inside and one outside;
/// `B`'s unlocked track holds the gap. With `ringed`, net `D` is also
/// sealed inside a ring of net `E` further right.
fn pocket(ringed: bool) -> RouteBoard {
    let mut b = BoardBuilder::new(30.0, 10.0, &["F.Cu"]);
    let f = b.footprint("U1", (15.0, 5.0));
    for (from, to) in [
        ((2.0, 2.0), (8.0, 2.0)),
        ((2.0, 8.0), (8.0, 8.0)),
        ((2.0, 2.0), (2.0, 8.0)),
        ((8.0, 2.0), (8.0, 4.6)),
        ((8.0, 5.4), (8.0, 8.0)),
    ] {
        b.track("WALL", "F.Cu", from, to, 0.2, true);
    }
    b.smd_pad(f, "1", "A", (4.0, 3.5), (0.6, 0.6), "F.Cu");
    b.smd_pad(f, "2", "A", (14.0, 3.5), (0.6, 0.6), "F.Cu");
    b.smd_pad(f, "3", "B", (4.0, 6.5), (0.6, 0.6), "F.Cu");
    b.smd_pad(f, "4", "B", (14.0, 6.5), (0.6, 0.6), "F.Cu");
    for (from, to) in [
        ((4.0, 6.5), (5.5, 5.0)),
        ((5.5, 5.0), (12.5, 5.0)),
        ((12.5, 5.0), (14.0, 6.5)),
    ] {
        b.track("B", "F.Cu", from, to, 0.15, false);
    }
    if ringed {
        b.smd_pad(f, "5", "D", (22.0, 5.0), (0.6, 0.6), "F.Cu");
        b.smd_pad(f, "6", "D", (27.0, 5.0), (0.6, 0.6), "F.Cu");
        ring(&mut b, f, "E", (22.0, 5.0), false);
    }
    with_uuids(b.build())
}

fn open_nets(r: &PipelineResult, board: &RouteBoard) -> Vec<String> {
    let mut nets: Vec<String> = r
        .detail
        .failure_details
        .iter()
        .map(|(id, _)| board.net(r.connections[id.index()].net).name.clone())
        .collect();
    nets.sort();
    nets
}

#[test]
fn a_sealed_pin_is_opened_by_ripping_the_fencing_net_and_both_nets_end_joined() {
    let board = sealed_pin(false);
    let r = run(&board, &settings(RipUp::Unlocked));
    assert_eq!(r.rip_up.open_before, 1, "{:?}", r.rip_up);
    assert_eq!(r.rip_up.open_after, 0, "{:?}", r.rip_up);
    assert_eq!(r.connections_after, 0, "both nets are joined");
    assert!(r.failed.is_empty(), "{:?}", r.failed);
    assert!(r.check.violations.is_empty(), "{:?}", r.check.violations);
    // Exactly the ring's four tracks are deleted, each named with its net,
    // layer and the net it blocked.
    let mut uuids: Vec<&str> = r.rip_up.ripped.iter().map(|i| i.uuid.as_str()).collect();
    uuids.sort();
    assert_eq!(uuids, ["track-0", "track-1", "track-2", "track-3"]);
    for item in &r.rip_up.ripped {
        assert_eq!(
            (
                item.kind,
                item.net.as_str(),
                item.layer.as_str(),
                item.round
            ),
            (ItemKind::Track, "B", "F.Cu", 1)
        );
        assert!(item.reason.contains("blocked A"), "{}", item.reason);
    }
    assert_eq!(r.rip_up.rerouted_nets, ["B"]);
    assert_eq!(r.rip_up.rounds, 1);
    assert_eq!(r.rip_up.stopped, "complete");
    assert!(r.rip_up.still_open.is_empty());
    // The plan routes both nets; the task list is the one of the board
    // without the ring.
    for net in ["A", "B"] {
        assert!(
            r.plan.tracks.iter().any(|t| t.net == net),
            "no {net} copper"
        );
    }
    assert_eq!(r.connections.len(), 2);
    assert_eq!(r.detail.routed.len(), 2);
}

#[test]
fn board_copper_stays_when_only_the_runs_own_copper_may_be_ripped() {
    let board = sealed_pin(false);
    let r = run(&board, &settings(RipUp::Own));
    let plain = run(&board, &settings(RipUp::None));
    assert!(r.rip_up.ripped.is_empty() && !r.rip_up.changed());
    assert_eq!(r.connections_after, 1);
    assert_eq!(r.routed, plain.routed);
    assert_eq!(r.rip_up.stopped, "nothing_to_rip");
    let open = &r.rip_up.still_open;
    assert_eq!(open.len(), 1, "{open:?}");
    assert_eq!((open[0].net.as_str(), open[0].from.as_str()), ("A", "U1.1"));
    assert_eq!(open[0].blocked_by.len(), 1, "{open:?}");
    assert_eq!(open[0].blocked_by[0].net, "B");
    assert_eq!(open[0].blocked_by[0].reason, "board_copper_not_requested");
    assert!(!open[0].blocked_by[0].uuids.is_empty());
}

#[test]
fn locked_copper_is_never_ripped_and_the_connection_reports_it_as_the_blocker() {
    let board = sealed_pin(true);
    let r = run(&board, &settings(RipUp::Unlocked));
    let plain = run(&board, &settings(RipUp::None));
    assert!(r.rip_up.ripped.is_empty(), "{:?}", r.rip_up.ripped);
    assert!(r.rip_up.attempts.is_empty(), "{:?}", r.rip_up.attempts);
    assert_eq!(r.connections_after, 1);
    assert_eq!(r.routed, plain.routed);
    assert_eq!(open_nets(&r, &board), ["A"]);
    let open = &r.rip_up.still_open;
    assert_eq!(open.len(), 1, "{open:?}");
    assert_eq!(open[0].blocked_by.len(), 1, "{open:?}");
    assert_eq!(open[0].blocked_by[0].net, "B");
    assert_eq!(open[0].blocked_by[0].reason, "locked");
    assert_eq!(r.rip_up.stopped, "nothing_to_rip");
}

#[test]
fn a_connection_walled_in_by_a_zone_fill_names_the_fill_and_nothing_is_ripped() {
    // A filled strip of net Z from edge to edge between A's two pads.
    let mut b = BoardBuilder::new(20.0, 12.0, &["F.Cu"]);
    let f = b.footprint("U1", (10.0, 6.0));
    b.smd_pad(f, "1", "A", (5.0, 6.0), (0.6, 0.6), "F.Cu");
    b.smd_pad(f, "2", "A", (15.0, 6.0), (0.6, 0.6), "F.Cu");
    b.zone("Z", "F.Cu", (9.0, 0.5, 11.0, 11.5));
    let board = b.build();
    let r = run(&board, &settings(RipUp::Unlocked));
    assert!(!r.rip_up.changed() && r.rip_up.ripped.is_empty());
    assert_eq!(r.connections_after, 1);
    let open = &r.rip_up.still_open;
    assert_eq!(open.len(), 1, "{open:?}");
    let by: Vec<(&str, &str)> = open[0]
        .blocked_by
        .iter()
        .map(|b| (b.net.as_str(), b.reason.as_str()))
        .collect();
    assert_eq!(by, [("Z", "zone_fill")]);
}

#[test]
fn an_attempt_that_joins_no_more_connections_leaves_the_starting_state_exactly() {
    // Ripping B lets A through the gap, but B then has no way out: one
    // open connection before, one after.
    let board = pocket(false);
    let r = run(&board, &settings(RipUp::Unlocked));
    let plain = run(&board, &settings(RipUp::None));
    // Stuck first, then both negotiated as one batch: neither pays.
    let orders: Vec<&str> = r.rip_up.attempts.iter().map(|a| a.order.as_str()).collect();
    assert_eq!(
        orders,
        ["stuck_first", "together"],
        "{:?}",
        r.rip_up.attempts
    );
    assert!(r.rip_up.attempts.iter().all(|a| !a.accepted));
    assert_eq!(r.rip_up.attempts[0].open_nets_after, ["B"]);
    let a = &r.rip_up.attempts[0];
    assert_eq!(
        (a.stuck_nets.as_slice(), a.ripped_nets.as_slice()),
        (&["A".to_string()][..], &["B".to_string()][..])
    );
    assert_eq!((a.open_before, a.open_after, a.accepted), (1, 1, false));
    assert_eq!(r.rip_up.stopped, "no_improvement");
    assert!(r.rip_up.ripped.is_empty() && r.rip_up.rerouted_nets.is_empty());
    assert_eq!((r.rip_up.open_before, r.rip_up.open_after), (1, 1));
    // Nothing of the attempt is left anywhere.
    assert_eq!(r.routed, plain.routed);
    assert_eq!(r.plan, plain.plan);
    assert_eq!(r.connections, plain.connections);
    assert_eq!(r.detail, plain.detail);
    assert_eq!(r.check, plain.check);
    assert_eq!(r.connections_after, plain.connections_after);
    assert_eq!(open_nets(&r, &board), ["A"]);
    assert_eq!(
        r.rip_up.still_open[0].blocked_by[0].reason,
        "ripping_did_not_help"
    );
}

#[test]
fn a_rip_set_already_tried_for_the_same_connection_is_not_tried_again() {
    // One net per attempt, so A (hopeless: it and B want the same gap) and
    // D (sealed by E's ring) are tried alone. A's attempt is dropped, D's
    // is kept; the next round would rip B for A again and does not.
    let board = pocket(true);
    let mut s = settings(RipUp::Unlocked);
    s.rip_up.max_nets = 1;
    s.rip_up.single_attempts = 8;
    let r = run(&board, &s);
    let seen: Vec<(&str, &str, bool, bool)> = r
        .rip_up
        .attempts
        .iter()
        .map(|a| {
            (
                a.stuck_nets[0].as_str(),
                a.ripped_nets[0].as_str(),
                a.accepted,
                a.repeat,
            )
        })
        .collect();
    assert_eq!(
        seen,
        [
            ("A", "B", false, false),
            ("A", "B", false, false),
            ("D", "E", true, false),
            ("A", "B", false, true),
        ]
    );
    assert_eq!(r.rip_up.stopped, "repeat");
    assert_eq!(r.rip_up.rounds, 2);
    assert_eq!((r.rip_up.open_before, r.rip_up.open_after), (2, 1));
    assert_eq!(r.connections_after, 1);
    assert_eq!(open_nets(&r, &board), ["A"]);
    // Only E's ring was deleted; B's track holds the gap as before.
    assert!(
        r.rip_up.ripped.iter().all(|i| i.net == "E"),
        "{:?}",
        r.rip_up.ripped
    );
    assert_eq!(r.rip_up.ripped.len(), 4);
    assert!(r.check.violations.is_empty(), "{:?}", r.check.violations);
}

#[test]
fn ripping_together_keeps_what_pays_and_the_reverse_rip_is_dropped() {
    // Both stuck connections in one attempt: A takes the gap from B, D
    // leaves its ring and E is routed again. Two open before, one after.
    // The next round rips A's new copper for B, gains nothing and is
    // dropped: the two nets do not keep evicting each other.
    let board = pocket(true);
    let r = run(&board, &settings(RipUp::Unlocked));
    let seen: Vec<(Vec<String>, Vec<String>, bool)> = r
        .rip_up
        .attempts
        .iter()
        .map(|a| (a.stuck_nets.clone(), a.ripped_nets.clone(), a.accepted))
        .collect();
    let names = |n: &[&str]| n.iter().map(|s| s.to_string()).collect::<Vec<_>>();
    assert_eq!(
        seen,
        [
            (names(&["A", "D"]), names(&["B", "E"]), true),
            (names(&["B"]), names(&["A"]), false),
            (names(&["B"]), names(&["A"]), false),
        ]
    );
    assert_eq!(r.rip_up.stopped, "no_improvement");
    assert_eq!((r.rip_up.open_before, r.rip_up.open_after), (2, 1));
    assert_eq!(r.connections_after, 1);
    assert_eq!(open_nets(&r, &board), ["B"]);
    assert_eq!(r.rip_up.ripped.len(), 7, "B's three tracks and E's four");
    assert!(r.check.violations.is_empty(), "{:?}", r.check.violations);
}

#[test]
fn the_board_without_the_ripped_copper_keeps_ids_equal_to_indices() {
    let board = sealed_pin(false);
    let ripped = vec![RippedItem {
        uuid: "track-1".to_string(),
        kind: ItemKind::Track,
        net: "B".to_string(),
        layer: "F.Cu".to_string(),
        round: 1,
        reason: String::new(),
    }];
    let b = without_ripped(&board, &ripped);
    assert_eq!(b.tracks.len(), board.tracks.len() - 1);
    assert!(b
        .tracks
        .iter()
        .all(|t| t.uuid.as_deref() != Some("track-1")));
    assert_eq!(b.validate(), Ok(()));
}

/// The sealed pin of [`sealed_pin`], and net `zone_net` with a zone that
/// has no fill. With `pads`, net P has three pads under its zone: two
/// connections a plane would join.
fn sealed_pin_beside_a_plane(zone_net: &str, pads: bool) -> RouteBoard {
    let mut b = BoardBuilder::new(20.0, 12.0, &["F.Cu"]);
    let f = b.footprint("U1", (10.0, 6.0));
    b.smd_pad(f, "1", "A", (5.0, 6.0), (0.6, 0.6), "F.Cu");
    b.smd_pad(f, "2", "A", (15.0, 6.0), (0.6, 0.6), "F.Cu");
    ring(&mut b, f, "B", (5.0, 6.0), false);
    if pads {
        for (number, x) in [("7", 11.0), ("8", 14.0), ("9", 17.0)] {
            b.smd_pad(f, number, "P", (x, 10.5), (0.6, 0.6), "F.Cu");
        }
    }
    b.zone(zone_net, "F.Cu", (10.0, 9.8, 18.0, 11.2));
    let mut board = with_uuids(b.build());
    for z in &mut board.zones {
        z.filled = false;
    }
    board
}

#[test]
fn connections_left_to_a_plane_are_neither_stuck_nor_counted_as_open() {
    let board = sealed_pin_beside_a_plane("P", true);
    let s = PipelineSettings {
        plane_tracks: crate::pipeline::PlaneTracks::Only(BTreeSet::new()),
        ..settings(RipUp::Unlocked)
    };
    let r = run(&board, &s);
    let p = board.net_by_name("P").unwrap();
    // P is left to its plane: two connections nobody routes.
    assert_eq!(r.left_to_planes.len(), 2);
    assert!(r.left_to_planes.iter().all(|c| c.net == p));
    assert!(r.connections.iter().all(|c| c.net != p));
    assert!(r.routed.tracks.iter().all(|t| t.net != p));
    // The stage sees one open connection, A's, and joins it by ripping B.
    assert_eq!((r.rip_up.open_before, r.rip_up.open_after), (1, 0));
    assert!(!r.rip_up.attempts.is_empty());
    for a in &r.rip_up.attempts {
        assert_eq!(a.stuck_nets, ["A"], "{a:?}");
        assert_eq!(a.ripped_nets, ["B"], "{a:?}");
        assert!(!a.open_nets_after.contains(&"P".to_string()), "{a:?}");
    }
    assert!(r.rip_up.still_open.is_empty(), "{:?}", r.rip_up.still_open);
    assert!(r.rip_up.ripped.iter().all(|i| i.net == "B"));
    assert!(r.failed.is_empty(), "{:?}", r.failed);
    // P's two connections are still open on the board, and that is not
    // "more open than before the stage": the kept attempt stays.
    assert_eq!(r.connections_after, 2);
    assert!(r.rip_up.changed() && !r.rip_up.restored);
}

#[test]
fn board_copper_of_a_net_with_a_zone_is_never_ripped() {
    // The fence is net B's, and B owns a zone: its tracks tie pads to a
    // pour the router does not redraw.
    let board = sealed_pin_beside_a_plane("B", false);
    let r = run(&board, &settings(RipUp::Unlocked));
    assert!(!r.rip_up.changed() && r.rip_up.ripped.is_empty());
    let open = &r.rip_up.still_open;
    assert_eq!(open.len(), 1, "{open:?}");
    assert_eq!(open[0].net, "A");
    let by: Vec<(&str, &str)> = open[0]
        .blocked_by
        .iter()
        .map(|b| (b.net.as_str(), b.reason.as_str()))
        .collect();
    assert_eq!(by, [("B", "plane_net")]);
}

#[test]
fn a_net_ripped_and_routed_again_keeps_the_width_its_connection_was_given() {
    let board = sealed_pin(false);
    let (a, b) = (
        board.net_by_name("A").unwrap(),
        board.net_by_name("B").unwrap(),
    );
    let pad = |number: &str| board.pads.iter().find(|p| p.number == number).unwrap().id;
    // B is joined on the board, so the entry governs no task at first; it
    // must find B's connection once the ring is gone.
    let s = PipelineSettings {
        connection_widths: vec![crate::pipeline::ConnectionWidth {
            net: b,
            from: vec![pad("B1")],
            to: vec![pad("B2")],
            width: 300_000,
        }],
        ..settings(RipUp::Unlocked)
    };
    let r = run(&board, &s);
    assert_eq!(r.rip_up.rerouted_nets, ["B"]);
    assert_eq!(r.connections_after, 0, "{:?}", r.rip_up);
    assert!(r.check.violations.is_empty(), "{:?}", r.check.violations);
    let widths = |net: NetId| -> BTreeSet<Coord> {
        r.routed
            .tracks
            .iter()
            .filter(|t| t.net == net)
            .map(|t| t.seg.width)
            .collect()
    };
    assert_eq!(widths(b), BTreeSet::from([300_000]));
    // A has no entry: the rule's width.
    assert_eq!(widths(a), BTreeSet::from([150_000]));
    // The task list of the board without the ring holds B's connection
    // under the entry.
    let task = r.connections.iter().find(|c| c.net == b).unwrap();
    assert_eq!(r.connection_widths.get(&task.id), Some(&0));
}
