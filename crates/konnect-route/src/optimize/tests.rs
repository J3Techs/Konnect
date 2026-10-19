use super::exact::{CheckerOptions, ExactChecker};
use super::*;
use crate::api::Generation;
use crate::connectivity::ConnectivityReport;
use crate::synthetic::{BoardBuilder, UniformRules};

const MM: Coord = 1_000_000;
const W: Coord = 150_000;

fn p(x: f64, y: f64) -> Point {
    Point::from_mm(x, y).unwrap()
}

fn track(net: NetId, layer: u8, a: Point, b: Point) -> RoutedTrack {
    RoutedTrack {
        net,
        layer: LayerId(layer),
        seg: Segment::new(a, b, W),
        gen: Generation(1),
    }
}

/// A 50 µm lattice staircase from `a` to `b` (x first, then y, alternating).
fn staircase(net: NetId, layer: u8, a: Point, b: Point, step: Coord) -> Vec<RoutedTrack> {
    let mut out = Vec::new();
    let mut cur = a;
    while cur != b {
        let mut next = cur;
        if cur.x != b.x {
            next.x += step.min((b.x - cur.x).abs()) * (b.x - cur.x).signum();
            out.push(track(net, layer, cur, next));
            cur = next;
        }
        if cur.y != b.y {
            let mut n2 = cur;
            n2.y += step.min((b.y - cur.y).abs()) * (b.y - cur.y).signum();
            out.push(track(net, layer, cur, n2));
            cur = n2;
        }
    }
    out
}

fn board_two_pads() -> RouteBoard {
    let mut b = BoardBuilder::new(30.0, 20.0, &["F.Cu", "B.Cu"]);
    let u = b.footprint("U1", (5.0, 5.0));
    b.smd_pad(u, "1", "SIG", (5.0, 5.0), (0.6, 0.6), "F.Cu");
    let v = b.footprint("U2", (20.0, 12.0));
    b.smd_pad(v, "1", "SIG", (20.0, 12.0), (0.6, 0.6), "F.Cu");
    b.build()
}

fn islands(board: &RouteBoard, copper: &RoutedCopper) -> usize {
    // Routed copper joins the board as tracks and vias for the union-find.
    let mut b = board.clone();
    for t in &copper.tracks {
        b.tracks.push(crate::board::Track {
            id: crate::api::TrackId(b.tracks.len() as u32),
            net: Some(t.net),
            layer: t.layer,
            seg: t.seg,
            ownership: crate::api::Ownership::Fixed,
            uuid: None,
        });
    }
    for v in &copper.vias {
        b.vias.push(crate::board::Via {
            id: crate::api::ViaId(b.vias.len() as u32),
            net: Some(v.net),
            center: v.center,
            diameter: v.diameter,
            drill: v.drill,
            layers: v.layers,
            ownership: crate::api::Ownership::Fixed,
            uuid: None,
        });
    }
    ConnectivityReport::build(&b, &konnect_geom::ArcPolicy::default()).connection_count()
}

fn clean(board: &RouteBoard, rules: &UniformRules, copper: &RoutedCopper) -> bool {
    let ck = ExactChecker::new(board, rules, CheckerOptions::default());
    let r = ck.check_copper(copper);
    if !r.violations.is_empty() {
        eprintln!("{:?}", r.violations);
    }
    r.violations.is_empty()
}

#[test]
fn a_lattice_staircase_pulls_tight_into_two_octilinear_segments() {
    let board = board_two_pads();
    let rules = UniformRules::default();
    let net = board.net_by_name("SIG").unwrap();
    let mut copper = RoutedCopper {
        tracks: staircase(net, 0, p(5.0, 5.0), p(20.0, 12.0), 50_000),
        vias: vec![],
    };
    let before = copper.tracks.len();
    assert!(before > 200);
    assert_eq!(islands(&board, &copper), 0);
    let report = optimize(&board, &rules, &mut copper);
    assert!(
        copper.tracks.len() <= 3,
        "{} tracks: {report:?}",
        copper.tracks.len()
    );
    assert!(report.length_after_nm < report.length_before_nm);
    assert_eq!(report.defects_after, 0, "{report:?}");
    assert_eq!(islands(&board, &copper), 0, "still connected");
    assert!(clean(&board, &rules, &copper));
    for t in &copper.tracks {
        assert!(is_octilinear(t.seg.b - t.seg.a), "{t:?}");
    }
}

#[test]
fn shortcuts_never_cut_through_another_nets_pad() {
    let mut b = BoardBuilder::new(30.0, 20.0, &["F.Cu", "B.Cu"]);
    let u = b.footprint("U1", (5.0, 5.0));
    b.smd_pad(u, "1", "SIG", (5.0, 5.0), (0.6, 0.6), "F.Cu");
    b.smd_pad(u, "2", "SIG", (15.0, 5.0), (0.6, 0.6), "F.Cu");
    // A blocker of another net right on the straight line.
    let k = b.footprint("K1", (10.0, 5.0));
    b.smd_pad(k, "1", "OTHER", (10.0, 5.0), (2.0, 2.0), "F.Cu");
    let board = b.build();
    let rules = UniformRules::default();
    let net = board.net_by_name("SIG").unwrap();
    // A detour 3 mm north, built as a staircase out and back.
    let mut tracks = staircase(net, 0, p(5.0, 5.0), p(7.0, 2.0), 50_000);
    tracks.push(track(net, 0, p(7.0, 2.0), p(13.0, 2.0)));
    tracks.extend(staircase(net, 0, p(13.0, 2.0), p(15.0, 5.0), 50_000));
    let mut copper = RoutedCopper {
        tracks,
        vias: vec![],
    };
    assert!(clean(&board, &rules, &copper));
    let report = optimize(&board, &rules, &mut copper);
    assert!(report.pull_tight_edits > 0);
    assert!(
        report.edits_rejected > 0,
        "the straight line was tried and refused"
    );
    assert!(clean(&board, &rules, &copper));
    assert_eq!(islands(&board, &copper), 0);
    assert_eq!(report.defects_after, 0, "{report:?}");
    assert!(copper.tracks.len() < 20);
}

#[test]
fn collinear_pieces_merge_and_t_junctions_split() {
    let board = board_two_pads();
    let rules = UniformRules::default();
    let net = board.net_by_name("SIG").unwrap();
    let mut tracks: Vec<RoutedTrack> = (0..10)
        .map(|i| {
            track(
                net,
                1,
                Point::new(5 * MM + i * MM, 15 * MM),
                Point::new(6 * MM + i * MM, 15 * MM),
            )
        })
        .collect();
    // A branch ending on the middle of one piece (a T to split).
    tracks.push(track(
        net,
        1,
        Point::new(7_500_000, 15 * MM),
        Point::new(7_500_000, 17 * MM),
    ));
    let mut copper = RoutedCopper {
        tracks,
        vias: vec![],
    };
    let opts = OptimizeOptions {
        pull_tight: false,
        corner_cuts: false,
        ..OptimizeOptions::default()
    };
    let report = optimize_with(
        &board,
        &rules,
        &mut copper,
        &opts,
        &mut Telemetry::default(),
    );
    assert_eq!(report.junctions_split, 1);
    // Two runs either side of the T, plus the branch.
    assert_eq!(copper.tracks.len(), 3, "{:?}", copper.tracks);
    assert_eq!(report.defects_after, 0);
}

#[test]
fn right_angles_get_a_45_degree_chamfer() {
    let mut b = BoardBuilder::new(30.0, 30.0, &["F.Cu", "B.Cu"]);
    let u = b.footprint("U1", (5.0, 5.0));
    b.smd_pad(u, "1", "SIG", (5.0, 5.0), (0.6, 0.6), "F.Cu");
    b.smd_pad(u, "2", "SIG", (15.0, 15.0), (0.6, 0.6), "F.Cu");
    // Block the diagonal so the L cannot simply be pulled tight.
    let k = b.footprint("K1", (10.0, 10.0));
    b.smd_pad(k, "1", "OTHER", (11.0, 9.0), (6.0, 6.0), "F.Cu");
    let board = b.build();
    let rules = UniformRules::default();
    let net = board.net_by_name("SIG").unwrap();
    let mut copper = RoutedCopper {
        tracks: vec![
            track(net, 0, p(5.0, 5.0), p(5.0, 15.0)),
            track(net, 0, p(5.0, 15.0), p(15.0, 15.0)),
        ],
        vias: vec![],
    };
    assert!(clean(&board, &rules, &copper));
    let report = optimize(&board, &rules, &mut copper);
    assert_eq!(
        (report.corners_cut, report.pull_tight_edits),
        (1, 0),
        "{report:?}"
    );
    assert_eq!(copper.tracks.len(), 3);
    // The middle piece is the 45 degree chamfer, at least a track width.
    let chamfer = copper.tracks[1..]
        .iter()
        .chain(&copper.tracks[..1])
        .find(|t| {
            let d = t.seg.b - t.seg.a;
            d.x != 0 && d.y != 0
        })
        .expect("a diagonal piece");
    let d = chamfer.seg.b - chamfer.seg.a;
    assert_eq!(d.x.abs(), d.y.abs());
    assert!(d.x.abs() >= i64::from(W));
    assert!(clean(&board, &rules, &copper));
    assert_eq!(report.defects_after, 0);
    assert!(report.length_after_nm < report.length_before_nm);
}

#[test]
fn a_via_joining_one_layer_to_itself_is_removed_but_a_layer_change_is_kept() {
    let board = board_two_pads();
    let rules = UniformRules::default();
    let net = board.net_by_name("SIG").unwrap();
    let all = board.layers.all();
    let via = |at: Point| RoutedVia {
        net,
        center: at,
        diameter: 600_000,
        drill: 300_000,
        layers: all,
        gen: Generation(1),
    };
    let mid = p(12.0, 5.0);
    let mut copper = RoutedCopper {
        tracks: vec![
            track(net, 0, p(5.0, 5.0), mid),
            track(net, 0, mid, p(20.0, 5.0)),
            track(net, 0, p(20.0, 5.0), p(20.0, 12.0)),
        ],
        vias: vec![via(mid)],
    };
    let report = optimize(&board, &rules, &mut copper);
    assert_eq!(report.vias_removed, 1);
    assert!(copper.vias.is_empty());
    assert_eq!(islands(&board, &copper), 0);

    // A real layer change stays.
    let mut copper = RoutedCopper {
        tracks: vec![
            track(net, 0, p(5.0, 5.0), mid),
            track(net, 1, mid, p(20.0, 5.0)),
            track(net, 1, p(20.0, 5.0), p(20.0, 11.0)),
        ],
        vias: vec![via(mid)],
    };
    let report = optimize(&board, &rules, &mut copper);
    assert_eq!(report.vias_removed, 0);
    assert_eq!(copper.vias.len(), 1);
}

/// Several nets of staircases, side by side.
fn crowded() -> (RouteBoard, RoutedCopper) {
    let mut b = BoardBuilder::new(40.0, 40.0, &["F.Cu", "B.Cu"]);
    let mut nets = Vec::new();
    for i in 0..8 {
        let name = format!("S{i}");
        let u = b.footprint(&format!("A{i}"), (4.0, 4.0 + 4.0 * i as f64));
        let y = 4.0 + 4.0 * i as f64;
        b.smd_pad(u, "1", &name, (4.0, y), (0.5, 0.5), "F.Cu");
        b.smd_pad(u, "2", &name, (34.0, y + 1.0), (0.5, 0.5), "F.Cu");
        nets.push((name, y));
    }
    let board = b.build();
    let mut tracks = Vec::new();
    for (name, y) in &nets {
        let net = board.net_by_name(name).unwrap();
        tracks.extend(staircase(net, 0, p(4.0, *y), p(34.0, y + 1.0), 100_000));
    }
    (
        board,
        RoutedCopper {
            tracks,
            vias: vec![],
        },
    )
}

#[test]
fn one_and_many_threads_give_identical_copper() {
    let (board, copper) = crowded();
    let rules = UniformRules::default();
    let run = |threads: usize| {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap();
        let mut c = copper.clone();
        let report = pool.install(|| optimize(&board, &rules, &mut c));
        (c, report)
    };
    let (one, r1) = run(1);
    let (many, r4) = run(4);
    assert_eq!(one, many);
    assert_eq!(r1, r4);
    assert!(r1.nets_changed == 8, "{r1:?}");
    assert!(clean(&board, &rules, &one));
    assert_eq!(r1.defects_after, 0, "{r1:?}");
}

#[test]
fn the_report_and_telemetry_are_recorded() {
    let (board, mut copper) = crowded();
    let rules = UniformRules::default();
    let mut t = Telemetry::default();
    let r = optimize_with(
        &board,
        &rules,
        &mut copper,
        &OptimizeOptions::default(),
        &mut t,
    );
    assert!(t.stage_us(Stage::Optimize) > 0 || r.exact_checks > 0);
    assert_eq!(t.counters["optimize_exact_checks"], r.exact_checks);
    assert_eq!(r.tracks_after as usize, copper.tracks.len());
    assert!(r.defects_before > 0, "staircases are jogs");
}

fn pads_board(pads: &[(&str, (f64, f64), (f64, f64))]) -> RouteBoard {
    let mut b = BoardBuilder::new(30.0, 30.0, &["F.Cu", "B.Cu"]);
    for (i, (net, at, size)) in pads.iter().enumerate() {
        let u = b.footprint(&format!("U{i}"), *at);
        b.smd_pad(u, "1", net, *at, *size, "F.Cu");
    }
    b.build()
}

#[test]
fn a_short_step_between_parallel_runs_becomes_a_gentle_slant() {
    let board = pads_board(&[
        ("SIG", (5.0, 10.0), (0.5, 0.5)),
        ("SIG", (20.0, 10.15), (0.5, 0.5)),
    ]);
    let rules = UniformRules::default();
    let net = board.net_by_name("SIG").unwrap();
    let mut copper = RoutedCopper {
        tracks: vec![
            track(net, 0, p(5.0, 10.0), p(12.0, 10.0)),
            track(net, 0, p(12.0, 10.0), p(12.15, 10.15)),
            track(net, 0, p(12.15, 10.15), p(20.0, 10.15)),
        ],
        vias: vec![],
    };
    assert_eq!(report_defects(&rules, &copper), 1, "a jog to start with");
    let opts = OptimizeOptions {
        pull_tight: false,
        ..OptimizeOptions::default()
    };
    let r = optimize_with(
        &board,
        &rules,
        &mut copper,
        &opts,
        &mut Telemetry::default(),
    );
    assert_eq!(r.jogs_smoothed, 1, "{r:?}");
    assert_eq!(r.defects_after, 0, "{r:?}");
    assert_eq!(islands(&board, &copper), 0);
    assert!(clean(&board, &rules, &copper));
}

fn report_defects(rules: &UniformRules, copper: &RoutedCopper) -> usize {
    geometry_defects(rules, &OptimizeOptions::default(), copper).len()
}

#[test]
fn two_tracks_meeting_sharply_in_a_pad_are_pulled_apart_inside_it() {
    let board = pads_board(&[
        ("SIG", (10.0, 10.0), (1.0, 1.0)),
        ("SIG", (4.0, 10.0), (0.5, 0.5)),
        ("SIG", (5.0, 15.0), (0.5, 0.5)),
    ]);
    let rules = UniformRules::default();
    let net = board.net_by_name("SIG").unwrap();
    let mut copper = RoutedCopper {
        tracks: vec![
            track(net, 0, p(4.0, 10.0), p(10.0, 10.0)),
            track(net, 0, p(5.0, 15.0), p(10.0, 10.0)),
        ],
        vias: vec![],
    };
    assert_eq!(report_defects(&rules, &copper), 1, "a 45 degree joint");
    let r = optimize(&board, &rules, &mut copper);
    assert_eq!(r.pad_entries_trimmed, 1, "{r:?}");
    assert_eq!(r.defects_after, 0, "{r:?}");
    assert_eq!(islands(&board, &copper), 0, "both still reach the pad");
    assert!(clean(&board, &rules, &copper));
}

#[test]
fn a_sharp_y_junction_becomes_a_right_angle_t() {
    let board = pads_board(&[
        ("SIG", (5.0, 10.0), (0.5, 0.5)),
        ("SIG", (15.0, 10.0), (0.5, 0.5)),
        ("SIG", (8.0, 12.0), (0.5, 0.5)),
    ]);
    let rules = UniformRules::default();
    let net = board.net_by_name("SIG").unwrap();
    let mut copper = RoutedCopper {
        tracks: vec![
            track(net, 0, p(5.0, 10.0), p(10.0, 10.0)),
            track(net, 0, p(10.0, 10.0), p(15.0, 10.0)),
            track(net, 0, p(8.0, 12.0), p(10.0, 10.0)),
        ],
        vias: vec![],
    };
    assert_eq!(report_defects(&rules, &copper), 1);
    let r = optimize(&board, &rules, &mut copper);
    assert_eq!(r.junctions_squared, 1, "{r:?}");
    assert_eq!(r.defects_after, 0, "{r:?}");
    assert_eq!(islands(&board, &copper), 0);
    assert!(clean(&board, &rules, &copper));
    // The branch now drops straight onto the run.
    assert!(copper
        .tracks
        .iter()
        .any(|t| t.seg == Segment::new(p(8.0, 12.0), p(8.0, 10.0), W)));
}
