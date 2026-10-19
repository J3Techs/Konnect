//! Synthetic tests for negotiation: tiling, BGA sub-tiles, convergence under
//! forced congestion, cross-tile pinning, named failures, and determinism
//! across thread counts. Every board is generated here; nothing is real.

use super::*;
use crate::api::{Ownership, TrackId, ViaId, TILE_CORE_NM, TILE_HALO_NM};
use crate::board::{Track, Via};
use crate::compute::{Plane, TileRaster, Window};
use crate::plan::RoutePlan;
use crate::synthetic::{bga_field, two_pin_nets, BoardBuilder, UniformRules};
use konnect_geom::{gap_cmp, Circle, Segment, Shape};
use obstacles::Obstacles;
use std::cmp::Ordering;

fn route_with(
    board: &RouteBoard,
    settings: NegotiationSettings,
    tuning: NegotiationTuning,
) -> (DetailResult, Telemetry) {
    route_with_rules(board, &UniformRules::default(), settings, tuning)
}

fn route_with_rules(
    board: &RouteBoard,
    rules: &UniformRules,
    settings: NegotiationSettings,
    tuning: NegotiationTuning,
) -> (DetailResult, Telemetry) {
    let connections = ConnectivityReport::build(board, &ArcPolicy::default()).connections(board);
    let mut backend = ReferenceSearch::new(settings.threads);
    let mut telemetry = Telemetry::default();
    let result = negotiate_connections(
        board,
        rules,
        &connections,
        &GlobalResult::default(),
        None,
        &settings,
        &tuning,
        &mut backend,
        &mut telemetry,
    );
    // Every connection is accounted for exactly once.
    assert_eq!(
        result.routed.len() + result.failed.len(),
        connections.len(),
        "{result:?}"
    );
    assert_eq!(result.failed.len(), result.failure_details.len());
    (result, telemetry)
}

fn route(board: &RouteBoard) -> (DetailResult, Telemetry) {
    route_with(
        board,
        NegotiationSettings {
            threads: 2,
            ..NegotiationSettings::default()
        },
        NegotiationTuning::default(),
    )
}

/// The board with the routed copper added as ordinary items.
fn with_copper(board: &RouteBoard, copper: &RoutedCopper) -> RouteBoard {
    let mut b = board.clone();
    for t in &copper.tracks {
        b.tracks.push(Track {
            id: TrackId(b.tracks.len() as u32),
            net: Some(t.net),
            layer: t.layer,
            seg: t.seg,
            ownership: Ownership::Routed {
                net: t.net,
                gen: t.gen,
            },
            uuid: None,
        });
    }
    for v in &copper.vias {
        b.vias.push(Via {
            id: ViaId(b.vias.len() as u32),
            net: Some(v.net),
            center: v.center,
            diameter: v.diameter,
            drill: v.drill,
            layers: v.layers,
            ownership: Ownership::Routed {
                net: v.net,
                gen: v.gen,
            },
            uuid: None,
        });
    }
    b
}

/// An independent brute-force DRC of routed copper under the default
/// `UniformRules`.
fn violations(board: &RouteBoard, copper: &RoutedCopper) -> Vec<String> {
    violations_with(board, copper, &UniformRules::default())
}

/// An independent brute-force DRC of routed copper under `rules`:
/// clearance to other nets' fixed and routed copper, edge clearance,
/// hole clearance between holes (plated holes of other nets, non-plated
/// holes) and copper, and hole-to-hole for new vias. Returns the
/// violations found.
fn violations_with(board: &RouteBoard, copper: &RoutedCopper, rules: &UniformRules) -> Vec<String> {
    let c = i64::from(rules.class.clearance);
    let hc = i64::from(rules.hole_clearance);
    let mut fixed: Vec<(Option<NetId>, LayerMask, Shape)> = Vec::new();
    for p in &board.pads {
        for s in &p.shapes {
            fixed.push((p.net, p.layers, s.clone()));
        }
    }
    for t in &board.tracks {
        fixed.push((t.net, LayerMask::single(t.layer), Shape::Segment(t.seg)));
    }
    for v in &board.vias {
        fixed.push((v.net, v.layers, v.shape()));
    }
    let mut new: Vec<(NetId, LayerMask, Shape)> = Vec::new();
    for t in &copper.tracks {
        new.push((t.net, LayerMask::single(t.layer), Shape::Segment(t.seg)));
    }
    for v in &copper.vias {
        new.push((
            v.net,
            v.layers,
            Shape::Circle(Circle::from_diameter(v.center, v.diameter)),
        ));
    }
    let mut out = Vec::new();
    for (i, (net, layers, shape)) in new.iter().enumerate() {
        for (fnet, flayers, fshape) in &fixed {
            if *fnet != Some(*net)
                && layers.intersects(*flayers)
                && gap_cmp(shape, fshape, c) == Ordering::Less
            {
                out.push(format!("new {i} vs fixed {fshape:?}"));
            }
        }
        for e in &board.outline.edges {
            if gap_cmp(shape, &Shape::Segment(*e), i64::from(rules.edge_clearance))
                == Ordering::Less
            {
                out.push(format!("new {i} vs edge"));
            }
        }
        for (j, (onet, olayers, oshape)) in new.iter().enumerate().skip(i + 1) {
            if onet != net
                && layers.intersects(*olayers)
                && gap_cmp(shape, oshape, c) == Ordering::Less
            {
                out.push(format!("new {i} vs new {j}"));
            }
        }
    }
    // Fixed holes with the net they belong to; `None` (non-plated, or no
    // net) constrains every net.
    let mut fixed_holes: Vec<(Option<NetId>, Shape)> = Vec::new();
    for p in &board.pads {
        if let Some(d) = &p.drill {
            let plated = d.plated && p.kind != crate::board::PadKind::NpThroughHole;
            fixed_holes.push((if plated { p.net } else { None }, d.shape.clone()));
        }
    }
    for v in &board.vias {
        fixed_holes.push((v.net, v.hole()));
    }
    for (i, (net, _, shape)) in new.iter().enumerate() {
        for (hnet, hole) in &fixed_holes {
            if *hnet != Some(*net) && gap_cmp(shape, hole, hc) == Ordering::Less {
                out.push(format!("new {i} vs fixed hole {hole:?}"));
            }
        }
    }
    for (i, v) in copper.vias.iter().enumerate() {
        let hole = Shape::Circle(Circle::from_diameter(v.center, v.drill));
        for (fnet, _, fshape) in &fixed {
            if *fnet != Some(v.net) && gap_cmp(&hole, fshape, hc) == Ordering::Less {
                out.push(format!("via {i} hole vs fixed {fshape:?}"));
            }
        }
        for (j, (onet, _, oshape)) in new.iter().enumerate() {
            if *onet != v.net && gap_cmp(&hole, oshape, hc) == Ordering::Less {
                out.push(format!("via {i} hole vs new {j}"));
            }
        }
    }
    let drills: Vec<Shape> = fixed_holes.iter().map(|(_, h)| h.clone()).collect();
    for (i, v) in copper.vias.iter().enumerate() {
        let hole = Shape::Circle(Circle::from_diameter(v.center, v.drill));
        for d in &drills {
            if gap_cmp(&hole, d, i64::from(rules.hole_to_hole)) == Ordering::Less {
                out.push(format!("via {i} hole vs fixed drill"));
            }
        }
        for (j, w) in copper.vias.iter().enumerate().skip(i + 1) {
            let other = Shape::Circle(Circle::from_diameter(w.center, w.drill));
            if gap_cmp(&hole, &other, i64::from(rules.hole_to_hole)) == Ordering::Less {
                out.push(format!("via {i} hole vs via {j} hole"));
            }
        }
    }
    out
}

fn assert_clean_and_connected(board: &RouteBoard, result: &DetailResult) {
    let v = violations(board, &result.copper);
    assert!(v.is_empty(), "violations: {v:?}");
    let routed = with_copper(board, &result.copper);
    let left = ConnectivityReport::build(&routed, &ArcPolicy::default()).connection_count();
    assert_eq!(
        left,
        result.failed.len(),
        "routed copper must join its pads"
    );
    // No acute joint: consecutive segments of one run turn by at most 90°.
    for w in result.copper.tracks.windows(2) {
        let (a, b) = (&w[0], &w[1]);
        if a.net == b.net && a.layer == b.layer && a.seg.b == b.seg.a {
            let (u, v) = (a.seg.b - a.seg.a, b.seg.b - b.seg.a);
            assert!(u.dot(v) >= 0, "acute joint between {a:?} and {b:?}");
        }
    }
    if !result.copper.tracks.is_empty() {
        RoutePlan::from_routed(board, &result.copper).expect("plan validates");
    }
}

/// `n` nets from the left column to the right column in reverse order on a
/// two-layer board: every net crosses every other.
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

#[test]
fn grid_tiles_cover_the_board_and_same_colours_never_share_a_halo() {
    let mut b = BoardBuilder::new(70.0, 40.0, &["F.Cu", "B.Cu"]);
    b.net("A");
    let board = b.build();
    for shift in [false, true] {
        let tiles = plan_tiles(&board, shift);
        assert!(tiles
            .iter()
            .all(|t| t.phase == 1 && t.pitch == LATTICE_PITCH_NM));
        let bb = board.outline.bbox.unwrap();
        // Cores tile the board: every probe point has exactly one owner.
        for x in (0..=70).step_by(5) {
            for y in (0..=40).step_by(5) {
                let p = Point::new(x * 1_000_000, y * 1_000_000);
                assert!(bb.contains(p));
                let n = tiles
                    .iter()
                    .filter(|t| {
                        p.x >= t.core.min.x
                            && p.x < t.core.max.x
                            && p.y >= t.core.min.y
                            && p.y < t.core.max.y
                    })
                    .count();
                assert_eq!(n, 1, "{p:?}");
            }
        }
        for (i, a) in tiles.iter().enumerate() {
            assert_eq!(a.core.width(), i64::from(TILE_CORE_NM.0));
            assert_eq!(a.window, a.core.inflate(TILE_HALO_NM));
            assert!(a.colour < 4);
            for t in &tiles[i + 1..] {
                if t.colour == a.colour {
                    assert!(!t.window.intersects(&a.window), "{a:?} / {t:?}");
                }
            }
        }
        let first = tiles[0].core.min;
        let expect = if shift {
            Point::new(-TILE_CORE_NM.0 / 2, -TILE_CORE_NM.1 / 2)
        } else {
            Point::new(0, 0)
        };
        assert_eq!(first, expect);
    }
}

#[test]
fn bga_fields_are_split_into_fine_sub_tiles_scheduled_first() {
    let board = bga_field(12, 12, 0.8, 0.4, false);
    let fields = detect_bga_fields(&board, &NegotiationTuning::default());
    assert_eq!(fields.len(), 1);
    assert_eq!(fields[0].pads, 144);
    assert_eq!(fields[0].pitch, 800_000);
    let tiles = plan_tiles(&board, false);
    let subs: Vec<&Tile> = tiles.iter().filter(|t| t.phase == 0).collect();
    assert_eq!(subs.len(), 4);
    assert!(subs.iter().all(|t| t.pitch == LATTICE_FINE_PITCH_NM));
    assert_eq!(
        subs.iter().map(|t| t.id.0).collect::<Vec<_>>(),
        vec![0, 1, 2, 3],
        "sub-tiles come first"
    );
    let mut colours: Vec<u8> = subs.iter().map(|t| t.colour).collect();
    colours.sort_unstable();
    assert_eq!(colours, vec![0, 1, 2, 3]);
    // Sub-cores tile the field region exactly.
    let area: i64 = subs.iter().map(|t| t.core.width() * t.core.height()).sum();
    assert_eq!(area, fields[0].region.width() * fields[0].region.height());
    // A sparse connector-like footprint is not a field.
    let board = two_pin_nets(30, false);
    assert!(detect_bga_fields(&board, &NegotiationTuning::default()).is_empty());
    // Nor is a fine-pitch perimeter package: 16 pads a side at 0.5 mm.
    let mut b = BoardBuilder::new(20.0, 20.0, &["F.Cu", "B.Cu"]);
    let q = b.footprint("U1", (10.0, 10.0));
    for i in 0..16 {
        let t = 6.25 + 0.5 * f64::from(i);
        for (side, at) in [(0, (t, 5.5)), (1, (t, 14.5)), (2, (5.5, t)), (3, (14.5, t))] {
            b.smd_pad(q, &format!("{side}_{i}"), "", at, (0.25, 0.8), "F.Cu");
        }
    }
    let quad = b.build();
    assert!(detect_bga_fields(&quad, &NegotiationTuning::default()).is_empty());
}

#[test]
fn bga_sub_tiles_route_escapes_on_the_fine_lattice() {
    // A 6 × 6 field with the outer ring joined pairwise to pads outside.
    let mut b = BoardBuilder::new(20.0, 20.0, &["F.Cu", "B.Cu"]);
    let u = b.footprint_with_courtyard("U1", (10.0, 10.0), (2.6, 2.6));
    let j = b.footprint("J1", (17.0, 10.0));
    for r in 0..7 {
        for c in 0..7 {
            let at = (7.6 + 0.8 * c as f64, 7.6 + 0.8 * r as f64);
            let net = if c == 6 && r % 2 == 0 {
                format!("S{r}")
            } else {
                String::new()
            };
            b.ball_pad(u, &format!("{r}_{c}"), &net, at, 0.4, "F.Cu");
        }
    }
    for r in (0..7).step_by(2) {
        b.smd_pad(
            j,
            &r.to_string(),
            &format!("S{r}"),
            (17.0, 7.6 + 0.8 * r as f64),
            (0.6, 0.4),
            "F.Cu",
        );
    }
    let board = b.build();
    let tuning = NegotiationTuning {
        bga_min_pads: 36,
        ..NegotiationTuning::default()
    };
    let (result, telemetry) = route_with(
        &board,
        NegotiationSettings {
            threads: 2,
            ..NegotiationSettings::default()
        },
        tuning,
    );
    assert_eq!(result.bga_fields.len(), 1);
    assert!(result.failed.is_empty(), "{:?}", result.failure_details);
    assert_clean_and_connected(&board, &result);
    // Fine-lattice coordinates (25 µm) only appear inside BGA sub-tiles.
    assert!(result
        .copper
        .tracks
        .iter()
        .all(|t| t.seg.a.x % LATTICE_FINE_PITCH_NM == 0 && t.seg.a.y % LATTICE_FINE_PITCH_NM == 0));
    assert!(telemetry.counters["tiles_built"] >= 2);
}

#[test]
fn parallel_two_pin_nets_route_in_one_iteration() {
    let board = two_pin_nets(6, false);
    let (result, telemetry) = route(&board);
    assert!(result.failed.is_empty(), "{:?}", result.failure_details);
    assert_eq!(result.routed.len(), 6);
    assert_eq!(result.iterations, 1, "{:?}", result.reports);
    assert_eq!(result.reports[0].conflict_pairs, 0);
    assert_clean_and_connected(&board, &result);
    assert!(telemetry.stage_us(Stage::Detail) > 0);
    assert!(telemetry.expansions_per_connection.total >= 6);
    assert_eq!(telemetry.threads, 2);
}

#[test]
fn negotiation_converges_on_a_crossing_bundle() {
    let board = crossing_bundle(6);
    // Iteration 0 blind to other nets forces conflicts that negotiation
    // must then resolve.
    let tuning = NegotiationTuning {
        initial_present_factor: 0,
        ..NegotiationTuning::default()
    };
    let (result, telemetry) = route_with(
        &board,
        NegotiationSettings {
            threads: 2,
            max_iterations: 20,
            ..NegotiationSettings::default()
        },
        tuning,
    );
    let r = &result.reports;
    assert!(r[0].conflict_pairs > 0, "{r:?}");
    assert!(result.iterations > 1);
    assert_eq!(r.last().unwrap().conflicted, 0, "{r:?}");
    assert!(result.failed.is_empty(), "{:?}", result.failure_details);
    assert_clean_and_connected(&board, &result);
    assert!(
        !result.copper.vias.is_empty(),
        "crossings need the other layer"
    );
    // Only conflicting connections were ripped up after iteration 0.
    for w in r.windows(2) {
        assert!(w[1].queued <= w[0].conflicted + w[0].failed, "{r:?}");
    }
    // History never decays.
    for w in r.windows(2) {
        assert!(w[1].history_total >= w[0].history_total);
        assert!(w[1].history_cells >= w[0].history_cells);
    }
    assert!(r[0].history_total > 0);
    // The present factor grows by `present_growth` per iteration.
    assert_eq!(r[1].present_factor, 500, "the floor lifts a blind start");
    if r.len() > 2 {
        assert_eq!(r[2].present_factor, 750);
    }
    assert!(telemetry.counters["rip_ups"] > 0);
    assert!(telemetry.congested_iteration_us > 0);
}

#[test]
fn unresolved_conflicts_keep_copper_greedily_and_fail_the_rest() {
    let board = crossing_bundle(6);
    // One iteration blind to congestion leaves conflicts unresolved.
    let tuning = NegotiationTuning {
        initial_present_factor: 0,
        ..NegotiationTuning::default()
    };
    let (result, _) = route_with(
        &board,
        NegotiationSettings {
            threads: 2,
            max_iterations: 1,
            ..NegotiationSettings::default()
        },
        tuning,
    );
    assert!(result.reports[0].conflicted > 0);
    assert!(!result.routed.is_empty());
    assert!(!result.failed.is_empty());
    for ((_, reason), (_, detail)) in result.failed.iter().zip(&result.failure_details) {
        assert_eq!(*reason, FailureReason::Congestion { iterations: 1 });
        assert!(matches!(detail, DetailFailure::Congestion { against, .. } if !against.is_empty()));
    }
    // What remains is legal.
    let v = violations(&board, &result.copper);
    assert!(v.is_empty(), "{v:?}");
}

/// Long nets across several tiles.
fn long_board() -> RouteBoard {
    let mut b = BoardBuilder::new(70.0, 34.0, &["F.Cu", "B.Cu"]);
    let l = b.footprint("J1", (3.0, 17.0));
    let r = b.footprint("J2", (67.0, 17.0));
    for i in 0..4 {
        let net = format!("L{i}");
        b.smd_pad(
            l,
            &(i + 1).to_string(),
            &net,
            (3.0, 5.0 + 7.0 * i as f64),
            (0.8, 0.5),
            "F.Cu",
        );
        b.smd_pad(
            r,
            &(i + 1).to_string(),
            &net,
            (67.0, 29.0 - 7.0 * i as f64),
            (0.8, 0.5),
            "F.Cu",
        );
    }
    b.build()
}

#[test]
fn long_connections_are_pinned_across_tiles() {
    let board = long_board();
    let (result, telemetry) = route(&board);
    assert!(result.failed.is_empty(), "{:?}", result.failure_details);
    assert!(result.reports[0].pins >= 8, "{:?}", result.reports);
    assert!(result.reports[0].pieces > result.reports[0].queued);
    assert_clean_and_connected(&board, &result);
    assert!(telemetry.counters["pins"] >= 8);
}

#[test]
fn results_do_not_depend_on_thread_count() {
    let tuning = NegotiationTuning {
        initial_present_factor: 0,
        chunk_tiles: 0,
        ..NegotiationTuning::default()
    };
    for board in [crossing_bundle(6), long_board()] {
        let run = |threads| {
            route_with(
                &board,
                NegotiationSettings {
                    threads,
                    max_iterations: 12,
                    ..NegotiationSettings::default()
                },
                tuning,
            )
            .0
        };
        let one = run(1);
        for threads in [3, 8] {
            let many = run(threads);
            assert_eq!(one, many, "{threads} threads");
            let a = RoutePlan::from_routed(&board, &one.copper).unwrap();
            let b = RoutePlan::from_routed(&board, &many.copper).unwrap();
            assert_eq!(a.to_routing_file_json(), b.to_routing_file_json());
        }
        // Chunking (how many tiles are alive at once, which only the
        // lockstep path reads) never matters either.
        let chunked = route_with(
            &board,
            NegotiationSettings {
                threads: 2,
                max_iterations: 12,
                ..NegotiationSettings::default()
            },
            NegotiationTuning {
                chunk_tiles: 1,
                lockstep_rounds: true,
                ..tuning
            },
        )
        .0;
        assert_eq!(one, chunked);
        // Nor does reusing hard raster state across iterations.
        let uncached = route_with(
            &board,
            NegotiationSettings {
                threads: 3,
                max_iterations: 12,
                ..NegotiationSettings::default()
            },
            NegotiationTuning {
                raster_cache_mb: 0,
                ..tuning
            },
        )
        .0;
        assert_eq!(one, uncached);
        // Nor does lockstep batching versus per-tile backend copies.
        let lockstep = route_with(
            &board,
            NegotiationSettings {
                threads: 4,
                max_iterations: 12,
                ..NegotiationSettings::default()
            },
            NegotiationTuning {
                lockstep_rounds: true,
                ..tuning
            },
        )
        .0;
        assert_eq!(one, lockstep);
    }
}

#[test]
fn unroutable_connections_get_named_reasons() {
    let mut b = BoardBuilder::new(30.0, 20.0, &["F.Cu", "In1.Cu", "B.Cu"]);
    // A pad walled in on every layer by a locked square of another net.
    let u = b.footprint("U1", (8.0, 10.0));
    b.smd_pad(u, "1", "TRAPPED", (8.0, 10.0), (0.6, 0.6), "F.Cu");
    let j = b.footprint("J1", (25.0, 10.0));
    b.smd_pad(j, "1", "TRAPPED", (25.0, 10.0), (0.6, 0.6), "F.Cu");
    for layer in ["F.Cu", "In1.Cu", "B.Cu"] {
        let pts = [
            (6.0, 8.0),
            (10.0, 8.0),
            (10.0, 12.0),
            (6.0, 12.0),
            (6.0, 8.0),
        ];
        for w in pts.windows(2) {
            b.track("WALL", layer, w[0], w[1], 0.3, true);
        }
    }
    // A pad only on an inner layer the router may not use.
    let k = b.footprint("K1", (20.0, 4.0));
    b.smd_pad(k, "1", "INNER", (18.0, 4.0), (0.6, 0.6), "In1.Cu");
    b.smd_pad(k, "2", "INNER", (22.0, 4.0), (0.6, 0.6), "In1.Cu");
    // And an ordinary net that must still route.
    let m = b.footprint("M1", (15.0, 17.0));
    b.smd_pad(m, "1", "FREE", (12.0, 17.0), (0.6, 0.6), "F.Cu");
    b.smd_pad(m, "2", "FREE", (18.0, 17.0), (0.6, 0.6), "F.Cu");
    let mut board = b.build();
    board.routable = LayerMask::single(LayerId(0)).with(LayerId(2));
    let (result, _) = route(&board);
    let name = |n: &str| board.net_by_name(n).unwrap();
    let conns = ConnectivityReport::build(&board, &ArcPolicy::default()).connections(&board);
    let conn_of = |n: &str| conns.iter().find(|c| c.net == name(n)).unwrap().id;
    let detail = |n: &str| {
        result
            .failure_details
            .iter()
            .find(|(c, _)| *c == conn_of(n))
            .map(|(_, f)| f.clone())
    };
    assert_eq!(detail("FREE"), None);
    assert!(result.routed.contains(&conn_of("FREE")));
    assert!(
        matches!(
            detail("TRAPPED"),
            Some(DetailFailure::Unreachable | DetailFailure::NoPin { .. })
        ),
        "{:?}",
        detail("TRAPPED")
    );
    assert!(matches!(
        detail("INNER"),
        Some(DetailFailure::NoAccess {
            item: ItemRef::Pad(_)
        })
    ));
    let reason = |n: &str| {
        result
            .failed
            .iter()
            .find(|(c, _)| *c == conn_of(n))
            .map(|(_, r)| r.clone())
    };
    assert_eq!(reason("TRAPPED"), Some(FailureReason::NoCorridor));
    assert!(matches!(
        reason("INNER"),
        Some(FailureReason::NoAccess { .. })
    ));
    // A hard failure is retried once in the shifted tiling, then given up.
    assert_eq!(result.iterations, 2, "{:?}", result.reports);
    assert_clean_and_connected(&board, &result);
}

#[test]
fn corridor_layers_and_missing_corridors_are_respected() {
    let board = two_pin_nets(3, false);
    let conns = ConnectivityReport::build(&board, &ArcPolicy::default()).connections(&board);
    let global = GlobalResult {
        corridors: vec![
            crate::global::Corridor {
                connection: conns[0].id,
                cells: Vec::new(),
                layers: LayerMask::single(LayerId(1)),
            },
            crate::global::Corridor {
                connection: conns[1].id,
                cells: Vec::new(),
                layers: board.layers.all(),
            },
        ],
        ..GlobalResult::default()
    };
    let rules = UniformRules::default();
    let mut backend = ReferenceSearch::new(1);
    let mut telemetry = Telemetry::default();
    let result = negotiate_connections(
        &board,
        &rules,
        &conns,
        &global,
        None,
        &NegotiationSettings::default(),
        &NegotiationTuning::default(),
        &mut backend,
        &mut telemetry,
    );
    // conns[0] may only use B.Cu, which its F.Cu pads cannot reach.
    let detail = |id| {
        result
            .failure_details
            .iter()
            .find(|(c, _)| *c == id)
            .map(|(_, f)| f.clone())
    };
    assert!(matches!(
        detail(conns[0].id),
        Some(DetailFailure::NoAccess { .. })
    ));
    assert_eq!(detail(conns[1].id), None);
    assert_eq!(detail(conns[2].id), Some(DetailFailure::NoCorridor));
}

#[test]
fn failure_details_map_onto_the_frozen_reasons() {
    let item = ItemRef::Pad(crate::api::PadId(3));
    assert_eq!(
        DetailFailure::NoAccess { item }.reason(),
        FailureReason::NoAccess { item }
    );
    assert_eq!(
        DetailFailure::Unreachable.reason(),
        FailureReason::NoCorridor
    );
    assert_eq!(
        DetailFailure::NoPin {
            from: TileId(0),
            to: TileId(1)
        }
        .reason(),
        FailureReason::NoCorridor
    );
    assert_eq!(
        DetailFailure::ExactCheckRejected.reason(),
        FailureReason::BudgetExhausted
    );
    assert_eq!(
        DetailFailure::Congestion {
            iterations: 7,
            against: vec![]
        }
        .reason(),
        FailureReason::Congestion { iterations: 7 }
    );
    assert!(!DetailFailure::NoAccess { item }.retryable());
    assert!(DetailFailure::BudgetExhausted.retryable());
}

#[test]
fn via_sites_erode_by_the_via_reach() {
    let w = Window::new(Point::new(0, 0), 50_000, 21, 21).unwrap();
    let mut r = TileRaster::new(TileId(0), LayerId(0), crate::api::ProfileId(0), w);
    r.via_site.fill(1);
    *r.halo.get_mut(10, 10) = 1;
    run::erode_for_test(&mut r, 2);
    for row in 0..21u32 {
        for col in 0..21u32 {
            let near_halo = col.abs_diff(10) <= 2 && row.abs_diff(10) <= 2;
            let near_edge = !(2..19).contains(&col) || !(2..19).contains(&row);
            let expect = u8::from(!(near_halo || near_edge));
            assert_eq!(r.via_site.get(col, row), expect, "({col}, {row})");
        }
    }
    let _: Plane<u8> = Plane::new(1, 1);
}

#[test]
fn reference_search_refuses_acute_turns_and_walls() {
    use crate::compute::{JobDescriptor, JobNode, SearchBackend};
    let w = Window::new(Point::new(0, 0), 50_000, 30, 30).unwrap();
    let mut layers = vec![TileRaster::new(
        TileId(0),
        LayerId(0),
        crate::api::ProfileId(0),
        w,
    )];
    // A wall across the window with one gap.
    for row in 0..30 {
        if row != 25 {
            *layers[0].core.get_mut(15, row) = 1;
        }
    }
    let mut backend = ReferenceSearch::new(1);
    for r in &mut layers {
        backend.derive_masks(r);
    }
    let stack = crate::compute::TileStack {
        tile: TileId(0),
        profile: crate::api::ProfileId(0),
        window: w,
        layers,
    };
    let node = |c, r| JobNode {
        col: c,
        row: r,
        layer: 0,
        heading_mask: 0xff,
        reserved: 0,
    };
    let mut pool = JobPool::default();
    pool.push(
        JobDescriptor {
            layer_mask: 1,
            cost_bound: u32::MAX,
            ..JobDescriptor::default()
        },
        &[node(2, 2)],
        &[node(28, 2)],
        &[],
    );
    let out = backend.search_batch(
        &Batch {
            index: 0,
            jobs: vec![0],
        },
        &pool,
        std::slice::from_ref(&stack),
        &CostParams::default(),
    );
    let SearchOutcome::Found(p) = &out[0] else {
        panic!("{out:?}");
    };
    assert!(
        p.nodes.iter().any(|n| n.col == 15 && n.row == 25),
        "through the gap"
    );
    assert!(p
        .nodes
        .iter()
        .all(|n| stack.layers[0].core.get(u32::from(n.col), u32::from(n.row)) == 0));
    // Headings along the path never turn by more than 90°.
    let dirs: Vec<(i32, i32)> = p
        .nodes
        .windows(2)
        .map(|w| {
            (
                i32::from(w[1].col) - i32::from(w[0].col),
                i32::from(w[1].row) - i32::from(w[0].row),
            )
        })
        .collect();
    for d in dirs.windows(2) {
        assert!(d[0].0 * d[1].0 + d[0].1 * d[1].1 >= 0, "{d:?}");
    }
    // Close the gap: unreachable.
    let mut stack = stack;
    *stack.layers[0].core.get_mut(15, 25) = 1;
    let out = backend.search_batch(
        &Batch {
            index: 0,
            jobs: vec![0],
        },
        &pool,
        std::slice::from_ref(&stack),
        &CostParams::default(),
    );
    assert!(
        matches!(out[0], SearchOutcome::Unreachable { .. }),
        "{out:?}"
    );
    let _ = Segment::new(Point::new(0, 0), Point::new(0, 0), 0);
}

#[test]
#[ignore = "diagnostic"]
fn diag_crossing_bundle() {
    for (shift, board) in [
        (false, crossing_bundle(6)),
        (true, crossing_bundle(6)),
        (true, long_board()),
    ] {
        let tuning = NegotiationTuning {
            initial_present_factor: 0,
            shift_odd_iterations: shift,
            ..NegotiationTuning::default()
        };
        let (result, _) = route_with(
            &board,
            NegotiationSettings {
                threads: 2,
                max_iterations: 20,
                ..NegotiationSettings::default()
            },
            tuning,
        );
        for r in &result.reports {
            eprintln!(
                "shift={shift} it={} pf={} queued={} pins={} pairs={} conflicted={} hist={}",
                r.iteration,
                r.present_factor,
                r.queued,
                r.pins,
                r.conflict_pairs,
                r.conflicted,
                r.history_total
            );
        }
        eprintln!(
            "vias={} tracks={} failed={:?}",
            result.copper.vias.len(),
            result.copper.tracks.len(),
            result.failure_details
        );
    }
}

/// Rules where a plated hole's clearance reaches well past its copper's.
fn hole_rules() -> UniformRules {
    let mut rules = UniformRules {
        hole_clearance: 300_000,
        ..UniformRules::default()
    };
    rules.class.clearance = 100_000;
    rules
}

#[test]
fn plated_holes_keep_hole_clearance_from_other_nets() {
    // A fence of other nets' vias with thin annular rings: copper
    // clearance alone leaves a lane between every two, hole clearance
    // closes them, so the route must go round the fence.
    let mut b = BoardBuilder::new(20.0, 12.0, &["F.Cu", "B.Cu"]);
    let l = b.footprint("J1", (3.0, 6.0));
    let r = b.footprint("J2", (17.0, 6.0));
    b.smd_pad(l, "1", "A", (3.0, 6.0), (0.6, 0.6), "F.Cu");
    b.smd_pad(r, "1", "A", (17.0, 6.0), (0.6, 0.6), "F.Cu");
    let fence: Vec<f64> = (0..9).map(|i| 2.4 + 0.9 * f64::from(i)).collect();
    for (i, &y) in fence.iter().enumerate() {
        b.via(&format!("F{i}"), (10.0, y), 0.45, 0.4, true);
    }
    let board = b.build();
    let rules = hole_rules();
    let (result, _) = route_with_rules(
        &board,
        &rules,
        NegotiationSettings {
            threads: 2,
            ..NegotiationSettings::default()
        },
        NegotiationTuning::default(),
    );
    assert!(result.failed.is_empty(), "{:?}", result.failure_details);
    let v = violations_with(&board, &result.copper, &rules);
    assert!(v.is_empty(), "violations: {v:?}");
    // It crosses the fence's line beyond the fence's ends.
    let (lo, hi) = (fence[0] * 1e6, fence[fence.len() - 1] * 1e6);
    let x = 10_000_000i64;
    let crossings: Vec<f64> = result
        .copper
        .tracks
        .iter()
        .filter_map(|t| {
            let (a, b) = (t.seg.a, t.seg.b);
            let (ax, bx) = (i64::from(a.x), i64::from(b.x));
            ((ax - x) * (bx - x) <= 0 && ax != bx).then(|| {
                f64::from(a.y)
                    + (f64::from(b.y) - f64::from(a.y)) * (x - ax) as f64 / (bx - ax) as f64
            })
        })
        .collect();
    assert!(!crossings.is_empty());
    assert!(
        crossings.iter().all(|&y| y < lo || y > hi),
        "{crossings:?} within the fence"
    );
}

#[test]
fn hole_clearance_decides_via_and_routed_conflicts() {
    use super::copper::{PieceCopper, SegCopper, ViaCopper};
    let mut b = BoardBuilder::new(12.0, 10.0, &["F.Cu", "B.Cu"]);
    b.net("A");
    // Another net's locked track 0.45 mm from where a via goes: its copper
    // keeps 0.15 mm from the via's disc, but only 0.175 mm from its hole.
    b.track("B", "F.Cu", (4.0, 5.45), (8.0, 5.45), 0.15, true);
    let board = b.build();
    let net_a = board.net_by_name("A").unwrap();
    let net_b = board.net_by_name("B").unwrap();
    let at = Point::new(6_000_000, 5_000_000);
    let disc = Circle::from_diameter(at, 450_000);
    let hole = Circle::from_diameter(at, 400_000);
    let tight = hole_rules();
    let loose = UniformRules {
        hole_clearance: 150_000,
        ..tight.clone()
    };
    let obstacles = Obstacles::build(&board, &tight);
    assert!(obstacles
        .check_via(&tight, net_a, &disc, &hole, &[])
        .is_some());
    let obstacles = Obstacles::build(&board, &loose);
    assert!(obstacles
        .check_via(&loose, net_a, &disc, &hole, &[])
        .is_none());

    // The same geometry between two routed connections.
    let via = PieceCopper::new(
        0,
        0,
        net_a,
        Vec::new(),
        vec![ViaCopper {
            center: at,
            diameter: 450_000,
            drill: 400_000,
            courtyards: Vec::new(),
        }],
    );
    let track = PieceCopper::new(
        1,
        0,
        net_b,
        vec![SegCopper {
            layer: LayerId(0),
            seg: Segment::new(
                Point::new(4_000_000, 5_450_000),
                Point::new(8_000_000, 5_450_000),
                150_000,
            ),
            courtyards: Vec::new(),
        }],
        Vec::new(),
    );
    for (rules, expect) in [(&tight, 1), (&loose, 0)] {
        let index = RoutedIndex::build(
            vec![Arc::new(via.clone()), Arc::new(track.clone())],
            2,
            rules.max_clearance() + 1,
        );
        let pairs = index.conflicts(rules, board.layers.all(), &|_: &PieceCopper| true);
        assert_eq!(pairs.len(), expect, "{pairs:?}");
    }
}

#[test]
fn a_panicking_tile_stops_the_run_instead_of_hanging() {
    use crate::compute::{CostField, JobPool, PaintShape, SearchBackend};
    /// A backend whose every search panics, on every fork.
    struct Panicky;
    impl SearchBackend for Panicky {
        fn search_batch(
            &mut self,
            _: &Batch,
            _: &JobPool,
            _: &[crate::compute::TileStack],
            _: &CostParams,
        ) -> Vec<SearchOutcome> {
            panic!("search failed")
        }
        fn cost_field(
            &mut self,
            _: &crate::compute::TileStack,
            _: &[crate::compute::JobNode],
            _: &CostParams,
        ) -> CostField {
            unreachable!()
        }
    }
    impl RouteCompute for Panicky {
        fn paint(&mut self, r: &mut TileRaster, s: &[PaintShape], p: &ProfileGeometry) {
            crate::compute::CpuCompute::new().paint(r, s, p);
        }
        fn unpaint(&mut self, r: &mut TileRaster, s: &[PaintShape], p: &ProfileGeometry) {
            crate::compute::CpuCompute::new().unpaint(r, s, p);
        }
        fn derive_masks(&mut self, r: &mut TileRaster) {
            ReferenceSearch::new(1).derive_masks(r);
        }
        fn update_costs(&mut self, _: &mut TileRaster, _: &CostParams) {}
        fn fork(&self) -> Option<Box<dyn RouteCompute + Send>> {
            Some(Box::new(Panicky))
        }
    }
    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let board = long_board();
        let rules = UniformRules::default();
        let connections =
            ConnectivityReport::build(&board, &ArcPolicy::default()).connections(&board);
        let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            negotiate_connections(
                &board,
                &rules,
                &connections,
                &GlobalResult::default(),
                None,
                &NegotiationSettings {
                    threads: 4,
                    ..NegotiationSettings::default()
                },
                &NegotiationTuning::default(),
                &mut Panicky,
                &mut Telemetry::default(),
            )
        }));
        let _ = tx.send(outcome.is_err());
    });
    let panicked = rx
        .recv_timeout(std::time::Duration::from_secs(120))
        .expect("the run finishes instead of hanging");
    assert!(panicked, "the tile's panic reaches the caller");
}

#[test]
fn no_routable_layers_fail_as_no_corridor() {
    let mut board = two_pin_nets(3, false);
    board.routable = LayerMask::default();
    let (result, _) = route(&board);
    assert_eq!(result.failed.len(), 3);
    assert!(result
        .failure_details
        .iter()
        .all(|(_, f)| *f == DetailFailure::NoCorridor));
}

#[test]
fn pinned_runs_are_merged_and_short_segments_are_reported() {
    let board = long_board();
    let rules = UniformRules {
        min_segment_length: Some(1_000_000),
        ..UniformRules::default()
    };
    let (result, telemetry) = route_with_rules(
        &board,
        &rules,
        NegotiationSettings {
            threads: 2,
            ..NegotiationSettings::default()
        },
        NegotiationTuning::default(),
    );
    assert!(result.failed.is_empty(), "{:?}", result.failure_details);
    assert_clean_and_connected(&board, &result);
    // Straight runs through tile pins come back as one track each.
    assert!(telemetry.counters["merged_segments"] > 0);
    let short = |s: &Segment| (s.b - s.a).length2() < 1_000_000i128 * 1_000_000;
    let expected = result
        .copper
        .tracks
        .iter()
        .filter(|t| short(&t.seg))
        .count();
    assert_eq!(result.short_segments.len(), expected);
    assert!(result
        .short_segments
        .iter()
        .all(|s| short(&s.seg) && s.min_length == 1_000_000));
    // Without the rule nothing is reported.
    let (plain, _) = route(&board);
    assert!(plain.short_segments.is_empty());
}
