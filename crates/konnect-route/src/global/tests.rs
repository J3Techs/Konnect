use super::grid::{Topology, Win};
use super::*;
use crate::api::{ClassId, LayerId, LayerMask};
use crate::board::KeepoutRules;
use crate::connectivity::ConnectivityReport;
use crate::synthetic::{self, BoardBuilder, UniformRules};
use konnect_geom::ArcPolicy;

fn connections(board: &RouteBoard) -> Vec<Connection> {
    ConnectivityReport::build(board, &ArcPolicy::default()).connections(board)
}

/// Every corridor walks adjacent cells (or changes layer in place, by one
/// through via between two allowed layers) from its `from` terminal's cell
/// to its `to` terminal's cell, on layers its class allows.
fn check_corridors(board: &RouteBoard, grid: &GCellGrid, conns: &[Connection], r: &GlobalResult) {
    for cor in &r.corridors {
        assert!(!cor.cells.is_empty());
        let conn = conns.iter().find(|c| c.id == cor.connection).unwrap();
        let area = |id: u32| grid.cell_bbox(id).expect("corridor ids are valid");
        let pad_point = |item: crate::api::ItemRef| match item {
            crate::api::ItemRef::Pad(p) => board.pads[p.index()].bbox().unwrap().center(),
            _ => unreachable!("synthetic connections join pads"),
        };
        let (_, first) = area(cor.cells[0]);
        let (_, last) = area(*cor.cells.last().unwrap());
        assert!(first.contains(pad_point(conn.from)) || cor.cells.len() > 1);
        assert!(last.contains(pad_point(conn.to)));
        let mut layers = LayerMask::EMPTY;
        for w in cor.cells.windows(2) {
            let (la, a) = area(w[0]);
            let (lb, b) = area(w[1]);
            layers = layers.with(la).with(lb);
            if la == lb {
                // Edge-adjacent: the boxes share a side of positive length.
                let shared_x = a.max.x.min(b.max.x) - a.min.x.max(b.min.x);
                let shared_y = a.max.y.min(b.max.y) - a.min.y.max(b.min.y);
                assert!(
                    (shared_x == 0 && shared_y > 0) || (shared_y == 0 && shared_x > 0),
                    "cells {} and {} are not adjacent",
                    w[0],
                    w[1]
                );
            } else {
                assert_eq!(a, b, "a layer change stays in one cell");
            }
        }
        if cor.cells.len() == 1 {
            layers = area(cor.cells[0]).0.into_mask();
        }
        assert_eq!(layers, cor.assigned);
        assert_eq!(
            cor.assigned.0 & !cor.layers.0,
            0,
            "assigned layers are allowed"
        );
        assert_eq!(cor.layers.0 & !grid.layers.0, 0);
    }
}

trait IntoMask {
    fn into_mask(self) -> LayerMask;
}

impl IntoMask for LayerId {
    fn into_mask(self) -> LayerMask {
        LayerMask::single(self)
    }
}

#[test]
fn ids_bboxes_and_locate_round_trip() {
    let board = synthetic::bga_field(6, 6, 0.8, 0.4, false);
    let settings = GlobalSettings {
        refine_min_pads: 6,
        ..Default::default()
    };
    let rules = UniformRules::default();
    let grid = build_grid(&board, &rules, &settings);
    assert_eq!(grid.layer_slots(), 3);
    assert_eq!(grid.refine_factor(), 2);
    assert!(!grid.refined.is_empty(), "a BGA field is dense");
    // Frozen id formula.
    assert_eq!(grid.id(1, 2, 3), (grid.rows + 3) * grid.cols + 2);
    let mut seen = 0;
    for id in 0..grid.cell_count() {
        let (layer, b) = grid.cell_bbox(id).unwrap();
        let base = grid.cols * grid.rows;
        let dead =
            id < grid.layer_slots() * base && grid.refined.binary_search(&(id % base)).is_ok();
        let c = b.center();
        if dead {
            assert_ne!(grid.locate(layer, c), Some(id));
        } else {
            assert_eq!(grid.locate(layer, c), Some(id), "id {id}");
            seen += 1;
        }
    }
    assert_eq!(
        seen,
        grid.layer_slots() * (grid.cols * grid.rows + 3 * grid.refined.len() as u32)
    );
    assert!(grid.cell_bbox(grid.cell_count()).is_none());
    assert_eq!(grid.layer_slot(LayerId(1)), Some(1));
    assert_eq!(grid.slot_layer(2), Some(LayerId(2)));
}

#[test]
fn topology_edges_are_symmetric_and_cover_mixed_boundaries() {
    let board = synthetic::bga_field(6, 6, 0.8, 0.4, false);
    let settings = GlobalSettings {
        refine_min_pads: 6,
        ..Default::default()
    };
    let grid = super::grid::lay_out(&board, &settings);
    let t = Topology::new(&grid);
    for c in 0..t.nc {
        let (a, b) = (t.adj_off[c as usize], t.adj_off[c as usize + 1]);
        if !t.alive[c as usize] {
            assert_eq!(a, b, "a refined base cell has no edges");
            continue;
        }
        for i in a..b {
            let n = t.adj_cell[i as usize];
            assert_eq!(t.edge_between(n, c), Some(t.adj_edge[i as usize]));
        }
        // Interior cells have a neighbour on every side.
        let (x0, y0, s) = (t.x0[c as usize], t.y0[c as usize], t.size[c as usize]);
        let gx1 = grid.bbox.max.x;
        let gy1 = grid.bbox.max.y;
        let sides = [
            x0 > grid.bbox.min.x,
            y0 > grid.bbox.min.y,
            x0 + s < gx1,
            y0 + s < gy1,
        ]
        .iter()
        .filter(|&&v| v)
        .count();
        assert!((b - a) as usize >= sides);
    }
    // Total boundary length on one layer equals that of an unrefined grid.
    let span: f64 = (0..t.edges as usize)
        .map(|e| {
            let (a, b) = (t.edge_a[e] as usize, t.edge_b[e] as usize);
            f64::from(t.size[a].min(t.size[b]))
        })
        .sum();
    let (c, r) = (f64::from(grid.cols), f64::from(grid.rows));
    let pitch = f64::from(grid.pitch);
    let refined_extra = grid.refined.len() as f64 * 2.0 * pitch; // two inner lines of one pitch
    assert_eq!(
        span,
        ((c - 1.0) * r + (r - 1.0) * c) * pitch + refined_extra
    );
}

#[test]
fn free_edges_hold_whole_track_pitches_and_the_edge_blocks() {
    // A blank 10 × 10 mm two-layer board.
    let board = BoardBuilder::new(10.0, 10.0, &["F.Cu", "B.Cu"]).build();
    let rules = UniformRules::default(); // 0.15 tracks, 0.2 clearance: pitch 0.35
    let grid = build_grid(&board, &rules, &GlobalSettings::default());
    let t = Topology::new(&grid);
    assert_eq!((grid.cols, grid.rows), (10, 10));
    let cap_of = |slot: u32, a: (u32, u32), b: (u32, u32)| {
        let ca = a.1 * grid.cols + a.0;
        let cb = b.1 * grid.cols + b.0;
        let e = t.edge_between(ca, cb).unwrap();
        grid.capacity[t.planar_res(slot, e) as usize]
    };
    // Tracks pack at 0.35 mm along the whole strip from the edge clearance
    // (0.3 mm + half a track = 0.375 mm) to the far one: 9.25 mm holds
    // 27, and each 1 mm edge gets 2 or 3 of them.
    let column: u32 = (0..10).map(|r| u32::from(cap_of(0, (4, r), (5, r)))).sum();
    assert_eq!(column, 27);
    let row: u32 = (0..10).map(|c| u32::from(cap_of(1, (c, 4), (c, 5)))).sum();
    assert_eq!(row, 27);
    for r in 1..9 {
        assert!((2..=3).contains(&cap_of(0, (4, r), (5, r))));
    }
    // The outer row holds the tracks at 0.375 and 0.725 mm.
    assert_eq!(cap_of(0, (4, 0), (5, 0)), 2);
    // Via sites: 1 mm² / 0.8² mm² ≈ 1.56 → 2 in an interior cell.
    assert_eq!(grid.capacity[t.via_res(4 * grid.cols + 4) as usize], 2);
}

#[test]
fn pads_and_keepouts_remove_capacity() {
    let mut b = BoardBuilder::new(10.0, 10.0, &["F.Cu", "B.Cu"]);
    let f = b.footprint("U1", (5.0, 5.0));
    // A 3 × 3 mm pad covering cells 4..=6.
    b.smd_pad(f, "1", "A", (5.5, 5.5), (3.0, 3.0), "F.Cu");
    b.keepout(
        "K",
        &["B.Cu"],
        (0.0, 2.0, 10.0, 3.0),
        KeepoutRules {
            tracks: true,
            vias: true,
            ..Default::default()
        },
    );
    let board = b.build();
    let grid = build_grid(&board, &UniformRules::default(), &GlobalSettings::default());
    let t = Topology::new(&grid);
    let edge = |a: u32, b: u32| t.edge_between(a, b).unwrap();
    let cell = |c: u32, r: u32| r * grid.cols + c;
    // Under the pad on F.Cu: nothing crosses.
    let e = edge(cell(5, 5), cell(6, 5));
    assert_eq!(grid.capacity[t.planar_res(0, e) as usize], 0);
    assert!(grid.capacity[t.planar_res(1, e) as usize] >= 2);
    // Through the keepout on B.Cu: nothing crosses vertically or
    // horizontally inside it.
    let e = edge(cell(1, 2), cell(2, 2));
    assert_eq!(grid.capacity[t.planar_res(1, e) as usize], 0);
    assert!(grid.capacity[t.planar_res(0, e) as usize] >= 2);
    // No via site under the pad or in the via keepout.
    assert_eq!(grid.capacity[t.via_res(cell(5, 5)) as usize], 0);
    assert_eq!(grid.capacity[t.via_res(cell(1, 2)) as usize], 0);
    assert!(super::capacity::window_total(&grid, &Win::full(&t)) > 0);
}

#[test]
fn two_pin_nets_route_without_overflow() {
    let board = synthetic::two_pin_nets(6, false);
    let rules = UniformRules::default();
    let settings = GlobalSettings::default();
    let grid = build_grid(&board, &rules, &settings);
    let conns = connections(&board);
    assert_eq!(conns.len(), 6);
    let mut tel = crate::telemetry::Telemetry::default();
    let r = route_global_with(
        &board,
        &rules,
        &grid,
        &conns,
        &settings,
        &LayerPolicy::default(),
        &mut tel,
    );
    assert_eq!(r.corridors.len(), 6);
    assert!(r.failed.is_empty());
    assert!(r.overflowed.is_empty());
    assert!(r.stats.converged);
    assert_eq!(r.overflow.len(), grid.cell_count() as usize);
    assert!(r.overflow.iter().all(|&v| v == 0));
    assert_eq!(r.stats.pattern_seeds, 6);
    check_corridors(&board, &grid, &conns, &r);
    // Straight horizontal runs on the horizontal layer: 16 cells each.
    for c in &r.corridors {
        assert_eq!(c.assigned, LayerMask::single(LayerId(0)));
        assert_eq!(c.layers, grid.layers);
        assert_eq!(c.cells.len(), 17);
    }
    assert!(tel.stage_us(crate::api::Stage::Global) > 0);
    assert_eq!(tel.counters["global_connections"], 6);
}

/// A wall across the board with two narrow gaps; more nets than one gap
/// holds.
fn gapped_wall(nets: usize) -> RouteBoard {
    let mut b = BoardBuilder::new(40.0, 20.0, &["F.Cu", "B.Cu"]);
    let wall = KeepoutRules {
        tracks: true,
        vias: true,
        ..Default::default()
    };
    for (y0, y1) in [(0.0, 4.0), (5.2, 14.8), (16.0, 20.0)] {
        b.keepout("W", &["F.Cu", "B.Cu"], (19.0, y0, 21.0, y1), wall);
    }
    let l = b.footprint("J1", (2.0, 10.0));
    let r = b.footprint("J2", (38.0, 10.0));
    for i in 0..nets {
        let y = 4.0 + 12.0 * i as f64 / (nets.max(2) - 1) as f64;
        let net = format!("S{i:02}");
        b.smd_pad(l, &format!("{i}"), &net, (2.5, y), (0.5, 0.3), "F.Cu");
        b.smd_pad(r, &format!("{i}"), &net, (37.5, y), (0.5, 0.3), "F.Cu");
    }
    b.build()
}

#[test]
fn negotiation_spreads_nets_over_both_gaps() {
    let board = gapped_wall(8);
    let rules = UniformRules::default();
    let settings = GlobalSettings::default();
    let grid = build_grid(&board, &rules, &settings);
    let conns = connections(&board);
    let r = route_global(&board, &rules, &grid, &conns, &settings);
    assert!(r.failed.is_empty(), "{:?}", r.failed);
    assert!(r.stats.converged, "{:?}", r.stats);
    assert!(r.overflowed.is_empty());
    check_corridors(&board, &grid, &conns, &r);
    // Both gaps carry nets.
    let crossing_rows: std::collections::BTreeSet<i32> = r
        .corridors
        .iter()
        .flat_map(|c| c.cells.iter())
        .filter_map(|&id| {
            let (_, b) = grid.cell_bbox(id)?;
            (b.min.x == 19_000_000).then_some(b.min.y / 1_000_000)
        })
        .collect();
    assert!(crossing_rows.iter().any(|&y| y < 10));
    assert!(crossing_rows.iter().any(|&y| y > 10));
}

#[test]
fn an_impassable_wall_reports_no_corridor() {
    let mut b = BoardBuilder::new(20.0, 10.0, &["F.Cu"]);
    b.keepout(
        "W",
        &["F.Cu"],
        (9.0, 0.0, 11.0, 10.0),
        KeepoutRules {
            tracks: true,
            vias: true,
            ..Default::default()
        },
    );
    let f = b.footprint("J1", (5.0, 5.0));
    b.smd_pad(f, "1", "A", (3.0, 5.0), (0.5, 0.5), "F.Cu");
    b.smd_pad(f, "2", "A", (17.0, 5.0), (0.5, 0.5), "F.Cu");
    let board = b.build();
    let rules = UniformRules::default();
    let settings = GlobalSettings::default();
    let grid = build_grid(&board, &rules, &settings);
    let conns = connections(&board);
    let r = route_global(&board, &rules, &grid, &conns, &settings);
    assert!(r.corridors.is_empty());
    assert_eq!(r.failed, vec![(conns[0].id, FailureReason::NoCorridor)]);
    assert_eq!(r.stats.full_grid_searches, 1);
}

#[test]
fn class_layer_masks_confine_corridors() {
    let board = gapped_wall(6);
    let rules = UniformRules::default();
    let settings = GlobalSettings::default();
    let grid = build_grid(&board, &rules, &settings);
    let conns = connections(&board);
    let mut policy = LayerPolicy::default();
    policy
        .class_layers
        .insert(ClassId(0), LayerMask::single(LayerId(1)));
    let r = route_global_with(
        &board,
        &rules,
        &grid,
        &conns,
        &settings,
        &policy,
        &mut crate::telemetry::Telemetry::default(),
    );
    assert!(r.failed.is_empty());
    for c in &r.corridors {
        // The pads are on F.Cu, so the class layer is reached by vias, and
        // every wire runs on B.Cu.
        for w in c.cells.windows(2) {
            let (la, a) = grid.cell_bbox(w[0]).unwrap();
            let (lb, b) = grid.cell_bbox(w[1]).unwrap();
            if a != b {
                assert_eq!((la, lb), (LayerId(1), LayerId(1)));
            }
        }
    }
}

/// A pseudo-random field of two-pin nets on four layers.
fn random_nets(n: usize, seed: u64) -> RouteBoard {
    let mut b = BoardBuilder::new(60.0, 40.0, &["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]);
    let f = b.footprint("U1", (30.0, 20.0));
    let mut s = seed;
    let mut next = |m: f64| {
        s = s
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((s >> 33) as f64 / (1u64 << 31) as f64) * m
    };
    for i in 0..n {
        let net = format!("R{i:03}");
        let (x0, y0) = (2.0 + next(56.0), 2.0 + next(36.0));
        let (dx, dy) = (next(20.0) - 10.0, next(16.0) - 8.0);
        let (x1, y1) = ((x0 + dx).clamp(2.0, 58.0), (y0 + dy).clamp(2.0, 38.0));
        let layer = if i % 3 == 0 { "B.Cu" } else { "F.Cu" };
        b.smd_pad(f, &format!("{i}a"), &net, (x0, y0), (0.4, 0.4), layer);
        b.smd_pad(f, &format!("{i}b"), &net, (x1, y1), (0.4, 0.4), "F.Cu");
    }
    b.build()
}

#[test]
fn one_and_many_threads_give_identical_results() {
    let board = random_nets(300, 7);
    let rules = UniformRules::default();
    let conns = connections(&board);
    let run = |threads: u32| {
        let settings = GlobalSettings {
            threads,
            ..Default::default()
        };
        let grid = build_grid(&board, &rules, &settings);
        let r = route_global(&board, &rules, &grid, &conns, &settings);
        (grid, r)
    };
    let (g1, r1) = run(1);
    let (g4, r4) = run(4);
    assert_eq!(g1, g4);
    assert_eq!(r1, r4);
    assert_eq!(r1.corridors.len() + r1.failed.len(), conns.len());
    check_corridors(&board, &g1, &conns, &r1);
    let json1 = serde_json::to_string(&r1).unwrap();
    let json4 = serde_json::to_string(&r4).unwrap();
    assert_eq!(json1, json4);
}

#[test]
fn overflow_map_sums_per_layer() {
    // Squeeze many nets through one gap on one layer so some overflow stays.
    let mut board = gapped_wall(12);
    board.routable = LayerMask::single(LayerId(0));
    let rules = UniformRules::default();
    let settings = GlobalSettings {
        max_iterations: 3,
        ..Default::default()
    };
    let grid = build_grid(&board, &rules, &settings);
    let conns = connections(&board);
    let r = route_global(&board, &rules, &grid, &conns, &settings);
    assert!(!r.stats.converged);
    assert!(!r.overflowed.is_empty());
    let layer = r.overflow_layer(&grid, 0);
    assert_eq!(layer.len(), (grid.cols * grid.rows) as usize);
    let planar_and_via: u64 = r.overflow.iter().map(|&v| u64::from(v)).sum();
    assert_eq!(
        layer.iter().map(|&v| u64::from(v)).sum::<u64>(),
        planar_and_via
    );
    assert!(planar_and_via >= r.stats.total_overflow.min(1));
    assert_eq!(r.iterations, 3);
    assert_eq!(r.stats.per_iteration.len(), 4);
    // The result is the best iteration, not the last.
    let least = r.stats.per_iteration.iter().map(|i| i.overflow).min();
    assert_eq!(Some(r.stats.total_overflow), least);
    let best = &r.stats.per_iteration[r.stats.best_iteration as usize];
    assert_eq!(best.overflow, r.stats.total_overflow);
}

#[test]
fn settings_and_results_round_trip_through_json() {
    let s = GlobalSettings::default();
    let json = serde_json::to_string(&s).unwrap();
    assert_eq!(serde_json::from_str::<GlobalSettings>(&json).unwrap(), s);
    // Older settings without the new fields still load.
    let old = r#"{"gcell_pitch":1000000,"fine_pitch":500000,"max_iterations":30}"#;
    assert_eq!(serde_json::from_str::<GlobalSettings>(old).unwrap(), s);
    let old_result = r#"{"corridors":[],"overflowed":[],"overflow":[]}"#;
    assert_eq!(
        serde_json::from_str::<GlobalResult>(old_result).unwrap(),
        GlobalResult::default()
    );
}

#[test]
fn a_pitch_too_fine_for_the_board_is_coarsened() {
    let board = BoardBuilder::new(200.0, 10.0, &["F.Cu"]).build();
    let settings = GlobalSettings {
        gcell_pitch: 1_000,
        fine_pitch: 500,
        ..Default::default()
    };
    let grid = super::grid::lay_out(&board, &settings);
    assert!(grid.cols <= 65_000 && grid.rows <= 65_000);
    assert!(grid.pitch > 1_000);
    assert!(grid.refined.is_empty());
    assert!(i64::from(grid.cols) * i64::from(grid.pitch) >= 200_000_000);
}

/// A three-layer board joining an F.Cu pad to a B.Cu pad, optionally with a
/// wall on every layer that only a detour far outside the pattern set
/// passes.
fn top_to_bottom(wall: bool) -> RouteBoard {
    let mut b = BoardBuilder::new(40.0, 20.0, &["F.Cu", "In1.Cu", "B.Cu"]);
    if wall {
        b.keepout(
            "W",
            &["F.Cu", "In1.Cu", "B.Cu"],
            (19.0, 0.0, 21.0, 17.0),
            KeepoutRules {
                tracks: true,
                vias: true,
                ..Default::default()
            },
        );
    }
    let f = b.footprint("J1", (20.0, 10.0));
    b.smd_pad(f, "1", "A", (4.0, 10.0), (0.5, 0.5), "F.Cu");
    b.smd_pad(f, "2", "A", (36.0, 10.0), (0.5, 0.5), "B.Cu");
    b.build()
}

#[test]
fn a_class_mask_with_a_gap_routes_through_vias_across_it() {
    // The class may use F.Cu and B.Cu but not In1.Cu between them: a via
    // joins the two directly, and no corridor cell lies on In1.Cu.
    let rules = UniformRules::default();
    let settings = GlobalSettings::default();
    let outer = LayerMask::single(LayerId(0)).with(LayerId(2));
    let mut policy = LayerPolicy::default();
    policy.class_layers.insert(ClassId(0), outer);
    for wall in [false, true] {
        let board = top_to_bottom(wall);
        let grid = build_grid(&board, &rules, &settings);
        let conns = connections(&board);
        assert_eq!(conns.len(), 1);
        let r = route_global_with(
            &board,
            &rules,
            &grid,
            &conns,
            &settings,
            &policy,
            &mut crate::telemetry::Telemetry::default(),
        );
        assert!(r.failed.is_empty(), "wall {wall}: {:?}", r.failed);
        assert!(r.stats.converged);
        check_corridors(&board, &grid, &conns, &r);
        let c = &r.corridors[0];
        assert_eq!(c.layers, outer);
        assert_eq!(c.assigned, outer, "wall {wall}");
        // The wall forces the maze search; without it a pattern serves.
        assert_eq!(r.stats.pattern_seeds, u32::from(!wall));
        // One via, from F.Cu straight to B.Cu.
        assert_eq!(r.stats.vias, 1);
        let changes = c
            .cells
            .windows(2)
            .filter(|w| grid.cell_bbox(w[0]).unwrap().0 != grid.cell_bbox(w[1]).unwrap().0)
            .count();
        assert_eq!(changes, 1);
    }
}

#[test]
fn a_through_via_costs_one_via_whatever_it_spans() {
    // All layers allowed: a straight top-to-bottom connection runs on the
    // two horizontal layers and changes layer once, F.Cu to B.Cu, without
    // stepping through In1.Cu.
    let board = top_to_bottom(false);
    let rules = UniformRules::default();
    let settings = GlobalSettings::default();
    let grid = build_grid(&board, &rules, &settings);
    let conns = connections(&board);
    let r = route_global(&board, &rules, &grid, &conns, &settings);
    assert!(r.failed.is_empty());
    check_corridors(&board, &grid, &conns, &r);
    assert_eq!(r.stats.vias, 1);
    assert_eq!(
        r.corridors[0].assigned,
        LayerMask::single(LayerId(0)).with(LayerId(2))
    );
}

#[test]
fn thirty_two_copper_layers_route() {
    let mut names = vec!["F.Cu".to_string()];
    names.extend((1..=30).map(|i| format!("In{i}.Cu")));
    names.push("B.Cu".to_string());
    let names: Vec<&str> = names.iter().map(String::as_str).collect();
    let mut b = BoardBuilder::new(12.0, 6.0, &names);
    let f = b.footprint("J1", (6.0, 3.0));
    b.smd_pad(f, "1", "A", (1.5, 3.0), (0.4, 0.4), "F.Cu");
    b.smd_pad(f, "2", "A", (10.5, 3.0), (0.4, 0.4), "B.Cu");
    let board = b.build();
    assert_eq!(board.routable.count(), 32);
    let rules = UniformRules::default();
    let settings = GlobalSettings::default();
    let grid = build_grid(&board, &rules, &settings);
    assert_eq!(grid.layer_slots(), 32);
    assert_eq!(super::router::all_slots(32), u32::MAX);
    assert_eq!(super::router::all_slots(0), 0);
    assert_eq!(super::router::all_slots(3), 0b111);
    let conns = connections(&board);
    let r = route_global(&board, &rules, &grid, &conns, &settings);
    assert!(r.failed.is_empty(), "{:?}", r.failed);
    assert_eq!(r.corridors[0].layers, grid.layers);
    check_corridors(&board, &grid, &conns, &r);
}

#[test]
fn a_fine_pitch_on_a_large_board_is_bounded_in_total() {
    // 200 × 200 mm at 1 µm would be 4·10^10 cells per layer; each side
    // alone fits 65,000 cells, so only the node budget catches it.
    let board = BoardBuilder::new(200.0, 200.0, &["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]).build();
    let settings = GlobalSettings {
        gcell_pitch: 1_000,
        fine_pitch: 500,
        ..Default::default()
    };
    let grid = super::grid::lay_out(&board, &settings);
    let nodes = u64::from(grid.cols) * u64::from(grid.rows) * u64::from(grid.layer_slots());
    assert!(nodes <= super::grid::MAX_NODES, "{nodes} nodes");
    assert!(
        nodes > super::grid::MAX_NODES / 2,
        "coarsened no more than needed"
    );
    assert!(super::grid::fits(&grid));
    assert!(i64::from(grid.cols) * i64::from(grid.pitch) >= 200_000_000);
    assert!(i64::from(grid.rows) * i64::from(grid.pitch) >= 200_000_000);
}

#[test]
fn an_oversized_grid_from_elsewhere_is_refused() {
    let board = synthetic::two_pin_nets(2, false);
    let rules = UniformRules::default();
    let settings = GlobalSettings::default();
    let mut grid = build_grid(&board, &rules, &settings);
    grid.cols = 65_000;
    grid.rows = 65_000;
    let conns = connections(&board);
    let r = route_global(&board, &rules, &grid, &conns, &settings);
    assert!(r.corridors.is_empty());
    assert_eq!(r.failed.len(), conns.len());
    assert!(r
        .failed
        .iter()
        .all(|(_, f)| *f == FailureReason::NoCorridor));
    assert!(!r.stats.converged);
}

#[test]
fn no_routable_layer_reports_no_access_and_no_convergence() {
    let mut board = synthetic::two_pin_nets(2, false);
    board.routable = LayerMask::EMPTY;
    let rules = UniformRules::default();
    let settings = GlobalSettings::default();
    let grid = build_grid(&board, &rules, &settings);
    let conns = connections(&board);
    let r = route_global(&board, &rules, &grid, &conns, &settings);
    assert!(r.corridors.is_empty());
    assert_eq!(r.failed.len(), conns.len());
    for ((id, f), c) in r.failed.iter().zip(&conns) {
        assert_eq!(*id, c.id);
        assert_eq!(*f, FailureReason::NoAccess { item: c.from });
    }
    assert!(!r.stats.converged);
}

#[test]
fn a_track_region_removes_track_capacity_outside_its_box_only() {
    let mut b = BoardBuilder::new(10.0, 10.0, &["F.Cu", "B.Cu"]);
    let f = b.footprint("U1", (5.0, 5.0));
    b.smd_pad(f, "1", "A", (9.0, 9.0), (0.4, 0.4), "F.Cu");
    let board = b.build();
    let rules = UniformRules::default();
    let open = build_grid(&board, &rules, &GlobalSettings::default());
    let settings = GlobalSettings {
        track_region: Some(TrackRegion {
            layer: LayerId(1),
            bbox: BBox::new(
                Point::from_mm(3.0, 3.0).unwrap(),
                Point::from_mm(7.0, 7.0).unwrap(),
            ),
        }),
        ..Default::default()
    };
    let grid = build_grid(&board, &rules, &settings);
    assert_eq!((grid.cols, grid.rows), (open.cols, open.rows));
    let t = Topology::new(&grid);
    let edge = |a: u32, b: u32| t.edge_between(a, b).unwrap();
    let cell = |c: u32, r: u32| r * grid.cols + c;
    // Outside the box: no track capacity on B.Cu, F.Cu untouched.
    let e = edge(cell(1, 1), cell(2, 1));
    assert_eq!(grid.capacity[t.planar_res(1, e) as usize], 0);
    assert_eq!(
        grid.capacity[t.planar_res(0, e) as usize],
        open.capacity[t.planar_res(0, e) as usize]
    );
    // Inside it: as without the region.
    let e = edge(cell(4, 5), cell(5, 5));
    assert!(grid.capacity[t.planar_res(1, e) as usize] >= 2);
    assert_eq!(
        grid.capacity[t.planar_res(1, e) as usize],
        open.capacity[t.planar_res(1, e) as usize]
    );
    // Via sites everywhere are unchanged.
    for c in 0..grid.cols * grid.rows {
        assert_eq!(
            grid.capacity[t.via_res(c) as usize],
            open.capacity[t.via_res(c) as usize]
        );
    }
}

#[test]
fn track_region_outside_boxes_tile_the_rest() {
    let r = TrackRegion {
        layer: LayerId(0),
        bbox: BBox::new(Point::new(30, 40), Point::new(60, 70)),
    };
    let within = BBox::new(Point::new(0, 0), Point::new(100, 100));
    let parts = r.outside(within);
    assert_eq!(parts.len(), 4);
    let area = |b: &BBox| i64::from(b.max.x - b.min.x) * i64::from(b.max.y - b.min.y);
    let total: i64 = parts.iter().map(area).sum();
    assert_eq!(total, 100 * 100 - 30 * 30);
    for p in &parts {
        assert!(within.contains_bbox(p));
        let ix = p.max.x.min(r.bbox.max.x) - p.min.x.max(r.bbox.min.x);
        let iy = p.max.y.min(r.bbox.max.y) - p.min.y.max(r.bbox.min.y);
        assert!(ix <= 0 || iy <= 0, "{p:?} overlaps the box");
    }
    // A box reaching past `within` leaves only what is outside it.
    let wide = TrackRegion {
        layer: LayerId(0),
        bbox: BBox::new(Point::new(-10, 40), Point::new(110, 70)),
    };
    let parts = wide.outside(within);
    assert_eq!(parts.len(), 2);
    assert_eq!(parts.iter().map(area).sum::<i64>(), 100 * 100 - 100 * 30);
}
