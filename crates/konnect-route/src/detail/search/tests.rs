//! Synthetic tests for the search kernel. Every board here is made up.

use super::*;
use crate::api::{LayerId, ProfileId, TileId};
use crate::compute::job::{JOB_ALLOW_VIA, JOB_MIN_RUN};
use crate::compute::raster::{TileRaster, Window};
use crate::compute::{Batch, CpuCompute, PaintKind, PaintShape, ProfileGeometry, SearchBackend};
use konnect_geom::{BBox, Segment, Shape};

const PITCH: Coord = 50_000;

fn window(cols: u32, rows: u32) -> Window {
    Window::new(Point::new(0, 0), PITCH, cols, rows).unwrap()
}

fn stack(cols: u32, rows: u32, layers: usize) -> TileStack {
    let w = window(cols, rows);
    let mut rs = Vec::new();
    for l in 0..layers {
        let mut r = TileRaster::new(TileId(0), LayerId(l as u8), ProfileId(0), w);
        derive_masks_uniform(&mut r, None);
        rs.push(r);
    }
    TileStack {
        tile: TileId(0),
        profile: ProfileId(0),
        window: w,
        layers: rs,
    }
}

fn block(
    st: &mut TileStack,
    layer: usize,
    cols: std::ops::RangeInclusive<u32>,
    rows: std::ops::RangeInclusive<u32>,
) {
    let r = &mut st.layers[layer];
    for row in rows {
        for col in cols.clone() {
            r.core.set(col, row, 1);
            r.halo.set(col, row, 1);
        }
    }
}

fn rederive(st: &mut TileStack) {
    for r in &mut st.layers {
        derive_masks_uniform(r, None);
    }
}

fn node(col: u16, row: u16, layer: u8) -> JobNode {
    JobNode {
        col,
        row,
        layer,
        heading_mask: 0xff,
        reserved: 0,
    }
}

fn job(flags: u8) -> JobDescriptor {
    JobDescriptor {
        flags,
        cost_bound: u32::MAX,
        ..Default::default()
    }
}

fn pool_with(j: JobDescriptor, src: &[JobNode], dst: &[JobNode]) -> (JobPool, u32) {
    let mut pool = JobPool::default();
    let i = pool.push(j, src, dst, &[]);
    (pool, i)
}

fn run_ctx(ctx: &SearchContext<'_>, pool: &JobPool, i: u32) -> (SearchOutcome, SearchStats) {
    let mut stats = SearchStats::default();
    let p = CostParams::default();
    let o = search_with(ctx, pool, &pool.jobs[i as usize], i, &p, &mut stats);
    (o, stats)
}

fn found(o: &SearchOutcome) -> &PathProposalRef {
    match o {
        SearchOutcome::Found(p) => p,
        other => panic!("expected a path, got {other:?}"),
    }
}

type PathProposalRef = crate::compute::PathProposal;

/// Headings of each straight run of `nodes` (same-layer pairs), in order;
/// a via breaks the sequence with `None`.
fn runs(nodes: &[JobNode]) -> Vec<Option<u8>> {
    nodes
        .windows(2)
        .map(|w| {
            if w[0].layer != w[1].layer {
                assert_eq!((w[0].col, w[0].row), (w[1].col, w[1].row), "via moves");
                return None;
            }
            let dc = i32::from(w[1].col) - i32::from(w[0].col);
            let dr = i32::from(w[1].row) - i32::from(w[0].row);
            assert!(
                dc == 0 || dr == 0 || dc.abs() == dr.abs(),
                "not octilinear: {w:?}"
            );
            assert!(dc != 0 || dr != 0, "zero-length run: {w:?}");
            Some(heading_of(dc.signum(), dr.signum()).unwrap())
        })
        .collect()
}

/// Every turn between consecutive runs is at most `max_k · 45°`.
fn assert_turns(nodes: &[JobNode], max_k: u8) {
    let rs = runs(nodes);
    for w in rs.windows(2) {
        if let (Some(a), Some(b)) = (w[0], w[1]) {
            assert_ne!(a, b, "corners must change heading");
            let d = (i32::from(b) - i32::from(a)).rem_euclid(8) as u8;
            let k = d.min(8 - d);
            assert!(k <= max_k, "turn of {} x 45 deg in {nodes:?}", k);
        }
    }
}

fn path_len_nm(nodes: &[JobNode]) -> f64 {
    nodes
        .windows(2)
        .map(|w| {
            let dx = f64::from(i32::from(w[1].col) - i32::from(w[0].col)) * f64::from(PITCH);
            let dy = f64::from(i32::from(w[1].row) - i32::from(w[0].row)) * f64::from(PITCH);
            (dx * dx + dy * dy).sqrt()
        })
        .sum()
}

#[test]
fn turn_sets_follow_the_angle_rule() {
    let t = turn_set(None);
    for a in 0..8u8 {
        for b in 0..8u8 {
            assert_eq!(
                t[a as usize] & (1 << b) != 0,
                turn_allowed(a, b),
                "{a}->{b}"
            );
        }
    }
    assert_eq!(turn_set(Some(89.0)), t);
    let t91 = turn_set(Some(91.0));
    assert_eq!(t91[0], 0b1000_0011);
    assert_eq!(turn_set(Some(170.0))[3], 1 << 3);
    // Headings: 2 is +y Y-up, which is row - 1.
    assert_eq!(heading_delta(2), (0, -1));
    assert_eq!(heading_delta(7), (1, 1));
    assert_eq!(heading_of(-1, 1), Some(5));
}

#[test]
fn straight_and_diagonal_runs_cost_exactly_octile() {
    let st = stack(30, 30, 1);
    let cfg = SearchConfig::default();
    let ctx = SearchContext::new(&st, &cfg);
    let (pool, i) = pool_with(job(0), &[node(2, 2, 0)], &[node(12, 2, 0)]);
    let (o, _) = run_ctx(&ctx, &pool, i);
    let p = found(&o);
    assert_eq!(p.cost, Cost(10_000));
    assert_eq!(p.nodes.len(), 2);
    assert_eq!(p.nodes[0].heading_mask, 1);
    let (pool, i) = pool_with(job(0), &[node(2, 2, 0)], &[node(12, 12, 0)]);
    let (o, _) = run_ctx(&ctx, &pool, i);
    assert_eq!(found(&o).cost, Cost(14_140));
    // One 45° bend: 5 diagonal + 5 straight.
    let (pool, i) = pool_with(job(0), &[node(2, 2, 0)], &[node(12, 7, 0)]);
    let (o, _) = run_ctx(&ctx, &pool, i);
    let p = found(&o);
    assert_eq!(p.cost, Cost(5 * 1414 + 5 * 1000 + cfg.bend_45.0));
    assert_eq!(p.nodes.len(), 3);
    assert_turns(&p.nodes, 2);
}

#[test]
fn detours_around_walls_and_never_turns_135_degrees() {
    let mut st = stack(40, 40, 1);
    block(&mut st, 0, 20..=21, 0..=34);
    block(&mut st, 0, 5..=15, 10..=11);
    rederive(&mut st);
    let cfg = SearchConfig::default();
    let ctx = SearchContext::new(&st, &cfg);
    for (s, t) in [
        ((10, 5), (30, 5)),
        ((10, 20), (30, 2)),
        ((2, 2), (38, 38)),
        ((10, 12), (10, 2)),
    ] {
        let (pool, i) = pool_with(job(0), &[node(s.0, s.1, 0)], &[node(t.0, t.1, 0)]);
        let (o, _) = run_ctx(&ctx, &pool, i);
        let p = found(&o);
        assert_turns(&p.nodes, 2);
        // No corner sits on a wall, and every run avoids it.
        for w in p.nodes.windows(2) {
            let (dc, dr) = (
                (i32::from(w[1].col) - i32::from(w[0].col)).signum(),
                (i32::from(w[1].row) - i32::from(w[0].row)).signum(),
            );
            let (mut c, mut r) = (i32::from(w[0].col), i32::from(w[0].row));
            while (c, r) != (i32::from(w[1].col), i32::from(w[1].row)) {
                c += dc;
                r += dr;
                assert_eq!(
                    st.layers[0].core.get(c as u32, r as u32),
                    0,
                    "through a wall"
                );
            }
        }
        assert_eq!(p.nodes.first().map(|n| (n.col, n.row)), Some(s));
        assert_eq!(p.nodes.last().map(|n| (n.col, n.row)), Some(t));
    }
}

#[test]
fn a_91_degree_rule_allows_only_45_degree_turns() {
    let st = stack(30, 30, 1);
    let cfg = SearchConfig {
        min_track_angle_deg: Some(91.0),
        ..SearchConfig::default()
    };
    let ctx = SearchContext::new(&st, &cfg);
    // Leave east only, end up due north of the start.
    let mut src = node(10, 20, 0);
    src.heading_mask = 1;
    let (pool, i) = pool_with(job(0), &[src], &[node(10, 5, 0)]);
    let (o, _) = run_ctx(&ctx, &pool, i);
    let p = found(&o);
    assert_turns(&p.nodes, 1);
    assert_eq!(runs(&p.nodes)[0], Some(0));
}

#[test]
fn vias_cross_a_wall_only_when_allowed() {
    let mut st = stack(30, 20, 2);
    block(&mut st, 0, 15..=15, 0..=19);
    rederive(&mut st);
    let cfg = SearchConfig::default();
    let ctx = SearchContext::new(&st, &cfg);
    let (pool, i) = pool_with(job(0), &[node(5, 10, 0)], &[node(25, 10, 0)]);
    assert!(matches!(
        run_ctx(&ctx, &pool, i).0,
        SearchOutcome::Unreachable { .. }
    ));
    let (pool, i) = pool_with(job(JOB_ALLOW_VIA), &[node(5, 10, 0)], &[node(25, 10, 0)]);
    let (o, _) = run_ctx(&ctx, &pool, i);
    let p = found(&o);
    let vias = runs(&p.nodes).iter().filter(|r| r.is_none()).count();
    assert_eq!(vias, 2);
    let geo = path_geometry(&p.nodes, None, &st.window);
    assert_eq!(geo.vias.len(), 2);
    let len: i64 = geo
        .runs
        .iter()
        .map(|(_, a, b)| i64::from((b.x - a.x).abs().max((b.y - a.y).abs())))
        .sum();
    assert_eq!(len, 20 * PITCH as i64);
    let params = CostParams::default();
    // No checker: each via site is VIA_CHECK, so it also pays near_step.
    let via = params.via.0 + params.near_step.0;
    assert_eq!(p.cost, Cost(20_000 + 2 * via));
    // A layer mask without layer 1 forbids the detour.
    let mut j = job(JOB_ALLOW_VIA);
    j.layer_mask = 1;
    let (pool, i) = pool_with(j, &[node(5, 10, 0)], &[node(25, 10, 0)]);
    assert!(matches!(
        run_ctx(&ctx, &pool, i).0,
        SearchOutcome::Unreachable { .. }
    ));
}

#[test]
fn multi_source_multi_target_takes_the_cheapest_pair() {
    let st = stack(60, 20, 1);
    let cfg = SearchConfig::default();
    let ctx = SearchContext::new(&st, &cfg);
    let (pool, i) = pool_with(
        job(0),
        &[node(2, 10, 0), node(30, 10, 0)],
        &[node(55, 10, 0), node(38, 10, 0), node(10, 2, 0)],
    );
    let (o, _) = run_ctx(&ctx, &pool, i);
    let p = found(&o);
    assert_eq!(p.cost, Cost(8_000));
    assert_eq!((p.nodes[0].col, p.nodes.last().unwrap().col), (30, 38));
}

#[test]
fn bounded_regions_retry_over_the_window() {
    let mut st = stack(40, 40, 1);
    // A wall across the terminals' box that only opens far away.
    block(&mut st, 0, 20..=20, 0..=35);
    rederive(&mut st);
    let cfg = SearchConfig {
        region_margin_nm: 0,
        region_margin_permille: 0,
        ..SearchConfig::default()
    };
    let ctx = SearchContext::new(&st, &cfg);
    let (pool, i) = pool_with(job(0), &[node(10, 10, 0)], &[node(30, 12, 0)]);
    let (o, stats) = run_ctx(&ctx, &pool, i);
    assert!(found(&o).nodes.iter().any(|n| n.row > 35));
    assert_eq!(stats.region_retries, 1);
    let no_retry = SearchConfig {
        retry_full_window: false,
        ..cfg
    };
    let ctx = SearchContext::new(&st, &no_retry);
    assert!(matches!(
        run_ctx(&ctx, &pool, i).0,
        SearchOutcome::Unreachable { .. }
    ));
}

#[test]
fn budgets_and_cost_bounds_stop_the_search() {
    let st = stack(50, 50, 1);
    let cfg = SearchConfig {
        max_expansions: 10,
        ..SearchConfig::default()
    };
    let ctx = SearchContext::new(&st, &cfg);
    let (pool, i) = pool_with(job(0), &[node(1, 1, 0)], &[node(45, 30, 0)]);
    assert!(matches!(
        run_ctx(&ctx, &pool, i).0,
        SearchOutcome::BudgetExhausted { .. }
    ));
    let cfg = SearchConfig::default();
    let ctx = SearchContext::new(&st, &cfg);
    let mut j = job(0);
    j.cost_bound = 5_000;
    let (pool, i) = pool_with(j, &[node(1, 1, 0)], &[node(45, 1, 0)]);
    assert!(matches!(
        run_ctx(&ctx, &pool, i).0,
        SearchOutcome::BudgetExhausted { .. }
    ));
}

#[test]
fn minimum_runs_hold_between_turns_and_at_both_ends() {
    let mut st = stack(40, 40, 1);
    block(&mut st, 0, 10..=11, 5..=30);
    rederive(&mut st);
    let cfg = SearchConfig {
        min_run_nm: 150_000,
        ..SearchConfig::default()
    };
    let ctx = SearchContext::new(&st, &cfg);
    let (pool, i) = pool_with(job(JOB_MIN_RUN), &[node(5, 20, 0)], &[node(20, 12, 0)]);
    let (o, _) = run_ctx(&ctx, &pool, i);
    let p = found(&o);
    assert_turns(&p.nodes, 2);
    for w in p.nodes.windows(2) {
        let dc = (i32::from(w[1].col) - i32::from(w[0].col)).abs();
        let dr = (i32::from(w[1].row) - i32::from(w[0].row)).abs();
        let len =
            f64::from(dc.max(dr)) * f64::from(PITCH) * if dc == dr { 2f64.sqrt() } else { 1.0 };
        assert!(len >= 150_000.0 - 1.0, "short run {w:?}");
    }
}

#[test]
fn congestion_pushes_the_path_off_used_cells() {
    let mut st = stack(30, 20, 1);
    for col in 0..30 {
        *st.layers[0].present.get_mut(col, 10) = 3;
    }
    let cfg = SearchConfig::default();
    let ctx = SearchContext::new(&st, &cfg);
    let (pool, i) = pool_with(job(0), &[node(2, 10, 0)], &[node(27, 10, 0)]);
    let (o, _) = run_ctx(&ctx, &pool, i);
    let p = found(&o);
    // Straight along row 10 would cost 25 · 4000; leaving it is cheaper.
    assert!(p.cost.0 < 100_000);
    assert!(p.nodes.len() >= 3);
}

#[test]
fn target_heading_masks_constrain_the_last_run() {
    let st = stack(30, 30, 1);
    let cfg = SearchConfig::default();
    let ctx = SearchContext::new(&st, &cfg);
    let mut t = node(20, 10, 0);
    t.heading_mask = 1 << 2; // enter moving towards row - 1
    let (pool, i) = pool_with(job(0), &[node(5, 5, 0)], &[t]);
    let (o, _) = run_ctx(&ctx, &pool, i);
    let p = found(&o);
    assert_eq!(*runs(&p.nodes).last().unwrap(), Some(2));
    assert_turns(&p.nodes, 2);
}

struct ForbidColumn {
    x: Coord,
    calls: std::sync::atomic::AtomicU64,
}

impl ExactCheck for ForbidColumn {
    fn segment_ok(&self, _job: &JobDescriptor, _layer: u8, a: Point, b: Point) -> bool {
        self.calls
            .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        !((a.x <= self.x && b.x >= self.x) || (b.x <= self.x && a.x >= self.x))
    }
    fn via_ok(&self, _job: &JobDescriptor, _at: Point) -> bool {
        true
    }
}

#[test]
fn near_edges_ask_the_exact_check_once() {
    let mut st = stack(30, 20, 1);
    for row in 0..20 {
        st.layers[0].halo.set(15, row, 1);
    }
    rederive(&mut st);
    let cfg = SearchConfig::default();
    let chk = ForbidColumn {
        x: 15 * PITCH,
        calls: Default::default(),
    };
    let mut ctx = SearchContext::new(&st, &cfg);
    let (pool, i) = pool_with(job(0), &[node(5, 10, 0)], &[node(25, 10, 0)]);
    let (o, _) = run_ctx(&ctx, &pool, i);
    // Without a checker the Near column is crossed at near_step extra.
    assert_eq!(
        found(&o).cost,
        Cost(20_000 + 2 * CostParams::default().near_step.0)
    );
    ctx.checker = Some(&chk);
    let (o, stats) = run_ctx(&ctx, &pool, i);
    assert!(matches!(o, SearchOutcome::Unreachable { .. }));
    assert!(stats.exact_checks > 0);
    assert_eq!(
        stats.exact_checks,
        chk.calls.load(std::sync::atomic::Ordering::Relaxed)
    );
}

#[test]
fn astar_matches_dijkstra_with_both_heuristics() {
    // Pseudo-random walls on three layers with preferred directions.
    let mut st = stack(48, 40, 3);
    let mut x: u64 = 7;
    for l in 0..3 {
        for _ in 0..25 {
            x = x
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            let c = (x >> 33) as u32 % 44;
            let r = (x >> 45) as u32 % 36;
            let len = 2 + (x >> 20) as u32 % 8;
            if x & 1 == 0 {
                block(&mut st, l, c..=(c + len).min(47), r..=r);
            } else {
                block(&mut st, l, c..=c, r..=(r + len).min(39));
            }
        }
    }
    rederive(&mut st);
    for r in &mut st.layers {
        r.via_site.fill(VIA_FREE);
    }
    let params = CostParams {
        wrong_way_step: Cost(700),
        ..CostParams::default()
    };
    let cfg = SearchConfig {
        layer_pref: vec![
            Preferred::Horizontal,
            Preferred::Vertical,
            Preferred::Horizontal,
        ],
        ..SearchConfig::default()
    };
    let map_cfg = SearchConfig {
        heuristic: Heuristic::Map,
        ..cfg.clone()
    };
    let la = MapLookahead::for_window(&st.window, 3, &params, &cfg).unwrap();
    let mut checked = 0;
    let mut exp_octile = 0;
    let mut exp_map = 0;
    for k in 0..40u32 {
        x = x
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        let s = node((x >> 33) as u16 % 48, (x >> 40) as u16 % 40, (k % 3) as u8);
        let t = node(
            (x >> 17) as u16 % 48,
            (x >> 24) as u16 % 40,
            ((k / 3) % 3) as u8,
        );
        let cell = |n: &JobNode| {
            st.layers[n.layer as usize]
                .core
                .get(n.col.into(), n.row.into())
        };
        if cell(&s) > 0 || cell(&t) > 0 {
            continue;
        }
        let field = cost_field_with(&st, &[s], &params, &cfg);
        let want = field[t.layer as usize * 48 * 40 + t.row as usize * 48 + t.col as usize];
        let mut pool = JobPool::default();
        let i = pool.push(job(JOB_ALLOW_VIA), &[s], &[t], &[]);
        for (c, la, exp) in [
            (&cfg, None, &mut exp_octile),
            (&map_cfg, Some(&la), &mut exp_map),
        ] {
            let ctx = SearchContext {
                lookahead: la,
                ..SearchContext::new(&st, c)
            };
            let mut stats = SearchStats::default();
            let o = search_with(&ctx, &pool, &pool.jobs[0], i, &params, &mut stats);
            *exp += stats.expansions;
            match o {
                SearchOutcome::Found(p) => assert_eq!(p.cost, want, "{s:?} -> {t:?}"),
                SearchOutcome::Unreachable { .. } => assert_eq!(want, Cost::INFINITE),
                other => panic!("{other:?}"),
            }
        }
        checked += 1;
    }
    assert!(checked >= 20);
    assert!(
        exp_map <= exp_octile,
        "map {exp_map} vs octile {exp_octile}"
    );
}

/// Dijkstra over the full window with `cfg` (the frozen `cost_field` uses
/// the default config).
fn cost_field_with(
    st: &TileStack,
    sources: &[JobNode],
    params: &CostParams,
    cfg: &SearchConfig,
) -> Vec<Cost> {
    let j = job(JOB_ALLOW_VIA);
    let p = Problem {
        stack: st,
        job: &j,
        job_index: 0,
        sources,
        targets: &[],
        corridor: &[],
        params,
        config: &SearchConfig {
            max_expansions: u64::MAX,
            ..cfg.clone()
        },
        checker: None,
        lookahead: None,
        gcells: None,
        mode: Mode::Field,
    };
    let g = UniformGrid::new(st.window);
    let region = Region::full(&g);
    let mut s = Scratch::default();
    let mut stats = SearchStats::default();
    kernel::run(&g, &p, region, &mut s, &mut stats);
    let mut out = Vec::new();
    kernel::field_costs(&s, region, st.layers.len() as u32, 1, &mut out);
    out
}

#[test]
fn cpu_cost_field_is_dijkstra_from_the_sources() {
    let st = stack(10, 8, 2);
    let mut cpu = CpuCompute::new();
    let f = cpu.cost_field(&st, &[node(2, 3, 0)], &CostParams::default());
    assert_eq!(f.layer_count, 2);
    assert_eq!(f.costs.len(), 160);
    assert_eq!(f.costs[3 * 10 + 2], Cost(0));
    assert_eq!(f.costs[3 * 10 + 7], Cost(5_000));
    assert_eq!(f.costs[0], Cost(2 * 1414 + 1000 + 50));
    // Layer 1 through one via (VIA_CHECK without a checker: + near_step).
    let p = CostParams::default();
    assert_eq!(f.costs[80 + 3 * 10 + 2], Cost(p.via.0 + p.near_step.0));
    let empty = cpu.cost_field(&st, &[], &p);
    assert!(empty.costs.iter().all(|c| c.is_infinite()));
}

#[test]
fn batches_are_deterministic_and_match_single_searches() {
    let mut st = stack(60, 50, 2);
    block(&mut st, 0, 30..=31, 0..=40);
    block(&mut st, 1, 10..=50, 25..=25);
    rederive(&mut st);
    let stacks = vec![st.clone(), st];
    let mut pool = JobPool::default();
    let mut x: u64 = 99;
    for k in 0..64u32 {
        x = x
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        let s = node((x >> 33) as u16 % 28, (x >> 40) as u16 % 50, 0);
        let t = node(
            33 + (x >> 17) as u16 % 27,
            (x >> 24) as u16 % 50,
            (k % 2) as u8,
        );
        let mut j = job(JOB_ALLOW_VIA);
        j.window = k % 2;
        j.connection = k;
        pool.push(j, &[s], &[t], &[]);
    }
    let batch = Batch {
        index: 0,
        jobs: (0..64).collect(),
    };
    let params = CostParams::default();
    let mut cpu = CpuCompute::new();
    let a = cpu.search_batch(&batch, &pool, &stacks, &params);
    let b = cpu.search_batch(&batch, &pool, &stacks, &params);
    assert_eq!(a, b);
    assert_eq!(a.len(), 64);
    let single = rayon::ThreadPoolBuilder::new()
        .num_threads(1)
        .build()
        .unwrap();
    let c = single.install(|| cpu.search_batch(&batch, &pool, &stacks, &params));
    assert_eq!(a, c);
    for (k, o) in a.iter().enumerate() {
        let mut stats = SearchStats::default();
        let job = pool.jobs[k];
        let want = search(
            &stacks[job.window as usize],
            &pool,
            &job,
            k as u32,
            &params,
            &mut stats,
        );
        assert_eq!(*o, want);
        assert!(
            matches!(o, SearchOutcome::Found(_)),
            "job {k}: {o:?} {:?} {:?}",
            pool.sources(&job),
            pool.targets(&job)
        );
    }
}

#[test]
fn non_uniform_lattices_hit_forced_lines_exactly() {
    // Pads at off-grid centres: x = 1.013 mm and 3.987 mm, y = 0.517 mm.
    let mut spec = LatticeSpec::new(BBox::new(
        Point::new(0, 0),
        Point::new(5_000_000, 2_000_000),
    ));
    spec.force_point(Point::new(1_013_000, 517_000));
    spec.force_point(Point::new(3_987_000, 517_000));
    spec.force_channel_midlines(&[2_100_000, 2_900_000], &[], 1_000_000);
    spec.force_box_edges(
        BBox::new(
            Point::new(4_400_000, 1_400_000),
            Point::new(4_500_000, 1_500_000),
        ),
        133_000,
    );
    assert!(spec.forced_x.contains(&2_500_000));
    assert!(spec.forced_x.contains(&4_267_000) && spec.forced_y.contains(&1_633_000));
    spec.fine_regions = vec![BBox::new(
        Point::new(2_000_000, 0),
        Point::new(3_000_000, 2_000_000),
    )];
    let lat = Lattice::build(&spec).unwrap();
    let w = lat.window();
    let mut rs = Vec::new();
    for l in 0..2 {
        rs.push(TileRaster::new(TileId(0), LayerId(l), ProfileId(0), w));
    }
    let mut st = TileStack {
        tile: TileId(0),
        profile: ProfileId(0),
        window: w,
        layers: rs,
    };
    // A track obstacle across the straight line, painted at lattice positions.
    let obstacle = PaintShape {
        shape: Shape::Segment(Segment::new(
            Point::new(2_500_000, 0),
            Point::new(2_500_000, 1_200_000),
            100_000,
        )),
        clearance: 100_000,
        kind: PaintKind::Hard,
    };
    let profile = ProfileGeometry {
        track_width: 100_000,
        sagitta: 0,
    };
    for r in &mut st.layers {
        paint_on_lattice(r, &lat, std::slice::from_ref(&obstacle), &profile, true);
        derive_masks_on_lattice(r, &lat, None);
    }
    let a = lat.nearest(Point::new(1_013_000, 517_000)).unwrap();
    let b = lat.nearest(Point::new(3_987_000, 517_000)).unwrap();
    assert_eq!(lat.node(a.0, a.1), Point::new(1_013_000, 517_000));
    let cfg = SearchConfig::default();
    let ctx = SearchContext {
        lattice: Some(&lat),
        ..SearchContext::new(&st, &cfg)
    };
    let (pool, i) = pool_with(
        job(0),
        &[node(a.0 as u16, a.1 as u16, 0)],
        &[node(b.0 as u16, b.1 as u16, 0)],
    );
    let (o, _) = run_ctx(&ctx, &pool, i);
    let p = found(&o);
    let pts: Vec<Point> = p
        .nodes
        .iter()
        .map(|n| lat.node(n.col.into(), n.row.into()))
        .collect();
    assert_eq!(pts[0], Point::new(1_013_000, 517_000));
    assert_eq!(*pts.last().unwrap(), Point::new(3_987_000, 517_000));
    for w in pts.windows(2) {
        let dx = (i64::from(w[1].x) - i64::from(w[0].x)).abs();
        let dy = (i64::from(w[1].y) - i64::from(w[0].y)).abs();
        assert!(
            dx == 0 || dy == 0 || dx == dy,
            "not octilinear in nm: {w:?}"
        );
        // Every run clears the obstacle by its clearance.
        let track = Shape::Segment(Segment::new(w[0], w[1], 100_000));
        assert_ne!(
            konnect_geom::gap_cmp(&obstacle.shape, &track, 100_000),
            std::cmp::Ordering::Less
        );
    }
    let len: f64 = pts
        .windows(2)
        .map(|w| {
            ((f64::from(w[1].x - w[0].x)).powi(2) + (f64::from(w[1].y - w[0].y)).powi(2)).sqrt()
        })
        .sum();
    assert!(len < 2.974e6 * 1.6, "length {len}");
    let _ = path_len_nm(&p.nodes);
}

#[test]
fn corridors_steer_the_search() {
    use crate::api::LayerMask;
    let st = stack(80, 80, 1);
    // 1 mm GCells over the 4 mm window: 4 × 4 on one layer.
    let gc = GCellGrid {
        bbox: BBox::new(Point::new(0, 0), Point::new(4_000_000, 4_000_000)),
        pitch: 1_000_000,
        cols: 4,
        rows: 4,
        layers: LayerMask(1),
    };
    // A corridor down column 0 then along row 3: an L, not the diagonal.
    let cells: Vec<u32> = vec![
        gc.id(0, 0, 0),
        gc.id(0, 0, 1),
        gc.id(0, 0, 2),
        gc.id(0, 0, 3),
        gc.id(0, 1, 3),
        gc.id(0, 2, 3),
        gc.id(0, 3, 3),
    ];
    let cfg = SearchConfig {
        corridor_penalty: Cost(2_000),
        ..SearchConfig::default()
    };
    let mut pool = JobPool::default();
    let i = pool.push(job(0), &[node(10, 10, 0)], &[node(70, 70, 0)], &cells);
    let free = SearchContext::new(&st, &cfg);
    let (o, _) = run_ctx(&free, &pool, i);
    assert_eq!(found(&o).nodes.len(), 2);
    let ctx = SearchContext {
        gcells: Some(&gc),
        ..SearchContext::new(&st, &cfg)
    };
    let (o, _) = run_ctx(&ctx, &pool, i);
    let p = found(&o);
    assert!(p.nodes.len() >= 3);
    assert!(p.nodes.iter().all(|n| n.col < 20 || n.row >= 60));
}

#[test]
fn warm_searches_do_not_grow_scratch() {
    let mut st = stack(60, 60, 2);
    block(&mut st, 0, 30..=31, 0..=50);
    rederive(&mut st);
    let cfg = SearchConfig::default();
    let ctx = SearchContext::new(&st, &cfg);
    let (pool, i) = pool_with(job(JOB_ALLOW_VIA), &[node(5, 5, 0)], &[node(55, 40, 0)]);
    let _ = run_ctx(&ctx, &pool, i);
    let before = SCRATCH.with(|s| s.borrow().footprint());
    for _ in 0..5 {
        let _ = run_ctx(&ctx, &pool, i);
    }
    let after = SCRATCH.with(|s| s.borrow().footprint());
    assert_eq!(before, after);
    assert!(before > 0);
}

#[test]
fn bend_costs_and_the_bend_bound_keep_open_searches_narrow() {
    // On an empty lattice every octilinear path with one bend is optimal;
    // without a bend-aware bound A* expands the whole parallelogram.
    let st = stack(220, 220, 3);
    let cfg = SearchConfig::default();
    let ctx = SearchContext::new(&st, &cfg);
    for (t, cost) in [
        ((190u16, 100u16), 90 * 1414 + 90 * 1000),
        ((60, 200), 50 * 1414 + 140 * 1000),
    ] {
        let (pool, i) = pool_with(job(JOB_ALLOW_VIA), &[node(10, 10, 0)], &[node(t.0, t.1, 0)]);
        let (o, stats) = run_ctx(&ctx, &pool, i);
        let p = found(&o);
        assert_eq!(p.cost, Cost(cost + cfg.bend_45.0), "{t:?}");
        assert_eq!(p.nodes.len(), 3);
        assert!(
            stats.expansions < 1_000,
            "{t:?}: {} expansions",
            stats.expansions
        );
    }
    // Without bend costs the plateau is real and only the optimum matters.
    let flat = SearchConfig {
        bend_45: Cost(0),
        bend_90: Cost(0),
        ..SearchConfig::default()
    };
    let ctx = SearchContext::new(&st, &flat);
    let (pool, i) = pool_with(job(JOB_ALLOW_VIA), &[node(10, 10, 0)], &[node(190, 100, 0)]);
    assert_eq!(
        found(&run_ctx(&ctx, &pool, i).0).cost,
        Cost(90 * 1414 + 90 * 1000)
    );
}
