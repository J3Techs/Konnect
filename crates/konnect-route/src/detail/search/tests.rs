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
        spent: 0,
        cap: u64::MAX,
        use_field: false,
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
        fine_pitch: 1_000_000,
        refined: Vec::new(),
        capacity: Vec::new(),
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

#[test]
fn sealed_targets_are_found_unreachable_without_flooding_the_window() {
    // The target sits in a closed ring; the source side is wide open.
    let mut st = stack(200, 200, 2);
    for l in 0..2 {
        block(&mut st, l, 150..=160, 150..=150);
        block(&mut st, l, 150..=160, 160..=160);
        block(&mut st, l, 150..=150, 150..=160);
        block(&mut st, l, 160..=160, 150..=160);
    }
    rederive(&mut st);
    let (pool, i) = pool_with(job(JOB_ALLOW_VIA), &[node(10, 10, 0)], &[node(155, 155, 0)]);
    let cfg = SearchConfig {
        dead_end_check_after: 2_000,
        ..SearchConfig::default()
    };
    let ctx = SearchContext::new(&st, &cfg);
    let (o, stats) = run_ctx(&ctx, &pool, i);
    assert!(
        matches!(o, SearchOutcome::Unreachable { expansions, .. } if expansions <= 4_000),
        "{o:?}"
    );
    // One proof per attempt (here the bounded region is the whole window).
    assert_eq!(stats.dead_ends, stats.region_retries + 1);
    // Without the check the same search exhausts the window.
    let off = SearchConfig {
        dead_end_check_after: 0,
        ..SearchConfig::default()
    };
    let ctx = SearchContext::new(&st, &off);
    let (o, _) = run_ctx(&ctx, &pool, i);
    assert!(
        matches!(o, SearchOutcome::Unreachable { expansions, .. } if expansions > 100_000),
        "{o:?}"
    );
    // An open target is never declared sealed.
    let (pool, i) = pool_with(job(JOB_ALLOW_VIA), &[node(10, 10, 0)], &[node(120, 190, 1)]);
    let ctx = SearchContext::new(&st, &cfg);
    assert!(matches!(run_ctx(&ctx, &pool, i).0, SearchOutcome::Found(_)));
}

#[test]
fn infinite_cost_terms_forbid_instead_of_wrapping() {
    // A Near column across the window: crossing it costs near_step.
    let mut st = stack(30, 20, 1);
    for row in 0..20 {
        st.layers[0].halo.set(15, row, 1);
    }
    rederive(&mut st);
    let cfg = SearchConfig::default();
    let ctx = SearchContext::new(&st, &cfg);
    let (pool, i) = pool_with(job(0), &[node(5, 10, 0)], &[node(25, 10, 0)]);
    let params = CostParams {
        near_step: Cost::INFINITE,
        ..CostParams::default()
    };
    let mut stats = SearchStats::default();
    let o = search_with(&ctx, &pool, &pool.jobs[i as usize], i, &params, &mut stats);
    assert!(matches!(o, SearchOutcome::Unreachable { .. }), "{o:?}");

    // Infinite bends: only a straight run reaches the target.
    let st = stack(30, 30, 1);
    let cfg = SearchConfig {
        bend_45: Cost::INFINITE,
        bend_90: Cost::INFINITE,
        ..SearchConfig::default()
    };
    let ctx = SearchContext::new(&st, &cfg);
    let (pool, i) = pool_with(job(0), &[node(5, 5, 0)], &[node(20, 5, 0)]);
    let (o, _) = run_ctx(&ctx, &pool, i);
    assert_eq!(found(&o).cost, Cost(15_000));
    let (pool, i) = pool_with(job(0), &[node(5, 5, 0)], &[node(20, 9, 0)]);
    let (o, _) = run_ctx(&ctx, &pool, i);
    assert!(matches!(o, SearchOutcome::Unreachable { .. }), "{o:?}");

    // An infinite via cost forbids the via; huge wrong-way and congestion
    // terms saturate rather than wrap.
    let mut st = stack(30, 20, 2);
    block(&mut st, 0, 15..=15, 0..=19);
    rederive(&mut st);
    for col in 0..30 {
        *st.layers[1].present.get_mut(col, 10) = u16::MAX;
        *st.layers[1].history.get_mut(col, 10) = u16::MAX;
    }
    let cfg = SearchConfig {
        layer_pref: vec![Preferred::Vertical, Preferred::Vertical],
        ..SearchConfig::default()
    };
    let ctx = SearchContext::new(&st, &cfg);
    let (pool, i) = pool_with(job(JOB_ALLOW_VIA), &[node(5, 10, 0)], &[node(25, 10, 0)]);
    let params = CostParams {
        via: Cost::INFINITE,
        ..CostParams::default()
    };
    let mut stats = SearchStats::default();
    let o = search_with(&ctx, &pool, &pool.jobs[i as usize], i, &params, &mut stats);
    assert!(matches!(o, SearchOutcome::Unreachable { .. }), "{o:?}");
    let params = CostParams {
        wrong_way_step: Cost(u32::MAX - 1),
        present_factor: u32::MAX,
        history_factor: u32::MAX,
        ..CostParams::default()
    };
    let mut stats = SearchStats::default();
    let o = search_with(&ctx, &pool, &pool.jobs[i as usize], i, &params, &mut stats);
    // Every crossing pays wrong-way steps on layer 1 (vertical preferred):
    // a saturated, not a wrapped, cost.
    if let SearchOutcome::Found(p) = &o {
        assert!(p.cost.0 > 1_000_000_000, "{:?}", p.cost);
    }
}

#[test]
fn minimum_runs_beyond_the_run_field_are_refused_not_shortened() {
    let st = stack(60, 60, 1);
    let (pool, i) = pool_with(job(JOB_MIN_RUN), &[node(5, 5, 0)], &[node(50, 40, 0)]);
    // 31 half pitches (775 µm) is the most the run field counts.
    let cfg = SearchConfig {
        min_run_nm: 775_000,
        ..SearchConfig::default()
    };
    let ctx = SearchContext::new(&st, &cfg);
    let (o, stats) = run_ctx(&ctx, &pool, i);
    let p = found(&o);
    assert_eq!(stats.min_run_rejects, 0);
    for w in p.nodes.windows(2) {
        let dc = (i32::from(w[1].col) - i32::from(w[0].col)).abs();
        let dr = (i32::from(w[1].row) - i32::from(w[0].row)).abs();
        let len =
            f64::from(dc.max(dr)) * f64::from(PITCH) * if dc == dr { 2f64.sqrt() } else { 1.0 };
        assert!(len >= 775_000.0 - 1.0, "short run {w:?}");
    }
    // One more half pitch does not fit: refused, not silently clamped.
    let cfg = SearchConfig {
        min_run_nm: 800_000,
        ..SearchConfig::default()
    };
    let ctx = SearchContext::new(&st, &cfg);
    let (o, stats) = run_ctx(&ctx, &pool, i);
    assert!(matches!(o, SearchOutcome::Unreachable { .. }), "{o:?}");
    assert_eq!(stats.min_run_rejects, 1);
    // Jobs without the flag ignore the minimum.
    let (pool, i) = pool_with(job(0), &[node(5, 5, 0)], &[node(50, 40, 0)]);
    let (o, _) = run_ctx(&ctx, &pool, i);
    found(&o);
}

#[test]
fn minimum_run_jobs_may_end_on_a_via_at_the_target() {
    // The target sits on layer 1 directly under a straight run on layer 0.
    let st = stack(40, 20, 2);
    let cfg = SearchConfig {
        min_run_nm: 300_000,
        ..SearchConfig::default()
    };
    let ctx = SearchContext::new(&st, &cfg);
    let mut t = node(20, 10, 1);
    // No track may enter the target on layer 1: only a via lands there.
    t.heading_mask = 0;
    let (pool, i) = pool_with(job(JOB_MIN_RUN | JOB_ALLOW_VIA), &[node(5, 10, 0)], &[t]);
    let (o, _) = run_ctx(&ctx, &pool, i);
    let p = found(&o);
    let last = p.nodes[p.nodes.len() - 1];
    let before = p.nodes[p.nodes.len() - 2];
    assert_eq!((last.col, last.row, last.layer), (20, 10, 1));
    assert_eq!((before.col, before.row, before.layer), (20, 10, 0));
}

#[test]
fn the_map_table_survives_negotiation_factor_changes() {
    let st = stack(20, 20, 2);
    let params = CostParams::default();
    let cfg = SearchConfig {
        heuristic: Heuristic::Map,
        ..SearchConfig::default()
    };
    let la = MapLookahead::for_window(&st.window, 2, &params, &cfg).unwrap();
    let later = CostParams {
        present_factor: params.present_factor * 3,
        history_factor: params.history_factor + 500,
        history_increment: 4,
        near_step: Cost(900),
        ..params
    };
    assert!(la.matches(&later, &cfg, PITCH, 2));
    let other_via = CostParams {
        via: Cost(params.via.0 + 1),
        ..params
    };
    assert!(!la.matches(&other_via, &cfg, PITCH, 2));
    let other_wrong = CostParams {
        wrong_way_step: Cost(params.wrong_way_step.0 + 1),
        ..params
    };
    assert!(!la.matches(&other_wrong, &cfg, PITCH, 2));
}

#[test]
fn search_stats_read_telemetry_written_before_new_counters() {
    let s: SearchStats =
        serde_json::from_str(r#"{"expansions":7,"exact_checks":2,"exact_check_cache_hits":1}"#)
            .unwrap();
    assert_eq!(s.expansions, 7);
    assert_eq!(s.min_run_rejects, 0);
}

#[test]
fn via_congestion_prices_vias_by_the_routed_copper_around_them() {
    // A wall on layer 0 forces two vias. Routed copper of other nets sits
    // on layer 0 five rows above and below the straight line: tracks along
    // row 10 never touch it, but a via on row 10 samples it.
    let mut st = stack(30, 21, 2);
    block(&mut st, 0, 15..=15, 0..=20);
    for col in 0..30 {
        *st.layers[0].present.get_mut(col, 5) = 5;
        *st.layers[0].present.get_mut(col, 15) = 5;
    }
    rederive(&mut st);
    let params = CostParams::default();
    let via = params.via.0 + params.near_step.0;
    let (pool, i) = pool_with(job(JOB_ALLOW_VIA), &[node(5, 10, 0)], &[node(25, 10, 0)]);
    let blind = SearchConfig::default();
    let (o, _) = run_ctx(&SearchContext::new(&st, &blind), &pool, i);
    assert_eq!(found(&o).cost, Cost(20_000 + 2 * via), "off by default");
    let priced = SearchConfig {
        via_congestion_reach_nm: Some(5 * PITCH),
        ..SearchConfig::default()
    };
    let (o, _) = run_ctx(&SearchContext::new(&st, &priced), &pool, i);
    let p = found(&o);
    let geo = path_geometry(&p.nodes, None, &st.window);
    assert_eq!(geo.vias.len(), 2);
    // Both vias moved off row 10, where every via would pay the congestion.
    assert!(
        geo.vias.iter().all(|&(_, _, at)| at.y != 10 * PITCH),
        "{geo:?}"
    );
    assert!(p.cost.0 > 20_000 + 2 * via, "the detour costs something");
    let congested_via =
        u64::from(params.via.0) * (1000 + 5 * u64::from(params.present_factor)) / 1000;
    assert!(u64::from(p.cost.0) < 20_000 + 2 * congested_via);
}

#[test]
fn the_relaxed_field_keeps_path_costs_and_cuts_floods() {
    // Walls, routed copper (present), history and HALO bands on three
    // layers, with priced vias: the field restart must find paths of the
    // same cost (or the same failure) as plain A*, with fewer expansions.
    let (cols, rows) = (64u32, 48u32);
    let mut st = stack(cols, rows, 3);
    let mut x: u64 = 11;
    let mut next = || {
        x = x
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        x
    };
    for l in 0..3 {
        for _ in 0..30 {
            let v = next();
            let c = (v >> 33) as u32 % (cols - 4);
            let r = (v >> 45) as u32 % (rows - 4);
            let len = 2 + (v >> 20) as u32 % 10;
            if v & 1 == 0 {
                block(&mut st, l, c..=(c + len).min(cols - 1), r..=r);
            } else {
                block(&mut st, l, c..=c, r..=(r + len).min(rows - 1));
            }
        }
    }
    rederive(&mut st);
    for (l, r) in st.layers.iter_mut().enumerate() {
        for row in 0..rows {
            for col in 0..cols {
                let v = next();
                if r.core.get(col, row) == 0 && v % 5 == 0 {
                    r.halo.set(col, row, 1);
                }
                // Long lines of routed copper and some history.
                if (l == 0 && row % 9 == 4) || (l == 1 && col % 11 == 5) || v % 13 == 0 {
                    r.present.set(col, row, 1 + (v >> 40) as u16 % 3);
                }
                if v % 17 == 0 {
                    r.history.set(col, row, 1 + (v >> 50) as u16 % 4);
                }
                r.via_site.set(
                    col,
                    row,
                    match v % 7 {
                        0 => VIA_NO,
                        1 => VIA_CHECK,
                        _ => VIA_FREE,
                    },
                );
            }
        }
    }
    let params = CostParams {
        present_factor: 4_000,
        history_factor: 1_000,
        ..CostParams::default()
    };
    let plain = SearchConfig {
        use_heading_mask: false,
        via_congestion_reach_nm: Some(100_000),
        ..SearchConfig::default()
    };
    let with_field = SearchConfig {
        field_after: 1,
        ..plain.clone()
    };
    let (mut exp_plain, mut exp_field, mut checked, mut field_runs) = (0, 0, 0, 0);
    for k in 0..60u32 {
        let v = next();
        let s = node((v >> 33) as u16 % 64, (v >> 40) as u16 % 48, (k % 3) as u8);
        let t = node(
            (v >> 17) as u16 % 64,
            (v >> 24) as u16 % 48,
            ((k / 3) % 3) as u8,
        );
        let mut pool = JobPool::default();
        let i = pool.push(job(JOB_ALLOW_VIA), &[s], &[t], &[]);
        let mut outs = Vec::new();
        for (cfg, exp) in [(&plain, &mut exp_plain), (&with_field, &mut exp_field)] {
            let ctx = SearchContext::new(&st, cfg);
            let mut stats = SearchStats::default();
            let o = search_with(&ctx, &pool, &pool.jobs[0], i, &params, &mut stats);
            *exp += stats.expansions;
            field_runs += stats.field_runs;
            outs.push(o);
        }
        match (&outs[0], &outs[1]) {
            (SearchOutcome::Found(a), SearchOutcome::Found(b)) => {
                assert_eq!(a.cost, b.cost, "{s:?} -> {t:?}");
                // The field path is a real path: it ends where asked.
                assert_eq!(
                    b.nodes.first().map(|n| (n.col, n.row)),
                    Some((s.col, s.row))
                );
                assert_eq!(b.nodes.last().map(|n| (n.col, n.row)), Some((t.col, t.row)));
            }
            (SearchOutcome::Unreachable { .. }, SearchOutcome::Unreachable { .. }) => {}
            (a, b) => panic!("{s:?} -> {t:?}: {a:?} vs {b:?}"),
        }
        checked += 1;
    }
    assert_eq!(checked, 60);
    assert!(field_runs > 30, "{field_runs} field runs");
    assert!(
        exp_field < exp_plain,
        "field {exp_field} vs plain {exp_plain}"
    );
}

#[test]
fn the_relaxed_field_proves_sealed_targets_unreachable() {
    let mut st = stack(60, 60, 2);
    // A ring around the target on both layers, with no via sites inside.
    for l in 0..2 {
        block(&mut st, l, 20..=40, 20..=20);
        block(&mut st, l, 20..=40, 40..=40);
        block(&mut st, l, 20..=20, 20..=40);
        block(&mut st, l, 40..=40, 20..=40);
    }
    rederive(&mut st);
    let cfg = SearchConfig {
        field_after: 1,
        dead_end_check_after: 0,
        ..SearchConfig::default()
    };
    let ctx = SearchContext::new(&st, &cfg);
    let (pool, i) = pool_with(job(JOB_ALLOW_VIA), &[node(2, 2, 0)], &[node(30, 30, 1)]);
    let (o, stats) = run_ctx(&ctx, &pool, i);
    assert!(matches!(o, SearchOutcome::Unreachable { .. }), "{o:?}");
    assert_eq!(stats.field_runs, 1);
    // The field ran dry inside the ring (and on it, plus the target's
    // pass-through state); the forward search never started.
    assert!(
        stats.field_settled <= 2 * 21 * 21 + 1,
        "{}",
        stats.field_settled
    );
    assert!(stats.expansions <= 2, "{}", stats.expansions);
}

/// A small deterministic generator for the field comparisons.
struct Lcg(u64);

impl Lcg {
    fn next(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        self.0
    }

    /// One to three headings.
    fn mask(&mut self) -> u8 {
        let v = self.next();
        let mut m = 1u8 << ((v >> 33) % 8);
        if v & 3 != 0 {
            m |= 1 << ((v >> 40) % 8);
        }
        if v & 3 == 3 {
            m |= 1 << ((v >> 47) % 8);
        }
        m
    }
}

/// Walls, routed copper, history, HALO and via sites on every layer.
fn rough_board(st: &mut TileStack, g: &mut Lcg, walls: u32) {
    let (cols, rows) = (st.window.cols, st.window.rows);
    for l in 0..st.layers.len() {
        for _ in 0..walls {
            let v = g.next();
            let c = (v >> 33) as u32 % (cols - 4);
            let r = (v >> 45) as u32 % (rows - 4);
            let len = 2 + (v >> 20) as u32 % 8;
            if v & 1 == 0 {
                block(st, l, c..=(c + len).min(cols - 1), r..=r);
            } else {
                block(st, l, c..=c, r..=(r + len).min(rows - 1));
            }
        }
    }
    for (l, r) in st.layers.iter_mut().enumerate() {
        for row in 0..rows {
            for col in 0..cols {
                let v = g.next();
                if r.core.get(col, row) == 0 && v.is_multiple_of(5) {
                    r.halo.set(col, row, 1);
                }
                if (l == 0 && row % 7 == 3) || (l == 1 && col % 9 == 4) || v.is_multiple_of(11) {
                    r.present.set(col, row, 1 + (v >> 40) as u16 % 3);
                }
                if v.is_multiple_of(17) {
                    r.history.set(col, row, 1 + (v >> 50) as u16 % 4);
                }
                r.via_site.set(
                    col,
                    row,
                    match v % 7 {
                        0 => VIA_NO,
                        1 => VIA_CHECK,
                        _ => VIA_FREE,
                    },
                );
            }
        }
    }
}

/// Run every job with and without the field from the first expansion and
/// require the same cost (or the same failure). Returns the expansions
/// without and with the field and the field runs.
fn field_matches_plain(
    ctx_plain: &SearchContext<'_>,
    ctx_field: &SearchContext<'_>,
    params: &CostParams,
    jobs: &[(u8, JobNode, JobNode)],
) -> (u64, u64, u64) {
    let (mut exp_plain, mut exp_field, mut field_runs) = (0, 0, 0);
    for &(flags, s, t) in jobs {
        let mut pool = JobPool::default();
        let i = pool.push(job(flags), &[s], &[t], &[]);
        let mut outs = Vec::new();
        for (ctx, exp) in [(ctx_plain, &mut exp_plain), (ctx_field, &mut exp_field)] {
            let mut stats = SearchStats::default();
            let o = search_with(ctx, &pool, &pool.jobs[0], i, params, &mut stats);
            *exp += stats.expansions;
            field_runs += stats.field_runs;
            outs.push(o);
        }
        match (&outs[0], &outs[1]) {
            (SearchOutcome::Found(a), SearchOutcome::Found(b)) => {
                assert_eq!(a.cost, b.cost, "{flags:#x} {s:?} -> {t:?}");
                assert_eq!(
                    b.nodes.first().map(|n| (n.col, n.row)),
                    Some((s.col, s.row))
                );
                assert_eq!(b.nodes.last().map(|n| (n.col, n.row)), Some((t.col, t.row)));
            }
            (SearchOutcome::Unreachable { .. }, SearchOutcome::Unreachable { .. }) => {}
            (a, b) => panic!("{flags:#x} {s:?} -> {t:?}: {a:?} vs {b:?}"),
        }
    }
    (exp_plain, exp_field, field_runs)
}

#[test]
fn the_relaxed_field_keeps_path_costs_at_masked_targets_and_minimum_runs() {
    // Targets entered only in one to three headings (pin access), sources
    // left only in some, the derived heading mask on, and minimum-run jobs:
    // the field's bound at a target must not undercut the cost of passing
    // through it in a heading its mask refuses.
    let (cols, rows) = (56u32, 44u32);
    let mut st = stack(cols, rows, 3);
    let mut g = Lcg(23);
    rough_board(&mut st, &mut g, 26);
    rederive(&mut st);
    let params = CostParams {
        present_factor: 4_000,
        history_factor: 1_000,
        ..CostParams::default()
    };
    for (use_heading_mask, flags) in [
        (false, JOB_ALLOW_VIA),
        (true, JOB_ALLOW_VIA),
        (true, JOB_ALLOW_VIA | JOB_MIN_RUN),
        (false, JOB_MIN_RUN),
    ] {
        let plain = SearchConfig {
            use_heading_mask,
            min_run_nm: 100_000,
            via_congestion_reach_nm: Some(100_000),
            ..SearchConfig::default()
        };
        let with_field = SearchConfig {
            field_after: 1,
            ..plain.clone()
        };
        let jobs: Vec<(u8, JobNode, JobNode)> = (0..80u32)
            .map(|k| {
                let v = g.next();
                let mut s = node(
                    (v >> 33) as u16 % cols as u16,
                    (v >> 40) as u16 % rows as u16,
                    (k % 3) as u8,
                );
                if k % 4 == 0 {
                    s.heading_mask = g.mask();
                }
                let mut t = node(
                    (v >> 17) as u16 % cols as u16,
                    (v >> 24) as u16 % rows as u16,
                    ((k / 3) % 3) as u8,
                );
                t.heading_mask = g.mask();
                (flags, s, t)
            })
            .collect();
        let ctx_plain = SearchContext::new(&st, &plain);
        let ctx_field = SearchContext::new(&st, &with_field);
        let (exp_plain, exp_field, runs) =
            field_matches_plain(&ctx_plain, &ctx_field, &params, &jobs);
        assert!(
            runs > 40,
            "{use_heading_mask} {flags:#x}: {runs} field runs"
        );
        assert!(
            exp_field < exp_plain,
            "{use_heading_mask} {flags:#x}: field {exp_field} vs plain {exp_plain}"
        );
    }
}

#[test]
fn the_relaxed_field_keeps_path_costs_on_a_non_uniform_lattice() {
    // Off-grid forced lines and a fine band make uneven gaps and diagonal
    // edges: the field walks the lattice's own steps, not the uniform
    // interior shortcut, and must still bound the real search exactly.
    let mut spec = LatticeSpec::new(BBox::new(
        Point::new(0, 0),
        Point::new(2_600_000, 2_000_000),
    ));
    let mut g = Lcg(5);
    for _ in 0..9 {
        let v = g.next();
        spec.force_point(Point::new(
            ((v >> 20) % 2_500_000) as Coord + 37_000,
            ((v >> 40) % 1_900_000) as Coord + 13_000,
        ));
    }
    spec.fine_regions = vec![BBox::new(
        Point::new(900_000, 0),
        Point::new(1_500_000, 2_000_000),
    )];
    let lat = Lattice::build(&spec).unwrap();
    let w = lat.window();
    let mut st = TileStack {
        tile: TileId(0),
        profile: ProfileId(0),
        window: w,
        layers: (0..2)
            .map(|l| TileRaster::new(TileId(0), LayerId(l), ProfileId(0), w))
            .collect(),
    };
    rough_board(&mut st, &mut g, 14);
    for r in &mut st.layers {
        derive_masks_on_lattice(r, &lat, None);
    }
    let params = CostParams {
        present_factor: 4_000,
        history_factor: 1_000,
        ..CostParams::default()
    };
    let (cols, rows) = (w.cols as u16, w.rows as u16);
    for flags in [JOB_ALLOW_VIA, JOB_ALLOW_VIA | JOB_MIN_RUN] {
        let plain = SearchConfig {
            min_run_nm: 100_000,
            ..SearchConfig::default()
        };
        let with_field = SearchConfig {
            field_after: 1,
            ..plain.clone()
        };
        let jobs: Vec<(u8, JobNode, JobNode)> = (0..60u32)
            .map(|k| {
                let v = g.next();
                let s = node(
                    (v >> 33) as u16 % cols,
                    (v >> 40) as u16 % rows,
                    (k % 2) as u8,
                );
                let mut t = node(
                    (v >> 17) as u16 % cols,
                    (v >> 24) as u16 % rows,
                    ((k / 2) % 2) as u8,
                );
                if k % 2 == 0 {
                    t.heading_mask = g.mask();
                }
                (flags, s, t)
            })
            .collect();
        let ctx_plain = SearchContext {
            lattice: Some(&lat),
            ..SearchContext::new(&st, &plain)
        };
        let ctx_field = SearchContext {
            lattice: Some(&lat),
            ..SearchContext::new(&st, &with_field)
        };
        let (_, _, runs) = field_matches_plain(&ctx_plain, &ctx_field, &params, &jobs);
        assert!(runs > 30, "{flags:#x}: {runs} field runs");
    }
}

#[test]
fn the_relaxed_field_bounds_passing_through_a_masked_target() {
    // The target sits in a one-node hole in a thick wall and may only be
    // entered from the pocket behind it, so every path passes through it
    // first, loops in the pocket and comes back. Its predecessor in the
    // hole is reached two ways: straight in, after a slightly dearer
    // diagonal (a little history), or diagonally, cheaper there but a 45°
    // bend into the target. If the field bounded the pass-through state at
    // the target by 0 (its end state's value), that state would close on
    // the first, dearer arrival before the straight one popped.
    let mut st = stack(40, 24, 1);
    for col in 19..=20 {
        block(&mut st, 0, col..=col, 3..=9);
        block(&mut st, 0, col..=col, 11..=17);
    }
    block(&mut st, 0, 19..=32, 3..=3);
    block(&mut st, 0, 19..=32, 17..=17);
    block(&mut st, 0, 32..=32, 3..=17);
    rederive(&mut st);
    st.layers[0].history.set(18, 10, 1);
    let params = CostParams {
        history_factor: 20,
        ..CostParams::default()
    };
    let plain = SearchConfig {
        use_heading_mask: false,
        ..SearchConfig::default()
    };
    let with_field = SearchConfig {
        field_after: 1,
        ..plain.clone()
    };
    let mut t = node(20, 10, 0);
    t.heading_mask = 1 << 4; // enter moving towards col - 1
    let jobs = [(0u8, node(17, 11, 0), t)];
    let (_, _, runs) = field_matches_plain(
        &SearchContext::new(&st, &plain),
        &SearchContext::new(&st, &with_field),
        &params,
        &jobs,
    );
    assert_eq!(runs, 1);
    let mut pool = JobPool::default();
    let i = pool.push(job(0), &[jobs[0].1], &[t], &[]);
    let mut stats = SearchStats::default();
    let o = search_with(
        &SearchContext::new(&st, &with_field),
        &pool,
        &pool.jobs[0],
        i,
        &params,
        &mut stats,
    );
    let p = found(&o);
    // Into the hole straight, through the target, round the pocket and
    // back in.
    let head: Vec<(u16, u16)> = p.nodes[..2].iter().map(|n| (n.col, n.row)).collect();
    assert_eq!(head, [(17, 11), (18, 10)]);
    assert_eq!(runs_of_target(&p.nodes), 2);
}

/// How many times a path visits its last node.
fn runs_of_target(nodes: &[JobNode]) -> usize {
    let t = nodes.last().unwrap();
    nodes
        .iter()
        .filter(|n| (n.col, n.row, n.layer) == (t.col, t.row, t.layer))
        .count()
}

#[test]
fn a_track_region_keeps_one_layer_s_runs_inside_its_box() {
    let mut st = stack(30, 20, 2);
    block(&mut st, 0, 15..=15, 0..=19);
    rederive(&mut st);
    let region = |c0: i32, c1: i32| crate::global::TrackRegion {
        layer: LayerId(1),
        bbox: BBox::new(
            Point::new(c0 * PITCH, 0),
            Point::new(c1 * PITCH, 19 * PITCH),
        ),
    };
    // The box spans the wall: the detour on layer 1 stays inside it.
    let cfg = SearchConfig {
        track_region: Some(region(12, 18)),
        ..SearchConfig::default()
    };
    let ctx = SearchContext::new(&st, &cfg);
    let (pool, i) = pool_with(job(JOB_ALLOW_VIA), &[node(5, 10, 0)], &[node(25, 10, 0)]);
    let (o, _) = run_ctx(&ctx, &pool, i);
    let p = found(&o);
    assert!(p.nodes.iter().any(|n| n.layer == 1));
    assert!(p
        .nodes
        .iter()
        .filter(|n| n.layer == 1)
        .all(|n| (12..=18).contains(&n.col)));
    // A box away from the wall: layer 1 cannot carry the detour.
    let cfg = SearchConfig {
        track_region: Some(region(2, 8)),
        ..SearchConfig::default()
    };
    let ctx = SearchContext::new(&st, &cfg);
    assert!(matches!(
        run_ctx(&ctx, &pool, i).0,
        SearchOutcome::Unreachable { .. }
    ));
}
