//! Capture real negotiation searches and replay them against the kernel: a
//! deterministic, single-threaded benchmark for kernel speed work.
//!
//! Both tests are gated and print why they skip.
//!
//! - **Capture** (`capture_negotiation_searches`): runs the pipeline on the
//!   board in `KONNECT_DB1_FIXTURE_DIR` with a backend that wraps
//!   [`KernelBackend`] and writes a sample of its searches (the stack, the
//!   job, the cost parameters, the tuning and the outcome) to
//!   `KONNECT_SEARCH_CAPTURE_DIR`. A search is kept when a hash of its
//!   connection, piece and iteration falls in one of
//!   `KONNECT_SEARCH_CAPTURE_EVERY` (default 150) buckets, so the sample
//!   does not depend on the thread count. `KONNECT_PIPELINE_ITERATIONS`
//!   limits negotiation as in `pipeline_real_fixture`.
//! - **Replay** (`replay_captured_searches`): reads every capture in
//!   `KONNECT_SEARCH_REPLAY_DIR`, runs each search again on one thread and
//!   prints time, expansions and nanoseconds per expansion. Unless
//!   `KONNECT_SEARCH_REPLAY_ASTAR` (×1000) changes the heuristic weight, it
//!   asserts that every outcome (path, cost, expansions) equals the
//!   captured one, which makes it an exactness check for kernel changes.
//!   `KONNECT_SEARCH_REPLAY_FIELD_AFTER` sets the kernel's
//!   `field_after` (which keeps path costs but may change ties), and
//!   `KONNECT_SEARCH_REPLAY_THREADS` runs searches on that many threads.
//!   `KONNECT_SEARCH_REPLAY_REPEAT` runs the set that many times.
//!
//! Captures hold board-derived rasters: keep them in an ignored build
//! location, never in the repository. The capture test refuses a
//! directory inside any git work tree.

use konnect_geom::Point;
use konnect_route::api::{LayerId, ProfileId, TileId};
use konnect_route::compute::{
    Batch, Cost, CostField, CostParams, JobDescriptor, JobNode, JobPool, Plane, RouteCompute,
    SearchBackend, SearchOutcome, TileRaster, TileStack, Window,
};
use konnect_route::detail::search::{search_with, SearchConfig, SearchContext, SearchStats};
use konnect_route::pipeline::backend::min_run_nets;
use konnect_route::pipeline::{route_board_with, KernelBackend, PipelineScope, PipelineSettings};
use konnect_route::{load_board_file, LoadOptions, ProjectRuleTable};
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Instant;

const MAGIC: &[u8; 4] = b"KSR1";

fn only_file(dir: &Path, ext: &str) -> Option<PathBuf> {
    let mut found = std::fs::read_dir(dir)
        .ok()?
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|e| e.to_str()) == Some(ext));
    let first = found.next()?;
    found.next().is_none().then_some(first)
}

fn env_num(name: &str, default: u64) -> u64 {
    std::env::var(name)
        .ok()
        .and_then(|v| v.trim().parse().ok())
        .unwrap_or(default)
}

// ─── A tiny little-endian codec ──────────────────────────────────────────────

struct Out(Vec<u8>);

impl Out {
    fn u8(&mut self, v: u8) {
        self.0.push(v);
    }
    fn u16(&mut self, v: u16) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn u32(&mut self, v: u32) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn i32(&mut self, v: i32) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn u64(&mut self, v: u64) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn bytes(&mut self, b: &[u8]) {
        self.u32(b.len() as u32);
        self.0.extend_from_slice(b);
    }
    /// Run-length encoded values: `(value, run)` pairs.
    fn rle(&mut self, vals: impl Iterator<Item = u16>) {
        let mut pairs: Vec<(u16, u32)> = Vec::new();
        for v in vals {
            match pairs.last_mut() {
                Some((pv, n)) if *pv == v => *n += 1,
                _ => pairs.push((v, 1)),
            }
        }
        self.u32(pairs.len() as u32);
        for (v, n) in pairs {
            self.u16(v);
            self.u32(n);
        }
    }
    fn node(&mut self, n: &JobNode) {
        self.u16(n.col);
        self.u16(n.row);
        self.u8(n.layer);
        self.u8(n.heading_mask);
    }
}

struct In<'a>(&'a [u8]);

impl In<'_> {
    fn take(&mut self, n: usize) -> &[u8] {
        let (a, b) = self.0.split_at(n);
        self.0 = b;
        a
    }
    fn u8(&mut self) -> u8 {
        self.take(1)[0]
    }
    fn u16(&mut self) -> u16 {
        u16::from_le_bytes(self.take(2).try_into().unwrap())
    }
    fn u32(&mut self) -> u32 {
        u32::from_le_bytes(self.take(4).try_into().unwrap())
    }
    fn i32(&mut self) -> i32 {
        i32::from_le_bytes(self.take(4).try_into().unwrap())
    }
    fn u64(&mut self) -> u64 {
        u64::from_le_bytes(self.take(8).try_into().unwrap())
    }
    fn bytes(&mut self) -> Vec<u8> {
        let n = self.u32() as usize;
        self.take(n).to_vec()
    }
    fn rle(&mut self) -> Vec<u16> {
        let pairs = self.u32();
        let mut out = Vec::new();
        for _ in 0..pairs {
            let v = self.u16();
            let n = self.u32() as usize;
            out.extend(std::iter::repeat_n(v, n));
        }
        out
    }
    fn node(&mut self) -> JobNode {
        JobNode {
            col: self.u16(),
            row: self.u16(),
            layer: self.u8(),
            heading_mask: self.u8(),
            reserved: 0,
        }
    }
}

fn put_plane8(o: &mut Out, p: &Plane<u8>) {
    o.rle(p.as_slice().iter().map(|&v| u16::from(v)));
}

fn put_plane16(o: &mut Out, p: &Plane<u16>) {
    o.rle(p.as_slice().iter().copied());
}

fn get_plane8(i: &mut In<'_>, cols: u32, rows: u32) -> Plane<u8> {
    let mut p = Plane::new(cols, rows);
    let v = i.rle();
    assert_eq!(v.len(), p.as_slice().len(), "plane size");
    for (d, s) in p.as_mut_slice().iter_mut().zip(v) {
        *d = s as u8;
    }
    p
}

fn get_plane16(i: &mut In<'_>, cols: u32, rows: u32) -> Plane<u16> {
    let mut p = Plane::new(cols, rows);
    let v = i.rle();
    assert_eq!(v.len(), p.as_slice().len(), "plane size");
    p.as_mut_slice().copy_from_slice(&v);
    p
}

fn put_job(o: &mut Out, j: &JobDescriptor) {
    for v in [
        j.connection,
        j.net,
        j.window,
        u32::from(j.profile),
        u32::from(j.size_class),
        u32::from(j.flags),
        j.layer_mask,
        j.cost_bound,
        j.order_key,
        j.iteration,
    ] {
        o.u32(v);
    }
}

fn get_job(i: &mut In<'_>) -> JobDescriptor {
    JobDescriptor {
        connection: i.u32(),
        net: i.u32(),
        window: i.u32(),
        profile: i.u32() as u16,
        size_class: i.u32() as u8,
        flags: i.u32() as u8,
        layer_mask: i.u32(),
        cost_bound: i.u32(),
        order_key: i.u32(),
        iteration: i.u32(),
        ..JobDescriptor::default()
    }
}

fn put_outcome(o: &mut Out, out: &SearchOutcome) {
    match out {
        SearchOutcome::Found(p) => {
            o.u8(0);
            o.u32(p.cost.0);
            o.u64(p.expansions);
            o.u32(p.nodes.len() as u32);
            for n in &p.nodes {
                o.node(n);
            }
        }
        SearchOutcome::Unreachable { expansions, .. } => {
            o.u8(1);
            o.u64(*expansions);
        }
        SearchOutcome::BudgetExhausted { expansions, .. } => {
            o.u8(2);
            o.u64(*expansions);
        }
    }
}

/// Straight runs of at most `max` lattice steps in a densified path (via
/// pairs excluded): the jogs a segment-length rule may refuse.
fn short_runs(nodes: &[JobNode], max: u32) -> (usize, usize) {
    let mut runs = Vec::new();
    let mut len = 0u32;
    let mut dir: Option<(i32, i32)> = None;
    for w in nodes.windows(2) {
        let (a, b) = (w[0], w[1]);
        if a.layer != b.layer {
            if len > 0 {
                runs.push(len);
            }
            len = 0;
            dir = None;
            continue;
        }
        let d = (
            i32::from(b.col) - i32::from(a.col),
            i32::from(b.row) - i32::from(a.row),
        );
        if dir.is_some_and(|x| x != d) {
            runs.push(len);
            len = 0;
        }
        dir = Some(d);
        len += 1;
    }
    if len > 0 {
        runs.push(len);
    }
    // Interior runs only: the first and last end on terminals.
    let inner = if runs.len() > 2 {
        &runs[1..runs.len() - 1]
    } else {
        &[][..]
    };
    (inner.iter().filter(|&&l| l <= max).count(), runs.len())
}

/// `(kind, cost, expansions, nodes)` of an outcome, for comparison.
type Summary = (u8, u32, u64, Vec<JobNode>);

fn summarize(out: &SearchOutcome) -> Summary {
    match out {
        SearchOutcome::Found(p) => (0, p.cost.0, p.expansions, p.nodes.clone()),
        SearchOutcome::Unreachable { expansions, .. } => (1, 0, *expansions, Vec::new()),
        SearchOutcome::BudgetExhausted { expansions, .. } => (2, 0, *expansions, Vec::new()),
    }
}

fn get_outcome(i: &mut In<'_>) -> Summary {
    match i.u8() {
        0 => {
            let cost = i.u32();
            let e = i.u64();
            let n = i.u32();
            let nodes = (0..n).map(|_| i.node()).collect();
            (0, cost, e, nodes)
        }
        k => (k, 0, i.u64(), Vec::new()),
    }
}

/// One captured search.
struct Capture {
    config: SearchConfig,
    min_run: bool,
    params: CostParams,
    job: JobDescriptor,
    sources: Vec<JobNode>,
    targets: Vec<JobNode>,
    stack: TileStack,
    outcome: Summary,
}

fn encode(
    config: &SearchConfig,
    min_run: bool,
    params: &CostParams,
    job: &JobDescriptor,
    pool: &JobPool,
    stack: &TileStack,
    outcome: &SearchOutcome,
) -> Vec<u8> {
    let mut o = Out(Vec::new());
    o.0.extend_from_slice(MAGIC);
    o.bytes(&serde_json::to_vec(config).expect("config serializes"));
    o.u8(u8::from(min_run));
    o.u32(params.via.0);
    o.u32(params.wrong_way_step.0);
    o.u32(params.present_factor);
    o.u32(params.history_factor);
    o.u16(params.history_increment);
    o.u32(params.near_step.0);
    put_job(&mut o, job);
    for list in [pool.sources(job), pool.targets(job)] {
        o.u32(list.len() as u32);
        for n in list {
            o.node(n);
        }
    }
    o.u32(stack.tile.0);
    o.u16(stack.profile.0);
    let w = stack.window;
    o.i32(w.origin.x);
    o.i32(w.origin.y);
    o.i32(w.pitch_nm);
    o.u32(w.cols);
    o.u32(w.rows);
    o.u32(stack.layers.len() as u32);
    for r in &stack.layers {
        o.u32(r.layer.0.into());
        put_plane8(&mut o, &r.core);
        put_plane8(&mut o, &r.halo);
        put_plane8(&mut o, &r.heading_mask);
        put_plane8(&mut o, &r.via_site);
        put_plane16(&mut o, &r.present);
        put_plane16(&mut o, &r.history);
    }
    put_outcome(&mut o, outcome);
    o.0
}

fn decode(bytes: &[u8]) -> Capture {
    let mut i = In(bytes);
    assert_eq!(i.take(4), MAGIC, "capture magic");
    let config: SearchConfig = serde_json::from_slice(&i.bytes()).expect("config parses");
    let min_run = i.u8() != 0;
    let params = CostParams {
        via: Cost(i.u32()),
        wrong_way_step: Cost(i.u32()),
        present_factor: i.u32(),
        history_factor: i.u32(),
        history_increment: i.u16(),
        near_step: Cost(i.u32()),
    };
    let job = get_job(&mut i);
    let mut lists = [Vec::new(), Vec::new()];
    for l in &mut lists {
        let n = i.u32();
        *l = (0..n).map(|_| i.node()).collect();
    }
    let [sources, targets] = lists;
    let tile = TileId(i.u32());
    let profile = ProfileId(i.u16());
    let origin = Point::new(i.i32(), i.i32());
    let pitch = i.i32();
    let (cols, rows) = (i.u32(), i.u32());
    let window = Window::new(origin, pitch, cols, rows).expect("window");
    let n = i.u32();
    let mut layers = Vec::new();
    for _ in 0..n {
        let layer = LayerId(i.u32() as _);
        let mut r = TileRaster::new(tile, layer, profile, window);
        r.core = get_plane8(&mut i, cols, rows);
        r.halo = get_plane8(&mut i, cols, rows);
        r.heading_mask = get_plane8(&mut i, cols, rows);
        r.via_site = get_plane8(&mut i, cols, rows);
        r.present = get_plane16(&mut i, cols, rows);
        r.history = get_plane16(&mut i, cols, rows);
        layers.push(r);
    }
    let outcome = get_outcome(&mut i);
    Capture {
        config,
        min_run,
        params,
        job,
        sources,
        targets,
        stack: TileStack {
            tile,
            profile,
            window,
            layers,
        },
        outcome,
    }
}

// ─── Capture ─────────────────────────────────────────────────────────────────

struct Shared {
    dir: PathBuf,
    every: u64,
    written: AtomicU64,
}

/// [`KernelBackend`] that writes a deterministic sample of its searches.
#[derive(Clone)]
struct CaptureBackend {
    inner: KernelBackend,
    shared: Arc<Shared>,
}

fn keep(job: &JobDescriptor, every: u64) -> bool {
    // SplitMix64 over the job's identity: independent of threads.
    let mut z = (u64::from(job.connection) << 32 | u64::from(job.order_key))
        ^ u64::from(job.iteration).wrapping_mul(0x9e37_79b9_7f4a_7c15);
    z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    z ^= z >> 31;
    z % every.max(1) == 0
}

impl SearchBackend for CaptureBackend {
    fn search_batch(
        &mut self,
        batch: &Batch,
        pool: &JobPool,
        stacks: &[TileStack],
        params: &CostParams,
    ) -> Vec<SearchOutcome> {
        let out = self.inner.search_batch(batch, pool, stacks, params);
        for (k, &j) in batch.jobs.iter().enumerate() {
            let job = &pool.jobs[j as usize];
            if !keep(job, self.shared.every) {
                continue;
            }
            let stack = &stacks[job.window as usize];
            // The job's own minimum-run flag, as the backend set it.
            let min_run = self.inner.min_run
                && self.inner.min_run_nets.as_ref().is_none_or(|nets| {
                    nets.get(job.net as usize).copied().unwrap_or(false)
                });
            let bytes = encode(
                &self.inner.config,
                min_run,
                params,
                job,
                pool,
                stack,
                &out[k],
            );
            let name = format!(
                "it{:02}-c{:05}-k{:08x}.ksr",
                job.iteration, job.connection, job.order_key
            );
            let mut f = std::fs::File::create(self.shared.dir.join(name)).expect("capture file");
            f.write_all(&bytes).expect("capture writes");
            self.shared.written.fetch_add(1, Ordering::Relaxed);
        }
        out
    }

    fn cost_field(
        &mut self,
        stack: &TileStack,
        sources: &[JobNode],
        params: &CostParams,
    ) -> CostField {
        self.inner.cost_field(stack, sources, params)
    }
}

impl RouteCompute for CaptureBackend {
    fn paint(
        &mut self,
        raster: &mut TileRaster,
        shapes: &[konnect_route::compute::PaintShape],
        profile: &konnect_route::compute::ProfileGeometry,
    ) {
        self.inner.paint(raster, shapes, profile);
    }

    fn unpaint(
        &mut self,
        raster: &mut TileRaster,
        shapes: &[konnect_route::compute::PaintShape],
        profile: &konnect_route::compute::ProfileGeometry,
    ) {
        self.inner.unpaint(raster, shapes, profile);
    }

    fn derive_masks(&mut self, raster: &mut TileRaster) {
        self.inner.derive_masks(raster);
    }

    fn update_costs(&mut self, raster: &mut TileRaster, params: &CostParams) {
        self.inner.update_costs(raster, params);
    }

    fn fork(&self) -> Option<Box<dyn RouteCompute + Send>> {
        Some(Box::new(self.clone()))
    }
}

#[test]
fn capture_negotiation_searches() {
    let Some(out) = std::env::var_os("KONNECT_SEARCH_CAPTURE_DIR").map(PathBuf::from) else {
        eprintln!("skipping: KONNECT_SEARCH_CAPTURE_DIR is not set");
        return;
    };
    let Some(dir) = std::env::var_os("KONNECT_DB1_FIXTURE_DIR").map(PathBuf::from) else {
        eprintln!("skipping: KONNECT_DB1_FIXTURE_DIR is not set");
        return;
    };
    let (Some(pcb), Some(pro)) = (only_file(&dir, "kicad_pcb"), only_file(&dir, "kicad_pro"))
    else {
        eprintln!("skipping: the fixture lacks a single .kicad_pcb and .kicad_pro");
        return;
    };
    // Captures are board data: refuse any directory inside a git work tree,
    // where one `git add` would commit them.
    let abs = std::path::absolute(&out).expect("capture dir path");
    if let Some(repo) = abs.ancestors().find(|a| a.join(".git").exists()) {
        panic!(
            "KONNECT_SEARCH_CAPTURE_DIR is inside the git work tree at {}; \
             captures hold board data, so put them outside any repository",
            repo.display()
        );
    }
    std::fs::create_dir_all(&out).expect("capture dir");
    let (mut board, _) = load_board_file(&pcb, &LoadOptions::default()).expect("board loads");
    let pro_text = std::fs::read_to_string(&pro).expect("project reads");
    let dru_text =
        only_file(&dir, "kicad_dru").map(|p| std::fs::read_to_string(p).expect("rules read"));
    let rules = ProjectRuleTable::from_project(&pro_text, dru_text.as_deref(), &board)
        .expect("rules compile");
    rules.assign_classes(&mut board);
    let mut settings = PipelineSettings::default();
    settings.negotiation.max_iterations = env_num("KONNECT_PIPELINE_ITERATIONS", 10) as u32;
    settings.optimize = None;
    let shared = Arc::new(Shared {
        dir: out.clone(),
        every: env_num("KONNECT_SEARCH_CAPTURE_EVERY", 150),
        written: AtomicU64::new(0),
    });
    // The kernel backend exactly as `route_board` sets it up.
    let mut inner = KernelBackend::new(settings.search.clone());
    if settings.search_min_run {
        if let Some((nets, min)) = min_run_nets(&board, &rules) {
            inner.min_run = true;
            inner.min_run_nets = Some(Arc::new(nets));
            inner.config.min_run_nm = min;
        }
    }
    let mut backend = CaptureBackend {
        inner,
        shared: shared.clone(),
    };
    let t = Instant::now();
    let r = route_board_with(
        &board,
        &rules,
        &PipelineScope::default(),
        &settings,
        &mut backend,
    )
    .expect("the pipeline plans");
    println!(
        "captured {} searches into {} in {:.1} s; routed {} of {}",
        shared.written.load(Ordering::Relaxed),
        out.display(),
        t.elapsed().as_secs_f64(),
        r.detail.routed.len(),
        r.connections.len()
    );
}

/// Print a found path's cost split into step, congestion, near, bend and
/// via parts (debugging the relaxed field).
fn explain(c: &Capture, nodes: &[JobNode]) {
    let (pf, hf) = (
        u64::from(c.params.present_factor),
        u64::from(c.params.history_factor),
    );
    let (mut base, mut cong, mut near, mut bends, mut vias, mut via_cost) =
        (0u64, 0u64, 0u64, 0u64, 0u64, 0u64);
    let mut prev_dir: Option<(i32, i32)> = None;
    for w in nodes.windows(2) {
        let (a, b) = (w[0], w[1]);
        if a.layer != b.layer {
            vias += 1;
            prev_dir = None;
            let mut pres = 0u64;
            let mut hist = 0u64;
            for l in &c.stack.layers {
                for (dc, dr) in [
                    (0i32, 0i32),
                    (5, 0),
                    (-5, 0),
                    (0, 5),
                    (0, -5),
                    (4, 4),
                    (4, -4),
                    (-4, 4),
                    (-4, -4),
                ] {
                    let (cc, rr) = (i32::from(a.col) + dc, i32::from(a.row) + dr);
                    if cc < 0 || rr < 0 || cc as u32 >= l.window.cols || rr as u32 >= l.window.rows
                    {
                        continue;
                    }
                    pres = pres.max(u64::from(l.present.get(cc as u32, rr as u32)));
                    hist = hist.max(u64::from(l.history.get(cc as u32, rr as u32)));
                }
            }
            via_cost += u64::from(c.params.via.0) * (1000 + hist * hf) / 1000 * (1000 + pres * pf)
                / 1000
                + u64::from(c.params.near_step.0);
            continue;
        }
        let d = (
            i32::from(b.col) - i32::from(a.col),
            i32::from(b.row) - i32::from(a.row),
        );
        let step = if d.0 != 0 && d.1 != 0 { 1414u64 } else { 1000 };
        let l = &c.stack.layers[usize::from(a.layer)];
        let (pres, hist) = (
            u64::from(l.present.get(b.col.into(), b.row.into())),
            u64::from(l.history.get(b.col.into(), b.row.into())),
        );
        let scaled = step * (1000 + hist * hf) / 1000 * (1000 + pres * pf) / 1000;
        base += step;
        cong += scaled - step;
        if l.halo.get(a.col.into(), a.row.into()) > 0 || l.halo.get(b.col.into(), b.row.into()) > 0
        {
            near += u64::from(c.params.near_step.0);
        }
        if let Some(p) = prev_dir {
            if p != d {
                bends += 1;
            }
        }
        prev_dir = Some(d);
    }
    println!(
        "  explain cost {}: base {base} congestion {cong} near {near} vias {vias} ({via_cost}) bends {bends}",
        c.outcome.1
    );
}

// ─── Replay ──────────────────────────────────────────────────────────────────

#[test]
fn replay_captured_searches() {
    let Some(dir) = std::env::var_os("KONNECT_SEARCH_REPLAY_DIR").map(PathBuf::from) else {
        eprintln!("skipping: KONNECT_SEARCH_REPLAY_DIR is not set");
        return;
    };
    let mut files: Vec<PathBuf> = std::fs::read_dir(&dir)
        .expect("replay dir")
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|e| e.to_str()) == Some("ksr"))
        .collect();
    files.sort();
    if let Ok(only) = std::env::var("KONNECT_SEARCH_REPLAY_ONLY") {
        files.retain(|f| f.to_string_lossy().contains(&only));
    }
    let limit = env_num("KONNECT_SEARCH_REPLAY_LIMIT", u64::MAX) as usize;
    files.truncate(limit);
    let astar = std::env::var("KONNECT_SEARCH_REPLAY_ASTAR")
        .ok()
        .and_then(|v| v.trim().parse::<u32>().ok());
    let repeat = env_num("KONNECT_SEARCH_REPLAY_REPEAT", 1).max(1);
    let field_after = std::env::var("KONNECT_SEARCH_REPLAY_FIELD_AFTER")
        .ok()
        .and_then(|v| v.trim().parse::<u64>().ok());
    let t_load = Instant::now();
    let caps: Vec<Capture> = files
        .iter()
        .map(|f| {
            let mut b = Vec::new();
            std::fs::File::open(f)
                .and_then(|mut h| h.read_to_end(&mut b))
                .expect("capture reads");
            decode(&b)
        })
        .collect();
    println!(
        "loaded {} captures in {:.1} s",
        caps.len(),
        t_load.elapsed().as_secs_f64()
    );
    // Searches run on a pool of `KONNECT_SEARCH_REPLAY_THREADS` (default 1),
    // so each batch's rayon call stays on the thread that issued it.
    let threads = env_num("KONNECT_SEARCH_REPLAY_THREADS", 1).max(1) as usize;
    let pool_n = rayon::ThreadPoolBuilder::new()
        .num_threads(threads)
        .build()
        .expect("pool");
    let run_one = |c: &Capture| -> (Summary, f64) {
        let ts = Instant::now();
        let mut config = c.config.clone();
        if let Some(a) = astar {
            config.astar_factor = a;
        }
        if let Some(f) = field_after {
            config.field_after = f;
        }
        // The capture holds the job's own flag, so flag every job.
        let mut backend = KernelBackend::new(config);
        backend.min_run = c.min_run;
        let mut pool = JobPool::default();
        let mut job = c.job;
        job.window = 0;
        pool.push(job, &c.sources, &c.targets, &[]);
        let batch = Batch {
            index: 0,
            jobs: vec![0],
        };
        let out = backend.search_batch(&batch, &pool, std::slice::from_ref(&c.stack), &c.params);
        (summarize(&out[0]), ts.elapsed().as_secs_f64())
    };
    let mut mismatches = 0usize;
    let (mut exp_old, mut exp_new) = (0u64, 0u64);
    let (mut cost_old, mut cost_new) = (0u64, 0u64);
    let mut found_old = 0usize;
    let mut found_new = 0usize;
    let mut best = f64::INFINITY;
    let mut last: Vec<Summary> = Vec::new();
    // Per search, the fastest of the repeats: robust to a busy machine.
    let mut fastest = vec![f64::INFINITY; caps.len()];
    for rep in 0..repeat {
        let t = Instant::now();
        let outs: Vec<(Summary, f64)> = pool_n.install(|| {
            use rayon::prelude::*;
            caps.par_iter().with_max_len(1).map(run_one).collect()
        });
        let mut times: Vec<(f64, usize)> = outs.iter().enumerate().map(|(i, o)| (o.1, i)).collect();
        for &(t, i) in &times {
            fastest[i] = fastest[i].min(t);
        }
        let results: Vec<Summary> = outs.into_iter().map(|o| o.0).collect();
        last = results.clone();
        times.sort_by(|a, b| b.0.total_cmp(&a.0));
        for &(t, ci) in times
            .iter()
            .take(env_num("KONNECT_SEARCH_REPLAY_SLOWEST", 0) as usize)
        {
            let c = &caps[ci];
            let (a, b) = (c.sources[0], c.targets[0]);
            let (dx, dy) = (
                (i64::from(a.col) - i64::from(b.col)).abs(),
                (i64::from(a.row) - i64::from(b.row)).abs(),
            );
            let oct = 1000 * dx.max(dy) + 414 * dx.min(dy);
            println!(
                "  {:.3} s {} exp {} -> {} cost {} octile {} layers {}->{} masks {:08b}->{:08b} nodes {}x{} sources {} targets {}",
                t,
                files[ci].file_name().unwrap().to_string_lossy(),
                c.outcome.2,
                results[ci].2,
                c.outcome.1,
                oct,
                a.layer,
                b.layer,
                a.heading_mask,
                b.heading_mask,
                c.stack.window.cols,
                c.stack.window.rows,
                c.sources.len(),
                c.targets.len()
            );
        }
        let secs = t.elapsed().as_secs_f64();
        best = best.min(secs);
        if rep == 0 {
            for (c, r) in caps.iter().zip(&results) {
                exp_old += c.outcome.2;
                exp_new += r.2;
                if c.outcome.0 == 0 {
                    found_old += 1;
                    cost_old += u64::from(c.outcome.1);
                }
                if r.0 == 0 {
                    found_new += 1;
                    cost_new += u64::from(r.1);
                }
                if *r != c.outcome {
                    mismatches += 1;
                }
            }
        }
        println!("replay {rep}: {secs:.3} s on {threads} threads");
    }
    println!(
        "searches {}: found {} -> {}, expansions {} -> {} ({:.3}x), found cost sum {} -> {}",
        caps.len(),
        found_old,
        found_new,
        exp_old,
        exp_new,
        exp_old as f64 / exp_new.max(1) as f64,
        cost_old,
        cost_new
    );
    let (mut short_old, mut short_new, mut runs_old, mut runs_new) = (0, 0, 0, 0);
    for (c, r) in caps.iter().zip(&last) {
        let (a, b) = short_runs(&c.outcome.3, 2);
        short_old += a;
        runs_old += b;
        let (a, b) = short_runs(&r.3, 2);
        short_new += a;
        runs_new += b;
    }
    println!("short interior runs (<= 2 steps): {short_old} -> {short_new}; runs {runs_old} -> {runs_new}");
    println!(
        "sum of per-search fastest times: {:.3} s",
        fastest.iter().sum::<f64>()
    );
    println!(
        "best {:.3} s, {:.1} ns per expansion, {} outcomes differ from the capture",
        best,
        best * 1e9 / exp_new.max(1) as f64,
        mismatches
    );
    if std::env::var_os("KONNECT_SEARCH_REPLAY_EXPLAIN").is_some() {
        for (c, r) in caps.iter().zip(&last) {
            explain(c, &r.3);
        }
    }
    if std::env::var_os("KONNECT_SEARCH_REPLAY_STATS").is_some() {
        let mut total = SearchStats::default();
        for c in &caps {
            let mut pool = JobPool::default();
            let mut job = c.job;
            job.window = 0;
            if c.min_run {
                job.flags |= konnect_route::compute::job::JOB_MIN_RUN;
            }
            pool.push(job, &c.sources, &c.targets, &[]);
            let mut config = c.config.clone();
            if let Some(a) = astar {
                config.astar_factor = a;
            }
            if let Some(f) = field_after {
                config.field_after = f;
            }
            let ctx = SearchContext::new(&c.stack, &config);
            let mut st = SearchStats::default();
            search_with(&ctx, &pool, &pool.jobs[0], 0, &c.params, &mut st);
            total.add(&st);
        }
        println!("stats: {total:?}");
    }
    if astar.is_none()
        && field_after.is_none()
        && std::env::var_os("KONNECT_SEARCH_REPLAY_NO_ASSERT").is_none()
    {
        assert_eq!(mismatches, 0, "the kernel's outcomes changed");
    }
}
