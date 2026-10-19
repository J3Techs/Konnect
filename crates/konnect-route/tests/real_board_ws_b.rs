//! WS-B stages on a real board kept outside the repository: connectivity
//! (S1), the terminal-access audit (S1), plane fanout (S2) and pin access
//! (S4).
//!
//! ```text
//! KONNECT_DB1_FIXTURE_DIR=/path/to/fixture cargo test --release \
//!     -p konnect-route --test real_board_ws_b -- --nocapture --test-threads 1
//! ```
//!
//! The directory holds one `.kicad_pcb` with its `.kicad_pro` and
//! `.kicad_dru`. An optional sub-directory with another `.kicad_pcb` (a
//! routed candidate of the same project) and the JSON reports of an older
//! connectivity script is compared as well, when present. Nothing about the
//! board is committed: the tests find files by extension, choose footprints
//! by shape (the two largest ball-grid footprints), assert only invariants
//! that hold for any KiCad-authored board, and print what they measure.
//! Reports go to `$CARGO_TARGET_DIR/ws-b-reports` (or the system temp dir).
//! Without the variable the tests print why and pass.

use konnect_geom::{ArcPolicy, Shape};
use konnect_route::api::{FootprintId, ItemRef, NetId, PadId, Stage};
use konnect_route::audit::{audit_terminal_access_with, AuditSettings, SizeChoice};
use konnect_route::board::{PadKind, RouteBoard};
use konnect_route::connectivity::{plane_nets, ConnectivityReport};
use konnect_route::telemetry::Telemetry;
use konnect_route::{load_board_file, LoadOptions, ProjectRuleTable};
use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};
use std::time::Instant;

fn only_file(dir: &Path, ext: &str) -> Option<PathBuf> {
    let mut found = std::fs::read_dir(dir)
        .ok()?
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|e| e.to_str()) == Some(ext));
    let first = found.next()?;
    found.next().is_none().then_some(first)
}

fn fixture_dir() -> Option<PathBuf> {
    let Some(dir) = std::env::var_os("KONNECT_DB1_FIXTURE_DIR").map(PathBuf::from) else {
        eprintln!("skipping: KONNECT_DB1_FIXTURE_DIR is not set");
        return None;
    };
    if only_file(&dir, "kicad_pcb").is_none() {
        eprintln!("skipping: {} holds no single .kicad_pcb", dir.display());
        return None;
    }
    Some(dir)
}

/// The first sub-directory holding a single `.kicad_pcb`.
fn candidate_dir(dir: &Path) -> Option<PathBuf> {
    let mut subs: Vec<PathBuf> = std::fs::read_dir(dir)
        .ok()?
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.is_dir())
        .collect();
    subs.sort();
    subs.into_iter()
        .find(|d| only_file(d, "kicad_pcb").is_some())
}

fn report_dir() -> PathBuf {
    let base = std::env::var_os("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(std::env::temp_dir);
    let d = base.join("ws-b-reports");
    std::fs::create_dir_all(&d).expect("report dir");
    d
}

struct Loaded {
    board: RouteBoard,
    rules: ProjectRuleTable,
}

fn load(pcb: &Path, project_dir: &Path, tel: &mut Telemetry) -> Loaded {
    let (mut board, report) = tel.time(Stage::Ingest, || {
        load_board_file(pcb, &LoadOptions::default()).expect("board loads")
    });
    assert_eq!(board.validate(), Ok(()));
    assert_eq!(report.skipped_total(), 0, "{:?}", report.skipped);
    let pro = std::fs::read_to_string(only_file(project_dir, "kicad_pro").expect("a .kicad_pro"))
        .expect("project reads");
    let dru = only_file(project_dir, "kicad_dru").map(|p| std::fs::read_to_string(p).unwrap());
    let rules = ProjectRuleTable::from_project(&pro, dru.as_deref(), &board).expect("rules");
    rules.assign_classes(&mut board);
    Loaded { board, rules }
}

fn pad_name(board: &RouteBoard, p: PadId) -> String {
    let pad = &board.pads[p.index()];
    let r = board.footprints[pad.footprint.index()]
        .reference
        .as_deref()
        .unwrap_or("?");
    format!("{r}-{}", pad.number)
}

/// The largest footprints whose pads are all round SMD lands: the BGAs.
fn ball_grids(board: &RouteBoard, n: usize) -> Vec<FootprintId> {
    let mut v: Vec<(u32, FootprintId)> = board
        .footprints
        .iter()
        .filter(|f| {
            let pads = &board.pads[f.pad_range.0 as usize..f.pad_range.1 as usize];
            pads.len() >= 64
                && pads.iter().all(|p| {
                    p.kind == PadKind::Smd && matches!(p.shapes.as_slice(), [Shape::Circle(_)])
                })
        })
        .map(|f| (f.pad_range.1 - f.pad_range.0, f.id))
        .collect();
    v.sort_by(|a, b| b.cmp(a));
    v.into_iter().take(n).map(|(_, f)| f).collect()
}

/// The older script's report: per net, its pad groups (only nets with more
/// than one group are listed).
/// Plane net names, and per net its pad groups by name.
type OldReport = (BTreeSet<String>, BTreeMap<String, Vec<Vec<String>>>);

fn old_groups(path: &Path) -> Option<OldReport> {
    let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(path).ok()?).ok()?;
    let planes = v["plane_nets"]
        .as_array()?
        .iter()
        .filter_map(|n| n.as_str().map(String::from))
        .collect();
    let mut groups = BTreeMap::new();
    for (net, g) in v["unconnected"].as_object()? {
        let gs: Vec<Vec<String>> = g
            .as_array()?
            .iter()
            .map(|grp| {
                grp.as_array()
                    .map(|a| {
                        a.iter()
                            .filter_map(|s| s.as_str().map(String::from))
                            .collect()
                    })
                    .unwrap_or_default()
            })
            .collect();
        groups.insert(net.clone(), gs);
    }
    Some((planes, groups))
}

/// Connectivity of one board, printed, and compared with the older
/// script's report when given.
fn connectivity(label: &str, l: &Loaded, olds: &[(PathBuf, OldReport)], tel: &mut Telemetry) {
    let board = &l.board;
    let conn = tel.time(Stage::Audit, || {
        ConnectivityReport::build(board, &ArcPolicy::default())
    });
    let c = conn.completeness(board);
    println!("== {label}: connectivity ==");
    println!(
        "nets with 2+ pads: {}; complete {}, partial {}, no track/via {}",
        c.multi_pad_nets, c.complete, c.partial, c.no_copper
    );
    println!(
        "connections needed: {} ({} on plane nets, {} signal)",
        c.connections, c.plane_connections, c.signal_connections
    );
    assert_eq!(c.complete + c.partial + c.no_copper, c.multi_pad_nets);
    assert_eq!(c.plane_connections + c.signal_connections, c.connections);
    assert_eq!(conn.connections(board).len(), c.connections);
    let joins = conn.plane_joins(board);
    for j in &joins {
        println!(
            "plane net {}: {} pieces, {} pads, {} joined, {} unjoined pads ({} SMD) in {} islands \
             ({} with escape copper), {} plane islands",
            board.net(j.net).name,
            j.pieces.len(),
            j.pads,
            j.joined_pads,
            j.unjoined_pads.len(),
            j.unjoined_smd_pads,
            j.unjoined_islands,
            j.unjoined_islands_with_copper,
            j.plane_islands
        );
        assert_eq!(j.joined_pads + j.unjoined_pads.len(), j.pads);
    }

    // Group pads by island, by name.
    let mut mine: BTreeMap<String, Vec<BTreeSet<String>>> = BTreeMap::new();
    for n in &conn.nets {
        if n.pad_count < 2 {
            continue;
        }
        let name = board.net(n.net).name.clone();
        let mut groups: Vec<BTreeSet<String>> = n
            .islands
            .iter()
            .filter(|i| i.has_pad())
            .map(|i| i.pads().map(|p| pad_name(board, p)).collect())
            .collect();
        groups.sort();
        mine.insert(name, groups);
    }
    // The older report that describes this board: the one whose per-net
    // group counts differ from these islands on the fewest nets.
    let differing = |old: &BTreeMap<String, Vec<Vec<String>>>| {
        mine.iter()
            .filter(|(n, g)| g.len() != old.get(*n).map_or(1, Vec::len))
            .count()
    };
    let Some((path, (old_planes, old))) = olds.iter().min_by_key(|(_, (_, o))| differing(o)) else {
        return;
    };
    println!(
        "older report {} ({} nets differ)",
        path.file_name().and_then(|f| f.to_str()).unwrap_or("?"),
        differing(old)
    );
    let old_sets: BTreeMap<&String, Vec<BTreeSet<String>>> = old
        .iter()
        .map(|(n, g)| (n, g.iter().map(|x| x.iter().cloned().collect()).collect()))
        .collect();
    let old_missing: usize = old.values().map(|g| g.len() - 1).sum();
    let old_plane_missing: usize = old
        .iter()
        .filter(|(n, _)| old_planes.contains(*n))
        .map(|(_, g)| g.len() - 1)
        .sum();
    println!(
        "older script: {} nets not complete, {} connections missing ({} on plane nets)",
        old.len(),
        old_missing,
        old_plane_missing
    );
    for p in old_planes {
        let g = old.get(p).map_or(0, |g| g.len().saturating_sub(1));
        let in_groups: usize = old
            .get(p)
            .map_or(0, |g| g.iter().skip(1).map(Vec::len).sum());
        println!("  older plane net {p}: {g} groups apart from the plane group, {in_groups} pads in them");
    }
    // Differences, net by net.
    let mut only_old = Vec::new();
    let mut only_mine = Vec::new();
    let mut differ = Vec::new();
    let mut delta_total: i64 = 0;
    for (name, groups) in &mine {
        let mine_k = groups.len();
        let old_k = old_sets.get(name).map_or(1, Vec::len);
        delta_total += mine_k as i64 - old_k as i64;
        match (mine_k > 1, old_k > 1) {
            (true, false) => only_mine.push((name.clone(), mine_k, old_k)),
            (false, true) => only_old.push((name.clone(), mine_k, old_k)),
            (true, true) if mine_k != old_k => differ.push((name.clone(), mine_k, old_k)),
            _ => {}
        }
    }
    let mut not_on_board = Vec::new();
    for name in old.keys() {
        if !mine.contains_key(name) {
            not_on_board.push(name.clone());
            delta_total -= old[name].len() as i64 - 1;
        }
    }
    println!(
        "per-net comparison: {} nets open only here, {} open only in the older report, {} with \
         different island counts, {} older nets without 2+ pads here; island delta {delta_total}",
        only_mine.len(),
        only_old.len(),
        differ.len(),
        not_on_board.len()
    );
    let show = |what: &str, v: &[(String, usize, usize)]| {
        for (n, a, b) in v.iter().take(12) {
            let detail = if mine[n].len() <= 4 {
                format!("{:?}", mine[n])
            } else {
                String::new()
            };
            let od = old_sets
                .get(n)
                .filter(|g| g.len() <= 4)
                .map(|g| format!("{g:?}"))
                .unwrap_or_default();
            println!("  {what} {n}: {a} islands here, {b} before {detail} | before {od}");
        }
    };
    show("open-here", &only_mine);
    show("open-before", &only_old);
    show("differs", &differ);
    for n in not_on_board.iter().take(12) {
        println!("  older-only {n}: {:?}", old[n]);
    }
    // Plane nets, pad by pad: pads outside the plane here against pads
    // outside the older report's largest (plane) group.
    for j in &joins {
        let name = &board.net(j.net).name;
        let here: BTreeSet<String> = j
            .unjoined_pads
            .iter()
            .map(|p| pad_name(board, *p))
            .collect();
        let before: BTreeSet<String> = old
            .get(name)
            .map(|g| g.iter().skip(1).flatten().cloned().collect())
            .unwrap_or_default();
        let only_here: Vec<&String> = here.difference(&before).collect();
        let only_before: Vec<&String> = before.difference(&here).collect();
        let by_ref = |v: &[&String]| {
            let mut m: BTreeMap<String, usize> = BTreeMap::new();
            for s in v {
                *m.entry(s.split('-').next().unwrap_or("").to_string())
                    .or_default() += 1;
            }
            m
        };
        println!(
            "  plane {name}: {} pads unjoined only here {:?}; {} only before {:?}",
            only_here.len(),
            by_ref(&only_here),
            only_before.len(),
            by_ref(&only_before)
        );
        if only_here.len() + only_before.len() <= 12 {
            println!("    here: {only_here:?} before: {only_before:?}");
        }
    }
    // A spanning tree over every pad of every 2+-pad net, ignoring all
    // existing copper.
    let naive: usize = conn
        .nets
        .iter()
        .filter(|n| n.pad_count >= 2)
        .map(|n| n.pad_count - 1)
        .sum();
    let naive_plane: usize = joins.iter().map(|j| j.pads - 1).sum();
    println!("pads-minus-one over 2+-pad nets: {naive} ({naive_plane} on plane nets)");
}

#[test]
fn ws_b_connectivity_matches_and_explains_the_old_counts() {
    let Some(dir) = fixture_dir() else { return };
    let mut tel = Telemetry::default();
    let base = load(&only_file(&dir, "kicad_pcb").unwrap(), &dir, &mut tel);
    let cand_dir = candidate_dir(&dir);
    // Older connectivity reports: any JSON beside the candidate with the
    // older script's shape.
    let olds: Vec<(PathBuf, OldReport)> = cand_dir
        .iter()
        .flat_map(|d| {
            std::fs::read_dir(d)
                .into_iter()
                .flatten()
                .filter_map(Result::ok)
        })
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|e| e.to_str()) == Some("json"))
        .filter_map(|p| old_groups(&p).map(|r| (p, r)))
        .collect();
    connectivity("current board", &base, &olds, &mut tel);
    if let Some(cd) = cand_dir {
        let cand = load(&only_file(&cd, "kicad_pcb").unwrap(), &dir, &mut tel);
        connectivity("routed candidate", &cand, &olds, &mut tel);
    }
    println!("telemetry: {:?}", tel.stages);
}

#[test]
fn ws_b_terminal_access_audit_of_the_ball_grids() {
    let Some(dir) = fixture_dir() else { return };
    let mut tel = Telemetry::default();
    let l = load(&only_file(&dir, "kicad_pcb").unwrap(), &dir, &mut tel);
    let board = &l.board;
    let fps = ball_grids(board, 2);
    tel.threads = rayon::current_num_threads() as u32;
    // What the rules give a track inside each field against one of its
    // balls and against a via there, and the same track outside.
    for &f in &fps {
        use konnect_route::api::ItemKind;
        use konnect_route::rules::{RuleSubject, RuleTable};
        let fp = &board.footprints[f.index()];
        let pad = &board.pads[fp.pad_range.0 as usize];
        let field = [f];
        let inside = RuleSubject::new(ItemKind::Track, pad.net.map(|n| NetId(n.0 + 1)))
            .with_courtyards(&field);
        let outside = RuleSubject::new(ItemKind::Track, pad.net.map(|n| NetId(n.0 + 1)));
        let ball = RuleSubject::new(ItemKind::Pad, pad.net)
            .with_footprint(f)
            .with_courtyards(&field)
            .with_local_clearance(pad.local_clearance);
        let via = RuleSubject::new(ItemKind::Via, pad.net).with_courtyards(&field);
        let l0 = board.layers.copper[0].id;
        println!(
            "{}: via rule {:?}; track width {:?}",
            fp.reference.as_deref().unwrap_or("?"),
            l.rules.via(&via),
            l.rules.track_width(&inside, l0)
        );
        println!(
            "  resolved: in-field {} nm, outside {} nm, track-via in-field {} nm",
            l.rules.clearance(&inside, &ball, l0),
            l.rules.clearance(&outside, &ball, l0),
            l.rules.clearance(&inside, &via, l0)
        );
    }
    let mut out = serde_json::Map::new();
    for (label, settings) in [
        ("preferred width, vias", AuditSettings::default()),
        (
            "minimum width, minimum vias",
            AuditSettings {
                width: SizeChoice::Minimum,
                via_size: SizeChoice::Minimum,
                ..Default::default()
            },
        ),
        (
            "preferred width, no new vias",
            AuditSettings {
                vias: false,
                ..Default::default()
            },
        ),
    ] {
        let start = Instant::now();
        let audit = tel.time(Stage::Audit, || {
            audit_terminal_access_with(board, &l.rules, &fps, &settings)
        });
        let ms = start.elapsed().as_millis();
        println!("== audit ({label}): {ms} ms, {} threads ==", tel.threads);
        for f in &audit.footprints {
            let fp = &board.footprints[f.footprint.index()];
            println!(
                "{}: {} terminals, {} unreachable ({} need a route), {} profiles, {} cells, {} exact reruns, {} ms",
                fp.reference.as_deref().unwrap_or("?"),
                f.terminals,
                f.unreachable,
                f.unreachable_needing_route,
                f.profiles,
                f.cells,
                f.exact_reruns,
                f.wall_ms
            );
        }
        let mut per_layer: BTreeMap<String, usize> = BTreeMap::new();
        for t in &audit.terminals {
            let names: Vec<&str> = t.reachable.iter().map(|l| board.layers.name(l)).collect();
            *per_layer.entry(names.join("+")).or_default() += 1;
        }
        println!("reachable layer sets: {per_layer:?}");
        let mut listed = Vec::new();
        for t in audit.unreachable() {
            let net = t.net.map_or("", |n| board.net(n).name.as_str());
            println!(
                "  unreachable {} ({net}) needs_route={} joined_to_plane={}: {}",
                pad_name(board, t.pad),
                t.needs_route,
                t.joined_to_plane,
                t.reason.as_deref().unwrap_or("")
            );
            listed.push(serde_json::json!({
                "pad": pad_name(board, t.pad),
                "net": net,
                "needs_route": t.needs_route,
                "joined_to_plane": t.joined_to_plane,
                "reason": t.reason,
            }));
        }
        out.insert(label.to_string(), serde_json::Value::Array(listed));
        // Every audited pad is reported once.
        let expected: usize = fps
            .iter()
            .map(|f| {
                let fp = &board.footprints[f.index()];
                board.pads[fp.pad_range.0 as usize..fp.pad_range.1 as usize]
                    .iter()
                    .filter(|p| p.net.is_some())
                    .count()
            })
            .sum();
        assert_eq!(audit.terminals.len(), expected);
    }
    let path = report_dir().join("audit-unreachable.json");
    std::fs::write(&path, serde_json::to_string_pretty(&out).unwrap()).unwrap();
    println!("report: {}", path.display());
    let _ = (NetId(0), ItemRef::Pad(PadId(0)), plane_nets(board).len());
}

#[test]
fn ws_b_plane_fanout_plan() {
    use konnect_route::api::{ItemKind, Ownership, TrackId, ViaId};
    use konnect_route::board::{Track, Via};
    use konnect_route::fanout::{
        plan_plane_vias_excluding, FanoutSettings, PlaneViaPlan, PlaneViaSite, ViaSizePolicy,
    };
    use konnect_route::rules::{RuleSubject, RuleTable};
    let Some(dir) = fixture_dir() else { return };
    let mut tel = Telemetry {
        threads: rayon::current_num_threads() as u32,
        ..Default::default()
    };
    let l = load(&only_file(&dir, "kicad_pcb").unwrap(), &dir, &mut tel);
    let board = &l.board;
    let conn = ConnectivityReport::build(board, &ArcPolicy::default());
    // The ball grids carry hand-drawn escapes; leave them alone.
    let exclude = ball_grids(board, 2);
    let summarize = |label: &str, p: &PlaneViaPlan, ms: u128| {
        println!(
            "== fanout ({label}): {ms} ms wall, {} threads ==",
            rayon::current_num_threads()
        );
        println!("stats: {:?}", p.stats);
        let mut per_net: BTreeMap<NetId, (usize, usize, usize)> = BTreeMap::new();
        for s in &p.sites {
            per_net.entry(s.net).or_default().0 += 1;
        }
        for d in &p.details {
            per_net.entry(d.net).or_default().1 += 1;
        }
        for (pad, _) in &p.skipped {
            if let Some(n) = board.pads[pad.index()].net {
                per_net.entry(n).or_default().2 += 1;
            }
        }
        for (n, (ok, failed, skipped)) in &per_net {
            println!(
                "  {}: {ok} vias, {failed} pads failed, {skipped} pads skipped",
                board.net(*n).name
            );
        }
        let mut why: BTreeMap<String, usize> = BTreeMap::new();
        for (_, f) in &p.failed {
            *why.entry(format!("{f:?}")).or_default() += 1;
        }
        let mut rej: BTreeMap<String, u32> = BTreeMap::new();
        for d in &p.details {
            for (k, v) in &d.rejections {
                *rej.entry(k.clone()).or_default() += v;
            }
        }
        let mut skip: BTreeMap<String, usize> = BTreeMap::new();
        for (_, s) in &p.skipped {
            let k = format!("{s:?}");
            let head = k.split_whitespace().next().unwrap_or("").to_string();
            *skip.entry(head).or_default() += 1;
        }
        println!("  failures: {why:?}");
        println!("  rejections of failed pads' candidates: {rej:?}");
        println!("  skipped: {skip:?}");
    };

    let mut runs = Vec::new();
    for (label, settings) in [
        ("net class via, then floor", FanoutSettings::default()),
        (
            "net class via only",
            FanoutSettings {
                via_size: ViaSizePolicy::NetClass,
                ..Default::default()
            },
        ),
        (
            "floor via only",
            FanoutSettings {
                via_size: ViaSizePolicy::Floor,
                ..Default::default()
            },
        ),
        (
            "default, sharing within 1 mm",
            FanoutSettings {
                share_within: 1_000_000,
                ..Default::default()
            },
        ),
        (
            "default, other nets' islands may be perforated",
            FanoutSettings {
                avoid_other_islands: false,
                ..Default::default()
            },
        ),
    ] {
        let start = Instant::now();
        let p = tel.time(Stage::Fanout, || {
            plan_plane_vias_excluding(board, &l.rules, &conn, &settings, &exclude)
        });
        let ms = start.elapsed().as_millis();
        summarize(label, &p, ms);
        runs.push((label, p));
    }
    let plan = &runs[0].1;
    // Determinism: one thread gives the same plan.
    let single = rayon::ThreadPoolBuilder::new()
        .num_threads(1)
        .build()
        .unwrap()
        .install(|| {
            let start = Instant::now();
            let p = plan_plane_vias_excluding(
                board,
                &l.rules,
                &conn,
                &FanoutSettings::default(),
                &exclude,
            );
            println!("single-thread fanout: {} ms", start.elapsed().as_millis());
            p
        });
    assert_eq!(
        single.sites, plan.sites,
        "1-thread and N-thread plans differ"
    );
    assert_eq!(single.failed, plan.failed);

    // Independent pairwise check of the chosen vias.
    let all = board.layers.all();
    let mut bad = 0usize;
    let mut sorted: Vec<&PlaneViaSite> = plan.sites.iter().collect();
    sorted.sort_by_key(|s| s.via.x);
    for (i, a) in sorted.iter().enumerate() {
        for b in sorted[i + 1..].iter() {
            if i64::from(b.via.x) - i64::from(a.via.x) > 3_000_000 {
                break;
            }
            let d2 = (a.via - b.via).length2();
            let h = i128::from(l.rules.hole_to_hole() + (a.via_drill + b.via_drill) / 2);
            if d2 < h * h {
                bad += 1;
            }
            if a.net != b.net {
                let sa = RuleSubject::new(ItemKind::Via, Some(a.net));
                let sb = RuleSubject::new(ItemKind::Via, Some(b.net));
                let c = all
                    .iter()
                    .map(|ly| l.rules.clearance(&sa, &sb, ly))
                    .max()
                    .unwrap();
                let need = i128::from(c + (a.via_diameter + b.via_diameter) / 2);
                if d2 < need * need {
                    bad += 1;
                }
            }
        }
    }
    println!(
        "pairwise via check: {bad} violations among {} vias",
        plan.sites.len()
    );
    assert_eq!(bad, 0);

    // Apply the plan to a copy and recompute connectivity.
    let mut joined = board.clone();
    for s in &plan.sites {
        if let Some((a, b, w)) = s.stub {
            joined.tracks.push(Track {
                id: TrackId(joined.tracks.len() as u32),
                net: Some(s.net),
                layer: s.layer,
                seg: konnect_geom::Segment::new(a, b, w),
                ownership: Ownership::Locked,
                uuid: None,
            });
        }
        joined.vias.push(Via {
            id: ViaId(joined.vias.len() as u32),
            net: Some(s.net),
            center: s.via,
            diameter: s.via_diameter,
            drill: s.via_drill,
            layers: joined.layers.all(),
            ownership: Ownership::Locked,
            uuid: None,
        });
    }
    let after = ConnectivityReport::build(&joined, &ArcPolicy::default());
    let before_c = conn.completeness(board);
    let after_c = after.completeness(&joined);
    println!(
        "after applying: connections {} -> {} ({} -> {} on plane nets)",
        before_c.connections,
        after_c.connections,
        before_c.plane_connections,
        after_c.plane_connections
    );
    for j in after.plane_joins(&joined) {
        println!(
            "  {}: {} pads still unjoined in {} islands",
            joined.net(j.net).name,
            j.unjoined_pads.len(),
            j.unjoined_islands
        );
    }
    let route = plan.route_plan(board).expect("plan validates");
    let path = report_dir().join("plane-vias.routing.json");
    std::fs::write(&path, serde_json::to_string(&route).unwrap()).unwrap();
    println!(
        "plan: {} vias, {} stubs, {} items -> {}",
        route.vias.len(),
        route.tracks.len(),
        route.item_count(),
        path.display()
    );
    let details = report_dir().join("plane-vias.failures.json");
    let failures: Vec<serde_json::Value> = plan
        .details
        .iter()
        .map(|d| {
            serde_json::json!({
                "pad": pad_name(board, d.pad),
                "net": board.net(d.net).name,
                "candidates": d.candidates,
                "legal": d.legal,
                "rejections": d.rejections,
            })
        })
        .collect();
    std::fs::write(&details, serde_json::to_string_pretty(&failures).unwrap()).unwrap();
    println!("failures: {}", details.display());
    println!("telemetry: {:?}", tel.stages);
}

#[test]
fn ws_b_pin_access_for_every_pad() {
    use konnect_route::access::{compute_pin_access_with, AccessSettings};
    use konnect_route::api::ProfileId;
    use konnect_route::rules::RuleTable;
    let Some(dir) = fixture_dir() else { return };
    let mut tel = Telemetry {
        threads: rayon::current_num_threads() as u32,
        ..Default::default()
    };
    let l = load(&only_file(&dir, "kicad_pcb").unwrap(), &dir, &mut tel);
    let board = &l.board;
    // One run per net class: the profile is the class.
    let mut by_class: BTreeMap<u16, Vec<PadId>> = BTreeMap::new();
    for p in board.pads.iter().filter(|p| p.net.is_some()) {
        by_class
            .entry(l.rules.class_of(p.net).0)
            .or_default()
            .push(p.id);
    }
    let start = Instant::now();
    let mut total = konnect_route::access::AccessStats::default();
    let mut none: Vec<PadId> = Vec::new();
    let mut points = 0usize;
    let mut first: Vec<konnect_route::access::PadAccess> = Vec::new();
    for (class, pads) in &by_class {
        let (out, stats) = tel.time(Stage::Access, || {
            compute_pin_access_with(
                board,
                &l.rules,
                pads,
                ProfileId(*class),
                &AccessSettings::default(),
            )
        });
        assert_eq!(out.len(), pads.len());
        total.pads += stats.pads;
        total.classes += stats.classes;
        total.class_checks += stats.class_checks;
        total.instance_checks += stats.instance_checks;
        total.escape_points += stats.escape_points;
        total.no_access += stats.no_access;
        for a in &out {
            points += a.points.len();
            if a.points.is_empty() {
                none.push(a.pad);
            }
        }
        if first.is_empty() {
            first = out;
        }
    }
    let ms = start.elapsed().as_millis();
    println!("== pin access: {ms} ms wall, {} threads ==", tel.threads);
    println!(
        "{} pads, {} classes ({:.1} pads per class), {} class checks, {} instance checks, {} points, {} escape-end points",
        total.pads,
        total.classes,
        total.pads as f64 / total.classes.max(1) as f64,
        total.class_checks,
        total.instance_checks,
        points,
        total.escape_points
    );
    let mut by_ref: BTreeMap<String, usize> = BTreeMap::new();
    for p in &none {
        let name = pad_name(board, *p);
        *by_ref
            .entry(name.split('-').next().unwrap_or("").to_string())
            .or_default() += 1;
    }
    println!("pads with no access point: {} {by_ref:?}", none.len());
    let joined: usize = {
        let conn = ConnectivityReport::build(board, &ArcPolicy::default());
        let isl = conn.island_of_pads(board);
        none.iter()
            .filter(|p| {
                let n = board.pads[p.index()].net.unwrap();
                isl[p.index()].is_some_and(|i| {
                    conn.nets[n.index()].islands[i as usize]
                        .items
                        .iter()
                        .any(|it| matches!(it, ItemRef::Zone(_)))
                })
            })
            .count()
    };
    println!("  of which already joined to a plane: {joined}");
    println!("telemetry: {:?}", tel.stages);
}
