//! G-Oracle: the internal checker against a saved KiCad DRC report.
//!
//! The board, its project files and the DRC report live outside the
//! repository. `KONNECT_DB1_FIXTURE_DIR` names the project directory (the
//! `.kicad_pro` and `.kicad_dru` are found there by extension). The routed
//! candidate defaults to the only `.kicad_pcb` in the first subdirectory
//! that holds one, its DRC report to the `*refilled*.json` there, and a
//! `.dsn` there (exported from KiCad's effective rules) cross-checks the
//! netclass resolution; `KONNECT_ORACLE_PCB`, `KONNECT_ORACLE_DRC` and
//! `KONNECT_ORACLE_DSN` override them. Nothing about the board is
//! committed: this file holds no names, counts or values from it.
//!
//! How the comparison is made:
//!
//! - the checker runs KiCad-equivalent ([`CheckOptions::kicad_equivalent`],
//!   KiCad's 0.5 µm DRC epsilon) with zone fills off, because the saved
//!   fills predate the routed copper and KiCad refilled before its DRC;
//! - each KiCad error must match a finding of the same category naming the
//!   same items (by UUID); KiCad reports a bend once from each track, so two
//!   of its errors may match one finding;
//! - a finding KiCad did not report is explained when it is a clearance
//!   error on a track or via KiCad already reported an error for (KiCad
//!   keeps the first clearance error per track unless "report all track
//!   errors" is on); anything else counts as a false positive.
//!
//! The gate: zero misses in the gated categories and false positives at
//! most 5% of everything flagged.
//!
//! ```text
//! KONNECT_DB1_FIXTURE_DIR=/path/to/fixture cargo test --release //!     -p konnect-route --test check_oracle -- --nocapture
//! ```
//!
//! Without the variable, or without the files, the tests print why and
//! pass.

use konnect_geom::{closest, Shape};
use konnect_route::api::ItemRef;
use konnect_route::board::RouteBoard;
use konnect_route::check::{BoardChecker, CheckOptions, CheckedItem, ViolationKind};
use konnect_route::telemetry::Telemetry;
use konnect_route::{load_board_file, LoadOptions, ProjectRuleTable, RuleTable};
use serde_json::Value;
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

fn report_file(dir: &Path) -> Option<PathBuf> {
    let mut found = std::fs::read_dir(dir)
        .ok()?
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| {
            p.extension().and_then(|e| e.to_str()) == Some("json")
                && p.file_name()
                    .and_then(|n| n.to_str())
                    .is_some_and(|n| n.contains("refilled"))
        });
    let first = found.next()?;
    found.next().is_none().then_some(first)
}

/// KiCad DRC rule keys the internal checker covers, by checker kind.
fn kicad_rule(kind: ViolationKind) -> &'static str {
    match kind {
        ViolationKind::Clearance => "clearance",
        ViolationKind::Short => "shorting_items",
        ViolationKind::TrackWidth => "track_width",
        ViolationKind::ViaSize => "via_size",
        ViolationKind::HoleClearance => "hole_clearance",
        ViolationKind::HoleToHole => "hole_to_hole",
        ViolationKind::EdgeClearance => "copper_edge_clearance",
        ViolationKind::TrackAngle => "track_angle",
        ViolationKind::SegmentLength => "track_segment_length",
        ViolationKind::Keepout => "keepout",
    }
}

/// The categories the gate requires with zero misses.
const GATED: [&str; 4] = [
    "clearance",
    "track_angle",
    "track_segment_length",
    "hole_clearance",
];

/// Rule keys whose KiCad violations the checker is expected to cover.
fn covered(rule: &str) -> bool {
    matches!(
        rule,
        "clearance"
            | "shorting_items"
            | "track_width"
            | "via_diameter"
            | "annular_width"
            | "hole_size"
            | "hole_clearance"
            | "hole_to_hole"
            | "drilled_holes_colocated"
            | "copper_edge_clearance"
            | "track_angle"
            | "track_segment_length"
            | "items_not_allowed"
    )
}

/// Our category for a KiCad rule key.
fn category(rule: &str) -> &str {
    match rule {
        "via_diameter" | "annular_width" | "hole_size" => "via_size",
        "drilled_holes_colocated" => "hole_to_hole",
        "items_not_allowed" => "keepout",
        r => r,
    }
}

fn uuid_of(board: &RouteBoard, item: CheckedItem) -> Option<String> {
    match item {
        CheckedItem::Board { item } => match item {
            ItemRef::Pad(id) => board.pads[id.index()].uuid.clone(),
            ItemRef::Track(id) => board.tracks[id.index()].uuid.clone(),
            ItemRef::Arc(id) => board.arcs[id.index()].uuid.clone(),
            ItemRef::Via(id) => board.vias[id.index()].uuid.clone(),
            ItemRef::Zone(_) => None,
        },
        _ => None,
    }
}

fn describe(board: &RouteBoard, item: CheckedItem) -> String {
    let net = |n: Option<konnect_route::api::NetId>| {
        n.map_or("<no net>".to_string(), |n| board.net(n).name.clone())
    };
    match item {
        CheckedItem::Board { item } => match item {
            ItemRef::Pad(id) => {
                let p = &board.pads[id.index()];
                let fp = &board.footprints[p.footprint.index()];
                format!(
                    "pad {}.{} [{}]{}",
                    fp.reference.as_deref().unwrap_or("?"),
                    p.number,
                    net(p.net),
                    if p.approximated {
                        " (approximated)"
                    } else {
                        ""
                    }
                )
            }
            ItemRef::Track(id) => {
                let t = &board.tracks[id.index()];
                format!(
                    "track [{}] on {} w {}",
                    net(t.net),
                    board.layers.name(t.layer),
                    t.seg.width
                )
            }
            ItemRef::Arc(id) => format!("arc [{}]", net(board.arcs[id.index()].net)),
            ItemRef::Via(id) => format!("via [{}]", net(board.vias[id.index()].net)),
            ItemRef::Zone(id) => {
                let z = &board.zones[id.index()];
                format!("zone [{}] on {}", net(z.net), board.layers.name(z.layer))
            }
        },
        CheckedItem::Edge => "board edge".to_string(),
        other => format!("{other:?}"),
    }
}

fn item_shapes(board: &RouteBoard, c: CheckedItem) -> Vec<Shape> {
    match c {
        CheckedItem::Board { item } => match item {
            ItemRef::Pad(id) => board.pads[id.index()].shapes.clone(),
            ItemRef::Track(id) => vec![Shape::Segment(board.tracks[id.index()].seg)],
            ItemRef::Via(id) => vec![board.vias[id.index()].shape()],
            ItemRef::Arc(id) => board.arcs[id.index()]
                .segments(&Default::default())
                .into_iter()
                .map(Shape::Segment)
                .collect(),
            ItemRef::Zone(id) => vec![Shape::Polygon(board.zones[id.index()].region.clone())],
        },
        _ => Vec::new(),
    }
}

/// An approximate gap between two board items (nm), for explaining a
/// finding.
fn gap_between(board: &RouteBoard, a: CheckedItem, b: CheckedItem) -> Option<f64> {
    let shapes = |c: CheckedItem| -> Vec<Shape> {
        match c {
            CheckedItem::Board { item } => match item {
                ItemRef::Pad(id) => board.pads[id.index()].shapes.clone(),
                ItemRef::Track(id) => vec![Shape::Segment(board.tracks[id.index()].seg)],
                ItemRef::Via(id) => vec![board.vias[id.index()].shape()],
                ItemRef::Arc(id) => board.arcs[id.index()]
                    .segments(&Default::default())
                    .into_iter()
                    .map(Shape::Segment)
                    .collect(),
                ItemRef::Zone(id) => vec![Shape::Polygon(board.zones[id.index()].region.clone())],
            },
            _ => Vec::new(),
        }
    };
    let (sa, sb) = (shapes(a), shapes(b));
    sa.iter()
        .flat_map(|x| sb.iter().map(move |y| closest(x, y).gap_nm))
        .reduce(f64::min)
}

/// Net → (track width µm, clearance µm) from a DSN made from KiCad's IPC
/// effective netclasses: `(class NAME net… (circuit …) (rule (width W)
/// (clearance C) …))`.
fn dsn_net_rules(text: &str) -> BTreeMap<String, (f64, f64)> {
    let mut out = BTreeMap::new();
    let mut rest = text;
    while let Some(i) = rest.find("(class ") {
        rest = &rest[i + 7..];
        let end = rest.find("(rule").unwrap_or(rest.len());
        let head = &rest[..end];
        let names_end = head.find("(circuit").unwrap_or(head.len());
        let names = tokens(&head[..names_end]);
        let rule = &rest[end..rest.len().min(end + 400)];
        let num = |key: &str| -> Option<f64> {
            let j = rule.find(key)? + key.len();
            rule[j..]
                .trim_start()
                .split(|c: char| c == ')' || c.is_whitespace())
                .next()?
                .parse()
                .ok()
        };
        if let (Some(w), Some(c)) = (num("(width"), num("(clearance")) {
            for n in names.into_iter().skip(1) {
                out.insert(n, (w, c));
            }
        }
    }
    out
}

/// Whitespace-separated tokens, honouring double quotes.
fn tokens(s: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut chars = s.chars().peekable();
    while let Some(&c) = chars.peek() {
        if c.is_whitespace() {
            chars.next();
        } else if c == '"' {
            chars.next();
            let mut t = String::new();
            for c in chars.by_ref() {
                if c == '"' {
                    break;
                }
                t.push(c);
            }
            out.push(t);
        } else {
            let mut t = String::new();
            while let Some(&c) = chars.peek() {
                if c.is_whitespace() {
                    break;
                }
                t.push(c);
                chars.next();
            }
            out.push(t);
        }
    }
    out
}

/// Items touching `at` on `layer`: what a track junction also meets.
fn items_at(
    board: &RouteBoard,
    at: konnect_geom::Point,
    layer: konnect_route::api::LayerId,
) -> String {
    use konnect_geom::point_shape_distance_cmp;
    use std::cmp::Ordering;
    let mut out = Vec::new();
    let tracks = board
        .tracks
        .iter()
        .filter(|t| t.layer == layer && (t.seg.a == at || t.seg.b == at))
        .count();
    out.push(format!("{tracks} track ends"));
    for v in &board.vias {
        if v.layers.contains(layer)
            && point_shape_distance_cmp(at, 0, &v.shape(), 0) != Ordering::Greater
        {
            out.push(format!(
                "via{}",
                if v.center == at { " centre" } else { "" }
            ));
        }
    }
    for p in &board.pads {
        if p.layers.contains(layer)
            && p.shapes
                .iter()
                .any(|s| point_shape_distance_cmp(at, 0, s, 0) != Ordering::Greater)
        {
            out.push(format!(
                "pad{}",
                if p.center == at { " centre" } else { "" }
            ));
        }
    }
    let mid = board.tracks.iter().filter(|t| {
        t.layer == layer
            && t.seg.a != at
            && t.seg.b != at
            && konnect_geom::point_segment_distance_cmp(at, t.seg.a, t.seg.b, 1) == Ordering::Less
    });
    let n = mid.count();
    if n > 0 {
        out.push(format!("{n} tracks passing through"));
    }
    out.join(", ")
}

/// The "actual X" (mm or degrees) KiCad prints in a violation's
/// description.
fn kicad_actual(description: &str) -> Option<f64> {
    let i = description.find("actual ")? + 7;
    let rest = &description[i..];
    let end = rest
        .find(|c: char| !(c.is_ascii_digit() || c == '.'))
        .unwrap_or(rest.len());
    rest[..end].parse().ok()
}

/// The inside angle of a track-angle finding's bend, degrees.
fn bend_deg(board: &RouteBoard, v: &konnect_route::check::Violation) -> Option<f64> {
    let seg = |c: CheckedItem| match c {
        CheckedItem::Board {
            item: ItemRef::Track(id),
        } => Some(board.tracks[id.index()].seg),
        _ => None,
    };
    let (a, b) = (seg(v.a)?, seg(v.b?)?);
    let away = |s: konnect_geom::Segment| {
        let far = if s.a == v.at { s.b } else { s.a };
        let d = far - v.at;
        (d.x as f64, d.y as f64)
    };
    let (u, w) = (away(a), away(b));
    let dot = u.0 * w.0 + u.1 * w.1;
    let cross = u.0 * w.1 - u.1 * w.0;
    Some(cross.abs().atan2(dot).to_degrees())
}

struct KicadViolation {
    rule: String,
    description: String,
    uuids: BTreeSet<String>,
    pos: (f64, f64),
}

fn kicad_errors(report: &Value) -> Vec<KicadViolation> {
    report["violations"]
        .as_array()
        .map(Vec::as_slice)
        .unwrap_or(&[])
        .iter()
        .filter(|v| v["severity"].as_str() == Some("error"))
        .map(|v| KicadViolation {
            rule: v["rule"].as_str().unwrap_or("").to_string(),
            description: v["description"].as_str().unwrap_or("").to_string(),
            uuids: v["items"]
                .as_array()
                .map(Vec::as_slice)
                .unwrap_or(&[])
                .iter()
                .filter_map(|i| i["uuid"].as_str().map(str::to_string))
                .collect(),
            pos: (
                v["pos"]["x"].as_f64().unwrap_or(f64::NAN),
                v["pos"]["y"].as_f64().unwrap_or(f64::NAN),
            ),
        })
        .collect()
}

#[test]
fn internal_checker_reproduces_the_saved_kicad_drc() {
    let Some(dir) = std::env::var_os("KONNECT_DB1_FIXTURE_DIR").map(PathBuf::from) else {
        eprintln!("skipping: KONNECT_DB1_FIXTURE_DIR is not set");
        return;
    };
    // The routed candidate lives in the first subdirectory (by name) that
    // holds a single board.
    let routed_dir = {
        let mut subdirs: Vec<PathBuf> = std::fs::read_dir(&dir)
            .map(|r| {
                r.filter_map(Result::ok)
                    .map(|e| e.path())
                    .filter(|p| p.is_dir())
                    .collect()
            })
            .unwrap_or_default();
        subdirs.sort();
        subdirs
            .into_iter()
            .find(|d| only_file(d, "kicad_pcb").is_some())
            .unwrap_or_else(|| dir.clone())
    };
    let pcb = std::env::var_os("KONNECT_ORACLE_PCB")
        .map(PathBuf::from)
        .or_else(|| only_file(&routed_dir, "kicad_pcb"));
    let drc = std::env::var_os("KONNECT_ORACLE_DRC")
        .map(PathBuf::from)
        .or_else(|| report_file(&routed_dir));
    let pro = only_file(&dir, "kicad_pro");
    let (Some(pcb), Some(drc), Some(pro)) = (pcb, drc, pro) else {
        eprintln!("skipping: the candidate board, DRC report or project file is missing");
        return;
    };
    let dru = only_file(&dir, "kicad_dru");
    // A DSN exported from KiCad's IPC effective rules, when present, is
    // KiCad's own netclass resolution; disagreements are reported and
    // KiCad's answer pinned.
    let dsn = std::env::var_os("KONNECT_ORACLE_DSN")
        .map(PathBuf::from)
        .or_else(|| {
            let mut found: Vec<PathBuf> = std::fs::read_dir(&routed_dir)
                .ok()?
                .filter_map(Result::ok)
                .map(|e| e.path())
                .filter(|p| p.extension().and_then(|e| e.to_str()) == Some("dsn"))
                .collect();
            found.sort();
            found.into_iter().next()
        });

    let t0 = Instant::now();
    let (mut board, load) = load_board_file(&pcb, &LoadOptions::default()).expect("board loads");
    let load_ms = t0.elapsed().as_secs_f64() * 1e3;
    assert_eq!(board.validate(), Ok(()));
    let pro_text = std::fs::read_to_string(&pro).expect("project reads");
    let dru_text = dru.map(|p| std::fs::read_to_string(p).expect("rules read"));
    let t1 = Instant::now();
    let mut rules = ProjectRuleTable::from_project(&pro_text, dru_text.as_deref(), &board)
        .expect("rules compile");
    let rules_ms = t1.elapsed().as_secs_f64() * 1e3;

    if let Some(dsn) = &dsn {
        let text = std::fs::read_to_string(dsn).expect("DSN reads");
        let kicad = dsn_net_rules(&text);
        let mut disagree = 0;
        let mut pinned = 0;
        for net in &board.nets {
            let Some(&(w, c)) = kicad.get(&net.name) else {
                continue;
            };
            let ours = rules.class(rules.class_of(Some(net.id))).clone();
            let um = |nm: i32| f64::from(nm) / 1000.0;
            if (um(ours.track_width) - w).abs() < 0.5 && (um(ours.clearance) - c).abs() < 0.5 {
                continue;
            }
            disagree += 1;
            let candidates: Vec<String> = rules
                .classes()
                .iter()
                .filter(|k| {
                    (um(k.track_width) - w).abs() < 0.5 && (um(k.clearance) - c).abs() < 0.5
                })
                .map(|k| k.name.clone())
                .collect();
            if let [only] = candidates.as_slice() {
                println!(
                    "netclass: KiCad puts {} in {only}, the project patterns give {}",
                    net.name, ours.name
                );
                if rules.pin_net_class(net.id, only) {
                    pinned += 1;
                }
            } else {
                println!(
                    "netclass: {} disagrees and is ambiguous: {candidates:?}",
                    net.name
                );
            }
        }
        println!(
            "netclass cross-check against KiCad's effective rules: {disagree} disagree, {pinned} pinned"
        );
    }
    rules.assign_classes(&mut board);
    println!(
        "board: {} tracks, {} arcs, {} vias, {} pads, {} zone pieces; load {load_ms:.0} ms, rules {rules_ms:.1} ms",
        board.tracks.len(),
        board.arcs.len(),
        board.vias.len(),
        board.pads.len(),
        board.zones.len()
    );
    println!("load notes: {:?}, skipped: {:?}", load.notes, load.skipped);

    // The candidate's zone fills predate its routed copper; KiCad refilled
    // before the saved DRC, so fills are left out here.
    let strict_options = CheckOptions {
        check_zone_fills: false,
        ..CheckOptions::default()
    };
    let gate_options = strict_options.kicad_equivalent();

    let threads = rayon::current_num_threads();
    let t2 = Instant::now();
    let checker = BoardChecker::new(&board, &rules, gate_options);
    let build_ms = t2.elapsed().as_secs_f64() * 1e3;
    let mut telemetry = Telemetry::default();
    let report = checker.check_board_timed(&mut telemetry);
    let check_ms = telemetry.stage_us(konnect_route::api::Stage::Check) as f64 / 1e3;
    println!(
        "checker: build {build_ms:.0} ms, check {check_ms:.0} ms on {threads} threads; stats {:?}",
        report.stats
    );
    println!("telemetry counters: {:?}", telemetry.counters);

    // Determinism: one thread gives the same report.
    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(1)
        .build()
        .unwrap();
    let (single, single_build, single_check) = pool.install(|| {
        let t = Instant::now();
        let c = BoardChecker::new(&board, &rules, gate_options);
        let b = t.elapsed().as_secs_f64() * 1e3;
        let t = Instant::now();
        let r = c.check_board();
        (r, b, t.elapsed().as_secs_f64() * 1e3)
    });
    println!("1 thread: build {single_build:.0} ms, check {single_check:.0} ms");
    assert_eq!(
        single, report,
        "1-thread and {threads}-thread reports differ"
    );

    let strict = BoardChecker::new(&board, &rules, strict_options).check_board();
    let margin = BoardChecker::new(&board, &rules, strict_options.with_safety_margin_nm(2_000))
        .check_board();
    let with_fills =
        BoardChecker::new(&board, &rules, CheckOptions::default().kicad_equivalent()).check_board();
    println!("strict (no epsilon): {:?}", strict.counts());
    println!("strict + 2 um margin: {:?}", margin.counts());
    println!(
        "KiCad-equivalent with the stale fills checked: {:?}",
        with_fills.counts()
    );

    // ─── Compare ─────────────────────────────────────────────────────────
    let oracle: Value =
        serde_json::from_str(&std::fs::read_to_string(&drc).expect("DRC report reads"))
            .expect("DRC report is JSON");
    let errors = kicad_errors(&oracle);
    let mut by_rule: BTreeMap<String, usize> = BTreeMap::new();
    for e in &errors {
        *by_rule.entry(e.rule.clone()).or_default() += 1;
    }
    println!("KiCad errors by rule: {by_rule:?}");
    println!("checker findings by kind: {:?}", report.counts());

    let ours: Vec<(String, BTreeSet<String>)> = report
        .violations
        .iter()
        .map(|v| {
            let mut ids = BTreeSet::new();
            ids.extend(uuid_of(&board, v.a));
            if let Some(b) = v.b {
                ids.extend(uuid_of(&board, b));
            }
            (kicad_rule(v.kind).to_string(), ids)
        })
        .collect();
    // KiCad reports some findings twice (a bend from each of its tracks);
    // each of its reports must match one of ours, and ours may match many.
    let mut used = vec![false; ours.len()];
    let mut table: BTreeMap<String, (usize, usize, usize, f64)> = BTreeMap::new();
    let mut misses = Vec::new();
    for e in errors.iter().filter(|e| covered(&e.rule)) {
        let cat = category(&e.rule).to_string();
        let hit = ours.iter().position(|(r, ids)| {
            (r == &cat || (cat == "clearance" && r == "shorting_items")) && ids == &e.uuids
        });
        let row = table.entry(cat.clone()).or_default();
        row.0 += 1;
        match hit {
            Some(i) => {
                row.1 += 1;
                if !used[i] {
                    used[i] = true;
                    row.2 += 1;
                }
                // KiCad's own measurement against ours: lengths in mm
                // (shown in um), angles in degrees.
                let v = &report.violations[i];
                let ours = match v.kind {
                    ViolationKind::SegmentLength => match v.a {
                        CheckedItem::Board {
                            item: ItemRef::Track(id),
                        } => Some((board.tracks[id.index()].seg.length() as f64 / 1e6, 1e3)),
                        _ => None,
                    },
                    ViolationKind::TrackAngle => bend_deg(&board, v).map(|d| (d, 1.0)),
                    _ => {
                        v.b.and_then(|b| gap_between(&board, v.a, b))
                            .map(|g| (g / 1e6, 1e3))
                    }
                };
                if let (Some(k), Some((o, scale))) = (kicad_actual(&e.description), ours) {
                    row.3 = row.3.max((k - o).abs() * scale);
                }
            }
            None => misses.push(e),
        }
    }
    // Tracks KiCad reported a clearance error for: it stops at the first
    // error per track unless "report all track errors" is on.
    let reported_tracks: BTreeSet<String> = errors
        .iter()
        .filter(|e| e.rule == "clearance")
        .flat_map(|e| e.uuids.iter().cloned())
        .collect();

    let mut explained: BTreeMap<&str, usize> = BTreeMap::new();
    let mut fps = Vec::new();
    for (i, v) in report.violations.iter().enumerate() {
        if used[i] {
            continue;
        }
        // KiCad's track loop covers vias too (a via is a track there).
        let track_uuid = |c: CheckedItem| match c {
            CheckedItem::Board {
                item: ItemRef::Track(_) | ItemRef::Via(_),
            } => uuid_of(&board, c),
            _ => None,
        };
        let suppressed = matches!(v.kind, ViolationKind::Clearance | ViolationKind::Short)
            && [Some(v.a), v.b]
                .into_iter()
                .flatten()
                .filter_map(track_uuid)
                .any(|u| reported_tracks.contains(&u));
        if suppressed {
            *explained
                .entry(
                    "true violation on a track or via KiCad already reported (it keeps the first clearance error per track or via)",
                )
                .or_default() += 1;
            continue;
        }
        let gap = v.b.and_then(|b| gap_between(&board, v.a, b));
        let extra = match v.kind {
            ViolationKind::TrackAngle => v
                .layer
                .map(|l| format!("; at the junction: {}", items_at(&board, v.at, l)))
                .unwrap_or_default(),
            ViolationKind::Clearance => {
                let ctx = |c: CheckedItem| match c {
                    CheckedItem::Board { item } => {
                        let (courts, _) = checker.subject_context(item);
                        let refs: Vec<String> = courts
                            .iter()
                            .map(|f| {
                                let fp = &board.footprints[f.index()];
                                let shapes = item_shapes(&board, c);
                                let gap = fp
                                    .courtyards
                                    .iter()
                                    .flat_map(|ct| {
                                        let court = Shape::Polygon(ct.region.clone());
                                        shapes
                                            .iter()
                                            .map(move |s| closest(s, &court).gap_nm)
                                            .collect::<Vec<_>>()
                                    })
                                    .fold(f64::INFINITY, f64::min);
                                format!("{}(gap {gap:.0})", fp.reference.as_deref().unwrap_or("?"))
                            })
                            .collect();
                        format!(
                            "{} class {}",
                            refs.join(" "),
                            rules.class(rules.class_of(board.net_of(item))).name
                        )
                    }
                    _ => String::new(),
                };
                format!(
                    "; courtyards a: [{}], b: [{}]",
                    ctx(v.a),
                    v.b.map(ctx).unwrap_or_default()
                )
            }
            _ => String::new(),
        };
        fps.push(format!(
            "FP {:?}: {} / {} on {:?} at {:?}; required {} nm, gap {:?} nm{extra}",
            v.kind,
            describe(&board, v.a),
            v.b.map_or("-".to_string(), |b| describe(&board, b)),
            v.layer.map(|l| board.layers.name(l).to_string()),
            v.at.to_mm(),
            v.required,
            gap.map(|g| g.round()),
        ));
    }

    println!();
    println!(
        "| category | KiCad errors | matched | distinct findings | max abs(KiCad actual - ours), um or deg |"
    );
    println!("|---|---|---|---|---|");
    for (cat, (n, m, f, d)) in &table {
        println!("| {cat} | {n} | {m} | {f} | {d:.2} |");
    }
    for e in &misses {
        println!("MISS {}: {} at {:?}", e.rule, e.description, e.pos);
        // Where the missed items end, and what else is there.
        for t in board
            .tracks
            .iter()
            .filter(|t| t.uuid.as_ref().is_some_and(|u| e.uuids.contains(u)))
        {
            for end in [t.seg.a, t.seg.b] {
                println!(
                    "    end {:?} on {}: {}",
                    end.to_mm(),
                    board.layers.name(t.layer),
                    items_at(&board, end, t.layer)
                );
            }
        }
    }
    for (why, n) in &explained {
        println!("explained, not false: {n} x {why}");
    }
    for f in &fps {
        println!("{f}");
    }
    let strict_extra: Vec<String> = strict
        .violations
        .iter()
        .filter(|v| !report.violations.contains(v))
        .map(|v| {
            let gap = v.b.and_then(|b| gap_between(&board, v.a, b));
            format!(
                "{:?}: {} / {}; required {} nm, gap {:?} nm",
                v.kind,
                describe(&board, v.a),
                v.b.map_or("-".to_string(), |b| describe(&board, b)),
                v.required,
                gap.map(|g| g.round())
            )
        })
        .collect();
    println!(
        "strict mode flags {} more (gaps under the rule by less than KiCad's epsilon):",
        strict_extra.len()
    );
    for s in &strict_extra {
        println!("  {s}");
    }

    let flagged = report.violations.len();
    println!(
        "flagged {flagged}, matching KiCad {}, explained {}, false positives {} ({:.1}%), misses {}",
        used.iter().filter(|u| **u).count(),
        explained.values().sum::<usize>(),
        fps.len(),
        100.0 * fps.len() as f64 / flagged.max(1) as f64,
        misses.len()
    );

    let gated_misses = misses
        .iter()
        .filter(|e| GATED.contains(&category(&e.rule)))
        .count();
    assert_eq!(
        gated_misses, 0,
        "checker missed KiCad errors in gated categories"
    );
    assert!(
        fps.len() * 20 <= flagged,
        "false positives exceed 5% of flagged items"
    );
}

/// The project board itself (fresh fills, fixed copper only): a timing and
/// sanity run of the zone checks on real plane pieces. It prints what it
/// finds and asserts only determinism.
#[test]
fn internal_checker_on_the_project_board() {
    let Some(dir) = std::env::var_os("KONNECT_DB1_FIXTURE_DIR").map(PathBuf::from) else {
        eprintln!("skipping: KONNECT_DB1_FIXTURE_DIR is not set");
        return;
    };
    let (Some(pcb), Some(pro)) = (only_file(&dir, "kicad_pcb"), only_file(&dir, "kicad_pro"))
    else {
        eprintln!("skipping: no single .kicad_pcb and .kicad_pro in the fixture directory");
        return;
    };
    let (mut board, _) = load_board_file(&pcb, &LoadOptions::default()).expect("board loads");
    let pro_text = std::fs::read_to_string(&pro).expect("project reads");
    let dru_text = only_file(&dir, "kicad_dru").map(|p| std::fs::read_to_string(p).unwrap());
    let rules = ProjectRuleTable::from_project(&pro_text, dru_text.as_deref(), &board)
        .expect("rules compile");
    rules.assign_classes(&mut board);
    let options = CheckOptions::default().kicad_equivalent();
    let t = Instant::now();
    let checker = BoardChecker::new(&board, &rules, options);
    let build_ms = t.elapsed().as_secs_f64() * 1e3;
    let t = Instant::now();
    let report = checker.check_board();
    let check_ms = t.elapsed().as_secs_f64() * 1e3;
    let edges: usize = board
        .zones
        .iter()
        .map(|z| z.region.rings().map(<[_]>::len).sum::<usize>())
        .sum();
    println!(
        "project board: {} zone pieces ({edges} fill vertices); build {build_ms:.0} ms, check {check_ms:.0} ms; {:?}; stats {:?}",
        board.zones.len(),
        report.counts(),
        report.stats
    );
    for v in report.violations.iter().take(20) {
        println!(
            "  {:?}: {} / {} at {:?}, required {}",
            v.kind,
            describe(&board, v.a),
            v.b.map_or("-".to_string(), |b| describe(&board, b)),
            v.at.to_mm(),
            v.required
        );
    }
    let single = rayon::ThreadPoolBuilder::new()
        .num_threads(1)
        .build()
        .unwrap()
        .install(|| BoardChecker::new(&board, &rules, options).check_board());
    assert_eq!(single, report);
}

/// The routed path on real data: the candidate's unlocked copper is turned
/// into routed copper over a board holding only the locked copper, and
/// `check_routed` must find exactly what the whole-board check finds on
/// pairs involving that copper.
#[test]
fn routed_check_agrees_with_the_board_check() {
    use konnect_route::api::{Generation, Ownership};
    use konnect_route::plan::{RoutedCopper, RoutedTrack, RoutedVia};
    let Some(dir) = std::env::var_os("KONNECT_DB1_FIXTURE_DIR").map(PathBuf::from) else {
        eprintln!("skipping: KONNECT_DB1_FIXTURE_DIR is not set");
        return;
    };
    let mut subdirs: Vec<PathBuf> = std::fs::read_dir(&dir)
        .map(|r| {
            r.filter_map(Result::ok)
                .map(|e| e.path())
                .filter(|p| p.is_dir())
                .collect()
        })
        .unwrap_or_default();
    subdirs.sort();
    let pcb = std::env::var_os("KONNECT_ORACLE_PCB")
        .map(PathBuf::from)
        .or_else(|| subdirs.iter().find_map(|d| only_file(d, "kicad_pcb")));
    let (Some(pcb), Some(pro)) = (pcb, only_file(&dir, "kicad_pro")) else {
        eprintln!("skipping: no routed candidate or project file");
        return;
    };
    let (mut full, _) = load_board_file(&pcb, &LoadOptions::default()).expect("board loads");
    let pro_text = std::fs::read_to_string(&pro).expect("project reads");
    let dru_text = only_file(&dir, "kicad_dru").map(|p| std::fs::read_to_string(p).unwrap());
    let rules = ProjectRuleTable::from_project(&pro_text, dru_text.as_deref(), &full)
        .expect("rules compile");
    rules.assign_classes(&mut full);

    // Split: locked copper stays on the board, the rest becomes routed.
    let mut base = full.clone();
    base.tracks.retain(|t| t.ownership == Ownership::Locked);
    base.vias.retain(|v| v.ownership == Ownership::Locked);
    for (i, t) in base.tracks.iter_mut().enumerate() {
        t.id = konnect_route::api::TrackId(i as u32);
    }
    for (i, v) in base.vias.iter_mut().enumerate() {
        v.id = konnect_route::api::ViaId(i as u32);
    }
    assert_eq!(base.validate(), Ok(()));
    let gen = Generation(1);
    let moved_tracks: Vec<_> = full
        .tracks
        .iter()
        .filter(|t| t.ownership != Ownership::Locked && t.net.is_some())
        .collect();
    let moved_vias: Vec<_> = full
        .vias
        .iter()
        .filter(|v| v.ownership != Ownership::Locked && v.net.is_some())
        .collect();
    let copper = RoutedCopper {
        tracks: moved_tracks
            .iter()
            .map(|t| RoutedTrack {
                net: t.net.unwrap(),
                layer: t.layer,
                seg: t.seg,
                gen,
            })
            .collect(),
        vias: moved_vias
            .iter()
            .map(|v| RoutedVia {
                net: v.net.unwrap(),
                center: v.center,
                diameter: v.diameter,
                drill: v.drill,
                layers: v.layers,
                gen,
            })
            .collect(),
    };
    let options = CheckOptions {
        check_zone_fills: false,
        ..CheckOptions::default()
    }
    .kicad_equivalent();

    let t = Instant::now();
    let checker = BoardChecker::new(&base, &rules, options);
    let build_ms = t.elapsed().as_secs_f64() * 1e3;
    let t = Instant::now();
    let routed = checker.check_routed(&copper);
    let routed_ms = t.elapsed().as_secs_f64() * 1e3;
    println!(
        "routed check: {} routed tracks, {} routed vias over {} fixed tracks; board side built in {build_ms:.0} ms, routed check {routed_ms:.0} ms; {:?}",
        copper.tracks.len(),
        copper.vias.len(),
        base.tracks.len(),
        routed.counts()
    );

    // The same findings by UUID from the whole-board check, keeping those
    // that involve moved copper.
    let uuid = |board: &RouteBoard, c: CheckedItem| -> Option<String> {
        match c {
            CheckedItem::RoutedTrack { index } => moved_tracks[index as usize].uuid.clone(),
            CheckedItem::RoutedVia { index } => moved_vias[index as usize].uuid.clone(),
            other => uuid_of(board, other),
        }
    };
    let key = |board: &RouteBoard, v: &konnect_route::check::Violation| {
        let mut ids: Vec<Option<String>> = vec![uuid(board, v.a)];
        if let Some(b) = v.b {
            ids.push(uuid(board, b));
        }
        ids.sort();
        (format!("{:?}", v.kind), ids)
    };
    let moved: BTreeSet<String> = moved_tracks
        .iter()
        .filter_map(|t| t.uuid.clone())
        .chain(moved_vias.iter().filter_map(|v| v.uuid.clone()))
        .collect();
    let whole = BoardChecker::new(&full, &rules, options).check_board();
    let want: BTreeSet<_> = whole
        .violations
        .iter()
        .map(|v| key(&full, v))
        .filter(|(_, ids)| ids.iter().flatten().any(|u| moved.contains(u)))
        .collect();
    let got: BTreeSet<_> = routed.violations.iter().map(|v| key(&base, v)).collect();
    let missing: Vec<_> = want.difference(&got).collect();
    let extra: Vec<_> = got.difference(&want).collect();
    println!(
        "routed vs whole-board: {} expected, {} found, {} missing, {} extra",
        want.len(),
        got.len(),
        missing.len(),
        extra.len()
    );
    for m in missing.iter().take(10) {
        println!("  missing {m:?}");
    }
    for x in extra.iter().take(10) {
        println!("  extra {x:?}");
    }
    assert!(missing.is_empty() && extra.is_empty());
}
