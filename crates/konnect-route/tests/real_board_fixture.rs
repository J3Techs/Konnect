//! Loads a real board from outside the repository and prints what the model
//! holds. Nothing about that board is committed: the directory comes from
//! `KONNECT_DB1_FIXTURE_DIR`, the files are found by extension, and the test
//! asserts only invariants that hold for any KiCad-authored board.
//!
//! ```text
//! KONNECT_DB1_FIXTURE_DIR=/path/to/fixture cargo test -p konnect-route \
//!     --test real_board_fixture -- --nocapture
//! ```
//!
//! Without the variable (or with a directory that holds no `.kicad_pcb`)
//! the test prints why and passes, so CI and fresh checkouts stay green.

use konnect_geom::ArcPolicy;
use konnect_route::api::{ItemKind, Ownership};
use konnect_route::board::PadKind;
use konnect_route::connectivity::ConnectivityReport;
use konnect_route::rules::RuleSubject;
use konnect_route::{load_board_file, LoadOptions, ProjectRuleTable, RuleTable};
use std::path::{Path, PathBuf};
use std::time::Instant;

/// The single file in `dir` with extension `ext`, if there is exactly one.
fn only_file(dir: &Path, ext: &str) -> Option<PathBuf> {
    let mut found = std::fs::read_dir(dir)
        .ok()?
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|e| e.to_str()) == Some(ext));
    let first = found.next()?;
    found.next().is_none().then_some(first)
}

#[test]
fn real_board_fixture_loads_faithfully() {
    let Some(dir) = std::env::var_os("KONNECT_DB1_FIXTURE_DIR").map(PathBuf::from) else {
        eprintln!("skipping: KONNECT_DB1_FIXTURE_DIR is not set");
        return;
    };
    let Some(pcb) = only_file(&dir, "kicad_pcb") else {
        eprintln!("skipping: {} holds no single .kicad_pcb", dir.display());
        return;
    };

    let start = Instant::now();
    let (mut board, report) =
        load_board_file(&pcb, &LoadOptions::default()).expect("fixture board loads");
    let load_ms = start.elapsed().as_millis();
    assert_eq!(board.validate(), Ok(()));

    let netted_pads = board.pads.iter().filter(|p| p.net.is_some()).count();
    let locked_tracks = board
        .tracks
        .iter()
        .filter(|t| t.ownership == Ownership::Locked)
        .count();
    let locked_vias = board
        .vias
        .iter()
        .filter(|v| v.ownership == Ownership::Locked)
        .count();
    let th_pads = board
        .pads
        .iter()
        .filter(|p| p.kind == PadKind::ThroughHole)
        .count();
    let courtyards = board
        .footprints
        .iter()
        .filter(|f| !f.courtyards.is_empty())
        .count();
    println!("load: {load_ms} ms");
    println!(
        "layers: {} copper ({} routable)",
        board.layer_count(),
        board.routable.count()
    );
    println!(
        "footprints: {} ({courtyards} with courtyards)",
        board.footprints.len()
    );
    println!(
        "pads: {} ({netted_pads} netted, {th_pads} through-hole)",
        board.pads.len()
    );
    println!("nets: {}", board.nets.len());
    println!(
        "tracks: {} ({locked_tracks} locked), arcs: {}, vias: {} ({locked_vias} locked)",
        board.tracks.len(),
        board.arcs.len(),
        board.vias.len()
    );
    println!(
        "zone pieces: {}, keepouts: {}, outline edges: {}",
        board.zones.len(),
        board.keepouts.len(),
        board.outline.edges.len()
    );
    println!("skipped: {:?}", report.skipped);
    println!("notes: {:?}", report.notes);

    assert!(!board.footprints.is_empty());
    assert!(netted_pads > 0);
    assert_eq!(
        report.skipped_total(),
        0,
        "a KiCad-authored board should load without dropping items: {:?}",
        report.skipped
    );

    // Rules from the sibling project files, when present.
    if let Some(pro) = only_file(&dir, "kicad_pro") {
        let pro_text = std::fs::read_to_string(&pro).expect("project file reads");
        let dru_text = only_file(&dir, "kicad_dru")
            .map(|p| std::fs::read_to_string(p).expect("rules file reads"));
        let table = ProjectRuleTable::from_project(&pro_text, dru_text.as_deref(), &board)
            .expect("project rules compile");
        table.assign_classes(&mut board);
        let rules = table.report();
        println!("classes: {:?}", rules.nets_per_class);
        println!(
            "custom rules applied: {}, inert: {}",
            rules.applied_rules.len(),
            rules.inert.len()
        );
        for inert in &rules.inert {
            println!("  inert: {} — {}", inert.rule, inert.reason);
        }
        println!("ambiguous class patterns: {}", rules.ambiguous_nets.len());
        println!("board rules: {:?}", rules.board);
        let any = RuleSubject::new(ItemKind::Track, None);
        assert!(
            table.clearance(&any, &any, board.layers.copper[0].id) >= rules.board.min_clearance
        );
        assert!(table.max_clearance() >= rules.board.min_clearance);
    }

    let start = Instant::now();
    let conn = ConnectivityReport::build(&board, &ArcPolicy::default());
    let conn_ms = start.elapsed().as_millis();
    let connections = conn.connections(&board);
    println!("connectivity: {conn_ms} ms");
    println!(
        "multi-pad nets: {}, connections needed: {}, unfilled zone pieces skipped: {}",
        conn.multi_pad_net_count(),
        conn.connection_count(),
        conn.unfilled_zones_skipped
    );
    assert_eq!(connections.len(), conn.connection_count());
}
