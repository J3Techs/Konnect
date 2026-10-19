//! Compares the net classes the project rule table resolves from a real
//! project's `netclass_patterns` with KiCad's own effective classes, as
//! captured in a Specctra export made through KiCad's IPC API (each DSN
//! class groups nets by KiCad's effective track width and clearance). Nothing
//! about the board is committed: the directory comes from
//! `KONNECT_DB1_FIXTURE_DIR`, the export is the one `*.dsn.konnect.json`
//! manifest (and its `.dsn`) in a subdirectory, and only the count of
//! disagreements is asserted (zero).
//!
//! ```text
//! KONNECT_DB1_FIXTURE_DIR=/path/to/fixture cargo test -p konnect-route \
//!     --test netclass_real_fixture -- --nocapture
//! ```

use konnect_route::rules::RuleTable;
use konnect_route::{load_board_file, LoadOptions, ProjectRuleTable};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

fn only_file(dir: &Path, suffix: &str) -> Option<PathBuf> {
    let mut found: Vec<PathBuf> = std::fs::read_dir(dir)
        .ok()?
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.to_string_lossy().ends_with(suffix))
        .collect();
    (found.len() == 1).then(|| found.remove(0))
}

/// The first manifest found one level down, with its DSN.
fn export(dir: &Path) -> Option<(PathBuf, PathBuf)> {
    let mut subdirs: Vec<PathBuf> = std::fs::read_dir(dir)
        .ok()?
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.is_dir())
        .collect();
    subdirs.sort();
    for d in subdirs {
        let mut manifests: Vec<PathBuf> = std::fs::read_dir(&d)
            .ok()?
            .filter_map(Result::ok)
            .map(|e| e.path())
            .filter(|p| p.to_string_lossy().ends_with(".dsn.konnect.json"))
            .collect();
        manifests.sort();
        for m in manifests {
            let dsn = PathBuf::from(m.to_string_lossy().trim_end_matches(".konnect.json"));
            if dsn.is_file() {
                return Some((m, dsn));
            }
        }
    }
    None
}

/// DSN class name to (width, clearance) in the DSN's units.
fn dsn_class_rules(dsn: &str) -> BTreeMap<String, (f64, f64)> {
    let mut out = BTreeMap::new();
    let mut rest = dsn;
    while let Some(i) = rest.find("(class ") {
        rest = &rest[i + 7..];
        let name: String = rest
            .chars()
            .take_while(|c| !c.is_whitespace() && *c != ')')
            .collect();
        let num = |key: &str| -> Option<f64> {
            let j = rest.find(key)?;
            let tail = &rest[j + key.len()..];
            tail.split(|c: char| c == ')' || c.is_whitespace())
                .next()?
                .parse()
                .ok()
        };
        if let (Some(w), Some(c)) = (num("(width "), num("(clearance ")) {
            out.insert(name, (w, c));
        }
    }
    out
}

#[test]
fn netclasses_match_kicads_effective_classes() {
    let Some(dir) = std::env::var_os("KONNECT_DB1_FIXTURE_DIR").map(PathBuf::from) else {
        eprintln!("skipping: KONNECT_DB1_FIXTURE_DIR is not set");
        return;
    };
    let (Some(pcb), Some(pro), Some((manifest, dsn))) = (
        only_file(&dir, ".kicad_pcb"),
        only_file(&dir, ".kicad_pro"),
        export(&dir),
    ) else {
        eprintln!(
            "skipping: {} lacks a board, project or DSN export",
            dir.display()
        );
        return;
    };
    let (board, _) = load_board_file(&pcb, &LoadOptions::default()).expect("board loads");
    let pro_text = std::fs::read_to_string(&pro).expect("project reads");
    let rules = ProjectRuleTable::from_project(&pro_text, None, &board).expect("rules compile");
    let manifest: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(&manifest).unwrap()).unwrap();
    let dsn_text = std::fs::read_to_string(&dsn).unwrap();
    let classes = dsn_class_rules(&dsn_text);
    // The DSN's unit per mm (`resolution` in µm steps).
    let per_mm = match manifest["coordinate_unit"].as_str() {
        Some("um") => 1000.0,
        Some("mm") => 1.0,
        _ => 1000.0,
    };
    let mut compared = 0usize;
    let mut disagree: Vec<String> = Vec::new();
    for net in manifest["nets"].as_array().into_iter().flatten() {
        let (Some(name), Some(class)) = (net["name"].as_str(), net["class_name"].as_str()) else {
            continue;
        };
        let (Some(id), Some(&(w, c))) = (board.net_by_name(name), classes.get(class)) else {
            continue;
        };
        let ours = rules.class(rules.class_of(Some(id)));
        let mm = |nm: i32| f64::from(nm) / 1e6 * per_mm;
        compared += 1;
        if (mm(ours.track_width) - w).abs() > 0.5 || (mm(ours.clearance) - c).abs() > 0.5 {
            disagree.push(format!(
                "{name}: ours {} ({}/{}), KiCad's {w}/{c}",
                ours.name,
                mm(ours.track_width),
                mm(ours.clearance)
            ));
        }
    }
    println!(
        "netclasses: {compared} nets compared, {} disagree, {} ambiguous",
        disagree.len(),
        rules.report().ambiguous_nets.len()
    );
    for d in disagree.iter().take(20) {
        println!("  {d}");
    }
    assert!(compared > 0, "no nets compared");
    assert!(disagree.is_empty(), "{} nets disagree", disagree.len());
}
