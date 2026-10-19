//! Applying a native route plan through `add_routing`'s write path, and
//! reading KiCad's DRC back onto the plan.
//!
//! A [`RoutePlan`] can hold more than the 20,000 items one `CreateItems`
//! call may carry. [`apply_plan`] preflights the whole plan first (member
//! checks, the strict geometry lint over every track, every net name
//! resolved), so nothing is written unless all of it can be; then it creates
//! one net-whole chunk ([`RoutePlan::chunks_by_net`]) at a time and records
//! the KIID KiCad gave each item ([`ApplyLedger`]). A failure part-way is
//! reported as [`ApplyOutcome::Partial`] with the chunks already created, so
//! a caller retries only what was not applied.
//!
//! [`map_drc`] then follows KiCad's verdict back to the router: DRC item
//! UUID → KIID → plan item → net. The nets it names are the only ones to
//! penalise and reroute.

use super::cli::DrcReport;
use super::route_geometry;
use super::routing_batch::{self, Batch};
use konnect_route::plan::{PlanChunk, PlanItemKind, PlanItemRef, RoutePlan};
use serde::Serialize;
use serde_json::json;
use std::collections::{BTreeMap, BTreeSet};

/// Where created items go: KiCad over IPC in production, a fake in tests.
pub(crate) trait ItemSink {
    /// Create `items` in one request, returning the created items as KiCad
    /// echoes them (with the KIIDs it assigned).
    fn create(&mut self, items: Vec<prost_types::Any>) -> anyhow::Result<Vec<prost_types::Any>>;
}

/// The live sink: one `CreateItems` per chunk on the bound document.
pub(crate) struct IpcSink<'a> {
    pub client: &'a konnect_ipc::client::KiCadIpcClient,
    pub document: konnect_ipc::gen::kiapi::common::types::DocumentSpecifier,
}

impl ItemSink for IpcSink<'_> {
    fn create(&mut self, items: Vec<prost_types::Any>) -> anyhow::Result<Vec<prost_types::Any>> {
        self.client
            .create_items_in_returning(self.document.clone(), items)
    }
}

/// One created item and where it came from.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub(crate) struct AppliedItem {
    pub kiid: String,
    pub chunk: u32,
    pub item: PlanItemRef,
    pub net: String,
}

/// What an apply created, chunk by chunk.
#[derive(Debug, Clone, Default, PartialEq, Serialize)]
pub(crate) struct ApplyLedger {
    pub chunks_total: usize,
    pub chunks_applied: usize,
    pub items_created: usize,
    /// Created items KiCad did not echo back with an id that matches a plan
    /// item; they exist on the board but cannot be mapped.
    pub items_unmapped: usize,
    pub items: Vec<AppliedItem>,
}

impl ApplyLedger {
    /// KIID (lower case) → index into `items`.
    pub(crate) fn by_kiid(&self) -> BTreeMap<String, usize> {
        self.items
            .iter()
            .enumerate()
            .map(|(i, a)| (a.kiid.to_ascii_lowercase(), i))
            .collect()
    }
}

#[derive(Debug)]
pub(crate) enum ApplyOutcome {
    Complete(ApplyLedger),
    /// Chunks `0..ledger.chunks_applied` were created; `failed_chunk` was
    /// sent and failed (KiCad may or may not hold some of it); later chunks
    /// were never sent.
    Partial {
        ledger: ApplyLedger,
        failed_chunk: usize,
        error: String,
    },
}

/// Why nothing was written.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum ApplyRefusal {
    Invalid {
        field: String,
        reason: String,
    },
    Geometry {
        defects: usize,
        detail: serde_json::Value,
    },
    MissingNets(Vec<String>),
}

impl std::fmt::Display for ApplyRefusal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ApplyRefusal::Invalid { field, reason } => write!(f, "{field}: {reason}"),
            ApplyRefusal::Geometry { defects, .. } => write!(
                f,
                "{defects} geometry defect(s) under geometry_check strict"
            ),
            ApplyRefusal::MissingNets(n) => {
                write!(f, "{} net(s) not on the board: {}", n.len(), n.join(", "))
            }
        }
    }
}

/// The `add_routing` batch for one plan (or chunk).
pub(crate) fn batch_of(plan: &RoutePlan, locked: bool) -> Batch {
    Batch {
        tracks: plan
            .tracks
            .iter()
            .map(|t| routing_batch::Track {
                net: t.net.clone(),
                layer: t.layer.clone(),
                width: t.width,
                start: (t.x1, t.y1),
                end: (t.x2, t.y2),
            })
            .collect(),
        vias: plan
            .vias
            .iter()
            .map(|v| routing_batch::Via {
                net: v.net.clone(),
                at: (v.x, v.y),
                drill: v.drill,
                diameter: v.diameter,
            })
            .collect(),
        locked,
    }
}

/// Everything that can be checked before the first write.
pub(crate) fn preflight(
    plan: &RoutePlan,
    limits: &route_geometry::Limits,
    max_items: usize,
) -> Result<Vec<PlanChunk>, ApplyRefusal> {
    plan.validate().map_err(|e| match e {
        konnect_route::plan::PlanError::Invalid { field, reason } => {
            ApplyRefusal::Invalid { field, reason }
        }
        other => ApplyRefusal::Invalid {
            field: "vias".to_string(),
            reason: other.to_string(),
        },
    })?;
    let whole = batch_of(plan, false);
    let report = route_geometry::lint(&whole.segments(), limits);
    if report.defects() > 0 {
        return Err(ApplyRefusal::Geometry {
            defects: report.defects(),
            detail: report.to_json(limits),
        });
    }
    Ok(plan.chunks_by_net(max_items.clamp(1, routing_batch::MAX_ITEMS)))
}

/// A position key in nm, so a created item matches its plan item exactly.
fn nm(v: f64) -> i64 {
    (v * 1_000_000.0).round() as i64
}

/// What a created item is matched to its plan item on: kind, net, layer
/// (KiCad's layer enum; 0 for a through via), width (nm; 0 for a via) and
/// the sorted end points (nm). Two same-net tracks with the same ends on
/// different layers, or of different widths, never swap KIIDs.
type ItemKey = (&'static str, String, i32, i64, Vec<(i64, i64)>);

fn plan_key(chunk: &PlanChunk, k: usize) -> Option<ItemKey> {
    match chunk.plan.tracks.get(k) {
        Some(t) => {
            let mut pts = vec![(nm(t.x1), nm(t.y1)), (nm(t.x2), nm(t.y2))];
            pts.sort_unstable();
            let layer = konnect_ipc::builders::layer_from_name(&t.layer) as i32;
            Some(("track", t.net.clone(), layer, nm(t.width), pts))
        }
        None => {
            let v = chunk.plan.vias.get(k - chunk.plan.tracks.len())?;
            Some(("via", v.net.clone(), 0, 0, vec![(nm(v.x), nm(v.y))]))
        }
    }
}

/// The KIID and match key of every track and via KiCad echoed back, read
/// exactly (nm) from the messages. A message that does not decode, or an
/// item without a KIID, is left out (and counted as unmapped).
fn created_keys(created: &[prost_types::Any]) -> Vec<(String, ItemKey)> {
    use konnect_ipc::builders;
    use konnect_ipc::gen::kiapi::board::types as b;
    use prost::Message;
    let pt = |v: &Option<konnect_ipc::gen::kiapi::common::types::Vector2>| {
        v.as_ref().map(|v| (v.x_nm, v.y_nm))
    };
    let mut out = Vec::with_capacity(created.len());
    for any in created {
        if builders::any_is(any, "kiapi.board.types.Track") {
            let Ok(m) = b::Track::decode(any.value.as_slice()) else {
                continue;
            };
            let (Some(a), Some(z), Some(id)) = (pt(&m.start), pt(&m.end), m.id) else {
                continue;
            };
            let mut pts = vec![a, z];
            pts.sort_unstable();
            let net = m.net.map(|n| n.name).unwrap_or_default();
            let width = m.width.map_or(0, |w| w.value_nm);
            out.push((id.value, ("track", net, m.layer, width, pts)));
        } else if builders::any_is(any, "kiapi.board.types.Via") {
            let Ok(m) = b::Via::decode(any.value.as_slice()) else {
                continue;
            };
            let (Some(at), Some(id)) = (pt(&m.position), m.id) else {
                continue;
            };
            let net = m.net.map(|n| n.name).unwrap_or_default();
            out.push((id.value, ("via", net, 0, 0, vec![at])));
        }
    }
    out
}

/// Create `plan` through `sink`, chunk by chunk, after a full preflight.
/// `codes` is the board's net name → net code map (one `GetNets`).
pub(crate) fn apply_plan(
    plan: &RoutePlan,
    locked: bool,
    limits: &route_geometry::Limits,
    max_items: usize,
    codes: &BTreeMap<String, i32>,
    sink: &mut dyn ItemSink,
) -> Result<ApplyOutcome, ApplyRefusal> {
    let chunks = preflight(plan, limits, max_items)?;
    // Every chunk's items are built before the first is sent, so a missing
    // net refuses the whole plan.
    let mut built = Vec::with_capacity(chunks.len());
    for chunk in &chunks {
        match batch_of(&chunk.plan, locked).items(codes) {
            Ok(items) => built.push(items),
            Err(missing) => {
                let all: BTreeSet<String> = plan
                    .tracks
                    .iter()
                    .map(|t| t.net.clone())
                    .chain(plan.vias.iter().map(|v| v.net.clone()))
                    .filter(|n| !codes.contains_key(n))
                    .collect();
                let mut names: Vec<String> = all.into_iter().collect();
                if names.is_empty() {
                    names = missing;
                }
                return Err(ApplyRefusal::MissingNets(names));
            }
        }
    }
    let mut ledger = ApplyLedger {
        chunks_total: chunks.len(),
        ..ApplyLedger::default()
    };
    for (ci, (chunk, items)) in chunks.iter().zip(built).enumerate() {
        let sent = items.len();
        let created = match sink.create(items) {
            Ok(created) => created,
            Err(error) => {
                return Ok(ApplyOutcome::Partial {
                    ledger,
                    failed_chunk: ci,
                    error: format!("{error:#}"),
                })
            }
        };
        ledger.chunks_applied += 1;
        ledger.items_created += sent;
        // Match what came back to the plan by kind, net and exact position.
        let mut wanted: BTreeMap<ItemKey, Vec<usize>> = BTreeMap::new();
        for k in (0..chunk.plan.item_count()).rev() {
            if let Some(key) = plan_key(chunk, k) {
                wanted.entry(key).or_default().push(k);
            }
        }
        let mut mapped = 0usize;
        // An item that does not decode stays unmapped (counted below),
        // never unreported.
        for (kiid, key) in created_keys(&created) {
            if kiid.is_empty() {
                continue;
            }
            let Some(k) = wanted.get_mut(&key).and_then(Vec::pop) else {
                continue;
            };
            let Some(item) = chunk.item(k) else { continue };
            ledger.items.push(AppliedItem {
                kiid,
                chunk: ci as u32,
                item,
                net: key.1,
            });
            mapped += 1;
        }
        ledger.items_unmapped += sent.saturating_sub(mapped);
    }
    Ok(ApplyOutcome::Complete(ledger))
}

/// One DRC item, followed back to the plan where it can be.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub(crate) struct MappedItem {
    pub uuid: Option<String>,
    pub plan_item: Option<PlanItemRef>,
    pub net: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub(crate) struct MappedFinding {
    pub category: &'static str,
    pub rule: String,
    pub severity: String,
    pub description: String,
    pub items: Vec<MappedItem>,
}

impl MappedFinding {
    pub(crate) fn touches_plan(&self) -> bool {
        self.items.iter().any(|i| i.plan_item.is_some())
    }
}

#[derive(Debug, Clone, Default, PartialEq, Serialize)]
pub(crate) struct NetFindings {
    pub errors: usize,
    pub warnings: usize,
    pub rules: BTreeMap<String, usize>,
}

/// KiCad's DRC, read back onto a plan.
#[derive(Debug, Clone, Default, PartialEq, Serialize)]
pub(crate) struct DrcPlanMap {
    pub findings: Vec<MappedFinding>,
    pub touching_plan: usize,
    pub not_touching_plan: usize,
    pub items_mapped: usize,
    /// Items with a UUID that is not one of the plan's KIIDs (board copper,
    /// pads, zones) or with no UUID at all.
    pub items_not_in_plan: usize,
    /// Plan nets with findings, by net.
    pub by_net: BTreeMap<String, NetFindings>,
    /// Plan nets an error-severity finding involves, sorted: the nets to
    /// penalise and reroute.
    pub nets_to_reroute: Vec<String>,
    /// DRC categories this report did not carry; when not empty the map is
    /// incomplete evidence, not a clean bill.
    pub missing_categories: Vec<&'static str>,
}

/// Map `report` onto the plan behind `ledger`. `plan` names the nets of
/// mapped items.
pub(crate) fn map_drc(report: &DrcReport, ledger: &ApplyLedger, plan: &RoutePlan) -> DrcPlanMap {
    let index = ledger.by_kiid();
    let mut map = DrcPlanMap {
        missing_categories: report.missing_categories(),
        ..DrcPlanMap::default()
    };
    let mut reroute: BTreeSet<String> = BTreeSet::new();
    let categories = [
        ("violations", Some(&report.violations)),
        ("unconnected_items", report.unconnected_items.as_ref()),
        ("schematic_parity", report.schematic_parity.as_ref()),
    ];
    for (category, list) in categories {
        for v in list.into_iter().flatten() {
            let items: Vec<MappedItem> = v
                .items
                .iter()
                .map(|item| {
                    let applied = item
                        .uuid
                        .as_deref()
                        .and_then(|u| index.get(&u.to_ascii_lowercase()))
                        .map(|&i| &ledger.items[i]);
                    let net = applied.map(|a| match a.item.kind {
                        PlanItemKind::Track => plan.tracks[a.item.index as usize].net.clone(),
                        PlanItemKind::Via => plan.vias[a.item.index as usize].net.clone(),
                    });
                    MappedItem {
                        uuid: item.uuid.clone(),
                        plan_item: applied.map(|a| a.item),
                        net,
                    }
                })
                .collect();
            let finding = MappedFinding {
                category,
                rule: v.rule.clone(),
                severity: v.severity.clone(),
                description: v.description.clone(),
                items,
            };
            let nets: BTreeSet<&String> = finding
                .items
                .iter()
                .filter_map(|i| i.net.as_ref())
                .collect();
            for i in &finding.items {
                if i.plan_item.is_some() {
                    map.items_mapped += 1;
                } else {
                    map.items_not_in_plan += 1;
                }
            }
            for net in &nets {
                let e = map.by_net.entry((*net).clone()).or_default();
                if finding.severity == "error" {
                    e.errors += 1;
                    reroute.insert((*net).clone());
                } else {
                    e.warnings += 1;
                }
                *e.rules.entry(finding.rule.clone()).or_default() += 1;
            }
            if finding.touches_plan() {
                map.touching_plan += 1;
            } else {
                map.not_touching_plan += 1;
            }
            map.findings.push(finding);
        }
    }
    map.nets_to_reroute = reroute.into_iter().collect();
    map
}

/// A compact JSON summary of an apply, for tool responses.
pub(crate) fn ledger_summary(ledger: &ApplyLedger) -> serde_json::Value {
    json!({
        "chunks_total": ledger.chunks_total,
        "chunks_applied": ledger.chunks_applied,
        "items_created": ledger.items_created,
        "items_mapped": ledger.items.len(),
        "items_unmapped": ledger.items_unmapped,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tools::cli::{DrcViolation, ReportItem};
    use konnect_ipc::builders;
    use konnect_ipc::gen::kiapi::board::types as b;
    use konnect_ipc::gen::kiapi::common::types::Kiid;
    use konnect_route::plan::{PlanTrack, PlanVia};
    use prost::Message;

    /// Gives every item a KIID and echoes it back, like KiCad.
    #[derive(Default)]
    struct FakeKiCad {
        calls: Vec<usize>,
        next: u32,
        fail_on_call: Option<usize>,
        /// Echo each call's items in reverse order.
        reverse: bool,
    }

    impl ItemSink for FakeKiCad {
        fn create(
            &mut self,
            items: Vec<prost_types::Any>,
        ) -> anyhow::Result<Vec<prost_types::Any>> {
            if self.fail_on_call == Some(self.calls.len()) {
                anyhow::bail!("KiCad created 0 of {} requested items", items.len());
            }
            self.calls.push(items.len());
            let mut out = Vec::new();
            for any in items {
                self.next += 1;
                let id = Some(Kiid {
                    value: format!("00000000-0000-4000-8000-{:012X}", self.next),
                });
                if builders::any_is(&any, "kiapi.board.types.Track") {
                    let mut t = b::Track::decode(any.value.as_slice()).unwrap();
                    t.id = id;
                    out.push(builders::pack_any(&t, "kiapi.board.types.Track"));
                } else {
                    let mut v = b::Via::decode(any.value.as_slice()).unwrap();
                    v.id = id;
                    out.push(builders::pack_any(&v, "kiapi.board.types.Via"));
                }
            }
            if self.reverse {
                out.reverse();
            }
            Ok(out)
        }
    }

    fn track(net: &str, x1: f64, x2: f64, y: f64) -> PlanTrack {
        PlanTrack {
            net: net.into(),
            layer: "F.Cu".into(),
            width: 0.15,
            x1,
            y1: y,
            x2,
            y2: y,
        }
    }

    fn plan() -> RoutePlan {
        RoutePlan {
            tracks: vec![
                track("A", 0.0, 2.0, 1.0),
                track("B", 0.0, 2.0, 3.0),
                track("A", 2.0, 4.0, 1.0),
                track("C", 0.0, 4.0, 5.0),
            ],
            vias: vec![PlanVia {
                net: "B".into(),
                x: 2.0,
                y: 3.0,
                drill: 0.3,
                diameter: 0.6,
            }],
        }
    }

    fn codes() -> BTreeMap<String, i32> {
        [("A", 1), ("B", 2), ("C", 3)]
            .into_iter()
            .map(|(n, c)| (n.to_string(), c))
            .collect()
    }

    fn limits() -> route_geometry::Limits {
        route_geometry::Limits {
            min_bend_angle_deg: 89.0,
            ..route_geometry::Limits::default()
        }
    }

    #[test]
    fn a_plan_is_created_in_net_whole_chunks_and_every_item_gets_its_kiid() {
        let mut kicad = FakeKiCad::default();
        let outcome = apply_plan(&plan(), false, &limits(), 3, &codes(), &mut kicad).unwrap();
        let ApplyOutcome::Complete(ledger) = outcome else {
            panic!("expected a complete apply")
        };
        // A (2 tracks) | B (track + via) + C: no net is split.
        assert_eq!(kicad.calls, vec![2, 3]);
        assert_eq!(ledger.chunks_applied, 2);
        assert_eq!(ledger.items_created, 5);
        assert_eq!(ledger.items_unmapped, 0);
        assert_eq!(ledger.items.len(), 5);
        let via = ledger
            .items
            .iter()
            .find(|a| a.item.kind == PlanItemKind::Via)
            .unwrap();
        assert_eq!((via.net.as_str(), via.item.index), ("B", 0));
        let kiids: BTreeSet<&String> = ledger.items.iter().map(|a| &a.kiid).collect();
        assert_eq!(kiids.len(), 5);
    }

    #[test]
    fn kiids_follow_layer_and_width_when_two_tracks_share_their_ends() {
        // One net, the same two ends, three tracks: F.Cu, B.Cu and a wider
        // F.Cu one. KiCad echoing them in another order must not swap them.
        let plan = RoutePlan {
            tracks: vec![
                track("A", 0.0, 2.0, 1.0),
                PlanTrack {
                    layer: "B.Cu".into(),
                    ..track("A", 0.0, 2.0, 1.0)
                },
                PlanTrack {
                    layer: "In1.Cu".into(),
                    width: 0.3,
                    ..track("A", 0.0, 2.0, 1.0)
                },
            ],
            vias: vec![],
        };
        let mut kicad = FakeKiCad {
            reverse: true,
            ..FakeKiCad::default()
        };
        let outcome = apply_plan(&plan, false, &limits(), 100, &codes(), &mut kicad).unwrap();
        let ApplyOutcome::Complete(ledger) = outcome else {
            panic!("expected a complete apply")
        };
        assert_eq!(ledger.items.len(), 3);
        assert_eq!(ledger.items_unmapped, 0);
        // The fake numbers KIIDs in send order: plan track k gets k + 1.
        for a in &ledger.items {
            let n = u32::from_str_radix(a.kiid.rsplit('-').next().unwrap(), 16).unwrap();
            assert_eq!(a.item.index + 1, n, "{a:?}");
        }

        // The width alone also tells two tracks apart.
        let plan = RoutePlan {
            tracks: vec![
                track("A", 0.0, 2.0, 1.0),
                PlanTrack {
                    width: 0.3,
                    ..track("A", 0.0, 2.0, 1.0)
                },
            ],
            vias: vec![],
        };
        let key = |k| plan_key(&plan.chunks_by_net(100)[0], k).unwrap();
        assert_ne!(key(0), key(1));
    }

    #[test]
    fn nothing_is_written_when_a_net_is_missing_or_the_geometry_is_bad() {
        let mut kicad = FakeKiCad::default();
        let mut few = codes();
        few.remove("C");
        let err = apply_plan(&plan(), false, &limits(), 3, &few, &mut kicad).unwrap_err();
        assert_eq!(err, ApplyRefusal::MissingNets(vec!["C".to_string()]));
        assert!(kicad.calls.is_empty());

        let mut jog = plan();
        jog.tracks = vec![
            track("A", 0.0, 5.0, 0.0),
            PlanTrack {
                x1: 5.0,
                y1: 0.0,
                x2: 5.0,
                y2: 0.08,
                ..track("A", 0.0, 0.0, 0.0)
            },
            track("A", 5.0, 9.0, 0.08),
        ];
        let err = apply_plan(&jog, false, &limits(), 3, &codes(), &mut kicad).unwrap_err();
        assert!(
            matches!(err, ApplyRefusal::Geometry { defects: 1, .. }),
            "{err}"
        );
        assert!(kicad.calls.is_empty());
    }

    #[test]
    fn a_failure_part_way_reports_what_was_applied() {
        let mut kicad = FakeKiCad {
            fail_on_call: Some(1),
            ..FakeKiCad::default()
        };
        let outcome = apply_plan(&plan(), true, &limits(), 3, &codes(), &mut kicad).unwrap();
        let ApplyOutcome::Partial {
            ledger,
            failed_chunk,
            error,
        } = outcome
        else {
            panic!("expected a partial apply")
        };
        assert_eq!((ledger.chunks_applied, failed_chunk), (1, 1));
        assert_eq!(ledger.items.len(), 2);
        assert!(error.contains("created 0 of 3"), "{error}");
    }

    fn item(uuid: Option<&str>) -> ReportItem {
        ReportItem {
            description: "x".into(),
            pos: None,
            uuid: uuid.map(String::from),
            kicad_reported_pos: None,
            ownership_status: None,
            item_kind: None,
            layer: None,
            owner: None,
        }
    }

    fn violation(rule: &str, severity: &str, items: Vec<ReportItem>) -> DrcViolation {
        DrcViolation {
            severity: severity.into(),
            description: rule.into(),
            rule: rule.into(),
            pos: None,
            items,
        }
    }

    #[test]
    fn drc_items_map_back_through_kiids_to_plan_items_and_nets() {
        let mut kicad = FakeKiCad::default();
        let p = plan();
        let ApplyOutcome::Complete(ledger) =
            apply_plan(&p, false, &limits(), 3, &codes(), &mut kicad).unwrap()
        else {
            panic!()
        };
        let kiid_of = |kind: PlanItemKind, index: u32| {
            ledger
                .items
                .iter()
                .find(|a| a.item == PlanItemRef { kind, index })
                .unwrap()
                .kiid
                .to_ascii_lowercase()
        };
        let report = DrcReport {
            violations: vec![
                // Plan track of C against a board pad (no plan id).
                violation(
                    "clearance",
                    "error",
                    vec![
                        item(Some(&kiid_of(PlanItemKind::Track, 3))),
                        item(Some("11111111-2222-3333-4444-555555555555")),
                    ],
                ),
                violation(
                    "track_angle",
                    "warning",
                    vec![item(Some(&kiid_of(PlanItemKind::Track, 0)))],
                ),
                violation("silk_overlap", "warning", vec![item(None)]),
            ],
            unconnected_items: Some(vec![violation(
                "unconnected_items",
                "error",
                vec![item(Some(&kiid_of(PlanItemKind::Via, 0)))],
            )]),
            schematic_parity: None,
            ownership_diagnostic: None,
            schematic_parity_diagnostic: None,
            cli_messages: vec![],
            custom_rules_error: None,
            custom_rules: None,
        };
        let map = map_drc(&report, &ledger, &p);
        assert_eq!(map.findings.len(), 4);
        assert_eq!((map.touching_plan, map.not_touching_plan), (3, 1));
        assert_eq!((map.items_mapped, map.items_not_in_plan), (3, 2));
        assert_eq!(map.nets_to_reroute, vec!["B".to_string(), "C".to_string()]);
        assert_eq!(map.by_net["A"].warnings, 1);
        assert_eq!(map.by_net["C"].rules["clearance"], 1);
        assert_eq!(map.missing_categories, vec!["schematic_parity"]);
        assert_eq!(map.findings[3].category, "unconnected_items");
    }

    // ─── Real board (outside the repository; skipped when unset) ─────────

    type Fixture = (std::path::PathBuf, std::path::PathBuf, std::path::PathBuf);

    fn fixture() -> Option<Fixture> {
        let dir = std::path::PathBuf::from(std::env::var_os("KONNECT_DB1_FIXTURE_DIR")?);
        let only = |d: &std::path::Path, ext: &str| {
            let mut v: Vec<_> = std::fs::read_dir(d)
                .ok()?
                .filter_map(Result::ok)
                .map(|e| e.path())
                .filter(|p| p.extension().and_then(|e| e.to_str()) == Some(ext))
                .collect();
            v.sort();
            (v.len() == 1).then(|| v.remove(0))
        };
        let mut subdirs: Vec<_> = std::fs::read_dir(&dir)
            .ok()?
            .filter_map(Result::ok)
            .map(|e| e.path())
            .filter(|p| p.is_dir())
            .collect();
        subdirs.sort();
        let routed_dir = subdirs
            .into_iter()
            .find(|d| only(d, "kicad_pcb").is_some())?;
        Some((
            only(&routed_dir, "kicad_pcb")?,
            only(&dir, "kicad_pro")?,
            routed_dir,
        ))
    }

    /// The optimizer's plan for a real routed board through the real
    /// `add_routing` lint and a chunked apply, and the board's saved KiCad
    /// DRC reports mapped onto its own routed copper by UUID.
    #[test]
    fn real_board_plan_lint_apply_and_drc_mapping() {
        use konnect_route::api::{Generation, Ownership};
        use konnect_route::optimize::{geometry_defects, optimize_with, OptimizeOptions};
        use konnect_route::plan::RoutedCopper;
        use std::time::Instant;
        let Some((pcb, pro, routed_dir)) = fixture() else {
            eprintln!("skipping: KONNECT_DB1_FIXTURE_DIR is not set or holds no routed board");
            return;
        };
        let dru = pro.with_extension("kicad_dru");
        let (mut board, _) =
            konnect_route::load_board_file(&pcb, &konnect_route::LoadOptions::default()).unwrap();
        let rules = konnect_route::ProjectRuleTable::from_project(
            &std::fs::read_to_string(&pro).unwrap(),
            std::fs::read_to_string(&dru).ok().as_deref(),
            &board,
        )
        .unwrap();
        rules.assign_classes(&mut board);

        // The routed copper keeps its own UUIDs, in take_unlocked order.
        let all = board.layers.all();
        let track_ids: Vec<Option<String>> = board
            .tracks
            .iter()
            .filter(|t| t.ownership == Ownership::Fixed && t.net.is_some() && t.seg.a != t.seg.b)
            .map(|t| t.uuid.clone())
            .collect();
        let via_ids: Vec<Option<String>> = board
            .vias
            .iter()
            .filter(|v| v.ownership == Ownership::Fixed && v.net.is_some() && v.layers == all)
            .map(|v| v.uuid.clone())
            .collect();
        let copper = RoutedCopper::take_unlocked(&mut board, None, Generation(0));
        assert_eq!(copper.tracks.len(), track_ids.len());
        let as_routed = RoutePlan::from_routed(&board, &copper).unwrap();
        let mut ledger = ApplyLedger::default();
        for (i, id) in track_ids.iter().enumerate() {
            if let Some(id) = id {
                ledger.items.push(AppliedItem {
                    kiid: id.clone(),
                    chunk: 0,
                    item: PlanItemRef {
                        kind: PlanItemKind::Track,
                        index: i as u32,
                    },
                    net: as_routed.tracks[i].net.clone(),
                });
            }
        }
        for (i, id) in via_ids.iter().enumerate() {
            if let Some(id) = id {
                ledger.items.push(AppliedItem {
                    kiid: id.clone(),
                    chunk: 0,
                    item: PlanItemRef {
                        kind: PlanItemKind::Via,
                        index: i as u32,
                    },
                    net: as_routed.vias[i].net.clone(),
                });
            }
        }
        let mut reports: Vec<_> = std::fs::read_dir(&routed_dir)
            .unwrap()
            .filter_map(Result::ok)
            .map(|e| e.path())
            .filter(|p| p.extension().and_then(|e| e.to_str()) == Some("json"))
            .collect();
        reports.sort();
        for path in reports {
            let Ok(raw) =
                serde_json::from_str::<serde_json::Value>(&std::fs::read_to_string(&path).unwrap())
            else {
                continue;
            };
            // A DRC report as Konnect serialises it (run_drc's result).
            let Ok(report) = serde_json::from_value::<DrcReport>(raw) else {
                continue;
            };
            let t = Instant::now();
            let map = map_drc(&report, &ledger, &as_routed);
            let mut rules_hit: BTreeMap<&str, (usize, usize)> = BTreeMap::new();
            for f in &map.findings {
                let e = rules_hit.entry(f.rule.as_str()).or_default();
                e.0 += 1;
                e.1 += usize::from(f.touches_plan());
            }
            println!(
                "DRC report ({} findings) mapped in {} us: {} touch routed copper, {} do not; {} items mapped, {} not; {} nets to reroute; per rule (all, touching routed): {rules_hit:?}",
                map.findings.len(),
                t.elapsed().as_micros(),
                map.touching_plan,
                map.not_touching_plan,
                map.items_mapped,
                map.items_not_in_plan,
                map.nets_to_reroute.len()
            );
        }

        // Optimize, then the real lint and a chunked apply.
        let mut optimized = copper.clone();
        let opts = OptimizeOptions::default();
        let mut telemetry = konnect_route::telemetry::Telemetry::default();
        let report = optimize_with(&board, &rules, &mut optimized, &opts, &mut telemetry);
        let plan = RoutePlan::from_routed(&board, &optimized).unwrap();
        let limits = route_geometry::Limits {
            min_bend_angle_deg: 89.0,
            ..route_geometry::Limits::default()
        };
        for (name, p) in [("as routed", &as_routed), ("optimized", &plan)] {
            let t = Instant::now();
            let lint = route_geometry::lint(&batch_of(p, false).segments(), &limits);
            let json = lint.to_json(&limits);
            println!(
                "{name}: {} tracks, lint {} ms, {} defects {}",
                p.tracks.len(),
                t.elapsed().as_millis(),
                lint.defects(),
                json["counts"]
            );
        }
        let mirror = geometry_defects(&rules, &opts, &optimized)
            .into_iter()
            .filter(|(_, _, d)| d.kind.is_lint_kind())
            .count();
        let lint = route_geometry::lint(&batch_of(&plan, false).segments(), &limits);
        println!(
            "optimizer: {} ms, mirror says {mirror} lint defects, lint says {}",
            telemetry.stage_us(konnect_route::api::Stage::Optimize) / 1000,
            lint.defects()
        );
        assert!(
            lint.defects() <= mirror,
            "the optimizer mirror must never be looser than the lint"
        );
        assert!(report.defects_after as usize >= mirror);

        let codes: BTreeMap<String, i32> = board
            .nets
            .iter()
            .enumerate()
            .map(|(i, n)| (n.name.clone(), i as i32 + 1))
            .collect();
        // The strict lint refuses any defect, so a plan with some left is
        // applied here with the lint relaxed, to measure the write path.
        let apply_limits = if lint.defects() == 0 {
            limits
        } else {
            println!(
                "note: {} defects remain; apply measured with the lint relaxed",
                lint.defects()
            );
            route_geometry::Limits {
                min_bend_angle_deg: 0.0,
                jog_length_mm: 1e-6,
            }
        };
        let mut kicad = FakeKiCad::default();
        let t = Instant::now();
        let outcome = apply_plan(&plan, false, &apply_limits, 20_000, &codes, &mut kicad).unwrap();
        let ApplyOutcome::Complete(applied) = outcome else {
            panic!("fake apply failed")
        };
        println!(
            "apply (fake KiCad): {} ms, {} calls of {:?} items, {} mapped, {} unmapped",
            t.elapsed().as_millis(),
            kicad.calls.len(),
            kicad.calls,
            applied.items.len(),
            applied.items_unmapped
        );
        assert_eq!(applied.items.len(), plan.item_count());
    }
}
