//! The connections a run leaves open, named by pad and by reason.
//!
//! Two views of one fact. [`RouteFailure`] is a task the route stage gave
//! up on, with the reason negotiation recorded; it is kept in the
//! checkpoint so a resumed job still knows it. [`Unrouted`] is what is
//! still open with the run's copper on the board (the connections KiCad's
//! DRC will list as unconnected once the plan is applied), each matched to
//! the stage that left it open. Nothing here is inferred: a connection no
//! stage accounts for is reported as `unexplained`.

use konnect_geom::BBox;
use konnect_route::api::{ClassId, Connection, ItemRef, NetId};
use konnect_route::connectivity::ConnectivityReport;
use konnect_route::detail::negotiate::DetailFailure;
use konnect_route::pipeline::{
    select_connections, split_excluded, split_left_to_planes, PlaneTracks,
};
use konnect_route::{RouteBoard, RuleTable};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet};

/// Other connections named per congestion failure; `against_total` counts all.
const AGAINST_LISTED: usize = 8;

/// The board's track and via counts before the run's own copper was
/// appended to it: an index at or past these is copper of the run.
pub type OwnFrom = (usize, usize);

/// One end of a connection, as the board holds it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Endpoint {
    /// `pad`, `track`, `arc`, `via` or `zone`.
    pub kind: String,
    /// A pad's footprint reference (absent when the footprint has none).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub reference: Option<String>,
    /// A pad's number.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pad: Option<String>,
    /// A pad's or via's centre; a track's or arc's start.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub position_mm: Option<[f64; 2]>,
    /// A track's or arc's other end.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub end_mm: Option<[f64; 2]>,
    /// The copper layers it is on.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub layers: Vec<String>,
    /// A zone piece's zone name, when it has one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub zone: Option<String>,
    /// Set when the item is copper of this run's plan or fanout plan (a
    /// track or via the run planned or took as its work), not copper the
    /// run left on the board.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub planned_by_this_run: bool,
}

impl Endpoint {
    /// `item` as `board` holds it: a pad by reference and number, anything
    /// else by its kind and where it is, never as a pad.
    pub fn of(board: &RouteBoard, own_from: OwnFrom, item: ItemRef) -> Endpoint {
        let mm = |p: konnect_geom::Point| {
            let (x, y) = p.to_mm();
            Some([x, y])
        };
        let names = |mask: konnect_route::api::LayerMask| -> Vec<String> {
            mask.iter()
                .filter(|l| l.index() < board.layers.len())
                .map(|l| board.layers.name(l).to_string())
                .collect()
        };
        let mut e = Endpoint {
            kind: match item {
                ItemRef::Pad(_) => "pad",
                ItemRef::Track(_) => "track",
                ItemRef::Arc(_) => "arc",
                ItemRef::Via(_) => "via",
                ItemRef::Zone(_) => "zone",
            }
            .to_string(),
            reference: None,
            pad: None,
            position_mm: None,
            end_mm: None,
            layers: Vec::new(),
            zone: None,
            planned_by_this_run: false,
        };
        match item {
            ItemRef::Pad(id) => {
                if let Some(p) = board.pads.get(id.index()) {
                    e.reference = board
                        .footprints
                        .get(p.footprint.index())
                        .and_then(|f| f.reference.clone());
                    e.pad = Some(p.number.clone());
                    e.position_mm = mm(p.center);
                    e.layers = names(p.layers);
                }
            }
            ItemRef::Track(id) => {
                e.planned_by_this_run = id.index() >= own_from.0;
                if let Some(t) = board.tracks.get(id.index()) {
                    e.position_mm = mm(t.seg.a);
                    e.end_mm = mm(t.seg.b);
                    e.layers = vec![board.layers.name(t.layer).to_string()];
                }
            }
            ItemRef::Arc(id) => {
                if let Some(a) = board.arcs.get(id.index()) {
                    e.position_mm = mm(a.start);
                    e.end_mm = mm(a.end);
                    e.layers = vec![board.layers.name(a.layer).to_string()];
                }
            }
            ItemRef::Via(id) => {
                e.planned_by_this_run = id.index() >= own_from.1;
                if let Some(v) = board.vias.get(id.index()) {
                    e.position_mm = mm(v.center);
                    e.layers = names(v.layers);
                }
            }
            ItemRef::Zone(id) => {
                if let Some(z) = board.zones.get(id.index()) {
                    e.zone = z.zone_name.clone();
                    e.layers = vec![board.layers.name(z.layer).to_string()];
                }
            }
        }
        e
    }

    /// `R5.2` for a pad; otherwise the kind and where it is.
    pub fn label(&self) -> String {
        match (&self.reference, &self.pad) {
            (Some(r), Some(p)) => format!("{r}.{p}"),
            (None, Some(p)) => format!("pad {p}"),
            _ => match self.position_mm {
                Some([x, y]) => format!("{} at ({x}, {y})", self.kind),
                None => self.kind.clone(),
            },
        }
    }
}

/// Another connection a failed one was still in conflict with.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Against {
    pub net: String,
    pub from: String,
    pub to: String,
}

/// A task the route stage gave up on.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RouteFailure {
    pub net: String,
    pub from: Endpoint,
    pub to: Endpoint,
    /// [`DetailFailure::name`].
    pub reason: String,
    /// `no_access`: the terminal that offers no open node.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub no_access: Option<Endpoint>,
    /// `congestion`: how many negotiation iterations ran.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub iterations: Option<u32>,
    /// `congestion`: the connections it was still in conflict with (the
    /// first few; `against_total` counts all).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub against: Vec<Against>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub against_total: Option<usize>,
}

impl RouteFailure {
    /// The route stage's failures, resolved against the board it routed
    /// on: the board as loaded with the stage's fanout appended
    /// (`connections` is the stage's task list; ids are positions in it).
    pub fn collect(
        board: &RouteBoard,
        own_from: OwnFrom,
        connections: &[Connection],
        failures: &[(konnect_route::api::ConnectionId, DetailFailure)],
    ) -> Vec<RouteFailure> {
        let net = |n: NetId| board.net(n).name.clone();
        failures
            .iter()
            .filter_map(|(id, f)| {
                let c = connections.get(id.0 as usize)?;
                let mut out = RouteFailure {
                    net: net(c.net),
                    from: Endpoint::of(board, own_from, c.from),
                    to: Endpoint::of(board, own_from, c.to),
                    reason: f.name().to_string(),
                    no_access: None,
                    iterations: None,
                    against: Vec::new(),
                    against_total: None,
                };
                match f {
                    DetailFailure::NoAccess { item } => {
                        out.no_access = Some(Endpoint::of(board, own_from, *item));
                    }
                    DetailFailure::Congestion {
                        iterations,
                        against,
                    } => {
                        out.iterations = Some(*iterations);
                        out.against_total = Some(against.len());
                        out.against = against
                            .iter()
                            .filter_map(|a| connections.get(a.0 as usize))
                            .take(AGAINST_LISTED)
                            .map(|a| Against {
                                net: net(a.net),
                                from: Endpoint::of(board, own_from, a.from).label(),
                                to: Endpoint::of(board, own_from, a.to).label(),
                            })
                            .collect();
                    }
                    _ => {}
                }
                Some(out)
            })
            .collect()
    }

    /// `from -> to: reason`, naming the walled-in terminal of a `no_access`.
    fn summary(&self) -> String {
        let mut s = format!(
            "{} -> {}: {}",
            self.from.label(),
            self.to.label(),
            self.reason
        );
        if let Some(item) = &self.no_access {
            s.push_str(&format!(" ({})", item.label()));
        }
        s
    }

    fn joins(&self, net: &str, a: &Endpoint, b: &Endpoint) -> bool {
        self.net == net
            && ((&self.from == a && &self.to == b) || (&self.from == b && &self.to == a))
    }
}

/// One connection still open with the run's copper on the board.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OpenConnection {
    pub net: String,
    pub from: Endpoint,
    pub to: Endpoint,
    /// Centre-to-centre distance of the two ends.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub length_mm: Option<f64>,
    /// Why it is open: a route-stage failure name (`no_access`,
    /// `no_corridor`, `no_pin`, `unreachable`, `budget_exhausted`,
    /// `exact_check_rejected`, `congestion`, `off_reference`) when the
    /// route stage failed exactly this pad pair, or `net_not_joined`,
    /// `cleared`, `excluded_by_policy`, `left_to_plane`, `outside_scope`,
    /// `not_routed`, `unexplained` (see [`reason_text`]).
    pub reason: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub no_access: Option<Endpoint>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub iterations: Option<u32>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub against: Vec<Against>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub against_total: Option<usize>,
    /// `net_not_joined`: the net's route-stage failures that match no open
    /// pad pair exactly, as `from -> to: reason` (and the terminal a
    /// `no_access` names).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub failed_tasks: Vec<String>,
    /// The clear stage also took copper of this net out.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub net_cleared: bool,
}

/// What a run leaves open.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct Unrouted {
    /// Every connection still open on the board with the run's copper
    /// (fanout and plan) applied, whatever its reason.
    pub total: usize,
    pub by_reason: BTreeMap<String, usize>,
    /// All of them: the run's own failures first, then the ones it was not
    /// asked to route.
    pub connections: Vec<OpenConnection>,
    /// Every task the route stage gave up on, as it saw them (the report
    /// file holds these; the tools list `connections`).
    #[serde(default)]
    pub route_failures: Vec<RouteFailure>,
}

/// What the run was asked to route, and what its later stages took out.
pub struct Account<'a> {
    pub nets: Option<&'a BTreeSet<NetId>>,
    pub region: Option<BBox>,
    /// Where the run's own copper starts in the applied board.
    pub own_from: OwnFrom,
    pub excluded_classes: &'a BTreeSet<ClassId>,
    /// Which plane nets the run routed as tracks: the open connections of
    /// the others were left to their planes, and are no failure.
    pub plane_tracks: &'a PlaneTracks,
    /// Whether the run routed (source native) rather than took copper.
    pub routed_here: bool,
    pub failures: &'a [RouteFailure],
    /// Nets the clear stage took copper from.
    pub cleared: &'a [NetId],
    /// Nets that lost copper to the reference-plane check.
    pub off_reference: &'a [NetId],
}

/// The open connections of `applied` (the board with all of the run's
/// copper on it), each with the reason the account gives for it.
pub fn unrouted(applied: &RouteBoard, rules: &dyn RuleTable, account: &Account<'_>) -> Unrouted {
    let conn = ConnectivityReport::build(applied, &konnect_geom::ArcPolicy::default());
    let open = conn.connections(applied);
    let key = |c: &Connection| (c.net, c.from, c.to);
    let selected = select_connections(applied, open.clone(), account.nets, account.region);
    let in_scope: BTreeSet<_> = selected.iter().map(key).collect();
    let (routable, excluded) = split_excluded(rules, selected, account.excluded_classes);
    let excluded: BTreeSet<_> = excluded.iter().map(key).collect();
    // Plane nets the run left to their planes, as the route stage split them.
    let (_, left) = split_left_to_planes(applied, routable, account.plane_tracks);
    let left_to_plane: BTreeSet<_> = left.iter().map(key).collect();

    // Failures not yet matched to an open pad pair, per net.
    let mut matched = vec![false; account.failures.len()];
    let mut rows: Vec<(u8, OpenConnection)> = Vec::with_capacity(open.len());
    let mut pending: Vec<usize> = Vec::new();
    for c in &open {
        let net = applied.net(c.net).name.clone();
        let (from, to) = (
            Endpoint::of(applied, account.own_from, c.from),
            Endpoint::of(applied, account.own_from, c.to),
        );
        let length_mm = match (from.position_mm, to.position_mm) {
            (Some([x1, y1]), Some([x2, y2])) => {
                Some((((x2 - x1).hypot(y2 - y1)) * 1000.0).round() / 1000.0)
            }
            _ => None,
        };
        let mut row = OpenConnection {
            net,
            from,
            to,
            length_mm,
            reason: String::new(),
            no_access: None,
            iterations: None,
            against: Vec::new(),
            against_total: None,
            failed_tasks: Vec::new(),
            net_cleared: account.cleared.contains(&c.net),
        };
        let exact = account
            .failures
            .iter()
            .enumerate()
            .find(|(i, f)| !matched[*i] && f.joins(&row.net, &row.from, &row.to));
        let rank = if let Some((i, f)) = exact {
            matched[i] = true;
            row.reason = f.reason.clone();
            row.no_access = f.no_access.clone();
            row.iterations = f.iterations;
            row.against = f.against.clone();
            row.against_total = f.against_total;
            0
        } else if !in_scope.contains(&key(c)) {
            row.reason = "outside_scope".to_string();
            3
        } else if excluded.contains(&key(c)) {
            row.reason = "excluded_by_policy".to_string();
            2
        } else if left_to_plane.contains(&key(c)) {
            row.reason = "left_to_plane".to_string();
            2
        } else if !account.routed_here {
            row.reason = "not_routed".to_string();
            2
        } else {
            // Decided once every exact match is known.
            pending.push(rows.len());
            1
        };
        rows.push((rank, row));
    }
    for i in pending {
        let (_, row) = &mut rows[i];
        let net_id = applied.net_by_name(&row.net);
        let tasks: Vec<String> = account
            .failures
            .iter()
            .enumerate()
            .filter(|(j, f)| !matched[*j] && f.net == row.net)
            .map(|(_, f)| f.summary())
            .collect();
        let is = |nets: &[NetId]| net_id.is_some_and(|n| nets.contains(&n));
        row.reason = if !tasks.is_empty() {
            row.failed_tasks = tasks;
            "net_not_joined"
        } else if is(account.off_reference) {
            "off_reference"
        } else if row.net_cleared {
            "cleared"
        } else {
            "unexplained"
        }
        .to_string();
    }
    rows.sort_by_key(|(rank, _)| *rank);
    let connections: Vec<OpenConnection> = rows.into_iter().map(|(_, r)| r).collect();
    let mut by_reason: BTreeMap<String, usize> = BTreeMap::new();
    for c in &connections {
        *by_reason.entry(c.reason.clone()).or_default() += 1;
    }
    Unrouted {
        total: connections.len(),
        by_reason,
        connections,
        route_failures: account.failures.to_vec(),
    }
}

/// What each `reason` means, for the tools' legend.
pub fn reason_text(reason: &str) -> &'static str {
    match reason {
        "no_access" => "A terminal offers no open routing-grid node on a layer its class may use: the item under no_access is walled in by other nets' copper, a keepout or the board edge at the class clearance, or it lies on no allowed layer.",
        "no_corridor" => "Global routing found no corridor for the connection, or its class may use no routable layer.",
        "no_pin" => "No legal crossing point where the route passes from one routing tile to the next.",
        "unreachable" => "The search found no path between the two ends past the hard obstacles (other nets' board copper, keepouts, the board edge) on the layers its class may use.",
        "budget_exhausted" => "The search's expansion budget ran out before a path was found.",
        "exact_check_rejected" => "Every path proposed for it failed the exact clearance check.",
        "congestion" => "Still in conflict with other connections after the last negotiation iteration; against names them.",
        "off_reference" => "Routed, then taken out because its copper left the reference plane its class's fallback names.",
        "net_not_joined" => "The route stage failed tasks of this net (failed_tasks), none between exactly these two pads: these are the closest pads of the two parts the net is left in.",
        "cleared" => "Routed, then taken out by the clear stage because the internal check still found a defect in the net's copper (see cleared in the report).",
        "excluded_by_policy" => "Its net class is left unrouted by the allowances (see excluded_by_policy).",
        "left_to_plane" => "Not a routing failure: its net owns a copper zone and is left to its plane and the plane fanout, so the run drew no track for it. plane_nets lists the net's pads that still need a via or a track; plane_nets_as_tracks routes the net as tracks.",
        "outside_scope" => "Not selected by the job's nets, or an end lies outside its region: the job did not try it.",
        "not_routed" => "The job took existing copper (source board_unlocked or plan) and routed nothing.",
        _ => "No stage of this run reported why it is open (a job resumed from a checkpoint written before failures were kept reports its failures this way).",
    }
}

/// The tools' bounded view: at most `listed` connections, the totals over
/// all, a legend for the reasons present and where the full list is.
pub fn listing(unrouted: &Value, listed: usize, full_list: Option<String>) -> Value {
    let empty = Vec::new();
    let all = unrouted["connections"].as_array().unwrap_or(&empty);
    let shown: Vec<&Value> = all.iter().take(listed).collect();
    let reasons: BTreeMap<&str, &str> = unrouted["by_reason"]
        .as_object()
        .map(|m| m.keys().map(|k| (k.as_str(), reason_text(k))).collect())
        .unwrap_or_default();
    let mut body = json!({
        "total": unrouted["total"],
        "by_reason": unrouted["by_reason"],
        "listed": shown.len(),
        "connections": shown,
        "reasons": reasons,
    });
    if shown.len() < all.len() {
        body["note"] = json!(match &full_list {
            Some(path) => format!(
                "{} of {} connections listed; raise max_listed, or read unrouted.connections in {path} for all of them",
                shown.len(),
                all.len()
            ),
            None => format!(
                "{} of {} connections listed; raise max_listed for the rest",
                shown.len(),
                all.len()
            ),
        });
    }
    if let Some(path) = full_list {
        body["full_list"] = json!(path);
    }
    body
}

#[cfg(test)]
mod tests {
    use super::*;
    use konnect_route::pipeline::{add_locked, PipelineSettings};
    use konnect_route::synthetic::{BoardBuilder, UniformRules};

    fn account<'a>(
        board: &RouteBoard,
        failures: &'a [RouteFailure],
        none: &'a (BTreeSet<ClassId>, Vec<NetId>),
    ) -> Account<'a> {
        Account {
            nets: None,
            region: None,
            own_from: (board.tracks.len(), board.vias.len()),
            excluded_classes: &none.0,
            plane_tracks: &PlaneTracks::All,
            routed_here: true,
            failures,
            cleared: &none.1,
            off_reference: &none.1,
        }
    }

    #[test]
    fn a_connection_the_router_gives_up_on_is_listed_by_pad_with_its_reason() {
        // One layer, and another net's locked track from edge to edge
        // between the two pads: no route exists.
        let mut b = BoardBuilder::new(40.0, 20.0, &["F.Cu"]);
        let r1 = b.footprint("R1", (5.0, 10.0));
        b.smd_pad(r1, "1", "SIG", (5.0, 10.0), (0.5, 0.5), "F.Cu");
        let r2 = b.footprint("R2", (35.0, 10.0));
        b.smd_pad(r2, "2", "SIG", (35.0, 10.0), (0.5, 0.5), "F.Cu");
        b.track("WALL", "F.Cu", (20.0, 0.0), (20.0, 20.0), 0.5, true);
        let board = b.build();
        let rules = UniformRules::default();
        let settings = PipelineSettings {
            optimize: None,
            clear_findings: false,
            ..PipelineSettings::default()
        };
        let routed = super::super::route_native(&board, &rules, None, None, &settings).unwrap();
        assert_eq!(routed.failures.len(), 1, "{:?}", routed.detail);
        let failure = &routed.failures[0];
        // The counts the stage record keeps are the same failures.
        assert_eq!(routed.detail["failed"][failure.reason.as_str()], 1);

        let none = Default::default();
        let mut applied = board.clone();
        add_locked(&mut applied, &routed.routed);
        let u = unrouted(&applied, &rules, &account(&board, &routed.failures, &none));
        assert_eq!(u.total, 1, "{u:?}");
        let c = &u.connections[0];
        assert_eq!(c.net, "SIG");
        let mut ends = [c.from.label(), c.to.label()];
        ends.sort();
        assert_eq!(ends, ["R1.1", "R2.2"]);
        assert_eq!(c.from.kind, "pad");
        assert_eq!(c.from.layers, ["F.Cu"]);
        assert_eq!(c.length_mm, Some(30.0));
        assert_eq!(c.reason, failure.reason);
        assert_ne!(reason_text(&c.reason), reason_text("unexplained"));
        assert_eq!(u.by_reason[&c.reason], 1);
        assert_eq!(u.route_failures, routed.failures);
    }

    #[test]
    fn an_end_that_is_not_a_pad_is_described_as_the_copper_it_is() {
        use konnect_route::api::{TrackId, ViaId};
        let mut b = BoardBuilder::new(40.0, 20.0, &["F.Cu", "B.Cu"]);
        b.track("SIG", "F.Cu", (30.0, 10.0), (35.0, 10.0), 0.2, true);
        b.via("SIG", (12.5, 6.0), 0.6, 0.3, true);
        let board = b.build();
        // The via counts as copper the run planned; the track was on the board.
        let own_from = (1, 0);
        let track = Endpoint::of(&board, own_from, ItemRef::Track(TrackId(0)));
        assert_eq!(track.kind, "track");
        assert_eq!((&track.reference, &track.pad), (&None, &None));
        assert_eq!(track.position_mm, Some([30.0, 10.0]));
        assert_eq!(track.end_mm, Some([35.0, 10.0]));
        assert_eq!(track.layers, ["F.Cu"]);
        assert!(!track.planned_by_this_run);
        assert_eq!(track.label(), "track at (30, 10)");
        let via = Endpoint::of(&board, own_from, ItemRef::Via(ViaId(0)));
        assert_eq!(via.kind, "via");
        assert_eq!((&via.reference, &via.pad), (&None, &None));
        assert_eq!(via.position_mm, Some([12.5, 6.0]));
        assert_eq!(via.layers, ["F.Cu", "B.Cu"]);
        assert!(via.planned_by_this_run);
        assert_eq!(via.label(), "via at (12.5, 6)");
    }

    #[test]
    fn connections_the_job_did_not_try_are_listed_last_with_why() {
        let mut b = BoardBuilder::new(40.0, 20.0, &["F.Cu"]);
        for (reference, net, y) in [("R1", "OTHER", 4.0), ("R2", "SIG", 10.0)] {
            let f = b.footprint(reference, (5.0, y));
            b.smd_pad(f, "1", net, (5.0, y), (0.5, 0.5), "F.Cu");
            b.smd_pad(f, "2", net, (9.0, y), (0.5, 0.5), "F.Cu");
        }
        let board = b.build();
        let none = Default::default();
        let only: BTreeSet<NetId> = [board.net_by_name("SIG").unwrap()].into();
        let mut account = account(&board, &[], &none);
        account.nets = Some(&only);
        account.routed_here = false;
        let u = unrouted(&board, &UniformRules::default(), &account);
        let listed: Vec<(&str, &str)> = u
            .connections
            .iter()
            .map(|c| (c.net.as_str(), c.reason.as_str()))
            .collect();
        assert_eq!(listed, [("SIG", "not_routed"), ("OTHER", "outside_scope")]);
        assert_eq!(u.total, 2);
        assert_eq!(u.by_reason["not_routed"], 1);
        assert_eq!(u.by_reason["outside_scope"], 1);
    }

    #[test]
    fn a_failed_task_between_other_pads_of_the_net_is_named_on_the_open_connection() {
        // Three pads in a row; the stage failed a task the board no longer
        // shows as a pad pair (R1.1 to R3.1), so the open pairs carry it.
        let mut b = BoardBuilder::new(40.0, 20.0, &["F.Cu"]);
        for (reference, x) in [("R1", 5.0), ("R2", 15.0), ("R3", 25.0)] {
            let f = b.footprint(reference, (x, 10.0));
            b.smd_pad(f, "1", "SIG", (x, 10.0), (0.5, 0.5), "F.Cu");
        }
        let board = b.build();
        let own_from = (board.tracks.len(), board.vias.len());
        let pad = |i: u32| ItemRef::Pad(konnect_route::api::PadId(i));
        let failures = vec![RouteFailure {
            net: "SIG".to_string(),
            from: Endpoint::of(&board, own_from, pad(0)),
            to: Endpoint::of(&board, own_from, pad(2)),
            reason: "no_access".to_string(),
            no_access: Some(Endpoint::of(&board, own_from, pad(2))),
            iterations: None,
            against: Vec::new(),
            against_total: None,
        }];
        let none = Default::default();
        let u = unrouted(
            &board,
            &UniformRules::default(),
            &account(&board, &failures, &none),
        );
        assert_eq!(u.total, 2, "{u:?}");
        for c in &u.connections {
            assert_eq!(c.reason, "net_not_joined");
            assert_eq!(c.failed_tasks, ["R1.1 -> R3.1: no_access (R3.1)"]);
        }
    }

    #[test]
    fn the_tools_listing_is_capped_and_says_where_the_full_list_is() {
        let mut b = BoardBuilder::new(40.0, 20.0, &["F.Cu"]);
        for (reference, y) in [("R1", 4.0), ("R2", 8.0), ("R3", 12.0)] {
            let f = b.footprint(reference, (5.0, y));
            let net = format!("N{reference}");
            b.smd_pad(f, "1", &net, (5.0, y), (0.5, 0.5), "F.Cu");
            b.smd_pad(f, "2", &net, (9.0, y), (0.5, 0.5), "F.Cu");
        }
        let board = b.build();
        let none = Default::default();
        let mut account = account(&board, &[], &none);
        account.routed_here = false;
        let u = unrouted(&board, &UniformRules::default(), &account);
        let all = serde_json::to_value(&u).unwrap();
        let body = listing(&all, 2, Some("job/report.json".to_string()));
        assert_eq!(body["total"], 3);
        assert_eq!(body["listed"], 2);
        assert_eq!(body["connections"].as_array().unwrap().len(), 2);
        assert_eq!(body["by_reason"]["not_routed"], 3);
        assert!(body["reasons"]["not_routed"].is_string());
        assert_eq!(body["full_list"], "job/report.json");
        let note = body["note"].as_str().unwrap();
        assert!(
            note.contains("2 of 3") && note.contains("job/report.json"),
            "{note}"
        );
        // Everything listed: no note, and the tools never repeat the raw
        // route-stage list.
        let body = listing(&all, 50, None);
        assert_eq!(body["listed"], 3);
        assert!(body.get("note").is_none() && body.get("route_failures").is_none());
    }
}
