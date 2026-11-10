//! Tap vias for named pads: one exact-checked through via near each pad,
//! joined to it by a stub on the pad's layer, so copper on another layer
//! (a power trunk on an inner layer, say) can reach the pad.
//!
//! This is the plane fanout's candidate and selection machinery with three
//! differences: the pads are named by the caller instead of found by net;
//! any net qualifies, and the via need not sit inside a plane of its net;
//! and the via must stay clear of other nets' zone fills on the target
//! layer, so copper there can leave it. Everything else is the fanout's:
//! every candidate is checked exactly against every layer (annulus and stub
//! clearance with the resolved rules, hole to hole, hole clearance, board
//! edge, keepouts, the via floor), the via stays out of other nets' islands
//! on every layer it passes, a via never lands on an SMD pad of its own net
//! (unless the caller names the pad, below), and sites of different pads are
//! placed so they clear each other.
//!
//! # Via-in-pad
//!
//! A pad the caller names ([`TapOptions::via_in_pad`]) may take a via of its
//! own net inside it, for a large pad with no legal site beside it. The
//! via's copper must lie wholly inside one copper shape of the pad (a via
//! across the pad's edge is refused), and every other check stays: the via
//! floor, clearance to other nets on every layer, hole to hole, hole
//! clearance, the board edge, rule areas, islands and keep-out boxes. Such
//! a site has no stub, since the pad itself joins the via, and says which
//! pad it is in ([`TapSite::in_pad`]). Sites inside the pad are tried before
//! the exits beside it, the pad's copper centre first. A pad that is not
//! named is never entered, and a pad whose copper the loader could only
//! approximate holds no via.
//!
//! Zones the caller names to carve ([`TapOptions::carve_zones`]) are ones a
//! refill will cut an antipad into around the via, on whichever copper
//! layer they lie (a tap via is a through via): neither their islands nor,
//! on the target layer, their fill rejects a site. [`super::via_carves`]
//! says which of them each placed via actually sits in. Caller keep-out
//! boxes ([`TapOptions::keepouts`]) take no via and no stub.
//!
//! A through-hole pad already reaches every layer: it gets no via and is
//! reported as its own tap.
//!
//! The stub runs from the pad's copper centre to the via centre, so every
//! via is anchored by a track. KiCad gives a via with no attached track the
//! net of the zone it sits in; the stub keeps the planned net.

use super::*;
use crate::api::KeepoutId;

/// One pad's tap.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TapSite {
    pub pad: PadId,
    pub net: NetId,
    /// Where the copper on the target layer meets the pad's net: the via
    /// centre, or a through-hole pad's anchor.
    pub at: Point,
    /// `None` for a through-hole pad, which needs no via.
    pub via: Option<(Coord, Coord)>,
    /// Stub from the pad's copper centre to the via, on `layer`. `None`
    /// also for a via inside its own pad.
    pub stub: Option<(Point, Point, Coord)>,
    pub layer: LayerId,
    /// The pad named for via-in-pad the via sits wholly inside.
    #[serde(default)]
    pub in_pad: Option<PadId>,
}

#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct TapPlan {
    /// Sites in the order the pads were asked for.
    pub sites: Vec<TapSite>,
    pub failed: Vec<(PadId, FanoutFailure)>,
    pub details: Vec<FailureDetail>,
    pub stats: FanoutStats,
}

impl TapPlan {
    /// The planned copper: every via spans all copper layers (`add_routing`
    /// makes through vias only).
    pub fn routed(&self, board: &RouteBoard) -> RoutedCopper {
        let mut out = RoutedCopper::default();
        let all = board.layers.all();
        for s in &self.sites {
            if let Some((a, b, w)) = s.stub {
                out.tracks.push(RoutedTrack {
                    net: s.net,
                    layer: s.layer,
                    seg: Segment::new(a, b, w),
                    gen: Generation(0),
                });
            }
            if let Some((diameter, drill)) = s.via {
                out.vias.push(RoutedVia {
                    net: s.net,
                    center: s.at,
                    diameter,
                    drill,
                    layers: all,
                    gen: Generation(0),
                });
            }
        }
        out
    }
}

/// What a tap plan may carve and must keep out of.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct TapOptions {
    /// Zones (exact names) a refill will cut an antipad into around a via,
    /// on any copper layer: their islands do not reject a site, nor, on the
    /// target layer, their fill.
    #[serde(default)]
    pub carve_zones: Vec<String>,
    /// Boxes no tap via or stub may touch.
    #[serde(default)]
    pub keepouts: Vec<BBox>,
    /// SMD pads that may take a via of their own net inside them (see the
    /// module documentation). Default: none.
    #[serde(default)]
    pub via_in_pad: Vec<PadId>,
}

/// Plan a tap for each of `pads` (a pad listed twice is planned once).
/// `target` is the layer the taps must reach: every via is a through via,
/// and it must also clear other nets' zone fills on `target`. The zones
/// named in `carve_zones` are carved around the via instead, on every layer
/// (see [`TapOptions::carve_zones`]).
pub fn plan_tap_vias(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    pads: &[PadId],
    target: LayerId,
    carve_zones: &[String],
    settings: &FanoutSettings,
) -> TapPlan {
    let options = TapOptions {
        carve_zones: carve_zones.to_vec(),
        ..TapOptions::default()
    };
    plan_tap_vias_with(board, rules, pads, target, &options, settings)
}

/// [`plan_tap_vias`] with keep-out boxes as well as carved zones.
pub fn plan_tap_vias_with(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    pads: &[PadId],
    target: LayerId,
    options: &TapOptions,
    settings: &FanoutSettings,
) -> TapPlan {
    let t0 = std::time::Instant::now();
    let mut s = *settings;
    s.require_own_plane = false;
    s.clear_of_zones_on = Some(target);
    let policy = ArcPolicy::default();
    let space = Space::new(board, rules, &policy, true);
    let carved = carve_mask(board, &options.carve_zones);
    let keepouts: Vec<Shape> = options
        .keepouts
        .iter()
        .filter_map(|b| ConvexPoly::axis_rect(*b, 0).map(Shape::ConvexPoly))
        .collect();
    let mut via_in_pad = options.via_in_pad.clone();
    via_in_pad.sort_unstable();
    via_in_pad.dedup();
    let mut plan = TapPlan::default();
    plan.stats.index_ms = t0.elapsed().as_millis() as u64;

    let mut seen = std::collections::BTreeSet::new();
    let mut order: Vec<PadId> = Vec::new();
    for &p in pads {
        if seen.insert(p) {
            order.push(p);
        }
    }
    let mut units: Vec<Unit> = Vec::new();
    let mut by_pad: BTreeMap<PadId, TapSite> = BTreeMap::new();
    for &pid in &order {
        let pad = &board.pads[pid.index()];
        let Some(net) = pad.net else {
            plan.failed.push((pid, FanoutFailure::NoLegalSite));
            continue;
        };
        if pad.kind == PadKind::ThroughHole && pad.layers.contains(target) {
            by_pad.insert(
                pid,
                TapSite {
                    pad: pid,
                    net,
                    at: pad.center,
                    via: None,
                    stub: None,
                    layer: target,
                    in_pad: None,
                },
            );
            continue;
        }
        units.push(Unit {
            net,
            pads: vec![pid],
            anchor: pad.center,
        });
    }
    plan.stats.eligible_islands = units.len();
    plan.stats.eligible_pads = units.len();

    let t1 = std::time::Instant::now();
    let ctx = Ctx {
        space: &space,
        rules,
        settings: &s,
        carved: &carved,
        target: &[],
        keepouts: &keepouts,
        via_in_pad: &via_in_pad,
    };
    let generated: Vec<(Vec<Cand>, Rejections)> = units
        .par_iter()
        .map_init(ClearanceCache::default, |cache, u| {
            candidates(&ctx, u, cache)
        })
        .collect();
    plan.stats.candidate_ms = t1.elapsed().as_millis() as u64;
    let mut cands: Vec<Vec<Cand>> = Vec::with_capacity(units.len());
    let mut rejections: Vec<Rejections> = Vec::with_capacity(units.len());
    for (c, r) in generated {
        plan.stats.candidates += r.total as usize;
        plan.stats.legal_candidates += c.len();
        cands.push(c);
        rejections.push(r);
    }
    let t2 = std::time::Instant::now();
    let mut chosen = select(&ctx, &units, &cands, &mut plan.stats);
    if s.retry_failed {
        plan.stats.retried_sites = retry(&ctx, &units, &mut cands, &mut rejections, &mut chosen);
    }
    plan.stats.select_ms = t2.elapsed().as_millis() as u64;

    for (ui, u) in units.iter().enumerate() {
        let pid = u.pads[0];
        match chosen[ui] {
            Some(ci) => {
                let c = &cands[ui][ci];
                plan.stats.floor_vias += usize::from(c.floor_via);
                plan.stats.minimum_stubs += usize::from(c.min_stub);
                by_pad.insert(
                    pid,
                    TapSite {
                        pad: pid,
                        net: u.net,
                        at: c.via,
                        via: Some((c.diameter, c.drill)),
                        stub: c.stub.map(|s| (s.a, s.b, s.width)),
                        layer: c.layer,
                        in_pad: c.in_pad,
                    },
                );
            }
            None => {
                let r = &rejections[ui];
                let failure = if cands[ui].is_empty() {
                    FanoutFailure::NoLegalSite
                } else {
                    FanoutFailure::Conflict
                };
                let blocked_by = if cands[ui].is_empty() {
                    Vec::new()
                } else {
                    blockers(&ctx, &units, &cands, &chosen, ui)
                };
                plan.failed.push((pid, failure));
                plan.details.push(FailureDetail {
                    pad: pid,
                    net: u.net,
                    candidates: r.total,
                    legal: cands[ui].len() as u32,
                    rejections: r.reasons(),
                    trials: r.trial_list(),
                    blocked_by,
                });
            }
        }
    }
    plan.sites = order.iter().filter_map(|p| by_pad.remove(p)).collect();
    plan.failed.sort_by_key(|f| f.0);
    plan.details.sort_by_key(|d| d.pad);
    plan
}

/// Why [`check_via_site`] refuses a site.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ViaSiteRefusal {
    /// The rule broken, as the tap planner names it (e.g. `via: copper
    /// clearance`, `via: inside another net's island`).
    pub reason: &'static str,
    /// The board item that breaks it (a pad, track, arc or via), if any.
    pub item: Option<ItemRef>,
    /// The zone piece (`board.zones` index) of an island.
    pub zone: Option<ZoneId>,
    /// The rule area that forbids the via.
    pub keepout: Option<KeepoutId>,
    /// The via floor at the site, when the via is below it.
    pub floor: Option<ViaRule>,
}

/// Whether a new through via of `net` at `center` meets the rules a tap via
/// meets: the via floor at the site, annulus clearance on every layer, hole
/// to hole, hole clearance, board edge and rule areas, no SMD pad of its own
/// net under it, and outside other nets' islands on every layer except the
/// islands of zones named in `carve_zones`. Other nets' base pours are not
/// obstacles (a refill pours around the via). Copper planned elsewhere is
/// checked only when the caller has frozen it into `board`.
///
/// A pad in `via_in_pad` may hold the via when it is of `net` and the via's
/// copper lies wholly inside the pad's (see the module documentation);
/// `Ok` names the pad the via is then in.
#[allow(clippy::too_many_arguments)]
pub fn check_via_site(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    net: NetId,
    center: Point,
    diameter: Coord,
    drill: Coord,
    carve_zones: &[String],
    via_in_pad: &[PadId],
) -> Result<Option<PadId>, ViaSiteRefusal> {
    let mut via_in_pad = via_in_pad.to_vec();
    via_in_pad.sort_unstable();
    via_in_pad.dedup();
    let settings = FanoutSettings {
        require_own_plane: false,
        clear_of_zones_on: None,
        ..FanoutSettings::default()
    };
    let policy = ArcPolicy::default();
    let space = Space::new(board, rules, &policy, true);
    let carved = carve_mask(board, carve_zones);
    let ctx = Ctx {
        space: &space,
        rules,
        settings: &settings,
        carved: &carved,
        target: &[],
        keepouts: &[],
        via_in_pad: &via_in_pad,
    };
    let mut cache = ClearanceCache::default();
    match check_via(&ctx, &mut cache, net, center, diameter, drill, &[]) {
        Ok((_, in_pad)) => Ok(in_pad),
        Err(why) => {
            let mut out = ViaSiteRefusal {
                reason: why.label(),
                item: None,
                zone: None,
                keepout: None,
                floor: None,
            };
            match why {
                ViaReject::Island(z) | ViaReject::TargetZone(z) => out.zone = Some(z),
                ViaReject::Floor(f) => out.floor = Some(f),
                ViaReject::OwnSmdPad(p) | ViaReject::PartlyOutsidePad(p) => {
                    out.item = Some(ItemRef::Pad(p))
                }
                ViaReject::Conflict(c) => match c {
                    Conflict::Copper { item, .. }
                    | Conflict::HoleToHole { item }
                    | Conflict::HoleClearance { item } => out.item = Some(item),
                    Conflict::Keepout { id } => out.keepout = Some(id),
                    Conflict::Edge => {}
                },
                ViaReject::OutsideOwnPlane | ViaReject::FewSpokes => {}
            }
            Err(out)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::check::check_routed;
    use crate::synthetic::{BoardBuilder, UniformRules};

    /// Two caps of different nets side by side on F.Cu, a GND via field
    /// around them, and a trunk layer In2.Cu carrying a foreign zone.
    fn board() -> RouteBoard {
        let mut b = BoardBuilder::new(30.0, 20.0, &["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]);
        for n in ["GND", "P5V", "P3V3"] {
            b.net(n);
        }
        let c1 = b.footprint("C1", (10.0, 10.0));
        b.smd_pad(c1, "1", "P5V", (9.5, 10.0), (0.6, 0.6), "F.Cu");
        b.smd_pad(c1, "2", "GND", (10.5, 10.0), (0.6, 0.6), "F.Cu");
        let c2 = b.footprint("C2", (10.0, 12.0));
        b.smd_pad(c2, "1", "P3V3", (9.5, 12.0), (0.6, 0.6), "F.Cu");
        b.smd_pad(c2, "2", "GND", (10.5, 12.0), (0.6, 0.6), "F.Cu");
        b.via("GND", (8.0, 10.0), 0.6, 0.3, false);
        b.zone("GND", "In2.Cu", (12.0, 6.0, 20.0, 16.0));
        b.build()
    }

    #[test]
    fn every_named_pad_gets_a_legal_anchored_via_clear_of_the_target_zone() {
        let board = board();
        let rules = UniformRules::default();
        let pads: Vec<PadId> = board
            .pads
            .iter()
            .filter(|p| p.number == "1")
            .map(|p| p.id)
            .collect();
        let target = board.layers.by_name("In2.Cu").unwrap();
        let plan = plan_tap_vias(
            &board,
            &rules,
            &pads,
            target,
            &[],
            &FanoutSettings::default(),
        );
        assert!(plan.failed.is_empty(), "{:?}", plan.details);
        assert_eq!(plan.sites.len(), 2);
        for s in &plan.sites {
            let (a, b, _) = s.stub.expect("every via has a stub");
            assert_eq!(b, s.at, "the stub ends on the via");
            let pad = &board.pads[s.pad.index()];
            assert_eq!(a, copper_center(&pad.shapes).unwrap());
            // Never a via in the pad.
            let (d, _) = s.via.unwrap();
            let disc = Shape::Circle(Circle::from_diameter(s.at, d));
            assert!(pad.shapes.iter().all(|p| !touches(&disc, p)));
            // Clear of the GND zone on In2.Cu by the clearance.
            assert!(s.at.x < 12_000_000 - 300_000 - 200_000 + 1, "{:?}", s.at);
        }
        let report = check_routed(&board, &rules, &plan.routed(&board));
        assert!(report.violations.is_empty(), "{:?}", report.violations);
    }

    #[test]
    fn a_through_hole_pad_is_its_own_tap() {
        let mut b = BoardBuilder::new(20.0, 20.0, &["F.Cu", "In1.Cu", "B.Cu"]);
        b.net("P5V");
        let j = b.footprint("J1", (10.0, 10.0));
        b.th_pad(j, "1", "P5V", (10.0, 10.0), 1.7, 1.0);
        let board = b.build();
        let rules = UniformRules::default();
        let target = board.layers.by_name("In1.Cu").unwrap();
        let plan = plan_tap_vias(
            &board,
            &rules,
            &[PadId(0)],
            target,
            &[],
            &FanoutSettings::default(),
        );
        assert_eq!(plan.sites.len(), 1);
        assert_eq!(plan.sites[0].via, None);
        assert_eq!(plan.sites[0].at, board.pads[0].center);
    }

    /// A P5V pad over a GND plane that covers all of In2.Cu, with a P3V3
    /// zone on In2.Cu over the pad's east side and a GND via south-west.
    fn plane_board() -> RouteBoard {
        let mut b = BoardBuilder::new(30.0, 20.0, &["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]);
        for n in ["GND", "P5V", "P3V3"] {
            b.net(n);
        }
        let c1 = b.footprint("C1", (10.0, 10.0));
        b.smd_pad(c1, "1", "P5V", (9.5, 10.0), (0.6, 0.6), "F.Cu");
        b.smd_pad(c1, "2", "GND", (10.5, 10.0), (0.6, 0.6), "F.Cu");
        b.via("GND", (8.3, 11.0), 0.6, 0.3, false);
        b.zone("GND", "In2.Cu", (0.5, 0.5, 29.5, 19.5));
        b.zone("P3V3", "In2.Cu", (9.0, 7.0, 14.0, 13.0));
        let mut board = b.build();
        board.zones[0].zone_name = Some("GND plane".to_string());
        board.zones[1].zone_name = Some("P3V3 island".to_string());
        board
    }

    #[test]
    fn a_carved_zone_admits_the_via_while_other_zones_and_its_own_copper_still_count() {
        let board = plane_board();
        let rules = UniformRules::default();
        let target = board.layers.by_name("In2.Cu").unwrap();
        let pad = board.pads.iter().find(|p| p.number == "1").unwrap().id;
        let s = FanoutSettings::default();
        // The plane covers every site on In2.Cu: nothing is legal.
        let plan = plan_tap_vias(&board, &rules, &[pad], target, &[], &s);
        assert!(plan.sites.is_empty());
        assert_eq!(plan.failed.len(), 1);
        let carve = ["GND plane".to_string()];
        let plan = plan_tap_vias(&board, &rules, &[pad], target, &carve, &s);
        assert!(plan.failed.is_empty(), "{:?}", plan.details);
        let site = &plan.sites[0];
        // Inside the carved plane, but still clear of the P3V3 zone (not
        // carved) by radius plus clearance.
        assert!(site.at.x <= 9_000_000 - 300_000 - 200_000, "{:?}", site.at);
        // The plane's own net keeps its copper: the GND via stays clear.
        let gnd_via = Shape::Circle(Circle::from_diameter(board.vias[0].center, 600_000));
        let disc = Shape::Circle(Circle::from_diameter(site.at, 600_000));
        assert_ne!(gap_cmp(&disc, &gnd_via, 200_000), Ordering::Less);
        let report = check_routed(&board, &rules, &plan.routed(&board));
        assert!(report.violations.is_empty(), "{:?}", report.violations);
    }

    /// A P5V pad on F.Cu; on In1.Cu a P12V zone of priority 1 (an island
    /// over the GND base pour) covers every site the planner tries. The
    /// target layer In2.Cu is empty.
    fn island_board() -> RouteBoard {
        let mut b = BoardBuilder::new(30.0, 20.0, &["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]);
        for n in ["GND", "P5V", "P12V"] {
            b.net(n);
        }
        let c1 = b.footprint("C1", (10.0, 10.0));
        b.smd_pad(c1, "1", "P5V", (9.5, 10.0), (0.6, 0.6), "F.Cu");
        b.zone("GND", "In1.Cu", (0.5, 0.5, 29.5, 19.5));
        b.zone("P12V", "In1.Cu", (3.0, 3.0, 17.0, 17.0));
        let mut board = b.build();
        board.zones[0].zone_name = Some("GND plane".to_string());
        board.zones[1].zone_name = Some("P12V island".to_string());
        board.zones[1].priority = 1;
        board
    }

    #[test]
    fn an_island_on_a_layer_the_via_only_passes_is_carved_only_when_named() {
        let board = island_board();
        let rules = UniformRules::default();
        let target = board.layers.by_name("In2.Cu").unwrap();
        let pad = board.pads[0].id;
        let s = FanoutSettings::default();
        let plan = plan_tap_vias(&board, &rules, &[pad], target, &[], &s);
        assert!(plan.sites.is_empty(), "{:?}", plan.sites);
        let reasons = &plan.details[0].rejections;
        assert!(
            reasons.contains_key("via: inside another net's island"),
            "{reasons:?}"
        );
        // Naming the base pour changes nothing: it never was an obstacle.
        let plan = plan_tap_vias(&board, &rules, &[pad], target, &["GND plane".into()], &s);
        assert!(plan.sites.is_empty());

        let carve = ["P12V island".to_string()];
        let plan = plan_tap_vias(&board, &rules, &[pad], target, &carve, &s);
        assert!(plan.failed.is_empty(), "{:?}", plan.details);
        let site = &plan.sites[0];
        let (d, _) = site.via.unwrap();
        let carves = via_carves(&board, &rules, &[(site.net, site.at, d)], &carve);
        assert_eq!(carves, vec![vec![ZoneId(1)]], "the via sits in the island");
        // A zone that is not named is never reported.
        let none = via_carves(&board, &rules, &[(site.net, site.at, d)], &[]);
        assert_eq!(none, vec![Vec::<ZoneId>::new()]);
        let report = check_routed(&board, &rules, &plan.routed(&board));
        assert!(report.violations.is_empty(), "{:?}", report.violations);
    }

    #[test]
    fn keep_out_boxes_take_no_tap_via_or_stub() {
        let board = board();
        let rules = UniformRules::default();
        let target = board.layers.by_name("In2.Cu").unwrap();
        let pad = board
            .pads
            .iter()
            .find(|p| p.number == "1" && p.center.y == 10_000_000)
            .unwrap()
            .id;
        let free = plan_tap_vias(
            &board,
            &rules,
            &[pad],
            target,
            &[],
            &FanoutSettings::default(),
        );
        let at = free.sites[0].at;
        // A box over the chosen site: the next site lies outside it.
        let b = BBox::new(
            Point::new(at.x - 100_000, at.y - 100_000),
            Point::new(at.x + 100_000, at.y + 100_000),
        );
        let options = TapOptions {
            keepouts: vec![b],
            ..TapOptions::default()
        };
        let plan = plan_tap_vias_with(
            &board,
            &rules,
            &[pad],
            target,
            &options,
            &FanoutSettings::default(),
        );
        assert!(plan.failed.is_empty(), "{:?}", plan.details);
        let s = &plan.sites[0];
        let ko = Shape::ConvexPoly(ConvexPoly::axis_rect(b, 0).unwrap());
        let (d, _) = s.via.unwrap();
        assert!(!touches(
            &Shape::Circle(Circle::from_diameter(s.at, d)),
            &ko
        ));
        let (a, z, w) = s.stub.unwrap();
        assert!(!touches(&Shape::Segment(Segment::new(a, z, w)), &ko));
        // A box over the whole pad leaves no site at all.
        let everything = TapOptions {
            keepouts: vec![BBox::new(
                Point::new(0, 0),
                Point::new(30_000_000, 20_000_000),
            )],
            ..TapOptions::default()
        };
        let plan = plan_tap_vias_with(
            &board,
            &rules,
            &[pad],
            target,
            &everything,
            &FanoutSettings::default(),
        );
        assert!(plan.sites.is_empty());
        assert!(plan.details[0]
            .rejections
            .keys()
            .any(|k| k.contains("keep-out box")));
    }

    #[test]
    fn a_single_via_site_is_judged_as_a_tap_via_and_names_what_refuses_it() {
        let board = island_board();
        let rules = UniformRules::default();
        let net = board.net_by_name("P5V").unwrap();
        let at = Point::new(12_000_000, 10_000_000);
        let r = check_via_site(&board, &rules, net, at, 600_000, 300_000, &[], &[]).unwrap_err();
        assert_eq!(r.reason, "via: inside another net's island");
        assert_eq!(r.zone, Some(ZoneId(1)));
        let carve = ["P12V island".to_string()];
        assert_eq!(
            check_via_site(&board, &rules, net, at, 600_000, 300_000, &carve, &[]),
            Ok(None)
        );
        // Below the floor (UniformRules' floor is the class via).
        let r = check_via_site(&board, &rules, net, at, 500_000, 300_000, &carve, &[]).unwrap_err();
        assert_eq!(r.reason, "via: below the via floor here");
        assert!(r.floor.is_some());
        // Too close to another net's via.
        let mut b = BoardBuilder::new(30.0, 20.0, &["F.Cu", "B.Cu"]);
        b.net("A");
        b.via("B", (10.0, 10.0), 0.6, 0.3, false);
        let board = b.build();
        let a = board.net_by_name("A").unwrap();
        let r = check_via_site(
            &board,
            &rules,
            a,
            Point::new(10_700_000, 10_000_000),
            600_000,
            300_000,
            &[],
            &[],
        )
        .unwrap_err();
        assert_eq!(r.reason, "via: copper clearance");
        assert_eq!(r.item, Some(ItemRef::Via(crate::api::ViaId(0))));
    }

    /// A 3.3 x 2.5 mm GND pad at (20, 20) on F.Cu (an SMC diode's anode),
    /// framed 0.3 mm away on every side by 8 mm of P5V pad copper, so no
    /// site beside it is legal. Pad 0 is the GND pad.
    fn boxed_pad_board() -> RouteBoard {
        let mut b = BoardBuilder::new(40.0, 40.0, &["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]);
        for n in ["GND", "P5V"] {
            b.net(n);
        }
        let d = b.footprint("D1", (20.0, 20.0));
        b.smd_pad(d, "2", "GND", (20.0, 20.0), (3.3, 2.5), "F.Cu");
        let f = b.footprint("X1", (20.0, 20.0));
        b.smd_pad(f, "1", "P5V", (20.0, 14.45), (19.9, 8.0), "F.Cu");
        b.smd_pad(f, "2", "P5V", (20.0, 25.55), (19.9, 8.0), "F.Cu");
        b.smd_pad(f, "3", "P5V", (14.05, 20.0), (8.0, 3.1), "F.Cu");
        b.smd_pad(f, "4", "P5V", (25.95, 20.0), (8.0, 3.1), "F.Cu");
        b.build()
    }

    fn tap(board: &RouteBoard, via_in_pad: &[PadId]) -> TapPlan {
        let options = TapOptions {
            via_in_pad: via_in_pad.to_vec(),
            ..TapOptions::default()
        };
        plan_tap_vias_with(
            board,
            &UniformRules::default(),
            &[PadId(0)],
            board.layers.by_name("In2.Cu").unwrap(),
            &options,
            &FanoutSettings::default(),
        )
    }

    #[test]
    fn a_pad_named_for_via_in_pad_takes_its_via_at_the_copper_centre_without_a_stub() {
        let board = boxed_pad_board();
        // As ever without the opt-in: the pad is never entered, and here
        // nothing beside it is legal.
        let plan = tap(&board, &[]);
        assert!(plan.sites.is_empty(), "{:?}", plan.sites);
        assert_eq!(plan.failed, vec![(PadId(0), FanoutFailure::NoLegalSite)]);

        let plan = tap(&board, &[PadId(0)]);
        assert!(plan.failed.is_empty(), "{:?}", plan.details);
        let site = &plan.sites[0];
        assert_eq!(site.in_pad, Some(PadId(0)));
        assert_eq!(site.at, Point::new(20_000_000, 20_000_000));
        assert_eq!(site.via, Some((600_000, 300_000)));
        assert_eq!(site.stub, None, "the pad is the join");
        let copper = plan.routed(&board);
        assert_eq!((copper.vias.len(), copper.tracks.len()), (1, 0));
        let report = check_routed(&board, &UniformRules::default(), &copper);
        assert!(report.violations.is_empty(), "{:?}", report.violations);
    }

    #[test]
    fn naming_a_pad_changes_nothing_for_the_pads_that_are_not_named() {
        // The two-cap board: C1.1 named, C2.1 not. C2.1's site is the one
        // it gets with nothing named, beside the pad. (C1.1's 0.6 mm pad is
        // as large as the via, which so fits only at its centre.)
        let board = board();
        let rules = UniformRules::default();
        let target = board.layers.by_name("In2.Cu").unwrap();
        let pads: Vec<PadId> = board
            .pads
            .iter()
            .filter(|p| p.number == "1")
            .map(|p| p.id)
            .collect();
        let s = FanoutSettings::default();
        let plain = plan_tap_vias(&board, &rules, &pads, target, &[], &s);
        let options = TapOptions {
            via_in_pad: vec![pads[0]],
            ..TapOptions::default()
        };
        let named = plan_tap_vias_with(&board, &rules, &pads, target, &options, &s);
        assert!(named.failed.is_empty(), "{:?}", named.details);
        assert_eq!(named.sites[1], plain.sites[1]);
        assert_eq!(named.sites[1].in_pad, None);
        assert_eq!(named.sites[0].in_pad, Some(pads[0]));
        assert_eq!(
            named.sites[0].at,
            copper_center(&board.pads[pads[0].index()].shapes).unwrap()
        );
    }

    #[test]
    fn a_single_site_in_a_named_pad_must_be_wholly_inside_it_and_of_its_net() {
        let board = boxed_pad_board();
        let rules = UniformRules::default();
        let gnd = board.net_by_name("GND").unwrap();
        let p5v = board.net_by_name("P5V").unwrap();
        let pad = PadId(0);
        let site = |net, x: Coord, named: &[PadId]| {
            let at = Point::new(x, 20_000_000);
            check_via_site(&board, &rules, net, at, 600_000, 300_000, &[], named)
        };
        // Not named: refused as it always was.
        let r = site(gnd, 20_000_000, &[]).unwrap_err();
        assert_eq!(r.reason, "via: on an SMD pad of its own net");
        assert_eq!(r.item, Some(ItemRef::Pad(pad)));
        // Named: accepted, and the reply names the pad. Touching the pad's
        // edge from inside still counts (the edge is at x = 21.65 mm).
        assert_eq!(site(gnd, 20_000_000, &[pad]), Ok(Some(pad)));
        assert_eq!(site(gnd, 21_350_000, &[pad]), Ok(Some(pad)));
        // A nanometre farther the via crosses the edge.
        let r = site(gnd, 21_350_001, &[pad]).unwrap_err();
        assert_eq!(r.reason, "via: partly outside the pad named for via-in-pad");
        assert_eq!(r.item, Some(ItemRef::Pad(pad)));
        // Another net's via in the named pad: the clearances to the pad
        // stand, named or not (the hole's is the first one found).
        let r = site(p5v, 20_000_000, &[pad]).unwrap_err();
        assert_eq!(r.reason, "via: hole clearance");
        assert_eq!(r.item, Some(ItemRef::Pad(pad)));
        // And naming a pad of another net admits no via of this one.
        let other = PadId(1);
        let at = board.pads[1].center;
        let r =
            check_via_site(&board, &rules, gnd, at, 600_000, 300_000, &[], &[other]).unwrap_err();
        assert_eq!(r.reason, "via: hole clearance");
        assert_eq!(r.item, Some(ItemRef::Pad(other)));
    }

    #[test]
    fn a_disc_is_inside_a_pad_only_when_one_copper_shape_holds_all_of_it() {
        let mut board = boxed_pad_board();
        let c = Point::new(20_000_000, 20_000_000);
        let at = |dx: Coord, dy: Coord| Point::new(c.x + dx, c.y + dy);
        let mut holds = |shape: Shape, p: Point, r: Coord| {
            board.pads[0].shapes = vec![shape];
            disc_within_pad(&board.pads[0], p, r)
        };
        // A round pad of radius 1 mm.
        let round = || Shape::Circle(Circle::new(c, 1_000_000));
        assert!(holds(round(), at(700_000, 0), 300_000));
        assert!(!holds(round(), at(700_001, 0), 300_000));
        assert!(!holds(round(), c, 1_000_001));
        // An oval pad: a 2 mm long, 1 mm wide stadium along x.
        let oval = || Shape::Segment(Segment::new(at(-1_000_000, 0), at(1_000_000, 0), 1_000_000));
        assert!(holds(oval(), at(1_200_000, 0), 300_000));
        assert!(!holds(oval(), at(1_200_001, 0), 300_000));
        assert!(holds(oval(), at(500_000, 200_000), 300_000));
        assert!(!holds(oval(), at(500_000, 200_001), 300_000));
        // A sharp 2 x 1 mm rectangle.
        let rect = |radius: Coord, half: (Coord, Coord)| {
            let b = BBox::new(at(-half.0, -half.1), at(half.0, half.1));
            Shape::ConvexPoly(ConvexPoly::axis_rect(b, radius).unwrap())
        };
        let sharp = || rect(0, (1_000_000, 500_000));
        assert!(holds(sharp(), at(700_000, 200_000), 300_000));
        assert!(!holds(sharp(), at(700_001, 200_000), 300_000));
        assert!(!holds(sharp(), at(700_000, 200_001), 300_000));
        // The same extent with corners rounded by 0.25 mm, less than the
        // disc's radius: the disc still fits into the corner, and no
        // farther along either edge.
        let rounded = || rect(250_000, (750_000, 250_000));
        assert!(holds(rounded(), at(700_000, 200_000), 300_000));
        assert!(!holds(rounded(), at(700_001, 200_000), 300_000));
        assert!(!holds(rounded(), at(700_000, 200_001), 300_000));
        assert!(holds(rounded(), at(700_000, 0), 300_000));
        assert!(!holds(rounded(), at(700_001, 0), 300_000));
        // A disc smaller than the rounding fits into the corner's arc only.
        let k = 176_776; // 0.25 mm / sqrt(2), rounded down
        assert!(holds(
            rounded(),
            at(750_000 + k / 2, 250_000 + k / 2),
            100_000
        ));
        assert!(!holds(rounded(), at(750_000 + k, 250_000 + k), 100_000));
        // Copper the loader could only approximate holds nothing.
        board.pads[0].shapes = vec![rect(0, (1_000_000, 500_000))];
        board.pads[0].approximated = true;
        assert!(!disc_within_pad(&board.pads[0], c, 300_000));
    }
}
