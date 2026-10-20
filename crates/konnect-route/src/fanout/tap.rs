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
//! edge, keepouts, the via floor), a via never lands on an SMD pad of its
//! own net, and sites of different pads are placed so they clear each other.
//!
//! A through-hole pad already reaches every layer: it gets no via and is
//! reported as its own tap.
//!
//! The stub runs from the pad's copper centre to the via centre, so every
//! via is anchored by a track. KiCad gives a via with no attached track the
//! net of the zone it sits in; the stub keeps the planned net.

use super::*;

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
    /// Stub from the pad's copper centre to the via, on `layer`.
    pub stub: Option<(Point, Point, Coord)>,
    pub layer: LayerId,
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

/// Plan a tap for each of `pads` (a pad listed twice is planned once).
/// `target` is the layer the taps must reach: every via is a through via,
/// and it must also clear other nets' zone fills on `target`.
pub fn plan_tap_vias(
    board: &RouteBoard,
    rules: &dyn RuleTable,
    pads: &[PadId],
    target: LayerId,
    settings: &FanoutSettings,
) -> TapPlan {
    let t0 = std::time::Instant::now();
    let mut s = *settings;
    s.require_own_plane = false;
    s.partial_plane_fallback = false;
    s.clear_of_zones_on = Some(target);
    let policy = ArcPolicy::default();
    let space = Space::new(board, rules, &policy, true);
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
                        stub: Some((c.stub.a, c.stub.b, c.stub.width)),
                        layer: c.layer,
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
        let plan = plan_tap_vias(&board, &rules, &pads, target, &FanoutSettings::default());
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
            &FanoutSettings::default(),
        );
        assert_eq!(plan.sites.len(), 1);
        assert_eq!(plan.sites[0].via, None);
        assert_eq!(plan.sites[0].at, board.pads[0].center);
    }
}
