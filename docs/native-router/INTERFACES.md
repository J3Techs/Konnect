# Native router: frozen interfaces

This is the interface freeze the native-router workstreams build against.
It covers two new crates, `konnect-geom` and `konnect-route`, plus two small
additions to `konnect-sexp`. Read it before you write code against them. When
you need an interface it does not provide, follow the change-control rule
below.

The router plans copper offline. It reads a board model, computes, and
returns a plan in `add_routing` form. It never talks to KiCad itself. The
MCP tools that drive it (the `native_routing` toolset) live in
`konnect-core` and read only saved files.

## Module ownership

Each module has one owner. Only the owner changes a module's behaviour.
Anyone may add tests to any module.

| Owner | Modules | Stage |
|---|---|---|
| INT (integrator) | `konnect-route::api`, `compute::{job, raster, cost, backend}` trait shapes, `plan` format, `telemetry`, `synthetic`, `pipeline` (the stage wiring and the kernel backend negotiation runs on), this document, workspace manifests, the tool registry | — |
| WS-A geometry and checker | `konnect-geom` (all), `konnect-route::check`, `konnect-route::rules` values and semantics | S7; kernel for S2/S5/S6 |
| WS-B ingest, audit, fanout, access | `konnect-route::{board, loader, connectivity, audit, fanout, access}`, the `konnect-sexp` board-scan additions | S0, S1, S2, S4 |
| WS-C global | `konnect-route::global`, `compute::batch` | S3 |
| WS-D1 search | `konnect-route::detail::search`, `CpuCompute::{search_batch, cost_field, derive_masks}` | S5 search |
| WS-D2 negotiation | `konnect-route::detail::negotiate`, `CpuCompute::{paint, unpaint, update_costs}` | S5 control |
| WS-E plan, apply, optimize | `konnect-route::{optimize, plan}` bodies, the job runner and the `native_routing` tools in `konnect-core` (registered by INT) | S6, S8 |
| WS-G GPU backend | a feature-gated GPU implementation of `RouteCompute` (a separate crate, off by default) | — |

Stub functions carry `todo!("WS-x: …")`, which names the owner. They only
panic if called. Replace a stub's body; keep its signature.

## Change control

- **Frozen:** every public type, field, function signature and constant in
  `konnect-route::api`, the `compute` traits and plain-data types, the plan
  format, and the `konnect-geom` public API.
- **To change one:** send the integrator the change and the reason. The
  integrator merges interface changes the same day on the integration branch
  and announces them. Never fork a frozen type locally.
- **Adding is cheap; changing is not.** New functions, new modules and new
  enum variants of non-exhaustive *reports* are fine within your own module.
  Renaming or repurposing an existing item is a change.
- Your own internals are your business: private helpers, caches and
  indexes. So are new fields on your own result types, as long as `Default`
  still builds them.

## Numeric model (frozen)

- **Coordinates** are `konnect_geom::Coord = i32` nanometres in KiCad's
  Y-down board frame. `mm_to_nm` rounds to the nearest nm, as `konnect-ipc`
  does, so a round trip through the plan is exact. Values beyond
  `MAX_COORD` (2^30 nm) are refused.
- **Arithmetic widens:** differences are `i64` (`Vector`), and dot and cross
  products are `i128`. Distance predicates compare products of squares in
  256 bits, so they are exact for every `i32` input.
- **Legality never uses floats.** `f64` appears only when converting mm,
  rotating KiCad pad frames, and approximating arcs.
- **Arcs** are lowered under an explicit `ArcPolicy`; the default
  `max_error_nm` is 1 µm. `Inscribed` is for copper you connect to: add
  `max_error_nm` to any clearance checked against it. `Circumscribed` is for
  obstacles, keepouts and courtyards: its chords are tangents, so it may
  only over-protect.
- **Costs** are `compute::Cost(u32)` fixed point: a straight step is 1000, a
  diagonal step 1414. Adds saturate, and `Cost::INFINITE` absorbs everything.

## Frozen types

### `konnect-geom`

| Item | Meaning |
|---|---|
| `Point { x, y: Coord }`, `Vector { x, y: i64 }` | Position and exact difference. `Ord` on `Point` exists only for deterministic sorting |
| `BBox { min, max }` | A closed box that is never empty; an empty set is `None` |
| `Segment { a, b, width }` | A track or stadium; `a == b` is a dot |
| `Circle { center, radius }` | A disc. `Circle::from_diameter` rounds an odd diameter's radius up |
| `ConvexPoly::new(pts, radius)` | A convex polygon swept by `radius`, which covers round-rect pads. Winding is normalized; non-convex input is refused |
| `Polygon::new(outline, holes)` | A region with holes: zone fills, keepouts, courtyards, outlines. The rings are private (`outline()`, `holes()`) |
| `Shape` | `Segment`, `Circle`, `ConvexPoly` or `Polygon`: a core plus a sweep radius. Deserializing a `ConvexPoly` or `Polygon` runs its constructor's checks |
| `gap_cmp(a, b, clearance) -> Ordering` | The edge-to-edge gap compared with `clearance`; `Less` is a violation |
| `has_clearance`, `touches` | Wrappers over `gap_cmp`. `touches` is the connectivity test |
| `point_segment_distance_cmp`, `segment_segment_distance_cmp`, `segment_polygon_distance_cmp`, `segments_intersect`, `point_in_polygon -> Containment` | The exact primitives |
| `approximate_arc`, `approximate_circle`, `ArcPolicy`, `ArcSide` | The arc policy |
| `LayerIndex`, `LayeredIndex`, `IndexEntry { id: u32, bbox }` | An `rstar` R-tree per layer. `query` returns sorted, de-duplicated ids |

### `konnect-route::api`

- **Ids.** `NetId`, `FootprintId`, `PadId`, `TrackId`, `ArcId`, `ViaId`,
  `ZoneId`, `KeepoutId`, `LayerId`, `ClassId`, `ConnectionId`, `TileId`,
  `ProfileId` and `Generation` are newtypes, and each id equals its index in
  the matching `RouteBoard` vector.
  - `NetId`s follow sorted net-name order.
  - `LayerId`s follow physical stack order: `F.Cu` is 0, then `In1.Cu` to
    `InN.Cu`, and `B.Cu` is last.
  - `ClassId(0)` is always `Default`.
- **`LayerMask(u32)`** holds bit `i` for `LayerId(i)`.
- **`Ownership`** is one of:
  - `Fixed`: existing copper the router may not move;
  - `Locked`: KiCad's lock flag, never moved;
  - `Routed { net, gen }`: copper this router made, which only generation
    `gen` may rip up.
- **`ItemRef`** refers to any copper item; **`ItemKind`** is the item's DRC
  `Type` (`kicad_type_name()`).
- **`Connection { id, net, from, to }`** is one routing task;
  **`FailureReason`** is why it failed. Failures are always reported, never
  dropped.
- **`Stage`** names S0 to S8, for telemetry.
- **Constants (the lattice and window freeze):**

  | Constant | Value |
  |---|---|
  | `LATTICE_PITCH_NM` | 50 µm |
  | `LATTICE_FINE_PITCH_NM` | 25 µm |
  | `GCELL_PITCH_NM` | 1 mm |
  | `GCELL_FINE_PITCH_NM` | 0.5 mm |
  | `TILE_CORE_NM` | 20 × 15 mm |
  | `TILE_HALO_NM` | 2 mm |
  | `GPU_WINDOW_MAX_NM` | 2^24 nm, about 16.78 mm, including the halo |

  GPU windows are uniform lattices. Fine-pitch fields are separate uniform
  windows stitched at fixed portals. CPU tiles may use the pad-aligned
  non-uniform lattice.

### `konnect-route::board::RouteBoard`

A `RouteBoard` is plain data:

- `outline`:
  - `regions`, filled only when the outline is provably a rectangle;
  - `edges`, the segments that edge clearance is measured to: zero width
    for straight pieces, and wide enough on curved ones to cover the true
    curve on both sides;
  - `bbox`.
- `layers` (`LayerTable`, with `expand("*.Cu" | "F&B.Cu" | name)`) and
  `routable`.
- `nets` (`name`, `class`, diff-pair `pair`).
- `footprints`, each with a reference, side, lock state, courtyard polygons
  and `pad_range`.
- `pads`. Each pad has:
  - a board-space `center` (the anchor, where the hole is; a drill offset
    moves only the copper) and `angle_deg`;
  - a copper `layers` mask;
  - `shapes`, a union of exact shapes;
  - an optional `drill`;
  - an optional `local_clearance`, the pad's own or else its footprint's;
  - an `approximated` flag.
- `tracks`, `arcs`, `vias` and `zones`. Each zone piece is one fill polygon
  on one layer and one net. An unfilled zone uses its outline, with
  `filled: false`.
- `keepouts`.

`validate()` checks the invariants every producer must keep.

### Rules: `konnect-route::rules`

- **`RuleTable`** answers:
  - `class_of`, `class`;
  - `clearance(a, b, layer)`;
  - `track_width`, `via`;
  - `min_track_angle_deg`, `min_segment_length`;
  - `edge_clearance`, `hole_clearance`, `hole_to_hole`;
  - `max_clearance`, which is the index inflation bound;
  - `inert_rules`.

  Each item is described by a `RuleSubject`: its kind, net, owning
  footprint, the courtyards it intersects, the areas that enclose it, and
  any local clearance.
- **`ProjectRuleTable::from_project(pro_json, dru_text, board)`** compiles a
  project's own files at runtime. The code fixes only the rule *kinds* it
  supports:
  - net class clearance, width, via and diff-pair values;
  - board minimums;
  - edge, hole and hole-to-hole clearance;
  - custom `clearance` overrides with conditions on `NetName`, `NetClass`,
    `Type`, `Reference`, `intersectsCourtyard()` and `enclosedByArea()`;
  - via sizes: `via_diameter` and `hole_size` (`min`, `opt`, `max`) and
    `annular_width` (`min`); a via's diameter never drops below its drill
    plus twice the annular floor;
  - `track_width` (`min`, `opt`, `max`), `track_segment_length` and
    `track_angle` (`min`).

  Anything else is reported inert with a reason. That includes a condition
  that names an area or footprint the board lacks, a limit the table does
  not apply (such as a `max` on a clearance), a length too large for a
  `Coord`, a rule with `(severity ignore)`, and a via constraint scoped to
  one layer.
- **Precedence is a hypothesis until the oracle confirms it.** For
  clearance, following KiCad's `DRC_ENGINE::EvalRules`:
  1. a local override on either item (the pad's own `(clearance …)`, else
     its footprint's), the larger of the two, beats every rule;
  2. otherwise the last matching custom rule, with two-item conditions tried
     in both orders;
  3. otherwise the net class (the larger of the two items' values).

  Other constraints take the last matching custom rule, then the net class.
  Board minimums are a floor under every result, and a floor beats a
  conflicting `max`.
- **Deviation from the plan.** The plan has week 1 run on a hard-coded table
  of one board's rules. The table is compiled from the project files
  instead, so no board's values live in this public repository and any
  project loads the same way.
- **`synthetic::UniformRules`** is a constant table for tests.

### Connectivity: `konnect-route::connectivity`

`ConnectivityReport::build(board, policy)` runs union-find over same-net
copper that touches on a shared layer, using `touches`. It returns islands
per net.

- Zone pieces with no fill are left out, because their outline only bounds
  the copper. `unfilled_zones_skipped` counts them.
- `connection_count()` is the sum over nets of pad islands minus one.
- `connections(board)` builds a deterministic minimum spanning tree over each
  net's pad islands and returns the routing task list.

### GPU-shaped compute seams: `konnect-route::compute`

- **`TileRaster`.** For each (tile, layer, profile), one flat plane per
  field, with every row padded to 4-byte words (`Plane<T>`):
  - `core` and `halo` (`u8`);
  - `present` and `history` (`u16`);
  - `heading_mask` and `via_site` (`u8`).

  A `TileStack` holds a tile's rasters for every routable layer.
- **Tri-state contract.** `classify(core, halo)` gives one of three states:
  - `Blocked` when `core > 0`;
  - `Free` when `halo == 0`, where an edge between two `Free` nodes needs no
    check;
  - `Near` otherwise, which needs an exact check.

  `core` is painted with the obstacle inflated by clearance, half the track
  width and the sagitta bound; `halo` adds half a cell diagonal. A counter
  that reaches 255 stays there until the raster is rebuilt.
- **`JobDescriptor`.** Exactly 64 bytes, `#[repr(C)]` and asserted at
  compile time. Variable-length endpoints and corridors live in a
  `JobPool`, as `JobNode`s of 8 bytes and GCell ids.
- **`BatchBuilder::build(pool, windows) -> Vec<Batch>`.** Deterministic, with
  pairwise-disjoint windows in each batch and first-fit placement in
  `order_key` order.
- **`SearchBackend`.** `search_batch` and `cost_field`.
- **`RouteCompute: SearchBackend`.** `paint`, `unpaint`, `derive_masks`,
  `update_costs`, and `fork` (an independent copy per worker thread, or
  `None` for a device-bound backend; negotiation then runs lockstep batch
  rounds instead of whole tiles per worker).
- **`CpuCompute`** is the reference. `paint`, `unpaint` and `update_costs`
  are the raster reference; `search_batch`, `cost_field` and `derive_masks`
  are WS-D1's kernel with default tuning (see *Detailed search* below).
- **A GPU only proposes.** Outcomes are lattice paths (`PathProposal`). The
  CPU re-checks every proposal exactly and commits in deterministic order.
  On device loss, the run latches to the CPU for its remainder.

### Plan: `konnect-route::plan` (the S8 format)

`RoutedCopper` (`RoutedTrack` and `RoutedVia`, in ids and nm) becomes a
`RoutePlan` through `RoutePlan::from_routed(board, copper)`. A `RoutePlan`
serialises exactly as an `add_routing` `routing_file`:

```json
{ "tracks": [ { "net": "…", "layer": "F.Cu", "width": 0.1, "x1": 0, "y1": 0, "x2": 1, "y2": 0 } ],
  "vias":   [ { "net": "…", "x": 1, "y": 0, "drill": 0.3, "diameter": 0.6 } ] }
```

- Only these two keys appear. `locked` is the tool argument, not part of the
  file.
- Vias must be through vias.
- `validate()` applies `add_routing`'s member checks.
- `chunks(ADD_ROUTING_MAX_ITEMS)` splits a plan for calls of at most 20,000
  items each.

### Telemetry: `konnect-route::telemetry`

`Telemetry` records:

- per-stage wall time (`time(stage, f)`);
- histograms of A* expansions per connection and of window nodes;
- time spent in congested iterations;
- raster rebuild time;
- the thread count;
- named counters.

Use `merge` to fold a worker's telemetry into the run's.

### `konnect-sexp` additions

- **`board::zone_geometry`** reads the outline and its holes (every later
  `(polygon …)` is a hole), every `filled_polygon` with its `island` flag,
  the priority, the name and keepout flags.
- **`board::footprint_courtyard_outlines`** returns closed courtyard
  outlines in board coordinates. `footprint_courtyard_outline` does the same
  for one footprint. `fp_line` and `fp_arc` pieces are chained into loops in
  local coordinates; open courtyards are skipped and counted.

## Integrated interfaces

Everything below was added by the workstreams after the freeze and merged
on the integration branch. All of it is additive: no frozen type, field or
signature changed meaning except where a bullet says so.

### The pipeline: `konnect-route::pipeline` (INT)

`route_board(board, rules, scope, settings) -> Result<PipelineResult, _>`
runs the stages in order, on WS-D1's kernel (`route_board_with` takes any
`RouteCompute + Send` backend instead):

1. S1 connectivity of the board as loaded;
2. S2 plane fanout (`PipelineSettings::fanout`, `None` skips it), kept to
   the scope and added to a working copy of the board as **locked** copper;
3. S1 again over the working board, then the scope
   (`pipeline::select_connections`: nets, and connections whose two
   terminals lie wholly inside the region; ids renumbered to their index);
4. S4 pin access, one profile per net class of the selected connections'
   pads (reported; negotiation still derives its own terminal nodes);
5. S3 `build_grid_with` and `route_global_with` over that task list;
6. S5 `negotiate_connections` with the same task list and the grid. A
   connection S3 failed gets an empty corridor on its class layers, so a
   coarse-grid failure is not final;
7. S6 `optimize_checked` (`PipelineSettings::optimize`), then a **guard**:
   the S7 checker counts each net's findings before and after, and any net
   the optimizer edited that gained a finding is put back to its routed
   copper, repeated until none gains;
8. S7 `BoardChecker::check_routed_timed` over the fanout and routed copper
   together, against the board as loaded;
9. S8 one `RoutePlan` (fanout first), and the connections still open on the
   board with the plan applied.

Tuning round 2 (merged together; all additive):

- **`PipelineSettings`** gained `use_access` (hand S4's access points to
  negotiation as pad stubs, default on), `neck_down: Option<NeckDown>`
  (default `None`), `class_fallback: BTreeMap<ClassId, ClassFallback>`
  (default empty) and `search_min_run` (**default on**, changed by INT at
  the round-2 merge).
  - The default negotiation schedule is 10 iterations, present-factor
    growth 2,000 (×1000) and a history step of 3, not
    `NegotiationSettings::default()`'s gentler stand-alone schedule.
  - `search_min_run` asks the kernel for a minimum run only on nets with a
    `track_segment_length` rule (`backend::min_run_nets`), at the longest
    such minimum, which then replaces any `min_run_nm` in `search`. A job
    that finds no path under its minimum run is searched again without it
    (`KernelBackend::min_run_retry`, default on), so the rule never costs a
    connection by itself. Measured at the merge on a dense real board: no
    short segment from detail and no S7 finding after S6, against a few of
    each with it off, for about as many routed connections (the difference
    was within the run-to-run spread that any small change causes).
- **`ClassFallback { layers, reference: Option<ReferencePlane> }`** and
  **`ReferencePlane { plane, over, margin }`**: after negotiation (step 6),
  the connections of such a class that are still failed are routed again
  on `layers`, around every copper routed so far. Tracks on `over` must lie
  over one filled piece of `plane` grown by `margin`; a net whose tracks
  leave it is dropped again. KiCad's fills are fractured, so antipads, pad
  clearances and voids count as off the reference. Counter:
  `pipeline_fallback_routed`; its time is charged to `Stage::Detail`.
- **`optimize_guarded(work, rules, routed, opts, guard, tel)`** is step 7 on
  its own and **`add_locked(board, copper)`** is public, so saved copper can
  be re-optimized offline exactly as the pipeline does.
- **`pipeline::backend`**: `KernelBackend` gained the public field
  `min_run_nets: Option<Arc<Vec<bool>>>` (by `NetId` index; `None` flags
  every job when `min_run` is on) and `min_run_retry: bool` (default on).
  Build it with `KernelBackend::new` rather than a struct literal. `min_run_nets(board, rules)` returns the flagged
  nets and the longest minimum. `FIELD_AFTER = 2_000`, which
  `KernelBackend::negotiation_config()` sets as `SearchConfig::field_after`.

**Live round 2: clearing what S7 still finds.**
`PipelineSettings::clear_findings` (**default on**) runs
`pipeline::clear_findings(work, rules, &mut routed) -> ClearReport` after S6
and its guard, so the plan is S7-clean: per net, one named routed track that
is spare (the net's pads stay as joined without it), else the named track
that opens the fewest joins, with the routed copper that then dangles;
whole joined pieces only for findings naming routed vias alone; repeated
until S7 names no routed copper. Then, anywhere, routed tracks with an end
touching nothing of their net and routed vias joined on one layer or none,
whenever the net's pads stay as joined. `ClearReport { single_tracks,
cut_tracks, opened, dangling_tracks, spurs, lone_vias, pieces, tracks,
vias, nets, findings }`; counters `pipeline_cleared_*`; time charged to
`Stage::Check`. `PipelineResult::cleared` holds it. A struct literal of
`PipelineSettings` needs the new field.

Every stage records its time in the result's `Telemetry` (`Stage::Audit` for
connectivity, `Fanout`, `Access`, `Global`, `Detail`, `Optimize`, `Check`
for the guard and the check, `Plan`) plus `pipeline_*` counters. The
`konnect-core` job runner's `CopperSource::Native` calls `route_board` with
`optimize: None` and runs S6–S8 itself, so it can checkpoint between them;
its `Checkpoint` and `PipelineReport` gained the fanout copper
(`fanout`, `fanout_tracks`, `fanout_vias`), which is board copper for S6/S7
and part of the plan.

### Detailed search and negotiation (WS-D1, WS-D2, INT)

- **`PathProposal::nodes` from the kernel are corner nodes**: the source,
  every node where the heading changes, both ends of each via (same
  `(col, row)`, consecutive), and the target; consecutive nodes on one
  layer are joined by one straight octilinear run. Each node's
  `heading_mask` holds the heading the path leaves it in.
- **`pipeline::KernelBackend`** is the backend negotiation runs on. It
  expands kernel paths to one node per lattice step (negotiation's exact
  check vetoes single steps), delegates raster maintenance to `CpuCompute`,
  and forks by cloning. `KernelBackend::negotiation_config()` turns the
  derived heading mask off (negotiation derives masks before it lifts a
  net's own copper; the kernel reads the live CORE counters instead) and
  turns via congestion on. `ReferenceSearch` (WS-D2) stays for tests and
  comparison.
- **`via_site` values**: `VIA_NO = 0`, `VIA_CHECK = 1` (an exact check
  decides), `VIA_FREE = 2`. Negotiation narrows sites to where its
  profile's whole via fits, so a nonzero site there is free of HALO.
- **Non-uniform lattices** (`Lattice`, `LatticeSpec`): rasters on them are
  painted with `paint_on_lattice` and derived with `derive_masks_on_lattice`;
  `Lattice::from_axes` refuses an axis gap above √2 · base pitch. The map
  lookahead is used on uniform lattices only.
- **`SearchConfig`** (kernel tuning beside the frozen `CostParams`) gained
  `via_congestion_reach_nm: Option<Coord>` (serde default `None`): a via is
  priced by the worst `present`/`history` on every stack layer at its node
  and at eight points this far out, `b · (1 + h·hf) · (1 + p·pf)`. Without
  it a via sits on another net's routed track at no cost and negotiation
  never clears the conflict.
- **`SearchStats`** is `#[serde(default)]` and gained `min_run_rejects`.
- **`compute::Plane::as_mut_slice`** beside `as_slice`.
- **`NegotiationTuning`** is `#[serde(default)]` and gained
  `raster_cache_mb` (default 4096, 0 off; never changes the result) and
  `bga_min_fill` (×1000, default 350). **`DetailResult`** gained
  `short_segments: Vec<ShortSegment>`. `negotiate_connections(board, rules,
  connections, global, grid, settings, tuning, backend, telemetry)` reads
  corridors by connection id from `connections` and decodes their cells
  (base and refined) with `GCellGrid::cell_bbox`.

- **Relaxed cost-to-go field (WS-D1/T3).** `SearchConfig.field_after:
  u64` (serde default 0 = off): after that many expansions the search
  restarts with a relaxed cost-to-go field in its heuristic. The field also
  bounds passing through a target whose entry mask refuses the arrival
  heading, so it is consistent: path costs are unchanged and only ties may
  differ. `SearchStats` gained `field_runs` and `field_settled` (telemetry
  `search_field_runs`, `search_field_settled`; `KernelBackend` does not
  forward them yet). `search_batch_with` runs a one-job batch on the
  calling thread with the same result.
- **`NegotiationTuning`** gained:
  - `grid_core: (Coord, Coord)` (default `TILE_CORE_NM`), snapped to the
    base lattice and never below two halos plus 0.5 mm per side; the
    odd-iteration half-tile shift is snapped too, which changes nothing at
    the default size;
  - `fill_passes` (default 2): after the last iteration, every connection
    still failed (not for lack of access or corridor) is routed again with
    all other routed copper as a hard wall, alternating the tiling; a fill
    pass never undoes routed copper;
  - `present_factor_max` (default 1,000,000; costs saturate at 32 bits, so
    an unbounded factor turns other nets into walls);
  - `stub_reach` (default 0.5 mm), the longest full-width pad stub to a
    lattice node;
  - `shift_iterations` (default 6): odd iterations below it use the shifted
    tiling, later ones keep the unshifted tiling so conflicts reroute only
    their pieces;
  - `partial_tries` (default 3): piece-only reroutes before a connection is
    cut again whole;
  - `stall_iterations` (default 0, off): stop once the conflict count has
    not improved for that many iterations.
- **`negotiate_connections_with(…, inputs: &NegotiationInputs, …)`**, with
  `NegotiationInputs { access: Vec<PadAccess>, neck_down: Option<NeckDown> }`;
  `negotiate_connections` keeps its signature and passes empty inputs.
  `NeckDown { width, reach, excluded }` (defaults 0.10 mm within 1 mm, no
  classes excluded) narrows tracks near pads for every class not excluded.
- **`DetailResult.congestion_lost`**: connections that lost a negotiation
  before the fill passes. **`IterationReport.fill`** marks fill passes.
  Unresolved conflicts keep a greedy independent set (fewest conflicts
  first); the losers are reported as `DetailFailure::Congestion { against }`,
  and a loser that a fill pass cannot route still reports `Congestion`, not
  `Unreachable`, `NoPin` or `BudgetExhausted`. Callers of the job runner and
  the MCP tools therefore see `congestion` where they used to see
  `no_corridor` for those connections.
- **Terminal headings.** S4 access headings, the kernel's masks and
  `TileRaster::heading_mask` share one encoding: 0 = +x, counter-clockwise
  in the Y-up sense; `detail::search::heading_delta(h)` is the board step
  (rows grow with board y). Negotiation's pad stubs and tile pins use it.
  A connection ending on a fixed track's end gets only the headings that
  keep the joint within the net's `track_angle` minimum (a source leaves in
  them, a target is arrived at in the reverse ones; no rule allows all).
- `ConflictSet::pieces_of(pair)` gives the (task, piece) of each side;
  `PinRegistry::marks` and `PinRegistry::add` are public.
- Telemetry: a `us_tile_search` counter (search time inside tiles).
  `KONNECT_DAG_TRACE=1` prints each iteration's tile critical path to
  stderr.

### Global routing (WS-C)

- **`GlobalSettings`**, `PresentSchedule`, `GlobalCosts` and `LayerPolicy`
  (per-class layer masks) configure S3; `build_grid_with` and
  `route_global_with` record telemetry.
- **`GCellGrid`** gained `fine_pitch`, `refined` (base cells split into
  fine cells, ascending) and `capacity`. A base cell keeps the frozen
  `GCellGrid::id`; fine cells follow every base id (`fine_id`). Use
  `cell_bbox(id)` and `locate(layer, point)` rather than decoding ids. The
  pitch coarsens until slots × cells ≤ 2^23; `route_global` refuses a grid
  beyond the limits (every connection `NoCorridor`).
- **`Corridor.layers`** keeps its frozen meaning, the layers the corridor
  may use; the new **`Corridor.assigned`** (serde default) is the subset its
  cells are on. A layer change is two consecutive ids of one area on two
  layers: one through via.
- **`GlobalStats.best_iteration`** (serde default): the result is the best
  iteration, not the last. `converged` also requires no failures; a
  terminal off the grid fails as `NoAccess`.

### Checker and geometry (WS-A)

- `CheckReport` keeps its frozen shape `{ violations }`; counts come from
  `BoardChecker::check_board_with_stats` / `check_routed_with_stats` as
  `(CheckReport, CheckStats)`, and the `*_timed` methods record them.
- `check_routed` uses `CheckOptions::for_routed()`: zone fills are left to
  KiCad's refill, and fixed copper is never re-checked against fixed copper.
- `konnect-geom` gained `IndexedPolygon` (edge-indexed polygons),
  `measure::{gap_nm, closest}` (f64, reports only),
  `point_shape_distance_cmp(p, width, shape, clearance)`,
  `segment_shape_distance_cmp(a, b, width, shape, clearance)` and
  `shape_within_polygon`.
- `RuleTable::clearance` answers must not depend on argument order;
  memoizing callers ask in one fixed order. `ProjectRuleTable::pin_net_class`
  also clears the net from `ambiguous_nets`.
- **Track angles, pairwise (live round 2).** New `check::junction`:
  `intersect(a, b)` (KiCad's segment intersection: ends count, collinear
  overlaps meet at the overlap's middle) and `pair_angle(a, b) ->
  Option<PairAngle { at, angle_deg, shared_vertex, collinear }>` (KiCad's
  measurement: ends on the point face away from it, a point along a track
  keeps its drawn direction and folds the angle below 90°). S7 adds a pass
  over every pair of same-net tracks on a layer whose centre lines meet away
  from a shared end (branches on a run's middle, crossings, collinear
  overlaps), routed against routed and board tracks, off pads. Behaviour
  change: a collinear overlap from a shared end is now reported at 0° (a
  live KiCad 10 DRC reports it); the oracle still matches with 0 false
  positives.
- A junction counts as on a pad only 5 µm (`PAD_HIT_INSET_NM`) inside the
  pad's outline; a bend on the very edge of a pad is reported, as KiCad
  does.

### Audit, fanout and access (WS-B)

- `AuditSettings` gained `max_states` (default 8,000,000) and `exact_only`;
  `FootprintAudit` gained `coarsened`, `classes`, `paint_ms`, `search_ms`;
  `AccessStats` gained `unknown_profile` (a profile naming no class the
  board uses: every pad is returned without points).
- `PlaneViaPlan::routed()` returns vias with an empty layer mask (the plan
  does not hold the stack); `route_plan(board)` and the pipeline fill in
  every copper layer.

- `FanoutSettings` gained `exclude_escaped_arrays` (default on),
  `array_min_pads` and `partial_plane_fallback` (default on);
  `FanoutStats` gained `escaped_arrays` and `partial_plane_sites`.
  `fanout::escaped_arrays(board, connectivity, min_pads)` lists ball-grid
  footprints whose escapes are drawn by hand; the fanout leaves every pad of
  them alone. The partial-plane pass places a via on a plane piece that
  covers only part of a pad's reach.

### Optimizer and job runner (WS-E)

- `optimize::record_counters(&OptimizeReport, &mut Telemetry)`.
- `OptimizeOptions::max_skew_deg: 0.0` also turns off the 1 µm snap, so a
  straight shortcut must be exactly octilinear.
- `konnect-core::native_route_job`: `StopKind` (typed stop reasons),
  `PipelineReport.stop_kind`/`resumed`, `CheckSummary.check_routed_error`.
- Live round 2: the runner's S6 is `pipeline::optimize_guarded` (the S7
  guard it lacked) with the sliver floor filled from the rules, followed by
  a `clear` stage (`pipeline::clear_findings`) before S7.
  `NativeJobSpec::clear_findings: Option<bool>` (serde default `None`: on
  for native routing, off for a plan or the board's own copper; the
  `clear_findings` argument of the job tools). `PipelineReport` gains
  `optimize_reverted` and `cleared: Option<ClearReport>` (both
  `#[serde(default)]`), also in the job status. The runner's route stage
  calls `route_board` with `clear_findings: false`. A struct literal of
  `NativeJobSpec` needs the new field.
- `OptimizeOptions` gained `prune_stubs` (**default on**): delete a track
  that makes an acute bend, or a dangling single-track branch off a
  junction, pad or via, when everything it touched stays joined without
  it. A dangling run of two or more tracks is never eaten back. The job
  runner uses the default, so imported (for example Freerouting) plans now
  change visibly in S6. `OptimizeReport::stubs_pruned` is
  `#[serde(default)]`, so old checkpoints still load.
- **Live round 2.** The lint (`lint_group`) also reports KiCad's pairwise
  track angles ([`check::junction`]) as `AcuteBend` (two fixed pieces
  excepted), so no edit adds one; stub pruning offers the tracks of such
  pairs for deletion. New `OptimizeOptions::sliver_floor_nm: Option<Coord>`
  (default `None`; `Some(0)` off): on nets with no `track_segment_length`
  rule, the geometry gate counts shorter segments as short too.
  `optimize::sliver_floor(board, rules)` is the board's smallest rule
  minimum; the pipeline and the job runner fill `None` with it. A struct
  literal of `OptimizeOptions` needs the new field.
- `OptimizeOptions::max_skew_deg` now defaults to 0.0: only exact
  octilinear pieces, since a skewed run meeting a right angle makes an
  interior angle just under 90°. The optimizer never accepts an edit that
  trades one DRC-reported defect for another.
- `lint::DefectKind::is_drc_kind()`: whether KiCad's DRC reports the kind
  too (`track_angle`, `track_segment_length`).
- The live apply path: `apply_route_plan` (the `native_routing` toolset)
  creates a plan of any size in the open board through
  `route_native_apply::apply_plan_with` (net-whole chunks, `strict_geometry`
  off counts the lint's defects in `ApplyLedger::geometry_defects`) and
  writes the KIID ledger; `map_route_plan_drc` reads a `run_drc` report back
  onto the plan through that ledger.
- `konnect-ipc::retry_while_busy` and `KiCadIpcClient::{save_board_when_ready,
  refill_zones_when_ready}`: KiCad 10 answers reads during a zone fill and
  refuses saves with AS_BUSY, so the DRC sync and `save_project` wait out
  AS_BUSY on the save itself.

### Allowances, net classes, class policy and fanout retry (WS-A4)

- **Net classes as KiCad 10 resolves them** (`rules::netclass`, private).
  `ProjectRuleTable::from_project` now matches a `netclass_patterns`
  entry against the **whole** net name as an anchored regular expression
  or a wildcard pattern, expands bus-notation patterns, collects every
  matching class (the highest-precedence one, lowest `priority` then
  name, is taken and the net reported in `ambiguous_nets`; it used to be
  the first match in file order), and, when nothing matches, gives a net
  whose name parses as a bus (`D[0..3]`, `PREFIX{A B}`) the class all its
  members share. A name with an escaped character such as `{slash}` is a
  one-member bus group, which is how KiCad puts it in a class whose
  pattern is the escape word. Behaviour change: such nets move out of
  `Default`. Workspace dependency `regex = "1"` (MIT/Apache-2.0).
- **`pipeline::allowances`**: `RoutingAllowances { class_layers,
  layer_fallback: Vec<LayerFallbackSpec>, neck_down: Option<NeckDownSpec>,
  exclude_classes: Option<Vec<String>>, fanout_share_within_mm }` names
  classes and layers as the project does (serde, unknown fields refused);
  `LayerFallbackSpec { class, layers, nets, reference: Option<ReferenceSpec
  { plane, over, margin_mm }> }`, `NeckDownSpec { width_mm (0.10),
  reach_mm (1.0), classes (allow list), exclude_classes }`.
  `resolve(board, classes, &mut PipelineSettings) -> Result<AllowanceReport,
  AllowanceError>` checks every name and writes the class layers,
  fallbacks, neck-down, exclusions and fanout sharing. Everything is off
  unless named, except the class policy: `exclude_classes: None` means
  `default_excluded_classes(classes)`, every class whose track width is
  at least `WIDE_CLASS_MIN_WIDTH_NM` (0.5 mm); `Some([])` routes every
  class.
- **`PipelineSettings::excluded_classes: BTreeSet<ClassId>`** (default
  empty): those classes' open connections go to
  **`PipelineResult::excluded`** (renumbered), never to the task list or
  the failures; `pipeline::split_excluded`; counter
  `pipeline_excluded_by_policy`. A struct literal of `PipelineSettings`
  needs the field.
- **`ClassFallback::nets: Option<BTreeSet<NetId>>`** (serde default
  `None`): only those nets of the class get the second chance. A struct
  literal of `ClassFallback` needs the field.
- **`pipeline::enforce_reference(work, rules, fallback, before, routed) ->
  ReferenceReport { tracks_checked, reverted_nets, dropped_nets }`** runs
  after S6 (in `route_board` and in the job runner) whenever a fallback
  names a reference: a net of such a class with a track off the reference
  on the covered layers goes back to its negotiated copper, or loses its
  copper. `PipelineResult::reference`; counters `pipeline_reference_*`.
  Reference pieces are edge-indexed (`IndexedPolygon`); the second
  chance's setup fell from about 35 s to 0.1 s on a dense real board.
  Counters `pipeline_fallback_tasks`, `pipeline_fallback_us_setup`,
  `pipeline_fallback_us_negotiate`.
- **Fanout retry.** `FanoutSettings::{retry_failed (default on), far_gaps
  (2.0, 2.5, 3.0 mm), far_directions (32)}`: islands the selection leaves
  without a site get every exit pass plus the far exits and are placed,
  serially, against every chosen site; first-round sites never move.
  `FanoutStats::retried_sites`; `FailureDetail::blocked_by` (the pads whose
  chosen sites block a `Conflict`). Behaviour change: the default fanout
  places more sites. Via sharing (`share_within`) stays off by default.
  Same-net candidates whose stubs touch on one layer are incompatible
  (KiCad measures the angle wherever same-net tracks meet).
- **Job runner and tools (`konnect-core`).** `NativeJobSpec` gained
  `allowances: RoutingAllowances`, `net_classes: Option<BTreeMap<String,
  Vec<String>>>` (KiCad's effective class name, then its constituents) and
  `fanout_plan: Option<PathBuf>`, all omitted from the serialised spec at
  their defaults (old spec hashes are unchanged). `PipelineReport` gained
  `net_classes: Option<NetClassSource>`, `allowances`,
  `excluded_by_policy` (per class), `reference` and `fanout_plan_path`, also
  in the job status. `start_native_route_job` and `route_native_region`
  take `class_layers`, `layer_fallback`, `neck_down`, `exclude_classes`,
  `fanout_share_within_mm`, `fanout_plan` and `net_classes` (`auto`,
  `kicad`, `project`). Behaviour change: a native job now leaves
  wide-copper classes unrouted by default. `plan_plane_vias` takes
  `share_within_mm` and `retry_failed` and reports each failed pad's
  reasons and blockers. The recommended live sequence is in
  [`LIVE-FLOW.md`](LIVE-FLOW.md).
- **Tests.** `KONNECT_PIPELINE_ALLOWANCES` (JSON or a file) in
  `pipeline_real_fixture`; gated `netclass_real_fixture` (resolved classes
  against KiCad's effective classes in a Specctra export) and
  `fanout_real_fixture` (each failed pad's reason).

### Workspace

- `rayon = "1.10"` in the workspace manifest (MIT/Apache-2.0).

### Known conflicts left open

- Negotiation's grid windows are about 24 × 19 mm, larger than the frozen
  `GPU_WINDOW_MAX_NM` (about 16.8 mm). A GPU backend has to split them or
  the constant has to change.
- The kernel's `SearchContext::gcells` corridor steering reads base cell
  ids only; negotiation does not pass corridors to the kernel yet (it uses
  them as guide lines and layer masks).

## Determinism rules

- 1-thread and N-thread runs must produce byte-identical plans.
- Never iterate a `HashMap` or `HashSet` where order can reach output; use
  `BTreeMap`, or sort first.
- Index queries return sorted ids. Batches commit in batch order, then in
  `order_key` order.
- Break ties explicitly, by id.

## Tests and fixtures

- **Unit tests** live beside the code (`#[cfg(test)] mod tests`). Build boards
  with `synthetic::BoardBuilder` or the generators: `two_pin_nets`,
  `plane_with_pads` and `bga_field`. Pair them with `UniformRules`.
- **Loader tests** use `synthetic::MINIMAL_KICAD10_BOARD`, a hand-written
  KiCad 10 board.
- **Commit only synthetic fixtures** generated in code, or small
  hand-written ones. This repository is public: never commit a real
  project's board data, file contents, net names, counts or evidence.
- **Real boards stay outside the repository.** A test that needs one reads
  the directory from `KONNECT_DB1_FIXTURE_DIR` and finds the files by
  extension (`.kicad_pcb`, `.kicad_pro`, `.kicad_dru`). When the variable is
  unset or the directory is missing, the test prints why and returns early;
  it does not fail. Such a test asserts only invariants that hold for any
  KiCad-authored board, and prints its counts. See
  `crates/konnect-route/tests/real_board_fixture.rs`.
- `tests/pipeline_real_fixture.rs` reads its knobs from
  `KONNECT_PIPELINE_*` variables (listed in its module docs, including
  `KONNECT_PIPELINE_GRID_CORE_MM` and `KONNECT_PIPELINE_CLEAR=0|1`).
  `tests/pipeline_tail_fixture.rs` re-runs
  S6–S8 on copper a previous run saved. `tests/search_replay.rs` is a gated
  capture/replay pair; the capture refuses a directory inside any git work
  tree, because captures hold board data.
- **Keep generated runs in ignored build or cache locations.**
- **Gate before merging:** `cargo clippy -p <crate> --all-targets -- -D
  warnings`, `cargo test -p <crate>` and `cargo fmt --check` for every crate
  you touched.
- **Dependencies:** permissive licences only (MIT, Apache-2.0, BSD or ISC).
  Port ideas from GPL routers; never copy their code.
