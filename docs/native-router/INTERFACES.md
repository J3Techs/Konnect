# Native router: frozen interfaces

This is the interface freeze the native-router workstreams build against.
It covers two new crates, `konnect-geom` and `konnect-route`, plus two small
additions to `konnect-sexp`. Read it before you write code against them. When
you need an interface it does not provide, follow the change-control rule
below.

The router plans copper offline. It reads a board model, computes, and
returns a plan in `add_routing` form. It never talks to KiCad itself, and it
registers no MCP tools yet.

## Module ownership

Each module has one owner. Only the owner changes a module's behaviour.
Anyone may add tests to any module.

| Owner | Modules | Stage |
|---|---|---|
| INT (integrator) | `konnect-route::api`, `compute::{job, raster, cost, backend}` trait shapes, `plan` format, `telemetry`, `synthetic`, this document, workspace manifests | — |
| WS-A geometry and checker | `konnect-geom` (all), `konnect-route::check`, `konnect-route::rules` values and semantics | S7; kernel for S2/S5/S6 |
| WS-B ingest, audit, fanout, access | `konnect-route::{board, loader, connectivity, audit, fanout, access}`, the `konnect-sexp` board-scan additions | S0, S1, S2, S4 |
| WS-C global | `konnect-route::global`, `compute::batch` | S3 |
| WS-D1 search | `konnect-route::detail::search`, `CpuCompute::{search_batch, cost_field, derive_masks}` | S5 search |
| WS-D2 negotiation | `konnect-route::detail::negotiate`, `CpuCompute::{paint, unpaint, update_costs}` | S5 control |
| WS-E plan, apply, optimize | `konnect-route::{optimize, plan}` bodies, the job runner (later, in `konnect-core`) | S6, S8 |
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
- **`RouteCompute: SearchBackend`.** `paint`, `unpaint`, `derive_masks` and
  `update_costs`.
- **`CpuCompute`** is the reference. `paint`, `unpaint` and `update_costs`
  are implemented; the rest are stubs owned by WS-D1.
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
- **Keep generated runs in ignored build or cache locations.**
- **Gate before merging:** `cargo clippy -p <crate> --all-targets -- -D
  warnings`, `cargo test -p <crate>` and `cargo fmt --check` for every crate
  you touched.
- **Dependencies:** permissive licences only (MIT, Apache-2.0, BSD or ISC).
  Port ideas from GPL routers; never copy their code.
