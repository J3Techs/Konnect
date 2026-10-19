# Native routing: the live flow

This is the recommended order of tool calls for routing a candidate board
that KiCad 10 holds open, from a saved board to a DRC report. It applies to
any project; nothing here is specific to one board.

## Recommended sequence

1. **Save the candidate in KiCad** (or start from a saved file KiCad has
   open). The job reads the saved file only.
2. **`start_native_route_job`** with:
   - `output_plan`: a new routing_file;
   - `fanout_plan`: a second new routing_file. The job's own plane fanout
     (S2) goes there and is left out of `output_plan`;
   - `net_classes`: `auto` (the default). When KiCad holds the board open,
     each net's effective class is read over IPC and the rule table is
     pinned to it; otherwise the project's patterns are resolved offline,
     as KiCad resolves them;
   - the owner's allowances, by name: `class_layers`, `layer_fallback`,
     `neck_down`, `exclude_classes`, `fanout_share_within_mm` (see below).
3. **`get_native_route_job`** until the state is `COMPLETED`. Its report
   gives `excluded_by_policy`, the resolved `allowances`, `net_classes`
   and the `reference` check.
4. **`apply_route_plan`** with `fanout_plan` and `locked: true`, then
   **`apply_route_plan`** with `output_plan` and `locked: false`. Neither
   refills, saves or runs DRC.
5. **One `run_drc`** with `sync_live_board: true` and `refill_zones: true`
   and an `output` file. It refills once over IPC, saves, and runs the CLI
   DRC on the saved file without refilling a second time.
6. **`map_route_plan_drc`** on each applied plan and its ledger.

## What this removes

The earlier flow planned the fanout with `plan_plane_vias`, applied it,
and ran a refilling, saving DRC before starting the job, so that the job
would read a board that already held the fanout. The job runs the same
fanout itself (S2, before routing), so with `fanout_plan` that whole phase
goes: one apply pair and one refill, save and DRC at the end. Do not call
`refill_zones` or `save_project` before a `run_drc` that syncs and
refills; it repeats the slowest step.

Keep the separate fanout phase only when the fanout itself is the thing
under review (its own DRC evidence before routing).

## Allowances

Everything is off unless named, except the class policy:

| Argument | Meaning |
|---|---|
| `class_layers` | `{ "Class": ["F.Cu"] }`: the layers a class routes on. Unlisted classes use every routable layer. |
| `layer_fallback` | `[{ "class", "layers", "nets"?, "reference"?: { "plane", "over"?, "margin_mm" } }]`: connections of the class negotiation failed are routed again on `layers`. With `reference`, tracks on `over` (default: the fallback layers beyond the class's own) must lie over one filled piece of `plane`, at least `margin_mm` inside it; the check runs again after the optimizer, which may not move such a track off the reference. `nets` limits the fallback to some nets of the class. |
| `neck_down` | `{ "width_mm": 0.1, "reach_mm": 1.0, "classes"?, "exclude_classes"? }`: a track may leave a pad narrower than its class width within the reach, never below the rules' minimum. |
| `exclude_classes` | Classes left unrouted (zones or hand-drawn copper), reported as `excluded_by_policy`, never as failures. Default: every class whose track width is at least 0.5 mm. `[]` routes every class. |
| `fanout_share_within_mm` | A plane pad that gets no via of its own may share a same-net via placed for a pad within this distance. Default: never. |

The resolved allowances are reported back by name, so the report shows
exactly what a run was allowed to do.
