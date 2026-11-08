# Design-wide net fixture

`design_nets.kicad_sch` (root) and `design_nets_child.kicad_sch` (the "Rail A
regulator" sheet) are a two-level hierarchy built to exercise the design-wide
net tools: `get_net_members`, `plan_sheet_removal` and `find_undriven_nets`.
`edit_schematic_label` uses the same pair for its label edits.

## Provenance

The pair was generated from KiCad 10.0.7's stock libraries (`Device:R`,
`Device:C`, `power:+3V3`, `power:GND`, `power:PWR_FLAG`, `74xGxx:74LVC1G04`,
`74xGxx:74LVC1G07`, `Regulator_Linear:AP2204K-1.5`,
`Connector_Generic:Conn_01x02`), with each library symbol embedded verbatim, and
then parsed and re-saved by KiCad:

```text
kicad-cli sch upgrade --force design_nets.kicad_sch
kicad-cli sch upgrade --force design_nets_child.kicad_sch
```

What is committed is that Eeschema serialization: `(version 20260306)`,
KiCad's own field placement, and the `Intersheetrefs` property KiCad adds to
every global label. KiCad 10.0.7 on Windows wrote CRLF line endings, and they
are kept.

The two netlists are KiCad's own answers, not this crate's:

```text
kicad-cli sch export netlist --format kicadsexpr --output design_nets.net design_nets.kicad_sch
```

`design_nets_child_removed.net` is the same export of a copy of the root with
the "Rail A regulator" `(sheet …)` block (UUID
`2a3b4c5d-6e7f-4a8b-9c0d-1e2f3a4b5c6d`) deleted — the copy
`plan_sheet_removal` makes. The only edit to either export is the absolute
path in the header's `(source …)`, replaced by the file name.

## What is on it

| Net | KiCad's members | Shape under test |
|---|---|---|
| `RAIL_A` | U10.5 VOUT `power_out` (child); C10.1, R10.1 (child); R2.1, U3.2 `input` (root) | a global rail sourced on the child: global labels on both sheets |
| `PGOOD` | U11.4 `open_collector` (child); R1.2 pull-up to `+3V3`, U1.2 `input` (root) | an open drain whose pull-up survives the child's removal |
| `/REG_EN` | U1.4 `output` (root); U10.3 EN `input` (child) | a root local label carried into the child by sheet pin `EN` and hierarchical label `EN` |
| `+3V3`, `GND` | power symbols on both sheets; `PWR_FLAG` on the root | rails driven by a flag |
| `FAULT_N` | U4.4 `open_collector`, R2.2 | an open drain pulled up to `RAIL_A` through R2 |
| `/FAULT_IN` | J1.1 connector, U4.2 `input` | a net fed from off the board |
| `STRANDED` | R4.1, C4.1 | passive-only: R4's other pin carries a no-connect flag, C4 reaches only `GND` |
| `/Rail A regulator/CHILD_LOCAL` | R10.2, U11.2 `input` | a child-local net fed through R10 from `RAIL_A` |
| `/RAIL_A_OK` | U3.4 `output` | one pin and a label |
| `ORPHAN` | — absent | a global label that reaches nothing: KiCad assigns it net code 8 and does not export it |
| `unconnected-(…)` | one pin each | the NC pins of U1, U3, U4, U10 and U11, and R4.2 under its no-connect flag |

With the child removed, KiCad's export keeps `RAIL_A` as R2.1 and U3.2 only (a
dead rail and an undriven input), `PGOOD` as R1.2 and U1.2 (pulled up, so
still defined), and `/REG_EN` as U1.4 alone.

## ERC

`kicad-cli sch erc` reports two violations, both deliberate: `label_dangling`
on `ORPHAN` and `isolated_pin_label` on `RAIL_A_OK`. It reports nothing for
`STRANDED`, which has no source at all — the gap `find_undriven_nets` exists
for.

## KiCad behaviour checked while building it

Throwaway variants of the child, netlisted with KiCad 10.0.7, settled how
labels of one name join (`design_nets.rs` models exactly this):

- a local label and a hierarchical label of one name on one sheet are one net;
- a local label and a global label of one name on one sheet are one net, with
  ERC warning `same_local_global_label`;
- a local label on the child and a global label of the same name on the root
  are two nets (`/Rail A regulator/FAULT_N` and `FAULT_N`);
- local labels of one name on two sheets are two nets.
