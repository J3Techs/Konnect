# Junction-rule fixture

`junction_rule.kicad_sch` puts one case of KiCad's junction rule in each area of
an A4 sheet, with a text note naming the case and its verdict. It exercises
`list_schematic_junctions`, `prune_schematic_junctions`, and the junction
re-judging in wire and component deletes, `batch_delete` and moves.

## Provenance

The sheet was generated as text with KiCad 10.0.7's stock `Device:R` embedded
verbatim (copied from `junction_sheet_pin.kicad_sch`), then parsed and re-saved
by KiCad:

```text
kicad-cli sch upgrade --force junction_rule.kicad_sch
```

What is committed is that Eeschema serialization: `(version 20260306)`, items
regrouped by type, CRLF line endings. KiCad kept all 12 junctions, both dots of
the duplicate pair included, so loading does not clean junctions up. Every UUID
is sequential and readable: `d07e0000-0000-4000-8000-000001000NNN` for junction
NNN, `…000002000NNN` for wire NNN, and `…000003…`/`…000004…` for the three
resistors and their pins.

## The cases

| Junction | At | What meets there | Exits | Verdict |
|---|---|---|---|---|
| 1 | (30.48, 30.48) | wires 1 and 2 end: a corner | 2 | unneeded |
| 2 | (55.88, 30.48) | wires 3 and 4 end: a straight join | 2 | unneeded |
| 3 | (86.36, 30.48) | wires 5, 6 and 7 end | 3 | needed |
| 4 | (116.84, 30.48) | wire 9 ends on wire 8's span | 3 | needed |
| 5 | (30.48, 60.96) | wires 10 and 11 cross | 4 | needed |
| 6 | (60.96, 60.96) | R1 pin 1 on wire 12's span | 3 | needed |
| 7 | (86.36, 60.96) | wire 13 ends on R2 pin 1 | 2 | unneeded |
| 8 | (111.76, 60.96) | wires 14 and 15 make a corner on R3 pin 1 | 3 | needed |
| 9 | (30.48, 91.44) | wires 16, 17 and 18 end | 3 | needed |
| 10 | (30.48, 91.44) | a second dot on junction 9's point | 3 | duplicate |
| 11 | (60.96, 91.44) | wires 19 and 20 make a corner under label `CORNER_NET` | 2 | unneeded |
| 12 | (86.36, 91.44) | wire 21 runs through | 2 | unneeded |

## KiCad behaviour checked while building it

- `kicad-cli sch export netlist` gives the same `(nets …)` section with and
  without junctions 1, 2, 7, 10, 11 and 12, so removing the dots the rule calls
  unneeded or duplicate changes no connection KiCad makes on this sheet.
- That check is weak on this sheet, because every net on it is a single
  resistor pin. The sensitive form of it, pruning a whole multi-sheet design
  and comparing KiCad's netlists, with removing a needed dot as a control, was
  run on a real project outside the test suite.
