# `stackup_six_layer_synthetic.kicad_pcb`

A hand-written board for the closed-board stackup tests in
`src/tools/board_stackup.rs` (`closed_setter_tests`). It was not made or saved by KiCad and is
not a design: it exists to give the setter something to preserve.

- A six-copper layer table in KiCad's ID order (`F.Cu` 0, `B.Cu` 2, `In1.Cu` 4 …), which is
  not the physical order the stackup must follow.
- A complete KiCad 10 `(setup (stackup …))` in KiCad's own layout, including a two-ply
  dielectric with a thickness lock on its first ply and a color on its second, a mask with
  material, εr, loss tangent and color, and a finish.
- Opaque data an edit must leave byte for byte: a footprint with a `pad_prop_bga` custom pad,
  primitives, nets, UUIDs, a 3D model, an attribute KiCad does not define, a segment and a zone,
  other `(setup …)` clauses and a board property whose text contains `(setup)`.
- A nominal `(general (thickness 1.6))` that differs from the stack's own 1.26 mm sum.
