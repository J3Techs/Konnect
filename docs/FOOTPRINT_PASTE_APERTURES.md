# Explicit footprint paste apertures

Imported footprints can disable automatic pad paste with a large negative
margin and carry their intended stencil openings as separate zones. If those
zones do not appear in the paste plot, the resulting stencil omits the pads.

Two scoped tools replace this representation with filled F.Paste polygons:

- `library.set_library_footprint_paste_apertures`: supplies `footprint_path`.
- `pcb_components.set_board_footprint_paste_apertures`: supplies `board` and
  `reference`, with that board open in KiCad.

Both take `apertures: [{pad_number, points: [{x, y}, ...]}]` in **footprint-local
millimetres**. Every pad must be covered. These tools support front SMD
footprints only and refuse groups, duplicate pad numbers, non-SMD pads and
incomplete coverage. They replace front paste artwork/zones and disable
automatic front paste on those pads; they do not select component parts or
design stencil apertures for the caller.

Start with `dry_run: true` (the default), review the input geometry, and apply
with `dry_run: false` plus the exact `expected_plan_revision`. The revision
covers the current footprint and supplied aperture geometry. The library path
uses revision-aware atomic writing and readback. The board path uses the live
target guard and one undo commit. Library and board are independent operations;
apply and verify both when maintaining a project-local library.

KiCad 10.0.6 defers replacement of a footprint until its undo commit is pushed,
so live verification runs **after** that commit. It compares protected metadata,
pad nets, positions, geometry and non-paste children, plus the actual paste
polygons. Only equivalent graphic defaults (empty net and solid/default stroke)
and generated graphic IDs are normalized. A failed verification restores the
original footprint through another undo commit and checks restoration.

The Pad protobuf includes KiCad 10.0.6's read-only `parent` field (tag 13), so
round-trip verification preserves and checks that ownership metadata instead
of discarding it. See the [upstream schema](https://github.com/KiCad/kicad-source-mirror/blob/10.0.6/api/proto/board/board_types.proto).

Validation: executable build passes; 143 footprint-related unit tests pass,
including seven aperture-specific tests, with one existing integration test
ignored. Live verification restored 20 source-derived apertures across ten
V1 footprints and updated their four project libraries. Saved-board DRC lost
all 20 paste-padstack warnings without increasing errors or unconnected items.
The front copper SVG was byte-identical after removing its timestamp/title.

Always inspect an exported paste plot after saving and refilling. Successful
geometry restoration does not qualify the original stencil design, selected
parts, assembly process or the rest of the PCB.
