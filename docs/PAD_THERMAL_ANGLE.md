# Placed-pad thermal-spoke angle

`pcb_components.set_pad_thermal_angle` changes only the thermal-spoke angle of
one uniquely numbered pad on a live board. It preserves placement, pad/drill
geometry, nets, connection style, spoke width and gap, and every other pad.
The library footprint is not changed.

Start with `board`, `reference`, `pad_number`, `angle` and `dry_run: true`.
Review `before_angle`, then apply the same request with `dry_run: false` and
`expected_angle` equal to that observed value. A stale angle is rejected.
Angles must be finite and in [0, 360). The tool refuses missing or duplicate
pad numbers, missing explicit thermal settings, and per-layer zone overrides.
The normal live-board target guard applies; there is no file fallback.

Apply verifies the angle through a fresh IPC readback. Refill zones, save the
board, and run DRC to verify the physical result. An angle readback alone does
not prove a sufficient spoke count or connection to non-isolated copper.

Validation: 71 IPC unit tests pass, including three focused tests for field
preservation, duplicate/missing pad rejection, invalid angles, inherited
settings and per-layer overrides. The executable builds successfully. A live
KiCad 10 V1 board trial changed H1B pad 3 from 0 to 45 degrees; both inner-layer
thermal errors cleared after refill/save. The top-layer isolated island needed
a local manual spoke in addition to a ground-stitching via; design rules stayed unchanged.
