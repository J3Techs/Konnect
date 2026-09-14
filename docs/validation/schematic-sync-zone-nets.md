# Schematic sync routing evidence

The previous snapshot treated every board net as routed whenever any zone existed. Copper zones already expose their assigned net through KiCad IPC; treating them as global routing evidence blocked isolated pad changes and assignment of newly placed footprints.

The snapshot now decodes the requested track, arc, via and zone message types and records each observed net. Rule areas do not create routing evidence. Malformed messages, incompatible zone settings, missing net assignments and unnamed nonzero nets stop the snapshot. The planner continues to reject changing a pad away from a net with routed copper; it allows an unrouted pad to join a routed destination net.

## Validation

- Workspace library/integration tests, documentation tests, clippy with warnings denied, formatting, and executable build passed with the locked dependency graph on Rust 1.96.0.
- Focused tests cover copper-zone net isolation, protected source nets, new pad assignment to routed destinations, tracks/arcs/vias, rule areas, malformed and missing evidence.
- Negative controls removed the source-net guard and the zone net assignment separately; both caused the focused regression test to fail. Restoring the implementation returned the focused suite to passing.
- Live KiCad 10.0.6 exercise: a saved 114-footprint, 8-layer board with six copper zones and 1,533 tracks/vias. After deliberately removing the affected source net routing, dry-run proposed exactly five footprint updates and 13 pad assignments. Revision-checked application reported all five updates and 13 assignments applied. Save and independent native pad-net readback confirmed the intended result; geometry and unrelated routing were preserved by the sync.
- This change does not provide geometric route planning. New or changed pads still require routing, zone refill and native DRC before board release.
