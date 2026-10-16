# Multi-layer Specctra fixture

`specctra_multilayer.kicad_pcb` is a hand-written KiCad 10 board that exercises
every multi-layer export path in one small file:

- a four-layer stack (F.Cu, In1.Cu as a `power` layer, In2.Cu, B.Cu);
- a rotated front footprint (`U1`, 90 degrees) with a rounded-rectangle pad, a
  custom pad, and a paste-only aperture without copper;
- a back-side footprint (`C1`, -90 degrees) with B.Cu-only SMD pads;
- a through-hole footprint (`J1`) with an oval slot, a repeated pad number, and
  an NPTH hole;
- a fiducial (`FID1`) with an unnumbered pad and a 0.6 mm local clearance;
- a GND zone on In1.Cu whose outline has a rectangular hole;
- a locked In2.Cu track and a locked 0.45/0.2 mm through via that does not match
  the net's rule via.

Unlike the two-resistor fixtures it was not saved by KiCad, so it is not a
parity oracle for KiCad's native exporter. The ignored
`freerouting_routes_the_multilayer_fixture` test routes it with Freerouting
2.3.0 and passes the session through the strict SES planner.
