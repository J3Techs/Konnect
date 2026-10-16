# Custom-shape pad footprints

`custom_pads_texas_rje0020a_kicad10.kicad_mod` and
`custom_pads_vqfn20_kicad10.kicad_mod` are two stock KiCad footprints whose
corner pads are `smd custom`. Konnect's typed placement path once could not
carry a custom-shape pad, so `update_pcb_from_schematic` refused a plan that
needed one. These are the two footprints that refusal met on a real board,
where it named neither of them (#657). They are now the success fixtures for
custom-pad placement; the refusal tests derive unsupported variants from them
in memory (`convexhull` clearance, an arc polygon edge) without editing these
files.

## Provenance

Copied byte for byte from the KiCad 10.0.5 standard library on Windows 11.
Nothing was edited, and the repository stores the directory `-text`, so the
bytes are KiCad's.

| Fixture | Library file | SHA-256 (first 16) |
|---|---|---|
| `custom_pads_texas_rje0020a_kicad10.kicad_mod` | `Package_DFN_QFN.pretty/Texas_RJE0020A_VQFN-20-1EP_3x3mm_P0.45mm_EP0.675x0.76mm.kicad_mod` | `e4aad731d0feb660` |
| `custom_pads_vqfn20_kicad10.kicad_mod` | `Package_DFN_QFN.pretty/VQFN-20-1EP_3x3mm_P0.45mm_EP1.55x1.55mm.kicad_mod` | `2be5b256050d408f` |

Both carry `(generator "kicad-footprint-generator")`. The file name inside a
`.pretty` directory has to equal the footprint name, so a test that resolves
them through a library table writes them out under their library names.

## What they contain

Both are 20-pin VQFNs with a rectangular exposed pad numbered 21. In each, the
eight corner pads (1, 5, 6, 10, 11, 15, 16 and 20) are `smd custom` with a
`primitives` polygon, and the other twelve signal pads are `smd roundrect`.
Unnumbered `F.Paste`-only pads sit under the exposed pad: one in the Texas
footprint, four in the generic one.

The first is the footprint the schematic symbol names; the second is what a
designer substituted when the first was refused. Both were refused for the
same reason, which is why a diagnostic that named nothing sent the designer
round twice.

The exposed pad 21 carries `(property pad_prop_heatsink)` and `(zone_connect 2)`.
KiCad 10's API has a field for the zone connection
(`PadStack.zone_settings`) but none for a pad's fabrication property, so the
byte-exact files are refused for the heatsink property alone. The success
tests strip that one line in memory (`test_support::without_heatsink_property`)
and keep everything else, `zone_connect` included, exactly as KiCad wrote it.
