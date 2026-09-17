# Footprint text orientation during library refresh

Library refresh now reads `(unlocked yes/no)` on footprint properties and user text as KiCad's keep-upright setting. It is independent of the item's edit-lock state: omitted or `no` means keep upright, and `yes` disables keep upright. Mandatory property placement remains instance-owned; library custom fields and user text preserve the supported orientation setting through IPC.

Source: KiCad's [text parser](https://docs.kicad.org/doxygen/pcb__io__kicad__sexpr__parser_8cpp_source.html) and `TextAttributes.keep_upright` in the bundled IPC schema. Unknown, malformed and repeated clauses still refuse before any update. No pad fabrication metadata support or file-fallback policy changes are included.

Validation: all 33 `pcb_footprint_update` unit tests passed, including new explicit/default orientation cases across front/back placement and four rotations, lock independence, and invalid/duplicate clauses. Build succeeded. Native board validation is recorded separately by the caller; passing parser tests does not establish physical footprint acceptance.

Native parity inspection additionally exposed two builder defaults. Footprint circles now preserve their transformed defining circumference point instead of substituting an equivalent point on the positive X axis. SMD pads carry an explicit circular, zero-size drill; this creates no hole but prevents KiCad from defaulting the unused drill shape to oblong. Comparison normalization no longer erases an explicit oblong shape on a zero-size drill.

Final validation: 68 IPC tests and 34 footprint-refresh tests passed. Native KiCad comparison and DRC confirmed that the corrected refresh clears test-point circle/drill mismatch reports; remaining unrelated custom land-pattern mismatches are not suppressed. Exported copper geometry and connectivity were checked separately.

## Nanometer conversion

IPC coordinates and pad dimensions now round to the nearest nanometer rather than truncate. For example, binary floating-point 2.05 mm previously became 2049999 nm instead of 2050000 nm. KiCad compares pad dimensions exactly, so that loss created native library mismatch warnings. Regression coverage includes signed values and translated polygon coordinates. Native refresh cleared U604 and both H1/H2 warnings, retaining all component placements and copper geometry to 0.00001 mm export precision. U602 retains a separate polygon warning: its source library has an explicitly repeated closing vertex that native IPC removes.
