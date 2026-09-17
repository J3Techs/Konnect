# Footprint text orientation during library refresh

Library refresh now reads `(unlocked yes/no)` on footprint properties and user text as KiCad's keep-upright setting. It is independent of the item's edit-lock state: omitted or `no` means keep upright, and `yes` disables keep upright. Mandatory property placement remains instance-owned; library custom fields and user text preserve the supported orientation setting through IPC.

Source: KiCad's [text parser](https://docs.kicad.org/doxygen/pcb__io__kicad__sexpr__parser_8cpp_source.html) and `TextAttributes.keep_upright` in the bundled IPC schema. Unknown, malformed and repeated clauses still refuse before any update. No pad fabrication metadata support or file-fallback policy changes are included.

Validation: all 33 `pcb_footprint_update` unit tests passed, including new explicit/default orientation cases across front/back placement and four rotations, lock independence, and invalid/duplicate clauses. Build succeeded. Native board validation is recorded separately by the caller; passing parser tests does not establish physical footprint acceptance.
