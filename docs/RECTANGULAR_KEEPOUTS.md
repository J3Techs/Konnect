# Rectangular routing exclusions

`add_rectangular_keepout` adds one named rule area to the requested live board. It requires a nonempty name, ordered finite bounds, and a unique list of enabled copper layers. It has no file fallback. Existing areas with the same name are refused to avoid accidental duplicate constraints.

Tracks, vias and copper fills are prohibited. Pads and footprints remain allowed unless their optional flags are enabled; this permits a printed antenna footprint to coexist with an exclusion for unrelated routing. This is not an exclusion for arbitrary new pads, so antenna reviews must still check component placement.

Creation is followed by inspection of KiCad's returned item. Layer order and the inactive placement-source default may be normalized by KiCad; effective prohibition flags, geometry and layer membership must match. A post-creation verification error means the board must be inspected before retrying. The tool neither saves nor refills the board.

Validation: finite geometry/layer rejection and protobuf content tests, full core unit suite (1254 passed, 13 ignored), and KiCad 10 live creation of four front-side antenna rule areas with verified returned geometry/settings. A three-layer rear/inner area was also saved and inspected; its initial strict comparison exposed KiCad's normalization and motivated semantic comparison. No fabricated-board or RF qualification is implied.
