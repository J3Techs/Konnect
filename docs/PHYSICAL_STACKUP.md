# Four-layer physical stackup

`set_four_layer_physical_stackup` configures a symmetric four-layer saved board with F.Cu, In1.Cu, In2.Cu and B.Cu. It accepts outer/inner copper, prepreg/core and mask thicknesses plus relative permittivity. The sum must be within 0.05 mm of the board's existing nominal thickness.

The tool replaces the physical layer entries, including their layer-specific properties, while retaining non-layer stackup properties and all content outside the stackup. It does not qualify impedance or change fabrication rules. Material is FR4; loss tangent and mask color are not specified by this profile.

Dry-run is the default. Apply requires the exact returned revision, a closed board and the normal saved-file safety checks. The atomic writer rejects a changed source and the tool verifies exact readback. Save and cleanly close KiCad before beginning a deliberately fresh closed-board session; never bypass an unsafe-file-fallback result.

Validation: profile replacement/idempotence, preservation of unrelated board content and stackup finish, rejection of incompatible layer sets, malformed parameters and inconsistent thickness. Full core suite: 1,254 passed, 13 ignored.
