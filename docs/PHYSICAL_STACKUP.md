# Physical stackup

`set_four_layer_physical_stackup` configures a symmetric four-layer saved board with F.Cu, In1.Cu, In2.Cu and B.Cu. It accepts outer/inner copper, prepreg/core and mask thicknesses plus relative permittivity. The sum must be within 0.05 mm of the board's existing nominal thickness.

The tool replaces the physical layer entries, including their layer-specific properties, while retaining non-layer stackup properties and all content outside the stackup. It does not qualify impedance or change fabrication rules. Material is FR4; loss tangent and mask color are not specified by this profile.

Dry-run is the default. Apply requires the exact returned revision, a closed board and the normal saved-file safety checks. The atomic writer rejects a changed source and the tool verifies exact readback. Save and cleanly close KiCad before beginning a deliberately fresh closed-board session; never bypass an unsafe-file-fallback result.

Validation: profile replacement/idempotence, preservation of unrelated board content and stackup finish, rejection of incompatible layer sets, malformed parameters and inconsistent thickness. Full core suite: 1,254 passed, 13 ignored.

## Six-layer composite stackup

`set_six_layer_physical_stackup` requires exactly F.Cu, In1.Cu, In2.Cu, In3.Cu, In4.Cu and B.Cu. It does not enable layers. The same closed-board and reviewed-revision requirements apply.

The symmetric stack is outer copper / prepreg / inner copper / core / inner copper / central prepreg-core-prepreg / inner copper / core / inner copper / prepreg / outer copper, plus the mask thickness on each side. `prepreg_mm` specifies each outer prepreg, `core_mm` each outer core, and the required `center_prepreg_mm` and `center_core_mm` specify the three central sublayers. Prepreg and core permittivities apply to their respective central materials too. KiCad's native `addsublayer` representation preserves all three thicknesses and permittivities, rather than substituting an effective dielectric. Each material is explicitly FR4; material procurement and impedance qualification remain external requirements.

The total must be within 0.05 mm of the board's nominal thickness. The tool replaces physical layer entries and retains non-layer stackup properties and unrelated board content. This profile does not support arbitrary asymmetric stacks or independent permittivity for each prepreg.

Six-layer validation: all three physical-stackup tests pass, including copper-layer order, composite central sublayers, preservation/idempotence, incompatible boards and invalid or inconsistent dimensions. The full core suite passes with 1,255 passed and 13 ignored. The MCP executable builds successfully.

A closed-board dry-run/apply and native KiCad 10 reopen/save round trip preserved the six copper layers and the central 0.1164 / 0.7 / 0.1164 mm sublayers, with 1.58688 mm total thickness and unchanged copper/connectivity DRC results. IPC-2581 reports the correct total but repeats the first central thickness for all three central StackupLayer entries; do not use that export alone to verify or procure the composite stack. Native round-trip inspection confirms the distinct thicknesses and permittivities. KiCad supplies default loss tangents and drops the mask permittivity from its saved stackup; these parameters are not qualified dielectric-loss or solder-mask simulation data.
