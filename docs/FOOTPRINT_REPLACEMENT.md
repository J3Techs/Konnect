# Targeted footprint replacement

`update_footprints_from_library` accepts an optional `replacements` map from reference to `Library:Footprint`. When nonempty, `references` must exactly equal its keys, and `library_ids` cannot be supplied. This avoids unintentionally refreshing unrelated footprints.

For example, dry-run with `references: ["C14", "C16"]` and `replacements: {"C14": "Capacitor_SMD:C_0603_1608Metric", "C16": "Capacitor_SMD:C_0603_1608Metric"}`. Apply the same arguments with `dry_run: false` and the exact returned `expected_plan_revision`.

The existing live-only, unsupported-content, missing-connected-pad, duplicate-reference and revision checks still apply. The replacement keeps instance identity, position, rotation, side, properties and nets by pad number. Pad geometry can move: inspect routing and run DRC afterward. The target ID participates in the plan hash and is reported in the change list; an ID-only change counts as metadata.

`edit_component` also accepts `value_visible: false` to hide a footprint value label without erasing its value. The operation reads visibility back from KiCad before reporting success.

Validation: 33 focused footprint-update tests passed, including replacement-scope validation, target-ID hashing and preserved instance/net state. Built the server and exercised a two-capacitor replacement and value-label visibility over live KiCad IPC.
