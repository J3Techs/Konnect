# Differential routing defaults

`create_netclass` accepts optional `diff_pair_width`, `diff_pair_gap` and `diff_pair_via_gap` values in millimetres. Each supplied value must be finite and positive. Omitted values remain unchanged in existing classes; new named classes inherit omitted settings from Default. Existing ordinary trace, clearance, via and schematic settings are preserved.

`get_netclasses` reports these three fields using the same resolved-value and inheritance reporting as ordinary routing settings. The update result reports stored values, which can be null where a named class inherits a setting.

These are KiCad router defaults, not impedance validation or custom DRC constraints. Supply geometry from the selected stackup calculation, assign the relevant nets, and reopen the project to load changed defaults. Existing tracks are not resized or respaced by a netclass update. A positive via gap is not proof of a qualified differential transition.

Validation: 28 netclass tests and the full core suite (1,253 passed, 13 ignored) pass. Coverage includes explicitly set differential values, readback, partial updates retaining omitted values, sparse-class inheritance, and invalid values rejected without modifying the project.
