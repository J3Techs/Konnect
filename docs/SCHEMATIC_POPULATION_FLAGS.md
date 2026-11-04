# Native schematic population flags

`set_schematic_population` in `sch_batch` sets KiCad's native `dnp` attribute for an explicit list of references in one saved schematic sheet. A custom field named DNP is not a substitute: native flags control BOM exclusion and the DNP state transferred by schematic-to-PCB synchronization.

Request a dry run with `schematic`, `references`, and a boolean `dnp`. Review `units`, `changed_units`, and `plan_revision`; apply with `dry_run: false` and that exact `expected_plan_revision`. Every placed unit of each requested reference is covered. Hierarchical sheets are addressed individually.

The tool rejects duplicate or missing references, malformed native attributes, and stale or missing apply revisions. It preserves wiring, custom properties, library symbols, `in_bom`, and `on_board`. Writes use the existing editor-lock and compare-and-swap atomic writer. Immediate readback verifies the resulting flags. A no-change request reports `noop`.

Validation includes multi-unit and unrelated-content preservation, ambiguous-input refusal, and revision-bound application tests. A real KiCad export verified that setting six native DNP flags changes the fitted BOM from 107 to 101 rows, excluding exactly those references. Live schematic-to-PCB sync transferred all six flags without adding footprints or changing pad nets. These checks establish population metadata behavior, not manufacturing readiness of that board.
