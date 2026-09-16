# Promote imported schematic symbols

`promote_embedded_symbols` in `library` makes imported, embedded symbol definitions available as a project-local library. It preserves pin coordinates, graphics, instance positions, properties, and UUIDs. Optional per-pin electrical types correct import metadata without changing a physical pin map.

Supply a new `.kicad_sym` path in the schematic directory, a library nickname, and entries containing `reference`, a unique new symbol `name`, and optional `pin_types` keyed by pin number. Dry-run is the default. Apply requires `dry_run: false` and the exact `expected_revision` returned by the dry-run. Register the resulting library with `register_symbol_library` afterward.

The tool rejects duplicate references/names, missing references/pins/definitions, invalid electrical types, existing destination libraries, and mismatched library IDs across placed units. It retains old embedded definitions so unrelated instances are unchanged. Each selected reference gets its own definition, including all units and graphics.

The library is created before the schematic commit. A stale or editor-owned schematic is not overwritten; if that final commit fails, the new library may remain unused. Inspect the reported paths before retrying; never delete or overwrite a pre-existing library to bypass the refusal. Successful apply reads back both saved files and verifies their complete contents.

Validation on an imported project: 25 symbols promoted with unchanged electrical connectivity, followed by project registration, library readback, and KiCad ERC. Physical pin assignments still require independent source validation; changing ERC types does not establish a part's identity or suitability.
