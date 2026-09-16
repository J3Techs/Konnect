# Verified saved-footprint library snapshots

`snapshot_saved_footprint_library` uses the native KiCad Python library writer and verifies the exported footprint with `FootprintNeedsUpdate` before publishing it. It reads only the authoritative saved board and never writes it. The caller must save current work first. Existing targets must resolve inside the project. Shared-entry replacement refuses divergent placements; an explicit `entry_name` creates a separate per-instance library entry instead.

Dry runs bind the saved board, target content or absence, reference, entry name and Python executable. Native output may contain freshly assigned UUIDs, so the revision binds the source and options rather than nondeterministic generated UUIDs. Every apply reruns native equivalence verification. Existing entries use revision-checked atomic replacement; new entries use no-clobber publication.

`link_board_footprint_library` changes only an existing live footprint's library identifier. It preserves all geometry, fields and nets, requires an exact dry-run revision, publishes one undo commit, and compares the full readback. If readback differs, it restores and verifies the original. It never refreshes footprint geometry. The caller must separately update the schematic footprint field after matching the verified snapshot.

Requires a native KiCad Python executable with `pcbnew`. Validation includes entry-name traversal rejection, dry-run/apply revision checks, native roundtrip checks and saved-board DRC after relinking imported V1 footprints. This does not validate manufacturer land patterns or qualify a footprint for assembly.
