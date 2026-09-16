# Reviewed pad-net changes during schematic synchronization

A circuit revision can change a pad's net while unrelated copper still uses the
old net. `update_pcb_from_schematic` refuses this by default.

For an intentional topology change, supply `reviewed_pad_net_changes` in both
the dry run and apply. Every entry names the exact `reference`, `pad`, `old_net`
and `new_net`. Empty `new_net` means disconnect. The entries must match actual
changes in the saved schematic and live board. Duplicate, stale, mistyped,
unmatched and wildcard entries do not grant permission and cause a conflict.
There is no blanket override.

The result repeats the reviewed list and explicitly reports that routing is
unchanged. The review revision includes the complete live footprint and copper
payloads, so moving copper without changing the number of objects also
invalidates the dry run. Apply still requires the exact revision and uses the
existing single KiCad undo commit. This operation remains live-IPC-only.

This is a pad-assignment operation, not a routing repair. Remove obsolete
connections before applying, then reroute or retag the specifically intended
copper, refill zones and run DRC. Existing copper retains its old net until
separately changed. Do not consider a successful sync to establish electrical
connectivity or fabrication readiness.
