# Scoped pad clearance rules

`set_scoped_design_rules` in the `verification` toolset plans and writes native KiCad edge or hole rules for exact footprint reference allowlists. It does not alter the board or global project minima, nor does it suppress violations. A scoped rule can deliberately override the global constraint for its selected pads; choose its minimum from an engineering and fabrication justification.

Supply the saved `.kicad_pcb`, rules with unique names, exact `references`, `constraint` (`edge_clearance` or `hole_clearance`), positive `minimum_mm`, and a single-line `rationale`. Edge rules match pads only. Hole rules match pad pairs within the same named footprint; they never match different footprints in the list. Other hole/track/via checks retain their existing constraints. The rationale is retained as a comment inside each owned rule.

Dry run is the default. Review `before`, `after`, and the returned revision, then apply the same arguments with `dry_run: false` and `expected_plan_revision`. Revision checking binds the saved board and rule preimage and preserves unrelated rules. Apply refuses missing references, arbitrary condition syntax, stale revisions, and nonpositive clearances. Always run native DRC afterward: syntax and actual geometric compliance are checked by KiCad, not inferred from the plan.

## Validation

Unit tests cover exact scopes, injection rejection, same-footprint hole pairing, preservation/idempotence, and revision binding. A KiCad 10 native DRC smoke check applied a 0.25 mm pad-edge rule to two fixed connector references: their 12 edge findings cleared, the other 19 errors remained, and unconnected items stayed zero. No exclusion or global limit change was used. A lower allowable minimum is a process-specific design decision, not general manufacturing qualification.
