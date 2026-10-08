# Roadmap

Where Konnect is going, in the order the work actually depends on itself.
There are no promised dates: an item ships when its contract, implementation,
and evidence agree. Opening or updating an issue is the best way to influence
priority.

This roadmap reflects `upstream/main` at
[v0.13.0](https://github.com/mixelpixx/Konnect/releases/tag/v0.13.0).
Release notes describe what shipped; this file describes the remaining user
outcomes and the architectural boundaries that constrain them. Priorities are
informed by the dated
[Improvement Backlog](https://github.com/mixelpixx/Konnect/discussions/165),
live issue labels, tracker issues, and benchmark evidence.

## 1. Truth, source authority, and write safety

Every response field must derive from an observed result, never from the
request. A verdict without evidence is `INCOMPLETE`; an unavailable safety
check is `BLOCKED`. Live IPC, saved files, CLI output, and compatibility
fallbacks are different evidence sources and must not be presented as
interchangeable truth.

- Complete the shared board-state contract in
  [#574](https://github.com/mixelpixx/Konnect/issues/574), including explicit
  source selection, provenance, stale-state refusal, and sibling-tool coverage.
- Resolve the remaining live-observation lifecycle problem in
  [#671](https://github.com/mixelpixx/Konnect/issues/671) and its implementation
  PR before broadening live mutation behavior.
- Keep uncertain mutations bounded: inspect before retrying, resume completed
  chunks rather than replaying them, and preserve no-write-on-refusal tests.
- Treat unavailable, malformed, and unreadable configuration as distinct from
  valid defaults. Never turn a load failure into a later destructive save.

v0.13.0 established the current foundation: exact-document reuse, explicit
mutation uncertainty, bounded IPC recovery, recoverable schematic-to-PCB
synchronization, live-board readback, and source-aware board operations.

## 2. KiCad-native architecture and compatibility

[Tracker #590](https://github.com/mixelpixx/Konnect/issues/590) is the
architectural horizon. Prefer supported KiCad IPC or CLI operations, keep
guarded file compatibility only where the supported KiCad release has no
usable native operation, and retire fallbacks once native equivalence is
proved. Konnect should expose KiCad capabilities safely; it should not quietly
become a second PCB editor, manufacturing suite, or UI-automation framework.

- Track vendored protocol provenance and stable/nightly drift in
  [#616](https://github.com/mixelpixx/Konnect/issues/616).
- Keep the KiCad 11 transition and SWIG removal visible through
  [#257](https://github.com/mixelpixx/Konnect/issues/257).
- Coordinate missing native coverage through
  [#436](https://github.com/mixelpixx/Konnect/issues/436), with an explicit
  product decision before adding replacement behavior.
- Preserve the architecture review filter in issue triage: native capability
  first, transparent AI-directed workflow second, Konnect-owned replacement
  only by deliberate exception.

## 3. Multilayer boards, stackup, planes, and routing

[Tracker #720](https://github.com/mixelpixx/Konnect/issues/720) coordinates the
whole board-layer workflow so individual fixes do not leave a half-supported
design path.

1. Establish guarded live copper-layer expansion through
   [#721](https://github.com/mixelpixx/Konnect/issues/721).
2. Preserve and validate stackup, ground-plane, layer-role, and source evidence
   before enabling broad stackup mutation.
3. Correct the native Specctra-export selection boundary in
   [#670](https://github.com/mixelpixx/Konnect/issues/670).
4. Expand the Freerouting round trip beyond its current two-layer boundary in
   [#449](https://github.com/mixelpixx/Konnect/issues/449).
5. Add rule areas through [#715](https://github.com/mixelpixx/Konnect/issues/715)
   only with the round-trip preservation contract tracked by
   [#722](https://github.com/mixelpixx/Konnect/issues/722).

The existing five-step Specctra/Freerouting workflow remains the supported
route path. New board constructs must survive export, routing, import,
readback, and DRC rather than merely serialize once.

## 4. Coordinates, geometry, and schematic correctness

[Tracker #575](https://github.com/mixelpixx/Konnect/issues/575) owns coordinate
units, frames, rounding, geometry, and result verification across schematic and
PCB tools. The immediate family includes noisy response coordinates and
unrounded bulk moves (#744/#747), overlap classification (#745), and missing
label identity needed for bounded deletion (#746).

- Finish mirror behavior and field placement together through
  [#450](https://github.com/mixelpixx/Konnect/issues/450),
  [#613](https://github.com/mixelpixx/Konnect/issues/613), and PR #738.
- Keep hierarchy and instance identity coordinated through
  [#578](https://github.com/mixelpixx/Konnect/issues/578).
- Prefer KiCad-authored fixtures, served-dispatch tests, exact readback, and a
  negative control for every load-bearing guard.

v0.13.0 completed the bus-connectivity, field-batching, hierarchical-sheet
movement, sheet-pin geometry, reference-prefix, and ERC-coordinate work that
previous versions listed as open foundations.

## 5. Libraries, parts, and manufacturing handoff

- Use [#296](https://github.com/mixelpixx/Konnect/issues/296) to coordinate
  advanced symbol and footprint controls rather than adding unrelated options
  one at a time. Oval-drill work in PR #621 remains part of that contract.
- Resolve multi-word JLCPCB search behavior through
  [#432](https://github.com/mixelpixx/Konnect/issues/432) and PR #726 before
  broadening catalogue enrichment.
- Add title-block support through
  [#714](https://github.com/mixelpixx/Konnect/issues/714) using the narrow
  KiCad-backed contract already recorded there.
- Keep variants, jobsets, and other advanced manufacturing orchestration
  outside Konnect unless KiCad exposes a stable IPC/CLI contract that Konnect
  can observe and verify.

## 6. Client discovery, startup, and guidance

- Make KiCad CLI and IPC prerequisite failures actionable once per startup
  through [#589](https://github.com/mixelpixx/Konnect/issues/589), not repeated
  surprises inside unrelated tool calls.
- Coordinate tool-catalog lifecycle behavior across clients through
  [#576](https://github.com/mixelpixx/Konnect/issues/576) and capability search
  through [#598](https://github.com/mixelpixx/Konnect/issues/598).
- Preserve independent Claude and Codex guidance review paths. Installed
  guidance drift is notification, not permission to overwrite user changes.
- Keep AI guidance responsible for workflow ordering, bounded mutation batches,
  readback, and evidence disclosure; do not bury those policies in opaque
  one-shot tools.

## 7. Platform, packaging, and lifecycle

- Complete the v0.13.0 post-release accounting in
  [#760](https://github.com/mixelpixx/Konnect/issues/760): stamp measured PCM
  metadata, synchronize the companion, update project status, and announce the
  verified release.
- Resolve or remove the undocumented `cdylib` lifecycle surface under
  [#731](https://github.com/mixelpixx/Konnect/issues/731); MCP remains the
  supported integration path.
- Address macOS signing and stable distribution before adding a Homebrew tap
  ([#131](https://github.com/mixelpixx/Konnect/issues/131), then
  [#154](https://github.com/mixelpixx/Konnect/issues/154)).
- Keep release-profile binaries, all three PCM packages, real-KiCad E2E, live
  IPC validation, and the routed-board benchmark in every release gate.

## 8. The quality flywheel

- A changed tool documents accepted input, invalid input, source/prerequisite
  state, observed changes, external-failure behavior, and safe recovery.
- Exercise deployed dispatch, not only private handlers. Mutations require
  no-write-on-refusal evidence and actual-result readback.
- Keep generated tool counts authoritative through
  `cargo xtask fix-doc-counts`; never edit quoted counts by hand.
- Treat tracker issues as completion contracts. Partial PRs use `Part of #N`;
  exactly one terminal PR closes the fully reconciled acceptance set.
- Keep contribution branches on current `main`, expose one dependent step at a
  time, and never treat stale green CI as merge evidence.

## Completed eras

- ~~v0.1-v0.3~~ — broad schematic, PCB, library, export, transport, and
  cross-platform packaging foundations.
- ~~v0.4-v0.7~~ — atomic PCB transfer, footprint graphics, client-scoped
  installs, and the first truth-and-safety enforcement arc.
- ~~v0.8-v0.11~~ — installation provenance, project ownership, hierarchy,
  executable guidance evidence, native Specctra/Freerouting, and safer fallback
  behavior.
- ~~v0.12-v0.13~~ — shared reliability contracts, live-source disclosure,
  bounded recovery, safer placement, richer schematic hierarchy and fields,
  stackup/3D-model/pad controls, and pre-tag release artifact validation.
