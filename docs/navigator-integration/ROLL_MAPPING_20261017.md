# Atomic Navigator calibration / roll mapping integration

This paired revision integrates the locally replayed candidate. It preserves owner1.20/0.97, Navigator geometry, fixed pinion safety model and existing low-speed tracking. The Comma has not been changed by this integration.

## Selection and message compatibility

`FordPrefRollAwareMapping` is a new persistent bool, default0. It requires the existing `FordPrefPersistentTracking` option plus Expedition/pinion/low-speed eligibility. The default factory remains BudgetedTrackingPrototype; the additional opt-in selects RollAwareMapping. No UI toggle or automatic parameter migration is added.

Controlsd carries an additive optional CarControl.navigatorCalibration snapshot from the exact inputs used for that angle target. card consumes that same packet, checking source/state/computation/control timing, validity, ordering and target identity. Producer target speed avoids receiver-speed skew. Old messages have no valid snapshot and preserve baseline. Old compiled parameter libraries treat the unknown new key as off until rebuilt. Install/build the paired parent and child together; the new producer requires the additive child schema.

## Output contract

Initial mapping scope50–70mph, bounded0.01rad/15%of feedforward, onset0.001rad/20Hz evaluation. It is roll-compensation sensitivity with calibration consistency gating, not an independently identified ratio/actuator gain model. Same-model forward/inverse conversion cancels ratio/offset at zero roll.

The unchanged baseline has independent state. Invalid/reversed corrections release toward its closest reachable output within existing range/slew limits. Rearming is blocked until final output rejoins. If baseline moves away at the rate limit, immediate/fixed-time release is impossible without changing that limit; tests verify bounded residual and blocked rearming until baseline becomes reachable.

## Evidence

Repository tests: `openpilot/selfdrive/car/tests/test_calibration_context.py` and child `opendbc/car/ford/tests/test_roll_mapping.py`. Full local Mac build/test results, backup/provenance and updated roadmap are retained under the shared workspace's `bluepilot/docs/research/navigator-paired-roll-build-2026-10-17/`.

The preceding staged candidate passed515raw segments/27sessions:1,904,757native TX checks per variant with zero rejection;9,983pre-configuration frames separately unhooked. Strict legacy controls were byte-identical to baseline; changed-command replay used labeled synthetic causal snapshots because old recordings lack exact producer snapshots. Five rate-limited release-tail frames remained. AST/source-equivalence evidence binds this integration to that tested candidate despite formatting and explicit default zip semantics.

These are local Mac/native/IPC checks. They do not establish AGNOS/Chestnut runtime, real bus/USB timing, hardware readiness or improved road tracking. Installation, option activation and road tests remain separate; no device parameter was written here.

## Integrated local validation — 2026-10-17

- Both full target sets pass on this Apple Silicon Mac: `scons --minimal -j4` and `scons -j4`, rebuilding affected schema/native application targets.
- Project-scoped parent suite:989passed,86skipped,1expectedfailure,343passing subtests.
- Full opendbc suite:3880passed,1690skipped,24673passing subtests. Uses the pre-existing unsanitized Mac debug native safety library; no UBSan result claimed.
- Local Panda SCons build passes, including H7 firmware artifacts; no flashing.
- Integrated actual producer/card+unique local IPC screen and320transition/abort cases(115200frames)pass; default-off factory and stale-snapshot final baseline rejoin verified.
- Scoped lint/diff checks pass. Protected steering/geometry sources and native safety library SHA unchanged. Source-equivalence receipt connects the integrated control logic to the prior515-segment staged replay; that replay was not repeated merely for formatting/integration.
- Independent review of the applied code and portable tests found no remaining concrete integration issue. The original paired source/HEADs were preserved before patch application.

Whole AGNOS/Chestnut runtime, installation and road validation remain separate from this successful local Mac build/test run.

Integrated child pin: `8cbc834fe534a248a3bc6d56ce964106c0f49503`.
