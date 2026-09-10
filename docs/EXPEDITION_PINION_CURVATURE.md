# Expedition pinion-curvature candidate

This draft integrates the paired opendbc controller/safety change on `navigator-angle-v1`. The `opendbc_repo` gitlink identifies the exact candidate. Panda source stays at `75aa44bec9140849868239b1f1e3f22624adb8fe`; its firmware must be compiled against that opendbc revision.

The Expedition CAN-FD fingerprint uses steering-pinion angle through a fixed vehicle model for controller and Panda curvature measurement. The selection is named `PINION_CURVATURE` in both car and safety flags and logged at controller initialization. There is no UI addition. Raw yaw remains recorded. The curvature-error allowance stays **0.002**, and HIGH/LOW factors remain unchanged.

## Evidence and review

The opendbc submodule contains `docs/ford-pinion-curvature/`: implementation details, fixed-model assumptions, recorded-signal comparisons, raw-log hashes, reproducible analysis/replay, and exact segment-local viewing windows. Review both linked PRs as a coordinated change.

The controller replay reproduces every baseline LMC2/LKA payload and its schedule. The native timing sweep improves rejection totals at four tested orderings, but an artificial -10 ms TX offset introduces two new rejections while removing two others. This is an unresolved draft candidate, not a confirmed steering-rejection fix. Actual MCU order is absent from the recordings.

Primary route: `0e09a1daaf2c4fd2|00000007--ebffff8226`.

| Case | Route seconds | Segment and local event time | Segments needed for ±10 seconds |
|---|---:|---|---|
| Non-brake 1 | 163.803932 | 2 at 42.791165 s | 2 |
| Non-brake 2 | 164.106263 | 2 at 43.093496 s | 2 |
| Brake | 464.675046 | 7 at 43.655655 s | 7 |
| Non-brake 3 | 657.658954 | 10 at 56.623135 s | 10 and 11 |
| Non-brake 4 | 658.360868 | 10 at 57.325050 s | 10 and 11 |
| Non-brake 5 | 679.674446 | 11 at 18.648354 s | 11 |
| Brake and driver steering | 700.635633 | 11 at 39.609541 s | 11 |
| Non-brake 6 | 1147.711498 | 19 at 6.673585 s | 18 and 19 |

Segment-local zero is the first raw CAN event in each file, not an assumed 60-second boundary. The full report also lists accepted comparison turns and the exact start/end of every cross-boundary window. A rejected command is not proof of a lane departure, sensor defect, or incorrect Panda behavior.

The two new candidate-only rejections in the -10 ms replay correspond to segment **11 at 37.091554152 s and 37.396380454 s**. Review segment 11 from **27.091554152–47.396380454 s** to cover both ±10-second windows. Both commands were accepted on the actual recorded drive; this is a replay regression, not a newly discovered device rejection.

## Offline build verification

With matching submodules and the build dependencies installed, the targets checked were:

```sh
# From the openpilot repository root:
scons -j2 --minimal openpilot/selfdrive/pandad/pandad
# From panda/, with opendbc import resolving to this checkout's opendbc_repo:
scons -j2 board/obj/panda_h7.bin.signed
```

These were offline ARM64 host and debug H7 builds. They do not establish device/road compatibility or MCU timing. No diagnostic instrumentation or fresh-metadata experiment is included.

## Separate installation checkpoint

No installation or drive is authorized by this draft. After review, a separate device step should:

1. Record and preserve the installed app/opendbc/Panda revisions, current signed firmware/signature, fingerprint override and dirty-file diff. Preserve raw logs and current Params; do not reset learned parameters as part of this change.
2. Review both exact PR heads and confirm the fixed geometry is appropriate. Prepare the corresponding app and firmware together while offroad; changing only the host source or only the safety firmware would mismatch measurement sources.
3. Verify runtime checkout and submodule SHAs, loaded firmware provenance, `FORD_EXPEDITION_MK4`, the named pinion startup log, CP pinion bit `2`, and safety pinion bit `8` without disturbing other bits. Confirm valid pinion reception and no RX faults before any separately approved collection drive.
4. Retain a coordinated rollback: host base `f35e6e5eb11ca9cddad3ed945d44d8f191679240`, opendbc `eb70bef4f56d2e9e91cb5a0a8374890a6acfaf51`, Panda `75aa44bec9140849868239b1f1e3f22624adb8fe`, and matching baseline firmware. Restore the preserved local override/diff without overwriting unrelated changes, then verify runtime provenance and flags again. Baseline signed H7 SHA-256: `6550d93929d05184c0946c10c6b08ff5a6d9925f667a737531f8f95a8a31f673`.

Do not assume a source checkout alone proves which firmware is running. The actual installation, rollback execution, MCU timing validation and candidate driving performance remain untested.
