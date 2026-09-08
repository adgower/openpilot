# Core path-angle experiment: integrated, activation blocked

This checkpoint connects the existing calculation to the Ford controller, adds bounded runtime transport observation and an explicit host inhibition path, and compiles the independent final-angle gate into real H7 firmware. A2 is the default. It does not enable Angle driving.

Starting parent: `5a75ba5de9e65b5039810e0669b6dc5ae8defbda`; child: `27e76255e4b0b1e372759e970227ec7a76f921d6`; Panda: `75aa44bec9140849868239b1f1e3f22624adb8fe`. Final paired commits are in the packaged source manifest and parent gitlink. No device, OS, model, learning or remote branch was changed.

## Startup and behavior

- `NAVIGATOR_A3_MODE` is read once when constructing the Ford controller: absent/`a2`, `shadow`, or `requested`. Values are case-sensitive. Unknown values fail explicitly.
- `NAVIGATOR_A3_PROFILE` must be an existing named profile for shadow/requested. There is no arbitrary numeric override or UI. Both modes require the fixed Expedition CAN-FD identity and A2 wheelbase 3.1115 m. This identity check is not a new physical specification.
- A2 remains byte-equivalent to the frozen controller. Shadow computes isolated proposals at the existing steering cadence while publishing A2 frames unchanged. Requested mode is permanently inhibited: no active A2 fallback, host-applied lateral/longitudinal/enabled flags false, actual steering neutral, existing `steerUnavailable` immediate-disable/no-entry behavior.
- The strategy receives the same general-controls curvature. No normal model preview or lane-position shaping was added; scripted maneuvers retain their prescribed input. The stock measurement band, rate limits, integer-grid quantization and four provisional profiles are retained.
- Source time comes from `carControl.logMonoTime`. Measurement time is the older successfully parsed yaw and brake-speed sample timestamp. Above 9 m/s the existing 100 ms validity contract still applies; missing timestamps are never replaced with now or a fabricated zero measurement.

## Runtime evidence and lifecycle

The runtime observer stores at most 2048 steering publications, eight Panda histories and 16 pending diagnostic observations. It matches physical bus/address/length/full bytes plus stream/provenance and native preceding order/time. All retained matching publication orders are returned. Once any publication history is evicted, attribution stays incomplete for the session; it never becomes uniquely matched merely because earlier possibilities were discarded. The compact published diagnostic lists at most 16 candidate orders and explicitly flags truncation/count. Normal CAN logs retain the full wire evidence for offline reconstruction. Diagnostic overflow has a cumulative visible counter.

The runtime stream UUID is carried in route-logged diagnostics; it is not represented as a known route filename. Live transport, recorded transport and generated replay commands are distinct. In REPLAY, newly generated publications are excluded from historical correlation. Candidate proposals are always labeled synthetic; returned/missing frames never acknowledge EPS execution. Recorded A2 feedback cannot acknowledge an A3 proposal.

Startup waits for ControlsReady and matching Panda configurations before arming configuration enforcement. Invalid or transitional startup health is diagnostic evidence, not a permanently latched fault. Unarmed proposals are invalid with configuration_pending. Once armed, configuration faults, invalid/out-of-order health and active permission loss latch. Direct active steering rejection is retained as a fault; returned frames, driver release and fresh RX do not clear it. Restart creates a new observer session; it does not grant transmission permission. Requested mode remains blocked regardless.

Ordinary invalid/stale strategy input and driver press remain neutralizing controller-state resets with the previously bounded return when evidence becomes valid. These are distinct from latched transport/configuration faults, and neither produces retry pulses. No new feedback-dependent recovery or control policy is enabled on the vehicle.

`CarOutput.navigatorA3` is an additive schema field containing version1, mode, inhibited, reason and diagnosticsJson. Old A2 fields remain unchanged. Diagnostics include native input/output state, requested/effective gains, proposed bytes, actual controller frame, actual publication/transport evidence and provenance. Nonfinite evidence becomes JSON null with an explicit flag; it cannot crash shadow logging. Existing selfdrived consumes inhibited requested status as steering unavailable. No new alert ID or acceptance telemetry is invented.

## Independent enforcement

Ford production safety checks the actual final encoded path-angle field in classic and CAN-FD messages. Only exact neutral raw1000 is admitted to the remaining stock checks. No profile, safety parameter, debug flag or host claim can bypass this gate. This factors the previous stock nonzero-angle rejection into an explicit named production check; it does not widen stock acceptance.

The algebraic inverse-map evaluator remains host-only research code, with its positive and negative tests intact. It is not included as an asserted physical model in firmware. Firmware tests exhaust all2048 raw encodings and exercise actual TX admission across controls states, modes and parameter combinations. A real Cortex-M7 STM32H725 binary was built, debug-signed with the repository debug certificate. It is not a release-signed artifact and must not be installed merely because it exists.

## Builds and reproduction

The artifact package contains actual compiler logs, dependency freezes, generated schema/native evidence, H7 ELF/binary/debug-signed application, source pins and checksums. `build-native.sh` records the exact local schema/msgq/params commands. Native modules are macOS arm64 test artifacts, not comma-device Linux binaries. Root SCons still stops on missing acados; no full target-device application build is claimed.

Host integration tests (no Car() construction or device opening):

```sh
cd /private/tmp/navigator-a3
DYLD_LIBRARY_PATH=/private/tmp/a3-native-build \
PYTHONPATH=/private/tmp/a3-native-build/msgq:/private/tmp/navigator-a3-opendbc:/private/tmp/navigator-a3:/Users/alex/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/lib/python3.12/site-packages:/private/tmp/navigator-diagnostics-venv/lib/python3.12/site-packages \
/private/tmp/navigator-a3-panda-integration/.buildvenv/bin/python -m pytest -q openpilot/selfdrive/car/tests/test_navigator_a3*.py
```

Real H7 rebuild, using the packaged frozen environment (no USB/Panda construction):

```sh
cd /private/tmp/navigator-a3-panda-integration
PATH=/private/tmp/navigator-a3-panda-integration/.buildvenv/bin:$PATH \
PYTHONPATH=/private/tmp/navigator-a3-opendbc \
/private/tmp/navigator-a3-panda-integration/.buildvenv/bin/scons -j4 board/obj/panda_h7.bin.signed
```

Before rebuilding, verify the child source pin/hash and actual `opendbc.INCLUDE_PATH`. The build log must reference this child. Do not install standalone Panda dependencies that silently select opendbc master. Debug signing does not establish release authorization or hardware acceptance.

Recorded bounded-observer replay:

```sh
cd /private/tmp/navigator-a3
PYTHONPATH=/private/tmp/navigator-a3 /private/tmp/navigator-diagnostics-venv/bin/python \
  -m tools.navigator_a3.integration_replay --events /path/to/transport-events.jsonl --output /path/to/new-results
```

A2 and both curve inputs use the previous extraction's hashed files. Core proposal replays use the prior maneuver/curve CSV inputs; the previous raw-admission overlays are retained as cached evidence with unchanged experimental evaluator bytes, not claimed as new firmware telemetry. Production gate and compiled regression tests are separately rerun. No synthetic motion improvement is claimed.

## Installation and rollback requirements

There is no automatic installer or launch command in this package. Launching pandad can flash firmware; root/model builds may interact with Chestnut. Those are not read-only actions.

Before any later installation: explicitly authorize a read-only device inventory of actual OS, boot checkout, submodule/dirty state, model artifacts, startup configuration, staged/prebuilt state and observed firmware identity. Preserve the actual matched A2 source/configuration/firmware and learned-state backup. Finalize device build commands only against that inventory. Do not change AGNOS or import BluePilot branches.

Python changes require this paired child and parent. The schema addition requires matching generated/native bindings and consumers on the target. Firmware source changes require the corresponding H7 build/signature; a Python selector is not a firmware rollback. Source bundles here are incremental and require their recorded base commits. Unbundle into an isolated repository with those bases, verify refs and hashes, then materialize matching submodules; never reset an unrelated dirty checkout.

Activation remains blocked by the unvalidated physical Angle envelope. Full Linux target build, installed firmware identity, lifecycle timing under actual load, physical response, gains and driver handoff remain unverified. The delivered integration is executable software and real build evidence, not approval for driving with Angle.
