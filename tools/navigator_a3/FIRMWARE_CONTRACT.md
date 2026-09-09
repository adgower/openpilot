# Shared firmware Angle checks

This checkpoint starts from parent a1e415c5035b460c0b4d15f6ee5c255c45de2fe2 and child 82cce928f8eb57ee9e669261bbffdfa1bc5d0142. It preserves Python controller/runtime behavior, Chestnut models, Navigator parameters, prescribed trajectories and A2 CAN generation.

## Runtime contract

`ford_angle.h` is included by the actual Ford safety header. Ford initialization resets its state. The generic RX dispatch passes the already calculated RX validity to the Ford-only observer, including rejected samples; other platforms do not enter that observer. Final nonneutral CAN-FD path-angle commands invoke the shared checker in the real TX hook and then always fail independent production permission. Legacy CAN path angle retains its existing gate.

The checker validates actual wire fields, profile, RX freshness, speed, relay condition, path-angle rate, and the existing curvature helper's permission/measurement/acceleration/jerk/aggregate-rate protections. Its inverse is an experimental algebraic assumption, not a demonstrated EPS transfer function. Per-command limits and gains are unchanged. The strict and aggregate historical headers remain byte-identical.

Checking and acceptance are separate. `ford_a3_check` updates attempted-message buckets as the upstream helper does but restores the accepted desired/power reference. `ford_a3_commit` can commit a successful pending result only in the host wrapper; production has no caller. The real production wrapper isolates experimental attempt history from A2 curvature history, discards the pending result and unconditionally returns false. Permission revocation is never restored or suppressed. No safety parameter, profile, debug/release setting or host claim selects active admission.

This is not a security boundary against arbitrary source edits. It is an invariant of the reviewed source and its available configuration/build paths.

## Handoff

The existing shared lifecycle and scheduler remain unchanged: driver intervention neutralizes demand; releasing the wheel makes an otherwise valid, fault-free session eligible for a scheduled bounded proposal. No catch-up updates occur. Direct A3 rejection, permission loss and configuration faults stay persistent; neutral/returned frames and driver release do not clear them. Host simulation tests this automatic bounded re-entry policy, not the EPS's physical response to it. Production still cannot execute A3 proposals.

## Reproduction

Use the exact interpreter and native dependency paths in the delivery `run-verification.py`. The ordinary Ford safety suite additionally needs cffi; this run reused cffi/pycparser from the isolated Panda build environment through a test-only directory, without installing into shared Python.

```sh
python -m tools.navigator_a3.firmware_contract --repo /path/to/matched-opendbc --output /path/to/new-output
python -m pytest tools/navigator_a3/test_firmware_contract.py -q --import-mode=importlib
```

The audit builds both historical aggregate and shared implementations, plain and instrumented, and compares admission, reason bits, accepted history, attempts and permission state for identical inputs. It includes jitter, compressed arrivals, bursts, timer wrap, driver transitions, isolated rejection and malformed/excessive commands. Instrumentation parity is checked separately. Reason locations belong to their respective source revisions. Synthetic admission is separate from recorded donor response.

A primed production probe verifies that a modest Angle command passes shared checks but is finally rejected, cannot leave a pending commit, preserves nonzero A2 desired/rate/measurement history, and permits a subsequent valid A2 command. A finite arithmetic parity sweep covers all 2,048 encoded angles, 60 integer speeds and four profiles; it is not exhaustive over fractional speeds or a proof of physical limits.

## MCU and target build

The delivery contains a locally built H7 ELF, binary and debug-signed image. Panda source remains pinned at 75aa44bec9140849868239b1f1e3f22624adb8fe. The pinned SConscript uses `opendbc.INCLUDE_PATH` and signs with `board/certs/debug` through `board/crypto/sign.py`. Compiler version, exact command, source hashes and actual artifact hashes are recorded separately. No release private key is used. Firmware initially failed to link host libm functions; the final header uses bounded finite arithmetic and the final build passes.

Python/controller and cereal schemas are unchanged. A new on-device checkout still requires matching native libraries and Chestnut model compilation. Use the refreshed package's build-only procedure after a separately authorized inventory/transfer. Neither an H7 build nor its debug signature is permission to flash, boot or drive.

Keep rollback matched across parent, child, startup configuration, native/model artifacts, OS and firmware. The previous H7 artifact is preserved in its earlier build directory; source rollback alone does not change installed firmware or AGNOS.

## Exact activation evidence still missing

1. A supported relationship (including uncertainty, both signs, speed dependence and transients) between final encoded path angle and response of this Navigator EPS. Identifier NL14-14D003-AE and the separate NL14-3F964-AB diagnostic response do not establish that relationship.
2. EPS behavior during sustained driver override, inactive command intervals, release, and the first active command after release. The donor recording is not a controlled handoff envelope or a proof that the proposed reset-to-neutral/ramp matches EPS state.
3. A production-independent enforceable bound derived from that evidence, plus behavior during the interval between command rejection and host observation. Missing echoes do not establish execution or a finite host detection bound.

The existing donor route and published BluePilot claims remain supporting leads with their recorded model/wheelbase/controller differences; they do not close these gates. Firmware acquisition stays deferred. If identifiable technical evidence cannot close them, the next physical review must specify a controlled facility/bench, competent supervision, a synchronized host/MCU/EPS capture, calibrated yaw/steering/speed/driver measurements, timestamp accuracy, approved excitation limits and stop criteria. No excitation amplitudes, driving speeds, flash or test procedure are authorized by this document. Those values must come from that review, not this algebraic inverse.
