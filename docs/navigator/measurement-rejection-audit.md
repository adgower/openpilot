# Measured-response rejection audit (offline)

The executable audit compares a fresh control run with a fresh single-fault run. It never passes compiled acceptance, rejection reasons, evaluator history, or historical Panda feedback into the controller. No safety decision code, firmware, live controller, recovery logic, device state, or transport is changed.

Run from the parent worktree with the child on PYTHONPATH:

```sh
PYTHONPATH=/private/tmp/navigator-a3-opendbc:/private/tmp/navigator-a3 /private/tmp/navigator-diagnostics-venv/bin/python -m tools.navigator_a3.rejection_diagnostics --stock-repo /private/tmp/navigator-a3-opendbc --output /private/tmp/a3-rejections
PYTHONPATH=/private/tmp/navigator-a3-opendbc:/private/tmp/navigator-a3 /private/tmp/navigator-diagnostics-venv/bin/python -m pytest tools/navigator_a3/test_rejection_diagnostics.py -q
```

`rejection-cases.json` contains all executable packet bytes, timestamps, controller inputs/proposals, compiled before/after history, exact executed failed stock checks, named candidate reason bits, source hashes, compilation commands, and full control/injected comparisons. The referenced compiled stock source is `b4ef5e1cf406ff143fa67bdbfb154739d43279c9`. Candidate code is the unchanged host-only `opendbc/safety/tests/navigator_a3/core_path_angle.h` from the child; its hash is recorded independently of the stock pin.

## Fixtures and observed results

Every run uses 25 m/s, 24 active proposals at 50 ms cadence, a constant general-controls curvature request of 0.0002 /m, and an independently supplied fresh, valid measured curvature of zero. Trusted-encoding synthetic Ford brake/PCM/yaw RX follows six warmup samples and then runs every 100 ms. Cruise-on is an explicit encoded test fixture, not a recorded engagement or a forced `controls_allowed` assignment. There is exactly one perturbation at 500000 us. Each paired run initializes once, executes the full prefix and suffix without resets, and uses its own compiled library. All control packets and proposals are admitted in this fixture.

| Single perturbation | First rejected proposal | Executed reason | Unmodified suffix |
| --- | --- | --- | --- |
| Drop one brake RX | 550000 us | `A3_FRESHNESS` (2) | One rejection; later scheduled valid RX removes staleness without a special action |
| Flip one brake checksum bit | 500000 us | `A3_FRESHNESS` (2) | 16 rejected proposals; invalid RX clears controls, later fresh RX does not re-enable them |
| Change one cruise RX from on to off | 500000 us | `A3_STOCK_ENVELOPE` (32), `!controls_allowed && steer_control_enabled` | 16 rejected proposals with controls off |
| Replace one wire proposal with angle=700 CAN units | 500000 us | `A3_PATH_RATE` (512) | No later rejection for this bounded constant-request fixture |

The stale case omits one frame; it does not inject timestamps into the evaluator or overwrite RX history. The corrupt case does not repair checksums, quality, or health. Cruise-off leaves all monitored RX packets intact. The excessive TX case changes only transmitted bytes: the controller's proposal history advances normally and the candidate's last accepted angle/time remains unchanged. The unchanged suffix is not a recovery routine. These results describe this fixture only, not the handling of arbitrary requests after rejection.

Snapshot continuity tests compare each exported after-state with the next before-state, excluding only the clock field. Every control proposal is identical to its injected counterpart, including the independent measured input and proposal state. All 11 tests passed after adopting the explicit measured-input ABI. Snapshots expose stock `desired_last`, measurement min/max and controls, plus all 14 exported candidate fields. They do not expose every internal stock RX counter/global; no claim of a complete stock memory dump is made. Candidate timestamps are decoded as uint32 and its angle history as signed int32.

## Attribution and instrumentation limits

Candidate `core_path_angle.h:72` combines unseen, invalid and older-than-100000-us RX into `A3_FRESHNESS`; the recorded per-channel seen/valid/time fields distinguish them. Line 75 compares wire path-angle change against the speed-dependent limit. Line 79 returns before stock curvature checks on an early candidate fault. Line 84 calls frozen stock curvature validation, and line 85 advances candidate accepted-command history only on admission.

The stock instrumenter records executed true violation assignments/OR expressions in Ford and lateral headers, preserving expression evaluation and side effects. For controls-off it reports `opendbc/safety/lateral.h:329`, `!controls_allowed && steer_control_enabled`, from the archived stock tree. Its absence on an early `A3_FRESHNESS`/`A3_PATH_RATE` rejection means **not evaluated**, not that all stock predicates would pass. Fresh checksum-valid RX following corruption exposes the later controls-off failure. In stock `safety.h:101–106`, invalid checksum/quality/counters clear controls; checksum validation is at lines 170–190. Those RX validation predicates are not individually instrumented by this audit. The known injected checksum bit and observed state transition support the fixture's causal explanation, but an arbitrary historical RX rejection cannot be assigned an exact predicate from this trace. Rows without traced checks or candidate reason bits explicitly retain unresolved attribution.

## Frozen Panda transport semantics

Parent source is frozen at `3d8536b53` and Panda source at `75aa44bec9140849868239b1f1e3f22624adb8fe` (read from `/private/tmp/navigator-a3-panda-pinned`).

- Parent `openpilot/selfdrive/pandad/panda.h:23–24` defines rejected offset `0xC0` and returned offset `0x80`. `panda.cc:262–269` starts with the physical bus and adds each flag's offset. A source of 192 is a rejected bus-0 echo; 128 is a returned bus-0 echo. These must not be treated as ordinary bus-0 RX.
- Panda `board/drivers/can_common.h:179–194` calls the TX safety hook unless explicitly skipped. A safety rejection increments `safety_tx_blocked`, marks returned=0/rejected=1, recalculates the transport checksum, and attempts to enqueue the rejected packet to the host RX queue. Queue overflow can therefore prevent a one-to-one observed echo count.
- Panda `board/drivers/fdcan.h:124–138` sets TXBAR and constructs the returned=1/rejected=0 host echo. This reports the transmit-path operation; it is not an EPS acknowledgement or proof of bus completion, actuator execution, or vehicle response.
- Panda `board/main_comms.h:24` copies the counter into health; parent `openpilot/selfdrive/pandad/pandad.cc:112` publishes it as `pandaStates.safetyTxBlocked`, declared in `openpilot/cereal/log.capnp:561`. It is an aggregate count of safety-rejected TX packets, not a boolean, per-message reason, or LMC2-only count. Panda `board/main.c:51` resets it when safety mode is set. A delta spanning reset/wrap or incomplete logging needs separate treatment.

`classify_src` returns `kind`, decoded `bus`, and `acknowledges_synthetic_a3=False`. Normal physical buses 0–7 and their single-tag offsets are decoded; unusual/combined tags remain unknown. Recorded A2 sendcan, rejection, return and aggregate health events describe that historical run. Even matching payloads/times cannot acknowledge counterfactual A3 proposals that were never transmitted. No historical feedback is used to seed, reset, accept, retry or alter these synthetic proposals.
