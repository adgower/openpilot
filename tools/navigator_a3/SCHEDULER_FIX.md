# Emission-based Angle shadow scheduler

> Timing provenance correction: the strict 50,000 us gate described below belongs to our historical host-only evaluator. It is not an established Ford/EPS requirement. The scheduler remains a deliberate host policy. See [TIMING_CONTRACT.md](TIMING_CONTRACT.md) for the separate aggregate-rate experiment; prior results and artifacts remain historical evidence.

Base parent `4b89fd3d4b59d5b6f1a41342d5f96a4e4bf9b03b`, child `0322627c07c4993e631b3974dc9bf66ac9143362`.

## Implemented behavior

`ShadowScheduler` is checked on every existing 100 Hz controller invocation. Actual A2 CAN scheduling and contents are unchanged. Active hypothetical proposals are emitted no earlier than 50 ms after the most recent hypothetical frame, active or inactive. An early call emits nothing, preserves demand/history/counter and waits for a later control invocation; there is no neutral pulse and no catch-up. Latest valid input is used when eligible.

The scheduler distinguishes last invocation, last emission and proposal history. Duplicate/backward times emit nothing. A call gap greater than 100 ms neutralizes; driver intervention and invalid/stale input also neutralize immediately. A continuing neutral condition emits no repeated neutral frames. Active re-entry waits at least 50 ms from the emitted inactive frame. These are host-side hypothetical transitions, not proof of EPS inactive semantics.

Pure strategy validation/limiting is probed on an immutable copy on each invocation so immediate invalidity can be detected while waiting; only an emitted result commits proposal history. The existing pure strategy, four profiles, bounds and C checks are unchanged. Here `scheduled_update` means the scheduler owns cadence and bypasses the legacy pure-strategy timing check; it does not assert the caller itself runs at exactly 20 Hz.

The separate hypothetical four-bit counter advances only on emission. It is not an acknowledgment. The runtime observer and fault latches are unchanged, including requested-mode startup inhibition and shadow's separate non-exempt fault latch.

## Diagnostic compatibility

Controller JSON schema version is 3; transport envelope schema stays 2, cereal wire version stays 1. `output` and `proposed_frame` are null when no frame is generated. `scheduler` records waiting, reset, new-proposal status, reason, next eligibility and emitted counter (null when absent). Timing explicitly indicates control-rate checking. `actual_frame` is populated only on iterations that actually publish A2 steering; off-cadence shadow proposals do not inherit an old A2 frame.

The counterfactual reader supports v3 scheduler input streams and absent proposals without requiring an A2 counter. v1/v2 retain their prior validity policies. Original diagnostics remain unchanged in the output, and no historical transport event acknowledges hypothetical bytes. CLI counts distinguish supported no-frame calls from missing evidence.

## Verified timing fix and remaining failures

The original otherwise-valid calls at 100/149/200/249/300 ms now emit at 100/200/300 ms. All three are accepted by the unchanged experimental evaluator. In a 100 Hz caller fixture, the early 149 ms call waits and 159 ms emits; it does not wait until the next 20 Hz A2 slot. Positive cases cover both signs and all four profiles, with driver and long-gap neutral transitions.

Independent C checks were not loosened. When the first packet is separately delayed 10 ms, 59 ms host spacing compresses to 49 ms presumed MCU spacing, reproducing cadence and subsequent rate rejection. An independently corrupted excessive command also remains rejected. Host history is identical across these transport/fault cases; no oracle repairs it. Therefore this fixes the demonstrated host scheduler incompatibility, not arbitrary transport timing or rejected-command-history uncertainty.

Current production still blocks nonneutral Angle. Experimental acceptance is not physical validation. A2 neutral-angle mode-1 messages remain subject to normal checks. Physical mapping, ECU handoff and response during rejection/detection delay remain unresolved.

## Saved-drive comparison

`scheduler_replay.py` reuses the preserved legacy counterfactual input stream, only at its 26,660 supported roughly-20-Hz timestamps. It does not invent intermediate 100 Hz measurements or claim full live replay. It emits 9,311 hypothetical frames: 9,241 active and 70 neutral; 17,349 supported calls emit no frame. Original faults and inferred-validity limitations remain visible. Unsupported holes start separately numbered hypothetical streams. These counts do not establish the live 100 Hz emission rate or steering improvement.

## Reproduction

Run on the Mac from `/private/tmp`, not on the device:

```sh
PYTHONPATH=/private/tmp/navigator-a3-opendbc:/private/tmp/navigator-a3 /private/tmp/navigator-diagnostics-venv/bin/python -m tools.navigator_a3.scheduler_admission --stock-repo /private/tmp/navigator-a3-opendbc --output /private/tmp/scheduler-admission-reproduction
PYTHONPATH=/private/tmp/navigator-a3-opendbc:/private/tmp/navigator-a3 /private/tmp/navigator-diagnostics-venv/bin/python -m tools.navigator_a3.scheduler_replay --input /Users/alex/.codex/visualizations/2026/09/05/01a07385-2ba0-7b50-8447-0f8152e58e2c/navigator-shadow-drive/counterfactual/counterfactual-timeline.jsonl --output /private/tmp/scheduler-replay-reproduction
```

The report's `run-verification.py` contains the exact full test command. It uses the previously built isolated native msgq/params dependencies, whose build recipe is `build_host_test_dependencies.py`. No schema definitions changed; no new schema/native library rebuild is required by this Python-only integration. Host C admission libraries are compiled independently for these tests. No MCU firmware was built, flashed or started, and no on-device build was attempted.

The paired local bundles are incremental and require the base commits above. Verify them in their corresponding repositories and inspect in separate worktrees. Source rollback uses those paired base pins; it is not OS, firmware or learned-state rollback. No device state changed in this checkpoint.
