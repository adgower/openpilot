# Reliable shadow checkpoint

Base parent: `4b418552ee6b797cddbb49887494fa860a0fc251`; base child: `01bea343a7f7d4e4f8a1947539abacb59ab8c278`.

## Behavior

Only shadow mode separates a direct A2 steering rejection from hypothetical calculation eligibility. The first transport fault remains latched in `fault_reason`/`evidence_fault_reason`; a separate `calculation_fault_reason` retains the first non-exempt fault even after the first rejection. Configuration-pending, permission/configuration faults, source checks and measurement checks remain enforced. Returned frames cannot clear either latch. Requested mode remains inhibited and never falls back to active A2.

Runtime invokes the strategy on the existing Ford `frame % STEER_STEP` branch. `Inputs.scheduled_update=True` permits positive elapsed intervals up to 100 ms without requiring an exact lower 50 ms interval. Each scheduled invocation applies the unchanged per-update limits once; there is no wall-clock catch-up. Duplicate/backward times produce neutral proposals and preserve command history when other inputs are valid. Invalid inputs and driver intervention still reset neutral; reset timestamps never regress. Gaps greater than 100 ms reset neutral. Standalone callers default to the previous strict cadence policy. This new scheduler is shadow-only: it is not a relaxation of the independent compiled evaluator's timing check.

JSON diagnostic `schema_version=2` is separate from unchanged cereal wire `version=1`. Existing keys remain. `calculation_eligible` describes the source/transport gate; `output.reason` and `output.mode` separately describe measurement, driver, inactivity and timing outcomes. Timing contains prior timestamp, elapsed nanoseconds and scheduled-call status. Proposed frames remain hypothetical, separately recorded from actual A2 frames. The v2 offline reader requires explicit calculation eligibility and no calculation fault; only legacy v1 uses the previously documented inferred source-validity policy.

## Reproduction

All commands below run on the Mac, not on the comma. No launcher, manager, Panda service or firmware flash is invoked. Existing worktrees are `/private/tmp/navigator-a3` and `/private/tmp/navigator-a3-opendbc`.

Install build dependencies only in the isolated diagnostics venv:

```sh
/private/tmp/navigator-diagnostics-venv/bin/pip install scons==4.9.1 Cython==3.1.4 setuptools==80.9.0 comma-deps-json11==20170411.0.post98 comma-deps-zeromq==4.3.5.post98 comma-deps-capnproto==1.0.1.post98 setproctitle==1.3.7 pyzmq==27.1.0
/private/tmp/navigator-diagnostics-venv/bin/python /private/tmp/navigator-a3/tools/navigator_a3/build_host_test_dependencies.py --msgq-repo /private/tmp/navigator-staging-verify-lfs/msgq_repo --child /private/tmp/navigator-a3-opendbc --build-dir /private/tmp/navigator-shadow-native-new
```

The build directory must not already contain `msgq`. The script archives the parent's exact msgq gitlink, runs its pinned SCons build, generates unchanged cereal C++ schema artifacts, and compiles the frozen native params wrapper for macOS. It records compiler commands and artifact hashes. This is a host integration-test dependency build, not an on-device staging build. No shared Python path files are changed.

From `/private/tmp`, set `PYTHONPATH` per command to the new msgq directory, child and parent. Run pytest on Ford `test_navigator_a2.py`, `test_navigator_a3.py`, `test_navigator_a3_runtime.py`, safety `test_production_angle_gate.py`, parent `test_navigator_a3_{card,runtime,wiring}.py` and `tools/navigator_a3`, with `-q --import-mode=importlib`. The delivered `run-verification.py` contains the complete executed argument vector.

Run `tools/navigator_a3/shadow_revision_comparison.py --input <preserved-counterfactual-timeline.jsonl> --output <new-directory> --child /private/tmp/navigator-a3-opendbc` with the same Python/PYTHONPATH. The comparison loads the frozen strategy from child git history and independently runs both histories. Every supported frozen result is compared against its saved counterpart. It preserves the recorded lifecycle and never consumes transport as synthetic feedback.

## Interpretation and remaining activation work

This replay reuses saved inferred validity, not exact live SubMaster checks or reconstruction of later hidden legacy faults. A complete source stream is required to evaluate the revised live bridge; the serialized card regression supplies controlled complete evidence for that case. No historical A2 acknowledgment applies to synthetic Angle commands. Timelines show proposed command differences, not predicted or measured motion.

Active nonneutral path angle remains independently blocked in production firmware. Neutral-angle mode-1 A2 is still permitted under stock checks. No production safety or host-only C evaluator source changed. A usable active Angle admission envelope remains unvalidated; algebraic curvature inversion does not establish EPS response. Driver override and re-entry are bounded in the synthetic controller, but physical inactive-state semantics remain unresolved.

The next activation milestone is an executable production admission and inactive/handoff contract: justify every bound against independent evidence, distinguish host from MCU accepted-command history, bound rejection-detection delay, and define driver release/neutral behavior. Unsupported physical bounds stay blocked; ordinary extra driving is not a substitute for the missing evidence.

## Local delivery and rollback

Paired incremental Git bundles require the base commits listed above. Verify each bundle in its matching repository with `git bundle verify`. Import and inspect in separate worktrees; use the parent commit's opendbc gitlink for the matching child. Do not overwrite a dirty checkout. The prior parent/child pins reproduce the source baseline and are retained in the manifests. Source rollback is not OS, firmware, parameters or learned-state rollback. This checkpoint made no device changes and provides no device installation commands.
