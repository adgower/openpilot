# Navigator model-speed forecast integration

This local baseline extends OEM ACC Vision Curve Speed to admit a camera model speed forecast before a geometric curve qualifies. It selects the robust three-to-ten-second forecast with sustained acceleration at or below -0.10 m/s², as selected in the prior offline comparison. The existing FordPrefVisionCurveSpeed toggle enables it on the Navigator in OEM ACC mode. The toggle still defaults off. No new setting or direct throttle/brake path was introduced.

The parent is c3cc6a8833bcb0d6ff11564d701ce458eaefdc92. Its opendbc pin df4de50b20b908ca622ea760782fa7801884b675 and Panda pin bf14f329132082db7559ddca33598be7c6272528 are retained. Existing steering and Alpha tuning, including 1.20/0.97, use the same child source.

## Runtime behavior

The observer evaluates once per model sample and caches its result for control callbacks. Driver input and sensor invalidation clear the cached authority between model samples. Entry requires a robust forecast drop of at least four mph for 0.6 seconds, fresh no-lead radar and eligible OEM ACC at 40–95 mph. The robust speed uses the twentieth percentile of future speed samples and at least three low samples spanning one second. The target is dynamic and stays within the original set speed and existing 25 mph floor. This uses no GPS, route identifier, mapped ramp or fixed curve speed.

Qualified forecast demand is shared by initial requests, missed-press retry bounds and restoration. Low lane probability alone does not block a qualified forecast. Valid corridor geometry, branch checks, manual overrides and existing acknowledgement reservations remain in force. Restoring a forecast-owned reduction requires explicit fresh clear evidence and no geometric slowdown demand. Ownership clears when the original ceiling is acknowledged. Invalid model data differs from a clear forecast. A fresh stop request pauses forecast authority without permanently faulting the cruise session; physical brake or gas intervention retains its existing override behavior.

## Replay

The production VisionCurveContext replay processed 148 saved segments across eleven routes and 840,547 control callbacks. It parsed 427,232 recorded EngBrakeData frames and 85,865 steering-switch frames. Model timestamps and physical switch data were recorded. Setpoint acknowledgements were simulated with normal and delayed responses and injected first missed SET−/SET+ requests.

The exact route 2c3 exit first requested SET− at +381.763 seconds, 4.965 seconds before recorded brake disengagement. Normal acknowledgements produced sixteen reductions from 65 to 49 mph. Dropping the first request resulted in a retry of the same 64 mph target at +383.1625 seconds and subsequent reductions. Low-confidence requests continued and all bounded-setpoint assertions passed. All three checked straight-like control windows produced no added requests.

Broader coverage increased SET− from 200 to 304, SET+ from 143 to 154 and direction changes from 23 to 29. Two new short reduction/restore episodes around 63–64 mph followed the immediate model acceleration weakening above the -0.10 threshold while the future speed remained lower. The sole restore-to-new-reduction gap under three seconds was also present on the baseline. The user accepts some straight-road slowing. These additional episodes remain a comfort tradeoff for road review. Neither replay nor these classifications prove an improvement in actual vehicle motion.

Recorded setpoint changes remain external changes and can reset the counterfactual session. The exact later exit contains no such changes. All motion and future model outputs stay fixed, so alternative vehicle response is unverified. The replay compares against pristine c3cc6a883 context and controller sources. A focused rerun on the final formatted source reproduced all five exact-exit request sequences.

## Checks

- Minimal Mac build passed with scons --minimal -j4.
- Full Mac development build passed with scons -j4.
- Focused existing/new Vision Curve Speed, observer, context and handoff regressions passed with 84 tests and sixteen subtests.
- Other car regressions passed with 560 tests, five skipped and seven native-cruise class items excluded. These overlap the focused tests.
- Ford vehicle and native safety regressions passed with 141 tests, 81 skipped and 12,068 subtests.
- Ruff and git diff whitespace checks passed.
- The default-disabled helper matched the original controller commands and state over 2,400 synthetic frames.

The unfiltered broader car run encountered five native cruise-simulation failures before its failure limit. A pristine-parent reproduction confirms that the 20-second 5 m/s case finishes at 4 m/s with QP status 3/MPC status 4 and imports no forecast module. Relevant planner, solver, maneuver, plant and test sources match c3cc6a883 exactly. The underlying native solver cause remains unresolved. Thus the entire car suite is not green. No planner or solver tuning change was made to mask it.

The Mac build used cached nanobind 3.1.0 from the lockfile. Required LFS font/model assets were hydrated from local bytes after verifying their SHA-256 hashes against the pinned pointers. No asset or dependency source change belongs to this integration.

ARM build/timing, live acknowledgement acceptance, parked startup and road evaluation remain pending. The Comma was not accessed or changed for this integration.

## Reproduce the command replay

Use the matching Python environment and the baseline checkout plus opendbc_repo on PYTHONPATH. Export the pristine parent context/helper files into a private directory using git show c3cc6a883: followed by their repository paths. A private manifest contains one full rlog.zst path per line.

```sh
python docs/navigator-integration/vision_forecast_replay.py PRIVATE_MANIFEST --baseline-source PRIVATE_PARENT_SOURCES --output PRIVATE_RECEIPT
python docs/navigator-integration/vision_forecast_validate.py PRIVATE_RECEIPT
```

Private receipts are retained under /private/tmp/nav-context-runtime-all.json, /private/tmp/nav-context-runtime-summary.json and /private/tmp/nav-context-runtime-reversals.json. Build and test output uses /private/tmp/nav-forecast-baseline-*.log. Native parent-failure proof is saved with the shared Navigator research documentation.
