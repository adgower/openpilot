# Measured-response checkpoint

This offline checkpoint starts at parent `3d8536b53` and child `e9c54bc8`. It preserves the earlier experiment and report directories. Production code, the independent C evaluator, firmware, vehicle parameters and the device remain unchanged.

## Controller and replay

`Inputs` now carries measured curvature, its timestamp and validity. Above 9 m/s, missing, invalid/nonfinite or stale/future evidence produces a distinct neutral reason. Validity includes ages 0 through 100 ms. At or below 9 m/s, stock behavior is retained without this measurement clamp.

The requested target is clipped to measured curvature ±0.002 /m before the existing acceleration/jerk, angle-rate and encoded-grid limits. A history outside the measurement band moves gradually toward it; the band is not imposed as an instantaneous bound that would force a jump. Raw requested angle, requested/effective gains, measurement age, measurement-limited target and final algebraic equivalent remain distinct outputs. None establishes an EPS transfer function.

Replay derives the actual limiter input as `-carState.yawRate / max(carState.vEgoRaw, 0.1)`, using native as-of samples and the oldest dependency timestamp. The old chart's `derived.yawCurvature` uses filtered speed, so it remains context and is not substituted for the stock limiter measurement. Tests verify the raw-speed/sign choice and missing-data handling.

## Comparisons and coverage

The original A2 run-2 window and both curve inputs retain their previous admission results for all four profiles: 80 accepted active A2 commands; 728 accepted active curve-17 commands; 1,729 accepted active curve-29–30 commands plus one initial missing-RX/speed rejection. Neutral counts remain 0, 340 and 404 respectively. Historical motion is unchanged because these are synthetic proposals against saved RX.

Across all plotted A2 maneuver windows, the measurement target is clipped in 482 rows per profile and encoded proposals differ in 499 rows per profile. These windows overlap and are not combined into a synthetic CAN stream. Both ordinary curve command sequences are unchanged by this limiter.

Run 2 is mostly below the threshold. An additional, separate complete A2 run-19 window at approximately 30 mph directly exercises the new clamp: all 83 proposals per profile pass the unchanged compiled evaluator. This extends coverage; no prior compiled run-19 baseline was available, so it is not an admission-improvement claim. The original comparison remains intact.

The artifact report includes native-clock plots, neutral counts, maximum command differences, exact command bytes, before/after evaluator snapshots and source/configuration hashes. A default report spans every exported maneuver window; `core_replay --run` can render a single window without joining overlaps.

## Unexpected rejection and transport evidence

See `measurement-rejection-audit.md` and `rejections/rejection-cases.json` for paired control/fault sequences. One stale frame, corrupt checksum, loss of controls permission or excessive encoded command is injected into an otherwise valid sequence. Proposal inputs/history do not depend on the evaluator result. No special resets, retries or recovery pulses occur.

Recorded transport data is saved separately as `recorded-transport.json`. It includes historical LMC2 publication/return/rejection events and unmodified aggregate `safetyTxBlocked` observations. Returned frames do not establish EPS execution, and aggregate counts are not per-command reasons. Historical A2 feedback cannot acknowledge synthetic A3 commands. Early-return checks that were not executed are not reported as passed; arbitrary untraced RX rejection remains unresolved.

## Verification and remaining work

The checkpoint's combined suite passed 390 tests and 9,011 subtests, with 74 skips, before packaging. Tests include all four profiles, both signs, measurement freshness endpoints, 9 m/s boundaries, changing speeds, gradual band entry, driver handoff, encoded quantization and unchanged negative safety tests. Independent review reran 148 controller/replay/fault tests.

Use the prior handoff's Python environment/import instructions; the new replay manifest records exact commands and inputs. New parent and child bundles have the same frozen prerequisites as earlier packages, and contain the local commits through this checkpoint. No new dependency download or native/schema rebuild is required for the added Python diagnostics.

Physical Angle mapping, gain validation, coordinated behavior after unexpected rejection, complete lifecycle/recovery design and production firmware integration remain unresolved. Historical replay and host C tests do not establish on-vehicle availability or improved handling. Installation still requires a separate review and approval.
