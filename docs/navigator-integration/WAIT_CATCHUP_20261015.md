# Bounded request-noise catch-up correction

This paired revision ports the report-local 2026-10-15 wait/catch-up prototype into the existing BudgetedTrackingPrototype selector. The parent change pins the paired child; it adds no new preference and does not change the default-OFF path.

The recovery predictor distinguishes fractional recovery from changes caused only by a smaller request denominator. A measured-recovery cap preserves wait/filter continuity for those fluctuations. A sustained unwind (three consecutive declines exceeding 5% cumulatively) retains predictive release, and the instantaneous 70% response threshold is unchanged. Every newly admitted exception consumes the existing0.00025rad·s episode budget. Existing quiet-dwell rearm, driver/freshness/lane/platform guards, caps/slew and25mph fade remain unchanged.

Owner1.20/0.97, Navigator geometry, pinion selection, longitudinal configuration and update policy are unchanged. No native safety/firmware/OS or dependency revisions change.

The frozen report prototype passed160-segment native replay and552 hypothetical plant/abort cases; this integration passes284 focused/Ford/selection tests,79 skipped and12,063 subtests. Fresh reviewer found a developer-path override in the new test; removed before commit, with no remaining important findings on re-review. The expanded188-segment integrated/frozen-prototype parity replay and isolated target checks are recorded in the private/local deployment report before activation. Build, replay and parked checks do not establish a road-response improvement.

A recorded21–22mph fixture gains about0.20s of continuous correction before exhausting the unchanged budget. Later pulses can be suppressed until a full valid quiet rearm. This small behavior change is not proof that the15mph or22mph tracking deficit is fixed. The newest drive still shows low-speed body/pinion response disagreement and has no matched-road causal comparison.

Device installation requires a fresh verified backup and rollback references, ignition-off activation and passive offroad/powered-Park acceptance. Existing preferences must be preserved, including any owner changes since the older snapshot. No live steering/diagnostic injection is part of acceptance.
