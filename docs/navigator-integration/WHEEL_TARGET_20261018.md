# Calibrated high-speed wheel-target/unwind integration

Local paired integration of the accepted wheel-target feedback prototype. Child exports `CalibratedWheelTargetReduction` in `lateral_angle_wheel_target.py`; the original `HighSpeedReduction` remains unchanged as its independently advanced installed-policy baseline. Parent selects the new policy through the existing prerequisite-aware/default-off `FordPrefHighSpeedTracking` path. No new preference, UI control, source publisher or device setting is added.

New policy blends70–75mph, using five distinct fresh wheel-source observations, current calibrated target/body confirmation and predictive closing release. Total15%/.01rad reduction cap, native final slew,1.20/0.97,fixed pinion model, geometry and all existing lower-speed/reference behavior are preserved. A bounded native handoff tail is allowed theoretically across70mph; accepted prototype recordings have no at/below70mph changes.

Research acceptance:110tests,32sessions/637segments,0additional native rejections,default-off byte parity. In the owner-described lane-edge fixture the trial adds20trim frames while excess persists and releases before under-target exit. Frozen motion does not establish physical path/handling improvement; the model-predicted near-edge path remains a separate issue.

Fresh integration builds, full parent/child suites, source equivalence, independent review and factory/native replay receipts are recorded in the local companion `docs/research/navigator-paired-wheel-target-build-2026-10-18` under the BluePilot workspace and summarized after validation. No Comma contact, physical CAN transmission, push or installation is authorized by this integration task.

Local configured validation before commit:139focused tests; parent1,018tests/343subtests (86skips,1expectedfailure); child3,990tests/24,673subtests (1,690skips). Minimal/development Mac SCons and Panda targets pass incrementally. Independent source review reports no important issue. Final committed factory replay is recorded in the companion evidence package; these results do not establish device/road acceptance.

Integrated child pin: `0e65f296260ec16da3c3b814c10ac5f36f8b2e4c`.
