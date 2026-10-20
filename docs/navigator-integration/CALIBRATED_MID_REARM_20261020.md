# Calibrated tracking, mid-speed episodes and resolution-aware rearm

The existing tracking + roll-mapping + high-speed opt-in factory now selects `CalibratedMidUnwind`. Existing vehicle/pinion/low-speed prerequisites remain mandatory; no new preference, parameter default or producer is introduced. Other factory branches are unchanged.

The policy ports the accepted combined application-frame calibrated feedback, bounded25–30mph allowance, once-faded60–70mph wheel-unwind feedback and resolution-aware neutral rearming. `SustainedErrorUnwind` and all earlier policy modules remain intact as independent references. Owner1.20/0.97, geometry, native pinion/shadow/deviation/range/slew limits and device settings are preserved.

The mid episode retains0.0025rad/five-percent path,0.0015rad-second and0.75s bounds. It requires agreed calibrated wheel/body deficit, fresh distinct evidence and native deviation headroom. Rearming requires one second of continuously valid near-neutral request/wheel/body evidence; bounded wheel resolution/body-noise tolerances and median drift checks avoid false rejection of settled input. Invalid/nonfinite/stale/duplicate/nonneutral evidence cannot refill it. The lifetime exposure counter retains spent effort across rearms.

Accepted report-local proof:46recordings/766segments/4,432,831generated-controller updates per compared variant, zero additional native rejections (two shared inactive/not-allowed0x83 rejects),91focused tests,714transition cases/228,480frames and128active-abort cases. Default-off generated output and recorded30–60mph and>=75mph output equal the prior policy. Saved26mph turn rearms and receives seven bounded packed corrections before catch-up withdrawal; later native clipping remains. Physical tracking improvement is not established.

Integration source/helper AST equivalence and actual-factory command equivalence carry the frozen replay evidence. This task does not rerun the full integrated766segment corpus; new-drive observational analysis is separate and uses the currently installed earlier revisions. Full configured Mac builds and parent/child suites are documented in the companion BluePilot evidence package `docs/research/navigator-paired-calibrated-rearm-build-2026-10-20`. These incremental local builds are not ARM/AGNOS boot, bus delivery, closed-loop stability or road proof. No push, install, settings write or firmware update is part of this integration.

Integrated child pin: `de70d4f945b2bb1472e5207b92d2276137d8c5a1`.
