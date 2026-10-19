# Bounded sustained tracking and unwind integration

This paired change adds `SustainedErrorUnwind` under the existing Expedition/pinion/low-speed, tracking, roll-mapping and high-speed preference prerequisites. Other factory combinations retain their previous classes. No new parameter or UI control is introduced.

The low-speed policy adds a separate sustained-error allowance of at most 0.0025 rad, 0.0025 rad-seconds and one second, inside the existing total cap and fade to zero by 25 mph. It requires fresh calibrated target, wheel and body deficit evidence; recovery or invalid/driver evidence withdraws admission. A separate continuous valid quiet dwell is required to refill.

Calibrated wheel-target reduction extends through 60–70 mph. The previous installed wheel policy is advanced independently on every update; final output fades back to it from 70 to 75 mph. Native slew remains enforced, with an explicit pending handoff if needed. This resolves stored-history differences above 75 mph without changing native limits.

The previous wheel policy and all owner 1.20/0.97, geometry, pinion, safety, parameter registry, producer/card and device settings snapshot sources are retained. Actual-output diagnostics distinguish the returned command from hidden extension state; `actual_release_pending` uses outer handoff plus the selected branch release state, rather than a union of both hidden branches during blending.

Accepted research evidence: 122 focused tests; 480 transition cases / 172,800 frames; 43 routes / 740 segments / 4,289,417 updates per variant; zero additional native rejects, zero >=75 mph wire differences and default-off / protected 25–60 mph parity. Exact 75.111 mph generated frame now equals the installed reference. Saved original-timing 15/64 mph prototype behavior is preserved.

Fresh production port/factory equivalence, full configured Mac builds and parent/child suites are recorded separately in the BluePilot workspace's `docs/research/navigator-paired-sustained-build-2026-10-19` package. Report-local replay proof is carried only through verified class and factory equivalence; it is not a fresh integrated road measurement. Local build/native evidence does not establish ARM/AGNOS startup, physical bus delivery, closed-loop stability or physical tracking improvement. No push, device access or installation is part of this integration.

Configured local validation: 134 focused tests and scoped Ruff pass; actual factory equals accepted prototype across 30,240 updates; 480 transition cases / 172,800 frames pass. Full parent suite: 1,018 passed / 343 subtests, 86 skips and one expected failure. Full child suite: 4,112 passed / 24,673 subtests, 1,690 skips. Minimal/development Mac SCons and Panda incremental builds pass. Independent integration review found no material issue. These are configured local checks, not clean-room or ARM/device acceptance.

Integrated child pin: `4167433e5c575f620a9dedf8144a0bc7cba14291`.
