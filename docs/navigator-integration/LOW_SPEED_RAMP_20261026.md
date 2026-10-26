# Low-speed ramp anti-windup integration — 2026-10-26

Local candidate only; not published or installed.

The public Ford integral-freeze policy now holds positive unattainable PID demand below 3 m/s only with fresh, valid, current-owner version-1 engineRamp feedback. Existing brake clearance, signed recovery, precharge, higher-speed behavior, 1.20/0.97 and lateral calibration remain unchanged. No post-ramp settling policy was included.

The integrated replay exactly matches the selected prototype across all 8 routes / 162 segments, 472,261 controller callbacks and every producer trace. Safety header hashes are unchanged, so the selected candidate's native replay applies to the identical command stream: 37,796,591 RX and 1,115,518 TX per variant, no added safety rejection or relay fault, without forced permission. This reuses native evidence; it is not a new native replay or hardware validation. Existing release-timeout failures remain represented.

Owner accepted the original 27 encoded total-brake requests one 0.0039 m/s² increment less negative (253 raw samples, maximum raw delta 0.000511467456817627); gas/brake bits unchanged there. Exact command equality confirms no additional variation relative to that selected candidate.

Focused tests passed (52), both Mac minimal and full builds passed, and parent suite passed 1,072 tests (87 skipped, 1 xfailed, 281 subtests). Targeted Ruff and independent whole-pair review passed. Child suite and Panda build results are recorded separately in the build ledger.

This reduces simulated integral buildup during a limited low-speed propulsion ramp. Physical gear/RPM or comfort improvement remains unproven. The separate highway/downhill pulsing remains open. ARM build, device checks and activation have not been performed for this candidate.

Final local child suite: 4,644 passed, 1,690 skipped, 24,693 subtests passed. Panda build passed. No ARM or device testing performed.
