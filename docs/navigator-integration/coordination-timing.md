# Navigator coordination pose publication timing

The opt-in coordination producer drains a dedicated motion/calibration subscriber
nonblocking immediately before publication. Each topic selects its newest
publication across the main and dedicated caches, then rebuilds calibrated pose,
source timestamp and validity together. The existing 100 ms consumer guard,
health/reversal checks, primary invalidity, ownership and reference withdrawal
remain intact. Stop/creep, default-off, steering and 1.20/0.97 are unchanged.

Baseline parent: fe08ede1af29a9c37b0c8b3550ce66cbd9392348.
Unchanged paired child: d2e8d0404143da0f2685c2ff4932b9b9f702951f.

Local evidence on October 23, 2026:

- 44 focused contracts; independent review passed.
- Minimal/full Mac and standalone Panda builds passed.
- Parent: 1,029 passed, 87 skipped, one expected failure, 281 passing subtests.
- Child: 4,485 passed, 1,690 skipped, 24,693 passing subtests, including MISRA checks.
- Frozen-source historical 1,048-segment / 73-route native regression passed,
  with 402 shared rejections and no added rejection, scope change or phase/counter
  mismatch. Production file hashes match that tested prototype exactly.
- Earlier 33-segment timing/native replay: one stale-pose fault becomes zero,
  without timestamp restamping, extended timeout or forced native permission.
- Latest 18-segment replay adds no native rejection. Its one release-timeout fault
  remains in both baseline and candidate; this pose fix does not remove it.

The latest fault at approximately 41.45 mph / 11:20:18 p.m. CDT has healthy
77.70 ms pose data. After a short brake/precharge pulse, received brake torque
remains 136–388 Nm across 52 distinct samples during the release wait, and
156 Nm at the fault. The existing one-second release guard disengages rather
than enabling propulsion without measured clearance. Neither pedal is pressed.
This is a separate coordination/actuator investigation; no release guard or
braking threshold is changed here. ECU torque is not independent physical
pressure/drag ground truth, and post-disengagement clearance is not proof of
what would happen with continued ownership.

Full provenance is retained in the local research records
`navigator-coordination-timing-prototype-2026-10-23` and
`navigator-timing-integration-2026-10-23` in the BluePilot research checkout.
No separate type checker, UBSan, ARM, device or road validation is claimed.
This local integration does not authorize installation or another Alpha trial.
