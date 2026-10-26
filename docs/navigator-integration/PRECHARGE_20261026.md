# Bounded precharge paired integration — 2026-10-26

Local integration into `codex/navigator-tracking-release` in both repositories. The consumer uses a versioned request carrying the exact planner target consumed by LongControl, with its original timestamp. Missing, incompatible or stale allocator metadata falls back to nominal braking without creating a controls fault.

Only weak nominal brake admission can enter precharge-only: at most 0.01 signed demand-area and a 0.3 s decision window. Callback quantization can extend the held command until the next delivery. Strong braking stays nominal. Precharge and actual braking both count as actuation for the unchanged release observer; propulsion stays inactive during precharge. Rearming requires continuous positive intent and measured brake clearance. The precharge integral follows signed error; release/ramp retains existing antiwindup.

Early coasting is disabled. This is not the complete BluePilot allocator, a planner acceleration retune, or a claim to solve physical highway pulsing. Steering 1.20/0.97, lateral geometry, safety limits, and settings are preserved. No device access or publication in this task.

## Evidence

See the companion research package `navigator-precharge-integration-2026-10-26` for exact commands, logs and ledger. Minimal and full development Mac builds pass. Targeted lint and 26 initial focused tests pass. The 137-segment integrated replay covers 401,970 callbacks; all six route wire streams match the accepted bounded prototype exactly, with 1,879 precharge-only callbacks. Raw acceleration, gas, brake acceleration and actuation union match baseline exactly. The two existing reconstructed older-route release faults remain; no added faults. This frozen-response replay cannot establish physical comfort or ECU response.

The extracted production allocator also matches the prototype over 100,000 seeded randomized transitions. Protected lateral/geometry and safety files match their baseline byte-for-byte. A fresh whole-pair static review found no blocking defect; its phase-spelling test suggestion was applied.

Full suite, Panda and native replay completion results follow below. Mac/Panda build evidence is separate from an ARM device build, model startup, parked acceptance and road evaluation; none of those device checks are performed here.

## Completed local checks

- Minimal and full development Mac builds and Panda build: pass.
- Full parent suite: 1,065 passed, 87 skipped, 1 expected failure, 281 passing subtests.
- Full child suite: 4,603 passed, 1,690 skipped, 24,693 passing subtests.
- Fresh native debug replay: all 137 segments, 32,171,016 RX and 951,332 TX per variant, zero added rejections or relay faults; no forced permissions and no sanitizer claim.
- Integrated producer integral, raw command, freeze decisions and target/source match baseline on 789,224 updates. Nominal protocol fallback has exact baseline wire on all six routes.
- Precharge-only: 1,879 callbacks in 189 entries, 37.6565 seconds total. Held-command duration can exceed the 0.3-second decision window until the next callback.
- Targeted Ruff and both diff whitespace checks: pass. Whole-pair static review: no blocking finding.

The first parent run failed sandbox temporary-directory creation; the next exposed missing timestamp data in a synthetic fixture. Both are retained in the logs, and the corrected complete suite passes. No runtime assertion was removed.
