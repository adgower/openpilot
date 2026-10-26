# Bounded rolling command recovery — October 26, 2026

Local paired integration. Replaces the earlier uninstalled broad below-3 m/s integral guard with the selected narrow command-reduction and local-constraint anti-windup design. Existing coordination/Alpha prerequisites remain authoritative; no new toggle, schema, safety limit, pressure clearance, timeout, vehicle geometry or steering adjustment. Preserve 1.20/0.97.

opendbc owns qualified moving-restart admission, the 0.25 m/s² / three-second bound, fresh owner/sequence validation and signed local-constraint anti-windup. Openpilot retains PID/state mechanics and supplies live source-validity and FCW/forced-deceleration inputs. Reduction unwinds at at most 0.1 m/s³ (0.001/update) with fresh unlimited-drive feedback at 3–12 m/s. No artificial I repayment, baseline state copying or replay monkeypatch is used in production.

The integrated pair exactly matches the accepted offline prototype over all 197 saved segments / 570,269 callbacks and every producer update, with zero wire differences. One retained 12-segment route has five active prefix-input gaps and is diagnostic-only; 185 segments have no active atomic-input gap. This carries forward the candidate's zero added timeout/lost-activity/weaker strong-target request/new retained-pressure gas gates. It does not eliminate the two preexisting reference release-timeouts.

Native safety evidence is reused for the identical command stream and unchanged safety headers: all 197 segments, 570,178 replacement ACC slots, zero added rejections or relay faults, no forced permission. This is hash-bound reuse, not a new native run or hardware validation.

Named city route unchanged (17 brake transitions / 21.533 s). Named highway pulsing windows remain unchanged at 20/22/12 transitions. Across the corpus braking exposure increases 4.349 seconds; lower rolling requested peaks (~2.0→1.75) do not prove lower physical RPM, earlier shifts or comfort. Highway pulsing remains unresolved. Do not characterize this as highway smoothing or road readiness.

Full local Mac build and suite receipts are kept in the companion navigator-command-recovery-integration-2026-10-26 evidence package. ARM, model startup, parked checks, publication and device activation are separate; none is authorized by this local integration.

## Completed local checks

- Minimal and full development Mac builds: pass.
- Parent suite: 1,086 passed, 87 skipped, 1 expected failure, 281 passing subtests.
- Child suite: 4,686 passed, 1,690 skipped, 24,693 passing subtests.
- Panda build, targeted Ruff and both whitespace checks: pass.
- Whole-pair review: no substantive finding; test helper cleanup applied.
- Exact producer/wire parity: all 197 segments / 570,269 callbacks, zero differences.

Initial fixture failures are retained in evidence; corrected tests use real FCW/driver-monitoring message fields and exercise qualified recovery rather than the superseded broad guard. No runtime checks were relaxed.
