# Navigator: paired budgeted tracking integration

Local integration only, 2026-10-15. No Comma connection, source/parameter writes, reboot, OS/firmware changes or vehicle queries during this task. Existing device settings and installation are untouched. No remote publication.

## Implementation

The existing `FordPrefPersistentTracking` opt-in selects `BudgetedTrackingPrototype` through a pure factory in the parent context module. Missing/OFF preference retains the existing feedforward controller. Required Expedition/pinion/low-speed flags remain unchanged; no new parameter or migration is introduced. The child ports the previous rising-demand/fractional-recovery guards into a repository module and adds the command budget. Owner coefficients1.20/0.97, geometry, pinion model, native safety C and original range/slew limits are byte-identical to the previous paired child commit.

Current truthful shadow-curvature intent is emitted before active Expedition/pinion angle requests, fixing the stale-marker28.44mph rejection. Metadata traffic intentionally changes; recorded actuation packets at≥25mph and≥45mph remain identical. Inactive/human-turn ordering is preserved.

The rising-episode budget is now **.00025rad*s before limiting**, reduced from.001 after evaluating final-command tails. Once a rising episode starts, its spend remains latched across steady demand, reversal, driver intervention, inactive/human override and invalid context. It rearms only after1continuous second of valid fresh/cadence-qualified near-zero demand and measured curvature (both≤.0005), measured range≤.0001 and adjacent slope≤.0002/s. Invalidity or override breaks the dwell. Prolonged quiet rearms only once. Pure steady episodes retain the existing release-aware correction; this budget is not a cap on every feedback correction or an entire drive. In practice this conservative latch may withhold new ramp assistance through quick zigzags until a settled straight interval occurs.

## Final-command residual evidence

Each hypothetical candidate uses the actual update path, including unchanged range/slew limits, with its own feedback plant. A separate feedforward reference receives the same candidate inputs and keeps independent slew history. Both final outputs are compared through the full trace, including periods after the extra term reaches zero. This measures counterfactual command residual, not identified physical vehicle response.

The screen includes300 changing-demand cases,2 saturation cases and10 forced abort cases across both signs. Forced events use elapsed frames from first correction and explicitly assert that every abort occurred; cases after a pulse has already ended are labeled separately. Both-sign maximum final difference area falls from **.0019235235rad*s** with the old.001 budget to **.0009797761rad*s** with the selected.00025 budget (~49.1% lower). The selected maximum post-last-term tail area is.0007404988rad*s. All312 trace ends settle to zero command difference. Original96-step and210 stress screens are retained separately; these are assumptions, not road improvement percentages.

The final area can exceed the prelimit budget because slew histories differ. No universal final-area bound or physical-response guarantee is claimed. Abrupt saturation scenarios demonstrate this rather than hiding it. The new budget makes the tested transient smaller while retaining earlier onset; it also reduces the amount/duration of new assistance.

Saved weak-turn reconstruction preserves first extra command at888.7800s vs installed889.7273s,0.9473s earlier. This uses actual logged steering-send timing. The full generated-controller replay uses a different fixed update phase and does not establish that same onset: its rising-budget latch is not exercised in most recorded drives (no qualifying rearm events). Behavioral tests and dedicated transient cases therefore supply the rearming/budget coverage; the recorded replay primarily checks protocol acceptance and unchanged high-speed behavior.

## Verification

- Full six-drive/89-segment native replay:322,306 configured TX accepted,zero rejected; baseline reproduces one rejection.20,227,501 physical RX frames,89 compressed input hashes verified. No controls-permission override or safety-limit change. DefaultOFF equals the reference with the same metadata fix. Each drive ends with zero final-vs-feedforward residual; totals include legacy steady assistance and are not per-episode budget numbers. Generated cadence gaps are enumerated in summary.json.
- Repository Ford/safety suite:268 passed,79 skipped,12,063 subtests passed. Includes portable inherited/ramp guards, marker timing, budget rearming and anonymous turn fixture.
- Full parent suite:970 passed,86 skipped,1 expected failure,343 subtests passed. A historical startup assertion expected the earlier LowSpeed class and missed the already-installed extra-low-speed factor; updated it to current behavior and exercised actual saved opt-in selection through the constructor seam. No runtime gain changed to satisfy that assertion.
- Additional focused report suite:57 passed. Mac full development build `scons -j4` passed. These are local Mac results, not an AGNOS/device build, Chestnut runtime, powered parked acceptance or road validation.

See source and commit receipt, summary.json, provenance.json, input-manifest.json, verification.txt, full-replay.json, final-command-screen.json, comparison-budget-001/final-command-screen.json, focused-tests-final.txt, ford-tests-final.txt, parent-suite-final.txt and build logs. Intermediate failed/old-budget logs are historical and are superseded by the named final results.

## Remaining validation

Hardware metadata cadence/bus behavior, target build/boot, powered stationary acceptance and actual closed-loop vehicle response remain unverified. The prior fresh private Mac backup remains available; no new backup was needed because this task does not contact or change the device. Local branch integration is not evidence those steps passed.
