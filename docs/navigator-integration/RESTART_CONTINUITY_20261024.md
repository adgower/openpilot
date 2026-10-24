# Navigator restart continuity — October 24, 2026

A speed-only crossing of 3 m/s could discard measured brake-clearance evidence and withdraw a valid propulsion request, causing handoff to restart from zero near 6.7 mph. The observer now retains consecutive fresh clear measurements across that crossing during uninterrupted owner-enabled active PID operation. The existing handoff rate is unchanged.

Carry requires at least three distinct 0–4 Nm samples spanning 40 ms, bounded freshness/sample/callback gaps, and no braking, override, phase/owner interruption or invalid feedback. The original observer still executes and retains its latched faults and one-second release deadline. Owner 1.20/0.97, lateral/vehicle values, default-off coordination and native safety source are unchanged.

Local integration evidence is in the Mac workspace report `docs/research/navigator-restart-continuity-integration-2026-10-24/REPORT.md`: actual-source contracts, class/constructor equivalence, fresh 136,308-callback trace equivalence to the prototype, complete local builds/test suites, and independent review. The prototype's 47-segment native comparison is retained and bound to the equivalent implementation, not described as a fresh integration replay.

This fixes the command-state reset in saved-input evaluation. It does not establish physical smoothness or resolve the two release-timeout episodes and highway pulsing. ARM, parked and road validation are separate; this task does not push or change the Comma.
