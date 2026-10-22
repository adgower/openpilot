# Earlier low-speed /70–75mph onset — local integration

The combined persistent-tracking, roll-aware and high-speed opt-ins now select EarlyLowHighOnset, subclassing the previously installed EarlierUnwindOnset policy. All vehicle flags and existing default-off preferences are retained; other opt-in combinations keep their prior classes.

At20–25mph, five independently fresh rising-demand observations can admit a bounded pre-clip term. It shares the0.00025rad·s ramp budget, caps at min(0.0015rad,5%feedforward), lasts at most0.25s, and fades out by25mph. Original atomic target consistency, current wheel/body deficit, predictive catch-up and original quiet-rearm rules remain required. No extra term is admitted once the existing deviation clamp binds. The severe clipped low-speed turn remains unresolved.

At70–75mph, validated withdrawing-demand history can credit existing onset persistence. Current body confirmation, wheel deadband/predictive closure, release/caps/rates remain in force for both blended branches. Roll mapping is not extended. Outside both bands, the independently advanced previous policy supplies the desired output, with final slew handling retained.

Owner factors1.20/0.97, geometry, native safety, calibration transport and settings registry are unchanged. No new parameter, limit widening, steering mode or parking command is introduced.

Accepted frozen prototype evidence:60recordings/878segments,5,063,641controller updates per variant; no added native rejection (five exact shared existing events).90focused tests,966transition cases,112active-abort cases and17,000shadow updates pass. These remain prototype evidence through normalized AST equality, not a fresh full integrated corpus run. Fresh integration selector/pipeline, target-drive replay, Mac builds/full parent and child suites are recorded in the companion local integration report at /Users/alex/Apps/bluepilot/docs/research/navigator-early-low-high-integration-2026-10-22/REPORT.md.

Local integration is separate from ARM build, device activation, parked acceptance and road validation. Frozen replay cannot predict changed vehicle response or establish85mph stability. No device operations were performed.
