# Navigator coasting and pose-expiry integration

The existing `FordPrefLongitudinalCoordination` opt-in now uses bounded weak brake-entry admission. It retains clear propulsion during qualification instead of resetting the handoff. The measured-clearance observer, fault deadline, stop/creep paths and meaningful-braking fallback remain unchanged.

The experimental sampled admission thresholds are a 0.01 m/s residual-demand integral and 0.3 seconds from the first observed negative sample. Discrete overshoot and packet latency mean these are not guaranteed physical speed-error or stopping bounds.

For the enabled Navigator/Alpha path, the parent creates an independent pose subscriber and calibrator. When the published pose exceeds 100 ms at consumption, a copied command may use a coherent, fresh measured snapshot. The implementation preserves producer identity and timestamp, validity, reference validity, lateral calibration snapshot and requested actuators. Healthy commands retain the prior path, apart from maintaining monotonic consumption after a previous refresh. A post-drain and post-compute clock is shared by tracking and `CI.apply`; source timestamps are never restamped.

The quiet-resume experiment is excluded. Resume packing, torque-clear threshold and release deadline remain unchanged. The four known saved release timeouts remain; this integration does not establish physical release or smoothness. Owner 1.20/0.97 and steering, geometry and native safety source are retained.

Local evidence is retained in `/Users/alex/Apps/bluepilot/docs/research/navigator-coast-pose-integration-2026-10-24`: four-drive actual-source equivalence across 247,765 callbacks; raw RX and original non-ACC TX native comparison; constructor, apply-clock, expiry and default-off tests; and local build/suite receipts. Past-only ACC substitution at actual send slots permits at most 40 ms after a reconstructed callback; the observed maximum is 21.81 ms. This is offline replay against recorded response, not physical validation or forwarding/heartbeat injection.

No new parameter default, OS/firmware update or device installation is part of this change. Publication, isolated device ARM validation and activation remain separate steps.
