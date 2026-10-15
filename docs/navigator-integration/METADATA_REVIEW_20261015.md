Historical metadata-only checkpoint; superseded by [paired budgeted integration](BUDGETED_TRACKING_20261015.md).

# Local metadata integration and device baseline

2026-10-15. This is a partial completion of the installation request: metadata scheduling is integrated as an uncommitted local opendbc edit; the experimental rising-demand/turn-exit steering correction is not integrated into the runtime or installed. No source commits, submodule pin update, publication, device installation, restart, parameter write, firmware update or CAN/ECU query was performed.

## Local state

Paired branch is `codex/navigator-tracking-release`, parent74f66e7a0bf1dc0718431532948ece2c4fb0041c / child1b965caadd4a6aea4d09985532bf7cca6c0873ca. The child changes only `opendbc/car/ford/carcontroller.py`: advertise truthful current shadow intent immediately before an active Expedition/pinion angle request. Existing neutral/inactive/human-turn scheduling and all actuator packet construction are unchanged.1.20/0.97, source geometry, runtime trial selector and native safety implementation are untouched.

The parent receives this review note only. Its opendbc pin still identifies the unchanged child commit, so this is **not a completed paired release**. Local modifications are retained for review, without deployment instructions.

## Fresh checks

21 focused actual-controller tests pass, including active timing, actuator parity, inactive/human-turn phases and pinion-disabled behavior. Regression first failed3 tests against the previous controller. Native Ford suite:213 passed,79 skipped,12,063 subtests passed. Diff whitespace and selected Python error checks pass.

Full six-drive/89-segment replay of the actual metadata-only source:20,227,501 physical RX frames;322,306 configured TX accepted,zero rejected, versus one baseline rejection at28.44mph. Non-0x3ca messages are asserted identical at **every update and speed**, not just above25mph. Default-OFF tracking equals the feedforward reference with the same metadata fix. The installed release-aware correction is used in both baseline and candidate; the newer rising-demand limiter is not involved. These results do not establish hardware bus-timing compatibility or safe road behavior.

See `summary.json`, `full-replay.json`, `green.txt`, `ford-tests.txt`, `metadata-review.patch`, and `REVIEW.md` in the corresponding BluePilot report folder.

## Device baseline

Read-only SSH connected to the expected installed parent revision. Fifteen seconds of passive subscriptions show offroad, ignition false, noOutput safety, controls disallowed, no RX-invalid/heartbeat/fault indications, no unexpectedly stopped processes.151 Panda samples retained a historical TX-blocked counter of6 without increase; this is not a zero-counter result. Device source still uses1.20/0.97.

No carState/carControl topics are present offroad, so powered stationary acceptance is **not verified**. These are baseline checks of the existing installation, not post-install checks. The requested experimental steering deployment was declined because current offline evidence does not establish safe vehicle response.

## Verified backup

Private Mac backup: `/Users/alex/CommaBackups/20261015T062221Z-readonly-pre-metadata-review/`. Parent and child tracked-source archives plus full parameter-directory snapshot;936,435,888 bytes in the original five inventoried files. Both device source trees are clean at the existing paired revisions before and after capture; selected settings unchanged. Local hashes, archive structure, selected parameter values and archived Ford source hashes verified. Passive sample receipt is stored privately with the backup. This is not a full recovery image: OS, git history, model weights, recordings and untracked artifacts are excluded.
