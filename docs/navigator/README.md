# Core path-angle experiment: offline handoff

This implements the feasible offline experiment on frozen A2-WB. **It is not an installable Angle controller.** The strategy and independent decoded-wire evaluator are deliberately unreachable from production control and firmware. The physical command-to-motion hypothesis is unvalidated; enabling a selector cannot authorize it. No device access, deployment, push or merge occurred.

The source baseline retains the driver-monitor fix, fixed Expedition fingerprint and A2 wheelbase selector. Existing parent and A2 child source files were byte-checked, including expanded LFS content: 1,194 parent files and 448 child files matched, except the exact previously deployed A2 startup selector. The final parent gitlink records the separately committed child; the manifest and incremental bundles preserve both repositories.

## Read the evidence

- [Source, dependencies and retained/changed/omitted behavior](a3-source-and-dependency-audit.md)
- [Compiled safety admission review](a3-safety-review.md)
- [Build, staging boundary and matched rollback](a3-build-and-rollback.md)
- Durable local artifact: `/Users/alex/.codex/visualizations/2026/09/05/01a07385-2ba0-7b50-8447-0f8152e58e2c/navigator-a3/index.html`

The artifact directory contains source/configuration manifests, test output, four compiled safety variants (each plain/instrumented), exact machine-readable admission cases, and native-timestamp diagnostics. Offline synthetic commands are compared with **old recorded A2 motion**. Accepted commands in the host hypothesis evaluator are not evidence that A3 improved handling or is safe to deploy.

The core changes driver handoff to immediate inactive output while pressed, rather than the donor's sustained takeover detector. Its neutral interval and zero-based re-entry remain control questions for review. Automatic recovery pulses are omitted; their trigger/timing is documented, not experimentally validated. This is not a complete BluePilot Angle or post-override recovery comparison.

## Reproduce locally

Current worktrees are `/private/tmp/navigator-a3` (parent) and `/private/tmp/navigator-a3-opendbc` (child), both on local `codex/navigator-a3-core`. The child is a separate worktree; it is not materialized inside the parent's `opendbc_repo` directory. Explicit import paths below are intentional. Do not start the manager, launcher or pandad to run these offline tools.

The existing Python 3.12 diagnostic environment supplies pytest, numpy, pycapnp and the log-reader dependencies. CFFI for the frozen safety suite is available in the bundled local Python 3.12 package directory. No downloaded dependency was installed for that suite. Record dependencies from `verification/environment.txt` before rebuilding elsewhere.

```sh
cd /private/tmp/navigator-a3-opendbc
export PYTHONPATH=/private/tmp/navigator-a3-opendbc:/private/tmp/navigator-a3:/Users/alex/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/lib/python3.12/site-packages
/private/tmp/navigator-diagnostics-venv/bin/python -m pytest -q opendbc/car/ford/tests/test_navigator_a2.py opendbc/car/ford/tests/test_navigator_a3.py opendbc/car/ford/tests/test_ford.py opendbc/safety/tests/test_ford.py /private/tmp/navigator-a3/tools/navigator_a3/safety_audit_test.py /private/tmp/navigator-a3/tools/navigator_a3/test_core_replay.py

cd /private/tmp/navigator-a3
/private/tmp/navigator-diagnostics-venv/bin/python tools/navigator_a3/safety_audit.py --donor-repo /Users/alex/Apps/bluepilot --stock-repo /private/tmp/navigator-a3-opendbc --output /private/tmp/navigator-a3-safety-repeat
/private/tmp/navigator-diagnostics-venv/bin/python tools/navigator_a3/core_replay.py --help
/private/tmp/navigator-diagnostics-venv/bin/python tools/navigator_a3/raw_admission.py --help
```

The raw-log reader additionally uses the already prepared frozen diagnostic checkout `/private/tmp/navigator-lateral-diagnostics` for `openpilot.tools.lib.logreader`. The replay manifest records actual input paths/hashes and output coverage. Curve events include their supplied approach segments. Maneuver windows overlap, so full command plots cover all windows while raw admission uses one explicitly selected representative A2 run, avoiding duplicate synthetic transmissions on one timeline.

Incremental parent and child Git bundles require the exact prerequisite commits listed by `git bundle verify` and in `manifest.json`; they are not standalone clones of all dependencies. Matching format-patches are also included. Preserve existing working trees. For a fresh **local** restoration, fetch each bundle into its corresponding repository under a new unused ref, then create isolated worktrees at the recorded heads. No push, remote merge, submodule update or deployment is implied. The child bundle includes the saved A2 patch commit as well as A3 additions. Other frozen submodules remain at the parent pins and must be separately available for a full native build.

## Review boundary

Physical path-angle mapping and gain selection, driver handoff and inactive availability, measurement alignment, profile agreement, timing/counter policy, and optional recovery remain unresolved. The actual decoded-wire evaluator has independent software limits, but its inverse-mapping assumption is not a validated vehicle model. No production safety registration or firmware enablement was added. The full native build and MCU build are not passing results; missing ARM/SCons tooling and absent production integration are explicit blockers.

There are no new schemas or native telemetry bindings. Python tools and host C test libraries are the actual additions. Future firmware integration, native/schema rebuilds if subsequently needed, installation and matched rollback require a separate review and approval. The current A2 vehicle remains the comparison baseline.
