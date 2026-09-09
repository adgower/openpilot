# Shared Angle runtime checkpoint

The card adapter and host simulated transport share `navigator_a3_lifecycle.Lifecycle` through `RuntimeBridge`. No simulation-specific fault assignment remains in the audit. The audit substitutes only its in-memory publication boundary: real A2 CAN remains serialized and compared; only virtual A3 publications enter the synthetic observer.

Control intent, driver pause and source/measurement freshness arrive through `control_input`. Configuration and observed permissions/rejections arrive through the same bridge observer used by card. Once intent requests actuation, permission-loss tracking remains armed for this process session, including inactive intervals. There is no session-reset API; a neutral frame or returned frame does not clear faults. Explicit inactive intent is distinct from driver pause and does not erase session history.

A monitored A2 rejection remains evidence without rejecting untransmitted A3 shadow calculations. A rejection in the simulated A3 stream instead persists as a calculation fault and experimental disengagement intent, even if attribution among repeated identical packets is ambiguous. Returned packets establish neither acceptance nor vehicle execution. Configuration and permission faults continue to inhibit both calculation paths.

The lifecycle returns neutral/reset intent; the existing strategy/scheduler performs resets and preserves its existing reason codes, measurement band, per-update bounds and absent-proposal behavior. Freshness uses the same raw speed signal as the unchanged Ford strategy. `lifecycle.version=1` is additive JSON; transport envelope version 2 and controller version 3 remain readable. The legacy top-level transport `calculation_eligible` field describes transport/configuration eligibility; the nested lifecycle additionally reports driver/intent/freshness eligibility. Neither is a statement of MCU acceptance or a replacement for downstream strategy limits.

Production permission is independent. Requested A3 still forces truthful inhibition through card; command selection never publishes its experimental slot on real transport; unchanged production C rejects active Angle bytes. Lifecycle decisions cannot grant that permission. Synthetic monitoring in shadow is refused for live provenance. No environment setting selects simulated transport.

## Reproduction

From a matched parent and opendbc checkout with native msgq/cereal dependencies available:

```sh
python -m pytest openpilot/selfdrive/car/tests/test_navigator_a3_lifecycle.py openpilot/selfdrive/car/tests/test_navigator_a3_card.py openpilot/selfdrive/car/tests/test_navigator_a3_runtime.py openpilot/selfdrive/car/tests/test_navigator_a3_wiring.py tools/navigator_a3 tools/navigator_staging -q --import-mode=importlib
python -m tools.navigator_a3.command_path_audit --stock-repo /path/to/matched-opendbc --output /path/to/new-audit
python -m tools.navigator_staging.package --parent /path/to/parent --sources /path/to/local-sources.json --output /path/to/new-package
```

The delivery directory includes the exact host interpreter/PYTHONPATH and full child regression list in `run-verification.py`, plus compiled admission inputs, outcomes and source hashes. Results are synthetic software evidence, not Angle response measurements.

## Target build and activation boundary

This checkpoint changes Python only. No cereal schema, model, Ford strategy/scheduler, safety header or firmware source changes. A fresh target checkout still needs its matching generated/native libraries, Chestnut model artifacts and MCU artifact verified; macOS host libraries cannot be used as Linux target binaries.

The self-contained package derives pins from the selected Git tree. Use its `prepare.py`, `transfer.sh`, `build_only.sh` and README only in separately authorized device stages. Build-only requires a separate checkout under `/data/navigator-staging`, AGNOS 19.7, idle powered Chestnut and pinned uv/SCons dependencies. It does not call launcher/manager or flash. The full on-device build is unexecuted in this checkpoint.

Panda's pinned `SConscript` includes `opendbc.INCLUDE_PATH`, builds `panda_h7`, and signs via `board/crypto/sign.py`. Target verification requires `panda_h7.bin.signed`; default signing is the repository debug certificate. No new MCU artifact is claimed here. Startup through pandad may install matching firmware and therefore needs separate review/approval. Preserve matched source/configuration, OS and firmware rollback; a source checkout is not an OS/firmware rollback.

Before active Angle: independently supported limits on the decoded final command, validated EPS driver-handoff/first-command re-entry, and a matching production safety implementation are still required. BluePilot mapping remains an experimental working assumption. This package is not an Angle-enabled driving build.
