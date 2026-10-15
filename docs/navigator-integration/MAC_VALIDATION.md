# Mac build validation follow-up — 2026-10-14

The complete parent suite passes: **957 passed, 86 skipped, 1 expected failure, 343 passing subtests** in 170.44 seconds. Focused messaging: 222 passed; Athena/uploader/real UI startup: 38 passed.

Both `scons --minimal -j4` and full `scons -j4` completed successfully on this Apple Silicon Mac. Full development targets include Cabana and native tests. Generated acados, rednose and vehicle-model native targets also build. The real Comma 4 UI startup test passes with the actual fonts and images.

## Reproduction

Use Python 3.12 and the pinned submodules in this checkout. `verification-environment.txt` records the private environment; use upstream setup/lockfiles when preparing a supported target environment. Install Homebrew `llvm@20` (20.1.8 here), as required by pinned tinygrad's CI setup. The tinygrad loader discovers its keg automatically. No shell profile or model source workaround was needed.

Fetch Git LFS objects for `openpilot/selfdrive/modeld/models/dmonitoring_model.onnx`, `driving_supercombo.onnx`, `big_*_tinygrad.pkl`, and `openpilot/selfdrive/assets/**`. The model manifest records verified SHA256 and size against Git LFS pointers. The Mac build compiled the small models and warp with LLVM/Metal. Chestnut artifacts were integrity checked, not executed on Chestnut hardware.

Activate the checkout's environment, then run:

```sh
export PATH="$PWD/.venv/bin:$PATH"
export PYTHONPATH="$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/rednose_repo:$PWD/tinygrad_repo:$PWD/teleoprtc_repo:$PWD"
scons --minimal -j4
scons -j4
python -m pytest openpilot -q --maxfail=10
```

On this host native GPU access and temporary Params namespaces require running outside the restricted build sandbox. The parent suite is scoped to `openpilot`; blindly recursing into third-party submodule examples collects unrelated hardware/GPU tests. Paired opendbc has its own separately recorded full suite. The process-replay CLI helper is marked non-collectable by pytest; real replay evidence is recorded separately.

## Test portability changes

Only tests changed: a multiprocessing child creates its own messaging socket under macOS spawn; Athena uses a thread for a local publisher and a signaled socketpair for deterministic mocked WebSocket readiness; echo cleanup closes sockets without shutting down an already disconnected stream; uploader assertions use the existing uncached portable xattr reader; pytest no longer mistakes the parameterized replay CLI helper for a fixture-based test. Real message, proxy and attribute assertions remain intact. A separate source review found no important actionable issues.

## Limits

These are Mac build/test results, not an AGNOS 19.9 build or a device boot, firmware flash, Chestnut runtime, or road evaluation. The 1.20/0.97 steering implementation and child commit are unchanged. No Comma access, publication or installation occurred during this follow-up. Prior failed build/collection logs remain historical evidence and are superseded by the new validation logs.
