# Navigator A3 staging package

This is source/build preparation, not completed target-device validation. A2 performs steering; the core path-angle experiment is shadow-only. Active Angle is independently blocked. No device action is performed by creating this package.

## Fixed configuration

Parent baseline18fd1a6505e072710514e723dae2edc65c0e3355; child01bea343a7f7d4e4f8a1947539abacb59ab8c278. The package manifest records the resulting parent commit and every submodule. Preserve Chestnut ONNX models, dual-resolution driver-monitor warp fix, fixed Expedition fingerprint, Navigator3.1115m wheelbase, and baseline mass. Do not transplant donor3.69m wheelbase, WMI V12, learned parameters, or a BluePilot firmware image.

`launch_openpilot.sh` uses the guarded staging entry only when `.staging-package.json` exists. Normal source checkouts retain their original entry. The staging guard requires AGNOS19.7, matching source/model hashes and complete local target outputs, then selects `shadow` and `expedition-provisional-v1`. This profile is provisional, not vehicle-validated. A2 startup for matched rollback is `tools/navigator_staging/launch_staging.sh a2`; neither entry selects boot or reboots. After exact artifact verification, the guarded entry creates the standard `prebuilt` marker so the normal launcher cannot rebuild after review. It refuses `.overlay_init` state rather than allow an overlay to replace the reviewed checkout.

## Installer/OS verification (8 September2026)

Official `release-chestnut` resolves to473eba53f280743d09c5bebc2d0613ea46883e02 and targetsAGNOS19.6. Its single-commit release history makes the huge behind-master count unsuitable as a count of missing patches. Official `nightly-chestnut`5d1dc8c826026927484cfeed2d3e32592f56917d also targets19.6. The proposed staging-chestnut URL did not resolve during this check.

Sources:
- https://github.com/commaai/openpilot/tree/release-chestnut
- https://github.com/commaai/openpilot/blob/473eba53f280743d09c5bebc2d0613ea46883e02/launch_env.sh
- https://github.com/commaai/openpilot/blob/5d1dc8c826026927484cfeed2d3e32592f56917d/launch_env.sh

Our frozen source requires19.7; last authorized device inspection showed18.5. Installing release-chestnut is not a matching-OS shortcut. Do not set AGNOS_VERSION=19.6 to silence this mismatch. A separately reviewed19.7 OS transition is required. OS updater implementation/manifest are in the pinned source; do not execute the launcher to perform that transition unintentionally. Installation endpoint/branch reachability does not prove device installation or hardware compatibility.

## Package and build stages

1. `package.py` creates complete-ancestry parent and submodule bundles with advertised exact pins. All Git bundles verify against an empty repository; no upstream fetch/push/merge is needed to materialize them. Package creation uses source Git objects, never a copy of a dirty working tree.
2. `transfer.sh PACKAGE comma@DEVICE_IP` copies files only into a newly created inbox. Requires separate transfer authorization. It never executes prepare/build/start remotely. Validate IP and SSH host key through the normal trusted SSH setup.
3. After inventory, backup and OS preparation, run `python3 INBOX/prepare.py --package INBOX --destination /data/navigator-staging/UNIQUE_NAME`. Only a new directory is accepted. Existing/shared/active checkouts are refused. The manifest stays in this private checkout.
4. After build authorization, run `bash CHECKOUT/tools/navigator_staging/build_only.sh CHECKOUT`. It requires Linuxaarch64/AGNOS19.7/offroad, creates a private uv environment/cache and calls SCons directly. No launch, manager, Panda instance, flash or reboot. Model compilation DOES use QCOM/Chestnut compute and therefore requires powered, idle hardware; it is not a read-only operation.
5. Review `.staging-results/scons.log`, dependency/toolchain record, and `build-result.json`. Only a successful full build followed by required model/native/H7 artifact verification creates target_built=true. Missing Chestnut model compilation is a failure even if SCons itself returned success.
6. Boot selection and startup are separate, authorized deployment actions, conditional on actual service/boot-path inventory. If the real service bypasses launch_openpilot.sh, it must explicitly be configured to use the guarded entry; do not claim a guard is enforced until that path is verified.

`uv sync --frozen --extra tools --python 3.12` uses the committed uv.lock and local submodule packages. No system pip installs or AGNOS changes are made. The frozen acados dependency is comma-deps-acados0.2.2.post98 with a manylinux_2_28_aarch64 wheel already listed in the lock. Availability/install is checked during the separately authorized build; this package does not claim that package download or target compilation has occurred. Missing uv, system headers, platform libraries, compiler support or wheels remain explicit failures. A private SCons cache uses NAVIGATOR_STAGING_SCONS_CACHE; absent that environment variable stock cache behavior is unchanged.

## Separate artifact responsibilities

Python: parent+all matching submodules and isolated dependency environment. Native/schema: full SCons rebuild and generated CarOutput.navigatorA3 consumers, msgq, params, camera/control libraries. Models: frozen ONNX hashes, both DM warps and DM metadata, QCOM small model and Chestnut big-model chunk manifests/files. Panda MCU: H7 image built from frozen Panda plus THIS opendbc include path. Default signing is the repository debug certificate; not a release key or approval to install. Native/device binaries and model programs from the earlier macOS build are not copied as Linux output.

Startup may install matching Panda firmware through pandad and may perform Chestnut initialization. Thus starting manager is a deployment action even with A2 steering. The compiler's success is not firmware installation, measured vehicle validation or a production-readiness claim.

## Matched rollback requirements

Before any OS/install/start change, preserve a fresh verified backup of CURRENT BluePilot source including dirty/untracked files, parameters/learned state, models, boot service/entry path, OS/version and observed firmware identity. The older A2 backup is a separate restore point, not a current BluePilot rollback. Archive source/configuration hashes and local firmware artifacts; mark flashed firmware identity unknown if it cannot be established read-only.

- Returning to BluePilot: restore its matched18.5 OS/source/config/models and separately reviewed firmware requirements. Switching only a Git branch from19.7 is not a verified rollback.
- Returning to staged A2: use this same19.7 source/schema/model/firmware set with startup modea2, restoring its saved configuration as needed. Never indiscriminately overlay old parameters onto another fork.
- Returning to original A2/Chestnut: use the verified pre-BluePilot A2 backup plus its matched19.7 OS/model/firmware requirements. The backup is not itself an AGNOS image or evidence of installed MCU firmware.

Do not automate an OS rollback, flash, reboot, param restore or boot-path rename before these exact current identities and actions are reviewed. Preserve both checkouts so an application build failure does not destroy the working install.

## Remaining gates

The local package, tests and host firmware evidence can be completed without device access. A verified ON-DEVICE staging build cannot be claimed until the separately authorized transfer,19.7 preparation and build succeed. The current work stops at that authorization boundary with exact executable preparation commands and named remaining target checks. Active Angle physical enforcement, driver handoff and physical response remain separate from shadow staging.
