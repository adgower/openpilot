# A3 build provenance and rollback

This is an offline preparation record. No device was contacted, started, flashed or rebooted. A host safety shared library is not MCU firmware. The core path-angle experiment remains disabled by default and is not approved for physical testing.

## Frozen build graph

Parent `e855605fdf1b2ba480185d9473fa544b8ee4b99c` root `SConstruct` adds `opendbc_repo` to Python search paths and native include paths, builds cereal, invokes `panda/SConscript` at line272, then native processes including pandad, locationd and modeld. The A2 driver-monitor warp fix is already in this parent and is retained.

Panda revision `75aa44bec9140849868239b1f1e3f22624adb8fe` was materialized as an exact detached Git worktree in isolated `/private/tmp/navigator-a3-panda-pinned` for source inspection. Its `SConscript` imports Python `opendbc` and uses **`opendbc.INCLUDE_PATH`** for MCU compilation. This must resolve to the candidate opendbc checkout, not an unrelated installed package. Checking only the parent gitlink or root compiler include list is insufficient.

The Panda project uses `arm-none-eabi-gcc`, `arm-none-eabi-objcopy`, and `arm-none-eabi-objdump`; STM32H725 Cortex-M7, Thumb, hard float, `fpv5-d16`, GNU11 and `-Werror`. Application starts at `0x08020000`. It compiles `board/main.c`, signs the application with `board/crypto/sign.py`, and produces:

- `panda/board/obj/panda_h7.bin.signed`: applicable H7 application.
- `panda/board/obj/bootstub.panda_h7.bin`: separate bootstub; not a substitute for the application.
- Jungle/body targets are separate projects and must not be selected for this vehicle.

Absent `RELEASE`, the build uses `board/certs/debug` and `ALLOW_DEBUG`. `RELEASE` requires a real certificate through `CERT`; no release signature is claimed. The generated version string alone does not capture dirty safety changes: retain source commits/patch hashes and compiler provenance alongside SHA256 and the firmware's embedded signature.

Pinned `panda/python/constants.py` defines `FW_PATH` from the loaded Panda module directory plus `board/obj`, and `McuType.H7.config.app_fn = panda_h7.bin.signed`. Therefore the expected file for the intended checkout is `/data/openpilot/panda/board/obj/panda_h7.bin.signed`, conditional on that being the actual loaded Panda package. No device package path/hardware identity was reverified in this offline task. The source supports H7, but physical identity must still be confirmed before any future installation.

## What needs rebuilding

| Change | Required treatment |
|---|---|
| Pure Python strategy, tests, offline report | Correct Python environment/imports; no MCU rebuild solely for Python |
| Cereal schemas, generated bindings or native telemetry consumers, if changed | Rebuild matching generated/native targets and dependent consumers; stale binaries cannot read an assumed new schema |
| `opendbc/safety/modes/ford.h` or included safety helpers | Recompile actual H7 application using candidate opendbc include path; sign and identify that artifact |
| Host `libsafety` audit | Compile host C implementation for tests; not the MCU application and never a flash artifact |
| Retained DM-warp fix | Existing root build includes required DM artifacts; no A3 requirement to change/retrain driving models or reinstall AGNOS |

See the final changed-file manifest for whether a schema change actually occurred; do not rebuild or claim a schema migration merely because diagnostics were added.

## Actual toolchain checks and local commands

The source audit ran these checks on the Mac:

```sh
/private/tmp/navigator-diagnostics-venv/bin/python -m SCons --version
arm-none-eabi-gcc --version
```

Results: `No module named SCons` (exit1) and `command not found: arm-none-eabi-gcc` (exit127). No MCU artifact was produced by this audit; consequently there is no A3 MCU SHA256/signature to report. This is a concrete missing-toolchain blocker, not a passing firmware build. The separate safety agent's compiled host results are recorded in the safety/test reports.

When a suitable isolated SCons + compilation_db + Panda signing-dependency environment and ARM cross-toolchain exist, the source-defined standalone target is:

```sh
cd /private/tmp/navigator-a3-panda-pinned
PYTHONPATH=/private/tmp/navigator-a3-opendbc /path/to/build-env/bin/python -c 'import opendbc; print(opendbc.INCLUDE_PATH)'
PYTHONPATH=/private/tmp/navigator-a3-opendbc /path/to/build-env/bin/scons -j4 board/obj/panda_h7.bin.signed
shasum -a 256 board/obj/panda_h7.bin.signed
```

These are future build commands, not successful commands from this audit. The first output must be exactly the candidate opendbc root. The detached Panda worktree HEAD was verified at `75aa44bec9140849868239b1f1e3f22624adb8fe`. Record `git rev-parse HEAD` and dirty state again when building; safety inputs are additionally identified through the candidate opendbc manifest.

For a fully materialized parent with all pinned submodules and its supported native development environment, root `scons -j4` is the normal source build. The frozen manager build wrapper attempts `scons`, retries `-j4`, then `-j1`; it is not an offline audit entry point because it changes hardware power state. Root build requires its native dependencies (Cap'n Proto, model/native toolchains and all pinned submodules); the isolated preparation parent does not itself prove these dependencies are installed. A Mac host compile does not verify comma-device native executables. No universal “reboot and rebuild” command is prescribed.

To extract a built application's expected signature without opening USB, use the pinned `Panda.get_signature_from_firmware(path)` method in a prepared import environment; never instantiate `Panda()` merely to read a file. Retain its result with the file hash and exact source/patch manifest.

## Startup can install firmware

`launch_chffrplus.sh` may swap in a finalized staged update before launch. It establishes package symlinks, then skips `build.py` if the root `prebuilt` marker exists. Source installs normally call the build wrapper; prebuilt installs do not. This offline task did not inspect current device markers or staged trees. Do not remove `prebuilt` blindly or assume modified safety C is recompiled on restart.

`openpilot/selfdrive/pandad/pandad.py` reads the expected H7 signature from `FW_PATH`, resets/recovers internal Panda during startup and can flash on mismatch/bootstub. It can also recover DFU devices and, if firmware will not boot, attempt bootstub recovery. Therefore merely starting manager/pandad is not a harmless inspection and is outside this task. The root launcher can also perform AGNOS update actions; it must not be used as an offline build-only shortcut.

## Future reviewed staging, not authorization

Deployment is blocked until independent final-command constraints and control assumptions have been reviewed, host tests pass, matched MCU artifacts exist, and the owner separately authorizes installation. Review core driver handoff separately from omitted automatic recovery pulses; no “Angle fixes recovery” claim follows from these tests.

An approved operation must first establish the actual boot source, parent/submodule revisions, dirty patches, package import paths, startup selectors, `prebuilt`/staged-update state, MCU identity and existing firmware signature. Preserve exact copies/hashes of source changes and current firmware files before replacement. Preserve CarParams and persistent learning/calibration keys as a private backup; do not reset or substitute them as part of installing this experiment.

Stage parent and opendbc from the manifest without fetching an unpublished dangling gitlink. Verify the candidate remains off by default and the A2 wheelbase selector remains present. Build native/schema consumers if changed, then compile the H7 application against the correct opendbc source. Record binary hash and embedded signature. Review both Python selection and firmware safety-mode/profile agreement before authorizing any manager startup.

Any later approved startup must be parked/offroad with stable power and an independently recoverable matched A2 package. Confirm actual loaded source, recorded runtime CarParams, selected strategy/profile, installed firmware signature, active safety configuration, and available safety rejection/mismatch counters before any driving. A host safety test or matching file on disk is not proof of installed firmware. No active maneuver or road comparison is authorized by this document.

## Matched rollback

Disabling a Python selector restores only host selection; it does not prove stock safety firmware is installed. A complete approved rollback restores the captured A2 parent/submodule source or patches, fixed fingerprint, A2 startup selector and matching A2 H7 firmware file. Preserve unrelated dirty state. Review startup's automatic flashing implications before restarting. Verify the installed signature against the retained A2 artifact and confirm runtime CarParams return to the A2 values and safety configuration.

Keep learning and calibration intact by default. If a separately approved recovery needs persistent-state restoration, use the captured device backup with provenance rather than values reconstructed from old logs. No learning reset commands, destructive checkout commands, USB flash commands or automatic deployment script are included here.
