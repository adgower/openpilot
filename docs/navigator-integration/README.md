[Current body-confirmed high-speed integration and validation](HIGH_SPEED_20261018.md).

[Current atomic calibration / roll-mapping integration and validation](ROLL_MAPPING_20261017.md).

[Current bounded wait/catch-up revision and limits](WAIT_CATCHUP_20261015.md). The baseline snapshot and historical validation below describe earlier integration stages.

# Navigator master integration

This personal fork ports the installed Navigator steering trial onto public openpilot master snapshot `6eb63e045469a582f4c7b406705543e891113606` and its mass-normalized opendbc base `b3bd79a659dae961af169a3baac723961f720133`. The local branch is `codex/navigator-master-integration` in both repositories. It has not been published or installed on the Comma.

## Preserved behavior

- Deliberately forced `FORD_EXPEDITION_MK4` identity for this Navigator; this is not native Navigator recognition or an upstream Expedition correction.
- Navigator SWB wheelbase 3.1115 m and steering ratio 19.6; equivalent normalized vehicle dynamics replace the obsolete mass fields.
- Ford angle control, engagement ordering and owner **1.20/0.97** high-curvature factors.
- Optional paired pinion feedback, with fixed Python/Panda geometry and invalid/missing/stale signal protection.
- Optional low-speed trial: gentle ×1.15 and tight ×1.05, broad smooth curvature blend and 30–45 mph fade. Existing steering bounds and driver override remain unchanged.
- Offroad-only controls in standard and Comma 4 MICI settings; both custom parameters default off in new storage. Existing preferences must be deliberately restored during a later installation.

The internal CarParams option bits moved to avoid upstream `HAS_BSM=2`: pinion is 4, low-speed trial is 8. These are not portable saved settings; startup regenerates them from the persistent preferences. Panda pinion safety parameter remains 8, combined with CANFD as 10.

## Upstream changes retained

AGNOS 19.9, current model/Chestnut requirements and automatic Ford checksum generation remain at the pinned master versions. The old AGNOS 19.7 pin, prebuilt marker, staging symlink and any device-only build hacks were not copied. The child pinion guard uses normalized stiffness fields; its Panda model retains the equivalent fixed formula. Angle metadata conversions were made MISRA compliant with exhaustive signed-value equivalence verification.

## Verification and limits

The full child suite passes: **3746 tests**, **1690 skips**, **24673 passing subtests**, including normal MISRA mutation checks. The build uses an unsanitized native debug safety library on macOS; no sanitizer result is claimed. Targeted parent UI/Params tests and all four real Params-to-Ford startup option combinations pass. Matching H7 firmware and native Params/messaging targets build locally.

The original local build and collection blockers are resolved. Both the full minimal and full development Mac builds pass, including real model compilation, Cabana and generated native solvers. The complete project-scoped parent suite (`pytest openpilot`) passes: **957 passed, 86 skipped, 1 expected failure and 343 passing subtests**. Test-only macOS portability fixes preserve the functional assertions; vehicle control code and the paired child revision are unchanged. See [Mac validation and reproduction](MAC_VALIDATION.md) and the new `validation-*` logs. Earlier failed logs remain historical evidence. This does not establish AGNOS or Chestnut runtime behavior.

Final 45-segment replay uses the real trial selector and compiled final safety hooks: **252406 controller updates, 10082330 received CAN frames, 148765 accepted TX and zero rejected TX**. Complete generated CAN streams are byte-identical to the previous trial on all three saved drives. The 12425 messages at or above 45 mph match the regular pinion baseline. With both options off, all three complete CAN stream hashes also match the preserved original baseline; its existing one-angle rejection remains unchanged. Each drive has one initial RX warmup invalid tick, and pre-configuration TX is excluded.

The latest drive's recorded relevant RX/all-TX replay has the same two angle and eight button rejections as before; it does not equate those counts to hardware blocked counters or simulate changed vehicle response. A fresh-context source review found no important actionable issues in the paired implementation. These checks do not reproduce full forwarding/hardware behavior or provide sanitizer coverage.

Publication, AGNOS target verification, installation and road evaluation remain separate steps. See `replay-summary.json`, the mode-specific JSON files and `provenance.json` for exact evidence.

## Reproducing the focused checks

Use a Python 3.12 environment with this project's build/test dependencies and initialize the pinned submodules. Put the paired child first on PYTHONPATH. The captured `verification-environment.txt` records the local verification packages; it is not a replacement for the project's supported lock/setup workflow.

On macOS the replay debug library can be built with `cc -shared -fPIC -DALLOW_DEBUG -Wall -Wextra -Werror -Wno-pointer-to-int-cast -std=gnu11 -O0 -g -I opendbc_repo opendbc_repo/opendbc/safety/tests/libsafety/safety.c -o docs/navigator-integration/safety-debug-macos.so`. This deliberately omits sanitizer instrumentation, whose loading was not supported in the earlier Mac test environment. It is not the Panda firmware binary.

Set `NAVIGATOR_RECORDINGS` to the private complete 45-segment dataset directory, then run `python docs/navigator-integration/full_replay.py low-speed-prototype` and `python docs/navigator-integration/full_replay.py default-off`. The harness requires the documented anonymous route/segment layout and writes JSON beside itself; recordings are not bundled or committed. For the H7 target run SCons in `panda` with `PYTHONPATH` pointing to this paired opendbc checkout and the ARM toolchain on PATH. Do not flash the built artifact as part of local verification.

## Settings and source provenance

The [device settings snapshot](DEVICE_SETTINGS_SNAPSHOT.md) records the earlier read-only inventory. Before any installation take a new private parameter/source backup; do not commit full device parameters, credentials or device identifiers. Temporary updates remain paused on the device; this branch does not automatically change that preference.

The custom opendbc commit is pinned as a gitlink and the submodule URL points to Alex's fork. A public clean clone cannot fetch unpublished child commits yet. Publish the reviewed child and parent branches together when authorized, verify both remote SHAs, and test cloning before planning device installation.

The longitudinal coordination candidate, MADS, automatic calibration and new smoothing changes are intentionally outside this baseline.
