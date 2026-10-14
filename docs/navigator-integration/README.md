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

The full minimal Mac build stops at Metal command-queue allocation during model warp compilation. Full parent pytest collection also stops on missing generated native solver/localizer artifacts and optional dependencies. Those failures are recorded in the logs; this is not a completed full openpilot build or full parent-suite pass, and it is not device/road validation.

The original migration design also requires full recorded-input safety replay, old/new trial command comparison, and independent review. Their final results and paired SHAs are recorded in the project handoff report; do not infer completion from this intermediate README alone. Publication, installation, AGNOS migration and road evaluation remain separate steps.

## Settings and source provenance

The [device settings snapshot](DEVICE_SETTINGS_SNAPSHOT.md) records the earlier read-only inventory. Before any installation take a new private parameter/source backup; do not commit full device parameters, credentials or device identifiers. Temporary updates remain paused on the device; this branch does not automatically change that preference.

The custom opendbc commit is pinned as a gitlink and the submodule URL points to Alex's fork. A public clean clone cannot fetch unpublished child commits yet. Publish the reviewed child and parent branches together when authorized, verify both remote SHAs, and test cloning before planning device installation.

The longitudinal coordination candidate, MADS, automatic calibration and new smoothing changes are intentionally outside this baseline.
