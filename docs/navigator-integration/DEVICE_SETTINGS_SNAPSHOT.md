# Navigator settings and migration checklist

Captured October 14, 2026 through read-only SSH and the preserved trial source. No settings, services, source, OS, firmware, or models were changed. This inventories the known project customizations and selected related preferences; without an original full parameter baseline, it cannot identify every setting Alex has ever changed.

## Persistent device settings

| Key | Fresh observed value | Migration treatment |
| --- | --- | --- |
| `DisableUpdates` | `1` | Temporary update pause. Preserve during candidate preparation; deliberately revisit after installation and update behavior are verified. Do not confuse this with a source patch or permanent updater repair. |
| `FordPrefSteerAngleCurvature` | `1` | Custom pinion-feedback option. Preserve the setting and port its parameter registration, startup wiring, UI and paired safety support. |
| `FordPrefLowSpeedGain` | `1` | Custom trial option. Preserve with its pinion dependency; do not enable against incompatible geometry. |
| `AlphaLongitudinalEnabled` | `0` | Alpha currently disabled. Preserve for the initial upstream migration. |
| `ExperimentalMode` | Absent | Do not interpret absence as a saved true value or silently enable it during migration. |
| `ExperimentalModeConfirmed` | `1` | Confirmation flag exists; it does not mean Experimental mode is active. |
| `OpenpilotEnabledToggle` | `1` | Observed normal preference; not established as a project-specific change. |
| `DisengageOnAccelerator` | `0` | Observed preference; preserve intentionally and validate semantics on the target source. |

Also checked: `ExperimentalLongitudinalEnabled`, `Passive`, `IsLdwEnabled`, `IsRhdMetric` and `IsMetric` were absent. Absence is not proof of their runtime defaults or that all are registered parameters on this installation. They are not project migration overrides.

A second scoped check of all existing `Ford*` and `Mads*` parameter files found only the two Ford trial settings above and no Mads files. `SnoozeUpdate` was absent. This confirms the known trial-specific parameter inventory; it does not substitute for a fresh private full parameter backup before installation.

## Code and launcher customizations

| Customization | Current value or behavior | Where it belongs |
| --- | --- | --- |
| Forced vehicle identity | `FINGERPRINT=FORD_EXPEDITION_MK4` | Installed `launch_env.sh`; carry as a documented fork override until proper Navigator recognition is implemented and verified. |
| Navigator geometry | Wheelbase **3.1115 m**, base mass **2742 kg**, fixed steering ratio **19.6** | opendbc Ford platform definition. Runtime mass **2878 kg** includes the old 136 kg allowance. Mass normalization on new master removes the mass field; preserve geometry and equivalent normalized dynamics, not an obsolete field name. |
| Pinion model geometry | Center-to-front 0.44 × wheelbase; no rear steer; fixed stiffness and geometry shared with Panda | opendbc pinion guard and Ford safety. Current absolute stiffness front/rear **353037.28125 / 438491.46875**; adapt guards and verify equivalent curvature under the new schema. |
| Owner angle calibration | **1.20 / 0.97**, Expedition-only high-curvature arm | opendbc `lateral_angle.py`; these are source constants, not current live parameter keys. |
| Angle control and engagement ordering | Path-angle-primary Ford control and ordered mode transitions | opendbc controller/CAN/safety plus parent dependency pin. |
| Low-speed trial | Gentle ×1.15, tight ×1.05; smooth curvature blend 0.00025–0.00125 /m; full addition below 30 mph, fades to zero at 45 mph | opendbc prototype selector and helper. Existing rate/range/deviation/override limits remain part of the paired implementation. |
| Trial settings UI | Both standard and Comma 4 MICI paths | openpilot parameter definitions, startup application and settings UI. |

The settings and source layout are detailed in the [feature inventory](README.md). Do not blindly replay an old patch over new master and assume schema or UI compatibility.

## Installed operational state

- AGNOS `/VERSION`: **19.7**; Ubuntu 24.04.4; kernel 4.9.103. Installed launcher defaults AGNOS to 19.7. The new candidate must use its supported OS requirement—currently the audited master requires **19.9**—rather than preserve this old version override.
- `prebuilt` marker exists. It is a build artifact/workaround, not a feature to commit or blindly carry into new source. Rebuild the candidate and verify matching artifacts.
- `/data/navigator-stage-20261014` points to `/data/openpilot`. This supports old build-time paths; the new installation must account for or eliminate that dependency through a clean build, rather than treating the link as a portable feature.
- Parent Git base `f35e6e5eb`, child base `eb70bef4`, with the documented source overlay. The current source hashes and local trial commits identify behavior more accurately than device HEAD alone.
- Prior source and parameter backup: `/data/navigator-backups/20261014-pinion-low-speed-preinstall/`; previous installation: `/data/navigator-previous-20261014/`. These are application/parameter backups, not a proven OS rollback image.

## Paired branch and installation plan

1. Preserve the current tested source, relevant settings, firmware provenance and evidence in durable storage. Take a fresh private parameter backup immediately before any installation; do not commit Wi-Fi credentials, keys, device identity or the complete parameter directory to GitHub.
2. Create a clean opendbc feature branch from the dependency pinned by the chosen openpilot master snapshot. Port/review the Navigator changes as small commits, adapting mass-normalized fields and safety model agreement.
3. Create a clean openpilot branch from that pinned master. Point its opendbc gitlink to the exact custom child commit, with a fetchable submodule URL/ref in Alex's fork so a clean clone can obtain it. Port parameter registration, startup wiring, settings UI and the deliberate fingerprint override.
4. Build and validate both repositories and matching Panda firmware; replay historical and new-drive data under correct schema migration. Assess unchanged calibration and high-speed behavior separately from safety acceptance. Keep new MADS, longitudinal coordination and automatic calibration out of this first baseline migration.
5. Publish the paired fork branches when ready, verify a clean clone resolves the exact child commit, and write a concrete parked installation/rollback procedure including AGNOS, model assets and Chestnut firmware requirements. Publication has not occurred in this documentation task.
6. Install the validated branch using a controlled SSH-assisted transition or the appropriate fork installer, preserving selected preferences. A factory reset is not inherently required; simply running `git pull`, changing a parent branch alone or restoring an old `prebuilt` marker is insufficient. OS/firmware rollback feasibility must be established separately before updates.
7. Verify installed source, parameters, OS, firmware, normal startup, update handling and actual Chestnut operation. Mark installation separately from subsequent road validation. Revisit update pause only after confirming it tracks Alex's tested fork branch rather than replacing custom behavior with stock upstream.

Suggested future branch naming: `codex/navigator-master-integration` in both forks. These are proposed names; no branches were created in this settings/documentation turn.
