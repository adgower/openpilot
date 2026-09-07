# Navigator A3 source and dependency audit

This is the **core path-angle experiment**, not complete BluePilot Angle. Source inspection is offline, against local Git objects; branch labels below are snapshots, not claims about the current remote. No master update or device inspection was performed.

## Reproducible sources

| Role | Revision |
|---|---|
| Recorded parent A2 base | `e855605fdf1b2ba480185d9473fa544b8ee4b99c` |
| Recorded opendbc before A2 patch | `b4ef5e1cf406ff143fa67bdbfb154739d43279c9` |
| Saved local A2 implementation | `9fa4a58e` (resolve full ID in final manifest) |
| Panda selected by parent | `75aa44bec9140849868239b1f1e3f22624adb8fe` |
| Pinned Expedition donor | `3210caa02d9e09b46d08be490f689edb23b22b20` |
| Local `bp-dev` | `f43e3f5a64ab16a501f74911c5b74e0903d67c10` |
| Local `upstream/bp-dev` | `501a7c0e911245044196fcc90cb077a69fa0749b` |
| Local `upstream/bp-7.0` | `e1d051d7ba270261b4455068bd68f1a58db15a4a` |

The local `bp-dev` does not contain this donor's `lateral_angle_ext.py`; it must not be substituted for the remote-tracking development snapshot. Relative to `upstream/bp-dev`, the pinned angle file adds the Expedition-only provisional base gain: 0.95 × 1.5 = 1.425. Other Ford dependency differences include `carstate_ext.py` and its tests. Relative to `upstream/bp-7.0`, the inspected Ford extension/safety directory diff adds lane-center trim and its tests, changes angle integration and tests, and includes the Expedition gain. Ford safety is unchanged in those scoped remote-tracking comparisons. These are content comparisons, not branch ancestry or validation claims.

The preserved parent already contains the driver-monitor warp fix: `modeld/SConscript` uses a separate two-fisheye `dm_camera_configs` list while retaining hardware-specific driving-model camera configurations. `launch_env.sh` retains `FINGERPRINT=FORD_EXPEDITION_MK4`. The saved A2 helper and authorized A2 startup selector preserve wheelbase 3.1115 m, final model mass 2136 kg, static ratio 17; learned ratio is separate. No model file upgrade is required by this experiment. The baseline audit hashes original tracked blobs (including expanded LFS content where available) to verify production code preservation. The new core module has no production selector or safety hook; its transmission permission remains false. Named configurations are offline replay selections only.

## What the donor actually computes

The executable conversion is `path_angle_calc = kappa_cmd * v_ego * curvature_factor`, not the header's `0.5 * kappa * d_ref`. Curvature is inverse metres, speed metres/second, path angle radians; the empirical factor therefore supplies the effective time-to-angle mapping, not a verified wheelbase geometric identity. The `d_ref` helper exists but does not establish that executable mapping. Path angle is a Ford path coefficient, not steering-wheel angle or generic `LatControlAngle`.

Donor low-curvature gain interpolates across speed 13.5–26.82 m/s from 1.0 to platform base × dampening. High-curvature gain interpolates from 1.30 × low-speed factor to platform base × high-speed factor. Curvature magnitude 0.0007–0.001 1/m interpolates between these gains. User factor ranges are 0.5–1.5; dampening is 0.25–1.25. The Expedition 1.425 base is explicitly provisional. None is a validated Navigator calibration.

Donor strategy runs every `STEER_STEP=5` at a 100 Hz control loop, hence 20 Hz. Its soft path-angle rate table is per 50 ms call. Caller negates path angle for the CAN wire. Angle returns zero curvature, curvature rate and path offset, so plotting its curvature output as the actuator command is wrong.

## Dependency and behavior decisions

Paths below are relative to donor `opendbc_repo/opendbc/` unless otherwise stated. Implementation/test result files define the actual candidate acceptance outcomes; this table records the design boundary.

| Donor source | Purpose | Core disposition and destination | Evidence/test obligation |
|---|---|---|---|
| `sunnypilot/car/ford/lateral_angle_ext.py` | Mapping, gain schedule, limiter state | Adapt mapping into isolated core strategy, named startup profiles; no wholesale mixin import | Signs, speeds, gains, quantization, common-input isolation |
| same | Model preview, variable lookahead, exit-biased blend, lane-change gain | Omitted from core; changes prescribed trajectory | Core must not consume another model trajectory |
| same | PSCM/DBC saturation unwind and hold | Not treated as proof of physical authority; candidate bounded-envelope behavior is separately defined | Report differences rather than claiming full donor behavior |
| `sunnypilot/car/ford/lateral_curv_ext.py` | Shared result type, initialization, model subscriptions, stock-like limiting | Replace cross-fork inheritance with explicit inputs/outputs; retain frozen A2 bound as common input | No hidden `SubMaster` dependency; A2 regression |
| `sunnypilot/car/ford/human_turn.py` | Sustained press and wheel-angle takeover detection | Changed to immediate driver-press inactive output in the offline core, not donor sustained thresholds; documented separately from optional recovery | Press/release and bounded re-entry positive tests |
| `sunnypilot/car/ford/lane_center_trim.py` | Curvature-domain lane-center/offset correction | Omitted; would change common trajectory | No lane-position input to core |
| `sunnypilot/car/ford/values_ext.py` | Gain/rate constants, SP safety flags, geometry table | Do not import SP parameter plumbing or broader rate limits; explicit candidate config | Unsupported config fails; no blanket Ford enable |
| `sunnypilot/car/ford/fordcan_ext.py` | Dynamic lateral fields, private LKA mode/shadow bits, unrelated longitudinal/HUD extensions | Only necessary lateral encoding concepts adapted; frozen longitudinal/HUD retained | Encoded-command check, checksum/counter tests, unaffected CAN regression |
| `car/ford/carcontroller.py` | Strategy selection, sign, cadence, mode0 lifecycle | Offline-only strategy selection in replay; no production controller, interface or launcher hook | Default byte equivalence; explicit candidate telemetry |
| `car/ford/carstate.py` and `sunnypilot/car/ford/carstate_ext.py` | Yaw, wheel/driver inputs, PSCM diagnostics and numerous extras | Use existing host inputs; add only needed evidence, label missing state unavailable | Freshness/invalid-input tests; no alternate pinion measurement port |
| `safety/modes/ford.h` | Actuation admission and reset latch | Independent candidate final-wire checks; blanket clearing excluded | Compiled donor differential and positive/negative candidate tests |
| `safety/tests/test_ford.py` | Existing and donor admission regressions | Preserve stock regressions; donor acceptance is not safety oracle | Full results in safety report |
| `sunnypilot/car/ford/tests/test_lateral_angle_ext.py` | Shadow publishing, params, measurement selection, trim integration | Useful reference cases; rewrite for explicit core interface | Actual tests must include lifecycle and final command, beyond donor harness |
| root `bluepilot/selfdrive/car/bp_card_publisher.py` | Fork-specific controller telemetry | Do not import publishing stack/schema wholesale | Correct path-angle units/time axis in existing offline tooling |

Donor `LateralCurvExt` creates a SubMaster for `modelV2`, `liveParameters`, `selfdriveState`, `radarState`, `liveDelay`. Those fork services are not drop-in replacements for host `vehicleParameters` and `lateralDelay`. Core conversion does not need donor model prediction, learned-delay preview, radar or fork schema. Recorded host learned parameters remain diagnostic/model context; they are not overwritten by a donor service adapter.

## Reset and recovery are different mechanisms

Donor human-turn detection requires sustained steering pressure and absolute wheel angle over 45°, for 1.5 seconds, or 3 seconds when contact began already beyond that angle. Angle then requests mode0 and zero signals, resets internal state, and resumes through a zero-based rate ramp. Donor curvature mode has a different reset path with active-mode zero command and post-reset ramp.

Angle additionally includes 300 ms inactive pulses: six 20 Hz frames, sustained press release after 0.5 s, or reactive clip-binding stall for 0.5 s; 2 s cooldown, at most three reactive pulses per episode, and a 0.10 rad path-angle gate. These are loss-of-lateral-availability intervals, not state cleanup. Donor comments report Mach-E observations; they are not Navigator evidence. Core omits these automatic pulses. Their evaluation here is source/timing analysis only, not an implemented or compiled recovery system. Core also uses immediate driver-press inactive output rather than the donor sustained-angle detector; this can change lateral availability and requires its own review. Post-driver-override recovery performance cannot be inferred from core conversion alone.

The donor safety latch is not excluded from Angle: both LMC and LMC2 paths clear accumulated `violation` at neutral/reset and while the latch counts down. `RESET_BYPASS_LATCH_DURATION` is 60 frames, approximately 3 seconds only at the assumed 20 Hz cadence. Zero/reset can re-arm it. The controller comments claiming mode0 behavior does not need a latch are a hypothesis about legitimate messages, not proof that the latch has no effect. See the compiled admission report for each failed check and state divergence; valid inactive messages must pass independently.

## Attribution and publication boundary

[Primary donor](https://github.com/BluePilotDev/bluepilot/tree/3210caa02d9e09b46d08be490f689edb23b22b20) contains both an `opendbc_repo/LICENSE` comma MIT notice and a root `LICENSE.md` titled Custom MIT License. The latter grants viewing/modification but specifies permission for commercial, for-profit or closed-source use and acknowledgments for redistribution/visibility. `values_ext.py` explicitly references that root file. Do not describe all donor code as ordinary MIT or strip notices from substantial copied material. This audit does not resolve licensing applicability for later publication; no redistribution/push is performed here.

Required donor acknowledgments, preserved verbatim:

> This software is licensed under a custom license requiring permission for use.

> This project uses software from Haibin Wen and SUNNYPILOT LLC and is licensed under a custom license requiring permission for use.

Any later publication must retain applicable original copyright/license notices. The isolated original implementation avoids importing the donor's broader fork stack; conceptual attribution still identifies the pinned source above.
