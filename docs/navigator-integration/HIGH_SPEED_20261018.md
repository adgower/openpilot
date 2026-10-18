# Body-confirmed high-speed trial integration

Integrated locally on the paired `codex/navigator-tracking-release` branches on October 18, 2026. This supersedes the research-only integration status of the wheel/body-release package. Validation receipts and exact paired revisions are recorded in the BluePilot workspace at `docs/research/navigator-paired-highspeed-build-2026-10-18/REPORT.md`.

## Selection and preservation

`FordPrefHighSpeedTracking` is a separate persistent Boolean, default **false**. The selector also requires the existing persistent-tracking and roll-mapping preferences and the pinion/low-speed CarParams options. Missing prerequisites retain the existing strategy. An older compiled Params registry treats the new option as false. No UI control is added in this integration, and no device preferences are written.

Owner **1.20/0.97**, Navigator geometry, fixed pinion conversion, existing roll map, Ford controller and native safety source remain unchanged. The extra reduction starts above 60 mph and fades fully in by 70 mph; replayed lower-speed commands must remain identical. This is a bounded reduction trial, not a fitted gain or a demonstrated handling fix.

## Runtime context

`card` subscribes to device motion and extrinsics calibration. Only the new opt-in strategy consumes the added context. The adapter keeps publication, sensor-source, subscriber-receipt and consumption times distinct. Live receipt uses SubMaster `recv_time`; replay explicitly uses publication time as its receipt proxy. Conflated updates are applied in publication order. Invalid calibration immediately revokes cached transformed motion, and generation changes prevent mixing histories across calibration resets.

Admission requires coherent fresh request/calibration/state context, persistent wheel excess, and fresh same-direction calibrated-body excess with unique source samples. The admitted bend retains a fixed peak: cumulative retreat latches withdrawal; a lower plateau cannot rearm it. New rising demand must establish fresh wheel/body evidence before persistence restarts. Driver pressure withdraws the extra correction through native slew; disengagement and human-turn handling retain the established reference path.

## Evidence boundaries

The portable controller tests, parent factory/receipt tests and actual producer-to-card capture test exercise local integration without a vehicle connection. The retained corpus replay uses frozen recorded motion and native Mac debug safety hooks. It cannot predict changed vehicle response, lane clearance, live bus timing or ARM/AGNOS behavior. Seven historical recorded permission/cancel rejects must remain distinguished from zero additional candidate rejects.

Publishing, device backup/build/activation, powered parked acceptance and road evaluation are separate steps. This integration performs no Comma access, firmware flashing or physical CAN transmission.

Integrated child pin: `9d5a3d485085f36beeb9aff7baf53c0d4ea603ee`.
