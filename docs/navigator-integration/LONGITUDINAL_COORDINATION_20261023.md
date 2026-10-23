# Navigator paired longitudinal coordination

Default-off local integration. `FordPrefLongitudinalCoordination` is a persistent opt-in read by controlsd and card at startup; Alpha alone does not enable it. No device setting was changed in this integration. Existing steering1.20/0.97 and all lateral refinements remain unchanged.

The paired schema transports the exact calibrated pose sample time, control producer time, owner epoch/sequence and actual controller feedback. card registers optional CAN125 brake-torque feedback only for this trial. LongControl prevents positive integral buildup into a fresh known propulsion cap; the Ford controller qualifies moving entry/release with fresh physical observations and preserves that obligation across pedal overrides. Invalid/reversed required context or release timeout latches local neutral longitudinal output and the normal selfdrived immediate-disable/no-entry `controlsMismatch` alert. Fault details remain in carOutput; fresh data alone cannot resume.

Full policy and provenance: [paired child notes](../../opendbc_repo/docs/navigator-integration/LONGITUDINAL_COORDINATION_20261023.md).

Verification:35focused contracts;915Ford/native tests and12,083subtests (79skips); full parent1,028passed/87skips/1expected failure/281subtests. Minimal/full Mac builds, source type/lint checks pass. Broad opendbc4,485tests/24,690subtests pass; three sandbox-blocked MISRA mutation subchecks pass separately with offline existing-cache access.

Actual integrated65,736slot replay: default-off exact ACC parity; all non-ACC output identical. Modeled release: frozen brake transitions213→77; real-parent replay221→65, zero modeled faults.288response scenarios:172improve tracking, none>10%worse, zero faults. Two final native ACC RX/timer replays process5,260,361received frames/65,728ACC slots each, with no added rejection. These do not prove physical smoothness or fuel efficiency. Old actuator response belongs to old commands; a changed-command/old-response fault is not counted as an improvement.

No push, device access, ARM build, installation, screen/alert acceptance, powered Park checks or road evaluation in this task. Device staging/build and acceptance remain separate steps. Detailed local receipts: `/Users/alex/Apps/bluepilot/docs/research/navigator-long-integration-2026-10-23/REPORT.md`.

Pinned child integration: `d2e8d0404143da0f2685c2ff4932b9b9f702951f`.
