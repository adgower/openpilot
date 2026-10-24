# Navigator calibration update continuity

Calibration and device-motion messages arrive independently. Previously, a calibration-only publication cleared the coordination pose-valid flag until the next motion update, even with unchanged calibrated extrinsics and healthy fresh motion. During active longitudinal PID this latched invalid_pose_or_accel and displayed Controls Mismatch.

The producer now rebuilds calibrated orientation and validity when either input changes, once a motion sample has been seen. It retains the original motion timestamp,100ms freshness limit, health/finite/reversed-time checks and genuine fault latching. The disabled coordination path retains its original update cadence. This is a parent-only fix; opendbc d2e8d0404143da0f2685c2ff4932b9b9f702951f, steering1.20/0.97 and vehicle geometry remain unchanged.

Nineteen continuity cases plus seven retained focused contracts cover real producer/publish/schema/card/controller engagement, calibration-only/changed/simultaneous inputs, missing motion, invalid/uncalibrated/nonfinite/stale/future/reversed inputs and owner-reset fault latching. The subprocess wrapper keeps model/control imports isolated from suites that later fork replay workers.

Saved-input replay repairs all five first-fault episodes without forced pose validity; the disabled producer is byte-identical over94,980controls. Full71-session/1,015-segment lateral safety replay, retained full longitudinal/native/model regressions and fresh failed-drive native replay add no safety rejection. Historical reference rejects remain reported. This is offline software evidence, not ARM, parked or road validation of this repair.

Detailed evidence and roadmap are maintained in the owner's bluepilot docs/research/navigator-calibration-update-fix-2026-10-23 and navigator-calibration-integration-2026-10-23 directories. No device configuration, publication or activation is part of this local integration.
