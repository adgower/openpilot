# Navigator very-low-speed trial revision

This branch retains the existing pinion and low-speed-trial settings. When both
FordPrefSteerAngleCurvature and FordPrefLowSpeedGain are selected at startup,
the Navigator/forced-Expedition trial adds 10% to its current curvature-to-path-angle
factor through 15 mph, fades smoothly over 15–25 mph, and adds nothing at/above
25 mph. The owner's 1.20/0.97 constants, inherited bounds and driver override remain.

This is experimental feedforward calibration, not a verified handling fix. The
previous trial remains in its own module; the revised strategy subclasses it.
Turning the existing low-speed trial OFF selects the regular angle strategy,
turning off both this addition and the preceding 30–45 mph trial. There is no
separate new toggle for just the 10% addition. Restart/offroad startup applies options.

Matching Panda safety logic and geometry are unchanged from the preceding paired
master integration. No safety or firmware update is part of this Python revision.
Use the matching prior firmware/artifacts; do not combine arbitrary versions.

The local standalone prototype passed 62 saved segments across four complete drives
with zero generated native safety rejections and byte-identical commands at >=25 mph,
including 12,425 messages at >=45 mph. Final selector integration is separately replayed
and documented in the Mac installation report before device activation. Saved-input
replay does not reproduce vehicle response or certify future closed-loop behavior.

Back up current source, Git history, parameters/calibration and launcher before
installation. Preserve disabled Alpha longitudinal, update pause and current vehicle
selection. The preceding parent/child revisions are 6ae31161b73de2ce5043f708569759ac5f55f283
and c09baff5c2cdd7c887e25c2d88b671f403e1ed9d for application rollback.
