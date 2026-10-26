# Bounded brake-release progress wait

Paired opendbc: `d4a201251d184a446cb2db383f4bd024c93e75cb`.

Two recorded longitudinal maneuver recoveries exceeded the one-second release watchdog while measured torque continued falling; physical clearance arrived about 1.49–1.53 seconds after the brake-off command. Permit a bounded additional wait only while fresh received torque shows substantial and continuing release. This never grants propulsion: keep the existing three distinct <=4 Nm samples over >=40 ms. Stalled feedback retains the nominal one-second timeout; the extended timeout threshold is two seconds. Preserve RestartClearance, owner reset/latching behavior, steering 1.20/0.97, CAN construction and native safety limits.

Local validation on October 25, 2026:

- Minimal and full Mac SCons builds and Panda build passed.
- Full parent suite: 1,061 passed, 87 skipped, one expected failure; 281 subtests passed.
- Full child suite: 4,581 passed, 1,690 skipped; 24,693 subtests passed. Native hooks use the existing plain Mac debug library; no sanitizer result is claimed.
- Recorded regression fails against the original module and passes integrated; adversarial stall, plateau, reversal, stale/duplicate feedback, restart and real-controller clearance contracts pass.
- Fresh 61-segment controller replay, 179,144 callbacks, exactly matches tested prototype wire/producer/trace bytes. Those wire streams passed native safety replay with no added rejection across 14,338,901 RX and 429,929 TX frames per variant; original rejections remain unchanged.
- Scoped Ruff and diff whitespace checks passed; protected steering/CAN/safety files remain unchanged.

This is local build/replay evidence. No Comma installation, ARM build, parked acceptance or road validation was performed. Separate pose faults and highway longitudinal comfort remain open. Recorded response is held fixed during replay; post-disengagement recovery is counterfactual. Eleven completed maneuver runs remain useful evidence, but later stages need another test after device validation.
