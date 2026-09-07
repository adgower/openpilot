# First-rejection investigation: bounded offline correction

The first A3 implementation skipped the frozen Ford adapter's acceleration/jerk envelope after receiving general-controls curvature. A bound in general controls does not replace the Ford adapter bound. The donor path-angle rate table alone admitted proposals that the unchanged compiled curvature envelope rejected.

Two observed requests became failing regression tests before the correction:

| Original event | Native timestamp ns | Previous host angle | Proposed host angle | Speed |
|---|---:|---:|---:|---:|
| A2 run 2, rising maneuver step | 1093282511972 | .030 rad | .0665 rad | 8.763889 m/s |
| Curve 17, first command after driver release | 6574578464128 | 0 rad | .013 rad | 23.76111 m/s |

Both fail `safety_max_limit_check(desired_curvature, highest_desired_curvature, lowest_desired_curvature)` in frozen `opendbc/safety/lateral.h:308`. Independent algebraic checks show their proposed equivalent-curvature steps exceed the existing host jerk envelope too. Later wire-rate failures follow from the host advancing proposal history while the C evaluator keeps last-accepted angle history. These are offline software observations, not measured EPS response.

## Correction

The candidate now reuses `CarControllerParams.CURVATURE_LIMITS.apply_limits` to obtain the frozen Ford acceleration/jerk window. It maps both endpoints to angle, intersects them with the existing angle-step and DBC bounds, then selects an encoded integer-grid point inside the intersection. This prevents rounding from undoing the limit. No nonempty intersection means neutral/unavailable, not permission to exceed a limit.

History records the algebraic inverse of the actual quantized proposal at the speed at which it was produced. It is proposal history, not a claim of acknowledged actuator execution. Diagnostics distinguish the requested conversion gain from the effective gain at the bounded output and expose equivalent curvature. The common bounded general-controls input is unchanged; normal model preview remains absent.

The independent C evaluator and production safety are byte-identical to the preceding experiment. No reset latch, rejected-command forgiveness, safety threshold increase or device action was introduced.

## Recorded-RX results

Each profile receives identical recorded RX and its synthetic TX stream, as before. A2 admission coverage remains run 2 (one complete maneuver window); both curve inputs remain the supplied segments. Original reports are retained under `navigator-a3`; new reports are under `navigator-a3-first-rejection` beside it.

| Profile | A2 rejected, before → after | Curve 17 rejected, before → after | Curve 29–30 rejected, before → after |
|---|---:|---:|---:|
| Expedition provisional | 60 → 0 | 68 → 0 | 1 → 1 |
| BOF reference | 60 → 0 | 552 → 0 | 406 → 1 |
| Low-factor sensitivity | 60 → 0 | 68 → 0 | 1 → 1 |
| High-factor/dampening sensitivity | 60 → 0 | 67 → 0 | 287 → 1 |

The remaining first command in curve 29–30 lacks initial RX/speed evidence (reason bits 66); its rejection remains. Each profile has 80 accepted active A2 commands, 728 accepted active curve-17 commands and 1,729 accepted active curve-29–30 commands. Valid inactive counts remain 0, 340 and 404 respectively. Acceptance counts do not establish a preferred physical gain or improved vehicle tracking.

## Tests and remaining boundary

The added integration matrix passes complete core proposals through the unchanged compiled evaluator across four profiles, two signs and six speeds, including driver press and release. Negative safety tests remain. A separate independent review also checked gradual speed transitions through gain knots; its bounded requests remained admitted. Actual combined execution output and source hashes accompany the report.

This correction restores acceleration/jerk limiting only. The host strategy still lacks the complete measured-yaw error-limiting behavior of the stock adapter, and unexpected rejection can still desynchronize histories. In particular, a large synthetic request against zero yaw can correctly trip the unchanged measurement gate. Do not claim universal recovery or complete BluePilot Angle equivalence.

Physical angle-to-motion mapping, gain calibration, driver-inactive availability, complete rejection handling, periodic firmware safety ticks and production firmware integration remain unresolved. No software was installed on or started on the comma device. A2 remains the vehicle baseline.
