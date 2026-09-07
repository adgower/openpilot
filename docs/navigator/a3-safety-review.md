# Core path-angle experiment: offline safety review

**Not deployment-ready. Production Ford safety is unchanged.** The candidate C evaluator is a host-test hypothesis in `opendbc/safety/tests/navigator_a3/core_path_angle.h`, not a registered safety mode or firmware hook. Its positive results establish software properties under stated fixtures, not a validated path-angle-to-vehicle-response relationship.

## Exact compiled comparison

`tools/navigator_a3/safety_audit.py` extracts safety headers from frozen stock opendbc `b4ef5e1cf406ff143fa67bdbfb154739d43279c9` and the vendored opendbc in donor `3210caa02d9e09b46d08be490f689edb23b22b20`. It compiles four variants using the installed host C compiler: stock, unmodified donor, donor minus violation clearing, and stock plus the host-only candidate evaluator. These are host shared libraries, **not panda MCU firmware artifacts**.

The no-clear variant removes precisely four `violation = false` statements following reset detection/decrement (two in classic CAN, two in CANFD). It retains latch initialization, decrement/rearming, helper invocations and other state updates. No claim is made that this isolated donor mutation is itself a complete controller design.

Every variant is compiled twice: plain and instrumented. Instrumentation evaluates each captured `violation |= expression` or nonconstant `violation = expression` once, records its boolean result, and returns that same result. For all 17 sequences the admissions and exposed before/after state are identical between plain and instrumented builds. The JSON preserves source-file/line/expression for executed failed predicates. Some nested curvature predicates are evaluated but their result is intentionally discarded by donor Angle mode at zero wire curvature; the enclosing Ford predicates show which results are actually accumulated. Instrumentation is not a complete trace of every branch or outer whitelist gate. Cases with unexplained differences would fail report generation rather than receive an invented attribution.

Each sequence starts from reset. Tests explicitly initialize donor-only static state to power-on defaults because donor `ford_init` itself does not reset all of those variables. Most tests explicitly set `controls_allowed` as a fixture; separate cruise RX tests exercise actual engage/disengage state changes. Valid Ford speed/second-speed/yaw encodings, counters, quality flags and checksums pass through `safety_rx_hook`. No vehicle or bus is accessed.

## Admission findings

The current corpus produces **132 changed admissions** across repeated sequences, not 132 distinct faults. JSON includes the exact inputs/timestamps, prior sequence, donor/no-clear/stock/candidate result, before/after latch and command state, failed predicates and a state-divergence flag. Every first difference is also reproduced by a separately reset replay of the exact truncated prefix. Later results must be interpreted in light of preceding state differences; the clearing assignment alone is isolated at first divergence.

| Fixture | Why the donor/no-clear result differs |
|---|---|
| Controls off, active neutral/reset message | Explicit `steer_control_enabled && !(controls_allowed || controls_allowed_lateral)` fails; neutral command clears it anyway |
| Active command after neutral with controls off | Same controls gate fails during the latch window |
| Large path-angle step after reset | `path_angle_cmd_checks` fails its actual encoded-angle rate bound |
| Nonzero path offset on neutral curvature/angle | Offset magnitude and offset rate checks fail, but neutral detection clears them |
| Nonzero angle in lateral-inactive message | `path_angle_cmd_checks` fails its inactive-zero requirement |
| Excessive curvature after reset | Absolute curvature and steering checks fail |
| Shadow deviation after reset | `ford_shadow_curvature_error_check` fails the measured-curvature interval |
| Latch expiry and rearming | Sixty non-neutral messages are cleared after reset; neutral frames can rearm indefinitely |

The latch is **message-count based**, not a three-second timer. “Three seconds” assumes 20 Hz, but faster or slower sending changes elapsed duration. It runs in CANFD Angle mode too; it is not a special exemption solely for curvature control. Accepted neutral frames do not prove every other field is valid. Re-arming does not require controls to be allowed.

All changed fixtures here contain unauthorized, excessive or malformed demands; their rejection must remain. The corpus did **not** find a valid normal transition whose admission requires blanket clearing. That is a bounded finding, not proof that all real driver-recovery scenarios remain functional. The donor's controller recovery behavior may need a different justified design.

Positive tests independently require small normal commands, valid inactive messages, bounded re-engagement and cruise engagement to be accepted without the donor bypass. Candidate tests cover both signs, four named profiles, speeds at 1/13.5/26.82/35 m/s, stale and invalid RX, unsupported profile, out-of-domain speed, excessive command, cadence, relay fault and wrong-length RX. Stock is asserted to continue rejecting nonzero path angle.

## Candidate enforcement and limitations

The host evaluator decodes the **final LMC2 bytes**: mode, curvature, path angle, path offset and curvature rate. It requires supported mode, inactive auxiliary fields, explicit compiled profile selection, valid recent ABS/PCM/yaw evidence, 1–60 m/s domain, 50 ms minimum active cadence and zero angle when inactive. Relay malfunction rejects even neutral commands. It also bounds the final encoded path-angle step using the donor per-call ROC schedule without the donor headroom or reset bypass.

It independently inverts the selected algebraic `wire_angle = wire_curvature * speed * gain(curvature, speed)` hypothesis, then applies the frozen stock curvature magnitude, acceleration, jerk, measured-error, speed-mismatch, controls-allowed and real-time checks. The donor controller callsite negates host path angle before encoding; inversion of **wire** angle therefore has the same sign as Ford wire yaw curvature. No host shadow field grants admission. Accepted-only path-angle history and timing prevent rejected packets from seeding a permissive path-angle ramp. Stock curvature helper state retains its existing rejection/reset semantics.

The inverse is an algebraic hypothesis, **not proof that EPS responds like equivalent curvature**. Profiles are provisional and fixed in the host test; production must not accept arbitrary host gain claims. Outstanding issues before any firmware integration include physical transfer calibration, independent measurement alignment during driver intervention, endpoint/quantization uncertainty, freshness tolerances under real timing, profile authorization/provenance, and coordinated inactive intervals/re-engagement. Outbound LMC2 checksum/counter policy is not newly validated here. Positive hardware availability is unproven.

The supplied CSV diagnostics lack complete trusted CAN RX state. Such rows must be labeled admission unavailable; populating RX/engagement from convenient host scalar signals would fabricate safety evidence. The host API can consume actual recorded RX when available. Synthetic fixture acceptance and historical motion are always separate.

Controller-state reset, valid inactive transmission, bounded re-engagement, optional automatic recovery pulses and suppression of rejection are distinct mechanisms. Core recovery clears stale internal demand and re-enters through normal checks; optional recovery pulses remain omitted. No safety result here validates the complete BluePilot post-override recovery strategy.

## Reproduce and review

From the isolated parent checkout:

```sh
PYTHONPATH=/private/tmp/navigator-a3 /private/tmp/navigator-diagnostics-venv/bin/python -m pytest tools/navigator_a3/safety_audit_test.py -q
/private/tmp/navigator-diagnostics-venv/bin/python tools/navigator_a3/safety_audit.py --donor-repo /Users/alex/Apps/bluepilot --stock-repo /private/tmp/navigator-a3-opendbc --output /private/tmp/navigator-a3-safety-build
```

Actual result: **57 tests passed**. All four variants built, 17 sequences evaluated, 132 admission differences, plain/instrumented equivalence confirmed. `admission-results.json` contains compile commands and SHA-256 hashes of the actual host libraries. No MCU firmware was built or installed by this audit. No production safety header was changed.

The importable `build(..., 'candidate', False)` returns a ctypes library. Call `audit_init()` (controls start false), `audit_timer(time_us)`, and `audit_packet(address,bus,eight_bytes,is_tx)` for a complete stream. `audit_reasons()` reports additional candidate rejection bits; `audit_packet_len` exists for malformed-length testing. `audit_controls` and `audit_relay` are test-only fixture setters, not substitutes for recorded evidence. Real replay must feed the recorded cruise/RX stream and preserve gaps and timing.
