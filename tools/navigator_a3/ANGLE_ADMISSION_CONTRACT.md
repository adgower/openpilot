# Core path-angle admission and inactive/handoff contract

> Timing provenance correction: the strict 50,000 us gate described below belongs to our historical host-only evaluator. It is not an established Ford/EPS requirement. The scheduler remains a deliberate host policy. See [TIMING_CONTRACT.md](TIMING_CONTRACT.md) for the separate aggregate-rate experiment; prior results and artifacts remain historical evidence.

Status: executable offline contract; **active Angle admission is not approved or implemented by this checkpoint**.

Frozen parent: `7f574ce4726413c091294ed3803148c9c77706e7`. Paired child: `0322627c07c4993e631b3974dc9bf66ac9143362`. Chestnut, Navigator parameters, production controller, schema and firmware remain unchanged.

## Authorities and interfaces

There are three separate authorities. The controller proposes bytes and keeps proposal history. Panda decides admission from decoded bytes, validated RX, permissions and its own history. The EPS determines physical response, which is not observable from a returned frame alone. None may be substituted for another.

The current production contract is exact: independently decode final path angle and reject every raw value other than 1000. This applies to active and inactive messages. Neutral-angle mode-1 A2 is allowed only if all stock checks pass. The `requested` host configuration remains inhibited from startup and requests truthful disengagement through the existing lifecycle. No ordinary flag, profile or DEBUG build bypasses the final-angle gate.

The host-only C evaluator is an experiment, outside the firmware include chain. It additionally decodes mode, curvature, offset, rate, path angle and bus/length; uses checksum-valid speed/yaw RX; and applies format, freshness, path-rate, cadence and stock-curvature checks. Its curvature inverse is an algebraic hypothesis. Its accepted messages are not approved physical commands.

## Named contract requirements

| ID | Requirement | Executable evidence | Remaining physical evidence |
|---|---|---|---|
| BYTE-1 | Check actual final bytes independently; neutral A2 stays available under stock rules | Current production gate regression across all raw angles, modes and representative params | No missing evidence for continued nonneutral blocking |
| MAP-1 | An admitted Angle must satisfy an independently justified command-to-motion envelope | Experimental evaluator decodes and limits final bytes; inverse explicitly hypothetical | EPS semantics and uncertainty bounds, both signs/speeds/transients; no admissible active envelope established |
| TIME-1 | No catch-up permission; controller schedule and MCU timing must be compatible without weakened actuation bounds | 49/51 ms scheduled sequence against unchanged compiled evaluator; retain every decision | Derivation of any future time-based physical rate envelope, including delayed/bunched transport |
| HIST-1 | A rejected packet cannot be treated as executed; controller proposal, candidate accepted-angle and stock safety history stay separate | Full before/after snapshots for rejected and subsequent inactive/active messages | EPS internal history after rejection/inactive frames cannot be inferred from Panda state |
| HAND-1 | Driver press resets host demand but grants no transmission permission | Driver pause and release sequences; valid and invalid inactive frames checked normally | Whether inactive frames hold, reset or otherwise affect EPS demand; first-active response |
| FAULT-1 | Direct rejection, permission loss, invalid health and mismatch remain truthful and persistent as specified | Actual runtime bridge with delayed, missing, ambiguous and restored evidence | No finite host detection bound under missing feedback; physical response during detection gap unresolved |
| PROV-1 | Returned frames, aggregate counts and missing echoes never acknowledge synthetic commands | Runtime observation contract and existing serialized card regressions | Returned frame does not prove EPS execution |

The unresolved physical requirements are not numeric defaults. They must not be filled with the donor's gain, the DBC representable range, an inverse formula, or a maximum observed in one drive.

## Value provenance: what each number does and does not establish

| Existing value | Origin/use | Evidence status |
|---|---|---|
| Raw Angle neutral 1000, wire step 0.0005 rad | Pinned DBC/packer and final-byte decoding | Encoding verified; radians are path-angle units, not steering-wheel angle |
| Host sign opposite wire sign | Frozen Ford callsite | Conversion verified; physical response sign still needs independent validation |
| 20 Hz / five 100 Hz controller frames | Frozen Ford steering schedule | Software schedule; not a minimum measured USB spacing |
| 50,000 us between active hypothesis admissions | Existing host-only C evaluator | Not relaxed here; scheduler jitter may fail this requirement |
| 100 ms source/measurement freshness | Existing host controller and experimental corroboration gate | Preserved software threshold, not proven EPS detection/response deadline |
| Above 9 m/s, target within measured curvature ±0.002 m⁻¹ | Frozen A2 adapter ordering | Target bound precedes rate limit; not an independently validated Angle envelope |
| Four named low/high/dampening profiles | Pinned research configuration | Provisional; none gains production permission |
| 5/50/100/250 ms observation delays in tests | Synthetic examples chosen to expose delayed knowledge | Not measured maximum latency and not a safe actuation duration |

## State and history contract

1. Startup requested-A3 is blocked. Tests must not simulate an active product by clearing `physical_enforcement_unvalidated`, forcing controls permission, or disabling a violation.
2. A valid scheduled shadow step advances proposal history once regardless of what the offline evaluator says. Duplicate/backward times cannot advance demand. Long gaps and invalid input retain the existing neutral behavior.
3. Driver input produces a neutral host proposal with reset internal demand. The encoded inactive frame still undergoes independent checks. Neither reset nor an inactive request grants a reset window.
4. A rejected proposal leaves the EPS execution outcome unknown. Record all exported safety state before and after; do not assume every safety variable is unchanged on rejection. Never synchronize the controller to evaluator acceptance as if the runtime had that oracle.
5. Driver release may produce bounded *hypothetical* proposals from the host's reset state. Compare these with the evaluator's actual retained history. Inactive/admitted/returned messages are not an engagement handshake and do not approve physical re-entry.
6. A direct A2 rejection remains persistent evidence in shadow mode. Calculation may continue under separately valid inputs. A later non-exempt fault remains a separate persistent calculation inhibition. In requested mode, startup inhibition already applies before any event; this cannot validate active fault response.
7. No automatic retry, recovery pulse, fallback, fault clearing or resumption is specified here. Active re-entry remains unavailable until MAP-1 and HAND-1 are justified.

## Executable artifacts and scope

`angle_handoff_contract.py` builds isolated host libraries, feeds identical synthetic packet sequences to plain/traced implementations, and records executed failed predicates and histories. It identifies the old stock reference separately from the unchanged current production gate. Output decisions apply to these constructed cases only, not the recorded MCU timeline. No MCU firmware is built or installed.

`angle_transport_contract.py` runs the actual runtime bridge with synthetic timestamped evidence, excluding compiled admission inputs. It records direct rejection latency only for unique preceding publications. Ambiguous or incomplete history never creates a unique acknowledgment. Each requested-mode case stays inhibited; missing evidence yields no finite guaranteed detection bound.

Existing production gate and real serialized card tests remain mandatory regressions. Compiled inactive/re-entry successes demonstrate software admission only. All generated command-response language must remain hypothetical; no measured A3 improvement is claimed.

## Closure criteria for active implementation

For each of MAP-1, TIME-1 and HAND-1, provide an independently reviewed evidence record containing source identity, applicable EPS/hardware/configuration, measured or documented quantities, uncertainty, coverage and unsupported conditions. Candidate commands outside established coverage remain rejected. A source revision, pass count or donor success is not a substitute.

If available technical evidence cannot establish these facts, the next physical-review artifact must specify synchronized capture of host publication, MCU RX/TX/rejections, EPS state/response, yaw, speed, steering angle and driver input; independent timing reference; instrumentation accuracy; contained test environment; supervising personnel; stop criteria; and approved excitation limits. **No excitation amplitude, speed or driving procedure is authorized here.** Those are precisely the unresolved decisions requiring competent physical review. Do not run such a procedure or unlock firmware from this document.

Success for this checkpoint is executable, reproducible admission/history evidence and an explicit contract—not activation readiness. No additional ordinary drive is requested.

## Reproduce this checkpoint

Run from `/private/tmp` with the existing isolated Python and per-command `PYTHONPATH`:

```sh
PYTHONPATH=/private/tmp/navigator-a3-opendbc:/private/tmp/navigator-a3 /private/tmp/navigator-diagnostics-venv/bin/python /private/tmp/navigator-a3/tools/navigator_a3/angle_handoff_contract.py --stock-repo /private/tmp/navigator-a3-opendbc --output /private/tmp/angle-contract-reproduction
PYTHONPATH=/private/tmp/navigator-a3-opendbc:/private/tmp/navigator-a3 /private/tmp/navigator-diagnostics-venv/bin/python /private/tmp/navigator-a3/tools/navigator_a3/angle_transport_contract.py --output /private/tmp/angle-contract-reproduction
```

The compiled cases use explicitly synthetic 25 m/s straight-motion RX, CAN-FD Ford safety parameter 2 and four existing profiles. The transport examples use parameter 3 solely as a matching configuration fixture. Neither is a reconstruction of the installed route. The existing production-gate regression separately covers representative safety parameters and both DEBUG/release host builds. Do not promote the compiled hypothesis library to MCU firmware.

For tests, add `/private/tmp/navigator-shadow-native-verified/msgq` first in `PYTHONPATH` when including real card-method regressions; the prior reliable-shadow checkpoint contains its exact native dependency recipe. Run both new `test_angle_*contract.py` files and the existing Ford/A3, production-gate, card and tool suites. The report includes exact executed output.
