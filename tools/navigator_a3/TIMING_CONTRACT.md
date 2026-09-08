# Angle timing contract: corrected provenance

Base parent `f6d99607c7a2679a8aeda68d96f28990262bafd4`; base child `12a4366c866b8b3d8bac46a0e730e337351567a0`.

## What this checkpoint changes

The strict minimum interval in `core_path_angle.h` is a historical experimental choice, not a documented Ford/EPS requirement. That file stays byte-identical. The 100 Hz shadow scheduler and its at-least-50-ms emission policy also stay unchanged. The new host-only aggregate evaluator removes the extra strict-interval predicate and reuses the existing stock aggregate message-rate function, verified against sunnypilot `f95f996f5917dcbbf2e32fe51b606a24cf836af6`.

This algorithm already existed beneath the extra strict guard in the historical evaluator. No second limiter or new timing constants are introduced. It checks current plus previous message buckets before incrementing, then rolls at half the 250,000 us interval. Its nominal frequency is 20 Hz, with the upstream 1.2 factor plus integer allowance. It is not a literal cap of five messages in every rolling 250 ms, nor proof of acceptable EPS arrival timing. Its actual boundary behavior is tested, including the initial allowance and bursts.

The aggregate variant preserves accepted path-angle and equivalent-curvature references on rejection; rate accounting is not rolled back. This differs deliberately from the historical stock curvature reference reset. A rejected packet must not become an accepted reference. Neutral packets undergo normal checks; they do not grant permissions, clear production/runtime fault latches or acknowledge any shadow proposal. Offline evaluator decisions never feed the controller.

## Constraint origins

| Constraint | Origin | Meaning and limit of evidence |
|---|---|---|
| Raw neutral 1000; 0.0005 rad quantum; encoded field bounds | Wire encoding: pinned Ford DBC | Path-angle encoding, not steering-wheel angle or validated vehicle authority |
| Host/wire sign reversal | Donor behavior: frozen Ford packing callsite | Verified software conversion, not a physical transfer model |
| Nominal 20 Hz Ford steering updates | Donor and frozen A2 software scheduling | Not a strict USB or MCU interarrival minimum |
| Shadow waits at least 50 ms after emission, no catch-up | Our experimental host scheduling choice | Unchanged; cannot guarantee transport spacing or admission |
| Strict active-admission interval >=50,000 us | Our historical experimental assumption | Retained only in the historical comparison variant |
| Half-window aggregate message-count algorithm | Upstream software protection at pinned sunnypilot source | Exact algorithm reused; integration into Angle remains experimental |
| Per-message path-angle delta bound | Donor-derived experimental bound | Preserved; not independently justified Navigator physical limits |
| Curvature acceleration/jerk, measurement checks | Frozen stock software protection | Applying these to an algebraic Angle inverse remains an unvalidated hypothesis |
| Above 9 m/s measured target clamp and 100 ms input freshness | Frozen A2/our host validation policy | Unchanged target-before-rate ordering; not an EPS deadline |
| Low/high/dampening profiles and algebraic inverse | Donor behavior plus experimental calibration | Gains unchanged; inverse is not evidence of command-to-motion limits |
| Preserve accepted reference after rejection | New experimental history policy | Separates accepted demand from rejected proposals; does not infer EPS state |
| Reject nonneutral final path angle in production | Independent production restriction | Unchanged in all configurations; no active Angle activation |

## Source references

- [Pinned sunnypilot aggregate check](https://github.com/sunnypilot/opendbc/blob/f95f996f5917dcbbf2e32fe51b606a24cf836af6/opendbc/safety/lateral.h#L177-L196).
- [Pinned sunnypilot interval declarations](https://github.com/sunnypilot/opendbc/blob/f95f996f5917dcbbf2e32fe51b606a24cf836af6/opendbc/safety/declarations.h).
- [Pinned BluePilot path-angle delta checks, without a minimum interval](https://github.com/BluePilotDev/bluepilot/blob/3210caa02d9e09b46d08be490f689edb23b22b20/opendbc_repo/opendbc/safety/modes/ford.h#L299-L325).
- [BluePilot executable conversion](https://github.com/BluePilotDev/bluepilot/blob/3210caa02d9e09b46d08be490f689edb23b22b20/opendbc_repo/opendbc/sunnypilot/car/ford/lateral_angle_ext.py#L520-L530).
- [BluePilot published firmware interpretation](https://bluepilot.dev/announcements/?post=bluepilot-7-0-the-return-of-angle-control-it-wasnt-the-models-fault). This is a developer claim; it does not identify a validated Navigator actuation envelope.

## Build and evidence boundaries

Only isolated host C libraries are compiled for the comparison. This checkpoint changes no controller code, schema/native interfaces, model artifacts, Panda firmware source dispatch or device installation. No MCU firmware artifact is produced. Existing tests cover serialized card integration, A2/shadow CAN equivalence and the production Angle block.

Every new trace is synthetic and uses identical packets/timestamps across variants. It is not a replay of actual MCU arrival timestamps. The prior drive's rejected frames remain recorded evidence; changing an offline evaluator does not reclassify those actual events or explain their failed checks. An absent rejection or an accepted host test does not prove vehicle execution.

Inspect both whole-sequence differences and their before/after histories: earlier admission differences can change later failures. Only executed failed checks are attributed; predicates skipped by early returns remain unevaluated. Preserve the historical reports as published rather than rewriting their outcomes.

## Specific next activation evidence

Timing agreement is one software property. Before considering an active build, the remaining evidence must identify:

1. The applicable PSCM/EPS firmware and calibration identifiers and hashes, the actual relevant code/tables or equivalent documented evidence, and applicability to this Navigator configuration. A firmware address alone is insufficient.
2. The relation from encoded path angle to yaw/steering response across both signs, speed, transient demand, load and relevant sensor uncertainty. DBC representable limits, the gain formula, and a maximum observed in one donor drive are not safe operating bounds.
3. EPS state after driver intervention, valid inactive messages, rejected/missing packets and first active re-entry, including loss-of-assist intervals and uncertainty. Returned CAN frames do not establish this state.
4. Independently reviewed final-byte enforcement implementing that supported envelope, with matched controller/firmware build artifacts and rollback before any installation approval.

If published evidence cannot answer these, a separately reviewed instrumented bench/contained-facility protocol must establish instrumentation, applicable hardware, timing accuracy, supervision and stop criteria before choosing excitation bounds. This document supplies no driving instructions or authority to disable the production gate. No additional ordinary road drive is requested for this checkpoint.

## Reproducible host commands

Run from `/private/tmp` with the existing isolated environment. Paths are local build/research worktrees, not the device checkout:

```sh
export PYTHONPATH=/private/tmp/navigator-shadow-native-verified/msgq:/private/tmp/navigator-a3-opendbc:/private/tmp/navigator-a3
/private/tmp/navigator-diagnostics-venv/bin/python -m tools.navigator_a3.aggregate_timing_contract --stock-repo /private/tmp/navigator-a3-opendbc --output /private/tmp/angle-timing-reproduction
/private/tmp/navigator-diagnostics-venv/bin/python -m pytest /private/tmp/navigator-a3/tools/navigator_a3/test_angle_timing_contract.py -q --import-mode=importlib
```

The new report's `run-verification.py` supplies the complete regression command, including native msgq resolution for card integration. It records the actual test output. `aggregate-timing-cases.json` records C compile commands, compiler identity, hashes and executed predicates for plain/instrumented builds. Those are unsigned host test libraries, not MCU firmware artifacts. No Python dependencies, shared paths or target OS were changed.

Paired bundles are incremental and require the corresponding base pins at the top of this document. Verify each with `git -C <matching-repository> bundle verify <bundle-path>` before importing it into a separate review checkout. The delivery manifest records the resulting heads and bundle checksums. Keep both source pins paired. Returning to the base pins only reverts this local software experiment; it is not an OS, device configuration, model-state or firmware rollback. No such device state changed here.
