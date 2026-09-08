# Offline steering rejection observer

This checkpoint starts from parent `1ff28439b088aac04e226542dd0bd7bfbe5688b2` and child
`27e76255e4b0b1e372759e970227ec7a76f921d6`. It adds standalone tools only.
The controller, independent C evaluator, production safety and Panda sources stay unchanged.
No device access, activation, firmware build/install, push or merge is part of this checkpoint.

## Reproduce on this checkout

Existing reader dependencies are installed in `/private/tmp/navigator-diagnostics-venv`.
The local `openpilot` reader import requires the diagnostic checkout on PYTHONPATH.
Pure observer and fixture processing use only Python's standard library; tests use pytest.
The manifest supplies local raw-log paths and prior window bounds. It never supplies
compiled admission decisions to the observer. Select a new output directory to preserve previous reports.

```sh
cd /private/tmp
export PYTHONPATH=/private/tmp/navigator-a3-opendbc:/private/tmp/navigator-lateral-diagnostics:/private/tmp/navigator-a3
export NAV_TRANSPORT_BASE=/Users/alex/.codex/visualizations/2026/09/05/01a07385-2ba0-7b50-8447-0f8152e58e2c
export NAV_TRANSPORT_OUT=/private/tmp/navigator-transport-reproduced
/private/tmp/navigator-diagnostics-venv/bin/python -m tools.navigator_a3.transport_replay \
  --manifest "$NAV_TRANSPORT_BASE/navigator-a3-measured-response/replay-manifest.json" \
  --output "$NAV_TRANSPORT_OUT"
/private/tmp/navigator-diagnostics-venv/bin/python -m tools.navigator_a3.transport_fixture_comparison \
  --fixtures "$NAV_TRANSPORT_BASE/navigator-a3-measured-response/rejections/rejection-cases.json" \
  --output "$NAV_TRANSPORT_OUT/fixtures"
```

The replay regenerates extracted events, observer timelines, candidate groups, per-recording
reports and `index.html`. The final delivered package additionally includes this document,
source verification, test logs and local Git bundle; those packaging artifacts are not fabricated by replay.

For already normalized transport evidence:

```sh
/private/tmp/navigator-diagnostics-venv/bin/python -m tools.navigator_a3.transport_observer \
  --events /path/to/transport-events.jsonl --output /path/to/new-observer-output
```

## Input and evidence contract

Each event has nonempty `route_id` and `provenance`, strictly increasing observation `order`,
native `t_ns`, `valid`, and `kind`. Publication/echo packets retain physical `bus`, `address`,
byte length `dlc`, complete `data_hex`, original tagged `src`, source file/event/packet indices.
The extractor preserves message and packet order within numerically ordered route segments;
it never sorts observations by timestamp. Identical whole input files are deduplicated by SHA256.
Route identity is literal from supplied filenames. Different naming aliases are deliberately not
merged without evidence. Inputs must describe one coherent observation stream per route/provenance;
conflicting copies of the same segment should be resolved before combining them.

Panda health retains index, validity, `safetyTxBlocked`, `controlsAllowed`,
`safetyRxChecksInvalid`, `safetyModel`, `safetyParam`, and `alternativeExperience`.
Incomplete/invalid health does not establish continuity. Counter decreases are discontinuities
with reset versus wrap unknown. A configuration change also prevents continuity inference.
An increase is aggregate traffic evidence only. Health sampled before/after a packet does not
establish the permission or configuration at its exact transmission time.

Steering scope is Ford LateralMotionControl2 (0x3D6), including mode 0 inactive messages.
Other rejected traffic is retained and separately classified. Unknown bus tags remain unknown.
Tag interpretation follows the frozen transport: 0x80+bus returned, 0xC0+bus rejected.
A returned CAN-FD frame is transport evidence; it does not prove EPS acceptance or execution.
The frozen firmware's TX request/returned-frame path is documented in the preceding integration
specification. This tool does not turn publication or missing rejection into permission to transmit.

Candidate correlation requires exact route, provenance, bus, address, DLC and payload,
earlier observation order, and publication native time no later than the echo.
No arbitrary time horizon or one-to-one match consumption hides repeated payloads.
`candidate_selector` references `publication-groups.json`; `resolve_candidates(groups, row)`
returns EVERY eligible publication. A unique match is unique within supplied coverage only.
Repeated identical echo content is flagged as possibly duplicated, not declared a duplicate.
A repeated echo therefore never increments a claim of uniquely rejected commands.

The JSONL timeline retains all steering, health, rejected and unknown observations. The larger
extracted file retains all publications and tagged CAN evidence, including unrelated normal traffic.
The HTML includes every direct steering rejection and a one-second approach/aftermath around the
first observation. Native timestamps remain shared across lanes; unique publication intervals are
not safety-decision latency, EPS response latency, or production timeouts.

## Findings in the saved recordings

Full A2 route: 74 valid steering rejection observations, with 56 unique, 11 ambiguous and
7 unmatched preceding-publication correlations. There are 44 other rejected-frame observations.
First observed steering rejection: native 879.054451896 s, mode 0 inactive, publication interval
6.391218 ms. That observation does not establish a maneuver fault. There are 72 mode 1 and
2 mode 0 rejection observations. Aggregate positive counter deltas sum to 110 across the route;
one counter decrease breaks continuity, so this is not an exact rejection total. Seven sampled
permission-loss transitions are distinct from individual rejected frames.

The prior run-2 and run-19 A2 windows each contain 89 steering publications and 89 returned
observations with zero direct steering rejections. Full-prefix matching is preserved when counting
these windows. Full curve17 and curve29-30 recordings contain no rejected frames or observed
counter increase. Initial counters are already 119 and 219 respectively; prior events are outside
that coverage. These findings do not establish acceptance or explain outward drift.

Synthetic comparisons emulate echo kind from the existing fault-fixture admission labels, outside
the observer. This is explicitly agreement by construction and tests evidence handling, not the
safety evaluator. Complete echoes expose 1 stale-RX, 16 corrupt-RX, 16 controls-off and 1
excessive-command rejection observations; control sequences expose none. Omitting rejected echoes
exposes none of those rejections and leaves receiver acceptance unknown. The synthetic +1 ns echo
offset is not real timing. No result is fed back to the controller.

## Review and remaining gates

TDD covered unique/ambiguous/missing/duplicate/reordered echoes, full bytes and identity,
invalid health, counter discontinuities, unrelated rejected traffic and mixed provenance.
Independent review found missing identity/configuration validation; regression tests first failed,
then passed after those gaps were fixed. Final review found no remaining blocking defects.
Actual final test and source evidence is in the report's pytest.txt and verification.json.

This observer has no production consumer or control authority. Unexpected rejection handling,
transport loss deadlines, physical Angle command-to-response envelope, Navigator gains,
driver handoff and reviewed production safety remain separate unresolved work. There is no
new native/schema or MCU artifact. No installation or rollback on the device is required because
nothing on it changed. Local rollback simply selects the frozen parent and child in another
checkout; do not reset unrelated work. The incremental bundle requires the frozen parent object.
