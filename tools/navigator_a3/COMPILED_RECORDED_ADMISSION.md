# Compiled recorded admission boundary

`compiled_recorded_admission.py` builds production safety headers directly from child
`01bea343a7f7d4e4f8a1947539abacb59ab8c278`. It builds both plain and traced copies,
checks every replayed packet decision and 19 named state values for equality, and
records the executed failing expression and original line for a modeled TX.
Archive extraction and instrumentation occur only under the output directory.

This tool does **not** reconstruct faithful firmware runtime. Actual failed checks
remain `unresolved` in every observation. A matching modeled rejection is not
firmware failure attribution, especially after the first admission disagreement.

The replay preserves supplied file, message, and packet order; it never sorts RX
and TX by timestamp. It consumes only actual RX and recorded published sendcan.
Returned/rejected packets are observations and are never reinjected. Identical
bus/address/bytes are matched FIFO within 100 ms by default; duplicate messages,
logging omissions, and publisher/MCU transport timing still make matches ambiguous.
Each record preserves raw-file SHA256, path, message and packet indices, and
message validity. Invalid envelopes are counted and retained only as an explicit
raw-runtime scenario assumption; invalid publications or observations are excluded
from agreement statistics. Unknown source tags are excluded using the shared exact
source classifier.

Timer input uses host logMonoTime reduced to 32-bit microseconds. A 1 Hz
`safety_tick` and `safety_mode_cnt` are modeled with an explicit phase, preserving
source-supported RX validity/lag permission changes. Controls permission is never
seeded from health, future data, or synthetic approval. The source in pinned panda
`75aa44bec9140849868239b1f1e3f22624adb8fe`, `board/main.c:201`, additionally clears
permission after three heartbeat-engagement mismatches. USB heartbeat timing,
firmware timer origin, precise mode transitions, and initial MCU state are absent
from the rlog, and these missing events are not synthesized.

For the recorded route, carParams reports Ford safetyParam **3**, alternative
experience **0**, and openpilotLongitudinalControl **true**. Startup panda health
is noOutput at 259473216878 ns, elm327 at 259594807758 ns, still elm327 at
268330724104 ns, then Ford3 at 268449297847 ns. The actual mode transition is only
bounded by the latter two health observations. Starting Ford at the beginning of
the route creates a persistent relay malfunction and must not be used as evidence.
Starting fresh at the first Ford sample is an explicitly truncated scenario with
unknown prior state, not a proved precise firmware initialization.

Example (run from `/private/tmp/navigator-a3`, supplying segments in numeric order):

```sh
PYTHONPATH=/private/tmp/navigator-a3:/private/tmp/navigator-a3-opendbc \
 /private/tmp/navigator-diagnostics-venv/bin/python \
 -m tools.navigator_a3.compiled_recorded_admission \
 --opendbc /private/tmp/navigator-a3-opendbc \
 --param 3 --alternative-experience 0 --start-ns 268449297847 \
 --tick-phase-us 0 --output /private/tmp/navigator-stock-recorded \
 --rlog /absolute/path/to/segment0/rlog.zst \
 --rlog /absolute/path/to/segment1/rlog.zst
```

Run the fresh-library RX permission/stale-tick/instrumentation test with:

```sh
PYTHONPATH=/private/tmp/navigator-a3:/private/tmp/navigator-a3-opendbc \
 /private/tmp/navigator-diagnostics-venv/bin/python \
 -m unittest tools.navigator_a3.test_compiled_recorded_admission
```

The combined dynamic-window check in `lateral.h:308` includes both per-frame jerk
and the measured-curvature error constraint. Calling it just a generic angle
magnitude failure is incorrect. The Ford final path angle check is a separate
neutral-wire requirement. Driver steering override is not itself an input to the
Ford RX hook's controls permission; a coincident override does not establish which
compiled predicate failed.


`compiled_first_rejection_order.py` is a separate route-specific bracket. Two fresh
compiled prefixes begin at the same first Ford sample. The host-order prefix
admits the unique first active TX at 511955516970 ns. The second prefix moves only
the actual brake RX (`0x165`, bytes `20c3410000000000`, packet 12 of the
511961025272 ns CAN batch) before that TX; the rejected echo is packet 72 of that
same batch. The injected packet uses the TX host timer as an explicitly assumed
order/time. The packet encodes brake value 2 and cruise state 3, so this does not isolate the
brake edge from the concurrent cruise disengagement. It naturally clears permission
and produces the executed predicate
`!controls_allowed && steer_control_enabled`. No permission override is used.
This supports a brake/permission timing explanation as plausible, while actual
firmware failure attribution remains unresolved. It does not explain other
rejections or prove the exact hardware receive order.

```sh
PYTHONPATH=/private/tmp/navigator-a3:/private/tmp/navigator-a3-opendbc \
 /private/tmp/navigator-diagnostics-venv/bin/python \
 -m tools.navigator_a3.compiled_first_rejection_order \
 --opendbc /private/tmp/navigator-a3-opendbc \
 --route /absolute/path/to/navigator-shadow-drive \
 --output /private/tmp/navigator-first-rejection-order
```
