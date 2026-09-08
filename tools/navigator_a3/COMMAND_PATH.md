# Core path-angle command path

This checkpoint uses BluePilot mapping as an experimental design assumption. It
does not establish Navigator-specific physical limits or approve active steering.

## Implementation

`opendbc.car.ford.navigator_a3_command.select_command` returns two distinct frame
slots. The Ford controller only appends `production_frame`. In requested mode,
that slot must contain a neutral inactive message; an active/nonneutral message
raises an error. `experimental_frame` contains the scheduler's new proposal or
None, including proposals between A2 scheduled updates. Neither slot grants MCU
permission. There is no production transport factory or environment-based route
to the host tool.

`command_transport.py` is an in-memory host-only tool. It consumes experimental
frames and feeds synthetic publication/echo/health evidence to RuntimeBridge.
Its first fault persists, including a permission loss during a neutral driver
pause. A returned frame cannot clear it. Compiled evaluator results have no input
interface into this transport; missing echoes remain unknown.

The actual production requested-mode lifecycle remains inhibited and requests
the existing steering-unavailable disengagement. The host experiment does not
exercise active driving through that gate. Production firmware remains unchanged.

## Behavior retained, changed and omitted

| Behavior | Treatment |
|---|---|
| Curvature-to-path-angle mapping and named speed factors | Retained as the existing experimental assumption; no gain change |
| Common bounded input, measured-response limit, driver pause | Retained |
| Proposal scheduler and counter | Retained; only newly emitted proposals reach virtual transport |
| Command choice | Factored into shared immutable selection, separate production and hypothetical slots |
| Rejection evidence | Persistent host experiment fault; no feedback from compiled admission oracle |
| Synthetic serialization | Corrected synthetic bridge publication provenance; live behavior unchanged |
| Model preview and lane positioning | Omitted from core variant |
| Automatic recovery pulses and violation clearing | Excluded |
| Active vehicle transmission | Still blocked independently by production software and firmware |

## Reproduce locally

Use the existing pinned host environment. These commands do not start manager,
the launcher, pandad or any hardware connection.

```sh
export PYTHONPATH=/private/tmp/navigator-shadow-native-verified/msgq:/private/tmp/navigator-a3-opendbc:/private/tmp/navigator-a3
cd /private/tmp
/private/tmp/navigator-diagnostics-venv/bin/python -m pytest /private/tmp/navigator-a3/tools/navigator_a3/test_command_transport.py /private/tmp/navigator-a3/tools/navigator_a3/test_command_path_audit.py /private/tmp/navigator-a3-opendbc/opendbc/car/ford/tests/test_navigator_a3_command.py -q --import-mode=importlib
/private/tmp/navigator-diagnostics-venv/bin/python -m tools.navigator_a3.command_path_audit --stock-repo /private/tmp/navigator-a3-opendbc --output /private/tmp/navigator-command-audit
```

The audit requires pytest because it reuses the real card-method test harness:
hardware initialization, IPC publication and time are replaced by scoped fixtures.
It then compiles plain/instrumented aggregate and production safety host libraries
with the existing compiler helpers; exact commands and hashes are in the JSON.
All host sequences are finalized before either compiled evaluator is run.
Deliberate wire corruption and arrival compression alter only copied evaluator
inputs. They are not recorded faults or observed MCU timing.

## Builds and remaining boundary

This checkpoint changes Python only. No schema or safety-header changes, MCU
firmware compilation, signing or installation occur. Host shared libraries are
test artifacts, not Panda firmware. Existing native msgq/cereal/CAN components
are reused for tests. Target ARM64 packaging/build and device startup were not
run; no new on-device build claim is made.

Before activation: an independently justified final-command physical envelope,
EPS-specific handoff/re-entry behavior, matched production safety implementation
and separately approved target build/install remain necessary. Firmware research
is deferred; its absence does not block these software tests. The next software
step can add passive PSCM status to recorded handoff timelines, but it is not
implemented here and does not itself establish those physical limits.

Preserve earlier baseline commits and use paired parent/child bundles. For local
rollback select a separate checkout of parent720080953 and child2b97f9ee; never
reset unrelated work. Source rollback is not OS or firmware rollback. No device
rollback is required because this checkpoint never touched the device.
