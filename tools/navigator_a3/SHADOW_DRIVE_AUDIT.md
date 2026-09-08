# Recorded A2 rejection / offline A3 comparison

This checkpoint adds offline tools only, based on parent `86b4a4cbb` and child
`01bea343`. Production controller, runtime latch, safety, model and parameters
remain unchanged. Never feed these results back into live control.

## Reproduce

Use the existing diagnostics virtual environment. Set `route` to the directory
containing `00000003--371271599e--0/rlog.zst` through segment 22, and `out` to a
new output directory. Run from the repository root:

```bash
export PYTHONPATH=/private/tmp/navigator-a3-opendbc:/private/tmp/navigator-a3
route=/absolute/path/to/navigator-shadow-drive
out=/private/tmp/navigator-shadow-reproduction
/private/tmp/navigator-diagnostics-venv/bin/python -m tools.navigator_a3.drive_rejection_timeline --root "$route" --output "$out/timelines"
/private/tmp/navigator-diagnostics-venv/bin/python -m tools.navigator_a3.counterfactual_replay --route-id 00000003--371271599e --output "$out/counterfactual" --logs "$route"/00000003--371271599e--*/rlog.zst
```

The counterfactual reader validates contiguous numeric segments and handles
adjacent native timestamp overlap. The timeline tool additionally uses the
preserved `analysis.jsonl` to define the warning/rejection windows. It retains
every plausible preceding byte-identical publication through the standalone
observer; no echo establishes vehicle execution.

Compiled replay commands, assumptions and MCU reconstruction limits are in
`COMPILED_RECORDED_ADMISSION.md`. `compiled_first_rejection_order.py` brackets the
first rejection by moving one actual brake/cruise frame; it does not prove real
MCU ordering. The signed firmware is not modified or installed by any tool.

## Observed findings

All 23 raw files were verified against prior hashes. Chestnut stayed on the big
model in all recorded model outputs. The first active steering rejection spans
a brake/cruise disengagement: host-order compiled replay accepts, while the
explicit same-batch RX-before-TX scenario rejects because controls are no longer
allowed. Later modeled dynamic-window failures remain hypotheses after admission
divergence. Actual failed MCU checks remain unresolved.

The recorded A3 evidence fault stays latched. The counterfactual computes a
separate stream using inferred native source validity, preserving all measurement,
driver, freshness, cadence and command limits. Invalid carOutput evidence is
unsupported. Repeated diagnostic publications do not advance controller time.
Other fault causes are not cleared. Synthetic bytes have unknown transport
acceptance and never gain transmission permission.

Focused tests include real first-rejection/permission events and a real
driver-press/release fixture, plus frozen A2/A3/runtime regressions. Full card
integration collection requires the native msgq dependency, unavailable in the
diagnostics venv. Do not describe this checkpoint as installation or driving
readiness.
