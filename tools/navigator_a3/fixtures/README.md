# Recorded Angle regression inputs

`recorded_angle_release.json` contains the two complete warning windows from the
20-segment BluePilot route `00000002--bfd966a552`. It also retains the active
rejection and all five exact preceding publications. The embedded manifest has
SHA256 hashes of the original analysis inputs and a canonical hash of the data.
The original report remains the source for warning interpretation and donor
response; these tests make no vehicle-improvement claim.

Rebuild from the saved route analysis:

```sh
python tools/navigator_a3/build_recorded_angle_fixture.py /path/to/bluepilot-angle-route/analysis
PYTHONPATH=/path/to/pinned-opendbc:/path/to/openpilot python -m pytest -q tools/navigator_a3/test_recorded_angle_fixtures.py
```

Native CAN publication and joined source timestamps are retained verbatim.
Strategy replay selects the next publication at least 50 ms after the prior
proposal. This is an explicitly synthetic scheduler, not production timing or a
faithful replay of the donor. Any resulting gap over 100 ms is passed through to
the strategy and must cause its existing cadence-neutral result. In warning 1,
the selected release sample has a 100.418125 ms gap; the tests require bounded
re-entry on the next valid step, not invented immediate availability.

Assertions cover all four profiles, both warning windows, measured-target bounds,
quantization, highway path-angle rate bounds, driver override neutrality, no added
recovery interval, five-way rejection ambiguity, persistent shadow fault, and
recorded-versus-synthetic isolation. A subsequent returned-frame case is generated
in the test and is explicitly a synthetic transport variant. Shadow faults do not
request A2 disengagement. The tests do not infer actual firmware admission, EPS
acceptance, or physical safety from these outputs. Controller and safety source
are unchanged.

Execution: the fixture tests were first run without their fixture and failed
(11 failures). After extraction and correcting two unsupported test assumptions
(all speeds above 25 m/s and every selected release gap below 100 ms), the suite
passed. Combined with the existing transport-observer suite: **34 passed**.
