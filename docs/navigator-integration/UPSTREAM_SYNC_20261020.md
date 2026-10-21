# Upstream master integration — October 20, 2026

Locally merge commaai/openpilot master `1ace967a7e27d2537a956c683978f646d6b6d739` into the Navigator feature branch, retaining custom opendbc `8ccc6cf0ee123afddddbf6e0a0ea3d3316669a5e`, its fork URL, all Ford geometry/safety/control source and owner1.20/0.97. The upstream msgq dependency changes to `a95f3c1c5e7a4d84b193433a2a04d20c7a72878c`; other dependency pins remain unchanged.

Both settings-screen conflicts preserve Navigator pinion/low-speed toggles and offroad/vehicle guards, while adopting upstream initial maximum speed controls and removal of Experimental Mode controls. The Mici test fixture supplies the new initial-speed widget and retains all Navigator guard tests.

Upstream Autospeed uses model speed planning by default on supported openpilot-longitudinal configurations. ExperimentalMode is removed; preserving an old off value does not select Chill. AlphaLongitudinalEnabled remains a separate actuation opt-in. Current saved device configuration uses OEM ACC and Alpha off; this local merge does not change it. Future installer/acceptance helpers must stop reading the removed ExperimentalMode key and verify Alpha/stock-ACC settings explicitly. Do not reuse an old helper that writes ExperimentalMode or expects its toggle.

The upstream scheduler resynchronizes after stalls. Existing Navigator monotonic timestamp, calibration provenance, freshness and release guards are retained. No new steering factors, MADS, longitudinal coordination candidate, safety limits, OS/firmware update or device installation are included.

Validation and exact merge receipt are recorded in the separate local research report. Configured Mac builds and synthetic/controller checks establish local compatibility; unchanged-child prior replay evidence is retained, not presented as a fresh whole-corpus run or ARM/vehicle/road proof.
