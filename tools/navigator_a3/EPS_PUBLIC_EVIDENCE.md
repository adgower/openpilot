# Navigator EPS public evidence audit

Date: 2026-09-08. Read-only public research through BrowserOS/BrowserClaw, session `navigator eps evidence`, two task-owned tabs (857 and 859). No downloads of firmware, diagnostic requests, device changes, purchases, account changes, or messages. No executable/calibration artifact was obtained. Search absence below means not located in this bounded search, not proven nonexistent.

## Result

`NL14-14D003-AE` is supported as an **ECU software-number response identity when associated with DID F188**, not a unique Navigator hardware identity or complete executable/calibration manifest. `NL14-3F964-AB` in a separate F110 response must remain a separate identity. Public third-party diagnostic mappings label F110 as a diagnostic specification/database reference, not a hardware-number DID. No authoritative Ford definition specifically for this PSCM's F110 payload was located. Neither identifier establishes the July 2026 article's precise filter table, address, or inactive/driver-handoff behavior on this Navigator.

## Primary repository evidence

Inspected immutable donor revision `3210caa02d9e09b46d08be490f689edb23b22b20` using git show. Its `opendbc` is a symlink to `opendbc_repo/opendbc`, so the canonical revision paths are below; the current checkout differs and was not substituted for pinned evidence.

- [Ford fingerprints](https://github.com/BluePilotDev/bluepilot/blob/3210caa02d9e09b46d08be490f689edb23b22b20/opendbc_repo/opendbc/car/ford/fingerprints.py): lines 107–109 list `NL14-14D003-AE` with ten trailing NUL bytes under `CAR.FORD_EXPEDITION_MK4`, `(Ecu.eps, 0x730, None)`. Lines 244–245 also contain this value under Ranger (with `NL14-14D003-AC` adjacent). This is accepted fingerprint data across platforms, not a one-to-one physical-rack lookup.
- [UDS identifiers](https://github.com/BluePilotDev/bluepilot/blob/3210caa02d9e09b46d08be490f689edb23b22b20/opendbc_repo/opendbc/car/uds.py): F188 is `VEHICLE_MANUFACTURER_ECU_SOFTWARE_NUMBER`; F189 is the software version-number DID; F191 is `VEHICLE_MANUFACTURER_ECU_HARDWARE_NUMBER`; F181/F182 separately identify application software/data. F188's name does not say it enumerates every application/data block. Do not rename it simply “calibration version” or infer the hardware part.

The canonical BluePilotDev raw files at this exact revision were independently opened and read in BrowserOS by the root reviewer. The current checkout was not substituted for this pinned evidence.

## F110: useful clue, not manufacturer confirmation

[tonesto7/fordpass-scriptable, module_did_desc.json](https://github.com/tonesto7/fordpass-scriptable/blob/798b5d13d54c4f7a8f35464ca001dec08c28778b/module_did_desc.json) was opened in BrowserOS and its raw main file read. GitHub showed the file's latest commit as `798b5d13d54c4f7a8f35464ca001dec08c28778b` (May 6, 2022).

Its implementation maps F110 to **Subsystem Specific Diagnostic Specification Part Number**, F111 to ECU Core Assembly Number, and F188 to Vehicle Manufacturer ECU Software Number. This is primary evidence of what that independent FordPass tool calls the identifiers; it is not an official Ford PSCM diagnostic specification. Supporting search results from CyanLabs called F110 an “On-line Diagnostic Database Reference Number,” but were not promoted to authoritative module semantics.

The supplied response contains `f1104453...`: the bytes `44 53` decode to `DS`. Preserve this exact raw prefix and the DID, padding, request and response context. A plausible diagnostic-specification interpretation is consistent with the independent tool's mapping, but **do not claim it proved** that `NL14-3F964-AB` is the firmware executable, hardware, or calibration. It certainly should not be combined with F188 into a single invented version string. Scanner query brand alone is not the responder's manufacturer identity.

Raw source read archive: `/Users/alex/.browseros/tool-output/read-1788907965251-7af718ec-fd1a-4962-b41d-bd5c8c70d33c.md` (F110 at line145; F188 at line216).

## What BluePilot's July 15 article supplies

[BluePilot 7.0 – The Return of Angle Control](https://bluepilot.dev/announcements/?post=bluepilot-7-0-the-return-of-angle-control-it-wasnt-the-models-fault) was read live. It says the authors reverse engineered Ford VBFs using Comma3x logs across CAN-FD F-150, Lightning, Mach-E, Expedition, Ranger and Escape. Navigator is not named in that list.

The article claims:

- A speed-indexed internal curvature low-pass filter at executable address `0x101B0B60` with time constants 328ms at 5km/h, 195ms at 20km/h, 95ms at 40km/h and 35ms at 100km/h. Its 5T settling figures describe the claimed active signal path; they do not specify what inactive control or driver override does to filter state.
- A polynomial `c0 + c1*x + 0.5*c2*x^2 + (1/6)*c3*x^3`, evaluated at a claimed reference distance about 1.4m at highway speed.
- Angle mode sets c2 and c3 to zero and derives c1 from desired curvature times speed times a vehicle-specific gain. It characterizes angle as avoiding the curvature memory.
- Vehicle dynamics and factory configuration affect the response, with model-level defaults and tuning factors rather than a universal setting.

This is the researchers' own account, but it does **not** publish an exact VBF filename/revision, executable hash, calibration block hash, vehicle identity mapping for the analyzed file, reproducible disassembly, symbol/function context, or logged experiment tying the table to this Navigator. The article contains neither NL14 identifier. The supplied link resolves to an announcements feed containing several articles; unrelated later feed entries about torque-interceptor handoff are not proof for this article's angle-control state machine.

Do not carry the address into a different binary by assumption. Do not infer relative dominance of inputs from polynomial coefficients alone: contribution also depends on each input's units/magnitude. Do not infer immediate physical steering release, inactive-state clearing, torque-override thresholds, or re-engagement reset semantics from its active-mode “zero” description.

Full read archive: `/Users/alex/.browseros/tool-output/read-1788907808958-a181d84c-38e1-492f-bf40-3da32be32dee.md`; the relevant July 15 section begins at line60 and ends before the next article.

## Manufacturer-supported acquisition route

[Motorcraft diagnostic software information](https://www.motorcraftservice.com/Diagnostic/HelmSupport?country=USA&language=EN-US&categoryId=286&channelId=46) was read live. Ford states that diagnostic software downloads are free but use requires a purchased license/activation key. IDS and FJDS licensing includes calibration-file access; FDRS is the cloud-based system for some 2018+ vehicles, included with an IDS or FJDS license. This documents a legitimate acquisition route, not availability of a specific historical NL14 file or proof of vehicle applicability.

No exact Ford public manifest or VBF was located for the two identifiers. Exact quoted identifier searches returned a derivative fingerprint repository for F188 and no exact indexed result for `NL14-3F964-AB`. Additional searches combined NL14, PSCM, VBF and calibration, and searched Ford's site for F110 definitions; they did not provide the missing manufacturer artifact. Google-generated AI text was explicitly ignored. Marketplace listings were not used as firmware evidence. No filename-guessing download endpoint, authentication bypass, or purchase was attempted.

## Precise next missing artifact

The next useful input is a **read-only, existing Ford IDS/FDRS/FORScan module-identification/session export for this exact PSCM**, retaining the original DID-to-response association, and the **authorized software package manifest plus exact application and calibration/data VBF files matching that installed identity**. Include filenames/revisions and SHA-256 hashes, module hardware/assembly identifiers if present, and relevant as-built/configuration identity. A latest-update package alone may not match currently installed code. Do not flash or update merely to collect evidence.

That would permit checking the address against the exact executable, locating its calibration table and callers, and tracing transitions for inactive lateral mode, driver override, invalid/missing command, and re-engagement. To compare directly with BluePilot's account, its authors' analyzed binary/calibration filename/hash and function/disassembly evidence are also needed; none is published in the article inspected. This artifact request is distinct from the root task's rawlog extractor: logs can establish observed input/output timing but cannot by themselves name an unpublished internal table or prove an unobserved branch.
