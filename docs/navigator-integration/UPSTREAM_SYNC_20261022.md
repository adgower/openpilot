# October 22 upstream sync: AGNOS 20

Merged official master `34d4241e19c983b048a806a17f3234ddd35de445` over the paired onset integration. Six incoming commits add AGNOS 20 and touch/scroll improvements. Child `fe59e503388a7c6593a7cd3117ac977d8894f0c5` and all Navigator steering code remain unchanged, including 1.20/0.97, geometry, original limits, both new onset bands, combined default-off options and explicit Expedition selection.

The matching launcher now requires AGNOS 20. The OS takes ownership of modem management; the repository modem process is removed. Use the official new manifest and matching candidate launcher. Do not transplant the previous installer’s AGNOS 19.9 launch_env override or preserve its old launcher hash as the target requirement.

Local minimal/full Mac and Panda builds pass. Full parent suite: 1,021 passed, 84 skipped, one expected failure and 281 subtests. Full child suite: 4,353 passed, 1,690 skipped and 24,673 subtests. Source preservation, upstream version/manifest contracts, scoped lint and independent merge review pass. These are Mac/source checks, not AGNOS 20 device proof.

Previous candidate 21325dccb/fe59e503 was published and staged, but not activated. Its isolated ARM build and 840 native tests passed on AGNOS 19.9. A scheduler harness assumption failed afterward; the corrected local harness requires both already-fixed schedulers to retain identical cadence. The followup was not run when the user requested latest master. Keep that stage held; its acceptance gate is not satisfied.

The last verified device remains db5842348/8ccc6cf0 on AGNOS 19.9. A fresh source/settings backup is verified on both device and Mac. Future deployment needs a reviewed A/B OS transition, matching source rollback, fresh AGNOS 20 build/runtime checks and offroad/powered stationary acceptance. No OS or firmware update, source activation, manual CAN or ECU request occurred during this merge.

Official release notes: https://github.com/commaai/openpilot/pull/39128
