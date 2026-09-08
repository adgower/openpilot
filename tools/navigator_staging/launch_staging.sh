#!/usr/bin/env bash
# Boot entry only AFTER separate OS/build/firmware/rollback review. Never called by build_only.sh.
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)
mode=${1:-shadow}
[[ "$mode" == a2 || "$mode" == shadow ]] || { echo 'Only A2/shadow staging permitted'; exit 2; }
[[ "$(cat /VERSION)" == 19.7 ]] || { echo 'STOP: AGNOS mismatch; refusing automatic OS transition'; exit 2; }
[[ -x "$root/.staging-venv/bin/python" ]] || { echo 'Missing isolated build environment'; exit 2; }
export PATH="$root/.staging-venv/bin:$PATH"
export PYTHONPATH="$root:$root/msgq_repo:$root/opendbc_repo:$root/rednose_repo:$root/teleoprtc_repo:$root/tinygrad_repo:$root/panda"
python "$root/tools/navigator_staging/verify_checkout.py" --root "$root" --artifacts --match-result
python -c 'import json,sys; assert json.load(open(sys.argv[1]))["target_built"] is True' "$root/.staging-results/build-result.json"
# Freeze the reviewed artifacts: the stock launcher otherwise rebuilds after verification.
# Refuse overlay installation paths that could replace the validated checkout.
[[ ! -e "$root/.overlay_init" ]] || { echo 'STOP: staging overlay state requires separate review'; exit 2; }
touch "$root/prebuilt"
export NAVIGATOR_STAGING_SCONS_CACHE="$root/.staging-cache/scons"
export NAVIGATOR_A3_MODE="$mode"
export NAVIGATOR_A3_PROFILE=expedition-provisional-v1
export AGNOS_VERSION=19.7
cd "$root"
exec ./launch_chffrplus.sh
