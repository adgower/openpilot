#!/usr/bin/env bash
# Reviewed preparation only: this script is for a later explicitly authorized device build.
set -euo pipefail
[[ $# == 1 ]] || { echo 'Usage: build_only.sh SEPARATE_CHECKOUT'; exit 2; }
root=$(cd "$1" && pwd -P)
[[ "$root" == /data/navigator-staging/* ]] || { echo 'Requires /data/navigator-staging/<unique checkout>'; exit 2; }
[[ "$root" != "$(readlink -f /data/openpilot)" ]] || { echo 'Refusing booted checkout'; exit 2; }
[[ "$(uname -s)" == Linux && "$(uname -m)" == aarch64 && -f /AGNOS ]] || { echo 'Requires AGNOS device'; exit 2; }
[[ "$(cat /VERSION)" == 19.7 ]] || { echo 'STOP: requires AGNOS 19.7; does not update the OS'; exit 2; }
[[ "$(cat /data/params/d/IsOffroad)" == 1 ]] || { echo 'Requires offroad'; exit 2; }
if ps -eo args= | grep -E '[m]odeld.py|[d]monitoringmodeld.py|[s]elfdrive.modeld.modeld|[s]elfdrive.modeld.dmonitoringmodeld' >/dev/null; then
  echo 'STOP: model processes are running; inspect and stop separately before build'; exit 2
fi
command -v uv >/dev/null || { echo 'Missing uv; inventory and provision separately'; exit 2; }
python3 "$root/tools/navigator_staging/verify_checkout.py" --root "$root"
cd "$root"
export UV_PROJECT_ENVIRONMENT="$root/.staging-venv"
export UV_CACHE_DIR="$root/.staging-cache/uv"
export UV_PYTHON_INSTALL_DIR="$root/.staging-cache/python"
export NAVIGATOR_STAGING_SCONS_CACHE="$root/.staging-cache/scons"
# Never inherit custom firmware signing/activation settings or shared Python paths.
unset PYTHONPATH RELEASE CERT DEBUG NAVIGATOR_A3_MODE NAVIGATOR_A3_PROFILE
uv sync --frozen --extra tools --python 3.12
export PATH="$UV_PROJECT_ENVIRONMENT/bin:$PATH"
export PYTHONPATH="$root:$root/msgq_repo:$root/opendbc_repo:$root/rednose_repo:$root/teleoprtc_repo:$root/tinygrad_repo:$root/panda"
mkdir -p .staging-results
rm -f .staging-results/build-result.json
python --version > .staging-results/toolchain.txt
uv --version >> .staging-results/toolchain.txt
uv pip freeze >> .staging-results/toolchain.txt
for compiler in clang clang++ arm-none-eabi-gcc; do
  command -v "$compiler" >> .staging-results/toolchain.txt
  "$compiler" --version >> .staging-results/toolchain.txt
done
# Model compilation uses QCOM/Chestnut compute; hardware must be idle and powered.
# No launcher, manager, Panda instance, firmware-flashing entry point or reboot is called.
# Chestnut link_up uses USB control I/O; this build is not read-only.
scons --minimal -j2 2>&1 | tee .staging-results/scons.log
python tools/navigator_staging/verify_checkout.py --root "$root" --artifacts > .staging-results/build-result.json
printf 'Build finished. Review %s/.staging-results; boot selection is unchanged.\n' "$root"
