#!/usr/bin/env bash
# Run only after the later read-only inventory is authorized.
set -euo pipefail
cat /VERSION
readlink -f /data/openpilot
readlink -f /data/pythonpath || true
for repo in /data/openpilot /data/openpilot/opendbc_repo /data/openpilot/panda; do
  git -C "$repo" rev-parse HEAD
  git -C "$repo" status --short
done
git -C /data/openpilot submodule status
cat /data/params/d/IsOffroad
df -h /data
command -v uv || true
python3 --version
systemctl cat comma.service 2>/dev/null || true
systemctl cat openpilot.service 2>/dev/null || true
ls -l /data/openpilot/prebuilt /data/openpilot/.overlay_init /data/safe_staging/finalized/.overlay_consistent 2>/dev/null || true
# Source artifacts only: do not instantiate Panda or invoke its flashing tooling.
sha256sum /data/openpilot/panda/board/obj/panda_h7.bin.signed 2>/dev/null || true
