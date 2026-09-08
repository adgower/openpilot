#!/usr/bin/env bash

# Only locally materialized staging packages take this guarded boot path.
if [[ -f .staging-package.json ]]; then
  exec ./tools/navigator_staging/launch_staging.sh shadow
fi
exec ./launch_chffrplus.sh
