#!/usr/bin/env bash
# Use Isaac's own USD build for headless audits without starting SimulationApp.
set -euo pipefail
ffc_usd_roots=(/isaac-sim/extscache/omni.usd.libs-*)
if [[ ${#ffc_usd_roots[@]} -ne 1 || ! -d "${ffc_usd_roots[0]}/pxr" ]]; then
  printf 'Expected one bundled Isaac USD runtime; run this inside the pinned Docker image.\n' >&2
  exit 2
fi
export PYTHONPATH="${ffc_usd_roots[0]}:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="${ffc_usd_roots[0]}/bin:${LD_LIBRARY_PATH:-}"
exec /isaac-sim/python.sh "$@"
