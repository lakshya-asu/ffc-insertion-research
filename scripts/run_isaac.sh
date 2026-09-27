#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_dir"
export OMNI_KIT_ACCEPT_EULA=YES
exec .venv-isaac/bin/python scripts/isaac_workcell.py "$@"
