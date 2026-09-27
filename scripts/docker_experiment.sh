#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_dir"
task="${1:-smoke}"
if [ "$#" -gt 0 ]; then shift; fi
case "$task" in
  build) docker compose build ;;
  smoke) docker compose run --rm experiment scripts/isaac_workcell.py --exercise-tool "$@" ;;
  segments) docker compose run --rm experiment scripts/isaac_workcell.py --cable-model segments --exercise-tool --output /workspace/outputs/e002-segments "$@" ;;
  assembly) docker compose run --rm experiment scripts/isaac_workcell.py --cable-model segments --assembly --video --output /workspace/outputs/e003-assembly "$@" ;;
  direct) docker compose run --rm experiment scripts/isaac_workcell.py --assembly --strategy direct --output /workspace/outputs/direct-repeat "$@" ;;
  deformable) docker compose run --rm experiment scripts/isaac_workcell.py --cable-model shell --steps 1000 --output /workspace/outputs/native-repeat "$@" ;;
  cantilever) docker compose run --rm experiment scripts/isaac_workcell.py --cantilever --steps 4000 --output /workspace/outputs/segment-cantilever-repeat "$@" ;;
  native-cantilever) docker compose run --rm experiment scripts/isaac_workcell.py --cable-model shell --cantilever --config /workspace/config/cantilever-iterations255.json --steps 2000 --output /workspace/outputs/native-cantilever-repeat "$@" ;;
  grasp) docker compose run --rm experiment scripts/isaac_workcell.py --grasp-benchmark --config /workspace/config/grasp-benchmark.json --output /workspace/outputs/grasp-repeat "$@" ;;
  verify) docker compose run --rm experiment scripts/check_artifacts.py "$@" ;;
  tests) docker compose run --rm --entrypoint /workspace/scripts/usd_python.sh experiment -m pytest -q "$@" ;;
  report) docker compose run --rm experiment scripts/render_report.py "$@" ;;
  shell) docker compose run --rm --entrypoint /bin/bash experiment ;;
  *) printf 'Usage: %s {build|smoke|segments|assembly|direct|deformable|cantilever|native-cantilever|grasp|verify|tests|report|shell} [arguments]\n' "$0" >&2; exit 2 ;;
esac
