#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_dir"
run_dir="$(realpath "${1:?Pass the capture output directory}")"
result_name="${2:-predictions}"
if [[ "${3:-}" == "--require-model" && ! -f "$run_dir/sensor/pixel-model.json" ]]; then
  printf 'Required frozen pixel model is missing; worker not launched.\n' >&2
  exit 2
fi
if [[ ! "$result_name" =~ ^[a-zA-Z0-9_-]+$ ]]; then exit 2; fi
if [[ -e "$run_dir/$result_name" ]]; then printf 'Use a fresh predictions directory.\n' >&2; exit 2; fi
mkdir -p "$run_dir/$result_name" "$run_dir/worker-code"
cp src/ffc/camera_observations.py src/ffc/cable_perception.py scripts/perception_worker.py "$run_dir/worker-code/"
docker run --rm --network none --read-only --cap-drop ALL --security-opt no-new-privileges \
  --user "$(id -u):$(id -g)" --group-add 1234 --tmpfs /tmp:rw,nosuid,nodev,size=64m \
  -e NVIDIA_VISIBLE_DEVICES=void -e PYTHONDONTWRITEBYTECODE=1 -e PYTHONUNBUFFERED=1 \
  -v "$run_dir/sensor:/sensor:ro" -v "$run_dir/worker-code:/code:ro" \
  -v "$run_dir/$result_name:/results:rw" \
  --entrypoint /isaac-sim/python.sh ffc-isaac:6.1.0 /code/perception_worker.py
