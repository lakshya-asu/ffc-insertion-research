#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ $# != 3 ]]; then
  echo 'Usage: scripts/run_zero_inference.sh SENSOR_DIRECTORY MODEL_DIRECTORY OUTPUT_DIRECTORY' >&2
  exit 2
fi
sensor_dir=$(realpath "$1")
model_dir=$(realpath "$2")
mkdir -p "$3"
result_dir=$(realpath "$3")
docker run --rm --gpus all --network none --read-only --cap-drop ALL --security-opt no-new-privileges \
  --user "$(id -u):$(id -g)" --tmpfs /tmp:rw,noexec,nosuid,size=512m \
  -e PYTHONPATH=/code -e XFORMERS_DISABLED=1 \
  -v "$sensor_dir:/sensor:ro" -v "$model_dir:/models:ro" -v "$result_dir:/results:rw" \
  -v "$PWD/third_party/dinov2:/backbone:ro" \
  -v "$PWD/scripts/zero_region_worker.py:/code/zero_region_worker.py:ro" \
  -v "$PWD/src/ffc/zero_region_model.py:/code/zero_region_model.py:ro" \
  -v "$PWD/src/ffc/camera_observations.py:/code/camera_observations.py:ro" \
  ffc-perception:2.8.0-cu128 /code/zero_region_worker.py
