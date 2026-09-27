#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ $# != 3 ]]; then
  echo 'Usage: scripts/run_pi_inference.sh SENSOR_DIRECTORY MODEL_DIRECTORY OUTPUT_DIRECTORY' >&2
  exit 2
fi
sensor_dir=$(realpath "$1")
model_dir=$(realpath "$2")
mkdir -p "$3"
result_dir=$(realpath "$3")
if [[ -e "$result_dir/predictions.json" ]]; then
  echo 'Use a fresh output directory' >&2
  exit 2
fi
docker run --rm --network none --read-only --cap-drop ALL --security-opt no-new-privileges \
  --user "$(id -u):$(id -g)" --tmpfs /tmp:rw,noexec,nosuid,size=256m \
  -e NVIDIA_VISIBLE_DEVICES=void -e PYTHONPATH=/code \
  -v "$sensor_dir:/sensor:ro" -v "$model_dir:/model:ro" -v "$result_dir:/results:rw" \
  -v "$PWD/scripts/pi_perception_worker.py:/code/pi_perception_worker.py:ro" \
  -v "$PWD/src/ffc/pi_perception_model.py:/code/pi_perception_model.py:ro" \
  -v "$PWD/src/ffc/camera_observations.py:/code/camera_observations.py:ro" \
  ffc-perception:2.8.0-cu128 /code/pi_perception_worker.py
