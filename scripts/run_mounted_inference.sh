#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ $# != 3 ]]; then echo 'Usage: run_mounted_inference.sh SENSOR_DIR MODEL_DIR NEW_OUTPUT_DIR' >&2; exit 2; fi
sensor_dir=$(realpath "$1")
model_dir=$(realpath "$2")
if [[ -e $3 ]]; then echo 'Fresh output required' >&2; exit 2; fi
mkdir -p "$3"
result_dir=$(realpath "$3")
docker run --rm --gpus all --network none --user "$(id -u):$(id -g)" \
 -e PYTHONPATH=/code:/opt/perception \
 -v "$sensor_dir:/sensor:ro" -v "$model_dir:/model:ro" -v "$result_dir:/results" \
 -v "$PWD/third_party/dinov3_hf:/backbone:ro" \
 -v "$PWD/scripts/infer_mounted_macro.py:/code/infer.py:ro" \
 -v "$PWD/src/ffc/zero_region_model.py:/code/zero_region_model.py:ro" \
 -v "$PWD/src/ffc/entrance_feature_model.py:/code/entrance_feature_model.py:ro" \
 -v "$PWD/src/ffc/dinov3_features.py:/code/dinov3_features.py:ro" \
 ffc-ros2:jazzy-dinov3 python3 /code/infer.py
