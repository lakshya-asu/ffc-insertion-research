#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ $# != 3 ]]; then echo 'Usage: run_feature_inference.sh SENSOR MODEL NEW_OUTPUT' >&2; exit 2; fi
sensor=$(realpath "$1")
model=$(realpath "$2")
if [[ -e $3 ]]; then echo 'Fresh output required' >&2; exit 2; fi
mkdir "$3"
result=$(realpath "$3")
docker run --rm --gpus all --network none --read-only --cap-drop ALL --security-opt no-new-privileges \
 --user "$(id -u):$(id -g)" --tmpfs /tmp:rw,noexec,nosuid,size=512m -e PYTHONPATH=/code \
 -v "$PWD/src/ffc/zero_region_model.py:/code/zero_region_model.py:ro" \
 -v "$PWD/src/ffc/entrance_feature_model.py:/code/entrance_feature_model.py:ro" \
 -v "$PWD/src/ffc/dinov3_features.py:/code/dinov3_features.py:ro" \
 -v "$PWD/third_party/dinov3_hf:/backbone:ro" -v "$sensor:/sensor:ro" -v "$model:/model:ro" \
 -v "$result:/results:rw" -v "$PWD/scripts/infer_entrance_features.py:/code/infer.py:ro" \
 ffc-foundation:4.57.6 /code/infer.py
