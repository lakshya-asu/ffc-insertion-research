#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ $# != 1 || -e $1 ]]; then echo 'Usage: run_inference_checks.sh FRESH_OUTPUT_DIRECTORY' >&2; exit 2; fi
for file in config/mounted-model-rig-v1.json outputs/macro-dinov3-test-001/sensor/0000-macro.png outputs/macro-dinov3-model-002/member0.pt third_party/dinov3_hf/model.safetensors; do
  test -f "$file"
done
mkdir -p "$1"
result_dir=$(realpath "$1")
for mode in normal slow; do
  mkdir "$result_dir/$mode"
  arguments=()
  if [[ $mode == slow ]]; then arguments+=(--slow); fi
  docker run --rm --gpus all --network none --shm-size=1g \
    -e ROS_DOMAIN_ID=75 -e FASTRTPS_DEFAULT_PROFILES_FILE=/config/fastdds-camera.xml \
    -v "$PWD/config/fastdds-camera.xml:/config/fastdds-camera.xml:ro" \
    -v "$PWD/config/mounted-model-rig-v1.json:/rig/camera.json:ro" \
    -v "$PWD/outputs/macro-dinov3-model-002:/model:ro" \
    -v "$PWD/third_party/dinov3_hf:/backbone:ro" \
    -v "$PWD/outputs/macro-dinov3-test-001/sensor/0000-macro.png:/input/rgb.png:ro" \
    -v "$PWD/scripts/test_ros2_inference.py:/test/probe.py:ro" \
    -v "$result_dir/$mode:/output" \
    ffc-ros2:jazzy-dinov3 python3 /test/probe.py "${arguments[@]}" > "$result_dir/$mode/log.txt" 2>&1
  test -s "$result_dir/$mode/report.json"
done
echo "Normal and delayed inference checks passed: $result_dir"
