#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ $# != 2 ]]; then
 echo 'Usage: train_zero_reliable.sh DEVELOPMENT_DIRECTORY NEW_MODEL_DIRECTORY' >&2
 exit 2
fi
data_dir=$(realpath "$1")
.venv/bin/python scripts/validate_zero_dataset.py "$data_dir"
mkdir "$2"
model_dir=$(realpath "$2")
docker run --rm --gpus all --network none --user "$(id -u):$(id -g)" \
 -e PYTHONPATH=/code -e XFORMERS_DISABLED=1 \
 -v "$data_dir:/data:ro" -v "$model_dir:/results:rw" -v "$PWD/third_party/dinov2:/backbone:ro" \
 -v "$PWD/scripts/train_zero_reliable.py:/code/train_zero_reliable.py:ro" \
 -v "$PWD/src/ffc/zero_region_model.py:/code/zero_region_model.py:ro" \
 ffc-perception:2.8.0-cu128 /code/train_zero_reliable.py --data /data --output /results
test -f "$model_dir/training-report.json"
