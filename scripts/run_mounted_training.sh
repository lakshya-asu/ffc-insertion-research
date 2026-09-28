#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ $# != 2 ]]; then echo 'Usage: run_mounted_training.sh VALIDATED_DATA_DIR NEW_MODEL_DIR' >&2; exit 2; fi
data_dir=$(realpath "$1")
test -s "$data_dir/validation.json"
if [[ -e $2 ]]; then echo 'Fresh output required' >&2; exit 2; fi
mkdir -p "$2"
model_dir=$(realpath "$2")
docker run --rm --gpus all --network none --user "$(id -u):$(id -g)" \
 -e PYTHONPATH=/code:/opt/perception \
 -v "$data_dir:/data:ro" -v "$model_dir:/results" \
 -v "$PWD/third_party/dinov3_hf:/backbone:ro" \
 -v "$PWD/scripts/train_mounted_macro.py:/code/train.py:ro" \
 -v "$PWD/src/ffc/zero_region_model.py:/code/zero_region_model.py:ro" \
 -v "$PWD/src/ffc/entrance_feature_model.py:/code/entrance_feature_model.py:ro" \
 -v "$PWD/src/ffc/dinov3_features.py:/code/dinov3_features.py:ro" \
 ffc-ros2:jazzy-dinov3 python3 /code/train.py --data /data --output /results
python3 - "$data_dir" "$model_dir" <<'PY'
import hashlib
import json
import sys
from pathlib import Path

data, model = map(Path, sys.argv[1:])
report = json.loads((model / 'training-report.json').read_text())
source = model / 'source'
source.mkdir()
files = ['scripts/train_mounted_macro.py', 'src/ffc/entrance_feature_model.py',
         'src/ffc/zero_region_model.py', 'src/ffc/dinov3_features.py',
         'ros2_ws/src/ffc_cell/ffc_cell/cv_frontend.py', 'config/macro-training-v1.json']
for filename in files:
    (source / Path(filename).name).write_bytes(Path(filename).read_bytes())
(model / 'sensor-contract.json').write_bytes((data / 'sensor/camera.json').read_bytes())
(model / 'frozen.json').write_text(json.dumps({
    'model_sha256': hashlib.sha256((model / 'member0.pt').read_bytes()).hexdigest(),
    'training_report': report,
    'source_sha256': {f: hashlib.sha256(Path(f).read_bytes()).hexdigest() for f in files},
}, indent=2))
PY
