#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ $# != 1 || ! "$1" =~ ^[a-zA-Z0-9_-]+$ ]]; then
  echo 'Usage: scripts/run_pi_perception_experiment.sh FRESH_RUN_NAME' >&2
  exit 2
fi
run_dir="$PWD/outputs/$1"
if [[ -e "$run_dir" ]]; then echo 'Use a fresh run name' >&2; exit 2; fi
docker image inspect ffc-perception:2.8.0-cu128 >/dev/null
mkdir -p "$run_dir/scene" "$run_dir/development" "$run_dir/model" "$run_dir/test"
container_run="/workspace/outputs/$1"
docker compose run --rm experiment scripts/render_raspberry_pi.py --refined --orbit-frames 48 --output "$container_run/scene"
docker compose run --rm experiment scripts/capture_pi_perception.py --split development --stage "$container_run/scene/raspberry-pi-workcell.usda" --output "$container_run/development"
.venv/bin/python scripts/validate_pi_dataset.py "$run_dir/development"
docker run --rm --gpus all --network none --user "$(id -u):$(id -g)" \
  -e PYTHONPATH=/code -e CUBLAS_WORKSPACE_CONFIG=:4096:8 \
  -v "$run_dir/development:/data:ro" -v "$run_dir/model:/model" \
  -v "$PWD/scripts/train_pi_perception.py:/code/train_pi_perception.py:ro" \
  -v "$PWD/src/ffc/pi_perception_model.py:/code/pi_perception_model.py:ro" \
  ffc-perception:2.8.0-cu128 /code/train_pi_perception.py --data /data --output /model --epochs 40
.venv/bin/python - "$run_dir/model" <<'PY'
import hashlib,json,sys,time
from pathlib import Path
p=Path(sys.argv[1])
(p/'freeze.json').write_text(json.dumps({'model_sha256':hashlib.sha256((p/'model.pt').read_bytes()).hexdigest(),
                                      'frozen_unix_s':time.time(),'selection':'validation only'},indent=2))
PY
docker compose run --rm experiment scripts/capture_pi_perception.py --split test --stage "$container_run/scene/raspberry-pi-workcell.usda" --output "$container_run/test"
.venv/bin/python scripts/validate_pi_dataset.py "$run_dir/test"
scripts/run_pi_inference.sh "$run_dir/test/sensor" "$run_dir/model" "$run_dir/predictions"
scripts/run_pi_inference.sh "$run_dir/test/sensor" "$run_dir/model" "$run_dir/replay"
.venv/bin/python scripts/check_pi_replay.py "$run_dir/predictions" "$run_dir/replay"
.venv/bin/python scripts/evaluate_pi_perception.py --data "$run_dir/test" --predictions "$run_dir/predictions" --output "$run_dir/predictions/evaluation.json"
