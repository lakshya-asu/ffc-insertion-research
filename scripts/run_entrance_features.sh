#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ $# != 1 ]]; then echo 'Usage: run_entrance_features.sh NEW_RUN_PREFIX' >&2; exit 2; fi
prefix=$1
for suffix in dev model test predictions; do
 if [[ -e outputs/$prefix-$suffix ]]; then echo "Existing run: $prefix-$suffix" >&2; exit 2; fi
done
code=(-v "$PWD/src/ffc/zero_region_model.py:/code/zero_region_model.py:ro"
 -v "$PWD/src/ffc/entrance_feature_model.py:/code/entrance_feature_model.py:ro"
 -v "$PWD/src/ffc/dinov3_features.py:/code/dinov3_features.py:ro")
docker compose run --rm experiment scripts/capture_entrance_features.py --split development \
 --output "/workspace/outputs/$prefix-dev" > "outputs/$prefix-dev.log" 2>&1
.venv/bin/python scripts/audit_feature_dataset.py "outputs/$prefix-dev"
cp scripts/capture_entrance_features.py "outputs/$prefix-dev/capture-source.py"
cp src/ffc/entrance_features.py "outputs/$prefix-dev/geometry-source.py"
mkdir "outputs/$prefix-model"
docker run --rm --gpus all --network none --user "$(id -u):$(id -g)" \
 -e PYTHONPATH=/code "${code[@]}" \
 -v "$PWD/third_party/dinov3_hf:/backbone:ro" \
 -v "$PWD/outputs/$prefix-dev:/data:ro" -v "$PWD/outputs/$prefix-model:/results:rw" \
 -v "$PWD/scripts/train_entrance_features.py:/code/train.py:ro" \
 ffc-foundation:4.57.6 /code/train.py --data /data --output /results > "outputs/$prefix-training.log" 2>&1
cp scripts/train_entrance_features.py "outputs/$prefix-model/training-source.py"
# Freeze model before generating independent test scenes.
test -s "outputs/$prefix-model/training-report.json"
sha256sum "outputs/$prefix-model/member0.pt" > "outputs/$prefix-model/frozen.sha256"
docker compose run --rm experiment scripts/capture_entrance_features.py --split test \
 --output "/workspace/outputs/$prefix-test" > "outputs/$prefix-test.log" 2>&1
.venv/bin/python scripts/audit_feature_dataset.py "outputs/$prefix-test" --other "outputs/$prefix-dev"
mkdir "outputs/$prefix-predictions"
# The inference container sees sensor files and weights only, never offline data or USD.
docker run --rm --gpus all --network none --read-only --cap-drop ALL --security-opt no-new-privileges \
 --user "$(id -u):$(id -g)" --tmpfs /tmp:rw,noexec,nosuid,size=512m -e PYTHONPATH=/code \
 "${code[@]}" -v "$PWD/third_party/dinov3_hf:/backbone:ro" \
 -v "$PWD/outputs/$prefix-test/sensor:/sensor:ro" -v "$PWD/outputs/$prefix-model:/model:ro" \
 -v "$PWD/outputs/$prefix-predictions:/results:rw" \
 -v "$PWD/scripts/infer_entrance_features.py:/code/infer.py:ro" \
 ffc-foundation:4.57.6 /code/infer.py > "outputs/$prefix-inference.log" 2>&1
sha256sum -c "outputs/$prefix-model/frozen.sha256"
.venv/bin/python scripts/evaluate_entrance_features.py "outputs/$prefix-test" \
 "outputs/$prefix-predictions" "outputs/$prefix-evaluation.json" > "outputs/$prefix-evaluation.log"
