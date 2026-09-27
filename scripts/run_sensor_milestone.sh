#!/usr/bin/env bash
# Reproduce static M1 capture, offline supervision, isolated inference and scoring.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_dir"
run_id="${1:?Pass a fresh run identifier}"
if [[ ! "$run_id" =~ ^[a-zA-Z0-9_-]+$ ]]; then exit 2; fi
base="outputs/$run_id"
if [[ -e "$base" ]]; then printf 'Use a fresh run identifier.\n' >&2; exit 2; fi
mkdir -p "$base/development" "$base/final"
docker compose run --rm experiment scripts/capture_fixed_cameras.py --output "/workspace/$base/development" > "$base/development/run.log" 2>&1
.venv/bin/python scripts/train_pixel_classifier.py "$base/development"
# Model is frozen before the final images are generated.
docker compose run --rm experiment scripts/capture_fixed_cameras.py --final-evaluation --evaluation-seed 92703 --output "/workspace/$base/final" > "$base/final/run.log" 2>&1
cp "$base/development/pixel-model.json" "$base/final/sensor/pixel-model.json"
scripts/run_perception_worker.sh "$base/final" predictions --require-model
.venv/bin/python scripts/evaluate_perception.py "$base/final" --predictions predictions
printf 'Artifacts: %s/final; evaluation is not physical qualification.\n' "$base"
