#!/usr/bin/env bash
# Historical privileged-state baseline only. Not the sensor-driven skill pipeline.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_dir"
if [[ "${1:-}" != "--historical-baseline" ]]; then
  printf 'This runner uses privileged simulator state. To reproduce historical geometry checks, pass --historical-baseline explicitly. See SENSOR_FIRST.md for the new milestones.\n' >&2
  exit 2
fi
shift
run_id="${1:-$(date -u +%Y%m%dT%H%M%SZ)}"
if [[ ! "$run_id" =~ ^[a-zA-Z0-9_-]+$ ]]; then
  printf 'Run identifier must contain only letters, digits, underscores and hyphens.\n' >&2
  exit 2
fi
suite_dir="outputs/suite-$run_id"
if [[ -e "$suite_dir" ]]; then
  printf 'Output already exists: %s. Choose a fresh run identifier.\n' "$suite_dir" >&2
  exit 2
fi
mkdir -p "$suite_dir"
./scripts/docker_experiment.sh build > "$suite_dir/build.log" 2>&1
./scripts/docker_experiment.sh tests > "$suite_dir/tests.log" 2>&1
run_case() {
  local mode="$1" label="$2"
  shift 2
  local destination="outputs/$run_id-$label"
  mkdir -p "$destination"
  local experiment_failed=0
  if ./scripts/docker_experiment.sh "$mode" --output "/workspace/$destination" "$@" > "$destination/run.log" 2>&1; then
    printf '%s simulation PASS; independent audits pending\n' "$label" >> "$suite_dir/execution-status.txt"
  else
    printf '%s FAIL (see %s)\n' "$label" "$destination/run.log" >> "$suite_dir/execution-status.txt"
    experiment_failed=1
  fi
  if ! ./scripts/docker_experiment.sh verify "$destination" > "$destination/verification.log" 2>&1; then
    printf '%s artifact verification FAIL\n' "$label" >> "$suite_dir/execution-status.txt"
    return 1
  fi
  if [[ "$mode" == "direct" || "$mode" == "assembly" ]]; then
    for analysis in analyze_handling audit_grip audit_seating; do
      docker compose run --rm --entrypoint /workspace/scripts/usd_python.sh experiment "scripts/$analysis.py" "$destination" > "$destination/$analysis.log" 2>&1 || return 1
    done
    python3 - "$destination" <<'PY_AUDIT' || experiment_failed=1
import json
import sys
from pathlib import Path
audit = json.loads((Path(sys.argv[1]) / "seating-audit.json").read_text())
if not audit["geometric_task_qualified"]:
    raise SystemExit("Independent grip/full-tip seating audit did not qualify the task")
PY_AUDIT
  fi
  if [[ "$experiment_failed" == 0 ]]; then
    printf '%s final PASS including applicable independent audits\n' "$label" >> "$suite_dir/execution-status.txt"
  else
    printf '%s final FAIL (simulation or independent audit)\n' "$label" >> "$suite_dir/execution-status.txt"
  fi
  return "$experiment_failed"
}
# These are infrastructure gates. Strategy failures are measured research results;
# still run the other strategy so the comparison remains available.
run_case cantilever cantilever --config /workspace/config/selected-static.json --cantilever-length .15 --steps 16000
python3 - "$run_id" <<'PY_CHECK'
import json
import sys
from pathlib import Path
r = json.loads((Path("outputs") / (sys.argv[1] + "-cantilever") / "results.json").read_text())
if not r["cantilever"].get("static_shape_agreement_qualified", False):
    raise SystemExit("Cantilever failed the declared static-shape agreement and motion-bound gate")
PY_CHECK
run_case smoke smoke
handling_failed=0
run_case direct direct || handling_failed=1
run_case assembly fixture || handling_failed=1
./scripts/docker_experiment.sh report > "$suite_dir/report.log" 2>&1
printf 'Suite: %s\nVideos: %s/outputs/index.html\n' "$suite_dir" "$project_dir"
exit "$handling_failed"
