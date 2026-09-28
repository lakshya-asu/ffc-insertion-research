#!/usr/bin/env bash
# Isaac-only runner; a process exit code alone is not a successful experiment.
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ $# -lt 1 || $# -gt 3 ]]; then
  echo 'Usage: run_isaac_joint_probe.sh NEW_RUN_NAME [JOINT_1_TO_7] [probe|cancel]' >&2
  exit 2
fi
run_name=$1
joint=${2:-7}
probe_case=${3:-probe}
if [[ ! $run_name =~ ^[a-zA-Z0-9_-]+$ || ! $joint =~ ^[1-7]$ || ! $probe_case =~ ^(probe|cancel)$ ]]; then
  echo 'Invalid run name, joint or case' >&2; exit 2
fi
if [[ -e outputs/$run_name || -e outputs/$run_name.log ]]; then
  echo 'Fresh run name required' >&2; exit 2
fi
docker compose -f compose.yaml run --rm experiment scripts/isaac_joint_probe.py \
  --joint "$joint" --case "$probe_case" --output "/workspace/outputs/$run_name" \
  > "outputs/$run_name.log" 2>&1
python3 - "$run_name" <<'PY'
import json
import sys
from pathlib import Path
path=Path('outputs')/sys.argv[1]/'results.json'
if not path.is_file():
    raise SystemExit('Missing completion report; inspect retained log')
report=json.loads(path.read_text())
if report.get('status')!='PASS':
    raise SystemExit('Isaac experiment failed: '+report.get('reason','unknown reason'))
print(json.dumps(report,indent=2))
PY
