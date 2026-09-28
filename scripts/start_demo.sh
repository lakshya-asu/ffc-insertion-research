#!/usr/bin/env bash
# Restart the reviewed demo on this lab host. No actuator service is launched.
set -euo pipefail
cd "$(dirname "$0")/.."
for asset in outputs/mount-review-003/mounted-workcell.usda outputs/macro-dinov3-model-002/member0.pt third_party/dinov3_hf/model.safetensors config/mounted-model-rig-v1.json; do
  if [[ ! -s "$asset" ]]; then echo "Missing required demo asset: $asset" >&2; exit 1; fi
done
docker compose -f compose.yaml -f compose.ros2.yaml -f compose.inference.yaml up -d perception inference feature_view
systemctl --user restart ffc-lab-live.service
if ! systemctl --user is-active --quiet ffc-lab-preview.service; then
  rm -f outputs/lab-live/stop-preview
  if systemctl --user cat ffc-lab-preview.service >/dev/null 2>&1; then
    systemctl --user restart ffc-lab-preview.service
  else
    systemd-run --user --collect --unit=ffc-lab-preview --working-directory="$PWD" \
      /usr/bin/docker compose -f compose.yaml -f compose.ros2.yaml run --rm experiment
  fi
fi
echo 'Demo starting. Open http://127.0.0.1:8766/demo/ or the published demo page.'
echo 'Allow camera/model startup, then run: python3 scripts/check_demo.py'
