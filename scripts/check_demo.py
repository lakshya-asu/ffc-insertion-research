"""Read-only preflight for the currently running demo, not insertion qualification."""

import hashlib
import json
import time
from pathlib import Path
from urllib.request import urlopen

root = Path(__file__).resolve().parents[1]
frozen = json.loads((root / "outputs/macro-dinov3-model-002/frozen.json").read_text())
with urlopen("http://127.0.0.1:8766/api/perception", timeout=5) as response:
    frame = json.load(response)
assert frame["model_sha256"] == frozen["model_sha256"], "Unexpected live model"
assert frame["motion_permitted"] is False
age = time.time() - frame["acquired_at"]
assert 0 <= age < 3, f"No recent feature frame ({age:.1f}s old); use recorded fallback"
assert frame["image"].startswith("data:image/jpeg;base64,")
recording = root / "docs/demo/live-perception.mp4"
assert recording.stat().st_size > 10000
print(f"Live matched feature frame: {age:.2f}s old. Continuous 500ms freshness is NOT implied.")
print("Model:", frame["model_sha256"])
print("Rig:", frame["rig_calibration_sha256"])
print("Fallback video SHA-256:", hashlib.sha256(recording.read_bytes()).hexdigest())
print("Demo preflight passed. Motion remains disabled.")
