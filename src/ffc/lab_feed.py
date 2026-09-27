"""Publish rendered camera snapshots for human observation, never for robot control."""

import json
import os
import time
from pathlib import Path

from PIL import Image


def publish_snapshot(rgb, directory, camera, sequence, description, source):
    directory = Path(directory)
    if camera not in {"workcell", "desk", "board"}:
        raise ValueError("Unknown lab camera")
    directory.mkdir(parents=True, exist_ok=True)
    tmp = directory / (camera + ".jpg.tmp")
    Image.fromarray(rgb).save(tmp, format="JPEG", quality=88)
    os.replace(tmp, directory / (camera + ".jpg"))
    metadata = {
        "captured_at": time.time(),
        "sequence": sequence,
        "source": source,
        "description": description,
        "camera": camera,
        "width": rgb.shape[1],
        "height": rgb.shape[0],
        "motion_permitted": False,
    }
    tmp = directory / (camera + ".json.tmp")
    tmp.write_text(json.dumps(metadata))
    os.replace(tmp, directory / (camera + ".json"))
