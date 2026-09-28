"""Offline geometric containment audit: mini-end labels must lie inside its projected envelope.

This is supervision QA, never an inference crop or runtime target.
"""

import argparse
import itertools
import json
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]


def rotation(axis, degrees):
    a = np.deg2rad(degrees)
    c, s = np.cos(a), np.sin(a)
    return (
        np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        if axis == "z"
        else np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    )


def projected_envelope(row, cfg, calibration):
    pose = row["offline_pose"]
    board_r = rotation("z", pose["yaw_deg"])
    cable_r = rotation("z", pose["cable_yaw_deg"]) @ rotation("x", pose["cable_roll_deg"])
    tip = np.array([0.0333 + pose["gap_m"], -0.0008 + pose["tip_dy_m"], 0.0022 + pose["tip_dz_m"]])
    spec = cfg["cable"]
    width = spec["mini_width_mm"] / 1000
    length = max(spec["assumed_support_length_mm"], spec["assumed_exposed_length_mm"]) / 1000
    # Conservative Z bounds enclose body, thin pads and stiffener, not a physical clearance claim.
    thickness = spec["assumed_header_thickness_mm"] / 1000
    corners = np.array(list(itertools.product([0, length], [-width / 2, width / 2], [-thickness, thickness])))
    world = (corners @ cable_r.T + tip) @ board_r.T + np.array(cfg["board_translation_m"]) + pose["shift_m"]
    camera = next(c for c in cfg["cameras"] if c["id"] == row["camera_id"])
    eye = np.array(camera["eye_m"])
    forward = np.array(camera["target_m"]) - eye
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0, 0, 1])
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    delta = world - eye
    depth = delta @ forward
    assert depth.min() > 0
    k = np.array(calibration[row["camera_id"]]["K"])
    u = k[0, 0] * (delta @ right) / depth + k[0, 2]
    v = k[1, 2] - k[1, 1] * (delta @ up) / depth
    x0, y0, _, _ = calibration[row["camera_id"]]["crop_xyxy"]
    return [
        float(u.min() - x0 - 3),
        float(v.min() - y0 - 3),
        float(u.max() - x0 + 3),
        float(v.max() - y0 + 3),
    ]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("data", type=Path)
    a = p.parse_args()
    cfg = json.loads((ROOT / "config/pi-zero-task.json").read_text())
    calibration = json.loads((a.data / "sensor/calibration.json").read_text())
    rows = json.loads((a.data / "offline/labels.json").read_text())["frames"]
    total = 0
    for row in rows:
        mask = np.array(Image.open(a.data / "offline" / row["file"]))
        y, x = np.where(mask == 3)
        box = projected_envelope(row, cfg, calibration)
        outside = (x < box[0]) | (x > box[2]) | (y < box[1]) | (y > box[3])
        if outside.any():
            raise ValueError(
                f"Mini-end label outside projected envelope: {row['file']}, {outside.sum()} pixels"
            )
        total += len(x)
    print(
        json.dumps(
            {
                "valid": True,
                "images": len(rows),
                "mini_end_pixels_checked": total,
                "scope": "offline supervision containment only; not physical geometry qualification",
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
