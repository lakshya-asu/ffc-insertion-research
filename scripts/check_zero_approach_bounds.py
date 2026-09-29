"""Offline sampled tool/board bounds. This does not authorize arm motion."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from pxr import Sdf, Usd, UsdGeom
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]


def check(scene, output):
    layer = Sdf.Layer.CreateAnonymous()
    layer.ImportFromString(scene.read_text().replace("/workspace/", str(ROOT) + "/"))
    stage = Usd.Stage.Open(layer)
    cache = UsdGeom.BBoxCache(stage.GetEndTimeCode(), ["default", "render"])
    tool, obstacles = [], []
    for prim in stage.Traverse():
        if not (prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Cube)):
            continue
        path = str(prim.GetPath())
        if not path.startswith(("/World/Tool/", "/World/Zero2W/", "/World/ZeroFixture/", "/World/Slot/")):
            continue
        bound = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        lo, hi = np.array(bound.GetMin()), np.array(bound.GetMax())
        if path.startswith("/World/Tool/"):
            points = np.array(
                [[x, y, z] for x in [lo[0], hi[0]] for y in [lo[1], hi[1]] for z in [lo[2], hi[2]]]
            )
            tool.append((path, points))
        else:
            obstacles.append((path, lo, hi))
    if not tool or not obstacles:
        raise ValueError("Need both tool and Zero assembly")
    lows = np.array([row[1] for row in obstacles])
    highs = np.array([row[2] for row in obstacles])
    pivot = np.array([0.5, 0.0105, 0.29])
    rows = []
    for dx in np.linspace(0, -0.0125, 51):
        for yaw in [-0.5, 0, 0.5]:
            rotation = Rotation.from_euler("z", yaw, degrees=True).as_matrix()
            best = None
            for path, points in tool:
                transformed = (points - pivot) @ rotation.T + pivot + [dx, 0, 0]
                lo, hi = transformed.min(axis=0), transformed.max(axis=0)
                separations = np.maximum(np.maximum(lows - hi, lo - highs), 0)
                distances = np.linalg.norm(separations, axis=1)
                index = int(distances.argmin())
                candidate = (float(distances[index]), path, obstacles[index][0])
                if best is None or candidate[0] < best[0]:
                    best = candidate
            rows.append(
                {"translation_x_m": float(dx), "yaw_deg": yaw, "separation_m": best[0], "pair": best[1:]}
            )
    report = {
        "scene": str(scene.relative_to(ROOT)),
        "scene_sha256": hashlib.sha256(scene.read_bytes()).hexdigest(),
        "scope": "Offline sampled translated/rotated component AABBs from held pose",
        "minimum_separation_m": min(row["separation_m"] for row in rows),
        "samples": rows,
        "motion_authorized": False,
        "omissions": [
            "Full robot path",
            "Camera and mount solids",
            "Continuous swept geometry",
            "Cable motion and contact",
            "Real assembly tolerances",
        ],
    }
    output.write_text(json.dumps(report, indent=2))
    print(json.dumps({"minimum_mm": report["minimum_separation_m"] * 1000, "samples": len(rows)}))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--scene", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    check(args.scene.resolve(), args.output)
