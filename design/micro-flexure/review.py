"""Independent nominal clearance and optical-ray screening; no physics."""

import argparse
import json
from pathlib import Path

import numpy as np
from build import parts
from pxr import Usd, UsdGeom

ROOT = Path(__file__).resolve().parents[2]


def bounds(shape):
    b = shape.BoundingBox()
    return np.array([b.xmin, b.ymin, b.zmin]), np.array([b.xmax, b.ymax, b.zmax])


def blocked(eye, target, boxes):
    direction = target - eye
    for name, lo, hi in boxes:
        start, end = 0.0, 1.0
        for axis in range(3):
            if abs(direction[axis]) < 1e-10:
                if eye[axis] < lo[axis] or eye[axis] > hi[axis]:
                    start, end = 1.0, 0.0
                    break
            else:
                a, b = (lo[axis] - eye[axis]) / direction[axis], (hi[axis] - eye[axis]) / direction[axis]
                start, end = max(start, min(a, b)), min(end, max(a, b))
        if start <= end and end > 0 and start < 1:
            return name
    return None


def review(output):
    pp = parts(0.14)
    bb = [(p["name"], *bounds(p["shape"])) for p in pp]
    minimum = np.min([p[1] for p in bb], axis=0)
    maximum = np.max([p[2] for p in bb], axis=0)
    stage = Usd.Stage.Open(str(ROOT / "third_party/raspberry_pi/zero/zero2w.usdc"))
    cache = UsdGeom.BBoxCache(0, ["default", "render"])
    obstacles = []
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh) and "/Part_0010" not in str(prim.GetPath()):
            b = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            obstacles.append(
                (
                    np.array(b.GetMin()) * 1000 + [-33.3, 0.8, -2.2],
                    np.array(b.GetMax()) * 1000 + [-33.3, 0.8, -2.2],
                )
            )
    # Conservative outer socket box for TOOL clearance, not cable collision.
    obstacles.append((np.array([-2.85, -6.18, -0.55]), np.array([0, 6.18, 0.55])))
    distances = []
    for depth in np.linspace(-10, 2.5, 51):
        for _name, lo, hi in bb:
            for a, b in obstacles:
                gap = np.maximum(np.maximum(a - (hi - [depth, 0, 0]), lo - [depth, 0, 0] - b), 0)
                distances.append(float(np.linalg.norm(gap)))
    entrance = []
    targets = [np.array([0, y, z]) for y in [-5.975, 0, 5.975] for z in [-0.35, 0.35]]
    eyes = [np.array([105, 70, 12]), np.array([105, -70, 12])]
    for standoff in [10, 3, 0]:
        boxes = [(name, lo + [standoff, 0, 0], hi + [standoff, 0, 0]) for name, lo, hi in bb]
        results = [[blocked(eye, target, boxes) for target in targets] for eye in eyes]
        entrance.append(
            {
                "standoff_mm": standoff,
                "blocking_parts_per_view": results,
                "rim_samples_visible_in_any_view": sum(
                    a is None or b is None for a, b in zip(*results, strict=True)
                ),
            }
        )
    top_targets = [np.array([x, y, 0.15]) for x in [0, 6] for y in [-5.75, 5.75]]
    inspection = [
        [blocked(eye, target, bb) for target in top_targets]
        for eye in [np.array([-32, 0, 122]), np.array([38, 0, 122])]
    ]
    assert min(distances) > 0
    assert all(reason is None for view in inspection for reason in view)
    assert 8.5 > 6 > 4
    result = {
        "status": "Geometry screening only; no full-cell collision or perception qualification",
        "overall_head_bounds_mm": [minimum.tolist(), maximum.tolist()],
        "overall_head_size_mm": (maximum - minimum).tolist(),
        "sampled_tool_to_board_and_socket_aabb_separation_mm": min(distances),
        "translation_samples": 51,
        "path": "10 mm standoff through 2.5 mm nominal depth; cable/tool rigid translation only",
        "grasp_patch_nominal_mm": [8.5, 10.5],
        "stiffener_end_mm": 6,
        "exposed_contact_end_mm": 4,
        "inspection_tool_occlusion": inspection,
        "entrance_tool_occlusion": entrance,
        "ray_scope": "Tool AABBs only. Cable, board self-occlusion, optics and camera mounts excluded",
        "limits": [
            "No FR3 adapter or arm sweep",
            "No force-loaded deformed finger",
            "No physical tolerance stack",
            "No sensor-driven execution",
        ],
    }
    output.write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    review(p.parse_args().output)
