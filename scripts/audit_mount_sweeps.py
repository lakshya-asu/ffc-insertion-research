"""Continuous translation-envelope checks for the nominal fixed-orientation tool route."""

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from ffc.mount_clearance import Box, box, separation  # noqa: E402

source = ROOT / "outputs/mount-clearance-004/report.json"
r = json.loads(source.read_text())
mouth = np.array(r["mouth_m"])
R = np.array(r["tool_rotation"])
point = np.array(r["reference_point_link7_m"])
obstacles = [
    Box(b["name"], np.array(b["center"]), np.array(b["half"]), np.array(b["axes"]))
    for b in r["candidates"][1]["obstacles"]
]
tool = [
    Box(b["name"], np.array(b["center"]), np.array(b["half"]), np.array(b["axes"]))
    for b in r["local_tool_boxes"]
]
paths = r["paths"]
groups = []
for group in ["cable", "latch_access"]:
    rows = [p for p in paths if p["group"] == group]
    segments = list(dict.fromkeys(p["segment"] for p in rows))
    for segment in segments:
        ends = [p for p in rows if p["segment"] == segment]
        a, b = ends[0], ends[-1]
        t = np.eye(4)
        t[:3, :3] = R
        t[:3, 3] = mouth + np.array(a["offset_m"]) + [0.01 if group == "cable" else 0, 0, 0] - R @ point
        moving = [part.transformed(t) for part in tool]
        if group == "cable":
            moving.append(
                box("rigid_held_cable", mouth + np.array(a["offset_m"]) + [0.1, 0, 0], [0.2, 0.016, 0.0006])
            )
        delta = np.array(b["offset_m"]) - a["offset_m"]
        minimum = (1e9, None, None)
        for part in moving:
            swept = Box(
                part.name, part.center + delta / 2, part.half + np.abs(part.axes.T @ delta) / 2, part.axes
            )
            for obstacle in obstacles:
                gap = separation(swept, obstacle)
                if gap < minimum[0]:
                    minimum = (gap, part.name, obstacle.name)
        groups.append(
            dict(
                group=group,
                segment=segment,
                minimum_gap_lower_bound_m=minimum[0],
                moving_part=minimum[1],
                mount_part=minimum[2],
            )
        )
result = dict(
    source=str(source.relative_to(ROOT)),
    segments=groups,
    minimum_nominal_gap_m=min(g["minimum_gap_lower_bound_m"] for g in groups),
    assembly_budget_m=0.002,
    required_remaining_gap_m=0.005,
    scope=(
        "Swept OBB containment for continuous fixed-orientation translations. "
        "Rigid cable; full robot remains a sampled IK check."
    ),
    motion_permitted=False,
)
result["passes_assumed_budget"] = (
    result["minimum_nominal_gap_m"] - result["assembly_budget_m"] >= result["required_remaining_gap_m"]
)
(ROOT / "outputs/mount-clearance-004/translation-sweeps.json").write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps(result, indent=2))
