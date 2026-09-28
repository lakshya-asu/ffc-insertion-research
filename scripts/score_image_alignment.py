"""Offline analytic projection scorer. Authored state never enters the estimator."""

import argparse
import json
from pathlib import Path

import numpy as np

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("dataset", type=Path)
p.add_argument("estimates", type=Path)
p.add_argument("output", type=Path)
a = p.parse_args()
if a.output.exists():
    p.error("Fresh output required")
labels = json.loads((a.dataset / "offline/labels.json").read_text())["frames"]
estimates = json.loads(a.estimates.read_text())
cal = json.loads((a.dataset / "sensor/camera.json").read_text())
k = np.asarray(cal["k"]).reshape(3, 3)
optical_world = np.linalg.inv(cal["world_optical"])
lookup = {r["file"]: r for r in labels}
rows = []


def project(transform, points):
    camera = (optical_world @ transform @ np.column_stack([points, np.ones(len(points))]).T)[:3]
    if not np.all(camera[2] > 0):
        raise ValueError("Projection behind camera")
    uv = k @ camera
    return ((uv[:2] / uv[2]).T + 0.5) * 0.5 - 0.5 + [4, 0]


for estimate in estimates["frames"]:
    truth = lookup[estimate["file"]]
    if estimate["rgb_sha256"] != truth["rgb_sha256"]:
        raise ValueError("Sensor identities differ")
    board = np.asarray(truth["authored"]["board_world"])
    cable = board @ truth["authored"]["cable_local"]
    # Authored substitute socket and stiffener dimensions, not manufacturer tolerances.
    references = {
        "upper_rim": project(board, [[0.0333, -0.00665, 0.00245], [0.0333, 0.00505, 0.00245]]),
        "lower_rim": project(board, [[0.0333, -0.00665, 0.00195], [0.0333, 0.00505, 0.00195]]),
        "cable_edge": project(cable, [[0, -0.00575, 0.00022], [0, 0.00575, 0.00022]]),
    }
    metrics = {}
    for name, feature in estimate["features"].items():
        endpoints = references[name]
        slope = (endpoints[1, 1] - endpoints[0, 1]) / (endpoints[1, 0] - endpoints[0, 0])
        intercept = endpoints[0, 1] - slope * endpoints[0, 0]
        x = np.linspace(
            max(feature["x_support"][0], min(endpoints[:, 0])),
            min(feature["x_support"][1], max(endpoints[:, 0])),
            25,
        )
        error = np.abs((feature["slope"] - slope) * x + feature["intercept"] - intercept) / np.sqrt(
            1 + slope * slope
        )
        metrics[name] = dict(
            mean_normal_error_px=float(error.mean()),
            max_normal_error_px=float(error.max()),
            angle_error_deg=abs(feature["angle_deg"] - float(np.degrees(np.arctan(slope)))),
        )
    invalid = truth["condition"] in ["absent_cable", "absent_socket", "empty_targets", "short_grasp"]
    rows.append(
        dict(
            file=estimate["file"],
            condition=truth["condition"],
            status=estimate["status"],
            unavailable_target_condition=invalid,
            reasons=estimate["reasons"],
            features=metrics,
            projected_reference_edges={key: value.tolist() for key, value in references.items()},
        )
    )
summary = {}
for name in ["upper_rim", "lower_rim", "cable_edge"]:
    values = [
        r["features"][name]
        for r in rows
        if r["status"] == "image_measurement" and not r["unavailable_target_condition"]
    ]
    summary[name] = dict(
        count=len(values),
        median_mean_error_px=float(np.median([v["mean_normal_error_px"] for v in values]))
        if values
        else None,
        worst_max_error_px=max((v["max_normal_error_px"] for v in values), default=None),
        worst_angle_error_deg=max((v["angle_error_deg"] for v in values), default=None),
    )
report = dict(
    summary=summary,
    frames=rows,
    accepted=sum(r["status"] == "image_measurement" for r in rows),
    invalid_condition_accepts=sum(
        r["status"] == "image_measurement" and r["unavailable_target_condition"] for r in rows
    ),
    protocol_sha256=estimates["protocol_sha256"],
    estimator_sha256=estimates["estimator_sha256"],
    scope=(
        "Retrospective diagnostic on previously inspected test scenes. "
        "Analytic authored geometry is scorer-only. "
        "No independent confidence calibration or metric pose claim."
    ),
)
a.output.write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps({k: v for k, v in report.items() if k != "frames"}, indent=2))
