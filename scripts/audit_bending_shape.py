"""Compare every saved link endpoint with the frozen reference centerline offline."""

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdGeom

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from ffc.bending_fit import rotated_offset


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--run", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    report = json.loads((a.run / "report.json").read_text())
    stage = Usd.Stage.Open(str(a.run / "bending.usda"))
    rows = []
    for case in report["cases"]:
        root = case["tip_path"].rsplit("/", 1)[0]
        n = round(case["length_m"] / 0.002)
        ds = case["length_m"] / n
        angles = np.r_[0, np.cumsum(case["reference"]["reference_joint_angles_rad"])]
        target = np.asarray(case["origin_m"]) + np.cumsum(
            np.column_stack([ds * np.cos(angles), np.zeros(n), -ds * np.sin(angles)]), axis=0
        )
        observed = []
        poses = []
        for i in range(n):
            prim = stage.GetPrimAtPath(f"{root}/segment_{i:03d}")
            parent = UsdGeom.Xformable(prim.GetParent()).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            if parent != Gf.Matrix4d(1):
                raise ValueError("Benchmark requires identity parent transforms")
            if list(prim.GetAttribute("xformOpOrder").Get()) != [
                "xformOp:translate",
                "xformOp:orient",
                "xformOp:scale",
            ] or tuple(prim.GetAttribute("xformOp:scale").Get()) != (1, 1, 1):
                raise ValueError("Unexpected benchmark transform layout")
            position = np.asarray(prim.GetAttribute("xformOp:translate").Get(), dtype=float)
            q = prim.GetAttribute("xformOp:orient").Get()
            quaternion = [float(q.GetReal()), *map(float, q.GetImaginary())]
            observed.append(position + rotated_offset(quaternion, (ds / 2, 0, 0)))
            poses.append({"position_m": position.tolist(), "quaternion_wxyz": quaternion})
        errors = np.linalg.norm(np.asarray(observed) - target, axis=1)
        max_error = float(errors.max())
        normalized = max_error / case["reference"]["nonlinear_hinge_chain_prediction_m"]
        sag = float(case["origin_m"][2] - observed[-1][2])
        relative_error = abs(sag / case["reference"]["nonlinear_hinge_chain_prediction_m"] - 1)
        rows.append(
            {
                "tip_path": case["tip_path"],
                "gain": case["gain"],
                "split": case["split"],
                "length_m": case["length_m"],
                "modulus_multiplier": case["modulus_multiplier"],
                "uniform_load_multiplier": case["uniform_load_multiplier"],
                "max_endpoint_error_m": max_error,
                "error_over_tip_sag": normalized,
                "within_5_percent_of_tip_sag": normalized <= 0.05,
                "observed_sag_m": sag,
                "relative_error": relative_error,
                "static_tip_within_5_percent": relative_error <= 0.05,
                "original_reported_sag_m": case["observed_sag_m"],
                "settling_revalidated": False,
                "segment_poses": poses,
            }
        )
    result = {
        "scope": "Offline saved final centerline only; no dynamics or contact qualification",
        "measurement": "Float64 normalized quaternion vector rotation; saved final USD poses",
        "settling_note": "Original trace lacks raw poses; corrected settling cannot be recovered from it",
        "sources": {
            name: hashlib.sha256((a.run / name).read_bytes()).hexdigest()
            for name in ["report.json", "bending.usda", "settings.json"]
        },
        "cases": rows,
    }
    with a.output.open("x") as stream:
        stream.write(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"cases": len(rows), "max_error_ratio": max(r["error_over_tip_sag"] for r in rows)}))


if __name__ == "__main__":
    main()
