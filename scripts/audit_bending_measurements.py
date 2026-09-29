"""Recompute bending sag and settling independently from archived raw poses."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def audit(run):
    report = json.loads((run / "report.json").read_text())
    with np.load(run / "measured-poses.npz", allow_pickle=False) as archive:
        times = archive["time_s"]
        poses = archive["position_xyz_quaternion_wxyz"]
        paths = archive["tip_paths"].tolist()
    cases = report["cases"]
    if paths != [c["tip_path"] for c in cases] or poses.shape != (len(times), len(cases), 7):
        raise ValueError("Raw pose archive does not match report cases")
    dt = report["settings"]["dt_s"]
    duration = report["settings"]["duration_s"]
    if not np.isfinite(poses).all() or not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError("Invalid poses or sample times")
    tail = times >= duration - 0.05 - dt / 2
    expected_times = np.arange(round(0.05 / dt) + 1) * dt + duration - 0.05
    if times[tail].shape != expected_times.shape or not np.allclose(
        times[tail], expected_times, atol=1e-12, rtol=0
    ):
        raise ValueError("Final 50 ms lacks complete per-step pose coverage")
    offsets = np.tile([0.001, 0, 0], (poses.shape[0] * poses.shape[1], 1))
    rotations = Rotation.from_quat(poses[:, :, 3:].reshape(-1, 4), scalar_first=True)
    tips = poses[:, :, :3] + rotations.apply(offsets).reshape(poses.shape[:2] + (3,))
    sags = np.asarray([c["origin_m"][2] for c in cases]) - tips[:, :, 2]
    rows = []
    for i, case in enumerate(cases):
        target = case["reference"]["nonlinear_hinge_chain_prediction_m"]
        sag = float(sags[-1, i])
        excursion = float(np.ptp(sags[tail, i]))
        error = abs(sag / target - 1)
        passed = error <= 0.05 and excursion <= 0.01 * target
        if abs(sag - case["observed_sag_m"]) > 1e-12 or abs(excursion - case["tail_excursion_m"]) > 1e-12:
            raise ValueError("Independent pose rescore disagrees with reported measurements")
        if passed != case["passed"]:
            raise ValueError("Independent acceptance disagrees with reported acceptance")
        rows.append(
            {
                "tip_path": case["tip_path"],
                "observed_sag_m": sag,
                "relative_error": error,
                "tail_excursion_m": excursion,
                "tail_excursion_over_reference_sag": excursion / target,
                "passed": passed,
            }
        )
    return {
        "scope": "Independent SciPy quaternion rescore of archived physics measurements",
        "sources": {
            name: hashlib.sha256((run / name).read_bytes()).hexdigest()
            for name in ["report.json", "measured-poses.npz", "settings.json"]
        },
        "tail_samples": int(tail.sum()),
        "cases": rows,
        "all_passed": all(r["passed"] for r in rows),
    }


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--run", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    result = audit(args.run)
    with args.output.open("x") as stream:
        stream.write(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"cases": len(result["cases"]), "all_passed": result["all_passed"]}))


if __name__ == "__main__":
    main()
