"""Independently check every tip corner in a completed run's final USD scene."""

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdGeom

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from ffc.seating import seating_metrics  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    result = json.loads((args.run / "results.json").read_text())
    cfg = result["configuration"]
    scene = args.run / ("settled.usda" if result["status"] == "PASS" else "failed-state.usda")
    stage = Usd.Stage.Open(str(scene))
    cable = cfg["cable"]
    prim = stage.GetPrimAtPath(f"/World/Cable/segment_{cable['segments'] - 1:03d}")
    transform = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
    tip = transform.Transform(Gf.Vec3d(cable["length_m"] / cable["segments"] / 2, 0, 0))
    metrics = seating_metrics(tip, np.asarray(transform.ExtractRotationMatrix()).T, cable, cfg["connector"])
    peaks_path = args.run / "contact-peaks.json"
    peaks = json.loads(peaks_path.read_text()) if peaks_path.exists() else []
    invalid_tool_contacts = [
        c
        for c in peaks
        if c["step"] >= 0
        and all(c[key].startswith("/World/Tool/") for key in ("collider0", "collider1"))
        and c["sum_contact_force_magnitudes_n"] > 0.1
    ]
    grip_path = args.run / "grip-audit.json"
    grip_qualified = (
        json.loads(grip_path.read_text())["grip_stability_qualified"] if grip_path.exists() else False
    )
    audit = {
        "original_experiment_status": result["status"],
        "tip_geometry": metrics,
        "unintended_tool_self_contacts": invalid_tool_contacts,
        "all_step_contact_peaks_available": peaks_path.exists(),
        "independent_grip_stability_qualified": grip_qualified,
        "geometric_task_qualified": bool(
            result["status"] == "PASS"
            and result.get("assembly_success_tested")
            and metrics["success"]
            and peaks_path.exists()
            and not invalid_tool_contacts
            and grip_qualified
        ),
        "electrical_connection_verified": False,
        "final_scene_sha256": hashlib.sha256(scene.read_bytes()).hexdigest(),
        "audit_source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "seating_math_sha256": hashlib.sha256((ROOT / "src/ffc/seating.py").read_bytes()).hexdigest(),
    }
    (args.run / "seating-audit.json").write_text(json.dumps(audit, indent=2))
    print(json.dumps(audit, indent=2))


if __name__ == "__main__":
    main()
