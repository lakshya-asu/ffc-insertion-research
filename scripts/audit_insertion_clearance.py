"""Offline per-mesh bounds against slot/support; overlap is not exact collision."""

import argparse
import json
from pathlib import Path

import numpy as np
from pxr import Usd, UsdGeom


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("archive", type=Path)
    a = p.parse_args()
    stage = Usd.Stage.Open(str(a.archive / "replay.usdz"))
    frames = json.loads((a.archive / "offline-cable-trace.json").read_text())
    meshes = [
        prim
        for prim in stage.Traverse()
        if prim.IsA(UsdGeom.Mesh) and str(prim.GetPath()).startswith("/World/Tool/")
    ]
    obstacles = [
        prim
        for prim in stage.Traverse()
        if prim.IsA(UsdGeom.Cube) and str(prim.GetPath()).startswith(("/World/Slot/", "/World/Support/"))
    ]
    minimum = float("inf")
    flagged = {}
    for frame in frames:
        cache = UsdGeom.BBoxCache(frame["time_s"] * 24, ["default", "render"])

        def bounds(prim, cache=cache):
            box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            return np.array(box.GetMin()), np.array(box.GetMax())

        for mesh in meshes:
            lo, hi = bounds(mesh)
            for obstacle in obstacles:
                other_lo, other_hi = bounds(obstacle)
                delta = np.maximum(other_lo - hi, lo - other_hi)
                distance = float(np.linalg.norm(np.maximum(delta, 0)))
                minimum = min(minimum, distance)
                if np.all(delta <= 0):
                    key = str(mesh.GetPath()) + " | " + str(obstacle.GetPath())
                    flagged.setdefault(key, frame["time_s"])
    result = {
        "scope": "Sampled CAD mesh AABBs against fixture/support; not swept or exact collision",
        "frames_checked": len(frames),
        "meshes_checked": len(meshes),
        "minimum_separation_lower_bound_m": minimum,
        "possible_overlap_pairs_first_time_s": flagged,
        "all_sampled_bounds_separate": not flagged,
        "full_robot_clearance_qualified": False,
    }
    with (a.archive / "clearance.json").open("x") as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps({k: v for k, v in result.items() if k != "possible_overlap_pairs_first_time_s"}))


if __name__ == "__main__":
    main()
