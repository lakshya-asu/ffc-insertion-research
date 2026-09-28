"""Offline mount design sweep using complete tool geometry and vendor FR3 link bounds."""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from ffc.isaac_kinematics import from_stage  # noqa: E402
from ffc.mount_clearance import Box, box, hardware, link_poses, separation  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--profile", type=Path, default=ROOT / "config/macro-basler-kowa.json")
    a = parser.parse_args()
    if a.output.exists():
        parser.error("Fresh output directory required")
    a.output.mkdir(parents=True)
    stage = Usd.Stage.Open(str(ROOT / "outputs/macro-preview-scene-002/macro-workcell.usda"))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    transforms = UsdGeom.XformCache()
    chain = from_stage(stage)
    links = []
    local = []
    for i in range(1, 8):
        joint = UsdPhysics.RevoluteJoint.Get(stage, f"/World/FR3/Physics/fr3v2_1_joint{i}")
        prim = stage.GetPrimAtPath(joint.GetBody1Rel().GetTargets()[0])
        links.append(str(prim.GetPath()))
        # Include own visual geometry; child robot links have separate boxes.
        visual = prim.GetChild(f"fr3v2_1_link{i}_visual")
        bound = cache.ComputeRelativeBound(visual, prim).ComputeAlignedRange()
        lo, hi = np.array(bound.GetMin()), np.array(bound.GetMax())
        if not np.isfinite(np.r_[lo, hi]).all() or np.any(hi <= lo):
            raise RuntimeError("Invalid link bound")
        local.append(box(f"fr3_link{i}", (hi + lo) / 2, hi - lo))
    # Link8 flange has its own visual and fixed offset; include it in link7 coordinates.
    flange = stage.GetPrimAtPath(links[-1] + "/fr3v2_1_link8")
    if flange:
        b = cache.ComputeRelativeBound(flange, stage.GetPrimAtPath(links[-1])).ComputeAlignedRange()
        lo, hi = np.array(b.GetMin()), np.array(b.GetMax())
        local.append(box("flange_link8", (hi + lo) / 2, hi - lo))
    link7world = np.asarray(transforms.GetLocalToWorldTransform(stage.GetPrimAtPath(links[-1]))).T
    inv = np.linalg.inv(link7world)
    tool = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/Tool")):
        if prim.IsA(UsdGeom.Cube) or prim.IsA(UsdGeom.Cylinder):
            b = cache.ComputeUntransformedBound(prim).ComputeAlignedRange()
            lo, hi = np.array(b.GetMin()), np.array(b.GetMax())
            transform = inv @ np.asarray(transforms.GetLocalToWorldTransform(prim)).T
            scale = np.linalg.norm(transform[:3, :3], axis=0)
            axes = transform[:3, :3] / scale
            if not np.allclose(axes.T @ axes, np.eye(3), atol=1e-6):
                raise RuntimeError("Sheared primitive needs a different bounding method")
            center = transform[:3, :3] @ ((lo + hi) / 2) + transform[:3, 3]
            half = (hi - lo) / 2 * scale
            name = str(prim.GetPath())
            if "/Carriage/" in name or "/LowerJaw/" in name:
                half += np.abs(axes.T) @ np.array([0.018, 0.020, 0])
            tool.append(Box(name, center, half, axes))
    profile = json.loads(a.profile.read_text())
    mouth = np.array(json.loads((ROOT / "config/zero-feature-v1.json").read_text())["mouth_m"])
    point = np.array([0.0, 0.121, 0.218])
    R = np.array([[-1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, -1.0, 0.0]])
    # Held-cable rigid envelope is deliberately included, not just the fingertip.
    waypoints = [
        ("side_high", [0.030, 0.080, 0.080]),
        ("side_low", [0.030, 0.080, 0]),
        ("preapproach", [0.030, 0, 0]),
        ("near_entry", [0.0003, 0, 0]),
        ("withdraw", [0.030, 0, 0]),
        ("side_exit", [0.030, 0.080, 0]),
        ("lift_clear", [0.030, 0.080, 0.080]),
    ]
    # Latch access is a reserved pre-contact tool corridor, not a latch skill.
    latch = [("latch_above", [-0.001, 0, 0.030]), ("latch_precontact", [-0.001, 0, 0.003])]
    seed = np.array([0, -0.5, 0, -2, 0, 1.5, 0])
    rng = np.random.default_rng(260928)
    rows = []
    paths = []
    for group, wp in [("cable", waypoints), ("latch_access", latch)]:
        for segment in range(len(wp) - 1):
            start, end = np.array(wp[segment][1]), np.array(wp[segment + 1][1])
            n = int(np.ceil(np.linalg.norm(end - start) / 0.002)) + 1
            for fraction in np.linspace(0, 1, n):
                offset = start * (1 - fraction) + end * fraction
                # Tool reference is 10 mm behind the leading edge during handling.
                target = mouth + offset + np.array([0.010 if group == "cable" else 0, 0, 0])
                seed, metric = chain.solve(target, R, seed, point, rng)
                poses = link_poses(chain, seed)
                moving = [b.transformed(poses[min(i, 6)]) for i, b in enumerate(local)]
                moving.extend(b.transformed(poses[-1]) for b in tool)
                if group == "cable":
                    moving.append(box("rigid_held_cable", mouth + offset + [0.1, 0, 0], [0.2, 0.016, 0.0006]))
                paths.append(
                    dict(
                        group=group,
                        segment=wp[segment + 1][0],
                        offset_m=offset.tolist(),
                        q=seed.tolist(),
                        position_error_m=metric["position_error_m"],
                        orientation_error_rad=metric["orientation_error_rad"],
                        link_poses=[t.tolist() for t in poses],
                        tool_pose=poses[-1].tolist(),
                    )
                )
                rows.append(moving)
    candidates = []
    for side in [1, -1]:
        obstacles, _ = hardware(profile, mouth, side)
        minimum = (1e9, None, None, None)
        failures = 0
        for idx, moving in enumerate(rows):
            for m in moving:
                for o in obstacles:
                    gap = separation(m, o)
                    if gap < minimum[0]:
                        minimum = (gap, idx, m.name, o.name)
                    if gap < 0.005:
                        failures += 1
        candidates.append(
            dict(
                post_side=side,
                minimum_separating_gap_m=minimum[0],
                worst_sample=minimum[1],
                moving_part=minimum[2],
                mount_part=minimum[3],
                pairs_below_5mm=failures,
                obstacles=[b.json() for b in obstacles],
            )
        )
    report = dict(
        samples=len(paths),
        candidates=candidates,
        paths=paths,
        link_paths=links,
        local_robot_boxes=[b.json() for b in local],
        local_tool_boxes=[b.json() for b in tool],
        reference_point_link7_m=point.tolist(),
        tool_rotation=R.tolist(),
        profile=profile,
        mouth_m=mouth.tolist(),
        motion_permitted=False,
        scope="Sampled offline CAD-envelope clearance against camera and mounting hardware only",
        limitations=[
            "Tool is an unqualified design concept; bounds include full actuator travel",
            "No arm self-collision, PCB contact, dynamics, deflection or measured tolerances qualified",
            "Latch corridor stops before contact; it does not demonstrate latch actuation",
            "IK uses geometry only for offline installation planning, never runtime perception",
        ],
    )
    (a.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(
        json.dumps(
            dict(
                samples=len(paths),
                candidates=[{k: v for k, v in c.items() if k != "obstacles"} for c in candidates],
            ),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
