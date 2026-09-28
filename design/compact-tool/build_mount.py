"""Build a provisional FR3/PGEA bracket and a separate stationary candidate scene.

CAD fit and parked-pose bounds only. No dynamics or manufacturing release.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path

import cadquery as cq
import numpy as np
from audit_geometry import SOURCE_SHA256
from build_fingers import CX, FRONT, MID, box, cylinder
from pxr import Gf, Usd, UsdGeom, UsdPhysics


def build(source, fingers, base_scene, output):
    if hashlib.sha256(source.read_bytes()).hexdigest() != SOURCE_SHA256:
        raise ValueError("Supplier source changed")
    output.mkdir(parents=True, exist_ok=False)
    # Normalized gripper frame: front face y=0, closing midplane z=0.
    ring = cylinder(31.5, 6, (0, -95, 0)).cut(cylinder(16, 6.2, (0, -95.1, 0)))
    flange_holes = []
    for angle in [45, 135, 225, 315]:
        x, z = 25 * math.cos(math.radians(angle)), 25 * math.sin(math.radians(angle))
        ring = ring.cut(cylinder(3.3, 6.2, (x, -95.1, z)))
        flange_holes.append([x, z])
    arm = box((18, 66, 6), (0, -62, -18))  # y=-95..-29, z=-21..-15
    for y in [-44.5, -34.5]:
        arm = arm.cut(cq.Solid.makeCylinder(1.7, 6.2, cq.Vector(0, y, -21.1), cq.Vector(0, 0, 1)))
    bracket = ring.fuse(arm).clean()
    if not bracket.isValid() or len(bracket.Solids()) != 1:
        raise ValueError("Bracket is not a valid connected solid")
    cq.exporters.export(bracket, str(output / "adapter-concept.step"))
    supplier = cq.importers.importStep(str(source)).solids().vals()
    supplier = [s.translate((-CX, -FRONT, -MID)) for s in supplier]
    fixed_overlap = abs(bracket.intersect(cq.Compound.makeCompound(supplier[:11])).Volume())
    rows = []
    for gap in [1, 1.14, 1.3, 3, 5, 7, 9, 11]:
        d = (11.4 - gap) / 2
        moving = cq.Compound.makeCompound(
            [s.translate((0, 0, d)) for s in supplier[11:16]]
            + [s.translate((0, 0, -d)) for s in supplier[16:21]]
        )
        rows.append(
            {"base_gap_mm": gap, "adapter_moving_overlap_mm3": abs(bracket.intersect(moving).Volume())}
        )
    adapter = Usd.Stage.CreateNew(str(output / "adapter.usda"))
    root = UsdGeom.Xform.Define(adapter, "/Adapter")
    adapter.SetDefaultPrim(root.GetPrim())
    UsdGeom.SetStageMetersPerUnit(adapter, 1)
    UsdGeom.SetStageUpAxis(adapter, "Z")
    vertices, triangles = bracket.tessellate(0.025, 0.15)
    mesh = UsdGeom.Mesh.Define(adapter, "/Adapter/Bracket")
    mesh.CreatePointsAttr([(v.x / 1000, v.y / 1000, v.z / 1000) for v in vertices])
    mesh.CreateFaceVertexCountsAttr([3] * len(triangles))
    mesh.CreateFaceVertexIndicesAttr([i for t in triangles for i in t])
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDisplayColorAttr([Gf.Vec3f(0.17, 0.35, 0.40)])
    adapter.GetRootLayer().Save()
    scene = Usd.Stage.CreateNew(str(output / "mounted-candidate.usda"))
    scene.GetRootLayer().subLayerPaths = [os.path.relpath(base_scene, output)]
    scene.OverridePrim("/World/Tool").SetActive(False)
    link = str(
        UsdPhysics.RevoluteJoint.Get(scene, "/World/FR3/Physics/fr3v2_1_joint7").GetBody1Rel().GetTargets()[0]
    )
    flange = link + "/fr3v2_1_link8"
    if not scene.GetPrimAtPath(flange):
        raise ValueError("FR3 flange missing")
    mount_path = flange + "/CompactMount"
    mount = UsdGeom.Xform.Define(scene, mount_path)
    transform = Gf.Matrix4d(1)
    transform.SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 90))
    transform.SetTranslateOnly(Gf.Vec3d(0, 0, 0.095))
    mount.AddTransformOp().Set(transform)
    tool = scene.DefinePrim(mount_path + "/Gripper")
    tool.GetReferences().AddReference(os.path.relpath(fingers / "compact-fingers.usda", output))
    scene.OverridePrim(mount_path + "/Gripper/reference").SetActive(False)
    scene.DefinePrim(mount_path + "/Adapter").GetReferences().AddReference("adapter.usda")
    # Only a visual mounting configuration; existing robot pose is not changed.
    scene.GetRootLayer().Save()
    bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    compact = bounds.ComputeWorldBound(mount.GetPrim()).ComputeAlignedRange()
    lo, hi = np.array(compact.GetMin()), np.array(compact.GetMax())
    obstacles = []
    for parent in ["/World/Mount", "/World/Hardware", "/World/MacroTask", "/World/Desk"]:
        for prim in scene.GetPrimAtPath(parent).GetChildren():
            if not prim.IsActive() or not prim.IsA(UsdGeom.Imageable):
                continue
            r = bounds.ComputeWorldBound(prim).ComputeAlignedRange()
            a, b = np.array(r.GetMin()), np.array(r.GetMax())
            if not np.isfinite(np.r_[a, b]).all():
                continue
            separation = np.maximum(np.maximum(a - hi, lo - b), 0)
            obstacles.append(
                {
                    "path": str(prim.GetPath()),
                    "aabb_separation_lower_bound_mm": float(np.linalg.norm(separation) * 1000),
                }
            )
    report = {
        "status": "STATIONARY_VISUAL_MOUNT_NOT_PHYSICS_OR_MANUFACTURING_RELEASED",
        "flange_source": "https://www.franka.de/hubfs/Product%20Manual%20Franka%20Research%203_R02210_1.5_EN-1.pdf",
        "flange_drawing": "Figure 6.11: 50 mm pitch circle, four M6 threaded holes; 45 degree pattern",
        "mount_path": mount_path,
        "adapter": {
            "outside_diameter_mm": 63,
            "ring_thickness_mm": 6,
            "center_opening_mm": 32,
            "flange_clearance_holes_mm": 6.6,
            "flange_holes_xz_mm": flange_holes,
            "side_arm_width_mm": 18,
            "side_arm_thickness_mm": 6,
            "body_hole_y_mm": [-44.5, -34.5],
            "body_screw_clearance_mm": 3.4,
            "proposed_body_screws": "M3 x 8, nominal 2 mm engagement into drawing depth 4 mm; not modeled",
            "finger_front_from_flange_mm": 107,
        },
        "adapter_fixed_supplier_overlap_mm3": fixed_overlap,
        "adapter_sweep": rows,
        "adapter_fit_pass": fixed_overlap < 1e-6
        and all(r["adapter_moving_overlap_mm3"] < 1e-6 for r in rows),
        "parked_tool_world_aabb_m": [lo.tolist(), hi.tolist()],
        "parked_obstacle_bounds": obstacles,
        "old_tool_active": scene.GetPrimAtPath("/World/Tool").IsActive(),
        "limitations": [
            "Bolt orientation relative to factory flange axes and locating fit not qualified",
            "Ring center opening does not establish wrist connector access",
            "Fasteners, pin retention, structural stiffness and cable bend radius not qualified",
            "No vacuum nozzle or hoses yet",
            "Bounds apply only to this unchanged parked robot pose; "
            "zero distance means inconclusive, not necessarily collision",
            "No arm self-collision, swept trajectory, perception visibility or insertion check",
            "Visual parenting only: no compact-tool rigid bodies, inertia or drives",
        ],
    }
    (output / "mount-report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(
        json.dumps(
            {"fit": report["adapter_fit_pass"], "fixed_overlap": fixed_overlap, "obstacles": obstacles}
        ),
        flush=True,
    )
    if not report["adapter_fit_pass"]:
        raise RuntimeError("Adapter interference: preserve this run and revise")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", type=Path, required=True)
    p.add_argument("--fingers", type=Path, required=True)
    p.add_argument("--base-scene", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    build(a.source.resolve(), a.fingers.resolve(), a.base_scene.resolve(), a.output.resolve())
