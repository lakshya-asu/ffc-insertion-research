"""Offline side-entry handoff CAD and conservative swept bounds. No physics."""

import argparse
import hashlib
import json
import os
import sys
from pathlib import Path

import cadquery as cq
import numpy as np
from pxr import Gf, Usd, UsdGeom

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
from ffc.grasp_footprint import EndGeometry, rectangle_half_bounds, review  # noqa: E402


def cube(stage, path, center, size, color):
    shape = UsdGeom.Cube.Define(stage, path)
    shape.CreateSizeAttr(1)
    shape.AddTranslateOp().Set(Gf.Vec3d(*(np.array(center) / 1000)))
    shape.AddScaleOp().Set(Gf.Vec3f(*(np.array(size) / 1000)))
    shape.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    return shape


def set_tool(stage, branch, x_mm, gap_mm):
    root = f"/World/{branch}/ToolPlacement"
    UsdGeom.Xformable(stage.GetPrimAtPath(root)).GetOrderedXformOps()[0].Set(
        Gf.Vec3d(x_mm / 1000, 0.012, 0.03007)
    )
    for group, sign in [("lower", 1), ("upper", -1)]:
        op = UsdGeom.Xformable(stage.GetPrimAtPath(root + "/Tool/" + group)).GetOrderedXformOps()[0]
        op.Set(Gf.Vec3d(0, 0, sign * (11.4 - gap_mm) / 2000))


def bounds(stage, path):
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    r = cache.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()
    return np.array(r.GetMin()) * 1000, np.array(r.GetMax()) * 1000


def separation(a, b):
    gap = np.maximum(b[0] - a[1], a[0] - b[1])
    if np.all(gap < 0):
        return float(np.max(gap))
    return float(np.linalg.norm(np.maximum(gap, 0)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    stage = Usd.Stage.CreateNew(str(out / "handoff.usda"))
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, "Z")
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/World").GetPrim())
    source = ROOT / "outputs/compact-fingers-002/compact-fingers.usda"
    branches = {"Fixture": -0.16, "Integrated": 0.16}
    fixture_boxes = [
        ("Base", (0, 106, 2), (36, 190, 4)),
        ("FrontSupport", (0, 29, 17), (11, 18, 26)),
        ("TailSupport", (0, 125, 17), (11, 150, 26)),
    ]
    fixture = None
    for _, center, size in fixture_boxes:
        part = cq.Workplane("XY").box(*size).val().translate(center)
        fixture = part if fixture is None else fixture.fuse(part)
    fixture = fixture.clean()
    if not fixture.isValid() or len(fixture.Solids()) != 1:
        raise ValueError("Fixture must be one connected valid solid")
    cq.exporters.export(fixture, str(out / "presentation-fixture.step"))
    cq.exporters.export(fixture, str(out / "presentation-fixture.stl"))
    obstacles = {}
    for name, shift in branches.items():
        root = UsdGeom.Xform.Define(stage, f"/World/{name}")
        root.AddTranslateOp().Set(Gf.Vec3d(shift, 0, 0))
        desk = f"/World/{name}/Desk"
        cube(stage, desk, (0, 65, -5), (290, 300, 10), (0.19, 0.22, 0.25))
        obstacles[name] = [desk]
        placement = UsdGeom.Xform.Define(stage, f"/World/{name}/ToolPlacement")
        placement.AddTranslateOp()
        placement.AddRotateZOp().Set(-90)
        tool = stage.DefinePrim(str(placement.GetPath()) + "/Tool")
        tool.GetReferences().AddReference(os.path.relpath(source, out))
        stage.OverridePrim(str(tool.GetPath()) + "/reference").SetActive(False)
        set_tool(stage, name, -10.5, 11)
        z = 30 if name == "Fixture" else 0
        # Uniform-width body is an end-and-tail clearance proxy; far-end transition omitted.
        cube(stage, f"/World/{name}/CableBody", (0, 100, z + 0.07), (11.5, 200, 0.14), (0.055, 0.06, 0.075))
        cube(stage, f"/World/{name}/TopStiffener", (0, 3, z + 0.22), (11.5, 6, 0.16), (0.06, 0.27, 0.75))
        for i in range(22):
            cube(
                stage,
                f"/World/{name}/Contacts/C{i:02}",
                (-5.25 + i * 0.5, 2, z - 0.001),
                (0.3, 4, 0.002),
                (0.88, 0.64, 0.2),
            )
        if name == "Fixture":
            for part_name, center, size in fixture_boxes:
                path = f"/World/{name}/Fixture/{part_name}"
                cube(stage, path, center, size, (0.14, 0.49, 0.45))
                obstacles[name].append(path)
        # Vacuum cartridge envelope only. No unverified supplier CAD is implied.
        cup = UsdGeom.Cylinder.Define(stage, f"/World/{name}/CupEnvelope")
        cup.CreateRadiusAttr(0.0035)
        cup.CreateHeightAttr(0.019)
        cup.AddTranslateOp().Set(Gf.Vec3d(0, 0.026, (z + 0.14 + 9.5) / 1000))
        cup.CreateDisplayColorAttr([Gf.Vec3f(0.65, 0.3, 0.68)])
        if name == "Fixture":
            UsdGeom.Imageable(cup).MakeInvisible()  # Delivery pickup has withdrawn before pinch.
        if name == "Integrated":
            # A guide line records demanded travel, not a selected or fitted actuator.
            cube(
                stage, "/World/Integrated/RequiredCupTravel", (7, 26, 24.64), (0.4, 0.4, 30), (0.9, 0.7, 0.25)
            )
    rows = []
    phases = [
        ("side_approach", (-45, 11), (-10.5, 11)),
        ("close_on_body", (-10.5, 11), (-10.5, 1.14)),
        ("reopen", (-10.5, 1.14), (-10.5, 11)),
    ]
    for branch in branches:
        path = f"/World/{branch}/ToolPlacement/Tool"
        meshes = [
            str(p.GetPath())
            for p in Usd.PrimRange(stage.GetPrimAtPath(path))
            if p.IsA(UsdGeom.Mesh) and p.IsActive()
        ]
        for label, start, end in phases:
            set_tool(stage, branch, *start)
            before = {m: bounds(stage, m) for m in meshes}
            set_tool(stage, branch, *end)
            for mesh in meshes:
                after = bounds(stage, mesh)
                swept = (np.minimum(before[mesh][0], after[0]), np.maximum(before[mesh][1], after[1]))
                for obstacle in obstacles[branch]:
                    rows.append(
                        {
                            "branch": branch,
                            "phase": label,
                            "mesh": mesh,
                            "obstacle": obstacle,
                            "swept_aabb_separation_mm": separation(swept, bounds(stage, obstacle)),
                        }
                    )
        set_tool(stage, branch, -10.5, 11)
    # Compare fixed housing against the cable tail, excluding intended pad contact.
    placement = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Fixture/ToolPlacement"))
    placement.GetOrderedXformOps()[1].Set(180)
    placement.GetOrderedXformOps()[0].Set(Gf.Vec3d(0, 0.0225, 0.03007))
    fixed = "/World/Fixture/ToolPlacement/Tool/fixed"
    tail = bounds(stage, "/World/Fixture/CableBody")
    inline_overlaps = [
        str(p.GetPath())
        for p in Usd.PrimRange(stage.GetPrimAtPath(fixed))
        if p.IsA(UsdGeom.Mesh) and separation(bounds(stage, str(p.GetPath())), tail) < 0
    ]
    placement.GetOrderedXformOps()[1].Set(-90)
    set_tool(stage, "Fixture", -10.5, 11)
    side_gap = min(
        separation(bounds(stage, str(p.GetPath())), tail)
        for p in Usd.PrimRange(stage.GetPrimAtPath(fixed))
        if p.IsA(UsdGeom.Mesh)
    )
    # Recheck rotated pad: now 3 mm across cable and 6 mm along it.
    footprint = review(EndGeometry(11.5, 4, 6), 0, 12, rectangle_half_bounds(3, 6, 10), 0.5, 1)
    report = {
        "scope": "Offline geometry only; prescribed tool movement; fixed reference cable",
        "source_usd_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "inline_housing_tail_possible_collisions": inline_overlaps,
        "side_entry_housing_tail_aabb_clearance_mm": side_gap,
        "fixture_valid_connected_solid": True,
        "fixture_support_height_mm": 30,
        "unsupported_leading_length_mm": 20,
        "pinch_center_setback_mm": 12,
        "pad_across_along_cable_mm": [3, 6],
        "rotated_pad_review": footprint,
        "cup_center_setback_mm": 26,
        "integrated_required_relative_cup_travel_mm": 30,
        "minimum_swept_aabb_separation_mm": min(r["swept_aabb_separation_mm"] for r in rows),
        "possible_obstacle_collisions": [r for r in rows if r["swept_aabb_separation_mm"] < 0],
        "checks": rows,
        "limitations": [
            "No arm/flange, bracket, nozzle mount, actuator or hose envelope",
            "Swept AABBs only qualify listed translation/jaw phases against desk/fixture",
            "Cable deformation, stability, vacuum handoff and delivery not simulated",
            "Full-width uniform tail proxy omits standard-end widening",
            "Contacts/stiffener dimensions and 30 mm support height are assumptions",
            "Fixture fastening, radii, ESD/contact material and release unqualified",
            "Cup is a cylindrical 7x19 mm clearance envelope, not supplier CAD",
            "Integrated axis has no selected actuator or mount; travel alone is not feasibility",
        ],
        "physics_steps": 0,
        "runtime_motion_enabled": False,
    }
    stage.GetRootLayer().Save()
    (out / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    (out / "source.py").write_bytes(Path(__file__).read_bytes())
    print(
        json.dumps(
            {k: v for k, v in report.items() if k not in ["checks", "possible_obstacle_collisions"]}, indent=2
        )
    )
    print("possible collisions", len(report["possible_obstacle_collisions"]))


if __name__ == "__main__":
    main()
