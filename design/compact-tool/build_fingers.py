"""Build and audit a compact finger concept against hash-checked supplier CAD.

CadQuery environment required. Millimetres in CAD, metres in USD. This is a
kinematic geometry study; no grasp, motor or contact physics is represented.
"""

import argparse
import hashlib
import json
from pathlib import Path

import cadquery as cq
from audit_geometry import SOURCE_SHA256
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

CX, FRONT, MID = 10.2985785382651, -248.2103195204206, 14.3601205939444
LOWER_CENTER = 3.91012059394438
INNER = 8.66012059394438


def box(size, center):
    return cq.Workplane("XY").box(*size).val().translate(center)


def cylinder(radius, length, origin):
    return cq.Solid.makeCylinder(radius, length, cq.Vector(*origin), cq.Vector(0, 1, 0))


def mirror_to_upper(shape):
    return shape.rotate((CX, FRONT, MID), (CX, FRONT + 1, MID), 180)


def build(source, output):
    if hashlib.sha256(source.read_bytes()).hexdigest() != SOURCE_SHA256:
        raise ValueError("Unreviewed supplier source")
    output.mkdir(parents=True, exist_ok=False)
    solids = cq.importers.importStep(str(source)).solids().vals()
    if len(solids) != 21:
        raise ValueError("Unexpected supplier components")
    # 3 mm mounting plate, with margins inside the 10.5 x 9.5 mm jaw face.
    plate = box((10, 3, 9), (CX, FRONT + 1.5, LOWER_CENTER))
    screw_hole = cylinder(1.7, 3.2, (CX, FRONT - 0.1, LOWER_CENTER))
    # Use the verified round locating hole; leave the supplier slot unused.
    pin_x, pin_z = CX + 2**0.5 * 2, LOWER_CENTER - 2**0.5 * 2
    pin_hole = cylinder(1.25, 3.2, (pin_x, FRONT - 0.1, pin_z))
    plate = plate.cut(screw_hole).cut(pin_hole)
    # Blade extends 12 mm from the mounting face, 9 mm beyond the plate.
    blade = box((6, 11, 1.4), (CX, FRONT + 6.5, INNER - 0.5))
    finger = plate.fuse(blade).clean()
    # Contact surface is 0.5 mm inward from the bare jaw's inner plane.
    pad = box((6, 3, 0.3), (CX, FRONT + 10.5, INNER + 0.35))
    # Nominal unthreaded screw envelope; thread engagement is an intentional
    # overlap with the supplier's tap-drill bore and is reported separately.
    screw = cylinder(1.5, 5, (CX, FRONT - 2, LOWER_CENTER)).fuse(
        cylinder(2.75, 3, (CX, FRONT + 3, LOWER_CENTER))
    )
    # A locating-pin concept, not a released tolerance/retention specification.
    pin = cylinder(1.24, 5, (pin_x, FRONT - 2, pin_z))
    lower_parts = {"finger": finger, "pad": pad, "screw_envelope": screw, "locating_pin": pin}
    upper_parts = {name: mirror_to_upper(s) for name, s in lower_parts.items()}
    rows = []
    fixed = cq.Compound.makeCompound(solids[:11])
    base_lower = cq.Compound.makeCompound(solids[11:16])
    base_upper = cq.Compound.makeCompound(solids[16:21])
    lower = cq.Compound.makeCompound(solids[11:16] + list(lower_parts.values()))
    upper = cq.Compound.makeCompound(solids[16:21] + list(upper_parts.values()))
    for gap in sorted({1 + i * 0.5 for i in range(21)} | {1.14, 1.3}):
        shift = (11.4 - gap) / 2
        lo, hi = lower.translate((0, 0, shift)), upper.translate((0, 0, -shift))
        row = {
            "base_gap_mm": gap,
            "pad_gap_mm": gap - 1,
            "opposing_groups_overlap_mm3": abs(lo.intersect(hi).Volume()),
            "lower_fixed_overlap_mm3": abs(lo.intersect(fixed).Volume()),
            "upper_fixed_overlap_mm3": abs(hi.intersect(fixed).Volume()),
        }
        rows.append(row)
        print(row, flush=True)
    fit = {}
    for group, parts, base in [("lower", lower_parts, base_lower), ("upper", upper_parts, base_upper)]:
        fit[group] = {name: abs(shape.intersect(base).Volume()) for name, shape in parts.items()}
    fit_pass = all(
        values[name] < 1e-6 for values in fit.values() for name in ["finger", "pad", "locating_pin"]
    )
    coupon_checks = []
    for width in [11.5, 16]:
        for thickness in [0.14, 0.3]:
            base_gap = thickness + 1
            offset = (11.4 - base_gap) / 2
            reference = box((width, 30, thickness), (CX, FRONT + 9, MID))
            entry = {"width_mm": width, "thickness_mm": thickness, "base_gap_mm": base_gap}
            for name, parts, sign in [("lower", lower_parts, 1), ("upper", upper_parts, -1)]:
                pad_at_contact = parts["pad"].translate((0, 0, sign * offset))
                finger_at_contact = parts["finger"].translate((0, 0, sign * offset))
                entry[name + "_pad_distance_mm"] = pad_at_contact.distance(reference)
                entry[name + "_pad_overlap_mm3"] = abs(pad_at_contact.intersect(reference).Volume())
                entry[name + "_finger_overlap_mm3"] = abs(finger_at_contact.intersect(reference).Volume())
            coupon_checks.append(entry)
    # Separate nominal cable coupons from the tool: no attachment or grasp.
    # At a 1.14 mm base opening the body coupon touches both pad planes.
    coupon = box((11.5, 30, 0.14), (CX, FRONT + 9, MID))
    # 12 mm of cable projects beyond the fingertip's front at y = FRONT+12.
    stage = Usd.Stage.CreateNew(str(output / "compact-fingers.usda"))
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, "Z")
    root = UsdGeom.Xform.Define(stage, "/CompactTool")
    stage.SetDefaultPrim(root.GetPrim())
    root.AddTranslateOp().Set(Gf.Vec3d(-CX / 1000, -FRONT / 1000, -MID / 1000))
    groups = {}
    for group in ["fixed", "lower", "upper"]:
        groups[group] = UsdGeom.Xform.Define(stage, f"/CompactTool/{group}").AddTranslateOp()
    groups["lower"].Set(Gf.Vec3d(0, 0, 0.0002))
    groups["upper"].Set(Gf.Vec3d(0, 0, -0.0002))
    assembly = cq.Assembly(name="compact_finger_concept")
    manifest = []

    def add(shape, name, group, rgb, supplier=False):
        if not shape.isValid():
            raise ValueError(f"Invalid solid: {name}")
        assembly.add(shape, name=name, color=cq.Color(*rgb))
        vertices, triangles = shape.tessellate(0.025, 0.15)
        path = f"/CompactTool/{group}/{name}"
        mesh = UsdGeom.Mesh.Define(stage, path)
        mesh.CreatePointsAttr([(v.x / 1000, v.y / 1000, v.z / 1000) for v in vertices])
        mesh.CreateFaceVertexCountsAttr([3] * len(triangles))
        mesh.CreateFaceVertexIndicesAttr([i for t in triangles for i in t])
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
        mat = UsdShade.Material.Define(stage, path + "/Material")
        shader = UsdShade.Shader.Define(stage, path + "/Material/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
        manifest.append(
            {"name": name, "group": group, "supplier_geometry": supplier, "volume_mm3": shape.Volume()}
        )

    for i, shape in enumerate(solids):
        group = "fixed" if i < 11 else "lower" if i < 16 else "upper"
        add(
            shape,
            f"supplier_{i:02d}",
            group,
            (0.18, 0.20, 0.23) if group == "fixed" else (0.6, 0.63, 0.66),
            True,
        )
    for group, parts in [("lower", lower_parts), ("upper", upper_parts)]:
        for name, shape in parts.items():
            rgb = (
                (0.07, 0.12, 0.15)
                if name == "pad"
                else (0.85, 0.57, 0.16)
                if name == "finger"
                else (0.5, 0.52, 0.55)
            )
            add(shape, f"{group}_{name}", group, rgb)
    add(coupon, "cable_dimensional_coupon", "reference", (0.86, 0.9, 0.94))
    stage.GetRootLayer().Save()
    assembly.save(str(output / "compact-fingers.step"))
    for group, parts in [("lower", lower_parts), ("upper", upper_parts)]:
        cq.exporters.export(parts["finger"], str(output / f"{group}-finger.step"))
        cq.exporters.export(parts["finger"], str(output / f"{group}-finger.stl"))
    report = {
        "status": "GEOMETRY_REVIEW_ONLY_NOT_MANUFACTURING_RELEASED",
        "source_sha256": SOURCE_SHA256,
        "cad_step_pose": "supplier imported 11.4 mm base gap",
        "usd_default_pose": "nominal 11 mm base gap (10 mm pad gap)",
        "dimensions_mm": {
            "plate": [10, 3, 9],
            "blade_width": 6,
            "blade_thickness": 1.4,
            "projection_from_jaw_face": 12,
            "pad": [6, 3, 0.3],
            "inward_offset_each": 0.5,
        },
        "mount": {
            "screw": "M3 x 5 nominal socket-head envelope, no threads/socket modeled",
            "clearance_diameter_mm": 3.4,
            "nominal_engagement_mm": 2,
            "drawing_thread_depth_mm": 3,
            "locating_pin_concept_diameter_mm": 2.48,
            "pin_insertion_mm": 2,
            "drawing_pin_hole_depth_mm": 2.5,
            "pin_retention": "unresolved; do not manufacture from this nominal CAD",
        },
        "parts": manifest,
        "mount_fit_intersections_mm3": fit,
        "mount_fit_pass_excluding_intentional_thread_engagement": fit_pass,
        "coupon_contact_checks": coupon_checks,
        "sweep": rows,
        "sweep_pass": all(r[k] < 1e-6 for r in rows for k in r if k.endswith("overlap_mm3")),
        "limitations": [
            "No threads, tightening, pin retention or manufacturing tolerance analysis",
            "Square blade/pad edges need final radii and material selection",
            "Material colors are review colors, not measured appearance",
            "No cable, actuator, vacuum, friction or damage physics",
            "Coupon is a fixed dimensional reference, never attached or lifted",
            "23 sampled poses do not establish continuous clearance",
            "No nozzle, robot adapter, PCB or camera clearance checked",
        ],
    }
    contact_pass = all(
        value < 1e-6
        for entry in coupon_checks
        for key, value in entry.items()
        if key.endswith(("_distance_mm", "_overlap_mm3"))
    )
    report["coupon_contact_pass"] = contact_pass
    (output / "assembly.json").write_text(json.dumps(report, indent=2) + "\n")
    if not (report["sweep_pass"] and fit_pass and contact_pass):
        raise RuntimeError("Assembled geometry audit failed; retained outputs for review")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    build(args.source, args.output)
