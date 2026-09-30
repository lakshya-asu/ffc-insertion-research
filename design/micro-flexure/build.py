"""Millimetre concept CAD: a small voice-coil gripper with instrumented fingers.

Run in /tmp/ffc-tool-cad-env. This authors solids and prescribed jaw travel,
not motor/contact physics. Supplier items are clearly labelled envelopes.
"""

import argparse
import hashlib
import json
from pathlib import Path

import cadquery as cq
import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, UsdUtils

HERE = Path(__file__).resolve().parent
STEEL, BODY, COIL = (0.62, 0.68, 0.72), (0.10, 0.25, 0.28), (0.76, 0.40, 0.16)
PAD, GAUGE = (0.09, 0.10, 0.12), (0.91, 0.57, 0.14)


def box(size, center):
    return cq.Workplane("XY").box(*size).val().translate(center)


def cylinder(radius, height, base, direction=(0, 0, 1)):
    return cq.Solid.makeCylinder(radius, height, cq.Vector(*base), cq.Vector(*direction))


def ring(outer, inner, height, base):
    return cylinder(outer, height, base).cut(cylinder(inner, height, base))


def travel(gap):
    delta = 1.2 - gap
    # Leading small-slope shortening term for a guided cubic leaf. This needs FEA.
    return np.array([0.6 * delta**2 / 12, 0, gap - 0.14])


def parts(gap=0.14):
    result = []
    shift = travel(gap)

    def add(name, shape, color, group="fixed", note="Custom nominal geometry"):
        if group == "upper":
            shape = shape.translate(tuple(shift))
        if not shape.isValid():
            raise ValueError("Invalid CAD: " + name)
        result.append(dict(name=name, shape=shape, color=color, group=group, note=note))

    add("base", box((22, 20, 1.3), (27, 0, -1.65)), BODY)
    for y in [-8.5, 8.5]:
        add("rear_post_" + str(int(y > 0)), box((5, 3, 20), (35.5, y, 9)), BODY)
        add("roof_rail_" + str(int(y > 0)), box((22, 3, 1.4), (27, y, 20)), BODY)
    bridge = box((4, 20, 1.4), (26, 0, 20))
    bridge = bridge.cut(cylinder(1.4, 2, (26, 0, 19)))
    add("actuator_mount", bridge, BODY, note="4-40 fastener clearance concept; supplier interface unresolved")
    add(
        "magnet_envelope",
        ring(5.55, 4.55, 9, (26, 0, 9.5)),
        STEEL,
        note="H2W outer envelope; bore/core are provisional, not supplier CAD",
    )
    add(
        "core_envelope",
        cylinder(3.25, 8.8, (26, 0, 9.7)),
        STEEL,
        note="Provisional internal core, excluded from manufacturing definition",
    )
    add("mount_spacer", ring(3, 1.4, 0.8, (26, 0, 18.5)), STEEL)
    add(
        "coil_envelope",
        ring(4.3, 3.5, 3.7, (26, 0, 5.8)),
        COIL,
        "upper",
        "Moving coil/bobbin envelope; flexible leads still to route",
    )
    add("coil_adapter", cylinder(4.3, 0.6, (26, 0, 5.2)), BODY, "upper")
    add("moving_bridge", box((8, 8, 1), (25, 0, 4.7)), BODY, "upper")
    add("moving_crosshead", box((2, 16, 7.03), (22, 0, 3.885)), BODY, "upper")
    add("lower_root", box((4, 6, 0.73), (20, 0, -0.635)), STEEL)
    add("lower_blade", box((14, 6, 0.6), (15, 0, -0.57)), STEEL)
    add("lower_pad", box((2, 6, 0.2), (9.5, 0, -0.17)), PAD)
    add("sensing_leaf", box((10, 4, 0.2), (17, 0, 0.37)), STEEL, "upper")
    add("upper_nose", box((4, 6, 0.6), (10, 0, 0.57)), STEEL, "upper")
    add("upper_pad", box((2, 6, 0.2), (9.5, 0, 0.17)), PAD, "upper")
    for y in [-0.9, 0.9]:
        add(
            "gauge_" + str(int(y > 0)),
            box((2, 0.65, 0.03), (20, y, 0.485)),
            GAUGE,
            "upper",
            "Bonded strain-gauge reserve; actual gauge and amplifier unselected",
        )
    for y in [-3.8, 3.8]:
        add("overtravel_rail_" + str(int(y > 0)), box((8, 1, 0.6), (17, y, 1.3)), BODY, "upper")
    add(
        "sensing_stop",
        box((1, 8.6, 0.4), (13.5, 0, 0.92)),
        BODY,
        "upper",
        "Protects beam overtravel only; NOT a force limiter",
    )
    for index, (y, z) in enumerate([(-7.3, 3), (7.3, 3), (-7.3, 7), (7.3, 7)]):
        xx = np.linspace(22 + shift[0], 34, 25)
        ss = (34 - xx) / (12 - shift[0])
        zz = z + 1.06 - (1.2 - gap) * (3 * ss**2 - 2 * ss**3)
        polygon = [(float(x), float(zv + 0.02)) for x, zv in zip(xx, zz, strict=True)]
        polygon += [(float(x), float(zv - 0.02)) for x, zv in zip(xx[::-1], zz[::-1], strict=True)]
        leaf = cq.Workplane("XZ", origin=(0, y + 0.75, 0)).polyline(polygon).close().extrude(1.5).val()
        add(f"guide_leaf_{index}", leaf, STEEL, "guide", "40 µm steel foil; prescribed cubic shape, not FEA")
        add(f"guide_rear_clamp_{index}", box((2, 2.5, 0.7), (34, y, z + 1.06 + 0.37)), BODY)
    bracket = box((5, 8, 2), (31, 14, 3)).cut(cylinder(1.65, 3, (31, 14, 1.5)))
    add("vacuum_bracket", bracket, BODY)
    add(
        "vacuum_stem",
        ring(1.5, 0.6, 5.5, (31, 14, -1.5)),
        STEEL,
        note="M3 fitting / tube envelope; no threads or pneumatic qualification",
    )
    add(
        "vacuum_lip",
        ring(1.85, 1.25, 1, (31, 14, -4)),
        (0.18, 0.23, 0.26),
        note="SUF 3 cup lip envelope, seal geometry not reproduced",
    )
    add("vacuum_neck", ring(1.3, 0.6, 1.5, (31, 14, -3)), PAD)
    add(
        "position_sensor_reserve",
        box((4, 2, 3), (30, -5.5, 7)),
        (0.2, 0.48, 0.45),
        note="Optical displacement sensor reserve; model and resolution not selected",
    )
    add("position_target", box((1, 1, 3), (29, -3, 6)), STEEL, "upper")
    # Rear attachment holes remain accessible from the robot side.
    for y in [-6, 6]:
        ear = box((3, 4, 6), (36.5, y, 12))
        ear = ear.cut(cylinder(1.1, 4, (34.5, y, 12), (1, 0, 0)))
        add(
            "mount_ear_" + str(int(y > 0)), ear, BODY, note="Nominal M2 clear holes; FR3 adapter not designed"
        )
    return result


def analytics(config):
    e = config["assumed_spring_steel_E_N_mm2"]
    g, b = config["guide"], config["sensing_beam"]
    k = g["count"] * e * g["width_mm"] * g["thickness_mm"] ** 3 / g["length_mm"] ** 3
    inertia = b["width_mm"] * b["thickness_mm"] ** 3 / 12
    length, overhang = b["length_mm"], b["pad_overhang_mm"]
    compliance = (length**3 / 3 + overhang * length**2 + overhang**2 * length) / (e * inertia)
    f = config["exploratory_pad_force_N"]
    flex = f * compliance
    stroke = config["open_pad_gap_mm"] - config["body_cable_thickness_mm"] + flex
    spring = k * stroke
    gravity = config["assumed_moving_mass_kg"] * 9.81
    drive = f + spring + gravity
    current = drive / config["actuator"]["force_constant_N_A"]
    return {
        "method": "Small-deflection beam equations; not FEA or measured force capability",
        "guide_stiffness_N_mm": k,
        "sensing_compliance_mm_N": compliance,
        "sensing_deflection_at_target_mm": flex,
        "required_motion_mm": stroke,
        "guide_restoring_force_N": spring,
        "gravity_allowance_N": gravity,
        "required_coil_force_N": drive,
        "estimated_current_A": current,
        "estimated_copper_power_W": current**2 * config["actuator"]["resistance_ohm"],
        "continuous_force_margin_N": config["actuator"]["continuous_force_N"] - drive,
        "guide_bending_stress_MPa": 3 * e * g["thickness_mm"] * stroke / g["length_mm"] ** 2,
        "sensing_bending_stress_MPa": 6 * f * (length + overhang) / (b["width_mm"] * b["thickness_mm"] ** 2),
        "estimated_parasitic_axis_shift_mm": 0.6 * stroke**2 / g["length_mm"],
        "exclusions": [
            "Coil lead forces",
            "Thermal derating",
            "Gauge bond and pad compliance",
            "Fatigue",
            "Assembly tolerances",
            "Large-deflection stiffness",
            "Dynamics",
        ],
    }


def mesh_data(shape):
    vertices, faces = shape.tessellate(0.06, 0.25)
    return np.array([(v.x, v.y, v.z) for v in vertices]), np.array(faces)


def guide_mesh(index, gap):
    y, z = [(-7.3, 3), (7.3, 3), (-7.3, 7), (7.3, 7)][index]
    shift = travel(gap)
    vertices, faces = [], []
    for x in np.linspace(22 + shift[0], 34, 25):
        s = (34 - x) / (12 - shift[0])
        zz = z + 1.06 - (1.2 - gap) * (3 * s**2 - 2 * s**3)
        vertices.extend(
            [
                [x, y - 0.75, zz - 0.02],
                [x, y + 0.75, zz - 0.02],
                [x, y + 0.75, zz + 0.02],
                [x, y - 0.75, zz + 0.02],
            ]
        )
    for i in range(24):
        for j in range(4):
            a, b, c, d = 4 * i + j, 4 * i + (j + 1) % 4, 4 * (i + 1) + (j + 1) % 4, 4 * (i + 1) + j
            faces.extend([[a, b, c], [a, c, d]])
    faces.extend([[0, 2, 1], [0, 3, 2], [96, 97, 98], [96, 98, 99]])
    return np.array(vertices), np.array(faces)


def build(output):
    output.mkdir(parents=True, exist_ok=False)
    cfg = json.loads((HERE / "parameters.json").read_text())
    (output / "parameters.json").write_text(json.dumps(cfg, indent=2))
    (output / "build.py").write_bytes(Path(__file__).read_bytes())
    report = {"status": cfg["status"], "analytical": analytics(cfg), "checks": [], "parts": []}
    for label, gap in [("closed", 0.14), ("open", 1.2)]:
        assembly = cq.Assembly(name="micro_flexure_" + label)
        for part in parts(gap):
            assembly.add(part["shape"], name=part["name"], color=cq.Color(*part["color"]))
        assembly.save(str(output / (label + ".step")))
    for gap in np.linspace(0.14, 1.2, 12):
        pp = parts(float(gap))
        moving = cq.Compound.makeCompound([p["shape"] for p in pp if p["group"] == "upper"])
        # Attachment joints/coil clearances intentionally not treated as free bodies.
        fixed = cq.Compound.makeCompound(
            [
                p["shape"]
                for p in pp
                if p["name"]
                in {
                    "base",
                    "rear_post_0",
                    "rear_post_1",
                    "roof_rail_0",
                    "roof_rail_1",
                    "lower_blade",
                    "lower_pad",
                    "vacuum_bracket",
                    "position_sensor_reserve",
                }
            ]
        )
        overlap = abs(moving.intersect(fixed).Volume())
        if overlap > 1e-7:
            raise ValueError(f"Unintended solid overlap at gap {gap}: {overlap}")
        report["checks"].append({"gap_mm": float(gap), "moving_selected_fixed_overlap_mm3": overlap})
    stage = Usd.Stage.CreateNew(str(output / "micro-flexure.usda"))
    root = UsdGeom.Xform.Define(stage, "/MicroFlexure")
    stage.SetDefaultPrim(root.GetPrim())
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, "Z")
    stage.SetTimeCodesPerSecond(24)
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(72)
    stage.GetRootLayer().customLayerData = {"scope": "Prescribed geometry animation; no physics"}
    snapshots = {}
    for frame, gap in [(0, 1.2), (24, 0.14), (48, 0.14), (72, 1.2)]:
        snapshots[frame] = parts(gap)
    for part in snapshots[0]:
        path = "/MicroFlexure/" + part["name"]
        mesh = UsdGeom.Mesh.Define(stage, path)
        vertices, faces = mesh_data(part["shape"])
        if part["group"] == "guide":
            vertices, faces = guide_mesh(int(part["name"].split("_")[-1]), 1.2)
        mesh.CreatePointsAttr((vertices / 1000).tolist())
        mesh.CreateFaceVertexCountsAttr([3] * len(faces))
        mesh.CreateFaceVertexIndicesAttr(faces.flatten().tolist())
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*part["color"])])
        mat = UsdShade.Material.Define(stage, path + "/Material")
        shader = UsdShade.Shader.Define(stage, path + "/Material/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*part["color"]))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
        if part["group"] == "upper":
            op = mesh.AddTranslateOp()
            for frame, gap in [(0, 1.2), (24, 0.14), (48, 0.14), (72, 1.2)]:
                op.Set(Gf.Vec3d(*((travel(gap) - travel(1.2)) / 1000)), frame)
        elif part["group"] == "guide":
            for frame, gap in [(0, 1.2), (24, 0.14), (48, 0.14), (72, 1.2)]:
                v, f = guide_mesh(int(part["name"].split("_")[-1]), gap)
                assert np.array_equal(f, faces)
                mesh.GetPointsAttr().Set((v / 1000).tolist(), frame)
        bb = part["shape"].BoundingBox()
        report["parts"].append(
            {k: part[k] for k in ["name", "group", "note"]}
            | {
                "volume_mm3": part["shape"].Volume(),
                "bounds_mm": [[bb.xmin, bb.ymin, bb.zmin], [bb.xmax, bb.ymax, bb.zmax]],
            }
        )
    stage.GetRootLayer().Save()
    UsdUtils.CreateNewUsdzPackage(
        Sdf.AssetPath(str(output / "micro-flexure.usda")), str(output / "micro-flexure.usdz")
    )
    report["source_sha256"] = {
        name: hashlib.sha256((output / name).read_bytes()).hexdigest()
        for name in ["build.py", "parameters.json"]
    }
    (output / "report.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report["analytical"], indent=2))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    build(p.parse_args().output)
