"""Pi 4 + official Camera Module 3 visual assets and dimensioned 15-way FFC.

No motion, rewards, task-state estimator or collision approximation is supplied.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdShade


def make_material(stage, path, colour, roughness=0.45, metallic=0.0):
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*colour))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def add_assets(stage, project: Path, detailed=False):
    """Place imported CAD in a dedicated namespace; old baseline is untouched."""
    asset_dir = project / "third_party/raspberry_pi"
    report = {"assets": {}, "motion_permitted": False, "collision_ready": False}
    for name, filename in [("Pi4", "pi4-v2.usdc" if detailed else "pi4.usdc"), ("Camera3", "camera3.usdc")]:
        source = asset_dir / filename
        if not source.exists():
            raise FileNotFoundError(f"Convert STEP before scene construction: {source}")
        prim = UsdGeom.Xform.Define(stage, "/World/Hardware/" + name)
        prim.GetPrim().GetReferences().AddReference(
            os.path.relpath(source, Path(stage.GetRootLayer().realPath).parent)
        )
        if name == "Pi4":
            # Source PCB lower-left/bottom = (-42.5, -28.352069, -1.152069) mm.
            prim.AddTranslateOp().Set(Gf.Vec3d(0.6425, -0.11164793078422418, 0.031152069215775825))
        else:
            # Official camera STEP has optical axis -Z. Turn lens upward for desk review.
            prim.AddTranslateOp().Set(Gf.Vec3d(0.478, -0.0675, 0.004))
            prim.AddRotateXOp().Set(180)
        report["assets"][name] = json.loads(source.with_suffix(".json").read_text())
    report["board_lower_left_bottom_m"] = [0.60, -0.14, 0.03]
    return report


def cable_centerline(length_m=0.2, count=801, height_m=0.003, lateral_m=0.0):
    """Author a shallow curved display pose with a specified centreline arc length."""
    u = np.linspace(0, 1, count)
    z = 0.0005 + height_m * np.sin(np.pi * u) ** 2
    y = lateral_m * np.sin(np.pi * u)
    lo, hi = 0.18, length_m
    for _ in range(45):
        span = (lo + hi) / 2
        arc = np.linalg.norm(np.diff(np.column_stack((span * u, y, z)), axis=0), axis=1).sum()
        if arc > length_m:
            hi = span
        else:
            lo = span
    return np.column_stack((0.335 + span * u, -0.19 + y, z))


def make_camera_cable(stage, cfg, root="/World/Hardware/CameraCable"):
    spec = cfg["cable"]
    center = cable_centerline(
        spec["length_mm"] / 1000,
        height_m=spec.get("curve_height_m", 0.003),
        lateral_m=spec.get("lateral_bow_m", 0.0),
    )
    distance = np.r_[0, np.cumsum(np.linalg.norm(np.diff(center, axis=0), axis=1))]
    tangent = np.gradient(center, axis=0)
    tangent /= np.linalg.norm(tangent, axis=1)[:, None]
    side = np.cross(np.tile([0.0, 0.0, 1.0], (len(center), 1)), tangent)
    side /= np.linalg.norm(side, axis=1)[:, None]
    normal = np.cross(tangent, side)
    if not spec.get("face_a_up", True):
        normal *= -1
        side *= -1
    UsdGeom.Xform.Define(stage, root)
    mats = {
        "film": make_material(stage, root + "/Materials/Film", (0.78, 0.79, 0.73), 0.36),
        "blue": make_material(stage, root + "/Materials/Support", (0.015, 0.06, 0.38), 0.52),
        "metal": make_material(stage, root + "/Materials/Contacts", (0.68, 0.69, 0.67), 0.3, 0.85),
    }
    width = spec["width_mm"] / 1000
    conductor = spec["conductor_thickness_mm"] / 1000
    film = spec["insulation_layer_thickness_mm"] / 1000
    body = spec["body_thickness_mm"] / 1000
    exposed = spec["exposed_contact_length_mm"] / 1000
    support = spec["support_tape_length_mm"] / 1000
    length = spec["length_mm"] / 1000
    header = spec["contact_header_thickness_mm"] / 1000

    def strip(name, start, end, ya, yb, za, zb, material):
        # Include exact longitudinal feature boundaries by interpolating the authored curve.
        ds = np.r_[start, distance[(distance > start) & (distance < end)], end]
        cs = np.column_stack([np.interp(ds, distance, center[:, k]) for k in range(3)])
        ns = np.column_stack([np.interp(ds, distance, normal[:, k]) for k in range(3)])
        ns /= np.linalg.norm(ns, axis=1)[:, None]
        sides = np.column_stack([np.interp(ds, distance, side[:, k]) for k in range(3)])
        sides /= np.linalg.norm(sides, axis=1)[:, None]
        points = []
        for c, n, lateral in zip(cs, ns, sides, strict=True):
            points.extend([c + y * lateral + z * n for y, z in [(ya, za), (yb, za), (yb, zb), (ya, zb)]])
        faces = [0, 3, 2, 1]
        for i in range(len(cs) - 1):
            a, b = 4 * i, 4 * (i + 1)
            for j in range(4):
                faces.extend([a + j, a + (j + 1) % 4, b + (j + 1) % 4, b + j])
        faces.extend([4 * (len(cs) - 1) + j for j in range(4)])
        mesh = UsdGeom.Mesh.Define(stage, root + "/" + name)
        mesh.CreatePointsAttr(np.asarray(points).tolist())
        mesh.CreateFaceVertexCountsAttr([4] * (len(faces) // 4))
        mesh.CreateFaceVertexIndicesAttr(faces)
        mesh.CreateSubdivisionSchemeAttr("none")
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mats[material])

    strip("Body", exposed, length - exposed, -width / 2, width / 2, -body / 2, body / 2, "film")
    # Opposite exposed faces as shown in the official standard-standard drawing.
    strip("TipAInsulation", 0, exposed, -width / 2, width / 2, -conductor / 2 - film, -conductor / 2, "film")
    strip(
        "TipBInsulation",
        length - exposed,
        length,
        -width / 2,
        width / 2,
        conductor / 2,
        conductor / 2 + film,
        "film",
    )
    strip(
        "TipASupport",
        0,
        support,
        -width / 2,
        width / 2,
        conductor / 2 - header,
        -conductor / 2 - film,
        "blue",
    )
    strip(
        "TipBSupport",
        length - support,
        length,
        -width / 2,
        width / 2,
        conductor / 2 + film,
        header - conductor / 2,
        "blue",
    )
    cw = spec["conductor_width_mm"] / 1000
    pitch = spec["pitch_mm"] / 1000
    for i in range(spec["contacts"]):
        y = (i - (spec["contacts"] - 1) / 2) * pitch
        for end, start, stop in [("A", 0, exposed), ("B", length - exposed, length)]:
            strip(
                f"Contacts{end}/Pin_{i + 1:02d}",
                start,
                stop,
                y - cw / 2,
                y + cw / 2,
                -conductor / 2,
                conductor / 2,
                "metal",
            )
    return {
        "centerline_arc_length_m": float(distance[-1]),
        "width_m": width,
        "contacts_per_end": spec["contacts"],
        "contact_sides": "opposite",
        "physics": "static authored pose; no elastic or contact solver",
        "dimensions_source": "RP-008146-DS-1; body thickness derived from two film layers",
        "appearance": "unmeasured film, support and contact rendering materials",
    }


def apply_review_materials(stage):
    """Explicit, uncalibrated appearance overrides; geometry stays CAD-derived."""
    mats = {
        "pcb": make_material(stage, "/World/Hardware/ReviewMaterials/SolderMask", (0.018, 0.16, 0.035), 0.43),
        "black": make_material(
            stage, "/World/Hardware/ReviewMaterials/BlackPolymer", (0.012, 0.014, 0.016), 0.46
        ),
        "metal": make_material(
            stage, "/World/Hardware/ReviewMaterials/ShieldMetal", (0.55, 0.57, 0.60), 0.28, 0.9
        ),
    }
    applied = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Subset) or not str(prim.GetPath()).startswith("/World/Hardware/"):
            continue
        mesh = prim.GetParent()
        name = mesh.GetCustomDataByKey("cad_component") or ""
        bound, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
        if not bound:
            continue
        shader = UsdShade.Shader(stage.GetPrimAtPath(str(bound.GetPath()) + "/Surface"))
        value = shader.GetInput("diffuseColor").Get()
        kind = None
        if "PCB, RPi4ModelB" in name and value[1] > value[0] + 0.2:
            kind = "pcb"
        elif "PIN HEADER" in name and max(value) - min(value) < 0.01:
            kind = "black"
        elif any(n in name for n in ["BCM2711", "Wireless Module Cover"]) and max(value) - min(value) < 0.01:
            kind = "metal"
        elif "/Camera3/" in str(prim.GetPath()) and "/Part_0162/" in str(prim.GetPath()):
            kind = "black"
        if kind:
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(mats[kind])
            applied.append({"subset": str(prim.GetPath()), "material": kind})
    return {
        "basis": "engineering appearance choices, not measured BRDF or calibrated camera response",
        "overrides": applied,
    }


def refine_pi4(stage, project):
    """Add drawing-based visual detail and materialise existing connector solids."""
    root = "/World/Hardware/Pi4"
    # Local CAD coordinates; the Pi4 parent carries the scene placement.
    origin = np.array([-0.0425, -0.0283520692157758, 0.00044793078422418])
    materials = {
        "ceramic": make_material(stage, root + "/DetailMaterials/Ceramic", (0.20, 0.13, 0.055), 0.6),
        "resistor": make_material(stage, root + "/DetailMaterials/Resistor", (0.014, 0.016, 0.018), 0.52),
        "solder": make_material(stage, root + "/DetailMaterials/Solder", (0.52, 0.55, 0.58), 0.32, 0.9),
        "ivory_polymer": make_material(stage, root + "/DetailMaterials/Housing", (0.72, 0.70, 0.60), 0.42),
        "black_polymer": make_material(stage, root + "/DetailMaterials/Latch", (0.012, 0.013, 0.014), 0.4),
        "silk": make_material(stage, root + "/DetailMaterials/Silk", (0.73, 0.77, 0.69), 0.7),
    }
    layout = json.loads((project / "config/pi4-detail-layout.json").read_text())
    for i, package in enumerate(layout["packages"]):
        x, y = np.array(package["xy_mm"]) / 1000
        dx, dy = np.array(package["size_xy_mm"]) / 1000
        height = float(np.clip(min(dx, dy) * 0.5, 0.00025, 0.0007))
        center = origin + [x, y, height / 2]
        path = root + f"/DrawingDetails/Package_{i:03d}"
        body = UsdGeom.Cube.Define(stage, path + "/Body")
        body.CreateSizeAttr(1)
        body.AddTranslateOp().Set(Gf.Vec3d(*center))
        body.AddScaleOp().Set(Gf.Vec3f(dx, dy, height))
        UsdShade.MaterialBindingAPI.Apply(body.GetPrim()).Bind(materials["ceramic" if i % 3 else "resistor"])
        major = 0 if dx >= dy else 1
        for j, sign in enumerate([-1, 1]):
            pos = center.copy()
            pos[major] += sign * [dx, dy][major] * 0.38
            size = np.array([dx, dy, height + 0.000015])
            size[major] *= 0.24
            cap = UsdGeom.Cube.Define(stage, path + f"/Termination{j}")
            cap.CreateSizeAttr(1)
            cap.AddTranslateOp().Set(Gf.Vec3d(*pos))
            cap.AddScaleOp().Set(Gf.Vec3f(*size))
            UsdShade.MaterialBindingAPI.Apply(cap.GetPrim()).Bind(materials["solder"])
    labels = json.loads((project / "config/pi4-silkscreen.json").read_text())
    for i, label in enumerate(labels["labels"]):
        xy = np.asarray(label["xy_mm"]) / 1000
        xyz = np.column_stack((xy, np.full(len(xy), 0.000015))) + origin
        mesh = UsdGeom.Mesh.Define(stage, root + f"/Silkscreen/Label_{i:02d}")
        mesh.CreatePointsAttr(xyz.tolist())
        mesh.CreateFaceVertexCountsAttr([3] * (len(label["triangles"]) // 3))
        mesh.CreateFaceVertexIndicesAttr(label["triangles"])
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(materials["silk"])
    connectors = {"csi": [], "dsi": []}
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh) or not str(prim.GetPath()).startswith(root + "/Components/"):
            continue
        name = prim.GetCustomDataByKey("cad_component") or ""
        index = prim.GetCustomDataByKey("cad_solid_index")
        if index is None:
            continue
        role = "contact" if index < 15 else "black_polymer" if index == 15 else "ivory_polymer"
        kind = "csi" if "Camera Connector" in name else "dsi"
        for subset in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Mesh(prim)):
            UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(
                materials["solder" if role == "contact" else role]
            )
        prim.SetCustomDataByKey("visual_role", role)
        connectors[kind].append({"path": str(prim.GetPath()), "role": role})
    return {
        "packages": layout["count"],
        "silkscreen_labels": len(labels["labels"]),
        "connectors": connectors,
        "provenance": (
            "XY footprints from official drawing; package heights/types, markings and finishes approximate"
        ),
        "latch_status": "CAD rest pose; no latch travel or contact mechanics claimed",
    }
