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


def add_assets(stage, project: Path):
    """Place imported CAD in a dedicated namespace; old baseline is untouched."""
    asset_dir = project / "third_party/raspberry_pi"
    report = {"assets": {}, "motion_permitted": False, "collision_ready": False}
    for name, filename in [("Pi4", "pi4.usdc"), ("Camera3", "camera3.usdc")]:
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


def cable_centerline(length_m=0.2, count=801):
    """Author a shallow curved display pose with a specified centreline arc length."""
    u = np.linspace(0, 1, count)
    z = 0.0005 + 0.003 * np.sin(np.pi * u) ** 2
    lo, hi = 0.18, length_m
    for _ in range(45):
        span = (lo + hi) / 2
        arc = np.linalg.norm(np.diff(np.column_stack((span * u, z)), axis=0), axis=1).sum()
        if arc > length_m:
            hi = span
        else:
            lo = span
    return np.column_stack((0.335 + span * u, np.full(count, -0.19), z))


def make_camera_cable(stage, cfg):
    spec = cfg["cable"]
    center = cable_centerline(spec["length_mm"] / 1000)
    distance = np.r_[0, np.cumsum(np.linalg.norm(np.diff(center, axis=0), axis=1))]
    tangent = np.gradient(center, axis=0)
    tangent /= np.linalg.norm(tangent, axis=1)[:, None]
    normal = np.column_stack((-tangent[:, 2], np.zeros(len(center)), tangent[:, 0]))
    root = "/World/Hardware/CameraCable"
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
        points = []
        for c, n in zip(cs, ns, strict=True):
            points.extend([c + [0, y, 0] + z * n for y, z in [(ya, za), (yb, za), (yb, zb), (ya, zb)]])
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
