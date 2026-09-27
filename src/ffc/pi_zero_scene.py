"""Static Zero 2 W asset and asymmetric camera-cable visibility probes."""

import json
import os
from pathlib import Path

import numpy as np
from pxr import Gf, UsdGeom, UsdShade

from ffc.raspberry_pi_scene import make_material


def box(stage, path, position, size, color, collision=False):
    if collision:
        raise ValueError("Static visual probes cannot enable collision")
    prim = UsdGeom.Xform.Define(stage, path)
    prim.AddTranslateOp().Set(Gf.Vec3d(*position))
    shape = UsdGeom.Cube.Define(stage, path + "/Shape")
    shape.CreateSizeAttr(1)
    shape.AddScaleOp().Set(Gf.Vec3f(*size))
    shape.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    return prim.GetPrim()


def add_zero(stage, root, cfg):
    source = root / cfg["asset"]
    board = UsdGeom.Xform.Define(stage, "/World/Hardware/Zero2W")
    board.GetPrim().GetReferences().AddReference(
        os.path.relpath(source, Path(stage.GetRootLayer().realPath).parent)
    )
    board.AddTranslateOp().Set(Gf.Vec3d(*cfg["board_translation_m"]))
    # Four supports stop at the PCB underside; no fixture qualification implied.
    tx, ty, tz = cfg["board_translation_m"]
    for i, (x, y) in enumerate([(-0.029, -0.0123), (0.029, -0.0123), (-0.029, 0.0107), (0.029, 0.0107)]):
        post = UsdGeom.Cylinder.Define(stage, f"/World/Hardware/ZeroSupports/Post{i}")
        post.CreateRadiusAttr(0.0024)
        post.CreateHeightAttr(tz)
        post.AddTranslateOp().Set(Gf.Vec3d(tx + x, ty + y, tz / 2))
        post.CreateDisplayColorAttr([Gf.Vec3f(0.12, 0.13, 0.14)])
    return json.loads(source.with_suffix(".json").read_text())


def make_zero_cable(stage, cfg):
    """Flat visual coupon: +X points away from its 22-pin leading edge."""
    root = "/World/Hardware/ZeroCable"
    UsdGeom.Xform.Define(stage, root)
    spec = cfg["cable"]
    length = spec["length_mm"] / 1000
    narrow, wide = spec["mini_width_mm"] / 1000, spec["standard_width_mm"] / 1000
    transition = spec["assumed_wide_end_length_mm"] / 1000
    body_t = spec["assumed_body_thickness_mm"] / 1000
    header_t = spec["assumed_header_thickness_mm"] / 1000
    exposed = spec["assumed_exposed_length_mm"] / 1000
    support = spec["assumed_support_length_mm"] / 1000
    film = make_material(stage, root + "/Materials/Film", (0.028, 0.032, 0.035), 0.46)
    blue = make_material(stage, root + "/Materials/Stiffener", (0.018, 0.055, 0.24), 0.48)
    metal = make_material(stage, root + "/Materials/Contacts", (0.65, 0.48, 0.17), 0.3, 0.8)
    xs = [0, length - transition - 0.002, length - transition, length]
    widths = [narrow, narrow, wide, wide]
    pts = []
    for x, w in zip(xs, widths, strict=True):
        pts.extend(
            [
                (x, -w / 2, -body_t / 2),
                (x, w / 2, -body_t / 2),
                (x, w / 2, body_t / 2),
                (x, -w / 2, body_t / 2),
            ]
        )
    faces = [0, 3, 2, 1]
    for i in range(3):
        for j in range(4):
            faces.extend([4 * i + j, 4 * i + (j + 1) % 4, 4 * (i + 1) + (j + 1) % 4, 4 * (i + 1) + j])
    faces.extend([12, 13, 14, 15])
    mesh = UsdGeom.Mesh.Define(stage, root + "/Body")
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr([4] * (len(faces) // 4))
    mesh.CreateFaceVertexIndicesAttr(faces)
    mesh.CreateSubdivisionSchemeAttr("none")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(film)
    # Mini contacts face down toward the board; far end face is a declared visual assumption.
    for end, count, pitch, width, x, face in [
        ("Mini", 22, 0.0005, narrow, exposed / 2, -1),
        ("Standard", 15, 0.001, wide, length - exposed / 2, 1),
    ]:
        for i in range(count):
            prim = box(
                stage,
                f"{root}/{end}Contacts/Pin_{i + 1:02d}",
                (x, (i - (count - 1) / 2) * pitch, face * (body_t / 2 + 0.000005)),
                (exposed, pitch * 0.7, 0.00001),
                (0.7, 0.5, 0.2),
                collision=False,
            )
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(metal)
        sx = support / 2 if end == "Mini" else length - support / 2
        stiff = box(
            stage,
            f"{root}/{end}Stiffener",
            (sx, 0, -face * ((header_t - 0.00001) / 2)),
            (support, width, header_t - body_t - 0.00001),
            (0.02, 0.06, 0.3),
            collision=False,
        )
        UsdShade.MaterialBindingAPI.Apply(stiff).Bind(blue)
    return {
        "length_m": length,
        "end_contacts": [22, 15],
        "pitch_m": [0.0005, 0.001],
        "width_m": [narrow, wide],
        "mechanics": "none; flat authored visual probe",
        "unmeasured_dimensions": {k: v for k, v in spec.items() if k.startswith("assumed_")},
    }


def set_probe(stage, cfg, case):
    cable = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Hardware/ZeroCable"))
    cable.ClearXformOpOrder()
    target = np.array(cfg["nominal_mouth_review_point_m"])
    if case == "loose":
        tip = np.array([0.34, -0.23, 0.0003])
    else:
        tip = target + np.array([{"approach": 0.015, "near": 0.003}[case], 0, 0])
    cable.AddTranslateOp().Set(Gf.Vec3d(*tip))
    jaws = UsdGeom.Xform.Define(stage, "/World/Hardware/ZeroJawProbe")
    UsdGeom.Imageable(jaws).MakeInvisible() if case == "loose" else UsdGeom.Imageable(jaws).MakeVisible()
    for i, sign in enumerate([-1, 1]):
        path = f"/World/Hardware/ZeroJawProbe/Jaw{i}"
        if stage.GetPrimAtPath(path):
            stage.RemovePrim(path)
        box(
            stage,
            path,
            tuple(tip + np.array([0.0075, 0, sign * 0.00175])),
            (0.004, 0.012, 0.003),
            (0.16, 0.18, 0.2),
            collision=False,
        )
    return {
        "tip_m": tip.tolist(),
        "case": case,
        "jaw_inner_gap_mm": 0.5,
        "description": "authored visibility probe; no grasp, contact, seating or latch actuation",
    }
