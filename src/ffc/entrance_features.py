"""Offline engineered socket geometry. Never imported by runtime perception."""

from pxr import Gf, UsdGeom, UsdShade, Vt

from ffc.pi_zero_scene import box
from ffc.raspberry_pi_scene import make_material


def split_front_box(stage, path, center, size, material):
    """Six solid faces; +X face is a separately labelled physical rim surface."""
    x, y, z = center
    a, b, c = [s / 2 for s in size]
    vertices = [
        (x + i * a, y + j * b, z + k * c)
        for i, j, k in [
            (-1, -1, -1),
            (-1, -1, 1),
            (-1, 1, 1),
            (-1, 1, -1),
            (1, -1, -1),
            (1, -1, 1),
            (1, 1, 1),
            (1, 1, -1),
        ]
    ]
    faces = [[0, 3, 2, 1], [0, 1, 5, 4], [3, 7, 6, 2], [0, 4, 7, 3], [1, 2, 6, 5], [4, 5, 6, 7]]
    for name, selected in [("Body", faces[:-1]), ("Front", faces[-1:])]:
        mesh = UsdGeom.Mesh.Define(stage, path + "/" + name)
        mesh.CreatePointsAttr(Vt.Vec3fArray(vertices))
        mesh.CreateFaceVertexCountsAttr([4] * len(selected))
        mesh.CreateFaceVertexIndicesAttr([i for f in selected for i in f])
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)


def make_socket(stage, root, spec):
    """Author a through-mouth housing and a translated slider, in a local frame."""
    UsdGeom.Xform.Define(stage, root)
    housing = make_material(stage, root + "/Materials/Housing", (0.68, 0.67, 0.6), 0.4)
    slider = make_material(stage, root + "/Materials/Slider", (0.025, 0.025, 0.03), 0.36)
    metal = make_material(stage, root + "/Materials/Contacts", (0.62, 0.48, 0.22), 0.26, 0.8)
    width = spec["assumptions"]["slot_width_mm"] / 1000
    gap = spec["assumptions"]["slot_height_open_mm"] / 1000
    # Existing CAD envelope 5 x 16.2 x 1.2 mm. Mouth at local X=0.
    for name, sign in [("Upper", 1), ("Lower", -1)]:
        thick = (0.0012 - gap) / 2 - (0.0001 if sign == 1 else 0)
        split_front_box(
            stage,
            root + "/" + name,
            (-0.0025, 0, sign * (gap / 2 + thick / 2)),
            (0.005, width, thick),
            housing,
        )
    for i, sign in enumerate([-1, 1]):
        p = box(
            stage,
            root + f"/Side{i}",
            (-0.0025, sign * (width / 2 + (0.0162 - width) / 4), 0),
            (0.005, (0.0162 - width) / 2, 0.0012),
            (0.68, 0.67, 0.6),
            collision=False,
        )
        UsdShade.MaterialBindingAPI.Apply(p).Bind(housing)
    p = box(stage, root + "/Back", (-0.0048, 0, 0), (0.0004, width, gap), (0.68, 0.67, 0.6), collision=False)
    UsdShade.MaterialBindingAPI.Apply(p).Bind(housing)
    # Slider is a separately movable part. Travel and closed intrusion are assumptions.
    UsdGeom.Xform.Define(stage, root + "/Slider")
    p = box(
        stage,
        root + "/Slider/Bar",
        (-0.001, 0, 0.00055),
        (0.0008, width - 0.0002, 0.0001),
        (0.025, 0.025, 0.03),
        collision=False,
    )
    UsdShade.MaterialBindingAPI.Apply(p).Bind(slider)
    for i in range(22):
        p = box(
            stage,
            root + f"/Contacts/Pin{i:02d}",
            (-0.002, (i - 10.5) * 0.0005, -gap / 2 + 0.00003),
            (0.002, 0.0003, 0.00006),
            (0.62, 0.48, 0.22),
            collision=False,
        )
        UsdShade.MaterialBindingAPI.Apply(p).Bind(metal)


def set_slider(stage, root, spec, opened):
    slider = UsdGeom.Xformable(stage.GetPrimAtPath(root + "/Slider"))
    slider.ClearXformOpOrder()
    travel = spec["assumptions"]["slider_travel_mm"] / 1000 if opened else 0
    # Open position raises no geometry: the actuator translates parallel to PCB.
    slider.AddTranslateOp().Set(Gf.Vec3d(travel, 0, 0))


def split_cable_cap(stage, root):
    """Move existing leading cap triangles into their own same-material mesh."""
    body = UsdGeom.Mesh(stage.GetPrimAtPath(root + "/Body"))
    points = body.GetPointsAttr().Get()
    counts = list(body.GetFaceVertexCountsAttr().Get())
    indices = list(body.GetFaceVertexIndicesAttr().Get())
    cap = UsdGeom.Mesh.Define(stage, root + "/LeadingEdge")
    cap.CreatePointsAttr(points)
    cap.CreateFaceVertexCountsAttr([counts[0]])
    cap.CreateFaceVertexIndicesAttr(indices[: counts[0]])
    cap.CreateSubdivisionSchemeAttr("none")
    cap.CreateDoubleSidedAttr(True)
    material = UsdShade.Material(stage.GetPrimAtPath(root + "/Materials/Film"))
    UsdShade.MaterialBindingAPI.Apply(cap.GetPrim()).Bind(material)
    body.GetFaceVertexCountsAttr().Set(counts[1:])
    body.GetFaceVertexIndicesAttr().Set(indices[counts[0] :])


def label_leading_band(stage, root, cfg):
    """Partition the first 0.3 mm of the existing stiffener; appearance unchanged."""
    UsdGeom.Imageable(stage.GetPrimAtPath(root + "/MiniStiffener")).MakeInvisible()
    spec = cfg["cable"]
    support = spec["assumed_support_length_mm"] / 1000
    width = spec["mini_width_mm"] / 1000
    body = spec["assumed_body_thickness_mm"] / 1000
    header = spec["assumed_header_thickness_mm"] / 1000
    band = 0.0003
    for name, start, length in [("LeadingBand", 0, band), ("StiffenerRest", band, support - band)]:
        prim = box(
            stage,
            root + "/" + name,
            (start + length / 2, 0, (header - 0.00001) / 2),
            (length, width, header - body - 0.00001),
            (0.018, 0.055, 0.24),
            collision=False,
        )
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            UsdShade.Material(stage.GetPrimAtPath(root + "/Materials/Stiffener"))
        )
