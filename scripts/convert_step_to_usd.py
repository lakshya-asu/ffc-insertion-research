"""Convert STEP assemblies to metre-scale USD; retain component names and CAD colours.

Run with requirements-cad.txt in a separate CPU Python 3.12 environment.
Visual geometry only: this does not invent collision, joint, or material mechanics.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.Quantity import Quantity_Color
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFDoc import XCAFDoc_ColorGen, XCAFDoc_ColorSurf, XCAFDoc_DocumentTool
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade


def convert(
    source: Path,
    output: Path,
    deflection_mm: float = 0.015,
    split_connectors: bool = False,
    assembly_name: str | None = None,
):
    reader = STEPCAFControl_Reader()
    reader.SetColorMode(True)
    reader.SetNameMode(True)
    if reader.ReadFile(str(source)) != IFSelect_RetDone:
        raise ValueError(f"Cannot read STEP: {source}")
    doc = TDocStd_Document(TCollection_ExtendedString("XCAF"))
    if not reader.Transfer(doc):
        raise ValueError("STEP transfer failed")
    shapes = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    colours = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(output))
    root = UsdGeom.Xform.Define(stage, "/Asset")
    stage.SetDefaultPrim(root.GetPrim())
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, "Z")
    records, materials = [], {}

    def colour(item, default):
        for kind in (XCAFDoc_ColorSurf, XCAFDoc_ColorGen):
            c = Quantity_Color()
            found = (
                colours.GetColor_s(item, kind, c)
                if isinstance(item, TDF_Label)
                else colours.GetColor(item, kind, c)
            )
            if found:
                return (c.Red(), c.Green(), c.Blue())
        return default

    def material(rgb):
        key = tuple(round(v, 5) for v in rgb)
        if key not in materials:
            path = f"/Asset/Materials/M{len(materials):03d}"
            mat = UsdShade.Material.Define(stage, path)
            shader = UsdShade.Shader.Define(stage, path + "/Surface")
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.42)
            mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
            materials[key] = mat
        return materials[key]

    def label_name(label):
        attr = TDataStd_Name()
        return attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), attr) else "unnamed"

    def visit(label, location, names, inherited=(0.5, 0.5, 0.5), solid=None, solid_index=None):
        name = label_name(label)
        rgb = colour(label, inherited)
        if shapes.IsReference_s(label):
            referred = TDF_Label()
            shapes.GetReferredShape_s(label, referred)
            visit(referred, location.Multiplied(shapes.GetLocation_s(label)), names + [name], rgb)
            return
        if shapes.IsAssembly_s(label):
            children = TDF_LabelSequence()
            shapes.GetComponents_s(label, children)
            for i in range(1, children.Length() + 1):
                visit(children.Value(i), location, names + [name], rgb)
            return
        shape = solid if solid is not None else shapes.GetShape_s(label)
        if (
            split_connectors
            and solid is None
            and any(n in name for n in ["Camera Connector", "Display Connector"])
        ):
            solids = TopExp_Explorer(shape, TopAbs_SOLID)
            index = 0
            while solids.More():
                visit(label, location, names, rgb, solids.Current(), index)
                index += 1
                solids.Next()
            if index != 17:
                raise ValueError(f"Expected 15 contacts and two polymer solids, got {index}: {name}")
            return
        if shape.IsNull():
            return
        mesher = BRepMesh_IncrementalMesh(shape, deflection_mm, False, 0.15, True)
        if not mesher.IsDone():
            raise ValueError(f"Meshing failed: {name}")
        points, indices, groups = [], [], {}
        explorer = TopExp_Explorer(shape, TopAbs_FACE)
        face_number = 0
        while explorer.More():
            face = TopoDS.Face_s(explorer.Current())
            loc = TopLoc_Location()
            tri = BRep_Tool.Triangulation_s(face, loc)
            if tri is None:
                raise ValueError(f"Missing face triangulation: {name}")
            transform = location.Multiplied(loc).Transformation()
            offset = len(points)
            for j in range(1, tri.NbNodes() + 1):
                p = tri.Node(j).Transformed(transform)
                points.append((p.X() * 0.001, p.Y() * 0.001, p.Z() * 0.001))
            face_rgb = colour(face, rgb)
            key = tuple(round(v, 5) for v in face_rgb)
            for j in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(j).Get()
                if face.Orientation() == TopAbs_REVERSED:
                    b, c = c, b
                indices.extend([offset + a - 1, offset + b - 1, offset + c - 1])
                groups.setdefault(key, []).append(face_number)
                face_number += 1
            explorer.Next()
        if not points:
            return
        path = f"/Asset/Components/Part_{len(records):04d}"
        mesh = UsdGeom.Mesh.Define(stage, path)
        mesh.CreatePointsAttr(points)
        mesh.CreateFaceVertexCountsAttr([3] * face_number)
        mesh.CreateFaceVertexIndicesAttr(indices)
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(False)
        mesh.GetPrim().SetCustomDataByKey("cad_component", " / ".join(names + [name]))
        if solid_index is not None:
            mesh.GetPrim().SetCustomDataByKey("cad_solid_index", solid_index)
        mesh.CreateDisplayColorAttr([rgb])
        for k, (face_rgb, faces) in enumerate(groups.items()):
            subset = UsdGeom.Subset.CreateGeomSubset(
                mesh, f"Colour_{k}", "face", faces, "materialBind", "partition"
            )
            UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(material(face_rgb))
        arr = np.asarray(points)
        records.append(
            {
                "path": path,
                "name": name,
                "solid_index": solid_index,
                "assembly": names,
                "triangles": face_number,
                "bounds_mm": [arr.min(axis=0).tolist(), arr.max(axis=0).tolist()],
                "cad_colours": [list(c) for c in groups],
            }
        )
        records[-1]["bounds_mm"] = (np.asarray(records[-1]["bounds_mm"]) * 1000).tolist()

    labels = TDF_LabelSequence()
    shapes.GetFreeShapes(labels)
    selected = []

    def find_assembly(label):
        if shapes.IsReference_s(label):
            referred = TDF_Label()
            shapes.GetReferredShape_s(label, referred)
            find_assembly(referred)
        elif label_name(label) == assembly_name:
            selected.append(label)
        elif shapes.IsAssembly_s(label):
            children = TDF_LabelSequence()
            shapes.GetComponents_s(label, children)
            for j in range(1, children.Length() + 1):
                find_assembly(children.Value(j))

    if assembly_name:
        for i in range(1, labels.Length() + 1):
            find_assembly(labels.Value(i))
        if len(selected) != 1:
            raise ValueError(f"Expected one assembly named {assembly_name!r}, found {len(selected)}")
    else:
        selected = [labels.Value(i) for i in range(1, labels.Length() + 1)]
    for label in selected:
        visit(label, TopLoc_Location(), [])
    bounds = np.asarray([r["bounds_mm"] for r in records])
    report = {
        "source_file": source.name,
        "selected_assembly": assembly_name,
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "units": "metres in USD; millimetres in report",
        "deflection_mm": deflection_mm,
        "angular_deflection_rad": 0.15,
        "component_count": len(records),
        "triangle_count": sum(r["triangles"] for r in records),
        "bounds_mm": [bounds[:, 0].min(axis=0).tolist(), bounds[:, 1].max(axis=0).tolist()],
        "materials": "CAD colours retained; roughness 0.42 is an unmeasured rendering assumption",
        "physics": "none: visual assembly only",
        "components": records,
    }
    stage.GetRootLayer().customLayerData = {
        "source_sha256": report["source_sha256"],
        "geometry_role": "visual_only",
    }
    stage.GetRootLayer().Save()
    output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k != "components"}, indent=2))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("source", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--deflection-mm", type=float, default=0.015)
    p.add_argument("--split-connectors", action="store_true")
    p.add_argument("--assembly-name", help="Extract exactly one named assembly in its own local frame")
    a = p.parse_args()
    convert(a.source, a.output, a.deflection_mm, a.split_connectors, a.assembly_name)
