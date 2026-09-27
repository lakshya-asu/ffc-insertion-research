"""Audit Zero 2 W board outline and hole spacing against the official drawing."""

import itertools
import json
from pathlib import Path

import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.XCAFDoc import XCAFDoc_DocumentTool

reader = STEPCAFControl_Reader()
reader.ReadFile("third_party/raspberry_pi/zero/optocam.step")
doc = TDocStd_Document(TCollection_ExtendedString("XCAF"))
reader.Transfer(doc)
tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
roots = TDF_LabelSequence()
tool.GetFreeShapes(roots)
found = []


def visit(label):
    if tool.IsReference_s(label):
        ref = TDF_Label()
        tool.GetReferredShape_s(label, ref)
        visit(ref)
        return
    name = TDataStd_Name()
    n = name.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), name) else ""
    if n == "PCB, Raspberry Pi Zero 2 W":
        shape = tool.GetShape_s(label)
        ex = TopExp_Explorer(shape, TopAbs_FACE)
        while ex.More():
            surf = BRepAdaptor_Surface(TopoDS.Face_s(ex.Current()))
            if surf.GetType() == GeomAbs_Cylinder:
                c = surf.Cylinder()
                p = c.Location()
                if abs(c.Radius() - 1.35) < 1e-5:
                    found.append([p.X(), p.Y(), p.Z(), c.Radius()])
            ex.Next()
        return
    if tool.IsAssembly_s(label):
        children = TDF_LabelSequence()
        tool.GetComponents_s(label, children)
        for i in range(1, children.Length() + 1):
            visit(children.Value(i))


for i in range(1, roots.Length() + 1):
    visit(roots.Value(i))


report = json.loads(Path("third_party/raspberry_pi/zero/zero2w.json").read_text())
pcb = next(c for c in report["components"] if c["name"] == "SOLID")
outline = np.diff(pcb["bounds_mm"], axis=0)[0][:2]
np.testing.assert_allclose(outline, [65, 30], atol=1e-5)
centers = np.unique(np.round(np.asarray(found)[:, :3], 6), axis=0)
assert len(centers) == 4
pairwise = sorted(float(np.linalg.norm(a - b)) for a, b in itertools.combinations(centers, 2))
np.testing.assert_allclose(pairwise, sorted([23, 23, 58, 58, np.hypot(23, 58), np.hypot(23, 58)]), atol=1e-5)
result = {
    "outline_mm": outline.tolist(),
    "mounting_hole_diameter_mm": 2.7,
    "mounting_hole_pitch_mm": [58, 23],
    "outline_and_hole_pattern": "pass",
    "source_sha256": report["source_sha256"],
    "component_count": report["component_count"],
    "triangle_count": report["triangle_count"],
    "reference": "official Zero 2 W mechanical drawing",
    "connector": "CAD names Molex 54548-2271; source BOM and aperture tolerances unverified",
    "limitations": [
        "Board dimensions do not certify connector clearance or latch state",
        "CAD PCB thickness 1.6 mm is not independently certified by this outline drawing",
    ],
}
Path("third_party/raspberry_pi/zero/audit.json").write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps(result, indent=2))
