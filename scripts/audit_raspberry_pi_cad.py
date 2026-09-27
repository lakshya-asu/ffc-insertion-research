"""Check the imported Pi PCB against the official outline and hole dimensions.

Requires the CAD environment. Does not certify connector tolerances or appearance.
"""

import json
from pathlib import Path

import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.XCAFDoc import XCAFDoc_DocumentTool

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "third_party/raspberry_pi"


def main():
    report = json.loads((ASSETS / "pi4.json").read_text())
    pcb = next(c for c in report["components"] if c["name"] == "PCB, RPi4ModelB")
    dimensions = np.diff(np.asarray(pcb["bounds_mm"]), axis=0)[0]
    np.testing.assert_allclose(dimensions[:2], [85, 56], atol=0.001)
    reader = STEPCAFControl_Reader()
    reader.ReadFile(str(ASSETS / "pi4-detailed.step"))
    doc = TDocStd_Document(TCollection_ExtendedString("XCAF"))
    assert reader.Transfer(doc)
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    children = TDF_LabelSequence()
    tool.GetComponents_s(roots.Value(1), children)
    shape = tool.GetShape_s(children.Value(1))
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    holes = set()
    while explorer.More():
        surface = BRepAdaptor_Surface(TopoDS.Face_s(explorer.Current()))
        if surface.GetType() == GeomAbs_Cylinder:
            cylinder = surface.Cylinder()
            axis = cylinder.Axis().Direction()
            if abs(cylinder.Radius() - 1.35) < 1e-6 and abs(axis.Z()) > 0.9999:
                p = cylinder.Location()
                holes.add(
                    (round(p.X() - pcb["bounds_mm"][0][0], 5), round(p.Y() - pcb["bounds_mm"][0][1], 5))
                )
        explorer.Next()
    expected = {(3.5, 3.5), (61.5, 3.5), (3.5, 52.5), (61.5, 52.5)}
    assert holes == expected, (holes, expected)
    result = {
        "outline_mm": dimensions[:2].tolist(),
        "outline_check": "pass",
        "hole_diameter_mm": 2.7,
        "hole_centres_from_pcb_lower_left_mm": sorted(holes),
        "hole_pattern_check": "pass",
        "reference": "RP-008343-DS-1",
        "community_pcb_thickness_mm": float(dimensions[2]),
        "connector_check": "uncertified nominal envelope; latch and contact travel unknown",
        "appearance_check": "not training-ready: Pi4 passives/silkscreen missing; materials unmeasured",
        "training_ready": False,
    }
    (ASSETS / "geometry-audit.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
