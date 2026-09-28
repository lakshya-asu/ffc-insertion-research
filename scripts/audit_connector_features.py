"""Audit source CAD topology before declaring any insertion features."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from pxr import Usd, UsdGeom
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--asset", type=Path, default=Path("third_party/raspberry_pi/zero/zero2w.usdc"))
p.add_argument("--output", type=Path, required=True)
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=False)
stage = Usd.Stage.Open(str(a.asset))
meshes = [
    prim
    for prim in stage.Traverse()
    if prim.IsA(UsdGeom.Mesh) and "22 Pin FPC Connector" in str(prim.GetCustomDataByKey("cad_component"))
]
assert len(meshes) == 1
prim = meshes[0]
mesh = UsdGeom.Mesh(prim)
points = np.array(mesh.GetPointsAttr().Get())
faces = np.array(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
assert all(n == 3 for n in mesh.GetFaceVertexCountsAttr().Get())
transform = np.array(UsdGeom.XformCache().GetLocalToWorldTransform(prim))
points = (np.c_[points, np.ones(len(points))] @ transform)[:, :3] * 1000
vertices, inverse = np.unique(np.round(points, 5), axis=0, return_inverse=True)
triangles = inverse[faces]
edges = np.concatenate([triangles[:, [0, 1]], triangles[:, [1, 2]], triangles[:, [2, 0]]])
graph = coo_matrix((np.ones(len(edges)), (edges[:, 0], edges[:, 1])), shape=(len(vertices), len(vertices)))
components, _ = connected_components(graph, directed=False)
report = dict(
    asset=str(a.asset),
    asset_sha256=hashlib.sha256(a.asset.read_bytes()).hexdigest(),
    connector_path=str(prim.GetPath()),
    triangles=len(faces),
    weld_tolerance_mm=0.00001,
    connected_components=int(components),
    bounds_mm=[points.min(0).tolist(), points.max(0).tolist()],
    authored_latch_joint=False,
    decision="No independent actuator mesh or joint. Use a documented simulation substitute.",
    manufacturer_url="https://www.molex.com/en-us/products/part-detail/545482271",
    actuator_type="slider",
    limitations=[
        "Connected topology does not prove physical construction",
        "Community mesh is not an aperture-tolerance or contact-surface certification",
        "Cross section is CAD geometry, not a measured specimen",
    ],
)
(a.output / "audit.json").write_text(json.dumps(report, indent=2) + "\n")
im = Image.new("RGB", (1400, 550), "white")
draw = ImageDraw.Draw(im)
for triangle in points[faces]:
    hits = []
    for start, end in zip(triangle, np.roll(triangle, -1, axis=0), strict=True):
        if (start[1] + 0.8) * (end[1] + 0.8) < 0:
            q = start + (-0.8 - start[1]) / (end[1] - start[1]) * (end - start)
            hits.append(((q[0] - 28) * 250 + 30, 500 - (q[2] - 1.4) * 250))
    if len(hits) == 2:
        draw.line(hits, fill="black", width=2)
draw.text(
    (20, 20),
    "CAD center section: board-local X 28..33.3 mm, Z 1.6..2.8 mm; approach from right",
    fill="black",
)
im.save(a.output / "section.png")
print(json.dumps(report, indent=2))
