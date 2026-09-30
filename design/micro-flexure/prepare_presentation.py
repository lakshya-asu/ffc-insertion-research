"""Detailed render meshes; edge finishing is a visual proposal, not released CAD."""

import argparse
import json
from pathlib import Path

import numpy as np
from build import BODY, parts
from pxr import Gf, Usd, UsdGeom


def prepare(output):
    output.mkdir(parents=True, exist_ok=False)
    changes = []
    for label, gap in [("closed", 0.14), ("open", 1.2)]:
        stage = Usd.Stage.CreateNew(str(output / f"{label}.usda"))
        root = UsdGeom.Xform.Define(stage, "/Tool")
        stage.SetDefaultPrim(root.GetPrim())
        UsdGeom.SetStageMetersPerUnit(stage, 1)
        UsdGeom.SetStageUpAxis(stage, "Z")
        for p in parts(gap):
            shape = p["shape"]
            radius = 0.12 if p["color"] == BODY else 0.025
            if any(s in p["name"] for s in ["leaf", "pad", "gauge", "reserve", "target", "coil_envelope"]):
                radius = 0
            if radius:
                try:
                    shape = shape.fillet(radius, shape.Edges())
                    changes.append({"pose": label, "part": p["name"], "edge_radius_mm": radius})
                except Exception:
                    changes.append(
                        {
                            "pose": label,
                            "part": p["name"],
                            "edge_radius_mm": 0,
                            "reason": "Fillet not feasible; original geometry retained",
                        }
                    )
            vv, ff = shape.tessellate(0.008, 0.08)
            v = np.array([[x.x, x.y, x.z] for x in vv]) / 1000
            f = np.array(ff)
            tri = v[f]
            face_n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
            face_n /= np.maximum(np.linalg.norm(face_n, axis=1, keepdims=True), 1e-20)
            incident = [[] for _ in v]
            for i, face in enumerate(f):
                for idx in face:
                    incident[idx].append(i)
            normals = []
            for i, face in enumerate(f):
                for idx in face:
                    adjacent = face_n[incident[idx]]
                    n = adjacent[(adjacent @ face_n[i]) > 0.8].sum(axis=0)
                    normals.append((n / max(np.linalg.norm(n), 1e-12)).tolist())
            mesh = UsdGeom.Mesh.Define(stage, "/Tool/" + p["name"])
            mesh.CreatePointsAttr(v.tolist())
            mesh.CreateFaceVertexCountsAttr([3] * len(f))
            mesh.CreateFaceVertexIndicesAttr(f.flatten().tolist())
            mesh.CreateNormalsAttr(normals)
            mesh.SetNormalsInterpolation("faceVarying")
            mesh.CreateSubdivisionSchemeAttr("none")
            mesh.CreateDisplayColorAttr([Gf.Vec3f(*p["color"])])
        stage.GetRootLayer().Save()
    (output / "geometry-scope.json").write_text(
        json.dumps(
            {
                "scope": "Proposed edge finishing for rendering; original CAD retained. No physics.",
                "changes": changes,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    prepare(p.parse_args().output)
