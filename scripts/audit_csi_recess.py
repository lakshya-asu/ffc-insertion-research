"""Offline CAD recess cross-section; not an insertion target or measured clearance."""

import json
from pathlib import Path

import numpy as np
from pxr import Usd, UsdGeom
from scipy.ndimage import label

ROOT = Path(__file__).resolve().parents[1]


def main():
    stage = Usd.Stage.Open(str(ROOT / "outputs/pi4-refined-002/raspberry-pi-workcell.usda"))
    triangles = []
    paths = []
    for prim in stage.Traverse():
        if prim.GetTypeName() != "Mesh" or "Camera Connector" not in str(prim.GetCustomData()):
            continue
        mesh = UsdGeom.Mesh(prim)
        points = np.asarray(mesh.GetPointsAttr().Get())
        matrix = np.asarray(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0))
        world = np.column_stack((points, np.ones(len(points)))) @ matrix
        indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
        triangles.extend(world[indices, :3])
        paths.append(str(prim.GetPath()))
    t = np.asarray(triangles)
    spacing = 0.000025
    xs = np.arange(0.643, 0.64951, spacing)
    ys = np.arange(-0.140, -0.11699, spacing)
    x, y = np.meshgrid(xs, ys)
    points = np.column_stack((x.ravel(), y.ravel()))
    top = np.full(len(points), -np.inf)
    for tri in t:
        a, b, c = tri[:, :2]
        v0, v1 = b - a, c - a
        den = v0[0] * v1[1] - v0[1] * v1[0]
        if abs(den) < 1e-16:
            continue
        relevant = np.flatnonzero(
            np.all(points >= tri[:, :2].min(axis=0), axis=1)
            & np.all(points <= tri[:, :2].max(axis=0), axis=1)
        )
        v2 = points[relevant] - a
        u = (v2[:, 0] * v1[1] - v2[:, 1] * v1[0]) / den
        v = (v0[0] * v2[:, 1] - v0[1] * v2[:, 0]) / den
        inside = (u >= -1e-8) & (v >= -1e-8) & (u + v <= 1 + 1e-8)
        z = tri[0, 2] + u * (tri[1, 2] - tri[0, 2]) + v * (tri[2, 2] - tri[0, 2])
        idx = relevant[inside]
        top[idx] = np.maximum(top[idx], z[inside])
    # Explicit audit plane just below the CAD housing lip. No physical tolerance claim.
    plane_z = 0.0368
    cavity = ((top < plane_z) & (top > 0.0328)).reshape(x.shape)
    ids, _ = label(cavity)
    seed = (np.abs(ys + 0.128).argmin(), np.abs(xs - 0.646).argmin())
    assert ids[seed] != 0
    cavity = ids == ids[seed]
    print("CAVITY", int(cavity.sum()), points[cavity.ravel()].min(axis=0), points[cavity.ravel()].max(axis=0))
    assert cavity.sum() > 100 and not cavity[[0, -1], :].any() and not cavity[:, [0, -1]].any()
    rectangles = []
    for row in range(len(ys)):
        cols = np.flatnonzero(cavity[row])
        if not len(cols):
            continue
        assert np.all(np.diff(cols) == 1)
        rectangles.append(
            [
                float(xs[cols[0]] - spacing / 2),
                float(ys[row] - spacing / 2),
                float(xs[cols[-1]] + spacing / 2),
                float(ys[row] + spacing / 2),
            ]
        )
    occupied = points[cavity.ravel()]
    report = {
        "scope": "offline CAD recess prototype; not a qualified entrance or free insertion path",
        "physical_validation": False,
        "motion_permitted": False,
        "cad_paths": paths,
        "plane_z_m": plane_z,
        "grid_spacing_m": spacing,
        "bounds_xy_m": [occupied.min(axis=0).tolist(), occupied.max(axis=0).tolist()],
        "sample_centroid_xy_m": occupied.mean(axis=0).tolist(),
        "nominal_review_center_x_m": 0.6465,
        "center_x_difference_mm": float((occupied[:, 0].mean() - 0.6465) * 1000),
        "rectangles_xy_m": rectangles,
        "limitations": [
            "Recess extracted from community CAD rest pose; latch state unverified.",
            "Cross-section plane and finite sampling are declared audit choices.",
            "A visible recess is not proof of an open latch or a traversable cable path.",
            "Use only for offline label review; no inference/control input.",
        ],
    }
    (ROOT / "outputs/csi-recess-audit.json").write_text(json.dumps(report, indent=2) + "\n")
    print(
        json.dumps({k: v for k, v in report.items() if k not in ["rectangles_xy_m", "cad_paths"]}, indent=2)
    )


if __name__ == "__main__":
    main()
