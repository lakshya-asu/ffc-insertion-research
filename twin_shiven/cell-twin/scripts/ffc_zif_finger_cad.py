"""Three CAD views of the two fingers with the cable in the jaws, drawn from the generated meshes (jaw frame: x down, y across, z forward)."""
import sys
from pathlib import Path
import numpy as np, trimesh
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
A = Path(__file__).resolve().parent.parent / "assets" / "ffc"; OUT = Path(__file__).resolve().parent.parent / "out" / "ffc_zif"
def load(name, color, shift=(0, 0, 0), R=np.eye(3)):
    m = trimesh.load(A / f"{name}.obj", force="mesh"); v = (m.vertices @ R.T + np.array(shift)) * 1e3; return v, m.faces, color
parts = [load("finger_lower_steel", "#B9BDC4"), load("finger_lower_body", "#EE8C32"),
         load("finger_upper_body", "#EE8C32", shift=(-0.0045, 0, 0)), load("finger_upper_pad", "#444449", shift=(-0.0045, 0, 0))]
Rc = np.column_stack([[0, 1, 0], [0, 0, -1], [1, 0, 0]]).astype(float)       # cable mesh: x -> jaw y, y -> jaw -z, z -> jaw x
parts.append(load("ffc_stiffener", "#3259BE", shift=(-0.00015, 0, 0.011), R=Rc))
# display frame: X = forward (jaw z), Y = across (jaw y), Z = up (-jaw x)
def disp(v): return np.column_stack([v[:, 2], v[:, 1], -v[:, 0]])
views = [("iso", 28, -50), ("side", 0, -90), ("front", 0, 0)]
fig = plt.figure(figsize=(18, 6.5), dpi=110)
for i, (name, el, az) in enumerate(views):
    ax = fig.add_subplot(1, 3, i + 1, projection="3d")
    for v, f, c in parts:
        d = disp(v); ax.add_collection3d(Poly3DCollection(d[f], facecolors=c, edgecolors="#2a2a2a", linewidths=0.25, alpha=1.0))
    ax.set_xlim(-32, 28); ax.set_ylim(-30, 30); ax.set_zlim(-6, 40); ax.set_box_aspect((60, 60, 46))
    ax.view_init(elev=el, azim=az); ax.set_xlabel("forward (mm)"); ax.set_ylabel("across (mm)"); ax.set_zlabel("up (mm)")
    ax.set_title({"iso": "isometric", "side": "side view (along the cable width)", "front": "front view (looking at the lip)"}[name])
fig.suptitle("Printed finger pair for the MEGP 25LS: lower finger with shim-steel shoe and 1.5 mm lip, upper finger with 2 mm pad, one leg each on the east side; cable shown gripped, tip 6 mm proud", fontsize=11)
plt.tight_layout(); fig.savefig(OUT / "fingers_cad.png"); print("saved", OUT / "fingers_cad.png")
