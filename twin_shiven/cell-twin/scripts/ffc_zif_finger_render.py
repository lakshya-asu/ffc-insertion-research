"""CAD-style render of the two printed fingers (and the cable between them) from the generated meshes."""
import sys
from pathlib import Path
import mujoco, numpy as np
from PIL import Image
ROOT = Path(__file__).resolve().parent.parent; A = ROOT / "assets" / "ffc"
spec = mujoco.MjSpec()
spec.visual.global_.offwidth = 1600; spec.visual.global_.offheight = 1000
for n in ("finger_lower_steel", "finger_lower_body", "finger_upper_body", "finger_upper_pad", "ffc_stiffener"):
    m = spec.add_mesh(); m.name = n; m.file = str(A / f"{n}.obj")
w = spec.worldbody
w.add_light(pos=[0.2, -0.3, 0.4], dir=[-0.4, 0.6, -0.7], castshadow=True); w.add_light(pos=[-0.3, 0.2, 0.3], dir=[0.6, -0.4, -0.6], castshadow=False)
w.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[1, 1, 0.1], pos=[0, 0, -0.02], rgba=[0.96, 0.95, 0.92, 1])
# jaw frame of the meshes: x down, y across, z forward. Place it so z (forward) points world -y and x (down) points world -z
R = np.column_stack([[0, 0, -1], [1, 0, 0], [0, -1, 0]]).astype(float)   # jaw x (down) -> world -z, jaw y (across) -> world x, jaw z (forward) -> world -y
q = np.zeros(4); mujoco.mju_mat2Quat(q, R.flatten())
jaw = w.add_body(name="jaw", pos=[0, 0, 0], quat=q.tolist())
jaw.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname="finger_lower_steel", rgba=[0.78, 0.80, 0.84, 1])
jaw.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname="finger_lower_body", rgba=[0.93, 0.55, 0.20, 1])
up = jaw.add_body(name="upper", pos=[-0.0045, 0, 0])          # pad open 4 mm + cable
up.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname="finger_upper_body", rgba=[0.93, 0.55, 0.20, 1])
up.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname="finger_upper_pad", rgba=[0.25, 0.25, 0.27, 1])
Rc = np.column_stack([[0, 1, 0], [0, 0, -1], [1, 0, 0]]).astype(float)   # mesh x -> jaw y, mesh y -> jaw -z (so the tip, at mesh -y, points forward), mesh z -> jaw x
qc = np.zeros(4); mujoco.mju_mat2Quat(qc, Rc.flatten())
cab = jaw.add_body(name="cable", pos=[-0.00015, 0, 0.018 + 0.006 - 0.013], quat=qc.tolist())  # root 13 mm behind the tip; tip 6 mm proud of the jaw line; centre 0.15 mm above the shoe top
cab.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname="ffc_stiffener", rgba=[0.20, 0.35, 0.75, 1])
def cam(name, pos, target, fovy):
    d = np.array(target) - np.array(pos); z = -d / np.linalg.norm(d); x = np.cross([0, 0, 1], z); x /= np.linalg.norm(x); y = np.cross(z, x)
    qq = np.zeros(4); mujoco.mju_mat2Quat(qq, np.array([x, y, z]).T.flatten()); w.add_camera(name=name, pos=list(pos), quat=qq.tolist(), fovy=fovy)
cam("iso", [0.085, -0.085, 0.045], [0, -0.012, 0.010], 32)
cam("side", [0.13, -0.014, 0.012], [0, -0.014, 0.012], 22)
cam("front", [0.0, -0.13, 0.025], [0, -0.012, 0.012], 24)
model = spec.compile(); data = mujoco.MjData(model); mujoco.mj_forward(model, data)
ren = mujoco.Renderer(model, 1000, 1600)
out = ROOT / "out" / "ffc_zif"
for c in ("iso", "side", "front"):
    ren.update_scene(data, camera=c); Image.fromarray(ren.render()).save(out / f"fingers_{c}.png")
print("rendered", out)
