"""Render and measure the initial state (cable at rest on the pallet) for a variant. usage: FFC_CASE=... python scripts/ffc_zif_initial.py [arm]"""
import sys, os, math
from pathlib import Path
import mujoco, numpy as np
from PIL import Image
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from cell_twin import ffc_zif as F
ARM = sys.argv[1] if len(sys.argv) > 1 else "meca500"; CASE = os.environ.get("FFC_CASE", "laptop"); TOOL = os.environ.get("FFC_TOOL", F.DEFAULT_TOOL[ARM])
spec = F.build(ARM, CASE, TOOL); o = F.connector_origin(ARM); w = spec.worldbody
w.add_camera(name="init_side", pos=[o[0] + 0.12, o[1] + 0.012, o[2] + 0.012], quat=F._look_quat(np.array([o[0] + 0.12, o[1] + 0.012, o[2] + 0.012]), np.array([o[0], o[1] + 0.012, o[2] + 0.006])), fovy=16)
model = spec.compile(); data = mujoco.MjData(model)
info = F.ARM_INFO[ARM]; J = [info["prefix"] + "_" + j for j in info["joints"]]
for j, d in zip(J, info["home"]): data.joint(j).qpos[0] = math.radians(d)
for i, a in enumerate(F.CABLE_REST):
    if CASE == "laptop":
        data.joint(f"ffc_bend{i}").qpos[0] = a; model.qpos_spring[model.joint(f"ffc_bend{i}").qposadr[0]] = a
mujoco.mj_forward(model, data)
for n in range(int(1.0 / F.DT)):
    mujoco.mj_step(model, data)
    for j, d in zip(J, info["home"]): data.joint(j).qpos[0] = math.radians(d); data.joint(j).qvel[0] = 0
mujoco.mj_forward(model, data)
print(f"{ARM} {CASE}: cable nodes relative to the connector mouth (y back, z up), mm")
print("  anchor", np.round((data.body('ffc_anchor').xpos - o)[1:] * 1e3, 1))
for i in range(F.N_SEG):
    print(f"  seg{i} start", np.round((data.body(f'ffc_seg{i}').xpos - o)[1:] * 1e3, 1))
print("  stiffener root", np.round((data.body('ffc_stiffener').xpos - o)[1:] * 1e3, 1), " tip", np.round((data.site('ffc_tip').xpos - o)[1:] * 1e3, 1))
print("  rest bar top z", round((F.PRESENTER_Z) * 1e3, 1), "at y", round(F.PRESENTER_Y * 1e3, 1), "| open lid top would be 6.1, open pad top at dip 30:", round((0.35 + 0.5 + F.PAD_OPEN * 1e3 + 3 + 8 * math.sin(math.radians(30))), 1))
ren = mujoco.Renderer(model, 720, 1080); ren.update_scene(data, camera="init_side"); out = Path("out/ffc_zif") / f"initial_{ARM}_{CASE}.png"; Image.fromarray(ren.render()).save(out); print("  ->", out)
