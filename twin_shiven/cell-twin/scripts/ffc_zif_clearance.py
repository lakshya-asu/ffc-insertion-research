"""Clearance audit of a recorded run: switches collision ON for everything the demo ignores (arm links, gripper body, finger legs,
camera bodies) against the laptop, pallet and bench, replays out/ffc_zif/<arm>/traj.npz and reports penetrations and closest approaches per phase.
usage: .venv/bin/python scripts/ffc_zif_clearance.py [meca500|lrmate200id]"""
import sys, math
from pathlib import Path
import mujoco, numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from cell_twin import ffc_zif as F

import os
ARM = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("FFC_ARM", "meca500")
CASE = os.environ.get("FFC_CASE", "laptop"); TOOL = os.environ.get("FFC_TOOL", F.DEFAULT_TOOL[ARM])
OUT = Path(__file__).resolve().parent.parent / "out" / "ffc_zif" / f"{ARM}-{TOOL}-{CASE}"
CHECK = 8
spec = F.build(ARM, CASE, TOOL)
check_names, obstacle_names = [], []
MOVING = ("stack_", "bracket_", "gripper_", "jaw_", "plate_", "leg_", "pad_leg", "pad_arm", "shoe_bar", "macro_", "d405", "arm_d405", "arm_macro", "light_", "arm_light")
OBSTACLES = ("deck", "wall_", "mainboard", "dimm", "mid_rail", "stop_", "presenter", "platform", "feed_", "pallet", "fiducial", "fid_", "coupling", "breadboard", "cam_post", "zif", "ffc")
if F.FIXED_CAMS:   # the macro cameras and the light stand on the bench: they are obstacles the arm must not sweep through
    MOVING = tuple(p for p in MOVING if p not in ("macro_", "light_", "arm_light", "arm_macro")); OBSTACLES = OBSTACLES + ("fixcam", "light_")
for g in spec.geoms:
    n = g.name or ""
    body = g.parent.name if g.parent is not None else ""
    is_arm = body.startswith(F.ARM_INFO[ARM]["prefix"] + "_") or body.startswith(F.ARM_INFO[ARM]["prefix"]) and body not in ("tool", "jaws", "shoe", "pad")
    if n.startswith(MOVING) or (is_arm and n not in ("arm_plate",)):
        g.contype = CHECK; g.conaffinity = 0; g.margin = 0.004; check_names.append(n or f"{body}:geom")
    elif n.startswith(OBSTACLES):
        g.conaffinity = g.conaffinity | CHECK; obstacle_names.append(n)
model = spec.compile(); data = mujoco.MjData(model)
tr = np.load(OUT / "traj.npz"); Q, T, PA, PL = tr["qpos"], tr["t"], tr["phase_at"], tr["phase_lbl"]
gname = lambda i: mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or f"geom{i}"
check_ids = {i for i in range(model.ngeom) if model.geom_contype[i] == CHECK}
print(f"checking {len(check_ids)} geoms (arm links, gripper body, finger legs, cameras) against {len(obstacle_names)} obstacle geoms over {len(Q)} frames")
worst = {}      # phase -> {(a,b): min dist}
for i in range(len(Q)):
    data.qpos[:] = Q[i]; mujoco.mj_forward(model, data)
    k = np.searchsorted(PA, i, side="right") - 1; ph = str(PL[k]) if k >= 0 else "start"
    for c in range(data.ncon):
        con = data.contact[c]
        if con.geom1 in check_ids or con.geom2 in check_ids:
            a, b = gname(con.geom1), gname(con.geom2)
            if a > b: a, b = b, a
            d = worst.setdefault(ph, {})
            if con.dist < d.get((a, b), (9, 0))[0]:
                d[(a, b)] = (con.dist, i)
print()
summary = {}
for ph in [str(p) for p in PL]:
    pairs = sorted(worst.get(ph, {}).items(), key=lambda kv: kv[1][0])[:4]
    if not pairs:
        print(f"{ph:52s} clear by more than 4 mm"); continue
    txt = "; ".join(f"{a} / {b}: {'PENETRATES ' if d < -0.0001 else ''}{d*1e3:+.2f} mm @f{fi}" for (a, b), (d, fi) in pairs)
    summary[ph] = [(a, b, round(d * 1e3, 2)) for (a, b), (d, fi) in pairs]
    print(f"{ph:52s} {txt}")
import json
(OUT / "clearance.json").write_text(json.dumps(summary, indent=1))
