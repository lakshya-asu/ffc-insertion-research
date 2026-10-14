"""Interactive MuJoCo viewer of the twin at a recorded pose (or free-running from it).

usage: python scripts/view.py [episode_id] [--frame N] [--run]
Drag to orbit, scroll to zoom, double-click a body to select it, space pauses/resumes.
"""
from __future__ import annotations
import argparse, sys, time
from pathlib import Path
import mujoco, mujoco.viewer, numpy as np
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from cell_twin.scene import compile_model, load_part_offsets
from scripts.replay import joint_ids, act_ids, set_qpos, set_ctrl, place_plug
EPS = ROOT / "inputs" / "nuc" / "episodes" / "objects"
ap = argparse.ArgumentParser(); ap.add_argument("episode", nargs="?", default=None); ap.add_argument("--frame", type=int, default=0); ap.add_argument("--run", action="store_true")
a = ap.parse_args()
ep = a.episode or sorted(p.name for p in EPS.iterdir())[0]
z = np.load(EPS / ep / "frames.npz"); state, action = z["observation.state"], z["action"]
spec, m = compile_model(part_mode="calibrated", part_offsets=load_part_offsets(), free_plug=True)
d = mujoco.MjData(m); jid, aid = joint_ids(m), act_ids(m)
set_qpos(d, jid, state[a.frame]); mujoco.mj_forward(m, d); place_plug(m, d); set_ctrl(d, aid, state[a.frame])
k = a.frame
with mujoco.viewer.launch_passive(m, d) as v:
    v.cam.lookat[:] = d.site("socket_mouth").xpos; v.cam.distance = 0.6; v.cam.azimuth = 150; v.cam.elevation = -20
    sub = int(round(1 / 30 / m.opt.timestep))
    while v.is_running():
        t0 = time.time()
        if a.run:
            set_ctrl(d, aid, state[min(k, len(state) - 1)]); k += 1
        for _ in range(sub):
            mujoco.mj_step(m, d)
        v.sync()
        time.sleep(max(0, 1 / 30 - (time.time() - t0)))
