"""Free-plug replay with close-up renders of the connector pair.

usage: python scripts/closeup.py <episode_id> [--frames N]
writes out/closeup_<ep>.png (start | deepest | end) and prints the depth trace summary.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from cell_twin.layout import FPS  # noqa: E402
from cell_twin.scene import compile_model, load_part_offsets  # noqa: E402
from scripts.replay import act_ids, joint_ids, place_plug, seat_depth, set_ctrl, set_qpos  # noqa: E402

EPS = ROOT / "inputs" / "nuc" / "episodes" / "objects"


def render_closeup(m, d, r, lookat, dist=0.045, az=90, el=-15):
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = lookat
    cam.distance = dist
    cam.azimuth = az
    cam.elevation = el
    r.update_scene(d, camera=cam)
    return cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("episode")
    ap.add_argument("--frames", type=int, default=0)
    ap.add_argument("--track", action="store_true", help="targets = measured state, gains x10")
    a = ap.parse_args()
    z = np.load(EPS / a.episode / "frames.npz")
    state, action = z["observation.state"], z["action"]
    n = len(state) if not a.frames else min(a.frames, len(state))
    spec, m = compile_model(part_mode="calibrated", part_offsets=load_part_offsets(), free_plug=True)
    if a.track:
        action = state
        for i in range(m.nu):
            if m.actuator(i).name.endswith(("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")):
                m.actuator_gainprm[i, 0] *= 10
                m.actuator_biasprm[i, 1] *= 10
                m.actuator_biasprm[i, 2] *= 3
    d = mujoco.MjData(m)
    jid, aid = joint_ids(m), act_ids(m)
    substeps = int(round(1.0 / FPS / m.opt.timestep))
    set_qpos(d, jid, state[0])
    mujoco.mj_forward(m, d)
    place_plug(m, d)
    set_ctrl(d, aid, action[0])
    for _ in range(int(0.3 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    r = mujoco.Renderer(m, 360, 480)
    depth, ncon, plugpos = np.zeros(n), np.zeros(n, int), np.zeros((n, 3))
    snaps = {}
    for k in range(n):
        set_ctrl(d, aid, action[k])
        for _ in range(substeps):
            mujoco.mj_step(m, d)
        depth[k] = seat_depth(m, d)
        ncon[k] = d.ncon
        plugpos[k] = d.body("plug").xpos
        if k == 0:
            snaps["start"] = render_closeup(m, d, r, d.site("socket_mouth").xpos)
    kmax = int(np.argmax(depth))
    print(f"frames {n}  depth start {depth[0]*1000:.2f} mm  max {depth.max()*1000:.2f} mm at frame {kmax}  "
          f"end {depth[-1]*1000:.2f} mm  contacts max {ncon.max()}  frames with contact {(ncon>0).sum()}")
    jumps = np.where(np.abs(np.diff(plugpos, axis=0)).max(1) > 0.005)[0]
    print("plug position jumps >5 mm/frame at frames:", jumps[:10], "..." if len(jumps) > 10 else "")
    print("depth every 10 frames (mm):", (depth[::10] * 1000).round(1)[:40])
    snaps["end"] = render_closeup(m, d, r, d.site("socket_mouth").xpos)
    # re-run to the deepest frame for its snapshot
    set_qpos(d, jid, state[0]); mujoco.mj_forward(m, d); place_plug(m, d); set_ctrl(d, aid, action[0])
    for _ in range(int(0.3 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    for k in range(kmax + 1):
        set_ctrl(d, aid, action[k])
        for _ in range(substeps):
            mujoco.mj_step(m, d)
    snaps["deepest"] = render_closeup(m, d, r, d.site("socket_mouth").xpos)
    out = np.hstack([snaps["start"], snaps["deepest"], snaps["end"]])
    cv2.imwrite(str(ROOT / "out" / f"closeup_{a.episode[:8]}.png"), out)
    print("wrote out/closeup_%s.png" % a.episode[:8])


if __name__ == "__main__":
    main()
