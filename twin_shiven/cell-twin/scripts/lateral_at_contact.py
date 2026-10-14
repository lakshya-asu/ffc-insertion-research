"""For each seated demo: lateral error of the pin row vs the cavities (housing frame) at first face contact
and at the end, in track mode. Tells how much grasp compliance the real cell is using."""
import sys, glob, json, numpy as np, mujoco
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent; sys.path.insert(0, str(ROOT))
from cell_twin import scene as S
from cell_twin.layout import FPS
from scripts.replay import joint_ids, act_ids, set_qpos, set_ctrl, place_plug, seat_depth
PRE = "552d4ea6-1f26-4783-8244-c8ba3eef00fa"
rows = []
for src in sorted(glob.glob(str(ROOT / "inputs/nuc/episodes/objects/*/source.json")))[:12]:
    meta = json.load(open(src))
    if meta.get("starting_pose") != PRE or meta["title"].startswith("Correction"):
        continue
    st = np.load(src.replace("source.json", "frames.npz"))["observation.state"]
    spec, m = S.compile_model(part_mode="calibrated", part_offsets=S.load_part_offsets(), free_plug=True)
    for i in range(m.nu):
        if m.actuator(i).name.endswith(("joint1","joint2","joint3","joint4","joint5","joint6")):
            m.actuator_gainprm[i,0]*=10; m.actuator_biasprm[i,1]*=10; m.actuator_biasprm[i,2]*=3
    d = mujoco.MjData(m); jid, aid = joint_ids(m), act_ids(m)
    set_qpos(d, jid, st[0]); mujoco.mj_forward(m, d); place_plug(m, d); set_ctrl(d, aid, st[0])
    for _ in range(int(0.3 / m.opt.timestep)): mujoco.mj_step(m, d)
    sub = int(round(1 / FPS / m.opt.timestep))
    def lateral():
        Ts = np.eye(4); Ts[:3,:3] = d.body("socket").xmat.reshape(3,3); Ts[:3,3] = d.body("socket").xpos; inv = np.linalg.inv(Ts)
        p = (inv @ np.append(d.geom("pin_tip_2").xpos, 1))[:3] * 1000
        return p[0], p[1], p[2] - S.HOUSING_H / 2 * 1000
    first = None
    for k in range(len(st)):
        set_ctrl(d, aid, st[k])
        for _ in range(sub): mujoco.mj_step(m, d)
        x, y, z = lateral()
        if first is None and z < 0.3:
            first = (k, x, y)
    x, y, z = lateral()
    rows.append((src.split('/')[-2][:8], first, (x, y, z)))
    print(f"{rows[-1][0]}  first face contact frame {first[0] if first else None}: lateral x {first[1]:+.2f} y {first[2]:+.2f} mm   end: x {x:+.2f} y {y:+.2f} depth {z:+.2f}" if first else f"{rows[-1][0]}  never reached the face; end x {x:+.2f} y {y:+.2f} depth {z:+.2f}", flush=True)
