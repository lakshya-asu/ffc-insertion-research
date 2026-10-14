"""Track-mode contact sheet at the housing face for the real-pin connector."""
import sys, numpy as np, mujoco, cv2
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent; sys.path.insert(0, str(ROOT))
from cell_twin import scene as S
from cell_twin.layout import FPS
from scripts.replay import joint_ids, act_ids, set_qpos, set_ctrl, place_plug, seat_depth
from scripts.calibrate import episode_offsets
ep = sys.argv[1]; per = "--per-episode" in sys.argv
z = np.load(ROOT / "inputs/nuc/episodes/objects" / ep / "frames.npz"); st = z["observation.state"]
spec, m = S.compile_model(part_mode="calibrated", part_offsets=episode_offsets(st[-1]) if per else S.load_part_offsets(), free_plug=True)
for i in range(m.nu):
    if m.actuator(i).name.endswith(("joint1","joint2","joint3","joint4","joint5","joint6")):
        m.actuator_gainprm[i,0]*=10; m.actuator_biasprm[i,1]*=10; m.actuator_biasprm[i,2]*=3
d = mujoco.MjData(m); jid, aid = joint_ids(m), act_ids(m)
set_qpos(d, jid, st[0]); mujoco.mj_forward(m, d); place_plug(m, d); set_ctrl(d, aid, st[0])
for _ in range(int(0.3 / m.opt.timestep)): mujoco.mj_step(m, d)
r = mujoco.Renderer(m, 300, 400); sub = int(round(1 / FPS / m.opt.timestep))
def view(az, el, dist=0.02):
    cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE; cam.lookat[:] = d.site("socket_mouth").xpos
    cam.distance = dist; cam.azimuth = az; cam.elevation = el; r.update_scene(d, camera=cam); return cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR)
def pins():
    Ts = np.eye(4); Ts[:3,:3] = d.body("socket").xmat.reshape(3,3); Ts[:3,3] = d.body("socket").xpos; inv = np.linalg.inv(Ts)
    x0 = -(S.N_PINS-1)*S.PITCH/2
    return [((inv @ np.append(d.geom(f"pin_tip_{i}").xpos,1))[:3]*1000 - np.array([(x0+i*S.PITCH)*1000,0,0])).round(2) for i in range(S.N_PINS)]
depth = np.zeros(len(st)); tiles = []; snap_at = set()
for k in range(len(st)):
    set_ctrl(d, aid, st[k])
    for _ in range(sub): mujoco.mj_step(m, d)
    depth[k] = seat_depth(m, d)
    if k % 50 == 0 or (k > 0 and depth[k] > 0 >= depth[k-1]):
        if len(tiles) < 10:
            p = pins(); print(f"frame {k}: depth {depth[k]*1000:6.2f} mm  pin0 tip err (x,y,z) {p[0]}  pin4 {p[4]}")
            t = np.hstack([view(90,-10), view(0,-10), view(90,-80)]); cv2.putText(t, f"frame {k}", (10,25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2); tiles.append(t)
print("max depth", round(depth.max()*1000,2), "final", round(depth[-1]*1000,2))
cv2.imwrite(str(ROOT/"out"/f"pin_sheet_{ep[:8]}.png"), np.vstack(tiles)); print("wrote out/pin_sheet_%s.png" % ep[:8])
