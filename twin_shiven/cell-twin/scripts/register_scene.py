"""Register the top-down scene camera from the real last frame of one demo (bases already registered).
Picks (640x480): the fingertip pads of each arm and each gripper's carriage block. The camera is solved for
position + orientation (6 DOF) with a weak pull toward straight-down, field of view discrete.
writes out/register_scene.json and out/camgrid/scene_registered.png"""
import sys, json, numpy as np, mujoco, cv2
from scipy.optimize import least_squares
sys.path.insert(0, "."); import cell_twin.scene as S
from cell_twin.scene import compile_model, load_part_offsets
from scripts.replay import joint_ids, set_qpos
EP = "inputs/nuc/episodes/objects/09710851-e26c-43ef-b7ed-0a684c28e4a4"; st = np.load(f"{EP}/frames.npz")["observation.state"][-1]
PICK = {"left_pads": (395, 266), "right_pads": (430, 268), "left_plate": (80, 255), "right_plate": (600, 255)}
spec, m = compile_model(part_mode="calibrated", part_offsets=load_part_offsets()); d = mujoco.MjData(m); set_qpos(d, joint_ids(m), st); mujoco.mj_forward(m, d)
P = {"left_pads": (d.geom_xpos[m.geom("left_left_pad").id] + d.geom_xpos[m.geom("left_right_pad").id]) / 2,
     "right_pads": (d.geom_xpos[m.geom("right_left_pad").id] + d.geom_xpos[m.geom("right_right_pad").id]) / 2,
     "left_plate": d.geom_xpos[m.geom("left_gripper_housing").id], "right_plate": d.geom_xpos[m.geom("right_gripper_housing").id]}
print({k: np.round(v, 4).tolist() for k, v in P.items()})
def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
def cam_R(x): return rpy(x[3], x[4], x[5])          # identity = straight down, image up = +y world
def project(x, p, f):
    R = cam_R(x); v = R.T @ (p - x[:3])
    if v[2] > -0.01: return np.array([1e3, 1e3])
    return np.array([320 + f * v[0] / (-v[2]), 240 - f * v[1] / (-v[2])])
def residuals(x, f):
    res = [(project(x, P[k], f) - np.array(PICK[k], float)) / 10.0 for k in PICK]
    return np.concatenate(res + [np.array([x[3], x[4]]) / np.radians(15)])     # weak prior: within ~15 deg of nadir
best = None
mid = (P["left_pads"] + P["right_pads"]) / 2
for fovy in (42.0, 58.0, 36.0, 50.0):
    f = 240 / np.tan(np.radians(fovy) / 2)
    for h in (0.25, 0.35, 0.5):
        for yaw0 in np.radians([-60, -30, 0, 30, 60]):
            x0 = np.array([mid[0], mid[1], mid[2] + h, 0, 0, yaw0]); r = least_squares(residuals, x0, args=(f,), max_nfev=3000)
            if best is None or r.cost < best[0]: best = (r.cost, r.x, fovy)
cost, x, fovy = best; f = 240 / np.tan(np.radians(fovy) / 2)
print(f"fovy {fovy}: pixel rms {np.sqrt(np.mean((residuals(x, f)[:8]*10)**2)):.1f} px; cam xyz {np.round(x[:3],4)} rpy {np.degrees(x[3:]).round(1)}; height above pads {(x[2]-mid[2])*1000:.0f} mm")
for k in PICK: print(f"  {k:12s} pick {PICK[k]} -> {project(x, P[k], f).round(0)}")
q = np.empty(4); mujoco.mju_mat2Quat(q, cam_R(x).reshape(-1))
json.dump({"fovy": fovy, "pos": x[:3].tolist(), "quat": q.tolist(), "rpy_deg": np.degrees(x[3:]).tolist(), "picks": PICK}, open("out/register_scene.json", "w"), indent=1)
S.SCENE_CAM_POS = tuple(x[:3]); S.SCENE_CAM_QUAT = tuple(q); S.SCENE_CAM_FOVY = fovy
spec, m = compile_model(part_mode="calibrated", part_offsets=load_part_offsets()); d = mujoco.MjData(m); set_qpos(d, joint_ids(m), st); mujoco.mj_forward(m, d)
r = mujoco.Renderer(m, 480, 640); r.update_scene(d, camera="scene_cam"); im = cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR)
cap = cv2.VideoCapture(f"{EP}/scene.mp4"); n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)); cap.set(cv2.CAP_PROP_POS_FRAMES, max(n-2, 0)); ok, real = cap.read(); cap.release()
cv2.imwrite("out/camgrid/scene_registered.png", np.hstack([im, real, cv2.addWeighted(im, 0.5, real, 0.5, 0)])); print("wrote out/camgrid/scene_registered.png")
