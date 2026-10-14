"""Joint fit of the scene camera pose and the right arm base (x, y, yaw) by edge matching.

The left arm is treated as the reference (its base defines the world). Frames are sampled
across episodes so the arms occupy different poses, which makes the base observable.
writes out/fit_camera_base.json and out/fit_camera_base.png
"""
import sys, json, numpy as np, mujoco, cv2
from pathlib import Path
from scipy.optimize import minimize
from scipy.ndimage import distance_transform_edt
ROOT = Path(__file__).resolve().parent.parent; sys.path.insert(0, str(ROOT))
from cell_twin import layout as LAY
from cell_twin.scene import compile_model, load_part_offsets
from scripts.replay import joint_ids, set_qpos
from scripts.fit_camera import read_frame, edges, rot_from_rpy
EPS = ROOT / "inputs/nuc/episodes/objects"
W, H = 320, 240
init = json.load(open(ROOT / "out/fit_scene_cam.json"))
samples = []
for ep in sorted(p.name for p in EPS.iterdir())[:6]:
    z = np.load(EPS / ep / "frames.npz"); n = len(z["observation.state"])
    for k in np.linspace(0, n - 1, 3).astype(int):
        fr = read_frame(EPS / ep / "scene.mp4", int(k))
        if fr is not None:
            samples.append((z["observation.state"][k], distance_transform_edt(~edges(fr)), fr))
print(len(samples), "frames")
def build(base_dx, base_dy, base_dyaw):
    xyz, yaw = LAY.ARM_BASE["right"]
    LAY.ARM_BASE["right"] = (np.array([0.6192 + base_dx, 0.0 + base_dy, 0.045]), 180.0 + base_dyaw)
    spec, m = compile_model(part_mode="calibrated", part_offsets=load_part_offsets())
    LAY.ARM_BASE["right"] = (np.array([0.6192, 0.0, 0.045]), 180.0)
    return m
cache = {}
def score(x):
    key = tuple(np.round(x[7:], 4))
    if key not in cache:
        m = build(*x[7:]); cache.clear(); cache[key] = (m, mujoco.MjData(m), joint_ids(m), mujoco.Renderer(m, H, W))
    m, d, jid, r = cache[key]
    cid = m.camera("scene_cam").id
    m.cam_pos[cid] = np.array(init["pos"]) + x[:3]
    R0 = np.empty(9); mujoco.mju_quat2Mat(R0, np.array(init["quat"])); R = rot_from_rpy(x[3:6]) @ R0.reshape(3, 3)
    q = np.empty(4); mujoco.mju_mat2Quat(q, R.reshape(-1)); m.cam_quat[cid] = q; m.cam_fovy[cid] = init["fovy"] * np.exp(x[6])
    tot = 0.0
    for state, dt, _ in samples:
        set_qpos(d, jid, state); mujoco.mj_forward(m, d); r.update_scene(d, camera="scene_cam")
        e = edges(cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR))
        tot += 50.0 if e.sum() < 50 else float(np.mean(np.minimum(dt[e], 20.0)))
    return tot / len(samples)
x0 = np.zeros(10); s0 = score(x0); print("start score", round(s0, 3))
steps = [0.03, 0.03, 0.03, 0.05, 0.05, 0.05, 0.08, 0.04, 0.04, 0.05]
best = minimize(score, x0, method="Nelder-Mead", options={"maxiter": 400, "xatol": 1e-4, "fatol": 1e-4,
                "initial_simplex": np.vstack([x0] + [x0 + np.eye(10)[i] * s for i, s in enumerate(steps)])})
print(f"score {s0:.3f} -> {best.fun:.3f}; right base shift dx {best.x[7]*1000:+.1f} mm dy {best.x[8]*1000:+.1f} mm dyaw {np.degrees(best.x[9]):+.2f} deg; camera dpos {np.round(best.x[:3],4)} fovy x{np.exp(best.x[6]):.3f}")
json.dump({"x": best.x.tolist(), "score": float(best.fun), "score0": s0, "camera_init": init}, open(ROOT / "out/fit_camera_base.json", "w"), indent=1)
m, d, jid, r = cache[tuple(np.round(best.x[7:], 4))]
tiles = []
for state, _, fr in samples[:4]:
    set_qpos(d, jid, state); mujoco.mj_forward(m, d); r.update_scene(d, camera="scene_cam")
    e = edges(cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR)); ov = fr.copy(); ov[e] = (0, 0, 255); tiles.append(ov)
cv2.imwrite(str(ROOT / "out/fit_camera_base.png"), np.hstack(tiles)); print("wrote out/fit_camera_base.png")
