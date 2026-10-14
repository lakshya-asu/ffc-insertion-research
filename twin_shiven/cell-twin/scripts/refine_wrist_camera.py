"""Local refinement of a wrist camera from its current (close) pose: small bounds on position (±15 mm),
rotation (±10 deg), fovy fixed. Objective: own-finger IoU in the bottom window + opposite-arm precision in the
centre band, over many frames. Writes out/refine_<arm>_wrist.json and the overlay."""
import sys, json, os, numpy as np, mujoco, cv2
from pathlib import Path
from scipy.optimize import minimize
ROOT = Path(__file__).resolve().parent.parent; sys.path.insert(0, str(ROOT))
from cell_twin.scene import compile_model, load_part_offsets
from scripts.replay import joint_ids, set_qpos
from scripts.fit_camera import read_frame, rot_from_rpy
from scripts.fit_wrist_camera import dark_mask
arm = sys.argv[1]; other = "left" if arm == "right" else "right"
EPS = ROOT / "inputs/nuc/episodes/objects"; W, H = 320, 240
spec, m = compile_model(part_mode="calibrated", part_offsets=load_part_offsets()); d = mujoco.MjData(m); jid = joint_ids(m)
cam = m.camera(f"{arm}_wrist_cam").id; r = mujoco.Renderer(m, H, W); r.enable_segmentation_rendering()
own = np.zeros(m.ngeom + 1, bool); oth = np.zeros(m.ngeom + 1, bool)
for i in range(m.ngeom):
    bn = m.body(m.geom_bodyid[i]).name
    if bn.startswith(arm + "_") and ("finger" in bn or "pad" in bn): own[i] = True
    if bn.startswith(other + "_"): oth[i] = True
samples = []
for ep in sorted(os.listdir(EPS))[:8]:
    z = np.load(EPS / ep / "frames.npz"); n = len(z["observation.state"])
    for k in np.linspace(0, n - 1, 3).astype(int):
        fr = read_frame(EPS / ep / f"{arm}_wrist.mp4", int(k))
        if fr is not None: samples.append((z["observation.state"][k], dark_mask(fr), fr))
pos0 = m.cam_pos[cam].copy(); q0 = m.cam_quat[cam].copy(); R0 = np.empty(9); mujoco.mju_quat2Mat(R0, q0); R0 = R0.reshape(3, 3)
win = np.zeros((H, W), bool); win[int(H * 0.55):, int(W * 0.3):int(W * 0.7)] = True
band = np.zeros((H, W), bool); band[:int(H * 0.65), int(W * 0.2):int(W * 0.8)] = True
def apply(x):
    m.cam_pos[cam] = pos0 + np.clip(x[:3], -0.015, 0.015)
    R = R0 @ rot_from_rpy(np.clip(x[3:6], -np.radians(10), np.radians(10)))
    q = np.empty(4); mujoco.mju_mat2Quat(q, R.reshape(-1)); m.cam_quat[cam] = q
def score(x):
    apply(x); tot = 0.0
    for state, dark, _ in samples:
        set_qpos(d, jid, state); mujoco.mj_forward(m, d); r.update_scene(d, camera=f"{arm}_wrist_cam")
        seg = r.render()[:, :, 0].astype(int); idx = np.clip(seg, -1, m.ngeom); valid = seg >= 0
        mo, mw = own[idx] & valid, oth[idx] & valid
        real_own = dark & win; iou = (mo & real_own).sum() / max(1, (mo | real_own).sum())
        mb = mw & band; prec = dark[mb].mean() if mb.sum() > 100 else 0.0
        cov = (mw & dark & band).sum() / max(1, (dark & band).sum())
        tot += (1 - iou) + 0.7 * (1 - prec) + 0.5 * (1 - min(1.0, cov * 2))
    return tot / len(samples)
x0 = np.zeros(6); s0 = score(x0)
res = minimize(score, x0, method="Nelder-Mead", options={"maxiter": 250, "xatol": 1e-4, "fatol": 1e-4,
               "initial_simplex": np.vstack([x0] + [x0 + np.eye(6)[i] * st for i, st in enumerate([0.006, 0.006, 0.006, 0.05, 0.05, 0.05])])})
apply(res.x)
print(f"{arm}: score {s0:.3f} -> {res.fun:.3f}; dpos (mm) {np.round(np.clip(res.x[:3],-0.015,0.015)*1000,1)} drot (deg) {np.round(np.degrees(np.clip(res.x[3:6],-0.1745,0.1745)),1)}")
json.dump({"arm": arm, "pos": m.cam_pos[cam].tolist(), "quat": m.cam_quat[cam].tolist(), "score0": s0, "score": float(res.fun), "x": res.x.tolist()},
          open(ROOT / "out" / f"refine_{arm}_wrist.json", "w"), indent=1)
tiles = []
for state, dark, fr in samples[:6]:
    set_qpos(d, jid, state); mujoco.mj_forward(m, d); r.update_scene(d, camera=f"{arm}_wrist_cam")
    seg = r.render()[:, :, 0].astype(int); idx = np.clip(seg, -1, m.ngeom); valid = seg >= 0
    mo, mw = own[idx] & valid, oth[idx] & valid; ov = fr.copy()
    ov[cv2.Canny(mw.astype(np.uint8) * 255, 50, 150) > 0] = (0, 0, 255); ov[cv2.Canny(mo.astype(np.uint8) * 255, 50, 150) > 0] = (0, 255, 255); tiles.append(ov)
cv2.imwrite(str(ROOT / "out" / f"refine_{arm}_wrist.png"), np.vstack([np.hstack(tiles[:3]), np.hstack(tiles[3:6])])); print("wrote overlay")
