"""Constrained wrist camera placement: start from the stock Menagerie mount orientation (which frames the
scene like the real D405: opposite gripper top-centre, own part bottom-centre) and search only a tilt about
the pinch axis and a position offset (along the fingers, off the gripper back). Scored on the own fingers'
overlap with the dark bottom-centre of the real frames plus the opposite gripper landing on dark pixels."""
import sys, json, numpy as np, mujoco, cv2
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent; sys.path.insert(0, str(ROOT))
from cell_twin.scene import compile_model, load_part_offsets
from scripts.replay import joint_ids, set_qpos
from scripts.fit_camera import read_frame
from scripts.fit_wrist_camera import dark_mask
arm = sys.argv[1] if len(sys.argv) > 1 else "right"; other = "left" if arm == "right" else "right"
EPS = ROOT / "inputs/nuc/episodes/objects"; W, H = 320, 240
spec, m = compile_model(part_mode="calibrated", part_offsets=load_part_offsets()); d = mujoco.MjData(m); jid = joint_ids(m)
cam = m.camera(f"{arm}_wrist_cam").id; r = mujoco.Renderer(m, H, W); r.enable_segmentation_rendering()
own = np.zeros(m.ngeom + 1, bool); oth = np.zeros(m.ngeom + 1, bool)
for i in range(m.ngeom):
    bn = m.body(m.geom_bodyid[i]).name
    if bn.startswith(arm + "_") and ("finger" in bn or "pad" in bn): own[i] = True
    if bn.startswith(other + "_"): oth[i] = True
samples = []
for ep in sorted(p.name for p in EPS.iterdir())[:6]:
    z = np.load(EPS / ep / "frames.npz"); n = len(z["observation.state"])
    for k in np.linspace(0, n - 1, 3).astype(int):
        fr = read_frame(EPS / ep / f"{arm}_wrist.mp4", int(k))
        if fr is not None: samples.append((z["observation.state"][k], dark_mask(fr), fr))
pos0 = m.cam_pos[cam].copy(); q0 = m.cam_quat[cam].copy(); R0 = np.empty(9); mujoco.mju_quat2Mat(R0, q0); R0 = R0.reshape(3, 3)
win = np.zeros((H, W), bool); win[int(H * 0.5):, int(W * 0.25):int(W * 0.75)] = True
band = np.zeros((H, W), bool); band[:int(H * 0.6), int(W * 0.25):int(W * 0.75)] = True
def apply(tilt, dz, dy):
    # tilt about the camera's own x axis (the pinch axis in the mount frame), position along the camera's
    # viewing axis (dz) and its up axis (dy), in the camera body frame
    c, s_ = np.cos(tilt), np.sin(tilt); Rx = np.array([[1, 0, 0], [0, c, -s_], [0, s_, c]])
    R = R0 @ Rx; q = np.empty(4); mujoco.mju_mat2Quat(q, R.reshape(-1)); m.cam_quat[cam] = q
    m.cam_pos[cam] = pos0 + R0 @ np.array([0.0, dy, dz])
def score():
    tot = 0.0
    for state, dark, _ in samples:
        set_qpos(d, jid, state); mujoco.mj_forward(m, d); r.update_scene(d, camera=f"{arm}_wrist_cam")
        seg = r.render()[:, :, 0].astype(int); idx = np.clip(seg, -1, m.ngeom); valid = seg >= 0
        mo, mw = own[idx] & valid, oth[idx] & valid
        real_own = dark & win; iou = (mo & real_own).sum() / max(1, (mo | real_own).sum())
        mb = mw & band; prec = dark[mb].mean() if mb.sum() > 100 else 0.0
        tot += (1 - iou) + 0.5 * (1 - prec)
    return tot / len(samples)
best = (1e9, None)
for tilt in np.radians(np.arange(-60, 1, 5)):
    for dz in np.arange(-0.16, 0.021, 0.02):
        for dy in np.arange(-0.02, 0.101, 0.02):
            apply(tilt, dz, dy); sc = score()
            if sc < best[0]: best = (sc, (tilt, dz, dy))
sc, (tilt, dz, dy) = best; apply(tilt, dz, dy)
print(f"best score {sc:.3f}: tilt {np.degrees(tilt):+.0f} deg, along-view {dz*1000:+.0f} mm, up {dy*1000:+.0f} mm; cam pos {m.cam_pos[cam].round(4)}")
json.dump({"camera": f"{arm}_wrist_cam", "pos": m.cam_pos[cam].tolist(), "quat": m.cam_quat[cam].tolist(), "fovy": float(m.cam_fovy[cam]), "score": sc,
           "tilt_deg": float(np.degrees(tilt)), "dz": float(dz), "dy": float(dy)}, open(ROOT / "out" / f"fit_{arm}_wrist_constrained.json", "w"), indent=1)
tiles = []
for state, dark, fr in samples[:6]:
    set_qpos(d, jid, state); mujoco.mj_forward(m, d); r.update_scene(d, camera=f"{arm}_wrist_cam")
    seg = r.render()[:, :, 0].astype(int); idx = np.clip(seg, -1, m.ngeom); valid = seg >= 0
    mo, mw = own[idx] & valid, oth[idx] & valid
    ov = fr.copy(); ov[cv2.Canny(mw.astype(np.uint8) * 255, 50, 150) > 0] = (0, 0, 255); ov[cv2.Canny(mo.astype(np.uint8) * 255, 50, 150) > 0] = (0, 255, 255); tiles.append(ov)
cv2.imwrite(str(ROOT / "out" / f"fit_{arm}_wrist_constrained.png"), np.vstack([np.hstack(tiles[:3]), np.hstack(tiles[3:6])])); print("wrote overlay")
