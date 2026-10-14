"""Overlay the twin's projected feature points (current registration) on the real frame so picks can be read off.
usage: python scripts/pick_sheet.py <episode> <frame index|last>   -> out/camgrid/pick_<cam>_<frame>.png (2x with grid)"""
import sys, numpy as np, mujoco, cv2
sys.path.insert(0, "."); from cell_twin.scene import compile_model
from cell_twin.bench import GRIP_SLIDE
from cell_twin.layout import j6_offset
from scripts.replay import joint_ids
ep, fr = sys.argv[1], sys.argv[2]; EP = f"inputs/nuc/episodes/objects/{ep}"
S = np.load(f"{EP}/frames.npz")["observation.state"]; idx = len(S) - 1 if fr == "last" else int(fr); st = S[idx]
spec, m = compile_model(part_mode="geometric"); d = mujoco.MjData(m); jid = joint_ids(m)
for arm, off in (("left", 0), ("right", 7)):
    for i in range(6): d.qpos[jid[(arm, i)]] = st[off + i] + (j6_offset(arm) if i == 5 else 0.0)
    d.qpos[jid[(arm, "fl")]] = GRIP_SLIDE; d.qpos[jid[(arm, "fr")]] = GRIP_SLIDE
mujoco.mj_forward(m, d)
FEATS = {}
for arm in ("left", "right"):
    z6 = d.xmat[m.body(f"{arm}_link_6").id].reshape(3, 3)[:, 2]
    for s in ("left", "right"):
        pc = d.geom_xpos[m.geom(f"{arm}_{s}_pad").id]
        FEATS[f"{arm}_{s[0]}tip"] = pc + 0.0072 * z6          # pad tip end (pad is 14.4 mm long)
    FEATS[f"{arm}_plate"] = d.geom_xpos[m.geom(f"{arm}_gripper_housing").id]
    FEATS[f"{arm}_cam"] = d.geom_xpos[m.geom(f"{arm}_d405").id]
def frame(path, i):
    cap = cv2.VideoCapture(path); n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)); cap.set(cv2.CAP_PROP_POS_FRAMES, min(i, n - 1)); ok, f = cap.read(); cap.release(); return f
for cam, vid in (("scene_cam", "scene"), ("right_wrist_cam", "right_wrist"), ("left_wrist_cam", "left_wrist")):
    cid = m.camera(cam).id; c = d.cam_xpos[cid]; R = d.cam_xmat[cid].reshape(3, 3); f = 240 / np.tan(np.radians(m.cam_fovy[cid]) / 2)
    im = cv2.resize(frame(f"{EP}/{vid}.mp4", idx), None, fx=2, fy=2)
    for x in range(0, 1280, 40): cv2.line(im, (x, 0), (x, 960), (0, 255, 255) if x % 200 == 0 else (90, 90, 90), 1)
    for y in range(0, 960, 40): cv2.line(im, (0, y), (1280, y), (0, 255, 255) if y % 200 == 0 else (90, 90, 90), 1)
    for x in range(0, 1280, 200):
        for y in range(0, 960, 200): cv2.putText(im, f"{x//2},{y//2}", (x + 2, y + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
    for name, p in FEATS.items():
        v = R.T @ (p - c)
        if v[2] >= -0.01: continue
        u, w = 320 + f * v[0] / (-v[2]), 240 - f * v[1] / (-v[2])
        if 0 <= u < 640 and 0 <= w < 480:
            cv2.circle(im, (int(2 * u), int(2 * w)), 6, (0, 0, 255), 2); cv2.putText(im, name, (int(2 * u) + 8, int(2 * w) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 80, 255), 2)
    cv2.imwrite(f"out/camgrid/pick_{vid}_{fr}.png", im)
print("wrote pick sheets for", ep[:8], fr, "state", np.round(st[:6], 3), np.round(st[7:13], 3))
