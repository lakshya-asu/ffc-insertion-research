"""Twin rendered over the real frame (45/55 blend) for the three cameras at frame 0 and the last frame of one demo.
usage: python scripts/blend_check.py [episode]   -> out/blend_both.png"""
import sys, numpy as np, mujoco, cv2
sys.path.insert(0, "."); from cell_twin.scene import compile_model
from cell_twin.bench import GRIP_SLIDE
from cell_twin.layout import j6_offset
from scripts.replay import joint_ids
ep = sys.argv[1] if len(sys.argv) > 1 else "09710851-e26c-43ef-b7ed-0a684c28e4a4"; EP = f"inputs/nuc/episodes/objects/{ep}"
S = np.load(f"{EP}/frames.npz")["observation.state"]
spec, m = compile_model(part_mode="geometric"); d = mujoco.MjData(m); jid = joint_ids(m); r = mujoco.Renderer(m, 480, 640)
def frame(path, i):
    cap = cv2.VideoCapture(path); n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)); cap.set(cv2.CAP_PROP_POS_FRAMES, min(i, n - 1)); ok, f = cap.read(); cap.release(); return f
rows = []
for name, idx in (("frame 0 (preinsertion2)", 0), ("last (seated)", len(S) - 1)):
    st = S[idx]
    for arm, off in (("left", 0), ("right", 7)):
        for i in range(6): d.qpos[jid[(arm, i)]] = st[off + i] + (j6_offset(arm) if i == 5 else 0.0)
        d.qpos[jid[(arm, "fl")]] = GRIP_SLIDE; d.qpos[jid[(arm, "fr")]] = GRIP_SLIDE
    mujoco.mj_forward(m, d); tiles = []
    for cam, vid in (("scene_cam", "scene"), ("right_wrist_cam", "right_wrist"), ("left_wrist_cam", "left_wrist")):
        r.update_scene(d, camera=cam); im = cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR); real = frame(f"{EP}/{vid}.mp4", idx)
        b = cv2.addWeighted(im, 0.45, real, 0.55, 0); cv2.putText(b, f"{vid}, {name}", (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2); tiles.append(b)
    rows.append(np.hstack(tiles))
cv2.imwrite("out/blend_both.png", np.vstack(rows)); print("wrote out/blend_both.png")
