"""Per-camera verification at one frame: twin | real | twin silhouette edges over real (red = right arm, yellow = left arm,
cyan = parts). usage: python scripts/cam_verify.py <camera: scene|right_wrist|left_wrist> [episode] [frame|last]"""
import sys, numpy as np, mujoco, cv2
sys.path.insert(0, "."); from cell_twin.scene import compile_model
from cell_twin.bench import GRIP_SLIDE
from cell_twin.layout import j6_offset
from scripts.replay import joint_ids
cam = sys.argv[1]; ep = sys.argv[2] if len(sys.argv) > 2 else "09710851-e26c-43ef-b7ed-0a684c28e4a4"; fr = sys.argv[3] if len(sys.argv) > 3 else "0"
EP = f"inputs/nuc/episodes/objects/{ep}"; S = np.load(f"{EP}/frames.npz")["observation.state"]; idx = len(S) - 1 if fr == "last" else int(fr); st = S[idx]
spec, m = compile_model(part_mode="geometric"); d = mujoco.MjData(m); jid = joint_ids(m)
for arm, off in (("left", 0), ("right", 7)):
    for i in range(6): d.qpos[jid[(arm, i)]] = st[off + i] + (j6_offset(arm) if i == 5 else 0.0)
    d.qpos[jid[(arm, "fl")]] = GRIP_SLIDE; d.qpos[jid[(arm, "fr")]] = GRIP_SLIDE
mujoco.mj_forward(m, d)
camname = {"scene": "scene_cam", "right_wrist": "right_wrist_cam", "left_wrist": "left_wrist_cam"}[cam]
r = mujoco.Renderer(m, 480, 640); r.update_scene(d, camera=camname); twin = cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR)
rs = mujoco.Renderer(m, 480, 640); rs.enable_segmentation_rendering(); rs.update_scene(d, camera=camname); seg = rs.render()[:, :, 0].astype(int)
cap = cv2.VideoCapture(f"{EP}/{cam}.mp4"); n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)); cap.set(cv2.CAP_PROP_POS_FRAMES, min(idx, n - 1)); ok, real = cap.read(); cap.release()
groups = {"right": (0, 0, 255), "left": (0, 255, 255), "part": (255, 255, 0)}
masks = {k: np.zeros(seg.shape, bool) for k in groups}
for g in range(m.ngeom):
    bn = m.body(m.geom_bodyid[g]).name
    key = "right" if bn.startswith("right_") else "left" if bn.startswith("left_") else "part" if ("plug" in bn or "housing" in bn or "socket" in bn or bn == "world") else None
    if key is None: continue
    if key == "part" and bn == "world": continue
    masks[key] |= (seg == g)
# parts are attached to the tool bodies in geometric mode: catch them by geom name instead
for g in range(m.ngeom):
    nm = m.geom(g).name
    if nm and (nm.startswith("pin") or nm.startswith("housing") or nm.startswith("plug") or nm.startswith("socket")):
        masks["part"] |= (seg == g); masks["right"] &= ~(seg == g); masks["left"] &= ~(seg == g)
ov = real.copy()
for k, col in groups.items():
    e = cv2.Canny(masks[k].astype(np.uint8) * 255, 50, 150); e = cv2.dilate(e, np.ones((2, 2), np.uint8)); ov[e > 0] = col
for im, t in ((twin, "twin"), (real, "real"), (ov, "twin edges on real")):
    cv2.putText(im, f"{cam} frame {idx}: {t}", (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
out = np.vstack([np.hstack([twin, real]), np.hstack([ov, cv2.addWeighted(twin, 0.5, real, 0.5, 0)])])
cv2.imwrite(f"out/camgrid/verify_{cam}_{fr}.png", out); print(f"wrote out/camgrid/verify_{cam}_{fr}.png")
