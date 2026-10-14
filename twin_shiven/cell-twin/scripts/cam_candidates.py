"""Render camera candidates beside the real last frame of an episode. usage: python scripts/cam_candidates.py"""
import sys, numpy as np, mujoco, cv2
sys.path.insert(0, ".")
import cell_twin.scene as S
from cell_twin.scene import compile_model, load_part_offsets
from scripts.replay import joint_ids, set_qpos
EP = "inputs/nuc/episodes/objects/09710851-e26c-43ef-b7ed-0a684c28e4a4"
st = np.load(f"{EP}/frames.npz")["observation.state"][-1]
def last_frame(p):
    cap = cv2.VideoCapture(p); n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)); cap.set(cv2.CAP_PROP_POS_FRAMES, max(n-2,0)); ok, f = cap.read(); cap.release(); return f
real = {c: last_frame(f"{EP}/{c}.mp4") for c in ("scene", "right_wrist", "left_wrist")}
def render(cam, label):
    spec, m = compile_model(part_mode="calibrated", part_offsets=load_part_offsets()); d = mujoco.MjData(m); set_qpos(d, joint_ids(m), st); mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, 480, 640); r.update_scene(d, camera=cam); im = cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR)
    cv2.putText(im, label, (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2); return im, m, d
mode = sys.argv[1]
if mode == "scene":
    # roll: make the left arm's tool z (finger direction) point to +u in the image, then vary the height
    spec, m = compile_model(part_mode="calibrated", part_offsets=load_part_offsets()); d = mujoco.MjData(m); set_qpos(d, joint_ids(m), st); mujoco.mj_forward(m, d)
    zl = d.xmat[m.body("left_link_6").id].reshape(3,3)[:, 2]; zr = d.xmat[m.body("right_link_6").id].reshape(3,3)[:, 2]
    ang = np.arctan2(zl[1], zl[0]); print("left tool z heading (deg)", np.degrees(ang), "right", np.degrees(np.arctan2(zr[1], zr[0])))
    # nadir camera: image u = cam x, image v (up) = cam y; cam z up. Rotating the camera about z by ang makes cam x = heading of zl
    q = np.array([np.cos(ang/2), 0, 0, np.sin(ang/2)]); R = np.array([[np.cos(ang), -np.sin(ang), 0], [np.sin(ang), np.cos(ang), 0], [0, 0, 1]])
    mid = (d.site("socket_mouth").xpos + d.site("plug_tip").xpos) / 2
    rows = []
    for h in (0.55, 0.70, 0.85):
        f = 240 / np.tan(np.radians(S.SCENE_CAM_FOVY) / 2); depth = h - mid[2]
        # place the camera so the mid point lands at real pixel (400, 262)
        du, dv = (400 - 320) * depth / f, -(262 - 240) * depth / f
        c = mid - R @ np.array([du, dv, 0]); c[2] = h
        S.SCENE_CAM_POS = tuple(c); S.SCENE_CAM_QUAT = tuple(q)
        im, _, _ = render("scene_cam", f"scene h={h:.2f} roll={np.degrees(ang):.0f}"); rows.append(np.hstack([im, real["scene"]]))
        print(h, "cam", np.round(c, 4), "quat", np.round(q, 4))
    cv2.imwrite("out/camgrid/scene.png", np.vstack(rows))
else:
    rows = []
    for (y, roll, tilt) in [(0.070, 180, -25), (0.070, 180, -35), (0.070, 180, -45), (-0.070, 0, -25), (-0.070, 0, -35), (-0.070, 0, -45), (0.070, 0, -15)]:
        S.WRIST_CAM_POS = (0.0, y, -0.020); S.WRIST_CAM_ROLL_DEG = roll; S.WRIST_CAM_TILT_DEG = tilt
        tiles = []
        for arm in ("right", "left"):
            im, _, _ = render(f"{arm}_wrist_cam", f"{arm} y={y*1000:+.0f} roll={roll} tilt={tilt}"); tiles += [im, real[f"{arm}_wrist"]]
        rows.append(np.hstack(tiles))
    cv2.imwrite("out/camgrid/wrist.png", cv2.resize(np.vstack(rows), None, fx=0.5, fy=0.5))
print("done")
