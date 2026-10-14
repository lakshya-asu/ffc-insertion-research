"""Register the wrist D405 mount from the real last frame.

Mount (photos 2026-10-07): a printed plate bolted to the gripper's flat base, camera at its front edge on the +y side of
link_6, looking down the fingers. In the real frame the own fingertips are the two dark wedges at the bottom edge, the
far arm's fingers converge in the middle, its D405 sits at the top. Search: position around I2RT's CAD bracket point,
pitch toward the finger axis, roll about the optical axis. Picks are 640x480 pixels in the right wrist frame.
writes out/register_wrist.json and out/camgrid/wrist_registered.png"""
import sys, json, numpy as np, mujoco, cv2
sys.path.insert(0, "."); import cell_twin.scene as S
from cell_twin.scene import compile_model, load_part_offsets
from scripts.replay import joint_ids, set_qpos
EP = "inputs/nuc/episodes/objects/09710851-e26c-43ef-b7ed-0a684c28e4a4"; st = np.load(f"{EP}/frames.npz")["observation.state"][-1]
PICK = {"own_pads": (320.0, 415.0), "far_pads": (320.0, 345.0), "far_cam": (290.0, 70.0), "far_plate": (335.0, 225.0)}
W8 = {"own_pads": 1.0, "far_pads": 0.6, "far_cam": 0.3, "far_plate": 0.3}
spec, m = compile_model(part_mode="calibrated", part_offsets=load_part_offsets()); d = mujoco.MjData(m); set_qpos(d, joint_ids(m), st); mujoco.mj_forward(m, d)
L6 = np.eye(4); L6[:3, :3] = d.xmat[m.body("right_link_6").id].reshape(3, 3); L6[:3, 3] = d.xpos[m.body("right_link_6").id]
P = {"own_pads": (d.geom_xpos[m.geom("right_left_pad").id] + d.geom_xpos[m.geom("right_right_pad").id]) / 2,
     "far_pads": (d.geom_xpos[m.geom("left_left_pad").id] + d.geom_xpos[m.geom("left_right_pad").id]) / 2,
     "far_cam": d.geom_xpos[m.geom("left_d405").id], "far_plate": d.geom_xpos[m.geom("left_gripper_housing").id]}
def T_from(y, z, pitch_deg, roll_deg):
    # camera looks along link +z (fingers), image-up along link +y (away from the fingers), then pitched toward the
    # finger axis (about x) and rolled about the optical axis
    R0 = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], float)        # cam x = link x, cam y = link -y ... fixed below
    R0 = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]], float)        # cam -z = link +z, cam +y = link +y
    p, r = np.radians(pitch_deg), np.radians(roll_deg)
    Rp = np.array([[1, 0, 0], [0, np.cos(p), -np.sin(p)], [0, np.sin(p), np.cos(p)]])
    Rr = np.array([[np.cos(r), -np.sin(r), 0], [np.sin(r), np.cos(r), 0], [0, 0, 1]])
    T = np.eye(4); T[:3, :3] = R0 @ Rp @ Rr; T[:3, 3] = [0.0, y, z]; return T
def proj_all(T, f):
    Tw = L6 @ T; c = Tw[:3, 3]; R = Tw[:3, :3]; out = {}
    for k, p in P.items():
        v = R.T @ (p - c)
        out[k] = (320 + f * v[0] / (-v[2]), 240 - f * v[1] / (-v[2])) if v[2] < -0.005 else (1e4, 1e4)
    return out
best = None
for fovy in (58.0, 50.0, 42.0):
  f = 240 / np.tan(np.radians(fovy) / 2)
  for y in np.arange(0.05, 0.10, 0.005):
    for z in np.arange(0.04, 0.11, 0.005):
        for pitch in np.arange(-60, 62, 2.0):
            for roll in (0.0, 180.0):
                mm = proj_all(T_from(y, z, pitch, roll), f); err = sum(W8[k] * ((mm[k][0] - PICK[k][0]) ** 2 + (mm[k][1] - PICK[k][1]) ** 2) for k in PICK)
                if best is None or err < best[0]: best = (err, y, z, pitch, roll, mm, fovy)
err, y, z, pitch, roll, mm, fovy = best; S.WRIST_CAM_FOVY = fovy
print(f"best weighted rms {np.sqrt(err/sum(W8.values())):.1f} px: fovy={fovy} y={y:.3f} z={z:.3f} pitch={pitch:.0f} roll={roll:.0f}")
for k in PICK: print(f"  {k:10s} pick {PICK[k]} -> twin ({mm[k][0]:.0f},{mm[k][1]:.0f})")
T = T_from(y, z, pitch, roll)
json.dump({"fovy": fovy, "y": y, "z": z, "pitch_deg": pitch, "roll_deg": roll, "T_cam_in_link6": T.tolist(), "picks": PICK,
           "twin_px": {k: [float(a) for a in v] for k, v in mm.items()}}, open("out/register_wrist.json", "w"), indent=1)
def last_frame(p):
    cap = cv2.VideoCapture(p); n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)); cap.set(cv2.CAP_PROP_POS_FRAMES, max(n-2, 0)); ok, fr = cap.read(); cap.release(); return fr
S.WRIST_CAM_USE_CAD = False; S.WRIST_CAM_T_IN_LINK = {"left": T, "right": T}
spec, m = compile_model(part_mode="calibrated", part_offsets=load_part_offsets()); d = mujoco.MjData(m); set_qpos(d, joint_ids(m), st); mujoco.mj_forward(m, d)
r = mujoco.Renderer(m, 480, 640); tiles = []
for arm in ("right", "left"):
    r.update_scene(d, camera=f"{arm}_wrist_cam"); im = cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR); real = last_frame(f"{EP}/{arm}_wrist.mp4")
    blend = cv2.addWeighted(im, 0.5, real, 0.5, 0); cv2.putText(im, f"{arm} registered", (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2); tiles.append(np.hstack([im, real, blend]))
cv2.imwrite("out/camgrid/wrist_registered.png", np.vstack(tiles)); print("wrote out/camgrid/wrist_registered.png")
