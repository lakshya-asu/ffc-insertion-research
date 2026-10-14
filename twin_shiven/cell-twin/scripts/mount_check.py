"""Render the right gripper from three free viewpoints at the seated pose so the camera mount can be checked by eye:
from the side (pinch axis toward the viewer), from above the flat base, and from behind along the fingers.
writes out/mount_check.png"""
import sys, numpy as np, mujoco, cv2
sys.path.insert(0, "."); from cell_twin.scene import compile_model
from cell_twin.bench import GRIP_SLIDE
from cell_twin.layout import j6_offset
from scripts.replay import joint_ids
EP = "inputs/nuc/episodes/objects/09710851-e26c-43ef-b7ed-0a684c28e4a4"; st = np.load(f"{EP}/frames.npz")["observation.state"][-1]
spec, m = compile_model(part_mode="geometric"); d = mujoco.MjData(m); jid = joint_ids(m)
for arm, off in (("left", 0), ("right", 7)):
    for i in range(6): d.qpos[jid[(arm, i)]] = st[off + i] + (j6_offset(arm) if i == 5 else 0.0)
    d.qpos[jid[(arm, "fl")]] = GRIP_SLIDE; d.qpos[jid[(arm, "fr")]] = GRIP_SLIDE
mujoco.mj_forward(m, d)
r = mujoco.Renderer(m, 480, 640); tiles = []
L6 = d.xmat[m.body("right_link_6").id].reshape(3, 3); p6 = d.xpos[m.body("right_link_6").id]; centre = p6 + L6 @ np.array([0, 0.03, 0.09])
cam_w = d.cam_xpos[m.camera("right_wrist_cam").id]
for label, dir_local in (("side: pinch axis toward you", np.array([1.0, 0, 0])), ("above the flat base", np.array([0, 1.0, 0])), ("behind, along the fingers", np.array([0, 0, -1.0]))):
    dw = L6 @ dir_local; eye = centre + 0.35 * dw
    mc = mujoco.MjvCamera(); mc.type = mujoco.mjtCamera.mjCAMERA_FREE; mc.lookat[:] = centre; mc.distance = 0.38
    v = -dw; mc.azimuth = np.degrees(np.arctan2(v[1], v[0])); mc.elevation = np.degrees(np.arcsin(np.clip(-v[2], -1, 1)))
    r.update_scene(d, camera=mc); im = cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR)
    # mark the registered camera position
    R = np.array(r.scene.camera[0].forward); up = np.array(r.scene.camera[0].up); pos = np.array(r.scene.camera[0].pos)
    cv2.putText(im, label, (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2); tiles.append(im)
cv2.imwrite("out/mount_check.png", np.hstack(tiles)); print("wrote out/mount_check.png")
