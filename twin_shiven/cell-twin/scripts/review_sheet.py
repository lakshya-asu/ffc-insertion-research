"""Render a visual review sheet of the twin: whole cell, each gripper, connector halves at start and seat."""
import sys, numpy as np, mujoco, cv2
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent; sys.path.insert(0, str(ROOT))
from cell_twin import scene as S
from scripts.replay import joint_ids, set_qpos, place_plug, seat_depth
from scripts.calibrate import episode_offsets
ep = "09710851-e26c-43ef-b7ed-0a684c28e4a4"
st = np.load(ROOT / "inputs/nuc/episodes/objects" / ep / "frames.npz")["observation.state"]
spec, m = S.compile_model(part_mode="calibrated", part_offsets=episode_offsets(st[-1]), free_plug=True)
d = mujoco.MjData(m); jid = joint_ids(m)
r = mujoco.Renderer(m, 480, 640)
def shot(lookat, dist, az, el, label):
    cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE; cam.lookat[:] = lookat; cam.distance = dist; cam.azimuth = az; cam.elevation = el
    r.update_scene(d, camera=cam); im = cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR)
    cv2.putText(im, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2); return im
tiles = []
for tag, state in (("start", st[0]), ("seated", st[-1])):
    set_qpos(d, jid, state); mujoco.mj_forward(m, d); place_plug(m, d)
    mouth = d.site("socket_mouth").xpos.copy()
    tiles.append(np.hstack([shot([0, 0, 0.25], 1.6, 135, -25, f"{tag}: whole cell"),
                            shot(d.body("left_link_6").xpos, 0.35, 120, -20, f"{tag}: left gripper (housing)"),
                            shot(d.body("right_link_6").xpos, 0.35, 60, -20, f"{tag}: right gripper (header)")]))
    tiles.append(np.hstack([shot(mouth, 0.05, 90, -15, f"{tag}: parts, side"),
                            shot(mouth, 0.05, 0, -15, f"{tag}: parts, along row"),
                            shot(mouth, 0.05, 45, -70, f"{tag}: parts, from above")]))
    print(f"{tag}: depth {seat_depth(m, d)*1000:.2f} mm, contacts {d.ncon}")
cv2.imwrite(str(ROOT / "out/review_sheet.png"), np.vstack(tiles)); print("wrote out/review_sheet.png")
