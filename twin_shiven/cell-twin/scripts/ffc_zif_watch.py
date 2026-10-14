"""Watch the FFC/ZIF twin run. Replays the trajectory recorded by ffc_zif_demo.py (out/ffc_zif/<arm>/traj.npz).
  .venv/bin/python scripts/ffc_zif_watch.py [meca500|lrmate200id]                 opens the MuJoCo viewer, real time, looping; drag to orbit
  .venv/bin/python scripts/ffc_zif_watch.py meca500 --video out.mp4 [--speed 1.0]  renders a close view + the wrist macro camera side by side
"""
import math, sys, time, subprocess
from pathlib import Path
import mujoco, numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from cell_twin import ffc_zif as F

import os
ARM = next((a for a in sys.argv[1:] if not a.startswith("--")), os.environ.get("FFC_ARM", "meca500"))
CASE = os.environ.get("FFC_CASE", "laptop"); TOOL = os.environ.get("FFC_TOOL", F.DEFAULT_TOOL[ARM])
OUT = Path(__file__).resolve().parent.parent / "out" / "ffc_zif" / f"{ARM}-{TOOL}-{CASE}"
VIDEO = sys.argv[sys.argv.index("--video") + 1] if "--video" in sys.argv else None
SPEED = float(sys.argv[sys.argv.index("--speed") + 1]) if "--speed" in sys.argv else 1.0

spec = F.build(ARM, CASE, TOOL)
o = F.connector_origin(ARM)
w = spec.worldbody
def cam(name, pos, target, fovy, up=(0, 0, 1)):
    w.add_camera(name=name, pos=list(pos), quat=F._look_quat(np.array(pos), np.array(target), up=up), fovy=fovy)
cam("watch_close", [o[0] + 0.075, o[1] - 0.055, o[2] + 0.055], [o[0], o[1] + 0.004, o[2] + 0.006], 20)   # three-quarter view from the connector's east end
cam("watch_side", [o[0] + 0.110, o[1] - 0.002, o[2] + 0.016], [o[0], o[1] - 0.001, o[2] + 0.004], 10)    # side view along the latch hinge, as a cross-section
model = spec.compile(); data = mujoco.MjData(model)
tr = np.load(OUT / "traj.npz"); Q = tr["qpos"]; T = tr["t"]; PA = tr["phase_at"]; PL = tr["phase_lbl"]
def label_at(i):
    k = np.searchsorted(PA, i, side="right") - 1
    return str(PL[k]) if k >= 0 else ""
print(f"{len(Q)} frames, {T[-1]:.1f} s of simulated time, phases: {list(PL)}")

if VIDEO:
    from PIL import Image, ImageDraw
    fps = 30; dt_sim = np.median(np.diff(T)) if len(T) > 1 else 0.008
    stride = max(1, int(round((1.0 / fps) * SPEED / dt_sim)))
    W, H = 960, 540
    ren = mujoco.Renderer(model, H, W)
    tmp = OUT / "video_frames"; tmp.mkdir(exist_ok=True)
    for f in tmp.glob("*.jpg"):
        f.unlink()
    n = 0
    for i in range(0, len(Q), stride):
        data.qpos[:] = Q[i]; mujoco.mj_forward(model, data)
        tiles = []
        for c in ("watch_side", "watch_close", "overview"):
            ren.update_scene(data, camera=c); tiles.append(Image.fromarray(ren.render()))
        im = Image.new("RGB", (W * 3, H + 40), (20, 20, 22))
        for k, t in enumerate(tiles):
            im.paste(t, (k * W, 40))
        d = ImageDraw.Draw(im)
        d.text((12, 12), f"{label_at(i)}    t = {T[i]:5.1f} s    latch {math.degrees(data.joint('zif_latch').qpos[0]):5.1f} deg", fill=(240, 236, 228))
        for k, name in enumerate(("side view along the latch hinge", "three-quarter view", "overview camera on the bench post")):
            d.text((k * W + 12, H + 40 - 18), name, fill=(200, 196, 188))
        im.save(tmp / f"{n:05d}.jpg", quality=88); n += 1
    print("frames", n)
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(fps), "-i", str(tmp / "%05d.jpg"), "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "26", "-movflags", "+faststart", VIDEO], check=True)
    print("wrote", VIDEO)
else:
    import mujoco.viewer
    with mujoco.viewer.launch_passive(model, data) as v:
        v.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        v.cam.lookat[:] = [o[0], o[1], o[2] + 0.005]; v.cam.distance = 0.09; v.cam.azimuth = 60; v.cam.elevation = -25
        i = 0
        while v.is_running():
            data.qpos[:] = Q[i]; mujoco.mj_forward(model, data); v.sync()
            dt = (T[i + 1] - T[i]) if i + 1 < len(T) else 0.008
            time.sleep(max(0.0, dt / SPEED)); i = (i + 1) % len(Q)
