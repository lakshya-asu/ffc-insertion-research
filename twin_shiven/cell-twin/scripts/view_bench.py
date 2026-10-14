"""Watch the insertion bench live: a pick error is sampled, then the align-by-contact controller runs.

usage: mjpython scripts/view_bench.py [--skew 4] [--lateral 1.5] [--controller align|push] [--slow 3]
The camera starts on the connector pair; drag to orbit, scroll to zoom, space pauses the viewer.
"""
import argparse, sys, time
from pathlib import Path
import mujoco, mujoco.viewer, numpy as np
ROOT = Path(__file__).resolve().parent.parent; sys.path.insert(0, str(ROOT))
from cell_twin.bench import Bench, PickError
from cell_twin import controllers as C
from scripts.calibrate import episode_offsets
ap = argparse.ArgumentParser(); ap.add_argument("--skew", type=float, default=4.0); ap.add_argument("--lateral", type=float, default=1.5)
ap.add_argument("--controller", default="align"); ap.add_argument("--slow", type=float, default=3.0); ap.add_argument("--loop", action="store_true")
a = ap.parse_args()
seated = np.load(ROOT / "inputs/nuc/episodes/objects/09710851-e26c-43ef-b7ed-0a684c28e4a4/frames.npz")["observation.state"][-1]
po = episode_offsets(seated)
rng = np.random.default_rng(int(time.time()) % 1000)
while True:
    ang = 0.0
    err = PickError(skew_deg=a.skew * rng.choice([-1.0, 1.0]), skew_dir_deg=0.0, lateral_mm=(0.0, a.lateral * rng.choice([-1.0, 1.0])))
    b = Bench(seated, err, part_offsets=po, rng=rng)
    print(f"pick error: tilt {a.skew} deg, lateral {a.lateral} mm at {np.degrees(ang):.0f} deg; controller {a.controller}")
    with mujoco.viewer.launch_passive(b.m, b.d) as v:
        v.cam.lookat[:] = b.d.site("socket_mouth").xpos; v.cam.distance = 0.09; v.cam.azimuth = 120; v.cam.elevation = -15
        v.opt.sitegroup[4] = True
        orig_step, orig_hold = b.step_tool, b.hold
        def step_tool(dpos, drot, **kw):
            orig_step(dpos, drot, **kw); v.sync(); time.sleep(a.slow / 30.0)
            if not v.is_running(): raise SystemExit
        def hold():
            orig_hold(); v.sync(); time.sleep(a.slow / 30.0)
        b.step_tool, b.hold = step_tool, hold
        v.sync(); time.sleep(1.0)
        ok = C.align_by_contact(b) if a.controller == "align" else C.push_only(b)
        print("RESULT:", "SEATED" if ok else "NOT SEATED", f"depth {b.seat_depth()*1000:.2f} mm", flush=True)
        t_end = time.time() + 4.0
        while v.is_running() and time.time() < t_end:
            b.hold()
    if not a.loop:
        break
