"""Live bench worker: runs episodes in a separate process, renders frames and writes state for the web app.

Reads commands from out/live/command.json (written by the server), writes out/live/frame.jpg (latest render),
out/live/cams.jpg (the three cell cameras side by side) and out/live/state.json (torques, depth, outcome, log).
"""
from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

import cv2
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from cell_twin.bench import Bench, PickError  # noqa: E402
from cell_twin import controllers as C  # noqa: E402

LIVE = ROOT / "out" / "live"
EPS = ROOT / "inputs" / "nuc" / "episodes" / "objects"
START_EP = "09710851-e26c-43ef-b7ed-0a684c28e4a4"


def atomic_write(path: Path, data: bytes):
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_bytes(data)
    os.replace(tmp, path)


def read_command():
    p = LIVE / "command.json"
    if not p.exists():
        return None
    try:
        cmd = json.loads(p.read_text())
    except Exception:
        return None
    p.unlink(missing_ok=True)
    return cmd


class Live:
    def __init__(self):
        self.seated = np.load(EPS / START_EP / "frames.npz")["observation.state"][-1]
        self.cfg = {"tilt_deg": 4.0, "slip_mm": 1.0, "female_tilt_deg": 0.0, "controller": "align", "slow": 2.0,
                    "cam": {"az": 120, "el": -15, "dist": 0.09}, "running": False, "loop": True}
        self.state = {"tick": 0, "torques": [], "depth": [], "outcome": "", "log": [], "episode": 0, "running": False}
        self.bench = None
        self.r = None
        self.rc = None

    def log(self, msg):
        self.state["log"] = (self.state["log"] + [f"{time.strftime('%H:%M:%S')} {msg}"])[-40:]

    def new_episode(self, rng):
        c = self.cfg
        herr = PickError(skew_deg=c["tilt_deg"] * rng.choice([-1.0, 1.0]), lateral_mm=(0.0, c["slip_mm"] * rng.choice([-1.0, 1.0])))
        ferr = PickError(skew_deg=c["female_tilt_deg"] * rng.choice([-1.0, 1.0])) if c["female_tilt_deg"] else None
        self.bench = Bench(self.seated, herr, housing_err=ferr, rng=rng)
        self.r = mujoco.Renderer(self.bench.m, 480, 640)
        self.rc = mujoco.Renderer(self.bench.m, 240, 320)
        self.state.update({"tick": 0, "torques": [], "depth": [], "outcome": "", "episode": self.state["episode"] + 1,
                           "pick": {"header_tilt": herr.skew_deg, "header_slip": herr.lateral_mm[1],
                                    "female_tilt": ferr.skew_deg if ferr else 0.0}})
        self.log(f"episode {self.state['episode']}: header tilt {herr.skew_deg:+.1f} deg, slip {herr.lateral_mm[1]:+.1f} mm, "
                 f"female tilt {ferr.skew_deg if ferr else 0:+.1f} deg, controller {c['controller']}")

    def render(self):
        b, cam = self.bench, self.cfg["cam"]
        mc = mujoco.MjvCamera(); mc.type = mujoco.mjtCamera.mjCAMERA_FREE
        mc.lookat[:] = b.d.site("socket_mouth").xpos; mc.distance = cam["dist"]; mc.azimuth = cam["az"]; mc.elevation = cam["el"]
        self.r.update_scene(b.d, camera=mc)
        frame = cv2.cvtColor(self.r.render(), cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80]); atomic_write(LIVE / "frame.jpg", buf.tobytes())
        tiles = []
        for name in ("scene_cam", "right_wrist_cam", "left_wrist_cam"):
            self.rc.update_scene(b.d, camera=name); im = cv2.cvtColor(self.rc.render(), cv2.COLOR_RGB2BGR)
            cv2.putText(im, name.replace("_cam", ""), (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2); tiles.append(im)
        ok, buf = cv2.imencode(".jpg", np.hstack(tiles), [cv2.IMWRITE_JPEG_QUALITY, 75]); atomic_write(LIVE / "cams.jpg", buf.tobytes())

    def write_state(self):
        s = dict(self.state); s["running"] = self.cfg["running"]; s["cfg"] = self.cfg
        atomic_write(LIVE / "state.json", json.dumps(s).encode())

    def run(self):
        rng = np.random.default_rng(int(time.time()) % 10000)
        self.new_episode(rng); self.render(); self.write_state()
        while True:
            cmd = read_command()
            if cmd:
                if "cfg" in cmd:
                    self.cfg.update(cmd["cfg"]); self.log("settings updated")
                if cmd.get("action") == "start":
                    self.cfg["running"] = True
                if cmd.get("action") == "stop":
                    self.cfg["running"] = False
                if cmd.get("action") == "reset":
                    self.cfg["running"] = False; self.new_episode(rng); self.render()
                self.write_state()
            if not self.cfg["running"]:
                self.render(); self.write_state(); time.sleep(0.25); continue
            # one episode, rendered every tick
            b = self.bench; base = np.mean([b.torques() for _ in range(5)], axis=0)
            fn = C.align_by_contact if self.cfg["controller"] == "align" else C.push_only
            orig_step = b.step_tool
            def step_tool(dpos, drot, **kw):
                orig_step(dpos, drot, **kw)
                tau = b._torque_hist[-1] if b._torque_hist else base
                self.state["tick"] += 1
                self.state["torques"].append([float(x) for x in (tau - base)]); self.state["depth"].append(float(b.seat_depth() * 1000))
                self.state["torques"] = self.state["torques"][-400:]; self.state["depth"] = self.state["depth"][-400:]
                if self.state["tick"] % 2 == 0:
                    self.render(); self.write_state()
                time.sleep(self.cfg["slow"] / 30.0)
                c = read_command()
                if c:
                    if c.get("action") == "stop":
                        self.cfg["running"] = False
                    if "cfg" in c and "cam" in c["cfg"]:
                        self.cfg["cam"] = c["cfg"]["cam"]
                    if not self.cfg["running"]:
                        raise StopIteration
            b.step_tool = step_tool
            try:
                ok = fn(b)
                self.state["outcome"] = b.outcome(); self.log(f"result: {self.state['outcome']} (depth {b.seat_depth()*1000:.2f} mm)")
            except StopIteration:
                self.log("stopped")
            finally:
                b.step_tool = orig_step
            self.render(); self.write_state()
            if self.cfg["running"] and self.cfg["loop"]:
                time.sleep(1.5); self.new_episode(rng)
            else:
                self.cfg["running"] = False


if __name__ == "__main__":
    LIVE.mkdir(parents=True, exist_ok=True)
    Live().run()
