"""Render short clips from the corrected twin for the guide page: an oblique view and a top view with the slot
roof drawn translucent, plus the per-tick depth and force log. Writes JSON (base64 WebP frames) to --out.

Clips: height error guided in by the funnel; sideways error jamming; turn error jamming; the expert from a hard
start with the researched sensor models.
"""
import argparse
import base64
import io
import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ffc_twin.env import TwinEnv, EstimatorConfig, Randomization  # noqa: E402
from ffc_twin.expert import Expert  # noqa: E402
from ffc_twin.mjcf import build_mjcf  # noqa: E402


def lit_xml(spec):
    xml = build_mjcf(spec)
    xml = xml.replace("<worldbody>", '''<visual><global offwidth="1280" offheight="720"/><headlight ambient="0.5 0.5 0.5" diffuse="0.55 0.55 0.55" specular="0.05 0.05 0.05"/><map znear="0.0003" zfar="2"/></visual>
  <asset><texture type="skybox" builtin="flat" rgb1="0.985 0.975 0.95" rgb2="0.985 0.975 0.95" width="32" height="32"/></asset>
  <worldbody>
    <light pos="0.01 -0.05 0.07" dir="-0.1 0.6 -0.8" diffuse="0.55 0.55 0.55" castshadow="false"/>''')
    return xml


class Viewer:
    def __init__(self, env: TwinEnv):
        self.env = env
        self.model = mujoco.MjModel.from_xml_string(lit_xml(env.spec))
        self.data = mujoco.MjData(self.model)
        self.r = mujoco.Renderer(self.model, height=360, width=640)
        self.opt = mujoco.MjvOption()
        self.obl = mujoco.MjvCamera(); self.obl.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.obl.lookat[:] = [0.0, 0.0, 0.0]; self.obl.distance = 0.026; self.obl.azimuth = 150; self.obl.elevation = -24
        self.top = mujoco.MjvCamera(); self.top.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.top.lookat[:] = [0.0, 0.0, 0.0]; self.top.distance = 0.027; self.top.azimuth = 90; self.top.elevation = -90

    def frame(self):
        # mirror the env's state into the lit model (same kinematic tree, visual differences only)
        self.data.qpos[:] = self.env.data.qpos; self.data.qvel[:] = self.env.data.qvel
        mujoco.mj_forward(self.model, self.data)
        self.r.update_scene(self.data, self.obl, self.opt); a = self.r.render().copy()
        self.r.update_scene(self.data, self.top, self.opt); b = self.r.render().copy()
        return np.concatenate([a, b], axis=1)


def clip(env, viewer, start, controller, max_ticks=150, every=2):
    obs, _ = env.reset(seed=3, level="L1", start=start)
    ex = Expert(env.spec) if controller == "expert" else None
    frames, log = [], []
    for k in range(max_ticks):
        a = ex.act(obs) if ex else np.array([1, 0, 0, 0, 0, 0.0])
        phase = ex.phase if ex else "push"
        obs, r, term, trunc, info = env.step(a)
        t = env.truth()
        log.append((t["depth_m"] * 1e3, float(np.linalg.norm(obs["fixture_wrench_n_nm"][:3])), phase))
        if k % every == 0:
            frames.append(viewer.frame())
        if term or trunc:
            break
    frames.append(viewer.frame())
    t = env.truth()
    return dict(frames=frames, log=log, seated=bool(t["seated"]), depth=t["depth_m"] * 1e3, peak=env.peak_force, retries=ex.retries if ex else 0, slip=t["grip_slip_m"] * 1e3)


def save(path, c, w=960):
    b64 = []
    for fr in c["frames"]:
        im = Image.fromarray(fr).resize((w, int(w * fr.shape[0] / fr.shape[1])), Image.LANCZOS)
        buf = io.BytesIO(); im.save(buf, format="WEBP", quality=70); b64.append(base64.b64encode(buf.getvalue()).decode())
    json.dump(dict(frames=b64, log=c["log"], seated=c["seated"], depth=c["depth"], peak=c["peak"], retries=c["retries"]), open(path, "w"))
    print(path.name, len(b64), "frames", f"seated {c['seated']} depth {c['depth']:.2f} peak {c['peak']:.2f} N retries {c['retries']} slip {c['slip']:.2f} mm")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--out", required=True); a = ap.parse_args()
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    env = TwinEnv(estimator=EstimatorConfig.good(), randomization=Randomization(enabled=False), log=False)
    v = Viewer(env)
    zero = dict(lateral=0.0, height=0.0, yaw=0.0, pitch=0.0, roll=0.0)
    save(out / "anim_success.json", clip(env, v, {**zero, "height": 0.3e-3}, "push", max_ticks=60))
    save(out / "anim_jam.json", clip(env, v, {**zero, "lateral": 0.3e-3, "height": 0.2e-3}, "push", max_ticks=50))
    save(out / "anim_yawjam.json", clip(env, v, {**zero, "yaw": math.radians(2.0)}, "push", max_ticks=50))
    env2 = TwinEnv(estimator=EstimatorConfig.good(), log=False)
    v2 = Viewer(env2)
    save(out / "anim_expert.json", clip(env2, v2, dict(lateral=1.0e-3, height=0.5e-3, yaw=math.radians(4), pitch=0.0, roll=0.0), "expert"))
