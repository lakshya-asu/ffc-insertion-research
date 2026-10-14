"""Render the twin at a recorded pose beside the real camera frames.

usage: python scripts/render_check.py [episode_id]
writes out/render_check.png (scene + right wrist, twin above real)
"""
from __future__ import annotations

import sys
from pathlib import Path

import cv2
import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from cell_twin.layout import JOINT_NAMES, j6_offset  # noqa: E402
from cell_twin.bench import GRIP_SLIDE  # noqa: E402
from cell_twin.scene import compile_model, load_part_offsets  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
EPS = ROOT / "inputs" / "nuc" / "episodes" / "objects"


def set_state(m, d, state14):
    for arm, off in (("left", 0), ("right", 7)):
        for i, jn in enumerate(JOINT_NAMES):
            d.joint(f"{arm}_{jn}").qpos[0] = state14[off + i] + (j6_offset(arm) if i == 5 else 0.0)
        s = GRIP_SLIDE          # pads on the housing end faces, as in the cell (the recorded value maps through the cell's own calibration)
        d.joint(f"{arm}_left_finger").qpos[0] = s
        d.joint(f"{arm}_right_finger").qpos[0] = s
    mujoco.mj_forward(m, d)


def first_frame(mp4: Path):
    cap = cv2.VideoCapture(str(mp4))
    ok, f = cap.read()
    cap.release()
    return f if ok else np.zeros((480, 640, 3), np.uint8)


def main():
    ep = sys.argv[1] if len(sys.argv) > 1 else sorted(p.name for p in EPS.iterdir())[0]
    z = np.load(EPS / ep / "frames.npz")
    spec, m = compile_model(part_mode="geometric")
    d = mujoco.MjData(m)
    set_state(m, d, z["observation.state"][0])
    r = mujoco.Renderer(m, 480, 640)
    panels = []
    for cam, real in (("scene_cam", "scene.mp4"), ("right_wrist_cam", "right_wrist.mp4"), ("left_wrist_cam", "left_wrist.mp4")):
        r.update_scene(d, camera=cam)
        twin = cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR)
        panels.append(np.vstack([twin, first_frame(EPS / ep / real)]))
    out = np.hstack(panels)
    (ROOT / "out").mkdir(exist_ok=True)
    cv2.imwrite(str(ROOT / "out" / "render_check.png"), out)
    print("wrote out/render_check.png for episode", ep)
    print("tool-to-tool distance at frame 0 (m):",
          np.linalg.norm(d.site("left_tcp_site").xpos - d.site("right_tcp_site").xpos).round(4))
    print("plug tip to socket mouth (m):",
          np.linalg.norm(d.site("plug_tip").xpos - d.site("socket_mouth").xpos).round(4)
          if m.nsite else None)


if __name__ == "__main__":
    main()
