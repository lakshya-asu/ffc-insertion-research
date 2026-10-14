"""Fit a camera's pose (and vertical FOV) to the real frames by edge matching.

The arms are in every frame and their joint angles are recorded, so the twin can render
the same instant. We compare Canny edges of the real frame with edges of the rendered
frame through a distance transform (chamfer score) and minimise over the camera's
6-DoF pose plus fovy with Nelder-Mead, using several frames from several episodes.

usage: python scripts/fit_camera.py scene_cam [--frames 4] [--episodes 3] [--iters 300]
writes out/fit_<camera>.json and out/fit_<camera>.png (edges overlaid, before/after)
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import mujoco
import numpy as np
from scipy.optimize import minimize
from scipy.ndimage import distance_transform_edt

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from cell_twin.scene import compile_model, load_part_offsets  # noqa: E402
from scripts.replay import joint_ids, set_qpos  # noqa: E402

EPS = ROOT / "inputs" / "nuc" / "episodes" / "objects"
CAM_VIDEO = {"scene_cam": "scene.mp4", "right_wrist_cam": "right_wrist.mp4", "left_wrist_cam": "left_wrist.mp4"}
W, H = 320, 240


def read_frame(mp4, k):
    cap = cv2.VideoCapture(str(mp4))
    cap.set(cv2.CAP_PROP_POS_FRAMES, k)
    ok, f = cap.read()
    cap.release()
    return cv2.resize(f, (W, H)) if ok else None


def edges(img_bgr):
    g = cv2.GaussianBlur(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY), (3, 3), 0)
    return cv2.Canny(g, 60, 140) > 0


def rot_from_rpy(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("camera")
    ap.add_argument("--frames", type=int, default=4)
    ap.add_argument("--episodes", type=int, default=3)
    ap.add_argument("--iters", type=int, default=300)
    a = ap.parse_args()

    spec, m = compile_model(part_mode="calibrated", part_offsets=load_part_offsets())
    d = mujoco.MjData(m)
    jid = joint_ids(m)
    cam_id = m.camera(a.camera).id
    r = mujoco.Renderer(m, H, W)

    # samples: (state14, real edge distance transform)
    samples = []
    for ep in sorted(p.name for p in EPS.iterdir())[: a.episodes]:
        z = np.load(EPS / ep / "frames.npz")
        n = len(z["observation.state"])
        for k in np.linspace(0, n - 1, a.frames).astype(int):
            fr = read_frame(EPS / ep / CAM_VIDEO[a.camera], int(k))
            if fr is None:
                continue
            e = edges(fr)
            samples.append((z["observation.state"][k], distance_transform_edt(~e), fr))
    print(f"{len(samples)} frames")

    # parameters: camera pos (3), rpy delta (3), fovy (1), initialised from the model
    pos0 = m.cam_pos[cam_id].copy()
    q0 = m.cam_quat[cam_id].copy()
    fov0 = float(m.cam_fovy[cam_id])
    R0 = np.empty(9)
    mujoco.mju_quat2Mat(R0, q0)
    R0 = R0.reshape(3, 3)

    def apply(x):
        m.cam_pos[cam_id] = pos0 + x[:3]
        R = rot_from_rpy(x[3:6]) @ R0
        q = np.empty(4)
        mujoco.mju_mat2Quat(q, R.reshape(-1))
        m.cam_quat[cam_id] = q
        m.cam_fovy[cam_id] = fov0 * np.exp(x[6])

    def score(x):
        apply(x)
        tot = 0.0
        for state, dt, _ in samples:
            set_qpos(d, jid, state)
            mujoco.mj_forward(m, d)
            r.update_scene(d, camera=a.camera)
            e = edges(cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR))
            if e.sum() < 50:
                tot += 50.0
                continue
            tot += float(np.mean(np.minimum(dt[e], 20.0)))     # mean distance from rendered edges to real edges, capped
        return tot / len(samples)

    # coarse initialisation: the hand-placed camera can be off by a quarter turn about its axis
    # and tens of cm in height, which Nelder-Mead cannot cross. Score a small grid first.
    x0 = np.zeros(7)
    s0 = score(x0)
    best0, bestx = s0, x0.copy()
    for yaw in np.radians([0, 90, 180, 270]):
        for dz in (-0.25, -0.1, 0.0, 0.1):
            for df in (-0.3, 0.0, 0.3):
                x = np.zeros(7); x[2] = dz; x[5] = yaw; x[6] = df
                sc = score(x)
                if sc < best0:
                    best0, bestx = sc, x.copy()
    print(f"coarse init: {s0:.3f} -> {best0:.3f} at yaw {np.degrees(bestx[5]):.0f} deg, dz {bestx[2]:+.2f} m, fovy x{np.exp(bestx[6]):.2f}")
    x0 = bestx
    best = minimize(score, x0, method="Nelder-Mead",
                    options={"maxiter": a.iters, "xatol": 1e-4, "fatol": 1e-4,
                             "initial_simplex": np.vstack([x0] + [x0 + np.eye(7)[i] * st for i, st in
                                                           enumerate([0.05, 0.05, 0.05, 0.08, 0.08, 0.08, 0.1])])})
    apply(best.x)
    print(f"score {s0:.3f} -> {best.fun:.3f}  pos {m.cam_pos[cam_id].round(4)}  fovy {m.cam_fovy[cam_id]:.1f}  "
          f"rpy delta deg {np.degrees(best.x[3:6]).round(2)}")
    out = {"camera": a.camera, "pos": m.cam_pos[cam_id].tolist(), "quat": m.cam_quat[cam_id].tolist(),
           "fovy": float(m.cam_fovy[cam_id]), "score_before": s0, "score_after": float(best.fun),
           "frames": len(samples), "delta": best.x.tolist()}
    json.dump(out, open(ROOT / "out" / f"fit_{a.camera}.json", "w"), indent=1)
    # overlay: real frame with rendered edges (red) for the first 3 samples, before and after
    tiles = []
    for xi in (x0, best.x):
        apply(xi)
        row = []
        for state, _, fr in samples[:3]:
            set_qpos(d, jid, state)
            mujoco.mj_forward(m, d)
            r.update_scene(d, camera=a.camera)
            e = edges(cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR))
            ov = fr.copy()
            ov[e] = (0, 0, 255)
            row.append(ov)
        tiles.append(np.hstack(row))
    cv2.imwrite(str(ROOT / "out" / f"fit_{a.camera}.png"), np.vstack(tiles))
    print("wrote out/fit_%s.json and .png" % a.camera)


if __name__ == "__main__":
    main()
