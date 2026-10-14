"""Fit a wrist camera's mount pose (and fovy) from the recordings.

In every right-wrist frame the LEFT gripper is visible, and its pose is known from the joints, so the only
unknown is the camera's pose on the right flange. The real image gives a mask of dark pixels (the grippers
are matte black on a light wood background); the twin renders a segmentation mask of the opposite arm from
the candidate camera. Score = how much of the rendered opposite-arm mask lands on dark real pixels, plus a
coverage term. Coarse search over orientations first (the mount could face any way), then Nelder-Mead.

usage: python scripts/fit_wrist_camera.py right [--episodes 6] [--frames 3] [--iters 300]
writes out/fit_<arm>_wrist.json and out/fit_<arm>_wrist.png (rendered mask edges in red over the real frames)
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

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from cell_twin.scene import compile_model, load_part_offsets  # noqa: E402
from scripts.replay import joint_ids, set_qpos  # noqa: E402
from scripts.fit_camera import read_frame, rot_from_rpy  # noqa: E402

EPS = ROOT / "inputs" / "nuc" / "episodes" / "objects"
W, H = 320, 240


def dark_mask(img_bgr, thresh=70):
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    return hsv[:, :, 2] < thresh


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("arm", choices=("right", "left"))
    ap.add_argument("--episodes", type=int, default=6)
    ap.add_argument("--frames", type=int, default=3)
    ap.add_argument("--iters", type=int, default=300)
    a = ap.parse_args()
    other = "left" if a.arm == "right" else "right"
    cam_name = f"{a.arm}_wrist_cam"

    spec, m = compile_model(part_mode="calibrated", part_offsets=load_part_offsets())
    d = mujoco.MjData(m)
    jid = joint_ids(m)
    cam_id = m.camera(cam_name).id
    r = mujoco.Renderer(m, H, W)
    r.enable_segmentation_rendering()
    # geoms of the opposite arm, and of the camera's OWN fingers (rigid with the camera: their image position
    # depends only on the mount pose, which pins it down independently of the arm joints)
    other_geoms = np.array([i for i in range(m.ngeom) if m.body(m.geom_bodyid[i]).name.startswith(other + "_")])
    other_set = np.zeros(m.ngeom + 1, bool); other_set[other_geoms] = True
    own_finger_geoms = np.array([i for i in range(m.ngeom) if m.body(m.geom_bodyid[i]).name.startswith(a.arm + "_")
                                 and ("finger" in m.body(m.geom_bodyid[i]).name or "pad" in m.body(m.geom_bodyid[i]).name)])
    own_set = np.zeros(m.ngeom + 1, bool); own_set[own_finger_geoms] = True
    # the real opposite gripper sits in the central column band; the table at the left edge is also dark
    band = np.zeros((H, W), bool); band[:, int(W * 0.22):int(W * 0.78)] = True

    samples = []
    for ep in sorted(p.name for p in EPS.iterdir())[: a.episodes]:
        z = np.load(EPS / ep / "frames.npz")
        n = len(z["observation.state"])
        for k in np.linspace(0, n - 1, a.frames).astype(int):
            fr = read_frame(EPS / ep / f"{a.arm}_wrist.mp4", int(k))
            if fr is not None:
                samples.append((z["observation.state"][k], dark_mask(fr), fr))
    print(len(samples), "frames")

    pos0 = m.cam_pos[cam_id].copy(); q0 = m.cam_quat[cam_id].copy(); fov0 = float(m.cam_fovy[cam_id])
    R0 = np.empty(9); mujoco.mju_quat2Mat(R0, q0); R0 = R0.reshape(3, 3)

    def apply(x):
        m.cam_pos[cam_id] = pos0 + np.clip(x[:3], -0.025, 0.025)      # the printed mount is near the stock one
        R = rot_from_rpy(x[3:6]) @ R0
        q = np.empty(4); mujoco.mju_mat2Quat(q, R.reshape(-1)); m.cam_quat[cam_id] = q
        m.cam_fovy[cam_id] = fov0                                      # intrinsics are known from the crop; not fitted

    def render_masks(state):
        set_qpos(d, jid, state); mujoco.mj_forward(m, d)
        r.update_scene(d, camera=cam_name)
        seg = r.render()[:, :, 0].astype(int)           # geom id per pixel, -1 background
        idx = np.clip(seg, -1, m.ngeom); valid = seg >= 0
        return other_set[idx] & valid, own_set[idx] & valid

    def score(x):
        apply(x)
        tot = 0.0
        tot += 2.0 * float(np.sum(np.maximum(0, np.abs(x[:3]) - 0.025)) * 100)    # outside the mount box: penalty
        for state, dark, _ in samples:
            mk_other, mk_own = render_masks(state)
            # own fingers: must be dark where rendered, and the real dark lower-centre region should be covered
            # own fingers: IoU against the dark pixels in the bottom-centre window, where the real fingers always are
            win = np.zeros((H, W), bool); win[int(H * 0.5):, int(W * 0.25):int(W * 0.75)] = True
            real_own = dark & win
            inter = (mk_own & real_own).sum(); union = (mk_own | real_own).sum()
            iou_own = inter / max(1, union)
            tot += 1.5 * (1 - iou_own)
            if mk_own.sum() < 300:
                tot += 1.0
            # opposite arm: must be dark where rendered, within the central band
            mo = mk_other & band
            if mo.sum() < 200:
                tot += 1.0
            else:
                tot += 0.6 * (1 - dark[mo].mean())
        return tot / len(samples)

    x0 = np.zeros(7); s0 = score(x0); best0, bestx = s0, x0.copy()
    for rx in np.radians([-60, -30, 0, 30, 60]):
        for ry in np.radians([-30, 0, 30]):
            for rz in np.radians([0, 90, 180, 270]):
                for dz in (-0.03, 0.0, 0.03):
                    x = np.zeros(7); x[2] = dz; x[3:6] = [rx, ry, rz]
                    sc = score(x)
                    if sc < best0:
                        best0, bestx = sc, x.copy()
    print(f"coarse: {s0:.3f} -> {best0:.3f} at rpy {np.degrees(bestx[3:6]).round(0)}")
    res = minimize(score, bestx, method="Nelder-Mead",
                   options={"maxiter": a.iters, "xatol": 1e-4, "fatol": 1e-4,
                            "initial_simplex": np.vstack([bestx] + [bestx + np.eye(7)[i] * st for i, st in
                                                          enumerate([0.01, 0.01, 0.01, 0.08, 0.08, 0.08, 0.0])])})
    apply(res.x)
    print(f"refined: {best0:.3f} -> {res.fun:.3f}; cam pos (mount frame) {m.cam_pos[cam_id].round(4)} fovy {m.cam_fovy[cam_id]:.1f} "
          f"rpy delta {np.degrees(res.x[3:6]).round(1)}")
    json.dump({"camera": cam_name, "pos": m.cam_pos[cam_id].tolist(), "quat": m.cam_quat[cam_id].tolist(),
               "fovy": float(m.cam_fovy[cam_id]), "score0": s0, "score": float(res.fun), "frames": len(samples)},
              open(ROOT / "out" / f"fit_{a.arm}_wrist.json", "w"), indent=1)
    tiles = []
    for state, dark, fr in samples[:6]:
        mo, mw = render_masks(state)
        e1 = cv2.Canny(mo.astype(np.uint8) * 255, 50, 150) > 0; e2 = cv2.Canny(mw.astype(np.uint8) * 255, 50, 150) > 0
        ov = fr.copy(); ov[e1] = (0, 0, 255); ov[e2] = (0, 255, 255); tiles.append(ov)
    cv2.imwrite(str(ROOT / "out" / f"fit_{a.arm}_wrist.png"), np.vstack([np.hstack(tiles[:3]), np.hstack(tiles[3:6])]))
    print("wrote out/fit_%s_wrist.json and .png" % a.arm)


if __name__ == "__main__":
    main()
