"""Fit the right arm's base pose from the seated frames.

At seat the header and housing are coaxial and the parts sit on each tool's axis
(printed fingers are symmetric), so the two tool z-axes must be collinear and
opposed. The left base is held at its layout value; the right base (x, y, z, yaw)
is fitted so that over the 16 seated end frames the right tool origin lies on the
left tool's axis and the axes are anti-parallel. Also reports the residual spread
and the implied flange-to-flange distance, which fixes the finger length.

writes out/base_fit.json
"""
from __future__ import annotations

import glob
import json
import sys
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import minimize

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from cell_twin.layout import ARM_BASE  # noqa: E402

PRE = "552d4ea6-1f26-4783-8244-c8ba3eef00fa"


def main():
    spec = mujoco.MjSpec.from_file(str(ROOT / "assets/yam/yam_linear_4310.xml"))
    m = spec.compile()
    d = mujoco.MjData(m)

    def tool_local(q):
        d.qpos[:] = 0
        d.qpos[:6] = q
        mujoco.mj_forward(m, d)
        T = np.eye(4)
        T[:3, :3] = d.site("tcp_site").xmat.reshape(3, 3)
        T[:3, 3] = d.site("tcp_site").xpos
        return T

    seated = []
    for src in sorted(glob.glob(str(ROOT / "inputs/nuc/episodes/objects/*/source.json"))):
        meta = json.load(open(src))
        if meta.get("starting_pose") != PRE or meta["title"].startswith("Correction"):
            continue
        s = np.load(src.replace("source.json", "frames.npz"))["observation.state"][-1]
        seated.append((tool_local(s[:6]), tool_local(s[7:13])))
    print(len(seated), "seated frames")

    def base_T(xyz, yaw):
        c, s_ = np.cos(yaw), np.sin(yaw)
        T = np.eye(4)
        T[:3, :3] = [[c, -s_, 0], [s_, c, 0], [0, 0, 1]]
        T[:3, 3] = xyz
        return T

    L = base_T(ARM_BASE["left"][0], np.radians(ARM_BASE["left"][1]))

    def residuals(x):
        R = base_T(x[:3], x[3])
        lat, ang, dist = [], [], []
        for TL_l, TR_l in seated:
            TL, TR = L @ TL_l, R @ TR_l
            zL, zR = TL[:3, 2], TR[:3, 2]
            v = TR[:3, 3] - TL[:3, 3]
            along = v @ zL
            lat.append(np.linalg.norm(v - along * zL))         # right origin off the left axis
            ang.append(np.arccos(np.clip(-zL @ zR, -1, 1)))     # axes should be opposed
            dist.append(along)
        return np.array(lat), np.array(ang), np.array(dist)

    def cost(x):
        lat, ang, _ = residuals(x)
        return float(np.mean(lat ** 2) * 1e4 + np.mean(ang ** 2) * 10)

    x0 = np.array([*ARM_BASE["right"][0], np.radians(ARM_BASE["right"][1])])
    lat0, ang0, dist0 = residuals(x0)
    print(f"layout base: lateral {lat0.mean()*1000:.1f} mm (std {lat0.std()*1000:.1f}), axis angle {np.degrees(ang0.mean()):.2f} deg, "
          f"flange distance {dist0.mean()*1000:.1f} mm")
    best = minimize(cost, x0, method="Nelder-Mead", options={"maxiter": 4000, "xatol": 1e-6, "fatol": 1e-9})
    lat, ang, dist = residuals(best.x)
    print(f"fitted base: x {best.x[0]:.4f} y {best.x[1]:.4f} z {best.x[2]:.4f} yaw {np.degrees(best.x[3]):.2f} deg")
    print(f"  lateral {lat.mean()*1000:.2f} mm (std {lat.std()*1000:.2f}), axis angle {np.degrees(ang.mean()):.2f} deg "
          f"(std {np.degrees(ang.std()):.2f}), flange distance {dist.mean()*1000:.1f} mm (std {dist.std()*1000:.2f})")
    print(f"  shift from layout: dx {1000*(best.x[0]-x0[0]):+.1f} mm dy {1000*(best.x[1]-x0[1]):+.1f} mm dz {1000*(best.x[2]-x0[2]):+.1f} mm "
          f"dyaw {np.degrees(best.x[3]-x0[3]):+.2f} deg")
    json.dump({"right_base_xyz": best.x[:3].tolist(), "right_base_yaw_deg": float(np.degrees(best.x[3])),
               "lateral_mm_mean": float(lat.mean() * 1000), "axis_deg_mean": float(np.degrees(ang.mean())),
               "flange_distance_mm": float(dist.mean() * 1000), "n": len(seated)},
              open(ROOT / "out/base_fit.json", "w"), indent=1)


if __name__ == "__main__":
    main()
