"""Derive where each connector half sits in its arm's tool frame, from the recordings.

At the last frame of a saved t5 episode the plug is seated. Define the mate
frame M as the midpoint between the two flanges, z along the left tool's z
(the approach axis), x chosen so M is right-handed with the left tool's x.
Then socket_in_left = inv(T_left) M and plug_in_right = inv(T_right) M, averaged
over episodes. No part dimension or finger length is assumed: the parts land
where the recordings say they meet.

writes out/part_offsets.json  (4x4 lists, plus spread statistics)
"""
from __future__ import annotations

import glob
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from cell_twin.layout import base_transform, j6_offset  # noqa: E402
from cell_twin.scene import INSERT_DEPTH  # noqa: E402

PREINSERTION2 = "552d4ea6-1f26-4783-8244-c8ba3eef00fa"


def tool_T(m, d, arm, q):
    d.qpos[:] = 0
    d.qpos[:6] = q
    d.qpos[5] += j6_offset(arm)          # gripper mount roll (layout.JOINT6_OFFSET_DEG)
    mujoco.mj_forward(m, d)
    T = np.eye(4)
    T[:3, :3] = d.site("tcp_site").xmat.reshape(3, 3)
    T[:3, 3] = d.site("tcp_site").xpos
    return base_transform(arm) @ T


def mate_frame(TL, TR):
    z = TL[:3, 2] / np.linalg.norm(TL[:3, 2])
    x = TL[:3, 0] - (TL[:3, 0] @ z) * z
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    M = np.eye(4)
    M[:3, 0], M[:3, 1], M[:3, 2] = x, y, z
    M[:3, 3] = (TL[:3, 3] + TR[:3, 3]) / 2
    return M


def avg_T(Ts):
    """Average rigid transforms: mean translation, quaternion mean via SVD-free normalisation."""
    t = np.mean([T[:3, 3] for T in Ts], axis=0)
    qs = []
    for T in Ts:
        q = np.empty(4)
        mujoco.mju_mat2Quat(q, T[:3, :3].reshape(-1))
        if qs and q @ qs[0] < 0:
            q = -q
        qs.append(q)
    q = np.mean(qs, axis=0)
    q /= np.linalg.norm(q)
    R = np.empty(9)
    mujoco.mju_quat2Mat(R, q)
    out = np.eye(4)
    out[:3, :3] = R.reshape(3, 3)
    out[:3, 3] = t
    return out


def episode_offsets(state14_last):
    """Mate frame from one episode's seated last frame (same construction as main())."""
    spec = mujoco.MjSpec.from_file(str(ROOT / "assets" / "yam" / "yam_linear_4310.xml"))
    m = spec.compile()
    d = mujoco.MjData(m)
    TL, TR = tool_T(m, d, "left", state14_last[:6]), tool_T(m, d, "right", state14_last[7:13])
    M = mate_frame(TL, TR)
    return {"mate_in_left_tool": (np.linalg.inv(TL) @ M).tolist(), "mate_in_right_tool": (np.linalg.inv(TR) @ M).tolist()}


def main():
    spec = mujoco.MjSpec.from_file(str(ROOT / "assets" / "yam" / "yam_linear_4310.xml"))
    m = spec.compile()
    d = mujoco.MjData(m)
    socks, plugs, dists = [], [], []
    for src in sorted(glob.glob(str(ROOT / "inputs/nuc/episodes/objects/*/source.json"))):
        meta = json.load(open(src))
        if meta.get("starting_pose") != PREINSERTION2 or meta["title"].startswith("Correction"):
            continue
        s = np.load(src.replace("source.json", "frames.npz"))["observation.state"]
        TL, TR = tool_T(m, d, "left", s[-1, :6]), tool_T(m, d, "right", s[-1, 7:13])
        M = mate_frame(TL, TR)
        socks.append(np.linalg.inv(TL) @ M)
        plugs.append(np.linalg.inv(TR) @ M)
        dists.append(np.linalg.norm(TL[:3, 3] - TR[:3, 3]))
    S, P = avg_T(socks), avg_T(plugs)
    # the socket mouth sits half the insertion depth behind the mate point along +z of M;
    # the plug body centre sits half its length in front. scene.py applies those from this frame.
    out = {
        "n_episodes": len(socks),
        "seat_flange_distance_m": {"mean": float(np.mean(dists)), "std": float(np.std(dists))},
        "mate_in_left_tool": S.tolist(),
        "mate_in_right_tool": P.tolist(),
        "socket_spread_mm": (np.std([T[:3, 3] for T in socks], axis=0) * 1000).round(2).tolist(),
        "plug_spread_mm": (np.std([T[:3, 3] for T in plugs], axis=0) * 1000).round(2).tolist(),
        "insert_depth_m_assumed": INSERT_DEPTH,
    }
    (ROOT / "out").mkdir(exist_ok=True)
    json.dump(out, open(ROOT / "out" / "part_offsets.json", "w"), indent=1)
    print(json.dumps({k: v for k, v in out.items() if "tool" not in k}, indent=1))
    print("mate in left tool: xyz(mm)", (S[:3, 3] * 1000).round(1), " z-axis", S[:3, 2].round(3))
    print("mate in right tool: xyz(mm)", (P[:3, 3] * 1000).round(1), " z-axis", P[:3, 2].round(3))


if __name__ == "__main__":
    main()
