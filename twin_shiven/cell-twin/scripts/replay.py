"""Replay a recorded episode through the twin and compare.

Two modes:
  kinematic  qpos is set from observation.state each frame; renders a 3-camera
             video with the real frames underneath (visual check).
  dynamic    ctrl is set from action each frame; the position actuators track;
             reports per-joint tracking error vs the real observation.state and
             actuator force vs observation.effort (physics / gain check).

usage: python scripts/replay.py <episode_id> [--mode kinematic|dynamic] [--video]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from cell_twin.layout import j6_offset, FPS, JOINT_NAMES, grip_to_slide  # noqa: E402
from cell_twin.scene import compile_model, load_part_offsets  # noqa: E402

EPS = ROOT / "inputs" / "nuc" / "episodes" / "objects"
CAMS = (("scene_cam", "scene.mp4"), ("right_wrist_cam", "right_wrist.mp4"), ("left_wrist_cam", "left_wrist.mp4"))


def joint_ids(m):
    ids = {}
    for arm, off in (("left", 0), ("right", 7)):
        for i, jn in enumerate(JOINT_NAMES):
            ids[(arm, i)] = m.joint(f"{arm}_{jn}").qposadr[0]
        ids[(arm, "fl")] = m.joint(f"{arm}_left_finger").qposadr[0]
        ids[(arm, "fr")] = m.joint(f"{arm}_right_finger").qposadr[0]
    return ids


def act_ids(m):
    ids = {}
    for arm in ("left", "right"):
        for i, jn in enumerate(JOINT_NAMES):
            ids[(arm, i)] = m.actuator(f"{arm}_{jn}").id
        ids[(arm, "g")] = m.actuator(f"{arm}_gripper").id
    return ids


def set_qpos(d, jid, state14):
    for arm, off in (("left", 0), ("right", 7)):
        for i in range(6):
            d.qpos[jid[(arm, i)]] = state14[off + i] + (j6_offset(arm) if i == 5 else 0.0)
        s = grip_to_slide(state14[off + 6])
        d.qpos[jid[(arm, "fl")]] = s
        d.qpos[jid[(arm, "fr")]] = s


def set_qvel_from(d, jid, dstate14):
    for arm, off in (("left", 0), ("right", 7)):
        for i in range(6):
            d.qvel[d.model.jnt_dofadr[d.model.joint(f"{arm}_{JOINT_NAMES[i]}").id]] = dstate14[off + i]


def set_ctrl(d, aid, action14):
    for arm, off in (("left", 0), ("right", 7)):
        for i in range(6):
            d.ctrl[aid[(arm, i)]] = action14[off + i] + (j6_offset(arm) if i == 5 else 0.0)
        d.ctrl[aid[(arm, "g")]] = grip_to_slide(action14[off + 6])


def place_plug(m, d):
    """Put the free plug where the weld wants it for the current arm pose."""
    from cell_twin.scene import PLUG_IN_TOOL
    T = PLUG_IN_TOOL["right_link_6"]
    tool = m.body("right_link_6").id
    Rt = d.xmat[tool].reshape(3, 3)
    pos = d.xpos[tool] + Rt @ T[:3, 3]
    q = np.empty(4)
    mujoco.mju_mat2Quat(q, (Rt @ T[:3, :3]).reshape(-1))
    adr = m.joint("plug_free").qposadr[0]
    d.qpos[adr:adr + 3] = pos
    d.qpos[adr + 3:adr + 7] = q
    d.qvel[m.joint("plug_free").dofadr[0]:m.joint("plug_free").dofadr[0] + 6] = 0
    mujoco.mj_forward(m, d)


def seat_depth(m, d):
    """Plug tip depth inside the socket along the socket axis (m); positive = inside the mouth."""
    mouth = d.site("socket_mouth").xpos
    axis = d.site("socket_mouth").xmat.reshape(3, 3)[:, 2]
    return float((mouth - d.site("plug_tip").xpos) @ axis)


def read_qpos(d, jid, arm):
    return np.array([d.qpos[jid[(arm, i)]] for i in range(6)])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("episode")
    ap.add_argument("--mode", default="kinematic", choices=("kinematic", "dynamic", "kincontact", "track"))
    ap.add_argument("--video", action="store_true")
    ap.add_argument("--frames", type=int, default=0, help="limit frames (0 = all)")
    ap.add_argument("--free-plug", action="store_true", help="plug is a free body held by a soft weld")
    ap.add_argument("--eps-root", default=None, help="episode objects root (default: inputs/nuc/episodes/objects)")
    ap.add_argument("--per-episode", action="store_true", help="fit the part offsets from this episode's own last frame")
    a = ap.parse_args()

    global EPS
    if a.eps_root:
        EPS = Path(a.eps_root)
    z = np.load(EPS / a.episode / "frames.npz")
    state, action, effort = z["observation.state"], z["action"], z["observation.effort"]
    n = len(state) if not a.frames else min(a.frames, len(state))

    offsets = load_part_offsets()
    if a.per_episode:
        from scripts.calibrate import episode_offsets
        offsets = episode_offsets(state[-1])
    spec, m = compile_model(part_mode="calibrated", part_offsets=offsets, free_plug=a.free_plug)
    if a.mode == "track":
        for i in range(m.nu):
            if m.actuator(i).name.endswith(("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")):
                m.actuator_gainprm[i, 0] *= 10
                m.actuator_biasprm[i, 1] *= 10
                m.actuator_biasprm[i, 2] *= 3
    d = mujoco.MjData(m)
    jid, aid = joint_ids(m), act_ids(m)
    substeps = int(round(1.0 / FPS / m.opt.timestep))

    set_qpos(d, jid, state[0])
    mujoco.mj_forward(m, d)
    if a.free_plug:
        place_plug(m, d)
    if a.mode in ("dynamic", "track"):
        set_ctrl(d, aid, action[0] if a.mode == "dynamic" else state[0])
        for _ in range(int(0.5 / m.opt.timestep)):   # settle under gravity at the first target
            mujoco.mj_step(m, d)

    writer = None
    caps = {}
    if a.video:
        (ROOT / "out").mkdir(exist_ok=True)
        r = mujoco.Renderer(m, 240, 320)
        out_path = ROOT / "out" / f"replay_{a.mode}_{a.episode[:8]}.mp4"
        writer = cv2.VideoWriter(str(out_path), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (960, 480))
        caps = {cam: cv2.VideoCapture(str(EPS / a.episode / mp4)) for cam, mp4 in CAMS}

    sim_q = np.zeros((n, 12))
    sim_f = np.zeros((n, 12))
    depth = np.zeros(n)
    weld_f = np.zeros(n)
    contact_f = np.zeros(n)
    for k in range(n):
        if a.mode == "kinematic":
            set_qpos(d, jid, state[k])
            mujoco.mj_forward(m, d)
        elif a.mode == "kincontact":
            s0, s1 = state[max(k - 1, 0)], state[k]
            for i in range(substeps):
                w = (i + 1) / substeps
                set_qpos(d, jid, s0 + w * (s1 - s0))
                set_qvel_from(d, jid, (s1 - s0) * FPS)
                mujoco.mj_step(m, d)
        else:
            set_ctrl(d, aid, action[k] if a.mode == "dynamic" else state[k])
            for _ in range(substeps):
                mujoco.mj_step(m, d)
        if a.free_plug:
            depth[k] = seat_depth(m, d)
            for ci in range(d.ncon):
                c = d.contact[ci]
                if "plug" in m.geom(c.geom1).name + m.geom(c.geom2).name:
                    ff = np.zeros(6)
                    mujoco.mj_contactForce(m, d, ci, ff)
                    contact_f[k] += abs(ff[0])
            eid = m.equality("plug_grasp").id
            if d.efc_type.size and eid in d.efc_id:
                weld_f[k] = np.linalg.norm(d.efc_force[d.efc_id == eid][:3])
        sim_q[k, :6] = read_qpos(d, jid, "left")
        sim_q[k, 6:] = read_qpos(d, jid, "right")
        for j, arm in enumerate(("left", "right")):
            for i in range(6):
                sim_f[k, 6 * j + i] = d.actuator_force[aid[(arm, i)]]
        if writer is not None:
            panels = []
            for cam, _ in CAMS:
                r.update_scene(d, camera=cam)
                twin = cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR)
                ok, real = caps[cam].read()
                real = cv2.resize(real, (320, 240)) if ok else np.zeros((240, 320, 3), np.uint8)
                panels.append(np.vstack([twin, real]))
            writer.write(np.hstack(panels))
    if writer is not None:
        writer.release()
        print("wrote", out_path)

    real_q = np.hstack([state[:n, :6], state[:n, 7:13]])
    real_f = np.hstack([effort[:n, :6], effort[:n, 7:13]])
    err = sim_q - real_q
    print(f"mode={a.mode} frames={n}")
    print("joint tracking RMS error (mrad), left j1..j6 | right j1..j6:")
    print("  ", (np.sqrt((err ** 2).mean(0)) * 1000).round(1))
    print("max abs error (mrad):", (np.abs(err).max(0) * 1000).round(1))
    if a.mode == "dynamic":
        print("actuator force vs real effort, right arm, mean (Nm):")
        print("  sim ", sim_f[:, 6:].mean(0).round(2))
        print("  real", real_f[:, 6:].mean(0).round(2))
        cmd_err_real = np.sqrt(((action[:n, 7:13] - state[:n, 7:13]) ** 2).mean(0)) * 1000
        cmd_err_sim = np.sqrt(((action[:n, 7:13] - sim_q[:, 6:]) ** 2).mean(0)) * 1000
        print("command-to-measured RMS (mrad), right: real", cmd_err_real.round(1), " sim", cmd_err_sim.round(1))
    if a.free_plug:
        from cell_twin.scene import INSERT_DEPTH
        # seat = depth reached AND every pin tip inside its cavity (lateral check in the housing frame),
        # so pins sliding alongside the housing cannot count as seated
        from cell_twin.scene import N_PINS, PITCH
        Ts = np.eye(4)
        Ts[:3, :3] = d.body("socket").xmat.reshape(3, 3)
        Ts[:3, 3] = d.body("socket").xpos
        inv = np.linalg.inv(Ts)
        x0 = -(N_PINS - 1) * PITCH / 2
        lateral = []
        for i in range(N_PINS):
            p = (inv @ np.append(d.geom(f"pin_tip_{i}").xpos, 1))[:3]
            lateral.append(max(abs(p[0] - (x0 + i * PITCH)), abs(p[1])))
        pins_in = max(lateral) < 0.0006
        seated = depth[-1] >= 0.9 * INSERT_DEPTH and pins_in
        print(f"      pins inside cavities at end: {'yes' if pins_in else 'no'} (max lateral {max(lateral)*1000:.2f} mm)")
        print(f"SEAT: {'yes' if seated else 'no'}  max depth {depth.max()*1000:.2f} mm of {INSERT_DEPTH*1000:.1f}  "
              f"final depth {depth[-1]*1000:.2f} mm  first-contact frame {int(np.argmax(depth > 0)) if (depth > 0).any() else -1}  "
              f"peak grasp force {weld_f.max():.2f} N")
        flange = np.linalg.norm(d.site("left_tcp_site").xpos - d.site("right_tcp_site").xpos)
        real_flange_end = None
        print(f"      contact frames {(contact_f > 0).sum()}  peak contact force {contact_f.max():.2f} N  "
              f"sim flange distance at end {flange*1000:.1f} mm (seat = 304.8)")
        np.save(ROOT / "out" / f"depth_{a.mode}_{a.episode[:8]}.npy", np.stack([depth, weld_f, contact_f]))


if __name__ == "__main__":
    main()
