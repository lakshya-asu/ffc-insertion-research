"""Build the two-arm cell as one MuJoCo model.

Two Menagerie YAM (linear_4310 gripper) arms attached at the recorded base
poses, a table, a top-down scene camera, and the two connector halves.

Parts are attached to the arms' tool frames, not placed in the world: the
socket rides in the left (fixture) gripper and the plug in the right gripper,
exactly as in the cell. Their offsets come from the recordings (see
`calibrate.py`): at seat the two flanges are 304.8 mm apart along the approach
axis, so each printed finger set carries its part about 152 mm from the flange.

Placeholders (flagged, replace with caliper numbers):
  PLUG_W, PLUG_T, PLUG_L   XH-class 5-pin housing, 2.54 mm pitch
  INSERT_DEPTH             housing-to-header overlap at full seat
  SCENE_CAM_*              top-down camera pose, fitted by eye to the real frame
"""
from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

from .layout import ARM_BASE, TABLE_D, TABLE_W, base_rotation

ASSETS = Path(__file__).resolve().parent.parent / "assets" / "yam"

import json
import os

# ---- connector: 2.54 mm pitch 5-position male pin header into a 2510/Dupont-type female crimp housing ----
# Male (Würth 61300511121 drawing, generic 2.54 header): pin 0.64 sq, 6.0 mm mating length, insulator 2.54 high,
# 12.7 mm long for 5 positions. Female (2510 type): length N*2.54+0.66 = 13.36, 2.54 thick, ~14 mm tall,
# one square cavity per contact with an entry chamfer. Full seat = housing face on the header insulator.
PITCH = 0.00254
N_PINS = 5
PIN_SQ = 0.00064
PIN_LEN = 0.0060                                  # mating length above the insulator
PIN_TIP_CHAMFER = 0.0004                          # tip narrowing over the last 0.4 mm (box approximation)
HEADER_L, HEADER_T, HEADER_H = 0.01336, 0.00254, 0.0140    # male PLUG housing (video 2026-10-06): same body as the female, pins out of its face
HOUSING_L, HOUSING_T, HOUSING_H = 0.01336, 0.00254, 0.0140
CAVITY_SQ = float(os.environ.get("TWIN_CAVITY_MM", "1.2")) * 1e-3    # square opening per contact; 1.0 jammed a 0.2 mm offset row at 50 N, placeholder
CAVITY_CHAMFER = float(os.environ.get("TWIN_CHAMFER_MM", "0.4")) * 1e-3   # entry chamfer size, placeholder
CAVITY_DEPTH = 0.0070                             # pin travel room inside the housing
# female contact springs: each cavity holds a sprung contact that grips the pin. Modelled as two compliant
# plates per cavity (facing the pin's y faces) with a small interference, from CONTACT_START below the face
# to the cavity floor. Tuned so a square insertion costs about the real 3 Nm rise on joint 2.
CONTACT_INTERFERENCE = float(os.environ.get("TWIN_CONTACT_INTERF_MM", "0.06")) * 1e-3   # per side
CONTACT_START = 0.0015                            # contact leaves start this far below the mating face
LEAF_T = 0.0004                                   # leaf thickness
LEAF_SOLREF = [float(os.environ.get("TWIN_LEAF_TC", "0.017")), 1.0]   # soft contact: the leaf's spring (tuned to the seat torque)
LEAF_SOLIMP = [0.5, 0.9, 0.0003, 0.5, 2.0]
INSERT_DEPTH = PIN_LEN                            # seat = full pin length inside
# legacy names used by replay/closeup (plug = header, socket = housing)
PLUG_W, PLUG_T, PLUG_L = HEADER_L, HEADER_T, HEADER_H + PIN_LEN
SOCKET_W, SOCKET_T, SOCKET_L = HOUSING_L, HOUSING_T, HOUSING_H
SEAT_FLANGE_DISTANCE = 0.3048                         # measured, 16 episodes, std 0.95 mm
# finger geometry: Menagerie pads are 136 mm from the flange; the seated flange distance (304.8) minus the parts
# stack (housing 14 held with 10 mm protruding, header insulator 2.54 pinched) says the real tips reach ~146 mm.
PAD_Z_STOCK = 0.1362
FINGER_EXT = float(os.environ.get("TWIN_FINGER_EXT_MM", "9.2")) * 1e-3   # 2*(PAD_Z + 7 mm) = 304.8 mm seated flange distance
PAD_Z = PAD_Z_STOCK + FINGER_EXT
HOUSING_PROTRUSION = 0.007                            # about half of each 14 mm housing is inside the pads (video 2026-10-06)
# finger reach from flange to the part's mating face, split evenly between the arms
FINGER_REACH = (SEAT_FLANGE_DISTANCE + INSERT_DEPTH) / 2.0   # 0.1559 m each side, placeholder split

# ---- scene camera placeholder (world frame, table top z = 0) --------------
SCENE_CAM_POS = (-0.0597, 0.1018, 0.4405)   # registered on the real top view (scripts/register_scene.py, 2026-10-07)
SCENE_CAM_QUAT = (1.0, 0.0, 0.0, 0.0)
SCENE_CAM_FOVY = 58.0
_reg = ASSETS.parent.parent / "out" / "register_scene.json"
if _reg.exists():                       # the registration file wins over the constants above
    _r = json.load(open(_reg)); SCENE_CAM_POS, SCENE_CAM_QUAT, SCENE_CAM_FOVY = tuple(_r["pos"]), tuple(_r["quat"]), float(_r["fovy"])


def _yaw_quat(yaw_deg: float) -> list[float]:
    h = np.radians(yaw_deg) / 2
    return [float(np.cos(h)), 0.0, 0.0, float(np.sin(h))]


def build(part_offsets: dict | None = None, free_plug: bool = False, part_mode: str = "geometric") -> mujoco.MjSpec:
    """Return an MjSpec of the whole cell.

    part_offsets: {"socket_in_left": 4x4, "plug_in_right": 4x4} tool-frame
    transforms from calibrate.py. When None, both parts sit on the tool z axis
    at FINGER_REACH. free_plug=True makes the plug a free body (contact
    physics); False welds it to the right tool for kinematic replay.
    """
    spec = mujoco.MjSpec()
    spec.modelname = "sandal_cell"
    spec.option.timestep = 0.002
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST   # joint springs/dampers (contact leaves) integrated implicitly
    spec.compiler.degree = False

    # visuals
    spec.visual.headlight.diffuse = [0.7, 0.7, 0.7]
    spec.visual.headlight.ambient = [0.35, 0.35, 0.35]
    spec.visual.global_.offwidth = 1280
    spec.visual.global_.offheight = 960

    # table and lights
    wb = spec.worldbody
    wb.add_light(pos=[0, 0, 1.5], dir=[0, 0, -1], type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL)
    wb.add_light(pos=[0.5, -0.5, 1.2], dir=[-0.3, 0.3, -1], type=mujoco.mjtLightType.mjLIGHT_SPOT, diffuse=[0.5, 0.5, 0.5])
    wb.add_geom(name="table", type=mujoco.mjtGeom.mjGEOM_BOX, pos=[0, 0, -0.02],
                size=[TABLE_W / 2, TABLE_D / 2, 0.02], rgba=[0.22, 0.22, 0.24, 1], contype=1, conaffinity=1)
    for arm, (xyz, _) in ARM_BASE.items():
        wb.add_geom(name=f"{arm}_plate", type=mujoco.mjtGeom.mjGEOM_BOX, pos=[xyz[0], xyz[1], xyz[2] / 2],
                    size=[0.08, 0.08, xyz[2] / 2], rgba=[0.6, 0.6, 0.62, 1])

    # scene camera (top-down)
    wb.add_camera(name="scene_cam", pos=list(SCENE_CAM_POS), quat=list(SCENE_CAM_QUAT), fovy=SCENE_CAM_FOVY,
                  resolution=[640, 480])

    # arms
    for arm, (xyz, yaw) in ARM_BASE.items():
        child = mujoco.MjSpec.from_file(str(ASSETS / "yam_linear_4310.xml"))
        child.modelname = arm
        _apply_real_gains(child)
        qb = np.empty(4); mujoco.mju_mat2Quat(qb, base_rotation(arm).reshape(-1))
        frame = wb.add_frame(pos=list(xyz), quat=[float(v) for v in qb])
        spec.attach(child, prefix=f"{arm}_", frame=frame)

    # parts, expressed in each arm's tool frame (tcp_site is at the flange)
    if part_mode == "geometric" or not part_offsets:
        # parts pinched between the pads, square to the tool: housing rear held, face 10 mm past the pad centre;
        # header insulator centred on the pads, pins protruding. Pick errors are applied on top (bench.py).
        # pads close along tool x and clamp each housing on its two narrow END faces (video 2026-10-06): the
        # 5-position row runs along the pinch axis x, the labelled face looks along y. About half of each
        # housing sits inside the pads; the rest protrudes toward the other arm, pins beyond it on the male.
        T_socket = _tz(PAD_Z + HOUSING_PROTRUSION - SOCKET_L / 2)
        T_plug = _tz(PAD_Z + HOUSING_PROTRUSION - HEADER_H / 2 + (PLUG_L / 2 - HEADER_H / 2))   # housing face at PAD_Z + protrusion
        if part_offsets and "pick" in part_offsets:
            T_plug = T_plug @ np.asarray(part_offsets["pick"]["header"])
            T_socket = T_socket @ np.asarray(part_offsets["pick"]["housing"])
    else:
        # calibrated: mate frame M in each tool frame from calibrate.py (replay of real episodes)
        M_left = np.asarray(part_offsets["mate_in_left_tool"])
        M_right = np.asarray(part_offsets["mate_in_right_tool"])
        T_socket = M_left @ _tz(INSERT_DEPTH / 2 - SOCKET_L / 2)
        T_plug = M_right @ _rx_pi() @ _tz(INSERT_DEPTH / 2 - PLUG_L / 2)
    if free_plug:
        spec.option.timestep = 0.00025

    left_tool = spec.body("left_link_6")
    right_tool = spec.body("right_link_6")
    if part_mode != "geometric" and part_offsets and "mate_in_left_tool" in part_offsets:
        # calibrate.py measures in the tcp_site frame (rotated 90 deg about z inside link_6); re-express in the body frame.
        # Geometric placement is defined in the body frame directly (pads close along body x).
        T_socket = _site_in_body(spec, "left_tcp_site") @ T_socket
        T_plug = _site_in_body(spec, "right_tcp_site") @ T_plug
    for arm, tool in (("left", left_tool), ("right", right_tool)):
        # fingers: i2rt stock linear_4310 tips (hollow-triangle nylon, rubber pads), already in the Menagerie model;
        # verified against the scene and wrist frames 2026-10-07. The cell's prints reach ~10 mm further: extend.
        if FINGER_EXT > 0:
            for side in ("left", "right"):
                fb = spec.body(f"{arm}_{side}_finger")
                fb.add_geom(name=f"{arm}_{side}_pad_ext", type=mujoco.mjtGeom.mjGEOM_BOX,
                            pos=[0, -0.0017, -(PAD_Z_STOCK + FINGER_EXT / 2)], size=[0.0072, 0.0017, FINGER_EXT / 2 + 0.004],
                            rgba=[0.12, 0.12, 0.12, 1], contype=0, conaffinity=0, density=1100)
        cam = spec.camera(f"{arm}_wrist_cam")
        cam.fovy = WRIST_CAM_FOVY
        cam.resolution = [640, 480]
        cam.focal_length = [0.0, 0.0]
        cam.focal_pixel = [0.0, 0.0]
        cam.sensor_size = [0.0, 0.0]
        # mount pose from the real wrist frames (2026-10-07): the fingertip pair holding the housing spans ~110 of
        # 640 px at a 65 deg HFOV and ~33 mm of real width -> camera ~170 mm from the tips, i.e. ~20 mm behind the
        # flange; tips at ~90 % frame height and the opposite gripper at ~30 % -> the camera looks straight along
        # the fingers from ~70 mm above the finger axis (the gripper back). Expressed in link_6: pos (0, +70, -20) mm,
        # looking along +z with image-up = +y (camera -z -> link +z, camera y -> link y: 180 deg about y).
        Tcb = _body_in_parent(spec, f"{arm}_wrist_camera")          # the camera element lives in the wrist_camera body
        # orientation: camera -z along link +z, image-up along link -y (the overlay with +y was upside down),
        # then pitched toward the fingers by WRIST_CAM_TILT_DEG so the fingertips sit low in the frame
        q_base = np.array([0.0, 1.0, 0.0, 0.0])           # 180 deg about x: -z_cam -> +z_link, y_cam -> -y_link
        t = np.radians(WRIST_CAM_TILT_DEG)
        q_tilt = np.array([np.cos(t / 2), np.sin(t / 2), 0.0, 0.0])
        rl = np.radians(WRIST_CAM_ROLL_DEG)
        q_roll = np.array([np.cos(rl / 2), 0.0, 0.0, np.sin(rl / 2)])   # about the optical axis
        q = np.empty(4); mujoco.mju_mulQuat(q, q_base, q_tilt); mujoco.mju_mulQuat(q, q, q_roll)
        T_cam_in_link = _pose_mat(list(WRIST_CAM_POS), [float(v) for v in q])
        if WRIST_CAM_T_IN_LINK.get(arm) is not None:          # registered pose (scripts/register_cameras.py)
            T_cam_in_link = np.asarray(WRIST_CAM_T_IN_LINK[arm], dtype=float)
        Tc = np.linalg.inv(Tcb) @ T_cam_in_link
        if WRIST_CAM_T_IN_LINK.get(arm) is not None or not WRIST_CAM_USE_CAD:
            cam.pos, cam.quat = _pose(Tc)              # registered pose wins; CAD only as the fallback
        else:
            # I2RT's CAD extrinsics from the vendored model (yam_station_linear_4310_d405): the wrist_camera body IS
            # the optical frame; keep its camera element and only apply the trim angles about it
            rl, t = np.radians(WRIST_CAM_ROLL_DEG), np.radians(WRIST_CAM_TILT_DEG)
            q_trim = np.empty(4); mujoco.mju_mulQuat(q_trim, np.array([np.cos(t / 2), np.sin(t / 2), 0, 0]), np.array([np.cos(rl / 2), 0, 0, np.sin(rl / 2)]))
            q0 = np.array(cam.quat, dtype=float); q1 = np.empty(4); mujoco.mju_mulQuat(q1, q0, q_trim); cam.quat = q1
        wcb = spec.body(f"{arm}_wrist_camera")
        if WRIST_CAM_T_IN_LINK.get(arm) is not None:
            # the cell's D405 sits on its own printed L bracket, not I2RT's: hide the vendored camera/bracket visuals
            # and draw the camera body (42 x 42 x 23 mm) directly behind the registered optical centre
            for g in wcb.geoms:
                g.group = 4                                   # not drawn by the renderer's default groups
            T_body = Tc @ _pose_mat([0.0, 0.0, 0.0135], [1, 0, 0, 0])   # camera looks along -z: body sits at +z
            bpos, bquat = _pose(T_body)
            wcb.add_geom(name=f"{arm}_d405_body", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.021, 0.021, 0.0115],
                         pos=bpos, quat=bquat, rgba=[0.12, 0.12, 0.12, 1], contype=0, conaffinity=0, mass=WRIST_CAM_MASS)
        else:
            wcb.add_geom(name=f"{arm}_d405_mass", type=mujoco.mjtGeom.mjGEOM_BOX,
                         size=[0.021, 0.012, 0.021], mass=WRIST_CAM_MASS,
                         rgba=[0.2, 0.2, 0.2, 1], contype=0, conaffinity=0)
    _add_socket(spec, left_tool, T_socket)
    _add_plug(spec, right_tool, T_plug, free_plug)
    return spec


# real controller (vendored i2rt robots/config/yam_v1.yml on the NUC): MIT-mode position loop
# kp 80/80/80/10/10/10 Nm/rad, kd 5/5/5/1.5/1.5/1.5, with gravity compensated by feedforward
# (factors 1.0/1.1/1.1/1.2/1.0/1.0). The Menagerie model's gravcomp bodies are that feedforward;
# the 10-20 % over-compensation on joints 2-4 is not modelled (it leaves ~12 mrad of residual error on j2).
GAINS = {"joint1": (80.0, 5.0), "joint2": (80.0, 5.0), "joint3": (80.0, 5.0),
         "joint4": (10.0, 1.5), "joint5": (10.0, 1.5), "joint6": (10.0, 1.5)}


def _apply_real_gains(child: mujoco.MjSpec) -> None:
    for b in child.bodies:
        b.gravcomp = 1.0 if b.gravcomp > 0 else 0.0     # keep the feedforward where the model has it
    for act in child.actuators:
        if act.name in GAINS:
            kp, kv = GAINS[act.name]
            act.gainprm[0] = kp
            act.biasprm[1] = -kp
            act.biasprm[2] = -kv


WRIST_CAM_FOVY = 58.0       # D405: centre-fit keeps the full 58 deg VFOV (horizontal cropped to 4:3)
WRIST_CAM_POS = (0.0, 0.070, -0.020)  # in link_6: on the gripper back, 20 mm behind the flange (from the real frames)
WRIST_CAM_TILT_DEG = -15.0            # pitch toward the finger axis; chosen from the overlays at -12 and -25
WRIST_CAM_ROLL_DEG = 0.0              # rotation about the optical axis (180 = module mounted inverted)
def _registered_wrist_T():
    """D405 mount in link_6 from scripts/register_cell.py (2026-10-07): on the L bracket above the flat base,
    y 96 mm, z 64 mm, pitched 31.5 deg down toward the fingertips; same bracket on both arms."""
    p = ASSETS.parent.parent / "out" / "register_cell.json"
    if p.exists():
        _r = json.load(open(p)); T = np.asarray(_r["T_cam_in_link6"], dtype=float)
        global WRIST_CAM_FOVY
        WRIST_CAM_FOVY = float(_r.get("fovy", WRIST_CAM_FOVY))
        return {"left": T, "right": T}
    return {"left": None, "right": None}


WRIST_CAM_T_IN_LINK = _registered_wrist_T()   # 4x4 camera pose in link_6 (MuJoCo camera: looks along -z, +y up); wins when set
WRIST_CAM_USE_CAD = True              # place the camera at I2RT's CAD extrinsics (vendored model) instead of WRIST_CAM_POS/TILT


def _wrist_cam_quat(tilt_deg):
    # MuJoCo cameras look along their -z with +y up. Start looking along link_6 +z (fingers) with
    # image-up = -y of link_6 (so the gripper back is at the bottom of the frame), then pitch down.
    import numpy as _np
    base = _np.array([0.0, 1.0, 0.0, 0.0])   # 180 deg about x: -z_cam -> +z_link, +y_cam -> -y_link
    t = _np.radians(tilt_deg)
    tilt = _np.array([_np.cos(t / 2), _np.sin(t / 2), 0.0, 0.0])
    q = _np.empty(4)
    mujoco.mju_mulQuat(q, base, tilt)
    return [float(x) for x in q]
WRIST_CAM_MASS = 0.15       # D405 (60 g) + printed mount + cable, placeholder

WRIST_CAM_POS = (0.0, -0.055, 0.045)     # tool frame: behind the finger axis, on the gripper back; placeholder
WRIST_CAM_TILT_DEG = -15.0            # pitch toward the finger axis; chosen from the overlays at -12 and -25
WRIST_CAM_ROLL_DEG = 0.0              # rotation about the optical axis (180 = module mounted inverted)
def _registered_wrist_T():
    """D405 mount in link_6 from scripts/register_cell.py (2026-10-07): on the L bracket above the flat base,
    y 96 mm, z 64 mm, pitched 31.5 deg down toward the fingertips; same bracket on both arms."""
    p = ASSETS.parent.parent / "out" / "register_cell.json"
    if p.exists():
        _r = json.load(open(p)); T = np.asarray(_r["T_cam_in_link6"], dtype=float)
        global WRIST_CAM_FOVY
        WRIST_CAM_FOVY = float(_r.get("fovy", WRIST_CAM_FOVY))
        return {"left": T, "right": T}
    return {"left": None, "right": None}


WRIST_CAM_T_IN_LINK = _registered_wrist_T()   # 4x4 camera pose in link_6 (MuJoCo camera: looks along -z, +y up); wins when set


def _add_wrist_camera(arm, tool):
    t = np.radians(WRIST_CAM_TILT_DEG)
    # camera looks along its -z; rotate so -z points along tool +z, then tilt about x toward the axis
    q_base = np.array([0.0, 1.0, 0.0, 0.0])            # 180 deg about x: camera -z -> tool +z, camera y -> tool -y
    q_tilt = np.array([np.cos(t / 2), np.sin(t / 2), 0.0, 0.0])
    q = np.empty(4)
    mujoco.mju_mulQuat(q, q_tilt, q_base)
    tool.add_camera(name=f"{arm}_d405", pos=list(WRIST_CAM_POS), quat=[float(x) for x in q], fovy=58.0,
                    resolution=[640, 480])


def _pose_mat(pos, quat) -> np.ndarray:
    R = np.empty(9)
    mujoco.mju_quat2Mat(R, np.asarray(quat, dtype=float))
    T = np.eye(4)
    T[:3, :3] = R.reshape(3, 3)
    T[:3, 3] = pos
    return T


def _body_in_parent(spec, body_name) -> np.ndarray:
    b = spec.body(body_name)
    q = np.asarray(b.quat, dtype=float)
    q = q / np.linalg.norm(q)
    return _pose_mat(list(b.pos), q)


def _site_in_body_from_file(site_name) -> np.ndarray:
    """Same as _site_in_body but from the arm file alone (no attached spec needed); site names carry the arm prefix."""
    spec = mujoco.MjSpec.from_file(str(ASSETS / "yam_linear_4310.xml"))
    return _site_in_body(spec, site_name.split("_", 1)[1])


def _site_in_body(spec, site_name) -> np.ndarray:
    site = spec.site(site_name)
    q = np.asarray(site.quat, dtype=float)
    q = q / np.linalg.norm(q)
    R = np.empty(9)
    mujoco.mju_quat2Mat(R, q)
    T = np.eye(4)
    T[:3, :3] = R.reshape(3, 3)
    T[:3, 3] = site.pos
    return T


def _tz(z: float) -> np.ndarray:
    T = np.eye(4)
    T[2, 3] = z
    return T


def _rz_90() -> np.ndarray:
    T = np.eye(4)
    T[0, 0], T[0, 1], T[1, 0], T[1, 1] = 0.0, -1.0, 1.0, 0.0
    return T


def _rx_pi() -> np.ndarray:
    T = np.eye(4)
    T[1, 1] = T[2, 2] = -1.0
    return T


def load_part_offsets(path=None) -> dict | None:
    import json
    p = Path(path) if path else Path(__file__).resolve().parent.parent / "out" / "part_offsets.json"
    return json.load(open(p)) if p.exists() else None


def _on_axis(reach: float) -> np.ndarray:
    T = np.eye(4)
    T[2, 3] = reach
    return T


def _pose(T: np.ndarray) -> tuple[list[float], list[float]]:
    q = np.empty(4)
    mujoco.mju_mat2Quat(q, T[:3, :3].reshape(-1))
    return [float(x) for x in T[:3, 3]], [float(x) for x in q]


PART_CONTYPE = 2            # parts collide with each other only, never with the arms
PLUG_IN_TOOL: dict = {}     # filled by build(): plug pose in the right tool frame (free-plug mode)
GRASP_SOLREF = (0.001, 1.0)   # grasp weld (0.002 let the plug creep 0.23 mm along the push axis under the seat load; Shiven: the grip is rigid); with PLUG_MASS 0.02 kg this is ~5 N/mm. At 1 N/mm the plug swung 13 mm and 36 deg on first contact.
PLUG_MASS = 0.02              # header + wire tail stand-in; raised from 2 g so the grasp spring has a usable time constant (placeholder)
CONTACT_SOLREF = [0.001, 1.0]
CONTACT_SOLIMP = [0.99, 0.999, 0.0002, 0.5, 2.0]
PART_FRICTION = float(os.environ.get('TWIN_FRICTION', '0.4'))   # plastic on plastic, placeholder


def _add_socket(spec, tool, T):
    """Female housing: a block whose mating face (+z of its frame) carries N_PINS square cavities."""
    pos, quat = _pose(T)
    b = tool.add_body(name="socket", pos=pos, quat=quat)
    hl = HOUSING_L / 2
    ht = HOUSING_T / 2
    hh = HOUSING_H / 2
    face = hh                                   # mating face at +hh
    cav_bottom = face - CAVITY_DEPTH
    dark = [0.08, 0.08, 0.08, 1]
    kw = dict(type=mujoco.mjtGeom.mjGEOM_BOX, rgba=dark, contype=PART_CONTYPE, conaffinity=PART_CONTYPE,
              solref=CONTACT_SOLREF, solimp=CONTACT_SOLIMP)
    # solid lower part of the housing (below the cavities)
    b.add_geom(name="housing_base", pos=[0, 0, (-hh + cav_bottom) / 2], size=[hl, ht, (cav_bottom + hh) / 2], **kw)
    wall_top = face - CAVITY_CHAMFER            # walls stop below the chamfer so the entry is a real funnel
    cz = (wall_top + cav_bottom) / 2
    chz = (wall_top - cav_bottom) / 2
    c = CAVITY_SQ / 2
    # two long side walls (across the thin dimension)
    b.add_geom(name="housing_side_py", pos=[0, c + (ht - c) / 2, cz], size=[hl, (ht - c) / 2, chz], **kw)
    b.add_geom(name="housing_side_ny", pos=[0, -c - (ht - c) / 2, cz], size=[hl, (ht - c) / 2, chz], **kw)
    # cross walls between and outside the cavities
    x0 = -(N_PINS - 1) * PITCH / 2
    edges = [x0 + i * PITCH + c for i in range(N_PINS - 1)]          # left edge of each inter-cavity wall
    for i, xl in enumerate(edges):
        w = PITCH - CAVITY_SQ
        b.add_geom(name=f"housing_cross_{i}", pos=[xl + w / 2, 0, cz], size=[w / 2, c, chz], **kw)
    for name, sign in (("end_px", 1), ("end_nx", -1)):
        inner = sign * (abs(x0) + c)                                # outer edge of the end cavity
        w = hl - abs(inner)
        b.add_geom(name=f"housing_{name}", pos=[inner + sign * w / 2, 0, cz], size=[w / 2, c, chz], **kw)
    # entry chamfer: four tilted plates per cavity, outer face flush with the mating face
    a = np.radians(45.0)
    L = CAVITY_CHAMFER / np.cos(a)
    th = 0.0003
    for i in range(N_PINS):
        cx = x0 + i * PITCH
        for nm, axis, sign in (("px", 0, 1), ("nx", 0, -1), ("py", 1, 1), ("ny", 1, -1)):
            # plate direction t = (sin a, cos a) in the (lateral, z) plane, outward normal n = (cos a, -sin a);
            # the plate's inner face must pass through the cavity edge at the wall top (c, face - CHAMFER)
            ctr = [cx, 0.0, face - CAVITY_CHAMFER - th / 2 * np.sin(a) + L / 2 * np.cos(a)]
            ctr[axis] += sign * (c + th / 2 * np.cos(a) + L / 2 * np.sin(a))
            rot = [np.cos(a / 2), 0, 0, 0]
            rot[1 + (1 - axis)] = sign * np.sin(a / 2) if axis == 0 else -sign * np.sin(a / 2)
            size = [th / 2, th / 2, L / 2]
            size[1 - axis] = c + CAVITY_CHAMFER
            b.add_geom(name=f"housing_ch_{i}_{nm}", type=mujoco.mjtGeom.mjGEOM_BOX, pos=ctr, quat=rot, size=size,
                       rgba=dark, contype=PART_CONTYPE, conaffinity=PART_CONTYPE, solref=CONTACT_SOLREF, solimp=CONTACT_SOLIMP)
    # female contact: the real sprung leaves grip each pin with ~0.5-1 N a side. Modelled per cavity as two
    # static plates on each y face: an entry ramp from the chamfer bottom (clear of the pin) down to the grip
    # line, then a straight gripping plate at a slight interference to the floor. The soft contact is the
    # leaf's spring; friction on it is the insertion drag. No step anywhere for the pin's shoulder to catch.
    z_top = face - CAVITY_CHAMFER
    z_grip = face - CONTACT_START - 0.0015                                # ramp ends 3 mm below the face
    y_top = c + 0.00002
    y_grip = PIN_SQ / 2 - CONTACT_INTERFERENCE
    lkw = dict(type=mujoco.mjtGeom.mjGEOM_BOX, rgba=[0.75, 0.65, 0.3, 1], contype=PART_CONTYPE, conaffinity=PART_CONTYPE,
               solref=LEAF_SOLREF, solimp=LEAF_SOLIMP, friction=[PART_FRICTION, 0.005, 0.0001])
    for i in range(N_PINS):
        cx = x0 + i * PITCH
        for nm, sign in (("py", 1), ("ny", -1)):
            # ramp segment: inner face from (y_top, z_top) to (y_grip, z_grip)
            L = np.hypot(z_top - z_grip, y_top - y_grip); a = np.arctan2(y_top - y_grip, z_top - z_grip)
            yc = sign * ((y_top + y_grip) / 2 + (LEAF_T / 2) * np.cos(a)); zc = (z_top + z_grip) / 2 - (LEAF_T / 2) * np.sin(a)
            b.add_geom(name=f"housing_ramp_{i}_{nm}", pos=[cx, yc, zc], quat=[np.cos(a / 2), -sign * np.sin(a / 2), 0, 0],
                       size=[c * 0.9, LEAF_T / 2, L / 2], **lkw)
            # straight gripping segment: inner face at y_grip from z_grip down to the floor
            zb = cav_bottom
            b.add_geom(name=f"housing_leaf_{i}_{nm}", pos=[cx, sign * (y_grip + LEAF_T / 2), (z_grip + zb) / 2],
                       size=[c * 0.9, LEAF_T / 2, (z_grip - zb) / 2], **lkw)
    b.add_site(name="socket_mouth", pos=[0, 0, face], size=[0.0015], rgba=[0, 1, 0, 0.5], group=4)
    return b


def _add_plug(spec, tool, T, free: bool):
    if not free:
        pos, quat = _pose(T)
        b = tool.add_body(name="plug", pos=pos, quat=quat)
    else:
        # world body at the tool offset evaluated at the arm's zero pose; the weld's relative pose
        # is then auto-computed from qpos0 and equals T. replay.py places it at the episode start.
        tool_world = _body_world_pose_at_zero(spec, tool.name)
        pos, quat = _pose(tool_world @ T)
        b = spec.worldbody.add_body(name="plug", pos=pos, quat=quat)
        b.add_freejoint(name="plug_free")
        eq = spec.add_equality(type=mujoco.mjtEq.mjEQ_WELD, name="plug_grasp",
                               objtype=mujoco.mjtObj.mjOBJ_BODY, name1=tool.name, name2="plug")
        eq.solref[0], eq.solref[1] = GRASP_SOLREF
        # weld data: anchor (3), relpose (pos 3 + quat 4) = pose of body2 (plug) in body1 (tool) frame.
        # Verified dynamically: this order holds the plug; the reverse order throws it (2026-10-06).
        q = np.empty(4)
        mujoco.mju_mat2Quat(q, T[:3, :3].reshape(-1))
        data = np.zeros(11)
        data[3:6] = T[:3, 3]
        data[6:10] = q
        data[10] = 1.0       # torque scale: hold rotation as firmly as translation (fingers pinch the housing)
        eq.data = data
        PLUG_IN_TOOL["right_link_6"] = T.copy()
    # frame: +z toward the female; plug housing at the back, pins in front. Body origin = centre of (housing + pins).
    half = PLUG_L / 2
    ins_c = -half + HEADER_H / 2
    kw = dict(contype=PART_CONTYPE, conaffinity=PART_CONTYPE, condim=3, friction=[PART_FRICTION, 0.005, 0.0001],
              solref=CONTACT_SOLREF, solimp=CONTACT_SOLIMP)
    b.add_geom(name="plug_body", type=mujoco.mjtGeom.mjGEOM_BOX, pos=[0, 0, ins_c],
               size=[HEADER_L / 2, HEADER_T / 2, HEADER_H / 2], rgba=[0.08, 0.08, 0.1, 1], mass=PLUG_MASS, **kw)
    # label stripe on the flat face (visual only), as on the real plug
    b.add_geom(name="plug_label", type=mujoco.mjtGeom.mjGEOM_BOX, pos=[0, HEADER_T / 2 + 0.00005, ins_c],
               size=[HEADER_L / 2 * 0.8, 0.00005, HEADER_H / 2 * 0.6], rgba=[0.2, 0.3, 0.9, 1], contype=0, conaffinity=0, mass=0)
    x0 = -(N_PINS - 1) * PITCH / 2
    pin_z0 = -half + HEADER_H
    body_len = PIN_LEN - PIN_TIP_CHAMFER
    for i in range(N_PINS):
        cx = x0 + i * PITCH
        b.add_geom(name=f"pin_{i}", type=mujoco.mjtGeom.mjGEOM_BOX, pos=[cx, 0, pin_z0 + body_len / 2],
                   size=[PIN_SQ / 2, PIN_SQ / 2, body_len / 2], rgba=[0.85, 0.75, 0.3, 1], mass=0.0001, **kw)
        b.add_geom(name=f"pin_tip_{i}", type=mujoco.mjtGeom.mjGEOM_BOX, pos=[cx, 0, pin_z0 + body_len + PIN_TIP_CHAMFER / 2],
                   size=[PIN_SQ / 2 * 0.7, PIN_SQ / 2 * 0.7, PIN_TIP_CHAMFER / 2], rgba=[0.85, 0.75, 0.3, 1], mass=0.00001, **kw)
    b.add_site(name="plug_tip", pos=[0, 0, half], size=[0.0015], rgba=[1, 0, 0, 0.5], group=4)
    return b


def _body_world_pose_at_zero(spec, body_name):
    m = spec.copy().compile()
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    T = np.eye(4)
    T[:3, :3] = d.body(body_name).xmat.reshape(3, 3)
    T[:3, 3] = d.body(body_name).xpos
    return T


def compile_model(**kw) -> tuple[mujoco.MjSpec, mujoco.MjModel]:
    spec = build(**kw)
    return spec, spec.compile()
