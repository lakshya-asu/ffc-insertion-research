"""FFC into a flip-latch ZIF: a modular bench you can build in an afternoon, as a MuJoCo scene.

Three parts, each a thing you can bolt to a tapped breadboard:
  bench   the breadboard, the arm on its plate, a post for a fixed overview camera
  pallet  a 300 x 300 mm plate on three ball-and-groove couplings with a ChArUco fiducial in one corner; everything the
          task touches sits on it: the laptop upside down in printed corner stops, bottom cover off, its connector,
          its keyboard cable. Lift the pallet off and drop another on (a coupon with a bare connector, a different laptop)
          and the couplings put it back within tens of micrometres. The policy sees the task in the pallet frame
          (the fiducial), so the same pallet on another arm or another bench needs calibration, not retraining.
  tool    the gripper stand-in on the arm's flange: wide flat jaws that close vertically, a thin lower shoe so the cable
          can lie at board height, a latch blade ahead of the jaws, a force site, the D405 and two macro cameras.

Dimensions marked ESTIMATE are read off the owner's photo (scaled against the SO-DIMM slot) or taken from the Hirose
FH12 datasheet class; each is replaced by a caliper reading. Frames: world z up, bench top at z = 0. In the pallet frame
the connector mouth faces +y, the cable comes from the keyboard slot at +y and slides in along -y, the latch hinge is at
the rear (-y) edge and the open lid stands up leaning back. The arm stands on the +y side and reaches over the cable.
"""
from __future__ import annotations

import math
import os
from pathlib import Path

import mujoco
import numpy as np

ASSETS = Path(__file__).resolve().parent.parent / "assets" / "robots"

# ---------------------------------------------------------------- connector and cable (metres) ---------------------------
PITCH = float(os.environ.get("ZIF_PITCH_MM", "0.8")) * 1e-3
N_CONTACTS = int(os.environ.get("ZIF_CONTACTS", "32"))
CABLE_W = float(os.environ.get("FFC_WIDTH_MM", "28.0")) * 1e-3             # photo: stiffener 27.7 mm against the DIMM slot  ESTIMATE
HOUSING_L = PITCH * (N_CONTACTS + 1) + 0.0050                              # Molex rule W = pitch x (N+1), plus end walls: 31.4 mm
HOUSING_D = 0.0060                                                         # depth along y                                   ESTIMATE
HOUSING_H = 0.0020                                                         # closed height above the board (FH12: 2.0 mm)
SLOT_FLOOR = 0.0006                                                        # cable floor above the board                     ESTIMATE
SLOT_GAP = 0.00040                                                         # slot height over the floor for a 0.30 mm cable; the contact springs stand 0.12 mm proud, so an inserted cable is lightly pinched and stays put with the latch open (Shiven, by hand)   ESTIMATE
INSERT_DEPTH = 0.0035                                                      # mouth to rear stop (FH12 class)                 ESTIMATE
ROOF_D = 0.0015                                                            # fixed roof over the rear of the slot            ESTIMATE
LID_D = HOUSING_D - ROOF_D - 0.0002
LATCH_OPEN_DEG = 100.0
LATCH_FRICTION = 0.0175                                                    # N m, 7 N at the lid tip (FH12: 6.9 N to close)
LATCH_CLAMP_GAP = 0.00027                                                  # lid underside above the floor when closed: 0.03 mm less than the cable, the contact springs take the difference
LID_PROUD = 0.0002                                                         # closed lid top stands this much above the housing (real actuators sit flush or proud)
LID_T = HOUSING_H + LID_PROUD - SLOT_FLOOR - LATCH_CLAMP_GAP               # 1.35 mm thick actuator; the hinge axis is 0.3 mm below its top so the rear corner clears the roof
LID_AXIS_BELOW_TOP = 0.0003
STIFF_T, STIFF_L = 0.00030, 0.0130                                         # stiffener thickness and length                  ESTIMATE
BODY_T, SEG_L, N_SEG = 0.00014, 0.0050, 5                                  # flexible body: five 5 mm segments. 25 + 13 mm stiffener = 38 mm; the deck anchor is 31.8 mm from the seat, so 6 mm of slack   ESTIMATE
E_MOD = 3.0e9                                                              # FFC repo dossier
CABLE_REST = [math.radians(a) for a in (-60, 0, 0, 55, 0)]   # bend memory: out of the deck slot at 60 deg, then level; the stiffener floats level at about 11.7 mm with its tip over the mouth (photos)                  # bend set from the photo: the unplugged cable rises out of the deck slot at about 40 deg, then levels off toward the connector   ESTIMATE
CABLE_MEMORY = [1.0, 1.0, 1.0, 1.0, 1.0]                                    # how much of that shape the springs hold (1 = full bend memory, the worst case for pull-out)
BEND_K = E_MOD * CABLE_W * BODY_T ** 3 / 12.0 / SEG_L                      # 3.2e-3 N m per rad per joint

# ---------------------------------------------------------------- laptop on the pallet --------------------------------------
LAPTOP_W, LAPTOP_D, LAPTOP_H = 0.360, 0.250, 0.020                         # 15.6 inch class                                 ESTIMATE
DECK_T = 0.0030
BOARD_Z = 0.0030                                                           # mainboard top above the deck surface (photo: deck plastic nearly flush with the board)   ESTIMATE
BOARD_T = 0.0016
WALL_T = 0.0025
CONN_IN_LAPTOP = (0.030, 0.015)                                            # connector centre from the laptop centre         ESTIMATE
RAIL_Y_FROM_CONN = 0.040                                                   # steel deck bracket beyond the cable slot (photo), 4 mm tall; nothing stands between the connector and the slot   ESTIMATE
DIMM_Y_FROM_CONN = -0.025                                                  # SO-DIMM slot 25 mm behind the connector
CABLE_EXIT_Z = -0.0015                                                     # deck surface 1.5 mm below the board top (board on standoffs)   ESTIMATE
CABLE_EXIT_Y_FROM_CONN = 0.031                                             # deck slot 31 mm behind the mouth (photo: 470 px at 15.2 px/mm against the 69.6 mm SO-DIMM)   ESTIMATE
FEED_Y = 0.040                                                              # platform case: the feed clip this far behind the mouth; a straight 25 mm body puts the stiffener root 17 mm back, clear of the rest
PRESENTER_Y, PRESENTER_Z, PRESENTER_L = 0.022, 0.0120, 0.002                # printed rest centred 20 mm behind the mouth, 8.7 mm tall, 2 mm long: the cable body lies over it so the stiffener floats level above the latch swing (6.1 mm) and above the standing lid for the grasp; the cable lifts off it freely during the slide (design)

# ---------------------------------------------------------------- pallet and bench ---------------------------------------
PALLET = (0.300, 0.300, 0.0127)                                            # Thorlabs MB12 class plate on three couplings
LAPTOP_ON_PALLET = (0.0, 0.0)                                              # laptop centre on the pallet
FIDUCIAL = (-0.125, 0.125, 0.060)                                          # ChArUco corner of the pallet, 60 mm board
BREADBOARD = (0.914, 0.610, 0.0127)
ARM_BASE = {"meca500": (0.0, 0.26, 0.0127), "lrmate200id": (0.0, 0.48, 0.0127)}   # arm on the +y side of the pallet
if os.environ.get("FFC_LRMATE_BASE_Y"):
    ARM_BASE["lrmate200id"] = (0.0, float(os.environ["FFC_LRMATE_BASE_Y"]), 0.0127)   # experiment hook: where the LR Mate stands behind the pallet
PLATE = {"meca500": (0.120, 0.100, 0.0127), "lrmate200id": (0.300, 0.300, 0.019)}

PART, TOOL = 2, 4
CONTACT_SOLREF = [0.001, 1.0]
CONTACT_SOLIMP = [0.99, 0.999, 0.0002, 0.5, 2.0]
FRICTION = [0.4, 0.005, 0.0001]
DT = 0.0001                                                                # 0.1 ms: lets the grasp weld be stiff (solref 0.2 ms) without instability


def _kw(rgba, group=0, contype=PART, conaffinity=PART):
    return dict(type=mujoco.mjtGeom.mjGEOM_BOX, rgba=rgba, contype=contype, conaffinity=conaffinity, group=group,
                solref=CONTACT_SOLREF, solimp=CONTACT_SOLIMP, friction=FRICTION)


def _look_quat(pos, target, up=(0, 0, 1)):
    d = np.array(target, float) - np.array(pos, float); d /= np.linalg.norm(d)
    z = -d; x = np.cross(up, z); x /= np.linalg.norm(x); y = np.cross(z, x)
    R = np.column_stack([x, y, z]); q = np.empty(4); mujoco.mju_mat2Quat(q, R.flatten()); return q.tolist()


def pallet_pose(arm):
    """Pallet centre on the bench. The connector ends up in front of the arm at a comfortable reach."""
    bx, by, bz = ARM_BASE[arm]
    reach = {"meca500": 0.19, "lrmate200id": float(os.environ.get("FFC_LRMATE_REACH", 0.34))}[arm]
    cy = by - reach - CONN_IN_LAPTOP[1] - LAPTOP_ON_PALLET[1]
    return (bx - CONN_IN_LAPTOP[0] - LAPTOP_ON_PALLET[0], cy, BREADBOARD[2])


PLATFORM = (0.120, 0.080, 0.012)                                           # aluminium plate carrying the reference connector in the platform case


def connector_in_pallet():
    if _CASE[0] == "platform":
        return np.array([LAPTOP_ON_PALLET[0] + CONN_IN_LAPTOP[0], LAPTOP_ON_PALLET[1] + CONN_IN_LAPTOP[1], PALLET[2] + PLATFORM[2]])
    return np.array([LAPTOP_ON_PALLET[0] + CONN_IN_LAPTOP[0], LAPTOP_ON_PALLET[1] + CONN_IN_LAPTOP[1], PALLET[2] + DECK_T + BOARD_Z])


def connector_origin(arm):
    return np.array(pallet_pose(arm)) + connector_in_pallet()


# ---------------------------------------------------------------- bench -------------------------------------------------------
def add_bench(spec, arm):
    w = spec.worldbody
    w.add_light(pos=[0.3, -0.6, 1.2], dir=[-0.2, 0.4, -1.0], castshadow=True)
    w.add_light(pos=[-0.6, 0.6, 1.0], dir=[0.4, -0.4, -1.0], castshadow=False)
    w.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[3, 3, 0.1], rgba=[0.90, 0.88, 0.84, 1], pos=[0, 0, -0.75])
    w.add_geom(name="breadboard", pos=[0, 0.05, BREADBOARD[2] / 2], size=[BREADBOARD[1] / 2, BREADBOARD[0] / 2, BREADBOARD[2] / 2], **_kw([0.22, 0.22, 0.24, 1], contype=1, conaffinity=1))
    px, py, pz = PLATE[arm]; bx, by, bz = ARM_BASE[arm]
    w.add_geom(name="arm_plate", pos=[bx, by, bz + pz / 2], size=[px / 2, py / 2, pz / 2], **_kw([0.72, 0.73, 0.75, 1], contype=0, conaffinity=0))
    # overview camera on a post behind the pallet, opposite the arm
    o = connector_origin(arm); post = np.array([o[0] + 0.18, o[1] - 0.16, BREADBOARD[2]])
    w.add_geom(name="cam_post", type=mujoco.mjtGeom.mjGEOM_CYLINDER, pos=[post[0], post[1], post[2] + 0.17], size=[0.0063, 0.17], rgba=[0.75, 0.75, 0.77, 1], contype=0, conaffinity=0)
    cpos = post + np.array([0, 0, 0.34])
    w.add_camera(name="overview", pos=cpos.tolist(), quat=_look_quat(cpos, o), fovy=40)
    bpos = [o[0] - 0.45, o[1] - 0.40, 0.38]
    w.add_camera(name="bench", pos=bpos, quat=_look_quat(bpos, o), fovy=42)


# ---------------------------------------------------------------- pallet ------------------------------------------------------
def add_pallet(spec, arm):
    """The hot-swappable object module: plate, three couplings, fiducial, printed corner stops, the laptop with its connector and cable."""
    px, py, pz = pallet_pose(arm)
    pal = spec.worldbody.add_body(name="pallet", pos=[px, py, pz])
    pal.add_geom(name="pallet_plate", pos=[0, 0, PALLET[2] / 2], size=[PALLET[0] / 2, PALLET[1] / 2, PALLET[2] / 2], **_kw([0.30, 0.30, 0.32, 1], contype=1, conaffinity=1))
    for i, (cx, cy) in enumerate(((-0.12, -0.12), (0.12, -0.12), (0.0, 0.12))):     # three ball and groove couplings under the plate
        pal.add_geom(name=f"coupling{i}", type=mujoco.mjtGeom.mjGEOM_CYLINDER, pos=[cx, cy, -0.004], size=[0.012, 0.004], rgba=[0.6, 0.6, 0.62, 1], contype=0, conaffinity=0)
    fx, fy, fs = FIDUCIAL
    pal.add_geom(name="fiducial", pos=[fx + fs / 2, fy - fs / 2, PALLET[2] + 0.0015], size=[fs / 2, fs / 2, 0.0015], **_kw([0.95, 0.95, 0.95, 1], contype=0, conaffinity=0))
    for k in range(6):
        for m in range(6):
            if (k + m) % 2 == 0:
                pal.add_geom(name=f"fid_{k}{m}", pos=[fx + fs / 12 + k * fs / 6, fy - fs / 12 - m * fs / 6, PALLET[2] + 0.0031], size=[fs / 12, fs / 12, 0.0001], **_kw([0.05, 0.05, 0.05, 1], group=1, contype=0, conaffinity=0))
    pal.add_site(name="task_frame", pos=[fx, fy, PALLET[2] + 0.003], size=[0.002] * 3, rgba=[1, 0, 1, 0.5])
    lx, ly = LAPTOP_ON_PALLET; lz = PALLET[2]
    for sx in (-1, 1):   # printed corner stops
        pal.add_geom(name=f"stop_{'e' if sx > 0 else 'w'}", pos=[lx + sx * (LAPTOP_W / 2 + 0.004), ly - LAPTOP_D / 2 + 0.03, lz + 0.006], size=[0.004, 0.03, 0.006], **_kw([0.93, 0.55, 0.20, 1], contype=0, conaffinity=0))
        pal.add_geom(name=f"stop_{'e' if sx > 0 else 'w'}_s", pos=[lx + sx * (LAPTOP_W / 2 - 0.03), ly - LAPTOP_D / 2 - 0.004, lz + 0.006], size=[0.03, 0.004, 0.006], **_kw([0.93, 0.55, 0.20, 1], contype=0, conaffinity=0))
    o = connector_in_pallet()
    if _CASE[0] == "platform":
        pal.add_geom(name="platform_plate", pos=[o[0] - 0.010, o[1] - 0.010, lz + PLATFORM[2] / 2], size=[PLATFORM[0] / 2, PLATFORM[1] / 2, PLATFORM[2] / 2], **_kw([0.80, 0.81, 0.83, 1], contype=1, conaffinity=1))
    else:
        add_laptop(pal, (lx, ly, lz))
    add_connector(pal, o)
    CMl = CAMERAS.get(_ARM[0], {})
    if CMl and not FIXED_CAMS and CMl["light_size"][0] > 0.060:               # fixture bar light on the pallet, aimed at the mouth from the west
        lw, lh, ldp = CMl["light_size"]; lp = np.array([o[0] - 0.060, o[1] + 0.010, o[2] + 0.030]); lq = _look_quat(lp, o + np.array([0, 0, 0.001]), up=(0, 0, 1))
        pal.add_geom(name="light_g", pos=lp.tolist(), quat=lq, size=[lw / 2, lh / 2, ldp / 2], mass=CMl["light_mass"], **_kw([0.10, 0.35, 0.85, 1], contype=0, conaffinity=0))
        pal.add_geom(name="light_post", pos=[lp[0], lp[1], (o[2] + lp[2] - ldp / 2) / 2], size=[0.005, 0.005, (lp[2] - ldp / 2 - o[2]) / 2], mass=0.02, **_kw(ALU, contype=0, conaffinity=0))
    # platform case only: a printed rest (rounded bar, 5 mm radius, top at PRESENTER_Z) behind the stiffener root. The cable body lies over it from
    # the feed, the stiffener cantilevers level beyond it, and the feed pays the body out over the bar as the arm carries the stiffener down.
    # The laptop has no room for one: the 25 mm body from the deck slot lies nearly flat once the cable is seated (3.5 mm of slack), so there
    # the cable's own bend memory presents the stiffener, as in the photos.
    if _CASE[0] == "platform":
      rk = _kw([0.93, 0.55, 0.20, 1], contype=PART | 1, conaffinity=PART | 1); rk["type"] = mujoco.mjtGeom.mjGEOM_CYLINDER; rk["solref"] = [0.01, 1.0]; rk["friction"] = [0.2, 0.005, 0.0001]
      pal.add_geom(name="presenter_ledge", pos=[o[0], o[1] + PRESENTER_Y, o[2] + PRESENTER_Z - 0.005], quat=[math.cos(math.pi / 4), 0, math.sin(math.pi / 4), 0], size=[0.005, CABLE_W / 2 - 0.001, 0], **rk)   # 5 mm radius so the cable slides over it as the arm pulls; axis along x, 1 mm narrower than the cable each side so the finger legs pass
      pal.add_geom(name="presenter_post", pos=[o[0], o[1] + PRESENTER_Y, o[2] + (PRESENTER_Z - 0.005) / 2], size=[CABLE_W / 2 - 0.001, 0.0015, (PRESENTER_Z - 0.005) / 2], **_kw([0.93, 0.55, 0.20, 1], contype=0, conaffinity=0))
    # no keeper: with about 3.6 mm of slack in the seated cable (38 mm cable, slot 31 mm back) a clip would make the slide go taut.
    # Spring-back after release is therefore measured, not prevented, in the twin: see "after release" in the demo output.
    add_cable(pal, o)
    return pal


def add_laptop(parent, at):
    lx, ly, lz = at
    lap = parent.add_body(name="laptop", pos=[lx, ly, lz])
    dark = [0.10, 0.10, 0.11, 1]; board = [0.08, 0.25, 0.45, 1]; alu = [0.70, 0.71, 0.72, 1]
    lap.add_geom(name="deck", pos=[0, 0, DECK_T / 2], size=[LAPTOP_W / 2, LAPTOP_D / 2, DECK_T / 2], **_kw(dark, contype=1, conaffinity=1))
    for nm, sx, sy, px, py in (("wall_n", LAPTOP_W / 2, WALL_T / 2, 0, LAPTOP_D / 2 - WALL_T / 2), ("wall_s", LAPTOP_W / 2, WALL_T / 2, 0, -LAPTOP_D / 2 + WALL_T / 2),
                               ("wall_e", WALL_T / 2, LAPTOP_D / 2, LAPTOP_W / 2 - WALL_T / 2, 0), ("wall_w", WALL_T / 2, LAPTOP_D / 2, -LAPTOP_W / 2 + WALL_T / 2, 0)):
        lap.add_geom(name=nm, pos=[px, py, LAPTOP_H / 2], size=[sx, sy, LAPTOP_H / 2], **_kw(dark, contype=1, conaffinity=1))
    cx, cy = CONN_IN_LAPTOP
    lap.add_geom(name="mainboard", pos=[0.02, cy - 0.070, DECK_T + BOARD_Z - BOARD_T / 2], size=[0.15, 0.080, BOARD_T / 2], **_kw(board, contype=1, conaffinity=1))
    lap.add_geom(name="dimm_slot", pos=[cx, cy + DIMM_Y_FROM_CONN - 0.0045, DECK_T + BOARD_Z + 0.002], size=[0.036, 0.0045, 0.002], **_kw(dark, contype=1, conaffinity=1))
    lap.add_geom(name="dimm_module", pos=[cx, cy + DIMM_Y_FROM_CONN - 0.026, DECK_T + BOARD_Z + 0.004], size=[0.034, 0.016, 0.0015], **_kw([0.1, 0.4, 0.2, 1], contype=1, conaffinity=1))
    lap.add_geom(name="mid_rail", pos=[0, cy + RAIL_Y_FROM_CONN + 0.004, DECK_T + 0.002], size=[LAPTOP_W / 2 - WALL_T, 0.004, 0.002], **_kw(alu, contype=1, conaffinity=1))   # 4 mm tall on the deck (photo), top 1 mm above the board
    return lap


def add_connector(parent, o):
    conn = parent.add_body(name="zif", pos=o.tolist())
    black = [0.07, 0.07, 0.08, 1]; gold = [0.85, 0.70, 0.25, 1]
    hl = HOUSING_L / 2; rear = -INSERT_DEPTH
    conn.add_geom(name="zif_floor", pos=[0, -HOUSING_D / 2, SLOT_FLOOR / 2], size=[hl, HOUSING_D / 2, SLOT_FLOOR / 2], **_kw(black))
    conn.add_geom(name="zif_stop", pos=[0, rear - (HOUSING_D - INSERT_DEPTH) / 2, HOUSING_H / 2], size=[hl, (HOUSING_D - INSERT_DEPTH) / 2, HOUSING_H / 2], **_kw(black))
    gw = (HOUSING_L - CABLE_W - 0.0006) / 2
    for s, nm in ((1, "zif_guide_e"), (-1, "zif_guide_w")):
        conn.add_geom(name=nm, pos=[s * (hl - gw / 2), -INSERT_DEPTH / 2, SLOT_FLOOR + HOUSING_H / 2], size=[gw / 2, INSERT_DEPTH / 2, HOUSING_H / 2], **_kw(black))
    conn.add_geom(name="zif_roof", pos=[0, rear + ROOF_D / 2, SLOT_FLOOR + SLOT_GAP + (HOUSING_H - SLOT_FLOOR - SLOT_GAP) / 2],
                  size=[hl - gw, ROOF_D / 2, (HOUSING_H - SLOT_FLOOR - SLOT_GAP) / 2], **_kw(black))
    ck = _kw(gold, group=1); ck["solref"] = [0.05, 1.0]; ck["friction"] = [0.6, 0.005, 0.0001]      # contact springs: soft, 0.05 mm proud, a light preload on the inserted cable
    conn.add_geom(name="zif_contacts", pos=[0, rear + ROOF_D + 0.0006, SLOT_FLOOR + 0.000025], size=[CABLE_W / 2 - 0.0005, 0.0006, 0.000025], **ck)
    hinge_y = rear + ROOF_D; hinge_z = SLOT_FLOOR + LATCH_CLAMP_GAP + LID_T - LID_AXIS_BELOW_TOP
    lid = conn.add_body(name="zif_lid", pos=[0, hinge_y, hinge_z])
    j = lid.add_joint(name="zif_latch", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[1, 0, 0], range=[0.0, math.radians(LATCH_OPEN_DEG)], damping=0.0004, frictionloss=LATCH_FRICTION, armature=1e-7)
    j.limited = mujoco.mjtLimited.mjLIMITED_TRUE; j.solref_limit = [0.005, 1.0]      # a firm moulded stop at the open angle
    # the plate starts 0.5 mm in front of the hinge line and is 0.2 mm narrower than the slot, so it can swing without touching the roof or the guides
    lid.add_geom(name="zif_lid_plate", pos=[0, 0.0005 + (LID_D - 0.0005) / 2, LID_AXIS_BELOW_TOP - LID_T / 2], size=[hl - gw - 0.0001, (LID_D - 0.0005) / 2, LID_T / 2], mass=0.0008, **_kw([0.16, 0.16, 0.18, 1]))
    conn.add_site(name="zif_mouth", pos=[0, 0, SLOT_FLOOR + SLOT_GAP / 2], size=[0.0003] * 3, rgba=[1, 0, 0, 0.6])
    conn.add_site(name="zif_seat", pos=[0, rear, SLOT_FLOOR + SLOT_GAP / 2], size=[0.0003] * 3, rgba=[0, 1, 0, 0.6])
    conn.add_site(name="zif_lip", pos=[0, hinge_y + LID_D, hinge_z + LID_AXIS_BELOW_TOP], size=[0.0003] * 3, rgba=[1, 1, 0, 0.6])   # front top edge of the closed lid
    return conn


def add_cable(parent, o):
    """Keyboard FFC rooted at the deck slot (it emerges 12 mm up behind the mid rail), five flexible segments, rigid stiffener at the free end."""
    blue = [0.20, 0.35, 0.75, 1]; amber = [0.85, 0.60, 0.25, 0.95]
    if _CASE[0] == "platform":
        # Lakshya's feed: the cable root sits in a carriage on a low-friction slide at rest height, straight and level; the arm pays it out
        anchor = o + np.array([0.0, FEED_Y, PRESENTER_Z + BODY_T / 2 + 0.0002])     # feed clip FEED_Y back at rest height: the cable lies level over the rest; the clip pays out along y as the arm pulls
        root = parent.add_body(name="ffc_anchor", pos=anchor.tolist())
        root.add_joint(name="ffc_feed", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[0, 1, 0], range=[-0.030, 0.005], damping=0.5, frictionloss=0.05)
        root.add_geom(name="ffc_anchor_g", pos=[0, 0, -0.0007], size=[CABLE_W / 2 + 0.0005, 0.003, 0.0005], mass=0.006, **_kw([0.55, 0.55, 0.58, 1], contype=0, conaffinity=0))   # clip bar under the cable (1 mm), carried by a post on the west side only: the gripper body passes east of the cable
        root.add_geom(name="ffc_anchor_post", pos=[-(CABLE_W / 2 + 0.006), 0, -(PRESENTER_Z - 0.0002) / 2 - 0.0007], size=[0.002, 0.003, (PRESENTER_Z - 0.0002) / 2], mass=0.004, **_kw([0.55, 0.55, 0.58, 1], contype=0, conaffinity=0))
        parent.add_geom(name="feed_rail", pos=[anchor[0] - 0.002, anchor[1], anchor[2] - 0.010], size=[CABLE_W / 2 + 0.002, 0.010, 0.002], **_kw([0.55, 0.55, 0.58, 1], contype=0, conaffinity=0))
    else:
        anchor = o + np.array([0.0, CABLE_EXIT_Y_FROM_CONN, CABLE_EXIT_Z])     # the cable leaves the keyboard deck at deck level, just below the board
        root = parent.add_body(name="ffc_anchor", pos=anchor.tolist())
        root.add_geom(name="ffc_anchor_g", size=[CABLE_W / 2, 0.001, 0.001], mass=0.001, **_kw([0.3, 0.3, 0.3, 1], contype=0, conaffinity=0))
    prev = root
    for i in range(N_SEG):
        seg = prev.add_body(name=f"ffc_seg{i}", pos=[0, -SEG_L if i else 0, 0])
        seg.add_joint(name=f"ffc_bend{i}", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[1, 0, 0], stiffness=BEND_K, damping=BEND_K * 0.03, range=[-2.2, 2.2], armature=1e-9, springref=(CABLE_REST[i] if _CASE[0] == "laptop" else 0.0))
        if i == 0:
            seg.add_joint(name="ffc_twist0", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 1, 0], stiffness=BEND_K * 3, damping=BEND_K * 0.1, range=[-0.5, 0.5], armature=1e-9)
        if i == 2:   # yaw of the floating cable about the vertical (FFC_CABLE_YAW_DEG): the stiffener then points off the slot axis and the expert must correct it before the slide
            seg.add_joint(name="ffc_yaw", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 0, 1], stiffness=BEND_K * 3, damping=BEND_K * 0.1, range=[-0.3, 0.3], armature=1e-9, springref=math.radians(float(os.environ.get("FFC_CABLE_YAW_DEG", "0"))))
        seg.add_geom(name=f"ffc_seg{i}_g", pos=[0, -SEG_L / 2, 0], size=[CABLE_W / 2, SEG_L / 2, BODY_T / 2], mass=0.00005, **_kw(amber))   # 28 x 5 x 0.14 mm of polyimide and copper: 0.05 g
        prev = seg
    stiff = prev.add_body(name="ffc_stiffener", pos=[0, -SEG_L, 0])
    stiff.add_joint(name="ffc_bend_s", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[1, 0, 0], stiffness=BEND_K, damping=BEND_K * 0.03, range=[-2.2, 2.2], armature=1e-9)
    stiff.add_geom(name="ffc_stiffener_g", pos=[0, -STIFF_L / 2, 0], size=[CABLE_W / 2, STIFF_L / 2, STIFF_T / 2], mass=0.0002, **_kw(blue))   # 28 x 13 x 0.3 mm stiffener plus the cable end: 0.2 g
    stiff.add_site(name="ffc_tip", pos=[0, -STIFF_L, 0], size=[0.0003] * 3, rgba=[0, 0.5, 1, 0.8])
    stiff.add_site(name="ffc_grip", pos=[0, -STIFF_L + 0.0050, 0], size=[0.0003] * 3, rgba=[0, 0.5, 1, 0.8])
    return stiff


# ---------------------------------------------------------------- arm and tool ------------------------------------------------
ARM_INFO = {
    "meca500": {"urdf": "meca500.urdf", "prefix": "meca", "joints": [f"meca_axis_{k}" for k in range(1, 7)], "flange": "link_6", "home": [-90, 30, 10, 0, 50, 0]},
    "lrmate200id": {"urdf": "lrmate200id.urdf", "prefix": "lrm", "joints": [f"J{k}" for k in range(1, 7)], "flange": "flange", "home": [-90, 30, -30, 0, -90, 0]},   # forearm level, flange pointing down: the IK seed that keeps the wrist clear of its limits over the connector
}
# ---- parts at their real sizes (metres). Grippers: from the manufacturer drawings; ESTIMATE where marked. Stacks: from the flange down.
ALU = [0.55, 0.56, 0.58, 1]; PRINTED = [0.93, 0.55, 0.20, 1]; BLACK = [0.16, 0.16, 0.18, 1]; SENSOR = [0.06, 0.54, 0.61, 1]
GRIPPERS = {
    # Dimensions from the makers' drawings (assets/ffc/parts.json has the sources). body_w: extent along the stroke (vertical here) without the side connector;
    # body_d: across; body_l: along the body axis without the jaw block; end_l: the jaw block; jaw_t: jaw thickness along the stroke; jaw_sep_open: inner
    # faces apart at our open position (we use 3 mm of the stroke); mount "side": the stock side plate on the big face with the flange axis
    # plate_from_end behind the jaw end (MEGP 25E manual Figure 1: 17 mm above the finger screw line, about 21 mm above the jaw end); mount "end": the
    # body's back end face carries the mounting pattern, so our bracket's vertical leg sits behind the body.
    # MEGP 25E numbers measured on Mecademic's STEP (assets/ffc/megp25e_*.obj, gripper frame: x along the stroke, + down; y across; z forward, jaw end at 0):
    # body 26.5 across the stroke x 18.0 deep, end block 33 x 18 at 7.5 to 15 behind the jaw end, jaws 4 thick x 10 deep x 8.1 long with their outer faces 25 apart at
    # full open (gap 17), the M8 connector hanging 15.7 beyond the body on one side (mounted UP), the stock STAP01 side plate 30 x 26 x 7 (2 mm boss into the body)
    "megp25e": dict(label="Mecademic MEGP 25E", mount="side", body_w=0.0265, body_d=0.018, body_l=0.0627, end_l=0.0075, jaw_t=0.004, jaw_depth=0.010, jaw_sep_open=0.014, plate_from_end=0.021,
                    plate_t=0.007, plate_wh=0.026, conn=(0.0, 0.0, 0.0), mass=0.106, color=BLACK, meshes=dict(body="megp25e_body", jaw_lo="megp25e_jaw_lo", jaw_up="megp25e_jaw_up", plate="megp25e_plate"), mesh_gap=0.017,
                    ids=dict(part="megp25e", jaw_l="megp25e_jaw_l", jaw_r="megp25e_jaw_r", finger_l="finger_printed_l", finger_r="finger_printed_r", bracket="bracket_90_megp")),
    # Zimmer: mounted by its side pattern (2 x M3 + 2 x 5H7 on 16 mm, also on the sides per the datasheet) straight onto the bracket leg, flange axis 30 mm behind the jaw end (ESTIMATE until the CAD is read)
    # SMC LEHZ16LK2-6: body 96 x 35 x 23.7 (35 across the stroke), jaws 8 thick on a 14 wide plate projecting about 40 mm (ESTIMATE), 3 mm per jaw, side mount 2 x M3 + 2 x 2.5 pins
    "lehz16":   dict(label="SMC LEHZ16LK2-6", mount="side", body_w=0.035, body_d=0.0237, body_l=0.0862, end_l=0.010, jaw_t=0.008, jaw_depth=0.014, jaw_sep_open=0.014, plate_from_end=0.030, jaw_lo_face=0.0185,
                    plate_t=0.0, plate_wh=0.035, conn=(0.010, 0.010, 0.012), mass=0.190, color=[0.20, 0.45, 0.30, 1],
                    ids=dict(part="smc_lehz16l", jaw_l="smc_lehz16l_jaw_l", jaw_r="smc_lehz16l_jaw_r", finger_l="finger_printed_lehz16_l", finger_r="finger_printed_lehz16_r", bracket="bracket_lehz16")),
    # Servo tool v1 (ours, 2026-10-13, docs/research/2026-10/research_notes/servo_tool_v1_design_numbers.md): printed PAHT-CF body 40 x 30 x 55 with one
    # Dynamixel XL330-M288-T (its 26 mm dimension vertical, so the body bottom stays 7 mm above the shoe plane like the MEGP) on a 4 x 1 mm left/right lead screw moving both jaws (0.24 um per step, self-locking), two single-axis load cells in the
    # lower finger read at 1 kHz (the 'ft' site sits on the shoe body), no six-axis sensor and no stock bracket plate: the body's back end bolts to our bracket leg
    "servo_v1":  dict(label="Servo tool v1 (ours): XL330 lead-screw jaws, finger load cells", mount="end", body_w=0.026, body_d=0.022, body_l=0.055, end_l=0.006, jaw_t=0.004, jaw_depth=0.012, jaw_sep_open=0.014, plate_from_end=0.030,
                    plate_t=0.0, plate_wh=0.026, conn=(0.0, 0.0, 0.0), mass=0.110, color=[0.93, 0.55, 0.20, 1], d405_on_body=False, fixed_lower=False, cells=True, pad_cell=True, jaw_speed=0.0064,   # both jaws move 1.5 mm, coupled by a rocker between the two carriers on one M4 x 1 screw (2026-10-13: a fixed lower jaw lost the pinch during the lift off the rest)
                    ids=dict(part="servo_tool_v1", jaw_l="servo_jaw_l", jaw_r="servo_jaw_r", finger_l="finger_printed_l", finger_r="finger_printed_r", bracket="bracket_servo")),
    "gep2006":  dict(label="Zimmer GEP2006IL-00-B (excluded: 40 N minimum)", mount="side", body_w=0.044, body_d=0.022, body_l=0.069, end_l=0.0165, jaw_t=0.007, jaw_depth=0.022, jaw_sep_open=0.019, plate_from_end=0.030,
                    plate_t=0.0, plate_wh=0.044, conn=(0.010, 0.010, 0.012), mass=0.180, color=[0.20, 0.20, 0.55, 1],
                    ids=dict(part="zimmer_gep2006il", jaw_l="zimmer_jaw_l", jaw_r="zimmer_jaw_r", finger_l="finger_printed_zimmer_l", finger_r="finger_printed_zimmer_r", bracket="bracket_zimmer")),
}
STACKS = {   # (id, part id in parts.json, shape, dims, mass, colour, x offset): cylinders (diameter, height), boxes (along tool x, along tool y, height)
    "meca_nano25":       [("flange", "meca500_flange", "cyl", (0.020, 0.0022), 0.0, [0.45, 0.46, 0.48, 1], 0), ("adapter_plate", "adapter_plate_meca_nano25", "cyl", (0.028, 0.004), 0.007, ALU, 0),
                          ("nano25", "nano25", "cyl", (0.025, 0.0216), 0.0634, SENSOR, 0)],   # ATI Nano25 SI-62.5-1.5 (decided 2026-10-13): 25 dia x 21.6, 63.4 g, 3 x M3 on a 20.32 mm circle both faces
    "meca_nano25_zaber": [("flange", "meca500_flange", "cyl", (0.020, 0.0022), 0.0, [0.45, 0.46, 0.48, 1], 0), ("adapter_plate", "adapter_plate_meca_nano25", "cyl", (0.028, 0.004), 0.007, ALU, 0),
                          ("nano25", "nano25", "cyl", (0.025, 0.0216), 0.0634, SENSOR, 0),
                          ("zaber", "zaber_xna08a25", "box", (0.1233, 0.0496, 0.0210), 0.140, [0.25, 0.25, 0.28, 0.0], 0.030)],   # Zaber's STEP (retracted): 123 long x 49.6 (controller box) x 21; invisible box for mass, the CAD mesh is drawn (acts along the insertion axis)
    # no sensor in these stacks (the load cells are in the finger); a printed standoff keeps the flange at the same height over the work as the sensor stacks,
    # because 22 mm lower the LR Mate's wrist housing and the tool body met the floating cable in the latch phase (run of 2026-10-13)
    "meca_servo":        [("flange", "meca500_flange", "cyl", (0.020, 0.0022), 0.0, [0.45, 0.46, 0.48, 1], 0), ("adapter_plate", "adapter_plate_meca_servo", "cyl", (0.028, 0.004), 0.007, ALU, 0),
                          ("standoff", "standoff_servo", "cyl", (0.028, 0.022), 0.010, PRINTED, 0)],
    "lrmate_servo":      [("flange", "lrmate_flange", "cyl", (0.040, 0.008), 0.0, [0.45, 0.46, 0.48, 1], 0), ("adapter_plate", "adapter_plate_lrmate_servo", "cyl", (0.044, 0.006), 0.040, ALU, 0),
                          ("standoff", "standoff_servo_lr", "cyl", (0.036, 0.034), 0.030, PRINTED, 0)],   # 34 mm (plus the 12.7 mm cell and 6 mm gap): puts the jaw line where the Mini40 stack has it, so the LR Mate wrist stays out of the bench cameras during the lid lift,
    "lrmate_mini40":     [("flange", "lrmate_flange", "cyl", (0.040, 0.008), 0.0, [0.45, 0.46, 0.48, 1], 0), ("adapter_plate", "adapter_plate_lrmate_mini40", "cyl", (0.044, 0.006), 0.040, ALU, 0),
                          ("mini40", "mini40", "cyl", (0.0401, 0.014), 0.050, SENSOR, 0)],
}
BRACKET_T, BRACKET_W = 0.006, 0.030                                        # our 90 degree bracket: 6 mm 6061, 30 mm wide along the body axis
FPLATE_T = 0.006                                                            # finger plate bolted to a jaw's outer face
JAW_TRAVEL = 0.0015                                                         # each jaw moves this much to close the 3 mm gap (both jaws move)


def _grip(arm, tool=None):
    return GRIPPERS[TOOL_INFO[tool or DEFAULT_TOOL[arm]]["gripper"]]


def upper_travel(arm, tool=None):
    """stroke of the upper (pad) jaw: the whole gap on the servo tool, half of it on the bought grippers"""
    return JAW_TRAVEL * (2 if _grip(arm, tool).get("fixed_lower") else 1)


def lower_travel(arm, tool=None):
    return 0.0 if _grip(arm, tool).get("fixed_lower") else JAW_TRAVEL


def jaw_speed(arm, tool=None):
    """jaw closing speed used by the expert for the open time (m/s): XL330-M077 on a 1 mm lead at 383 rpm is 6.4 mm/s"""
    return _grip(arm, tool).get("jaw_speed", 0.010)
# vision parts (assets/ffc/vision_parts.json): body w x h x depth, lens dia x length, working distance from the lens front, vertical field of view at that distance
CAMERAS = {
    "meca500": dict(macro="basler_daA1920_160um_smount", lens="eo_37182_16mm_f4_m12", light="ccs_ldl2_33x8bl", size=(0.029, 0.029, 0.0149), lens_dl=(0.016, 0.0165), mass=0.015, lens_mass=0.008,
                    wd=0.108, fovy=13.6, pitch_deg=25.0, light_size=(0.044, 0.0104, 0.013), light_mass=0.020, mesh=None),
    "lrmate200id": dict(macro="alvium_1800_u500m_open_housing_c_mount", lens="eo_33303_12mm_uc_c_mount", light="svl_lm75_470", size=(0.0294, 0.0294, 0.030), lens_dl=(0.030, 0.0352), mass=0.045, lens_mass=0.049,
                        wd=0.100, fovy=22.4, pitch_deg=25.0, light_size=(0.080, 0.026, 0.034), light_mass=0.128, mesh=None),
}
# 2026-10-13: the macro cameras and the bar light stand on the bench, not on the wrist. The wrist arms sat in their own lenses' view (every
# macro frame was 96 percent bracket and arm), a learned policy needs a view the arm cannot shake, and the 16 mm M12 lens is only specified from
# 150 mm. One camera set for both arms: Basler dart + Edmund #37-182 at 160 mm (optimised range 150 to 250 mm), 31 um per pixel, 60 x 37 mm field.
# Azimuth from east (+x) toward north (+y, behind the connector); the cable comes from the south. The defaults come from the occlusion sweep
# (scripts/ffc_zif_camera_check.py --sweep). FFC_FIXED_CAMS=0 restores the wrist mounts.
# Basler docs (2026-10-13): the S-mount dart is 20 x 29 x 29 mm, under 15 g
FIXED_CAMS = os.environ.get("FFC_FIXED_CAMS", "1") == "1"
CAMERAS_WRIST = CAMERAS
FIXED_CAM = dict(macro="basler_daA1920_160um_smount", lens="eo_37182_16mm_f4_m12", light="ccs_ldl2_33x8bl", size=(0.029, 0.029, 0.020), lens_dl=(0.016, 0.0165), mass=0.015, lens_mass=0.008,
                 wd=0.160, fovy=14.7, um_per_px=31.0, field_mm=(60, 37), pitch_deg=25.0, light_size=(0.044, 0.0104, 0.013), light_mass=0.020, mesh=None,
                 az_deg=(float(os.environ.get("FFC_CAM_AZ_E", "-60")), float(os.environ.get("FFC_CAM_AZ_W", "-150"))), el_deg=float(os.environ.get("FFC_CAM_EL", "25")),
                 light_wd=0.060, light_az_deg=180.0, light_el_deg=25.0)
if FIXED_CAMS:
    CAMERAS = {"meca500": dict(FIXED_CAM), "lrmate200id": dict(FIXED_CAM)}
TOOL_INFO = {
    "megp_nano25":       dict(label="MEGP 25E + Nano25, 0.5 mm shoe", stack="meca_nano25", gripper="megp25e", shoe_t=0.0005),
    "megp_nano25_thin":  dict(label="MEGP 25E + Nano25, 0.3 mm shoe", stack="meca_nano25", gripper="megp25e", shoe_t=0.0003),
    "megp_nano25_zaber": dict(label="MEGP 25E + Nano25 + Zaber X-NA08A25 pusher", stack="meca_nano25_zaber", gripper="megp25e", shoe_t=0.0005),
    "servo_v1":          dict(label="Servo tool v1 on the Meca500 (no F/T sensor, finger load cells)", stack="meca_servo", gripper="servo_v1", shoe_t=0.0005),
    "servo_v1_lr":       dict(label="Servo tool v1 on the LR Mate (no F/T sensor, finger load cells)", stack="lrmate_servo", gripper="servo_v1", shoe_t=0.0005),
    "lehz16_mini40":     dict(label="SMC LEHZ16 + Mini40, 0.5 mm shoe", stack="lrmate_mini40", gripper="lehz16", shoe_t=0.0005),
    "gep_mini40":        dict(label="Zimmer GEP2006 + Mini40 (excluded)", stack="lrmate_mini40", gripper="gep2006", shoe_t=0.0005),
}
DEFAULT_TOOL = {"meca500": "megp_nano25", "lrmate200id": "lehz16_mini40"}
CASES = {"laptop": "the FX505 upside down on the pallet, cable out of the deck slot", "platform": "Lakshya's platform: connector on a plate, cable in a feed"}
_CASE = ["laptop"]
JAW_DROP = 0.030                                                         # jaw line to the shoe plane: the L fingers hang 30 mm below the gripper's jaw carriages
BLOCK_CLEAR = 0.015                                                      # gripper body underside above the shoe plane, whatever the body size
LEG_W = 0.006                                                             # 6 x 6 mm printed legs: 0.05 mm deflection at the 5 N pinch (4 x 4 gave 0.24)
FINGER_OFF = 0.026                                                          # gripper body centreline east of the cable centreline: the body (18 or 22 across) stands beside the 28 mm cable, never over it
JAW_LO_FACE = 0.009                                                         # the lower jaw's outer face this far above the shoe plane: the end block clears the board by 2.5 mm, the upper finger stays under 30 mm
PAD_W, PAD_D = 0.030, 0.006                                              # upper pad 32 mm: covers the 28 mm cable, stays off parts beside the 31 mm connector; 6 mm along the cable so it never reaches past the stiffener root onto the rest
PAD_T_UP, PAD_T_LO = 0.002, 0.0005                                       # thick upper pad, thin steel lower shoe so the cable lies at board height
SHOE_W = 0.020                                                           # the lower shoe is narrower than the cable so it passes between the presenter rails
PAD_OPEN = 0.003                                                         # jaw opening: 3 mm is plenty for a 0.3 mm cable and keeps the open pad below the floating cable during the latch sweep
BLADE_L, BLADE_T, BLADE_W = 0.0015, 0.0004, 0.020                        # latch lip: a short tongue ahead of the shoe, flush with its underside (hooks the lid overhang, presses nothing)
SHOE_D = 0.007                                                           # shoe depth along the cable: short, so the press pad fits over the lid without touching the rising cable behind it
TOOL_MASS = 0.20


def add_arm(spec, arm):
    info = ARM_INFO[arm]
    child = mujoco.MjSpec.from_file(str(ASSETS / info["urdf"])); child.modelname = info["prefix"]
    for g in child.geoms:
        g.contype = 0; g.conaffinity = 0; g.group = 2
    bx, by, bz = ARM_BASE[arm]
    frame = spec.worldbody.add_frame(pos=[bx, by, bz + PLATE[arm][2]])
    spec.attach(child, prefix=info["prefix"] + "_", frame=frame)
    return spec.body(info["prefix"] + "_" + info["flange"])


def add_tool(spec, flange, arm, tool_name=None):
    """The wrist stack the way it will be built, every part at its real size. Tool frame: z down the flange axis, x backward (the jaws
    point along -x), y east. From the flange: adapter plate, F/T sensor, tool-side plate, then our 90 degree bracket whose vertical leg
    carries the gripper's stock side plate; the gripper body lies horizontal pointing at the connector with its jaw end 17 mm ahead of
    the flange axis (MEGP 25E manual, Figure 1) and its stroke vertical. Both jaws move; each carries one C-shaped finger from the east.
    Jaw frame: z forward (toward the connector), x down (closing direction), y across (+ east); origin on the open shoe's upper surface,
    16 mm behind the jaw tip line."""
    ti = TOOL_INFO[tool_name or DEFAULT_TOOL[arm]]; G = GRIPPERS[ti["gripper"]]; PAD_T_LO = ti["shoe_t"]
    tool = flange.add_body(name="tool", pos=[0, 0, 0])
    V = dict(contype=0, conaffinity=0)
    # ---- the stack along the flange axis
    z0 = 0.0
    for pid, part, shape, dims, mass, col, xoff in STACKS[ti["stack"]]:
        if pid == "flange":                                                   # the robot's own flange disc, drawn above its face (z < 0) so it has a card
            d, h = dims; k = _kw(col, **V); k["type"] = mujoco.mjtGeom.mjGEOM_CYLINDER
            tool.add_geom(name="stack_flange", pos=[0, 0, -h / 2], size=[d / 2, h / 2, 0], mass=0.0, **k); continue
        if shape == "cyl":
            d, h = dims; k = _kw(col, **V); k["type"] = mujoco.mjtGeom.mjGEOM_CYLINDER
            tool.add_geom(name=f"stack_{pid}", pos=[xoff, 0, z0 + h / 2], size=[d / 2, h / 2, 0], mass=mass, **k)
            if pid in ("nano17", "nano25", "mini40"):
                tool.add_site(name="ft", pos=[0, 0, z0 + h / 2], size=[0.004] * 3, rgba=[1, 0.3, 0.3, 0.4])
        else:
            w, dd, h = dims
            tool.add_geom(name=f"stack_{pid}", pos=[xoff, 0, z0 + h / 2], size=[w / 2, dd / 2, h / 2], mass=mass, **_kw(col, **V))
            if pid == "zaber":   # the CAD mesh: its long axis (file z) along tool x, the controller box (file x) across tool y
                spec.add_mesh(name="zaber_xna08a25", file=str(ASSETS.parent / "ffc" / "zaber_xna08a25.obj"))
                _q, _off = _mesh_fit("zaber_xna08a25", str(ASSETS.parent / "ffc" / "zaber_xna08a25.obj"), dims)   # the STEP's own axes do not matter: the fit lays the mesh on the slab
                tool.add_geom(name="stack_zaber_mesh", meshname="zaber_xna08a25", pos=[xoff + _off[0], _off[1], z0 + h / 2 + _off[2]], quat=_q, mass=0.0, type=mujoco.mjtGeom.mjGEOM_MESH, contype=0, conaffinity=0, rgba=[0.25, 0.25, 0.28, 1])
        z0 += h
    Ls = z0
    # ---- 90 degree bracket (ours, 6061): horizontal leg under the stack, vertical leg down the east side carrying the gripper's plate
    jlf = G.get("jaw_lo_face", JAW_LO_FACE)                                     # the LEHZ16's long body needs its jaw line higher so the body clears the laptop's edge wall (14 mm) and the platform clip
    C = jlf + G["jaw_sep_open"] / 2 + G["jaw_t"]                                  # stroke centreline above the shoe plane
    top_above_C = (max(G["plate_wh"], G["body_w"]) / 2 + (0.006 if G.get("cells") else 0.0)) if G["mount"] == "end" else max(G["plate_wh"] / 2, 0.02895)   # MEGP: the connector reaches 28.95 above the stroke centre; servo tool: the 6 mm belt cover on top of the body
    bt = 0.0127 if G.get("cells") else BRACKET_T                                 # the horizontal member: a 12.7 mm bar load cell on the servo tool, the 6 mm 6061 leg otherwise
    gap = 0.002
    zj = Ls + bt + gap + top_above_C + C                                          # flange to the shoe plane (open jaws): the highest part sits under the bracket
    z_plate = zj - C                                                              # stroke centreline, in tool z
    ZJ = 0.002                                                                   # jaw end face, 14 mm behind the shoe tip line
    if G["mount"] == "side":
        y_plate = FINGER_OFF + G["body_d"] / 2 + G["plate_t"] / 2               # gripper's stock side plate centre (east of the cable centreline)
        y_leg = FINGER_OFF + G["body_d"] / 2 + G["plate_t"] + BRACKET_T / 2
        tool.add_geom(name="bracket_h", pos=[0, (y_leg + BRACKET_T / 2 - 0.015) / 2, Ls + BRACKET_T / 2], size=[BRACKET_W / 2, (y_leg + BRACKET_T / 2 + 0.015) / 2, BRACKET_T / 2], mass=0.025, **_kw(ALU, **V))
        tool.add_geom(name="bracket_v", pos=[0, y_leg, (Ls + BRACKET_T + z_plate + G["plate_wh"] / 2) / 2], size=[BRACKET_W / 2, BRACKET_T / 2, (z_plate + G["plate_wh"] / 2 - Ls - BRACKET_T) / 2], mass=0.030, **_kw(ALU, **V))
        if G["plate_t"] > 0:
            tool.add_geom(name="gripper_plate", pos=[0, y_plate, z_plate], size=[BRACKET_W / 2, G["plate_t"] / 2, G["plate_wh"] / 2], mass=0.012, **_kw([*G["color"][:3], 0.0] if "meshes" in G else G["color"], contype=0, conaffinity=0))   # the stock side plate, flange axis through its centre (invisible box when the CAD plate is drawn)
        px = -(G["plate_from_end"] - ZJ)                                         # flange axis at jaw z = ZJ - plate_from_end
    else:
        # end mount: the body's back end face bolts to the bracket's vertical leg, which hangs right under the flange axis
        L = G["body_l"] + G["end_l"]; px = ZJ - L - BRACKET_T / 2                 # flange axis half a leg thickness behind the body's back face
        w_leg = FINGER_OFF + G["body_d"] / 2 + 0.004
        if G.get("cells"):
            # servo tool: two 5 kg bar load cells (TAL220B, 55 x 12.7 x 12.7) in series carry the whole tool from the standoff, so they read every
            # force at the tips (the pinch is internal and cancels). Bar A runs east from under the standoff (its fixed end) and bends along the body
            # axis: the insertion axis (slide, seat, lid push). Bar B hangs off A's free end and runs along the body axis beside the body's east
            # face, bending along the flange axis: the vertical axis (lid lift, latch press). A 4 mm corner plate on top joins A's free end to B's fixed end;
            # B's free end carries a tab on top and a side plate down the body's east face (4 x M3). Nothing but the standoff is on the fixed side.
            CELL = [0.30, 0.32, 0.36, 1]; hb = 0.00635
            tool.add_geom(name="bracket_h", pos=[0, (w_leg - 0.014) / 2, Ls + hb], size=[hb, (w_leg + 0.014) / 2, hb], mass=0.024, **_kw(CELL, **V))                       # bar A (insertion axis)
            y_b = w_leg + hb + 0.001; xb1 = hb; xb0 = xb1 - 0.055
            tool.add_geom(name="cell_b", pos=[(xb0 + xb1) / 2, y_b, Ls + hb], size=[0.0275, hb, hb], mass=0.024, **_kw(CELL, **V))                                        # bar B (vertical axis)
            tool.add_geom(name="cell_corner", pos=[0, (w_leg - 0.015 + y_b + hb) / 2, Ls - 0.002], size=[hb, (y_b + hb - w_leg + 0.015) / 2, 0.002], mass=0.006, **_kw(ALU, **V))   # 4 mm 6061 plate on top of A's free end and B's fixed end (beside the standoff)
            y_face = FINGER_OFF + G["body_d"] / 2
            tool.add_geom(name="cell_tab", pos=[xb0 + 0.0075, (y_face + y_b + hb) / 2, Ls - 0.002], size=[0.0075, (y_b + hb - y_face) / 2, 0.002], mass=0.005, **_kw(ALU, **V))         # tab on top of B's free end, in to the body's east face
            tool.add_geom(name="bracket_v", pos=[xb0 + 0.0075, y_face + 0.002, (Ls - 0.004 + z_plate) / 2], size=[0.0075, 0.002, (z_plate - Ls + 0.004) / 2], mass=0.008, **_kw(ALU, **V))   # side plate down the body's east face, 4 x M3
        else:
            tool.add_geom(name="bracket_h", pos=[0, (w_leg - 0.015) / 2, Ls + BRACKET_T / 2], size=[BRACKET_W / 2, (w_leg + 0.015) / 2, BRACKET_T / 2], mass=0.030, **_kw(ALU, **V))
            tool.add_geom(name="bracket_v", pos=[0, FINGER_OFF, (Ls + BRACKET_T + z_plate + G["body_w"] / 2) / 2], size=[BRACKET_T / 2, G["body_d"] / 2 + 0.004, (z_plate + G["body_w"] / 2 - Ls - BRACKET_T) / 2], mass=0.040, **_kw(ALU, **V))
    jaws = tool.add_body(name="jaws", pos=[px, 0, zj], quat=[math.cos(-math.pi / 4), 0, math.sin(-math.pi / 4), 0])
    if G.get("cells"):
        tool.add_site(name="cell", pos=[0, w_leg, Ls + 0.00635], size=[0.003] * 3, rgba=[1, 0.3, 0.3, 0.4])   # the two bar cells: on the tool body like the ATI's 'ft' site, so the sensor reads the net external force on everything below the flange (a site on a child body miscounts the grasp weld's reaction)
    bw, bd, bl, el, jt, sep = G["body_w"], G["body_d"], G["body_l"], G["end_l"], G["jaw_t"], G["jaw_sep_open"]
    HAS_MESH = "meshes" in G
    hid = dict(contype=0, conaffinity=0)
    bodycol = [*G["color"][:3], 0.0] if HAS_MESH else G["color"]                 # with CAD meshes the boxes stay only as mass and reference geometry, invisible
    jaws.add_geom(name="gripper_body", pos=[-C, FINGER_OFF, ZJ - el - bl / 2], size=[bw / 2, bd / 2, bl / 2], mass=G["mass"] * 0.8, **_kw(bodycol, **hid))
    jaws.add_geom(name="gripper_end", pos=[-C, FINGER_OFF, ZJ - el / 2], size=[sep / 2 - 0.0005, bd / 2, el / 2], mass=G["mass"] * 0.1, **_kw(bodycol, **hid))   # the jaw guide between the jaws
    if G.get("cells"):
        jaws.add_geom(name="gripper_cover", pos=[-C - bw / 2 - 0.003, FINGER_OFF, ZJ - el - bl / 2], size=[0.003, bd / 2, bl / 2], mass=0.012, **_kw(bodycol, **hid))   # belt cover: the servo lies flat and drives the screw through a GT2 belt over the top
    if HAS_MESH:
        for key, fn in G["meshes"].items():
            spec.add_mesh(name=fn, file=str(ASSETS.parent / "ffc" / f"{fn}.obj"))
        mk = dict(type=mujoco.mjtGeom.mjGEOM_MESH, contype=0, conaffinity=0, group=0, rgba=G["color"])
        jaws.add_geom(name="gripper_mesh", meshname=G["meshes"]["body"], pos=[-C, FINGER_OFF, ZJ], mass=0.0, **mk)
        if G["mount"] == "side":
            jaws.add_geom(name="gripper_plate_mesh", meshname=G["meshes"]["plate"], pos=[-C, FINGER_OFF, ZJ - G["plate_from_end"]], mass=0.0, **mk)
    else:
        cw, cd, cl = G["conn"]
        if cw > 0: jaws.add_geom(name="gripper_conn", pos=[-C - bw / 2 - cw / 2, FINGER_OFF, ZJ - el - 0.035], size=[cw / 2, cd / 2, cl / 2], mass=0.005, **_kw([0.35, 0.36, 0.38, 1], **V))   # side connector, on top
    tipz = PAD_D + 0.010
    LEG_Y = CABLE_W / 2 + 0.001 + LEG_W / 2; PLEG_Y = LEG_Y + LEG_W + 0.0005
    y_jaw0, y_jaw1 = FINGER_OFF - bd / 2, FINGER_OFF + bd / 2
    # ---- lower jaw + lower finger, on the shoe body (slides up when the gripper closes)
    shoe = jaws.add_body(name="shoe", pos=[0, 0, 0])
    if not G.get("fixed_lower"):   # servo tool: the lower jaw is welded to the body (a locked joint would feed its constraint force into the wrist force sensor as if it were external)
        sj = shoe.add_joint(name="shoe_slide", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[-1, 0, 0], range=[0, JAW_TRAVEL + 0.0001], damping=3.0)
        sj.limited = mujoco.mjtLimited.mjLIMITED_TRUE; sj.solref_limit = [0.002, 1.0]
    sk = _kw([0.8, 0.8, 0.82, 1], contype=TOOL, conaffinity=PART); sk["friction"] = [0.5, 0.005, 0.0001]        # steel shoe, a little texture
    shoe.add_geom(name="shoe_g", pos=[PAD_T_LO / 2, 0, tipz - SHOE_D / 2], size=[PAD_T_LO / 2, SHOE_W / 2, SHOE_D / 2], mass=0.004, **sk)
    shoe.add_geom(name="shoe_bar", pos=[PAD_T_LO / 2, (LEG_Y + LEG_W / 2 - SHOE_W / 2) / 2, tipz - SHOE_D + 0.0005], size=[PAD_T_LO / 2, (LEG_Y + LEG_W / 2 + SHOE_W / 2) / 2, 0.0015], mass=0.002, **_kw([0.8, 0.8, 0.82, 1], contype=TOOL, conaffinity=PART))   # rear wing out to the leg
    shoe.add_geom(name="blade", pos=[PAD_T_LO - BLADE_T / 2, 0, tipz + BLADE_L / 2], size=[BLADE_T / 2, BLADE_W / 2, BLADE_L / 2], mass=0.001, **_kw([0.85, 0.85, 0.87, 1], contype=TOOL, conaffinity=PART))   # the lip
    x_lo = -(C - sep / 2 - jt)                                                   # lower jaw's outer (bottom) face above the shoe plane (= -jlf)
    shoe.add_geom(name="jaw_lo", pos=[x_lo - jt / 2, FINGER_OFF, ZJ - el / 2], size=[jt / 2, G["jaw_depth"] / 2, el / 2], mass=0.005, **_kw(bodycol, **hid))
    if HAS_MESH:
        shoe.add_geom(name="jaw_lo_mesh", meshname=G["meshes"]["jaw_lo"], pos=[-C - (G["mesh_gap"] - sep) / 2, FINGER_OFF, ZJ], mass=0.0, **mk)   # CAD jaw at full open, moved in to our open gap
    y_pl0, y_pl1 = LEG_Y - LEG_W / 2, FINGER_OFF + G["jaw_depth"] / 2             # finger plates span from the leg to the jaw's east edge
    shoe.add_geom(name="plate_lo", pos=[x_lo + FPLATE_T / 2, (y_pl0 + y_pl1) / 2, (ZJ - el + tipz - SHOE_D + 0.003) / 2], size=[FPLATE_T / 2, (y_pl1 - y_pl0) / 2, (tipz - SHOE_D + 0.003 - ZJ + el) / 2], mass=0.008, **_kw(PRINTED, **V))   # finger plate on the jaw's outer face (M4 + two 1.5 pins), reaching forward to the leg
    shoe.add_geom(name="leg_e", pos=[(x_lo + FPLATE_T) / 2, LEG_Y, tipz - SHOE_D + 0.001], size=[-(x_lo + FPLATE_T) / 2, LEG_W / 2, 0.002], mass=0.004, **_kw(PRINTED, contype=TOOL, conaffinity=PART))   # the one lower leg, east of the cable
    if not any(pid in ("nano17", "nano25", "mini40") for pid, *_ in STACKS[ti["stack"]]):
        tool.add_site(name="ft", pos=[0, 0, Ls], size=[0.004] * 3, rgba=[1, 0.3, 0.3, 0.2])                                  # the IK's orientation reference (no six-axis sensor in this stack)
    shoe.add_site(name="blade_tip", pos=[PAD_T_LO, 0, tipz + BLADE_L], size=[0.0004] * 3, rgba=[1, 0, 1, 0.9])     # lip underside, front edge
    shoe.add_site(name="tcp", pos=[0, 0, tipz], size=[0.0005] * 3, rgba=[0, 1, 1, 0.9])                 # jaw tip line on the shoe's upper surface
    shoe.add_site(name="shoe_bottom", pos=[PAD_T_LO, 0, tipz - SHOE_D / 2], size=[0.0005] * 3, rgba=[0, 1, 1, 0.9])   # centre of the shoe underside: the latch press pad
    # ---- upper jaw + upper finger, on the pad body (slides down when the gripper closes)
    px0 = -PAD_OPEN - PAD_T_UP / 2; pz = PAD_D / 2 + 0.010
    pad = jaws.add_body(name="pad", pos=[px0, 0, pz])
    pj = pad.add_joint(name="pad_slide", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[1, 0, 0], range=[0, (2 if G.get("fixed_lower") else 1) * JAW_TRAVEL + 0.0001], damping=3.0)   # servo tool: the upper jaw does the whole 3 mm
    pj.limited = mujoco.mjtLimited.mjLIMITED_TRUE; pj.solref_limit = [0.002, 1.0]     # hard mechanical stops
    pk = _kw(PRINTED, contype=TOOL, conaffinity=PART); pk["friction"] = [0.9, 0.005, 0.0001]     # rubber face
    pad.add_geom(name="pad_g", size=[PAD_T_UP / 2, PAD_W / 2, PAD_D / 2], mass=0.006, **pk)
    x_up = -(C + sep / 2 + jt)                                                   # upper jaw's outer (top) face above the shoe plane
    pad.add_geom(name="jaw_up", pos=[x_up + jt / 2 - px0, FINGER_OFF, ZJ - el / 2 - pz], size=[jt / 2, G["jaw_depth"] / 2, el / 2], mass=0.005, **_kw(bodycol, **hid))
    if HAS_MESH:
        pad.add_geom(name="jaw_up_mesh", meshname=G["meshes"]["jaw_up"], pos=[-C + (G["mesh_gap"] - sep) / 2 - px0, FINGER_OFF, ZJ - pz], mass=0.0, **mk)
    zp0 = ZJ - el + (0.0127 if G.get("pad_cell") else 0.0)                     # servo tool: the rear 12.7 mm of the upper plate is a bar load cell
    pad.add_geom(name="plate_up", pos=[x_up - FPLATE_T / 2 - px0, (y_pl0 + y_pl1) / 2, (zp0 + pz + PAD_D / 2) / 2 - pz], size=[FPLATE_T / 2, (y_pl1 - y_pl0) / 2, (pz + PAD_D / 2 - zp0) / 2], mass=0.008, **_kw(PRINTED, **V))
    if G.get("pad_cell"):
        # third bar cell (TAL220B 5 kg): the upper finger's cross member on top of the moving jaw, fixed end at the jaw, free end over the cable
        # carrying the printed plate, leg, arm and pad; it bends along the stroke, so it reads the pinch on the pad directly (the wrist pair cannot:
        # the pinch is internal to them). Insensitive to the slide friction, which loads it across its stiff width.
        y_pc1 = FINGER_OFF + G["jaw_depth"] / 2 + 0.003; y_pc0 = y_pc1 - 0.055
        pad.add_geom(name="cell_p", pos=[x_up - 0.00635 - px0, (y_pc0 + y_pc1) / 2, (ZJ - el + 0.00635) - pz], size=[0.00635, 0.0275, 0.00635], mass=0.024, **_kw([0.30, 0.32, 0.36, 1], **V))
        # the printed upper plate reaches the cell's free end with a tongue on top of it (2 x M5 down into the cell's threaded free end) and a web down to the plate level
        pad.add_geom(name="plate_up_tongue", pos=[x_up - 0.0127 - 0.002 - px0, (y_pc0 + y_pc0 + 0.016) / 2, (ZJ - el + 0.00635) - pz], size=[0.002, 0.008, 0.00635], mass=0.003, **_kw(PRINTED, **V))
        pad.add_geom(name="plate_up_web", pos=[(x_up - 0.0127 - 0.004 + x_up - FPLATE_T) / 2 - px0, (y_pc0 + y_pc0 + 0.016) / 2, (ZJ - el + 0.0127 + 0.002) - pz], size=[(FPLATE_T + 0.0127 + 0.004) / 2 - 0.0005, 0.008, 0.002], mass=0.003, **_kw(PRINTED, **V))
    x_pad_top = -PAD_OPEN - PAD_T_UP                                              # the pad's top face (jaw frame, x down)
    # the arm between the pad and the leg: 12 to 15 mm ahead of the jaw line (ahead of the lower plate's front edge at 12, behind the
    # connector's guide walls, which an arm reaching to 18 mm clipped by 0.6 mm when the tip was seated), and 8 mm tall above the pad
    # for stiffness, which it can be because the lower plate does not exist ahead of 12 mm
    x_arm_top = -PAD_OPEN - PAD_T_UP - 0.008
    pad.add_geom(name="pad_arm", pos=[(x_arm_top + x_pad_top) / 2 - px0, (PAD_W / 2 + PLEG_Y + LEG_W / 2) / 2, 0.0005], size=[(x_pad_top - x_arm_top) / 2, (PLEG_Y + LEG_W / 2 - PAD_W / 2) / 2, 0.0015], mass=0.002, **_kw(PRINTED, **V))
    pad.add_geom(name="pad_leg_e", pos=[(x_up - FPLATE_T + x_arm_top) / 2 - px0, PLEG_Y, 0.0005], size=[(x_arm_top - x_up + FPLATE_T) / 2, LEG_W / 2, 0.0015], mass=0.004, **_kw(PRINTED, contype=TOOL, conaffinity=PART))   # the one upper leg, plate to arm, 12 to 15 mm ahead
    # ---- cameras and their mounts. Macro cameras: at the lens working distance from the jaw tips, 25 degrees down, one each side; each
    # hangs on a 6 x 10 mm 6061 arm that runs UNDER the bracket's horizontal leg (the sensor sits above it), outboard of the leg's ends, forward
    # to the camera's x, then out to the camera's inner side face, where a small angle plate carries the camera's own holes. D405: behind the
    # flange, 12 mm west of the cable line, under a rear arm. Light: on a short drop from the west arm. Everything here is audited for collisions.
    CM = CAMERAS[arm]; look = np.array([0.0, 0.0, tipz + 0.004])                 # the jaw tips, where the cable meets the mouth
    reach = CM["wd"] + CM["lens_dl"][1] + CM["size"][2] / 2                       # look point to the camera body centre, along the line of sight
    pitch = math.radians(CM["pitch_deg"]); z_bar = Ls + BRACKET_T + 0.003          # arm bars: 6 mm thick, hung under the leg
    cam_positions = {}
    # D405 (42 wide x 42 tall x 23 deep): behind and west of the flange, its back against a plate at the end of an L arm that leaves the leg's
    # west end at leg level and runs back; it looks forward-down at the jaw tips. Its two M3 (20 mm apart) are on its back, so the plate carries them.
    # D405: beside the stack, 46 mm west of the cable line and 55 mm back, its centre 36 mm above the shoe plane so the tilted body's lowest
    # point stays 12 mm above the board while the shoe is on it (2026-10-13: at 56 mm below the flange it went 6 mm into the laptop deck).
    # Carried by a 3 mm plate on its west side face (the D405's 2 x M2, 18 mm apart); the bars run beside the camera, never over or through it.
    if G.get("d405_on_body"):
        # servo tool: the D405 rides on top of the tool body near its back end on a short printed pedestal (no bars), pitched down at the tips
        d405_jaw = (-(C + bw / 2 + 0.004 + 0.024), FINGER_OFF, ZJ - el - 0.010)
        d405_look = _look_quat(d405_jaw, [0, FINGER_OFF, tipz + 0.015], up=(-1, 0, 0))
        jaws.add_geom(name="arm_d405_plate", pos=[-(C + bw / 2) - 0.002, FINGER_OFF, d405_jaw[2]], size=[0.002, 0.010, 0.010], mass=0.004, **_kw(PRINTED, **V))   # pedestal, printed with the body
        jaws.add_geom(name="d405_g", pos=list(d405_jaw), quat=d405_look, size=[0.021, 0.021, 0.0115], mass=0.058, **_kw([0.12, 0.12, 0.13, 1], **V))
        jaws.add_camera(name="wrist_d405", pos=list(d405_jaw), quat=d405_look, fovy=58)
        G.get("fixed_lower") or spec.add_equality(name="jaws_couple", type=mujoco.mjtEq.mjEQ_JOINT, name1="shoe_slide", name2="pad_slide", data=[0, 1, 0, 0, 0] + [0] * 6, objtype=mujoco.mjtObj.mjOBJ_JOINT, solref=[0.001, 1.0])
        return tool
    d405_jaw = (-0.036, -0.046, -0.040 if G.get("cells") else -0.055); d405_tool = (px - d405_jaw[2], d405_jaw[1], zj + d405_jaw[0])   # servo tool: 15 mm further forward, clear of the bar that hangs under cell A (81 mm from the tips, D405 minimum 70)
    d405_look = _look_quat(d405_jaw, [0, d405_jaw[1], tipz + 0.015], up=(-1, 0, 0))                   # pitched only, no yaw: the body's side faces stay in the y planes so the side plate sits flush; the tips are 27 deg off the optical axis, inside the 87 deg field
    z_leg = Ls + bt + 0.003                                                                    # the arm bars hang under the bracket leg (bolted lap, 2 x M3)
    y_bar = d405_tool[1] - 0.029                                                                      # bar centreline 29 mm west of the camera centre: 5 mm clear of the side plate's outer face
    x_end = d405_tool[0] + 0.015
    tool.add_geom(name="arm_d405_y", pos=[0.010, y_bar / 2, z_leg], size=[0.005, abs(y_bar) / 2 + 0.005, 0.003], mass=0.012, **_kw(ALU, **V))   # under the leg, from its centreline (15 mm lap) west past the camera
    tool.add_geom(name="arm_d405", pos=[(0.010 + x_end) / 2, y_bar, z_leg], size=[abs(x_end - 0.010) / 2 + 0.005, 0.005, 0.003], mass=0.012, **_kw(ALU, **V))   # along x beside the camera's west face (back on the bought stacks, forward on the servo tool)
    z_pl0, z_pl1 = z_leg - 0.003, d405_tool[2] + 0.012
    tool.add_geom(name="arm_d405_plate", pos=[d405_tool[0], d405_tool[1] - 0.0225, (z_pl0 + z_pl1) / 2], size=[0.015, 0.0015, (z_pl1 - z_pl0) / 2], mass=0.010, **_kw(ALU, **V))   # side plate: bolted through its top into the bar, 2 x M2 into the camera's side
    jaws.add_geom(name="d405_g", pos=list(d405_jaw), quat=d405_look, size=[0.021, 0.021, 0.0115], mass=0.058, **_kw([0.12, 0.12, 0.13, 1], **V))   # 42 x 42 x 23 mm, 58 g, facing its view
    jaws.add_camera(name="wrist_d405", pos=list(d405_jaw), quat=d405_look, fovy=58)
    for g in spec.geoms:   # printed finger parts weigh what their volume says (PAHT-CF 1.2 g/cc; the pad with its silicone face 1.4), not the round numbers they were sketched with
        if g.name in ("plate_lo", "leg_e", "plate_up", "pad_arm", "pad_leg_e", "pad_g", "plate_up_tongue", "plate_up_web"):
            g.mass = (1400.0 if g.name == "pad_g" else 1200.0) * 8.0 * float(g.size[0]) * float(g.size[1]) * float(g.size[2])
    if FIXED_CAMS:   # the macro cameras and the light stand on the bench (add_fixed_cameras); only the D405 rides the wrist
        G.get("fixed_lower") or spec.add_equality(name="jaws_couple", type=mujoco.mjtEq.mjEQ_JOINT, name1="shoe_slide", name2="pad_slide", data=[0, 1, 0, 0, 0] + [0] * 6, objtype=mujoco.mjtObj.mjOBJ_JOINT, solref=[0.001, 1.0])
        return tool
    half_in = CM["size"][0] / 2; cs = CM["size"]
    cam_bars = {}
    for sgn, nm in ((1, "macro_e"), (-1, "macro_w")):
        cam_jaw = (-(reach * math.sin(pitch)), sgn * reach * math.cos(pitch), look[2] - 0.004)   # 25 deg down from each side, level with the tips along z
        cam_positions[nm] = cam_jaw
        cam_tool = (px - cam_jaw[2], cam_jaw[1], zj + cam_jaw[0])
        # the camera looks inward and down, so its lens points at the stack: the arm comes in UNDER the lens and carries the camera by its bottom face
        cam_bottom = cam_tool[2] + half_in * math.cos(pitch) + cs[2] / 2 * math.sin(pitch)       # lowest point of the tilted body, tool z
        z_cbar = cam_bottom + 0.002 + 0.003                                                       # bar centre: 2 mm under the body, 6 mm thick
        cam_bars[nm] = z_cbar
        y_out = (FINGER_OFF + G["body_d"] / 2 + G["plate_t"] + BRACKET_T + 0.005) if sgn > 0 else -0.020   # just outboard of the leg's end
        x_link0, x_link1 = min(cam_tool[0] - 0.005, -BRACKET_W / 2 + 0.010), -BRACKET_W / 2 + 0.010
        tool.add_geom(name=f"arm_{nm}_post", pos=[x_link1 - 0.005, y_out, (Ls + BRACKET_T + z_cbar - 0.003) / 2], size=[0.005, 0.005, (z_cbar - 0.003 - Ls - BRACKET_T) / 2 + 0.0005], mass=0.006, **_kw(ALU, **V))   # post from the leg's underside down to the bar level
        tool.add_geom(name=f"arm_{nm}_link", pos=[(x_link0 + x_link1) / 2, y_out, z_cbar], size=[abs(x_link1 - x_link0) / 2, 0.005, 0.003], mass=0.006, **_kw(ALU, **V))
        tool.add_geom(name=f"arm_{nm}", pos=[cam_tool[0], (y_out + cam_tool[1]) / 2, z_cbar], size=[0.005, abs(cam_tool[1] - y_out) / 2 + 0.005, 0.003], mass=0.012, **_kw(ALU, **V))   # out to the camera's centre, under the lens
        tool.add_geom(name=f"arm_{nm}_block", pos=[cam_tool[0], cam_tool[1], (z_cbar - 0.003 + cam_bottom) / 2], size=[0.010, 0.010, (z_cbar - 0.003 - cam_bottom) / 2 + 0.0005], mass=0.004, **_kw(ALU, **V))   # mounting block between the bar and the camera's bottom face (its M2/M3 holes)
    ld, ll = CM["lens_dl"]
    for sgn, nm in ((1, "macro_e"), (-1, "macro_w")):
        cpos = np.array(cam_positions[nm]); q = _look_quat(cpos, look, up=(-1, 0, 0))
        cam = jaws.add_body(name=f"{nm}_body", pos=cpos.tolist(), quat=q)       # body frame: -z along the line of sight (camera convention)
        cam.add_geom(name=f"{nm}_g", size=[cs[0] / 2, cs[1] / 2, cs[2] / 2], mass=CM["mass"], **_kw([0.12, 0.12, 0.13, 1], contype=0, conaffinity=0))
        lk = _kw([0.05, 0.05, 0.06, 1], contype=0, conaffinity=0); lk["type"] = mujoco.mjtGeom.mjGEOM_CYLINDER
        cam.add_geom(name=f"{nm}_lens", pos=[0, 0, -(cs[2] / 2 + ll / 2)], size=[ld / 2, ll / 2, 0], mass=CM["lens_mass"], **lk)   # lens barrel ahead of the body, along the line of sight
        cam.add_camera(name=nm, pos=[0, 0, -(cs[2] / 2 + ll)], quat=[1, 0, 0, 0], fovy=CM["fovy"])
    # the blue bar light: low on the west side, aimed at the mouth from about 55 mm, hung from the west camera arm by a short drop
    lw, lh, ldp = CM["light_size"]; z_wbar = cam_bars["macro_w"]
    if lw > 0.060:
        # a bar light this size (the LR Mate's LM75, 80 mm, 128 g) is a fixture light: it stands on the pallet 60 mm west of the mouth, 30 mm up, aimed at it
        CM["light_on_pallet"] = True
        return tool
    lpos = np.array([-(zj - (z_wbar + 0.003 + 0.004 + ldp / 2)), -0.050, tipz - 0.006])   # light centre: its top 4 mm under the west bar
    lq = _look_quat(lpos, [0, 0, tipz], up=(-1, 0, 0))
    jaws.add_geom(name="light_g", pos=lpos.tolist(), quat=lq, size=[lw / 2, lh / 2, ldp / 2], mass=CM["light_mass"], **_kw([0.10, 0.35, 0.85, 1], contype=0, conaffinity=0))
    # light on its own bars off the west link: a bar out to the light's y, a drop to the light's top
    l_tool = (px - lpos[2], lpos[1], zj + lpos[0])
    tool.add_geom(name="arm_light_drop", pos=[l_tool[0], l_tool[1], (z_wbar + 0.003 + l_tool[2] - ldp / 2) / 2], size=[0.005, 0.003, abs(l_tool[2] - ldp / 2 - z_wbar - 0.003) / 2 + 0.0005], mass=0.003, **_kw(ALU, contype=0, conaffinity=0))   # short drop from the west camera bar to the light
    G.get("fixed_lower") or spec.add_equality(name="jaws_couple", type=mujoco.mjtEq.mjEQ_JOINT, name1="shoe_slide", name2="pad_slide", data=[0, 1, 0, 0, 0] + [0] * 6, objtype=mujoco.mjtObj.mjOBJ_JOINT, solref=[0.001, 1.0])   # both jaws move together, as on a rack-driven parallel gripper
    return tool


def add_fixed_cameras(spec, arm):
    """Macro cameras and the bar light on the bench: two cameras on 12.7 mm posts at the lens's working distance, looking at the mouth from
    the cable side, and the bar light on its own post. Geoms do not collide in the physics (like the other fixtures) but the clearance audit
    treats them as obstacles, so an arm or tool sweeping through them is reported. The camera names macro_e and macro_w are kept so every
    snapshot, video and tour still works."""
    if not FIXED_CAMS:
        return
    CM = CAMERAS[arm]; o = connector_origin(arm); look = o + np.array([0.0, 0.003, 0.004]); w = spec.worldbody
    cs = CM["size"]; ld, ll = CM["lens_dl"]; z_bench = BREADBOARD[2]
    def direction(az, el):
        a, e = math.radians(az), math.radians(el)
        return np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])
    for nm, az in (("macro_e", CM["az_deg"][0]), ("macro_w", CM["az_deg"][1])):
        d = direction(az, CM["el_deg"]); body_c = look + (CM["wd"] + ll + cs[2] / 2) * d
        q = _look_quat(body_c, look, up=(0, 0, 1))
        cam = w.add_body(name=f"fixcam_{nm}_body", pos=body_c.tolist(), quat=q)
        cam.add_geom(name=f"fixcam_{nm}_g", size=[cs[0] / 2, cs[1] / 2, cs[2] / 2], mass=CM["mass"], **_kw([0.12, 0.12, 0.13, 1], contype=0, conaffinity=0))
        lk = _kw([0.05, 0.05, 0.06, 1], contype=0, conaffinity=0); lk["type"] = mujoco.mjtGeom.mjGEOM_CYLINDER
        cam.add_geom(name=f"fixcam_{nm}_lens", pos=[0, 0, -(cs[2] / 2 + ll / 2)], size=[ld / 2, ll / 2, 0], mass=CM["lens_mass"], **lk)
        cam.add_camera(name=nm, pos=[0, 0, -(cs[2] / 2 + ll)], quat=[1, 0, 0, 0], fovy=CM["fovy"])
        # the tilt block is flush on the body's bottom face (in the body's own tilted frame), and the post stands under the block's lowest
        # point, on the pallet if the foot lands on it, else on the breadboard; nothing is drawn through anything
        bh = 0.004; cam.add_geom(name=f"fixcam_{nm}_block", pos=[0, -(cs[1] / 2 + bh), 0], size=[0.012, bh, 0.010], **_kw(ALU, contype=0, conaffinity=0))
        Rb = np.zeros(9); mujoco.mju_quat2Mat(Rb, np.array(q)); Rb = Rb.reshape(3, 3)
        blk_c = body_c + Rb @ np.array([0, -(cs[1] / 2 + bh), 0]); blk_low = blk_c[2] - (abs(Rb[2, 0]) * 0.012 + abs(Rb[2, 1]) * bh + abs(Rb[2, 2]) * 0.010)
        ppx, ppy, ppz = pallet_pose(arm); on_pallet = abs(blk_c[0] - ppx) < PALLET[0] / 2 and abs(blk_c[1] - ppy) < PALLET[1] / 2
        z_foot = ppz + PALLET[2] if on_pallet else z_bench
        w.add_geom(name=f"fixcam_{nm}_post", type=mujoco.mjtGeom.mjGEOM_CYLINDER, pos=[blk_c[0], blk_c[1], (z_foot + blk_low) / 2], size=[0.00635, (blk_low - z_foot) / 2], rgba=[0.75, 0.75, 0.77, 1], contype=0, conaffinity=0)
    lw, lh, ldp = CM["light_size"]; d = direction(CM["light_az_deg"], CM["light_el_deg"]); lp = look + CM["light_wd"] * d
    lq = _look_quat(lp, look, up=(0, 0, 1))
    w.add_geom(name="light_g", pos=lp.tolist(), quat=lq, size=[lw / 2, lh / 2, ldp / 2], mass=CM["light_mass"], **_kw([0.10, 0.35, 0.85, 1], contype=0, conaffinity=0))
    Rl = np.zeros(9); mujoco.mju_quat2Mat(Rl, np.array(lq)); Rl = Rl.reshape(3, 3); l_low = lp[2] - (abs(Rl[2, 0]) * lw / 2 + abs(Rl[2, 1]) * lh / 2 + abs(Rl[2, 2]) * ldp / 2)
    ppx, ppy, ppz = pallet_pose(arm); z_foot = ppz + PALLET[2] if (abs(lp[0] - ppx) < PALLET[0] / 2 and abs(lp[1] - ppy) < PALLET[1] / 2) else z_bench
    w.add_geom(name="light_post", type=mujoco.mjtGeom.mjGEOM_CYLINDER, pos=[lp[0], lp[1], (z_foot + l_low) / 2], size=[0.00635, (l_low - z_foot) / 2], rgba=[0.75, 0.75, 0.77, 1], contype=0, conaffinity=0)


_MESH_FIT = {}
def _mesh_fit(name, path, target_dims):
    """Which of the 24 axis-aligned orientations lays a CAD mesh on its modelled slab, and where its bounding-box centre lands: MuJoCo
    recentres and realigns meshes internally, so this is measured on a throwaway compile rather than assumed from the file's axes.
    Returns (quat, offset) so that geom pos = slab centre + offset, quat = quat puts the mesh's bounding box on the slab."""
    if name in _MESH_FIT:
        return _MESH_FIT[name]
    import itertools
    best = None
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            R = np.zeros((3, 3))
            for i, (j, sg) in enumerate(zip(perm, signs)):
                R[i, j] = sg
            if np.linalg.det(R) < 0:
                continue
            q = np.zeros(4); mujoco.mju_mat2Quat(q, R.flatten())
            sp = mujoco.MjSpec(); sp.add_mesh(name=name, file=path); sp.worldbody.add_geom(name="m", meshname=name, type=mujoco.mjtGeom.mjGEOM_MESH, quat=q.tolist())
            m = sp.compile(); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
            V = m.mesh_vert[m.mesh_vertadr[0]:m.mesh_vertadr[0] + m.mesh_vertnum[0]]; W = d.geom_xpos[0] + V @ d.geom_xmat[0].reshape(3, 3).T
            dims = W.max(0) - W.min(0); err = float(np.abs(dims - np.asarray(target_dims)).sum())
            if best is None or err < best[0]:
                best = (err, q.tolist(), (-(W.max(0) + W.min(0)) / 2).tolist(), dims)
    _MESH_FIT[name] = (best[1], best[2])
    return _MESH_FIT[name]


_ARM = ["meca500"]
def build(arm="meca500", case="laptop", tool=None):
    _CASE[0] = case; _ARM[0] = arm
    spec = mujoco.MjSpec()
    spec.compiler.degree = False          # all joint ranges and spring references here are radians (the MJCF default is degrees)
    spec.option.timestep = DT; spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.option.noslip_iterations = 5; spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.visual.global_.offwidth = 1280; spec.visual.global_.offheight = 960; spec.visual.map.znear = 0.0005
    add_bench(spec, arm); add_pallet(spec, arm); add_fixed_cameras(spec, arm)
    flange = add_arm(spec, arm); add_tool(spec, flange, arm, tool)
    spec.add_equality(name="grasp", type=mujoco.mjtEq.mjEQ_WELD, objtype=mujoco.mjtObj.mjOBJ_BODY, name1="tool", name2="ffc_stiffener", active=False, solref=[0.0002, 1.0])   # on the jaws body (the fingers), so a force sensor between jaws and tool sees the cable's pull balanced (2026-10-13)   # a 5 N rubber-pad grip: about 15000 N/m, 0.07 mm per newton
    # pad servo: 20 N/mm so 0.3 mm of commanded overshoot gives a 6 N grip, as the MEGP 25LS does at its rated force
    spec.add_actuator(name="pad_act", target="pad_slide", trntype=mujoco.mjtTrn.mjTRN_JOINT, gaintype=mujoco.mjtGain.mjGAIN_FIXED, biastype=mujoco.mjtBias.mjBIAS_AFFINE, gainprm=[20000] + [0] * 9, biasprm=[0, -20000, -60] + [0] * 7, ctrlrange=[0, upper_travel(arm, tool) + 0.001], forcerange=[-25, 25])   # position servo: 20 N/mm, 25 N cap (biastype must be AFFINE or the spring term is silently ignored and this is a bare motor)
    fsite = "cell" if not any(pid in ("nano17", "nano25", "mini40") for pid, *_ in STACKS[TOOL_INFO[tool or DEFAULT_TOOL[arm]]["stack"]]) else "ft"   # the servo tool measures at the finger
    spec.add_sensor(name="ft_force", type=mujoco.mjtSensor.mjSENS_FORCE, objtype=mujoco.mjtObj.mjOBJ_SITE, objname=fsite)
    spec.add_sensor(name="ft_torque", type=mujoco.mjtSensor.mjSENS_TORQUE, objtype=mujoco.mjtObj.mjOBJ_SITE, objname=fsite)
    spec.add_sensor(name="latch_angle", type=mujoco.mjtSensor.mjSENS_JOINTPOS, objtype=mujoco.mjtObj.mjOBJ_JOINT, objname="zif_latch")
    return spec


# ---------------------------------------------------------------- helpers -----------------------------------------------------
def ik(model, data, site_name, joint_names, target_pos, R_target=None, iters=400, step=0.06, orient_site="ft"):
    """Position of site_name to target_pos; orientation of orient_site (on the tool body, so the bent jaw frame does not confuse it) to R_target."""
    sid = model.site(site_name).id; oid = model.site(orient_site).id if R_target is not None else sid
    dofs = [model.joint(j).dofadr[0] for j in joint_names]; qadr = [model.joint(j).qposadr[0] for j in joint_names]
    jacp = np.zeros((3, model.nv)); jacr = np.zeros((3, model.nv))
    for _ in range(iters):
        mujoco.mj_kinematics(model, data); mujoco.mj_comPos(model, data)
        p = data.site_xpos[sid]; R = data.site_xmat[oid].reshape(3, 3)
        e_p = np.asarray(target_pos) - p
        if R_target is not None:
            dR = R_target @ R.T; e_r = 0.5 * np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]])
        else:
            e_r = np.zeros(3)
        if np.linalg.norm(e_p) < 1e-5 and np.linalg.norm(e_r) < 1e-3:
            break
        mujoco.mj_jacSite(model, data, jacp, jacr, sid)
        if R_target is not None and oid != sid:
            jacr2 = np.zeros((3, model.nv)); mujoco.mj_jacSite(model, data, np.zeros((3, model.nv)), jacr2, oid); jacr = jacr2
        J = np.vstack([jacp[:, dofs], jacr[:, dofs]]); e = np.concatenate([e_p, e_r * 0.4])
        dq = J.T @ np.linalg.solve(J @ J.T + 1e-6 * np.eye(6), e); dq = np.clip(dq, -step, step)
        for k, (a, j) in enumerate(zip(qadr, joint_names)):
            jid = model.joint(j).id
            data.qpos[a] += dq[k]
            if model.jnt_limited[jid]:
                data.qpos[a] = float(np.clip(data.qpos[a], model.jnt_range[jid][0] + 0.02, model.jnt_range[jid][1] - 0.02))
    mujoco.mj_kinematics(model, data)
    return float(np.linalg.norm(np.asarray(target_pos) - data.site_xpos[sid]))


def tool_R(dip_deg=0.0):
    """Tool orientation: body vertical (tool z down), jaws forward along -y toward the connector, jaw closing direction down.
    dip_deg rotates the whole tool about world x so the jaw tips (and the blade) dip toward the board."""
    R_vert = np.column_stack([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
    d = math.radians(dip_deg)
    R_x = np.array([[1, 0, 0], [0, math.cos(d), -math.sin(d)], [0, math.sin(d), math.cos(d)]])
    return R_x @ R_vert


level_tool_R = tool_R
