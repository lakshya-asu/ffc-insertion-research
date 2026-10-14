"""Three phases in the FFC/ZIF twin with a scripted expert: open the latch with the blade, slide the stiffened tip under it,
press the latch closed, verify. Prints the force signature and the verification, writes frames and log to out/ffc_zif/<arm>/.
usage: .venv/bin/python scripts/ffc_zif_demo.py [meca500|lrmate200id]"""
import math, sys, json
from pathlib import Path
import mujoco, numpy as np
from PIL import Image
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from cell_twin import ffc_zif as F

import os
ARM = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("FFC_ARM", "meca500")
CASE = os.environ.get("FFC_CASE", "laptop"); TOOL = os.environ.get("FFC_TOOL", F.DEFAULT_TOOL[ARM])
VARIANT = f"{ARM}-{TOOL}-{CASE}"
OUT = Path(__file__).resolve().parent.parent / "out" / "ffc_zif" / VARIANT; OUT.mkdir(parents=True, exist_ok=True)
print("variant:", VARIANT, "|", F.TOOL_INFO[TOOL]["label"], "|", F.CASES[CASE])
spec = F.build(ARM, CASE, TOOL); model = spec.compile(); data = mujoco.MjData(model)
info = F.ARM_INFO[ARM]; J = [info["prefix"] + "_" + j for j in info["joints"]]
for j, deg in zip(J, info["home"]):
    data.joint(j).qpos[0] = math.radians(deg)
import os
MEMORY = os.environ.get("FFC_CABLE_MEMORY", "1") != "0"     # FFC_CABLE_MEMORY=0: a flat, slack cable laid on the rest (Lakshya's platform presentation)
for i, a in enumerate(F.CABLE_REST):                     # bend memory (laptop case): start in the rest shape and make the springs hold it
    if CASE == "laptop":
        data.joint(f"ffc_bend{i}").qpos[0] = a
        model.qpos_spring[model.joint(f"ffc_bend{i}").qposadr[0]] = a * F.CABLE_MEMORY[i] if MEMORY else 0.0
print("cable bend memory:", "on" if MEMORY else "off")
mujoco.mj_forward(model, data)
ren = mujoco.Renderer(model, 480, 640); log = []
pad_q = model.joint("pad_slide").qposadr[0]; latch = model.joint("zif_latch").qposadr[0]; eq = model.equality("grasp").id
HOLD = {j: data.joint(j).qpos[0] for j in J}
FORCE_STOP = 4.0          # N: a guarded move stops here (the real loop uses the wrist sensor the same way)


def snap(tag):
    for cam in ("overview", "macro_e", "macro_w", "wrist_d405"):
        ren.update_scene(data, camera=cam); Image.fromarray(ren.render()).save(OUT / f"{tag}_{cam}.png")


PAD_CTRL = [0.0]
UPT, LOWT, JAW_SPEED = F.upper_travel(ARM, TOOL), F.lower_travel(ARM, TOOL), F.jaw_speed(ARM, TOOL)   # servo tool: the upper jaw does the whole stroke, the lower is fixed
DEBUG_LATCH = [False]
TRAJ, TRAJ_T, PHASE_AT, PHASE_LBL = [], [], [], []     # every 40th step (8 ms) so a replay can be watched in the viewer or rendered to video
CUR_PHASE = ["start"]
def phase(label):
    CUR_PHASE[0] = label; PHASE_AT.append(len(TRAJ)); PHASE_LBL.append(label)
def settle(steps, pad_target=None, q_from=None, q_to=None, pad_from=None):
    """Step the physics. The arm is position-held; when q_from/q_to are given the hold ramps between them with a smooth
    (cosine) profile so the tool moves at a finite speed. pad_from lets the pad command ramp too."""
    if pad_target is not None and pad_from is None:
        PAD_CTRL[0] = pad_target
    for k in range(steps):
        if q_to is not None:
            a = 0.5 - 0.5 * math.cos(math.pi * min(1.0, (k + 1) / steps))
            for j in J:
                HOLD[j] = q_from[j] + (q_to[j] - q_from[j]) * a
        if pad_from is not None:
            PAD_CTRL[0] = pad_from + (pad_target - pad_from) * min(1.0, (k + 1) / steps)
        data.ctrl[model.actuator("pad_act").id] = PAD_CTRL[0]
        mujoco.mj_step(model, data)
        for j in J:
            data.joint(j).qpos[0] = HOLD[j]; data.joint(j).qvel[0] = 0.0
        if k % 80 == 0:
            TRAJ.append(data.qpos.copy()); TRAJ_T.append(data.time)
        if DEBUG_LATCH[0] and k % 400 == 0:
            lid_c = [c for c in contacts(8).split("; ") if "zif_lid_plate" in c]
            print(f"      t={data.time:.2f} latch {math.degrees(data.qpos[latch]):6.1f}  lip tip y {(data.site('blade_tip').xpos[1]-data.body('zif_lid').xpos[1])*1e3:+.2f} z {(data.site('blade_tip').xpos[2]-data.body('zif_lid').xpos[2])*1e3:+.2f}  {lid_c[:2]}")
    mujoco.mj_forward(model, data)


def pad_to(target, seconds):
    """ramp the pad command over `seconds` (closing on the cable or opening), physics running"""
    settle(int(seconds / F.DT), pad_target=target, pad_from=PAD_CTRL[0])


def contacts(top=6):
    out = []
    f6 = np.zeros(6)
    for i in range(data.ncon):
        c = data.contact[i]; mujoco.mj_contactForce(model, data, i, f6)
        out.append((abs(f6[0]), mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom1), mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom2)))
    out.sort(reverse=True)
    return "; ".join(f"{a}<->{b} {f:.1f}N" for f, a, b in out[:top])


def cable_state(label):
    o_ = data.body("zif").xpos; tip = (data.site("ffc_tip").xpos - o_) * 1e3; root = (data.body("ffc_stiffener").xpos - o_) * 1e3
    cs = [c for c in contacts(12).split("; ") if "ffc_stiffener" in c or "ffc_seg4" in c]
    feed = f" feed {data.joint('ffc_feed').qpos[0]*1e3:+.1f}" if CASE == "platform" else ""
    print(f"  [pad q {data.qpos[pad_q]*1e3:.2f}{feed}] cable after {label}: tip y {tip[1]:+.1f} z {tip[2]:+.2f} | root y {root[1]:+.1f} z {root[2]:+.2f} | {cs[:4]}")
    if label in ("p2 above mouth", "p2 at mouth"):
        print(f"      all contacts: {contacts(8)}")


def record(label, err=0.0):
    f = data.sensor("ft_force").data.copy()
    log.append({"phase": label, "ik_err_mm": err * 1e3, "F": [round(float(x), 3) for x in f], "latch_deg": round(math.degrees(data.qpos[latch]), 2),
                "tip_mm": [round(float(x) * 1e3, 3) for x in data.site("ffc_tip").xpos]})
    return float(np.linalg.norm(f))


SEEDS = [[math.radians(d) for d in info["home"]]] + [[math.radians(d) + (0.3 if k == 4 else 0.0) * sgn for k, d in enumerate(info["home"])] for sgn in (1, -1)]
def probe_dip_max():
    """How far this arm can tilt the tool nose-down at the connector: the largest dip whose IK solves from the home pose."""
    q0 = {j: data.joint(j).qpos[0] for j in J}; best = 0.0
    hinge0 = data.body("zif_lid").xpos.copy()
    for dip in range(0, 75, 5):
        for j, deg in zip(J, info["home"]):
            data.joint(j).qpos[0] = math.radians(deg)
        e = F.ik(model, data, "blade_tip", J, hinge0 + np.array([0, 0.002, 0.003]), F.tool_R(dip))
        if e < 0.001:
            best = float(dip)
    for j in J:
        data.joint(j).qpos[0] = q0[j]
    mujoco.mj_forward(model, data); return best
DIP_MAX = min(probe_dip_max(), 20.0)     # tilt cap: keeps the open pad's rear corner under the floating cable during the sweep
print(f"tool tilt reachable at the connector: {DIP_MAX:.0f} deg (used as the sweep cap)")
IK_OK = [True]
def goto(site, target, R, steps=None, pad_target=None, label="", speed=0.03, min_steps=300):
    """Move site to target with orientation R at `speed` (m/s of the site), joints interpolated; steps overrides the duration."""
    q0 = {j: data.joint(j).qpos[0] for j in J}; p_start = data.site(site).xpos.copy()
    def solve():
        e = F.ik(model, data, site, J, target, R)
        Rft = data.site("ft").xmat.reshape(3, 3); ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(Rft.T @ R) - 1) / 2))))
        return e if ang < 0.6 else max(e, 0.01)        # an orientation miss over 0.6 deg counts as unreachable: 0.2 mm over the shoe length
    err = solve()
    if err > 0.001:                      # retry from a few seed configurations (the local IK can wedge against a joint limit)
        for seed in SEEDS:
            for j, v in zip(J, seed):
                data.joint(j).qpos[0] = v
            err = solve()
            if err <= 0.001:
                break
    if err > 0.001:                      # unreachable: stay where we are instead of parking the arm in a bad configuration
        for j in J:
            data.joint(j).qpos[0] = q0[j]
        mujoco.mj_forward(model, data); IK_OK[0] = False
        print(f"  {label}: unreachable (ik err {err*1e3:.1f} mm), holding")
        settle((steps or 600) // 3, pad_target); return record(label + " (unreachable)", err)
    IK_OK[0] = True
    q1 = {j: data.joint(j).qpos[0] for j in J}
    for j in J:
        data.joint(j).qpos[0] = q0[j]
    mujoco.mj_forward(model, data)
    if steps is None:
        steps = max(min_steps, int(np.linalg.norm(np.asarray(target) - p_start) / speed / F.DT))
    settle(steps, pad_target, q_from=dict(q0), q_to=q1); return record(label, err)


ZERO_GUARD = os.environ.get("FFC_ZERO_GUARD", "1") == "1"   # 2026-10-14: bias the wrist reading at the start of each guarded move, as the real controller does


def guarded(site, p0, p1, R, n, steps=None, label="", stop=FORCE_STOP, speed=0.003):
    """Move in n segments at `speed` and stop when the wrist force, measured from its value at the start of the move, exceeds `stop`
    (returns the fraction completed). FFC_ZERO_GUARD=0 restores the raw-magnitude guard used before 2026-10-14."""
    f0 = data.sensor("ft_force").data.copy() if ZERO_GUARD else 0.0
    for k in range(1, n + 1):
        goto(site, p0 + (p1 - p0) * k / n, R, steps=steps, label=f"{label} {k}/{n}", speed=speed, min_steps=100)
        fmag = float(np.linalg.norm(data.sensor("ft_force").data - f0))
        if fmag > stop:
            print(f"  {label}: force {fmag:.1f} N at step {k}/{n}, stopping [{contacts()}]"); return k / n
    return 1.0


def stiff_R():
    return data.xmat[model.body("ffc_stiffener").id].reshape(3, 3).copy()


o = data.body("zif").xpos.copy()   # connector origin, on the board surface at the mouth line
mouth = data.site("zif_mouth").xpos.copy(); seat = data.site("zif_seat").xpos.copy(); lip = data.site("zif_lip").xpos.copy(); hinge = data.body("zif_lid").xpos.copy()
lid_under = o[2] + F.SLOT_FLOOR + F.LATCH_CLAMP_GAP
print("board z", round(o[2] * 1e3, 2), "mm; lid underside", round(lid_under * 1e3, 2), "mm; lip y", round(lip[1] * 1e3, 2), "mm; hinge", np.round(hinge * 1e3, 2))

# ---- phase 1: open the latch. Hook the blade tip under the lid's front overhang, then sweep the tip along the lid's arc while
# rotating the tool with the lid, so only the tip rides under it (a fingernail). Force guarded throughout.
def lid_sweep(th0, th1, n, side, label, stop=20.0):
    """side = -1 rides under the lid plane (opening), +1 rides on top of it (closing)."""
    for th in np.linspace(th0, th1, n):
        t = math.radians(th)
        under = F.LID_T - F.LID_AXIS_BELOW_TOP                       # the lid underside is this far below the hinge axis
        off = -(under + 0.0005) if side < 0 else (F.LID_AXIS_BELOW_TOP + 0.0005)
        pt = hinge + (F.LID_D - 0.0008) * np.array([0, math.cos(t), math.sin(t)]) + off * np.array([0, -math.sin(t), math.cos(t)])
        fmag = goto("blade_tip", pt, F.tool_R(min(10.0 + 0.6 * th, DIP_MAX)), label=f"{label} {th:.0f}", speed=0.010, min_steps=150)
        if not IK_OK[0]:
            print(f"  {label}: unreachable at {th:.0f} deg, handing over"); return
        if fmag > stop:
            print(f"  {label}: force {fmag:.1f} N at {th:.0f} deg, stopping [{contacts()}]"); return
R1 = F.tool_R(10.0)
phase("phase 1: hook the blade under the lid lip")
# come down over the connector (the cable floats 14 mm up, in front of it), then forward under the cable to just past the lid's front edge
# the jaws reach 30 mm behind the lip, and the cable floats 14 mm up in front of the mouth: come down well behind the connector,
# then travel forward at 5 mm height (over the closed lid, under the cable) until the lip is just past the lid's front edge
goto("blade_tip", np.array([o[0], hinge[1] - 0.016, o[2] + 0.035]), F.tool_R(0.0), label="p1 approach above", speed=0.04)   # 16 mm behind the hinge: clear board between the housing (6 mm) and the SO-DIMM slot (25 mm)
goto("blade_tip", np.array([o[0], hinge[1] - 0.016, o[2] + 0.0030]), F.tool_R(0.0), label="p1 approach down", speed=0.012); cable_state("p1 approach down")
goto("blade_tip", np.array([o[0], lip[1] + 0.0015, o[2] + 0.0030]), F.tool_R(0.0), label="p1 forward under the cable", speed=0.012); cable_state("p1 forward")   # level: shoe 0.8 mm over the closed lid, open pad top (5.5 mm up) 2.5 mm under the floating stiffener
goto("blade_tip", np.array([o[0], lip[1] + 0.0015, o[2] + 0.00035]), R1, label="p1 down to board", speed=0.003); cable_state("p1 down")   # dips to 10 deg on the way down
guarded("blade_tip", np.array([o[0], lip[1] + 0.0015, o[2] + 0.00035]), np.array([o[0], lip[1] - 0.0010, o[2] + 0.00035]), R1, n=5, label="p1 under overhang", stop=20, speed=0.003)
snap("p1_hooked")
phase("phase 1: lift the lid along its arc")
def lid_sweep_tracking(th_end, lead=4.0, label="p1 sweep", stop=25.0, max_steps=140):
    """Lift the lid by following where it actually is: each step places the lip tip under the lid at (measured angle + lead),
    so the lip never runs ahead of the lid and the lifting force stays small. The real cell does the same with the wrist sensor."""
    stall = 0; last = math.degrees(data.qpos[latch])
    for k in range(max_steps):
        th_now = math.degrees(data.qpos[latch])
        if th_now >= th_end:
            break
        th = min(th_now + lead, th_end)
        t = math.radians(th); under = F.LID_T - F.LID_AXIS_BELOW_TOP; off = -(under + 0.0005)
        pt = hinge + (F.LID_D - 0.0008) * np.array([0, math.cos(t), math.sin(t)]) + off * np.array([0, -math.sin(t), math.cos(t)])
        fmag = goto("blade_tip", pt, F.tool_R(min(10.0 + 0.6 * th, DIP_MAX)), label=f"{label} {th:.0f}", speed=0.008, min_steps=120)
        if not IK_OK[0] or fmag > stop:
            print(f"  {label}: {'unreachable' if not IK_OK[0] else f'force {fmag:.1f} N'} at {th_now:.0f} deg, stopping"); return
        stall = stall + 1 if math.degrees(data.qpos[latch]) < last + 0.3 else 0; last = math.degrees(data.qpos[latch])
        if stall >= 6:
            print(f"  {label}: lid stalled at {last:.1f} deg; |F| {fmag:.1f} N; contacts: {contacts(8)}")
            bt = (data.site("blade_tip").xpos - hinge) * 1e3; print(f"      lip tip rel hinge y {bt[1]:+.2f} z {bt[2]:+.2f}; commanded th {th:.1f}, dip {min(10.0 + 0.6 * th, DIP_MAX):.0f}")
            dof = model.joint("zif_latch").dofadr[0]
            print(f"      latch dof: qfrc_constraint {data.qfrc_constraint[dof]:+.5f} N m, qfrc_passive {data.qfrc_passive[dof]:+.5f}, qfrc_smooth {data.qfrc_smooth[dof]:+.5f}; jnt range {np.degrees(model.jnt_range[model.joint('zif_latch').id])}; frictionloss {model.dof_frictionloss[dof]}; damping {model.dof_damping[dof]}")
            rows = [(int(data.efc_type[i]), int(data.efc_id[i]), float(data.efc_force[i])) for i in range(data.nefc) if data.efc_type[i] in (0, 1, 2) and int(data.efc_id[i]) == dof]
            print(f"      efc rows on the latch dof (type 0=eq 1=frictiondof 2=limit): {rows}")
            lid_id = model.body('zif_lid').id; print(f"      lid cfrc_ext {np.round(data.cfrc_ext[lid_id], 4)}  lid xpos rel conn {np.round((data.xpos[lid_id]-data.body('zif').xpos)*1e3,2)}")
            return
lid_sweep_tracking(88.0)
print(f"  after sweep: latch {math.degrees(data.qpos[latch]):.1f} deg")
# the last stretch: push the lid's front face over the top with the blade nearly level
p0, p1 = np.array([o[0], hinge[1] + 0.0026, hinge[2] + 0.0020]), np.array([o[0], hinge[1] - 0.0007, hinge[2] + 0.0020])
DEBUG_LATCH[0] = os.environ.get("FFC_DEBUG_LATCH", "0") == "1"
for k in range(1, 31):                       # push the lid's front face over the top, in 0.11 mm steps, until it reaches its stop
    fmag = goto("blade_tip", p0 + (p1 - p0) * k / 30, F.tool_R(10.0), label=f"p1 push over {k}/30", speed=0.003, min_steps=100)
    if math.degrees(data.qpos[latch]) >= F.LATCH_OPEN_DEG - 5.0 or fmag > 20:
        break
DEBUG_LATCH[0] = False
print(f"after phase 1: latch {math.degrees(data.qpos[latch]):.1f} deg (target {F.LATCH_OPEN_DEG})")
phase("phase 1: blade retreats")
# the lip ends against the standing lid's front face with the floating cable 7 mm over the open jaws: backing out would drag the shoe through
# the lid, rising would push the open pad into the cable, so the tool leaves sideways east (the fingers' one leg moves away from the cable,
# the lip slides along the lid's face and clears its end after 24 mm), then rises
phase("phase 1: tool leaves sideways")
goto("blade_tip", data.site("blade_tip").xpos + np.array([0, 0.0025, 0.0]), F.tool_R(10.0), label="p1 lip off the lid face", speed=0.004)   # the push-over ends with the lip pressed on the lid: step 2.5 mm toward the mouth first, or the exit drags the lid past its stop
print(f"  lip off the lid: latch {math.degrees(data.qpos[latch]):.1f} deg")
goto("blade_tip", data.site("blade_tip").xpos + np.array([0.045, 0.0, 0.0]), F.tool_R(10.0), label="p1 retreat sideways east", speed=0.02)
print(f"  after the sideways exit: latch {math.degrees(data.qpos[latch]):.1f} deg")
goto("blade_tip", data.site("blade_tip").xpos + np.array([0, 0, 0.030]), F.tool_R(0.0), label="p1 retreat up", speed=0.03)
cable_state("phase 1")
snap("p1_open")

# ---- grasp: the shoe slides under the floating stiffener from its tip, parallel to it, until the jaw line is 6 mm behind the tip
tipP = data.site("ffc_tip").xpos.copy(); gripP = data.site("ffc_grip").xpos.copy()
u = gripP - tipP; u[0] = 0.0; u /= np.linalg.norm(u)                               # along the stiffener from tip to root (+y, with its droop)
pitch = max(-DIP_MAX, min(DIP_MAX, math.degrees(math.atan2(u[2], u[1]))))         # positive: the tip hangs lower than the root; the jaws dip to match
nrm = np.array([0.0, -u[2], u[1]])                                                 # the stiffener's upward normal
Rg = F.tool_R(pitch)
grip = gripP + 0.001 * u                                                            # jaw front line 6 mm behind the tip: the tip protrudes 6 mm
print(f"  stiffener pitch {pitch:+.1f} deg ({'tip below root' if pitch > 0 else 'tip above root'}); the jaws match it")
phase("grasp: jaws close on the stiffener")
pad_to(0.0, 0.3)                               # GripperOpen before the approach
back = grip - 0.014 * u                        # jaw line 14 mm short of the grasp line: the shoe's rear edge (7 mm behind it) stops 1 mm short of the tip
goto("tcp", grip - 0.036 * u + np.array([0, 0, 0.015]), Rg, label="grasp above", speed=0.04); cable_state("grasp above")
goto("tcp", back + np.array([0, 0, 0.015]), Rg, label="grasp forward, high", speed=0.015); cable_state("grasp forward")
goto("tcp", back - 0.0006 * nrm, Rg, label="grasp down beside the tip", speed=0.006); cable_state("grasp level")
goto("tcp", grip - 0.0006 * nrm, Rg, label="grasp under", speed=0.004)      # the shoe slides under the stiffener along it for 14 mm, its top 0.45 mm below the cable; the pad then closes the 3 mm gap
if LOWT == 0:   # fixed lower jaw: the shoe rises to the cable itself before the pad closes (the coupled jaws did this while closing); otherwise the pad bends the floating cable down over the shoe bar and the wrist cells read 3 to 6 N of cable preload
    goto("tcp", data.site("tcp").xpos + 0.0007 * nrm, Rg, label="grasp lift to the cable", speed=0.002); cable_state("grasp lift")   # 0.6 mm approach gap + 0.1 mm preload; a 1.5 mm lift swung the cable 0.56 mm back on the shoe and the lip met the standing lid at the end of the slide
print(f"  before closing: stiffener centre {(data.site('ffc_grip').xpos[2]-data.site('tcp').xpos[2])*1e3:+.3f} mm above the shoe top, pad underside {(F.PAD_OPEN - 2 * data.qpos[pad_q])*1e3:.2f} mm above it; tip ahead of the jaw line {(data.site('tcp').xpos[1]-data.site('ffc_tip').xpos[1])*1e3:.2f} mm; grip target z was {grip[2]*1e3:.2f}, grip site now {data.site('ffc_grip').xpos[2]*1e3:.2f}")
pad_to((UPT + LOWT - F.STIFF_T) / (2 if LOWT > 0 else 1) + 0.00025, 1.0)                                  # both jaws close over 1 s (1.5 mm each), ending 0.25 mm into the servo: about 5 N on the cable
settle(int(0.3 / F.DT))
print(f"pad closed to {data.qpos[pad_q]*1e3:.3f} mm of {(F.JAW_TRAVEL - F.STIFF_T / 2)*1e3:.3f} per jaw; stiffener centre above shoe top {(data.site('ffc_grip').xpos[2]-data.site('tcp').xpos[2])*1e3:+.3f} mm; pad force {abs(float(data.actuator_force[model.actuator('pad_act').id])):.1f} N")
tid, sid = int(model.eq_obj1id[eq]), model.body("ffc_stiffener").id   # the weld's own first body (jaws)
USE_WELD = os.environ.get("FFC_WELD", "1") == "1"      # a 6 N pinch on a 0.3 mm stiffener is rigid in practice; the weld is taken at the measured pose, no idealised jump
if USE_WELD:   # optional backup: weld the cable where it actually is (no idealised jump)
    T1 = np.eye(4); T1[:3, :3] = data.xmat[tid].reshape(3, 3); T1[:3, 3] = data.xpos[tid]
    T2 = np.eye(4); T2[:3, :3] = data.xmat[sid].reshape(3, 3); T2[:3, 3] = data.xpos[sid]
    rel = np.linalg.inv(T1) @ T2; q = np.empty(4); mujoco.mju_mat2Quat(q, rel[:3, :3].flatten())
    model.eq_data[eq, :3] = 0; model.eq_data[eq, 3:6] = rel[:3, 3]; model.eq_data[eq, 6:10] = q; data.eq_active[eq] = 1
    settle(500)
record("grasped"); snap("p2_grasped"); cable_state("grasped"); print(f"  pad q {data.qpos[pad_q]*1e3:.3f} mm, ctrl {PAD_CTRL[0]*1e3:.3f}, actuator force {float(data.actuator_force[model.actuator('pad_act').id]):+.2f} N, pad-cable contacts: {[c for c in contacts(12).split('; ') if 'pad_g' in c][:2]}")


# ---- phase 2: level the stiffener (its normal up, its length along +y), tip at the mouth, slide to the seat under force guard
# yaw correction: the pinch does not rotate the cable, so the stiffener's yaw about the vertical is measured (on the real cell by the two
# macro cameras, here from the twin) and the tool is turned by the same angle about world z so the stiffener runs along the slot axis
_axis = data.site("ffc_grip").xpos - data.site("ffc_tip").xpos
yaw = math.atan2(_axis[0], _axis[1])          # the stiffener's direction in the x-y plane relative to +y (the slot axis)
cy_, sy_ = math.cos(yaw), math.sin(yaw); Rz = np.array([[cy_, -sy_, 0], [sy_, cy_, 0], [0, 0, 1]])
tipP_in_tool = data.xmat[tid].reshape(3, 3).T @ (data.site("ffc_tip").xpos - data.site("tcp").xpos)
R_tool_des = Rz @ F.tool_R(0.0)                # jaws level, turned by the measured yaw so the clamped stiffener is parallel to the slot
print(f"  stiffener yaw {math.degrees(yaw):+.2f} deg about the vertical; tool turned to match (slot width allows about 0.6 deg)")
tip = data.site("ffc_tip").xpos.copy(); tcp = data.site("tcp").xpos.copy()
def tcp_for_tip(tip_target):
    # with the tool at R_tool_des the tip sits at tcp + R_tool_des @ (tip - tcp in tool coords)
    d_tool = data.xmat[tid].reshape(3, 3).T @ (data.site("ffc_tip").xpos - data.site("tcp").xpos)
    return tip_target - R_tool_des @ d_tool      # with the yawed R_tool_des the tip lands on target and the stiffener runs along the slot
pre = np.array([mouth[0], mouth[1] + 0.0025, o[2] + F.SLOT_FLOOR + 0.00017])   # tip 2.5 mm in front of the mouth, cable bottom 0.02 mm above the slot floor
phase("phase 2: tip to the mouth")
goto("tcp", data.site("tcp").xpos + np.array([0, 0, 0.003]), R_tool_des, label="p2 lift off the rest", speed=0.004); cable_state("p2 lift")
goto("tcp", tcp_for_tip(pre + np.array([0, 0, 0.008])), R_tool_des, label="p2 above mouth", speed=0.015); cable_state("p2 above mouth")
if CASE == "platform":
    print(f"  feed carriage y {data.joint('ffc_feed').qpos[0]*1e3:+.2f} mm")
goto("tcp", tcp_for_tip(pre), R_tool_des, label="p2 at mouth", speed=0.004); cable_state("p2 at mouth")
tip = data.site("ffc_tip").xpos.copy()
print(f"tip at mouth: lateral {(tip[0]-mouth[0])*1e3:+.3f} mm, height over slot centre {(tip[2]-mouth[2])*1e3:+.3f} mm, ahead {(tip[1]-mouth[1])*1e3:.2f} mm; tip above shoe top {(tip[2]-data.site('tcp').xpos[2])*1e3:+.3f} mm")
snap("p2_at_mouth")
goal_tip = np.array([seat[0], seat[1] + 0.0002, o[2] + F.SLOT_FLOOR + 0.00017])
phase("phase 2: slide to the seat, force guarded")
frac = guarded("tcp", tcp_for_tip(pre), tcp_for_tip(goal_tip), R_tool_des, n=14, label="p2 slide", speed=0.002)
tip = data.site("ffc_tip").xpos.copy()
print(f"after phase 2: depth {(mouth[1]-tip[1])*1e3:.2f} mm of {F.INSERT_DEPTH*1e3:.1f}, lateral {(tip[0]-mouth[0])*1e3:+.3f} mm, |F| {np.linalg.norm(log[-1]['F']):.2f} N")
snap("p2_seated")

# ---- release and retreat
phase("release the cable")
data.eq_active[eq] = 0
# both jaws open: the shoe would drop 1.5 mm into the board, so the arm lifts 1.5 mm in the same half second while the gripper opens
q_from = {j: data.joint(j).qpos[0] for j in J}; e_rel = F.ik(model, data, "tcp", J, data.site("tcp").xpos + np.array([0, 0, LOWT]), R_tool_des); q_to = {j: data.joint(j).qpos[0] for j in J}
for j in J:
    data.joint(j).qpos[0] = q_from[j]
mujoco.mj_forward(model, data)
settle(int(0.5 / F.DT), pad_target=0.0, q_from=q_from, q_to=q_to, pad_from=PAD_CTRL[0]); record("release: open + lift 1.5 mm", e_rel); settle(int(0.2 / F.DT)); cable_state("pad opened")
# the shoe is still under the seated cable: it leaves sideways (east, away from the fingers' one leg), at the same height, then the tool rises
goto("tcp", data.site("tcp").xpos + np.array([0.026, 0, 0]), R_tool_des, label="release: shoe slides out east", speed=0.008); cable_state("shoe out")
goto("tcp", data.site("tcp").xpos + np.array([0, 0.004, 0.012]), R_tool_des, label="release: up", speed=0.01)
tip_after = data.site("ffc_tip").xpos.copy(); print("after release: tip moved", np.round((tip_after - tip) * 1e3, 3), "mm"); RELEASE_CREEP = float((tip_after - tip)[1] * 1e3)

# ---- phase 3: close the latch. The standing lid is pushed forward from behind with the blade level (its rear face, 2.5 mm above
# the hinge), until it leans about 55 deg; then the flat shoe comes straight down on the lid and presses it shut (force guarded).
phase("phase 3: push the lid forward from behind")
R3 = F.tool_R(0.0)
zb = hinge[2] + 0.0020      # shoe underside 2.0 mm above the hinge: its rear edge meets the standing lid's back 2.0 to 2.5 mm up, 1.9 mm clear of the housing top
# The pusher is the shoe's rear edge with its wings (8.5 mm behind the lip tip, 0.5 mm steel, 38 mm wide), backed 1 mm behind by the closed
# pad's rear face (3 mm tall): the tool stays level and un-turned over the far side and drags that edge forward against the lid's back.
BAR = F.BLADE_L + F.SHOE_D
pad_to(0.0, max(0.3, (UPT + LOWT) / JAW_SPEED))   # jaws open: the shoe sits at its reference height (both jaws move, so the sites are exact only when open)
if math.degrees(data.qpos[latch]) > 60.0:     # only a standing lid is pushed from behind; a lid already leaning forward goes straight to the press
    y0, y1 = hinge[1] - 0.0060 - BAR, hinge[1] + 0.0035 - BAR
    goto("blade_tip", np.array([o[0], y0, zb + 0.014]), R3, label="p3 behind lid, above", speed=0.03)
    goto("blade_tip", np.array([o[0], y0, zb]), R3, label="p3 behind lid", speed=0.008)
    guarded("blade_tip", np.array([o[0], y0, zb]), np.array([o[0], y1, zb]), R3, n=16, label="p3 push forward", stop=12.0, speed=0.004)
    print(f"  after push forward: latch {math.degrees(data.qpos[latch]):.1f} deg")
    goto("blade_tip", data.site("blade_tip").xpos + np.array([0, 0, 0.014]), R3, label="p3 lift off", speed=0.02)
phase("phase 3: press the lid shut")
py = hinge[1] + (0.0005 + F.LID_D) / 2 + 0.0005                        # shoe centred over the lid; the lip ahead of it sits over the fixed roof
lid_top = o[2] + F.HOUSING_H + F.LID_PROUD
goto("shoe_bottom", np.array([o[0], py, lid_top + 0.008]), R3, label="p3 above lid", speed=0.02)
guarded("shoe_bottom", np.array([o[0], py, lid_top + 0.008]), np.array([o[0], py, lid_top + 0.0012]), R3, n=14, label="p3 press approach", stop=10.0, speed=0.005)
guarded("shoe_bottom", np.array([o[0], py, lid_top + 0.0012]), np.array([o[0], py, lid_top - 0.0001]), R3, n=65, label="p3 press", stop=10.0, speed=0.0005)   # 0.02 mm steps at 0.5 mm/s: stops within a step of 10 N
settle(int(0.3 / F.DT)); record("p3 hold")
print(f"after phase 3: latch {math.degrees(data.qpos[latch]):.2f} deg, |F| {np.linalg.norm(log[-1]['F']):.2f} N")
snap("p3_pressed")
goto("shoe_bottom", data.site("shoe_bottom").xpos + np.array([0, 0.004, 0.015]), R3, label="p3 retreat", speed=0.02)

# ---- verify
tip = data.site("ffc_tip").xpos.copy()
ver = {"latch_closed": bool(math.degrees(data.qpos[latch]) < 5.0), "latch_deg": round(math.degrees(data.qpos[latch]), 2),
       "depth_mm": round((mouth[1] - tip[1]) * 1e3, 3), "depth_ok": bool(abs((mouth[1] - tip[1]) - F.INSERT_DEPTH) < 0.0004),
       "lateral_mm": round((tip[0] - mouth[0]) * 1e3, 3), "lateral_ok": bool(abs(tip[0] - mouth[0]) < 0.0002), "phase2_fraction": frac}
print("VERIFY", json.dumps(ver))
snap("final")
phase("verify")
ver.update({"variant": VARIANT, "arm": ARM, "tool": TOOL, "case": CASE, "dip_max": DIP_MAX, "release_creep_mm": round(RELEASE_CREEP, 3), "slide_force_N": round(float(np.linalg.norm(log[-1]["F"])), 2) if False else None})
(OUT / "log.json").write_text(json.dumps({"verify": ver, "log": log}, indent=1))
np.savez(OUT / "traj.npz", qpos=np.array(TRAJ), t=np.array(TRAJ_T), phase_at=np.array(PHASE_AT), phase_lbl=np.array(PHASE_LBL))
print("trajectory", len(TRAJ), "frames ->", OUT / "traj.npz")
