"""Design review of each wrist stack, from the model's own numbers and the makers' limits: mass and moment on the flange, sensor ranges
against the task forces, gripper finger limits, finger stiffness, lip stress, reach, cables and electrics, and what the real laptop still
has to confirm. Writes out/ffc_zif/review.json for the blueprint page.  usage: .venv/bin/python scripts/ffc_zif_review.py"""
import json, math, sys
from pathlib import Path
import mujoco, numpy as np
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from cell_twin import ffc_zif as F

LIMITS = {
    "meca500": dict(payload_kg=0.5, payload_note="0.5 kg rated (1 kg at reduced speed), Mecademic", flange_moment_Nm=3.4, flange_moment_note="max moment on the flange 3.4 N m (Mecademic knowledge base)"),
    "lrmate200id": dict(payload_kg=7.0, payload_note="7 kg rated, FANUC", flange_moment_Nm=9.4, flange_moment_note="wrist J6 allowable moment 9.4 N m, inertia 0.15 kg m2 (FANUC LR Mate 200iD spec table; J4 and J5 16.6 N m)"),
}
SENSORS = {"nano17": dict(name="ATI Nano17 SI-50-0.5", fxy=50, fz=70, txy=0.5, tz=0.5, overload_fxy=250, overload_fz=480, res_N=1 / 80, note="SI-50-0.5 calibration: the press torque at 35 mm needs the 0.5 N m range; single-axis overload Fxy 250 N, Fz 480 N"),
           "nano25": dict(name="ATI Nano25 SI-125-3", fxy=125, fz=500, txy=3.0, tz=3.0, overload_fxy=2300, overload_fz=7300, res_N=1/48, note="SI-125-3 calibration (ATI Nano25 page, read 2026-10-14: SI-125-3 is not offered): force resolution 1/48 N Fxy, 1/16 N Fz; torque 1/1320 N m Txy, 1/2640 N m Tz"),
           "cell": dict(name="bar load cells (3 x TAL220B 5 kg: wrist pair + pad, 24-bit ADC at 1 kHz)", fxy=49, fz=49, txy=None, tz=None, overload_fxy=59, overload_fz=59, res_N=0.02, note="design 2026-10-13: TAL220B 5 kg, safe overload 120 percent; ADS122C04 at 1 kSPS about 0.07 N peak to peak, ADS1256 0.03 to 0.05 N; no torque channels"),
           "mini40": dict(name="ATI Mini40 SI-40-2", fxy=40, fz=120, txy=2.0, tz=2.0, overload_fxy=810, overload_fz=2400, res_N=0.01, note="SI-40-2 calibration (decided 2026-10-13): 0.01 N force, 0.0003 N m torque resolution")}
GRIP = {"megp25e": dict(name="MEGP 25E", max_finger_g=20, max_reach_mm=30, grip_N=40, Mx=1.50, My=0.50, Mz=0.42, Fx=70, stroke_per_jaw=3.0, min_gap=11.0, max_gap=17.0),
        "gep2006": dict(name="Zimmer GEP2006IL-00-B", max_finger_g=50, max_reach_mm=60, grip_N=40, Mx=None, My=None, Mz=None, Fx=None, stroke_per_jaw=6.0, min_gap=16.0, max_gap=28.0),
        "servo_v1": dict(name="Servo tool v1 (ours)", max_finger_g=60, max_reach_mm=40, grip_N=8, Mx=0.5, My=0.3, Mz=0.3, Fx=40, stroke_per_jaw=1.5, min_gap=None, max_gap=None),   # printed jaw carrier in the 6 mm guide slot: 0.5 N m is an 83 N couple on the 14 x 6 mm slot faces, 1 MPa on PAHT-CF (yield about 100 MPa); 20 N at 16 mm reach is 0.32 N m (ESTIMATE until built)
        "lehz16": dict(name="SMC LEHZ16LK2-6", max_finger_g=50, max_reach_mm=110, grip_N=8, Mx=0.68, My=0.68, Mz=1.36, Fx=98, stroke_per_jaw=3.0, min_gap=None, max_gap=None)}   # SMC LEH catalogue, static allowable moment per finger LEHZ16(L)K2-6: pitch Mp 0.68, yaw My 0.68, roll Mr 1.36 N m, vertical Fv 98 N (allowable load F = M / L); gripping point L graphed to 110 mm (catalogue p. 812: at L = 46 mm the compact LEHZ16L gives about 7 N at 100 percent and 5 N at 70 percent pushing force); finger mass limit not published (50 g ESTIMATE)   # finger mass and reach limits not published: ESTIMATE from the catalogue's allowable moment
# datasheets read 2026-10-13: Nano17 SI-50-0.5 force resolution 0.0125 N (1/80 N), torque 0.0001 N m, overload Txy 1.6 N m; MEGP 25E Fig. 2: |Mx| 1.50, |My| 0.50, |Mz| 0.42 N m, |Fx| 70 N, x2+z2 <= 30 mm, 20 g per finger (1.30 N m is the 25LS)
TASK = dict(pinch_N=5.0, slide_guard_N=4.0, seat_N=3.0, lid_lift_N=2.0, push_guard_N=12.0, press_N=10.0, press_guard_N=10.0, press_max_seen_N=20.0)
E_PAHT = 7.0e9     # Pa, PAHT-CF printed, along the beam (ESTIMATE; 6061 is 69e9)
E_AL = 69e9


def beam_deflection(F_N, L_m, b_m, h_m, E):
    I = b_m * h_m ** 3 / 12
    return F_N * L_m ** 3 / (3 * E * I)


def review(arm, tool):
    spec = F.build(arm, "laptop", tool); gm = {g.name: float(g.mass) for g in spec.geoms}; model = spec.compile(); data = mujoco.MjData(model); mujoco.mj_forward(model, data)
    ti = F.TOOL_INFO[tool]; G = F.GRIPPERS[ti["gripper"]]; L = LIMITS[arm]; gl = GRIP[ti["gripper"]]
    sensor = "cell" if tool.startswith("servo") else "nano25" if "nano25" in tool else "nano17" if "nano17" in tool else "mini40"; S = SENSORS[sensor]
    # everything below the flange: bodies tool, jaws, shoe, pad, macro_*_body; mass and centre of mass in the tool frame
    tool_bid = model.body("tool").id; below = [b for b in range(model.nbody) if model.body(b).name in ("tool", "jaws", "shoe", "pad", "macro_e_body", "macro_w_body")]
    m_tot = sum(float(model.body_mass[b]) for b in below)
    # subtree COM of the tool body (includes all children) in world, expressed relative to the flange
    com_w = data.subtree_com[tool_bid]; fl = data.xpos[tool_bid]; R = data.xmat[tool_bid].reshape(3, 3)
    com_t = R.T @ (com_w - fl)                                   # tool frame: z down the flange axis
    lever = math.hypot(com_t[0], com_t[1])                       # lateral offset of the COM from the flange axis
    m_static = m_tot * 9.81 * lever
    m_press = m_static + TASK["press_max_seen_N"] * math.hypot(float(model.site_pos[model.site("tcp").id][0]), 0) if False else m_static
    # press moment at the flange: the press force acts at the shoe, offset from the flange axis by the jaw-end distance + shoe depth
    jaws_pos = model.body_pos[model.body("jaws").id]; shoe_offset = abs(float(jaws_pos[0])) + 0.016   # flange axis to the shoe centre along x
    m_press = TASK["press_max_seen_N"] * shoe_offset
    rows = []
    def add(area, item, value, limit, ok, note=""):
        rows.append({"area": area, "item": item, "value": value, "limit": limit, "status": ok, "note": note})
    add("mass", "mass below the flange", f"{m_tot*1e3:.0f} g", f"{L['payload_kg']*1e3:.0f} g payload", "OK" if m_tot < L["payload_kg"] else "FAIL", L["payload_note"] + ". Includes cameras, arms, fingers; cables not included (add about 40 g).")
    add("mass", "centre of mass", f"{com_t[2]*1e3:.0f} mm below the flange, {lever*1e3:.0f} mm off axis", "", "OK", "static moment on the flange from the offset mass: %.2f N m" % m_static)
    add("loads", "moment at the flange during the 20 N latch press", f"{m_press:.2f} N m (20 N at {shoe_offset*1e3:.0f} mm)", f"{L['flange_moment_Nm']} N m", "OK" if m_press < 0.5 * L["flange_moment_Nm"] else "CHECK", L["flange_moment_note"])
    add("sensor", f"{S['name']} Fz range vs the latch press", f"press {TASK['press_N']} N (guard {TASK['press_guard_N']} N, worst seen {TASK['press_max_seen_N']} N)", f"Fz {S['fz']} N, overload {S['overload_fz']} N", "OK" if TASK["press_max_seen_N"] < S["fz"] else "CHECK", "the press runs at 30 to 60 percent of the calibrated range: fine for a stop, not for precision.")
    add("sensor", f"{S['name']} Fxy range vs the slide and the lid push", f"slide guard {TASK['slide_guard_N']} N, push guard {TASK['push_guard_N']} N", f"Fxy {S['fxy']} N", "OK" if TASK["push_guard_N"] < S["fxy"] else "CHECK", f"resolution {S['res_N']*1e3:.1f} mN: the 1 to 3 N seat signal is hundreds of counts.")
    if S["txy"]:
        add("sensor", "torque at the sensor from the press (10 N guard at the shoe offset; 20 N worst case in brackets)", f"{TASK['press_guard_N']*shoe_offset:.2f} N m ({TASK['press_max_seen_N']*shoe_offset:.2f})", f"Txy {S['txy']} N m", "OK" if TASK["press_guard_N"] * shoe_offset < S["txy"] else "CHECK", "the sensor sees the press as a torque because the shoe is ahead of the flange axis; the overload torque is 10x the range, so it cannot be damaged by the 10 N guard, but readings saturate above the range." if TASK["press_max_seen_N"] * shoe_offset >= S["txy"] else "")
        # horizontal tip forces (slide, lid push) act at the bottom of the whole stack, not at the sideways offset: measured from the twin (2026-10-13)
        ftp = data.site("ft").xpos; Rf = data.site_xmat[model.site("ft").id].reshape(3, 3); vt = Rf.T @ (data.site("tcp").xpos - ftp); axial = abs(float(vt[2]))
        t_slide, t_push_nom, t_push = TASK["slide_guard_N"] * axial, 7.0 * axial, TASK["push_guard_N"] * axial
        add("sensor", f"torque at the sensor from horizontal tip forces (tip {axial*1e3:.0f} mm below the sensor)", f"slide guard 4 N: {t_slide:.2f} N m; latch push 7 N: {t_push_nom:.2f} N m; push guard 12 N: {t_push:.2f} N m", f"Txy {S['txy']} N m",
            "OK" if t_push < 0.8 * S["txy"] else "CHECK",
            ("the 12 N push guard clips the torque channel (no damage, but the reading is invalid while clipped, which is when the guard is needed): cap the push guard at 7.5 N, or step up to the ATI Nano25 SI-125-3 (Txy 1.5 N m, force resolution 10 mN, 63 g, 25 x 21.6 mm, 3 x M3 on a 20.32 mm circle)" if t_push >= S["txy"] else
             "thin margin at the 12 N guard: order the SI-40-2 calibration (Txy 2 N m, 10 mN resolution) of the same sensor" if t_push >= 0.8 * S["txy"] else ""))
    else:
        add("sensor", "force sensing on the tool", f"{S['name']}: insertion axis and vertical, resolution {S['res_N']*1e3:.0f} mN", "0.1 N at 1 kHz for the guards; no torque channels", "CHECK", "two single-axis cells in the lower finger cover the slide, lift, push and press guards; the lateral axis and the torques are not measured; bench-verify noise and drift")
    # fingers
    plate_lo = model.geom("plate_lo"); plate_up = model.geom("plate_up"); leg_up = model.geom("pad_leg_e")
    m_lo = sum(gm[n] for n in ("plate_lo", "leg_e", "shoe_g", "shoe_bar", "blade"))
    m_up = sum(gm[n] for n in ("plate_up", "pad_leg_e", "pad_arm", "pad_g"))
    add("gripper", "finger mass (lower / upper)", f"{m_lo*1e3:.0f} g / {m_up*1e3:.0f} g", f"{gl['max_finger_g']} g per finger ({gl['name']})", "OK" if max(m_lo, m_up) * 1e3 < gl["max_finger_g"] else "CHECK")
    reach_up = abs(float(model.geom_pos[leg_up.id][0])) + float(model.geom_size[leg_up.id][0]) + F.PAD_OPEN + F.PAD_T_UP / 2
    reach_up = (G.get("jaw_lo_face", F.JAW_LO_FACE) + G["jaw_sep_open"] + 2 * G["jaw_t"]) - F.PAD_OPEN   # upper jaw face to the pad underside
    reach_lo = G.get("jaw_lo_face", F.JAW_LO_FACE)
    add("gripper", "finger reach (jaw face to the working face), upper / lower", f"{reach_up*1e3:.0f} mm / {reach_lo*1e3:.0f} mm", f"{gl['max_reach_mm']} mm", "OK" if reach_up * 1e3 <= gl["max_reach_mm"] else "CHECK", "the maker's finger envelope (sqrt(x2+z2))")
    if gl["My"]:
        add("gripper", "moment on the jaw from the 5 N pinch at the finger reach", f"{TASK['pinch_N']*reach_up:.2f} N m", f"My {gl['My']} N m, Mz {gl['Mz']} N m", "OK" if TASK["pinch_N"] * reach_up < gl["My"] else "CHECK")
        add("gripper", "moment on the lower jaw from the 20 N press", f"{TASK['press_max_seen_N']*(reach_lo+0.007):.2f} N m", f"Mx {gl['Mx']} N m", "OK" if TASK["press_max_seen_N"] * (reach_lo + 0.007) < gl["Mx"] else "CHECK", "the press goes through the shoe, 7 mm ahead of the jaw's pin line")
    leg_alu = arm == "lrmate200id"      # 2026-10-13: the LR Mate's longer upper leg (46 mm reach) is machined from 6061, not printed: 0.21 mm of deflection printed, 0.02 mm in aluminium; the LEHZ16 allows 50 g per finger
    d_leg = beam_deflection(TASK["pinch_N"], reach_up, F.LEG_W, F.LEG_W, E_AL if leg_alu else E_PAHT)
    add("stiffness", "upper leg deflection at the 5 N pinch (%s, %g x %g mm)" % ("machined 6061" if leg_alu else "printed PAHT-CF", F.LEG_W * 1e3, F.LEG_W * 1e3), f"{d_leg*1e3:.2f} mm", "0.1 mm", "OK" if d_leg < 1e-4 else "CHECK", "with a 6 x 6 mm leg: %.2f mm; in 6061: %.3f mm. The pinch is force-controlled, so deflection costs no grip, but it sets the pad's angle." % (beam_deflection(TASK["pinch_N"], reach_up, 0.006, 0.006, E_PAHT) * 1e3, beam_deflection(TASK["pinch_N"], reach_up, F.LEG_W, F.LEG_W, E_AL) * 1e3))
    sig = 6 * TASK["lid_lift_N"] * F.BLADE_L / (F.BLADE_W * F.BLADE_T ** 2)
    add("stiffness", "lip bending stress lifting the lid (2 N on the 1.5 mm steel lip)", f"{sig/1e6:.0f} MPa", "about 500 MPa yield (spring steel)", "OK" if sig < 300e6 else "CHECK")
    sig_push = 6 * TASK["push_guard_N"] * 0.0025 / (F.BLADE_W * 0.0005 ** 2) if False else 0
    add("task", "latch push force through the shoe edge (12 N guard)", f"{TASK['push_guard_N']} N on a 0.5 x 20 mm edge", "", "OK", "the edge is backed by the wing and the plate; the lid friction torque needs about 6 N at 3 mm")
    # reach: the sweep tilt the arm managed
    add("reach", "tool tilt at the connector (latch sweep)", "20 deg reached (capped) in the twin", "lid needs the lip to follow its arc", "OK", "IK probe at the connector pose; the LR Mate had the same cap")
    # clearances that depend on the real laptop
    add("laptop", "shoe underside over the board during the slide", "0.17 mm", "any part taller than 0.1 mm within 20 x 10 mm in front of the mouth", "CHECK", "the FX505 board around CON3104: confirm nothing (passives, solder) stands in the shoe's footprint; the twin's board is bare")
    add("laptop", "pad 46 mm wide over the connector", "46 mm span, 3 to 5 mm above the board", "components beside the connector", "CHECK", "confirm with a photo along the connector; the pad can be cut to 32 mm if needed")
    add("laptop", "gripper body beside the cable, 17 to 35 mm east, 7 mm above the board", "", "nothing taller than 6 mm east of the cable within 90 mm", "CHECK", "the twin's laptop has the SO-DIMM 25 mm behind and a 4 mm deck rail 40 mm behind; the east side is unmodelled")
    add("laptop", "sideways exit, 45 mm east after the latch opens", "", "clear path 45 mm east of the connector at 3 to 50 mm height", "CHECK", "unmodelled on the real board")
    # cables and electrics
    add("cables", "MEGP 25E cable to the Meca500 tool port", "the old MGC-SS35 and MGC-AA25 jumpers are discontinued (2026); current parts are the coiled MGC-HFC (straight) and MGC-HFA (90 deg), length unpublished", "gripper sits about 45 mm from the flange via the bracket", "CHECK" if ti["gripper"] == "megp25e" else "n/a", "confirm the coiled cable reaches 45 mm plus the bend before ordering; the tool port is proprietary and feeds only the gripper (its own draw is 24 V 0.14 A)")
    if sensor == "cell":
        add("cables", "load cell leads", "4 wires per cell, inside the tool to the ADC boards", "nothing leaves the tool but the 24 V pair and the USB pair", "OK", "the pad cell's leads run along the upper finger plate")
    else:
        add("cables", f"{S['name']} cable", "ATI cable exits the sensor's side; 2 m to the interface box", "must clear the bracket leg and the wrist", "CHECK", "orient the sensor's cable exit west (away from the bracket leg) when setting the dowels")
    if sensor == "cell":
        add("electrics", "force electronics", "tool MCU (RP2040 class) + two ADS122C04 24-bit boards on the body, 1 kSPS per cell, USB 2 to the PC", "0.1 N at 1 kHz for the guards", "CHECK", "bench: noise and drift under 0.1 N at 1 kHz with the chosen ADC and cells")
    else:
        add("electrics", "force/torque electronics", "ATI Net F/T or USB DAQ interface box (not in the stack)", "", "CHECK", "add to the order: Nano17/Mini40 need ATI's interface (about $1,500 to $3,000) or choose Bota MiniONE Pro with built-in EtherCAT")
    if ti["gripper"] == "lehz16":
        add("electrics", "LEHZ16 control", "JXC51/JXC61 parallel IO (NPN/PNP) or JXC91 EtherNet/IP, 24 V", "FANUC R-30iB Mate Plus DO polarity, or the EtherNet/IP option", "CHECK", "confirm the controller's DO polarity before choosing JXC51 or JXC61; the step data holds the force and the two jaw positions")
    elif ti["gripper"] == "gep2006":
        add("electrics", "Zimmer GEP2006 control", "IO-Link, 24 V", "IO-Link master on the LR Mate controller", "CHECK", "add an IO-Link master (about $300) and the LR Mate EE connector 24 V")
    add("electrics", "cameras", "D405 USB3 on the arm; 2 macro cameras on the bench" if F.FIXED_CAMS else "2 x USB3 macro cameras + D405 USB3", "1 USB3 cable along the arm to the PC" if F.FIXED_CAMS else "3 USB3 cables along the arm to the PC", "OK" if F.FIXED_CAMS else "CHECK", "use a USB3 hub on link 3 or run three cables; strobe the light from a camera GPIO")
    # joint-limit margins through the recorded run (laptop case), per joint
    try:
        tr = np.load(ROOT / "out" / "ffc_zif" / f"{arm}-{tool}-laptop" / "traj.npz"); Q = tr["qpos"]; info = F.ARM_INFO[arm]
        worst = []
        for jn in info["joints"]:
            j = model.joint(info["prefix"] + "_" + jn); lo, hi = model.jnt_range[j.id]; q = Q[:, j.qposadr[0]]
            if hi > lo:
                worst.append((jn, math.degrees(min(q.min() - lo, hi - q.max()))))
        wj = min(worst, key=lambda t: t[1])
        add("reach", "closest approach to a joint limit over the whole run", f"{wj[1]:.1f} deg on {wj[0]}", "more than 5 deg", "OK" if wj[1] > 5 else "CHECK", "; ".join(f"{n} {m:.0f} deg" for n, m in worst))
    except Exception as e:
        add("reach", "joint-limit margins", "no run found", "", "CHECK", str(e)[:80])
    # screws on the sensor under the press torque: the moment is reacted by the bolt circle
    T_press = TASK["press_max_seen_N"] * shoe_offset
    pcd_s = {"nano17": 0.0125, "nano25": 0.02032, "mini40": 0.033, "cell": 0.030}[sensor]; n_s = 3 if sensor != "cell" else 4; screw = "M2" if sensor == "nano17" else "M3"; proof = 2.5e3 if sensor == "nano17" else 5.8e3   # N, property class 12.9 approx (ESTIMATE)
    F_screw = T_press / (n_s / 2 * pcd_s / 2)                                 # worst-case tension on one screw, bolt circle reacting the moment
    add("loads", f"tension on one {screw} sensor screw from the 20 N press", f"{F_screw:.0f} N", f"proof load about {proof:.0f} N", "OK" if F_screw < 0.2 * proof else "CHECK", f"{n_s} x {screw} on {pcd_s*1e3:g} mm PCD; preload them to 60 percent of proof so the joint never gaps")
    # bracket deflection: the vertical leg as a cantilever under the press reaction at the gripper plate
    Lb = abs(zc_leg := (float(model.geom_pos[model.geom('bracket_v').id][2]) - (float(model.geom_pos[model.geom('bracket_h').id][2])))) if False else float(model.geom_size[model.geom("bracket_v").id][2]) * 2
    if sensor == "cell":
        add("stiffness", "tool compliance through the two bar cells (TAL220B class: about 0.5 mm at the 49 N rating, parallelogram flexure, ESTIMATE)", "0.005 mm at the 0.5 N slide friction, 0.04 mm at the 4 N guard, 0.12 mm at the 12 N lid push", "0.2 mm lateral window; the bench cameras re-measure the tip after the grasp", "CHECK", "bench: load the tip with 4 N along the slide and read the tip motion with a dial gauge; a 10 kg Fz bar halves it at 0.04 N resolution")
    d_br = beam_deflection(TASK["press_max_seen_N"], Lb, F.BRACKET_W, F.BRACKET_T, E_AL)
    if sensor != "cell": add("stiffness", "bracket vertical leg deflection under the 20 N press (6061, %g x %g leg, %.0f mm long)" % (F.BRACKET_W * 1e3, F.BRACKET_T * 1e3, Lb * 1e3), f"{d_br*1e3:.3f} mm", "0.05 mm", "OK" if d_br < 5e-5 else "CHECK", "the press goes through the gripper and the bracket into the sensor; this is the bending of the leg about its thin axis, worst case")
    # dynamics
    a_max = 0.04 / 0.3                                                         # worst: 40 mm/s reached in 0.3 s
    add("loads", "inertial load from the arm's motion", f"{m_tot*a_max:.2f} N ({m_tot*1e3:.0f} g at {a_max:.2f} m/s2)", "", "OK", "speeds stay under 40 mm/s; inertial forces are two orders below the task forces")
    if ti["gripper"] == "lehz16":
        add("gripper", "grip force setting", "3 to 8 N, pushing force 40 to 100 percent in the step data", "5 N pinch in the twin", "CHECK", "open-loop force, +-30 percent of full scale: bench-calibrate with a scale; 40 percent force only at 5 to 20 mm/s pushing speed")
    elif ti["gripper"] == "gep2006":
        add("gripper", "lowest grip force", "40 N (IO-Link selects 40, 75, 110 or 145 N; confirmed)", "5 N pinch assumed in the twin", "CHECK", "the Zimmer cannot pinch gently: 40 N on the 32 x 6 mm pad is 0.2 MPa on the cable, harmless to polyimide and copper, and the finger legs are sized for it; the better match for the LR Mate is a gripper with a settable low force (SMC LEHZ10 class, 2.5 to 6.5 N): being researched")
    elif ti["gripper"] == "servo_v1":
        add("gripper", "grip force setting", "5 N pinch closed on the finger load cells; 0.24 um and about 0.04 N per encoder step through the pad", "+-0.3 N at 5 N", "CHECK", "the servo is a position stage; bench: pinch repeatability on a scale, target +-0.3 N; the servo current is not used for force (Dynamixel current deadband 0.45 N, TF-Gripper measurement)")
    else:
        add("gripper", "grip force setting", "5 N = 12 percent of 40 N", "GripperForce 1 to 100 percent (Meca500 firmware)", "OK", "set 12 to 15 percent; the firmware's lowest usable level should be confirmed on the bench")
    if arm == "meca500":
        # Mecademic Fig. 19 (0.5 kg curves, read 2026-10-13), at the chart's left edge (30 mm off axis; ours is nearer the axis): 75% acceleration allows 40 mm below the flange, 50% allows 60 mm, 25% allows 110 mm
        accel = "75" if com_t[2] <= 0.040 else "50" if com_t[2] <= 0.060 else "25" if com_t[2] <= 0.110 else None
        add("mass", "Meca500 centre-of-mass chart (manual Fig. 19)", f"{m_tot*1e3:.0f} g with the COM {com_t[2]*1e3:.0f} mm below the flange, {lever*1e3:.0f} mm off axis", "0.5 kg curves at 30 mm off axis: 40 mm below at 75 percent acceleration, 60 mm at 50 percent, 110 mm at 25 percent",
            "OK" if accel else "CHECK", (f"inside the {accel} percent curve at the chart's full 0.5 kg; at our {m_tot*1e3:.0f} g the moment is {m_tot/0.5*100:.0f} percent of that, so set the joint acceleration to {accel} percent (the task moves at 2 mm/s and never needs more); the 3.4 N m flange moment is respected by a wide margin" if accel else "outside every curve: trim the stack"))
    else:
        add("mass", "LR Mate payload", f"{m_tot*1e3:.0f} g", "7 kg", "OK", "")
    if F.FIXED_CAMS:
        CM = F.CAMERAS[arm]
        add("vision", "macro cameras", f"2 x Basler dart daA1920-160um + Edmund #37-182 16 mm on bench posts, {CM['wd']*1e3:.0f} mm from the mouth, {CM['el_deg']:.0f} deg elevation, azimuths {CM['az_deg'][0]:.0f} and {CM['az_deg'][1]:.0f} deg from east", "lens optimised for 150 to 250 mm; a still view for the policy; nothing on the stack in the way", "OK", "fixed to the breadboard: no wrist vibration, no USB loom on the arm, the lens inside its range; scripts/ffc_zif_camera_check.py renders the real views along the run and flags any frame where the stack hides the tip, the mouth or the lid edge")
        add("vision", "resolution at the mouth", "31 um per pixel, field 60 x 37 mm", "0.2 mm lateral tolerance = 6.5 px; 0.6 deg of yaw over 28 mm = 0.29 mm = 9 px", "OK", "a sub-pixel edge fit on the stiffener gives 5 to 10 um; the hand-eye calibration (ChArUco at the working distance) is the larger term, 20 to 40 um")
        add("vision", "depth of field", "about 5 mm at f/4 in blue at 160 mm (twice the 108 mm figure)", "tip, lid edge and mouth within 5 mm of depth", "OK", "f/8 doubles it but is diffraction-limited in blue: stay at f/4 to f/5.6")
        add("vision", "shutter", "Basler: global shutter", "", "OK", "")
        add("vision", "light strobe", "no strobe input on the CCS bar", "", "CHECK", "switch the 24 V line from a camera GPIO, or run the bar continuous and shorten the exposure")
        add("vision", "camera availability", "Basler dart daA1920-160um S-mount: product page says not in series production yet", "", "CHECK", "fallbacks: daA1920-30um (30 fps) or the C-mount ace 2 a2A1920-160umBAS with a 25 mm lens at 200 mm; on a bench post the camera mass no longer matters")
        add("vision", "policy camera set", "2 fixed macro views at up to 160 fps + the wrist D405 RGB 848 x 480 at 30 fps", "ACT/ALOHA: 2 static + wrist 480 x 640 RGB at 50 Hz", "OK", "downsample the macro frames to 640 x 400; mono is fine under blue light (the scene is monochrome anyway)")
    else:
        CM = F.CAMERAS[arm]
        add("vision", "macro lens working distance", f"{CM['wd']*1e3:.0f} mm, 35 deg down", "lens specified to 150 mm and beyond (Meca500 pick)" if arm == "meca500" else "lens specified for the range", "CHECK" if arm == "meca500" else "OK", "the 16 mm M12 lenses are only specified from 150 mm: at 108 mm the barrel is unscrewed 0.8 mm past its stated limit; bench-test the image, or move the cameras out to 150 mm (longer arms)")
        add("vision", "depth of field on a 35 deg grazing view", "about 3 mm at f/4 in blue", "the tip, the lid edge and the mouth within 3 mm of height", "OK", "f/8 would be diffraction-limited in blue (10.7 um Airy disc): stay at f/4 to f/5.6")
        add("vision", "shutter", "Basler: global shutter" if arm == "meca500" else "Alvium AR0521: rolling shutter", "", "OK" if arm == "meca500" else "CHECK", "" if arm == "meca500" else "rolling shutter smears a moving tip at 2 mm/s only by 0.1 um per line: fine for the slide, but use the strobe for the latch sweep")
        add("vision", "light strobe", "no strobe input on the CCS bar" if arm == "meca500" else "LM75 has a strobe input", "", "CHECK" if arm == "meca500" else "OK", "switch the 24 V line from a camera GPIO, or run the bar continuous and shorten the exposure" if arm == "meca500" else "")
        add("vision", "camera availability", "Basler dart daA1920-160um S-mount: product page says not in series production yet" if arm == "meca500" else "Alvium 1800 U-500m: in stock at Edmund", "", "CHECK" if arm == "meca500" else "OK", "confirm stock before ordering; the Commonlands 16 mm lens is the cheaper alternative" if arm == "meca500" else "")
    add("task", "latch lid front edge", "the twin hooks a 0.4 mm lip 1 mm under the lid's front edge", "the real CON3104 actuator must overhang the slot entrance by at least 0.5 mm", "CHECK", "confirm on the real connector with a calliper or a macro photo: if the actuator has no undercut, the lip becomes a wedge that lifts the edge instead")
    add("task", "board flatness under the shoe", "shoe underside 0.17 mm above the twin's flat board", "IPC-6012 allows 0.75 percent bow: 0.75 mm over 100 mm, about 0.15 mm over the shoe's 20 mm", "CHECK", "ride the cable tip 0.2 to 0.3 mm above the slot floor line on approach (the mouth chamfer guides it down) and let the force guard catch contact; the 0.3 mm shoe variant gives 0.2 mm more")
    add("task", "cable yaw into the mouth", "28 mm cable in a 28.6 mm slot: 0.3 mm a side", "yaw within 0.6 deg before the slide", "CHECK", "the pinch does not rotate the cable: the two macro cameras measure the stiffener's yaw and the wrist corrects it (joint 6) before the slide; add this to the expert")
    if sensor == "cell":
        add("sensor", "gravity compensation", "only the 20 g lower finger hangs on the cells; the tool tilts up to 20 deg", "bias under 0.2 N, within the guard margins", "OK", "zero the cells at the approach pose; no mass model needed")
    else:
        add("sensor", "gravity compensation", "400 g hangs under the sensor and the tool tilts up to 20 deg", "bias error up to 4 N if uncompensated", "CHECK", "standard: bias the sensor at each approach pose, or compensate with the known mass and the flange orientation from the controller")
    if sensor == "cell":
        add("cables", "cable loom on the arm", "1 x USB3 high-flex (D405) + one 4-core tool cable (12 V pair + USB 2 data), both along the arm", "", "OK", "about 120 g of cable; service loop at the wrist, strain relief at the standoff")
    else:
        add("cables", "cable loom on the arm", "3 x USB3 high-flex (62 g/m, 6.5 mm OD, Helukabel 805287 class) + sensor cable + light cable + gripper cable", "", "CHECK", "about 250 g of cable along the arm plus drag at the wrist: route along the upper arm with a service loop at J5, strain relief at the bracket; no drag-chain-rated USB3 hub exists, so run three cables")
    add("electrics", "ESD", "polyimide cable on a live mainboard", "", "CHECK", "ground the steel shoe through the bracket and the arm; PAHT-CF prints are mildly conductive; use an ESD mat under the pallet")
    add("fixture", "platform feed clip", "1 mm bar under the cable, one post on the west", "", "OK", "redesigned so the gripper body passes east of it; the laptop case has no fixture")
    return {"arm": arm, "tool": tool, "mass_g": round(m_tot * 1e3), "com_mm": [round(v * 1e3, 1) for v in com_t], "rows": rows}


out = {}
for arm, tool in (("meca500", "megp_nano25"), ("meca500", "megp_nano25_thin"), ("meca500", "megp_nano25_zaber"), ("lrmate200id", "lehz16_mini40"), ("meca500", "servo_v1"), ("lrmate200id", "servo_v1_lr")):
    r = review(arm, tool); out[f"{arm}/{tool}"] = r
    n_check = sum(1 for x in r["rows"] if x["status"] == "CHECK"); print(arm, tool, f"{r['mass_g']} g below the flange, COM {r['com_mm']} mm;", n_check, "items to check")
(ROOT / "out" / "ffc_zif" / "review.json").write_text(json.dumps(out, indent=1)); print("wrote out/ffc_zif/review.json")
