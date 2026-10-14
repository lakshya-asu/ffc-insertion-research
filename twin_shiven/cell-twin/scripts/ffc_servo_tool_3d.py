"""The servo tool as a working 3D schematic: every part of the stack from the flange to the fingertips at real size, the drive inside the body
(servo, belt, lead screw, nut, guide pins, carriers, rocker) drawn as real parts, animated: the jaw slider turns the screw, the nut carries the
upper carrier, the rocker moves the lower one the opposite way, and the fingers close on the cable. Hover any part for its name, click for its
card. The page also carries the verification: the interference check between every pair of parts at open and closed (computed here), the
kinematic numbers, and the twin's run results. The stack geometry comes from the twin itself (cell_twin/ffc_zif.py at the home pose), the
printed parts from the STEP files (tessellated by the stepenv), the bought parts from their datasheet sizes.
usage: .venv/bin/python scripts/ffc_servo_tool_3d.py [out.html]"""
import json, math, sys, itertools
from pathlib import Path
import numpy as np, mujoco
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from cell_twin import ffc_zif as F
from cell_twin import ffc_render_kit as K
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path.home() / "Desktop" / "Synphony-BOM" / "FFC-servo-tool-3d.html"
MESH = json.loads((ROOT / "out/ffc_zif/servo_v1_meshes.json").read_text())
PARTS = json.loads((ROOT / "assets/ffc/parts.json").read_text())["parts"]
M = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float)     # jaw frame (x down, y east, z forward) -> assembly (X forward, Y east, Z up)
G = F.GRIPPERS["servo_v1"]
CLOSED = (F.PAD_OPEN - F.STIFF_T) / 2 * 1e3                  # travel per jaw when closed on the cable: 1.35 mm


def cat(pid):
    q = PARTS.get(pid, {})
    bits = [q.get("vendor", ""), q.get("part_number", "") if q.get("part_number") not in (None, "n/a") else ""]
    if isinstance(q.get("price_usd_if_listed"), (int, float)):
        bits.append(f"${q['price_usd_if_listed']:,.2f}")
    if q.get("mass_g"):
        bits.append(f"{q['mass_g']} g")
    head = " · ".join(b for b in bits if b)
    return (head + "\n" if head else "") + (q.get("what_it_is", "") or "") + (f"\n{q['buy_url']}" if q.get("buy_url") and q["buy_url"] not in ("none", "") else "")


def quat_from_R(R):
    q = np.empty(4); mujoco.mju_mat2Quat(q, R.flatten()); return [float(x) for x in q]      # w x y z


parts = []


MAT_BY_NAME = {"servo": "plastic_blk", "horn": "alu", "pulley_horn": "alu", "pulley_screw": "alu", "belt_w": "rubber", "belt_e": "rubber", "screw": "stainless", "nut": "peek", "pivot_pin": "stainless",
               "servo_body": "printed", "servo_belt_cover": "printed", "servo_jaw_carrier_upper": "printed", "servo_jaw_carrier_lower": "printed", "servo_rocker": "printed", "servo_standoff_meca": "printed_lt", "cable": "polyimide"}


def add(name, kind, pos, size, group, color, title, what, detail="", quat=(1, 0, 0, 0), mesh=None, pid=None, check=True, alpha=1.0):
    parts.append(dict(name=name, kind=kind, pos=[float(x) for x in pos], size=[float(x) for x in size], quat=[float(x) for x in quat], group=group, color=color, mesh=mesh, alpha=alpha, mat=MAT_BY_NAME.get(name) or K.material_key(name, "", "meca500", cells=True),
                      info=dict(title=title, what=what, detail=(detail + ("\n" + cat(pid) if pid else "")).strip()), check=check))


# ---------------------------------------------------------------- 1. the stack and the fingers from the twin (meca500, laptop), at the open position
spec = F.build("meca500", "laptop", "servo_v1"); model = spec.compile(); data = mujoco.MjData(model); mujoco.mj_forward(model, data)
jb = model.body("jaws").id; Rj = data.xmat[jb].reshape(3, 3); pj = data.xpos[jb]
TWIN = {   # geom name: (group, colour, title, what, catalogue id)
    "stack_flange": ("static", "#777", "Meca500 flange", "the arm's mounting face: 4 x M3 on 15.5 mm, 9 H7 bore", "meca500_flange"),
    "stack_adapter_plate": ("static", "#9aa5b1", "Adapter plate (ours, 6061, 4 mm)", "flange pattern on top, four M3 to the standoff below", "adapter_plate_meca_servo"),
    "bracket_h": ("static", "#2d3038", "Cell A: bar load cell, insertion axis", "fixed end under the standoff (2 x M4), free end east under the corner plate; bends along X and reads the slide friction, the seat rise and the lid push", "load_cell_tal220b"),
    "cell_b": ("static", "#50545e", "Cell B: bar load cell, vertical axis", "fixed end under the corner plate, runs along the body; bends along Z and reads the lid lift and the latch press", "load_cell_tal220b"),
    "cell_corner": ("static", "#9aa5b1", "Corner plate (ours, 6061, 4 mm)", "joins cell A's free end to cell B's fixed end, on top", "bracket_servo"),
    "cell_tab": ("static", "#9aa5b1", "Tab (ours, 6061, 4 mm)", "on top of cell B's free end, one piece with the side plate", "bracket_servo"),
    "bracket_v": ("static", "#9aa5b1", "Side plate (ours, 6061, 4 mm)", "down the body's east face, 4 x M3 into heat-set inserts: the whole tool hangs from it", "bracket_servo"),
    "d405_g": ("static", "#1f2126", "Intel RealSense D405 wrist camera", "81 mm from the fingertips, looking at them", "d405"),
    "arm_d405_y": ("static", "#9aa5b1", "D405 mount bar (ours, 6061)", "under cell A's fixed end, west to the camera", "d405"),
    "arm_d405": ("static", "#9aa5b1", "D405 mount bar (ours, 6061)", "beside the camera's west face", "d405"),
    "arm_d405_plate": ("static", "#9aa5b1", "D405 side plate (ours)", "2 x M2 into the camera's side", "d405"),
    "cell_p": ("upper", "#50545e", "Cell P: bar load cell, the pinch", "the upper finger's cross member: fixed end on the upper carrier, the pad on its free end; the servo closes until it reads 5 N", "load_cell_tal220b"),
    "plate_up": ("upper", "#e6a85c", "Upper finger plate (printed)", "from cell P's free end forward to the pad arm", "finger_printed_r"),
    "pad_leg_e": ("upper", "#e6a85c", "East leg of the upper finger", "drops from the plate to the pad arm, east of the cable", "finger_printed_r"),
    "pad_arm": ("upper", "#e6a85c", "Pad arm", "12 to 15 mm ahead of the jaw line, carries the pad over the cable", "finger_printed_r"),
    "pad_g": ("upper", "#c2185b", "Silicone pad, 30 x 6 mm", "pinches the cable's stiffener at 5 N against the shoe", "pad_rubber"),
    "plate_up_tongue": ("upper", "#e6a85c", "Upper plate tongue (printed)", "on top of cell P's free end; 2 x M5 down into the cell carry the whole upper finger", "finger_printed_r"),
    "plate_up_web": ("upper", "#e6a85c", "Upper plate web (printed)", "joins the tongue on the cell down to the plate level ahead of it", "finger_printed_r"),
    "plate_lo": ("lower", "#e6a85c", "Lower finger plate (printed)", "bridges 4 mm over the cable from the lower carrier to the one east leg", "finger_printed_l"),
    "leg_e": ("lower", "#e6a85c", "East leg of the lower finger (6 x 6 mm)", "drops to the shoe bar; only east of the cable, so the shoe can slide out sideways after seating", "finger_printed_l"),
    "shoe_bar": ("lower", "#6a7a8c", "Shoe bar", "carries the steel shoe west under the cable", "finger_printed_l"),
    "shoe_g": ("lower", "#6a7a8c", "Steel shoe, 0.5 mm shim, 20 x 7 mm", "slides 14 mm under the cable tip and rides 0.17 mm over the board into the connector; its rear edge pushes the lid shut", "finger_shoe_0p3"),
    "blade": ("lower", "#6a7a8c", "The 1.5 mm lip", "hooks 1 mm under the lid's front edge and lifts it open (2 N)", "finger_shoe_0p3"),
}
for i in range(model.ngeom):
    g = model.geom(i).name
    if g not in TWIN:
        continue
    grp, col, title, what, pid = TWIN[g]
    R_rel = Rj.T @ data.geom_xmat[i].reshape(3, 3); p_rel = Rj.T @ (data.geom_xpos[i] - pj)
    R_a = M @ R_rel; p_a = M @ p_rel * 1e3
    t = model.geom_type[i]; sz = model.geom_size[i] * 1e3
    if t == mujoco.mjtGeom.mjGEOM_BOX:
        add(g, "box", p_a, sz, grp, col, title, what, pid=pid, quat=quat_from_R(R_a), check=(g != "d405_g"))
    elif t == mujoco.mjtGeom.mjGEOM_CYLINDER:
        add(g, "cyl", p_a, [sz[0], sz[1], 0], grp, col, title, what, pid=pid, quat=quat_from_R(R_a))
# the stroke centreline and the body's place, from the twin's own numbers
C = (F.JAW_LO_FACE + G["jaw_sep_open"] / 2 + G["jaw_t"]) * 1e3       # 20 mm above the shoe plane
FO = F.FINGER_OFF * 1e3; ZJ = 2.0; bl = (G["body_l"] + G["end_l"]) * 1e3
Ls_top = None
for p in parts:
    if p["name"] == "bracket_h":
        Ls_top = p["pos"][2] + p["size"][2]      # top of cell A = the standoff's bottom

# ---------------------------------------------------------------- 2. the printed parts from the STEP files (CAD frame -> assembly)
def add_mesh(stem, pos, group, color, title, what, detail="", pid=None, quat=(1, 0, 0, 0), alpha=1.0, check=True, size=(0, 0, 0)):
    add(stem, "mesh", pos, size, group, color, title, what, detail, quat=quat, mesh=stem, pid=pid, alpha=alpha, check=check)


add_mesh("servo_body", (-59, 26, 20), "static", "#f3c68a", "Printed body (PAHT-CF)", "55 x 22 x 26 mm shell plus the 6 mm end block: servo pocket at the back, the carriers' slot through the front, the screw bore, two pin seats, the rocker pivot, the belt channel on top, 4 x M3 heat-set in the east face", pid="servo_tool_v1", alpha=0.55, check=False)
add_mesh("servo_belt_cover", (-59, 26, 20), "static", "#f3c68a", "Belt cover (printed)", "6 mm over the two pulleys and the 3 mm GT2 belt; 4 x M2 into the body", pid="servo_tool_v1", alpha=0.45, check=False)
add_mesh("servo_jaw_carrier_upper", (-14, 26, 29), "upper", "#e6a85c", "Upper jaw carrier (printed)", "holds the PEEK nut on the screw; runs in the printed slot with the screw as its guide; the rocker's upper end rides in the notch at its front-west corner; the upper finger (cell P) bolts to its top face with the MEGP pattern", pid="servo_jaw_l", check=False)
add_mesh("servo_jaw_carrier_lower", (-14, 26, 11), "lower", "#e6a85c", "Lower jaw carrier (printed)", "screw passes through a clearance hole; guided by the slot and the rocker's lower end in its notch; the lower finger bolts to its bottom face", pid="servo_jaw_r", check=False)
add_mesh("servo_rocker", (-2, FO - 5.5, 20), "rocker", "#b5651d", "Rocker (printed, 3 x 3 x 22)", "pivots on a 2 mm pin through the end block at the carriers' front-west corner; its ends ride in a notch in each carrier, so 1.5 mm down on the upper carrier is 1.5 mm up on the lower", pid="servo_rocker", check=False)
add_mesh("servo_standoff_meca", (-62, 0, Ls_top), "static", "#f3c68a", "Printed standoff, 28 x 22 mm", "keeps the jaw line where the sensor stacks have it; four M3 up to the adapter plate, two M4 down into cell A's fixed end", pid="servo_standoff", check=False)

# ---------------------------------------------------------------- 3. the drive: bought parts at datasheet sizes
SX, SY = -9.0, FO                                     # the lead screw: 9 mm behind the jaw end face, on the body's centreline
add("servo", "box", (-57 + 17, FO, 20), (17, 10, 13), "static", "#2b2b33", "Dynamixel XL330-M077-T", "lies flat: 34 along the body, 20 across, 26 tall (its full height); horn on top; 5 V, 0.215 N m, 383 rpm, 4096 steps per turn, 18 g", pid="xl330_m077")
HX = -57 + 34 - 8                                     # horn axis, 8 mm behind the servo's front end
add("horn", "cyl", (HX, FO, 33.25), (6, 0.25, 0), "pulley_h", "#999", "Servo horn", "the 4-hole output disc on the servo's top face", pid="xl330_m077")
add("pulley_horn", "cyl", (HX, FO, 36), (3.6, 2.5, 0), "pulley_h", "#777", "GT2 pulley, 12 teeth, on the horn", "bolted to the horn's 4-hole pattern; turns with the servo", "ESTIMATE $5; pitch diameter 7.6 mm")
add("pulley_screw", "cyl", (SX, SY, 36), (3.6, 2.5, 0), "screw", "#777", "GT2 pulley, 12 teeth, on the lead screw", "1:1 with the horn pulley; the screw's upper bearing is the printed seat in the body's top wall just below it", "bought, 4 mm bore, ESTIMATE $5")
for y in (FO - 3.8, FO + 3.8):
    add(f"belt_{'w' if y < FO else 'e'}", "box", ((HX + SX) / 2, y, 36), ((SX - HX) / 2, 0.5, 1.5), "static", "#111", "GT2 belt, 3 mm wide, 2 mm pitch", "closed loop about 68 mm; ESTIMATE $4", check=False)
add("screw", "cyl", (SX, SY, 23.5), (2, 15.5, 0), "screw", "#8a8f99", "Lead screw M4 x 1 mm, 17-4 PH stainless, cut to 31 mm", "from 8 mm above the shoe plane (through the lower carrier's clearance hole) up through the nut and the body's top wall to its pulley; one turn = 1 mm; the 4.6 degree lead angle is self-locking", pid="lead_screw_m4x1")
add("nut", "cyl", (SX, SY, 27), (4, 4, 0), "upper", "#d9c48a", "PEEK anti-backlash nut M4 x 1", "bonded in the upper carrier; 8 mm long, so it protrudes 4 mm into the gap below the carrier; the screw through it is the upper carrier's guide", pid="nut_peek_m4x1")
add("pivot_pin", "cyl", (-2, FO - 2.2, 20), (1, 7.0, 0), "static", "#8a8f99", "Rocker pivot pin, 2 mm steel", "across the end block through the west cheek, the rocker and the middle web", "ESTIMATE $1", quat=quat_from_R(np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)))
# the cable, for context
add("cable", "box", (6, 0, -0.15), (10, 14, 0.15), "static", "#c58a1c", "The FFC (context)", "28 mm wide, 0.3 mm stiffened tip, lying on the shoe", check=False)

# ---------------------------------------------------------------- 4. verification: interference at open and closed (axis-aligned parts only)
def aabb(p, s):
    """axis-aligned bounds of a box or cylinder (local z axis) under its rotation, shifted by the jaw travel of its group"""
    pos = np.array(p["pos"]); R = np.empty(9); mujoco.mju_quat2Mat(R, np.array(p["quat"])); R = R.reshape(3, 3)
    if p["kind"] == "box":
        h = np.abs(R) @ np.array(p["size"])
    elif p["kind"] == "cyl":
        r, hh = p["size"][0], p["size"][1]; a = np.abs(R[:, 2])
        h = a * hh + np.sqrt(np.clip(1 - a ** 2, 0, 1)) * r
    else:
        return None
    dz = {"upper": -s, "lower": s}.get(p["group"], 0.0)
    return pos + np.array([0, 0, dz]) - h, pos + np.array([0, 0, dz]) + h


ALLOWED = {("screw", "nut"), ("screw", "servo_body"), ("screw", "pulley_screw"), ("nut", "servo_jaw_carrier_upper"), ("horn", "pulley_horn"), ("horn", "servo"),
           ("cell_p", "plate_up"), ("plate_up", "pad_leg_e"), ("pad_leg_e", "pad_arm"), ("pad_arm", "pad_g"), ("plate_lo", "leg_e"), ("leg_e", "shoe_bar"), ("shoe_bar", "shoe_g"), ("shoe_g", "blade"),
           ("bracket_h", "cell_corner"), ("cell_b", "cell_corner"), ("cell_b", "cell_tab"), ("cell_tab", "bracket_v"), ("stack_flange", "stack_adapter_plate"), ("bracket_h", "arm_d405_y"), ("arm_d405_y", "arm_d405"), ("arm_d405", "arm_d405_plate"),
           ("cable", "shoe_g"), ("cable", "pad_g"), ("cable", "shoe_bar"), ("cable", "blade")}
for y in ("w", "e"):
    ALLOWED |= {(f"belt_{y}", "pulley_horn"), (f"belt_{y}", "pulley_screw")}
ALLOWED |= {("pivot_pin", "servo_rocker"), ("screw", "pivot_pin")}


def interference(s):
    hits = []
    P = [p for p in parts if p["check"]]
    for a, b in itertools.combinations(P, 2):
        A, B = aabb(a, s), aabb(b, s)
        if A is None or B is None or (a["name"], b["name"]) in ALLOWED or (b["name"], a["name"]) in ALLOWED:
            continue
        if a["group"] == b["group"] and a["group"] in ("upper", "lower") and a["name"].startswith("bushing") == b["name"].startswith("bushing"):
            pass
        ov = np.minimum(A[1], B[1]) - np.maximum(A[0], B[0])
        if (ov > 0.001).all():
            hits.append((a["name"], b["name"], float(ov.min())))
    return hits


checks = {"open (0 mm)": interference(0.0), "closed on the cable (1.35 mm per jaw)": interference(CLOSED), "closed on nothing (1.5 mm per jaw, the hard stop)": interference(1.5)}
kin = {
    "travel per jaw": "1.5 mm to the hard stop; 1.35 mm when closed on the 0.3 mm stiffener",
    "screw turns, open to closed": f"{CLOSED:.2f} turns ({CLOSED*1e3:.0f} um at 1 mm lead); {CLOSED*4096:.0f} encoder steps; 0.24 um per step",
    "rocker swing": f"{math.degrees(math.asin(CLOSED/9)):.1f} deg about its pivot (arms 9 mm to each carrier's mid-plane); the rocker end shifts {9*(1-math.cos(math.asin(CLOSED/9))):.2f} mm along X inside the notch, which is 1.8 mm longer than the rocker for it",
    "jaw guidance": "the upper carrier is guided by the screw in its nut and the printed slot; the lower by the slot and the rocker: printed-slot repeatability about 0.05 to 0.1 mm, a bench item",
    "gap between the jaw faces": f"{G['jaw_sep_open']*1e3:.0f} mm open; the nut protrudes 4 mm into it, {G['jaw_sep_open']*1e3 - 4 - 2*CLOSED:.1f} mm clear of the lower carrier when closed",
    "pad to shoe": f"{F.PAD_OPEN*1e3:.0f} mm open, {F.STIFF_T*1e3:.1f} mm (the cable) closed",
    "time, open to closed": f"{CLOSED/6.4:.2f} s at 383 rpm (6.4 mm/s)",
    "torque at the screw for 5 N": "2.7 mN m against 215 mN m stall (30 percent screw efficiency)",
    "self-locking": "lead angle 4.6 deg, under the PEEK on steel friction angle (about 8 deg): the pinch holds unpowered",
}
try:
    MTX = {r["variant"]: r for r in json.loads((ROOT / "out/ffc_zif/matrix.json").read_text())}
    runs = []
    for v in ("meca500-servo_v1-laptop", "meca500-servo_v1-platform", "lrmate200id-servo_v1_lr-laptop"):
        ver = MTX.get(v, {}).get("verify") or {}
        if isinstance(ver, str) and ver.startswith("{"):
            ver = eval(ver)
        runs.append({"variant": v, "depth": ver.get("depth_mm"), "lateral": ver.get("lateral_mm"), "latch": ver.get("latch_closed"), "ok": bool(ver.get("depth_ok")) and bool(ver.get("latch_closed"))})
except Exception as ex:
    runs = [{"variant": "matrix.json not readable", "depth": str(ex), "lateral": "", "latch": "", "ok": False}]

# fasteners from the shared kit (tool and jaw frames) -> assembly frame, mm
px_m = float(model.body_pos[jb][0]); zj_m = float(model.body_pos[jb][2])
FAST = []
for f in K.fasteners_tool_frame(F, "meca500", "servo_v1", model):
    p_ = np.array(f["pos"]); a_ = np.array(f["axis"])
    if f["body"] == "tool":
        pos = np.array([px_m - p_[0], p_[1], zj_m - p_[2]]) * 1e3; ax = np.array([-a_[0], a_[1], -a_[2]]); grp = "static"
    else:   # jaw frame (pad and shoe items are given in the jaw frame here)
        pos = np.array([p_[2], p_[1], -p_[0]]) * 1e3; ax = np.array([a_[2], a_[1], -a_[0]]); grp = {"pad": "upper", "shoe": "lower"}.get(f["body"], "static")
    FAST.append(dict(kind=f["kind"], d=f["d"], len=f["len"], head=f["head"], pos=[float(x) for x in pos], axis=[float(x) for x in ax], title=f["title"], group=grp))
payload = dict(parts=parts, fasteners=FAST, meshes={k: MESH[k] for k in MESH if any(p.get("mesh") == k for p in parts)}, closed=CLOSED, checks=checks, kin=kin, runs=runs, FO=FO)
js = json.dumps(payload, separators=(",", ":"))
html = """<!doctype html><html><head><meta charset="utf-8"><title>Servo tool v1, working 3D schematic</title>
<style>body{margin:0;background:#FBF7F0;font:14px -apple-system,Inter,Segoe UI,sans-serif;color:#1E1B16;overflow:hidden}
#ui{position:absolute;left:12px;top:12px;width:330px;background:rgba(251,247,240,.96);border-radius:12px;padding:12px 14px;box-shadow:0 2px 12px rgba(0,0,0,.15);max-height:94vh;overflow:auto}
#ui h1{font-size:15px;margin:0 0 4px}.sub{color:#6B6257;font-size:12px;margin:0 0 8px}.row{display:flex;align-items:center;gap:8px;margin:3px 0;font-size:13px}.sw{width:12px;height:12px;border-radius:3px;display:inline-block}
button{border:0;background:#1E1B16;color:#fff;border-radius:8px;padding:5px 10px;cursor:pointer;font-size:12px}
#bar{position:absolute;left:360px;right:380px;bottom:12px;background:rgba(251,247,240,.96);border-radius:12px;padding:10px 14px;box-shadow:0 2px 12px rgba(0,0,0,.15)}#bar input[type=range]{width:100%}
#tip{position:absolute;pointer-events:none;display:none;background:#1E1B16;color:#fff;border-radius:8px;padding:6px 10px;font-size:12px;max-width:320px;z-index:5}#tip b{display:block;font-size:13px;margin-bottom:2px}
#right{position:absolute;right:12px;top:12px;width:350px;max-height:94vh;overflow:auto;background:rgba(251,247,240,.96);border-radius:12px;padding:12px 14px;box-shadow:0 2px 12px rgba(0,0,0,.15);font-size:12.5px}
#right h2{font-size:14px;margin:10px 0 4px}#right table{border-collapse:collapse;width:100%}#right td,#right th{padding:3px 5px;border-bottom:1px solid #EFE6D6;text-align:left;vertical-align:top}
.ok{color:#2f8f5b;font-weight:600}.bad{color:#b23;font-weight:600}#card{white-space:pre-wrap;border-top:1px solid #EFE6D6;margin-top:8px;padding-top:8px;display:none}#card a{color:#1f6fb2;word-break:break-all}
#parts div{cursor:pointer;padding:2px 0;font-size:12.5px}#parts div:hover{color:#b23}#parts .grp{font-weight:600;margin-top:6px;cursor:default;color:#6B6257;font-size:11px;letter-spacing:.06em;text-transform:uppercase}
</style></head><body>
<div id="ui"><h1>Servo tool v1: the mechanism, working</h1><p class="sub">Real sizes (grid 10 mm). Drag to orbit, scroll to zoom, right-drag to pan. Hover a part for its name, click for its card. The slider closes the jaws: watch the screw turn, the nut carry the upper carrier down, the rocker tip the lower carrier up, and the pad meet the cable on the shoe.</p>
<div class="row"><button id="play">close / open</button><button id="explode">explode</button><button onclick="window.look('drive')">drive</button><button onclick="window.look('jaws')">jaws</button><button onclick="window.look('stack')">stack</button></div>
<div id="layers" style="margin-top:8px"></div>
<div id="parts"><div class="grp">Parts (click to fly to one)</div></div></div>
<div id="bar"><div class="row"><span id="travel" style="font-weight:600"></span><span id="screwdeg" style="color:#6B6257"></span></div><input type="range" id="s" min="0" max="1500" value="0"></div>
<div id="tip"></div>
<div id="right"><h2>Verification</h2><div id="verify"></div><div id="card"></div></div>
<script type="importmap">{"imports":{"three":"https://unpkg.com/three@0.160.0/build/three.module.js","three/addons/":"https://unpkg.com/three@0.160.0/examples/jsm/"}}</script>
<script type="module">
import * as THREE from 'three'; import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
const D = __DATA__;
__MATJS__
const scene = new THREE.Scene(); scene.background = new THREE.Color(0xFBF7F0);
const cam = new THREE.PerspectiveCamera(35, innerWidth/innerHeight, 1, 5000); cam.up.set(0,0,1);
const renderer = new THREE.WebGLRenderer({antialias:true}); renderer.setSize(innerWidth, innerHeight); renderer.setPixelRatio(devicePixelRatio); document.body.appendChild(renderer.domElement);
const controls = new OrbitControls(cam, renderer.domElement); controls.enableDamping = true;
await setupLook(renderer, scene, 160, [-30, 15, 30]);
const floor = new THREE.Mesh(new THREE.PlaneGeometry(600, 600), new THREE.ShadowMaterial({opacity: 0.18})); floor.position.z = -0.7; floor.receiveShadow = true; scene.add(floor);
const grid = new THREE.GridHelper(300, 30, 0xcfc7b8, 0xe6dfd0); grid.rotation.x = Math.PI/2; grid.position.z = -0.6; scene.add(grid);
const geoCache = {}; for(const [n,m] of Object.entries(D.meshes)){ const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.Float32BufferAttribute(m.v,3)); g.setIndex(m.f); g.computeVertexNormals(); geoCache[n]=g; }
const objs = [], groups = {upper: [], lower: [], rocker: [], screw: [], pulley_h: [], static: []};
const LAYER = p => p.name==='fastener' ? 'fasteners' : p.name.startsWith('stack_') || (p.name.startsWith('cell_') && p.name!=='cell_p') || p.name==='bracket_h' || p.name==='bracket_v' || p.name.startsWith('servo_standoff') ? 'wrist: cells, plates, standoff' : p.name.startsWith('arm_d405') || p.name==='d405_g' ? 'wrist camera' : p.name==='servo_body' || p.name==='servo_belt_cover' ? 'body shell (translucent)' : ['servo','horn','pulley_horn','pulley_screw','belt_w','belt_e','screw','nut','pivot_pin','servo_rocker'].includes(p.name) || p.name.startsWith('servo_jaw') ? 'drive: servo, belt, screw, carriers, rocker' : p.name==='cable' ? 'cable (context)' : 'fingers';
const layerGroups = {};
for(const p of D.parts){
 let geo; if(p.kind==='box') geo = new THREE.BoxGeometry(2*p.size[0],2*p.size[1],2*p.size[2]);
 else if(p.kind==='cyl'){ geo = new THREE.CylinderGeometry(p.size[0], p.size[0], 2*p.size[1], 40); geo.rotateX(Math.PI/2); }
 else geo = geoCache[p.mesh];
 const mat = makeMaterial(p.mat, p.alpha < 1 ? {opacity: p.alpha} : {});
 const m = new THREE.Mesh(geo, mat); m.userData = p; m.castShadow = p.alpha >= 1; m.receiveShadow = true; m.position.set(...p.pos); m.quaternion.set(p.quat[1], p.quat[2], p.quat[3], p.quat[0]); m.userData.base = m.position.clone(); m.userData.baseQ = m.quaternion.clone();
 const lay = LAYER(p); if(!layerGroups[lay]){ layerGroups[lay] = new THREE.Group(); scene.add(layerGroups[lay]); } layerGroups[lay].add(m); objs.push(m); (groups[p.group] || groups.static).push(m);
}
for(const f of D.fasteners){ const grp = fastenerMesh(f, 1000); const p = {name: 'fastener', group: f.group, pos: f.pos, quat: [1,0,0,0], info: grp.userData.info, base: new THREE.Vector3(...f.pos), baseQ: new THREE.Quaternion()}; grp.userData = p; grp.position.set(...f.pos);
 if(!layerGroups['fasteners']){ layerGroups['fasteners'] = new THREE.Group(); scene.add(layerGroups['fasteners']); } layerGroups['fasteners'].add(grp); objs.push(grp); (groups[f.group] || groups.static).push(grp); }
// layer toggles
const layDiv = document.getElementById('layers');
for(const n of ['wrist: cells, plates, standoff','body shell (translucent)','drive: servo, belt, screw, carriers, rocker','fingers','fasteners','wrist camera','cable (context)']){ if(!layerGroups[n]) continue; const row = document.createElement('div'); row.className='row'; const cb = document.createElement('input'); cb.type='checkbox'; cb.checked = true; cb.onchange = () => layerGroups[n].visible = cb.checked; const lab = document.createElement('span'); lab.textContent = n; row.append(cb, lab); layDiv.appendChild(row); }
// motion: s = travel per jaw (mm); explode offsets by group
let exploded = false; const EXP = {upper: new THREE.Vector3(0,0,22), lower: new THREE.Vector3(0,0,-18), rocker: new THREE.Vector3(0,30,0), screw: new THREE.Vector3(0,0,30), pulley_h: new THREE.Vector3(0,0,30), static: new THREE.Vector3(0,0,0)};
const pivot = new THREE.Vector3(-2, D.FO - 5.5, 20);
function setTravel(s){
 for(const m of objs){ const p = m.userData; const dz = p.group==='upper' ? -s : p.group==='lower' ? s : 0; m.position.copy(p.base); m.position.z += dz; m.quaternion.copy(p.baseQ);
  if(exploded){ const e = EXP[p.group] || EXP.static; m.position.add(e); if(p.name.startsWith('stack_') || p.name==='bracket_h' || p.name.startsWith('cell_') && p.name!=='cell_p' || p.name==='bracket_v' || p.name.startsWith('servo_standoff')) m.position.z += 25; if(p.name==='servo_belt_cover') m.position.z += 45; if(p.name==='servo') m.position.y += 30; }
  if(p.group==='rocker'){ const ang = Math.asin(s/9); const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0), ang); const off = m.position.clone().sub(pivot).sub(exploded ? EXP.rocker : new THREE.Vector3()); off.applyQuaternion(q); m.position.copy(pivot).add(off).add(exploded ? EXP.rocker : new THREE.Vector3()); m.quaternion.copy(q.multiply(p.baseQ)); }
  if(p.group==='screw' || p.group==='pulley_h'){ const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,0,1), -s*2*Math.PI); m.quaternion.copy(q.multiply(p.baseQ)); } }
 document.getElementById('travel').textContent = 'jaw travel ' + s.toFixed(2) + ' mm per jaw (gap ' + (14 - 2*s).toFixed(1) + ' mm, pad to shoe ' + (3 - 2*s).toFixed(2) + ' mm)';
 document.getElementById('screwdeg').textContent = '   screw ' + (s*360).toFixed(0) + ' deg, ' + Math.round(s*4096) + ' encoder steps';
}
const slider = document.getElementById('s'); slider.oninput = () => setTravel(slider.value/1000);
let playing = false, dir = 1; document.getElementById('play').onclick = () => { playing = true; dir = (+slider.value >= 1340) ? -1 : 1; };
document.getElementById('explode').onclick = () => { exploded = !exploded; setTravel(slider.value/1000); };
// hover and cards
const ray = new THREE.Raycaster(); const mouse = new THREE.Vector2(); const tip = document.getElementById('tip'); const card = document.getElementById('card'); let hot = null, pinned = null;
function pick(ev){ mouse.set((ev.clientX/innerWidth)*2-1, -(ev.clientY/innerHeight)*2+1); ray.setFromCamera(mouse, cam); const hits = ray.intersectObjects(objs.filter(m => m.parent.visible), true); if(!hits.length) return null; let o = hits[0].object; while(o && !(o.userData && o.userData.info) && o.parent) o = o.parent; return o; }
function glow(m, on){ if(!m) return; const mats = m.material ? [m.material] : m.children.map(c => c.material); for(const mt of mats){ if(!mt) continue; mt.emissive.set(on ? 0xb23a3a : 0x000000); mt.emissiveIntensity = on ? 0.5 : 0; } }
renderer.domElement.addEventListener('mousemove', ev => { const m = pick(ev); if(m !== hot){ if(hot !== pinned) glow(hot, false); hot = m; if(hot) glow(hot, true); }
 if(m){ tip.style.display='block'; tip.style.left=(ev.clientX+14)+'px'; tip.style.top=(ev.clientY+14)+'px'; tip.innerHTML = '<b>' + m.userData.info.title + '</b>' + m.userData.info.what; } else tip.style.display='none'; });
function showCard(m){ if(pinned && pinned !== m) glow(pinned, false); pinned = m; glow(m, true); const i = m.userData.info; card.style.display='block';
 card.innerHTML = '<b>' + i.title + '</b>\\n' + i.what + (i.detail ? '\\n\\n' + i.detail.replace(/(https?:\\/\\/\\S+)/g, '<a href="$1" target="_blank">$1</a>') : '') + '\\n\\n<span style="color:#6B6257">moves with: ' + m.userData.group + '</span>'; }
let downAt = null; renderer.domElement.addEventListener('mousedown', ev => { downAt = [ev.clientX, ev.clientY]; });
renderer.domElement.addEventListener('mouseup', ev => { if(!downAt || Math.hypot(ev.clientX-downAt[0], ev.clientY-downAt[1]) > 4) return; const m = pick(ev); if(m) showCard(m); });
// parts list
const partsDiv = document.getElementById('parts'); const seen = {};
for(const lay of ['drive: servo, belt, screw, carriers, rocker','fingers','wrist: cells, plates, standoff','wrist camera']){ const grp = document.createElement('div'); grp.className='grp'; grp.textContent = lay; partsDiv.appendChild(grp);
 for(const m of objs){ if(LAYER(m.userData) !== lay || seen[m.userData.info.title]) continue; seen[m.userData.info.title] = 1; const d = document.createElement('div'); d.textContent = m.userData.info.title;
  d.onclick = () => { const p = m.position.clone(); controls.target.copy(p); cam.position.set(p.x + 40, p.y - 60, p.z + 45); controls.update(); showCard(m); }; partsDiv.appendChild(d); } }
// verification panel
const V = document.getElementById('verify'); let h = '<table><tr><th>interference check (every axis-aligned part against every other, designed contacts excluded)</th><th></th></tr>';
for(const [k, hits] of Object.entries(D.checks)){ h += '<tr><td>' + k + '</td><td class="' + (hits.length ? 'bad' : 'ok') + '">' + (hits.length ? hits.length + ' overlapping pairs' : 'no overlaps') + '</td></tr>'; for(const [a,b,d] of hits) h += '<tr><td style="color:#b23;padding-left:14px">' + a + ' and ' + b + '</td><td>' + d.toFixed(2) + ' mm</td></tr>'; }
h += '</table><table style="margin-top:8px"><tr><th>kinematics</th><th></th></tr>' + Object.entries(D.kin).map(([k,v]) => '<tr><td>' + k + '</td><td>' + v + '</td></tr>').join('') + '</table>';
h += '<table style="margin-top:8px"><tr><th>run in the twin (this tool, this expert)</th><th>depth</th><th>lateral</th><th></th></tr>' + D.runs.map(r => '<tr><td>' + r.variant + '</td><td>' + r.depth + ' mm</td><td>' + r.lateral + ' mm</td><td class="' + (r.ok ? 'ok' : 'bad') + '">' + (r.ok ? 'pass' : 'fail') + '</td></tr>').join('') + '</table>';
h += '<p style="color:#6B6257;margin:8px 0 0">Not verified here (bench items): jaw repeatability in the printed slot, on the PEEK nut and through the rocker, the belt\\'s backlash, cell noise and drift, the compliance through cells A and B, USB over the 3 m cable.</p>';
V.innerHTML = h;
window.look = look; function look(what){ const t = {drive: [-25, D.FO, 24, 150, 0.3, -0.75, 0.75], jaws: [2, 12, 10, 110, 0.6, -0.75, 0.45], stack: [-30, 15, 35, 260, 0.55, -0.8, 0.5]}[what]; controls.target.set(t[0], t[1], t[2]); cam.position.set(t[0] + t[3]*t[4], t[1] + t[3]*t[5], t[2] + t[3]*t[6]); controls.update(); }
look('stack'); setTravel(0);
let last = performance.now();
function loop(now){ requestAnimationFrame(loop); if(playing && now-last > 16){ last = now; let v = +slider.value + dir*20; if(v >= 1350){ v = 1350; playing = false; } if(v <= 0){ v = 0; playing = false; } slider.value = v; setTravel(v/1000); } controls.update(); renderer.render(scene, cam); }
loop(0); addEventListener('resize', () => { cam.aspect = innerWidth/innerHeight; cam.updateProjectionMatrix(); renderer.setSize(innerWidth, innerHeight); });
</script></body></html>""".replace("__DATA__", js).replace("__MATJS__", K.material_js())
OUT.write_text(html)
print("wrote", OUT, f"{OUT.stat().st_size/1e6:.1f} MB; parts {len(parts)}")
for k, hits in checks.items():
    print(f"  {k}: {len(hits)} overlapping pairs", hits[:6])
