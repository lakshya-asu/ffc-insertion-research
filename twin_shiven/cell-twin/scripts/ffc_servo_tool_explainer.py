"""The servo tool v1 design explainer: a standalone HTML page with drawings, for Shiven to learn how our own end effector works on both
arms, which parts it uses, how their specs fit, the electrics, and what the twin and the audits say. Reads the twin's review.json,
matrix.json, the camera views and the parts catalogue; runs the self-overlap audit itself.
usage: .venv/bin/python scripts/ffc_servo_tool_explainer.py [out.html]"""
import json, math, sys, html
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from cell_twin import ffc_zif as F
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path.home() / "Desktop" / "Synphony-BOM" / "FFC-servo-tool.html"
e = html.escape
R = json.loads((ROOT / "out/ffc_zif/review.json").read_text())
M = json.loads((ROOT / "out/ffc_zif/matrix.json").read_text())
P = json.loads((ROOT / "assets/ffc/parts.json").read_text())["parts"]
views_path = Path.home() / "Desktop" / "Synphony-BOM" / "FFC-camera-views" / "views.json"
VIEWS = json.loads(views_path.read_text()) if views_path.exists() else []
G = F.GRIPPERS["servo_v1"]
SV = {"Meca500": R.get("meca500/servo_v1"), "LR Mate 200iD": R.get("lrmate200id/servo_v1_lr")}
BV = {"Meca500": R.get("meca500/megp_nano25"), "LR Mate 200iD": R.get("lrmate200id/lehz16_mini40")}


def mrow(variant):
    for r in M:
        if r["variant"] == variant:
            v = r.get("verify") or {}
            if isinstance(v, str) and v.startswith("{"):
                v = eval(v)
            c = r.get("clearance") or {}
            return v, c
    return {}, {}


def table(head, rows, cls=""):
    return f"<table class='{cls}'><tr>" + "".join(f"<th>{h}</th>" for h in head) + "</tr>" + "".join("<tr>" + "".join(f"<td>{c}</td>" for c in row) + "</tr>" for row in rows) + "</table>"


def status(s):
    col = {"OK": "#2f8f5b", "CHECK": "#b7791f", "n/a": "#888"}.get(s, "#b23")
    return f"<span class='st' style='background:{col}'>{e(s)}</span>"


# ---------------------------------------------------------------- drawings (jaw frame: x down, y east, z forward; all in mm)
C = F.JAW_LO_FACE + G["jaw_sep_open"] / 2 + G["jaw_t"]           # stroke centreline above the shoe plane
C_mm = C * 1e3; bw, bd, bl, el = G["body_w"] * 1e3, G["body_d"] * 1e3, G["body_l"] * 1e3, G["end_l"] * 1e3
ZJ = 2.0; L = bl + el; px = ZJ - L - 3.0                            # body back face, flange axis (jaw z)
FO = F.FINGER_OFF * 1e3; HB = 6.35; bt = 12.7; gap = 2.0
Ls = 4.0 + 22.0                                                     # Meca500: adapter 4 + standoff 22 (flange face to the cells' top)
zj = Ls + bt + gap + bw / 2 + C_mm                                  # flange face to the shoe plane
x_up = -(C_mm + G["jaw_sep_open"] * 1e3 / 2 + G["jaw_t"] * 1e3)    # upper jaw's outer face (open)
tipz = 16.0; PAD_OPEN = F.PAD_OPEN * 1e3
w_leg = FO + bd / 2 + 4.0
cam = (-36.0, -46.0, -40.0)


def svg_electrics():
    boxes = {  # name: (x, y, w, h, title, sub)
        "pc": (20, 120, 150, 70, "PC (policy, logging)", "USB 3 to the D405; USB 2 to the tool"),
        "d405": (250, 20, 170, 60, "Intel RealSense D405", "RGB + depth, USB 3 along the arm"),
        "src": (20, 260, 150, 70, "24 V source", "LR Mate: EE connector 24 V pins; Meca500: bench 24 V 1 A supply"),
        "buck": (250, 260, 170, 60, "Pololu D36V28F5", "24 V to 5 V, 3.2 A"),
        "mcu": (480, 120, 190, 80, "QT Py RP2040 (tool MCU)", "1 kHz loop, pinch control, USB CDC frames"),
        "adc1": (740, 40, 190, 60, "ADS122C04 board 1", "cell A, cell B at 1 kSPS each"),
        "adc2": (740, 120, 190, 60, "ADS122C04 board 2", "cell P (pinch) at 1 kSPS"),
        "servo": (740, 220, 190, 70, "XL330-M077-T", "5 V, TTL half duplex, 0.2 A typical, 1.47 A stall"),
        "cells": (980, 60, 90, 130, "3 x TAL220B", "1 mV/V bridges"),
    }
    g = ["<svg viewBox='0 0 1100 350' width='100%' style='max-width:1100px;background:#fffdf8;font-size:12px'>"]
    for k, (x, y, w, h, t, s) in boxes.items():
        g.append(f"<rect x='{x}' y='{y}' width='{w}' height='{h}' rx='8' fill='#f6efe2' stroke='#7a5520'/><text x='{x+w/2}' y='{y+22}' text-anchor='middle' font-weight='600'>{e(t)}</text>")
        words = s.split(); line = ""; ly = y + 40
        for wd in words:
            if len(line) + len(wd) > 30:
                g.append(f"<text x='{x+w/2}' y='{ly}' text-anchor='middle' fill='#444' font-size='11'>{e(line)}</text>"); ly += 14; line = ""
            line += (" " if line else "") + wd
        g.append(f"<text x='{x+w/2}' y='{ly}' text-anchor='middle' fill='#444' font-size='11'>{e(line)}</text>")
    def wire(x1, y1, x2, y2, label, col="#333", dash=""):
        g.append(f"<line x1='{x1}' y1='{y1}' x2='{x2}' y2='{y2}' stroke='{col}' stroke-width='2' {dash}/><text x='{(x1+x2)/2}' y='{(y1+y2)/2-6}' text-anchor='middle' fill='{col}' font-size='11'>{e(label)}</text>")
    wire(170, 140, 250, 55, "USB 3 cable along the arm (D405)", "#1f2126")
    wire(170, 160, 480, 160, "USB 2 full speed, one shielded pair of the tool cable (3 m)", "#1f6fb2")
    wire(170, 295, 250, 290, "24 V pair, same tool cable", "#b23")
    wire(420, 290, 740, 255, "5 V to the servo", "#b23")
    wire(335, 260, 480, 200, "5 V to the MCU", "#b23")
    wire(670, 150, 740, 70, "I2C", "#333")
    wire(670, 160, 740, 150, "I2C", "#333")
    wire(670, 180, 740, 250, "TTL data (one line), 5 V bus", "#333")
    wire(930, 70, 980, 100, "", "#333"); wire(930, 150, 980, 150, "", "#333")
    g.append("<text x='20' y='340' fill='#444' font-size='11'>One 4-core shielded cable along the arm (igus CF211-02-02-02, 6.5 mm, 51 g/m): pair 1 carries 24 V, pair 2 the USB data; the cable shield is the USB ground reference, bonded to the arm at one end. The D405 keeps its own USB 3 cable. Everything on the tool runs from the one 5 V rail (under 2 A worst case).</text>")
    g.append("</svg>")
    return "".join(g)


# ---------------------------------------------------------------- content
def compare_table():
    sm, sl, bm, bl = SV["Meca500"], SV["LR Mate 200iD"], BV["Meca500"], BV["LR Mate 200iD"]
    rows = [
        ["gripper", "MEGP 25E ($1,675) on the Meca500; SMC LEHZ16 with its JXC91 driver (about $1,300) on the LR Mate: bought electric parallel grippers", "printed body with one XL330-M077-T servo ($27), an M4 x 1 lead screw, a PEEK nut, a 6 mm rail and a printed rocker: about $90"],
        ["force sensing", "ATI Nano25 (Meca500) or Mini40 (LR Mate) six-axis sensor with its Net F/T box: $7,000 to $8,000", "three TAL220B bar load cells ($15 each) on two ADS122C04 24-bit boards ($20 each) and a QT Py RP2040 ($10)"],
        ["between the flange and the gripper", "adapter plate, the sensor puck, a 90 degree 6061 bracket", "adapter plate, a printed standoff, the two wrist cells joined by 4 mm plates (they are the bracket)"],
        ["fingers: shoe, lip, pad, legs, plates", "ours, printed PAHT-CF and 0.5 mm shim steel", "the same parts; the upper plate's rear 12.7 mm is cell P"],
        ["jaw motion", "both jaws 1.5 mm, 3 mm gap", "the same: both jaws 1.5 mm, coupled by the rocker"],
        ["wrist camera", "D405 on a side plate", "the same, 15 mm further forward (81 mm from the tips)"],
        ["bench cameras, light, laptop fixture, expert", "as designed", "unchanged"],
        ["cables along the arm", "three: gripper, sensor, camera", "two: one 4-core tool cable (24 V + USB data), camera"],
        ["mass below the flange", f"{bm['mass_g'] if bm else '?'} g (Meca500), {bl['mass_g'] if bl else '?'} g (LR Mate)", f"{sm['mass_g'] if sm else '?'} g (Meca500), {sl['mass_g'] if sl else '?'} g (LR Mate)"],
        ["what the sensor measures", "six axes, 7 kHz, 10 mN", "three axes (insertion, vertical, pinch), 1 kHz, about 70 mN peak to peak"],
        ["grip force", "set open loop: MEGP in 0.4 N steps, LEHZ16 plus or minus 2.4 N", "closed loop on cell P, target plus or minus 0.3 N at 5 N"],
        ["insertion precision", "set by the arm and the two bench cameras", "unchanged: the same arm and cameras"],
        ["parts cost per arm", "about $8,700 (Meca500), $9,300 (LR Mate)", "about $340 for the first tool"],
        ["lead time", "stock to 6 weeks", "servo about 2 months (sold out at ROBOTIS America today); the rest in stock"],
        ["risk", "none: catalogue parts with published specs", "engineering: print, machine, and the bench tests in section 10 before it replaces a bought stack"],
    ]
    return table(["", "bought stack", "servo tool v1"], [[f"<b>{e(a)}</b>", e(b), e(c)] for a, b, c in rows])


def review_items(key, areas=None):
    r = R.get(key) or {}
    return [x for x in r.get("rows", r.get("items", [])) if areas is None or x.get("area") in areas]


def verify_table():
    rows = []
    for v in ("meca500-servo_v1-laptop", "meca500-servo_v1-platform", "lrmate200id-servo_v1_lr-laptop", "meca500-megp_nano25-laptop", "lrmate200id-lehz16_mini40-laptop"):
        ver, clr = mrow(v)
        if not ver:
            rows.append([e(v), "not run yet", "", "", "", ""]); continue
        pen = [p for p in (clr.get("penetrations") or clr.get("pairs") or []) if (p.get("dist", 0) if isinstance(p, dict) else 0) < 0] if isinstance(clr, dict) else []
        ok = bool(ver.get("depth_ok")) and bool(ver.get("latch_closed")) and abs(float(ver.get("lateral_mm") or 0)) <= 0.2
        rows.append([e(v), f"{ver.get('depth_mm')} mm", f"{ver.get('lateral_mm')} mm", "yes" if ver.get("latch_closed") else "no", "none" if not pen else f"{len(pen)} pairs", status("OK" if ok else "FAIL")])
    return table(["variant", "insertion depth (3.5 needed)", "lateral at the mouth (0.2 window)", "latch closed", "clearance penetrations", ""], rows)


def overlap_counts():
    try:
        sys.path.insert(0, str(ROOT / "scripts")); import ffc_zif_selfcheck as SC
        import io, contextlib
        out = {}
        for v in (("meca500", "servo_v1", "laptop"), ("meca500", "servo_v1", "platform"), ("lrmate200id", "servo_v1_lr", "laptop")):
            with contextlib.redirect_stdout(io.StringIO()):
                out["-".join(v)] = len(SC.check(*v))
        return out
    except Exception as ex:
        return {"error": str(ex)}


def camera_rows():
    rows = []
    for vv in VIEWS:
        if "servo" not in vv.get("variant", ""):
            continue
        hw = {}; hidden = {}
        for x in vv["rows"]:
            hw[x["camera"]] = max(hw.get(x["camera"], 0), x["hardware_frac"])
            for pn, p in x["points"].items():
                if p["in_view"] and not p["visible"] and p["by"] and not p["by"].startswith(("ffc_", "zif", "lid")):
                    hidden[x["camera"]] = hidden.get(x["camera"], 0) + 1
        rows.append([e(vv["variant"])] + [f"{hw.get(c, 0)*100:.0f}% of the frame, {hidden.get(c, 0)} task points hidden" for c in ("macro_e", "macro_w", "wrist_d405")])
    return table(["variant", "east bench camera", "west bench camera", "wrist D405"], rows) if rows else "<p class='n'>camera views for the servo variants not rendered yet (the camera check runs after the matrix).</p>"


def mass_rows():
    rows = []
    for arm in ("Meca500", "LR Mate 200iD"):
        s, b = SV[arm], BV[arm]
        if not s:
            rows.append([arm, "review not run yet", "", ""]); continue
        off = (s["com_mm"][0] ** 2 + s["com_mm"][1] ** 2) ** 0.5
        lim = "500 g payload; Fig. 19: 40 mm below the flange at 75% acceleration, 60 mm at 50%" if arm == "Meca500" else "7 kg payload; 16.6 N m at J5"
        rows.append([arm, f"{s['mass_g']} g (bought stack {b['mass_g']} g)", f"{s['com_mm'][2]:.0f} mm below the flange, {off:.0f} mm off axis", e(lim)])
    return table(["arm", "mass below the flange", "centre of mass", "limit"], rows)


def spec_rows():
    s_m = SV["Meca500"] or {}; s_l = SV["LR Mate 200iD"] or {}
    pinch_T = 5.0 * 1.0e-3 / (2 * math.pi * 0.3)   # N m at the screw for 5 N on a 1 mm lead at 30 percent efficiency
    rows = [
        ["XL330-M077-T servo", "torque at the screw for the 5 N pinch", f"{pinch_T*1e3:.1f} mN m (1 mm lead, 30% efficiency)", "215 mN m stall at 5 V", "OK", "ROBOTIS e-manual"],
        ["", "jaw speed and the release", "6.4 mm/s: 1.5 mm per jaw in 0.23 s", "0.5 s release in the expert; the M288 would take 0.9 s", "OK", "383 rpm no load"],
        ["printed rocker (couples the two carriers)", "load", "the 5 N pinch reaction and the lower finger's 20 g; pins 2 mm steel", "PAHT-CF, 10 x 4 mm lever", "CHECK", "bench: backlash at the lower jaw under 0.05 mm"],
        ["", "supply", "5 V from the buck; 0.2 A typical, 1.47 A at stall", "3.7 to 6.0 V", "OK", "e-manual"],
        ["", "position resolution at the jaw", "0.24 um per step; about 0.04 N per step through the pad", "4096 steps per turn", "OK", "arithmetic"],
        ["M4 x 1 lead screw 90500A160", "self-locking (holds the pinch unpowered)", "lead angle 4.6 deg", "friction angle PEEK on steel about 8 deg", "OK", "arithmetic; McMaster page"],
        ["", "axial force at servo stall", "about 400 N", "5 N used", "OK", "arithmetic"],
        ["PEEK nut 90500A310", "backlash", "not published (wear-compensating O-ring)", "jaw repeatability 0.05 mm wanted", "CHECK", "bench: dial gauge"],
        ["printed slot (carrier guide)", "load", "upper: finger 44 g + pinch 5 N + slide shear 4 N, about 10 N; lower: the 20 N latch press on 16 x 14 mm faces, under 0.1 MPa", "PAHT-CF, 100 MPa class", "CHECK", "repeatability 0.05 to 0.1 mm: bench item"],
        ["GT2 belt and 12T pulleys", "backlash and tension", "1:1, about 68 mm loop; the belt only turns the screw, it carries no jaw load", "", "CHECK", "bench: backlash at the jaw under 0.05 mm"],
        ["TAL220B 5 kg cells (x3)", "range vs the worst load", "20 N latch press (41%), 12 N lid push, 5 N pinch", "49 N rated, 59 N safe overload", "OK", "SparkFun datasheet"],
        ["", "resolution for the guards", "about 0.07 N peak to peak at 1 kSPS (ADS122C04); 0.03 to 0.05 N with the ADS1256", "0.1 N at 1 kHz", "OK", "TI noise tables"],
        ["", "compliance at the tips", "0.005 mm at 0.5 N slide friction, 0.04 mm at the 4 N guard, 0.12 mm at the 12 N push", "0.2 mm lateral window; cameras re-measure after the grasp", "CHECK", "about 0.5 mm at rating, ESTIMATE; bench: dial gauge"],
        ["ADS122C04 boards (x2)", "sample rate", "2 kSPS turbo, two channels each: 1 kSPS per cell", "1 kHz loop", "OK", "TI datasheet"],
        ["QT Py RP2040", "loop and bus", "1 kHz ADC reads, Dynamixel TTL at 1 Mbaud, USB CDC", "", "OK", "Adafruit page"],
        ["Pololu D36V28F5", "current", "under 2 A worst case (servo stall + boards)", "3.2 A", "OK", "Pololu page"],
        ["igus CF211-02-02-02 cable", "USB full speed over 3 m of shielded pair", "12 Mb/s, one pair + shield", "", "CHECK", "bench; RS-485 on the same MCU is the fallback"],
        ["D405 wrist camera", "distance to the tips", "81 mm", "70 mm minimum range", "OK", "Intel datasheet"],
        ["Meca500 flange", "mass below the flange", f"{s_m.get('mass_g', '?')} g", "500 g", "OK" if s_m and s_m["mass_g"] < 500 else "CHECK", "manual 2026.B"],
        ["", "moment at the flange, 20 N press", "1.56 N m (20 N at 78 mm)", "3.4 N m", "OK", "manual"],
        ["", "centre of mass (Fig. 19)", f"{s_m['com_mm'][2]:.0f} mm below, {(s_m['com_mm'][0]**2+s_m['com_mm'][1]**2)**0.5:.0f} mm off axis" if s_m else "?", "60 mm below at 50% acceleration", "OK" if s_m and s_m["com_mm"][2] < 60 else "CHECK", "manual Fig. 19"],
        ["", "M3 screws into the flange", "4 mm engagement, 1.5 N m", "4 mm max", "OK", "manual"],
        ["LR Mate 200iD", "mass below the flange", f"{s_l.get('mass_g', '?')} g", "7 kg", "OK", "FANUC datasheet"],
        ["", "EE connector 24 V budget for the tool", "about 0.5 A at 24 V worst case", "not stated in the manual we have", "CHECK", "ask FANUC or use the bench supply"],
    ]
    return table(["part", "what is checked", "in use", "limit", "", "source"], [[e(a), e(b), e(c), e(d), status(s), e(f)] for a, b, c, d, s, f in rows])


def parts_rows():
    ids = ["xl330_m077", "lead_screw_m4x1", "nut_peek_m4x1", "rail_sseb6", "load_cell_tal220b", "adc_ads122c04", "mcu_qtpy_rp2040", "buck_d36v28f5", "tool_cable_cf211", "servo_tool_v1", "servo_jaw_l", "bracket_servo", "standoff_servo", "adapter_plate_meca_servo", "adapter_plate_lrmate_servo"]
    qty = {"load_cell_tal220b": 3, "adc_ads122c04": 2}
    rows = []
    for i in ids:
        p = P.get(i)
        if not p:
            continue
        price = p.get("price_usd_if_listed")
        rows.append([f"<a href='{e(p['buy_url'])}'>{e(p['name'])}</a>" if p.get("buy_url") and p["buy_url"] not in ("none", "") else e(p["name"]), e(str(p.get("part_number", ""))), str(qty.get(i, 1)), f"${price:,.2f}" if isinstance(price, (int, float)) else e(str(p.get("notes", ""))[:60] if "ESTIMATE" in str(p.get("notes", "")) else "ours / quote"), e(str(p.get("what_it_is", "")))])
    return table(["part", "number", "qty", "unit price today", "what it does"], rows)


# ---------------------------------------------------------------- page
ov = overlap_counts()
page = f"""<!doctype html><html><head><meta charset='utf-8'><title>Servo tool v1: design explainer</title>
<style>
body{{font:15px/1.55 -apple-system,Inter,Helvetica,sans-serif;margin:0;background:#FDF9F2;color:#222}} main{{max-width:1140px;margin:0 auto;padding:28px 20px 60px}}
h1{{font-size:30px;margin:0 0 6px}} h2{{font-size:22px;margin:40px 0 10px}} h3{{font-size:17px;margin:22px 0 8px}} p{{margin:8px 0}}
.sheet{{background:#F6EFE2;border-radius:12px;padding:16px 18px;margin:14px 0}} .eyebrow{{font-size:11px;letter-spacing:.08em;text-transform:uppercase;color:#7a5520;margin-bottom:6px}}
table{{border-collapse:collapse;width:100%;margin:10px 0;font-size:13.5px}} th,td{{text-align:left;vertical-align:top;padding:7px 9px;border-bottom:1px solid #EFE6D6}} th{{font-weight:600;background:#F6EFE2}}
.st{{color:#fff;font-size:11px;padding:2px 7px;border-radius:6px}} .n{{color:#666;font-size:13px}} .grid{{display:grid;grid-template-columns:1fr 1fr;gap:14px}} @media(max-width:800px){{.grid{{grid-template-columns:1fr}}}}
a{{color:#1f6fb2}} .big{{font-size:17px}} ul{{margin:6px 0 6px 20px}} li{{margin:3px 0}}
</style></head><body><main>
<h1>Servo tool v1: how our own end effector works</h1>
<p class='n'>Synphony Co, 2026-10-13. Built from the twin (cell_twin/ffc_zif.py), its audits (out/ffc_zif/review.json, matrix.json, FFC-camera-views) and the research notes in docs/research/2026-10/research_notes (custom_end_effector_requirements, servo_tool_v1_design_numbers, servo_tool_v1_parts, custom_servo_end_effector). Companion pages: <a href='FFC-final-setup.html'>FFC-final-setup.html</a> (the bought stacks, section 3c compares), <a href='FFC-twin-3d-meca500-servo.html'>3D viewer Meca500</a>, <a href='FFC-twin-3d-lrmate-servo.html'>3D viewer LR Mate</a>, <a href='FFC-custom-parts/servo_v1/index.html'>STEP files</a>, <a href='FFC-camera-views/index.html'>camera views</a>.</p>

<div class='sheet'><div class='eyebrow'>In one paragraph</div>
<p class='big'>One small bus servo turns a 1 mm pitch lead screw; its nut carries the upper jaw, and a printed rocker lever couples the lower jaw to it, so both jaws move 1.5 mm and close the 3 mm gap on the cable, the same motion as the bought grippers. Three $15 bar load cells read every force the task cares about at 1 kHz: two at the wrist read the insertion axis and the vertical axis, one in the upper finger reads the pinch, so the grip is closed on a measured 5 N instead of set open loop. It bolts to either arm through a 4 mm adapter plate and a printed standoff, needs one 4-core cable along the arm, weighs about {SV['Meca500']['mass_g'] if SV['Meca500'] else '300'} g with the wrist camera, and costs about $340 in parts against about $8,700 for the bought gripper and six-axis sensor. It replaces both. It is not more precise at the insertion, because that precision comes from the arm and the two bench cameras; it is cheaper, lighter and good enough on force.</p></div>

<h2>1. What changed against the bought stacks, part by part</h2>
<img src='FFC-servo-tool-figs/meca500_stack.png' style='width:100%'>
<p class='n'>Left: the bought Meca500 stack (adapter plate, ATI six-axis sensor, 90 degree bracket, MEGP 25E gripper). Right: the servo tool on the same arm at the same instant of the same run. Everything from the fingers down is the same part; everything between the flange and the fingers is replaced. Rendered from the twin with the robot arm hidden, labels on each part.</p>
{compare_table()}
<img src='FFC-servo-tool-figs/lrmate_stack.png' style='width:100%'>
<p class='n'>The LR Mate 200iD pair: SMC LEHZ16 gripper and ATI Mini40 on the left, the servo tool on the right. Only the adapter plate (ISO 31.5 pattern, 44 x 6 mm) and the standoff (36 x 34 mm) differ from the Meca500 build.</p>

<h2>2. What a servo is, and why this one</h2>
<p>A servo is a motor, a gearbox, a position encoder and a controller in one case. You send it a target position over a serial bus and it goes there and holds, reporting its position and motor current back. The <b>Dynamixel XL330-M077-T</b> (ROBOTIS, $27.49, 18 g, 20 x 34 x 26 mm) runs on 5 V, has 4,096 encoder steps per turn, 0.215 N m of stall torque and 383 rpm with no load, and speaks a 1 Mbaud half-duplex TTL bus that a $10 microcontroller drives directly. We chose the fast M077 gearing over the strong M288 because the task needs speed, not torque: on a 1 mm lead the pinch takes 1 percent of the stall torque, while the release (1.5 mm per jaw) has a half-second budget in the run (M077: 0.23 s; M288: 0.9 s). Its current reading is <b>not</b> used as a force signal: measured Dynamixel force-by-current has a 0.45 N deadband and 7 to 19 percent hysteresis (TF-Gripper, arXiv 2602.10013), and through a lead screw it would be worse. Force comes from the load cells.</p>

<h2>3. The mechanism</h2>
<img src='FFC-servo-tool-figs/meca500_fingers.png' style='width:100%'>
<p class='n'>What the west bench camera sees at the slide into the connector: the lower finger's shoe under the cable, the pad on top, the one east leg, the lid standing open. Identical on both sides; on the servo tool the upper finger bolts to cell P and the lower finger to the rocker-driven carrier.</p>
<div class='grid'>
<div class='sheet'><div class='eyebrow'>Jaw drive</div><p>The servo lies flat in a pocket in the printed body with its horn up; a <b>3 mm GT2 belt</b> over the top (under a 6 mm cover) turns the <b>M4 x 1 mm lead screw</b> (McMaster 90500A160, 17-4 PH stainless) 1:1, and the screw runs down through the body's top wall into the upper jaw carrier. A <b>PEEK anti-backlash nut</b> (90500A310) is bonded in that carrier, and the screw through it is the carrier's guide; both carriers (16 x 14 x 4 mm) run in the printed slot through the front of the body, and a <b>printed rocker</b> pivoted on a 2 mm pin through the end block rides in a notch in each carrier and moves the lower one the opposite way, so one right-hand screw drives both jaws symmetrically. The working 3D schematic with the interference check: <a href='FFC-servo-tool-3d.html'>FFC-servo-tool-3d.html</a>. One turn moves the upper jaw 1 mm, one encoder step 0.24 um. The 4.6 degree lead angle is below the friction angle, so the screw holds the pinch with the power off. Closing (1.5 mm per jaw) takes 0.23 s.</p></div>
<div class='sheet'><div class='eyebrow'>Jaws and fingers</div><p>The <b>lower jaw carrier</b> carries the lower finger (the printed plate that bridges 4 mm over the cable, the one east leg, the 0.5 mm shim-steel shoe with its 1.5 mm lip) with the MEGP finger pattern, M4 plus two 1.5 mm pins; the 20 N latch press goes through it into the slot walls of the end block. On the <b>upper jaw carrier's</b> outer face sits the third load cell, and on the cell's free end the printed upper plate, the arm 12 to 15 mm ahead of the jaw line and the 30 x 6 mm pad with its silicone face. Open gap 3 mm, stroke 3 mm, both unchanged from the bought stacks, so the shoe, lip, pad and legs are the same parts that pass the task there.</p></div>
</div>
<img src='FFC-servo-tool-figs/lrmate_fingers.png' style='width:100%'>
<div class='sheet'><div class='eyebrow'>Why the body sits beside the cable and the flange over it</div><p>The connector's lid and the floating cable leave no room above the mouth, so every stack puts the tool body 26 mm east of the cable centreline and reaches the fingers back west over the cable. The arm's flange stays on the cable line (where the IK wants it); the 55 mm cell A is what carries the body east, which is exactly where a 90 degree bracket was on the bought stacks. The LR Mate uses a 36 x 34 mm standoff and a 44 x 6 mm adapter plate instead of 28 x 22 and 28 x 4, so its jaw line sits where the Mini40 stack has it and the wrist stays out of the west bench camera during the lid lift (a 14 mm standoff put it in the frame).</p></div>

<h2>4. Force sensing: three cells, three jobs</h2>
<table><tr><th>cell</th><th>where</th><th>bends along</th><th>what it reads</th><th>what loads it in the run</th></tr>
<tr><td><b>A</b></td><td>under the standoff, running east (its fixed end is the only part on the arm side)</td><td>the body axis (insertion direction)</td><td>every force at the tips along the insertion axis: slide friction, the seat rise, the lid push</td><td>0.5 N friction, 1 to 3 N seat rise, 4 N slide guard, 7 N lid push, 12 N guard</td></tr>
<tr><td><b>B</b></td><td>off A's free end, running along the body beside its east face</td><td>the flange axis (vertical)</td><td>every vertical force at the tips: lid lift, latch press</td><td>2 N lift, 10 N press, 20 N worst, plus the 1.5 N weight of what hangs on it (zeroed at the approach pose)</td></tr>
<tr><td><b>P</b></td><td>the upper finger's cross member, fixed end at the moving jaw, pad on its free end</td><td>the stroke (vertical at the pad)</td><td>the pinch on the cable, directly</td><td>5 N target, closed loop: the servo steps until P reads 5 N, then the screw holds</td></tr></table>
<div class='grid'>
<div class='sheet'><div class='eyebrow'>Why P exists</div><p>A and B carry the whole tool, so the pinch, an internal force between the two fingers, cancels in them. Setting the pinch by position alone would not work: the silicone pad is about 180 kN/m stiff, so 5 N is 28 um of squeeze, and the stiffener's own thickness tolerance (0.3 plus or minus 0.03 mm) would move an open-loop pinch by about 5 N. With P the loop closes on the measured force to within the cell's noise.</p></div>
<div class='sheet'><div class='eyebrow'>What is lost against the ATI sensor</div><p>Six axes become three (no lateral force, no torques), 7 kHz becomes 1 kHz, 10 mN becomes about 70 mN peak to peak, and the bars flex: a TAL220B moves about 0.5 mm at its 49 N rating, so the tips move about 0.04 mm under the 4 N slide guard and 0.12 mm under the 12 N lid push, where the ATI moves nothing. The guards only need 0.1 N at 1 kHz, the bench cameras re-measure the tip after the grasp, and the push happens when tip position no longer matters. Gravity compensation becomes trivial: a fixed 150 g hangs on A and B, zeroed at each approach pose.</p></div></div>

<h2>5. The run, phase by phase, and what each cell says</h2>
<table><tr><th>phase</th><th>what the arm does</th><th>what the tool does</th><th>cell A</th><th>cell B</th><th>cell P</th></tr>
<tr><td>1 latch open</td><td>hooks the lip 1 mm under the lid's front edge, lifts it along its arc to 100 deg, tilting the tool up to 20 deg</td><td>jaws open; the shoe's lip does the work</td><td>about 0</td><td>+2 N lift (guard 20 N)</td><td>0</td></tr>
<tr><td>grasp</td><td>slides the shoe 14 mm under the stiffener tip, 0.45 mm below it</td><td>both jaws close 1.5 mm at 6.4 mm/s until P reads 5 N; the screw locks</td><td>about 0</td><td>cable preload under 1 N</td><td>rises to 5 N, holds</td></tr>
<tr><td>2 slide</td><td>carries the tip to the mouth (cameras check lateral and yaw), slides 3.5 mm at 2 mm/s</td><td>holds the pinch</td><td>0.5 N friction, then the 1 to 3 N seat rise; stop at 4 N</td><td>about 0</td><td>5 N</td></tr>
<tr><td>release</td><td>lifts 1.5 mm while the jaws open (the shoe would otherwise drop into the board), then slides the shoe out east 26 mm</td><td>jaws open 1.5 mm each in 0.23 s</td><td>0</td><td>0</td><td>falls to 0</td></tr>
<tr><td>3 latch close</td><td>pushes the lid forward with the shoe's rear edge, then presses it flat</td><td>jaws open; the shoe edge does the work</td><td>7 N push, guard 12 N</td><td>10 N press, guard 10 N, 20 N worst</td><td>0</td></tr></table>

<h2>6. Electrics</h2>
{svg_electrics()}
<div class='grid'>
<div class='sheet'><div class='eyebrow'>Power</div><p>One 24 V pair comes down the tool cable: on the <b>LR Mate</b> from the forearm EE connector's 24 V and 0 V pins (the manual does not state the current budget; the tool needs about 0.5 A worst case, to confirm with FANUC or feed from the bench), on the <b>Meca500</b> from a bench 24 V 1 A supply, because the Meca500's wrist port is for Mecademic tools only. A Pololu D36V28F5 makes 5 V at up to 3.2 A on the tool. The servo draws 0.2 A in normal use and 1.47 A only at stall, which the pinch never approaches; the boards take 0.1 A.</p></div>
<div class='sheet'><div class='eyebrow'>Signals</div><p>The QT Py RP2040 reads the two ADS122C04 boards over I2C at 1 kHz (each board samples two cells at 1 kSPS in turbo mode), runs the pinch loop (step the servo until P reads the target, then hold), drives the servo over its one-wire TTL bus, and streams a 1 kHz frame (three forces, servo position and current, status) to the PC as USB CDC serial over the cable's second shielded pair. Commands back: open, close to a pinch target, zero the cells. USB full speed over 3 m of twisted pair is a bench item; if it is marginal, the same board drives an RS-485 transceiver instead. The D405 keeps its own USB 3 cable. ESD: the steel shoe is grounded through the plates and the arm; the cable shield bonds to the arm at one end.</p></div></div>

<h2>7. Does every spec fit? (verified against the datasheets and the twin)</h2>
{spec_rows()}
<h3>Mass and centre of mass, from the twin's review</h3>
{mass_rows()}

<h2>8. What we buy, and what we make</h2>
{parts_rows()}
<p class='n'>Printed in PAHT-CF (Bambu, 0.6 mm hardened nozzle, enclosed printer): body, belt cover, two jaw carriers, rocker, standoff, finger plates. Machined 6061, 4 mm: adapter plate, corner plate, tab and side plate. Cut from 0.5 mm shim steel: the shoe with its 1.5 mm lip (no catalogue part has the lip). STEP files: <a href='FFC-custom-parts/servo_v1/index.html'>FFC-custom-parts/servo_v1</a>. Sourced prices, stock and lead times with 71 sources: docs/research/2026-10/research_notes/servo_tool_v1_parts.md. The one lead-time risk is the servo: ROBOTIS America shows every XL330 line sold out with about 2 months to ship; everything else is in stock or a quote away. First-tool parts about $340 ($92 of it ESTIMATE lines where McMaster and Misumi prices need a browser), plus the D405 already on order.</p>

<h2>9. Tested in the twin</h2>
<p>The same expert, connector, cable and laptop as the bought stacks; only the tool changed. Pass means 3.5 mm of insertion, the tip inside the 0.2 mm lateral window at the mouth and the latch closed at the end.</p>
{verify_table()}
<p>Self-overlap audit (every part against every other at the home pose): {e(json.dumps(ov))} overlapping pairs. Camera audit along the run (share of each frame covered by the tool's own hardware, and task points hidden behind it):</p>
{camera_rows()}
<h3>The review's open rows for the servo tool</h3>
{table(["area", "item", "value", "limit", "", "note"], [[e(x["area"]), e(x["item"]), e(str(x["value"])), e(str(x["limit"])), status(x["status"]), e(x.get("note", ""))] for x in review_items("meca500/servo_v1") if x["status"] == "CHECK"]) if SV["Meca500"] else "<p class='n'>review not run yet</p>"}

<h2>10. What is still an assumption, and how we close each one on the bench</h2>
<table><tr><th>assumption</th><th>test</th><th>pass</th></tr>
<tr><td>jaw repeatability of a Dynamixel on the PEEK nut, and the rocker's backlash at the lower jaw (no published figures)</td><td>dial gauge on each jaw, 50 open-close cycles</td><td>under 0.05 mm</td></tr>
<tr><td>pinch loop on cell P holds the force</td><td>close on a 0.3 mm shim on a scale, hold an hour, repeat at 0.27 and 0.33 mm</td><td>5 N plus or minus 0.3 N</td></tr>
<tr><td>cell noise and drift with the ADS122C04 at 1 kSPS</td><td>log A, B, P unloaded for an hour</td><td>under 0.1 N peak to peak, drift under 0.1 N per hour</td></tr>
<tr><td>compliance through A and B</td><td>load the tips with 4 N along the slide and 20 N down, dial gauge at the tips</td><td>under 0.05 mm at 4 N; the press figure recorded</td></tr>
<tr><td>crosstalk: a 12 N push read on B, a 20 N press read on A</td><td>same rig</td><td>under 0.3 N</td></tr>
<tr><td>USB full speed over the 3 m tool cable</td><td>stream 1 kHz frames for an hour</td><td>no dropped frames; else RS-485</td></tr>
<tr><td>the LR Mate EE connector's 24 V current budget</td><td>ask FANUC, or measure the tool's draw and compare</td><td>0.5 A available</td></tr>
<tr><td>pad stiffness and the force-per-step figure (Shore A 40 assumed)</td><td>push the pad on a scale by known steps</td><td>recorded; sets the loop gain</td></tr>
<tr><td>printed body stiffness under the 20 N press (PAHT-CF 7 GPa assumed)</td><td>press the shoe on a scale, dial gauge at the body</td><td>under 0.1 mm</td></tr></table>
<p class='n'>Also carried over from the bought stacks and unchanged: the real CON3104 lid must overhang the slot by 0.5 mm for the lip; board flatness under the shoe; components beside the connector under the 46 mm pad span; the sideways exit path 45 mm east.</p>
</main></body></html>"""
OUT.write_text(page)
print("wrote", OUT, f"{OUT.stat().st_size/1e3:.0f} kB")
