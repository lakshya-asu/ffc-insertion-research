"""The final-setup page: both arm setups with every number that decides them (precision, force, mass, cameras), the proof videos, the
leader for teleop, and the complete order list with links. Reads out/ffc_zif/{review,matrix,design}.json, the camera check and sweep
JSONs, and assets/ffc/{parts,vision_parts}.json.
usage: .venv/bin/python scripts/ffc_final_setup.py  -> ~/Desktop/Synphony-BOM/FFC-final-setup.html"""
import json, html, sys, os
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from cell_twin import ffc_zif as F
OUT = ROOT / "out" / "ffc_zif"; DESK = Path.home() / "Desktop" / "Synphony-BOM"; DEST = DESK / "FFC-final-setup.html"
P = json.loads((ROOT / "assets" / "ffc" / "parts.json").read_text())["parts"]
V = json.loads((ROOT / "assets" / "ffc" / "vision_parts.json").read_text())
R = json.loads((OUT / "review.json").read_text())
M = json.loads((OUT / "matrix.json").read_text())
views = json.loads((DESK / "FFC-camera-views" / "views.json").read_text()) if (DESK / "FFC-camera-views" / "views.json").exists() else []
sweep = json.loads((DESK / "FFC-camera-views" / "sweep.json").read_text()) if (DESK / "FFC-camera-views" / "sweep.json").exists() else {}
e = html.escape
CM = F.CAMERAS["meca500"]

def vpart(sec, k):
    return V.get(sec, {}).get(k, {})

# ----------------------------------------------------------------------------------------------------------------- the setups
SETUPS = {
    "meca500": dict(title="Setup A: Mecademic Meca500 R4", review="meca500/megp_nano25",
                    arm=dict(name="Meca500 R4 six-axis arm with the PS200-R4 power module, SKU 9100-002", payload="0.5 kg rated (1 kg with testing, lower acceleration)", reach="330 mm", repeat="0.005 mm", mass="4.6 kg", flange="3.4 N m moment limit, 20 mm flange, 4 x M3 on 15.5 mm PCD, 9 H6 bore",
                             price="quote only; EUR 17,960 to 19,860 ex VAT in Germany, about $18,000 to $23,000 with power supply and cables (research 2026-10-12)",
                             links=[("Electromate, US distributor, configure and request a quote", "https://www.electromate.com/meca500/"), ("Mecademic product page (Get in Touch for a direct quote)", "https://mecademic.com/industrial-robots-mecademic/meca500-industrial-robot-arm/"), ("Manual 2026.B (specs, flange drawing, payload charts)", "https://resources.mecademic.com/en/doc/MC-UM-MECA500/2026.B/mc-um-meca500.pdf"), ("Mechatronic Solutions (US reseller)", "https://mechatronicsolutions.com/product/mecademic-meca500/")],
                             contact="Electromate US office Greenville DE, 877.378.0240. Ask for a written ship date; no US channel publishes stock."),
                    gripper=dict(id="megp25e", stroke="3 mm per jaw (we use 1.5 mm)", force="40 N max, set 1 to 100 percent in firmware: 5 N = 12 percent, 0.4 N steps", repeat="0.03 mm", mass="106 g", fingers="20 g and 30 mm reach per finger max (ours 3 g and 5 g, 28 mm)", control="Meca500 tool port, MGC-HFC coiled cable (MGC-SS35 and MGC-AA25 are discontinued)",
                                 price="$1,675 (Electromate listing)", links=[("Electromate MEGP 25E", "https://www.electromate.com/megp-25e-electric-parallel-gripper/"), ("MEGP 25 manual", "https://resources.mecademic.com/en/doc/MC-UM-MEGP25/latest/manual/installing-the-gripper.html")]),
                    sensor=dict(id="nano25", name="ATI Nano25 SI-125-3", ranges="Fxy 125 N, Fz 500 N, Txy 3 N m, Tz 3 N m; resolution 0.021 N (1/48 N) and 0.0004 N m; overload Fxy 2300 N, Txy 43 N m", mass="63 g, 25 mm dia x 21.6 mm", price="quote (about $6,000 to $8,000 with the Net F/T box, ESTIMATE)", electronics="ATI Net F/T interface box (Ethernet, 7 kHz)",
                                links=[("ATI Nano25", "https://ati.novanta.com/product/nano25-force-torque-sensor/")]),
                    machined=["Adapter plate, Meca500 flange to Nano25: 28 mm round x 4 mm 6061, 4 x M3 countersunk from below on 15.5 mm PCD, 9 h6 boss, 3 x M3 counterbored + 2 x 3 mm dowels on the sensor's 20.32 mm circle (FFC-custom-parts/meca_adapter_plate_nano25)",
                              "90 degree bracket, 30 x 6 mm 6061: Nano25 tool pattern (3 x M3 + 2 x 3 mm dowels on a 20.32 mm circle) counterbored under the leg, MEGP STAP01 plate pattern on the vertical leg (FFC-custom-parts/meca_bracket)",
                              "Shoe, 0.3 mm (or 0.5 mm) shim steel 20 x 7 mm with the 1.5 mm lip, laser or wire cut, bonded and screwed to the lower finger (FFC-custom-parts/shoe)",
                              "D405 arm: 6 x 10 mm 6061 L with a 3 mm back plate (FFC-custom-parts/meca_arm_d405)"],
                    printed=["Lower finger with the east leg, PAHT-CF (FFC-custom-parts/finger_lower_megp25e)", "Upper finger with the pad arm, PAHT-CF (FFC-custom-parts/finger_upper_megp25e); 1.5 mm silicone pad face bonded",
                             "Two camera tilt blocks for the bench posts (25 deg)", "Pallet corner stops, platform rest and feed clip (case 2)"]),
    "lrmate200id": dict(title="Setup B: FANUC LR Mate 200iD", review="lrmate200id/lehz16_mini40",
                        arm=dict(name="LR Mate 200iD (standard, 7 kg, 717 mm) with the R-30iB Mate Plus, iPendant Touch, S636 External Control Package (Stream Motion J519 + Remote Motion R912), R632 KAREL, R648 User Socket Messaging", payload="7 kg", reach="717 mm", repeat="plus or minus 0.01 mm (ISO 9283)", mass="25 kg", flange="J4/J5 16.6 N m, J6 9.4 N m; ISO 9409-1-31.5 flange",
                                 price="quote only; guides $17,500 to $25,000 new with controller; used $14,900 to $17,000 (SCC Robotics, Robots Done Right) plus about $2,700 of options, $1,000 install and an UNVERIFIED core software upgrade to V9.40P",
                                 links=[("FANUC America LR Mate 200iD", "https://www.fanucamerica.com/products/robots/series/lr-mate/lr-mate-200id"), ("FANUC America Union City (Bay Area office), (888) 326-8287", "https://fanucamerica.com/about-us/locations/northwest-usa-san-francisco-bay-ca"), ("Used: SCC Robotics", "https://sccrobotics.com/product/fanuc-lr-mate-200id-w-r-30ib-plus-control-4/"), ("Used: Robots Done Right", "https://robotsdoneright.com/FANUC/LR-Mate-Series/FANUC-LR-Mate-200id.html"), ("fanuc_driver (ROS 2) requirements: S636, V9.40P/81", "https://fanuc-corporation.github.io/fanuc_driver_doc/main/docs/environment/system_requirements.html")],
                                 contact="FANUC America, 29700 Kohoutek Way, Union City CA 94587. A used unit must show its software version before payment."),
                        gripper=dict(id="smc_lehz16l", stroke="3 mm per jaw (we use 1.5 mm)", force="3 to 8 N settable (pushing force 40 to 100 percent in the step data), open loop plus or minus 30 percent of full scale: bench-calibrate on a scale", repeat="plus or minus 0.02 mm", mass="190 g", fingers="50 g per finger allowed (ours 3 g and 6 g)", control="JXC51/JXC61 parallel IO (24 V, NPN/PNP to the R-30iB DO) or JXC91 EtherNet/IP",
                                     price="about $800 for the gripper, $257 to $515 for the controller (ESTIMATE)", links=[("SMC LEH series catalogue (PDF)", "https://www.smcworld.com/catalog/Electric-en/mpv/Electric-LEH_en/data/Electric-LEH_en.pdf"), ("SMC USA", "https://www.smcusa.com/")]),
                        sensor=dict(id="mini40", name="ATI Mini40 SI-20-1", ranges="Fxy 20 N, Fz 60 N, Txy 1 N m; resolution 1/200 N (5 mN); overload 2400 N", mass="50 g", price="quote (about $6,000 to $8,000 with the interface box, ESTIMATE)", electronics="ATI Net F/T interface box", links=[("ATI Mini40", "https://ati.novanta.com/product/mini40-force-torque-sensor/")]),
                        machined=["Adapter plate, ISO 31.5 flange to Mini40 33 mm PCD: 44 dia x 6 mm 6061 (FFC-custom-parts/lrmate_adapter_plate)",
                                  "90 degree bracket, 30 x 6 mm 6061, LEHZ16 side pattern 2 x M3 + 2 x 2.5 H9 pins (position ESTIMATE until the SMC drawing is in hand) (FFC-custom-parts/lrmate_bracket)",
                                  "Upper finger leg in 6061 (not printed): the 46 mm reach would deflect 0.21 mm printed, 0.02 mm in aluminium",
                                  "Shoe, shim steel with the 1.5 mm lip; D405 arm, 6 x 10 mm 6061 L"],
                        printed=["Lower finger plate (PAHT-CF) on the 14 x 8 mm jaw pattern", "Pad arm (PAHT-CF) with the 1.5 mm silicone face", "Two camera tilt blocks", "Pallet corner stops, platform rest and feed clip (case 2)"]),
}
COMMON = dict(
    cameras=[("Basler dart daA1920-160um, S-mount, mono, global shutter, 1920 x 1200, 160 fps, 15 g", "2", "EUR 259 list; the product page says not in series production yet: confirm stock, fallback daA1920-30um or ace 2 a2A1920-160umBAS C-mount", "https://www.baslerweb.com/en/shop/daa1920-160um-s-mount/"),
             ("Edmund Optics #37-182 TECHSPEC 16 mm f/4 Blue Series M12 lens (optimised 150 to 250 mm)", "2", "$115, 20+ in stock (read in Chrome today)", "https://www.edmundoptics.com/p/16mm-fl-f40-blue-series-m12-lens/46302/"),
             ("Intel RealSense D405 (wrist, RGB + depth 848 x 480 at 30 fps, 58 g)", "1", "about $384 at DigiKey, 500+ in stock (read today)", "https://www.digikey.com/en/products/detail/intel-realsense/82635DSD405/16056868"),
             ("CCS LDL2-33X8BL(2) blue 470 nm bar light, 44 x 10 x 13 mm, with a 24 V driver", "1", "quote (Graftek)", "https://graftek.com/product/ccs-america-ldl2-33x8bl2-blue-470-nm-led-bar-light-44-x-10-4-x-13-l-x-w-x-h-mm-fcb/"),
             ("Thorlabs 12.7 mm posts, post holders and bases for two cameras and the light (TR75/PH75/BA1 class)", "3 sets", "about $35 per stand", "https://www.thorlabs.com/navigation.cfm?guide_id=47"),
             ("USB 3 high-flex cable, 2 m, along the arm for the D405 (Helukabel 805287 class, 62 g/m)", "1", "about $40", "https://www.helukabel.com/"),
             ("USB 3 cables for the two bench cameras (standard, 3 m)", "2", "about $15 each", "")],
    leader=[("3Dconnexion SpaceMouse Compact: 6-DoF Cartesian input, what HIL-SERL and Fraunhofer used to collect insertion demonstrations; scale the motion 5:1 for the last millimetres", "$179 to $204", "https://3dconnexion.com/us/product/spacemouse-compact/"),
            ("GELLO-style printed twin of the Meca500 (6 x Dynamixel XL330-M288-T + U2D2), joint mirroring at 0.5 to 0.75 scale; no community design exists, a few days of CAD", "$200 to $300 in parts", "https://github.com/wuphilipp/gello_mechanical"),
            ("The YAM leader already owned, through IK retargeting at 0.3 to 0.5 scale (Isaac Teleop or gello_software)", "$0", "https://github.com/i2rt-robotics/i2rt"),
            ("Force Dimension omega.7 (haptic, 12 N, 4 kHz): the only route that feeds the latch press back to the hand", "$28,175, not worth it for one dev cell", "https://www.roboticscenter.ai/store/product/force-dimension-omega-7/")],
    videos=[("Flexiv Rizon 4s FPC insertion (AI vision + 1 kHz wrist force, visual servo)", "https://www.youtube.com/watch?v=p6niEdAp-78"),
            ("Hirose FH62/FH63 FPC insertion and extraction by a Kawasaki duAro (one-action lock connector)", "https://www.youtube.com/watch?v=DICTwdO7ZeI"),
            ("Epson: force sensor + robot recognises, grips and inserts cables into connectors", "https://www.youtube.com/watch?v=lyeY9QxfxiU"),
            ("Kyocera one-action-lock FPC/FFC connector robot demos", "https://ele.kyocera.com/en/technical/con_robot/"),
            ("IRISO robot assembly demos (FANUC CRX, Mitsubishi RV-FR, UR, YuMi)", "https://www.irisoele.com/en/technology/solution/automation-connector/demonstration/"),
            ("Meca500 video: how SDC integrated the Meca500 in a medical device assembly cell (YouTube)", "https://www.youtube.com/watch?v=3jDsE8ihlok"),
            ("Meca500 video: three Meca500s on a motherboard electronics assembly cell (YouTube)", "https://www.youtube.com/watch?v=gAkaEtr6i5I"),
            ("Meca500 video: two Meca500s with Solomon 3D vision assembling minute parts (YouTube)", "https://www.youtube.com/watch?v=Axnl3V2hGJM"),
            ("LR Mate video: LR Mate 200iD 4S intelligent assembling system (YouTube)", "https://www.youtube.com/watch?v=TIpo2TMF1LU"),
            ("Meca500 proof: SDC microscopic assembly, 1 to 2 um held in production (case study with video)", "https://mecademic.com/insights/case-studies/sdc-uses-mecademics-six-axis-robot-to-automate-a-microscopic-assembly-process-with-99-efficiency/"),
            ("Meca500 proof: 0.406 mm wire into a 0.431 mm needle, 25 um clearance, live at Automate", "https://mecademic.com/blog/redefining-medical-micro-automation-live-from-the-automate-show/"),
            ("Meca500 proof: XVP Photonics optical alignment (lens onto CMOS sensor)", "https://mecademic.com/insights/case-studies/how-xvp-photonics-automates-optical-alignment-assembly-with-meca500/"),
            ("LR Mate proof: FANUC force sensor page, LR Mate 200iD/7L assembling waterproof connectors under force and vision", "https://www.fanucamerica.com/products/accessories/force-sensor")],
)

# ----------------------------------------------------------------------------------------------------------------- numbers from the twin
def matrix_rows():
    rows = []
    for r in M:
        v = r.get("verify"); v = eval(v) if isinstance(v, str) and v.startswith("{") else (v or {})
        rows.append((r["variant"], v.get("depth_mm"), v.get("lateral_mm"), v.get("latch_closed"), v.get("latch_deg"), r.get("clearance_rc")))
    return rows

def review_rows(key, groups):
    d = R[key]
    return d["mass_g"], d["com_mm"], [x for x in d["rows"] if x.get("group") in groups]

def table(head, rows):
    return "<table><tr>" + "".join(f"<th>{e(h)}</th>" for h in head) + "</tr>" + "".join("<tr>" + "".join(f"<td>{c}</td>" for c in r) + "</tr>" for r in rows) + "</table>"

def link(t, u):
    return f'<a href="{e(u)}" target="_blank" rel="noopener">{e(t)}</a>' if u else e(t)

def status(s):
    return f'<span class="st {s.lower()}">{e(s)}</span>'

css = """body{margin:0;background:#FDF9F2;color:#1F1B16;font:15px/1.5 Inter,-apple-system,Helvetica,sans-serif}.wrap{max-width:1180px;margin:0 auto;padding:36px 24px 80px}
h1{font:600 30px/1.2 'Plus Jakarta Sans',Inter,sans-serif;margin:0 0 6px}h2{font:600 21px/1.3 'Plus Jakarta Sans',Inter,sans-serif;margin:40px 0 10px;padding-top:12px}h3{font:600 16px/1.3 Inter,sans-serif;margin:22px 0 6px}
p{margin:6px 0 10px}table{border-collapse:collapse;width:100%;margin:8px 0 14px;font-size:13.5px}th{text-align:left;background:#F3EBDD;padding:7px 9px;font-weight:600}td{padding:6px 9px;border-bottom:1px solid #EFE6D6;vertical-align:top}
.sheet{background:#F6EFE3;border-radius:10px;padding:14px 18px;margin:10px 0}.eyebrow{font-size:11px;letter-spacing:.08em;text-transform:uppercase;color:#8A7A62;margin-bottom:4px}
.st{display:inline-block;padding:1px 7px;border-radius:4px;font-size:12px;font-weight:600}.ok{background:#DDF5EA;color:#0A7A52}.check{background:#FFF1D6;color:#9A5B00}.fail{background:#FBE1E1;color:#A61B1B}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:18px}@media(max-width:800px){.grid{grid-template-columns:1fr}}img.v{width:100%;border-radius:6px;display:block}.n{color:#6B6257;font-size:13px}
a{color:#2563EB;text-decoration:none}a:hover{text-decoration:underline}.big{font-size:26px;font-weight:600}ul{margin:4px 0 10px 20px}li{margin:3px 0}.pill{background:#E8F0FF;color:#1E40AF;border-radius:4px;padding:1px 6px;font-size:12px}"""

parts = []
parts.append(f"""<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>FFC cell, final setups</title><style>{css}</style></head><body><div class="wrap">
<div class="eyebrow">Synphony Co, FFC into flip-latch ZIF, 2026-10-13</div><h1>The two setups, finalised: what we order, why the numbers work, and what is still open</h1>
<p>Every figure below comes from the twin that passed the task (out/ffc_zif/review.json, matrix.json), the parts database, or the research notes of 2026-10-12. Prices marked ESTIMATE or quote are not confirmed. <b>Decision of today:</b> the two macro cameras and the bar light move off the wrist onto bench posts. Rendering each wrist camera's own view showed 96 to 97 percent of every macro frame was the stack's own bracket and arm, a policy needs a view the arm cannot shake, and the 16 mm lens is only specified from 150 mm.</p>""")

# ---- precision
mr = matrix_rows()
parts.append("<h2>1. Precision: the unit is millimetres (micrometres for the small terms), degrees for yaw</h2>")
parts.append('<div class="grid"><div class="sheet"><div class="eyebrow">What the task tolerates (connector geometry in the twin)</div>' + table(["quantity", "window", "where it comes from"], [
    ["lateral (across the slot)", "<b>plus or minus 0.3 mm</b>; pass criterion plus or minus 0.2 mm", "28.0 mm cable in a 28.6 mm slot"],
    ["yaw before the slide", "<b>under 0.6 deg</b>", "0.3 mm a side over the 28 mm width"],
    ["height of the tip at the mouth", "<b>plus or minus 0.05 mm</b>", "0.3 mm cable in a 0.4 mm mouth; the shoe carries the tip at slot-floor height, so this is a fixture number, not an arm number"],
    ["insertion depth", "3.5 mm, <b>plus or minus 0.4 mm</b>", "mouth to rear stop (FH12 class)"],
    ["latch", "closed, under 1 deg", "lid flat on the cable"]]) + "</div>")
parts.append('<div class="sheet"><div class="eyebrow">What our setups deliver (error budget)</div>' + table(["term", "Meca500", "LR Mate 200iD"], [
    ["arm repeatability", "0.005 mm", "0.01 to 0.02 mm"],
    ["camera, 31 um per pixel at 160 mm", "5 to 10 um on a sub-pixel edge fit", "same (same cameras)"],
    ["hand-eye calibration (ChArUco at the working distance)", "20 to 40 um", "20 to 40 um"],
    ["cable width tolerance", "plus or minus 0.05 mm", "plus or minus 0.05 mm"],
    ["gripper jaw repeatability (does not enter: the cable is located by camera after the grasp)", "0.03 mm", "0.02 mm"],
    ["<b>sum against the 0.2 mm criterion</b>", "<b>about 0.06 to 0.1 mm, 2 to 3x margin</b>", "<b>about 0.08 to 0.11 mm, about 2x margin</b>"]]) + "</div></div>")
parts.append("<h3>Twin results, every variant, this run</h3>" + table(["variant", "depth mm (3.5 target)", "lateral mm", "latch closed", "lid deg", "clearance audit"], [[e(v), f"{d}", f"{l}", "yes" if lc else "no", f"{ld}", "clean" if rc in (0, "0") else "see log"] for v, d, l, lc, ld, rc in mr]) +
             '<p class="n">Depth varies between 3.3 and 3.5 mm from run to run because the 0.2 mm creep after release depends on the contact solver; all within the 0.4 mm window. Lateral error is at the solver floor.</p>')

# ---- force
parts.append("<h2>2. Force: you need about 0.1 N of resolution and plus or minus 0.5 N of accuracy over 0 to 20 N</h2>")
parts.append(table(["event", "force", "what the sensor must do"], [
    ["sliding friction during the 3.5 mm slide", "about 0.5 N", "baseline"],
    ["seat (tip on the rear stop)", "rise of 1 to 3 N", "detect the rise: 0.1 N resolution, under 10 ms latency at 2 mm/s"],
    ["slide guard", "4 N", "stop within 0.15 N of overshoot (2 mm/s x 5 ms x 15 N/mm)"],
    ["lid lift", "2 N on the lip", "guard"],
    ["latch push", "7 N at the lid tip, 12 N guard", "guard"],
    ["press", "10 N, 20 N worst case", "guard; the connector's rating is the ceiling"],
    ["pinch", "5 N", "set by the gripper: MEGP in 0.4 N steps, LEHZ16 plus or minus 2.4 N open loop (calibrate)"],
    ["gravity bias of the 330 to 830 g tool when tilted 20 deg", "up to 4 N", "must be compensated from the payload model"],
    ["<b>torque seen by the sensor</b>: a horizontal force at the tip acts about 72 mm (Meca500) or 78 mm (LR Mate) below the sensor", "4 N slide = 0.28 / 0.31 N m; 7 N push = 0.50 / 0.55 N m; 12 N push guard = 0.86 / 0.94 N m", "Nano25 SI-125-3 range 1.5 N m: 57% at the guard. Mini40 SI-40-2 range 2 N m: 47%"]]))
parts.append(table(["sensor", "range", "resolution", "margin on the 0.1 N need", "torque range vs the 12 N push guard at the tip"], [
    ["ATI Nano25 SI-125-3 (Meca500)", "Fxy 125 N, Fz 500 N, Txy 3 N m", "21 mN force (1/48 N), 0.76 mN m torque (1/1320 N m)", "10x", "0.78 of 3 N m: 52%. 63 g, 25 dia x 21.6 mm, 3 x M3 on a 20.32 mm circle (drawing 9230-05-1312)"],
    ["ATI Mini40 SI-40-2 (LR Mate)", "Fxy 40 N, Fz 120 N, Txy 2 N m", "10 mN, 0.3 mN m", "10x", "0.94 of 2 N m: 47%"]]))
parts.append("<p>Resolution is never the limit: at a 65 mm lever 0.1 mN m resolves 1.5 mN at the lip, and the 2 N lid lift is over 700 counts. Range is: any horizontal tip force sits on the whole stack length, so the sensor should be chosen so the 12 N push guard stays under about 60% of its torque range. Decision pending: Nano25 SI-125-3 and Mini40 SI-40-2, or keep the Nano17 with a 7.5 N push cap.</p>")
parts.append("<p>So both sensors are comfortably enough. The lid is driven by the arm, not by a second motor: the gripper only pinches, the shoe's steel lip hooks and arcs the lid open, the shoe's rear edge pushes it shut and presses, every move force-guarded. A separate lid actuator would add mass and an axis to calibrate; the Zaber variant that tried it is over the Meca500 payload (583 g).</p>")

# ---- setups
for key, S in SETUPS.items():
    m_g, com, rows = review_rows(S["review"], ("mass",))
    A, G, Sn = S["arm"], S["gripper"], S["sensor"]
    parts.append(f"<h2>{e(S['title'])}</h2><div class='grid'>")
    parts.append("<div class='sheet'><div class='eyebrow'>Arm</div><p><b>" + e(A["name"]) + "</b></p>" + table(["", ""], [["payload", A["payload"]], ["reach", A["reach"]], ["repeatability", A["repeat"]], ["mass", A["mass"]], ["wrist limits", A["flange"]], ["price", A["price"]]]) + "<ul>" + "".join(f"<li>{link(t, u)}</li>" for t, u in A["links"]) + f"</ul><p class='n'>{e(A['contact'])}</p></div>")
    gp = P.get(G["id"], {})
    parts.append("<div class='sheet'><div class='eyebrow'>Gripper</div><p><b>" + e(gp.get("name", G["id"])) + "</b></p>" + table(["", ""], [["stroke", G["stroke"]], ["force", G["force"]], ["repeatability", G["repeat"]], ["mass", G["mass"]], ["fingers", G["fingers"]], ["control", G["control"]], ["price", G["price"]]]) + "<ul>" + "".join(f"<li>{link(t, u)}</li>" for t, u in G["links"]) + "</ul>"
                 + "<div class='eyebrow'>Force sensor</div><p><b>" + e(Sn["name"]) + "</b>: " + e(Sn["ranges"]) + f". {e(Sn['mass'])}. {e(Sn['electronics'])}. {e(Sn['price'])}. " + " ".join(link(t, u) for t, u in Sn["links"]) + "</p></div></div>")
    parts.append(f"<h3>Mass and payload</h3><p><span class='big'>{m_g} g</span> below the flange, centre of mass {com[2]:.0f} mm below the flange and {(com[0]**2+com[1]**2)**0.5:.0f} mm off axis.</p>" +
                 table(["check", "value", "limit", ""], [[e(x["item"]), e(str(x["value"])), e(str(x["limit"])), status(x["status"])] for x in rows]))
    _, _, srows = review_rows(S["review"], ("stiffness", "sensor", "gripper", "finger"))
    if srows:
        parts.append("<h3>Loads, stiffness, finger and sensor limits</h3>" + table(["check", "value", "limit", ""], [[e(x["item"]), e(str(x["value"])), e(str(x["limit"])), status(x["status"])] for x in srows]))
    parts.append("<h3>We machine</h3><ul>" + "".join(f"<li>{e(t)}</li>" for t in S["machined"]) + "</ul><h3>We print</h3><ul>" + "".join(f"<li>{e(t)}</li>" for t in S["printed"]) + "</ul>")

# ---- variants tried
parts.append("""<h2>3b. Variants tried and set aside</h2>
<p><b>Zaber X-NA08A25 pusher (Setup A variant).</b> A 25.4 mm travel stepper-screw micro actuator with its controller built in: 0.0476 um microstep, under 1 um repeatability, 40 um accuracy, 25 N thrust, 140 g, 137 mm long (<a href="https://www.zaber.com/products/linear-actuators/X-NA?part=X-NA08A25" target="_blank" rel="noopener">Zaber page</a>). It was bolted between the Nano17 and the gripper bracket along the insertion axis so that the 3.5 mm slide would be done by the actuator on a perfectly straight line while the arm only parks the tool at the mouth (the macro-micro idea from the fine-stage research). Pros: a sub-micrometre, dead-straight slide independent of the arm's path, so the arm's repeatability stops mattering for the slide. Cons: 140 g and a 123 mm lever on a 0.5 kg arm (583 g before the cameras came off the wrist, 458 g now, centre of mass 47 mm below the flange); open loop with no encoder, so a stall is silent; a second controller, cable and calibration; its long body blocks the bench cameras in 14 of 36 views; and the twin shows the arm alone already lands the tip within 0.001 mm laterally and inside the 0.4 mm depth window, so the stage adds nothing measurable. It stays modelled and audited but is not ordered. If the real arm's slide ever proves not straight enough, this is the ready answer.</p>
<p><b>Thin shoe (0.3 mm) variant.</b> Same stack with the thinner shim; passes identically; the 0.5 mm shoe is stiffer and is the default.</p>
<p><b>MEGP 25LS, Schunk EGK 25, Zimmer GEP2006, SMC LEHZ10.</b> Rejected earlier: the 25LS jaw rail reaches below the board, the EGK is 620 g, the Zimmer's minimum force is 40 N, the LEHZ10 tops out at 6 N.</p>""")
# ---- servo tool v1 (our own end effector), compared with the bought stacks on the same runs
def _row(k, item):
    d = R.get(k, {}); return next((x for x in d.get("rows", []) if item in x["item"]), None)
sv = {"meca": R.get("meca500/servo_v1"), "lr": R.get("lrmate200id/servo_v1_lr")}
if sv["meca"] and sv["lr"]:
    mA, mB = R["meca500/megp_nano25"], R["lrmate200id/lehz16_mini40"]
    def mr(variant):
        for r_ in M:
            if r_["variant"] == variant:
                v = r_.get("verify"); v = eval(v) if isinstance(v, str) and v.startswith("{") else (v or {}); return v
        return {}
    parts.append("""<h2>3c. Our own end effector: servo tool v1, designed and run through the same twin</h2>
<p>Neil's question was whether an in-house servo tool could be more precise, cheaper and more accurate than the bought gripper plus six-axis sensor. The answer from the research (docs/research/2026-10/research_notes/custom_servo_end_effector.md, servo_tool_v1_design_numbers.md, servo_tool_v1_parts.md, custom_end_effector_requirements.md) and from the twin: <b>not more precise</b>, because the insertion precision is set by the arm and the two bench cameras, not by the jaw drive; <b>cheaper by about 25 times</b> and <b>lighter</b>; <b>accurate enough</b> on force, with the pinch closed on its own load cell. Full design explainer with drawings: <a href='FFC-servo-tool.html'>FFC-servo-tool.html</a>.</p>
<div class="sheet"><div class="eyebrow">The design</div>
<p>One Dynamixel XL330-M077-T (5 V, 0.215 N m stall, 383 rpm, 18 g, 4096 steps per turn) turns an M4 x 1 mm lead screw (McMaster 90500A160 with a PEEK anti-backlash nut) whose nut carries the upper jaw; a printed rocker couples the lower jaw so <b>both move 1.5 mm</b> on a Misumi SSEB6 miniature rail, the same motion as the bought grippers (a fixed lower jaw lost the pinch in the twin during the lift off the cable rest). 6.4 mm/s, 0.24 um per encoder step, self-locking. <b>Three TAL220B 5 kg bar load cells</b> on two ADS122C04 24-bit boards at 1 kHz: two in series between the standoff and the body (bar A along the bracket reads the insertion axis: slide, seat rise, lid push; bar B beside the body reads the vertical axis: lid lift, latch press) and one as the upper finger's cross member that reads the pinch directly (the wrist pair cannot see it, it is internal to them; without it the stiffener's 0.03 mm thickness tolerance would change an open-loop pinch by about 5 N). The shoe, lip, pad and fingers are the same parts that pass on the bought stacks; the D405 rides the same side plate. Printed PAHT-CF body 55 x 22 x 26 mm plus a 6 mm jaw guide, a printed standoff (22 mm Meca500, 34 mm LR Mate), one 4-core cable along the arm (24 V pair plus USB data).</p></div>""")
    rows = []
    for label, kb, ks, vb, vs in (("Meca500", "meca500/megp_nano25", "meca500/servo_v1", "meca500-megp_nano25-laptop", "meca500-servo_v1-laptop"), ("LR Mate 200iD", "lrmate200id/lehz16_mini40", "lrmate200id/servo_v1_lr", "lrmate200id-lehz16_mini40-laptop", "lrmate200id-servo_v1_lr-laptop")):
        b, sv_ = R[kb], R[ks]; vb_, vs_ = mr(vb), mr(vs)
        rows += [[f"{label}: mass below the flange", f"{b['mass_g']} g", f"{sv_['mass_g']} g"],
                 [f"{label}: centre of mass below the flange / off axis", f"{b['com_mm'][2]:.0f} / {(b['com_mm'][0]**2+b['com_mm'][1]**2)**0.5:.0f} mm", f"{sv_['com_mm'][2]:.0f} / {(sv_['com_mm'][0]**2+sv_['com_mm'][1]**2)**0.5:.0f} mm"],
                 [f"{label}: insertion depth / lateral (laptop run)", f"{vb_.get('depth_mm')} mm / {vb_.get('lateral_mm')} mm", f"{vs_.get('depth_mm')} mm / {vs_.get('lateral_mm')} mm"],
                 [f"{label}: latch closed", "yes" if vb_.get("latch_closed") else "no", "yes" if vs_.get("latch_closed") else "no"]]
    rows += [["force sensing", "six axes, 10 mN, 7 kHz (ATI Nano25 / Mini40 with Net F/T)", "three bar cells at 1 kHz: insertion axis and vertical axis at the wrist (every force at the tips), pinch at the pad; about 0.07 N peak to peak on the ADS122C04; no lateral axis, no torques"],
             ["grip force", "MEGP: 0.4 N steps, tolerance unpublished; LEHZ16: open loop plus or minus 2.4 N, calibrated on a scale", "closed loop on the pad cell, about 0.04 N per servo step through the silicone; target plus or minus 0.3 N at 5 N; jaw repeatability and hysteresis to be measured on the bench"],
             ["jaw repeatability (does not enter the insertion precision)", "0.03 mm (MEGP), 0.02 mm (LEHZ16), published", "no published figure for a Dynamixel on a lead screw: dial-gauge test first"],
             ["cables on the arm", "gripper, sensor and D405: three", "one 4-core tool cable (24 V pair + USB data) and the D405 USB: two"],
             ["parts cost per arm (sourced 2026-10-13, servo_tool_v1_parts.md)", "about $8,700 (Meca500: MEGP 25E $1,675 + Nano25 with Net F/T about $7,000); about $9,300 (LR Mate)", "about $340 in parts for the first tool (servo $27, three cells $47, two ADC boards $40, MCU $10, buck $24, screw, nut and rail about $50, cable about $40, filament and plate stock about $90 shared), plus the D405 already ordered"],
             ["lead time", "stock to 6 weeks (ATI)", "the XL330 is sold out at ROBOTIS America (about 2 months, order now); cells, ADC, MCU in stock; McMaster and Misumi parts to confirm in a browser; then two weeks of printing, machining and bench tests"],
             ["what it needs before it can replace a bought stack", "", "bench: jaw repeatability under 0.05 mm, pinch held within 0.3 N over an hour, cell noise under 0.1 N at 1 kHz, tip motion under 0.05 mm at 4 N through the cells, USB full speed over the 3 m cable; then the real-connector runs"]]
    parts.append(table(["", "bought stack", "servo tool v1"], [[e(a), e(b_), e(c_)] for a, b_, c_ in rows]))
    parts.append("<p class='n'>The servo tool's own review rows (loads, deflection, joint margins, camera views) are in the same review.json and camera-views index as the bought stacks, under meca500/servo_v1 and lrmate200id/servo_v1_lr. Parts list with sourced prices: docs/research/2026-10/research_notes/servo_tool_v1_parts.md.</p>")

# ---- cameras
parts.append("<h2>4. Cameras: on the bench, verified frame by frame</h2>")
parts.append(f"""<div class="grid"><div class="sheet"><div class="eyebrow">The set, one for both arms</div>
<p><b>2 x Basler dart daA1920-160um + Edmund #37-182 16 mm</b> on 12.7 mm posts, lens front <b>{CM['wd']*1e3:.0f} mm</b> from the mouth (the lens is optimised for 150 to 250 mm; at 108 mm on the wrist it was out of spec), elevation <b>{CM['el_deg']:.0f} deg</b>, azimuths <b>{CM['az_deg'][0]:.0f} and {CM['az_deg'][1]:.0f} deg</b> from east, on the cable side of the connector.
<b>31 um per pixel, 60 x 37 mm field, about 5 mm depth of field at f/4</b> (f/8 doubles it but is diffraction-limited in blue). Global shutter, up to 160 fps. The CCS blue bar light stands 60 mm west at 25 deg.</p>
<p><b>Wrist: Intel RealSense D405</b> on its L arm behind the flange, RGB plus depth 848 x 480 at 30 fps, looking at the jaw tips; its view is 47 percent tool, which is normal for a wrist camera.</p>
<p><b>Good enough for ACT?</b> Yes. ALOHA's recipe is two static and one or two wrist 480 x 640 RGB streams at 50 Hz; we have two still 1920 x 1200 views (downsample to 640 x 400) at any rate up to 160 fps plus the wrist D405. The fixed mounts are what makes the views usable for training: nothing on the arm can shake or re-aim them between sessions. Mono is fine, the scene is lit blue and the policy sees contrast, not colour.</p>
<p><b>Risk:</b> the Basler 160um page says not in series production yet (it loads in Chrome today). Fallbacks that change nothing else: daA1920-30um (30 fps), or the C-mount ace 2 a2A1920-160umBAS with a 25 mm lens at 200 mm, since mass no longer matters on a post.</p></div>""")
# sweep table
if sweep:
    tag0 = "meca500-megp_nano25-laptop" if "meca500-megp_nano25-laptop" in sweep else list(sweep)[0]
    cells = sweep[tag0]["cells"]; els = sorted({c["el"] for c in cells}); azs = sorted({c["az"] for c in cells})
    rows = []
    for az in azs:
        rows.append([f"{az}"] + [f"{next(c['visible'] for c in cells if c['az']==az and c['el']==el)*100:.0f}%" for el in els])
    parts.append(f"<div class='sheet'><div class='eyebrow'>Placement sweep ({e(tag0)}): share of frames where the tip, mouth and lid edge are visible</div>" + table(["azimuth deg"] + [f"elevation {el} deg" for el in els], rows) +
                 "<p class='n'>Azimuth from east toward north; the cable comes from the south (-90). The east side (0 to 60) is blocked by the gripper body and bracket, the north by the D405 arm; the south quadrant at 25 deg elevation keeps about 90 percent, and what remains hidden is the finger, shoe and pad legitimately in front of the tip while it carries the cable. Same picture on every variant and both arms.</p></div></div>")
# views
if views:
    parts.append("<h3>What the cameras see at the mouth (rendered from their real poses)</h3><div class='grid'>")
    for vv in views:
        if vv.get("variant") in ("meca500-megp_nano25-laptop", "lrmate200id-lehz16_mini40-laptop"):
            for x in vv["rows"]:
                if x["phase"].startswith("phase 2: tip") :
                    hidden = [pn for pn, p in x["points"].items() if p["in_view"] and not p["visible"]]
                    parts.append(f"<div><img class='v' src='FFC-camera-views/{e(x['png'])}'><div class='n'>{e(vv['variant'])}, {e(x['camera'])}: hardware {x['hardware_frac']*100:.0f}% of the frame, fingers {x['fingers_frac']*100:.0f}%, connector and cable {x['target_frac']*100:.0f}%" + (f"; hidden: {', '.join(hidden)}" if hidden else "; all three task points visible") + "</div></div>")
    parts.append("</div><p>Every view along the run, all six variants: <a href='FFC-camera-views/index.html'>FFC-camera-views/index.html</a>.</p>")
_, _, vrows = review_rows("meca500/megp_nano25", ("vision",))
parts.append(table(["vision check", "value", "limit", ""], [[e(x["item"]), e(str(x["value"])), e(str(x["limit"])), status(x["status"])] for x in vrows]))

# ---- power and signals
def wiring_svg(title, boxes, wires, w=1080, h=330):
    out = [f'<svg viewBox="0 0 {w} {h}" width="100%" style="max-width:{w}px;font:12px Inter,sans-serif">', f'<text x="8" y="18" font-size="13" font-weight="600">{e(title)}</text>']
    for k, (x, y, bw, bh, label, fill) in boxes.items():
        out.append(f'<rect x="{x}" y="{y}" width="{bw}" height="{bh}" rx="8" fill="{fill}" stroke="#C9BBA3"/>')
        for i, ln in enumerate(label.split("|")):
            out.append(f'<text x="{x + bw/2}" y="{y + 18 + i*15}" text-anchor="middle" font-weight="{600 if i == 0 else 400}">{e(ln)}</text>')
    for a, b, label, color in wires:
        ax, ay, aw, ah = boxes[a][:4]; bx, by, bw2, bh2 = boxes[b][:4]
        x1, y1 = ax + aw/2, ay + ah; x2, y2 = bx + bw2/2, by
        if abs(ay - by) < 5: y1 = ay + ah/2; y2 = by + bh2/2; x1 = ax + aw if bx > ax else ax; x2 = bx if bx > ax else bx + bw2
        out.append(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}" stroke-width="2.5"/>')
        out.append(f'<text x="{(x1+x2)/2}" y="{(y1+y2)/2 - 6}" text-anchor="middle" fill="{color}" font-size="11">{e(label)}</text>')
    out.append('</svg>'); return "".join(out)
ALU_, MAINS, PCB = "#F3EBDD", "#FBE1E1", "#DDF5EA"
A_boxes = {"mains": (20, 40, 150, 48, "Mains 120 V|bench outlet", MAINS), "ps200": (20, 130, 150, 56, "PS200-R4|90 to 250 V in, 24 V 5.5 A out", ALU_), "arm": (230, 130, 190, 56, "Meca500 R4|controller in the base", ALU_),
           "port": (230, 230, 190, 48, "wrist tool port (M8)|24 V + comms", ALU_), "megp": (230, 300, 190, 26, "MEGP 25E gripper", PCB),
           "psu24": (470, 40, 160, 48, "24 V 5 A supply|bench", MAINS), "netft": (470, 130, 160, 56, "ATI Net F/T box|Nano25 cable in, Ethernet out", ALU_), "nano25": (470, 230, 160, 48, "Nano25 sensor|on the flange", PCB),
           "ccs": (680, 130, 150, 56, "CCS light controller|24 V in", ALU_), "light": (680, 230, 150, 48, "blue bar light|bench post", PCB),
           "pc": (870, 40, 190, 56, "Linux PC|Ethernet switch + USB 3", MAINS), "d405": (870, 130, 190, 48, "D405 (wrist)|USB 3 along the arm", PCB), "cams": (870, 230, 190, 48, "2 x Basler dart|USB 3 on the bench", PCB)}
A_wires = [("mains", "ps200", "IEC cord", "#B91C1C"), ("ps200", "arm", "24 V + comm, 2 m M12", "#B91C1C"), ("arm", "port", "inside the arm", "#6B6257"), ("port", "megp", "MGC cable, 24 V 0.14 A + control", "#0A7A52"),
           ("psu24", "netft", "24 V", "#B91C1C"), ("netft", "nano25", "transducer cable 2 m, along the arm", "#0A7A52"), ("psu24", "ccs", "24 V", "#B91C1C"), ("ccs", "light", "light cable", "#0A7A52"),
           ("pc", "d405", "USB 3 (one cable on the arm)", "#1E40AF"), ("pc", "cams", "USB 3 x 2", "#1E40AF")]
B_boxes = {"mains": (20, 40, 150, 48, "Mains 240 V 20 A|NEMA 6-20, 1.2 kVA", MAINS), "ctrl": (20, 130, 150, 56, "R-30iB Mate Plus|cabinet", ALU_), "arm": (230, 130, 190, 56, "LR Mate 200iD|robot cable 7 m", ALU_),
           "pendant": (230, 230, 190, 48, "iPendant Touch|10 m to the cabinet", ALU_),
           "psu24": (470, 40, 160, 48, "24 V 5 A supply|bench", MAINS), "jxc": (470, 130, 160, 56, "SMC JXC91|gripper controller, bench", ALU_), "lehz": (470, 230, 160, 48, "LEHZ16 gripper|motor + encoder cable 3 m", PCB),
           "netft": (680, 130, 150, 56, "ATI Net F/T box|Mini40 cable in", ALU_), "mini40": (680, 230, 150, 48, "Mini40 sensor|on the flange", PCB),
           "pc": (870, 40, 190, 56, "Linux PC|Ethernet switch + USB 3", MAINS), "d405": (870, 130, 190, 48, "D405 (wrist)|USB 3 along the arm", PCB), "cams": (870, 230, 190, 48, "2 x Basler dart + CCS light|bench", PCB)}
B_wires = [("mains", "ctrl", "hard-wired", "#B91C1C"), ("ctrl", "arm", "robot connection cable", "#B91C1C"), ("ctrl", "pendant", "pendant cable", "#6B6257"),
           ("psu24", "jxc", "24 V, about 0.6 A peak", "#B91C1C"), ("jxc", "lehz", "actuator cable along the arm", "#0A7A52"), ("psu24", "netft", "24 V", "#B91C1C"), ("netft", "mini40", "transducer cable 2 m, along the arm", "#0A7A52"),
           ("pc", "d405", "USB 3 (one cable on the arm)", "#1E40AF"), ("pc", "cams", "USB 3 x 2, 24 V light via its controller", "#1E40AF")]
parts.append("<h2>5a. Power and signals: how each gripper and sensor is wired</h2>")
parts.append("<div class='sheet'><div class='eyebrow'>Setup A, Meca500</div>" + wiring_svg("Red: power. Green: along-the-arm cables. Blue: USB. The PC talks to the arm, the Net F/T box and (via the arm) the gripper over one Ethernet switch.", A_boxes, A_wires) +
             "<p>The MEGP 25E is the only gripper the Meca500's wrist port accepts: it gets 24 V (0.14 A) and its control signals through the MGC cable from the M8 port on the wrist, and the Meca500's own firmware drives it (GripperOpen, GripperClose, SetGripperForce 5 to 100%). Nothing else uses that port. The Nano25 is electrically separate: its 2 m transducer cable runs down the arm to the ATI Net F/T box on the bench, which takes 24 V and streams force data at up to 7 kHz over Ethernet to the PC. The D405's single USB 3 cable is the only other cable on the arm. So three cables ride the arm: the MGC gripper cable (inside the wrist), the Nano25 cable and the D405 USB; the two bench cameras and the light never move.</p></div>")
parts.append("<div class='sheet'><div class='eyebrow'>Setup B, LR Mate 200iD</div>" + wiring_svg("Red: power. Green: along-the-arm cables. Blue: USB and Ethernet. Nothing is wired into the FANUC controller except its own arm and pendant.", B_boxes, B_wires) +
             "<p>The LEHZ16 is a stepper-motor gripper with an encoder; its single actuator cable (the robotic-grade 3 m version) runs down the arm to the SMC JXC91 controller sitting on the bench next to the robot. The JXC91 takes 24 V from a bench supply (14 W peak) and executes stored step data (open, close at 70% force, speed) when told to over EtherNet/IP, which our Linux PC sends over the same switch the robot's Stream Motion runs on. The FANUC controller does not see the gripper at all; its end-effector I/O connector on the forearm stays unused. The Mini40 is wired exactly like the Nano25: 2 m cable down the arm to a Net F/T box on the bench, 24 V in, Ethernet out. Cables on the arm: the LEHZ16 actuator cable, the Mini40 cable and the D405 USB.</p></div>")
parts.append("<p class='n'>One 24 V 5 A bench supply covers the JXC91, both Net F/T boxes and the CCS light controller with margin. The Meca500's PS200 and the FANUC cabinet have their own mains feeds. Dress the three arm cables together in one sleeve with a strain relief at the wrist and a service loop at the base, like the Mecademic and SMC manuals ask.</p>")

# ---- leader
parts.append("<h2>5. Leader for teleop and demonstrations</h2>" + table(["option", "cost", "link"], [[e(t), e(c), link("link", u)] for t, c, u in COMMON["leader"]]) +
             "<p>Order the SpaceMouse now: it is what the two published insertion-demo pipelines used, it needs no mechanical design, and motion scaling handles the last millimetres. Build the GELLO twin only if joint-space demos turn out to train better. No source has measured demonstration precision at 0.2 mm through any of these; the follower's force loop finishes the last part of every move regardless.</p>")

# ---- order list
parts.append("<h2>6. Order list</h2><h3>Per setup</h3>" + table(["item", "Setup A (Meca500)", "Setup B (LR Mate)"], [
    ["arm + controller + power", link("Meca500 R4 9100-002, Electromate or Mecademic, quote", "https://www.electromate.com/meca500/"), link("LR Mate 200iD + R-30iB Mate Plus + S636 + R632 + R648, FANUC Union City, quote", "https://www.fanucamerica.com/products/robots/series/lr-mate/lr-mate-200id")],
    ["gripper", link("MEGP 25E + MEGP25-TAP plate + 2 x MGC-HFC cable, $1,675 + cables", "https://www.electromate.com/megp-25e-electric-parallel-gripper/"), link("SMC LEHZ16LK2-6 + JXC51/61 or JXC91, about $800 + $257 to $515", "https://www.smcworld.com/catalog/Electric-en/mpv/Electric-LEH_en/data/Electric-LEH_en.pdf")],
    ["force sensor + box", link("ATI Nano25 SI-125-3 + Net F/T, quote", "https://ati.novanta.com/product/nano25-force-torque-sensor/"), link("ATI Mini40 SI-40-2 + Net F/T, quote", "https://ati.novanta.com/product/mini40-force-torque-sensor/")],
    ["machined (6061, shim steel)", "adapter plate, bracket, shoe, D405 arm", "adapter plate, bracket, upper finger leg, shoe, D405 arm"],
    ["printed (PAHT-CF)", "2 fingers, tilt blocks, stops, rest, clip", "lower finger plate, pad arm, tilt blocks, stops, rest, clip"],
    ["base", link("MUAP01 base adapter plate, quote", "https://www.electromate.com/meca500/"), link("floor base plate F-RR-LRMate-BP-N, $1,508.75 (Ballard)", "https://ballardintl.com/product/fanuc-lr-mate-200id-r30ib-mate-e75146")],
    ["power", "IEC C13 cord, surge strip, DB15 safety cable", "240 V 20 A circuit, NEMA 6-20, 1.2 kVA"]]))
parts.append("<h3>Common to both (cameras, light, stands, cables, leader)</h3>" + table(["item", "qty", "price and stock", "link"], [[e(t), e(q), e(pr), link("buy", u)] for t, q, pr, u in COMMON["cameras"]] + [[e(t), "1", e(c), link("buy", u)] for t, c, u in COMMON["leader"][:1]]))
parts.append("<p class='n'>STEP files and drawings for everything we make: <a href='FFC-custom-parts/index.html'>FFC-custom-parts</a>. Full stack blueprint with hover cards and buy links: <a href='FFC-arm-stacks-blueprint.html'>FFC-arm-stacks-blueprint.html</a>. Links in the parts database were checked today; five dead ones (DigiKey .cn, Qviro, Gibson, Eezee, SMC reseller) were replaced.</p>")

# ---- videos
parts.append("<h2>7. Proof videos and pages</h2><ul>" + "".join(f"<li>{link(t, u)}</li>" for t, u in COMMON["videos"]) + "</ul><p>None of the published demos flips a hinged ZIF latch; they use one-action-lock connectors or an already-open receptacle. The latch sequence is ours, verified only in the twin so far. The Meca500 pages prove the arm holds 1 to 25 um in production; the FANUC page proves the LR Mate does force-guided connector assembly.</p>")

# ---- open items
_, _, orows = review_rows("meca500/megp_nano25", ("task", "electrics", "laptop", "cable", "gripper", "sensor"))
orows = [x for x in orows if x["status"] == "CHECK"]
parts.append("<h2>8. Still open before the first real run</h2>" + table(["item", "what we assume", "what to do"], [[e(x["item"]), e(str(x["value"]) + (" / " + str(x["limit"]) if x.get("limit") else "")), e(x.get("note", ""))] for x in orows]))
# ---- spec compliance, part by part, read from the datasheets on 2026-10-13
mA = R["meca500/megp_nano25"]; mB = R["lrmate200id/lehz16_mini40"]
comA = (mA["com_mm"][0] ** 2 + mA["com_mm"][1] ** 2) ** 0.5; comB = (mB["com_mm"][0] ** 2 + mB["com_mm"][1] ** 2) ** 0.5
SRC = {"meca": ("Meca500 manual 2026.B, Table 7, Fig. 19, Fig. 33", "https://resources.mecademic.com/en/doc/MC-UM-MECA500/2026.B/mc-um-meca500.pdf"),
       "megp": ("MEGP 25 manual, spec table and Fig. 2", "https://hs.mecademic.com/hubfs/MEGP%2025E/MEGP%2025E%20Documentation%20(Latest)/MC-UM-MEGP25-EN-10-1.pdf"),
       "nano25": ("ATI Nano25 page and drawing 9230-05-1312", "https://ati.novanta.com/product/nano25-force-torque-sensor/"), "mini": ("ATI Mini40 page", "https://ati.novanta.com/product/mini40-force-torque-sensor/"),
       "smc": ("SMC LEH catalogue, p. 812 and 816 and the spec table", "https://www.smcworld.com/catalog/Electric-en/mpv/Electric-LEH_en/data/Electric-LEH_en.pdf"),
       "fanuc": ("FANUC LR Mate 200iD datasheet; wrist moments from the mechanical unit manual B-83494EN/06 Table 3.1", "https://cdn.craft.cloud/6166486b-1eb4-43d5-990c-ea573de4e750/assets/files/data-sheets/lr-mate-200id-datasheet.pdf"),
       "basler": ("Basler daA1920-160um docs", "https://docs.baslerweb.com/daa1920-160um"), "eo": ("Edmund #37-182 spec sheet", "https://www.1stvision.com/lens/edmund-optics/dataman/spec_37182.pdf"),
       "d405": ("RealSense D400 series datasheet, Oct 2025", "https://realsenseai.com/wp-content/uploads/2025/09/Intel-RealSense-D400-Series-Datasheet-October-2025.pdf"),
       "ccs": ("Graftek CCS LDL2-33X8BL2 page", "https://graftek.com/product/ccs-america-ldl2-33x8bl2-blue-470-nm-led-bar-light-44-x-10-4-x-13-l-x-w-x-h-mm-fcb/")}
def S_(k): return link(SRC[k][0], SRC[k][1])
COMP = [
 ("Meca500 R4", "rated payload", "0.5 kg (everything on the flange)", f"{mA['mass_g']} g", "OK", "meca"),
 ("Meca500 R4", "payload chart, Fig. 19, 0.5 kg curves", "at 75% joint acceleration the centre of mass may sit 40 mm below the flange at 30 mm off axis, 30 mm below at 40 mm off axis; at 50%: 60 mm below at 30 mm off axis; 100% allowed with 0.5 kg when joint 6 is aligned with gravity", f"{mA['com_mm'][2]:.0f} mm below, {comA:.0f} mm off axis, at {mA['mass_g']} g (64% of the chart's mass)", "OK at 75% acceleration or less; the tool hangs flange-down so joint 6 is near gravity-aligned", "meca"),
 ("Meca500 R4", "moment on the flange", "3.4 N m max", "0.70 N m worst case (20 N press at 35 mm)", "OK", "meca"),
 ("Meca500 R4", "flange, Fig. 33", "4 x M3x0.5, 4 deep, on 15.5 BCD; 2 x dia 3, 3.5 deep, on 15.5 BCD; dia 9 H7, 2.2 deep; dia 20 disk; M3 at 1.5 N m, thread penetration 4 mm, no more", "adapter plate: 4 x M3 flat head on 15.5 PCD, 2 x 3 mm dowels, 9 h6 boss 2 mm tall; M3 x 8 flat head through the 4 mm plate gives 4 mm in the flange", "OK (bore is H7 in the 2026.B drawing, not H6 as noted earlier; the h6 boss still fits)", "meca"),
 ("Meca500 R4", "joint speeds, R4", "J1 J2 225, J3 225, J4 J5 350, J6 500 deg/s", "under 10 deg/s in the task (2 mm/s at the tool)", "OK", "meca"),
 ("Meca500 R4", "input power", "24 V DC, 5.5 A max; PS200 90 to 250 V AC", "PS200 on 120 V", "OK", "meca"),
 ("MEGP 25E", "finger mass", "0.02 kg per finger", "lower 2 g printed + 1 g steel shoe, upper 5 g (volumes at 1.2 g/cc)", "OK, 4x margin; weigh the prints", "megp"),
 ("MEGP 25E", "finger reach", "sqrt(x2 + z2) <= 30 mm from the jaw face", "upper 28 mm, lower 9 mm", "OK", "megp"),
 ("MEGP 25E", "jaw loads, Fig. 2", "|Mx| 1.50, |My| 0.50, |Mz| 0.42 N m, |Fx| 70 N", "pinch: 0.14 N m; press on the lower jaw: 0.32 N m (20 N worst case); Fx 20 N", "OK", "megp"),
 ("MEGP 25E", "gripping force", "0 to about 40 N, SetGripperForce 5 to 100%", "5 N = 12%", "OK", "megp"),
 ("MEGP 25E", "stroke, repeatability, mass, temperature", "3 mm per jaw, 0.03 mm, 0.106 kg, 5 to 55 C", "1.5 mm per jaw used; lab 20 C", "OK", "megp"),
 ("ATI Nano25 SI-125-3", "force and torque ranges, resolution", "Fxy 125 N, Fz 500 N, Txy 3 N m, Tz 3 N m; resolution 0.021 N (1/48 N), 0.0004 N m", "slide guard 4 N = 0.28 N m; latch push 7 N = 0.50 N m; push guard 12 N = 0.86 N m (tip 72 mm below the sensor with the taller body); press 10 N = 0.32 N m, 20 N worst = 0.63 N m", "OK, every task load under 60% of range; 10 mN resolution is 10x the 0.1 N need", "nano25"),
 ("ATI Nano25 SI-125-3", "single-axis overload", "Fxy 2300 N, Fz 7300 N, Txy 43 N m, Tz 63 N m", "20 N, 0.86 N m worst case", "OK", "nano25"),
 ("ATI Nano25 SI-125-3", "mounting", "3 x M3x0.5 7.0 deep + 2 x 3 mm dowels 7.0 deep on a 20.32 mm circle (mounting side); 3 x M3 + 2 x 3 mm dowels 3.3 deep (tool side); 25 mm dia, 21.6 mm tall, 63.4 g (drawing 9230-05-1312)", "28 mm round adapter plate with counterbored M3 and 3 mm dowels; bracket leg counterbored M3 with 3 mm dowels 3 mm deep", "OK", "nano25"),
 ("LR Mate 200iD", "payload, reach, repeatability, mass", "7 kg, 717 mm, +-0.01 mm, 25 kg", f"{mB['mass_g']} g on the wrist", "OK", "fanuc"),
 ("LR Mate 200iD", "wrist allowable moment and inertia", "J4 J5 16.6 N m, J6 9.4 N m; 0.47 / 0.47 / 0.15 kg m2", f"0.88 N m worst case; COM {mB['com_mm'][2]:.0f} mm below, {comB:.0f} mm off axis", "OK", "fanuc"),
 ("LR Mate 200iD", "flange", "ISO 9409-1-31.5-4-M5", "adapter: 4 x M5 flat head on 31.5 PCD, 5 H7 dowel, 8 mm max in the flange", "OK", "fanuc"),
 ("SMC LEHZ16LK2-6", "gripping force (compact)", "3 to 8 N, pushing force 40 to 100%, accuracy +-30% F.S.; at gripping point L = 46 mm the p. 812 graph gives about 7 N at 100% and 5 N at 70%", "5 N at L = 46 mm (upper finger)", "OK: set about 70%, confirm on a scale", "smc"),
 ("SMC LEHZ16LK2-6", "gripping point L", "graphed to 110 mm (p. 812)", "46 mm upper, 18 mm lower", "OK (the earlier 40 mm limit was an estimate)", "smc"),
 ("SMC LEHZ16LK2-6", "static allowable finger loads (p. 816)", "Fv 98 N, pitch Mp 0.68, yaw My 0.68, roll Mr 1.36 N m; allowable F = M / L", "pinch 5 N at 46 mm = 0.23 N m; press 20 N at 25 mm = 0.50 N m", "OK (3x and 1.4x)", "smc"),
 ("SMC LEHZ16LK2-6", "stroke, speeds, repeatability, backlash", "6 mm both sides; open/close 5 to 80 mm/s, pushing 5 to 50 mm/s; repeatability +-0.02 mm; backlash 0.25 mm per side; lost motion 0.3 mm", "1.5 mm per jaw; the backlash is in the closing direction only, taken up by the pinch", "OK", "smc"),
 ("SMC LEHZ16LK2-6", "mass, temperature, enclosure", "190 g, 5 to 40 C, IP20, 24 V DC", "lab bench", "OK", "smc"),
 ("ATI Mini40 SI-40-2", "ranges, resolution, overload", "Fxy 40 N, Fz 120 N, Txy 2 N m, Tz 2 N m; resolution 0.01 N, 0.02 N, 0.0003 N m; overload 810 N, 2400 N, 19 N m", "slide 4 N, push 12 N (Fxy); press 10 N, 20 N worst (Fz); torque: 12 N push guard at 78 mm = 0.94 N m, 20 N press at 41 mm = 0.82 N m", "OK, 47% of the torque range at the push guard", "mini"),
 ("ATI Mini40 SI-40-2", "mounting", "3 x M3 on 33 PCD, tool side 3.0 mm deep, base 5.0 mm; 2 x 3 mm dowels", "bracket: M3 x 6 counterbored into 3.0 mm; adapter: 5 mm engagement", "OK", "mini"),
 ("Basler daA1920-160um", "sensor and optics", "IMX392, 1/2.3 inch, 7.9 mm diagonal, 3.45 um, 1920 x 1200, global shutter, S-mount", "with the 16 mm lens: 31 um per pixel, 60 x 37 mm field at 160 mm", "OK", "basler"),
 ("Basler daA1920-160um", "frame rate, power, temperature, size", "67 fps default, 164 fps with the throughput limit off; 1.9 W at 5 V; S-mount housing 0 to 50 C; 20 x 29 x 29 mm, under 15 g", "30 to 60 fps for the policy; bench posts", "OK; status: not in series production yet (docs page)", "basler"),
 ("Edmund #37-182", "working distance", "150 mm to infinity", "160 mm", "OK (was 108 mm on the wrist, out of spec)", "eo"),
 ("Edmund #37-182", "image circle and format", "9.00 mm max image circle, 1/1.8 inch max format; 1/2.5 inch horizontal field 52.5 mm at the reference distance", "Basler 1/2.3 inch, 7.9 mm diagonal", "OK, no vignetting", "eo"),
 ("Edmund #37-182", "mechanical", "M12 x 0.5, 22.5 mm long, 16 mm max diameter, BFL 10.05 to 10.79 mm, 400 to 700 nm, f/4", "modelled 16 dia x 16.5 long; S-mount holder on the dart", "CHECK the dart's S-mount back focus takes 10.05 to 10.79 mm (standard S-mount: expected yes)", "eo"),
 ("RealSense D405", "depth range", "7 cm to 1 m (varies with lighting)", "113 mm from the camera to the jaw tips", "OK", "d405"),
 ("RealSense D405", "mass and mounting", "58 g +-10%; 2 x M3 on the back 20 mm apart, 0.4 N m max; 1/4-20 bottom", "back plate with 2 x M3", "OK", "d405"),
 ("CCS LDL2-33X8BL2", "electrical", "24 V, 0.8 W, external controller required, no strobe control listed, 0 to 40 C, 20 g, $414", "continuous, 60 mm from the mouth at 25 deg", "OK; strobe not available: use a short exposure instead", "ccs"),
]
def _st(st):
    if st.startswith("OK"):
        return status("OK") + " " + e(st[2:].strip(" ,:;"))
    return status("CHECK") + " " + e(st[5:].strip(" ,:;"))
parts.append("<h2>9. Spec compliance, part by part (datasheets read 2026-10-13)</h2><p>Each line is the manufacturer's figure for the exact model we specify against what the twin and the task put on it. Corrections that came out of this pass: the MEGP 25E's Mx limit is 1.50 N m (1.30 is the 25LS), the Meca500 flange bore is 9 H7 in the current drawing, and the originally modelled Nano17 SI-50-0.5 would have clipped its 0.5 N m torque channel at the 12 N push guard (a horizontal tip force acts on the whole stack length), which is why the Meca500 now carries the Nano25 SI-125-3 and the LR Mate the Mini40 in its SI-40-2 calibration.</p>" +
             table(["part", "item", "datasheet", "ours", "status", "source"], [[e(a), e(b), e(c), e(d), _st(st), S_(k)] for a, b, c, d, st, k in COMP]))

parts.append("</div></body></html>")
DEST.write_text("\n".join(parts)); print("wrote", DEST)
