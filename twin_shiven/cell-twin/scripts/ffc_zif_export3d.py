"""Export a variant's scene and recorded run as one standalone interactive 3D page (three.js): every geom to scale, grouped by
layer with its own colour, a 10 mm grid, a frame slider that scrubs the recorded trajectory, and a legend.
usage: FFC_ARM=meca500 FFC_TOOL=megp_nano17 FFC_CASE=laptop .venv/bin/python scripts/ffc_zif_export3d.py [out.html]"""
import json, math, os, sys
from pathlib import Path
import mujoco, numpy as np
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from cell_twin import ffc_zif as F
from cell_twin import ffc_render_kit as K

ARM = os.environ.get("FFC_ARM", "meca500"); CASE = os.environ.get("FFC_CASE", "laptop"); TOOL = os.environ.get("FFC_TOOL", F.DEFAULT_TOOL[ARM])
VAR = f"{ARM}-{TOOL}-{CASE}"; OUTDIR = ROOT / "out" / "ffc_zif" / VAR
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else OUTDIR / "viewer.html"
spec = F.build(ARM, CASE, TOOL); model = spec.compile(); data = mujoco.MjData(model)
tr = np.load(OUTDIR / "traj.npz"); Q = tr["qpos"]; T = tr["t"]; PA = list(tr["phase_at"]); PL = [str(p) for p in tr["phase_lbl"]]
STRIDE = max(1, len(Q) // 360)

gname = lambda i: mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or f"geom{i}"
bname = lambda i: mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or f"body{i}"
prefix = F.ARM_INFO[ARM]["prefix"]


def layer_of(g, b):
    if g in ("floor", "breadboard", "arm_plate", "cam_post") or b in ("world",) and g.startswith(("coupling",)):
        return "bench"
    if b == "pallet" and (g.startswith(("pallet", "coupling", "fiducial", "fid_", "stop_", "presenter", "feed", "platform"))):
        return "pallet"
    if g in ("deck", "wall_n", "wall_s", "wall_e", "wall_w", "mainboard", "dimm_slot", "dimm_module", "mid_rail"):
        return "laptop"
    if g.startswith("zif") or b.startswith("zif"):
        return "connector"
    if g.startswith("ffc") or b.startswith("ffc"):
        return "cable"
    if g.startswith(("stack_", "bracket_")):
        return "wrist stack"
    if g.startswith(("gripper_", "jaw_")):
        return "gripper body"
    if g.startswith(("shoe_", "blade", "plate_", "leg_", "pad_")):
        return "fingers"
    if g.startswith(("macro_", "d405")) or b.startswith("macro_"):
        return "cameras"
    if b.startswith(prefix) or g.startswith(prefix):
        return "arm"
    return "other"


PARTS = json.loads((ROOT / "assets" / "ffc" / "parts.json").read_text())["parts"]
TI = F.TOOL_INFO[TOOL]; GR = F.GRIPPERS[TI["gripper"]]; IDS = GR.get("ids", {}); CELLS = bool(GR.get("cells"))
STACK_PID = {pid: part for pid, part, *_ in F.STACKS[TI["stack"]]}
ARM_LABEL = {"meca500": "Meca500", "lrmate200id": "LR Mate 200iD"}[ARM]


def catalogue(pid):
    """vendor, number, price, mass and the catalogue's own one-line description for a part id"""
    q = PARTS.get(pid)
    if not q:
        return ""
    bits = [q.get("vendor", ""), q.get("part_number", "") if q.get("part_number") not in (None, "n/a") else ""]
    if isinstance(q.get("price_usd_if_listed"), (int, float)):
        bits.append(f"${q['price_usd_if_listed']:,.2f}")
    if q.get("mass_g"):
        bits.append(f"{q['mass_g']} g")
    head = " · ".join(b for b in bits if b)
    return (head + "\n" if head else "") + (q.get("what_it_is", "") or "") + (f"\n{q['buy_url']}" if q.get("buy_url") and q["buy_url"] not in ("none", "") else "")


def describe(g, b):
    """title, one-line summary, and the detail text shown when a part is clicked"""
    def P(pid, title, what, extra=""):
        return dict(title=title, what=what, detail=(extra + "\n" if extra else "") + catalogue(pid), part=pid)
    if g == "stack_flange": return P(STACK_PID.get("flange", ""), f"{ARM_LABEL} flange", "the arm's tool mounting face; everything below it is ours")
    if g == "stack_adapter_plate": return P(STACK_PID.get("adapter_plate", ""), "Adapter plate (ours, 6061)", "bolts to the flange pattern on top, carries the stack below")
    if g in ("stack_nano25", "stack_nano17", "stack_mini40"): return P(STACK_PID.get(g[6:], ""), "ATI six-axis force/torque sensor", "measures all forces and torques at the wrist; the slide, push and press guards read it at 7 kHz")
    if g == "stack_standoff": return P(STACK_PID.get("standoff", ""), "Printed standoff", "a spacer that puts the jaw line where the sensor stacks have it; cell A bolts under it")
    if g == "stack_zaber": return P("zaber_xna08a25", "Zaber linear stage", "the fine-axis variant: a 25 mm stage between the sensor and the gripper")
    if g == "bracket_h":
        return P("load_cell_tal220b", "Cell A: bar load cell, insertion axis", "fixed end under the standoff, free end carries the corner plate; it bends along the body axis and reads the slide friction, the seat rise and the lid push (guards at 4 and 12 N)") if CELLS else P(IDS.get("bracket", ""), "90 degree bracket, horizontal leg (ours, 6061)", "under the sensor, reaches east to carry the gripper beside the cable")
    if g == "bracket_v":
        return P("bracket_servo", "Side plate (ours, 6061, 4 mm)", "down the body's east face from cell B's free end; 4 x M3 into the printed body") if CELLS else P(IDS.get("bracket", ""), "90 degree bracket, vertical leg (ours, 6061)", "drops from the horizontal leg to the gripper's mounting face")
    if g == "cell_b": return P("load_cell_tal220b", "Cell B: bar load cell, vertical axis", "fixed end on the corner plate, runs along the body; it bends along the flange axis and reads the lid lift (2 N) and the latch press (10 N, guard 10, worst 20)")
    if g == "cell_corner": return P("bracket_servo", "Corner plate (6061, 4 mm)", "joins cell A's free end to cell B's fixed end on top")
    if g == "cell_tab": return P("bracket_servo", "Tab on cell B's free end (6061, 4 mm)", "one piece with the side plate")
    if g == "cell_p": return P("load_cell_tal220b", "Cell P: bar load cell, the pinch", "the upper finger's cross member: fixed end on the upper jaw carrier, the pad hangs on its free end; the servo closes until this reads 5 N")
    if g == "gripper_body":
        return P(IDS.get("part", ""), "Printed body: XL330-M077-T servo + M4 x 1 lead screw", "the servo lies flat in a pocket at the back with its horn up; a 3 mm GT2 belt over the top turns the lead screw 1:1 through two 12-tooth pulleys; the screw runs down through the body's top wall into the nut in the upper carrier; 0.24 um per encoder step, self-locking", "servo " + catalogue("xl330_m077")) if CELLS else P(IDS.get("part", ""), "Bought gripper body", "an electric parallel gripper with its own controller")
    if g == "gripper_cover": return P("servo_tool_v1", "Belt cover (printed with the body)", "a 6 mm cover over the 3 mm GT2 belt that links the servo's horn pulley to the pulley on top of the lead screw, 1:1")
    if g == "gripper_end":
        return P("servo_rocker", "End block: slot, two carriers, rocker", "the two printed carriers run in the printed slot; the upper one holds the PEEK nut on the screw (its guide), the printed rocker at the front-west corner moves the lower one the opposite way, so both jaws move 1.5 mm, the same motion the bought grippers make and the twin was validated with. The 20 N latch press goes through the lower carrier into the slot walls", "rocker: " + catalogue("servo_rocker")) if CELLS else P(IDS.get("part", ""), "Jaw guide of the bought gripper", "the jaws run in it")
    if g in ("gripper_mesh", "gripper_plate", "gripper_plate_mesh", "gripper_conn"): return P(IDS.get("part", ""), "Bought gripper" + (" side plate" if "plate" in g else " connector" if g == "gripper_conn" else " (CAD shape)"), "electric parallel gripper; the jaws move 1.5 mm each to close the 3 mm gap")
    if g.startswith("jaw_lo"): return P(IDS.get("jaw_r" if CELLS else "jaw_l", ""), "Lower jaw carrier (rocker-driven)" if CELLS else "Lower jaw", "carries the lower finger: MEGP pattern, M4 + two 1.5 mm pins")
    if g.startswith("jaw_up"): return P(IDS.get("jaw_l" if CELLS else "jaw_r", ""), "Upper jaw carrier on the PEEK nut" if CELLS else "Upper jaw", "carries the upper finger" + (" (cell P's fixed end)" if CELLS else ""))
    if g == "plate_lo": return P(IDS.get("finger_l", ""), "Lower finger plate (printed PAHT-CF)", "bridges 4 mm over the cable from the jaw to the one east leg; the slide, push and press loads pass through it")
    if g == "leg_e": return P(IDS.get("finger_l", ""), "East leg of the lower finger (6 x 6 mm)", "drops from the plate to the shoe bar; only one leg, east of the cable, so the sideways exit never sweeps through the seated cable")
    if g == "shoe_bar": return P(IDS.get("finger_l", ""), "Shoe bar", "carries the steel shoe west under the cable")
    if g == "shoe_g": return P("finger_shoe_0p3", "Steel shoe, 0.5 mm shim, 20 x 7 mm", "slides 14 mm under the cable's stiffened tip and rides 0.17 mm over the board into the connector; its rear edge pushes the lid shut in phase 3")
    if g == "blade": return P("finger_shoe_0p3", "The 1.5 mm lip", "hooks 1 mm under the lid's front edge and lifts it along its arc to open the latch (2 N)")
    if g == "plate_up": return P(IDS.get("finger_r", ""), "Upper finger plate (printed)", "from the upper jaw forward to the pad arm" + ("; its rear 12.7 mm is cell P" if CELLS else ""))
    if g == "plate_up_tongue": return P(IDS.get("finger_r", ""), "Upper plate tongue (printed)", "on top of cell P's free end: 2 x M5 down into the cell's threaded holes carry the whole upper finger from the cell")
    if g == "plate_up_web": return P(IDS.get("finger_r", ""), "Upper plate web (printed)", "joins the tongue on the cell down to the plate level ahead of the cell")
    if g == "pad_leg_e": return P(IDS.get("finger_r", ""), "East leg of the upper finger", "drops from the upper plate to the pad arm, east of the cable")
    if g == "pad_arm": return P(IDS.get("finger_r", ""), "Pad arm", "12 to 15 mm ahead of the jaw line, 8 mm tall; carries the pad over the cable")
    if g == "pad_g": return P(IDS.get("finger_r", ""), "Silicone pad, 30 x 6 mm", "closes the 3 mm gap and pinches the cable's stiffener at 5 N against the shoe; the slide carries the cable by friction here")
    if g == "d405_g": return P("d405", "Intel RealSense D405 wrist camera", "RGB and depth for the policy's wrist view; 7 cm minimum range, looks at the jaw tips")
    if g.startswith("arm_d405"): return P("d405", "D405 mount (ours, 6061)", "bars under the bracket's west end and a side plate on the camera's west face")
    if g.startswith("fixcam_"):
        side = "east" if "_e_" in g else "west"; kind = g.split("_")[-1]
        return P("macro_cam_basler_dart", f"Bench camera, {side}" + {"g": "", "lens": ": 16 mm lens (Edmund #37-182)", "block": ": tilt block (printed)", "post": ": 12.7 mm post"}.get(kind, ""), "Basler dart + 16 mm lens on the bench, 160 mm from the mouth, 25 deg down: 31 um per pixel; these two views are the policy's eyes and the lateral/yaw measurement before the slide")
    if g.startswith("light_"): return P("light_blue_470", "Blue bar light" + (" post" if g.endswith("post") else ""), "470 nm bar on the pallet, 60 mm west of the mouth, 25 deg down")
    if g in ("cam_post", "arm_plate", "breadboard"): return dict(title={"cam_post": "Camera post", "arm_plate": "Arm mounting plate", "breadboard": "Optical breadboard"}[g], what="bench fixture", detail="")
    if g.startswith("zif"): return dict(title={"zif_floor": "Connector slot floor", "zif_contacts": "Connector contacts", "zif_guide_e": "Guide wall, east", "zif_guide_w": "Guide wall, west", "zif_lid_plate": "Flip lid (the latch)", "zif_roof": "Connector roof", "zif_stop": "Back wall (the seat, 3.5 mm in)"}.get(g, g), what="ASUS TUF FX505 keyboard FFC connector (CON3104 class, 0.5 mm pitch flip-latch ZIF)", detail="opened by the lip in phase 1, the cable slides 3.5 mm to the back wall in phase 2, the lid is pushed and pressed shut in phase 3")
    if g.startswith("ffc"): return dict(title="Cable stiffener (0.3 mm)" if "stiff" in g else "Cable root" if "anchor" in g else f"Cable segment {g[7:8]}", what="the keyboard FFC: 28 mm wide polyimide, stiffened tip", detail="the pinch holds the stiffener; the body floats behind it on its rest")
    if g in ("deck", "wall_n", "wall_s", "wall_e", "wall_w", "mainboard", "dimm_slot", "dimm_module", "mid_rail"): return dict(title={"deck": "Laptop deck", "mainboard": "Mainboard", "dimm_slot": "SO-DIMM slot", "dimm_module": "SO-DIMM module", "mid_rail": "Deck rail"}.get(g, "Laptop wall"), what="the open laptop around the connector", detail="the parts the stack must clear: the 14 mm edge wall, the SO-DIMM 25 mm behind, the 4 mm rail 40 mm behind")
    if b == "pallet" or g.startswith(("pallet", "coupling", "fiducial", "fid_", "stop_", "presenter", "feed", "platform")): return dict(title="Pallet fixture: " + g, what="holds the laptop (or the cable presenter on the platform case) at a known place", detail="")
    if b.startswith(prefix): return dict(title=f"{ARM_LABEL}: {b.replace(prefix + '_', '').replace('_', ' ')}", what="the robot arm", detail={"meca500": "Mecademic Meca500 R4: 0.5 kg payload, 5 um repeatability", "lrmate200id": "FANUC LR Mate 200iD: 7 kg payload, 0.01 to 0.02 mm repeatability"}[ARM])
    return dict(title=g, what="", detail="")


COLORS = {"bench": "#3b3b40", "pallet": "#e09a3c", "laptop": "#1f4fa8", "connector": "#2a2a2e", "cable": "#2563eb", "wrist stack": "#0e8a9b",
          "gripper body": "#555a60", "fingers": "#ee8c32", "cameras": "#111111", "arm": "#8a8a90", "other": "#999999"}
meshes = {}; inst = []
for i in range(model.ngeom):
    g = gname(i); b = bname(model.geom_bodyid[i]); lay = layer_of(g, b)
    if g == "floor":
        continue
    t = int(model.geom_type[i]); size = model.geom_size[i].tolist(); pos = model.geom_pos[i].tolist(); quat = model.geom_quat[i].tolist()
    rgba = model.geom_rgba[i].tolist()
    entry = {"name": g, "body": int(model.geom_bodyid[i]), "layer": lay, "pos": pos, "quat": quat, "rgba": rgba, "group": int(model.geom_group[i]), "info": describe(g, b), "mat": K.material_key(g, lay, ARM, cells=CELLS)}
    if t == mujoco.mjtGeom.mjGEOM_BOX:
        entry.update(type="box", size=size)
    elif t == mujoco.mjtGeom.mjGEOM_CYLINDER:
        entry.update(type="cylinder", size=size)
    elif t == mujoco.mjtGeom.mjGEOM_MESH:
        mid = int(model.geom_dataid[i]); mn = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mid) or f"mesh{mid}"
        if mn not in meshes:
            va, vn = int(model.mesh_vertadr[mid]), int(model.mesh_vertnum[mid]); fa, fn = int(model.mesh_faceadr[mid]), int(model.mesh_facenum[mid])
            V, Fc = model.mesh_vert[va:va + vn].astype(np.float32), model.mesh_face[fa:fa + fn].astype(np.int64)
            cap = 24000 if mn.startswith("lrm_") else 12000       # 2026-10-13: the LR Mate wrist and forearm looked crushed at 6000 faces; keep its CAD whole
            if fn > cap and not mn.startswith(("megp", "zaber")):
                import fast_simplification
                V, Fc = fast_simplification.simplify(V, Fc, target_reduction=1 - cap / fn)
            meshes[mn] = {"v": np.round(np.asarray(V), 5).flatten().tolist(), "f": np.asarray(Fc).flatten().tolist()}
        entry.update(type="mesh", mesh=mn)
    else:
        continue
    inst.append(entry)
# our detailed visual meshes, attached to the bodies they belong to (the simple boxes stay as the physics layer)
A = ROOT / "assets" / "ffc"


def load_obj(name):
    v, f = [], []
    for line in (A / f"{name}.obj").read_text().splitlines():
        if line.startswith("v "):
            v.append([float(x) for x in line.split()[1:4]])
        elif line.startswith("f "):
            f.append([int(x.split("/")[0]) - 1 for x in line.split()[1:4]])
    return {"v": np.round(np.array(v), 5).flatten().tolist(), "f": np.array(f).flatten().tolist()}


detail = [("zif_housing", "zif", [0, 0, 0], "#2a2a2e", "connector (detail)"), ("zif_actuator", "zif_lid", [0, 0, 0], "#e6e3da", "connector (detail)"),
          ("ffc_stiffener", "ffc_stiffener", [0, 0, 0], "#2563eb", "cable (detail)")]
for mn, body, off, col, lay in detail:
    if (A / f"{mn}.obj").exists():
        meshes[mn] = load_obj(mn)
        inst.append({"name": mn, "body": int(model.body(body).id), "layer": lay, "pos": off, "quat": [1, 0, 0, 0], "rgba": [0, 0, 0, 1], "group": 0, "type": "mesh", "mesh": mn, "color": col})
COLORS.update({"connector (detail)": "#2a2a2e", "cable (detail)": "#2563eb", "fingers (detail)": "#ee8c32"})
# frames: body poses
frames = []
for k in range(0, len(Q), STRIDE):
    data.qpos[:] = Q[k]; mujoco.mj_forward(model, data)
    frames.append({"t": round(float(T[k]), 3), "p": np.round(data.xpos, 5).flatten().tolist(), "q": np.round(data.xquat, 5).flatten().tolist()})
phases = [{"frame": int(a // STRIDE), "label": l} for a, l in zip(PA, PL)]
o = F.connector_origin(ARM)
FAST = K.to_body_frames(F, K.fasteners_tool_frame(F, ARM, TOOL, model), model)
BODY_ID = {"tool": int(model.body("tool").id), "jaws": int(model.body("jaws").id), "pad": int(model.body("pad").id), "shoe": int(model.body("shoe").id)}
for f_ in FAST:
    f_["body"] = BODY_ID[f_["body"]]
payload = {"variant": VAR, "fasteners": FAST, "arm": ARM, "label": f"{ARM} · {F.TOOL_INFO[TOOL]['label']} · {F.CASES[CASE]}", "nbody": int(model.nbody), "bodies": [bname(i) for i in range(model.nbody)],
           "meshes": meshes, "geoms": inst, "frames": frames, "phases": phases, "colors": COLORS, "origin": o.tolist()}
js = json.dumps(payload, separators=(",", ":"))
html = """<!doctype html><html><head><meta charset="utf-8"><title>FFC cell twin, interactive</title>
<style>body{margin:0;background:#FBF7F0;font:14px -apple-system,Inter,Segoe UI,sans-serif;color:#1E1B16;overflow:hidden}
#ui{position:absolute;left:12px;top:12px;width:300px;background:rgba(251,247,240,.95);border-radius:12px;padding:12px 14px;box-shadow:0 2px 12px rgba(0,0,0,.15);max-height:95vh;overflow:auto}
#ui h1{font-size:15px;margin:0 0 4px}#ui .sub{color:#6B6257;font-size:12px;margin:0 0 8px}
.row{display:flex;align-items:center;gap:8px;margin:3px 0;font-size:13px}.sw{width:12px;height:12px;border-radius:3px;display:inline-block}
#bar{position:absolute;left:330px;right:12px;bottom:12px;background:rgba(251,247,240,.95);border-radius:12px;padding:10px 14px;box-shadow:0 2px 12px rgba(0,0,0,.15)}
#bar input[type=range]{width:100%}#phase{font-weight:600}button{border:0;background:#1E1B16;color:#fff;border-radius:8px;padding:5px 10px;cursor:pointer}
#hint{position:absolute;right:12px;top:12px;background:rgba(251,247,240,.9);border-radius:10px;padding:8px 12px;font-size:12px;color:#6B6257}
#tip{position:absolute;pointer-events:none;display:none;background:#1E1B16;color:#fff;border-radius:8px;padding:6px 10px;font-size:12px;max-width:320px;z-index:5}#tip b{display:block;font-size:13px;margin-bottom:2px}
#detail{position:absolute;right:12px;top:56px;width:340px;background:rgba(251,247,240,.97);border-radius:12px;padding:12px 14px;box-shadow:0 2px 12px rgba(0,0,0,.15);font-size:13px;display:none;max-height:70vh;overflow:auto;white-space:pre-wrap}
#detail h2{font-size:15px;margin:0 0 6px}#detail .x{float:right;cursor:pointer;color:#6B6257}#detail a{color:#1f6fb2;word-break:break-all}
#parts{margin-top:8px;border-top:1px solid #EFE6D6;padding-top:6px}#parts div{cursor:pointer;padding:2px 0;font-size:12.5px}#parts div:hover{color:#b23}#parts .grp{font-weight:600;margin-top:5px;cursor:default;color:#6B6257;font-size:11px;letter-spacing:.06em;text-transform:uppercase}
</style></head><body>
<div id="ui"><h1 id="title"></h1><p class="sub">To scale. Grid: 10 mm (fine lines 1 mm near the connector). Drag to orbit, scroll to zoom, right-drag to pan. Double-click a layer name to look at it.</p>
<div id="layers"></div><p class="sub" style="margin-top:8px">Hover any part for its name; click it for the full card (what it is, vendor, part number, price, its job in the run). Physics boxes are the simple shapes the simulation collides with; "detail" layers are the real-shaped parts drawn at the same places.</p>
<div id="parts"><div class="grp">Parts of the tool (click to fly to one)</div></div></div><div id="tip"></div><div id="detail"></div>
<div id="hint">views: <button onclick="window.look('connector')">connector</button> <button onclick="window.look('fingers')">fingers</button> <button onclick="window.look('wrist')">wrist stack</button> <button onclick="window.look('arm')">whole arm</button></div>
<div id="bar"><div class="row"><button id="play">play</button><span id="phase"></span><span id="time" style="color:#6B6257"></span></div><input type="range" id="frame" min="0" value="0"></div>
<script type="importmap">{"imports":{"three":"https://unpkg.com/three@0.160.0/build/three.module.js","three/addons/":"https://unpkg.com/three@0.160.0/examples/jsm/"}}</script>
<script type="module">
import * as THREE from 'three'; import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
const D = __DATA__;
__MATJS__
document.getElementById('title').textContent = D.label;
const scene = new THREE.Scene(); scene.background = new THREE.Color(0xFBF7F0);
const cam = new THREE.PerspectiveCamera(35, innerWidth/innerHeight, 0.001, 10);
const renderer = new THREE.WebGLRenderer({antialias:true}); renderer.setSize(innerWidth, innerHeight); renderer.setPixelRatio(devicePixelRatio); document.body.appendChild(renderer.domElement);
const controls = new OrbitControls(cam, renderer.domElement); controls.enableDamping = true; controls.zoomSpeed = 5; controls.zoomToCursor = true; controls.minDistance = 0.004; controls.maxDistance = 4; controls.rotateSpeed = 0.9;
await setupLook(renderer, scene, 0.9, [D.origin[0], D.origin[1], D.origin[2] + 0.15]);
scene.up = new THREE.Vector3(0,0,1); cam.up.set(0,0,1);
const o = D.origin;
// grids: 10 mm over the pallet, 1 mm near the connector
function grid(size, step, color, z, cx, cy){const g = new THREE.Group(); const m = new THREE.LineBasicMaterial({color, transparent:true, opacity:0.35}); const n = Math.round(size/step);
 for(let i=-n;i<=n;i++){const a=i*step; const p1=new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(cx-size,cy+a,z),new THREE.Vector3(cx+size,cy+a,z)]); const p2=new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(cx+a,cy-size,z),new THREE.Vector3(cx+a,cy+size,z)]); g.add(new THREE.Line(p1,m)); g.add(new THREE.Line(p2,m));} return g;}
scene.add(grid(0.15, 0.01, 0x6B6257, o[2]+0.0002, o[0], o[1])); scene.add(grid(0.02, 0.001, 0x6B6257, o[2]+0.0003, o[0], o[1]));
const layers = {}; const groups = {};
function layerGroup(name){ if(!groups[name]){ groups[name] = new THREE.Group(); scene.add(groups[name]); } return groups[name]; }
const meshGeo = {};
for(const [n,m] of Object.entries(D.meshes)){ const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.Float32BufferAttribute(m.v,3)); g.setIndex(m.f); g.computeVertexNormals(); meshGeo[n]=g; }
const objs = [];
for(const g of D.geoms){
 let geo; if(g.type==='box') geo = new THREE.BoxGeometry(2*g.size[0],2*g.size[1],2*g.size[2]);
 else if(g.type==='cylinder'){ geo = new THREE.CylinderGeometry(g.size[0], g.size[0], 2*g.size[1], 32); geo.rotateX(Math.PI/2); }
 else geo = meshGeo[g.mesh];
 const col = g.color || (g.type==='mesh' && g.layer==='arm' ? D.colors.arm : (g.rgba[3] < 1 && g.layer!=='connector' ? D.colors[g.layer] : D.colors[g.layer]));
 const physics = !g.color && ['connector','cable'].includes(g.layer);
 const mat = makeMaterial(g.mat, physics ? {opacity: 0.3} : (g.color && g.type==='mesh' && g.layer!=='arm' ? {color: g.color} : {}));
 const mesh = new THREE.Mesh(geo, mat); mesh.userData = g; mesh.castShadow = !physics; mesh.receiveShadow = true; const lay = physics ? g.layer + ' (physics boxes)' : g.layer;
 layerGroup(lay).add(mesh); objs.push(mesh);
}
for(const f of D.fasteners){ const grp = fastenerMesh(f, 1); grp.userData.body = f.body; grp.userData.pos = f.pos; grp.userData.quat = [1,0,0,0]; grp.userData.layer = 'fasteners'; grp.userData.name = 'fastener'; layerGroup('fasteners').add(grp); objs.push(grp); }
const B = D.nbody; const tmpQ = new THREE.Quaternion(), tmpP = new THREE.Vector3(), lq = new THREE.Quaternion(), lp = new THREE.Vector3();
function setFrame(k){ const f = D.frames[k]; for(const m of objs){ const b = m.userData.body; tmpP.set(f.p[3*b], f.p[3*b+1], f.p[3*b+2]); tmpQ.set(f.q[4*b+1], f.q[4*b+2], f.q[4*b+3], f.q[4*b]);
 lq.set(m.userData.quat[1], m.userData.quat[2], m.userData.quat[3], m.userData.quat[0]); lp.set(...m.userData.pos).applyQuaternion(tmpQ).add(tmpP);
 m.position.copy(lp); m.quaternion.copy(tmpQ).multiply(lq); }
 let ph = ''; for(const p of D.phases){ if(k >= p.frame) ph = p.label; } document.getElementById('phase').textContent = ph; document.getElementById('time').textContent = '  t = ' + f.t.toFixed(2) + ' s'; }
const slider = document.getElementById('frame'); slider.max = D.frames.length-1; slider.oninput = () => setFrame(+slider.value);
let playing = false; document.getElementById('play').onclick = () => { playing = !playing; document.getElementById('play').textContent = playing ? 'pause' : 'play'; };
// layer toggles
const order = ['arm','wrist stack','gripper body','fasteners','cameras','fingers (detail)','fingers (physics boxes)','cable (detail)','cable (physics boxes)','connector (detail)','connector (physics boxes)','laptop','pallet','bench','other'];
const lay = document.getElementById('layers');
for(const n of order){ if(!groups[n]) continue; const row = document.createElement('div'); row.className='row'; const cb = document.createElement('input'); cb.type='checkbox'; cb.checked = !n.includes('physics');
 groups[n].visible = cb.checked; cb.onchange = () => groups[n].visible = cb.checked; const sw = document.createElement('span'); sw.className='sw'; sw.style.background = D.colors[n.replace(' (physics boxes)','').replace(' (detail)','')] || '#999';
 const lab = document.createElement('span'); lab.textContent = n; lab.style.cursor='pointer'; lab.ondblclick = () => look(n.split(' ')[0]); row.append(cb, sw, lab); lay.appendChild(row); }
window.look = look; function look(what){ const d = {connector: 0.045, fingers: 0.09, wrist: 0.28, arm: 0.9}[what] || 0.1; const target = new THREE.Vector3(o[0], o[1] + (what==='arm'?0.1:what==='wrist'?0.03:0.005), o[2] + (what==='arm'?0.1:what==='wrist'?0.06:0.01));
 controls.target.copy(target); cam.position.set(target.x + d*0.7, target.y - d*0.7, target.z + d*0.5); controls.update(); }
// hover and click: part names and cards
const ray = new THREE.Raycaster(); const mouse = new THREE.Vector2(); const tip = document.getElementById('tip'); const detail = document.getElementById('detail'); let hot = null, pinned = null;
function pick(ev){ mouse.set((ev.clientX/innerWidth)*2-1, -(ev.clientY/innerHeight)*2+1); ray.setFromCamera(mouse, cam); const hits = ray.intersectObjects(objs.filter(m => m.visible && m.parent.visible), true); if(!hits.length) return null; let o = hits[0].object; while(o && !(o.userData && o.userData.info) && o.parent) o = o.parent; return o; }
function glow(m, on){ if(!m) return; const mats = m.material ? [m.material] : m.children.map(c => c.material); for(const mt of mats){ if(!mt) continue; mt.emissive = mt.emissive || new THREE.Color(0); mt.emissive.set(on ? 0xb23a3a : 0x000000); mt.emissiveIntensity = on ? 0.6 : 0; } }
renderer.domElement.addEventListener('mousemove', ev => { const m = pick(ev); if(m !== hot){ if(hot !== pinned) glow(hot, false); hot = m; if(hot) glow(hot, true); }
 if(m && m.userData.info && m.userData.info.title){ tip.style.display='block'; tip.style.left = (ev.clientX + 14) + 'px'; tip.style.top = (ev.clientY + 14) + 'px'; tip.innerHTML = '<b>' + m.userData.info.title + '</b>' + (m.userData.info.what || '') + '<div style="color:#bbb;margin-top:3px">click for the card · ' + m.userData.layer + '</div>'; } else tip.style.display='none'; });
renderer.domElement.addEventListener('mouseleave', () => { tip.style.display='none'; });
function showCard(m){ if(pinned && pinned !== m) glow(pinned, false); pinned = m; glow(m, true); const i = m.userData.info; const txt = (i.detail || '').replace(/(https?:\/\/\S+)/g, '<a href="$1" target="_blank">$1</a>');
 detail.style.display='block'; detail.innerHTML = '<span class="x" onclick="window.closeCard()">close</span><h2>' + i.title + '</h2>' + (i.what ? i.what + '\\n\\n' : '') + txt + '\\n\\n<span style="color:#6B6257">twin part: ' + m.userData.name + ' · layer: ' + m.userData.layer + '</span>'; }
window.closeCard = () => { detail.style.display='none'; if(pinned){ glow(pinned, false); pinned = null; } };
let downAt = null; renderer.domElement.addEventListener('mousedown', ev => { downAt = [ev.clientX, ev.clientY]; });
renderer.domElement.addEventListener('mouseup', ev => { if(!downAt || Math.hypot(ev.clientX-downAt[0], ev.clientY-downAt[1]) > 4) return; const m = pick(ev); if(m && m.userData.info && m.userData.info.title) showCard(m); });
// the parts list: one entry per title among the tool's own layers, fly to the part on click
const partsDiv = document.getElementById('parts'); const seen = {};
for(const lay of ['wrist stack','gripper body','fingers','cameras']){ const grp = document.createElement('div'); grp.className='grp'; grp.textContent = lay; let any = false;
 for(const m of objs){ if(m.userData.layer !== lay || !m.userData.info || !m.userData.info.title || seen[m.userData.info.title]) continue; seen[m.userData.info.title] = m; any = true;
  const d = document.createElement('div'); d.textContent = m.userData.info.title; d.onclick = () => { const p = new THREE.Vector3(); m.getWorldPosition(p); controls.target.copy(p); const dist = lay==='cameras' ? 0.12 : 0.06; cam.position.set(p.x + dist*0.6, p.y - dist*0.7, p.z + dist*0.45); controls.update(); showCard(m); };
  partsDiv.appendChild(grp); if(any && grp.parentNode !== partsDiv) partsDiv.appendChild(grp); partsDiv.appendChild(d); } }
const H = Object.fromEntries(location.hash.slice(1).split('&').filter(Boolean).map(kv => kv.split('='))); look(H.view || 'fingers'); const f0 = Math.min(+(H.frame || 0), D.frames.length-1); slider.value = f0; setFrame(f0);   // #view=connector&frame=120 opens at that view and frame
let last = performance.now();
function loop(now){ requestAnimationFrame(loop); if(playing && now-last > 40){ last = now; slider.value = (+slider.value + 1) % D.frames.length; setFrame(+slider.value);} controls.update(); renderer.render(scene, cam); }
loop(0); addEventListener('resize', () => { cam.aspect = innerWidth/innerHeight; cam.updateProjectionMatrix(); renderer.setSize(innerWidth, innerHeight); });
</script></body></html>""".replace("__DATA__", js).replace("__MATJS__", K.material_js())
OUT.write_text(html); print("wrote", OUT, f"{OUT.stat().st_size/1e6:.1f} MB,", len(frames), "frames,", len(inst), "geoms,", len(meshes), "meshes")
