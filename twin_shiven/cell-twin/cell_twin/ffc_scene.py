"""Scene collection shared by the interactive exporters: every geom of a built variant as a drawable (box, cylinder or mesh) with its
body index and a part id, the recorded run as body poses per frame, and the mapping from geom names to the parts database."""
import json
from pathlib import Path
import mujoco, numpy as np
from . import ffc_zif as F

ROOT = Path(__file__).resolve().parent.parent
ASSETS = ROOT / "assets" / "ffc"


def geom_part(g, b, arm, tool):
    """Which part a geom belongs to (ids match assets/ffc/parts.json)."""
    ti = F.TOOL_INFO[tool]; G = F.GRIPPERS[ti["gripper"]]; ids = G["ids"]; prefix = F.ARM_INFO[arm]["prefix"]
    stack_ids = {pid: part for pid, part, *_ in F.STACKS[ti["stack"]]}
    if g.startswith("stack_"):
        return stack_ids.get(g[6:], g[6:])
    if g in ("cell_b", "cell_p"):
        return "load_cell_tal220b"
    if g.startswith(("bracket_", "cell_")):
        return ids["bracket"]
    if g == "gripper_plate" or g.startswith("gripper_"):
        return ids["part"]
    if g.startswith("jaw_lo"):
        return ids["jaw_l"]
    if g.startswith("jaw_up"):
        return ids["jaw_r"]
    if g in ("shoe_g", "shoe_bar", "blade"):
        return "finger_shoe_0p3" if ti["shoe_t"] < 0.0004 else "shoe_steel"
    if g in ("plate_lo", "leg_e"):
        return ids["finger_l"]
    if g in ("plate_up", "pad_leg_e", "pad_arm"):
        return ids["finger_r"]
    if g == "pad_g":
        return "pad_rubber"
    if g.startswith("arm_"):
        return "camera_arms"
    if g.endswith("_lens"):
        return F.CAMERAS[arm]["lens"]
    if g == "light_g":
        return F.CAMERAS[arm]["light"]
    if g.startswith(("macro_", "fixcam_")) and not g.endswith(("_post", "_block")):
        return F.CAMERAS[arm]["macro"]
    if g.startswith("fixcam_") or g == "light_post":
        return "camera_stands"
    if g == "d405_g":
        return "d405"
    if b.startswith(prefix) or g.startswith(prefix):
        return "arm_" + arm
    if g.startswith("zif") or b.startswith("zif"):
        return "connector"
    if g.startswith("ffc") or b.startswith("ffc"):
        return "cable"
    if g in ("deck", "wall_n", "wall_s", "wall_e", "wall_w", "mainboard", "dimm_slot", "dimm_module", "mid_rail"):
        return "laptop"
    if g.startswith(("presenter", "feed_", "platform")):
        return "platform_fixture"
    if g.startswith(("pallet", "coupling", "fiducial", "fid_", "stop_")):
        return "pallet"
    if g in ("breadboard", "arm_plate", "cam_post"):
        return "bench"
    return "other"


def load_obj(name):
    v, f = [], []
    for line in (ASSETS / f"{name}.obj").read_text().splitlines():
        if line.startswith("v "):
            v.append([float(x) for x in line.split()[1:4]])
        elif line.startswith("f "):
            f.append([int(x.split("/")[0]) - 1 for x in line.split()[1:4]])
    return {"v": np.round(np.array(v), 5).flatten().tolist(), "f": np.array(f).flatten().tolist()}


def collect(arm, tool, case, meshes, max_faces=6000):
    """Build the variant and return (model, geoms, bodies). Shared mesh dict is filled in place (arm meshes decimated)."""
    spec = F.build(arm, case, tool); model = spec.compile()
    gname = lambda i: mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or f"geom{i}"
    bname = lambda i: mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or f"body{i}"
    geoms = []
    for i in range(model.ngeom):
        g = gname(i); b = bname(model.geom_bodyid[i])
        if g == "floor" or model.geom_rgba[i][3] == 0.0:
            continue
        t = int(model.geom_type[i]); entry = {"name": g, "body": int(model.geom_bodyid[i]), "part": geom_part(g, b, arm, tool),
                                               "pos": model.geom_pos[i].tolist(), "quat": model.geom_quat[i].tolist(), "rgba": model.geom_rgba[i].tolist()}
        if t == mujoco.mjtGeom.mjGEOM_BOX:
            entry.update(type="box", size=model.geom_size[i].tolist())
        elif t == mujoco.mjtGeom.mjGEOM_CYLINDER:
            entry.update(type="cylinder", size=model.geom_size[i].tolist())
        elif t == mujoco.mjtGeom.mjGEOM_MESH:
            mid = int(model.geom_dataid[i]); mn = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mid) or f"mesh{mid}"
            if mn not in meshes:
                va, vn = int(model.mesh_vertadr[mid]), int(model.mesh_vertnum[mid]); fa, fn = int(model.mesh_faceadr[mid]), int(model.mesh_facenum[mid])
                V, Fc = model.mesh_vert[va:va + vn].astype(np.float32), model.mesh_face[fa:fa + fn].astype(np.int64)
                cap = max(max_faces, 24000) if mn.startswith("lrm_") else max(max_faces, 12000)   # 2026-10-13: the LR Mate's wrist and forearm looked crushed at 6000 faces
                if fn > cap + 1000 and not mn.startswith(("megp", "zaber")):      # the gripper's and the Zaber's CAD keep full detail
                    import fast_simplification
                    V, Fc = fast_simplification.simplify(V, Fc, target_reduction=1 - cap / fn)
                meshes[mn] = {"v": np.round(np.asarray(V), 5).flatten().tolist(), "f": np.asarray(Fc).flatten().tolist()}
            entry.update(type="mesh", mesh=mn)
        else:
            continue
        geoms.append(entry)
    # detailed visual meshes for the connector and cable, drawn in place of the physics boxes
    for mn, body, col, part in (("zif_housing", "zif", "#2a2a2e", "connector"), ("zif_actuator", "zif_lid", "#e6e3da", "connector"), ("ffc_stiffener", "ffc_stiffener", "#2563eb", "cable")):
        if (ASSETS / f"{mn}.obj").exists():
            if mn not in meshes:
                meshes[mn] = load_obj(mn)
            geoms.append({"name": mn, "body": int(model.body(body).id), "part": part, "pos": [0, 0, 0], "quat": [1, 0, 0, 0], "rgba": [0, 0, 0, 1], "type": "mesh", "mesh": mn, "color": col, "detail": True})
    bodies = [bname(i) for i in range(model.nbody)]
    return model, geoms, bodies


def frames_of(model, traj_path, n_frames=240):
    tr = np.load(traj_path); Q = tr["qpos"]; T = tr["t"]; PA = list(tr["phase_at"]); PL = [str(p) for p in tr["phase_lbl"]]
    stride = max(1, len(Q) // n_frames); data = mujoco.MjData(model); frames = []
    for k in range(0, len(Q), stride):
        data.qpos[:] = Q[k]; mujoco.mj_forward(model, data)
        frames.append({"t": round(float(T[k]), 3), "p": np.round(data.xpos, 5).flatten().tolist(), "q": np.round(data.xquat, 5).flatten().tolist()})
    phases = [{"frame": int(a // stride), "label": l} for a, l in zip(PA, PL)]
    return frames, phases


def parts_db():
    """The parts database: assets/ffc/parts.json (researched) merged over the built-in entries for our own parts and the scene."""
    db = {"parts": {}, "stacks": {}, "wrist_extras": {}}
    p = ASSETS / "parts.json"
    if p.exists():
        db = json.loads(p.read_text())
    own = {
        "adapter_plate": dict(name="Adapter plate (ours)", vendor="Synphony, machined or printed", what_it_is="Plate that carries the robot flange bolt pattern on one face and the force sensor's pattern on the other.", buy_url="", mass_g=40),
        "tool_plate": dict(name="Sensor tool-side plate (ours)", vendor="Synphony", what_it_is="Plate on the sensor's tool side that the 90 degree bracket bolts to.", buy_url="", mass_g=15),
        "bracket_90": dict(name="90 degree bracket (ours)", vendor="Synphony, 6 mm 6061", what_it_is="L bracket: horizontal leg under the sensor plate, vertical leg down the east side carrying the gripper's stock side plate, so the gripper lies horizontal with its stroke vertical.", buy_url="", mass_g=55),
        "finger_lower": dict(name="Lower finger (ours, printed PAHT-CF)", vendor="Synphony", what_it_is="Plate on the lower jaw's outer face (M4 + two 1.5 mm pins), one leg down the east side of the cable to the steel shoe.", buy_url="", mass_g=12),
        "finger_upper": dict(name="Upper finger (ours, printed PAHT-CF)", vendor="Synphony", what_it_is="Plate on the upper jaw's outer face, one leg down to the 46 mm pad that presses the cable onto the shoe.", buy_url="", mass_g=12),
        "shoe_steel": dict(name="Shoe and lip, 0.5 mm shim steel (ours)", vendor="OSH Cut or hand cut", what_it_is="The thin steel plate the cable rides on, with a 1.5 mm lip that hooks the latch. Bonded under the lower finger.", buy_url="https://oshcut.com", mass_g=4),
        "pad_rubber": dict(name="Pad face, 1.5 mm silicone (ours)", vendor="McMaster-Carr", what_it_is="Compliant face on the upper finger that pinches the cable at about 5 N.", buy_url="https://www.mcmaster.com", mass_g=2),
        "d405": dict(name="Intel RealSense D405", vendor="Intel", what_it_is="Short-range depth camera that finds the connector from above; never used for the last millimetre.", buy_url="https://www.intelrealsense.com/depth-camera-d405/", mass_g=60),
        "arm_meca500": dict(name="Mecademic Meca500 R4", vendor="Mecademic", what_it_is="Six-axis 0.5 kg payload arm with 5 um repeatability; the working arm of the cell.", buy_url="https://www.mecademic.com/product/meca500-six-axis-robot-arm/"),
        "arm_lrmate200id": dict(name="FANUC LR Mate 200iD", vendor="FANUC", what_it_is="Six-axis 7 kg payload arm on an R-30iB Mate Plus controller; the alternative arm.", buy_url="https://www.fanucamerica.com/products/robots/series/lr-mate/lr-mate-200id"),
        "connector": dict(name="Keyboard FFC connector CON3104 (0.8 mm pitch flip-latch ZIF)", vendor="on the FX505 mainboard", what_it_is="Horizontal ZIF: the cable slides 3.5 mm in, the white actuator bar at the back flips up to open and presses down to clamp.", buy_url=""),
        "cable": dict(name="Keyboard flat flex cable, 28 mm wide, 0.3 mm stiffened tip", vendor="ASUS TUF FX505 keyboard", what_it_is="The part being inserted: a 38 mm flat flex with a 13 mm stiffener, rooted at the keyboard deck slot.", buy_url=""),
        "laptop": dict(name="ASUS TUF FX505, upside down on the pallet", vendor="ASUS", what_it_is="The workpiece. Mainboard up, SO-DIMM 25 mm behind the connector, mid rail 40 mm behind.", buy_url=""),
        "platform_fixture": dict(name="Case 2 fixture: plate, rounded rest, feed carriage (ours)", vendor="Synphony, printed", what_it_is="Lakshya's platform: the connector on a plate, the cable held level in a feed that pays out, over a 12 mm rest.", buy_url=""),
        "pallet": dict(name="Pallet with couplings and fiducial (ours)", vendor="Synphony", what_it_is="Lifts out of the bench; carries the laptop or the platform.", buy_url=""),
        "bench": dict(name="Breadboard bench", vendor="Thorlabs class", what_it_is="Optical breadboard with the arm plate and camera post.", buy_url=""),
        "camera_stands": dict(name="Camera and light stands (Thorlabs 12.7 mm posts, clamps and a printed tilt block)", vendor="Thorlabs + Synphony", what_it_is="Three posts on the breadboard carry the two macro cameras at 160 mm from the mouth and the blue bar light at 60 mm; azimuth and elevation set once per fixture and left alone, so the policy sees a still scene.", buy_url="https://www.thorlabs.com/navigation.cfm?guide_id=47"),
        "camera_arms": dict(name="Camera arms (ours, 6 x 10 mm 6061 bars)", vendor="Synphony", what_it_is="Two outrigger arms over the bracket leg carrying the macro cameras at the lens working distance, 35 degrees down each side; a rear arm under the leg for the D405; a short drop for the light.", buy_url="", mass_g=40),
    }
    vp = ASSETS / "vision_parts.json"
    if vp.exists():
        V = json.loads(vp.read_text())
        for sec in ("cameras", "lenses", "lights", "cables"):
            for k, x in V.get(sec, {}).items():
                o = x.get("outline_mm") or {}
                shape = (f"{o.get('w')} x {o.get('h')} x {o.get('d')} mm" if "w" in o else (f"{o.get('dia')} mm dia x {o.get('length')} mm" if "dia" in o else "")) + (f" ({o['note']})" if o.get("note") else "")
                what = x.get("what_it_is") or x.get("note") or ""
                if sec == "lenses":
                    what = f"{x.get('focal_mm')} mm lens: at {x.get('working_distance_mm')} mm working distance it sees {x.get('fov_mm', {}).get('w')} x {x.get('fov_mm', {}).get('h')} mm at {x.get('res_um_px')} um per pixel; depth of field {x.get('dof_mm_f4')} mm at f/4, {x.get('dof_mm_f8')} mm at f/8. " + what
                elif sec == "cameras":
                    what = f"{x.get('pixels', {}).get('w')} x {x.get('pixels', {}).get('h')} pixels of {x.get('pixel_um')} um, {x.get('fps')} fps, {x.get('interface')}. " + what
                db["parts"][k] = {"name": x.get("name"), "vendor": x.get("vendor"), "part_number": x.get("part_number"), "what_it_is": what, "shape": shape, "mass_g": x.get("mass_g"),
                                  "base_interface": (str(x.get("mount_holes"))[:160] if x.get("mount_holes") else x.get("lens_mount", "")), "tool_interface": x.get("lens_mount", ""), "buy_url": x.get("buy_url", ""), "price_usd_if_listed": x.get("price_usd"), "notes": x.get("note", ""), "source_url": x.get("source_url", "")}
    for k, v in own.items():
        db["parts"].setdefault(k, {}).update({kk: vv for kk, vv in v.items() if kk not in db["parts"].get(k, {})})
    return db
