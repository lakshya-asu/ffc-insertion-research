"""Blueprint page of the wrist stacks: every variant (arm x tool stack x case) as one interactive, to-scale three.js scene with the
recorded run, hover cards per part (name, what it is, where to buy), an exploded view of the stack, a 2D dimensioned elevation of the
stack, and a connection check (each part's mounting pattern against its neighbour's, and the geometry: no gaps, no overlaps).
usage: .venv/bin/python scripts/ffc_zif_blueprint.py [out.html]      (reads out/ffc_zif/<variant>/traj.npz for each variant)"""
import json, math, sys
from pathlib import Path
import mujoco, numpy as np
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from cell_twin import ffc_zif as F, ffc_scene as S
from cell_twin import ffc_render_kit as K

OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "out" / "ffc_zif" / "blueprint.html"
VARIANTS = [("meca500", "megp_nano25", "laptop"), ("meca500", "megp_nano25", "platform"), ("meca500", "megp_nano25_thin", "laptop"), ("meca500", "megp_nano25_zaber", "laptop"),
            ("lrmate200id", "lehz16_mini40", "laptop"), ("lrmate200id", "lehz16_mini40", "platform"),
            ("meca500", "servo_v1", "laptop"), ("lrmate200id", "servo_v1_lr", "laptop")]
DB = S.parts_db()
REVIEW = json.loads((ROOT / "out" / "ffc_zif" / "review.json").read_text()) if (ROOT / "out" / "ffc_zif" / "review.json").exists() else {}
COLORS = {"arm": "#8a8a90", "wrist stack": "#0e8a9b", "bracket": "#9aa0a8", "camera arms": "#b8bec6", "gripper": "#2b2b2e", "fingers": "#ee8c32", "shoe": "#b9bdc4", "pad": "#444449",
          "cameras": "#111111", "connector": "#2a2a2e", "cable": "#2563eb", "laptop": "#1f4fa8", "pallet": "#e09a3c", "bench": "#3b3b40", "fixture": "#d9930d", "other": "#999999"}


def layer_of(part):
    if part.startswith("arm_"): return "arm"
    if part in ("meca500_flange", "lrmate_flange", "adapter_plate_meca_nano25", "adapter_plate_lrmate_mini40", "nano17", "mini40", "zaber_xna08a25"): return "wrist stack"
    if part.startswith("bracket"): return "bracket"
    if part in ("megp25e", "zimmer_gep2006il", "smc_lehz16l") or "_jaw_" in part: return "gripper"
    if part.startswith("finger_printed"): return "fingers"
    if part in ("shoe_steel", "finger_shoe_0p3"): return "shoe"
    if part == "pad_rubber": return "pad"
    if part in ("d405",) or part.startswith("macro_cam"): return "cameras"
    if part == "camera_arms": return "camera arms"
    if part == "platform_fixture": return "fixture"
    return part if part in COLORS else "other"


import re
def iface_key(s):
    """What matters for a mate: the thread sizes named and the circle or spacing they sit on."""
    s = (s or "").lower()
    threads = set(re.findall(r"\bm(\d+(?:\.\d+)?)", s)) | set(re.findall(r"(\d/\d+-\d+)", s))
    pcd = set(re.findall(r"(\d+(?:\.\d+)?)\s*(?:mm\s*)?(?:pcd|spacing|apart|circle)", s)) | set(re.findall(r"on\s+(\d+(?:\.\d+)?)\s*mm", s))
    return threads, pcd


def iface_match(a, b):
    ta, pa = iface_key(a); tb, pb = iface_key(b)
    if not ta and not tb:
        return None                                   # nothing to compare (free surfaces, "design choice")
    if ta and tb and not (ta & tb):
        return False
    if pa and pb and not (pa & pb):
        return False
    return True


def connection_check(arm, tool, model):
    """Mechanical chain: pattern on each part's tool side against the next part's base; geometry: the stack parts touch along the flange axis."""
    key = {"gep_mini40": "lrmate200id/smallgrip_mini40", "lehz16_mini40": "lrmate200id/lehz16_mini40"}.get(tool, f"{arm}/{tool}"); chain = DB.get("stacks", {}).get(key) or []
    rows = []
    for a, b in zip(chain, chain[1:]):
        pa, pb = DB["parts"].get(a, {}), DB["parts"].get(b, {})
        ti, bi = pa.get("tool_interface", ""), pb.get("base_interface", "")
        m = iface_match(ti, bi)
        rows.append({"from": pa.get("name", a), "to": pb.get("name", b), "via": f"{ti or '?'}  to  {bi or '?'}", "bolts": pb.get("bolts", ""), "ok": bool(m is not False), "note": "" if m is not None else "no bolt pattern to compare (free surface or design choice)", "kind": "pattern"})
    # geometry along the flange axis: consecutive stack geoms, then plate under the bracket, bracket leg beside the gripper plate, plate on the body, jaws on the finger plates
    gz = {}
    for i in range(model.ngeom):
        n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
        if n and n.startswith("stack_"):
            gz[n[6:]] = (float(model.geom_pos[i][2] - model.geom_size[i][2 if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_BOX else 1]), float(model.geom_pos[i][2] + model.geom_size[i][2 if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_BOX else 1]))
    order = [(pid, part) for pid, part, *_ in F.STACKS[F.TOOL_INFO[tool]["stack"]] if pid != "flange"]
    prev_top = 0.0; prev = "robot flange face"
    for pid, part in order:
        z0, z1 = gz[pid]; gap = (z0 - prev_top) * 1e3
        rows.append({"from": prev, "to": DB["parts"].get(part, {}).get("name", part), "via": f"face to face along the flange axis, gap {gap:+.2f} mm", "bolts": DB["parts"].get(part, {}).get("bolts", ""), "ok": bool(abs(gap) < 0.01), "kind": "geometry"})
        prev_top = z1; prev = DB["parts"].get(part, {}).get("name", part)
    bh = model.geom("bracket_h"); gap = (model.geom_pos[bh.id][2] - model.geom_size[bh.id][2] - prev_top) * 1e3
    G = F.GRIPPERS[F.TOOL_INFO[tool]["gripper"]]; bv = model.geom("bracket_v"); gb = model.geom("gripper_body")
    rows.append({"from": prev, "to": "cell A, fixed end (bar load cell)" if G.get("cells") else "90 degree bracket, horizontal leg", "via": f"face to face, gap {gap:+.2f} mm", "bolts": "2 x M4 through the cell's fixed end" if G.get("cells") else "4 x M3", "ok": bool(abs(gap) < 0.01), "kind": "geometry"})
    if G.get("cells"):
        cb, cc, ct = model.geom("cell_b"), model.geom("cell_corner"), model.geom("cell_tab")
        for frm, to, g_from, g_to, bolts in (("cell A, free end", "corner plate (on top)", bh, cc, "2 x M5"), ("corner plate", "cell B, fixed end", cc, cb, "2 x M4"), ("cell B, free end", "tab (on top, one piece with the side plate)", cb, ct, "2 x M5")):
            gap = ((model.geom_pos[g_to.id][2] + model.geom_size[g_to.id][2]) - (model.geom_pos[g_from.id][2] - model.geom_size[g_from.id][2])) * 1e3 if g_to in (cc, ct) else ((model.geom_pos[g_to.id][2] - model.geom_size[g_to.id][2]) - (model.geom_pos[g_from.id][2] + model.geom_size[g_from.id][2])) * 1e3
            rows.append({"from": frm, "to": to, "via": f"face to face along the flange axis, gap {gap:+.2f} mm", "bolts": bolts, "ok": bool(abs(gap) < 0.01), "kind": "geometry"})
        gap = ((model.geom_pos[bv.id][1] - model.geom_size[bv.id][1]) - (F.FINGER_OFF + G["body_d"] / 2)) * 1e3
        rows.append({"from": "side plate", "to": "printed body, east face", "via": f"face to face across, gap {gap:+.2f} mm", "bolts": "4 x M3 into heat-set inserts", "ok": bool(abs(gap) < 0.01), "kind": "geometry"})
    elif G["mount"] == "side" and G["plate_t"] == 0:
        gap = ((model.geom_pos[bv.id][1] - model.geom_size[bv.id][1]) - (F.FINGER_OFF + G["body_d"] / 2)) * 1e3
        rows.append({"from": "90 degree bracket, vertical leg", "to": "gripper body, side mounting pattern", "via": f"face to face across, gap {gap:+.2f} mm; flange axis {G['plate_from_end']*1e3:.0f} mm behind the jaw end (ESTIMATE)", "bolts": DB["parts"].get(G["ids"]["part"], {}).get("bolts", ""), "ok": bool(abs(gap) < 0.01), "kind": "geometry"})
    elif G["mount"] == "side":
        gp = model.geom("gripper_plate")
        gap = ((model.geom_pos[bv.id][1] - model.geom_size[bv.id][1]) - (model.geom_pos[gp.id][1] + model.geom_size[gp.id][1])) * 1e3
        rows.append({"from": "90 degree bracket, vertical leg", "to": "gripper's stock side plate", "via": f"face to face across, gap {gap:+.2f} mm; flange axis through the plate centre, {G['plate_from_end']*1e3:.0f} mm behind the jaw end", "bolts": "4 x M3", "ok": bool(abs(gap) < 0.01), "kind": "geometry"})
        gap = ((model.geom_pos[gp.id][1] - model.geom_size[gp.id][1]) - (F.FINGER_OFF + G["body_d"] / 2)) * 1e3
        rows.append({"from": "gripper's stock side plate", "to": "gripper body (big side face)", "via": f"gap {gap:+.2f} mm", "bolts": "2 x M2.5 (stock)", "ok": bool(abs(gap) < 0.01), "kind": "geometry"})
    else:
        px = float(model.body_pos[model.body("jaws").id][0])
        x_back = px - (float(model.geom_pos[gb.id][2]) - float(model.geom_size[gb.id][2]))          # body's back end face, tool x
        x_front = float(model.geom_pos[bv.id][0]) - float(model.geom_size[bv.id][0])                # bracket leg's front face, tool x
        gap = (x_back - x_front) * 1e3
        rows.append({"from": "90 degree bracket, vertical leg", "to": "gripper body, back end face", "via": f"face to face along the body axis, gap {gap:+.2f} mm; the flange axis is {(-px)*1e3:.1f} mm behind the jaw end", "bolts": DB["parts"].get(G["ids"]["part"], {}).get("bolts", ""), "ok": bool(abs(gap) < 0.01), "kind": "geometry"})
    for jaw, plate, name in (("jaw_lo", "plate_lo", "lower finger"), ("jaw_up", "plate_up", "upper finger")):
        j, p = model.geom(jaw), model.geom(plate)
        # along the jaw x axis (closing direction): the plate sits on the jaw's outer face
        if jaw == "jaw_lo":
            gap = ((model.geom_pos[p.id][0] - model.geom_size[p.id][0]) - (model.geom_pos[j.id][0] + model.geom_size[j.id][0])) * 1e3
        else:
            gap = ((model.geom_pos[j.id][0] - model.geom_size[j.id][0]) - (model.geom_pos[p.id][0] + model.geom_size[p.id][0])) * 1e3
        rows.append({"from": f"{'lower' if jaw == 'jaw_lo' else 'upper'} jaw, outer face", "to": name + " plate", "via": f"gap {gap:+.2f} mm", "bolts": "M4 x 0.7 + two 1.5 mm H7 pins", "ok": bool(abs(gap) < 0.01), "kind": "geometry"})
    return rows


def elevation_svg(arm, tool, model):
    """2D side elevation of the stack (looking west, along the cable width): flange at the top, parts stacked down, the gripper lying horizontal,
    fingers to the shoe plane. 1 mm = 4 px. Dimensions in mm."""
    S_ = 4.0; W, H = 820, 620; OX, OY = 360.0, 40.0
    X = lambda x: OX + x * S_; Y = lambda z: OY + z * S_      # x: tool x (backward +), z: down the flange axis
    ti = F.TOOL_INFO[tool]; G = F.GRIPPERS[ti["gripper"]]
    p = [f'<svg viewBox="0 0 {W} {H}" xmlns="http://www.w3.org/2000/svg" style="background:#FBF7F0;max-width:100%">']
    def box(x0, x1, z0, z1, fill, label=None, op=1.0):
        p.append(f'<rect x="{X(min(x0,x1)):.1f}" y="{Y(min(z0,z1)):.1f}" width="{abs(x1-x0)*S_:.1f}" height="{abs(z1-z0)*S_:.1f}" fill="{fill}" fill-opacity="{op}" stroke="#1E1B16" stroke-width="0.8"/>')
        if label: p.append(f'<text x="{X(max(x0,x1))+8:.1f}" y="{Y((z0+z1)/2)+4:.1f}" font-size="11" fill="#1E1B16" font-family="Inter,Helvetica,Arial,sans-serif">{label}</text>')
    def dim(z0, z1, x, label):
        p.append(f'<line x1="{X(x):.1f}" y1="{Y(z0):.1f}" x2="{X(x):.1f}" y2="{Y(z1):.1f}" stroke="#C0392B" stroke-width="1"/><line x1="{X(x)-4:.1f}" y1="{Y(z0):.1f}" x2="{X(x)+4:.1f}" y2="{Y(z0):.1f}" stroke="#C0392B"/><line x1="{X(x)-4:.1f}" y1="{Y(z1):.1f}" x2="{X(x)+4:.1f}" y2="{Y(z1):.1f}" stroke="#C0392B"/>'
                 f'<text x="{X(x)-6:.1f}" y="{Y((z0+z1)/2)+4:.1f}" font-size="11" fill="#C0392B" text-anchor="end" font-family="Inter,Helvetica,Arial,sans-serif">{label}</text>')
    p.append(f'<text x="14" y="22" font-size="13" font-weight="700" fill="#1E1B16" font-family="Inter,Helvetica,Arial,sans-serif">{F.ARM_INFO[arm]["prefix"]} · {ti["label"]}: side elevation of the stack, flange at the top, 1 mm = 4 px</text>')
    # flange line
    p.append(f'<line x1="{X(-30):.1f}" y1="{Y(0):.1f}" x2="{X(30):.1f}" y2="{Y(0):.1f}" stroke="#1E1B16" stroke-width="2"/><text x="{X(-30):.1f}" y="{Y(0)-6:.1f}" font-size="11" fill="#6B6257" font-family="Inter">robot flange</text>')
    z = 0.0; xdim = -32
    for pid, part, shape, dims, mass, col, xoff in F.STACKS[ti["stack"]]:
        h = dims[-1] * 1e3; w = dims[0] * 1e3; name = DB["parts"].get(part, {}).get("name", part)
        if pid == "flange":
            box(-w / 2, w / 2, -h, 0, "#777", f"{name} (the robot's own), {h:g} mm", 0.9); continue
        box(-w / 2 + xoff * 1e3, w / 2 + xoff * 1e3, z, z + h, "#%02x%02x%02x" % tuple(int(c * 255) for c in col[:3]), f"{name}, {h:g} mm", 0.9)
        dim(z, z + h, xdim, f"{h:g}"); z += h
    Ls = z; C = (G.get("jaw_lo_face", F.JAW_LO_FACE) + G["jaw_sep_open"] / 2 + G["jaw_t"]) * 1e3; bt = F.BRACKET_T * 1e3; pw = max(G["plate_wh"], G["body_w"]) * 1e3
    top_above_C = (max(G["plate_wh"], G["body_w"]) / 2 if G["mount"] == "end" else max(G["plate_wh"] / 2, 0.02895)) * 1e3
    zj = Ls + bt + 2 + top_above_C + C
    box(-15, 15, Ls, Ls + bt, "#9aa0a8", f"bracket, horizontal leg, {bt:g} mm")
    zp = zj - C; bw = G["body_w"] * 1e3; bl = (G["body_l"] + G["end_l"]) * 1e3
    if G["mount"] == "side":
        x_end = -(G["plate_from_end"] * 1e3)                       # jaw end ahead of the flange axis; stock side plate behind this view
    else:
        x_end = -(bl + bt / 2); box(-bt / 2, bt / 2, Ls + bt, zp + bw / 2, "#9aa0a8", "bracket, vertical leg: the gripper's back end face bolts to it")
    if G["mount"] == "side" and G["plate_t"] == 0: x_end = -(G["plate_from_end"] * 1e3)
    box(x_end, x_end + bl, zp - bw / 2, zp + bw / 2, "#2b2b2e", f"{G['label']} body, {bl:g} x {bw:g} (side view; it stands beside the cable, {F.FINGER_OFF*1e3:g} mm east)", 0.85)
    if G["mount"] == "side": box(x_end + 58, x_end + 70, zp - 28.95, zp - bw / 2, "#3a3a3e", "M8 connector, pointing up")
    box(x_end, x_end + G["end_l"] * 1e3, zp - bw / 2, zp + bw / 2, "#3a3a3e")
    jt = G["jaw_t"] * 1e3; sep = G["jaw_sep_open"] * 1e3
    box(x_end, x_end + G["end_l"] * 1e3, zp + sep / 2, zp + sep / 2 + jt, "#555a60", "lower jaw (moves up 1.5 mm to close)")
    box(x_end, x_end + G["end_l"] * 1e3, zp - sep / 2 - jt, zp - sep / 2, "#555a60", "upper jaw (moves down 1.5 mm)")
    box(x_end - 12, x_end + 20, 0, 0, "#000")  # placeholder no-op line
    # fingers: plates on the jaw outer faces, legs down to the shoe plane at zj
    fp = F.FPLATE_T * 1e3
    box(x_end - 13, x_end + G["end_l"] * 1e3, zp + sep / 2 + jt, zp + sep / 2 + jt + fp, "#ee8c32", "lower finger plate (M4 + 2 pins)")
    box(x_end - 13, x_end - 9, zp + sep / 2 + jt + fp, zj, "#ee8c32", "leg, east of the cable")
    box(x_end - 13, x_end + G["end_l"] * 1e3, zp - sep / 2 - jt - fp, zp - sep / 2 - jt, "#ee8c32", "upper finger plate")
    box(x_end - 12, x_end - 8, zp - sep / 2 - jt - fp, zj - F.PAD_OPEN * 1e3 - F.PAD_T_UP * 1e3, "#ee8c32")
    box(x_end - 16, x_end - 10, zj - F.PAD_OPEN * 1e3 - F.PAD_T_UP * 1e3, zj - F.PAD_OPEN * 1e3, "#ee8c32", "pad, 2 mm, 3 mm open")
    box(x_end - 17.5, x_end - 9, zj, zj + ti["shoe_t"] * 1e3, "#b9bdc4", f"shoe {ti['shoe_t']*1e3:g} mm, lip 1.5 ahead")
    p.append(f'<line x1="{X(-60):.1f}" y1="{Y(zj):.1f}" x2="{X(60):.1f}" y2="{Y(zj):.1f}" stroke="#2E5E3A" stroke-width="2" stroke-dasharray="6 4"/><text x="{X(-60):.1f}" y="{Y(zj)+16:.1f}" font-size="11" fill="#2E5E3A" font-family="Inter">board plane (shoe underside), {zj:.1f} mm below the flange</text>')
    dim(Ls, Ls + bt, xdim, f"{bt:g}"); dim(zp - pw / 2, zp + pw / 2, xdim, f"plate {pw:g}"); dim(0, zj, xdim - 28, f"{zj:.1f} flange to shoe")
    dim(zp - bw / 2, zp + bw / 2, xdim - 14, f"body {bw:g}")
    p.append(f'<line x1="{X(0):.1f}" y1="{Y(-8):.1f}" x2="{X(0):.1f}" y2="{Y(zj+8):.1f}" stroke="#6B6257" stroke-width="0.8" stroke-dasharray="3 3"/><text x="{X(0)+4:.1f}" y="{Y(zj+8):.1f}" font-size="10" fill="#6B6257" font-family="Inter">flange axis</text>')
    p.append('</svg>'); return "".join(p)


def walkthrough(arm, tool, case):
    """Numbered steps for explaining the stack to someone: each highlights parts, looks at them, and says what they are for; the last steps replay the task."""
    ti = F.TOOL_INFO[tool]; G = F.GRIPPERS[ti["gripper"]]; ids = G["ids"]; stack = [(pid, part) for pid, part, *_ in F.STACKS[ti["stack"]]]
    nm = lambda k: DB["parts"].get(k, {}).get("name", k)
    sensor = next((part for pid, part in stack if pid in ("nano17", "nano25", "mini40")), None)
    if G.get("cells"):
        return walkthrough_servo(arm, tool, case, ti, G, ids, stack, nm)
    steps = [
        {"title": "1. The robot's flange", "parts": [stack[0][1]], "look": stack[0][1],
         "text": f"{nm(stack[0][1])}. Everything hangs from here. Its pattern: {DB['parts'].get(stack[0][1], {}).get('tool_interface', '')}."},
        {"title": "2. Adapter plate (we make it)", "parts": [stack[1][1]], "look": stack[1][1],
         "text": f"Turns the flange pattern into the sensor's pattern. {DB['parts'].get(stack[1][1], {}).get('notes', '')}"},
        {"title": "3. Force/torque sensor", "parts": [sensor], "look": sensor,
         "text": f"{nm(sensor)}. It is the cell's sense of touch: it feels the lid resisting in phase 1, the cable meeting the mouth and the seat in phase 2, and the press in phase 3. Every guarded move stops on its reading."},
    ]
    if len(stack) > 3:
        steps.append({"title": "4. Zaber fine stage (optional variant)", "parts": [stack[3][1]], "look": stack[3][1],
                      "text": f"{nm(stack[3][1])}: a 25 mm screw stage that could push the gripper along the insertion axis in 0.1 um steps while the arm holds still. Drawn in the stack; its bracket needs its own M3 pattern (the connection table flags that)."})
    steps += [
        {"title": f"{len(steps)+1}. 90 degree bracket (we make it)", "parts": [ids["bracket"]], "look": ids["bracket"],
         "text": "Its horizontal leg carries the sensor's tool-side pattern directly (screws counterbored from below), so there is no separate tool plate. The key mechanical idea: the cable must be pinched vertically (pad from above, steel shoe from below), so the gripper's stroke has to be vertical. " +
                 ("The MEGP mounts by a plate on its big side face, so the bracket's vertical leg carries that plate and the body lies horizontal, pointing at the connector, 20 mm east of the cable." if G["mount"] == "side"
                  else "The Zimmer mounts by its back end face, so the bracket's vertical leg sits behind the body and the body points at the connector, 20 mm east of the cable.")},
        {"title": f"{len(steps)+2}. The gripper", "parts": [ids["part"]], "look": ids["part"],
         "text": f"{nm(ids['part'])}. {DB['parts'].get(ids['part'], {}).get('what_it_is', '')} Stroke per jaw 3 mm; we use 1.5 mm per jaw. " +
                 ("The MEGP 25LS was rejected: its 66 mm jaw rail would reach below the board when the stroke is vertical." if ti["gripper"] == "megp25e" else "The Schunk EGK 25 (53 mm stroke, 620 g) and the Zimmer GEP2006 (40 N minimum force) were rejected; the LEHZ16 pinches at 3 to 8 N, open loop, so its force is bench-calibrated.")},
        {"title": f"{len(steps)+3}. Jaws and fingers", "parts": [ids["jaw_l"], ids["jaw_r"], ids["finger_l"], ids["finger_r"]], "look": ids["finger_l"],
         "text": f"Both jaws move toward the centre. Each finger is a printed plate bolted to a jaw's outer face ({DB['parts'].get(ids['jaw_l'], {}).get('tool_interface', '')}) with one leg down the east side of the cable, so after the cable is seated the shoe can slide out sideways; a leg on both sides would sweep through the cable."},
        {"title": f"{len(steps)+4}. Shoe, lip and pad", "parts": ["shoe_steel", "finger_shoe_0p3", "pad_rubber"], "look": "pad_rubber",
         "text": f"Under the lower finger: {ti['shoe_t']*1e3:g} mm shim steel, 20 x 7 mm, with a 1.5 mm lip flush with its underside that hooks the lid. On the upper finger: a 46 x 6 x 2 mm pad with a silicone face that pinches the cable at about 5 N. The cable rides on the shoe with 6 mm of its tip proud, flat at slot-floor height."},
        {"title": f"{len(steps)+5}. Cameras, lenses and light", "parts": ["d405", F.CAMERAS[arm]["macro"], F.CAMERAS[arm]["lens"], F.CAMERAS[arm]["light"], "camera_stands" if F.FIXED_CAMS else "camera_arms"], "look": F.CAMERAS[arm]["macro"],
         "text": (lambda cm, lens: (f"Two {nm(cm['macro'])} with {nm(cm['lens'])} on posts on the breadboard, {cm['wd']*1e3:.0f} mm from the mouth at {cm['el_deg']:.0f} degrees elevation, azimuths {cm['az_deg'][0]:.0f} and {cm['az_deg'][1]:.0f} degrees from east, a still view the arm cannot shake and the stack cannot block (checked frame by frame along the run): " if F.FIXED_CAMS else f"Two {nm(cm['macro'])} with {nm(cm['lens'])} on 6 x 10 mm arms off the bracket, {cm['wd']*1e3:.0f} mm from the cable tip at 35 degrees down from each side: ") + f"field {lens.get('fov_mm', {}).get('w')} x {lens.get('fov_mm', {}).get('h')} mm at {lens.get('res_um_px')} um per pixel, depth of field {lens.get('dof_mm_f4')} mm at f/4. {nm(cm['light'])} sits low on the west side, 55 mm from the mouth, as dark-field light. {nm('d405')} hangs behind the flange, 12 mm west of the cable line, and finds the connector from 95 mm; it is never used for the last millimetre. Below: what the twin's cameras see at the mouth.")(F.CAMERAS[arm], DB["parts"].get(F.CAMERAS[arm]["lens"], {})),
         "images": [str(p) for p in (ROOT / "out" / "ffc_zif" / f"{arm}-{tool}-{case}" / n for n in ("p2_at_mouth_macro_e.png", "p2_at_mouth_macro_w.png", "p2_at_mouth_wrist_d405.png")) if p.exists()]},
        {"title": f"{len(steps)+6}. Task, phase 1: open the latch", "parts": ["shoe_steel", "finger_shoe_0p3"], "phase": "phase 1: hook", "view": "fingers",
         "text": "Jaws open. The tool comes in level 3 mm over the closed lid and under the floating cable, drops to the board, hooks the lip 1 mm under the lid's front edge, and lifts the lid along its arc to the 100 degree stop while the wrist tilts with it (20 degrees at most). Then it steps 2.5 mm off the lid and leaves sideways east."},
        {"title": f"{len(steps)+7}. Task: grasp", "parts": ["pad_rubber"], "phase": "grasp", "view": "fingers",
         "text": "The shoe comes down beside the cable's tip and slides under it along the cable's own line, 0.45 mm below. Both jaws close 1.5 mm; the pad pinches the cable onto the shoe at 5 N with 6 mm of tip proud."},
        {"title": f"{len(steps)+8}. Task, phase 2: slide in", "parts": ["cable"], "phase": "phase 2: slide", "view": "connector",
         "text": "Tip to the mouth, flat on the slot-floor line, then 3.5 mm in at 2 mm/s under a 4 N force guard. The sensor feels the seat at 1 to 3 N."},
        {"title": f"{len(steps)+9}. Task: release", "parts": ["shoe_steel", "finger_shoe_0p3"], "phase": "release", "view": "fingers",
         "text": "The gripper opens while the arm lifts 1.5 mm (both jaws move, so the shoe would otherwise drop into the board). The shoe slides out sideways east from under the seated cable. The cable creeps 0.2 mm deeper and stays."},
        {"title": f"{len(steps)+10}. Task, phase 3: close the latch", "parts": ["shoe_steel", "finger_shoe_0p3"], "phase": "phase 3: push", "view": "connector",
         "text": "Level and un-turned, the shoe's rear edge pushes the standing lid forward from behind until it leans 25 degrees; then the shoe comes straight down and presses it flat at 10 N. Verified: latch closed, cable at depth, lateral within 0.2 mm."},
    ]
    import base64
    for st in steps:
        if st.get("images"):
            st["images"] = ["data:image/png;base64," + base64.b64encode(Path(pth).read_bytes()).decode() for pth in st["images"]]
    return steps


def walkthrough_servo(arm, tool, case, ti, G, ids, stack, nm):
    """the servo tool's own walk-through: flange, adapter, standoff, the two wrist cells, the printed gripper, jaws and fingers, the pinch cell, electronics, cameras, then the task"""
    steps = [
        {"title": "1. The robot's flange", "parts": [stack[0][1]], "look": stack[0][1],
         "text": f"{nm(stack[0][1])}. Everything hangs from here. Its pattern: {DB['parts'].get(stack[0][1], {}).get('tool_interface', '')}."},
        {"title": "2. Adapter plate (we make it)", "parts": [stack[1][1]], "look": stack[1][1],
         "text": f"4 mm 6061: the flange pattern on top, four M3 for the standoff below. {DB['parts'].get(stack[1][1], {}).get('notes', '')}"},
        {"title": "3. Printed standoff (we make it)", "parts": [stack[2][1]], "look": stack[2][1],
         "text": f"{nm(stack[2][1])}: a PAHT-CF spacer that puts the jaw line where the sensor stacks have it (22 mm on the Meca500, 34 mm on the LR Mate, where a shorter one put the wrist into the west bench camera). Cell A bolts to its underside with two M4."},
        {"title": "4. The two wrist load cells, our force sensor (we make the plates)", "parts": [ids["bracket"], "load_cell_tal220b"], "look": ids["bracket"],
         "text": "Two TAL220B bar load cells ($15.50 each, 55 x 12.7 x 12.7 mm) replace the ATI six-axis sensor and the 90 degree bracket at once. Cell A's fixed end is under the standoff; it runs east and bends along the insertion axis, so it reads the slide friction, the seat rise and the lid push. A 4 mm corner plate on its free end carries cell B, which runs along the body and bends vertically: the lid lift and the latch press. A tab and a 4 mm side plate carry the body. Every force at the fingertips passes through the two cells; the pinch is internal and cancels. Read at 1 kHz, about 0.07 N of noise; the ATI's 10 mN and its torques are what we give up."},
        {"title": "5. The printed gripper body: servo, belt, lead screw (we make it)", "parts": [ids["part"], "xl330_m077", "lead_screw_m4x1", "nut_peek_m4x1", "gt2_12t_pulleys_belt"], "look": ids["part"],
         "text": "A 55 x 22 x 26 mm PAHT-CF shell beside the cable. The Dynamixel XL330-M077-T ($27, 18 g, 4,096 steps per turn, 383 rpm, 0.215 N m) lies flat in a pocket at the back with its horn up; a 3 mm GT2 belt over the top (under a 6 mm cover) turns an M4 x 1 mm lead screw (McMaster 90500A160) 1:1, and the screw runs down through the body's top wall into the nut in the upper carrier. One encoder step is 0.24 um of jaw travel; the 4.6 degree lead angle is self-locking, so the pinch holds with the power off. Its own current is not used as a force signal."},
        {"title": "6. End block: slot, two carriers, rocker (we make it)", "parts": [ids["jaw_l"], ids["jaw_r"], "servo_rocker"], "look": ids["jaw_l"],
         "text": "Two printed carriers, 16 x 14 x 4 mm, run in the printed slot through the front of the body. The upper one holds the PEEK anti-backlash nut (McMaster 90500A310) on the screw, which guides it; a printed rocker pivoted on a 2 mm pin through the end block at the carriers' front-west corner rides in a notch in each carrier and moves the lower one the opposite way. So one screw moves both jaws 1.5 mm, the same motion the bought grippers make and the twin was validated with. The front 6 mm is open above and below the carriers so the fingers bolt to their exposed faces. The working 3D schematic with the interference check: FFC-servo-tool-3d.html."},
        {"title": "7. Jaws and fingers (same parts as the bought stacks)", "parts": [ids["finger_l"], ids["finger_r"]], "look": ids["finger_l"],
         "text": "Each carrier's outer face has the MEGP finger pattern (M4 + two 1.5 mm pins), so the fingers are the same printed plates: a bridge 4 mm over the cable and one leg down the east side, so the shoe can slide out sideways after the cable is seated."},
        {"title": "8. Cell P, the pinch sensor, and the shoe, lip and pad", "parts": ["load_cell_tal220b", "finger_shoe_0p3", "pad_rubber"], "look": "pad_rubber",
         "text": f"The upper finger's cross member is a third TAL220B: fixed end on the upper carrier, the pad on its free end, so it reads the pinch directly and the servo closes until it reads 5 N (the wrist cells cannot see the pinch, and an open-loop pinch would move about 5 N with the stiffener's 0.03 mm thickness tolerance). Under the lower finger: {ti['shoe_t']*1e3:g} mm shim steel, 20 x 7 mm, with the 1.5 mm lip that hooks the lid; the pad's silicone face pinches the cable at 5 N."},
        {"title": "9. Electronics and the one cable", "parts": ["adc_ads122c04", "mcu_qtpy_rp2040", "buck_d36v28f5", "tool_cable_cf211"], "look": ids["part"],
         "text": "On the body: two Adafruit ADS122C04 24-bit boards (1 kSPS per cell) and a QT Py RP2040 that reads them, closes the pinch loop, drives the servo's TTL bus and streams forces to the PC over USB; a Pololu 5 V 3.2 A regulator. One igus 4-core shielded cable along the arm: a 24 V pair (the LR Mate's EE connector, or a bench supply on the Meca500) and the USB data pair. The D405 keeps its own USB 3 cable."},
        {"title": "10. Cameras, lenses and light", "parts": ["d405", F.CAMERAS[arm]["macro"], F.CAMERAS[arm]["lens"], F.CAMERAS[arm]["light"], "camera_stands" if F.FIXED_CAMS else "camera_arms"], "look": F.CAMERAS[arm]["macro"],
         "text": f"Unchanged: two Basler dart + 16 mm bench cameras {F.CAMERAS[arm]['wd']*1e3:.0f} mm from the mouth at {F.CAMERAS[arm]['el_deg']:.0f} degrees, the blue bar light, and the D405 on its side plate (15 mm further forward here, 81 mm from the tips). The bench cameras set the insertion precision; the tool does not.",
         "images": [str(p) for p in (ROOT / "out" / "ffc_zif" / f"{arm}-{tool}-{case}" / n for n in ("p2_at_mouth_macro_e.png", "p2_at_mouth_macro_w.png", "p2_at_mouth_wrist_d405.png")) if p.exists()]},
        {"title": "11. Task, phase 1: open the latch", "parts": ["finger_shoe_0p3"], "phase": "phase 1: hook", "view": "fingers",
         "text": "Jaws open. The tool comes in level over the closed lid and under the floating cable, drops to the board, hooks the lip 1 mm under the lid's front edge and lifts the lid along its arc to the 100 degree stop while the wrist tilts with it. Cell B reads the 2 N lift."},
        {"title": "12. Task: grasp", "parts": ["pad_rubber", "load_cell_tal220b"], "phase": "grasp", "view": "fingers",
         "text": "The shoe slides 14 mm under the cable's stiffened tip, 0.45 mm below it. The servo closes both jaws at 6.4 mm/s until cell P reads 5 N; the screw locks."},
        {"title": "13. Task, phase 2: slide in", "parts": ["cable"], "phase": "phase 2: slide", "view": "connector",
         "text": "Tip to the mouth (the bench cameras check lateral and yaw), then 3.5 mm in at 2 mm/s. Cell A reads the 0.5 N friction and the 1 to 3 N seat rise; the guard stops at 4 N."},
        {"title": "14. Task: release", "parts": ["finger_shoe_0p3"], "phase": "release", "view": "fingers",
         "text": "The jaws open 1.5 mm each in 0.23 s while the arm lifts 1.5 mm (both jaws move, so the shoe would otherwise drop into the board). The shoe slides out east from under the seated cable."},
        {"title": "15. Task, phase 3: close the latch", "parts": ["finger_shoe_0p3"], "phase": "phase 3: push", "view": "connector",
         "text": "The shoe's rear edge pushes the standing lid forward (cell A: 7 N, guard 12 N), then presses it flat (cell B: 10 N, guard 10 N). Verified in the twin on both arms: latch closed, cable at depth, lateral within 0.2 mm."},
    ]
    import base64
    for st in steps:
        if st.get("images"):
            st["images"] = ["data:image/png;base64," + base64.b64encode(Path(pth).read_bytes()).decode() for pth in st["images"]]
    return steps


def order_list(arm, tool):
    ti = F.TOOL_INFO[tool]; G = F.GRIPPERS[ti["gripper"]]; ids = G["ids"]
    cm = F.CAMERAS[arm]
    chain = [part for pid, part, *_ in F.STACKS[ti["stack"]] if pid != "flange"] + ([ids["bracket"], "load_cell_tal220b", ids["part"], "xl330_m077", "lead_screw_m4x1", "nut_peek_m4x1", "gt2_12t_pulleys_belt", "servo_rocker", "adc_ads122c04", "mcu_qtpy_rp2040", "buck_d36v28f5", "tool_cable_cf211"] if G.get("cells") else [ids["bracket"], ids["part"]]) + [ids["finger_l"], ids["finger_r"], ("finger_shoe_0p3" if ti["shoe_t"] < 0.0004 else "shoe_steel"), "pad_rubber", "d405", cm["macro"], cm["lens"], cm["light"], "camera_stands" if F.FIXED_CAMS else "camera_arms", "usb3_high_flex_2m", "light_24v_cable"]
    rows = []
    for k in chain:
        p = DB["parts"].get(k, {}); ours = (not p.get("buy_url")) or p.get("buy_url") == "none" or "(ours)" in p.get("name", "")
        rows.append({"part": p.get("name", k), "pn": p.get("part_number", "") or "", "vendor": p.get("vendor", "") or "", "qty": (1 if F.FIXED_CAMS else 3) if k == "usb3_high_flex_2m" else (2 if k in (cm["macro"], cm["lens"]) else 1),
                     "price": p.get("price_usd_if_listed", "") or "", "url": "" if ours else (p.get("buy_url", "") or ""), "make": "we make it" if ours else "buy", "notes": (p.get("notes", "") or "")[:160]})
    return rows


meshes = {}; payloads = []
for arm, tool, case in VARIANTS:
    traj = ROOT / "out" / "ffc_zif" / f"{arm}-{tool}-{case}" / "traj.npz"
    if not traj.exists():
        print("skip (no run yet):", arm, tool, case); continue
    model, geoms, bodies = S.collect(arm, tool, case, meshes)
    _cells = bool(F.GRIPPERS[F.TOOL_INFO[tool]["gripper"]].get("cells"))
    for g_ in geoms:
        g_["mat"] = K.material_key(g_["name"], g_.get("layer", ""), arm, cells=_cells)
    fast = K.to_body_frames(F, K.fasteners_tool_frame(F, arm, tool, model), model)
    _bid = {"tool": int(model.body("tool").id), "jaws": int(model.body("jaws").id), "pad": int(model.body("pad").id), "shoe": int(model.body("shoe").id)}
    for f_ in fast:
        f_["body"] = _bid[f_["body"]]
    if np.load(traj)["qpos"].shape[1] != model.nq:
        print("skip (run predates the current model):", arm, tool, case); continue
    frames, phases = S.frames_of(model, traj, 240)
    for g in geoms:
        g["layer"] = layer_of(g["part"])
    rows = connection_check(arm, tool, model)
    payloads.append({"id": f"{arm}-{tool}-{case}", "label": f"{arm} · {F.TOOL_INFO[tool]['label']} · {F.CASES[case]}", "arm": arm, "tool": tool, "case": case,
                     "nbody": int(model.nbody), "geoms": geoms, "frames": frames, "phases": phases, "origin": F.connector_origin(arm).tolist(),
                     "connections": rows, "elevation": elevation_svg(arm, tool, model), "steps": walkthrough(arm, tool, case), "order": order_list(arm, tool), "review": REVIEW.get(f"{arm}/{tool}", {}), "fasteners": fast, "stack_order": [part for pid, part, *_ in F.STACKS[F.TOOL_INFO[tool]["stack"]]] + [F.GRIPPERS[F.TOOL_INFO[tool]["gripper"]]["ids"][k] for k in ("bracket", "part", "jaw_l", "finger_l", "jaw_r", "finger_r")] + [("finger_shoe_0p3" if F.TOOL_INFO[tool]["shoe_t"] < 0.0004 else "shoe_steel"), "pad_rubber", "d405", F.CAMERAS[arm]["macro"]]})
    print("collected", arm, tool, case, len(geoms), "geoms", len(frames), "frames", sum(1 for r in rows if not r["ok"]), "connection problems")
parts = {k: {kk: vv for kk, vv in v.items() if kk in ("name", "vendor", "part_number", "what_it_is", "shape", "mass_g", "base_interface", "tool_interface", "bolts", "cable_power", "buy_url", "price_usd_if_listed", "notes", "source_url")} for k, v in DB["parts"].items()}
js = json.dumps({"variants": payloads, "meshes": meshes, "parts": parts, "colors": COLORS}, separators=(",", ":"))
html = r"""<!doctype html><html><head><meta charset="utf-8"><title>FFC wrist stacks, blueprint</title>
<style>body{margin:0;background:#FBF7F0;font:14px -apple-system,Inter,Segoe UI,sans-serif;color:#1E1B16;overflow:hidden}
#ui{position:absolute;left:12px;top:12px;width:320px;background:rgba(251,247,240,.96);border-radius:12px;padding:12px 14px;box-shadow:0 2px 12px rgba(0,0,0,.15);max-height:94vh;overflow:auto}
#ui h1{font-size:15px;margin:0 0 6px}.sub{color:#6B6257;font-size:12px;margin:0 0 8px}select{width:100%;font:13px inherit;padding:5px;border-radius:8px;border:1px solid #d8d2c6;background:#fff}
.row{display:flex;align-items:center;gap:8px;margin:3px 0;font-size:13px}.sw{width:12px;height:12px;border-radius:3px;display:inline-block}
button{border:0;background:#1E1B16;color:#fff;border-radius:8px;padding:5px 10px;cursor:pointer;font:13px inherit}button.off{background:#8a8a90}
#bar{position:absolute;left:350px;right:12px;bottom:12px;background:rgba(251,247,240,.96);border-radius:12px;padding:10px 14px;box-shadow:0 2px 12px rgba(0,0,0,.15)}
#bar input[type=range]{width:100%}#phase{font-weight:600}
#views{position:absolute;right:12px;top:12px;background:rgba(251,247,240,.92);border-radius:10px;padding:8px 12px;font-size:12px;color:#6B6257;display:flex;gap:6px;align-items:center;flex-wrap:wrap;max-width:60vw;justify-content:flex-end}
#card{position:absolute;display:none;max-width:340px;background:#1E1B16;color:#fff;border-radius:10px;padding:10px 12px;font-size:12.5px;pointer-events:none;box-shadow:0 4px 16px rgba(0,0,0,.3);z-index:5}
#card b{font-size:13.5px}#card .m{color:#cfcbc2;margin:3px 0}#card a{color:#8fd3ff}
#sheet{position:absolute;left:350px;right:12px;top:60px;bottom:90px;background:rgba(251,247,240,.98);border-radius:12px;padding:14px 18px;overflow:auto;display:none;box-shadow:0 2px 12px rgba(0,0,0,.15)}
table.c{border-collapse:collapse;font-size:12.5px;width:100%}table.c td,table.c th{padding:5px 8px;border-bottom:1px solid #e7e1d6;text-align:left;vertical-align:top}.ok{color:#1E8E5A;font-weight:600}.bad{color:#C0392B;font-weight:600}
#parts{font-size:12.5px}#parts div{padding:4px 0;border-bottom:1px solid #e7e1d6;cursor:pointer}#parts div:hover{background:#f3ede3}
</style></head><body>
<div id="ui"><h1>Wrist stacks, flange to fingertip</h1><p class="sub">To scale. Drag to orbit, scroll to zoom, right-drag to pan. Hover any part for its card; click to open where to buy it.</p>
<select id="variant"></select>
<div class="row" style="margin-top:8px"><button id="walk">walk me through it</button><button id="explode">explode the stack</button><button id="sheetbtn">drawing, connections, order list</button></div>
<div id="layers" style="margin-top:8px"></div>
<p class="sub" style="margin-top:10px">Parts in this stack, flange down (click to look at one):</p><div id="parts"></div></div>
<div id="views">views: <button onclick="window.look('fingers')">fingers</button> <button onclick="window.look('stack')">stack</button> <button onclick="window.look('connector')">connector</button> <button onclick="window.look('arm')">whole arm</button></div>
<div id="card"></div>
<div id="tour" style="position:absolute;right:12px;top:60px;width:360px;background:rgba(251,247,240,.97);border-radius:12px;padding:12px 14px;box-shadow:0 2px 12px rgba(0,0,0,.15);display:none;font-size:13px"><div id="tourt" style="font-weight:700;font-size:14px;margin-bottom:6px"></div><div id="tourx" style="line-height:1.45"></div><div class="row" style="margin-top:10px"><button id="tprev">back</button><button id="tnext">next</button><span id="tn" style="color:#6B6257"></span><button id="tclose" class="off" style="margin-left:auto">close</button></div></div>
<div id="sheet"></div>
<div id="bar"><div class="row"><button id="play">play</button><span id="phase"></span><span id="time" style="color:#6B6257"></span></div><input type="range" id="frame" min="0" value="0"></div>
<script type="importmap">{"imports":{"three":"https://unpkg.com/three@0.160.0/build/three.module.js","three/addons/":"https://unpkg.com/three@0.160.0/examples/jsm/"}}</script>
<script type="module">
import * as THREE from 'three'; import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
const D = __DATA__; const P = D.parts;
__MATJS__
const scene = new THREE.Scene(); scene.background = new THREE.Color(0xFBF7F0);
const cam = new THREE.PerspectiveCamera(35, innerWidth/innerHeight, 0.001, 10); cam.up.set(0,0,1);
const renderer = new THREE.WebGLRenderer({antialias:true}); renderer.setSize(innerWidth, innerHeight); renderer.setPixelRatio(devicePixelRatio); document.body.appendChild(renderer.domElement);
const controls = new OrbitControls(cam, renderer.domElement); controls.enableDamping = true; controls.zoomSpeed = 5; controls.zoomToCursor = true; controls.minDistance = 0.004; controls.maxDistance = 4; controls.rotateSpeed = 0.9;
await setupLook(renderer, scene, 0.9, [D.variants[0].origin[0], D.variants[0].origin[1], D.variants[0].origin[2] + 0.15]);
const meshGeo = {}; for(const [n,m] of Object.entries(D.meshes)){ const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.Float32BufferAttribute(m.v,3)); g.setIndex(m.f); g.computeVertexNormals(); meshGeo[n]=g; }
let V = null, objs = [], groups = {}, root = new THREE.Group(), gridG = null, exploded = false; scene.add(root);
const sel = document.getElementById('variant'); for(const v of D.variants){ const o = document.createElement('option'); o.value = v.id; o.textContent = v.label; sel.appendChild(o); }
function grid(size, step, color, z, cx, cy){const g = new THREE.Group(); const m = new THREE.LineBasicMaterial({color, transparent:true, opacity:0.3}); const n = Math.round(size/step);
 for(let i=-n;i<=n;i++){const a=i*step; const p1=new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(cx-size,cy+a,z),new THREE.Vector3(cx+size,cy+a,z)]); const p2=new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(cx+a,cy-size,z),new THREE.Vector3(cx+a,cy+size,z)]); g.add(new THREE.Line(p1,m)); g.add(new THREE.Line(p2,m));} return g; }
const order = ['arm','wrist stack','bracket','gripper','fasteners','fingers','shoe','pad','cameras','camera arms','connector','cable','laptop','fixture','pallet','bench','other'];
function load(id){
 V = D.variants.find(v => v.id === id); root.clear(); objs = []; groups = {}; if(gridG) scene.remove(gridG);
 const o = V.origin; gridG = new THREE.Group(); gridG.add(grid(0.15, 0.01, 0x6B6257, o[2]+0.0002, o[0], o[1])); gridG.add(grid(0.02, 0.001, 0x6B6257, o[2]+0.0003, o[0], o[1])); scene.add(gridG);
 for(const g of V.geoms){
  let geo; if(g.type==='box') geo = new THREE.BoxGeometry(2*g.size[0],2*g.size[1],2*g.size[2]);
  else if(g.type==='cylinder'){ geo = new THREE.CylinderGeometry(g.size[0], g.size[0], 2*g.size[1], 40); geo.rotateX(Math.PI/2); }
  else geo = meshGeo[g.mesh];
  const col = g.color || D.colors[g.layer] || '#999';
  const physics = !g.detail && (g.part==='connector' || g.part==='cable');
  const mat = makeMaterial(g.mat, physics ? {opacity: 0.25} : {});
  const mesh = new THREE.Mesh(geo, mat); mesh.userData = g; mesh.userData.baseColor = mat.color.clone(); mesh.castShadow = !physics; mesh.receiveShadow = true; if(physics) mesh.userData.nohover = true;
  if(!groups[g.layer]){ groups[g.layer] = new THREE.Group(); root.add(groups[g.layer]); } groups[g.layer].add(mesh); objs.push(mesh);
 }
 for(const f of V.fasteners){ const grp = fastenerMesh(f, 1); P[f.title] = P[f.title] || {name: f.title, what_it_is: 'fastener: ' + f.kind + ' M' + f.d}; grp.userData = {name: 'fastener', body: f.body, pos: f.pos, quat: [1,0,0,0], part: f.title, layer: 'fasteners', info: grp.userData.info};
  if(!groups['fasteners']){ groups['fasteners'] = new THREE.Group(); root.add(groups['fasteners']); } groups['fasteners'].add(grp); objs.push(grp); }
 const lay = document.getElementById('layers'); lay.innerHTML = '';
 for(const n of order){ if(!groups[n]) continue; const row = document.createElement('div'); row.className='row'; const cb = document.createElement('input'); cb.type='checkbox'; cb.checked = true; cb.onchange = () => groups[n].visible = cb.checked;
  const sw = document.createElement('span'); sw.className='sw'; sw.style.background = D.colors[n] || '#999'; const lab = document.createElement('span'); lab.textContent = n; row.append(cb, sw, lab); lay.appendChild(row); }
 const pl = document.getElementById('parts'); pl.innerHTML = '';
 for(const pid of V.stack_order){ const p = P[pid] || {name: pid}; const d = document.createElement('div'); d.innerHTML = `<b>${p.name || pid}</b><br><span style="color:#6B6257">${p.what_it_is || ''}</span>`; d.onclick = () => lookAtPart(pid); pl.appendChild(d); }
 document.getElementById('sheet').innerHTML = `<h2 style="margin:0 0 6px;font-size:16px">${V.label}</h2><p class="sub">Side elevation of the stack and the connection check: every mating pattern and every face-to-face joint in the model.</p>${V.elevation}
  <h3 style="font-size:14px;margin:14px 0 6px">Connection check</h3><table class="c"><tr><th>from</th><th>to</th><th>how</th><th>fasteners</th><th>check</th></tr>${V.connections.map(r => `<tr><td>${r.from}</td><td>${r.to}</td><td>${r.via}${r.note?` <span style="color:#6B6257">(${r.note})</span>`:''}</td><td>${r.bolts||''}</td><td class="${r.ok?'ok':'bad'}">${r.ok?'OK':'CHECK'}</td></tr>`).join('')}</table>
  ${V.review && V.review.rows ? `<h3 style="font-size:14px;margin:18px 0 6px">Design review: ${V.review.mass_g} g below the flange, centre of mass ${V.review.com_mm[2]} mm down and ${Math.hypot(V.review.com_mm[0], V.review.com_mm[1]).toFixed(0)} mm off axis</h3><table class="c"><tr><th>area</th><th>item</th><th>value</th><th>limit</th><th>status</th><th>note</th></tr>${V.review.rows.map(r => `<tr><td>${r.area}</td><td>${r.item}</td><td>${r.value}</td><td>${r.limit}</td><td class="${r.status==='OK'?'ok':(r.status==='CHECK'?'bad':'')}">${r.status}</td><td style="color:#6B6257">${r.note||''}</td></tr>`).join('')}</table>` : ''}
  <h3 style="font-size:14px;margin:18px 0 6px">What to order for this stack (flange down), and what we make</h3><table class="c"><tr><th>part</th><th>vendor, part number</th><th>qty</th><th>price</th><th>buy / make</th></tr>${V.order.map(r => `<tr><td>${r.part}<div style="color:#6B6257;font-size:11.5px">${r.notes}</div></td><td>${[r.vendor, r.pn].filter(Boolean).join(', ')}</td><td>${r.qty}</td><td>${r.price?('$'+r.price):''}</td><td>${r.url?`<a href="${r.url}" target="_blank">buy</a>`:'<b>we make it</b>'}</td></tr>`).join('')}</table>`;
 slider.max = V.frames.length-1; slider.value = 0; setFrame(0); if(exploded) applyExplode();
}
const B = () => V.nbody; const tmpQ = new THREE.Quaternion(), tmpP = new THREE.Vector3(), lq = new THREE.Quaternion(), lp = new THREE.Vector3();
function setFrame(k){ const f = V.frames[k]; for(const m of objs){ const b = m.userData.body; tmpP.set(f.p[3*b], f.p[3*b+1], f.p[3*b+2]); tmpQ.set(f.q[4*b+1], f.q[4*b+2], f.q[4*b+3], f.q[4*b]);
 lq.set(m.userData.quat[1], m.userData.quat[2], m.userData.quat[3], m.userData.quat[0]); const off = m.userData.explode || [0,0,0]; lp.set(m.userData.pos[0]+off[0], m.userData.pos[1]+off[1], m.userData.pos[2]+off[2]).applyQuaternion(tmpQ).add(tmpP);
 m.position.copy(lp); m.quaternion.copy(tmpQ).multiply(lq); }
 let ph = ''; for(const p of V.phases){ if(k >= p.frame) ph = p.label; } document.getElementById('phase').textContent = ph; document.getElementById('time').textContent = '  t = ' + f.t.toFixed(2) + ' s'; }
const slider = document.getElementById('frame'); slider.oninput = () => setFrame(+slider.value);
let playing = false; document.getElementById('play').onclick = () => { playing = !playing; document.getElementById('play').textContent = playing ? 'pause' : 'play'; };
// explode: stack parts slide along the flange axis (tool z), the bracket and gripper sideways, fingers down
function applyExplode(){ const k = exploded ? 1 : 0; const g1 = 0.012; if(groups['fasteners']) groups['fasteners'].visible = !exploded;
 // the flange stays; every part below it moves down its own step along the flange axis (tool z), the bracket and plate also step east (tool y);
 // gripper and fingers live in the jaw frame (x down, y east), so their steps are along x; the two fingers part further along the stroke
 const zstep = {stack_adapter_plate: 1, stack_nano17: 2, stack_tool_plate: 3, stack_zaber: 3.5};
 for(const m of objs){ const g = m.userData; let off = [0,0,0]; const n = g.name;
  if(n==='stack_flange'){ off = [0,0,0]; }
  else if(n.startsWith('stack_')){ off = [0, 0, k*g1*(zstep[n]||3)]; }
  else if(n.startsWith('bracket_')){ off = [0, k*g1*1.6, k*g1*4.2]; }
  else if(n==='gripper_plate'){ off = [0, k*g1*0.8, k*g1*4.2]; }
  else if(n.startsWith('gripper_')){ off = [k*g1*4.2, 0, 0]; }
  else if(n==='jaw_lo'||n==='plate_lo'||n==='leg_e'||n==='shoe_g'||n==='shoe_bar'||n==='blade'){ off = [k*g1*(n==='jaw_lo'?5.0:6.0), 0, 0]; }
  else if(n==='jaw_up'||n==='plate_up'||n==='pad_leg_e'||n==='pad_arm'||n==='pad_g'){ off = [k*g1*(n==='jaw_up'?3.4:2.4), 0, 0]; }
  else if(n.startsWith('macro_')||n==='d405_g'){ off = [0, 0, 0]; }
  g.explode = off; }
 setFrame(+slider.value); }
document.getElementById('explode').onclick = (e) => { exploded = !exploded; e.target.textContent = exploded ? 'assemble' : 'explode the stack'; applyExplode(); };
let ti = 0; const tour = document.getElementById('tour');
function showStep(i){ ti = Math.max(0, Math.min(i, V.steps.length-1)); const st = V.steps[ti]; for(const m of objs) highlight(partOf(m), false);
 document.getElementById('tourt').textContent = st.title; document.getElementById('tourx').innerHTML = st.text + (st.images ? '<div style="display:flex;gap:4px;margin-top:8px">' + st.images.map(u => `<img src="${u}" style="width:33%;border-radius:6px">`).join('') + '</div>' : ''); document.getElementById('tn').textContent = `${ti+1} / ${V.steps.length}`;
 document.getElementById('sheet').style.display = 'none'; document.getElementById('sheetbtn').textContent = 'drawing, connections, order list';
 if(st.phase){ const ph = V.phases.find(p => p.label.startsWith(st.phase)); if(ph){ slider.value = Math.min(ph.frame + 20, V.frames.length-1); setFrame(+slider.value); } look(st.view || 'fingers'); }
 else if(st.look){ lookAtPart(st.look, true); }
 for(const pid of st.parts) highlight(pid, true); }
document.getElementById('walk').onclick = () => { tour.style.display = 'block'; showStep(0); };
document.getElementById('tprev').onclick = () => showStep(ti-1); document.getElementById('tnext').onclick = () => showStep(ti+1);
document.getElementById('tclose').onclick = () => { tour.style.display = 'none'; for(const m of objs) highlight(partOf(m), false); };
document.getElementById('sheetbtn').onclick = (e) => { const s = document.getElementById('sheet'); const on = s.style.display !== 'block'; s.style.display = on ? 'block' : 'none'; e.target.textContent = on ? 'back to the 3D view' : 'drawing + connections'; };
// hover cards
const ray = new THREE.Raycaster(); const mouse = new THREE.Vector2(); let hovered = null; const card = document.getElementById('card');
function partOf(m){ return m.userData.part; }
function highlight(pid, on){ for(const m of objs){ if(partOf(m)===pid && !m.userData.nohover){ const mats = m.material ? [m.material] : m.children.map(c => c.material); for(const mt of mats){ if(!mt) continue; mt.emissive = new THREE.Color(on ? 0x335577 : 0x000000); mt.emissiveIntensity = on ? 0.8 : 0; } } } }
addEventListener('pointermove', (e) => { if(document.getElementById('sheet').style.display==='block') return; mouse.set((e.clientX/innerWidth)*2-1, -(e.clientY/innerHeight)*2+1); ray.setFromCamera(mouse, cam);
 const hits = ray.intersectObjects(objs.filter(m => m.visible && m.parent.visible && !m.userData.nohover), true); let ho = hits.length ? hits[0].object : null; while(ho && ho.userData.part === undefined && ho.parent) ho = ho.parent; const h = ho ? partOf(ho) : null;
 if(h !== hovered){ if(hovered) highlight(hovered, false); hovered = h; if(h) highlight(h, true); }
 if(h){ const p = P[h] || {name: h}; const sh = p.shape; const dims = !sh ? '' : `<div class="m">${typeof sh==='string' ? sh : (sh.type==='cylinder' ? `${sh.diameter_mm} mm dia x ${sh.height_mm} mm` : [sh.length_mm, sh.width_mm, sh.height_mm].filter(v => v!=null).join(' x ') + ' mm') + (sh.note ? ` (${sh.note})` : '')}</div>`; const m = p.mass_g ? `${p.mass_g} g` : '';
  card.innerHTML = `<b>${p.name || h}</b><div class="m">${[p.vendor, p.part_number, m].filter(Boolean).join(' · ')}</div><div>${p.what_it_is || ''}</div>${dims}${p.base_interface ? `<div class="m">mounts on: ${p.base_interface}</div>` : ''}${p.tool_interface ? `<div class="m">carries: ${p.tool_interface}</div>` : ''}${p.price_usd_if_listed ? `<div class="m">about $${p.price_usd_if_listed}</div>` : ''}${p.buy_url ? `<div class="m">click to open: ${p.buy_url.replace(/^https?:\/\//,'').slice(0,60)}</div>` : ''}`;
  card.style.display = 'block'; card.style.left = Math.min(e.clientX + 16, innerWidth - 360) + 'px'; card.style.top = Math.min(e.clientY + 16, innerHeight - 200) + 'px'; } else card.style.display = 'none'; });
addEventListener('click', () => { if(hovered && P[hovered] && P[hovered].buy_url) window.open(P[hovered].buy_url, '_blank'); });
window.look = look; function look(what){ const o = V.origin; const d = {connector: 0.045, fingers: 0.09, stack: 0.22, arm: 0.9}[what] || 0.1;
 const target = new THREE.Vector3(o[0] + (what==='stack'?0.02:0), o[1] + (what==='arm'?0.1:what==='stack'?0.03:0.005), o[2] + (what==='arm'?0.1:what==='stack'?0.06:0.01));
 controls.target.copy(target); cam.position.set(target.x + d*0.7, target.y - d*0.7, target.z + d*0.5); controls.update(); }
function lookAtPart(pid, keep){ const ms = objs.filter(m => partOf(m)===pid); if(!ms.length) return; const c = new THREE.Vector3(); for(const m of ms) c.add(m.position); c.multiplyScalar(1/ms.length); controls.target.copy(c); const d = 0.19; cam.position.set(c.x + d*0.7, c.y - d*0.7, c.z + d*0.45); controls.update(); highlight(pid, true); if(!keep) setTimeout(() => highlight(pid, false), 1500); }
sel.onchange = () => load(sel.value);
const H = Object.fromEntries(location.hash.slice(1).split('&').filter(Boolean).map(kv => kv.split('='))); load(H.variant && D.variants.some(v => v.id===H.variant) ? H.variant : D.variants[0].id); if(H.variant) sel.value = H.variant; look(H.view || 'stack'); if(H.explode){ document.getElementById('explode').click(); } if(H.sheet){ document.getElementById('sheetbtn').click(); } if(H.tour){ document.getElementById('walk').click(); showStep(+H.tour - 1); } if(H.frame){ slider.value = Math.min(+H.frame, V.frames.length-1); setFrame(+slider.value); }
let last = performance.now();
function loop(now){ requestAnimationFrame(loop); if(playing && now-last > 40){ last = now; slider.value = (+slider.value + 1) % V.frames.length; setFrame(+slider.value);} controls.update(); renderer.render(scene, cam); }
loop(0); addEventListener('resize', () => { cam.aspect = innerWidth/innerHeight; cam.updateProjectionMatrix(); renderer.setSize(innerWidth, innerHeight); });
</script></body></html>""".replace("__DATA__", js).replace("__MATJS__", K.material_js())
OUT.write_text(html); print("wrote", OUT, f"{OUT.stat().st_size/1e6:.1f} MB,", len(payloads), "variants")
