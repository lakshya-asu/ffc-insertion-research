"""Labelled renders from the twin for the servo tool explainer: the bought stack and the servo tool side by side on the same arm, at the
same frame of the same run (the slide into the connector), from two viewpoints, every part named with a leader line.
usage: .venv/bin/python scripts/ffc_servo_tool_figures.py [out dir]"""
import sys, json
from pathlib import Path
import numpy as np, mujoco
from PIL import Image, ImageDraw, ImageFont
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from cell_twin import ffc_zif as F
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path.home() / "Desktop" / "Synphony-BOM" / "FFC-servo-tool-figs"
OUT.mkdir(parents=True, exist_ok=True)
W, H = 900, 760
LABELS = {
    "bought": {"stack_flange": "robot flange", "stack_adapter_plate": "adapter plate (ours, 6061)", "stack_nano25": "ATI Nano25 six-axis sensor (about $7,000)",
               "stack_mini40": "ATI Mini40 six-axis sensor (about $8,000)", "bracket_v": "90 deg bracket (ours, 6061)", "gripper_mesh": "bought gripper (MEGP 25E $1,675)",
               "gripper_body": "bought gripper (SMC LEHZ16, about $1,300)", "plate_up": "upper finger (printed)", "pad_g": "silicone pad", "shoe_g": "steel shoe with the lip",
               "plate_lo": "lower finger (printed)", "d405_g": "D405 wrist camera", "arm_d405_plate": "camera side plate", "blade": "1.5 mm lip (hooks the lid)", "leg_e": "east leg of the lower finger", "pad_leg_e": "east leg of the upper finger", "pad_arm": "pad arm", "shoe_bar": "shoe bar", "jaw_up_mesh": "upper jaw", "jaw_lo_mesh": "lower jaw"},
    "servo": {"stack_flange": "robot flange", "stack_adapter_plate": "adapter plate (ours, 6061)", "stack_standoff": "printed standoff", "bracket_h": "cell A: bar load cell, insertion axis",
              "cell_b": "cell B: bar load cell, vertical axis", "cell_corner": "corner plate joining A and B", "bracket_v": "side plate to the body", "gripper_body": "printed body: XL330 servo + M4 x 1 lead screw",
              "gripper_end": "end block: rail, two carriers, rocker", "cell_p": "cell P: bar load cell, the pinch", "plate_up": "upper finger (same part)", "pad_g": "silicone pad (same)",
              "shoe_g": "steel shoe with the lip (same)", "plate_lo": "lower finger (same part)", "d405_g": "D405 wrist camera (same)", "blade": "1.5 mm lip (same)", "leg_e": "east leg (same)", "pad_leg_e": "east leg of the upper finger (same)", "pad_arm": "pad arm (same)", "shoe_bar": "shoe bar (same)", "jaw_up": "upper jaw carrier (on the nut)", "jaw_lo": "lower jaw carrier (rocker-driven)"}}
VIEWS = {"stack": dict(dist=0.21, az=-140, el=-28, what="whole stack, flange to connector, seen from the cable side (south-west); the robot arm is hidden"), "fingers": dict(cam="macro_w", what="the fingers at the connector mouth as the west bench camera sees them (160 mm away, 25 deg down)")}
try:
    FONT = ImageFont.truetype("/System/Library/Fonts/Supplemental/Arial.ttf", 15); FONTB = ImageFont.truetype("/System/Library/Fonts/Supplemental/Arial Bold.ttf", 19)
except Exception:
    FONT = ImageFont.load_default(); FONTB = FONT


def pose_frame(variant):
    z = np.load(ROOT / "out/ffc_zif" / variant / "traj.npz", allow_pickle=True)
    lbl = [str(x) for x in z["phase_lbl"]]; at = list(z["phase_at"])
    k = next((i for i, l in enumerate(lbl) if l.startswith("phase 2: slide")), len(lbl) - 1)
    f0 = at[k]; f1 = at[k + 1] if k + 1 < len(at) else len(z["qpos"])
    return z["qpos"][(f0 + f1) // 2]


def render(arm, tool, case, kind, view):
    variant = f"{arm}-{tool}-{case}"
    spec = F.build(arm, case, tool); model = spec.compile(); data = mujoco.MjData(model)
    q = pose_frame(variant); data.qpos[:len(q)] = q; mujoco.mj_forward(model, data)
    ren = mujoco.Renderer(model, H, W)
    cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    v = VIEWS[view]
    tcp = data.site("tcp").xpos; tool_p = data.xpos[model.body("tool").id]
    cam.lookat[:] = (tcp + tool_p) / 2 if view == "stack" else tcp + np.array([0, 0, 0.004])
    if "dist" in v:
        cam.distance = v["dist"]; cam.azimuth = v["az"]; cam.elevation = v["el"]
    opt = mujoco.MjvOption(); opt.geomgroup[3] = 0            # group 3 = hidden in these figures
    hide_prefix = (F.ARM_INFO[arm]["prefix"],)
    for g in range(model.ngeom):
        bn = model.body(model.geom_bodyid[g]).name; gn = model.geom(g).name
        if bn.startswith(F.ARM_INFO[arm]["prefix"]) or gn.startswith(hide_prefix):
            model.geom_group[g] = 3
    if v.get("cam"):
        cam = v["cam"]                                            # one of the twin's own bench cameras
    ren.update_scene(data, camera=cam, scene_option=opt); rgb = ren.render().copy()
    ren.enable_segmentation_rendering(); ren.update_scene(data, camera=cam, scene_option=opt); seg = ren.render()[:, :, 0].astype(int)
    ren.disable_segmentation_rendering()
    names = {model.geom(g).name: g for g in range(model.ngeom) if model.geom(g).name and model.geom_rgba[g][3] > 0.05}
    cents = {}
    for nm, g in names.items():
        if nm in LABELS[kind]:
            ys, xs = np.nonzero(seg == g)
            if len(xs) > 25:
                cents[nm] = (float(xs.mean()), float(ys.mean()), len(xs))
    return Image.fromarray(rgb), cents, variant


def panel(img, cents, kind, side, title):
    """the render with a label column on one side, leader lines to each part's pixel centroid"""
    M = 400; pw = W + M
    out = Image.new("RGB", (pw, H + 50), "#FDF9F2"); d = ImageDraw.Draw(out)
    ox = M if side == "left" else 0
    out.paste(img, (ox, 50)); d.text((ox + 10, 12), title, fill="#222", font=FONTB)
    items = sorted(cents.items(), key=lambda kv: kv[1][1])
    n = len(items); ys = np.linspace(80, H + 20, n) if n else []
    for (nm, (cx, cy, _)), ly in zip(items, ys):
        px, py = ox + cx, 50 + cy
        if side == "left":
            tx = 10; ex = M - 8
        else:
            tx = W + 12; ex = W + 6
        d.line([(px, py), (ex, ly)], fill="#b23", width=1); d.ellipse([px - 3, py - 3, px + 3, py + 3], fill="#b23")
        txt = LABELS[kind][nm]
        d.text((tx, ly - 9), txt, fill="#222", font=FONT) if side == "right" else d.text((ex - 4 - d.textlength(txt, font=FONT), ly - 9), txt, fill="#222", font=FONT)
    return out


def figure(arm, bought, servo, view, fname):
    ib, cb, vb = render(arm, bought, "laptop", "bought", view)
    is_, cs, vs = render(arm, servo, "laptop", "servo", view)
    pb = panel(ib, cb, "bought", "left", f"{F.ARM_INFO[arm].get('label', arm)}: bought stack ({vb})")
    ps = panel(is_, cs, "servo", "right", f"{F.ARM_INFO[arm].get('label', arm)}: servo tool v1 ({vs})")
    out = Image.new("RGB", (pb.width + ps.width + 20, pb.height + 30), "#FDF9F2"); out.paste(pb, (0, 0)); out.paste(ps, (pb.width + 20, 0))
    d = ImageDraw.Draw(out); d.text((12, pb.height + 4), f"Same run, same frame (the slide into the connector), same view: {VIEWS[view]['what']}. Rendered from the twin.", fill="#555", font=FONT)
    out.save(OUT / fname); print("wrote", OUT / fname, out.size, "labels", len(cb), len(cs))


figure("meca500", "megp_nano25", "servo_v1", "stack", "meca500_stack.png")
figure("meca500", "megp_nano25", "servo_v1", "fingers", "meca500_fingers.png")
figure("lrmate200id", "lehz16_mini40", "servo_v1_lr", "stack", "lrmate_stack.png")
figure("lrmate200id", "lehz16_mini40", "servo_v1_lr", "fingers", "lrmate_fingers.png")
