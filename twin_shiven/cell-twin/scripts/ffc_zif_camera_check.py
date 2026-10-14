"""Camera view check: render each wrist camera's OWN view along the recorded run and measure what the stack hides.
For every variant and every phase of the run (plus the final frame) this renders macro_e, macro_w and wrist_d405 from the camera's
real pose, with segmentation and depth, and reports (a) the share of the image covered by the stack's own hardware (bracket, gripper body,
camera arms, cameras, light), (b) the share covered by the fingers, shoe and pad (expected near the cable), (c) whether the three task
points, the cable tip, the connector mouth and the lid's front edge, are visible or hidden, and by which geom.
usage: .venv/bin/python scripts/ffc_zif_camera_check.py  -> ~/Desktop/Synphony-BOM/FFC-camera-views/{index.html, views.json, <variant>/*.png}"""
import json, math, sys, os
from pathlib import Path
import mujoco, numpy as np
from PIL import Image, ImageDraw
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from cell_twin import ffc_zif as F
VARIANTS = [("meca500", "megp_nano25", "laptop"), ("meca500", "megp_nano25", "platform"), ("meca500", "megp_nano25_thin", "laptop"), ("meca500", "megp_nano25_zaber", "laptop"),
            ("lrmate200id", "lehz16_mini40", "laptop"), ("lrmate200id", "lehz16_mini40", "platform"),
            ("meca500", "servo_v1", "laptop"), ("meca500", "servo_v1", "platform"), ("lrmate200id", "servo_v1_lr", "laptop")]   # as scripts/ffc_zif_matrix.py (not imported: importing it runs the matrix)

DEST = Path(os.environ.get("FFC_CAMVIEW_OUT", str(Path.home() / "Desktop/Synphony-BOM/FFC-camera-views")))
DEST.mkdir(parents=True, exist_ok=True)
CAMS = ("macro_e", "macro_w", "wrist_d405")
W, H = 640, 400
POINTS = {"cable tip": ("site", "ffc_tip"), "connector mouth": ("site", "zif_mouth"), "lid front edge": ("site", "zif_lip")}
HARDWARE_PREFIX = ("stack_", "bracket_", "gripper_", "jaw_", "plate_", "macro_", "d405", "arm_", "light_", "cam_", "zaber", "flange")
FINGER_PREFIX = ("leg_", "pad_", "shoe", "blade", "finger")


def descendants(model, root):
    out = set(); stack = [root]
    while stack:
        b = stack.pop(); out.add(b)
        stack += [c for c in range(model.nbody) if model.body_parentid[c] == b and c != b]
    return out


def project(model, data, cam_id, p):
    c = data.cam_xpos[cam_id]; R = data.cam_xmat[cam_id].reshape(3, 3)
    q = R.T @ (p - c)                                   # camera frame: looks along -z
    if q[2] >= -1e-6:
        return None
    f = (H / 2) / math.tan(math.radians(model.cam_fovy[cam_id]) / 2)
    u = W / 2 + f * (q[0] / -q[2]); v = H / 2 - f * (q[1] / -q[2])
    return u, v, -q[2]


def sweep(arm, tool, case, azs=range(-180, 180, 30), els=(25, 40), wd=None):
    """Where should a bench camera stand? One trial camera per (azimuth, elevation) at the lens working distance, aimed at the mouth; for every
    phase frame of the recorded run, is each task point visible past the arm and the stack? Prints the table and the best pair of azimuths."""
    tag = f"{arm}-{tool}-{case}"; src = ROOT / "out" / "ffc_zif" / tag / "traj.npz"
    spec = F.build(arm, case, tool); CM = F.CAMERAS[arm]; wd = wd or CM["wd"]
    o = F.connector_origin(arm); look = o + np.array([0.0, 0.003, 0.004]); w = spec.worldbody; names = []
    for az in azs:
        for el in els:
            a, e = math.radians(az), math.radians(el); d = np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])
            p = look + wd * d; nm = f"sweep_{az}_{el}"; names.append((nm, az, el))
            w.add_camera(name=nm, pos=p.tolist(), quat=F._look_quat(p, look, up=(0, 0, 1)), fovy=CM["fovy"])
    model = spec.compile(); data = mujoco.MjData(model)
    tr = np.load(src); Q = tr["qpos"]; PA = list(tr["phase_at"])
    frames = sorted({min(int(i), len(Q) - 1) for i in PA} | {len(Q) - 1} | {int(x) for x in np.linspace(0, len(Q) - 1, 12)})
    gname = [model.geom(g).name for g in range(model.ngeom)]
    tgt = {g for g in range(model.ngeom) if model.body(model.geom_bodyid[g]).name.startswith(("zif", "ffc"))}
    seg = mujoco.Renderer(model, H, W); dep = mujoco.Renderer(model, H, W); seg.enable_segmentation_rendering(); dep.enable_depth_rendering()
    score = {}
    for nm, az, el in names:
        cid = model.camera(nm).id; vis = 0; tot = 0; blockers = {}
        for i in frames:
            data.qpos[:] = Q[i]; mujoco.mj_forward(model, data)
            seg.update_scene(data, camera=nm); s = seg.render()[:, :, 0]; dep.update_scene(data, camera=nm); d = dep.render()
            for pn, (kind, sn) in POINTS.items():
                pr = project(model, data, cid, data.site(sn).xpos.copy()); tot += 1
                if pr is None: continue
                u, v, z = pr
                if not (0 <= u < W and 0 <= v < H): continue
                gid = int(s[int(v), int(u)])
                if float(d[int(v), int(u)]) >= z - 0.0008 or gid in tgt: vis += 1
                else: blockers[gname[gid] if gid >= 0 else "?"] = blockers.get(gname[gid] if gid >= 0 else "?", 0) + 1
        score[(az, el)] = (vis / tot, sorted(blockers.items(), key=lambda kv: -kv[1])[:3])
    print(f"\n{tag}: visibility of the tip, the mouth and the lid edge over {len(frames)} frames (azimuth from east toward north; cable comes from the south, -90)")
    for (az, el), (f, bl) in sorted(score.items()):
        print(f"  az {az:5d}  el {el:3d}  visible {f*100:5.1f}%   blocked by {bl}")
    best = None
    for (a1, e1), (f1, _) in score.items():
        for (a2, e2), (f2, _) in score.items():
            sep = abs((a1 - a2 + 180) % 360 - 180)
            if sep >= 60 and (best is None or f1 + f2 > best[0]):
                best = (f1 + f2, (a1, e1), (a2, e2))
    print(f"  best pair: az {best[1][0]} el {best[1][1]} and az {best[2][0]} el {best[2][1]} (mean visibility {best[0]/2*100:.1f}%)")
    sj = DEST / "sweep.json"; allres = json.loads(sj.read_text()) if sj.exists() else {}
    allres[tag] = {"frames": len(frames), "wd_mm": round(wd * 1e3, 1), "cells": [{"az": az, "el": el, "visible": round(f, 3), "blockers": bl} for (az, el), (f, bl) in sorted(score.items())],
                   "best": {"az": [best[1][0], best[2][0]], "el": [best[1][1], best[2][1]], "mean_visible": round(best[0] / 2, 3)}}
    sj.write_text(json.dumps(allres, indent=1))
    return score, best


def run(arm, tool, case):
    tag = f"{arm}-{tool}-{case}"; src = ROOT / "out" / "ffc_zif" / tag / "traj.npz"
    if not src.exists():
        return {"variant": tag, "error": "no trajectory"}
    spec = F.build(arm, case, tool); model = spec.compile(); data = mujoco.MjData(model)
    tr = np.load(src); Q = tr["qpos"]; PA = list(tr["phase_at"]); PL = [str(x) for x in tr["phase_lbl"]]
    frames = [(lbl, min(int(i), len(Q) - 1)) for lbl, i in zip(PL, PA)] + [("final", len(Q) - 1)]
    tool_bodies = descendants(model, model.body("tool").id)
    gname = [model.geom(g).name for g in range(model.ngeom)]
    hw, fg, tgt = set(), set(), set()
    conn_bodies = descendants(model, model.body("zif").id)
    cable_bodies = {b for b in range(model.nbody) if model.body(b).name.startswith("ffc_")}
    for g in range(model.ngeom):
        b = model.geom_bodyid[g]; n = gname[g]
        if b in tool_bodies:
            (fg if n.startswith(FINGER_PREFIX) else hw).add(g)
        elif b in conn_bodies or b in cable_bodies:
            tgt.add(g)
    ren = mujoco.Renderer(model, H, W); seg = mujoco.Renderer(model, H, W); dep = mujoco.Renderer(model, H, W)
    seg.enable_segmentation_rendering(); dep.enable_depth_rendering()
    vdir = DEST / tag; vdir.mkdir(exist_ok=True)
    rows = []
    for lbl, i in frames:
        data.qpos[:] = Q[i]; mujoco.mj_forward(model, data)
        for cam in CAMS:
            cid = model.camera(cam).id
            ren.update_scene(data, camera=cam); rgb = ren.render()
            seg.update_scene(data, camera=cam); s = seg.render()[:, :, 0]
            dep.update_scene(data, camera=cam); d = dep.render()
            n = s.size
            f_hw = float(np.isin(s, list(hw)).sum() / n); f_fg = float(np.isin(s, list(fg)).sum() / n); f_t = float(np.isin(s, list(tgt)).sum() / n)
            pts = {}
            img = Image.fromarray(rgb); dr = ImageDraw.Draw(img)
            for pn, (kind, nm) in POINTS.items():
                p = data.site(nm).xpos.copy()
                pr = project(model, data, cid, p)
                if pr is None:
                    pts[pn] = {"in_view": False, "visible": False, "by": "behind the camera"}; continue
                u, v, z = pr
                if not (0 <= u < W and 0 <= v < H):
                    pts[pn] = {"in_view": False, "visible": False, "by": "outside the frame"}; continue
                ui, vi = int(u), int(v); dz = float(d[vi, ui])
                gid = int(s[vi, ui]); by = gname[gid] if gid >= 0 else "nothing"
                vis = dz >= z - 0.0008 or gid in tgt
                pts[pn] = {"in_view": True, "visible": bool(vis), "by": None if vis else by, "px": [round(u), round(v)]}
                col = (0, 200, 90) if vis else (230, 40, 40)
                dr.ellipse([u - 5, v - 5, u + 5, v + 5], outline=col, width=2); dr.text((u + 7, v - 6), pn, fill=col)
            dr.text((6, 6), f"{tag}  {lbl}  {cam}", fill=(255, 255, 255))
            dr.text((6, H - 16), f"hardware {f_hw*100:.0f}%  fingers {f_fg*100:.0f}%  target {f_t*100:.0f}%", fill=(255, 255, 255))
            fn = f"{len(rows):02d}_{lbl.replace(' ', '_').replace('/', '-')}_{cam}.png"
            img.save(vdir / fn)
            rows.append({"phase": lbl, "frame": i, "camera": cam, "hardware_frac": round(f_hw, 3), "fingers_frac": round(f_fg, 3), "target_frac": round(f_t, 3), "points": pts, "png": f"{tag}/{fn}"})
    return {"variant": tag, "rows": rows}


def main():
    res = [run(*v) for v in VARIANTS]
    (DEST / "views.json").write_text(json.dumps(res, indent=1))
    # a page: one section per variant, one row per phase, three camera views across, red border where a task point is hidden by hardware
    html = ["<!doctype html><meta charset=utf-8><title>FFC wrist camera views</title><style>body{font:14px/1.4 -apple-system,Helvetica,sans-serif;margin:24px;background:#fdf9f2;color:#222}h1{font-size:22px}h2{font-size:17px;margin-top:28px}table{border-collapse:collapse}td{padding:4px;vertical-align:top}img{width:320px;display:block;border:3px solid #ddd}img.bad{border-color:#c62828}img.ok{border-color:#2e7d32}.n{font-size:12px;color:#555}.sum{background:#fff3e0;padding:8px 12px;border-radius:6px;display:inline-block}</style>",
            "<h1>What each wrist camera actually sees along the run</h1><p>Rendered from each camera's real pose in the twin at the start of every phase and at the end. Green border: the cable tip, the mouth and the lid edge are all visible (or legitimately out of frame). Red border: a task point is hidden behind the stack's own hardware. The footer of each view gives the share of the image covered by the stack hardware, by the fingers and by the connector and cable.</p>"]
    for r in res:
        if "error" in r:
            html.append(f"<h2>{r['variant']}</h2><p>{r['error']}</p>"); continue
        bad = [(x['phase'], x['camera'], pn, p['by']) for x in r["rows"] for pn, p in x["points"].items() if x["points"] and p["in_view"] and not p["visible"] and p["by"] and any(p["by"].startswith(h) for h in HARDWARE_PREFIX)]
        hwmax = max(x["hardware_frac"] for x in r["rows"]) if r["rows"] else 0
        html.append(f"<h2>{r['variant']}</h2><p class=sum>hardware in view: max {hwmax*100:.0f}% of a frame · task points hidden by hardware: {len(bad)}</p>")
        if bad:
            html.append("<ul class=n>" + "".join(f"<li>{ph} · {c}: {pn} behind {by}</li>" for ph, c, pn, by in bad) + "</ul>")
        phases = []
        for x in r["rows"]:
            if x["phase"] not in phases: phases.append(x["phase"])
        html.append("<table><tr><td></td>" + "".join(f"<td><b>{c}</b></td>" for c in CAMS) + "</tr>")
        for ph in phases:
            html.append(f"<tr><td class=n><b>{ph}</b></td>")
            for c in CAMS:
                x = next(y for y in r["rows"] if y["phase"] == ph and y["camera"] == c)
                hidden = [pn for pn, p in x["points"].items() if p["in_view"] and not p["visible"] and p["by"] and any(p["by"].startswith(h) for h in HARDWARE_PREFIX)]
                cls = "bad" if hidden else "ok"
                note = "; ".join(f"{pn}: {'ok' if p['visible'] else ('hidden by ' + str(p['by'])) if p['in_view'] else p['by']}" for pn, p in x["points"].items())
                html.append(f"<td><img class={cls} src='{x['png']}'><div class=n>{note}</div></td>")
            html.append("</tr>")
        html.append("</table>")
    (DEST / "index.html").write_text("\n".join(html))
    for r in res:
        if "rows" in r:
            nb = sum(1 for x in r["rows"] for pn, p in x["points"].items() if p["in_view"] and not p["visible"] and p["by"] and any(p["by"].startswith(h) for h in HARDWARE_PREFIX))
            print(f"{r['variant']}: {len(r['rows'])} views, hardware max {max(x['hardware_frac'] for x in r['rows'])*100:.0f}%, task points hidden by hardware: {nb}")
        else:
            print(r["variant"], r["error"])
    print("->", DEST / "index.html")


if __name__ == "__main__":
    if "--sweep" in sys.argv:
        for v in VARIANTS:
            if (ROOT / "out" / "ffc_zif" / f"{v[0]}-{v[1]}-{v[2]}" / "traj.npz").exists() and (len(sys.argv) < 3 or v[0] in sys.argv):
                sweep(*v)
    else:
        main()
