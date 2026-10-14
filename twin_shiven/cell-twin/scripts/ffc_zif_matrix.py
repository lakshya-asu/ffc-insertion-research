"""Run the variant matrix: arm x tool stack x scene case. Each variant: scripted expert, clearance audit, video.
usage: .venv/bin/python scripts/ffc_zif_matrix.py            -> out/ffc_zif/matrix.json (+ per-variant folders)"""
import json, os, subprocess, sys, time
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent
PY = str(ROOT / ".venv" / "bin" / "python")
VARIANTS = [
    ("meca500", "megp_nano25", "laptop"),
    ("meca500", "megp_nano25", "platform"),
    ("meca500", "megp_nano25_thin", "laptop"),
    ("meca500", "megp_nano25_zaber", "laptop"),
    ("lrmate200id", "lehz16_mini40", "laptop"),
    ("lrmate200id", "lehz16_mini40", "platform"),
    ("meca500", "servo_v1", "laptop"),
    ("meca500", "servo_v1", "platform"),
    ("lrmate200id", "servo_v1_lr", "laptop"),
]
if len(sys.argv) > 1:   # run a subset: substrings of "arm-tool-case"; the other variants keep their previous results in matrix.json
    VARIANTS = [v for v in VARIANTS if any(k in "-".join(v) for k in sys.argv[1:])]
results = []
for arm, tool, case in VARIANTS:
    env = dict(os.environ, FFC_ARM=arm, FFC_TOOL=tool, FFC_CASE=case)
    name = f"{arm}-{tool}-{case}"; out = ROOT / "out" / "ffc_zif" / name; out.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    r = subprocess.run([PY, str(ROOT / "scripts/ffc_zif_demo.py"), arm], env=env, capture_output=True, text=True)
    (out / "demo.log").write_text(r.stdout + r.stderr)
    lines = r.stdout.splitlines()
    pick = [l for l in lines if any(k in l for k in ("VERIFY", "after phase", "after release", "after push", "stopping", "unreachable", "tool tilt", "tip at mouth"))]
    c = subprocess.run([PY, str(ROOT / "scripts/ffc_zif_clearance.py"), arm], env=env, capture_output=True, text=True)
    (out / "clearance.log").write_text(c.stdout + c.stderr)
    v = subprocess.run([PY, str(ROOT / "scripts/ffc_zif_watch.py"), arm, "--video", str(out / "run.mp4")], env=env, capture_output=True, text=True)
    ver = {}
    try:
        ver = json.loads((out / "log.json").read_text())["verify"]
    except Exception as e:
        ver = {"error": str(e)}
    clr = {}
    try:
        clr = json.loads((out / "clearance.json").read_text())
    except Exception:
        pass
    results.append({"variant": name, "arm": arm, "tool": tool, "case": case, "verify": ver, "key_lines": pick, "clearance": clr, "seconds": round(time.time() - t0, 1),
                    "video": str(out / "run.mp4") if (out / "run.mp4").exists() else None, "demo_rc": r.returncode, "clearance_rc": c.returncode, "video_rc": v.returncode})
    print(f"{name:42s} rc={r.returncode} {ver.get('latch_closed')} depth={ver.get('depth_mm')} lateral={ver.get('lateral_mm')} creep={ver.get('release_creep_mm')} dip={ver.get('dip_max')}  ({time.time()-t0:.0f}s)", flush=True)
    for l in pick:
        if "unreachable" in l or "stopping" in l:
            print("   ", l[:150], flush=True)
mpath = ROOT / "out" / "ffc_zif" / "matrix.json"
if len(sys.argv) > 1 and mpath.exists():
    done = {r["variant"] for r in results}
    results = [r for r in json.loads(mpath.read_text()) if r["variant"] not in done] + results
mpath.write_text(json.dumps(results, indent=1))
print("wrote", ROOT / "out" / "ffc_zif" / "matrix.json")
