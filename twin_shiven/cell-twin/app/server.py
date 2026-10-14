"""Cell twin web app: live bench view, experiments, camera checks. Serves on http://127.0.0.1:8771

run: .venv/bin/python app/server.py   (starts the live worker process too)
"""
from __future__ import annotations

import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

from fastapi import FastAPI, Request
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse, Response

ROOT = Path(__file__).resolve().parent.parent
LIVE = ROOT / "out" / "live"
OUT = ROOT / "out"
PY = ROOT / ".venv" / "bin" / "python"
app = FastAPI()
LIVE.mkdir(parents=True, exist_ok=True)
_worker: subprocess.Popen | None = None
_jobs: dict[str, dict] = {}


def ensure_worker():
    global _worker
    if _worker is None or _worker.poll() is not None:
        _worker = subprocess.Popen([str(PY), str(ROOT / "app" / "worker.py")], cwd=ROOT,
                                   stdout=open(LIVE / "worker.log", "a"), stderr=subprocess.STDOUT)


@app.on_event("startup")
def _start():
    ensure_worker()


@app.get("/", response_class=HTMLResponse)
def index():
    return (ROOT / "app" / "index.html").read_text()


def _img(path: Path):
    if not path.exists():
        return Response(status_code=404)
    return Response(path.read_bytes(), media_type="image/jpeg", headers={"Cache-Control": "no-store"})


@app.get("/frame.jpg")
def frame():
    return _img(LIVE / "frame.jpg")


@app.get("/cams.jpg")
def cams():
    return _img(LIVE / "cams.jpg")


@app.get("/api/state")
def state():
    ensure_worker()
    p = LIVE / "state.json"
    return JSONResponse(json.loads(p.read_text()) if p.exists() else {"running": False, "log": ["worker starting"]})


@app.post("/api/command")
async def command(req: Request):
    body = await req.json()
    (LIVE / "command.json").write_text(json.dumps(body))
    return {"ok": True}


# ---- experiments: capture-region sweeps as background jobs ----
@app.get("/api/experiments")
def experiments():
    rows = []
    for log in sorted(OUT.glob("capture_*.log"), key=os.path.getmtime, reverse=True):
        txt = log.read_text()
        results = re.findall(r"^== (\w+) skew ([\d.]+) deg: (\d+)/(\d+)", txt, re.M)
        kind = "lateral" if "lateral" in log.name else "skew"
        rows.append({"name": log.stem, "kind": kind, "mtime": time.strftime("%m-%d %H:%M", time.localtime(os.path.getmtime(log))),
                     "results": [{"controller": c, "x": float(x), "seats": int(a), "n": int(b)} for c, x, a, b in results],
                     "running": any(j["log"] == log.name and j["proc"].poll() is None for j in _jobs.values())})
    return rows


@app.post("/api/experiments")
async def launch(req: Request):
    body = await req.json()
    kind = body.get("kind", "lateral"); values = body.get("values", "0,0.5,1,1.5,2,3"); n = int(body.get("n", 10))
    controllers = body.get("controllers", "push,align")
    stamp = time.strftime("%m%d-%H%M%S"); log = f"capture_{kind}_{stamp}.log"
    args = [str(PY), str(ROOT / "scripts" / "capture_region.py"), "--n", str(n), "--controllers", controllers]
    args += ["--laterals", values] if kind == "lateral" else ["--skews", values]
    proc = subprocess.Popen(args, cwd=ROOT, stdout=open(OUT / log, "w"), stderr=subprocess.STDOUT)
    _jobs[stamp] = {"proc": proc, "log": log}
    return {"ok": True, "log": log}


@app.get("/api/experiments/{name}")
def experiment_detail(name: str):
    p = OUT / f"{name}.log"
    return {"text": p.read_text()[-20000:] if p.exists() else ""}


# ---- camera checks ----
@app.post("/api/cameras/render")
def cameras_render():
    subprocess.run([str(PY), str(ROOT / "scripts" / "render_check.py")], cwd=ROOT, capture_output=True)
    subprocess.run([str(PY), str(ROOT / "scripts" / "wrist_cam_check.py")], cwd=ROOT, capture_output=True)
    subprocess.run([str(PY), str(ROOT / "scripts" / "mount_check.py")], cwd=ROOT, capture_output=True)
    subprocess.run([str(PY), str(ROOT / "scripts" / "blend_check.py")], cwd=ROOT, capture_output=True)
    for cam in ("scene", "right_wrist", "left_wrist"):
        subprocess.run([str(PY), str(ROOT / "scripts" / "cam_verify.py"), cam], cwd=ROOT, capture_output=True)
        src = OUT / "camgrid" / f"verify_{cam}_0.png"
        if src.exists(): (OUT / f"verify_{cam}_0.png").write_bytes(src.read_bytes())
    return {"ok": True}


@app.get("/out/{name}")
def out_file(name: str):
    p = OUT / name
    if not p.exists() or not p.suffix in (".png", ".jpg", ".json", ".log", ".mp4"):
        return Response(status_code=404)
    return FileResponse(p, headers={"Cache-Control": "no-store"})


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8771, log_level="warning")
