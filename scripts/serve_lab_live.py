"""Read-only public lab view. Serves only the website and curated experiment telemetry."""

import argparse
import json
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parents[1]
LIVE = ROOT / "outputs/lab-live"
VIEWS = ("workcell", "desk", "board")


def load(path, default):
    try:
        return json.loads(path.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return default


def snapshot():
    status = load(LIVE / "status.json", {})
    feeds = {}
    for name in VIEWS:
        meta = load(LIVE / "feed" / (name + ".json"), None)
        if meta and (LIVE / "feed" / (name + ".jpg")).is_file():
            feeds[name] = meta
    history = load(ROOT / "outputs/pi-perception-model-002/history.json", [])
    training = {"epochs_planned": 40, "epochs_completed": len(history), "history": history}
    if history:
        training["best_validation_foreground_mean_iou"] = max(r["foreground_mean_iou"] for r in history)
    training["invalid"] = load(ROOT / "outputs/pi-perception-model-002/invalid.json", None)
    training["complete"] = (ROOT / "outputs/pi-perception-model-002/training-report.json").exists()
    return {
        "server_time": time.time(),
        "status": status,
        "feeds": feeds,
        "training": training,
        "access": "read-only observation; no robot controls",
        "motion_permitted": False,
    }


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT / "docs"), **kwargs)

    def end_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        super().end_headers()

    def list_directory(self, path):
        self.send_error(403, "Directory listing disabled")
        return None

    def reply(self, data, content_type, head=False):
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        if not head:
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

    def route(self, head=False):
        path = urlsplit(self.path).path
        if path == "/api/status":
            self.reply(json.dumps(snapshot(), allow_nan=False).encode(), "application/json", head)
            return True
        for name in VIEWS:
            if path == "/api/frame/" + name:
                image = LIVE / "feed" / (name + ".jpg")
                if image.is_file():
                    self.reply(image.read_bytes(), "image/jpeg", head)
                else:
                    self.send_error(404, "No camera frame yet")
                return True
        if path.startswith("/api/"):
            self.send_error(404, "Unknown endpoint")
            return True
        if path == "/":
            self.send_response(302)
            self.send_header("Location", "/live/")
            self.end_headers()
            return True
        return False

    def do_GET(self):
        if not self.route():
            super().do_GET()

    def do_HEAD(self):
        if not self.route(True):
            super().do_HEAD()

    def log_message(self, format, *args):
        # No request/visitor log is required for this research view.
        pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8766)
    args = parser.parse_args()
    print(f"Read-only lab view on 127.0.0.1:{args.port}", flush=True)
    ThreadingHTTPServer(("127.0.0.1", args.port), Handler).serve_forever()
