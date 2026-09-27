"""Exercise the public observation boundary using real HTTP requests."""

import importlib.util
import json
import threading
from http.server import ThreadingHTTPServer
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("lab_live_server", ROOT / "scripts/serve_lab_live.py")
server_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server_module)


def test_live_server_exposes_only_curated_read_only_data(tmp_path, monkeypatch):
    (tmp_path / "docs/live").mkdir(parents=True)
    (tmp_path / "docs/live/index.html").write_text("Public lab page")
    (tmp_path / "private.txt").write_text("must never be served")
    live = tmp_path / "outputs/lab-live"
    live.mkdir(parents=True)
    (live / "status.json").write_text(json.dumps({"title": "Test observation", "events": []}))
    monkeypatch.setattr(server_module, "ROOT", tmp_path)
    monkeypatch.setattr(server_module, "LIVE", live)
    server = ThreadingHTTPServer(("127.0.0.1", 0), server_module.Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_port}"
    try:
        with urlopen(base + "/api/status") as response:
            result = json.load(response)
            assert result["status"]["title"] == "Test observation"
            assert result["motion_permitted"] is False
            assert result["feeds"] == {}
        for path in [
            "/../private.txt",
            "/%2e%2e/private.txt",
            "/outputs/lab-live/status.json",
            "/api/frame/../../private.txt",
            "/live/../",
        ]:
            with pytest.raises(HTTPError) as error:
                urlopen(base + path)
            assert error.value.code in (403, 404)
        with pytest.raises(HTTPError) as error:
            urlopen(Request(base + "/api/status", data=b'{"move":true}', method="POST"))
        assert error.value.code == 501
        with urlopen(base + "/live/") as response:
            assert response.read() == b"Public lab page"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
