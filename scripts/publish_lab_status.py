"""Publish a concise public work update. Never exports terminal logs or private reasoning."""

import argparse
import json
import os
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--phase", required=True)
    p.add_argument("--title", required=True)
    p.add_argument("--detail", required=True)
    p.add_argument("--next", required=True)
    a = p.parse_args()
    path = ROOT / "outputs/lab-live/status.json"
    previous = json.loads(path.read_text()) if path.exists() else {"events": []}
    previous.update(phase=a.phase, title=a.title, detail=a.detail, next=a.next, updated_at=time.time())
    previous["events"].append({"time": previous["updated_at"], "title": a.title, "detail": a.detail})
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(previous, indent=2))
    os.replace(temporary, path)


if __name__ == "__main__":
    main()
