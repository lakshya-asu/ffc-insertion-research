"""Fetch official Camera 3 CAD, community Pi 4 CAD, and official dimension drawings."""

import argparse
import concurrent.futures
import hashlib
import json
import zipfile
from pathlib import Path
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / "third_party/raspberry_pi"
LOCK = json.loads((ROOT / "config/raspberry-pi-sources.json").read_text())
SOURCES = {name: record["url"] for name, record in LOCK.items()}


def fetch(item):
    name, url = item
    path = DEST / name
    if not path.exists():
        with urlopen(Request(url, headers={"User-Agent": "Mozilla/5.0"}), timeout=60) as response:
            content = response.read()
        path.write_bytes(content)
    content = path.read_bytes()
    digest = hashlib.sha256(content).hexdigest()
    if digest != LOCK[name]["sha256"]:
        raise ValueError(f"Source changed or download is invalid: {name}; inspect before updating the lock")
    if name.endswith(".zip"):
        with zipfile.ZipFile(path) as archive:
            for info in archive.infolist():
                extract_root = DEST / Path(name).stem
                target = extract_root / Path(info.filename)
                if not target.resolve().is_relative_to(extract_root.resolve()):
                    raise ValueError("Unsafe archive path")
            archive.extractall(DEST / Path(name).stem)
    record = {
        "url": url,
        "bytes": path.stat().st_size,
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
    }
    print(name, record["bytes"], flush=True)
    return name, record


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=ROOT / "config/raspberry-pi-sources.json")
    parser.add_argument("--destination", type=Path, default=DEST)
    args = parser.parse_args()
    DEST = args.destination
    LOCK = json.loads(args.manifest.read_text())
    SOURCES = {name: record["url"] for name, record in LOCK.items()}
    DEST.mkdir(parents=True, exist_ok=True)
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        result = dict(pool.map(fetch, SOURCES.items()))
    (DEST / "sources.json").write_text(json.dumps(result, indent=2) + "\n")
