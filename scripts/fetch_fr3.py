"""Fetch a pinned upstream FR3 model and record checksums and its license."""

from __future__ import annotations

import hashlib
import json
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[1]
REPOSITORY = "google-deepmind/mujoco_menagerie"


def download(url: str) -> bytes:
    """Read one public upstream resource with a bounded timeout."""
    request = Request(url, headers={"User-Agent": "ffc-research-model-fetch"})
    with urlopen(request, timeout=60) as response:
        return response.read()


def main() -> None:
    """Download FR3 files at the recorded revision; preserve provenance."""
    provenance_path = ROOT / "config" / "fr3-source.json"
    if provenance_path.exists():
        revision = json.loads(provenance_path.read_text())["revision"]
    else:
        revision = json.loads(download(f"https://api.github.com/repos/{REPOSITORY}/commits/main"))["sha"]
    tree = json.loads(download(f"https://api.github.com/repos/{REPOSITORY}/git/trees/{revision}?recursive=1"))
    if tree.get("truncated"):
        raise RuntimeError("Upstream tree is truncated; refusing incomplete asset download")
    entries = [x for x in tree["tree"] if x["type"] == "blob" and x["path"].startswith("franka_fr3/")]
    if not any(x["path"] == "franka_fr3/fr3.xml" for x in entries):
        raise RuntimeError("FR3 model missing from upstream tree")

    def fetch(entry: dict) -> tuple[str, str]:
        path = entry["path"]
        target = ROOT / "third_party" / path
        payload = download(f"https://raw.githubusercontent.com/{REPOSITORY}/{revision}/{path}")
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(payload)
        return path, hashlib.sha256(payload).hexdigest()

    with ThreadPoolExecutor(max_workers=6) as executor:
        checksums = dict(executor.map(fetch, entries))
    provenance_path.parent.mkdir(parents=True, exist_ok=True)
    provenance_path.write_text(
        json.dumps(
            {
                "repository": f"https://github.com/{REPOSITORY}",
                "revision": revision,
                "license": "See third_party/franka_fr3/LICENSE",
                "sha256": checksums,
            },
            indent=2,
        )
        + "\n"
    )
    print(f"Fetched {len(entries)} FR3 files at {revision}")


if __name__ == "__main__":
    main()
