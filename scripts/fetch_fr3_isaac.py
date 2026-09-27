"""Fetch pinned official FR3 USD archive and verify it before safe extraction."""

from __future__ import annotations

import hashlib
import json
import tarfile
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
URL = "https://github.com/frankarobotics/franka_simulation/releases/download/v0.1.0/fr3v2_1-isaacsim6.tar.gz"
DIGEST = "b46fba7bc563d62ff1fd5c3af318a4f125173a41cf8662b32117df07be51d18c"


def main() -> None:
    destination = ROOT / "third_party/franka_isaac"
    destination.mkdir(parents=True, exist_ok=True)
    archive = destination / "fr3v2_1-isaacsim6.tar.gz"
    if not archive.exists():
        urllib.request.urlretrieve(URL, archive)
    digest = hashlib.sha256(archive.read_bytes()).hexdigest()
    if digest != DIGEST:
        raise RuntimeError(f"FR3 archive hash mismatch: {digest}")
    with tarfile.open(archive) as data:
        data.extractall(destination, filter="data")
    (ROOT / "config/fr3-isaac-source.json").write_text(
        json.dumps(
            {
                "source": URL,
                "sha256": DIGEST,
                "release": "v0.1.0",
                "variant": "FR3 v2.1",
                "upstream_tested_runtime": "6.0.0-rc.59",
                "local_target_runtime": "6.1.0.0",
            },
            indent=2,
        )
        + "\n"
    )
    print(destination / "fr3v2_1/fr3v2_1.usda")


if __name__ == "__main__":
    main()
