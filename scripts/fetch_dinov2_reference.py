"""Fetch and verify the pinned official DINOv2 source and ViT-B/14 backbone."""

import hashlib
import json
import tarfile
from pathlib import Path
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
lock = json.loads((ROOT / "config/dinov2-backbone.json").read_text())
dest = ROOT / "third_party/dinov2"
dest.mkdir(parents=True, exist_ok=True)
for name, url, key in [
    (
        "source.tar.gz",
        f"https://api.github.com/repos/facebookresearch/dinov2/tarball/{lock['commit']}",
        "source_archive_sha256",
    ),
    ("dinov2_vitb14_pretrain.pth", lock["weights_url"], "weights_sha256"),
]:
    path = dest / name
    if not path.exists():
        with urlopen(url, timeout=120) as response:
            path.write_bytes(response.read())
    if hashlib.sha256(path.read_bytes()).hexdigest() != lock[key]:
        raise ValueError(f"Hash mismatch: {name}")
    print("VERIFIED", name)
source = dest / "source"
if not source.exists():
    source.mkdir()
    with tarfile.open(dest / "source.tar.gz") as archive:
        for member in archive.getmembers():
            parts = Path(member.name).parts
            if len(parts) < 2:
                continue
            member.name = str(Path(*parts[1:]))
            archive.extract(member, source, filter="data")
