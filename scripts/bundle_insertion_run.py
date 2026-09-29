"""Package frozen contact-pilot sources and CAD with explicit untested-rerun status."""

import argparse
import hashlib
import json
import shutil
import subprocess
import tarfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("run", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    report = json.loads((a.run / "report.json").read_text())
    out = a.output
    out.mkdir(parents=True, exist_ok=False)
    for source in a.run.glob("*.py"):
        target = (
            out / "scripts/isaac_insertion_bench.py"
            if source.name == "source.py"
            else out / "src/ffc" / source.name
        )
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    (out / "config/cables").mkdir(parents=True)
    shutil.copy2(a.run / "rpi-camera-standard-mini-200-rev2.json", out / "config/cables")
    for name in ["zero-reference-contact-v1.json", "pi-socket-evidence-v1.json"]:
        if (a.run / name).exists():
            (out / "config/connectors").mkdir(parents=True, exist_ok=True)
            shutil.copy2(a.run / name, out / "config/connectors" / name)
    asset = "outputs/compact-fingers-002/compact-fingers.usda"
    (out / asset).parent.mkdir(parents=True)
    shutil.copy2(ROOT / asset, out / asset)
    dockerfile = (ROOT / "Dockerfile").read_text().split("COPY config/fr3-source.json")[0]
    dockerfile += (
        "COPY scripts/patch_isaac_entrypoint.py /workspace/scripts/patch_isaac_entrypoint.py\n"
        "RUN /isaac-sim/python.sh /workspace/scripts/patch_isaac_entrypoint.py\n"
        'ENTRYPOINT ["/isaac-sim/python.sh"]\n'
    )
    (out / "Dockerfile").write_text(dockerfile)
    shutil.copy2(ROOT / "scripts/patch_isaac_entrypoint.py", out / "scripts")
    options = f"--case {report['case']} --dt {report['physics_dt_s']} --segments {report['segment_count']}"
    if "tool_orientation" in report:
        options += f" --orientation {report['tool_orientation']}"
    if report.get("socket_reference"):
        invocation = report["invocation"]
        options += (
            f" --socket --socket-offset-mm {invocation['socket_offset_mm']}"
            f" --socket-height-mm {invocation['socket_height_mm']}"
        )
        if invocation["square_entry"]:
            options += " --square-entry"
    (out / "run.sh").write_text(
        '#!/usr/bin/env bash\nset -euo pipefail\ncd -- "$(dirname -- "$0")"\n'
        'run_name="${1:-rerun-001}"\n'
        '[[ "$run_name" =~ ^[a-zA-Z0-9_-]+$ ]] || exit 2\n'
        'test ! -e "outputs/$run_name"\n'
        "docker run --rm --gpus all --shm-size=4gb -e ACCEPT_EULA=Y -e OMNI_KIT_ACCEPT_EULA=YES "
        '-e PRIVACY_CONSENT=N -v "$PWD:/workspace" -w /workspace '
        '"${FFC_RUNTIME_IMAGE:-ffc-insertion-rerun:6.1.0}" scripts/isaac_insertion_bench.py '
        + options
        + ' --output "/workspace/outputs/$run_name"\n'
    )
    (out / "README.md").write_text(f"""# Frozen sources: {a.run.name}

Build with `docker build -t ffc-insertion-rerun:6.1.0 .`, then `bash run.sh rerun-001`.
On the original lab machine, use `FFC_RUNTIME_IMAGE=ffc-isaac:6.1.0 bash run.sh rerun-001`.
NVIDIA GPU Docker support and Isaac licence acceptance are required.

This bundle has not had a fresh image build or an independent physics rerun.
It includes recorded source snapshots and the current CAD asset, hashed at packaging.
The experiment did not hash that CAD at acquisition, so its historic identity is not independently proved.
USD playback is separate from physics execution. Supplier ownership is unchanged.
Check report.json and sensor-trace.json, not only process exit status. No seating is verified.
""")
    identity = subprocess.check_output(
        ["docker", "image", "inspect", "ffc-isaac:6.1.0", "--format", "{{.Id}}"], text=True
    ).strip()
    manifest = {
        "source_run": a.run.name,
        "runtime_image_id_at_packaging": identity,
        "fresh_build_tested": False,
        "rerun_tested": False,
        "sha256": {
            str(f.relative_to(out)): hashlib.sha256(f.read_bytes()).hexdigest()
            for f in out.rglob("*")
            if f.is_file()
        },
    }
    (out / "bundle-manifest.json").write_text(json.dumps(manifest, indent=2))
    with tarfile.open(str(out) + ".tar.gz", "w:gz") as archive:
        archive.add(out, arcname=out.name)
    print(str(out) + ".tar.gz")


if __name__ == "__main__":
    main()
