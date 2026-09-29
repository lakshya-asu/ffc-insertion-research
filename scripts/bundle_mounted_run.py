"""Freeze a mounted-run source/asset bundle. This packages a rerun, not a validation claim."""

import argparse
import hashlib
import json
import shutil
import subprocess
import tarfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def bundle(run, out):
    out.mkdir(parents=True, exist_ok=False)
    hashes = json.loads((run / "source-hashes.json").read_text())
    for name, digest in hashes.items():
        source = run / name
        if hashlib.sha256(source.read_bytes()).hexdigest() != digest:
            raise ValueError("Source snapshot changed: " + name)
        if name == "isaac_fr3_flexible_lift.py":
            target = out / "scripts" / name
        elif name.endswith(".py"):
            target = out / "src/ffc" / name
        else:
            directory = "config/cables" if name.startswith("rpi-camera") else "config"
            if name in ("zero-reference-contact-v1.json", "pi-socket-evidence-v1.json"):
                directory = "config/connectors"
            target = out / directory / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    for rel in ["outputs/compact-fingers-002/compact-fingers.usda", "outputs/compact-mount-001/adapter.usda"]:
        dest = out / rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / rel, dest)
    shutil.copytree(ROOT / "third_party/franka_isaac/fr3v2_1", out / "third_party/franka_isaac/fr3v2_1")
    # The lab publisher is not needed: reruns intentionally omit --live-feed.
    dockerfile = (ROOT / "Dockerfile").read_text().split("COPY config/fr3-source.json")[0]
    dockerfile += (
        "COPY scripts/patch_isaac_entrypoint.py /workspace/scripts/patch_isaac_entrypoint.py\n"
        "RUN /isaac-sim/python.sh /workspace/scripts/patch_isaac_entrypoint.py\n"
        'ENTRYPOINT ["/isaac-sim/python.sh"]\n'
    )
    (out / "Dockerfile").write_text(dockerfile)
    shutil.copy2(ROOT / "scripts/patch_isaac_entrypoint.py", out / "scripts/patch_isaac_entrypoint.py")
    if (run / "report.json").exists():
        report = json.loads((run / "report.json").read_text())
        case, dt = report["case"], report["dt_s"]
        if report.get("zero_cell"):
            dest = out / "third_party/raspberry_pi/zero"
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copytree(ROOT / "third_party/raspberry_pi/zero", dest)
        servo_option = (
            f" --joint-integral-gain {report['joint_integral_gain_per_s']}"
            if "joint_integral_gain_per_s" in report
            else ""
        )
        if report.get("zero_cell"):
            servo_option += " --zero-cell"
        for key, rel in [
            ("cad_sha256", "outputs/compact-fingers-002/compact-fingers.usda"),
            ("adapter_sha256", "outputs/compact-mount-001/adapter.usda"),
        ]:
            if hashlib.sha256((out / rel).read_bytes()).hexdigest() != report["tool"][key]:
                raise ValueError("Recorded asset differs: " + rel)
    else:
        raise ValueError("Need a report to establish original case and timestep")
    (out / "run.sh").write_text(
        '#!/usr/bin/env bash\nset -euo pipefail\ncd -- "$(dirname -- "$0")"\n'
        'run_name="${1:-rerun-001}"\n'
        '[[ "$run_name" =~ ^[a-zA-Z0-9_-]+$ ]] || { echo "Use a simple fresh run name"; exit 2; }\n'
        'test ! -e "outputs/$run_name"\n'
        "docker run --rm --gpus all --shm-size=4gb -e ACCEPT_EULA=Y -e OMNI_KIT_ACCEPT_EULA=YES "
        '-e PRIVACY_CONSENT=N -v "$PWD:/workspace" -w /workspace '
        '"${FFC_RUNTIME_IMAGE:-ffc-mounted-rerun:6.1.0}" scripts/isaac_fr3_flexible_lift.py '
        f'--case {case} --dt {dt}{servo_option} --output "/workspace/outputs/$run_name"\n'
    )
    (out / "run.sh").chmod(0o755)
    (out / "README.md").write_text(
        f"# Frozen rerun bundle: {run.name}\n\n"
        "Source snapshots and CAD/robot assets are included. Requires NVIDIA Docker GPU support and "
        "acceptance of the Isaac Sim licence. No credentials or camera streams are included.\n\n"
        "```bash\ndocker build -t ffc-mounted-rerun:6.1.0 .\nbash run.sh rerun-001\n```\n\n"
        "On the original lab machine, reuse its recorded runtime:\n\n"
        "```bash\nFFC_RUNTIME_IMAGE=ffc-isaac:6.1.0 bash run.sh rerun-001\n```\n\n"
        "A recorded failure should remain a failure. Inspect report.json, failure.txt and sensor-trace.json; "
        "Container exit status alone is insufficient. Physics reruns differ from USD playback. "
        "Build recipe pins the Isaac base and Python packages but OS packages/driver are not fully hermetic. "
        "Bitwise equality across GPUs is not established. Supplier ownership is unchanged.\n"
    )
    image_id = subprocess.check_output(
        ["docker", "image", "inspect", "ffc-isaac:6.1.0", "--format", "{{.Id}}"], text=True
    ).strip()
    manifest = {
        "source_run": run.name,
        "case": case,
        "dt_s": dt,
        "packaging_host_runtime_image_id": image_id,
        "fresh_build_tested": False,
        "rerun_tested": False,
        "sha256": {
            str(p.relative_to(out)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(out.rglob("*"))
            if p.is_file()
        },
    }
    (out / "bundle-manifest.json").write_text(json.dumps(manifest, indent=2))
    with tarfile.open(str(out) + ".tar.gz", "w:gz") as tar:
        tar.add(out, arcname=out.name)
    print(json.dumps({"bundle": str(out) + ".tar.gz", "files": len(manifest["sha256"])}))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("run", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    bundle(a.run, a.output)
