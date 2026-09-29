"""Archive and review recorded-state macro RGB; never authorize motion."""

import argparse
import hashlib
import html
import json
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from pxr import Sdf, Usd, UsdPhysics, UsdUtils

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from ffc.camera_observations import CameraGate, CameraSpec  # noqa: E402
from ffc.held_end_inspection import InspectionConfig, inspect  # noqa: E402


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--ffmpeg", required=True)
    args = parser.parse_args()
    source, output = args.capture, args.output
    manifest = json.loads((source / "manifest.json").read_text())
    calibration = json.loads((source / "calibration.json").read_text())
    if not manifest["complete"] or len(manifest["frames"]) != manifest["expected_frames"]:
        raise ValueError("Incomplete capture")
    identity = dict(calibration)
    calibration_id = identity.pop("calibration_id")
    if hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest() != calibration_id:
        raise ValueError("Calibration identity mismatch")
    width, height = calibration["resolution"]
    gate = CameraGate({calibration["camera_id"]: CameraSpec(width, height, calibration_id)})
    output.mkdir(parents=True, exist_ok=False)
    for name in [
        "manifest.json",
        "calibration.json",
        "capture-config.json",
        "optics-profile.json",
        "capture-source.py",
    ]:
        shutil.copy2(source / name, output / name)
    shutil.copy2(__file__, output / "review-source.py")
    results, cards = [], []
    for row in manifest["frames"]:
        path = source / row["file"]
        if digest(path) != row["sha256"] or row["clock"] != "recorded_simulation_seconds":
            raise ValueError("Frame identity/clock mismatch")
        pixels = np.asarray(Image.open(path).convert("RGB"))
        if pixels.shape != (height, width, 3):
            raise ValueError("Image dimensions disagree with calibration")
        packet = {key: row[key] for key in ["sequence", "timestamp_s", "camera_id", "calibration_id"]}
        packet.update(width=width, height=height, rgb=pixels.tobytes())
        # Offline delivery in the recorded clock; this does not measure runtime latency.
        frame = gate.accept(packet, row["timestamp_s"])
        result, terminal, tool = inspect(frame, InspectionConfig(calibration_id))
        result["clock"] = row["clock"]
        results.append(result)
        shutil.copy2(path, output / path.name)
        painted = pixels.astype(float)
        painted[terminal] = painted[terminal] * 0.55 + np.array([0, 210, 255]) * 0.45
        painted[tool] = painted[tool] * 0.7 + np.array([255, 140, 0]) * 0.3
        overlay = Image.fromarray(painted.astype(np.uint8))
        overlay.save(output / f"overlay-{row['sequence']:04}.png")
        Image.fromarray(terminal.astype(np.uint8) * 255).save(output / f"terminal-{row['sequence']:04}.png")
        Image.fromarray(tool.astype(np.uint8) * 255).save(output / f"tool-{row['sequence']:04}.png")
        panel = Image.new("RGB", (1224, 570), "#162b27")
        panel.paste(Image.fromarray(pixels).resize((612, 512)), (0, 58))
        panel.paste(overlay.resize((612, 512)), (612, 58))
        draw = ImageDraw.Draw(panel)
        draw.text(
            (15, 10), f"Recorded time {row['timestamp_s']:.2f} s | RGB / appearance candidates", fill="white"
        )
        draw.text(
            (15, 30),
            "Sampled replay montage; no new physics, verified edge or motion permission",
            fill="white",
        )
        panel.save(output / f"panel-{row['sequence']:04}.png")
        reasons = html.escape(", ".join(result["reasons"]))
        cards.append(
            f"<article><h2>{row['timestamp_s']:.2f} s</h2><p>{reasons}</p>"
            f'<div class="pair"><a href="{path.name}"><img src="{path.name}" alt="Raw RGB at '
            f'{row["timestamp_s"]} seconds" loading="lazy"></a><img src="overlay-{row["sequence"]:04}.png" '
            'alt="Colour-region diagnostic overlay" loading="lazy"></div></article>'
        )
    subprocess.run(
        [
            args.ffmpeg,
            "-y",
            "-framerate",
            "1",
            "-i",
            str(output / "panel-%04d.png"),
            "-c:v",
            "libx264",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(output / "review.mp4"),
        ],
        check=True,
        capture_output=True,
    )
    stage = Usd.Stage.Open(str(source / "inspection-scene.usda"))
    if not stage.GetPrimAtPath("/World/Cameras/HeldEndInspection"):
        raise ValueError("Inspection camera absent")
    for prim in stage.Traverse():
        if (
            prim.HasAPI(UsdPhysics.RigidBodyAPI)
            and UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Get()
        ):
            raise ValueError("Playback physics enabled")
    stage.Flatten().Export(str(output / "inspection.usdc"))
    if not UsdUtils.CreateNewUsdzPackage(
        Sdf.AssetPath(str(output / "inspection.usdc")), str(output / "inspection.usdz")
    ):
        raise RuntimeError("Replay packaging failed")
    replay = Usd.Stage.Open(str(output / "inspection.usdz"))
    if not replay.GetPrimAtPath("/World/Cameras/HeldEndInspection"):
        raise ValueError("Packaged camera missing")
    (output / "inspection.usdc").unlink()
    report = {
        "scope": "Seven samples of one recorded lift; no held-out evaluation or new physics",
        "motion_permitted": False,
        "calibration_kind": calibration["calibration_kind"],
        "results": results,
        "review_findings": [
            "Free-end contacts visible; full terminal clips near final pose",
            "Focus varies through lift; mid-stroke appears sharper",
            "Tool reference is cropped; colour regions cannot verify slip",
            "Articulated ribbon surface seams remain visible",
            "Camera mount clearance and physical optics are unqualified",
        ],
        "sha256": {p.name: digest(p) for p in output.iterdir() if p.is_file()},
    }
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    (output / "index.html").write_text(
        """<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Held cable · Isaac camera review</title>
<style>body{font:17px/1.65 system-ui;background:#f4f3ed;color:#253c37;max-width:1100px;margin:auto;
padding:40px 24px}h1{font-size:clamp(32px,5vw,56px);line-height:1.1;letter-spacing:-.04em}
a{color:#176b57}article{border-top:1px solid #bbc8c0;padding:20px 0}img,video{width:100%}
.pair{display:grid;grid-template-columns:1fr 1fr;gap:12px}
@media(max-width:650px){.pair{grid-template-columns:1fr}}</style>
<a href="../../library/#mounted">← Motion library</a><h1>The camera can see the held end.</h1>
<p>Isaac has now rendered the proposed inspection view. All seven raw frames are below, beside an RGB-only
colour diagnostic. Click a raw image for full resolution. These are samples from one recorded lift,
not independent test scenes. The inspection policy still permits no motion.</p>
<video controls playsinline preload="metadata" poster="panel-0003.png" src="review.mp4"></video>
<p>The montage holds each sample for one second; its playback speed is not the robot's speed.
The source is the recorded mounted lift, not a fresh physics run.</p>
<h2>What this view tells us</h2><p>The contacts and free edge are visible. Focus is better around mid-stroke
than at the beginning or end. By the final pose, the upper terminal region reaches the image boundary;
the yellow tool is cropped throughout. Visible seams also expose the segmented cable geometry.
This is useful camera-placement evidence, but it does not establish accurate edge pose or realistic
material appearance.</p>
<p>The Basler ace 2 a2A2448-75ucBAS / Kowa LM35JC10M reference is rendered at 1224 × 1024,
45° elevation, with a 127.1 mm principal-plane focus distance and equivalent renderer f/11.04
(physical f/8 target). Calibration is synthetic; lens distortion is idealized and radiometry,
mount clearance and physical camera performance are unqualified.</p>
<h2>Next decision</h2><p>Use a repeatable inspection pose with focus and framing set for the held end.
Then independently label the leading edge, visibility and terminal side before evaluating a learned
estimate. Colour regions are diagnostic candidates; they cannot establish slip or authorize insertion.</p>
<p><a href="calibration.json">Synthetic calibration</a> · <a href="manifest.json">Capture manifest</a> ·
<a href="report.json">Measurements and hashes</a> · <a href="inspection.usdz">Native Isaac replay</a> ·
<a href="../historical.html">Earlier colour-baseline failure</a></p>
<p>For native playback, open the USDZ, select /World/Cameras/HeldEndInspection and scrub the timeline.
Physics is disabled. The exported scene was rendered in Isaac; the USDZ package was reopened with USD.</p>
<h2>Every captured view</h2><p>Left: raw camera RGB. Right: blue and yellow appearance candidates.
Neither overlay is a simulator label or verified insertion target.</p>"""
        + "".join(cards)
        + "</html>"
    )
    print(json.dumps({"frames": len(results), "output": str(output), "motion_permitted": False}))


if __name__ == "__main__":
    main()
