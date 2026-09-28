"""Stationary FR3 compact-tool review; prescribed empty-jaw motion, no physics."""

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--candidate", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--gui", action="store_true")
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": not a.gui,
            "hide_ui": not a.gui,
            "renderer": "RaytracedLighting",
            "anti_aliasing": 2,
            "extra_args": ["--allow-root", "--/telemetry/enableAnonymousData=false"],
        }
    )
    import numpy as np
    import omni.replicator.core as rep
    import omni.timeline
    import omni.usd
    from PIL import Image, ImageDraw, ImageFont
    from pxr import Gf, UsdGeom

    from ffc.isaac_scene import camera

    ctx = omni.usd.get_context()
    ctx.open_stage(str(a.candidate / "mounted-candidate.usda"))
    for _ in range(15):
        app.update()
    stage = ctx.get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    report = json.loads((a.candidate / "mount-report.json").read_text())
    path = report["mount_path"]
    camera(stage, "/World/Cameras/CompactCell", (1.35, -1.22, 1.08), (0.38, -0.06, 0.36), 36)
    camera(stage, "/World/Cameras/CompactDetail", (0.82, -0.34, 0.30), (0.6436, -0.084, 0.1532), 55)
    camera(stage, "/World/Cameras/CompactBracket", (0.74, 0.015, 0.265), (0.6436, -0.068, 0.158), 48)
    lower = UsdGeom.Xformable(stage.GetPrimAtPath(path + "/Gripper/lower")).GetOrderedXformOps()[0]
    upper = UsdGeom.Xformable(stage.GetPrimAtPath(path + "/Gripper/upper")).GetOrderedXformOps()[0]
    timeline = omni.timeline.get_timeline_interface()
    timeline.pause()
    streams = []
    names = ["CompactCell", "CompactDetail", "CompactBracket"]
    for name in names:
        rp = rep.create.render_product("/World/Cameras/" + name, (800, 700))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach(rp)
        streams.append((rp, rgb))

    def pose(gap):
        d = (11.4 - gap) / 2000
        lower.Set(Gf.Vec3d(0, 0, d))
        upper.Set(Gf.Vec3d(0, 0, -d))

    def capture():
        rep.orchestrator.step(delta_time=0, rt_subframes=4, pause_timeline=True)
        if timeline.is_playing() or timeline.get_current_time() != 0:
            raise RuntimeError("Unexpected physics advancement")
        return [Image.fromarray(np.asarray(rgb.get_data())[..., :3].astype(np.uint8)) for _, rgb in streams]

    pose(11)
    for _ in range(3):
        capture()
    for name, img in zip(names, capture(), strict=True):
        img.save(a.output / (name + ".png"))
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 21)
    process = subprocess.Popen(
        [
            "ffmpeg",
            "-y",
            "-loglevel",
            "error",
            "-f",
            "rawvideo",
            "-pixel_format",
            "rgb24",
            "-video_size",
            "1600x800",
            "-framerate",
            "12",
            "-i",
            "-",
            "-an",
            "-c:v",
            "libx264",
            "-preset",
            "fast",
            "-crf",
            "20",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(a.output / "mounted-review.mp4"),
        ],
        stdin=subprocess.PIPE,
    )
    trace = []
    try:
        for label, begin, end in [("Open", 11, 11), ("Close empty fingers", 11, 1.14), ("Reopen", 1.14, 11)]:
            for i in range(24):
                t = i / 23
                gap = begin + (end - begin) * t * t * (3 - 2 * t)
                pose(gap)
                images = capture()
                frame = Image.new("RGB", (1600, 800), (18, 23, 30))
                frame.paste(images[0], (0, 100))
                frame.paste(images[1], (800, 100))
                draw = ImageDraw.Draw(frame)
                draw.text(
                    (20, 12),
                    f"Compact PGEA on FR3 | {label} | pad gap {gap - 1:.2f} mm",
                    font=font,
                    fill="white",
                )
                draw.text(
                    (20, 47),
                    "Stationary robot. Prescribed jaw motion. "
                    "No grasp, insertion or physical mounting validation.",
                    font=font,
                    fill=(230, 188, 103),
                )
                process.stdin.write(frame.tobytes())
                trace.append({"frame": len(trace), "base_gap_mm": gap})
            print("MOUNT_REVIEW_PHASE_DONE", label, flush=True)
        process.stdin.close()
        if process.wait() != 0:
            raise RuntimeError("ffmpeg failed")
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
    # The session carries cameras/pose only; the original workcell is untouched.
    session = stage.GetSessionLayer().ExportToString()
    (a.output / "review-session.usda").write_text(session)
    (a.output / "render-report.json").write_text(
        json.dumps(
            {
                "physics_steps": 0,
                "robot_pose_changed": False,
                "frames": len(trace),
                "fps": 12,
                "trace": trace,
            },
            indent=2,
        )
        + "\n"
    )
    for f in a.output.iterdir():
        if f.is_file():
            os.chmod(f, 0o644)
    for rp, rgb in streams:
        rgb.detach(rp)
        rp.destroy()
    print("MOUNTED_COMPACT_REVIEW_READY", flush=True)
    if a.gui:
        import omni.ui as ui
        from omni.kit.viewport.utility import get_active_viewport

        viewport = get_active_viewport()
        viewport.set_active_camera("/World/Cameras/CompactDetail")
        panel = ui.Window("Compact tool on FR3", width=370, height=180)
        with panel.frame:
            with ui.VStack(spacing=6):
                ui.Label("Stationary geometry review; old tool removed")
                for label, name in [
                    ("Whole workcell", "CompactCell"),
                    ("Mounted compact tool", "CompactDetail"),
                    ("Adapter close-up", "CompactBracket"),
                ]:
                    ui.Button(
                        label,
                        clicked_fn=lambda name=name: viewport.set_active_camera("/World/Cameras/" + name),
                    )
        while app.is_running():
            app.update()
    app.close()


if __name__ == "__main__":
    main()
