"""Isaac rendering of isolated handoff clearance studies; zero physics steps."""

import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--stage", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    (a.output / "source.py").write_bytes(Path(__file__).read_bytes())
    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": True,
            "renderer": "RaytracedLighting",
            "extra_args": ["--allow-root", "--/telemetry/enableAnonymousData=false"],
        }
    )
    import numpy as np
    import omni.replicator.core as rep
    import omni.timeline
    import omni.usd
    from PIL import Image, ImageDraw, ImageFont
    from pxr import Gf, UsdGeom, UsdLux

    from ffc.isaac_scene import camera

    ctx = omni.usd.get_context()
    ctx.open_stage(str(a.stage))
    stage = ctx.get_stage()
    timeline = omni.timeline.get_timeline_interface()
    timeline.pause()
    UsdLux.DomeLight.Define(stage, "/World/Light").CreateIntensityAttr(1100)
    streams = []
    for branch, x in [("Fixture", -0.16), ("Integrated", 0.16)]:
        path = f"/World/Cameras/{branch}"
        camera(stage, path, (x + 0.12, -0.10, 0.13), (x - 0.015, 0.035, 0.022), 48)
        rp = rep.create.render_product(path, (800, 600))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach(rp)
        streams.append((rp, rgb))
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 20)
    process = subprocess.Popen(
        [
            "ffmpeg",
            "-y",
            "-loglevel",
            "error",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "rgb24",
            "-video_size",
            "1600x700",
            "-framerate",
            "12",
            "-i",
            "-",
            "-an",
            "-c:v",
            "libx264",
            "-pix_fmt",
            "yuv420p",
            "-crf",
            "20",
            "-movflags",
            "+faststart",
            str(a.output / "review.mp4"),
        ],
        stdin=subprocess.PIPE,
    )
    trace = []
    try:
        for frame_index in range(72):
            t = frame_index / 71
            x_mm = -45 + 34.5 * min(t * 3, 1)
            gap = 11 - 9.86 * max(0, min((t - 1 / 3) * 3, 1))
            placement = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Fixture/ToolPlacement"))
            placement.GetOrderedXformOps()[0].Set(Gf.Vec3d(x_mm / 1000, 0.012, 0.03007))
            for group, sign in [("lower", 1), ("upper", -1)]:
                path = "/World/Fixture/ToolPlacement/Tool/" + group
                UsdGeom.Xformable(stage.GetPrimAtPath(path)).GetOrderedXformOps()[0].Set(
                    Gf.Vec3d(0, 0, sign * (11.4 - gap) / 2000)
                )
            cup = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Integrated/CupEnvelope"))
            cup.GetOrderedXformOps()[0].Set(Gf.Vec3d(0, 0.026, 0.00964 + 0.030 * t))
            rep.orchestrator.step(delta_time=0, rt_subframes=4, pause_timeline=True)
            if timeline.is_playing() or timeline.get_current_time() != 0:
                raise RuntimeError("Geometry review advanced physics time")
            frame = Image.new("RGB", (1600, 700), (20, 25, 32))
            draw = ImageDraw.Draw(frame)
            draw.text((18, 10), "Fixture: sideways approach and geometric closure", font=font, fill="white")
            draw.text(
                (818, 10), "Integrated: required cup travel; cable stays on desk", font=font, fill="white"
            )
            draw.text(
                (18, 45),
                "Prescribed layout study. No vacuum, deformation, grasp or insertion physics.",
                font=font,
                fill=(242, 196, 107),
            )
            for i, (_, rgb) in enumerate(streams):
                data = np.asarray(rgb.get_data())
                if data.shape[:2] != (600, 800):
                    raise RuntimeError("Missing render")
                frame.paste(Image.fromarray(data[:, :, :3].astype(np.uint8)), (i * 800, 100))
            process.stdin.write(frame.tobytes())
            if frame_index in [0, 35, 71]:
                frame.save(a.output / f"frame-{frame_index:03d}.png")
                print(f"HANDOFF_REVIEW_FRAME {frame_index}", flush=True)
            trace.append(
                {"frame": frame_index, "fixture_x_mm": x_mm, "base_gap_mm": gap, "cup_retraction_mm": 30 * t}
            )
        process.stdin.close()
        if process.wait() != 0:
            raise RuntimeError("Video encoding failed")
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
    stage.GetRootLayer().Export(str(a.output / "review.usda"))
    (a.output / "report.json").write_text(json.dumps({"physics_steps": 0, "frames": trace}, indent=2))
    print("HANDOFF_REVIEW_COMPLETE", flush=True)
    app.close()


if __name__ == "__main__":
    main()
