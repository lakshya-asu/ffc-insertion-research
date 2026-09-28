"""Render the compact finger CAD study in Isaac; optionally leave a desktop viewer open."""

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
    p.add_argument("--cad", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--gui", action="store_true")
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": not args.gui,
            "hide_ui": not args.gui,
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
    from pxr import Gf, UsdGeom, UsdLux

    from ffc.isaac_scene import camera

    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, "Z")
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    support = UsdGeom.Xform.Define(stage, "/World/Review")
    support.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.04))
    tool = stage.DefinePrim("/World/Review/Tool")
    tool.GetReferences().AddReference(str(args.cad / "compact-fingers.usda"))
    floor = UsdGeom.Cube.Define(stage, "/World/Table")
    floor.CreateSizeAttr(1)
    floor.AddTranslateOp().Set(Gf.Vec3d(0, -0.04, -0.005))
    floor.AddScaleOp().Set(Gf.Vec3f(0.4, 0.4, 0.01))
    floor.CreateDisplayColorAttr([Gf.Vec3f(0.2, 0.23, 0.26)])
    light = UsdLux.DomeLight.Define(stage, "/World/Light")
    light.CreateIntensityAttr(900)
    camera(stage, "/World/Cameras/Assembly", (0.115, 0.105, 0.12), (0, -0.032, 0.043), 46)
    camera(stage, "/World/Cameras/Fingers", (0.040, 0.064, 0.060), (0, 0.008, 0.040), 55)
    lower = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Review/Tool/lower")).GetOrderedXformOps()[0]
    upper = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Review/Tool/upper")).GetOrderedXformOps()[0]
    timeline = omni.timeline.get_timeline_interface()
    timeline.pause()
    streams = []
    for name in ["Assembly", "Fingers"]:
        rp = rep.create.render_product(f"/World/Cameras/{name}", (800, 700))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach(rp)
        streams.append((rp, rgb))
    if args.gui:
        from omni.kit.viewport.utility import get_active_viewport

        get_active_viewport().set_active_camera("/World/Cameras/Assembly")

    def pose(gap):
        if not 1.14 <= gap <= 11:
            raise ValueError("Review pose outside prescribed range")
        offset = (11.4 - gap) / 2000
        lower.Set(Gf.Vec3d(0, 0, offset))
        upper.Set(Gf.Vec3d(0, 0, -offset))

    def capture():
        rep.orchestrator.step(delta_time=0, rt_subframes=4, pause_timeline=True)
        if timeline.is_playing() or timeline.get_current_time() != 0:
            raise RuntimeError("Review must not advance physics")
        return [Image.fromarray(np.asarray(rgb.get_data())[..., :3].astype(np.uint8)) for _, rgb in streams]

    pose(11)
    for _ in range(3):
        capture()
    for name, img in zip(["assembly-open", "fingers-open"], capture(), strict=True):
        img.save(args.output / f"{name}.png")
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 21)
    trace = []
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
            str(args.output / "finger-motion.mp4"),
        ],
        stdin=subprocess.PIPE,
    )
    try:
        phases = [
            ("Open", 11, 11, 12),
            ("Close to dimensional coupon", 11, 1.14, 36),
            ("Touching geometry only", 1.14, 1.14, 18),
            ("Reopen", 1.14, 11, 30),
        ]
        for label, start, end, count in phases:
            for i in range(count):
                t = i / max(1, count - 1)
                gap = start + (end - start) * t * t * (3 - 2 * t)
                pose(gap)
                images = capture()
                frame = Image.new("RGB", (1600, 800), (17, 23, 31))
                for x, img in zip([0, 800], images, strict=True):
                    frame.paste(img, (x, 100))
                draw = ImageDraw.Draw(frame)
                draw.text(
                    (22, 12),
                    f"PGEA compact fingers | {label} | pad gap {gap - 1:.2f} mm",
                    font=font,
                    fill="white",
                )
                draw.text(
                    (22, 46),
                    "Prescribed CAD motion. Fixed cable coupon. "
                    "No grasp, contact physics or insertion demonstrated.",
                    font=font,
                    fill=(230, 188, 103),
                )
                process.stdin.write(frame.tobytes())
                trace.append({"frame": len(trace), "base_gap_mm": gap, "pad_gap_mm": gap - 1})
            print(f"REVIEW_PHASE_DONE {label}", flush=True)
        process.stdin.close()
        if process.wait() != 0:
            raise RuntimeError("Video encoding failed")
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
    pose(1.14)
    for name, img in zip(["assembly-contact", "fingers-contact"], capture(), strict=True):
        img.save(args.output / f"{name}.png")
    stage.GetRootLayer().Export(str(args.output / "review.usda"))
    (args.output / "render-report.json").write_text(
        json.dumps(
            {
                "renderer": "Isaac Sim RTX RaytracedLighting",
                "physics_steps": 0,
                "frame_count": len(trace),
                "fps": 12,
                "trace": trace,
                "limitations": "Prescribed geometric motion; no grasp/contact validation; fixed coupon",
            },
            indent=2,
        )
        + "\n"
    )
    for path in args.output.iterdir():
        if path.is_file():
            os.chmod(path, 0o644)
    print("COMPACT_REVIEW_READY", flush=True)
    for rp, rgb in streams:
        rgb.detach(rp)
        rp.destroy()
    if args.gui:
        import omni.ui as ui
        from omni.kit.viewport.utility import get_active_viewport

        review_path = str(args.output / "review.usda")

        def load(path, camera_path):
            ctx.open_stage(path)
            get_active_viewport().set_active_camera(camera_path)
            timeline.pause()

        panel = ui.Window("Cable tool review", width=390, height=180)
        with panel.frame:
            with ui.VStack(spacing=8):
                ui.Label("Compact fingers: geometry review only")
                ui.Button(
                    "Compact fingers / close-up",
                    clicked_fn=lambda: load(review_path, "/World/Cameras/Fingers"),
                )
                ui.Button(
                    "Compact gripper / assembly",
                    clicked_fn=lambda: load(review_path, "/World/Cameras/Assembly"),
                )
                ui.Button(
                    "Existing Pi workcell",
                    clicked_fn=lambda: load(
                        str(ROOT / "outputs/mount-review-003/mounted-workcell.usda"),
                        "/World/Cameras/MountReview",
                    ),
                )
        get_active_viewport().set_active_camera("/World/Cameras/Fingers")
        while app.is_running():
            app.update()
    app.close()


if __name__ == "__main__":
    main()
