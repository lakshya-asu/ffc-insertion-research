"""Human-only stationary macro preview; no model inputs, ROS or robot control."""

import argparse
import signal
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--stage", type=Path, required=True)
    p.add_argument("--seconds", type=float, default=43200)
    p.add_argument("--ros", action="store_true", help="Publish native RGB and fixed calibration via ROS 2")
    a = p.parse_args()
    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": True,
            "renderer": "RaytracedLighting",
            "anti_aliasing": 2,
            "extra_args": ["--allow-root", "--/telemetry/enableAnonymousData=false"],
        }
    )
    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    streams = []
    ros_camera = None
    try:
        import numpy as np
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        from isaacsim.core.utils.extensions import enable_extension
        from pxr import UsdGeom

        from ffc.isaac_scene import camera
        from ffc.lab_feed import publish_snapshot

        if a.ros:
            enable_extension("isaacsim.ros2.bridge")
            for _ in range(8):
                app.update()
        context = omni.usd.get_context()
        context.open_stage(str(a.stage))
        for _ in range(8):
            app.update()
        stage = context.get_stage()
        stage.SetEditTarget(stage.GetSessionLayer())
        camera(stage, "/World/Cameras/MacroCell", (1.4, -1.4, 1.25), (0.3, -0.02, 0.42), 28)
        mounted = bool(stage.GetPrimAtPath("/World/Cameras/MountMacro"))
        envelope = UsdGeom.Imageable(
            stage.GetPrimAtPath(
                "/World/Hardware/MountMacroEnvelope" if mounted else "/World/Hardware/MacroEnvelope"
            )
        )
        for name, path, size in [
            ("board", "MountMacro" if mounted else "Macro", (2448, 2048) if a.ros else (1224, 1024)),
            ("desk", "MountReview" if mounted else "MacroOverview", (960, 720)),
            ("workcell", "MacroCell", (960, 640)),
        ]:
            rp = rep.create.render_product("/World/Cameras/" + path, size)
            rgb = rep.AnnotatorRegistry.get_annotator("rgb")
            rgb.attach(rp)
            streams.append((name, rp, rgb))
        if a.ros:
            from isaac_ros_camera import RosCamera

            cam = UsdGeom.Camera.Get(
                stage, "/World/Cameras/MountMacro" if mounted else "/World/Cameras/Macro"
            )
            fx = cam.GetFocalLengthAttr().Get() / cam.GetHorizontalApertureAttr().Get() * 2448
            fy = cam.GetFocalLengthAttr().Get() / cam.GetVerticalApertureAttr().Get() * 2048
            k = np.array([[fx, 0, 1224], [0, fy, 1024], [0, 0, 1]])
            world_camera = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(cam.GetPrim())).T
            world_optical = world_camera @ np.diag([1.0, -1.0, -1.0, 1.0])
            ros_camera = RosCamera(k, world_optical)
        timeline = omni.timeline.get_timeline_interface()
        timeline.pause()
        initial, start, sequence = timeline.get_current_time(), time.monotonic(), 0
        feed = ROOT / "outputs/lab-live/feed"
        while (
            running and time.monotonic() - start < a.seconds and not (feed.parent / "stop-preview").exists()
        ):
            for is_macro in [True, False]:
                (envelope.MakeInvisible if is_macro else envelope.MakeVisible)()
                rep.orchestrator.step(delta_time=0, rt_subframes=4, pause_timeline=True)
                if "STEPPING" in str(rep.orchestrator.get_status()):
                    raise RuntimeError("Capture did not complete; refusing to timestamp a stale frame")
                if timeline.is_playing() or timeline.get_current_time() != initial:
                    raise RuntimeError("Static preview advanced physics")
                for name, _, rgb in streams:
                    if (name == "board") != is_macro:
                        continue
                    arr = np.asarray(rgb.get_data())[..., :3].astype(np.uint8)
                    if is_macro and ros_camera is not None:
                        ros_camera.publish(arr)
                    publish_snapshot(
                        arr,
                        feed,
                        name,
                        sequence,
                        "Zero 2 W macro study. Static scene, 0.3 mm gap; no robot motion. "
                        + (
                            "Basler/Kowa framing with uncalibrated finite-aperture blur; "
                            "native RGB when ROS is enabled."
                            if is_macro
                            else "Camera and lens are dimensioned placement envelopes."
                        ),
                        "Isaac Sim live static macro preview",
                    )
            sequence += 1
            time.sleep(0.5)
    finally:
        if ros_camera is not None:
            ros_camera.close()
        for _, rp, rgb in streams:
            rgb.detach()
            rp.destroy()
        app.close()


if __name__ == "__main__":
    main()
