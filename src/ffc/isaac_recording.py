"""Record actual RTX frames and experiment state to a portable MP4."""

from __future__ import annotations

import json
import re
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from pxr import Gf, UsdGeom


class Recorder:
    """Two camera views: the complete cell and a close view following the tool."""

    def __init__(
        self,
        stage,
        destination: Path,
        fps: int = 30,
        scope_label: str = "Physics experiment: uncalibrated cable and contact parameters",
    ):
        self.scope_label = scope_label
        import omni.replicator.core as rep

        from ffc.isaac_scene import camera

        self.stage = stage
        self.destination, self.fps = destination, fps
        self.phases = []
        self.closed = False
        self.detail = camera(stage, "/World/Cameras/Action", (0.4, -0.4, 0.3), (0.4, -0.18, 0.02), 30)
        self.products, self.annotators = [], []
        for path in ("/World/Cameras/Overview", "/World/Cameras/Action"):
            product = rep.create.render_product(path, (640, 480))
            annotator = rep.AnnotatorRegistry.get_annotator("rgb")
            annotator.attach(product)
            self.products.append(product)
            self.annotators.append(annotator)
        self.stderr = destination.with_suffix(".ffmpeg.log").open("w")
        self.process = subprocess.Popen(
            [
                "ffmpeg",
                "-y",
                "-loglevel",
                "warning",
                "-f",
                "rawvideo",
                "-pixel_format",
                "rgb24",
                "-video_size",
                "1280x480",
                "-framerate",
                str(fps),
                "-i",
                "-",
                "-an",
                "-c:v",
                "libx264",
                "-preset",
                "veryfast",
                "-crf",
                "20",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                str(destination),
            ],
            stdin=subprocess.PIPE,
            stderr=self.stderr,
        )
        self.frames = 0
        # Initialize sensors before the timed experiment; never silently omit
        # startup frames from the recorded experiment.
        for _ in range(12):
            rep.orchestrator.step(delta_time=0.0, pause_timeline=False)
            if all(np.asarray(a.get_data()).size for a in self.annotators):
                break
        else:
            raise RuntimeError("RTX cameras failed to initialize")

    def aim(self, target, eye_offset=None):
        target = np.asarray(target)
        eye = target + (np.asarray(eye_offset) if eye_offset is not None else [-0.055, -0.045, 0.045])
        matrix = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1))
        UsdGeom.Xformable(self.detail).GetOrderedXformOps()[0].Set(matrix.GetInverse())

    def write(self, phase: str, sim_time: float):
        import omni.replicator.core as rep

        # Explicit capture is required by the headless Replicator pipeline.
        # A zero delta preserves the physics state and simulation timestamp.
        rep.orchestrator.step(delta_time=0.0, pause_timeline=False)
        frames = [np.asarray(a.get_data()) for a in self.annotators]
        if any(f.size == 0 for f in frames):
            raise RuntimeError("RTX camera returned an empty frame during recording")
        if not self.phases or self.phases[-1]["phase"] != phase:
            self.phases.append(
                {"phase": phase, "first_frame": self.frames, "frames": 0, "first_simulation_time_s": sim_time}
            )
        self.phases[-1]["frames"] += 1
        frame = np.concatenate([f[..., :3] for f in frames], axis=1).astype(np.uint8)
        img = Image.fromarray(frame)
        draw = ImageDraw.Draw(img)
        draw.rectangle((0, 0, 1280, 44), fill=(15, 20, 25))
        draw.text((12, 8), f"FR3 / FFC   |   {phase}   |   simulation {sim_time:.2f} s", fill="white")
        draw.text((12, 25), self.scope_label, fill=(220, 190, 100))
        if self.frames % self.fps == 0 or self.phases[-1]["frames"] == 1:
            img.save(self.destination.parent / "latest-frame.jpg", quality=90)
            (self.destination.parent / "recording-progress.json").write_text(
                json.dumps(
                    {
                        "phase": phase,
                        "simulation_time_s": sim_time,
                        "frames": self.frames + 1,
                    },
                    indent=2,
                )
            )
        self.process.stdin.write(np.asarray(img).tobytes())
        self.frames += 1

    def close(self):
        if self.closed:
            return
        self.closed = True
        for annotator, product in zip(self.annotators, self.products, strict=True):
            annotator.detach(product)
            product.destroy()
        self.process.stdin.close()
        code = self.process.wait(timeout=60)
        self.stderr.close()
        if code:
            raise RuntimeError(f"Video encoding failed: ffmpeg exit {code}")
        self.destination.with_suffix(".phases.json").write_text(json.dumps(self.phases, indent=2))
        if not self.frames:
            return
        clips = self.destination.parent / "stage-clips"
        clips.mkdir(exist_ok=True)
        for i, phase in enumerate(self.phases):
            name = re.sub(r"[^a-zA-Z0-9_-]", "_", phase["phase"])
            subprocess.run(
                [
                    "ffmpeg",
                    "-y",
                    "-loglevel",
                    "error",
                    "-i",
                    str(self.destination),
                    "-ss",
                    str(phase["first_frame"] / self.fps),
                    "-t",
                    str(phase["frames"] / self.fps),
                    "-an",
                    "-c:v",
                    "libx264",
                    "-preset",
                    "veryfast",
                    "-crf",
                    "20",
                    "-pix_fmt",
                    "yuv420p",
                    "-movflags",
                    "+faststart",
                    str(clips / f"{i:02d}-{name}.mp4"),
                ],
                check=True,
                timeout=60,
            )
