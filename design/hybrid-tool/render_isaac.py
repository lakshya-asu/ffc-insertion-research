"""Isaac RTX rendering of prescribed CAD motion. No physics or grasp claim."""

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--cad", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    shutil.copy2(__file__, a.output / "render_source.py")
    shutil.copy2(a.cad / "assembly.json", a.output / "input_assembly.json")
    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": True,
            "renderer": "RaytracedLighting",
            "anti_aliasing": 2,
            "extra_args": ["--allow-root", "--/telemetry/enableAnonymousData=false"],
        }
    )
    process = None
    try:
        import numpy as np
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        from PIL import Image, ImageDraw, ImageFont
        from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade

        from ffc.isaac_scene import camera

        ctx = omni.usd.get_context()
        ctx.new_stage()
        stage = ctx.get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, 1)
        UsdGeom.SetStageUpAxis(stage, "Z")
        root = UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(root.GetPrim())
        tool = UsdGeom.Xform.Define(stage, "/World/Tool").AddTranslateOp()
        groups = {
            g: UsdGeom.Xform.Define(stage, "/World/Tool/" + g).AddTranslateOp()
            for g in ("fixed", "carriage", "jaw", "deploy_rod")
        }
        report = json.loads((a.cad / "assembly.json").read_text())
        material_cache = {}

        def material(rgb):
            key = tuple(rgb)
            if key not in material_cache:
                path = f"/World/Materials/M{len(material_cache)}"
                m = UsdShade.Material.Define(stage, path)
                s = UsdShade.Shader.Define(stage, path + "/Shader")
                s.CreateIdAttr("UsdPreviewSurface")
                s.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
                s.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.48)
                m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(), "surface")
                material_cache[key] = m
            return material_cache[key]

        for part in report["parts"]:
            raw = (a.cad / "parts" / (part["name"] + ".stl")).read_bytes()
            n = int.from_bytes(raw[80:84], "little")
            dtype = np.dtype([("normal", "<f4", (3,)), ("v", "<f4", (3, 3)), ("attr", "<u2")])
            data = np.frombuffer(raw, dtype=dtype, count=n, offset=84)
            mesh = UsdGeom.Mesh.Define(stage, f"/World/Tool/{part['group']}/{part['name']}")
            mesh.CreatePointsAttr((data["v"].reshape(-1, 3) / 1000).tolist())
            mesh.CreateFaceVertexCountsAttr([3] * n)
            mesh.CreateFaceVertexIndicesAttr(list(range(3 * n)))
            mesh.CreateSubdivisionSchemeAttr("none")
            mesh.CreateDoubleSidedAttr(True)
            UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material(part["color"]))

        def cube(path, xyz, size, rgb):
            c = UsdGeom.Cube.Define(stage, path)
            c.CreateSizeAttr(1)
            c.AddTranslateOp().Set(Gf.Vec3d(*xyz))
            c.AddScaleOp().Set(Gf.Vec3f(*size))
            UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(material(rgb))
            return c

        cube("/World/Table", (0, -0.05, -0.006), (0.32, 0.36, 0.012), (0.62, 0.65, 0.64))
        cable = UsdGeom.Mesh.Define(stage, "/World/CableIllustration")
        cable.CreateSubdivisionSchemeAttr("none")
        cable.CreateDoubleSidedAttr(True)
        UsdShade.MaterialBindingAPI.Apply(cable.GetPrim()).Bind(material((0.08, 0.09, 0.10)))
        # A prescribed strip illustrates clearance only. It is not a deformable solve.
        ys = np.linspace(-0.188, 0.012, 101)
        faces = []
        for i in range(100):
            faces.extend(
                [
                    4 * i,
                    4 * i + 1,
                    4 * i + 5,
                    4 * i + 4,
                    4 * i + 2,
                    4 * i + 6,
                    4 * i + 7,
                    4 * i + 3,
                    4 * i,
                    4 * i + 4,
                    4 * i + 6,
                    4 * i + 2,
                    4 * i + 1,
                    4 * i + 3,
                    4 * i + 7,
                    4 * i + 5,
                ]
            )
        faces.extend([0, 2, 3, 1, 400, 401, 403, 402])
        cable.CreateFaceVertexCountsAttr([4] * (402))
        cable.CreateFaceVertexIndicesAttr(faces)
        header = cube(
            "/World/IllustrativeHeader", (0, 0.009, 0.00015), (0.0115, 0.006, 0.0003), (0.12, 0.32, 0.64)
        )
        header_op = header.GetOrderedXformOps()[0]

        light = UsdLux.DomeLight.Define(stage, "/World/Light")
        light.CreateIntensityAttr(650)
        light.CreateColorAttr(Gf.Vec3f(0.95, 0.97, 1))
        camera(stage, "/World/Cameras/Overview", (-0.29, 0.32, 0.22), (-0.015, -0.03, 0.07), 42)
        camera(stage, "/World/Cameras/Detail", (-0.085, 0.115, 0.045), (0, -0.007, 0.027), 48)
        camera(stage, "/World/Cameras/Hero", (-0.30, 0.36, 0.24), (-0.015, -0.035, 0.075), 48)
        streams = []
        for path, size in [("Overview", (800, 700)), ("Detail", (800, 700)), ("Hero", (1400, 1100))]:
            rp = rep.create.render_product("/World/Cameras/" + path, size)
            rgb = rep.AnnotatorRegistry.get_annotator("rgb")
            rgb.attach(rp)
            streams.append((rp, rgb))
        timeline = omni.timeline.get_timeline_interface()
        timeline.pause()
        start_time = timeline.get_current_time()
        fontfile = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
        font = ImageFont.truetype(fontfile, 22)
        small = ImageFont.truetype(fontfile, 16)

        def pose(d, e, lift):
            if not (0 <= d <= 20 and 0 <= e <= 20 and lift >= 0):
                raise ValueError("Requested pose exceeds actuator stroke or lift bounds")
            tool.Set(Gf.Vec3d(0, 0, 0.00114 + lift))
            groups["fixed"].Set(Gf.Vec3d(0))
            groups["carriage"].Set(Gf.Vec3d((d - 18) / 1000, 0, 0))
            groups["jaw"].Set(Gf.Vec3d((d - 18) / 1000, 0, (8 - e) / 1000))
            groups["deploy_rod"].Set(Gf.Vec3d((d - 18) / 1000, 0, 0))
            points = []
            for y in ys:
                blend = np.clip((y + 0.10) / 0.08, 0, 1)
                blend = blend * blend * (3 - 2 * blend)
                z = lift * blend
                w = 0.016 if y < -0.161 else 0.0115
                points.extend(
                    [[-w / 2, y, z], [w / 2, y, z], [-w / 2, y, z + 0.00014], [w / 2, y, z + 0.00014]]
                )
            cable.GetPointsAttr().Set(points)
            header_op.Set(Gf.Vec3d(0, 0.009, lift + 0.00015))

        def capture():
            rep.orchestrator.step(delta_time=0, rt_subframes=4, pause_timeline=True)
            if timeline.is_playing() or timeline.get_current_time() != start_time:
                raise RuntimeError("Unexpected physics advancement")
            return [np.asarray(rgb.get_data())[..., :3].astype(np.uint8) for _, rgb in streams]

        pose(18, 8, 0.035)
        for _ in range(5):
            capture()
        frames = capture()
        Image.fromarray(frames[2]).save(a.output / "assembled.png")
        stage.GetRootLayer().Export(str(a.output / "assembly.usda"))
        groups["carriage"].Set(Gf.Vec3d(-0.045, 0, 0.025))
        groups["jaw"].Set(Gf.Vec3d(0.025, 0.025, -0.012))
        groups["deploy_rod"].Set(Gf.Vec3d(0.015, 0, 0))
        UsdGeom.Imageable(cable).MakeInvisible()
        UsdGeom.Imageable(header).MakeInvisible()
        for _ in range(3):
            frames = capture()
        Image.fromarray(frames[2]).save(a.output / "exploded.png")
        UsdGeom.Imageable(cable).MakeVisible()
        UsdGeom.Imageable(header).MakeVisible()
        phases = [
            ("Vacuum pickup | finger stowed above desk", (0, 0, 0), (0, 0, 0)),
            ("Lift 35 mm | prescribed cable shape", (0, 0, 0), (0, 0, 0.035)),
            ("Lower the parked finger", (0, 0, 0.035), (0, 18, 0.035)),
            ("Deploy underneath | 18 mm lateral travel", (0, 18, 0.035), (18, 18, 0.035)),
            ("Close to the ribbon | 10 mm return", (18, 18, 0.035), (18, 8, 0.035)),
            ("Pinch pose | retention is NOT simulated", (18, 8, 0.035), (18, 8, 0.035)),
            ("Open before withdrawing the finger", (18, 8, 0.035), (18, 18, 0.035)),
            ("Withdraw sideways", (18, 18, 0.035), (0, 18, 0.035)),
            ("Raise into the stowed position", (0, 18, 0.035), (0, 0, 0.035)),
            ("Lower and release | prescribed motion", (0, 0, 0.035), (0, 0, 0)),
        ]
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
                "1600x780",
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
                "21",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                str(a.output / "mechanism.mp4"),
            ],
            stdin=subprocess.PIPE,
        )
        trace = []
        for phase, (label, begin, end) in enumerate(phases):
            for i in range(24):
                t = i / 23
                t = t * t * (3 - 2 * t)
                d, e, lift = np.array(begin) + (np.array(end) - np.array(begin)) * t
                pose(float(d), float(e), float(lift))
                eye = Gf.Vec3d(-0.085, 0.115, 0.045 + lift)
                target = Gf.Vec3d(0, -0.007, 0.008 + lift)
                cam = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Cameras/Detail"))
                cam.GetOrderedXformOps()[0].Set(
                    Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0, 0, 1)).GetInverse()
                )
                frames = capture()
                image = Image.new("RGB", (1600, 780), (18, 28, 26))
                image.paste(Image.fromarray(frames[0]), (0, 80))
                image.paste(Image.fromarray(frames[1]), (800, 80))
                draw = ImageDraw.Draw(image)
                draw.text((20, 10), label, font=font, fill="white")
                draw.text(
                    (20, 44),
                    (
                        "KINEMATIC CAD REVIEW | Actual PQ12 geometry | "
                        "No contact, vacuum-flow or force simulation"
                    ),
                    font=small,
                    fill=(225, 190, 113),
                )
                process.stdin.write(np.asarray(image).tobytes())
                trace.append(
                    dict(
                        frame=len(trace),
                        phase=label,
                        deploy_mm=float(d),
                        clamp_extension_mm=float(e),
                        lift_mm=float(lift * 1000),
                    )
                )
                if i == 23:
                    image.save(a.output / f"phase-{phase:02d}.jpg")
            print(f"Rendered phase {phase + 1}/{len(phases)}: {label}", flush=True)
        process.stdin.close()
        if process.wait() != 0:
            raise RuntimeError("Video encoding failed")
        process = None
        (a.output / "review.json").write_text(
            json.dumps(
                {
                    "status": "RENDERED_KINEMATIC_REVIEW",
                    "input_cad_manifest_sha256": hashlib.sha256(
                        (a.cad / "assembly.json").read_bytes()
                    ).hexdigest(),
                    "frames": len(trace),
                    "fps": 12,
                    "physics_time_advanced": timeline.get_current_time() - start_time,
                    "trace": trace,
                },
                indent=2,
            )
        )
        for rp, rgb in streams:
            rgb.detach()
            rp.destroy()
    except Exception as exc:
        (a.output / "failed.json").write_text(json.dumps({"error": str(exc)}))
        raise
    finally:
        if process is not None:
            process.stdin.close()
            process.wait()
        app.close()


if __name__ == "__main__":
    main()
