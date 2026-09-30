"""Isaac path-traced presentation of the actual compact tool concept, without physics."""

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": True,
            "renderer": "PathTracing",
            "width": 1920,
            "height": 1440,
            "extra_args": ["--allow-root", "--/telemetry/enableAnonymousData=false"],
        }
    )
    try:
        import carb
        import numpy as np
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        from PIL import Image
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade

        from ffc.isaac_scene import box, camera

        settings = carb.settings.get_settings()
        settings.set("/rtx/pathtracing/spp", 64)
        settings.set("/rtx/pathtracing/totalSpp", 512)
        settings.set("/rtx/pathtracing/maxBounces", 8)
        settings.set("/rtx/pathtracing/optixDenoiser/enabled", True)
        context = omni.usd.get_context()
        context.new_stage()
        stage = context.get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, 1)
        UsdGeom.SetStageUpAxis(stage, "Z")
        world = UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(world.GetPrim())

        def mat(name, color, metal=0, rough=0.4):
            m = UsdShade.Material.Define(stage, "/World/Looks/" + name)
            s = UsdShade.Shader.Define(stage, m.GetPath().AppendChild("Shader"))
            s.CreateIdAttr("UsdPreviewSurface")
            for key, value, kind in [
                ("diffuseColor", Gf.Vec3f(*color), Sdf.ValueTypeNames.Color3f),
                ("metallic", metal, Sdf.ValueTypeNames.Float),
                ("roughness", rough, Sdf.ValueTypeNames.Float),
            ]:
                s.CreateInput(key, kind).Set(value)
            m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(), "surface")
            return m

        materials = {
            "frame": mat("AnodizedAluminium", (0.18, 0.22, 0.24), 0.75, 0.3),
            "steel": mat("GroundSteel", (0.52, 0.55, 0.59), 1, 0.23),
            "rubber": mat("Elastomer", (0.018, 0.021, 0.023), 0, 0.78),
            "coil": mat("CoilEnvelope", (0.31, 0.12, 0.045), 0.65, 0.28),
            "gauge": mat("GaugeReserve", (0.38, 0.24, 0.08), 0.25, 0.45),
            "film": mat("CableInsulation", (0.12, 0.135, 0.14), 0, 0.32),
            "blue": mat("BlueStiffener", (0.025, 0.095, 0.24), 0, 0.3),
            "gold": mat("ContactMetal", (0.72, 0.44, 0.12), 1, 0.22),
        }
        tool = UsdGeom.Xform.Define(stage, "/World/Tool")
        source = ROOT / "outputs/micro-flexure-finished-001"
        tool.GetPrim().GetReferences().AddReference(str(source / "closed.usda"))
        for prim in Usd.PrimRange(tool.GetPrim()):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            name = prim.GetName()
            color = UsdGeom.Mesh(prim).GetDisplayColorAttr().Get()[0]
            kind = "frame" if abs(color[0] - 0.10) < 0.01 else "steel"
            if "pad" in name or name in ["vacuum_lip", "vacuum_neck"]:
                kind = "rubber"
            if "coil_envelope" in name:
                kind = "coil"
            if "gauge" in name:
                kind = "gauge"
            if "reserve" in name:
                kind = "rubber"
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(materials[kind])

        def block(name, pos, size, material):
            prim = box(stage, "/World/" + name, pos, size, (0.5, 0.5, 0.5), collision=False)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)
            return prim

        block("CableBody", (0.023, 0, 0), (0.046, 0.0115, 0.00014), materials["film"])
        block("Stiffener", (0.003, 0, 0.00004), (0.006, 0.0115, 0.00022), materials["blue"])
        for i in range(22):
            block(
                "Contact" + str(i),
                (0.002, (i - 10.5) * 0.0005, -0.00009),
                (0.004, 0.0003, 0.000025),
                materials["gold"],
            )
        floor_mat = mat("Bench", (0.30, 0.32, 0.33), 0, 0.62)
        block("Bench", (0, 0, -0.007), (0.7, 0.7, 0.0055), floor_mat)
        dome = UsdLux.DomeLight.Define(stage, "/World/Fill")
        dome.CreateIntensityAttr(300)
        for name, position, target, intensity, size in [
            ("Key", (-0.08, -0.04, 0.16), (0.02, 0, 0), 1400, 0.12),
            ("Rim", (0.08, 0.09, 0.09), (0.02, 0, 0.008), 1900, 0.08),
            ("SoftFill", (0.04, -0.15, 0.04), (0.02, 0, 0.008), 1600, 0.07),
        ]:
            light = UsdLux.RectLight.Define(stage, "/World/" + name)
            light.CreateWidthAttr(size)
            light.CreateHeightAttr(size)
            light.CreateIntensityAttr(intensity)
            transform = (
                Gf.Matrix4d()
                .SetLookAt(Gf.Vec3d(*position), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1))
                .GetInverse()
            )
            light.AddTransformOp().Set(transform)
        cam = camera(stage, "/World/Camera", (-0.049, -0.07, 0.056), (0.022, 0.002, 0.008), 65)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.0001, 10))
        board = UsdGeom.Xform.Define(stage, "/World/Board")
        board.GetPrim().GetReferences().AddReference(str(ROOT / "third_party/raspberry_pi/zero/zero2w.usdc"))
        board.AddTranslateOp().Set(Gf.Vec3d(-0.0333, 0.0008, -0.0022))
        UsdGeom.Imageable(board).MakeInvisible()
        timeline = omni.timeline.get_timeline_interface()
        timeline.pause()
        rp = rep.create.render_product("/World/Camera", (1920, 1440))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach(rp)
        shots = [
            ("assembly", (-0.049, -0.07, 0.056), (0.022, 0.002, 0.008), 65, False),
            ("fingers", (-0.026, -0.036, 0.022), (0.011, 0, 0.0015), 70, False),
            ("pickup", (0.005, 0.085, 0.038), (0.025, 0.007, 0.006), 65, False),
            ("board", (-0.12, -0.16, 0.13), (-0.012, 0, 0.005), 65, True),
        ]
        for name, eye, target, focal, show_board in shots:
            (UsdGeom.Imageable(board).MakeVisible if show_board else UsdGeom.Imageable(board).MakeInvisible)()
            c = UsdGeom.Xformable(cam.GetPrim())
            c.ClearXformOpOrder()
            c.AddTransformOp().Set(
                Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse()
            )
            cam.GetFocalLengthAttr().Set(focal)
            for _ in range(12):
                app.update()
            for _ in range(12):
                rep.orchestrator.step(delta_time=0, rt_subframes=8, pause_timeline=True)
            arr = np.asarray(rgb.get_data())[..., :3].astype(np.uint8)
            assert arr.shape == (1440, 1920, 3) and arr.std() > 5
            Image.fromarray(arr).save(args.output / (name + ".png"))
            print("RENDERED", name, flush=True)
            assert not timeline.is_playing() and timeline.get_current_time() == 0
        stage.GetRootLayer().Export(str(args.output / "presentation.usda"))
        (args.output / "scope.json").write_text(
            json.dumps(
                {
                    "renderer": "Isaac Sim RTX PathTracing",
                    "resolution": [1920, 1440],
                    "physics": False,
                    "samples_per_pixel_requested": 512,
                    "geometry": "Concept with proposed edge finishing; supplier internals provisional",
                    "materials": "Uncalibrated PBR appearance choices",
                    "cameras": "Presentation cameras, not calibrated task observations",
                    "board": "Existing community CAD, not qualified mating geometry",
                },
                indent=2,
            )
        )
        rgb.detach()
        rp.destroy()
    finally:
        app.close()


if __name__ == "__main__":
    main()
