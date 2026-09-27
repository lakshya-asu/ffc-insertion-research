"""Render a candidate optical layout with explicit, static cable/jaw occlusion probes."""

import argparse
import hashlib
import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--recess-labels", action="store_true")
    parser.add_argument("--config", type=Path, default=ROOT / "config/sensor-layout-v2.json")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Preserve evidence: use a new output directory")
    args.output.mkdir(parents=True)
    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": True,
            "renderer": "RaytracedLighting",
            "anti_aliasing": 2,
            "extra_args": ["--allow-root", "--/telemetry/enableAnonymousData=false"],
        }
    )
    products = []
    recess_annotators = {}
    recess_counts = {}
    exit_code = 0
    try:
        import numpy as np
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        from PIL import Image
        from pxr import Gf, UsdGeom, UsdLux, UsdSemantics

        from ffc.isaac_scene import box, camera
        from ffc.raspberry_pi_scene import make_camera_cable

        cfg = json.loads(args.config.read_text())
        context = omni.usd.get_context()
        context.new_stage()
        stage = context.get_stage()
        stage.GetRootLayer().subLayerPaths = [
            str(ROOT / "outputs/pi4-refined-002/raspberry-pi-workcell.usda")
        ]
        for _ in range(8):
            app.update()
        lighting = cfg["inspection_lighting"]
        box(
            stage,
            "/World/SensorStudyBackdrop",
            lighting["backdrop_center_m"],
            lighting["backdrop_size_m"],
            (0.035, 0.04, 0.04),
            collision=False,
        )
        light = UsdLux.RectLight.Define(stage, "/World/Lighting/InspectionFill")
        light.AddTransformOp().Set(
            Gf.Matrix4d()
            .SetLookAt(
                Gf.Vec3d(*lighting["light_eye_m"]), Gf.Vec3d(*lighting["light_target_m"]), Gf.Vec3d(0, 0, 1)
            )
            .GetInverse()
        )
        light.CreateWidthAttr(lighting["light_size_m"][0])
        light.CreateHeightAttr(lighting["light_size_m"][1])
        light.CreateIntensityAttr(lighting["intensity"])
        width, height = cfg["image_size"]
        aperture = cfg["sensor_aperture_mm"]
        metrics = []
        for view in cfg["cameras"]:
            distance = float(np.linalg.norm(np.array(view["eye_m"]) - view["target_m"]))
            focal = aperture[0] * distance * 1000 / view["horizontal_fov_mm"]
            cam = camera(
                stage,
                "/World/Cameras/SensorStudy_" + view["id"].replace("-", "_"),
                view["eye_m"],
                view["target_m"],
                focal,
            )
            cam.CreateHorizontalApertureAttr(aperture[0])
            cam.CreateVerticalApertureAttr(aperture[1])
            product = rep.create.render_product(str(cam.GetPath()), (width, height))
            rgb = rep.AnnotatorRegistry.get_annotator("rgb")
            rgb.attach(product)
            products.append((view["id"], product, rgb))
            if args.recess_labels:
                annotator = rep.AnnotatorRegistry.get_annotator(
                    "semantic_segmentation", init_params={"colorize": False, "semanticTypes": ["class"]}
                )
                annotator.attach(product)
                recess_annotators[view["id"]] = annotator
            metrics.append(
                {
                    **view,
                    "optical_center_to_target_mm": distance * 1000,
                    "ideal_focal_mm": focal,
                    "normal_plane_pixels_per_mm": width / view["horizontal_fov_mm"],
                    "normal_plane_vertical_fov_mm": view["horizontal_fov_mm"] * height / width,
                }
            )

        root = "/World/SensorStudyCable"
        task = json.loads((ROOT / "config/raspberry-pi-task.json").read_text())
        task["cable"]["curve_height_m"] = 0
        make_camera_cable(stage, task, root=root)
        xf = UsdGeom.Xformable(stage.GetPrimAtPath(root))
        op = xf.AddTransformOp()
        # Local source length +X -> vertical +Z, width +Y preserved, face +Z -> -X.
        board_rotation = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 1, 0), -90))
        # Presentation rotates that width onto X and broad-face normal onto Y.
        present_rotation = board_rotation * Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90))
        source_tip = Gf.Vec3d(0.335, -0.19, 0.0005)
        UsdGeom.Xform.Define(stage, "/World/SensorStudyJaws")
        jaw_paths = []
        for sign in [-1, 1]:
            path = "/World/SensorStudyJaws/Jaw" + str(sign).replace("-", "Minus")
            box(stage, path, (0, 0, 0), (0.004, 0.012, 0.003), (0.27, 0.30, 0.32), collision=False)
            jaw_paths.append((sign, path))
        timeline = omni.timeline.get_timeline_interface()
        timeline.pause()
        t0 = timeline.get_current_time()
        frames = []
        cases = [
            ("loose", None, None),
            ("present", cfg["presentation_tip_m"], present_rotation),
            ("approach", [0.6465, -0.1285, 0.0521], board_rotation),
            ("near", [0.6465, -0.1285, 0.0401], board_rotation),
        ]
        for case, tip, rotation in cases:
            UsdGeom.Imageable(stage.GetPrimAtPath("/World/Hardware/CameraCable")).GetVisibilityAttr().Set(
                "inherited" if case == "loose" else "invisible"
            )
            for path in [root, "/World/SensorStudyJaws"]:
                UsdGeom.Imageable(stage.GetPrimAtPath(path)).GetVisibilityAttr().Set(
                    "invisible" if case == "loose" else "inherited"
                )
            if tip:
                tip = Gf.Vec3d(*tip)
                op.Set(Gf.Matrix4d().SetTranslate(-source_tip) * rotation * Gf.Matrix4d().SetTranslate(tip))
                for sign, path in jaw_paths:
                    x = UsdGeom.Xformable(stage.GetPrimAtPath(path))
                    x.ClearXformOpOrder()
                    m = Gf.Matrix4d().SetTranslate(Gf.Vec3d(sign * 0.0022, 0, 0.005))
                    if case == "present":
                        m *= Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90))
                    x.AddTransformOp().Set(m * Gf.Matrix4d().SetTranslate(tip))
            for _ in range(2):
                rep.orchestrator.step(delta_time=0.0, rt_subframes=8, pause_timeline=True)
            for name, _, rgb in products:
                raw = np.asarray(rgb.get_data())[..., :3].astype(np.uint8)
                assert raw.shape == (height, width, 3)
                if (
                    (case == "loose" and name == "overview")
                    or (case == "present" and name == "inspection")
                    or (case in ["approach", "near"] and name.startswith("insertion"))
                ):
                    assert raw.std() > 5, (case, name, "unexpectedly empty task view")
                filename = (
                    f"{case}-{name}.png" if not args.recess_labels else f"{case}-{name}-annotation-pass.png"
                )
                Image.fromarray(raw).save(args.output / filename)
                frames.append({"case": case, "camera": name, "file": filename})
            if args.recess_labels:
                recess = json.loads((ROOT / "outputs/csi-recess-audit.json").read_text())
                plane = UsdGeom.Mesh.Define(stage, "/World/OfflineRecess/Plane")
                pts = []
                for xa, ya, xb, yb in recess["rectangles_xy_m"]:
                    pts.extend(
                        [
                            (xa, ya, recess["plane_z_m"]),
                            (xb, ya, recess["plane_z_m"]),
                            (xb, yb, recess["plane_z_m"]),
                            (xa, yb, recess["plane_z_m"]),
                        ]
                    )
                plane.GetVisibilityAttr().Set("inherited")
                plane.CreatePointsAttr(pts)
                plane.CreateFaceVertexCountsAttr([4] * (len(pts) // 4))
                plane.CreateFaceVertexIndicesAttr(list(range(len(pts))))
                plane.CreateSubdivisionSchemeAttr("none")
                plane.CreateDoubleSidedAttr(True)
                UsdSemantics.LabelsAPI.Apply(plane.GetPrim(), "class").CreateLabelsAttr(["cad_recess_audit"])
                for _ in range(3):
                    rep.orchestrator.step(delta_time=0.0, rt_subframes=4, pause_timeline=True)
                for name, annotator in recess_annotators.items():
                    data = annotator.get_data()
                    ids = np.asarray(data["data"]).reshape(height, width)
                    mask = np.zeros((height, width), np.uint8)
                    for key, value in data["info"]["idToLabels"].items():
                        if value.get("class") == "cad_recess_audit":
                            mask[ids == int(key)] = 255
                    (args.output / f"{case}-{name}-debug.json").write_text(
                        json.dumps(
                            {
                                "labels": data["info"]["idToLabels"],
                                "ids": np.unique(ids).tolist(),
                                "visibility": str(plane.ComputeVisibility()),
                                "attr": str(plane.GetVisibilityAttr().Get()),
                            }
                        )
                    )
                    for debug_name, _, debug_rgb in products:
                        if debug_name == name:
                            Image.fromarray(np.asarray(debug_rgb.get_data())[..., :3].astype(np.uint8)).save(
                                args.output / f"{case}-{name}-offline-debug.png"
                            )
                    if case in ["loose", "present"]:
                        assert (mask > 0).sum() > 100, (case, name, "missing recess label")
                    (args.output / f"{case}-{name}-label-map.json").write_text(
                        json.dumps(data["info"]["idToLabels"])
                    )
                    Image.fromarray(mask).save(args.output / f"{case}-{name}-offline-mask.png")
                    recess_counts[f"{case}-{name}"] = int((mask > 0).sum())
                # Keep the annotation mesh visible throughout this dedicated offline pass.
                # RGB for review comes from a separate run without this mesh.
            assert not timeline.is_playing() and timeline.get_current_time() == t0
            print("RENDERED", case, flush=True)
        stage.GetRootLayer().Export(str(args.output / "sensor-study.usda"))
        (args.output / "review.json").write_text(
            json.dumps(
                {
                    "configuration": cfg,
                    "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    "camera_metrics": metrics,
                    "frames": frames,
                    "offline_recess_pixels": recess_counts,
                    "rgb_contains_annotation_plane": bool(args.recess_labels),
                    "annotation_scope": "CAD cross-section prototype only; no learned entrance prediction",
                    "timeline_elapsed_s": timeline.get_current_time() - t0,
                    "motion_permitted": False,
                    "validation_scope": "static visual and ideal optical study only",
                },
                indent=2,
            )
        )
    except Exception:
        import traceback

        exit_code = 1
        traceback.print_exc()
        (args.output / "invalid.json").write_text(json.dumps({"error": traceback.format_exc()}))
    finally:
        for annotator in recess_annotators.values():
            annotator.detach()
        for _, product, rgb in products:
            rgb.detach()
            product.destroy()
        if os.geteuid() == 0:
            owner = ROOT.stat()
            for path in [args.output, *args.output.rglob("*")]:
                os.chown(path, owner.st_uid, owner.st_gid)
        app.close(exit_code=exit_code)


if __name__ == "__main__":
    main()
