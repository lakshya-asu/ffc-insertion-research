"""Fixed calibrated RGB views for Zero-cell commissioning.

Camera mounting coordinates are authoring parameters. Returned observations
contain RGB and camera calibration only, never scene object transforms.
"""

import json

import numpy as np


class ZeroCameras:
    def __init__(self, stage, root, output):
        import omni.replicator.core as rep
        from pxr import Gf, UsdGeom

        from ffc.camera_optics import apply_reference_optics
        from ffc.macro_camera import optical_budget

        self.output = output
        self.annotators, self.calibrations = [], []
        profile = json.loads((root / "config/macro-mounted-camera.json").read_text())
        budget = optical_budget(profile)
        profile["focal_length_mm"] = budget["projection_focal_length_mm"]
        views = [
            ("left", [0.491, 0.0105, 0.290], [-0.035, 0, 0.122]),
            ("right", [0.491, 0.0105, 0.290], [0.035, 0, 0.122]),
            ("entrance", [0.478, 0.0105, 0.290], [0.122, 0.025, 0.014]),
        ]
        for name, target, offset in views:
            target, axis = np.array(target), np.array(offset)
            eye = (
                target
                + axis
                / np.linalg.norm(axis)
                * budget["object_distance_from_inferred_principal_plane_mm"]
                / 1000
            )
            cam = UsdGeom.Camera.Define(stage, "/World/Cameras/Measured_" + name)
            view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1))
            cam.AddTransformOp().Set(view.GetInverse())
            cam.CreateClippingRangeAttr(Gf.Vec2f(0.0001, 20))
            calibration = apply_reference_optics(
                stage, cam, profile, float(np.linalg.norm(eye - target)), (1224, 1024)
            )
            cam.CreateFStopAttr(budget["equivalent_renderer_f_number"])
            extrinsic = np.diag([1, -1, -1, 1]) @ np.asarray(view).T
            calibration.update(
                name=name,
                P=(np.asarray(calibration["K"]) @ extrinsic[:3]).tolist(),
                world_to_camera=extrinsic.tolist(),
                eye_m=eye.tolist(),
                depth_of_field_enabled=True,
            )
            self.calibrations.append(calibration)
            product = rep.create.render_product(str(cam.GetPath()), (1224, 1024))
            ann = rep.AnnotatorRegistry.get_annotator("rgb")
            ann.attach(product)
            self.annotators.append(ann)
        (output / "camera-calibration.json").write_text(json.dumps(self.calibrations, indent=2))

    def capture(self, simulation_time_s):
        import omni.replicator.core as rep
        from PIL import Image

        from ffc.stereo_terminal import estimate

        for _ in range(8):
            rep.orchestrator.step(delta_time=0, pause_timeline=True)
        images = [np.asarray(ann.get_data())[..., :3].copy() for ann in self.annotators]
        observation = {
            "acquisition_simulation_time_s": simulation_time_s,
            "acquisition_mode": "Stopped physics, eight zero-time render updates",
            "source": "Native RGB and fixed camera calibration",
            "images": [],
            "motion_permitted": False,
            "entrance_metric_pose_verified": False,
        }
        for rgb, calibration in zip(images, self.calibrations, strict=True):
            name = calibration["name"] + ".png"
            Image.fromarray(rgb).save(self.output / name)
            observation["images"].append(name)
        try:
            observation["terminal"] = estimate(*images[:2], *[c["P"] for c in self.calibrations[:2]])
        except (ValueError, np.linalg.LinAlgError) as error:
            observation["terminal_error"] = str(error)
        (self.output / "inspection-observation.json").write_text(json.dumps(observation, indent=2))
        return observation
