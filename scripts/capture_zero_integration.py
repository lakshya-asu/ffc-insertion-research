"""Capture stereo RGB in the composed FR3/Zero scene; playback, not a physics run."""

import argparse
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--output", type=Path, required=True)
p.add_argument("--view", choices=["held", "entrance"], default="held")
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=False)
hashes = {}
for relative in [
    "scripts/capture_zero_integration.py",
    "src/ffc/zero_workstation.py",
    "src/ffc/socket_reference.py",
    "src/ffc/camera_optics.py",
    "src/ffc/macro_camera.py",
    "config/macro-mounted-camera.json",
    "config/connectors/zero-reference-contact-v1.json",
    "config/connectors/pi-socket-evidence-v1.json",
]:
    raw = (ROOT / relative).read_bytes()
    destination = a.output / "sources" / relative
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_bytes(raw)
    hashes[relative] = hashlib.sha256(raw).hexdigest()
(a.output / "source-hashes.json").write_text(json.dumps(hashes, indent=2))
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "extra_args": ["--allow-root"]})
try:
    import numpy as np
    import omni.replicator.core as rep
    import omni.timeline
    import omni.usd
    from PIL import Image
    from pxr import Gf, UsdGeom, UsdPhysics

    from ffc.camera_optics import apply_reference_optics
    from ffc.macro_camera import optical_budget
    from ffc.zero_workstation import add_workstation

    source = ROOT / "docs/library/fr3-flex-008/replay.usdz"
    omni.usd.get_context().open_stage(str(source))
    for _ in range(8):
        app.update()
    stage = omni.usd.get_context().get_stage()
    registration = add_workstation(stage, ROOT)
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
        if prim.IsA(UsdPhysics.Joint):
            UsdPhysics.Joint(prim).CreateJointEnabledAttr(False)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
    profile = json.loads((ROOT / "config/macro-mounted-camera.json").read_text())
    budget = optical_budget(profile)
    profile["focal_length_mm"] = budget["projection_focal_length_mm"]
    target = np.array([0.491 if a.view == "held" else 0.478, 0.0105, 0.290])
    offsets = (
        [[-0.035, 0, 0.122], [0.035, 0, 0.122]]
        if a.view == "held"
        else [[0.105, 0.07, 0.012], [0.08, 0.09, 0.012]]
    )
    calibrations = []
    annotators = []
    for i, offset in enumerate(offsets):
        axis = np.asarray(offset) / np.linalg.norm(offset)
        eye = target + axis * budget["object_distance_from_inferred_principal_plane_mm"] / 1000
        cam = UsdGeom.Camera.Define(stage, f"/World/Cameras/Stereo{i}")
        view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1))
        cam.AddTransformOp().Set(view.GetInverse())
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.0001, 20))
        calibration = apply_reference_optics(
            stage, cam, profile, float(np.linalg.norm(eye - target)), (1224, 1024)
        )
        # A fixed-focus optical reference, finite aperture as used by held-end capture.
        cam.CreateFStopAttr(budget["equivalent_renderer_f_number"])
        cv_world_to_camera = np.diag([1, -1, -1, 1]) @ np.asarray(view).T
        calibration["P"] = (np.asarray(calibration["K"]) @ cv_world_to_camera[:3]).tolist()
        calibration["world_to_camera"] = cv_world_to_camera.tolist()
        calibration["eye_m"] = eye.tolist()
        calibration["depth_of_field_enabled"] = True
        calibrations.append(calibration)
        product = rep.create.render_product(str(cam.GetPath()), (1224, 1024))
        ann = rep.AnnotatorRegistry.get_annotator("rgb")
        ann.attach(product)
        annotators.append(ann)
    timeline = omni.timeline.get_timeline_interface()
    timeline.pause()
    end = stage.GetEndTimeCode() / stage.GetTimeCodesPerSecond()
    rows = []
    stamps = [max(0.05, end - 0.6), end - 0.3, end] if a.view == "held" else [end]
    for sequence, stamp in enumerate(stamps):
        timeline.set_current_time(stamp)
        for _ in range(8):
            rep.orchestrator.step(delta_time=0, pause_timeline=True)
        for i, ann in enumerate(annotators):
            file = f"rgb-{sequence}-{i}.png"
            Image.fromarray(np.asarray(ann.get_data())[..., :3]).save(a.output / file)
        rows.append(
            {"sequence": sequence, "time_s": stamp, "images": [f"rgb-{sequence}-{i}.png" for i in range(2)]}
        )
    (a.output / "calibration.json").write_text(json.dumps(calibrations, indent=2))
    (a.output / "registration-offline.json").write_text(json.dumps(registration, indent=2))
    (a.output / "manifest.json").write_text(
        json.dumps(
            {"frames": rows, "mode": "Composed replay; not a physics trial", "source": str(source)}, indent=2
        )
    )
    stage.GetRootLayer().Export(str(a.output / "composed.usda"))
finally:
    app.close()
