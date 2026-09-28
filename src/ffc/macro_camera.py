"""Macro projection and hardware envelopes; no object state enters a controller."""

import math

import numpy as np


def optical_budget(profile):
    m = profile["magnification"]
    f = profile["nominal_focal_length_mm"]
    pitch = profile["pixel_pitch_um"] / 1000
    n = profile["starting_settings"]["physical_f_number"]
    u, v = f * (1 + 1 / m), f * (1 + m)
    w, h = profile["native_resolution"]
    return {
        "object_distance_from_inferred_principal_plane_mm": u,
        "principal_plane_behind_lens_front_mm": u - profile["lens_front_working_distance_mm"],
        "projection_focal_length_mm": v,
        "field_of_view_normal_plane_mm": [w * pitch / m, h * pitch / m],
        "object_sampling_um_per_pixel": pitch * 1000 / m,
        "K": [[v / pitch, 0, w / 2], [0, v / pitch, h / 2], [0, 0, 1]],
        "approx_total_dof_one_pixel_coc_mm": 2 * n * pitch * (1 + m) / m**2,
        "approx_airy_diameter_pixels_550nm": 2.44 * 0.00055 * n * (1 + m) / pitch,
        "equivalent_renderer_f_number": n * (1 + m),
        "bayer8_payload_MB_s_at_target_fps": w * h * profile["starting_settings"]["fps"] / 1e6,
        "notes": (
            "Thin-lens estimates, unit pupil magnification. DOF excludes diffraction; "
            "pixel sampling is not optical resolution. Settings are targets, not achieved rates."
        ),
    }


def add_macro_camera(
    stage, profile, mouth, camera_path="/World/Cameras/Macro", envelope_path="/World/Hardware/MacroEnvelope"
):
    from pxr import Gf, UsdGeom

    from ffc.camera_optics import apply_reference_optics
    from ffc.pi_zero_scene import box

    budget = optical_budget(profile)
    pose = profile["placement"]
    yaw, elevation = map(math.radians, [pose["yaw_degrees"], pose["elevation_degrees"]])
    axis = np.array(
        [math.cos(elevation) * math.cos(yaw), math.cos(elevation) * math.sin(yaw), math.sin(elevation)]
    )
    target = np.asarray(mouth) + np.asarray(pose["target_offset_from_mouth_mm"]) / 1000
    front = target + axis * profile["lens_front_working_distance_mm"] / 1000
    eye = target + axis * budget["object_distance_from_inferred_principal_plane_mm"] / 1000
    cam = UsdGeom.Camera.Define(stage, camera_path)
    view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1))
    cam.AddTransformOp().Set(view.GetInverse())
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.0001, 20))
    projection = dict(profile, focal_length_mm=budget["projection_focal_length_mm"])
    apply_reference_optics(stage, cam, projection, float(np.linalg.norm(eye - target)))
    path = envelope_path
    root = UsdGeom.Xform.Define(stage, path)
    mat = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*front), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse()
    root.AddTransformOp().Set(mat)
    diameter, length = np.array(profile["lens_envelope_diameter_length_mm"]) / 1000
    barrel = UsdGeom.Cylinder.Define(stage, path + "/Lens")
    barrel.CreateRadiusAttr(diameter / 2)
    barrel.CreateHeightAttr(length)
    barrel.AddTranslateOp().Set(Gf.Vec3d(0, 0, length / 2))
    barrel.CreateDisplayColorAttr([Gf.Vec3f(0.1, 0.11, 0.12)])
    width, height, depth = np.array(profile["camera_envelope_width_height_length_mm"]) / 1000
    box(stage, path + "/Body", (0, 0, length + depth / 2), (width, height, depth), (0.1, 0.3, 0.55))
    return (
        cam,
        root,
        dict(
            budget,
            eye_m=eye.tolist(),
            lens_front_m=front.tolist(),
            target_m=target.tolist(),
            envelope_status=(
                "Dimensioned primitives, not vendor CAD; no bracket, USB cable or collision qualification"
            ),
        ),
    )
