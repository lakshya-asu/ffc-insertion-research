"""Unanchored segmented ribbon settling on a fixture. Offline physics study only."""

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--spec", type=Path, default=ROOT / "config/cables/rpi-camera-standard-mini-200-rev2.json")
    p.add_argument("--stiffness-scale", type=float, default=1)
    p.add_argument("--segments", type=int, default=100)
    p.add_argument("--dt", type=float, default=0.00025)
    a = p.parse_args()
    if not math.isfinite(a.stiffness_scale) or a.stiffness_scale <= 0 or a.segments not in (100, 200):
        raise ValueError("Positive stiffness scale and 100 or 200 segments required")
    if a.dt not in (0.00025, 0.000125):
        raise ValueError("Unreviewed timestep")
    from ffc.cable_spec import load_spec, provenance_record, values, width_at

    spec = load_spec(a.spec)
    v = values(spec)
    a.output.mkdir(parents=True, exist_ok=False)
    (a.output / "cable-spec.json").write_bytes(a.spec.read_bytes())
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
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleRigidPrim
    from pxr import PhysxSchema, UsdGeom, UsdLux, UsdPhysics, UsdShade

    from ffc.isaac_recording import Recorder
    from ffc.isaac_scene import box, camera, drive, joint

    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/World").GetPrim())
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, "Z")
    supports = [
        ("Base", (0, 0.106, 0.002), (0.036, 0.190, 0.004)),
        ("FrontSupport", (0, 0.029, 0.017), (0.011, 0.018, 0.026)),
        ("TailSupport", (0, 0.125, 0.017), (0.011, 0.150, 0.026)),
        ("Desk", (0, 0.065, -0.005), (0.290, 0.300, 0.010)),
    ]
    material = UsdShade.Material.Define(stage, "/World/Material")
    physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_material.CreateStaticFrictionAttr(v["static_friction"])
    physics_material.CreateDynamicFrictionAttr(v["dynamic_friction"])
    physics_material.CreateRestitutionAttr(0)
    for name, center, size in supports:
        box(
            stage,
            "/World/Fixture/" + name,
            center,
            size,
            (0.14, 0.49, 0.45) if name != "Desk" else (0.19, 0.22, 0.25),
        )
        UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath("/World/Fixture/" + name + "/Shape")).Bind(
            material, UsdShade.Tokens.weakerThanDescendants, "physics"
        )
    length = v["length_m"] / a.segments
    thickness, density = v["body_thickness_m"], v["equivalent_density_kg_m3"]
    young = v["equivalent_young_pa"] * a.stiffness_scale
    paths, heights, eis = [], [], []
    for i in range(a.segments):
        path = f"/World/Cable/S{i:03}"
        y = (i + 0.5) * length
        width = width_at(spec, y)
        stiff = min(y, v["length_m"] - y) < v["stiffener_length_m"]
        h = v["header_thickness_m"] if stiff else thickness
        body = box(
            stage,
            path,
            (0, (i + 0.5) * length, 0.03 + h / 2),
            (width, length, h),
            (0.06, 0.27, 0.75) if stiff else (0.055, 0.06, 0.075),
            density * width * length * h,
        )
        api = PhysxSchema.PhysxRigidBodyAPI(body)
        api.CreateSolverPositionIterationCountAttr(255)
        api.CreateSolverVelocityIterationCountAttr(8)
        api.CreateSleepThresholdAttr(0)
        UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(path + "/Shape")).Bind(
            material, UsdShade.Tokens.weakerThanDescendants, "physics"
        )
        mini = y < v["exposed_length_m"]
        standard = y > v["length_m"] - v["exposed_length_m"]
        if mini or standard:
            end = "mini" if mini else "standard"
            count = int(v[end + "_contacts"])
            for k in range(count):
                box(
                    stage,
                    path + f"/Contact{k:02}",
                    (
                        (k - (count - 1) / 2) * v[end + "_pitch_m"],
                        0,
                        (-1 if mini else 1) * (h / 2 + 0.000001),
                    ),
                    (0.0003, length, 0.000002),
                    (0.88, 0.64, 0.2),
                    collision=False,
                )
        paths.append(path)
        heights.append(h)
        eis.append(young * width * h**3 / 12)
        if i == 0:
            continue
        # Joint world anchor is at body midplane; thickness transition has an offset.
        j = joint(
            stage,
            f"/World/Joints/J{i:03}",
            UsdPhysics.Joint,
            paths[i - 1],
            path,
            (0, length / 2, (thickness - heights[i - 1]) / 2),
            (0, -length / 2, (thickness - h) / 2),
        )
        for axis in ("transX", "transY", "transZ"):
            lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), axis)
            lim.CreateLowAttr(1)
            lim.CreateHighAttr(-1)
        # Compliance of half of each adjacent segment adds in series.
        k_rad = 1 / (length / (2 * eis[i - 1]) + length / (2 * eis[i]))
        k_degree = k_rad * math.pi / 180
        for axis in ("rotX", "rotY", "rotZ"):
            limit = UsdPhysics.LimitAPI.Apply(j.GetPrim(), axis)
            limit.CreateLowAttr(-35 if axis == "rotX" else -10)
            limit.CreateHighAttr(35 if axis == "rotX" else 10)
            gain = k_degree * (1 if axis == "rotX" else 10)
            drive(j, axis, gain, gain * 0.01, 0, 0.01)
    UsdLux.DomeLight.Define(stage, "/World/Light").CreateIntensityAttr(1000)
    camera(stage, "/World/Cameras/Overview", (0.16, -0.10, 0.16), (0, 0.075, 0.025), 45)
    world = World(physics_dt=a.dt, rendering_dt=1 / 24, stage_units_in_meters=1)
    world.get_physics_context().set_solver_type("PGS")
    handles = [
        world.scene.add(SingleRigidPrim(prim_path=path, name=f"segment{i}")) for i, path in enumerate(paths)
    ]
    world.reset()
    recorder = Recorder(
        stage,
        a.output / "settle.mp4",
        fps=24,
        scope_label="Gravity settling only / assumed stiffness / no pickup or controller",
    )
    recorder.aim((0, 0.012, 0.03), eye_offset=(0.045, -0.03, 0.018))
    trace = []
    try:
        for step in range(round(2 / a.dt) + 1):
            if step:
                world.step(render=False)
            if step % round(1 / (24 * a.dt)) == 0 or step == round(2 / a.dt):
                positions = np.asarray([h.get_world_pose()[0] for h in handles])
                if not np.isfinite(positions).all() or np.max(np.abs(positions)) > 1:
                    raise RuntimeError("Nonfinite or exploded cable state")
                trace.append({"time_s": step * a.dt, "positions_m": positions.tolist()})
                recorder.write("Unanchored ribbon settling", step * a.dt)
        final = np.asarray(trace[-1]["positions_m"])
        pinch_index = int(0.012 / length)
        report = {
            "scope": "Gravity/support sensitivity, no closed-loop skill",
            "cable_spec": provenance_record(spec),
            "outline": "Revision-two width profile with assumed transition and terminal lengths",
            "stiffness_scale": a.stiffness_scale,
            "assumed_young_pa": young,
            "assumed_density_kg_m3": density,
            "body_ei_nm2": young * v["mini_width_m"] * thickness**3 / 12,
            "segments": a.segments,
            "dt_s": a.dt,
            "physics_steps": round(2 / a.dt),
            "tip_center_drop_mm": float((0.03 + v["header_thickness_m"] / 2 - final[0, 2]) * 1000),
            "pinch_region_center_drop_mm": float((0.03 + thickness / 2 - final[pinch_index, 2]) * 1000),
            "max_segment_center_spacing_mm": float(
                np.max(np.linalg.norm(np.diff(final, axis=0), axis=1)) * 1000
            ),
            "offline_scoring_only": True,
            "anchored_or_attached": False,
            "limitations": [
                "Homogeneous equivalent modulus and density are unmeasured hypotheses",
                "Segmented rigid ribbon is not a calibrated laminate/shell FEA model",
                "D6 equal swing gains couple torsion and in-plane stiffness assumptions",
                "Friction 0.5/0.4, angular damping, torque caps and joint limits assumed",
                "No gripper, vacuum, perception or ROS controller participates",
                "Width transition is discretized by segment-center sampling",
                "Far-end contact face is an unverified visual convention",
                "Final state is at 2 seconds, not a demonstrated equilibrium",
            ],
        }
        (a.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report), flush=True)
        stage.GetRootLayer().Export(str(a.output / "settle.usda"))
    finally:
        (a.output / "offline-trace.json").write_text(json.dumps(trace) + "\n")
        recorder.close()
        app.close()


if __name__ == "__main__":
    main()
