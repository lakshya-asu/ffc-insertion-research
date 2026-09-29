"""Isolate thin-body bending against a nonlinear static hinge-chain reference."""

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
    p.add_argument("--dt", type=float, default=0.000125)
    p.add_argument("--segments", type=int, default=16)
    p.add_argument("--solver", choices=["PGS", "TGS"], default="PGS")
    p.add_argument("--length", type=float, default=0.032)
    p.add_argument("--load-scale", type=float, default=1.0)
    p.add_argument("--modulus-scale", type=float, default=1.0)
    p.add_argument("--effective-gain", type=float, default=1.0)
    p.add_argument("--joint-viscosity-scale", type=float, default=1.0)
    p.add_argument("--formulation", choices=["maximal", "articulated", "hinge"], default="maximal")
    p.add_argument("--velocity-iterations", type=int, choices=[0, 8], default=8)
    p.add_argument("--external-forces-every-iteration", action="store_true")
    p.add_argument("--awake-articulation", action="store_true")
    p.add_argument("--mass-unit-scale", type=float, choices=[1.0, 1000.0], default=1.0)
    p.add_argument("--position-iterations", type=int, choices=[16, 32, 128, 255], default=255)
    a = p.parse_args()
    if a.dt not in [0.00025, 0.000125, 0.0000625, 0.00003125] or a.segments not in [12, 14, 16, 20, 32]:
        raise ValueError("Unreviewed numerical setting")
    if a.length not in [0.024, 0.028, 0.032, 0.040]:
        raise ValueError("Unreviewed span")
    if any(
        not math.isfinite(x) or x <= 0
        for x in [a.load_scale, a.modulus_scale, a.effective_gain, a.joint_viscosity_scale]
    ):
        raise ValueError("Positive finite scales required")
    from ffc.bending_fit import rotated_offset
    from ffc.cable_spec import load_spec, provenance_record, values
    from ffc.cantilever_reference import hinge_chain_reference

    spec = load_spec(a.spec)
    v = values(spec)
    length, width, thickness = a.length, v["mini_width_m"], v["body_thickness_m"]
    ei = v["equivalent_young_pa"] * a.modulus_scale * width * thickness**3 / 12
    k_degree = ei / (length / a.segments) * math.pi / 180
    cable = {
        "length_m": length,
        "width_m": width,
        "thickness_m": thickness,
        "segments": a.segments,
        "mass_kg": v["equivalent_density_kg_m3"] * width * thickness * length * a.load_scale,
        "start_xyz_m": [0, 0, 0.15],
        "stiffener_segments": 0,
        "stiffener_gain_multiplier": 1,
        "bend_stiffness_nm_per_degree": k_degree,
        "swing_stiffness_nm_per_degree": k_degree * 10,
        "damping_nm_s_per_degree": k_degree * 0.01 * a.joint_viscosity_scale,
        "solver_iterations": a.position_iterations,
        "benchmark_drag_per_s": 20,
    }
    reference = hinge_chain_reference(cable)
    # Effective numerical gains are deliberately excluded from the reference world.
    cable["angular_drive_gain_scale"] = a.effective_gain
    a.output.mkdir(parents=True, exist_ok=False)
    for path in [
        Path(__file__),
        ROOT / "src/ffc/isaac_scene.py",
        ROOT / "src/ffc/cantilever_reference.py",
        ROOT / "src/ffc/cable_spec.py",
        a.spec,
    ]:
        (a.output / path.name).write_bytes(path.read_bytes())
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
    from pxr import PhysxSchema, UsdGeom, UsdLux, UsdPhysics

    from ffc.isaac_recording import Recorder
    from ffc.isaac_scene import box, camera, drive, joint, make_cable

    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/World").GetPrim())
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, "Z")
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1 / a.mass_unit_scale)
    numerical_cable = dict(cable)
    for key in [
        "mass_kg",
        "bend_stiffness_nm_per_degree",
        "swing_stiffness_nm_per_degree",
        "damping_nm_s_per_degree",
    ]:
        numerical_cable[key] *= a.mass_unit_scale
    paths = make_cable(stage, {"cable": numerical_cable})
    for i in range(1, a.segments):
        prim = stage.GetPrimAtPath(f"/World/Cable/Joints/joint_{i:03d}")
        for axis in ["rotX", "rotY", "rotZ"]:
            UsdPhysics.DriveAPI(prim, axis).GetMaxForceAttr().Set(0.01 * a.mass_unit_scale)
    if a.formulation == "hinge":
        for i in range(1, a.segments):
            path = f"/World/Cable/Joints/joint_{i:03d}"
            stage.RemovePrim(path)
            hinge = joint(
                stage,
                path,
                UsdPhysics.RevoluteJoint,
                paths[i - 1],
                paths[i],
                (length / a.segments / 2, 0, 0),
                (-length / a.segments / 2, 0, 0),
            )
            hinge.CreateAxisAttr("Y")
            hinge.CreateLowerLimitAttr(-35)
            hinge.CreateUpperLimitAttr(35)
            drive(
                hinge,
                "angular",
                k_degree * a.effective_gain * a.mass_unit_scale,
                cable["damping_nm_s_per_degree"] * a.effective_gain * a.mass_unit_scale,
                0,
                0.01 * a.mass_unit_scale,
            )
    for path in paths:
        api = PhysxSchema.PhysxRigidBodyAPI(stage.GetPrimAtPath(path))
        api.CreateSolverVelocityIterationCountAttr(a.velocity_iterations)
        api.CreateSleepThresholdAttr(0)
    clamp = joint(
        stage, "/World/Clamp", UsdPhysics.FixedJoint, None, paths[0], (length / a.segments / 2, 0, 0.15)
    )
    if a.formulation != "maximal":
        UsdPhysics.ArticulationRootAPI.Apply(clamp.GetPrim())
        articulation = PhysxSchema.PhysxArticulationAPI.Apply(clamp.GetPrim())
        articulation.CreateSolverPositionIterationCountAttr(a.position_iterations)
        articulation.CreateSolverVelocityIterationCountAttr(a.velocity_iterations)
        articulation.CreateEnabledSelfCollisionsAttr(True)
        if a.awake_articulation:
            articulation.CreateSleepThresholdAttr(0)
            articulation.CreateStabilizationThresholdAttr(0)
    box(
        stage,
        "/World/ClampVisual",
        (-0.004, 0, 0.146),
        (0.008, 0.02, 0.008),
        (0.2, 0.45, 0.42),
        collision=False,
    )
    UsdLux.DomeLight.Define(stage, "/World/Light").CreateIntensityAttr(1000)
    camera(stage, "/World/Cameras/Overview", (0.07, -0.06, 0.20), (0.013, 0, 0.15), 55)
    world = World(physics_dt=a.dt, rendering_dt=1 / 24, stage_units_in_meters=1)
    world.get_physics_context().set_solver_type(a.solver)
    scene_api = PhysxSchema.PhysxSceneAPI(next(p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)))
    scene_api.CreateEnableExternalForcesEveryIterationAttr(a.external_forces_every_iteration)
    handles = [world.scene.add(SingleRigidPrim(prim_path=path, name=f"s{i}")) for i, path in enumerate(paths)]
    world.reset()
    recorder = Recorder(
        stage,
        a.output / "bending.mp4",
        fps=24,
        scope_label="Thin-body numerical benchmark / clamped root / no cable contact or pickup",
    )
    recorder.aim((0.02, 0, 0.15), eye_offset=(0.035, -0.04, 0.013))
    trace, tail = [], []
    count = round(1 / a.dt)
    try:
        for step in range(count + 1):
            if step:
                world.step(render=False)
            if step % max(1, round(1 / (24 * a.dt))) == 0 or step >= count - round(0.05 / a.dt):
                pos, quat = handles[-1].get_world_pose()
                tip = np.asarray(pos, dtype=np.float64) + rotated_offset(
                    quat, (length / a.segments / 2, 0, 0)
                )
                sag = float(0.15 - tip[2])
                if not math.isfinite(sag) or abs(sag) > length:
                    raise RuntimeError("Invalid cable deflection")
                entry = {"time_s": step * a.dt, "tip_sag_m": sag}
                if step >= count - round(0.05 / a.dt):
                    tail.append(entry)
                if step % max(1, round(1 / (24 * a.dt))) == 0 or step == count:
                    trace.append(entry)
                    recorder.write(
                        f"{a.formulation} / gain {a.effective_gain:g} / {a.solver} / dt {a.dt * 1000:g} ms",
                        step * a.dt,
                    )
        predicted = reference["nonlinear_hinge_chain_prediction_m"]
        observed = trace[-1]["tip_sag_m"]
        excursion = max(x["tip_sag_m"] for x in tail) - min(x["tip_sag_m"] for x in tail)
        error = abs(observed / predicted - 1)
        report = {
            "scope": "Homogeneous thin-body numerical bending; not material calibration",
            "cable_spec": provenance_record(spec),
            "cable": cable,
            "reference": reference,
            "physics_steps": count,
            "dt_s": a.dt,
            "solver": a.solver,
            "effective_gain": a.effective_gain,
            "joint_viscosity_scale": a.joint_viscosity_scale,
            "formulation": a.formulation,
            "velocity_iterations": a.velocity_iterations,
            "external_forces_every_iteration": a.external_forces_every_iteration,
            "awake_articulation": a.awake_articulation,
            "mass_unit_scale": a.mass_unit_scale,
            "load_scale": a.load_scale,
            "modulus_scale": a.modulus_scale,
            "observed_tip_sag_m": observed,
            "relative_error": error,
            "final_50ms_tip_excursion_m": excursion,
            "declared_numerical_reference_tolerance": 0.05,
            "single_case_reference_agreement": error <= 0.05 and excursion <= 0.01 * predicted,
            "full_cable_physics_qualified": False,
            "learning_dataset_release": False,
            "limitations": [
                "Body properties are assumptions from the evidence profile",
                "First full segment fixed; compare discrete boundary, not ideal continuum clamp",
                "20/s artificial body drag assists settling and is not material damping",
                "No end stiffener, width transition, contact, vacuum or insertion",
                "Single-case agreement cannot establish timestep/mesh convergence",
            ],
        }
        (a.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        (a.output / "trace.json").write_text(json.dumps(trace, indent=2) + "\n")
        (a.output / "equilibrium-window.json").write_text(json.dumps(tail) + "\n")
        stage.GetRootLayer().Export(str(a.output / "bending.usda"))
        print(
            json.dumps(
                {
                    k: report[k]
                    for k in ["observed_tip_sag_m", "relative_error", "single_case_reference_agreement"]
                }
            ),
            flush=True,
        )
    finally:
        recorder.close()
        app.close()


if __name__ == "__main__":
    main()
