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
    a = p.parse_args()
    if a.dt not in [0.00025, 0.000125, 0.0000625, 0.00003125] or a.segments not in [16, 32]:
        raise ValueError("Unreviewed numerical setting")
    from ffc.cable_spec import load_spec, provenance_record, values
    from ffc.cantilever_reference import hinge_chain_reference

    spec = load_spec(a.spec)
    v = values(spec)
    length, width, thickness = 0.032, v["mini_width_m"], v["body_thickness_m"]
    ei = v["equivalent_young_pa"] * width * thickness**3 / 12
    k_degree = ei / (length / a.segments) * math.pi / 180
    cable = {
        "length_m": length,
        "width_m": width,
        "thickness_m": thickness,
        "segments": a.segments,
        "mass_kg": v["equivalent_density_kg_m3"] * width * thickness * length,
        "start_xyz_m": [0, 0, 0.15],
        "stiffener_segments": 0,
        "stiffener_gain_multiplier": 1,
        "bend_stiffness_nm_per_degree": k_degree,
        "swing_stiffness_nm_per_degree": k_degree * 10,
        "damping_nm_s_per_degree": k_degree * 0.01,
        "solver_iterations": 255,
        "benchmark_drag_per_s": 20,
    }
    reference = hinge_chain_reference(cable)
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
    from pxr import Gf, PhysxSchema, UsdGeom, UsdLux, UsdPhysics

    from ffc.isaac_recording import Recorder
    from ffc.isaac_scene import box, camera, joint, make_cable

    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/World").GetPrim())
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, "Z")
    paths = make_cable(stage, {"cable": cable})
    for path in paths:
        api = PhysxSchema.PhysxRigidBodyAPI(stage.GetPrimAtPath(path))
        api.CreateSolverVelocityIterationCountAttr(8)
        api.CreateSleepThresholdAttr(0)
    joint(stage, "/World/Clamp", UsdPhysics.FixedJoint, None, paths[0], (length / a.segments / 2, 0, 0.15))
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
                rotation = Gf.Rotation(Gf.Quatd(float(quat[0]), Gf.Vec3d(*[float(x) for x in quat[1:]])))
                tip = np.asarray(pos) + np.asarray(
                    rotation.TransformDir(Gf.Vec3d(length / a.segments / 2, 0, 0))
                )
                sag = float(0.15 - tip[2])
                if not math.isfinite(sag) or abs(sag) > length:
                    raise RuntimeError("Invalid cable deflection")
                entry = {"time_s": step * a.dt, "tip_sag_m": sag}
                if step >= count - round(0.05 / a.dt):
                    tail.append(entry)
                if step % max(1, round(1 / (24 * a.dt))) == 0 or step == count:
                    trace.append(entry)
                    recorder.write(f"{a.solver} / dt {a.dt * 1000:g} ms / {a.segments} segments", step * a.dt)
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
