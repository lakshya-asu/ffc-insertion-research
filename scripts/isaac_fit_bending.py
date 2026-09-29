"""Run a frozen fitting or validation batch in Isaac; no policy or task controller."""

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--targets", type=Path, default=ROOT / "outputs/world-model-targets-001/targets.json")
    p.add_argument("--split", choices=["fit", "validation"], required=True)
    p.add_argument("--gains", type=float, nargs="+", required=True)
    p.add_argument("--formulation", choices=["articulated", "hinge"], required=True)
    p.add_argument("--dt", type=float, default=0.000125)
    p.add_argument("--solver", choices=["PGS", "TGS"], default="PGS")
    p.add_argument("--velocity-iterations", type=int, choices=[0, 8], default=8)
    p.add_argument("--external-forces-every-iteration", action="store_true")
    p.add_argument("--joint-viscosity-scale", type=float, default=1.0)
    p.add_argument("--live-feed", type=Path)
    p.add_argument("--selection", type=Path)
    p.add_argument("--awake-articulation", action="store_true")
    p.add_argument("--position-iterations", type=int, choices=[32, 128, 255], default=255)
    a = p.parse_args()
    if a.dt not in [0.000125, 0.0000625, 0.00003125] or any(not math.isfinite(g) or g <= 0 for g in a.gains):
        raise ValueError("Invalid numerical setting")
    if a.split == "validation" and len(a.gains) != 1:
        raise ValueError("Freeze one gain before validation")
    if not math.isfinite(a.joint_viscosity_scale) or a.joint_viscosity_scale <= 0:
        raise ValueError("Invalid joint viscosity scale")
    from ffc.bending_fit import rotated_offset
    from ffc.cable_spec import load_spec, values

    spec_path = ROOT / "config/cables/rpi-camera-standard-mini-200-rev2.json"
    spec = load_spec(spec_path)
    v = values(spec)
    raw = a.targets.read_bytes()
    targets = json.loads(raw)
    if targets["sources"].get(str(spec_path.relative_to(ROOT))) != spec["sha256"]:
        raise ValueError("Target material profile mismatch")
    cases = [{**r, "gain": gain} for gain in a.gains for r in targets["responses"] if r["split"] == a.split]
    a.output.mkdir(parents=True, exist_ok=False)
    frozen = {}
    for path in [
        Path(__file__),
        ROOT / "src/ffc/isaac_scene.py",
        ROOT / "src/ffc/bending_fit.py",
        ROOT / "src/ffc/isaac_recording.py",
        spec_path,
        a.targets,
    ]:
        source = path.read_bytes()
        (a.output / path.name).write_bytes(source)
        frozen[str(path)] = hashlib.sha256(source).hexdigest()
    settings = {
        "formulation": a.formulation,
        "dt_s": a.dt,
        "gains": a.gains,
        "split": a.split,
        "position_iterations": a.position_iterations,
        "velocity_iterations": a.velocity_iterations,
        "solver": a.solver,
        "external_forces_every_iteration": a.external_forces_every_iteration,
        "duration_s": 1.0,
        "body_drag_per_s": 20,
        "joint_viscosity_scale": a.joint_viscosity_scale,
        "awake_articulation": a.awake_articulation,
        "relative_error_limit": 0.05,
        "target_sha256": hashlib.sha256(raw).hexdigest(),
        "source_hashes": frozen,
    }
    if a.split == "validation":
        if a.selection is None:
            raise ValueError("Validation requires a frozen fitting decision")
        decision_bytes = a.selection.read_bytes()
        decision = json.loads(decision_bytes)
        if a.gains != [decision["selected_gain"]]:
            raise ValueError("Selected gain changed before validation")
        for key, value in decision["numerical_settings"].items():
            if settings.get(key) != value:
                raise ValueError(f"Numerical setting changed before validation: {key}")
        settings["selection_sha256"] = hashlib.sha256(decision_bytes).hexdigest()
        (a.output / "selection.json").write_bytes(decision_bytes)
    (a.output / "settings.json").write_text(json.dumps(settings, indent=2) + "\n")
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
    from ffc.isaac_scene import box, camera, drive, joint, make_cable

    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/World").GetPrim())
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, "Z")
    for idx, case in enumerate(cases):
        length = case["length_m"]
        n = round(length / 0.002)
        ds = length / n
        origin = (idx % 10 * 0.065, idx // 10 * 0.025, 0.15)
        case["origin_m"] = origin
        root = f"/World/Cable{idx:03d}"
        k = case["ei_nm2"] / ds * math.pi / 180
        cable = {
            "segments": n,
            "length_m": length,
            "width_m": v["mini_width_m"],
            "thickness_m": v["body_thickness_m"],
            "mass_kg": v["equivalent_density_kg_m3"]
            * length
            * v["mini_width_m"]
            * v["body_thickness_m"]
            * case["uniform_load_multiplier"],
            "start_xyz_m": origin,
            "stiffener_segments": 0,
            "stiffener_gain_multiplier": 1,
            "bend_stiffness_nm_per_degree": k,
            "swing_stiffness_nm_per_degree": k * 10,
            "damping_nm_s_per_degree": k * 0.01 * a.joint_viscosity_scale,
            "angular_drive_gain_scale": case["gain"],
            "solver_iterations": a.position_iterations,
            "benchmark_drag_per_s": 20,
        }
        paths = make_cable(stage, {"cable": cable}, root_path=root)
        # Appearance only: a dark shielded body keeps the reference line readable.
        for path in paths:
            UsdGeom.Gprim(stage.GetPrimAtPath(path + "/Shape")).GetDisplayColorAttr().Set(
                [Gf.Vec3f(0.035, 0.045, 0.06)]
            )
        case["tip_path"] = paths[-1]
        for path in paths:
            api = PhysxSchema.PhysxRigidBodyAPI(stage.GetPrimAtPath(path))
            api.CreateSleepThresholdAttr(0)
            api.CreateSolverVelocityIterationCountAttr(a.velocity_iterations)
        if a.formulation == "hinge":
            for i in range(1, n):
                path = f"{root}/Joints/joint_{i:03d}"
                stage.RemovePrim(path)
                hinge = joint(
                    stage,
                    path,
                    UsdPhysics.RevoluteJoint,
                    paths[i - 1],
                    paths[i],
                    (ds / 2, 0, 0),
                    (-ds / 2, 0, 0),
                )
                hinge.CreateAxisAttr("Y")
                hinge.CreateLowerLimitAttr(-35)
                hinge.CreateUpperLimitAttr(35)
                drive(
                    hinge,
                    "angular",
                    k * case["gain"],
                    k * 0.01 * a.joint_viscosity_scale * case["gain"],
                    0,
                    0.01,
                )
        clamp = joint(
            stage,
            root + "/Clamp",
            UsdPhysics.FixedJoint,
            None,
            paths[0],
            (origin[0] + ds / 2, origin[1], origin[2]),
        )
        UsdPhysics.ArticulationRootAPI.Apply(clamp.GetPrim())
        art = PhysxSchema.PhysxArticulationAPI.Apply(clamp.GetPrim())
        art.CreateSolverPositionIterationCountAttr(a.position_iterations)
        art.CreateSolverVelocityIterationCountAttr(a.velocity_iterations)
        art.CreateEnabledSelfCollisionsAttr(True)
        if a.awake_articulation:
            art.CreateSleepThresholdAttr(0)
            art.CreateStabilizationThresholdAttr(0)
        box(
            stage,
            root + "/ClampVisual",
            (origin[0] - 0.002, origin[1], origin[2] - 0.003),
            (0.004, 0.016, 0.006),
            (0.12, 0.40, 0.38),
            collision=False,
        )
        angles = np.cumsum(case["reference"]["reference_joint_angles_rad"])
        # The reference is beside the near edge for visibility; its X/Z shape is unchanged.
        reference_y = origin[1] - v["mini_width_m"] / 2 - 0.0005
        points = [
            Gf.Vec3f(origin[0], reference_y, origin[2]),
            Gf.Vec3f(origin[0] + ds, reference_y, origin[2]),
        ]
        for angle in angles:
            last = points[-1]
            points.append(last + Gf.Vec3f(ds * np.cos(angle), 0, -ds * np.sin(angle)))
        curve = UsdGeom.BasisCurves.Define(stage, root + "/OfflineReference")
        curve.CreateTypeAttr("linear")
        curve.CreateCurveVertexCountsAttr([len(points)])
        curve.CreatePointsAttr(points)
        curve.CreateWidthsAttr([0.00015])
        curve.SetWidthsInterpolation("constant")
        curve.CreateDisplayColorAttr([Gf.Vec3f(0.8, 0.06, 0.25)])
    selected = max(cases, key=lambda c: c["reference"]["nonlinear_hinge_chain_prediction_m"])
    ox, oy, oz = selected["origin_m"]
    camera(stage, "/World/Cameras/Overview", (0.3, -0.60, 0.70), (0.3, 0.06, 0.14), 45)
    UsdLux.DomeLight.Define(stage, "/World/Light").CreateIntensityAttr(1000)
    world = World(physics_dt=a.dt, rendering_dt=1 / 24, stage_units_in_meters=1)
    world.get_physics_context().set_solver_type(a.solver)
    scene_api = PhysxSchema.PhysxSceneAPI(next(p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)))
    scene_api.CreateEnableExternalForcesEveryIterationAttr(a.external_forces_every_iteration)
    world.reset()
    clock = {"steps": 0, "time_s": 0.0}

    def count_step(dt):
        clock["steps"] += 1
        clock["time_s"] += float(dt)

    world.add_physics_callback("bending-clock", count_step)
    handles = [SingleRigidPrim(prim_path=c["tip_path"], name=f"tip{i}") for i, c in enumerate(cases)]
    for handle in handles:
        handle.initialize()
    recorder = Recorder(
        stage,
        a.output / "bending.mp4",
        fps=24,
        scope_label="Assumed-world bending / red is offline reference / no pickup or contact",
    )
    recorder.aim((ox + 0.02, oy, oz - 0.003), eye_offset=(0.025, -0.045, 0.018))
    traces, tail, pose_samples, sample_times = [], [], [], []
    count = round(1 / a.dt)
    try:
        for step in range(count + 1):
            if step:
                world.step(render=False)
            capture = step % max(1, round(1 / (24 * a.dt))) == 0 or step == count
            if capture or step >= count - round(0.05 / a.dt):
                values_now = []
                poses_now = []
                for case, handle in zip(cases, handles, strict=True):
                    pos, q = handle.get_world_pose()
                    poses_now.append([*map(float, pos), *map(float, q)])
                    tip = np.asarray(pos, dtype=np.float64) + rotated_offset(q, (0.001, 0, 0))
                    sag = float(case["origin_m"][2] - tip[2])
                    if not math.isfinite(sag) or abs(sag) > case["length_m"]:
                        raise RuntimeError("Invalid simulated tip")
                    values_now.append(sag)
                pose_samples.append(poses_now)
                sample_times.append(step * a.dt)
                if step >= count - round(0.05 / a.dt):
                    tail.append(values_now)
                if capture:
                    traces.append({"time_s": step * a.dt, "sag_m": values_now})
                    recorder.write(f"{a.split} / {a.formulation} / {len(cases)} responses", step * a.dt)
                    if a.live_feed:
                        from ffc.lab_feed import publish_snapshot

                        rgb = np.concatenate(
                            [np.asarray(ann.get_data())[..., :3] for ann in recorder.annotators], axis=1
                        )
                        publish_snapshot(
                            rgb,
                            a.live_feed,
                            "workcell",
                            len(traces),
                            f"Bending {a.split} batch at {step * a.dt:.3f}s; "
                            "red reference is offline only; no robot control",
                            a.output.name,
                        )
                    print(json.dumps({"progress": round(step / count, 3)}), flush=True)
        excursions = np.ptp(np.asarray(tail), axis=0)
        for i, case in enumerate(cases):
            target = case["reference"]["nonlinear_hinge_chain_prediction_m"]
            case["observed_sag_m"] = traces[-1]["sag_m"][i]
            case["relative_error"] = abs(case["observed_sag_m"] / target - 1)
            case["tail_excursion_m"] = float(excursions[i])
            case["passed"] = bool(case["relative_error"] <= 0.05 and excursions[i] <= 0.01 * target)
        np.savez_compressed(
            a.output / "measured-poses.npz",
            time_s=np.asarray(sample_times),
            position_xyz_quaternion_wxyz=np.asarray(pose_samples),
            tip_paths=np.asarray([c["tip_path"] for c in cases]),
        )
        (a.output / "trace.json").write_text(json.dumps(traces) + "\n")
        stage.GetRootLayer().Export(str(a.output / "bending.usda"))
        if clock["steps"] != count or abs(clock["time_s"] - 1.0) > 1e-6:
            raise RuntimeError(f"Physics clock mismatch: {clock}; expected {count} steps and one second")
        report = {
            "settings": settings,
            "physics_clock": dict(clock),
            "cases": cases,
            "all_passed": all(c["passed"] for c in cases),
            "scope": "Static bending approximation only; dynamic and contact behavior unqualified",
        }
        (a.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps({"passed": sum(c["passed"] for c in cases), "total": len(cases)}), flush=True)
        if a.split == "validation":
            import omni.replicator.core as rep
            from PIL import Image

            (a.output / "cases").mkdir()
            for idx, case in enumerate(cases):
                ox, oy, oz = case["origin_m"]
                recorder.aim(
                    (ox + case["length_m"] / 2, oy, oz - case["observed_sag_m"] / 2),
                    eye_offset=(0.025, -0.045, 0.018),
                )
                for _ in range(3):
                    rep.orchestrator.step(delta_time=0.0, pause_timeline=False)
                rgb = np.asarray(recorder.annotators[1].get_data())[..., :3]
                Image.fromarray(rgb).save(a.output / "cases" / f"case-{idx:03d}.jpg", quality=94)
            if clock["steps"] != count:
                raise RuntimeError("Inspection image capture advanced physics")
        recorder.close()
        (a.output / "completed.json").write_text(
            json.dumps(
                {
                    "physics_clock": clock,
                    "cases": len(cases),
                    "passed": sum(c["passed"] for c in cases),
                    "video_sha256": hashlib.sha256((a.output / "bending.mp4").read_bytes()).hexdigest(),
                    "inspection_images": len(list((a.output / "cases").glob("*.jpg")))
                    if a.split == "validation"
                    else 0,
                },
                indent=2,
            )
            + "\n"
        )
    except Exception:
        import traceback

        failure = traceback.format_exc()
        (a.output / "failure.txt").write_text(failure)
        print(failure, flush=True)
        raise
    finally:
        recorder.close()
        app.close()


if __name__ == "__main__":
    main()
