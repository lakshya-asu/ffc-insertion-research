"""Build, validate, simulate and render the FFC cell in Isaac Sim."""

from __future__ import annotations

import argparse
import hashlib
import json
import linecache
import shutil
import signal
import sys
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--steps", type=int, default=480)
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/e002")
    parser.add_argument("--exercise-tool", action="store_true")
    parser.add_argument("--cable-model", choices=["shell", "segments"], default="segments")
    parser.add_argument("--config", type=Path, default=ROOT / "config/isaac-workcell.json")
    parser.add_argument("--assembly", action="store_true")
    parser.add_argument("--grasp-benchmark", action="store_true")
    parser.add_argument("--transfer-benchmark", action="store_true")
    parser.add_argument("--pinch-benchmark", action="store_true")
    parser.add_argument("--strategy", choices=["fixture", "direct"], default="fixture")
    parser.add_argument("--video", action="store_true", default=True)
    parser.add_argument("--no-video", action="store_false", dest="video")
    parser.add_argument("--cantilever", action="store_true")
    parser.add_argument("--cantilever-length", type=float, default=0.05)
    args = parser.parse_args()
    if args.cable_model == "shell" and args.config == ROOT / "config/isaac-workcell.json":
        args.config = ROOT / "config/native-shell.json"
    if args.grasp_benchmark or args.transfer_benchmark:
        args.assembly = True
    if args.transfer_benchmark:
        args.strategy = "direct"
    if args.pinch_benchmark:
        args.exercise_tool = False
    if args.steps < 240:
        parser.error("--steps must be at least 240 for the smoke test")
    if args.assembly and args.cable_model != "segments":
        parser.error("The gated assembly experiment currently requires --cable-model segments")
    if any((args.output / name).exists() for name in ("source", "results.json", "workcell.usda")):
        parser.error("Output contains an earlier experiment; use a fresh --output directory")
    args.output.mkdir(parents=True, exist_ok=True)
    source = args.output / "source"
    source.mkdir(exist_ok=True)
    shutil.copytree(
        ROOT / "src/ffc", source / "src/ffc", dirs_exist_ok=True, ignore=shutil.ignore_patterns("__pycache__")
    )
    shutil.copy2(__file__, source / "isaac_workcell.py")
    frozen_runner = (source / "isaac_workcell.py").read_text()
    # Keep traceback lines tied to this run even while the workspace evolves.
    linecache.cache[str(Path(__file__).resolve())] = (
        len(frozen_runner),
        None,
        frozen_runner.splitlines(keepends=True),
        str(Path(__file__).resolve()),
    )
    for name in ("Dockerfile", "compose.yaml", "pyproject.toml", "uv.lock", "config/fr3-isaac-source.json"):
        path = ROOT / name
        if path.exists():
            shutil.copy2(path, source / path.name)
    shutil.copy2(args.config, source / "config.json")
    args.config = source / "config.json"
    sys.path.insert(0, str(source / "src"))
    hashes = {
        str(p.relative_to(source)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in source.rglob("*")
        if p.is_file()
    }
    (source / "sha256.json").write_text(json.dumps(hashes, indent=2))
    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": not args.gui,
            "width": 1280,
            "height": 800,
            "renderer": "RaytracedLighting",
            "anti_aliasing": 2,
            "extra_args": [
                "--allow-root",
                "--/telemetry/enableAnonymousData=false",
                "--/privacy/usage=false",
                "--/privacy/performance=false",
                "--/privacy/personalization=false",
            ],
        }
    )

    interrupted_signal = None

    def interrupt_run(signum, _frame):
        nonlocal interrupted_signal
        interrupted_signal = signum

    signal.signal(signal.SIGINT, interrupt_run)
    signal.signal(signal.SIGTERM, interrupt_run)
    recorder = None
    report = {"status": "RUNNING"}
    try:
        import carb
        import numpy as np
        import omni.kit.app
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.types import ArticulationAction
        from pxr import Gf, Usd, UsdGeom

        from ffc.isaac_scene import build
        from ffc.isaac_validation import validate_stage

        cfg = json.loads(args.config.read_text())
        carb.settings.get_settings().set_bool("/physics/updateToUsd", True)
        cfg["cable_model"] = args.cable_model
        cfg["handling_strategy"] = args.strategy
        cfg["grasp_benchmark"] = args.grasp_benchmark
        cfg["transfer_benchmark"] = args.transfer_benchmark
        if args.cantilever:
            original_length = cfg["cable"]["length_m"]
            if not 0 < args.cantilever_length <= original_length:
                raise ValueError("Cantilever length must be positive and no longer than the configured cable")
            cfg["cable"]["length_m"] = args.cantilever_length
            cfg["cable"]["mass_kg"] *= args.cantilever_length / original_length
            cfg["cable"]["start_xyz_m"][2] = 0.20
            if args.cable_model == "segments":
                cfg["cable"]["segments"] = round(
                    cfg["cable"]["segments"] * args.cantilever_length / original_length
                )
                cfg["cable"]["stiffener_segments"] = 0
        if cfg["cable"].get("benchmark_drag_per_s", 0) and not args.cantilever:
            raise ValueError("Artificial equilibrium drag is restricted to the cantilever benchmark")
        np.random.seed(cfg["seed"])
        stage_path = args.output / "workcell.usda"
        context = omni.usd.get_context()
        context.new_stage()
        stage = context.get_stage()
        stage.GetRootLayer().Export(str(stage_path))
        context.open_stage(str(stage_path))
        for _ in range(5):
            app.update()
        stage = context.get_stage()
        paths = build(stage, ROOT, cfg)
        if args.cantilever and args.cable_model == "shell":
            from pxr import Gf

            from ffc.isaac_shell import shell_points

            anchor = UsdGeom.Xform.Define(stage, "/World/CantileverAnchor")
            indices = list(range(2 * (cfg["shell"]["width_elements"] + 1)))
            points = shell_points(stage)
            attachment = stage.DefinePrim("/World/CantileverAttachment", "OmniPhysicsVtxXformAttachment")
            attachment.GetRelationship("omniphysics:src0").SetTargets(["/World/Cable/Surface"])
            attachment.GetRelationship("omniphysics:src1").SetTargets([anchor.GetPath()])
            attachment.GetAttribute("omniphysics:vtxIndicesSrc0").Set(indices)
            attachment.GetAttribute("omniphysics:localPositionsSrc1").Set(
                [Gf.Vec3f(*points[i]) for i in indices]
            )
            stage.GetRootLayer().Save()
        elif args.cantilever:
            from pxr import UsdPhysics

            from ffc.isaac_scene import joint

            first = stage.GetPrimAtPath(paths["cable_paths"][0])
            position = UsdGeom.XformCache().GetLocalToWorldTransform(first).ExtractTranslation()
            joint(stage, "/World/CableClamp", UsdPhysics.FixedJoint, None, str(first.GetPath()), position)
            stage.GetRootLayer().Save()
        from ffc.isaac_contacts import ContactMonitor

        contacts = ContactMonitor(stage, cfg["physics_dt_s"])
        report = {
            "configuration": cfg,
            "geometry_checks": validate_stage(stage, cfg),
            "assembly_success_tested": args.assembly
            and not (args.grasp_benchmark or args.transfer_benchmark),
            "transfer_benchmark_only": args.transfer_benchmark,
            "grasp_benchmark_only": args.grasp_benchmark,
            "pinch_benchmark_only": args.pinch_benchmark,
            "suction_attachment_enabled": args.assembly,
            "controller": "scripted privileged-state baseline" if args.assembly else "hold / tool exercise",
            "electrical_continuity_modeled": False,
            "material_parameters_calibrated": False,
            "gravity_compensation": "PhysX model-based generalized gravity feedforward; simulated mass model",
            "runtime": {
                "python": sys.version,
                "numpy": np.__version__,
                "usd_version": list(Usd.GetVersion()),
                "physx_extension": omni.kit.app.get_app()
                .get_extension_manager()
                .get_enabled_extension_id("omni.physx"),
            },
        }
        (args.output / "geometry-checks.json").write_text(json.dumps(report, indent=2))
        world = World(
            physics_dt=cfg["physics_dt_s"],
            rendering_dt=1 / 60,
            stage_units_in_meters=1,
            physics_prim_path="/World/PhysicsScene",
        )
        world.get_physics_context().set_solver_type(cfg.get("physics_solver", "TGS"))
        report["physics_solver"] = world.get_physics_context().get_solver_type()
        if cfg.get("disable_stabilization", False):
            world.get_physics_context().enable_stabilization(False)
        if args.cable_model == "shell":
            world.get_physics_context().enable_gpu_dynamics(True)
            world.get_physics_context().set_broadphase_type("GPU")
        arm = world.scene.add(SingleArticulation(prim_path=paths["robot_root"], name="fr3"))
        world.reset()
        names = arm.dof_names
        home = arm.get_joint_positions().copy()
        for i, q in enumerate(cfg["robot_home_rad"], 1):
            home[names.index(f"fr3v2_1_joint{i}")] = q
        home[names.index("clamp")] = -0.018
        if args.pinch_benchmark:
            if args.cable_model != "segments":
                raise ValueError("Pinch diagnostic requires the segmented cable")
            home[names.index("deploy")] = -0.018
            home[names.index("clamp")] = -0.0062
        arm.set_joint_positions(home)
        arm.set_joint_velocities(np.zeros_like(home))
        if args.pinch_benchmark:
            home[names.index("clamp")] = cfg["tool"]["closed_clamp_target_m"]
        arm.apply_action(ArticulationAction(joint_positions=home))
        report["dof_names"] = names
        # Compare independently extracted USD kinematics against the physics view.
        from ffc.isaac_kinematics import from_stage

        for _ in range(4):
            world.step(render=False, update_fabric=True)
        world.render()
        chain = from_stage(stage)
        q_arm = arm.get_joint_positions()[[names.index(f"fr3v2_1_joint{i}") for i in range(1, 8)]]
        link7 = next(p for p in stage.Traverse() if p.GetName() == "fr3v2_1_link7")
        actual = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(link7)).T
        expected = chain.forward(q_arm)
        report["kinematics_position_error_m"] = float(np.linalg.norm(actual[:3, 3] - expected[:3, 3]))
        if report["kinematics_position_error_m"] > 0.0002:
            raise RuntimeError(f"USD/physics kinematics mismatch: {report['kinematics_position_error_m']}")
        episode = None
        if args.assembly:
            from ffc.isaac_episode import Episode

            episode = Episode(stage, arm, cfg, args.output)
        if args.video:
            from ffc.isaac_recording import Recorder

            recorder = Recorder(stage, args.output / "experiment.mp4")
        samples = []
        physics_ticks = []
        world.add_physics_callback("ffc_timestep_audit", lambda dt: physics_ticks.append(float(dt)))
        executed_steps = 0
        dense_previous = None
        dense_reference = None
        dense_records = []
        dense_motion = {
            "samples": 0,
            "maximum_linear_speed_m_s": 0.0,
            "maximum_angular_speed_rad_s": 0.0,
            "maximum_surface_point_excursion_bound_m": 0.0,
        }
        steps = (
            max(args.steps, int(240 / cfg["physics_dt_s"]))
            if args.assembly
            else max(args.steps, int(10 / cfg["physics_dt_s"]) if args.exercise_tool else 0)
        )
        control_stride = max(
            1,
            round(cfg.get("robot_controller", {}).get("period_s", cfg["physics_dt_s"]) / cfg["physics_dt_s"]),
        )
        report["controller_period_s"] = control_stride * cfg["physics_dt_s"]
        fabric_stride = int(cfg.get("fabric_update_stride", 1))
        if fabric_stride < 1:
            raise ValueError("fabric_update_stride must be positive")
        sample_stride = max(1, round(0.1 / cfg["physics_dt_s"]))
        report["maximum_fabric_sync_stride_steps"] = fabric_stride
        signal.signal(signal.SIGINT, interrupt_run)
        signal.signal(signal.SIGTERM, interrupt_run)
        for step in range(steps):
            if interrupted_signal is not None:
                raise KeyboardInterrupt(f"Experiment interrupted by signal {interrupted_signal}")
            contacts.begin_step(step)
            if episode and step % control_stride == 0:
                episode.step(control_stride * cfg["physics_dt_s"])
                if episode.done:
                    break
            elif args.exercise_tool and step % control_stride == 0:
                phase = step / steps
                target = home.copy()
                target[names.index("deploy")] = -0.018 if 0.25 <= phase < 0.75 else 0
                target[names.index("clamp")] = (
                    -0.0062 if 0.40 <= phase < 0.60 else (0 if 0.15 <= phase < 0.85 else -0.018)
                )
                arm.apply_action(ArticulationAction(joint_positions=target))
            if step % control_stride == 0:
                arm.set_joint_efforts(arm._articulation_view.get_generalized_gravity_forces()[0])
            # Fabric copies poses out of the physics engine. Preserve every
            # physics/contact step, but permit fewer copies between consumers.
            # Always synchronize before control, recording, sampling and the
            # dense final cantilever motion audit.
            sync_fabric = (
                (step + 1) % fabric_stride == 0
                or (step + 1) % control_stride == 0
                or step % sample_stride == 0
                or step == steps - 1
                or bool(recorder and step * cfg["physics_dt_s"] + 1e-12 >= recorder.frames / recorder.fps)
                or (args.cantilever and step >= steps - round(0.025 / cfg["physics_dt_s"]))
            )
            world.step(render=False, update_fabric=sync_fabric)
            executed_steps += 1
            if (
                args.cantilever
                and args.cable_model == "segments"
                and step >= steps - round(0.025 / cfg["physics_dt_s"])
            ):
                from scipy.spatial.transform import Rotation

                dense_cache = UsdGeom.XformCache()
                matrices = np.asarray(
                    [
                        dense_cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path))
                        for path in paths["cable_paths"]
                    ]
                ).transpose(0, 2, 1)
                positions = matrices[:, :3, 3]
                rotations = Rotation.from_matrix(matrices[:, :3, :3])
                if dense_reference is None:
                    dense_reference = positions.copy(), rotations
                radius = (
                    np.linalg.norm(
                        [
                            cfg["cable"]["length_m"] / cfg["cable"]["segments"],
                            cfg["cable"]["width_m"],
                            cfg["cable"]["thickness_m"],
                        ]
                    )
                    / 2
                )
                excursion = (
                    np.linalg.norm(positions - dense_reference[0], axis=1)
                    + radius * (rotations * dense_reference[1].inv()).magnitude()
                )
                dense_motion["maximum_surface_point_excursion_bound_m"] = max(
                    dense_motion["maximum_surface_point_excursion_bound_m"], float(excursion.max())
                )
                if dense_previous is not None:
                    linear = float(
                        np.linalg.norm(positions - dense_previous[0], axis=1).max() / cfg["physics_dt_s"]
                    )
                    angular = float(
                        (rotations * dense_previous[1].inv()).magnitude().max() / cfg["physics_dt_s"]
                    )
                    dense_motion["maximum_linear_speed_m_s"] = max(
                        dense_motion["maximum_linear_speed_m_s"], linear
                    )
                    dense_motion["maximum_angular_speed_rad_s"] = max(
                        dense_motion["maximum_angular_speed_rad_s"], angular
                    )
                    dense_motion["samples"] += 1
                    dense_records.append(
                        {
                            "step": step,
                            "maximum_linear_speed_m_s": linear,
                            "maximum_angular_speed_rad_s": angular,
                            "maximum_surface_excursion_bound_m": float(excursion.max()),
                        }
                    )
                dense_previous = positions.copy(), rotations
            if episode or args.pinch_benchmark or args.exercise_tool:
                contacts.gate()
            if recorder and step * cfg["physics_dt_s"] + 1e-12 >= recorder.frames / recorder.fps:
                body = stage.GetPrimAtPath("/World/Tool/Body")
                position = (
                    UsdGeom.XformCache().GetLocalToWorldTransform(body).Transform(Gf.Vec3d(0, 0.120, 0.086))
                )
                cable_center = np.asarray(cfg["cable"]["start_xyz_m"]) + [cfg["cable"]["length_m"] / 2, 0, 0]
                recorder.aim(
                    episode.patch()
                    if episode
                    else (position if args.exercise_tool or args.pinch_benchmark else cable_center),
                    UsdGeom.XformCache()
                    .GetLocalToWorldTransform(body)
                    .TransformDir(Gf.Vec3d(0.075, -0.025, 0.025))
                    if episode or args.exercise_tool or args.pinch_benchmark
                    else None,
                )
                world.render()
                if args.cantilever:
                    phase_name = "cantilever_gravity_response"
                elif args.pinch_benchmark:
                    phase_name = "mechanical_pinch_hold"
                elif args.exercise_tool:
                    fraction = step / steps
                    phase_name = next(
                        name
                        for end, name in [
                            (0.15, "parked_above_pickup_plane"),
                            (0.25, "lower_jaw_opening"),
                            (0.40, "jaw_deployment"),
                            (0.60, "pinch_closure"),
                            (0.75, "pinch_release"),
                            (0.85, "lateral_retraction"),
                            (1.01, "jaw_parking"),
                        ]
                        if fraction < end
                    )
                else:
                    phase_name = "cable_settling"
                recorder.write(
                    episode.phases[episode.phase_index][0] if episode else phase_name,
                    (step + 1) * cfg["physics_dt_s"],
                )
            if step % sample_stride == 0 or step == steps - 1:
                cache = UsdGeom.XformCache()
                xyz = [
                    list(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(p)).ExtractTranslation())
                    for p in paths["cable_paths"]
                ]
                if args.cable_model == "shell":
                    from ffc.isaac_shell import shell_points

                    xyz = shell_points(stage)
                q = arm.get_joint_positions()
                applied = arm.get_applied_action()
                drive_stiffness, drive_damping = arm._articulation_view.get_gains()
                if not np.isfinite(xyz).all() or not np.isfinite(q).all():
                    raise RuntimeError(f"Nonfinite simulation state at step {step}")
                samples.append(
                    {
                        "step": step,
                        "cable_xyz_m": xyz,
                        "joint_positions": q.tolist(),
                        "joint_position_targets": np.asarray(applied.joint_positions).tolist(),
                        "joint_velocity_targets": np.asarray(applied.joint_velocities).tolist(),
                        "joint_applied_efforts": np.asarray(applied.joint_efforts).tolist(),
                        "joint_drive_stiffness": np.asarray(drive_stiffness)[0].tolist(),
                        "joint_drive_damping": np.asarray(drive_damping)[0].tolist(),
                        "tool_patch_xyz_m": list(
                            cache.GetLocalToWorldTransform(stage.GetPrimAtPath("/World/Tool/Body")).Transform(
                                Gf.Vec3d(0, 0.120, 0.086)
                            )
                        ),
                    }
                )
                if args.cable_model == "segments":
                    last_body = stage.GetPrimAtPath(paths["cable_paths"][-1])
                    half_length = cfg["cable"]["length_m"] / cfg["cable"]["segments"] / 2
                    samples[-1]["cable_tip_xyz_m"] = list(
                        cache.GetLocalToWorldTransform(last_body).Transform(Gf.Vec3d(half_length, 0, 0))
                    )
                (args.output / "live-state.json").write_text(json.dumps(samples[-1], indent=2))
        if episode and not episode.done:
            raise RuntimeError("Assembly experiment exceeded the simulation time limit")
        report["timestep_audit"] = {
            "requested_steps": executed_steps,
            "observed_physics_callbacks": len(physics_ticks),
            "observed_simulated_time_s": sum(physics_ticks),
            "expected_simulated_time_s": executed_steps * cfg["physics_dt_s"],
        }
        if len(physics_ticks) != executed_steps or not np.isclose(
            sum(physics_ticks), executed_steps * cfg["physics_dt_s"], atol=1e-6
        ):
            raise RuntimeError(f"Physics/render clock mismatch: {report['timestep_audit']}")
        if args.exercise_tool and not args.assembly:
            positions = np.asarray([sample["joint_positions"] for sample in samples])
            strokes = {name: float(np.ptp(positions[:, names.index(name)])) for name in ("deploy", "clamp")}
            report["tool_strokes_m"] = strokes
            if strokes["deploy"] < 0.017 or strokes["clamp"] < 0.017:
                raise RuntimeError(f"Tool did not complete its commanded strokes: {strokes}")
        if recorder:
            recorder.close()
            report["video_frames"] = recorder.frames
            recorder = None
            if report["video_frames"] == 0:
                raise RuntimeError("No RTX video frames were produced")
        z_values = [p[2] for sample in samples for p in sample["cable_xyz_m"]]
        if args.cantilever:
            c, s = cfg["cable"], cfg["shell"]
            elements = s["longitudinal_elements"] if args.cable_model == "shell" else c["segments"]
            free_length = c["length_m"] * (1 - 1 / elements)
            line_load = c["mass_kg"] * 9.81 / c["length_m"]
            ei = s["youngs_modulus_pa"] * c["width_m"] * c["thickness_m"] ** 3 / 12
            analytic = line_load * free_length**4 / (8 * ei)
            if args.cable_model == "shell":
                last_row = np.asarray(samples[-1]["cable_xyz_m"])[-(s["width_elements"] + 1) :]
                tip_z = last_row[:, 2].mean()
            else:
                last = stage.GetPrimAtPath(paths["cable_paths"][-1])
                tip_z = (
                    UsdGeom.XformCache()
                    .GetLocalToWorldTransform(last)
                    .Transform(Gf.Vec3d(c["length_m"] / c["segments"] / 2, 0, 0))[2]
                )
            measured = float(c["start_xyz_m"][2] - tip_z)
            report["cantilever"] = {
                "beam_prediction_m": analytic,
                "simulated_tip_sag_m": measured,
                "ratio_to_beam_prediction": measured / analytic,
                "beam_agreement_within_25_percent": bool(abs(measured / analytic - 1) <= 0.25),
                "hardware_calibrated": False,
                "note": "Finite-width shell and Poisson effects differ from 1D beam theory"
                if args.cable_model == "shell"
                else (
                    "Clamping the first full segment adds a discrete boundary compliance; "
                    "compare the exact linear hinge-chain prediction too."
                ),
            }
            if args.cable_model == "segments":
                from ffc.cantilever_reference import hinge_chain_reference

                reference = hinge_chain_reference(c)
                discrete = reference["nonlinear_hinge_chain_prediction_m"]
                report["cantilever"].update(reference)
                report["cantilever"].update(
                    {
                        "ratio_to_hinge_chain_prediction": measured / discrete,
                        "hinge_chain_agreement_within_20_percent": bool(abs(measured / discrete - 1) <= 0.20),
                    }
                )
            if args.cable_model == "segments":
                recent_tip_heights = [sample["cable_tip_xyz_m"][2] for sample in samples[-4:]]
                settling_range = float(np.ptp(recent_tip_heights))
                angular_speeds = [
                    np.linalg.norm(
                        UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(path)).GetAngularVelocityAttr().Get()
                    )
                    * np.pi
                    / 180
                    for path in paths["cable_paths"]
                ]
                maximum_angular_speed = float(max(angular_speeds))
                static_tolerance = cfg.get("static_shape_relative_tolerance", 0.20)
                report["cantilever"].update(
                    {
                        "declared_static_shape_relative_tolerance": static_tolerance,
                        "maximum_cable_angular_speed_rad_s": maximum_angular_speed,
                        "dense_pose_motion_final_25_ms": dense_motion,
                        "equilibrium_velocity_source": (
                            "Finite differences of every link pose at each physics step; "
                            "reported velocities retained separately"
                        ),
                        "angular_speed_below_0_01_rad_s": maximum_angular_speed < 0.01,
                        "static_shape_excursion_tolerance_m": 0.01 * c["thickness_m"],
                        "static_shape_agreement_qualified": bool(
                            dense_motion["samples"] >= 10
                            and dense_motion["maximum_surface_point_excursion_bound_m"]
                            < 0.01 * c["thickness_m"]
                            and settling_range < 0.02 * discrete
                            and abs(measured / discrete - 1) <= static_tolerance
                        ),
                        "last_four_samples_tip_height_range_m": settling_range,
                        "equilibrium_window_duration_s": (samples[-1]["step"] - samples[-4]["step"])
                        * cfg["physics_dt_s"]
                        if len(samples) >= 4
                        else 0,
                        "equilibrium_window_within_2_percent": bool(
                            len(samples) >= 4 and settling_range < 0.02 * discrete
                        ),
                        "equilibrium_agreement_qualified": bool(
                            len(samples) >= 4
                            and settling_range < 0.02 * discrete
                            and abs(measured / discrete - 1) <= 0.20
                            and dense_motion["samples"] >= 10
                            and dense_motion["maximum_angular_speed_rad_s"] < 0.01
                            and dense_motion["maximum_linear_speed_m_s"] < 0.0001
                        ),
                    }
                )
        report["physics"] = {
            "steps": executed_steps,
            "simulated_time_s": sum(physics_ticks),
            "min_cable_center_z_m": min(z_values),
            "max_cable_center_z_m": max(z_values),
            "finite_state": True,
            "tool_exercised": args.exercise_tool,
            "cable_above_desk": min(z_values) > -cfg["cable"]["thickness_m"],
        }
        if not report["physics"]["cable_above_desk"]:
            raise RuntimeError(f"Cable penetrated desk: minimum center z={min(z_values)}")
        if not args.assembly and not args.cantilever and not args.pinch_benchmark:
            final_z = float(np.median(np.asarray(samples[-1]["cable_xyz_m"])[:, 2]))
            report["physics"]["final_median_cable_z_m"] = final_z
            if not 0 <= final_z <= cfg["cable"]["thickness_m"] * 0.85:
                raise RuntimeError(f"Cable did not settle to its finite-thickness desk contact: z={final_z}")
        if (
            args.cantilever
            and not 0.000001 < report["cantilever"]["simulated_tip_sag_m"] < cfg["cable"]["length_m"]
        ):
            raise RuntimeError("Cantilever did not show a finite, plausible gravity response")
        if args.pinch_benchmark:
            c = cfg["cable"]
            last = stage.GetPrimAtPath(paths["cable_paths"][-1])
            tip = np.asarray(
                UsdGeom.XformCache()
                .GetLocalToWorldTransform(last)
                .Transform(Gf.Vec3d(c["length_m"] / c["segments"] / 2, 0, 0))
            )
            report["pinch_retention"] = {"cable_tip_xyz_m": tip.tolist(), "vacuum_used": False}
            if tip[2] < c["start_xyz_m"][2] - 0.01:
                raise RuntimeError("Pure mechanical pinch failed to retain the cable")
        # RTX capture through Replicator, using authored camera optics.
        import omni.replicator.core as rep
        from PIL import Image

        captures = []
        for path in paths["camera_paths"]:
            rp = rep.create.render_product(path, (1200, 800))
            rgb = rep.AnnotatorRegistry.get_annotator("rgb")
            rgb.attach(rp)
            rep.orchestrator.step(rt_subframes=4, delta_time=0.0, pause_timeline=False)
            data = np.asarray(rgb.get_data())
            if data.size == 0 or data[..., :3].std() < 1:
                raise RuntimeError(f"Empty or constant camera output: {path}")
            name = path.rsplit("/", 1)[-1] + ".png"
            Image.fromarray(data).save(args.output / name)
            captures.append(name)
            rgb.detach(rp)
            rp.destroy()
        report["captures"] = captures
        report["status"] = "PASS"
        (args.output / "results.json").write_text(json.dumps(report, indent=2))
        (args.output / "trajectory.json").write_text(json.dumps(samples, indent=2))
        stage.GetRootLayer().Export(str(args.output / "settled.usda"))
        print("FFC_RESULT " + json.dumps(report), flush=True)
        if args.gui:
            from isaacsim.core.utils.viewports import set_camera_view

            set_camera_view(eye=np.array([1.35, -1.3, 1.05]), target=np.array([0.3, 0, 0.25]))
            while app.is_running():
                world.step(render=True)
    except (Exception, KeyboardInterrupt) as exc:
        if "stage" in locals():
            stage.GetRootLayer().Export(str(args.output / "failed-state.usda"))
        report.update(
            status="ABORTED" if isinstance(exc, KeyboardInterrupt) else "FAIL",
            error=str(exc),
            traceback=traceback.format_exc(),
        )
        (args.output / "results.json").write_text(json.dumps(report, indent=2))
        if "samples" in locals():
            (args.output / "trajectory.json").write_text(json.dumps(samples, indent=2))
        if recorder:
            for _ in range(30):
                recorder.write(
                    report["status"] + " - frozen last frame: " + str(exc)[:70],
                    locals().get("step", 0) * cfg["physics_dt_s"],
                )
        raise
    finally:
        if "physics_ticks" in locals():
            report["timestep_audit"] = {
                "requested_steps": executed_steps,
                "observed_physics_callbacks": len(physics_ticks),
                "observed_simulated_time_s": sum(physics_ticks),
                "expected_simulated_time_s": executed_steps * cfg["physics_dt_s"],
                "clock_consistent": bool(
                    len(physics_ticks) == executed_steps
                    and np.isclose(sum(physics_ticks), executed_steps * cfg["physics_dt_s"], atol=1e-6)
                ),
            }
            (args.output / "results.json").write_text(json.dumps(report, indent=2))
        if "dense_records" in locals() and dense_records:
            (args.output / "dense-motion.json").write_text(json.dumps(dense_records, indent=2))
        if "contacts" in locals():
            (args.output / "rigid-contacts.json").write_text(json.dumps(contacts.records, indent=2))
            (args.output / "contact-peaks.json").write_text(
                json.dumps(list(contacts.peaks.values()), indent=2)
            )
            (args.output / "joint-events.json").write_text(json.dumps(contacts.joint_events, indent=2))
            contacts.close()
        if recorder:
            recorder.close()
        app.close(exit_code=0 if report.get("status") == "PASS" else 1)


if __name__ == "__main__":
    main()
