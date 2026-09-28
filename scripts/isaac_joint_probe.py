"""Isaac-only unloaded FR3 joint tracking probe. No task state or vision controller.

A fresh output is mandatory. This runner is prepared for integration testing;
CPU trajectory tests do not qualify its Isaac execution or collision clearance.
"""

import argparse
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--joint", type=int, choices=range(1, 8), default=7)
    p.add_argument("--case", choices=["probe", "cancel"], default="probe")
    a = p.parse_args()
    if a.output.exists():
        p.error("Fresh output directory required")
    a.output.mkdir(parents=True)
    report = dict(
        status="NOT_STARTED",
        engine="Isaac Sim",
        scope="Unloaded FR3 joint tracking only",
        camera_control=False,
        contact_qualified=False,
        physical_robot_connected=False,
        case=a.case,
    )
    source = a.output / "source"
    source.mkdir()
    for name in [
        "scripts/isaac_joint_probe.py",
        "src/ffc/joint_trajectory.py",
        "src/ffc/isaac_recording.py",
        "src/ffc/isaac_scene.py",
        "src/ffc/isaac_contacts.py",
        "config/fr3-isaac-source.json",
    ]:
        path = ROOT / name
        (source / path.name).write_bytes(path.read_bytes())
    (source / "hashes.json").write_text(
        json.dumps({x.name: hashlib.sha256(x.read_bytes()).hexdigest() for x in source.iterdir()}, indent=2)
    )
    app = recorder = None
    samples = []
    phase_results = []
    try:
        from isaacsim import SimulationApp

        app = SimulationApp(
            {
                "headless": True,
                "renderer": "RaytracedLighting",
                "extra_args": ["--allow-root", "--/telemetry/enableAnonymousData=false"],
            }
        )
        import math

        import numpy as np
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.types import ArticulationAction
        from pxr import PhysxSchema, UsdGeom, UsdLux, UsdPhysics

        from ffc.isaac_contacts import ContactMonitor
        from ffc.isaac_recording import Recorder
        from ffc.isaac_scene import box, camera, drive
        from ffc.joint_trajectory import reference, validate_move

        context = omni.usd.get_context()
        context.new_stage()
        stage = context.get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1)
        stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/World").GetPrim())
        robot = UsdGeom.Xform.Define(stage, "/World/FR3").GetPrim()
        robot.GetReferences().AddReference(str(ROOT / "third_party/franka_isaac/fr3v2_1/fr3v2_1.usda"))
        settings = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath("/World/FR3/Geometry/base"))
        settings.CreateEnabledSelfCollisionsAttr(True)
        settings.CreateSolverPositionIterationCountAttr(255)
        settings.CreateSolverVelocityIterationCountAttr(4)
        home = np.array([0.0, -0.55, 0.0, -2.15, 0.0, 1.65, 0.7853981634])
        lower = []
        upper = []
        for i, q in enumerate(home, 1):
            joint = UsdPhysics.RevoluteJoint.Get(stage, f"/World/FR3/Physics/fr3v2_1_joint{i}")
            if not joint:
                raise RuntimeError(f"Missing official joint {i}")
            lower.append(math.radians(joint.GetLowerLimitAttr().Get()))
            upper.append(math.radians(joint.GetUpperLimitAttr().Get()))
            # Same assumed drive profile as the earlier Isaac workcell; not identified FR3 firmware.
            drive(
                joint,
                "angular",
                5 * (40 if i < 5 else 15),
                5 * (4 if i < 5 else 1.5),
                math.degrees(q),
                87 if i < 5 else 12,
            )
        UsdLux.DomeLight.Define(stage, "/World/Light").CreateIntensityAttr(300)
        box(stage, "/World/VisualGround", (0, 0, -0.03), (3, 3, 0.02), (0.18, 0.22, 0.24), collision=False)
        camera(stage, "/World/Cameras/Overview", (1.5, -1.5, 1.15), (0.25, 0, 0.45), 35)
        world = World(physics_dt=0.001, rendering_dt=1 / 30, stage_units_in_meters=1)
        world.get_physics_context().set_solver_type("PGS")
        monitor = ContactMonitor(stage, 0.001)
        arm = world.scene.add(SingleArticulation(prim_path="/World/FR3/Geometry/base", name="fr3"))
        world.reset()
        names = arm.dof_names
        if len(names) != 7:
            raise RuntimeError("Unloaded seven-joint articulation required")
        indices = [names.index(f"fr3v2_1_joint{i}") for i in range(1, 8)]
        initial = np.empty(7)
        initial[indices] = home
        arm.set_joint_positions(initial)  # Initialization only; all subsequent motion uses drives.
        arm.set_joint_velocities(np.zeros(7))
        arm.apply_action(ArticulationAction(joint_positions=initial))
        recorder = Recorder(
            stage,
            a.output / "probe.mp4",
            fps=25,
            scope_label="Isaac unloaded joint tracking / no cable, PCB or camera control",
        )
        recorder.aim((0.25, 0, 0.5), eye_offset=(0.7, -0.7, 0.3))
        ticks = []
        world.add_physics_callback("probe_clock", lambda dt: ticks.append(float(dt)))
        start_time = world.current_time
        integral = np.zeros(7)
        commands = 0
        max_tracking = 0.0
        max_velocity = 0.0
        cancel_state = None
        max_cancel_drift = 0.0
        phases = [
            ("settle", home, 3.0),
            ("joint_probe", home + np.eye(7)[a.joint - 1] * 0.005, 2.0),
            ("return", home, 2.0),
        ]
        previous = home.copy()
        for phase, target, duration in phases:
            validate_move(previous, target, lower, upper, duration, 0.00501, 0.01, 0.02)
            phase_start = world.current_time
            done = False
            while world.current_time - phase_start < duration + 4.0:
                elapsed = world.current_time - phase_start
                q = np.array(arm.get_joint_positions())[indices]
                v = np.array(arm.get_joint_velocities())[indices]
                if not np.isfinite([q, v]).all():
                    raise RuntimeError("Nonfinite joint feedback")
                desired, velocity = reference(previous, target, elapsed, duration)
                if a.case == "cancel" and phase == "joint_probe" and elapsed >= duration / 2:
                    cancel_state = dict(
                        time_s=float(world.current_time), q_rad=q.tolist(), velocity_rad_s=v.tolist()
                    )
                    target = desired.copy()
                    previous = target.copy()
                    phase = "cancel_hold"
                    phase_start = world.current_time
                    elapsed = 0.0
                    duration = 0.25
                    velocity = np.zeros(7)
                if cancel_state is not None:
                    max_cancel_drift = max(max_cancel_drift, float(np.max(abs(q - cancel_state["q_rad"]))))
                    if max_cancel_drift > 0.001:
                        raise RuntimeError("Cancel drift exceeds 0.001 rad experiment bound")
                if np.max(abs(v)) > 0.03:
                    raise RuntimeError("Measured speed exceeds 0.03 rad/s experiment bound")
                sample_time = world.current_time
                max_tracking = max(max_tracking, float(np.max(abs(q - desired))))
                max_velocity = max(max_velocity, float(np.max(abs(v))))
                integral = np.clip(integral + 5 * 0.001 * (desired - q), -0.03, 0.03)
                positions = np.empty(7)
                velocities = np.empty(7)
                positions[indices] = np.clip(desired + integral, lower, upper)
                velocities[indices] = velocity
                arm.apply_action(ArticulationAction(joint_positions=positions, joint_velocities=velocities))
                if elapsed > 0.5 and np.max(abs(q - desired)) > 0.02:
                    raise RuntimeError("Tracking deviation exceeds 0.02 rad")
                monitor.begin_step(commands)
                world.step(render=False, update_fabric=True)
                monitor.gate()  # Independent experiment validity audit, not visual-policy input.
                commands += 1
                if commands % 20 == 0:
                    samples.append(
                        dict(
                            time_s=float(sample_time),
                            phase=phase,
                            q_rad=q.tolist(),
                            velocity_rad_s=v.tolist(),
                            reference_rad=desired.tolist(),
                        )
                    )
                if commands % 40 == 0:
                    before = world.current_time
                    recorder.write(phase, world.current_time - start_time)
                    if world.current_time != before:
                        raise RuntimeError("Rendering advanced physics")
                if elapsed >= duration and np.max(abs(q - target)) < 0.0002 and np.max(abs(v)) < 0.001:
                    done = True
                    break
            if not done:
                raise RuntimeError(f"{phase}: measured settling deadline exceeded")
            phase_results.append(
                dict(
                    phase=phase,
                    target_rad=target.tolist(),
                    measured_rad=q.tolist(),
                    velocity_rad_s=v.tolist(),
                    error_rad=float(np.max(abs(q - target))),
                    elapsed_s=float(world.current_time - phase_start),
                )
            )
            previous = target.copy()
            if phase == "cancel_hold":
                break
        if len(ticks) != commands or abs(sum(ticks) - (world.current_time - start_time)) > 1e-6:
            raise RuntimeError("Physics clock mismatch")
        (a.output / "joint-trace.json").write_text(json.dumps(samples, indent=2))
        recorder.close()
        recorder = None
        report.update(
            status="PASS",
            physics_steps=commands,
            physics_seconds=sum(ticks),
            joint=a.joint,
            amplitude_rad=0.005,
            max_tracking_error_rad=max_tracking,
            max_measured_velocity_rad_s=max_velocity,
            phases=phase_results,
            solver="PGS",
            cancel_state=cancel_state,
            max_cancel_drift_rad=max_cancel_drift,
            contact_pair_peaks=list(monitor.peaks.values()),
        )
    except BaseException as exc:
        report.update(status="FAILED", reason=f"{type(exc).__name__}: {exc}")
        raise
    finally:
        (a.output / "joint-trace.json").write_text(json.dumps(samples, indent=2))
        (a.output / "results.json").write_text(json.dumps(report, indent=2) + "\n")
        if recorder is not None:
            recorder.close()
        if app is not None:
            app.close()


if __name__ == "__main__":
    main()
