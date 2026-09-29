"""Exploratory flexible-cable side-entry contact bench, initialized aligned.

No robot, visual servo or exact Pi connector claim. Actor uses pad, feed encoder
and instrumented-fixture load equivalents; cable poses are offline only.
"""

import argparse
import json
import os
import sys
from collections import deque
from dataclasses import asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--case", choices=["open", "blocked", "load_dropout"], default="open")
    p.add_argument("--dt", type=float, choices=[0.000125, 0.0000625, 0.00003125], default=0.000125)
    p.add_argument("--segments", type=int, choices=[100, 200], default=100)
    p.add_argument("--live-feed", type=Path)
    p.add_argument("--orientation", choices=["inline", "side"], default="side")
    a = p.parse_args()
    from ffc.cable_spec import load_spec, provenance_record
    from ffc.profile_cable import create_profile_cable

    spec_path = ROOT / "config/cables/rpi-camera-standard-mini-200-rev2.json"
    spec = load_spec(spec_path)
    a.output.mkdir(parents=True, exist_ok=False)
    (a.output / "source.py").write_bytes(Path(__file__).read_bytes())
    (a.output / "pinch_skill.py").write_bytes((ROOT / "src/ffc/pinch_skill.py").read_bytes())
    for source in [
        spec_path,
        ROOT / "src/ffc/profile_cable.py",
        ROOT / "src/ffc/insertion_skill.py",
        ROOT / "src/ffc/cable_spec.py",
        ROOT / "src/ffc/isaac_scene.py",
        ROOT / "src/ffc/isaac_contacts.py",
        ROOT / "src/ffc/isaac_recording.py",
    ]:
        (a.output / source.name).write_bytes(source.read_bytes())
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
    from pxr import Gf, PhysxSchema, UsdGeom, UsdLux, UsdPhysics, UsdShade

    from ffc.insertion_skill import FeedObservation, InsertionFeed
    from ffc.isaac_contacts import ContactMonitor
    from ffc.isaac_recording import Recorder
    from ffc.isaac_scene import box, camera, drive, joint, pose
    from ffc.pinch_skill import PinchObservation, PinchSkill

    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, "Z")
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/World").GetPrim())
    dt, z0, initial_closing = a.dt, 0.055, 0.0045
    control_stride = round(0.005 / dt)
    groups = {"fixed": 0.12, "lower": 0.02, "upper": 0.02}
    for name, mass in groups.items():
        prim = UsdGeom.Xform.Define(stage, "/World/Tool/" + name).GetPrim()
        offset = initial_closing if name == "lower" else (-initial_closing if name == "upper" else 0)
        pose(prim, (0, 0, z0 + offset))
        UsdPhysics.RigidBodyAPI.Apply(prim)
        api = UsdPhysics.MassAPI.Apply(prim)
        api.CreateMassAttr(mass)
        com_y = -0.03 if name == "fixed" else -0.004
        com = (0, com_y, 0) if a.orientation == "inline" else (0.0105 - com_y, 0.012, 0)
        api.CreateCenterOfMassAttr(Gf.Vec3f(*com))
        api.CreateDiagonalInertiaAttr(Gf.Vec3f(0.00004 if name == "fixed" else 0.000002))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateSolverPositionIterationCountAttr(255)
        rb.CreateSolverVelocityIterationCountAttr(8)
        rb.CreateEnableCCDAttr(True)
        visual = UsdGeom.Xform.Define(stage, str(prim.GetPath()) + "/Visual")
        if a.orientation == "inline":
            visual.AddTranslateOp().Set(
                Gf.Vec3d(-0.0102985785382651, 0.2482103195204206, -0.0143601205939444)
            )
        else:
            # Rotate the CAD about its pad center, then set a 12 mm insulated-body setback.
            visual.AddTranslateOp().Set(
                Gf.Vec3d(-0.2377103195204206, 0.0017014214617349, -0.0143601205939444)
            )
            visual.AddRotateZOp().Set(90)
        child = stage.DefinePrim(str(visual.GetPath()) + "/SupplierAndFinger")
        child.GetReferences().AddReference(
            str(ROOT / "outputs/compact-fingers-002/compact-fingers.usda"), "/CompactTool/" + name
        )
    lift = joint(
        stage, "/World/Joints/lift", UsdPhysics.PrismaticJoint, None, "/World/Tool/fixed", (0, 0, z0)
    )
    lift.CreateAxisAttr("Y")
    lift.CreateLowerLimitAttr(-0.005)
    lift.CreateUpperLimitAttr(0.0005)
    lift_drive = drive(lift, "linear", 200000, 100, 0, 30)
    jaw_drives = {}
    pad_paths = []
    for name, sign in [("lower", 1), ("upper", -1)]:
        path = "/World/Tool/" + name
        j = joint(stage, "/World/Joints/" + name, UsdPhysics.PrismaticJoint, "/World/Tool/fixed", path)
        j.CreateAxisAttr("Z")
        j.CreateLowerLimitAttr(0 if sign == 1 else -0.005)
        j.CreateUpperLimitAttr(0.005 if sign == 1 else 0)
        jaw_drives[name] = drive(j, "linear", 10000, 2, sign * initial_closing, 0.8)
        # The visible CAD remains intact. Only the contact pads are colliders in
        # this isolated contact bench; whole-tool collision qualification is separate.
        pad = box(
            stage,
            path + "/PadCollision",
            (0, 0.0105 if a.orientation == "inline" else 0.012, -sign * 0.00515),
            (0.006, 0.003, 0.0003) if a.orientation == "inline" else (0.003, 0.006, 0.0003),
            (0.1, 0.12, 0.13),
        )
        UsdGeom.Imageable(pad).MakeInvisible()
        pad_paths.append(str(pad.GetPath()) + "/Shape")
    material = UsdShade.Material.Define(stage, "/World/PadMaterial")
    friction = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    friction.CreateStaticFrictionAttr(0.5)
    friction.CreateDynamicFrictionAttr(0.4)
    friction.CreateRestitutionAttr(0)
    for path in pad_paths:
        UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(path)).Bind(
            material, UsdShade.Tokens.weakerThanDescendants, "physics"
        )
    box(
        stage,
        "/World/Support",
        (0, 0.113, (z0 - 0.00015) / 2),
        (0.025, 0.190, z0 - 0.00015),
        (0.25, 0.28, 0.3),
    )
    box(stage, "/World/Ground", (0, 0, -0.006), (0.4, 0.4, 0.012), (0.2, 0.23, 0.26))
    cable_paths, profile = create_profile_cable(stage, spec, z0, a.segments)
    # An explicitly assumed open-channel coupon, not the Pi CAD socket.
    # Mouth at Y=-1 mm, 0.6 mm gap for the 0.3 mm terminal; 12 mm inner width.
    fixture_paths = []
    for name, center, size in [
        ("floor", (0, -0.003, z0 - 0.0008), (0.014, 0.004, 0.001)),
        ("roof", (0, -0.003, z0 + 0.0008), (0.014, 0.004, 0.001)),
        ("left", (-0.0065, -0.003, z0), (0.001, 0.004, 0.0006)),
        ("right", (0.0065, -0.003, z0), (0.001, 0.004, 0.0006)),
    ]:
        fixture = box(stage, "/World/Slot/" + name, center, size, (0.6, 0.62, 0.65), mass=0.05)
        UsdPhysics.RigidBodyAPI(fixture).CreateKinematicEnabledAttr(True)
        PhysxSchema.PhysxRigidBodyAPI(fixture).CreateEnableCCDAttr(False)
        fixture_paths.append(str(fixture.GetPath()) + "/Shape")
    if a.case == "blocked":
        block = box(
            stage, "/World/Slot/block", (0, -0.00125, z0), (0.012, 0.0005, 0.0006), (0.8, 0.2, 0.1), mass=0.05
        )
        UsdPhysics.RigidBodyAPI(block).CreateKinematicEnabledAttr(True)
        PhysxSchema.PhysxRigidBodyAPI(block).CreateEnableCCDAttr(False)
        fixture_paths.append(str(block.GetPath()) + "/Shape")
    (a.output / "sections.json").write_text(json.dumps(profile, indent=2) + "\n")
    UsdLux.DomeLight.Define(stage, "/World/Light").CreateIntensityAttr(900)
    camera(stage, "/World/Cameras/Overview", (0.22, 0.25, 0.23), (0, 0.060, 0.052), 48)
    world = World(physics_dt=dt, rendering_dt=1 / 24, stage_units_in_meters=1)
    world.get_physics_context().set_solver_type("PGS")
    handles = {
        name: world.scene.add(SingleRigidPrim(prim_path="/World/Tool/" + name, name=name)) for name in groups
    }
    monitor = ContactMonitor(stage, dt)
    # Report from pads and instrumented fixture; omit cable/support report traffic.
    # Contact counterpart identities never enter the controller.
    for path in cable_paths:
        stage.GetPrimAtPath(path).RemoveAPI(PhysxSchema.PhysxContactReportAPI)
    world.reset()
    cable_handles = [
        SingleRigidPrim(prim_path=path, name=f"offline_segment_{i}") for i, path in enumerate(cable_paths)
    ]
    for handle in cable_handles:
        handle.initialize()
    recorder = Recorder(
        stage,
        a.output / "insertion.mp4",
        fps=24,
        scope_label="Side-entry contact pilot / prealigned channel / NOT Pi or FR3 integration",
    )
    recorder.aim((0, -0.001, z0), eye_offset=(-0.025, -0.04, 0.023))
    controller = PinchSkill()
    controller.closing = initial_closing
    feed = InsertionFeed()
    feed_command = None
    feed_history = deque(maxlen=round(0.01 / dt))
    history = deque(maxlen=round(0.01 / dt))
    trace, offline = [], []
    command = controller.command()
    sensor_sum = np.zeros(2)
    initial_time = world.current_time
    final = {}
    fault_time = None
    fault_lift = None
    dropout_injected = False
    previous_command = None
    try:
        for i in range(round(10 / dt)):
            if command != previous_command:
                jaw_drives["lower"].GetTargetPositionAttr().Set(command.closing_travel_m)
                jaw_drives["upper"].GetTargetPositionAttr().Set(-command.closing_travel_m)
                lift_drive.GetTargetPositionAttr().Set(0)
                previous_command = command
            if feed_command is not None:
                lift_drive.GetTargetPositionAttr().Set(-feed_command.target_m)
            monitor.begin_step(i)
            world.step(render=False, update_fabric=True)
            # Sensor adapter sees only impulses applied at its known pad surface.
            # Counterpart identity is discarded and never enters the skill.
            forces = np.zeros(2)
            for contact in monitor.current:
                for index, path in enumerate(pad_paths):
                    if path in (contact["collider0"], contact["collider1"]):
                        forces[index] += contact["sum_contact_force_magnitudes_n"]
            fixture_force = sum(
                c["sum_contact_force_magnitudes_n"]
                for c in monitor.current
                if c["collider0"] in fixture_paths or c["collider1"] in fixture_paths
            )
            feed_history.append(fixture_force)
            history.append(forces)
            now = (i + 1) * dt
            if i % control_stride == 0:
                sensor_sum = np.mean(history, axis=0)
                base_z = float(handles["fixed"].get_world_pose()[0][2])
                lower_z = float(handles["lower"].get_world_pose()[0][2])
                upper_z = float(handles["upper"].get_world_pose()[0][2])
                if a.case == "load_dropout" and feed_command is not None and feed_command.target_m >= 0.0005:
                    sensor_sum[:] = 0
                    dropout_injected = True
                observation = PinchObservation(
                    now,
                    lower_z - base_z,
                    base_z - upper_z,
                    base_z - z0,
                    float(sensor_sum[0]),
                    float(sensor_sum[1]),
                )
                if feed_command is None:
                    command = controller.update(now, observation)
                if command.state == "lift":
                    feed_observation = FeedObservation(
                        now,
                        -float(handles["fixed"].get_world_pose()[0][1]),
                        float(sensor_sum[0]),
                        float(sensor_sum[1]),
                        float(np.mean(feed_history)),
                    )
                    feed_command = feed.update(now, feed_observation)
                trace.append(
                    {
                        "time_s": now,
                        "observation": asdict(observation),
                        "command": asdict(command),
                        "feed_observation": asdict(feed_observation) if feed_command else None,
                        "feed_command": asdict(feed_command) if feed_command else None,
                    }
                )
            if i % round(1 / (24 * dt)) == 0:
                recorder.write(feed_command.state if feed_command else command.state, now)
                if cable_handles:
                    poses = [handle.get_world_pose() for handle in cable_handles]
                    offline.append(
                        {
                            "time_s": now,
                            "positions_m": [pos.tolist() for pos, q in poses],
                            "quaternions_wxyz": [q.tolist() for pos, q in poses],
                            "tool_poses": {
                                name: {
                                    "xyz": handle.get_world_pose()[0].tolist(),
                                    "quat": handle.get_world_pose()[1].tolist(),
                                }
                                for name, handle in handles.items()
                            },
                        }
                    )
                if a.live_feed:
                    from ffc.lab_feed import publish_snapshot

                    rgb = np.concatenate(
                        [np.asarray(ann.get_data())[..., :3] for ann in recorder.annotators], axis=1
                    )
                    publish_snapshot(
                        rgb,
                        a.live_feed,
                        "workcell",
                        recorder.frames,
                        "Presented flexible-cable motion pilot / " + command.state,
                        a.output.name,
                    )
                print(
                    json.dumps(
                        {
                            "time_s": now,
                            "state": feed_command.state if feed_command else command.state,
                            "load_n": sensor_sum.tolist(),
                        }
                    ),
                    flush=True,
                )
            terminal = feed_command is not None and feed_command.state in {
                "stopped",
                "retracted",
                "travel_complete_unverified",
            }
            if (command.state == "fault" or terminal) and fault_time is None:
                fault_time, fault_lift = now, observation.lift_m
            finished = fault_time is not None and now - fault_time >= 0.25
            if finished:
                final = {
                    "command": asdict(command),
                    "observation": asdict(observation),
                    "time_s": now,
                    "feed_command": asdict(feed_command) if feed_command else None,
                    "feed_observation": asdict(feed_observation) if feed_command else None,
                }
                recorder.write(
                    (feed_command.state + " / " + feed_command.reason)
                    if feed_command
                    else (command.state + " / " + command.reason),
                    now,
                )
                break
        recorder.close()
        final_positions = [handle.get_world_pose()[0].tolist() for handle in cable_handles]
        report = {
            "case": a.case,
            "tool_orientation": a.orientation,
            "scope": "Prealigned flexible-cable channel pilot; no robot, camera control or Pi connector",
            "fixture": {
                "gap_m": 0.0006,
                "width_m": 0.012,
                "mouth_y_m": -0.001,
                "length_m": 0.004,
                "status": "exploratory geometry; not supplier dimensions",
            },
            "seating_verified": False,
            "offline_tip_depth_m": -0.001 - (final_positions[0][1] - profile[0]["length_m"] / 2),
            "cable_spec": provenance_record(spec),
            "segment_count": a.segments,
            "initial_closing_travel_m": initial_closing,
            "control_period_s": control_stride * dt,
            "offline_final_positions_m": final_positions,
            "contact_mechanics_qualified": False,
            "static_benchmark_settings_inherited_without_qualification": False,
            "body_drag_per_s": 20,
            "joint_viscosity_scale": 1e-5,
            "final": final,
            "physics_steps": i + 1,
            "physics_dt_s": dt,
            "physics_elapsed_s": world.current_time - initial_time,
            "source_of_control": (
                "Measured tool body displacement as encoder equivalent; "
                "10 ms mean pad and instrumented-fixture contact impulse magnitude / timestep"
            ),
            "camera_control": False,
            "ros_connected": False,
            "cable_pose_control": False,
            "grasp_attachment": False,
            "dropout_injected": dropout_injected,
            "fault_time_s": fault_time,
            "post_fault_lift_drift_m": None if fault_lift is None else observation.lift_m - fault_lift,
            "assumptions": {
                "cable_mass_kg": sum(section["mass_kg"] for section in profile),
                "pad_center_distance_from_tip_m": 0.0105 if a.orientation == "inline" else 0.012,
                "pad_friction_static_dynamic": [0.5, 0.4],
                "jaw_drive_force_cap_N": 0.8,
                "jaw_stiffness_N_m": 10000,
                "jaw_damping_N_s_m": 2,
            },
            "limitations": [
                "Free-articulation full cable and its contact dynamics are not yet qualified",
                "Source profile uses equivalent material and assumed terminal dimensions",
                "100/200 rigid sections approximate width by midpoint sampling",
                "Both contact faces are a visual convention; pads exclude exposed strips on both faces",
                "Artificial body drag is not identified material damping",
                "Closure starts from a presented 1 mm gap, not a camera-guided approach",
                "Controller update rate 200 Hz is a proposed interface, not verified hardware timing",
                "No vacuum, robot motion, Pi connector or vision servo",
                "Initial alignment is supplied by bench setup; no alignment policy",
                "Fixture force is ideal total contact magnitude, not a qualified hardware sensor",
                "0.25 N exploratory stop setting is not a damage threshold",
                "Travel completion is not seating success; offline tip depth is approximate",
                "Only pads collide on the tool",
                "Pad sensor uses total impulse magnitude, not calibrated normal/shear tactile channels",
                "Mass, friction and actuator dynamics uncalibrated",
                "A load hold is not verified object retention or insertion success",
            ],
        }
        (a.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        (a.output / "sensor-trace.json").write_text(json.dumps(trace, indent=2) + "\n")
        (a.output / "offline-cable-trace.json").write_text(json.dumps(offline, indent=2) + "\n")
        stage.GetRootLayer().Export(str(a.output / "bench.usda"))
        print(json.dumps(report), flush=True)
    except Exception:
        import traceback

        (a.output / "failure.txt").write_text(traceback.format_exc())
        raise
    finally:
        (a.output / "sensor-trace.json").write_text(json.dumps(trace, indent=2) + "\n")
        (a.output / "offline-cable-trace.json").write_text(json.dumps(offline, indent=2) + "\n")
        recorder.close()
        monitor.close()
        for path in a.output.iterdir():
            if path.is_file():
                os.chmod(path, 0o644)
        app.close()


if __name__ == "__main__":
    main()
