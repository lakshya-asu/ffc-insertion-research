"""Physical pinch commissioning on a supported rigid terminal coupon.

This is a prerequisite experiment, not full desk pickup or Pi insertion. The
controller receives only encoder equivalents and pad load readings. Coupon pose
is recorded separately for offline scoring and never passed to the controller.
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
    p.add_argument("--case", choices=["coupon", "empty", "load_dropout"], default="coupon")
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    (a.output / "source.py").write_bytes(Path(__file__).read_bytes())
    (a.output / "pinch_skill.py").write_bytes((ROOT / "src/ffc/pinch_skill.py").read_bytes())
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
    dt, z0 = 0.0005, 0.055
    groups = {"fixed": 0.12, "lower": 0.02, "upper": 0.02}
    for name, mass in groups.items():
        prim = UsdGeom.Xform.Define(stage, "/World/Tool/" + name).GetPrim()
        pose(prim, (0, 0, z0))
        UsdPhysics.RigidBodyAPI.Apply(prim)
        api = UsdPhysics.MassAPI.Apply(prim)
        api.CreateMassAttr(mass)
        api.CreateCenterOfMassAttr(Gf.Vec3f(0, -0.03 if name == "fixed" else -0.004, 0))
        api.CreateDiagonalInertiaAttr(Gf.Vec3f(0.00004 if name == "fixed" else 0.000002))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateSolverPositionIterationCountAttr(255)
        rb.CreateSolverVelocityIterationCountAttr(8)
        rb.CreateEnableCCDAttr(True)
        visual = UsdGeom.Xform.Define(stage, str(prim.GetPath()) + "/Visual")
        visual.AddTranslateOp().Set(Gf.Vec3d(-0.0102985785382651, 0.2482103195204206, -0.0143601205939444))
        child = stage.DefinePrim(str(visual.GetPath()) + "/SupplierAndFinger")
        child.GetReferences().AddReference(
            str(ROOT / "outputs/compact-fingers-002/compact-fingers.usda"), "/CompactTool/" + name
        )
    lift = joint(
        stage, "/World/Joints/lift", UsdPhysics.PrismaticJoint, None, "/World/Tool/fixed", (0, 0, z0)
    )
    lift.CreateAxisAttr("Z")
    lift.CreateLowerLimitAttr(0)
    lift.CreateUpperLimitAttr(0.012)
    lift_drive = drive(lift, "linear", 200000, 100, 0, 30)
    jaw_drives = {}
    pad_paths = []
    for name, sign in [("lower", 1), ("upper", -1)]:
        path = "/World/Tool/" + name
        j = joint(stage, "/World/Joints/" + name, UsdPhysics.PrismaticJoint, "/World/Tool/fixed", path)
        j.CreateAxisAttr("Z")
        j.CreateLowerLimitAttr(0 if sign == 1 else -0.005)
        j.CreateUpperLimitAttr(0.005 if sign == 1 else 0)
        jaw_drives[name] = drive(j, "linear", 10000, 2, 0, 0.8)
        # The visible CAD remains intact. Only the contact pads are colliders in
        # this isolated contact bench; whole-tool collision qualification is separate.
        pad = box(
            stage,
            path + "/PadCollision",
            (0, 0.0105, -sign * 0.00515),
            (0.006, 0.003, 0.0003),
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
        (0, 0.020, (z0 - 0.00015) / 2),
        (0.025, 0.007, z0 - 0.00015),
        (0.25, 0.28, 0.3),
    )
    box(stage, "/World/Ground", (0, 0, -0.006), (0.4, 0.4, 0.012), (0.2, 0.23, 0.26))
    if a.case != "empty":
        box(stage, "/World/Coupon", (0, 0.021, z0), (0.0115, 0.03, 0.0003), (0.08, 0.25, 0.8), mass=0.0002)
    UsdLux.DomeLight.Define(stage, "/World/Light").CreateIntensityAttr(900)
    camera(stage, "/World/Cameras/Overview", (0.14, 0.14, 0.16), (0, -0.015, 0.052), 48)
    world = World(physics_dt=dt, rendering_dt=1 / 24, stage_units_in_meters=1)
    world.get_physics_context().set_solver_type("PGS")
    handles = {
        name: world.scene.add(SingleRigidPrim(prim_path="/World/Tool/" + name, name=name)) for name in groups
    }
    coupon = (
        world.scene.add(SingleRigidPrim(prim_path="/World/Coupon", name="offline_coupon"))
        if a.case != "empty"
        else None
    )
    monitor = ContactMonitor(stage, dt)
    world.reset()
    recorder = Recorder(
        stage,
        a.output / "pinch.mp4",
        fps=24,
        scope_label="Physical pinch bench / ideal pad loads / rigid coupon / NOT desk pickup or insertion",
    )
    recorder.aim((0, 0.010, z0), eye_offset=(0.035, 0.055, 0.025))
    controller = PinchSkill()
    history = deque(maxlen=20)
    trace, offline = [], []
    command = controller.command()
    sensor_sum = np.zeros(2)
    initial_time = world.current_time
    final = {}
    fault_time = None
    fault_lift = None
    dropout_injected = False
    try:
        for i in range(round(10 / dt)):
            jaw_drives["lower"].GetTargetPositionAttr().Set(command.closing_travel_m)
            jaw_drives["upper"].GetTargetPositionAttr().Set(-command.closing_travel_m)
            lift_drive.GetTargetPositionAttr().Set(command.lift_m)
            monitor.begin_step(i)
            world.step(render=False, update_fabric=True)
            # Sensor adapter sees only impulses applied at its known pad surface.
            # Counterpart identity is discarded and never enters the skill.
            forces = np.zeros(2)
            for contact in monitor.current:
                for index, path in enumerate(pad_paths):
                    if path in (contact["collider0"], contact["collider1"]):
                        forces[index] += contact["sum_contact_force_magnitudes_n"]
            history.append(forces)
            sensor_sum = np.mean(history, axis=0)
            base_z = float(handles["fixed"].get_world_pose()[0][2])
            lower_z = float(handles["lower"].get_world_pose()[0][2])
            upper_z = float(handles["upper"].get_world_pose()[0][2])
            now = (i + 1) * dt
            if a.case == "load_dropout" and command.lift_m >= 0.002:
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
            command = controller.update(now, observation)
            if i % 20 == 0:
                trace.append({"time_s": now, "observation": asdict(observation), "command": asdict(command)})
                if coupon is not None:
                    offline.append({"time_s": now, "position_m": coupon.get_world_pose()[0].tolist()})
            if i % round(1 / (24 * dt)) == 0:
                recorder.write(command.state, now)
            if command.state == "fault" and fault_time is None:
                fault_time, fault_lift = now, observation.lift_m
            finished = command.state == "contact_hold_complete" or (
                fault_time is not None and now - fault_time >= 0.25
            )
            if finished:
                final = {"command": asdict(command), "observation": asdict(observation), "time_s": now}
                recorder.write(command.state + " / " + command.reason, now)
                break
        recorder.close()
        report = {
            "case": a.case,
            "scope": "Presented rigid coupon contact bench; not full FFC or desk pickup",
            "final": final,
            "physics_steps": i + 1,
            "physics_dt_s": dt,
            "physics_elapsed_s": world.current_time - initial_time,
            "source_of_control": (
                "Measured tool body displacement as encoder equivalent; "
                "10 ms mean pad contact impulse magnitude / timestep"
            ),
            "camera_control": False,
            "ros_connected": False,
            "coupon_pose_control": False,
            "grasp_attachment": False,
            "dropout_injected": dropout_injected,
            "fault_time_s": fault_time,
            "post_fault_lift_drift_m": None if fault_lift is None else observation.lift_m - fault_lift,
            "assumptions": {
                "coupon_mass_kg": 0.0002,
                "coupon_dimensions_m": [0.0115, 0.03, 0.0003],
                "pad_friction_static_dynamic": [0.5, 0.4],
                "jaw_drive_force_cap_N": 0.8,
                "jaw_stiffness_N_m": 10000,
                "jaw_damping_N_s_m": 2,
            },
            "limitations": [
                "Rigid coupon on a support; not the 200 mm compliant cable",
                "No vacuum, robot motion, Pi connector or vision servo",
                "Only pads collide on the tool",
                "Pad sensor uses total impulse magnitude, not calibrated normal/shear tactile channels",
                "Mass, friction and actuator dynamics uncalibrated",
                "A load hold is not verified object retention or insertion success",
            ],
        }
        (a.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        (a.output / "sensor-trace.json").write_text(json.dumps(trace, indent=2) + "\n")
        (a.output / "offline-coupon-trace.json").write_text(json.dumps(offline, indent=2) + "\n")
        stage.GetRootLayer().Export(str(a.output / "bench.usda"))
        print(json.dumps(report), flush=True)
    finally:
        recorder.close()
        monitor.close()
        for path in a.output.iterdir():
            if path.is_file():
                os.chmod(path, 0o644)
        app.close()


if __name__ == "__main__":
    main()
