"""Loaded FR3 commissioning: presented cable, tool feedback and bounded lift.

Robot encoder kinematics and ideal pad sensors drive the skill. Cable poses and
contact identities belong only to the separate experiment audit. No Pi insertion.
"""

import argparse
import hashlib
import json
import math
import sys
from collections import deque
from dataclasses import asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--case", choices=["cable", "empty", "load_dropout"], default="cable")
    p.add_argument("--dt", type=float, choices=[0.000125, 0.0000625], default=0.000125)
    p.add_argument("--live-feed", type=Path)
    p.add_argument("--duration", type=float, default=5.0)
    p.add_argument("--joint-integral-gain", type=float, default=0.0)
    p.add_argument(
        "--zero-cell",
        action="store_true",
        help="Compose Zero board and inspect lifted end; insertion stays gated",
    )
    a = p.parse_args()
    if not 0 <= a.joint_integral_gain <= 5:
        p.error("joint-integral-gain must be in [0, 5] per second")
    a.output.mkdir(parents=True, exist_ok=False)
    sources = [
        Path(__file__),
        ROOT / "src/ffc/compact_mounted.py",
        ROOT / "src/ffc/profile_cable.py",
        ROOT / "src/ffc/pinch_skill.py",
        ROOT / "src/ffc/isaac_kinematics.py",
        ROOT / "src/ffc/joint_trajectory.py",
        ROOT / "src/ffc/macro_contract.py",
        ROOT / "src/ffc/isaac_scene.py",
        ROOT / "src/ffc/isaac_contacts.py",
        ROOT / "src/ffc/cable_spec.py",
        ROOT / "src/ffc/isaac_recording.py",
        ROOT / "config/cables/rpi-camera-standard-mini-200-rev2.json",
        ROOT / "config/fr3-flexible-lift-pose.json",
        ROOT / "config/fr3-isaac-source.json",
    ]
    if a.zero_cell:
        sources += [
            ROOT / name
            for name in [
                "src/ffc/zero_workstation.py",
                "src/ffc/zero_cameras.py",
                "src/ffc/stereo_terminal.py",
                "src/ffc/socket_reference.py",
                "config/connectors/zero-reference-contact-v1.json",
                "config/connectors/pi-socket-evidence-v1.json",
                "config/macro-mounted-camera.json",
                "src/ffc/camera_optics.py",
                "src/ffc/macro_camera.py",
            ]
        ]
    hashes = {}
    for source in sources:
        raw = source.read_bytes()
        (a.output / source.name).write_bytes(raw)
        hashes[source.name] = hashlib.sha256(raw).hexdigest()
    (a.output / "source-hashes.json").write_text(json.dumps(hashes, indent=2))
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
    from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
    from isaacsim.core.utils.types import ArticulationAction
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdLux, UsdPhysics
    from scipy.spatial.transform import Rotation

    from ffc.cable_spec import load_spec, provenance_record
    from ffc.compact_mounted import mount
    from ffc.isaac_contacts import ContactMonitor
    from ffc.isaac_kinematics import from_stage
    from ffc.isaac_recording import Recorder
    from ffc.isaac_scene import box, camera, drive, pose
    from ffc.joint_trajectory import bounded_integral
    from ffc.pinch_skill import PinchObservation, PinchSkill
    from ffc.profile_cable import create_profile_cable

    context = omni.usd.get_context()
    context.new_stage()
    stage = context.get_stage()
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/World").GetPrim())
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, "Z")
    robot = stage.DefinePrim("/World/FR3")
    robot.GetReferences().AddReference(str(ROOT / "third_party/franka_isaac/fr3v2_1/fr3v2_1.usda"))
    api = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath("/World/FR3/Geometry/base"))
    api.CreateEnabledSelfCollisionsAttr(True)
    # Millimetre motion must not stop when mass-normalized kinetic energy is low.
    api.CreateSleepThresholdAttr(0.0)
    api.CreateSolverPositionIterationCountAttr(255)
    api.CreateSolverVelocityIterationCountAttr(8)
    plan = json.loads((a.output / "fr3-flexible-lift-pose.json").read_text())
    home = np.array(plan["home_rad"])
    local = np.array(plan["tool_local_link7"])
    chain = from_stage(stage)
    tool_pose = chain.forward(home) @ local
    for i, q in enumerate(home, 1):
        j = UsdPhysics.RevoluteJoint.Get(stage, f"/World/FR3/Physics/fr3v2_1_joint{i}")
        drive(
            j,
            "angular",
            5 * (40 if i < 5 else 15),
            5 * (4 if i < 5 else 1.5),
            math.degrees(q),
            87 if i < 5 else 12,
        )
    pads, tool_report = mount(stage, ROOT, plan["link7"], local, tool_pose)
    pinch_local = local[:3, 3] + local[:3, :3] @ [0, 0.0105, 0]
    initial_link = chain.forward(home)
    pinch_origin = initial_link[:3, 3] + initial_link[:3, :3] @ pinch_local
    lift_grid = np.linspace(0, 0.01, 21)
    path, seed = [], home.copy()
    for height in lift_grid:
        seed, metric = chain.solve(
            pinch_origin + [0, 0, height], initial_link[:3, :3], seed, pinch_local, np.random.default_rng(39)
        )
        if metric["position_error_m"] > 1e-5 or metric["orientation_error_rad"] > 1e-4:
            raise RuntimeError("Lift kinematics failed")
        path.append(seed.copy())
    path = np.array(path)
    if np.max(abs(path - home)) > 0.1 or np.max(abs(np.diff(path, axis=0))) / 0.0005 * 0.005 > 0.04:
        raise RuntimeError("Reference joint step or speed bound exceeded")
    spec = load_spec(a.output / "rpi-camera-standard-mini-200-rev2.json")
    cable_paths, profile = (
        create_profile_cable(stage, spec, 0.28, mini_contacts_down=a.zero_cell)
        if a.case != "empty"
        else ([], [])
    )
    ribbon_rotation = Rotation.from_euler("z", -90, degrees=True)
    qxyzw = ribbon_rotation.as_quat()
    for body_path, section in zip(cable_paths, profile, strict=True):
        pose(
            stage.GetPrimAtPath(body_path),
            [0.488 + section["center_m"], 0.0105, 0.28],
            Gf.Quatf(float(qxyzw[3]), Gf.Vec3f(*map(float, qxyzw[:3]))),
        )
    support_lo, support_hi = np.array([0.510, -0.002, 0.16]), np.array([0.696, 0.023, 0.27985])
    box(
        stage,
        "/World/Support",
        ((support_lo + support_hi) / 2).tolist(),
        (support_hi - support_lo).tolist(),
        (0.25, 0.28, 0.3),
    )
    box(stage, "/World/Desk", (0.55, 0.02, 0.14), (0.50, 0.45, 0.04), (0.20, 0.23, 0.26))
    box(stage, "/World/Ground", (0, 0, -0.03), (3, 3, 0.02), (0.18, 0.22, 0.24), collision=False)
    zero_registration = None
    if a.zero_cell:
        from ffc.zero_workstation import add_workstation

        zero_registration = add_workstation(stage, ROOT)
        (a.output / "registration-offline.json").write_text(json.dumps(zero_registration, indent=2))
    # Per-component conservative bounds for the known fixture, using robot CAD/FK.
    bounds = []
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    obstacles = [(support_lo, support_hi)]
    if a.zero_cell:
        for prim in stage.Traverse():
            if str(prim.GetPath()).startswith(("/World/Zero2W/", "/World/ZeroFixture/", "/World/Slot/")) and (
                prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Cube)
            ):
                bound = cache.ComputeWorldBound(prim).ComputeAlignedRange()
                obstacles.append((np.array(bound.GetMin()), np.array(bound.GetMax())))
    for group in ["fixed", "lower", "upper"]:
        body = stage.GetPrimAtPath("/World/Tool/" + group)
        for prim in Usd.PrimRange(body):
            if prim.IsA(UsdGeom.Mesh):
                b = cache.ComputeRelativeBound(prim, body).ComputeAlignedRange()
                lo, hi = np.array(b.GetMin()), np.array(b.GetMax())
                corners = np.array(
                    [[x, y, z] for x in [lo[0], hi[0]] for y in [lo[1], hi[1]] for z in [lo[2], hi[2]]]
                )
                bounds.append((group, str(prim.GetPath()), corners))

    def clearance(q, jaw_q):
        t = chain.forward(q) @ local
        minimum = 100.0
        for group, _, corners in bounds:
            offset = jaw_q[0] if group == "lower" else jaw_q[1] if group == "upper" else 0
            pts = (corners + [0, 0, offset]) @ t[:3, :3].T + t[:3, 3]
            lo, hi = pts.min(axis=0), pts.max(axis=0)
            for obstacle_lo, obstacle_hi in obstacles:
                separation = np.maximum(np.maximum(obstacle_lo - hi, lo - obstacle_hi), 0)
                minimum = min(minimum, float(np.linalg.norm(separation)))
        return minimum

    preflight = min(clearance(q, [gap, -gap]) for q in path for gap in [0.0045, 0.00495, 0.005])
    if preflight <= 0:
        raise RuntimeError("Tool/support bounds overlap; review before motion")
    (a.output / "preflight.json").write_text(
        json.dumps(
            {
                "sampled_tool_support_clearance_m": preflight,
                "joint_path_rad": path.tolist(),
                "scope": "Known fixture/tool bounds, plus Zero board when enabled; not global clearance",
            },
            indent=2,
        )
    )
    UsdLux.DomeLight.Define(stage, "/World/Light").CreateIntensityAttr(900)
    camera(stage, "/World/Cameras/Overview", (1.15, -0.95, 0.85), (0.34, 0, 0.3), 38)
    world = World(physics_dt=a.dt, rendering_dt=1 / 24, stage_units_in_meters=1)
    world.get_physics_context().set_solver_type("PGS")
    arm = world.scene.add(SingleArticulation(prim_path="/World/FR3/Geometry/base", name="fr3"))
    cable_art = (
        world.scene.add(SingleArticulation(prim_path=cable_paths[0], name="ribbon")) if cable_paths else None
    )
    monitor = ContactMonitor(stage, a.dt)
    for pth in cable_paths:
        stage.GetPrimAtPath(pth).RemoveAPI(PhysxSchema.PhysxContactReportAPI)
    world.reset()
    names = arm.dof_names
    indices = [names.index(f"fr3v2_1_joint{i}") for i in range(1, 8)]
    jaw_indices = [names.index("lower_travel"), names.index("upper_travel")]
    if len(names) != 9:
        raise RuntimeError("Expected seven arm and two jaw joints")
    initial = np.zeros(9)
    initial[indices] = home
    initial[jaw_indices] = [0.0045, -0.0045]
    arm.set_joint_positions(initial)
    arm.set_joint_velocities(np.zeros(9))
    arm.apply_action(ArticulationAction(joint_positions=initial))
    # Reset can briefly solve contacts at the vendor default robot pose. Restore
    # the declared ribbon initial condition after initializing the arm, before
    # any recording/controller step. No cable pose setters run during the trial.
    if cable_art is not None:
        cable_art.set_world_pose(np.array([0.489, 0.0105, 0.28]), np.array([qxyzw[3], *qxyzw[:3]]))
        cable_art.set_joint_positions(np.zeros(cable_art.num_dof))
        cable_art.set_joint_velocities(np.zeros(cable_art.num_dof))
        cable_art.set_linear_velocity(np.zeros(3))
        cable_art.set_angular_velocity(np.zeros(3))
    cable_handles = [SingleRigidPrim(prim_path=p, name=f"offline{i}") for i, p in enumerate(cable_paths)]
    for handle in cable_handles:
        handle.initialize()
    tool_handles = {
        name: SingleRigidPrim(prim_path="/World/Tool/" + name, name="audit_" + name)
        for name in ["fixed", "lower", "upper"]
    }
    for handle in tool_handles.values():
        handle.initialize()

    def audit_bodies():
        result = {}
        for name, handle in tool_handles.items():
            pos, quat = handle.get_world_pose()
            result[name] = {"position_m": pos.tolist(), "quaternion_wxyz": quat.tolist()}
        return result

    (a.output / "initial-body-audit.json").write_text(
        json.dumps(
            {
                "tool": audit_bodies(),
                "expected_tool_matrix": tool_pose.tolist(),
                "cable": [
                    {
                        "position_m": h.get_world_pose()[0].tolist(),
                        "quaternion_wxyz": h.get_world_pose()[1].tolist(),
                    }
                    for h in cable_handles
                ],
            },
            indent=2,
        )
    )
    recorder = Recorder(
        stage,
        a.output / "lift.mp4",
        fps=24,
        scope_label="FR3 + Zero inspection commissioning / no insertion"
        if a.zero_cell
        else "FR3 loaded motion pilot / ideal tool feedback / camera review only / no insertion",
    )
    recorder.aim((0.493, 0.0105, 0.282), eye_offset=(-0.045, 0.045, 0.028))
    measured_cameras = None
    if a.zero_cell:
        from ffc.zero_cameras import ZeroCameras

        measured_cameras = ZeroCameras(stage, ROOT, a.output)
    controller = PinchSkill()
    controller.closing = 0.0045
    command = controller.command()
    window = deque(maxlen=round(0.01 / a.dt))
    control_stride = round(0.005 / a.dt)
    trace, offline, unexpected = [], [], []
    clock = {"steps": 0, "time_s": 0.0}

    def tick(dt):
        clock["steps"] += 1
        clock["time_s"] += float(dt)

    world.add_physics_callback("loaded_clock", tick)
    targets = initial.copy()
    previous_q_ref = home.copy()
    integral = np.zeros(7)
    fault_at = fault_q = None
    max_speed = max_tracking = 0.0
    min_clearance = preflight
    finished = False
    stage.GetRootLayer().Export(str(a.output / "initial-scene.usda"))
    try:
        for step in range(round(a.duration / a.dt)):
            now = step * a.dt
            if step % control_stride == 0:
                q = np.array(arm.get_joint_positions())
                dq = np.array(arm.get_joint_velocities())
                if not np.isfinite(np.r_[q, dq]).all():
                    raise RuntimeError("Nonfinite encoder feedback")
                loads = np.mean(window, axis=0) if window else np.zeros(2)
                if a.case == "load_dropout" and command.lift_m >= 0.002:
                    loads[:] = 0
                t = chain.forward(q[indices])
                center = t[:3, 3] + t[:3, :3] @ pinch_local
                obs = PinchObservation(
                    now,
                    float(q[jaw_indices[0]]),
                    float(-q[jaw_indices[1]]),
                    float(center[2] - pinch_origin[2]),
                    float(loads[0]),
                    float(loads[1]),
                )
                command = controller.update(now, obs)
                q_ref = np.array([np.interp(command.lift_m, lift_grid, path[:, j]) for j in range(7)])
                path_ref = q_ref.copy()
                if command.state == "fault":
                    if fault_at is None:
                        fault_at, fault_q = now, q[indices].copy()
                    q_ref = fault_q.copy()
                    integral[:] = 0
                else:
                    integral = bounded_integral(
                        q_ref,
                        q[indices],
                        integral,
                        chain.lower,
                        chain.upper,
                        0.003,
                        a.joint_integral_gain * 0.005,
                    )
                    q_ref = np.clip(q_ref + integral, chain.lower, chain.upper)
                targets[indices] = q_ref
                targets[jaw_indices] = [command.closing_travel_m, -command.closing_travel_m]
                velocity = np.zeros(9)
                velocity[indices] = 0 if command.state == "fault" else (q_ref - previous_q_ref) / 0.005
                previous_q_ref = q_ref.copy()
                # Robot-only gravity model: depends on its own joint configuration
                # and declared link/payload masses, never cable pose/contact truth.
                gravity = arm._articulation_view.get_generalized_gravity_forces()[0].copy()
                gravity[jaw_indices] = 0
                arm.apply_action(
                    ArticulationAction(
                        joint_positions=targets, joint_velocities=velocity, joint_efforts=gravity
                    )
                )
                max_speed = max(max_speed, float(np.max(abs(dq[indices]))))
                max_tracking = max(max_tracking, float(np.max(abs(q[indices] - q_ref))))
                if max_speed > 0.06 or max_tracking > 0.02:
                    raise RuntimeError("Arm speed/tracking experiment bound exceeded")
                trace.append(
                    {
                        "time_s": now,
                        "observation": asdict(obs),
                        "command": asdict(command),
                        "arm_q_rad": q[indices].tolist(),
                        "arm_reference_rad": q_ref.tolist(),
                        "arm_path_reference_rad": path_ref.tolist(),
                        "joint_integral_rad": integral.tolist(),
                        "gravity_feedforward_nm": gravity[indices].tolist(),
                    }
                )
                finished = command.state == "contact_hold_complete" or (
                    fault_at is not None and now - fault_at >= 0.25
                )
            monitor.begin_step(step)
            world.step(render=False, update_fabric=True)
            forces = np.zeros(2)
            for item in monitor.current:
                pair = (item["collider0"], item["collider1"])
                for idx, pad in enumerate(pads):
                    if pad in pair:
                        forces[idx] += item["sum_contact_force_magnitudes_n"]
                tool = any(p.startswith("/World/Tool/") for p in pair)
                env = any(
                    p.startswith(
                        (
                            "/World/Support",
                            "/World/Desk",
                            "/World/Cable/",
                            "/World/Zero2W/",
                            "/World/ZeroFixture/",
                            "/World/Slot/",
                        )
                    )
                    for p in pair
                )
                intended = any(p in pads for p in pair) and any(p.startswith("/World/Cable/") for p in pair)
                robot_env = any(p.startswith("/World/FR3/") for p in pair) and env
                if ((tool and env and not intended) or robot_env) and item[
                    "sum_contact_force_magnitudes_n"
                ] > 0.02:
                    unexpected.append(item)
            window.append(forces)
            if unexpected:
                raise RuntimeError("Unintended loaded-tool contact; independent experiment audit")
            if step % round(1 / (24 * a.dt)) == 0 or finished:
                recorder.write(command.state, (step + 1) * a.dt)
                min_clearance = min(min_clearance, clearance(q[indices], q[jaw_indices]))
                poses = [h.get_world_pose() for h in cable_handles]
                offline.append(
                    {
                        "time_s": (step + 1) * a.dt,
                        "positions_m": [p.tolist() for p, _ in poses],
                        "quaternions_wxyz": [r.tolist() for _, r in poses],
                        "tool_bodies": audit_bodies(),
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
                        "FR3 flexible-cable pilot / " + command.state,
                        a.output.name,
                    )
                print(
                    json.dumps(
                        {
                            "time_s": now,
                            "state": command.state,
                            "lift_mm": obs.lift_m * 1000,
                            "pad_load_n": loads.tolist(),
                        }
                    ),
                    flush=True,
                )
            if finished:
                break
        if not finished or clock["steps"] != step + 1 or abs(clock["time_s"] - (step + 1) * a.dt) > 1e-6:
            raise RuntimeError("Completion or physics clock check failed")
        inspection = None
        if measured_cameras is not None:
            before = clock["steps"]
            inspection = measured_cameras.capture(clock["time_s"])
            if clock["steps"] != before:
                raise RuntimeError("Inspection rendering advanced physics unexpectedly")
        report = {
            "case": a.case,
            "final": trace[-1],
            "clock": clock,
            "tool": tool_report,
            "cable_spec": provenance_record(spec),
            "dt_s": a.dt,
            "robot_sleep_threshold": 0.0,
            "joint_integral_gain_per_s": a.joint_integral_gain,
            "joint_integral_limit_rad": 0.003,
            "fault_stop": "Latch current measured arm angles, zero velocity and integral; hold jaw reference",
            "controller_period_s": 0.005,
            "grasp_material_coordinate_m": 0.012,
            "cable_pose_control": False,
            "grasp_attachment": False,
            "ros_connected": False,
            "camera_control": False,
            "zero_cell": a.zero_cell,
            "inspection": inspection,
            "next_stage": "Stopped: entrance pose, face polarity and alignment not qualified"
            if a.zero_cell
            else None,
            "contact_mechanics_qualified": False,
            "max_arm_speed_rad_s": max_speed,
            "max_arm_tracking_rad": max_tracking,
            "sampled_tool_support_clearance_m": min_clearance,
            "post_fault_joint_drift_rad": None
            if fault_q is None
            else float(np.max(abs(q[indices] - fault_q))),
            "unintended_contacts": unexpected,
            "scope": "FR3 presented-cable lift in Zero cell with fresh RGB inspection; no insertion"
            if a.zero_cell
            else "FR3 local loaded-motion pilot with full external tool collision; no Pi insertion",
            "gravity_model": "Robot articulation masses and own joint angles only; jaw feedforward disabled",
            "limitations": [
                "Full-cable contact/damping remains exploratory",
                "Ideal pad and encoder feedback",
                "Known presented placement, not camera-guided pickup",
                "Per-part convex hulls and filtered internal guides",
                "Inspection cameras only; not calibrated perception or tip-pose verification",
                "ROS, suction and insertion not connected",
                "Sampled tool/support bound is not global swept clearance",
            ],
        }
        (a.output / "report.json").write_text(json.dumps(report, indent=2))
        stage.GetRootLayer().Export(str(a.output / "loaded-fr3.usda"))
    except Exception:
        import traceback

        (a.output / "failure.txt").write_text(traceback.format_exc())
        raise
    finally:
        stage.GetRootLayer().Export(str(a.output / "last-scene.usda"))
        (a.output / "sensor-trace.json").write_text(json.dumps(trace, indent=2))
        (a.output / "offline-cable-trace.json").write_text(json.dumps(offline))
        (a.output / "unintended-contacts.json").write_text(json.dumps(unexpected, indent=2))
        (a.output / "contact-peaks.json").write_text(json.dumps(list(monitor.peaks.values()), indent=2))
        recorder.close()
        monitor.close()
        app.close()


if __name__ == "__main__":
    main()
