"""Gated, scripted handling experiment. Grasp acquisition is explicitly idealized.

The pinch/regrasp phase uses contact physics, not a welded cable. Any failed
gate aborts the episode and is retained in the event log.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from isaacsim.core.utils.types import ArticulationAction
from pxr import Gf, UsdGeom

from ffc.control_reference import retain_unchanged_targets
from ffc.isaac_kinematics import from_stage
from ffc.isaac_suction import attach, attachment_geometry, check_seal, release
from ffc.seating import seating_metrics


class Episode:
    """Finite state experiment with smooth joint commands and measured phase gates."""

    def __init__(self, stage, arm, cfg: dict, output: Path):
        self.stage, self.arm, self.cfg, self.output = stage, arm, cfg, output
        self.chain = from_stage(stage)
        self.names = arm.dof_names
        self.indices = [self.names.index(f"fr3v2_1_joint{i}") for i in range(1, 8)]
        self.rng = np.random.default_rng(cfg["seed"])
        self.events = []
        self.phase_index, self.phase_time = 0, 0.0
        self.q_start = arm.get_joint_positions().copy()
        self.q_goal = self.q_start.copy()
        self.integral = np.zeros(len(self.q_start))
        self.patch_link7 = np.array([0.0, 0.120, 0.218])
        self.rotation = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
        self.pick_index = cfg["cable"]["segments"] - 3
        self.pick_path = f"/World/Cable/segment_{self.pick_index:03d}"
        self.segment_length = cfg["cable"]["length_m"] / cfg["cable"]["segments"]
        self.free_length = 2.5 * self.segment_length
        sx, sy, _ = cfg["cable"]["start_xyz_m"]
        self.pick = np.array(
            [sx + (self.pick_index + 0.5) * self.segment_length, sy, cfg["cable"]["thickness_m"] + 0.0002]
        )
        fx, fy, fz = cfg["fixture"]["center_xyz_m"]
        self.place = np.array(
            [fx + cfg["fixture"]["shelf_length_m"] / 2 + 0.01, fy, fz + cfg["cable"]["thickness_m"] + 0.0002]
        )
        self.insert = np.asarray(cfg["connector"]["mouth_xyz_m"]) + [
            -self.free_length,
            0,
            cfg["cable"]["thickness_m"] / 2,
        ]
        self.phases = [
            ("settle", 1.0, None, 0.0, -0.018),
            ("approach_pick", 3.0, self.pick + [0, 0, 0.04], 0.0, -0.018),
            ("hover_pick", 3.0, self.pick + [0, 0, 0.004], 0.0, -0.018),
            ("descend_pick", 3.0, self.pick, 0.0, -0.018),
            ("acquire_suction", 0.5, None, 0.0, -0.018),
            ("lift", 3.0, self.pick + [0, 0, 0.12], 0.0, -0.018),
            ("raise_for_fixture", 3.0, np.array([self.pick[0], self.pick[1], 0.30]), 0.0, -0.018),
            ("transport_to_fixture", 3.0, np.array([self.place[0], self.place[1], 0.30]), 0.0, -0.018),
            ("place_on_fixture", 3.0, self.place, 0.0, -0.018),
            ("release_suction", 1.0, None, 0.0, -0.018),
            ("clear_fixture", 2.0, self.place + [0, 0, 0.04], 0.0, -0.018),
            ("open_lower_jaw", 1.0, None, 0.0, 0.0),
            ("regrasp_approach", 2.0, self.place, 0.0, 0.0),
            ("deploy_lower_jaw", 1.0, None, -0.018, 0.0),
            ("pinch", 1.0, None, -0.018, -0.00635),
            ("lift_after_regrasp", 3.0, self.place + [0, 0, 0.12], -0.018, -0.00635),
            ("raise_after_fixture", 3.0, np.array([self.place[0], self.place[1], 0.30]), -0.018, -0.00635),
            (
                "transport_to_pcb",
                3.0,
                np.array([self.insert[0] - 0.015, self.insert[1], 0.30]),
                -0.018,
                -0.00635,
            ),
            ("align_connector", 3.0, self.insert + [-0.015, 0, 0], -0.018, -0.00635),
            ("approach_slot", 3.0, self.insert + [-0.001, 0, 0], -0.018, -0.00635),
            (
                "insert",
                3.0,
                self.insert + [cfg["connector"]["insertion_depth_m"] - 0.0002, 0, 0],
                -0.018,
                -0.00635,
            ),
            ("verify_seating", 1.0, None, -0.018, -0.00635),
        ]
        if cfg.get("handling_strategy") == "direct":
            start = next(i for i, phase in enumerate(self.phases) if phase[0] == "raise_for_fixture")
            end = next(i for i, phase in enumerate(self.phases) if phase[0] == "transport_to_pcb")
            self.phases[start:end] = [
                ("open_lower_jaw", 1.0, None, 0.0, 0.0),
                ("deploy_lower_jaw", 1.0, None, -0.018, 0.0),
                ("pinch", 1.0, None, -0.018, -0.00635),
                ("release_after_pinch", 1.0, None, -0.018, -0.00635),
                ("raise_after_pinch", 3.0, np.array([self.pick[0], self.pick[1], 0.30]), -0.018, -0.00635),
            ]
        if cfg.get("fixture_laydown", False) and cfg.get("handling_strategy") != "direct":
            place_index = next(i for i, phase in enumerate(self.phases) if phase[0] == "place_on_fixture")
            self.phases.insert(
                place_index, ("land_tail_on_fixture", 3.0, self.place + [-0.06, 0, 0.09], 0.0, -0.018)
            )
        if cfg.get("grasp_benchmark"):
            self.phases = [
                phase for phase in self.phases if phase[0] in ("settle", "acquire_suction", "lift")
            ]
        if cfg.get("transfer_benchmark"):
            allowed = {
                "settle",
                "acquire_suction",
                "lift",
                "open_lower_jaw",
                "deploy_lower_jaw",
                "pinch",
                "release_after_pinch",
            }
            if cfg.get("transfer_benchmark_transport", False):
                allowed.add("raise_after_pinch")
            self.phases = [phase for phase in self.phases if phase[0] in allowed]
        closed = cfg.get("tool", {}).get("closed_clamp_target_m", -0.0068)
        self.phases = [
            (name, duration, xyz, deploy, closed if clamp == -0.00635 else clamp)
            for name, duration, xyz, deploy, clamp in self.phases
        ]
        self.done = False
        self.begin_phase()

    def record(self, event: str, **data):
        self.events.append({"event": event, "phase": self.phases[self.phase_index][0], **data})
        (self.output / "episode-events.json").write_text(json.dumps(self.events, indent=2))

    def body_matrix(self, path):
        return UsdGeom.XformCache().GetLocalToWorldTransform(self.stage.GetPrimAtPath(path))

    def patch(self):
        return np.array(self.body_matrix("/World/Tool/Body").Transform(Gf.Vec3d(0, 0.120, 0.086)))

    def cable_tip(self):
        p = f"/World/Cable/segment_{self.cfg['cable']['segments'] - 1:03d}"
        return np.array(self.body_matrix(p).Transform(Gf.Vec3d(self.segment_length / 2, 0, 0)))

    def begin_phase(self):
        previous_goal = self.q_goal.copy()
        name, duration, xyz, deployment, clamp = self.phases[self.phase_index]
        target_rotation = self.rotation
        if name == "land_tail_on_fixture":
            tail = np.array(
                self.body_matrix("/World/Cable/segment_000").Transform(
                    Gf.Vec3d(-self.segment_length / 2, 0, 0)
                )
            )
            fixture = self.cfg["fixture"]
            desired_tail = np.asarray(fixture["center_xyz_m"]) + [
                -fixture["shelf_length_m"] / 2 + 0.01,
                0,
                self.cfg["cable"]["thickness_m"] / 2,
            ]
            xyz = desired_tail - (tail - self.patch())
            self.phases[self.phase_index] = (name, duration, xyz, deployment, clamp)
            self.record(
                "privileged_tail_landing_target",
                measured_tail_xyz_m=tail.tolist(),
                desired_tail_xyz_m=desired_tail.tolist(),
                patch_target_xyz_m=xyz.tolist(),
            )
        if self.cfg.get("tip_pose_alignment", False) and name in (
            "align_connector",
            "approach_slot",
            "insert",
        ):
            mouth = np.asarray(self.cfg["connector"]["mouth_xyz_m"])
            depth = {
                "align_connector": -0.015,
                "approach_slot": -0.001,
                "insert": self.cfg["connector"]["insertion_depth_m"] - 0.0002,
            }[name]
            desired_tip = mouth + [depth, 0, 0]
            last_path = f"/World/Cable/segment_{self.cfg['cable']['segments'] - 1:03d}"
            tip_rotation = np.asarray(self.body_matrix(last_path).ExtractRotationMatrix()).T
            body_rotation = np.asarray(self.body_matrix("/World/Tool/Body").ExtractRotationMatrix()).T
            correction = tip_rotation.T
            target_rotation = correction @ body_rotation
            relative_tip = correction @ (self.cable_tip() - self.patch())
            xyz = desired_tip - relative_tip
            self.phases[self.phase_index] = (name, duration, xyz, deployment, clamp)
            self.record(
                "privileged_tip_pose_alignment",
                measured_tip_xyz_m=self.cable_tip().tolist(),
                desired_tip_xyz_m=desired_tip.tolist(),
                corrected_patch_target_xyz_m=xyz.tolist(),
            )
        self.q_start = self.arm.get_joint_positions().copy()
        self.q_goal = self.q_start.copy()
        self.arm_waypoints = [self.q_start[self.indices].copy()]
        if xyz is not None:
            start_xyz = self.patch()
            count = (
                1 if name == "approach_pick" else max(1, int(np.ceil(np.linalg.norm(xyz - start_xyz) / 0.01)))
            )
            solution = self.q_start[self.indices]
            for fraction in np.linspace(0, 1, count + 1)[1:]:
                point = start_xyz + fraction * (xyz - start_xyz)
                solution, metrics = self.chain.solve(
                    point, target_rotation, solution, self.patch_link7, self.rng
                )
                self.arm_waypoints.append(solution.copy())
            self.q_goal[self.indices] = solution
            self.record("ik_solution", **metrics, cartesian_waypoints=count)
        else:
            self.arm_waypoints.append(self.q_goal[self.indices].copy())
        self.arm_waypoints = np.asarray(self.arm_waypoints)
        self.q_goal[self.names.index("deploy")] = deployment
        self.q_goal[self.names.index("clamp")] = clamp
        self.q_start = retain_unchanged_targets(
            self.q_start,
            previous_goal,
            self.q_goal,
            [self.names.index("deploy"), self.names.index("clamp")],
        )
        # At most 0.5 rad/s through a smoothstep interpolation (peak factor 1.5).
        self.duration = max(
            duration,
            float(np.max(np.abs(np.diff(self.arm_waypoints, axis=0)))) * (len(self.arm_waypoints) - 1) * 3,
        )
        if name == "acquire_suction":
            attach(self.stage, self.pick_path)
            self.record(
                "idealized_suction_attachment",
                patches=2,
                break_force_per_patch_n=30000 * np.pi * 0.0015**2,
                patch_spacing_m=0.005,
                break_torque_per_patch_nm=0.0001,
            )
        if name in ("release_suction", "release_after_pinch"):
            release_mode = self.cfg.get("suction_release_mode", "zero_drives")
            self.record("vacuum_pre_vent_geometry", patches=attachment_geometry(self.stage))
            release(self.stage, release_mode)
            self.record("vacuum_released", release_mode=release_mode)
        self.record("phase_started", duration_s=self.duration, joint_target=self.q_goal.tolist())

    def gate(self):
        name, _, target, _, _ = self.phases[self.phase_index]
        if target is not None:
            error = float(np.linalg.norm(self.patch() - target))
            self.record(
                "tracking",
                patch_error_m=error,
                cable_tip_xyz_m=self.cable_tip().tolist(),
                joint_actual=self.arm.get_joint_positions().tolist(),
                integral_correction_rad=self.integral[self.indices].tolist(),
            )
            if error > 0.0015:
                raise RuntimeError(f"{name}: tool tracking gate failed, error={error:.6f} m")
        tip = self.cable_tip()
        if name == "pinch":
            self.gripped_tip_local = np.asarray(
                self.body_matrix("/World/Tool/Body").GetInverse().Transform(Gf.Vec3d(*tip))
            )
            self.record("grip_reference", tip_in_tool_frame_m=self.gripped_tip_local.tolist())
        if name in ("lift", "lift_after_regrasp"):
            minimum = 0.07 if name == "lift" else self.place[2] + 0.07
            if tip[2] < minimum:
                raise RuntimeError(f"{name}: cable was not lifted; tip z={tip[2]:.6f} m")
        if name == "release_suction":
            if abs(tip[1] - self.place[1]) > 0.012 or tip[2] < self.place[2] - 0.012:
                raise RuntimeError(f"Fixture retention gate failed: tip={tip.tolist()}")
        if name == "release_after_pinch" and tip[2] < self.pick[2] + 0.09:
            raise RuntimeError(f"Pinch retention failed after venting vacuum: tip={tip.tolist()}")
        if hasattr(self, "gripped_tip_local") and name in (
            "release_after_pinch",
            "raise_after_pinch",
            "lift_after_regrasp",
            "raise_after_fixture",
            "transport_to_pcb",
        ):
            current_tip_local = np.asarray(
                self.body_matrix("/World/Tool/Body").GetInverse().Transform(Gf.Vec3d(*tip))
            )
            drift = float(np.linalg.norm(current_tip_local - self.gripped_tip_local))
            self.record("grip_stability", tip_drift_in_tool_frame_m=drift)
            if drift > self.cfg.get("maximum_free_tip_grip_drift_m", 0.001):
                raise RuntimeError(f"Grip tip drift exceeded 1 mm before insertion: {drift:.6f} m")
        if name == "verify_seating":
            last_path = f"/World/Cable/segment_{self.cfg['cable']['segments'] - 1:03d}"
            metrics = seating_metrics(
                tip,
                np.asarray(self.body_matrix(last_path).ExtractRotationMatrix()).T,
                self.cfg["cable"],
                self.cfg["connector"],
            )
            self.record("geometric_seating", **metrics)
            if not metrics["success"]:
                raise RuntimeError("Geometric seating gate failed")

    def step(self, dt: float):
        """Advance commands only; the simulator advances all bodies through physics."""
        if self.done:
            return
        check_seal(self.stage)
        self.phase_time += dt
        fraction = min(1.0, self.phase_time / self.duration)
        smooth = fraction * fraction * (3 - 2 * fraction)
        desired = self.q_start + smooth * (self.q_goal - self.q_start)
        path_position = smooth * (len(self.arm_waypoints) - 1)
        segment = min(int(path_position), len(self.arm_waypoints) - 2)
        weight = path_position - segment
        desired[self.indices] = (1 - weight) * self.arm_waypoints[segment] + weight * self.arm_waypoints[
            segment + 1
        ]
        error = desired - self.arm.get_joint_positions()
        control = self.cfg.get("robot_controller", {})
        integral_gain = control.get("integral_gain_per_s", 5.0)
        limit = control.get("integral_limit_rad", 0.03)
        self.integral[self.indices] = np.clip(
            self.integral[self.indices] + integral_gain * dt * error[self.indices], -limit, limit
        )
        commanded = desired + self.integral
        commanded[self.indices] = np.clip(commanded[self.indices], self.chain.lower, self.chain.upper)
        desired_velocity = 6 * fraction * (1 - fraction) / self.duration * (self.q_goal - self.q_start)
        desired_velocity[self.indices] = (
            (self.arm_waypoints[segment + 1] - self.arm_waypoints[segment])
            * (len(self.arm_waypoints) - 1)
            * 6
            * fraction
            * (1 - fraction)
            / self.duration
        )
        self.arm.apply_action(
            ArticulationAction(joint_positions=commanded, joint_velocities=desired_velocity)
        )
        target_xyz = self.phases[self.phase_index][2]
        settled = target_xyz is None or np.linalg.norm(self.patch() - target_xyz) < 0.00005
        if self.phase_time >= self.duration + 0.4 and (settled or self.phase_time >= self.duration + 4.0):
            self.gate()
            self.record("phase_completed")
            self.phase_index += 1
            self.phase_time = 0.0
            if self.phase_index == len(self.phases):
                self.done = True
                return
            self.begin_phase()
