"""Record PhysX rigid contact impulses; report forces as impulse / timestep."""

from __future__ import annotations

import numpy as np
from pxr import PhysicsSchemaTools, PhysxSchema, UsdPhysics


class ContactMonitor:
    """Rigid contacts only: this interface does not validate deformable contact forces."""

    def __init__(self, stage, dt: float):
        from omni.physx import get_physx_simulation_interface

        self.dt, self.step_index = dt, -1
        self.mass_unit_kg = UsdPhysics.GetStageKilogramsPerUnit(stage)
        self.current, self.records = [], []
        self.peaks = {}
        self.record_stride = max(1, round(0.01 / dt))
        self.joint_events = []
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        self.subscription = get_physx_simulation_interface().subscribe_contact_report_events(self.callback)
        from omni.physx import get_physx_interface
        from omni.physx.bindings._physx import SimulationEvent

        def joint_event(event):
            if event.type == int(SimulationEvent.JOINT_BREAK):
                path = PhysicsSchemaTools.decodeSdfPath(*event.payload["jointPath"])
                self.joint_events.append({"step": self.step_index, "event": "joint_break", "path": str(path)})

        self.joint_subscription = (
            get_physx_interface().get_simulation_event_stream_v2().create_subscription_to_pop(joint_event)
        )

    def begin_step(self, index: int):
        self.step_index = index
        self.current = []
        if index == 0:
            self.peaks.clear()

    def callback(self, headers, data):
        for header in headers:
            if header.num_contact_data == 0:
                continue
            a = str(PhysicsSchemaTools.intToSdfPath(header.collider0))
            b = str(PhysicsSchemaTools.intToSdfPath(header.collider1))
            force = 0.0
            separation = float("inf")
            for i in range(header.contact_data_offset, header.contact_data_offset + header.num_contact_data):
                force += float(np.linalg.norm(data[i].impulse)) / self.dt * self.mass_unit_kg
                separation = min(separation, float(data[i].separation))
            item = {
                "step": self.step_index,
                "collider0": a,
                "collider1": b,
                "sum_contact_force_magnitudes_n": force,
                "minimum_separation_m": separation,
            }
            self.current.append(item)
            key = tuple(sorted((a, b)))
            if key not in self.peaks or force > self.peaks[key]["sum_contact_force_magnitudes_n"]:
                self.peaks[key] = item
            if force > 0.0001 and self.step_index % self.record_stride == 0:
                self.records.append(item)

    def gate(self):
        """Reject robot/environment collisions and excessive cable/connector load."""
        if self.joint_events:
            raise RuntimeError(f"Breakable grasp attachment failed: {self.joint_events[-1]}")
        for c in self.current:
            a, b = c["collider0"], c["collider1"]
            force = c["sum_contact_force_magnitudes_n"]
            cable_connector = ("/Cable/" in a and "/Connector/" in b) or (
                "/Cable/" in b and "/Connector/" in a
            )
            if cable_connector and force > 0.5:
                raise RuntimeError(f"Insertion contact force gate exceeded 0.5 N: {c}")
            if a.startswith("/World/FR3/") and b.startswith("/World/FR3/") and force > 1.0:
                raise RuntimeError(f"Robot self-collision: {c}")
            if a.startswith("/World/Tool/") and b.startswith("/World/Tool/") and force > 0.1:
                raise RuntimeError(f"Unintended tool self-contact: {c}")
            for robot, tool in ((a, b), (b, a)):
                if (
                    robot.startswith("/World/FR3/")
                    and tool.startswith("/World/Tool/")
                    and "fr3v2_1_link7" not in robot
                    and force > 0.1
                ):
                    raise RuntimeError(f"Unintended arm/tool contact: {c}")
            for robot, environment in ((a, b), (b, a)):
                is_robot = robot.startswith("/World/Tool/") or robot.startswith("/World/FR3/")
                is_env = any(
                    environment.startswith("/World/" + name)
                    for name in ("Desk", "Regrasp", "PCB", "Connector")
                )
                # Only the fixed base mounting interface is an intended robot/desk contact.
                is_base = "fr3v2_1_link0" in robot and "fr3v2_1_link1" not in robot
                intended_mount = is_base and environment.startswith("/World/Desk")
                if is_robot and is_env and not intended_mount and force > 0.1:
                    raise RuntimeError(f"Unintended robot/environment contact: {c}")

    def close(self):
        self.subscription = None
        self.joint_subscription = None
