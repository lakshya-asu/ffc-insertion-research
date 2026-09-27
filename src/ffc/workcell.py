"""Build and check a geometric FR3 workcell; no insertion physics is modeled."""

from __future__ import annotations

import json
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
from numpy.typing import NDArray

ROOT = Path(__file__).resolve().parents[2]
HORIZONTAL_ROTATION = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


def text_vector(values: list[float] | NDArray[np.float64]) -> str:
    """Serialize a numeric vector for MJCF."""
    return " ".join(f"{value:.10g}" for value in values)


def load_config(path: Path | None = None) -> dict:
    """Read the explicit exploratory geometry and evaluation configuration."""
    return json.loads((path or ROOT / "config/workcell.json").read_text())


def build_model(config: dict) -> tuple[mujoco.MjModel, str]:
    """Load the pinned FR3 and attach proxies for workspace-clearance checks."""
    source = ROOT / "third_party/franka_fr3"
    root = ET.parse(source / "fr3.xml").getroot()
    root.set("model", "ffc_workcell_geometry_proxy")
    root.find("compiler").set("meshdir", str(source / "assets"))
    ET.SubElement(root, "visual")
    ET.SubElement(root.find("visual"), "global", offwidth="1600", offheight="1000")
    ET.SubElement(root.find("visual"), "headlight", diffuse="0.7 0.7 0.7", ambient="0.3 0.3 0.3")
    world = root.find("worldbody")
    ET.SubElement(world, "light", pos="0.1 -0.6 1.8", dir="0.2 0.3 -1", directional="true")
    ET.SubElement(world, "light", pos="0.7 0.7 1.3", dir="-0.2 -0.3 -1", directional="true")

    def box(parent: ET.Element, name: str, pos: list[float], size: list[float], rgba: str) -> None:
        ET.SubElement(
            parent, "geom", name=name, type="box", pos=text_vector(pos), size=text_vector(size), rgba=rgba
        )

    box(world, "desk", [0.3, 0, -0.04], [0.65, 0.48, 0.04], "0.23 0.27 0.30 1")
    box(world, "floor", [0, 0, -0.84], [2, 2, 0.04], "0.10 0.12 0.14 1")
    for x in (-0.27, 0.86):
        for y in (-0.40, 0.40):
            box(world, f"desk_leg_{x}_{y}", [x, y, -0.44], [0.025, 0.025, 0.36], "0.12 0.14 0.16 1")
    x, y = config["connector_mouth_xy_m"]
    height = config["fixture_height_m"]
    pcb_t = config["pcb_thickness_m"]
    box(world, "fixture", [x + 0.045, y, height / 2], [0.025, 0.022, height / 2], "0.45 0.49 0.52 1")
    box(world, "pcb", [x + 0.04, y, height + pcb_t / 2], [0.04, 0.03, pcb_t / 2], "0.07 0.40 0.27 1")
    box(
        world,
        "connector_envelope",
        [x + config["connector_depth_m"] / 2, y, height + pcb_t + config["connector_height_m"] / 2],
        [config["connector_depth_m"] / 2, config["connector_width_m"] / 2, config["connector_height_m"] / 2],
        "0.83 0.81 0.73 1",
    )
    ET.SubElement(
        world,
        "site",
        name="connector_reference",
        pos=text_vector([x, y, height + pcb_t + config["assumed_slot_center_above_pcb_m"]]),
        type="sphere",
        size="0.0012",
        rgba="1 0.5 0.05 1",
    )
    # Desk cable is visual only. It never attaches to the tool during this experiment.
    pickup_x, pickup_y = config["pickup_xy_m"][0]
    cable = ET.SubElement(world, "body", name="static_cable_proxy")
    box(
        cable,
        "cable_visual",
        [pickup_x - 0.055, pickup_y, config["cable_thickness_m"] / 2],
        [config["cable_length_m"] / 2, config["cable_width_m"] / 2, config["cable_thickness_m"] / 2],
        "0.88 0.88 0.78 1",
    )
    box(
        cable,
        "stiffener_visual",
        [pickup_x + 0.015, pickup_y, 0.00032],
        [0.005, config["cable_width_m"] / 2, 0.00010],
        "0.12 0.39 0.75 1",
    )
    for geom in cable.findall("geom"):
        geom.set("contype", "0")
        geom.set("conaffinity", "0")

    link = root.find(".//body[@name='fr3_link7']")
    tool = ET.SubElement(link, "body", name="tool_envelope", pos="0 0 0.107")
    box(tool, "tool_body", [0, 0, 0.045], [0.024, 0.024, 0.04], "0.11 0.14 0.17 1")
    for side in (-1, 1):
        box(tool, f"jaw_{side}", [0, side * 0.006, 0.112], [0.005, 0.003, 0.018], "0.56 0.64 0.68 1")
    ET.SubElement(
        tool,
        "site",
        name="tip_reference",
        pos=f"0 0 {config['tool_tip_from_attachment_m']}",
        size="0.002",
        rgba="1 0.5 0.05 1",
    )
    carriage = ET.SubElement(tool, "body", name="suction_carriage", pos="0 -0.03 0.13")
    ET.SubElement(
        carriage,
        "joint",
        name="suction_extension",
        type="slide",
        axis="0 -1 0",
        range="0 0.06",
        damping="1",
        armature="0.001",
    )
    ET.SubElement(
        carriage,
        "geom",
        name="suction_nozzle",
        type="capsule",
        fromto="0 0.015 0 0 -0.01 0",
        size="0.003",
        rgba="0.25 0.65 0.80 1",
        mass="0.02",
    )
    ET.SubElement(
        carriage, "site", name="suction_contact", pos="0 -0.013 0", size="0.0015", rgba="0.1 0.8 0.9 1"
    )
    key = root.find("keyframe/key")
    key.set("qpos", key.get("qpos") + " 0")
    xml = ET.tostring(root, encoding="unicode")
    return mujoco.MjModel.from_xml_string(xml), xml


@dataclass(frozen=True)
class Target:
    """World-frame target for a named tool site, expressed in meters."""

    name: str
    site: str
    position_m: NDArray[np.float64]
    extension_m: float


def targets(config: dict) -> list[Target]:
    """Generate fixed pose checks, not manipulation rollouts."""
    result = []
    for index, (x, y) in enumerate(config["pickup_xy_m"]):
        for phase, dz in (("contact", 0), ("lift", config["lift_height_m"])):
            result.append(
                Target(
                    f"pickup_{index}_{phase}",
                    "suction_contact",
                    np.array([x, y, config["cable_thickness_m"] + dz]),
                    config["suction_extension_m"],
                )
            )
    x, y = config["connector_mouth_xy_m"]
    z = config["fixture_height_m"] + config["pcb_thickness_m"] + config["assumed_slot_center_above_pcb_m"]
    for distance in config["approach_distances_m"]:
        result.append(
            Target(f"approach_{distance:.3f}m", "tip_reference", np.array([x - distance, y, z]), 0.0)
        )
    return result


def rotation_error(target: NDArray[np.float64], current: NDArray[np.float64]) -> NDArray[np.float64]:
    """Return the world-frame SO(3) logarithmic error, in radians."""
    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(quaternion, (target @ current.T).ravel())
    if quaternion[0] < 0:
        quaternion *= -1
    error = np.empty(3)
    mujoco.mju_quat2Vel(error, quaternion, 1.0)
    return error


def contact_diagnostics(model: mujoco.MjModel, data: mujoco.MjData, tolerance_m: float) -> list[dict]:
    """List contacts, excluding only the intentional base mounting interface."""
    result = []
    for contact in data.contact:
        names = [model.geom(int(index)).name or f"geom_{index}" for index in contact.geom]
        if set(names) == {"desk", "fr3_link0_collision"}:
            continue
        result.append(
            {"geoms": names, "distance_m": float(contact.dist), "reject": bool(contact.dist < -tolerance_m)}
        )
    return result


def solve_target(model: mujoco.MjModel, target: Target, config: dict, seed: int) -> dict:
    """Search joint-limited IK starts and prefer converged, nonpenetrating endpoints."""
    rng = np.random.default_rng(seed)
    data = mujoco.MjData(model)
    site_id = model.site(target.site).id
    lower, upper = model.jnt_range[:7].T
    home = model.key_qpos[0, :7].copy()
    jacp, jacr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    best = None
    for restart in range(config["ik_restarts"]):
        data.qpos[:7] = home if restart == 0 else rng.uniform(lower + 0.05, upper - 0.05)
        data.qpos[7] = target.extension_m
        for _iteration in range(config["ik_iterations"]):
            mujoco.mj_forward(model, data)
            position_error = target.position_m - data.site_xpos[site_id]
            angle_error = rotation_error(HORIZONTAL_ROTATION, data.site_xmat[site_id].reshape(3, 3))
            position_norm = float(np.linalg.norm(position_error))
            angle_norm = float(np.linalg.norm(angle_error))
            if (
                position_norm < config["position_tolerance_m"]
                and angle_norm < config["orientation_tolerance_rad"]
            ):
                break
            mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
            # Rotation is scaled to a 0.15 m characteristic tool length.
            jac = np.vstack((jacp[:, :7], jacr[:, :7] * 0.15))
            error = np.concatenate((position_error, angle_error * 0.15))
            delta = jac.T @ np.linalg.solve(jac @ jac.T + 0.002**2 * np.eye(6), error)
            delta *= min(1.0, 0.12 / max(np.linalg.norm(delta), 1e-12))
            data.qpos[:7] = np.clip(data.qpos[:7] + delta, lower + 1e-5, upper - 1e-5)
        mujoco.mj_forward(model, data)
        position_norm = float(np.linalg.norm(target.position_m - data.site_xpos[site_id]))
        angle_norm = float(
            np.linalg.norm(rotation_error(HORIZONTAL_ROTATION, data.site_xmat[site_id].reshape(3, 3)))
        )
        contacts = contact_diagnostics(model, data, config["penetration_tolerance_m"])
        converged = (
            position_norm < config["position_tolerance_m"]
            and angle_norm < config["orientation_tolerance_rad"]
        )
        clear = not any(contact["reject"] for contact in contacts)
        record = {
            "target": target.name,
            "site": target.site,
            "target_position_m": target.position_m.tolist(),
            "qpos": data.qpos.tolist(),
            "position_error_m": position_norm,
            "orientation_error_rad": angle_norm,
            "ik_converged": converged,
            "endpoint_clear": clear,
            "feasible_endpoint": converged and clear,
            "contacts": contacts,
            "restart": restart,
            "iterations": _iteration + 1,
            "seed": seed,
        }
        rank = (not converged, not clear, position_norm + angle_norm * 0.15)
        if best is None or rank < best[0]:
            best = (rank, record)
        if converged and clear:
            break
    return best[1]
