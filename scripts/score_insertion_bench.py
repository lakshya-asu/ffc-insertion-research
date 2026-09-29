"""Offline geometry audit of the exploratory slot bench; never actor input."""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("run", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    report = json.loads((a.run / "report.json").read_text())
    frames = json.loads((a.run / "offline-cable-trace.json").read_text())
    profile = json.loads((a.run / "sections.json").read_text())
    trace = json.loads((a.run / "sensor-trace.json").read_text())
    tip = profile[0]
    local = np.array(
        [
            [x, -tip["length_m"] / 2, z]
            for x in [-tip["width_m"] / 2, tip["width_m"] / 2]
            for z in [-tip["thickness_m"] / 2, tip["thickness_m"] / 2]
        ]
    )
    geometry = report["fixture"]
    cx, cz = geometry.get("center_x_m", 0), geometry.get("center_z_m", 0.055)
    reviewed = []
    for frame in frames:
        q = frame["quaternions_wxyz"][0]
        rot = Rotation.from_quat([q[1], q[2], q[3], q[0]])
        corners = rot.apply(local) + frame["positions_m"][0]
        depths = geometry["mouth_y_m"] - corners[:, 1]
        fits = bool(
            np.all(abs(corners[:, 0] - cx) <= geometry["width_m"] / 2)
            and np.all(abs(corners[:, 2] - cz) <= geometry["gap_m"] / 2)
        )
        reviewed.append(
            {
                "time_s": frame["time_s"],
                "tip_depth_range_m": [float(depths.min()), float(depths.max())],
                "tip_cross_section_fits_channel": fits,
            }
        )
    last = reviewed[-1]
    entered = (
        last["tip_depth_range_m"][0] >= (0.002 if report.get("socket_reference") else 0.0025)
        and last["tip_depth_range_m"][1] <= geometry["length_m"]
        and last["tip_cross_section_fits_channel"]
    )
    final = report["final"].get("feed_command")
    loads = [row["feed_observation"]["fixture_force_n"] for row in trace if row["feed_observation"]]
    stop_index = next(
        (
            i
            for i, row in enumerate(trace)
            if row["feed_command"] and row["feed_command"]["state"] == "stopped"
        ),
        None,
    )
    stop_review = None
    if stop_index is not None:
        start = trace[stop_index]["feed_observation"]["travel_m"]
        travel = [row["feed_observation"]["travel_m"] - start for row in trace[stop_index:]]
        stop_review = {
            "observed_s": trace[-1]["time_s"] - trace[stop_index]["time_s"],
            "max_forward_excursion_m": max(travel),
            "net_displacement_m": travel[-1],
        }
    contact_review = None
    if report.get("socket_reference"):
        settings = report["socket_settings"]
        # Nose half-height is frozen in socket_reference.py revision 1.
        rest_z = cz + settings["assumed_contact_rest_top_relative_to_cable_center_m"] - 0.00005
        deflections = [
            rest_z - pose["xyz"][2] for frame in frames for pose in frame.get("contact_poses", {}).values()
        ]
        contact_review = {
            "sampled_contact_poses": len(deflections),
            "max_downward_deflection_m": max(deflections, default=0),
            "max_upward_excursion_m": max((-d for d in deflections), default=0),
            "configured_travel_m": settings["assumed_contact_max_deflection_m"],
            "scope": "Sampled rigid spring noses; not calibrated contact beam strain or damage",
        }
    result = {
        "score_revision": 4,
        "tip_crossed_mouth": last["tip_depth_range_m"][0] > 0,
        "tip_reached_review_depth": last["tip_depth_range_m"][0]
        >= (0.002 if report.get("socket_reference") else 0.0025),
        "final_backstop_overrun_m": max(0.0, last["tip_depth_range_m"][1] - geometry["length_m"]),
        "entry_flag_semantics": "Legacy flag tests strict containment, not merely entry",
        "contact_review": contact_review,
        "fixture": geometry,
        "entry_depth_review_threshold_m": 0.002 if report.get("socket_reference") else 0.0025,
        "post_stop_review": stop_review,
        "scope": "Offline assumed-channel geometry; neither seating nor Pi connector qualification",
        "case": report["case"],
        "controller": final,
        "tip_entered_assumed_channel": bool(entered),
        "completed_travel_without_valid_containment": bool(
            final and final["state"] == "travel_complete_unverified" and not entered
        ),
        "peak_10ms_fixture_load_n": max(loads, default=0),
        "final_geometry": last,
        "geometry_trace": reviewed,
        "seating_verified": False,
        "actor_receives_this_report": False,
    }
    a.output.parent.mkdir(parents=True, exist_ok=True)
    with a.output.open("x") as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps({key: value for key, value in result.items() if key != "geometry_trace"}))


if __name__ == "__main__":
    main()
