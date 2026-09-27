"""Compare measured full-span geometry with the independent static chain model."""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    result = json.loads((args.run / "results.json").read_text())
    samples = json.loads((args.run / "trajectory.json").read_text())
    check = result["cantilever"]
    cable = result["configuration"]["cable"]
    spacing = cable["length_m"] / cable["segments"]
    angles = np.r_[0, np.cumsum(check["reference_joint_angles_rad"])]
    nodes = np.vstack(([0, 0], np.cumsum(spacing * np.c_[np.cos(angles), -np.sin(angles)], axis=0)))
    centers = (nodes[:-1] + nodes[1:]) / 2
    centers -= centers[0]
    measured = np.asarray(samples[-1]["cable_xyz_m"])[:, [0, 2]]
    measured -= measured[0]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), layout="constrained")
    axes[0].plot(centers[:, 0] * 1000, centers[:, 1] * 1000, label="Independent static reference")
    axes[0].plot(measured[:, 0] * 1000, measured[:, 1] * 1000, ".-", label="Isaac Sim link centers")
    axes[0].set(xlabel="Distance from clamped link center (mm)", ylabel="Vertical displacement (mm)")
    axes[0].set_aspect("equal", adjustable="datalim")
    axes[0].legend()
    time = np.array([s["step"] for s in samples]) * result["configuration"]["physics_dt_s"]
    initial_height = samples[0]["cable_xyz_m"][0][2]
    sag = np.array([initial_height - s["cable_tip_xyz_m"][2] for s in samples])
    reference = check["nonlinear_hinge_chain_prediction_m"]
    tolerance = check.get("declared_static_shape_relative_tolerance", 0.20)
    axes[1].plot(time, sag * 1000, label="Measured tip sag")
    axes[1].axhline(reference * 1000, color="black", linestyle="--", label="Static reference")
    axes[1].axhspan(
        reference * (1 - tolerance) * 1000,
        reference * (1 + tolerance) * 1000,
        alpha=0.12,
        color="green",
        label=f"Declared ±{100 * tolerance:g}% gate",
    )
    axes[1].set(xlabel="Audited simulation time (s)", ylabel="Downward tip displacement (mm)")
    axes[1].legend()
    for ax in axes:
        ax.spines[["top", "right"]].set_visible(False)
        ax.grid(alpha=0.15)
    fig.suptitle(
        "150 mm cable: static numerical validation, uncalibrated material\n"
        "Body drag used only to settle this benchmark; no end stiffeners",
        fontsize=12,
    )
    for suffix in ("png", "svg"):
        fig.savefig(args.run / f"static-shape-comparison.{suffix}", dpi=180)


if __name__ == "__main__":
    main()
