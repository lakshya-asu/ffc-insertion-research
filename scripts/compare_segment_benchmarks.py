"""Compare the segmented cantilever against continuum and discrete references.

The discrete reference follows static moment balance for the actual authored
chain. It does not fit any simulator measurements or identify cable material.
"""

import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]


def main():
    rows = []
    for path in sorted((ROOT / "outputs").glob("e006-*/results.json")):
        r = json.loads(path.read_text())
        if "cantilever" not in r or "timestep_audit" not in r:
            continue
        cfg = r["configuration"]
        c = cfg["cable"]
        n = c["segments"] - 1
        spacing = c["length_m"] / c["segments"]
        line_load = c["mass_kg"] * 9.81 / c["length_m"]
        scale = c.get("angular_drive_gain_scale", 1)
        spring = c["bend_stiffness_nm_per_degree"] * 180 / math.pi * scale
        reference = line_load * spacing**3 / (2 * spring) * sum(i**3 for i in range(1, n + 1))
        sag = r["cantilever"]["simulated_tip_sag_m"]
        rows.append(
            {
                "run": path.parent.name,
                "dt_s": cfg["physics_dt_s"],
                "position_iterations": c.get("solver_iterations", 32),
                "angular_gain_multiplier": scale,
                "measured_sag_m": sag,
                "continuum_prediction_m": r["cantilever"]["beam_prediction_m"],
                "discrete_chain_prediction_m": reference,
                "ratio_to_discrete_chain": sag / reference,
                "nominal_material": scale == 1,
            }
        )
    nominal = sorted((r for r in rows if r["nominal_material"]), key=lambda r: r["dt_s"], reverse=True)
    fig, ax = plt.subplots(figsize=(9, 5), layout="constrained")
    labels = [f"{r['dt_s'] * 1000:g} ms / {r['position_iterations']} iterations" for r in nominal]
    values = [r["ratio_to_discrete_chain"] for r in nominal]
    bars = ax.barh(labels, values, color="#407d91")
    ax.bar_label(bars, fmt="%.3f", padding=5)
    ax.axvline(1, color="#222", linestyle="--", label="Static hinge-chain reference")
    ax.set_xscale("log")
    ax.set_xlim(0.8, 40)
    ax.set_xlabel("Simulated sag / predicted sag (log scale)")
    ax.set_title("Ribbon numerical compliance decreases with timestep refinement")
    ax.spines[["top", "right"]].set_visible(False)
    ax.legend(loc="lower right")
    fig.text(
        0.02, -0.015, "Same nominal mass and spring stiffness. No measured cable calibration.", fontsize=9
    )
    out = ROOT / "outputs/benchmarks"
    out.mkdir(exist_ok=True)
    fig.savefig(out / "segment-convergence.png", dpi=180, bbox_inches="tight")
    fig.savefig(out / "segment-convergence.svg", bbox_inches="tight")
    with (out / "segment-convergence.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (out / "segment-convergence.json").write_text(json.dumps(rows, indent=2))
    print(json.dumps(rows, indent=2))


if __name__ == "__main__":
    main()
