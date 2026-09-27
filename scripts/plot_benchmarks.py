"""Plot audited native-shell convergence measurements against a beam reference."""

import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def main():
    rows = []
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), layout="constrained")
    for directory in sorted((ROOT / "outputs").glob("e003-cantilever-*")):
        path = directory / "results.json"
        if not path.exists():
            continue
        r = json.loads(path.read_text())
        if r.get("status") != "PASS" or "timestep_audit" not in r:
            continue
        cfg, result = r["configuration"], r["cantilever"]
        nx, ny = cfg["shell"]["longitudinal_elements"], cfg["shell"]["width_elements"]
        iterations = cfg["shell"].get("solver_iterations", 64)
        if not 1 <= iterations <= 255:
            continue
        dt_ms = cfg["physics_dt_s"] * 1000
        label = f"{nx}×{ny}, {dt_ms:g} ms, {iterations} iterations"
        samples = json.loads((directory / "trajectory.json").read_text())
        xyz = np.asarray(samples[-1]["cable_xyz_m"]).reshape(nx + 1, ny + 1, 3)
        sag = cfg["cable"]["start_xyz_m"][2] - xyz.mean(axis=1)[:, 2]
        axes[0].plot(np.linspace(0, 1, nx), sag[1:] / result["beam_prediction_m"], label=label)
        rows.append({"run": directory.name, "label": label, **result})
    x = np.linspace(0, 1, 100)
    axes[0].plot(x, x * x * (6 - 4 * x + x * x) / 3, "k--", label="Euler–Bernoulli reference")
    axes[0].set(
        xlabel="Fraction of free span",
        ylabel="Sag / predicted beam tip sag",
        title="Static profile; nominal homogeneous material",
    )
    axes[0].legend(fontsize=8)
    axes[1].barh([r["label"] for r in rows], [r["ratio_to_beam_prediction"] for r in rows], color="#407d91")
    axes[1].axvline(1, color="black", linestyle="--")
    axes[1].set(xlabel="Simulated / analytical tip sag", title="Solver and mesh sensitivity")
    for ax in axes:
        ax.spines[["top", "right"]].set_visible(False)
        ax.grid(axis="x", alpha=0.2)
    fig.suptitle("FFC shell benchmark — numerical checks, not hardware calibration", fontsize=13)
    out = ROOT / "outputs/benchmarks"
    out.mkdir(exist_ok=True)
    fig.savefig(out / "shell-convergence.png", dpi=180)
    fig.savefig(out / "shell-convergence.svg")
    with (out / "shell-convergence.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(out)


if __name__ == "__main__":
    main()
