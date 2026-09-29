"""Generate frozen, assumption-based static response targets for simulator fitting."""

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from ffc.cable_spec import load_spec, values  # noqa: E402
from ffc.cantilever_reference import hinge_chain_reference  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    config_path = ROOT / "config/cables/world-model-v1.json"
    cfg = json.loads(config_path.read_text())
    spec_path = ROOT / cfg["cable_profile"]
    spec = load_spec(spec_path)
    if spec["sha256"] != cfg["cable_profile_sha256"]:
        raise ValueError("Reference profile changed; review and version the world model")
    v = values(spec)
    rows = []
    for split in ["fit", "validation"]:
        for length in cfg[f"{split}_lengths_m"]:
            count = round(length / cfg["segment_length_m"])
            for e_scale in cfg["equivalent_modulus_multipliers"]:
                ei = v["equivalent_young_pa"] * e_scale * v["mini_width_m"] * v["body_thickness_m"] ** 3 / 12
                for load in cfg["uniform_load_multipliers"]:
                    cable = {
                        "segments": count,
                        "stiffener_segments": 0,
                        "length_m": length,
                        "mass_kg": v["equivalent_density_kg_m3"]
                        * length
                        * v["mini_width_m"]
                        * v["body_thickness_m"]
                        * load,
                        "bend_stiffness_nm_per_degree": ei / (length / count) * math.pi / 180,
                    }
                    rows.append(
                        {
                            "split": split,
                            "length_m": length,
                            "modulus_multiplier": e_scale,
                            "uniform_load_multiplier": load,
                            "ei_nm2": ei,
                            "reference": hinge_chain_reference(cable),
                        }
                    )
    args.output.mkdir(parents=True, exist_ok=False)
    sources = {}
    for path in [config_path, spec_path, Path(__file__), ROOT / "src/ffc/cantilever_reference.py"]:
        raw = path.read_bytes()
        (args.output / path.name).write_bytes(raw)
        sources[str(path.relative_to(ROOT))] = hashlib.sha256(raw).hexdigest()
    report = {
        "world_model": cfg["id"],
        "scope": cfg["claim"],
        "sources": sources,
        "simulator_fitted": False,
        "responses": rows,
    }
    (args.output / "targets.json").write_text(json.dumps(report, indent=2) + "\n")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(9, 5))
    for split in ["fit", "validation"]:
        for length in cfg[f"{split}_lengths_m"]:
            curve = [r for r in rows if r["length_m"] == length and r["modulus_multiplier"] == 1]
            ax.plot(
                [r["uniform_load_multiplier"] for r in curve],
                [r["reference"]["nonlinear_hinge_chain_prediction_m"] * 1000 for r in curve],
                "o-" if split == "fit" else "s--",
                label=f"{length * 1000:g} mm / {split}",
            )
    ax.set(
        xlabel="Uniform load / nominal gravity load",
        ylabel="Predicted static tip sag (mm)",
        title="Assumed-world response targets / nominal equivalent stiffness",
    )
    ax.legend()
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(args.output / "reference-curves.png", dpi=180)
    plt.close(fig)
    print(
        json.dumps(
            {
                "responses": len(rows),
                "fit": sum(r["split"] == "fit" for r in rows),
                "validation": sum(r["split"] == "validation" for r in rows),
                "simulator_fitted": False,
            }
        )
    )


if __name__ == "__main__":
    main()
